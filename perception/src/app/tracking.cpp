//this is used to track hand
#include "Utils.h"
#include "ConfigParser.h"
#include "Renderer.h"
#include "Hand.h"
#include "PoseEstimator.h"
#include "tinyxml2.h"
#include "yaml-cpp/yaml.h"
#include "PoseHypo.h"

#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string TOPIC_NAME = "/head_camera/rgb/image_raw";
static const std::string DEPTH_TOPIC_NAME = "/head_camera/depth/image_raw";
static const std::string CAM_INFO_NAME = "/head_camera/depth/camera_info";

using namespace Eigen;


cv::Mat scene_bgr;
cv::Mat scene_depth;
bool initFlag;
Eigen::Matrix3f cam_info_K;

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& caminfo){
  // get the camera intrisic from cam info
  for (int i=0;i<9;i++)
  {
    cam_info_K(i/3,i%3) = caminfo->K[i];
  }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      scene_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg) {
    
      
    try {
        cv::Mat scene_depthRaw;
        scene_depthRaw = cv_bridge::toCvShare(msg, "16UC1")->image;

        if(!initFlag){
          scene_depth = cv::Mat::zeros(scene_depthRaw.rows, scene_depthRaw.cols, CV_32FC1);
          initFlag = true;
        }
        for (int u = 0; u < scene_depthRaw.rows; u++){
          for (int v = 0; v < scene_depthRaw.cols; v++)
          {
            unsigned short depthShort = scene_depthRaw.at<unsigned short>(u, v);  // 16bits
            if(msg->is_bigendian == 1)
              depthShort = ((depthShort & 0x00ff) <<  8) | ((depthShort & 0xff00) >> 8);
            float depth = (float)depthShort * SR300_DEPTH_UNIT;  //NOTE: 10000 for previously saved depth images
            if (depth>2.0 || depth<0.03)
            {
              scene_depth.at<float>(u, v) = 0.0;
            }
            else
            {
              scene_depth.at<float>(u, v) = depth;
            }
          }
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '16UC1'.",
                msg->encoding.c_str());
        ROS_ERROR("error: %s", e.what());
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::namedWindow("depth_view");

  ros::WallTime start_time, end_time;

  initFlag = false;

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  std::string config_dir;
  if (argc<2)
  {
    ROS_ERROR("please provide the configuration file!!!");
    return 0;
  }
  else
  {
    config_dir = std::string(argv[1]);
  }
  std::cout<<"Using config file: "<<config_dir<<std::endl;
  ConfigParser cfg(config_dir);

  // load the point pair features of the object
  std::map<std::vector<int>, std::vector<std::pair<int, int>>> ppfs;
  std::ifstream ppf_file(cfg.yml["ppf_path"].as<std::string>(), std::ios::binary);
  boost::archive::binary_iarchive iarch(ppf_file);
  iarch >> ppfs; // this is point pair features

  // load the object model and downsample it
  PointCloudSurfel::Ptr model(new PointCloudSurfel);
  pcl::io::loadPLYFile(cfg.object_model_path, *model);
  assert(model->points.size()>0);
  PointCloudSurfel::Ptr model001(new PointCloudSurfel);
  Utils::downsamplePointCloud(model, model001, 0.001);
  Utils::downsamplePointCloud(model001, model, 0.005);

  // initilize the object pose estimator
  PoseEstimator<pcl::PointSurfel> est(&cfg, model, model001, cfg.cam_intrinsic);

  // Compute min gripper dist
  {
    pcl::PointSurfel minPt, maxPt;
    pcl::getMinMax3D(*model001, minPt, maxPt);
    cfg.gripper_min_dist = 0.01 * std::min(std::min(std::abs(minPt.x - maxPt.x), std::abs(minPt.y - maxPt.y)), std::abs(minPt.z - maxPt.z));
  }
  
  tf::TransformListener tf_listener;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_rgb = it.subscribe(TOPIC_NAME, 1,
          imageCallback);
  image_transport::Subscriber sub_depth = it.subscribe(DEPTH_TOPIC_NAME, 1,
          imageDepthCallback);

  ros::Subscriber caminfoSub = nh.subscribe(CAM_INFO_NAME, 10, camInfoCallback,
                    ros::TransportHints().tcpNoDelay());

  ros::Publisher handbase_pub = nh.advertise<PointCloudRGBNormal> ("handbase_points", 1);
  ros::Publisher object_only_pointcloud_pub = nh.advertise<PointCloudRGBNormal> ("object_only", 1);
  ros::Publisher predicted_hand_pointcloud_pub = nh.advertise<PointCloudRGBNormal> ("predicted_hand", 1);
  ros::Publisher visible_pointcloud_pub = nh.advertise<PointCloudRGBNormal> ("visible_part_of_hand", 1);
  ros::Publisher predicted_object_pointcloud_pub = nh.advertise<PointCloudSurfel> ("predicted_object", 1);

  HandT42 hand(&cfg, cfg.cam_intrinsic);

  // tf::TransformBroadcaster object_pose_transform_broad_caster;
  // tf::Transform object_pose_transform;

  ros::Rate loop_rate(1);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    if(scene_bgr.rows == 0 || scene_depth.rows == 0){
      std::cout << "no image in!\n";
      continue;
    }

    tf::StampedTransform tf_cam2gripper;
    tf_listener.waitForTransform("/head_camera_rgb_optical_frame", "/gripper_link", ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform("/head_camera_rgb_optical_frame", "/gripper_link", ros::Time(0), tf_cam2gripper);
    Eigen::Affine3d handbase_in_cam_in_affine3d;
    tf::transformTFToEigen(tf_cam2gripper, handbase_in_cam_in_affine3d);

    // handbase in left cam
    Eigen::Matrix4f handbase_in_cam = handbase_in_cam_in_affine3d.cast<float>().matrix();

    start_time = ros::WallTime::now();

    // generate the rgb depth of scene
    PointCloudRGBNormal::Ptr scene_rgb(new PointCloudRGBNormal);
    Utils::convert3dOrganizedRGB<pcl::PointXYZRGBNormal>(scene_depth, scene_bgr, cam_info_K, scene_rgb);

    Utils::calNormalIntegralImage<pcl::PointXYZRGBNormal>(scene_rgb, -1, 0.02, 10, true);

    // pcl filter to filter out point that is too far or too close
    {
      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.03, 2.0);
      pass.filter(*scene_rgb);
    }

    PointCloudRGBNormal::Ptr scene_organized(new PointCloudRGBNormal);
    pcl::copyPointCloud(*scene_rgb, *scene_organized);
    Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(scene_rgb, scene_rgb, 0.001);

    // filter out point not in the hand link bounding

    Eigen::Matrix4f cam_in_handbase = handbase_in_cam.inverse();
    pcl::transformPointCloudWithNormals(*scene_rgb, *scene_rgb, cam_in_handbase);

    {
      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.08, 0.08);
      pass.filter(*scene_rgb);

      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-0.15, 0.08);
      pass.filter(*scene_rgb);

      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-0.07, 0.07);
      pass.filter(*scene_rgb);

    }

    pcl::transformPointCloudWithNormals(*scene_rgb, *scene_rgb, cam_in_handbase.inverse());

    // publish the pointcloud of hand
    scene_rgb->header.frame_id = "/head_camera_rgb_optical_frame";
    handbase_pub.publish(scene_rgb); // the scene_rgb is correct

    // scene_rgb is in camera base
    hand.setCurScene(scene_depth, scene_organized, scene_rgb, handbase_in_cam);

    const float finger_min_match = cfg.yml["hand_match"]["finger_min_match"].as<float>();
    const float finger_dist_thres = cfg.yml["hand_match"]["finger_dist_thres"].as<float>();
    const float finger_normal_angle = cfg.yml["hand_match"]["finger_normal_angle"].as<float>();

    if (cam_in_handbase(1, 3) > 0) // Cam on the right side of hand
    {
      hand.matchOneComponentPSO("r_gripper_finger_link", 0, 0.05, false, finger_dist_thres, finger_normal_angle, finger_min_match);
      hand.matchOneComponentPSO("l_gripper_finger_link", -0.05, 0, false, finger_dist_thres, finger_normal_angle, finger_min_match);
    }
    else
    {
      hand.matchOneComponentPSO("l_gripper_finger_link", -0.05, 0, false, finger_dist_thres, finger_normal_angle, finger_min_match);
      hand.matchOneComponentPSO("r_gripper_finger_link", 0, 0.05, false, finger_dist_thres, finger_normal_angle, finger_min_match);
    }

    // hand.adjustHandHeight();
    hand.makeHandCloud();
    
    PointCloudRGBNormal::Ptr check_hand(new PointCloudRGBNormal);
    pcl::transformPointCloudWithNormals(*(hand._hand_cloud), *check_hand, hand._handbase_in_cam);
    
    // extract the point cloud being to the object
    PointCloudSurfel::Ptr object1(new PointCloudSurfel);
    PointCloudSurfel::Ptr tmp_scene(new PointCloudSurfel);
    pcl::copyPointCloud(*scene_rgb, *tmp_scene);
    const float near_dist = cfg.yml["near_hand_dist"].as<float>();
    hand.removeSurroundingPointsAndAssignProbability<pcl::PointSurfel, true>(tmp_scene, object1, near_dist * near_dist);

    PointCloudRGBNormal::Ptr cloud_withouthand_raw(new PointCloudRGBNormal);
    pcl::copyPointCloud(*object1, *cloud_withouthand_raw);

    Utils::calNormalMLS<pcl::PointSurfel>(object1, 0.003);

    PointCloudSurfel::Ptr object_segment(new PointCloudSurfel);
    pcl::copyPointCloud(*object1, *object_segment);
    Utils::downsamplePointCloud<pcl::PointSurfel>(object_segment, object_segment, 0.003);
    Utils::removeAllNaNFromPointCloud(object_segment);
    for (auto &pt : object_segment->points)
    {
      pcl::flipNormalTowardsViewpoint(pt, 0, 0, 0, pt.normal[0], pt.normal[1], pt.normal[2]);
    }

    // build the kdtree of the object in scene
    pcl::KdTreeFLANN<pcl::PointSurfel> kdtree;
    kdtree.setInputCloud(object1);

    // calculate the confidence of point of cloud
    for (auto &pt : object_segment->points)
    {
      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      if (kdtree.nearestKSearch(pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0)
      {
        pt.confidence = 0;
        continue;
      }
      pt.confidence = object1->points[pointIdxNKNSearch[0]].confidence;
    }

    PointCloudSurfel::Ptr scene_003(new PointCloudSurfel);
    pcl::copyPointCloud(*scene_rgb, *scene_003);
    Utils::downsamplePointCloud<pcl::PointSurfel>(scene_003, scene_003, 0.003);

    
    est.setCurScene(scene_003, cloud_withouthand_raw, object_segment, scene_bgr, scene_depth);
    est.registerHandMesh(&hand);
    est.registerMesh(cfg.object_mesh_path, "object", Eigen::Matrix4f::Identity());
    bool succeed = est.runSuper4pcs(ppfs);

    if (!succeed)
    {
      printf("No pose found...\n");
    }
    else{
      printf("pose found...\n");

      est.clusterPoses(30, 0.015, true);
      est.refineByICP();
      est.clusterPoses(5, 0.003, false);
      est.rejectByCollisionOrNonTouching(&hand);
      est.rejectByRender(cfg.pose_estimator_wrong_ratio, &hand);
      PoseHypo best(-1);
      est.selectBest(best);
      Eigen::Matrix4f model2scene = best._pose;
      // std::cout << "best tf:\n"
      //           << model2scene << "\n\n";

      PointCloudSurfel::Ptr predicted_model(new PointCloudSurfel);
      pcl::transformPointCloud(*model, *predicted_model, model2scene);
      predicted_model->header.frame_id = "/head_camera_rgb_optical_frame";
      predicted_object_pointcloud_pub.publish(predicted_model);

      // Eigen::Quaterniond eigen_quat(model2scene.block<3,3>(0,0).cast<double>());
      // Eigen::Vector3d eigen_trans(model2scene.block<3,1>(0,3).cast<double>());


      // tf::Quaternion tf_quat;
      // tf::Vector3 tf_trans;
      // tf::quaternionEigenToTF(eigen_quat, tf_quat);
      // tf::vectorEigenToTF(eigen_trans, tf_trans);

      // object_pose_transform.setOrigin(tf_trans);
      // object_pose_transform.setRotation(tf_quat);
      // object_pose_transform_broad_caster.sendTransform(tf::StampedTransform(object_pose_transform, ros::Time::now(), "/head_camera_rgb_optical_frame", "/object"));

      object1->header.frame_id = "/head_camera_rgb_optical_frame";
      object_only_pointcloud_pub.publish(object1);
    }
    end_time = ros::WallTime::now();

    ROS_INFO_STREAM("Exec time(ms): " << (end_time - start_time).toNSec()*1e-6);
    

    check_hand->header.frame_id = "/head_camera_rgb_optical_frame";
    predicted_hand_pointcloud_pub.publish(check_hand);

    hand._visible_set->header.frame_id = "/gripper_link";
    visible_pointcloud_pub.publish(hand._visible_set);

    hand.reset();
    est.reset();

    cv::imshow("view", scene_bgr);
    cv::imshow("depth_view", scene_depth);
    cv::waitKey(1);
  }
  
  cv::destroyWindow("view");
  cv::destroyWindow("depth_view");

  return 0;

}