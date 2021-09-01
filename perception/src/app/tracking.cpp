//this is used to track hand
#include "Utils.h"
#include "ConfigParser.h"
#include "Renderer.h"
#include "Hand.h"
#include "PoseEstimator.h"
#include "tinyxml2.h"
#include "yaml-cpp/yaml.h"
#include "PoseHypo.h"
#include "ParticleFilter.h"
#include "MarkerHelper.h"

#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>

#include <icra20_manipulation_pose/SearchObject.h>
#include <rail_manipulation_msgs/SegmentObjects.h>

#include <random>
#include <math.h>


static const std::string TOPIC_NAME = "/head_camera/rgb/image_raw";
static const std::string DEPTH_TOPIC_NAME = "/head_camera/depth/image_raw";
static const std::string CAM_INFO_NAME = "/head_camera/depth/camera_info";

using namespace Eigen;

cv::Mat scene_bgr;
cv::Mat scene_depth;

bool initFlag;
Eigen::Matrix3f cam_info_K;

Eigen::Matrix4f model2hand;
Eigen::Matrix4f predicted_hand_pose;
Eigen::Matrix4f predicted_table_place_pose;
Eigen::Matrix4f predicted_table_center_pose;
Eigen::Matrix4f predicted_pose_in_hand;
bool isTracking;
int actiontype;
unsigned int trackstamp;

class Table_cloud_receiver{
  public:
    Table_cloud_receiver(): tablepointcloud(new PointCloud){
      gotData = false;
      centroidx = 0.0;
      centroidy = 0.0;
      centroidz = 0.0;
      orientationx = 0.0;
      orientationy = 0.0;
      orientationz = 0.0;
      orientationw = 0.0;
    }
    // callback function for table detection
    void tableCallback(const rail_manipulation_msgs::SegmentedObject::ConstPtr &table){
      ROS_INFO("receive table data!");
      gotData = true;
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(table->point_cloud, pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2, *tablepointcloud);

      centroidx = table->centroid.x;
      centroidy = table->centroid.y;
      centroidz = table->centroid.z;
      centerx = table->center.x;
      centery = table->center.y;
      centerz = table->center.z;
      orientationx = table->orientation.x;
      orientationy = table->orientation.y;
      orientationz = table->orientation.z;
      orientationw = table->orientation.w;
    }
    PointCloud::Ptr tablepointcloud;
    float centroidx, centroidy, centroidz;
    float centerx, centery, centerz;
    float orientationx, orientationy, orientationz, orientationw; 
    bool gotData;
};

// softmax function
void softmax(std::vector<float> &input) {

	int i;
	double m, sum, constant;
  int size = input.size();

	m = -INFINITY;
	for (i = 0; i < size; ++i) {
		if (m < input[i]) {
			m = input[i];
		}
	}

	sum = 0.0;
	for (i = 0; i < size; ++i) {
		sum += exp(input[i] - m);
	}

	constant = m + log(sum);
	for (i = 0; i < size; ++i) {
		input[i] = exp(input[i] - constant);
	}

}


// callback function for camera information
void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& caminfo){
  // get the camera intrisic from cam info
  for (int i=0;i<9;i++)
  {
    cam_info_K(i/3,i%3) = caminfo->K[i];
  }
}

// image callback function
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      scene_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

// callback function for depth
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


// This is a thread to keep publishing the object's pose
void publish_object_state(int* publish_rate){

  ros::NodeHandle n;
  
  ros::Rate loop_rate(*publish_rate);

  tf::TransformBroadcaster object_pose_transform_broad_caster;
  tf::Transform object_pose_transform;
  tf::Transform hand_pose_transform;
  tf::Transform predicted_table_place_pose_transform;
  tf::Transform predicted_table_center_pose_transform;

  ros::Publisher trackstamp_pub = n.advertise<std_msgs::UInt16>("trackstamp", 10);
  
  while(ros::ok()){
    loop_rate.sleep();
    if(model2hand.isIdentity() || !isTracking){
      continue;
    }
    Eigen::Quaterniond eigen_quat(model2hand.block<3,3>(0,0).cast<double>());
    Eigen::Vector3d eigen_trans(model2hand.block<3,1>(0,3).cast<double>());

    tf::Quaternion tf_quat;
    tf::Vector3 tf_trans;
    tf::quaternionEigenToTF(eigen_quat, tf_quat);
    tf::vectorEigenToTF(eigen_trans, tf_trans);

    object_pose_transform.setOrigin(tf_trans);
    object_pose_transform.setRotation(tf_quat);
    object_pose_transform_broad_caster.sendTransform(tf::StampedTransform(object_pose_transform, ros::Time::now(), "/gripper_link", "/object"));

    if(actiontype == 2 || actiontype == 3 || actiontype == 4){
      if(!predicted_hand_pose.isIdentity() && !predicted_table_place_pose.isIdentity()){
        // broadcast the pose of predicted hand
        Eigen::Quaterniond hand_quat(predicted_hand_pose.block<3,3>(0,0).cast<double>());
        Eigen::Vector3d hand_trans(predicted_hand_pose.block<3,1>(0,3).cast<double>());

        tf::quaternionEigenToTF(hand_quat, tf_quat);
        tf::vectorEigenToTF(hand_trans, tf_trans);

        hand_pose_transform.setOrigin(tf_trans);
        hand_pose_transform.setRotation(tf_quat);
        object_pose_transform_broad_caster.sendTransform(tf::StampedTransform(hand_pose_transform, ros::Time::now(), "/head_camera_rgb_optical_frame", "/predicted_hand"));

        //  broadcast the pose of predicted place pose
        Eigen::Quaterniond table_quat(predicted_table_place_pose.block<3,3>(0,0).cast<double>());
        Eigen::Vector3d table_trans(predicted_table_place_pose.block<3,1>(0,3).cast<double>());

        tf::quaternionEigenToTF(table_quat, tf_quat);
        tf::vectorEigenToTF(table_trans, tf_trans);

        predicted_table_place_pose_transform.setOrigin(tf_trans);
        predicted_table_place_pose_transform.setRotation(tf_quat);
        object_pose_transform_broad_caster.sendTransform(tf::StampedTransform(predicted_table_place_pose_transform, ros::Time::now(), "/base_link", "/predicted_table_place"));
    
        // broadcast the predicted pose of table
        Eigen::Quaterniond table_center_quat(predicted_table_center_pose.block<3,3>(0,0).cast<double>());
        Eigen::Vector3d table_center_trans(predicted_table_center_pose.block<3,1>(0,3).cast<double>());

        tf::quaternionEigenToTF(table_center_quat, tf_quat);
        tf::vectorEigenToTF(table_center_trans, tf_trans);

        predicted_table_center_pose_transform.setOrigin(tf_trans);
        predicted_table_center_pose_transform.setRotation(tf_quat);
        object_pose_transform_broad_caster.sendTransform(tf::StampedTransform(predicted_table_center_pose_transform, ros::Time::now(), "/base_link", "/predicted_table_center"));
      }
    }
    std_msgs::UInt16 tstamp;
    tstamp.data = trackstamp;
    trackstamp_pub.publish(tstamp);
  }
}

// here is the server function to receive the command
bool searchObjectTrig(icra20_manipulation_pose::SearchObject::Request &req,
        icra20_manipulation_pose::SearchObject::Response &res){

  predicted_pose_in_hand.setZero();
  predicted_hand_pose.setIdentity();
  predicted_table_place_pose.setIdentity();
  predicted_table_center_pose.setIdentity();

  if(req.startsearch){
    isTracking = true;
    actiontype = req.actiontype;
    if(req.actiontype == 2 || req.actiontype == 3 || req.actiontype == 4){
      
      if(req.poseInHand.position.x != 0 || req.poseInHand.position.y != 0 ||
         req.poseInHand.position.z != 0 || req.poseInHand.orientation.x != 0 ||
         req.poseInHand.orientation.y != 0 || req.poseInHand.orientation.z != 0 ||
         req.poseInHand.orientation.w != 0 ){ // if there is a init pose given
        Eigen::Matrix3f R = Eigen::Quaternionf(req.poseInHand.orientation.w, 
                                               req.poseInHand.orientation.x, 
                                               req.poseInHand.orientation.y, 
                                               req.poseInHand.orientation.z).toRotationMatrix();
        predicted_pose_in_hand.block<3,3>(0,0) = R;

        predicted_pose_in_hand(0,3) = req.poseInHand.position.x;
        predicted_pose_in_hand(1,3) = req.poseInHand.position.y;
        predicted_pose_in_hand(2,3) = req.poseInHand.position.z;
        predicted_pose_in_hand(3,3) = 1.0;
      }
    }
  }else{
    isTracking = false;
    actiontype = 0;
  }
  model2hand.setIdentity();
  
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;
  // cv::namedWindow("color");
  // cv::namedWindow("depth_view");

  // initialize all variables
  model2hand.setIdentity();
  predicted_pose_in_hand.setZero();
  predicted_hand_pose.setIdentity();
  predicted_table_place_pose.setIdentity();
  predicted_table_center_pose.setIdentity();
  initFlag = false;
  isTracking = false;
  bool isObjectTracking = false;
  actiontype = 0;
  trackstamp = 0;
  bool istableTracked = false;
  Eigen::Matrix4f table_offset;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> pose_particles;
  pose_particles.reserve(30);
  table_offset.setIdentity();
  
  // action type:
  // 0: null
  // 1: in-hand object detection and pose estimation
  // 2: placing
  // 3: grasping
  // 4: fixtureless fixturing regrasping

  // spawn another thread
  int rate_b = 5;
  boost::thread thread_b(publish_object_state, &rate_b);

  // used to measure the time cost
  ros::WallTime start_time, end_time;

  // initialize the pcl
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

  // initialize tools of ros
  // init tf listener
  tf::TransformListener tf_listener;
  // init image receiver
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_rgb = it.subscribe(TOPIC_NAME, 1,
          imageCallback);
  image_transport::Subscriber sub_depth = it.subscribe(DEPTH_TOPIC_NAME, 1,
          imageDepthCallback);
  ros::Subscriber caminfoSub = nh.subscribe(CAM_INFO_NAME, 10, camInfoCallback,
                    ros::TransportHints().tcpNoDelay());
  Table_cloud_receiver table_cloud_receiver;
  ros::Subscriber tableSub = nh.subscribe("/table_searcher/segmented_table", 1, &Table_cloud_receiver::tableCallback, &table_cloud_receiver);

  // init publisher for pointclouds
  ros::Publisher handbase_pub = nh.advertise<PointCloudRGBNormal> ("handbase_points", 1); // pointcloud around the hand
  ros::Publisher object_only_pointcloud_pub = nh.advertise<PointCloudRGBNormal> ("object_only", 1); // pointcloud without hand
  ros::Publisher predicted_hand_pointcloud_pub = nh.advertise<PointCloudRGBNormal> ("predicted_hand", 1); // predicted hand
  ros::Publisher visible_pointcloud_pub = nh.advertise<PointCloudRGBNormal> ("visible_part_of_hand", 1); // visible part of palm

  // used for debug
  ros::Publisher test1_pub = nh.advertise<PointCloudRGBNormal> ("test1", 1);
  ros::Publisher test2_pub = nh.advertise<PointCloudRGBNormal> ("test2", 1);
  ros::Publisher table_pub = nh.advertise<PointCloud> ("table_debug", 1);

  // marker helper
  MarkerHelper marker_helper(nh, "package://in_hand_manipulation/scripts/objects/cuboid.stl");

  // estimated pose
  ros::Publisher predicted_object_pointcloud_pub = nh.advertise<PointCloudSurfel> ("predicted_object", 1);
  // init service server with function 'searchObjectTrig'
  ros::ServiceServer service = nh.advertiseService("searchObject", searchObjectTrig);

  // init hand
  HandT42 hand(&cfg, cfg.cam_intrinsic, 640, 480);

  // init particle filter
  ParticleFilter partcile_filter(100);

  ros::Rate loop_rate(10);

  // main loop
  // all events of loop according to the action type
  // 0. no tracking
  //    if isTracking is set to false, then no need to run any detect and track for saving computing resource
  // 1. in-hand detection
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();

    // event 0
    if(!isTracking){
      ROS_INFO("not tracking!");
      continue;
    }

    if(!table_cloud_receiver.gotData){
      ROS_INFO("need table information!");
      continue;
    }

    // if no image in, then continue to wait
    if(scene_bgr.rows == 0 || scene_depth.rows == 0){
      std::cout << "no image in!\n";
      continue;
    }

    // extract the hand pose in camera frame
    tf::StampedTransform tf_cam2gripper;
    tf_listener.waitForTransform("/head_camera_rgb_optical_frame", "/gripper_link", ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform("/head_camera_rgb_optical_frame", "/gripper_link", ros::Time(0), tf_cam2gripper);
    Eigen::Affine3d handbase_in_cam_in_affine3d;
    tf::transformTFToEigen(tf_cam2gripper, handbase_in_cam_in_affine3d);

    // handbase in cam
    Eigen::Matrix4f handbase_in_cam = handbase_in_cam_in_affine3d.cast<float>().matrix();

    start_time = ros::WallTime::now();

    // generate the pointcloud from the rgb depth of scene
    PointCloudRGBNormal::Ptr scene_rgb(new PointCloudRGBNormal);
    Utils::convert3dOrganizedRGB<pcl::PointXYZRGBNormal>(scene_depth, scene_bgr, cam_info_K, scene_rgb);
    Utils::calNormalIntegralImage<pcl::PointXYZRGBNormal>(scene_rgb, -1, 0.02, 10, true);

    // pcl filter to filter out point that is too far or too close
    {
      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.03, 2.0);
      pass.setKeepOrganized(true);
      pass.filter(*scene_rgb);
    }

    PointCloudRGBNormal::Ptr scene_organized_with_table(new PointCloudRGBNormal);
    pcl::copyPointCloud(*scene_rgb, *scene_organized_with_table);

    // filter out point not in the hand link bounding
    Eigen::Matrix4f cam_in_handbase = handbase_in_cam.inverse();
    pcl::transformPointCloudWithNormals(*scene_rgb, *scene_rgb, cam_in_handbase);

    {
      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.08, 0.08);
      pass.setKeepOrganized(true);
      pass.filter(*scene_rgb);

      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-0.145, 0.08);
      pass.setKeepOrganized(true);
      pass.filter(*scene_rgb);

      pass.setInputCloud(scene_rgb);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-0.07, 0.07);
      pass.setKeepOrganized(true);
      pass.filter(*scene_rgb);
    }

    pcl::transformPointCloudWithNormals(*scene_rgb, *scene_rgb, cam_in_handbase.inverse());

    PointCloudRGBNormal::Ptr scene_organized(new PointCloudRGBNormal);
    pcl::copyPointCloud(*scene_rgb, *scene_organized);

    PointCloudRGBNormal::Ptr occluding_edge(new PointCloudRGBNormal);
    Utils::get3DEdge(scene_rgb, occluding_edge);

    occluding_edge->header.frame_id = "/head_camera_rgb_optical_frame";
    occluding_edge->header.stamp = ros::Time::now().toNSec()/1e3;
    test1_pub.publish(occluding_edge);

    Utils::downsamplePointCloud<pcl::PointXYZRGBNormal>(scene_rgb, scene_rgb, 0.001);


    // filter out the pointcloud according to the event
    // need to process on scene_organized
    if(actiontype == 2 || actiontype == 3 || actiontype == 4){
      // event 2,3,4 require table tracking
      // 1. get the camera pose 
      // 2. find the table in camera frame
      // 3. use icp to track the table
      // 

      // get camera pose in base frame
      tf::StampedTransform tf_base2cam;
      tf_listener.waitForTransform("/base_link", "/head_camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
      tf_listener.lookupTransform("/base_link", "/head_camera_rgb_optical_frame", ros::Time(0), tf_base2cam);
      Eigen::Affine3d cam_in_base_in_affine3d;
      tf::transformTFToEigen(tf_base2cam, cam_in_base_in_affine3d);

      // cam in base
      Eigen::Matrix4f cam_in_base = cam_in_base_in_affine3d.cast<float>().matrix();

      // get the table pointcloud in camera frame
      // need to downsample the pointcloud for icp
      PointCloud::Ptr table_in_cam(new PointCloud);
      pcl::transformPointCloud(*(table_cloud_receiver.tablepointcloud), *table_in_cam, cam_in_base.inverse());
      Utils::downsamplePointCloud<pcl::PointXYZ>(table_in_cam, table_in_cam, 0.01);

      PointCloud::Ptr scene_pointcloud_XYZ(new PointCloud);
      pcl::copyPointCloud(*scene_organized_with_table, *scene_pointcloud_XYZ);
      Utils::downsamplePointCloud<pcl::PointXYZ>(scene_pointcloud_XYZ, scene_pointcloud_XYZ, 0.01);

      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputSource(table_in_cam);
      icp.setInputTarget(scene_pointcloud_XYZ);
      icp.setMaxCorrespondenceDistance(0.05);
      icp.setMaximumIterations(15);

      PointCloud Final;
      icp.align(Final);
      table_offset = icp.getFinalTransformation();

      if(icp.hasConverged()){
        istableTracked = true;
        ROS_INFO("table is tracked!");
        pcl::transformPointCloud(*table_in_cam, *table_in_cam, table_offset);

        table_in_cam->header.frame_id = "/head_camera_rgb_optical_frame";
        table_in_cam->header.stamp = ros::Time::now().toNSec()/1e3;
        table_pub.publish(table_in_cam);

        // update the predicted hand and table poses
        Eigen::Matrix3f R = Eigen::Quaternionf(table_cloud_receiver.orientationw, table_cloud_receiver.orientationx, table_cloud_receiver.orientationy, table_cloud_receiver.orientationz).toRotationMatrix();
        predicted_table_place_pose.block<3,3>(0,0) = R;
        predicted_table_center_pose.block<3,3>(0,0) = R;

        predicted_table_place_pose(0,3) = table_cloud_receiver.centroidx;
        predicted_table_place_pose(1,3) = table_cloud_receiver.centroidy;
        predicted_table_place_pose(2,3) = table_cloud_receiver.centroidz;

        predicted_table_center_pose(0,3) = table_cloud_receiver.centerx;
        predicted_table_center_pose(1,3) = table_cloud_receiver.centery;
        predicted_table_center_pose(2,3) = table_cloud_receiver.centerz;

        predicted_table_place_pose = predicted_table_place_pose * table_offset.inverse();

        predicted_table_center_pose = predicted_table_center_pose * table_offset.inverse();

        Eigen::Matrix4f filterTransform = predicted_table_place_pose.inverse() * cam_in_base;

        pcl::transformPointCloudWithNormals(*scene_rgb, *scene_rgb, filterTransform);

        {
          pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
          pass.setInputCloud(scene_rgb);
          pass.setFilterFieldName("z");
          pass.setFilterLimits(0.0004, 1.0);
          pass.filter(*scene_rgb);
        }

        pcl::transformPointCloudWithNormals(*scene_rgb, *scene_rgb, filterTransform.inverse());

        // PointCloudRGBNormal::Ptr scene_organized_check(new PointCloudRGBNormal);
        // pcl::copyPointCloud(*scene_organized, *scene_organized_check);
        pcl::transformPointCloudWithNormals(*scene_organized, *scene_organized, filterTransform);

        {
          pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
          pass.setInputCloud(scene_organized);
          pass.setFilterFieldName("z");
          pass.setFilterLimits(0.0004, 1.0);
          pass.setKeepOrganized(true);
          pass.filter(*scene_organized);
        }

        pcl::transformPointCloudWithNormals(*scene_organized, *scene_organized, filterTransform.inverse());
        
      }else{
        table_offset.setIdentity();
        istableTracked = false;
      }
    }

    // cv::Mat scene_depth_check = cv::Mat::zeros(scene_depth.rows, scene_depth.cols, CV_32FC1);
    Utils::convert2dDepth<pcl::PointXYZRGBNormal>(scene_organized, cam_info_K, scene_depth);
    cv::imshow("depth_view", scene_depth);
    cv::waitKey(5);

    // // publish the pointcloud of hand
    scene_rgb->header.frame_id = "/head_camera_rgb_optical_frame";
    handbase_pub.publish(scene_rgb);

    // scene_rgb is in camera base, search the hand base and track it
    hand.setCurScene(scene_depth, scene_organized, scene_rgb, handbase_in_cam);

    // if the hand is not in camera view, then continue
    if(! hand.in_cam){
      // reset the hand and estimator      
      hand.reset();
      continue;
    }

    // track fingers according to the handbase
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

    hand.makeHandCloud();
    
    PointCloudRGBNormal::Ptr predicted_hand(new PointCloudRGBNormal);
    pcl::transformPointCloudWithNormals(*(hand._hand_cloud), *predicted_hand, hand._handbase_in_cam);

    end_time = ros::WallTime::now();

    ROS_INFO_STREAM("hand tracking time(ms): " << (end_time - start_time).toNSec()*1e-6);
    
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
    Eigen::Matrix4f model2scene;

    if(!isObjectTracking){
      // run super 4pcs
      bool succeed;
      if(predicted_pose_in_hand.isZero()){
        succeed = est.runSuper4pcs(ppfs, Eigen::Matrix4f::Identity());
      }else{
        succeed = est.runSuper4pcs(ppfs, hand._handbase_in_cam * predicted_pose_in_hand);
      }
      
      if(succeed){
        est.clusterPoses(30, 0.015, true);
        est.refineByICP();
        est.clusterPoses(5, 0.003, false);
        est.rejectByCollisionOrNonTouching(&hand);
        est.rejectByRender(cfg.pose_estimator_wrong_ratio, &hand);

        // select the best and publish
        PoseHypo best(-1);
        est.selectBest(best);//, &hand);

        // generate the particles
        partcile_filter.sampling(best._pose);
        pose_particles.clear();
        for(int n = 0; n < partcile_filter.size(); n++)
          pose_particles.push_back(partcile_filter.get(n));

        model2scene = best._pose;

        isObjectTracking = true;
      }
      else{
        isObjectTracking = false;
      }
    }
    else{

      est.runUpdate(pose_particles);
      est.refineByICP();
      est.clusterPoses(2, 0.0005, true);
      est.rejectByCollisionOrNonTouching(&hand);
      est.rejectByRender(cfg.pose_estimator_wrong_ratio, &hand);

      pose_particles.clear();

      std::vector<float> hypo_wrong_ratios;

      PoseHypo best(-1);
      est.selectBest(best);//, &hand);

      for(int n = 0; n < est.getNumOfHypos(); n++){
        PoseHypo selectedHypo(-1);
        est.selectIndex(selectedHypo, n);
        // if the wrong ratio is too high, then ignore it
        if(selectedHypo._wrong_ratio > 5.0 || selectedHypo._lcp_score < 100)
          continue;
        pose_particles.push_back(selectedHypo._pose);
        hypo_wrong_ratios.push_back(-selectedHypo._wrong_ratio);
      }

      if(pose_particles.size() == 0)
        isObjectTracking = false;
      else{
        softmax(hypo_wrong_ratios);

        int maxWeightIndex = std::max_element(hypo_wrong_ratios.begin(), hypo_wrong_ratios.end()) - hypo_wrong_ratios.begin();
        model2scene = pose_particles[maxWeightIndex];

        marker_helper.publishMarkers(pose_particles, hypo_wrong_ratios);

        // need to resampling
        partcile_filter.resample(pose_particles, hypo_wrong_ratios);
        pose_particles.clear();
        for(int n = 0; n < partcile_filter.size(); n++)
          pose_particles.push_back(partcile_filter.get(n));
      }
    }

    if(isObjectTracking){
      printf("object is tracked!");

      // publish the predicted object pointcloud in camera frame
      PointCloudSurfel::Ptr predicted_model(new PointCloudSurfel);
      pcl::transformPointCloud(*model, *predicted_model, model2scene);
      predicted_model->header.frame_id = "/head_camera_rgb_optical_frame";
      predicted_object_pointcloud_pub.publish(predicted_model);

      // publish the pointcloud does not belong to the hand in camera frame
      object_segment->header.frame_id = "/head_camera_rgb_optical_frame";
      object_only_pointcloud_pub.publish(object_segment);

      // calculate the predicted object pose to hand
      model2hand = hand._handbase_in_cam.inverse() * model2scene;
      if(actiontype == 2 || actiontype == 3 || actiontype == 4){

        predicted_pose_in_hand = model2hand;
      }
      // // publish the markers
      // marker_helper.publishMarkers(pose_particles);
    }
    else{
      printf("lost the object!");
      model2hand.setIdentity();
    }

    predicted_hand_pose = hand._handbase_in_cam;
  
    end_time = ros::WallTime::now();

    ROS_INFO_STREAM("Exec time(ms): " << (end_time - start_time).toNSec()*1e-6);

    predicted_hand->header.frame_id = "/head_camera_rgb_optical_frame";
    predicted_hand_pointcloud_pub.publish(predicted_hand);

    hand._visible_set->header.frame_id = "/gripper_link";
    visible_pointcloud_pub.publish(hand._visible_set);

    // hand.test1->header.frame_id = "/gripper_link";
    // test1_pub.publish(hand.test1);

    // hand.test2->header.frame_id = "/gripper_link";
    // test2_pub.publish(hand.test2);

    hand.reset();
    est.reset();
    // update the time stamp
    trackstamp++;

    // cv::imshow("view", scene_bgr);
    // cv::imshow("depth_view", scene_depth_check);
    // cv::waitKey(5);
  }
  
  // cv::destroyWindow("color");
  // cv::destroyWindow("depth_view");

  thread_b.join();

  return 0;

}