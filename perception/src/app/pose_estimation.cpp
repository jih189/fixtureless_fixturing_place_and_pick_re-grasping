//this is used to track hand
#include "Utils.h"
#include "ConfigParser.h"
#include "Renderer.h"
#include "Hand.h"
#include "PoseEstimator.h"
#include "tinyxml2.h"
#include "yaml-cpp/yaml.h"
#include "ParticleFilter.h"
#include "MarkerHelper.h"

#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>

#include <Eigen/StdVector>

#include <random>
#include <math.h>

#include <icra20_manipulation_pose/SearchObject.h>
#include "object_segmentation/Object_segmentation.h"

// static const std::string TOPIC_NAME = "/head_camera/rgb/image_raw";
// static const std::string DEPTH_TOPIC_NAME = "/head_camera/depth/image_raw";
// static const std::string CAM_INFO_NAME = "/head_camera/depth/camera_info";
static const std::string TOPIC_NAME = "/head_camera/rgb/image_rect_color";
static const std::string DEPTH_TOPIC_NAME = "/head_camera/depth/image_rect_raw";
static const std::string CAM_INFO_NAME = "/head_camera/depth/camera_info";

Eigen::Matrix3f cam_info_K;

cv::Mat scene_bgr;
cv::Mat object_mask;
cv::Mat scene_depth;

Eigen::Matrix4f target_pose;

bool initFlag;
bool isrunning;

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

// This is a thread to keep publishing the object's pose
void publish_object_state(int* publish_rate){

  ros::NodeHandle n;
  
  ros::Rate loop_rate(*publish_rate);

  tf::TransformBroadcaster object_pose_transform_broad_caster;
  tf::Transform object_pose_transform;
//   tf::Transform predicted_table_center_pose_transform;
  
  while(ros::ok()){
    loop_rate.sleep();
    if(target_pose.isIdentity()){
      continue;
    }
    Eigen::Quaterniond eigen_quat(target_pose.block<3,3>(0,0).cast<double>());
    Eigen::Vector3d eigen_trans(target_pose.block<3,1>(0,3).cast<double>());

    tf::Quaternion tf_quat;
    tf::Vector3 tf_trans;
    tf::quaternionEigenToTF(eigen_quat, tf_quat);
    tf::vectorEigenToTF(eigen_trans, tf_trans);

    object_pose_transform.setOrigin(tf_trans);
    object_pose_transform.setRotation(tf_quat);
    object_pose_transform_broad_caster.sendTransform(tf::StampedTransform(object_pose_transform, ros::Time::now(), "/head_camera_rgb_optical_frame", "cup"));
  }
}

// here is the server function to receive the command
bool searchObjectTrig(icra20_manipulation_pose::SearchObject::Request &req,
        icra20_manipulation_pose::SearchObject::Response &res){

  if(req.startsearch){
    target_pose.setIdentity();
    isrunning = true;
  }else{
    isrunning = false;
  }
  
  return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle nh;

    tf::TransformListener tf_listener;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    initFlag = false;
    isrunning = false;
    target_pose.setIdentity();

    // spawn another thread
    int rate_b = 5;
    boost::thread thread_b(publish_object_state, &rate_b);

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

    // initilize the object segementation filter server
    ros::ServiceClient client = nh.serviceClient<object_segmentation::Object_segmentation>("object_filter");
    client.waitForExistence();

    // init image receiver
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_rgb = it.subscribe(TOPIC_NAME, 1,
            imageCallback);

    image_transport::Subscriber sub_depth = it.subscribe(DEPTH_TOPIC_NAME, 1,
            imageDepthCallback);

    ros::Subscriber caminfoSub = nh.subscribe(CAM_INFO_NAME, 10, camInfoCallback,
                        ros::TransportHints().tcpNoDelay());

    ros::Publisher point_cloud_pub = nh.advertise<PointCloudRGBNormal> ("scene", 1);
    ros::Publisher object_3d_mask_pub = nh.advertise<PointCloudRGBNormal> ("object_3d_mask", 1);
    ros::Publisher predicted_object_pointcloud_pub = nh.advertise<PointCloudSurfel> ("predicted_object", 1);

    // init service server with function 'searchObjectTrig'
    ros::ServiceServer service = nh.advertiseService("searchObject", searchObjectTrig);

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> pose_particles;
    pose_particles.reserve(30);

    // marker helper
    MarkerHelper marker_helper(nh, "package://fetch_description/meshes/object/cup.obj");

    bool isTracking = false;

    // init particle filter
    ParticleFilter particle_filter(100);

    ros::Rate loop_rate(10);

    // main loop
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();

        if(!isrunning){
            ROS_INFO("is not running!");
            continue;
        }

        // if no image in, then continue to wait
        if(scene_bgr.rows == 0 || scene_depth.rows == 0){
            ROS_INFO("no image comming!");
            continue;
        }
        
        // generate the pointcloud from the rgb depth of scene
        PointCloudRGBNormal::Ptr scene_rgb(new PointCloudRGBNormal);
        PointCloudRGBNormal::Ptr object_rgb(new PointCloudRGBNormal);
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

        pcl::copyPointCloud(*scene_rgb, *object_rgb);

        // pass the image to the server
        object_segmentation::Object_segmentation srv;
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", scene_bgr).toImageMsg(srv.request.Image);


        if(!client.call(srv)){
            isTracking = false;
            ROS_INFO("object segmentation: FAILURE!");
            continue;
        }

        cv::cvtColor(cv_bridge::toCvCopy(srv.response.Result, "rgb8")->image, object_mask, CV_BGR2GRAY);

        cv::imshow("mask", object_mask);

        if(cv::countNonZero(object_mask) > 800){ // check whether detect the object

            // extract point cloud belong into the object
            for (int u = 0; u < object_mask.rows; u++){
                for (int v = 0; v < object_mask.cols; v++){
                    if(object_mask.at<uchar>(u, v) == 0){
                        object_rgb->at(v, u).x = 0;
                        object_rgb->at(v, u).y = 0;
                        object_rgb->at(v, u).z = 0;
                    }
                }
            }

            // pcl filter to filter out point that is too far or too close
            {
                pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
                pass.setInputCloud(object_rgb);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(0.03, 2.0);
                pass.setKeepOrganized(true);
                pass.filter(*object_rgb);
            }

            object_rgb->header.frame_id = "/head_camera_rgb_optical_frame";
            object_rgb->header.stamp = ros::Time::now().toNSec()/1e3;
            object_3d_mask_pub.publish(object_rgb);

            // extract the point cloud being to the object
            PointCloudSurfel::Ptr object1(new PointCloudSurfel);
            pcl::copyPointCloud(*object_rgb, *object1);

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

            Utils::downsamplePointCloud<pcl::PointSurfel>(object_segment, object_segment, 0.005);

            PointCloudSurfel::Ptr scene_003(new PointCloudSurfel);
            pcl::copyPointCloud(*object_rgb, *scene_003);
            Utils::downsamplePointCloud<pcl::PointSurfel>(scene_003, scene_003, 0.003);

            // assign high comfidence on each point
            for (pcl::PointSurfel &pt:object_segment->points)
                pt.confidence=1.0;

            est.setCurScene(scene_003, cloud_withouthand_raw, object_segment, scene_bgr, scene_depth);

            if(!isTracking){
                ROS_INFO("DETECT");

                bool succeed = est.runSuper4pcs(ppfs, Eigen::Matrix4f::Identity());
                if(succeed){
                    est.clusterPoses(30, 0.015, true);
                    est.refineByICP();
                    est.clusterPoses(5, 0.003, false);
                    // est.rejectByRender(cfg.pose_estimator_wrong_ratio, &hand);

                    // select the best and publish
                    PoseHypo best(-1);
                    est.selectBest(best);

                    // generate the particles
                    particle_filter.sampling(best._pose);
                    pose_particles.clear();
                    for(int n = 0; n < particle_filter.size(); n++)
                        pose_particles.push_back(particle_filter.get(n));
                    
                    isTracking = true;

                    
                }
            }
            else{

                ROS_INFO("tracking");

                est.runUpdate(pose_particles);
                est.refineByICP();
                est.clusterPoses(2, 0.0005, true);

                pose_particles.clear();

                std::vector<float> hypo_lcp_scores;

                PoseHypo best(-1);
                est.selectBest(best);

                for(int n = 0; n < est.getNumOfHypos(); n++){
                    PoseHypo selectedHypo(-1);
                    est.selectIndex(selectedHypo, n);
                    if(selectedHypo._lcp_score < 50)
                        continue;
                    pose_particles.push_back(selectedHypo._pose);
                    hypo_lcp_scores.push_back(selectedHypo._lcp_score);
                }

                // ROS_INFO_STREAM("number of particle " << pose_particles.size());

                if(pose_particles.size() == 0)
                    isTracking = false;
                else{
                    softmax(hypo_lcp_scores);

                    int maxWeightIndex = std::max_element(hypo_lcp_scores.begin(), hypo_lcp_scores.end()) - hypo_lcp_scores.begin();
                    target_pose = pose_particles[maxWeightIndex];

                    marker_helper.publishMarkers(pose_particles, hypo_lcp_scores);

                    // need to resampling
                    particle_filter.resample(pose_particles, hypo_lcp_scores);
                    pose_particles.clear();
                    for(int n = 0; n < particle_filter.size(); n++)
                        pose_particles.push_back(particle_filter.get(n));

                }                
            }
            est.reset();
        }
        else{
            ROS_INFO("the mask rcnn can't detect the object in scene!");
            isTracking = false;
        }

        // object_rgb->header.frame_id = "/head_camera_rgb_optical_frame";
        // object_rgb->header.stamp = ros::Time::now().toNSec()/1e3;
        // object_3d_mask_pub.publish(object_rgb);
    
        scene_rgb->header.frame_id = "/head_camera_rgb_optical_frame";
        scene_rgb->header.stamp = ros::Time::now().toNSec()/1e3;
        point_cloud_pub.publish(scene_rgb);

        cv::imshow("view", scene_bgr);
        cv::imshow("depth_view", scene_depth);
        
        cv::waitKey(5);
    }
    return 0;
}