#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <in_hand_manipulation/StopOctoMap.h>

ros::Publisher pub;
// pcl::PCLPointCloud2 cloud;
sensor_msgs::PointCloud2 output;
bool hasfreepoint;

bool cleanOcto(in_hand_manipulation::StopOctoMap::Request &req,
               in_hand_manipulation::StopOctoMap::Response &res)
{
  hasfreepoint = !hasfreepoint;
  ROS_INFO("stop updating the Octo map");
  return true;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::toPCL(*input, cloud);

  // pcl_conversions::fromPCL(cloud, output);
  if(!hasfreepoint){
    output = sensor_msgs::PointCloud2(*input);
  }

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{

  hasfreepoint = false;

  // Initialize ROS
  ros::init (argc, argv, "octomap_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/head_camera/depth_downsample/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/head_camera/depth_downsample/free_space_points", 1);

  // Create a ROS server
  ros::ServiceServer service = nh.advertiseService("stop_octo_map", cleanOcto);

  // Spin
  ros::spin ();
}
