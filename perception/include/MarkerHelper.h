#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


class MarkerHelper{
  public:
    MarkerHelper(ros::NodeHandle &nh, std::string mesh_source_path){
      _mesh_source_path = mesh_source_path;
      marker_pub = nh.advertise<visualization_msgs::MarkerArray>("particles", 1);
      headerNum = 30;
      currentNumOfMarkers = 0;
    }

    void cleanMarkers(int numofParticle){
      if(numofParticle == 0)
        return;
      visualization_msgs::MarkerArray markers;

      // clear markers
      for(int pn = 0; pn < numofParticle ; ++pn){
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time::now();

        marker.id = pn + headerNum;

        marker.type = visualization_msgs::Marker::MESH_RESOURCE;

        marker.action = visualization_msgs::Marker::DELETE;
        markers.markers.push_back(marker);
      }
      marker_pub.publish(markers);
      markers.markers.clear();
      numofParticle = 0;
    }

    void deleteAllMarkers(){

      visualization_msgs::MarkerArray markers;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/head_camera_rgb_optical_frame";
      marker.header.stamp = ros::Time::now();
      marker.action = visualization_msgs::Marker::DELETEALL;
      markers.markers.push_back(marker);
      marker_pub.publish(markers);
      markers.markers.clear();
    }

    void publishMarkers(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &inputs, std::vector<float> &weights){

      float height_weight = *std::max_element(weights.begin(), weights.end());

      // cleanMarkers(currentNumOfMarkers);
      deleteAllMarkers();
      visualization_msgs::MarkerArray markers;

      currentNumOfMarkers = inputs.size();

      for(int pn = 0; pn < inputs.size() ; ++pn){
        // std::cout << pose_particles[pn] << std::endl;

        visualization_msgs::Marker marker;

        marker.header.frame_id = "/head_camera_rgb_optical_frame";
        marker.header.stamp = ros::Time::now();

        marker.id = pn + headerNum;

        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = _mesh_source_path; //"package://regrasp_planner/scripts/objects/cuboid.stl";

        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Quaternionf eigen_quat(inputs[pn].block<3,3>(0,0).cast<float>());
        Eigen::Vector3f eigen_trans(inputs[pn].block<3,1>(0,3).cast<float>());

        marker.pose.orientation.x = eigen_quat.x();
        marker.pose.orientation.y = eigen_quat.y();
        marker.pose.orientation.z = eigen_quat.z();
        marker.pose.orientation.w = eigen_quat.w();
        marker.pose.position.x = eigen_trans.x();
        marker.pose.position.y = eigen_trans.y();
        marker.pose.position.z = eigen_trans.z();

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // marker.scale.x = 0.001;
        // marker.scale.y = 0.001;
        // marker.scale.z = 0.001;

        marker.color.a = 0.2 + 0.8 * (weights[pn] / height_weight);
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 0.5;

        markers.markers.push_back(marker);
      }

      marker_pub.publish(markers);
      markers.markers.clear();
    }

    ros::Publisher marker_pub;
    int headerNum, currentNumOfMarkers;
    std::string _mesh_source_path;
};