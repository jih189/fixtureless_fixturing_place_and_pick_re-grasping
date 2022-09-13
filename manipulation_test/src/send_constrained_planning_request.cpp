#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

void transformTFToGeoPose(const tf::Transform &t, geometry_msgs::Pose &p){
  p.orientation.x = t.getRotation().x();
  p.orientation.y = t.getRotation().y();
  p.orientation.z = t.getRotation().z();
  p.orientation.w = t.getRotation().w();
  p.position.x = t.getOrigin().x();
  p.position.y = t.getOrigin().y();
  p.position.z = t.getOrigin().z();
}

int main(int argc, char** argv)
{
  
  // predefine in-hand object poses
  std::vector<tf::Transform> in_hand_poses;

  float positions[5][3] = {
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025},
	  		{0.202,0,-0.025}
  			};
  float rotations[5][4] = {
	  		{-0.3,0.067,0.947,-0.084},
	  		{-0.303,-0.052,0.907,0.284},
	  		{-0.245,-0.186,0.674,0.671},
	  		{-0.177,-0.251,0.44,0.843},
	  		{-0.041,-0.305,0.001,0.951}
  			};


  for(int i = 0; i < 5; i++){
    tf::Transform one_in_hand_pose;
    one_in_hand_pose.setOrigin(tf::Vector3(positions[i][0], positions[i][1], positions[i][2]));
    one_in_hand_pose.setRotation(tf::Quaternion(rotations[i][0],rotations[i][1],rotations[i][2],rotations[i][3]));
    in_hand_poses.push_back(one_in_hand_pose);
  }

  ros::init(argc, argv, "send_constrained_planning_request");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup*joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // extract the pose of the can
  tf::TransformListener listener;

  //Eigen::Affine3d object_in_world_frame;
  tf::StampedTransform can_transform;
  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/can", "/base_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/base_link","/can", ros::Time(0), can_transform);
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  // add the can into the planning scene
  moveit_msgs::CollisionObject collision_object_can;
  collision_object_can.header.frame_id = move_group.getPlanningFrame();
  collision_object_can.id = "can";
  std::string can_dir = "package://objects_description/can/can.stl";
  const Eigen::Vector3d scaling(0.001,0.001,0.001);
  shapes::Mesh *m = shapes::createMeshFromResource(can_dir, scaling);
  shape_msgs::Mesh can_mesh;
  shapes::ShapeMsg can_mesh_msg;
  shapes::constructMsgFromShape(m, can_mesh_msg);
  can_mesh = boost::get<shape_msgs::Mesh>(can_mesh_msg);

  geometry_msgs::Pose can_pose;
  transformTFToGeoPose(can_transform, can_pose);

  collision_object_can.meshes.push_back(can_mesh);
  collision_object_can.pose = can_pose;
  collision_object_can.operation = collision_object_can.ADD;
  
  collision_objects.push_back(collision_object_can);

  // add the table into the planning scene
  moveit_msgs::CollisionObject collision_object_table;
  collision_object_table.header.frame_id = move_group.getPlanningFrame();

  collision_object_table.id = "table";

  shape_msgs::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = 0.5;
  table_primitive.dimensions[1] = 1.2;
  table_primitive.dimensions[2] = 0.2;

  geometry_msgs::Pose table_pose;
  table_pose.position.x = 0.56;
  table_pose.position.y = 0;
  table_pose.position.z = 0.445;

  table_pose.orientation.x = 0;
  table_pose.orientation.y = 0;
  table_pose.orientation.z = 0;
  table_pose.orientation.w = 1;


  collision_object_table.primitives.push_back(table_primitive);
  collision_object_table.pose = table_pose;
  collision_object_table.operation = collision_object_table.ADD;
  collision_objects.push_back(collision_object_table);

  planning_scene_interface.addCollisionObjects(collision_objects);
  planning_scene_interface.applyCollisionObjects(collision_objects);

  // analyze where to grasp the object
  for(int i = 0; i < in_hand_poses.size(); i++){
    tf::Transform grasp_pose_in_world_frame = can_transform * in_hand_poses[i].inverse(); 

    moveit_visual_tools::MoveItVisualToolsPtr grasp_visuals = std::make_shared<moveit_visual_tools::MoveItVisualTools>("base_link", "/move_group/display_contacts");

    // visualize grasp pose
    geometry_msgs::Pose grasp_pose;
    transformTFToGeoPose(grasp_pose_in_world_frame, grasp_pose);
    std::vector<double> ee_joint_pos{0.045, 0.045};

    // visualize pre-grasp pose
    geometry_msgs::Pose pre_grasp_pose;
    transformTFToGeoPose(grasp_pose_in_world_frame * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-0.08,0,0)), pre_grasp_pose);

    //grasp_visuals->publishEEMarkers(grasp_pose, grasp_visuals->getSharedRobotState()->getJointModelGroup("gripper"), ee_joint_pos, rviz_visual_tools::WHITE, "grasp_pose_" + std::to_string(i));
    grasp_visuals->publishEEMarkers(pre_grasp_pose, grasp_visuals->getSharedRobotState()->getJointModelGroup("gripper"), ee_joint_pos, rviz_visual_tools::BLUE, "pre-grasp_pose_" + std::to_string(i));
  }
  return 0;
}
  /*
  geometry_msgs::Pose target_pose;
  target_pose.position.x = result.getOrigin().x();
  target_pose.position.y = result.getOrigin().y();
  target_pose.position.z = result.getOrigin().z();
  target_pose.orientation.x = result.getRotation().x();
  target_pose.orientation.y = result.getRotation().y();
  target_pose.orientation.z = result.getRotation().z();
  target_pose.orientation.w = result.getRotation().w();

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan constrained_plan;
  bool success = (move_group.plan(constrained_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  // Visualizing the solution trajectory
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

  visual_tools.publishTrajectoryLine(constrained_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  ////////////////////////////////////////////////////
  std::cout << "default planner: " << move_group.getDefaultPlannerId() << std::endl;

  // set planner id
  move_group.setPlannerId("CBIRRTConfigDefault");
  // get planner id
  std::string current_planner_id;
  std::cout << "current planner id: " << move_group.getPlannerId() << std::endl;

  // set the path constraints
  moveit_msgs::Constraints constraints;
  constraints.name = "horizontal_constraint";

  // set position constraint
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.link_name = "base_link";
  position_constraint.target_point_offset.x = 0.0;
  position_constraint.target_point_offset.y = 0.0;
  position_constraint.target_point_offset.z = 0.0;
  position_constraint.weight = 1.0;
  constraints.position_constraints.push_back(position_constraint);

  // set orientation constraint
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = "base_link";
  orientation_constraint.header.stamp = ros::Time(0);
  orientation_constraint.link_name = "wrist_roll_link";
  geometry_msgs::Quaternion constrained_quaternion;
  constrained_quaternion.x = 0;
  constrained_quaternion.y = 0;
  constrained_quaternion.z = -0.707;
  constrained_quaternion.w = 0.707;
  orientation_constraint.orientation = constrained_quaternion;
  orientation_constraint.weight = 1.0;
  orientation_constraint.absolute_x_axis_tolerance = 0.8;
  orientation_constraint.absolute_y_axis_tolerance = 0.8;
  orientation_constraint.absolute_z_axis_tolerance = 0.8;
  constraints.orientation_constraints.push_back(orientation_constraint);

  constraints.in_hand_pose.position.x = 0.01;
  //constraints.in_hand_pose.orientation.x = 0.05;
  //constraints.in_hand_pose.orientation.y = 0.0;
  //constraints.in_hand_pose.orientation.z = 0.0;
  //constraints.in_hand_pose.orientation.w = 0.999;
  
  move_group.setPathConstraints(constraints);

  // get current pose of the end-effector
  geometry_msgs::PoseStamped result = move_group.getCurrentPose("wrist_roll_link");
  std::cout << "current pose:" << std::endl;
  std::cout << "position" << std::endl;
  std::cout << "x: " << result.pose.position.x << std::endl;
  std::cout << "y: " << result.pose.position.y << std::endl;
  std::cout << "z: " << result.pose.position.z << std::endl;
  std::cout << "orientation" << std::endl;
  std::cout << "x: " << result.pose.orientation.x << std::endl;
  std::cout << "y: " << result.pose.orientation.y << std::endl;
  std::cout << "z: " << result.pose.orientation.z << std::endl;
  std::cout << "w: " << result.pose.orientation.w << std::endl;
	
  geometry_msgs::Pose target_pose;
  target_pose.position.x = result.pose.position.x;
  target_pose.position.y = result.pose.position.y;
  target_pose.position.z = result.pose.position.z - 0.15;
  target_pose.orientation.x = result.pose.orientation.x;
  target_pose.orientation.y = result.pose.orientation.y;
  target_pose.orientation.z = result.pose.orientation.z;
  target_pose.orientation.w = result.pose.orientation.w;

  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan constrained_plan;
  bool success = (move_group.plan(constrained_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  // Visualizing the solution trajectory
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

  visual_tools.publishTrajectoryLine(constrained_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  //if(success)
  //  move_group.move();



  ROS_INFO_NAMED("tutorials", "planning done!");

  ros::shutdown();

  return 0;
  */
