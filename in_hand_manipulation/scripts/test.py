#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from geometry_msgs.msg import Pose2D
from tf_util import  TF_Helper, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat, adjointRepresentationMatrix
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import copy
import time
if __name__=='__main__':

    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=True)

    init_trans = [[0.907, -0.356, 0.8],[0, 0, 0, 1]]

    ik_planning_time = time.time()
    init_ik_result = robot.solve_ik_collision_free_in_base(init_trans, 10)
    print("ik planning time ", time.time() - ik_planning_time)

    approach_plan = robot.planto_joints(init_ik_result.state.joint_state.position, init_ik_result.state.joint_state.name)
    robot.execute_plan(approach_plan)

    # generate cartesian path from the pose waypoints
    moveit_robot_state = copy.deepcopy(init_ik_result.state)

    gripper_trajectory = []
    for i in range(50):
        temp = copy.deepcopy(init_trans)
        temp[0][2] += (i * 0.001)
        gripper_trajectory.append(temp)

    planning_time = time.time()
    cartesian_plan, fraction = robot.planFollowEndEffectorTrajectory(moveit_robot_state, gripper_trajectory, 0.1)
    print("length of waypoint ", len(gripper_trajectory))
    print("length of cartesian path ", len(cartesian_plan.joint_trajectory.points))
    print("cartesian planning time ", time.time() - planning_time)

    robot.execute_plan(cartesian_plan)