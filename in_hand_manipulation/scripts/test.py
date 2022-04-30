#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from geometry_msgs.msg import Pose2D
from tf_util import  TF_Helper, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat, adjointRepresentationMatrix
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


import copy
import time
if __name__=='__main__':

    rospy.init_node('test_node')
    # robot = Fetch_Robot(sim=False)


    def lift_robot_torso(joint_value=[0.35]): #0 - .4
        joint_names = ['torso_lift_joint']
        client = actionlib.SimpleActionClient("/torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        rospy.loginfo('Waiting for joint trajectory action')    
        client.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        # point.velocities.append(0.1)
        point.positions = joint_value
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        client.send_goal_and_wait(goal)

    lift_robot_torso()

