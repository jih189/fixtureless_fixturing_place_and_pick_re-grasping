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
    robot = Fetch_Robot(sim=False)
    robot.closeGripper()