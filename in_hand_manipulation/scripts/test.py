#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from geometry_msgs.msg import Pose2D
from tf_util import  TF_Helper, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

if __name__=='__main__':

    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=True)
    tf_helper = TF_Helper()

    plan = robot.planto_open_gripper()
    robot.display_trajectory([plan])
    robot.openGripper()

