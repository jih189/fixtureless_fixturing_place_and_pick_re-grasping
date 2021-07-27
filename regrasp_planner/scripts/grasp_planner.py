#!/usr/bin/python
import sys
import rospy

from geometry_msgs.msg import Pose
from icra20_manipulation_pose.srv import SearchObject
from rospy.impl.tcpros_service import wait_for_service

from visualization_msgs.msg import Marker
from fetch_robot import Fetch_Robot

import tf

from  tf_util import TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans, findCloseTransform, getTransformFromPoseMat
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander

if __name__=='__main__':
    rospy.init_node('grasp_node')


    # tf_helper = TF_Helper()

    # get the object searcher trigger to control the tracker
    # rospy.wait_for_service('searchObject')
    # objectSearcherTrigger = rospy.ServiceProxy('searchObject', SearchObject)

    # robot = Fetch_Robot()

    # robot.goto_pose(0.34969, 0.20337, 0.92054, 0.081339, 0.012991, -0.63111, 0.77131)

    # ## launch the tracker
    # objectSearcherTrigger(True, 1, Pose())

    # try:
    #     # add the object to the moveit
    #     target_transform = tf_helper.getTransform('/base_link', '/cup')
    #     robot.addCollisionObject("object", target_transform, "objects/cuboid.stl")
    # except:
    #     print "fail to detect the object"
        
    # objectSearcherTrigger(False, 0, Pose())