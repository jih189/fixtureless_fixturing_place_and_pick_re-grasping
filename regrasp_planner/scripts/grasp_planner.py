#!/usr/bin/python
import sys
import rospy

from geometry_msgs.msg import Pose
from rail_segmentation.srv import SearchTable
from icra20_manipulation_pose.srv import SearchObject
from rospy.impl.tcpros_service import wait_for_service

from visualization_msgs.msg import Marker
from fetch_robot import Fetch_Robot

import tf

from  tf_util import TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans, findCloseTransform, getTransformFromPoseMat
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from scipy.spatial.transform import Rotation as R

if __name__=='__main__':
    rospy.init_node('grasp_node')

    tf_helper = TF_Helper()

    # get the object searcher trigger to control the tracker
    rospy.wait_for_service('searchObject')
    objectSearcherTrigger = rospy.ServiceProxy('searchObject', SearchObject)

    robot = Fetch_Robot()

    ## need to add table
    rospy.wait_for_service('table_searcher/search_table')
    tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

    try:
        tableresult = tableSearcher()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

    r = R.from_quat([tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w])
    # need to adjust the rotation of the table
    original_r = r.as_euler('zyx')
    table_quaternion = R.from_euler('zyx', [0,original_r[1], original_r[2]]).as_quat()

    robot.addCollisionTable("table", tableresult.center.x, tableresult.center.y, tableresult.center.z, \
            table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
            tableresult.width, tableresult.depth, 0.001)

    ## launch the tracker
    objectSearcherTrigger(True, 1, Pose())

    try:
        # add the object to the moveit
        target_transform = tf_helper.getTransform('/base_link', '/cup')
        robot.addCollisionObject("object", target_transform, "objects/cup.stl")
    except Exception as e:
        print e
    
    objectSearcherTrigger(False, 0, Pose())