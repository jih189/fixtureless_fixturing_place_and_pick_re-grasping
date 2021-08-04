#!/usr/bin/python
import sys
import os
import rospy

from geometry_msgs.msg import Pose
from rail_segmentation.srv import SearchTable
from icra20_manipulation_pose.srv import SearchObject
from rospy.impl.tcpros_service import wait_for_service

from visualization_msgs.msg import Marker
from fetch_robot import Fetch_Robot

import tf

import pandaplotutils.pandactrl as pandactrl
from database import dbaccess as db

from  tf_util import *
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from scipy.spatial.transform import Rotation as R
from manipulation.grip.fetch_gripper import fetch_grippernm
from regrasp_planner import RegripPlanner


# this function will return both pre grasp pose and grasp pose in base link frame
def find_grasping_point(planner,tran_base_object):
    # filter out based on placement so we know which is the actuall grasp
    gripper_pos_list = planner.getGraspsbyPlacementPose(tran_base_object)

    print("Going through this many grasp pose: " ,len(gripper_pos_list))
    for i, (obj_grasp_pos, jaw_width) in enumerate(gripper_pos_list):
        
        obj_grasp_trans = getTransformFromPoseMat(obj_grasp_pos) #Tranfrom gripper posmatx to (trans,rot)
        obj_pre_grasp_trans =  transformProduct(obj_grasp_trans, [[-0.06,0,0],[0,0,0,1]]) #adjust the grasp pos to be a little back 
        obj_pre_grasp_trans = transformProduct(tran_base_object, obj_pre_grasp_trans)
        obj_grasp_trans = transformProduct(tran_base_object, obj_grasp_trans)

        # need to ensure both grasp and pre-grasp is valid for robot
        grasp_ik_result = robot.solve_ik_sollision_free_in_base(obj_grasp_trans, 10)

        if grasp_ik_result == None:
            print 'check on grasp ', i
            continue

        pre_grasp_ik_result = robot.solve_ik_sollision_free_in_base(obj_pre_grasp_trans, 10)

        if pre_grasp_ik_result == None:
            print 'check on grasp ', i
            continue
        
        return obj_pre_grasp_trans, pre_grasp_ik_result, obj_grasp_trans, jaw_width
    # if not solution, then return None
    return None, None, None, None


def move_to_pickup(robot, planner, target_transform):
    #Move to starting position
    #robot.goto_pose(0.34969, 0.20337, 0.92054, 0.081339, 0.012991, -0.63111, 0.77131)
    robot.openGripper()

    #Go through all grasp pos and find a valid pos. 
    obj_pre_grasp_trans, pre_grasp_ik_result, obj_grasp_trans, gripper_width = find_grasping_point(planner, target_transform)

    if pre_grasp_ik_result == None: # can't find any solution then return false.
        return False
        
    # move to pre grasp pose
    plan = robot.planto_pose(obj_pre_grasp_trans)
    robot.display_trajectory(plan)
    raw_input("ready to pre-grasp")
    robot.execute_plan(plan)

    # move to grasp pose
    raw_input("ready to grasp")
    robot.moveToFrame(obj_grasp_trans, True)
    
    # close the gripper with proper width
    print "grasp width = ", gripper_width
    raw_input("ready to close grasp")
    robot.setGripperWidth(gripper_width)

    return True

    # planner.plotObject(base)
    # base.run()

if __name__=='__main__':
    rospy.init_node('grasp_node')

    tf_helper = TF_Helper()

    # get the object searcher trigger to control the tracker
    rospy.wait_for_service('searchObject')
    objectSearcherTrigger = rospy.ServiceProxy('searchObject', SearchObject)

    # initialize the robot
    robot = Fetch_Robot(False)

    # Creates a base simulator in the world. 
    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])

    # initialize the planner
    this_dir, this_filename = os.path.split(__file__)   
    objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cup.stl") 
    handpkg = fetch_grippernm  #SQL grasping database interface 
    gdb = db.GraspDB()   #SQL grasping database interface
    planner = RegripPlanner(objpath, handpkg, gdb)

    ## need to add table
    print "--- search for the table ---"
    rospy.wait_for_service('table_searcher/search_table')
    tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

    try:
        tableresult = tableSearcher()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        print "--- status: failure ---"
        exit()
    print "--- status: done ---"

    r = R.from_quat([tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w])
    # need to adjust the rotation of the table
    original_r = r.as_euler('zyx')
    table_quaternion = R.from_euler('zyx', [0,original_r[1], original_r[2]]).as_quat()

    # add table into the moveit
    robot.addCollisionTable("table", tableresult.center.x, tableresult.center.y, tableresult.center.z + 0.04, \
            table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
            tableresult.width, tableresult.depth, 0.001)
    table_base_transform = transformProduct([[0,0,-0.3],[0,0,0,1]], \
            [[tableresult.center.x,tableresult.center.y,tableresult.center.z], \
            [table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3]]])
    robot.addCollisionTable("table_base", table_base_transform[0][0], table_base_transform[0][1], table_base_transform[0][2], \
            table_base_transform[1][0], table_base_transform[1][1], table_base_transform[1][2], table_base_transform[1][3], \
            tableresult.width, tableresult.depth, 0.6)

    print "--- search for the object ---"
    ## launch the tracker
    objectSearcherTrigger(True, 1, Pose())

    raw_input("object detection")

    try:
        # add the object to the moveit
        target_transform = tf_helper.getTransform('/base_link', '/cup')
        robot.addCollisionObject("object", target_transform, objpath)
    except Exception as e:
        print e
        print "--- status: failure ---"
        exit()
    print "--- status: done ---"
    
    objectSearcherTrigger(False, 0, Pose())

    print "--- pick up the object ---"
    if(move_to_pickup(robot, planner, target_transform)):
        print "--- status: done ---"
    else:
        print "--- status: failure ---"