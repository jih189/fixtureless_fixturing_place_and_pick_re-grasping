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
from utils import dbcvt as dc

from  tf_util import TF_Helper, Panda_Helper, transformProduct, getMatrixFromQuaternionAndTrans, findCloseTransform, getTransformFromPoseMat
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from scipy.spatial.transform import Rotation as R
from manipulation.grip.fetch_gripper import fetch_grippernm
from regrasp_planner import RegripPlanner
from manipulation.grip.ffreplacement import FF_replacement_planner
from manipulation.grip.ffregrasp import PandaPosMax_t_PosMat, ff_regrasp_planner

# this is the main code to detect the object and grasp it, then regrasp it in the open space area
# for further manipulation. Each function in this script can be considered as a action to run.

# 
def move_to_regrasp_placement( init_graspTrans, jawwidth, tableresult, planner):

    sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % "cup"
    result = gdb.execute(sql)
    if not result:
        print "please add the object name to the table first!!"
        raise Exception("the table does not contain the object!")
    else:
        objectId = int(result[0][0])

    sql = "SELECT grasppose, jawwidth FROM targetgrasps WHERE idobject = '%d'" % objectId
    targetgrasp_result = gdb.execute(sql)

    # target grasps (grasp pose(in numpy format), jawwidth(in meter))
    target_grasps = []
    for grasppose, jawwidth in targetgrasp_result:
        target_grasps.append((PandaPosMax_t_PosMat(dc.strToMat4(grasppose)), float(jawwidth) / 1000))
    
    #regrasp_planner = ff_regrasp_planner()
    

    #TODO: An optimal path to the target grasp should always be 1 and will always exsist for our objects
    # This is not true for other objects. 
    for grasp in target_grasps:
        init_graspPose = getMatrixFromQuaternionAndTrans(init_graspTrans[1],init_graspTrans[0])
        planner.addStartGrasp(init_graspPose,jawwidth)
        planner.addGoalGrasp(grasp[0], grasp[1])
        path = planner.find_shortest_PlacementG_path()
        grasp_t = planner.get_placement_grasp_trajectory()

        if len(path) == 1:
            break 
    
    placement_id = path[0]
    placement_stable = path[1] # 0 == stable; 1 == unstable
    
    """"----- Placing the object on table according to placement_id"""
    # get the pose mat from base link to table
    tablePos = PandaPosMax_t_PosMat(getMatrixFromQuaternionAndTrans(tableresult.orientation, tableresult.centroid))
    tableTrans = getTransformFromPoseMat(tablePos)
    placementTrans = getTransformFromPoseMat(planner.getPlacementsById(path[0]))


    base_t_tableobject_trans = transformProduct(tableTrans,placementTrans)
    gripper_t_tablePlacement = transformProduct(base_t_tableobject_trans, init_graspTrans)
  

    """ --- Preform Regrasp """
    if placement_stable == 0:
        "pick and place"
    else:
        ""



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

    # pick up the object
    #TODO
    return True , obj_grasp_trans, gripper_width

    # planner.plotObject(base)
    # base.run()


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
    robot.addCollisionTable("table_base", tableresult.center.x, tableresult.center.y, tableresult.center.z - 0.3, \
            table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
            tableresult.width, tableresult.depth, 0.6)

    ## launch the tracker
    objectSearcherTrigger(True, 1, Pose())

    try:
        # add the object to the moveit
        target_transform = tf_helper.getTransform('/base_link', '/cup')
        robot.addCollisionObject("object", target_transform, "objects/cup.stl")
    except Exception as e:
        print e
    
    objectSearcherTrigger(False, 0, Pose())

    
    print "Entering Pick up"
    print("Test print")

    # Creates a base simulator in the world. 
    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    #Get the path of the object file 
    this_dir, this_filename = os.path.split(__file__)   
    objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cup.stl") 
    handpkg = fetch_grippernm  #SQL grasping database interface 
    gdb = db.GraspDB()   #SQL grasping database interface
    planner = RegripPlanner(objpath, handpkg, gdb)

    try:
        success, init_grasptrans, jawwidth = move_to_pickup(robot, planner,target_transform)
    except Exception as e:
        print e 
        
    move_to_regrasp_placement( init_grasptrans, jawwidth, tableresult, planner)