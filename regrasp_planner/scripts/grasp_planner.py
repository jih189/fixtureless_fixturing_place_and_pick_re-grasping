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


def move_to_regrasp_placement( obj_path, init_grasp, handpkg,gdb):

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
    
    regrasp_planner = ff_regrasp_planner()
    replacement_planner = FF_replacement_planner(obj_path,handpkg,gdb)
    for grasp in target_grasps:
        replacement_planner.insert_ini_grasp_as_placement(init_grasp)


def find_grasping_point(planner,tran_base_object ):
    #TODO filter out based on place ment so we know which is the actuall grasp

    pd_helper = Panda_Helper()
    print("Going through this many grasp pose: " ,len(planner.freegriprotmats))
    for i in range(0,len(planner.freegriprotmats)):
        obj_grasp_pos = planner.freegriprotmats[i] #Gives transforom matrix of grasp based on objrct ref  
        jaw_width = planner.freegripjawwidth[i] / 1000 #Give jaw_with in non_panda form
        #planner.showHand( planner.freegripjawwidth[i], obj_grasp_pos, base)

        #change obj_pos from panda config to normal. Change pos rep to Quatorian. 
        obj_grasp_pos =  pd_helper.PandaPosMax_t_PosMat(obj_grasp_pos) 
        obj_grasp_pos = pd_helper.RotateGripper(obj_grasp_pos)
        obj_grasp_pos_Q = getTransformFromPoseMat(obj_grasp_pos) #Tranfrom gripper posmatx to (trans,rot)
        obj_grasp_pos_Q =  transformProduct(obj_grasp_pos_Q, [[-0.12,0,0],[0,0,0,1]]) #adjust the grasp pos to be a little back 

        world_grasp_pos_Q = transformProduct(tran_base_object, obj_grasp_pos_Q)
        t,r = world_grasp_pos_Q
        world_grasp_pos = tf.TransformerROS().fromTranslationRotation(t,r)

        base_link_in_torso_link = tf_helper.getPoseMat('/torso_lift_link', '/base_link')
        torso_grasp_pos = base_link_in_torso_link.dot(world_grasp_pos)
        t_g_p_q = getTransformFromPoseMat(torso_grasp_pos)
        grasp_ik_result = robot.solve_ik_collision_free(t_g_p_q, 10)
        print("Done with Ik solver")
        if grasp_ik_result == None:
            print(i)
            continue
        else:
            world_grasp_pos_Q = transformProduct(tran_base_object, obj_grasp_pos_Q)
            return world_grasp_pos_Q, t_g_p_q , jaw_width


def move_to_pickup(robot):
    #Move to starting position
    #robot.goto_pose(0.34969, 0.20337, 0.92054, 0.081339, 0.012991, -0.63111, 0.77131)
    robot.openGripper()
    # robot.closeGripper([-0.01])

    # Creates a base simulator in the world. 
    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    #Get the path of the object file 
    this_dir, this_filename = os.path.split(__file__)   
    objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cup.stl") 
    handpkg = fetch_grippernm  #SQL grasping database interface 
    gdb = db.GraspDB()   #SQL grasping database interface
    planner = RegripPlanner(objpath, handpkg, gdb)

    tran_base_object = tf_helper.getTransform('/base_link', '/cup') #return tuple (trans,rot) of parent_lin to child_link
    #Go through all grasp pos and find a valid pos. 
    world_grasp_pos_pre, torso_grasp_pos_pre, gripper_width = find_grasping_point(planner, tran_base_object)
    world_grasp_pos_Q = transformProduct(world_grasp_pos_pre, [[0.12,0,0],[0,0,0,1]]) 
    #torso_grasp_pos = transformProduct(torso_grasp_pos_pre, [[0.12,0,0],[0,0,0,1]])
        
    plan = robot.planto_pose(world_grasp_pos_pre)
    robot.display_trajectory(plan)
    raw_input("check")
    robot.execute_plan(plan)

    raw_input("check")
    robot.moveToFrame(world_grasp_pos_Q)
    
    if robot._sim == True:
        gripper_width = gripper_width -0.04
    else:
        gripper_width = gripper_width/2
    robot.closeGripper(gripper_width)

    " pick up the object"
    
    move_to_regrasp_placement(objpath, world_grasp_pos_Q, handpkg,gdb )
    # robot.switchController('my_cartesian_motion_controller', 'arm_controller')
    # while not rospy.is_shutdown():
    #     if(robot.moveToFrame(torso_grasp_pos)):
    #         break
    #     rospy.sleep(0.05)
    # robot.switchController('arm_controller', 'my_cartesian_motion_controller')

    # plan = robot.planto_pos(world_grasp_pos)
    # robot.display_trajectory(plan)
    #robot.execute_plan(plan)

    planner.plotObject(base)
    base.run()

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
    move_to_pickup(robot)