#!/usr/bin/env python
import os

from numpy.lib.function_base import place
import rospy
from fetch_robot import Fetch_Robot
from geometry_msgs.msg import Pose
from panda3d.bullet import BulletDebugNode
from tf_util import PosMat_t_PandaPosMax, TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat, PandaPosMax_t_PosMat, align_vectors, pointDown
from rail_segmentation.srv import SearchTable
from in_hand_manipulation.srv import StopOctoMap
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.fetch_gripper import fetch_grippernm
from regrasp_planner import RegripPlanner
from manipulation.grip.ffregrasp import ff_regrasp_planner
from database import dbaccess as db
from utils import dbcvt as dc
from utils import robotmath as rm
import numpy as np
from scipy.spatial.transform import Rotation as R
import pandaplotutils.pandageom as pandageom
import time
from icra20_manipulation_pose.srv import SearchObject
from scipy.special import softmax

import networkx as nx
import matplotlib.pyplot as plt


def grasp_estimation(tf_helper, robot, object_name, object_path, isSim):
    """
    This function will return the gripper pose in the object frame
    return: issuccess, poseMat_of_object
    """

    try:
        init_hand_in_obj_transform = tf_helper.getTransform('/' + object_name, '/gripper_link')

        # add the object into the planning scene
        current_object_transform = tf_helper.getTransform('/base_link', '/' + object_name)
        robot.addCollisionObject(object_name + "_collision", current_object_transform, object_path)
    except Exception as e:# if failure, then return False
        print e
        return False, None

    return True, init_hand_in_obj_transform

def add_table(robot, tf_helper):
    """
    add the table into the planning scene
    """
    # get table pose
    table_transform = tf_helper.getTransform('/base_link', '/Table')
    # add it to planning scene
    robot.addCollisionTable("table", table_transform[0][0], table_transform[0][1], table_transform[0][2], \
        table_transform[1][0], table_transform[1][1], table_transform[1][2], table_transform[1][3], \
        0.8, 0.8, 0.8)

# pickup is the action to move the gripper up in the base_link frame
def pickup(robot, tf_helper, height):
    """Pick up object"""
    
    target_transform = tf_helper.getTransform('/base_link', '/gripper_link')
    target_transform[0][2] += height

    robot.switchController('my_cartesian_motion_controller', 'arm_controller')

    while not rospy.is_shutdown():
        if robot.moveToFrame(target_transform, True):
            break
        rospy.sleep(0.05)

    robot.switchController('arm_controller', 'my_cartesian_motion_controller')

    return True, target_transform

def placedown(robot, placing_grasp):
    # move to pre placing position
    buffer = 0.03
    pre_placing_grasp = getTransformFromPoseMat(placing_grasp)
    pre_placing_grasp[0][2] += buffer

    placing_plan = robot.planto_pose(pre_placing_grasp)
    if not placing_plan.joint_trajectory.points:
        print "found no plan to place down"
        return False

    robot.display_trajectory(placing_plan)
    robot.execute_plan(placing_plan)
    
    # place the object down to the table
    robot.switchController('my_cartesian_motion_controller', 'arm_controller')
    while not rospy.is_shutdown():
        if robot.moveToFrame(getTransformFromPoseMat(placing_grasp), True):
            break
        rospy.sleep(0.2)
    robot.switchController('arm_controller', 'my_cartesian_motion_controller')
    return True

def moveback(robot, tf_helper, move_dis = -0.06):
    # move back
    current_trans = tf_helper.getTransform('/base_link', '/gripper_link')
    post_place_trans =  transformProduct(current_trans, [[move_dis,0,0],[0,0,0,1]])

    robot.switchController('my_cartesian_motion_controller', 'arm_controller')

    while not rospy.is_shutdown():
        if robot.moveToFrame(post_place_trans, True):
            break
        rospy.sleep(0.05)

    robot.switchController('arm_controller', 'my_cartesian_motion_controller')

def regrasping(tf_helper, robot, planner, dmgplanner, object_name=None, manipulation_position_list = None, target_grasps = None, init_grasp_transform_in_object_frame = None, init_jawwidth = None, base = None):

    movebackmatrix = np.identity(4)
    movebackmatrix[2, 3] = -0.06

    # load the dmg planner from database
    dmgplanner.loadDB()

    # get the manipulation position in pose mat format in the base link frame
    manipulation_position = getMatrixFromQuaternionAndTrans(manipulation_position_list[0][1], manipulation_position_list[0][0])

    # input pose should be numpy format in the base_link
    def feasible(inputpose):
        place_ik_result = robot.solve_ik_sollision_free_in_base(getTransformFromPoseMat(inputpose),20)

        if place_ik_result != None:
            return True
        else:
            return False
   
    """Replace Object"""
    init_graspPose = getMatrixFromQuaternionAndTrans(init_grasp_transform_in_object_frame[1],init_grasp_transform_in_object_frame[0]) #int_Grasp in object frame
  
    # create the regrasp graph
    planner.CreatePlacementGraph()

    # the init grasp pose is numpy format, init jawwidth is in meter unit
    planner.addStartGrasp(init_graspPose,init_jawwidth)
    
    # add goal grasps into the graph
    for grasp, jawwidth in target_grasps:
        planner.addGoalGrasp(grasp, jawwidth)


    find_solution = False
    solution = None

    while not find_solution:
        # first level
        try:
            # this function will return multiple paths
            paths = planner.find_shortest_PlacementG_path()
        except Exception as e:
            print e
            return False, None
        if len(paths) == 0: # can't find a solution
            return False, None

        path = [('int_g', -1)] + paths[0]
        solution = []

        # # show the re-grasping graph
        # planner.showgraph()
        find_solution = True

        # search through the path
        for i in range(len(path)-1):
            last_p = path[i][0]
            next_p = path[i+1][0]
            next_p_type = path[i+1][1]
            foundcommonfeasiblegrasp = False
            for g in planner.getCommonGraspids(last_p, next_p):
                last_found, next_found = False, False
                pick_grasp, place_grasp = None, None
                pick_real_placement, place_real_placement = None, None

                c_jawwidth = g[1] / 1000.0 if type(g) == tuple else planner.getGraspsById([g])[0][1]
                c_grasp = PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * g[0]) if type(g) == tuple else planner.getGraspsById([g])[0][0]

                if last_p != 'int_g':
                    for tableangle in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
                        rotationInZ = np.identity(4)
                        rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()

                        ## calculate the gripper pose on table
                        pick_real_placement = rotationInZ.dot(planner.getPlacementsById([planner.PlacementG.nodes[last_p]['placement']])[0])
                        pick_grasp = manipulation_position.dot(pick_real_placement).dot(c_grasp)

                        if feasible(pick_grasp) and feasible(pick_grasp.dot(movebackmatrix)):
                            last_found = True
                            break

                else:
                    last_found = True

                if not last_found:
                    continue

                if next_p != 'end_g':
                    for tableangle in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
                        rotationInZ = np.identity(4)
                        rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()

                        ## calculate the gripper pose on table
                        place_real_placement = rotationInZ.dot(planner.getPlacementsById([planner.PlacementG.nodes[next_p]['placement']])[0])
                        place_grasp = manipulation_position.dot(place_real_placement).dot(c_grasp)

                        if feasible(place_grasp) and feasible(place_grasp.dot(movebackmatrix)):
                            next_found = True
                            break
                else:
                    next_found = True
                
                if last_found and next_found:
                    foundcommonfeasiblegrasp = True
                    solution.append((pick_grasp, place_grasp, c_jawwidth, c_grasp, last_p, g, next_p, next_p_type, pick_real_placement, place_real_placement))
                    break

            if not foundcommonfeasiblegrasp:
                # need to remove the edge in level 1
                planner.removeEdgeOnG(last_p, next_p)

                find_solution = False
                break
        
    if find_solution:
        print "found solution"
    else:
        print "not solution"
        return False, None

    # get place-and-pick action sequence from solution
    place_and_pick_solution = []
    
    for p in range(len(solution) - 1):
        # place grasp, place jawwidth, pick grasp, pick jawwidth, placement id, placement type, place real placement on table
        place_and_pick_solution.append((solution[p][1], solution[p][3] ,solution[p][2], solution[p+1][0], solution[p+1][3],solution[p+1][2], solution[p][6], solution[p][7],  solution[p][9]))

    # attach object into the hand
    robot.attachManipulatedObject(object_name + "_collision")

    executeSuccess = True

    # execute place and pick action sequence
    for place_grasp, place_in_hand_pose, place_jawwidth, pick_grasp, pick_in_hand_pose, pick_jawwidth, placement_id, placement_type, object_placement_on_table in place_and_pick_solution:

        # add current object into the planning scene
        robot.reattachManipulatedObject(object_name + "_collision", getTransformFromPoseMat(np.linalg.inv(place_in_hand_pose)))

        # place the object down to the table
        if not placedown(robot, place_grasp):
            executeSuccess = False
            break

        # check the type of current placement
        if placement_type == 0: # current placement is table

            robot.openGripper()
            robot.detachManipulatedObject(object_name + "_collision")
            moveback(robot, tf_helper)

            pick_trans = getTransformFromPoseMat(pick_grasp)

            pre_pick_trans =  transformProduct(pick_trans, [[-0.06,0,0],[0,0,0,1]]) #adjust the grasp pos to be a little back 

            picking_plan = robot.planto_pose(pre_pick_trans)
            robot.display_trajectory(picking_plan)

            robot.execute_plan(picking_plan)

            # move forward
            robot.switchController('my_cartesian_motion_controller', 'arm_controller')

            while not rospy.is_shutdown():
                if robot.moveToFrame(pick_trans, True):
                    break
                rospy.sleep(0.05)

            robot.switchController('arm_controller', 'my_cartesian_motion_controller')

        else:

            robot.setGripperWidth(place_jawwidth + 0.005)
            robot.detachManipulatedObject(object_name + "_collision")

            dmgresult = dmgplanner.getTrajectory(place_in_hand_pose, pick_in_hand_pose, pick_jawwidth, object_placement_on_table, base)

            if dmgresult == None:
                executeSuccess = False
                break

            #Convert from object frame to torso frame
            object_in_base_link = manipulation_position.dot(object_placement_on_table)

            robot.switchController('my_cartesian_motion_controller', 'arm_controller')
            for graspPos in dmgresult:

                regraspTran = getTransformFromPoseMat(object_in_base_link.dot(graspPos))
              
                # move to regrasp pose
                while not rospy.is_shutdown():
                    if robot.moveToFrame(regraspTran, True):
                        break
                    rospy.sleep(0.1)

            robot.switchController('arm_controller', 'my_cartesian_motion_controller')

        robot.closeGripper()
        robot.attachManipulatedObject(object_name + "_collision")
        pickup(robot, tf_helper, 0.1)


        break
    # show the re-grasping graph
    if not executeSuccess:
        return False, None

    return True, None


if __name__=='__main__':

    object_name = "book"
    isSim = True

    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=isSim)
    tf_helper = TF_Helper()
    
    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(os.path.realpath(__file__))
    objpath = os.path.join(os.path.split(this_dir)[0], "objects", object_name + ".stl") 
    handpkg = fetch_grippernm  #SQL grasping database interface 
    gdb = db.GraspDB()   #SQL grasping database interface

    planner = RegripPlanner(objpath, handpkg, gdb)
    dmg_planner = ff_regrasp_planner(objpath, handpkg, gdb)  

    # add table into planning scene
    add_table(robot, tf_helper)

    result, grasp_trans = grasp_estimation(tf_helper, robot, object_name=object_name, object_path=objpath, isSim=isSim)
    print "grasp pose estimation"
    if result:
        print "---SUCCESS---"
    else:
        print "---FAILURE---"
        exit()

    init_grasp_pose = getMatrixFromQuaternionAndTrans(grasp_trans[1], grasp_trans[0])

    def getTargetGrasps(gdb, object_name):
        sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % object_name
        result = gdb.execute(sql)
        if not result:
            print "please add the object name to the table first!!"
            return False, None
        else:
            objectId = int(result[0][0])

        sql = "SELECT grasppose, jawwidth FROM targetgrasps WHERE idobject = '%d'" % objectId
        targetgrasp_result = gdb.execute(sql)

        # target grasps (grasp pose(in numpy format), jawwidth(in meter))
        target_grasps = []
        for grasppose, jawwidth in targetgrasp_result:
            target_grasps.append((PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(grasppose)), float(jawwidth) / 1000))

        return True, target_grasps

    result, target_grasps = getTargetGrasps(gdb,object_name)
    if not result:
        print "no target grasp given"
        exit()

    init_jawwidth = robot.getFingerValue() * 2 + 0.01
    
    manipulation_pos_list = [[[0.65, 0.0, 0.79], [0.0,0.0,0.0,1.0]]]

    # show the position tf in rviz
    tf_helper.pubTransform("place_pos", manipulation_pos_list[0])


    start_time = time.time()
    result, object_pose_in_hand = regrasping(tf_helper, robot, planner, dmg_planner, object_name, manipulation_pos_list, target_grasps, grasp_trans, init_jawwidth, base=base)
    print "run time of regrasping = ", time.time() - start_time
    print "regrasping"
    if result:
        print "---SUCCESS---"
    else:
        print "---FAILURE---"
        robot.detachManipulatedObject(object_name + "_collision")
        exit()

    # clean the object from planning scene

    robot.removeCollisionObject(object_name + "_collision")


    # planner.plotObject(base)
    # planner.showHand(jawwidth * 1000, pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * PosMat_t_PandaPosMax(init_grasp_pose), base)

    # 

    # # base.run()

    # planner.showFrame(base)

    # raw_input("done")