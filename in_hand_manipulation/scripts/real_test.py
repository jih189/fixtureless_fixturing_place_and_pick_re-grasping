#!/usr/bin/env python
import os
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

# this file is the main file to run the robot for picking the object and regrasping it on the table.
# the structure of the code will be following. Each function will be considered as action, and they will
# be used in the main function. Each of them will return true for sucess execution or false for failure.
# if one action needs to return some value which will be used by other action, then this action function will
# return multiple value.

# detect_table_and_placement will call the rail_segmentation server to detect the table and analyze which is the 
# open area for manipulation. 
# return: issuccess, transform_of_manipulation_position
def detect_table_and_placement(tf_helper, robot):
  # call the table searcher server for searching table
  rospy.wait_for_service('table_searcher/search_table')
  tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

  # analyze where to manipulate the object
  try:
    tableresult = tableSearcher()
  except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))
      return False, None

  r = R.from_quat([tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w])
  # need to adjust the rotation of the table
  original_r = r.as_euler('zyx')
  table_quaternion = R.from_euler('zyx', [0,original_r[1], original_r[2]]).as_quat()

  robot.addCollisionTable("table", tableresult.center.x, tableresult.center.y, tableresult.center.z, \
            table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
            tableresult.width * 1.8, tableresult.depth * 2, 0.001)
  # robot.attachTable("table")
  robot.addCollisionTable("table_base", tableresult.center.x, tableresult.center.y, tableresult.center.z - 0.3, \
          table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
          tableresult.width, tableresult.depth, 0.6)

  # show the position tf in rviz
  tf_helper.pubTransform("place_pos", ((tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z), \
                  (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w)))

  return True, [[[tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z], \
                  [tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w]]]

# detectino_object will detect the object and return the object pose
# return: issuccess, poseMat_of_object
def detection_object(tf_helper, robot, object_name, isSim):

  if not isSim:
    # call the pose estimation node
    # get the object searcher trigger to control the tracker
    rospy.wait_for_service('searchObject')
    objectSearcherTrigger = rospy.ServiceProxy('searchObject', SearchObject)

    ## launch the tracker
    objectSearcherTrigger(True, 1, Pose())

    raw_input("running pose estimation")

    try:
        # add the object to the moveit
        target_transform = tf_helper.getTransform('/base_link', '/' + object_name)
        robot.addCollisionObject(object_name + "_collision", target_transform, "objects/" + object_name + ".stl")
    except Exception as e:# if failure, then return False
        objectSearcherTrigger(False, 0, Pose())
        print e
        return False, None
    
    objectSearcherTrigger(False, 0, Pose())
  else:
    try:
      target_transform = tf_helper.getTransform('/base_link', '/' + object_name)
      robot.addCollisionObject(object_name + "_collision", target_transform, "objects/" + object_name + ".stl")
    except Exception as e:# if failure, then return False
        print e
        return False, None

  if not isSim:
    # stop updating octo map
    rospy.wait_for_service('stop_octo_map')
    try:
      octoclient = rospy.ServiceProxy('stop_octo_map', StopOctoMap)
      octoclient()
    except rospy.ServiceException as e:
      print ("Fail to stop octo map controller: %s"%e)
      return False, None

  return True, target_transform

# grasp_object will try to grasp the object
# input: object pose
# return: issuccess, grasp transform in object frame
def grasp_object( planner, object_pose, given_grasps=None, object_name = None):

  # this function will return both pre grasp pose and grasp pose in base link frame
  def find_grasping_point(planner, tran_base_object, gripper_pos_list=None):
    if gripper_pos_list == None:
      # filter out based on placement so we know which is the actuall grasp
      gripper_pos_list = planner.getGraspsbyPlacementPose(tran_base_object)
      if len(gripper_pos_list) == 0:
        # we have to try all grasps
        gripper_pos_list = planner.getAllGrasps()
    else:
      pre_definedGrasps = []
      for obj_grasp_pos, jaw_width in gripper_pos_list:
        pre_definedGrasps.append([obj_grasp_pos, jaw_width])
      possibleGrasps = planner.getAllGrasps()
      gripper_pos_list = []
      for t_pose, _ in pre_definedGrasps:
        grasp_inv_rot = np.transpose(t_pose[:3,:3])
        grasp_trans = t_pose[:,3:]

        grasp_temp = []
        rot_diff = []
        tran_diff = []
        for ppose, pwidth in possibleGrasps:
          rot_diff.append(np.linalg.norm(R.from_dcm(np.dot(grasp_inv_rot,ppose[:3,:3])).as_rotvec()))
          tran_diff.append(np.linalg.norm(ppose[:,3:] - grasp_trans))
          grasp_temp.append([ppose, pwidth, 0])
        tran_diff = softmax(tran_diff)
        rot_diff = softmax(rot_diff)
        for i in range(len(grasp_temp)):
          grasp_temp[i][2] = tran_diff[i] + rot_diff[i]

        def sortfun(e):
            return e[2]
        grasp_temp.sort(key=sortfun)

        gripper_pos_list.append((grasp_temp[0][0], grasp_temp[0][1]))

    print "Going through this many grasp pose: " ,len(gripper_pos_list)
    for i, (obj_grasp_pos, jaw_width) in enumerate(gripper_pos_list):

        obj_grasp_trans_obframe = getTransformFromPoseMat(obj_grasp_pos) #Tranfrom gripper posmatx to (trans,rot)
        # obj_grasp_trans_obframe = transformProduct(obj_grasp_trans_obframe, [[0.01,0,0],[0,0,0,1]]) # try to move the gripper forward little
        obj_pre_grasp_trans =  transformProduct(obj_grasp_trans_obframe, [[-0.06,0,0],[0,0,0,1]]) #adjust the grasp pos to be a little back 
        obj_pre_grasp_trans = transformProduct(tran_base_object, obj_pre_grasp_trans)
        obj_grasp_trans = transformProduct(tran_base_object, obj_grasp_trans_obframe)

        # need to ensure both grasp and pre-grasp is valid for robot
        # grasp_ik_result = robot.solve_ik_sollision_free_in_base(obj_grasp_trans, 80)

        # if grasp_ik_result == None:
        #     print 'check on grasp ', i
        #     continue

        pre_grasp_ik_result = robot.solve_ik_sollision_free_in_base(obj_pre_grasp_trans, 50)

        if pre_grasp_ik_result == None:
            print 'check on grasp ', i
            continue
        
        return obj_pre_grasp_trans, pre_grasp_ik_result, obj_grasp_trans, jaw_width, obj_grasp_trans_obframe
    # if not solution, then return None
    return None, None, None, None, None

  #Move to starting position
  robot.openGripper()

  #Go through all grasp pos and find a valid pos. 

  obj_pre_grasp_trans,  pre_grasp_ik_result, obj_grasp_trans, gripper_width, obj_grasp_trans_obframe = find_grasping_point(planner, object_pose, given_grasps)

  if pre_grasp_ik_result == None: # can't find any solution then return false.
      return False, None, None

  raw_input("found solution to pick")

  # move to pre grasp pose
  plan = robot.planto_pose(obj_pre_grasp_trans)
  robot.display_trajectory(plan)
  # raw_input("ready to pre-grasp")
  robot.execute_plan(plan)

  robot.switchController('my_cartesian_motion_controller', 'arm_controller')

  # move to grasp pose
  # raw_input("ready to grasp")
  while not rospy.is_shutdown():
    if robot.moveToFrame(obj_grasp_trans, True):
      break
    rospy.sleep(0.05)

  robot.switchController('arm_controller', 'my_cartesian_motion_controller')

  # close the gripper with proper width
  # print "grasp width = ", gripper_width
  # raw_input("ready to close grasp")
  robot.closeGripper()

  # attach object into the hand
  robot.attachManipulatedObject(object_name + "_collision")

  return True, obj_grasp_trans_obframe, gripper_width

# in hand pose estimation is currently optional. This function is used to estimate the object
# pose in hand.
# input: initialized estimated pose. If there is not initialize guess pose in hand then give None
# output: issuccess, preditect object pose in hand
def in_hand_pose_estimation(tf_helper, robot, guess_pose = None):

  object_pose_in_hand = None
  return True, object_pose_in_hand

# get init grasps will return a set of init grasp of the object
# return: issuccess, list of target grasps
def getInitGrasps(gdb, object_name):
    sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % object_name
    result = gdb.execute(sql)
    if not result:
        print "please add the object name to the table first!!"
        return False, None
    else:
        objectId = int(result[0][0])

    sql = "SELECT grasppose, jawwidth FROM initgrasps WHERE idobject = '%d'" % objectId
    initgrasp_result = gdb.execute(sql)

    # target grasps (grasp pose(in numpy format), jawwidth(in meter))
    init_grasps = []
    for grasppose, jawwidth in initgrasp_result:
        init_grasps.append((PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(grasppose)), float(jawwidth) / 1000))

    return True, init_grasps

# get target grasps will return a set of target grasp of the object
# return: issuccess, list of target grasps
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

def align_gripper(tf_helper, robot, manipulation_pos, height):
  currenttransform = tf_helper.getTransform('/base_link', '/gripper_link')
  currentpose = getMatrixFromQuaternionAndTrans(currenttransform[1], currenttransform[0])
  # align_pose = align_vectors(currentpose[:3, 0], [0,0,-1])
  align_pose = pointDown(currentpose)
  align_pose[0,3] = manipulation_pos[0][0][0]
  align_pose[1,3] = manipulation_pos[0][0][1]
  align_pose[2,3] = manipulation_pos[0][0][2] + height
  align_trans = getTransformFromPoseMat(align_pose)
  tf_helper.pubTransform("first_move", align_trans)

  robot.switchController('my_cartesian_motion_controller', 'arm_controller')
  while not rospy.is_shutdown():
    if robot.moveToFrame(align_trans, True):
      break
    rospy.sleep(0.05)
  robot.switchController('arm_controller', 'my_cartesian_motion_controller')

  return True

# pickup is the action to move the gripper up in the base_link frame
def pickup(tf_helper, height):
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

def placedown(placing_grasp):
  # move to pre placing position
  buffer = 0.02
  pre_placing_grasp = getTransformFromPoseMat(placing_grasp)
  pre_placing_grasp[0][2] += buffer

  # need to set path contraints too
  placing_plan = robot.planto_pose(pre_placing_grasp)
  if not placing_plan.joint_trajectory.points:
    print "found no plan to place down"
    return False

  robot.display_trajectory(placing_plan)
  print("Placing object down")
  raw_input("move to pre-place") 
  robot.execute_plan(placing_plan)
  
  # place the object down to the table
  robot.switchController('my_cartesian_motion_controller', 'arm_controller')
  while not rospy.is_shutdown():
    if robot.moveToFrame(getTransformFromPoseMat(placing_grasp), True):
      break
    rospy.sleep(0.05)
  robot.switchController('arm_controller', 'my_cartesian_motion_controller')
  return True

def placedown_with_constraints(placing_grasp):
  # move to pre placing position
  buffer = 0.03
  pre_placing_grasp = getTransformFromPoseMat(placing_grasp)
  pre_placing_grasp[0][2] += buffer

  # need to set path contraints too
  placing_plan = robot.planto_pose_with_constraints(pre_placing_grasp)
  # placing_plan = robot.planto_pose(pre_placing_grasp)
  if not placing_plan.joint_trajectory.points:
    print "found no plan to place down"
    return False

  robot.display_trajectory(placing_plan)
  print("Placing object down")
  raw_input("move to pre-place") 
  robot.execute_plan(placing_plan)
  
  # place the object down to the table
  robot.switchController('my_cartesian_motion_controller', 'arm_controller')
  while not rospy.is_shutdown():
    if robot.moveToFrame(getTransformFromPoseMat(placing_grasp), True):
      break
    rospy.sleep(0.05)
  robot.switchController('arm_controller', 'my_cartesian_motion_controller')
  return True

# regrasping will regrasp the object on the table
# input: list of possible manipulating place on the table and current object pose in the hand
# output: issuccess, final object pose in hand
def regrasping(tf_helper, robot, planner, dmgplanner, object_name=None,manipulation_position_list = None, target_grasps = None, init_grasp_transform_in_object_frame = None, init_jawwidth = None, base = None):

  # load the dmg planner from database
  dmgplanner.loadDB()

  # have a buffer on the manipulation position
  manipulation_position_list[0][0][2] += 0.005

  # get the manipulation position in pose mat format in the base link frame
  manipulation_position = getMatrixFromQuaternionAndTrans(manipulation_position_list[0][1], manipulation_position_list[0][0])

  # input pose should be numpy format in the base_link
  def feasible(inputpose):
    place_ik_result = robot.solve_ik_sollision_free_in_base(getTransformFromPoseMat(inputpose),30)

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

  # g = planner.PlacementG
  # labels = {n: g.nodes[n]['stable'] for n in g.nodes}
  # colors = [g.nodes[n]['stable'] for n in g.nodes]
  # nx.draw(g, with_labels=True, labels=labels, node_color=colors)
  # plt.draw()
  # plt.show()

  try:
    # this function will return multiple paths
    paths = planner.find_shortest_PlacementG_path()
  except Exception as e:
    print e
    return False, None

  print "find following paths for regrasping"
  for idx, p in enumerate(paths):
    print idx, p

  selected_i = input("please select the path to execute\n")

  path = paths[selected_i]

  # get the common grasps between placements on the path
  grasps_between_placements = planner.get_placement_grasp_trajectory(path)

  # planner.showgraph()

  for i, (currentplacementid, currentplacementtype) in enumerate(path):
    # if the current planning placement is end goal, then we can break the loop
    if currentplacementid == "end_g":
      break
    foundsolution = False

    for tableangle in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
      print "check table angle"

      rotationInZ = np.identity(4)
      rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()

      ## calculate the gripper pose on table
      real_placement = rotationInZ.dot(planner.getPlacementsById([currentplacementid])[0])
      placing_grasp = manipulation_position.dot(real_placement).dot(init_graspPose)

      # need to attach the object in hand properly
      robot.reattachManipulatedObject(object_name + "_collision", getTransformFromPoseMat(np.linalg.inv(init_graspPose)))

      # set the object is in the hand in moveit, then test
      if not feasible(placing_grasp):
        print "fail with placing grasp"
        continue

      # while not rospy.is_shutdown():

      #   tf_helper.pubTransform("current_place", getTransformFromPoseMat(manipulation_position.dot(real_placement)))

      #   gripper_id_list = planner.getGraspIdsByPlacementId(currentplacementid)
      #   current_grasps_of_placement = planner.getGraspsById(gripper_id_list) # List of pos that that match placement
      #   for e, current_grasp in enumerate(current_grasps_of_placement):
      #     tf_helper.pubTransform("current_grasp_" + str(e), getTransformFromPoseMat(manipulation_position.dot(real_placement).dot(current_grasp[0])))

        # for e, nextgrasp_candidate_pair in enumerate(grasps_between_placements[i]):
        #   if type(nextgrasp_candidate_pair) == tuple:
        #     nextgrasp_candidate = PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * nextgrasp_candidate_pair[0]) # convert the target grasp to correct format
        #   else:
        #     nextgrasp_candidate = planner.getGraspsById([nextgrasp_candidate_pair])[0][0] # convert grasp id to grasp pose
        #   tf_helper.pubTransform("place_grasp", getTransformFromPoseMat(placing_grasp))
        #   tf_helper.pubTransform("pick_grasp_" + str(e), getTransformFromPoseMat(manipulation_position.dot(real_placement).dot(nextgrasp_candidate)))
      
      for nextgrasp_candidate_pair in grasps_between_placements[i]:
        print "check next grasp"
        candidate_jawwidth = 0
        if type(nextgrasp_candidate_pair) == tuple:
          candidate_jawwidth = nextgrasp_candidate_pair[1] / 1000.0
          nextgrasp_candidate = PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * nextgrasp_candidate_pair[0]) # convert the target grasp to correct format
        else:
          candidate_jawwidth = planner.getGraspsById([nextgrasp_candidate_pair])[0][1]
          nextgrasp_candidate = planner.getGraspsById([nextgrasp_candidate_pair])[0][0] # convert grasp id to grasp pose


        # at this point, the next grasp candidate should be numpy pose matrix in object frame
        # and the candidate jawwidth should be in meter unit

        tf_helper.pubTransform("place_grasp", getTransformFromPoseMat(placing_grasp))
        tf_helper.pubTransform("pick_grasp", getTransformFromPoseMat(manipulation_position.dot(real_placement).dot(nextgrasp_candidate)))

        # check whether the next grasp is feasible or not
        robot.reattachManipulatedObject(object_name + "_collision", getTransformFromPoseMat(np.linalg.inv(nextgrasp_candidate)))
        if not feasible(manipulation_position.dot(real_placement).dot(nextgrasp_candidate)):
          robot.reattachManipulatedObject(object_name + "_collision", getTransformFromPoseMat(np.linalg.inv(init_graspPose)))
          continue
        robot.reattachManipulatedObject(object_name + "_collision", getTransformFromPoseMat(np.linalg.inv(init_graspPose)))

        # check the type of current placement
        if currentplacementtype == 0: # current placement is table
          
          # place the object down to the table
          if not placedown(placing_grasp):
            continue

          robot.openGripper()

          robot.detachManipulatedObject(object_name + "_collision")

          # move back
          current_trans = tf_helper.getTransform('/base_link', '/gripper_link')
          post_place_trans =  transformProduct(current_trans, [[-0.06,0,0],[0,0,0,1]])

          robot.switchController('my_cartesian_motion_controller', 'arm_controller')

          while not rospy.is_shutdown():
            if robot.moveToFrame(post_place_trans, True):
              break
            rospy.sleep(0.05)

          robot.switchController('arm_controller', 'my_cartesian_motion_controller')

          # move to pick trans
          pick_trans = getTransformFromPoseMat(manipulation_position.dot(real_placement).dot(nextgrasp_candidate))

          pre_pick_trans =  transformProduct(pick_trans, [[-0.06,0,0],[0,0,0,1]]) #adjust the grasp pos to be a little back 

          picking_plan = robot.planto_pose(pre_pick_trans)
          robot.display_trajectory(picking_plan)
          # raw_input("ready to pick")
          robot.execute_plan(picking_plan)

          # move forward
          robot.switchController('my_cartesian_motion_controller', 'arm_controller')

          while not rospy.is_shutdown():
            if robot.moveToFrame(pick_trans, True):
              break
            rospy.sleep(0.05)

          robot.switchController('arm_controller', 'my_cartesian_motion_controller')

        else: # current placement is unstable
          # tf_helper.pubTransform("place_grasp", getTransformFromPoseMat(placing_grasp))
          # tf_helper.pubTransform("pick_grasp", getTransformFromPoseMat(manipulation_position.dot(real_placement).dot(nextgrasp_candidate)))
          # get a trajectory of regrasping pose in the object frame with numpy pose mat format
          dmgresult = dmgplanner.getTrajectory(init_graspPose, nextgrasp_candidate, candidate_jawwidth, real_placement, base)
          if dmgresult == None:
            continue
          else:
            if not placedown(placing_grasp):
              continue

            # move the gripper according to the dmg result
            robot.detachManipulatedObject(object_name + "_collision")
            # open the gripper according to the next grasp width
            robot.setGripperWidth(candidate_jawwidth - 0.003)
            #Convert from object frame to torso frame
            object_in_base_link = manipulation_position.dot(real_placement)

            robot.switchController('my_cartesian_motion_controller', 'arm_controller')
            for graspPos in dmgresult:

              regraspTran = getTransformFromPoseMat(object_in_base_link.dot(graspPos))
              # publish the next regrasp pos in the tf for debug
              tf_helper.pubTransform("regrasp pos", regraspTran)
              
              # move to regrasp pose
              while not rospy.is_shutdown():
                if robot.moveToFrame(regraspTran, True):
                  break
                rospy.sleep(0.05)

            robot.switchController('arm_controller', 'my_cartesian_motion_controller')

        robot.closeGripper()
        robot.attachManipulatedObject(object_name + "_collision")
        pickup(tf_helper, 0.1)
        init_graspPose = nextgrasp_candidate
        foundsolution = True
        break
      if foundsolution:
        break
    if not foundsolution:
      print "we need to replan the path"
      break
    else:
      print "placing done"
  

  current_object_pose_in_hand = None
  return True, current_object_pose_in_hand

def move_arm_to_startPos(robot):
  pass


if __name__=='__main__':

  # object_name = "cup"
  # object_name = "book"
  # object_name = "box"
  # object_name = "cuboid"
  object_name = "almonds_can"
  isSim = False


  rospy.init_node('test_node')
  robot = Fetch_Robot(sim=isSim)
  tf_helper = TF_Helper()
  
  base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
  this_dir, this_filename = os.path.split(__file__)   
  objpath = os.path.join(os.path.split(this_dir)[0], "objects", object_name + ".stl") 
  handpkg = fetch_grippernm  #SQL grasping database interface 
  gdb = db.GraspDB()   #SQL grasping database interface
  planner = RegripPlanner(objpath, handpkg, gdb)
  dmg_planner = ff_regrasp_planner(objpath, handpkg, gdb)   

  result, manipulation_trans_on_table = detect_table_and_placement(tf_helper, robot)
  print "table detection and placement analysis"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  result, object_pose_in_base_link = detection_object(tf_helper, robot, object_name=object_name, isSim=isSim)
  print "object pose estimation"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()
  
  raw_input("go grasp object")

  # extract the list of init grasps
  result, init_grasps = getInitGrasps(gdb, object_name=object_name)
  print "get init grasps"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  result, init_grasp_transform_in_object_frame, init_jawwidth = grasp_object(planner, object_pose_in_base_link, given_grasps = init_grasps, object_name=object_name)
  print "grasp object"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()
  
  raw_input("Ready to pick up object?")
  
  result = pickup(tf_helper, 0.2)
  print "pick up object"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  # # need to move to pre-place-plan-position
  # result = align_gripper(tf_helper, robot, manipulation_trans_on_table, 0.35)
  # print "align the gripper down"
  # if result:
  #   print "---SUCCESS---"
  # else:
  #   print "---FAILURE---"
  #   exit()
  

  '''
  result, object_pose_in_hand = in_hand_pose_estimation(tf_helper, robot, init_grasp_transform_in_object_frame)
  print "in-hand object pose estimation"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()
  '''

  raw_input("ready to regrasp")

  # extract the list of target grasps
  result, target_grasps = getTargetGrasps(gdb, object_name=object_name)
  print "get target grasps"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  result, object_pose_in_hand = regrasping(tf_helper, robot, planner, dmg_planner, object_name, manipulation_trans_on_table, target_grasps, init_grasp_transform_in_object_frame, init_jawwidth, base=base)
  print "regrasping"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    robot.detachManipulatedObject(object_name + "_collision")
    exit()

  robot.detachManipulatedObject(object_name + "_collision")

  # # show the collision net
  # def updateworld(world, task):
  #     world.doPhysics(globalClock.getDt())
  #     return task.cont

  # debugNode = BulletDebugNode('Debug')
  # debugNode.showWireframe(True)
  # debugNode.showConstraints(True)
  # debugNode.showBoundingBoxes(False)
  # debugNode.showNormals(False)
  # bullcldrnp = base.render.attachNewNode("bulletcollider")
  # debugNP = bullcldrnp.attachNewNode(debugNode)
  # debugNP.show()
  # planner.bulletworldhplowest.setDebugNode(debugNP.node())
  # taskMgr.add(updateworld, "updateworld", extraArgs=[planner.bulletworldhplowest], appendTask=True)

  # debugNode1 = BulletDebugNode('Debug1')
  # debugNode1.showWireframe(True)
  # debugNode1.showConstraints(True)
  # debugNode1.showBoundingBoxes(False)
  # debugNode1.showNormals(False)
  # bullcldrnp1 = base.render.attachNewNode("bulletcollider1")
  # debugNP1 = bullcldrnp1.attachNewNode(debugNode1)
  # debugNP1.show()
  # planner.bulletworld.setDebugNode(debugNP1.node())
  # taskMgr.add(updateworld, "updateworld", extraArgs=[planner.bulletworld], appendTask=True)

  # base.run()

