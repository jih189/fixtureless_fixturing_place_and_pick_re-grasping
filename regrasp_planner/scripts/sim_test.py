#!/usr/bin/env python
import os
import rospy
from fetch_robot import Fetch_Robot
from tf_util import PosMat_t_PandaPosMax, TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat, PandaPosMax_t_PosMat
from rail_segmentation.srv import SearchTable
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.fetch_gripper import fetch_grippernm
from regrasp_planner import RegripPlanner
from database import dbaccess as db
from utils import dbcvt as dc
import numpy as np
from scipy.spatial.transform import Rotation as R

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

  # add the table into moveit
  tran_base_Table = tf_helper.getTransform('/base_link', '/Table') 
  robot.addCollisionObject("table_collsion", tran_base_Table,'objects/Table.stl', size_scale=1.0)

  # analyze where to manipulate the object
  try:
    tableresult = tableSearcher()
  except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))
      return False, None

  # show the position tf in rviz
  tf_helper.pubTransform("place_pos", ((tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z), \
                  (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w)))

  return True, [((tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z), \
                  (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w))]

# detectino_object will detect the object and return the object pose
# return: issuccess, poseMat_of_object
def detection_object(tf_helper, robot):

  # call the pose estimation node
  # TODO
  # if failure, then return False

  # add the cup into the moveit
  tran_base_Cup = tf_helper.getTransform('/base_link', '/Cup') #return tuple (trans,rot) of parent_lin to child_link
  robot.addCollisionObject("cup_collsion", tran_base_Cup,'objects/cup.stl')

  return True, tran_base_Cup

# grasp_object will try to grasp the object
# input: object pose
# return: issuccess, object_pose_in_hand
def grasp_object(robot, planner, object_pose):

  # this function will return both pre grasp pose and grasp pose in base link frame
  def find_grasping_point(planner,tran_base_object):
    # filter out based on placement so we know which is the actuall grasp
    gripper_pos_list = planner.getGraspsbyPlacementPose(tran_base_object)

    print("Going through this many grasp pose: " ,len(gripper_pos_list))
    for i, (obj_grasp_pos, jaw_width) in enumerate(gripper_pos_list):

        obj_grasp_trans_obframe = getTransformFromPoseMat(obj_grasp_pos) #Tranfrom gripper posmatx to (trans,rot)
        obj_pre_grasp_trans =  transformProduct(obj_grasp_trans_obframe, [[-0.06,0,0],[0,0,0,1]]) #adjust the grasp pos to be a little back 
        obj_pre_grasp_trans = transformProduct(tran_base_object, obj_pre_grasp_trans)
        obj_grasp_trans = transformProduct(tran_base_object, obj_grasp_trans_obframe)

        # need to ensure both grasp and pre-grasp is valid for robot
        grasp_ik_result = robot.solve_ik_sollision_free_in_base(obj_grasp_trans, 10)

        if grasp_ik_result == None:
            print 'check on grasp ', i
            continue

        pre_grasp_ik_result = robot.solve_ik_sollision_free_in_base(obj_pre_grasp_trans, 10)

        if pre_grasp_ik_result == None:
            print 'check on grasp ', i
            continue
        
        return obj_pre_grasp_trans, pre_grasp_ik_result, obj_grasp_trans, jaw_width, obj_grasp_trans_obframe
    # if not solution, then return None
    return None, None, None, None

  #Move to starting position
  robot.openGripper()

  #Go through all grasp pos and find a valid pos. 
  obj_pre_grasp_trans, pre_grasp_ik_result, obj_grasp_trans, gripper_width, obj_grasp_trans_obframe = find_grasping_point(planner, object_pose)

  if pre_grasp_ik_result == None: # can't find any solution then return false.
      return False, None

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
  print "grasp width = ", gripper_width
  # raw_input("ready to close grasp")
  robot.closeGripper()

  # attach object into the hand
  robot.attachManipulatedObject("cup_collsion")

  return True, obj_grasp_trans_obframe, gripper_width, 

# in hand pose estimation is currently optional. This function is used to estimate the object
# pose in hand.
# input: initialized estimated pose. If there is not initialize guess pose in hand then give None
# output: issuccess, preditect object pose in hand
def in_hand_pose_estimation(tf_helper, robot, guess_pose = None):

  object_pose_in_hand = None
  return True, object_pose_in_hand

# get target grasps will return a set of target grasp of the object
# return: issuccess, list of target grasps
def getTargetGrasps(gdb):
    sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % "cup"
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
        target_grasps.append((PandaPosMax_t_PosMat(dc.strToMat4(grasppose)), float(jawwidth) / 1000))

    return True, target_grasps

# regrasping will regrasp the object on the table
# input: list of possible manipulating place on the table and current object pose in the hand
# output: issuccess, final object pose in hand
def regrasping(tf_helper, robot, planner, manipulation_position_list = None, target_grasps = None, init_object_pose_in_hand = None, init_jawwidth = None):
   
  """Pick up object"""
  #init_object_pose_in_hand = transformProduct(init_object_pose_in_hand, [[0,0,.02],[0,0,0,1]]) #Move up in the z by 6cm?
  target_transform = tf_helper.getTransform('/base_link', '/Cup')
  target_transform[0][2] += 0.06
  obj_grasp_baslink_trans = transformProduct(target_transform, init_object_pose_in_hand)

  robot.switchController('my_cartesian_motion_controller', 'arm_controller')

  # move to grasp pose
  # raw_input("ready to grasp")
  while not rospy.is_shutdown():
    if robot.moveToFrame(obj_grasp_baslink_trans, True):
      break
    rospy.sleep(0.05)

  robot.switchController('arm_controller', 'my_cartesian_motion_controller')
  
  """Regrasp Object"""

  init_graspPose = getMatrixFromQuaternionAndTrans(init_object_pose_in_hand[1],init_object_pose_in_hand[0])
  planner.CreatePlacementGraph()
  planner.addStartGrasp(PosMat_t_PandaPosMax(init_graspPose),init_jawwidth)
  #TODO: An optimal path to the target grasp should always be 1 and will always exsist for our objects
  # This is not true for other objects. 
  for grasp, jawwidth in target_grasps:
      planner.addGoalGrasp(PosMat_t_PandaPosMax(grasp), jawwidth)

      try: 
        path = planner.find_shortest_PlacementG_path()
      except Exception as e:
        print e
        print("No path between two grasp")
        continue
      grasp_t = planner.get_placement_grasp_trajectory(path)

      if len(path) == 1:
          break

  print "path"
  print path

  tableTrans = manipulation_position_list[0] # trans of table in base_link
  tablePoseMat = getMatrixFromQuaternionAndTrans(tableTrans[1], tableTrans[0])
  base_link_in_torso_lift = tf_helper.getPoseMat('/torso_lift_link', '/base_link')

  for current_placement_id, current_placement_type in path:
    """"----- Placing the object on table according to placement_id"""
    currentPlacementPoseMat = planner.getPlacementsById([path[current_placement_id][0]])[0]
    up_matrix = np.identity(4)
    up_matrix[2,3] = 0.02
    pre_tablePoseMat = tablePoseMat.dot(up_matrix)

    for tableangle in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
      rotationInZ = np.identity(4)
      rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()

      ## calculate the gripper pose on table
      place_pose_in_table = rotationInZ.dot(currentPlacementPoseMat)

      # publish the tf
      test_tf = getTransformFromPoseMat(tablePoseMat.dot(place_pose_in_table).dot(init_graspPose))
      tf_helper.pubTransform("test", test_tf)
      place_pose_tmp = base_link_in_torso_lift.dot(tablePoseMat).dot(place_pose_in_table).dot(init_graspPose)
      place_pose = getTransformFromPoseMat(place_pose_tmp)
      place_ik_result = robot.solve_ik_collision_free(place_pose, 300)

      if place_ik_result != None:
        break
      
    if place_ik_result != None:
      robot.display_place_robot_state(place_ik_result)
    else:
      print "no solution found for placing the object"
      robot.detachManipulatedObject("cup_collsion")
      return False, None

    # get the placement pose in the base_link frame
    torso_lift_in_base_link = tf_helper.getTransform('/base_link', '/torso_lift_link')
    place_pose = transformProduct(torso_lift_in_base_link, place_pose)

    # ## find the plan to place the object to the pre-place
    plan = robot.planto_pose(place_pose)
    # # visualize the plan
    robot.display_trajectory(plan)

    robot.execute_plan(plan)

  robot.detachManipulatedObject("cup_collsion")
  
  current_object_pose_in_hand = None
  return True, current_object_pose_in_hand

if __name__=='__main__':
  rospy.init_node('test_node')
  robot = Fetch_Robot(sim=True)
  tf_helper = TF_Helper()

  base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
  this_dir, this_filename = os.path.split(__file__)   
  objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cup.stl") 
  handpkg = fetch_grippernm  #SQL grasping database interface 
  gdb = db.GraspDB()   #SQL grasping database interface
  planner = RegripPlanner(objpath, handpkg, gdb)      

  result, manipulation_trans_on_table = detect_table_and_placement(tf_helper, robot)
  print "table detection and placement analysis"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  result, object_pose_in_base_link = detection_object(tf_helper, robot)
  print "object pose estimation"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  result, init_object_pose_in_hand, init_jawwidth = grasp_object(robot, planner, object_pose_in_base_link)
  print "grasp object"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  '''
  result, object_pose_in_hand = in_hand_pose_estimation(tf_helper, robot, init_object_pose_in_hand)
  print "in-hand object pose estimation"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()
  '''

  # extract the list of target grasps
  result, target_grasps = getTargetGrasps(gdb)
  print "get target grasps"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

  result, object_pose_in_hand = regrasping(tf_helper, robot, planner, manipulation_trans_on_table, target_grasps, init_object_pose_in_hand, init_jawwidth)
  print "regrasping"
  if result:
    print "---SUCCESS---"
  else:
    print "---FAILURE---"
    exit()

