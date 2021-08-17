#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from tf_util import TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans
from rail_segmentation.srv import SearchTable


# this file is the main file to run the robot for picking the object and regrasping it on the table.
# the structure of the code will be following. Each function will be considered as action, and they will
# be used in the main function. Each of them will return true for sucess execution or false for failure.
# if one action needs to return some value which will be used by other action, then this action function will
# return multiple value.

# detect_table_and_placement will call the rail_segmentation server to detect the table and analyze which is the 
# open area for manipulation. 
# return: issuccess, poseMat_of_manipulation_position
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
      
  # get the pose mat from base link to table
  tableposMat = getMatrixFromQuaternionAndTrans(tableresult.orientation, tableresult.centroid)

  # show the position tf in rviz
  tf_helper.pubTransform("place_pos", ((tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z), \
                  (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w)))

  return True, tableposMat

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
def grasp_object(tf_helper, robot, object_pose):

  robot.openGripper()

  # search all possible grasp pose with given object_pose

  # if find the grasp pose
  #   move to the grasp pose
  # else
  # return False, None

  object_pose_in_hand = None

  robot.closeGripper()

  return True, object_pose_in_hand

# in hand pose estimation is currently optional. This function is used to estimate the object
# pose in hand.
# input: initialized estimated pose. If there is not initialize guess pose in hand then give None
# output: issuccess, preditect object pose in hand
def in_hand_pose_estimation(tf_helper, robot, guess_pose = None):

  object_pose_in_hand = None
  return True, object_pose_in_hand

# regrasping will regrasp the object on the table
# input: list of possible manipulating place on the table and current object pose in the hand
# output: issuccess, final object pose in hand
def regrasping(tf_helper, robot, manipulation_position_list = None, target_grasps = None, init_object_pose_in_hand = None):

  current_object_pose_in_hand = None
  return True, current_object_pose_in_hand

if __name__=='__main__':
    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=True)
    tf_helper = TF_Helper()

    result, manipulation_poses_on_table = detect_table_and_placement(tf_helper, robot)
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

    result, init_object_pose_in_hand = grasp_object(tf_helper, robot, object_pose_in_base_link)
    print "grasp object"
    if result:
      print "---SUCCESS---"
    else:
      print "---FAILURE---"
      exit()

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
    target_grasps = None

    result, object_pose_in_hand = regrasping(tf_helper, robot, manipulation_poses_on_table, target_grasps, init_object_pose_in_hand)
    print "regrasping"
    if result:
      print "---SUCCESS---"
    else:
      print "---FAILURE---"
      exit()

