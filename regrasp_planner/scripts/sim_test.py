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
# return: issuccess
def grasp_object(tf_helper, robot, object_pose):

  robot.openGripper()

  return True

if __name__=='__main__':
    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=True)
    tf_helper = TF_Helper()

    result, manipulation_pose_on_table = detect_table_and_placement(tf_helper, robot)
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

    result = grasp_object(tf_helper, robot, object_pose_in_base_link)
    print "grasp object"
    if result:
      print "---SUCCESS---"
    else:
      print "---FAILURE---"
      exit()