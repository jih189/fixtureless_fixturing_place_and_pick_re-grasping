#!/usr/bin/python
# here is the main file

import os
import itertools

from numpy.lib.function_base import place

import MySQLdb as mdb
import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.core import *
from shapely.geometry import LinearRing
from shapely.geometry import Point
from shapely.geometry import Polygon

from manipulation.grip.fetch_gripper import fetch_grippernm

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh
from pandaplotutils import pandageom as pg
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from database import dbaccess as db

import math

from regrasp_planner import RegripPlanner

import rospy

from std_msgs.msg import UInt16
from geometry_msgs.msg import Pose
from rail_segmentation.srv import SearchTable
from icra20_manipulation_pose.srv import SearchObject

from visualization_msgs.msg import Marker
from fetch_robot import Fetch_Robot

import tf
from scipy.spatial.transform import Rotation as R

from  tf_util import TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans, findCloseTransform

trackstamp = 0

def showPlacePos(rotation, trans):
   marker = Marker()
   marker.header.frame_id = "base_link"
   marker.type = marker.MESH_RESOURCE
   marker.action = marker.ADD
   marker.scale.x = 0.001
   marker.scale.y = 0.001
   marker.scale.z = 0.001
   marker.mesh_resource = "package://regrasp_planner/scripts/objects/cuboid.stl"
   marker.color.a = 1.0
   marker.color.r = 0.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.x = rotation[0]
   marker.pose.orientation.y = rotation[1]
   marker.pose.orientation.z = rotation[2]
   marker.pose.orientation.w = rotation[3]
   marker.pose.position.x = trans[0]
   marker.pose.position.y = trans[1]
   marker.pose.position.z = trans[2]
   marker.id = 1

   return marker

def showManipulationPos(x, y, z):
   cylinderHeight = 0.01
   marker = Marker()
   marker.header.frame_id = "base_link"
   marker.id = 0
   marker.type = marker.CYLINDER
   marker.action = marker.ADD
   marker.pose.position.x = x
   marker.pose.position.y = y
   marker.pose.position.z = z + cylinderHeight/2.0

   marker.color.a = 0.4
   marker.color.r = 0.0
   marker.color.g = 0.0
   marker.color.b = 1.0

   marker.scale.x = 0.05
   marker.scale.y = 0.05
   marker.scale.z = cylinderHeight

   marker.pose.orientation.x = 0.0
   marker.pose.orientation.y = 0.0
   marker.pose.orientation.z = 0.0
   marker.pose.orientation.w = 1.0

   return marker

def trackstamp_callback(input):
   global trackstamp
   trackstamp = input.data

if __name__=='__main__':
   rospy.init_node('main_node')

   PRE_PLACE_HEIGHT = 0.03

   tf_helper = TF_Helper()

   rospy.Subscriber("trackstamp", UInt16, trackstamp_callback)

   base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
   gdb = db.GraspDB()
   this_dir, this_filename = os.path.split(__file__)
   objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cuboid.stl")

   # # pre-compute the re-grasp graph
   handpkg = fetch_grippernm
   planner = RegripPlanner(objpath, handpkg, gdb)
   planner.generateGraph()
   # # planner.showgraph()

   # get the object searcher trigger to control the tracker
   rospy.wait_for_service('searchObject')
   objectSearcherTrigger = rospy.ServiceProxy('searchObject', SearchObject)

   # get table information
   rospy.wait_for_service('table_searcher/search_table')
   tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

   try:
      tableresult = tableSearcher()
   except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

   ## move the end-effector into robot's camera view
   robot = Fetch_Robot()

   ## show the manipulation position in rviz
   # marker = showManipulationPos(tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z)
   # marker_pub.publish(marker)
   # add the table as a collision object into the world
   robot.addCollisionTable("table", tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z, \
         tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w, \
         tableresult.width, tableresult.depth, tableresult.height)

   robot.goto_pose(0.34969, 0.20337, 0.92054, 0.081339, 0.012991, -0.63111, 0.77131)

   ## launch the tracker
   objectSearcherTrigger(True, 1, Pose())

   foundObject = True
   hand_grasp_pose = None
   pose_in_hand = Pose()
   try:
      # attand the object as a part of the arm
      trans,rot = tf_helper.getTransform('/base_link', '/object')
      robot.addManipulatedObject("object", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], "objects/cuboid.stl")

      ## get the current grasp in the object frame
      trans,rot = tf_helper.getTransform('/object', '/gripper_link')
      hand_grasp_pose = tf.TransformerROS().fromTranslationRotation(trans, rot)
      startpose = Mat4( hand_grasp_pose[0][0],hand_grasp_pose[1][0],hand_grasp_pose[2][0],0.0, \
                        hand_grasp_pose[0][1],hand_grasp_pose[1][1],hand_grasp_pose[2][1],0.0, \
                        hand_grasp_pose[0][2],hand_grasp_pose[1][2],hand_grasp_pose[2][2],0.0, \
                        hand_grasp_pose[0][3] * 1000,hand_grasp_pose[1][3] * 1000,hand_grasp_pose[2][3] * 1000,1.0)
      ## convert it to be used in panda3d
      startpose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * startpose

      ## get current hand width
      trans, rot = tf_helper.getTransform('/gripper_link', '/r_gripper_finger_link')
      starthandwidth = trans[1] * 1000

      ## get the object pose in hand
      tf_helper.getPose('/gripper_link', '/object', pose_in_hand)

   except:
      print "fail to detect the object"
      foundObject = False

   ## stop the tracker
   objectSearcherTrigger(False, 0, Pose())

   ## set goal grasp pose in object frame
   goalpose = Mat4(1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,53.3199386597,-8.46575927734,-4.76837158203e-07,1.0)
   goalhandwidth = 38.999997139
   planner.addGoalGrasp(goalhandwidth, goalpose)

   if foundObject:
      planner.addStartGrasp(starthandwidth, startpose)
      # planner.showgraph()
      placementsequence = planner.searchPath()

      planner.showHand(starthandwidth, startpose, base)
      print "placement sequence ", placementsequence

      current_place_count = 0

      if len(placementsequence) == 0:
         print "there is no way to place the object"
      else:
         # extract one of the placement sequence plan
         plan1 = placementsequence[0]
         # get the placement sequence of the plan
         placements = planner.getPlacements(plan1)

         # get the manipulation position on the table
         tablepos = getMatrixFromQuaternionAndTrans(tableresult.orientation, tableresult.centroid)
         tablepos[2,3] += PRE_PLACE_HEIGHT

         for p in range(len(placements)):
            # get current placement on the table
            current_place = placements[p]

            # move the pre placement position
            placeGripperPoses = []
            possibleGripperPosesInTable = []
            # set different placement angle for planing
            for tableangle in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
               rotationInZ = np.identity(4)
               rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()

               ## calculate the gripper pose on table
               gripperP = tablepos.dot(rotationInZ).dot(current_place).dot(hand_grasp_pose)
               rotationQ = tf.transformations.quaternion_from_matrix(gripperP) # this function takes 4*4 matrix
               transformT = np.squeeze(np.asarray(gripperP[:3,3]))

               placeGripperPoses.append((transformT, rotationQ))

               gripperP = rotationInZ.dot(current_place).dot(hand_grasp_pose)
               rotationQ = tf.transformations.quaternion_from_matrix(gripperP) # this function takes 4*4 matrix
               transformT = np.squeeze(np.asarray(gripperP[:3,3]))

               possibleGripperPosesInTable.append((transformT, rotationQ))

            # ## find the plan to place the object to the pre-place
            plan = robot.planto_poses(placeGripperPoses)
            # # visualize the plan
            robot.display_trajectory(plan)

            raw_input("ready to place!!")

            # move the gripper to pre-place position
            robot.execute_plan(plan)

            objectSearcherTrigger(True, 2, pose_in_hand)

            ## need to update the object pose in hand
            try:
               # attand the object as a part of the arm
               trans, rot = tf_helper.getTransform('/base_link', '/object')
               robot.addManipulatedObject("object", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], "objects/cuboid.stl")
            except:
               print "fail to detect the object"
               foundObject = False
               break

            raw_input("start to place!!")

            # place from pre-place to place with cartesian motion controller
            ## need to switch to cartesian motion controller
            robot.switchController('my_cartesian_motion_controller', 'arm_controller')

            # get gripper pose for place in table frame
            place_hand_in_table = findCloseTransform(tf_helper.getTransform('/original_table', '/gripper_link'), possibleGripperPosesInTable)

            current_stamp = trackstamp - 1

            while not rospy.is_shutdown():
               if current_stamp < trackstamp:
                  current_stamp += 1

                  # get current gripper pose in cartisian motion base
                  gripper_pose = tf_helper.getTransform('/torso_lift_link', '/gripper_link')

                  # get the current table relate to the hand
                  predicted_table_in_hand = tf_helper.getTransform('/predicted_hand', '/predicted_table')

                  predict_place = transformProduct(predicted_table_in_hand, place_hand_in_table)
                  targetposition, targetorientation = transformProduct(gripper_pose, predict_place)

                  if(robot.moveToFrame(targetposition, targetorientation)):
                     break
                  rospy.sleep(0.05)
                  # trans_error, rot_error = robot.getError()
                  # print "error trans ", trans_error, " rot ", rot_error
               else:
                  rospy.sleep(1.0)
            objectSearcherTrigger(False, 0, Pose())

            raw_input("finish place")

            # detach the object from gripper in moveit
            robot.detachManipulatedObject("object")

            # need to open the gripper and move the gripper away from the object
            robot.openGripper()

            # get current gripper pose in cartisian motion base
            gripper_pose = tf_helper.getTransform('/torso_lift_link', '/gripper_link')

            targetposition, targetorientation = transformProduct(gripper_pose, [[-0.07,0,0],[0,0,0,1]])

            while not rospy.is_shutdown():
               if(robot.moveToFrame(targetposition, targetorientation)):
                  break
               rospy.sleep(0.05)

            robot.switchController('arm_controller', 'my_cartesian_motion_controller')

            raw_input("release object")

            # plan for the next grasp

            break

            

         # open gripper and unattanch the object from the end-effector

         # if len(placementsequence) == current_place_count + 1:
         #    # this is final placement, so we need to grasp the object with target grasp pose
         #    pass
         # else:
         #    next_placement = placementsequence[current_place_count + 1]

         #    current_place_count += 1

   # show object state in hand
   planner.plotObject(base)
   
   # def myFunction(task):
   

   #    # (trans,rot) = listener.lookupTransform('/object', '/gripper_link', rospy.Time())
   #    # hand_grasp_pose = tf.TransformerROS().fromTranslationRotation(trans, rot)

   #    # ## get current hand width
   #    # (trans,rot) = listener.lookupTransform('/gripper_link', '/r_gripper_finger_link', rospy.Time())

   #    # startpose = Mat4( hand_grasp_pose[0][0],hand_grasp_pose[1][0],hand_grasp_pose[2][0],0.0, \
   #    #                   hand_grasp_pose[0][1],hand_grasp_pose[1][1],hand_grasp_pose[2][1],0.0, \
   #    #                   hand_grasp_pose[0][2],hand_grasp_pose[1][2],hand_grasp_pose[2][2],0.0, \
   #    #                   hand_grasp_pose[0][3] * 1000,hand_grasp_pose[1][3] * 1000,hand_grasp_pose[2][3] * 1000,1.0)
   #    # starthandwidth = trans[1] * 1000

   #    # ## convert it to be used in panda3d
   #    # startpose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * startpose

   #    # planner.showHand(starthandwidth, startpose, base)
   #    return task.again

   # myTask = taskMgr.doMethodLater(0.1, myFunction, 'tickTask')
  
   base.run()