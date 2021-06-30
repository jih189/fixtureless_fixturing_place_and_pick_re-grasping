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

def transformProduct(t1, t2):
   trans1, rot1= t1
   trans1_mat = tf.transformations.translation_matrix(trans1)
   rot1_mat = tf.transformations.quaternion_matrix(rot1)
   mat1 = np.dot(trans1_mat, rot1_mat)

   trans2, rot2 = t2
   trans2_mat = tf.transformations.translation_matrix(trans2)
   rot2_mat = tf.transformations.quaternion_matrix(rot2)
   mat2 = np.dot(trans2_mat, rot2_mat)

   mat3 = np.dot(mat1, mat2)
   trans3 = tf.transformations.translation_from_matrix(mat3)
   rot3 = tf.transformations.quaternion_from_matrix(mat3)

   return trans3, rot3
   

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

   listener = tf.TransformListener()
   br = tf.TransformBroadcaster()
   rospy.Subscriber("trackstamp", UInt16, trackstamp_callback)
   
   # marker_pub = rospy.Publisher("/manipulate_pos", Marker, queue_size=1)
   goal_pub1 = rospy.Publisher("/goal_object1", Marker, queue_size=1)
   goal_pub2 = rospy.Publisher("/goal_object2", Marker, queue_size=1)
   goal_pub3 = rospy.Publisher("/goal_object3", Marker, queue_size=1)
   goal_pub4 = rospy.Publisher("/goal_object4", Marker, queue_size=1)
   goal_pub5 = rospy.Publisher("/goal_object5", Marker, queue_size=1)
   goal_pub6 = rospy.Publisher("/goal_object6", Marker, queue_size=1)
   goal_pub7 = rospy.Publisher("/goal_object7", Marker, queue_size=1)
   goal_pub8 = rospy.Publisher("/goal_object8", Marker, queue_size=1)

   base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
   gdb = db.GraspDB()
   this_dir, this_filename = os.path.split(__file__)
   objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cuboid.stl")

   # # pre-compute the re-grasp graph
   handpkg = fetch_grippernm
   planner = RegripPlanner(objpath, handpkg, gdb)
   planner.generateGraph()
   # # planner.showgraph()

   # manipulation area selection, given the pointcloud to generate the position to manipulate
   rospy.wait_for_service('searchObject')
   objectSearcherTrigger = rospy.ServiceProxy('searchObject', SearchObject)

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
      listener.waitForTransform('/base_link', '/object', rospy.Time(), rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/base_link', '/object', rospy.Time())
      robot.addManipulatedObject("object", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], "objects/cuboid.stl")

      ## get the current grasp in the object frame
      listener.waitForTransform('/object', '/gripper_link', rospy.Time(), rospy.Duration(4.0))
      rospy.sleep(3.0) # it needs wait 5 seconds for the robot to update the object's pose
      (trans,rot) = listener.lookupTransform('/object', '/gripper_link', rospy.Time())
      hand_grasp_pose = tf.TransformerROS().fromTranslationRotation(trans, rot)
      startpose = Mat4( hand_grasp_pose[0][0],hand_grasp_pose[1][0],hand_grasp_pose[2][0],0.0, \
                        hand_grasp_pose[0][1],hand_grasp_pose[1][1],hand_grasp_pose[2][1],0.0, \
                        hand_grasp_pose[0][2],hand_grasp_pose[1][2],hand_grasp_pose[2][2],0.0, \
                        hand_grasp_pose[0][3] * 1000,hand_grasp_pose[1][3] * 1000,hand_grasp_pose[2][3] * 1000,1.0)
      ## convert it to be used in panda3d
      startpose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * startpose

      ## get current hand width
      (trans,rot) = listener.lookupTransform('/gripper_link', '/r_gripper_finger_link', rospy.Time())
      starthandwidth = trans[1] * 1000

      ## get the object pose in hand
      listener.waitForTransform('/gripper_link', '/object', rospy.Time(), rospy.Duration(4.0))
      (trans,rot) = listener.lookupTransform('/gripper_link', '/object',rospy.Time())
      pose_in_hand.position.x = trans[0]
      pose_in_hand.position.y = trans[1]
      pose_in_hand.position.z = trans[2]

      pose_in_hand.orientation.x = rot[0]
      pose_in_hand.orientation.y = rot[1]
      pose_in_hand.orientation.z = rot[2]
      pose_in_hand.orientation.w = rot[3]

   except:
      print "fail to detect the object"
      foundObject = False

   ## stop the tracker
   objectSearcherTrigger(False, 0, Pose())

   ## set goal grasp pose in object frame
   goalpose = Mat4(1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,53.3199386597,-8.46575927734,-4.76837158203e-07,1.0)
   goalhandwidth = 38.999997139
   planner.addGoalGrasp(goalhandwidth, goalpose)

   object_goal_place = None

   placeGripperPoses = []
   showRotate = []
   
   if foundObject:
      planner.addStartGrasp(starthandwidth, startpose)
      # planner.showgraph()
      placementsequence = planner.searchPath()

      planner.showHand(starthandwidth, startpose, base)
      print "placement sequence ", placementsequence

      if len(placementsequence) == 0:
         print "there is no way to place the object"
      else:
         plan1 = placementsequence[0]
         placements = planner.getPlacements(plan1)

         # tablepos = np.identity(4)
         quaternion_array = (tableresult.orientation.x,tableresult.orientation.y,tableresult.orientation.z,tableresult.orientation.w)
         tablepos = np.matrix(tf.transformations.quaternion_matrix(quaternion_array))
         # print "check"
         # print tablepos
         tablepos[0,3] = tableresult.centroid.x
         tablepos[1,3] = tableresult.centroid.y
         tablepos[2,3] = tableresult.centroid.z + PRE_PLACE_HEIGHT

         for ri, tableangle in enumerate([0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]):
            # for tableangle in [0.0]:
            rotationInZ = np.identity(4)
            rotationInZ[0][0] = math.cos(tableangle)
            rotationInZ[0][1] = -math.sin(tableangle)
            rotationInZ[1][0] = math.sin(tableangle)
            rotationInZ[1][1] = math.cos(tableangle)
            afterRotate = np.dot(tablepos, rotationInZ)

            ## calculate the object pose on table
            objectP = np.dot(afterRotate, placements[0])
            rotationQ = tf.transformations.quaternion_from_matrix(objectP)
            transformT = np.squeeze(np.asarray(objectP[:3,3]))

            object_goal_place = showPlacePos(rotationQ, transformT)
            showRotate.append(object_goal_place)

            ## calculate the gripper pose on table
            gripperP = np.dot(objectP, hand_grasp_pose)
            rotationQ = tf.transformations.quaternion_from_matrix(gripperP)
            transformT = np.squeeze(np.asarray(gripperP[:3,3]))

            placeGripperPoses.append((transformT, rotationQ))

         # ## find the plan to place the object
         plan = robot.planto_poses(placeGripperPoses)
         # # visualize the plan
         robot.display_trajectory(plan)

         raw_input("ready to place!!")
         robot.execute_plan(plan)

         objectSearcherTrigger(True, 2, pose_in_hand)

         ## need to update the object pose in hand
         try:
            # attand the object as a part of the arm
            listener.waitForTransform('/base_link', '/object', rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('/base_link', '/object', rospy.Time())
            robot.addManipulatedObject("object", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], "objects/cuboid.stl")
         except:
            print "fail to detect the object"
            foundObject = False
         raw_input("start to place!!")
         ## need to switch to cartesian controller
         robot.switchController('my_cartesian_motion_controller', 'arm_controller')

         # get gripper pose when placing
         listener.waitForTransform('/original_table', '/gripper_link', rospy.Time(), rospy.Duration(4.0))
         place_hand_in_table = listener.lookupTransform('/original_table', '/gripper_link', rospy.Time())
         place_hand_in_table[0][2] -= PRE_PLACE_HEIGHT

         current_stamp = trackstamp - 1

         while not rospy.is_shutdown():
            if current_stamp < trackstamp:
               current_stamp += 1

               # get current gripper pose in cartisian motion base
               listener.waitForTransform('/torso_lift_link', '/gripper_link', rospy.Time(), rospy.Duration(4.0))
               gripper_pose = listener.lookupTransform('/torso_lift_link', '/gripper_link', rospy.Time())

               # get the current table relate to the hand
               listener.waitForTransform('/predicted_hand', '/predicted_table', rospy.Time(), rospy.Duration(4.0))
               predicted_table_in_hand = listener.lookupTransform('/predicted_hand', '/predicted_table', rospy.Time())

               predict_place = transformProduct(predicted_table_in_hand, place_hand_in_table)

               targetposition, targetorientation = transformProduct(gripper_pose, predict_place)

               if(robot.moveToFrame(targetposition, targetorientation)):
                  break
               trans_error, rot_error = robot.getError()
               print "error trans ", trans_error, " rot ", rot_error
            else:
               rospy.sleep(1.0)

         robot.switchController('arm_controller', 'my_cartesian_motion_controller')

         raw_input("finish place")

         objectSearcherTrigger(False, 0, Pose())

         # open gripper and unattanch the object from the end-effector

   # show object state in hand
   planner.plotObject(base)
   
   def myFunction(task):
      # # broadcast the grasp pose
      # for i, v in enumerate(placeGripperPoses):
      #    transformT, rotationQ = v
      #    br.sendTransform(transformT, rotationQ, rospy.Time.now(), str(i), "base_link")
      # br.sendTransform((tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z), (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w), rospy.Time.now(), "place", "base_link")

      if len(showRotate) > 0:
         if showRotate[0] != None:
            goal_pub1.publish(showRotate[0])

         if showRotate[1] != None:
            goal_pub2.publish(showRotate[1])

         if showRotate[2] != None:
            goal_pub3.publish(showRotate[2])

         if showRotate[3] != None:
            goal_pub4.publish(showRotate[3])

         if showRotate[4] != None:
            goal_pub5.publish(showRotate[4])

         if showRotate[5] != None:
            goal_pub6.publish(showRotate[5])

         if showRotate[6] != None:
            goal_pub7.publish(showRotate[6])

         if showRotate[7] != None:
            goal_pub8.publish(showRotate[7])

      # (trans,rot) = listener.lookupTransform('/object', '/gripper_link', rospy.Time())
      # hand_grasp_pose = tf.TransformerROS().fromTranslationRotation(trans, rot)

      # ## get current hand width
      # (trans,rot) = listener.lookupTransform('/gripper_link', '/r_gripper_finger_link', rospy.Time())

      # startpose = Mat4( hand_grasp_pose[0][0],hand_grasp_pose[1][0],hand_grasp_pose[2][0],0.0, \
      #                   hand_grasp_pose[0][1],hand_grasp_pose[1][1],hand_grasp_pose[2][1],0.0, \
      #                   hand_grasp_pose[0][2],hand_grasp_pose[1][2],hand_grasp_pose[2][2],0.0, \
      #                   hand_grasp_pose[0][3] * 1000,hand_grasp_pose[1][3] * 1000,hand_grasp_pose[2][3] * 1000,1.0)
      # starthandwidth = trans[1] * 1000

      # ## convert it to be used in panda3d
      # startpose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * startpose

      # planner.showHand(starthandwidth, startpose, base)
      return task.again

   myTask = taskMgr.doMethodLater(0.1, myFunction, 'tickTask')
  
   base.run()