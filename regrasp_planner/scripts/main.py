#!/usr/bin/python
# here is the main file

import os
import itertools

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

from regrasp_planner import RegripPlanner

import rospy

import actionlib
import std_srvs
from std_srvs.srv import Empty
from rail_segmentation.srv import SearchTable

from visualization_msgs.msg import Marker
from fetch_robot import Fetch_Robot

import tf


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

if __name__=='__main__':
   rospy.init_node('main_node')

   listener = tf.TransformListener()
   
   marker_pub = rospy.Publisher("/manipulate_pos", Marker, queue_size=1)

   base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
   gdb = db.GraspDB()
   this_dir, this_filename = os.path.split(__file__)
   objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cuboid.stl")

   # # pre-compute the re-grasp graph
   handpkg = fetch_grippernm
   planner = RegripPlanner(objpath, handpkg, gdb)
   planner.generateGraph()
   # # planner.showgraph()

   heightresult = 0.0
   # manipulation area selection, given the pointcloud to generate the position to manipulate
   rospy.wait_for_service('table_searcher/search_table')
   tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)
   try:
      heightresult = tableSearcher()
      print "table height = ", heightresult
   except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

   manipulationposx = 0.61644
   manipulationposy = 0.049959

   ## move the end-effector into robot's camera view
   robot = Fetch_Robot()
   robot.goto_pose(0.3638, 0.2, 0.818, -0.081003, -0.250516, -0.522513, 0.810962)

   ## show the manipulation position in rviz
   # marker = showManipulationPos(manipulationposx, manipulationposy, heightresult.tableHeight)
   robot.addCollisionObject("table", manipulationposx, manipulationposy, heightresult.tableHeight)
   (trans,rot) = listener.lookupTransform('/world', '/object', rospy.Time(0))
   robot.addManipulatedObject("object", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], "objects/cuboid.stl")

   ## get the current grasp
   (trans,rot) = listener.lookupTransform('/object', '/gripper_link', rospy.Time(0))
   hand_grasp_pose = tf.TransformerROS().fromTranslationRotation(trans, rot)

   ## get current hand width
   (trans,rot) = listener.lookupTransform('/gripper_link', '/r_gripper_finger_link', rospy.Time(0))

   startpose = Mat4( hand_grasp_pose[0][0],hand_grasp_pose[1][0],hand_grasp_pose[2][0],hand_grasp_pose[3][0], \
                     hand_grasp_pose[0][1],hand_grasp_pose[1][1],hand_grasp_pose[2][1],hand_grasp_pose[3][1], \
                     hand_grasp_pose[0][2],hand_grasp_pose[1][2],hand_grasp_pose[2][2],hand_grasp_pose[3][2], \
                     hand_grasp_pose[0][3],hand_grasp_pose[1][3],hand_grasp_pose[2][3],hand_grasp_pose[3][3])
   starthandwidth = trans[1] * 1000

   planner.addStartGrasp(starthandwidth, startpose)
   planner.plotObject(base)
   planner.showHand(starthandwidth, startpose, base)

   goalpose = Mat4(1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,53.3199386597,-8.46575927734,-4.76837158203e-07,1.0)
   goalhandwidth = 38.999997139
   planner.addGoalGrasp(goalhandwidth, goalpose)
   planner.showgraph()
   placementsequence = planner.searchPath()
   print "placement sequence ", placementsequence

   ## 
   

   # rate = rospy.Rate(10.0)
   # while not rospy.is_shutdown():
   #    try:
   #       (trans,rot) = listener.lookupTransform('/world', '/object', rospy.Time(0))
   #       robot.addManipulatedObject("object", trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], "objects/cuboid.stl")
   #    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   #       continue


   # while not rospy.is_shutdown():
   #     marker_pub.publish(marker)

   # raw_input("press enter!")
  
   base.run()