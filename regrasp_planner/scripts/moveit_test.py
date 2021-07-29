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
# from icra20_manipulation_pose.srv import SearchObject

from visualization_msgs.msg import Marker
from fetch_robot import Fetch_Robot

import tf
from scipy.spatial.transform import Rotation as R

from  tf_util import TF_Helper,Panda_Helper, transformProduct, getMatrixFromQuaternionAndTrans, findCloseTransform, getTransformFromPoseMat

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



def find_grasping_point(planner,tran_base_object ):
   print(len(planner.freegriprotmats))
   for i in range(0,len(planner.freegriprotmats)):
      obj_grasp_pos = planner.freegriprotmats[i] #Gives transforom matrix of grasp based on objrct ref  
      #planner.showHand( planner.freegripjawwidth[i], obj_grasp_pos, base)

      #change obj_pos from panda config to normal. Change pos rep to Quatorian. 
      obj_grasp_pos =  pd_helper.PandaPosMax_t_PosMat(obj_grasp_pos) 
      obj_grasp_pos = pd_helper.RotateGripper(obj_grasp_pos)
      obj_grasp_pos_Q = getTransformFromPoseMat(obj_grasp_pos) #Tranfrom gripper posmatx to (trans,rot)
    

      world_grasp_pos_Q = transformProduct(tran_base_object, obj_grasp_pos_Q)
      t,r = world_grasp_pos_Q
      world_grasp_pos = tf.TransformerROS().fromTranslationRotation(t,r)

      base_link_in_torso_link = tf_helper.getPoseMat('/torso_lift_link', '/base_link')
      torso_grasp_pos = base_link_in_torso_link.dot(world_grasp_pos)
      t_g_p_q = getTransformFromPoseMat(torso_grasp_pos)
      print("About to do ik solver")
      grasp_ik_result = robot.solve_ik_collision_free(t_g_p_q, 10)
      print("Done with Ik solver")
      if grasp_ik_result == None:
         print(i)
         continue
      else:
         world_grasp_pos_Q = transformProduct(tran_base_object, obj_grasp_pos_Q)
         return world_grasp_pos_Q


if __name__=='__main__':
   rospy.init_node('main_node')

   tf_helper = TF_Helper()
   pd_helper = Panda_Helper()

   ## move the end-effector into robot's camera view
   robot = Fetch_Robot()

   #robot.goto_pose(0.34969, 0.20337, 0.92054, 0.081339, 0.012991, -0.63111, 0.77131)
   robot.openGripper()
   # robot.closeGripper([-0.01])
  

   # Creates a base simulator in the world. 
   base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
   #Get the path of the object file 
   this_dir, this_filename = os.path.split(__file__)   
   objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cuboid.stl") 
   print(objpath)
   handpkg = fetch_grippernm  #SQL grasping database interface 
   gdb = db.GraspDB()   #SQL grasping database interface
   planner = RegripPlanner(objpath, handpkg, gdb)


   #Add object as collision for moveit. 
   tran_base_object = tf_helper.getTransform('/base_link', '/poing') #return tuple (trans,rot) of parent_lin to child_link
   robot.addCollisionObject("poing_collsion", tran_base_object, objpath, .001) 
   #tran_base_Table = tf_helper.getTransform('/base_link', '/Table') #return tuple (trans,rot) of parent_lin to child_link
   #robot.addCollisionObject("table_collsion", tran_base_Table,'objects/Table.stl') 

   #We are just graping a random graps pos 
   #TODO filter out based on place ment so we know which is the actuall grasp
   world_grasp_pos = find_grasping_point(planner, tran_base_object)

   plan = robot.planto_pose(world_grasp_pos)
   robot.execute_plan(plan)
   #tf.TransformerROS().fromTranslationRotation(trans, rot) 

   planner.plotObject(base)
   base.run()