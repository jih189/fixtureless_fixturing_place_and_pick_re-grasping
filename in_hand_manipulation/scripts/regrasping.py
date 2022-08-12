#!/usr/bin/env python
import os
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.fetch_gripper import fetch_grippernm
from regrasp_planner import RegripPlanner
from manipulation.grip.ffregrasp import ff_regrasp_planner
from database import dbaccess as db
from utils import dbcvt as dc
import numpy as np
from scipy.spatial.transform import Rotation as R
from tf_util import PandaPosMax_t_PosMat, PosMat_t_PandaPosMax
from pandaplotutils import pandageom as pg 

import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout
import matplotlib.pyplot as plt


def regrasping(planner, dmgplanner, base):
  # convert the unit
  def convertUnit(inputmatrix):
    outputmatrix = np.array(inputmatrix)
    outputmatrix[0][3] *= 1000
    outputmatrix[1][3] *= 1000
    outputmatrix[2][3] *= 1000

    return outputmatrix

  # load the dmg planner from database
  dmgplanner.loadDB()

  # # create the regrasp graph
  planner.CreatePlacementGraph()

  while True:
    grasp_id_1, grasp_pose_1 = planner.getRandomGraspId()
    grasp_id_2, grasp_pose_2 = planner.getRandomGraspId()
    # ensure the grasp poses has the same grasping axis direction
    if np.linalg.norm(grasp_pose_1[:3, 1] + grasp_pose_2[:3, 1]) > 0.99:
      break

  # print("grasp id ", grasp_id_1, " ", grasp_id_2)
  # grasp_id_1 = 515
  # grasp_id_2 = 161
  result1 = planner.getGraspsById([grasp_id_1])
  result2 = planner.getGraspsById([grasp_id_2])

  planner.addStartGrasp(result1[0][0], result1[0][1], base)
  planner.addGoalGrasp(result2[0][0], result2[0][1], base)

  # planner.showgraph()

  checkCase = True
  
  success = False
  while planner.has_path():
    # print("has path")
    first_level_path, first_level_grasps, first_level_jawwidths = planner.find_first_level_path()
    # print("first level path ", first_level_path)
    # generate the second level graph
    findapath = True
    for s in range(len(first_level_path)-1):
      placeAndPickActionList = planner.findRegraspAction(first_level_path[s], first_level_path[s+1])
      placegrasppose = first_level_grasps[s]
      placejawwidth = first_level_jawwidths[s]
      pickgrasppose = first_level_grasps[s+1]
      pickjawwidth = first_level_jawwidths[s+1]

      # find is there any way to place and pick
      findsolution = False
      for placementpose, regrasptype, _ in placeAndPickActionList:
        if regrasptype == "stable":
          #### search ik solution for place and pick in stable placement
          placepose = placementpose.dot(placegrasppose)
          findPlaceSolution = False # True # solveIk(placepose)
          if not findPlaceSolution:
            continue
          pickpose = placementpose.dot(pickgrasppose)
          findPickSolution = False # True # solveIk(pickpose)
          if not findPickSolution:
            continue
          findsolution = True
          break
        elif regrasptype == "unstable":
          # search for the end-effector path for sliding motion over the object's surface
          # remember comment this back
          # dmgresult = dmgplanner.getTrajectory(convertUnit(placegrasppose), convertUnit(pickgrasppose), placejawwidth * 1000.0, convertUnit(placementpose), base)
          # if dmgresult == None:
          #   continue
          findSlidingArmMotionSolution = True # True # solveSlidingMotion(dmgresult)
          if not findSlidingArmMotionSolution:
            continue
          findsolution = True
      if checkCase:
        findsolution = False
        checkCase = False
      if not findsolution:
        # there is no solution between current place and pick grasp poses
        # remove the edge bewteen them in the first level graph
        planner.removeEdge(first_level_path[s], first_level_path[s+1])
        findapath = False
    if findapath:
      success = True
      break
  if success:
    print("find regrasp solution")
  else:
    print("can't find regrasp solution")

  planner.reset()


if __name__=='__main__':

  # object_name = "cup"
  # object_name = "book"
  object_name = "Lshape"
  # object_name = "Ushape"
  # object_name = "triangle"
  # object_name = "Oshape"
  # object_name = "Hshape"
  # object_name = "box"
  # object_name = "cuboid"
  # object_name = "almonds_can"
  
  base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
  this_dir, this_filename = os.path.split(__file__)   
  objpath = os.path.join(os.path.split(this_dir)[0], "objects", object_name + ".stl") 
  handpkg = fetch_grippernm  #SQL grasping database interface 
  gdb = db.GraspDB()   #SQL grasping database interface
  dmg_planner = ff_regrasp_planner(objpath, handpkg, gdb)   
  planner = RegripPlanner(objpath, handpkg, gdb, dmg_planner) 

  regrasping(planner, dmg_planner, base)