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
import matplotlib.pyplot as plt


def regrasping(planner, dmgplanner, base):

  # load the dmg planner from database
  dmgplanner.loadDB()

  # create the regrasp graph
  planner.CreatePlacementGraph()
  # planner.showgraph()

  test_step = 0
  success_num = 0
  num_of_test = 1
  while test_step < num_of_test:
    grasp_id_1, grasp_pose_1 = planner.getRandomGraspId()
    grasp_id_2, grasp_pose_2 = planner.getRandomGraspId()
    # ensure the grasp poses has the same grasping axis direction
    if np.linalg.norm(grasp_pose_1[:3, 1] + grasp_pose_2[:3, 1]) < 1.0:
      continue
    test_step += 1
    # print("grasp id ", grasp_id_1, " ", grasp_id_2)
    result1 = planner.getGraspsById([grasp_id_1])
    result2 = planner.getGraspsById([grasp_id_2])

    planner.addStartGrasp(grasp_id_1, result1[0][0], result1[0][1])
    planner.addGoalGrasp(grasp_id_2, result2[0][0], result2[0][1])

    if planner.has_path():
      paths = planner.find_shortest_PlacementG_path()
      found_solution = False
      for l in range(len(paths)):
        path = paths[l]

        local_solution = True
        current_grasp_pose = result1[0][0]
        grasps_between_placements = planner.get_placement_grasp_trajectory(path)
        #########################################################################
        print("path")
        print(path[:-1])
        print("common grasps")
        print(grasps_between_placements[:-1])
      if found_solution:
        success_num += 1

    planner.reset()
    print(success_num, " / ", test_step)

  print("success rate = ", float(success_num) / float(num_of_test))
  



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
  planner = RegripPlanner(objpath, handpkg, gdb)
  dmg_planner = ff_regrasp_planner(objpath, handpkg, gdb)   

  regrasping(planner, dmg_planner, base)