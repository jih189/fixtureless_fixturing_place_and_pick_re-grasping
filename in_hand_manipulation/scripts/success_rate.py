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
  num_of_test = 100
  while test_step < num_of_test:
    grasp_id_1, grasp_pose_1 = planner.getRandomGraspId()
    grasp_id_2, grasp_pose_2 = planner.getRandomGraspId()
    # print("check grasp ids ", grasp_id_1, " and ", grasp_id_2)
    # ensure the grasp poses has the same grasping axis direction
    if np.linalg.norm(grasp_pose_1[:3, 1] + grasp_pose_2[:3, 1]) < 1.0:
      continue
    test_step += 1
    # print("grasp id ", grasp_id_1, " ", grasp_id_2)
    result1 = planner.getGraspsById([grasp_id_1])
    result2 = planner.getGraspsById([grasp_id_2])

    planner.addStartGrasp(result1[0][0], result1[0][1], base)
    planner.addGoalGrasp(result2[0][0], result2[0][1], base)
    # if planner.has_path():
    #   success_num += 1
    # planner.showgraph()
    # break
    if planner.has_path():
      paths = planner.find_shortest_PlacementG_path()
      found_solution = False
      for l in range(len(paths)):
        path = paths[l]

        local_solution = True
        current_grasp_pose = result1[0][0]
        grasps_between_placements = planner.get_placement_grasp_trajectory(path)
        #########################################################################
        for i, (currentplacementid, _, current_real_placementid) in enumerate(path):
          # if the current planning placement is end goal, then we can break the loop
          if currentplacementid == "end_g":
            break

          find_next_grasp = False
          for nextgrasp_candidate_pair in grasps_between_placements[i]:
            candidate_jawwidth = 0
            if type(nextgrasp_candidate_pair) == tuple:
              candidate_jawwidth = nextgrasp_candidate_pair[1]
              nextgrasp_candidate = nextgrasp_candidate_pair[0] # convert the target grasp to correct format
            else:
              candidate_jawwidth = planner.getGraspsById([nextgrasp_candidate_pair])[0][1]
              nextgrasp_candidate = planner.getGraspsById([nextgrasp_candidate_pair])[0][0] # convert grasp id to grasp pose

            # convert the unit
            def convertUnit(inputmatrix):
              outputmatrix = np.array(inputmatrix)
              outputmatrix[0][3] *= 1000
              outputmatrix[1][3] *= 1000
              outputmatrix[2][3] *= 1000

              return outputmatrix

            dmgresult = dmgplanner.getTrajectory(convertUnit(current_grasp_pose), convertUnit(nextgrasp_candidate), candidate_jawwidth, convertUnit(planner.getPlacementsById([current_real_placementid])[0]), base)
            # if dmgresult == None:
            #   print("no dmg result")
            #   errorplacement = planner.getPlacementsById([current_real_placementid])[0]
            #   dmgplanner.renderObject(base, PosMat_t_PandaPosMax(errorplacement))
            #   starthnd = fetch_grippernm.newHandNM(hndcolor=[0, 0, 1, 0.5])
            #   starthnd.setMat(pandanpmat4 = PosMat_t_PandaPosMax(errorplacement.dot(current_grasp_pose)))
            #   starthnd.setJawwidth(50)
            #   starthnd.reparentTo(base.render)

            #   endhnd = fetch_grippernm.newHandNM(hndcolor=[1, 0, 0, 0.5])
            #   endhnd.setMat(pandanpmat4 = PosMat_t_PandaPosMax(errorplacement.dot(nextgrasp_candidate)))
            #   endhnd.setJawwidth(50)
            #   endhnd.reparentTo(base.render)

            #   # show the ground
            #   pg.plotLinesegs(base.render, np.array([[100,0,0],[-100, 0, 0]]))
            #   pg.plotLinesegs(base.render, np.array([[0,100,0],[0, -100, 0]]))

            #   base.run()

            if dmgresult == None:
              continue
            else:
              current_grasp_pose = nextgrasp_candidate
              find_next_grasp = True
              break
          if not find_next_grasp:
            local_solution = False
            break
        if local_solution:
          found_solution = True
          break
      if found_solution:
        success_num += 1
    planner.reset()
    print(success_num, " / ", test_step)

  print("success rate = ", float(success_num) / float(num_of_test))
  



if __name__=='__main__':

  # object_name = "cup"
  # object_name = "book"
  # object_name = "Lshape"
  # object_name = "Ushape"
  # object_name = "triangle"
  # object_name = "Oshape"
  object_name = "Hshape"
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