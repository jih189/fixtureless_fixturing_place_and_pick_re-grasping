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

  print("grasp id ", grasp_id_1, " ", grasp_id_2)
  grasp_id_1 = 515
  grasp_id_2 = 161
  result1 = planner.getGraspsById([grasp_id_1])
  result2 = planner.getGraspsById([grasp_id_2])

  planner.addStartGrasp(result1[0][0], result1[0][1], base)
  planner.addGoalGrasp(result2[0][0], result2[0][1], base)

  # planner.showgraph()

  if planner.has_path():
    paths = planner.find_shortest_PlacementG_path()
    # found_solution = False
    for l in range(len(paths)):
      actionpathgraphNodes = []
      path = paths[l]
      print("intermediate placement ", path[:-1])
      grasps_between_placements_temp = planner.get_placement_grasp_trajectory(path)
      grasps_between_placements = []
      # print("grasps")
      for g in grasps_between_placements_temp[:-1]:
        grasps_between_placements.append([planner.getGraspsById([j])[0][0] for j in g])
      # grasps_between_placements.append(grasps_between_placements_temp[-1][0])
      # print("grasps ", grasps_between_placements)
      actionpathgraph = nx.DiGraph()
      actionpathgraph.add_node("start", graspOrPlace=0, pose = result1[0][0], labelname="init grasp")
      actionpathgraph.add_node("end", graspOrPlace=0, pose = result2[0][0], labelname="target grasp")

      # add the first intermediate placement
      firstintermediateplacement = path[0]
      sampledplacementpose = planner.getPlacementsById([firstintermediateplacement[2]])[0]
      currentStep = ["start"]
      currentSteptemp = []
      for subindex, tableangle in enumerate([0.0, 1.5707963267948966, 3.141592653589793]):
        rotationInZ = np.identity(4)
        rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()

        newnodeid = len(actionpathgraphNodes)
        actionpathgraph.add_node(newnodeid, graspOrPlace=1, pose=sampledplacementpose.dot(rotationInZ), labelname=str(0) + "_" +str(subindex))
        actionpathgraphNodes.append(newnodeid)
        currentSteptemp.append(newnodeid)
        for c in currentStep:
          actionpathgraph.add_edge(c, newnodeid)

      currentStep = currentSteptemp
      currentSteptemp = []

      for s in range(len(grasps_between_placements)):
        print("current step ", s)
        for subindex, g in enumerate(grasps_between_placements[s][:min(4, len(grasps_between_placements[s]))]):
          newnodeid = len(actionpathgraphNodes)
          actionpathgraph.add_node(newnodeid, graspOrPlace=0, pose=g, labelname=str(s) + "_" + str(subindex))
          actionpathgraphNodes.append(newnodeid)
          currentSteptemp.append(newnodeid)
          for c in currentStep:
            actionpathgraph.add_edge(c, newnodeid)

        currentStep = currentSteptemp
        currentSteptemp = []

        sampledplacementpose = planner.getPlacementsById([path[s+1][2]])[0]
        for subindex, tableangle in enumerate([0.0, 1.5707963267948966, 3.141592653589793]):
          rotationInZ = np.identity(4)
          rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
          # actionpathgraphNodesTemp.append(len(actionpathgraphNodes)-1)
          newnodeid = len(actionpathgraphNodes)
          actionpathgraph.add_node(newnodeid, graspOrPlace=1, pose=sampledplacementpose.dot(rotationInZ), labelname=str(s+1) + "_" + str(subindex))
          actionpathgraphNodes.append(newnodeid)
          currentSteptemp.append(newnodeid)
          for c in currentStep:
            actionpathgraph.add_edge(c, newnodeid)

        currentStep = currentSteptemp
        currentSteptemp = []

      # add target grasp into graph
      for c in currentStep:
        actionpathgraph.add_edge(c, "end")

      color_map = ['red' if actionpathgraph.node[n]['graspOrPlace'] == 0 else 'green' for n in actionpathgraph]
      pos = graphviz_layout(actionpathgraph, prog="dot")
      node_labels = nx.get_node_attributes(actionpathgraph, "labelname")
      # nx.draw(actionpathgraph, pos, labels=node_labels, node_color=color_map, node_size=600)
      nx.draw(actionpathgraph, pos, with_labels=True, node_color=color_map, node_size=600)
      # nx.draw_networkx_labels(actionpathgraph, pos, labels=node_labels)
        
      plt.draw()
      plt.show()

      # find all placement nodes
      for placementnode in [n for n in actionpathgraph if actionpathgraph.node[n]['graspOrPlace'] == 1]:

        placementpose = actionpathgraph.node[placementnode]['pose']
        
        # print("placement node", placementnode)
        # print("pred ", list(actionpathgraph.predecessors(placementnode)))
        # print("des ", list(actionpathgraph.successors(placementnode)))
        predecessorsId = list(actionpathgraph.predecessors(placementnode))
        successorsId = list(actionpathgraph.successors(placementnode))
        actionpathgraph.remove_node(placementnode)

        # placenodes
        placenodes = []
        for i in predecessorsId:
          grasppose = actionpathgraph.node[i]['pose']
          actionpathgraph.add_node(str(i) + "-" + str(placementnode), graspOrPlace=1, pose=placementpose.dot(grasppose), labelname="")
          placenodes.append(str(i) + "-" + str(placementnode))
          actionpathgraph.add_edge(i, str(i) + "-" + str(placementnode))

        # picknodes
        for i in successorsId:
          grasppose = actionpathgraph.node[i]['pose']
          actionpathgraph.add_node(str(i) + "-" + str(placementnode), graspOrPlace=1, pose=placementpose.dot(grasppose), labelname="")
          actionpathgraph.add_edge(str(i) + "-" + str(placementnode), i)

          for pn in placenodes:
            actionpathgraph.add_edge(pn, str(i) + "-" + str(placementnode))

      color_map = ['red' if actionpathgraph.node[n]['graspOrPlace'] == 0 else 'green' for n in actionpathgraph]
      pos = graphviz_layout(actionpathgraph, prog="dot")
      node_labels = nx.get_node_attributes(actionpathgraph, "labelname")
      # nx.draw(actionpathgraph, pos, labels=node_labels, node_color=color_map, node_size=600)
      nx.draw(actionpathgraph, pos, with_labels=True, node_color=color_map, node_size=600)
      # nx.draw_networkx_labels(actionpathgraph, pos, labels=node_labels)

      #update graph
      plt.draw()
      plt.show()

      break
      
     
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