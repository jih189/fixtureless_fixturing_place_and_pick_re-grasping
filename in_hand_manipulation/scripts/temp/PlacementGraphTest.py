import os
import itertools

import MySQLdb as mdb
import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.core import *

from manipulation.grip.fetch_gripper import fetch_grippernm

import pandaplotutils.pandageom as pandageom
import trimesh
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from database import dbaccess as db
import tf
import pandaplotutils.pandactrl as pandactrl

import networkx as nx
from  tf_util import PandaPosMax_t_PosMat, PosMat_t_PandaPosMax


class Placement_graph_test():
    def __init__(self, objpath, handpkg, gdb, offset=0.0):
        self.handpkg = handpkg
        self.handname = handpkg.getHandName()

        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.8])
        
        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = offset)
        self.bulletworld.attachRigidBody(self.planebullnode)

        self.bulletworldhplowest = BulletWorld()
        self.planebullnode1 = cd.genCollisionPlane(offset=35)
        self.bulletworldhplowest.attachRigidBody(self.planebullnode1)


        self.gdb = gdb
        
        self.__loadFreeAirGrip()
        self.__loadFreeTablePlacement()

        self.PlacementG = nx.Graph()
        self.startnodeids = None
        self.goalnodeids = None
        self.shortestpaths = None
        self.inital_grasp = ("int_g", -1)
        self.PlacementG.add_node("int_g", stable=-1)

        self.end_grasp = ("end_g", -1)
        self.PlacementG.add_node("end_g", stable=-2)

    def __loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, self.handname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def __loadFreeTablePlacement(self):
        freetabletopplacementdata = self.gdb.loadFreeTabletopPlacementIncludeFF(self.dbobjname)
        if freetabletopplacementdata is None:
            raise ValueError("Plan the freeairgrip first!")
        self.tpsmat4s, self.placementid, self.placementtype = freetabletopplacementdata
        
    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])
    def CreatePlacementGraph(self):
            for p in range(len(self.placementid)):
                self.PlacementG.add_node(self.placementid[p], stable=self.placementtype[p])
            
            for i in range(len(self.freegripid)):
                sql = "SELECT freetabletopgrip.idfreetabletopplacement FROM freetabletopgrip WHERE \
                    freetabletopgrip.idfreeairgrip=%d" % self.freegripid[i]
                result = self.gdb.execute(sql)
                if len(result) > 1:
                    for edge in list(itertools.combinations(np.array(result)[:,0], 2)):
                        if not self.PlacementG.has_edge(*edge):
                            self.PlacementG.add_edge(*edge, graspid=[self.freegripid[i]])
                        else:
                            temp = self.PlacementG.edges[edge[0],edge[1]]['graspid']
                            temp.append(self.freegripid[i])
                            self.PlacementG.add_edge(*edge, graspid=temp)

    # start grasp is in numpy format
    def addStartGrasp(self, startrotmat4, starthandwidth):

        startrotmat4 = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * PosMat_t_PandaPosMax(startrotmat4)
        starthandwidth *= 1000
        
        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):
            # if the hand does not hit the ground, then this placement can connect to the goal node
            tmphnd = self.hand
            tmphnd.setJawwidth(starthandwidth)
            tmphnd.setMat(pandanpmat4 = startrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            hndbull_without_fingers_node = cd.genCollisionMeshMultiNp(tmphnd.palmnp)
            result1 = self.bulletworldhplowest.contactTest(hndbull_without_fingers_node)
            if not result.getNumContacts() and not result1.getNumContacts():
                if self.placementtype[p] == 0: # when placement is stable
                    self.PlacementG.add_edge('int_g', self.placementid[p])
                else: # when placement is unstable
                    p1 = self.getPointFromPose(startrotmat4 * self.tpsmat4s[p], [-20, 50, 0])
                    p2 = self.getPointFromPose(startrotmat4 * self.tpsmat4s[p], [-20, -50, 0])
                    if abs(p1[2] - p2[2]) < 10:
                        self.PlacementG.add_edge('int_g', self.placementid[p])
    # goal grasp is in Panda format, and goal handwidth is not in meter unit
    def addGoalGrasp(self, goalrotmat4, goalhandwidth):

        goalrotmat4 = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * PosMat_t_PandaPosMax(goalrotmat4)
        goalhandwidth *= 1000

        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):

            if self.placementtype[p] != 0: # if placement is not stable, then check whether the gripper is perpendicular to the table
                p1 = self.getPointFromPose(goalrotmat4 * self.tpsmat4s[p], [-20, 50, 0])
                p2 = self.getPointFromPose(goalrotmat4 * self.tpsmat4s[p], [-20, -50, 0])
                if abs(p1[2] - p2[2]) >= 10:
                    continue

            # if the hand does not hit the ground, then this placement can connect to the goal node
            tmphnd = self.hand
            tmphnd.setJawwidth(goalhandwidth)
            tmphnd.setMat(pandanpmat4 = goalrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            hndbull_without_fingers_node = cd.genCollisionMeshMultiNp(tmphnd.palmnp)
            result1 = self.bulletworldhplowest.contactTest(hndbull_without_fingers_node)
            if not result.getNumContacts() and not result1.getNumContacts():
                addededge = ('end_g', self.placementid[p])
                if not self.PlacementG.has_edge(*addededge):
                    self.PlacementG.add_edge(*addededge, graspid = [(goalrotmat4, goalhandwidth)])
                else:
                    temp = self.PlacementG.edges['end_g', self.placementid[p]]['graspid']
                    temp.append((goalrotmat4, goalhandwidth))
                    self.PlacementG.add_edge(*addededge, graspid = temp)
    #get init grasps will return a set of init grasp of the object
    # return: issuccess, list of target grasps
    def getInitGrasps(self, gdb, object_name):
        sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % object_name
        result = gdb.execute(sql)
        if not result:
            print "please add the object name to the table first!!"
            return False, None
        else:
            objectId = int(result[0][0])

        sql = "SELECT grasppose, jawwidth FROM initgrasps WHERE idobject = '%d'" % objectId
        initgrasp_result = gdb.execute(sql)

        # target grasps (grasp pose(in numpy format), jawwidth(in meter))
        init_grasps = []
        for grasppose, jawwidth in initgrasp_result:
            init_grasps.append((PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(grasppose)), float(jawwidth) / 1000))

        return True, init_grasps

    # get target grasps will return a set of target grasp of the object
    # return: issuccess, list of target grasps
    def getTargetGrasps(self, gdb, object_name):
        sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % object_name
        result = gdb.execute(sql)
        if not result:
            print "please add the object name to the table first!!"
            return False, None
        else:
            objectId = int(result[0][0])

        sql = "SELECT grasppose, jawwidth FROM targetgrasps WHERE idobject = '%d'" % objectId
        targetgrasp_result = gdb.execute(sql)

        # target grasps (grasp pose(in numpy format), jawwidth(in meter))
        target_grasps = []
        for grasppose, jawwidth in targetgrasp_result:
            target_grasps.append((PandaPosMax_t_PosMat(pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(grasppose)), float(jawwidth) / 1000))

        return True, target_grasps

if __name__=='__main__':
    object_name = "book"

    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)   
    objpath = os.path.join(os.path.split(this_dir)[0], "objects", object_name + ".stl") 
    handpkg = fetch_grippernm  #SQL grasping database interface 
    gdb = db.GraspDB()   #SQL grasping database interface


    planner = Placement_graph_test(objpath, handpkg, gdb)
    result, init_grasp = planner.getInitGrasps(gdb,object_name)
    result, target_grasps = planner.getTargetGrasps(gdb,object_name)
    planner.CreatePlacementGraph()
    #planner.addStartGrasp()
    # add goal grasps into the graph
    for grasp, jawwidth in target_grasps:
        planner.addGoalGrasp(grasp, jawwidth)
    g = planner.PlacementG
    labels = {n: g.nodes[n]['stable'] for n in g.nodes}
    colors = [g.nodes[n]['stable'] for n in g.nodes]
    nx.draw(g, with_labels=True, labels=labels, node_color=colors)
    plt.draw()
    plt.show()