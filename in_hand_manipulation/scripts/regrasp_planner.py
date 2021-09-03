#!/usr/bin/python

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

import networkx as nx
from  tf_util import PandaPosMax_t_PosMat, PosMat_t_PandaPosMax

# this is planner to plan a sequence of placement and regrasping on table for in-hand regrasping.
class RegripPlanner():
    def __init__(self, objpath, handpkg, gdb, offset=0.0):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.facets, self.facetnormals = self.objtrimesh.facets_over(faceangle=.95, segangle = .95)
        self.objgeom = pandageom.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.8])

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        # self.regg = nx.Graph()
        # placement graph 
        self.PlacementG = nx.Graph() 

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = offset)
        self.bulletworld.attachRigidBody(self.planebullnode)

        self.bulletworldhplowest = BulletWorld()
        self.planebullnode1 = cd.genCollisionPlane(offset=35)
        self.bulletworldhplowest.attachRigidBody(self.planebullnode1)

        self.startnodeids = None
        self.goalnodeids = None
        self.shortestpaths = None

        self.gdb = gdb

        self.__loadFreeAirGrip()
        self.__loadFreeTablePlacement()
        self.handtmp = None

        self.objectid = self.gdb.loadIdObject(self.dbobjname)

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
    
    def insert_init_graspID_as_placementGraphNode(self, graspid):

        placement_list = self.getPlacementIdsByGraspId(graspid)
        for p in placement_list:
            edge = ("int_g",p)
            self.PlacementG.add_edge(*edge)
        
    def insert_end_graspID_as_placementGraphNode(self,graspid):

        placement_list = self.getPlacementIdsByGraspId(graspid)
        for p in placement_list:
            edge = ("end_g",p)
            self.PlacementG.add_edge(*edge)

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
                    if abs(p1[2] - p2[2]) < 15:
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

    def find_shortest_PlacementG_path(self):
        # path = nx.shortest_path(self.PlacementG, self.inital_grasp[0], self.end_grasp[0])
        # this function will return all possible shortest path
        paths = nx.all_shortest_paths(self.PlacementG, self.inital_grasp[0], self.end_grasp[0])
        results = []
        for path in paths:
            #remove end graps node and init grasp node in graph path.
            path.pop(0)
            path_and_type = []
            for p in path:
                type = self.PlacementG.nodes[p]["stable"]
                path_and_type.append((p,type))
            results.append(path_and_type)
        return results

    def get_placement_grasp_trajectory(self,path_and_type):
        path = path_and_type
        grasp_traj = []
        if len(path) > 1:
            for i in range(0,len(path)-1):
                currnt_p = path[i][0]
                next_p = path[i+1][0]
                graspid_list = self.PlacementG.get_edge_data(currnt_p,next_p)['graspid']
                grasp_traj.append(graspid_list)
        elif len(path) == 1:
            return []
        else:
            print("error, no Placement path be grasp points")
        return grasp_traj
    
    # def generateGraph(self):
    #     for p in range(len(self.placementid)):
    #         self.regg.add_node(self.placementid[p])

    #     # according the grip, generate the edge between placements
    #     for g in range(len(self.freegripid)):
    #         sql = "SELECT freetabletopgrip.idfreetabletopplacement FROM freetabletopgrip WHERE \
    #             freetabletopgrip.idfreeairgrip=%d" % self.freegripid[g]
    #         result = self.gdb.execute(sql)
    #         if len(result) != 0 and len(result) != 1:
    #             for edge in list(itertools.combinations(np.array(result)[:,0], 2)):
    #                 if not self.regg.has_edge(*edge):
    #                     self.regg.add_edge(*edge)   


    #             self.regg.add_edge('s', self.placementid[p])       

    # def searchPath(self):
    #     result = []
    #     self.shortestpaths = nx.all_shortest_paths(self.regg, source = 's', target = 'g')
    #     for path in self.shortestpaths:
    #         result.append(path[1:-1])
    #     return result

    # get all grasp poses in normal format and jawwdith in meter unit
    # this function will return a list like
    # [(pose1, jawwidth1), (pose2, jawwidth2), ...]
    def getAllGrasps(self):
        results = []
        for grasppose, jawwidth in zip(self.freegriprotmats, self.freegripjawwidth): 

            grasppose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * grasppose
            # need to convert it bakc to normal metrics
            results.append((np.array([[grasppose[0][0],grasppose[1][0],grasppose[2][0],grasppose[3][0]/1000.0], \
                                     [grasppose[0][1],grasppose[1][1],grasppose[2][1],grasppose[3][1]/1000.0], \
                                     [grasppose[0][2],grasppose[1][2],grasppose[2][2],grasppose[3][2]/1000.0], \
                                     [grasppose[0][3],grasppose[1][3],grasppose[2][3],grasppose[3][3]]]), jawwidth/1000.0))
        return results

    # get the grasp pose of grasp ids under specific placement
    # this function will return a list like
    # [(pose1, jawwidth1), (pose2, jawwidth2), ...]
    def getGraspsById(self, graspids):
        results = []
        for i in graspids:
            sql = "SELECT freeairgrip.rotmat, freeairgrip.jawwidth FROM freeairgrip WHERE \
                    freeairgrip.idfreeairgrip=%d AND freeairgrip.idobject=%d" % (i, self.objectid)
            result = self.gdb.execute(sql)
            if len(result) == 0:
                continue
            grasppose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(result[0][0])
            jawwidth = float(result[0][1]) / 1000
            # need to convert it back to normal metrics
            results.append((np.array([[grasppose[0][0],grasppose[1][0],grasppose[2][0],grasppose[3][0]/1000.0], \
                                     [grasppose[0][1],grasppose[1][1],grasppose[2][1],grasppose[3][1]/1000.0], \
                                     [grasppose[0][2],grasppose[1][2],grasppose[2][2],grasppose[3][2]/1000.0], \
                                     [grasppose[0][3],grasppose[1][3],grasppose[2][3],grasppose[3][3]]]), jawwidth))
        return results

    # given the placement id, this function will return all grasp id related to it
    def getGraspIdsByPlacementId(self, placementid):
        results = []
        sql =  "SELECT freetabletopgrip.idfreeairgrip FROM freetabletopgrip WHERE freetabletopgrip.idfreetabletopplacement=%d \
                    " % placementid
        result = self.gdb.execute(sql)

        for i in range(len(result)):
            results.append(int(result[i][0]))
        return results

    # given the grasp id, this function will return all placement id related to it
    def getPlacementIdsByGraspId(self, graspid):
        results = []
        sql =  "SELECT freetabletopgrip.idfreetabletopplacement FROM freetabletopgrip WHERE freetabletopgrip.idfreeairgrip=%d \
                    " % graspid
        result = self.gdb.execute(sql)

        for i in range(len(result)):
            results.append(int(result[i][0]))
        return results

    # this function will return all placement poses in normal format
    def getAllPlacements(self):
        results = []
        for placementpose in self.tpsmat4s:
            results.append(np.array([[placementpose[0][0],placementpose[1][0],placementpose[2][0],placementpose[3][0]/1000.0], \
                                     [placementpose[0][1],placementpose[1][1],placementpose[2][1],placementpose[3][1]/1000.0], \
                                     [placementpose[0][2],placementpose[1][2],placementpose[2][2],placementpose[3][2]/1000.0], \
                                     [placementpose[0][3],placementpose[1][3],placementpose[2][3],placementpose[3][3]]]))
        return results

    # get the ground direction vector from pose in object frame
    def getGroundDirection(self, pose):
        gnd_dir = np.array([0,0,-1]) #ground direction (i.e gravity vector )
        pos_r = pose[:3,:3]
        obj_grd_dir_pos = np.dot( np.transpose(pos_r), gnd_dir) # vecktro containing the dir of the object to the groud in the object frame
        obj_grd_dir_pos= obj_grd_dir_pos/ np.linalg.norm(obj_grd_dir_pos)
        return obj_grd_dir_pos

    # given a pose, this function will return the placement id whose has most simlilar pose.
    def getPlacementIdFromPose(self, pose):
        obj_dir_pos_2match = self.getGroundDirection(pose)
        diff = 2.0 
        closest_placementid = None
        for placementid, placementpose in zip(self.placementid, self.tpsmat4s):
            placement_pose = PandaPosMax_t_PosMat(placementpose)
            currnt_dir_pos = self.getGroundDirection(placement_pose)
            
            currnt_diff = np.linalg.norm(currnt_dir_pos - obj_dir_pos_2match)
            if currnt_diff <= diff:
                closest_placementid = placementid
                diff = currnt_diff
        return closest_placementid 

    # get placements in normal format with list of placement id
    def getPlacementsById(self, placementids):
        results = []
        for i in placementids:
            sql = "SELECT freetabletopplacement.rotmat FROM freetabletopplacement WHERE \
                    freetabletopplacement.idfreetabletopplacement=%d AND freetabletopplacement.idobject=%d" % (i, self.objectid)
            result = self.gdb.execute(sql)
            if len(result) == 0:
                continue
            placementpose = dc.strToMat4(result[0][0])
            # need to convert it back to normal metrics
            results.append(np.array([[placementpose[0][0],placementpose[1][0],placementpose[2][0],placementpose[3][0]/1000.0], \
                                     [placementpose[0][1],placementpose[1][1],placementpose[2][1],placementpose[3][1]/1000.0], \
                                     [placementpose[0][2],placementpose[1][2],placementpose[2][2],placementpose[3][2]/1000.0], \
                                     [placementpose[0][3],placementpose[1][3],placementpose[2][3],placementpose[3][3]]]))
        return results

    # given object pose, this function use find the most similar placement according to the rotation
    # part of the matrix, then return all grasps related to this pose.
    def getGraspsbyPlacementPose(self, placementTransform):
        t,r = placementTransform
        tran_base_object_pos = tf.TransformerROS().fromTranslationRotation(t,r)
        matched_obj_placement_id = self.getPlacementIdFromPose(tran_base_object_pos) #gets the most likly object placement given pose
        gripper_id_list = self.getGraspIdsByPlacementId(matched_obj_placement_id)
        return self.getGraspsById(gripper_id_list) # List of pos that that match placement

    def showPlacementSequence(self, sequence, base):
        distancebetweencell = 300
        numOfP = len(sequence)
        showtable = np.zeros(numOfP)
        for i in range(len(showtable)):
            showtable[i] = (i - int(numOfP/2)) * distancebetweencell

    def showgraph(self):
        nx.draw(self.PlacementG, with_labels=True)
        plt.draw()
        plt.show()

    def plotObject(self, base):
        geomnodeobj = GeomNode('obj')
        geomnodeobj.addGeom(self.objgeom)
        npnodeobj = NodePath('obj')
        npnodeobj.attachNewNode(geomnodeobj)
        npnodeobj.reparentTo(base.render)
        pandageom.plotAxisSelf(base.render, spos=Vec3(0,0,0))

    def showHand(self, hndjawwidth, hndrotmat, base):

        # if self.handtmp == None:
        self.handtmp = fetch_grippernm.Fetch_gripperNM(hndcolor=[0, 1, 0, .5])
        self.handtmp.setMat(pandanpmat4=hndrotmat)
        self.handtmp.setJawwidth(hndjawwidth)
        self.handtmp.reparentTo(base.render)
        # else:
        #     self.handtmp.setMat(pandanpmat4=hndrotmat)
        #     self.handtmp.setJawwidth(hndjawwidth)

    def findCommandGrasp(self, placement_1_id, placement_2_id):

        # need to check whether two placement id belonging to the target object

        sql = "SELECT freetabletopplacement.idobject FROM freetabletopplacement WHERE freetabletopplacement.idfreetabletopplacement=%d" % placement_1_id
        result = self.gdb.execute(sql)
        if len(result) == 0 or int(result[0][0]) != self.objectid:
            raise "placement 1 id does not belong to the target object"

        sql = "SELECT freetabletopplacement.idobject FROM freetabletopplacement WHERE freetabletopplacement.idfreetabletopplacement=%d" % placement_2_id
        result = self.gdb.execute(sql)
        if len(result) == 0 or int(result[0][0]) != self.objectid:
            raise "placement 2 id does not belong to the target object"

        commonids = []
        sql =  "SELECT freeairgrip.idfreeairgrip FROM freeairgrip WHERE \
                    freeairgrip.idfreeairgrip IN (SELECT freetabletopgrip.idfreeairgrip FROM freetabletopgrip WHERE freetabletopgrip.idfreetabletopplacement=%d) \
                    AND \
                    freeairgrip.idfreeairgrip IN (SELECT freetabletopgrip.idfreeairgrip FROM freetabletopgrip WHERE freetabletopgrip.idfreetabletopplacement=%d) \
                    " % (placement_1_id, placement_2_id)
        result = self.gdb.execute(sql)
        for r in result:
            commonids.append(int(r[0]))
        return commonids

if __name__=='__main__':
    pass

    # base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    # gdb = db.GraspDB()
    # this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0], "objects", "cuboid.stl")

    # handpkg = fetch_grippernm
    # planner = RegripPlanner(objpath, handpkg, gdb)
    # planner.generateGraph()

    # goalpose = Mat4(1.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,1.0,0.0,0.0,53.3199386597,-8.46575927734,-4.76837158203e-07,1.0)
    # goalhandwidth = 38.999997139
    # planner.addGoalGrasp(goalhandwidth, goalpose)

    # startpose = Mat4( -0.999999940395,-1.22464685259e-16,5.35310124002e-24,0.0,0.0,-4.37113882867e-08,-1.0,0.0,1.22464672024e-16,-1.0,4.37113882867e-08,0.0,-51.3811569214,-0.922590315342,-4.76837158203e-07,1.0)
    # starthandwidth = 38.999997139
    # planner.addStartGrasp(starthandwidth, startpose)

    # placementsequence = planner.searchPath()
    # print "placement sequence ", placementsequence

    # planner.showPlacementSequence(placementsequence[0], base)

    # # planner.findCommandGrasp(placementsequence[0][0], placementsequence[0][1])
    # commongraspids = planner.findCommandGrasp(4, 5)
    # currentgrasppose = planner.getGrasps(4, commongraspids)
    # print "current grasp poses"
    # print currentgrasppose

    # # show the regrasp graph
    # # nx.draw(planner.regg, with_labels=True)
    # # plt.draw()
    # # plt.show()


    # base.run()