#!/usr/bin/python

import os
import itertools

import MySQLdb as mdb
import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.core import *

from manipulation.grip.fetch_gripper import fetch_grippernm

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from database import dbaccess as db

import networkx as nx

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
        self.regg = nx.Graph()

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = offset)
        self.bulletworld.attachRigidBody(self.planebullnode)

        self.startnodeids = None
        self.goalnodeids = None
        self.shortestpaths = None

        self.gdb = gdb

        self.__loadFreeAirGrip()
        self.__loadFreeTablePlacement()
        self.handtmp = None

        self.objectid = self.gdb.loadIdObject(self.dbobjname)


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

    def generateGraph(self):
        for p in range(len(self.placementid)):
            self.regg.add_node(self.placementid[p])

        # according the grip, generate the edge between placements
        for g in range(len(self.freegripid)):
            sql = "SELECT freetabletopgrip.idfreetabletopplacement FROM freetabletopgrip WHERE \
                freetabletopgrip.idfreeairgrip=%d" % self.freegripid[g]
            result = self.gdb.execute(sql)
            if len(result) != 0 and len(result) != 1:
                for edge in list(itertools.combinations(np.array(result)[:,0], 2)):
                    if not self.regg.has_edge(*edge):
                        self.regg.add_edge(*edge)   

    def addGoalGrasp(self, goalhandwidth, goalrotmat4):

        self.regg.add_node('g')
        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):
            
            # if the hand does not hit the ground, then this placement can connect to the goal node
            tmphnd = self.hand
            tmphnd.setJawwidth(goalhandwidth)
            tmphnd.setMat(pandanpmat4 = goalrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                self.regg.add_edge('g', self.placementid[p])

    def addStartGrasp(self, starthandwidth, startrotmat4):

        self.regg.add_node('s')
        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):
            
            # if the hand does not hit the ground, then this placement can connect to the start node
            tmphnd = self.hand
            tmphnd.setJawwidth(starthandwidth)
            tmphnd.setMat(pandanpmat4 = startrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                self.regg.add_edge('s', self.placementid[p])       

    def searchPath(self):
        result = []
        self.shortestpaths = nx.all_shortest_paths(self.regg, source = 's', target = 'g')
        for path in self.shortestpaths:
            result.append(path[1:-1])
        return result

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
            jawwidth = float(result[0][1])
            # need to convert it back to normal metrics
            results.append((np.array([[grasppose[0][0],grasppose[1][0],grasppose[2][0],grasppose[3][0]/1000.0], \
                                     [grasppose[0][1],grasppose[1][1],grasppose[2][1],grasppose[3][1]/1000.0], \
                                     [grasppose[0][2],grasppose[1][2],grasppose[2][2],grasppose[3][2]/1000.0], \
                                     [grasppose[0][3],grasppose[1][3],grasppose[2][3],grasppose[3][3]]]), jawwidth))
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

    def showPlacementSequence(self, sequence, base):
        distancebetweencell = 300
        numOfP = len(sequence)
        showtable = np.zeros(numOfP)
        for i in range(len(showtable)):
            showtable[i] = (i - int(numOfP/2)) * distancebetweencell

    def showgraph(self):
        nx.draw(self.regg, with_labels=True)
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
        
        
        # objrotmat = self.tpsmat4s[placement_1_id-1]
        # objrotmat.setCell(3,0,-300)

        # # show object
        # geom = pg.packpandageom(self.objtrimesh.vertices,
        #                         self.objtrimesh.face_normals,
        #                         self.objtrimesh.faces)
        # node = GeomNode('obj')
        # node.addGeom(geom)
        # star = NodePath('obj')
        # star.attachNewNode(node)

        # star.setTransparency(TransparencyAttrib.MAlpha)
        # star.setMat(objrotmat)
        # star.reparentTo(base.render)

        # sql = "SELECT freetabletopgrip.rotmat, freetabletopgrip.jawwidth, freetabletopgrip.idfreeairgrip FROM freetabletopgrip WHERE freetabletopgrip.idfreetabletopplacement=%d" % placement_1_id
        # result = self.gdb.execute(sql)

        # handRot = []
        # handWidth = []
        # gripid = []
        # for r in result:
        #     handRot.append(dc.strToMat4(r[0]))
        #     handWidth.append(float(r[1]))
        #     gripid.append(r[2])

        # print "grip ids"
        # print gripid

        # # # show the gripers
        # for j in range(len(handRot)):
        #     hndrotmat = handRot[j]
        #     hndrotmat.setCell(3,0,-300)
        #     hndjawwidth = handWidth[j]
        #     # show grasps
        #     tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
        #     tmphnd.setMat(pandanpmat4 = hndrotmat)
        #     tmphnd.setJawwidth(hndjawwidth)
        #     # tmphnd.setJawwidth(0)
        #     # tmprtq85.setJawwidth(80)
        #     tmphnd.reparentTo(base.render)

        # objrotmat = self.tpsmat4s[placement_2_id-1]
        # objrotmat.setCell(3,0,300)

        # # show object
        # geom = pg.packpandageom(self.objtrimesh.vertices,
        #                         self.objtrimesh.face_normals,
        #                         self.objtrimesh.faces)
        # node = GeomNode('obj')
        # node.addGeom(geom)
        # star = NodePath('obj')
        # star.attachNewNode(node)

        # star.setTransparency(TransparencyAttrib.MAlpha)
        # star.setMat(objrotmat)
        # star.reparentTo(base.render)
        
        # sql = "SELECT freetabletopgrip.rotmat, freetabletopgrip.jawwidth, freetabletopgrip.idfreeairgrip FROM freetabletopgrip WHERE freetabletopgrip.idfreetabletopplacement=%d" % placement_2_id
        # result = self.gdb.execute(sql)

        # handRot = []
        # handWidth = []
        # gripid = []

        # for r in result:
        #     handRot.append(dc.strToMat4(r[0]))
        #     handWidth.append(float(r[1]))
        #     gripid.append(r[2])

        # print "grip ids"
        # print gripid

        # # # show the gripers
        # for j in range(len(handRot)):
        #     hndrotmat = handRot[j]
        #     hndrotmat.setCell(3,0,300)
        #     hndjawwidth = handWidth[j]
        #     # show grasps
        #     tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
        #     tmphnd.setMat(pandanpmat4 = hndrotmat)
        #     tmphnd.setJawwidth(hndjawwidth)
        #     # tmphnd.setJawwidth(0)
        #     # tmprtq85.setJawwidth(80)
        #     tmphnd.reparentTo(base.render)
        


        
                

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