#!/usr/bin/python

from __future__ import with_statement
import os
import itertools

import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.core import *

from manipulation.grip.fetch_gripper import fetch_grippernm
from scipy.spatial.transform import Rotation as R

import pandaplotutils.pandageom as pandageom
import trimesh
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
import matplotlib.pyplot as plt
from database import dbaccess as db
import tf
import pandaplotutils.pandactrl as pandactrl

import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout
from  tf_util import PandaPosMax_t_PosMat, PosMat_t_PandaPosMax
import random


class RegripPlanner():
    """
    this is planner to plan a sequence of placement and regrasping on table for in-hand regrasping.
    """
    def __init__(self, objpath, handpkg, gdb, dmg_planner, offset=0.0):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.facets, self.facetnormals = self.objtrimesh.facets_over(faceangle=.95, segangle = .95)
        self.objgeom = pandageom.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.8])

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # placement graph 
        self.PlacementG = nx.Graph() 

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = offset)
        self.bulletworld.attachRigidBody(self.planebullnode)

        self.bulletworldhplowest = BulletWorld()
        self.planebullnode1 = cd.genCollisionPlane(offset=-5)
        self.bulletworldhplowest.attachRigidBody(self.planebullnode1)

        self.gdb = gdb
        self.dmg_planner = dmg_planner

        self.__loadFreeAirGrip()
        self.__loadFreeTablePlacement()
        self.handtmp = None

        self.objectid = self.gdb.loadIdObject(self.dbobjname)

        self.inital_grasp = "int_g"
        self.end_grasp = "end_g"


    def __loadFreeAirGrip(self):
        """
        This function will load all grasps related to the object in the object frame.
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
        """
        This function will return a list of placement poses, placement ids, and placement types.
        The first part of the list are stable placements, while the second part of the list are unstable placements.
        """
        freetabletopplacementdata = self.gdb.loadFreeTabletopPlacementIncludeFF(self.dbobjname)
        if freetabletopplacementdata is None:
            raise ValueError("Plan the freeairgrip first!")
        self.tpsmat4s, self.placementid, self.placementtype = freetabletopplacementdata

    def getPointFromPose(self, pose, point):
        """
        Calculate the point position with a pose.
        """
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])

    def getPlaneWithTwoPoints(self, p1, p0):
        """
        Given two points on the different size of a plane, this function will return the plane's value (position, normal direction).
        """
        normal_direction = np.array([p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]])
        normal_direction = normal_direction / np.linalg.norm(normal_direction)
        plane_center = np.array([p1[0] + p0[0], p1[1] + p0[1], p1[2] + p0[2]]) / 2
        return np.concatenate((plane_center, normal_direction))

    def removeEdge(self, graspid1, graspid2):
        self.PlacementG.remove_edge(graspid1, graspid2)

    def CreatePlacementGraph(self):
        """
        when you create the node, each stable placement and fixtureless fixturing motion plane will be considered
        as one.
        """

        # create nodes for grasp poses
        for g in range(len(self.freegripid)):
            self.PlacementG.add_node(self.freegripid[g], pose=PandaPosMax_t_PosMat(self.freegriprotmats[g]), jawwidth = self.freegripjawwidth[g] / 1000.0)
        
        ## build the connections between nodes
        ## if the placement is stable
        for p in range(len(self.placementid)):
            if self.placementtype[p] == 0: # insert a stable placement into the graph
                
                # sample a set of placement poses of this placement id
                placementposeset = []
                pose_0 = PandaPosMax_t_PosMat(self.tpsmat4s[p])
                for tableangle in [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]:
                    rotationInZ = np.identity(4)
                    rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
                    placementposeset.append([pose_0.dot(rotationInZ), 'stable', self.placementid[p]])

                sql = "SELECT idfreeairgrip FROM freetabletopgrip WHERE idfreetabletopplacement=%d" % self.placementid[p]
                relativeGraspids = [int(item[0]) for item in self.gdb.execute(sql)]
                if len(relativeGraspids) > 1:
                    for edge in list(itertools.combinations(relativeGraspids, 2)):
                        if not self.PlacementG.has_edge(*edge):
                            self.PlacementG.add_edge(*edge, placementposes=placementposeset)
                        else:
                            temp = self.PlacementG.edges[edge[0],edge[1]]['placementposes']
                            self.PlacementG.add_edge(*edge, placementposes=temp + placementposeset)

        # if placement is unstable
        sql = "SELECT iddmg, placementpose FROM dmgs"
        result = self.gdb.execute(sql)
        for r in result:
            dmgid = int(r[0])
            pose_0 = PandaPosMax_t_PosMat(dc.strToMat4(r[1]))
            placementposeset = []
            for tableangle in [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]:
                rotationInZ = np.identity(4)
                rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
                placementposeset.append([pose_0.dot(rotationInZ), 'unstable', dmgid])


            sql = "SELECT idfreeairgrip FROM graspid2dmgid WHERE iddmg=%d" % dmgid
            relativeGraspids = [int(item[0]) for item in self.gdb.execute(sql)]

            if len(relativeGraspids) > 1:
                for edge in list(itertools.combinations(relativeGraspids, 2)):
                    if not self.PlacementG.has_edge(*edge):
                        self.PlacementG.add_edge(*edge, placementposes=placementposeset)
                    else:
                        temp = self.PlacementG.edges[edge[0],edge[1]]['placementposes']
                        self.PlacementG.add_edge(*edge, placementposes=temp + placementposeset)
        
        # remove the grasp which can't be used for re-grasping
        deletenodeindex = []
        for n in self.PlacementG:
            if len(list(self.PlacementG[n])) == 0:
                deletenodeindex.append(n)
        for d in deletenodeindex:
            self.PlacementG.remove_node(d)


    def addStartGrasp(self, startrotmat4_, starthandwidth, base):
        """
        add the initial grasp into the regrasping graph, and start grasp is in numpy format.
        """

        # sql = "SELECT iddmg FROM graspid2dmgid WHERE idfreeairgrip=%d" % startgraspid
        
        # for dmgid in self.gdb.execute(sql):
        #     self.PlacementG.add_edge('int_g', -int(dmgid[0]), graspid=[(startrotmat4_, starthandwidth)])
        # print("connect start grasp to ", [-int(r[0]) for r in self.gdb.execute(sql)])
    
        startrotmat4 = PosMat_t_PandaPosMax(startrotmat4_)
        self.PlacementG.add_node(self.inital_grasp, pose=startrotmat4_, jawwidth = starthandwidth)
        startrotmat4_temp = np.copy(startrotmat4_)
        startrotmat4_temp[0][3] *= 1000.0
        startrotmat4_temp[1][3] *= 1000.0
        startrotmat4_temp[2][3] *= 1000.0

        for p in range(len(self.placementid)):
            currentplacementpose = PandaPosMax_t_PosMat(self.tpsmat4s[p])
            currentplacementpose[0][3] *= 1000.0
            currentplacementpose[1][3] *= 1000.0
            currentplacementpose[2][3] *= 1000.0

            p1 = self.getPointFromPose(startrotmat4 * self.tpsmat4s[p], [0, 1, 0])
            p0 = self.getPointFromPose(startrotmat4 * self.tpsmat4s[p], [0, -1, 0])
            # if the hand does not hit the ground, then this placement can connect to the goal node
            tmphnd = fetch_grippernm.Fetch_gripperNM(hndcolor=[1, 0, 0, 0])
            tmphnd.setJawwidth(starthandwidth*1000.0)
            tmphnd.setMat(pandanpmat4 = startrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            result1 = self.bulletworldhplowest.contactTest(hndbullnode)

            tmphnd.removeNode()
            if not result.getNumContacts() and not result1.getNumContacts():
                if self.placementtype[p] == 0: # when placement is stable
                    # sample a set of placement poses of this placement id
                    placementposeset = []
                    pose_0 = PandaPosMax_t_PosMat(self.tpsmat4s[p])
                    for tableangle in [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]:
                        rotationInZ = np.identity(4)
                        rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
                        placementposeset.append([pose_0.dot(rotationInZ), 'stable', self.placementid[p]])

                    sql = "SELECT idfreeairgrip FROM freetabletopgrip WHERE idfreetabletopplacement=%d" % self.placementid[p]
                    relativeGraspids = [int(item[0]) for item in self.gdb.execute(sql)]
                    for g in relativeGraspids:
                        self.PlacementG.add_edge(self.inital_grasp , g, placementposes=placementposeset)
                else:

                    self.dmg_planner.renderObject(base, self.tpsmat4s[p])
                    sql = "SELECT dmgs.iddmg, dmgs.planevector FROM dmgs WHERE dmgs.placementid=%d " % self.placementid[p]
                    for dmgid, dmgplane in self.gdb.execute(sql):
                        # sample a set of placement poses of this placement id
                        placementposeset = []
                        pose_0 = PandaPosMax_t_PosMat(self.tpsmat4s[p])
                        for tableangle in [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]:
                            rotationInZ = np.identity(4)
                            rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
                            placementposeset.append([pose_0.dot(rotationInZ), 'unstable', dmgid])

                        dmg_plane = dc.strToV6(dmgplane)

                        grasp_plane = self.getPlaneWithTwoPoints(p1, p0)
                        if np.linalg.norm(grasp_plane[3:6] - dmg_plane[3:6]) <= 0.1 and (grasp_plane[3] * (dmg_plane[0] - grasp_plane[0]) + grasp_plane[4] * (dmg_plane[1] - grasp_plane[1]) + grasp_plane[5] * (dmg_plane[2] - grasp_plane[2]) < 10.0):
                            # find all grasps related to the this dmg
                            sql = "SELECT idfreeairgrip FROM graspid2dmgid WHERE iddmg=%d" % dmgid
                            relativeGraspids = [int(item[0]) for item in self.gdb.execute(sql)]

                            sql = "SELECT freeairgrip.rotmat FROM graspid2dmgid, freeairgrip WHERE freeairgrip.idfreeairgrip=graspid2dmgid.idfreeairgrip AND graspid2dmgid.iddmg=%d" % dmgid
                            
                            for grasppose in [PandaPosMax_t_PosMat(dc.strToMat4(gp[0])) for gp in self.gdb.execute(sql)]:
                                grasppose[0][3] *= 1000.0
                                grasppose[1][3] *= 1000.0
                                grasppose[2][3] *= 1000.0
                                if self.dmg_planner.checkCollisionBetweenGrasps(currentplacementpose.dot(startrotmat4_temp), currentplacementpose.dot(grasppose), starthandwidth * 1000.0, base):
                                    for g in relativeGraspids:
                                        self.PlacementG.add_edge(self.inital_grasp, g, placementposes=placementposeset)
                                    break
                    self.dmg_planner.cleanRenderedObject(base)

                            
    def addGoalGrasp(self, goalrotmat4_, goalhandwidth, base):
        """
        add target grasp into the regrasping graph, and target grasp in numpy format
        """

        # sql = "SELECT iddmg FROM graspid2dmgid WHERE idfreeairgrip=%d" % goalgraspid
        # for dmgid in self.gdb.execute(sql):
        #     self.PlacementG.add_edge('end_g', -int(dmgid[0]), graspid=[(goalrotmat4, goalhandwidth)])

        goalrotmat4 = PosMat_t_PandaPosMax(goalrotmat4_)
        self.PlacementG.add_node(self.end_grasp, pose=goalrotmat4_, jawwidth = goalhandwidth)

        goalrotmat4_temp = np.copy(goalrotmat4_)
        goalrotmat4_temp[0][3] *= 1000.0
        goalrotmat4_temp[1][3] *= 1000.0
        goalrotmat4_temp[2][3] *= 1000.0

        for p in range(len(self.placementid)):
            currentplacementpose = PandaPosMax_t_PosMat(self.tpsmat4s[p])
            currentplacementpose[0][3] *= 1000.0
            currentplacementpose[1][3] *= 1000.0
            currentplacementpose[2][3] *= 1000.0

            p1 = self.getPointFromPose(goalrotmat4 * self.tpsmat4s[p], [0, 1, 0])
            p0 = self.getPointFromPose(goalrotmat4 * self.tpsmat4s[p], [0, -1, 0])
            # if the hand does not hit the ground, then this placement can connect to the goal node
            tmphnd = fetch_grippernm.Fetch_gripperNM(hndcolor=[1, 0, 0, 0])
            tmphnd.setJawwidth(goalhandwidth * 1000.0)
            tmphnd.setMat(pandanpmat4 = goalrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            result1 = self.bulletworldhplowest.contactTest(hndbullnode)

            tmphnd.removeNode()
            if not result.getNumContacts() and not result1.getNumContacts():
                if self.placementtype[p] == 0: # when placement is stable
                    # sample a set of placement poses of this placement id
                    placementposeset = []
                    pose_0 = PandaPosMax_t_PosMat(self.tpsmat4s[p])
                    for tableangle in [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]:
                        rotationInZ = np.identity(4)
                        rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
                        placementposeset.append([pose_0.dot(rotationInZ), 'stable', self.placementid[p]])

                    sql = "SELECT idfreeairgrip FROM freetabletopgrip WHERE idfreetabletopplacement=%d" % self.placementid[p]
                    relativeGraspids = [int(item[0]) for item in self.gdb.execute(sql)]
                    for g in relativeGraspids:
                        self.PlacementG.add_edge(self.end_grasp , g, placementposes=placementposeset)
                else:

                    self.dmg_planner.renderObject(base, self.tpsmat4s[p])
                    sql = "SELECT dmgs.iddmg, dmgs.planevector FROM dmgs WHERE dmgs.placementid=%d " % self.placementid[p]
                    for dmgid, dmgplane in self.gdb.execute(sql):
                        # sample a set of placement poses of this placement id
                        placementposeset = []
                        pose_0 = PandaPosMax_t_PosMat(self.tpsmat4s[p])
                        for tableangle in [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]:
                            rotationInZ = np.identity(4)
                            rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
                            placementposeset.append([pose_0.dot(rotationInZ), 'unstable', dmgid])

                        dmg_plane = dc.strToV6(dmgplane)

                        grasp_plane = self.getPlaneWithTwoPoints(p1, p0)
                        if np.linalg.norm(grasp_plane[3:6] - dmg_plane[3:6]) <= 0.1 and (grasp_plane[3] * (dmg_plane[0] - grasp_plane[0]) + grasp_plane[4] * (dmg_plane[1] - grasp_plane[1]) + grasp_plane[5] * (dmg_plane[2] - grasp_plane[2]) < 10.0):
                            # find all grasps related to the this dmg
                            sql = "SELECT idfreeairgrip FROM graspid2dmgid WHERE iddmg=%d" % dmgid
                            relativeGraspids = [int(item[0]) for item in self.gdb.execute(sql)]

                            sql = "SELECT freeairgrip.rotmat FROM graspid2dmgid, freeairgrip WHERE freeairgrip.idfreeairgrip=graspid2dmgid.idfreeairgrip AND graspid2dmgid.iddmg=%d" % dmgid
                            
                            for grasppose in [PandaPosMax_t_PosMat(dc.strToMat4(gp[0])) for gp in self.gdb.execute(sql)]:
                                grasppose[0][3] *= 1000.0
                                grasppose[1][3] *= 1000.0
                                grasppose[2][3] *= 1000.0
                                if self.dmg_planner.checkCollisionBetweenGrasps(currentplacementpose.dot(goalrotmat4_temp), currentplacementpose.dot(grasppose), goalhandwidth * 1000.0, base):
                                    for g in relativeGraspids:
                                        self.PlacementG.add_edge(self.end_grasp, g, placementposes=placementposeset)
                                    break
                    self.dmg_planner.cleanRenderedObject(base)

    def reset(self):
        self.PlacementG.remove_node(self.inital_grasp)
        self.PlacementG.remove_node(self.end_grasp)

    def has_path(self):
        return nx.has_path(self.PlacementG, self.inital_grasp, self.end_grasp)

    def find_first_level_path(self):
        path = nx.shortest_path(self.PlacementG, self.inital_grasp, self.end_grasp)
        grasps = [self.PlacementG.node[p]['pose'] for p in path]
        jawwidths = [self.PlacementG.node[p]['jawwidth'] for p in path]
        return path, grasps, jawwidths

    def findRegraspAction(self, placegraspid, pickgraspid):
        return self.PlacementG[placegraspid][pickgraspid]['placementposes']

    def find_shortest_PlacementG_path(self):
        """"
        this function will return all possible shortest path
        """
        paths = nx.all_shortest_paths(self.PlacementG, self.inital_grasp, self.end_grasp)
        results = []
        for path in paths:
            #remove end graps node and init grasp node in graph path.
            path.pop(0)
            path_and_type = []
            for p in path:
                type = self.PlacementG.nodes[p]["stable"]
                real_placement_id = self.PlacementG.nodes[p]["placement"]
                path_and_type.append((p,type,real_placement_id))
            results.append(path_and_type)
        return results

    def get_placement_grasp_trajectory(self,path_and_type):
        """
        Given a sequence of placement, this function will return the grasp id between each placement.
        """
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

    def getRandomGraspId(self):
        sql = "SELECT freeairgrip.idfreeairgrip, freeairgrip.rotmat FROM freeairgrip WHERE freeairgrip.idobject=%d" % (self.objectid)
        result = self.gdb.execute(sql)
        random_index = random.randint(0,len(result)-1)

        return int(result[random_index][0]), PandaPosMax_t_PosMat(dc.strToMat4(result[random_index][1]))

    def getGraspsById(self, graspids):
        """
        get the grasp pose of grasp ids under specific placement
        this function will return a list like
        [(pose1, jawwidth1), (pose2, jawwidth2), ...]
        """
        results = []
        for i in graspids:
            sql = "SELECT freeairgrip.rotmat, freeairgrip.jawwidth FROM freeairgrip WHERE \
                    freeairgrip.idfreeairgrip=%d AND freeairgrip.idobject=%d" % (i, self.objectid)
            result = self.gdb.execute(sql)
            if len(result) == 0:
                continue
            # grasppose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(result[0][0])
            grasppose = dc.strToMat4(result[0][0])
            jawwidth = float(result[0][1])
            # need to convert it back to normal metrics
            results.append((np.array([[grasppose[0][0],grasppose[1][0],grasppose[2][0],grasppose[3][0]/1000.0], \
                                     [grasppose[0][1],grasppose[1][1],grasppose[2][1],grasppose[3][1]/1000.0], \
                                     [grasppose[0][2],grasppose[1][2],grasppose[2][2],grasppose[3][2]/1000.0], \
                                     [grasppose[0][3],grasppose[1][3],grasppose[2][3],grasppose[3][3]]]), jawwidth/1000.0))
        return results

    def getGroundDirection(self, pose):
        """
        get the ground direction vector from pose in object frame
        """
        gnd_dir = np.array([0,0,-1]) #ground direction (i.e gravity vector )
        pos_r = pose[:3,:3]
        obj_grd_dir_pos = np.dot( np.transpose(pos_r), gnd_dir) # vector containing the dir of the object to the groud in the object frame
        obj_grd_dir_pos= obj_grd_dir_pos/ np.linalg.norm(obj_grd_dir_pos)
        return obj_grd_dir_pos

    def getPlacementIdFromPose(self, pose):
        """
        given a pose, this function will return the placement id whose has most simlilar pose.
        """
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

    def getPlacementsById(self, placementids):
        """
        get placements in normal format with list of placement id.
        """
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

    def showgraph(self):
        """
        Show the re-grasping graph.
        """
        # for node1, node2, data in g.edges(data=True):
        #     if data.has_key('placementposes'):
        #         g[node1][node2]['numofgrasps'] = len(data['placementposes'])
        #     else:
        #         g[node1][node2]['numofgrasps'] = 0 # here can only happen to initial grasp
            # print g[e[0]][e[1]], g[e[0]][e[1]]['placementposes'] 

        # edge_labels = nx.get_edge_attributes(g, "numofgrasps")
        print("number of connected co", nx.number_connected_components(self.PlacementG))
        # nx.draw(g, pos, with_labels=True, node_color=colormap, node_size=400)
        # nx.draw_networkx_edge_labels(g, edge_labels=edge_labels)
        nx.draw(self.PlacementG, with_labels=True)
        
        plt.draw()
        plt.show()

if __name__=='__main__':
    pass