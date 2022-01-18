#!/usr/bin/python

from __future__ import with_statement
import os
import itertools

import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.core import *

from manipulation.grip.fetch_gripper import fetch_grippernm

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
from  tf_util import PandaPosMax_t_PosMat, PosMat_t_PandaPosMax


class RegripPlanner():
    """
    this is planner to plan a sequence of placement and regrasping on table for in-hand regrasping.
    """
    def __init__(self, objpath, handpkg, gdb, offset=0.0):
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
        self.planebullnode1 = cd.genCollisionPlane(offset=35)
        self.bulletworldhplowest.attachRigidBody(self.planebullnode1)

        self.gdb = gdb

        self.__loadFreeAirGrip()
        self.__loadFreeTablePlacement()
        self.handtmp = None

        self.objectid = self.gdb.loadIdObject(self.dbobjname)

        self.inital_grasp = "int_g"
        self.PlacementG.add_node(self.inital_grasp, stable=-1)

        self.end_grasp = "end_g"
        self.PlacementG.add_node(self.end_grasp, stable=-2)


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

    def removeEdgeOnG(self, p1, p2):
        self.PlacementG.remove_edge(p1, p2)

    def CreatePlacementGraph(self):
        """
        when you create the node, each stable placement and fixtureless fixturing motion plane will be considered
        as one.
        """
        # create the nodes in the graph
        for p in range(len(self.placementid)):
            if self.placementtype[p] == 0: # insert a stable placement into the graph
                self.PlacementG.add_node(self.placementid[p], stable=self.placementtype[p], placement=self.placementid[p])
            else:
                # search for the dmg belong to this ff placement, and dmg will be represented as a negative value.
                for dmgid in self.gdb.execute("SELECT dmgs.iddmg FROM dmgs WHERE dmgs.placementid=%d" % self.placementid[p]):
                    self.PlacementG.add_node(-int(dmgid[0]), stable=self.placementtype[p], placement=self.placementid[p])
        
        # create the edges in the graph
        for i in range(len(self.freegripid)):
            # get stable placement ids belong to current grasp
            sql = "SELECT freetabletopgrip.idfreetabletopplacement FROM freetabletopgrip, freetabletopplacement WHERE \
                freetabletopgrip.idfreeairgrip=%d AND freetabletopplacement.idfreetabletopplacement=freetabletopgrip.idfreetabletopplacement \
                AND freetabletopplacement.placement=0 " % self.freegripid[i]
            relatedstableplacementid = self.gdb.execute(sql)

            # get dmg ids belong to current grasp
            sql = "SELECT dmgs.iddmg, dmgs.planevector, dmgs.placementpose FROM dmgs, freetabletopgrip, freetabletopplacement WHERE \
                freetabletopgrip.idfreeairgrip=%d AND freetabletopplacement.idfreetabletopplacement=freetabletopgrip.idfreetabletopplacement \
                AND freetabletopplacement.placement=1 AND dmgs.placementid=freetabletopgrip.idfreetabletopplacement " % self.freegripid[i]
            relateddmg = self.gdb.execute(sql)

            relateddmgid = []
            
            for u in range(len(relateddmg)):
                # calculate grasp plane
                c0 = self.getPointFromPose(self.freegriprotmats[i] * dc.strToMat4(relateddmg[u][2]), Point3(0,-1,0))
                c1 = self.getPointFromPose(self.freegriprotmats[i] * dc.strToMat4(relateddmg[u][2]), Point3(0,1,0))
                grasp_plane = self.getPlaneWithTwoPoints(c1, c0)
                # calculate dmg plane
                dmg_plane = dc.strToV6(relateddmg[u][1])
                if np.linalg.norm(grasp_plane[3:6] - dmg_plane[3:6]) <= 0.1 and (grasp_plane[3] * (dmg_plane[0] - grasp_plane[0]) + grasp_plane[4] * (dmg_plane[1] - grasp_plane[1]) + grasp_plane[5] * (dmg_plane[2] - grasp_plane[2]) < 10.0):
                    relateddmgid.append(-int(relateddmg[u][0]))

            relatednodes = relatedstableplacementid + relateddmgid

            if len(relatednodes) > 1:
                for edge in list(itertools.combinations(relatednodes, 2)):
                    if not self.PlacementG.has_edge(*edge):
                        self.PlacementG.add_edge(*edge, graspid=[self.freegripid[i]])
                    else:
                        temp = self.PlacementG.edges[edge[0],edge[1]]['graspid']
                        temp.append(self.freegripid[i])
                        self.PlacementG.add_edge(*edge, graspid=temp)

    def addStartGrasp(self, startrotmat4, starthandwidth):
        """
        add the initial grasp into the regrasping graph, and start grasp is in numpy format.
        """

        startrotmat4 = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * PosMat_t_PandaPosMax(startrotmat4)
        starthandwidth *= 1000
        
        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):
            p1 = self.getPointFromPose(startrotmat4 * self.tpsmat4s[p], [0, 1, 0])
            p0 = self.getPointFromPose(startrotmat4 * self.tpsmat4s[p], [0, -1, 0])
            if self.placementtype[p] != 0 and abs(p1[2] - p0[2]) >= 0.2:
                continue

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
                    self.PlacementG.add_edge('int_g', self.placementid[p], graspid=[(startrotmat4, starthandwidth)])
                else: # when placement is unstable
                    # need to check which dmg plane this grasp belongs to
                    sql = "SELECT dmgs.iddmg, dmgs.planevector FROM dmgs WHERE dmgs.placementid=%d " % self.placementid[p]
                    for dmgid, dmgplane in self.gdb.execute(sql):
                        dmg_plane = dc.strToV6(dmgplane)

                        grasp_plane = self.getPlaneWithTwoPoints(p1, p0)
                        if np.linalg.norm(grasp_plane[3:6] - dmg_plane[3:6]) <= 0.1 and (grasp_plane[3] * (dmg_plane[0] - grasp_plane[0]) + grasp_plane[4] * (dmg_plane[1] - grasp_plane[1]) + grasp_plane[5] * (dmg_plane[2] - grasp_plane[2]) < 10.0):
                            self.PlacementG.add_edge('int_g', -int(dmgid), graspid=[(startrotmat4, starthandwidth)])

    def addGoalGrasp(self, goalrotmat4, goalhandwidth):
        """
        add target grasp into the regrasping graph, and target grasp in numpy format
        """

        goalrotmat4 = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * PosMat_t_PandaPosMax(goalrotmat4)
        goalhandwidth *= 1000

        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):
            p1 = self.getPointFromPose(goalrotmat4 * self.tpsmat4s[p], [0, 1, 0])
            p0 = self.getPointFromPose(goalrotmat4 * self.tpsmat4s[p], [0, -1, 0])
            if self.placementtype[p] != 0 and abs(p1[2] - p0[2]) >= 0.2:
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
                if self.placementtype[p] == 0:
                    addededge = ('end_g', self.placementid[p])
                    if not self.PlacementG.has_edge(*addededge):
                        self.PlacementG.add_edge(*addededge, graspid = [(goalrotmat4, goalhandwidth)])
                    else:
                        temp = self.PlacementG.edges['end_g', self.placementid[p]]['graspid']
                        temp.append((goalrotmat4, goalhandwidth))
                        self.PlacementG.add_edge(*addededge, graspid = temp)
                else:
                    sql = "SELECT dmgs.iddmg, dmgs.planevector FROM dmgs WHERE dmgs.placementid=%d " % self.placementid[p]
                    for dmgid, dmgplane in self.gdb.execute(sql):
                        addededge = ('end_g', -int(dmgid))
                        dmg_plane = dc.strToV6(dmgplane)
                        grasp_plane = self.getPlaneWithTwoPoints(p1, p0)
                        if np.linalg.norm(grasp_plane[3:6] - dmg_plane[3:6]) <= 0.1 and (grasp_plane[3] * (dmg_plane[0] - grasp_plane[0]) + grasp_plane[4] * (dmg_plane[1] - grasp_plane[1]) + grasp_plane[5] * (dmg_plane[2] - grasp_plane[2]) < 10.0):
                            if not self.PlacementG.has_edge(*addededge):
                                self.PlacementG.add_edge(*addededge, graspid = [(goalrotmat4, goalhandwidth)])
                            else:
                                temp = self.PlacementG.edges['end_g', self.placementid[p]]['graspid']
                                temp.append((goalrotmat4, goalhandwidth))
                                self.PlacementG.add_edge(*addededge, graspid = temp)
                            
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
                path_and_type.append((p,type))
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

    def getAllGrasps(self):
        """
        get all grasp poses in normal format and jawwdith in meter unit
        this function will return a list like
        [(pose1, jawwidth1), (pose2, jawwidth2), ...]
        """
        results = []
        for grasppose, jawwidth in zip(self.freegriprotmats, self.freegripjawwidth): 

            grasppose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * grasppose
            # need to convert it bakc to normal metrics
            results.append((np.array([[grasppose[0][0],grasppose[1][0],grasppose[2][0],grasppose[3][0]/1000.0], \
                                     [grasppose[0][1],grasppose[1][1],grasppose[2][1],grasppose[3][1]/1000.0], \
                                     [grasppose[0][2],grasppose[1][2],grasppose[2][2],grasppose[3][2]/1000.0], \
                                     [grasppose[0][3],grasppose[1][3],grasppose[2][3],grasppose[3][3]]]), jawwidth/1000.0))
        return results

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
            grasppose = pandageom.cvtMat4(rm.rodrigues([0, 1, 0], 180)) * dc.strToMat4(result[0][0])
            jawwidth = float(result[0][1]) / 1000
            # need to convert it back to normal metrics
            results.append((np.array([[grasppose[0][0],grasppose[1][0],grasppose[2][0],grasppose[3][0]/1000.0], \
                                     [grasppose[0][1],grasppose[1][1],grasppose[2][1],grasppose[3][1]/1000.0], \
                                     [grasppose[0][2],grasppose[1][2],grasppose[2][2],grasppose[3][2]/1000.0], \
                                     [grasppose[0][3],grasppose[1][3],grasppose[2][3],grasppose[3][3]]]), jawwidth))
        return results

    def getGraspIdsByPlacementId(self, placementid):
        """
        given the placement id, this function will return all grasp id related to it.
        """
        results = []
        sql =  "SELECT freetabletopgrip.idfreeairgrip FROM freetabletopgrip WHERE freetabletopgrip.idfreetabletopplacement=%d \
                    " % placementid
        result = self.gdb.execute(sql)

        for i in range(len(result)):
            results.append(int(result[i][0]))
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

    def getGraspsbyPlacementPose(self, placementTransform):
        """
        given object pose, this function use find the most similar placement according to the rotation
        part of the matrix, then return all grasps related to this pose.
        """
        t,r = placementTransform
        tran_base_object_pos = tf.TransformerROS().fromTranslationRotation(t,r)
        matched_obj_placement_id = self.getPlacementIdFromPose(tran_base_object_pos) #gets the most likly object placement given pose
        gripper_id_list = self.getGraspIdsByPlacementId(matched_obj_placement_id)
        return self.getGraspsById(gripper_id_list) # List of pos that that match placement

    def getCommonGraspids(self, p1, p2):
        return self.PlacementG[p1][p2]['graspid']

    # def showPlacementSequence(self, sequence, base):
    #     distancebetweencell = 300
    #     numOfP = len(sequence)
    #     showtable = np.zeros(numOfP)
    #     for i in range(len(showtable)):
    #         showtable[i] = (i - int(numOfP/2)) * distancebetweencell

    def showgraph(self):
        """
        Show the re-grasping graph.
        """
        g = self.PlacementG
        colormap = []
        for node1, node2, data in g.edges(data=True):
            if data.has_key('graspid'):
                g[node1][node2]['numofgrasps'] = len(data['graspid'])
            else:
                g[node1][node2]['numofgrasps'] = 0 # here can only happen to initial grasp
            # print g[e[0]][e[1]], g[e[0]][e[1]]['graspid'] 
        for n in g.nodes:
            if g.node[n]['stable'] == 0:
                colormap.append(g.node[n]['placement'])
            elif g.node[n]['stable'] == 1:
                colormap.append(g.node[n]['placement'])
            else:
                colormap.append(0)
        edge_labels = nx.get_edge_attributes(g, "numofgrasps")
        pos = nx.spring_layout(g, k=0.75, iterations=20)
        # nx.draw(g, pos, with_labels=True, node_color=colormap, node_size=400)
        nx.draw_networkx_edge_labels(g, pos, edge_labels=edge_labels)
        nx.draw(g, pos, with_labels=True, node_color=colormap)
        
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

        self.handtmp = fetch_grippernm.Fetch_gripperNM(hndcolor=[0, 1, 0, .5])
        self.handtmp.setMat(pandanpmat4=hndrotmat)
        self.handtmp.setJawwidth(hndjawwidth)
        self.handtmp.reparentTo(base.render)

    def showFrame(self, base):
        wp = WindowProperties()
        wp.setFullscreen(False)
        wp.setSize(800, 600)
        base.win.requestProperties(wp)

        base.disableMouse()
        base.cam.setPos(0, 1000, 0)
        base.cam.lookAt(0,0,0)

        base.graphicsEngine.renderFrame()
        base.graphicsEngine.renderFrame()

if __name__=='__main__':
    object_name = "book"

    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    # this_dir, this_filename = os.path.split(__file__)
    this_dir, this_filename = os.path.split(os.path.realpath(__file__))
    objpath = os.path.join(os.path.split(this_dir)[0], "objects", object_name + ".stl")
    handpkg = fetch_grippernm  #SQL grasping database interface 
    gdb = db.GraspDB()   #SQL grasping database interface

    planner = RegripPlanner(objpath, handpkg, gdb)

    def getInitGrasps(gdb, object_name):
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

    def getTargetGrasps(gdb, object_name):
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

    result, init_grasp = getInitGrasps(gdb,object_name)
    if not result:
        print "no initial grasp given"
        exit()
    result, target_grasps = getTargetGrasps(gdb,object_name)
    if not result:
        print "no target grasp given"
        exit()

    planner.CreatePlacementGraph()

    # add init grasps into the graph
    for grasp, jawwidth in init_grasp:
        print grasp
        print jawwidth
        planner.addStartGrasp(grasp, jawwidth)

    # add goal grasps into the graph
    for grasp, jawwidth in target_grasps:
        planner.addGoalGrasp(grasp, jawwidth)

    planner.showgraph()