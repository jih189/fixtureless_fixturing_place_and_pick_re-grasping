from posix import GRND_NONBLOCK
import networkx as nx
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.special import softmax

class dexterousManipulationGraph:
    def __init__(self, angle_range, edges):
        # this function will read a set of possible angles and edges to build the dexterous manipulation graph
        self.nodes = None # [ [g0,g5], [g1,g2],[g3,g4,gx...], ....]
        self.ang_range_node_idx = []  #[ 0,0, 1,1 2,2,...]
        self.Graph = None
        _convert_to_networkX(angle_range,edges) #Sets nodes and creates a graph with edges using nodes
        
    def _convert_to_networkX(self, angle_range, edges):
        G = nx.Graph()

        simp_angle_range = [item for sublist in angle_range[0] for item in sublist]
        self.nodes = simp_angle_range

        for i , angel_range in enumerate(self.nodes):
            G.add_node(i)
            for _ in angel_range:                    
                self.ang_range_node_cnt.append(i)
        
        helper_sae = [0]
        for i in range(1,len(angle_range)):
            helper_sae.append(len(angle_range[i-1]))

        for i in range(0,len(edges)):
            e1 = helper_sae[ edges[i][0][0] ] + edges[i][0][1]
            e2 = helper_sae[ edges[i][1][0] ] + edges[i][1][1]
            #simp_angle_edge.append((e1,e2))
            G.add_edge(e1,e2)

        self.Graph = G

    def get_closest_angle_range(self, grasp):
        "Given the a grasp, this function will return the closeset angel range to that grasp"
        tran_diff = []
        rot_diff = []
        grasp_inv_rot = np.transpose(grasp[:3,:3])
        grasp_trans = grasp[:,3:]

        for angel_range in self.nodes:
            for pos in angel_range:
                pos_rot = pos[:3,:3]
                pos_tran = pos[:,3:]

                rot = R.from_dcm(np.dot(grasp_inv_rot,pos_rot))
                crnt_rot_diff = np.linalg.norm(rot.as_rotvec())

                crnt_tran_diff = np.linalg.norm(pos_tran - grasp_trans)
                tran_diff.append(crnt_tran_diff)
                rot_diff.append(crnt_rot_diff)
                            
        
        #Normilize and add trans and rot diff together
        tran_diff = softmax(tran_diff)
        rot_diff = softmax(rot_diff)
        tran_N_rot_dif = [ x[0]+x[1]  for x in zip(tran_diff, rot_diff) ]

        closet_ang_range_idx = self.ang_range_node_idx[np.argmin(tran_N_rot_dif)] #find the min diff, return angle_range idx
        return self.nodes[closet_ang_range_idx], closet_ang_range_idx

             

    def isConnected(self, grasp, angle_range):
        #Checks if the grasp can connect to any in angle_range
        #Return true, and the connected grasp
        pass

    def insert_grasp_2_DMG(self,grasp):
        closest_angel_range, clst_ang_rang_idx = self.get_closest_angle_range(grasp)
        #Only checks the closet angel range for connection. 
        if isConnected(grasp,closest_angel_range):
            self.nodes.append([grasp])
            grasp_idx = len(self.nodes)
            self.ang_range_node_idx.append(grasp_idx)

            Graph.add_node(grasp_idx)
            Graph.add_edge(clst_ang_rang_idx,grasp_idx)
            return grasp_idx
        else:
            print("Grasp not inserted! Closest angel_range not connected")

        "Checks if grasp points have a possible path"
    def is_trajectory_possible(self, start_grasp,end_grasp):
        
        all_pos = [item for sublist in self.nodes for item in sublist]
        st_pos_deteced = False
        ed_pos_deteced = False
        st_grasp_idx = None
        ed_grasp_idx = None
        #Check to see if start or end graps already belong in DMG. If so we return the node idx for that grasp
        for i, pos in enumerate(all_pos):
            if (start_grasp == pos).all():
                st_pos_deteced = True
                st_grasp_idx = self.ang_range_node_idx[i]
            if (end_grasp == pos).all():  
                ed_pos_deteced = True                  
                ed_grasp_idx = self.ang_range_node_idx[i]
            if st_pos_deteced & ed_pos_deteced:
                break
        
        if not st_pos_deteced:
            st_grasp_idx = self.insert_grasp_2_DMG(start_grasp)
        if not ed_pos_deteced:
            ed_grasp_idx = self.insert_grasp_2_DMG(end_grasp)
        
        del all_pos
        return nx.path.bidirectional_dijkstra(self.Graph,st_grasp_idx,ed_grasp_idx)
       


    def getGraspTrajectory(self, start_grasp, end_grasp):
        # this function will insert both start grasp and end grasp into the dexterous manipulation graph
        # and plan for the grasp trajectory from start grasp to end grasp
        self.is_trajectory_possible(start_grasp,end_grasp)
        pass
    
class ff_regrasp_planner:
    def __init__(self,gipper_path, obj_path, gdb):
        # in this class, each placement could have several motion plane, and each motion plane has one dexterous manipulation graph
        self.mesh = loadMesh(obj_path)
        self.regrasp_graphs = []
        self.placement_directions = [] # list of placement direction. Ex. [(x1, y1, z1), (x2, y2, z2), ...]
        self.plane_normal_poses = [] # list of list of normal of plane. Ex. [[(x1, y1, z1, nx1, ny1, nz1), ...], [], ...]
        pass

    def isInSamePlane(grasp1,grasp2):
        pass
    
    def check_connect(self, grasp1, grasp2):
        if isInSamePlane(grasp1, grasp2):
            g = get_DMG(grasp1)
            return g.is_trajectory_possible(grasp1, grasp2)
        else:
            return False

    def check_valid(current_pose, next_pose, placement):
        # give the curent pose and next pose, this function will check whether the gripper can move from start to end directly without collision.
        pass
    def project_3d_to_2d_plane(self, point, plane):
        # this function will project 3d points to 2d, where object mass center will be considered as the center of the plane
        pass
    def get_contact_point_edge_by_voronoi(self, grasp_point_2d):
        # generate the voronoi graph on grasp point, if two point share one edge, then make a edge between them
        # it will return a list of edge(point pair index)
        return [(), ()]
    def generate_angle_and_feasibleMask(self, point_pairs):
        pass # this function will generate different direction of grasps based on point pair and mask bit which represents the feasible mask
    def get_DMG(self, placement_pose, grasp_pose):
        # given placement pose and grasp pose, this function will return the proper dexterous manipulation graph
        placement_direction = getPlacementDirection(placement_pose)
        placement_id = getPlacementIdByPlacementDirection(self.placement_directions, placement_direction)
        plane_normal_pose_id = getPlaneNormalPose(self.place_normal_poses[placement_id], grasp_pose)
        return self.regrasp_graph[placement_id][plane_normal_pose_id]

    def has_common_mask_bit(self, mask1, mask2):
        return mask1 & mask2

    def generateAngleRange(self, angleSet, angleMask):
        pass

    def angle_range_edge_converter(angle_mask,j):

    # generate a set of DMGs of given placement id
    def build_regrasp_graph(self, placementid, grasp_point_pairs_lists, plane_lists):
        placement = getPlacementById(placementid)
        #this function will build the DMG for specific placement
        render_object_by_placement(placement) # render the object in placement
        
        angle_range_edges = []
        angleRanges = []
        # this loop will go over through each plane
        # grasp_point_pairs_lists: [[grasp point pairs list for plane 1],[grasp point pairs list for plane 2],...]
        # plane_lists: [(x,y,z,nx,ny,nz),.....]
        for planeid, grasp_point_pairs, plane in enumerate(zip(grasp_point_pairs_lists, plane_lists)):
            angleSet, angleMask = self.generate_angle_and_feasibleMask(grasp_point_pairs)
            # angleSet: a set of list of grasp which share grasp point pair
            # angleSet = [[grasp1_0, grasp2_0....], [grasp1_1, grasp2_1....]] where graspx_0 where share the same contact point
            # angleMask = [[00011100,00110010],...] where each bit represent the state of that grasp is feasible or not
            # generate the edge between contact point pair
            voroni_grasp_point_edges = self.get_contact_point_edge_by_voronoi(grasp_point_pairs, plane)
            
            #[[idx1, idx2], ...] where idx1 and idx2 are the index of grasp_point_pairs_list
            for idx1, idx2 in vornoi_grasp_point_edges:
                for i in range(len(angleMask[idx1])):
                    for j in range(len(angleMask[idx2])):
                        if self.has_common_mask_bit(angleMask[idx1][i], angleMask[idx2][j]):
                            
                            #angle_range_edges.append(((idx1, i),(idx2, j))) 

            angleRanges.append(self.generateAngleRange(angleSet, angleMask), planeid) #[  [ [g1_0,g2_0],[g2_1,..] ],.....] all angel_range; set of all angel_range per point, angel_range
                                                                        # g1_0... are the grasp pose 
            # build the dexterous manipulation grasp
            return dexterousManipulationGraph(angleRanges, angle_range_edges)
    def group_grasp_points_by_planes(self, grasps):
        # this function will return a set of point pair list and its plane
        return [[pair1, pair2,...], [pair3, pair4,....],...], [plane1, pland2,.....]
        
    def build_regrasp_graph_for_all_placements():
        # need to initialize the regrasp graph with number of placement
        self.regrasp_graph = [[]] * len(ffplacements)
        # this function will build the DMG for all ff placements

        # analize each placement. ffplacement: fixtureles fixturing placement(unstable placement)
        for placementid, ffplacement in enumerate(ffplacements):
            # get all possible grasps under this placement
            grasps = ffplacement.getGrasps() #planner.getGraspsbyPlacementPose(tran_base_object)
            # this function returns a list of possible motion plan of this placement, and all grasp point pair related to it.
            grasp_point_pairs_list, plane_list = self.group_grasp_points_by_planes(grasps) # motion plane under that placement [ [grap_pair1_plan,gp2...] ,[gp1,gp2...]]; [info_plan1, i_p2]
            # build table for each motion plane, and init a DMG for it.
            for _ in range(len(plane_list)):
                dmg = self.build_regrasp_graph(placementid, grasp_point_pairs_list, plane_list)
                self.regrasp_graph[placementid].append(dmg)



    def main():
        regrasp_plan = ff_regrasp_planner()
        regrasp_plan.build_regrasp_graph_for_all_placements()
