#!/usr/bin/env python
import os
import rospy
from fetch_robot import Fetch_Robot
from geometry_msgs.msg import Pose2D
from panda3d.bullet import BulletDebugNode
from tf_util import  TF_Helper, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat
from rail_segmentation.srv import SearchTable
import numpy as np
from scipy.spatial.transform import Rotation as R
import pandaplotutils.pandageom as pandageom
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import json
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import copy
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory

def detect_table_and_placement(robot_, numberOfSampling = 20, sizeOfTable2d = 500):
    """
    Search for a table surface and sampling a set of position on the table for manipulation.
    """
    # call the table searcher server for searching table
    rospy.wait_for_service('table_searcher/search_table')
    tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

    # analyze where to manipulate the object
    try:
        tableresult = tableSearcher()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        return

    # add table into planning scene
    r = R.from_quat([tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w])
    # need to adjust the rotation of the table
    original_r = r.as_euler('zyx')
    table_quaternion = R.from_euler('zyx', [0,original_r[1], original_r[2]]).as_quat()

    robot_.addCollisionTable("table", tableresult.center.x, tableresult.center.y, tableresult.center.z, \
                table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
                tableresult.width * 1.0, tableresult.depth * 2, 0.001)
    # robot.attachTable("table")
    robot_.addCollisionTable("table_base", tableresult.center.x, tableresult.center.y, tableresult.center.z - 0.3, \
            table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
            tableresult.width, tableresult.depth, 0.6)

    pc_list = np.array([[p[0],p[1],p[2]] for p in pc2.read_points(tableresult.point_cloud, skip_nans=True, field_names=("x", "y", "z"))])

    tablewidth_high = max(pc_list[:, 0])
    tablewidth_low = min(pc_list[:, 0])
    tablewidth =  tablewidth_high - tablewidth_low
    tableheight_high = max(pc_list[:, 1])
    tableheight_low = min(pc_list[:, 1])
    tableheight = tableheight_high - tableheight_low
    tablesize = max(tablewidth, tableheight)
    tablegrid = np.zeros((int(sizeOfTable2d * tablewidth / tablesize), int(sizeOfTable2d * tableheight / tablesize)))
    for t in pc_list:
        tablegrid[min(int((t[0] - tablewidth_low)/(tablesize/sizeOfTable2d)), tablegrid.shape[0] - 1), min(int((t[1] - tableheight_low)/(tablesize/sizeOfTable2d)), tablegrid.shape[1] - 1)] = 1


    def create_circular_mask(h, w, center=None, radius=None):

        if center is None: # use the middle of the image
            center = (int(w/2), int(h/2))
        if radius is None: # use the smallest distance between the center and image walls
            radius = min(center[0], center[1], w-center[0], h-center[1])

        Y, X = np.ogrid[:h, :w]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

        mask = dist_from_center <= radius
        return mask

    circlesize = 70

    temp = np.zeros((circlesize, circlesize))
    temp[create_circular_mask(circlesize, circlesize)] = 1
    totalcirclenum = np.count_nonzero(temp)

    validindex = []
    for r in np.random.choice(pc_list.shape[0], numberOfSampling):
        tablegrid_temp = tablegrid.copy()
        x = int((pc_list[r, 0] - tablewidth_low)/(tablesize/sizeOfTable2d))
        y = int((pc_list[r, 1] - tableheight_low)/(tablesize/sizeOfTable2d))
        tablegrid_temp[~create_circular_mask(tablegrid_temp.shape[0], tablegrid_temp.shape[1], [y, x], int(circlesize/2))] = 0
        if np.count_nonzero(tablegrid_temp) > 0.9 * totalcirclenum:
            validindex.append(r)


    # # # Draw point based on above x, y axis values.
    # plt.scatter(pc_list[:, 0], pc_list[:, 1], s=1)
    # plt.scatter(pc_list[validindex, 0], pc_list[validindex, 1], s=3)
    # # Set x, y label text.
    # plt.show()

    return [(p, (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w)) for p in pc_list[validindex]]

def loadEnd_effector_trajectory():
    with open('actionqueue/json_data.json') as json_file:
        data = json.load(json_file)

    result = [[data[0][0], data[0][1]]]
    visual_result = []
    
    for a, t in data[1:]:
        if result[-1][0] == a:
            result[-1][1] += t
        else:
            result.append([a, t])

    for a, t in result:
        for e in t:
            e[0][3] /= 1000
            e[1][3] /= 1000
            e[2][3] /= 1000
            visual_result.append(e)

    return result, visual_result

if __name__=='__main__':

    isSim = True

    rospy.init_node('pivoting_node')
    robot = Fetch_Robot(sim=isSim)
    tf_helper = TF_Helper()

    # load json trajectory
    end_effector_trajectory, visual_end_effector_trajectory = loadEnd_effector_trajectory()

    # subsampling the visual trajectory
    visual_end_effector_trajectory = [visual_end_effector_trajectory[i] for i in range(0, len(visual_end_effector_trajectory), 4)]
    
    valid_manipulation_points = detect_table_and_placement(robot)
    
    marker_publisher = rospy.Publisher("trajectory_marker", MarkerArray, queue_size=10)

    markerArray = MarkerArray()
    init_ik_result = None
    found_result = False
    current_init_grasp = None
    totalPlan = []
    for p in valid_manipulation_points:
        testpoint = getMatrixFromQuaternionAndTrans(p[1], p[0])
        for tableangle in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
            rotationInZ = np.identity(4)
            rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
            ## calculate the gripper pose on table
            current_testpoint = testpoint.dot(rotationInZ)
            
            current_last_grasp = getTransformFromPoseMat(current_testpoint.dot(np.array(end_effector_trajectory[-1][1][-1])))
            if robot.solve_ik_collision_free_in_base(current_last_grasp, 30) == None:
                continue

            ik_feasible = True
            for g in [getTransformFromPoseMat(current_testpoint.dot(np.array(t[0]))) for _, t in end_effector_trajectory]:
                if robot.solve_ik_collision_free_in_base(g, 30) == None:
                    ik_feasible = False
                    break
            if not ik_feasible: # some grasp is not feasible
                continue

            # use ik solver to check whether the initial pose is feasible or not
            current_init_grasp = getTransformFromPoseMat(current_testpoint.dot(np.array(end_effector_trajectory[0][1][0])))
            init_ik_result = robot.solve_ik_collision_free_in_base(current_init_grasp, 30)

            moveit_robot_state = copy.deepcopy(init_ik_result.state)
            # start with move to initial position
            totalPlan = []
            totalPlan.append(robot.planto_open_gripper())
            totalPlan.append(robot.planto_joints(init_ik_result.state.joint_state.position, init_ik_result.state.joint_state.name))
            path_feasible = True
            for a, t in end_effector_trajectory:
                if a == 'pivot':
                    totalPlan.append(robot.planto_close_gripper())
                elif a == 'fingerGait':
                    totalPlan.append(robot.planto_open_gripper())
                (currentTrajectoryPlan, fraction) = robot.verifyEndEffectorTrajectory(moveit_robot_state, [getTransformFromPoseMat(current_testpoint.dot(np.array(e))) for e in t])
                if fraction < 0.9:
                    path_feasible = False
                    break
                totalPlan.append(currentTrajectoryPlan)
                moveit_robot_state.joint_state.position = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].positions)
                moveit_robot_state.joint_state.velocity = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].velocities)
                moveit_robot_state.joint_state.effort = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].effort)
            if not path_feasible:
                continue

            
            for id, e in enumerate(visual_end_effector_trajectory):
                current_grasp_trans, current_grasp_rot = getTransformFromPoseMat(current_testpoint.dot(np.array(e)))

                marker = Marker()
                marker.id = id
                marker.header.frame_id = "/base_link"
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.x = current_grasp_rot[0]
                marker.pose.orientation.y = current_grasp_rot[1]
                marker.pose.orientation.z = current_grasp_rot[2]
                marker.pose.orientation.w = current_grasp_rot[3]
                marker.pose.position.x = current_grasp_trans[0]
                marker.pose.position.y = current_grasp_trans[1]
                marker.pose.position.z = current_grasp_trans[2]

                markerArray.markers.append(marker)

            found_result = True
            break
        if found_result:
            break

    if not found_result:
        print("there is no ik result")
    else:
        print("found solution")
        robot.display_trajectory(totalPlan)

    while not rospy.is_shutdown():
        # for i, p in enumerate(valid_manipulation_points):
        #     tf_helper.pubTransform("pose " + str(i), p)
        marker_publisher.publish(markerArray)
        rospy.sleep(0.5)
        