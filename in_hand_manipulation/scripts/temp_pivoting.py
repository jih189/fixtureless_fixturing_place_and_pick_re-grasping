#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from tf_util import  TF_Helper, getMatrixFromQuaternionAndTrans, getTransformFromPoseMat, adjointRepresentationMatrix, skew_twist
from rail_segmentation.srv import SearchTable
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import json
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import copy
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import RobotTrajectory

from geometry_msgs.msg import Pose
from fetch_coppeliasim.srv import ObjectPose
from itertools import compress
import time
from scipy.linalg import expm

def detect_table_and_placement(robot_, numberOfSampling = 50, sizeOfTable2d = 500):
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
                tableresult.width * 1.0, tableresult.depth * 2, 0.02)
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

    circlesize = 50

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
    # last_action = result[0][0]
    visual_result = []
    
    for a, t in data[1:]:
        if result[-1][0] == a:
            result[-1][1] += t
        else:
            result.append([a, t])

    for a, t in result:
        if a == "setGripper" or a == "closeGripper":
            continue
        for e in t:
            e[0][3] /= 1000
            e[1][3] /= 1000
            e[2][3] /= 1000
            visual_result.append(e)

    with open('actionqueue/grasp_json_data.json') as json_file:
        grasp_data = json.load(json_file)
        grasp_data[0][3] /= 1000
        grasp_data[1][3] /= 1000
        grasp_data[2][3] /= 1000

    # subsampling the visual trajectory
    visual_result = [visual_result[i] for i in range(0, len(visual_result), 15)]

    return result, visual_result, grasp_data

if __name__=='__main__':

    isSim = True
    momThreshold = 2.1

    rospy.init_node('pivoting_node')
    robot = Fetch_Robot(sim=isSim)

    tf_helper = TF_Helper()

    # if running in simulation, we can have a object mover helping us
    objectMover = None
    if isSim:
        rospy.wait_for_service('move_object')
        objectMover = rospy.ServiceProxy('move_object', ObjectPose)

    # load json trajectory
    end_effector_trajectory, visual_end_effector_trajectory, object_grasp = loadEnd_effector_trajectory()

    # find a set of valid position on table for manipulation
    valid_manipulation_points = detect_table_and_placement(robot)
    
    marker_publisher = rospy.Publisher("trajectory_marker", MarkerArray, queue_size=10)

    markerArray = MarkerArray()
    init_ik_result = None
    current_init_grasp = None
    desired_object_trans_in_base = None
    found_solution = False
    totalPlan = []

    for p in valid_manipulation_points:
        testpoint = getMatrixFromQuaternionAndTrans(p[1], p[0])
        for tableangle in [0.0, 0.5235987755982988, 1.0471975511965976, 1.5707963267948966, 2.0943951023931953, 2.617993877991494, 3.141592653589793, 3.665191429188092, 4.1887902047863905, 4.71238898038469, 5.235987755982988, 5.759586531581287]:
            rotationInZ = np.identity(4)
            rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
            ## calculate the gripper pose on table
            current_testpoint = testpoint.dot(rotationInZ)

            # check the ik feasibility of the trajectory
            ik_feasible = True
            for g in [getTransformFromPoseMat(current_testpoint.dot(np.array(t))) for t in visual_end_effector_trajectory]:
                if robot.solve_ik_collision_free_in_base(g, 10) == None:
                    ik_feasible = False
                    break
            if not ik_feasible: # some grasp is not feasible
                continue

            # momOfTrajectory = []
            # needToRefine = False
            # refine_need_joints = None

            # use ik solver to check whether the initial pose is feasible or not
            current_init_grasp = getTransformFromPoseMat(current_testpoint.dot(np.array(end_effector_trajectory[0][1][0])))
            init_ik_result = robot.solve_ik_collision_free_in_base(current_init_grasp, 10, current_joint_seed)
            if init_ik_result == None:
                break
            
            # check initial manipubility is low or not
            if robot.get_mom(list(init_ik_result.state.joint_state.position)) < momThreshold:
                # if initial manipubility is too low, then skip it
                break
            
            # save current joint as the joint seed to refine process
            current_joint_seed = list(init_ik_result.state.joint_state.position)

            # generate cartesian path from the pose waypoints
            moveit_robot_state = copy.deepcopy(init_ik_result.state)

            totalPlan = []
            totalPlan.append(("gripper", robot.planto_open_gripper(), 0.08))
            totalPlan.append(("arm", robot.planto_joints(init_ik_result.state.joint_state.position, init_ik_result.state.joint_state.name)))

            path_feasible = True
            for a, t in end_effector_trajectory:
                if a == 'closeGripper':
                    totalPlan.append(("gripper", robot.planto_close_gripper(), 0.0))
                    continue
                elif a == 'setGripper':
                    totalPlan.append(("gripper", robot.planto_open_gripper(t/1000.0), t/1000.0))
                    continue
                
                (currentTrajectoryPlan, fraction) = robot.planFollowEndEffectorTrajectory(moveit_robot_state, [getTransformFromPoseMat(current_testpoint.dot(np.array(e))) for e in t], 0.05)
                if fraction != 1.0:
                    path_feasible = False
                    break
                
                # calculate the mom of the trajectory
                for p in range(0, len(currentTrajectoryPlan.joint_trajectory.points)):
                    current_joints = list(currentTrajectoryPlan.joint_trajectory.points[p].positions)
                    measureOfManipulability_fast = robot.get_mom(current_joints)
                    if measureOfManipulability_fast < momThreshold: # need to refine the trajectory
                        if not needToRefine: # record the first position where mom is lower than threshold
                            needToRefine = True
                            refine_need_joints = {}
                            for jn, jv in zip(list(currentTrajectoryPlan.joint_trajectory.joint_names), current_joints):
                                refine_need_joints[jn] = jv

                    momOfTrajectory.append(measureOfManipulability_fast)

                moveit_robot_state.joint_state.position = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].positions)
                moveit_robot_state.joint_state.velocity = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].velocities)
                moveit_robot_state.joint_state.effort = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].effort)

            if not path_feasible:
                break
            
            current_mom_score = max(min(momOfTrajectory), current_mom_score)
            print("current mom score = ", current_mom_score)

            refining_mom_of_trajectory = [momOfTrajectory]

            """
                if needToRefine:
                    needToRefine = False
                    # find where the gripper link is with refine needs joints
                    refineTrans = robot.get_fk(refine_need_joints, "gripper_link")
                    refinePose = getMatrixFromQuaternionAndTrans(refineTrans[0], refineTrans[1])

                    # calculate mom of refine need joints
                    refine_joint = [refine_need_joints[jn] for jn in robot.group.get_active_joints()]
                    measureOfManipulability_fast = robot.get_mom(refine_joint)

                    # calculate the surface normal vector of singular point
                    Jacobian_matrix = robot.get_jacobian_matrix(refine_joint)
                    pseudoInverse_matrix = np.linalg.pinv(Jacobian_matrix)
                    DerivativeOfMoM = []
                    for j in range(len(refine_joint)):
                        temp_joint = list(refine_joint)
                        temp_joint[j] += 0.01
                        DerivativeOfMoM.append(measureOfManipulability_fast * np.trace(((robot.get_jacobian_matrix(temp_joint) - Jacobian_matrix)/ 0.01).dot(pseudoInverse_matrix)))
                    disToSin = np.array(DerivativeOfMoM).dot(pseudoInverse_matrix)
                    singularity_normal = np.transpose(disToSin) / np.linalg.norm(disToSin)

                    # calculate the new position for the refine need joint
                    pre_refine_twist_in_base = np.concatenate((singularity_normal[3:], singularity_normal[:3]))
                    pre_refine_twist_on_table = np.dot(adjointRepresentationMatrix(np.linalg.inv(testpoint)), pre_refine_twist_in_base)
                    refine_twist_on_table = np.array([0,0,pre_refine_twist_on_table[2], pre_refine_twist_on_table[3], pre_refine_twist_on_table[4], 0])
                    print("refine twist on table ", refine_twist_on_table)
                    refine_twist_in_base = np.dot(adjointRepresentationMatrix(testpoint), refine_twist_on_table)
                    
                    # update the trajectory until the mom is higher than a threshold
                    new_mom_score = measureOfManipulability_fast
                    better_initial_position = None
                    refined_trans_list = []
                    
                    isRefinable = True

                    for update_dis in range(1, 10):
                        onestep_mom_of_trajectory = []
                        refined_pose_in_base = np.dot(expm(skew_twist(refine_twist_in_base) * 0.005 * update_dis), refinePose)
                        refined_trans_in_base = getTransformFromPoseMat(refined_pose_in_base)
                        ik_result = robot.solve_ik_collision_free_in_base(refined_trans_in_base, 5, refine_joint)
                        if ik_result == None:
                            print("failure of ik for refined point")
                            break
                        new_measureOfManipulability_fast = robot.get_mom(list(ik_result.state.joint_state.position))
                        if new_measureOfManipulability_fast > new_mom_score:
                            if update_dis == 1 and new_measureOfManipulability_fast - new_mom_score < 0.002: # check if the first refine is good enough
                                isRefinable = False # if it is not, then it mean this trajectory can not be refined.
                                break
                            print("improved value = ", new_measureOfManipulability_fast - new_mom_score)
                            new_mom_score = new_measureOfManipulability_fast
                            better_initial_position = refined_pose_in_base.dot(np.linalg.inv(refinePose)).dot(current_testpoint)
                        else:
                            print("mom of refined point is decreasing")
                            break

                        # draw the refine trajectory#########################################################################################
                        initial_position_temp = refined_pose_in_base.dot(np.linalg.inv(refinePose)).dot(current_testpoint)

                        refined_trans_list.append(getTransformFromPoseMat(initial_position_temp))

                        current_init_grasp_temp = getTransformFromPoseMat(initial_position_temp.dot(np.array(end_effector_trajectory[0][1][0])))
                        init_ik_result_temp = robot.solve_ik_collision_free_in_base(current_init_grasp_temp, 10, current_joint_seed)
                        if init_ik_result_temp == None:
                            print("failure of ik for initial point")
                            break
                        
                        # check initial manipubility is low or not
                        if robot.get_mom(list(init_ik_result_temp.state.joint_state.position)) < momThreshold:
                            # if initial manipubility is too low, then skip it
                            print("failure of low mom at initial point")
                            break
            
                        # generate cartesian path from the pose waypoints
                        moveit_robot_state_temp = copy.deepcopy(init_ik_result_temp.state)
                        path_feasible = True
                        for a, t in end_effector_trajectory:
                            if a == 'closeGripper' or a == 'setGripper': # skip gripper action
                                continue
                            (currentTrajectoryPlan_temp, fraction) = robot.planFollowEndEffectorTrajectory(moveit_robot_state_temp, [getTransformFromPoseMat(initial_position_temp.dot(np.array(e))) for e in t], 0.05)
                            if fraction != 1.0:
                                path_feasible = False
                                break
                            
                            # calculate the mom of the trajectory
                            for p in range(0, len(currentTrajectoryPlan_temp.joint_trajectory.points)):
                                onestep_mom_of_trajectory.append(robot.get_mom(list(currentTrajectoryPlan_temp.joint_trajectory.points[p].positions)))

                            moveit_robot_state_temp.joint_state.position = copy.deepcopy(currentTrajectoryPlan_temp.joint_trajectory.points[-1].positions)
                            moveit_robot_state_temp.joint_state.velocity = copy.deepcopy(currentTrajectoryPlan_temp.joint_trajectory.points[-1].velocities)
                            moveit_robot_state_temp.joint_state.effort = copy.deepcopy(currentTrajectoryPlan_temp.joint_trajectory.points[-1].effort)

                        if not path_feasible:
                            print("failure of infeasible path")
                            break

                        refining_mom_of_trajectory.append(onestep_mom_of_trajectory)

                        #####################################################################################################################
                    if not isRefinable:
                        print("not refinable")
                        break

                    if new_mom_score > measureOfManipulability_fast:
                        print("find a better initial position")
                        current_testpoint = better_initial_position
                    else:
                        print("can't find a better initial position")

                    if len(refining_mom_of_trajectory) <= 1: # no refinement available
                        break

                    for l in range(len(refining_mom_of_trajectory)):
                        plt.plot(refining_mom_of_trajectory[l], label="line" + str(l + 1))
                    # plt.ylim(ymin=0)
                    plt.legend()
                    plt.show()

                    while not rospy.is_shutdown():
                        for i, p in enumerate(refined_trans_list):
                            tf_helper.pubTransform("refine " + str(i), p)
                        rospy.sleep(0.5)

                    print("refine done")
                    exit()
                else:
                    print("find a trajectory without low manipubility")
                    break
                """

            if current_mom_score < momThreshold:
                continue
            

            '''
            # visualize the end-effector trajectory
            # for id, e in enumerate(visual_end_effector_trajectory):
            #     current_grasp_trans, current_grasp_rot = getTransformFromPoseMat(current_testpoint.dot(np.array(e)))

            #     marker = Marker()
            #     marker.id = id
            #     marker.header.frame_id = "/base_link"
            #     marker.type = marker.SPHERE
            #     marker.action = marker.ADD
            #     marker.scale.x = 0.02
            #     marker.scale.y = 0.02
            #     marker.scale.z = 0.02
            #     marker.color.a = 1.0
            #     marker.color.r = 1.0
            #     marker.color.g = 1.0
            #     marker.color.b = 0.0
            #     marker.pose.orientation.x = current_grasp_rot[0]
            #     marker.pose.orientation.y = current_grasp_rot[1]
            #     marker.pose.orientation.z = current_grasp_rot[2]
            #     marker.pose.orientation.w = current_grasp_rot[3]
            #     marker.pose.position.x = current_grasp_trans[0]
            #     marker.pose.position.y = current_grasp_trans[1]
            #     marker.pose.position.z = current_grasp_trans[2]

            #     markerArray.markers.append(marker)
            '''
            found_solution = True
            break
        if found_solution:
            break

    if not found_solution:
        print("there is no ik result")
    else:
        print("found solution")
        exit()
    #     robot.display_trajectory([p[1] for p in totalPlan])

    #     raw_input("execute plan")

    #     for p in totalPlan[:2]:
    #         if p[0] == "gripper":
    #             robot.setGripperWidth(p[2])
    #         elif p[0] == "arm":
    #             robot.execute_plan(p[1])

    #     # if it is in Sim, then we can move the object to the position where the robot can grasp
    #     # get the transform from world to baselink
    #     world_to_base_mat = tf_helper.getPoseMat("world", "base_link")

    #     desired_object_pose_mat_in_base = getMatrixFromQuaternionAndTrans(current_init_grasp[1], current_init_grasp[0]).dot(np.linalg.inv(np.array(object_grasp)))
    #     desired_object_trans_in_base = getTransformFromPoseMat(desired_object_pose_mat_in_base)
    #     desired_object_trans = getTransformFromPoseMat(world_to_base_mat.dot(desired_object_pose_mat_in_base))
    #     # need to show object marker in rviz

    #     object_marker_publisher = rospy.Publisher("object_marker", Marker, queue_size=10)

    #     marker = Marker()
    #     marker.header.frame_id = "/base_link"
    #     marker.type = marker.MESH_RESOURCE
    #     marker.mesh_resource = "package://in_hand_manipulation/objects/bottle.stl"
    #     marker.action = marker.ADD
    #     marker.color.a = 0.5
    #     marker.color.r = 0.0
    #     marker.color.g = 0.0
    #     marker.color.b = 1.0
    #     marker.scale.x = 1.0
    #     marker.scale.y = 1.0
    #     marker.scale.z = 1.0
    #     marker.pose.orientation.x = desired_object_trans_in_base[1][0]
    #     marker.pose.orientation.y = desired_object_trans_in_base[1][1]
    #     marker.pose.orientation.z = desired_object_trans_in_base[1][2]
    #     marker.pose.orientation.w = desired_object_trans_in_base[1][3]
    #     marker.pose.position.x = desired_object_trans_in_base[0][0]
    #     marker.pose.position.y = desired_object_trans_in_base[0][1]
    #     marker.pose.position.z = desired_object_trans_in_base[0][2]

    #     if isSim:
    #         # move the object to where the robot can grasp
    #         try:
    #             object_pose = Pose()
    #             object_pose.position.x = desired_object_trans[0][0]
    #             object_pose.position.y = desired_object_trans[0][1]
    #             object_pose.position.z = desired_object_trans[0][2]

    #             object_pose.orientation.x = desired_object_trans[1][0]
    #             object_pose.orientation.y = desired_object_trans[1][1]
    #             object_pose.orientation.z = desired_object_trans[1][2]
    #             object_pose.orientation.w = desired_object_trans[1][3]

    #             objectMover("bottle", object_pose)
    #         except rospy.ServiceException as exc:
    #             print("Service did not process request: " + str(exc))
    #             exit()

    #     # while not rospy.is_shutdown():
    #     #     object_marker_publisher.publish(marker)
    #     #     rospy.sleep(0.5)  
    #     # exit()  
    #     raw_input("ready to pivot")

    #     # robot.addManipulatedObject("object", desired_object_trans_in_base[0][0],  desired_object_trans_in_base[0][1],  desired_object_trans_in_base[0][2],
    #     #                 desired_object_trans_in_base[1][0], desired_object_trans_in_base[1][1], desired_object_trans_in_base[1][2], desired_object_trans_in_base[1][3], "/home/lambda/catkin_ws/src/in_hand_manipulation/objects/bottle.stl")

    #     # execute the actions
    #     for p in totalPlan[2:]:
    #         if p[0] == "gripper":
    #             robot.setGripperWidth(p[2])
    #         elif p[0] == "arm":
    #             # hope we are lucky enoughs
    #             for i in range(len(p[1].joint_trajectory.points) - 1):
    #                 if (p[1].joint_trajectory.points[i+1].time_from_start - p[1].joint_trajectory.points[i].time_from_start) == rospy.Duration(0):
    #                     p[1].joint_trajectory.points[i+1].time_from_start += rospy.Duration(secs=0, nsecs=10000)

    #             joint_error = robot.execute_plan(p[1])
    #             if joint_error > 1.0:
    #                 for i in range(len(p[1].joint_trajectory.points) - 1):
    #                     difftime = p[1].joint_trajectory.points[i+1].time_from_start - p[1].joint_trajectory.points[i].time_from_start
    #                     print(difftime, difftime==rospy.Duration(0))
    #                     if difftime==rospy.Duration(0):
    #                         print("---", p[1].joint_trajectory.points[i].time_from_start + difftime / 2)
    #                 # # print the error trajectory
    #                 print("something is wrong")
    #                 break

    #     final_trans, final_rot = robot.getCurrentHandFrameInBase()

    #     final_object_pose_mat_in_base = getMatrixFromQuaternionAndTrans(final_rot, final_trans).dot(np.linalg.inv(np.array(object_grasp)))
    #     final_object_trans_in_base = getTransformFromPoseMat(final_object_pose_mat_in_base)
    #     marker.pose.orientation.x = final_object_trans_in_base[1][0]
    #     marker.pose.orientation.y = final_object_trans_in_base[1][1]
    #     marker.pose.orientation.z = final_object_trans_in_base[1][2]
    #     marker.pose.orientation.w = final_object_trans_in_base[1][3]
    #     marker.pose.position.x = final_object_trans_in_base[0][0]
    #     marker.pose.position.y = final_object_trans_in_base[0][1]
    #     marker.pose.position.z = final_object_trans_in_base[0][2]

    #     while not rospy.is_shutdown():
    #         object_marker_publisher.publish(marker)
    #         rospy.sleep(0.5)

    # # while not rospy.is_shutdown():
    # #     for i, p in enumerate(valid_manipulation_points):
    # #         tf_helper.pubTransform("pose " + str(i), p)
    # #     marker_publisher.publish(markerArray)
    # #     rospy.sleep(0.5)
        