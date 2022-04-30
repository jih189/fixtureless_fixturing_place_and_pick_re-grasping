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
from scipy.linalg import expm

def detect_table_and_placement(robot_, numberOfSampling = 300, sizeOfTable2d = 500):
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
                tableresult.width, tableresult.depth , 0.01)
    # robot.attachTable("table")
    robot_.addCollisionTable("table_base", tableresult.center.x, tableresult.center.y, tableresult.center.z - 0.15, \
            table_quaternion[0], table_quaternion[1], table_quaternion[2], table_quaternion[3], \
            tableresult.width, tableresult.depth, 0.3)

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
        if pc_list[r, 0] > 0.79 or pc_list[r, 0] < 0.5 or pc_list[r, 1] > 0.17 or pc_list[r, 1] < -0.17:
            continue
        tablegrid_temp = tablegrid.copy()
        x = int((pc_list[r, 0] - tablewidth_low)/(tablesize/sizeOfTable2d))
        y = int((pc_list[r, 1] - tableheight_low)/(tablesize/sizeOfTable2d))
        tablegrid_temp[~create_circular_mask(tablegrid_temp.shape[0], tablegrid_temp.shape[1], [y, x], int(circlesize/2))] = 0
        if np.count_nonzero(tablegrid_temp) > 0.08 * totalcirclenum:
            validindex.append(r)


    # # Draw point based on above x, y axis values.
    # plt.scatter(pc_list[:, 0], pc_list[:, 1], s=1)
    # plt.scatter(pc_list[validindex, 0], pc_list[validindex, 1], s=3)
    # # Set x, y label text.
    # plt.show()

    return [(p, (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w)) for p in pc_list[validindex]]

def loadEnd_effector_trajectory(object_name):
    with open('actionqueue/' + object_name + '_json_data.json') as json_file:
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

    with open('actionqueue/' + object_name + '_grasp_json_data.json') as json_file:
        grasp_data = json.load(json_file)
        grasp_data[0][3] /= 1000
        grasp_data[1][3] /= 1000
        grasp_data[2][3] /= 1000

    # subsampling the visual trajectory
    visual_result = [visual_result[i] for i in range(0, len(visual_result), 15)]

    return result, visual_result, grasp_data

if __name__=='__main__':

    isSim = True
    isSimple = False
    momThreshold = 2.06
    # momThreshold = 2.03
    # object_name = "book"
    object_name = "bottle"
    # object_name = "can"

    rospy.init_node('pivoting_node')
    robot = Fetch_Robot(sim=isSim)

    tf_helper = TF_Helper()

    # if running in simulation, we can have a object mover helping us
    objectMover = None
    if isSim:
        rospy.wait_for_service('move_object')
        objectMover = rospy.ServiceProxy('move_object', ObjectPose)

    # load json trajectory
    end_effector_trajectory, visual_end_effector_trajectory, object_grasp = loadEnd_effector_trajectory(object_name)

    # find a set of valid position on table for manipulation
    valid_manipulation_points = detect_table_and_placement(robot)

    init_ik_result = None # initial arm configuration for grasping
    current_init_grasp = None # initial grasp trasnform in the base frame
    found_solution = False
    # originalPlan = [] # original plan before refinement
    totalPlan = [] # plan to pregrasp the object
    graspingPlan = [] # plan to grasp the object

    for p in valid_manipulation_points:
        testpoint = getMatrixFromQuaternionAndTrans(p[1], p[0])
        for tableangle in [0.0, 0.5235987755982988, 1.0471975511965976, 1.5707963267948966, 2.0943951023931953, 2.617993877991494, 3.141592653589793, 3.665191429188092, 4.1887902047863905, 4.71238898038469, 5.235987755982988, 5.759586531581287]:
            rotationInZ = np.identity(4)
            rotationInZ[:3,:3] = R.from_rotvec(tableangle * np.array([0,0,1])).as_dcm()
            ## calculate the gripper pose on table
            current_testpoint = testpoint.dot(rotationInZ)
            debug_current_testpoint = testpoint.dot(rotationInZ)

            if isSimple: # test for directly lifting up
                # use ik solver to check whether the initial pose is feasible or not
                current_init_grasp = getTransformFromPoseMat(current_testpoint.dot(np.array(end_effector_trajectory[0][1][0])))
                init_ik_result = robot.solve_ik_collision_free_in_base(current_init_grasp, 10)
                if init_ik_result == None:
                    continue

                totalPlan = []
                totalPlan.append(("gripper", robot.planto_open_gripper(), 0.1))
                totalPlan.append(("arm", robot.planto_joints(init_ik_result.state.joint_state.position, init_ik_result.state.joint_state.name)))
                graspingPlan = copy.deepcopy(totalPlan)
                # originalPlan = copy.deepcopy(totalPlan)

                totalPlan.append(("gripper", robot.planto_close_gripper(), 0.0))
                moveit_robot_state = copy.deepcopy(init_ik_result.state)

                # add lift up planning
                liftup_joint_pair = {}
                for jn, jv in zip(list(moveit_robot_state.joint_state.name), list(moveit_robot_state.joint_state.position)):
                    liftup_joint_pair[jn] = jv
                liftup_trans = robot.get_fk(liftup_joint_pair, "gripper_link")
                liftup_trans[0][2] += 0.09
                liftup_plan, fraction = robot.planFollowEndEffectorTrajectory(moveit_robot_state, [liftup_trans], 0.01)
                if fraction != 1.0:
                    continue
                totalPlan.append(("arm", liftup_plan))

                found_solution = True

                break

            # check the ik feasibility of the trajectory
            ik_feasible = True
            for g in [getTransformFromPoseMat(current_testpoint.dot(np.array(t))) for t in visual_end_effector_trajectory]:
                if robot.solve_ik_collision_free_in_base(g, 10) == None:
                    ik_feasible = False
                    break
            if not ik_feasible: # some grasp is not feasible
                continue

            # use ik solver to check whether the initial pose is feasible or not
            current_init_grasp = getTransformFromPoseMat(current_testpoint.dot(np.array(end_effector_trajectory[0][1][0])))
            init_ik_result = robot.solve_ik_collision_free_in_base(current_init_grasp, 10)
            if init_ik_result == None:
                continue
            
            # check initial manipubility is low or not
            if robot.get_mom(list(init_ik_result.state.joint_state.position)) < momThreshold:
                # if initial manipubility is too low, then skip it
                continue

            # generate cartesian path from the pose waypoints
            debug_moveit_robot_state = copy.deepcopy(init_ik_result.state)
            moveit_robot_state = copy.deepcopy(init_ik_result.state)

            totalPlan = []
            totalPlan.append(("gripper", robot.planto_open_gripper(), 0.1))
            totalPlan.append(("arm", robot.planto_joints(init_ik_result.state.joint_state.position, init_ik_result.state.joint_state.name)))
            totalPlan.append(("gripper", robot.planto_close_gripper(), 0.0))
            graspingPlan = copy.deepcopy(totalPlan)
            # originalPlan = copy.deepcopy(totalPlan)

            path_feasible = True
            last_grasp_value = 0.0
            step = 0
            for a, t in end_effector_trajectory:
                if a == 'closeGripper':
                    totalPlan.append(("gripper", robot.planto_close_gripper(), 0.0))
                    last_grasp_value = 0.0
                    continue
                elif a == 'setGripper':
                    # totalPlan.append(("gripper", robot.planto_open_gripper(t/1000.0), t/1000.0))
                    # last_grasp_value = t/1000.0
                    totalPlan.append(("gripper", robot.planto_open_gripper(), 0.1))
                    last_grasp_value = 0.1
                    continue

                needToRefine = False
                refine_need_joints = None
                momOfTrajectory = []

                print("step ", step)
                step += 1
                print("length of waypoints ", len(t))
                (currentTrajectoryPlan, fraction) = robot.planFollowEndEffectorTrajectory(moveit_robot_state, [getTransformFromPoseMat(current_testpoint.dot(np.array(e))) for e in t], 0.01)
                if fraction != 1.0:
                    print("low fraction for initial cartesian planning with ", fraction)
                    path_feasible = False

                    # check the mom of incomplete planning
                    current_joints = None
                    for p in range(0, len(currentTrajectoryPlan.joint_trajectory.points)):
                        current_joints = list(currentTrajectoryPlan.joint_trajectory.points[p].positions)
                        momOfTrajectory.append(robot.get_mom(current_joints))
                    # the last point should be the one too close to the singularity
                    needToRefine = True
                    refine_need_joints = {}
                    for jn, jv in zip(list(currentTrajectoryPlan.joint_trajectory.joint_names), current_joints):
                        refine_need_joints[jn] = jv
                
                else:
                    # calculate the mom of the trajectory
                    for p in range(0, len(currentTrajectoryPlan.joint_trajectory.points)):
                        current_joints = list(currentTrajectoryPlan.joint_trajectory.points[p].positions)
                        measureOfManipulability_fast = robot.get_mom(current_joints)
                        momOfTrajectory.append(measureOfManipulability_fast)
                        if not needToRefine and measureOfManipulability_fast < momThreshold: # need to refine the trajectory
                            needToRefine = True
                            path_feasible = False
                            refine_need_joints = {}
                            for jn, jv in zip(list(currentTrajectoryPlan.joint_trajectory.joint_names), current_joints):
                                refine_need_joints[jn] = jv

                current_mom = min(momOfTrajectory)

                if needToRefine:
                    #######################################################################################################
                    print("try to refine")
                    numberOfRefineStep = 10
                    current_fraction = 0.0
                    refine_dis = 0.015
                    for update_dis in range(1, numberOfRefineStep):
                        # find where the gripper link is with refine needs joints
                        refineTrans = robot.get_fk(refine_need_joints, "gripper_link")
                        refinePose = getMatrixFromQuaternionAndTrans(refineTrans[1], refineTrans[0])

                        # # calculate mom of refine need joints
                        refine_joint = [refine_need_joints[jn] for jn in robot.group.get_active_joints()]
                        measureOfManipulability_fast = robot.get_mom(refine_joint)

                        # calculate the surface normal vector of singular point
                        Jacobian_matrix = robot.get_jacobian_matrix(refine_joint)
                        pseudoInverse_matrix = np.linalg.pinv(Jacobian_matrix)
                        DerivativeOfMoM = []
                        for j in range(len(refine_joint)):
                            temp_joint = list(refine_joint)
                            temp_joint[j] += 0.005
                            DerivativeOfMoM.append(measureOfManipulability_fast * np.trace(((robot.get_jacobian_matrix(temp_joint) - Jacobian_matrix)/ 0.005).dot(pseudoInverse_matrix)))
                        disToSin = np.array(DerivativeOfMoM).dot(pseudoInverse_matrix)
                        singularity_normal = np.transpose(disToSin) / np.linalg.norm(disToSin)

                        # calculate the new position for the refine need joint
                        pre_refine_twist_in_base = np.concatenate((singularity_normal[3:], singularity_normal[:3]))
                        pre_refine_twist_on_table = np.dot(adjointRepresentationMatrix(np.linalg.inv(testpoint)), pre_refine_twist_in_base)
                        refine_twist_on_table = np.array([0,0,pre_refine_twist_on_table[2], pre_refine_twist_on_table[3], pre_refine_twist_on_table[4], 0])
                        refine_twist_in_base = np.dot(adjointRepresentationMatrix(testpoint), refine_twist_on_table)

                        # find the new initial position for manipulation
                        refined_pose_in_base = np.dot(expm(skew_twist(refine_twist_in_base) * refine_dis), refinePose)
                        refined_initial_position = refined_pose_in_base.dot(np.linalg.inv(refinePose)).dot(current_testpoint)

                        current_init_grasp_temp = getTransformFromPoseMat(refined_initial_position.dot(np.array(t[0])))
                        init_ik_result_temp = robot.solve_ik_collision_free_in_base(current_init_grasp_temp, 5, list(moveit_robot_state.joint_state.position))
                        if init_ik_result_temp == None:
                            print("ik solve failure in refine")
                            break

                        if robot.get_mom(list(init_ik_result_temp.state.joint_state.position)) < momThreshold:
                            print("fail in low initial mom")
                            # if initial manipubility is too low, then skip it
                            break

                        # need to plan to shift the object to better position
                        (shift_plan, fraction) = robot.planFollowEndEffectorTrajectory(moveit_robot_state, [current_init_grasp_temp], 0.01)
                        if fraction != 1.0:
                            print("fail in low fraction in shifting")
                            break

                        # calculate the mom of the shift trajectory
                        shift_mom_of_trajectory = [robot.get_mom(list(shift_plan.joint_trajectory.points[p].positions)) for p in range(0, len(shift_plan.joint_trajectory.points))]
                        if min(shift_mom_of_trajectory) < momThreshold:
                            print("shifting has low mom value")
                            break

                        moveit_robot_state_temp = copy.deepcopy(moveit_robot_state)
                        moveit_robot_state_temp.joint_state.position = copy.deepcopy(shift_plan.joint_trajectory.points[-1].positions)
                        moveit_robot_state_temp.joint_state.velocity = copy.deepcopy(shift_plan.joint_trajectory.points[-1].velocities)
                        moveit_robot_state_temp.joint_state.effort = copy.deepcopy(shift_plan.joint_trajectory.points[-1].effort)


                        refined_mom_of_trajectory = []
                        (currentTrajectoryPlan_temp, fraction) = robot.planFollowEndEffectorTrajectory(moveit_robot_state_temp, [getTransformFromPoseMat(refined_initial_position.dot(np.array(e))) for e in t], 0.01)
                        if fraction != 1.0:
                            print("refined fraction = ", fraction)
                            if update_dis == numberOfRefineStep - 1 or current_fraction > fraction:
                                print("can't refine fraction anymore")
                                break
                            else:
                                current_fraction = fraction

                                # need to refine because of low fraction
                                current_joints = None
                                for p in range(0, len(currentTrajectoryPlan_temp.joint_trajectory.points)):
                                    current_joints = list(currentTrajectoryPlan_temp.joint_trajectory.points[p].positions)
                                    refined_mom_of_trajectory.append(robot.get_mom(current_joints))
                                # the last point should be the one too close to the singularity
                                refine_need_joints = {}
                                for jn, jv in zip(list(currentTrajectoryPlan_temp.joint_trajectory.joint_names), current_joints):
                                    refine_need_joints[jn] = jv
                        else:
                            # need to refine because of low mom
                            for p in range(0, len(currentTrajectoryPlan_temp.joint_trajectory.points)):
                                current_joints = list(currentTrajectoryPlan_temp.joint_trajectory.points[p].positions)
                                measureOfManipulability_fast = robot.get_mom(current_joints)
                                refined_mom_of_trajectory.append(measureOfManipulability_fast)
                                if measureOfManipulability_fast < momThreshold: # need to refine the trajectory
                                    # find the new refine joint
                                    refine_need_joints = {}
                                    for jn, jv in zip(list(currentTrajectoryPlan_temp.joint_trajectory.joint_names), current_joints):
                                        refine_need_joints[jn] = jv

                            test_mom = min(refined_mom_of_trajectory)
                            print("refine current mom = ", test_mom)

                        if fraction == 1.0 and test_mom > momThreshold: # refine successfully, so set the path feasible to true
                            path_feasible = True
                            # add shift plan
                            totalPlan.append(("gripper", robot.planto_close_gripper(), 0.0))
                            totalPlan.append(("arm", shift_plan))
                            totalPlan.append(("gripper", robot.planto_open_gripper(last_grasp_value), last_grasp_value))

                            currentTrajectoryPlan = copy.deepcopy(currentTrajectoryPlan_temp)
                            current_testpoint = copy.deepcopy(refined_initial_position)
                            break
                        
                        current_testpoint = copy.deepcopy(refined_initial_position)

                        if fraction == 1.0:
                            # if the refinement is going wrong direction, then break
                            if current_mom > test_mom:
                                if current_mom > (test_mom + 0.01):
                                    print("fail in refinement wrong way")
                                    break
                            else:
                                current_mom = test_mom

                if not path_feasible:
                    break

                totalPlan.append(("arm", currentTrajectoryPlan))

                moveit_robot_state.joint_state.position = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].positions)
                moveit_robot_state.joint_state.velocity = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].velocities)
                moveit_robot_state.joint_state.effort = copy.deepcopy(currentTrajectoryPlan.joint_trajectory.points[-1].effort)

            # for a, t in end_effector_trajectory:
            #     if a == 'closeGripper' or a == 'setGripper':
            #         continue
            #     (debug_currentTrajectoryPlan, _) = robot.planFollowEndEffectorTrajectory(debug_moveit_robot_state, [getTransformFromPoseMat(debug_current_testpoint.dot(np.array(e))) for e in t], 0.01)
            #     originalPlan.append(("arm", debug_currentTrajectoryPlan))
            #     debug_moveit_robot_state.joint_state.position = copy.deepcopy(debug_currentTrajectoryPlan.joint_trajectory.points[-1].positions)
            #     debug_moveit_robot_state.joint_state.velocity = copy.deepcopy(debug_currentTrajectoryPlan.joint_trajectory.points[-1].velocities)
            #     debug_moveit_robot_state.joint_state.effort = copy.deepcopy(debug_currentTrajectoryPlan.joint_trajectory.points[-1].effort)

            # add lift up planning
            liftup_joint_pair = {}
            for jn, jv in zip(list(moveit_robot_state.joint_state.name), list(moveit_robot_state.joint_state.position)):
                liftup_joint_pair[jn] = jv
            liftup_trans = robot.get_fk(liftup_joint_pair, "gripper_link")
            liftup_trans[0][2] += 0.1
            liftup_plan, fraction = robot.planFollowEndEffectorTrajectory(moveit_robot_state, [liftup_trans], 0.01)
            if fraction < 0.7:
                path_feasible = False

            print("path feasible", path_feasible, "---------------------------------------------------")

            if not path_feasible:
                continue

            totalPlan.append(("arm", liftup_plan))

            found_solution = True
            break
        if found_solution:
            break

    if not found_solution:
        print("there is no ik result")
    else:
        print("found solution")
        # check the whole mom of the trajectory solution
        # total_mom_trajectory = []
        # grasp_mom_trajectory = []
        # original_mom_trajectory = []

        # for point in totalPlan:
        #     if point[0] == "gripper":
        #         continue
        #     action, trajectory = point
        #     for p in trajectory.joint_trajectory.points:
        #         total_mom_trajectory.append(robot.get_mom(list(p.positions)))

        # for point in originalPlan:
        #     if point[0] == "gripper":
        #         continue
        #     action, trajectory = point
        #     for p in trajectory.joint_trajectory.points:
        #         original_mom_trajectory.append(robot.get_mom(list(p.positions)))


        # for point in graspingPlan:
        #     if point[0] == "gripper":
        #         continue
        #     action, trajectory = point
        #     for p in trajectory.joint_trajectory.points:
        #         grasp_mom_trajectory.append(robot.get_mom(list(p.positions)))

        # plt.plot(total_mom_trajectory, label="mom of solution trajectory")
        # plt.plot(original_mom_trajectory, label="mom of original trajectory")
        # plt.plot(grasp_mom_trajectory, label="mom of grasping trajectory")
        # # plt.ylim(ymin=0)
        # plt.legend()
        # plt.show()

        robot.display_trajectory([p[1] for p in totalPlan])

        raw_input("execute plan")

        for p in totalPlan[:2]:
            if p[0] == "gripper":
                robot.setGripperWidth(p[2])
            elif p[0] == "arm":
                robot.execute_plan(p[1])

        desired_object_pose_mat_in_base = getMatrixFromQuaternionAndTrans(current_init_grasp[1], current_init_grasp[0]).dot(np.linalg.inv(np.array(object_grasp)))
        desired_object_trans_in_base = getTransformFromPoseMat(desired_object_pose_mat_in_base)
        
        if isSim:
            # if it is in Sim, then we can move the object to the position where the robot can grasp
            # get the transform from world to baselink
            world_to_base_mat = tf_helper.getPoseMat("world", "base_link")
            desired_object_trans = getTransformFromPoseMat(world_to_base_mat.dot(desired_object_pose_mat_in_base))

            # move the object to where the robot can grasp
            try:
                object_pose = Pose()
                object_pose.position.x = desired_object_trans[0][0]
                object_pose.position.y = desired_object_trans[0][1]
                object_pose.position.z = desired_object_trans[0][2]

                object_pose.orientation.x = desired_object_trans[1][0]
                object_pose.orientation.y = desired_object_trans[1][1]
                object_pose.orientation.z = desired_object_trans[1][2]
                object_pose.orientation.w = desired_object_trans[1][3]

                objectMover(object_name, object_pose)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
                exit()
    
        # add the object in planning scene for visualizing the manipulation
        robot.addCollisionObject("object", desired_object_trans_in_base, "/home/lambda/catkin_ws/src/objects_description/" + object_name + "/" + object_name + ".stl", 0.001)

        if isSimple:
            # need to attach the object to the hand
            robot.attachManipulatedObject("object")

        raw_input("grasping")

        if totalPlan[2][0] == "gripper":
            robot.setGripperWidth(totalPlan[2][2])

        raw_input("execute grasp optimization")

        # execute the actions
        for p in totalPlan[3:]:
            if p[0] == "gripper":
                robot.setGripperWidth(p[2])
                if p[2] == 0.0:
                    robot.attachManipulatedObject("object")
                else:
                    robot.detachManipulatedObject("object")

            elif p[0] == "arm":
                # hope we are lucky enoughs
                for i in range(len(p[1].joint_trajectory.points) - 1):
                    if (p[1].joint_trajectory.points[i+1].time_from_start - p[1].joint_trajectory.points[i].time_from_start) == rospy.Duration(0):
                        p[1].joint_trajectory.points[i+1].time_from_start += rospy.Duration(secs=0, nsecs=10000)

                joint_error = robot.execute_plan(p[1])
                if joint_error > 1.0:
                    for i in range(len(p[1].joint_trajectory.points) - 1):
                        difftime = p[1].joint_trajectory.points[i+1].time_from_start - p[1].joint_trajectory.points[i].time_from_start
                        print(difftime, difftime==rospy.Duration(0))
                        if difftime==rospy.Duration(0):
                            print("---", p[1].joint_trajectory.points[i].time_from_start + difftime / 2)
                    # # print the error trajectory
                    print("something is wrong")
                    break

        # # calculate current pose of the object to hand for grasp optimization
        if isSim:
            def unit_vector(vector):
                """ Returns the unit vector of the vector.  """
                return vector / np.linalg.norm(vector)

            def angle_between(v1, v2):

                v1_u = unit_vector(v1)
                v2_u = unit_vector(v2)
                return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

            current_object_pose_in_hand = tf_helper.getPoseMat(object_name, "gripper_link")
            print("current object pose in hand")
            print(current_object_pose_in_hand)

            # calculate the desired pose of the object to hand
            print("desired object pose in hand")
            desired_object_pose_in_hand = np.array(object_grasp)
            print(desired_object_pose_in_hand)

            print("translation error")
            print(np.linalg.norm(current_object_pose_in_hand[:3, 3] - desired_object_pose_in_hand[:3, 3]))
            print("rotational error") # we compare only z axis of the pose
            print(angle_between(current_object_pose_in_hand[:3, 2], desired_object_pose_in_hand[:3, 2]) * 180.0 / np.pi)

        # raw_input("move to place")

        # # placingpose = np.identity(4) # bottle
        # # placingpose[0][3] = 0.149
        # # placingpose[1][3] = 0.53
        # # placingpose[2][3] = 0.67        
        # # placingpose = np.array([[0,0,1,0.14],[0,1,0,0.57],[-1,0,0,0.8],[0,0,0,1]]) # book
        # placingpose = np.array([[1,0,0,0.17],[0,1,0,0.64],[0,0,1,0.8],[0,0,0,1]]) # can
        
        # # while True:
        # #     tf_helper.pubTransform("placing trans", getTransformFromPoseMat(placingpose))
        
        # # exit()

        # placingTrans = []

        # found_solution = False
        # pre_placing_plan = None
        # placedown_plan = None

        # for placingangle in [0.0, 0.5235987755982988, 1.0471975511965976, 1.5707963267948966, 2.0943951023931953, 2.617993877991494, 3.141592653589793, 3.665191429188092, 4.1887902047863905, 4.71238898038469, 5.235987755982988, 5.759586531581287]:
        #     # for placingangle in [0, 3.141592653589793]:

        #     rotationInZ = np.identity(4)
        #     rotationInZ[:3,:3] = R.from_rotvec(placingangle * np.array([0,0,1])).as_dcm()
        #     pre_place_trans = getTransformFromPoseMat(placingpose.dot(rotationInZ).dot(np.array(object_grasp)))
        #     placing_result = robot.solve_ik_collision_free_in_base(pre_place_trans, 10)
        #     if placing_result == None:
        #         continue

        #     pre_placing_plan = robot.planto_joints_with_rotational_constraint(placing_result.state.joint_state.position, placing_result.state.joint_state.name, np.linalg.inv(np.array(object_grasp)))
        #     # pre_placing_plan = robot.planto_joints(placing_result.state.joint_state.position, placing_result.state.joint_state.name)
            
        #     if pre_placing_plan is None:
        #         continue

        #     placing_joint_pair = {}
        #     for jn, jv in zip(list(placing_result.state.joint_state.name), list(placing_result.state.joint_state.position)):
        #         placing_joint_pair[jn] = jv
        #     current_trans = robot.get_fk(placing_joint_pair, "gripper_link")

        #     placingTrajectory = []
        #     for _ in range(15):
        #         placingTrajectory.append(copy.deepcopy(current_trans))
        #         current_trans[0][2] -= 0.01
        #     placedown_plan, fraction = robot.planFollowEndEffectorTrajectory(placing_result.state, placingTrajectory, 0.01)

        #     if fraction < 0.8:
        #         continue

        #     found_solution = True
        #     raw_input("check the pre place plan first")
        #     robot.execute_plan(pre_placing_plan)
        #     break


        # if not found_solution:
        #     print("can't find solution for placing")
        #     exit()

        # # # calculate current pose of the object to hand for simple grasping
        # if isSim and isSimple:
        #     def unit_vector(vector):
        #         """ Returns the unit vector of the vector.  """
        #         return vector / np.linalg.norm(vector)

        #     def angle_between(v1, v2):

        #         v1_u = unit_vector(v1)
        #         v2_u = unit_vector(v2)
        #         return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        #     current_object_pose_in_hand = tf_helper.getPoseMat(object_name, "gripper_link")
        #     print("current object pose in hand")
        #     print(current_object_pose_in_hand)

        #     # calculate the desired pose of the object to hand
        #     print("desired object pose in hand")
        #     desired_object_pose_in_hand = np.array(object_grasp)
        #     print(desired_object_pose_in_hand)

        #     print("translation error")
        #     print(np.linalg.norm(current_object_pose_in_hand[:3, 3] - desired_object_pose_in_hand[:3, 3]))
        #     print("rotational error") # we compare only z axis of the pose
        #     print(angle_between(current_object_pose_in_hand[:3, 2], desired_object_pose_in_hand[:3, 2]) * 180.0 / np.pi)


        # robot.execute_plan(placedown_plan)

        # raw_input("open gripper")
        # robot.openGripper()
        
