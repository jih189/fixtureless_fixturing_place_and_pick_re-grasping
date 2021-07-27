import sys

from numpy.core.defchararray import join
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from geometry_msgs.msg import Pose, PoseStamped
import copy
from controller_manager_msgs.srv import SwitchController, ListControllers
import threading
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
import random
import actionlib
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class Fetch_Robot():
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        ## instatiate a robotCommander object.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a planningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1.0)

        self.viewboxname = "viewbox"

        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name, , wait_for_servers=60.0)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.display_place_robot_state_publisher = rospy.Publisher('/move_group/display_place_robot_state', moveit_msgs.msg.DisplayRobotState, queue_size=10)
        self.display_pick_robot_state_publisher = rospy.Publisher('/move_group/display_pick_robot_state', moveit_msgs.msg.DisplayRobotState, queue_size=10)
        self.cartesian_motion_controller_publisher = rospy.Publisher('my_cartesian_motion_controller/target_frame', PoseStamped, queue_size=5)
        # self.gripper_publisher = rospy.Publisher('gripper_controller/command', JointTrajectory, queue_size=5)
        self.joint_names = ['r_gripper_finger_joint']
        self.open_joints = [0.04]
        self.close_joints = [-0.04]
        self.client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        rospy.loginfo('Waiting for joint trajectory action')    
        self.client.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')

        planning_frame = self.group.get_planning_frame()
        print "=========== Reference frame:%s" % planning_frame

        self.eef_link = self.group.get_end_effector_link()
        print "=========== End effector: %s" % self.eef_link

        self.group_names = self.robot.get_group_names()
        print "=========== Robot Groups:", self.group_names

        self.timeout = 4.0

        ############ parameters for cartisian motion controller #######################
        self.armbasename = 'torso_lift_link'
        self.targetFrame = PoseStamped()
        self.targetFrame.header.frame_id = self.armbasename
        self.thread = None

        self.transThreshold = 0.003
        self.rotThreshold = 0.01

        # used to get current robot state
        self.tf_listener = tf.TransformListener()

        # self.traj = JointTrajectory()
        # self.traj.joint_names = ['r_gripper_finger_joint']

        self.ik_solver = IK('torso_lift_link', 'gripper_link')

        # wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity', GetStateValidity)

    def generate_seed_state(self, numOfJoints):
        result = []
        for _ in range(numOfJoints):
            result.append(random.uniform(-3.14, 3.14))
        return result

    def solve_ik_collision_free(self, pose, numOfAttempt):
        robot_joint_state = moveit_msgs.msg.DisplayRobotState()
        trans, rot = pose

        for _ in range(numOfAttempt):
            seed_state = self.generate_seed_state(self.ik_solver.number_of_joints)
            
            joint_values = self.ik_solver.get_ik(seed_state, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
            if joint_values == None:
                return None

            current_robot_state = moveit_msgs.msg.RobotState()
            current_robot_state.joint_state.header.stamp = rospy.Time.now()
            current_robot_state.joint_state.position = joint_values
            current_robot_state.joint_state.name = self.ik_solver.joint_names

            if self.is_state_valid(current_robot_state):
                robot_joint_state.state = current_robot_state
                return robot_joint_state
    
        return None

    def is_state_valid(self, checked_robot_state):
        req = GetStateValidityRequest()
        req.group_name = 'arm'
        req.robot_state = copy.deepcopy(checked_robot_state)
        req.robot_state.joint_state.header.stamp = rospy.Time.now()
        res = self.state_valid_service(req)
        return res.valid

    def solve_ik(self, pose):
        robot_joint_state = moveit_msgs.msg.DisplayRobotState()
        trans, rot = pose
        seed_state = [0.0] * self.ik_solver.number_of_joints
        
        joint_values = self.ik_solver.get_ik(seed_state, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
        if joint_values == None:
            return None
        robot_joint_state.state.joint_state.position = joint_values
        robot_joint_state.state.joint_state.header.stamp = rospy.Time.now()
        robot_joint_state.state.joint_state.name = self.ik_solver.joint_names
        
        return robot_joint_state

    def getFingerValue(self):
        if not rospy.is_shutdown():
            js = rospy.wait_for_message('joint_states', JointState)
            # get index of finger
            index = js.name.index('r_gripper_finger_joint')
            return js.position[index]
        else:
            return None

    def openGripper(self):
        # pt = JointTrajectoryPoint()
        # pt.positions = [0.04]
        # pt.time_from_start = rospy.Duration(1.0)
        # self.traj.points = [pt]

        # r = rospy.Rate(10)
        # last_value = self.getFingerValue()
        # while not rospy.is_shutdown():
        #     self.traj.header.stamp = rospy.Time.now()
        #     self.gripper_publisher.publish(self.traj)
        #     r.sleep()
        #     if self.getFingerValue() > 0.036:
        #         last_value = self.getFingerValue()
        #         break
        # return last_value

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.open_joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)
            

    def closeGripper(self):
        # pt = JointTrajectoryPoint()
        # pt.positions = [-0.02]
        # pt.time_from_start = rospy.Duration(1.0)
        # self.traj.points = [pt]

        # r = rospy.Rate(10)
        # last_value = self.getFingerValue()
        # stablenum = 0
        # while not rospy.is_shutdown():
        #     self.traj.header.stamp = rospy.Time.now()
        #     self.gripper_publisher.publish(self.traj)
        #     r.sleep()
        #     # print "error ", abs(last_value - self.getFingerValue()) 
        #     if abs(last_value - self.getFingerValue()) < 0.0001:
        #         stablenum+=1
        #     else:
        #         stablenum = 0
        #     if stablenum == 5 or abs(pt.positions[0] - self.getFingerValue()) < 0.0001:
        #         break
        #     last_value = self.getFingerValue()
        # return last_value
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.close_joints
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        self.client.send_goal_and_wait(goal)

    def setErrorThreshold(self, transThreshold, rotThreshold):
        self.transThreshold = transThreshold
        self.rotThreshold = rotThreshold

    def publishTargetFrame(self):
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            self.targetFrame.header.stamp = rospy.Time()
            self.cartesian_motion_controller_publisher.publish(self.targetFrame)
            transerror, rotationerror = self.getError()
            if(transerror < self.transThreshold and rotationerror < self.rotThreshold):
                break

    # this function returns the error between target frame and current frame
    def getError(self):

        trans1 = [self.targetFrame.pose.position.x, self.targetFrame.pose.position.y, self.targetFrame.pose.position.z]
        rot1 = [self.targetFrame.pose.orientation.x, self.targetFrame.pose.orientation.y, self.targetFrame.pose.orientation.z, self.targetFrame.pose.orientation.w]

        rot1_mat = tf.transformations.quaternion_matrix(rot1)[:3,:3]

        trans2, rot2 = self.getCurrentFrame()
        rot2_mat = tf.transformations.quaternion_matrix(rot2)[:3,:3]

        pd = R.from_dcm(np.dot(rot1_mat, rot2_mat.transpose())).as_quat()
        angle = 2 * math.atan2(np.linalg.norm(pd[:3]),pd[3])

        trans = np.linalg.norm(np.array(trans1) - np.array(trans2))

        return trans, angle

    # this function is used by cartisian motion controller
    def moveToFrame(self, targetposition, targetorientation):

        self.setTargetFrame(targetposition, targetorientation)

        transerror, rotationerror = self.getError()
        # if the error is lower than threshold, then it will not set the target frame
        if(transerror < self.transThreshold and rotationerror < self.rotThreshold):
            return True

        if(self.thread == None):
            self.thread = threading.Thread(target=self.publishTargetFrame, args=())
            self.thread.start()
        elif(not self.thread.is_alive()):
            self.thread.join()
            self.thread = threading.Thread(target=self.publishTargetFrame, args=())
            self.thread.start()
            
        
        return False

    def setTargetFrame(self, targetposition, targetorientation):
        
        self.targetFrame.pose.position.x = targetposition[0]
        self.targetFrame.pose.position.y = targetposition[1]
        self.targetFrame.pose.position.z = targetposition[2]

        self.targetFrame.pose.orientation.x = targetorientation[0]
        self.targetFrame.pose.orientation.y = targetorientation[1]
        self.targetFrame.pose.orientation.z = targetorientation[2]
        self.targetFrame.pose.orientation.w = targetorientation[3]

    def getCurrentFrame(self):
        # need to set the current end-effector pose as the target frame
        try:
            self.tf_listener.waitForTransform(self.armbasename, '/gripper_link', rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.tf_listener.lookupTransform(self.armbasename, '/gripper_link', rospy.Time())
            return trans, rot
        except:
            print "fail to detect gripper state!"
            return None, None

    def switchController(self, startController, stopController):
        rospy.wait_for_service('/controller_manager/list_controllers')
        try:
            list_controller = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
            # if the controller is alreay running, the return directly
            if(startController in [controller.name for controller in list_controller().controller if controller.state=="running"]):
                return
        except rospy.ServiceException as e:
            print "Service called: %s" % e

        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            switch_controller([startController], [stopController], 2, True, 2.0)
        except rospy.ServiceException as e:
            print "Service called: %s" % e

    def addCollisionTable(self, objectname, x, y, z, rx, ry, rz, rw, width, depth, height):
        
        table_pose = PoseStamped()
        cylinderHeight = 0.001
        table_pose.header.frame_id = self.robot.get_planning_frame()
        table_pose.pose.position.x = x
        table_pose.pose.position.y = y
        table_pose.pose.position.z = z + cylinderHeight/2.0

        table_pose.pose.orientation.x = rx
        table_pose.pose.orientation.y = ry
        table_pose.pose.orientation.z = rz
        table_pose.pose.orientation.w = rw
        
        # self.scene.add_cylinder(objectname, table_pose, height=cylinderHeight, radius=0.05)
        self.scene.add_box(objectname, table_pose, size=(width,depth,height))
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            if objectname in self.scene.get_known_object_names():
                break
            second = rospy.get_time()

    def detachManipulatedObject(self, objectname):
        # detach the object from the hand
        self.scene.remove_attached_object(self.eef_link, name=objectname)
        # gaurantee the object is detached
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([objectname])
            if len(attached_objects.keys()) == 0:
                break
            second = rospy.get_time()

    def addViewBox(self):
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = 0.37
        object_pose.pose.position.y = 0
        object_pose.pose.position.z = 0.9

        object_pose.pose.orientation.x = 0
        object_pose.pose.orientation.y = 0
        object_pose.pose.orientation.z = 0
        object_pose.pose.orientation.w = 1
        self.scene.add_box(self.viewboxname, object_pose,size=(0.2,0.1,0.32))

    def removeViewBox(self):
        self.scene.remove_world_object(self.viewboxname)

    def addCollisionObject(self, objectname, transform, filename):
        trans,rot = transform
        
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = trans[0]
        object_pose.pose.position.y = trans[1]
        object_pose.pose.position.z = trans[2]

        object_pose.pose.orientation.x = rot[0]
        object_pose.pose.orientation.y = rot[1]
        object_pose.pose.orientation.z = rot[2]
        object_pose.pose.orientation.w = rot[3]

        self.scene.add_mesh(objectname, object_pose, filename, size=(0.001,0.001,0.001))
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            if objectname in self.scene.get_known_object_names():
                break
            second = rospy.get_time()
            
    def addManipulatedObject(self, objectname, x, y, z, rx, ry, rz, rw, filename):
        touch_links = self.robot.get_link_names(group=self.group_name)
        # need to add the links which are not belong to the group
        touch_links.append("gripper_link")
        touch_links.append("l_gripper_finger_link")
        touch_links.append("r_gripper_finger_link")
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = x
        object_pose.pose.position.y = y
        object_pose.pose.position.z = z

        object_pose.pose.orientation.x = rx
        object_pose.pose.orientation.y = ry
        object_pose.pose.orientation.z = rz
        object_pose.pose.orientation.w = rw

        self.scene.add_mesh(objectname, object_pose, filename, size=(0.001,0.001,0.001))
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            if objectname in self.scene.get_known_object_names():
                break
            second = rospy.get_time()
        self.scene.attach_mesh(self.eef_link, objectname, touch_links=touch_links)
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([objectname])
            if len(attached_objects.keys()) > 0:
                break
            second = rospy.get_time()

    def goto_pose(self, x, y, z, rx, ry, rz, rw):
        pose_goal = Pose()
        pose_goal.orientation.x = rx
        pose_goal.orientation.y = ry
        pose_goal.orientation.z = rz
        pose_goal.orientation.w = rw
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        self.group.set_pose_target(pose_goal)
        
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def planto_pose(self, pose):
        trans, rotation = pose
        current_endeffector = self.group.get_end_effector_link()
        self.group.set_end_effector_link("gripper_link")
        pose_goal = Pose()
        pose_goal.orientation.x = rotation[0]
        pose_goal.orientation.y = rotation[1]
        pose_goal.orientation.z = rotation[2]
        pose_goal.orientation.w = rotation[3]
        pose_goal.position.x = trans[0]
        pose_goal.position.y = trans[1]
        pose_goal.position.z = trans[2]
        self.group.set_pose_target(pose_goal)
        
        plan = self.group.plan()
        self.group.clear_pose_targets()
        self.group.set_end_effector_link(current_endeffector)
        return plan

    def execute_plan(self, plan):
        self.group.stop()
        self.group.execute(plan)
        self.group.clear_pose_targets()

    def planto_poses(self, poses):
        current_endeffector = self.group.get_end_effector_link()
        self.group.set_end_effector_link("gripper_link")
        input = []
        for trans, rotation in poses:
            pose_goal = Pose()
            pose_goal.orientation.x = rotation[0]
            pose_goal.orientation.y = rotation[1]
            pose_goal.orientation.z = rotation[2]
            pose_goal.orientation.w = rotation[3]
            pose_goal.position.x = trans[0]
            pose_goal.position.y = trans[1]
            pose_goal.position.z = trans[2]
            input.append(pose_goal)

        self.group.set_pose_targets(input)
            
        plan = self.group.plan()
        self.group.clear_pose_targets()
        self.group.set_end_effector_link(current_endeffector)
        return plan

    def plan_cartesian_path(self, goal_motion):

        current_endeffector = self.group.get_end_effector_link()
        self.group.set_end_effector_link("gripper_link")

        waypoints = []
        wpose = self.group.get_current_pose().pose
        wpose.position.x += goal_motion[0]
        wpose.position.y += goal_motion[1]
        wpose.position.z += goal_motion[2]

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

        self.group.clear_pose_targets()
        self.group.set_end_effector_link(current_endeffector)

        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def display_place_robot_state(self, state):
        state.state.joint_state.header.stamp = rospy.Time.now()

        self.display_place_robot_state_publisher.publish(state)

    def display_pick_robot_state(self, state):
        state.state.joint_state.header.stamp = rospy.Time.now()

        self.display_pick_robot_state_publisher.publish(state)