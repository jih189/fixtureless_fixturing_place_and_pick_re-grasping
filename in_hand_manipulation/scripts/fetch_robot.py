import sys

from numpy.core.defchararray import join
import rospy
import moveit_commander
from std_msgs.msg import Header
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest

from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
import copy
from controller_manager_msgs.srv import SwitchController, ListControllers
import threading
from rospy.core import is_shutdown
import tf
from tf_util import transformProduct
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
import random
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal

# Fetch_Robot class is a fetch robot interface for accessing moveit and planning scene, so user can use this to control the robot
# in real world or simulation.
class Fetch_Robot():
    # when sim = True, then this class is used for simulation. If it is False, then it is used for real robot.
    def __init__(self, sim=True):

        self._sim = sim

        moveit_commander.roscpp_initialize(sys.argv)
        ## instatiate a robotCommander object.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a planningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1.0)


        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name, wait_for_servers=60.0)

        self.hand_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=60.0)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        # need to check whether the Fetch is in simulation or real world
        if self._sim == True:
            self.gripper_client = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
            self.cartesian_motion_controller_publisher = rospy.Publisher('my_cartesian_motion_controller/target_frame', PoseStamped, queue_size=5)
        else:
            self.gripper_client = actionlib.SimpleActionClient("/gripper_controller/gripper_action", GripperCommandAction)
            self.cartesian_motion_controller_publisher = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped, queue_size=10)



        self.joint_names = ['r_gripper_finger_joint']
        if self._sim == True:
            self.open_joints = [0.04]
            self.close_joints = [-0.04]
        else:
            self.open_joints = 0.4
            self.close_joints = 0.0
        

        rospy.loginfo('Waiting for gripper controller')    
        self.gripper_client.wait_for_server()
        rospy.loginfo('Found gripper controller!')

        planning_frame = self.group.get_planning_frame()
        print "=========== Reference frame:%s" % planning_frame

        self.eef_link = self.group.get_end_effector_link()
        print "=========== End effector: %s" % self.eef_link

        self.group_names = self.robot.get_group_names()
        print "=========== Robot Groups:", self.group_names

        self.collision_object_pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=10)

        self.timeout = 4.0

        ############ parameters for cartisian motion controller #######################
        self.armbasename = 'torso_lift_link'
        self.basename = 'base_link'
        self.targetFrame = PoseStamped()
        self.targetFrame.header.frame_id = self.armbasename
        self.thread = None

        self.transThreshold = 0.001
        self.rotThreshold = 0.03

        # used to get current robot state
        self.tf_listener = tf.TransformListener()

        self.ik_solver = IK('torso_lift_link', 'gripper_link')

        # wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity', GetStateValidity)

        self.lastHandState = self.hand_group.get_current_joint_values()

    def generate_seed_state(self, numOfJoints):
        result = []
        for _ in range(numOfJoints):
            result.append(random.uniform(-3.14, 3.14))
        return result

    def solve_ik_collision_free_in_base(self, transform, numOfAttempt):
        # find the transform from arm base to the base link
        self.tf_listener.waitForTransform(self.armbasename, self.basename, rospy.Time(), rospy.Duration(4.0))
        base_link_in_torso_link_transform = self.tf_listener.lookupTransform(self.armbasename, self.basename, rospy.Time())

        return self.solve_ik_collision_free(transformProduct(base_link_in_torso_link_transform, transform), numOfAttempt)

    def solve_ik_collision_free(self, transform, numOfAttempt):
        robot_joint_state = moveit_msgs.msg.DisplayRobotState()
        trans, rot = transform

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

    def solve_ik(self, transform):
        robot_joint_state = moveit_msgs.msg.DisplayRobotState()
        trans, rot = transform
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

    # set the distance between fingers in meter unit
    def setGripperWidth(self, pos):
        if self._sim == True:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = [pos - 0.04]
            point.time_from_start = rospy.Duration(1)
            goal.trajectory.points.append(point)
            self.gripper_client.send_goal_and_wait(goal)
        else:
            goal = GripperCommandGoal()
            goal.command.position = float(pos)
            goal.command.max_effort = 100
            self.gripper_client.send_goal_and_wait(goal)

    def openGripper(self):
        self.setGripperWidth(.1)
    
    def closeGripper(self):
        self.setGripperWidth(0.0)

    def setErrorThreshold(self, transThreshold, rotThreshold):
        self.transThreshold = transThreshold
        self.rotThreshold = rotThreshold

    def publishTargetFrame(self):
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            self.targetFrame.header.stamp = rospy.Time()
            
            if self._sim == True:
                self.cartesian_motion_controller_publisher.publish(self.targetFrame)
            else:
                self.cartesian_motion_controller_publisher.publish(self.getTwistError())
                
            transerror, rotationerror = self.getError()
            # print "trans error ", transerror, " rotation error ", rotationerror
            if(transerror < self.transThreshold and rotationerror < self.rotThreshold):
                break

    def getTwistError(self, frame_id='torso_link'):
        trans1 = [self.targetFrame.pose.position.x, self.targetFrame.pose.position.y, self.targetFrame.pose.position.z]
        rot1 = [self.targetFrame.pose.orientation.x, self.targetFrame.pose.orientation.y, self.targetFrame.pose.orientation.z, self.targetFrame.pose.orientation.w]
        trans2, rot2 = self.getCurrentFrame()

        tran_error = np.array(trans1) - np.array(trans2)
        rot1_mat = tf.transformations.quaternion_matrix(rot1)[:3,:3]
        rot2_mat = tf.transformations.quaternion_matrix(rot2)[:3,:3]        

        rot_error = R.from_dcm(np.dot(rot1_mat, rot2_mat.transpose())).as_euler('xyz')
        twist_cmd = TwistStamped()
        twist_cmd.header.frame_id = frame_id
        twist_cmd.twist.linear.x =  tran_error[0]
        twist_cmd.twist.linear.y =  tran_error[1]
        twist_cmd.twist.linear.z=  tran_error[2]
        twist_cmd.twist.angular.x =  rot_error[0]
        twist_cmd.twist.angular.y =  rot_error[1]
        twist_cmd.twist.angular.z =  rot_error[2]
        return twist_cmd

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
    def moveToFrame(self, transform, isInBaselink=True):

        if isInBaselink: # if is move in base link, then need to convert it to arm base first
            self.tf_listener.waitForTransform(self.armbasename, self.basename, rospy.Time(), rospy.Duration(4.0))
            base_link_in_torso_link_transform = self.tf_listener.lookupTransform(self.armbasename, self.basename, rospy.Time())
            transform = transformProduct(base_link_in_torso_link_transform, transform)

        targetposition, targetorientation = transform 
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

    def getCurrentHandFrameInBase(self):
        # need to set the current end-effector pose as the target frame
        try:
            self.tf_listener.waitForTransform(self.basename, '/gripper_link', rospy.Time(), rospy.Duration(4.0))
            (trans,rot) = self.tf_listener.lookupTransform(self.basename, '/gripper_link', rospy.Time())
            return trans, rot
        except:
            print "fail to detect gripper state!"
            return None, None      

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
        if not self._sim:
            # if the robot is in real world, then there is no need to switch controller
            return
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

    def attachTable(self, tablename):
        touch_links = []
        # need to add the links which are not belong to the group
        touch_links.append("base_link")
        touch_links.append("bellows_link")
        touch_links.append("bellows_link2")
        touch_links.append("shoulder_pan_link")
        touch_links.append("torso_fixed_link")
        touch_links.append("torso_lift_link")
        self.scene.attach_mesh(self.basename, tablename, touch_links=touch_links)
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([tablename])
            if len(attached_objects.keys()) > 0:
                break
            second = rospy.get_time()


    def attachManipulatedObject(self, objectname):

        touch_links = self.robot.get_link_names(group=self.group_name)
        # need to add the links which are not belong to the group
        touch_links.append("gripper_link")
        touch_links.append("l_gripper_finger_link")
        touch_links.append("r_gripper_finger_link")
        self.scene.attach_mesh(self.eef_link, objectname, touch_links=touch_links)
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([objectname])
            if len(attached_objects.keys()) > 0:
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

    def addCollisionObject(self, objectname, transfrom_Q, filename, size_scale = .001):

        t,r = transfrom_Q

        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.get_planning_frame()
        object_pose.pose.position.x = t[0]
        object_pose.pose.position.y = t[1]
        object_pose.pose.position.z = t[2]

        object_pose.pose.orientation.x = r[0]
        object_pose.pose.orientation.y = r[1]
        object_pose.pose.orientation.z = r[2]
        object_pose.pose.orientation.w = r[3]

        self.scene.add_mesh(objectname, object_pose, filename, size=(size_scale,size_scale,size_scale))
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            if objectname in self.scene.get_known_object_names():
                break
            second = rospy.get_time()

    def removeCollisionObject(self, objectname):
        self.scene.remove_world_object(objectname)
        start = rospy.get_time()
        second = rospy.get_time()
        while (second - start) < self.timeout and not rospy.is_shutdown():
            if not objectname in self.scene.get_known_object_names():
                break
            second = rospy.get_time()

    # this function will reattach the object in hand by given new object in hand pose
    def reattachManipulatedObject(self, objectname, transformInHand):
        self.detachManipulatedObject(objectname)

        collision_object_msgs = moveit_msgs.msg.CollisionObject()
        collision_object_msgs.operation = collision_object_msgs.MOVE

        collision_object_msgs.id = objectname
        inhandpose = Pose()
        inhandpose.orientation.x = transformInHand[1][0]
        inhandpose.orientation.y = transformInHand[1][1]
        inhandpose.orientation.z = transformInHand[1][2]
        inhandpose.orientation.w = transformInHand[1][3]
        inhandpose.position.x = transformInHand[0][0]
        inhandpose.position.y = transformInHand[0][1]
        inhandpose.position.z = transformInHand[0][2]
        collision_object_msgs.mesh_poses = [inhandpose]
        collision_object_msgs.header.stamp = rospy.Time.now()
        collision_object_msgs.header.frame_id = "gripper_link"

        self.collision_object_pub.publish(collision_object_msgs)
        self.attachManipulatedObject(objectname)

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

    def planto_pose_with_constraints(self, pose):
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

        start_pose = self.group.get_current_pose()

        path_contraints = moveit_msgs.msg.Constraints()
        path_contraints.name = "keep gripper horizontal"
        orientation_c = moveit_msgs.msg.OrientationConstraint()
        orientation_c.header = start_pose.header
        orientation_c.link_name = self.group.get_end_effector_link()
        orientation_c.orientation.w = 1.0
        orientation_c.absolute_x_axis_tolerance = 3.14
        orientation_c.absolute_y_axis_tolerance = 3.14
        orientation_c.absolute_z_axis_tolerance = 2.0
        orientation_c.weight = 0.2

        path_contraints.orientation_constraints.append(orientation_c)
        self.group.set_path_constraints(path_contraints)
        original_planning_time = self.group.get_planning_time()
        # print "original planning time ", original_planning_time
        self.group.set_planning_time(10.0)
        
        plan = self.group.plan()
        self.group.set_planning_time(original_planning_time)

        self.group.clear_pose_targets()
        self.group.clear_path_constraints()
        self.group.set_end_effector_link(current_endeffector)
        
        return plan

    def verifyEndEffectorTrajectory(self, initial_robot_state, trajectory):
        current_endeffector = self.group.get_end_effector_link()
        self.group.set_end_effector_link("gripper_link")
        waypoints = []
        for t in trajectory:
            wpose = Pose()
            wpose.orientation.x = t[1][0]
            wpose.orientation.y = t[1][1]
            wpose.orientation.z = t[1][2]
            wpose.orientation.w = t[1][3]

            wpose.position.x = t[0][0]
            wpose.position.y = t[0][1]
            wpose.position.z = t[0][2]
            waypoints.append(wpose)
        
        # set start state
        self.group.set_start_state(initial_robot_state)
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.001, 0)
        # reset the start state
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()
        self.group.set_end_effector_link(current_endeffector)

        return (plan, fraction)

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

        original_planning_time = self.group.get_planning_time()
        self.group.set_planning_time(20.0)
        
        plan = self.group.plan()
        self.group.set_planning_time(original_planning_time)
        self.group.clear_pose_targets()
        self.group.set_end_effector_link(current_endeffector)
        return plan

    def planto_open_gripper(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['l_gripper_finger_joint', 'r_gripper_finger_joint']
        joint_state.position = self.lastHandState
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.hand_group.set_start_state(moveit_robot_state)
        self.hand_group.set_joint_value_target([0.04, 0.04])
        self.lastHandState = [0.04, 0.04]
        plan = self.hand_group.plan()
        self.hand_group.clear_pose_targets()
        return plan

    def planto_close_gripper(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['l_gripper_finger_joint', 'r_gripper_finger_joint']
        joint_state.position = self.lastHandState
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        self.hand_group.set_start_state(moveit_robot_state)
        self.hand_group.set_joint_value_target([0.0, 0.0])
        self.lastHandState = [0.0, 0.0]
        plan = self.hand_group.plan()
        self.hand_group.clear_pose_targets()
        return plan

    def planto_joints(self, joints, names=None):
        if names == None:
            self.group.set_joint_value_target(joints)
        else:
            self.group.set_joint_value_target(dict(zip(names, joints)))
        plan = self.group.plan()
        self.group.clear_pose_targets()
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

    def get_current_state(self):
        return self.robot.get_current_state()
    
    def display_trajectory(self, plans, start_state = None):

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        if start_state == None:
            display_trajectory.trajectory_start = self.robot.get_current_state()
        else:
            display_trajectory.trajectory_start = start_state
        for p in plans:
            display_trajectory.trajectory.append(p)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)
