import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import copy
from controller_manager_msgs.srv import SwitchController, ListControllers
import threading
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class Fetch_Robot():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        ## instatiate a robotCommander object.
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a planningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1.0)

        self.group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.cartesian_motion_controller_publisher = rospy.Publisher('/my_cartesian_motion_controller/target_frame', PoseStamped, queue_size=5)

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
        self.thread = threading.Thread(target=self.publishTargetFrame, args=())

        self.transThreshold = 0.003
        self.rotThreshold = 0.01

        # used to get current robot state
        self.tf_listener = tf.TransformListener()

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

        transerror, rotationerror = self.getError()
        # if the error is lower than threshold, then it will not set the target frame
        if(transerror < self.transThreshold and rotationerror < self.rotThreshold):
            return True

        self.setTargetFrame(targetposition, targetorientation)
        
        if(not self.thread.is_alive()):
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

    def planto_pose(self, trans, rotation):
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