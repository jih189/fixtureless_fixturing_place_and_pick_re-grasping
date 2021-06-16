import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

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
   
        planning_frame = self.group.get_planning_frame()
        print "=========== Reference frame:%s" % planning_frame

        self.eef_link = self.group.get_end_effector_link()
        print "=========== End effector: %s" % self.eef_link

        self.group_names = self.robot.get_group_names()
        print "=========== Robot Groups:", self.group_names

        self.timeout = 4.0

    def addCollisionObject(self, objectname, x, y, z):
        
        table_pose = geometry_msgs.msg.PoseStamped()
        cylinderHeight = 0.001
        table_pose.header.frame_id = self.robot.get_planning_frame()
        table_pose.pose.position.x = x
        table_pose.pose.position.y = y
        table_pose.pose.position.z = z + cylinderHeight/2.0

        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        
        self.scene.add_cylinder(objectname, table_pose, height=cylinderHeight, radius=0.05)
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
        object_pose = geometry_msgs.msg.PoseStamped()
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
        pose_goal = geometry_msgs.msg.Pose()
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