from fetch_robot import Fetch_Robot
import rospy

if __name__=='__main__':
  rospy.init_node('open_gripper_node')
  robot = Fetch_Robot(sim=False)
  robot.openGripper()
  # robot.setGripperWidth(0.08)