#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from  tf_util import TF_Helper, transformProduct

if __name__=='__main__':
    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=False)
    currentPose = robot.getCurrentFrame()
    print "current pose "
    print currentPose
    targetPose = transformProduct(currentPose, [[0.05,0,0],[0,0,0,1]])
    robot.moveToFrame(targetPose)