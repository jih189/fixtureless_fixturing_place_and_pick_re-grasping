#!/usr/bin/env python
# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import numpy as np
import rospy
from std_msgs.msg import Float64
from fetch_coppeliasim.srv import ObjectPose, ObjectPoseResponse
import tf

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

class SimObjectMover(object):
    def __init__(self, clientId_):
        self.clientId = clientId_
        s = rospy.Service('move_object', ObjectPose, self.MoveObjectHandler)
        rospy.loginfo("object mover in sim is ready!!")
        rospy.spin()

    def getMatrixFromQuaternionAndTrans(self, trans_array, quaternion_array):
        poseMatrix = np.array(tf.transformations.quaternion_matrix(quaternion_array))
        poseMatrix[0][3] = trans_array[0]
        poseMatrix[1][3] = trans_array[1]
        poseMatrix[2][3] = trans_array[2]
        return poseMatrix

    def getTransformFromPoseMat(self, pose_mat):
        rotation = tf.transformations.quaternion_from_matrix(pose_mat) # this function takes 4*4 matrix
        translation = [pose_mat[0,3], pose_mat[1,3], pose_mat[2,3]]
        return (translation, rotation)

    def MoveObjectHandler(self, req):
        self.moveObjectTo(req.object_name, [req.object_pose.position.x, req.object_pose.position.y, req.object_pose.position.z], 
                        [req.object_pose.orientation.x, req.object_pose.orientation.y, req.object_pose.orientation.z, req.object_pose.orientation.w])
        return ObjectPoseResponse()

    def moveObjectTo(self, objectname, targetposition, targetquaternion):
        ret, objecthandle = sim.simxGetObjectHandle(self.clientId, objectname + "_respondable", sim.simx_opmode_oneshot_wait)
        
        if ret == sim.simx_return_ok:
            # get the pose of pose publisher
            pose_ret, posehandle = sim.simxGetObjectHandle(self.clientId, objectname + "_pose", sim.simx_opmode_oneshot_wait)
            if pose_ret != sim.simx_return_ok:
                rospy.logerr("the object does not have pose publisher in sim!!!")
                return

            _, pub_position = sim.simxGetObjectPosition(self.clientId, posehandle, -1, sim.simx_opmode_blocking)
            _, pub_quaternion = sim.simxGetObjectQuaternion(self.clientId, posehandle, -1, sim.simx_opmode_blocking)
            current_pub_pose = self.getMatrixFromQuaternionAndTrans(pub_position, pub_quaternion)
                
            _, position = sim.simxGetObjectPosition(self.clientId, objecthandle, -1, sim.simx_opmode_blocking)
            _, quaternion = sim.simxGetObjectQuaternion(self.clientId, objecthandle, -1, sim.simx_opmode_blocking)
            current_obj_pose = self.getMatrixFromQuaternionAndTrans(position, quaternion)
            from_pub_to_obj = np.linalg.inv(current_pub_pose).dot(current_obj_pose)
            pub_target_pose = self.getMatrixFromQuaternionAndTrans(targetposition, targetquaternion)
            (targetposition, targetquaternion) = self.getTransformFromPoseMat(pub_target_pose.dot(from_pub_to_obj))

            while not np.isclose(targetposition, position).all():
                sim.simxSetObjectPosition(self.clientId, objecthandle, -1, targetposition, sim.simx_opmode_oneshot)
                sim.simxSetObjectQuaternion(self.clientId, objecthandle, -1, targetquaternion, sim.simx_opmode_oneshot)
                _, position = sim.simxGetObjectPosition(self.clientId, objecthandle, -1, sim.simx_opmode_blocking)
                _, quaternion = sim.simxGetObjectQuaternion(self.clientId, objecthandle, -1, sim.simx_opmode_blocking)
            
        else:
            rospy.logerr("no such object in the sim!!")


if __name__ == "__main__":
    rospy.init_node('sim_object_server')

    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        server = SimObjectMover(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')