# this is the tf util
import rospy
import tf
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

def getErrorBetweenTransforms(p1, p2):
    trans1, rot1 = p1
    rot1_mat = tf.transformations.quaternion_matrix(rot1)[:3,:3]

    trans2, rot2 = p2
    rot2_mat = tf.transformations.quaternion_matrix(rot2)[:3,:3]

    pd = R.from_dcm(np.dot(rot1_mat, rot2_mat.transpose())).as_quat()
    angle = 2 * math.atan2(np.linalg.norm(pd[:3]),pd[3])

    trans = np.linalg.norm(np.array(trans1) - np.array(trans2))
    return trans, angle

def findCloseTransform(trans, translist):

    resulttrans = None
    resultrot = None
    resulterror = 1000.0

    for t in translist:
        _, roterror = getErrorBetweenTransforms(trans, t)
        if roterror < resulterror:
            resulttrans = t[0]
            resultrot = t[1]
            resulterror = roterror

    return resulttrans, resultrot

def transformProduct(t1, t2):
   trans1, rot1= t1
   trans1_mat = tf.transformations.translation_matrix(trans1)
   rot1_mat = tf.transformations.quaternion_matrix(rot1)
   mat1 = np.dot(trans1_mat, rot1_mat)

   trans2, rot2 = t2
   trans2_mat = tf.transformations.translation_matrix(trans2)
   rot2_mat = tf.transformations.quaternion_matrix(rot2)
   mat2 = np.dot(trans2_mat, rot2_mat)

   mat3 = np.dot(mat1, mat2)
   trans3 = tf.transformations.translation_from_matrix(mat3)
   rot3 = tf.transformations.quaternion_from_matrix(mat3)

   return trans3, rot3

def getMatrixFromQuaternionAndTrans(quaternion_array, trans_array):
    quaternion_array = (quaternion_array.x,quaternion_array.y,quaternion_array.z,quaternion_array.w)
    poseMatrix = np.matrix(tf.transformations.quaternion_matrix(quaternion_array))
    poseMatrix[0,3] = trans_array.x
    poseMatrix[1,3] = trans_array.y
    poseMatrix[2,3] = trans_array.z
    return poseMatrix

def getTransformFromPoseMat(pose_mat):
    rotation = tf.transformations.quaternion_from_matrix(pose_mat) # this function takes 4*4 matrix
    translation = [pose_mat[0,3], pose_mat[1,3], pose_mat[2,3]]
    return (translation, rotation)

class TF_Helper():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def getTransform(self, parent_link, child_link):
        self.listener.waitForTransform(parent_link, child_link, rospy.Time(), rospy.Duration(10.0))
        trans, rot = self.listener.lookupTransform(parent_link, child_link, rospy.Time())
        return trans, rot

    def getPose(self, parent_link, child_link, pose):
        trans, rot = self.getTransform(parent_link, child_link)
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]

        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

    def getPoseMat(self, parent_link, child_link):
        trans, rot = self.getTransform(parent_link, child_link)
        trans_mat = tf.transformations.translation_matrix(trans)
        rot_mat = tf.transformations.quaternion_matrix(rot)
        return np.dot(trans_mat, rot_mat)