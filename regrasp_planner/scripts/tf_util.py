# this is the tf util
import rospy
import tf
import numpy as np
import math
from panda3d.core import Mat4
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

def PandaPosMax_t_PosMat(panda_posmtx):
    """The panda pose matrix needs to be scaled and transposed to be a normal pose matrix form."""
    I =  np.identity(4) #TODO: make it for any size matix. 
    posmtx = I.dot(panda_posmtx) #matrix of panda in np form.  
    
    posmtx[3][0] = posmtx[3][0] / 1000
    posmtx[3][1] = posmtx[3][1] / 1000
    posmtx[3][2] = posmtx[3][2] / 1000
    
    return np.transpose(posmtx)

def PosMat_t_PandaPosMax(posmtx):
    pose = Mat4( posmtx[0][0],posmtx[1][0],posmtx[2][0],0.0, \
                    posmtx[0][1],posmtx[1][1],posmtx[2][1],0.0, \
                    posmtx[0][2],posmtx[1][2],posmtx[2][2],0.0, \
                    posmtx[0][3] * 1000,posmtx[1][3] * 1000,posmtx[2][3] * 1000,1.0)

    return pose
  

def RotateGripper(posmtx):
    """ The gripper pose given from panda has is orientated different from the real robot. 
    This funciton Rotates the gipper pose given, about the Z or Y axis to correct it.
    """
    
    rotate = np.identity(4)
    #Rotate about z 
    # rotate[0][0] = -1 
    # rotate[0][1] = 0 
    # rotate[1][0] = 0 
    # rotate[1][1] = -1
    #Rotate about Y
    rotate[0][0] = -1 
    rotate[0][2] = 0 
    rotate[2][0] = 0 
    rotate[2][2] = -1
    posmtx = np.dot(posmtx,rotate)
    return posmtx

class TF_Helper():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def pubTransform(self, transform):
        t, q = transform
        self.br.sendTransform(t, q, rospy.Time.now(), 'test', 'base_link')

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