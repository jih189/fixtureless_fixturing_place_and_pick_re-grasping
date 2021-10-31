# this is the tf util
import rospy
import tf
import numpy as np
import math
from panda3d.core import Mat4
from scipy.spatial.transform import Rotation as R

def unit_vector(data, axis=None, out=None):
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data*data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data
 

def rotation_matrix(angle, direction, point=None):
    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array([[ 0.0,         -direction[2],  direction[1]],
                      [ direction[2], 0.0,          -direction[0]],
                      [-direction[1], direction[0],  0.0]])
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float64, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M


def unitize(points, check_valid=False):
    points = np.asanyarray(points)
    axis   = len(points.shape) - 1
    length = np.sum(points ** 2, axis=axis) ** .5
    if check_valid:
        valid = np.greater(length, 1e-12)
        if axis == 1:
            unit_vectors = (points[valid].T / length[valid]).T
        elif len(points.shape) == 1 and valid:
            unit_vectors = points / length
        else:
            unit_vectors = np.array([])
        return unit_vectors, valid
    else:
        unit_vectors = (points.T / length).T
    return unit_vectors

def align_vectors(vector_start, vector_end, return_angle=False):

    # the following code is added by weiwei on 07212017
    # to correct the problems of same vectors and inverse vectors
    if np.array_equal(vector_start, vector_end):
        T = np.eye(4)
        angle = 0.0
        if return_angle:
            return T, angle
        return T
    if np.array_equal(-vector_start, vector_end):
        T = np.eye(4)
        T[:3, 2] *= -1.0
        T[:3, 1] *= -1.0
        angle = np.pi
        if return_angle:
            return T, angle
        return T

    vector_start = unitize(vector_start)
    vector_end   = unitize(vector_end)
    cross        = np.cross(vector_start, vector_end)
    # we clip the norm to 1, as otherwise floating point bs
    # can cause the arcsin to error
    norm         = np.clip(np.linalg.norm(cross), -1.0, 1.0)
    direction    = np.sign(np.dot(vector_start, vector_end))

    if norm < 1e-12:
        # if the norm is zero, the vectors are the same
        # and no rotation is needed
        T       = np.eye(4)
        T[0:3] *= direction
    else:
        angle = np.arcsin(norm)
        if direction < 0:
            angle = np.pi - angle
        T = rotation_matrix(angle, cross)

    check = np.dot(T[:3,:3], vector_start) - vector_end
    if not np.allclose(check, 0.0, atol=1e-4):
        raise ValueError('Vectors unaligned!')

    if return_angle:
        return T, angle
    return T

def pointDown(pose):
    result = np.eye(4)
    result[:3,2] = pose[:3,2]
    result[2,2] = 0.0
    result[:3,2] = result[:3,2] / np.linalg.norm(result[:3,2])
    result[:3,0] = np.array([0,0,-1])
    result[:3,1] = -np.cross(result[:3,0], result[:3,2])
    return result



def isRotationMatrix(M):
    # check if the matrix is rotation matrix or not
    tag = False
    I = np.identity(M.shape[0])
    if np.all((np.matmul(M, M.T)) == I) and (np.linalg.det(M)==1): tag = True
    return tag 

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
    # if type(quaternion_array).__module__ == np.__name__:
    poseMatrix = np.array(tf.transformations.quaternion_matrix(quaternion_array))
    poseMatrix[0][3] = trans_array[0]
    poseMatrix[1][3] = trans_array[1]
    poseMatrix[2][3] = trans_array[2]
    return poseMatrix
    # else:
    #     quaternion_array = (quaternion_array.x,quaternion_array.y,quaternion_array.z,quaternion_array.w)
    #     poseMatrix = np.matrix(tf.transformations.quaternion_matrix(quaternion_array))
    #     poseMatrix[0,3] = trans_array.x
    #     poseMatrix[1,3] = trans_array.y
    #     poseMatrix[2,3] = trans_array.z
    #     return poseMatrix

def getTransformFromPoseMat(pose_mat):
    rotation = tf.transformations.quaternion_from_matrix(pose_mat) # this function takes 4*4 matrix
    translation = [pose_mat[0,3], pose_mat[1,3], pose_mat[2,3]]
    return (translation, rotation)

def PandaPosMax_t_PosMat(panda_posmtx):
    """The panda pose matrix needs to be scaled and transposed to be a normal pose matrix form."""

    mat = np.array([[panda_posmtx[0][0],panda_posmtx[1][0],panda_posmtx[2][0],panda_posmtx[3][0]/1000.0], \
                     [panda_posmtx[0][1],panda_posmtx[1][1],panda_posmtx[2][1],panda_posmtx[3][1]/1000.0], \
                     [panda_posmtx[0][2],panda_posmtx[1][2],panda_posmtx[2][2],panda_posmtx[3][2]/1000.0], \
                     [0.0,0.0,0.0,1.0]])

    # if not isRotationMatrix(mat[:3,:3]):
    #     raise Exception("The rotation part is not a rotation matrix!!")

    return mat

def PosMat_t_PandaPosMax(posmtx):
    # if not isRotationMatrix(posmtx[:3,:3]):
    #     raise Exception("The rotation part is not a rotation matrix!!")
    # convert pose mat to panda pose mat
    # pose = Mat4( posmtx[0,0],posmtx[1,0],posmtx[2,0],0.0, \
    #              posmtx[0,1],posmtx[1,1],posmtx[2,1],0.0, \
    #              posmtx[0,2],posmtx[1,2],posmtx[2,2],0.0, \
    #              posmtx[0,3] * 1000.0,posmtx[1,3] * 1000.0,posmtx[2,3] * 1000.0,1.0)
    pose = Mat4( posmtx[0][0],posmtx[1][0],posmtx[2][0],0.0, \
                 posmtx[0][1],posmtx[1][1],posmtx[2][1],0.0, \
                 posmtx[0][2],posmtx[1][2],posmtx[2][2],0.0, \
                 posmtx[0][3] * 1000.0,posmtx[1][3] * 1000.0,posmtx[2][3] * 1000.0,1.0)

    return pose
  

def RotateGripper(posmtx):
    """ The gripper pose given from panda has is orientated different from the real robot. 
    This funciton Rotates the gipper pose given, about the Y axis to correct it.
    """
    
    rotate = np.identity(4)
    #Rotate about Y
    rotate[0][0] = -1 
    rotate[0][2] = 0 
    rotate[2][0] = 0 
    rotate[2][2] = -1
    posmtx = np.dot(posmtx,rotate)
    return posmtx

class TF_Helper():
    # this class is used to create both listener and broadcaster for the tf.
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def pubTransform(self, name, transform):
        t, q = transform
        self.br.sendTransform(t, q, rospy.Time.now(), name, 'base_link')

    def getTransform(self, parent_link, child_link):
        now = rospy.Time.now()
        self.listener.waitForTransform(parent_link, child_link, now, rospy.Duration(10.0))
        trans, rot = self.listener.lookupTransform(parent_link, child_link, now)
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