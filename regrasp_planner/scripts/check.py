from tf_util import TF_Helper, getMatrixFromQuaternionAndTrans
from geometry_msgs.msg import Pose
import rospy
from scipy.spatial.transform import Rotation as R
import math
import numpy as np

class TransClass():
    def __init__(self, _x, _y, _z):
        self.x = _x
        self.y = _y
        self.z = _z

class RotClass():
    def __init__(self, _x, _y, _z, _w):
        self.x = _x
        self.y = _y
        self.z = _z
        self.w = _w
    

if __name__=='__main__':
    for r in [0.0, 0.7853975, 1.570795, 2.3561925, 3.14159, 3.9269875, 4.712385, 5.4977825]:
        rotationInZ = np.identity(4)
        rotationInZ[0][0] = math.cos(r)
        rotationInZ[0][1] = -math.sin(r)
        rotationInZ[1][0] = math.sin(r)
        rotationInZ[1][1] = math.cos(r)
        print rotationInZ
        rotationInZ = np.identity(4)
        rotationInZ[:3,:3] = R.from_rotvec(r * np.array([0,0,1])).as_dcm()
        print rotationInZ
