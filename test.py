import numpy as np
from geometry_msgs.msg import Quaternion
from lib.kalman import*
import sys
import lib.Kinematics_with_Quaternions as kinematic

# b = Quaternion(0, 0, 0, 1)
# a = np.array(3*[Quaternion(0, 0, 0, 1)])
#
# print a[0]
# print a[0][0]

# acc1_data = np.array(5*[[4, 0, 0], [2, 0, 0]], dtype=np.float32)
# acc2_data = np.array(10*[[0, 2, 0]], dtype=np.float32)
# acc = [acc1_data, acc2_data]
#
# sys.exit("I am done")

# X = np.array([[0.],[0.],[0.],[0.],[0.],[0.]])
# M = np.array([0.,0.,0.,0.,0.,0.])
# F = np.eye(6, dtype=np.float32)
#
# print "X", X
# print "F", F
# print "M", M
#
# result = F.dot(X) + M.reshape(6,1)
# print result

# def rotate(q,v):
#     v_rotated = np.zeros(3)
#
#     v_rotated[0] = v[0]*(q[1]**2 + q[2]**2 - q[3]**2 - q[0]**2) + 2*v[1]*(q[2]*q[3]-q[1]*q[0]) + 2*v[2]*(q[1]*q[3] + q[2]*q[0])
#
#     v_rotated[1] = 2*v[0]*(q[1]*q[0] + q[2]*q[3]) + v[1]*(q[1]**2 - q[2]**2 + q[3]**2 - q[0]**2) + 2*v[2]*(q[3]*q[0] - q[1]*q[2])
#
#     v_rotated[2] = 2*v[0]*(q[2]*q[0] - q[1]*q[3]) + 2*v[1]*(q[1]*q[2] + q[3]*q[0]) + v[2]*(q[1]**2 - q[2]**2 - q[3]**2 + q[0]**2)
#
#     return v_rotated
#
# rotated1 = rotate(np.array([0.7071068, 0., 0., 0.7071068]),np.array([-0.1454, -0.1, 9.7963]))
# rotated = kinematic.q_rotate(np.array([0., 0.7071068, 0., 0.7071068]),np.array([10.0, 0., 0]))
# print rotated1
