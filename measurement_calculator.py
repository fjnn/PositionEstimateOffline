'''
This is just for offline mode. You have to iterate it in real time
'''

import numpy as np
from math import acos
from math import sqrt
import lib.Kinematics_with_Quaternions as kinematic
import lib.kalman as kf
from lib.kalman import KalmanFilter
# from IMU_subscriber_class_v2 import IMUsubscriber
from geometry_msgs.msg import Quaternion
from my_human_pkg.msg import test_msg

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from lib.util import*

'''
I have list of links and quaternions
'''


def measured_rotation(l,q):
    '''
    Computes link_rotated = qlq*
    @params q: list of joint rotation wrt previous link body frame. quaternion represented as numpy array n*[q0, q1, q2, q3] -> [w, x, y, z]
    @params l: list of body links. Represented as numpy array. Expressed as pure quaternion. [v0, v1, v2] -> [x, y, z]
    @return: measurement list
    '''
    q_accumulated = np.zeros([len(q), len(q[0]), 4], dtype=np.float64)
    q_accumulated[0][0] = np.array([1, 0, 0, 0], dtype=np.float64)
    q_accumulated[1][0] = np.array([1, 0, 0, 0], dtype=np.float64)
    # for i in range(1, len(q[0])):
    #     # q_accumulated[0][i] = kinematic.q_norm(kinematic.q_multiply(q_accumulated[0][i-1], q[0][i]))
    #     q_accumulated[0][i] = kinematic.q_multiply(q_accumulated[0][i-1], q[0][i])
    #     # q_accumulated[1][i] = kinematic.q_norm(kinematic.q_multiply(q_accumulated[1][i-1], q[1][i]))
    #     l[0] = kinematic.q_rotate(q[0][i], l[0])
    #     # l[1] = kinematic.q_rotate(q[1][i], l[1])
    print "q0", q[0][1], "q1", q[0][2], "q01", kinematic.q_multiply(q[0][0], q[0][1])
    q_accumulated[0][1] = kinematic.q_norm(kinematic.q_multiply(q[0][0], q[0][1]))
    q_accumulated[0][2] = kinematic.q_norm(kinematic.q_multiply(q_accumulated[0][1], q[0][2]))
    q_accumulated[0][3] = kinematic.q_norm(kinematic.q_multiply(q_accumulated[0][2], q[0][3]))
    q_accumulated[0][4] = kinematic.q_norm(kinematic.q_multiply(q_accumulated[0][3], q[0][4]))
    q_accumulated[0][5] = kinematic.q_norm(kinematic.q_multiply(q_accumulated[0][4], q[0][5]))


    # print "final quaternions", q_accumulated[0][:5]
    sys.exit("done")
    rotated_links = np.array([l[0], l[0]+l[1]])

    return rotated_links
