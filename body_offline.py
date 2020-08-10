'''
    body.py offline version.

    Body measurements, relations of the links with each other, Kalman parameters

    p: positions of joints -> chest, shoulder, elbow, wrist, (hand)
    s: link lengths, between two joints -> chest, lua, lla, hand
    q_SB: sensor to body orientation, constant.
    q_SG: sensor to global orientation, depend on the previous joint. not updated by Kalman. Input
    q_BG: body to global orientation, the target.

'''

import numpy as np
from math import acos
from math import sqrt
import lib.Kinematics_with_Quaternions as kinematic
import lib.kalman as kf
# from IMU_subscriber_class_v2 import IMUsubscriber
from geometry_msgs.msg import Quaternion
from my_human_pkg.msg import test_msg

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from lib.util import*

# CONSTANTS
NUM = 4  # number of IMU
INDEX = 0
DT = 0.01
GRAVITY = np.array([0, 0, 9.81])

# GLOBALS


def get_filtered_data(file_name):
    global INDEX
    cur_dir = os.getcwd()
    fs = 512.0
    cutoff = 10.0
    print "cutoff_fs:", cutoff/fs
    file_path = os.path.join(cur_dir, 'data', file_name)
    acc, quat = np.array(read_data_xlsx(file_path))
    # print "quat init:", quat[0][0]
    # print "inverted quat init:", kinematic.q_invert(quat[0][0])
    # quat[0].shape[0]
    for i in range(1, quat[0].shape[0]):
        quat[0][i] = kinematic.q_multiply(quat[0][i], kinematic.q_invert(quat[0][0]))  # calibration quat
        quat[1][i] = kinematic.q_multiply(quat[1][i], kinematic.q_invert(quat[1][0]))  # calibration quat
    #  first filter(acc) or rotate(acc)? => rotate(acc)
        acc[0][i] = kinematic.q_rotate(quat[0][i],acc[0][i])
        acc[0][i] = acc[0][i] - GRAVITY
        acc[1][i] = kinematic.q_rotate(quat[1][i],acc[1][i])
        acc[1][i] = acc[1][i] - GRAVITY
        # print "acc_rotated:", acc[0][i], kinematic.v_magnitude(acc[0][i])

    median_data = median_filter(acc[0], 155)
    comb_data = freq_filter(median_data, 155, cutoff/fs)

    # plot_subplot(acc_data[0], 'raw data')
    # plot_subplot(comb_data, 'filtered data')
    # plt.show()

    # print "************", index
    INDEX += 1
    return acc, quat, comb_data


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError('No file name specified')
    acc, quat, acc_filtered = get_filtered_data(sys.argv[1])
    input_raw = kf.calculate_b_u(acc, quat)
    input_filtered = kf.calculate_b_u(acc_filtered, quat)

    print "raw:", input_filtered.shape
    print "filtered:", input_filtered.shape

    print "error_raw:", input_raw[0][-1]
    plot_subplot(input_raw[0], 'raw data')
    print "error_filtered:", input_filtered[0][-1]
    plot_subplot(input_filtered[0], 'filtered data')
    # plt.show()


    # stateMatrix = np.zeros((6, 1), np.float64)  # [p0 (3x1), p1 (3x1)]
    # estimateCovariance = np.eye(stateMatrix.shape[0])
    # transitionMatrix = np.eye(stateMatrix.shape[0], np.float32)
    # processNoiseCov = np.eye(stateMatrix.shape[0], np.float32) * 0.001
    # measurementStateMatrix = np.zeros((3, 1), np.float64)
    # observationMatrix = np.array([[1,0,0,-1,0,0],[0,1,0,0,-1,0],[0,0,1,0,0,-1]], np.float32)
    # measurementNoiseCov = np.array([[1,0,0],[0,1,0],[0,0,1]], np.float32) * 1000
    # kalman = KalmanFilter(X=stateMatrix,
    #                       P=estimateCovariance,
    #                       F=transitionMatrix,
    #                       Q=processNoiseCov,
    #                       Z=measurementStateMatrix,
    #                       H=observationMatrix,
    #                       R=measurementNoiseCov,
    #                       M=input)
    # for i in range(len)
