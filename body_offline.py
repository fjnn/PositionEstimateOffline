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
from lib.kalman import KalmanFilter
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
    acc, quat = read_data_xlsx(file_path)
    num_of_imu = acc.shape[0]
    num_of_data = acc[0].shape[0]
    print "num_of_imu:", num_of_imu
    print "num_of_data:", num_of_data
    acc_filtered = np.empty([num_of_imu, acc[0].shape[0], acc[0].shape[1]])
    median_data = np.empty([num_of_imu, acc[0].shape[0], acc[0].shape[1]])
    quat_filtered = np.empty([num_of_imu, quat[0].shape[0], quat[0].shape[1]])
    median_quat = np.empty([num_of_imu, quat[0].shape[0], quat[0].shape[1]])

    median_data[0] = median_filter(acc[0], 155)
    acc_filtered[0] = freq_filter(median_data[0], 155, cutoff/fs)

    median_data[1] = median_filter(acc[1], 155)
    acc_filtered[1] = freq_filter(median_data[1], 155, cutoff/fs)

    median_quat[0] = median_filter(quat[0], 155)
    quat_filtered[0] = freq_filter(median_quat[0], 155, cutoff/fs)

    for i in range(1, num_of_data):
        quat[0][i] = kinematic.q_multiply(quat[0][i], kinematic.q_invert(quat[0][0]))  # calibration quat
        quat[1][i] = kinematic.q_multiply(quat[1][i], kinematic.q_invert(quat[1][0]))  # calibration quat
    for i in range(1, num_of_data):
        acc_filtered[0][i] = kinematic.q_rotate(quat[0][i-155],acc_filtered[0][i])
        acc_filtered[0][i] = acc_filtered[0][i] - GRAVITY
        acc_filtered[1][i] = kinematic.q_rotate(quat[1][i-155],acc_filtered[1][i])
        acc_filtered[1][i] = acc_filtered[1][i] - GRAVITY
    print "sample acc raw:", acc[0][2000]
    print "sample acc filtered:", acc_filtered[0][2000]

    plot_subplot(acc[0], 'raw data', hold=True)
    plot_subplot(acc_filtered[0], 'filtered data')
    print "acc data:", acc[0][4101]
    print "acc filtered data:", acc_filtered[0][4101]
    # plt.show()

    plot_subplot(quat[0], 'raw data', hold=True)
    plot_subplot(quat_filtered[0], 'filtered data')
    print "quat data:", quat[0][4101]
    print "quat filtered data:", quat_filtered[0][4101]
    plt.show()

    # print "************", index
    # INDEX += 1
    return acc, quat, acc_filtered, quat_filtered


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError('No file name specified')
    acc, quat, acc_filtered = get_filtered_data(sys.argv[1])
    # plot_subplot(acc[0], 'raw acc data', hold=True)
    # plot_subplot(acc_filtered[0], 'filtered acc data', hold=True)
    # plt.show()

    # input_raw = kf.calculate_b_u(acc, quat)
    input_filtered = kf.calculate_b_u(acc_filtered, quat)

    # print "error_raw:", input_raw[-1]
    # plot_subplot(input_raw[:,:3], 'b_u IMU0 part raw')
    # print "error_filtered:", input_filtered[-1]
    # plot_subplot(input_filtered[:,3:], 'b_u IMU1 part filtered')
    # plt.show()

    stateMatrix = np.zeros((6, 1), dtype=np.float64)  # [p0 (3x1), p1 (3x1)]
    estimateCovariance = np.eye(stateMatrix.shape[0])
    transitionMatrix = np.eye(stateMatrix.shape[0], dtype=np.float32)
    processNoiseCov = np.eye(stateMatrix.shape[0], dtype=np.float32) * 0.001
    measurementStateMatrix = np.zeros((3, 1), dtype=np.float64)
    observationMatrix = np.array([[1,0,0,-1,0,0],[0,1,0,0,-1,0],[0,0,1,0,0,-1]], dtype=np.float32)
    measurementNoiseCov = np.array([[1,0,0],[0,1,0],[0,0,1]], dtype=np.float32) * 1000
    kalman = KalmanFilter(X=stateMatrix,
                          P=estimateCovariance,
                          F=transitionMatrix,
                          Q=processNoiseCov,
                          Z=measurementStateMatrix,
                          H=observationMatrix,
                          R=measurementNoiseCov,
                          M=input_raw)
    print "input size", input_raw[0].shape
    current_prediction = np.empty([len(acc[0]), 6, 1])
    measurement = np.zeros((len(acc[0]), 3, 1))
    estimated_position = np.zeros((len(acc[0]), 3, 1))
    for i in range(len(acc[0])):
        current_prediction[i] = kalman.predict(M=input_raw[i])
        kalman.correct(measurement[i])
        estimated_position[i] = kalman.X[:3]
    estimated_position = estimated_position.reshape((len(acc[0]), 3))
    # plot_subplot(estimated_position, 'state estimate')
    # plt.show()
