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
from measurement_calculator import measured_rotation

# CONSTANTS
NUM = 4  # number of IMU
INDEX = 0
DT = 0.01
GRAVITY = np.array([0, 0, 9.83])

# GLOBALS


def get_filtered_data(file_name):
    global INDEX
    cur_dir = os.getcwd()
    fs = 512.0
    cutoff = 10.0
    win_size = 51
    print "cutoff_fs:", cutoff/fs
    file_path = os.path.join(cur_dir, 'data', file_name)
    acc, quat, measurement = read_data_xlsx(file_path)
    num_of_imu = acc.shape[0]
    num_of_data = acc[0].shape[0]
    # print "num_of_imu:", num_of_imu
    # print "num_of_data:", num_of_data
    acc_filtered = np.empty([num_of_imu, acc[0].shape[0], acc[0].shape[1]])
    median_data = np.empty([num_of_imu, acc[0].shape[0], acc[0].shape[1]])

    median_data[0] = median_filter(acc[0], win_size)
    acc_filtered[0] = freq_filter(median_data[0], win_size, cutoff/fs)

    median_data[1] = median_filter(acc[1], win_size)
    acc_filtered[1] = freq_filter(median_data[1], win_size, cutoff/fs)

    GRAVITY = np.average(acc_filtered[0][(win_size/2 +1):(win_size+1)],axis=0)  # 26=win_size/2 + 1
    print "GRAVITY:", GRAVITY
    # print "quat:", type(quat[0][26])
    # print "acc_filtered_sample", acc_filtered[0][26:52]

    for i in range(1, num_of_data):
        quat[0][i] = kinematic.q_multiply(quat[0][i], kinematic.q_invert(quat[0][0]))  # calibration quat
        quat[1][i] = kinematic.q_multiply(quat[1][i], kinematic.q_invert(quat[1][0]))  # calibration quat
        # TODO: SLERP or filtering may be required

    link_0 = np.array([0.34, 0, 0], dtype=np.float32)
    link_1 = np.array([0.12, 0, 0], dtype=np.float32)
    body_link = np.array([link_0, link_1])
    rotated_measurement = measured_rotation(body_link,quat)
    # print rotated_measurement[0][-1]
    # print rotated_measurement[1][-1]
    measurement_diff = rotated_measurement[1]-rotated_measurement[0]
    # print "diff:", measurement_diff[-1]

    # print "sample acc raw 200:", acc[0][200]
    # print "sample acc filtered 200:", acc_filtered[0][200]
    # print "sample acc raw 1100:", acc[0][1100]
    # print "sample acc filtered 1100:", acc_filtered[0][1100]
    plot_subplot(acc[1], 'raw data', hold=True)
    plot_subplot(acc_filtered[1], 'filtered data')

    for i in range(1, num_of_data):
        acc_filtered[0][i] = kinematic.q_rotate(quat[0][i],acc_filtered[0][i])
        acc_filtered[0][i] = acc_filtered[0][i] - GRAVITY
        acc_filtered[1][i] = kinematic.q_rotate(quat[1][i],acc_filtered[1][i])
        acc_filtered[1][i] = acc_filtered[1][i] - GRAVITY

    # for i in range(920,1100):
    #     print "quat", i,":", quat[0][i]
    #     print "acc_rotated",i,":", acc_filtered[0][i]

    offset = np.average(acc_filtered[0][(win_size/2 +1):(win_size+1)], axis=0)
    for i in range(0, num_of_imu):
        for j in range(0, num_of_data):
            acc_filtered[i][j] = acc_filtered[i][j]-offset
    print "offset", offset
    # TODO: why there is such an offset?
    # print "quat 200", quat[0][200]
    # print "quat 1000", quat[0][1100]
    # print "acc_rotated 200", acc_filtered[0][200]
    # print "acc_rotated 1100", acc_filtered[0][1100]
    plot_subplot(acc_filtered[0], 'rotated data')
    # print "sample acc filtered rotated:", acc_filtered[0][26:52]

    # plot_subplot(acc_filtered[0], 'filtered data')
    # plt.show()

    # print "************", index
    # INDEX += 1
    return acc, quat, acc_filtered, measurement_diff


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError('No file name specified')
    acc, quat, acc_filtered, measurement = get_filtered_data(sys.argv[1])
    plot_subplot(acc[0], 'raw acc data', hold=True)
    # plot_subplot(acc_filtered[0], 'filtered acc0 data', hold=True)
    # plot_subplot(acc_filtered[1], 'filtered acc1 data', hold=True)

    # delta_p, input_raw = kf.calculate_b_u(acc, quat)
    pos, input_filtered = kf.calculate_b_u(acc_filtered, quat)
    print "pos", pos.shape
    plot_subplot(pos[1], 'pos_IMU1')
    plot_subplot(pos[0], 'pos_IMU0')

    # print "error_raw:", input_raw[-1]
    # plot_subplot(input_raw[:,:3], 'b_u IMU0 part raw')
    # print "error_filtered:", input_filtered[-1]
    # plot_subplot(input_filtered[:,:3], 'b_u IMU0 part filtered')
    # plt.show()

    stateMatrix = np.zeros((6, 1), dtype=np.float64)  # [p0 (3x1), p1 (3x1)]
    estimateCovariance = np.eye(stateMatrix.shape[0])
    # transitionMatrix = np.eye(stateMatrix.shape[0], dtype=np.float32)
    transitionMatrix = np.array([[1,0,0,0,0,0],[0, 1, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]], dtype=np.float32)
    processNoiseCov = np.eye(stateMatrix.shape[0], dtype=np.float32) * 0.01
    # processNoiseCov = np.array([[1000,0,0,0,0,0],[0, 1000, 0, 0, 0, 0],[0, 0, 1000, 0, 0, 0],[0, 0, 0, 0.001, 0, 0], [0, 0, 0, 0, 0.001, 0], [0, 0, 0, 0, 0, 0.001]], dtype=np.float32) * 1000
    # processNoiseCov = np.array([[0.001,0,0,0,0,0],[0, 0.001, 0, 0, 0, 0],[0, 0, 0.001, 0, 0, 0],[0, 0, 0, 1000, 0, 0], [0, 0, 0, 0, 1000, 0], [0, 0, 0, 0, 0, 1000]], dtype=np.float32) * 1000
    measurementStateMatrix = np.zeros((3, 1), dtype=np.float64)
    observationMatrix = np.array([[-1,0,0,1,0,0],[0,-1,0,0,1,0],[0,0,-1,0,0,1]], dtype=np.float32)
    # observationMatrix = np.array([[-1,0,0,1,0,0],[0,-1,0,0,1,0],[0,0,-1,0,0,1]], dtype=np.float32)
    measurementNoiseCov = np.array([[1,0,0],[0,1,0],[0,0,1]], dtype=np.float32) * 0.01
    kalman = KalmanFilter(X=stateMatrix,
                          P=estimateCovariance,
                          F=transitionMatrix,
                          Q=processNoiseCov,
                          Z=measurementStateMatrix,
                          H=observationMatrix,
                          R=measurementNoiseCov,
                          M=input_filtered)
    current_prediction = np.empty([len(acc[0]), 6, 1])
    # measurement = np.zeros((len(acc[0]), 3, 1))
    print "measurement", measurement[:10]
    estimated_position_0 = np.zeros((len(acc[0]), 3, 1))
    estimated_position_1 = np.zeros((len(acc[0]), 3, 1))
    for i in range(len(acc[0])):
        current_prediction[i] = kalman.predict(M=input_filtered[i])
        kalman.correct(measurement[i].reshape((3,1)))
        estimated_position_0[i] = kalman.X[:3]
        estimated_position_1[i] = kalman.X[3:]
    estimated_position_0 = estimated_position_0.reshape((len(acc[0]), 3))
    estimated_position_1 = estimated_position_1.reshape((len(acc[0]), 3))
    # print "estimated_position", estimated_position_0[:-10][:3]
    plot_subplot(estimated_position_0, 'state estimate_0')
    plot_subplot(estimated_position_1, 'state estimate_1')
    # print "b_u", input_filtered[:-10]
    # plot_subplot(delta_p[0], "delta_p")

    fig, ax=plt.subplots()
    index=np.arange(len(input_filtered))*0.01
    ax.plot(index, input_filtered, label="b_u")
    ax.set_xlim([0,len(input_filtered)*0.01])
    ax.set_xlabel('Time [sec]')
    ax.set_title('b_u')
    ax.legend()
    plt.show()
