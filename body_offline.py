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
GRAVITY = np.array([0, -9.81, 0])

# GLOBALS
body = np.empty([NUM, 3])
q_bs_init = []
q_bs = []  # each sensors' respective orientations on their body links
q_gb = []  # each body in global frame
quat_raw = []  # This is the direct IMU reading. Needs to be multiplied by q_init = Rz(-90)&Rx(-90)
q_init_chest = Quaternion(0, 0, -0.7071068, 0.7071068)
q_init_others = Quaternion(-0.7071068, 0, 0, 0.7071068)
# q_init = Quaternion(-0.5, 0.5, -0.5, 0.5)  # from global to sensor frame the rotation Rz(-90)*Rx(-90) initially - burada da bir imu.q_body_init eşlemesi yapmak gerekecek gibi ama bakalım


def get_data(file_name):
    global INDEX
    cur_dir = os.getcwd()
    fs = 512.0
    cutoff = 10.0
    print "cutoff_fs:", cutoff_fs
    file_path = os.path.join(cur_dir, 'data', file_name)
    acc, quat = np.array(read_data_xlsx(file_path))
    median_data = median_filter(acc[0], 155)
    comb_data = freq_filter(median_data, 155, cutoff/fs)
    for i in range(0, NUM):
        quat[i] = kinematic.q_multiply(quat[0], quat[i])  # calibration
        quat[i] = kinematic.q_multiply(q_init_others, quat[i])  # initialization
    # plot_subplot(acc_data[0], 'raw data')
    # plot_subplot(comb_data, 'filtered data')
    # plt.show()

    # print "************", index
    INDEX += 1
    return acc, quat


def init_body_measurements(q_gs_init):
    '''
    All initial measurements here. All in meters.
    @param q_gs: calibrated initial quaternion readings.

    @returns q_gb_init and body_global_init
    '''
    global body

    hand = 0.06
    lower_arm = 0.3
    upper_arm = 0.3
    half_chest = 0.25

    # All defined in body frame. Joint positions wrt previous body frame
    # body[0] = 0.0 * np.array([1.0, 0.0, 0.0])  # global origin ile body origin arasındaki mesafe
    body[0] = half_chest * np.array([1.0, 0.0, 1.0])
    body[1] = upper_arm * np.array([1.0, 0.0, 0.0])
    body[2] = lower_arm * np.array([1.0, 0.0, 0.0])
    body[3] = hand * np.array([1.0, 0.0, 0.0])

    body_global_init = np.empty([NUM+1, 3])  # joint positions wrt global frame
    body_global_init[0] = np.array([0.0, 0.0, 0.0])

    q_bs = np.array(NUM*[Quaternion(0, 0, 0, 1)])  # for now all sensors are aligned with the their respective body limbs.
    q_gb_init = np.empty([NUM, 4])

    for i in range(0, NUM):
        q_gb_init[i] = kinematic.q_multiply(q_gs_init[i], kinematic.q_invert(q_bs[i]))
        body_global_init[i] = kinematic.segment_kinematics(body_global_init[i-1], q_gb[i], body[i])

    return q_gb_init, body_global_init


def init_matrices():
    '''
    R and Q(maybe) changes for all body parts
    y vector for each body parts
    '''
    global A, C, state_estimate, R, Q, P, body, body_global, y_measurement

    state_estimate = np.zeros((NUM, 6), np.float64)  # [pos_prev, pos_curr]
    for i in range(0, NUM+1):
        state_estimate[i] = np.concatenate((body_global[i+1], body_global[i]))
    A = np.eye(state_estimate.shape[1], dtype=np.float32)  # 6x6
    C = np.concatenate((np.eye(3), -np.eye(3)), axis=1)
    P = np.eye(state_estimate.shape[1], dtype=np.float64)
    processCov = 10
    measurCov = 0.001
    Q = np.eye(state_estimate.shape[1], dtype=np.float64) * processCov
    R = [np.zeros([3, 3]) for _ in range(0, NUM+1)]
    R[1] = np.eye(3) * np.array([100.0, 100.0, 10.0]) * measurCov
    R[2] = np.eye(3) * np.array([50.0, 50.0, 50.0]) * measurCov
    R[3] = np.eye(3) * np.array([50.0, 10.0, 50.0]) * measurCov
    R[4] = np.eye(3) * np.array([50.0, 50.0, 50.0]) * measurCov

    # y_measurement = [body[_] for _ in range(0, NUM+1)]
    y_measurement = np.zeros((NUM+1, 3), np.float64)
    for i in range(0, NUM+1):
        y_measurement[i] = kinematic.q_rotate(q_gs[i], body[i])
    print "y:", y_measurement


def calculate_b_u(acc, quat):
    '''
    B*u matrix result is calculated here and returned 6*1 matrix
    B*u = delta(position) in global frame
    '''
    q_gs = quat  # so noob. fix it later
    delta_p = [np.zeros(3) for _ in range(0, NUM)]
    acc_global = [np.zeros(3) for _ in range(0, NUM)]
    b_u = [np.zeros(6) for _ in range(0, NUM)]

    for body_link in range(0, NUM):
        acc_global[body_link] = kinematic.q_rotate(q_gs[body_link], acc[body_link]) - GRAVITY
        # delta_p[body_link] = kinematic.double_integrate_acc(acc_global[body_link], dt=dt)
        delta_p[body_link] = 0.5*acc_global[body_link]*DT*DT
        # b_u[body_link][0:3] = [0, 0, 0]
        # b_u[body_link][3:6] = delta_p[body_link] + kinematic.q_rotate(q_gs[body_link], body[body_link])
        b_u[body_link][0:3] = delta_p[body_link]
        b_u[body_link][3:6] = b_u[body_link][0:3]
    return b_u


def update_pose(file_path):
    acc, quat = get_data(file_path)
    b_u = np.empty([])
    b_u = calculate_b_u(acc, quat)  # Maybe EKF olacak
    # b_u = [np.zeros(6) for _ in range(0, NUM+1)]
    # print "b_u: ", b_u
    # for i in range(0, NUM):
    #     y_measurement[i+1] = kinematic.q_rotate(q_gs[i], body[i+1])
    y_measurement = np.array([[0, 0, 0],
                              [0.25000001, 0.25000001, 0],
                              [0.30000002, 0, 0],
                              [0.30000002, 0, 0],
                              [0.06, 0, 0]])

    for body_link in range(0, NUM+1):
        state_estimate[body_link] = kf.kalman_predict(A, state_estimate[body_link], P, Q, b_u[body_link])
        state_estimate[body_link] = kf.kalman_update(state_estimate[body_link], P, C, R[body_link], y_measurement[body_link])

        # make sure that it is calculated after calibration
        body_global[body_link] = state_estimate[body_link][:3]

    print "******************"
    # print "state_estimate: ", state_estimate
    return body_global


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError('No file name specified')
    acc, quat = get_data(sys.argv[1])
    init_body_measurements()
    init_matrices()
    get_data(sys.argv[1])
