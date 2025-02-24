#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
In this class different kinematic and dynamic equalities are calculated

"""

# TODO: do the orientation calculations by KF later by yourself using gyro and acc measurements. Now use the orientation data that Xsens provides
# TODO: in q_invert(), check if the input is a unit quaternion

# imports
# import Data.data_logger_module as data_logger
import calculate_time
import numpy as np
from math import sqrt
from math import sin
from math import cos
from math import acos
from math import asin
from math import atan2
from math import degrees as r2d
import pyquaternion as pq
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_matrix as q2m
from tf.transformations import euler_from_matrix as m2e
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import quaternion_from_matrix as m2q
from tf.transformations import rotation_matrix
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_conjugate as conj
import scipy.integrate as integrate

import sys


# Private Globals
gravity = [0, -9.81, 0]  # TODO: can be tuned # gravity vector in global frame


# Private Functions
def q_scale(q, s):
    '''
    Scale Quaternion objects
    '''
    if type(q) == Quaternion:
        q_scaled = Quaternion(s*q.x, s*q.y, s*q.z, s*q.w)
    elif type(q) == np.ndarray:
        q_scaled = s*q
    else:
        print "unknown dtype"
    return q_scaled


def q_rotate(q, v):
    ''''
    Computes v_rotated = qvq* fastly
    @params q: rotating quaternion represented as numpy array [q0, q1, q2, q3] -> [w, x, y, z]
    @params v: to be rotated vector. Represented as numpy array. Expressed as pure quaternion. [v0, v1, v2] -> [x, y, z]
    '''
    if type(q) == np.ndarray:
        #  [q0, q1, q2, q3] -> [w, x, y, z]
        v_rotated = np.array([v[0]*(q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) + 2*v[1]*(q[1]*q[2]-q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[1]*q[3]),
                          2*v[0]*(q[0]*q[3] + q[1]*q[2]) + v[1]*(q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2) + 2*v[2]*(q[2]*q[3] - q[0]*q[1]),
                          2*v[0]*(q[1]*q[3] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[2]*q[3]) + v[2]*(q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)])

        # [q0, q1, q2, q3] -> [x, y, z, w]  NOT WORKING
        # v_rotated = np.array([v[0]*(q[1]**2 + q[2]**2 - q[3]**2 - q[0]**2) + 2*v[1]*(q[2]*q[3]-q[1]*q[0]) + 2*v[2]*(q[1]*q[3] + q[2]*q[0]),
        #                   2*v[0]*(q[1]*q[0] + q[2]*q[3]) + v[1]*(q[1]**2 - q[2]**2 + q[3]**2 - q[0]**2) + 2*v[2]*(q[3]*q[0] - q[1]*q[2]),
        #                   2*v[0]*(q[2]*q[0] - q[1]*q[3]) + 2*v[1]*(q[1]*q[2] + q[3]*q[0]) + v[2]*(q[1]**2 - q[2]**2 - q[3]**2 + q[0]**2)])
    elif type(q) == Quaternion and type(v) == np.ndarray:
        v_rotated = np.array([v[0]*(q.w**2 + q.x**2 - q.y**2 - q.z**2) + 2*v[1]*(q.x*q.y-q.w*q.z) + 2*v[2]*(q.w*q.y + q.x*q.z),
                          2*v[0]*(q.w*q.z + q.x*q.y) + v[1]*(q.w**2 - q.x**2 + q.y**2 - q.z**2) + 2*v[2]*(q.y*q.z - q.w*q.x),
                          2*v[0]*(q.x*q.z - q.w*q.y) + 2*v[1]*(q.w*q.x + q.y*q.z) + v[2]*(q.w**2 - q.x**2 - q.y**2 + q.z**2)])
    elif type(q) == Quaternion and type(v) == Vector3:
        v_rotated = np.array([v.x*(q.w**2 + q.x**2 - q.y**2 - q.z**2) + 2*v.y*(q.x*q.y-q.w*q.z) + 2*v.z*(q.w*q.y + q.x*q.z),
                          2*v.x*(q.w*q.z + q.x*q.y) + v.y*(q.w**2 - q.x**2 + q.y**2 - q.z**2) + 2*v.z*(q.y*q.z - q.w*q.x),
                          2*v.x*(q.x*q.z - q.w*q.y) + 2*v.y*(q.w*q.x + q.y*q.z) + v.z*(q.w**2 - q.x**2 - q.y**2 + q.z**2)])
    else:
        print "unknown quaternion type"
        v_rotated = v

    # v2 = np.concatenate([[0], v1], axis=0)
    # q2 = quaternion_multiply(v2, conj(q1))
    # q2 = quaternion_multiply(q1, q2)
    return v_rotated


    def q_rotate2(q, v):
        p = np.array([0, v[0], v[1], v[2]], dtype=np.float)
        q_rotated = q_multiply(q, q_multiply(p, q_invert(q)))
        print "rotated: "
        sys.exit(q_rotated)


def q_invert(q):
    '''
    Invert unit quaternion
    '''
    # print "q input:{0}".format(q)
    q_test = pq.Quaternion()
    q_inverted = Quaternion()
    if type(q) == Quaternion:
        # if the dtype is tf.quaternion
        q_inverted.x = -q.x
        q_inverted.y = -q.y
        q_inverted.z = -q.z
        q_inverted.w = q.w
        return q_inverted
    elif type(q) == np.ndarray:  # w-x-y-z
        q_inverted = np.array([q[0], -q[1], -q[2], -q[3]])
        return q_inverted
    elif type(q) == type(q_test):
        q_inverted = pq.Quaternion(q.w, -q.x, -q.y, -q.z)
        # print "q inverted:{0}".format(q_inverted)
        return q_inverted
    else:
        print "unknown dtype"
        return q

def q_mult_norm(q, p):
    '''
    Multiply quaternions and normalize the result to use in rotation
    '''
    q_qp = np.array([1, 0, 0, 0], dtype=np.float64)  # w-x-y-z
    if (type(p) == type(q)) and (type(p) == np.ndarray):
        q_qp[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]
        q_qp[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2]
        q_qp[2] = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1]
        q_qp[3] = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0]
    else:
        "unknown quaternion type"
    return q_qp


def q_multiply(q, p):
    '''
    Multiply quaternions
    '''
    q_qp = pq.Quaternion()
    if type(q) == Quaternion:
        # print "type: geometry_msgs.msg/Quaternion"
        # TODO: test it before use it. p*q -> q*p olmus olabilir
        q_multiplied = Quaternion()
        q_multiplied.w = q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z
        q_multiplied.x = q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y
        q_multiplied.y = q.w*p.y + q.y*p.w - q.x*p.z + q.z*p.x
        q_multiplied.z = q.w*p.z + q.z*p.w + q.x*p.y - q.y*p.x
        return q_multiplied
    # elif type(q) == np.ndarray:
    #     # print "type: np.array()"
    #     q_multiplied = np.array([0.0, 0.0, 0.0, 1.0])  # 'xyzw'
    #     q_multiplied[0] = q[3]*p[0] + q[0]*p[3] + q[1]*p[2] - q[2]*p[1]
    #     q_multiplied[1] = q[3]*p[1] + q[1]*p[3] - q[0]*p[2] + q[2]*p[0]
    #     q_multiplied[2] = q[3]*p[2] + q[2]*p[3] + q[0]*p[1] - q[1]*p[0]
    #     q_multiplied[3] = q[3]*p[3] - q[0]*p[0] - q[1]*p[1] - q[2]*p[2]
    elif type(q) == np.ndarray:
        q_qp = np.array([1, 0, 0, 0], dtype=np.float64)  # w-x-y-z
        if (type(p) == type(q)) and (type(p) == np.ndarray):
            q_qp[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3]
            q_qp[1] = q[0]*p[1] + q[1]*p[0] + q[2]*p[3] - q[3]*p[2]
            q_qp[2] = q[0]*p[2] - q[1]*p[3] + q[2]*p[0] + q[3]*p[1]
            q_qp[3] = q[0]*p[3] + q[1]*p[2] - q[2]*p[1] + q[3]*p[0]
        return q_qp
    elif type(q) == type(q_qp):
        print "type: pyquaternion/Quaternion"
        q_qp = q*p
        return q_qp
    else:
        print "unknown dtype"
        return q


def q_tf_convert(q):
    '''
    Convert geometry_msgs.Quaternion objects into numpy array for making use of in tf.transformations
    '''
    q_converted = np.array([0.0, 0.0, 0.0, 1.0])
    q_converted[0] = q.x
    q_converted[1] = q.y
    q_converted[2] = q.z
    q_converted[3] = q.w
    return q_converted


def q_magnitude(q):
    if type(q) == Quaternion:
        q_mag = sqrt((q.w**2 + q.x**2 + q.y**2 + q.z**2))
    elif type(q) == np.ndarray:
        q_mag = sqrt((q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2))
    else:
        print "unknown dtype"
    return q_mag


def v_magnitude(v):
    v_mag = sqrt(v[0]**2+v[1]**2+v[2]**2)
    return v_mag


def q_norm(q):
    flag = "normalization problem"
    try:
        q_normalized = q_scale(q, q_magnitude(q)**(-1))
        flag = "normalized"
    except ZeroDivisionError:
        print("Magnitude is zero")
        q_normalized = q
    finally:
        # print flag
        return q_normalized


def find_angle(a, b):
    """
    https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
    &
    A Tutorial on Euler Angles and Quaternions
    Moti Ben-Ari
    Department of Science Teaching
    Weizmann Institute of Science
    http://www.weizmann.ac.il/sci-tea/benari/
    """
    # v = np.cross(a, b)
    # # print v
    # skew_v = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    # dot_ab = np.dot(a, b)
    # # print dot_ab
    # k = 1/(dot_ab + 1)  # check if angle -180
    # C_unnorm = np.eye(3) + skew_v + np.square(skew_v)*k
    # C = C_unnorm/np.linalg.det(C_unnorm)
    # trace_c = C[0][0] + C[1][1] + C[2][2]
    # q = Quaternion()
    # q.w = sqrt((trace_c + 1)/4)
    # q.x = sqrt(C[0][0]/2 + (1-trace_c)/4)
    # q.y = sqrt(C[1][1]/2 + (1-trace_c)/4)
    # q.z = sqrt(C[2][2]/2 + (1-trace_c)/4)
    # solid_angle = acos(q.w)*180/3.14
    # a2 = q_rotate(q, a)
    # print a2, b/v_magnitude(b)

#  Interestingly, this doesn't give me ZeroDivisionError. Instead it gives RuntimeWarning. Both doesn't work in any ways. Let's do it in a shitty way you shit python. Is that what you want? ARGHH...
    # try:
    #     a = a/v_magnitude(a)
    #     b = b/v_magnitude(b)
    #
    #     cross_ab = np.cross(a, b)
    #     v = cross_ab / v_magnitude(cross_ab)
    #     dot_ab = np.dot(a, b)
    #     theta = acos(dot_ab)/2
    #     q = Quaternion(sin(theta)*v[0], sin(theta)*v[1], sin(theta)*v[2], cos(theta))
    #
    #     angle_x = atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x**2+q.y**2))
    #     angle_y = asin(2*(q.w*q.y-q.z*q.x))
    #     angle_z = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))
    #     angle = [angle_x, angle_y, angle_z]
    #
    # except RuntimeWarning:
    #     print "Zero dimension vector detected. Angle couldn't be found."
    #     angle = [0, 0, 0]
    #     q = Quaternion(0, 0, 0, 1)
    #
    # return angle, q

# Shitt way
    if not (v_magnitude(a) == 0 or v_magnitude(b) == 0):
        # print "A:", a
        # print "B:", b
        a = a/v_magnitude(a)
        b = b/v_magnitude(b)
        # print "unitA:", a
        # print "unitB:", b

        cross_ab = np.cross(a, b)
        v = cross_ab / v_magnitude(cross_ab)
        dot_ab = np.dot(a, b)
        theta = acos(dot_ab)/2
        q = Quaternion(sin(theta)*v[0], sin(theta)*v[1], sin(theta)*v[2], cos(theta))

        if q.w == 1.00:
            angle_x = 0.0
            angle_y = 0.0
            angle_z = 0.0
            angle = [angle_x, angle_y, angle_z]
        else:
            angle_x = atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x**2+q.y**2))
            angle_y = asin(2*(q.w*q.y-q.z*q.x))
            angle_z = atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2))
            angle = [angle_x, angle_y, angle_z]

        angle_deg = [r2d(angle_x), r2d(angle_y), r2d(angle_z)]
        # print "angle: ", angle_deg

    else:
        # print "Zero dimension vector detected. Angle couldn't be found."
        angle = [0, 0, 0]
        q = Quaternion(0, 0, 0, 1)

    return angle, q


# Functions
def q_gb_update(q_sensor_to_global, q_sensor_to_body):
    '''
    Known orientation of each segment and given sensor orientation in global frame, calculates the body orientation in global frame.
    {GB}q = {GS}q * {BS}q'  (q' is the conjugate quaternion)
    @param q_sensor_to_global: Calculated orientation in quaternion from sensor to global frame (given by xsens for now)
    @param q_sensor_to_body: Known measurement of sensor placement wrt body origin
    @ returns: Orientation from body frame to global frame in quaternion
    '''
    q_body_to_global = quaternion_multiply(q_sensor_to_global, conj(q_sensor_to_body))
    return q_body_to_global


def angular_vel_to_orientation(q_sensor, gyro_sensor):
    '''
    Calculates the rotation from sensor frame to global frame by integration:
    {GS}q_t_dot = 0.5 * {GS}q_t * omega_t
    omega_t = (0, w_x, w_y, w_z) : angular velocity w_t
    @param q_sensor: {GS}q_t: previous orientation
    @param ang_vel: Gyroscope reading
    @returns: Orientation from sensor frame to global frame
    TODO: I skip the integration for now. I am using the orientation quaternion from the sensor directly. Implement this part in the future.
    '''
    pass
    # gyro_quat = Quaternion(gyro_sensor[0], gyro_sensor[1], gyro_sensor[2], 0)
    # q_gs_dot = q_scale(q_multiply(q_sensor,gyro_quat), 0.5)
    # q_gs = integrate.simps([q_gs_dot.x, q_gs_dot.y, q_gs_dot.z, q_gs_dot.w])
    # return q_gs


def acc_to_pos(q_sensor, acc_sensor, dt):
    '''
    Calculates integrated position from accelerometer readings
    {G}a_t - {G}g = {GS}q_t * ({S}a_t - {S}g) * {GS}q'_t
    {g}p_t_dotdot = {G}a_t
    @param
    '''
    acc_global = q_rotate(q_sensor, acc_sensor)
    delta_p = integrate.simps(integrate.simps(acc_global - gravity, dx=dt))  # 3x1
    return delta_p


def segment_kinematics(p_u_prev, q_u, s_u):
    '''
    Calculates the origin of the next body segment in global frame.
    {G}p_u_next = {G}p_u_prev + {GB}q_u * {B}s_u * {GB}q_u_conj
    @param p_u_prev: np.darray, position of the previous joint in global frame
    @param q_u: orientation of the respective body part from body to global frame
    @param s_u: the length of the respective body part
    @returns: the next body segment's origin position.

    '''
    p_u_next = np.empty([1, 3])
    p_u_next = p_u_prev + q_rotate(q_u, s_u)
    return p_u_next


def join_update():
    '''
    Calculates the joint position using KF.

    TODO: every C matrix should be different than each other. for shoulder for example and for knee.
    '''
    C = np.concatenate((np.eye(3), -np.eye(3)), axis=1)

    pass


def double_integrate_acc(acc_global, dt=0.01):
    '''
    3D acc integration. rectangular way is computed.abs
    TODO: improve with trapezoid -> take prev acc value as a param
    '''
    delta_p = np.zeros(3)
    delta_v = np.zeros(3)
    for i in range(3):
        delta_v[i] += integrate.simps([acc_global[i], acc_global[i]], dx=dt)
        delta_p[i] += integrate.simps([delta_v[i], delta_v[i]], dx=dt)
    return delta_p
