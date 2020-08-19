#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
In this file, there is only kalman related things.abs
# TODO: make it as a class

"""

import calculate_time
import numpy as np
from util import*


def calculate_b_u(acc, quat, win_size=51, index=0):
    DT = 0.01
    half_window = (win_size+1)*0.5
    pos = np.empty([len(acc), acc[0].shape[0], 3])
    delta_p = np.empty([len(acc), acc[0].shape[0], 3])
    vel = np.empty([len(acc), acc[0].shape[0], 3])
    b_u = np.empty([acc[0].shape[0], 6])
    for i in range(len(acc)):
        delta_p[i][0] = np.zeros(3)
        vel[i][0] = np.zeros(3)
        pos[i][0] = np.zeros(3)
        # TODO: alttakini de multiple IMU icin yap
    for i in range(1, 26):  # until window_size/2 +1
    #  This recovers the errors in the calibration step
        delta_p[0][i] = np.zeros(3)
        delta_p[1][i] = np.zeros(3)

        vel[0][i] = np.zeros(3)
        vel[1][i] = np.zeros(3)

        pos[0][i] = np.zeros(3)
        pos[1][i] = np.zeros(3)

        b_u[i] = np.concatenate((delta_p[0][i], delta_p[1][i]), axis=0)
        #  This has to chage if you don't use zero array for this part
    for i in range(26, acc[0].shape[0]):  # until window_size/2 +1
        delta_p[0][i] = vel[0][i-1]*DT+0.5*acc[0][i]*DT*DT
        delta_p[1][i] = vel[1][i-1]*DT+0.5*acc[1][i]*DT*DT

        pos[0][i] = pos[0][i-1]+delta_p[0][i]
        pos[1][i] = pos[1][i-1]+delta_p[1][i]

        vel[0][i] = vel[0][i-1]+acc[0][i]*DT
        vel[1][i] = vel[1][i-1]+acc[1][i]*DT

        b_u[i] = np.concatenate((delta_p[0][i], delta_p[1][i]), axis=0)
        # print "b_u[i]",i, b_u[-1]
        # b_u[i] = np.concatenate(((delta_p[0][i]-delta_p[0][i-1]), (delta_p[1][i]-delta_p[1][i-1])), axis=0)
    # print "acc:", acc[0].shape
    # print "vel:", vel[0].shape
    # print "delta_p", delta_p[0].shape
    # print "p:", delta_p[0]
    # print "v:", vel[0]
    # print "a:", acc[0]
    # t = np.arange(0, len(acc[0]))
    # plot_subplot(acc[0], "accelerometer", dt=DT)
    # plot_subplot(vel[0], "vel_IMU0", dt=DT)
    # plot_subplot(vel[1], "vel_IMU1", dt=DT)
    # plot_subplot(pos[1], "pos_IMU1", dt=DT)
    # plot_subplot(pos[0], "pos_IMU0", dt=DT)
    # plt.show()
    return pos, b_u


class KalmanFilter:
    """
    Simple Kalman filter
    """

    def __init__(self, X, F, Q, Z, H, R, P, M=np.array([0])):
        """
        Initialise the filter
        Args:
            X: State estimate
            P: Estimate covariance
            F: State transition model
            M: Control matrix
            Q: Process noise covariance
            Z: Measurement of the state X
            H: Observation model
            R: Observation noise covariance
        """
        self.X = X
        self.P = P
        self.F = F
        self.M = M
        self.Q = Q
        self.Z = Z
        self.H = H
        self.R = R
        self.index = 0

    def predict(self, M=np.array([0])):
        """
        Predict the future state
        Args:
            self.X: State estimate
            self.P: Estimate covariance
            self.B: Control matrix
            self.M: Control vector
        Returns:
            updated self.X
        """
        # print "self.x: {0}, index:{1}".format(self.X, self.index)
        self.M = M.reshape(self.F.shape[0],1)
        self.index += 1
        # Project the state ahead
        if self.index == 2503:
            print "X prev:", self.X[2]-self.X[5]
            # print "index:", self.index
        # print "M:", self.M
        self.X = self.F.dot(self.X) + self.M
        if self.index == 2503:
            print "X prev2:", self.X[2]-self.X[5]
        # print "F:", self.F
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
        self.index += 1

        return self.X

    def correct(self, Z):
        """
        Update the Kalman Filter from a measurement
        Args:
            self.X: State estimate
            self.P: Estimate covariance
            Z: State measurement
        Returns:
            updated X
        """
        K = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
        # print "size X:", self.X.shape
        # print "size K:", K.shape
        # print "size H:", self.H.shape
        # print "size Z: ", Z.shape
        if self.index == 2504:
            print "index:", self.index
            print "X after:", self.X[2]-self.X[5]
        print "error", (self.H.dot(self.X)), "y", Z
        self.X = self.X + K.dot(Z - self.H.dot(self.X))
        self.P = self.P - K.dot(self.H).dot(self.P)

        # self.X[:3] = self.X[:3]+self.X[3:]

        return self.X
