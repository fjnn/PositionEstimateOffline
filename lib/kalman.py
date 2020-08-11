#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
In this file, there is only kalman related things.abs
# TODO: make it as a class

"""

import calculate_time
import numpy as np


def calculate_b_u(acc, quat, index=0):
    DT = 0.01
    delta_p = np.empty([len(acc), acc[0].shape[0], 3])
    b_u = np.empty([acc[0].shape[0], 6])
    for i in range(acc[0].shape[0]):
        delta_p[0][i] = 0.5*acc[0][i]*DT*DT
        delta_p[1][i] = 0.5*acc[1][i]*DT*DT
        # TODO. is it necessary to add v.dt
        b_u[i] = np.concatenate((delta_p[0][i], delta_p[1][i]), axis=0)
    return b_u


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
        self.M = M
        self.index += 1
        # Project the state ahead
        self.X = self.F.dot(self.X) + self.M.dot(self.M)
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
        self.index += 1

        # print "u:", self.M
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
        self.X += K.dot(Z - self.H.dot(self.X))
        self.P = self.P - K.dot(self.H).dot(self.P)

        return self.X
