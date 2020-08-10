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

    def __init__(self, X, F, Q, Z, H, R, P, B=np.array([1, 1]), M=np.array([0])):
        """
        Initialise the filter
        Args:
            X: State estimate
            P: Estimate covariance
            F: State transition model
            B: Control matrix
            M: Control vector
            Q: Process noise covariance
            Z: Measurement of the state X
            H: Observation model
            R: Observation noise covariance
        """
        self.X = X
        self.P = P
        self.F = F
        self.B = B
        self.M = M
        self.Q = Q
        self.Z = Z
        self.H = H
        self.R = R
        self.index = 0

    def predict(A, X, P, Q, M):
        """
        Predict the future state
        Args:
            A: Transition matrix
            X: State estimate
            P: Estimate covariance
            Q: Process covariance
            B_u: Control matrix * Control vector
            self.M: Control vector
        Returns:
            updated X
        """
        # Project the state ahead
        # print "state:", X
        X = A.dot(X) + M
        P = A.dot(P).dot(A.T) + Q
        return X

    @calculate_time.profile
    def update(X, P, C, R, Z):
        """
        Update the Kalman Filter from a measurement
        Args:
            X: State estimate
            P: Estimate covariance
            C: Observation matrix
            R: Measurement covariance
            Z: State measurement
        Returns:
            updated X
        """
        # calculate_time.print_prof_data()
        # print "correct X: ", self.X
        K = P.dot(C.T).dot(np.linalg.inv(C.dot(P).dot(C.T) + R))
        # print "K: ", K
        # print "Z: ", Z
        # print "size X:", self.X.shape
        X += K.dot((Z - C.dot(X)))
        P = P - K.dot(C).dot(P)
        # print "state: ", X
        return X
