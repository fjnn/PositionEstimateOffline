#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
In this file, there is only kalman related things.abs
# TODO: make it as a class

"""

import calculate_time
import numpy as np

DT = 0.01


def calculate_b_u(acc, quat, index=0):
    delta_p = np.empty([len(acc), acc[0].shape[0], 3])
    error_p = np.zeros([len(acc), acc[0].shape[0], 3])
    for i in range(acc[0].shape[0]):
        delta_p[0][i] = 0.5*acc[0][i]*DT*DT
        # print "delta_p", delta_p[0][i], index
    return delta_p


def kalman_predict(A, X, P, Q, B_u):
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
    X = A.dot(X) + B_u
    P = A.dot(P).dot(A.T) + Q
    return X


@calculate_time.profile
def kalman_update(X, P, C, R, Z):
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
