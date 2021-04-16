""" This module defines various kinematic functions used in the stewartClasses and the stewartPlatform. """

import numpy as np
from numpy import dot, sqrt, cos, sin, arcsin, arctan2, array, matmul, ceil
from SPUdeS.Platform import config

np.seterr(invalid='ignore')


def getRotationMatrixFromAngles(alpha, beta, gamma):
    """ Sets the rotation matrix needed given yaw, pitch and roll.
        Based on Yaw, pitch, roll - Euler ZYX(alpha, beta, gamma) convention.
        platform unitvectors are given as: [[Px],[Py],[Pz]] in reference frame Base
        alpha = angle around z
        beta = angle around y
        gamma = angle around x """

    # Elementary Rotation Matrix
    R_Z = array([[cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0], [0, 0, 1]])
    R_Y = array([[cos(beta), 0, sin(beta)], [0, 1, 0], [-sin(beta), 0, cos(beta)]])
    R_X = array([[1, 0, 0], [0, cos(gamma), -sin(gamma)], [0, sin(gamma), cos(gamma)]])

    # Full rotation matrix
    return matmul(matmul(R_Z, R_Y), R_X)


def getRotationMatrix(vb1, vb2):
    """ Computes the rotation matrix between two vector bases.
     Convention: Euler ZYX (α,β,γ). """
    return matmul(vb1, vb2)


def getAnglesFromRotationMatrix(b_R_p):
    """ Computes Yaw, pitch, roll from rotation matrix b_R_p.
    Convention: Euler ZYX (α,β,γ). """
    alpha = arctan2(b_R_p[1][0], b_R_p[0][0])
    beta = arctan2(-b_R_p[2][0], sqrt(b_R_p[0][0] ** 2 + b_R_p[1][0] ** 2))
    gamma = arctan2(b_R_p[2][1], b_R_p[2][2])
    return [alpha, beta, gamma]


def getNumberOfWaypoints(initialOrigin, initialVectorBase, targetOrigin, targetVectorBase):
    """ Returns the number of samples.
        Distance to travel sampled at a rate defined by config variable pathSamplingPrecision. """
    distance = sqrt(
        np.subtract(targetOrigin[0], initialOrigin[0]) ** 2 +
        np.subtract(targetOrigin[1], initialOrigin[1]) ** 2 +
        np.subtract(targetOrigin[2], initialOrigin[2]) ** 2)
    angle = max(getAnglesFromRotationMatrix(getRotationMatrix(initialVectorBase, targetVectorBase)))

    distanceWaypoints = ceil(distance / config.pathSamplingPrecision)
    angleWaypoints = ceil(abs(angle / config.angleSamplingPrecision))
    numberOfWaypoints = int(max(distanceWaypoints, angleWaypoints))
    return numberOfWaypoints if (numberOfWaypoints > 0) else 1


def getAlpha(effectiveLegLength, beta, base, platform):
    """ Computes servo angle as a function of the platform orientation and position,
        the leg's effective length and it's angle beta (angle between servo arm and base x-axis). """
    L = effectiveLegLength ** 2 - (config.legLength ** 2 - config.armLength ** 2)
    # M = 2a*(zp - zb)
    M = 2 * config.armLength * (platform.getOrigin()[2] - base.getOrigin()[2])
    # N = 2a*(cos Beta * (xp-xb) + sin Beta * (yp-yb)))
    N = 2 * config.armLength * (cos(beta) * (platform.getOrigin()[0] - base.getOrigin()[0])
                                + sin(beta) * (platform.getOrigin()[1] - base.getOrigin()[1]))
    return arcsin(L / sqrt(M ** 2 + N ** 2)) - arctan2(N, M)
