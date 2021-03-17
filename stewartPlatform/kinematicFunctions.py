import numpy as np
from numpy import sqrt, cos, sin, arcsin, arctan2, pi, floor
import configFile as config


def getAnglesFromRotationMatrix(b_R_p):
    """ Computes Yaw, pitch, roll from rotation matrix b_R_p. """
    # Get yaw pitch roll angles
    psy = arctan2(b_R_p[1][0], b_R_p[0][0])
    theta = arctan2(-b_R_p[2][0], sqrt(b_R_p[0][0] ** 2 + b_R_p[1][0] ** 2))
    phi = arctan2(b_R_p[2][1], b_R_p[2][2])
    return [psy, theta, phi]


def getAlpha(effectiveLegLength, beta, base, platform):
    """ Computes servo angle as a function of the platform orientation and position,
        the leg's effective length and it's angle beta (angle between servo arm and base x-axis. """
    L = effectiveLegLength ** 2 - (config.legLength ** 2 - config.armLength ** 2)
    # M = 2a*(zp - zb)
    M = 2 * config.armLength * (platform.getOrigin()[0][2] - base.getOrigin()[0][2])
    # N = 2a*(cos Beta * (xp-xb) + sin Beta * (yp-yb)))
    N = 2 * config.armLength * (cos(beta * (platform.getOrigin()[0][0] - base.getOrigin()[0][0])) + sin(
        beta * (platform.getOrigin()[0][1] - base.getOrigin()[0][1])))
    return arcsin((L / sqrt(M ** 2 + N ** 2)) % 1) - arctan2(N, M)

# TODO: put general functions like this in another place/file (maybe?)
def getRotationMatrix(frame1, frame2):
    """ Computes the rotation matrix between the base and the platform. """
    return [
        [np.dot(frame2[0], frame1[0]), np.dot(frame2[1], frame1[0]), np.dot(frame2[2], frame1[0])],
        [np.dot(frame2[0], frame1[1]), np.dot(frame2[1], frame1[1]), np.dot(frame2[2], frame1[1])],
        [np.dot(frame2[0], frame1[2]), np.dot(frame2[1], frame1[2]), np.dot(frame2[2], frame1[2])]
    ]