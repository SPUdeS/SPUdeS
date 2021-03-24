from numpy import dot, sqrt, cos, sin, arcsin, arctan2
from Platform import config


def getRotationMatrix(vb1, vb2):
    """ Computes the rotation matrix between two vector bases.
     Convention: Euler ZYX (α,β,γ). """
    return [
        [dot(vb1[0], vb2[0]), dot(vb1[0], vb2[1]), dot(vb1[0], vb2[2])],
        [dot(vb1[1], vb2[0]), dot(vb1[1], vb2[1]), dot(vb1[1], vb2[2])],
        [dot(vb1[2], vb2[0]), dot(vb1[2], vb2[1]), dot(vb1[2], vb2[2])]
    ]


def getAnglesFromRotationMatrix(b_R_p):
    """ Computes Yaw, pitch, roll from rotation matrix b_R_p.
    Convention: Euler ZYX (α,β,γ). """
    alpha = arctan2(b_R_p[1][0], b_R_p[0][0])
    beta = arctan2(-b_R_p[2][0], sqrt(b_R_p[0][0] ** 2 + b_R_p[1][0] ** 2))
    gamma = arctan2(b_R_p[2][1], b_R_p[2][2])
    return [alpha, beta, gamma]


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
