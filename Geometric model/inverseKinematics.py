import numpy as np
from numpy import sqrt, cos, sin, arcsin, arctan2, pi, floor
import configFile as config
import matplotlib.pyplot as plt
import matplotlib.collections as mc


class frame:
    """ Position : column vector describing position in Base Frame
        Unit vectors [[Nx],[Ny],[Nz]] where Nx,y,z are column vectors
        representing unitvectors of the frame in the Base reference frame"""
    def __init__(self, origin, vectorBase):
        self.origin = origin
        self.vectorBase = vectorBase

    def getOrigin(self):
        return [self.origin]

    def getVectorBase(self):
        return [self.vectorBase]

    def setOrigin(self, origin):
        self.origin = origin

    def setVectorBase(self, vectorBase):
        self.vectorBase = vectorBase


def initializePlatform():
    # Position of hexagon corners - Base & Platform
    baseCorners = []
    platformCorners = []
    for i in range(config.numberOfCorners):
        baseCorners.append(corner(i, config.baseInteriorRadius, config.baseExteriorRadius, "base"))
        platformCorners.append(corner(i, config.platformInteriorRadius, config.platformExteriorRadius, "platform"))

    # Home position platform - computed from base/platform corners #1 but any would do by symmetry
    platformHomeZPosition = sqrt(config.armLength ** 2 + config.legLength ** 2 - (baseCorners[0][0] - platformCorners[0][0]) ** 2 - (
                baseCorners[0][1] - platformCorners[0][1]) ** 2)

    # Angle of servos at home position
    L0 = 2 * config.armLength ** 2
    M0 = 2 * config.armLength * platformHomeZPosition
    N0 = 2 * config.armLength * (baseCorners[0][0] - platformCorners[0][0])
    alpha0 = arcsin(L0 / sqrt(M0 ** 2 + N0 ** 2)) - arctan2(N0, M0)

    return [baseCorners, platformCorners, platformHomeZPosition, alpha0, config.armLength, config.legLength, config.baseInteriorRadius,
            config.baseExteriorRadius, config.platformInteriorRadius, config.platformExteriorRadius, config.betas]


def corner(idx, r_in, r_out, type):
    """ Returns the position of a corner of an hexagon.
        Hexagon is purely 2D, thus z = 0 in it's frame of reference."""
    baseAndPlatformOffsetAngle = 0 if type == "base" else pi / 3
    angle = (baseAndPlatformOffsetAngle + (2 * pi / 3) * floor(idx / 2)) % (2 * pi)
    a_delta = 2 * (2 * r_in - r_out) / sqrt(3)
    x = r_out * cos(angle) + ((-1) ** idx) * (a_delta / 2) * sin(angle)
    y = r_out * sin(angle) - ((-1) ** idx) * (a_delta / 2) * cos(angle)
    return [x, y, 0]


def pathSampling(platform, targetPosition):
    """ Return a list of waypoints to get to the target.
        Each waypoint is a new frame coinciding with the new platform location
        Uses precision from config file """
    actualPos = platform.getOrigin()[0]
    target = targetPosition.getOrigin()[0]

    # Number of samples - distance to travel sampled at a rate defined by config variable pathSamplingPrecision
    distance = sqrt(np.subtract(target[0], actualPos[0]) ** 2 + np.subtract(target[1], actualPos[1]) ** 2 + np.subtract(target[2], actualPos[2]) ** 2)
    numberOfWaypoints = round(distance / config.pathSamplingPrecision)

    # Rotation angles
    [psy, theta, phi] = np.subtract(getAngles(getRotationMatrix(base, targetPosition)), getAngles(getRotationMatrix(base, platform)))

    # Path Sampling
    waypoints = []
    for i in range(numberOfWaypoints):
        # Rotate platform's basis vectors
        new_vectorBase = np.matmul(setRotationMatrix(i * psy / numberOfWaypoints, i * theta / numberOfWaypoints, i * phi / numberOfWaypoints), platform.getVectorBase()[0])
        waypoints.append(frame([
                        actualPos[0] + (i * (target[0] - actualPos[0]) / numberOfWaypoints),
                        actualPos[1] + (i * (target[1] - actualPos[1]) / numberOfWaypoints),
                        actualPos[2] + (i * (target[2] - actualPos[2]) / numberOfWaypoints)],
                             new_vectorBase))
    return waypoints


def getAngles(b_R_p):
    """ Computes Yaw, pitch, roll from rotation matrix b_R_p. """
    # Get yaw pitch roll angles
    psy = arctan2(b_R_p[1][0], b_R_p[0][0])
    theta = arctan2(-b_R_p[2][0], sqrt(b_R_p[0][0]**2 + b_R_p[1][0]**2))
    phi = arctan2(b_R_p[2][1], b_R_p[2][2])
    return [psy, theta, phi]


def getRotationMatrix(base, platform):
    """ Computes the rotation matrix between 2 frames. """
    # Get rotation matrix between base and platform
    b = base.getVectorBase()[0]
    p = platform.getVectorBase()[0]
    b_R_p = [[np.dot(p[0], b[0]), np.dot(p[1], b[0]), np.dot(p[2], b[0])],
             [np.dot(p[0], b[1]), np.dot(p[1], b[1]), np.dot(p[2], b[1])],
             [np.dot(p[0], b[2]), np.dot(p[1], b[2]), np.dot(p[2], b[2])]]
    return b_R_p


def setRotationMatrix(psy, theta, phi):
    """ Sets the rotation matrix needed given yaw, pitch and roll.
        Based on Yaw, pitch, roll - Euler ZYX(psy, theta, phi) convention.
        platform unitvectors are given as: [[Px],[Py],[Pz]] in reference frame Base
        psy = angle around z
        theta = angle around y
        phi = angle around x """

    # Elementary Rotation Matrix
    R_Z = np.array([[cos(psy), -sin(psy), 0], [sin(psy), cos(psy), 0], [0, 0, 1]])
    R_Y = np.array([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
    R_X = np.array([[1, 0, 0], [0, cos(phi), -sin(phi)], [0, sin(phi), cos(phi)]])
    
    # Full rotation matrix
    R = np.matmul(np.matmul(R_Z, R_Y), R_X)
    return R


def getAlpha(effectiveLegLength, beta, base, platform):
    """ Computes servo angle as a function of the platform orientation and position,
        the leg's effective length and it's angle beta (angle between servo arm and base x-axis. """
    L = effectiveLegLength**2 - (legLength**2 - armLength**2)
    # M = 2a*(zp - zb)
    M = 2 * armLength * (platform.getOrigin()[0][2] - base.getOrigin()[0][2])
    # N = 2a*(cos Beta * (xp-xb) + sin Beta * (yp-yb)))
    N = 2 * armLength * (cos(beta * (platform.getOrigin()[0][0] - base.getOrigin()[0][0])) + sin(beta * (platform.getOrigin()[0][1] - base.getOrigin()[0][1])))
    alpha = arcsin(L / sqrt(M**2 + N**2)) - arctan2(N, M)
    # Return servo motor angle
    return alpha


def inverseKinematics(targetPosition):
    """ Takes in parameters the new target frame the user wants to go to and the actual
        frame (given by the object platform).
        Returns servo angles necessary to accomplish the movement. """
    translation = targetPosition.getOrigin()
    base_R_platform = getRotationMatrix(base, targetPosition)

    # Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi
    leg_lengths = []
    for leg in range(config.numberOfCorners):
        leg_lengths.append(np.add(translation, np.subtract(np.matmul(base_R_platform, platformCorners[leg]), baseCorners[leg])))

    # Compute servo angles to get effective leg lengths
    servoAngles = []
    for servo in range(config.numberOfCorners):
        servoAngles.append(getAlpha(np.linalg.norm(leg_lengths[servo]), betas[servo], base, targetPosition))
    return servoAngles


if __name__ == "__main__":
    # Initialize platform
    [baseCorners, platformCorners, platformHomeZPosition, alpha0, armLength, legLength, baseInteriorRadius,
     baseExteriorRadius, platformInteriorRadius, platformExteriorRadius, betas] = initializePlatform()

    # Initialize reference frames
    referenceVectorBase = np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])])
    base = frame([0, 0, 0], referenceVectorBase)
    platform = frame([0, 0, platformHomeZPosition], referenceVectorBase)
    bx = []
    by = []
    bz = []
    px = []
    py = []
    pz = []
    pCorners = []

    # Place points on base and platform
    for i in range(config.numberOfCorners):
        pCorners.append(np.add(platform.getOrigin(), np.matmul(getRotationMatrix(base, platform), platformCorners[i])))
        bx.append(baseCorners[i][0])
        by.append(baseCorners[i][1])
        bz.append(baseCorners[i][2])
        px.append(pCorners[i][0][0])
        py.append(pCorners[i][0][1])
        pz.append(pCorners[i][0][2])
    # List of lines to plot
    baseLines = []
    platformLines = []
    legLines = []
    for i in range(config.numberOfCorners):
        baseLines.append([(bx[i], by[i], bz[i]), (bx[(i + 1) % 6], by[(i + 1) % 6], bz[(i + 1) % 6])])
        platformLines.append([(px[i], py[i], pz[i]), (px[(i + 1) % 6], py[(i + 1) % 6], pz[(i + 1) % 6])])
        legLines.append([(bx[i], by[i], bz[i]), (px[(i+1)%6], py[(i+1)%6], pz[(i+1)%6])])
    figure = plt.figure()
    axis = plt.axes(projection = '3d')
    axis.scatter(bx, by, bz)
    axis.scatter(px, py, pz)
    plt.show()

    # Set target
    targetPosition = [1, 1, platformHomeZPosition]
    targetOrientation = np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])])
    target = frame(targetPosition, targetOrientation)

    # Discretize trajectory
    waypoints = pathSampling(platform, target)

    # Compute set of servo angles to follow trajectory
    for point in waypoints:
        servoAngles = inverseKinematics(point)
        print(servoAngles)

    # Update platform current position
    platform.setOrigin(targetPosition)
    platform.setVectorBase(targetOrientation)