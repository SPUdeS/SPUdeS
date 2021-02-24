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


class _piece(frame):
    """ Position : column vector describing position in Base Frame
        Unit vectors [[Nx],[Ny],[Nz]] where Nx,y,z are column vectors
        representing unitvectors of the frame in the Base reference frame"""

    def __init__(self, origin, vectorBase):
        super().__init__(origin, vectorBase)
        self.offsetAngle = None
        self.interiorRadius = None
        self.exteriorRadius = None
        self.corners = None
        self.cornersX = []
        self.cornersY = []
        self.cornersZ = []
        self.pointsToJoin = []

    def corner(self, idx, r_in, r_out):
        """ Returns the position of a corner of an hexagon.
            Hexagon is purely 2D, thus z = 0 in it's frame of reference."""
        angle = (self.offsetAngle + (2 * pi / 3) * floor(idx / 2)) % (2 * pi)
        a_delta = 2 * (2 * r_in - r_out) / sqrt(3)
        x = r_out * cos(angle) + ((-1) ** idx) * (a_delta / 2) * sin(angle)
        y = r_out * sin(angle) - ((-1) ** idx) * (a_delta / 2) * cos(angle)
        return [x, y, 0]

    def getHomeCorners(self):
        hexagonCorners = []
        for i in range(config.numberOfCorners):
            hexagonCorners.append(self.corner(i, self.interiorRadius, self.exteriorRadius))
        return hexagonCorners

    def updatePlotCorners(self):
        cx = []
        cy = []
        cz = []

        # Place plot points for platform
        for i in range(config.numberOfCorners):
            cx.append(self.corners[i][0])
            cy.append(self.corners[i][1])
            cz.append(self.corners[i][2])
        self.cornersX = cx
        self.cornersY = cy
        self.cornersZ = cz

    def getPlotCorners(self):
        return [self.cornersX, self.cornersY, self.cornersZ]

    def updatePointsToJoin(self):
        self.pointsToJoin = []
        [bx, by, bz] = self.getPlotCorners()
        for i in range(config.numberOfCorners):
            self.pointsToJoin.append([(bx[i], by[i], bz[i]), (bx[(i + 1) % 6], by[(i + 1) % 6], bz[(i + 1) % 6])])

    def getPointsToJoin(self):
        return self.pointsToJoin


class _base(_piece):
    def __init__(self, origin=config.baseHomePosition, vectorBase=config.vectorBase):
        super().__init__(origin, vectorBase)
        self.offsetAngle = 0
        self.interiorRadius = config.platformInteriorRadius
        self.exteriorRadius = config.platformExteriorRadius
        self.corners = self.getHomeCorners()
        self.updatePlotCorners()
        self.updatePointsToJoin()

    def corner(self, idx, r_in, r_out):
        return super(_base, self).corner(idx, r_in, r_out)


class _platform(_piece):
    def __init__(self, linkedBase, origin=config.platformHomePosition, vectorBase=config.vectorBase):
        super().__init__(origin, vectorBase)
        self.linkedBase = linkedBase
        self.offsetAngle = pi / 3
        self.origin = config.platformHomePosition
        self.interiorRadius = config.platformInteriorRadius
        self.exteriorRadius = config.platformExteriorRadius
        self.homeCorners = self.getHomeCorners()
        self.corners = self.homeCorners

    def _getHomePosition(self, linkedBase):
        # deprecated
        # Home position platform - computed from base/platform corners #1 but any would do by symmetry
        platformHomeZPosition = sqrt(
            config.armLength ** 2 + config.legLength ** 2 - (linkedBase.corners[0][0] - self.homeCorners[0][0]) ** 2 - (
                    linkedBase.corners[0][1] - self.homeCorners[0][1]) ** 2)
        return [0, 0, platformHomeZPosition]

    def getHomeCorners(self):
        hexagonCorners = super(_platform, self).getHomeCorners()
        platformCorners = []
        for i in range(config.numberOfCorners):
            platformCorners.append(np.add(self.getOrigin(),
                                          np.matmul(getRotationMatrix(self.linkedBase, self),
                                                    hexagonCorners[i])).tolist()[0])
        return platformCorners

    def corner(self, idx, r_in, r_out):
        return super(_platform, self).corner(idx, r_in, r_out)


class stewartPlatform:
    """ Contains inner classes base and platform"""

    def __init__(self):
        self.base = _base()
        self.platform = _platform(self.base)
        self.homeServoAngles = None
        self.homeServoAngles = self.getHomeServoAngle()
        self.legPointsToJoin = []
        self.servoAngles = []

    def getHomeServoAngle(self):
        if self.homeServoAngles is not None: return self.homeServoAngles
        # Angle of servos at home position
        L0 = 2 * config.armLength ** 2
        M0 = 2 * config.armLength * self.platform.origin[config.zPosition]
        N0 = 2 * config.armLength * (self.base.corners[0][0] - self.platform.homeCorners[0][0])
        return arcsin(L0 / sqrt(M0 ** 2 + N0 ** 2)) - arctan2(N0, M0)

    def plot(self):
        self.updatePlotCornersAndLines()
        figure = plt.figure()
        axis = plt.axes(projection='3d')
        [bx, by, bz] = self.base.getPlotCorners()
        [px, py, pz] = self.platform.getPlotCorners()
        axis.scatter(bx, by, bz)
        axis.scatter(px, py, pz)
        axis.scatter(self.base.origin[config.xPosition], self.base.origin[config.yPosition],
                     self.base.origin[config.zPosition])
        axis.scatter(self.platform.origin[config.xPosition], self.platform.origin[config.yPosition],
                     self.platform.origin[config.zPosition])

        self.drawLinesBetweenPoints(axis, self.platform.getPointsToJoin())
        self.drawLinesBetweenPoints(axis, self.base.getPointsToJoin())
        self.drawLinesBetweenPoints(axis, self.getLegPointsToJoin())
        plt.show()

    def drawLinesBetweenPoints(self, axis, listOfPointsToJoin):
        for points in listOfPointsToJoin:
            axis.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], [points[0][2], points[1][2]])

    def setLines(self):
        pass

    def updateLegLines(self):
        self._legPointsToJoin = []
        [bx, by, bz] = self.base.getPlotCorners()
        [px, py, pz] = self.platform.getPlotCorners()
        for i in range(config.numberOfCorners):
            self._legPointsToJoin.append([(bx[i], by[i], bz[i]), (px[(i + 1) % 6], py[(i + 1) % 6], pz[(i + 1) % 6])])

    def getLegPointsToJoin(self):
        return self.legPointsToJoin

    def updatePlotCornersAndLines(self):
        self.platform.updatePlotCorners()

        # List of lines to plot
        self.platform.updatePointsToJoin()
        self.updateLegLines()

    def pathSampling(self, targetPosition):
        """ Return a list of waypoints to get to the target.
            Each waypoint is a new frame coinciding with the new platform location
            Uses precision from config file """
        actualPos = self.platform.getOrigin()[0]
        target = targetPosition.getOrigin()[0]

        # Number of samples - distance to travel sampled at a rate defined by config variable pathSamplingPrecision
        distance = sqrt(
            np.subtract(target[0], actualPos[0]) ** 2 + np.subtract(target[1], actualPos[1]) ** 2 + np.subtract(
                target[2], actualPos[2]) ** 2)
        numberOfWaypoints = round(distance / config.pathSamplingPrecision)

        # Rotation angles
        [psy, theta, phi] = np.subtract(getAngles(getRotationMatrix(self.base, targetPosition)),
                                        getAngles(getRotationMatrix(self.base, self.platform)))

        # Path Sampling
        waypoints = []
        for i in range(numberOfWaypoints):
            # Rotate platform's basis vectors
            new_vectorBase = np.matmul(
                setRotationMatrix(i * psy / numberOfWaypoints, i * theta / numberOfWaypoints,
                                  i * phi / numberOfWaypoints),
                self.platform.getVectorBase()[0])
            waypoints.append(frame([
                actualPos[0] + (i * (target[0] - actualPos[0]) / numberOfWaypoints),
                actualPos[1] + (i * (target[1] - actualPos[1]) / numberOfWaypoints),
                actualPos[2] + (i * (target[2] - actualPos[2]) / numberOfWaypoints)],
                new_vectorBase))
        return waypoints

    def inverseKinematics(self, targetPosition):
        """ Takes in parameters the new target frame the user wants to go to and the actual
            frame (given by the object platform).
            Returns servo angles necessary to accomplish the movement. """
        translation = targetPosition.getOrigin()
        base_R_platform = getRotationMatrix(self.base, targetPosition)

        # Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi
        leg_lengths = []
        for leg in range(config.numberOfCorners):
            leg_lengths.append(
                np.add(translation, np.subtract(np.matmul(base_R_platform, self.platform.corners[leg]),
                                                self.base.corners[leg])))

        # Compute servo angles to get effective leg lengths
        servoAngles = []
        for servo in range(config.numberOfCorners):
            servoAngles.append(
                getAlpha(np.linalg.norm(leg_lengths[servo]), config.betas[servo], self.base, targetPosition))
        return servoAngles


def getRotationMatrix(base, position):
    """ Computes the rotation matrix between 2 frames. """
    # Get rotation matrix between base and platform
    b = base.getVectorBase()[0]
    p = position.getVectorBase()[0]
    return [
        [np.dot(p[0], b[0]), np.dot(p[1], b[0]), np.dot(p[2], b[0])],
        [np.dot(p[0], b[1]), np.dot(p[1], b[1]), np.dot(p[2], b[1])],
        [np.dot(p[0], b[2]), np.dot(p[1], b[2]), np.dot(p[2], b[2])]
    ]


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
    return np.matmul(np.matmul(R_Z, R_Y), R_X)


def getAngles(b_R_p):
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
    N = 2 * config.armLength * (cos(beta * (platform.getOrigin()[0][0] - base.getOrigin()[0][0])) + sin(beta * (platform.getOrigin()[0][1] - base.getOrigin()[0][1])))
    return arcsin((L / sqrt(M ** 2 + N ** 2)) % 1) - arctan2(N, M)
