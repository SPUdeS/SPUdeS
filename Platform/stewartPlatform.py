import numpy as np
from numpy import sqrt, cos, sin, arcsin, arctan2, pi, floor
from Platform import config
import matplotlib.pyplot as plt
import matplotlib.collections as mc
from Platform import stewartClasses as sc
from Platform import kinematicFunctions as kf

class stewartPlatform:
    """ Contains inner classes base and platform"""

    def __init__(self):
        self.base = sc._base()
        self.platform = sc._platform(self.base)
        self.homeServoAngles = None
        self.homeServoAngles = self.getHomeServoAngle()
        self.legPointsToJoin = self.updateLegLines()
        self.servoAngles = self.homeServoAngles

    def getHomeServoAngle(self):
        if self.homeServoAngles is not None: return self.homeServoAngles
        # Angle of anchors at home position
        L0 = 2 * config.armLength ** 2
        M0 = 2 * config.armLength * self.platform.origin[2]
        N0 = 2 * config.armLength * (self.base.anchors[0][0] - self.platform.anchors[0][0])
        return arcsin(L0 / sqrt(M0 ** 2 + N0 ** 2)) - arctan2(N0, M0)

    def plot(self):
        self.updateLegLines() #TODO: is this good?
        figure = plt.figure()
        axis = plt.axes(projection='3d')
        [bx, by, bz] = self.base.getPlotAnchors()
        [px, py, pz] = self.platform.getPlotAnchors()
        #TODO : ajuster hauteur np.add(self.initAnchors(), [0,0, config.platformHomePosition[2]])

        # Plot the 12 anchor points
        axis.scatter(bx, by, bz)
        axis.scatter(px, py, pz)

        # Plot base origin
        axis.scatter(self.base.origin[0], self.base.origin[1],
                     self.base.origin[2])
        # Plot platform origin
        axis.scatter(self.platform.origin[0], self.platform.origin[1],
                     self.platform.origin[2])

        self.drawLinesBetweenPoints(axis, self.platform.getPointsToJoin())
        self.drawLinesBetweenPoints(axis, self.base.getPointsToJoin())
        self.drawLinesBetweenPoints(axis, self.getLegPointsToJoin())

        plt.show()

    def drawLinesBetweenPoints(self, axis, listOfPointsToJoin):
        for points in listOfPointsToJoin:
            axis.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], [points[0][2], points[1][2]])

    def updateLegLines(self):
        self.legPointsToJoin = []
        [bx, by, bz] = self.base.getPlotAnchors()
        [px, py, pz] = self.platform.getPlotAnchors()
        for i in range(config.numberOfAnchors):
            self.legPointsToJoin.append([(bx[i], by[i], bz[i]), (px[i], py[i], pz[i])])

    def getLegPointsToJoin(self):
        return self.legPointsToJoin


    def pathSampling(self, targetPosition):
        """ Return a list of waypoints to get to the target.
            Each waypoint is a new frame coinciding with the new platform location
            Uses precision from config file """
        actualPos = self.platform.getOrigin()
        target = targetPosition.getOrigin()

        # Number of samples - distance to travel sampled at a rate defined by config variable pathSamplingPrecision
        distance = sqrt(
            np.subtract(target[0], actualPos[0]) ** 2 + np.subtract(target[1], actualPos[1]) ** 2 + np.subtract(
                target[2], actualPos[2]) ** 2)
        numberOfWaypoints = round(distance / config.pathSamplingPrecision)

        # Rotation angles
        [psy, theta, phi] = np.subtract(kf.getAnglesFromRotationMatrix(self.getRotationMatrixToTarget(targetPosition.vectorBase)),
                                        kf.getAnglesFromRotationMatrix(self.getBasePlatformRotationMatrix()))

        # Path Sampling
        waypoints = []
        for i in range(numberOfWaypoints):
            # Rotate platform's basis vectors
            new_vectorBase = np.matmul(
                self.setRotationMatrix(i * psy / numberOfWaypoints, i * theta / numberOfWaypoints,
                                  i * phi / numberOfWaypoints),
                self.platform.getVectorBase())
            waypoints.append(sc.frame([
                actualPos[0] + (i * (target[0] - actualPos[0]) / numberOfWaypoints),
                actualPos[1] + (i * (target[1] - actualPos[1]) / numberOfWaypoints),
                actualPos[2] + (i * (target[2] - actualPos[2]) / numberOfWaypoints)],
                new_vectorBase))
        return waypoints

    def inverseKinematics(self, target):
        """ Takes in parameters the new target frame the user wants to go to and the actual
            frame (given by the object platform).
            Returns servo angles necessary to accomplish the movement.
            targetPosition = [x, y, z, ψ, θ, φ ] (angles d'euler)
            """
        targetPosition = self.setTargetPosition(target)
        translation = targetPosition.getOrigin()
        base_R_platform = self.getRotationMatrixToTarget(targetPosition.vectorBase)

        # Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi
        leg_lengths = []
        for leg in range(config.numberOfAnchors):
            leg_lengths.append(
                np.add(translation, np.subtract(np.matmul(base_R_platform, self.platform.anchors[leg]),
                                                self.base.anchors[leg])))

        # Compute anchor angles to get effective leg lengths
        servoAngles = []
        for anchor in range(config.numberOfAnchors):
            servoAngles.append(
                kf.getAlpha(np.linalg.norm(leg_lengths[anchor]), config.betas[anchor], self.base, targetPosition))
        return servoAngles


    def setTargetPosition(self, target):
        return sc.frame(target.origin, target.vectorBase)

    def getBasePlatformRotationMatrix(self):
        """ Returns the rotation matrix between the base and the platform. """
        return kf.getRotationMatrix(self.base.getVectorBase(), self.platform.getVectorBase())

    def getRotationMatrixToTarget(self, targetVectorBase):
        """ Returns the rotation matrix between the base and a target frame. """
        return kf.getRotationMatrix(self.base.getVectorBase(), targetVectorBase)

    def setRotationMatrix(self, psy, theta, phi):
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

if __name__ == "__main__":
    # Initialize platform
    stewart = stewartPlatform()
    stewart.plot()

    # Set target
    targetPosition = [0, 0, stewart.platform.origin[2]+50]
    targetOrientation = np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])])
    target = sc.frame(targetPosition, targetOrientation)

    # Discretize trajectory
    waypoints = stewart.pathSampling(target)

    # Compute set of anchor angles to follow trajectory
    #todo: append servo angles in list in a stewart platform function
    for point in waypoints:
        #TODO: if NaN, dont append and return
        servoAngles = stewart.inverseKinematics(point)
        print(servoAngles)

    # Update platform current position
    stewart.platform.setOrigin(targetPosition)
    stewart.platform.setVectorBase(targetOrientation)

    stewart.plot()
    print(stewart.platform.getOrigin())
