from numpy import sqrt, arcsin, arctan2, isnan, add, subtract, matmul, column_stack, linalg, array
from Platform import config
import matplotlib.pyplot as plt
from Platform import stewartClasses as sc
from Platform import kinematicFunctions as kf


class stewartPlatform:
    """ Contains inner classes base and platform"""

    def __init__(self):
        self.base = sc.base()
        self.platform = sc.platform(self.base)
        self.homeServoAngles = self.initHomeServoAngle()
        self.servoAngles = self.homeServoAngles

    def initHomeServoAngle(self):
        # Angle of anchors at home position
        L0 = 2 * config.armLength ** 2
        M0 = 2 * config.armLength * self.platform.getOrigin()[2]
        N0 = 2 * config.armLength * (self.base.anchors[0][0] - self.platform.anchors[0][0])
        return arcsin(L0 / sqrt(M0 ** 2 + N0 ** 2)) - arctan2(N0, M0)

    def getHomeServoAngle(self):
        return self.homeServoAngles

    def plot(self):
        axis = plt.axes(projection='3d')
        baseOrigin = self.base.getOrigin()
        platformOrigin = self.base.getOrigin()
        [bx, by, bz] = self.base.getPlotAnchors()
        [px, py, pz] = self.platform.getPlotAnchors()

        # Plot the 12 anchor points and the base and platform origin
        axis.scatter(bx, by, bz)
        axis.scatter(px, py, pz)
        axis.scatter(baseOrigin[0], baseOrigin[1], baseOrigin[2])
        axis.scatter(platformOrigin[0], platformOrigin[1], platformOrigin[2])

        # Draw the base and platform hexagon lines and the connecting arms
        self.drawLinesBetweenPoints(axis, self.platform.getPointsToJoin())
        self.drawLinesBetweenPoints(axis, self.base.getPointsToJoin())
        self.drawLinesBetweenPoints(axis, self.getLegPointsToJoin())

        plt.show()

    @staticmethod
    def drawLinesBetweenPoints(axis, listOfPointsToJoin):
        for points in listOfPointsToJoin:
            axis.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], [points[0][2], points[1][2]])

    def getLegPointsToJoin(self):
        legPointsToJoin = []
        [bx, by, bz] = self.base.getPlotAnchors()
        [px, py, pz] = self.platform.getPlotAnchors()
        for i in range(config.numberOfAnchors):
            legPointsToJoin.append([(bx[i], by[i], bz[i]), (px[i], py[i], pz[i])])
        return legPointsToJoin

    def pathSampling(self, targetFrame):
        """ Return a list of waypoints to get to the target.
            Each waypoint is a new frame coinciding with the new platform location
            Uses precision from config file """
        initialOrigin = self.platform.getOrigin()
        targetOrigin = targetFrame.getOrigin()
        numberOfWaypoints = kf.getNumberOfWaypoints(initialOrigin, targetOrigin)
        [alpha, beta, gamma] = self.getRotationAnglesToFrame(targetFrame)
        incrementedTarget = []

        for i in range(numberOfWaypoints):
            increment = (i + 1) / numberOfWaypoints

            # Displace platform origin
            incrementedOrigin = [
                initialOrigin[0] + (increment * (targetOrigin[0] - initialOrigin[0])),
                initialOrigin[1] + (increment * (targetOrigin[1] - initialOrigin[1])),
                initialOrigin[2] + (increment * (targetOrigin[2] - initialOrigin[2]))]
            # Rotate platform's basis vectors
            incrementedVectorBase = matmul(
                kf.getRotationMatrixFromAngles(increment * alpha, increment * beta, increment * gamma),
                self.platform.getVectorBase())
            # Create new intermediate target waypoint
            incrementedTarget.append(sc.frame(incrementedOrigin, incrementedVectorBase))

        return incrementedTarget

    def inverseKinematics(self, targetPoint):
        """ Takes in parameters the new target frame the user wants to go to and the actual
            frame (given by the object platform).
            Returns servo angles necessary to accomplish the movement.
            target = [x, y, z, ψ, θ, φ ] (angles d'Euler)
            """
        targetFrame = sc.frame(targetPoint.origin, targetPoint.vectorBase)
        legLengths = self.getEffectiveLegLengths(targetFrame)

        # Compute anchor angles to get effective leg lengths
        servoAngles = []
        for i in range(config.numberOfAnchors):
            servoAngles.append(
                kf.getAlpha(linalg.norm(legLengths[i]), config.betas[i], self.base, targetFrame))
        return servoAngles

    def getEffectiveLegLengths(self, targetFrame):
        """ Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi . """
        base_R_target = self.getRotationMatrixToTarget(targetFrame.vectorBase)
        legLengths = []
        for i in range(config.numberOfAnchors):
            legLengths.append(add(targetFrame.getOrigin(),
                                  subtract(matmul(base_R_target, self.platform.anchors[i]), self.base.anchors[i])))
        return legLengths

    def getRotationAnglesToFrame(self, targetFrame):
        """ Returns the rotation angle between the platform and a target frame. """
        return subtract(
            kf.getAnglesFromRotationMatrix(self.getRotationMatrixToTarget(targetFrame.vectorBase)),
            kf.getAnglesFromRotationMatrix(self.getBasePlatformRotationMatrix()))

    def getBasePlatformRotationMatrix(self):
        """ Returns the rotation matrix between the base and the platform. """
        return kf.getRotationMatrix(self.base.getVectorBase(), self.platform.getVectorBase())

    def getRotationMatrixToTarget(self, targetVectorBase):
        """ Returns the rotation matrix between the base and a target frame. """
        return kf.getRotationMatrix(self.base.getVectorBase(), targetVectorBase)

    def listServoAngles(self, waypoints):
        """ Return a list of six-tuples servo angles to move through the trajectory.
            Returns the last waypoint to later update the platform's origin. """
        lastWaypoint = waypoints[-1]
        listServoAngles = []
        for point in waypoints:
            servoAngles = self.inverseKinematics(point)
            if not (isnan(servoAngles)).any():
                listServoAngles.append(servoAngles)
            else:
                lastWaypoint = point
                break
        return [listServoAngles, lastWaypoint]

    def requestFromFlask(self, type_, data_):
        # Confirm validity of request and check the type of request: "target" or "sweep"
        requestType = self.confirmRequestValidity(type_, data_)
        if requestType == config.unsuccessfulRequest: return config.unsuccessfulRequest # TODO: what to return here?
        # Generate matrix of targets
        listOfTargets = self.generateListOfTargets(requestType, data_)
        # Calculate the servo angle paths
        # Send path to motors
        #

    @staticmethod
    def confirmRequestValidity(type_, data_):
        """ Confirms whether or not the request is valid.
            Returns 0 if type is target, 1 if type is sweep, and -1 if there is a problem with the request. """
        requestType = config.unsuccessfulRequest
        # TODO: create exception if the request is not properly formatted instead of print(...)
        if type_ == "target" and len(data_) == 6 and all(isinstance(n, (int, float)) for n in data):
            # TODO: make sure their types: int, float ?
            requestType = config.targetRequest
        elif type_ == "sweep" and data_ == "x" or "y" or "z" or "a" or "b" or "c":
            requestType = config.sweepRequest
        elif type_ == "target" or "sweep":
            print("Data is wrong or missing!")
        else:
            print("Type error!")
        return requestType

    def generateListOfTargets(self, requestType, data_):
        """ Redirects request depending on its type. """
        return self.targetListForTarget(data_) if requestType == config.targetRequest else self.targetListForSweep(data)

    def targetListForTarget(self, displacements):
        currentPosition = array(self.platform.getOrigin() + self.platform.getAngles())
        return [(currentPosition + array(displacements)).tolist()]

    def targetListForSweep(self, DoF):
        pass

if __name__ == "__main__":
    # Initialize platform
    stewart = stewartPlatform()

    stewart.targetListForTarget([1,1,1,1,1,1])
    stewart.plot()

    # Set target
    targetPosition = [0, 0, stewart.platform.origin[2] - 20]
    # [array([1, 0, 0]), array([0, 1, 0]), array([0, 0, 1])]
    targetOrientation = column_stack([array([1, 0, 0]), array([0, 0, -1]), array([0, 1, 0])])
    target = sc.frame(targetPosition, targetOrientation)

    # Discretize trajectory
    wp = stewart.pathSampling(target)
    [listAngles, endPosition] = stewart.listServoAngles(wp)

    # Update platform current position
    stewart.platform.updateFrame(endPosition)

    stewart.plot()
    print(stewart.platform.getOrigin())
