""" This file contains the stewartPlatform class. This defines the geometry of the stewart platform. """
from numpy import sqrt, arcsin, ones, cos, sin, arctan2, isnan, add, subtract, matmul, column_stack, linalg, array, pi
from SPUdeS.Platform import config
import matplotlib.pyplot as plt
from SPUdeS.Platform import stewartClasses as sc
from SPUdeS.Platform import kinematicFunctions as kf
from mpl_toolkits import mplot3d

class stewartPlatform:
    """ Contains inner classes base and platform. This defines the geometry of the stewart platform. """

    def __init__(self):
        self.base = sc.base()
        self.platform = sc.platform(self.base)
        self.homeServoAngles = self.initHomeServoAngle()
        self.servoAngles = ones(6) * self.homeServoAngles
        self.armPoints = []

    def initHomeServoAngle(self):
        """ This function defines the servo six servo angles at the home position. """
        # Angle of servos at home position
        L0 = 2 * config.armLength ** 2
        M0 = 2 * config.armLength * self.platform.getOrigin()[2]
        N0 = 2 * config.armLength * (self.base.anchors[0][0] - self.platform.anchors[0][0])
        return arcsin(L0 / sqrt(M0 ** 2 + N0 ** 2)) - arctan2(N0, M0)


    def getServoAngles(self):
        return self.servoAngles

    def updateAllPlots(self, updateArmPoints = True):
        """ This functions runs both plot generating function.
        This has the advantage of only updating the arm points once.
        Variable updateArmPoints defaults to true and specifies if the arms are to be recalculated."""
        if updateArmPoints: self.UpdateArmPoints()
        self.displayView()
        self.display3D()

    def displayView(self, updateArmPoints = False):
        """ This function plots three views of the current position of the stewart platform.
            Redefines the three 2D png plots found in the PythonUI module.
            Variable updateArmPoints defaults to false and specifies if the arms are to be recalculated. """

        if updateArmPoints: self.UpdateArmPoints()

        # Get useful point
        [bx, by, bz] = self.base.getPlotAnchors()
        [px, py, pz] = self.platform.getPlotAnchors()
        [ax, ay, az] = self.getArmPoints()

        # UpView
        upView = plt.figure()
        plt.fill(bx, by, 'g')
        plt.fill(px, py, 'r')
        plt.plot([ax, bx], [ay, by], 'b')
        plt.plot([ax, px], [ay, py], 'y')
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Top View")
        plt.axis('equal')
        upView.savefig(config.plotUpViewPath, bbox_inches='tight')

        # FrontView
        frontView = plt.figure()
        plt.plot(bx, bz, 'g')
        plt.plot(px, pz, 'r')
        plt.plot([ax, bx], [az, bz], 'b')
        plt.plot([ax, px], [az, pz], 'y')
        plt.xlabel("x")
        plt.ylabel("z")
        plt.title("Front View")
        plt.axis('equal')
        frontView.savefig(config.plotFrontViewPath, bbox_inches='tight')

        # RightView
        rightView = plt.figure()
        plt.plot(by, bz, 'g')
        plt.plot(py, pz, 'r')
        plt.plot([ay, by], [az, bz], 'b')
        plt.plot([ay, py], [az, pz], 'y')
        plt.xlabel("Y")
        plt.ylabel('Z')
        plt.title("Right View")
        plt.axis('equal')
        rightView.savefig(config.plotRightViewPath, bbox_inches='tight')


    def display3D(self , updateArmPoints = False):
        """ This function plots the main 3D plot of the stewartPlatform
            Redefines the 3D png plot found in the PythonUI module.
            Variable updateArmPoints defaults to false and specifies if the arms are to be recalculated. """

        if updateArmPoints: self.UpdateArmPoints()

        # Get useful point
        baseOrigin = self.base.getOrigin()
        platformOrigin = self.platform.getOrigin()
        [bx, by, bz] = self.base.getPlotAnchors()
        [px, py, pz] = self.platform.getPlotAnchors()
        [ax, ay, az] = self.getArmPoints()

        # create a 3D graph
        plot3D = plt.figure()
        axis = plt.axes(projection='3d')
        axis.set_xlabel('X')
        axis.set_ylabel('Y')
        axis.set_zlabel('Z')

        # Add points to the graph
        axis.scatter(baseOrigin[0], baseOrigin[1], baseOrigin[2], c="green")
        axis.scatter(platformOrigin[0], platformOrigin[1], platformOrigin[2], c="red")
        axis.scatter(ax, ay, az, c="blue")
        axis.scatter(bx, by, bz, c="green")
        axis.scatter(px, py, pz, c="red")

        # Add lines to the graph
        self.drawLinesBetweenPoints(axis, self.platform.getPointsToJoin(), 'r')
        self.drawLinesBetweenPoints(axis, self.base.getPointsToJoin(), 'g')
        self.drawLinesBetweenPoints(axis, self.getArmPointsToJoin(), 'b')
        self.drawLinesBetweenPoints(axis, self.getLegPointsToJoin(), 'y')

        plot3D.savefig(config.plot3DPath, bbox_inches='tight')
        ### Uncomment to update default plot ###
        #plt.savefig(config.plot3DHomePath, bbox_inches='tight')

    def UpdateArmPoints(self):
        """ Initiates useful variables to plot the stewart platform. """
        X = []
        Y = []
        Z = []
        basePoints = self.base.getPlotAnchors()
        armAngle = self.getServoAngles()

        # Calculate points for each motors
        for index in range(config.numberOfAnchors):

            # Calculate theta(angle in the x,y plane)
            if index == 4 or index == 5:
                theta = config.betas[index]
            else:
                theta = pi - config.betas[index]

            # Calculate the displacement
            L = config.armLength * cos(abs(armAngle[index]))
            deltaZ = config.armLength * sin(abs(armAngle[index]))
            deltaX = L * cos(theta)
            deltaY = L * sin(theta)

            # Find the extremity of the arm of the servos
            Z.append(basePoints[2][index] + deltaZ)
            if index == 0 or index == 2 or index == 4:
                Y.append(basePoints[1][index] - deltaY)
            else:
                Y.append(basePoints[1][index] + deltaY)
            if index == 1 or index == 2 or index == 4:
                X.append(basePoints[0][index] - deltaX)
            else:
                X.append(basePoints[0][index] + deltaX)

        self.armPoints = [X, Y, Z]

    def getArmPoints(self):
        return self.armPoints

    def getArmPointsToJoin(self):
        """ Helper function for the plot that defines the points that are to join for the servo arms. """
        armPointsToJoin = []
        [bx, by, bz] = self.base.getPlotAnchors()
        [ax, ay, az] = self.armPoints
        for i in range(config.numberOfAnchors):
            armPointsToJoin.append([(bx[i], by[i], bz[i]), (ax[i], ay[i], az[i])])
        return armPointsToJoin

    def getLegPointsToJoin(self):
        """ Helper function for the plot that defines the points that are to join to the platform. """
        legPointsToJoin = []
        [ax, ay, az] = self.armPoints
        [px, py, pz] = self.platform.getPlotAnchors()
        for i in range(config.numberOfAnchors):
            legPointsToJoin.append([(ax[i], ay[i], az[i]), (px[i], py[i], pz[i])])
        return legPointsToJoin

    @staticmethod
    def drawLinesBetweenPoints(axis, listOfPointsToJoin, colour):
        """ This function takes a plot and a list of points to join as parameters.
            The colour parameter specifies the colour of the line to draw. """
        for points in listOfPointsToJoin:
            axis.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], [points[0][2], points[1][2]], colour)


    def pathSampling(self, targetFrame):
        """ Return a list of waypoints to get to the target.
            Each waypoint is a new frame coinciding with the new platform location
            Uses precision from config file """
        initialOrigin = self.platform.getOrigin()
        targetOrigin = targetFrame.getOrigin()
        numberOfWaypoints = kf.getNumberOfWaypoints(initialOrigin, self.platform.getVectorBase(),
                                                    targetOrigin, targetFrame.getVectorBase())
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

    def inverseKinematics(self, targetFrame):
        """ Takes in parameters the new target frame the user wants to go to and the actual
            frame (given by the object platform).
            Returns servo angles necessary to accomplish the movement.
            target = [x, y, z, ψ, θ, φ ] (angles d'Euler)
            """

        legLengths = self.getEffectiveLegLengths(targetFrame)

        # Compute servo angles to get effective leg lengths
        servoAngles = []
        for i in range(config.numberOfAnchors):
            servoAngles.append(kf.getAlpha(legLengths[i], config.betas[i], self.base, targetFrame))
        return servoAngles

    @staticmethod
    def getTargetFrameFromPoint(targetPoint):
        """ Returns a frame for a target point. """
        origin = [targetPoint[0], targetPoint[1], targetPoint[2]]
        vectorBase = kf.getRotationMatrixFromAngles(targetPoint[3], targetPoint[4], targetPoint[5])
        return sc.frame(origin, vectorBase)


    def getEffectiveLegLengths(self, targetFrame):
        """ Compute effective leg lengths : Effective length = T + b_R_p * Pi - Bi. """
        legLengths = []
        for i in range(config.numberOfAnchors):
            legCoord = add(targetFrame.getOrigin(),
                           subtract(matmul(targetFrame.vectorBase, self.platform.anchors[i]), self.base.anchors[i]))
            legLengths.append(linalg.norm(legCoord))
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

    def getListServoAngles(self, listOfTargets):
        """ Return a list of six-tuples servo angles to move through the trajectory.
            Returns the last waypoint to later update the platform's origin. """
        listServoAngles = []
        for target in listOfTargets:
            targetFrame = self.getTargetFrameFromPoint(target)
            waypoints = self.pathSampling(targetFrame)
            lastWaypoint = waypoints[-1]
            for frame in waypoints:
                servoAngles = self.inverseKinematics(frame)
                if not (isnan(servoAngles)).any():
                    listServoAngles.append(servoAngles)
                else:
                    lastWaypoint = frame
                    break
            self.platform.updateFrame(lastWaypoint)

        if not listServoAngles:
            listServoAngles.append(self.servoAngles)
        self.servoAngles = listServoAngles[-1]
        return listServoAngles

    def requestShowoffFromFlask(self):
        """ This function handles a showoff request from flask. Defines a complex trajectory.
            Returns a list of servo angles. """
        listOfTargets = []
        for DoF in config.DoFSet:
            listOfTargets += self.targetListForSweep(DoF)
        return self.getListServoAngles(listOfTargets)


    def requestFromFlask(self, type_, data_):
        """ This function handles an initialization, displacement or sweep request from flask.
            Returns a list of servo angles. """
        requestType = self.confirmRequestValidity(type_, data_)
        if requestType == config.unsuccessfulRequest: return [self.servoAngles]
        listOfTargets = self.generateListOfTargets(requestType, data_)
        return self.getListServoAngles(listOfTargets)

    @staticmethod
    def confirmRequestValidity(type_, data_):
        """ Confirms whether or not the request is valid.
            Returns 0 if type is target, 1 if type is sweep, and -1 if there is a problem with the request. """
        requestType = config.unsuccessfulRequest
        if type_ == "initialization":
            requestType = config.initializationRequest
        elif type_ == "target" and len(data_) == 6 and all(isinstance(n, (int, float)) for n in data_):
            # TODO: make sure their types: int, float ?
            requestType = config.targetRequest
        elif type_ == "sweep" and data_ in config.DoFSet:
            requestType = config.sweepRequest
        elif type_ == "target" or "sweep":
            print("Data is wrong or missing!")
        else:
            print("Type error!")
        return requestType

    def generateListOfTargets(self, requestType, data_):
        """ Redirects request depending on its type. """
        if requestType == config.targetRequest:
            return self.targetListForTarget(data_)
        elif requestType == config.sweepRequest:
            return self.targetListForSweep(data_)
        elif requestType == config.initializationRequest:
            return self.targetListForInitialize()
        else:
            return self.targetListForInitialize()

    def targetListForInitialize(self):
        """ Defines the list of targets in an initialization request. """
        return [self.platform.initialPosition]

    def targetListForTarget(self, displacements):
        """ Defines the list of targets in a simple displacement request. """
        currentPosition = array(self.platform.getOrigin() + self.platform.getAngles())
        return [(array(currentPosition) + array(displacements)).tolist()]

    def targetListForSweep(self, DoF):
        """ Defines the list of targets in a sweep request. """
        return [
            self.platform.initialPosition,
            add(self.platform.initialPosition, config.displacementDictionary[DoF]).tolist(),
            self.platform.initialPosition,
            subtract(self.platform.initialPosition, config.displacementDictionary[DoF]).tolist(),
            self.platform.initialPosition
        ]



if __name__ == "__main__":
    stewart = stewartPlatform()

