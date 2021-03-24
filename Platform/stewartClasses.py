import numpy as np
from numpy import sqrt, cos, sin, pi, floor
from Platform import configFile as config
from Platform import kinematicFunctions as kf

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
        self.anchors = None
        self.anchorsX = []
        self.anchorsY = []
        self.anchorsZ = []
        self.pointsToJoin = []

    def setAnchors(self, idx, r_in, r_out):
        """ Returns the position of a corner of an hexagon.
            Hexagon is purely 2D, thus z = 0 in it's frame of reference."""
        angle = (self.offsetAngle + (2 * pi / 3) * floor(idx / 2)) % (2 * pi)
        a_delta = 2 * (2 * r_in - r_out) / sqrt(3)
        x = r_out * cos(angle) + ((-1) ** idx) * (a_delta / 2) * sin(angle)
        y = r_out * sin(angle) - ((-1) ** idx) * (a_delta / 2) * cos(angle)
        return [x, y, 0]

    def initAnchorPositions(self):
        hexagonAnchors = []
        for i in range(config.numberOfAnchors):
            hexagonAnchors.append(self.setAnchors(i, self.interiorRadius, self.exteriorRadius))
        return hexagonAnchors

    def updatePlotAnchors(self):
        cx = []
        cy = []
        cz = []

        # Place plot points for platform
        for i in range(config.numberOfAnchors):
            cx.append(self.anchors[i][config.xPosition])
            cy.append(self.anchors[i][config.yPosition])
            cz.append(self.anchors[i][config.zPosition])
        self.anchorsX = cx
        self.anchorsY = cy
        self.anchorsZ = cz

    def getPlotAnchors(self):
        return [self.anchorsX, self.anchorsY, self.anchorsZ]

    def updatePointsToJoin(self):
        self.pointsToJoin = []
        [bx, by, bz] = self.getPlotAnchors()
        for i in range(config.numberOfAnchors):
            self.pointsToJoin.append([(bx[i], by[i], bz[i]), (bx[(i + 1) % 6], by[(i + 1) % 6], bz[(i + 1) % 6])])

    def getPointsToJoin(self):
        return self.pointsToJoin


class _base(_piece):
    def __init__(self, origin=config.baseHomePosition, vectorBase=config.vectorBase):
        super().__init__(origin, vectorBase)
        self.offsetAngle = 0
        self.interiorRadius = config.platformInteriorRadius
        self.exteriorRadius = config.platformExteriorRadius
        self.anchors = self.initAnchorPositions()
        self.updatePlotAnchors()
        self.updatePointsToJoin()

    def setAnchors(self, idx, r_in, r_out):
        return super(_base, self).setAnchors(idx, r_in, r_out)


class _platform(_piece):
    def __init__(self, linkedBase, origin=config.platformHomePosition, vectorBase=config.vectorBase):
        super().__init__(origin, vectorBase)
        self.linkedBase = linkedBase
        self.offsetAngle = pi / 3
        self.origin = config.platformHomePosition
        self.interiorRadius = config.platformInteriorRadius
        self.exteriorRadius = config.platformExteriorRadius
        self.anchors = self.initAnchorPositions()

    def _getHomePosition(self, linkedBase):
        # deprecated
        # Home position platform - computed from base/platform anchors #1 but any would do by symmetry
        platformHomeZPosition = sqrt(
            config.armLength ** 2 + config.legLength ** 2 - (linkedBase.anchors[0][0] - self.anchors[0][0]) ** 2 - (
                    linkedBase.anchors[0][1] - self.anchors[0][1]) ** 2)
        return [0, 0, platformHomeZPosition]

    def initAnchorPositions(self):
        hexagonAnchors = super(_platform, self).initAnchorPositions()
        platformAnchors = []
        for i in range(config.numberOfAnchors):
            platformAnchors.append(np.add(self.getOrigin(),
                                          np.matmul(kf.getRotationMatrix(self.linkedBase.getVectorBase()[0], self.getVectorBase()[0]),
                                                    hexagonAnchors[i])).tolist()[0])
        return platformAnchors

    def setAnchors(self, idx, r_in, r_out):
        return super(_platform, self).setAnchors(idx, r_in, r_out)