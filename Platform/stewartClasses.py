import numpy as np
from numpy import sqrt, cos, sin, pi, floor
from Platform import config
from Platform import kinematicFunctions as kf


class frame:
    """ Position : column vector describing position in Base Frame
        Unit vectors [[Nx],[Ny],[Nz]] where Nx,y,z are column vectors
        representing unitvectors of the frame in the Base reference frame"""

    def __init__(self, origin, vectorBase):
        self.origin = origin
        self.vectorBase = vectorBase

    def getOrigin(self):
        return self.origin

    def getVectorBase(self):
        return self.vectorBase

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

    def getAnchors(self):
        return self.anchors

    def initAnchors(self):
        hexagonAnchors = []
        for i in range(config.numberOfAnchors):
            idx = i + 1 if isinstance(self, _base)  else i
            hexagonAnchors.append(self._initAnchor(idx, self.interiorRadius, self.exteriorRadius))
        return hexagonAnchors

    def _initAnchor(self, idx, r_in, r_out):
        """ Returns the position of a corner of an hexagon.
            Hexagon is purely 2D, thus z = 0 in it's frame of reference.
            Helper function for initAnchors"""
        angle = (self.offsetAngle + (2 * pi / 3) * floor(idx / 2)) % (2 * pi)
        a_delta = 2 * (2 * r_in - r_out) / sqrt(3)
        x = r_out * cos(angle) + ((-1) ** idx) * (a_delta / 2) * sin(angle)
        y = r_out * sin(angle) - ((-1) ** idx) * (a_delta / 2) * cos(angle)
        return [x, y, 0]

    def getPlotAnchors(self):
        anchorX = []
        anchorY = []
        anchorZ = []

        # Place plot points for platform
        for i in range(config.numberOfAnchors):
            anchorX.append(self.anchors[i][0] + self.origin[0])
            anchorY.append(self.anchors[i][1] + self.origin[1])
            anchorZ.append(self.anchors[i][2] + self.origin[2])
        return [anchorX, anchorY, anchorZ]

    def getPointsToJoin(self):
        [bx, by, bz] = self.getPlotAnchors()
        pointsToJoin = []
        for i in range(config.numberOfAnchors):
            pointsToJoin.append([(bx[i], by[i], bz[i]), (bx[(i + 1) % 6], by[(i + 1) % 6], bz[(i + 1) % 6])])
        return pointsToJoin


class _base(_piece):
    def __init__(self, origin=config.stewartHomePosition, vectorBase=config.stewartVectorBase):
        super().__init__(origin, vectorBase)
        self.offsetAngle = config.baseOffsetAngle
        self.interiorRadius = config.baseInteriorRadius
        self.exteriorRadius = config.baseExteriorRadius
        self.anchors = self.initAnchors()


class _platform(_piece):
    def __init__(self, linkedBase, origin=config.stewartHomePosition, vectorBase=config.stewartVectorBase):
        super().__init__(origin, vectorBase)
        self.offsetAngle = config.platformOffsetAngle
        self.interiorRadius = config.platformInteriorRadius
        self.exteriorRadius = config.platformExteriorRadius
        self.anchors = self.initAnchors()
        self.homePosition = self.initHomePosition(linkedBase)
        self.setOrigin(self.homePosition)

    def initHomePosition(self, linkedBase):
        # Home position platform - computed from base/platform anchors #1 but any would do by symmetry
        platformHomeZPosition = sqrt(
            config.armLength ** 2 + config.legLength ** 2 - (linkedBase.anchors[0][0] - self.anchors[0][0]) ** 2 - (
                    linkedBase.anchors[0][1] - self.anchors[0][1]) ** 2)
        return [0, 0, platformHomeZPosition]