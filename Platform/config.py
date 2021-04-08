""" Configuration file to define the geometry of the platform.
    This file contain the relevant information to perform the inverse kinematics computations.
    If the platform geometry is changed, then this file should be modified and everything else
    should stay the same in the stewartPlatform file."""
from numpy import array, column_stack, pi
import os
from PythonUI import config as pythonUIConfig

# Constants - Distances are in mm, angles are in radians
numberOfAnchors = 6
pathSamplingPrecision = 0.5
armLength = 25
legLength = 150
baseInteriorRadius = 75
baseExteriorRadius = 90
platformInteriorRadius = 60
platformExteriorRadius = 90
baseOffsetAngle = 0
platformOffsetAngle = pi / 3
stewartVectorBase = column_stack([array([1, 0, 0]), array([0, 1, 0]), array([0, 0, 1])])
stewartHomePosition = [0, 0, 0]
sweepDisplacement = 20

# Flask request logic
targetRequest = 0
sweepRequest = 1
initializationRequest = 2
unsuccessfulRequest = -1
DoFSet = {"x", "y", "z", "a", "b", "c"}

pythonUIPath = pythonUIConfig.pythonUIPath
plot3D = "plot3D.png"
plot3DHome = "plot3DHome.png"
plot3DPath = os.path.join(pythonUIPath, plot3D)
plot3DHomePath = os.path.join(pythonUIPath, plot3DHome)

plotUpView = "plotUpView.png"
plotFrontView = "plotFrontView.png"
plotRightView = "plotRightView.png"
plotUpViewPath = os.path.join(pythonUIPath, plotUpView)
plotFrontViewPath = os.path.join(pythonUIPath, plotFrontView)
plotRightViewPath = os.path.join(pythonUIPath, plotRightView)

plotUpViewHome = "plotUpViewHome.png"
plotFrontViewHome = "plotFrontViewHome.png"
plotRightViewHome = "plotRightViewHome.png"
plotUpViewHomePath = os.path.join(pythonUIPath, plotUpViewHome)
plotFrontViewHomePath = os.path.join(pythonUIPath, plotFrontViewHome)
plotRightViewHomePath = os.path.join(pythonUIPath, plotRightViewHome)

# TODO: Calculate when calling stewartPlatform class
# Betas are the angles formed by each of the servo arms and the x-axis of the platform.
betas = [
    150*(pi/180),
    150*(pi/180),
    -90*(pi/180),
    -90*(pi/180),
    30*(pi/180),
    30*(pi/180)
]
# TODO: Deprecated?
#platformHomePosition = [0, 0, 119.81652640600127]