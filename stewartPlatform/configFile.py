""" Configuration file to define the geometry of the platform.
    This file contain the relevant information to perform the inverse kinematics computations.
    If the platform geometry is changed, then this file should be modified and everything else
    should stay the same in the inverse kinematics file."""
from numpy import sqrt, array, column_stack
# Constants - distances are in mm
numberOfCorners = 6
pathSamplingPrecision = 0.5
armLength = 25
legLength = 150
baseInteriorRadius = 60
baseExteriorRadius = 90
platformInteriorRadius = 60
platformExteriorRadius = 90
# Betas are the angles formed by each of the servo arms and the x-axis of the platform.
betas = [
    2.61788,
    2.61788,
    -1.5708,
    -1.5708,
    -2.61788,
    -2.61788
]
vectorBase = column_stack([array([1, 0, 0]), array([0, 1, 0]), array([0, 0, 1])])
xPosition = 0
yPosition = 1
zPosition = 2
baseHomePosition = [0, 0, 0]
platformHomePosition = [0, 0, 119.81652640600127]