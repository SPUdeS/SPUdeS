""" Configuration file to define the geometry of the platform.
    This file contain the relevant information to perform the inverse kinematics computations.
    If the platform geometry is changed, then this file should be modified and everything else
    should stay the same in the inverse kinematics file."""

# Constants - distances are in mm
numberOfCorners = 6
pathSamplingPrecision = 0.5
armLength = 16
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