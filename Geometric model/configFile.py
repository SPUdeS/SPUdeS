""" Configuration file to define the geometry of the platform.
    This file contain the relevant information to perform the inverse kinematics computations.
    If the platform geometry is changed, then this file should be modified and everything else
    should stay the same in the inverse kinematics file."""

# Constants - dimensions are in mm
armLength = 16
legLength = 150
radius_int_base = 60
radius_ext_base = 90
radius_int_platform = 60
radius_ext_platform = 90
""" Betas are the angles formed by each of the servo arms and the x-axis of the platform. """
betas = [2.61788,
         2.61788,
         -1.5708,
         -1.5708,
         -2.61788,
         -2.61788]