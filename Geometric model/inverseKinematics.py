import numpy as np
from numpy import sqrt, cos, sin, arcsin, arctan2, pi, floor
import configFile as config
import matplotlib.pyplot as plt
import matplotlib.collections as mc
import stewartPlatform as sp
from stewartPlatform import stewartPlatform


if __name__ == "__main__":
    # Initialize platform
    stewart = stewartPlatform()
    stewart.plot()
    print(stewart.platform.getOrigin())
    # Set target
    targetPosition = [10, 10, stewart.platform.origin[config.zPosition]]
    targetOrientation = np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])])
    target = sp.frame(targetPosition, targetOrientation)

    # Discretize trajectory
    waypoints = stewart.pathSampling(target)

    # Compute set of servo angles to follow trajectory
    for point in waypoints:
        servoAngles = stewart.inverseKinematics(point)
        print(servoAngles)

    # Update platform current position
    stewart.platform.setOrigin(targetPosition)
    stewart.platform.setVectorBase(targetOrientation)

    stewart.plot()
    print(stewart.platform.getOrigin())
