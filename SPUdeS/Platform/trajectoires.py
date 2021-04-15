# Modèle d'input de target
# targetPosition = [0, 0, stewart.platform.origin[config.zPosition]+50]
# targetOrientation = np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])])
# target = sc.frame(targetPosition, targetOrientation)

import numpy as np
from numpy import sqrt, cos, sin, arcsin, arctan2, pi, floor

# Trajectoire côté-côté (test axe x)
# Trajectoire avant-arrière (test axe y)
# Trajectoire haut-bas (test axe z)
positive = 15
negative = 15

positions = []
orientations = []

# Configuration actuelle : haut-bas. Réorganiser colones positions pour autres axes
# Up
for i in range(positive+1):
    positions.append([0, 0, i])
    orientations.append(np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]))
# Down
for i in range(positive+negative+1):
    positions.append([0, 0, positive - i])
    orientations.append(np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]))
# Up
for i in range(negative+1):
    positions.append([0, 0, i - negative])
    orientations.append(np.column_stack([np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]))

# Tilt haut-bas (test Pitch; rotate around X)
positions = []
orientations = []

# Up
for i in range(positive+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([1, 0, 0]), np.array([0, cos(theta_rad), sin(theta_rad)]), np.array([0, -sin(theta_rad), cos(theta_rad)])]))
# Down
for i in range(positive+negative+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([1, 0, 0]), np.array([0, cos(theta_rad), sin(theta_rad)]), np.array([0, -sin(theta_rad), cos(theta_rad)])]))
# Up
for i in range(negative+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([1, 0, 0]), np.array([0, cos(theta_rad), sin(theta_rad)]), np.array([0, -sin(theta_rad), cos(theta_rad)])]))

# Tilt gauche-droite (test Roll; rotate around Y)
# Up
for i in range(positive+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([cos(theta_rad), 0, -sin(theta_rad)]), np.array([0, 1, 0]), np.array([sin(theta_rad), 0, cos(theta_rad)])]))
# Down
for i in range(positive+negative+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([cos(theta_rad), 0, -sin(theta_rad)]), np.array([0, 1, 0]), np.array([sin(theta_rad), 0, cos(theta_rad)])]))
# Up
for i in range(negative+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([cos(theta_rad), 0, -sin(theta_rad)]), np.array([0, 1, 0]), np.array([sin(theta_rad), 0, cos(theta_rad)])]))

# Virage gauche-droite (test Yaw; rotate around Z)
# Up
for i in range(positive+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([cos(theta_rad), -sin(theta_rad), 0]), np.array([sin(theta_rad), cos(theta_rad), 0]), np.array([0, 0, 1])]))
# Down
for i in range(positive+negative+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([cos(theta_rad), -sin(theta_rad), 0]), np.array([sin(theta_rad), cos(theta_rad), 0]), np.array([0, 0, 1])]))
# Up
for i in range(negative+1):
    theta_rad = 2 * pi * (i / 360)
    positions.append([0, 0, 0])
    orientations.append(np.column_stack([np.array([cos(theta_rad), -sin(theta_rad), 0]), np.array([sin(theta_rad), cos(theta_rad), 0]), np.array([0, 0, 1])]))