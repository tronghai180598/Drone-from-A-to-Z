#!/usr/bin/env python3
"""
Plot heart trajectory for UAV
Based on the heart trajectory equation in control.ino
"""

import numpy as np
import matplotlib.pyplot as plt

# Heart trajectory parameters (from control.ino)
HEART_X0 = 0.0
HEART_Y0 = 0.0
HEART_SCALE = 0.5  # Scale factor for heart size
HEART_PERIOD = 20.0  # Period in seconds (time to complete one heart)

# Generate time array
tau = np.linspace(0, HEART_PERIOD, 1000)  # 1000 points for smooth curve

# Calculate heart trajectory
phi = 2.0 * np.pi * (tau / HEART_PERIOD)

s = np.sin(phi)
c = np.cos(phi)

# Heart equation (same as in control.ino)
x_raw = 16.0 * s * s * s
y_raw = 13.0 * c - 5.0 * np.cos(2.0 * phi) - 2.0 * np.cos(3.0 * phi) - np.cos(4.0 * phi)

# Apply scale and offset
X_set = HEART_X0 + HEART_SCALE * x_raw
Y_set = HEART_Y0 + HEART_SCALE * y_raw

# Plot
plt.figure(figsize=(6, 6))
plt.plot(X_set, Y_set, linewidth=2, color='red')
plt.grid(True)
plt.gca().set_aspect("equal", "box")
plt.xlabel("trục x (m)")
plt.ylabel("trục y (m)")
plt.title("Quỹ đạo 2D trái tim")

# Add start point marker
plt.plot(HEART_X0, HEART_Y0, 'go', markersize=10, label='Start/Home')
plt.legend()

plt.show()

