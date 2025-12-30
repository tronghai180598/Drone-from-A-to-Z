#!/usr/bin/env python3
"""
Plot heart trajectory from uav_position.log file
Shows actual trajectory vs setpoint trajectory
"""

import numpy as np
import matplotlib.pyplot as plt

# Read data from log file
try:
    data = np.loadtxt('uav_position.log', comments='#')
    
    if data.size == 0:
        print("Error: Log file is empty")
        exit(1)
    
    # Extract columns: t X Y Z X_set Y_set Z_set
    t = data[:, 0]
    X = data[:, 1]
    Y = data[:, 2]
    Z = data[:, 3]
    X_set = data[:, 4]
    Y_set = data[:, 5]
    Z_set = data[:, 6]
    
    # Plot 2D trajectory (X-Y plane)
    plt.figure(figsize=(12, 6))
    
    # Subplot 1: X-Y trajectory
    plt.subplot(1, 2, 1)
    plt.plot(X_set, Y_set, linewidth=2, color='blue', label='Setpoint (Heart trajectory)')
    plt.plot(X, Y, linewidth=1.5, color='red', alpha=0.7, label='Actual position')
    plt.grid(True)
    plt.gca().set_aspect("equal", "box")
    plt.xlabel("trục x (m)")
    plt.ylabel("trục y (m)")
    plt.title("Quỹ đạo 2D trái tim (X-Y plane)")
    plt.legend()
    
    # Subplot 2: Z (altitude) over time
    plt.subplot(1, 2, 2)
    plt.plot(t, Z_set, linewidth=2, color='blue', label='Z setpoint')
    plt.plot(t, Z, linewidth=1.5, color='red', alpha=0.7, label='Z actual')
    plt.grid(True)
    plt.xlabel("Thời gian (s)")
    plt.ylabel("Cao độ Z (m)")
    plt.title("Cao độ theo thời gian")
    plt.legend()
    
    plt.tight_layout()
    plt.show()
    
except FileNotFoundError:
    print("Error: File 'uav_position.log' not found")
    print("Please run the simulation first to generate the log file")
except Exception as e:
    print(f"Error reading log file: {e}")

