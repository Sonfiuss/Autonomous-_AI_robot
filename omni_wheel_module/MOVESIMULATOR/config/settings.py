"""Simulation settings for the 3-wheel omni robot."""

import math

# Robot geometry
ROBOT_RADIUS = 0.215  # meters (center to wheel)
WHEEL_RADIUS = 0.043  # meters (8.6 cm diameter)
# Wheel order matches OpenBase kiwi model: [left (240°), back (0°), right (120°)]
WHEEL_ANGLES = [240, 0, 120]  # degrees

# Stepper parameters
STEPS_PER_REV = 200  # full steps per motor rev
GEAR_RATIO = 1.0     # wheel rev per motor rev (1:1)

# Kinematics
MAX_WHEEL_SPEED = 10.0  # rad/s
ACCELERATION = 2.0  # rad/s^2
DECELERATION = 3.0  # rad/s^2

# Simulation
DT = 0.05  # seconds
WINDOW_SIZE = (800, 800)
GRID_SIZE = 10  # meters

# Controls
KEY_MAPPING = {
    "w": 0,       # Wheel A forward
    "a": 1,       # Wheel B forward
    "d": 2,       # Wheel C forward
    "up": 0,      # Wheel A reverse
    "left": 1,    # Wheel B reverse
    "down": 2,    # Wheel C reverse
}
