# Omni-Wheel Movement Simulator

A browser-based simulator for a 3-wheel omni-directional robot with pathfinding and real-time inverse kinematics visualization.


## Features

- Three omni-wheels positioned at 150, 270, and 30 degrees
- A-star pathfinding with obstacle avoidance
- Catmull-Rom spline path smoothing
- Real-time wheel velocity display (V1, V2, V3)
- Multiple obstacle types: circle, rectangle, parallelogram


## Robot Specifications

    Outer radius        19 cm
    Wheel distance (L)  14.4 cm
    Wheel radius (r)    4.1 cm
    V1 angle            150 degrees
    V2 angle            270 degrees
    V3 angle            30 degrees


## Installation

    pip install -r requirements.txt
    python app.py

Open browser at http://localhost:5000


## Controls

    Left click          Move robot to target position
    Right panel         Add obstacles
    Reset Position      Return robot to origin
    Clear Path          Remove current path


## Project Structure

    movement/
        app.py              Flask server
        requirements.txt    Python dependencies
        templates/
            index.html      Main interface
        static/js/
            main.js         Simulator controller and animation
            robot.js        Robot rendering with hexagon body
            kinematics.js   Inverse and forward kinematics
            pathfinding.js  A-star algorithm and path smoothing
            obstacles.js    Obstacle classes and collision detection
            grid.js         Coordinate system and grid rendering


## API Endpoints

GET /api/config

    Returns robot configuration including dimensions and wheel angles.

POST /api/pathfind

    Computes path from start to goal avoiding obstacles.

    Request body:
    {
      "start": {"x": 0, "y": 0},
      "goal": {"x": 50, "y": 50},
      "obstacles": []
    }


## Inverse Kinematics

Wheel velocities are computed from robot velocity using:

    V1 = Vx * cos(150) + Vy * sin(150) + omega * L
    V2 = Vx * cos(270) + Vy * sin(270) + omega * L
    V3 = Vx * cos(30)  + Vy * sin(30)  + omega * L

Where Vx and Vy are linear velocities, omega is angular velocity, and L is wheel distance from center.


## Requirements

    Python 3.10 or higher
    Flask 3.0 or higher
    NumPy
