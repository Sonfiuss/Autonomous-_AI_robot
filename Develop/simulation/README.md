# Simulation Module

Robot simulation environment with two main components:

- Web Simulators: Browser-based tools for quick testing without complex setup
- Gazebo/ROS2: Full 3D physics simulation environment


## Directory Structure

    simulation/
        web/                        Browser-based simulators
            arm/                    6-DOF robot arm simulator
                index.html          Main interface
                app.js              Three.js rendering and kinematics
                styles.css          Visual styling
            movement/               3-wheel omni robot motion simulator
                app.py              Flask server
                requirements.txt    Python dependencies
                templates/          HTML templates
                static/js/          JavaScript modules

        gazebo/                     Gazebo/ROS2 simulation
            config/                 Parameter files
                robot_params.yaml   Robot dimensions and limits
                gazebo_params.yaml  Physics settings
            launch/                 ROS2 launch files
                robot_sim.launch.py Main launch file
            models/                 Robot models
                robot/              URDF/Xacro definitions
            scripts/                Helper scripts
                spawn_robot.py      Spawn robot into Gazebo
                teleop_sim.py       Keyboard teleoperation
            worlds/                 World definitions
                default.world       Default environment


## Web Simulators

### Arm Simulator

6-DOF robot arm visualization with forward kinematics.

Features:

    - Control 6 joints (J1 through J6) via sliders
    - 3D rendering with Three.js and OrbitControls
    - Preset poses: Home, Ready, Pick, Place

Usage:

    Open web/arm/index.html directly in browser

    Or serve with Python:
        cd web/arm
        python -m http.server 8080
        Open http://localhost:8080


### Movement Simulator

3-wheel omni-directional robot with pathfinding and kinematics visualization.

Features:

    - Omni-wheels at 150, 270, and 30 degrees
    - A-star pathfinding with obstacle avoidance
    - Catmull-Rom path smoothing
    - Real-time wheel velocity display

Robot parameters:

    Outer radius        19 cm
    Wheel distance      14.4 cm
    Wheel radius        4.1 cm

Usage:

    cd web/movement
    pip install -r requirements.txt
    python app.py
    Open http://localhost:5000

Controls:

    Left click          Move robot to position
    Right panel         Add obstacles
    Reset Position      Return to origin
    Clear Path          Remove path display


## Gazebo/ROS2 Simulation

### Requirements

    ROS2 Humble or later
    Gazebo Classic or Ignition/Garden
    Python 3.10 or higher

### Usage

Build workspace:

    colcon build --packages-select robot_simulation

Launch simulation:

    ros2 launch robot_simulation robot_sim.launch.py

Keyboard control (separate terminal):

    python gazebo/scripts/teleop_sim.py


## References

    OpenBase model              omni_wheel_module/OpenBase-master/
    Original MOVESIMULATOR      omni_wheel_module/MOVESIMULATOR/
    Realtime control guide      omni_wheel_module/ARD_DRVDM556/docs/


## Development

Adding a new simulator:

    1. Create folder in web/ for browser-based or gazebo/ for ROS2
    2. Update this README
    3. Add requirements file if needed

Technology stack:

    Arm Simulator       Three.js, JavaScript
    Movement Simulator  Flask, Canvas 2D, A-star
    Gazebo Simulation   ROS2, URDF/Xacro
