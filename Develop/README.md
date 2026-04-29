Develop - Autonomous AI Robot
=============================

Main development directory for the Autonomous AI Robot project.
Organized by module-based architecture.


Directory Layout
----------------

    Develop/
        ai/             Recognition, analysis, decision making, AI Agents
        hardware/       Firmware (Arduino/STM32) and Python API
        ui/             Web-based manual control interface
        simulation/     Gazebo/ROS2 simulation + Web simulators
        shared/         Shared config, logging, constants


Module Overview
---------------

AI (ai/)

    perception/     Object detection, depth estimation, camera interface
    planning/       Path planning, decision engine
    control/        Translate AI decisions into hardware commands
    agents/         Multi-Agent pipeline (CrewAI) for code conversion, review, verification

Hardware (hardware/)

    firmware/       C++ code for Arduino (arm, drive) and STM32
    api/            Python serial API to communicate with microcontrollers
    protocols/      Command format definitions

UI (ui/)

    backend/        Flask/FastAPI server with REST API and WebSocket
    frontend/       HTML/CSS/JS control panel

Simulation (simulation/)

    web/            Browser-based simulators
        arm/        Robot arm 6-DOF simulator (Three.js)
        movement/   Omni-wheel movement simulator (Flask + Canvas)
    gazebo/         Gazebo/ROS2 simulation
        launch/     ROS2 launch files
        worlds/     Gazebo world definitions
        models/     Robot URDF/SDF models
        config/     Simulation parameters
        scripts/    Helper scripts (spawn, teleop)

Shared (shared/)

    config.py       Centralized configuration (paths, ports, environment)
    logger.py       Unified logging across all modules
    constants.py    Robot dimensions, motion limits, sensor parameters


Requirements
------------

    Python 3.10+
    PlatformIO          (firmware build)
    ROS2 Humble+        (simulation)
    Node.js             (UI frontend, optional)
