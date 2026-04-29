UI Module
=========

Web-based control panel for manual operation of the robot.


Directory Layout
----------------

    ui/
        backend/
            app.py                  Main Flask server
            requirements.txt
            api/
                routes.py           REST API endpoint definitions
                websocket.py        WebSocket handler for real-time control

        frontend/
            index.html              Main page
            css/
                style.css
            js/
                app.js              Core application logic
                controller.js       Joystick and gamepad input handling
                telemetry.js        Sensor data display
            assets/                 Images, icons


Features
--------

    Manual Control      Virtual joystick and physical gamepad support
    Camera Feed         Live video stream from the robot camera
    Telemetry           Real-time sensor readouts (distance, IMU, encoder)
    Arm Control         Joint-level control interface for the robot arm
    WebSocket           Low-latency bidirectional communication
