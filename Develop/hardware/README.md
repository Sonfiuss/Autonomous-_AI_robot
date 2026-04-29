Hardware Control Module
======================

Manages all hardware interaction for the Autonomous AI Robot, including
microcontroller firmware and the Python serial API layer.


Directory Layout
----------------

    hardware/
        firmware/
            arms/                       Robot arm firmware (Arduino)
                platformio.ini
                src/main.cpp
            drive/                      Omni wheel drive firmware (Arduino + DM556)
                platformio.ini
                src/main.cpp
            stm32/                      STM32 sensor hub (CMake project)

        api/
            serial_interface.py         Low-level serial communication
            arm_controller.py           Arm control API
            drive_controller.py         Drive control API
            sensor_reader.py            Read sensor data

        protocols/
            command_protocol.py         Command format sent/received over serial


Communication
-------------

    Transport       Serial (USB) between Python API and Arduino/STM32
    Protocol        Custom text-based commands or JSON payloads


Reference
---------

    Original arm code           ARD_ARMS/
    Original omni wheel code    omni_wheel_module/ARD_DRVDM556/
    Original STM32 code         VS_STM32/
