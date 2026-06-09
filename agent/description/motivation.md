# Module: motivation

## Purpose
C++ motion controller running on Jetson. Manages the omni-wheel drive and camera servos via UART to ESP32.

## Key files
| File | Role |
|------|------|
| `main.cpp` | Entry point, 3 run modes: interactive / demo / nav |
| `MotionClient.hpp/.cpp` | UART client — sends commands, receives odometry |
| `SimBridge.hpp/.cpp` | ZMQ bridge — receives goals from simulation, publishes odometry |
| `PoseController.hpp` | Closed-loop pose controller for navigation mode |
| `esp32_unified_controller/` | ESP32 firmware (Arduino) for motor + servo control |

## Run modes
```bash
./motivation --port /dev/ttyUSB0             # interactive (keyboard)
./motivation --port /dev/ttyUSB0 --demo      # preset move sequence
./motivation --port /dev/ttyUSB0 --nav       # receive goals from simulation via ZMQ
./motivation --nav --hz 30                   # nav at 30 Hz control loop
```

## MotionClient API
```cpp
mc.connect()                          // open UART, wait for READY
mc.setVelocity(vx, vy, omega)        // continuous velocity (m/s, m/s, rad/s)
mc.moveForward(dist_m, speed_ms)     // move straight, blocks until K ack
mc.turn(angle_deg, omega_rads)       // rotate, blocks until K ack
mc.setServoVelocity(pan_v, tilt_v)  // camera pan/tilt deg/s
mc.stop()                             // halt all
mc.resetOdometry()                    // zero x/y/theta
mc.getOdometry()                      // → {x, y, theta, timestamp}
mc.setOdomCallback(fn)               // called at 10 Hz
mc.setDoneCallback(fn)               // called on motion complete (K)
```

## Navigation data flow
```
Simulation (laptop)
    └─ ZMQ PUB → port 5555 → SimBridge.start()
                                   └─ GoalCallback → PoseController.compute()
                                                          └─ MotionClient.setVelocity()
                                                                └─ UART → ESP32 → wheels
ESP32 → UART → MotionClient → SimBridge.publishOdom() → ZMQ PUB → port 5556 → Simulation
```

## Status
- [x] UART protocol working
- [x] Odometry feedback at 10 Hz
- [x] Navigation mode with ZMQ goals
- [x] PoseController basic implementation
- [ ] communication module integration (voice commands)
