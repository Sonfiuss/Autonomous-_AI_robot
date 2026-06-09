# Module Interfaces

## ZMQ (WiFi — Laptop ↔ Jetson)
| Direction | Port | Socket type | Message format |
|-----------|------|-------------|----------------|
| Simulation → motivation | **5555** | PUB → SUB | `G <x_m> <y_m> <theta_deg>` |
| motivation → Simulation | **5556** | PUB → SUB | `O <x_m> <y_m> <theta_deg>` |

- Jetson **binds** both ports; Laptop **connects** to Jetson IP
- `JETSON_IP=192.168.x.x python simulation/movement/app.py`
- Scale note: simulation uses cm internally, converts to meters before ZMQ send (`CM_TO_M = 0.01`)

## UART (USB serial — Jetson ↔ ESP32)
Both `motivation` and `stereo-camera` use ESP32 over `/dev/ttyUSB0` at 115200 baud.
**They share the same ESP32 unified controller** — do not run both simultaneously on the same port.

### motivation TX → ESP32
```
M <vx> <vy> <omega>\n    velocity (m/s, m/s, rad/s)
F <dist_m> <spd_ms>\n    move forward N metres
T <angle_deg> <rads>\n   turn N degrees
V <pan_v> <tilt_v>\n     camera servo velocity (deg/s)
S\n                       stop all motion
R\n                       reset odometry to 0
```

### motivation RX ← ESP32
```
O <x> <y> <theta_deg>\n  odometry at 10 Hz
P <pan> <tilt>\n          servo angles at 50 Hz
K\n                        motion-complete ack
READY\n                   boot acknowledgement
```

### stereo-camera ServoClient TX/RX (subset)
```
TX: V <pan_vel> <tilt_vel>\n   set servo angular velocity
TX: S\n                         stop servos
TX: R\n                         reset servos to 0°
RX: P <pan> <tilt>\n            position feedback at 50 Hz
RX: READY\n                     boot ack
```

## Simulation Flask API (port 5000)
| Endpoint | Method | Body / Response |
|----------|--------|-----------------|
| `/api/robot/goal` | POST | `{x: cm, y: cm}` → sends ZMQ goal in meters |
| `/api/robot/pose` | GET | `{x_cm, y_cm, theta_deg, connected, ts}` |
| `/api/robot/stop` | POST | Sends `S` over ZMQ |
| `/api/pathfind` | POST | A* path `{start, goal, obstacles}` |
| `/api/config` | GET/POST | Robot/grid/simulation config |

## PoseController (motivation internal)
`compute(current_pose, target_pose)` → `{vx, vy, omega, reached}`
Drives motion loop at configurable Hz (default 20 Hz).
