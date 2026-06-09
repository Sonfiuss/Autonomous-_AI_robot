# Module: simulation

## Purpose
Python/Flask web app (runs on laptop). Browser-based path planning simulator for the omni-wheel robot with real-time ZMQ bridge to the Jetson motivation module.

## Key files
| File | Role |
|------|------|
| `movement/app.py` | Flask server + ZMQ RobotBridge |
| `movement/templates/index.html` | Main browser UI |
| `movement/static/js/main.js` | Simulator controller, animation |
| `movement/static/js/robot.js` | Robot hexagon rendering |
| `movement/static/js/kinematics.js` | Forward/inverse kinematics |
| `movement/static/js/pathfinding.js` | A* + Catmull-Rom path smoothing |
| `movement/static/js/obstacles.js` | Circle, rect, parallelogram obstacles |
| `movement/static/js/grid.js` | Coordinate system + grid |

## Run
```bash
cd simulation/movement
pip install -r requirements.txt
JETSON_IP=192.168.x.x python app.py   # with real robot
python app.py                          # simulation only (localhost)
# Browser: http://localhost:5000
```

## RobotBridge (ZMQ inside app.py)
```python
bridge.start()              # connects PUB to :5555, SUB to :5556
bridge.send_goal(x_m, y_m) # publishes "G x y 0.0"
bridge.get_pose()           # → {x, y, theta, ts} from odometry
```
Note: scale conversion — simulation grid uses cm, API converts to meters before sending.

## Browser controls
| Action | Result |
|--------|--------|
| Left click on map | Move robot to target (sends goal via API) |
| Right panel | Add obstacles |
| Reset Position | Return to origin |
| Clear Path | Remove current path |

## Config (env vars)
```bash
JETSON_IP=192.168.1.100   # Jetson WiFi IP
GOAL_PORT=5555             # ZMQ goal port (must match motivation SimBridge)
ODOM_PORT=5556             # ZMQ odom port (must match motivation SimBridge)
```

## Status
- [x] A* pathfinding + path smoothing
- [x] ZMQ bridge to Jetson
- [x] Real-time odometry display
- [x] Obstacle editor
- [ ] 3D map overlay from stereo-camera output
- [ ] Voice command trigger from communication module
