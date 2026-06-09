# Module: stereo-camera

## Purpose
C++ perception pipeline on Jetson. Two independent sub-modules:
1. **servo** — pan/tilt camera sweep via ESP32 UART
2. **cv** — AI object detection + stereo depth → 2D environment map

---

## Sub-module 1: servo (control/)

Controls ESP32 pan/tilt servos to aim the stereo camera.

| File | Role |
|------|------|
| `control/ServoClient.hpp/.cpp` | UART client — velocity commands, position feedback |

```cpp
servo.connect()
servo.setVelocity(pan_vel, tilt_vel)   // deg/s, bounces at ±70°
servo.getState()                        // → {pan, tilt, timestamp}
```
**Status: complete.**

---

## Sub-module 2: cv (vision/)

### Current state (raw depth → 3D point cloud)
| File | Role |
|------|------|
| `vision/StereoCamera.hpp/.cpp` | Stereo capture + SGBM disparity → CV_32F depth map (metres) |
| `vision/MapBuilder.hpp/.cpp` | Accumulates depth frames at known pan/tilt → 3D point cloud |

Output: PLY / NPY point cloud file. Coordinate system: X=right, Y=up, Z=forward.

### Target state (AI detection → 2D object map)

Full pipeline:
```
Left camera frame
    └─ AI detector (YOLO/TensorRT) → bounding boxes + class labels
              ↓
    Stereo depth map (StereoCamera)
              ↓
    For each detected bbox:
        crop depth region → median depth → 3D position (x, y, z)
              ↓
    Project onto X-Z plane (floor) → 2D object map
              ↓
    Publish via ZMQ → simulation module (obstacle overlay)
```

### New components needed
| Component | Description |
|-----------|-------------|
| `vision/Detector.hpp/.cpp` | AI inference wrapper (TensorRT / OpenCV DNN / YOLO) |
| `vision/ObjectMapper.hpp/.cpp` | Fuses bbox + depth → 3D instance, projects to 2D |
| `vision/Map2D.hpp` | 2D map structure: grid or object list with (x, z, label, confidence) |
| ZMQ publisher | Sends Map2D to simulation for obstacle display |

---

## Files (current)
| File | Role |
|------|------|
| `main.cpp` | Scan loop (servo sweep + depth capture) |
| `tools/stereo_calibrate.py` | Calibration → generates .yml |
| `tools/stream_cameras.py` | Live preview |
| `tools/map3d_server.py` | Browser point cloud viewer |

## Run
```bash
./stereo_scan --port /dev/ttyUSB0 --left 0 --right 1 --output scan.ply
./stereo_scan --calib stereo_calib.yml --duration 60 --hz 5
```

## Status
- [x] Servo pan/tilt UART
- [x] SGBM stereo depth
- [x] 3D point cloud (PLY/NPY)
- [ ] AI object detector integration
- [ ] 3D instance extraction from bbox + depth
- [ ] 2D map output
- [ ] ZMQ publish to simulation
