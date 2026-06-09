---
id: 2026-06-06_stereo-cv-pipeline
status: planning
module: stereo-camera
started: 2026-06-06
---

## Task
Build full CV sub-module: YOLO ONNX detection + stereo depth instance extraction + 2D occupancy grid + ZMQ publisher.

## Input
- Runtime: OpenCV DNN (ONNX) — no TensorRT dependency
- Model: YOLOv5/v8 pretrained COCO 80 classes (.onnx file, user provides path via CLI)
- Depth source: existing StereoCamera (CV_32F depth map, metres)
- Output consumer: simulation module (Python/Flask) via ZMQ
- Existing ZMQ ports: 5555 (goals), 5556 (odometry) — must not conflict
- Coordinate system: X=right, Y=up, Z=forward (matches MapBuilder)

## Expected output
Running `./stereo_scan --detect --model yolo.onnx` on Jetson:
1. Prints detected objects per frame: `[label] conf=0.87 pos=(x=1.20m z=2.40m)`
2. Publishes occupancy grid over ZMQ port 5557 at ~5 Hz
3. Simulation module can subscribe and overlay grid on browser map
4. No crash when no objects detected (empty grid published)

## Plan
- [ ] 1. `vision/Detector.hpp/.cpp` — OpenCV DNN YOLO ONNX wrapper (bbox + label + confidence)
- [ ] 2. `vision/ObjectMapper.hpp/.cpp` — per-bbox depth crop → median depth → 3D (x,y,z) instance
- [ ] 3. `vision/Map2D.hpp/.cpp` — occupancy grid (X-Z plane), binary serialisation for ZMQ
- [ ] 4. `vision/MapPublisher.hpp/.cpp` — ZMQ PUB socket bound to port 5557, publishes Map2D
- [ ] 5. `main.cpp` — add `--detect --model <path>` mode wiring all components together
- [ ] 6. `agent/description/interfaces.md` — document port 5557 message format
- [ ] 7. CMakeLists.txt — add new source files, verify OpenCV DNN linked

## Execution log
<!-- [HH:MM] step N: <action> | risk: <note> | info: <fact> -->

## Test result
<!-- Fill after user confirms -->
