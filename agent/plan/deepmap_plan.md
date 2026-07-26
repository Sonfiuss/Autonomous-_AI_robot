# Mapping module plan (Astra Pro + Open3D SLAM)

## Current pipeline (2026-07 onward)
Depth source = **Orbbec Astra Pro hardware depth via OpenNI2**. Live map for a
freely-moving robot is built with **Open3D pose-graph SLAM** (ICP odometry +
FPFH loop closure). This replaced the whole DA-V2 mono / manual-stereo /
deepmap-360 line, which has been deleted as dead code.

### Files (depth-anything/src)
| File | Role |
|------|------|
| `astra_cloud.py` | Astra depth front-end: `open_depth_stream`, `read_depth_mm`, `calibrate_intrinsics` (self-calib pinhole from `oniCoordinateConverterDepthToWorld`), `backproject`, `save_ply_xyz`. Depth-only XYZ. |
| `astra_slam.py`  | Live SLAM: live/replay/record sources, `make_pcd` (stride + optional RGB), `icp_odom`, map accumulation, keyframes, offline loop-closure pose-graph, Open3D live window, ESP32 odom stub. |
| `rec/`           | Recorded depth frames (`.npy`, uint16 mm) for offline replay tests. |

### CLI (astra_slam.py)
`--replay DIR` / `--record N DIR` / `--out map.ply` / `--voxel 0.03` /
`--stride 2` / `--max-frames` / `--icp-fitness-min 0.30` / `--icp-rmse-max 0.06` /
`--keyframe-dist 0.3` / `--keyframe-ang 15` / `--loop-fit 0.4` / `--save-every 100` /
`--no-display` / `--odom` / `--rgb [--rgb-index N --rgb-du N --rgb-dv N]`.

## Setup (Windows, verified)
- Driver `obdrv4 v4.3.0.22` + OpenNI2 SDK Windows + `vcredist_x64` (VC++ 2013).
- `pip install openni open3d opencv-python`; run with
  `D:\University\Python\Python310\python.exe`.
- Must `os.add_dll_directory(OPENNI2_REDIST)` before `openni2.initialize` (done
  in astra_cloud). Astra Pro exposes depth (OpenNI2, PID_0403) and colour
  (UVC/cv2, PID_0501) as two separate USB devices; no hardware RGB-D registration.
- Jetson port: OpenNI2 linux/arm64 from the same orbbec/OpenNI_SDK release.

## Status
- astra_cloud: done. astra_slam: implemented (steps 0–8) + live-hardening.
- Live run verified end-to-end after USB replug; ~7–9 FPS at stride 2, voxel 3cm.
- RGB colouring = **mode A (approximate)**: colour sampled at same pixel, tune
  parallax with `--rgb-du/dv`. Mode B (extrinsic calibration) is the upgrade path.

## Pending / next
- [ ] Verify a full closed loop live: walls/floor not doubled in `astra_map.ply`;
      confirm loop-closure pose gap shrinks; then lock FIT_MIN / RMSE_MAX from
      real ICP logs.
- [ ] RGB mode B: depth↔colour extrinsic calibration for pixel-accurate colour.
- [ ] Wire ESP32 wheel odometry into `read_wheel_odom()` (currently identity
      stub) → use as ICP init for robustness during fast rotation.
- [ ] Official successor remains RTAB-Map (ROS2); this Open3D build is the
      intermediate/verification path.
