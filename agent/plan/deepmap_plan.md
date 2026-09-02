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
| `astra_calib.py` | RGB mode B calibration: `--capture` (paired IR+UVC checkerboard shots, projector MUST be taped over), `--calibrate` (color intrinsics + depth→color R,T in mm) → `astra_calib.npz`. |
| `astra_rgbd.py`  | `align_color_to_depth()` — pixel-accurate color on the depth grid (backproject→R,T→project, cached undistort remap); `--preview` blends aligned color over depth for a visual check. |
| `astra_slam.py`  | Live SLAM: live/replay/record sources, `make_pcd` (stride + RGB mode A/B), `icp_odom` + optional hybrid RGB-D odometry, map accumulation, keyframes, offline loop-closure pose-graph, TSDF fusion, Open3D live window, ESP32 odom stub. |
| `rec/`           | Recorded frames for offline replay (`.npy` depth-only, `.npz` depth+color). |

### CLI (astra_slam.py)
`--replay DIR` / `--record N DIR` / `--out map.ply` / `--voxel 0.03` /
`--stride 2` / `--max-frames` / `--icp-fitness-min 0.30` / `--icp-rmse-max 0.06` /
`--keyframe-dist 0.3` / `--keyframe-ang 15` / `--loop-fit 0.4` / `--save-every 100` /
`--no-display` / `--odom` / `--rgb [--rgb-index N --rgb-du N --rgb-dv N]` /
`--calib astra_calib.npz` (mode B color) / `--rgbd-odom` (image-based motion,
ICP fallback, `MAX_STEP_M=0.3` jump guard) / `--tsdf [--tsdf-voxel 0.015]`
(final map = TSDF fusion at optimized poses; raw accumulate kept as `*_raw.ply`).

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
- 2026-07-26 RGB-D upgrade (task `2026-07-26_astra-rgbd-color-slam`, testing):
  mode B calibration tooling, pixel-accurate colour, hybrid RGB-D odometry
  (motion measured from images — wheel slip cannot corrupt the map), TSDF
  fusion for a clean coloured final map. Offline smoke tests pass (synthetic
  replay: icp-only, mode A rgbd-odom, mode B + tsdf). Live/HW runs pending.
- Coordinate note: RGBD odometry/TSDF run in the y-DOWN camera frame; poses are
  conjugated with `flip_T` (diag(1,-1,1,1)) to match the y-UP cloud frame.

## Pending / next
- [ ] HW: run `astra_calib.py --capture/--calibrate` (tape over the laser
      projector!) → real `astra_calib.npz`, check stereo rms < 0.5 px, verify
      `astra_rgbd.py --preview` alignment. IR stream via OpenNI2 untested on HW.
- [ ] Verify a full closed loop live with `--rgb --calib --rgbd-odom --tsdf`:
      walls/floor not doubled, colours correct, tape-measured 2 m within ~5%;
      then lock FIT_MIN / RMSE_MAX / MAX_STEP_M from real logs.
- [ ] Wire ESP32 wheel odometry into `read_wheel_odom()` (currently identity
      stub) → use as ICP init for robustness during fast rotation.
- [ ] Official successor remains RTAB-Map (ROS2); this Open3D build is the
      intermediate/verification path.
