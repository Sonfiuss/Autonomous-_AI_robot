# Session log — 2026-06-30 — Module usage guides (`readme.txt` x10)

Spans: every top-level module folder. Goal: add a per-module `readme.txt`
hướng dẫn sử dụng (Vietnamese quick-start guide) so each module is usable
without reading source.

---

## What was done

Created `readme.txt` in all 10 module folders. Each follows the same shape:
**chức năng → build/cài đặt → lệnh chạy cụ thể → file chính → ghi chú/cảnh báo.**
Commands were verified against actual entry points / CMake targets / argparse,
not assumed. Where a module already had a fuller `README.md`/`GUIDELINE.md`,
the `readme.txt` is a quick-start that points to it.

| Module | Saving logic — how the readme was derived (source of truth) |
|--------|-------------------------------------------------------------|
| `core-control/` | Usage block in `main.cpp` header + `add_executable(core_control)` in CMake; robot constants (r=5.5, L=21, 12800 steps, α=60/180/300) from `project_overview.md`. |
| `motivation/` | `main.cpp` option list (`--port/--teleop/--demo/--nav/--hz`); CMake builds `motivation`+`stepper_ctrl`; UART + ZMQ tables from `interfaces.md`. Captured shared-ESP32 warning. |
| `deepmap/` | `scan360.py` docstring = the one-command path; manual two-step (`rotate_scan.py`→`build_map.py`) + `scale_calib.py` from `deepmap_plan.md` workflow. Noted legacy/seed-only vs slam. |
| `depth-anything/` | `run_local.py` argparse (`--encoder/--input-size/--device/--half`); util scripts in `src/`; install via `install_jetson.sh`. Marked DA-V2 as shared depth source for deepmap+slam. |
| `area-detection/` | Condensed existing `README.md` pipeline (rectify→disparity→Viterbi boundary→overlay) + `run.py` flags (`--display/--once/--no-rectify`). |
| `manual-control/` | `GUIDELINE.md` hardware table (PCA9685 0x40, pan ch12/tilt ch14, ranges) + `run.sh` build/upload/run path. |
| `simulation/` | `app.py` run (`JETSON_IP=… app.py`, port 5000); Flask API + ZMQ tables from `interfaces.md`. Noted Jetson must run `./motivation --nav`. |
| `slam/` | Condensed `README.md` one-time setup (apt rtabmap, colcon build, env.sh) + `make_intrinsics.py`/`calibrate_scale.py`/`robot_drive.py`; layout from tree. Points to `2026-06-28_rtabmap-slam.md` as source of truth. |
| `stereo-camera/` | Folder tree (vision/control/tools/calib) + ServoClient UART subset from `interfaces.md`; `./stereo_scan` run from `project_overview.md`. Shared-ESP32 warning. |
| `omni_wheel_module/` | Inspected `OpenBase-master/README.md` + `ARD_DRVDM556/` → documented as REFERENCE-only (Gazebo sim + driver docs), with note that real kinematics live in ESP32 firmware, not OpenBase. |

---

## Decisions made
- **`readme.txt` ≠ `README.md`:** kept both. The new `.txt` files are quick
  Vietnamese usage guides; existing `README.md`/`GUIDELINE.md` remain the deep
  docs and are cross-referenced, not duplicated.
- **Language:** Vietnamese, matching how the user communicates.
- **No invented commands:** every run line traces to a real argparse arg,
  CMake target, docstring, or interfaces.md entry.

## Interface changes
- None. Documentation-only session; no code, build, or protocol changes.

## Pending / follow-ups
- Offered (not yet done): a root-level `readme.txt` indexing all modules +
  system start-up order. Awaiting user go-ahead.
- `readme.txt` files are new/untracked — not committed (no commit requested).
