# Claude Action Log

Actions taken beyond the literal request are recorded here for transparency.

## 2026-06-24 — C++ call-graph tracing pipeline (outscope/)

Implemented `outscope/implement-Doxyfile.md`. Created:
- `outscope/Doxyfile` (XML-only Doxygen config, §5.2)
- `outscope/tools/callgraph_trace.py` (XML→DOT trace + 4 break rules, §4/§6)
- `outscope/run_callgraph.sh` + `run_callgraph.bat` (Doxygen→trace→Graphviz pipeline)
- `outscope/README_callgraph.md` (usage — beyond literal request, for handoff)
Notes: `command.txt` left empty (no commands supplied yet). doxygen / graphviz / a
working python are NOT installed on this machine, so the pipeline was authored but
not executed. C: drive is full — shell/PowerShell output capture failing.

## 2026-06-24 — Deep DBM tracing + Shift-JIS robustness (outscope/)

Root cause found: tracer anchored entries in `testtool` and only allowed one hop
into DBM, but the test-tool functions (DBMDownLoadTest.cpp::Refine/OutlierDetection)
are standalone *copies* that only call MedianFilter — so the walk dead-ended and
never reached the real DBM call graph. Fixes:
- `tools/callgraph_trace.py`: replaced `in_testtool` anchor with `defined_in_project`
  (DBM-defined entries preferred over identically-named test-tool copies); raised
  default `MAX_NODES` 8→50; added `--max-nodes` CLI flag.
- `tools/command.txt`: switched to qualified DBM entries (ADCensusStereo::Match,
  MultiStepRefiner::Refine/OutlierDetection).
- Shift-JIS risk (real source has CP932 Japanese files like DBMuim/*): added
  `tools/gen_encoding_map.py` to auto-detect non-UTF-8 sources and emit a
  `doxygen_out/encoding.inc` (`INPUT_FILE_ENCODING = *path=CP932`) @INCLUDE'd by
  `Doxyfile`; set `INPUT_ENCODING = UTF-8`. Wired Step 0 (encoding map) into
  `run_callgraph.sh`/`.bat` before Doxygen.
Verified on local XML: Match → ComputeCost/CostAggregation/ScanlineOptimize/
MultiStepRefine/ComputeDisparity(+Right); Refine → OutlierDetection/IterativeRegionVoting/
ProperInterpolation/DepthDiscontinuityAdjustment/EdgeDetect/MedianFilter. Local repo
has no Shift-JIS files (encoding map empty as expected); map will populate on the
real DBMuim source.

## 2026-06-24 — Install tools + node cap + default-method break rule (outscope/)

Per user request: installed Doxygen 1.17.0 and Graphviz 15.1.0 via winget; added
`C:\Program Files\Graphviz\bin` to the user PATH (beyond literal "install doxygen"
— graphviz needed for the render step). `dot -c` plugin registration failed (needs
admin) but SVG rendering verified working anyway.
Changed break rule 4 from `MAX_DEPTH=25` to `MAX_NODES=8` (kept MAX_DEPTH=50 as a
secondary recursion guard). Added break rule 3 "default methods": new
`outscope/defaultmethod.txt` user list + auto-detection of C++ special members
(ctor `Class::Class`, dtor `~Class`). Updated README tuning section.
Still pending: NO working Python on this Windows box (launcher points at missing
`C:\Python\Python310`), so the trace step cannot run here yet.

## 2026-06-24 — Ran pipeline end-to-end + bug fixes (outscope/)

User: "command run". Installed Python 3.12 (winget, user scope) to unblock the
trace step. Ran the full Doxygen→trace→Graphviz pipeline for command `Refine`.
Fixes made while getting a correct result:
- Entry anchoring: Doxygen records the *declaration* in location/@file (a DBM
  header) and the *definition* in location/@bodyfile (testtool/main.cpp). The
  matcher anchored on @file and found 0 entries. Added `Func.bodyfile` and made
  `in_testtool` check bodyfile (fallback file).
- Match precision: changed `MATCH_MODE` substring→exact. Substring matched
  "Refine" against class "MultiStepRefiner", wrongly picking up OutlierDetection
  as a 2nd entry. Exact mode gives 1 correct entry.
- Run scripts (.sh/.bat): pointed --commands at `testtool/command.txt` (where the
  file actually lives), not `command.txt` in outscope.
Result: `run_callgraph.bat svg` produces callgraph_output/Refine.{dot,svg};
Refine → {DepthDiscontinuityAdjustment→EdgeDetect, IterativeRegionVoting,
OutlierDetection, ProperInterpolation, adcensus_util::MedianFilter}; 7 nodes
(under the 8 cap), no recursion, cross-namespace call resolved correctly via refid.

## 2026-06-23 — Tool chụp bàn cờ stereo + calibrate (CLI)

User yêu cầu tool dùng stereo chụp hình bàn cờ để xác định thông số config camera.
Beyond-request / notable:
- Tạo `stereo-camera/tools/capture_checkerboard.py` và `stereo-camera/run_capture_checkerboard.sh`.
- Phát hiện đã có `tools/stereo_calibrate.py` (chụp+calibrate trong browser, KHÔNG lưu ảnh).
  Theo lựa chọn user: tool mới CLI cv2.imshow, LƯU ảnh gốc + calibrate ngay. Không sửa file cũ.
- Thêm sharpness gate (Laplacian var) loại frame mờ — ngoài yêu cầu, để tăng chất lượng calib.
- Output calib/stereo.yml giữ đúng schema StereoCamera::loadCalibration của tool cũ.

## 2026-06-23 — Drivable-area detection (stereo-area-detection)

User: dùng alignment config → AD-Census depth → drivable area theo repo
sajaysurya/drivable_area_detection, trên ảnh stereo-area-detection/captures.
Beyond-request / notable:
- `pip install hmmlearn` vào Python310 (D:\University\Python\Python310) — dependency repo thiếu.
- Clone repo vào `stereo-area-detection/drivable_area_detection/`.
- Tạo `stereo-area-detection/run_drivable.py` — wrapper 3 stage (align→AD-Census→freespace).
- Quyết định kỹ thuật: align chỉ khử rotation+ty, GIỮ tx (tx=0 trong warp) vì tx
  chính là horizontal disparity = tín hiệu depth; --full-warp để áp cả tx.
- Reimplement thuật toán freespace (không dùng trực tiếp freespace.py vì np.float bị
  gỡ ở numpy 1.24 + popup matplotlib + onehot int64 nổ RAM). Có credit nguồn trong file.
- AD-Census exe cần MinGW OpenCV DLL trên PATH (C:\msys64\mingw64\bin) → inject vào subprocess env.
- AD-Census bad_alloc ở 1280x720 → downscale 640x360 trước khi chạy exe.
- (Sau khi user phản hồi) Phát hiện alignment.yml (210px từ bàn cờ) KHÔNG hợp lệ để
  standardize cặp ảnh phòng. Đo lại trên chính cặp ảnh (analyze_alignment.py, 350 inliers):
  thực tế lệch DỌC 88.5px + xoay 1.13°, ngang chỉ 13.5px. Tạo captures/_room_align.yml
  và chạy lại → disparity sạch hẳn (gradient sàn mượt). Cập nhật memory project-stereo-alignment.
- Sửa run_adcensus dùng os.path.abspath cho mọi path (exe chạy ở cwd khác → path tương đối lỗi).
- (User: "lưu config và dùng hình đã chuẩn hoá để detect freespace") Lưu config chính thức
  → `stereo-area-detection/calib/alignment.yml` (từ _room_align.yml). Đặt làm default --align
  trong run_drivable.py. Chạy lại → freespace trên ảnh đã chuẩn hoá, output captures/drive_*.

## 2026-06-23 — Tool calibrate OFFLINE từ ảnh đã lưu

User có sẵn cặp ảnh bàn cờ (chụp trên màn hình), muốn tool định nghĩa config 2 cam.
Beyond-request / notable:
- Tạo `stereo-camera/tools/calibrate_from_images.py` + `run_calibrate_from_images.sh`.
  Bổ sung cho capture_checkerboard.py (live) bằng đường offline đọc ảnh từ đĩa.
- Cảnh báo user: ảnh hiện có KHÔNG calibrate được (bàn cờ hiện trên màn hình + bị
  cửa sổ trắng che + tràn khung + chỉ 4 cặp + scale 18mm không đúng vì là màn hình).
  Đề nghị chụp lại bằng bàn cờ in giấy. Chưa implement khi chưa có xác nhận (theo
  rule confirm-before-apply); user chọn "build tool + tự chụp lại".
- Thêm ảnh debug detect_*.jpg + rectified.jpg (epipolar) — ngoài yêu cầu, để chẩn đoán.
- Làm rõ "8x8" = 8x8 Ô → 7x7 GÓC TRONG (default tool).

## 2026-06-22 — Port AD-Census to `stereo-camera-AD-Census/`

User asked to vendor ethan-li-coding/AD-Census into a new folder, keep image-file
input, build for Jetson. Beyond-request / notable actions:
- Modified the vendored sources (user said "keep using the solution", but they did
  not build as-is on GCC): converted all `.cpp/.h` from GBK→UTF-8, and added missing
  standard headers (`<cstdio>`, `<cstring>`, `<cmath>`) that MSVC includes
  transitively but GCC does not. Algorithm logic untouched.
- Did NOT vendor upstream `main.cpp` (Windows-only imshow/system("pause")/fopen_s);
  wrote a headless replacement instead.
- Left a temp clone at `/tmp/adcensus_src` (outside repo, can be deleted).

## 2026-06-22 — Arm mask + free-space grid (task arm-mask-freespace)

User approved a 5-step plan to mask the robot arm and add a free-space grid. Beyond
the literal request:
- Installed a system package: `mingw-w64-x86_64-python-opencv` (OpenCV 4.13) into
  MSYS2 via pacman, because NO working Python+OpenCV existed on the box (the base
  `C:\Python\Python310` referenced by the `py` launcher and the `labelimg` venv had
  been removed; `Python313` has no interpreter exe). User chose this option.
  -> Run the wrapper with `C:\msys64\mingw64\bin\python`.
- Changed `run_adcensus()` to CAPTURE the binary's stdout (previously inherited) to
  parse the "Disparity range" line — needed because `_disp.png` is min-max
  normalized, so metric depth is otherwise unrecoverable.
- Verified algorithm sources via `g++ -fsyntax-only` (clean).
- Added one-shot build+run scripts (`run_ad-census.sh`, `build.sh`, `build_windows.bat`)
  and a per-module `captures/` copy of the test pair; default output now lands in
  `stereo-camera-AD-Census/captures/`.
- Removed a stray `-p` folder created by a bad Windows `mkdir -p build`.
- INSTALLED SYSTEM TOOLING (user asked to "install all tools needed"): MSYS2 via
  winget, plus pacman packages mingw-w64-x86_64-{gcc,opencv,pkgconf} + make. Built
  (g++ 16.1.0 + OpenCV 4.13) and ran successfully on Windows — output PNGs produced.
  Result is noisy because the capture pair is unrectified (expected).
- Added `tools/preprocess_run.py` (user-requested): rectify→denoise+CLAHE→adcensus
  →speckle+guided clean→depth-band zoning. Reuses depth_grid.py's uncalibrated
  rectification. Key finding: rectify MUST precede denoise (denoise blurs keypoints
  → residual 17.9px; reversed order → 0.34px). With correct order the depth map is
  coherent (near=red, far=blue) — fixes the "too much noise" complaint.

## 2026-06-21 — Apply depth-grid rectify-fix plan (tools/depth_grid.py)

User asked to apply the plan in `agent/tasks/2026-06-21_depth-grid-rectify-fix.md`.
Implemented all 6 steps. Beyond-request / notable actions:
- Created comparison output images while diagnosing: `captures/depth_grid_norect.jpg`
  (--no-rectify) and `captures/depth_grid_swap.jpg` (swapped L/R) to confirm image
  ordering and quantify the rectify-vs-coverage tradeoff. Can be deleted.
- Created temporary scratch script `/tmp/rtest.py` (outside repo) to tune the RANSAC
  threshold; found 1.0/conf 0.999 fixes the degenerate homography.
- Added a self-validation step NOT in the original plan: reject the rectification and
  fall back to the original pair if the warp does not lower the vertical residual.
- Step 6 target docs (`PROJECT_KNOWLEDGE.md`, `agent/plan/stereo-camera_plan.md`) do
  not exist; recorded the field note in `agent/plan/implement_plan_stereo.md` Phase 3
  instead, the relevant existing doc.

## 2026-06-14 — Stereo point-cloud fusion (C++ port of implement_plan_stereo.md)

User asked to apply `agent/plan/implement_plan_stereo.md` in `stereo-camera/`, save a
session summary, and update `agent/plan/stereo_plan.md`. Decisions (confirmed by user):
keep C++ (not the plan's Python), adopt the plan's convention (mm, X-fwd/Y-right/Z-up),
implement all phases 0–8, review whole diff at the end.

Beyond-request / notable actions:
- **Changed coordinate convention** of MapBuilder from X-right/Y-up/Z-fwd (metres) to
  X-fwd/Y-right/Z-up (mm). Breaking change to cloud orientation — required by the plan.
- **Replaced** the spherical projection in `MapBuilder::addFrame` (the old code did
  `X = r·cosφ·sinθ`, which the plan §3 explicitly forbids) with proper pinhole
  unprojection + rotation via new `core/Geometry`.
- **Fixed three deferred open bugs** while porting: FOV-as-half-FOV (removed entirely),
  no max-depth clamp (added z_min/z_max validity mask), sequential `.read()` stereo skew
  (switched to grab()+retrieve()).
- **Rewrote** the existing `tests/test_map_builder.cpp` (it encoded the retired spherical
  convention and would have failed/compiled against the old API).
- Added `--pan-tiling` CLI flag and rectified-intrinsics plumbing
  (`StereoCamera::applyIntrinsicsTo`) so Reconstruct unprojects with the same K used for
  depth.
- Verified geometry (G1–G6) and sync (S1–S3) suites compile and pass locally with g++.
  Could not build OpenCV/threaded targets on this Windows box (MinGW.org lacks
  `std::mutex`); those must be built + `ctest`-run on the Jetson.

Pending (noted, not done): continuous-sweep firmware feedback wiring (ServoController is
step-and-hold only — does not stream P-feedback into an AngleBuffer); hand-eye calibration
is a documented scaffold (needs checkerboard detections).
