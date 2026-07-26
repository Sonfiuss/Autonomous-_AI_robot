#!/usr/bin/env python3
"""astra_slam.py -- live map building for a freely-moving robot from Astra Pro
depth (OpenNI2) using Open3D pose-graph SLAM (ICP odometry + FPFH loop closure).

INTERMEDIATE / verification build. The official target is RTAB-Map on ROS2
(agent/tasks/2026-06-28_rtabmap-slam.md); this script exists to validate the
depth quality and registration approach cheaply before that investment.

Reuses the depth front-end from astra_cloud.py WITHOUT modifying it:
    open_depth_stream, read_depth_mm, calibrate_intrinsics, backproject,
    save_ply_xyz.

Internal units are METRES (depth_mm / 1000). Default voxel v = 0.03 m.

Frame source is abstracted so the whole algorithm can be tested offline:
    live   : OpenNI2 stream (default)
    replay : --replay DIR  reads frame_*.npy (uint16 mm) recorded earlier
    record : --record N DIR captures N frames to .npy then exits

Jetson port: OpenNI2 linux/arm64 build (see astra_cloud.py header). Open3D and
this script are platform independent.

------------------------------------------------------------------- usage -----
  python astra_slam.py --record 30 rec/            # step 0: static test set
  python astra_slam.py --replay rec/ --no-display  # offline algorithm test
  python astra_slam.py                             # live, window shows growing map
  python astra_slam.py --help                      # all flags
"""
import argparse
import copy
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Reuse the depth front-end (no modification to astra_cloud.py).
from astra_cloud import (
    open_depth_stream,
    read_depth_mm,
    calibrate_intrinsics,
    backproject,
    save_ply_xyz,
)

_DEF_OUT = Path(__file__).resolve().parent.parent / "output" / "astra_map.ply"


# ---------------------------------------------------------------- frame source ---
class LiveSource:
    """OpenNI2 depth stream wrapped as an iterator of uint16 mm frames."""

    WARMUP = 10  # discard first frames: sensor output is unstable right after start
    READ_TIMEOUT = 3.0  # s; if no frame arrives in this long the stream is dead

    def __init__(self, rgb=False, rgb_index=0):
        print("[slam] opening OpenNI2 depth stream ...", flush=True)
        self.openni2, self.depth = open_depth_stream()
        print("[slam] stream open, warming up ...", flush=True)
        for i in range(self.WARMUP):
            if self._grab() is None:
                raise RuntimeError(
                    f"no depth frame within {self.READ_TIMEOUT}s during warmup "
                    f"(frame {i}). Replug the Astra USB and retry.")
        self.first = self._grab()
        if self.first is None:
            raise RuntimeError("no depth frame after warmup -- replug the Astra")
        self.h, self.w = self.first.shape
        self.fx, self.fy, self.cx, self.cy = calibrate_intrinsics(
            self.openni2, self.depth, self.w, self.h)
        self._served_first = False

        # Astra Pro colour is a separate UVC webcam (PID_0501), NOT reachable
        # via OpenNI2. Open it through cv2 alongside the depth stream.
        self.cap = None
        if rgb:
            print(f"[slam] opening colour (cv2 index {rgb_index}) ...", flush=True)
            cap = cv2.VideoCapture(rgb_index, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
            ok, _ = cap.read()
            if ok:
                self.cap = cap
                print("[slam] colour stream OK", flush=True)
            else:
                cap.release()
                print(f"[slam] WARN colour index {rgb_index} gave no frame -- "
                      "running depth-only. Try a different --rgb-index.",
                      flush=True)

    def read_color(self):
        """Latest BGR colour frame (HxWx3 uint8) or None if colour is off."""
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def _grab(self):
        """One timeout-guarded depth read. None if the stream produced nothing.

        wait_for_any_stream stops read_frame() from blocking forever when the
        Astra is slow to start or drops its USB stream."""
        ready = self.openni2.wait_for_any_stream([self.depth],
                                                 timeout=self.READ_TIMEOUT)
        if ready is None:
            return None
        return read_depth_mm(self.depth)

    def intrinsics(self):
        return self.fx, self.fy, self.cx, self.cy

    def read(self):
        if not self._served_first:
            self._served_first = True
            return self.first
        return self._grab()

    def close(self):
        if self.cap is not None:
            self.cap.release()
        self.depth.stop()
        self.openni2.unload()


class ReplaySource:
    """Reads frame_*.npy (uint16 mm) from a directory, in name order.

    Intrinsics are recomputed from OpenNI2 only if available; otherwise the
    Astra-Pro verified defaults are used so replay works without hardware."""

    # Verified Astra Pro 640x480 defaults (astra_cloud self-calib range).
    DEFAULT_INTR = (554.0, 554.0, 320.0, 240.0)

    def __init__(self, directory):
        self.files = sorted(Path(directory).glob("frame_*.npy"))
        if not self.files:
            raise FileNotFoundError(f"no frame_*.npy in {directory}")
        self.i = 0
        probe = np.load(self.files[0])
        self.h, self.w = probe.shape

    def intrinsics(self):
        return self.DEFAULT_INTR

    def read_color(self):
        return None  # recorded frames are depth-only

    def read(self):
        if self.i >= len(self.files):
            return None
        arr = np.load(self.files[self.i]).astype(np.uint16)
        self.i += 1
        return arr

    def close(self):
        pass


# ------------------------------------------------------------------- step 0 ------
def record(n, out_dir):
    """Capture N live depth frames (every ~5th grabbed) to out_dir/frame_*.npy."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    src = LiveSource()
    saved = 0
    grabbed = 0
    print(f"[rec] recording {n} frames to {out} ... move the camera slowly")
    try:
        while saved < n:
            depth = src.read()
            grabbed += 1
            if grabbed % 5:
                continue
            valid_frac = float(np.count_nonzero(depth)) / depth.size
            if valid_frac < 0.60:
                print(f"[rec] WARN frame {saved}: only {valid_frac*100:.0f}% "
                      "valid depth (too close/far or occluded)")
            np.save(out / f"frame_{saved:03d}.npy", depth)
            saved += 1
            print(f"[rec] {saved}/{n}  valid={valid_frac*100:.0f}%")
    finally:
        src.close()
    print(f"[rec] done -> {saved} files in {out}")


# ------------------------------------------------------------- geometry helpers ---
def make_pcd(depth_mm, intr, voxel, with_fpfh=True, stride=1,
             color_bgr=None, rgb_shift=(0, 0)):
    """depth (uint16 mm) -> (pcd_full, pcd_down, fpfh|None). Metres internally.

    `stride` subsamples the depth grid (stride=2 -> ~4x fewer pixels, ~4x
    faster back-projection) BEFORE the 3 cm voxel downsample, so map resolution
    is barely affected. Intrinsics are scaled to keep the metric geometry
    correct: a strided pixel u' maps to original u = u'*stride, which is
    equivalent to using fx/stride, cx/stride (same for y).

    APPROXIMATE colour (mode A): if `color_bgr` is given, each depth pixel is
    coloured by the SAME pixel in the colour image (resized to depth res, then
    shifted by rgb_shift=(du,dv) to hand-correct the depth<->colour parallax).
    No extrinsic calibration -- colours are right around one distance and smear
    at object edges elsewhere. Upgrade path = mode B (calibrated projection)."""
    import open3d as o3d
    fx, fy, cx, cy = intr
    colors = None
    if color_bgr is not None:
        # Align colour to the FULL depth grid, nudge for parallax, then apply
        # the same stride so it lines up 1:1 with the back-projected pixels.
        h, w = depth_mm.shape
        col = cv2.resize(color_bgr, (w, h))
        du, dv = rgb_shift
        if du or dv:
            col = np.roll(col, (dv, du), axis=(0, 1))
        col = col[::stride, ::stride] if stride > 1 else col
    if stride > 1:
        depth_mm = depth_mm[::stride, ::stride]
        fx, fy, cx, cy = fx / stride, fy / stride, cx / stride, cy / stride
    valid = depth_mm > 0
    pts_mm = backproject(depth_mm, fx, fy, cx, cy)      # Nx3 mm, Y up, Z fwd
    pts_m = pts_mm.astype(np.float64) / 1000.0
    full = o3d.geometry.PointCloud()
    full.points = o3d.utility.Vector3dVector(pts_m)
    if color_bgr is not None:
        # backproject keeps raster order of depth>0, so col[valid] matches.
        rgb = col[valid][:, ::-1].astype(np.float64) / 255.0  # BGR->RGB, 0..1
        full.colors = o3d.utility.Vector3dVector(rgb)
    down = full.voxel_down_sample(voxel)
    if len(down.points) < 100:
        # Degenerate frame (lens covered / too close / too far): normals and
        # ICP are meaningless -- caller must skip it.
        return full, down, None
    down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=2 * voxel, max_nn=30))
    down.orient_normals_towards_camera_location(np.zeros(3))
    fpfh = None
    if with_fpfh:
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=5 * voxel, max_nn=100))
    return full, down, fpfh


def save_map_ply(path, pcd):
    """Save a point cloud to PLY. xyz-only via astra_cloud.save_ply_xyz, or
    xyz+rgb (binary LE) when the cloud carries colours."""
    pts = np.asarray(pcd.points, dtype=np.float32)
    if not pcd.has_colors():
        save_ply_xyz(path, pts)
        return len(pts)
    rgb = (np.asarray(pcd.colors) * 255.0).clip(0, 255).astype(np.uint8)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(pts)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    rec = np.empty(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("r", "u1"), ("g", "u1"), ("b", "u1")])
    rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(rec.tobytes())
    return len(pts)


def icp_odom(src_down, tgt_down, init, voxel):
    """Point-to-plane ICP src->tgt. Returns (T_4x4, fitness, rmse)."""
    import open3d as o3d
    res = o3d.pipelines.registration.registration_icp(
        src_down, tgt_down, 1.5 * voxel, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
    return res.transformation, res.fitness, res.inlier_rmse


# ------------------------------------------------------------------- step 7 ------
def read_wheel_odom():
    """Relative pose since last call from ESP32 wheel odometry.

    STUB -- returns identity. TODO: open pyserial to ESP32, parse O/T/F/M/S
    protocol (see agent/description/interfaces.md), integrate wheel velocities
    into a 4x4 relative transform. When wired, this becomes the ICP init so ICP
    only refines -- far more robust during fast rotation."""
    return np.eye(4)


# --------------------------------------------------------------- loop closure ----
def optimize_pose_graph(keyframes, voxel, loop_fit, kf_dist):
    """Build a pose graph from keyframes (odometry + FPFH loop edges) and run
    global optimization. Returns optimized poses list (4x4). keyframes is a list
    of dicts: {pose, down, fpfh}."""
    import open3d as o3d
    reg = o3d.pipelines.registration
    pg = o3d.pipelines.registration.PoseGraph()
    for kf in keyframes:
        pg.nodes.append(o3d.pipelines.registration.PoseGraphNode(
            np.linalg.inv(kf["pose"])))

    # Sequential odometry edges from stored world poses.
    for i in range(len(keyframes) - 1):
        Ti, Tj = keyframes[i]["pose"], keyframes[i + 1]["pose"]
        T_ij = np.linalg.inv(Ti) @ Tj
        pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, i + 1, T_ij, np.eye(6), uncertain=False))

    # Loop-closure edges: spatially close but temporally distant keyframes.
    loops = 0
    for i in range(len(keyframes)):
        for j in range(i + 6, len(keyframes)):
            ti = keyframes[i]["pose"][:3, 3]
            tj = keyframes[j]["pose"][:3, 3]
            if np.linalg.norm(ti - tj) > 2 * kf_dist:
                continue
            ransac = reg.registration_ransac_based_on_feature_matching(
                keyframes[i]["down"], keyframes[j]["down"],
                keyframes[i]["fpfh"], keyframes[j]["fpfh"], True, 1.5 * voxel,
                reg.TransformationEstimationPointToPlane(), 3,
                [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                 reg.CorrespondenceCheckerBasedOnDistance(1.5 * voxel)],
                reg.RANSACConvergenceCriteria(100000, 0.999))
            refine = reg.registration_icp(
                keyframes[i]["down"], keyframes[j]["down"], 1.5 * voxel,
                ransac.transformation,
                reg.TransformationEstimationPointToPlane())
            if refine.fitness < loop_fit:
                continue
            pg.edges.append(o3d.pipelines.registration.PoseGraphEdge(
                i, j, refine.transformation, np.eye(6), uncertain=True))
            loops += 1
    print(f"[loop] added {loops} loop-closure edge(s)")

    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=1.5 * voxel,
        edge_prune_threshold=0.25, reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pg,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
    return [np.linalg.inv(node.pose) for node in pg.nodes]


# ------------------------------------------------------------------- run ---------
def run(args):
    import open3d as o3d

    if args.replay:
        if args.rgb:
            print("[slam] WARN --rgb ignored on replay (recorded frames are "
                  "depth-only)")
            args.rgb = False
        src = ReplaySource(args.replay)
    else:
        src = LiveSource(rgb=args.rgb, rgb_index=args.rgb_index)
    intr = src.intrinsics()
    fx, fy, cx, cy = intr
    print(f"[slam] intrinsics fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

    voxel = args.voxel
    display = not args.no_display
    vis = None
    map_pcd = o3d.geometry.PointCloud()

    T_world = np.eye(4)          # current camera pose in world
    T_rel_prev = np.eye(4)       # constant-velocity init
    prev_down = None
    lost = 0
    keyframes = []
    last_kf_pose = None
    frame_i = 0
    loop = []                    # per-frame loop times for FPS

    view_init = False            # reset the camera once the map has points
    if display:
        vis = o3d.visualization.Visualizer()
        vis.create_window("astra_slam (close window to finish)", 1280, 720)
        # Register the (empty) map geometry ONCE up front. Every frame then just
        # mutates map_pcd's points in place and calls update_geometry -- so the
        # window keeps growing regardless of what the first frame looked like.
        vis.add_geometry(map_pcd, reset_bounding_box=False)

    def pump():
        """Push the current map to the window; return False if user closed it."""
        nonlocal view_init
        if not display:
            return True
        vis.update_geometry(map_pcd)
        if not view_init and len(map_pcd.points) > 0:
            vis.reset_view_point(True)   # frame the cloud the first time
            view_init = True
        alive = vis.poll_events()
        vis.update_renderer()
        return alive

    def maybe_keyframe(down):
        nonlocal last_kf_pose
        if last_kf_pose is None:
            take = True
        else:
            d = np.linalg.norm(T_world[:3, 3] - last_kf_pose[:3, 3])
            R = last_kf_pose[:3, :3].T @ T_world[:3, :3]
            ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
            take = d >= args.keyframe_dist or ang >= args.keyframe_ang
        if take:
            # FPFH is only needed for loop closure -> compute at keyframes only
            # (computing it every frame cost ~half the loop time).
            fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                down,
                o3d.geometry.KDTreeSearchParamHybrid(radius=5 * voxel,
                                                     max_nn=100))
            keyframes.append({"pose": T_world.copy(), "down": down,
                              "fpfh": fpfh})
            last_kf_pose = T_world.copy()
        return take

    step = "init"
    try:
        while True:
            if args.max_frames and frame_i >= args.max_frames:
                break
            step = "read_frame"
            depth = src.read()
            if depth is None:
                if not args.replay:
                    print("[slam] depth stream stopped (timeout/USB drop) -- "
                          "saving map; replug the Astra to run again")
                break
            t0 = time.time()
            step = "read_color"
            color = src.read_color() if args.rgb else None
            step = "make_pcd"
            full, down, _ = make_pcd(depth, intr, voxel, with_fpfh=False,
                                     stride=args.stride, color_bgr=color,
                                     rgb_shift=(args.rgb_du, args.rgb_dv))
            n_down = len(down.points)

            # --- one informative debug line per frame -----------------------
            # status | ICP quality | how far the camera has travelled | how big
            # the map is | keyframes so far. Read `LOST`/`SKIP` streaks to know
            # when to slow down / move away from a surface.
            status, fit, rmse, kf_flag = "OK", 0.0, 0.0, ""

            if n_down < 100:
                # Degenerate frame -- skip entirely, keep previous tracking state.
                lost += 1
                dist = float(np.linalg.norm(T_world[:3, 3]))
                print(f"f{frame_i:<4d} SKIP  empty({n_down}pts) streak={lost}"
                      f" | dist={dist:.2f}m map={len(map_pcd.points)//1000}k"
                      f" kf={len(keyframes)}", flush=True)
                frame_i += 1
                if not pump():
                    print("[slam] window closed")
                    break
                continue

            if prev_down is None:
                # First frame anchors the world.
                step = "anchor_first_frame"
                map_pcd += copy.deepcopy(down)
                if maybe_keyframe(down):
                    kf_flag = " +KF"
                status = "INIT"
            else:
                step = "icp_odom"
                init = read_wheel_odom() if args.odom else T_rel_prev
                T_icp, fit, rmse = icp_odom(down, prev_down, init, voxel)
                if fit < args.icp_fitness_min or rmse > args.icp_rmse_max:
                    lost += 1
                    status = "LOST"
                    if lost >= 5:
                        status = "LOST!"  # sustained -> user should slow down
                else:
                    lost = 0
                    T_rel_prev = T_icp
                    T_world = T_world @ T_icp
                    step = "accumulate_map"
                    map_pcd += copy.deepcopy(down).transform(T_world)
                    step = "keyframe"
                    if maybe_keyframe(down):
                        kf_flag = " +KF"

            dist = float(np.linalg.norm(T_world[:3, 3]))
            warn = "  <-- too close, back off >=1m" if n_down < 2000 else ""
            print(f"f{frame_i:<4d} {status:<5s} pts={n_down:<5d}"
                  f" fit={fit:.2f} rmse={rmse*1000:4.0f}mm"
                  f" | dist={dist:.2f}m map={len(map_pcd.points)//1000}k"
                  f" kf={len(keyframes)}{kf_flag}{warn}", flush=True)
            if status == "LOST!":
                print("[slam] tracking lost >=5 frames -- move slower / add "
                      "features in view", flush=True)

            prev_down = down
            frame_i += 1

            if frame_i % 10 == 0:
                # Downsample IN PLACE: the Visualizer holds a reference to
                # map_pcd, rebinding it would freeze the display.
                step = "voxel_downsample_map"
                ds = map_pcd.voxel_down_sample(voxel)
                map_pcd.points = ds.points
                if map_pcd.has_colors():
                    map_pcd.colors = ds.colors   # keep colours in sync with points
                if len(map_pcd.points) > 3_000_000:
                    print(f"[slam] WARN map {len(map_pcd.points)} pts > 3M cap "
                          "-- raise --voxel")

            # Periodic snapshot: dump the raw (unoptimized) map so far to a
            # numbered PLY next to --out, e.g. astra_map_live_000100.ply.
            if args.save_every and frame_i % args.save_every == 0:
                step = "snapshot"
                out = Path(args.out)
                snap = out.with_name(f"{out.stem}_{frame_i:06d}{out.suffix}")
                n = save_map_ply(snap, map_pcd)
                print(f"[slam] snapshot {n} pts -> {snap}", flush=True)

            step = "render"
            if not pump():
                print("[slam] window closed")
                break

            loop.append(time.time() - t0)
            if len(loop) == 20:
                fps = 20 / sum(loop)
                print(f"[slam] ~{fps:.1f} FPS")
                if fps < 2:
                    print("[slam] WARN <2 FPS -- raise --voxel or render less")
                loop = []
    except Exception:
        # Pinpoint which step broke, dump the traceback, but still fall through
        # to save whatever map was built so far.
        import traceback
        print(f"\n[slam] ERROR at step '{step}' (frame {frame_i}):", flush=True)
        traceback.print_exc()
    finally:
        if vis is not None:
            vis.destroy_window()
        src.close()

    # Step 5: offline loop closure + pose-graph optimization, then rebuild map.
    if len(keyframes) >= 3:
        d_before = float(np.linalg.norm(
            keyframes[0]["pose"][:3, 3] - keyframes[-1]["pose"][:3, 3]))
        try:
            poses = optimize_pose_graph(
                keyframes, voxel, args.loop_fit, args.keyframe_dist)
            d_after = float(np.linalg.norm(poses[0][:3, 3] - poses[-1][:3, 3]))
            print(f"[loop] start-end pose gap {d_before:.3f} -> {d_after:.3f} m")
        except Exception as exc:  # optimization is best-effort for this build
            print(f"[loop] optimization skipped: {exc}")

    n = save_map_ply(args.out, map_pcd)
    print(f"[slam] {n} pts -> {args.out}"
          f"{' (with rgb)' if map_pcd.has_colors() else ''}")


# ------------------------------------------------------------------- main --------
def main():
    p = argparse.ArgumentParser(
        description="Astra Pro depth -> live Open3D pose-graph SLAM map.")
    p.add_argument("--replay", metavar="DIR",
                   help="replay frame_*.npy from DIR instead of live stream")
    p.add_argument("--record", nargs=2, metavar=("N", "DIR"),
                   help="record N live depth frames to DIR then exit")
    p.add_argument("--out", default=str(_DEF_OUT), help="output map .ply")
    p.add_argument("--voxel", type=float, default=0.03,
                   help="voxel size (m) for downsample / map resolution")
    p.add_argument("--stride", type=int, default=2,
                   help="subsample depth grid by this factor for speed "
                        "(2 = ~4x faster, map resolution barely changes)")
    p.add_argument("--max-frames", type=int, default=0,
                   help="stop after N frames (0 = unlimited)")
    p.add_argument("--save-every", type=int, default=100,
                   help="dump a numbered PLY snapshot every N frames "
                        "(0 = only the final map)")
    p.add_argument("--icp-fitness-min", type=float, default=0.30,
                   help="below this ICP fitness = lost tracking, skip frame")
    p.add_argument("--icp-rmse-max", type=float, default=0.06,
                   help="above this ICP rmse (m) = lost tracking, skip frame")
    p.add_argument("--keyframe-dist", type=float, default=0.3,
                   help="new keyframe after moving this many metres")
    p.add_argument("--keyframe-ang", type=float, default=15.0,
                   help="new keyframe after rotating this many degrees")
    p.add_argument("--loop-fit", type=float, default=0.40,
                   help="min ICP fitness to accept a loop-closure edge")
    p.add_argument("--no-display", action="store_true",
                   help="headless: no Open3D window (for offline tests)")
    p.add_argument("--odom", action="store_true",
                   help="use ESP32 wheel odometry as ICP init (stub for now)")
    p.add_argument("--rgb", action="store_true",
                   help="colour the cloud from the Astra UVC camera (mode A, "
                        "approximate: no depth<->colour calibration)")
    p.add_argument("--rgb-index", type=int, default=0,
                   help="cv2 camera index of the Astra colour stream")
    p.add_argument("--rgb-du", type=int, default=0,
                   help="hand-nudge colour horizontally (px) to fix parallax")
    p.add_argument("--rgb-dv", type=int, default=0,
                   help="hand-nudge colour vertically (px) to fix parallax")
    args = p.parse_args()

    if args.record:
        record(int(args.record[0]), args.record[1])
        return
    run(args)


if __name__ == "__main__":
    sys.exit(main())
