"""Build a room map from Astra depth: stop, look, and tell the tool where the camera stands.

Minimal strategy (task 2026-09-25_vision-occupancy-map): no scan matching, no exploration. Every
capture's pose is dead-reckoned from the moves the operator reports - the same prior a robot leg
(F / T) will provide later. Move the camera by hand, press the key for that move, hold still,
press space.

Run:
  python vision/map_builder.py                                # live; records vision/captures/map_<time>/
  python vision/map_builder.py --replay vision/captures/map_20260925_120000         # rebuild offline
  python vision/map_builder.py --replay DIR --cam-pitch 4                           # ... other mount guess

Keys (live, window focused):
  a / d     the camera turned left (CCW) / right by --rot-step degrees
  w / s     the camera moved forward / back by --move-step meters along its heading
  space     capture here: CAPTURE_FRAMES depth frames + YOLO labels go into the map
  f         camera view: toggle the per-pixel floor / obstacle coloring (on at start)
  m         write the map files now
  q / ESC   write the map files and quit
Map files, in the session folder: map.png, map_grid.npy + map_meta.json, scene.json (v2.0), and per
capture NNN_floor.png - what that capture fed the map (floor green, obstacle red, above the robot
blue, the ignored 5-8 cm band yellow, below the floor magenta). --replay rewrites them with the
mount it was given, so a mount guess can be checked by eye.
"""
import argparse
import json
import logging
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

import depth_source
from detector import DEFAULT_CONF, DEFAULT_IMGSZ, DEFAULT_MODEL, Detection
from drawing import floor_view, render_map, render_view
from floor_geometry import DEFAULT_MOUNT, CameraMount, backproject, camera_to_body, fit_floor, pixel_heights
from frame_grabber import pair_frames
from object_distance import Intrinsics, intrinsics_from_fov, load_intrinsics, measure_object, object_mask
from occupancy_map import DEFAULT_OBSTACLE_MAX_M, OccupancyMap, classify_heights
from scene_export import extract_regions, to_scene

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURES_DIR = os.path.join(BASE_DIR, "captures")
SESSION_FILE = "session.json"
POSES_FILE = "poses.jsonl"
CAPTURE_FRAMES = 3           # depth frames per capture; noise hits one frame, real surfaces all of them
NEXT_FRAME_TIMEOUT_S = 1.0
POINT_STRIDE = 2             # every 2nd row/column (~77k points/frame). 4 left a moire of never-hit
                             # cells on the far floor, where one pixel step spans most of a 5 cm cell
DEFAULT_ROT_STEP_DEG = 45.0  # 58 deg FOV -> ~13 deg overlap between neighbouring views
DEFAULT_MOVE_STEP_M = 0.5
LIVE_FPS = 6.0
MAX_SKEW_S = 0.040
MAX_AGE_S = 0.5
MAP_VIEW_PX = 480
EXPORT_MAP_PX = 900
FLOOR_HEIGHT_TOL_M = 0.02    # floor fit vs configured mount: beyond these, suggest new values
FLOOR_PITCH_TOL_DEG = 1.5
WINDOW = "Map builder"
KEY_ESC, KEY_SPACE = 27, 32

logger = logging.getLogger(__name__)


def integrate_capture(occ_map, depth_frames, detections, intrinsics, mount, pose):
    """Depth frames -> occupancy; detections (on the first frame) -> label votes. Returns the floor
    fit of the first frame, the check on the mount parameters."""
    for depth in depth_frames:
        occ_map.integrate(camera_to_body(backproject(depth, intrinsics, POINT_STRIDE), mount), pose)
    first = depth_frames[0]
    for det in detections:
        rng = measure_object(first, det.box, intrinsics)
        if rng is None:
            continue
        points = backproject(first, intrinsics, mask=object_mask(first, det.box, rng.z_m))
        occ_map.add_label_votes(det.name, det.conf, camera_to_body(points, mount), pose)
    return fit_floor(camera_to_body(backproject(first, intrinsics, POINT_STRIDE), mount), mount)


def floor_image(color_bgr, depth_mm, intrinsics, mount, max_obstacle_height):
    """The color frame painted with what each pixel contributes to the map."""
    return floor_view(color_bgr, classify_heights(pixel_heights(depth_mm, intrinsics, mount), max_obstacle_height))


def _report_floor(fit, mount):
    if fit is None:
        logger.info("floor fit: not enough floor in view")
        return
    logger.info("floor fit: camera height %.3f m (set %.3f), pitch %.1f deg down (set %.1f), %d points",
                fit.height_m, mount.height_m, fit.pitch_deg, mount.pitch_deg, fit.points)
    if abs(fit.height_m - mount.height_m) > FLOOR_HEIGHT_TOL_M or abs(fit.pitch_deg - mount.pitch_deg) > FLOOR_PITCH_TOL_DEG:
        logger.warning("floor fit disagrees with the mount - consider --cam-height %.2f --cam-pitch %.1f",
                       fit.height_m, fit.pitch_deg)


def export(occ_map, poses, out_dir, meta):
    regions = extract_regions(occ_map)
    scene = to_scene(occ_map, regions, poses[-1] if poses else (0.0, 0.0, 0.0))
    with open(os.path.join(out_dir, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(scene, f, indent=2)
    np.save(os.path.join(out_dir, "map_grid.npy"), occ_map.log_odds)
    with open(os.path.join(out_dir, "map_meta.json"), "w", encoding="utf-8") as f:
        json.dump(dict(meta, res_m=occ_map.res, origin=list(occ_map.origin), cells=occ_map.n,
                       grid="log-odds float32 [iy, ix]; row iy grows with world y"), f, indent=2)
    cv2.imwrite(os.path.join(out_dir, "map.png"),
                render_map(occ_map, regions, poses, poses[-1] if poses else None, EXPORT_MAP_PX))
    labels = [o["class"] for o in scene["objects"]]
    logger.info("map: %d objects %s, %d walls, room %.2f x %.2f m -> %s", len(labels), labels,
                len(scene["walls"]), scene["room"]["width"], scene["room"]["length"], out_dir)
    return regions


def _meta(mount, intrinsics, max_obstacle_height):
    return {"mount": mount._asdict(), "intrinsics": intrinsics._asdict(), "max_obstacle_height": max_obstacle_height}


def _resolve_mount(args, saved=None):
    saved = saved or DEFAULT_MOUNT._asdict()
    return CameraMount(args.cam_height if args.cam_height is not None else saved["height_m"],
                       args.cam_pitch if args.cam_pitch is not None else saved["pitch_deg"],
                       args.cam_forward if args.cam_forward is not None else saved["forward_m"])


# ----------------------------------------------------------------------------- replay

def replay(args):
    with open(os.path.join(args.replay, SESSION_FILE), encoding="utf-8") as f:
        saved = json.load(f)
    mount = _resolve_mount(args, saved["mount"])
    max_h = args.max_obstacle_height if args.max_obstacle_height is not None else saved["max_obstacle_height"]
    intrinsics = Intrinsics(**saved["intrinsics"])
    occ_map = OccupancyMap(max_h)
    poses = []
    with open(os.path.join(args.replay, POSES_FILE), encoding="utf-8") as f:
        for line in f:
            rec = json.loads(line)
            frames = [cv2.imread(os.path.join(args.replay, p), cv2.IMREAD_UNCHANGED) for p in rec["depth"]]
            detections = [Detection(d["class_id"], d["name"], d["conf"], tuple(d["box"])) for d in rec["detections"]]
            pose = tuple(rec["pose"])
            fit = integrate_capture(occ_map, frames, detections, intrinsics, mount, pose)
            color = cv2.imread(os.path.join(args.replay, rec["color"]))
            cv2.imwrite(os.path.join(args.replay, f"{rec['index']:03d}_floor.png"),
                        floor_image(color, frames[0], intrinsics, mount, max_h))
            logger.info("capture %d at (%.2f, %.2f, %.0f deg): %d detections", rec["index"], pose[0], pose[1],
                        math.degrees(pose[2]), len(detections))
            _report_floor(fit, mount)
            poses.append(pose)
    export(occ_map, poses, args.replay, _meta(mount, intrinsics, max_h))


# ----------------------------------------------------------------------------- live

def _next_depth_frames(depth_src, after_t, count):
    frames, last_t = [], after_t
    deadline = time.monotonic() + NEXT_FRAME_TIMEOUT_S
    while len(frames) < count and time.monotonic() < deadline:
        frame = depth_src.latest()
        if frame is not None and frame.t > last_t:
            frames.append(frame.data)
            last_t = frame.t
        else:
            time.sleep(0.01)
    return frames


def _record_capture(session_dir, index, pose, color, depth_frames, detections, floor_img):
    names = [f"{index:03d}_depth_{k}.png" for k in range(len(depth_frames))]
    for name, depth in zip(names, depth_frames):
        cv2.imwrite(os.path.join(session_dir, name), depth)   # 16-bit PNG, registered, millimeters
    cv2.imwrite(os.path.join(session_dir, f"{index:03d}_color.png"), color)
    cv2.imwrite(os.path.join(session_dir, f"{index:03d}_floor.png"), floor_img)
    rec = {"index": index, "pose": [round(v, 6) for v in pose], "depth": names, "color": f"{index:03d}_color.png",
           "detections": [dict(d._asdict(), box=list(d.box)) for d in detections]}
    with open(os.path.join(session_dir, POSES_FILE), "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")


def _moved(pose, key, rot_step, move_step):
    x, y, theta = pose
    if key == ord("a"):
        return x, y, theta + rot_step
    if key == ord("d"):
        return x, y, theta - rot_step
    if key in (ord("w"), ord("s")):
        step = move_step if key == ord("w") else -move_step
        return x + step * math.cos(theta), y + step * math.sin(theta), theta
    return pose


def live(args):
    from color_source import ColorSource   # live-only imports: replay needs neither camera nor model
    from depth_source import DepthSource, resolve_redist_path
    from detector import Detector, select_device

    mount = _resolve_mount(args)
    max_h = args.max_obstacle_height if args.max_obstacle_height is not None else DEFAULT_OBSTACLE_MAX_M
    session_dir = os.path.join(CAPTURES_DIR, datetime.now().strftime("map_%Y%m%d_%H%M%S"))
    os.makedirs(session_dir, exist_ok=True)
    redist_path = resolve_redist_path()
    device = select_device(args.device)
    detector = Detector(args.model, device, args.conf, args.imgsz)
    intrinsics = load_intrinsics(args.intrinsics) if args.intrinsics else None
    depth_src, color_src = DepthSource(redist_path), ColorSource(args.color_index)
    depth_src.start()
    color_src.start()
    occ_map = OccupancyMap(max_h)
    pose, poses, regions = (0.0, 0.0, 0.0), [], []
    show_floor = True
    rot_step, move_step = math.radians(args.rot_step), args.move_step
    logger.info("session %s | mount %s | obstacles up to %.2f m", session_dir, mount, max_h)
    period = 1.0 / LIVE_FPS
    clean = True
    try:
        while True:
            t0 = time.monotonic()
            if intrinsics is None and depth_src.fov is not None:
                intrinsics = intrinsics_from_fov(depth_source.WIDTH, depth_source.HEIGHT, *depth_src.fov)
            color = color_src.latest()
            pair, status = pair_frames(color, depth_src.frames(), t0, MAX_SKEW_S, MAX_AGE_S)
            depth, results, painted = None, [], None
            if pair is not None and intrinsics is not None:
                color, depth = pair
                results = [(d, measure_object(depth.data, d.box, intrinsics)) for d in detector.detect(color.data)]
                painted = floor_image(color.data, depth.data, intrinsics, mount, max_h)
            info = f"pose {pose[0]:+.2f} {pose[1]:+.2f} m {math.degrees(pose[2]):+.0f} deg | captures {len(poses)}"
            shown = color._replace(data=painted) if show_floor and painted is not None else color
            view = render_view(shown, depth, results, status, info, False)
            hfov = depth_src.fov[0] if depth_src.fov else None
            cv2.imshow(WINDOW, np.hstack((view, render_map(occ_map, regions, poses, pose, MAP_VIEW_PX, hfov))))
            key = cv2.waitKey(max(int((period - (time.monotonic() - t0)) * 1000), 1)) & 0xFF
            if key in (ord("q"), KEY_ESC) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("a"), ord("d"), ord("w"), ord("s")):
                pose = _moved(pose, key, rot_step, move_step)
            elif key == ord("f"):
                show_floor = not show_floor
            elif key == KEY_SPACE:
                if depth is None:
                    logger.warning("not captured: %s", status)
                    continue
                if not poses:
                    with open(os.path.join(session_dir, SESSION_FILE), "w", encoding="utf-8") as f:
                        json.dump(dict(_meta(mount, intrinsics, max_h), created=datetime.now().isoformat()), f, indent=2)
                detections = [d for d, _ in results]
                frames = [depth.data] + _next_depth_frames(depth_src, depth.t, CAPTURE_FRAMES - 1)
                _record_capture(session_dir, len(poses), pose, color.data, frames, detections, painted)
                _report_floor(integrate_capture(occ_map, frames, detections, intrinsics, mount, pose), mount)
                poses.append(pose)
                regions = extract_regions(occ_map)
                logger.info("capture %d at %s: %d frames, %d detections", len(poses) - 1, info, len(frames), len(detections))
            elif key == ord("m") and poses:
                regions = export(occ_map, poses, session_dir, _meta(mount, intrinsics, max_h))
    except KeyboardInterrupt:
        pass
    finally:
        clean = color_src.close()
        clean = depth_src.close() and clean
        cv2.destroyAllWindows()
        if poses:
            export(occ_map, poses, session_dir, _meta(mount, intrinsics, max_h))
    if not clean:
        logger.warning("exiting hard: a camera thread did not stop")
        logging.shutdown()
        os._exit(0)


def _parse_args():
    parser = argparse.ArgumentParser(description="Room map from Astra depth, stop-and-look captures.")
    parser.add_argument("--replay", metavar="DIR", help="rebuild the map of a recorded session (no camera)")
    parser.add_argument("--cam-height", type=float, help=f"camera height above floor, m (default {DEFAULT_MOUNT.height_m})")
    parser.add_argument("--cam-pitch", type=float, help=f"camera pitch, deg DOWN (default {DEFAULT_MOUNT.pitch_deg})")
    parser.add_argument("--cam-forward", type=float, help="camera offset ahead of the robot center, m (default 0)")
    parser.add_argument("--max-obstacle-height", type=float,
                        help=f"ignore points above this, m (default {DEFAULT_OBSTACLE_MAX_M}; the robot passes under)")
    parser.add_argument("--rot-step", type=float, default=DEFAULT_ROT_STEP_DEG, help="a/d turn, degrees")
    parser.add_argument("--move-step", type=float, default=DEFAULT_MOVE_STEP_M, help="w/s move, meters")
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF)
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ)
    parser.add_argument("--color-index", type=int, default=0)
    parser.add_argument("--intrinsics", help="JSON {fx, fy, cx, cy} of the RGB camera at 640x480")
    return parser.parse_args()


def main():
    args = _parse_args()
    if args.replay:
        replay(args)
    else:
        live(args)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    main()
