#!/usr/bin/env python3
"""
stereo_walk_map — straight-line stereo-metric mapping walk (threaded pipeline).

Drive forward in fixed steps; at every stop capture a STEREO pair, run the
stereo-ruler pipeline (DA-V2 + golden points + f*B) for a METRIC depth map,
back-project with the rectified K, place the cloud at the dead-reckoned pose
(z = 0 / 0.3 / 0.6 ...), and merge everything into one PLY.

PIPELINE MODE (default): one thread per stage over pipeline_bus, so compute
overlaps driving and the walk is drive-bound, not compute-bound:

                 ┌───────────► DepthWorker (DA-V2, owns model) ──"mono"──┐
CaptureWorker ───┤  "pair"                                               ├─► FusionMapper
(owns cameras,   └───────────► StereoWorker (rectify+golden) ──"golden"──┘   (join by idx →
 persistent open)                                                             fuse → cloud →
      ▲ "capture_req"                                                         merge+save)
Coordinator (main thread, walk state machine)
      │ "move"                      ▲ "move_done"(+pose)
      ▼                             │
MotionWorker (owns UART via slam/robot_drive, keeps dead-reckoned Pose)

One owner per hardware resource: cameras=CaptureWorker, UART=MotionWorker,
model=DepthWorker. Capture happens only while stationary (Coordinator requests
it strictly between move_done and the next move). Future VoiceWorker /
DetectWorker plug into the same bus without touching the walk logic.

SEQUENTIAL MODE (--sequential): the original one-stage-after-another loop, for
A/B timing — but capturing through the SAME PersistentStereoCam, so the A/B
isolates the threading gain from the persistent-camera gain.

Why stereo scale: the f*B (5.4 cm baseline) puts EVERY frame on the same
real-metre scale by construction. The floor plane is still fitted, but only
for a rotation (level the camera tilt) — never a scale.

Nothing is re-implemented — this chains proven parts:
  stereo_ruler   stereo_half / depth_half / fuse_metric / load_rectify / load_model
  explore_map    Pose dead-reckoning, voxel_dedup, icp_merge_clouds
  slam/robot_drive  calibrated drive_forward(cm)
  merge_360      write_ply
  pipeline_bus   Msg / Bus / Worker / join_all

Usage:
  python3 stereo_walk_map.py --steps 3 --step-cm 30      # hardware, pipeline
  python3 stereo_walk_map.py --steps 3 --sequential      # hardware, A/B base
  python3 stereo_walk_map.py --test [--profile]          # offline, saved pair
Outputs in --out-dir (default deepmap/output/stereo_walk/):
  shot_N_left.jpg / shot_N_right.jpg   raw captures per stop
  depth_N_m.npy                        metric depth (m) per stop
  walk_map.ply                         merged metric cloud (incremental saves too)
"""
import argparse
import os
import queue
import sys
import threading
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, '..', 'depth-anything', 'src')))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, '..', 'slam')))

import stereo_ruler as sr                              # noqa: E402
from depth_to_3d import fit_floor_plane                # noqa: E402
from merge_360 import write_ply                        # noqa: E402
from explore_map import (                              # noqa: E402
    Pose, voxel_dedup, icp_merge_clouds, _rot_to_up,
)
import pipeline_bus as pb                              # noqa: E402
import visual_odom as vo                               # noqa: E402

_DEF_RECTIFY = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'calib', 'stereo_rectify.yml'))
_DEF_TEST_LEFT = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'tools', 'captures', 'left.jpg'))
_DEF_TEST_RIGHT = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'tools', 'captures', 'righ.jpg'))


# ─────────────────────────────────────────────────────────────────────────────
# Geometry — back-project metric depth, level camera tilt, place at world pose
# ─────────────────────────────────────────────────────────────────────────────
def backproject(Z, valid, right_rect, calib, stride, z_min, z_max):
    """Metric depth (right-rectified frame) → camera-frame 3D + colors.

    Camera frame matches explore_map/depth_to_points: X right, Y UP, Z forward
    (image v grows downward, hence the minus on Y).
    Returns (pts Nx3 float32, cols Nx3 uint8 RGB, rows N source image row).
    """
    h, w = Z.shape
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    ys, xs = ys.ravel(), xs.ravel()
    z = Z[ys, xs]
    ok = valid[ys, xs] & (z > z_min) & (z < z_max)
    z, u, v = z[ok], xs[ok], ys[ok]
    x = (u - calib.cx) * z / calib.fx
    y = -(v - calib.cy) * z / calib.fy
    pts = np.stack([x, y, z], axis=1).astype(np.float32)
    cols = right_rect[v, u][:, ::-1].astype(np.uint8)          # BGR → RGB
    return pts, cols, v


def _rot_x_up(tilt_deg):
    """Pitch-up rotation about X undoing a downward camera tilt: the optical
    axis (0,0,1) maps to (0, -sin t, cos t) — i.e. the cloud is raised so the
    camera looks level."""
    t = np.deg2rad(tilt_deg)
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float32)


def level_to_floor(pts, rows, img_h, bottom_frac, tilt_deg=None):
    """Rotate the cloud so the floor is horizontal, floor at Y=0.

    Two stages: FIRST undo the KNOWN downward camera tilt (per-frame estimate
    from stereo golden points — the tilt servo can sit anywhere after boot),
    THEN fit the floor plane for the small residual (roll + estimate error).
    Before, leveling relied on the plane fit ALONE: on frames whose dense
    depth is unreliable the fit is shaky, and when it failed the cloud kept
    the full ~20-30 deg tilt. Now a failed/absurd fit (>20 deg residual, i.e.
    it latched onto furniture, not floor) falls back to the known tilt with
    the floor height taken as the median of the bottom-rows points.

    ROTATION ONLY — the stereo scale is already metric, nothing is scaled.
    Returns (pts, cam_height_m, fit_ok, lvl); lvl = (R, floor_y) so
    visual_odom can move its keypoints into the SAME leveled frame; only when
    no tilt is given AND the fit fails are the points returned unchanged with
    lvl=None.
    """
    floor_mask = rows >= int((1.0 - bottom_frac) * img_h)
    R = np.eye(3, dtype=np.float32)
    p = pts
    if tilt_deg:
        R = _rot_x_up(tilt_deg)
        p = pts @ R.T
    normal, centroid = fit_floor_plane(p, floor_mask)
    resid_deg = (np.degrees(np.arccos(np.clip(abs(normal[1]), -1.0, 1.0)))
                 if normal is not None else None)
    if normal is None or (tilt_deg and resid_deg > 20.0):
        if not tilt_deg:
            return pts, None, False, None
        floor_y = (float(np.median(p[floor_mask, 1])) if floor_mask.any()
                   else 0.0)
        p = p.copy()
        p[:, 1] -= floor_y                 # tilt-only leveling
        return p.astype(np.float32), -floor_y, False, (R, floor_y)
    R2 = _rot_to_up(normal.astype(np.float32))
    p = p @ R2.T
    floor_y = float(centroid.astype(np.float32) @ R2.T[:, 1])  # (centroid@R2.T)[1]
    p[:, 1] -= floor_y                     # floor → Y=0, camera at Y=|floor_y|
    return p.astype(np.float32), -floor_y, True, (R2 @ R, floor_y)


def to_world(pts, pose):
    """Place a camera cloud at the dead-reckoned pose (straight walk: yaw=0)."""
    p = pts.copy()
    if abs(pose.yaw) > 1e-9:
        c, s = np.cos(pose.yaw), np.sin(pose.yaw)
        x, z = p[:, 0].copy(), p[:, 2].copy()
        p[:, 0] = c * x + s * z
        p[:, 2] = -s * x + c * z
    p[:, 0] += pose.x
    p[:, 2] += pose.z
    return p


def _pose_snapshot(pose):
    """Immutable copy for messages — MotionWorker keeps mutating its own."""
    p = Pose()
    p.x, p.z, p.yaw = pose.x, pose.z, pose.yaw
    return p


def _pose_from_xzyaw(xzyaw):
    """visual_odom works on plain (x, z, yaw) tuples — wrap back into Pose."""
    p = Pose()
    p.x, p.z, p.yaw = float(xzyaw[0]), float(xzyaw[1]), float(xzyaw[2])
    return p


def _make_tracker(args, calib):
    return None if args.no_vo else vo.VOTracker(
        calib, z_min=args.z_min, z_max=args.z_max,
        min_inliers=args.vo_min_inliers)


def _ts():
    """Wall-clock prefix for per-stop output lines."""
    return time.strftime('[%H:%M:%S]')


# ─────────────────────────────────────────────────────────────────────────────
# Cameras — persistent open (kills the 3–5 s open/warmup per stop)
# ─────────────────────────────────────────────────────────────────────────────
class PersistentStereoCam:
    """Both cameras opened ONCE and held for the whole walk. Usable without
    the bus (sequential mode captures through it too, so the A/B compares
    threading alone, not threading + persistent cameras)."""

    def __init__(self, dev_left, dev_right, width, height):
        import glob as _glob
        self.cap_l = cv2.VideoCapture(dev_left)
        self.cap_r = cv2.VideoCapture(dev_right)
        for cap in (self.cap_l, self.cap_r):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # minimize stale V4L2 frames
        if not (self.cap_l.isOpened() and self.cap_r.isOpened()):
            which = []
            if not self.cap_l.isOpened():
                which.append(f'left=/dev/video{dev_left}')
            if not self.cap_r.isOpened():
                which.append(f'right=/dev/video{dev_right}')
            present = ', '.join(sorted(_glob.glob('/dev/video*'))) or '(none)'
            self.close()
            raise RuntimeError(
                f"could not open {' and '.join(which)}. This rig needs TWO "
                f"cameras. Present now: {present}. Plug in the missing camera, "
                f"then check 'v4l2-ctl --list-devices' and pass its index via "
                f"--device-left/--device-right.")
        self.dev = (dev_left, dev_right)

    def grab_fresh(self, discard_s=0.2):
        """Drop the frames V4L2 buffered DURING the move (blurry, wrong pose),
        then read one pair. grab() without retrieve() is cheap, so ~discard_s
        seconds of grabbing flushes the driver queue on both cameras."""
        t0 = time.time()
        while time.time() - t0 < discard_s:
            self.cap_l.grab()
            self.cap_r.grab()
        ok_l, left = self.cap_l.read()
        ok_r, right = self.cap_r.read()
        if not (ok_l and ok_r):
            raise RuntimeError(f'frame grab failed on /dev/video{self.dev[0]}'
                               f'//dev/video{self.dev[1]}')
        return left, right

    def close(self):
        for cap in (getattr(self, 'cap_l', None), getattr(self, 'cap_r', None)):
            if cap is not None:
                cap.release()


# ─────────────────────────────────────────────────────────────────────────────
# Workers — one thread per stage, one owner per hardware resource
# ─────────────────────────────────────────────────────────────────────────────
class CaptureWorker(pb.Worker):
    """Owns the cameras. "capture_req"(pose) → fresh pair → "pair"."""

    def __init__(self, bus, in_q, cam, out_dir, discard_s):
        super().__init__('capture', bus, in_q)
        self.cam, self.out_dir, self.discard_s = cam, out_dir, discard_s

    def handle(self, msg):
        left, right = self.cam.grab_fresh(self.discard_s)
        self.bus.publish(pb.Msg('pair', msg.idx, {
            'left': left, 'right': right, 'pose': msg.payload['pose']}))
        cv2.imwrite(os.path.join(self.out_dir, f'shot_{msg.idx}_left.jpg'), left)
        cv2.imwrite(os.path.join(self.out_dir, f'shot_{msg.idx}_right.jpg'), right)

    def cleanup(self):
        self.cam.close()


class FakeCaptureWorker(pb.Worker):
    """--test double: same message interface, saved pair, no cv2.VideoCapture."""

    def __init__(self, bus, in_q, test_left, test_right, out_dir):
        super().__init__('capture', bus, in_q)
        self.left = cv2.imread(test_left)
        self.right = cv2.imread(test_right)
        if self.left is None or self.right is None:
            raise FileNotFoundError(f'--test pair missing: {test_left} / '
                                    f'{test_right}')
        self.out_dir = out_dir

    def handle(self, msg):
        self.bus.publish(pb.Msg('pair', msg.idx, {
            'left': self.left, 'right': self.right,
            'pose': msg.payload['pose']}))
        cv2.imwrite(os.path.join(self.out_dir, f'shot_{msg.idx}_left.jpg'),
                    self.left)
        cv2.imwrite(os.path.join(self.out_dir, f'shot_{msg.idx}_right.jpg'),
                    self.right)


class StereoWorker(pb.Worker):
    """"pair" → rectify + golden points → "golden". Too few points is an
    EXPECTED failure (textureless scene): published with err set so the
    FusionMapper skips that stop and the walk continues."""

    def __init__(self, bus, in_q, calib, inject_badframe=None):
        super().__init__('stereo', bus, in_q)
        self.calib = calib
        self.inject_badframe = inject_badframe

    def handle(self, msg):
        if msg.idx == self.inject_badframe:
            self.bus.publish(pb.Msg('golden', msg.idx, {'pose': msg.payload['pose']},
                                    err='injected <10-golden frame (test)'))
            return
        _, right_rect, golden = sr.stereo_half(
            msg.payload['left'], msg.payload['right'], self.calib)
        err = None
        if len(golden) < 10:
            err = f'only {len(golden)} golden points — need >=10'
        self.bus.publish(pb.Msg('golden', msg.idx, {
            'golden': golden, 'right_rect': right_rect,
            'pose': msg.payload['pose']}, err=err))


class DepthWorker(pb.Worker):
    """"pair" → DA-V2 mono depth (owns the model) → "mono"."""

    def __init__(self, bus, in_q, calib, model, input_size, inject_crash=None):
        super().__init__('depth', bus, in_q)
        self.calib, self.model, self.input_size = calib, model, input_size
        self.inject_crash = inject_crash

    def handle(self, msg):
        if msg.idx == self.inject_crash:
            sys.exit('injected worker crash (test)')   # SystemExit on purpose
        mono, valid = sr.depth_half(msg.payload['right'], self.calib,
                                    self.model, self.input_size)
        self.bus.publish(pb.Msg('mono', msg.idx, {'mono': mono, 'valid': valid}))


class FusionMapper(pb.Worker):
    """Join "golden" + "mono" by idx (arrival order is NOT guaranteed) →
    fuse_metric → cloud → accumulate → incremental save every --save-every.
    A frame that can't be fused is SKIPPED and the walk continues."""

    def __init__(self, bus, in_q, calib, args, out_ply, tracker=None):
        super().__init__('fusion', bus, in_q)
        self.calib, self.args, self.out_ply = calib, args, out_ply
        self.tracker = tracker                # visual_odom.VOTracker or None
        self.pending = {}                     # idx -> {'golden': Msg, 'mono': Msg}
        self.all_pts, self.all_cols = [], []
        self.stops_done, self.skipped = 0, []

    def handle(self, msg):
        slot = self.pending.setdefault(msg.idx, {})
        slot[msg.topic] = msg
        if 'golden' in slot and 'mono' in slot:
            self._process(msg.idx, self.pending.pop(msg.idx))

    def _process(self, idx, slot):
        t0 = time.time()
        g, m = slot['golden'], slot['mono']
        err, Z, ok, info = g.err or m.err, None, None, None
        if not err:
            try:
                a = self.args
                Z, ok, info = sr.fuse_metric(
                    g.payload['golden'], m.payload['mono'],
                    m.payload['valid'], self.calib,
                    cam_h=None if a.no_floor_anchor else a.cam_height_m,
                    tilt_deg=a.cam_tilt_deg)
            except RuntimeError as e:
                err = str(e)
        if err:
            self.skipped.append(idx)
            print(f'{_ts()} stop {idx}: SKIPPED — {err} (walk continues)',
                  flush=True)
            return
        a = self.args
        np.save(os.path.join(a.out_dir, f'depth_{idx}_m.npy'), Z)
        pts, cols, rows = backproject(Z, ok, g.payload['right_rect'],
                                      self.calib, a.stride, a.z_min, a.z_max)
        height_note, lvl = '', None
        if a.level:
            pts, cam_h, lok, lvl = level_to_floor(
                pts, rows, a.height, a.bottom_frac,
                tilt_deg=info.get('tilt_deg', a.cam_tilt_deg))
            height_note = (f'cam_h={cam_h:.2f}m' if lok
                           else f'cam_h={cam_h:.2f}m (tilt-only)' if lvl
                           else 'floor fit FAILED (raw frame)')
        pose = g.payload['pose']
        vo_note = ''
        if self.tracker is not None:
            xzyaw, vo_note = self.tracker.update(
                g.payload['right_rect'], Z, ok, lvl,
                (pose.x, pose.z, pose.yaw))
            pose = _pose_from_xzyaw(xzyaw)
        self.all_pts.append(to_world(pts, pose))
        self.all_cols.append(cols)
        self.stops_done += 1

        zc = Z[ok]
        print(f'{_ts()} stop {idx}: {pose}  golden={info["n_golden"]} '
              f'(inl={info["n_inliers"]}, rms={info["rms"]:.4f}, '
              f'span=x{info["span"]:.1f}, anch={info["n_anchors"]})  '
              f'pts={len(pts)}  Z[{zc.min():.2f},{zc.max():.2f}]m  '
              f'{height_note}  [fused +{time.time() - t0:.1f}s]', flush=True)
        if vo_note:
            print(f'{_ts()} stop {idx}: {vo_note}', flush=True)

        # Incremental PLY on THIS thread's clock: it re-voxelizes ALL points
        # (cost grows with map size) and must never block the join loop above
        # — a bigger --save-every keeps big walks cheap.
        if a.save_every and self.stops_done % a.save_every == 0:
            save_map(self.all_pts, self.all_cols, self.out_ply, a)


class MotionWorker(pb.Worker):
    """Single owner of the ESP32 serial port AND the dead-reckoned Pose.
    "move"(cm) → robot_drive.drive_forward (blocking) → "move_done"(pose).
    A future VoiceWorker publishes "move" to THIS worker — never to the port."""

    def __init__(self, bus, in_q, args):
        super().__init__('motion', bus, in_q)
        import robot_drive                    # slam/ — needs built binaries
        self.drive = robot_drive
        self.args = args
        self.pose = Pose()

    def handle(self, msg):
        if self.stop_event.is_set():          # a fault elsewhere: no NEW move
            return
        cm = msg.payload['cm']
        ok = self.drive.drive_forward(cm, port=self.args.port, hz=self.args.hz)
        if ok:
            time.sleep(self.args.settle)      # let the chassis stop wobbling
            self.pose.advance(cm / 100.0)
        self.bus.publish(pb.Msg('move_done', msg.idx,
                                {'pose': _pose_snapshot(self.pose)},
                                err=None if ok else 'drive_forward failed'))


class FakeMotionWorker(pb.Worker):
    """--test double: 0.5 s per move, same messages, no robot_drive import."""

    def __init__(self, bus, in_q, args):
        super().__init__('motion', bus, in_q)
        self.args = args
        self.pose = Pose()

    def handle(self, msg):
        if self.stop_event.is_set():
            return
        time.sleep(0.5)
        self.pose.advance(msg.payload['cm'] / 100.0)
        self.bus.publish(pb.Msg('move_done', msg.idx,
                                {'pose': _pose_snapshot(self.pose)}))


# ─────────────────────────────────────────────────────────────────────────────
# Args / shared save
# ─────────────────────────────────────────────────────────────────────────────
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Stereo-metric walk map: drive N stops, metric depth per '
                    'stop, merge into one PLY',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    w = p.add_argument_group('walk')
    w.add_argument('--steps', type=int, default=3,
                   help='Number of capture stops (drives happen between them)')
    w.add_argument('--step-cm', type=float, default=30.0,
                   help='Forward distance between stops (cm)')
    w.add_argument('--port', default='/dev/ttyUSB0', help='ESP32 serial port')
    w.add_argument('--hz', type=int, default=3000, help='Stepper step frequency')
    w.add_argument('--settle', type=float, default=0.5,
                   help='Dwell after a move before capture (s)')
    w.add_argument('--sequential', action='store_true',
                   help='Original one-stage-at-a-time loop (A/B baseline); '
                        'still uses the persistent cameras')
    w.add_argument('--profile', action='store_true',
                   help='Per-stage timing table at the end (pipeline mode)')
    w.add_argument('--wait-timeout', type=float, default=180.0,
                   help='Coordinator max wait per pipeline event (s) — a hung '
                        'stage stops the walk instead of freezing it')

    c = p.add_argument_group('cameras / calib (1280x720 = stereo_rectify.yml frame)')
    c.add_argument('--device-left', type=int, default=0)
    c.add_argument('--device-right', type=int, default=2)
    c.add_argument('--width', type=int, default=1280)
    c.add_argument('--height', type=int, default=720)
    c.add_argument('--rectify', default=_DEF_RECTIFY)
    c.add_argument('--discard-s', type=float, default=0.2,
                   help='grab() flush time per capture — drops frames V4L2 '
                        'buffered during the move (stale/blurry)')

    d = p.add_argument_group('depth / cloud')
    d.add_argument('--encoder', default='vits', choices=['vits', 'vitb', 'vitl'])
    d.add_argument('--input-size', type=int, default=518)
    d.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    d.add_argument('--stride', type=int, default=2, help='Pixel subsampling')
    d.add_argument('--z-min', type=float, default=0.2, help='Reject closer (m)')
    d.add_argument('--z-max', type=float, default=5.0, help='Reject farther (m)')
    d.add_argument('--bottom-frac', type=float, default=0.30,
                   help='Bottom image fraction assumed floor (for leveling)')
    d.add_argument('--cam-height-m', type=float, default=0.32,
                   help='Measured camera height above the floor — anchors the '
                        'metric fit far range via floor rows (fix for the '
                        'near-floor-only golden cluster compressing the scene)')
    d.add_argument('--cam-tilt-deg', type=float, default=13.0,
                   help='Fallback downward camera tilt (deg) — normally the '
                        'tilt is estimated per frame from floor golden points '
                        '(the tilt servo can sit anywhere after boot)')
    d.add_argument('--no-floor-anchor', action='store_true',
                   help='Disable floor anchoring; frames whose golden inliers '
                        'span too little depth are then SKIPPED (span guard) '
                        'instead of rescued')
    d.add_argument('--no-level', dest='level', action='store_false', default=True,
                   help='Skip floor leveling (keep raw tilted camera frame)')
    d.add_argument('--no-vo', action='store_true',
                   help='Disable visual odometry — place clouds at the raw '
                        'dead-reckoned pose (VO measures the real travelled '
                        'distance/heading from the images and only falls back '
                        'to dead-reckoning when unconfident)')
    d.add_argument('--vo-min-inliers', type=int, default=12,
                   help='Min RANSAC inlier matches before a VO pose is trusted')

    o = p.add_argument_group('output')
    o.add_argument('--out-dir', default=os.path.join(ROOT, 'output', 'stereo_walk'))
    o.add_argument('--merge', choices=['icp', 'all'], default='icp',
                   help="'icp' refines residual dead-reckon drift on final save")
    o.add_argument('--voxel', type=float, default=0.02,
                   help='Voxel dedup (m) on save; 0 = off')
    o.add_argument('--save-every', type=int, default=1,
                   help='Incremental PLY every K fused stops (0 = final only)')
    o.add_argument('--test', action='store_true',
                   help='Offline: no robot, no cameras — a fixed saved pair is '
                        'reused at every stop (pipeline check; the object WILL '
                        'appear once per stop since the scene never changes)')
    o.add_argument('--test-left', default=_DEF_TEST_LEFT)
    o.add_argument('--test-right', default=_DEF_TEST_RIGHT)
    o.add_argument('--replay', action='store_true',
                   help='Offline: re-run the fuse+map from shot_N_*.jpg saved '
                        'in --out-dir by a previous walk (no robot, no '
                        'cameras; sequential mode only). Dead-reckon poses '
                        'are rebuilt from --step-cm as on the original walk')
    o.add_argument('--inject-crash', type=int, default=None, metavar='IDX',
                   help='(test) DepthWorker raises SystemExit at this stop — '
                        'verifies fault shutdown + partial map save')
    o.add_argument('--inject-badframe', type=int, default=None, metavar='IDX',
                   help='(test) StereoWorker reports <10 golden points at this '
                        'stop — verifies the stop is skipped, walk continues')
    return p.parse_args(argv)


def save_map(all_pts, all_cols, out_path, args, final=False):
    if not all_pts:
        return 0
    if final and args.merge == 'icp':
        pts, cols = icp_merge_clouds(all_pts, all_cols, max(args.voxel, 0.01))
    else:
        pts, cols = np.vstack(all_pts), np.vstack(all_cols)
    pts, cols = voxel_dedup(pts, cols, args.voxel)
    write_ply(out_path, pts, cols)
    return len(pts)


def _final_report(all_pts, all_cols, out_ply, args, pose, t_wall):
    n = save_map(all_pts, all_cols, out_ply, args, final=True)
    if all_pts:
        m = np.vstack(all_pts)
        lo, hi = m.min(axis=0), m.max(axis=0)
        print(f'\nDONE  {len(all_pts)} stops fused  final pose {pose}  '
              f'wall {t_wall:.1f}s')
        print(f'  {n} points -> {out_ply}')
        print(f'  bounds  X[{lo[0]:+.2f},{hi[0]:+.2f}]  '
              f'Y[{lo[1]:+.2f},{hi[1]:+.2f}]  Z[{lo[2]:+.2f},{hi[2]:+.2f}] m')
    else:
        print(f'\nDONE  0 stops fused — nothing to save  wall {t_wall:.1f}s')


# ─────────────────────────────────────────────────────────────────────────────
# Pipeline mode — Coordinator state machine on the main thread
# ─────────────────────────────────────────────────────────────────────────────
def _wait_for(q, idx, stop_event, timeout, what):
    """Block for the Msg with idx on q; None on stop/timeout (never freezes)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if stop_event.is_set():
            return None
        try:
            msg = q.get(timeout=0.2)
        except queue.Empty:
            continue
        if msg is not None and msg.idx == idx:
            return msg
    print(f'  WARNING: {what} (idx {idx}) not seen in {timeout:.0f}s — '
          f'stopping walk', flush=True)
    stop_event.set()
    return None


def _profile_table(workers, t_wall):
    print('\n--profile  (per-step handle() time, ms)')
    timed = [w for w in workers if w.stage_ms]
    all_idx = sorted({i for w in timed for i in w.stage_ms})
    header = '  step  ' + ''.join(f'{w.name:>9}' for w in timed)
    print(header)
    for i in all_idx:
        cells = ''.join(
            f'{w.stage_ms[i]:9.0f}' if i in w.stage_ms else f'{"-":>9}'
            for w in timed)
        print(f'  {i:<6}{cells}')

    print('\n--profile  (per-stage summary)')
    total = 0.0
    for w in timed:
        vals = list(w.stage_ms.values())
        total += sum(vals)
        print(f'  {w.name:<8} n={len(vals)}  avg={np.mean(vals):7.0f} ms  '
              f'total={sum(vals) / 1e3:6.1f} s')
    if total:
        print(f'  wall {t_wall:.1f} s  vs  sum-of-stages {total / 1e3:.1f} s  '
              f'(overlap x{total / 1e3 / max(t_wall, 1e-9):.2f} — >1 means the '
              f'threads worked in parallel; the gap to the ideal also shows '
              f'GIL contention, e.g. golden_points vs fusion numpy)')


def run_pipeline(args, calib, model, out_ply):
    bus = pb.Bus()
    stop = bus.stop_event

    q_capture = bus.subscribe('capture_req')
    q_stereo = bus.subscribe('pair')
    q_depth = bus.subscribe('pair')
    q_pair_ack = bus.subscribe('pair', maxsize=4)      # coordinator's ack copy
    q_fusion = bus.subscribe('golden', maxsize=4)      # ONE queue, two topics:
    bus.subscribe('mono', q=q_fusion)                  # the join pattern
    q_move = bus.subscribe('move')
    q_move_done = bus.subscribe('move_done', maxsize=4)

    if args.test:
        capture = FakeCaptureWorker(bus, q_capture, args.test_left,
                                    args.test_right, args.out_dir)
        motion = FakeMotionWorker(bus, q_move, args)
    else:
        cam = PersistentStereoCam(args.device_left, args.device_right,
                                  args.width, args.height)
        capture = CaptureWorker(bus, q_capture, cam, args.out_dir, args.discard_s)
        motion = MotionWorker(bus, q_move, args)
    stereo = StereoWorker(bus, q_stereo, calib, args.inject_badframe)
    depth = DepthWorker(bus, q_depth, calib, model, args.input_size,
                        args.inject_crash)
    fusion = FusionMapper(bus, q_fusion, calib, args, out_ply,
                          tracker=_make_tracker(args, calib))
    # join_all order = upstream first, so in-flight msgs drain before pills
    workers = [capture, stereo, depth, fusion, motion]

    t0 = time.time()
    pose = _pose_snapshot(motion.pose)
    for w in workers:
        w.start()
    try:
        for i in range(args.steps):
            if stop.is_set():
                break
            bus.publish(pb.Msg('capture_req', i, {'pose': pose}))
            pair = _wait_for(q_pair_ack, i, stop, args.wait_timeout, 'pair')
            if pair is None:
                break
            if (pair.payload['left'].shape[0],
                    pair.payload['left'].shape[1]) != (args.height, args.width):
                raise RuntimeError(
                    f"shot {i} is {pair.payload['left'].shape[1]}x"
                    f"{pair.payload['left'].shape[0]}, calib expects "
                    f'{args.width}x{args.height}')
            if i < args.steps - 1:
                # move as soon as the PAIR lands — depth/stereo/fusion chew on
                # it WHILE the robot drives; that overlap is the whole speedup
                bus.publish(pb.Msg('move', i, {'cm': args.step_cm}))
                done = _wait_for(q_move_done, i, stop, args.wait_timeout,
                                 'move_done')
                if done is None:
                    break
                if done.err:
                    print(f'  WARNING: {done.err} — stopping walk', flush=True)
                    break
                pose = done.payload['pose']
    finally:
        stuck = pb.join_all(workers, stop, timeout=max(args.wait_timeout, 30.0))
        for w in stuck:
            print(f'  WARNING: {w.name} did not join — resources may leak',
                  flush=True)
        t_wall = time.time() - t0
        # all workers joined → fusion's accumulators are final; save the map
        # (partial on any fault) from the coordinator
        _final_report(fusion.all_pts, fusion.all_cols, out_ply, args,
                      motion.pose, t_wall)
        if fusion.skipped:
            print(f'  skipped stops: {fusion.skipped}')
        if args.profile:
            _profile_table(workers, t_wall)
    return out_ply


# ─────────────────────────────────────────────────────────────────────────────
# Sequential mode — original loop, kept for A/B (same persistent cameras)
# ─────────────────────────────────────────────────────────────────────────────
def run_sequential(args, calib, model, out_ply):
    cam = None
    if args.test:
        test_left = cv2.imread(args.test_left)
        test_right = cv2.imread(args.test_right)
        if test_left is None or test_right is None:
            raise FileNotFoundError(f'--test pair missing: {args.test_left} / '
                                    f'{args.test_right}')
    elif args.replay:
        pass                                   # per-stop shots read in the loop
    else:
        cam = PersistentStereoCam(args.device_left, args.device_right,
                                  args.width, args.height)
        import robot_drive                     # slam/ — needs built binaries

    t0_all = time.time()
    pose = Pose()
    tracker = _make_tracker(args, calib)
    all_pts, all_cols = [], []
    try:
        for i in range(args.steps):
            t0 = time.time()
            if args.test:
                left, right = test_left, test_right
            elif args.replay:
                lp = os.path.join(args.out_dir, f'shot_{i}_left.jpg')
                rp = os.path.join(args.out_dir, f'shot_{i}_right.jpg')
                left, right = cv2.imread(lp), cv2.imread(rp)
                if left is None or right is None:
                    raise FileNotFoundError(f'--replay shot missing: {lp} / {rp}')
            else:
                left, right = cam.grab_fresh(args.discard_s)
            if not args.replay:                # in replay the shots ARE the input
                cv2.imwrite(os.path.join(args.out_dir, f'shot_{i}_left.jpg'), left)
                cv2.imwrite(os.path.join(args.out_dir, f'shot_{i}_right.jpg'), right)
            if (left.shape[0], left.shape[1]) != (args.height, args.width):
                raise RuntimeError(f'shot {i} is {left.shape[1]}x{left.shape[0]}'
                                   f', calib expects {args.width}x{args.height}')
            Z, valid, info = sr.compute_metric_depth(
                left, right, calib, model, input_size=args.input_size,
                cam_h=None if args.no_floor_anchor else args.cam_height_m,
                tilt_deg=args.cam_tilt_deg)
            np.save(os.path.join(args.out_dir, f'depth_{i}_m.npy'), Z)

            pts, cols, rows = backproject(Z, valid, info['right_rect'], calib,
                                          args.stride, args.z_min, args.z_max)
            height_note, lvl = '', None
            if args.level:
                pts, cam_h, ok, lvl = level_to_floor(
                    pts, rows, args.height, args.bottom_frac,
                    tilt_deg=info.get('tilt_deg', args.cam_tilt_deg))
                height_note = (f'cam_h={cam_h:.2f}m' if ok
                               else f'cam_h={cam_h:.2f}m (tilt-only)' if lvl
                               else 'floor fit FAILED (raw frame)')
            place_pose, vo_note = pose, ''
            if tracker is not None:
                xzyaw, vo_note = tracker.update(
                    info['right_rect'], Z, valid, lvl,
                    (pose.x, pose.z, pose.yaw))
                place_pose = _pose_from_xzyaw(xzyaw)
            all_pts.append(to_world(pts, place_pose))
            all_cols.append(cols)

            zc = Z[valid]
            print(f'{_ts()} stop {i}: {place_pose}  golden={info["n_golden"]} '
                  f'(inl={info["n_inliers"]}, rms={info["rms"]:.4f}, '
                  f'span=x{info["span"]:.1f}, anch={info["n_anchors"]})  '
                  f'pts={len(pts)}  Z[{zc.min():.2f},{zc.max():.2f}]m  '
                  f'{height_note}  [{time.time() - t0:.1f}s]', flush=True)
            if vo_note:
                print(f'{_ts()} stop {i}: {vo_note}', flush=True)

            if args.save_every and (i + 1) % args.save_every == 0:
                save_map(all_pts, all_cols, out_ply, args)   # incremental

            if i < args.steps - 1:
                if args.replay:
                    pass                       # commanded pose only — no robot
                elif args.test:
                    print(f'  [test] skip drive_forward({args.step_cm:g}) '
                          f'(+0.5s like the fake worker)', flush=True)
                    time.sleep(0.5)
                else:
                    if not robot_drive.drive_forward(args.step_cm,
                                                     port=args.port, hz=args.hz):
                        print('  WARNING: forward step failed — stopping walk')
                        break
                    time.sleep(args.settle)
                pose.advance(args.step_cm / 100.0)
    finally:
        if cam is not None:
            cam.close()
        _final_report(all_pts, all_cols, out_ply, args, pose,
                      time.time() - t0_all)
    return out_ply


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    out_ply = os.path.join(args.out_dir, 'walk_map.ply')

    if args.replay:
        args.sequential = True                 # replay is sequential-only
    calib = sr.load_rectify(args.rectify, (args.width, args.height))
    model, device = sr.load_model(args.encoder, args.device)
    mode = 'SEQUENTIAL' if args.sequential else 'PIPELINE'
    src = ('REPLAY (saved shots)' if args.replay
           else 'TEST (offline)' if args.test else 'HARDWARE')
    print(f'device={device} encoder={args.encoder}  f*B={calib.fb:.2f}  '
          f'{args.steps} stops x {args.step_cm}cm  {mode}  {src}')

    run = run_sequential if args.sequential else run_pipeline
    return run(args, calib, model, out_ply)


if __name__ == '__main__':
    main()
