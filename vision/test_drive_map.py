"""Offline tests of the drive-map chain - no camera, no robot, no model. Run: python test_drive_map.py

drive_timeline (the commanded profile, aligning it to measured motion, and the scale of a run commanded
with an older wheel radius), drive_map's poses from the odometry (a leg the firmware cut short, a gap
filled at the leg's own ratio, a lost recording) and the tape scale, frame_motion (visual
odometry on a synthetic textured floor seen from known poses), drive_map's object landmarks, turn
axis and video, and stream_bench's rule for the largest good odometry step.
"""
import json
import math
import os
import sys
import tempfile

import cv2
import numpy as np

import drive_map
import drive_timeline
import leg_odometry
import stream_bench
from drawing import view_window
from floor_geometry import CameraMount
from floor_segment import floor_hit, floor_project
from frame_motion import estimate_motion, floor_rows, track, vo_scale, vo_turn_scale
from object_distance import Intrinsics
from occupancy_map import LOG_MAX, PIXEL_FREE, OccupancyMap
from scene_export import extract_regions

H, W = 480, 640
INTR = Intrinsics(fx=570.0, fy=570.0, cx=319.5, cy=239.5)
MOUNT = CameraMount(height_m=0.24, pitch_deg=-0.92, forward_m=0.185, left_m=0.062)
TEXTURE_RES_M = 0.004              # floor texture: 4 mm per texel
TEXTURE_HALF_M = 4.0
# Blur of the texture, texels. Band-limited, as a lens delivers the real floor: at 2 texels the far
# floor aliases in the rendering and its moire, fixed to the pixel grid, pulled a 0.8 deg turn to 0.74.
TEXTURE_BLUR = 8.0
SKY = 90                           # grey above the horizon
RUN_CRUISE_M_S, RUN_YAW_RAD_S = 0.15, 0.3   # the synthetic run.json's plan speeds (_write_run)
RECORDED_DIGITS = 3                # run.json's duration_s, ms (communication/motion_plan JSON_DIGITS)
# check_vo_positions: a 0.4 m leg (LEG_S at RUN_CRUISE_M_S) filmed at FPS, sent READY_WAIT_S after its
# status line (constants.h READY_WAIT_MS), its K ACK_LAG_S late (0.14-0.22 s measured 2026-09-28).
FPS = 30.0
LEG_S = 4.3
READY_WAIT_S = 2.0
ACK_LAG_S = 0.15
FIRMWARE_SPEEDUP = 1.65            # the 2026-09-28 backward legs: the firmware ran the whole trapezoid this fast
SLIP_K = 0.93                      # forward on the grey tile
GAP_FRAMES = 10                    # unmeasured pairs at cruise
TEST_VO_SCALE = 1.06               # the synthetic odometry reads this much short
TEST_TURN_SCALE = 0.9              # ... and turns 1 / this long (check_vo_turn_scale)
TEST_TURN_RAD = math.radians(45.0)   # fits LEG_S at RUN_YAW_RAD_S with room to ramp
TURN_TOL_RAD = math.radians(0.05)
POSE_TOL_M = 0.003
# check_measure_leg: STILL_S still, MOVE_S moving to LEG_TRUTH (dx, dy m, dtheta rad), STILL_S still.
STILL_S, MOVE_S = 0.5, 1.5
LEG_TRUTH = (0.20, 0.03, math.radians(5.0))
LEG_TOL = (0.005, 0.005, math.radians(0.2))
INDEX_TIME_TOL_S = 1e-3                # the index rounds t_capture to 0.1 ms
BOX_HEIGHT_PX = 80                     # synthetic YOLO boxes: any height, only the bottom edge places them
BOX_CONF = 0.8
SIGHTINGS_PER_VIEW = 3                 # LANDMARK_MIN_SIGHTINGS: each part alone would be a landmark
BOTTLE_HALF_M = 0.04
PERSON_VIEWS = 15                      # 15 x 0.8 = 12 votes >= scene_export.WALL_OBJECT_MIN_VOTES
EXTENT_TOL_M = 0.2
VOTE_TOL = 1e-3
SAMPLES_PER_CELL = 2                   # _occupy / the bed-front mask: points along a segment per grid cell
BOTTLE_TURNS_DEG = (0, 5, 10, 15)      # two bottles on one bearing, each seen from these headings
EDGE_BOX_COLUMNS = (300, 340)          # a box near the bottom edge: any columns, any top
EDGE_BOX_TOP = 100
EDGE_PROBE_PX = 3                      # probe FOOT_EDGE_PX this much inside / outside


def _leg(kind, amount, duration, speed):
    t, s = drive_timeline.trapezoid_profile(amount, speed, duration)
    return drive_timeline.Leg(0, kind, amount, duration, 0.0, t, s)


def check_trapezoid():
    """The fallback profile covers the leg in its duration, monotonically, never above cruise speed."""
    errors = []
    for amount, speed, duration in ((math.pi / 2, 0.3, 5.82), (-0.3, 0.15, 3.2), (0.02, 0.15, 0.5)):
        t, s = drive_timeline.trapezoid_profile(amount, speed, duration)
        v = np.diff(s) / np.diff(t)
        if abs(s[-1] - amount) > 1e-6 or np.any(np.sign(amount) * np.diff(s) < -1e-12):
            errors.append(f"{amount} in {duration} s: ends at {s[-1]:.5f}, monotonic {np.all(np.sign(amount) * np.diff(s) >= 0)}")
        if np.abs(v).max() > speed + 1e-6:
            errors.append(f"{amount}: peak {np.abs(v).max():.4f} over the cruise speed {speed}")
    return errors


def check_align():
    """A leg really started 2.2 s after its status line and went 0.8 of the command, measured at 30 fps
    with noise: align_leg must find the start within 50 ms."""
    errors = []
    rng = np.random.default_rng(3)
    for kind, amount, speed in ((drive_timeline.ROTATE, math.pi / 2, 0.3), (drive_timeline.FORWARD, 0.6, 0.15)):
        leg = _leg(kind, amount, 5.82 if kind == drive_timeline.ROTATE else 5.2, speed)
        times = np.arange(0.0, 12.0, 1 / 30.0)
        truth = 2.2
        moved = 0.8 * drive_timeline.displacement(leg, truth, times)
        measured = np.diff(moved, prepend=0.0) / (1 / 30.0) + rng.normal(0.0, 0.02 * speed, times.size)
        start, score = drive_timeline.align_leg(leg, times, measured, 0.0, 10.0)
        if abs(start - truth) > 0.05 or score < 0.9:
            errors.append(f"{kind}: start {start:.3f} (truth {truth}), score {score:.3f}")
    return errors


def _write_run(run_dir, duration_s, wheel_radius_m=None, primitive="FORWARD 0.400000", ack_t=None):
    """run.json of a one-leg demo_drive run at RUN_CRUISE_M_S / RUN_YAW_RAD_S, announced at t 100; ack_t:
    when robot_link announced the STOP leg, i.e. the leg's K came in."""
    plan = {"legs": [{"primitive": primitive, "duration_s": duration_s},
                     {"primitive": "STOP", "duration_s": 0.0}], "cruise_speed": RUN_CRUISE_M_S, "yaw_rate": RUN_YAW_RAD_S}
    if wheel_radius_m is not None:
        plan["wheel_radius_m"] = wheel_radius_m
    events = [{"t": 100.0, "event": "status", "text": "leg 1/2  F 0.400 0.150"}]
    if ack_t is not None:
        events.append({"t": ack_t, "event": "status", "text": "leg 2/2  S  (stop)"})
    events.append({"t": ack_t + 0.1 if ack_t is not None else 105.0, "event": "status", "text": "done"})
    with open(os.path.join(run_dir, drive_timeline.RUN_FILE), "w", encoding="utf-8") as f:
        json.dump({"plan": plan, "events": events}, f)


def check_run_scale():
    """A run recorded before the 2026-09-27 wheel-radius fix (no wheel_radius_m, 3.86 s for 0.4 m) keeps
    its recorded timing although today's MC plans the leg in 4.3 s, and its commanded metres are scaled
    by 0.040 / 0.055; a run carrying today's radius is scaled by 1."""
    errors = []
    with tempfile.TemporaryDirectory() as run_dir:
        _write_run(run_dir, 3.86)
        old = drive_timeline.load_run(run_dir)
        leg = old.legs[0]
        if old.wheel_radius_m != drive_timeline.LEGACY_WHEEL_RADIUS_M:
            errors.append(f"legacy run: wheel radius {old.wheel_radius_m}")
        if abs(leg.profile_t[-1] - 3.86) > drive_timeline.PROFILE_DT_S or abs(leg.profile_s[-1] - 0.4) > 1e-6:
            errors.append(f"legacy run: profile ends at {leg.profile_t[-1]:.2f} s / {leg.profile_s[-1]:.4f} m, recorded 3.86 s / 0.4 m")
        scale, want = drive_timeline.distance_scale(old), 0.040 / drive_timeline.LEGACY_WHEEL_RADIUS_M
        if abs(scale - want) > 1e-6:
            errors.append(f"legacy run: distance scale {scale:.4f}, want {want:.4f}")
        _write_run(run_dir, 4.3, 0.040)
        new = drive_timeline.load_run(run_dir)
        if abs(drive_timeline.distance_scale(new) - 1.0) > 1e-6:
            errors.append(f"run at today's radius: distance scale {drive_timeline.distance_scale(new):.4f}, want 1")
        # MC's own profile, not the fallback - also for a turn whose last tick sits 0.02 s + float noise
        # before its duration (it once tipped a one-tick tolerance into the fallback). Durations are today's
        # MC plans, rounded as run.json records them (6.08 s for the turn at L 0.2217 m, 6.04 at 0.21).
        for primitive in ("FORWARD 0.400000", "ROTATE 1.570800"):
            kind, amount = primitive.split()
            mc = drive_timeline._mc_profile(kind, float(amount), RUN_CRUISE_M_S, RUN_YAW_RAD_S)
            if mc is None:
                errors.append(f"{primitive}: MC unavailable, nothing to compare")
                continue
            duration = round(float(mc[0][-1]), RECORDED_DIGITS)
            _write_run(run_dir, duration, 0.040, primitive)
            leg = drive_timeline.load_run(run_dir).legs[0]
            uses_mc = np.array_equal(leg.profile_t, mc[0])
            # MC's clock is float32 summed tick by tick: compare within the code's own tolerance.
            if not uses_mc or abs(leg.profile_t[-1] - duration) > drive_timeline.DURATION_TOL_S:
                errors.append(f"{primitive} at today's radius: profile ends at {leg.profile_t[-1]:.3f} s "
                              f"({'MC' if uses_mc else 'fallback'}), recorded {duration}")
    return errors


def _motion_of(times, real_x):
    """drive_map motion rows of a straight run whose centre was at real_x (m) at `times`: every pair
    measured, reading 1 / TEST_VO_SCALE of the truth."""
    motion = np.zeros((len(times), leg_odometry.MOTION_COLUMNS))
    motion[1:, leg_odometry.COL_OK] = 1
    motion[1:, leg_odometry.COL_DX] = np.diff(real_x) / TEST_VO_SCALE
    return motion


def _fit_run(run_dir, times, motion):
    """(fits, poses) of run_dir's run.json on `motion`, at TEST_VO_SCALE and READY_WAIT_S."""
    run = drive_timeline.load_run(run_dir)
    _, odo_xy = leg_odometry.odometry_track(motion)
    fits = drive_map.align_legs(run, times, motion, odo_xy, TEST_VO_SCALE, READY_WAIT_S)
    return fits, drive_map.integrate_poses(times, motion, fits, 1.0, TEST_VO_SCALE)


def check_vo_positions():
    """Positions come from the odometry (task 2026-09-27_explore-map Pha 1' A): a backward leg the firmware
    ran FIRMWARE_SPEEDUP fast (acked after 2.6 of 4.3 s, 0.24 of 0.4 m, as on 2026-09-28) ends where the
    images put it and is flagged cut short; a forward leg at 0.93 of its command with GAP_FRAMES unmeasured
    frames at cruise is filled at its own measured ratio; a leg whose recording stopped a third of the way
    in is weak - not trusted."""
    errors = []
    times = 99.0 + np.arange(0.0, 9.0, 1.0 / FPS)
    sent = 100.0 + READY_WAIT_S                          # _write_run announces the leg at t 100
    with tempfile.TemporaryDirectory() as run_dir:
        _write_run(run_dir, LEG_S, 0.040, "FORWARD -0.400000", ack_t=sent + LEG_S / FIRMWARE_SPEEDUP + ACK_LAG_S)
        leg = drive_timeline.load_run(run_dir).legs[0]
        real_x = drive_timeline.displacement(leg, sent, sent + (times - sent) * FIRMWARE_SPEEDUP) / FIRMWARE_SPEEDUP
        fits, poses = _fit_run(run_dir, times, _motion_of(times, real_x))
        if not fits[0].cut_short or fits[0].trusted():
            errors.append(f"compressed leg: cut_short {fits[0].cut_short}, trusted {fits[0].trusted()} "
                          f"(firmware {fits[0].firmware_s:.2f} s of {LEG_S})")
        if abs(poses[-1][0] - real_x[-1]) > POSE_TOL_M:
            errors.append(f"compressed leg ends at x {poses[-1][0]:+.3f}, the images saw {real_x[-1]:+.3f}")

        _write_run(run_dir, LEG_S, 0.040, "FORWARD 0.400000", ack_t=sent + LEG_S + ACK_LAG_S)
        leg = drive_timeline.load_run(run_dir).legs[0]
        real_x = SLIP_K * drive_timeline.displacement(leg, sent, times)
        motion = _motion_of(times, real_x)
        gap = np.searchsorted(times, sent + LEG_S / 2) + np.arange(GAP_FRAMES)
        motion[gap, leg_odometry.COL_OK] = 0
        motion[gap, leg_odometry.COL_DX] = 0.0
        fits, poses = _fit_run(run_dir, times, motion)
        if fits[0].cut_short or not fits[0].trusted():
            errors.append(f"slipping leg: cut_short {fits[0].cut_short}, trusted {fits[0].trusted()} "
                          f"({drive_map.PERCENT * fits[0].pairs:.0f} % pairs)")
        if abs(poses[-1][0] - real_x[-1]) > POSE_TOL_M:
            errors.append(f"slipping leg with a gap ends at x {poses[-1][0]:+.3f}, truth {real_x[-1]:+.3f}")

        lost = times < sent + LEG_S / 3
        fits, _ = _fit_run(run_dir, times[lost], motion[lost])
        if fits[0].trusted():
            errors.append(f"recording lost mid-leg: trusted with {drive_map.PERCENT * fits[0].pairs:.0f} % pairs")
    return errors


def check_vo_scale():
    """frame_motion.vo_scale: sum(tape) / sum(VO) of the file's legs for the mount it names, 1 for any
    other mount (a calib_floor refit) and without a file; vo_turn_scale the same over its turns, 1 for a
    file without turns (written before 2026-09-29)."""
    errors = []
    legs = [{"tape_m": 0.38, "vo_m": 0.35}, {"tape_m": -0.24, "vo_m": -0.23}]
    turns = [{"truth_deg": 359.5, "vo_deg": 398.4}, {"truth_deg": -358.0, "vo_deg": -395.0}]
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "vo_scale.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"mount": dict(MOUNT._asdict()), "legs": legs}, f)
        if vo_turn_scale(MOUNT, path) != 1.0:
            errors.append(f"a file without turns: turn scale {vo_turn_scale(MOUNT, path)}")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"mount": dict(MOUNT._asdict()), "legs": legs, "turns": turns}, f)
        for mount, want, want_turn in ((MOUNT, 0.62 / 0.58, 717.5 / 793.4), (MOUNT._replace(pitch_deg=-1.5), 1.0, 1.0)):
            got, got_turn = vo_scale(mount, path), vo_turn_scale(mount, path)
            if abs(got - want) > 1e-9 or abs(got_turn - want_turn) > 1e-9:
                errors.append(f"mount {tuple(mount)}: scale {got:.4f} / turn {got_turn:.4f}, want {want:.4f} / {want_turn:.4f}")
        missing = os.path.join(tmp, "none.json")
        if vo_scale(MOUNT, missing) != 1.0 or vo_turn_scale(MOUNT, missing) != 1.0:
            errors.append(f"no file: scale {vo_scale(MOUNT, missing)}, turn {vo_turn_scale(MOUNT, missing)}")
    return errors


def check_vo_turn_scale():
    """A turn the odometry read 1 / TEST_TURN_SCALE long: at that turn scale the leg measures and the
    poses end at the true turn."""
    errors = []
    times = 99.0 + np.arange(0.0, 9.0, 1.0 / FPS)
    sent = 100.0 + READY_WAIT_S
    with tempfile.TemporaryDirectory() as run_dir:
        _write_run(run_dir, LEG_S, 0.040, f"ROTATE {TEST_TURN_RAD:.6f}", ack_t=sent + LEG_S + ACK_LAG_S)
        leg = drive_timeline.load_run(run_dir).legs[0]
        real = drive_timeline.displacement(leg, sent, times)
        motion = np.zeros((len(times), leg_odometry.MOTION_COLUMNS))
        motion[1:, leg_odometry.COL_OK] = 1
        motion[1:, leg_odometry.COL_DTHETA] = np.diff(real) / TEST_TURN_SCALE
        run = drive_timeline.load_run(run_dir)
        _, odo_xy = leg_odometry.odometry_track(motion)
        fits = drive_map.align_legs(run, times, motion, odo_xy, 1.0, READY_WAIT_S, TEST_TURN_SCALE)
        poses = drive_map.integrate_poses(times, motion, fits, 1.0, 1.0, TEST_TURN_SCALE)
        theta, _ = leg_odometry.odometry_track(motion, turn_scale=TEST_TURN_SCALE)
        for name, got in (("leg measured", fits[0].measured), ("pose heading", poses[-1][2]), ("odometry_track", theta[-1])):
            if abs(got - real[-1]) > TURN_TOL_RAD:
                errors.append(f"{name} {math.degrees(got):+.3f} deg, truth {math.degrees(real[-1]):+.3f}")
    return errors


def _texture(seed=7):
    rng = np.random.default_rng(seed)
    n = int(2 * TEXTURE_HALF_M / TEXTURE_RES_M)
    return cv2.normalize(cv2.GaussianBlur(rng.integers(0, 255, (n, n)).astype(np.uint8), (0, 0), TEXTURE_BLUR),
                         None, 0, 255, cv2.NORM_MINMAX)


def _render_floor(texture, pose, mount=MOUNT):
    """The camera frame of a textured floor, robot centre at pose = (x, y, theta) in the texture's world."""
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    fwd, left, _ = floor_hit(u.ravel(), v.ravel(), INTR, mount)
    x, y, th = pose
    wx = x + math.cos(th) * fwd - math.sin(th) * left
    wy = y + math.sin(th) * fwd + math.cos(th) * left
    map_x = ((wx + TEXTURE_HALF_M) / TEXTURE_RES_M).reshape(H, W).astype(np.float32)
    map_y = ((wy + TEXTURE_HALF_M) / TEXTURE_RES_M).reshape(H, W).astype(np.float32)
    image = cv2.remap(texture, map_x, map_y, cv2.INTER_LINEAR, borderValue=SKY)
    image[~np.isfinite(fwd.reshape(H, W))] = SKY
    return image


def check_floor_odometry():
    """Two views of a textured floor from known poses - an in-place turn (the camera swings on its
    arc) and a turn with a sideways slip: estimate_motion must return the true relative motion."""
    errors = []
    texture = _texture()
    top, bottom = floor_rows(INTR, MOUNT, H)
    mask = np.zeros((H, W), np.uint8)
    mask[top:bottom] = 255
    for second in ((0.0, 0.0, math.radians(0.8)), (0.0, 0.0, math.radians(2.5)), (0.004, -0.006, math.radians(-1.5))):
        g0, g1 = _render_floor(texture, (0.0, 0.0, 0.0)), _render_floor(texture, second)
        m = estimate_motion(*track(g0, g1, mask), INTR, MOUNT, np.random.default_rng(0))
        if m is None:
            errors.append(f"motion {second}: not measured")
            continue
        if abs(m.dtheta - second[2]) > math.radians(0.02) or abs(m.dx - second[0]) > 0.003 or abs(m.dy - second[1]) > 0.003:
            errors.append(f"motion {second}: got dx {m.dx:.4f} dy {m.dy:.4f} dtheta {math.degrees(m.dtheta):.3f} deg")
    return errors


def check_measure_leg():
    """leg_odometry.measure_leg on a synthetic recording - still, moving by LEG_TRUTH over MOVE_S, still -
    with a half-written last index line (the recorder mid-write): the leg's motion within LEG_TOL; a span
    running past the last frame (the camera lost mid-leg) is weak."""
    errors = []
    texture = _texture()
    times = np.arange(0.0, STILL_S + MOVE_S + STILL_S, 1.0 / FPS)
    progress = np.clip((times - STILL_S) / MOVE_S, 0.0, 1.0)
    progress = progress * progress * (3.0 - 2.0 * progress)          # smoothstep: starts and stops gently
    with tempfile.TemporaryDirectory() as run_dir:
        raw_dir = os.path.join(run_dir, leg_odometry.RAW_DIR)
        os.makedirs(raw_dir)
        with open(os.path.join(raw_dir, leg_odometry.RAW_INDEX), "w", encoding="utf-8") as index:
            for seq, (t, p) in enumerate(zip(times, progress)):
                pose = tuple(p * v for v in LEG_TRUTH)
                cv2.imwrite(os.path.join(raw_dir, leg_odometry.RAW_PATTERN.format(seq=seq)), _render_floor(texture, pose))
                index.write(json.dumps({"seq": seq, "t_capture": 100.0 + t}) + "\n")
            index.write('{"seq": 9999, "t_capt')
        last = leg_odometry._last_capture(run_dir)
        if last is None or abs(last - (100.0 + times[-1])) > INDEX_TIME_TOL_S:
            errors.append(f"index tail: last capture {last}, want {100.0 + times[-1]:.3f} (the half line skipped)")
        t0, t1 = 100.0 + STILL_S / 2, 100.0 + STILL_S * 1.5 + MOVE_S
        leg = leg_odometry.measure_leg(run_dir, t0, t1, INTR, MOUNT)
        got = (leg.dx, leg.dy, leg.dtheta)
        if leg.weak or any(abs(g - w) > tol for g, w, tol in zip(got, LEG_TRUTH, LEG_TOL)):
            errors.append(f"leg: dx {leg.dx:+.4f} dy {leg.dy:+.4f} dtheta {math.degrees(leg.dtheta):+.2f} deg, "
                          f"{leg.pairs:.2f} of it measured; truth {LEG_TRUTH[0]:+.4f} {LEG_TRUTH[1]:+.4f} "
                          f"{math.degrees(LEG_TRUTH[2]):+.2f} deg")
        lost = leg_odometry.measure_leg(run_dir, t0, 100.0 + times[-1] + MOVE_S, INTR, MOUNT)
        if not lost.weak:
            errors.append(f"span past the last frame: not weak ({lost.pairs:.2f} of it measured)")
    return errors


def check_landmarks():
    """A bottle standing still, seen from 5 poses of a turn, lands as ONE landmark where it is; a class
    seen twice is not a landmark."""
    errors = []
    world = (2.0, 0.5)
    poses = {seq: (0.0, 0.0, math.radians(a)) for seq, a in zip(range(10, 15), (0, 5, 10, 15, 20))}
    rows = []
    for seq, (x, y, th) in poses.items():
        c, s = math.cos(th), math.sin(th)
        bx, by = c * world[0] + s * world[1], -s * world[0] + c * world[1]     # world -> body
        u, v = floor_project([bx], [by], INTR, MOUNT)
        box = [float(u[0]) - 10, float(v[0]) - 60, float(u[0]) + 10, float(v[0])]
        rows.append({"seq": seq, "detections": [{"class": "bottle", "confidence": 0.8, "box": box}]})
    rows[0]["detections"].append({"class": "cup", "confidence": 0.6, "box": [300, 300, 320, 350]})
    rows[1]["detections"].append({"class": "cup", "confidence": 0.6, "box": [300, 300, 320, 350]})
    with tempfile.TemporaryDirectory() as run_dir:
        with open(os.path.join(run_dir, drive_map.DETECTIONS_FILE), "w", encoding="utf-8") as f:
            f.write("\n".join(json.dumps(r) for r in rows) + "\n")
        sightings = drive_map.object_sightings(run_dir, poses, INTR, MOUNT, OccupancyMap(), H)
    landmarks = drive_map.cluster_landmarks(sightings)
    if len(landmarks) != 1 or landmarks[0]["class"] != "bottle" or landmarks[0]["sightings"] != 5:
        errors.append(f"expected one bottle seen 5 times, got {landmarks}")
    elif math.hypot(landmarks[0]["x"] - world[0], landmarks[0]["y"] - world[1]) > 0.01:
        errors.append(f"bottle at {landmarks[0]['x']}, {landmarks[0]['y']}, truth {world}")
    return errors


def _world_to_body(xy, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return c * (xy[0] - x) + s * (xy[1] - y), -s * (xy[0] - x) + c * (xy[1] - y)


def _box(pose, foot, left_end, right_end, height_px=BOX_HEIGHT_PX):
    """A YOLO box seen from pose: bottom edge on the floor row of `foot` (world), side edges at the columns
    of the world points left_end / right_end."""
    us = [float(floor_project(*[[v] for v in _world_to_body(p, pose)], INTR, MOUNT)[0][0]) for p in (left_end, right_end)]
    v = float(floor_project(*[[c] for c in _world_to_body(foot, pose)], INTR, MOUNT)[1][0])
    return [min(us), v - height_px, max(us), v]


def _sightings_of(views, occ_map=None):
    """object_sightings of `views` [(pose, class, box)], one detections.jsonl row each."""
    poses = {seq: pose for seq, (pose, _, _) in enumerate(views)}
    rows = [{"seq": seq, "detections": [{"class": name, "confidence": BOX_CONF, "box": box}]}
            for seq, (_, name, box) in enumerate(views)]
    with tempfile.TemporaryDirectory() as run_dir:
        with open(os.path.join(run_dir, drive_map.DETECTIONS_FILE), "w", encoding="utf-8") as f:
            f.write("\n".join(json.dumps(r) for r in rows) + "\n")
        return drive_map.object_sightings(run_dir, poses, INTR, MOUNT, occ_map or OccupancyMap(), H)


def check_big_object_landmarks():
    """A person lying 1.6 m long, seen feet-only from the start, whole after a turn and head-first up close:
    three clusters that the whole-body box held together -> ONE landmark of 3 parts. Two bottles on one line
    of sight 1 m apart stay two (a bottle's box is a few cm wide). A box ending FOOT_EDGE_PX or less above
    the frame's bottom has its foot out of view and gives no sighting."""
    errors = []
    head, feet = (1.0, -0.8), (2.6, -0.8)
    feet_view, whole_view, head_view = (0.0, 0.0, 0.0), (0.0, 0.0, -0.3), (0.4, 0.0, -0.9)
    views = []
    for _ in range(SIGHTINGS_PER_VIEW):
        views.append((feet_view, "person", _box(feet_view, (2.5, -0.8), (2.5, -0.65), (2.5, -0.95))))
        views.append((whole_view, "person", _box(whole_view, (1.8, -0.8), head, feet)))
        views.append((head_view, "person", _box(head_view, (1.1, -0.8), (1.1, -0.6), (1.1, -1.0))))
    sightings = _sightings_of(views)
    clusters = drive_map.greedy_clusters(sightings)["person"]
    if len(clusters) != 3:
        errors.append(f"expected the 3 parts as 3 greedy clusters (else the test proves nothing), got {len(clusters)}")
    landmarks = drive_map.cluster_landmarks(sightings)
    if len(landmarks) != 1 or landmarks[0]["parts"] != 3 or landmarks[0]["sightings"] != len(views):
        errors.append(f"lying person: expected 1 landmark of 3 parts, {len(views)} sightings, got "
                      f"{[(lm['x'], lm['y'], lm['parts'], lm['sightings']) for lm in landmarks]}")
    else:
        x0, y0, x1, y1 = landmarks[0]["extent_m"]
        if not (x0 < head[0] + EXTENT_TOL_M and x1 > feet[0] - EXTENT_TOL_M and y0 < head[1] + EXTENT_TOL_M
                and y1 > head[1] - EXTENT_TOL_M):
            errors.append(f"lying person extent {landmarks[0]['extent_m']} does not span head to feet")

    near, far = (1.5, 0.3), (2.5, 0.5)                    # one bearing from the lens, 1 m apart
    views = []
    for bottle in (near, far):
        for heading in BOTTLE_TURNS_DEG:
            pose = (0.0, 0.0, math.radians(heading))
            ends = (bottle[0], bottle[1] + BOTTLE_HALF_M), (bottle[0], bottle[1] - BOTTLE_HALF_M)
            views.append((pose, "bottle", _box(pose, bottle, *ends)))
    landmarks = drive_map.cluster_landmarks(_sightings_of(views))
    if len(landmarks) != 2 or any(lm["parts"] != 1 for lm in landmarks):
        errors.append(f"two bottles on one bearing: expected 2 landmarks, got "
                      f"{[(lm['x'], lm['y'], lm['parts']) for lm in landmarks]}")

    pose = (0.0, 0.0, 0.0)
    for above, want in ((drive_map.FOOT_EDGE_PX - EDGE_PROBE_PX, False), (drive_map.FOOT_EDGE_PX + EDGE_PROBE_PX, True)):
        box = [EDGE_BOX_COLUMNS[0], EDGE_BOX_TOP, EDGE_BOX_COLUMNS[1], H - above]
        row = {"seq": 0, "detections": [{"class": "person", "confidence": BOX_CONF, "box": box}]}
        if bool(drive_map.row_sightings(row, pose, INTR, MOUNT, H)) != want:
            errors.append(f"a box ending {above} px above the bottom edge: sighting {not want}, want {want}")
    return errors


def _occupy(occ_map, start, end):
    """Mark the cells along the world segment start -> end occupied."""
    for t in np.linspace(0.0, 1.0, int(math.dist(start, end) / occ_map.res) * SAMPLES_PER_CELL + 1):
        ix, iy = occ_map.world_to_cell(start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))
        occ_map.log_odds[iy, ix] = LOG_MAX


def check_support_votes():
    """A person lying on a bed: the box ends on the mattress, so its floor point lands behind the bed's
    front. Their class votes go on that front (support_points), which the scene makes a "person" object
    instead of a wall; a bottle standing before it stays its own object, a bare line stays a wall."""
    errors = []
    occ_map = OccupancyMap()
    bed_front = ((1.2, -0.6), (3.4, -0.6))
    _occupy(occ_map, *bed_front)
    _occupy(occ_map, (4.0, -1.0), (4.0, 1.0))                # a bare wall
    for y in (-0.3, -0.25, -0.2):                            # a bottle before the bed, off the person's rays
        _occupy(occ_map, (2.4, y), (2.5, y))
    pose = (0.0, 0.0, 0.0)
    views = [(pose, "person", _box(pose, (2.8, -0.9), (2.5, -0.9), (3.1, -0.9))) for _ in range(PERSON_VIEWS)]
    views += [(pose, "bottle", _box(pose, (2.45, -0.18), (2.45, -0.15), (2.45, -0.35))) for _ in range(PERSON_VIEWS)]
    _sightings_of(views, occ_map)
    person = occ_map.votes.get("person")
    front = np.zeros(occ_map.log_odds.shape, bool)
    (x0, y), (x1, _) = bed_front
    for x in np.linspace(x0, x1, int((x1 - x0) / occ_map.res) * SAMPLES_PER_CELL + 1):
        ix, iy = occ_map.world_to_cell(x, y)
        front[iy, ix] = True
    got = 0.0 if person is None else float(person[front].sum())
    if abs(got - PERSON_VIEWS * BOX_CONF) > VOTE_TOL:
        errors.append(f"person votes on the bed front: {got:.2f}, want {PERSON_VIEWS * BOX_CONF:.2f}")
    labels = sorted((r.kind, r.label) for r in extract_regions(occ_map))
    want = [("object", "bottle"), ("object", "person"), ("wall", "wall")]
    if labels != want:
        errors.append(f"regions {labels}, want {want}")
    return errors


def check_rotation_axis():
    """An in-place turn about the true centre, measured with the lens assumed 0.5 cm further ahead and
    on the centre line (the mount before the user measured it): rotation_axis finds the true pivot."""
    errors = []
    texture = _texture()
    assumed = MOUNT._replace(forward_m=0.18, left_m=0.0)
    top, bottom = floor_rows(INTR, assumed, H)
    mask = np.zeros((H, W), np.uint8)
    mask[top:bottom] = 255
    step = math.radians(1.5)
    motion = np.zeros((drive_map.AXIS_MIN_FRAMES + 2, leg_odometry.MOTION_COLUMNS))
    prev = _render_floor(texture, (0.0, 0.0, 0.0))
    for k in range(1, len(motion)):
        cur = _render_floor(texture, (0.0, 0.0, k * step))
        m = estimate_motion(*track(prev, cur, mask), INTR, assumed, np.random.default_rng(0))
        prev = cur
        if m is not None:
            motion[k] = (1, m.dx, m.dy, m.dtheta, m.inliers, m.tracks, m.rms_px)
    axis = drive_map.rotation_axis(motion)
    truth = (assumed.forward_m - MOUNT.forward_m, assumed.left_m - MOUNT.left_m)
    if axis is None or math.hypot(axis[0] - truth[0], axis[1] - truth[1]) > 0.01:
        errors.append(f"turn axis {axis}, truth {truth} (the true centre in the assumed body frame)")
    if drive_map.rotation_axis(motion[:drive_map.AXIS_MIN_FRAMES // 2]) is not None:
        errors.append("an axis from too few turning frames")
    return errors


def check_sweep_rule():
    """largest_good_step: the largest stride that, with every smaller one, measured all pairs and the
    turn within MAX_TOTAL_ERR - a good stride past a bad one does not count."""
    errors = []
    row = stream_bench.SweepRow
    rows = [row(1, 0.4, 1.0, 50, 1.0, 1.002), row(2, 0.9, 1.7, 50, 1.0, 0.99), row(5, 2.2, 3.0, 50, 0.96, 0.95),
            row(10, 4.4, 5.4, 50, 1.0, 1.0)]
    for given, want in ((rows, 2), (rows[:2], 2), (rows[2:], None), ([rows[0]._replace(total_ratio=math.nan)], None)):
        got = stream_bench.largest_good_step(given)
        if (got.stride if got else None) != want:
            errors.append(f"strides {[r.stride for r in given]}: largest good {got}, want {want}")
    return errors


def check_video():
    """MapVideo on a synthetic run: real-time length across a gap the camera dropped, both panels, and
    the video's own map ends equal to build_floor's (map.png's) - a no-plane analysis stays out of both."""
    errors = []
    texture = _texture()
    times = np.array([k / 30.0 for k in range(7)] + [0.3 + k / 30.0 for k in range(5)])   # 0.2 -> 0.3 s: 2 dropped
    poses = np.array([(0.0, 0.0, math.radians(0.5 * k)) for k in range(len(times))])
    frames = [{"seq": k, "t_capture": 100.0 + t} for k, t in enumerate(times)]
    labels = np.zeros((H, W), np.uint8)
    labels[400:, :] = PIXEL_FREE
    contacts = [[u, 399] for u in range(0, W, 4)]
    with tempfile.TemporaryDirectory() as run_dir:
        for sub_dir in (leg_odometry.RAW_DIR, drive_map.FLOOR_DIR):
            os.makedirs(os.path.join(run_dir, sub_dir))
        for fr, pose in zip(frames, poses):
            cv2.imwrite(os.path.join(run_dir, leg_odometry.RAW_DIR, leg_odometry.RAW_PATTERN.format(seq=fr["seq"])),
                        cv2.cvtColor(_render_floor(texture, tuple(pose)), cv2.COLOR_GRAY2BGR))
        cv2.imwrite(os.path.join(run_dir, drive_map.FLOOR_DIR, "labels.png"), labels)
        with open(os.path.join(run_dir, drive_map.FLOOR_DIR, drive_map.FLOOR_INDEX), "w", encoding="utf-8") as f:
            for seq, plane in ((2, [0.04, 0.0, -12.0]), (5, None), (8, [0.04, 0.0, -12.0])):
                f.write(json.dumps({"seq": seq, "labels": "labels.png", "plane": plane, "contacts": contacts}) + "\n")
        det = {"class": "bottle", "class_id": 39, "confidence": 0.8, "box": [300, 300, 340, 390], "range_m": 1.2,
               "z_m": 1.2, "xyz_cam": [0.0, 0.2, 1.2], "valid_fraction": None}
        with open(os.path.join(run_dir, drive_map.DETECTIONS_FILE), "w", encoding="utf-8") as f:
            for seq in (0, 4, 9):
                f.write(json.dumps({"seq": seq, "infer_ms": 50.0, "motion": "leg 1/1", "detections": [det]}) + "\n")
        pose_of = {fr["seq"]: tuple(p) for fr, p in zip(frames, poses)}
        rate_of = {fr["seq"]: 0.0 for fr in frames}
        final = OccupancyMap()
        drive_map.build_floor(final, run_dir, pose_of, rate_of, INTR, MOUNT)
        dm = drive_map.DriveMap(run_dir, {"frame_size": [W, H], "floor": {"far_m": 3.0, "half_width_m": None}}, INTR,
                                MOUNT, None, 1.0, frames, times, np.zeros((len(frames), leg_odometry.MOTION_COLUMNS)),
                                np.zeros((len(frames), 2)), [], poses, np.zeros(len(frames)), pose_of, rate_of, final,
                                (2, 1), [])
        video = drive_map.MapVideo(dm, view_window(final, [tuple(p) for p in poses]))
        path = os.path.join(run_dir, drive_map.VIDEO_FILE)
        count = video.write(path)
        want = int((times[-1] - times[0]) * drive_map.VIDEO_FPS) + 1
        if count != want:
            errors.append(f"{count} video frames for {times[-1] - times[0]:.3f} s, want {want}")
        cap = cv2.VideoCapture(path)
        ok, image = cap.read()
        cap.release()
        if not ok or image.shape != (H, W + H, 3):
            errors.append(f"video frame {None if not ok else image.shape}, want {(H, W + H, 3)}")
        if not np.array_equal(video.occ_map.log_odds, final.log_odds):
            errors.append("the video's map ends different from build_floor's")
        if not final.free().any():
            errors.append("the synthetic floor made no free cell: the comparison above proves nothing")
    return errors


def main():
    failed = 0
    for check in (check_trapezoid, check_align, check_run_scale, check_vo_positions, check_vo_scale,
                  check_vo_turn_scale, check_floor_odometry, check_measure_leg, check_landmarks, check_big_object_landmarks,
                  check_support_votes, check_rotation_axis, check_sweep_rule, check_video):
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
