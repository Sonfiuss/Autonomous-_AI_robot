"""Drawing shared by perception.py and map_builder.py: the camera view and the top-down map view."""
import math

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX
FRAME_W, FRAME_H = 640, 480
OVERLAY_ALPHA = 0.4
ESTIMATE_MARK = "~"          # prefixes a distance estimated from floor geometry rather than measured
DEPTH_VIEW_MIN_MM, DEPTH_VIEW_MAX_MM = 600, 8000   # sensor range, documents/firmware/Orbbec.txt

MAP_FREE, MAP_UNKNOWN, MAP_OCCUPIED = 255, 128, 0
MAP_MARGIN_M = 0.75          # around everything seen
MAP_EMPTY_HALF_SPAN_M = 2.0  # view half-width before anything has been seen
FOV_WEDGE_M = 1.0
WALL_COLOR = (0, 0, 220)
UNKNOWN_COLOR = (0, 140, 255)
POSE_COLOR = (0, 160, 0)
CURRENT_POSE_COLOR = (255, 0, 255)


def class_color(key):
    """Stable, distinct BGR color per class id (int) or class name (str)."""
    if isinstance(key, str):
        key = sum(ord(c) * (i + 1) for i, c in enumerate(key))
    hsv = np.uint8([[[(key * 37) % 180, 200, 255]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def format_range(rng):
    """'1.23 m' measured by depth; '~1.23 m' estimated from floor geometry (valid_fraction None)."""
    if rng is None:
        return "--"
    return f"{ESTIMATE_MARK if rng.valid_fraction is None else ''}{rng.range_m:.2f} m"


def put_text(canvas, text, org, color, scale=0.5):
    # Black outline first so the text stays legible over any background.
    cv2.putText(canvas, text, org, FONT, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(canvas, text, org, FONT, scale, color, 1, cv2.LINE_AA)


def depth_view(depth_mm):
    clipped = np.clip(depth_mm, DEPTH_VIEW_MIN_MM, DEPTH_VIEW_MAX_MM)
    scale = 255.0 / (DEPTH_VIEW_MAX_MM - DEPTH_VIEW_MIN_MM)
    view = cv2.applyColorMap(((clipped - DEPTH_VIEW_MIN_MM) * scale).astype(np.uint8), cv2.COLORMAP_JET)
    view[depth_mm == 0] = 0
    return view


def render_view(color, depth, results, status, info, show_overlay):
    """Camera view: RGB (optionally with depth blended in), boxes with distances, a status bar."""
    canvas = color.data.copy() if color is not None else np.zeros((FRAME_H, FRAME_W, 3), np.uint8)
    if show_overlay and depth is not None:
        canvas = cv2.addWeighted(canvas, 1.0 - OVERLAY_ALPHA, depth_view(depth.data), OVERLAY_ALPHA, 0)
    for det, rng in results:
        x1, y1, x2, y2 = (int(round(v)) for v in det.box)
        box_color = class_color(det.class_id)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), box_color, 2)
        put_text(canvas, f"{det.name} {det.conf:.2f} | {format_range(rng)}", (x1 + 2, max(y1 - 6, 34)), box_color)
    cv2.rectangle(canvas, (0, 0), (FRAME_W, 22), (0, 0, 0), -1)
    put_text(canvas, status, (6, 16), (0, 220, 0) if status == "OK" else (0, 0, 255))
    put_text(canvas, info, (140, 16), (255, 255, 255))
    return canvas


# Keys are occupancy_map.PIXEL_* labels. BGR.
PIXEL_COLORS = {1: (0, 200, 0), 2: (0, 0, 230), 3: (230, 120, 0), 4: (230, 0, 230)}
PIXEL_NAMES = {1: "free floor (DA)", 2: "obstacle", 3: "above robot", 4: "drop (DA)"}
PIXEL_ALPHA = 0.45
TRAPEZOID_COLOR = (255, 255, 255)
CONTACT_COLOR = (255, 255, 255)


def floor_view(background, labels, trapezoid=None, contacts=None):
    """What the map gets from this frame: free floor green, obstacle red, above-the-robot blue, a drop
    magenta, the rest dimmed. White: the outline of the floor trapezoid Depth Anything searched, and
    the contacts - where each of its columns met the first obstacle."""
    canvas = (background * 0.6).astype(np.uint8)
    for label, color in PIXEL_COLORS.items():
        mask = labels == label
        canvas[mask] = ((1 - PIXEL_ALPHA) * background[mask] + PIXEL_ALPHA * np.array(color)).astype(np.uint8)
    if trapezoid is not None and trapezoid.any():
        outlines, _ = cv2.findContours(trapezoid.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, outlines, -1, TRAPEZOID_COLOR, 1)
    if contacts is not None and len(contacts):
        canvas[contacts[:, 1], contacts[:, 0]] = CONTACT_COLOR
    total = labels.size
    y = FRAME_H - 8
    for label, name in reversed(list(PIXEL_NAMES.items())):
        text = f"{name} {100.0 * (labels == label).sum() / total:.0f}%"
        put_text(canvas, text, (6, y), PIXEL_COLORS[label])
        y -= 18
    return canvas


def blend_floor(background, labels, trapezoid=None):
    """floor_view's colors over an undimmed image, no legend: for video, where the scene must stay
    readable under the overlay. The trapezoid Depth Anything searched is outlined in white."""
    canvas = background.copy()
    for label, color in PIXEL_COLORS.items():
        mask = labels == label
        canvas[mask] = ((1 - PIXEL_ALPHA) * background[mask] + PIXEL_ALPHA * np.array(color)).astype(np.uint8)
    if trapezoid is not None and trapezoid.any():
        outlines, _ = cv2.findContours(trapezoid.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, outlines, -1, TRAPEZOID_COLOR, 1)
    return canvas


def _view_window(occ_map, poses):
    """World (x0, y0, span) of a square around every known cell and pose, plus a margin."""
    iy, ix = np.nonzero(occ_map.occupied() | occ_map.free())
    xs = [occ_map.origin[0] + ix.min() * occ_map.res, occ_map.origin[0] + (ix.max() + 1) * occ_map.res] if ix.size else []
    ys = [occ_map.origin[1] + iy.min() * occ_map.res, occ_map.origin[1] + (iy.max() + 1) * occ_map.res] if iy.size else []
    xs += [p[0] for p in poses]
    ys += [p[1] for p in poses]
    if not xs:
        return -MAP_EMPTY_HALF_SPAN_M, -MAP_EMPTY_HALF_SPAN_M, 2 * MAP_EMPTY_HALF_SPAN_M
    span = max(max(xs) - min(xs), max(ys) - min(ys)) + 2 * MAP_MARGIN_M
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    return cx - span / 2, cy - span / 2, span


def render_map(occ_map, regions, poses, current_pose, size_px, hfov_rad=None):
    """Top-down map, world y up: free white, occupied black, unknown gray. Walls red, objects outlined
    in their class color with labels (unknown in orange), capture poses green, the current pose
    magenta with the camera's horizontal FOV."""
    x0, y0, span = _view_window(occ_map, poses + ([current_pose] if current_pose else []))
    centers = (np.arange(size_px) + 0.5) / size_px * span
    ix = np.floor((x0 + centers - occ_map.origin[0]) / occ_map.res).astype(int)
    iy = np.floor((y0 + span - centers - occ_map.origin[1]) / occ_map.res).astype(int)   # row 0 = top = max y
    grid = np.full(occ_map.log_odds.shape, MAP_UNKNOWN, np.uint8)
    grid[occ_map.free()] = MAP_FREE
    grid[occ_map.occupied()] = MAP_OCCUPIED
    inside = (iy[:, None] >= 0) & (iy[:, None] < occ_map.n) & (ix[None, :] >= 0) & (ix[None, :] < occ_map.n)
    gray = np.where(inside, grid[np.clip(iy, 0, occ_map.n - 1)[:, None], np.clip(ix, 0, occ_map.n - 1)[None, :]],
                    MAP_UNKNOWN).astype(np.uint8)
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def px(p):
        return (int(round((p[0] - x0) / span * size_px)), int(round((y0 + span - p[1]) / span * size_px)))

    for r in regions:
        if r.kind == "wall":
            cv2.line(img, px(r.polygon[0]), px(r.polygon[1]), WALL_COLOR, 2)
            continue
        color = UNKNOWN_COLOR if r.label == "unknown" else class_color(r.label)
        cv2.polylines(img, [np.array([px(p) for p in r.polygon], np.int32)], True, color, 2)
        cx, cy = px(r.center)
        put_text(img, r.label, (cx + 4, cy - 4), color, 0.4)
    for pose in poses:
        cv2.circle(img, px(pose), 3, POSE_COLOR, -1)
    if current_pose is not None:
        x, y, theta = current_pose
        if hfov_rad:
            for side in (-1, 1):
                a = theta + side * hfov_rad / 2
                cv2.line(img, px((x, y)), px((x + FOV_WEDGE_M * math.cos(a), y + FOV_WEDGE_M * math.sin(a))),
                         CURRENT_POSE_COLOR, 1)
        cv2.arrowedLine(img, px((x, y)), px((x + 0.3 * math.cos(theta), y + 0.3 * math.sin(theta))),
                        CURRENT_POSE_COLOR, 2, tipLength=0.4)
    put_text(img, f"{span:.1f} m across", (6, size_px - 8), (255, 255, 255), 0.45)
    return img
