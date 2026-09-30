"""The real room's map as a PNG to judge it by: what the web page shows, as a file.

  python3 realroom/map_image.py                   # maps/latest.png: the current map, robot and trail
  python3 realroom/map_image.py --map <id> [--out FILE]   # a published map as it was built

map_store redraws maps/latest.png after every pose change and maps/map_<id>.png when a map is published.
One fixed palette, whatever the viewer's theme - the page (static/index.html) uses the same one - chosen so
the three kinds of floor never look alike: free floor near white, occupied near black, unseen mid gray
hatched (lightness 96 / 11 / 55). Objects: a light wash of their own color (color_rgb), amber outline, a
label with the color as a swatch.
The robot: its chassis, three wheels and dashed collision hull as the simulator draws it (scene_export
robot_shape), a disc for maps published before it had one. 0.5 m grid on the free floor, axes in metres.
Text is ASCII: OpenCV's fonts have no Vietnamese.
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

import map_store

PX_PER_M = 200                 # 5 mm a pixel
MAX_SIDE_PX = 1800             # a bigger room is drawn smaller
VIEW_PAD_M = 0.25              # around the room, the robot and its trail
GRID_STEP_M = 0.5
MARGIN_LEFT, MARGIN_RIGHT, MARGIN_TOP, MARGIN_BOTTOM = 48, 16, 36, 30
LEGEND_H = 34
HATCH_PX = 8
ARROW_M = 0.3
LABEL_GAP_PX = 6
LABEL_PAD_PX = 3
SWATCH_PX = 10                 # an object's color in its label
LEGEND_SWATCH_PX = 14
MAX_LABEL_TRIES = 12
LABEL_SPACING_PX = 2           # between two labels stacked to avoid each other
TEXT_DROP_PX = 2               # a label's text baseline above its box's bottom padding
DASH_PX, GAP_PX = 6, 4
TRAIL_DOT_PX = 3
GOAL_DOT_PX = 5
ARROW_TIP_SHARE = 0.25         # the heading arrow's head, as a share of its length
THIN_PX, BOLD_PX = 1, 2        # line widths
TICK_PX = 4
TICK_LABEL_BELOW_PX = 17       # an x tick label's baseline under the panel
TICK_LABEL_GAP_PX = 8          # a y tick label's right end left of the panel
TICK_LABEL_HALF_PX = 4         # half a label's height: y labels centred on their tick
UNIT_BELOW_PX, UNIT_LEFT_PX = 28, 10    # the "m" under the x axis's end
LEGEND_TOP_PX = 6              # the legend's row under the tick labels
LEGEND_TEXT_GAP_PX = 6
LEGEND_ITEM_GAP_PX = 18
LEGEND_TEXT_RAISE_PX = 3
SCALE_BAR_ROOM_PX = 40         # kept right of the 1 m bar for its label
SCALE_BAR_Y_PX, SCALE_BAR_END_PX = 10, 5
SCALE_LABEL_DX_PX, SCALE_LABEL_DY_PX = 6, 14
TITLE_ABOVE_PX = 12            # the title's baseline above the panel
CHASSIS_ALPHA = 0.5
LABEL_ALPHA = 0.85
OBJECT_WASH_ALPHA = 0.25       # light enough that a black object's free cells stay far from unseen gray
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE, TITLE_SCALE = 0.42, 0.5
AA = cv2.LINE_AA

# "#rrggbb", as the page's CSS has them.
PAPER = "#ffffff"
FREE = "#f4f4f0"
OCCUPIED = "#1c1c1c"
UNSEEN, HATCH = "#84847e", "#6c6c66"
GRID_LINE = "#d2d2cb"
ROOM_EDGE = "#6b6b66"
OBJECT_EDGE = "#d08a00"
INK, MUTED = "#1d1d1b", "#6b6b66"
ROBOT_FILL, ROBOT_EDGE = "#2196f3", "#1565c0"
WHEEL, WHEEL_EDGE = "#333333", "#111111"
TRAIL = "#1f6feb"
PATH = "#d1242f"
NO_COLOR = "#ffffff"          # an object whose color is unknown: an empty swatch
FREE_ID, OCCUPIED_ID, UNSEEN_ID = 0, 1, 2   # per-pixel classes of the map panel


def bgr(hex_color):
    """OpenCV's (b, g, r) of "#rrggbb"."""
    value = int(hex_color.lstrip("#"), 16)
    return value & 0xFF, (value >> 8) & 0xFF, value >> 16


class View:
    """Scene metres <-> canvas pixels: y up in the scene, down in the image."""

    def __init__(self, x0, y0, x1, y1, scale):
        self.x0, self.y0, self.x1, self.y1, self.scale = x0, y0, x1, y1, scale
        self.width = int(math.ceil((x1 - x0) * scale))
        self.height = int(math.ceil((y1 - y0) * scale))

    def px(self, x, y):
        return (MARGIN_LEFT + (x - self.x0) * self.scale, MARGIN_TOP + (self.y1 - y) * self.scale)

    def ipx(self, x, y):
        u, v = self.px(x, y)
        return int(round(u)), int(round(v))


def _body_points(robot, points):
    """World (x, y) of body-frame {x, y} points of the robot."""
    c, s = math.cos(robot["theta"]), math.sin(robot["theta"])
    return [(robot["x"] + c * p["x"] - s * p["y"], robot["y"] + s * p["x"] + c * p["y"]) for p in points]


def _view(scene, plan_path):
    """The View around the room, the robot, its trail and the plan, VIEW_PAD_M beyond them."""
    robot, room = scene["robot"], scene["room"]
    xs, ys = [0.0, room["width"]], [0.0, room["length"]]
    for x, y in ([(h["x"], h["y"]) for h in scene.get("pose_history", [])] + list(plan_path or [])
                 + [(robot["x"] + dx * robot["radius"], robot["y"] + dy * robot["radius"])
                    for dx in (-1, 1) for dy in (-1, 1)]):
        xs.append(x)
        ys.append(y)
    x0, x1, y0, y1 = min(xs) - VIEW_PAD_M, max(xs) + VIEW_PAD_M, min(ys) - VIEW_PAD_M, max(ys) + VIEW_PAD_M
    scale = min(PX_PER_M, MAX_SIDE_PX / max(x1 - x0, y1 - y0))
    return View(x0, y0, x1, y1, scale)


def _classes(grid, view):
    """(height, width) FREE_ID / OCCUPIED_ID / UNSEEN_ID of every panel pixel, from its centre's cell."""
    xs = view.x0 + (np.arange(view.width) + 0.5) / view.scale
    ys = view.y1 - (np.arange(view.height) + 0.5) / view.scale
    ix = np.floor((xs - grid.origin[0]) / grid.res).astype(np.int64)
    iy = np.floor((ys - grid.origin[1]) / grid.res).astype(np.int64)
    rows, cols = grid.cells.shape
    in_x, in_y = (ix >= 0) & (ix < cols), (iy >= 0) & (iy < rows)
    cells = np.full((view.height, view.width), map_store.KNOWN_UNSEEN, np.int8)
    cells[np.ix_(in_y, in_x)] = grid.cells[np.ix_(iy[in_y], ix[in_x])]
    classes = np.full(cells.shape, UNSEEN_ID, np.uint8)
    classes[cells == map_store.KNOWN_FREE] = FREE_ID
    classes[cells == map_store.KNOWN_OCCUPIED] = OCCUPIED_ID
    return classes


def _panel(grid, view):
    """The map panel: hatched unseen floor, free floor with the metre grid, occupied cells."""
    classes = _classes(grid, view)
    rows, cols = np.indices(classes.shape)
    panel = np.empty(classes.shape + (3,), np.uint8)
    panel[:] = bgr(UNSEEN)
    panel[(rows + cols) % HATCH_PX == 0] = bgr(HATCH)
    panel[classes == FREE_ID] = bgr(FREE)
    lines = np.zeros(classes.shape, bool)
    for x in _ticks(view.x0, view.x1):
        lines[:, min(max(int(round((x - view.x0) * view.scale)), 0), view.width - 1)] = True
    for y in _ticks(view.y0, view.y1):
        lines[min(max(int(round((view.y1 - y) * view.scale)), 0), view.height - 1), :] = True
    panel[lines & (classes == FREE_ID)] = bgr(GRID_LINE)
    panel[classes == OCCUPIED_ID] = bgr(OCCUPIED)
    return panel


def _dashed(img, points, color, thickness=THIN_PX, closed=False):
    """A dashed polyline through pixel points."""
    pts = list(points) + ([points[0]] if closed else [])
    for (u0, v0), (u1, v1) in zip(pts, pts[1:]):
        length = math.hypot(u1 - u0, v1 - v0)
        at = 0.0
        while at < length:
            end = min(at + DASH_PX, length)
            a, b = at / length, end / length
            cv2.line(img, (int(round(u0 + a * (u1 - u0))), int(round(v0 + a * (v1 - v0)))),
                     (int(round(u0 + b * (u1 - u0))), int(round(v0 + b * (v1 - v0)))), bgr(color), thickness, AA)
            at = end + GAP_PX


def _polygon(view, points):
    return np.array([view.ipx(x, y) for x, y in points], np.int32)


def _draw_robot(img, view, robot):
    """Chassis (translucent), wheels, dashed hull and heading arrow; a disc without the outline fields."""
    centre = view.ipx(robot["x"], robot["y"])
    if "chassis" in robot:
        _dashed(img, [tuple(p) for p in _polygon(view, _body_points(robot, robot["footprint"]))], ROBOT_EDGE,
                closed=True)
        chassis = _polygon(view, _body_points(robot, robot["chassis"]))
        layer = img.copy()
        cv2.fillPoly(layer, [chassis], bgr(ROBOT_FILL), AA)
        cv2.addWeighted(layer, CHASSIS_ALPHA, img, 1.0 - CHASSIS_ALPHA, 0.0, dst=img)
        cv2.polylines(img, [chassis], True, bgr(ROBOT_EDGE), BOLD_PX, AA)
        for wheel in robot["wheels"]:
            poly = _polygon(view, _body_points(robot, wheel["polygon"]))
            cv2.fillPoly(img, [poly], bgr(WHEEL), AA)
            cv2.polylines(img, [poly], True, bgr(WHEEL_EDGE), THIN_PX, AA)
    else:
        cv2.circle(img, centre, int(round(robot["radius"] * view.scale)), bgr(ROBOT_EDGE), BOLD_PX, AA)
    tip = view.ipx(robot["x"] + ARROW_M * math.cos(robot["theta"]), robot["y"] + ARROW_M * math.sin(robot["theta"]))
    cv2.arrowedLine(img, centre, tip, bgr(ROBOT_EDGE), BOLD_PX, AA, tipLength=ARROW_TIP_SHARE)
    cv2.circle(img, centre, TRAIL_DOT_PX, bgr(ROBOT_EDGE), cv2.FILLED, AA)


def _text_size(text, scale=FONT_SCALE):
    """(width, height incl. descent) of text in pixels."""
    (w, h), baseline = cv2.getTextSize(text, FONT, scale, THIN_PX)
    return w, h + baseline


def _label_box(obj, anchor, taken, bounds):
    """(x, y, w, h) of an object's label: right of its outline, moved down past labels already placed."""
    text_w, text_h = _text_size(_label_text(obj))
    w, h = SWATCH_PX + LABEL_PAD_PX * 3 + text_w, text_h + 2 * LABEL_PAD_PX
    x = min(max(anchor[0] + LABEL_GAP_PX, bounds[0]), bounds[2] - w)
    y = min(max(anchor[1], bounds[1]), bounds[3] - h)
    for _ in range(MAX_LABEL_TRIES):
        if not any(x < tx + tw and tx < x + w and y < ty + th and ty < y + h for tx, ty, tw, th in taken):
            break
        y = min(y + h + LABEL_SPACING_PX, bounds[3] - h)
    return x, y, w, h


def _label_text(obj):
    return f"{obj['id']} {obj['class']} ({obj['color']})"


def _draw_objects(img, view, objects):
    """A light wash of each object's own color, amber outlines, then one label per object with its color as
    a swatch."""
    polys = [_polygon(view, [(p["x"], p["y"]) for p in obj["polygon"]]) for obj in objects]
    layer = img.copy()
    for obj, poly in zip(objects, polys):
        if obj.get("color_rgb"):
            cv2.fillPoly(layer, [poly], bgr(obj["color_rgb"]), AA)
    cv2.addWeighted(layer, OBJECT_WASH_ALPHA, img, 1.0 - OBJECT_WASH_ALPHA, 0.0, dst=img)
    for poly in polys:
        cv2.polylines(img, [poly], True, bgr(OBJECT_EDGE), BOLD_PX, AA)
    bounds = (MARGIN_LEFT, MARGIN_TOP, MARGIN_LEFT + view.width, MARGIN_TOP + view.height)
    taken = []
    for obj, poly in sorted(zip(objects, polys), key=lambda item: int(item[1][:, 1].min())):
        box = _label_box(obj, (int(poly[:, 0].max()), int(poly[:, 1].min())), taken, bounds)
        taken.append(box)
        x, y, w, h = box
        backing = img[y:y + h + 1, x:x + w + 1]         # only the label's own pixels, not a copy of the picture
        backing[:] = (backing * (1.0 - LABEL_ALPHA) + np.array(bgr(PAPER)) * LABEL_ALPHA).astype(np.uint8)
        cv2.rectangle(img, (x, y), (x + w, y + h), bgr(OBJECT_EDGE), THIN_PX)
        top = y + (h - SWATCH_PX) // 2
        corners = (x + LABEL_PAD_PX, top), (x + LABEL_PAD_PX + SWATCH_PX, top + SWATCH_PX)
        cv2.rectangle(img, *corners, bgr(obj.get("color_rgb") or NO_COLOR), cv2.FILLED)
        cv2.rectangle(img, *corners, bgr(INK), THIN_PX)
        cv2.putText(img, _label_text(obj), (x + SWATCH_PX + 2 * LABEL_PAD_PX, y + h - LABEL_PAD_PX - TEXT_DROP_PX),
                    FONT, FONT_SCALE, bgr(INK), THIN_PX, AA)


def _ticks(low, high):
    """The multiples of GRID_STEP_M in [low, high], metres."""
    return [k * GRID_STEP_M for k in range(int(math.ceil(low / GRID_STEP_M)), int(math.floor(high / GRID_STEP_M)) + 1)]


def _draw_axes(img, view):
    """Ticks and metre labels every GRID_STEP_M under and left of the panel."""
    bottom, left = MARGIN_TOP + view.height, MARGIN_LEFT
    for x in _ticks(view.x0, view.x1):
        u, _ = view.ipx(x, 0.0)
        cv2.line(img, (u, bottom), (u, bottom + TICK_PX), bgr(MUTED), THIN_PX)
        text = f"{x:g}"
        cv2.putText(img, text, (u - _text_size(text)[0] // 2, bottom + TICK_LABEL_BELOW_PX), FONT, FONT_SCALE,
                    bgr(MUTED), THIN_PX, AA)
    for y in _ticks(view.y0, view.y1):
        _, v = view.ipx(0.0, y)
        cv2.line(img, (left - TICK_PX, v), (left, v), bgr(MUTED), THIN_PX)
        text = f"{y:g}"
        cv2.putText(img, text, (left - TICK_LABEL_GAP_PX - _text_size(text)[0], v + TICK_LABEL_HALF_PX), FONT,
                    FONT_SCALE, bgr(MUTED), THIN_PX, AA)
    cv2.putText(img, "m", (MARGIN_LEFT + view.width - UNIT_LEFT_PX, bottom + UNIT_BELOW_PX), FONT, FONT_SCALE,
                bgr(MUTED), THIN_PX, AA)


def _swatch(img, x, y, fill, edge, hatch=False):
    """A legend square at (x, y), hatched like the unseen floor when asked."""
    patch = img[y:y + LEGEND_SWATCH_PX, x:x + LEGEND_SWATCH_PX]
    patch[:] = bgr(fill)
    if hatch:
        rows, cols = np.indices(patch.shape[:2])
        patch[(rows + cols) % HATCH_PX == 0] = bgr(HATCH)
    cv2.rectangle(img, (x, y), (x + LEGEND_SWATCH_PX - 1, y + LEGEND_SWATCH_PX - 1), bgr(edge), THIN_PX)


def _draw_legend(img, view):
    """Swatches of what the colors mean, and a 1 m scale bar."""
    y = MARGIN_TOP + view.height + MARGIN_BOTTOM + LEGEND_TOP_PX
    x = MARGIN_LEFT
    entries = ((FREE, MUTED, False, "free floor (seen or driven)"), (OCCUPIED, OCCUPIED, False, "occupied"),
               (UNSEEN, MUTED, True, "unseen"), (PAPER, OBJECT_EDGE, False, "object"),
               (ROBOT_FILL, ROBOT_EDGE, False, "robot"), (TRAIL, TRAIL, False, "robot trail"))
    for fill, edge, hatch, text in entries:
        _swatch(img, x, y, fill, edge, hatch)
        cv2.putText(img, text, (x + LEGEND_SWATCH_PX + LEGEND_TEXT_GAP_PX, y + LEGEND_SWATCH_PX - LEGEND_TEXT_RAISE_PX),
                    FONT, FONT_SCALE, bgr(INK), THIN_PX, AA)
        x += LEGEND_SWATCH_PX + LEGEND_ITEM_GAP_PX + _text_size(text)[0]
    bar = int(round(view.scale))                  # one metre
    x = max(x, img.shape[1] - MARGIN_RIGHT - bar - SCALE_BAR_ROOM_PX)
    bar_y = y + SCALE_BAR_Y_PX
    cv2.line(img, (x, bar_y), (x + bar, bar_y), bgr(INK), BOLD_PX)
    for end in (x, x + bar):
        cv2.line(img, (end, bar_y - SCALE_BAR_END_PX), (end, bar_y + SCALE_BAR_END_PX), bgr(INK), BOLD_PX)
    cv2.putText(img, "1 m", (x + bar + SCALE_LABEL_DX_PX, y + SCALE_LABEL_DY_PX), FONT, FONT_SCALE, bgr(INK), THIN_PX,
                AA)


def _title(scene):
    """One line: map id, room size, the robot's pose and what put it there."""
    robot, history = scene["robot"], scene.get("pose_history") or [{}]
    heading = math.degrees(math.atan2(math.sin(robot["theta"]), math.cos(robot["theta"])))
    last = history[-1]
    return (f"map {scene.get('map_id', '?')}  room {scene['room']['width']:g} x {scene['room']['length']:g} m  "
            f"robot ({robot['x']:.2f}, {robot['y']:.2f}) m {heading:+.0f} deg  "
            f"[{last.get('source', '')}{', predicted' if last.get('predicted') else ''}]")


def draw(scene, grid, plan_path=None):
    """The map picture (BGR): known grid, objects, trail, plan_path [(x, y)] if given, robot, legend."""
    view = _view(scene, plan_path)
    img = np.empty((MARGIN_TOP + view.height + MARGIN_BOTTOM + LEGEND_H,
                    MARGIN_LEFT + view.width + MARGIN_RIGHT, 3), np.uint8)
    img[:] = bgr(PAPER)
    img[MARGIN_TOP:MARGIN_TOP + view.height, MARGIN_LEFT:MARGIN_LEFT + view.width] = _panel(grid, view)
    room = scene["room"]
    cv2.rectangle(img, view.ipx(0.0, room["length"]), view.ipx(room["width"], 0.0), bgr(ROOM_EDGE), THIN_PX)
    trail = [view.ipx(h["x"], h["y"]) for h in scene.get("pose_history", [])]
    if len(trail) > 1:
        _dashed(img, trail, TRAIL, BOLD_PX)
    for point in trail:
        cv2.circle(img, point, TRAIL_DOT_PX, bgr(TRAIL), cv2.FILLED, AA)
    if plan_path:
        cv2.polylines(img, [_polygon(view, plan_path)], False, bgr(PATH), BOLD_PX, AA)
        cv2.circle(img, view.ipx(*plan_path[-1]), GOAL_DOT_PX, bgr(PATH), cv2.FILLED, AA)
    _draw_objects(img, view, scene["objects"])
    _draw_robot(img, view, scene["robot"])
    _draw_axes(img, view)
    _draw_legend(img, view)
    cv2.putText(img, _title(scene), (MARGIN_LEFT, MARGIN_TOP - TITLE_ABOVE_PX), FONT, TITLE_SCALE, bgr(INK), THIN_PX,
                AA)
    return img


def png_bytes(scene, grid, plan_path=None):
    """draw() encoded as PNG."""
    ok, data = cv2.imencode(".png", draw(scene, grid, plan_path))
    if not ok:
        raise ValueError("PNG encoding failed")
    return data.tobytes()


def write_png(scene, grid, path):
    """draw() into path, atomically (a viewer never reads half a file). Returns path."""
    tmp = path + ".tmp.png"
    if not cv2.imwrite(tmp, draw(scene, grid)):
        raise OSError(f"cannot write {tmp}")
    os.replace(tmp, path)
    return path


def _parse_args():
    parser = argparse.ArgumentParser(description="The real room's map as a PNG.")
    parser.add_argument("--map", help="a published map id (default: latest.json, with the robot's current pose)")
    parser.add_argument("--out", help=f"default: {map_store.MAPS_DIR}/latest.png or map_<id>.png")
    return parser.parse_args()


def main():
    args = _parse_args()
    scene = map_store.load_map(args.map) if args.map else map_store.load_latest()
    if scene is None:
        print(f"no map {args.map}" if args.map else "no map yet: python3 vision/drive_map.py --run <run> --publish")
        return 1
    default = map_store.IMAGE_PATTERN.format(id=args.map) if args.map else map_store.LATEST_IMAGE
    print(write_png(scene, map_store.load_grid(scene), args.out or os.path.join(map_store.MAPS_DIR, default)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
