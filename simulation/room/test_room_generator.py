"""Smoke test: invariants over many seeds. Run: python test_room_generator.py [count]"""
import math
import sys

from room_generator import (MAX_POLYGON_VERTICES, generate, polygon_inside_room,
                            polygons_overlap, robot_footprint)

REQUIRED_CLASSES = ("bed", "table", "chair", "fan")
DOOR_WIDTH_TOL_M = 0.01
DEFAULT_SEED_COUNT = 50


def check_scene(scene):
    errors = []
    w, l = scene["room"]["width"], scene["room"]["length"]
    polys = [[(p["x"], p["y"]) for p in o["polygon"]] for o in scene["objects"]]
    classes = [o["class"] for o in scene["objects"]]

    for cls in REQUIRED_CLASSES:
        if cls not in classes:
            errors.append(f"missing class {cls}")

    for o, poly in zip(scene["objects"], polys):
        if not 3 <= len(poly) <= MAX_POLYGON_VERTICES:
            errors.append(f"{o['id']} has {len(poly)} vertices")
        if not polygon_inside_room(poly, w, l, 0.0):
            errors.append(f"{o['id']} outside room")
        if not o["free_sides"] and o["class"] in ("bed", "table"):
            errors.append(f"{o['id']} {o['class']} has no free side")
        if len(o["near"]) == 0:
            errors.append(f"{o['id']} has no near landmark")

    for i in range(len(polys)):
        for j in range(i + 1, len(polys)):
            if polygons_overlap(polys[i], polys[j]):
                errors.append(f"{scene['objects'][i]['id']} overlaps {scene['objects'][j]['id']}")

    r = scene["robot"]
    disc = robot_footprint(r["x"], r["y"], r["theta"])
    if not polygon_inside_room(disc, w, l, 0.0):
        errors.append("robot outside room")
    for o, poly in zip(scene["objects"], polys):
        if polygons_overlap(disc, poly):
            errors.append(f"robot overlaps {o['id']}")

    d = scene["doors"][0]
    on_wall = (d["p1"]["x"] == d["p2"]["x"] and d["p1"]["x"] in (0.0, w)) or \
              (d["p1"]["y"] == d["p2"]["y"] and d["p1"]["y"] in (0.0, l))
    if len(r["chassis"]) != 6 or len(r["wheels"]) != 3:
        errors.append("robot shape fields missing")
    if not on_wall:
        errors.append("door not on a wall")
    if abs(math.hypot(d["p2"]["x"] - d["p1"]["x"], d["p2"]["y"] - d["p1"]["y"]) - d["width"]) > DOOR_WIDTH_TOL_M:
        errors.append("door width mismatch")
    return errors


def main(count):
    failed = 0
    for seed in range(1, count + 1):
        errors = check_scene(generate(seed))
        if errors:
            failed += 1
            print(f"seed {seed}: " + "; ".join(errors))
    print(f"{count - failed}/{count} seeds passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_SEED_COUNT))
