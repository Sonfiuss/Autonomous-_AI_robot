"""Offline tests for the grounding funnel with a scripted fake LLM (no API key needed).

Run:  python test_grounding.py
The MV bridge test is skipped when the shared library has not been built yet.
"""
import math
import sys

import config
import mv_client
from candidates import build_candidates
from dialog import GroundingSession, load_prompt
from llm_client import FakeClient

sys.path.insert(0, config.ROOM_DIR)
from room_generator import generate, polygons_overlap, robot_footprint  # noqa: E402

CHECK_SEEDS = 200
SEARCH_SEEDS = 500               # seeds scanned when looking for a scene with a given property
MV_SEED = 7                      # scene used by the MV bridge test
REPLAY_TOL_M = 0.02              # primitive replay must land this close to the last waypoint
REPLAY_TOL_DEG = 1.0             # ...and this close to the goal heading
BLOCKED_POINT_M = 0.01           # room corner: always inside the inflated wall


def _scene_with_multi_side_bed():
    """First seed whose bed has >= 2 free sides and a side with 3 spots."""
    for seed in range(1, SEARCH_SEEDS):
        scene = generate(seed)
        bed = next((o for o in scene["objects"] if o["class"] == "bed"), None)
        if bed is None or len(bed["free_sides"]) < 2:
            continue
        spots = build_candidates(scene)
        for side in bed["free_sides"]:
            if sum(1 for s in spots if s["target_id"] == bed["id"] and s["side"] == side) == 3:
                return seed, scene, bed, side
    raise AssertionError("no suitable seed")


def test_candidates_collision_free():
    for seed in range(1, CHECK_SEEDS + 1):
        scene = generate(seed)
        polys = {o["id"]: [(p["x"], p["y"]) for p in o["polygon"]] for o in scene["objects"]}
        for s in build_candidates(scene):
            foot = robot_footprint(s["x"], s["y"], s["theta"])
            assert all(0 <= x <= scene["room"]["width"] and 0 <= y <= scene["room"]["length"] for x, y in foot), (seed, s["id"])
            for oid, poly in polys.items():
                assert not polygons_overlap(foot, poly), (seed, s["id"], oid)
            if s["target_id"] in polys:
                # faces the object: heading points from spot toward object center
                o = next(o for o in scene["objects"] if o["id"] == s["target_id"])
                dx, dy = o["center"]["x"] - s["x"], o["center"]["y"] - s["y"]
                ang = abs((math.atan2(dy, dx) - s["theta"] + math.pi) % (2 * math.pi) - math.pi)
                assert ang < math.pi / 2, (seed, s["id"], ang)


def test_bed_funnel():
    seed, scene, bed, side = _scene_with_multi_side_bed()
    bid = bed["id"]
    corner = f"{bid}:{side}:corner_a"
    llm = FakeClient([
        {"type": "ask", "question": "Which side of the bed?", "slots": {"target": bid, "side": None, "spot": None},
         "matches": [f"{bid}:*"]},
        {"type": "ask", "question": "Corner or middle?", "slots": {"target": bid, "side": side, "spot": None},
         "matches": [f"{bid}:{side}:*"]},
        {"type": "goal", "question": "", "slots": {"target": bid, "side": side, "spot": "corner_a"},
         "matches": [corner]},
    ])
    s = GroundingSession(scene, llm, load_prompt())
    r1 = s.say("near my bed")
    assert r1["type"] == "ask" and not r1["resolved"], r1
    assert {m["target_id"] for m in r1["matches"]} == {bid}
    assert len(r1["matches"]) == sum(1 for c in s.spots if c["target_id"] == bid)
    r2 = s.say(side)
    assert r2["type"] == "ask" and len(r2["matches"]) == 3, r2
    r3 = s.say("the corner")
    assert r3["type"] == "goal" and r3["resolved"] and [m["id"] for m in r3["matches"]] == [corner], r3
    assert s.slots == {"target": bid, "side": side, "spot": "corner_a"}
    # the map summary is in the system prompt and the history grows 2 per turn
    assert "SPOTS (candidate id" in llm.calls[0][0]
    assert len(llm.calls[2][1]) == 5
    print(f"bed funnel ok (seed {seed}, {bid} side {side})")


def test_autofill_single_option():
    """Object with exactly one free side and one spot: no question, straight to goal."""
    for seed in range(1, SEARCH_SEEDS):
        scene = generate(seed)
        spots = build_candidates(scene)
        single = [o for o in scene["objects"]
                  if sum(1 for s in spots if s["target_id"] == o["id"]) == 1]
        if single:
            break
    obj = single[0]
    llm = FakeClient([{"type": "ask", "question": "Which side?", "slots": {"target": obj["id"], "side": None, "spot": None},
                       "matches": [f"{obj['id']}:*"]}])
    r = GroundingSession(scene, llm, load_prompt()).say(f"go to the {obj['class']}")
    assert r["type"] == "goal" and len(r["matches"]) == 1, r
    print(f"autofill ok (seed {seed}, {obj['id']} {obj['class']})")


def test_validator_rejects_bad_ids():
    scene = generate(7)
    bed = next(o for o in scene["objects"] if o["class"] == "bed")
    llm = FakeClient([
        {"type": "goal", "question": "", "slots": {"target": "obj_99", "side": None, "spot": None}, "matches": ["obj_99:front:center"]},
        {"type": "goal", "question": "", "slots": {"target": bed["id"], "side": "nowhere", "spot": None}, "matches": []},
    ])
    s = GroundingSession(scene, llm, load_prompt())
    r = s.say("near the bed")
    assert len(llm.calls) == 2, "should retry once with the validator error"
    assert "invalid" in llm.calls[1][1][-1]["content"] and len(llm.calls[1][1]) == 1
    assert r["type"] == "ask" and r["matches"] == [] and s.slots["target"] is None, r
    print("validator ok")


def test_none_and_restart():
    scene = generate(7)
    llm = FakeClient([
        {"type": "none", "question": "No piano here.", "slots": {"target": None, "side": None, "spot": None}, "matches": []},
    ])
    s = GroundingSession(scene, llm, load_prompt())
    r = s.say("next to the piano")
    assert r["type"] == "none" and r["matches"] == []
    print("none ok")


def _dist_point_to_polygon(px, py, poly):
    """Shortest distance from a point to a polygon outline; negative when inside."""
    best, inside = float("inf"), False
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        vx, vy = b["x"] - a["x"], b["y"] - a["y"]
        wx, wy = px - a["x"], py - a["y"]
        span = vx * vx + vy * vy
        t = 0.0 if span == 0.0 else max(0.0, min(1.0, (wx * vx + wy * vy) / span))
        best = min(best, math.hypot(px - (a["x"] + t * vx), py - (a["y"] + t * vy)))
        if (a["y"] > py) != (b["y"] > py) and px < a["x"] + (py - a["y"]) / (b["y"] - a["y"]) * vx:
            inside = not inside
    return -best if inside else best


def _replay(start, primitives):
    """Dead-reckons the primitive list from a start pose -> (x, y, theta)."""
    x, y, theta = start["x"], start["y"], start.get("theta", 0.0)
    for p in primitives:
        if p["type"] == "ROTATE":
            theta += p["a"]
        elif p["type"] == "FORWARD":
            x, y = x + p["a"] * math.cos(theta), y + p["a"] * math.sin(theta)
        elif p["type"] == "MOVE":
            x, y = x + p["a"], y + p["b"]
    return x, y, theta


def test_mv_bridge():
    """MV plans through the ctypes bridge: paths clear the obstacles, primitives reach the goal."""
    try:
        mv_client.load_library()
    except mv_client.MvError as exc:
        print("mv bridge SKIPPED:", exc)
        return
    scene = generate(MV_SEED)
    spots = build_candidates(scene)
    robot = scene["robot"]
    # inflation the planner applies; a cell centre may sit up to half a diagonal inside the free zone
    clearance = robot["radius"] - math.hypot(config.MV_CELL_M, config.MV_CELL_M) / 2.0
    planned = 0
    for spot in spots:
        result = mv_client.plan_path(scene, spot)
        assert "ok" in result and "status" in result, result
        if not result["ok"]:
            assert result["reason"], result            # unreachable goals are reported, never silent
            continue
        planned += 1
        assert result["path"] and result["waypoints"], result
        assert result["length_m"] >= 0.0, result       # 0 when the goal spot is the robot's own cell
        for point in result["path"]:
            for obj in scene["objects"]:
                d = _dist_point_to_polygon(point["x"], point["y"], obj["polygon"])
                assert d >= clearance, (spot["id"], obj["id"], round(d, 3), round(clearance, 3))
        # primitives are relative: replay starts at the snapped cell with the robot's own heading
        x, y, theta = _replay(dict(result["snapped_start"], theta=robot["theta"]), result["primitives"])
        end = result["waypoints"][-1]
        assert math.hypot(x - end["x"], y - end["y"]) < REPLAY_TOL_M, (spot["id"], x, y, end)
        heading_error = abs((theta - spot["theta"] + math.pi) % (2 * math.pi) - math.pi)
        assert heading_error < math.radians(REPLAY_TOL_DEG), (spot["id"], theta, spot["theta"])
        assert result["primitives"][-1]["type"] == "STOP", result["primitives"][-1]
    assert planned, f"no spot was reachable in seed {MV_SEED}"
    blocked = mv_client.plan_path(scene, {"x": BLOCKED_POINT_M, "y": BLOCKED_POINT_M, "theta": 0.0})
    assert not blocked["ok"] and blocked["status"] == mv_client.MV_ERR_GOAL_BLOCKED, blocked
    print(f"mv bridge ok ({planned}/{len(spots)} spots reachable, seed {MV_SEED})")


def test_session_tracks_goal():
    """The dialog exposes its resolved spot so /api/plan can default to it."""
    seed, scene, bed, side = _scene_with_multi_side_bed()
    corner = f"{bed['id']}:{side}:corner_a"
    llm = FakeClient([
        {"type": "ask", "question": "Which side?", "slots": {"target": bed["id"], "side": None, "spot": None},
         "matches": [f"{bed['id']}:*"]},
        {"type": "goal", "question": "", "slots": {"target": bed["id"], "side": side, "spot": "corner_a"},
         "matches": [corner]},
    ])
    s = GroundingSession(scene, llm, load_prompt())
    assert s.resolved_goal() is None
    s.say("near my bed")
    assert s.resolved_goal() is None, "still ambiguous"
    s.say(f"the {side} corner")
    goal = s.resolved_goal()
    assert goal is not None and goal["id"] == corner, goal
    s.reset()
    assert s.resolved_goal() is None
    print(f"session goal tracking ok (seed {seed}, {corner})")


if __name__ == "__main__":
    test_candidates_collision_free()
    print(f"candidates collision-free over {CHECK_SEEDS} seeds")
    test_bed_funnel()
    test_autofill_single_option()
    test_validator_rejects_bad_ids()
    test_none_and_restart()
    test_session_tracks_goal()
    test_mv_bridge()
    print("ALL OK")
