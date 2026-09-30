# Session 2026-09-28 → 29 — VO-first motion, natural commands, the real room's map

Tasks: `agent/tasks/2026-09-27_explore-map.md` (Pha 1' A–E done, F waits for the robot) and
`agent/tasks/2026-09-28_natural-command.md` (steps 1–13 done, status testing).

## Found
- **Backward legs cut short by the ESP32** (tape 24 / 23 cm per 40, forward 38 / 37): same peak speed, whole
  trapezoid ~1.6× fast in time, K ~1.7 s early, `O` still sums −0.400. Firmware logic is symmetric on the host
  (215 ticks both ways) → hardware-only, mechanism unknown. Seen only on backward legs right after a forward one.
- **The whole USB hub (CH340 + Astra + WiFi) reset mid-leg** (22:18:59): serial and camera died, the ESP32
  finished the leg alone (34 cm). Jetson and motors have separate supplies; the user defers power fixes.
- VO / tape 0.943 over 4 legs (yesterday 0.974) → `vision/vo_scale.json` factor 1.063 for this mount.
- Gemini 2.5 Flash free tier: 5 requests / min and **20 / day** per model + project. The first key was revoked
  by Google as "leaked" (not tracked in git; source unknown); the user made a new one.
- MV's legs run straight between its waypoints, not along its dense grid path — anything checking a route must
  walk the waypoint polyline.

## Implemented
- VO is the truth for motion: `drive_map` positions from VO × vo_scale; `alignment.txt` flags legs the firmware
  acked early (`CUT`) and weak VO; `vision/leg_odometry.py` (`measure_leg`); `communication/leg_executor.py`
  (`demo_drive --compensate`: one leg at a time, measured, topped up ≤ 2×, stuck / camera-lost stops);
  `communication/slip_model.py` (k per floor × direction, never learns cut or weak legs).
- Natural commands: prompt v1.2 (turns in any word order, sideless turns, revolutions, purpose clauses, typos,
  English, `goal`); rule parser with the same vocabulary, continuations, context-aware questions (answer goes
  where the question pointed, no reading may drop a step); intent check in stage [2];
  `natural_commands.json` (40 cases) + `test_live_parser.py`.
- `realroom/` (parallel to `simulation/room`, which is untouched): `map_store.py` (maps/latest.json + known grid,
  robot pose + history), `planner.py` (MV + swept-disc check: no occupied cell, ≤ 35 % unseen, start not moved),
  `app.py` + viewer on :5002. `drive_map --publish` writes it; the robot's footprint counts as free.
- `demo_drive`: goals ("đi tới cái ghế") → `goto.py` (CM grounding with Gemini, rules without) → route → drive;
  start / end pose on the map in run.json, map moved by VO (or predicted), not moved after an early stop.

## Decisions
- Trust VO over commands; never learn a firmware-cut leg as slip; explore with forward legs (MV turn–go–turn).
- LLM copies words (numbers, place names); code decides every number, side and coordinate.
- The rules finish an exchange the LLM failed in, re-reading all of its messages.

## Pending
- Robot: `demo_drive --compensate "đi 40 cm"` / `"lùi 40 cm"` ×3 → tape 40 ± 3 cm; `"rẽ trái"`, `"rẽ phải"`;
  `"đi tới cái ghế"` on map 20260928_222009 (robot placed where that run ended).
- Gemini live corpus cases 9–40: `python3 communication/test_live_parser.py --first 9` after the quota resets.
- Firmware: why the motion loop runs fast on backward legs; USB hub power / ground.
- explore-map Pha 2 (scan_match, multi-stop maps into one world frame) onwards.

## Interface changes
`interfaces.md`: drive command JSON v1.2 (`turn`, `rev`, `goal`), `realroom` (files, API, port 5002), run.json
(`start_pose`, `end_pose`, `executed_legs`, `requests`, events `leg_acked` / `leg_measured`), `slip_profiles.json`,
`vo_scale.json`. `mc_client.constant(name)`; `drive_timeline.Leg.ack_t / waits_ready`; `OccupancyMap.copy /
integrate_footprint / known_grid`; `drive_map --vo-scale --publish`.
