# CM — LLM target grounding (`project/src/CM/`)

Roadmap layers L2 (LLM grounding + dialog) and L3 (candidate geometry) over the scene JSON produced by
`simulation/room/`. The user types a place in natural language, the LLM fills three slots
(**target** object → **side** of it → **spot** along that side), asking one short question per missing
slot, until exactly one precomputed reachable point is left. The UI highlights the candidates after
every turn, draws the robot hull at the final goal, and paints the route MV plans to it (L4).

The LLM never sees coordinates. Code enumerates every reachable spot (with a compass + landmark
description), the LLM only picks candidate ids, and a validator rejects any id that is not on the map.

## Run

```bash
cd project/src/CM
# paste your key into .env (git-ignored; template in .env.example):
#   CM_PROVIDER=gemini  GEMINI_API_KEY=...   (default)  |  openai + OPENAI_API_KEY  |  anthropic + ANTHROPIC_API_KEY
python app.py                          # http://localhost:5001
set CM_PROVIDER=fake & python app.py   # no key: scripted replies, UI + geometry only
python test_grounding.py               # offline tests with a fake LLM
python candidates.py --seed 7 --summary
```

Python on this PC: `D:\University\Python\Python310\python.exe`.

| Env var | Default | Meaning |
|---|---|---|
| `CM_PROVIDER` | `gemini` | `gemini` (OpenAI-compatible endpoint, uses the installed `openai` SDK) · `openai` · `anthropic` (`pip install anthropic`) · `fake` |
| `CM_GEMINI_MODEL` | `gemini-2.5-flash` | Gemini model |
| `CM_OPENAI_MODEL` | `gpt-4o-mini` | OpenAI chat model |
| `CM_ANTHROPIC_MODEL` | `claude-opus-5` | Claude model (structured JSON output, server-side refusal fallbacks) |

## Dialog example

```
user: near my bed
bot : Which side of the bed: the north side (near the book) or the east side (near the chair)?   [6 spots highlighted]
user: east
bot : Along the east side: the middle, or the south-east corner near the door?                 [2 spots]
user: the corner
bot : Target: south-east corner of the bed → (1.568, 0.593)                                    [goal + hull drawn]
```

Slots with a single option are filled by code, so an object with one free side and one spot is
resolved without a question.

## Files

```
config.py             constants (paths, clearance, provider/model, retries)
candidates.py         L3: object x free side -> spots corner_a/center/corner_b, collision-filtered,
                      compass + nearest-landmark descriptions; scene_summary() = text the LLM reads
prompts/grounding.json  editable prompt: system text, rules, output schema, few-shot examples
llm_client.py         GeminiClient / OpenAIClient / AnthropicClient / FakeClient, all: complete(system, messages, schema) -> dict
dialog.py             GroundingSession: prompt build, validator (+1 retry), auto-fill, state machine
mv_client.py          L4 bridge: ctypes wrapper over the MV C library (plan_path, grid, status_text)
mc_client.py          L5 bridge: ctypes wrapper over the MC executor (run -> per-tick wheel speeds)
app.py                Flask :5001 — /api/scene/new|latest, /api/ground/say|reset, /api/prompt,
                      /api/plan, /api/trajectory
static/index.html     canvas renderer + chat panel; dim dots = all spots, pink = still possible, green = goal,
                      purple polyline = planned path (dotted = A* cells, solid = waypoints),
                      orange robot + wheel-speed chart = the MC trajectory playing back
test_grounding.py     collision-free spots over 200 seeds, bed funnel, auto-fill, validator, "none"
```

## Candidate spots

For every object and each entry of its `free_sides`, the robot hull is placed outside that side at
`half_extent + hull_radius + 0.05 m`, facing the object. Sides at least 0.45 m long get three spots
(`corner_a` = CCW end, `center`, `corner_b`); shorter sides only `center`. Spots whose hull leaves the
room or overlaps another object are dropped. Each door gets one spot just inside it.

Candidate id: `obj_3:left:corner_a`. Description: `"north-east corner of the bed, nearest landmark
obj_4 (chair, 0.7 m away)"`. Screen mapping given to the LLM: top = north, right = east.

## LLM reply schema

```json
{"type": "ask" | "goal" | "none",
 "question": "one short sentence",
 "slots": {"target": "obj_1", "side": "right", "spot": null},
 "matches": ["obj_1:right:*"]}
```

`matches` accepts exact ids and wildcards `obj_1:*` / `obj_1:right:*`. The validator checks target
exists, side ∈ `free_sides`, spot exists; on failure it re-asks the LLM once with the error, then falls
back to a code-generated question. Whatever the LLM says, the response type is decided by code from
the slot state: 1 remaining spot → `goal`, several → `ask`, none → `none`.

## Response to the UI

```json
{"type": "ask", "text": "...", "slots": {...}, "resolved": false,
 "matches": [{"id": "obj_1:right:center", "x": 1.568, "y": 1.53, "theta": -3.141, "description": "..."}]}
```

## Path planning (L4, MV)

After a `goal` turn the page posts the resolved spot to `POST /api/plan`; with no body the server
re-uses the goal the dialog last resolved.

```json
{"ok": true, "path": [{"x": 2.62, "y": 0.38}, ...], "waypoints": [...], "length_m": 1.323,
 "primitives": [{"type": "ROTATE", "a": 1.6137, "b": 0.0}, {"type": "FORWARD", "a": 1.3232, "b": 0.0},
                {"type": "ROTATE", "a": -1.6137, "b": 0.0}, {"type": "STOP", "a": 0.0, "b": 0.0}],
 "snapped_start": {...}, "snapped_goal": {...}, "goal": {...}}
```

MV plans the robot as a disc of `robot.radius`, while `candidates.py` tests the real footprint polygon,
so MV is the stricter of the two: over 40 seeds (1452 spots) 95% plan, 4% are genuinely walled off for a
0.45 m robot, and 0.3% are spots MV will not enter because the disc does not fit where the polygon does.
All three answer with a reason instead of a path through an obstacle.

Unreachable goals come back as `{"ok": false, "status": 5, "reason": "no path between start and goal"}`;
a missing or unloadable library answers 503 with the same shape. Obstacles are inflated by the hull
radius, so goals behind a gap narrower than the robot are correctly reported unreachable rather than
cut through. Build the library first (`project/tools/build_mv.sh`, see the MV section of
`project/README.md`) — it must match this interpreter's bitness, 64-bit here. Set `CM_MV_HOLONOMIC=1`
to get one `MOVE dx dy` per leg instead of ROTATE/FORWARD.

## Motion (L5, MC)

`POST /api/trajectory` runs MV and then the MC executor, returning the plan plus per-tick wheel speeds.
The page calls it after every resolved goal, so the map gains a player: press Play (or drag the
scrubber) and an orange robot walks the path while the chart below shows the three wheel speeds and a
marker at the current tick. The readout prints wheel speed, pulse rate, body velocity and pose.

```json
{"...plan fields...":  "...",
 "origin": {"x": 0.875, "y": 0.375, "theta": 1.571},
 "trajectory": {"ok": true, "step_count": 2636, "stride": 5, "tick_s": 0.02, "duration_s": 52.72,
                "steps": [{"t": 0.0, "u": 0.0, "v": 0.0, "r": 0.0, "w": [0, 0, 0], "hz": [0, 0, 0],
                           "dir": [1, 1, 1], "x": 0.0, "y": 0.0, "theta": 1.571}],
                "end": {"x": -0.166, "y": 3.304, "theta": -1.571}}}
```

Poses in `steps` are displacements from `origin` (the snapped start cell), so the page draws
`origin + step`. Long routes are thinned to `MC_UI_MAX_ROWS` rows by an integer `stride`; the whole
trajectory is still executed, and the player uses `tick_s * stride` per row so playback stays real
time. If the MC library is missing the reply still carries the path, with
`trajectory.ok = false` and a reason. Build it with `project/tools/build_mc.sh`.

## Not done yet

- Sending the resolved pose or the primitives to the robot (`/api/robot/goal`), by design.
- Live API calls are untested in this session (no key on this machine); the Anthropic path follows
  the current SDK docs (`client.beta.messages.create`, `output_config` JSON schema, `fallbacks`).
