# communication module plan (`communication/`)

## Purpose
Text (later: voice) command → robot motion, without a map. First deliverable: the drive demo —
"đi thẳng 30 cm, rẽ trái, đi tiếp 60 cm" parsed by an LLM, turned stage by stage into primitives,
UART lines and per-wheel numbers, driven through SEQ/robot_link while `vision/record.py` films.
This is the `primitive` branch of L2 in `agent/description/robot_process_llm.md` (skips L3/L4).
Task: `agent/tasks/2026-09-25_llm-drive-demo.md`.

## Architecture
```
text → cmd_parser (LLM via CM/llm_client | rule regex) → ONE validator → MotionStep[]
     → motion_plan: FORWARD/ROTATE/STOP (plan.txt) → wire preview (F/T/S) → wheels via MC (ctypes)
     → demo_drive: robot_link --run-plan (SEQ, one leg in flight)  ∥  vision/record.py → vision/output/<run>/
```
No new wire format and no new C++: the demo reuses SEQ's plan-file contract and MC's C API.

## Decisions (2026-09-25)
- **The LLM transcribes, the code decides.** Every number in an LLM reply must appear in the user's
  own words (digits or a single number word); otherwise re-ask once, then ask the user. Defaults
  (90° for a bare "rẽ trái", 180° for "quay đầu") are filled in code, never by the LLM.
- **Rule parser as offline fallback**, same raw shape, same validator. It refuses a whole command
  when any number or motion word is left unmatched — dropping a step silently is worse than asking.
- **`auto` falls back, an explicit provider does not** (same philosophy as `detector.select_device`).
- **Turns are chunked below 180°** because SEQ wraps headings into (−180°, 180°].
- **Recorder in its own session**, robot_link in the terminal's process group: Ctrl-C reaches
  robot_link directly (abort + `S`), and the orchestrator closes the video afterwards.
- **Video paced to wall time** (frames repeated when inference is slower), so it lines up with
  `run.json`'s timeline.
- **Numbers must also come in the user's order** (single message): catches an LLM that swaps two
  real numbers between steps. Off across a question/answer, where an answer fills an earlier step.
- **Video shows the free floor and a distance for every box** (user, 2026-09-25): Depth Anything +
  `analyze_floor` on one worker thread (newest frame wins, ~0.45 s per result on the Orin); distance
  measured by the Astra when it has depth, else estimated from the box bottom by floor geometry and
  printed with `~`.

## Decisions (2026-09-28, task 2026-09-28_natural-command)
- **Gemini 2.5 Flash is the parser** (`auto`), the rules the offline fallback with the same vocabulary. Free tier:
  5 requests / min and **20 / day** per model and project — the rules take over at 429, finish the exchange in
  progress (re-reading every message of it) and hand the next command back to the LLM.
- **Turns in natural words**: any turn verb + side / angle / revolutions / clockwise in any order; a turn with no side
  is `turn` — 180° / 360° go left, anything else is ASKED; revolutions become degrees in code; `turn_around` takes
  a stated angle. Purpose clauses ("để trở về vị trí ban đầu") are checked against the steps, never turned into steps.
- **Questions carry context**: the understood steps are listed and the answer goes where the question pointed; no
  reading that drops a pending step is ever accepted (an answer once silently dropped a backward step).
- **Places are the map's**: the parser only copies the words ("goal"); CM grounding picks a spot on the REAL map
  (`realroom/`, a module parallel to `simulation/room`, which stays untouched), realroom's planner checks the path
  against what the camera saw. The robot's pose on that map carries from command to command (VO-measured with
  `--compensate`, predicted otherwise, not moved after a run that stopped early).

## Decisions (2026-09-30, task 2026-09-30_realroom-chat-drive)
- **One path for the terminal and the page**: `demo_drive.prepare()` (map request split, parser, questions through an
  `ask` callback, grounding + routing) is what both call; the page never drives by itself - Run starts demo_drive
  `--plan-json` in its own process group, Stop is SIGINT to it (the tested Ctrl-C path).
- **"lập bản đồ" = the scan + drive_map --publish** of the run's own recording; the whole map is replaced (runs are not
  merged yet). Never after a dry run, nor after a run that ended where nobody knows.
- **A route is only driven from where it was planned** (`from_pose`); **one demo_drive at a time** (flock).

## Done
- [x] 2026-09-25 `cmd_parser.py`, `motion_plan.py`, `demo_drive.py`, `drive_config.py`,
      `prompts/drive_command.json`, `test_drive.py` (all green), README.
- [x] 2026-09-25 `vision/record.py`; `result_records` moved into `vision/object_distance.py`.
- [x] 2026-09-25 Floor overlay + clear-ahead + floor-geometry distance (`floor_segment.box_floor_range`,
      `drawing.blend_floor`), `test_vision` check. Jetson env: torchvision 0.16.1 built from source
      (wheel in `~/wheels`), ultralytics `--no-deps`, transformers 4.40.1, DA V2 Small in `vision/models`.
- [x] 2026-09-25 `/code-standards-review` (13 fixed) + `/code-logic-review` (4 fixed).
- [x] 2026-09-25 End to end against a fake ESP32 on a pty: the real `robot_link` + SEQ sent
      `F / T / F / S` one leg per `K`; Ctrl-C mid-leg → `S`, nothing further sent.
- [x] 2026-09-25 Firmware pins from the user in `fw/config.h`; `DIR_INVERTED[]` flag; `sdkconfig.defaults`
      silences IDF logs on UART0.

- [x] 2026-09-28 natural commands: prompt v1.2, rule turn grammar + continuations + purpose / return clauses,
      context dialogue, intent check, `natural_commands.json` (40 cases) — rules 38 / 38 (2 cases LLM-only), Gemini 10 / 10 of the
      cases it reached before the daily quota; `goto.py` + `realroom/` (map_store, planner, app :5002);
      `--compensate` (`leg_executor.py`, `slip_model.py`) from task 2026-09-27_explore-map.

- [x] 2026-09-30 realroom web chat drives the robot (`realroom/drive_chat.py`, app endpoints, page); demo_drive
      `--plan-json` / `--map` / `--run-dir`, "lập bản đồ" auto-publish, robot flock; `realroom/test_chat.py` 9/9.
- [ ] 2026-09-30 realroom chat UI (task 2026-09-30_realroom-chat-ui): chat = go to a place on the map it has
      (target named + A* route → Run → "Đã tới …"), no mapping from the web; wide chat column; rule goals with
      color; Gemini 3.8-flash → 3.5-flash-lite on 429 / 503. Code + tests done (test_chat 11/11); waiting for the
      user's real run.
      Waiting: the real-robot trial (task step 7).

## Pending
- [ ] ESP-IDF install needs `sudo apt-get install gperf python3-venv ninja-build ccache dfu-util`
      (password), then `tools/esp32_setup_idf.sh`, build, back up the current flash, flash.
- [ ] Smoke test on the robot: `--forward 0.1` (goes forward? W2 still?), `--turn 90` (CCW?); set
      `DIR_INVERTED` accordingly; measure 30 cm / 90° with a tape. The 2026-06-29 memory recorded an
      effective roll radius of 4.03 cm vs the 5.5 cm in `constants.h` — calibrate before trusting distances.
- [ ] Servo: the camera servos are on a PCA9685 (I2C SDA 21 / SCL 17); `servo_driver.cpp` drives LEDC
      pins instead. Not needed by the demo.
- [x] LLM key in `project/src/CM/.env` (Gemini, 2026-09-28; the first key was revoked by Google as leaked).
      On this Jetson (Python 3.8) only gemini/openai work: anthropic 0.72 is the last SDK for 3.8.
- [ ] Gemini live corpus: `test_live_parser.py --first 9` once the daily quota resets (cases 9–40 unrun).
- [ ] Real robot: `demo_drive --compensate "đi tới cái ghế"` on map 20260928_222009.
- [ ] Astra depth on the Jetson: the system OpenNI2 (PS1080 driver) does not see the Astra Pro; needs
      Orbbec's OpenNI2 SDK for Linux ARM64. Until then every distance is the `~` floor estimate.
- [ ] Measure the camera pitch on the robot (`--cam-pitch`): 1 deg moves the floor at 3 m by ~0.3 m.
- [ ] Voice input (INMP441 is wired to the ESP32, not the Jetson) — later.
