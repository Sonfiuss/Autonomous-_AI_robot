# 2026-09-25 — communication: text command → LLM → wheels, with a detection video

Task: `agent/tasks/2026-09-25_llm-drive-demo.md` (status `executing`: everything but the flash).
Plan: `agent/plan/comm_plan.md`.

## Implemented
- **`communication/`** (new module): `cmd_parser.py` (LLM via CM's `llm_client` + offline rule parser,
  one validator), `motion_plan.py` (steps → FORWARD/ROTATE/STOP → F/T/S preview → per-wheel pulses,
  Hz, rad/s via MC ctypes), `demo_drive.py` (orchestrator: stages printed, recorder started, `robot_link
  --run-plan`, `run.json`), `drive_config.py`, `prompts/drive_command.json`, `test_drive.py`, README.
- **`vision/record.py`** (new): headless recorder — YOLO boxes with a distance on each, Depth Anything
  free-floor overlay + "clear ahead", motion step bar; `detections.mp4` paced to wall time +
  `detections.jsonl`. `floor_segment.box_floor_range`, `drawing.blend_floor`,
  `object_distance.result_records` (moved from perception.py, gained `range_source`).
- **Firmware**: user's pin map in `fw/config.h` (DM556 PUL/DIR 26/27, 12/13, 4/5), `DIR_INVERTED[]`
  applied in `step_dir_driver.cpp`, `sdkconfig.defaults` silencing IDF logs on UART0.
- **Jetson environment** (first time anything of this ran here): `libmc.so` and `robot_link` built;
  torchvision 0.16.1 built from source (the pip one had a broken `_C.so` ABI → YOLO NMS crashed), wheel
  kept in `~/wheels`; `pip --user`: ultralytics 8.4.162 `--no-deps`, transformers 4.40.1 (+ pinned
  tokenizers 0.19.1 / safetensors 0.4.3 / hf-hub 0.23.4 — newer ones need Rust on py3.8), openai 1.89,
  anthropic 0.72, openni, python-dotenv. NVIDIA torch untouched.

## Decisions
- LLM transcribes only. Every number it returns must appear in the user's words, **in the same order**
  within one message; defaults (90° turn, 180° U-turn) are code's. `auto` provider falls back to the
  rules; an explicit provider never does.
- Turns chunked below 180°: SEQ wraps headings into (−180°, 180°], so an unchunked right 180 drives left.
- Recorder in its own session; robot_link in the terminal's group so Ctrl-C reaches it directly.
- Floor distance estimate (`~`) from the box bottom + camera height/pitch, because the Astra depth does
  not open on the Jetson (below). Requested by the user mid-task, together with the DA floor overlay.

## Verified
- `test_drive.py` (all), `test_vision.py` (5 checks, new `check_box_floor_range`), `test_mapping.py`,
  `test_mc`, `test_motion` — green.
- Real `robot_link` + SEQ against a fake ESP32 on a pty: `F / T / F / S`, one leg per `K`; a `READY`
  mid-plan → REBOOTED + `S`; Ctrl-C mid-leg → `S`, nothing further sent.
- Full dry-run with vision: recorder ready in ~12 s (YOLO + DA load), video 10 fps, floor on 80/84
  frames, YOLO ~40 ms and DA ~290 ms (518 px, fp16) on the Orin.

## Pending
- **ESP-IDF needs `sudo apt-get install -y gperf python3-venv ninja-build ccache dfu-util`** (password),
  then `tools/esp32_setup_idf.sh`, `tools/esp32_flash.sh --build-only`, back up the current flash
  (`esptool.py read_flash`), flash. Nothing has been flashed; the ESP32 still runs whatever it had.
- On the robot: `robot_link --forward 0.1` (forward? W2 still?) and `--turn 90` (CCW?) → set
  `DIR_INVERTED`; tape-measure 30 cm / 90°. A 2026-06-29 memory recorded an effective roll radius of
  4.03 cm against the 5.5 cm in `constants.h`.
- Astra depth: system OpenNI2 (PS1080) does not see the Astra Pro → needs Orbbec's ARM64 SDK.
- LLM key in `project/src/CM/.env`; on py3.8 only gemini/openai work (anthropic 0.72 lacks what CM passes).
- Camera pitch on the robot unmeasured; servo driver speaks LEDC but the servos are on a PCA9685.
- `vision/models/` is not git-ignored although `vision/README.md` says it is (weights ~100 MB).

## Interface changes
None to the wire or the C APIs. New consumer of the SEQ plan-file contract (`communication/`), new
jsonl fields (`range_source`, `floor`), new `drawing.format_range` convention (`~` = estimate).
