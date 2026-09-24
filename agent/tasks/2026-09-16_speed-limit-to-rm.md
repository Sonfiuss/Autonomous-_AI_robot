---
id: 2026-09-16_speed-limit-to-rm
status: testing
module: project/ (RM + MC) + motivation/esp32_unified_controller
started: 2026-09-16
---

## Task
Move `speed_limit` out of MC and into RM, then reposition MC in the documentation as a **tool**
(preview / feasibility check) rather than a stage of the live pipeline. Option B, approved by the
user 2026-09-16.

## Input
- `speed_limit` is pure chassis maths — given a direction of travel it returns the feasible speed and
  ramp — so it belongs with the rest of the chassis maths in RM. It ended up in MC only because MC
  was the first module that needed it.
- The concrete symptom: `motivation/esp32_unified_controller/src/motion_state.cpp` includes
  `"MC/speed_limit.h"` and `build_test_motion.sh` compiles `src/MC/speed_limit.cpp`. The firmware
  drags in the MC module for one maths function it has no other use for.
- Option A (2026-09-16) already made MC a preview: the ESP32 recomputes the trajectory live from
  `F`/`T`. MC never drives the robot. The documentation still presents it as a pipeline stage, which
  is what made this confusing in the first place.
- Rejected alternatives: deleting MC entirely (loses `/api/trajectory`, the wheel-speed chart the
  user asked for on 2026-09-13, and `test_mc`'s end-to-end odometry replay); merging MC's `Executor`
  with the firmware's `MotionState` (correct in principle, but a refactor of working tested code, and
  the two genuinely differ — batch walk vs. reactive tick).

## Expected output
- `rm::limitsFor` / `rm::peakWheelOmega` / `rm::AxisLimits` live in `project/{include,src}/RM/`.
- `MC/speed_limit.h` and `src/MC/speed_limit.cpp` are gone; nothing includes them.
- The firmware compiles against RM only — no MC path in `build_test_motion.sh`.
- `test_rm`, `test_mv`, `test_mc`, `test_link`, `test_motion` all still pass.
- README, `system_architecture.md` and `mc_plan.md` describe MC as a tool, not a pipeline stage.

## Plan
- [x] 1. This task file.
- [x] 2. `config/constants.h`: `MIN_WHEEL_COEFF` moves from `mc::cfg` to `rm::cfg`.
- [x] 3. New `include/RM/speed_limit.h` — `AxisLimits` + `peakWheelOmega` + `limitsFor`, namespace `rm`.
- [x] 4. New `src/RM/speed_limit.cpp` — same code, `RM_DLOG`.
- [x] 5. Delete `include/MC/speed_limit.h` + `src/MC/speed_limit.cpp`; drop `AxisLimits` from `MC/types.h`.
- [x] 6. Repoint callers: `MC/executor.{h,cpp}`, `MC/mc_api.cpp`, `tests/test_mc.cpp`,
        `motivation/esp32_unified_controller/src/motion_state.cpp`.
- [x] 7. Builds: `project/CMakeLists.txt` (RM_SOURCES gains it, MC_SOURCES loses it),
        `tools/build_mc.sh`, `motivation/esp32_unified_controller/tools/build_test_motion.sh`.
- [x] 8. Rebuild and run all five suites.
- [x] 9. Docs: `project/README.md`, `agent/description/system_architecture.md`, `agent/plan/mc_plan.md`,
        `agent/plan/rm_plan.md` — MC is a tool; state what it IS for and what it is NOT.
- [x] 10. History + action log.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[--:--] steps 2-5: speed_limit moved MC -> RM, namespace mc:: -> rm::, MC_DLOG -> RM_DLOG; AxisLimits left MC/types.h; MIN_WHEEL_COEFF left mc::cfg | info: the move is pure relocation, not one line of logic changed
[--:--] steps 6-7: four callers repointed, three build files updated | info: test_mc.cpp's include of speed_limit.h turned out to be unused and was dropped rather than repointed
[--:--] step 8: test_mc PASS, test_motion PASS, test_mv PASS, test_link PASS, test_rm PASS | info: firmware re-syntax-checked against the IDF stubs, zero warnings, and `grep MC/ src include` in the firmware now returns nothing — it compiles against RM alone
[--:--] step 9: MC reframed as a tool in README (title "trajectory preview" + a blockquote saying nothing it produces reaches the robot), system_architecture.md ("Side branch", not "Stage 6"), mc_plan.md purpose, rm_plan.md done-list | info: mc_plan's old "feed a trajectory to the ESP32" item marked CLOSED by option A rather than deleted, so the decision stays visible

## Test result
