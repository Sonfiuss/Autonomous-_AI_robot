# SEQ module plan (`project/src/SEQ` + `motivation/jetson/mission_runner.*`)

## Purpose
The loop between a plan and a moving robot: walk MV's primitive list onto the wire one leg at a
time — send a leg, wait for the firmware's `K`, send the next. Stage 6 of
`agent/description/system_architecture.md`, `[MISSING]` until 2026-09-24.

## Architecture
Pure state machine in `project/` (no port, no clock, no heap), platform loop in `motivation/jetson/`.
The split is what makes the handshake testable on a PC against a fake firmware: `test_seq` drives
every failure path — ack timeout, dropped one-shot, mid-plan reboot, abort — with no ESP32 present.

## Decisions (2026-09-24, with the user)
- **A1 — `MOVE` is decomposed into `T` + `F`, not streamed as `M`.** The protocol has no world-frame
  straight-line command. Streaming `M` would keep the motion holonomic but leave the leg with no
  acknowledgement, putting the robot back on the Jetson's timing — the exact thing option A was
  chosen to avoid. Cost: the robot turns onto each bearing instead of crabbing. Verified harmless:
  a holonomic plan and its non-holonomic twin produce identical wire lines.
- **B — SEQ lives in `project/`, not in `motivation/jetson/`.** The latter needs POSIX termios and
  does not build on Windows, so the state machine would have been untestable there.
- **C — `RobotState` gained `readies`.** `ready` latches true on the first `READY`, so a reboot
  mid-plan was undetectable; a count makes it detectable, and it voids the plan.

## Done
- [x] 2026-09-24 `seq::Sequencer` — expansion, heading-bias bookkeeping, RM-derived ack timeout,
      fail-fast on `E 1`/`E 2`/`E 4` and on a reboot, deferred stop on `abort()`.
- [x] 2026-09-24 `test_seq` — 15 groups, all passing on the PC.
- [x] 2026-09-24 `MissionRunner` + `robot_link --run-plan FILE [--start-theta|--speed|--yaw-rate]`.
      The plan-file parser skips any line that is not a primitive, so `mv_cli plan ...` output is a
      valid plan file verbatim.
- [x] 2026-09-24 `tools/build_seq.sh`, CMake `seq` target, `interfaces.md` + `system_architecture.md`.

## Facts worth keeping (they cost an afternoon to find)
- **A leg the firmware refuses vanishes silently.** `applyOneShot` discards the bool from
  `beginForward`/`beginTurn`, so a leg under `mc::cfg::MIN_LEG_LENGTH_M` / `MIN_LEG_ANGLE_RAD`
  produces no `K` and bumps no counter. The sequencer filters those itself with the same constants;
  without that it would hang until the ack timeout on every plan containing a rounding-sized leg.
- **Legs are timed, not pose-driven**, so a leg's duration is exactly computable and the ack timeout
  is principled rather than a fixed guess.
- **`READY` cannot be a gate**, only a grace period: the firmware may have booted long before the
  port was opened. Every counter is re-baselined when the plan goes live, so a fault raised before
  the first leg is not blamed on it — and a boot zeroes the firmware's tallies, which would otherwise
  read as a fresh fault.
- **`S` latches.** The trailing `S` is how a plan ends; the next explicit move releases it.

## Pending
- [ ] Run a plan on the real robot. Nothing below SEQ has ever executed on hardware: `RobotLink` and
      the firmware are both still `[UNBUILT]`.
- [ ] No frame reconciliation. The firmware's odometry (`O`, zeroed at boot) and MV's room frame are
      unrelated, so `--start-theta` is supplied by hand and the pose printed during a run is the
      firmware's, not the room's. Needed before CM can drive a plan end to end.
- [ ] Open loop, by construction: the sequencer trusts `K` and never compares `O` against where the
      leg should have ended. With no encoder there is nothing to compare against that would detect
      slip — revisit together with the encoder.
- [ ] Wire SEQ to CM (an `/api/run` that hands `/api/plan` output to the runner) and to the ZMQ
      bridge. That is the next break in the chain, and the last one.
- [ ] Optional: a streamed-`M` mode for genuinely holonomic MOVE legs, if crabbing turns out to
      matter. It needs a completion rule that does not depend on Jetson timing.
