# Jetson link

The Jetson half of the serial link: opens `/dev/ttyUSB0`, sends commands, and
decodes the telemetry the ESP32 sends back.

**Status: never built or run.** It needs POSIX termios and the machine this was
written on is Windows. The protocol it speaks, however, is the shared
`project/src/LINK` implementation, which IS covered by `test_link`.

## Division of labour (option A)

The ESP32 owns the kinematics. This side sends *intent* — a body-frame velocity,
a distance, a turn — and never per-wheel speeds:

```
CM / MV / MC  ->  RobotLink  ->  "M vx vy omega"  ->  ESP32 (RM) -> wheels
                     ^                              |
                     +---  "O x y theta", "K", "E" -+
```

MC still computes a full per-tick trajectory on this side, but as a *preview and
feasibility check*, not as the thing that drives the motors. Both sides call the
same RM library, so the numbers agree.

## Keep-alive is mandatory

The firmware treats an `M` older than `link::cfg::MOTION_CMD_TIMEOUT_MS` (200 ms)
as zero. Going quiet therefore means **stop**, never "hold course". `poll()`
repeats the last velocity every `KEEPALIVE_MS` (50 ms) automatically — so call
`poll()` regularly, or the robot coasts to a halt.

Any one-shot command (`F`, `T`, `S`, `R`) cancels the stream, so a keep-alive can
never restart the robot the moment a leg finishes.

## Build and run

```bash
tools/build_jetson.sh
./build/robot_link --monitor
./build/robot_link --vel 0.15 0 0          # forward at 0.15 m/s until Ctrl-C
./build/robot_link --forward 1.0           # one metre, waits for K
./build/robot_link --turn 90               # 90 degrees, waits for K
./build/robot_link --stop
./build/robot_link --port /dev/ttyUSB1 --monitor

# a whole MV plan, leg by leg -- mv_cli output is a valid plan file as it stands
./build/robot_link --run-plan plan.txt --start-theta 0
```

`--monitor` prints pose, servo angles, ack count, and any error counters the
firmware reports over `E`.

## Files

```
include/jetson/
  serial_port.h    termios wrapper, non-copyable
  robot_link.h     commands + telemetry + the keep-alive; RobotState
  mission_runner.h the sequencer's loop: poll, sample the link, send what SEQ asks for
src/
  serial_port.cpp  raw 8N1, select() for the read timeout
  robot_link.cpp   uses project/src/LINK for every line it reads or writes
  mission_runner.cpp  also the plan-file reader
  main.cpp         the CLI
tools/build_jetson.sh
```

## Running a plan

`--run-plan` hands an MV primitive list to `seq::Sequencer` and walks it onto the
wire one leg at a time: send a leg, wait for `K`, send the next. The handshake is
not optional — the firmware's Motion task pops a one-shot only while it is idle,
so legs sent back to back pile into an 8-deep FIFO and the ninth is dropped.

```
ROTATE 1.1332
FORWARD 3.0085
MOVE 0.5 -0.25
STOP
```

Any line that is not one of those four words is ignored, so `mv_cli plan ...`
output works verbatim, headers and coordinate rows included.

A `MOVE` becomes a turn onto its bearing plus a drive along it, because the
protocol has no world-frame straight-line command and streaming `M` would leave
the leg with no acknowledgement to wait for. The state machine itself lives in
`project/src/SEQ` and is covered by `test_seq`, which runs on a PC with no robot
attached; what is here is only the loop, the clock and the port.

## Not done yet

- Nothing connects this to CM or to the simulation's ZMQ bridge on ports
  5555/5556. `MissionRunner` is the piece those would call: CM would hand it
  `/api/plan` output instead of a file.
- No frame reconciliation. The firmware's odometry is zeroed at boot and has
  nothing to do with MV's room frame, so `--start-theta` is supplied by hand.
- `interfaces.md:15` still says `motivation` and `stereo-camera` must not use the
  ESP32 at the same time. This tool does not arbitrate that.
