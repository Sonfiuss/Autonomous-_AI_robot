# Task: Real-time C++ Keyboard Teleop for Omni Robot

**Module:** motivation  
**Status:** done  
**Input:** User request — write all movement control in C++  
**Expected output:** Real-time keyboard teleop (WASD) integrated into motivation binary

## Plan

1. Create `Teleop.hpp` / `Teleop.cpp` — raw-terminal non-blocking keyboard control, 20 Hz velocity loop
2. Update `main.cpp` — add `--teleop` flag, call `runTeleop()`
3. Update `CMakeLists.txt` — add `Teleop.cpp` to sources

## Key mapping
| Key | Action |
|-----|--------|
| W/S | forward / backward (+/-vx) |
| A/D | strafe left / right (+/-vy) |
| Q/E | rotate CCW / CW (+/-omega) |
| J/L | pan camera left / right |
| I/K | tilt camera up / down |
| Space | emergency stop |
| +/- | increase / decrease linear speed |
| R | reset odometry |
| X | quit |

## Execution log
- Created Teleop.hpp (function signature + key-map doc)
- Created Teleop.cpp (raw terminal, 20 Hz control loop, WASD+QE+IJKL+Space+R+X)
- Updated main.cpp: added `#include "Teleop.hpp"`, `--teleop` flag parsing, `runTeleop()` call
- Updated CMakeLists.txt: added `Teleop.cpp` to executable sources
- Verified: Teleop.cpp and MotionClient.cpp compile clean with g++ -std=c++17

## Test result
Build blocked by missing `libzmq` (pre-existing, not related to this change).
To test on robot: install libzmq (`sudo apt install libzmq3-dev`), then:
```
cd motivation/build && cmake .. && make -j$(nproc)
./motivation --port /dev/ttyUSB0 --teleop
```
