# Task: core-control module — kinematics core

**Module:** core-control (new standalone module)
**Status:** executing

## Input
High-level text commands: "move left 30", "spin 45", "arc forward 20 turn 30"

## Expected output
Wheel rotation angles in degrees for each of 3 omni-wheels (W1=150°, W2=270°, W3=30°)

## Plan
1. Create `core-control/KinematicsCore.hpp/.cpp` — math layer
2. Create `core-control/CommandParser.hpp/.cpp` — text → Command struct
3. Create `core-control/main.cpp` — interactive CLI + single-shot mode
4. Create `core-control/CMakeLists.txt` — standalone build

## Robot constants
- Wheel angles: θ1=150°, θ2=270°, θ3=30°
- Wheel radius r = 4.1 cm
- Center-to-wheel distance L = 14.4 cm
- Formula: φᵢ (deg) = [dx·cos(θᵢ) + dy·sin(θᵢ)] / r × (180/π) + rotate_deg × L/r

## Execution log
[done] step 1: KinematicsCore.hpp/.cpp created — φᵢ = (dx·cosθᵢ + dy·sinθᵢ)/r × RAD2DEG + rot×L/r
[done] step 2: CommandParser.hpp/.cpp created — supports move/spin/self-round/arc with direction tokens
[done] step 3: main.cpp — interactive + single-shot CLI
[done] step 4: CMakeLists.txt — standalone, no external deps
[done] build: cmake+make clean, all 5 command types produce correct wheel angles

**Status:** done
