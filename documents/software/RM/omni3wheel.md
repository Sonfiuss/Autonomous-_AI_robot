# RM (Movement Module) - Logic-Focused Summary

## Core Purpose
RM is the **kinematics logic layer** that converts between:
- **Jetson commands** (desired velocities in global frame) 
- **Wheel speeds** (what stepper drivers need to execute)
- **Odometry feedback** (actual robot position from encoder counts)

RM does NOT control the stepper driver directly; it only calculates what speeds to send.

---

## 1. Omni_Kinematics Logic

### What It Does
Implements the mathematical transformation between robot velocity commands and individual wheel speeds.

### Core Logic (From Paper Equation 16)

**Given:** Desired motion in robot body frame: [u, v, r]
- u = forward velocity (m/s)
- v = lateral/sideways velocity (m/s)  
- r = rotational velocity (rad/s)

**Calculate:** Required wheel angular velocities [ω₁, ω₂, ω₃]

```
ω₁ = (1/a)(v + Lr)
ω₂ = (1/2a)(-√3·u - v + 2Lr)
ω₃ = (1/2a)(√3·u - v + 2Lr)

Parameters:
  a = wheel radius (0.05m)
  L = distance from wheel to robot center (0.15m)
  ω = rad/s (will be converted to pulse frequency by driver)
```

### Input/Output
```
Input (from Jetson via UART):
  v_x, v_y, omega_z (global frame velocities)
  → Transform to robot frame: u, v, r

Process:
  Apply inverse kinematics matrix

Output (to driver):
  ω₁, ω₂, ω₃ (angular velocities for each wheel)
```

### Key Feature: Bidirectional
```
Forward Direction (command → wheel speeds):
  [u, v, r] → [ω₁, ω₂, ω₃]  (inverse kinematics)

Reverse Direction (encoder → robot velocity):
  [ω₁, ω₂, ω₃] → [u, v, r]  (forward kinematics, for odometry)
```

### No Stepper Control Here
- Omni_kinematics outputs **angular velocity in rad/s**
- The driver layer converts this to **pulse frequency in Hz**
- Example: ω₁ = 10 rad/s → stepper driver converts to 318 Hz pulse train

---

## 2. Velocity_Profile Logic

### What It Does
Ensures smooth, ramped acceleration/deceleration of wheel speeds to prevent stepper motor skipping.

### Core Logic

**Input:** Target wheel angular velocity ω_target

**Output:** Time-based velocity profile v(t) that ramps smoothly

**Three Phases:**

```
Acceleration Phase (0 → t_accel):
  v(t) = a_max × t
  where a_max = max acceleration (rad/s²)
  distance: s_accel = v_target² / (2 × a_max)

Constant Velocity Phase (t_accel → t_accel + t_hold):
  v(t) = v_target (constant)
  duration: t_hold = remaining_distance / v_target

Deceleration Phase (end - t_decel → end):
  v(t) = v_target - a_decel × time_since_decel_start
  distance: s_decel = v_target² / (2 × a_decel)
```

### Mathematical Constraint Check
```
If total_distance < 2 × s_accel:
  → Use triangular profile (ramp up then immediately ramp down)
  → Never reaches v_target
Else:
  → Use full trapezoidal profile (accel, constant, decel)
```

### Input/Output
```
Input:
  ω_target (rad/s from omni_kinematics)
  distance_to_move (meters)
  a_max, a_decel (acceleration limits)

Process:
  Generate velocity at each time step: v[i]

Output:
  v[0], v[1], v[2], ... v[N]
  (smooth velocity array that avoids stepper skipping)
```

### Key Feature: Prevents Stepper Skipping
- Stepper motors have torque limits
- If you ask for speed change too fast → motor loses steps
- Profile ensures gradual acceleration within motor torque curve
- Typical a_max: 50-100 mm/s² (conservative)

### No Stepper Control Here
- Outputs smooth velocity profile
- Driver converts velocity to actual step pulses

---

## 3. Odometry Logic

### What It Does
Tracks robot's actual position and orientation based on encoder pulse counts from wheels.

### Core Logic (From Paper Equation 17)

**Input:** Encoder counts from three wheels
```
Δsteps[1], Δsteps[2], Δsteps[3]
(since last odometry update)
```

**Step 1: Convert Steps to Distances**
```
Δdist[i] = Δsteps[i] × (wheel_circumference / steps_per_revolution)
         = Δsteps[i] × (2π × a / steps_per_rev)
         
Example: 100 steps × (2π × 0.05 / 2000) = 0.0157m
```

**Step 2: Apply Forward Kinematics (Paper Eq. 17)**
```
Convert wheel distances → robot body velocities:

u = (a/3)(2·Δdist₁ - Δdist₂ - Δdist₃)
v = (a/3)(-√3·Δdist₁ + 0·Δdist₂ + 0·Δdist₃)
r = (a/3L)(Δdist₁ + Δdist₂ + Δdist₃)

(Note: divide by time interval Δt to get velocities)
```

**Step 3: Transform to Global Frame**
```
Current robot heading: θ

Global displacement:
  Δx = u·cos(θ) - v·sin(θ)  [forward direction in global frame]
  Δy = u·sin(θ) + v·cos(θ)  [lateral direction in global frame]
  Δθ = r × Δt               [rotation]

Update position:
  x_new = x_old + Δx
  y_new = y_old + Δy
  θ_new = θ_old + Δθ
```

### Input/Output
```
Input (from stepper driver/encoders):
  Encoder pulse counts (Δsteps for each wheel)
  Time since last update (Δt)

Process:
  Apply forward kinematics + pose integration

Output (to Jetson via UART):
  Updated position: x, y, θ
  Estimated velocity: v_x, v_y, ω_z
  Status flags
```

### Key Features

**1. Detects Wheel Slip**
```
Expected count from command ≠ Actual encoder count
→ Motor skipped steps or wheel slipped
→ Send error flag to Jetson for correction
```

**2. Tracks Cumulative Position**
```
Position error grows over time (dead reckoning)
Typical error: 1-2% of distance traveled
After 10m: error ~ 10-20cm
```

**3. Provides Feedback for Path Correction**
```
Jetson receives: actual_position
Jetson calculates: error = desired_path - actual_position
Jetson adjusts: next velocity command to correct course
```

---

## 4. Driver_Stepdir Logic (OUTSIDE RM)

### What It Does
Converts RM's velocity outputs into actual stepper pulse signals.

**NOT part of RM logic layer** - this is hardware control.

### Data Flow
```
RM Layer Output: ω[t] = smooth angular velocity profile (rad/s)
                        ↓
Driver Layer:     f[t] = ω[t] / (2π) × steps_per_revolution
                        ↓
Hardware:         Pulse train at frequency f[t]
                  Step signal + Direction signal to stepper driver IC
                        ↓
Motor:            Rotates at desired speed
                  Encoder sends pulse counts back
```

### Why It's Separate from RM
- RM calculates **what speed** (in physical units: rad/s, m/s)
- Driver calculates **how to make it happen** (pulse frequency in Hz)
- Different concerns: physics vs. hardware interface

---

## 5. Complete Data Flow (Showing RM's Role)

```
┌─────────────────────────────────────────────────────────┐
│  JETSON ORIN NANO (High-level decisions)               │
│  ┌─────────────────────────────────────────────────┐   │
│  │ Desired motion: v_x, v_y, omega_z (m/s, rad/s) │   │
│  └─────────────────┬───────────────────────────────┘   │
│                    │ UART Send                          │
└────────────────────┼─────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────────────────┐
│  ESP32 - RM (MOVEMENT MODULE LOGIC)                      │
│                                                           │
│  ┌─────────────────────────────────────────────────┐   │
│  │ 1. Transform Global→Body Frame                  │   │
│  │    v_x, v_y, omega_z → u, v, r                │   │
│  └──────────────────┬────────────────────────────┘   │
│                     ↓                                  │
│  ┌─────────────────────────────────────────────────┐   │
│  │ 2. OMNI_KINEMATICS LOGIC                        │   │
│  │    [u, v, r] → [ω₁, ω₂, ω₃]                   │   │
│  │    (Inverse kinematics equations)               │   │
│  └──────────────────┬────────────────────────────┘   │
│                     ↓                                  │
│  ┌─────────────────────────────────────────────────┐   │
│  │ 3. VELOCITY_PROFILE LOGIC                       │   │
│  │    [ω₁, ω₂, ω₃] → smooth acceleration ramps   │   │
│  │    Output: v_smooth[t] for each wheel           │   │
│  └──────────────────┬────────────────────────────┘   │
│                     ↓                                  │
│  ┌─────────────────────────────────────────────────┐   │
│  │ 4. Send to DRIVER_STEPDIR (Hardware Control)    │   │
│  │    Convert rad/s → Hz pulse frequency           │   │
│  │    Generate step/direction signals              │   │
│  └──────────────────┬────────────────────────────┘   │
│                     │                                 │
│        ┌────────────┴───────────┐                     │
│        ↓                        ↓                     │
│     Motor 1              Motor 2/3                    │
│     Stepper              Stepper                      │
│        ↓                        ↓                     │
│     Encoder1              Encoder2/3                  │
│        ↓                        ↓                     │
│        └────────────┬───────────┘                     │
│                     ↓                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │ 5. ODOMETRY LOGIC (Feedback Loop)               │   │
│  │    [Δsteps₁, Δsteps₂, Δsteps₃]                 │   │
│  │    → Forward kinematics                         │   │
│  │    → Update position [x, y, θ]                  │   │
│  └──────────────────┬────────────────────────────┘   │
│                     │ UART Send                       │
└─────────────────────┼────────────────────────────────┘
                      ↓
┌──────────────────────────────────────────────────────┐
│  JETSON ORIN NANO                                    │
│  ┌────────────────────────────────────────────────┐ │
│  │ Received: actual x, y, θ, v_x, v_y, omega_z  │ │
│  │ Compare with desired path                      │ │
│  │ Adjust next command if error detected          │ │
│  └────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────┘
```

---

## 6. RM Logic Summary (What RM Must Do)

| Component | Input | Logic | Output |
|-----------|-------|-------|--------|
| **Omni_Kinematics** | [u, v, r] | Inverse kinematics matrix | [ω₁, ω₂, ω₃] rad/s |
| **Velocity_Profile** | [ω₁, ω₂, ω₃], distance | Trapezoidal ramp generation | v_smooth[t] for each wheel |
| **Odometry** | [Δsteps₁, Δsteps₂, Δsteps₃], Δt | Forward kinematics + pose integration | [x, y, θ] updated position |
| **Driver_Stepdir** | [ω₁, ω₂, ω₃] rad/s | Frequency conversion + pulse generation | Step/direction signal to motor IC |

---

## 7. Key Mathematical Formulas (RM Needs to Implement)

### Equation Set 1: Inverse Kinematics (Omni_Kinematics)
```
ω₁ = (1/a)(v + Lr)
ω₂ = (1/2a)(-√3·u - v + 2Lr)
ω₃ = (1/2a)(√3·u - v + 2Lr)

Parameters: a = wheel_radius, L = robot_radius
```

### Equation Set 2: Forward Kinematics (Odometry)
```
u = (a/3)(2·Δdist₁ - Δdist₂ - Δdist₃)
v = (a/3)(-√3·Δdist₁)
r = (a/3L)(Δdist₁ + Δdist₂ + Δdist₃)

Then convert to global frame using rotation matrix:
Δx = u·cos(θ) - v·sin(θ)
Δy = u·sin(θ) + v·cos(θ)
Δθ = r·Δt
```

### Equation Set 3: Trapezoidal Profile (Velocity_Profile)
```
Acceleration phase: v[i] = a_max × t[i]
Constant phase:     v[i] = v_target
Deceleration phase: v[i] = v_target - a_decel × (t[i] - t_decel_start)
```

---

## 8. RM Does NOT Do

❌ **Does NOT:**
- Generate stepper pulse signals
- Control GPIO pins directly
- Manage stepper motor current/voltage
- Handle UART/serial communication directly
- Calculate trajectory planning (that's Jetson's job)
- Perform sensor fusion (that's future enhancement)

✅ **ONLY Does:**
- Pure mathematical kinematics calculations
- Smooth velocity curve generation
- Position tracking from encoder counts
- Send calculated values to driver layer

---

## Conclusion

**RM = Kinematics Brain**
- Omni_kinematics: "What wheel speeds are needed?"
- Velocity_profile: "How do we smoothly reach those speeds?"
- Odometry: "Where are we actually now?"
- Driver_stepdir: (separate) "How do we make motors do it?"

RM is the **logic and math layer**, not the **hardware control layer**.