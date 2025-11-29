# Ford-Specific BluePilot Innovations - Implementation Report

## 1. Curvature Blending

**What Changes:** Blends model's `desiredCurvature` with `predictedCurvature` (from model orientationRate) instead of using desired curvature alone.

**Implementation:**
```python
# Line 401-421 in carcontroller.py
current_curvature = -yawRate / vEgo  # Vehicle sensor
desired_curvature = actuators.curvature  # From model action
predicted_curvature = orientationRate.z[curvature_lookup_time] / vEgo  # From model predictions
```

**Blend Formula:**
```python
blend_ratio = interp(abs(desired_curvature), [0.0, 0.001], [pc_blend_ratio_low, pc_blend_ratio_high])
requested_curvature = (predicted_curvature * blend_ratio) + (desired_curvature * (1 - blend_ratio))
```

**Default Values:**
- **pc_blend_ratio_low_C_CAN** = 0.40 (CAN vehicles, low curvature)
- **pc_blend_ratio_high_C_CAN** = 0.40 (CAN vehicles, high curvature)
- **pc_blend_ratio_low_C_CANFD** = 0.40 (CANFD vehicles, low curvature)
- **pc_blend_ratio_high_C_CANFD** = 0.40 (CANFD vehicles, high curvature)
- **Breakpoints:** [0.0, 0.001] (1/m curvature)

**Configurable:**
- UI overrides: `pc_blend_ratio_low_C_UI`, `pc_blend_ratio_high_C_UI` (lines 141-142, 382-383)
- Custom profile enables UI values (line 381)

**Noise Considerations:**
- **NOT noise-based** - blend ratio is fixed at 40% predicted / 60% desired
- No dynamic adaptation based on model confidence/noise
- No model noise/uncertainty metrics found in codebase
- **Could be implemented** by extracting std from `modelV2.meta` fields or computing variance of recent predictions, but this is not currently done

---

## 2. Four-Signal EPAS Control

**What Changes:** Sends 4 steering signals instead of single curvature command (CANFD only).

**Signals Sent (line 698-701):**
1. **apply_curvature** - Primary steering command (clipped to 0.0115 vs DBC max 0.02)
2. **desired_curvature_rate** - Rate of curvature change (d/dt of predicted curvature)
3. **path_offset** - Lane positioning offset (blends model position.y with laneline centers)
4. **path_angle** - Fine steering adjustment (PID output from path offset error)

**Implementation:**
```python
# Curvature Rate (lines 524-546)
curvature_rate = (predicted_curvature[-1] - predicted_curvature[0]) / 0.3s / vEgo
# Scaled by speed factor [1.0@0-14.5m/s, 0.0@15.5m/s+] and curvature factor

# Path Offset (lines 556-579)
path_offset = blend(model.position.y, laneline_centers) + custom_offset
# Laneline blend based on confidence [0.6-0.8] and lane width [3.75-4.25m]

# Path Angle (lines 594-622)
path_angle = PID(path_offset_error * speed_factor)
# PID: k_p=0.25, k_i=0.05, rate=20Hz
```

**Default Gains:**
- **LC_PID_GAIN_CAN** = 5.0 (CAN vehicles)
- **LC_PID_GAIN_CANFD_SMALL_VEHICLE** = 3.0 (Escape, Mach-E)
- **LC_PID_GAIN_CANFD_LARGE_VEHICLE** = 3.0 (F-150, Expedition, etc.)
- **k_p** = 0.25, **k_i** = 0.05

**Configurable:**
- `LC_PID_GAIN_UI` override (line 173, 384)
- `custom_path_offset` for manual lane positioning (line 158, 575)
- `enable_lane_positioning` toggle (line 122, 591)

**Max Limits (line 635-638):**
- Curvature: 0.0115 (1/m) - **reduced from DBC 0.02 to prevent windup**
- Curvature rate: 0.001023 (1/m/s)
- Path offset: 2.0 (m) - **currently zeroed out line 642**
- Path angle: 0.5 (rad)

---

## 3. Anti-Windup Logic

**What Changes:** Prevents integrator windup during tight turns via reset + ramp logic.

**Detection (lines 451-460):**
```python
human_turn = steeringPressed and abs(steeringAngleDeg) > 45
reset_steering = (human_turn and enable_human_turn_detection) or (vEgo < 0.1)
```

**Reset Behavior (lines 463-478):**
- Sets all signals to 0 immediately
- Clears PID integrators (line 667)
- Uses `ramp_type=3` (immediate) in CAN message (line 664)

**Post-Reset Ramp (lines 481-503):**
```python
# Gradually ramp from 0 to requested using rate limits
apply_curvature = apply_std_steer_angle_limits(requested, 0.0, vEgo, ...)
# Exit when within 10% or 0.001 threshold
```

**Configurable:**
- `enable_human_turn_detection` toggle (line 457)

**Additional Limits:**
- At v > 9 m/s: clips curvature to ±0.002 from current curvature (lines 60-62)
- CANFD: clips to max lateral accel 2.4 m/s² (lines 69-72)

---

## 4. Separate Gas/Brake

**What Changes:** Ford uses discrete gas and brake signals instead of combined acceleration.

**Implementation (lines 735-748):**
```python
# Input: accel (m/s²) from longitudinal controller
gas = accel  # Initially same
accel = apply_creep_compensation(accel, vEgo)  # Subtract creep at low speed
accel = max(accel, prev_accel - 3.5*dt)  # Rate limit brake to 3.5 m/s³

# Separate signals
if not longActive or gas < -0.5:
    gas = -5.0  # INACTIVE_GAS sentinel
# accel used for braking only

# Brake request flag
accel_pitch_compensated = accel + pitch_accel
if accel_pitch_compensated > 0.3 or not longActive:
    brake_request = False
elif accel_pitch_compensated < 0.0:
    brake_request = True
```

**Default Values:**
- **MIN_GAS** = -0.5 m/s²
- **INACTIVE_GAS** = -5.0 (sentinel value)
- **ACCEL_MIN** = -3.5 m/s²
- **ACCEL_MAX** = 2.0 m/s²
- **Brake rate limit** = 3.5 m/s³

**Not Configurable** - hardcoded values

---

## 5. Pitch Compensation

**What Changes:** Adds road grade compensation to brake threshold logic.

**Implementation (lines 740-748):**
```python
# Extract pitch from orientationNED (Roll, Pitch, Yaw)
accel_due_to_pitch = sin(pitch) * 9.81  # m/s²

# Adjust accel for brake decision
accel_pitch_compensated = accel + accel_due_to_pitch

# Brake thresholds
if accel_pitch_compensated > 0.3:  # Uphill or light decel
    brake_request = False
elif accel_pitch_compensated < 0.0:  # Downhill or heavy decel
    brake_request = True
```

**Purpose:**
- Prevents unnecessary brake activation on uphills when decelerating
- Engages brake earlier on downhills

**Not Configurable** - always active if `orientationNED` available

---

## Summary Table

| Feature | Default Values | Configurable? | Noise-Adaptive? |
|---------|---------------|---------------|-----------------|
| **Curvature Blend** | 40% predicted / 60% desired | ✅ Via UI (low/high) | ❌ Fixed ratio |
| **Four Signals** | Gains: 3.0-5.0, PID k_p=0.25 | ✅ Gain, offset, toggle | ❌ No |
| **Anti-Windup** | Reset @ >45° + steering pressed | ✅ Detection toggle | ❌ No |
| **Gas/Brake Split** | Gas < -0.5 → inactive, Rate 3.5 m/s³ | ❌ Hardcoded | ❌ No |
| **Pitch Comp** | Threshold 0.3 m/s² | ❌ Hardcoded | ❌ No |

## Noise-Based Adaptation Feasibility

**Current State:** No noise/confidence adaptation implemented

**Potential Implementation:**
1. Extract `modelV2.meta` confidence fields (if available)
2. Compute variance of recent `orientationRate.z` predictions
3. Dynamically adjust `pc_blend_ratio`:
   - High noise → increase predicted weight (trust smoothness)
   - Low noise → increase desired weight (trust E2E model)
4. Similar logic for path_angle PID gain

**Required Changes:**
- Add rolling variance calculation (similar to `curvature_rate_deque`)
- Add confidence-based lookup table
- Modify lines 418-421 to use dynamic ratio

---

## Reference File Locations

All line numbers reference: `opendbc_repo/opendbc/car/ford/carcontroller.py`
