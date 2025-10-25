# ArduCopter Tuning Guide

## Table of Contents

- [Overview](#overview)
- [Tuning Philosophy and Safety](#tuning-philosophy-and-safety)
- [Understanding PID Controllers](#understanding-pid-controllers)
- [Transmitter-Based Tuning](#transmitter-based-tuning)
- [AutoTune Mode](#autotune-mode)
- [QuickTune System](#quicktune-system)
- [Rate Controllers](#rate-controllers)
- [Attitude Controllers](#attitude-controllers)
- [Filter Configuration](#filter-configuration)
- [Parameter Reference](#parameter-reference)
- [Troubleshooting](#troubleshooting)
- [Vehicle-Specific Considerations](#vehicle-specific-considerations)

---

## Overview

Proper tuning is essential for optimal flight performance, stability, and safety in ArduCopter. This guide covers multiple tuning approaches suitable for different experience levels and vehicle types, from manual PID tuning to automated methods like AutoTune and QuickTune.

### Why Tuning Matters

- **Flight Performance**: Well-tuned vehicles respond precisely to pilot inputs and autopilot commands
- **Stability**: Proper gains prevent oscillations and ensure smooth flight
- **Safety**: Correct tuning prevents dangerous behavior like flip-overs or uncontrolled oscillations
- **Efficiency**: Optimized controllers minimize unnecessary motor corrections, extending flight time
- **Mission Accuracy**: Tight control enables precise waypoint navigation and positioning

### Documentation Scope

This guide covers:
- **Rate Controllers** (`ATC_RAT_*`): Body-frame angular rate control (roll, pitch, yaw rates)
- **Attitude Controllers** (`ATC_ANG_*`): Angle-to-rate cascade control
- **Manual Tuning**: Systematic PID adjustment procedures
- **Transmitter Tuning**: Real-time in-flight parameter adjustment (source: `ArduCopter/tuning.cpp`)
- **AutoTune**: Automated PID optimization (source: `ArduCopter/mode_autotune.cpp`)
- **QuickTune**: Rapid oscillation-based tuning (source: `libraries/AP_Quicktune/AP_Quicktune.cpp`)

---

## Tuning Philosophy and Safety

### Core Principles

1. **Start Conservative**: Begin with low gains and increase gradually
2. **One Parameter at a Time**: Change single parameters to understand their effects
3. **Test Incrementally**: Make small changes and test before continuing
4. **Document Baselines**: Record original parameters before tuning
5. **Monitor Continuously**: Watch for oscillations and instability

### Safety Considerations

⚠️ **Critical Safety Requirements**:

- **Safe Environment**: Tune in open areas away from people and obstacles
- **Pre-Flight Checks**: Complete all arming checks before tuning flights
- **Kill Switch Ready**: Always have ability to disarm immediately
- **Altitude Margin**: Maintain sufficient altitude for recovery (5+ meters recommended)
- **Backup Parameters**: Save known-good parameter sets before tuning
- **Conservative Limits**: Use angle limits to prevent excessive tilting
- **Weather Conditions**: Tune only in calm wind conditions (<5 m/s)
- **Battery Reserves**: Maintain 30%+ battery capacity during tuning flights

### Pre-Tuning Checklist

✅ **Hardware Validation**:
- [ ] All motors spinning in correct direction
- [ ] Props installed correctly and secured
- [ ] IMU calibrated and healthy
- [ ] Compass calibrated (if using GPS navigation)
- [ ] Radio calibrated with proper failsafe configured
- [ ] Vibration levels acceptable (check `VIBE` logs)
- [ ] Motor/ESC timing configured correctly
- [ ] Battery voltage and capacity adequate

✅ **Software Validation**:
- [ ] Correct frame type configured (`FRAME_CLASS`, `FRAME_TYPE`)
- [ ] Motor outputs mapped correctly
- [ ] Sensor orientation correct (`AHRS_ORIENTATION`)
- [ ] RC input functional on all channels
- [ ] Flight modes configured and tested on ground

---

## Understanding PID Controllers

### PID Fundamentals

ArduCopter uses cascaded PID (Proportional-Integral-Derivative) controllers for attitude and rate control. Understanding each term is essential for effective tuning.

#### Proportional (P) Term

**Function**: Generates correction proportional to the error
- **Effect**: Primary restoring force toward desired state
- **Too Low**: Sluggish response, slow to reach setpoint
- **Too High**: Oscillations, overshooting target
- **Formula**: `output = kP × error`

**Rate Controller P** (`ATC_RAT_*_P`):
- Controls how aggressively rate error is corrected
- Higher values = faster response to rate commands
- Source: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h` defines defaults:
  - Roll/Pitch: `AC_ATC_MULTI_RATE_RP_P = 0.135`
  - Yaw: `AC_ATC_MULTI_RATE_YAW_P = 0.180`

**Angle Controller P** (`ATC_ANG_*_P`):
- Converts angle error to desired rate
- Higher values = more aggressive angle corrections
- Typical range: 4.5 to 10.0

#### Integral (I) Term

**Function**: Accumulates error over time to eliminate steady-state offset
- **Effect**: Removes persistent bias and trim errors
- **Too Low**: Persistent offset, drift in hover
- **Too High**: Slow oscillations, overshoot recovery
- **Formula**: `output = kI × ∫(error)dt`

**Rate Controller I** (`ATC_RAT_*_I`):
- Compensates for aerodynamic effects and motor imbalances
- Typically set equal to P term (`ATC_RAT_*_I = ATC_RAT_*_P`)
- Source: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h`:
  - Roll/Pitch: `AC_ATC_MULTI_RATE_RP_I = 0.135`
  - Yaw: `AC_ATC_MULTI_RATE_YAW_I = 0.018`

**I-Term Limits** (`ATC_RAT_*_IMAX`):
- Prevents integrator windup
- Typical value: `0.5` (50% throttle authority)
- Source: `AC_ATC_MULTI_RATE_RP_IMAX = 0.5f`

#### Derivative (D) Term

**Function**: Responds to rate of change of error
- **Effect**: Damping, reduces overshoot and oscillations
- **Too Low**: Overshooting, ringing after maneuvers
- **Too High**: Amplifies noise, rapid jittering
- **Formula**: `output = kD × d(error)/dt`

**Rate Controller D** (`ATC_RAT_*_D`):
- Primary damping term for rate control
- Requires low-pass filtering to avoid noise amplification
- Source: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h`:
  - Roll/Pitch: `AC_ATC_MULTI_RATE_RP_D = 0.0036`
  - Yaw: `AC_ATC_MULTI_RATE_YAW_D = 0.0` (typically zero for yaw)

#### Feed-Forward (FF) Term

**Function**: Anticipates required control based on desired rate (helicopter-specific)
- **Effect**: Improves tracking of rate commands
- **Application**: Primarily used in helicopter configurations
- **Formula**: `output = kFF × desired_rate`

**Helicopter FF Terms** (source: `ArduCopter/tuning.cpp:127-137`):
- `TUNING_RATE_PITCH_FF`: Pitch feed-forward
- `TUNING_RATE_ROLL_FF`: Roll feed-forward  
- `TUNING_RATE_YAW_FF`: Yaw feed-forward
- Typical range: 0.025 to 0.5

### Control Loop Architecture

ArduCopter implements a cascaded control structure:

```
Pilot Input / Navigation Commands
           ↓
   [Attitude Controller]
   Converts angle error → desired rate
   Parameters: ATC_ANG_RLL_P, ATC_ANG_PIT_P, ATC_ANG_YAW_P
           ↓
      [Rate Controller]
   Converts rate error → motor commands
   Parameters: ATC_RAT_RLL_*, ATC_RAT_PIT_*, ATC_RAT_YAW_*
           ↓
       [Motor Mixer]
   Distributes commands to individual motors
           ↓
        Motors/ESCs
```

**Loop Rates** (source: `ArduCopter/Copter.cpp:scheduler_tasks[]`):
- Rate Controller: 400 Hz (main loop rate)
- Attitude Controller: 400 Hz
- Navigation Updates: 50 Hz
- Transmitter Tuning: 3.3 Hz (source: `ArduCopter/tuning.cpp:10`)

---

## Transmitter-Based Tuning

Transmitter tuning allows real-time parameter adjustment during flight using an RC channel, enabling immediate feedback on gain changes.

### Setup Requirements

**Source**: `ArduCopter/tuning.cpp:8-34`

1. **Assign Tuning Channel**:
   - Configure an RC channel (typically channel 6) for tuning
   - Channel must provide continuous input (potentiometer or slider)
   - Verify channel outputs 1000-2000 µs range

2. **Select Tuning Parameter**:
   - Set `TUNE` parameter to desired function (see table below)
   - Parameter determines which gain is adjusted

3. **Configure Range**:
   - `TUNE_MIN`: Value at minimum stick position
   - `TUNE_MAX`: Value at maximum stick position
   - Start with narrow ranges around current values

### Tuning Parameter Functions

**Source**: `ArduCopter/defines.h` and `ArduCopter/tuning.cpp:34-201`

| Function ID | Name | Parameters Affected | Description | Typical Range |
|-------------|------|---------------------|-------------|---------------|
| 1 | `TUNING_STABILIZE_ROLL_PITCH_KP` | `ATC_ANG_RLL_P`, `ATC_ANG_PIT_P` | Angle controller P (both axes) | 4.5 - 10.0 |
| 3 | `TUNING_STABILIZE_YAW_KP` | `ATC_ANG_YAW_P` | Yaw angle controller P | 4.5 - 10.0 |
| 4 | `TUNING_RATE_ROLL_PITCH_KP` | `ATC_RAT_RLL_P`, `ATC_RAT_PIT_P` | Rate P (both axes) | 0.08 - 0.20 |
| 5 | `TUNING_RATE_ROLL_PITCH_KI` | `ATC_RAT_RLL_I`, `ATC_RAT_PIT_I` | Rate I (both axes) | 0.08 - 0.20 |
| 6 | `TUNING_YAW_RATE_KP` | `ATC_RAT_YAW_P` | Yaw rate P | 0.15 - 0.25 |
| 7 | `TUNING_THROTTLE_RATE_KP` | Velocity Z P | Throttle rate P | 4.0 - 12.0 |
| 10 | `TUNING_WP_SPEED` | Waypoint speed | Max horizontal speed (cm/s) | 500 - 2000 |
| 12 | `TUNING_LOITER_POSITION_KP` | Position NE P | Loiter position P | 0.5 - 2.0 |
| 14 | `TUNING_ALTITUDE_HOLD_KP` | Position U P | Altitude hold P | 0.5 - 3.0 |
| 21 | `TUNING_RATE_ROLL_PITCH_KD` | `ATC_RAT_RLL_D`, `ATC_RAT_PIT_D` | Rate D (both axes) | 0.002 - 0.008 |
| 22 | `TUNING_VEL_XY_KP` | Velocity NE P | Horizontal velocity P | 1.0 - 4.0 |
| 25 | `TUNING_ACRO_RP_RATE` | Acro roll/pitch rate | Acro mode max rate (deg/s) | 180 - 720 |
| 26 | `TUNING_YAW_RATE_KD` | `ATC_RAT_YAW_D` | Yaw rate D | 0.0 - 0.01 |
| 28 | `TUNING_VEL_XY_KI` | Velocity NE I | Horizontal velocity I | 0.2 - 2.0 |
| 34 | `TUNING_ACCEL_Z_KP` | Accel Z P | Vertical accel P | 0.3 - 1.5 |
| 35 | `TUNING_ACCEL_Z_KI` | Accel Z I | Vertical accel I | 0.5 - 3.0 |
| 36 | `TUNING_ACCEL_Z_KD` | Accel Z D | Vertical accel D | 0.0 - 0.5 |
| 38 | `TUNING_DECLINATION` | Compass declination | Magnetic declination (rad) | -0.5 - 0.5 |
| 39 | `TUNING_CIRCLE_RATE` | Circle mode rate | Circle rate (deg/s) | 5 - 45 |
| 40 | `TUNING_ACRO_YAW_RATE` | Acro yaw rate | Acro mode yaw rate (deg/s) | 45 - 360 |
| 45 | `TUNING_RC_FEEL_RP` | Input time constant | Roll/pitch smoothing | 0.0 - 0.5 |
| 46 | `TUNING_RATE_PITCH_KP` | `ATC_RAT_PIT_P` | Pitch rate P only | 0.08 - 0.20 |
| 47 | `TUNING_RATE_PITCH_KI` | `ATC_RAT_PIT_I` | Pitch rate I only | 0.08 - 0.20 |
| 48 | `TUNING_RATE_PITCH_KD` | `ATC_RAT_PIT_D` | Pitch rate D only | 0.002 - 0.008 |
| 49 | `TUNING_RATE_ROLL_KP` | `ATC_RAT_RLL_P` | Roll rate P only | 0.08 - 0.20 |
| 50 | `TUNING_RATE_ROLL_KI` | `ATC_RAT_RLL_I` | Roll rate I only | 0.08 - 0.20 |
| 51 | `TUNING_RATE_ROLL_KD` | `ATC_RAT_RLL_D` | Roll rate D only | 0.002 - 0.008 |
| 55 | `TUNING_RATE_MOT_YAW_HEADROOM` | Motor yaw headroom | Yaw authority reserve | 0 - 500 |
| 56 | `TUNING_RATE_YAW_FILT` | `ATC_RAT_YAW_FLTE` | Yaw rate filter (Hz) | 1.0 - 10.0 |
| 58 | `TUNING_SYSTEM_ID_MAGNITUDE` | SystemID magnitude | System ID signal strength | 0.0 - 1.0 |
| 59 | `TUNING_POS_CONTROL_ANGLE_MAX` | Position control max angle | Max lean angle (deg) | 10 - 45 |
| 60 | `TUNING_LOITER_MAX_XY_SPEED` | Loiter max speed | Max loiter speed (cm/s) | 500 - 2500 |

**Implementation Details** (source: `ArduCopter/tuning.cpp:34-201`):
- Switch statement handles 50+ tunable parameters
- Linear interpolation between min/max: `tuning_value = linear_interpolate(min, max, control_in, -1, 1)`
- Updates executed at 3.3 Hz
- Changes are temporary (not saved to EEPROM during flight)
- Logging available with `LOG_TUNING` parameter

### Transmitter Tuning Workflow

**Step 1: Initial Setup**
```
1. Set TUNE = 4 (Rate Roll/Pitch P)
2. Set TUNE_MIN = 0.08
3. Set TUNE_MAX = 0.20
4. Set tuning channel to middle position
5. Arm and take off to hover
```

**Step 2: Find Optimal Gain**
```
1. Gradually move tuning channel higher
2. Perform small sharp stick inputs (roll/pitch)
3. Observe response:
   - Too low: Sluggish, slow to level
   - Too high: Oscillations after inputs
   - Optimal: Quick response, minimal overshoot
4. Note optimal channel position
```

**Step 3: Test and Refine**
```
1. Test with larger maneuvers
2. Verify stability in hover
3. Check response in forward flight
4. Land and save parameter value
```

**Step 4: Proceed to Next Parameter**
```
- Repeat for I term (TUNE = 5)
- Then D term (TUNE = 21)
- Follow systematic tuning order
```

### Safety Features

**Source**: `ArduCopter/tuning.cpp:13-25`

Transmitter tuning includes multiple safety checks:

1. **Channel Validation**: Exits if tuning channel not configured
   ```cpp
   if (rc_tuning == nullptr) return;
   ```

2. **Parameter Validation**: Exits if tuning function or range invalid
   ```cpp
   if ((g.rc_tuning_param <= 0) || (is_zero(g2.tuning_min.get()) && is_zero(g2.tuning_max.get()))) return;
   ```

3. **Failsafe Protection**: Disables during radio failsafe
   ```cpp
   if (!rc().has_valid_input() || rc_tuning->get_radio_in() == 0) return;
   ```

---

## AutoTune Mode

AutoTune automatically determines optimal PID gains through systematic flight testing, suitable for most multirotor configurations.

### How AutoTune Works

**Source**: `ArduCopter/mode_autotune.cpp` and `libraries/AC_AutoTune/`

AutoTune performs the following sequence:
1. **Initialization**: Validates flight conditions and prerequisites
2. **Testing**: Induces controlled oscillations on each axis
3. **Measurement**: Records response characteristics and phase margins
4. **Calculation**: Computes optimal P, I, D gains
5. **Verification**: Tests calculated gains for stability
6. **Completion**: Presents gains for pilot acceptance or rejection

**Algorithm** (traditional multicopters):
- Starts with reduced gains (50% of current)
- Gradually increases P gain until onset of oscillation
- Backs off to maintain phase margin
- Sets I = P and D = P × 0.027 (approximate ratios)
- Performs verification test

**Algorithm** (helicopters, source: `libraries/AC_AutoTune/AC_AutoTune_Heli.cpp:29-70`):
- Sweep-based frequency response testing
- Tests VFF (velocity feed-forward), Rate D/P, and Angle P sequentially
- Frequency range: `AUTOTUNE_FRQ_MIN` (10 Hz) to `AUTOTUNE_FRQ_MAX` (70 Hz)
- Gain limits defined by constants (e.g., `AUTOTUNE_RD_MAX = 0.020`)

### Prerequisites

**Flight Conditions** (source: `ArduCopter/mode_autotune.cpp:9-33`):

✅ **Required Conditions**:
- Must be in a mode that allows AutoTune (Stabilize, AltHold, Loiter, PosHold)
- Vehicle armed and flying (not landed)
- Throttle above zero (`!copter.ap.throttle_zero`)
- Auto-armed (`copter.ap.auto_armed`)
- Not in land-complete state

✅ **Environmental Requirements**:
- Calm conditions: Wind <5 m/s (11 mph)
- Open area: 20m × 20m minimum
- Altitude: 5-10 meters recommended
- Battery: >50% capacity (AutoTune can take 10-20 minutes)

✅ **Configuration Requirements**:
- Frame type correctly configured
- Motor directions verified
- Basic tune already applied (vehicle should be flyable)
- RC inputs calibrated

### AutoTune Parameters

**Source**: `libraries/AC_AutoTune/AC_AutoTune_Heli.cpp:71-99` (helicopter variant)

#### Core Parameters

| Parameter | Description | Default | Range | Notes |
|-----------|-------------|---------|-------|-------|
| `AUTOTUNE_AXES` | Axes to tune (bitmask) | 7 (all) | 0: Roll, 1: Pitch, 2: Yaw | Standard multicopters |
| `AUTOTUNE_AGGR` | Aggressiveness | 0.1 | 0.05 - 0.2 | Higher = more aggressive tune |
| `AUTOTUNE_MIN_D` | Minimum D gain | 0.001 | 0.0 - 0.01 | Lower bound for D term |

#### Helicopter-Specific Parameters

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `AUTOTUNE_SEQ` | Tuning sequence bitmask | 3 | 0: VFF, 1: Rate D/P, 2: Angle P, 3: Max Gain, 4: Tune Check |
| `AUTOTUNE_FRQ_MIN` | Minimum sweep frequency | 10.0 | 10 - 30 Hz |
| `AUTOTUNE_FRQ_MAX` | Maximum sweep frequency | 70.0 | 50 - 120 Hz |
| `AUTOTUNE_GN_MAX` | Maximum gain increase | 2.0 | 1.0 - 4.0 |

### AutoTune Flight Procedure

**Step 1: Pre-Flight**
```
1. Complete pre-flight checks
2. Verify battery >50%
3. Confirm calm wind conditions
4. Clear 20m × 20m area
5. Configure AutoTune switch on transmitter
```

**Step 2: Enter AutoTune Mode**
```
1. Take off in Loiter or AltHold mode
2. Climb to 5-10 meters
3. Switch to AutoTune mode
4. Vehicle will hover and begin testing
```

**Step 3: During AutoTune** (source: `ArduCopter/mode_autotune.cpp:35-53`)
```
- Maintain position with minimal stick inputs
- AutoTune will perform automated movements:
  * Sharp pitch movements
  * Sharp roll movements  
  * Sharp yaw movements
- Each axis tested for ~2-3 minutes
- Total time: 10-20 minutes typical
- Monitor battery level continuously
```

**Pilot Override** (source: `ArduCopter/mode_autotune.cpp:72-77`):
- Stick inputs pause AutoTune testing
- Release sticks to resume automated testing
- Large inputs can abort tune if angle error excessive

**Step 4: Completion**
```
AutoTune will:
1. Play completion tone
2. Display "AutoTune: Success" message
3. Wait for pilot decision
```

**Step 5: Accept or Reject Gains**
```
Option A - Accept Gains:
- Switch out of AutoTune to Loiter/AltHold/etc.
- New gains saved to EEPROM automatically
- Test fly with new gains

Option B - Reject Gains:
- Disarm before switching out of AutoTune
- Original gains restored
- No parameters changed
```

### AutoTune Safety Features

**Source**: `ArduCopter/mode_autotune.cpp` and `libraries/AC_AutoTune/`

1. **Angle Error Limits**:
   - Traditional: `AUTOTUNE_ANGLE_MAX_RP_CD = 3000` (30°)
   - Helicopter: Configurable per axis
   - Aborts tune if exceeded

2. **Pilot Override**:
   - Any significant stick input pauses testing
   - Pilot maintains control authority at all times

3. **Automatic Disarm**: Disarms on landing detection
   ```cpp
   if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
       copter.arming.disarm(AP_Arming::Method::LANDED);
   }
   ```

4. **Mode Restrictions**: Only allows entry from specific safe modes

5. **Testing Timeouts**: Each test step has timeout protection (source: `libraries/AC_AutoTune/AC_AutoTune_Heli.cpp:29`)
   - `AUTOTUNE_TESTING_STEP_TIMEOUT_MS = 5000` (5 seconds)

### Post-AutoTune Verification

After accepting AutoTune gains:

1. **Hover Test**: Verify stable hover with minimal drift
2. **Slow Maneuvers**: Test gentle pitch/roll/yaw inputs
3. **Faster Maneuvers**: Progressively test quicker movements
4. **Flight Modes**: Test all intended flight modes
5. **Parameter Backup**: Save tuned parameters to file

If issues found:
- Consider re-running AutoTune
- Manually adjust problematic gains
- Check for mechanical issues (bent props, loose arms)

---

## QuickTune System

QuickTune provides rapid automated tuning using oscillation detection, completing in 1-3 minutes versus 10-20 minutes for AutoTune.

### QuickTune Overview

**Source**: `libraries/AP_Quicktune/AP_Quicktune.cpp`

QuickTune operates by:
1. **Gain Increase**: Rapidly doubles gains over configured time period
2. **Oscillation Detection**: Monitors for onset of oscillations using gyro analysis
3. **Gain Reduction**: Backs off gain by configured margin when oscillation detected
4. **I-Term Setting**: Automatically sets I gain as ratio of final P gain
5. **Filter Adjustment**: Optionally sets filter frequencies based on `INS_GYRO_FILTER`

**Key Advantages**:
- Very fast (1-3 minutes total)
- Can be used with 2 or 3-position switch
- Automatic parameter saving option
- Suitable for in-field tuning
- Handles asymmetric vehicles better than AutoTune

**Limitations**:
- Requires reasonable starting tune
- Less conservative than AutoTune
- May need manual refinement after
- Sensitive to vibration and noise

### QuickTune Parameters

**Source**: `libraries/AP_Quicktune/AP_Quicktune.cpp:28-130`

#### Essential Parameters

| Parameter | Description | Default | Range | Units |
|-----------|-------------|---------|-------|-------|
| `QUIK_ENABLE` | Enable QuickTune | 0 | 0: Disabled, 1: Enabled | - |
| `QUIK_AXES` | Axes to tune (bitmask) | 7 | 0: Roll, 1: Pitch, 2: Yaw | - |
| `QUIK_DOUBLE_TIME` | Time to double gains | 10 | 5 - 20 | seconds |
| `QUIK_GAIN_MARGIN` | Gain reduction after oscillation | 60 | 20 - 80 | percent |
| `QUIK_OSC_SMAX` | Oscillation detection threshold | 4 | 1 - 10 | - |

#### Advanced Parameters

| Parameter | Description | Default | Range | Units |
|-----------|-------------|---------|-------|-------|
| `QUIK_YAW_P_MAX` | Maximum yaw P gain | 0.5 | 0.1 - 3.0 | - |
| `QUIK_YAW_D_MAX` | Maximum yaw D gain | 0.01 | 0.001 - 1.0 | - |
| `QUIK_RP_PI_RATIO` | Roll/Pitch P to I ratio | 1.0 | 1.0 - 2.0 | - |
| `QUIK_Y_PI_RATIO` | Yaw P to I ratio | 10 | 1.0 - 20 | - |
| `QUIK_AUTO_FILTER` | Auto-set filters | 1 | 0: Disabled, 1: Enabled | - |
| `QUIK_AUTO_SAVE` | Auto-save delay | 0 | 0 - 60 | seconds |
| `QUIK_REDUCE_MAX` | Maximum gain reduction allowed | 20 | 0 - 100 | percent |
| `QUIK_OPTIONS` | Additional options | 0 | 0: Two Position Switch | bitmask |
| `QUIK_ANGLE_MAX` | Maximum angle error for abort | 10 | 5 - 45 | degrees |

### Parameter Details

**`QUIK_DOUBLE_TIME`** (source: line 49):
- Controls tuning speed
- Shorter time = faster but more aggressive tuning
- Longer time = slower but more conservative
- Recommended: 10s for first tune, can reduce to 8s for refinement

**`QUIK_GAIN_MARGIN`** (source: line 57):
- Percentage to reduce gain after oscillation detected
- 60% means final gain = 60% of oscillation point
- Higher value = more conservative tune
- Lower value = more aggressive (risk of remaining oscillations)

**`QUIK_OSC_SMAX`** (source: line 64):
- Threshold for oscillation detection
- Lower value = detects oscillations earlier (more conservative)
- Higher value = allows more movement before detecting (more aggressive)
- Adjust if getting false positive oscillation detections

**`QUIK_REDUCE_MAX`** (source: line 114):
- Prevents excessive gain reduction
- 20% limit means gains won't be reduced more than 20% below starting values
- Protects against false oscillation detections at low gains
- Set to 0 if vehicle already well-tuned to prevent reductions

**`QUIK_AUTO_SAVE`** (source: line 106):
- Enables automatic parameter saving after tune completion
- Useful with 2-position switch operation
- Value is delay in seconds before auto-save
- 0 disables auto-save (requires 3-position switch)

**`QUIK_OPTIONS`** (source: line 121):
- Bit 0: Two Position Switch mode
  - High position starts tune
  - Low position stops tune
  - Must use with `QUIK_AUTO_SAVE` > 0

**`QUIK_AUTO_FILTER`** (source: line 99):
- When enabled, automatically sets rate controller filters based on `INS_GYRO_FILTER`
- Sets `ATC_RAT_*_FLTD` = `INS_GYRO_FILTER` × 0.5
- Sets `ATC_RAT_*_FLTT` = `INS_GYRO_FILTER` × 0.5
- Recommended to leave enabled

### QuickTune Procedure

**Configuration (Three-Position Switch)**:
```
1. Set QUIK_ENABLE = 1
2. Set QUIK_AXES = 7 (tune all axes)
3. Set QUIK_DOUBLE_TIME = 10
4. Set QUIK_GAIN_MARGIN = 60
5. Configure 3-position RC switch:
   - Low: Disabled
   - Mid: Tuning
   - High: Save and Complete
```

**Configuration (Two-Position Switch)**:
```
1. Set QUIK_ENABLE = 1
2. Set QUIK_AXES = 7
3. Set QUIK_OPTIONS = 1 (enable two-position mode)
4. Set QUIK_AUTO_SAVE = 3 (auto-save after 3 seconds)
5. Configure 2-position RC switch:
   - Low: Disabled
   - High: Tuning
```

**Flight Procedure**:
```
1. Take off in Loiter or AltHold mode
2. Climb to 10+ meters (higher than AutoTune)
3. Switch QuickTune to mid/high position
4. Make sharp stick inputs on axis being tuned:
   - Roll axis: Sharp left-right inputs
   - Pitch axis: Sharp forward-back inputs
   - Yaw axis: Sharp yaw inputs
5. Continue inputs until completion tone
6. 3-position: Switch to high to save, low to abort
7. 2-position: Wait for auto-save, or switch low to abort
```

**Timing**:
- Roll: ~30-60 seconds
- Pitch: ~30-60 seconds
- Yaw: ~30-60 seconds
- Total: 1.5 - 3 minutes

### QuickTune Safety and Abort Conditions

**Abort Triggers** (source: `libraries/AP_Quicktune/AP_Quicktune.cpp:125-128`):

1. **Excessive Angle Error**:
   - Aborts if angle error exceeds `QUIK_ANGLE_MAX` (default 10°)
   - Error message: "Quicktune: attitude error ABORTING"
   - If false positive, increase `QUIK_ANGLE_MAX` or `QUIK_DOUBLE_TIME`

2. **Mode Change**:
   - Switching to non-quicktune mode aborts and restores original gains
   - Prevents leaving vehicle in intermediate tuning state

3. **Disarm**:
   - Disarming during tune aborts and restores gains

4. **Switch to Low Position**:
   - Manually abort by switching to low position

**Recovery Procedure**:
If QuickTune aborts or produces poor results:
```
1. Original parameters automatically restored on abort
2. Land and review logs for oscillation detection
3. Adjust QUIK_DOUBLE_TIME (increase for slower tune)
4. Adjust QUIK_OSC_SMAX (decrease for more conservative)
5. Check for mechanical issues or vibration problems
6. Consider manual tuning of problem axis
```

### QuickTune vs AutoTune Comparison

| Feature | QuickTune | AutoTune |
|---------|-----------|----------|
| **Duration** | 1-3 minutes | 10-20 minutes |
| **Battery Usage** | Low (~10%) | High (~30-40%) |
| **Pilot Input** | Required (active stick inputs) | Minimal (hands-off) |
| **Conservative** | Moderate | High |
| **Asymmetric Vehicles** | Handles well | May struggle |
| **Starting Tune Requirement** | Must be flyable | Can be very rough |
| **Switch Options** | 2 or 3-position | Typically 3-position |
| **Post-Tune Refinement** | Often beneficial | Usually good as-is |
| **Vibration Sensitivity** | Higher | Lower |

**When to Use QuickTune**:
- Field tuning after configuration changes
- Refining existing tune
- Time/battery constrained
- Asymmetric or unusual frames
- Already have reasonable tune

**When to Use AutoTune**:
- First-time tune
- Poor starting tune
- Maximum conservatism desired
- Battery capacity available
- Standard symmetric frame

---

## Rate Controllers

Rate controllers form the innermost control loop, converting desired angular rates to motor commands with minimal latency.

### Rate Controller Architecture

**Source**: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h`

The rate controllers operate at 400 Hz (2.5 ms loop time) and implement PID control:

```cpp
rate_target = attitude_controller_output;  // From outer loop
rate_error = rate_target - gyro_measurement;
rate_output = P_term + I_term + D_term + FF_term;  // Helicopter only for FF
```

**Controller Structure** (source: lines 50-55):
```cpp
AC_PID& get_rate_roll_pid();
AC_PID& get_rate_pitch_pid();
AC_PID& get_rate_yaw_pid();
```

### Rate Controller Parameters

**Source**: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h:10-39`

#### Roll Rate Controller (`ATC_RAT_RLL_*`)

| Parameter | Description | Default | Typical Range | Units |
|-----------|-------------|---------|---------------|-------|
| `ATC_RAT_RLL_P` | Rate roll proportional gain | 0.135 | 0.08 - 0.25 | - |
| `ATC_RAT_RLL_I` | Rate roll integral gain | 0.135 | 0.08 - 0.25 | - |
| `ATC_RAT_RLL_D` | Rate roll derivative gain | 0.0036 | 0.002 - 0.010 | - |
| `ATC_RAT_RLL_IMAX` | Rate roll integrator maximum | 0.5 | 0.25 - 0.75 | - |
| `ATC_RAT_RLL_FILT` | Rate roll input filter | 20.0 | 10.0 - 40.0 | Hz |
| `ATC_RAT_RLL_FLTD` | Rate roll D term filter | 20.0 | 10.0 - 40.0 | Hz |
| `ATC_RAT_RLL_FLTT` | Rate roll target filter | 20.0 | 10.0 - 40.0 | Hz |
| `ATC_RAT_RLL_FLTE` | Rate roll error filter | 20.0 | 2.0 - 40.0 | Hz |
| `ATC_RAT_RLL_FF` | Rate roll feed-forward | 0.0 | 0.0 - 0.5 | - |
| `ATC_RAT_RLL_SMAX` | Rate roll slew rate max | 0 | 0 - 200 | deg/s/s |

#### Pitch Rate Controller (`ATC_RAT_PIT_*`)

| Parameter | Description | Default | Typical Range | Units |
|-----------|-------------|---------|---------------|-------|
| `ATC_RAT_PIT_P` | Rate pitch proportional gain | 0.135 | 0.08 - 0.25 | - |
| `ATC_RAT_PIT_I` | Rate pitch integral gain | 0.135 | 0.08 - 0.25 | - |
| `ATC_RAT_PIT_D` | Rate pitch derivative gain | 0.0036 | 0.002 - 0.010 | - |
| `ATC_RAT_PIT_IMAX` | Rate pitch integrator maximum | 0.5 | 0.25 - 0.75 | - |
| `ATC_RAT_PIT_FILT` | Rate pitch input filter | 20.0 | 10.0 - 40.0 | Hz |
| `ATC_RAT_PIT_FLTD` | Rate pitch D term filter | 20.0 | 10.0 - 40.0 | Hz |
| `ATC_RAT_PIT_FLTT` | Rate pitch target filter | 20.0 | 10.0 - 40.0 | Hz |
| `ATC_RAT_PIT_FLTE` | Rate pitch error filter | 20.0 | 2.0 - 40.0 | Hz |
| `ATC_RAT_PIT_FF` | Rate pitch feed-forward | 0.0 | 0.0 - 0.5 | - |
| `ATC_RAT_PIT_SMAX` | Rate pitch slew rate max | 0 | 0 - 200 | deg/s/s |

#### Yaw Rate Controller (`ATC_RAT_YAW_*`)

| Parameter | Description | Default | Typical Range | Units |
|-----------|-------------|---------|---------------|-------|
| `ATC_RAT_YAW_P` | Rate yaw proportional gain | 0.180 | 0.15 - 0.30 | - |
| `ATC_RAT_YAW_I` | Rate yaw integral gain | 0.018 | 0.015 - 0.030 | - |
| `ATC_RAT_YAW_D` | Rate yaw derivative gain | 0.0 | 0.0 - 0.005 | - |
| `ATC_RAT_YAW_IMAX` | Rate yaw integrator maximum | 0.5 | 0.25 - 0.75 | - |
| `ATC_RAT_YAW_FILT` | Rate yaw input filter | 2.5 | 1.0 - 10.0 | Hz |
| `ATC_RAT_YAW_FLTD` | Rate yaw D term filter | 2.5 | 1.0 - 10.0 | Hz |
| `ATC_RAT_YAW_FLTT` | Rate yaw target filter | 2.5 | 1.0 - 10.0 | Hz |
| `ATC_RAT_YAW_FLTE` | Rate yaw error filter | 2.5 | 1.0 - 10.0 | Hz |
| `ATC_RAT_YAW_FF` | Rate yaw feed-forward | 0.0 | 0.0 - 0.3 | - |
| `ATC_RAT_YAW_SMAX` | Rate yaw slew rate max | 0 | 0 - 100 | deg/s/s |

### Default Gain Definitions

**Source**: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h:10-39`

```cpp
// Roll and Pitch defaults
#define AC_ATC_MULTI_RATE_RP_P           0.135f
#define AC_ATC_MULTI_RATE_RP_I           0.135f
#define AC_ATC_MULTI_RATE_RP_D           0.0036f
#define AC_ATC_MULTI_RATE_RP_IMAX        0.5f
#define AC_ATC_MULTI_RATE_RPY_FILT_HZ    20.0f

// Yaw defaults
#define AC_ATC_MULTI_RATE_YAW_P          0.180f
#define AC_ATC_MULTI_RATE_YAW_I          0.018f
#define AC_ATC_MULTI_RATE_YAW_D          0.0f
#define AC_ATC_MULTI_RATE_YAW_IMAX       0.5f
#define AC_ATC_MULTI_RATE_YAW_FILT_HZ    2.5f
```

### Manual Rate Controller Tuning

#### Step 1: Rate P Tuning

**Objective**: Find P gain that provides responsive control without oscillation

**Procedure**:
```
1. Start with P = 0.10 (if unknown)
2. Hover and make sharp 20-30° roll inputs
3. Observe response:
   - P too low: Slow, mushy response
   - P too high: Rapid oscillations during/after input
   - P optimal: Quick, crisp response with minimal overshoot
4. Increase P by 0.01 until oscillations appear
5. Reduce by 20-30% for safety margin
6. Repeat for pitch axis
```

**Typical Signs**:
- **P = 0.08**: Very sluggish, drifts during maneuvers
- **P = 0.135**: Good starting point for 5" props
- **P = 0.18**: Aggressive, suitable for race quads
- **P = 0.25+**: Very aggressive, usually too high for standard frames

#### Step 2: Rate D Tuning

**Objective**: Add damping to reduce overshoot and oscillations

**Procedure**:
```
1. Start with D = P × 0.027 (rule of thumb)
2. Make sharp inputs and observe overshoot
3. Increase D if:
   - Overshooting target angle
   - Ringing/oscillations after maneuvers
   - Propwash oscillations in descents
4. Typical range: D = 0.002 to 0.008
5. D too high causes:
   - Jittery motors
   - Noise amplification
   - Hot motors
```

**D-Term Filter Tuning** (`ATC_RAT_*_FLTD`):
- Filters noise before D-term calculation
- Default: 20 Hz (source: `AC_ATC_MULTI_RATE_RPY_FILT_HZ`)
- Higher filtering (lower Hz) if motors hot/noisy
- Lower filtering (higher Hz) for better damping response
- Typical range: 15-30 Hz for roll/pitch

#### Step 3: Rate I Tuning

**Objective**: Eliminate steady-state error and improve wind rejection

**Procedure**:
```
1. Start with I = P (standard recommendation)
2. Hover in light wind
3. Observe drift:
   - I too low: Persistent drift, doesn't return to center
   - I too high: Slow oscillations (period > 3 seconds)
   - I optimal: Returns to center, holds position
4. I is often less critical than P and D
5. Typical: I = P (roll/pitch), I = P/10 (yaw)
```

**I-Term Limits** (`ATC_RAT_*_IMAX`):
- Prevents excessive integral accumulation
- Default: 0.5 (50% throttle authority)
- Increase if fighting strong persistent winds
- Decrease if seeing slow large-amplitude oscillations

### Rate Controller Filter Configuration

**Filter Types**:

1. **`ATC_RAT_*_FILT`**: Input rate filter
   - Filters incoming rate measurements
   - Default: 20 Hz roll/pitch, 2.5 Hz yaw
   - Lower for noisy gyros

2. **`ATC_RAT_*_FLTD`**: D-term filter
   - Filters error before D calculation
   - Critical for preventing D-term noise amplification
   - Default: 20 Hz roll/pitch, 2.5 Hz yaw

3. **`ATC_RAT_*_FLTT`**: Target rate filter
   - Filters desired rate command
   - Reduces excitation of vehicle resonances
   - Default: 20 Hz roll/pitch, 2.5 Hz yaw

4. **`ATC_RAT_*_FLTE`**: Error filter
   - Filters rate error signal
   - Additional smoothing for rate commands
   - Default: 20 Hz roll/pitch, 2.5 Hz yaw

**Filter Tuning Guidelines**:
```
Rule of Thumb: All filters ≈ INS_GYRO_FILTER
- If INS_GYRO_FILTER = 40 Hz, set rate filters to 20 Hz
- If INS_GYRO_FILTER = 20 Hz, set rate filters to 10 Hz
- Yaw filters typically 5-10× lower than roll/pitch
```

### Yaw-Specific Considerations

**Yaw Rate Differences**:
- **Lower P gain**: Yaw typically 1.3× roll/pitch P (source: 0.180 vs 0.135)
- **Much lower I gain**: Yaw I typically P/10 (source: 0.018 vs 0.180)
- **Minimal D gain**: Usually 0.0 for yaw (source: `AC_ATC_MULTI_RATE_YAW_D = 0.0f`)
- **Lower filter frequencies**: 2.5 Hz vs 20 Hz (source: `AC_ATC_MULTI_RATE_YAW_FILT_HZ = 2.5f`)

**Yaw Tuning Priority**:
1. Tune roll and pitch first
2. Then tune yaw independently
3. Yaw is typically less critical for stability

---

## Attitude Controllers

Attitude controllers form the outer control loop, converting angle errors to desired rate commands.

### Attitude Controller Architecture

Attitude controllers run at 400 Hz and implement P-only control (no I or D terms at angle level):

```
angle_error = desired_angle - current_angle;
desired_rate = ATC_ANG_*_P × angle_error;
// desired_rate is passed to rate controller
```

### Attitude Controller Parameters

| Parameter | Description | Default | Typical Range | Units |
|-----------|-------------|---------|---------------|-------|
| `ATC_ANG_RLL_P` | Angle roll P gain | 4.5 | 4.0 - 10.0 | - |
| `ATC_ANG_PIT_P` | Angle pitch P gain | 4.5 | 4.0 - 10.0 | - |
| `ATC_ANG_YAW_P` | Angle yaw P gain | 4.5 | 4.0 - 8.0 | - |

**Parameter Access** (source: `ArduCopter/tuning.cpp:38-40,58-60`):
```cpp
case TUNING_STABILIZE_ROLL_PITCH_KP:
    attitude_control->get_angle_roll_p().set_kP(tuning_value);
    attitude_control->get_angle_pitch_p().set_kP(tuning_value);

case TUNING_STABILIZE_YAW_KP:
    attitude_control->get_angle_yaw_p().set_kP(tuning_value);
```

### Angle P Tuning

**Objective**: Determine how aggressively angle errors are corrected

**Tuning Procedure**:
```
1. Ensure rate controllers tuned first
2. Start with P = 4.5
3. In Stabilize mode, make 45° bank angle inputs
4. Observe tracking:
   - P too low: Sluggish, doesn't reach commanded angle
   - P too high: Overshoot, oscillations at ~1-2 Hz
   - P optimal: Quickly reaches angle, minimal overshoot
5. Increase by 0.5 until oscillations appear
6. Reduce by 20% for safety margin
```

**Effect on Flight Modes**:
- **Stabilize**: Directly affects stick responsiveness
- **AltHold/Loiter**: Affects disturbance rejection
- **Auto**: Affects waypoint tracking tightness

**Typical Values by Vehicle Size**:
- Small (250mm): 6.0 - 8.0
- Medium (450mm): 4.5 - 6.0
- Large (800mm+): 3.5 - 5.0
- Helicopters: 3.0 - 4.5

### Relationship to Rate Controllers

The angle and rate controllers work together:

```
Large angle error → High desired rate (angle P)
High rate error → High motor command (rate PID)
```

**Tuning Order is Critical**:
1. ✅ Tune rate controllers first (P, D, I)
2. ✅ Then tune angle P
3. ❌ Never tune angle P before rate controllers

**Interaction Example**:
```
Scenario: 30° angle error in roll

With ATC_ANG_RLL_P = 4.5:
desired_rate = 4.5 × 30° = 135°/s

Rate controller then converts 135°/s target to motor commands
using ATC_RAT_RLL_P/I/D
```

### Input Shaping (`ATC_INPUT_TC`)

**Parameter**: `ATC_INPUT_TC` (source: `ArduCopter/tuning.cpp:151`)
- **Description**: Input time constant for roll/pitch
- **Function**: Smooths pilot stick inputs
- **Default**: 0.15 seconds (typical)
- **Range**: 0.0 - 0.5 seconds
- **Tunable via**: `TUNING_RC_FEEL_RP` (function 45)

**Effect**:
- **Low values (0.0 - 0.1)**: Very direct, "locked-in" feel
- **Medium values (0.15 - 0.25)**: Smooth, cinematic feel
- **High values (0.3 - 0.5)**: Very smooth, may feel sluggish

**Tuning Recommendation**:
- Sport/racing: 0.05 - 0.10
- Aerial photography: 0.20 - 0.30
- General flying: 0.15 (default)

### Angle Limits

**Maximum Lean Angle** (`ATC_ANG_LIM_TC`):
- Limits maximum tilt angle in auto modes
- Protects against excessive lean during aggressive maneuvers
- Separate from manual mode angle limits
- Typical: 45° for agile, 30° for stable platforms

**Position Controller Angle Max** (`TUNING_POS_CONTROL_ANGLE_MAX`, function 59):
- Maximum lean angle for position controller
- Used in Loiter, PosHold, Auto modes
- Source: `ArduCopter/tuning.cpp:194-196`

---

## Filter Configuration

Proper filter configuration is critical for noise rejection while maintaining control responsiveness. Filters must balance noise reduction against phase lag.

### Filter Hierarchy

ArduCopter implements multiple filter stages:

```
Raw Gyro → INS_GYRO_FILTER → Rate Controller Filters → Motor Output
            (20-80 Hz)        (10-40 Hz)
```

### Primary Gyro Filter

**Parameter**: `INS_GYRO_FILTER`
- **Location**: Sensor layer (AP_InertialSensor)
- **Function**: Primary anti-aliasing and noise rejection
- **Typical Range**: 20 - 80 Hz
- **Default**: 20 Hz (conservative)

**Tuning Guidelines**:
```
Frame Size / Props          | Recommended INS_GYRO_FILTER
----------------------------|----------------------------
Large (>800mm, slow props) | 20 - 30 Hz
Medium (450mm)             | 40 - 60 Hz
Small (250mm, fast props)  | 60 - 80 Hz
Racing quad                | 80 - 100 Hz
```

**Effects**:
- **Too Low (10 Hz)**: Excessive phase lag, reduced control authority
- **Optimal (matched to frame)**: Clean control, minimal noise
- **Too High (120 Hz)**: Motor noise passes through, hot motors

### Rate Controller Filters

All rate controller filters typically set as fractions of `INS_GYRO_FILTER`:

**Roll/Pitch Filters** (source: `AC_ATC_MULTI_RATE_RPY_FILT_HZ = 20.0f`):
- `ATC_RAT_RLL_FILT` = `INS_GYRO_FILTER` × 0.5
- `ATC_RAT_RLL_FLTD` = `INS_GYRO_FILTER` × 0.5
- `ATC_RAT_RLL_FLTT` = `INS_GYRO_FILTER` × 0.5
- `ATC_RAT_RLL_FLTE` = `INS_GYRO_FILTER` × 0.5

**Yaw Filters** (source: `AC_ATC_MULTI_RATE_YAW_FILT_HZ = 2.5f`):
- `ATC_RAT_YAW_FILT` = 2.5 - 5.0 Hz (much lower than roll/pitch)
- Yaw dynamics are slower, require more filtering
- Source: `ArduCopter/tuning.cpp:184-186`

### QuickTune Automatic Filtering

**Source**: `libraries/AP_Quicktune/AP_Quicktune.cpp:99`

When `QUIK_AUTO_FILTER = 1`:
- Automatically configures rate controller filters
- Sets `ATC_RAT_*_FLTD` = `INS_GYRO_FILTER` × `FLTD_MUL` (0.5)
- Sets `ATC_RAT_*_FLTT` = `INS_GYRO_FILTER` × `FLTT_MUL` (0.5)
- Simplifies filter configuration

### Harmonic Notch Filters

Modern ArduPilot includes harmonic notch filters for targeting specific noise frequencies:

**Parameters**:
- `INS_HNTCH_ENABLE`: Enable harmonic notch filter
- `INS_HNTCH_FREQ`: Center frequency (Hz)
- `INS_HNTCH_BW`: Bandwidth (Hz)
- `INS_HNTCH_ATT`: Attenuation (dB)
- `INS_HNTCH_REF`: Reference throttle

**Typical Configuration**:
```
Motor noise fundamental frequency: 150-400 Hz
INS_HNTCH_ENABLE = 1
INS_HNTCH_MODE = 1 (throttle-based)
INS_HNTCH_FREQ = 200 (typical for 5" props)
INS_HNTCH_BW = 100
INS_HNTCH_ATT = 40
```

**Setup Procedure**:
1. Perform test flight with logging enabled
2. Analyze FFT of gyro data
3. Identify motor noise peak frequency
4. Configure notch filter at that frequency
5. Enable harmonics if multiple peaks present

### Filter Tuning Workflow

**Step 1: Baseline Configuration**
```
1. Set INS_GYRO_FILTER based on frame size (table above)
2. Set rate controller filters to INS_GYRO_FILTER / 2
3. Test flight and record logs
```

**Step 2: Vibration Analysis**
```
1. Check VIBE messages in logs
2. Acceptable levels:
   - Clip count: 0 (any clipping is problematic)
   - Vibration X/Y/Z: <30 (ideally <15)
3. If high vibration:
   - Fix mechanical issues first
   - Balance props
   - Check motor bearings
   - Soft-mount FC if necessary
```

**Step 3: Filter Optimization**
```
1. If motors hot/noisy:
   - Decrease INS_GYRO_FILTER (more filtering)
   - Decrease rate controller filters
   
2. If response sluggish:
   - Increase INS_GYRO_FILTER (less filtering)
   - Increase rate controller filters
   - Check for excessive phase lag

3. For persistent narrow-band noise:
   - Enable harmonic notch filter
   - Target specific frequency
```

**Step 4: Advanced: Notch Filter Setup**
```
1. Enable logging: LOG_BITMASK includes IMU_RAW
2. Test flight (2-3 minutes hover + maneuvers)
3. Download logs and run FFT analysis
4. Identify motor noise peaks
5. Configure INS_HNTCH_* parameters
6. Test flight to verify improvement
7. May allow increasing INS_GYRO_FILTER
```

### Filter Effects on Control

**Phase Lag**:
- All filters introduce phase lag (delay)
- More filtering = more lag = reduced control authority
- Lag accumulates through filter chain:
  ```
  Total lag ≈ INS_GYRO lag + Rate Controller lag
  ```

**Optimal Filtering Philosophy**:
1. Filter as little as necessary, not as much as possible
2. Fix vibration sources rather than over-filtering
3. Target specific noise frequencies with notch filters
4. Balance noise rejection vs. control performance

**Signs of Over-Filtering**:
- Sluggish response despite high PIDs
- Inability to tune out oscillations
- "Wallowing" in turbulence
- Poor disturbance rejection

**Signs of Under-Filtering**:
- Hot motors after flight
- High-frequency oscillations/buzzing
- Motor desynchronization
- Reduced flight time

---

## Parameter Reference

### Complete Parameter List

This section provides a comprehensive reference of all tuning-related parameters.

#### Attitude Control Parameters

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `ATC_ANG_RLL_P` | Angle Roll P gain | 4.5 | - | AC_AttitudeControl |
| `ATC_ANG_PIT_P` | Angle Pitch P gain | 4.5 | - | AC_AttitudeControl |
| `ATC_ANG_YAW_P` | Angle Yaw P gain | 4.5 | - | AC_AttitudeControl |
| `ATC_INPUT_TC` | Attitude control input time constant | 0.15 | seconds | AC_AttitudeControl |
| `ATC_RATE_FF_ENAB` | Rate feedforward enable | 1 | boolean | AC_AttitudeControl |
| `ATC_THR_MIX_MAN` | Throttle vs attitude control mix | 0.5 | - | AC_AttitudeControl |
| `ATC_THR_MIX_MIN` | Throttle mix minimum | 0.1 | - | AC_AttitudeControl |
| `ATC_THR_MIX_MAX` | Throttle mix maximum | 0.5 | - | AC_AttitudeControl |
| `ATC_ANGLE_BOOST` | Angle boost enable | 1 | boolean | AC_AttitudeControl |

#### Rate Control Parameters (Roll)

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `ATC_RAT_RLL_P` | Rate Roll P gain | 0.135 | - | AC_AttitudeControl_Multi.h:11 |
| `ATC_RAT_RLL_I` | Rate Roll I gain | 0.135 | - | AC_AttitudeControl_Multi.h:14 |
| `ATC_RAT_RLL_D` | Rate Roll D gain | 0.0036 | - | AC_AttitudeControl_Multi.h:17 |
| `ATC_RAT_RLL_IMAX` | Rate Roll I maximum | 0.5 | - | AC_AttitudeControl_Multi.h:20 |
| `ATC_RAT_RLL_FILT` | Rate Roll input filter | 20.0 | Hz | AC_AttitudeControl_Multi.h:23 |
| `ATC_RAT_RLL_FLTD` | Rate Roll D term filter | 20.0 | Hz | - |
| `ATC_RAT_RLL_FLTT` | Rate Roll target filter | 20.0 | Hz | - |
| `ATC_RAT_RLL_FLTE` | Rate Roll error filter | 20.0 | Hz | - |
| `ATC_RAT_RLL_FF` | Rate Roll feedforward | 0.0 | - | - |
| `ATC_RAT_RLL_SMAX` | Rate Roll slew rate limit | 0 | deg/s/s | - |

#### Rate Control Parameters (Pitch)

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `ATC_RAT_PIT_P` | Rate Pitch P gain | 0.135 | - | AC_AttitudeControl_Multi.h:11 |
| `ATC_RAT_PIT_I` | Rate Pitch I gain | 0.135 | - | AC_AttitudeControl_Multi.h:14 |
| `ATC_RAT_PIT_D` | Rate Pitch D gain | 0.0036 | - | AC_AttitudeControl_Multi.h:17 |
| `ATC_RAT_PIT_IMAX` | Rate Pitch I maximum | 0.5 | - | AC_AttitudeControl_Multi.h:20 |
| `ATC_RAT_PIT_FILT` | Rate Pitch input filter | 20.0 | Hz | AC_AttitudeControl_Multi.h:23 |
| `ATC_RAT_PIT_FLTD` | Rate Pitch D term filter | 20.0 | Hz | - |
| `ATC_RAT_PIT_FLTT` | Rate Pitch target filter | 20.0 | Hz | - |
| `ATC_RAT_PIT_FLTE` | Rate Pitch error filter | 20.0 | Hz | - |
| `ATC_RAT_PIT_FF` | Rate Pitch feedforward | 0.0 | - | - |
| `ATC_RAT_PIT_SMAX` | Rate Pitch slew rate limit | 0 | deg/s/s | - |

#### Rate Control Parameters (Yaw)

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `ATC_RAT_YAW_P` | Rate Yaw P gain | 0.180 | - | AC_AttitudeControl_Multi.h:26 |
| `ATC_RAT_YAW_I` | Rate Yaw I gain | 0.018 | - | AC_AttitudeControl_Multi.h:29 |
| `ATC_RAT_YAW_D` | Rate Yaw D gain | 0.0 | - | AC_AttitudeControl_Multi.h:32 |
| `ATC_RAT_YAW_IMAX` | Rate Yaw I maximum | 0.5 | - | AC_AttitudeControl_Multi.h:35 |
| `ATC_RAT_YAW_FILT` | Rate Yaw input filter | 2.5 | Hz | AC_AttitudeControl_Multi.h:38 |
| `ATC_RAT_YAW_FLTD` | Rate Yaw D term filter | 2.5 | Hz | - |
| `ATC_RAT_YAW_FLTT` | Rate Yaw target filter | 2.5 | Hz | - |
| `ATC_RAT_YAW_FLTE` | Rate Yaw error filter | 2.5 | Hz | - |
| `ATC_RAT_YAW_FF` | Rate Yaw feedforward | 0.0 | - | - |
| `ATC_RAT_YAW_SMAX` | Rate Yaw slew rate limit | 0 | deg/s/s | - |

#### Transmitter Tuning Parameters

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `TUNE` | Tuning function selection | 0 | - | Parameters.h:302 |
| `TUNE_MIN` | Tuning range minimum | 0.0 | - | Parameters_g2 |
| `TUNE_MAX` | Tuning range maximum | 0.0 | - | Parameters_g2 |

#### AutoTune Parameters (Multicopter)

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `AUTOTUNE_AXES` | Axes to tune (bitmask) | 7 | - | AC_AutoTune |
| `AUTOTUNE_AGGR` | Aggressiveness | 0.1 | - | AC_AutoTune |
| `AUTOTUNE_MIN_D` | Minimum D gain | 0.001 | - | AC_AutoTune |

#### AutoTune Parameters (Helicopter)

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `AUTOTUNE_AXES` | Axes to tune (bitmask) | 1 | - | AC_AutoTune_Heli.cpp:78 |
| `AUTOTUNE_SEQ` | Tuning sequence (bitmask) | 3 | - | AC_AutoTune_Heli.cpp:85 |
| `AUTOTUNE_FRQ_MIN` | Minimum sweep frequency | 10.0 | Hz | AC_AutoTune_Heli.cpp:92 |
| `AUTOTUNE_FRQ_MAX` | Maximum sweep frequency | 70.0 | Hz | AC_AutoTune_Heli.cpp:99 |
| `AUTOTUNE_GN_MAX` | Maximum gain multiplier | 2.0 | - | AC_AutoTune_Heli |

#### QuickTune Parameters

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `QUIK_ENABLE` | Enable QuickTune | 0 | - | AP_Quicktune.cpp:34 |
| `QUIK_AXES` | Axes to tune (bitmask) | 7 | - | AP_Quicktune.cpp:41 |
| `QUIK_DOUBLE_TIME` | Gain doubling time | 10 | seconds | AP_Quicktune.cpp:49 |
| `QUIK_GAIN_MARGIN` | Gain reduction margin | 60 | percent | AP_Quicktune.cpp:57 |
| `QUIK_OSC_SMAX` | Oscillation threshold | 4 | - | AP_Quicktune.cpp:64 |
| `QUIK_YAW_P_MAX` | Max yaw P gain | 0.5 | - | AP_Quicktune.cpp:71 |
| `QUIK_YAW_D_MAX` | Max yaw D gain | 0.01 | - | AP_Quicktune.cpp:78 |
| `QUIK_RP_PI_RATIO` | Roll/Pitch P to I ratio | 1.0 | - | AP_Quicktune.cpp:85 |
| `QUIK_Y_PI_RATIO` | Yaw P to I ratio | 10 | - | AP_Quicktune.cpp:92 |
| `QUIK_AUTO_FILTER` | Auto-configure filters | 1 | boolean | AP_Quicktune.cpp:99 |
| `QUIK_AUTO_SAVE` | Auto-save delay | 0 | seconds | AP_Quicktune.cpp:106 |
| `QUIK_REDUCE_MAX` | Maximum gain reduction | 20 | percent | AP_Quicktune.cpp:114 |
| `QUIK_OPTIONS` | Additional options | 0 | bitmask | AP_Quicktune.cpp:121 |
| `QUIK_ANGLE_MAX` | Max angle error for abort | 10 | degrees | AP_Quicktune.cpp:128 |

#### Position and Navigation Parameters

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `PSC_POSXY_P` | Position XY P gain | 1.0 | - | AC_PosControl |
| `PSC_VELXY_P` | Velocity XY P gain | 2.0 | - | AC_PosControl |
| `PSC_VELXY_I` | Velocity XY I gain | 1.0 | - | AC_PosControl |
| `PSC_VELXY_D` | Velocity XY D gain | 0.5 | - | AC_PosControl |
| `PSC_VELXY_IMAX` | Velocity XY I maximum | 1.0 | - | AC_PosControl |
| `PSC_VELXY_FILT` | Velocity XY filter | 5.0 | Hz | AC_PosControl |
| `PSC_POSZ_P` | Position Z P gain | 1.0 | - | AC_PosControl |
| `PSC_VELZ_P` | Velocity Z P gain | 5.0 | - | AC_PosControl |
| `PSC_ACCZ_P` | Acceleration Z P gain | 0.5 | - | AC_PosControl |
| `PSC_ACCZ_I` | Acceleration Z I gain | 1.0 | - | AC_PosControl |
| `PSC_ACCZ_D` | Acceleration Z D gain | 0.0 | - | AC_PosControl |

#### Filter Parameters

| Parameter | Description | Default | Units | Source |
|-----------|-------------|---------|-------|--------|
| `INS_GYRO_FILTER` | Gyro low-pass filter | 20 | Hz | AP_InertialSensor |
| `INS_ACCEL_FILTER` | Accelerometer low-pass filter | 20 | Hz | AP_InertialSensor |
| `INS_HNTCH_ENABLE` | Harmonic notch enable | 0 | - | AP_InertialSensor |
| `INS_HNTCH_FREQ` | Harmonic notch frequency | 0 | Hz | AP_InertialSensor |
| `INS_HNTCH_BW` | Harmonic notch bandwidth | 40 | Hz | AP_InertialSensor |
| `INS_HNTCH_ATT` | Harmonic notch attenuation | 40 | dB | AP_InertialSensor |
| `INS_HNTCH_MODE` | Harmonic notch mode | 0 | - | AP_InertialSensor |
| `INS_HNTCH_REF` | Harmonic notch reference | 0 | - | AP_InertialSensor |

### Parameter Groups and Relationships

**Cascaded Control Relationship**:
```
Pilot Input
    ↓
[Angle P] (ATC_ANG_*_P)
    ↓ (desired rate)
[Rate PID] (ATC_RAT_*_P/I/D)
    ↓ (motor command)
Motors
```

**Filter Chain**:
```
Gyro → INS_GYRO_FILTER → ATC_RAT_*_FILT → ATC_RAT_*_FLTD/FLTT/FLTE → PID
```

**AutoTune Effects**:
- Directly modifies: `ATC_RAT_*_P`, `ATC_RAT_*_I`, `ATC_RAT_*_D`
- May modify: `ATC_ANG_*_P` (depending on configuration)
- Does not modify: Filter parameters, position control

**QuickTune Effects**:
- Directly modifies: `ATC_RAT_*_P`, `ATC_RAT_*_I`, `ATC_RAT_*_D`
- Optionally modifies: `ATC_RAT_*_FLTD`, `ATC_RAT_*_FLTT` (if `QUIK_AUTO_FILTER = 1`)
- Does not modify: `ATC_ANG_*_P`, position control

---

## Troubleshooting

### Common Issues and Solutions

#### Oscillations

**Slow Oscillations (1-3 Hz)**

**Symptoms**:
- Slow rocking motion
- Worsens in windy conditions
- Period: 0.3 - 1.0 seconds

**Likely Causes**:
1. **Angle P too high**
   - Solution: Reduce `ATC_ANG_RLL_P` / `ATC_ANG_PIT_P` by 10-20%
   
2. **Rate I too high**
   - Solution: Reduce `ATC_RAT_RLL_I` / `ATC_RAT_PIT_I` by 20%
   
3. **Position P too high** (in Loiter)
   - Solution: Reduce `PSC_POSXY_P`

**Fast Oscillations (5-15 Hz)**

**Symptoms**:
- Rapid shaking or buzzing
- Hot motors after flight
- High-frequency noise in logs
- Period: 0.067 - 0.2 seconds

**Likely Causes**:
1. **Rate P too high**
   - Solution: Reduce `ATC_RAT_RLL_P` / `ATC_RAT_PIT_P` by 10-20%
   
2. **Rate D too high**
   - Solution: Reduce `ATC_RAT_RLL_D` / `ATC_RAT_PIT_D` by 30-50%
   - Increase D-term filtering (`ATC_RAT_*_FLTD`)
   
3. **Insufficient filtering**
   - Solution: Reduce `INS_GYRO_FILTER` and rate controller filters
   
4. **Mechanical vibration**
   - Solution: Fix vibration sources (balance props, check bearings)

**Propwash Oscillations**

**Symptoms**:
- Oscillations during descents
- Stable in climbs and level flight
- Occurs when descending through own rotor wash

**Solutions**:
1. Increase D gain: `ATC_RAT_RLL_D` / `ATC_RAT_PIT_D` by 20-30%
2. Reduce descent rate in auto modes
3. Slightly reduce P gain if D increase insufficient
4. Check motor/ESC timing configuration

#### Sluggish Response

**Symptoms**:
- Slow to respond to stick inputs
- Drifts during maneuvers
- Doesn't reach commanded angles
- "Mushy" feel

**Likely Causes**:
1. **Rate P too low**
   - Solution: Increase `ATC_RAT_RLL_P` / `ATC_RAT_PIT_P` by 20%
   
2. **Angle P too low**
   - Solution: Increase `ATC_ANG_RLL_P` / `ATC_ANG_PIT_P` by 0.5-1.0
   
3. **Excessive filtering**
   - Solution: Increase `INS_GYRO_FILTER` (less filtering)
   - Increase rate controller filter frequencies
   
4. **Input time constant too high**
   - Solution: Reduce `ATC_INPUT_TC`

#### Altitude Hold Problems

**Symptoms - Oscillating Altitude**:
- Porpoising up and down
- Unstable hover altitude
- Constant throttle corrections

**Solutions**:
1. **Reduce Altitude P**: Lower `PSC_POSZ_P`
2. **Reduce Throttle Rate P**: Lower `PSC_VELZ_P`
3. **Check barometer health**: Verify baro not affected by prop wash
4. **Increase Accel Z I**: Higher `PSC_ACCZ_I` for better trim

**Symptoms - Altitude Drift**:
- Slowly climbs or descends
- Cannot maintain altitude
- Requires constant throttle input

**Solutions**:
1. **Increase Throttle Rate I**: Higher `PSC_ACCZ_I`
2. **Check hover throttle**: Verify `MOT_HOVER_LEARN` enabled
3. **Barometer mounting**: Ensure proper foam covering
4. **EKF health**: Check for EKF altitude variances

#### Loiter / Position Hold Issues

**Symptoms - Position Drift**:
- Drifts away from position
- Cannot hold loiter
- Wanders in wind

**Solutions**:
1. **GPS health**: Check HDOP <1.5, satellite count >10
2. **Increase Position P**: Higher `PSC_POSXY_P`
3. **Increase Velocity I**: Higher `PSC_VELXY_I` for wind rejection
4. **Check compass**: Verify compass cal and health
5. **EKF health**: Check EKF position variances

**Symptoms - Oscillating Position**:
- Bounces back and forth
- Overshoots target position
- Unstable loiter

**Solutions**:
1. **Reduce Position P**: Lower `PSC_POSXY_P`
2. **Reduce Velocity P**: Lower `PSC_VELXY_P`
3. **Reduce Velocity D**: Lower `PSC_VELXY_D`
4. **Increase Velocity filtering**: Lower `PSC_VELXY_FILT`

#### Toilet Bowling

**Symptoms**:
- Circular flight path expanding outward
- Occurs in Loiter or Auto modes
- Stable in Stabilize

**Likely Causes and Solutions**:
1. **Compass calibration issues**
   - Solution: Recalibrate compass in all orientations
   - Check for magnetic interference near compass

2. **Compass/GPS orientation mismatch**
   - Solution: Verify `COMPASS_ORIENT` and GPS mounting

3. **Vibration affecting compass**
   - Solution: Move compass away from motors
   - Use external compass on mast

#### Flip on Takeoff or Landing

**Symptoms**:
- Vehicle flips immediately on arming and throttle up
- Cannot stabilize even with full stick deflection
- Rapid uncontrolled rotation

**Likely Causes**:
1. **Motor direction incorrect**
   - Solution: Verify all motors spinning correct direction
   - Check motor numbering matches frame type

2. **Props installed backwards**
   - Solution: Verify prop rotation direction matches motor
   - CW motors need CW props, CCW need CCW props

3. **Incorrect frame type**
   - Solution: Verify `FRAME_CLASS` and `FRAME_TYPE` correct

4. **ESC calibration**
   - Solution: Calibrate ESC endpoints
   - Verify all ESCs starting at same throttle point

5. **Accelerometer calibration**
   - Solution: Recalibrate accelerometer on level surface

#### AutoTune or QuickTune Failure

**AutoTune Aborts with "Attitude Error"**:

**Causes**:
- Insufficient starting tune
- Wind too strong
- Vehicle too heavy for motors
- Battery voltage too low

**Solutions**:
1. Manually tune to flyable state first
2. Wait for calmer conditions (wind <5 m/s)
3. Reduce weight or increase motor/prop size
4. Use fresher battery (>50%)

**QuickTune Aborts with "Attitude Error"**:

**Causes**:
- `QUIK_DOUBLE_TIME` too short (tuning too fast)
- `QUIK_ANGLE_MAX` too low
- Starting tune too far from optimal
- Mechanical issues

**Solutions**:
1. Increase `QUIK_DOUBLE_TIME` to 12-15 seconds
2. Increase `QUIK_ANGLE_MAX` to 15-20 degrees
3. Manually improve starting tune
4. Check for loose components, bent arms

**AutoTune Results in Oscillations**:

**Solutions**:
1. Reject gains and manually reduce by 20%
2. Re-run AutoTune with lower `AUTOTUNE_AGGR`
3. Check for vibration or mechanical issues
4. Verify props balanced and motors healthy

### Diagnostic Logs

**Key Log Messages**:

| Message | Meaning | Action |
|---------|---------|--------|
| `AutoTune: Success` | AutoTune completed successfully | Accept or reject gains |
| `AutoTune: angle error ABORTING` | Angle error exceeded limits | Improve starting tune, reduce wind |
| `Quicktune: Roll complete` | QuickTune finished roll axis | Continue to next axis |
| `Quicktune: attitude error ABORTING` | Angle error excessive | Increase QUIK_DOUBLE_TIME or ANGLE_MAX |
| `Tuning: <param> = <value>` | Transmitter tuning active | Current parameter being adjusted |

**Log Analysis Tools**:
- **MAVExplorer**: Interactive Python log analysis
- **Plot Juggler**: Time-series visualization
- **UAV Log Viewer**: Web-based log analysis
- **Mission Planner**: Built-in graphing and FFT analysis

**Key Log Fields for Tuning**:
- `ATT`: Attitude (roll, pitch, yaw angles)
- `RATE`: Rate controller desired vs actual
- `PIDR/PIDP/PIDY`: PID loop internals (P/I/D/FF terms)
- `IMU`: Gyro and accelerometer data
- `VIBE`: Vibration levels
- `CTUN`: Control tuning (throttle, climb rate)

---

## Vehicle-Specific Considerations

### Frame Size Effects

**Small Frames (≤250mm)**:

**Characteristics**:
- Fast dynamics
- Low inertia
- High prop speeds

**Tuning Recommendations**:
- Higher gains: P = 0.18-0.25
- Higher filter frequencies: `INS_GYRO_FILTER` = 60-80 Hz
- Lower D gains: D may approach zero
- Faster tuning: `QUIK_DOUBLE_TIME` = 6-8 seconds

**Medium Frames (450-550mm)**:

**Characteristics**:
- Moderate dynamics
- Balanced inertia
- Standard prop speeds

**Tuning Recommendations**:
- Default gains: P = 0.135, D = 0.0036
- Default filtering: `INS_GYRO_FILTER` = 40 Hz
- Standard AutoTune works well
- Follow default tuning procedures

**Large Frames (≥800mm)**:

**Characteristics**:
- Slow dynamics
- High inertia
- Low prop speeds

**Tuning Recommendations**:
- Lower gains: P = 0.08-0.12
- Lower filter frequencies: `INS_GYRO_FILTER` = 20-30 Hz
- Higher D gains: D = 0.005-0.010 for damping
- Slower tuning: `QUIK_DOUBLE_TIME` = 12-15 seconds
- Higher angle P: `ATC_ANG_*_P` = 5-7 for responsiveness

### Motor and Propeller Combinations

**High KV Motors (>2000 KV) with Small Props**:
- Fast response, low inertia
- Higher gains possible
- May need less D term
- Higher filtering to manage motor noise

**Low KV Motors (<1000 KV) with Large Props**:
- Slow response, high inertia
- Lower gains required
- More D term for damping
- Lower filtering appropriate

**Prop Balance Importance**:
- Unbalanced props cause vibration
- Vibration requires more filtering
- More filtering reduces performance
- Balance props before tuning

### Helicopter-Specific Tuning

**Traditional Helicopters** use different control approach:

**Source**: `libraries/AC_AutoTune/AC_AutoTune_Heli.cpp`

**Key Differences**:
1. **Feed-forward terms** dominant (source: tuning.cpp:127-137)
   - `TUNING_RATE_PITCH_FF` (52)
   - `TUNING_RATE_ROLL_FF` (53)
   - `TUNING_RATE_YAW_FF` (54)

2. **External gyro gain** (source: tuning.cpp:123-125)
   - `TUNING_HELI_EXTERNAL_GYRO` (13)
   - Controls tail gyro for yaw

3. **Frequency-based AutoTune** (source: AC_AutoTune_Heli.cpp:88-99)
   - Sweep testing from `AUTOTUNE_FRQ_MIN` to `AUTOTUNE_FRQ_MAX`
   - Identifies phase margins and gain margins
   - Typical range: 10-70 Hz

4. **Sequential tuning** (source: AC_AutoTune_Heli.cpp:82-85)
   - VFF (velocity feed-forward)
   - Rate D and P
   - Angle P
   - Max gain and tune check

**Helicopter Tuning Order**:
1. Set hover throttle (`H_COL_MID`)
2. Tune VFF gains
3. Tune rate P and D
4. Tune angle P
5. Refine for stability margins

### Asymmetric Vehicles

**Vehicles with unequal roll/pitch characteristics**:

**Examples**:
- Tricopters (yaw via servo)
- Hexacopters with unusual layouts
- Payload-carrying vehicles
- Long-arm camera platforms

**Tuning Approach**:
1. **Tune axes independently**:
   - Use transmitter tuning function 49-51 (roll only)
   - Use transmitter tuning function 46-48 (pitch only)
   - Use QuickTune (handles asymmetry well)

2. **Different gains per axis**:
   - Roll: `ATC_RAT_RLL_P` may differ from pitch
   - Pitch: `ATC_RAT_PIT_P` may differ from roll
   - Acceptable to have 20-30% difference

3. **AutoTune limitations**:
   - AutoTune applies same gains to roll and pitch
   - May not be optimal for asymmetric frames
   - Consider manual tuning or QuickTune instead

### Coaxial and Unique Configurations

**Coaxial Rotors**:
- Each axis tunes independently
- May require significantly different yaw gains
- Test yaw authority separately

**Y6, X8 Configurations**:
- Stacked motors affect airflow
- May need more aggressive D term
- Check for motor-to-motor interference

**Bicopters/Tricopters**:
- Yaw via servo tilt rather than motor differential
- Servo rate limits affect yaw tuning
- Tune yaw last, more conservatively

### Weight and Payload Effects

**Adding Payload**:
- Increases inertia (slower dynamics)
- May require reduced gains (10-20%)
- Test with actual payload weight
- Consider separate parameter sets for different payloads

**Battery Size Changes**:
- Heavier battery increases inertia
- Affects hover throttle and response
- Re-tune if battery weight changes >20%

**Center of Gravity**:
- CG shifts affect pitch/roll balance
- May need asymmetric gains if CG off-center
- Strive for balanced CG for best performance

---

## Summary and Best Practices

### Tuning Workflow Summary

**For New Vehicles**:
```
1. Complete pre-flight validation (hardware, software)
2. Perform initial hover test (manual Stabilize)
3. If flyable:
   → Run AutoTune (10-20 minutes)
   → Accept gains if performance good
   → Or use QuickTune for faster tuning (1-3 minutes)
4. If not flyable:
   → Manually tune Rate P to flyable state
   → Then run AutoTune or QuickTune
5. Fine-tune with transmitter tuning if needed
6. Optimize filters based on log analysis
```

**For Existing Vehicles**:
```
1. Backup current parameters
2. Test current performance
3. If minor adjustment needed:
   → Use transmitter tuning for real-time adjustment
   → Or use QuickTune for quick optimization
4. If major adjustment needed:
   → Run full AutoTune
5. Verify in all intended flight modes
```

### Golden Rules

1. ✅ **Always tune rate controllers before angle controllers**
2. ✅ **Fix mechanical issues before tuning** (balance props, tighten screws)
3. ✅ **Tune in calm conditions** (wind <5 m/s)
4. ✅ **Make one change at a time** when manually tuning
5. ✅ **Save parameter backups** before major changes
6. ✅ **Test thoroughly** after tuning (all modes, various maneuvers)
7. ✅ **Monitor logs** for vibration and control saturation
8. ✅ **Maintain safety margins** (don't tune to oscillation point)

### Common Mistakes to Avoid

1. ❌ Tuning angle P before rate PID
2. ❌ Ignoring mechanical vibration issues
3. ❌ Over-filtering to compensate for vibration
4. ❌ Running AutoTune with low battery
5. ❌ Tuning in windy conditions
6. ❌ Not backing up parameters before tuning
7. ❌ Accepting oscillating tune results
8. ❌ Skipping post-tune verification flights

### Safety Reminders

⚠️ **Critical Safety Points**:
- Maintain altitude >5m during tuning
- Keep kill switch readily accessible
- Never tune near people or obstacles
- Monitor battery level continuously
- Stop tuning at first sign of problems
- Have plan for emergency landing
- Test new tunes conservatively first
- Always perform multiple test flights after tuning

### Performance Optimization Checklist

After successful tuning:

- [ ] Test hover stability (1-2 minutes hands-off)
- [ ] Test slow maneuvers (gentle roll/pitch/yaw)
- [ ] Test fast maneuvers (sport flying)
- [ ] Test altitude hold (stable hover at altitude)
- [ ] Test loiter (stable position hold)
- [ ] Test waypoint navigation (if applicable)
- [ ] Check motor temperatures (should be warm, not hot)
- [ ] Review logs for signs of issues
- [ ] Save parameters to file backup
- [ ] Document final parameter values

---

## Additional Resources

### Log Analysis
- **MAVExplorer**: `Tools/autotest/mavexplorer.py`
- **Plot Juggler**: Time-series visualization tool
- **Web-based log viewer**: https://logs.px4.io / APM Planner

### Parameter Documentation
- Full parameter list: via GCS parameter editor
- Parameter descriptions: In-code `@Param` tags (source: `ArduCopter/Parameters.cpp`)

### Source Code References

Key files for tuning implementation:
- **Transmitter tuning**: `ArduCopter/tuning.cpp`
- **AutoTune mode**: `ArduCopter/mode_autotune.cpp`
- **QuickTune**: `libraries/AP_Quicktune/AP_Quicktune.cpp`
- **Rate controllers**: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.h`
- **Helicopter AutoTune**: `libraries/AC_AutoTune/AC_AutoTune_Heli.cpp`
- **Parameter definitions**: `ArduCopter/Parameters.cpp`, `ArduCopter/Parameters.h`
- **Tuning enums**: `ArduCopter/defines.h` (tuning_func enum)

### Community Resources
- ArduPilot Discourse forum: https://discuss.ardupilot.org
- ArduPilot Wiki: https://ardupilot.org/copter/
- Discord community chat
- YouTube tutorials and guides

---

**Document Version**: 1.0  
**Last Updated**: Based on ArduPilot source code analysis  
**Applicable Versions**: ArduCopter 4.x and later  
**Contributors**: Generated from source code in ArduCopter/ and libraries/

---

*This tuning guide is based on comprehensive analysis of ArduPilot source code, including `tuning.cpp`, `mode_autotune.cpp`, `AC_AttitudeControl_Multi.h`, `AP_Quicktune.cpp`, and related files. All parameter defaults, ranges, and implementation details are sourced directly from the codebase.*
