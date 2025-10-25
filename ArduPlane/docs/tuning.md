# ArduPlane Tuning Guide

## Table of Contents
- [Overview](#overview)
- [Safety Considerations](#safety-considerations)
- [Tuning Preparation](#tuning-preparation)
- [Fixed-Wing PID Tuning](#fixed-wing-pid-tuning)
  - [Roll Controller Tuning](#roll-controller-tuning)
  - [Pitch Controller Tuning](#pitch-controller-tuning)
  - [Yaw Controller Tuning](#yaw-controller-tuning)
- [TECS Tuning](#tecs-tuning)
- [QuadPlane/VTOL Tuning](#quadplanevtol-tuning)
  - [Hover Mode PID Tuning](#hover-mode-pid-tuning)
  - [Position Control Tuning](#position-control-tuning)
- [Automated Tuning Methods](#automated-tuning-methods)
  - [AUTOTUNE Mode](#autotune-mode)
  - [Quicktune System](#quicktune-system)
  - [VTOL Quicktune Script](#vtol-quicktune-script)
- [Manual Tuning Methodology](#manual-tuning-methodology)
- [Transmitter-Based In-Flight Tuning](#transmitter-based-in-flight-tuning)
- [Parameter Tables and Recommended Ranges](#parameter-tables-and-recommended-ranges)
- [Tuning for Different Aircraft Types](#tuning-for-different-aircraft-types)
- [Troubleshooting](#troubleshooting)

## Overview

This guide provides comprehensive tuning procedures for ArduPlane autopilot systems, covering both traditional fixed-wing aircraft and QuadPlane/VTOL configurations. Proper tuning is essential for achieving stable, responsive flight characteristics while maintaining safety margins.

**Source References:**
- Fixed-wing tuning: `/ArduPlane/tuning.cpp`, `/ArduPlane/tuning.h`
- Autotune implementation: `/libraries/APM_Control/AP_AutoTune.cpp`
- Quicktune system: `/libraries/AP_Quicktune/AP_Quicktune.cpp`
- VTOL Quicktune: `/libraries/AP_Scripting/applets/VTOL-quicktune.lua`

### What is PID Tuning?

PID (Proportional-Integral-Derivative) controllers form the foundation of ArduPlane's control system. Each component serves a specific purpose:

- **P (Proportional)**: Generates control output proportional to the error. Higher P values result in stronger response but can cause oscillations if too high.
- **I (Integral)**: Eliminates steady-state error by accumulating past errors. Helps maintain desired setpoints against disturbances like wind.
- **D (Derivative)**: Dampens response based on rate of change. Reduces overshoot and oscillations but can amplify noise.
- **FF (Feed-Forward)**: Provides control output based on desired rate/angle without waiting for error. Improves response time and tracking.

### Tuning Goals

The objective of tuning is to achieve:

1. **Stability**: Aircraft maintains controlled flight without oscillations
2. **Responsiveness**: Quick, accurate response to pilot inputs and navigation commands
3. **Disturbance Rejection**: Maintains attitude and trajectory despite wind and turbulence
4. **Efficiency**: Minimal control surface deflection and energy consumption
5. **Safety Margins**: Maintains stability across the flight envelope

## Safety Considerations

> **CRITICAL**: Always perform initial tuning in calm weather conditions with a safety pilot ready to take manual control.

### Pre-Tuning Safety Checklist

- [ ] Verify aircraft is mechanically sound with secure components
- [ ] Check center of gravity is within manufacturer specifications
- [ ] Confirm control surfaces move in correct directions with adequate throw
- [ ] Test manual control and practice recovery procedures
- [ ] Start with conservative (low) gain values
- [ ] Fly in open area away from obstacles and people
- [ ] Monitor battery levels - allow sufficient power for tuning and safe landing
- [ ] Ensure ground station telemetry link is reliable
- [ ] Brief all personnel on emergency procedures

### Warning Signs During Tuning

Immediately revert gains and land if you observe:

- **Rapid oscillations** in any axis (sign of excessive P or D gain)
- **Growing oscillations** over time (sign of excessive I gain or insufficient D)
- **Sluggish response** followed by overshoot (insufficient P or FF)
- **Attitude drift** or inability to maintain level flight
- **Excessive control surface activity** ("hunting")
- **Uncommanded banking or pitching**

## Tuning Preparation

### Required Equipment

- Ground control station (Mission Planner, QGroundControl, or MAVProxy)
- Telemetry link with sufficient range and reliability
- Flight data logging enabled (automatic in most configurations)
- Optional: Real-time graphing capability for PID analysis
- Optional: Transmitter with auxiliary channels for in-flight tuning

### Initial Parameter Setup

Before beginning tuning, ensure these parameters are properly configured:

**Control Surface Setup:**
```
SERVO_OUTPUT_MIN/MAX/TRIM - Verify correct PWM ranges
RCn_MIN/MAX/TRIM - Calibrate RC inputs
MIXING_GAIN - Set to 0.5 initially
```

**Rate Limits:**
```
RLL_RATE_MAX - Maximum roll rate (default: 140 deg/s)
PTCH_RATE_MAX - Maximum pitch rate (default: 100 deg/s)  
YAW_RATE_MAX - Maximum yaw rate (default: 60 deg/s)
```

**Attitude Limits:**
```
LIM_ROLL_CD - Roll angle limit (default: 4500 = 45 degrees)
LIM_PITCH_MAX - Maximum pitch up (default: 2000 = 20 degrees)
LIM_PITCH_MIN - Maximum pitch down (default: -2500 = -25 degrees)
```

### Baseline Configuration

Start with conservative default values from the parameter table (see [Parameter Tables](#parameter-tables-and-recommended-ranges)). These provide a stable starting point for most aircraft.

### Test Flight Procedures

1. **Pre-tune baseline flight**: Verify basic stability in FBWA mode
2. **Record initial performance**: Note response characteristics and handling
3. **Incremental changes**: Adjust one parameter at a time, in small increments
4. **Test after each change**: Verify improvement without introducing instability
5. **Document results**: Keep detailed notes of parameter changes and results

## Fixed-Wing PID Tuning

Fixed-wing aircraft in ArduPlane use dedicated roll, pitch, and yaw controllers. The tuning parameters are accessed through the `rollController`, `pitchController`, and `yawController` objects.

**Source**: `/ArduPlane/tuning.cpp` lines 206-228

### Roll Controller Tuning

The roll controller manages bank angle and roll rate. It is typically the first axis to tune as it has the most direct control authority.

#### Roll Controller Parameters

| Parameter | Description | Typical Range | Units |
|-----------|-------------|---------------|-------|
| `RLL_P` | Roll rate proportional gain | 0.3 - 1.5 | - |
| `RLL_I` | Roll rate integral gain | 0.01 - 0.3 | - |
| `RLL_D` | Roll rate derivative gain | 0.01 - 0.1 | - |
| `RLL_FF` | Roll rate feed-forward | 0.2 - 0.8 | - |
| `RLL_IMAX` | Roll integrator maximum | 20 - 45 | deg |
| `RLL_RATE_MAX` | Maximum roll rate | 60 - 180 | deg/s |

**Parameter Access**: Via `AP_Tuning_Plane` tuning enums `TUNING_RLL_P`, `TUNING_RLL_I`, `TUNING_RLL_D`, `TUNING_RLL_FF`

#### Roll Tuning Procedure

1. **Set Initial Values**:
   ```
   RLL_P = 0.4
   RLL_I = 0.04
   RLL_D = 0.02
   RLL_FF = 0.4
   RLL_IMAX = 30
   ```

2. **Test Roll Response**:
   - In FBWA mode, command sharp roll inputs
   - Observe response on attitude graph
   - Look for clean, damped response without oscillation

3. **Tune P Gain**:
   - If response is sluggish: Increase `RLL_P` by 20%
   - If oscillations occur: Decrease `RLL_P` by 30%
   - Repeat until crisp response without overshoot

4. **Tune D Gain**:
   - Increase `RLL_D` to dampen any remaining oscillations
   - Typical value is 5-15% of P gain
   - Too much D causes "nervous" control or amplifies noise

5. **Tune FF Gain**:
   - `RLL_FF` should be ratio of control surface deflection to achieved rate
   - Start at 0.4 and adjust for tracking performance
   - Higher FF improves tracking but may cause initial overshoot

6. **Tune I Gain**:
   - Set `RLL_I` to approximately 10% of P gain (or use `AUTOTUNE_I_RATIO = 0.75` relationship)
   - I gain removes steady-state error from wind/trim
   - Too much I causes slow oscillations or "bounce back"

7. **Verify IMAX**:
   - `RLL_IMAX` limits integrator authority
   - Set to maximum bank angle in degrees (e.g., 30-45)
   - Monitor integrator in logs to ensure it doesn't saturate

### Pitch Controller Tuning

The pitch controller manages pitch angle and pitch rate. Pitch control is often more sensitive than roll due to weight distribution and aerodynamic characteristics.

#### Pitch Controller Parameters

| Parameter | Description | Typical Range | Units |
|-----------|-------------|---------------|-------|
| `PTCH_P` | Pitch rate proportional gain | 0.5 - 2.0 | - |
| `PTCH_I` | Pitch rate integral gain | 0.02 - 0.4 | - |
| `PTCH_D` | Pitch rate derivative gain | 0.01 - 0.15 | - |
| `PTCH_FF` | Pitch rate feed-forward | 0.2 - 0.8 | - |
| `PTCH_IMAX` | Pitch integrator maximum | 20 - 45 | deg |
| `PTCH_RATE_MAX` | Maximum pitch rate | 40 - 120 | deg/s |

**Parameter Access**: Via tuning enums `TUNING_PIT_P`, `TUNING_PIT_I`, `TUNING_PIT_D`, `TUNING_PIT_FF`

#### Pitch Tuning Procedure

1. **Set Initial Values** (typically higher than roll):
   ```
   PTCH_P = 0.8
   PTCH_I = 0.08
   PTCH_D = 0.04
   PTCH_FF = 0.5
   PTCH_IMAX = 30
   ```

2. **Test Pitch Response**:
   - Command moderate pitch inputs in FBWA
   - Avoid aggressive pitch commands initially
   - Monitor for oscillations or sluggish response

3. **Tune P Gain**:
   - Increase `PTCH_P` in 10-20% increments
   - Pitch is often more sensitive than roll
   - Watch for porpoising (oscillation in pitch axis)

4. **Tune D Gain**:
   - `PTCH_D` typically 5-10% of P gain
   - Increase to dampen pitch oscillations
   - Essential for aircraft with large moment of inertia

5. **Tune FF Gain**:
   - Adjust `PTCH_FF` for improved pitch tracking
   - Test during climbs and descents
   - Should minimize attitude error during maneuvers

6. **Tune I Gain**:
   - `PTCH_I` maintains pitch trim against varying conditions
   - Critical for maintaining altitude in wind
   - Set to 10% of P gain as starting point

#### Pitch-Specific Considerations

- **Center of Gravity**: Forward CG requires higher pitch gains
- **Airspeed Variation**: Pitch response changes significantly with speed
- **Throttle Coupling**: Some aircraft pitch up/down with throttle changes
- **Elevator Authority**: Limited authority requires lower rate limits

### Yaw Controller Tuning

Yaw control in fixed-wing aircraft is primarily used for coordinated turns and sideslip correction. Many aircraft have minimal yaw authority and don't require aggressive yaw tuning.

#### Yaw Controller Parameters

| Parameter | Description | Typical Range | Units |
|-----------|-------------|---------------|-------|
| `YAW_P` | Yaw rate proportional gain (not directly available) | N/A | - |
| `YAW_I` | Yaw rate integral gain (not directly available) | N/A | - |
| `YAW_RATE_MAX` | Maximum yaw rate | 30 - 90 | deg/s |

> **Note**: Fixed-wing yaw control is less directly tunable than roll/pitch in standard ArduPlane. Yaw damping is often handled through coordinated turn logic rather than dedicated PID loops.

#### Yaw Tuning Guidelines

1. **Coordinated Turns**: Primary yaw control maintains coordinated flight
2. **Crosswind Correction**: Yaw helps maintain ground track in wind
3. **Low Priority**: Tune roll and pitch before attempting yaw optimization
4. **Conservative Rates**: Set `YAW_RATE_MAX` conservatively (30-60 deg/s)

## TECS Tuning

The Total Energy Control System (TECS) manages the coupled relationship between airspeed and altitude by controlling throttle and pitch angle. TECS tuning is essential for smooth autonomous flight.

### TECS Overview

TECS treats the aircraft as an energy management system:
- **Total Energy (TE)**: Sum of kinetic energy (speed) and potential energy (altitude)
- **Energy Balance**: Throttle controls total energy; pitch distributes it between speed and altitude
- **Coupling**: Speed and altitude changes are interdependent

### Key TECS Parameters

| Parameter | Description | Typical Range | Default |
|-----------|-------------|---------------|---------|
| `TECS_SPDWEIGHT` | Speed vs altitude priority | 0.5 - 2.0 | 1.0 |
| `TECS_TIME_CONST` | Response time constant | 3 - 10 | 5.0 |
| `TECS_THR_DAMP` | Throttle damping | 0.1 - 0.5 | 0.3 |
| `TECS_PTCH_DAMP` | Pitch damping | 0.1 - 0.5 | 0.2 |
| `TECS_INTEGRATOR` | Integrator gain | 0.0 - 0.3 | 0.1 |
| `TECS_VERT_ACC` | Vertical acceleration limit | 1.0 - 5.0 | 2.0 |

### TECS Tuning Procedure

1. **Set Time Constant**:
   - `TECS_TIME_CONST` = 5.0 for most aircraft
   - Smaller aircraft (< 2 kg): Use 3-4
   - Larger aircraft (> 10 kg): Use 6-8
   - Determines overall response speed

2. **Tune Speed Weight**:
   - `TECS_SPDWEIGHT = 1.0`: Equal priority
   - `TECS_SPDWEIGHT > 1.0`: Prioritize airspeed maintenance
   - `TECS_SPDWEIGHT < 1.0`: Prioritize altitude maintenance
   - Racing/aerobatic: 1.5-2.0
   - Mapping/survey: 0.5-0.8

3. **Set Damping Values**:
   - Start with `TECS_THR_DAMP = 0.3`
   - Start with `TECS_PTCH_DAMP = 0.2`
   - Increase if hunting/oscillations occur
   - Decrease if sluggish response

4. **Configure Vertical Acceleration**:
   - `TECS_VERT_ACC` limits pitch angle changes
   - Set based on aircraft performance
   - Higher values = more aggressive climbs/descents

5. **Test Flight Scenarios**:
   - Altitude changes in AUTO mode
   - Airspeed changes in AUTO mode
   - Transitions between waypoints
   - Wind penetration performance

### TECS Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Airspeed oscillations | `THR_DAMP` too low | Increase throttle damping |
| Altitude oscillations | `PTCH_DAMP` too low | Increase pitch damping |
| Slow altitude response | `TIME_CONST` too high | Reduce time constant |
| Speed drops in climbs | `SPDWEIGHT` too low | Increase speed weight |
| Altitude drops in turns | Insufficient lift | Check stall margins |

## QuadPlane/VTOL Tuning

QuadPlane configurations combine fixed-wing and multirotor control systems, requiring tuning for both hover and forward flight modes. VTOL tuning is more complex due to the additional degrees of freedom and transition dynamics.

**Source**: `/ArduPlane/qautotune.cpp`, `/libraries/AP_Quicktune/`

### VTOL Architecture Overview

QuadPlane aircraft use two separate control systems:
- **Multirotor Controllers**: Active in hover modes (QHOVER, QLOITER, QLAND, etc.)
- **Fixed-Wing Controllers**: Active in forward flight modes (FBWA, AUTO, CRUISE, etc.)
- **Transition Logic**: Manages handoff between control modes

### Hover Mode PID Tuning

Hover modes use multirotor-style cascaded PID controllers:
1. **Angle P Controllers**: Convert attitude errors to desired rates
2. **Rate PID Controllers**: Convert rate errors to motor outputs
3. **Position PID Controllers**: Convert position errors to attitude targets (in QLOITER)

#### Rate Controller Parameters (Inner Loop)

These are the most critical parameters for hover stability.

**Roll Rate PID:**

| Parameter | Description | Typical Range | Access Via |
|-----------|-------------|---------------|------------|
| `Q_A_RAT_RLL_P` | Roll rate P gain | 0.08 - 0.25 | TUNING_RATE_ROLL_P |
| `Q_A_RAT_RLL_I` | Roll rate I gain | 0.08 - 0.25 | TUNING_RATE_ROLL_I |
| `Q_A_RAT_RLL_D` | Roll rate D gain | 0.001 - 0.012 | TUNING_RATE_ROLL_D |
| `Q_A_RAT_RLL_FF` | Roll rate FF gain | 0.0 - 0.5 | TUNING_RATE_ROLL_FF |
| `Q_A_RAT_RLL_FLTD` | Roll rate D-term filter | 10 - 40 | Hz |
| `Q_A_RAT_RLL_FLTT` | Roll rate target filter | 10 - 40 | Hz |

**Pitch Rate PID:**

| Parameter | Description | Typical Range | Access Via |
|-----------|-------------|---------------|------------|
| `Q_A_RAT_PIT_P` | Pitch rate P gain | 0.08 - 0.25 | TUNING_RATE_PITCH_P |
| `Q_A_RAT_PIT_I` | Pitch rate I gain | 0.08 - 0.25 | TUNING_RATE_PITCH_I |
| `Q_A_RAT_PIT_D` | Pitch rate D gain | 0.001 - 0.012 | TUNING_RATE_PITCH_D |
| `Q_A_RAT_PIT_FF` | Pitch rate FF gain | 0.0 - 0.5 | TUNING_RATE_PITCH_FF |
| `Q_A_RAT_PIT_FLTD` | Pitch rate D-term filter | 10 - 40 | Hz |
| `Q_A_RAT_PIT_FLTT` | Pitch rate target filter | 10 - 40 | Hz |

**Yaw Rate PID:**

| Parameter | Description | Typical Range | Access Via |
|-----------|-------------|---------------|------------|
| `Q_A_RAT_YAW_P` | Yaw rate P gain | 0.15 - 0.5 | TUNING_RATE_YAW_P |
| `Q_A_RAT_YAW_I` | Yaw rate I gain | 0.01 - 0.12 | TUNING_RATE_YAW_I |
| `Q_A_RAT_YAW_D` | Yaw rate D gain | 0.0 - 0.01 | TUNING_RATE_YAW_D |
| `Q_A_RAT_YAW_FF` | Yaw rate FF gain | 0.0 - 0.5 | TUNING_RATE_YAW_FF |
| `Q_A_RAT_YAW_FLTT` | Yaw rate target filter | 2 - 20 | Hz |

**Source**: `/ArduPlane/tuning.cpp` lines 122-199 (get_param_pointer implementation)

#### Rate Controller Tuning Procedure

1. **Start with Conservative Gains**:
   ```
   Q_A_RAT_RLL_P = 0.135
   Q_A_RAT_RLL_I = 0.135
   Q_A_RAT_RLL_D = 0.0036
   Q_A_RAT_PIT_P = 0.135
   Q_A_RAT_PIT_I = 0.135
   Q_A_RAT_PIT_D = 0.0036
   Q_A_RAT_YAW_P = 0.18
   Q_A_RAT_YAW_I = 0.018
   ```

2. **Hover Test**:
   - Enter QHOVER or QLOITER mode
   - Hover at 3-5 meters altitude
   - Observe stability and control response
   - Look for oscillations or drift

3. **Tune Roll/Pitch Rate P**:
   - Increase P in 10-20% increments
   - Test after each change
   - Stop when small, rapid oscillations appear
   - Reduce by 30% for final value

4. **Tune Roll/Pitch Rate D**:
   - Increase D to dampen oscillations
   - Typical D is 5-10% of P value
   - Too much D causes "jittery" motors
   - Watch motor output traces

5. **Tune Roll/Pitch Rate I**:
   - Set I equal to P initially
   - Reduce if slow oscillations appear
   - I compensates for CG offset and external forces

6. **Tune Yaw Rate**:
   - Yaw is less coupled and more forgiving
   - Start with P = 0.18, I = 0.018
   - Adjust if yaw response is sluggish or overshoots

7. **Set Filters**:
   - `FLTD`: D-term notch filter (typically 20 Hz)
   - `FLTT`: Target filter (typically 20 Hz)
   - Lower if motor noise is visible in logs
   - Higher values give crisper response

#### Angle Controller Parameters (Outer Loop)

Angle controllers convert attitude errors to rate requests.

| Parameter | Description | Typical Range | Access Via |
|-----------|-------------|---------------|------------|
| `Q_A_ANG_RLL_P` | Roll angle P gain | 4.5 - 12.0 | TUNING_ANG_ROLL_P |
| `Q_A_ANG_PIT_P` | Pitch angle P gain | 4.5 - 12.0 | TUNING_ANG_PITCH_P |
| `Q_A_ANG_YAW_P` | Yaw angle P gain | 4.5 - 12.0 | TUNING_ANG_YAW_P |

**Source**: `/ArduPlane/tuning.cpp` lines 159-166

#### Angle Controller Tuning Procedure

1. **Set Initial Values**:
   ```
   Q_A_ANG_RLL_P = 4.5
   Q_A_ANG_PIT_P = 4.5
   Q_A_ANG_YAW_P = 4.5
   ```

2. **Test Angle Response**:
   - In QSTABILIZE mode, command roll/pitch inputs
   - Release sticks and observe return to level
   - Should return smoothly without overshoot

3. **Tune Roll/Pitch Angle P**:
   - Increase in steps of 0.5
   - Higher values = faster return to level
   - Too high causes oscillation or overshoot
   - Typical final values: 6.0 - 8.0

4. **Tune Yaw Angle P**:
   - Less critical than roll/pitch
   - Set equal to roll/pitch initially
   - Adjust based on heading hold performance

### Position Control Tuning

Position controllers operate in QLOITER and AUTO mode, converting position errors to attitude commands.

#### Position Controller Parameters

**Horizontal Position:**

| Parameter | Description | Typical Range | Access Via |
|-----------|-------------|---------------|------------|
| `Q_P_POSXY_P` | XY position P gain | 0.5 - 2.0 | TUNING_PXY_P |
| `Q_P_VELXY_P` | XY velocity P gain | 1.0 - 3.0 | TUNING_VXY_P |
| `Q_P_VELXY_I` | XY velocity I gain | 0.3 - 2.0 | TUNING_VXY_I |
| `Q_P_VELXY_D` | XY velocity D gain | 0.2 - 1.0 | - |

**Vertical Position:**

| Parameter | Description | Typical Range | Access Via |
|-----------|-------------|---------------|------------|
| `Q_P_POSZ_P` | Z position P gain | 0.5 - 3.0 | TUNING_PZ_P |
| `Q_P_VELZ_P` | Z velocity P gain | 3.0 - 8.0 | TUNING_VZ_P |
| `Q_P_ACCZ_P` | Z acceleration P gain | 0.3 - 1.5 | TUNING_AZ_P |
| `Q_P_ACCZ_I` | Z acceleration I gain | 0.5 - 3.0 | TUNING_AZ_I |
| `Q_P_ACCZ_D` | Z acceleration D gain | 0.0 - 0.4 | TUNING_AZ_D |

**Source**: `/ArduPlane/tuning.cpp` lines 168-190

#### Position Tuning Procedure

1. **Start with Default Values** (usually adequate for most aircraft)
2. **Test in QLOITER**: Hold position against light wind
3. **Adjust if Position Drift**: Increase `Q_P_POSXY_P` slightly
4. **Adjust if Oscillating**: Reduce velocity P or increase D
5. **Tune Altitude Hold**: Adjust `Q_P_ACCZ_P/I/D` for smooth altitude changes

## Automated Tuning Methods

ArduPlane provides several automated tuning systems that can significantly reduce the time and expertise required for achieving a good tune. These systems automatically adjust PID gains based on aircraft response.

### AUTOTUNE Mode

AUTOTUNE is ArduPlane's fixed-wing automatic tuning mode. It flies the aircraft through a series of maneuvers to characterize response and automatically calculates optimal PID gains.

**Source**: `/libraries/APM_Control/AP_AutoTune.cpp`, `/ArduPlane/mode_autotune.cpp`

#### AUTOTUNE Overview

The AUTOTUNE algorithm:
1. Operates in FBWA-like mode with user control
2. Monitors control surface deflections and achieved rates
3. Calculates feed-forward (FF) gains from actuator-to-rate ratios
4. Progressively increases P and D gains until oscillation detected
5. Backs off gains to provide safety margin
6. Automatically saves gains when complete

**Source**: `/libraries/APM_Control/AP_AutoTune.cpp` lines 67-89 (tuning_table)

#### AUTOTUNE Tuning Levels

AUTOTUNE uses pre-configured aggressiveness levels (1-11) that set initial time constant (tau) and maximum rate (rmax):

| Level | Tau | RMAX | Character | Best For |
|-------|-----|------|-----------|----------|
| 1 | 1.00 | 20 | Very Soft | Heavy trainers, beginner aircraft |
| 2 | 0.90 | 30 | Soft | General trainers |
| 3 | 0.80 | 40 | Moderate-Soft | Sport aircraft |
| 4 | 0.70 | 50 | Moderate | All-purpose aircraft |
| 5 | 0.60 | 60 | Moderate-Aggressive | Sport aircraft (default) |
| 6 | 0.50 | 75 | Aggressive | Aerobatic trainers |
| 7 | 0.30 | 90 | Very Aggressive | 3D aircraft |
| 8 | 0.20 | 120 | Competition | Racing aircraft |
| 9 | 0.15 | 160 | Extreme | Expert racing |
| 10 | 0.10 | 210 | Maximum | Competition racing |
| 11 | 0.10 | 300 | Beyond Maximum | Special use cases |

**Source**: `/libraries/APM_Control/AP_AutoTune.cpp` lines 74-88

#### AUTOTUNE Parameters

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `AUTOTUNE_LEVEL` | Aggressiveness level | 6 | 1-11 |
| `AUTOTUNE_AXES` | Axes to tune (bitmask) | 7 (all) | 1:Roll, 2:Pitch, 4:Yaw |

#### AUTOTUNE Procedure

1. **Pre-Flight Preparation**:
   - Ensure aircraft has basic stable tune
   - Configure `AUTOTUNE_LEVEL` based on aircraft type
   - Set `AUTOTUNE_AXES` if you only want to tune specific axes
   - Verify sufficient flight space and altitude (minimum 100m AGL recommended)
   - Plan for 10-15 minutes of flight time
   - Calm wind conditions (< 10 mph) are ideal

2. **Enter AUTOTUNE Mode**:
   - Take off and climb to safe altitude
   - Switch to AUTOTUNE mode via RC or GCS
   - Aircraft behaves like FBWA mode

3. **Execute Tuning Maneuvers**:
   - Fly moderate-amplitude stick inputs (50-70% deflection)
   - Make smooth, deliberate roll and pitch inputs
   - Hold inputs for 1-2 seconds, then center
   - Perform 8-12 roll maneuvers in each direction
   - Perform 8-12 pitch maneuvers in each direction
   - Avoid violent or rapid stick movements

4. **Monitor Progress**:
   - Watch GCS messages for tuning status
   - Messages indicate which axis is being tuned
   - System reports when D limit found: "RollD: 0.0234"
   - System reports when P limit found: "RollP: 0.456"
   - Both limits must be found for successful tune

5. **Complete and Save**:
   - After limits found, continue flying 1-2 minutes for refinement
   - Switch out of AUTOTUNE to save gains
   - Gains are automatically saved to parameters
   - Test new gains in FBWA mode
   - If unsatisfactory, revert via parameter restore

**Source**: `/libraries/APM_Control/AP_AutoTune.cpp` lines 175-400 (update logic)

#### AUTOTUNE Algorithm Details

The AUTOTUNE algorithm operates through state machine:

**States** (from source code):
- `IDLE`: Waiting for pilot input
- `DEMAND_POS`: Positive rate demand detected
- `DEMAND_NEG`: Negative rate demand detected

**Actions** (from source code):
- Monitors actuator deflection vs achieved rate
- Calculates FF = actuator / (rate * scaler)
- Applies median filtering to FF estimate (5-sample window)
- Increases P and D gains incrementally
- Detects oscillations via Dmod (slew rate limiter saturation)
- When oscillation detected, reduces gains by safety factor
- Sets D_limit first, then works on P_limit

**Oscillation Detection**:
- Monitors slew rate limiter (from PID controller)
- When slew limiting occurs (Dmod < 1.0), oscillation detected
- Reduces D by 70% first time, then P by 65%
- Provides conservative final gains

**Source**: `/libraries/APM_Control/AP_AutoTune.cpp` lines 234-400

#### AUTOTUNE Best Practices

- **Good CG**: Ensure proper center of gravity before tuning
- **Calm Conditions**: Wind introduces errors in FF calculation
- **Smooth Inputs**: Jerky inputs confuse the algorithm
- **Sufficient Events**: Need 8+ events per axis for good FF estimate
- **Altitude Buffer**: Maintain altitude >50m during aggressive maneuvers
- **One Axis at Time**: Set `AUTOTUNE_AXES` to focus on problematic axis
- **Level Selection**: Start with level 4-5, increase if aircraft can handle it

#### AUTOTUNE Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Low rate" messages | Insufficient control authority | Increase control surface throws |
| Never finds D limit | D too low to cause oscillation | Manually set higher starting D |
| Oscillates after tune | Safety margins too small | Reduce AUTOTUNE_LEVEL |
| Sluggish after tune | Level too conservative | Increase AUTOTUNE_LEVEL |
| Inconsistent results | Wind or turbulence | Wait for calmer conditions |

### Quicktune System

Quicktune is a real-time, in-flight tuning system that progressively increases gains until oscillations are detected, then backs off to a safe margin. Unlike AUTOTUNE, it works continuously without requiring specific maneuvers.

**Source**: `/libraries/AP_Quicktune/AP_Quicktune.cpp`

#### Quicktune Overview

Quicktune operates by:
1. Incrementally increasing PID gains during flight
2. Monitoring gyro data for oscillations (high slew rates)
3. When oscillation detected, reducing gain by configured margin
4. Automatically computing I gains from P gains using configured ratio
5. Optionally adjusting filters based on INS_GYRO_FILTER

**Supported Platforms**:
- Multirotor (ArduCopter)
- QuadPlane hover modes (ArduPlane)
- Ground vehicles (Rover)

#### Quicktune Parameters

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `QWIK_ENABLE` | Enable quicktune system | 0 | 0:Disabled, 1:Enabled |
| `QWIK_AXES` | Axes bitmask | 7 | 0:Roll, 1:Pitch, 2:Yaw |
| `QWIK_DOUBLE_TIME` | Time to double gains | 10 | 5-20 seconds |
| `QWIK_GAIN_MARGIN` | Reduction after oscillation | 60 | 20-80 % |
| `QWIK_OSC_SMAX` | Oscillation detection threshold | 4 | 1-10 |
| `QWIK_YAW_P_MAX` | Maximum yaw P gain | 0.5 | 0.1-3.0 |
| `QWIK_YAW_D_MAX` | Maximum yaw D gain | 0.01 | 0.001-1.0 |
| `QWIK_RP_PI_RATIO` | Roll/Pitch P to I ratio | 1.0 | 1.0-2.0 |
| `QWIK_Y_PI_RATIO` | Yaw P to I ratio | 10 | 1.0-20 |
| `QWIK_AUTO_FILTER` | Auto-configure filters | 1 | 0:Disabled, 1:Enabled |
| `QWIK_AUTO_SAVE` | Auto-save delay (0=disabled) | 0 | 0-120 seconds |
| `QWIK_REDUCE_MAX` | Maximum gain reduction | 20 | 0-100 % |
| `QWIK_OPTIONS` | Options bitmask | 0 | 0:TwoPositionSwitch |
| `QWIK_ANGLE_MAX` | Maximum angle error for abort | 10 | degrees |

**Source**: `/libraries/AP_Quicktune/AP_Quicktune.cpp` lines 28-130

#### Quicktune Procedure

1. **Setup**:
   ```
   QWIK_ENABLE = 1
   QWIK_AXES = 7          (tune all axes)
   QWIK_DOUBLE_TIME = 10  (10 seconds doubling time)
   QWIK_GAIN_MARGIN = 60  (40% margin after oscillation)
   QWIK_AUTO_FILTER = 1   (auto-configure filters)
   ```

2. **Configure RC Switch** (three-position recommended):
   - Set RCx_OPTION = 163 (Quicktune)
   - Low position: Disabled/Revert
   - Mid position: Active tuning
   - High position: Save and complete

3. **Alternative Two-Position Setup**:
   ```
   QWIK_OPTIONS = 1       (two-position switch mode)
   QWIK_AUTO_SAVE = 30    (auto-save after 30 seconds)
   ```
   - Low: Disabled
   - High: Active (saves automatically after delay)

4. **Flight Procedure**:
   - Take off and enter QLOITER or QHOVER
   - Ensure stable hover at 5-10m altitude
   - Switch quicktune to Mid position
   - System announces "Quicktune: starting tune"
   - Fly moderate maneuvers (roll, pitch, yaw inputs)
   - Gains increase progressively over time
   - When oscillation detected, gain is automatically reduced
   - Message: "Quicktune: Roll: P:0.123 D:0.0045"

5. **Monitor Progress**:
   - Watch for oscillations or unusual behavior
   - System aborts if attitude error exceeds `QWIK_ANGLE_MAX`
   - Continue flying 1-2 minutes after all axes report complete
   - Each axis completes independently

6. **Save Results**:
   - Switch to High position to save parameters
   - Or let `QWIK_AUTO_SAVE` save automatically
   - Or switch to Low to revert changes
   - Saved parameters persist across reboots

**Source**: `/libraries/AP_Quicktune/AP_Quicktune.cpp` lines 134-200

#### Quicktune Algorithm

**Gain Increase Rate**:
```
gain_multiplier = 2^(elapsed_time / QWIK_DOUBLE_TIME)
```
Gains double every `QWIK_DOUBLE_TIME` seconds.

**Oscillation Detection**:
- Monitors gyro slew rate (derivative of angular rate)
- When slew rate exceeds `QWIK_OSC_SMAX`, oscillation detected
- Immediately stops increasing gains
- Reduces gain by `QWIK_GAIN_MARGIN` percentage

**I Gain Calculation**:
```
I_gain = P_gain / RP_PI_RATIO    (for roll/pitch)
I_gain = P_gain / Y_PI_RATIO     (for yaw)
```

**Filter Auto-Configuration** (when `QWIK_AUTO_FILTER = 1`):
```
FLTD = INS_GYRO_FILTER * 0.5
FLTT = INS_GYRO_FILTER * 0.5
```

#### Quicktune Best Practices

- **Start Conservative**: Use `QWIK_DOUBLE_TIME = 15` for first tune
- **Good Initial Tune**: Quicktune works best starting from stable gains
- **Fly Actively**: System needs motion to detect oscillations
- **Hover Stability**: Maintain stable hover during tune
- **Calm Conditions**: Wind can trigger false oscillation detection
- **Safety Altitude**: Stay above 10m in case of momentary oscillation
- **Save Frequently**: Switch can revert instantly if problems occur

#### Quicktune vs AUTOTUNE

| Feature | AUTOTUNE (Fixed-Wing) | Quicktune (VTOL) |
|---------|----------------------|------------------|
| Platform | Fixed-wing only | Multirotor/QuadPlane |
| Maneuvers | Requires specific inputs | Any flight motion |
| Duration | 10-15 minutes | 3-5 minutes |
| User Involvement | Active piloting needed | Minimal input required |
| Axes | Roll, Pitch (Yaw limited) | Roll, Pitch, Yaw |
| Real-time | No (batch processing) | Yes (continuous) |

### VTOL Quicktune Script

The VTOL Quicktune Lua script provides an alternative quicktune implementation via scripting, with similar functionality to the built-in C++ Quicktune.

**Source**: `/libraries/AP_Scripting/applets/VTOL-quicktune.lua`

#### Script Overview

The Lua implementation:
- Provides same functionality as C++ Quicktune
- Useful for testing or customization
- Requires scripting support enabled
- Parameters use `QUIK_` prefix (vs `QWIK_` in C++)

#### Script Parameters

Parameters are identical to Quicktune but with `QUIK_` prefix:

```
QUIK_ENABLE = 1
QUIK_AXES = 7
QUIK_DOUBLE_TIME = 10
QUIK_GAIN_MARGIN = 60
QUIK_OSC_SMAX = 5
QUIK_YAW_P_MAX = 0.5
QUIK_YAW_D_MAX = 0.01
QUIK_RP_PI_RATIO = 1.0
QUIK_Y_PI_RATIO = 10
QUIK_AUTO_FILTER = 1
QUIK_AUTO_SAVE = 0
QUIK_RC_FUNC = 163  (RC function number)
```

**Source**: `/libraries/AP_Scripting/applets/VTOL-quicktune.lua` lines 44-150

#### Installing the Script

1. **Enable Scripting**:
   ```
   SCR_ENABLE = 1
   SCR_HEAP_SIZE = 100000  (or larger)
   ```

2. **Copy Script**:
   - Place `VTOL-quicktune.lua` in SD card `/scripts/` directory
   - Reboot autopilot

3. **Configure RC Switch**:
   - Set RCx_OPTION to function number (typically 300+)
   - Or use `QUIK_RC_FUNC` parameter

4. **Use Identical Procedure** to C++ Quicktune

#### Lua vs C++ Quicktune

| Feature | C++ Quicktune | Lua Script |
|---------|---------------|------------|
| Performance | Faster | Slightly slower |
| Memory | Lower overhead | Requires scripting heap |
| Customization | Requires rebuild | Easy to modify |
| Availability | Built-in (newer versions) | External script |
| Stability | Production tested | Experimental |

**Recommendation**: Use C++ Quicktune (`QWIK_`) if available in your firmware version.

## Manual Tuning Methodology

Manual tuning provides precise control over the tuning process and deeper understanding of aircraft dynamics. It's essential when automated methods don't produce satisfactory results or for specialized aircraft.

### Manual Tuning Philosophy

The manual approach follows proven aerospace engineering principles:

1. **Inner Loop First**: Tune rate controllers before angle controllers
2. **One Parameter at Time**: Isolate effects of each change
3. **Small Increments**: Make 10-20% adjustments, never double gains
4. **Test Thoroughly**: Fly 2-3 minutes after each change
5. **Document Everything**: Record all parameter values and observations
6. **Safety Margins**: Back off from oscillation threshold by 20-30%

### Step-by-Step Manual Tuning Process

#### Phase 1: Preparation

1. **Set Conservative Baseline**:
   ```
   RLL_P = 0.3
   RLL_I = 0.03
   RLL_D = 0.01
   RLL_FF = 0.3
   PTCH_P = 0.5
   PTCH_I = 0.05
   PTCH_D = 0.02
   PTCH_FF = 0.4
   ```

2. **Configure Logging**:
   - Enable full-rate logging: `LOG_BITMASK = 393854` (all)
   - Verify ATT, RATE, PIDP, PIDR logs enabled
   - Large SD card for multiple test flights

3. **Setup Monitoring**:
   - Real-time telemetry on ground station
   - Graph attitude and rates
   - Monitor control surface outputs

#### Phase 2: Roll Rate Controller

1. **Tune P Gain** (`RLL_P`):
   - Start at 0.3, increase to 0.4
   - Test with sharp aileron inputs in FBWA
   - Look for crisp, well-damped response
   - Increase until small oscillations appear
   - Reduce by 25% for safety margin
   - Typical final value: 0.6 - 1.2

2. **Tune D Gain** (`RLL_D`):
   - Start at 0.01, increase to 0.015
   - D dampens oscillations and adds phase lead
   - Set to 5-10% of P gain
   - Too much D causes high-frequency "buzz"
   - Typical final value: 0.04 - 0.12

3. **Tune FF Gain** (`RLL_FF`):
   - FF improves tracking of rate commands
   - Measure stick input vs achieved roll rate
   - FF ≈ (stick deflection %) / (roll rate deg/s)
   - Start at 0.3-0.4, adjust based on response
   - Higher FF = better tracking, possible initial spike
   - Typical final value: 0.4 - 0.7

4. **Tune I Gain** (`RLL_I`):
   - Set to 10% of P gain initially
   - Fly in wind and observe steady-state roll
   - Increase if aircraft drifts in bank angle
   - Decrease if slow oscillations develop
   - Typical final value: 0.05 - 0.2

5. **Verify IMAX** (`RLL_IMAX`):
   - Check integrator doesn't saturate in logs
   - Typical value: 30-45 degrees
   - Should match maximum desired bank angle

#### Phase 3: Pitch Rate Controller

Follow identical process for pitch:

1. `PTCH_P`: Start 0.5, tune to 0.8 - 1.5
2. `PTCH_D`: Typically 5-10% of P
3. `PTCH_FF`: Usually 0.4 - 0.6
4. `PTCH_I`: About 10% of P
5. `PTCH_IMAX`: 30-45 degrees

**Pitch Differences from Roll**:
- Pitch often requires higher gains due to stability
- More sensitive to CG position
- Airspeed effects are more pronounced
- Elevator authority varies with configuration

#### Phase 4: Flight Testing and Validation

1. **Slow Speed Test**:
   - Fly at minimum safe airspeed
   - Verify stable control
   - Check for mushy response or oscillations

2. **High Speed Test**:
   - Fly at maximum cruise speed
   - Verify no high-speed oscillations
   - Check control effectiveness

3. **Wind Penetration**:
   - Fly in 10-15 mph winds
   - Test upwind, downwind, crosswind
   - Verify I gain handles steady-state errors

4. **Aggressive Maneuvers**:
   - Steep turns (45-60 degrees bank)
   - Rapid roll reversals
   - Pitch transitions (climbs/dives)
   - Verify no overshoot or oscillation

5. **Autonomous Flight**:
   - Test in AUTO mode with waypoints
   - Observe navigation accuracy
   - Check altitude and speed tracking

### Log Analysis for Manual Tuning

Effective tuning requires analyzing flight logs to understand controller behavior.

#### Key Log Messages

| Log | Fields | Use |
|-----|--------|-----|
| `ATT` | Roll, Pitch, Yaw | Actual attitude |
| `RATE` | RDes, R, PDes, P | Desired and actual rates |
| `PIDP` | Des, P, I, D, FF, Dmod | Pitch controller internals |
| `PIDR` | Des, P, I, D, FF, Dmod | Roll controller internals |

#### Log Analysis Procedure

1. **Plot Desired vs Actual**:
   ```
   Graph: RATE.RDes vs RATE.R (roll rate)
   Graph: RATE.PDes vs RATE.P (pitch rate)
   ```
   - Should track closely with minimal lag
   - Oscillations indicate excessive gain
   - Lag indicates insufficient FF or P

2. **Check PID Components**:
   ```
   Graph: PIDR.P, PIDR.I, PIDR.D, PIDR.FF
   ```
   - P should dominate during maneuvers
   - I should be small, steady-state component
   - D should oppose rapid changes
   - FF should track desired rate

3. **Monitor Slew Limiter**:
   ```
   Graph: PIDR.Dmod
   ```
   - Dmod = 1.0: No limiting (good)
   - Dmod < 1.0: Slew limiting active (oscillation)
   - Frequent limiting indicates excessive P or D

4. **Integrator Saturation**:
   ```
   Graph: PIDR.I
   ```
   - Should stay well below IMAX
   - Frequent saturation indicates insufficient P or excessive I
   - Constant high I suggests trim issue

### Advanced Manual Tuning Techniques

#### Slew Rate Limiting

ArduPlane uses slew rate limiting to prevent control surface oscillations:

```
Slew Limit: Limits rate of change of control output
Purpose: Prevents actuator-induced oscillations
Parameter: Automatic in rate controllers
Effect: Dmod < 1.0 when limiting active
```

When tuning, avoid conditions that cause persistent slew limiting.

#### Feed-Forward Tuning

FF is the most impactful parameter for tracking performance:

**Optimal FF Calculation**:
```
FF_optimal = Actuator_Deflection / (Achieved_Rate * Scaler)
```

**Practical FF Tuning**:
1. Command constant rate (e.g., 60 deg/s roll)
2. Observe steady-state actuator position (e.g., 40%)
3. Calculate: FF = 0.40 / (60 * scaler)
4. Scaler accounts for airspeed and configuration

#### Rate Limit Optimization

Rate limits prevent excessive commanded rates:

```
RLL_RATE_MAX: Maximum roll rate (deg/s)
PTCH_RATE_MAX: Maximum pitch rate (deg/s)
```

**Setting Rate Limits**:
- Should match or slightly exceed aircraft capability
- Too high: Impossible rate commands, poor tracking
- Too low: Sluggish response, limited maneuverability
- Test: Command full stick, observe maximum achieved rate
- Set limit 10-20% above maximum observed rate

## Transmitter-Based In-Flight Tuning

ArduPlane supports in-flight parameter adjustment via RC transmitter, enabling real-time tuning without landing.

**Source**: `/ArduPlane/tuning.cpp`, `/ArduPlane/tuning.h`

### Transmitter Tuning Overview

The system allows adjustment of individual parameters or parameter sets using:
- **RC Channel**: Dedicated channel controls parameter value
- **Tuning Knob**: Typically 3-position switch or potentiometer
- **Real-time Updates**: Changes applied immediately in flight
- **Parameter Saving**: Long press to save current value

### Available Tuning Parameters

**Source**: `/ArduPlane/tuning.cpp` lines 11-16 (TUNE_PARAM values)

#### Individual Parameters (Value < 50 = QuadPlane, Value >= 50 = Fixed-Wing)

**QuadPlane/VTOL Parameters:**
- `1`: RateRollPI (P and I together)
- `2`: RateRollP
- `3`: RateRollI
- `4`: RateRollD
- `5`: RatePitchPI
- `6`: RatePitchP
- `7`: RatePitchI
- `8`: RatePitchD
- `9`: RateYawPI
- `10`: RateYawP
- `11`: RateYawI
- `12`: RateYawD
- `13`: AngleRollP
- `14`: AnglePitchP
- `15`: AngleYawP
- `16`: PosXYP
- `17`: PosZP
- `18`: VelXYP
- `19`: VelXYI
- `20`: VelZP
- `21`: AccelZP
- `22`: AccelZI
- `23`: AccelZD
- `24`: RatePitchFF
- `25`: RateRollFF
- `26`: RateYawFF

**Fixed-Wing Parameters:**
- `50`: FixedWingRollP
- `51`: FixedWingRollI
- `52`: FixedWingRollD
- `53`: FixedWingRollFF
- `54`: FixedWingPitchP
- `55`: FixedWingPitchI
- `56`: FixedWingPitchD
- `57`: FixedWingPitchFF

#### Parameter Sets (Value > 100)

**Source**: `/ArduPlane/tuning.cpp` lines 29-62

- `101`: Set_RateRollPitch (Roll D, Roll PI, Pitch D, Pitch PI)
- `102`: Set_RateRoll (Roll D, Roll PI)
- `103`: Set_RatePitch (Pitch D, Pitch PI)
- `104`: Set_RateYaw (Yaw P, I, D)
- `105`: Set_AngleRollPitch (Angle Roll P, Angle Pitch P)
- `106`: Set_VelXY (Velocity XY P, I)
- `107`: Set_AccelZ (Accel Z P, I, D)
- `108`: Set_RatePitchDP (Pitch D, Pitch P)
- `109`: Set_RateRollDP (Roll D, Roll P)
- `110`: Set_RateYawDP (Yaw D, Yaw P)

### Transmitter Tuning Setup

#### Step 1: Configure Tuning Parameters

```
TUNE_PARAM = 102          # Parameter/set to tune (e.g., 102 = Roll rate set)
TUNE_MIN = 0.1            # Minimum value
TUNE_MAX = 2.0            # Maximum value  
TUNE_SELECTOR = 0         # 0 = use TUNE_PARAM
```

#### Step 2: Assign RC Channel

Configure transmitter channel (e.g., Channel 6) to control tuning:

```
TUNE_CHAN = 6             # RC channel for tuning input
```

Channel PWM maps to parameter value:
```
PWM 1000 = TUNE_MIN
PWM 1500 = (TUNE_MIN + TUNE_MAX) / 2
PWM 2000 = TUNE_MAX
```

#### Step 3: Optional - Parameter Saving

Set up a momentary switch for saving:

```
RC7_OPTION = 301          # Save tuning parameter
```

Or use 2-second stick hold to save automatically.

### In-Flight Tuning Procedure

1. **Pre-Flight**:
   - Set `TUNE_PARAM` to desired parameter
   - Configure `TUNE_MIN` and `TUNE_MAX` range
   - Verify tuning channel connected
   - Center tuning knob/slider

2. **Take Off**:
   - Perform normal takeoff
   - Climb to safe altitude (100m+ AGL)
   - Enter FBWA or tuning-compatible mode

3. **Adjust Parameter**:
   - Move tuning channel input
   - Observe immediate effect on aircraft behavior
   - Look for improved response or stability
   - Avoid sudden large changes

4. **Find Optimal Value**:
   - Increase gain until response becomes crisp
   - Watch for onset of oscillations
   - Reduce gain slightly for margin
   - Test in various maneuvers

5. **Save Value**:
   - Activate save switch (if configured)
   - Or use stick pattern to save
   - Verify save via GCS message
   - Parameter persists after reboot

6. **Move to Next Parameter**:
   - Land aircraft
   - Change `TUNE_PARAM` to next parameter
   - Adjust `TUNE_MIN`/`TUNE_MAX` if needed
   - Repeat process

### Transmitter Tuning Best Practices

- **Small Ranges**: Set `TUNE_MIN`/`TUNE_MAX` close together (±50% of current value)
- **One Parameter**: Focus on single most impactful parameter per flight
- **Safe Altitude**: Maintain altitude buffer for recovery from oscillations
- **Smooth Inputs**: Make gradual adjustments, observe effects
- **Document**: Record knob position and resulting parameter value
- **Save Frequently**: Save good values before trying more aggressive settings

### Tuning Sets vs Individual Parameters

**Individual Parameters**: Fine-tune specific gain
- Precise control over single parameter
- Useful for advanced tuning
- Requires understanding of PID theory

**Parameter Sets**: Tune related parameters together
- Faster than individual tuning
- Maintains relationships between P, I, D
- Good for initial tuning or field adjustments

**Recommended Sets**:
- `102` (RateRoll): Most common starting point
- `103` (RatePitch): After roll is tuned
- `108/109` (DP sets): Tune D and P simultaneously

## Parameter Tables and Recommended Ranges

This section provides comprehensive parameter tables with recommended starting values for different aircraft categories.

### Fixed-Wing Control Parameters

#### Roll Controller Parameters

| Parameter | Trainer | Sport | Aerobatic | Racing | Units | Description |
|-----------|---------|-------|-----------|--------|-------|-------------|
| `RLL_P` | 0.4 | 0.8 | 1.2 | 1.5 | - | Roll rate proportional gain |
| `RLL_I` | 0.04 | 0.08 | 0.12 | 0.15 | - | Roll rate integral gain |
| `RLL_D` | 0.02 | 0.04 | 0.06 | 0.08 | - | Roll rate derivative gain |
| `RLL_FF` | 0.3 | 0.5 | 0.6 | 0.7 | - | Roll rate feed-forward |
| `RLL_IMAX` | 30 | 35 | 40 | 45 | deg | Roll integrator limit |
| `RLL_RATE_MAX` | 60 | 120 | 180 | 240 | deg/s | Maximum roll rate |

#### Pitch Controller Parameters

| Parameter | Trainer | Sport | Aerobatic | Racing | Units | Description |
|-----------|---------|-------|-----------|--------|-------|-------------|
| `PTCH_P` | 0.6 | 1.0 | 1.5 | 2.0 | - | Pitch rate proportional gain |
| `PTCH_I` | 0.06 | 0.10 | 0.15 | 0.20 | - | Pitch rate integral gain |
| `PTCH_D` | 0.03 | 0.05 | 0.08 | 0.10 | - | Pitch rate derivative gain |
| `PTCH_FF` | 0.4 | 0.5 | 0.6 | 0.7 | - | Pitch rate feed-forward |
| `PTCH_IMAX` | 30 | 35 | 40 | 45 | deg | Pitch integrator limit |
| `PTCH_RATE_MAX` | 40 | 80 | 120 | 160 | deg/s | Maximum pitch rate |

#### TECS Parameters

| Parameter | Trainer | Sport | Mapping | Racing | Units | Description |
|-----------|---------|-------|---------|--------|-------|-------------|
| `TECS_TIME_CONST` | 6.0 | 5.0 | 5.0 | 4.0 | s | Time constant |
| `TECS_SPDWEIGHT` | 1.0 | 1.0 | 0.7 | 1.5 | - | Speed vs altitude priority |
| `TECS_THR_DAMP` | 0.3 | 0.3 | 0.4 | 0.2 | - | Throttle damping |
| `TECS_PTCH_DAMP` | 0.2 | 0.2 | 0.3 | 0.15 | - | Pitch damping |
| `TECS_INTEGRATOR` | 0.1 | 0.1 | 0.15 | 0.05 | - | Integrator gain |
| `TECS_VERT_ACC` | 2.0 | 3.0 | 2.0 | 4.0 | m/s² | Vertical acceleration limit |

### QuadPlane/VTOL Parameters

#### Rate Controller Parameters (Hover)

| Parameter | Heavy | Standard | Agile | Racing | Units | Description |
|-----------|-------|----------|-------|--------|-------|-------------|
| `Q_A_RAT_RLL_P` | 0.10 | 0.135 | 0.18 | 0.25 | - | Roll rate P |
| `Q_A_RAT_RLL_I` | 0.10 | 0.135 | 0.18 | 0.25 | - | Roll rate I |
| `Q_A_RAT_RLL_D` | 0.003 | 0.0036 | 0.005 | 0.008 | - | Roll rate D |
| `Q_A_RAT_RLL_FF` | 0.0 | 0.0 | 0.15 | 0.30 | - | Roll rate FF |
| `Q_A_RAT_PIT_P` | 0.10 | 0.135 | 0.18 | 0.25 | - | Pitch rate P |
| `Q_A_RAT_PIT_I` | 0.10 | 0.135 | 0.18 | 0.25 | - | Pitch rate I |
| `Q_A_RAT_PIT_D` | 0.003 | 0.0036 | 0.005 | 0.008 | - | Pitch rate D |
| `Q_A_RAT_PIT_FF` | 0.0 | 0.0 | 0.15 | 0.30 | - | Pitch rate FF |
| `Q_A_RAT_YAW_P` | 0.18 | 0.18 | 0.30 | 0.50 | - | Yaw rate P |
| `Q_A_RAT_YAW_I` | 0.018 | 0.018 | 0.030 | 0.050 | - | Yaw rate I |
| `Q_A_RAT_YAW_D` | 0.0 | 0.0 | 0.003 | 0.008 | - | Yaw rate D |

#### Angle Controller Parameters (Hover)

| Parameter | Heavy | Standard | Agile | Racing | Units | Description |
|-----------|-------|----------|-------|--------|-------|-------------|
| `Q_A_ANG_RLL_P` | 4.5 | 4.5 | 6.0 | 8.0 | - | Roll angle P |
| `Q_A_ANG_PIT_P` | 4.5 | 4.5 | 6.0 | 8.0 | - | Pitch angle P |
| `Q_A_ANG_YAW_P` | 4.5 | 4.5 | 6.0 | 8.0 | - | Yaw angle P |

#### Position Controller Parameters

| Parameter | Recommended | Range | Units | Description |
|-----------|-------------|-------|-------|-------------|
| `Q_P_POSXY_P` | 1.0 | 0.5-2.0 | - | XY position P |
| `Q_P_VELXY_P` | 2.0 | 1.0-3.0 | - | XY velocity P |
| `Q_P_VELXY_I` | 1.0 | 0.3-2.0 | - | XY velocity I |
| `Q_P_POSZ_P` | 1.0 | 0.5-3.0 | - | Z position P |
| `Q_P_VELZ_P` | 5.0 | 3.0-8.0 | - | Z velocity P |
| `Q_P_ACCZ_P` | 0.5 | 0.3-1.5 | - | Z acceleration P |
| `Q_P_ACCZ_I` | 1.0 | 0.5-3.0 | - | Z acceleration I |
| `Q_P_ACCZ_D` | 0.0 | 0.0-0.4 | - | Z acceleration D |

### Filter Parameters

#### Fixed-Wing Filters

Most fixed-wing controllers use slew rate limiting rather than explicit filters.

| Parameter | Typical Value | Range | Units | Description |
|-----------|---------------|-------|-------|-------------|
| `RLL_RATE_SMAX` | 150 | 50-500 | deg/s/s | Roll slew rate limit |
| `PTCH_RATE_SMAX` | 150 | 50-500 | deg/s/s | Pitch slew rate limit |

#### VTOL Filters

| Parameter | Typical Value | Range | Units | Description |
|-----------|---------------|-------|-------|-------------|
| `Q_A_RAT_RLL_FLTD` | 20 | 10-40 | Hz | Roll D-term filter |
| `Q_A_RAT_RLL_FLTT` | 20 | 10-40 | Hz | Roll target filter |
| `Q_A_RAT_PIT_FLTD` | 20 | 10-40 | Hz | Pitch D-term filter |
| `Q_A_RAT_PIT_FLTT` | 20 | 10-40 | Hz | Pitch target filter |
| `Q_A_RAT_YAW_FLTE` | 2 | 1-5 | Hz | Yaw error filter |
| `Q_A_RAT_YAW_FLTT` | 10 | 2-20 | Hz | Yaw target filter |

**Filter Selection Guidelines**:
- Lower filters (10-15 Hz): Noisy motors or flexible frames
- Higher filters (30-40 Hz): Stiff frames, high-quality motors
- Match to `INS_GYRO_FILTER` for consistency
- Use Quicktune auto-filter feature for automatic configuration

## Tuning for Different Aircraft Types

Aircraft characteristics significantly impact optimal tuning parameters. This section provides starting points for common aircraft categories.

### Small Trainers (< 2 kg, High-Wing, Mild Performance)

**Examples**: Bixler, Apprentice, Ranger

**Characteristics**:
- High stability, gentle flight characteristics
- Large control surfaces, high authority
- Forgiving of tuning errors
- Low airspeeds (8-15 m/s)

**Recommended Starting Parameters**:
```
RLL_P = 0.4
RLL_I = 0.04
RLL_D = 0.02
RLL_FF = 0.3
RLL_RATE_MAX = 60

PTCH_P = 0.6
PTCH_I = 0.06
PTCH_D = 0.03
PTCH_FF = 0.4
PTCH_RATE_MAX = 50

TECS_TIME_CONST = 6.0
TECS_SPDWEIGHT = 1.0

AUTOTUNE_LEVEL = 3
```

**Tuning Notes**:
- Very forgiving, easy to tune
- Focus on smooth, comfortable flight
- Conservative rate limits appropriate
- AUTOTUNE level 3-4 recommended

### Sport Aircraft (2-5 kg, Moderate Performance)

**Examples**: Bixler 2, Skywalker X8, Mini Talon

**Characteristics**:
- Balanced stability and performance
- Moderate control authority
- Efficient cruise, reasonable maneuverability
- Medium airspeeds (12-20 m/s)

**Recommended Starting Parameters**:
```
RLL_P = 0.7
RLL_I = 0.07
RLL_D = 0.04
RLL_FF = 0.5
RLL_RATE_MAX = 100

PTCH_P = 0.9
PTCH_I = 0.09
PTCH_D = 0.05
PTCH_FF = 0.5
PTCH_RATE_MAX = 70

TECS_TIME_CONST = 5.0
TECS_SPDWEIGHT = 1.0

AUTOTUNE_LEVEL = 5
```

**Tuning Notes**:
- Most common category
- Default values work well as starting point
- AUTOTUNE highly effective
- Balance efficiency and responsiveness

### Mapping/Survey Aircraft (Focus on Stability)

**Examples**: Skywalker X8, Finwing Penguin, Believer

**Characteristics**:
- Emphasis on stability over maneuverability
- Long endurance, high efficiency
- Payload (camera/sensors)
- Moderate airspeeds (15-22 m/s)

**Recommended Starting Parameters**:
```
RLL_P = 0.6
RLL_I = 0.08        # Higher I for wind rejection
RLL_D = 0.04
RLL_FF = 0.5
RLL_RATE_MAX = 80

PTCH_P = 0.8
PTCH_I = 0.10       # Higher I for altitude hold
PTCH_D = 0.05
PTCH_FF = 0.5
PTCH_RATE_MAX = 60

TECS_TIME_CONST = 5.0
TECS_SPDWEIGHT = 0.7    # Favor altitude over speed
TECS_THR_DAMP = 0.4     # Smoother throttle
TECS_PTCH_DAMP = 0.3    # Smoother pitch

AUTOTUNE_LEVEL = 4
```

**Tuning Notes**:
- Prioritize smooth, stable flight
- Higher I gains for disturbance rejection
- Lower TECS speed weight for altitude priority
- Conservative rate limits
- Test thoroughly in wind

### Aerobatic Aircraft (High Performance, 3D Capable)

**Examples**: 3D planes, pattern aircraft, unlimited aerobatic

**Characteristics**:
- High control authority
- Wide speed range (0-30+ m/s)
- Capable of extreme attitudes
- High power-to-weight ratio

**Recommended Starting Parameters**:
```
RLL_P = 1.2
RLL_I = 0.12
RLL_D = 0.06
RLL_FF = 0.65
RLL_RATE_MAX = 180

PTCH_P = 1.5
PTCH_I = 0.15
PTCH_D = 0.08
PTCH_FF = 0.65
PTCH_RATE_MAX = 120

TECS_TIME_CONST = 4.0
TECS_SPDWEIGHT = 1.5    # Favor speed control

AUTOTUNE_LEVEL = 7
```

**Tuning Notes**:
- High gains needed for crisp response
- FF critical for tracking performance
- Ensure CG is correct (typically well forward)
- Test across full speed envelope
- Rate limits should match capability

### Racing Aircraft (Maximum Performance)

**Examples**: Racing wings, high-speed FPV planes

**Characteristics**:
- Maximum agility
- Very high airspeeds (20-40+ m/s)
- Minimal stability margins
- Expert pilots only

**Recommended Starting Parameters**:
```
RLL_P = 1.5
RLL_I = 0.15
RLL_D = 0.08
RLL_FF = 0.7
RLL_RATE_MAX = 240

PTCH_P = 2.0
PTCH_I = 0.20
PTCH_D = 0.10
PTCH_FF = 0.7
PTCH_RATE_MAX = 160

TECS_TIME_CONST = 4.0
TECS_SPDWEIGHT = 2.0
TECS_VERT_ACC = 5.0

AUTOTUNE_LEVEL = 9
```

**Tuning Notes**:
- Aggressive gains throughout
- High FF for tracking
- Maximum rate limits
- TECS prioritizes speed maintenance
- Requires expert tuning skills
- Test incrementally

### Standard QuadPlane VTOL (Tilt-Rotor, Fixed Rotor)

**Examples**: FireFLY6, Convergence-style

**Characteristics**:
- Combines multirotor and fixed-wing
- Moderate hover performance
- Efficient cruise
- Complex transition dynamics

**Hover Mode Parameters**:
```
Q_A_RAT_RLL_P = 0.135
Q_A_RAT_RLL_I = 0.135
Q_A_RAT_RLL_D = 0.0036
Q_A_RAT_PIT_P = 0.135
Q_A_RAT_PIT_I = 0.135
Q_A_RAT_PIT_D = 0.0036
Q_A_RAT_YAW_P = 0.18
Q_A_RAT_YAW_I = 0.018

Q_A_ANG_RLL_P = 4.5
Q_A_ANG_PIT_P = 4.5
Q_A_ANG_YAW_P = 4.5
```

**Forward Flight Parameters**: Use Sport Aircraft values

**Tuning Notes**:
- Tune hover modes first using Quicktune
- Then tune fixed-wing modes using AUTOTUNE
- Test transitions thoroughly
- Verify transition airspeed is adequate
- Check Q_ASSIST parameters for emergency assist

### Tailsitter VTOL

**Examples**: Custom tailsitters, experimental designs

**Characteristics**:
- Vertical takeoff/landing
- No dedicated hover motors
- Complex transition requiring motor/control mixing
- Challenging to tune

**Recommended Approach**:
- Start with conservative multirotor gains
- Tune hover in QHOVER first
- Gradually increase gains using Quicktune
- Test transitions at altitude initially
- Verify control allocation is correct

### Large Aircraft (>10 kg, UAV platforms)

**Examples**: Large mapping platforms, cargo UAVs

**Characteristics**:
- High inertia
- Lower control surface authority
- Strong ground effect
- Slower response times

**Recommended Starting Parameters**:
```
RLL_P = 0.5
RLL_I = 0.08
RLL_D = 0.03
RLL_FF = 0.4
RLL_RATE_MAX = 40

PTCH_P = 0.7
PTCH_I = 0.10
PTCH_D = 0.04
PTCH_FF = 0.45
PTCH_RATE_MAX = 35

TECS_TIME_CONST = 7.0
TECS_THR_DAMP = 0.4
TECS_PTCH_DAMP = 0.3
```

**Tuning Notes**:
- Higher time constants due to inertia
- Lower rate limits match slower response
- More damping needed for smooth flight
- Pay attention to structural flex
- May require custom tuning approach

## Troubleshooting

This section addresses common tuning problems and their solutions.

### Oscillations

#### High-Frequency Oscillations (>5 Hz, "Buzzing")

**Symptoms**:
- Rapid vibration in control surfaces
- High-frequency noise in logs
- Motor/servo buzzing sound

**Causes**:
- Excessive D gain
- Insufficient D filtering
- Mechanical resonance
- Sensor noise

**Solutions**:
1. Reduce D gain by 30-50%
2. Lower filter frequencies (`FLTD`, `FLTT`)
3. Check for mechanical slop in linkages
4. Verify sensor mounting is vibration-isolated
5. Check `INS_GYRO_FILTER` not too high

#### Medium-Frequency Oscillations (1-5 Hz, "Rocking")

**Symptoms**:
- Visible rocking motion
- Aircraft "hunts" for correct attitude
- Oscillation at constant amplitude

**Causes**:
- Excessive P gain
- Insufficient D gain
- Slew rate limiting active

**Solutions**:
1. Reduce P gain by 20-30%
2. Increase D gain by 20-30%
3. Check `Dmod` in logs for slew limiting
4. Verify control surface linkages are tight
5. Check FF is not too high

#### Low-Frequency Oscillations (<1 Hz, "Porpoising")

**Symptoms**:
- Slow pitch or roll oscillations
- "Bounce back" after maneuver
- Growing amplitude over time

**Causes**:
- Excessive I gain
- IMAX too high
- Integrator wind-up

**Solutions**:
1. Reduce I gain by 30-50%
2. Reduce `IMAX` parameter
3. Check integrator saturation in logs
4. Increase D gain slightly
5. Verify trim is correct

### Poor Tracking

#### Sluggish Response

**Symptoms**:
- Delayed response to inputs
- Aircraft feels "mushy"
- Large attitude errors during maneuvers

**Causes**:
- Insufficient P gain
- Low FF gain
- Rate limits too low
- Control surface authority limited

**Solutions**:
1. Increase P gain by 20%
2. Increase FF gain
3. Raise `RATE_MAX` parameters
4. Check control surface throws are adequate
5. Verify linkages are not binding

#### Overshoot

**Symptoms**:
- Aircraft overshoots desired attitude
- Large initial response, then settles
- "Ballooning" in turns

**Causes**:
- Insufficient D gain
- Excessive FF gain
- Excessive P gain
- Wrong aircraft model in TECS

**Solutions**:
1. Increase D gain by 30%
2. Reduce FF gain slightly
3. Check P gain not excessive
4. Verify TECS parameters appropriate for aircraft

### Altitude/Airspeed Issues

#### Altitude Oscillations in AUTO Mode

**Symptoms**:
- Aircraft climbs and descends cyclically
- Unstable altitude hold
- Speed varies with altitude

**Causes**:
- `TECS_PTCH_DAMP` too low
- `TECS_TIME_CONST` too low
- Pitch controller oscillating
- Airspeed sensor errors

**Solutions**:
1. Increase `TECS_PTCH_DAMP` to 0.3-0.4
2. Increase `TECS_TIME_CONST`
3. Verify pitch controller is stable
4. Check airspeed sensor calibration

#### Airspeed Oscillations

**Symptoms**:
- Throttle hunts constantly
- Airspeed varies ±3 m/s or more
- Unstable cruise

**Causes**:
- `TECS_THR_DAMP` too low
- `TECS_SPDWEIGHT` inappropriate
- Throttle response nonlinear

**Solutions**:
1. Increase `TECS_THR_DAMP` to 0.4-0.5
2. Adjust `TECS_SPDWEIGHT` toward 1.0
3. Check propeller and motor efficiency
4. Verify throttle calibration

#### Cannot Maintain Altitude in Wind

**Symptoms**:
- Altitude drops in headwind
- Altitude rises in tailwind
- Cannot maintain commanded altitude

**Causes**:
- Insufficient pitch I gain
- Airspeed priority too high
- Insufficient throttle authority
- Wrong `TECS_SPDWEIGHT`

**Solutions**:
1. Increase pitch I gain (`PTCH_I`)
2. Reduce `TECS_SPDWEIGHT` toward 0.7
3. Verify full throttle can climb
4. Check airspeed sensor for errors

### QuadPlane Hover Issues

#### Hover Drift

**Symptoms**:
- Aircraft drifts in QLOITER
- Cannot hold position in calm air
- Steady-state position error

**Causes**:
- Insufficient velocity I gain
- CG offset
- Motor thrust imbalance
- Sensor calibration errors

**Solutions**:
1. Increase `Q_P_VELXY_I`
2. Check CG is centered
3. Verify all motors produce equal thrust
4. Recalibrate compass and accelerometers

#### Hover Wobble

**Symptoms**:
- Small oscillations while hovering
- Aircraft rocks gently
- Constant motor adjustments

**Causes**:
- Excessive rate P or I gain
- Insufficient angle P gain
- Vibration affecting sensors

**Solutions**:
1. Reduce rate P by 10-20%
2. Increase angle P slightly
3. Check vibration levels (`VIBE` logs)
4. Verify motor mounts are solid

#### Poor Transition Performance

**Symptoms**:
- Altitude loss during transition
- Stalls during back-transition
- Oscillations in transition

**Causes**:
- Transition airspeed too low
- Fixed-wing tune inadequate
- Hover tune inadequate
- Wrong transition parameters

**Solutions**:
1. Increase `Q_TRANS_DECEL` for back-transitions
2. Ensure hover tune is solid first
3. Verify fixed-wing tune works well
4. Adjust `Q_ASSIST_SPEED` appropriately
5. Test transitions at safe altitude

### Log Analysis Troubleshooting

#### Checking for Slew Limiting

```
Graph: PIDR.Dmod or PIDP.Dmod
Normal: Dmod = 1.0 most of the time
Problem: Dmod < 1.0 frequently
Solution: Reduce P or D gain
```

#### Checking Integrator Saturation

```
Graph: PIDR.I or PIDP.I
Normal: I stays well below IMAX
Problem: I frequently at +/- IMAX
Solution: Reduce I gain or increase P gain
```

#### Checking Control Authority

```
Graph: RCOU.C1, C2, C3, C4 (servo outputs)
Normal: Outputs stay in middle 70% of range
Problem: Outputs at limits frequently
Solution: Increase servo throws or reduce rate limits
```

#### Checking Vibration

```
Graph: VIBE.VibeX, VibeY, VibeZ
Normal: < 15 m/s/s for most platforms
Problem: > 30 m/s/s consistently
Solution: Improve motor/sensor mounting, balance props
```

### When to Start Over

Sometimes the best solution is to reset to defaults and start fresh:

**Indicators**:
- Cannot find stable gains despite many attempts
- Aircraft behavior unpredictable
- Major hardware changes made
- Logs show confusing or contradictory data

**Reset Procedure**:
1. Save current parameters as backup
2. Reset PID gains to defaults:
   ```
   Mission Planner: Full Parameter List → Right-click → Reset to Default
   ```
3. Verify mechanical systems:
   - CG location
   - Control surface operation
   - Motor/servo operation
   - Sensor calibration
4. Start tuning from scratch following systematic approach
5. Document everything

## Conclusion

Tuning ArduPlane requires patience, systematic approach, and understanding of PID control theory. Key takeaways:

1. **Safety First**: Always maintain safety margins and test in controlled conditions
2. **Start Conservative**: Begin with low gains and increase gradually
3. **One Parameter at Time**: Isolate effects of each change
4. **Use Automated Tools**: AUTOTUNE and Quicktune are highly effective
5. **Analyze Logs**: Use flight logs to understand controller behavior
6. **Document Everything**: Keep detailed notes of changes and results
7. **Test Thoroughly**: Verify tuning across entire flight envelope

With proper tuning, ArduPlane provides exceptional flight performance, stability, and autonomous capability across a wide range of aircraft types and missions.

### Additional Resources

- **ArduPilot Documentation**: https://ardupilot.org/plane/
- **ArduPilot Discourse**: https://discuss.ardupilot.org/
- **PID Theory**: https://en.wikipedia.org/wiki/PID_controller
- **TECS Paper**: "Total Energy Control System for Small Unmanned Aircraft"
- **Parameter Files**: https://github.com/ArduPilot/ardupilot/tree/master/Tools/Frame_params

### Contributing

This documentation is maintained by the ArduPilot development community. To report errors or suggest improvements, please visit:
- GitHub: https://github.com/ArduPilot/ardupilot
- Discuss: https://discuss.ardupilot.org/

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**Applies to**: ArduPlane 4.3+  
**Source References**: 
- `/ArduPlane/tuning.cpp`
- `/ArduPlane/tuning.h`
- `/libraries/APM_Control/AP_AutoTune.cpp`
- `/libraries/AP_Quicktune/AP_Quicktune.cpp`
- `/libraries/AP_Scripting/applets/VTOL-quicktune.lua`