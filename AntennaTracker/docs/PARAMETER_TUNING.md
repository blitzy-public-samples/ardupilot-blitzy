# AntennaTracker Parameter Tuning Guide

![ArduPilot AntennaTracker](https://img.shields.io/badge/ArduPilot-AntennaTracker-blue)
![Documentation](https://img.shields.io/badge/type-parameter%20tuning-green)

## Table of Contents
- [Overview](#overview)
- [Quick Start Tuning Checklist](#quick-start-tuning-checklist)
- [PID Controller Tuning](#pid-controller-tuning)
  - [YAW2SRV Controller](#yaw2srv-controller)
  - [PITCH2SRV Controller](#pitch2srv-controller)
  - [PID Tuning Methodology](#pid-tuning-methodology)
- [Servo Configuration](#servo-configuration)
  - [Servo Types](#servo-types)
  - [Servo Slew Rates](#servo-slew-rates)
  - [Servo Limits and Ranges](#servo-limits-and-ranges)
- [Tracking Behavior](#tracking-behavior)
  - [Distance and Altitude Configuration](#distance-and-altitude-configuration)
  - [Trim Adjustments](#trim-adjustments)
- [Startup Configuration](#startup-configuration)
- [Scan Mode Parameters](#scan-mode-parameters)
- [Advanced Configuration](#advanced-configuration)
  - [Filter Settings](#filter-settings)
  - [On-Off Servo Parameters](#on-off-servo-parameters)
  - [Continuous Rotation Servos](#continuous-rotation-servos)
- [Complete Parameter Reference](#complete-parameter-reference)
- [Tuning Procedures by Antenna Type](#tuning-procedures-by-antenna-type)
- [Common Issues and Solutions](#common-issues-and-solutions)

## Overview

The AntennaTracker uses a sophisticated control system to maintain accurate pointing at a target vehicle. Proper parameter tuning is essential for:

- **Smooth tracking**: Minimizing oscillations and overshoots
- **Fast response**: Quickly acquiring and following moving targets
- **Mechanical protection**: Preventing excessive servo speeds that could damage equipment
- **Accuracy**: Maintaining precise pointing within your system's capabilities

**Source Files**: 
- `/AntennaTracker/Parameters.h` - Parameter declarations
- `/AntennaTracker/Parameters.cpp` - Parameter definitions and defaults
- `/AntennaTracker/servos.cpp` - Servo control implementation
- `/AntennaTracker/tracking.cpp` - Tracking logic

> **Note**: All tuning should be performed with the antenna tracker in a safe configuration where unexpected movements won't cause damage. Always start with conservative values and gradually refine.

## Quick Start Tuning Checklist

Before detailed tuning, configure these essential parameters:

1. **Servo Type Selection**:
   - `SERVO_YAW_TYPE` = 0 (Position servo - most common)
   - `SERVO_PITCH_TYPE` = 0 (Position servo - most common)

2. **Servo Range Configuration**:
   - `YAW_RANGE` = 360 (degrees, full rotation capability)
   - `PITCH_MIN` = -90 (degrees, lowest elevation angle)
   - `PITCH_MAX` = 90 (degrees, highest elevation angle)

3. **RC Channel Configuration**:
   - Configure `RC1_MIN` and `RC1_MAX` for yaw servo full range
   - Configure `RC2_MIN` and `RC2_MAX` for pitch servo full range
   - Set `RC1_REV` and `RC2_REV` as needed for correct direction

4. **Safety Parameters**:
   - `STARTUP_DELAY` = 2.0 seconds (allows servo initialization)
   - `DISTANCE_MIN` = 5 meters (minimum tracking distance)
   - `YAW_SLEW_TIME` = 2.0 seconds (limits maximum yaw speed)
   - `PITCH_SLEW_TIME` = 2.0 seconds (limits maximum pitch speed)

5. **Initial PID Values** (defaults are generally good starting points):
   - `PITCH2SRV_P` = 0.2
   - `PITCH2SRV_I` = 0.0
   - `PITCH2SRV_D` = 0.05
   - `YAW2SRV_P` = 0.2
   - `YAW2SRV_I` = 0.0
   - `YAW2SRV_D` = 0.05

## PID Controller Tuning

The AntennaTracker uses two independent PID controllers to maintain accurate pointing:
- **YAW2SRV**: Controls azimuth (horizontal rotation)
- **PITCH2SRV**: Controls elevation (vertical angle)

Both controllers follow the same tuning principles but require independent tuning due to different mechanical characteristics.

### YAW2SRV Controller

The yaw controller drives the antenna azimuth to track the vehicle's bearing from the tracker position.

**Source**: `/AntennaTracker/servos.cpp:update_yaw_position_servo()` (lines 158-212)

#### YAW2SRV Parameters

| Parameter | Description | Range | Default | Units |
|-----------|-------------|-------|---------|-------|
| `YAW2SRV_P` | Proportional gain | 0.0 - 3.0 | 0.2 | - |
| `YAW2SRV_I` | Integral gain | 0.0 - 3.0 | 0.0 | - |
| `YAW2SRV_D` | Derivative gain | 0.001 - 0.1 | 0.05 | - |
| `YAW2SRV_IMAX` | I-term maximum | 0 - 4000 | 4000 | d% |
| `YAW2SRV_FF` | Feed-forward gain | 0 - 0.5 | 0.02 | - |
| `YAW2SRV_FLTT` | Target filter frequency | 1 - 50 | - | Hz |
| `YAW2SRV_FLTE` | Error filter frequency | 1 - 100 | - | Hz |
| `YAW2SRV_FLTD` | Derivative filter frequency | 1 - 100 | - | Hz |
| `YAW2SRV_SMAX` | Slew rate limit | 0 - 200 | 0 | - |
| `YAW2SRV_PDMX` | PD sum maximum | 0 - 4000 | 4000 | d% |
| `YAW2SRV_D_FF` | Derivative feed-forward | 0 - 0.1 | 0.0 | - |

**Implementation Details**:
- Output is constrained to ±18000 (±180 degrees in centidegrees)
- I-term is reset when servo reaches position limits
- Servo change per update is limited to ±18000
- Final output respects `YAW_RANGE` parameter limits

### PITCH2SRV Controller

The pitch controller drives the antenna elevation to point at the calculated pitch angle to the vehicle.

**Source**: `/AntennaTracker/servos.cpp:update_pitch_position_servo()` (lines 55-97)

#### PITCH2SRV Parameters

| Parameter | Description | Range | Default | Units |
|-----------|-------------|-------|---------|-------|
| `PITCH2SRV_P` | Proportional gain | 0.0 - 3.0 | 0.2 | - |
| `PITCH2SRV_I` | Integral gain | 0.0 - 3.0 | 0.0 | - |
| `PITCH2SRV_D` | Derivative gain | 0.001 - 0.1 | 0.05 | - |
| `PITCH2SRV_IMAX` | I-term maximum | 0 - 4000 | 4000 | d% |
| `PITCH2SRV_FF` | Feed-forward gain | 0 - 0.5 | 0.02 | - |
| `PITCH2SRV_FLTT` | Target filter frequency | 1 - 50 | - | Hz |
| `PITCH2SRV_FLTE` | Error filter frequency | 1 - 100 | - | Hz |
| `PITCH2SRV_FLTD` | Derivative filter frequency | 1 - 100 | - | Hz |
| `PITCH2SRV_SMAX` | Slew rate limit | 0 - 200 | 0 | - |
| `PITCH2SRV_PDMX` | PD sum maximum | 0 - 4000 | 4000 | d% |
| `PITCH2SRV_D_FF` | Derivative feed-forward | 0 - 0.1 | 0.0 | - |

**Implementation Details**:
- Output is constrained between `PITCH_MIN` and `PITCH_MAX` (in centidegrees)
- I-term is reset when servo reaches position limits
- Filtered output is maintained for monitoring (10 Hz cutoff)
- Controller updates at tracking rate (typically 50 Hz)

### PID Tuning Methodology

Follow this systematic approach to tune each axis independently:

#### Step 1: Preparation

1. Ensure mechanical system is properly assembled and secure
2. Verify servo directions are correct (use `RC1_REV` and `RC2_REV` if needed)
3. Set all PID gains to defaults or zeros
4. Configure slew rate limits to protect hardware:
   ```
   YAW_SLEW_TIME = 2.0    # Full rotation in 2 seconds maximum
   PITCH_SLEW_TIME = 2.0  # Full pitch range in 2 seconds maximum
   ```

#### Step 2: Tune Proportional Gain (P)

The P gain provides immediate response proportional to the tracking error.

**Procedure**:
1. Start with P = 0.1, I = 0.0, D = 0.0
2. Place a target at a known location and switch to AUTO mode
3. Observe the response:
   - **Too Low**: Slow response, large steady-state error, sluggish tracking
   - **Too High**: Oscillations around target, overshoot, mechanical vibration
4. Increase P in steps of 0.05 until you observe slight oscillation
5. Reduce P by 20-30% for stability margin
6. Typical values: 0.1 - 0.5 depending on servo and antenna mass

**Example from source code** (`/AntennaTracker/servos.cpp:77`):
```cpp
// PID controller calculates servo change based on angle error
float new_servo_out = SRV_Channels::get_output_scaled(SRV_Channel::k_tracker_pitch) + 
                      g.pidPitch2Srv.update_error(nav_status.angle_error_pitch, G_Dt);
```

#### Step 3: Tune Derivative Gain (D)

The D gain provides damping and responds to rate of change of error.

**Procedure**:
1. With P set from previous step, start with D = 0.0
2. Increase D in steps of 0.01
3. Observe the response:
   - **Too Low**: Overshoot, oscillations during rapid target movement
   - **Too High**: Sluggish response, resistance to movement, noise sensitivity
4. Optimal D will dampen oscillations without slowing response
5. Typical values: 0.02 - 0.08

**Interaction with FLTD**:
- `YAW2SRV_FLTD` and `PITCH2SRV_FLTD` filter the derivative term
- Higher filter frequency (Hz) = less filtering, more responsive but noisier
- Lower filter frequency = more filtering, smoother but may reduce D effectiveness
- Start with 10-20 Hz and adjust based on noise levels

#### Step 4: Tune Integral Gain (I)

The I gain eliminates steady-state errors but can cause overshoot if too high.

**Procedure**:
1. With P and D set from previous steps, start with I = 0.0
2. Track a stationary target and note any steady-state error
3. If steady-state error exists, increase I in steps of 0.01
4. Observe the response:
   - **Too Low**: Persistent steady-state error, fails to reach exact target
   - **Too High**: Overshoot, oscillation, I-term windup during saturation
5. Typical values: 0.0 - 0.05 (often can remain at 0.0 for antenna trackers)

**I-term Management** (`/AntennaTracker/servos.cpp:82-86`):
```cpp
// I-term is automatically reset when servo reaches limits to prevent windup
if (new_servo_out <= pitch_min_cd) {
    new_servo_out = pitch_min_cd;
    g.pidPitch2Srv.reset_I();
}
```

#### Step 5: Feed-Forward Tuning (Advanced)

Feed-forward gains can improve tracking of moving targets by anticipating required control output.

**Parameters**:
- `YAW2SRV_FF` / `PITCH2SRV_FF`: Feed-forward based on target rate
- `YAW2SRV_D_FF` / `PITCH2SRV_D_FF`: Derivative feed-forward based on rate of change of target

**Procedure**:
1. Only tune after P, I, D are well-tuned
2. Track a fast-moving vehicle
3. Increase FF in steps of 0.01 if tracking lags behind target
4. Typical values: 0.0 - 0.05

#### Step 6: Advanced Filter Configuration

Modern AC_PID controllers include multiple filters to reduce noise and improve stability.

**Filter Parameters**:
- **FLTT** (Target Filter): Filters the target setpoint
  - Reduces response to noisy target data
  - Set to 5-20 Hz based on target position update rate
  
- **FLTE** (Error Filter): Filters the error signal
  - Reduces noise in P-term response
  - Set to 10-50 Hz based on sensor noise levels
  
- **FLTD** (Derivative Filter): Filters the derivative term
  - Critical for reducing D-term noise amplification
  - Set to 10-30 Hz, lower values for noisy systems

**Tuning Approach**:
1. Start with filters disabled (high frequency values)
2. Tune P, I, D gains
3. If system is noisy or oscillates at high frequency, enable filters
4. Reduce filter frequencies until acceptable smoothness achieved
5. Re-tune gains if filters significantly change response

#### Slew Rate Limiting (SMAX)

**Purpose**: Protects servos and mechanics from excessive rates during rapid maneuvers.

**Source**: `/AntennaTracker/Parameters.cpp:323-328, 419-424`

**Configuration**:
- `YAW2SRV_SMAX` and `PITCH2SRV_SMAX` limit combined P+D output rate
- Set to no more than 25% of actuator maximum slew rate
- Value of 0 disables limiting
- Gain is reduced to respect limit (minimum 10% of nominal value)
- Helps prevent high-frequency oscillations from excessive gain

**Typical Values**: 50-150 for most antenna systems

## Servo Configuration

### Servo Types

The AntennaTracker supports three servo types for each axis, configured independently:

**Source**: `/AntennaTracker/Parameters.cpp:80-92`

#### Position Servos (Type 0) - DEFAULT

Standard position-controlled servos that move to a specific angle.

**Parameters**:
- `SERVO_YAW_TYPE = 0`
- `SERVO_PITCH_TYPE = 0`

**Characteristics**:
- Most common configuration
- Direct angle control via PID
- Requires full range calibration with RC_MIN/RC_MAX
- Typically 180-360 degree range depending on gearing

**RC Channel Setup**:
```
# Yaw Servo (Channel 1)
RC1_MIN = 680      # Adjust for -180° position (or -YAW_RANGE/2)
RC1_MAX = 2380     # Adjust for +180° position (or +YAW_RANGE/2)
RC1_TRIM = 1500    # Center position (0°)
RC1_REV = -1       # Reverse if needed for correct direction

# Pitch Servo (Channel 2)
RC2_MIN = 640      # Adjust for PITCH_MIN position
RC2_MAX = 2540     # Adjust for PITCH_MAX position
RC2_TRIM = 1500    # Horizon position (0°)
RC2_REV = -1       # Reverse if needed (increasing PWM should increase elevation)
```

**Implementation**: `/AntennaTracker/servos.cpp:update_yaw_position_servo()`, `update_pitch_position_servo()`

#### On-Off Servos (Type 1)

Bang-bang control for servos that only support full-speed movement in either direction.

**Parameters**:
- `SERVO_YAW_TYPE = 1`
- `SERVO_PITCH_TYPE = 1`

**Additional Parameters Required**:
- `ONOFF_YAW_RATE`: Rotation speed in degrees/second
- `ONOFF_PITCH_RATE`: Rotation speed in degrees/second
- `ONOFF_YAW_MINT`: Minimum movement time in seconds
- `ONOFF_PITCH_MINT`: Minimum movement time in seconds

**Use Cases**:
- Linear actuators with limit switches
- Servos with no position feedback
- Motorized mounts with only direction control

**Implementation**: `/AntennaTracker/servos.cpp:update_yaw_onoff_servo()`, `update_pitch_onoff_servo()`

#### Continuous Rotation Servos (Type 2)

Servos modified for continuous rotation, controlled by speed rather than position.

**Parameters**:
- `SERVO_YAW_TYPE = 2`
- `SERVO_PITCH_TYPE = 2`

**Characteristics**:
- PID output controls rotation speed
- Suitable for continuous 360° rotation
- Requires accurate heading feedback

**Use Cases**:
- Full 360° azimuth rotation
- Slip ring equipped systems
- CR servo modifications

**Implementation**: `/AntennaTracker/servos.cpp:update_yaw_cr_servo()`, `update_pitch_cr_servo()`

### Servo Slew Rates

Slew rate parameters limit the maximum speed of servo movement, protecting mechanical components.

**Source**: `/AntennaTracker/Parameters.cpp:26-42`

#### YAW_SLEW_TIME

**Description**: Time for yaw servo to slew through its full range.

**Parameter**: `YAW_SLEW_TIME`
- **Units**: Seconds
- **Range**: 0 - 20
- **Default**: 2.0
- **Increment**: 0.1

**Configuration**:
- Value of 0 = unlimited servo speed (use with caution)
- Value of 2.0 = full `YAW_RANGE` rotation takes 2 seconds
- Actual maximum speed = `YAW_RANGE` / `YAW_SLEW_TIME` degrees/second

**Example**:
```
YAW_RANGE = 360 degrees
YAW_SLEW_TIME = 3.0 seconds
Maximum yaw speed = 360/3 = 120 degrees/second
```

**Tuning Guidelines**:
- Start with conservative value (3-5 seconds)
- Gradually reduce while monitoring mechanical stress
- Consider antenna mass and servo torque ratings
- Increase if gears skip, servos overheat, or structure vibrates

#### PITCH_SLEW_TIME

**Description**: Time for pitch servo to slew through its full range.

**Parameter**: `PITCH_SLEW_TIME`
- **Units**: Seconds
- **Range**: 0 - 20
- **Default**: 2.0
- **Increment**: 0.1

**Configuration**:
- Value of 0 = unlimited servo speed
- Value of 2.0 = full `PITCH_MIN` to `PITCH_MAX` movement takes 2 seconds
- Actual maximum speed = (`PITCH_MAX` - `PITCH_MIN`) / `PITCH_SLEW_TIME` degrees/second

**Example**:
```
PITCH_MIN = -90 degrees
PITCH_MAX = 90 degrees
PITCH_SLEW_TIME = 2.5 seconds
Maximum pitch speed = 180/2.5 = 72 degrees/second
```

#### MIN_REVERSE_TIME

**Description**: Minimum time to apply when reversing yaw direction at limits.

**Parameter**: `MIN_REVERSE_TIME`
**Source**: `/AntennaTracker/Parameters.cpp:44-51`
- **Units**: Seconds
- **Range**: 0 - 20
- **Default**: 1.0
- **Increment**: 1.0

**Purpose**:
- When tracker reaches yaw limit, it must reverse direction
- This parameter ensures servo moves fully in reverse direction
- Compensates for servo lag and mechanical backlash
- Prevents getting stuck at limits

**Configuration**:
- Increase if tracker gets stuck at yaw limits
- Increase for systems with significant mechanical lag
- Can reduce for fast-responding systems

### Servo Limits and Ranges

#### YAW_RANGE

**Description**: Total range of yaw axis motion.

**Parameter**: `YAW_RANGE`
**Source**: `/AntennaTracker/Parameters.cpp:148-155`
- **Units**: Degrees
- **Range**: 0 - 360
- **Default**: 360 (from YAW_RANGE_DEFAULT)
- **Increment**: 0.1

**Configuration**:
```
YAW_RANGE = 360  # Full rotation capability
YAW_RANGE = 180  # Limited to ±90° from center
YAW_RANGE = 270  # Limited to ±135° from center
```

**Implementation**: Servo output is constrained to ±(`YAW_RANGE`/2) centidegrees from center

**Use Cases**:
- 360°: Slip ring systems, continuous rotation
- <360°: Cable-managed systems, limited mechanical range

#### PITCH_MIN and PITCH_MAX

**Description**: Minimum and maximum pitch (elevation) angles.

**Parameters**:
**Source**: `/AntennaTracker/Parameters.cpp:182-198`

`PITCH_MIN`:
- **Units**: Degrees
- **Range**: -90 to 0
- **Default**: -90 (from PITCH_MIN_DEFAULT)
- **Increment**: 1.0

`PITCH_MAX`:
- **Units**: Degrees
- **Range**: 0 to 90
- **Default**: 90 (from PITCH_MAX_DEFAULT)
- **Increment**: 1.0

**Configuration Examples**:
```
# Full elevation range
PITCH_MIN = -90  # Point straight down
PITCH_MAX = 90   # Point straight up

# Ground-based tracker (no downward pointing)
PITCH_MIN = 0    # Horizon
PITCH_MAX = 90   # Zenith

# Limited range to avoid obstacles
PITCH_MIN = -10  # Slightly below horizon
PITCH_MAX = 80   # Nearly overhead
```

**Important**: Ensure RC2_MIN/RC2_MAX are calibrated to achieve these exact angles

## Tracking Behavior

### Distance and Altitude Configuration

#### DISTANCE_MIN

**Description**: Minimum distance at which tracker will track targets.

**Parameter**: `DISTANCE_MIN`
**Source**: `/AntennaTracker/Parameters.cpp:157-164`
- **Units**: Meters
- **Range**: 0 - 100
- **Default**: 5 (from DISTANCE_MIN_DEFAULT)
- **Increment**: 1.0

**Purpose**:
- Prevents erratic tracking when vehicle is very close
- Reduces wear when precise tracking unnecessary
- Avoids servo chatter from position noise at short range

**Configuration Guidelines**:
```
DISTANCE_MIN = 5    # Standard for most applications
DISTANCE_MIN = 10   # Larger antennas, reduce close-range tracking
DISTANCE_MIN = 1    # Precision applications requiring close tracking
```

**Implementation**: When `nav_status.distance < DISTANCE_MIN`, tracking may be inhibited depending on mode

#### ALT_SOURCE

**Description**: Source of altitude information for pitch calculation.

**Parameter**: `ALT_SOURCE`
**Source**: `/AntennaTracker/Parameters.cpp:166-171, /AntennaTracker/tracking.cpp:64-79`
- **Values**: 
  - 0 = Barometer
  - 1 = GPS
  - 2 = GPS vehicle only
- **Default**: 0 (Barometer)

**Source Implementation**: `/AntennaTracker/tracking.cpp:64-79`
```cpp
// Calculate altitude difference based on ALT_SOURCE setting
if (g.alt_source == ALT_SOURCE_GPS){
    nav_status.alt_difference_gps = (vehicle.location_estimate.alt - current_loc.alt) * 0.01f;
} else {
    // ALT_SOURCE_GPS_VEH_ONLY: Use vehicle's relative altitude
    nav_status.alt_difference_gps = vehicle.relative_alt * 0.01f;
}

// Calculate pitch to vehicle
if (g.alt_source == ALT_SOURCE_BARO) {
    nav_status.pitch = degrees(atan2f(nav_status.alt_difference_baro, nav_status.distance));
} else {
    nav_status.pitch = degrees(atan2f(nav_status.alt_difference_gps, nav_status.distance));
}
```

**Configuration Guidelines**:

| ALT_SOURCE | Use Case | Requirements |
|------------|----------|--------------|
| 0 (Barometer) | Stationary tracker with barometer | Tracker has barometer, both at similar altitude |
| 1 (GPS) | Mobile tracker | Both tracker and vehicle have GPS |
| 2 (GPS Vehicle Only) | Stationary tracker without baro | Vehicle GPS altitude relative to home, tracker altitude assumed same as vehicle home |

**Choosing the Right Source**:
- **Barometer (0)**: Best accuracy for fixed ground stations with calibrated baro
- **GPS (1)**: Required for tracker on moving platform
- **GPS Vehicle Only (2)**: Simplest for fixed trackers without barometer

### Trim Adjustments

Trim parameters allow fine-tuning of pointing accuracy to compensate for sensor biases or mechanical misalignment.

#### YAW_TRIM

**Description**: Additional yaw offset to add when tracking.

**Parameter**: `YAW_TRIM`
**Source**: `/AntennaTracker/Parameters.cpp:130-137`
- **Units**: Degrees
- **Range**: -10 to 10
- **Default**: 0
- **Increment**: 0.1

**Purpose**:
- Compensates for compass declination errors
- Corrects for mechanical misalignment
- Fine-tunes pointing accuracy

**Tuning Procedure**:
1. Track a stationary target at known location
2. Measure actual pointing direction vs. expected
3. Set `YAW_TRIM` to offset (positive = rotate clockwise)
4. Verify correction with multiple bearings

#### PITCH_TRIM

**Description**: Additional pitch offset to add when tracking.

**Parameter**: `PITCH_TRIM`
**Source**: `/AntennaTracker/Parameters.cpp:139-146`
- **Units**: Degrees
- **Range**: -10 to 10
- **Default**: 0
- **Increment**: 0.1

**Purpose**:
- Compensates for barometer calibration errors
- Corrects for mechanical misalignment in elevation
- Adjusts for consistent altitude measurement bias

**Tuning Procedure**:
1. Track a target at known altitude
2. Measure actual pitch angle vs. expected
3. Set `PITCH_TRIM` to offset (positive = pitch up)
4. Verify with targets at various altitudes

## Startup Configuration

### STARTUP_DELAY

**Description**: Delay before first servo movement from trim position.

**Parameter**: `STARTUP_DELAY`
**Source**: `/AntennaTracker/Parameters.cpp:71-78, /AntennaTracker/tracking.cpp:96-100`
- **Units**: Seconds
- **Range**: 0 - 10
- **Default**: 0
- **Increment**: 0.1

**Purpose**:
- Allows servos to initialize and reach trim positions
- Prevents initial jerking motion on power-up
- Gives time for sensor calibration to complete

**Implementation**: `/AntennaTracker/tracking.cpp:96-100`
```cpp
// Servo updates are blocked until startup delay expires
if (g.startup_delay > 0 &&
    AP_HAL::millis() - start_time_ms < g.startup_delay*1000) {
    return;  // No servo updates
}
```

**Configuration Guidelines**:
```
STARTUP_DELAY = 0     # No delay (servos move immediately)
STARTUP_DELAY = 2.0   # Recommended for most systems
STARTUP_DELAY = 5.0   # Systems requiring extended initialization
```

**Use Cases**:
- **0 seconds**: Fast-reboot scenarios, pre-calibrated systems
- **2-3 seconds**: Standard configuration, allows clean startup
- **5-10 seconds**: Mechanical systems requiring settling time

### START_LATITUDE and START_LONGITUDE

**Description**: Initial tracker position before GPS lock.

**Parameters**:
**Source**: `/AntennaTracker/Parameters.cpp:53-69`

`START_LATITUDE`:
- **Units**: Degrees
- **Range**: -90 to 90
- **Default**: 0
- **Increment**: 0.000001

`START_LONGITUDE`:
- **Units**: Degrees
- **Range**: -180 to 180
- **Default**: 0
- **Increment**: 0.000001

**Use Cases**:

1. **GPS-equipped mobile tracker**:
   ```
   START_LATITUDE = 0   # GPS will provide position
   START_LONGITUDE = 0
   ```

2. **Fixed tracker with GPS**:
   ```
   START_LATITUDE = 37.774929   # Approximate location
   START_LONGITUDE = -122.419418
   # GPS will refine position after lock
   ```

3. **Fixed tracker without GPS**:
   ```
   START_LATITUDE = 37.774929   # Exact surveyed position
   START_LONGITUDE = -122.419418
   # Used as permanent tracker location
   ```

**Accuracy Requirements**:
- Within 10 meters: ±0.0001 degrees
- Within 100 meters: ±0.001 degrees
- For precise tracking, use 6+ decimal places

## Scan Mode Parameters

Scan mode provides autonomous search patterns when target is lost or for initial acquisition.

**Source**: `/AntennaTracker/Parameters.cpp:473-489`

### SCAN_SPEED_YAW

**Description**: Rotation speed for yaw axis in SCAN mode.

**Parameter**: `SCAN_SPEED_YAW`
- **Units**: Degrees/second
- **Range**: 0 - 100
- **Default**: 2
- **Increment**: 1

**Configuration Guidelines**:
```
SCAN_SPEED_YAW = 2    # Slow scan for wide area
SCAN_SPEED_YAW = 10   # Medium scan for known search area
SCAN_SPEED_YAW = 30   # Fast scan for quick acquisition
```

**Considerations**:
- Slower speeds improve target detection probability
- Faster speeds reduce time to cover search area
- Must be compatible with `YAW_SLEW_TIME` limits
- Consider antenna beamwidth (wider beam = faster scan possible)

### SCAN_SPEED_PIT

**Description**: Rotation speed for pitch axis in SCAN mode.

**Parameter**: `SCAN_SPEED_PIT`
- **Units**: Degrees/second
- **Range**: 0 - 100
- **Default**: 5
- **Increment**: 1

**Configuration Guidelines**:
```
SCAN_SPEED_PIT = 1    # Very slow elevation scan
SCAN_SPEED_PIT = 5    # Standard elevation scan
SCAN_SPEED_PIT = 15   # Fast elevation scan
```

**Scan Pattern Strategy**:
- Typically, yaw scans faster than pitch
- Pitch may scan in steps (e.g., scan full azimuth, step up 10°, repeat)
- Configure for expected target altitude range

### INITIAL_MODE

**Description**: Mode tracker will switch into after initialization.

**Parameter**: `INITIAL_MODE`
**Source**: `/AntennaTracker/Parameters.cpp:491-495`
- **Values**:
  - 0 = MANUAL
  - 1 = STOP
  - 2 = SCAN
  - 10 = AUTO
- **Default**: 10 (AUTO)

**Mode Descriptions**:

| Mode | Value | Behavior |
|------|-------|----------|
| MANUAL | 0 | Manual RC control of servos |
| STOP | 1 | Servos hold position or output trim/zero |
| SCAN | 2 | Autonomous scan pattern using SCAN_SPEED parameters |
| AUTO | 10 | Track vehicle based on telemetry |

**Configuration Guidelines**:
```
INITIAL_MODE = 10  # AUTO - immediately track when telemetry received (most common)
INITIAL_MODE = 2   # SCAN - scan for target on startup
INITIAL_MODE = 1   # STOP - wait for mode change command
INITIAL_MODE = 0   # MANUAL - RC control only
```

### AUTO_OPTIONS

**Description**: Bitmask of options for AUTO mode behavior.

**Parameter**: `AUTO_OPTIONS`
**Source**: `/AntennaTracker/Parameters.cpp:503-508`
- **Bitmask**: Bit 0: Scan for unknown target
- **Default**: 0

**Configuration**:
```
AUTO_OPTIONS = 0  # No auto-scan
AUTO_OPTIONS = 1  # Enable scan when target unknown (bit 0 set)
```

**Bit 0 (Scan for unknown target)**:
- When enabled, tracker will scan if no target telemetry received
- Automatically switches to tracking when target acquired
- Uses SCAN_SPEED parameters for scan pattern

## Advanced Configuration

### Filter Settings

Modern antenna trackers include filtering to improve tracking smoothness and reduce noise.

#### Servo Output Filters

**Source**: `/AntennaTracker/servos.cpp:25-26`

```cpp
yaw_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
pitch_servo_out_filt.set_cutoff_frequency(SERVO_OUT_FILT_HZ);
```

**Implementation Details**:
- Low-pass filters on servo output
- Fixed cutoff frequency (SERVO_OUT_FILT_HZ, typically 10 Hz)
- Reduces high-frequency servo movement
- Filtered output used for monitoring, not control

**Effect**:
- Smoother visual tracking
- Reduced servo wear
- May introduce slight lag in rapid movements

### On-Off Servo Parameters

Configuration for bang-bang control when using on-off servos (`SERVO_*_TYPE = 1`).

**Source**: `/AntennaTracker/Parameters.cpp:94-128, /AntennaTracker/servos.cpp:104-121, 220-234`

#### ONOFF_YAW_RATE

**Description**: Yaw rotation speed for on-off servos.

**Parameter**: `ONOFF_YAW_RATE`
- **Units**: Degrees/second
- **Range**: 0 - 50
- **Default**: 9.0
- **Increment**: 0.1

**Purpose**: Tells the tracker how fast the servo moves when activated, used to calculate movement time.

#### ONOFF_PITCH_RATE

**Description**: Pitch rotation speed for on-off servos.

**Parameter**: `ONOFF_PITCH_RATE`
- **Units**: Degrees/second
- **Range**: 0 - 50
- **Default**: 1.0
- **Increment**: 0.1

#### ONOFF_YAW_MINT

**Description**: Minimum movement time for yaw on-off servo.

**Parameter**: `ONOFF_YAW_MINT`
- **Units**: Seconds
- **Range**: 0 - 2
- **Default**: 0.1
- **Increment**: 0.01

**Purpose**: Prevents servo chatter by enforcing minimum activation time.

#### ONOFF_PITCH_MINT

**Description**: Minimum movement time for pitch on-off servo.

**Parameter**: `ONOFF_PITCH_MINT`
- **Units**: Seconds
- **Range**: 0 - 2
- **Default**: 0.1
- **Increment**: 0.01

#### On-Off Control Algorithm

**Source**: `/AntennaTracker/servos.cpp:109`

```cpp
float acceptable_error = g.onoff_pitch_rate * g.onoff_pitch_mintime;
```

**Logic**:
1. Calculate acceptable error = rate × minimum time
2. If angle error < acceptable error: Stop servo (output 0)
3. If angle error > acceptable error: Activate servo (full speed in appropriate direction)

**Tuning Process**:
1. Measure actual servo movement speed → set ONOFF_*_RATE
2. Determine minimum useful movement → set ONOFF_*_MINT
3. Result: Servo only activates when error is large enough to justify movement time

**Example**:
```
ONOFF_YAW_RATE = 10.0 deg/s
ONOFF_YAW_MINT = 0.1 s
Acceptable error = 10.0 * 0.1 = 1.0 degree

Result: Servo ignores errors < 1°, activates for errors > 1°
```

### Continuous Rotation Servos

Configuration for continuous rotation servos (`SERVO_*_TYPE = 2`).

**Source**: `/AntennaTracker/servos.cpp:126-130, 239-243`

#### Control Algorithm

**Yaw CR Servo** (`/AntennaTracker/servos.cpp:241`):
```cpp
const float yaw_out = constrain_float(-g.pidYaw2Srv.update_error(nav_status.angle_error_yaw, G_Dt), 
                                      -g.yaw_range * 100/2, g.yaw_range * 100/2);
```

**Pitch CR Servo** (`/AntennaTracker/servos.cpp:128`):
```cpp
const float pitch_out = constrain_float(g.pidPitch2Srv.update_error(nav_status.angle_error_pitch, G_Dt), 
                                        -(-g.pitch_min+g.pitch_max) * 100/2, 
                                        (-g.pitch_min+g.pitch_max) * 100/2);
```

**Characteristics**:
- PID output directly controls rotation speed
- Positive output = clockwise/up, negative = counter-clockwise/down
- Requires accurate heading feedback (compass/IMU)
- No position limits (continuous rotation)

**Tuning Guidelines**:
1. Tune PID gains similar to position servos
2. P gain typically lower (0.05 - 0.15)
3. D gain important for damping (0.02 - 0.08)
4. I gain can help with steady-state tracking (0.0 - 0.03)

### SAFE_DISARM_PWM

**Description**: PWM output behavior when disarmed or in STOP mode.

**Parameter**: `SAFE_DISARM_PWM`
**Source**: `/AntennaTracker/Parameters.cpp:497-501, /AntennaTracker/tracking.cpp:108-118`
- **Values**:
  - 0 = Zero PWM
  - 1 = Trim PWM
- **Default**: 0

**Implementation**: `/AntennaTracker/tracking.cpp:108-118`
```cpp
if (!hal.util->get_soft_armed()) {
    switch ((PWMDisarmed)g.disarm_pwm.get()) {
    case PWMDisarmed::TRIM:
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
        break;
    default:
    case PWMDisarmed::ZERO:
        SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, 0);
        SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, 0);
        break;
    }
}
```

**Configuration**:
- **0 (Zero PWM)**: Servos receive 0 µs pulse → completely off, may drift
- **1 (Trim PWM)**: Servos receive trim pulse → hold center position

**Selection Guidelines**:
- Use **Zero** if servos have brakes or mechanical locks
- Use **Trim** to maintain position when disarmed
- Consider power consumption and servo heating

### GCS_PID_MASK

**Description**: Bitmask of PIDs to send MAVLink PID_TUNING messages for.

**Parameter**: `GCS_PID_MASK`
**Source**: `/AntennaTracker/Parameters.cpp:466-471`
- **Bitmask**:
  - Bit 0: Pitch PID
  - Bit 1: Yaw PID
- **Default**: 0 (disabled)

**Configuration**:
```
GCS_PID_MASK = 0  # No PID tuning messages
GCS_PID_MASK = 1  # Send Pitch PID data (bit 0)
GCS_PID_MASK = 2  # Send Yaw PID data (bit 1)
GCS_PID_MASK = 3  # Send both Pitch and Yaw (bits 0+1)
```

**Use Cases**:
- Enable during PID tuning to see real-time performance
- View in ground station graphing tools
- Disable during normal operation to reduce telemetry bandwidth

### Target System ID

#### SYSID_TARGET

**Description**: MAVLink system ID of vehicle being tracked.

**Parameter**: `SYSID_TARGET`
**Source**: `/AntennaTracker/Parameters.cpp:19-24`
- **Range**: 1 - 255
- **Default**: 0 (auto-detect)

**Configuration**:
```
SYSID_TARGET = 0    # Auto-detect (track first vehicle seen)
SYSID_TARGET = 1    # Track vehicle with MAV_SYSID = 1
SYSID_TARGET = 2    # Track vehicle with MAV_SYSID = 2
```

**Use Cases**:
- **0 (Auto)**: Single vehicle operations
- **Specific ID**: Multi-vehicle environments, track specific vehicle
- Must match the MAV_SYSID parameter of the target vehicle

### MAV_UPDATE_RATE

**Description**: Rate at which MAVLink position and baro data is processed.

**Parameter**: `MAV_UPDATE_RATE`
**Source**: `/AntennaTracker/Parameters.cpp:173-180`
- **Units**: Hz
- **Range**: 1 - 10
- **Default**: 1
- **Increment**: 1

**Configuration**:
```
MAV_UPDATE_RATE = 1   # Update once per second (low bandwidth)
MAV_UPDATE_RATE = 5   # Update 5 times per second (standard)
MAV_UPDATE_RATE = 10  # Update 10 times per second (fast, high bandwidth)
```

**Trade-offs**:
- **Higher rate**: More responsive tracking, higher telemetry bandwidth
- **Lower rate**: Reduced bandwidth, more prediction/extrapolation
- Tracking loop runs at 50 Hz regardless; this controls telemetry processing

## Complete Parameter Reference

### Core Tracking Parameters

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `SYSID_TARGET` | Target vehicle MAVLink system ID | - | 1-255 | 0 (auto) | Parameters.cpp:19 |
| `DISTANCE_MIN` | Minimum tracking distance | m | 0-100 | 5 | Parameters.cpp:157 |
| `ALT_SOURCE` | Altitude information source | - | 0-2 | 0 (Baro) | Parameters.cpp:166 |
| `YAW_TRIM` | Yaw pointing offset | deg | -10 to 10 | 0 | Parameters.cpp:130 |
| `PITCH_TRIM` | Pitch pointing offset | deg | -10 to 10 | 0 | Parameters.cpp:139 |

### Servo Configuration

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `SERVO_YAW_TYPE` | Yaw servo type | - | 0-2 | 0 (Position) | Parameters.cpp:87 |
| `SERVO_PITCH_TYPE` | Pitch servo type | - | 0-2 | 0 (Position) | Parameters.cpp:80 |
| `YAW_SLEW_TIME` | Yaw full range slew time | s | 0-20 | 2.0 | Parameters.cpp:26 |
| `PITCH_SLEW_TIME` | Pitch full range slew time | s | 0-20 | 2.0 | Parameters.cpp:35 |
| `MIN_REVERSE_TIME` | Minimum yaw reversal time | s | 0-20 | 1.0 | Parameters.cpp:44 |
| `YAW_RANGE` | Yaw axis total range | deg | 0-360 | 360 | Parameters.cpp:148 |
| `PITCH_MIN` | Minimum pitch angle | deg | -90 to 0 | -90 | Parameters.cpp:182 |
| `PITCH_MAX` | Maximum pitch angle | deg | 0-90 | 90 | Parameters.cpp:191 |

### YAW2SRV PID Controller

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `YAW2SRV_P` | Proportional gain | - | 0.0-3.0 | 0.2 | Parameters.cpp:359 |
| `YAW2SRV_I` | Integral gain | - | 0.0-3.0 | 0.0 | Parameters.cpp:366 |
| `YAW2SRV_D` | Derivative gain | - | 0.001-0.1 | 0.05 | Parameters.cpp:381 |
| `YAW2SRV_IMAX` | Integrator maximum | d% | 0-4000 | 4000 | Parameters.cpp:373 |
| `YAW2SRV_FF` | Feed-forward gain | - | 0-0.5 | 0.02 | Parameters.cpp:388 |
| `YAW2SRV_FLTT` | Target filter frequency | Hz | 1-50 | - | Parameters.cpp:395 |
| `YAW2SRV_FLTE` | Error filter frequency | Hz | 1-100 | - | Parameters.cpp:403 |
| `YAW2SRV_FLTD` | Derivative filter frequency | Hz | 1-100 | - | Parameters.cpp:411 |
| `YAW2SRV_SMAX` | Slew rate limit | - | 0-200 | 0 | Parameters.cpp:419 |
| `YAW2SRV_PDMX` | PD sum maximum | d% | 0-4000 | 4000 | Parameters.cpp:426 |
| `YAW2SRV_D_FF` | Derivative feed-forward | - | 0-0.1 | 0.0 | Parameters.cpp:434 |

### PITCH2SRV PID Controller

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `PITCH2SRV_P` | Proportional gain | - | 0.0-3.0 | 0.2 | Parameters.cpp:263 |
| `PITCH2SRV_I` | Integral gain | - | 0.0-3.0 | 0.0 | Parameters.cpp:270 |
| `PITCH2SRV_D` | Derivative gain | - | 0.001-0.1 | 0.05 | Parameters.cpp:285 |
| `PITCH2SRV_IMAX` | Integrator maximum | d% | 0-4000 | 4000 | Parameters.cpp:277 |
| `PITCH2SRV_FF` | Feed-forward gain | - | 0-0.5 | 0.02 | Parameters.cpp:292 |
| `PITCH2SRV_FLTT` | Target filter frequency | Hz | 1-50 | - | Parameters.cpp:299 |
| `PITCH2SRV_FLTE` | Error filter frequency | Hz | 1-100 | - | Parameters.cpp:307 |
| `PITCH2SRV_FLTD` | Derivative filter frequency | Hz | 1-100 | - | Parameters.cpp:315 |
| `PITCH2SRV_SMAX` | Slew rate limit | - | 0-200 | 0 | Parameters.cpp:323 |
| `PITCH2SRV_PDMX` | PD sum maximum | d% | 0-4000 | 4000 | Parameters.cpp:330 |
| `PITCH2SRV_D_FF` | Derivative feed-forward | - | 0-0.1 | 0.0 | Parameters.cpp:338 |

### On-Off Servo Parameters

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `ONOFF_YAW_RATE` | Yaw rate for on-off servos | deg/s | 0-50 | 9.0 | Parameters.cpp:94 |
| `ONOFF_PITCH_RATE` | Pitch rate for on-off servos | deg/s | 0-50 | 1.0 | Parameters.cpp:103 |
| `ONOFF_YAW_MINT` | Yaw minimum movement time | s | 0-2 | 0.1 | Parameters.cpp:112 |
| `ONOFF_PITCH_MINT` | Pitch minimum movement time | s | 0-2 | 0.1 | Parameters.cpp:121 |

### Startup and Mode Parameters

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `STARTUP_DELAY` | Delay before first servo movement | s | 0-10 | 0 | Parameters.cpp:71 |
| `START_LATITUDE` | Initial latitude before GPS lock | deg | -90 to 90 | 0 | Parameters.cpp:53 |
| `START_LONGITUDE` | Initial longitude before GPS lock | deg | -180 to 180 | 0 | Parameters.cpp:62 |
| `INITIAL_MODE` | Mode after initialization | - | 0,1,2,10 | 10 (AUTO) | Parameters.cpp:491 |
| `SAFE_DISARM_PWM` | PWM when disarmed/stopped | - | 0-1 | 0 (Zero) | Parameters.cpp:497 |
| `AUTO_OPTIONS` | AUTO mode options bitmask | - | Bitmask | 0 | Parameters.cpp:503 |

### Scan Mode Parameters

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `SCAN_SPEED_YAW` | Yaw rotation speed in SCAN | deg/s | 0-100 | 2 | Parameters.cpp:473 |
| `SCAN_SPEED_PIT` | Pitch rotation speed in SCAN | deg/s | 0-100 | 5 | Parameters.cpp:482 |

### Communication Parameters

| Parameter | Description | Units | Range | Default | Source |
|-----------|-------------|-------|-------|---------|--------|
| `MAV_UPDATE_RATE` | MAVLink update rate | Hz | 1-10 | 1 | Parameters.cpp:173 |
| `GCS_PID_MASK` | PID tuning message bitmask | - | Bitmask | 0 | Parameters.cpp:466 |

## Tuning Procedures by Antenna Type

### Small Light Antenna (< 1 kg)

**Characteristics**: Fast response, low inertia, may be sensitive to wind

**Recommended Parameters**:
```
# Servo Rates
YAW_SLEW_TIME = 1.5
PITCH_SLEW_TIME = 1.5

# PID Gains - Yaw
YAW2SRV_P = 0.3
YAW2SRV_I = 0.02
YAW2SRV_D = 0.08
YAW2SRV_FLTD = 20

# PID Gains - Pitch
PITCH2SRV_P = 0.3
PITCH2SRV_I = 0.02
PITCH2SRV_D = 0.08
PITCH2SRV_FLTD = 20

# Tracking
DISTANCE_MIN = 5
```

**Tuning Notes**:
- Can use higher P and D gains due to low inertia
- May need I-term to counter wind
- Higher filter frequencies for responsive control
- Watch for oscillations in windy conditions

### Medium Antenna (1-5 kg)

**Characteristics**: Balanced performance, most common configuration

**Recommended Parameters**:
```
# Servo Rates
YAW_SLEW_TIME = 2.0
PITCH_SLEW_TIME = 2.0

# PID Gains - Yaw
YAW2SRV_P = 0.2
YAW2SRV_I = 0.0
YAW2SRV_D = 0.05
YAW2SRV_FLTD = 15

# PID Gains - Pitch
PITCH2SRV_P = 0.2
PITCH2SRV_I = 0.0
PITCH2SRV_D = 0.05
PITCH2SRV_FLTD = 15

# Tracking
DISTANCE_MIN = 5
```

**Tuning Notes**:
- Default parameters often work well
- Moderate response, good stability
- I-term usually not needed
- Balance between speed and smoothness

### Large Heavy Antenna (> 5 kg)

**Characteristics**: High inertia, powerful servos required, smooth tracking

**Recommended Parameters**:
```
# Servo Rates (protect mechanics)
YAW_SLEW_TIME = 3.0
PITCH_SLEW_TIME = 3.0

# PID Gains - Yaw
YAW2SRV_P = 0.15
YAW2SRV_I = 0.0
YAW2SRV_D = 0.04
YAW2SRV_FLTD = 10
YAW2SRV_SMAX = 100

# PID Gains - Pitch
PITCH2SRV_P = 0.15
PITCH2SRV_I = 0.0
PITCH2SRV_D = 0.04
PITCH2SRV_FLTD = 10
PITCH2SRV_SMAX = 80

# Tracking
DISTANCE_MIN = 10
```

**Tuning Notes**:
- Lower P and D gains to prevent overshoot
- Enable SMAX slew rate limiting for safety
- More conservative slew times
- Lower filter frequencies for smooth control
- Monitor servo/motor temperature during tuning

### Directional Antenna (Narrow Beam)

**Characteristics**: Requires high accuracy, less forgiving of pointing errors

**Recommended Parameters**:
```
# Servo Rates
YAW_SLEW_TIME = 2.0
PITCH_SLEW_TIME = 2.0

# PID Gains - Yaw (tighter control)
YAW2SRV_P = 0.25
YAW2SRV_I = 0.01
YAW2SRV_D = 0.06
YAW2SRV_FLTD = 20

# PID Gains - Pitch
PITCH2SRV_P = 0.25
PITCH2SRV_I = 0.01
PITCH2SRV_D = 0.06
PITCH2SRV_FLTD = 20

# Tracking
DISTANCE_MIN = 10  # Avoid erratic close-range tracking
```

**Tuning Notes**:
- Higher P gain for tighter tracking
- Small I-term to eliminate steady-state error
- Accurate trim adjustments critical
- Consider enabling GCS_PID_MASK for monitoring
- May need custom filter frequencies

### Omnidirectional Antenna

**Characteristics**: Wide beam, less critical pointing, may not need pitch control

**Recommended Parameters**:
```
# Servo Rates (can be faster)
YAW_SLEW_TIME = 1.0
PITCH_SLEW_TIME = 1.0  # Or disable pitch tracking

# PID Gains - Yaw (lower gains acceptable)
YAW2SRV_P = 0.15
YAW2SRV_I = 0.0
YAW2SRV_D = 0.03
YAW2SRV_FLTD = 10

# PID Gains - Pitch (if used)
PITCH2SRV_P = 0.1
PITCH2SRV_I = 0.0
PITCH2SRV_D = 0.02

# Tracking
DISTANCE_MIN = 5
SCAN_SPEED_YAW = 20  # Faster scan acceptable
```

**Tuning Notes**:
- Pointing accuracy less critical
- Can use lower gains for smoother operation
- Faster slew rates acceptable
- May only need yaw tracking for cable management

## Common Issues and Solutions

### Issue: Oscillation Around Target

**Symptoms**: Servo continuously moves back and forth around target position

**Causes and Solutions**:

1. **P gain too high**:
   - Reduce `YAW2SRV_P` or `PITCH2SRV_P` by 20-30%
   - Symptom: Large amplitude oscillations

2. **D gain too low**:
   - Increase `YAW2SRV_D` or `PITCH2SRV_D` in steps of 0.01
   - Symptom: Sustained oscillations

3. **I-term windup**:
   - Reduce `YAW2SRV_I` or `PITCH2SRV_I`
   - Reduce `YAW2SRV_IMAX` or `PITCH2SRV_IMAX`
   - Symptom: Slow oscillations that build over time

4. **Mechanical backlash**:
   - Increase D gain slightly
   - Add small I-term (0.005-0.01)
   - Consider mechanical improvements

5. **Noisy sensor data**:
   - Lower filter frequencies: `FLTD`, `FLTE`, `FLTT`
   - Start with all filters at 10 Hz

### Issue: Sluggish Tracking

**Symptoms**: Tracker lags behind fast-moving targets

**Causes and Solutions**:

1. **P gain too low**:
   - Increase `YAW2SRV_P` or `PITCH2SRV_P` in steps of 0.05
   - Test with moving target

2. **Slew rate limits too conservative**:
   - Reduce `YAW_SLEW_TIME` and `PITCH_SLEW_TIME`
   - Monitor for mechanical stress

3. **SMAX limiting gains**:
   - Increase `YAW2SRV_SMAX` or `PITCH2SRV_SMAX`
   - Or disable (set to 0) if not needed

4. **Excessive filtering**:
   - Increase filter frequencies (FLTT, FLTE, FLTD)
   - Start with 20-30 Hz and adjust down if noisy

5. **Missing feed-forward**:
   - Add small `FF` term (0.01-0.05) for moving targets
   - Requires well-tuned P and D first

### Issue: Servo Chatter or Jitter

**Symptoms**: Small rapid servo movements, servo noise, increased power consumption

**Causes and Solutions**:

1. **Noisy sensor data**:
   - Enable and tune filters: `FLTD`, `FLTE`, `FLTT`
   - Start with 10 Hz cutoff, increase if too slow

2. **D gain too high**:
   - Reduce `YAW2SRV_D` or `PITCH2SRV_D`
   - Increase `FLTD` filter cutoff

3. **Target too close**:
   - Increase `DISTANCE_MIN` parameter
   - GPS noise has larger angular effect at close range

4. **Quantization noise**:
   - Increase D filtering (`FLTD` lower frequency)
   - May need to accept some jitter at very close range

### Issue: Steady-State Pointing Error

**Symptoms**: Tracker consistently points slightly off target

**Causes and Solutions**:

1. **Insufficient I gain**:
   - Add small I-term: `YAW2SRV_I` or `PITCH2SRV_I` = 0.005-0.02
   - Monitor for oscillations

2. **Sensor bias**:
   - Adjust `YAW_TRIM` or `PITCH_TRIM` to correct
   - Typical values: ±0.5 to ±2.0 degrees

3. **Mechanical misalignment**:
   - Use trim parameters for correction
   - Consider physical realignment for large errors

4. **Wrong altitude source**:
   - Verify `ALT_SOURCE` setting appropriate for configuration
   - Check barometer calibration if using ALT_SOURCE = 0

5. **Servo calibration error**:
   - Verify `RC1_MIN/MAX` and `RC2_MIN/MAX` achieve exact angular limits
   - Re-calibrate servo endpoints

### Issue: Jerky Movement on Startup

**Symptoms**: Violent servo movement immediately after power-on

**Causes and Solutions**:

1. **No startup delay**:
   - Set `STARTUP_DELAY = 2.0` or higher
   - Allows sensors to stabilize

2. **Servo initialization**:
   - Increase `STARTUP_DELAY` to 3-5 seconds
   - Servos need time to reach trim positions

3. **Initial mode inappropriate**:
   - Change `INITIAL_MODE` from AUTO (10) to STOP (1)
   - Manually switch to AUTO when ready

### Issue: Tracker Gets Stuck at Yaw Limits

**Symptoms**: Tracker reaches yaw limit and fails to reverse direction

**Causes and Solutions**:

1. **Insufficient reverse time**:
   - Increase `MIN_REVERSE_TIME` parameter
   - Try 2-3 seconds for systems with lag

2. **Wrong yaw range**:
   - Verify `YAW_RANGE` matches actual mechanical range
   - Check servo calibration (`RC1_MIN/MAX`)

3. **Mechanical binding**:
   - Check for cable interference at limits
   - Verify servo can physically reach limits

4. **Incorrect servo direction**:
   - Verify servo moves correct direction (check `RC1_REV`)
   - Test manual movement through full range

### Issue: Poor Tracking in Wind

**Symptoms**: Accuracy degrades in windy conditions

**Causes and Solutions**:

1. **No I-term to counter steady bias**:
   - Add I gain: `YAW2SRV_I` = 0.01-0.02
   - Increase `IMAX` if needed (2000-4000)

2. **Insufficient D gain**:
   - Increase D for better damping
   - Balance with D filtering to avoid noise

3. **Mechanical flexibility**:
   - Increase structure rigidity if possible
   - Lower D gain if structure resonates

### Issue: Tracker Doesn't Move at All

**Symptoms**: Servos receive power but don't move

**Causes and Solutions**:

1. **Not armed**:
   - Arm the tracker (specific method depends on configuration)
   - Check `SAFE_DISARM_PWM` setting

2. **Startup delay not expired**:
   - Wait for `STARTUP_DELAY` seconds after boot
   - Or set to 0 for immediate operation

3. **Safety switch disarmed**:
   - Press safety switch if equipped
   - Verify safety switch state

4. **Wrong initial mode**:
   - Check `INITIAL_MODE` parameter
   - Switch to AUTO mode manually if needed

5. **No target telemetry**:
   - Verify telemetry link to vehicle
   - Check `SYSID_TARGET` matches vehicle
   - Try SCAN mode to verify servos work

6. **Servo outputs not configured**:
   - Verify servo channel assignments
   - Check RC1 and RC2 output configuration

### Diagnostic Tips

**Enable PID Tuning Messages**:
```
GCS_PID_MASK = 3  # Enable both pitch and yaw
```
View real-time PID performance in ground station.

**Test Individual Axes**:
- Mechanically lock one axis
- Tune the free axis in isolation
- Reduces interaction effects

**Check Servo Output**:
- Monitor servo positions in ground station
- Verify they respond to tracking errors
- Check for saturation at limits

**Verify Tracking Calculation**:
- Log `VPOS` (vehicle position) messages
- Verify bearing and distance calculations
- Check altitude source providing data

**Progressive Tuning**:
- Start with all gains low
- Increase one parameter at a time
- Test thoroughly before next adjustment
- Document changes and results

---

## Additional Resources

**ArduPilot Documentation**:
- [AntennaTracker Home](https://ardupilot.org/antennatracker/)
- [First Time Setup](https://ardupilot.org/antennatracker/docs/initial-setup.html)
- [MAVLink Protocol](https://mavlink.io/en/)

**Source Code References**:
- Main parameter definitions: `/AntennaTracker/Parameters.cpp`
- Servo control implementation: `/AntennaTracker/servos.cpp`
- Tracking algorithms: `/AntennaTracker/tracking.cpp`
- PID controller library: `/libraries/AC_PID/`

**Community Support**:
- [ArduPilot Discuss Forum](https://discuss.ardupilot.org/)
- [AntennaTracker Category](https://discuss.ardupilot.org/c/ardutracker)

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**ArduPilot Version**: 4.6+  
**Maintainer**: ArduPilot Development Team

> **Safety Note**: Always test antenna tracker tuning in a safe environment where unexpected movements cannot cause injury or damage. Start with conservative parameters and gradually optimize for your specific hardware configuration.

