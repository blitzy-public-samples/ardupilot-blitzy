# ArduRover Tuning Guide

![ArduRover](https://img.shields.io/badge/ArduPilot-Rover-blue)
![Version](https://img.shields.io/badge/version-4.7+-green)

## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [Vehicle-Specific Configuration](#vehicle-specific-configuration)
- [Speed and Throttle Tuning](#speed-and-throttle-tuning)
- [Steering Tuning](#steering-tuning)
- [Navigation Parameter Tuning](#navigation-parameter-tuning)
- [Attitude Control Tuning](#attitude-control-tuning)
- [Position Control Tuning](#position-control-tuning)
- [Obstacle Avoidance Tuning](#obstacle-avoidance-tuning)
- [Path Planning Tuning](#path-planning-tuning)
- [Loiter and Circle Tuning](#loiter-and-circle-tuning)
- [Failsafe Configuration](#failsafe-configuration)
- [Frame-Specific Tuning](#frame-specific-tuning)
  - [Ackermann Steering](#ackermann-steering)
  - [Skid-Steer Vehicles](#skid-steer-vehicles)
  - [Omni-Directional Vehicles](#omni-directional-vehicles)
  - [Balance Bots](#balance-bots)
  - [Boats and Marine Vehicles](#boats-and-marine-vehicles)
  - [Sailboats](#sailboats)
- [Advanced Tuning](#advanced-tuning)
- [Performance Optimization](#performance-optimization)
- [Troubleshooting](#troubleshooting)

---

## Overview

This guide provides comprehensive tuning procedures for ArduRover ground vehicles. Proper tuning ensures optimal performance across different vehicle types, terrains, and operational requirements.

### Prerequisites

Before tuning, ensure:
- Vehicle is mechanically sound and properly assembled
- All sensors are calibrated (compass, accelerometer, RC radio)
- Correct frame class and type are configured
- Basic connectivity with ground control station is established
- Vehicle can arm and respond to manual control

### Tuning Philosophy

ArduRover tuning follows a layered approach:
1. **Foundation**: Configure vehicle frame and basic speed/throttle parameters
2. **Steering Control**: Tune low-level steering response
3. **Navigation**: Configure waypoint tracking and path following
4. **Advanced Features**: Enable obstacle avoidance, precision navigation

> **Safety Note**: Always test tuning changes in a safe, open area with manual mode as a fallback. Start with conservative values and gradually increase aggressiveness.

---

## Getting Started

### Step 1: Vehicle Frame Configuration

**Source**: `Rover/Parameters.cpp:426-430`

Configure your vehicle's basic frame type to ensure appropriate control algorithms are used:

```
FRAME_CLASS: Frame Class
  Values: 0=Undefined, 1=Rover, 2=Boat, 3=BalanceBot
  Default: 1
  
FRAME_TYPE: Frame Type (for specialized configurations)
  Values: 0=Undefined, 1=Omni3, 2=OmniX, 3=OmniPlus, 4=Omni3Mecanum
  Default: 0
  Requires reboot after change
```

**Configuration Guidelines**:
- **Standard Rover** (cars, tracked vehicles): `FRAME_CLASS=1`, `FRAME_TYPE=0`
- **Boat/Marine**: `FRAME_CLASS=2`, `FRAME_TYPE=0`
- **Balance Bot**: `FRAME_CLASS=3`, `FRAME_TYPE=0`
- **Omni-Directional**: `FRAME_CLASS=1`, `FRAME_TYPE=1-4` (depending on configuration)

### Step 2: Initial Mode Configuration

**Source**: `Rover/Parameters.cpp:29-34, 156-192`

```
INITIAL_MODE: Initial driving mode on boot
  CopyValuesFrom: MODE1
  Default: MANUAL (0)
  
MODE_CH: RC Channel for mode selection
  Default: Channel defined in MODE_CHANNEL constant
  
MODE1-MODE6: Driving modes for each switch position
  Values: 0=Manual, 1=Acro, 3=Steering, 4=Hold, 5=Loiter,
          6=Follow, 7=Simple, 8=Dock, 9=Circle, 10=Auto,
          11=RTL, 12=SmartRTL, 15=Guided
```

**Recommended Mode Setup**:
- MODE1: MANUAL (for direct control and emergencies)
- MODE2: STEERING (for assisted driving)
- MODE3: HOLD (for emergency stop)
- MODE4: AUTO (for mission execution)
- MODE5: RTL (return to launch)
- MODE6: LOITER (maintain position)

---

## Vehicle-Specific Configuration

### Pilot Input Configuration

**Source**: `Rover/Parameters.cpp:84-89`

```
PILOT_STEER_TYPE: Pilot input steering interpretation
  Values:
    0: Default (standard RC input)
    1: Two Paddles Input (tank-style controls)
    2: Direction reversed when backing up
    3: Direction unchanged when backing up
  Default: 0
```

**When to Use**:
- **Type 0 (Default)**: Standard RC car-style control (steering wheel + throttle)
- **Type 1 (Two Paddles)**: Tank/skid-steer vehicles with independent track control
- **Type 2 (Reversed)**: Vehicles where steering should reverse when backing up (most cars)
- **Type 3 (Unchanged)**: Boats, omni vehicles where direction remains constant

### Manual Mode Options

**Source**: `Rover/Parameters.cpp:608-613, 621-627`

```
MANUAL_OPTIONS: Manual mode specific options
  Bitmask: 0=Enable steering speed scaling
  Default: 0
  
MANUAL_STR_EXPO: Manual Steering Expo
  Description: Faster steering response at stick edges
  Values: 0=Disabled, 0.1=Very Low, 0.2=Low, 0.3=Medium,
          0.4=High, 0.5=Very High
  Range: -0.5 to 0.95
  Default: 0
```

**Tuning Recommendations**:
- **Speed Scaling** (bit 0): Enable for high-speed vehicles to reduce steering sensitivity at speed
- **Steering Expo**: Use 0.2-0.3 for precise slow-speed maneuvering with responsive high-speed turns

---

## Speed and Throttle Tuning

Speed and throttle parameters control how fast the vehicle moves and how aggressively it accelerates.

### Basic Speed Parameters

**Source**: `Rover/Parameters.cpp:65-82, 567-574`

```
CRUISE_SPEED: Target cruise speed in auto modes
  Units: m/s
  Range: 0 to 100 m/s
  Increment: 0.1 m/s
  Default: CRUISE_SPEED constant
  
CRUISE_THROTTLE: Base throttle percentage in auto
  Description: Initial throttle estimate to achieve CRUISE_SPEED
  Units: %
  Range: 0 to 100%
  Increment: 1%
  Default: 50%
  
SPEED_MAX: Maximum vehicle speed
  Description: Maximum speed at full throttle (0=auto-estimate)
  Units: m/s
  Range: 0 to 30 m/s
  Increment: 0.1 m/s
  Default: 0 (estimated from CRUISE_SPEED and CRUISE_THROTTLE)
```

### Tuning Procedure: Speed Parameters

#### Step 1: Determine Cruise Speed

1. **Drive the vehicle manually** at your desired cruising speed in an open area
2. **Note the speed** from telemetry (typically 2-5 m/s for small rovers, 5-15 m/s for larger vehicles)
3. **Set CRUISE_SPEED** to this value

**Example Values**:
- Small indoor rover: `CRUISE_SPEED = 1.0` (1 m/s)
- Medium outdoor rover: `CRUISE_SPEED = 3.0` (3 m/s)
- Large/fast rover: `CRUISE_SPEED = 8.0` (8 m/s)
- Boat: `CRUISE_SPEED = 2.0-5.0` (2-5 m/s depending on size)

#### Step 2: Set Cruise Throttle

1. While maintaining cruise speed manually, **note the throttle percentage** from telemetry
2. **Set CRUISE_THROTTLE** to this value
3. This provides the speed controller with a good starting estimate

**Typical Values**:
- Efficient vehicles: 40-50%
- Standard vehicles: 50-60%
- Heavy/inefficient vehicles: 60-75%

> **Note**: The speed controller will automatically adjust throttle to maintain CRUISE_SPEED, but starting with an accurate CRUISE_THROTTLE reduces oscillations during initial autonomous runs.

#### Step 3: Configure Maximum Speed (Optional)

If SPEED_MAX is left at 0, ArduRover estimates it based on CRUISE_SPEED and CRUISE_THROTTLE:
```
Estimated SPEED_MAX = CRUISE_SPEED / (CRUISE_THROTTLE / 100)
```

**Manual Configuration**:
1. **Drive at full throttle** in manual mode on flat ground
2. **Record maximum sustained speed** from telemetry
3. **Set SPEED_MAX** to 90% of this value (safety margin)

**Why Set SPEED_MAX?**
- Enables accurate speed control at high speeds
- Improves throttle controller performance
- Allows configuration of speed limits relative to maximum capability

### Speed Learning Feature

**Source**: `Rover/cruise_learn.cpp`

ArduRover can automatically learn optimal cruise throttle settings:

**Procedure**:
1. Assign an RC channel aux function to "LearnCruiseSpeed" (value 2)
2. Drive the vehicle at desired cruise speed in manual mode
3. Activate the learn function via the assigned switch
4. CRUISE_SPEED and CRUISE_THROTTLE are updated automatically

> **Best Practice**: Use cruise learning after mechanical changes or when operating in significantly different terrain/conditions.

### RTL Speed Configuration

**Source**: `Rover/Parameters.cpp:416-423`

```
RTL_SPEED: Return-to-Launch speed default
  Description: Speed used during RTL mode (0=use CRUISE_SPEED)
  Units: m/s
  Range: 0 to 100 m/s
  Increment: 0.1 m/s
  Default: 0 (use CRUISE_SPEED or WP_SPEED)
```

**Configuration Guidelines**:
- **RTL_SPEED = 0**: Use CRUISE_SPEED (most common)
- **RTL_SPEED > CRUISE_SPEED**: Return home faster than normal navigation
- **RTL_SPEED < CRUISE_SPEED**: Return home more cautiously (useful in cluttered environments)

---

## Steering Tuning

Steering control is handled by the attitude control library (ATC_ parameters) which manages both steering and throttle outputs.

### Steering Controller Architecture

**Source**: `Rover/Parameters.cpp:388-390`

The AR_AttitudeControl library provides:
- Steering rate control (deg/s)
- Steering angle control (for waypoint navigation)
- Speed control integration
- Coordinated turn limiting

All steering parameters use the `ATC_` prefix and are documented in:
`/libraries/APM_Control/AR_AttitudeControl.cpp`

### Key Steering Parameters

The primary steering tuning parameters control the PID loops for steering rate and angle:

```
ATC_STR_RAT_P: Steering Rate Proportional Gain
  Description: Primary gain for steering rate controller
  Effect: Higher values = more aggressive steering correction
  
ATC_STR_RAT_I: Steering Rate Integral Gain  
  Description: Corrects for sustained steering errors
  Effect: Higher values = faster correction of steady-state errors
  
ATC_STR_RAT_D: Steering Rate Derivative Gain
  Description: Damping term to reduce overshoot
  Effect: Higher values = more damping, less overshoot
  
ATC_STR_RAT_FF: Steering Rate Feed-Forward
  Description: Direct steering command based on desired rate
  Effect: Improves immediate response to steering commands
  
ATC_STR_RAT_FILT: Steering Rate Filter Frequency
  Units: Hz
  Description: Low-pass filter for rate controller
  Typical: 10-20 Hz (lower for slow vehicles, higher for fast)
  
ATC_STR_ANG_P: Steering Angle Proportional Gain
  Description: Gain for waypoint steering angle control
  Effect: Higher values = tighter waypoint tracking
```

### Steering Tuning Procedure

#### Step 1: Test in ACRO Mode

ACRO mode directly commands steering rate, making it ideal for testing basic steering response:

1. **Set conservative initial values**:
   ```
   ATC_STR_RAT_P = 0.2
   ATC_STR_RAT_I = 0.2
   ATC_STR_RAT_D = 0.0
   ATC_STR_RAT_FF = 0.0
   ```

2. **Drive in ACRO mode** with gentle steering inputs
3. **Observe steering response**:
   - **Sluggish/slow**: Increase `ATC_STR_RAT_P` by 0.1
   - **Oscillation/hunting**: Decrease `ATC_STR_RAT_P` by 0.05
   - **Overshoot on turns**: Increase `ATC_STR_RAT_D` by 0.01-0.02

4. **Iterate** until steering is responsive without oscillation

#### Step 2: Add Feed-Forward

Feed-forward provides immediate steering response:

1. **Start with FF = 0.1**
2. **Gradually increase** to 0.3-0.5
3. **Optimal when**: Steering responds immediately to commands with minimal lag
4. **Too high when**: Steering is twitchy or overshoots

> **Note**: For vehicles with significant mechanical steering lag (hydraulics, large vehicles), keep FF lower (0.1-0.3). For responsive vehicles (small electric steering), use higher values (0.4-0.7).

#### Step 3: Tune Integral Term

The integral term corrects steady-state errors:

1. **Drive in STEERING mode** and command a straight line
2. **Observe heading error** in telemetry
3. If vehicle drifts consistently to one side:
   - Increase `ATC_STR_RAT_I` by 0.1
   - Monitor for oscillation
4. **Typical final values**: 0.2-0.5

**Warning**: Excessive integral gain causes slow oscillations. If you see periodic weaving, reduce I term.

#### Step 4: Waypoint Steering (Angle Control)

After rate control is tuned, configure angle control for waypoint navigation:

1. **Start with** `ATC_STR_ANG_P = 2.0`
2. **Run a simple AUTO mission** with waypoints 20-50m apart
3. **Observe cornering behavior**:
   - **Cuts corners**: Increase `ATC_STR_ANG_P` by 0.5
   - **Oscillates around path**: Decrease `ATC_STR_ANG_P` by 0.5
   - **Overshoots waypoints**: Check WP_RADIUS and WP_OVERSHOOT

**Typical Values**:
- Slow vehicles (< 2 m/s): `ATC_STR_ANG_P = 1.5-2.5`
- Medium vehicles (2-5 m/s): `ATC_STR_ANG_P = 2.5-3.5`
- Fast vehicles (> 5 m/s): `ATC_STR_ANG_P = 3.5-4.5`

### Turn Radius Configuration

**Source**: `Rover/Parameters.cpp:392-399`

```
TURN_RADIUS: Turn radius of vehicle at low speeds
  Units: m
  Range: 0 to 10 m
  Increment: 0.1 m
  Default: 0.9 m
```

**Measuring Turn Radius**:
1. Drive in STEERING mode at 1-2 m/s
2. Command full right turn and maintain for complete circle
3. Measure circle diameter, divide by 2
4. Set TURN_RADIUS to this value

**Effect**:
- Used by path planner to determine achievable paths
- Affects pivot turn triggering on skid-steer vehicles
- Influences turn speed calculations

### ACRO Mode Turn Rate

**Source**: `Rover/Parameters.cpp:401-408`

```
ACRO_TURN_RATE: Maximum turn rate in ACRO mode
  Units: deg/s
  Range: 0 to 360 deg/s
  Increment: 1 deg/s
  Default: 180 deg/s
```

**Configuration**:
- Full RC stick deflection commands this turn rate
- Higher values = more aggressive maneuvering capability
- Limited by vehicle's physical steering capability

**Typical Values**:
- Large vehicles: 45-90 deg/s
- Medium vehicles: 90-180 deg/s
- Small/agile vehicles: 180-360 deg/s

---

## Navigation Parameter Tuning

Navigation parameters control waypoint tracking, path following, and autonomous behavior.

### Waypoint Navigation Parameters

**Source**: `Rover/Parameters.cpp:553-555`

The WP_Nav (Waypoint Navigation) library provides comprehensive path following:

```
Group: WP_ (AR_WPNav_OA library)
Path: /libraries/AR_WPNav/AR_WPNav.cpp
```

**Key WP_ Parameters**:

```
WP_SPEED: Waypoint navigation speed
  Description: Speed for waypoint tracking (0=use CRUISE_SPEED)
  Units: m/s
  Range: 0 to 100 m/s
  
WP_RADIUS: Waypoint acceptance radius
  Description: Distance from waypoint to consider it reached
  Units: m
  Range: 0 to 1000 m
  Typical: 0.5-2.0 m for precise navigation, 2.0-5.0 m for rough terrain
  
WP_OVERSHOOT: Waypoint overshoot distance
  Description: Distance past waypoint before initiating turn
  Units: m
  Range: 0 to 10 m
  Typical: 0.5-2.0 m (allows smooth cornering at speed)
  
WP_PIVOT_ANGLE: Pivot turn angle threshold
  Description: Angle error that triggers pivot turn (skid-steer only)
  Units: deg
  Range: 0 to 360 deg
  Default: 0 (disabled)
  Typical: 60-90 deg for skid-steer vehicles
  
WP_PIVOT_RATE: Pivot turn rate
  Description: Turn rate during pivot maneuver
  Units: deg/s
  Range: 0 to 360 deg/s
  Typical: 45-90 deg/s
```

### Navigation Tuning Procedure

#### Basic Waypoint Tracking

1. **Set initial waypoint parameters**:
   ```
   WP_SPEED = CRUISE_SPEED (or 0 to use CRUISE_SPEED)
   WP_RADIUS = 2.0 m (larger for initial testing)
   WP_OVERSHOOT = 1.0 m
   ```

2. **Create a simple box mission** (4 waypoints, 30m apart)

3. **Observe waypoint tracking**:
   - **Circles waypoints**: Decrease `WP_RADIUS`
   - **Sharp corners**: Increase `WP_OVERSHOOT`
   - **Smooth but cuts corners**: Increase `ATC_STR_ANG_P`

4. **Iterate for optimal balance** between precision and smooth motion

#### Precision Navigation

For applications requiring precise waypoint tracking (survey, payload delivery):

```
WP_RADIUS = 0.5-1.0 m
WP_OVERSHOOT = 0.3-0.5 m
ATC_STR_ANG_P = 3.5-4.5
CRUISE_SPEED = Reduce by 30-50% for precise tracking
```

#### High-Speed Navigation

For fast transit in open areas:

```
WP_RADIUS = 3.0-5.0 m
WP_OVERSHOOT = 2.0-3.0 m
ATC_STR_ANG_P = 2.5-3.5
CRUISE_SPEED = Maximum safe speed
```

### Pivot Turns (Skid-Steer Vehicles)

**Source**: `Rover/Parameters.cpp:659-673` (comments)

Pivot turns allow skid-steer vehicles to rotate in place for sharp corners:

**Configuration**:
1. **Enable pivot turns**: Set `WP_PIVOT_ANGLE` > 0 (typically 60-90 degrees)
2. **Set pivot rate**: `WP_PIVOT_RATE` = 45-90 deg/s
3. **Set small WP_RADIUS**: 0.5-1.5 m to ensure tight turns are detected

**How It Works**:
- When heading error exceeds `WP_PIVOT_ANGLE`, vehicle stops forward motion
- Executes in-place rotation at `WP_PIVOT_RATE`
- Resumes forward motion when aligned within threshold

**Best Practices**:
- Use for vehicles that cannot execute tight turns while moving
- Not recommended for Ackermann steering (cannot pivot)
- Essential for indoor navigation with skid-steer robots

---

## Attitude Control Tuning

The Attitude Control system manages steering rate and speed control through coordinated PID controllers.

### Speed Controller Parameters

**Source**: `Rover/Parameters.cpp:388-390` (ATC group)

```
Group: ATC_ (AR_AttitudeControl library)
Path: /libraries/APM_Control/AR_AttitudeControl.cpp
```

**Speed Control PID Parameters**:

```
ATC_SPEED_P: Speed Proportional Gain
  Description: Primary gain for speed controller
  Effect: Higher values = more aggressive throttle corrections
  Typical: 0.2-0.5
  
ATC_SPEED_I: Speed Integral Gain
  Description: Corrects for sustained speed errors
  Effect: Higher values = faster correction on hills/resistance
  Typical: 0.1-0.3
  
ATC_SPEED_D: Speed Derivative Gain
  Description: Damping to reduce speed oscillations
  Effect: Higher values = smoother speed control
  Typical: 0.0-0.1 (often zero for rovers)
  
ATC_SPEED_IMAX: Speed Integral Maximum
  Units: %
  Description: Limits integral windup
  Typical: 10-30%
```

### Speed Control Tuning Procedure

#### Step 1: Proportional Gain

1. **Set initial values**:
   ```
   ATC_SPEED_P = 0.2
   ATC_SPEED_I = 0.0 (disable initially)
   ATC_SPEED_D = 0.0
   ```

2. **Drive in AUTO mode** on flat ground at cruise speed

3. **Observe speed response**:
   - **Too slow to reach target**: Increase `ATC_SPEED_P` by 0.1
   - **Oscillates around target speed**: Decrease `ATC_SPEED_P` by 0.05
   - **Overshoots then settles**: Add `ATC_SPEED_D = 0.02`

4. **Typical final P values**: 0.3-0.7 depending on vehicle responsiveness

#### Step 2: Integral Gain

After P tuning, add integral correction for hills and resistance:

1. **Set** `ATC_SPEED_I = 0.1`
2. **Test on varied terrain** (inclines, rough ground)
3. **Observe**:
   - Speed maintained on hills → Good integral action
   - Slow oscillations → Reduce `ATC_SPEED_I`
   - Speed sags on hills → Increase `ATC_SPEED_I`

4. **Set integral limit**: `ATC_SPEED_IMAX = 20-30%`

#### Step 3: Throttle Slew Rate

```
ATC_THR_SLEWRATE: Throttle Slew Rate Limit
  Units: %/s
  Description: Maximum throttle change rate
  Range: 0-100 %/s (0=unlimited)
  Typical: 50-100 %/s
```

**Purpose**:
- Prevents abrupt throttle changes
- Reduces mechanical stress
- Improves battery efficiency
- Enhances passenger comfort

**Configuration**:
- **Gentle vehicles** (passengers, delicate payloads): 30-50 %/s
- **Standard vehicles**: 50-100 %/s
- **Aggressive/racing**: 100+ %/s or 0 (unlimited)

---

## Position Control Tuning

The Position Control system provides velocity and position control for advanced navigation modes.

### Position Control Parameters

**Source**: `Rover/Parameters.cpp:597-599`

```
Group: PSC (AR_PosControl library)
Path: /libraries/APM_Control/AR_PosControl.cpp
```

**Key PSC Parameters**:

```
PSC_VEL_P: Velocity Proportional Gain
  Description: Converts position error to velocity demand
  Typical: 0.1-0.3
  
PSC_VEL_I: Velocity Integral Gain
  Description: Corrects sustained position errors
  Typical: 0.05-0.15
  
PSC_VEL_D: Velocity Derivative Gain
  Description: Damping for velocity control
  Typical: 0.0-0.01
  
PSC_VEL_IMAX: Velocity Integral Maximum
  Units: m/s
  Description: Limits velocity correction from integral term
  Typical: 0.5-2.0 m/s
```

### Position Control Tuning Procedure

Position control is primarily used in LOITER and GUIDED modes:

#### Step 1: Test Loiter Performance

1. **Set initial conservative values**:
   ```
   PSC_VEL_P = 0.2
   PSC_VEL_I = 0.1
   PSC_VEL_D = 0.0
   ```

2. **Enter LOITER mode** at a fixed location

3. **Observe position holding**:
   - **Drifts from loiter point**: Increase `PSC_VEL_P`
   - **Oscillates around point**: Decrease `PSC_VEL_P`, add `PSC_VEL_D = 0.01`
   - **Slow to return when pushed**: Increase `PSC_VEL_I`

#### Step 2: Tune Loiter Speed Gain

**Source**: `Rover/Parameters.cpp:576-582`

```
LOIT_SPEED_GAIN: Loiter speed gain
  Description: Aggressiveness of loiter position correction
  Range: 0 to 5
  Increment: 0.01
  Default: 0.5
```

**Tuning**:
- **Higher values**: Vehicle returns to loiter point more aggressively
- **Lower values**: Gentler corrections, more drift tolerance
- **Typical**: 0.5-1.0 for most applications

#### Step 3: Loiter Radius

**Source**: `Rover/Parameters.cpp:509-516`

```
LOIT_RADIUS: Loiter radius
  Description: Distance from target within which vehicle may drift
  Units: m
  Range: 0 to 20 m
  Default: 2 m
```

**Configuration**:
- **Precision applications**: 0.5-1.0 m
- **General use**: 2.0-3.0 m
- **Boats/marine** (current/wind): 3.0-5.0 m

### Loiter Type Configuration

**Source**: `Rover/Parameters.cpp:478-483`

```
LOIT_TYPE: Loiter type
  Values:
    0: Forward or reverse to target point
    1: Always face bow towards target point
    2: Always face stern towards target point
  Default: 0
```

**Use Cases**:
- **Type 0**: Standard rovers (most efficient)
- **Type 1**: Boats/vehicles with directional sensors pointing forward
- **Type 2**: Vehicles with rear-facing sensors or equipment

---

## Obstacle Avoidance Tuning

ArduRover supports multiple obstacle avoidance systems for safe autonomous operation.

### Avoidance System Architecture

**Source**: `Rover/Parameters.cpp:438-442`

```
Group: AVOID_ (AC_Avoid library)
Path: /libraries/AC_Avoidance/AC_Avoid.cpp
```

**Requires**:
- Proximity sensors (PRX_) - lidar, radar, or rangefinders
- Or simple rangefinder (RNGFND_) for basic obstacle detection

### Proximity Sensor Configuration

**Source**: `Rover/Parameters.cpp:432-436`

```
Group: PRX (AP_Proximity library)
Path: /libraries/AP_Proximity/AP_Proximity.cpp
```

**Supported Proximity Sensors**:
- 360-degree lidars (Lightware SF40C, RPLidar, etc.)
- Terraranger Tower/Tower EVO
- Multiple rangefinders configured in radial pattern

**Basic Configuration**:

```
PRX_TYPE: Proximity sensor type
  Values: 0=None, 3=RangeFinder, 4=RPLidar, 7=TeraRanger, 
          10=SITL, 14=LightwareSF45B
  
PRX_ORIENT: Sensor orientation
  Description: Direction sensor is facing
  
PRX_YAW_CORR: Yaw correction
  Description: Offset angle for sensor mounting
  
PRX_IGN_ANG1-4: Ignore angles
  Description: Angular sectors to ignore (for vehicle structure)
  
PRX_IGN_WID1-4: Ignore widths
  Description: Width of ignored sectors
```

### Avoidance Parameters

**Key AVOID_ Parameters**:

```
AVOID_ENABLE: Avoidance enable
  Bitmask:
    0: UseProximitySensor
    1: UseBeaconFence
    2: StopAtBeaconFence
    3: EnableSmartRTL
    6: EnableObjectAvoidance
  Default: 0 (disabled)
  
AVOID_MARGIN: Avoidance margin
  Description: Minimum distance to maintain from obstacles
  Units: m
  Range: 0.1-10 m
  Typical: 1.0-3.0 m
  
AVOID_BEHAVE: Avoidance behavior
  Values:
    0: Slide (try to go around)
    1: Stop (halt before obstacle)
  Default: 0
  
AVOID_ACCEL_MAX: Maximum avoidance acceleration
  Description: Maximum lateral acceleration for avoidance
  Units: m/s²
  Range: 0-10 m/s²
  Typical: 1.0-3.0 m/s²
  
AVOID_BACKUP_SPD: Backup speed
  Description: Speed when backing away from obstacle
  Units: m/s
  Range: 0-2 m/s
  Typical: 0.3-0.5 m/s
```

### Avoidance Tuning Procedure

#### Step 1: Configure Proximity Sensor

1. **Set sensor type** based on your hardware
2. **Configure orientation and mounting** (PRX_ORIENT, PRX_YAW_CORR)
3. **Define ignore zones** for vehicle body (PRX_IGN_ANG, PRX_IGN_WID)
4. **Verify sensor data** in ground station (should show obstacles correctly)

#### Step 2: Enable Basic Avoidance

1. **Enable proximity sensor avoidance**:
   ```
   AVOID_ENABLE = 1 (bit 0 set)
   ```

2. **Set conservative margin**:
   ```
   AVOID_MARGIN = 2.0 m
   ```

3. **Choose behavior**:
   ```
   AVOID_BEHAVE = 1 (Stop) for initial testing
   AVOID_BEHAVE = 0 (Slide) for missions after validation
   ```

4. **Test in AUTO mode** approaching an obstacle:
   - Vehicle should stop at AVOID_MARGIN distance
   - No collision should occur
   - GCS should show avoidance messages

#### Step 3: Tune Avoidance Aggressiveness

For "Slide" behavior (trying to navigate around obstacles):

1. **Set maximum avoidance acceleration**:
   ```
   AVOID_ACCEL_MAX = 1.0 m/s² (gentle)
   AVOID_ACCEL_MAX = 2.0-3.0 m/s² (moderate)
   ```

2. **Adjust margin based on vehicle speed**:
   - Slow vehicles (< 2 m/s): AVOID_MARGIN = 1.0-1.5 m
   - Fast vehicles (> 5 m/s): AVOID_MARGIN = 3.0-5.0 m

3. **Configure backup capability**:
   ```
   AVOID_BACKUP_SPD = 0.5 m/s
   ```
   Allows vehicle to reverse if cornered

#### Step 4: Simple Stop Behavior

For simple stop-on-obstacle behavior (without path planning):

**Configuration**:
```
AVOID_ENABLE = 1
AVOID_BEHAVE = 1 (Stop)
AVOID_MARGIN = 2.0 m
```

**How It Works**:
- Vehicle stops when obstacle detected within AVOID_MARGIN
- Remains stopped until obstacle clears or mode changed
- Simple but effective for cautious operation

---

## Path Planning Tuning

Advanced path planning uses the Object Avoidance (OA) system to navigate around detected obstacles.

### Path Planner Configuration

**Source**: `Rover/Parameters.cpp:561-565`

```
Group: OA_ (AP_OAPathPlanner library)
Path: /libraries/AC_Avoidance/AP_OAPathPlanner.cpp
Requires: AP_OAPATHPLANNER_ENABLED
```

**Key OA_ Parameters**:

```
OA_TYPE: Object Avoidance Path Planning algorithm
  Values:
    0: Disabled
    1: BendyRuler
    2: Dijkstra
  Default: 0
  
OA_MARGIN_MAX: Maximum margin from obstacles
  Description: Distance path planner attempts to maintain
  Units: m
  Range: 0.1-10 m
  Typical: 2.0-5.0 m
  
OA_LOOKAHEAD: Path planning lookahead distance
  Description: How far ahead to plan path
  Units: m
  Range: 1-100 m
  Typical: 15-30 m
```

### Path Planning Algorithms

#### BendyRuler Algorithm

**Best For**: Open environments with scattered obstacles

**How It Works**:
- Creates multiple candidate paths curving around obstacles
- Selects path closest to direct route that maintains clearance
- Fast, efficient for real-time operation

**Configuration**:
```
OA_TYPE = 1
OA_MARGIN_MAX = 2.0-3.0 m
OA_LOOKAHEAD = 15-20 m
```

**Tuning**:
- **OA_MARGIN_MAX**: Larger values create wider berth around obstacles
- **OA_LOOKAHEAD**: Must be > vehicle stopping distance at cruise speed

#### Dijkstra Algorithm

**Best For**: Complex environments with many obstacles

**How It Works**:
- Creates grid-based representation of environment
- Computes optimal path using Dijkstra graph search
- More computationally intensive but handles complex scenarios

**Configuration**:
```
OA_TYPE = 2
OA_MARGIN_MAX = 2.5-4.0 m
OA_LOOKAHEAD = 20-40 m
```

**Additional Dijkstra Parameters**:
```
OA_DIJKSTRA_SIZE: Grid size
  Description: Size of planning grid
  Larger = more detail but slower computation
  
OA_DIJKSTRA_RES: Grid resolution
  Description: Size of each grid cell
  Typical: 0.5-1.0 m
```

### Integrated Avoidance System

For complete obstacle avoidance with path planning:

**Configuration**:
```
PRX_TYPE = <your sensor type>
AVOID_ENABLE = 65 (bits 0 and 6: proximity + object avoidance)
AVOID_MARGIN = 1.5 m
OA_TYPE = 1 or 2 (BendyRuler or Dijkstra)
OA_MARGIN_MAX = 2.5 m
OA_LOOKAHEAD = 20 m
```

**System Interaction**:
1. **Proximity sensor** detects obstacles
2. **AVOID system** ensures minimum safe distance
3. **OA path planner** computes route around obstacles
4. **WP_NAV** executes planned path
5. **ATC** controls steering and speed

**Tuning Priorities**:
- **Safety first**: Start with large margins (AVOID_MARGIN, OA_MARGIN_MAX)
- **Test incrementally**: Reduce margins as confidence builds
- **Monitor performance**: Watch CPU load and planning latency

---

## Loiter and Circle Tuning

### Circle Mode Parameters

**Source**: `Rover/Parameters.cpp:638-640`

```
Group: CIRC (ModeCircle class)
Path: Rover/mode_circle.cpp
```

**Circle Mode Configuration**:

```
CIRC_RADIUS: Circle mode radius
  Description: Radius of circle to drive
  Units: m
  Range: 0-1000 m
  Typical: 5-20 m depending on application
  
CIRC_SPEED: Circle mode speed
  Description: Speed while circling (0=use WP_SPEED/CRUISE_SPEED)
  Units: m/s
  Range: 0-100 m/s
  
CIRC_DIR: Circle direction
  Values: 0=Clockwise, 1=Counter-Clockwise
  Default: 0 (clockwise)
```

**Use Cases**:
- Aerial photography platform (following drone)
- Surveillance of fixed point
- Testing vehicle turning characteristics
- Sensor calibration procedures

### Tuning Circle Performance

#### Step 1: Set Basic Circle Parameters

1. **Choose radius** based on application:
   - Small circles (testing): 5-10 m
   - Medium circles (observation): 10-30 m
   - Large circles (perimeter): 30-100 m

2. **Set appropriate speed**:
   - Radius/speed ratio should allow comfortable turning
   - Centripetal acceleration = v²/r
   - Keep acceleration reasonable (< 2 m/s²)

**Example**:
```
For CIRC_RADIUS = 10 m:
- CIRC_SPEED = 2.0 m/s → acceleration = 0.4 m/s² (comfortable)
- CIRC_SPEED = 5.0 m/s → acceleration = 2.5 m/s² (aggressive)
```

#### Step 2: Verify Turning Capability

**Check**: Can your vehicle execute the circle at specified speed?

**Test Procedure**:
1. Enter CIRCLE mode
2. Observe actual circle radius from GPS track
3. Compare to CIRC_RADIUS setting

**If actual radius > CIRC_RADIUS**:
- Vehicle cannot turn tight enough at specified speed
- **Solutions**:
  - Reduce CIRC_SPEED
  - Increase CIRC_RADIUS  
  - Improve steering tuning (ATC_STR_ANG_P)

#### Step 3: Optimize for Smooth Circles

**Factors Affecting Circle Quality**:
- **GPS update rate**: Higher is better (5-10 Hz)
- **Steering response**: Must be well-tuned (see Steering Tuning section)
- **Speed control**: Should maintain constant speed
- **Turn radius**: Vehicle's physical TURN_RADIUS must be < CIRC_RADIUS

**For Perfect Circles**:
```
ATC_STR_ANG_P = 3.0-4.0 (good waypoint tracking)
WP_RADIUS = 1.0-2.0 m (tight waypoint acceptance)
TURN_RADIUS = accurately measured
GPS = high update rate, good antenna placement
```

---

## Failsafe Configuration

Proper failsafe configuration is critical for safe autonomous operation.

### Failsafe Parameters

**Source**: `Rover/Parameters.cpp:91-148`

#### Radio Failsafe

```
FS_THR_ENABLE: Throttle Failsafe Enable
  Values:
    0: Disabled
    1: Enabled
    2: Enabled Continue with Mission in Auto
  Default: Enabled (1)
  
FS_THR_VALUE: Throttle Failsafe Value
  Description: PWM value below which failsafe triggers
  Range: 910-1100 PWM
  Default: 910 PWM
  Typical: 900-950 PWM (below normal RC minimum)
  
FS_TIMEOUT: Failsafe timeout
  Description: Duration of failsafe condition before action
  Units: s
  Range: 1-100 s
  Default: 1.5 s
  Typical: 1.0-3.0 s
  
FS_ACTION: Failsafe Action
  Values:
    0: Nothing (not recommended)
    1: RTL (Return to Launch)
    2: Hold (stop and maintain position)
    3: SmartRTL or RTL
    4: SmartRTL or Hold
    5: Terminate (emergency disarm - DANGEROUS)
    6: Loiter or Hold (Hold if no GPS)
  Default: Hold (2)
  Recommended: RTL (1) or SmartRTL or RTL (3)
```

#### GCS Failsafe

```
FS_GCS_ENABLE: GCS failsafe enable
  Description: Action when telemetry heartbeat lost
  Values:
    0: Disabled
    1: Enabled
    2: Enabled Continue with Mission in Auto
  Default: Disabled (0)
  Recommended: Enabled (1) for autonomous missions
  
FS_GCS_TIMEOUT: GCS failsafe timeout
  Description: Time before GCS failsafe triggers
  Units: s
  Range: 2-120 s
  Default: 5 s
  Typical: 5-10 s
```

#### EKF Failsafe

```
FS_EKF_ACTION: EKF Failsafe Action
  Description: Action when navigation solution degrades
  Values:
    0: Disabled (not recommended)
    1: Hold (stop immediately)
    2: ReportOnly (warning but continue)
  Default: Hold (1)
  
FS_EKF_THRESH: EKF failsafe variance threshold
  Description: Acceptable navigation variance
  Values:
    0.6: Strict (tight requirements)
    0.8: Default (balanced)
    1.0: Relaxed (permissive)
  Default: 0.8
```

#### Crash Detection

```
FS_CRASH_CHECK: Crash check action
  Description: Action when crash/tip detected
  Values:
    0: Disabled
    1: Hold (stop but remain armed)
    2: HoldAndDisarm (stop and disarm)
  Default: Disabled (0)
  Recommended: HoldAndDisarm (2) for autonomous operation
  
CRASH_ANGLE: Crash Angle
  Description: Pitch/roll angle triggering crash detection
  Units: deg
  Range: 0-60 deg
  Default: 0 (disabled)
  Typical: 30-45 deg (enables crash detection)
```

### Failsafe Configuration Best Practices

#### Recommended Failsafe Setup

**For Manual/Line-of-Sight Operation**:
```
FS_THR_ENABLE = 1 (Enabled)
FS_THR_VALUE = 925 PWM
FS_ACTION = 2 (Hold)
FS_TIMEOUT = 1.5 s
FS_GCS_ENABLE = 0 (Disabled)
FS_CRASH_CHECK = 1 (Hold)
```

**For Autonomous Missions (Long Range)**:
```
FS_THR_ENABLE = 2 (Continue Mission)
FS_THR_VALUE = 925 PWM
FS_ACTION = 1 (RTL)
FS_TIMEOUT = 2.0 s
FS_GCS_ENABLE = 1 (Enabled)
FS_GCS_TIMEOUT = 10 s
FS_CRASH_CHECK = 2 (HoldAndDisarm)
CRASH_ANGLE = 35 deg
FS_EKF_ACTION = 1 (Hold)
FS_EKF_THRESH = 0.8
```

#### Testing Failsafes

**Critical**: Always test failsafes before autonomous operation!

**Radio Failsafe Test**:
1. Arm vehicle in safe area
2. Enter STEERING or AUTO mode
3. **Turn off transmitter** (or enable trainer/off mode)
4. Verify vehicle executes FS_ACTION
5. Restore transmitter, verify control recovery

**GCS Failsafe Test** (if using telemetry):
1. Arm vehicle in safe area
2. Enter AUTO mode
3. **Disconnect telemetry** or block MAVLink heartbeat
4. Verify vehicle executes FS_ACTION after FS_GCS_TIMEOUT
5. Reconnect and verify recovery

**EKF Failsafe Test** (simulation):
- Best tested in SITL with GPS glitches injected
- Monitor EKF variance in telemetry logs
- Verify appropriate response when variance exceeds threshold

#### Failsafe Options

**Source**: `Rover/Parameters.cpp:584-589`

```
FS_OPTIONS: Failsafe Options
  Bitmask:
    0: Failsafe enabled in Hold mode
  Default: 0
```

**Bit 0 Effect**:
- **Disabled (0)**: Failsafes ignored when in HOLD mode
- **Enabled (1)**: Failsafes active even in HOLD
- **Use Case**: Enable if you want failsafe actions even after manual HOLD command

---

## Frame-Specific Tuning

Different vehicle types require specialized tuning approaches.

### Ackermann Steering

**Frame Configuration**:
```
FRAME_CLASS = 1 (Rover)
FRAME_TYPE = 0 (Undefined/standard)
PILOT_STEER_TYPE = 2 (Direction reversed when backing)
```

**Characteristics**:
- Front wheel steering, rear wheel drive (like a car)
- Cannot pivot turn in place
- Minimum turn radius determined by steering geometry
- Backing up typically reverses steering sense

**Tuning Priorities**:

#### 1. Accurate Turn Radius

**Critical for Ackermann vehicles**:
```
TURN_RADIUS = <measured minimum turn radius>
```

**Measurement Procedure**:
1. Find open area with at least 20m diameter
2. Drive at 1-2 m/s in STEERING mode
3. Command full lock turn (full stick deflection)
4. Complete a full circle
5. Measure diameter, divide by 2
6. Set TURN_RADIUS to this value

**Typical Values**:
- Small RC car: 0.5-1.0 m
- Large rover: 1.5-3.0 m
- Car-sized vehicle: 3.0-6.0 m

#### 2. Steering Calibration

Ensure steering servo endpoints are correctly configured:

```
SERVOx_MIN = PWM at full left
SERVOx_MAX = PWM at full right
SERVOx_TRIM = PWM for straight ahead
```

**Verification**:
- Full left stick = full left steering
- Full right stick = full right steering
- Centered stick = straight wheels

#### 3. Reverse Steering Behavior

**Most Ackermann vehicles need**:
```
PILOT_STEER_TYPE = 2 (Direction reversed when backing up)
```

**Why**: When backing up, turning the front wheels right makes the rear move left (relative to direction of travel). This mode compensates automatically.

#### 4. Waypoint Navigation

Ackermann vehicles require larger waypoint acceptance:

```
WP_RADIUS = 2.0-5.0 m (larger than skid-steer)
WP_OVERSHOOT = 2.0-4.0 m (allow smooth cornering)
CRUISE_SPEED = Moderate (excessive speed causes corner cutting)
```

**Path Planning**:
- Enable generous turn margins
- Plan waypoints accounting for turn radius
- Avoid sharp corners in mission planning

### Skid-Steer Vehicles

**Frame Configuration**:
```
FRAME_CLASS = 1 (Rover)
FRAME_TYPE = 0 (Undefined/standard)
PILOT_STEER_TYPE = 0 or 1 (Standard or Two Paddles)
```

**Characteristics**:
- Independent left/right track or wheel control
- Can pivot turn in place
- No Ackermann constraints
- May have significant track slippage

**Tuning Priorities**:

#### 1. Motor Configuration

Configure motor outputs correctly:

```
For skid-steer:
MOT_THR_MIN = Minimum throttle (typically 0 or 5%)
MOT_THR_MAX = Maximum throttle (typically 100%)
MOT_SLEWRATE = Throttle slew rate (0-100 %/s)
MOT_VEC_ANGLEMAX = Not used for skid-steer
```

Verify motor mixing:
- Forward throttle → both motors forward equally
- Right steering → left motor faster than right
- Left steering → right motor faster than left
- Pivot right → left forward, right reverse

#### 2. Enable Pivot Turns

**Configuration**:
```
WP_PIVOT_ANGLE = 60-90 deg
WP_PIVOT_RATE = 45-90 deg/s
WP_RADIUS = 1.0-2.0 m (tight for pivot activation)
```

**Effect**:
- When heading error > WP_PIVOT_ANGLE, vehicle stops and rotates
- Enables precise navigation in tight spaces
- Essential for indoor/warehouse operation

#### 3. Steering Tuning

Skid-steer vehicles often need higher steering gains:

```
ATC_STR_RAT_P = 0.4-0.8 (higher than Ackermann)
ATC_STR_RAT_I = 0.2-0.4
ATC_STR_RAT_FF = 0.3-0.5
```

**Why Higher Gains**:
- Steering achieved through differential drive, not dedicated servo
- More resistance from track friction
- Immediate response needed to prevent drift

#### 4. Wheel Encoder Integration

**Source**: `Rover/Parameters.cpp:384-393`

For accurate odometry:

```
Group: WENC (AP_WheelEncoder library)
Path: /libraries/AP_WheelEncoder/AP_WheelEncoder.cpp

Group: WRC (AP_WheelRateControl library)  
Path: /libraries/AP_WheelEncoder/AP_WheelRateControl.cpp
```

**Wheel Encoder Configuration**:
```
WENC_TYPE = <encoder type>
WENC_CPR = Counts per revolution
WENC_RADIUS = Wheel radius in meters
WENC_POS_X/Y = Encoder position relative to center of gravity
```

**Benefits**:
- Improved position estimation (especially without GPS)
- Better straight-line tracking
- Slip detection and compensation

**Wheel Rate Control**:
```
WRC_SPEED_FF: Speed feed-forward gain
WRC_SPEED_P: Speed proportional gain
WRC_SPEED_I: Speed integral gain
WRC_SPEED_IMAX: Integral maximum
```

Enable for direct wheel speed control when encoders are available.

### Omni-Directional Vehicles

**Frame Configuration**:
```
FRAME_CLASS = 1 (Rover)
FRAME_TYPE = 1-4 (Omni3, OmniX, OmniPlus, Omni3Mecanum)
PILOT_STEER_TYPE = 3 (Direction unchanged when backing)
```

**Supported Configurations**:
- **Omni3 (Type 1)**: Three omniwheels at 120° spacing
- **OmniX (Type 2)**: Four omniwheels in X configuration (45° angles)
- **OmniPlus (Type 3)**: Four omniwheels in + configuration (90° angles)
- **Omni3Mecanum (Type 4)**: Three mecanum wheels at 120° spacing

**Characteristics**:
- Can move in any direction without rotating
- Can rotate while moving
- No minimum turn radius
- Complex motor mixing required

**Tuning Priorities**:

#### 1. Motor Layout Configuration

**Critical**: Motor outputs must match physical layout

**OmniX Example** (4 motors at 45° angles):
```
SERVO1_FUNCTION = 33 (Motor1 - Front Right)
SERVO2_FUNCTION = 34 (Motor2 - Rear Left)
SERVO3_FUNCTION = 35 (Motor3 - Rear Right)
SERVO4_FUNCTION = 36 (Motor4 - Front Left)
```

Verify mixing:
- Forward: All motors same direction
- Strafe right: Front-right/rear-left forward, others reverse
- Rotate: Alternating motor directions

#### 2. Steering and Throttle Balance

Omniwheels require balanced control:

```
ATC_STR_RAT_P = Start conservative: 0.2-0.3
ATC_STR_RAT_FF = 0.1-0.3 (reduce if twitchy)
ATC_SPEED_P = 0.3-0.5
```

**Test Procedure**:
1. Test forward/backward motion (all wheels equal)
2. Test strafing (lateral motion)
3. Test rotation in place
4. Test combined motions (forward while rotating)

#### 3. Turn Radius

Omni vehicles have TURN_RADIUS = 0 capability:

```
TURN_RADIUS = 0.1-0.5 m (very small)
```

**Effect**:
- Path planner can use tighter turns
- Waypoint radius can be smaller
- Enables precise positioning

#### 4. Advanced Navigation

Omniwheels excel at precision tasks:

```
WP_RADIUS = 0.5-1.0 m (tighter than standard rovers)
LOIT_RADIUS = 0.5-1.0 m (precise loitering)
PSC_VEL_P = 0.3-0.5 (higher for responsive position hold)
```

### Balance Bots

**Frame Configuration**:
```
FRAME_CLASS = 3 (BalanceBot)
FRAME_TYPE = 0
```

**Characteristics**:
- Two-wheeled self-balancing vehicle
- Inherently unstable (requires active control)
- Pitch angle directly controls acceleration
- Cannot stop completely (must maintain balance)

**Tuning Priorities**:

#### 1. Balance Control Parameters

**Source**: `Rover/Parameters.cpp:446-453, 535-542`

```
BAL_PITCH_MAX: Maximum pitch angle at 100% throttle
  Units: deg
  Range: 0-15 deg
  Default: 10 deg
  Description: Maximum pitch angle vehicle will achieve
  
BAL_PITCH_TRIM: Pitch trim angle for balancing
  Units: deg
  Range: -2 to +2 deg
  Default: 0 deg
  Description: Offset to compensate for CG position
```

**Tuning Procedure**:

1. **Set BAL_PITCH_MAX**:
   - Start with 5 degrees (conservative)
   - Gradually increase to 10-12 degrees
   - Higher values = faster acceleration
   - Too high = aggressive, potentially unstable

2. **Determine BAL_PITCH_TRIM**:
   - Arm vehicle and attempt to balance
   - If leans forward at rest → BAL_PITCH_TRIM = +0.5° to +1.5°
   - If leans backward at rest → BAL_PITCH_TRIM = -0.5° to -1.5°
   - Iterate until vehicle balances with minimal motor activity

#### 2. Speed Control

Balance bots use pitch angle to control speed:

```
ATC_SPEED_P = 0.2-1.0 (may need higher than standard rovers)
ATC_SPEED_I = 0.1-0.3
ATC_SPEED_IMAX = 5-15% (constrained to limit pitch angle)
```

**Balance Bot Speed Logic**:
- Desired speed → target pitch angle (via ATC_SPEED)
- Pitch angle → motor drive (balancing algorithm)
- Faster speed = more pitch angle (up to BAL_PITCH_MAX)

#### 3. Balance Algorithm Tuning

Balance control uses internal PID loops (not user-configurable for safety):

**What You Can Tune**:
- Sensor calibration (accelerometer, gyro) - **Critical!**
- BAL_PITCH_MAX (aggressiveness)
- BAL_PITCH_TRIM (static balance point)
- Speed controller (ATC_SPEED) parameters

**What You Cannot Tune**:
- Internal balance PID gains (hardcoded for stability)

#### 4. Operational Considerations

**Balance Bot Limitations**:
- Cannot truly stop (must maintain balance)
- HOLD mode maintains minimal speed/oscillation
- Not suitable for steep slopes
- Requires smooth, flat surfaces

**Configuration for Balance Bots**:
```
CRUISE_SPEED = 1.0-3.0 m/s (moderate speeds)
WP_RADIUS = 1.0-2.0 m (don't expect precision stops)
FS_CRASH_CHECK = 2 (essential - detects fall/tip)
CRASH_ANGLE = 30 deg (trigger failsafe if tilted too much)
```

### Boats and Marine Vehicles

**Frame Configuration**:
```
FRAME_CLASS = 2 (Boat)
FRAME_TYPE = 0
PILOT_STEER_TYPE = 3 (Direction unchanged when backing)
```

**Characteristics**:
- Operates on water surface
- Subject to currents and wind
- Momentum and inertia significant
- Cannot stop instantly
- No traction (relies on propeller thrust)

**Tuning Priorities**:

#### 1. Speed and Throttle

Boats have different speed characteristics:

```
CRUISE_SPEED = 2.0-10.0 m/s (depends on boat size/type)
CRUISE_THROTTLE = 40-70% (hulls have high drag at low speed)
SPEED_MAX = Accurately measure maximum speed
ATC_THR_SLEWRATE = 30-50 %/s (gentle for fuel efficiency)
```

**Speed Tuning**:
- Boats often have non-linear throttle/speed relationships
- Planing hulls have distinct displacement vs. planing speeds
- Speed controller may need more aggressive I term for current compensation

```
ATC_SPEED_P = 0.3-0.5
ATC_SPEED_I = 0.2-0.5 (higher to counter current)
ATC_SPEED_IMAX = 30-50% (allow significant integral correction)
```

#### 2. Steering Control

Marine steering characteristics:

```
ATC_STR_RAT_P = 0.2-0.4 (lower than land rovers)
ATC_STR_RAT_I = 0.1-0.3
ATC_STR_RAT_FF = 0.2-0.4
ATC_STR_ANG_P = 1.5-2.5 (lower for momentum/drift)
```

**Why Different**:
- Boats have significant momentum
- Steering effectiveness depends on forward speed
- Current/wind introduce disturbances
- Rudder has delayed response

#### 3. Navigation Parameters

Boats need different navigation settings:

```
TURN_RADIUS = 2.0-10.0 m (larger than comparable land vehicle)
WP_RADIUS = 3.0-10.0 m (wider acceptance due to drift)
WP_OVERSHOOT = 2.0-5.0 m (allow momentum)
LOIT_RADIUS = 5.0-15.0 m (current/wind causes drift)
LOIT_TYPE = 1 or 2 (orient bow or stern toward loiter point)
```

**Current Compensation**:
Position controller integral terms help counter steady current:

```
PSC_VEL_I = 0.2-0.5 (higher for strong currents)
PSC_VEL_IMAX = 1.0-3.0 m/s
```

#### 4. Loiter Behavior

**Source**: `Rover/Parameters.cpp:478-483`

Boats often benefit from oriented loitering:

```
LOIT_TYPE = 1 (Face bow toward loiter point)
```

**Why**:
- Bow-first provides better control in current
- Camera/sensors often mounted facing forward
- More efficient motor use

**Alternative**:
```
LOIT_TYPE = 2 (Face stern toward loiter point)
```
Use if rear-mounted equipment or better stern handling.

#### 5. Depth and Underwater Obstacle Avoidance

For boats with downward-facing rangefinder:

```
Group: RNGFND
RNGFND_TYPE = <downward facing rangefinder>
RNGFND_MIN_CM = 20 cm (minimum depth alarm)
```

Configure depth monitoring for shallow water warning.

### Sailboats

**Frame Configuration**:
```
FRAME_CLASS = 2 (Boat)
FRAME_TYPE = 0
```

**Additional Configuration**:
**Source**: `Rover/Parameters.cpp:557-559, 518-520`

```
Group: SAIL_ (Sailboat class)
Path: Rover/sailboat.cpp

Group: WNDVN_ (AP_WindVane library)
Path: /libraries/AP_WindVane/AP_WindVane.cpp
```

**Characteristics**:
- Primary propulsion from wind via sails
- Optional motor for calm conditions
- Steering via rudder
- Must account for wind direction
- Cannot sail directly into wind (no-go zone)
- Requires tacking maneuvers

**Tuning Priorities**:

#### 1. Wind Vane Configuration

**Essential for sailboat operation**:

```
WNDVN_TYPE: Wind vane sensor type
  Values:
    0: None
    1: Home heading (SITL only)
    2: RC input (manual indication)
    3: Analog (wind vane potentiometer)
    4: Modern Devices (wind sensor)
    10: SITL true wind
    11: NMEA (from dedicated wind instrument)
  
WNDVN_DIR_PIN: Analog pin for wind direction
WNDVN_DIR_V_MIN: Voltage at minimum angle
WNDVN_DIR_V_MAX: Voltage at maximum angle
WNDVN_DIR_OFS: Wind direction offset
WNDVN_SPEED_TYPE: Wind speed sensor type
WNDVN_SPEED_PIN: Wind speed sensor pin
```

**Calibration Procedure**:
1. Install wind vane with known orientation
2. Point vehicle north, rotate vane to indicate north wind
3. Record voltage/reading
4. Repeat for known wind angles
5. Set WNDVN_DIR parameters to match

#### 2. Sail Control

```
SAIL_ANGLE_MIN: Minimum sail angle (tight sheeting)
  Units: deg
  Description: Sail angle for close-hauled sailing
  Typical: 0-20 deg
  
SAIL_ANGLE_MAX: Maximum sail angle (loose sheeting)
  Units: deg
  Description: Sail angle for running downwind
  Typical: 80-90 deg
  
SAIL_ANGLE_IDEAL: Ideal sail angle
  Units: deg
  Description: Optimal angle for best lift/drag
  Typical: 25-35 deg
  
SAIL_HEEL_MAX: Maximum heel angle
  Units: deg
  Description: Heel angle that triggers sail easing
  Typical: 20-35 deg (prevents capsizing)
```

#### 3. Tacking Configuration

Sailboats cannot sail directly upwind and must tack:

```
SAIL_NO_GO_ANGLE: No-go zone angle
  Units: deg
  Description: Angular range upwind where sailing is impossible
  Typical: 45-60 deg either side of wind
  
SAIL_TACK_ANGLE: Tacking angle
  Units: deg
  Description: Angle off wind when close-hauled
  Typical: 45-50 deg
```

**How Tacking Works**:
1. Waypoint is upwind within no-go zone
2. Sailboat steers to SAIL_TACK_ANGLE off wind
3. Sails on this heading for distance/time
4. Tacks (turns through wind) to opposite SAIL_TACK_ANGLE
5. Repeats until waypoint is reachable

#### 4. Motor Assist

Most sailboats have auxiliary motor:

```
SAIL_MOTOR_IDLE: Motor idle speed
  Units: %
  Description: Motor throttle when not in use
  Typical: 0% (off)
  
SAIL_MOTOR_ASSIST: Motor assist enable
  Bitmask:
    0: Enable motor assist in no-wind
    1: Enable motor assist upwind
    2: Enable motor in AUTO always
  
SAIL_WNDSPD_MIN: Minimum wind speed for sailing
  Units: m/s
  Description: Below this wind speed, use motor
  Typical: 1.0-2.0 m/s
```

**Configuration Examples**:

**Pure Sailing** (no motor assist):
```
SAIL_MOTOR_ASSIST = 0 (disabled)
```

**Motor Assist in Calm**:
```
SAIL_MOTOR_ASSIST = 1 (bit 0)
SAIL_WNDSPD_MIN = 1.5 m/s
SAIL_MOTOR_IDLE = 0%
```

**Motor Assist for Upwind**:
```
SAIL_MOTOR_ASSIST = 3 (bits 0 and 1)
SAIL_WNDSPD_MIN = 1.5 m/s
```

**Always Use Motor in AUTO**:
```
SAIL_MOTOR_ASSIST = 4 (bit 2)
(Acts like powered boat in AUTO mode)
```

#### 5. Sailboat Navigation

Sailboat navigation requires different parameters:

```
CRUISE_SPEED = 1.0-5.0 m/s (depends on wind)
WP_RADIUS = 5.0-20.0 m (wider for tacking approaches)
WP_OVERSHOOT = 3.0-10.0 m (allow sailing momentum)
TURN_RADIUS = 3.0-15.0 m (depends on boat size)
LOIT_TYPE = 1 (face bow to loiter point)
LOIT_RADIUS = 10.0-30.0 m (large for wind/current)
```

**Tacking Considerations**:
- Waypoint missions should avoid dense waypoint patterns upwind
- Allow wide spacing for tacking maneuvers
- Consider wind direction when planning missions
- Use LOITER rather than precise waypoint holds

#### 6. Tuning Procedure for Sailboats

**Step 1: Wind Sensor Calibration**
- Verify accurate wind direction and speed reading
- Test in known wind conditions
- Validate no-go zone detection

**Step 2: Sail Control Tuning**
- Test sail actuation (servo response)
- Verify sail angles (min, max, ideal)
- Confirm heel angle triggers sail easing

**Step 3: Sailing Performance**
- Test downwind sailing (sail at SAIL_ANGLE_MAX)
- Test beam reach (sail at SAIL_ANGLE_IDEAL)
- Test close-hauled (sail at SAIL_ANGLE_MIN)
- Verify acceptable speed in each condition

**Step 4: Tacking Validation**
- Create waypoint upwind within no-go zone
- Enter AUTO mode
- Verify proper tacking behavior
- Adjust SAIL_TACK_ANGLE if needed

**Step 5: Motor Assist**
- Test calm conditions (wind < SAIL_WNDSPD_MIN)
- Verify motor engages as configured
- Test transition back to sailing when wind returns

---

## Advanced Tuning

### Guided Mode Options

**Source**: `Rover/Parameters.cpp:601-606`

```
GUID_OPTIONS: Guided mode options
  Bitmask:
    6: SCurves used for navigation
  Default: 0
```

**S-Curve Navigation** (Bit 6):
- When enabled, guided mode uses S-curve velocity profiles
- Smoother acceleration/deceleration
- Reduces mechanical stress
- More comfortable for passengers
- May slightly increase mission time

**Configuration**:
```
GUID_OPTIONS = 64 (bit 6 set) - Enable S-curves
GUID_OPTIONS = 0 - Linear velocity profiles
```

### Stick Mixing

**Source**: `Rover/Parameters.cpp:546-551`

```
STICK_MIXING: Stick mixing in auto modes
  Values:
    0: Disabled (full autonomous control)
    1: Enabled (pilot input adds to autonomous commands)
  Default: 0
```

**When to Enable**:
- Training/demonstration operations
- Need for operator override without mode change
- Fine adjustments during autonomous operation

**Behavior with Stick Mixing = 1**:
- In AUTO/GUIDED modes, pilot stick input adds to commanded outputs
- Steering stick adjusts heading without changing waypoint target
- Throttle stick adjusts speed without changing speed setpoint
- Returning sticks to center resumes normal autonomous control

**Safety Note**: Stick mixing can lead to unexpected behavior if pilot inputs are not centered. Use cautiously.

### Auto Kickstart

**Source**: `Rover/Parameters.cpp:56-63`

```
AUTO_KICKSTART: Auto mode trigger acceleration
  Units: m/s²
  Range: 0-20 m/s²
  Default: 0 (disabled)
```

**Purpose**: Prevent unwanted motion in AUTO mode by requiring physical push to start

**Configuration**:

**AUTO_KICKSTART = 0**:
- Throttle engages immediately upon entering AUTO mode
- Standard behavior

**AUTO_KICKSTART = 0.5-2.0 m/s²**:
- Vehicle waits for forward acceleration (push start)
- Once threshold exceeded, auto mission begins
- Prevents unexpected startup

**Use Cases**:
- Demonstration environments (vehicle won't move until pushed)
- Safety in AUTO mode startup
- Combined with AUTO_TRIGGER_PIN for manual start systems

### Auto Trigger Pin

**Source**: `Rover/Parameters.cpp:49-54`

```
AUTO_TRIGGER_PIN: Auto mode trigger pin
  Values: -1=Disabled, 0-8=APM pins, 50-55=AUX pins
  Default: -1 (disabled)
```

**Purpose**: Physical button/switch to enable throttle in AUTO mode

**Configuration Example**:
```
AUTO_TRIGGER_PIN = 50 (AUX1)
INITIAL_MODE = AUTO (boot into AUTO mode)
AUTO_KICKSTART = 0 (immediate start when button pressed)
```

**Behavior**:
1. Vehicle boots into AUTO mode
2. Motors will not run until trigger pin is pulled low (button pressed)
3. Releasing button stops motors
4. Re-pressing button resumes mission

**Use Case**: "Push button to start" rover for displays, tours, or remote deployments

### Simple Mode

**Source**: `Rover/Parameters.cpp:501-507`

```
SIMPLE_TYPE: Simple mode types
  Values:
    0: InitialHeading (steer relative to startup heading)
    1: CardinalDirections (steer N/E/S/W regardless of heading)
  Default: 0
  Requires: Reboot after change
```

**Simple Mode Explanation**:

**SIMPLE_TYPE = 0 (InitialHeading)**:
- Forward stick = move in direction vehicle faced at boot
- Right stick = move 90° right of initial heading
- Useful for inexperienced operators

**SIMPLE_TYPE = 1 (CardinalDirections)**:
- Forward stick = North
- Right stick = East
- Back stick = South
- Left stick = West
- Heading-independent piloting

**When to Use Simple Mode**:
- Training new operators
- Remote operation without video feedback
- Operations where global direction more important than vehicle heading

---

## Performance Optimization

### Maximizing Navigation Accuracy

**GPS Configuration**:
- Use RTK GPS for centimeter-level accuracy
- Ensure good GPS antenna placement (clear sky view)
- Enable GPS blending if using multiple receivers
- Configure appropriate GPS_TYPE for your hardware

**EKF Tuning**:
```
EK3_ENABLE = 1 (use EKF3 for navigation)
EK3_GPS_TYPE = 0 (3D velocity and position)
AHRS_EKF_TYPE = 3 (use EKF3)
```

**Sensor Configuration**:
- Calibrate compass away from magnetic interference
- Mount IMU with minimal vibration
- Use wheel encoders for improved odometry
- Configure optical flow if available (indoor/GPS-denied)

### Optimizing Speed Control

**For Consistent Speed Maintenance**:
```
ATC_SPEED_P = 0.5-0.7 (higher for responsive speed control)
ATC_SPEED_I = 0.3-0.5 (corrects for terrain changes)
ATC_SPEED_IMAX = 30% (allows significant correction)
CRUISE_THROTTLE = Accurately measured
SPEED_MAX = Accurately measured
```

**For Smooth Operation**:
```
ATC_THR_SLEWRATE = 50-100 %/s (limits acceleration)
MOT_SLEWRATE = 50-100 %/s (coordinates with ATC setting)
```

### Minimizing Corner Cutting

**Symptoms**: Vehicle cuts inside corners, doesn't reach waypoints

**Solutions**:
1. **Increase waypoint tracking gain**:
   ```
   ATC_STR_ANG_P = 3.5-4.5 (higher values)
   ```

2. **Reduce waypoint radius**:
   ```
   WP_RADIUS = 1.0-2.0 m (tighter acceptance)
   ```

3. **Increase waypoint overshoot**:
   ```
   WP_OVERSHOOT = 2.0-4.0 m (approach beyond waypoint before turning)
   ```

4. **Reduce speed around corners**:
   ```
   CRUISE_SPEED = Lower value
   or use DO_CHANGE_SPEED mission commands for specific segments
   ```

5. **Improve steering response**:
   ```
   ATC_STR_RAT_P = Increase if steering is sluggish
   ATC_STR_RAT_FF = Increase for immediate response
   ```

### Reducing Weaving/Oscillation

**Symptoms**: Vehicle weaves side-to-side around path

**Solutions**:
1. **Reduce steering gains**:
   ```
   ATC_STR_ANG_P = Decrease by 0.5
   ATC_STR_RAT_P = Decrease by 0.05-0.1
   ```

2. **Add damping**:
   ```
   ATC_STR_RAT_D = 0.01-0.03
   ```

3. **Reduce integral term**:
   ```
   ATC_STR_RAT_I = Decrease to 0.1-0.2
   ```

4. **Check GPS update rate**:
   - Ensure GPS is providing 5-10 Hz updates
   - Poor GPS data causes navigation oscillations

5. **Verify compass calibration**:
   - Poor compass calibration causes heading errors
   - Recalibrate compass away from magnetic interference

### Improving Loiter Performance

**Symptoms**: Drifts from loiter point, circles instead of stopping

**Solutions**:
1. **Increase position control gains**:
   ```
   PSC_VEL_P = 0.3-0.5
   PSC_VEL_I = 0.2-0.3
   LOIT_SPEED_GAIN = 0.7-1.0
   ```

2. **Adjust loiter radius**:
   ```
   LOIT_RADIUS = Smaller for tighter hold (1.0-2.0 m)
   ```

3. **Check GPS accuracy**:
   - Loiter performance depends on GPS precision
   - Consider RTK GPS for precision loitering
   - Check HDOP values (should be < 1.5)

4. **Verify speed control**:
   - Loiter uses speed controller to maintain position
   - Ensure ATC_SPEED parameters are well-tuned

### Battery Life Optimization

**Efficient Speed Settings**:
```
CRUISE_SPEED = Find optimal speed (often 40-60% of max)
CRUISE_THROTTLE = Match to efficient cruise speed
ATC_THR_SLEWRATE = 30-50 %/s (gentle acceleration)
```

**Mission Planning**:
- Minimize sharp turns (constant speed more efficient)
- Use optimal cruise speed throughout mission
- Avoid excessive speed for conditions
- Plan direct routes minimizing total distance

**Motor/ESC Configuration**:
- Use appropriate motor/gear ratios for application
- Ensure motors aren't oversized (wasting power)
- Configure ESC timing for efficiency vs. performance

---

## Troubleshooting

### Vehicle Won't Arm

**Check Arming Requirements**:
```
ARMING_CHECK: Arming check bitmask
  View in parameter list for specific failed checks
```

**Common Issues**:
1. **RC failsafe**: Ensure transmitter is on and bound
2. **GPS lock**: Wait for 3D fix (6+ satellites)
3. **Compass**: Calibrate compass properly
4. **EKF**: Wait for EKF to initialize (30-60 seconds)
5. **Battery**: Check battery voltage meets minimum
6. **Gyro**: Ensure vehicle is still during initialization

**Bypass Checks** (for testing only):
```
ARMING_CHECK = 0 (disable all checks - UNSAFE)
```

### Vehicle Won't Move in AUTO

**Possible Causes**:

1. **AUTO_TRIGGER_PIN configured**:
   - Check AUTO_TRIGGER_PIN != -1
   - Ensure trigger switch is activated
   - Solution: Set AUTO_TRIGGER_PIN = -1 or activate switch

2. **AUTO_KICKSTART configured**:
   - Check AUTO_KICKSTART > 0
   - Vehicle waiting for physical push
   - Solution: Push vehicle forward or set AUTO_KICKSTART = 0

3. **Mission not loaded**:
   - Verify mission uploaded to vehicle
   - Check mission contains waypoints
   - Solution: Upload valid mission via GCS

4. **Throttle parameters incorrect**:
   - Check CRUISE_THROTTLE > 0
   - Check MOT_THR_MIN/MAX configured
   - Solution: Set appropriate throttle values

### Steering Oscillation

**Symptoms**: Vehicle weaves, hunts, or oscillates around heading

**Diagnosis and Solutions**:

1. **Excessive Proportional Gain**:
   - Reduce ATC_STR_RAT_P by 0.1
   - Reduce ATC_STR_ANG_P by 0.5

2. **Excessive Integral Gain**:
   - Reduce ATC_STR_RAT_I to 0.1-0.2
   - Slow oscillations indicate integral wind-up

3. **Insufficient Damping**:
   - Increase ATC_STR_RAT_D to 0.01-0.03
   - Adds damping to reduce overshoot

4. **Mechanical Issues**:
   - Check for loose steering linkage
   - Verify servo has sufficient power
   - Ensure steering moves smoothly without binding

5. **Poor GPS/Compass Data**:
   - Recalibrate compass
   - Verify GPS HDOP < 2.0
   - Check for EMI affecting compass

### Speed Oscillation

**Symptoms**: Throttle/speed oscillates, surging motion

**Diagnosis and Solutions**:

1. **Excessive Proportional Gain**:
   - Reduce ATC_SPEED_P by 0.1

2. **Excessive Integral Gain**:
   - Reduce ATC_SPEED_I by 0.05
   - Check ATC_SPEED_IMAX is reasonable (20-30%)

3. **Poor Throttle Calibration**:
   - Recalibrate ESC throttle range
   - Ensure CRUISE_THROTTLE accurately matches cruise speed

4. **Mechanical Issues**:
   - Check for binding in drivetrain
   - Verify motor/ESC are functioning properly
   - Ensure adequate battery voltage

### Vehicle Cuts Corners

**Symptoms**: Doesn't reach waypoints, cuts inside of turns

**Solutions**:
1. Increase ATC_STR_ANG_P (more aggressive steering to waypoints)
2. Increase WP_OVERSHOOT (approach past waypoint before turning)
3. Decrease WP_RADIUS (require closer approach to waypoint)
4. Reduce CRUISE_SPEED (allow more time for steering correction)
5. Improve steering tuning (higher responsiveness)

### Waypoint Navigation Inaccurate

**Symptoms**: Inconsistent waypoint reaching, poor path tracking

**Diagnosis and Solutions**:

1. **GPS Accuracy Issues**:
   - Check GPS HDOP (should be < 1.5-2.0)
   - Verify adequate satellite count (8+ satellites)
   - Consider RTK GPS for precision applications
   - Check GPS antenna placement (clear sky view)

2. **Compass Calibration**:
   - Perform compass calibration away from metal/EMI
   - Verify compass offsets are reasonable (< 300)
   - Check for magnetic interference near compass

3. **EKF Variance**:
   - Monitor EKF variance in telemetry
   - High variance indicates navigation uncertainty
   - Check EK3_POS_I_GATE, EK3_VEL_I_GATE settings

4. **Parameter Tuning**:
   - Verify TURN_RADIUS matches vehicle capability
   - Ensure WP_RADIUS is appropriate for vehicle and speed
   - Check ATC_STR_ANG_P is sufficient for tight tracking

### Obstacle Avoidance Not Working

**Symptoms**: Vehicle collides despite proximity sensors

**Diagnosis and Solutions**:

1. **Avoidance Not Enabled**:
   - Check AVOID_ENABLE > 0
   - Verify appropriate bits set (bit 0 for proximity)

2. **Proximity Sensor Issues**:
   - Verify PRX_TYPE set correctly
   - Check sensor is detecting obstacles in GCS
   - Validate sensor orientation (PRX_ORIENT, PRX_YAW_CORR)

3. **Margin Too Small**:
   - Increase AVOID_MARGIN (try 2.0-3.0 m)
   - At high speeds, need larger margins for stopping distance

4. **Sensor Blind Spots**:
   - Check PRX_IGN_ANG settings not blocking obstacle
   - Verify sensor coverage in direction of travel
   - Add additional sensors for full coverage

5. **Path Planning Disabled**:
   - If using object avoidance, verify OA_TYPE > 0
   - Check OA_MARGIN_MAX and OA_LOOKAHEAD configured

### Failsafe Not Triggering

**Symptoms**: Vehicle continues despite RC/GCS loss

**Diagnosis and Solutions**:

1. **Failsafe Disabled**:
   - Check FS_THR_ENABLE = 1 or 2
   - Check FS_GCS_ENABLE = 1 or 2 (if using telemetry failsafe)

2. **Threshold Not Reached**:
   - Verify FS_THR_VALUE < normal RC minimum
   - Test RC failsafe by turning off transmitter
   - Check FS_TIMEOUT is appropriate

3. **Failsafe in HOLD Mode**:
   - If FS_OPTIONS bit 0 not set, failsafes ignored in HOLD
   - Solution: Set FS_OPTIONS = 1 or avoid HOLD mode

4. **GCS Heartbeat Still Received**:
   - GCS failsafe requires loss of MAVLink heartbeat
   - Check GCS is actually disconnected during test
   - Verify FS_GCS_TIMEOUT is appropriate

### Logs and Diagnostics

**Enable Comprehensive Logging**:
```
LOG_BITMASK = 65535 (enable all logs)
```

**Key Log Messages for Troubleshooting**:
- **PSCN (Position Control)**: Position and velocity control performance
- **CTRL (Control)**: Steering and throttle commands
- **STER (Steering)**: Steering controller internal state
- **NTUN (Navigation Tuning)**: Waypoint navigation performance
- **ATT (Attitude)**: Heading and orientation
- **POS (Position)**: GPS position and EKF estimates
- **NAVEKF**: EKF variance and health

**Analyzing Logs**:
1. Download logs from vehicle via MAVLink
2. Open in MAVExplorer, Mission Planner, or QGroundControl
3. Graph relevant parameters vs. time
4. Look for oscillations, delays, or unexpected behavior
5. Adjust parameters based on observed performance

### Getting Help

**Before Requesting Support**:
1. Download and review logs
2. Verify all sensors are calibrated
3. Confirm ArduPilot version and hardware
4. Document specific symptoms and conditions
5. Note any error messages in GCS

**Support Resources**:
- **ArduPilot Forum**: https://discuss.ardupilot.org/c/rover
- **Discord**: ArduPilot Discord server, #rover channel
- **Wiki**: https://ardupilot.org/rover/
- **GitHub Issues**: https://github.com/ArduPilot/ardupilot/issues

**When Posting for Help**:
- Provide vehicle description (size, type, frame class/type)
- Include relevant parameters (copy from GCS)
- Attach log files (use dataflash logs, not tlogs)
- Describe tuning steps already attempted
- Specify ArduPilot version and hardware platform

---

## Summary and Quick Reference

### Tuning Order

**Recommended Tuning Sequence**:

1. **Frame Configuration**: Set FRAME_CLASS, FRAME_TYPE, PILOT_STEER_TYPE
2. **Speed/Throttle**: Tune CRUISE_SPEED, CRUISE_THROTTLE, SPEED_MAX
3. **Basic Steering**: Tune ATC_STR_RAT_P/I/D/FF in ACRO mode
4. **Waypoint Steering**: Tune ATC_STR_ANG_P in AUTO mode
5. **Navigation**: Set TURN_RADIUS, WP_RADIUS, WP_OVERSHOOT
6. **Position Control**: Tune PSC_VEL parameters for LOITER mode
7. **Advanced Features**: Enable and tune obstacle avoidance, path planning
8. **Failsafes**: Configure and test all failsafe modes

### Parameter Quick Reference

**Essential Parameters**:
```
# Frame Configuration
FRAME_CLASS = 1 (Rover), 2 (Boat), or 3 (BalanceBot)
FRAME_TYPE = 0 (Standard) or 1-4 (Omni configurations)

# Speed Control
CRUISE_SPEED = 2.0-5.0 m/s (typical)
CRUISE_THROTTLE = 50% (starting point)
SPEED_MAX = 0 (auto-estimate) or measured max

# Steering Control (ATC_ group)
ATC_STR_RAT_P = 0.2-0.5 (start conservative)
ATC_STR_RAT_I = 0.2-0.3
ATC_STR_RAT_D = 0.0-0.02
ATC_STR_RAT_FF = 0.2-0.5
ATC_STR_ANG_P = 2.0-3.5

# Speed Control (ATC_ group)
ATC_SPEED_P = 0.3-0.5
ATC_SPEED_I = 0.1-0.3
ATC_SPEED_IMAX = 20-30%

# Navigation (WP_ group)
WP_RADIUS = 2.0 m (start large)
WP_OVERSHOOT = 1.0 m
WP_PIVOT_ANGLE = 0 (disabled) or 60-90 deg (skid-steer)

# Position Control (PSC group)
PSC_VEL_P = 0.2-0.3
PSC_VEL_I = 0.1-0.2
LOIT_RADIUS = 2.0 m
LOIT_SPEED_GAIN = 0.5

# Failsafes
FS_THR_ENABLE = 1
FS_ACTION = 1 (RTL) or 2 (Hold)
FS_TIMEOUT = 1.5 s
FS_GCS_ENABLE = 1 (for autonomous missions)
FS_CRASH_CHECK = 2 (HoldAndDisarm recommended)
```

### Common Parameter Sets

**Small Rover (< 1m, indoor)**:
```
CRUISE_SPEED = 1.0-2.0 m/s
WP_RADIUS = 0.5-1.0 m
TURN_RADIUS = 0.3-0.5 m
ATC_STR_ANG_P = 3.0-4.0
```

**Medium Rover (1-2m, outdoor)**:
```
CRUISE_SPEED = 2.0-5.0 m/s
WP_RADIUS = 2.0-3.0 m
TURN_RADIUS = 0.5-1.5 m
ATC_STR_ANG_P = 2.5-3.5
```

**Large Rover (> 2m, outdoor)**:
```
CRUISE_SPEED = 3.0-10.0 m/s
WP_RADIUS = 3.0-5.0 m
TURN_RADIUS = 1.5-5.0 m
ATC_STR_ANG_P = 2.0-3.0
```

**Boat/Marine**:
```
FRAME_CLASS = 2
CRUISE_SPEED = 2.0-8.0 m/s
WP_RADIUS = 5.0-10.0 m
LOIT_RADIUS = 10.0-20.0 m
LOIT_TYPE = 1 (face bow to loiter point)
```

---

## Conclusion

Proper tuning of ArduRover requires systematic adjustment of parameters starting from basic vehicle configuration through advanced navigation features. Always test changes incrementally in safe environments and validate failsafe behavior before autonomous operations.

**Key Principles**:
- Start conservative, increase aggressiveness gradually
- Tune in order: speed → steering → navigation → advanced features
- Test each parameter change before moving to next
- Always configure and test failsafes
- Maintain logs for troubleshooting and refinement

For additional support and detailed technical information, consult the [ArduPilot Rover documentation](https://ardupilot.org/rover/) and community forums.

---

**Document Information**:
- **Source Files**: Rover/Parameters.cpp, Rover/Parameters.h, Rover/Steering.cpp
- **Parameter Groups**: ATC (AR_AttitudeControl), PSC (AR_PosControl), WP (AR_WPNav), MOT (AP_MotorsUGV)
- **Library Documentation**: See /libraries/APM_Control/, /libraries/AR_WPNav/, /libraries/AR_Motors/
- **Last Updated**: Generated from ArduPilot master branch parameter definitions
- **Version**: Compatible with ArduPilot 4.7+

