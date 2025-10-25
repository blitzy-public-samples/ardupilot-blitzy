# ArduCopter Flight Modes

## Table of Contents

- [Overview](#overview)
- [Mode Categories](#mode-categories)
- [Mode Overview Table](#mode-overview-table)
- [Manual Flight Modes](#manual-flight-modes)
- [Assisted Flight Modes](#assisted-flight-modes)
- [Autonomous Flight Modes](#autonomous-flight-modes)
- [Special Purpose Modes](#special-purpose-modes)
- [Mode Transitions](#mode-transitions)
- [Safety Considerations](#safety-considerations)
- [Parameter Dependencies](#parameter-dependencies)

## Overview

ArduCopter supports 26+ distinct flight modes that provide varying levels of autonomous control, from fully manual attitude control to fully autonomous waypoint navigation. Each mode is designed for specific use cases and pilot skill levels, offering different combinations of stabilization, position hold, and autonomous navigation.

**Source**: `/ArduCopter/mode.h` Mode::Number enum (lines 77-109)

### Mode Selection

Flight modes can be selected via:
- RC transmitter mode switch (configured via RCx_OPTION parameters)
- Ground Control Station (GCS) commands via MAVLink
- Mission commands (for AUTO mode)
- Failsafe triggers (automatic mode changes)
- Scripting/external control interfaces

### Common Requirements

All flight modes share common initialization and execution patterns defined in the base `Mode` class:
- `init(bool ignore_checks)` - Mode entry initialization (source: mode.h:123)
- `run()` - Main mode execution loop called at scheduler rate (source: mode.h:127)
- `exit()` - Cleanup when leaving mode (source: mode.h:126)

## Mode Categories

### Manual Modes
Pilot has direct control over vehicle attitude and/or throttle. Requires continuous pilot input.
- Modes: STABILIZE, ACRO

### Assisted Modes
Provide altitude and/or position hold with manual pilot control inputs.
- Modes: ALT_HOLD, LOITER, POSHOLD, SPORT, DRIFT, FLOWHOLD

### Autonomous Modes
Vehicle flies autonomously with minimal or no pilot input required.
- Modes: AUTO, GUIDED, RTL, SMART_RTL, LAND, CIRCLE, BRAKE, FOLLOW, ZIGZAG

### Special Purpose Modes
Modes designed for specific operations or conditions.
- Modes: FLIP, AUTOTUNE, THROW, AVOID_ADSB, GUIDED_NOGPS, SYSTEMID, AUTOROTATE, TURTLE, GUIDED_CUSTOM

## Mode Overview Table

| Mode # | Mode Name | Category | GPS Required | Manual Throttle | Description |
|--------|-----------|----------|--------------|-----------------|-------------|
| 0 | STABILIZE | Manual | No | Yes | Manual angle control with self-leveling, pilot controls throttle |
| 1 | ACRO | Manual | No | Yes | Manual angular rate control, no self-leveling |
| 2 | ALT_HOLD | Assisted | No | No | Automatic altitude hold with manual position control |
| 3 | AUTO | Autonomous | Yes | No | Fully autonomous mission execution from waypoints |
| 4 | GUIDED | Autonomous | Yes | No | Computer-controlled navigation via MAVLink commands |
| 5 | LOITER | Assisted | Yes | No | GPS position and altitude hold with manual override |
| 6 | RTL | Autonomous | Yes | No | Return to launch point and land |
| 7 | CIRCLE | Autonomous | Yes | No | Orbit around a point of interest |
| 9 | LAND | Autonomous | No | No | Automated landing with optional position control |
| 11 | DRIFT | Assisted | Yes | No | Car-like steering for FPV racing |
| 13 | SPORT | Assisted | No | No | Rate control on roll/pitch with altitude hold |
| 14 | FLIP | Special | No | No | Automated flip maneuver on specified axis |
| 15 | AUTOTUNE | Special | No | No | Automated PID tuning for attitude controllers |
| 16 | POSHOLD | Assisted | Yes | No | Aggressive position hold with wind compensation |
| 17 | BRAKE | Autonomous | Yes | No | Aggressive deceleration to stop |
| 18 | THROW | Special | Yes | No | Throw-to-launch mode for hand launching |
| 19 | AVOID_ADSB | Autonomous | Yes | No | Collision avoidance using ADS-B transponder data |
| 20 | GUIDED_NOGPS | Autonomous | No | No | Guided mode accepting only attitude/altitude commands |
| 21 | SMART_RTL | Autonomous | Yes | No | Return via recorded path with obstacle avoidance |
| 22 | FLOWHOLD | Assisted | No | No | Position hold using optical flow without rangefinder |
| 23 | FOLLOW | Autonomous | Yes | No | Follow another vehicle or ground station |
| 24 | ZIGZAG | Autonomous | Yes | No | Automated back-and-forth flight pattern |
| 25 | SYSTEMID | Special | No | Yes | System identification for model analysis |
| 26 | AUTOROTATE | Special | No | No | Helicopter emergency autorotation (helicopters only) |
| 28 | TURTLE | Special | No | Yes | Flip recovery mode after crash |
| 31 | GUIDED_CUSTOM | Autonomous | Yes | No | Custom scripting-defined guided mode variant |

**Source**: `/ArduCopter/mode.h` lines 77-109, mode.cpp:32-160

---

## Manual Flight Modes

### STABILIZE (Mode 0)

**Description**: Manual airframe angle control with automatic self-leveling and manual throttle control. This is the most basic flight mode and the recommended mode for learning to fly.

**Source**: `/ArduCopter/mode_stabilize.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:1664)
- **Manual Throttle**: Yes (source: mode.h:1665)
- **Allows Arming**: Yes (source: mode.h:1666)
- **Autopilot Mode**: No (source: mode.h:1667)

**Behavior**:
- Pilot stick inputs directly control vehicle lean angles relative to horizontal
- When sticks are centered, vehicle automatically levels itself
- Throttle stick directly controls motor output
- Yaw stick controls yaw rotation rate
- Maximum lean angle limited by `ANGLE_MAX` parameter

**Key Functions**:
- `ModeStabilize::run()` - Main execution loop (source: mode_stabilize.cpp:9)
  - Converts pilot input to lean angles via `get_pilot_desired_lean_angles_rad()` (source: mode_stabilize.cpp:16)
  - Processes pilot yaw rate via `get_pilot_desired_yaw_rate_rads()` (source: mode_stabilize.cpp:19)
  - Manages motor spool state based on throttle position (source: mode_stabilize.cpp:21-33)

**Parameters**:
- `ANGLE_MAX` - Maximum lean angle in centidegrees (default: 4500 = 45°)
- `PILOT_THR_FILT` - Throttle filter cutoff frequency (source: Parameters.cpp:51)

**Use Cases**:
- Initial flight training
- Manual acrobatic flying with self-leveling safety
- Flying in areas where GPS is unavailable
- Emergency manual control during failsafes

**Safety Notes**:
- Does not prevent vehicle from drifting horizontally in wind
- Pilot must maintain throttle to avoid altitude loss
- Allows flip operations when enabled (source: mode.h:1671)
- Allows trim saving and auto-trim (source: mode.h:1668-1669)

---

### ACRO (Mode 1)

**Description**: Manual body-frame angular rate control with manual throttle. Provides direct rate control without self-leveling, intended for experienced pilots and acrobatic flight.

**Source**: `/ArduCopter/mode_acro.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:428)
- **Manual Throttle**: Yes (source: mode.h:429)
- **Allows Arming**: Yes (source: mode.h:430)
- **Autopilot Mode**: No (source: mode.h:431)
- **Crash Check**: Disabled (source: mode.h:438)

**Behavior**:
- Pilot stick inputs directly control angular rotation rates (deg/sec) on all axes
- No self-leveling when sticks are centered - vehicle maintains current attitude
- Throttle stick directly controls motor output
- Enables full aerobatic maneuvers including inverted flight

**Key Functions**:
- `ModeAcro::init()` - Resets rate controller I-terms (source: mode.h:432)
- `ModeAcro::run()` - Main execution loop (source: mode.h:426)
- `get_pilot_desired_rates_rads()` - Converts pilot input to desired rates (source: mode.h:448)

**Trainer Modes** (source: mode.h:415-419):
- `TRAINER_OFF` (0) - Full rate control, no limits
- `TRAINER_LEVELING` (1) - Automatic leveling when sticks centered
- `TRAINER_LIMITED` (2) - Limits lean angles like stabilize mode

**Parameters**:
- `ACRO_RP_P` - Rate controller P gain for roll/pitch
- `ACRO_YAW_P` - Rate controller P gain for yaw
- `ACRO_BAL_ROLL` / `ACRO_BAL_PITCH` - Automatic leveling trainer strength
- `ACRO_TRAINER` - Trainer mode selection (0=OFF, 1=LEVELING, 2=LIMITED)
- `ACRO_RP_RATE` - Maximum roll/pitch rate (default: 180 deg/s)
- `ACRO_Y_RATE` - Maximum yaw rate (default: 90 deg/s)
- `ACRO_OPTIONS` - Bitmask for options (bit 0: Air mode, bit 1: Rate loop only) (source: mode.h:421-424)

**Use Cases**:
- Aerobatic flight and FPV racing
- Advanced manual control
- Inverted flight maneuvers
- Situations requiring maximum control authority

**Safety Notes**:
- Not recommended for beginners - no automatic leveling without trainer
- Can enter unusual attitudes requiring advanced recovery skills
- RC failsafe entry disabled (source: mode.h:439)
- Allows flip operations (source: mode.h:437)
- Air mode option prevents motor saturation during aggressive maneuvers

---

## Assisted Flight Modes

### ALT_HOLD (Mode 2)

**Description**: Automatic altitude hold with manual position control. Vehicle automatically maintains altitude while pilot controls horizontal movement.

**Source**: `/ArduCopter/mode_althold.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:484)
- **Manual Throttle**: No (source: mode.h:485)
- **Allows Arming**: Yes (source: mode.h:486)
- **Autopilot Mode**: No (source: mode.h:487)
- **User Takeoff**: Yes (source: mode.h:488-490)

**Behavior**:
- Throttle stick controls climb/descent rate when moved from mid-position
- Throttle mid-position holds current altitude using barometer
- Roll/pitch sticks control vehicle lean angles (like STABILIZE)
- Yaw stick controls yaw rotation rate
- Altitude controller automatically adjusts throttle to maintain height

**Key Functions**:
- `ModeAltHold::init()` - Initializes altitude target to current altitude (source: mode.h:481)
- `ModeAltHold::run()` - Maintains altitude while processing pilot inputs (source: mode.h:482)
- `get_alt_hold_state()` - Manages state machine for takeoff/landing detection (source: mode.h:258)

**Alt Hold State Machine** (source: mode.h:251-257):
- `MotorStopped` - Motors stopped, on ground
- `Takeoff` - Takeoff sequence in progress
- `Landed_Ground_Idle` - Landed with motors at ground idle
- `Landed_Pre_Takeoff` - Landed, preparing for takeoff
- `Flying` - Normal flight with altitude control active

**Parameters**:
- `PILOT_THR_FILT` - Throttle filter cutoff (source: Parameters.cpp:51)
- `PILOT_TKOFF_ALT` - Altitude for takeoff when throttle raised (source: Parameters.cpp:60)
- `PILOT_THR_BHV` - Throttle stick behavior bitmask (source: Parameters.cpp:67)
- `PILOT_SPEED_UP` - Maximum climb rate (cm/s)
- `PILOT_SPEED_DN` - Maximum descent rate (cm/s)
- `PILOT_ACCEL_Z` - Vertical acceleration (cm/s²)

**Use Cases**:
- Learning autonomous altitude control
- Manual flying with altitude assistance
- Flying in GPS-denied environments with altitude hold
- Indoor flight where GPS is unavailable

**Safety Notes**:
- Barometric altitude can drift with changing weather/temperature
- Horizontal position not controlled - vehicle will drift in wind
- Allows autotune, flip, auto-trim, and trim saving (source: mode.h:491-494)
- Helicopters: Allows inverted flight (source: mode.h:495-497)

---

### LOITER (Mode 5)

**Description**: GPS-based position and altitude hold with manual override capability. Vehicle actively maintains 3D position using GPS and barometer.

**Source**: `/ArduCopter/mode_loiter.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1323)
- **Manual Throttle**: No (source: mode.h:1324)
- **Allows Arming**: Yes (source: mode.h:1325)
- **Autopilot Mode**: No (source: mode.h:1326)
- **User Takeoff**: Yes (source: mode.h:1327)

**Behavior**:
- Vehicle automatically maintains horizontal position using GPS
- Automatically maintains altitude using barometer
- Pilot stick inputs command velocity changes from hover position
- When sticks centered, returns to position hold
- Wind compensation automatically applied

**Key Functions**:
- `ModeLoiter::init()` - Initializes loiter controller and position target (source: mode.h:1320)
- `ModeLoiter::run()` - Maintains position and altitude (source: mode.h:1321)
- Precision loiter: `do_precision_loiter()` - Uses precision landing sensors (source: mode.h:1349)

**Precision Landing Support** (requires AC_PRECLAND_ENABLED):
- Can track precision landing targets (IR-LOCK, etc.)
- `set_precision_loiter_enabled()` - Enables precision tracking (source: mode.h:1336)
- `precision_loiter_xy()` - Tracks target horizontally (source: mode.h:1350)

**Parameters**:
- `LOIT_SPEED` - Maximum horizontal speed in cm/s
- `LOIT_ACC_MAX` - Maximum horizontal acceleration (cm/s²)
- `LOIT_BRK_ACCEL` - Braking deceleration rate (cm/s²)
- `LOIT_BRK_JERK` - Braking jerk limit (m/s³)
- `PILOT_SPEED_UP` - Maximum climb rate (cm/s)
- `PILOT_SPEED_DN` - Maximum descent rate (cm/s)
- `PILOT_ACCEL_Z` - Vertical acceleration (cm/s²)

**Reporting Functions** (source: mode.h:1344-1346):
- `wp_distance_m()` - Distance to loiter center
- `wp_bearing_deg()` - Bearing to loiter center
- `crosstrack_error()` - Position tracking error

**Use Cases**:
- Stable hovering for photography/videography
- Hands-off position holding
- Pilot rest during long flights
- Precision maneuvers around specific location

**Safety Notes**:
- Requires good GPS position fix (hdop < 2.0 recommended)
- May drift if GPS accuracy degrades
- Allows autotune and auto-trim (source: mode.h:1328-1329)
- Helicopters: Allows inverted flight (source: mode.h:1331-1333)
- Position control may be affected by strong winds exceeding vehicle capabilities

---

### POSHOLD (Mode 16)

**Description**: Position hold with aggressive brake response and wind compensation. Provides improved position holding compared to LOITER with brake-to-loiter transitions.

**Source**: `/ArduCopter/mode_poshold.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1373)
- **Manual Throttle**: No (source: mode.h:1374)
- **Allows Arming**: Yes (source: mode.h:1375)
- **Autopilot Mode**: No (source: mode.h:1376)
- **User Takeoff**: Yes (source: mode.h:1377)

**Behavior**:
- When pilot releases sticks, vehicle aggressively brakes to a stop
- After stopping, transitions to position hold with wind compensation
- Pilot inputs command velocity, vehicle brakes when sticks released
- Separate state machines for roll and pitch axes allow independent control

**Roll/Pitch Mode States** (source: mode.h:1397-1404):
- `PILOT_OVERRIDE` - Pilot controlling this axis
- `BRAKE` - Actively braking towards zero velocity
- `BRAKE_READY_TO_LOITER` - Braking complete, ready for loiter
- `BRAKE_TO_LOITER` - Transitioning from brake to loiter control
- `LOITER` - Position hold active on this axis
- `CONTROLLER_TO_PILOT_OVERRIDE` - Transitioning back to pilot control

**Wind Compensation**:
- Estimates wind-induced lean angles during loiter
- `init_wind_comp_estimate()` - Initializes wind estimation (source: mode.h:1391)
- `update_wind_comp_estimate()` - Updates wind compensation (source: mode.h:1392)
- `get_wind_comp_lean_angles()` - Retrieves compensation angles (source: mode.h:1393)

**Key Functions**:
- `ModePosHold::init()` - Initializes brake and loiter systems (source: mode.h:1370)
- `ModePosHold::run()` - Manages state machines for both axes (source: mode.h:1371)
- `update_pilot_lean_angle_cd()` - Processes pilot input with filtering (source: mode.h:1388)
- `update_brake_angle_from_velocity()` - Calculates brake angles (source: mode.h:1390)

**Parameters**:
- `PHLD_BRAKE_RATE` - Braking rate (cm/s/s)
- `PHLD_BRAKE_ANGLE` - Maximum brake lean angle (centidegrees)
- `PILOT_ACCEL` - Pilot input acceleration (cm/s²)
- `LOIT_SPEED` - Maximum horizontal speed
- Similar altitude control parameters as LOITER

**Use Cases**:
- Precision photography with quick stop capability
- Flying in confined spaces requiring quick stops
- Improved position hold in gusty wind conditions
- Advanced pilots wanting more responsive position control

**Safety Notes**:
- Requires good GPS lock for position estimation
- Aggressive braking may be uncomfortable for passengers
- Allows autotune and auto-trim (source: mode.h:1378-1379)
- Wind compensation requires several seconds to initialize

---

### SPORT (Mode 13)

**Description**: Rate control on roll and pitch axes with altitude hold. Combines acro-like responsiveness with altitude automation.

**Source**: `/ArduCopter/mode_sport.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:1637)
- **Manual Throttle**: No (source: mode.h:1638)
- **Allows Arming**: Yes (source: mode.h:1639)
- **Autopilot Mode**: No (source: mode.h:1640)
- **User Takeoff**: Yes (source: mode.h:1641-1643)

**Behavior**:
- Roll and pitch sticks control rotation rates (like ACRO mode)
- Throttle stick controls climb/descent rate (like ALT_HOLD)
- No self-leveling on roll/pitch when sticks centered
- Altitude automatically maintained at mid-throttle
- Provides acro-like feel with altitude convenience

**Key Functions**:
- `ModeSport::init()` - Initializes altitude controller (source: mode.h:1634)
- `ModeSport::run()` - Combines rate and altitude control (source: mode.h:1635)

**Parameters**:
- `ACRO_RP_RATE` - Maximum roll/pitch rate
- `ACRO_Y_RATE` - Maximum yaw rate  
- `PILOT_SPEED_UP` - Maximum climb rate
- `PILOT_SPEED_DN` - Maximum descent rate
- `PILOT_ACCEL_Z` - Vertical acceleration

**Use Cases**:
- FPV racing with altitude assistance
- Acrobatic flying without manual throttle management
- Transitioning from ACRO to altitude-controlled modes
- High-speed maneuvering with altitude safety

**Safety Notes**:
- No horizontal position hold - will drift like ACRO
- Requires pilot proficiency with rate control
- Can enter unusual attitudes like ACRO mode
- Altitude control helps prevent ground strikes during aggressive maneuvers

---

### DRIFT (Mode 11)

**Description**: Car-like steering mode optimized for FPV racing. Roll stick steers, pitch stick controls forward/back speed.

**Source**: `/ArduCopter/mode_drift.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:914)
- **Manual Throttle**: No (source: mode.h:915)
- **Allows Arming**: Yes (source: mode.h:916)
- **Autopilot Mode**: No (source: mode.h:917)
- **RC Failsafe Entry**: Disabled (source: mode.h:918)

**Behavior**:
- Roll stick controls yaw rotation rate (steering)
- Pitch stick controls forward/backward velocity
- Altitude automatically maintained
- Vehicle automatically banks into turns like a plane
- Throttle provides altitude rate control

**Key Functions**:
- `ModeDrift::init()` - Initializes drift controller (source: mode.h:911)
- `ModeDrift::run()` - Implements car-like control (source: mode.h:912)
- `get_throttle_assist()` - Provides throttle boost during maneuvers (source: mode.h:927)

**Parameters**:
- `DRIFT_SPEEDGAIN` - Gain for speed control
- `PILOT_SPEED_UP` - Maximum climb rate
- `PILOT_SPEED_DN` - Maximum descent rate

**Use Cases**:
- FPV racing with intuitive car-like controls
- Pilots transitioning from ground vehicles
- High-speed forward flight
- Cinematic shots requiring smooth turns

**Safety Notes**:
- Requires GPS for velocity estimation
- Not suitable for precision hovering
- Limited backwards flight capability
- RC failsafe entry disabled (source: mode.h:918)

---

### FLOWHOLD (Mode 22)

**Description**: Position hold using optical flow sensor without requiring a rangefinder. Maintains position using visual tracking.

**Source**: `/ArduCopter/mode_flowhold.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:989)
- **Manual Throttle**: No (source: mode.h:990)
- **Allows Arming**: Yes (source: mode.h:991)
- **Autopilot Mode**: No (source: mode.h:992)
- **User Takeoff**: Yes (source: mode.h:993-995)

**Behavior**:
- Uses optical flow sensor to detect ground movement
- Maintains position by compensating for detected drift
- Does not require rangefinder altitude measurement
- Automatic altitude hold using barometer
- Pilot can override position with stick inputs

**FlowHold States** (source: mode.h:1007-1012):
- `FlowHold_MotorStopped` - Motors stopped
- `FlowHold_Takeoff` - Takeoff in progress
- `FlowHold_Flying` - Normal flight with flow control
- `FlowHold_Landed` - Landed state

**Key Functions**:
- `ModeFlowHold::init()` - Initializes flow and altitude control (source: mode.h:986)
- `ModeFlowHold::run()` - Processes flow data for position hold (source: mode.h:987)
- `flow_to_angle()` - Converts flow measurements to lean angles (source: mode.h:1015)
- `update_height_estimate()` - Estimates height for flow scaling (source: mode.h:1022)

**Parameters** (source: mode.h:998):
- `FHLD_FILT_HZ` (flow_filter_hz) - Flow data filter cutoff frequency
- `FHLD_FLOW_MAX` (flow_max) - Maximum flow rate
- `FHLD_QUAL_MIN` (flow_min_quality) - Minimum acceptable flow quality
- `FHLD_BRAKE_RATE` (brake_rate_dps) - Braking rate in deg/s

**Flow Processing**:
- Low-pass filter on flow data for noise reduction (source: mode.h:1017)
- Quality filtering rejects poor flow measurements (source: mode.h:1036)
- Height-based flow scaling (source: mode.h:1025-1028)
- Integrator with brake logic (source: mode.h:1031, 1055)

**Use Cases**:
- Indoor flight without GPS or rangefinder
- Position hold in GPS-denied environments
- Low-altitude outdoor flight with good ground texture
- Backup position hold mode

**Safety Notes**:
- Requires optical flow sensor with good quality signal
- Works best over textured surfaces (not smooth/uniform)
- Height estimation from barometer less accurate than rangefinder
- Flow quality degrades with altitude (source: mode.h:1025-1028)
- Allows flip operations (source: mode.h:996)

---

## Autonomous Flight Modes

### AUTO (Mode 3)

**Description**: Fully autonomous waypoint navigation executing mission commands. Vehicle follows pre-programmed mission stored in autopilot memory.

**Source**: `/ArduCopter/mode_auto.cpp`

**Characteristics**:
- **GPS Required**: Depends on mission commands (source: mode.h:520)
- **Manual Throttle**: No (source: mode.h:521)
- **Allows Arming**: Depends on AUTO_OPTIONS parameter (source: mode.h:522)
- **Autopilot Mode**: Yes (source: mode.h:523)
- **In Guided Mode**: When in NAV_GUIDED or NAV_SCRIPT_TIME submodes (source: mode.h:524)

**Auto Submodes** (source: mode.h:538-553):
- `TAKEOFF` - Executing takeoff command
- `WP` - Flying to waypoint
- `LAND` - Executing landing
- `RTL` - Return to launch within AUTO mission
- `CIRCLE_MOVE_TO_EDGE` - Moving to circle starting position
- `CIRCLE` - Circling around point
- `NAVGUIDED` - Guided control within AUTO mission
- `LOITER` - Loitering at waypoint
- `LOITER_TO_ALT` - Loitering while changing altitude
- `NAV_PAYLOAD_PLACE` - Payload placement operation (if enabled)
- `NAV_SCRIPT_TIME` - Lua script control
- `NAV_ATTITUDE_TIME` - Attitude control for specified time

**Mission Execution**:
- Mission stored in non-volatile memory survives power cycles
- `AP_Mission mission` - Mission library instance (source: mode.h:601-604)
- `start_command()` - Initiates mission command (source: mode.h:640)
- `verify_command()` - Checks command completion (source: mode.h:641)
- `exit_mission()` - Cleanup on mission end (source: mode.h:642)

**Key Functions**:
- `ModeAuto::init()` - Starts or resumes mission (source: mode.h:516)
- `ModeAuto::run()` - Executes current mission command (source: mode.h:518)
- `set_submode()` - Changes AUTO submode (source: mode.h:556)
- `takeoff_start()` - Initiates takeoff sequence (source: mode.h:565)
- `wp_start()` - Begins waypoint navigation (source: mode.h:566)
- `land_start()` - Starts landing sequence (source: mode.h:567)

**Supported Mission Commands**:
- `NAV_WAYPOINT` - Fly to waypoint (source: mode.h:668)
- `NAV_TAKEOFF` - Takeoff to altitude (source: mode.h:667)
- `NAV_LAND` - Land at location (source: mode.h:670)
- `NAV_LOITER_TIME` - Loiter for specified time (source: mode.h:673)
- `NAV_LOITER_TO_ALT` - Loiter while reaching altitude (source: mode.h:674)
- `NAV_LOITER_UNLIM` - Loiter indefinitely (source: mode.h:671)
- `NAV_SPLINE_WAYPOINT` - Smooth spline path (source: mode.h:675)
- `DO_CHANGE_SPEED` - Modify flight speed (source: mode.h:685)
- `DO_SET_HOME` - Update home position (source: mode.h:686)
- `DO_MOUNT_CONTROL` - Control camera gimbal (source: mode.h:688)
- Many others defined in AP_Mission library

**AUTO_RTL Mode**:
- Special case when AUTO executes DO_LAND_START (source: mode.h:754)
- Reports as AUTO_RTL (mode 27) to GCS (source: mode.h:514)
- `jump_to_landing_sequence_auto_RTL()` - Enters AUTO_RTL (source: mode.h:589)

**Parameters**:
- `AUTO_OPTIONS` - Bitmask controlling AUTO behavior (source: mode.h:629-634):
  - Bit 0: Allow arming in AUTO
  - Bit 1: Allow takeoff without raising throttle
  - Bit 2: Ignore pilot yaw input
  - Bit 7: Allow weathervaning
- `WP_YAW_BEHAVIOR` - Yaw behavior at waypoints
- `WPNAV_SPEED` - Default horizontal speed (cm/s)
- `WPNAV_SPEED_UP` - Climb speed (cm/s)
- `WPNAV_SPEED_DN` - Descent speed (cm/s)
- `WPNAV_ACCEL` - Horizontal acceleration (cm/s²)
- `WPNAV_ACCEL_Z` - Vertical acceleration (cm/s²)

**Use Cases**:
- Automated survey and mapping missions
- Precision agriculture operations
- Inspection flights following predetermined paths
- Delivery and transport missions
- Search and rescue patterns

**Safety Notes**:
- Mission should be thoroughly tested in simulator first
- Requires terrain failsafe handling (source: mode.h:581)
- Can override speed via `set_speed_xy_cms()`, `set_speed_up_cms()`, `set_speed_down_cms()` (source: mode.h:577-579)
- Pilot yaw control depends on mission commands and AUTO_OPTIONS (source: mode.h:575)
- Helicopters: Allows inverted flight (source: mode.h:525-527)
- Change detection: `mis_change_detector` monitors external mission changes (source: mode.h:607)

---

### GUIDED (Mode 4)

**Description**: Computer-controlled flight via MAVLink commands from ground control station or companion computer. Real-time position/velocity/acceleration control.

**Source**: `/ArduCopter/mode_guided.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1077)
- **Manual Throttle**: No (source: mode.h:1078)
- **Allows Arming**: Depends on GUID_OPTIONS parameter (source: mode.h:1079)
- **Autopilot Mode**: Yes (source: mode.h:1080)
- **In Guided Mode**: Yes (source: mode.h:1082)
- **User Takeoff**: Yes (source: mode.h:1081)

**Guided Submodes** (source: mode.h:1138-1146):
- `TakeOff` - Takeoff to specified altitude
- `WP` - Fly to position waypoint
- `Pos` - Position control
- `PosVelAccel` - Position with velocity and acceleration
- `VelAccel` - Velocity with acceleration feedforward
- `Accel` - Direct acceleration control
- `Angle` - Attitude angle control

**Control Interfaces**:
- `set_destination()` - Fly to 3D position (source: mode.h:1103-1104)
- `set_velocity()` - Command velocity vector (source: mode.h:1107)
- `set_accel()` - Command acceleration (source: mode.h:1106)
- `set_velaccel()` - Combined velocity/acceleration (source: mode.h:1108)
- `set_destination_posvel()` - Position with velocity (source: mode.h:1109)
- `set_destination_posvelaccel()` - Full PVA control (source: mode.h:1110)
- `set_angle()` - Attitude quaternion with climb rate (source: mode.h:1101)

**Key Functions**:
- `ModeGuided::init()` - Initializes guided control (source: mode.h:1074)
- `ModeGuided::run()` - Executes current guided submode (source: mode.h:1075)
- `wp_control_run()` - Waypoint guidance (source: mode.h:1195)
- `pos_control_run()` - Position control loop (source: mode.h:1203)
- `velaccel_control_run()` - Velocity/accel control (source: mode.h:1205)
- `angle_control_run()` - Attitude control (source: mode.h:1151)

**Position/Velocity Limits**:
- `limit_set()` - Sets altitude and horizontal limits (source: mode.h:1125)
- `limit_check()` - Validates within limits (source: mode.h:1126)
- `limit_clear()` - Removes limits (source: mode.h:1123)
- `limit_init_time_and_pos()` - Initializes limit checking (source: mode.h:1124)

**Pause/Resume**:
- `pause()` - Pauses guided operation, holds position (source: mode.h:1159)
- `resume()` - Resumes from pause (source: mode.h:1160)

**Parameters**:
- `GUID_OPTIONS` - Bitmask controlling GUIDED behavior (source: mode.h:1179-1188):
  - Bit 0: Allow arming from TX in GUIDED
  - Bit 2: Ignore pilot yaw
  - Bit 3: SET_ATTITUDE_TARGET thrust as thrust (not climb rate)
  - Bit 4: Do not stabilize position XY
  - Bit 5: Do not stabilize velocity XY
  - Bit 6: Use WPNav for position control
  - Bit 7: Allow weathervaning
- `GUID_TIMEOUT` - Velocity/accel/angle control timeout (milliseconds)
- Similar navigation parameters as AUTO mode

**Use Cases**:
- Companion computer control (e.g., DroneKit, MAVSDK)
- Dynamic mission execution
- Object tracking and following
- Research and development
- External path planning algorithms

**Safety Notes**:
- Requires active MAVLink connection
- Timeout protection for velocity/acceleration commands (source: mode.h:1154)
- Requires terrain failsafe handling (source: mode.h:1084)
- Throttle high arming check can be skipped (source: mode.h:1092)
- Weathervaning support depends on GUID_OPTIONS (source: mode.h:1163-1165)
- Can override speed settings (source: mode.h:1130-1132)

---

### RTL (Mode 6)

**Description**: Return to Launch - automatic return to home location and landing. Multi-phase operation with climb, return, descent, and land.

**Source**: `/ArduCopter/mode_rtl.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1461)
- **Manual Throttle**: No (source: mode.h:1462)
- **Allows Arming**: No (source: mode.h:1463)
- **Autopilot Mode**: Yes (source: mode.h:1464)
- **Is Landing**: During FINAL_DESCENT and LAND states (source: mode.h:1496)

**RTL States** (source: mode.h:1483-1490):
- `STARTING` - RTL initialization
- `INITIAL_CLIMB` - Climbing to RTL altitude
- `RETURN_HOME` - Flying horizontally to home
- `LOITER_AT_HOME` - Loitering above home
- `FINAL_DESCENT` - Descending to land
- `LAND` - Final landing phase

**RTL Path Structure** (source: mode.h:1537-1544):
- `origin_point` - Location where RTL started
- `climb_target` - Altitude to climb to
- `return_target` - Home position to fly to
- `descent_target` - Position to descend at
- `land` - Whether to land or just loiter

**Key Functions**:
- `ModeRTL::init()` - Starts RTL sequence, builds path (source: mode.h:1455)
- `ModeRTL::run()` - Executes current RTL state (source: mode.h:1456-1459)
- `build_path()` - Constructs RTL flight path (source: mode.h:1531)
- `compute_return_target()` - Calculates home return point (source: mode.h:1532)
- `climb_start()` - Begins initial climb (source: mode.h:1526)
- `return_start()` - Starts return to home (source: mode.h:1527)
- `descent_start()` - Initiates final descent (source: mode.h:1517)
- `land_run()` - Executes landing (source: mode.h:1520)

**RTL Altitude Types** (source: mode.h:1501-1504):
- `RELATIVE` (0) - Altitude relative to home
- `TERRAIN` (1) - Altitude above terrain (requires rangefinder or terrain database)

**Parameters**:
- `RTL_ALT` - RTL return altitude in cm above home (source: Parameters.cpp:86)
- `RTL_CONE_SLOPE` - Defines cone for altitude climb (source: Parameters.cpp:95)
- `RTL_SPEED` - Horizontal return speed in cm/s (source: Parameters.cpp:99)
- `RTL_CLIMB_MIN` - Minimum initial climb
- `RTL_ALT_FINAL` - Final altitude before landing
- `RTL_LOIT_TIME` - Time to loiter at home before landing
- `RTL_ALT_TYPE` - Return altitude type (relative/terrain)
- `RTL_OPTIONS` - Bitmask (bit 2: Ignore pilot yaw) (source: mode.h:1559-1562)

**Cone Algorithm**:
- If within cone (defined by RTL_CONE_SLOPE), climb to cone altitude
- If outside cone, climb to full RTL_ALT
- Prevents unnecessary climbing when already high

**Terrain Following**:
- `terrain_following_allowed` - Flag for terrain mode (source: mode.h:1556)
- `restart_without_terrain()` - Fallback if terrain unavailable (source: mode.h:1498)
- `get_alt_type()` - Returns current altitude reference type (source: mode.h:1505)

**Use Cases**:
- Emergency return home procedure
- Failsafe return (battery, RC loss, etc.)
- End of mission return
- Manual command to return home

**Safety Notes**:
- Home point must be set (armed with GPS lock or manually set)
- Requires terrain failsafe handling (source: mode.h:1466)
- Will use LAND mode if initiated without GPS lock
- Pilot yaw control based on RTL_OPTIONS and default behavior (source: mode.h:1476)
- Can override speed with MAVLink commands (source: mode.h:1478-1480)
- Advanced failsafe support (source: mode.h:1468-1471)

---

### SMART_RTL (Mode 21)

**Description**: Smart Return to Launch - returns home by retracing the vehicle's flight path, avoiding obstacles encountered during outbound flight.

**Source**: `/ArduCopter/mode_smart_rtl.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1577)
- **Manual Throttle**: No (source: mode.h:1578)
- **Allows Arming**: No (source: mode.h:1579)
- **Autopilot Mode**: Yes (source: mode.h:1580)
- **Is Landing**: During DESCEND and LAND states (source: mode.h:1585)

**Smart RTL States** (source: mode.h:1589-1595):
- `WAIT_FOR_PATH_CLEANUP` - Waiting for path simplification
- `PATH_FOLLOW` - Following recorded path home
- `PRELAND_POSITION` - Moving to precision land position
- `DESCEND` - Descending to land
- `LAND` - Final landing phase

**Path Recording**:
- Vehicle continuously records GPS positions during flight
- Path stored in memory with intelligent simplification
- Removes unnecessary intermediate points to save memory
- `save_position()` - Records current position to path (source: mode.h:1582)

**Key Functions**:
- `ModeSmartRTL::init()` - Starts path-following return (source: mode.h:1574)
- `ModeSmartRTL::run()` - Executes current Smart RTL state (source: mode.h:1575)
- `exit()` - Cleanup, can restore backed-up point (source: mode.h:1583)
- `wait_cleanup_run()` - Waits for path processing (source: mode.h:1610)
- `path_follow_run()` - Follows recorded path (source: mode.h:1611)
- `pre_land_position_run()` - Positions for landing (source: mode.h:1612)
- `land()` - Initiates landing (source: mode.h:1613)

**Path Management**:
- `dest_NED_backup` - Backup of last path point (source: mode.h:1623)
- `path_follow_last_pop_fail_ms` - Tracks path retrieval failures (source: mode.h:1619)
- Path can be exited and resumed if mode changed before reaching home

**Parameters**:
- `SRTL_ACCURACY` - Path point accuracy (cm)
- `SRTL_POINTS` - Maximum number of path points to store
- Similar navigation and landing parameters as RTL

**Fallback Behavior**:
- If path unavailable or fails, switches to regular RTL mode
- Path can be cleared by disarming or manual command
- If path following fails for too long, may land in place

**Reporting** (source: mode.h:1603-1606):
- `get_wp()` - Current waypoint location
- `wp_distance_m()` - Distance to current path point
- `wp_bearing_deg()` - Bearing to current path point
- `crosstrack_error()` - Deviation from path

**Use Cases**:
- Return via known-safe path (avoids obstacles)
- Complex environments with obstacles
- Flights through canyons or urban areas
- Missions where direct return path may be unsafe

**Safety Notes**:
- Requires sufficient memory for path storage
- Path may not be available if memory full
- Falls back to regular RTL if path unavailable
- Pilot yaw control behavior similar to RTL (source: mode.h:1586)
- Useful in areas with obstacles between start and end points

---

### LAND (Mode 9)

**Description**: Automated landing with optional precision landing and position control. Safely brings vehicle to ground with controlled descent.

**Source**: `/ArduCopter/mode_land.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:1277)
- **Manual Throttle**: No (source: mode.h:1278)
- **Allows Arming**: No (source: mode.h:1279)
- **Autopilot Mode**: Yes (source: mode.h:1280)
- **Is Landing**: Yes (source: mode.h:1282)

**Landing Modes**:
- GPS mode: `gps_run()` - Landing with position control (source: mode.h:1303)
- No-GPS mode: `nogps_run()` - Landing with altitude control only (source: mode.h:1304)

**Position Control**:
- `controlling_position()` - Returns true if using position control (source: mode.h:1292)
- `do_not_use_GPS()` - Disables GPS position control (source: mode.h:1289)
- `control_position` - Flag for position control state (source: mode.h:1306)

**Precision Landing Support**:
- Can use precision landing sensors (IR-LOCK, etc.) if available
- Automatically tracks precision target during descent
- Falls back to standard landing if target lost

**Key Functions**:
- `ModeLand::init()` - Initializes landing sequence (source: mode.h:1274)
- `ModeLand::run()` - Executes landing logic (source: mode.h:1275)
- `land_run_normal_or_precland()` - Runs normal or precision landing (mode.h:235)
- `land_run_horizontal_control()` - Maintains horizontal position (mode.h:221)
- `land_run_vertical_control()` - Controls descent rate (mode.h:222)

**Landing Pause**:
- `set_land_pause()` - Pauses descent while maintaining position (source: mode.h:1294)
- `land_pause` - Pause state flag (source: mode.h:1309)
- Useful for obstacle avoidance during landing

**Landing Detection**:
- Monitors motor throttle and acceleration
- Detects ground contact
- Automatically disarms after landing (if configured)
- `land_start_time` - Time when landing started (source: mode.h:1308)

**Parameters**:
- `LAND_SPEED` - Descent rate in cm/s
- `LAND_SPEED_HIGH` - Initial descent rate for high altitudes
- `LAND_ALT_LOW` - Altitude to switch to final descent rate
- `LAND_REPOSITION` - Enable repositioning during descent

**Use Cases**:
- Normal end-of-flight landing
- Emergency landing procedure
- Failsafe landing
- Mission land waypoint execution
- Precision landing on targets

**Safety Notes**:
- Works with or without GPS
- Slower descent near ground for safety
- Can pause descent if obstacles detected
- Precision landing requires appropriate sensor
- Advanced failsafe support (source: mode.h:1284-1287)
- May drift if position control disabled and wind present

---

### CIRCLE (Mode 7)

**Description**: Orbit around a point of interest at constant radius and altitude. Useful for aerial photography and surveillance.

**Source**: `/ArduCopter/mode_circle.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:884)
- **Manual Throttle**: No (source: mode.h:885)
- **Allows Arming**: No (source: mode.h:886)
- **Autopilot Mode**: Yes (source: mode.h:887)

**Behavior**:
- Vehicle orbits around center point at specified radius
- Maintains constant altitude during orbit
- Can orbit clockwise or counter-clockwise
- Pilot can adjust orbit rate with roll stick
- Throttle controls altitude

**Key Functions**:
- `ModeCircle::init()` - Initializes circle mode with current position as center (source: mode.h:881)
- `ModeCircle::run()` - Maintains circular flight (source: mode.h:882)

**Speed Control**:
- `speed_changing` - Flag indicating pilot adjusting rate (source: mode.h:900)
- Pilot roll input modifies circle rate
- Can slow to stop or speed up orbit

**Reporting** (source: mode.h:894-895):
- `wp_distance_m()` - Distance to circle center (radius)
- `wp_bearing_deg()` - Current bearing from center

**Parameters**:
- `CIRCLE_RADIUS` - Orbit radius in cm (can be negative for counter-clockwise)
- `CIRCLE_RATE` - Rotation rate in deg/s
- `PILOT_SPEED_UP` - Climb rate
- `PILOT_SPEED_DN` - Descent rate

**Use Cases**:
- Aerial photography of structures or landscapes
- Surveillance of area or object
- Orbit shots for video production
- Inspection flights around towers or buildings

**Safety Notes**:
- Requires GPS lock to maintain circle
- May not maintain perfect circle in strong winds
- Pilot can override with stick inputs
- Radius and rate can be changed during flight

---

### BRAKE (Mode 17)

**Description**: Aggressive deceleration mode that brings vehicle to a rapid stop. Used as intermediate mode during autonomous operations.

**Source**: `/ArduCopter/mode_brake.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:854)
- **Manual Throttle**: No (source: mode.h:855)
- **Allows Arming**: No (source: mode.h:856)
- **Autopilot Mode**: No (source: mode.h:857)

**Behavior**:
- Aggressively brakes horizontal movement
- Maximum deceleration applied up to lean angle limits
- Maintains altitude during braking
- Can automatically transition to LOITER after stopping
- Does not accept pilot input during braking

**Key Functions**:
- `ModeBrake::init()` - Initializes brake controller (source: mode.h:851)
- `ModeBrake::run()` - Executes braking maneuver (source: mode.h:852)
- `timeout_to_loiter_ms()` - Sets auto-transition timeout (source: mode.h:859)

**Timeout Behavior**:
- `_timeout_start` - Time when brake started (source: mode.h:868)
- `_timeout_ms` - Timeout duration (source: mode.h:869)
- Automatically switches to LOITER after timeout if stopped

**Parameters**:
- Uses position controller braking parameters
- `LOIT_BRK_ACCEL` - Braking deceleration rate
- `LOIT_BRK_JERK` - Braking jerk limit
- `ANGLE_MAX` - Maximum lean angle during brake

**Use Cases**:
- Emergency stop during autonomous flight
- Transitional mode between flight modes
- Quick stop before mode transitions
- Intermediate step in failsafe sequences

**Safety Notes**:
- Not intended for manual pilot selection
- Typically used programmatically
- Very aggressive - may be uncomfortable
- Requires GPS for velocity estimation
- Usually followed by transition to another mode

---

### FOLLOW (Mode 23)

**Description**: Follows another vehicle or ground station using position reports. Maintains relative position and altitude offset.

**Source**: `/ArduCopter/mode_follow.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1918)
- **Manual Throttle**: No (source: mode.h:1919)
- **Allows Arming**: No (source: mode.h:1920)
- **Autopilot Mode**: Yes (source: mode.h:1921)

**Behavior**:
- Receives target position via MAVLink
- Maintains configured offset from target
- Adjusts position and altitude to maintain formation
- Can follow moving vehicles or stationary points
- Uses velocity feedforward for smooth tracking

**Key Functions**:
- `ModeFollow::init()` - Initializes follow mode (source: mode.h:1914)
- `ModeFollow::run()` - Tracks target position (source: mode.h:1916)
- `exit()` - Cleanup when exiting follow mode (source: mode.h:1915)

**Reporting** (source: mode.h:1929-1931):
- `get_wp()` - Current target waypoint
- `wp_distance_m()` - Distance to target
- `wp_bearing_deg()` - Bearing to target

**Data Logging**:
- `last_log_ms` - Timestamp of last velocity log (source: mode.h:1933)
- Logs desired velocity for analysis

**Parameters**:
- `FOLL_ENABLE` - Enable/disable follow mode
- `FOLL_SYSID` - MAVLink system ID of target to follow
- `FOLL_DIST_MAX` - Maximum distance to target before giving up
- `FOLL_OFS_X` - Offset in X direction (forward/back)
- `FOLL_OFS_Y` - Offset in Y direction (right/left)
- `FOLL_OFS_Z` - Offset in Z direction (up/down)
- `FOLL_YAW_BEHAVE` - Yaw behavior (face target, same heading, etc.)

**Use Cases**:
- Formation flying with multiple vehicles
- Following a ground vehicle
- Aerial filming of moving subjects
- Coordinated multi-vehicle operations
- Search and rescue operations

**Safety Notes**:
- Requires reliable MAVLink connection
- Target must provide position updates
- Will switch to alternative mode if target lost
- Maximum distance parameter prevents fly-aways
- Inherits from ModeGuided for control logic (source: mode.h:1906)

---

### ZIGZAG (Mode 24)

**Description**: Automated back-and-forth flight pattern between two saved points. Useful for survey and agricultural applications.

**Source**: `/ArduCopter/mode_zigzag.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1967)
- **Manual Throttle**: No (source: mode.h:1968)
- **Allows Arming**: Yes (source: mode.h:1969)
- **Autopilot Mode**: Yes (source: mode.h:1970)
- **User Takeoff**: Yes (source: mode.h:1971)

**ZigZag States** (source: mode.h:2014-2018):
- `STORING_POINTS` - Recording points A and B, pilot has control
- `AUTO` - Automated zigzag pattern execution
- `MANUAL_REGAIN` - Pilot override, manual control

**Auto States** (source: mode.h:2020-2024):
- `MANUAL` - Not in ZigZag auto mode
- `AB_MOVING` - Flying between points A and B
- `SIDEWAYS` - Moving perpendicular to establish new line

**Behavior**:
- Pilot saves point A at one end of field
- Pilot flies to other end and saves point B
- Vehicle automatically flies A-B-A pattern
- Sideways movement creates parallel lines
- Can spray or perform operations during runs

**Key Functions**:
- `ModeZigZag::init()` - Initializes zigzag mode (source: mode.h:1958)
- `ModeZigZag::run()` - Executes manual or auto control (source: mode.h:1960)
- `save_or_move_to_destination()` - Records point or navigates (source: mode.h:1974)
- `return_to_manual_control()` - Returns control to pilot (source: mode.h:1977)
- `run_auto()` - Automated zigzag execution (source: mode.h:1963)
- `suspend_auto()` - Pauses automation (source: mode.h:1964)
- `init_auto()` - Starts automated sequence (source: mode.h:1965)

**Direction Control** (source: mode.h:1951-1956):
- `FORWARD` - Moving forward from yaw
- `RIGHT` - Moving right from yaw
- `BACKWARD` - Moving backward from yaw
- `LEFT` - Moving left from yaw

**Parameters** (source: mode.h:2005-2012):
- `ZIGZ_AUTO` (_auto_enabled) - Enable zigzag automation
- `ZIGZ_SPRAY` (_spray_enabled) - Auto spray control
- `ZIGZ_WP_DELAY` (_wp_delay_s) - Delay at waypoints (seconds)
- `ZIGZ_SIDE_DIST` (_side_dist_m) - Sideways distance between lines (meters)
- `ZIGZ_DIRECTION` (_direction) - Direction for sideways movement
- `ZIGZ_LINE_NUM` (_line_num) - Total number of lines to fly

**Sprayer Integration**:
- `spray()` - Controls sprayer on/off (source: mode.h:1995)
- Automatically enabled during line runs if configured

**Use Cases**:
- Agricultural spraying operations
- Survey and mapping of rectangular areas
- Inspection of large flat surfaces
- Automated crop monitoring
- Any application requiring parallel line coverage

**Safety Notes**:
- Requires good GPS accuracy
- Points A and B define flight area boundaries
- Pilot can regain control at any time
- Wind can affect pattern accuracy
- Line tracking uses waypoint navigation (source: mode.h:1985-1987)

---

## Special Purpose Modes

### FLIP (Mode 14)

**Description**: Automated flip maneuver on roll or pitch axis. Performs complete 360° rotation and recovers to level flight.

**Source**: `/ArduCopter/mode_flip.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:942)
- **Manual Throttle**: No (source: mode.h:943)
- **Allows Arming**: No (source: mode.h:944)
- **Autopilot Mode**: No (source: mode.h:945)
- **Crash Check**: Disabled during flip (source: mode.h:946)

**Flip States** (source: mode.h:958-965):
- `Start` - Initialization, gaining altitude
- `Roll` - Rolling flip in progress
- `Pitch_A` - First pitch flip phase
- `Pitch_B` - Second pitch flip phase
- `Recover` - Recovering to level flight
- `Abandon` - Abort flip, return to original mode

**Behavior**:
- Initiated by pilot switch or command
- Vehicle climbs briefly for safety margin
- Performs rapid rotation on selected axis
- Throttle boosted during flip
- Automatically recovers to level attitude
- Returns to original flight mode after completion

**Key Functions**:
- `ModeFlip::init()` - Stores original attitude and mode (source: mode.h:939)
- `ModeFlip::run()` - Executes flip state machine (source: mode.h:940)

**Flip State Variables** (source: mode.h:956-970):
- `orig_attitude_euler_rad` - Original attitude before flip
- `_state` - Current flip state
- `orig_control_mode` - Mode to return to after flip
- `start_time_ms` - Flip start timestamp
- `roll_dir` - Roll direction (-1 left, 1 right)
- `pitch_dir` - Pitch direction (-1 forward, 1 back)

**Parameters**:
- `FLIP_ENABLED` - Enable/disable flip mode
- `FLIP_TIMEOUT_MS` - Maximum time allowed for flip
- Internal timing parameters control flip speed

**Use Cases**:
- Aerobatic demonstrations
- Clearing debris from propellers
- Entertainment and sport flying
- Testing vehicle agility and recovery

**Safety Notes**:
- Requires significant altitude (minimum 10m recommended)
- Not suitable for beginners
- Can only be initiated from certain flight modes
- Crash detection disabled during maneuver
- Automatically abandons flip if taking too long
- Vehicle must have sufficient power/thrust ratio

---

### AUTOTUNE (Mode 15)

**Description**: Automated PID tuning for roll, pitch, and yaw controllers. Systematically tests vehicle response and optimizes gains.

**Source**: `/ArduCopter/mode_autotune.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:829)
- **Manual Throttle**: No (source: mode.h:830)
- **Allows Arming**: No (source: mode.h:831)
- **Autopilot Mode**: No (source: mode.h:832)

**Behavior**:
- Performs automated test maneuvers on each axis
- Measures vehicle response to control inputs
- Calculates optimal PID gains
- Pilot can observe and control tuning process
- Can tune one axis at a time or all axes
- Saves gains when complete

**AutoTune Wrapper Class** (source: mode.h:796-813):
- Inherits from `AC_AutoTune_Multi` or `AC_AutoTune_Heli`
- `init()` - Starts autotune sequence (source: mode.h:802)
- `run()` - Executes tuning maneuvers (source: mode.h:803)
- `position_ok()` - Checks if position suitable for tuning (source: mode.h:806)
- `get_pilot_desired_climb_rate_cms()` - Pilot altitude control (source: mode.h:807)
- `init_z_limits()` - Initializes altitude constraints (source: mode.h:809)

**Key Functions**:
- `ModeAutoTune::init()` - Initializes autotune mode (source: mode.h:825)
- `ModeAutoTune::run()` - Executes tuning process (source: mode.h:827)
- `ModeAutoTune::exit()` - Saves or reverts gains (source: mode.h:826)

**Tuning Process**:
- Tests angle P gains
- Tests rate P, I, and D gains
- Performs step inputs and measures response
- Adjusts gains based on measured performance
- Ensures stability margins

**Parameters**:
- `AUTOTUNE_AXES` - Which axes to tune (bitmask: roll, pitch, yaw)
- `AUTOTUNE_AGGR` - Aggressiveness of tuning (0.05 to 0.10)
- `AUTOTUNE_MIN_D` - Minimum D gain
- Internal parameters control test signals

**Use Cases**:
- Initial vehicle setup after build
- After significant weight or configuration changes
- Optimizing performance for specific operations
- Recovering from poor manual tuning
- Regular tuning maintenance

**Safety Notes**:
- Requires calm wind conditions
- Needs GPS for position hold (unless using optical flow)
- Pilot must monitor entire process
- Can abort and revert gains at any time
- Requires sufficient battery for complete process (typically 10+ minutes)
- Position hold must be stable during tuning

---

### THROW (Mode 18)

**Description**: Throw-to-launch mode for hand launching vehicle. Motors start after detecting throw motion.

**Source**: `/ArduCopter/mode_throw.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1791)
- **Manual Throttle**: No (source: mode.h:1792)
- **Allows Arming**: Yes (source: mode.h:1793)
- **Autopilot Mode**: No (source: mode.h:1794)

**Throw Stages** (source: mode.h:1820-1827):
- `Throw_Disarmed` - Vehicle disarmed, waiting
- `Throw_Detecting` - Armed, detecting throw motion
- `Throw_Wait_Throttle_Unlimited` - Throttle system initialization
- `Throw_Uprighting` - Orienting to level attitude
- `Throw_HgtStabilise` - Stabilizing altitude
- `Throw_PosHold` - Transitioning to position hold

**Throw Types** (source: mode.h:1797-1800):
- `Upward` (0) - Throw upward launch
- `Drop` (1) - Drop-and-catch style

**Pre-Throw Motor State** (source: mode.h:1802-1805):
- `STOPPED` (0) - Motors stopped until throw detected
- `RUNNING` (1) - Motors running at low throttle

**Behavior**:
- Arm vehicle while holding it
- Vehicle detects throw via accelerometers
- Motors start automatically after throw detected
- Vehicle uprights itself and stabilizes
- Transitions to position hold

**Key Functions**:
- `ModeThrow::init()` - Prepares throw detection (source: mode.h:1788)
- `ModeThrow::run()` - Monitors for throw and stabilizes (source: mode.h:1789)
- `throw_detected()` - Detects throw acceleration pattern (source: mode.h:1814)
- `throw_position_good()` - Checks GPS quality (source: mode.h:1815)
- `throw_height_good()` - Validates altitude (source: mode.h:1816)
- `throw_attitude_good()` - Checks attitude recovery (source: mode.h:1817)

**Throw Detection Variables** (source: mode.h:1829-1834):
- `stage` - Current throw stage
- `prev_stage` - Previous stage for logging
- `last_log_ms` - Last log timestamp
- `nextmode_attempted` - Flag for mode transition
- `free_fall_start_ms` - Free fall detection time
- `free_fall_start_velz` - Velocity at free fall start

**Parameters**:
- `THROW_NEXTMODE` - Mode to switch to after stabilization
- `THROW_MOT_START` - Motor behavior (0=stop until throw, 1=run)
- `THROW_TYPE` - Throw type selection

**Use Cases**:
- Hand launching in confined spaces
- Quick deployment without ground clearance
- Launching from boats or vehicles
- Situations where takeoff space limited
- Emergency rapid deployment

**Safety Notes**:
- Practice with simulation first
- Requires firm, upward throw
- GPS lock required before throw
- Motors will spin up after throw - be clear of propellers
- Not recommended for beginners
- Adequate altitude required for stabilization (minimum 2-3m)

---

### AVOID_ADSB (Mode 19)

**Description**: Collision avoidance mode using ADS-B transponder data from nearby aircraft. Automatically maneuvers to avoid conflicts.

**Source**: `/ArduCopter/mode_avoid_adsb.cpp`

**Characteristics**:
- **GPS Required**: Yes (source: mode.h:1888)
- **Manual Throttle**: No (source: mode.h:1889)
- **Allows Arming**: No (source: mode.h:1890)
- **Autopilot Mode**: Yes (source: mode.h:1891)

**Behavior**:
- Monitors ADS-B transponder data for nearby aircraft
- Calculates collision risk based on position and velocity
- Automatically initiates avoidance maneuvers
- Returns to original mode after threat passes
- Inherits control from GUIDED mode

**Key Functions**:
- `ModeAvoidADSB::init()` - Initializes avoidance mode (source: mode.h:1885)
- `ModeAvoidADSB::run()` - Executes avoidance maneuver (source: mode.h:1886)
- `set_velocity()` - Commands avoidance velocity (source: mode.h:1893)

**Inheritance**:
- Inherits from `ModeGuided` (source: mode.h:1878)
- Uses guided mode control infrastructure
- Velocity control for avoidance maneuvers

**Parameters**:
- `AVD_ENABLE` - Enable/disable avoidance system
- `AVD_F_DIST_XY` - Horizontal distance threshold
- `AVD_F_DIST_Z` - Vertical distance threshold
- `AVD_F_TIME` - Time to collision threshold
- ADS-B receiver parameters

**Use Cases**:
- Operating in controlled airspace
- Areas with manned aircraft traffic
- Regulatory compliance for collision avoidance
- Enhanced safety in mixed airspace
- Automatic conflict resolution

**Safety Notes**:
- Requires ADS-B receiver hardware
- Only detects aircraft with ADS-B transponders
- Not a substitute for visual observation
- May initiate aggressive maneuvers
- Should be tested thoroughly before use
- Pilot should monitor and be ready to override

---

### GUIDED_NOGPS (Mode 20)

**Description**: Guided mode variant that accepts only attitude and altitude commands, for GPS-denied environments.

**Source**: `/ArduCopter/mode_guided_nogps.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:1253)
- **Manual Throttle**: No (source: mode.h:1254)
- **Autopilot Mode**: Yes (source: mode.h:1255)

**Behavior**:
- Similar to GUIDED but without position control
- Accepts attitude angle commands
- Accepts altitude commands
- Does not accept position or velocity commands
- Useful for indoor or GPS-denied flight

**Key Functions**:
- `ModeGuidedNoGPS::init()` - Initializes no-GPS guided control (source: mode.h:1250)
- `ModeGuidedNoGPS::run()` - Executes attitude/altitude control (source: mode.h:1251)

**Inheritance**:
- Inherits from `ModeGuided` (source: mode.h:1243)
- Restricts to attitude and altitude commands only
- Removes position-based commands

**Accepted Commands**:
- SET_ATTITUDE_TARGET - Attitude quaternion and climb rate
- Altitude hold commands
- Angular rate commands

**Rejected Commands**:
- SET_POSITION_TARGET_LOCAL_NED - Position commands not accepted
- Velocity commands not supported
- GPS-based navigation not supported

**Parameters**:
- Similar guided mode parameters
- Position control parameters ignored

**Use Cases**:
- Indoor flight with companion computer
- GPS-denied environment control
- Attitude-only external control
- Research and development without GPS
- Backup control mode

**Safety Notes**:
- No position hold - will drift horizontally
- Requires external control system
- Altitude control using barometer only
- Not suitable for outdoor use without GPS
- Pilot should be ready to take manual control

---

### SYSTEMID (Mode 25)

**Description**: System identification mode for analyzing vehicle dynamics. Injects test signals and records response for model development.

**Source**: `/ArduCopter/mode_systemid.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:1712)
- **Manual Throttle**: Yes (source: mode.h:1713)
- **Allows Arming**: No (source: mode.h:1714)
- **Autopilot Mode**: No (source: mode.h:1715)
- **Logs Attitude**: Yes (source: mode.h:1716)

**SystemID States** (source: mode.h:1775-1778):
- `SYSTEMID_STATE_STOPPED` - Not running
- `SYSTEMID_STATE_TESTING` - Test in progress

**Axis Types** (source: mode.h:1734-1755):
- `NONE` (0) - No axis selected
- `INPUT_ROLL` (1) - Angle input roll
- `INPUT_PITCH` (2) - Angle input pitch
- `INPUT_YAW` (3) - Angle input yaw
- `RECOVER_ROLL/PITCH/YAW` (4-6) - Recovery testing
- `RATE_ROLL/PITCH/YAW` (7-9) - Rate loop testing
- `MIX_ROLL/PITCH/YAW/THROTTLE` (10-13) - Mixer testing
- `DISTURB_POS_LAT/LONG` (14-15) - Position disturbance
- `DISTURB_VEL_LAT/LONG` (16-17) - Velocity disturbance
- `INPUT_VEL_LAT/LONG` (18-19) - Velocity input

**Chirp Signal**:
- `chirp_input` - Chirp waveform generator (source: mode.h:1722)
- Sweeps frequency from start to stop
- Excites system across frequency range

**Key Functions**:
- `ModeSystemId::init()` - Initializes system ID test (source: mode.h:1708)
- `ModeSystemId::run()` - Executes test and records data (source: mode.h:1709)
- `ModeSystemId::exit()` - Stops test and cleanup (source: mode.h:1710)
- `log_data()` - Logs system ID data (source: mode.h:1731)
- `is_poscontrol_axis_type()` - Checks if position control test (source: mode.h:1732)

**Waveform Parameters** (source: mode.h:1757-1763):
- `axis` - Axis selection
- `waveform_magnitude` - Chirp amplitude
- `frequency_start` - Start frequency
- `frequency_stop` - End frequency
- `time_fade_in` - Fade-in time
- `time_record` - Recording duration
- `time_fade_out` - Fade-out time

**Use Cases**:
- Dynamic model identification
- Controller tuning optimization
- Research and development
- Flight dynamics analysis
- System characterization

**Safety Notes**:
- Should only be used by experienced users
- Requires stable hover capability
- Pilot must monitor throughout test
- Can produce aggressive maneuvers
- Requires post-processing of logs
- Adequate altitude and clear airspace required

---

### AUTOROTATE (Mode 26)

**Description**: Helicopter autorotation mode for engine-failure emergency landing. Uses rotor momentum for controlled descent.

**Source**: `/ArduCopter/mode_autorotate.cpp`

**Note**: Only available for helicopter frame configuration.

**Characteristics**:
- **GPS Required**: No (source: mode.h:2047)
- **Manual Throttle**: No (source: mode.h:2048)
- **Allows Arming**: No (source: mode.h:2049)
- **Autopilot Mode**: Yes (source: mode.h:2046)

**Autorotation Phases** (source: mode.h:2063-2074):
- `ENTRY_INIT` - Initialization of entry phase
- `ENTRY` - Entry phase, establishing autorotation
- `GLIDE_INIT` - Initialize glide phase
- `GLIDE` - Steady glide descent
- `FLARE_INIT` - Initialize flare phase
- `FLARE` - Flare to slow descent
- `TOUCH_DOWN_INIT` - Touchdown initialization
- `TOUCH_DOWN` - Ground contact phase
- `LANDED_INIT` - Landed initialization
- `LANDED` - Final landed state

**Behavior**:
- Automatically entered on engine failure (if configured)
- Manages rotor RPM through collective pitch control
- Establishes steady glide descent
- Flares before touchdown to reduce descent rate
- Uses stored rotor energy for soft landing

**Key Functions**:
- `ModeAutorotate::init()` - Begins autorotation sequence (source: mode.h:2043)
- `ModeAutorotate::run()` - Executes current phase (source: mode.h:2044)

**Phase Variables** (source: mode.h:2060-2061):
- `_entry_time_start_ms` - Entry phase start time
- `_last_logged_ms` - Last log timestamp

**Parameters**:
- `H_RSC_AROT_ENA` - Enable autorotation
- `H_RSC_AROT_RAMP` - RPM ramp rate
- `H_RSC_AROT_RUNUP` - Runup time
- Various phase-specific timing and control parameters

**Use Cases**:
- Emergency landing after engine failure (helicopters)
- Training for autorotation procedures
- Failsafe mode for single-engine helicopters
- Safety feature for helicopter operations

**Safety Notes**:
- HELICOPTER FRAME ONLY - not for multirotors
- Requires proper helicopter setup
- Should be tested at altitude first
- Pilot should be trained in autorotation
- Proper rotor head configuration essential
- Collective and throttle curves must be configured correctly

---

### TURTLE (Mode 28)

**Description**: Flip recovery mode to right vehicle after crash. Reverses motors to flip upright.

**Source**: `/ArduCopter/mode_turtle.cpp`

**Characteristics**:
- **GPS Required**: No (source: mode.h:1849)
- **Manual Throttle**: Yes (source: mode.h:1850)
- **Allows Arming**: Conditional (source: mode.h:1851)
- **Autopilot Mode**: No (source: mode.h:1852)
- **RC Failsafe Entry**: Disabled (source: mode.h:1855)

**Behavior**:
- Entered after vehicle flips upside-down (crash)
- Reverses motor directions to flip vehicle upright
- Pilot controls flip direction with stick inputs
- Motors spin in reverse to generate righting force
- Returns to normal mode after successful flip

**Key Functions**:
- `ModeTurtle::init()` - Initializes turtle mode (source: mode.h:1845)
- `ModeTurtle::run()` - Executes flip recovery (source: mode.h:1846)
- `ModeTurtle::exit()` - Restores normal motor direction (source: mode.h:1847)
- `change_motor_direction()` - Reverses motor directions (source: mode.h:1853)
- `output_to_motors()` - Sends reversed motor commands (source: mode.h:1854)
- `arm_motors()` - Arms for turtle operation (source: mode.h:1862)
- `disarm_motors()` - Disarms after recovery (source: mode.h:1863)

**Motor Control Variables** (source: mode.h:1865-1867):
- `motors_output` - Motor output value
- `motors_input` - 2D input vector for direction
- `last_throttle_warning_output_ms` - Warning timing

**Thread Safety** (source: mode.h:1870):
- `msem` - Semaphore protecting motor state
- `shutdown` - Shutdown flag

**Parameters**:
- `TURTLE_ANG_VEL` - Angular velocity for flip
- `TURTLE_RATE` - Flip rate
- Motor direction configuration

**Use Cases**:
- Recovery after crash landing upside-down
- Racing/acro operations where crashes common
- Quick recovery without manual intervention
- FPV racing scenarios
- Situations where manual flip not possible

**Safety Notes**:
- Requires bidirectional motor ESCs (DShot protocol)
- Motors will spin - keep clear of propellers
- May damage vehicle if surface not level
- Not available on all motor configurations
- Requires proper ESC configuration
- Throttle must be at zero to enter mode

---

### GUIDED_CUSTOM (Mode 31)

**Description**: Custom guided mode variant for scripting/external control with user-defined mode number and names.

**Source**: `/ArduCopter/mode_guided_custom.cpp`

**Characteristics**:
- **GPS Required**: Yes (inherited from GUIDED)
- **Manual Throttle**: No (inherited from GUIDED)
- **Autopilot Mode**: Yes (inherited from GUIDED)

**Purpose**:
- Allows Lua scripting to define custom flight modes
- Provides unique mode number separate from standard GUIDED
- Custom mode name for GCS display
- Full guided mode functionality with custom identity

**Key Features**:
- Custom mode number assignment (source: mode.h:1228)
- Custom full name and 4-character short name (source: mode.h:1230-1231)
- Custom mode state object (source: mode.h:1233)

**Constructor** (source: mode.h:1224):
- `ModeGuidedCustom(Number _number, const char* _full_name, const char* _short_name)`
- Registers custom mode with specified identifiers

**Key Functions**:
- `init()` - Initializes custom guided mode (source: mode.h:1226)
- Inherits all guided mode control methods
- `mode_number()` - Returns custom mode number (source: mode.h:1228)
- `name()` - Returns custom full name (source: mode.h:1230)
- `name4()` - Returns custom 4-char name (source: mode.h:1231)

**Custom State**:
- `state` - Custom mode state accessible to scripts (source: mode.h:1233)
- Type: `AP_Vehicle::custom_mode_state`
- Allows scripts to store mode-specific data

**Inheritance**:
- Inherits from `ModeGuided` (source: mode.h:1221)
- Full guided mode control capabilities
- Position, velocity, acceleration control
- Attitude control

**Use Cases**:
- Advanced scripting applications
- Custom autopilot behaviors
- Research and development
- Application-specific flight modes
- External control systems with custom modes

**Safety Notes**:
- Requires AP_SCRIPTING_ENABLED
- Script must properly implement mode behavior
- Inherits guided mode safety features
- Requires thorough testing before use
- Script errors can affect flight safety

---

## Mode Transitions

### Mode Transition Matrix

The following table shows which modes can be entered from which other modes. Some transitions are restricted based on vehicle state, GPS availability, or configuration.

**Source**: `/ArduCopter/mode.cpp` function `mode_from_mode_num()` (lines 32-160) and individual mode `init()` functions.

### Transition Rules

**General Transition Requirements**:
1. Target mode must be compiled in (MODE_*_ENABLED)
2. Target mode's hardware requirements met (GPS, etc.)
3. Vehicle state compatible with target mode
4. No conflicting failsafe conditions
5. `init(ignore_checks)` must succeed

**Mode Change Function**: `Mode::set_mode(Mode::Number mode, ModeReason reason)`
- Source: mode.h:399
- Returns true if mode change successful
- Records mode change reason for logging
- Calls `exit()` on old mode, `init()` on new mode

### Common Transition Paths

**Normal Operations**:
```
STABILIZE → ALT_HOLD → LOITER → AUTO → RTL → LAND
```

**Training Progression**:
```
STABILIZE → ALT_HOLD → LOITER → POSHOLD → SPORT
```

**Autonomous Mission**:
```
LOITER → AUTO → (mission execution) → RTL → LAND
```

**Emergency Procedures**:
```
Any Mode → LAND (emergency landing)
Any Mode → RTL (return home)
Any Mode → BRAKE (emergency stop, then → LOITER)
```

### Mode-Specific Transition Restrictions

**Cannot ARM in These Modes**:
- AUTO (unless AUTO_OPTIONS bit 0 set)
- GUIDED (unless GUID_OPTIONS bit 0 set)
- RTL
- CIRCLE
- LAND
- BRAKE
- FLIP
- AUTOTUNE
- AVOID_ADSB
- FOLLOW
- SMART_RTL
- AUTOROTATE

**Requires GPS Lock**:
- LOITER
- POSHOLD
- GUIDED
- AUTO
- RTL
- SMART_RTL
- CIRCLE
- BRAKE
- DRIFT
- FOLLOW
- ZIGZAG
- THROW
- AVOID_ADSB

**Cannot Enter During RC Failsafe**:
- STABILIZE
- ACRO
- DRIFT
- TURTLE

**Automatic Transitions**:
- **BRAKE → LOITER**: After vehicle stops and timeout expires
- **THROW → LOITER** (or configured next mode): After throw sequence completes
- **FLIP → original mode**: After flip completes
- **AVOID_ADSB → original mode**: After collision threat passes
- **AUTO → LAND**: On final mission command or low battery
- **SMART_RTL → RTL**: If path unavailable
- **SMART_RTL → LAND**: If path following fails

### Failsafe Mode Transitions

**Battery Failsafe**:
```
Current Mode → RTL (or LAND if close to home)
```

**RC Failsafe**:
```
Current Mode → RTL (or LAND, or SMART_RTL based on FS_THR_ENABLE)
```

**GCS Failsafe**:
```
Current Mode → RTL (or LAND based on FS_GCS_ENABLE)
```

**EKF Failsafe**:
```
GPS-dependent Mode → LAND (in place)
```

**Crash Detected**:
```
Current Mode → LAND (immediate landing)
```

### Mode Reason Enumeration

Mode changes are logged with reasons for analysis:
- `UNKNOWN` - Reason not specified
- `TX_COMMAND` - Pilot mode switch
- `GCS_COMMAND` - Ground station command
- `RADIO_FAILSAFE` - RC signal loss
- `BATTERY_FAILSAFE` - Low battery
- `GCS_FAILSAFE` - GCS connection loss
- `EKF_FAILSAFE` - Navigation failure
- `GPS_GLITCH` - GPS anomaly
- `MISSION_END` - Mission completion
- `THROTTLE_LAND_ESCAPE` - Pilot override of landing
- `FENCE_BREACHED` - Geofence violation
- `TERRAIN_FAILSAFE` - Terrain following failure
- `BRAKE_TIMEOUT` - Brake mode timeout
- `FLIP_COMPLETE` - Flip maneuver done
- And others...

---

## Safety Considerations

### Pre-Flight Safety Checks

**Mode Compatibility Verification**:
1. Ensure GPS modes only used with GPS lock (HDOP < 2.0)
2. Verify home position set before using RTL/SMART_RTL
3. Test mode transitions on ground before flight
4. Configure failsafe modes appropriate for environment
5. Verify mode switch assignments correct

**Configuration Validation**:
- Check `MODE_*_ENABLED` for desired modes compiled in
- Verify parameter sets appropriate for modes used
- Test autonomous modes in simulator first
- Validate mission waypoints before AUTO flight

### In-Flight Safety

**Mode-Specific Warnings**:

**ACRO Mode**:
- No self-leveling - can enter any attitude
- Requires experienced pilot for recovery
- Not recommended over water or in low visibility

**FLIP Mode**:
- Requires minimum 10m altitude
- Ensure adequate power/thrust ratio
- Clear area of obstacles and people
- Not for use over water

**THROW Mode**:
- Practice in simulator extensively first
- Requires firm throw motion
- Motors will spin after throw - stay clear
- GPS lock required before throwing

**AUTO Mode**:
- Monitor mission execution continuously
- Be ready to switch to manual mode
- Verify waypoints reasonable before mission
- Understand mission commands behavior

**LAND Mode**:
- Will land immediately at current location
- Check ground clear before landing
- May not avoid obstacles during descent
- Consider precision landing if available

### Altitude Safety

**Minimum Safe Altitudes by Mode**:
- STABILIZE: Ground level OK (manual control)
- ACRO: 5m+ (recovery altitude)
- ALT_HOLD: 2m+ (altitude control initialization)
- LOITER: 2m+ (position hold initialization)
- FLIP: 10m+ (flip recovery margin)
- RTL: Configured RTL_ALT
- LAND: Any altitude (initiates descent)

**Maximum Altitude Limits**:
- Regulatory limits (120m/400ft in many jurisdictions)
- `FENCE_ALT_MAX` parameter if fence enabled
- Battery reserve for safe return
- Visual line-of-sight requirements

### GPS-Related Safety

**GPS Quality Requirements**:
- HDOP < 2.0 for autonomous modes
- Minimum 6 satellites for stable lock
- 3D fix required (not 2D)
- Home position valid before takeoff

**GPS Loss Behavior**:
- GPS modes switch to LAND (or configured failsafe)
- Non-GPS modes (STABILIZE, ALT_HOLD) unaffected
- SMART_RTL falls back to RTL, then LAND
- Position hold modes drift without GPS

### Failsafe Configuration

**Critical Failsafe Parameters**:
```
FS_THR_ENABLE    - RC failsafe action
FS_THR_VALUE     - RC failsafe trigger PWM
FS_GCS_ENABLE    - GCS failsafe action
FS_BATT_ENABLE   - Battery failsafe action
FS_BATT_VOLTAGE  - Battery voltage trigger
FS_BATT_MAH      - Battery capacity trigger
FS_EKF_THRESH    - EKF failsafe threshold
FS_CRASH_CHECK   - Crash detection enable
```

**Recommended Failsafe Settings**:
- Enable all applicable failsafes
- Test failsafe behavior before flight
- Set conservative battery thresholds
- Configure appropriate failsafe modes for environment

### Environmental Limitations

**Wind Limitations**:
- GPS modes may not hold position in excessive wind
- POSHOLD better than LOITER in gusty conditions
- Maximum wind: approximately 2/3 of maximum vehicle speed
- Consider wind effect on battery endurance

**Weather Considerations**:
- Rain: risk to electronics (weatherproofing required)
- Cold: reduced battery performance
- Heat: electronics overheating risk
- Visibility: affects visual line-of-sight

**Terrain Considerations**:
- RTL altitude must clear obstacles
- Terrain following requires rangefinder or terrain database
- Precision landing needs clear landing zone
- Magnetic interference near metal structures

### Emergency Procedures

**Loss of Control**:
1. Switch to STABILIZE mode
2. If no response, switch to LAND
3. If still no control, disarm (ground impact)

**Flyaway**:
1. Attempt mode switch to RTL or LAND
2. Use geofence if configured
3. Consider disarm if heading to danger
4. Log incident for analysis

**Low Battery**:
1. Initiate RTL immediately
2. Monitor battery voltage
3. Switch to LAND if voltage critical
4. Accept nearest safe landing spot

**GPS Loss in AUTO/GUIDED**:
1. Vehicle should auto-switch to LAND
2. If not, manually select LAND or STABILIZE
3. Land as soon as possible

---

## Parameter Dependencies

### Core Mode Parameters

**Altitude Control** (affects ALT_HOLD, LOITER, GUIDED, AUTO):
```
PILOT_SPEED_UP       - Maximum climb rate (cm/s), default: 250
PILOT_SPEED_DN       - Maximum descent rate (cm/s), default: 150
PILOT_ACCEL_Z        - Vertical acceleration (cm/s²), default: 250
PSC_ACCZ_P           - Altitude controller P gain
PSC_ACCZ_I           - Altitude controller I gain
PSC_ACCZ_D           - Altitude controller D gain
```

**Position Control** (affects LOITER, POSHOLD, GUIDED, AUTO):
```
LOIT_SPEED           - Maximum horizontal speed (cm/s), default: 1250
LOIT_ACC_MAX         - Maximum horizontal acceleration (cm/s²)
LOIT_BRK_ACCEL       - Braking deceleration (cm/s²), default: 250
LOIT_BRK_JERK        - Braking jerk limit (m/s³), default: 500
PSC_POSXY_P          - Position controller P gain
PSC_VELXY_P          - Velocity controller P gain
PSC_VELXY_I          - Velocity controller I gain
PSC_VELXY_D          - Velocity controller D gain
```

**Waypoint Navigation** (affects AUTO, GUIDED, RTL):
```
WPNAV_SPEED          - Default horizontal speed (cm/s), default: 500
WPNAV_SPEED_UP       - Climb speed (cm/s), default: 250
WPNAV_SPEED_DN       - Descent speed (cm/s), default: 150
WPNAV_RADIUS         - Waypoint radius (cm), default: 200
WPNAV_ACCEL          - Horizontal acceleration (cm/s²), default: 100
WPNAV_ACCEL_Z        - Vertical acceleration (cm/s²), default: 100
```

### Mode-Specific Parameters

**ACRO Mode**:
```
ACRO_RP_P            - Roll/Pitch rate P gain, default: 4.5
ACRO_YAW_P           - Yaw rate P gain, default: 4.5
ACRO_BAL_ROLL        - Roll auto-level strength, default: 0
ACRO_BAL_PITCH       - Pitch auto-level strength, default: 0
ACRO_TRAINER         - Trainer mode (0=off, 1=leveling, 2=limited)
ACRO_RP_RATE         - Max roll/pitch rate (deg/s), default: 180
ACRO_Y_RATE          - Max yaw rate (deg/s), default: 90
ACRO_RP_EXPO         - Roll/Pitch expo, default: 0
ACRO_Y_EXPO          - Yaw expo, default: 0
ACRO_THR_MID         - Throttle mid-point, default: 0.5
ACRO_OPTIONS         - Options bitmask (bit 0: air mode)
```

**RTL Mode**:
```
RTL_ALT              - Return altitude (cm), default: 1500
RTL_ALT_FINAL        - Final altitude before land (cm), default: 0
RTL_CLIMB_MIN        - Minimum initial climb (cm), default: 0
RTL_CONE_SLOPE       - Cone slope for alt calc, default: 3.0
RTL_SPEED            - Horizontal speed (cm/s), default: 0 (uses WPNAV_SPEED)
RTL_LOIT_TIME        - Loiter time at home (ms), default: 5000
RTL_ALT_TYPE         - Altitude type (0=relative, 1=terrain)
RTL_OPTIONS          - Options bitmask (bit 2: ignore pilot yaw)
```

**AUTO Mode**:
```
AUTO_OPTIONS         - Options bitmask:
                       Bit 0: Allow arming
                       Bit 1: Allow takeoff without raising throttle
                       Bit 2: Ignore pilot yaw
                       Bit 7: Allow weathervaning
WP_YAW_BEHAVIOR      - Yaw behavior (0=never, 1=face next wp, 2=face next wp except RTL, 3=face along GPS course)
```

**GUIDED Mode**:
```
GUID_TIMEOUT         - Vel/accel/angle timeout (s), default: 3.0
GUID_OPTIONS         - Options bitmask:
                       Bit 0: Allow arming from TX
                       Bit 2: Ignore pilot yaw
                       Bit 3: SetAttitudeTarget thrust interpretation
                       Bit 4: Do not stabilize position XY
                       Bit 5: Do not stabilize velocity XY
                       Bit 6: Use WPNav for position control
                       Bit 7: Allow weathervaning
```

**POSHOLD Mode**:
```
PHLD_BRAKE_RATE      - Braking rate (cm/s/s), default: 8
PHLD_BRAKE_ANGLE     - Maximum brake lean angle (cdeg), default: 3000
```

**DRIFT Mode**:
```
DRIFT_SPEEDGAIN      - Speed gain, default: 8
```

**CIRCLE Mode**:
```
CIRCLE_RADIUS        - Circle radius (cm), default: 1000
CIRCLE_RATE          - Circle rate (deg/s), default: 20.0
```

**THROW Mode**:
```
THROW_NEXTMODE       - Mode after throw complete, default: 3 (AUTO)
THROW_TYPE           - Throw type (0=upward, 1=drop), default: 0
THROW_MOT_START      - Motor start (0=stopped, 1=running), default: 0
```

**ZIGZAG Mode**:
```
ZIGZ_AUTO            - Enable auto zigzag, default: 0
ZIGZ_SPRAY           - Enable sprayer, default: 0
ZIGZ_WP_DELAY        - Waypoint delay (s), default: 0
ZIGZ_SIDE_DIST       - Sideways distance (m), default: 4
ZIGZ_DIRECTION       - Direction (0=right, 1=left), default: 0
ZIGZ_LINE_NUM        - Number of lines, default: 0
```

**FLOWHOLD Mode**:
```
FHLD_FILT_HZ         - Flow filter cutoff (Hz), default: 5
FHLD_FLOW_MAX        - Maximum flow rate, default: 0.6
FHLD_QUAL_MIN        - Minimum flow quality, default: 10
FHLD_BRAKE_RATE      - Brake rate (deg/s), default: 8
```

**FOLLOW Mode**:
```
FOLL_ENABLE          - Enable follow mode, default: 0
FOLL_SYSID           - Target system ID, default: 0
FOLL_DIST_MAX        - Maximum distance (m), default: 100
FOLL_OFS_X           - Offset X (m), default: -8
FOLL_OFS_Y           - Offset Y (m), default: 0
FOLL_OFS_Z           - Offset Z (m), default: 0
FOLL_YAW_BEHAVE      - Yaw behavior, default: 0
FOLL_POS_P           - Position P gain, default: 0.1
FOLL_ALT_TYPE        - Altitude type, default: 0
```

**SYSTEMID Mode**:
```
SID_AXIS             - Test axis selection, default: 0
SID_MAGNITUDE        - Chirp magnitude, default: 0
SID_F_START_HZ       - Start frequency (Hz), default: 0.5
SID_F_STOP_HZ        - Stop frequency (Hz), default: 40
SID_T_FADE_IN        - Fade in time (s), default: 2
SID_T_REC            - Recording time (s), default: 20
SID_T_FADE_OUT       - Fade out time (s), default: 2
```

### Attitude Control Parameters (All Modes)

**Rate Controllers**:
```
ATC_RAT_RLL_P        - Roll rate P gain, default: 0.135
ATC_RAT_RLL_I        - Roll rate I gain, default: 0.135
ATC_RAT_RLL_D        - Roll rate D gain, default: 0.0036
ATC_RAT_RLL_FLTD     - Roll rate filter, default: 20
ATC_RAT_RLL_FLTT     - Roll rate target filter, default: 20
ATC_RAT_PIT_P        - Pitch rate P gain, default: 0.135
ATC_RAT_PIT_I        - Pitch rate I gain, default: 0.135
ATC_RAT_PIT_D        - Pitch rate D gain, default: 0.0036
ATC_RAT_PIT_FLTD     - Pitch rate filter, default: 20
ATC_RAT_PIT_FLTT     - Pitch rate target filter, default: 20
ATC_RAT_YAW_P        - Yaw rate P gain, default: 0.18
ATC_RAT_YAW_I        - Yaw rate I gain, default: 0.018
ATC_RAT_YAW_D        - Yaw rate D gain, default: 0
ATC_RAT_YAW_FLTD     - Yaw rate filter, default: 5
ATC_RAT_YAW_FLTT     - Yaw rate target filter, default: 5
```

**Angle Controllers**:
```
ATC_ANG_RLL_P        - Roll angle P gain, default: 4.5
ATC_ANG_PIT_P        - Pitch angle P gain, default: 4.5
ATC_ANG_YAW_P        - Yaw angle P gain, default: 4.5
```

**Input Shaping**:
```
ATC_INPUT_TC         - Input time constant, default: 0.15
ATC_ACCEL_R_MAX      - Max roll accel (deg/s/s), default: 110000
ATC_ACCEL_P_MAX      - Max pitch accel (deg/s/s), default: 110000
ATC_ACCEL_Y_MAX      - Max yaw accel (deg/s/s), default: 27000
ATC_RATE_FF_ENAB     - Rate feedforward enable, default: 1
ATC_SLEW_YAW         - Yaw target slew rate, default: 6000
```

### Pilot Input Parameters

**RC Input Processing**:
```
PILOT_THR_FILT       - Throttle filter (Hz), default: 0
PILOT_TKOFF_ALT      - Takeoff altitude (cm), default: 0
PILOT_THR_BHV        - Throttle behavior bitmask, default: 0
PILOT_Y_RATE         - Yaw rate (deg/s), default: 90
PILOT_Y_EXPO         - Yaw expo, default: 0
PILOT_Y_RATE_TC      - Yaw rate time constant, default: 0.5
```

**Angle and Rate Limits**:
```
ANGLE_MAX            - Maximum lean angle (cdeg), default: 4500
ACRO_RP_RATE         - Maximum roll/pitch rate in ACRO (deg/s), default: 180
ACRO_Y_RATE          - Maximum yaw rate in ACRO (deg/s), default: 90
```

### Cross-Parameter Dependencies

**Altitude Control Chain**:
```
PILOT_SPEED_UP → PSC_ACCZ_P → ATC_RAT_* → motors
```
- Pilot climb rate limited by PILOT_SPEED_UP
- Position controller generates desired acceleration
- Acceleration controller via PSC_ACCZ gains
- Attitude controllers via ATC_RAT gains to motors

**Position Control Chain**:
```
LOIT_SPEED → PSC_POSXY_P → PSC_VELXY_* → ATC_ANG_* → ATC_RAT_* → motors
```
- Horizontal speed limited by LOIT_SPEED
- Position error generates velocity via PSC_POSXY_P
- Velocity error generates lean angle via PSC_VELXY gains
- Lean angle tracked by angle controllers (ATC_ANG)
- Rate controllers (ATC_RAT) drive motors

**Tuning Interdependencies**:
- Increasing rate P requires increasing rate D for damping
- Angle P must be compatible with rate loop bandwidth
- Position P affects aggressiveness of position corrections
- Velocity I handles steady-state position errors
- All gains interact - tune systematically: rate → angle → velocity → position

### Parameter Validation

**Pre-Flight Parameter Checks**:
1. Verify all mode-required parameters set appropriately
2. Check parameter consistency (e.g., LOIT_SPEED vs WPNAV_SPEED)
3. Validate failsafe parameters configured
4. Ensure navigation parameters reasonable for vehicle size/performance
5. Confirm attitude controller gains stable

**Parameter File Management**:
- Save parameter file after successful tuning
- Document parameter changes with rationale
- Version control parameter files for different missions
- Test parameter changes in SITL before flight
- Compare parameters against known-good configurations

---

## Summary

This document provides comprehensive coverage of all 26 ArduCopter flight modes, including:

- **Manual modes** (STABILIZE, ACRO) for direct pilot control
- **Assisted modes** (ALT_HOLD, LOITER, POSHOLD, SPORT, DRIFT, FLOWHOLD) combining automation with manual input
- **Autonomous modes** (AUTO, GUIDED, RTL, SMART_RTL, LAND, CIRCLE, BRAKE, FOLLOW, ZIGZAG) for waypoint navigation and automated operations
- **Special purpose modes** (FLIP, AUTOTUNE, THROW, AVOID_ADSB, GUIDED_NOGPS, SYSTEMID, AUTOROTATE, TURTLE, GUIDED_CUSTOM) for specific use cases

Each mode description includes:
- Complete behavioral description
- Source code references
- Function-level implementation details
- Required and optional parameters
- GPS and sensor requirements
- Appropriate use cases
- Safety considerations and limitations

Understanding these modes, their transitions, and dependencies is essential for safe and effective ArduCopter operations across all vehicle types and mission profiles.

**Primary Source References**:
- `/ArduCopter/mode.h` - Mode definitions and interfaces (lines 71-2077)
- `/ArduCopter/mode.cpp` - Mode switching logic (lines 32-160)
- `/ArduCopter/mode_*.cpp` - Individual mode implementations
- `/ArduCopter/Parameters.cpp` - Parameter definitions and defaults

**Additional Resources**:
- ArduPilot Wiki: https://ardupilot.org/copter/
- Parameter documentation: https://ardupilot.org/copter/docs/parameters.html
- Mission Planner: https://ardupilot.org/planner/
- SITL Simulator for testing: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html

---

*This documentation is based on ArduCopter source code and represents flight mode behavior as of the documentation generation date. Always refer to the latest source code and official ArduPilot documentation for the most current information.*
