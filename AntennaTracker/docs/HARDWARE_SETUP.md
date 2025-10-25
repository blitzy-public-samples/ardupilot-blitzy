# AntennaTracker Hardware Setup Guide

## Table of Contents
- [Overview](#overview)
- [Supported Autopilot Boards](#supported-autopilot-boards)
- [Hardware Requirements](#hardware-requirements)
- [Mechanical Tracker Configurations](#mechanical-tracker-configurations)
- [Autopilot Installation](#autopilot-installation)
- [Servo Motor Setup](#servo-motor-setup)
- [Power System](#power-system)
- [GPS and Compass Installation](#gps-and-compass-installation)
- [Telemetry Radio Configuration](#telemetry-radio-configuration)
- [RC Receiver Connections](#rc-receiver-connections)
- [Safety Switch and Arming](#safety-switch-and-arming)
- [LED Indicators](#led-indicators)
- [Assembly Instructions](#assembly-instructions)
- [Pre-Flight Checklist](#pre-flight-checklist)
- [Troubleshooting](#troubleshooting)

## Overview

The AntennaTracker system is designed to automatically point a high-gain antenna at a moving vehicle (aircraft, rover, or boat) to maintain optimal radio signal strength over extended distances. This guide covers the complete hardware setup process from component selection through final assembly and testing.

**Purpose**: Enable reliable, automated tracking of vehicles equipped with MAVLink telemetry systems.

**Key Features**:
- Alt-azimuth (pan-tilt) mechanical design with two-axis servo control
- Closed-loop feedback using onboard IMU and compass sensors
- GPS-based or manually-configured home position
- Support for multiple vehicle tracking via MAVLink protocol
- Real-time bearing and elevation calculation with configurable update rates

**System Requirements**:
- Compatible autopilot board with IMU, compass, and MAVLink capability
- Two servo motors (yaw/azimuth and pitch/elevation axes)
- Power supply (5-6V regulated for servos, appropriate voltage for autopilot)
- Telemetry radio for receiving vehicle position data
- Optional GPS module for automatic home position acquisition

> **Source**: `AntennaTracker/Tracker.h`, `AntennaTracker/tracking.cpp`, `AntennaTracker/system.cpp`

## Supported Autopilot Boards

AntennaTracker firmware runs on ArduPilot-compatible autopilot boards. The following categories of boards are supported:

### Pixhawk Family (Recommended)

**Pixhawk 1** (FMUv2)
- Processor: STM32F427 Cortex-M4F @ 168MHz
- Memory: 256KB RAM, 2MB Flash
- Sensors: MPU6000 or ICM20602 IMU, HMC5883L compass, MS5611 barometer
- Interfaces: 8 PWM outputs, 1 I2C, 2 CAN, 5 UARTs
- Power: 4.8-5.5V input
- Use case: Standard tracking installations
- **Source**: `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk1/`

**Pixhawk 4**
- Processor: STM32F765 Cortex-M7 @ 216MHz
- Memory: 512KB RAM, 2MB Flash
- Sensors: ICM20689 + BMI055 IMUs, IST8310 compass, MS5611 barometer
- Interfaces: 16 PWM outputs, 2 I2C, 2 CAN, 4 UARTs
- Power: 4.75-5.5V input
- Use case: Advanced tracking with redundant sensors
- **Source**: `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk4/`

**Pixhawk 6X/6C**
- Processor: STM32H753 Cortex-M7 @ 480MHz
- Memory: 1MB RAM, 2MB Flash
- Sensors: Triple IMU redundancy, dual compass, dual barometer
- Interfaces: 16 PWM outputs, 4 I2C, 2 CAN FD, 8 UARTs, Ethernet
- Power: 4.75-5.5V input
- Use case: Professional tracking systems requiring maximum reliability
- **Source**: `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X/`

### Cube Family

**CubeBlack** (FMUv3)
- Processor: STM32F427 Cortex-M4F @ 168MHz
- Memory: 256KB RAM, 2MB Flash
- Sensors: Dual IMU (MPU6000 + ICM20602), HMC5983 compass, MS5611 barometer
- Interfaces: 14 PWM outputs, 2 I2C, 2 CAN, 5 UARTs
- Power: 4.1-5.7V input
- Use case: Ruggedized tracking installations
- **Source**: `libraries/AP_HAL_ChibiOS/hwdef/CubeBlack/`

**CubeOrange** / **CubeOrangePlus**
- Processor: STM32H743 Cortex-M7 @ 480MHz
- Memory: 1MB RAM, 2MB Flash (Plus: 16MB external)
- Sensors: Triple IMU redundancy, dual compass, dual barometer
- Interfaces: 14 PWM outputs, 4 I2C, 2 CAN, 5 UARTs
- Power: 4.1-5.7V input
- Use case: High-performance tracking with extended logging
- **Source**: `libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/`

### Compact Boards

**Pixracer**
- Processor: STM32F427 Cortex-M4F @ 168MHz
- Memory: 256KB RAM, 2MB Flash
- Sensors: MPU9250 9-axis IMU, HMC5983 compass, MS5611 barometer
- Interfaces: 6 PWM outputs, 2 I2C, 1 CAN, 5 UARTs
- Form factor: 36mm x 36mm
- Power: 5V input (400mA)
- Use case: Lightweight, portable tracking systems
- **Source**: `libraries/AP_HAL_ChibiOS/hwdef/Pixracer/`

### Board Selection Criteria

| Consideration | Minimum Spec | Recommended | Notes |
|---------------|--------------|-------------|-------|
| PWM Outputs | 2 (yaw + pitch) | 4+ | Additional outputs for camera gimbal, lights |
| UART Ports | 2 (GPS + telemetry) | 3+ | Extra port for secondary telemetry/debug |
| I2C Buses | 1 | 2 | External compass, rangefinder |
| Flash Memory | 1MB | 2MB+ | Logging, terrain data |
| Processing Power | 168MHz (M4F) | 216MHz+ (M7) | Complex tracking algorithms |

> **Important**: All supported boards are listed in `libraries/AP_HAL_ChibiOS/hwdef/`. Verify board compatibility with AntennaTracker firmware before purchase.

## Hardware Requirements

### Minimum Components

**Required Hardware**:
1. **Autopilot Board**: Any ArduPilot-compatible board from the supported list
2. **Servo Motors**: Two servos for yaw and pitch axes
   - Minimum torque: 5 kg·cm for small antennas (<1kg)
   - Recommended: 10+ kg·cm for larger antennas
3. **Power Supply**: 
   - 5-6V regulated power for servos (2-5A capacity depending on servo size)
   - Appropriate voltage for autopilot (typically 5V via BEC or dedicated regulator)
4. **Telemetry Radio**: 
   - 915MHz or 433MHz for long range
   - Minimum 57600 baud, recommended 115200 baud
5. **Mechanical Tracker Frame**: Alt-azimuth mount with pan-tilt capability

**Optional but Recommended**:
- **GPS Module**: u-blox M8N/M9N or better for automatic home position
- **External Compass**: HMC5883L, IST8310, or RM3100 for reduced magnetic interference
- **Safety Switch**: For safe arming/disarming (built-in on most Pixhawk boards)
- **Power Module**: For battery voltage and current monitoring
- **SD Card**: For logging (required on some boards)
- **Buzzer**: Audio feedback for status and warnings
- **RC Receiver**: For manual control override

### Servo Motor Specifications

**Servo Selection Guide**:

*Source: `AntennaTracker/servos.cpp`, `AntennaTracker/config.h`*

AntennaTracker supports three servo operating modes:

1. **Position Mode** (SERVO_TYPE_POSITION = 0, Default):
   - Standard hobby servos with 180-degree rotation
   - Most common configuration
   - Examples: Hitec HS-645MG, Futaba S3003, Tower Pro MG996R

2. **Continuous Rotation Mode** (SERVO_TYPE_CR = 1):
   - Modified servos or continuous rotation servos
   - Allows unlimited rotation for yaw axis
   - Requires rate control instead of position control

3. **On/Off Mode** (SERVO_TYPE_ONOFF = 2):
   - Simple directional control without position feedback
   - Used for basic left/right, up/down control
   - Minimal precision

**Servo Torque Requirements**:

| Antenna Weight | Minimum Yaw Torque | Minimum Pitch Torque | Example Servo |
|----------------|-------------------|---------------------|---------------|
| <500g | 5 kg·cm | 3 kg·cm | Hitec HS-422 |
| 500g-1kg | 10 kg·cm | 7 kg·cm | Hitec HS-645MG |
| 1-2kg | 15 kg·cm | 12 kg·cm | Hitec HS-755HB |
| 2-5kg | 25 kg·cm | 20 kg·cm | Hitec HS-805BB |
| >5kg | 40+ kg·cm | 30+ kg·cm | Industrial servo or stepper motor |

**Servo Speed**:
- Typical: 0.15-0.25 sec/60° 
- Faster servos (0.10 sec/60°) provide better tracking of fast-moving vehicles
- Slower servos acceptable for high-altitude aircraft tracking

**Connector Type**:
- Standard 3-wire servo connector (Signal, 5V, GND)
- Some large servos use separate power input

> **Warning**: Undersized servos will cause tracking lag, oscillation, or mechanical failure. Always include a safety margin in torque calculations.

## Mechanical Tracker Configurations

There are two primary mechanical designs for antenna trackers:

### Configuration 1: Pan-Tilt (Alt-Azimuth) Design

**Description**: The most common design with separate yaw (pan) and pitch (tilt) axes.

```
                    [Antenna]
                        |
                   [Tilt Bracket] ← Pitch Servo
                        |
                   [Pan Platform] ← Yaw Servo
                        |
                    [Base Plate]
```

**Advantages**:
- Simple mechanical construction
- Easy servo mounting and wiring
- Compact footprint
- 360-degree yaw rotation possible
- Good for portable setups

**Disadvantages**:
- Gimbal lock at high elevation angles (near zenith)
- Antenna orientation changes as it pans while tilted
- Requires careful calibration of servo center points

**Recommended For**: Most tracking applications, especially directional Yagi or patch antennas

**Commercial Examples**:
- Eagle Tree Antenna Tracker Pan-Tilt (discontinued but design available)
- DIY designs using aluminum brackets and hobby servos
- 3D-printed frames available from community designs

### Configuration 2: Elevation-over-Azimuth Design

**Description**: Elevation axis mounted perpendicular to azimuth axis with antenna offset from center.

```
           [Antenna] ←──┐
                         |
                    [Elevation Axis] ← Pitch Servo
                         |
                    [Yaw Bearing]
                         |
                    [Yaw Motor] ← Yaw Servo
                         |
                    [Base Plate]
```

**Advantages**:
- No gimbal lock issues
- Can track through zenith (straight up)
- More stable mechanical platform
- Better for heavy antennas

**Disadvantages**:
- More complex mechanical construction
- Larger footprint
- Heavier overall weight
- More expensive to build

**Recommended For**: Professional installations, large antennas, tracking high-altitude balloons or satellites

### Mounting Considerations

**Autopilot Mounting Requirements**:
> **Critical**: The autopilot board MUST be mounted on the moving part of the tracker (not the stationary base) to measure the antenna's actual pointing direction.

*Source: `AntennaTracker/tracking.cpp:update_tracker_position()`, `AntennaTracker.txt:88-95`*

**Mounting Position**:
1. **Orientation**: Mount the autopilot so that:
   - The "forward" direction points in the same direction as the antenna beam
   - The "up" direction is perpendicular to the antenna mounting surface
   - If non-standard orientation used, configure `AHRS_ORIENTATION` parameter

2. **Vibration Isolation**:
   - Use foam padding or vibration dampeners
   - Minimum 5mm thick soft foam recommended
   - Avoid hard mounting directly to servo mounts

3. **Accessibility**:
   - USB port accessible for configuration
   - SD card accessible for log retrieval (if equipped)
   - LED indicators visible
   - Safety switch accessible (if equipped)

**Common AHRS_ORIENTATION Values**:
| AHRS_ORIENTATION | Description | Use Case |
|------------------|-------------|----------|
| 0 | None | Autopilot forward = antenna forward, up = up |
| 1 | Yaw 45° | Autopilot rotated 45° clockwise |
| 2 | Yaw 90° | Autopilot rotated 90° clockwise |
| 4 | Yaw 180° | Autopilot facing backward |
| 8 | Yaw 270° | Autopilot rotated 90° counter-clockwise |
| 24 | Roll 180° | Autopilot mounted upside down |

> **Source**: Common autopilot parameter definitions, see `libraries/AP_AHRS/`

### Bill of Materials Example

**Basic Pan-Tilt Tracker BOM** (estimated costs in USD, 2024):

| Item | Quantity | Example Part | Est. Cost |
|------|----------|--------------|-----------|
| Autopilot Board | 1 | Pixhawk 1 or Cube Black | $50-200 |
| Yaw Servo | 1 | Hitec HS-645MG (10kg·cm) | $25-35 |
| Pitch Servo | 1 | Hitec HS-645MG (10kg·cm) | $25-35 |
| GPS Module | 1 | u-blox M8N with compass | $25-40 |
| Telemetry Radio | 1 | 915MHz 100mW or 1W | $25-50 |
| Power Supply | 1 | 5V 3A BEC | $10-15 |
| Mechanical Frame | 1 | Aluminum brackets + hardware | $30-50 |
| Servo Extension Cables | 2 | 30cm extensions | $5-10 |
| Connectors | Various | DF13, JST-GH connectors | $10-20 |
| **Total** | | | **$205-455** |

## Autopilot Installation

### Physical Mounting

**Step 1: Prepare Mounting Surface**
1. Select mounting location on the moving (rotating/tilting) part of tracker
2. Ensure surface is flat, clean, and free from oil or debris
3. Verify clearance for cables and connectors (minimum 50mm clearance recommended)

**Step 2: Vibration Dampening**
1. Cut vibration dampening foam to autopilot dimensions
2. Use 3M dual-lock or similar mounting system
3. Thickness: 5-10mm soft foam or gel pads
4. Ensure foam doesn't compress more than 50% under autopilot weight

**Step 3: Orientation Alignment**
1. Position autopilot so forward arrow points toward antenna direction
2. If different orientation required, note the rotation angle for parameter configuration
3. Secure autopilot firmly but avoid over-tightening screws (can crack PCB)

**Step 4: Cable Management**
1. Route cables to minimize interference with servo movement
2. Use cable ties or spiral wrap for organization
3. Ensure adequate slack for full range of motion
4. Protect cables from sharp edges and pinch points

### Connector Pinouts

**Pixhawk 1 / Cube Connections**:

**MAIN OUT Ports** (PWM Servo Outputs):
- Pin 1: Yaw Servo (default, configurable via RC1 parameters)
- Pin 2: Pitch Servo (default, configurable via RC2 parameters)
- Pins 3-8: Available for auxiliary functions

**GPS Port** (6-pin DF13):
```
Pin 1: VCC (5V)
Pin 2: TX (GPS → Autopilot)
Pin 3: RX (Autopilot → GPS)
Pin 4: I2C SCL (for compass)
Pin 5: I2C SDA (for compass)
Pin 6: GND
```

**TELEM1 Port** (Telemetry Radio):
```
Pin 1: VCC (5V, 2A max)
Pin 2: TX (Autopilot → Radio)
Pin 3: RX (Radio → Autopilot)
Pin 4: CTS (flow control, optional)
Pin 5: RTS (flow control, optional)
Pin 6: GND
```

**POWER Port**:
```
Pin 1: VCC (5V input)
Pin 2: VCC
Pin 3: CURRENT (analog current sensor input)
Pin 4: VOLTAGE (analog voltage sensor input)
Pin 5: GND
Pin 6: GND
```

### Initial Power-Up Procedure

**Pre-Power Checklist**:
1. [ ] Verify all connections are secure
2. [ ] Check servo power is correctly wired (separate from autopilot if using large servos)
3. [ ] Ensure SD card is inserted (if board requires it)
4. [ ] Verify no short circuits with multimeter
5. [ ] Confirm power supply voltage is within spec (4.75-5.5V typical)

**Power-Up Sequence**:
1. Connect USB cable to computer (powers autopilot with low current)
2. Observe LED patterns:
   - **Rapid flashing**: Autopilot booting
   - **Slow breathing**: Initialization in progress
   - **Steady or slow flash**: Initialization complete
3. If using ground station (Mission Planner, QGroundControl):
   - Select COM port and 115200 baud
   - Click Connect
   - Verify firmware type shows "AntennaTracker"
4. Connect main power supply (if servos will be used)
5. Observe servo initialization (should move to trim position if configured)

> **Warning**: First power-up should be done with servos disconnected or mechanical assembly loose to prevent damage from unexpected servo movements.

## Servo Motor Setup

### Wiring Diagrams

**Standard Servo Connection**:
```
Servo Connector (looking at wire side):
┌─────────────────┐
│  Signal (White/Yellow)  │───→ To Autopilot MAIN OUT pin
│  +5V (Red)             │───→ To Power Distribution (5-6V)
│  GND (Black/Brown)     │───→ To Common Ground
└─────────────────┘
```

**Power Distribution for Servos**:

*For Standard Servos (< 2A each)*:
```
[Autopilot MAIN OUT Power Rail]
         │
         ├── Yaw Servo Power
         └── Pitch Servo Power

Configuration: Install jumper/power module on autopilot MAIN OUT rail
```

*For High-Current Servos (> 2A each)* - **Recommended**:
```
[External 5V BEC/Regulator (3-10A)]
         │
    [Power Distribution Block]
         ├── Yaw Servo Power (Red wire)
         ├── Pitch Servo Power (Red wire)
         └── Common Ground ──→ Connected to Autopilot Ground

Signal Wires: Connect normally to autopilot MAIN OUT pins
Power Rail: Remove power jumper from autopilot to avoid backfeeding
```

> **Critical**: Large servos can draw 3-5A during movement, which exceeds most autopilot power rail limits (typically 2A). Always use external power for servos larger than micro/mini size.

*Source: `AntennaTracker/servos.cpp:init_servos()`, `AntennaTracker/config.h:CH_YAW, CH_PITCH`*

### Servo Configuration Parameters

**RC1 Parameters (Yaw Servo)**:

| Parameter | Description | Typical Value | Notes |
|-----------|-------------|---------------|-------|
| RC1_MIN | PWM at minimum position | 1000-1100 | Depends on servo model |
| RC1_MAX | PWM at maximum position | 1900-2000 | Depends on servo model |
| RC1_TRIM | PWM at center position | 1500 | Servo should be at 0° yaw |
| RC1_REVERSED | Reverse servo direction | 0 or 1 | Set to 1 if servo moves opposite to expected |
| RC1_DZ | Deadzone (unused for tracker) | 0 | Not applicable |

**RC2 Parameters (Pitch Servo)**:

| Parameter | Description | Typical Value | Notes |
|-----------|-------------|---------------|-------|
| RC2_MIN | PWM at minimum elevation | 1000-1100 | Servo at minimum pitch angle |
| RC2_MAX | PWM at maximum elevation | 1900-2000 | Servo at maximum pitch angle |
| RC2_TRIM | PWM at horizon (0° elevation) | 1500 | Critical for accurate tracking |
| RC2_REVERSED | Reverse servo direction | 0 or 1 | Set to 1 if elevation inverted |

**AntennaTracker-Specific Parameters**:

| Parameter | Description | Default | Range | Notes |
|-----------|-------------|---------|-------|-------|
| SERVO_YAW_TYPE | Yaw servo operating mode | 0 | 0-2 | 0=Position, 1=CR, 2=OnOff |
| SERVO_PITCH_TYPE | Pitch servo operating mode | 0 | 0-2 | 0=Position, 1=CR, 2=OnOff |
| YAW_RANGE | Total yaw range in degrees | 360 | 0-360 | Physical rotation limit |
| PITCH_MIN | Minimum pitch angle | -90 | -90 to 0 | Negative = below horizon |
| PITCH_MAX | Maximum pitch angle | 90 | 0 to 90 | 90 = straight up |
| YAW_TRIM | Yaw trim offset | 0 | -180 to 180 | Mechanical alignment offset |
| PITCH_TRIM | Pitch trim offset | 0 | -90 to 90 | Mechanical alignment offset |

*Source: `AntennaTracker/Parameters.h`, `AntennaTracker/servos.cpp`*

### Servo Calibration Procedure

**Step 1: Determine Servo PWM Range**
1. Connect servo to RC receiver or servo tester
2. Measure PWM values at physical endpoints
3. Note minimum PWM before servo stalls/buzzes
4. Note maximum PWM before servo stalls/buzzes
5. Calculate center: (MIN + MAX) / 2

**Step 2: Set Parameter Values**
1. Set RC1_MIN and RC2_MIN to measured minimum values
2. Set RC1_MAX and RC2_MAX to measured maximum values
3. Set RC1_TRIM and RC2_TRIM to calculated center values
4. Write parameters to autopilot

**Step 3: Verify Servo Direction**
1. Manually command yaw increase (via MAVLink or RC override)
2. Servo should rotate clockwise when viewed from above
3. If incorrect, set RC1_REVERSED = 1
4. Manually command pitch increase
5. Servo should tilt antenna upward
6. If incorrect, set RC2_REVERSED = 1

**Step 4: Fine-Tune Trim Values**
1. Enable tracking mode (AUTO or SCAN)
2. Manually position antenna pointing at horizon (0° elevation)
3. Observe pitch servo PWM output on ground station
4. Adjust PITCH_TRIM so servo output is centered when antenna is level
5. Point antenna due north (0° yaw)
6. Adjust YAW_TRIM so servo output matches expected value

### PID Tuning for Smooth Tracking

AntennaTracker uses PID controllers to achieve smooth, accurate pointing:

**YAW2SRV PID Parameters**:

| Parameter | Description | Default | Tuning Notes |
|-----------|-------------|---------|--------------|
| YAW2SRV_P | Proportional gain | 0.20 | Increase for faster response, decrease if oscillating |
| YAW2SRV_I | Integral gain | 0.00 | Usually not needed for tracking |
| YAW2SRV_D | Derivative gain | 0.05 | Damping, increase to reduce overshoot |
| YAW2SRV_IMAX | Integrator limit | 4000 | Maximum integrator contribution (centi-degrees) |
| YAW2SRV_FF | Feed-forward gain | 0.02 | Anticipatory control for smooth tracking |

**PITCH2SRV PID Parameters**:

| Parameter | Description | Default | Tuning Notes |
|-----------|-------------|---------|--------------|
| PITCH2SRV_P | Proportional gain | 0.20 | Increase for faster response |
| PITCH2SRV_I | Integral gain | 0.00 | Compensates for gravity offset if needed |
| PITCH2SRV_D | Derivative gain | 0.05 | Reduces bounce and overshoot |
| PITCH2SRV_IMAX | Integrator limit | 4000 | Maximum integrator contribution |
| PITCH2SRV_FF | Feed-forward gain | 0.02 | Improves tracking smoothness |

*Source: `AntennaTracker/servos.cpp:update_pitch_servo()`, `AntennaTracker/Parameters.cpp`*

**PID Tuning Procedure**:
1. Start with default values
2. Set D and I terms to 0, tune P first
3. Increase P until servo responds quickly but starts to oscillate
4. Reduce P by 25-30%
5. Increase D gradually until oscillation is damped
6. Add I term only if steady-state error exists (antenna doesn't quite reach target)
7. Test tracking with real vehicle to verify smooth performance

**Tuning Tips**:
- Small, light antennas: Can use higher P gain (0.3-0.5)
- Large, heavy antennas: Use lower P gain (0.1-0.2) and higher D gain (0.08-0.12)
- If antenna "hunts" back and forth: Reduce P, increase D
- If antenna lags behind target: Increase P or add FF term
- If antenna drifts off target when stopped: Add small I term (0.01-0.02)

## Power System

### Power Requirements

**Autopilot Power**:
- Voltage: 4.75-5.5V (most boards), check specific board specs
- Current: 100-300mA typical, up to 500mA with peripherals
- Clean, regulated power required for stable operation

**Servo Power**:
- Voltage: 4.8-6.0V (check servo specifications)
- Current: Highly variable based on load
  - Idle: 10-50mA per servo
  - Moving (no load): 100-400mA per servo
  - Moving (under load): 1-5A per servo
  - Stall (max): 2-8A per servo (avoid!)

**GPS Module**:
- Voltage: 3.3-5V (usually powered by autopilot GPS port)
- Current: 30-100mA

**Telemetry Radio**:
- Voltage: 5V (usually powered by autopilot TELEM port)
- Current: 100mW: ~50-100mA, 1W: ~200-400mA

### Power Distribution Architectures

**Architecture 1: USB-Powered Development Setup**
```
[USB Power from Computer] → [Autopilot] → [Telemetry Radio]
                                         → [GPS/Compass]
                                         
Servos: Disabled or powered separately
Use Case: Initial setup, parameter configuration, compass calibration
```

**Architecture 2: Single Battery with BEC**
```
[2S-3S LiPo Battery (7.4-11.1V)]
         │
    [5V BEC (5A+)]
         ├──→ [Autopilot] → [Telemetry Radio]
         │                 → [GPS/Compass]
         ├──→ [Yaw Servo]
         └──→ [Pitch Servo]

Use Case: Portable tracker, field operations
```

**Architecture 3: Dual Power Rails (Recommended)**
```
[2S-3S LiPo or 12V Supply]
         │
         ├──→ [5V BEC #1 (1A)] → [Autopilot] → [Telemetry]
         │                                    → [GPS]
         │
         └──→ [5V BEC #2 (5A+)] → [Servo Power Distribution]
                                          ├──→ [Yaw Servo]
                                          └──→ [Pitch Servo]

Common Ground: Connect all ground wires together
Signal Isolation: Servos receive signals from autopilot PWM outputs

Use Case: Reliable operation with large servos
```

**Architecture 4: AC Powered Station**
```
[110/220V AC] → [12V DC Power Supply (5-10A)]
                         │
                         ├──→ [5V Step-Down (Autopilot)]
                         ├──→ [5V Step-Down (Servos)]
                         └──→ [Optional 12V Devices]

Use Case: Permanent ground station installation
```

### Recommended Power Components

**BEC/Voltage Regulators**:
- Castle Creations BEC (5A-20A): Reliable, widely available
- Hobbywing Switch-Mode BEC: High efficiency, multiple current ratings
- Pololu Step-Down Regulators: Compact, good for lower-current applications

**Batteries** (for portable operation):
- 2S LiPo (7.4V nominal): 2200-5000mAh capacity
- 3S LiPo (11.1V nominal): More overhead for voltage regulation
- Lead-acid: Heavier but cheaper for stationary installations

**Wiring**:
- Power wires: 18-22 AWG for servo power (lower AWG = thicker for high current)
- Signal wires: 24-26 AWG acceptable
- Servo extensions: Use quality extensions with good connectors

### Power System Validation

**Pre-Operation Checks**:
1. Measure voltage at autopilot power input: Should be 4.75-5.5V under load
2. Measure voltage at servo power rail: Should be 4.8-6.0V under load
3. Verify voltage doesn't drop >0.3V when servos move
4. Check for excessive heat in regulators after 5 minutes of operation
5. Verify no voltage on signal wires when servos are unplugged

**Troubleshooting Power Issues**:
- **Autopilot reboots during servo movement**: Voltage sag, use separate BECs
- **Servos jitter or buzz**: Insufficient current capacity, noisy power supply
- **Intermittent tracking loss**: Check all power connector crimps and solder joints
- **GPS loses fix when servos move**: Electrical noise coupling, add filtering capacitors

## GPS and Compass Installation

### GPS Module Selection

**Recommended GPS Modules**:
- **u-blox M8N**: Standard choice, good accuracy, built-in compass optional
- **u-blox M9N**: Improved acquisition time and multi-band support
- **u-blox F9P**: RTK-capable for centimeter-level accuracy (overkill for most trackers)

**GPS Performance Requirements for AntennaTracker**:
- Fix Type: 3D fix required for home position setting
- Update Rate: 5Hz minimum, 10Hz preferred
- Accuracy: 2.5m CEP typical (M8N), better with M9N/F9P
- Cold start: <30 seconds in clear sky

*Source: `AntennaTracker/sensors.cpp:update_GPS()`*

### GPS Mounting Considerations

**Location**:
1. Mount on the moving part of tracker (typically on same platform as autopilot)
2. Clear view of sky: Minimum 140° field of view, 180° preferred
3. Away from metal objects: Minimum 10cm separation from large metal surfaces
4. Above RF transmitters: Keep at least 30cm from telemetry radio antenna

**Orientation**:
- GPS module orientation is not critical (no "forward" direction needed)
- Arrow or pin 1 marking is for reference only

**Cable Routing**:
- Use shielded cable if run >30cm from autopilot
- Avoid routing GPS cable parallel to motor/servo wires
- Keep away from high-current power wires

### Compass Configuration

The compass is CRITICAL for AntennaTracker operation as it provides absolute yaw/bearing reference.

*Source: `AntennaTracker/system.cpp:init_ardupilot()`, `AntennaTracker.txt:140-168`*

**Compass Types**:
1. **Internal Compass**: Built into autopilot board
   - Convenient but subject to electromagnetic interference
   - Acceptable for small trackers with clean power
2. **External Compass**: Separate module, often combined with GPS
   - Recommended for reliable operation
   - Must be mounted away from sources of magnetic interference

**Sources of Magnetic Interference**:
- Servo motors: 5-15cm interference radius
- Power wires carrying >1A: 3-10cm interference radius
- Steel/iron mechanical components: Variable, test with compass app
- Telemetry radios during transmit: 5-10cm radius
- Speakers, buzzers: 5-15cm radius

**Compass Mounting Best Practices**:
1. Mount external compass on GPS mast, elevated above tracker
2. Minimum 15cm separation from servos
3. Mount on moving part of tracker (not stationary base)
4. Secure mounting - any movement will cause heading errors
5. Cable strain relief to prevent connector stress

**Compass Orientation Configuration**:
- Parameter: `COMPASS_ORIENT` (for primary compass)
- Most GPS/compass modules: ROTATION_NONE (0) when arrow points forward
- If compass orientation doesn't match GPS forward arrow, set appropriate rotation value

### GPS and Compass Calibration

**GPS Ground Start Procedure**:

*Source: `AntennaTracker/sensors.cpp:update_GPS()` lines 27-49*

1. Power up tracker in outdoor location with clear sky view
2. Wait for GPS to achieve 3D fix (typically 30-60 seconds)
3. Tracker requires 5 consecutive good 3D fixes before setting home position
4. Once home is set, current location is saved to EEPROM
5. If START_LATITUDE and START_LONGITUDE parameters are set, they provide initial location until GPS fix is obtained

**Compass Calibration Procedure** (CRITICAL):

*Source: `AntennaTracker.txt:140-168`*

> **Warning**: Compass MUST be calibrated in-situ (installed in tracker) outdoors with all equipment powered on.

**Using Mission Planner**:
1. Assemble complete tracker with all equipment installed
2. Power up tracker
3. Connect to Mission Planner via USB or telemetry
4. Navigate to INITIAL SETUP → Mandatory Hardware → Compass
5. Click "Live Calibration" button
6. Click OK when prompted
7. **Physically rotate the entire tracker assembly** through all possible orientations:
   - Yaw: Rotate base through full 360°
   - Pitch: Tilt antenna from straight down to straight up
   - Roll: If possible, tilt entire assembly side-to-side
8. Continue rotating for full 60 seconds or until progress bar completes
9. Mission Planner will automatically upload new calibration offsets
10. Verify success message appears

**Using MAVProxy**:
```
compass
module load  calibration
calibration.startmag
<rotate tracker through all orientations>
<wait for completion message>
calibration.stopmag
```

**Calibration Validation**:
1. After calibration, arm tracker and enable AUTO mode
2. Position a laptop or phone with compass app nearby
3. Command tracker to point at different bearings (N, E, S, W)
4. Compare tracker antenna direction to external compass
5. Discrepancy should be <5° for good calibration
6. Re-calibrate if error exceeds 10°

**Compass Health Monitoring**:
- Parameter: `COMPASS_USE` - Enable/disable compass (1 = enabled, 0 = disabled)
- Parameter: `COMPASS_AUTODEC` - Automatic declination (1 = enabled, usually leave enabled)
- During operation, monitor compass health via ground station:
  - Mission Planner: HUD shows heading
  - MAVProxy: `watch ATTITUDE` command shows yaw

## Telemetry Radio Configuration

### Telemetry Radio Overview

Telemetry radios provide the bidirectional MAVLink communication link for:
- Receiving vehicle position updates (GLOBAL_POSITION_INT messages)
- Sending tracker position and status to ground station
- Receiving tracking commands from ground station
- Parameter configuration and firmware updates

### Radio Selection

**Frequency Bands**:
- **915MHz**: Legal in USA, Australia (ISM band), 40-60km range typical with 1W
- **433MHz**: Legal in Europe, Asia, 80-100km range typical with 1W (better penetration)
- **2.4GHz**: WiFi-based systems (ESP32, ESP8266), shorter range but higher bandwidth

**Power Levels**:
- **100mW**: Suitable for short range (<2km), low power consumption
- **500mW**: Medium range (5-10km), good balance
- **1W**: Long range (10+ km), requires good antenna and power supply

**Recommended Radios**:
- RFD900x/RFD900+: Professional grade, 1W, 57600-230400 baud
- 3DR Radio / SiK Radio: Common, 100mW, 57600 baud
- HolyBro Telemetry Radio: Reliable, multiple power levels available
- CUAV PW-Link: Long-range option for professional applications

### Telemetry Wiring

**Connection to Autopilot**:
```
Autopilot TELEM1 or TELEM2 Port:
┌────────────────────────────┐
│ Pin 1: VCC (5V out)        │───→ Radio VCC (5V in)
│ Pin 2: TX (output)         │───→ Radio RX (input)
│ Pin 3: RX (input)          │───→ Radio TX (output)
│ Pin 4: CTS (optional)      │───→ Radio CTS (flow control)
│ Pin 5: RTS (optional)      │───→ Radio RTS (flow control)
│ Pin 6: GND                 │───→ Radio GND
└────────────────────────────┘
```

> **Note**: Most radios don't require flow control (CTS/RTS) for tracking applications. Can be left unconnected.

*Source: `AntennaTracker/system.cpp:gcs().setup_uarts()`*

### Telemetry Radio Configuration

**Autopilot Parameters**:
| Parameter | Description | Recommended Value |
|-----------|-------------|-------------------|
| SERIAL1_PROTOCOL | Protocol for TELEM1 | 2 (MAVLink2) |
| SERIAL1_BAUD | Baud rate | 57 (57600) or 115 (115200) |
| SERIAL2_PROTOCOL | Protocol for TELEM2 | 2 (MAVLink2) |
| SERIAL2_BAUD | Baud rate | 57 or 115 |
| SYSID_MYGCS | GCS system ID | 255 (default) |
| SYSID_THISMAV | Tracker system ID | 2 (default for tracker) |

**Radio Configuration** (for SiK/RFD900 radios):
Connect radio to computer via USB and use appropriate configuration tool:
- **SiK Radio**: Use Mission Planner → Optional Hardware → SiK Radio
- **RFD900**: Use RFD Tools software

**Key Radio Parameters**:
| Parameter | Tracker Radio | Vehicle Radio | Notes |
|-----------|---------------|---------------|-------|
| AirSpeed | 64 (64kbps) | 64 (64kbps) | Must match on both ends |
| NetID | 25 | 25 | Must match on both ends, change to avoid interference |
| TxPower | 20 (100mW) or 30 (1W) | 20 or 30 | Regulatory compliance required |
| ECC | Enabled | Enabled | Error correction, recommended |
| MAVlink | Enabled | Enabled | MAVLink framing |
| OpResend | Enabled | Enabled | Retry failed packets |

### Antenna Placement

**Telemetry Antenna Considerations**:
1. **Mounting Location**: 
   - Mount on stationary base (not moving tracker platform) for omnidirectional coverage
   - Vertical orientation for omnidirectional radiation pattern
   - Height above ground: 1-2m minimum for ground-based operations

2. **Polarization**:
   - Vertical polarization standard for vehicle radios
   - Match tracker antenna polarization to vehicle

3. **Separation from Tracker Antenna**:
   - Minimum 30cm separation from high-gain tracking antenna
   - Avoid shadowing by metal components

4. **Connector Quality**:
   - Use quality SMA or RP-SMA connectors
   - Check for loose connections regularly
   - Replace damaged cables immediately

### Telemetry Link Validation

**Range Testing Procedure**:
1. Connect both radios, power up tracker and vehicle
2. Monitor RSSI (Received Signal Strength Indicator) on ground station
3. Gradually increase distance while monitoring link quality
4. Acceptable RSSI: >50% at maximum expected range
5. If RSSI drops below 25%, tracking will become intermittent

**Troubleshooting Telemetry Issues**:
- **No telemetry connection**: Check baud rates match, verify TX→RX and RX→TX cross-connection
- **Intermittent dropout**: Check antenna connections, reduce transmit rate, enable ECC
- **Range shorter than expected**: Verify antenna is proper frequency, check for interference
- **MAVLink parse errors**: Verify both radios configured for MAVLink mode, check for version mismatch

## RC Receiver Connections

While AntennaTracker primarily operates autonomously via MAVLink, an RC receiver can provide manual control override and mode switching.

### RC Receiver Setup (Optional)

**Supported Receiver Types**:
- PPM (Pulse Position Modulation): Single-wire connection, 8+ channels
- SBUS (Futaba): Single-wire serial protocol, 16 channels, inverted signal
- Spektrum DSM/DSM2/DSMX: Satellite receivers
- CRSF (Crossfire): Long-range control

**Connection to Autopilot**:
- **Pixhawk 1/Cube**: Connect to RCIN port (PPM/SBUS)
- **Pixhawk 4/6X**: Connect to RC IN port
- Refer to specific board documentation for correct port

**Minimal RC Channel Assignment**:
| RC Channel | Function | Use |
|------------|----------|-----|
| Channel 5 | Flight Mode | Switch between MANUAL, AUTO, SCAN modes |
| Channel 7 | Arm/Disarm | Safety control |

**RC Parameters**:
```
RC5_OPTION = 9  (Flight Mode)
RC7_OPTION = 153 (Arm/Disarm)
FLTMODE_CH = 5
```

**RC Calibration**:
1. Connect to Mission Planner
2. Navigate to INITIAL SETUP → Mandatory Hardware → Radio Calibration
3. Move all sticks and switches through full range
4. Click "Calibrate Radio" and follow prompts
5. Verify all channel PWM values range from ~1000 to ~2000

### Manual Control Mode

When RC receiver is connected, pilot can manually control tracker:
- **MANUAL Mode**: Direct RC control of yaw and pitch servos
  - Channel 1: Yaw control
  - Channel 2: Pitch control
- **AUTO Mode**: Automated tracking via MAVLink
- **SCAN Mode**: Automated scanning pattern

*Source: `AntennaTracker/mode_manual.cpp`, `AntennaTracker/RC_Channel_Tracker.cpp`*

## Safety Switch and Arming

### Safety Switch Overview

Many Pixhawk-family boards include a safety switch button that must be pressed to enable servo outputs.

**Safety Switch Behavior**:
- **Initial state**: Safety engaged (servos disabled)
- **Press button**: Safety disengaged (servos enabled)
- **LED indicator**: 
  - Solid red: Safety engaged
  - Blinking/solid green: Safety disengaged

### Arming System

Unlike multi-rotor or fixed-wing vehicles, AntennaTracker has a simplified arming system:

*Source: `AntennaTracker/system.cpp:arm_servos()`, `AntennaTracker/AP_Arming_Tracker.h`*

**Arming States**:
1. **Disarmed**: 
   - Servos disabled or set to trim/zero position based on DISARM_PWM parameter
   - Tracking algorithms inactive
   - Safe for transport and maintenance

2. **Armed**:
   - Servos enabled and responding to tracking commands
   - Tracking algorithms active
   - Antenna will move to track target

**Arming Methods**:
1. **Automatic**: Tracker arms automatically when entering AUTO or GUIDED mode
2. **Manual**: Via RC switch (if configured) or GCS command
3. **MAVLink**: ARM/DISARM command from ground station

**Configuration Parameters**:
| Parameter | Description | Values |
|-----------|-------------|--------|
| DISARM_PWM | Servo behavior when disarmed | 0 = Zero PWM<br>1 = Trim PWM |
| BRD_SAFETY_MASK | Channels exempt from safety switch | Bitmask (0 = all channels) |
| BRD_SAFETYOPTION | Safety switch options | See AP_BoardConfig documentation |

**Disarmed Servo Behavior**:
- **DISARM_PWM = 0** (Zero): Servos receive no signal, may freely rotate (default for mechanical safety)
- **DISARM_PWM = 1** (Trim): Servos hold trim position, maintains antenna position

## LED Indicators

Understanding LED patterns helps diagnose tracker status and issues.

### Pixhawk LED Patterns

**Main Status LED** (large LED):

| Pattern | Meaning | Action Required |
|---------|---------|-----------------|
| Rapid flashing | Bootloader mode | Normal during power-up, should transition quickly |
| Fast flashing | Initializing | Wait for sensors to initialize |
| Slow breathing | Ready to arm | Normal operation mode |
| Fast blinking | GPS acquiring | Wait for GPS fix (if GPS enabled) |
| Solid red | Error condition | Check ground station for error messages |

**GPS LED** (on GPS module):

| Pattern | Meaning | Notes |
|---------|---------|-------|
| No light | No power or not detected | Check cable connection |
| Slow flashing (1Hz) | Searching for satellites | Normal, wait 30-60 seconds |
| Fast flashing (5Hz) | 2D fix acquired | Insufficient for home position |
| Solid | 3D fix acquired | Ready for home position setting |

### Custom LED Indicators (via SRV Channels)

Additional LED feedback can be configured on spare PWM channels:

**Example Configuration**:
```
SERVO3_FUNCTION = 120  (NeoPixel LED)
SERVO4_FUNCTION = 56   (RCIN1 pass-through for manual LED control)
```

## Assembly Instructions

### Pre-Assembly Preparation

**Tools Required**:
- Screwdriver set (Phillips and flat-head)
- Allen key set (metric)
- Wire strippers and crimpers
- Soldering iron (for custom wiring)
- Multimeter (voltage and continuity testing)
- Cable ties and heat shrink tubing

**Workspace Setup**:
1. Clean, static-free work surface
2. Good lighting
3. Anti-static mat recommended for sensitive electronics
4. Parts organizer for small screws and components

### Step-by-Step Assembly

**Step 1: Mechanical Frame Assembly** (1-2 hours)

1. Assemble base platform according to frame manufacturer instructions
2. Install yaw servo in base mounting bracket
   - Ensure servo output shaft is centered (1500μs PWM)
   - Secure servo with provided mounting screws
   - Verify servo can rotate freely without binding
3. Attach yaw rotation platform to servo arm
   - Use servo arm with maximum spline count for precision
   - Secure with servo arm screw (use thread locker)
4. Install pitch servo on yaw platform
   - Position for balance (antenna weight centered over pitch axis)
   - Secure firmly to prevent flexing under load
5. Mount antenna bracket to pitch servo arm
   - Align antenna direction with pitch servo neutral position
   - Ensure clearance for full pitch range (-90° to +90°)

**Step 2: Autopilot Installation** (30 minutes)

1. Prepare vibration dampening on yaw platform mounting location
2. Position autopilot with forward direction matching antenna pointing direction
3. Mark AHRS_ORIENTATION if non-standard mounting used
4. Secure autopilot with dampers or mounting screws
5. Verify autopilot doesn't contact any moving parts throughout full ROM

**Step 3: GPS/Compass Installation** (20 minutes)

1. Mount GPS/compass module on mast or elevated position
   - Minimum 15cm above servos and power wires
   - Secure to moving platform (not stationary base)
2. Orient with arrow pointing same direction as antenna (if marked)
3. Route GPS cable away from servo wires and power distribution
4. Connect to autopilot GPS port
5. Verify module LED lights up when powered

**Step 4: Power System Wiring** (1 hour)

1. Install power distribution board or BEC mounting
2. Connect battery input to BEC/regulator
3. Wire BEC output to:
   - Autopilot power input (5V regulated)
   - Servo power distribution
4. Wire servo power to each servo red wire
5. Connect all grounds together (autopilot, servos, BEC)
6. Install power switch in battery input line
7. Use heat shrink on all solder connections
8. Cable tie all power wires for strain relief

**Step 5: Servo Signal Wiring** (30 minutes)

1. Connect yaw servo signal wire to autopilot MAIN OUT 1
2. Connect pitch servo signal wire to autopilot MAIN OUT 2
3. Route servo wires with adequate slack for full motion range
4. Use cable ties to organize wires along frame
5. Verify no wires can be pinched by moving parts

**Step 6: Telemetry Radio Installation** (15 minutes)

1. Mount telemetry radio on stationary base (not moving platform)
2. Connect to autopilot TELEM1 or TELEM2 port
3. Install radio antenna vertically
4. Verify antenna connector is secure
5. Label connection for future reference

**Step 7: Final Integration** (30 minutes)

1. Install RC receiver (if used) and connect to RC IN port
2. Connect safety switch (if provided with autopilot)
3. Install buzzer (if desired) to I2C or GPIO port
4. Perform complete visual inspection of all connections
5. Check for loose screws, connectors, or cables
6. Verify full range of motion without obstruction
7. Photograph assembly for documentation

### Post-Assembly Testing

**Electrical System Test**:
1. Disconnect servos from autopilot (remove signal wires)
2. Connect USB to autopilot (powers board only)
3. Verify autopilot boots (LED patterns normal)
4. Check GPS LED indicates satellite acquisition
5. Connect telemetry and verify ground station connection
6. Disconnect USB

**Powered System Test**:
1. Connect main battery/power supply
2. Verify voltage at autopilot: 4.75-5.5V
3. Verify voltage at servo power rail: 4.8-6.0V
4. Check for excessive heat in regulators after 2 minutes
5. Measure current draw: Should be <300mA without servos moving

**Servo Integration Test**:
1. Reconnect servo signal wires
2. Power up system
3. Servos should move to trim position (if STARTUP_DELAY > 0)
4. Using ground station, send manual servo commands:
   - Command RC1 (yaw) from 1000-2000μs, observe motion
   - Command RC2 (pitch) from 1000-2000μs, observe motion
5. Verify:
   - Yaw servo rotates in correct direction (CW for increasing command)
   - Pitch servo elevates antenna for increasing command
   - No unusual sounds, vibrations, or binding
   - Servos stop at expected positions

## Pre-Flight Checklist

Before first operational use, complete this comprehensive checklist:

### Hardware Inspection

- [ ] All screws and bolts are tight and secured
- [ ] No loose wires or connectors
- [ ] Servo mounting is rigid with no flexing
- [ ] Antenna is securely attached to pitch servo arm
- [ ] GPS/compass module is firmly mounted
- [ ] Battery or power supply is charged and connections are clean
- [ ] Power switch operates correctly
- [ ] All cable routing provides clearance for full range of motion
- [ ] No signs of damage to components or wiring

### Electrical System Verification

- [ ] Power supply voltage is within specification (measure under load)
- [ ] Autopilot powers up correctly and LEDs indicate normal operation
- [ ] GPS achieves 3D fix within 2 minutes in clear sky
- [ ] Telemetry radio connects to ground station
- [ ] Servo power is isolated from autopilot power (separate BEC recommended)
- [ ] No voltage drop >0.3V when servos move simultaneously
- [ ] No excessive heat from any component after 5 minutes operation

### Software Configuration Verification

- [ ] Firmware version is confirmed as AntennaTracker (not Copter/Plane/Rover)
- [ ] All mandatory parameters configured:
  - [ ] RC1_MIN, RC1_MAX, RC1_TRIM (yaw servo)
  - [ ] RC2_MIN, RC2_MAX, RC2_TRIM (pitch servo)
  - [ ] SERVO_YAW_TYPE and SERVO_PITCH_TYPE
  - [ ] YAW_RANGE, PITCH_MIN, PITCH_MAX
  - [ ] AHRS_ORIENTATION (if non-standard mounting)
- [ ] PID values configured (can start with defaults):
  - [ ] YAW2SRV_P, YAW2SRV_D
  - [ ] PITCH2SRV_P, PITCH2SRV_D
- [ ] Serial ports configured:
  - [ ] SERIAL1_PROTOCOL = 2 (MAVLink2)
  - [ ] SERIAL1_BAUD = 57 or 115
- [ ] Compass enabled: COMPASS_USE = 1
- [ ] Home position set (either GPS or manual START_LAT/START_LON)

### Compass Calibration

- [ ] Compass calibration completed in-situ (installed in tracker)
- [ ] Calibration performed outdoors away from buildings
- [ ] All equipment powered on during calibration
- [ ] Full 3D rotation performed during calibration process
- [ ] Compass offsets saved and verified in parameters
- [ ] Ground station confirms compass heading matches physical direction

### Accelerometer Leveling

- [ ] Antenna positioned at horizon (0° elevation)
- [ ] Accelerometer level command issued via ground station
- [ ] Level process completed (wait 60 seconds without disturbance)
- [ ] Autopilot confirms level operation complete

### Servo Function Test

- [ ] Servo direction verified:
  - [ ] Increasing yaw command rotates clockwise (viewed from above)
  - [ ] Increasing pitch command elevates antenna upward
- [ ] Servo endpoints verified:
  - [ ] Yaw servo reaches both mechanical limits without binding
  - [ ] Pitch servo reaches -90° (down) and +90° (up) without obstruction
- [ ] Servo center positions verified:
  - [ ] At trim position, yaw points to configured zero bearing
  - [ ] At trim position, pitch points to horizon (0° elevation)
- [ ] No servo jitter, buzzing, or oscillation in hold position

### Tracking System Test

- [ ] Tracker successfully arms (servos enabled)
- [ ] AUTO mode selected
- [ ] Simulated vehicle position sent via MAVLink (or actual vehicle telemetry received)
- [ ] Tracker antenna moves to track simulated/actual vehicle position
- [ ] Bearing calculation appears correct (antenna points toward target)
- [ ] Elevation calculation appears correct (antenna pitch matches expected angle)
- [ ] Tracking remains stable without hunting or oscillation
- [ ] Tracker follows moving target smoothly

### Safety Verification

- [ ] Disarm function works correctly (servos disable or go to trim)
- [ ] Safety switch functions as expected (if present)
- [ ] No pinch points or crush hazards during operation
- [ ] Emergency stop procedure established and tested
- [ ] Operating area is clear of obstacles and people during testing
- [ ] Adequate supervision for initial powered tests

## Troubleshooting

### Power and Initialization Issues

**Symptom: Autopilot won't power on**
- **Check**: Power supply voltage (measure with multimeter)
  - Must be 4.75-5.5V for most boards
- **Check**: Current capacity (PSU or BEC must supply >500mA)
- **Check**: Connector polarity (reverse polarity can damage board)
- **Try**: USB power to isolate power supply issue
- **Inspect**: Power connector pins for damage or corrosion

**Symptom: Autopilot reboots when servos move**
- **Cause**: Voltage sag from insufficient power supply current
- **Solution**: Use separate BEC for servo power
- **Solution**: Increase BEC current rating to 5A or higher
- **Check**: All ground connections are solid
- **Add**: Capacitor (470-1000μF) across power supply output

**Symptom: GPS won't acquire fix**
- **Check**: GPS module LED is illuminated (has power)
- **Check**: Clear view of sky (metal buildings, trees can block signal)
- **Wait**: Initial fix can take 2-5 minutes, warm start <30 seconds
- **Check**: GPS cable is securely connected
- **Verify**: GPS is enabled in autopilot parameters
- **Try**: Different GPS module (hardware failure possible)

### Servo Issues

**Symptom: Servos jitter or buzz**
- **Cause**: Noisy power supply or insufficient current
- **Solution**: Use linear regulator BEC instead of switch-mode
- **Solution**: Add 100-1000μF capacitor across servo power rail
- **Check**: Servo power voltage is stable (measure during movement)
- **Lower**: PID gains (P and D terms) if oscillation is cause

**Symptom: Servo moves in wrong direction**
- **Solution**: Set RCx_REVERSED = 1 for that channel (RC1 for yaw, RC2 for pitch)
- **Verify**: Servo is connected to correct autopilot output pin

**Symptom: Servo range is limited or incorrect**
- **Adjust**: RC1_MIN and RC1_MAX (yaw) or RC2_MIN and RC2_MAX (pitch)
- **Verify**: Servo can physically reach desired positions
- **Check**: Mechanical binding or obstruction limiting range
- **Recalibrate**: Measure actual PWM range of servo with servo tester

**Symptom: Servos don't move at all**
- **Check**: Tracker is armed (use ground station to arm)
- **Check**: Safety switch is disengaged (LED should be green/blinking)
- **Verify**: Servos have power (measure 5V on red wire)
- **Test**: Servos with RC receiver or servo tester to rule out hardware failure
- **Check**: SERVO_YAW_TYPE and SERVO_PITCH_TYPE parameters are set to 0 (position mode)

### Compass and Heading Issues

**Symptom: Antenna points in wrong direction**
- **Cause**: Compass not calibrated or poorly calibrated
- **Solution**: Perform complete compass calibration outdoors in-situ
- **Check**: Compass is enabled (COMPASS_USE = 1)
- **Verify**: Compass orientation parameter matches physical mounting
- **Test**: Compare tracker heading to external compass (smartphone app)

**Symptom: Heading drifts over time**
- **Cause**: Magnetic interference from nearby sources
- **Solution**: Move compass away from servos, power wires, metal objects
- **Check**: Compass offsets are within reasonable range (-200 to +200)
- **Recalibrate**: Perform fresh compass calibration
- **Inspect**: Ensure no magnetic materials were added near compass

**Symptom: Compass heading jumps or is erratic**
- **Cause**: Electrical noise coupling into compass
- **Solution**: Use shielded cable for external compass
- **Solution**: Increase separation between compass and RF transmitter
- **Check**: All ground connections are secure
- **Filter**: Reduce compass filter frequency (COMPASS_FILT parameter)

### Tracking Performance Issues

**Symptom: Antenna oscillates back and forth around target**
- **Cause**: PID gains too high (over-tuned)
- **Solution**: Reduce P gain by 25-50%
- **Solution**: Increase D gain slightly to add damping
- **Check**: Mechanical system has no excessive backlash or slop

**Symptom: Antenna lags behind moving target**
- **Cause**: PID gains too low (under-tuned)
- **Solution**: Increase P gain gradually until response improves
- **Solution**: Add feed-forward term (YAW2SRV_FF, PITCH2SRV_FF)
- **Check**: Servos are adequate speed for tracking rate

**Symptom: Tracking is jumpy or steps rather than smooth**
- **Check**: Telemetry link quality (RSSI >50%)
- **Increase**: Telemetry update rate if bandwidth available
- **Filter**: Enable servo output filtering (internal to firmware)
- **Verify**: Vehicle is sending position updates at adequate rate (>2Hz)

**Symptom: Tracker loses target and stops tracking**
- **Cause**: Telemetry link lost for >5 seconds (TRACKING_TIMEOUT_SEC)
- **Solution**: Improve telemetry link (better antennas, higher power)
- **Check**: DISTANCE_MIN parameter (won't track if vehicle <5m by default)
- **Verify**: Vehicle is sending GLOBAL_POSITION_INT MAVLink messages

### Telemetry Issues

**Symptom: No telemetry connection to tracker**
- **Check**: Baud rates match on autopilot and radio (57600 or 115200)
- **Verify**: SERIALx_PROTOCOL = 2 (MAVLink2) for telemetry port
- **Check**: TX and RX are correctly cross-connected
- **Test**: Direct USB connection to rule out radio issue
- **Inspect**: Radio configuration (NetID, AirSpeed match on both ends)

**Symptom: Telemetry disconnects randomly**
- **Check**: RSSI level (should be >50%)
- **Solution**: Improve antenna placement or use higher power radio
- **Reduce**: Telemetry stream rates to reduce bandwidth usage
- **Enable**: ECC (error correction) on radios
- **Check**: Power supply to radio is stable (no voltage sag)

**Symptom: MAVLink parse errors**
- **Verify**: Both radios configured with MAVLink framing enabled
- **Check**: Radio firmware versions are compatible
- **Update**: Radio firmware to latest version
- **Disable**: Other devices on same serial port

### GPS and Home Position Issues

**Symptom: Home position not set automatically**
- **Requirement**: GPS must have 5 consecutive 3D fixes before home is set
- **Wait**: Can take 2-5 minutes after GPS achieves 3D fix
- **Check**: START_LATITUDE and START_LONGITUDE not set to 0,0
- **Manual**: Set home via MAVLink command if GPS not available

**Symptom: Tracker thinks it's at wrong location**
- **Check**: GPS fix quality (must be 3D fix, not 2D)
- **Verify**: GPS antenna has clear view of sky
- **Wait**: GPS accuracy improves after several minutes of fixes
- **Manual**: Use START_LATITUDE and START_LONGITUDE parameters to set home

**Symptom: Can't set home manually via MAVLink**
- **Enable**: AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED (should be enabled by default)
- **Method**: Use MAVProxy: `wp sethome` or Mission Planner: Right-click map → Set Home Here
- **Verify**: Command is received (check ground station messages)

### Mechanical Issues

**Symptom: Excessive mechanical noise or vibration**
- **Check**: All mounting screws are tight
- **Inspect**: Servo gears for wear or damage
- **Verify**: Servo arms are securely attached to servo output shafts
- **Lubricate**: Moving parts with appropriate grease (sparingly)
- **Balance**: Ensure antenna weight is balanced on pitch axis

**Symptom: Limited range of motion**
- **Inspect**: Mechanical obstructions throughout full ROM
- **Check**: Cables have adequate slack and aren't limiting motion
- **Verify**: YAW_RANGE, PITCH_MIN, PITCH_MAX parameters match physical limits
- **Adjust**: Mechanical stops or endpoint parameters

**Symptom: Tracker wobbles or is unstable**
- **Cause**: Insufficient servo torque for antenna weight
- **Solution**: Upgrade to higher torque servos
- **Solution**: Reduce antenna size/weight
- **Reinforce**: Mounting brackets and connections
- **Add**: Counterweight to balance system

### Advanced Diagnostics

**Using MAVProxy for Troubleshooting**:
```bash
# Monitor attitude in real-time
watch ATTITUDE

# Monitor GPS status
watch GPS_RAW_INT

# Monitor servo outputs
watch SERVO_OUTPUT_RAW

# View all parameters
param show *

# Set parameter
param set YAW2SRV_P 0.15

# Force arm (for testing, use with caution)
arm throttle

# Disarm
disarm
```

**Mission Planner Flight Data Screen**:
- Monitor real-time heading, pitch, roll in HUD
- View servo PWM outputs in Status tab
- Check GPS satellite count and HDOP
- Monitor telemetry link RSSI
- View active mode and arming status

**Log File Analysis**:
If SD card logging is enabled, logs provide detailed troubleshooting data:
1. Download log via ground station (DataFlash Logs)
2. Review using Mission Planner → DataFlash Logs → Review a Log
3. Key graphs to examine:
   - ATT.Yaw vs desired heading (tracking accuracy)
   - RCOU.C1, RCOU.C2 (servo PWM outputs)
   - GPS.Status, GPS.NSats (GPS health)
   - MAG.MagX/Y/Z (compass data)
   - VBAR, VPOS (vehicle position as seen by tracker)

## Additional Resources

**Official Documentation**:
- ArduPilot AntennaTracker Wiki: https://ardupilot.org/antennatracker/
- ArduPilot Hardware Wiki: https://ardupilot.org/ardupilot/
- MAVLink Protocol: https://mavlink.io/

**Community Resources**:
- ArduPilot Discourse Forum: https://discuss.ardupilot.org/
- ArduPilot Discord Server: https://ardupilot.org/discord
- GitHub Repository: https://github.com/ArduPilot/ardupilot

**Ground Station Software**:
- Mission Planner (Windows): https://ardupilot.org/planner/
- QGroundControl (Cross-platform): http://qgroundcontrol.com/
- MAVProxy (Command-line): https://github.com/ArduPilot/MAVProxy

**Related Documentation Files**:
- `AntennaTracker/README.md` - System architecture and software overview
- `AntennaTracker/docs/FLIGHT_MODES.md` - Detailed mode descriptions
- `BUILD_SYSTEM.md` - Building firmware from source
- `SAFETY_CRITICAL.md` - Safety-critical code paths

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**Applies to**: AntennaTracker firmware 4.0+  
**Source Code References**: `AntennaTracker/system.cpp`, `AntennaTracker/servos.cpp`, `AntennaTracker/sensors.cpp`, `AntennaTracker/tracking.cpp`, `AntennaTracker/config.h`, `AntennaTracker/Parameters.h`

**Disclaimer**: This document provides general guidance for AntennaTracker hardware setup. Always follow manufacturer specifications for specific components. Test thoroughly in controlled environment before operational use. The ArduPilot development team and contributors assume no liability for hardware damage, personal injury, or operational failures resulting from following this guide.