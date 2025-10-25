# ArduSub Frame Configurations

![Frame Config](https://img.shields.io/badge/component-frame_configuration-blue)
![Safety Level](https://img.shields.io/badge/safety-configuration-orange)

## Table of Contents
- [Overview](#overview)
- [Frame Type Selection](#frame-type-selection)
- [Supported Frame Configurations](#supported-frame-configurations)
  - [BlueROV1 (Frame 0)](#bluerov1-frame-0)
  - [Vectored (Frame 1)](#vectored-frame-1)
  - [Vectored 6DOF (Frame 2)](#vectored-6dof-frame-2)
  - [Vectored 6DOF 90° (Frame 3)](#vectored-6dof-90-frame-3)
  - [SimpleROV-3 (Frame 4)](#simplerov-3-frame-4)
  - [SimpleROV-4 (Frame 5)](#simplerov-4-frame-5)
  - [SimpleROV-5 (Frame 5)](#simplerov-5-frame-5)
  - [Custom (Frame 7)](#custom-frame-7)
- [Understanding 6DOF Movement](#understanding-6dof-movement)
- [Motor Mixing Explained](#motor-mixing-explained)
- [Frame-Specific Parameters](#frame-specific-parameters)
- [Motor Configuration](#motor-configuration)
- [Custom Frame Setup](#custom-frame-setup)
- [Motor Testing and Verification](#motor-testing-and-verification)
- [Troubleshooting](#troubleshooting)

## Overview

ArduSub supports multiple ROV and AUV frame configurations, each optimized for different thruster layouts and movement capabilities. The frame configuration determines how pilot inputs are translated into individual motor commands through a motor mixing matrix.

**Source Files**: 
- `/libraries/AP_Motors/AP_Motors6DOF.cpp` - Frame definitions and motor mixing implementations
- `/libraries/AP_Motors/AP_Motors6DOF.h` - Motor control interface
- `/ArduSub/Parameters.cpp` - Frame configuration parameter (FRAME_CONFIG)
- `/ArduSub/motors.cpp` - Motor output and testing functions

### Key Concepts

**Motor Mixing**: The process of converting pilot control inputs (roll, pitch, yaw, throttle, forward, lateral) into individual thruster commands based on each motor's contribution to vehicle motion.

**6 Degrees of Freedom (6DOF)**: Complete control authority over all possible vehicle movements:
- **3 Translational DOF**: Forward/Backward, Left/Right, Up/Down
- **3 Rotational DOF**: Roll, Pitch, Yaw

**Vectored Thrusters**: Horizontally-mounted thrusters that can contribute to both translational movement and vertical control through coordinated thrust vectoring.

## Frame Type Selection

The frame configuration is selected using the **FRAME_CONFIG** parameter.

```cpp
// Source: /ArduSub/Parameters.cpp:350-356
// @Param: FRAME_CONFIG
// @DisplayName: Frame configuration
// @Description: Set this parameter according to your vehicle/motor configuration
// @User: Standard
// @RebootRequired: True
// @Values: 0:BlueROV1, 1:Vectored, 2:Vectored_6DOF, 3:Vectored_6DOF_90, 
//          4:SimpleROV-3, 5:SimpleROV-4, 6:SimpleROV-5, 7:Custom
```

| Frame ID | Frame Name | Thrusters | DOF | Primary Use Case |
|----------|------------|-----------|-----|------------------|
| 0 | BlueROV1 | 6 | 5DOF | Legacy BlueROV1 platform |
| 1 | Vectored | 6 | 5DOF | Standard vectored ROV, no roll control |
| 2 | Vectored_6DOF | 8 | 6DOF | Full 6DOF control with vectored thrusters |
| 3 | Vectored_6DOF_90 | 8 | 6DOF | BlueROV2 Heavy configuration |
| 4 | SimpleROV-3 | 3 | 3DOF | Minimal configuration (forward, yaw, vertical) |
| 5 | SimpleROV-4 | 4 | 4DOF | Basic ROV (forward, yaw, vertical, roll) |
| 6 | SimpleROV-5 | 5 | 5DOF | Extended SimpleROV with lateral movement |
| 7 | Custom | Variable | Variable | User-defined configuration |

> **⚠️ Important**: Changing FRAME_CONFIG requires a reboot to take effect.

## Supported Frame Configurations

### BlueROV1 (Frame 0)

The original BlueROV1 frame configuration with 6 motors providing 5 degrees of freedom (no roll control).

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:134-142

                Front
        Motor 6 ←─────→ (Lateral)
                  │
         Motor 1 ─┼─ Motor 2  (Forward/Yaw)
                  │
         Motor 3 ─┴─ Motor 4  (Vertical/Roll/Pitch)
                  │
               Motor 5         (Vertical/Pitch)
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Function |
|-------|------|-------|-----|----------|---------|---------|----------|
| 1 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Left |
| 2 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Right |
| 3 | -0.5 | 0.5 | 0.0 | 0.45 | 0.0 | 0.0 | Vertical + Attitude |
| 4 | 0.5 | 0.5 | 0.0 | 0.45 | 0.0 | 0.0 | Vertical + Attitude |
| 5 | 0.0 | -1.0 | 0.0 | 1.0 | 0.0 | 0.0 | Vertical + Pitch |
| 6 | -0.25 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 | Lateral |

#### Movement Capabilities

- ✅ Forward/Backward
- ✅ Left/Right (Lateral)
- ✅ Up/Down (Vertical)
- ✅ Pitch
- ✅ Yaw
- ❌ Roll (limited, dependent on other movements)

### Vectored (Frame 1)

Standard 6-thruster vectored configuration commonly used in entry-level ROVs. Provides 5DOF control without independent roll authority.

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:168-176

                  Front
        
     M1 ↗              ↖ M3    (Vectored: Forward/Lateral/Yaw)
             ╔════╗
     M2 ↘    ║ROV ║    ↙ M4
             ╚════╝
     
        M5 ⊕      ⊕ M6         (Vertical: Roll/Throttle)
        
    ⊕ = Thruster pointing down (vertical)
    ↗↖↙↘ = Thruster at 45° angle (vectored)
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Thruster Position |
|-------|------|-------|-----|----------|---------|---------|-------------------|
| 1 | 0.0 | 0.0 | 1.0 | 0.0 | -1.0 | 1.0 | Front-Left (vectored) |
| 2 | 0.0 | 0.0 | -1.0 | 0.0 | -1.0 | -1.0 | Front-Right (vectored) |
| 3 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 1.0 | Rear-Left (vectored) |
| 4 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | -1.0 | Rear-Right (vectored) |
| 5 | 1.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Left vertical |
| 6 | -1.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Right vertical |

#### Movement Capabilities

- ✅ Forward/Backward (coupled through 4 vectored thrusters)
- ✅ Lateral (coupled through 4 vectored thrusters)
- ✅ Up/Down (2 dedicated vertical thrusters)
- ✅ Yaw (differential thrust on vectored thrusters)
- ✅ Roll (using vertical thrusters differentially)
- ❌ Pitch (no independent pitch control)

#### Special Characteristics

- **Forward/Vertical Coupling**: Vertical thrust affects forward movement and vice versa on vectored thrusters
- **Coupling Compensation**: The `MOT_FV_CPLNG_K` parameter (default 1.0) compensates for hydrodynamic coupling
- **Implementation**: Uses `output_armed_stabilizing_vectored()` function with forward coupling limits

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:450-453
// Forward coupling compensation
float forward_coupling_limit = 1 - _forwardVerticalCouplingFactor * fabsf(throttle_thrust);
```

### Vectored 6DOF (Frame 2)

Full 6 degrees of freedom configuration with 8 thrusters: 4 horizontal vectored thrusters for translation and yaw, plus 4 vertical thrusters for altitude and full attitude control.

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:156-166

                  Front
        
     M1 ↗              ↖ M3    (Horizontal vectored)
             ╔════╗
     M2 ↘    ║ROV ║    ↙ M4
             ╚════╝
     
     M7 ⊕              ⊕ M5    (Vertical thrusters)
     
     M8 ⊕              ⊕ M6    (Vertical thrusters)
     
                 Rear
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Thruster Type |
|-------|------|-------|-----|----------|---------|---------|---------------|
| 1 | 0.0 | 0.0 | 1.0 | 0.0 | -1.0 | 1.0 | Front-Left horizontal |
| 2 | 0.0 | 0.0 | -1.0 | 0.0 | -1.0 | -1.0 | Front-Right horizontal |
| 3 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 1.0 | Rear-Left horizontal |
| 4 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | -1.0 | Rear-Right horizontal |
| 5 | 1.0 | -1.0 | 0.0 | -1.0 | 0.0 | 0.0 | Front-Left vertical |
| 6 | -1.0 | -1.0 | 0.0 | -1.0 | 0.0 | 0.0 | Front-Right vertical |
| 7 | 1.0 | 1.0 | 0.0 | -1.0 | 0.0 | 0.0 | Rear-Left vertical |
| 8 | -1.0 | 1.0 | 0.0 | -1.0 | 0.0 | 0.0 | Rear-Right vertical |

#### Movement Capabilities

- ✅ Forward/Backward (4 horizontal thrusters)
- ✅ Lateral Left/Right (4 horizontal thrusters)
- ✅ Up/Down (4 vertical thrusters)
- ✅ Roll (differential vertical thrust)
- ✅ Pitch (front/rear vertical thrust differential)
- ✅ Yaw (differential horizontal thrust)

#### Special Characteristics

- **Full 6DOF Control**: Independent control of all 6 degrees of freedom
- **Decoupled Movement**: Horizontal and vertical thrusters separated for minimal coupling
- **Special Motor Mixing**: Uses `output_armed_stabilizing_vectored_6dof()` with separate normalization for RPT (Roll/Pitch/Throttle) and YFL (Yaw/Forward/Lateral) groups

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:530-541
// Separate calculations for vertical (RPT) and horizontal (YFL) thruster groups
rpt_out[i] = roll_thrust * _roll_factor[i] +
             pitch_thrust * _pitch_factor[i] +
             throttle_thrust * _throttle_factor[i];

yfl_out[i] = yaw_thrust * _yaw_factor[i] +
             forward_thrust * _forward_factor[i] +
             lateral_thrust * _lateral_factor[i];
```

### Vectored 6DOF 90° (Frame 3)

BlueROV2 Heavy configuration with 8 thrusters arranged at 90° to the frame axis. This is the most common heavy-lift ROV configuration.

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:144-154

                  Front
        
      M6 ⊕                 ⊕ M1    (Vertical, angled for attitude)
             ╔════╗
      M2 →   ║ROV ║   ← M7         (Horizontal forward)
             ╚════╝
      M8 ⊕                 ⊕ M3    (Vertical, angled for attitude)
      
        M4 ←              → M5     (Horizontal lateral)
        
                 Rear
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Position |
|-------|------|-------|-----|----------|---------|---------|----------|
| 1 | 1.0 | 1.0 | 0.0 | 1.0 | 0.0 | 0.0 | Front-Right vertical |
| 2 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | 0.0 | Front-Left forward |
| 3 | 1.0 | -1.0 | 0.0 | 1.0 | 0.0 | 0.0 | Rear-Right vertical |
| 4 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 | Left lateral |
| 5 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 | Right lateral |
| 6 | -1.0 | 1.0 | 0.0 | 1.0 | 0.0 | 0.0 | Front-Left vertical |
| 7 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 0.0 | Front-Right forward |
| 8 | -1.0 | -1.0 | 0.0 | 1.0 | 0.0 | 0.0 | Rear-Left vertical |

#### Movement Capabilities

- ✅ Forward/Backward (2 dedicated forward thrusters)
- ✅ Lateral Left/Right (2 dedicated lateral thrusters)
- ✅ Up/Down (4 vertical thrusters)
- ✅ Roll (differential vertical thrust left/right)
- ✅ Pitch (differential vertical thrust front/rear)
- ✅ Yaw (differential forward thrust)

#### Special Characteristics

- **Heavy Lift Capability**: 4 vertical thrusters provide maximum lifting force
- **Decoupled Translation**: Separate thrusters for each translational axis minimize coupling
- **90° Thruster Orientation**: Forward and lateral thrusters aligned with vehicle axes
- **Optimal for Inspection**: Stable hovering and precise positioning

### SimpleROV-3 (Frame 4)

Minimal 3-thruster configuration providing basic ROV functionality with forward, yaw, and vertical control.

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:182-187

                Front
        
        M1 ↗        ↖ M2    (Forward/Yaw)
             ╔════╗
             ║ROV ║
             ╚════╝
        
              M3 ⊕          (Vertical only)
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Function |
|-------|------|-------|-----|----------|---------|---------|----------|
| 1 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Left |
| 2 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Right |
| 3 | 0.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Vertical |

#### Movement Capabilities

- ✅ Forward/Backward
- ✅ Up/Down
- ✅ Yaw
- ❌ Lateral (no left/right movement)
- ❌ Roll
- ❌ Pitch

#### Use Cases

- Educational platforms
- Tethered observation systems
- Low-cost prototypes
- Underwater camera platforms

### SimpleROV-4 (Frame 5)

Basic 4-thruster ROV configuration adding roll control to the SimpleROV-3 design.

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:188-198

                Front
        
        M1 ↗        ↖ M2    (Forward/Yaw)
             ╔════╗
             ║ROV ║
             ╚════╝
        
        M3 ⊕          ⊕ M4  (Vertical/Roll)
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Function |
|-------|------|-------|-----|----------|---------|---------|----------|
| 1 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Left |
| 2 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Right |
| 3 | 1.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Vertical + Roll Left |
| 4 | -1.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Vertical + Roll Right |

#### Movement Capabilities

- ✅ Forward/Backward
- ✅ Up/Down
- ✅ Yaw
- ✅ Roll
- ❌ Lateral
- ❌ Pitch

### SimpleROV-5 (Frame 5)

Extended SimpleROV configuration with 5 thrusters adding lateral movement capability.

#### Motor Layout

```
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:188-198

                Front
        
        M1 ↗        ↖ M2    (Forward/Yaw)
             ╔════╗
        M5 → ║ROV ║         (Lateral)
             ╚════╝
        
        M3 ⊕          ⊕ M4  (Vertical/Roll)
```

#### Motor Mixing Matrix

| Motor | Roll | Pitch | Yaw | Vertical | Forward | Lateral | Function |
|-------|------|-------|-----|----------|---------|---------|----------|
| 1 | 0.0 | 0.0 | -1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Left |
| 2 | 0.0 | 0.0 | 1.0 | 0.0 | 1.0 | 0.0 | Forward + Yaw Right |
| 3 | 1.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Vertical + Roll Left |
| 4 | -1.0 | 0.0 | 0.0 | -1.0 | 0.0 | 0.0 | Vertical + Roll Right |
| 5 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 1.0 | Lateral |

#### Movement Capabilities

- ✅ Forward/Backward
- ✅ Left/Right (Lateral)
- ✅ Up/Down
- ✅ Yaw
- ✅ Roll
- ❌ Pitch

#### Use Cases

- Intermediate ROV platforms
- Pipeline inspection
- Pool and tank operations
- Educational advanced projects

### Custom (Frame 7)

User-defined custom frame configuration for specialized ROV designs.

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:178-180
case SUB_FRAME_CUSTOM:
    // Put your custom motor setup here
    //break;
```

#### Implementation

Custom frame configurations require modifying the source code in `AP_Motors6DOF.cpp` within the `SUB_FRAME_CUSTOM` case. See [Custom Frame Setup](#custom-frame-setup) section for detailed instructions.

## Understanding 6DOF Movement

Six Degrees of Freedom (6DOF) refers to the freedom of movement in three-dimensional space, consisting of three translational movements and three rotational movements.

### Translational Degrees of Freedom

```
         ↑ Heave (Up/Down)
         │
         │     ↗ Surge (Forward/Backward)
         │   ╱
         │ ╱
         └────────→ Sway (Left/Right)
```

1. **Surge**: Forward (+) and Backward (-) movement along the X-axis
2. **Sway**: Right (+) and Left (-) movement along the Y-axis  
3. **Heave**: Down (+) and Up (-) movement along the Z-axis

### Rotational Degrees of Freedom

```
         Roll →  ╔════╗  ← Pitch
                 ║ ⊙  ║
         Yaw ↻   ╚════╝
```

1. **Roll**: Rotation around the longitudinal X-axis (left side up/down)
2. **Pitch**: Rotation around the lateral Y-axis (nose up/down)
3. **Yaw**: Rotation around the vertical Z-axis (heading change)

### DOF by Frame Type

| Frame | Surge | Sway | Heave | Roll | Pitch | Yaw | Total DOF |
|-------|-------|------|-------|------|-------|-----|-----------|
| BlueROV1 | ✅ | ✅ | ✅ | ⚠️ | ✅ | ✅ | 5DOF |
| Vectored | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ | 5DOF |
| Vectored_6DOF | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | **6DOF** |
| Vectored_6DOF_90 | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | **6DOF** |
| SimpleROV-3 | ✅ | ❌ | ✅ | ❌ | ❌ | ✅ | 3DOF |
| SimpleROV-4 | ✅ | ❌ | ✅ | ✅ | ❌ | ✅ | 4DOF |
| SimpleROV-5 | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ | 5DOF |

✅ = Full independent control  
⚠️ = Limited/coupled control  
❌ = No control

## Motor Mixing Explained

Motor mixing is the mathematical process of converting pilot control inputs into individual motor thrust commands.

### Motor Mixing Formula

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:337-350
// For each motor:
motor_output[i] = roll_input    * roll_factor[i] +
                  pitch_input   * pitch_factor[i] +
                  yaw_input     * yaw_factor[i] +
                  throttle_input * throttle_factor[i] +
                  forward_input  * forward_factor[i] +
                  lateral_input  * lateral_factor[i];

// Then apply motor direction and constraints:
final_output[i] = constrain(motor_reverse[i] * motor_output[i], -1.0, 1.0);
```

### Motor Factors

Each motor has 6 factors defining its contribution to vehicle movement:

| Factor | Range | Purpose |
|--------|-------|---------|
| **roll_factor** | -1.0 to 1.0 | Motor contribution to roll moment |
| **pitch_factor** | -1.0 to 1.0 | Motor contribution to pitch moment |
| **yaw_factor** | -1.0 to 1.0 | Motor contribution to yaw moment |
| **throttle_factor** | -1.0 to 1.0 | Motor contribution to vertical thrust |
| **forward_factor** | -1.0 to 1.0 | Motor contribution to forward thrust |
| **lateral_factor** | -1.0 to 1.0 | Motor contribution to lateral thrust |

### Understanding Factor Values

- **1.0**: Full positive contribution
- **0.0**: No contribution
- **-1.0**: Full negative (opposite) contribution
- **0.5**: Half contribution (reduced authority)

### Example: Vectored 6DOF Motor 1

```
Motor 1 Factors:
- Roll: 0.0      → No roll contribution
- Pitch: 0.0     → No pitch contribution  
- Yaw: 1.0       → Full yaw right contribution
- Throttle: 0.0  → No vertical contribution
- Forward: -1.0  → Full backward thrust
- Lateral: 1.0   → Full left thrust

When pilot commands:
- Forward stick: Motor 1 thrusts backward (negative forward factor)
- Left stick: Motor 1 thrusts to help move left
- Yaw right: Motor 1 thrusts to help yaw right
```

### Vectored Frame Special Handling

Vectored and Vectored_6DOF frames use special motor mixing algorithms to handle hydrodynamic coupling:

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:450-470
// Forward/Vertical coupling compensation on vectored frames
float forward_coupling_limit = 1 - _forwardVerticalCouplingFactor * fabsf(throttle_thrust);

// Limit "rear" thrusters when moving forward to prevent pitch coupling
if (!is_zero(forward_thrust)) {
    if ((forward_thrust < 0) == (forward_coupling_direction[i] < 0)) {
        forward_thrust_limited = constrain(forward_thrust, 
                                          -forward_coupling_limit, 
                                          forward_coupling_limit);
    }
}
```

## Frame-Specific Parameters

### Core Frame Parameters

#### FRAME_CONFIG

```
Parameter: FRAME_CONFIG
Type: Integer (0-7)
Default: 1 (Vectored)
Reboot Required: Yes
Source: /ArduSub/Parameters.cpp:350-356
```

Sets the frame type. Must match your physical thruster configuration.

**Critical**: Always verify motor test output matches expected behavior after changing this parameter.

#### MOT_FV_CPLNG_K

```
Parameter: MOT_FV_CPLNG_K  
Type: Float
Range: 0.0 - 1.5
Default: 1.0
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:85-91
```

**Forward/Vertical Coupling Factor** - Compensates for hydrodynamic coupling on vectored frames.

- **0.0**: No coupling compensation (may cause pitch oscillations during forward motion)
- **1.0**: Standard compensation (recommended starting point)
- **1.2**: Increased compensation for heavy vehicles or strong currents
- **1.5**: Maximum compensation

**Applies to**: Vectored (Frame 1) only

**Tuning Guide**:
1. Start with default value (1.0)
2. Command forward motion at medium speed
3. If vehicle pitches nose-down: Increase parameter
4. If vehicle remains level: No adjustment needed
5. Adjust in increments of 0.1

### Motor Direction Parameters

Each motor can be individually reversed without rewiring:

```
Parameters: MOT_1_DIRECTION through MOT_12_DIRECTION
Type: Integer
Values: 1 (normal), -1 (reverse)
Default: 1
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:29-119
```

#### When to Reverse Motors

- Motor spins opposite to expected direction during motor test
- Thruster installed in reversed orientation
- Easier than physically rewiring the motor

#### Example Configuration

```
# BlueROV2 Heavy standard configuration
MOT_1_DIRECTION = 1   # Front-Right vertical (CCW)
MOT_2_DIRECTION = -1  # Front-Left forward (reversed)
MOT_3_DIRECTION = 1   # Rear-Right vertical (CCW)
MOT_4_DIRECTION = 1   # Left lateral (CCW)
MOT_5_DIRECTION = 1   # Right lateral (CCW)
MOT_6_DIRECTION = -1  # Front-Left vertical (reversed)
MOT_7_DIRECTION = 1   # Front-Right forward (CCW)
MOT_8_DIRECTION = -1  # Rear-Left vertical (reversed)
```

### PWM Output Parameters

```
Parameter: MOT_PWM_MIN
Range: 1000-2000 µs
Default: 1100 µs

Parameter: MOT_PWM_MAX  
Range: 1000-2000 µs
Default: 1900 µs

Source: Inherited from AP_MotorsMulticopter
```

Defines the PWM pulse width range sent to ESCs:
- **1500 µs**: Neutral (stopped)
- **1100-1500 µs**: Reverse thrust
- **1500-1900 µs**: Forward thrust

**ESC Compatibility**: Must match your ESC's configured range (usually 1100-1900 for bidirectional ESCs).

### Current Limiting Parameters

```
Parameter: MOT_BAT_CURR_MAX
Type: Float (Amperes)
Default: 0 (disabled)
Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:362-396
```

Automatically limits motor output to prevent exceeding maximum battery current.

**Current Limiting Algorithm**:
```cpp
// Predicts current based on rate of change
predicted_current = current + (current_change_rate * dt * 5);

// Scales all motor outputs if limit would be exceeded
if (predicted_current > MOT_BAT_CURR_MAX) {
    output_scale = reduce_to_limit();
}
```

**Setup**:
1. Determine maximum safe continuous current for your battery
2. Set MOT_BAT_CURR_MAX to 80% of that value (safety margin)
3. Monitor actual current during aggressive maneuvers
4. Adjust if current spikes above safe levels

## Motor Configuration

### Motor Numbering Convention

ArduSub uses a specific motor numbering convention that determines servo output channel assignments:

```
Motor Number  →  Servo Output Channel
═══════════════════════════════════
Motor 1       →  SERVO1 (RC Output 1)
Motor 2       →  SERVO2 (RC Output 2)
Motor 3       →  SERVO3 (RC Output 3)
Motor 4       →  SERVO4 (RC Output 4)
Motor 5       →  SERVO5 (RC Output 5)
Motor 6       →  SERVO6 (RC Output 6)
Motor 7       →  SERVO7 (RC Output 7)
Motor 8       →  SERVO8 (RC Output 8)
```

### Physical Wiring

Connect ESCs to autopilot outputs according to the motor number in your frame configuration.

**BlueROV2 Heavy Example**:
```
Autopilot Output  →  ESC  →  Thruster Location
═════════════════════════════════════════════════
SERVO1 (Main Out 1) → ESC1 → Front-Right Vertical
SERVO2 (Main Out 2) → ESC2 → Front-Left Forward
SERVO3 (Main Out 3) → ESC3 → Rear-Right Vertical
SERVO4 (Main Out 4) → ESC4 → Left Lateral
SERVO5 (Main Out 5) → ESC5 → Right Lateral
SERVO6 (Main Out 6) → ESC6 → Front-Left Vertical
SERVO7 (Main Out 7) → ESC7 → Front-Right Forward
SERVO8 (Main Out 8) → ESC8 → Rear-Left Vertical
```

### ESC Configuration

Before first use, configure your ESCs for bidirectional operation:

1. **Calibration Range**: Set to match MOT_PWM_MIN and MOT_PWM_MAX (typically 1100-1900 µs)
2. **Deadband**: Set to 25 µs or as recommended by manufacturer  
3. **Bidirectional Mode**: Enable (required for ROV operation)
4. **Timing**: Auto or Medium (consult ESC manual)
5. **Brake**: Disabled (allows free coasting)

### Motor Testing Order

Each frame defines a testing order for systematic verification:

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:201-210
add_motor_raw_6dof(motor_num, roll_fac, pitch_fac, yaw_fac, 
                   throttle_fac, forward_fac, lat_fac, 
                   testing_order);  // ← Testing sequence
```

The testing order determines which motor is activated when using the motor test feature.

## Custom Frame Setup

### Creating a Custom Frame Configuration

To implement a custom frame configuration:

#### Step 1: Modify Source Code

Edit `/libraries/AP_Motors/AP_Motors6DOF.cpp` in the `setup_motors()` function:

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:178-180
case SUB_FRAME_CUSTOM:
    _frame_class_string = "CUSTOM";
    
    // Add your motors here using add_motor_raw_6dof()
    // Parameters: motor_num, roll, pitch, yaw, throttle, forward, lateral, test_order
    
    // Example: 6-thruster custom configuration
    add_motor_raw_6dof(AP_MOTORS_MOT_1,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,  0.5f, 1);
    add_motor_raw_6dof(AP_MOTORS_MOT_2,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f, -0.5f, 2);
    add_motor_raw_6dof(AP_MOTORS_MOT_3,  1.0f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f, 3);
    add_motor_raw_6dof(AP_MOTORS_MOT_4, -1.0f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f, 4);
    add_motor_raw_6dof(AP_MOTORS_MOT_5,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f, 5);
    add_motor_raw_6dof(AP_MOTORS_MOT_6,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f, 6);
    
    break;
```

#### Step 2: Calculate Motor Factors

For each motor, determine its contribution to each degree of freedom:

**Roll Factor**:
- Positive: Motor on left side or contributes to right roll
- Negative: Motor on right side or contributes to left roll
- Magnitude: Proportional to moment arm distance from roll axis

**Pitch Factor**:
- Positive: Motor at rear or contributes to nose-up pitch
- Negative: Motor at front or contributes to nose-down pitch
- Magnitude: Proportional to moment arm distance from pitch axis

**Yaw Factor**:
- Positive: Motor thrust contributes to yaw right
- Negative: Motor thrust contributes to yaw left
- Magnitude: Proportional to moment arm and thrust angle

**Throttle (Vertical) Factor**:
- Positive: Thruster points down (provides upward thrust)
- Negative: Thruster points up (provides downward thrust)
- Magnitude: Cosine of angle from vertical (1.0 = purely vertical)

**Forward Factor**:
- Positive: Thruster contributes to forward motion
- Negative: Thruster contributes to backward motion
- Magnitude: Cosine of angle from longitudinal axis

**Lateral Factor**:
- Positive: Thruster contributes to right motion
- Negative: Thruster contributes to left motion
- Magnitude: Cosine of angle from lateral axis

#### Step 3: Determine Motor Mixing Algorithm

Choose appropriate output function:

```cpp
// Standard mixing (most frames)
output_armed_stabilizing()

// Vectored frame with forward/vertical coupling
output_armed_stabilizing_vectored()

// 6DOF vectored with separate RPT/YFL groups
output_armed_stabilizing_vectored_6dof()
```

Modify frame class check in `output_armed_stabilizing()` if needed:

```cpp
// Source: /libraries/AP_Motors/AP_Motors6DOF.cpp:294-298
if ((sub_frame_t)_active_frame_class == SUB_FRAME_CUSTOM) {
    output_armed_stabilizing_vectored();  // or your custom function
} else {
    // Standard mixing
}
```

#### Step 4: Compile and Test

1. Compile modified firmware: `./waf copter`
2. Upload to autopilot
3. Set `FRAME_CONFIG = 7` (Custom)
4. Reboot autopilot
5. Perform motor test to verify each motor
6. Test in water with tether safety

#### Step 5: Document Your Configuration

Create documentation including:
- Motor layout diagram
- Motor mixing matrix table
- Movement capability list
- Special characteristics or limitations
- Parameter recommendations

### Custom Frame Design Considerations

#### Thruster Placement Guidelines

**Symmetry**: Maintain symmetry to minimize unwanted coupling
- Mirror motor placements left/right for balanced roll
- Mirror motor placements front/rear for balanced pitch

**Moment Arms**: Larger moment arms provide more torque
- Place attitude control thrusters far from center of rotation
- Vertical thrusters near edges provide better roll/pitch authority

**Thrust Vectoring**: Angled thrusters provide multi-axis control
- 45° angles split force evenly between two axes
- Shallow angles (<30°) favor one axis strongly

**Redundancy**: Consider failure modes
- Can vehicle surface if one vertical thruster fails?
- Can vehicle return if one horizontal thruster fails?

#### Factor Calculation Example

For a thruster at 45° pointing forward-right:

```
forward_factor = cos(45°) = 0.707   (forward component)
lateral_factor = cos(45°) = 0.707   (right component)
yaw_factor = depends on position relative to center
throttle_factor = 0                  (horizontal thruster)
```

## Motor Testing and Verification

### Motor Test Mode

ArduSub provides a motor test mode for safely verifying motor configuration.

#### Enabling Motor Test

```cpp
// Source: /ArduSub/motors.cpp:33-60
bool Sub::init_motor_test()
{
    // Requirements:
    // 1. Vehicle must be armed
    // 2. Hardware safety switch must be disabled
    // 3. 10-second cooldown after previous failure
    
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }
    
    return true;
}
```

#### Using Motor Test via MAVLink

Send `MAV_CMD_DO_MOTOR_TEST` command with parameters:

```
param1: motor_number (1-12)
param2: throttle_type (1=PWM, 2=Percent)
param3: throttle_value (1100-1900 for PWM, 0-100 for percent)
param4: timeout (seconds, ignored)
param5: motor_count (ignored)
param6: test_type (must be MOTOR_TEST_ORDER_BOARD = 0)
```

**Example via MAVProxy**:
```bash
# Test motor 1 at 30% throttle
motor test 1 2 30 0 1 0

# Test motor 3 at 1600µs PWM
motor test 3 1 1600 0 1 0
```

#### Motor Test Safety Features

```cpp
// Source: /ArduSub/motors.cpp:66-84
bool Sub::verify_motor_test()
{
    // Requires continuous commands at >2Hz
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_INFO, "Motor test timed out!");
        ap.motor_test = false;
        AP::arming().disarm(AP_Arming::Method::MOTORTEST);
        return false;
    }
    return true;
}
```

**Safety Features**:
- Automatic disarm if commands stop for >500ms
- 10-second cooldown after test failure
- Requires armed state
- Hardware safety switch must be off

### Verification Procedure

#### Step 1: Visual Inspection

Before powering on:
1. Verify all ESC connections match frame configuration
2. Check thruster directions match expected orientation
3. Ensure propellers are installed correctly
4. Verify no loose wiring near propellers

#### Step 2: Bench Test (Out of Water)

**⚠️ Warning**: Thrusters should only run briefly out of water to prevent overheating.

1. Set FRAME_CONFIG parameter
2. Reboot autopilot
3. Arm vehicle (in Manual mode)
4. Test each motor individually at low power (20-30%)
5. Verify spin direction matches expected
6. If wrong direction: Set MOT_X_DIRECTION = -1

**Expected Motor Behavior by Frame**:

*Vectored 6DOF 90° (BlueROV2 Heavy)*:
```
Motor 1 (Front-Right Vertical): Spins to push water down
Motor 2 (Front-Left Forward): Spins to push water backward
Motor 3 (Rear-Right Vertical): Spins to push water down
Motor 4 (Left Lateral): Spins to push water right
Motor 5 (Right Lateral): Spins to push water left
Motor 6 (Front-Left Vertical): Spins to push water down
Motor 7 (Front-Right Forward): Spins to push water backward
Motor 8 (Rear-Left Vertical): Spins to push water down
```

#### Step 3: Water Test

Perform in controlled environment (pool/tank) with safety tether:

1. **Vertical Control**:
   - Throttle up → vehicle rises
   - Throttle down → vehicle sinks
   - Vehicle remains level (no unwanted roll/pitch)

2. **Forward/Backward**:
   - Forward stick → vehicle moves forward
   - Backward stick → vehicle moves backward
   - Minimal pitch change during motion

3. **Lateral Control** (if equipped):
   - Right stick → vehicle moves right
   - Left stick → vehicle moves left
   - Minimal roll during motion

4. **Yaw Control**:
   - Yaw right → vehicle rotates clockwise (viewed from above)
   - Yaw left → vehicle rotates counter-clockwise
   - No unwanted translation

5. **Roll Control** (6DOF frames):
   - Roll right → right side down, left side up
   - Roll left → left side down, right side up

6. **Pitch Control** (6DOF frames):
   - Pitch up → nose up, tail down
   - Pitch down → nose down, tail up

#### Step 4: Motor Direction Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| Forward stick moves backward | Front motors reversed | Reverse MOT_X_DIRECTION for front thrusters |
| Left stick moves right | Lateral motors reversed | Reverse MOT_X_DIRECTION for lateral thrusters |
| Yaw right rotates left | Yaw motors reversed | Reverse both yaw motor directions |
| Throttle up sinks | Vertical motors reversed | Reverse all vertical MOT_X_DIRECTION |
| Vehicle rolls opposite | Vertical motors swapped | Check physical wiring to SERVO outputs |
| Unstable pitch during forward | Coupling issue | Adjust MOT_FV_CPLNG_K parameter |

## Troubleshooting

### Common Issues

#### Motors Don't Spin During Test

**Symptoms**: No motor response to motor test commands

**Possible Causes**:
1. Vehicle not armed
2. Hardware safety switch enabled
3. ESCs not calibrated
4. Incorrect PWM range

**Solutions**:
```
1. Arm vehicle: Set mode to Manual, arm via GCS
2. Disable safety switch: Press and hold until LED changes
3. Calibrate ESCs: Follow ESC manufacturer procedure
4. Verify parameters:
   - MOT_PWM_MIN = 1100
   - MOT_PWM_MAX = 1900
   - Match ESC calibration range
```

#### Wrong Motor Spins

**Symptoms**: Motor test activates different motor than expected

**Possible Causes**:
1. ESC connected to wrong servo output
2. Incorrect FRAME_CONFIG
3. Modified source code error

**Solutions**:
```
1. Verify physical wiring:
   - Motor 1 → SERVO1 output
   - Motor 2 → SERVO2 output
   - etc.

2. Confirm FRAME_CONFIG matches physical configuration

3. Review motor mixing code in AP_Motors6DOF.cpp
```

#### Vehicle Moves Opposite to Input

**Symptoms**: Forward stick moves backward, yaw right rotates left, etc.

**Cause**: Motor direction reversed

**Solution**:
```
1. Identify affected motors
2. Set MOT_X_DIRECTION = -1 for those motors
3. Retest

Example for Motor 2:
MOT_2_DIRECTION = -1
```

#### Unstable Pitch During Forward Motion

**Symptoms**: Vehicle pitches up or down when moving forward (vectored frames only)

**Cause**: Forward/vertical coupling not properly compensated

**Solution**:
```
1. Check FRAME_CONFIG = 1 (Vectored)
2. Adjust MOT_FV_CPLNG_K:
   - Pitches down: Increase value (try 1.2)
   - Pitches up: Decrease value (try 0.8)
   - Adjust in 0.1 increments
3. Test at medium speed
4. Fine-tune until stable
```

#### Weak Attitude Control

**Symptoms**: Slow roll/pitch response, difficulty maintaining attitude

**Possible Causes**:
1. Vertical thrusters underpowered
2. Thrusters too close to center
3. Center of gravity not centered
4. Incorrect motor factors

**Solutions**:
```
1. Verify all vertical thrusters operational
2. Check physical moment arms (>15cm from center recommended)
3. Balance vehicle mass distribution
4. Verify motor mixing factors in source code
5. Tune attitude PIDs (separate documentation)
```

#### One Motor Not Working

**Symptoms**: Vehicle drifts, unstable, weak control in one direction

**Diagnosis**:
```
1. Motor test each motor individually
2. Identify non-responsive motor
3. Check:
   - ESC connection and power
   - Thruster physically moves freely
   - Correct PWM signal (oscilloscope/logic analyzer)
   - ESC configuration matches others
```

**Solutions**:
- Swap ESC with known-good unit to isolate problem
- Check thruster for blockage or damage
- Verify servo output channel functioning
- Replace faulty component

#### Current Limiting Too Aggressive

**Symptoms**: Weak response during aggressive maneuvers, frequent power limiting

**Cause**: MOT_BAT_CURR_MAX set too low

**Solution**:
```
1. Monitor peak current during normal operation
2. If peaks stay well below MOT_BAT_CURR_MAX:
   - Increase parameter by 5-10A
3. If current exceeds battery safe limits:
   - Reduce aggressive maneuvers
   - Upgrade to higher capacity battery
   - Keep parameter at safe level
```

### Frame-Specific Troubleshooting

#### BlueROV1
- **Issue**: Lateral movement weak
  - **Cause**: Single lateral motor (Motor 6)
  - **Solution**: Expected behavior, design limitation

#### Vectored
- **Issue**: No pitch control
  - **Cause**: Design limitation, no pitch motors
  - **Solution**: Use Vectored_6DOF frame if pitch control needed

#### Vectored_6DOF / Vectored_6DOF_90
- **Issue**: Complex interactions between axes
  - **Cause**: Coupled motor mixing on 8-motor system
  - **Solution**: Requires careful tuning, especially with heavy payloads

#### SimpleROV-3/4/5
- **Issue**: Limited DOF restricts operations
  - **Cause**: Fewer thrusters by design
  - **Solution**: Plan missions within frame capabilities or upgrade frame

### Diagnostic Tools

#### MAVLink Inspector

Monitor motor outputs in real-time:
```
Messages to watch:
- SERVO_OUTPUT_RAW: PWM values to each motor
- BATTERY_STATUS: Current draw
- ATTITUDE: Roll/pitch/yaw response
```

#### Parameter Comparison

Compare your parameters with known-good configurations:
```bash
# Export current parameters
param save current_params.param

# Compare with reference
diff current_params.param reference_bluerov2_heavy.param
```

#### Logging and Analysis

Enable detailed logging:
```
LOG_BITMASK: Set bits for RCOUT, ATTITUDE, CTUN
```

Analyze logs for:
- Motor output saturation (hitting -1.0 or 1.0 limits)
- Unexpected coupling between axes
- Motor output balance (some motors working harder than others)

## Summary

ArduSub's flexible frame configuration system supports a wide range of ROV designs from simple 3-thruster platforms to complex 8-thruster 6DOF vehicles. Key points:

1. **Choose the correct FRAME_CONFIG** that matches your physical thruster layout
2. **Verify motor directions** using individual motor tests before water operations
3. **Tune coupling compensation** (MOT_FV_CPLNG_K) for vectored frames
4. **Always test systematically** from bench test → pool test → operational deployment
5. **Document custom configurations** for future reference and troubleshooting

For additional support:
- ArduPilot Documentation: https://ardupilot.org/ardusub/
- ArduPilot Forums: https://discuss.ardupilot.org/c/ardurov-ardusub
- Source Code: https://github.com/ArduPilot/ardupilot

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**Source Code Version**: ArduSub master branch  
**Related Documentation**: 
- [ArduSub README](../README.md)
- [Safety Procedures](SAFETY_PROCEDURES.md)
- [Navigation Algorithms](NAVIGATION_ALGORITHMS.md)
- [Depth Sensing](DEPTH_SENSING.md)

