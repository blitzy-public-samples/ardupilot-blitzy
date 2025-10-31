# ArduPilot Unit Conventions

## Overview

This document defines the standard units used throughout the ArduPilot codebase. Consistent unit usage is critical for:
- Preventing calculation errors and unit conversion bugs
- Ensuring safety in flight-critical computations
- Maintaining code clarity and developer understanding
- Facilitating integration between modules

**Key Principle**: Always be explicit about units in variable names, comments, and documentation. Never assume units from context alone.

## Unit Categories

### Length Units

ArduPilot uses different length units depending on the context and precision requirements:

#### Meters (m)
**Usage Context**:
- High-level navigation calculations
- Waypoint distances and mission planning
- GPS coordinate conversions to local coordinates
- Altitude references (relative to home, terrain, or sea level)
- Sensor ranges and detection distances
- Parameter values exposed to users

**Examples**:
- `wp_distance_m` - Distance to waypoint in meters
- `alt_target_m` - Target altitude in meters
- `rangefinder_distance_m` - Distance measurement in meters

#### Centimeters (cm)
**Usage Context**:
- Internal position control calculations requiring higher precision
- EKF (Extended Kalman Filter) state estimates
- Position controller intermediate values
- Small distance measurements where meter precision is insufficient

**Examples**:
- `pos_target_cm` - Position target in centimeters (Vector3f)
- `inertial_nav_position_cm` - Estimated position in centimeters
- `pos_error_cm` - Position error in centimeters

**Conversion**:
```
1 meter = 100 centimeters
1 cm = 0.01 m
```

**Rationale**: Using centimeters internally provides sub-meter precision using integer arithmetic (when stored as int32_t), avoiding floating-point precision issues while maintaining sufficient accuracy for position control.

---

### Angle Units

ArduPilot employs three different angle representations, each optimized for specific use cases:

#### Degrees (deg or °)
**Usage Context**:
- Human-readable parameter values
- Ground control station display
- User-facing documentation
- High-level mission planning (e.g., heading waypoints)
- Parameter defaults and configuration

**Examples**:
- `COMPASS_DEC` parameter - Magnetic declination in degrees
- `heading_deg` - Vehicle heading in degrees (0-360)
- `roll_limit_deg` - Maximum roll angle limit in degrees

#### Radians (rad)
**Usage Context**:
- Mathematical calculations and trigonometric functions
- Rotation matrices and quaternion conversions
- Control algorithm implementations
- EKF prediction and update equations
- Any computation involving sin(), cos(), tan(), atan2()

**Examples**:
- `roll_rad` - Roll angle in radians for trigonometric calculations
- `yaw_rate_rad` - Yaw rate in radians per second
- `angle_diff_rad` - Angular difference in radians

**Conversion**:
```
180 degrees = π radians
1 degree = π/180 radians ≈ 0.0174533 rad
1 radian = 180/π degrees ≈ 57.2958 deg
```

**Rationale**: Radians are the natural unit for mathematical operations. Standard math library functions (sin, cos, etc.) expect radians as input.

#### Centidegrees (cdeg)
**Usage Context**:
- Internal attitude storage for precision
- Pilot input scaling (RC stick positions often represented as ±4500 cdeg = ±45°)
- Attitude targets with sub-degree precision
- Lean angle calculations in multicopter control

**Examples**:
- `pilot_roll_cd` - Pilot roll input in centidegrees (-4500 to 4500)
- `angle_target_cd` - Target angle in centidegrees
- `lean_angle_max_cd` - Maximum lean angle in centidegrees

**Conversion**:
```
1 degree = 100 centidegrees
1 cdeg = 0.01 deg
1 radian ≈ 5729.58 cdeg
```

**Rationale**: Centidegrees provide sub-degree precision using integer arithmetic, useful for RC input processing and attitude control where 0.01° resolution is sufficient.

---

### Time Units

Time representation varies by precision requirements and context:

#### Seconds (s)
**Usage Context**:
- High-level timing (mission waypoint delays, timeouts)
- User-facing time parameters
- Long-duration measurements
- Rate conversions (Hz = 1/s)

**Examples**:
- `loiter_time_max_s` - Maximum loiter time in seconds
- `failsafe_timeout_s` - Failsafe trigger timeout in seconds
- `elapsed_time_s` - Elapsed time since event in seconds

#### Milliseconds (ms)
**Usage Context**:
- Scheduler task timing and loop rates
- System uptime and timestamps
- Moderate-precision timeouts and delays
- Sensor update intervals
- Most internal timing and time-since calculations

**Examples**:
- `last_update_ms` - Last update timestamp in milliseconds (from AP_HAL::millis())
- `timeout_ms` - Timeout duration in milliseconds
- `loop_time_ms` - Loop execution time in milliseconds

**Conversion**:
```
1 second = 1,000 milliseconds
1 ms = 0.001 s
```

**Rationale**: Milliseconds provide good resolution for most real-time control timing without overflow concerns. AP_HAL::millis() returns uint32_t milliseconds since boot (overflows after ~49.7 days).

#### Microseconds (μs or us)
**Usage Context**:
- PWM (Pulse Width Modulation) pulse widths for servos and ESCs
- High-precision timing measurements
- Interrupt timing and latency measurements
- RC input pulse widths

**Examples**:
- `pwm_output_us` - PWM output in microseconds (typically 1000-2000)
- `pulse_width_us` - RC receiver pulse width in microseconds
- `execution_time_us` - Function execution time in microseconds (from AP_HAL::micros64())

**Conversion**:
```
1 millisecond = 1,000 microseconds
1 second = 1,000,000 microseconds
1 μs = 0.001 ms = 0.000001 s
```

**Typical PWM Range**: 1000-2000 μs (with 1500 μs typically representing neutral/center)

**Rationale**: Microseconds are the standard unit for RC and servo protocols. PWM timing requires microsecond precision.

---

### Velocity Units

#### Meters per Second (m/s)
**Usage Context**:
- Navigation speed targets and measurements
- Ground speed and airspeed
- User-facing speed parameters
- Waypoint navigation speeds
- Wind speed estimates

**Examples**:
- `target_speed_ms` - Target velocity in meters per second
- `ground_speed_ms` - Ground speed in m/s
- `wind_speed_ms` - Estimated wind speed in m/s

#### Centimeters per Second (cm/s)
**Usage Context**:
- Internal position controller velocity calculations
- High-precision velocity control
- EKF velocity state estimates
- Velocity error calculations in position control loops

**Examples**:
- `vel_target_cms` - Velocity target in cm/s (Vector3f)
- `vel_error_cms` - Velocity error in cm/s
- `accel_to_vel_cms` - Acceleration integrated to velocity in cm/s

**Conversion**:
```
1 m/s = 100 cm/s
1 cm/s = 0.01 m/s
```

**Rationale**: Similar to position units, using cm/s internally provides sub-m/s precision for precise velocity control while avoiding floating-point precision issues.

---

### Angular Velocity (Rotation Rates)

#### Degrees per Second (deg/s or °/s)
**Usage Context**:
- User-facing rate parameters (maximum yaw rate, pitch rate limits)
- Pilot input rates from RC transmitter
- Rate mode target rates
- Display and logging of rotation rates

**Examples**:
- `yaw_rate_max_degs` - Maximum yaw rate in degrees per second
- `pilot_yaw_rate_degs` - Pilot commanded yaw rate in deg/s
- `gyro_drift_degs` - Gyro drift rate in deg/s

#### Radians per Second (rad/s)
**Usage Context**:
- Internal rate controller calculations
- Gyroscope measurements (raw and filtered)
- Angular velocity state estimates
- Rate control loops and PID controllers
- Quaternion derivative calculations

**Examples**:
- `gyro_rate_rads` - Gyroscope rate in radians per second (Vector3f)
- `rate_target_rads` - Target angular rate in rad/s
- `rate_error_rads` - Rate tracking error in rad/s

**Conversion**:
```
1 deg/s = π/180 rad/s ≈ 0.0174533 rad/s
1 rad/s = 180/π deg/s ≈ 57.2958 deg/s
```

**Rationale**: Radians per second are natural for rate controller mathematics, while degrees per second are more intuitive for users and parameters.

---

### Acceleration Units

#### Meters per Second Squared (m/s²)
**Usage Context**:
- Accelerometer measurements (after calibration and scaling)
- Acceleration limits and constraints
- Gravity representation (9.80665 m/s²)
- Navigation acceleration estimates

**Examples**:
- `accel_mss` - Acceleration in m/s² (Vector3f)
- `accel_limit_mss` - Acceleration limit in m/s²
- `GRAVITY_MSS` - Standard gravity constant (9.80665 m/s²)

#### Centimeters per Second Squared (cm/s²)
**Usage Context**:
- Internal position controller acceleration calculations
- High-precision acceleration control
- Jerk limiting and trajectory generation

**Examples**:
- `accel_target_cmss` - Target acceleration in cm/s²
- `accel_max_cmss` - Maximum acceleration in cm/s²

**Conversion**:
```
1 m/s² = 100 cm/s²
1 cm/s² = 0.01 m/s²
```

---

### Pressure Units

#### Pascals (Pa)
**Usage Context**:
- SI standard unit for pressure
- Barometer sensor readings (after conversion)
- Pressure altitude calculations

**Examples**:
- `pressure_pa` - Atmospheric pressure in Pascals
- `pressure_diff_pa` - Differential pressure in Pascals (airspeed sensors)

#### Millibars (mbar) or Hectopascals (hPa)
**Usage Context**:
- Alternative pressure representation (commonly used in aviation)
- Display and logging
- Parameter values for pressure settings

**Examples**:
- `pressure_mbar` - Atmospheric pressure in millibars
- `ground_pressure_mbar` - Ground reference pressure in mbar

**Conversion**:
```
1 millibar = 100 Pascals
1 mbar = 1 hPa (hectopascal)
1 Pa = 0.01 mbar
Standard atmosphere = 1013.25 mbar = 101325 Pa
```

---

### PWM (Pulse Width Modulation) Units

#### Microseconds (μs)
**Usage Context**:
- Servo output pulse widths
- ESC (Electronic Speed Controller) control signals
- RC receiver input pulse widths
- Standard servo range: 1000-2000 μs

**Examples**:
- `servo_pwm_us` - Servo PWM output in microseconds
- `rc_input_us` - RC channel input in microseconds
- `pwm_min_us` - Minimum PWM value (typically 1000 μs)
- `pwm_max_us` - Maximum PWM value (typically 2000 μs)

**Standard Ranges**:
- **Servos**: 1000-2000 μs (1500 μs = center/neutral)
- **ESCs**: 1000-2000 μs (1000 μs = zero throttle, 2000 μs = full throttle)
- **Some servos**: Extended ranges like 800-2200 μs

**Rationale**: PWM timing in microseconds is the universal standard for RC servo control, defined by servo and ESC manufacturers.

---

### Battery and Electrical Units

#### Voltage
- **Volts (V)**: Battery voltage, cell voltage
- **Millivolts (mV)**: High-precision voltage measurements

#### Current
- **Amperes (A)**: Battery current draw
- **Milliamperes (mA)**: Low-current measurements

#### Capacity
- **Milliampere-hours (mAh)**: Battery capacity
- **Ampere-hours (Ah)**: Large battery capacity

**Examples**:
- `battery_voltage_v` - Battery voltage in volts
- `battery_current_a` - Battery current in amperes
- `battery_capacity_mah` - Battery capacity in milliampere-hours
- `consumed_mah` - Energy consumed in mAh

---

### Frequency Units

#### Hertz (Hz)
**Usage Context**:
- Loop rates and update frequencies
- Sensor sampling rates
- Control loop frequencies
- Filter cutoff frequencies

**Examples**:
- `main_loop_rate_hz` - Main loop rate in Hz (e.g., 400 Hz for Copter)
- `imu_sample_rate_hz` - IMU sampling rate in Hz
- `notch_freq_hz` - Notch filter center frequency in Hz

**Common Rates**:
- **ArduCopter main loop**: 400 Hz (2.5 ms period)
- **ArduPlane main loop**: 50-400 Hz (vehicle dependent)
- **IMU sampling**: 1000-8000 Hz (sensor dependent)
- **EKF updates**: 50-400 Hz

---

## Context-Specific Unit Conventions

### In Parameters (AP_Param)

**Convention**: Use human-friendly units that are intuitive to configure
- **Distances**: Meters (not centimeters)
- **Angles**: Degrees (not radians or centidegrees)
- **Speeds**: m/s or cm/s depending on precision needs
- **Rates**: Degrees per second
- **Time**: Seconds for long durations, milliseconds for short timeouts

**Parameter Naming**: Include unit suffix for clarity
- `WPNAV_SPEED` - Speed in cm/s (explicitly documented)
- `ANGLE_MAX` - Maximum angle in centidegrees
- `FS_TIMEOUT` - Failsafe timeout in seconds

**Rationale**: Parameters are set by end users who expect intuitive, human-readable units.

### In Internal Calculations

**Convention**: Use units optimized for precision and computational efficiency
- **Positions**: Centimeters (integer-friendly precision)
- **Attitudes**: Radians (natural for trigonometry)
- **Velocities**: cm/s (matches position precision)
- **Rates**: rad/s (natural for rate controllers)
- **Time**: Milliseconds (from AP_HAL::millis())

**Rationale**: Internal calculations prioritize numerical stability, computational efficiency, and mathematical naturalness.

### In MAVLink Messages

**Convention**: Units defined by MAVLink protocol specification
- **Follow MAVLink XML definitions strictly**
- **Common units**: SI base units (meters, radians, m/s, rad/s)
- **GPS coordinates**: Degrees × 10^7 (int32_t latitude/longitude)
- **Altitude**: Millimeters or meters (message dependent)

**Example**:
```cpp
// MAVLink GLOBAL_POSITION_INT message
// lat/lon in degrees × 10^7 (int32_t)
// alt in millimeters above MSL (int32_t)
// relative_alt in millimeters above home (int32_t)
// vx, vy, vz in cm/s (int16_t)
```

**Rationale**: MAVLink compatibility requires strict adherence to protocol-defined units for interoperability with ground stations and companion computers.

### In Log Files (DataFlash)

**Convention**: Balance human readability with storage efficiency
- **High-precision values**: Use scaled integers (e.g., cm, cdeg, cm/s)
- **User-facing values**: Use intuitive units (deg, m/s)
- **Timestamps**: Milliseconds or microseconds since boot
- **Attitude**: Typically centidegrees or degrees
- **Position**: Typically centimeters or meters

**Example Log Message Units**:
- `CTUN.Alt` - Altitude in centimeters
- `RATE.RDes` - Desired roll rate in centidegrees per second
- `POS.Lat` - Latitude in degrees × 10^7

**Rationale**: Logs balance precision with file size and post-flight analysis usability.

### In Sensor Drivers

**Convention**: Convert sensor raw values to standard ArduPilot units early
- **IMU accelerometers**: Convert to m/s²
- **IMU gyroscopes**: Convert to rad/s
- **Barometers**: Convert to Pascals
- **Magnetometers**: Convert to milliGauss or Tesla
- **GPS**: Convert to standard representations (meters, m/s, degrees)

**Rationale**: Standardizing units at the sensor driver boundary ensures all consuming code uses consistent units.

---

## Unit Conversion Helper Functions

ArduPilot provides helper functions and macros for common conversions:

### Angle Conversions
```cpp
// In AP_Math/definitions.h
#define DEG_TO_RAD   0.017453292519943295769236907684886f
#define RAD_TO_DEG   57.295779513082320876798154814105f

radians(degrees)    // Degrees to radians
degrees(radians)    // Radians to degrees
```

### Common Macros
```cpp
// Centidegree conversions
degrees = centidegrees / 100.0f
centidegrees = degrees * 100.0f

// Position conversions (cm <-> m)
meters = centimeters / 100.0f
centimeters = meters * 100.0f

// Time conversions
seconds = milliseconds / 1000.0f
milliseconds = microseconds / 1000
```

### Vector Unit Conversions
When converting vector quantities (position, velocity, acceleration), apply conversion to all components:
```cpp
// Example: Convert position from meters to centimeters
Vector3f pos_m(x_m, y_m, z_m);
Vector3f pos_cm = pos_m * 100.0f;

// Example: Convert velocity from cm/s to m/s
Vector3f vel_cms(vx_cms, vy_cms, vz_cms);
Vector3f vel_ms = vel_cms / 100.0f;
```

---

## Best Practices

### 1. Always Be Explicit
**DO**: Use descriptive variable names with unit suffixes
```cpp
float altitude_m = 100.0f;           // Clear: altitude in meters
float target_speed_ms = 5.0f;        // Clear: speed in m/s
uint16_t servo_pwm_us = 1500;        // Clear: PWM in microseconds
```

**DON'T**: Use ambiguous variable names
```cpp
float altitude = 100.0f;             // Unclear: meters or centimeters?
float speed = 5.0f;                  // Unclear: m/s or cm/s?
uint16_t servo = 1500;               // Unclear: PWM? Position? Angle?
```

### 2. Document Units in Comments
When units aren't in the variable name, document them clearly:
```cpp
/**
 * @brief Calculate distance to waypoint
 * @param wp_lat Waypoint latitude in degrees × 10^7
 * @param wp_lon Waypoint longitude in degrees × 10^7
 * @return Distance to waypoint in meters
 */
float get_distance_to_waypoint(int32_t wp_lat, int32_t wp_lon);
```

### 3. Convert at API Boundaries
Convert units at interface boundaries, not throughout internal code:
```cpp
// GOOD: Convert once at the boundary
void set_target_altitude(float alt_m) {
    // Convert to internal representation immediately
    _target_alt_cm = alt_m * 100.0f;
    // All internal code uses cm consistently
}

// BAD: Converting repeatedly in internal code
void update() {
    float alt_m = _target_alt_cm / 100.0f;  // Convert
    float error_m = (wp_alt_cm / 100.0f) - alt_m;  // Convert again
    float correction_cm = error_m * 100.0f * gain;  // Convert back
}
```

### 4. Use Constants for Standard Values
```cpp
// Define standard constants with units
#define GRAVITY_MSS 9.80665f              // Standard gravity in m/s²
#define SERVO_CENTER_US 1500              // Standard servo center in μs
#define SERVO_MIN_US 1000                 // Standard servo min in μs
#define SERVO_MAX_US 2000                 // Standard servo max in μs
```

### 5. Be Careful with Integer Conversions
```cpp
// WRONG: Integer division loses precision
int32_t alt_cm = 150;  // 1.5 meters
int32_t alt_m = alt_cm / 100;  // Result: 1 meter (lost 0.5m!)

// CORRECT: Convert to float first
int32_t alt_cm = 150;
float alt_m = alt_cm / 100.0f;  // Result: 1.5 meters
```

### 6. Watch for Overflow in Conversions
```cpp
// DANGEROUS: May overflow if value is large
int16_t angle_deg = 180;
int32_t angle_cdeg = angle_deg * 100;  // Risk of overflow with int16_t

// SAFER: Use appropriate data types
int32_t angle_cdeg = (int32_t)angle_deg * 100;  // Cast to prevent overflow
```

---

## Unit Testing Considerations

When writing tests, verify unit conversions explicitly:

```cpp
// Test unit conversion accuracy
TEST(UnitsTest, DegreeToRadianConversion) {
    float deg_90 = 90.0f;
    float rad_90 = radians(deg_90);
    EXPECT_FLOAT_EQ(rad_90, M_PI / 2.0f);
    
    float deg_back = degrees(rad_90);
    EXPECT_FLOAT_EQ(deg_back, deg_90);
}

TEST(UnitsTest, MeterToCentimeterConversion) {
    float meters = 1.5f;
    float cm = meters * 100.0f;
    EXPECT_FLOAT_EQ(cm, 150.0f);
}
```

---

## Common Pitfalls

### Pitfall 1: Mixing Units in Calculations
```cpp
// WRONG: Mixing meters and centimeters
float distance_m = 10.0f;
float height_cm = 500.0f;
float hypotenuse = sqrtf(distance_m * distance_m + height_cm * height_cm);  // WRONG!

// CORRECT: Use consistent units
float distance_cm = 1000.0f;
float height_cm = 500.0f;
float hypotenuse_cm = sqrtf(distance_cm * distance_cm + height_cm * height_cm);
```

### Pitfall 2: Forgetting Time Unit in Rate Calculations
```cpp
// WRONG: Inconsistent time units
float speed_ms = 5.0f;         // m/s
float time_ms = 1000.0f;        // milliseconds
float distance = speed_ms * time_ms;  // WRONG: gives 5000 instead of 5 meters!

// CORRECT: Convert to consistent time unit
float time_s = time_ms / 1000.0f;
float distance_m = speed_ms * time_s;  // Correct: 5 meters
```

### Pitfall 3: Assuming PWM Units
```cpp
// WRONG: Assuming arbitrary PWM range
uint16_t pwm = rc_input * 20;  // What units? What range?

// CORRECT: Explicitly scale to microseconds
uint16_t pwm_us = constrain_int16(
    1000 + (rc_input * 1000 / 100),  // Scale 0-100 to 1000-2000 μs
    1000, 2000
);
```

### Pitfall 4: GPS Coordinate Units
```cpp
// WRONG: Treating GPS coordinates as plain degrees
int32_t lat = 473928320;  // Actually degrees × 10^7
float lat_deg = lat;  // WRONG! This is 473928320 degrees!

// CORRECT: Scale GPS coordinates properly
int32_t lat_1e7 = 473928320;  // 47.3928320 degrees
float lat_deg = lat_1e7 / 1.0e7f;  // Correct: 47.3928320 degrees
```

---

## Quick Reference Table

| Quantity | Primary Unit | Alternative Unit | Conversion | Context |
|----------|--------------|------------------|------------|---------|
| **Distance** | meters (m) | centimeters (cm) | 1 m = 100 cm | m: navigation, cm: internal |
| **Altitude** | meters (m) | centimeters (cm) | 1 m = 100 cm | m: parameters, cm: internal |
| **Angle** | degrees (°) | radians (rad) | 180° = π rad | °: parameters, rad: math |
| **Angle** | degrees (°) | centidegrees (cdeg) | 1° = 100 cdeg | °: display, cdeg: internal |
| **Time (long)** | seconds (s) | milliseconds (ms) | 1 s = 1000 ms | s: delays, ms: timestamps |
| **Time (short)** | milliseconds (ms) | microseconds (μs) | 1 ms = 1000 μs | ms: timing, μs: PWM |
| **Speed** | m/s | cm/s | 1 m/s = 100 cm/s | m/s: navigation, cm/s: internal |
| **Angular rate** | deg/s | rad/s | 1 deg/s = π/180 rad/s | deg/s: parameters, rad/s: control |
| **Acceleration** | m/s² | cm/s² | 1 m/s² = 100 cm/s² | m/s²: sensors, cm/s²: internal |
| **Pressure** | Pascals (Pa) | millibars (mbar) | 1 mbar = 100 Pa | Both common |
| **PWM** | microseconds (μs) | - | 1000-2000 μs typical | Standard servo range |
| **Voltage** | volts (V) | millivolts (mV) | 1 V = 1000 mV | V: typical usage |
| **Current** | amperes (A) | milliamperes (mA) | 1 A = 1000 mA | A: typical usage |
| **Capacity** | mAh | Ah | 1 Ah = 1000 mAh | mAh: typical battery size |
| **Frequency** | Hertz (Hz) | - | cycles per second | Loop and sample rates |
| **GPS Lat/Lon** | degrees × 10^7 | degrees | divide by 10^7 | MAVLink/internal format |

---

## Related Documentation

- **Coordinate Frames**: See [docs/coordinate-frames.md](coordinate-frames.md) for coordinate system conventions
- **Glossary**: See [docs/glossary.md](glossary.md) for ArduPilot-specific terminology
- **AP_Math Library**: See [libraries/AP_Math/README.md](../libraries/AP_Math/README.md) for mathematical utilities and conversion functions

---

## Revision History

- **2025-01**: Initial comprehensive unit conventions documentation

---

**Last Updated**: 2025-01

**Maintainer**: ArduPilot Documentation Team


