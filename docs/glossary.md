# ArduPilot Glossary

## Purpose

This glossary defines ArduPilot-specific terminology, acronyms, and conventions used throughout the codebase and documentation. It is intended to ensure consistent usage across all documentation and to help new developers understand project-specific terms.

## Navigation and Estimation Terms

### AHRS (Attitude and Heading Reference System)
System that provides vehicle orientation (roll, pitch, yaw) by fusing data from inertial sensors (gyroscopes, accelerometers) and magnetometers. ArduPilot supports multiple AHRS backends including DCM and EKF-based implementations.

**Source**: libraries/AP_AHRS/

### DCM (Direction Cosine Matrix)
A 3×3 rotation matrix representation used to describe vehicle attitude and perform coordinate frame transformations. ArduPilot includes a DCM-based AHRS implementation as an alternative to EKF.

**Source**: libraries/AP_AHRS/AP_AHRS_DCM.cpp

### EKF (Extended Kalman Filter)
Advanced state estimation algorithm that fuses data from multiple sensors (IMU, GPS, barometer, compass, etc.) to estimate vehicle position, velocity, and attitude. ArduPilot implements two versions:

**Source**: libraries/AP_NavEKF2/, libraries/AP_NavEKF3/

### EKF2
Second-generation Extended Kalman Filter implementation in ArduPilot. Supports multiple IMUs, GPS, and advanced sensor fusion.

**Source**: libraries/AP_NavEKF2/

### EKF3
Third-generation Extended Kalman Filter implementation with improvements over EKF2, including better handling of visual odometry, wheel encoders, beacon positioning, and the GSF (Gaussian Sum Filter) yaw estimator for improved yaw estimation.

**Source**: libraries/AP_NavEKF3/

### GPS (Global Positioning System)
Satellite-based navigation system providing position, velocity, and timing information. ArduPilot supports multiple GPS protocols including UBLOX, NMEA, SBF, GSOF, and RTK (Real-Time Kinematic) for centimeter-level accuracy.

**Source**: libraries/AP_GPS/

### GSF (Gaussian Sum Filter)
Yaw estimation algorithm used within EKF3 to improve heading estimation, particularly during GPS aiding or when compass quality is poor.

**Source**: libraries/AP_NavEKF3/

### IMU (Inertial Measurement Unit)
Sensor package containing gyroscopes (measuring angular rates) and accelerometers (measuring linear acceleration). ArduPilot supports multiple IMU chips including MPU6000, ICM20xxx, BMI088, and others.

**Source**: libraries/AP_InertialSensor/

### Inertial Navigation
Position and velocity estimation by integrating accelerometer measurements over time, with corrections from GPS and other position sources.

**Source**: libraries/AP_InertialNav/

## Control and Guidance Terms

### AC_ Prefix
Naming convention for ArduCopter-specific control libraries. Examples include AC_AttitudeControl, AC_PosControl, AC_WPNav, AC_Fence, AC_PID.

**Source**: libraries/AC_*/

### APM_Control
Fixed-wing (ArduPlane) control library implementing roll, pitch, yaw, and steering controllers for planes.

**Source**: libraries/APM_Control/

### Attitude Control
Control of vehicle orientation (roll, pitch, yaw). Implemented in AC_AttitudeControl for multirotors and APM_Control for fixed-wing.

**Source**: libraries/AC_AttitudeControl/, libraries/APM_Control/

### L1 Controller
L1 adaptive navigation controller used for fixed-wing lateral path following and waypoint navigation. Provides smooth trajectory tracking with configurable period parameter.

**Source**: libraries/AP_L1_Control/

### PID (Proportional-Integral-Derivative Controller)
Fundamental control algorithm using proportional, integral, and derivative terms to minimize error between desired and actual states. Used throughout ArduPilot for stabilization and navigation.

**Source**: libraries/PID/, libraries/AC_PID/

### Position Control
Control of vehicle 3D position. Implemented in AC_PosControl for multirotors with velocity and position control modes.

**Source**: libraries/AC_PosControl/

### TECS (Total Energy Control System)
Energy management algorithm for fixed-wing aircraft that couples altitude and airspeed control by managing kinetic and potential energy. Used for climbs, descents, and maintaining speed during maneuvers.

**Source**: libraries/AP_TECS/

## Coordinate Frames and Reference Systems

### Body Frame
Vehicle-fixed coordinate system where:
- X-axis: Points forward (nose direction)
- Y-axis: Points right (starboard wing)
- Z-axis: Points down (belly direction)

Follows right-hand rule convention. Used for sensor measurements and control outputs.

### Earth Frame
Fixed coordinate system relative to the Earth's surface. In ArduPilot, typically refers to NED frame.

### ENU (East-North-Up)
Alternative earth-fixed coordinate frame where:
- X-axis: Points east
- Y-axis: Points north
- Z-axis: Points up

Less commonly used in ArduPilot; most code uses NED convention.

### NED (North-East-Down)
Primary earth-fixed coordinate frame used throughout ArduPilot where:
- X-axis: Points north
- Y-axis: Points east
- Z-axis: Points down (toward Earth center)

Follows right-hand rule. All position and velocity estimates from EKF are in NED frame.

### Quaternion
Four-dimensional representation of 3D rotation (w, x, y, z) that avoids gimbal lock. Used extensively in ArduPilot for attitude representation and interpolation.

**Source**: libraries/AP_Math/quaternion.cpp

## Communication Protocols

### CAN (Controller Area Network)
Robust serial bus protocol used for connecting peripherals like ESCs, GPS, compass, and airspeed sensors. ArduPilot supports DroneCAN/UAVCAN protocols over CAN.

**Source**: libraries/AP_CANManager/

### DroneCAN
CAN bus protocol (successor to UAVCAN v0) for connecting autopilot to smart peripherals. Supports GPS, compass, ESC telemetry, rangefinders, and other devices.

**Source**: libraries/AP_DroneCAN/

### FrSky Telemetry
Telemetry protocol for FrSky RC receivers, allowing vehicle data to be displayed on FrSky transmitters. Supports multiple variants including D, X, and passthrough protocols.

**Source**: libraries/AP_Frsky_Telem/

### GCS (Ground Control Station)
Software application (e.g., Mission Planner, QGroundControl, MAVProxy) that communicates with the autopilot via MAVLink for configuration, mission planning, and telemetry monitoring.

### MAVLink (Micro Air Vehicle Link)
Lightweight messaging protocol for communicating with ground control stations and onboard companion computers. Defines messages for telemetry, commands, mission upload/download, and parameter management.

**Source**: libraries/GCS_MAVLink/

### MSP (MultiWii Serial Protocol)
Protocol originally from MultiWii firmware, supported by ArduPilot primarily for OSD (On-Screen Display) integration with devices expecting MSP.

**Source**: libraries/AP_MSP/

### RC (Radio Control)
Manual control inputs from pilot's transmitter. ArduPilot supports numerous RC protocols including SBUS, PPM, DSM, CRSF, IBUS, and others.

**Source**: libraries/AP_RCProtocol/

### UAVCAN
CAN-based protocol for connecting peripherals (predecessor to DroneCAN). ArduPilot maintains compatibility with UAVCAN v0 devices through DroneCAN implementation.

## Hardware Abstraction Terms

### ChibiOS
Real-time operating system (RTOS) used on ARM-based flight controllers. Provides threading, synchronization primitives, and low-level hardware drivers for STM32 microcontrollers.

**Source**: modules/ChibiOS/, libraries/AP_HAL_ChibiOS/

### HAL (Hardware Abstraction Layer)
Interface layer that abstracts hardware-specific details, allowing ArduPilot code to run on different platforms (ARM, Linux, ESP32, etc.) without modification. Defines interfaces for UART, SPI, I2C, GPIO, timers, and other peripherals.

**Source**: libraries/AP_HAL/

### hwdef (Hardware Definition)
Board-specific configuration files (in .dat format) defining pin assignments, peripherals, memory layout, and features for each flight controller board. Used by the build system to generate board support code.

**Source**: libraries/AP_HAL_ChibiOS/hwdef/

### SITL (Software In The Loop)
Simulation mode where ArduPilot firmware runs on a desktop computer with simulated sensors and physics. Used for testing without hardware. Integrates with simulators like Gazebo, FlightGear, and JSBSim.

**Source**: libraries/AP_HAL_SITL/, libraries/SITL/

## Flight Modes

### ALT_HOLD (Altitude Hold)
Flight mode that maintains constant altitude using barometer and/or rangefinder. Pilot controls horizontal position manually.

**Vehicle**: Copter, Plane

### AUTO (Autonomous Mode)
Flight mode that executes pre-programmed mission waypoints from AP_Mission. Supports navigation commands, camera triggers, and conditional logic.

**Vehicle**: Copter, Plane, Rover, Sub

### GUIDED
Flight mode where vehicle follows position/velocity targets sent via MAVLink from GCS or companion computer. Used for dynamic path planning and obstacle avoidance.

**Vehicle**: Copter, Plane, Rover

### LAND
Automated landing sequence. For copters, descends vertically and disarms on touchdown. For planes, follows landing pattern.

**Vehicle**: Copter, Plane

### LOITER
Position hold mode using GPS. Vehicle maintains 3D position (altitude and horizontal position). Pilot can override with stick inputs.

**Vehicle**: Copter, Plane

### MANUAL
Direct pilot control with no stabilization assistance. Pilot controls servos/motors directly.

**Vehicle**: Plane, Rover

### RTL (Return To Launch)
Automated return to launch location. Copters climb to RTL_ALT, fly to home, and land. Planes may use rally points or landing patterns.

**Vehicle**: Copter, Plane, Rover, Sub

### SMART_RTL
Return to launch by retracing the vehicle's flight path, avoiding obstacles encountered during outbound flight. Falls back to RTL if path not available.

**Vehicle**: Copter, Rover

### STABILIZE
Basic stabilization mode. Pilot inputs control desired roll/pitch angles (copter) or roll/pitch rates (plane). Altitude and position not controlled.

**Vehicle**: Copter, Plane

## Safety Systems

### Advanced Failsafe
Enhanced failsafe system providing additional layers of protection beyond basic failsafes, including hardware termination pins for gas engines or parachute deployment.

**Source**: libraries/AP_AdvancedFailsafe/

### Arming
Safety mechanism requiring explicit pilot action before motors can spin. Includes pre-arm checks (system health validation) and arming checks (pilot-initiated safety confirmation).

**Source**: libraries/AP_Arming/

### EKF Failsafe
Triggered when navigation solution quality degrades beyond acceptable thresholds. Actions include switching to stabilize mode, RTL, or land depending on configuration.

### Failsafe
Emergency procedures triggered by loss of RC signal, low battery, GPS failure, EKF failure, geofence breach, or other critical conditions. Configurable actions include RTL, LAND, or user-defined behavior.

**Source**: Vehicle-specific failsafe.cpp files

### Fence (Geofence)
Virtual boundaries that trigger actions when breached. Supports altitude limits, cylindrical boundaries, and complex polygons with inclusion/exclusion zones.

**Source**: libraries/AC_Fence/

### Pre-Arm Checks
Automated safety checks performed before arming is allowed. Validates sensor health, calibration status, GPS lock, battery voltage, and other critical parameters.

**Source**: libraries/AP_Arming/

### Rally Points
Alternate landing locations that can be used instead of home position during RTL. Useful for avoiding obstacles or selecting better landing sites.

**Source**: libraries/AP_Rally/

### Throttle Failsafe
Triggered when RC receiver outputs a failsafe signal (typically low throttle and centered other channels) indicating loss of transmitter signal.

## Measurement Units and Conventions

### Centidegrees (cdeg)
Angle measurement in hundredths of a degree (angle × 100). Used extensively in ArduPilot for roll, pitch, yaw angles to avoid floating-point arithmetic in time-critical code. Range: -18000 to 18000 for ±180°.

**Example**: 4500 centidegrees = 45.0 degrees

### Centimeters (cm)
Distance measurement used in some altitude and position calculations. EKF outputs are typically in meters, but some legacy code uses centimeters.

### Degrees per Second (deg/s)
Angular rate measurement for gyroscopes and rate controllers. Standard unit for rotation rates throughout ArduPilot.

### Meters (m)
Standard SI unit for distance. Used for EKF position estimates, waypoint coordinates (relative), and altitude measurements.

### Meters per Second (m/s)
Velocity measurement. EKF velocity estimates are in m/s in NED frame. Some legacy code may use cm/s.

### Microseconds (μs or us)
Time measurement used for PWM pulse widths to servos/ESCs. Typical range: 1000-2000 μs for standard RC PWM signals.

### Milliseconds (ms)
Time measurement used for scheduler timing, delays, and timeouts. Accessed via AP_HAL::millis() or AP_HAL::millis64().

### Radians (rad)
Angle measurement in radians (0 to 2π). Used in mathematical calculations and control algorithms. Often converted from degrees or centidegrees for computation.

## Sensor Types

### Airspeed Sensor
Differential pressure sensor measuring dynamic pressure for true airspeed calculation. Critical for fixed-wing flight. Supported types include MS4525, MS5525, DLVR, and analog sensors.

**Source**: libraries/AP_Airspeed/

### Baro (Barometer)
Pressure sensor used for altitude estimation. Measures atmospheric pressure to calculate height above sea level. Common chips: MS5611, BMP280, LPS25H.

**Source**: libraries/AP_Baro/

### Compass (Magnetometer)
Sensor measuring Earth's magnetic field to determine heading. Requires calibration for hard and soft iron distortion. Common chips: HMC5843, LSM303D, QMC5883L, IST8310.

**Source**: libraries/AP_Compass/

### Lidar
Light Detection and Ranging sensor (also called laser rangefinder). Measures distance to ground or obstacles using laser pulses. Used for terrain following and precision landing.

**Source**: libraries/AP_RangeFinder/

### OpticalFlow
Camera-based sensor measuring ground velocity by tracking visual features. Used for position hold without GPS. Common devices: PX4Flow, Cheerson CX-OF.

**Source**: libraries/AP_OpticalFlow/

### RangeFinder (Distance Sensor)
Generic term for distance-measuring sensors including ultrasonic, lidar, and radar. Used for terrain following, precision landing, and obstacle detection.

**Source**: libraries/AP_RangeFinder/

### RPM Sensor
Measures engine or rotor rotation speed. Used for helicopter rotor speed governors and internal combustion engine monitoring.

**Source**: libraries/AP_RPM/

## Build System and Development Terms

### Board
Specific flight controller hardware target (e.g., Pixhawk, CubeOrange, MatekH743). Each board has a hwdef file defining its hardware configuration.

### Feature Flags
Compile-time configuration options (e.g., HAL_*, AP_*_ENABLED) that enable or disable features. Used to fit firmware into memory-constrained boards.

**Source**: hwdef files, libraries/AP_HAL/AP_HAL_Boards.h

### Submodule
Git submodule containing external dependencies like ChibiOS, MAVLink definitions, DroneCAN libraries. Located in modules/ directory.

### waf
Python-based build system used by ArduPilot. Configured via wscript files and Tools/ardupilotwaf/ modules.

**Source**: wscript, Tools/ardupilotwaf/

## Additional Common Terms

### Bootloader
Small program in protected flash memory that enables firmware updates via USB or serial. Supports protocols like DFU (Device Firmware Update) on USB-equipped boards.

**Source**: Tools/AP_Bootloader/

### DAL (Data Abstraction Layer)
Abstraction layer used by log replay system to allow EKF and other algorithms to run on logged data for debugging and development.

**Source**: libraries/AP_DAL/

### ESC (Electronic Speed Controller)
Device that controls motor speed based on PWM, DShot, or other protocol inputs from autopilot. May provide telemetry (RPM, voltage, current, temperature).

### Lua Scripting
Embedded Lua interpreter allowing users to write custom behaviors without modifying C++ code. Scripts can interact with vehicle via AP bindings.

**Source**: libraries/AP_Scripting/

### OSD (On-Screen Display)
Overlay of flight data (altitude, speed, battery, etc.) on FPV video feed. Supports analog OSD chips (MAX7456) and digital protocols (MSP, MAVLink).

**Source**: libraries/AP_OSD/

### Parameter
User-configurable value stored in EEPROM/flash that modifies vehicle behavior. Accessed via GCS or MAVLink. Examples: PID gains, sensor offsets, mode configurations.

**Source**: libraries/AP_Param/

### PWM (Pulse Width Modulation)
Signal encoding method where information is conveyed by pulse duration. Standard RC and servo control uses 1000-2000 μs pulses at 50 Hz.

### Scheduler
Task management system that runs periodic and priority-based tasks. Configured via scheduler_tasks[] arrays in vehicle code. Monitors timing and CPU load.

**Source**: libraries/AP_Scheduler/, vehicle Copter.cpp/Plane.cpp

### Semaphore
Synchronization primitive preventing concurrent access to shared resources. Used extensively with WITH_SEMAPHORE macro for thread-safe operations.

**Source**: libraries/AP_HAL/Semaphores.h

### Singleton
Design pattern where class has single global instance accessed via static methods (e.g., AP::ahrs(), AP::gps()). Used extensively throughout ArduPilot.

### WITH_SEMAPHORE
Macro providing RAII-style semaphore locking. Automatically acquires semaphore at scope entry and releases at scope exit, preventing deadlocks.

**Example**: `WITH_SEMAPHORE(hal.i2c_mgr->get_semaphore());`

## Usage Notes

- **Consistency**: Always use these standard terms in documentation and code comments
- **Cross-References**: When using acronyms, reference this glossary on first use in documentation
- **Updates**: This glossary should be updated when new major subsystems or terminology are introduced
- **Units**: Always specify units explicitly; never assume readers know implicit units

## Related Documentation

- Coordinate frame transformations: docs/coordinate-frames.md
- Unit conversion guidelines: docs/units.md
- Module documentation: See README.md files in libraries/ directories
- Full API documentation: Generated Doxygen HTML documentation

---

**Last Updated**: 2025  
**Maintainers**: ArduPilot Development Team
