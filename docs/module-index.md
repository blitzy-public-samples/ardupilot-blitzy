# ArduPilot Module Index

This index provides a comprehensive overview of all ArduPilot vehicle implementations, libraries, and tools. Use this as a quick reference to locate specific modules and their documentation.

## Vehicle Implementations

Core flight controller implementations for different vehicle types.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **ArduCopter/** | Multicopter flight controller supporting quad, hex, octo, and other multirotor configurations. Includes 30+ flight modes, advanced position control, and mission execution. | [ArduCopter/README.md](../ArduCopter/README.md) |
| **ArduPlane/** | Fixed-wing aircraft flight controller with TECS (Total Energy Control System) for altitude/airspeed management and L1 navigation for waypoint tracking. Supports conventional planes and quadplanes. | [ArduPlane/README.md](../ArduPlane/README.md) |
| **Rover/** | Ground vehicle controller supporting Ackermann steering (car-like) and skid steering (tank-like). Includes waypoint navigation, pivot turns, and guided mode for ground operations. | [Rover/README.md](../Rover/README.md) |
| **ArduSub/** | Underwater vehicle controller with depth hold algorithms, buoyancy compensation, and joystick control mapping. Supports ROVs and AUVs with specialized failsafe logic for underwater operations. | [ArduSub/README.md](../ArduSub/README.md) |
| **Blimp/** | Lighter-than-air vehicle controller for airships with specialized fin control, wind compensation, and buoyancy-optimized altitude hold. | [Blimp/README.md](../Blimp/README.md) |
| **AntennaTracker/** | Antenna pointing system for ground stations. Tracks vehicle position and orients directional antennas for optimal telemetry range. | [AntennaTracker/README.md](../AntennaTracker/README.md) |

## Hardware Abstraction Layer

Platform-independent interfaces and platform-specific implementations for hardware access.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_HAL/** | Core HAL interface definitions providing platform-independent abstractions for UART, SPI, I2C, GPIO, timers, and scheduler. All platform implementations must implement these interfaces. | [libraries/AP_HAL/README.md](../libraries/AP_HAL/README.md) |
| **AP_HAL_ChibiOS/** | ARM Cortex-M platform implementation using ChibiOS RTOS. Supports 150+ flight controller boards with hardware definitions in hwdef files. Primary platform for most flight controllers. | [libraries/AP_HAL_ChibiOS/README.md](../libraries/AP_HAL_ChibiOS/README.md) |
| **AP_HAL_Linux/** | Linux platform support for boards like Navio, BeagleBone Blue, and generic Linux systems. Uses sysfs for device access and Linux scheduling. | [libraries/AP_HAL_Linux/README.md](../libraries/AP_HAL_Linux/README.md) |
| **AP_HAL_ESP32/** | ESP32 microcontroller platform support with WiFi/Bluetooth integration and ESP-IDF framework. Enables IoT and telemetry applications. | [libraries/AP_HAL_ESP32/README.md](../libraries/AP_HAL_ESP32/README.md) |
| **AP_HAL_SITL/** | Software-in-the-loop simulation platform for testing without physical hardware. Simulates sensors, GPS, and vehicle physics for development and automated testing. | [libraries/AP_HAL_SITL/README.md](../libraries/AP_HAL_SITL/README.md) |
| **AP_HAL_QURT/** | Qualcomm Hexagon DSP platform support for high-performance signal processing on QURT RTOS. Enables advanced sensor fusion on DSP hardware. | [libraries/AP_HAL_QURT/README.md](../libraries/AP_HAL_QURT/README.md) |
| **AP_HAL_Empty/** | Template HAL implementation providing stub patterns for porting to new platforms. Useful reference for implementing new platform support. | [libraries/AP_HAL_Empty/README.md](../libraries/AP_HAL_Empty/README.md) |

## Sensor Libraries

Drivers and management for inertial, environmental, and positioning sensors.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_InertialSensor/** | Inertial Measurement Unit (IMU) subsystem managing gyroscopes and accelerometers. Supports multiple IMU chips (MPU6000, ICM20xxx, BMI088), sensor fusion, calibration, and filtering. | [libraries/AP_InertialSensor/README.md](../libraries/AP_InertialSensor/README.md) |
| **AP_Compass/** | Magnetometer subsystem for heading determination. Supports multiple compass types, calibration algorithms, motor interference compensation, and external/internal compass management. | [libraries/AP_Compass/README.md](../libraries/AP_Compass/README.md) |
| **AP_Baro/** | Barometer subsystem for altitude estimation via pressure sensing. Supports MS5611, BMP280, LPS25H and other pressure sensors with temperature compensation. | [libraries/AP_Baro/README.md](../libraries/AP_Baro/README.md) |
| **AP_GPS/** | GPS subsystem supporting UBLOX, NMEA, SBF, GSOF protocols. Handles RTK corrections, GPS blending from multiple receivers, and GPS+compass combination devices. | [libraries/AP_GPS/README.md](../libraries/AP_GPS/README.md) |
| **AP_RangeFinder/** | Distance sensor subsystem for terrain following and obstacle detection. Supports lidar (Lightware, TFMini), sonar (MaxBotix), and radar sensors with multi-rangefinder configurations. | [libraries/AP_RangeFinder/README.md](../libraries/AP_RangeFinder/README.md) |
| **AP_Airspeed/** | Airspeed sensor subsystem for fixed-wing aircraft using differential pressure sensors (MS4525, MS5525, DLVR). Includes calibration and wind estimation. | [libraries/AP_Airspeed/README.md](../libraries/AP_Airspeed/README.md) |
| **AP_OpticalFlow/** | Optical flow sensor subsystem for velocity estimation without GPS. Supports PX4Flow, CXOF, MSP optical flow sensors with EKF integration. | [libraries/AP_OpticalFlow/README.md](../libraries/AP_OpticalFlow/README.md) |
| **AP_BattMonitor/** | Battery monitoring subsystem for voltage, current, and capacity tracking. Supports analog sensors, SMBus fuel gauges, and ESC telemetry integration with failsafe triggers. | [libraries/AP_BattMonitor/README.md](../libraries/AP_BattMonitor/README.md) |

## Navigation and Estimation

Attitude determination, state estimation, and position tracking algorithms.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_AHRS/** | Attitude and Heading Reference System providing vehicle orientation. Supports DCM (Direction Cosine Matrix) and EKF backends with coordinate frame transformations and sensor fusion. | [libraries/AP_AHRS/README.md](../libraries/AP_AHRS/README.md) |
| **AP_NavEKF2/** | Extended Kalman Filter implementation (version 2) for sensor fusion. Estimates position, velocity, and attitude with innovation checking and covariance prediction. | [libraries/AP_NavEKF2/README.md](../libraries/AP_NavEKF2/README.md) |
| **AP_NavEKF3/** | Extended Kalman Filter implementation (version 3) with improvements over EKF2. Adds multi-IMU support, visual odometry, beacon fusion, and GSF yaw estimator. Current recommended EKF. | [libraries/AP_NavEKF3/README.md](../libraries/AP_NavEKF3/README.md) |
| **AP_InertialNav/** | Inertial navigation for position and velocity estimation. Integrates accelerometer data with EKF corrections for high-rate position updates. | [libraries/AP_InertialNav/README.md](../libraries/AP_InertialNav/README.md) |
| **AP_Beacon/** | Beacon positioning subsystem for indoor navigation. Performs triangulation from multiple beacon distance measurements for GPS-denied environments. | [libraries/AP_Beacon/README.md](../libraries/AP_Beacon/README.md) |
| **AP_VisualOdom/** | Visual odometry integration for camera-based positioning. Supports Intel T265 tracking camera and MAVLink visual odometry messages. | [libraries/AP_VisualOdom/README.md](../libraries/AP_VisualOdom/README.md) |
| **AP_Navigation/** | Navigation helper functions for waypoint calculations, distance/bearing computations, and path planning utilities. | [libraries/AP_Navigation/README.md](../libraries/AP_Navigation/README.md) |

## Control Systems

Attitude, position, and guidance control algorithms for autonomous flight.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AC_AttitudeControl/** | Multicopter attitude control with rate and angle controllers. Implements PID control, input shaping, slew rates, and square root controller for responsive flight. | [libraries/AC_AttitudeControl/README.md](../libraries/AC_AttitudeControl/README.md) |
| **AC_PosControl/** | Multicopter position control for velocity and position hold. Implements S-curve navigation, altitude hold, and loiter control with acceleration limits. | [libraries/AC_PosControl/README.md](../libraries/AC_PosControl/README.md) |
| **AC_WPNav/** | Waypoint navigation for multirotors with trajectory generation, speed profiles, corner cutting, and spline waypoint support. Handles terrain following in missions. | [libraries/AC_WPNav/README.md](../libraries/AC_WPNav/README.md) |
| **AC_AutoTune/** | Automated PID tuning system using frequency sweep analysis. Performs safe in-flight tuning with oscillation detection and safety mechanisms. | [libraries/AC_AutoTune/README.md](../libraries/AC_AutoTune/README.md) |
| **APM_Control/** | Fixed-wing control with roll, pitch, and yaw controllers. Implements PID tuning for planes and control surface mixing. | [libraries/APM_Control/README.md](../libraries/APM_Control/README.md) |
| **AP_L1_Control/** | L1 navigation controller for fixed-wing waypoint tracking and path following. Handles loiter circles and crosstrack error correction. | [libraries/AP_L1_Control/README.md](../libraries/AP_L1_Control/README.md) |
| **AP_TECS/** | Total Energy Control System for fixed-wing altitude and airspeed coupling. Optimizes climb/descent performance and implements landing flare control. | [libraries/AP_TECS/README.md](../libraries/AP_TECS/README.md) |

## Motor and Servo Control

Motor mixing, ESC communication, and servo output management.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_Motors/** | Motor control for multirotors with frame-specific mixing. Supports various frame types (quad, hex, octo, Y6), motor ordering, thrust curves, and safety limits. | [libraries/AP_Motors/README.md](../libraries/AP_Motors/README.md) |
| **AR_Motors/** | Rover motor control for ground vehicles. Implements differential steering, Ackermann steering, throttle/brake control, and motor output scaling. | [libraries/AR_Motors/README.md](../libraries/AR_Motors/README.md) |
| **SRV_Channel/** | Servo channel management for output function mapping. Handles servo assignment, min/max/trim configuration, reversing, and channel allocation. | [libraries/SRV_Channel/README.md](../libraries/SRV_Channel/README.md) |
| **AP_BLHeli/** | BLHeli ESC support with passthrough for ESC configuration using BLHeliSuite. Provides ESC telemetry and 4-way interface. | [libraries/AP_BLHeli/README.md](../libraries/AP_BLHeli/README.md) |
| **AP_FETtecOneWire/** | FETtec OneWire ESC protocol implementation with configuration commands and telemetry parsing. | [libraries/AP_FETtecOneWire/README.md](../libraries/AP_FETtecOneWire/README.md) |

## Communication Protocols

Telemetry, command, and peripheral communication systems.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **GCS_MAVLink/** | Ground Control Station communication via MAVLink protocol. Handles message routing, command processing, telemetry streaming, mission protocol, parameter protocol, and FTP. | [libraries/GCS_MAVLink/README.md](../libraries/GCS_MAVLink/README.md) |
| **AP_CANManager/** | CAN bus management for multiple CAN interfaces. Routes messages to CAN protocol handlers (DroneCAN, PiccoloCAN) and manages CAN driver selection. | [libraries/AP_CANManager/README.md](../libraries/AP_CANManager/README.md) |
| **AP_DroneCAN/** | DroneCAN/UAVCAN protocol implementation for CAN peripherals. Supports GPS, compass, ESC, servos via CAN with DNA server and firmware updates. | [libraries/AP_DroneCAN/README.md](../libraries/AP_DroneCAN/README.md) |
| **AP_DDS/** | DDS (Data Distribution Service) integration for ROS2 communication via Micro-XRCE-DDS. Enables ArduPilot-ROS2 interoperability. | [libraries/AP_DDS/README.md](../libraries/AP_DDS/README.md) |
| **AP_RCProtocol/** | RC receiver protocol decoding supporting SBUS, PPM, DSM, CRSF, and others. Implements protocol auto-detection and failsafe handling. | [libraries/AP_RCProtocol/README.md](../libraries/AP_RCProtocol/README.md) |
| **AP_Frsky_Telem/** | FrSky telemetry protocol implementation (D, X, passthrough variants). Provides battery, GPS, and status telemetry to FrSky transmitters. | [libraries/AP_Frsky_Telem/README.md](../libraries/AP_Frsky_Telem/README.md) |
| **AP_MSP/** | MultiWii Serial Protocol for OSD integration. Provides telemetry data and command handling for MSP-compatible on-screen displays. | [libraries/AP_MSP/README.md](../libraries/AP_MSP/README.md) |
| **AP_RCTelemetry/** | RC telemetry base class for various telemetry protocols. Manages telemetry data selection and transmission scheduling. | [libraries/AP_RCTelemetry/](../libraries/AP_RCTelemetry/) |

## Mission and Safety

Autonomous mission execution, geofencing, and safety systems.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_Mission/** | Mission management for autonomous waypoint navigation. Handles mission storage, command execution (waypoints, loiter, takeoff, land), resume logic, and conditional commands. | [libraries/AP_Mission/README.md](../libraries/AP_Mission/README.md) |
| **AC_Fence/** | Geofencing system with multiple fence types (cylinder, polygon, altitude). Implements breach detection, breach actions, polygon inclusion/exclusion zones, and enable/disable logic. | [libraries/AC_Fence/README.md](../libraries/AC_Fence/README.md) |
| **AP_Rally/** | Rally point system for alternate landing locations. Manages rally point storage, RTL rally selection, and landing rally points. | [libraries/AP_Rally/README.md](../libraries/AP_Rally/README.md) |
| **AP_Arming/** | Arming system with comprehensive pre-flight checks. Implements vehicle-specific safety checks, sensor validation, GPS quality checks, and override mechanisms. | [libraries/AP_Arming/README.md](../libraries/AP_Arming/README.md) |
| **AP_AdvancedFailsafe/** | Advanced failsafe with termination conditions. Handles hardware termination pins, GCS heartbeat monitoring, and critical system failures. | [libraries/AP_AdvancedFailsafe/README.md](../libraries/AP_AdvancedFailsafe/README.md) |
| **AP_SmartRTL/** | Smart Return-to-Launch with optimized return paths. Records flight path, simplifies trajectory, and generates efficient return routes avoiding obstacles. | [libraries/AP_SmartRTL/README.md](../libraries/AP_SmartRTL/README.md) |
| **AP_Parachute/** | Parachute deployment system for emergency recovery. Handles deployment triggers, servo control, and safety interlocks. | [libraries/AP_Parachute/](../libraries/AP_Parachute/) |

## Logging and Storage

Data logging, parameter storage, and filesystem abstraction.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_Logger/** | Binary logging system for flight data recording. Implements high-speed logging, multiple storage backends (SD card, flash), streaming, and log message definitions. | [libraries/AP_Logger/README.md](../libraries/AP_Logger/README.md) |
| **AP_Param/** | Parameter system for persistent configuration storage. Manages parameter groups, EEPROM layout, default values, parameter conversion, and runtime access. | [libraries/AP_Param/README.md](../libraries/AP_Param/README.md) |
| **StorageManager/** | Storage abstraction layer with wear leveling. Provides storage areas for parameters, fences, rally points with backend independence. | [libraries/StorageManager/README.md](../libraries/StorageManager/README.md) |
| **AP_Filesystem/** | Filesystem abstraction supporting FAT, LittleFS, and ROMFS. Handles SD card and flash storage with unified API. | [libraries/AP_Filesystem/README.md](../libraries/AP_Filesystem/README.md) |
| **AP_FlashStorage/** | Flash-based persistent storage for microcontrollers without EEPROM. Implements wear leveling and error correction. | [libraries/AP_FlashStorage/](../libraries/AP_FlashStorage/) |

## Math and Utilities

Mathematical operations, filtering, and task scheduling.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_Math/** | Math library for vectors, matrices, quaternions. Implements coordinate transformations, splines, curves, geometric algorithms, and control theory functions. | [libraries/AP_Math/README.md](../libraries/AP_Math/README.md) |
| **Filter/** | Digital filter library with low-pass, notch, and complementary filters. Provides filter design patterns and tuning guidance. | [libraries/Filter/README.md](../libraries/Filter/README.md) |
| **PID/** | PID controller implementation with anti-windup, derivative filtering, and feedforward. Includes tuning guidelines. | [libraries/PID/README.md](../libraries/PID/README.md) |
| **AC_PID/** | Advanced PID controller with ArduPilot-specific extensions. Adds square root controller, slew rate limiting, and integrated logging. | [libraries/AC_PID/README.md](../libraries/AC_PID/README.md) |
| **AP_Scheduler/** | Task scheduler for real-time loop management. Handles task priorities, loop rate control, timing budgets, and performance monitoring. | [libraries/AP_Scheduler/README.md](../libraries/AP_Scheduler/README.md) |
| **AP_Common/** | Common utility functions and definitions used across ArduPilot. Provides basic types, macros, and helper functions. | [libraries/AP_Common/](../libraries/AP_Common/) |

## Peripherals and Accessories

Camera, gimbal, LED, and accessory control.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **AP_Camera/** | Camera trigger and control system. Supports intervalometer, mission integration, geotagging, and MAVLink camera protocol. | [libraries/AP_Camera/README.md](../libraries/AP_Camera/README.md) |
| **AP_Mount/** | Gimbal mount control with multiple backends (MAVLink, Storm32, servo). Implements pointing modes (GPS, ROI, retract), RC control, and mission integration. | [libraries/AP_Mount/README.md](../libraries/AP_Mount/README.md) |
| **AP_OSD/** | On-screen display system for FPV video overlays. Manages OSD panel configuration, font management, and backend support (MAVLink, MSP). | [libraries/AP_OSD/README.md](../libraries/AP_OSD/README.md) |
| **AP_Scripting/** | Lua scripting interface for custom behaviors. Provides sandboxed scripting environment with ArduPilot API bindings and safety restrictions. | [libraries/AP_Scripting/README.md](../libraries/AP_Scripting/README.md) |
| **AP_Notify/** | Notification system for LEDs, buzzers, and external indicators. Provides visual/audio feedback for arming, GPS, modes, and errors. | [libraries/AP_Notify/](../libraries/AP_Notify/) |
| **AP_Button/** | Button input handling for physical switches. Supports single, double, and long-press actions with configurable functions. | [libraries/AP_Button/](../libraries/AP_Button/) |
| **AP_Relay/** | Relay control for switching external devices. Manages relay outputs with MAVLink and mission control. | [libraries/AP_Relay/](../libraries/AP_Relay/) |
| **AP_Gripper/** | Gripper control for cargo delivery. Supports servo and EPM (electro-permanent magnet) grippers. | [libraries/AP_Gripper/](../libraries/AP_Gripper/) |

## Tools and Build System

Development tools, testing frameworks, and build infrastructure.

| Module | Description | Documentation |
|--------|-------------|---------------|
| **Tools/autotest/** | Automated testing framework using SITL simulation. Includes test scenarios, test creation patterns, and CI integration for comprehensive system testing. | [Tools/autotest/README.md](../Tools/autotest/README.md) |
| **Tools/Replay/** | Log replay system for algorithm development and debugging. Uses Data Abstraction Layer (DAL) to replay sensor data for EKF tuning and analysis. | [Tools/Replay/README.md](../Tools/Replay/README.md) |
| **Tools/ros2/** | ROS2 integration tools and configuration. Provides DDS configuration, message mapping, example ROS2 applications, and troubleshooting guides. | [Tools/ros2/README.md](../Tools/ros2/README.md) |
| **Tools/environment_install/** | Development environment setup scripts. Platform-specific installation for dependencies, toolchains, and IDE configuration. | [Tools/environment_install/README.md](../Tools/environment_install/README.md) |
| **Tools/scripts/** | Build and utility scripts including documentation generation, parameter extraction, and log analysis tools. | [Tools/scripts/README.md](../Tools/scripts/README.md) |
| **Tools/ardupilotwaf/** | Build system internals using waf. Explains board configuration, feature flags, and build customization. | [Tools/ardupilotwaf/README.md](../Tools/ardupilotwaf/README.md) |

## Additional Resources

- **[ArduPilot Glossary](glossary.md)** - Definitions of ArduPilot-specific terminology, acronyms, and concepts
- **[Coordinate Frames Reference](coordinate-frames.md)** - NED frame, body frame, and transformation conventions
- **[Unit Conventions](units.md)** - Standard units for length, angles, time, and velocity
- **[Documentation Generation Guide](README.md)** - How to build and contribute to ArduPilot documentation

## Quick Module Lookup by Function

### Need to understand...
- **Sensor integration?** → Start with AP_InertialSensor, AP_Compass, AP_Baro, AP_GPS
- **State estimation?** → AP_AHRS, AP_NavEKF3, AP_InertialNav
- **Control algorithms?** → AC_AttitudeControl (copter), APM_Control (plane), AR_Motors (rover)
- **Mission execution?** → AP_Mission, AC_Fence, AP_Rally
- **Hardware porting?** → AP_HAL, AP_HAL_ChibiOS (hwdef system)
- **Communication protocols?** → GCS_MAVLink, AP_DroneCAN, AP_DDS
- **Safety systems?** → AP_Arming, AP_AdvancedFailsafe, AC_Fence
- **Motor control?** → AP_Motors, SRV_Channel, AP_BLHeli
- **Testing?** → Tools/autotest, Tools/Replay, AP_HAL_SITL

### Working on...
- **New flight mode?** → Study mode.h/mode.cpp in vehicle directories
- **Sensor driver?** → Follow probe→init→register_periodic_callback→update pattern
- **Board support?** → Create hwdef file in AP_HAL_ChibiOS/hwdef/
- **Parameter tuning?** → AP_Param system and vehicle Parameters.cpp
- **Algorithm debugging?** → Tools/Replay with flight logs

---

*This index is generated from the ArduPilot repository structure. For detailed implementation information, refer to the linked README files and generated Doxygen API documentation.*

