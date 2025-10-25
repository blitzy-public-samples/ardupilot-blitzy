# ArduPilot Glossary

This glossary provides comprehensive definitions of terms, acronyms, and technical terminology used throughout the ArduPilot documentation and codebase. Terms are organized by category for easy reference.

---

## Table of Contents

- [Common Acronyms](#common-acronyms)
- [Coordinate Systems](#coordinate-systems)
- [Flight Mode Terminology](#flight-mode-terminology)
- [Hardware Terms](#hardware-terms)
- [Software Architecture Terms](#software-architecture-terms)
- [Communication Protocols](#communication-protocols)
- [Sensor Terminology](#sensor-terminology)
- [Navigation and Control Terms](#navigation-and-control-terms)
- [Safety System Terminology](#safety-system-terminology)
- [Build System Terms](#build-system-terms)

---

## Common Acronyms

### A

**AC** - ArduCopter prefix for copter-specific libraries (e.g., AC_AttitudeControl)

**ADC** - Analog-to-Digital Converter; converts analog sensor signals to digital values

**AFS** - Advanced Failsafe System; comprehensive failsafe framework for loss of control detection

**AHRS** - Attitude Heading Reference System; provides vehicle orientation estimate by fusing IMU, GPS, and magnetometer data

**AGL** - Above Ground Level; altitude relative to terrain directly below the vehicle

**AP** - ArduPilot prefix for shared libraries (e.g., AP_GPS, AP_InertialSensor)

**API** - Application Programming Interface; defines how software components interact

**AR** - ArduRover prefix for rover-specific libraries

**ARMED** - State where motors/actuators are enabled and ready for operation

**ARSPD** - Airspeed; forward velocity relative to surrounding air

**ASL** - Above Sea Level; altitude relative to mean sea level

**AUV** - Autonomous Underwater Vehicle

### B

**BARO** - Barometric pressure sensor; used for altitude estimation

**BEC** - Battery Elimination Circuit; voltage regulator that powers electronics from main battery

**BLHeli** - Popular ESC firmware supporting telemetry and configuration protocols

### C

**CAN** - Controller Area Network; robust serial bus protocol for vehicle networks

**CCM** - Core-Coupled Memory; fast SRAM region on some ARM processors for time-critical data

**CHIBIOS** - Real-Time Operating System used by ArduPilot on most flight controllers

**COMPASS** - Magnetometer sensor; measures magnetic field for heading estimation

**CPU** - Central Processing Unit

**CRC** - Cyclic Redundancy Check; error detection code for data integrity

**CRSF** - Crossfire RC protocol; high-performance radio control link protocol

### D

**DAL** - Data Abstraction Layer; isolation layer for EKF replay capability

**DCM** - Direction Cosine Matrix; rotation matrix representation for attitude; also legacy AHRS algorithm

**DDS** - Data Distribution Service; middleware for real-time distributed systems (e.g., ROS 2 integration)

**DMA** - Direct Memory Access; hardware subsystem for high-speed data transfers without CPU involvement

**DO_JUMP** - Mission command to repeat a sequence of waypoints

**DSHOT** - Digital Shot; digital ESC protocol offering improved reliability over PWM

**DroneCAN** - Lightweight CAN protocol for drones (formerly UAVCAN v0); see also UAVCAN

### E

**EEPROM** - Electrically Erasable Programmable Read-Only Memory; non-volatile storage for parameters

**EKF** - Extended Kalman Filter; sensor fusion algorithm estimating position, velocity, and attitude

**EKF2** - AP_NavEKF2 library; second-generation Extended Kalman Filter implementation

**EKF3** - AP_NavEKF3 library; third-generation Extended Kalman Filter implementation (current default)

**ELRS** - ExpressLRS; open-source long-range RC protocol

**ESC** - Electronic Speed Controller; motor driver accepting throttle commands

**EXPO** - Exponential curve applied to control inputs for smoother feel around center stick

### F

**FCU** - Flight Control Unit; the autopilot hardware (synonymous with flight controller)

**FENCE** - Geofence; virtual boundary that triggers actions when breached

**FMU** - Flight Management Unit; main processor on Pixhawk-style autopilots

**FPV** - First Person View; piloting via real-time video transmission

**FRAM** - Ferroelectric RAM; non-volatile memory technology used for parameter storage

### G

**GCS** - Ground Control Station; software for vehicle command and telemetry (e.g., Mission Planner, QGroundControl)

**GNSS** - Global Navigation Satellite System; generic term for GPS, GLONASS, Galileo, BeiDou

**GPS** - Global Positioning System; satellite navigation providing position and velocity

**GUIDED** - Flight mode accepting real-time position targets from GCS or companion computer

### H

**HAL** - Hardware Abstraction Layer; isolates platform-specific code from vehicle logic

**HDOP** - Horizontal Dilution of Precision; GPS accuracy metric (lower is better)

**HIL** - Hardware In the Loop; simulation mode where real hardware runs against simulated sensors

**HW** - Hardware

**HWDEF** - Hardware Definition; configuration file specifying board pinouts and features

### I

**I2C** - Inter-Integrated Circuit; two-wire serial bus for sensors and peripherals

**IMU** - Inertial Measurement Unit; sensor package containing accelerometers and gyroscopes

**INS** - Inertial Navigation System; dead-reckoning navigation from IMU integration

**INT** - Interrupt; hardware signal triggering immediate CPU response

**IO** - Input/Output; also refers to secondary processor on some Pixhawk boards handling PWM

**IRC** - Internet Relay Chat; real-time text communication (ArduPilot community uses Discord)

### L

**LED** - Light Emitting Diode; status indicator lights

**LIDAR** - Light Detection and Ranging; laser-based distance sensor

**LOS** - Line of Sight; direct radio path without obstructions

**LPF** - Low-Pass Filter; removes high-frequency noise from sensor data

**LQI** - Link Quality Indicator; measure of radio link reliability

**LUA** - Scripting language integrated into ArduPilot for user customization

### M

**MAG** - Magnetometer (same as COMPASS)

**MAVLINK** - Micro Air Vehicle Link; primary telemetry and command protocol for ArduPilot

**MSL** - Mean Sea Level; reference altitude for barometric calculations

**MSP** - MultiWii Serial Protocol; alternative telemetry protocol for FPV equipment integration

### N

**NED** - North-East-Down coordinate frame; earth-fixed reference frame with +X north, +Y east, +Z down

**NEU** - North-East-Up coordinate frame; earth-fixed reference frame with +X north, +Y east, +Z up

**NMEA** - National Marine Electronics Association; standard GPS message format

**NuttX** - Real-Time Operating System formerly used by ArduPilot (replaced by ChibiOS)

### O

**OBC** - On-Board Computer (same as companion computer)

**OSD** - On-Screen Display; text/graphics overlay on FPV video

### P

**PARAM** - Parameter; configurable value stored in non-volatile memory

**PID** - Proportional-Integral-Derivative controller; fundamental feedback control algorithm

**POSHOLD** - Position Hold flight mode; maintains current GPS position

**PPM** - Pulse Position Modulation; RC receiver protocol encoding multiple channels in single signal

**PPK** - Post-Processed Kinematic; GPS technique achieving cm-level accuracy via ground station correction after flight

**PRU** - Programmable Real-time Unit; auxiliary processor on BeagleBone boards

**PWM** - Pulse Width Modulation; signal encoding value as pulse duration; traditional servo/ESC protocol

### Q

**Q_** - Parameter prefix for QuadPlane (VTOL) features in ArduPlane

**QHOVER** - QuadPlane hover mode; multirotor-style hovering in VTOL aircraft

**QLOITER** - QuadPlane loiter mode; GPS position hold using multirotor motors

**QRTL** - QuadPlane Return to Launch; RTL using multirotor mode

**QUATERNION** - Four-element representation of 3D rotation; avoids gimbal lock

### R

**RADAT** - Raw Data; unprocessed sensor values

**RADIO** - RC (Radio Control) receiver and transmitter system

**RALLY** - Rally point; alternate landing location for failsafe recovery

**RANGEFINDER** - Distance sensor (ultrasonic, lidar, radar) for altitude or obstacle detection

**RC** - Radio Control; pilot input via transmitter

**RELAY** - Digital output for controlling external devices (e.g., camera trigger)

**ROLL** - Rotation about vehicle's longitudinal (X) axis

**ROSMASTER** - ROS (Robot Operating System) master node coordinating distributed nodes

**ROV** - Remotely Operated Vehicle; pilot-controlled underwater vehicle

**RPM** - Revolutions Per Minute; motor or propeller rotation rate

**RSSI** - Received Signal Strength Indicator; radio link quality measurement

**RTK** - Real-Time Kinematic; GPS technique achieving cm-level accuracy via ground station correction

**RTL** - Return to Launch; autonomous flight mode returning vehicle to takeoff location

### S

**SBUS** - Serial Bus; Futaba RC protocol using inverted serial communication

**SERIAL** - UART (Universal Asynchronous Receiver-Transmitter) communication port

**SERVO** - Actuator for control surfaces; also generic term for any PWM/DSHOT output

**SIM** - Simulation; also prefix for SITL parameters

**SITL** - Software In The Loop; simulation environment running vehicle code on host computer

**SOAR** - Autonomous soaring mode for gliders using thermal updrafts

**SPI** - Serial Peripheral Interface; four-wire high-speed bus for sensors and peripherals

**SRTM** - Shuttle Radar Topography Mission; global terrain elevation database

**SRXL** - Spektrum Remote Link; Spektrum RC protocol

**STABILIZE** - Basic flight mode with pilot stick inputs controlling attitude rates

**SUMD** - Graupner HoTT serial protocol

### T

**TECS** - Total Energy Control System; airspeed/altitude controller for fixed-wing aircraft

**TELEM** - Telemetry; sensor data and status information transmitted to ground

**TERRAIN** - Terrain-following system using terrain database and rangefinder

**THROTTLE** - Motor power control; vertical axis on RC transmitter

**TILT** - Tilting mechanism (motors or wings) for VTOL transition

**TOF** - Time of Flight; distance measurement technique used in some rangefinders

**TRIM** - Neutral position offset for control inputs

### U

**UART** - Universal Asynchronous Receiver-Transmitter; serial communication hardware

**UAVCAN** - CAN protocol for UAVs (v0 is now DroneCAN; v1 is Cyphal)

**UBLOX** - U-blox; popular GPS manufacturer

**UDP** - User Datagram Protocol; connectionless network protocol used in SITL

### V

**VDOP** - Vertical Dilution of Precision; GPS altitude accuracy metric

**VFR** - Visual Flight Rules; also MAVLink message with key flight parameters

**VTOL** - Vertical Take-Off and Landing; aircraft combining multirotor and fixed-wing capabilities

**VTX** - Video Transmitter; FPV video transmission equipment

### W

**WAF** - Waf build system; Python-based build tool used by ArduPilot

**WAYPOINT** - GPS coordinate defining mission flight path

**WP** - Waypoint (abbreviated)

**WS** - Wingspan; also prefix for some fixed-wing parameters

### Y

**YAW** - Rotation about vehicle's vertical (Z) axis; heading direction

---

## Coordinate Systems

### Body Frame
Right-handed coordinate system fixed to the vehicle:
- **+X axis**: Forward (nose direction)
- **+Y axis**: Right wing/right side
- **+Z axis**: Down (through belly)

Rotations:
- **Roll**: Rotation about X axis
- **Pitch**: Rotation about Y axis  
- **Yaw**: Rotation about Z axis

### NED Frame (North-East-Down)
Earth-fixed inertial reference frame:
- **+X axis**: True north
- **+Y axis**: East
- **+Z axis**: Down (toward Earth's center)

**Usage**: Default earth frame in ArduPilot; navigation calculations, EKF state, mission waypoints

**Advantages**: 
- Matches aviation conventions
- Z-down simplifies altitude calculations (positive altitude = negative Z)
- Avoids singularities at poles

### NEU Frame (North-East-Up)
Earth-fixed inertial reference frame:
- **+X axis**: True north
- **+Y axis**: East
- **+Z axis**: Up (away from Earth's center)

**Usage**: Some external systems (ROS, geographic standards), MAVLink global position messages

**Conversion from NED**: Negate Z component

### Local Tangent Plane
2D coordinate system tangent to Earth's surface at origin point:
- **Origin**: Typically vehicle home/launch location
- **+X axis**: North at origin
- **+Y axis**: East at origin

**Usage**: Local navigation when distances are small enough to ignore Earth curvature (<10km)

### ECEF (Earth-Centered Earth-Fixed)
3D Cartesian system with origin at Earth's center:
- Rotates with Earth
- Used for precise GPS calculations

### Geodetic Coordinates
Latitude, longitude, altitude (WGS84 ellipsoid):
- Used for mission waypoints and global position reporting
- Converted to/from NED for navigation calculations

---

## Flight Mode Terminology

### Mode Categories

**Manual Modes**: Direct pilot control with minimal stabilization
- Raw RC inputs control actuators with limited processing

**Stabilized Modes**: Pilot controls attitude, autopilot maintains stability
- Rate stabilization, attitude hold, self-leveling

**Autonomous Modes**: Autopilot controls vehicle following predefined plan
- Mission execution, return to launch, automated landing

**Guided Modes**: Autopilot controls vehicle following real-time external commands
- GCS commanded positions, companion computer control

### Common Flight Modes (Cross-Vehicle)

**MANUAL**: Direct pilot control, no stabilization (Plane, Rover)

**STABILIZE**: Pilot controls rates, autopilot stabilizes attitude (Copter, Plane, Sub)

**ACRO**: Acrobatic mode, pilot controls angular rates directly (Copter, Plane)

**LOITER**: GPS position hold with pilot override (Copter, Rover)

**AUTO**: Mission execution mode following waypoints

**RTL**: Return to Launch, autonomous return to home location

**GUIDED**: Accepts position/velocity commands from GCS or companion computer

**HOLD**: Stops vehicle and holds position (mode specifics vary by vehicle type)

**BRAKE**: Aggressive stop using maximum deceleration (Copter)

### Copter-Specific Modes

**ALT_HOLD**: Altitude hold using barometer/rangefinder, pilot controls horizontal position

**POS_HOLD**: GPS position and altitude hold (older name for LOITER)

**DRIFT**: Simplified control for FPV flight, coordinated turns

**SPORT**: Rate-limited attitude control for smooth flight

**FLIP**: Automated flip maneuver

**AUTOTUNE**: Automated PID tuning through flight test maneuvers

**LAND**: Automated landing with descent rate control

**CIRCLE**: Flies in circle around point of interest

**FLOWHOLD**: Position hold using optical flow sensor (no GPS)

**ZIGZAG**: Automated crop spraying pattern

**SYSTEMID**: System identification mode for model parameter extraction

**AUTOROTATE**: Helicopter autorotation for engine failure

**TURTLE**: Inverted mode for self-recovery after crash (BetaFlight-style)

### Plane-Specific Modes

**FBWA**: Fly-By-Wire A, stabilized with pilot speed/height control

**FBWB**: Fly-By-Wire B, adds altitude and airspeed locks

**CRUISE**: Altitude and heading hold, optimized for level flight

**TRAINING**: Self-leveling with pilot override, prevents steep banks

**AUTOTUNE**: Automated tuning of roll/pitch controllers

**THERMAL**: Autonomous thermal soaring for gliders

**QSTABILIZE, QHOVER, QLOITER**: QuadPlane multirotor modes

**QLAND**: QuadPlane vertical landing

**QRTL**: QuadPlane return to launch using multirotor

### Rover-Specific Modes

**STEERING**: Manual steering with speed control

**FOLLOW**: Follow another vehicle or object

**SIMPLE**: Simplified control mode

**DOCK**: Automated docking with precision positioning

### Sub-Specific Modes

**SURFTRAK**: Surface tracking mode maintaining constant altitude below surface

**POSHOLD**: Position and depth hold underwater

**MOTOR_DETECT**: Detects which thrusters are reverse-configured

### Mode Transitions

**Mode Switch**: RC channel or GCS command triggering mode change

**Failsafe Mode**: Automatic mode entered when failsafe condition detected

**Mode Requirements**: Conditions that must be met to enter mode (e.g., GPS lock for LOITER)

**Mode Fallback**: Automatic transition to simpler mode when requirements not met

---

## Hardware Terms

### Flight Controllers

**Autopilot**: Computer hardware running ArduPilot firmware

**Flight Controller**: Synonym for autopilot (FCU)

**Pixhawk**: Popular open-hardware autopilot design family

**Cube**: Ruggedized Pixhawk variant with modular carrier boards

**Kakute**: Compact flight controller optimized for racing drones

**Matek**: Flight controller manufacturer producing various ArduPilot-compatible boards

**Navigator**: Autopilot designed for companion computer integration (Raspberry Pi)

**Durandal**: High-performance H7-based autopilot

**CUAV**: Autopilot manufacturer producing Pixhawk-compatible boards

### Processors

**STM32**: ARM Cortex-M microcontroller family used in most flight controllers

**F4**: STM32F4 series (Cortex-M4, 168-180MHz)

**F7**: STM32F7 series (Cortex-M7, 216MHz)

**H7**: STM32H7 series (Cortex-M7, 400-480MHz)

**IOMCU**: Secondary STM32F1 processor handling PWM I/O on some Pixhawk boards

**Companion Computer**: Separate Linux computer (Raspberry Pi, NVIDIA Jetson) for vision/AI

### Sensors

**Accelerometer**: Measures linear acceleration along three axes

**Gyroscope**: Measures angular rotation rate about three axes

**Magnetometer**: Measures magnetic field strength for heading determination

**Barometer**: Measures atmospheric pressure for altitude estimation

**GPS Module**: Receiver for satellite navigation signals

**Optical Flow**: Downward camera tracking ground motion for non-GPS position hold

**Rangefinder**: Distance sensor (ultrasonic, lidar, radar) measuring altitude or obstacles

**Airspeed Sensor**: Pitot tube measuring dynamic pressure for airspeed calculation

**Current Sensor**: Measures battery current draw for power monitoring

**Voltage Sensor**: Measures battery voltage for capacity estimation

**RPM Sensor**: Measures motor or engine rotation speed

### Actuators

**Brushless Motor**: Three-phase electric motor driven by ESC

**Brushed Motor**: Traditional DC motor with mechanical commutation

**Servo**: Position-controlled actuator for control surfaces

**Linear Actuator**: Extends/retracts for tilt mechanisms or doors

**Solenoid**: Electromagnetic switch for valves or releases

### ESC (Electronic Speed Controller)

**ESC**: Motor driver converting throttle commands to motor power

**BLHeli**: Popular ESC firmware family

**BLHeli_32**: 32-bit ARM-based BLHeli variant with advanced features

**BLHeli_S**: 8-bit BLHeli variant for smaller ESCs

**SimonK**: Open-source ESC firmware optimized for multirotors

**ESC Telemetry**: Feedback from ESC reporting RPM, voltage, current, temperature

**ESC Calibration**: Process of teaching ESC throttle range endpoints

**4-in-1 ESC**: Single board containing four ESC circuits

**DShot**: Digital ESC protocol providing improved reliability and bidirectional communication

**Oneshot**: High-speed PWM ESC protocol (Oneshot125, Oneshot42)

**Multishot**: Even faster PWM ESC protocol

**Bidirectional DShot**: DShot variant returning RPM telemetry without separate wire

### Power Systems

**Battery**: Energy storage (LiPo, Li-ion, LiFe, NiMH)

**LiPo**: Lithium Polymer battery, common for drones (3.7V/cell nominal)

**Cell Count**: Number of battery cells in series (2S=7.4V, 3S=11.1V, 4S=14.8V, 6S=22.2V)

**C Rating**: Battery discharge rate (e.g., 30C = 30× capacity per hour)

**Battery Monitor**: Circuit measuring voltage and current for capacity estimation

**Power Module**: Integrated voltage regulator and current sensor

**BEC**: Battery Elimination Circuit, voltage regulator for electronics

**PDB**: Power Distribution Board, routes battery power to ESCs

### RC (Radio Control) Equipment

**Transmitter**: Handheld controller operated by pilot (TX)

**Receiver**: Aircraft-mounted radio receiving pilot commands (RX)

**RC Channel**: Single control input (throttle, roll, pitch, yaw, switches)

**Mode Switch**: RC channel configured to select flight mode

**Failsafe Setting**: Receiver behavior when signal lost

**Binding**: Process of pairing transmitter and receiver

**Telemetry Radio**: Bidirectional radio link for GCS communication (e.g., 433MHz, 915MHz)

### Communication Hardware

**Telemetry Module**: Radio modem for MAVLink communication (e.g., SiK radio, RFD900)

**FPV Camera**: Video camera for first-person view

**Video Transmitter (VTX)**: Broadcasts FPV video signal

**Video Receiver**: Ground station equipment receiving FPV video

**Antenna Tracker**: Motorized directional antenna automatically pointing at vehicle

**Crossfire (CRSF)**: TBS long-range RC system

**ExpressLRS (ELRS)**: Open-source long-range RC system

**LTE Module**: Cellular modem for beyond-line-of-sight communication

### Peripherals

**LED**: Status indicator lights (internal and external)

**Buzzer**: Audio feedback for arming, errors, warnings

**Safety Switch**: Physical button required before arming (Pixhawk feature)

**Parachute**: Emergency recovery system with servo release

**Sprayer**: Liquid delivery system for agriculture

**Gripper**: Servo-actuated cargo release mechanism

**Winch**: Motor-driven cable for lifting or lowering payloads

**Camera Gimbal**: Stabilized camera mount with servo or brushless motors

**Camera Trigger**: Automated camera shutter control for photogrammetry

---

## Software Architecture Terms

### Core Architecture

**Scheduler**: Task management system calling functions at specified frequencies

**Main Loop**: Primary control loop running at 400Hz (typically) on copter, variable on plane

**Fast Loop**: High-rate attitude control loop (gyro rate, typically 400-1600Hz)

**Update Rate**: Frequency at which function is called (Hz)

**Task Priority**: Scheduler ordering for critical vs non-critical tasks

**Semaphore**: Mutual exclusion lock protecting shared resources from concurrent access

**Thread Safety**: Code property ensuring correct behavior under concurrent execution

### Code Organization

**Library**: Self-contained module providing specific functionality (`AP_*`, `AC_*` prefixes)

**Frontend**: User-facing API of a library (e.g., `AP_GPS`)

**Backend**: Hardware-specific implementation (e.g., `AP_GPS_UBLOX`)

**Driver**: Low-level code interfacing with specific hardware

**Vehicle Code**: Platform-specific control logic (`ArduCopter/`, `ArduPlane/`, etc.)

**HAL**: Hardware Abstraction Layer isolating platform dependencies

**Singleton**: Class with single global instance accessed via static method

**AP Namespace**: Global scope for accessing singletons (`AP::gps()`, `AP::ahrs()`)

### Memory Management

**Static Allocation**: Memory allocated at compile time (preferred in ArduPilot)

**Dynamic Allocation**: Runtime memory allocation (avoided in flight-critical code)

**Stack**: Temporary memory for function calls and local variables

**Heap**: Dynamic memory pool (rarely used in ArduPilot)

**DMA Buffer**: Memory region accessible by Direct Memory Access hardware

**CCM**: Core-Coupled Memory, fast SRAM for time-critical data

**SRAM**: Static RAM, main working memory

**Flash**: Non-volatile program storage

**Memory Pool**: Pre-allocated buffer for specific purpose

### Parameter System

**Parameter**: Named configuration value stored in non-volatile memory

**Parameter Group**: Related parameters organized hierarchically

**Default Value**: Initial parameter value if not explicitly set

**Parameter Metadata**: Information about valid ranges, units, description

**EEPROM**: Storage location for parameters (may be actual EEPROM, flash, or FRAM)

**Parameter List**: MAVLink protocol for downloading/uploading parameters

### Logging

**DataFlash**: Historical name for onboard logging system (now AP_Logger)

**AP_Logger**: Library managing onboard log writing

**Log Message**: Structured data record with message type and fields

**Log Backend**: Storage destination (SD card, internal flash, MAVLink streaming)

**Replay**: Tool for re-running EKF with recorded sensor data for analysis

**Log Analysis**: Post-flight log review for performance tuning and issue diagnosis

### State Management

**Arm State**: Whether vehicle is armed (motors enabled) or disarmed

**Pre-arm Check**: Safety validation before allowing arming

**Flight Mode**: Current control behavior (STABILIZE, AUTO, RTL, etc.)

**Control Mode**: Synonym for flight mode

**Failsafe State**: Safety condition requiring autonomous action

**Home Position**: Takeoff location, RTL destination

### Events and Messaging

**GCS Message**: MAVLink message sent to ground control station

**Status Text**: Human-readable message displayed in GCS

**Event**: Logged occurrence (mode change, failsafe trigger, waypoint reached)

**Notification**: Visual/audio feedback (LED patterns, buzzer tones)

**Callback**: Function called when event occurs

**Message Handler**: Code processing received MAVLink message

---

## Communication Protocols

### MAVLink

**MAVLink**: Micro Air Vehicle Link, primary telemetry and command protocol

**MAVLink 1**: Original protocol with 8-bit message IDs (up to 255 messages)

**MAVLink 2**: Extended protocol with 24-bit message IDs, signing, extensibility

**Message**: Structured data packet with type ID and fields

**System ID**: Unique identifier for vehicle in MAVLink network

**Component ID**: Identifier for subsystem within vehicle (autopilot=1, gimbal=154)

**Message Stream**: Periodic transmission of telemetry message

**Stream Rate**: Frequency (Hz) of message stream

**Request Data Stream**: GCS command setting stream rates

**Command Protocol**: MAVLink commands using COMMAND_LONG and COMMAND_INT messages

**Command ACK**: Acknowledgment of command execution success/failure

**Mission Protocol**: MAVLink message sequences for uploading/downloading waypoint missions

**Parameter Protocol**: MAVLink message sequences for reading/writing parameters

**FTP Protocol**: MAVLink file transfer for logs, terrain data, parameters

**Signing**: MAVLink 2 feature cryptographically authenticating messages

**Message Routing**: Forwarding MAVLink messages between multiple systems/components

### DroneCAN (UAVCAN v0)

**DroneCAN**: Lightweight CAN-based protocol for vehicle networks (formerly UAVCAN v0)

**CAN**: Controller Area Network, robust serial bus

**CAN Node**: Device on CAN bus with unique Node ID

**Node ID**: Address (1-127) identifying CAN device

**Dynamic Node Allocation**: Automatic node ID assignment

**UAVCAN**: Original protocol name (v0 became DroneCAN after v1 divergence)

**Cyphal**: Next-generation protocol (UAVCAN v1)

**DNA Server**: Dynamic Node Allocation server assigning node IDs

**Message Type**: Data structure definition for CAN messages

**Service**: Request/response transaction between CAN nodes

**Broadcast**: One-to-many message transmission

**ESC Telemetry**: Motor feedback via DroneCAN

**GPS over CAN**: GPS receiver using CAN instead of serial

**Compass over CAN**: Magnetometer on CAN bus

### Other Protocols

**SBUS**: Futaba serial RC protocol (inverted UART, 100Kbaud, 16 channels)

**CRSF**: Crossfire protocol for long-range RC and telemetry

**SUMD**: Graupner HoTT serial RC protocol

**SRXL**: Spektrum Remote Receiver Link

**PPM**: Pulse Position Modulation, analog RC protocol combining channels

**PWM**: Pulse Width Modulation for individual RC channels or servo/ESC control

**DSHOT**: Digital ESC protocol (DSHOT150, DSHOT300, DSHOT600, DSHOT1200)

**NMEA**: GPS message format (ASCII text sentences)

**UBX**: u-blox binary GPS protocol

**RTCM**: Radio Technical Commission for Maritime Services, RTK correction format

**MSP**: MultiWii Serial Protocol for FPV equipment integration

**LTM**: Light Telemetry protocol for FPV OSD

**Passthrough Telemetry**: ArduPilot telemetry embedded in RC protocol (FrSky, CRSF)

**FrSky**: RC manufacturer, protocol variants (D/X/SPORT)

### ROS/ROS2 Integration

**DDS**: Data Distribution Service, ROS 2 middleware

**ROS**: Robot Operating System, robotics software framework

**ROS 2**: Second generation ROS with improved real-time performance

**Topic**: Named channel for publish/subscribe communication

**Node**: Independent process in ROS network

**MAVROS**: ROS package bridging MAVLink and ROS

**AP_DDS**: ArduPilot library providing native DDS/ROS 2 integration

---

## Sensor Terminology

### Inertial Sensors (IMU)

**IMU**: Inertial Measurement Unit containing accelerometers and gyroscopes

**Accelerometer**: Measures linear acceleration including gravity (m/s²)

**Gyroscope**: Measures angular rotation rate (rad/s or deg/s)

**3-Axis**: Sensor measuring along X, Y, Z axes

**6-DOF**: Six degrees of freedom (3-axis accel + 3-axis gyro)

**9-DOF**: Nine degrees of freedom (6-DOF + 3-axis magnetometer)

**10-DOF**: Ten degrees of freedom (9-DOF + barometer)

**IMU Backend**: Driver for specific IMU hardware (Invensense, Bosch, etc.)

**Sample Rate**: Frequency at which sensor data is read (Hz)

**IMU Batch Sampling**: Reading multiple samples using hardware FIFO for averaging

**IMU Filtering**: Digital filtering to remove noise while preserving bandwidth

**Notch Filter**: Removes specific frequency (e.g., propeller vibration)

**Harmonic Notch**: Dynamic notch filter tracking motor RPM harmonics

**IMU Calibration**: Process of determining sensor offsets and scale factors

**Accel Calibration**: Six-position calibration measuring gravity in all axes

**Gyro Calibration**: Stationary calibration measuring zero-rate bias

**Temperature Calibration**: Characterizing sensor error vs temperature

### Magnetometer (Compass)

**Magnetometer**: Measures magnetic field strength (gauss or tesla)

**Compass**: Magnetometer used for heading determination

**Declination**: Angle between magnetic north and true north

**Inclination**: Angle of magnetic field relative to horizontal plane

**Hard Iron**: Magnetic offset from ferrous materials (constant in body frame)

**Soft Iron**: Magnetic distortion from ferrous materials (rotation-dependent)

**Compass Calibration**: Process of characterizing hard/soft iron effects

**Compass Motor Compensation**: Correcting for magnetic interference from motor currents

**External Compass**: Magnetometer on mast/wing away from vehicle magnetic interference

**Primary Compass**: Compass selected as heading source when multiple available

**Compass Consistency Check**: Monitoring agreement between multiple compasses

### Barometric Pressure

**Barometer**: Measures atmospheric pressure for altitude estimation

**Static Pressure**: Ambient air pressure (vs dynamic pressure in airspeed sensor)

**Pressure Altitude**: Altitude calculated from pressure using standard atmosphere model

**QNH**: Altimeter setting adjusted to report altitude above sea level

**Temperature Compensation**: Correcting pressure altitude for non-standard temperature

**Baro Drift**: Slow pressure sensor offset change over time/temperature

### GPS (GNSS)

**GPS**: Global Positioning System (US satellite navigation)

**GNSS**: Generic term for all satellite navigation systems

**GLONASS**: Russian satellite navigation system

**Galileo**: European satellite navigation system

**BeiDou**: Chinese satellite navigation system

**QZSS**: Japanese regional satellite navigation system

**GPS Fix**: Valid position solution from satellite signals

**Fix Type**: Solution quality (No fix, 2D, 3D, DGPS, RTK Float, RTK Fixed)

**2D Fix**: Latitude/longitude without altitude (insufficient satellites)

**3D Fix**: Latitude, longitude, and altitude solution

**DGPS**: Differential GPS using correction data for improved accuracy (~1m)

**RTK**: Real-Time Kinematic GPS achieving cm-level accuracy

**RTK Float**: RTK solution without carrier phase ambiguity resolution (~30cm)

**RTK Fixed**: RTK solution with resolved ambiguities (1-2cm)

**PPK**: Post-Processed Kinematic, RTK accuracy computed after flight

**Base Station**: Stationary GPS providing correction data to rover

**NTRIP**: Network Transport of RTCM via Internet Protocol (RTK over internet)

**GPS Satellites**: Number of satellites used in position solution

**HDOP**: Horizontal Dilution of Precision, geometry quality metric (lower is better)

**GPS Velocity**: Doppler-derived velocity from satellite signals

**GPS Heading**: Heading from dual GPS antennas or GPS velocity (moving base)

**GPS Glitch**: Sudden erroneous position jump detected by EKF innovation checks

**GPS Blending**: Fusing multiple GPS receivers for improved reliability

### Rangefinders

**Rangefinder**: Distance sensor measuring altitude or obstacles

**Ultrasonic**: Sound-based rangefinder (e.g., MaxBotix, HC-SR04)

**Lidar**: Light-based rangefinder (e.g., Lightware, Benewake)

**Radar**: Radio-based rangefinder (e.g., Ainstein, Phoenix)

**TOF**: Time of Flight measurement principle

**Laser Altimeter**: Downward lidar for terrain following

**Proximity Sensor**: 360° obstacle detection system

**Rangefinder Filtering**: Rejection of spurious readings (multipath, reflections)

### Airspeed

**Airspeed Sensor**: Pitot tube measuring dynamic pressure for airspeed calculation

**Pitot Tube**: Forward-facing tube measuring total pressure

**Static Port**: Side-facing port measuring static pressure

**Differential Pressure**: Pressure difference indicating airspeed

**True Airspeed**: Actual speed through air mass

**Indicated Airspeed**: Uncorrected airspeed from sensor

**Calibrated Airspeed**: Corrected for instrument and position errors

**Airspeed Ratio**: Correction factor for sensor calibration

### Optical Flow

**Optical Flow**: Downward camera measuring ground motion

**Flow Rate**: Apparent ground motion in sensor frame (rad/s)

**Quality Metric**: Optical flow confidence indicator

**Surface Texture**: Ground pattern detail enabling optical tracking

**Flow Fusion**: Integrating optical flow into position estimate

### Other Sensors

**Current Sensor**: Measures battery discharge current (amperes)

**Voltage Sensor**: Measures battery voltage for capacity estimation

**RPM Sensor**: Tachometer measuring engine/motor rotation speed

**Temperature Sensor**: Environmental or component temperature monitoring

**Humidity Sensor**: Atmospheric moisture measurement

**Wind Sensor**: Measures wind speed and direction

**Wheel Encoder**: Measures wheel rotation for odometry (rovers)

---

## Navigation and Control Terms

### Attitude Estimation

**Attitude**: Vehicle orientation (roll, pitch, yaw) relative to earth frame

**Euler Angles**: Roll, pitch, yaw angle representation

**Quaternion**: Four-element rotation representation avoiding gimbal lock

**Rotation Matrix**: 3×3 matrix transforming vectors between frames

**DCM**: Direction Cosine Matrix (rotation matrix)

**AHRS**: Attitude Heading Reference System fusing IMU and compass

**EKF Attitude**: Attitude estimate from Extended Kalman Filter

**Gimbal Lock**: Mathematical singularity in Euler angle representation at ±90° pitch

### Position Estimation

**Position**: Location in earth frame (north, east, down) or (lat, lon, alt)

**Velocity**: Rate of change of position (m/s)

**EKF**: Extended Kalman Filter fusing all sensors for position/velocity estimate

**GPS Fusion**: Incorporating GPS measurements into EKF

**Baro Fusion**: Incorporating barometric altitude into EKF

**Optical Flow Fusion**: Incorporating optical flow velocity into EKF

**Innovation**: Difference between sensor measurement and EKF prediction

**Innovation Variance**: Expected innovation magnitude based on uncertainties

**EKF Health**: Statistical measure of EKF consistency and reliability

**Variance**: Statistical uncertainty in estimated state

**Position Reset**: Discontinuous EKF position correction when switching sensors

### Waypoint Navigation

**Waypoint**: GPS coordinate defining mission flight path

**Home Position**: Takeoff location, RTL destination, mission origin

**Rally Point**: Alternate landing location for failsafe

**Mission**: Sequence of waypoints and commands

**MAV_CMD**: MAVLink command type (e.g., MAV_CMD_NAV_WAYPOINT)

**DO Command**: Mission command executed immediately (e.g., DO_SET_SERVO)

**NAV Command**: Mission command with associated waypoint location

**Waypoint Radius**: Distance from waypoint considered "reached"

**Loiter**: Circle around waypoint at specified radius

**Loiter Turns**: Number of circles before proceeding to next waypoint

**Spline Waypoint**: Smooth curved path through waypoint

**Terrain Following**: Maintaining constant AGL altitude using terrain database or rangefinder

**WP_NAV**: Waypoint navigation library (AC_WPNav)

### Path Planning

**L1 Controller**: Lateral navigation algorithm following curved paths

**NPFG**: Nonlinear Path Following Guidance (modern fixed-wing path controller)

**Path Tracking**: Following desired trajectory

**Crosstrack Error**: Perpendicular distance from desired path

**Along-Track Error**: Distance along path (ahead/behind desired position)

**Look-Ahead Distance**: Forward distance for path following algorithm

**Pursuit Point**: Target point on path for vehicle to track

### Attitude Control

**Attitude Controller**: Converts desired attitude to motor/servo outputs

**Rate Controller**: PID controller for angular rate (deg/s)

**Angle Controller**: P controller for attitude angle error, outputs rate target

**Rate Loop**: Inner (fast) control loop

**Angle Loop**: Outer (slow) control loop

**PID**: Proportional-Integral-Derivative controller

**P Gain**: Proportional term, immediate response to error

**I Gain**: Integral term, eliminates steady-state error

**D Gain**: Derivative term, damping and phase lead

**Feed-Forward**: Control term based on desired trajectory (not error)

**IMAX**: Maximum integrator windup limit

**Input Shaping**: Filtering commanded inputs for smooth response

**Slew Rate**: Maximum rate of change limit

### Position Control

**Position Controller**: Converts desired position to attitude targets

**Velocity Controller**: Intermediate loop between position and attitude

**Leash**: Maximum horizontal distance from pilot stick in position control modes

**Pilot Velocity**: Desired velocity from pilot stick input

**Braking**: Deceleration profile for smooth stops

**Jerk Limiting**: Limiting rate of acceleration change for smooth flight

**S-Curve**: Acceleration profile with smooth jerk-limited transitions

**SCurve Navigation**: Using S-curves for waypoint transitions

### Altitude Control

**Altitude Hold**: Maintaining constant altitude using baro/GPS

**Climb Rate**: Vertical velocity (m/s, positive = up)

**Throttle Hover**: Throttle percentage to maintain altitude (copters)

**Altitude Error**: Difference between desired and actual altitude

**Rangefinder Fusion**: Using rangefinder for precision altitude hold near ground

**Surface Tracking**: Maintaining constant height above terrain/water

**TECS**: Total Energy Control System balancing altitude/airspeed (fixed-wing)

### Speed Control

**Throttle**: Motor power control

**Cruise Throttle**: Throttle for level flight at cruise speed (plane)

**Throttle Curve**: Mapping from pilot input to motor output

**Throttle Slew**: Rate limit on throttle changes

**Speed Controller**: Maintains desired airspeed or ground speed

**TECS**: Total Energy Control System for coordinated altitude/speed control

---

## Safety System Terminology

### Arming System

**Arming**: Enabling motors/actuators for flight

**Disarming**: Disabling motors/actuators

**Pre-arm Check**: Safety validation before allowing arming

**Arming Check**: Final validation during arming sequence

**Safety Switch**: Physical button required before arming (Pixhawk feature)

**Force Arming**: Override arming checks (dangerous, not recommended)

**Rudder Arming**: Arming via RC stick command (right rudder)

**Arming Timeout**: Automatic disarm if no throttle within time limit

**Motor Interlock**: Secondary safety requiring explicit motor enable

### Failsafe System

**Failsafe**: Autonomous action when safety condition detected

**Failsafe Trigger**: Event causing failsafe activation

**Failsafe Action**: Response to failsafe (RTL, LAND, continue mission, etc.)

**Failsafe Priority**: Higher priority failsafes override lower priority

**Failsafe Recovery**: Automatic return to normal operation when condition clears

**Radio Failsafe**: No RC input received within timeout

**Throttle Failsafe**: RC throttle below configured failsafe value

**GCS Failsafe**: No GCS heartbeat received within timeout

**Battery Failsafe**: Battery voltage/capacity below critical threshold

**EKF Failsafe**: Navigation estimate unreliable

**GPS Failsafe**: GPS quality inadequate or GPS lost

**Fence Breach**: Vehicle exceeded geofence boundary

**AFS**: Advanced Failsafe System for autonomous recovery

### Geofence

**Geofence**: Virtual boundary triggering actions when breached

**Altitude Fence**: Maximum altitude limit

**Circular Fence**: Cylinder around home position

**Polygon Fence**: Custom boundary defined by waypoints

**Inclusion Fence**: Area where vehicle must remain

**Exclusion Fence**: Area vehicle must avoid

**Fence Action**: Response to fence breach (RTL, brake, report only)

**Fence Margin**: Distance buffer before triggering action

### Crash Detection

**Crash Check**: Monitoring for crash condition during flight

**Parachute Release**: Automatic or manual emergency recovery system

**Impact Detection**: Accelerometer-based crash detection

**Flip Detection**: Detecting vehicle inversion

**Motor Failure Detection**: Detecting individual motor or ESC failure

**Thrust Loss**: Detecting insufficient thrust for controlled flight

### Pre-Flight Safety

**GPS Lock**: Valid GPS position required for some modes

**Compass Consistency**: Agreement between multiple compasses

**Accelerometer Health**: Accelerometer sanity checks

**Gyro Health**: Gyroscope sanity checks

**Barometer Health**: Barometer sanity check

**Airspeed Health**: Airspeed sensor validation

**Battery Check**: Adequate battery voltage/capacity for flight

**RC Calibration Check**: Valid RC input ranges

**Sensor Calibration Check**: Required sensors properly calibrated

### In-Flight Monitoring

**Vibration Monitoring**: Detecting excessive vibrations affecting sensors

**EKF Variance Check**: Monitoring EKF uncertainty levels

**Innovation Check**: Detecting sensor glitches or failures

**Internal Error**: Software error or inconsistency detection

**Watchdog**: Hardware or software timeout detecting system hang

**Brown-Out Detection**: Detecting power supply voltage sag

**Temperature Monitoring**: Overtemperature protection

---

## Build System Terms

### Waf Build System

**Waf**: Python-based build automation tool used by ArduPilot

**wscript**: Build configuration file defining targets and options

**configure**: Build phase detecting toolchain and options

**build**: Build phase compiling source to binaries

**Board**: Hardware platform target (e.g., Pixhawk4, CubeOrange, MatekH743)

**Vehicle**: Software platform target (copter, plane, rover, sub, etc.)

**Build Target**: Specific combination of vehicle and board

**Toolchain**: Compiler, linker, and associated tools for target platform

**Cross-Compilation**: Building for different architecture than host (e.g., ARM on x86)

### Build Configuration

**HAL**: Hardware Abstraction Layer selection (ChibiOS, Linux, SITL)

**Feature**: Optional functionality enabled/disabled at compile time

**ENABLE_/DISABLE_**: Preprocessor defines controlling features

**Board Configuration**: Board-specific settings and pin assignments

**hwdef.dat**: Hardware definition file describing board capabilities

**Bootloader**: Small program loading main firmware

**APJ File**: ArduPilot JSON firmware format for bootloader updates

### Build Tools

**arm-none-eabi-gcc**: GNU compiler for ARM embedded targets

**Python**: Required for waf build system

**git**: Version control for source code management

**gdb**: GNU debugger for debugging

**Submodule**: Git repository embedded within ArduPilot repository

**PX4 Firmware**: Legacy dependency for some drivers (being phased out)

**ChibiOS**: RTOS kernel and HAL used by most ArduPilot boards

### Build Options

**Debug Build**: Binary with debug symbols, optimization disabled

**Optimized Build**: Production binary with compiler optimization

**Size Optimization**: Optimizing for smaller binary size

**Speed Optimization**: Optimizing for execution speed

**LTO**: Link-Time Optimization for size/speed improvements

**UBSAN**: Undefined Behavior Sanitizer for bug detection

**Coverage**: Code coverage instrumentation for testing

### SITL (Software In The Loop)

**SITL**: Software In The Loop simulation running on host computer

**SIM_**: Parameter prefix for simulation configuration

**Physics Model**: Simulated vehicle dynamics

**Simulated Sensors**: Virtual IMU, GPS, baro, etc.

**MAVProxy**: Command-line GCS for SITL testing

**JSBSim**: High-fidelity flight dynamics simulator integration

**Gazebo**: 3D robot simulator integration

**AirSim**: Microsoft simulator integration

**RealFlight**: Commercial RC flight simulator integration

### Testing and CI

**Autotest**: Automated test framework using SITL

**Unit Test**: Code-level test of individual functions

**Integration Test**: Test of multiple components working together

**CI**: Continuous Integration, automated building and testing

**GitHub Actions**: CI system running ArduPilot tests

**Regression Test**: Ensuring changes don't break existing functionality

---

## Usage Notes

### Cross-References

Terms marked in **bold** within definitions are also defined in this glossary. Follow these cross-references to understand related concepts.

### Frame Conventions

When interpreting code or documentation, pay careful attention to coordinate frame:
- Body frame vs earth frame
- NED vs NEU conventions  
- Euler angles vs quaternions
- Units (degrees vs radians, meters vs centimeters)

### Safety-Critical Terms

Terms related to arming, failsafes, crash detection, and geofencing are safety-critical. Incorrect understanding or configuration can lead to vehicle loss or injury. Always validate safety system behavior through thorough ground testing before flight.

### Version Differences

ArduPilot is under active development. Some terms and features may vary between versions. Consult version-specific documentation for your firmware.

### Additional Resources

- **ArduPilot Wiki**: https://ardupilot.org/
- **Developer Documentation**: https://ardupilot.org/dev/
- **Discourse Forum**: https://discuss.ardupilot.org/
- **Discord Chat**: https://ardupilot.org/discord

---

*This glossary is maintained as part of the ArduPilot documentation enhancement initiative. Contributions and corrections are welcome through the standard ArduPilot development process.*
