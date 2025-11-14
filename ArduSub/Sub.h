/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file Sub.h
 * @brief Main ArduSub vehicle class declaration
 * 
 * @details This is the foundational header file for the ArduSub underwater vehicle.
 *          It declares the Sub class which serves as the main vehicle singleton,
 *          managing all subsystems including:
 *          - 6-DOF motor control for underwater maneuvering
 *          - Depth control and pressure sensing
 *          - Navigation and position holding in underwater environments
 *          - Joystick/gamepad input processing
 *          - Underwater-specific flight modes (Manual, Stabilize, Depth Hold, etc.)
 *          - Leak detection and safety systems
 *          - Buoyancy compensation
 *          - Camera and light control
 * 
 *          The Sub class inherits from AP_Vehicle and implements the scheduler-based
 *          architecture used throughout ArduPilot, with tasks running at various rates
 *          (fast loop at 400Hz, medium loops at 50Hz, 10Hz, 3Hz, 1Hz).
 * 
 * @note This file should be included by all ArduSub source files that need access
 *       to the main vehicle state and subsystems.
 * 
 * @warning This is safety-critical code for underwater vehicle control. Modifications
 *          to failsafe logic, depth control, or motor management could result in
 *          vehicle loss or operator injury.
 */

#pragma once

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

// Standard C/C++ libraries
#include <cmath>        // Mathematical functions (sin, cos, sqrt, etc.)
#include <stdio.h>      // Standard I/O for debugging
#include <stdarg.h>     // Variable argument lists

// Hardware Abstraction Layer - platform-independent interface to hardware
#include <AP_HAL/AP_HAL.h>

// Common dependencies - core ArduPilot infrastructure
#include <AP_Common/AP_Common.h>                    // Common definitions, macros, and utility functions
#include <AP_Common/Location.h>                     // GPS location handling (lat/lon/alt)
#include <AP_Param/AP_Param.h>                      // Parameter storage and management system
#include <StorageManager/StorageManager.h>          // Non-volatile storage abstraction
#include <AP_AccelCal/AP_AccelCal.h>                // Accelerometer calibration interface and mathematics
#include <AP_Math/AP_Math.h>                        // Vector/Matrix math library (Vector3f, Matrix3f, quaternions)
#include <AP_Declination/AP_Declination.h>          // Magnetic declination calculations (true north vs magnetic north)

// Sensor libraries - hardware sensor interfaces and drivers
#include <AP_GPS/AP_GPS.h>                          // GPS/GNSS receiver interface (position, velocity, time)
#include <AP_Logger/AP_Logger.h>                    // Binary logging system for flight data recording
#include <AP_Baro/AP_Baro.h>                        // Barometer/pressure sensor (depth sensing for subs)
#include <AP_Compass/AP_Compass.h>                  // Magnetometer/compass interface (heading reference)
#include <AP_InertialSensor/AP_InertialSensor.h>    // IMU interface (accelerometer and gyroscope)

// Navigation and state estimation libraries
#include <AP_AHRS/AP_AHRS.h>                        // Attitude and Heading Reference System (orientation estimation)
#include <AP_InertialNav/AP_InertialNav.h>          // Inertial navigation (velocity and position estimation)

// Mission management
#include <AP_Mission/AP_Mission.h>                  // Autonomous mission command execution

// Control libraries - attitude and position control
#include <AC_AttitudeControl/AC_AttitudeControl_Sub.h> // Underwater vehicle attitude control (roll, pitch, yaw)
#include <AC_AttitudeControl/AC_PosControl.h>          // Position controller (lat/lon/depth hold)
#include <AP_Motors/AP_Motors.h>                       // Motor mixing and output (6-DOF for subs)

// Utility libraries
#include <Filter/Filter.h>                          // Digital filters (low-pass, complementary, etc.)
#include <AP_Relay/AP_Relay.h>                      // Relay control for lights, cameras, etc.
#include <AP_Mount/AP_Mount.h>                      // Gimbal/camera mount control and stabilization

// Core vehicle framework
#include <AP_Vehicle/AP_Vehicle.h>                  // Base vehicle class (inherited by Sub)

// Waypoint navigation libraries
#include <AC_WPNav/AC_WPNav.h>                      // Waypoint navigation (straight-line paths)
#include <AC_WPNav/AC_Loiter.h>                     // Loiter/position hold navigation
#include <AC_WPNav/AC_Circle.h>                     // Circle navigation mode

// Scheduler and performance monitoring
#include <AP_Scheduler/AP_Scheduler.h>              // Task scheduler (runs callbacks at fixed rates)
#include <AP_Scheduler/PerfInfo.h>                  // Performance monitoring and loop timing stats

// Power and safety systems
#include <AP_BattMonitor/AP_BattMonitor.h>          // Battery voltage/current monitoring and failsafes

// Terrain and obstacle avoidance
#include <AP_Terrain/AP_Terrain.h>                  // Terrain database and following

// Sub-specific systems
#include <AP_JSButton/AP_JSButton.h>                // Joystick/gamepad button mapping and function assignment
#include <AP_LeakDetector/AP_LeakDetector.h>        // Leak detection for underwater vehicle safety
#include <AP_Proximity/AP_Proximity.h>              // Proximity sensors for collision avoidance

// Rally points for emergency returns
#include <AP_Rally/AP_Rally.h>                      // Rally point storage and selection

// On-Screen Display
#include <AP_OSD/AP_OSD.h>                          // OSD panel management for video overlay

// ArduSub-specific local modules
#include "defines.h"                        // Sub-specific constants and enumerations
#include "config.h"                         // Build-time configuration options
#include "GCS_MAVLink_Sub.h"                // MAVLink ground control station interface for Sub
#include "RC_Channel_Sub.h"                 // RC input channel management (Sub-specific extensions)
#include "Parameters.h"                     // Parameter definitions and groups (g, g2)
#include "AP_Arming_Sub.h"                  // Pre-arm and arming safety checks (Sub-specific)
#include "GCS_Sub.h"                        // Ground control station telemetry handler
#include "mode.h"                           // Flight mode base class and mode declarations
#include "script_button.h"                  // Scripting button interface

// Optical flow for velocity estimation (often used in GPS-denied environments)
#include <AP_OpticalFlow/AP_OpticalFlow.h>

// Conditionally-compiled libraries (dependent on build configuration)
// These includes are guarded by feature flags defined in defines.h and config.h

#if RCMAP_ENABLED
// RC input channel mapping (allows remapping RC channels at runtime)
#include <AP_RCMapper/AP_RCMapper.h>
#endif

// RPM sensor support (for motor or thruster RPM monitoring)
#include <AP_RPM/AP_RPM_config.h>
#if AP_RPM_ENABLED
#include <AP_RPM/AP_RPM.h>
#endif

#if AVOIDANCE_ENABLED
// Fence and obstacle avoidance system
#include <AC_Avoidance/AC_Avoid.h>
#endif

// Camera triggering and control
#include <AP_Camera/AP_Camera.h>

#if AP_SCRIPTING_ENABLED
// Lua scripting engine for custom behaviors
#include <AP_Scripting/AP_Scripting.h>
#endif

/**
 * @class Sub
 * @brief Main vehicle class for ArduSub underwater vehicle
 * 
 * @details The Sub class is the central singleton that manages all subsystems for the
 *          ArduSub underwater vehicle. It inherits from AP_Vehicle and implements:
 * 
 *          **Scheduler Architecture:**
 *          - Fast loop (400Hz): Attitude control, motor output, inertial sensor updates
 *          - 50Hz loop: RC input, position control updates
 *          - 10Hz loop: Navigation updates, battery monitoring
 *          - 3Hz loop: Compass updates, surface/bottom detection
 *          - 1Hz loop: Logging, statistics, slow updates
 * 
 *          **Subsystem Ownership:**
 *          - Motor control: AP_Motors6DOF (6 degrees of freedom)
 *          - Navigation: Inertial navigation, AHRS, EKF
 *          - Control: Attitude control (roll/pitch/yaw), position control (lat/lon/depth)
 *          - Sensors: Barometer (depth), compass, IMU, GPS, rangefinder, optical flow
 *          - Safety: Leak detector, failsafe manager, arming checks
 *          - Input: Joystick/gamepad button processing (6-DOF control)
 *          - Communication: MAVLink GCS interface, telemetry, parameter management
 * 
 *          **Flight Modes:**
 *          Manual, Stabilize, Acro, AltHold (Depth Hold), Poshold, Guided, Auto,
 *          Circle, Surface (automatic ascent), Surftrak (terrain following), Motordetect
 * 
 *          **Coordinate Frames:**
 *          - Body frame: X-forward, Y-right, Z-down (standard aircraft convention)
 *          - NED frame: North-East-Down earth-fixed frame
 *          - Depth: Positive values = deeper underwater (meters or centimeters)
 * 
 *          **Singleton Pattern:**
 *          There is exactly one Sub instance created at startup, accessible globally
 *          via the "sub" extern and Sub::get_singleton().
 * 
 * @note This class uses friend declarations extensively to allow mode classes and
 *       subsystems to access private members directly for performance.
 * 
 * @warning This is safety-critical code. Modifications to control loops, failsafe
 *          logic, motor mixing, or depth control could result in loss of vehicle,
 *          injury, or drowning hazard. All changes must be tested thoroughly in
 *          simulation (SITL) before deployment on actual hardware.
 */
class Sub : public AP_Vehicle {
public:
    // Friend class declarations - allow direct access to private members for performance
    friend class GCS_MAVLINK_Sub;      ///< MAVLink message handler needs vehicle state access
    friend class GCS_Sub;               ///< Ground control station interface
    friend class Parameters;            ///< Parameter loading needs direct member access
    friend class ParametersG2;          ///< Secondary parameter group
    friend class AP_Arming_Sub;         ///< Arming checks need access to vehicle state
    friend class RC_Channels_Sub;       ///< RC input processing
    friend class RC_Channel_Sub;        ///< Individual RC channel management
    friend class Mode;                  ///< Base mode class needs vehicle state
    friend class ModeManual;            ///< Manual mode (direct pilot control)
    friend class ModeStabilize;         ///< Stabilize mode (angle hold)
    friend class ModeAcro;              ///< Acrobatic mode (rate control)
    friend class ModeAlthold;           ///< Altitude/Depth hold mode
    friend class ModeSurftrak;          ///< Surface tracking mode (terrain following)
    friend class ModeGuided;            ///< Guided mode (GCS control)
    friend class ModePoshold;           ///< Position hold mode
    friend class ModeAuto;              ///< Autonomous mission mode
    friend class ModeCircle;            ///< Circle mode
    friend class ModeSurface;           ///< Automatic surface ascent mode
    friend class ModeMotordetect;       ///< Motor detection and configuration mode

    /**
     * @brief Constructor for Sub vehicle class
     * 
     * @details Initializes the Sub singleton instance. Called once at vehicle startup.
     *          The constructor sets up default values and initializes subsystems to
     *          safe states. Actual hardware initialization occurs in init_ardupilot().
     * 
     * @note Constructor runs before HAL initialization is complete, so hardware
     *       access is not available here.
     */
    Sub(void);

protected:

    bool should_zero_rc_outputs_on_reboot() const override { return true; }

private:

    /**
     * @brief Key aircraft parameters shared across multiple libraries
     * 
     * @details Contains vehicle-specific parameters like motor layout, frame type,
     *          and physical characteristics. Used by motor mixing and control libraries.
     */
    AP_MultiCopter aparm;

    /**
     * @brief Primary parameter group
     * 
     * @details Contains the majority of user-configurable parameters for the vehicle,
     *          organized into logical groups. Parameters are stored in EEPROM and
     *          accessible via MAVLink for ground control station configuration.
     */
    Parameters g;
    
    /**
     * @brief Secondary parameter group
     * 
     * @details Additional parameters added to avoid reordering the primary group
     *          (which would break parameter compatibility with existing vehicles).
     */
    ParametersG2 g2;

    /**
     * @brief Primary input control channels (6-DOF control for underwater vehicle)
     * 
     * @details ArduSub uses 6 degrees of freedom:
     *          - Roll: Bank left/right
     *          - Pitch: Tilt forward/back
     *          - Yaw: Rotate left/right (heading)
     *          - Throttle: Vertical up/down (depth control)
     *          - Forward: Translate forward/backward
     *          - Lateral: Translate left/right (strafe)
     * 
     * @note These pointers are set during RC initialization to point to the
     *       appropriate RC_Channel objects from the RC_Channels array.
     */
    RC_Channel *channel_roll;       ///< Roll control input
    RC_Channel *channel_pitch;      ///< Pitch control input
    RC_Channel *channel_throttle;   ///< Vertical throttle (depth) control
    RC_Channel *channel_yaw;        ///< Yaw rotation control
    RC_Channel *channel_forward;    ///< Forward/backward translation
    RC_Channel *channel_lateral;    ///< Left/right lateral translation
    
#if AP_SUB_RC_ENABLED  
    /**
     * @brief Flight mode switch positions array
     * 
     * @details Convenience array mapping mode switch positions to flight modes.
     *          Allows quick access to configured modes for each switch position.
     */
    AP_Int8 *flight_modes;
    const uint8_t num_flight_modes = 6;  ///< Number of available mode switch positions
#endif

    /**
     * @brief Leak detection system for underwater vehicle safety
     * 
     * @details Monitors leak detector probes placed inside the vehicle hull.
     *          Triggers failsafe actions (surface, disarm, warn) if water ingress detected.
     * 
     * @warning Critical safety system - leak detection failure could result in
     *          vehicle flooding and loss.
     */
    AP_LeakDetector leak_detector;

    /**
     * @brief Rangefinder state for terrain following and obstacle detection
     * 
     * @details Maintains current rangefinder readings, health status, and filtered
     *          altitude data. Used for surface tracking (Surftrak mode) and terrain
     *          following in autonomous missions.
     */
    struct {
        bool enabled;                           ///< True if rangefinder is present and configured
        bool alt_healthy;                       ///< True if altitude reading is reliable
        float alt;                              ///< Tilt-compensated altitude from rangefinder (meters)
        float min;                              ///< Minimum reliable rangefinder distance (meters)
        float max;                              ///< Maximum reliable rangefinder distance (meters)
        uint32_t last_healthy_ms;               ///< Timestamp of last healthy reading (milliseconds)
        float inertial_alt_cm;                  ///< Inertial altitude at time of last sample (cm)
        float rangefinder_terrain_offset_cm;    ///< Terrain height above EKF origin (cm)
        LowPassFilterFloat alt_filt;            ///< Low-pass filter for altitude smoothing
    } rangefinder_state = { false, false, 0, 0, 0, 0, 0, 0 };

#if AP_RPM_ENABLED
    /**
     * @brief RPM sensor for motor/thruster monitoring
     * 
     * @details Measures rotational speed of thrusters or auxiliary motors.
     *          Can be used for performance monitoring or closed-loop thrust control.
     */
    AP_RPM rpm_sensor;
#endif

    /**
     * @brief Autonomous mission management system
     * 
     * @details Handles waypoint missions, conditional commands, and mission scripting.
     *          Callbacks are bound to Sub methods for mission command execution:
     *          - start_command: Initialize a new mission command
     *          - verify_command: Check if current command is complete
     *          - exit_mission: Clean up when mission ends or is aborted
     * 
     * @note Mission commands include navigation (waypoints, loiter), camera control,
     *       conditional logic (delays, altitude checks), and custom scripting.
     */
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Sub::start_command, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Sub::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Sub::exit_mission, void)};

#if AP_OPTICALFLOW_ENABLED
    /**
     * @brief Optical flow sensor for velocity estimation
     * 
     * @details Provides ground-relative velocity measurements using downward-facing
     *          camera and optical flow algorithm. Useful for position hold in
     *          GPS-denied underwater environments or when near the seafloor.
     */
    AP_OpticalFlow optflow;
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
    /**
     * @brief On-Screen Display management
     * 
     * @details Manages OSD panels for video overlay, displaying flight data
     *          (depth, heading, battery, mode, etc.) on analog video feed.
     */
    AP_OSD osd;
#endif

    /**
     * @brief Timestamp of last EKF yaw reset event
     * 
     * @details The Extended Kalman Filter occasionally resets yaw (heading) when
     *          new information becomes available (e.g., GPS velocity, magnetometer).
     *          This timestamp tracks when the last reset occurred (milliseconds).
     * 
     * @note Yaw resets can cause momentary heading jumps. Mode logic uses this
     *       to detect and handle resets appropriately.
     */
    uint32_t ekfYawReset_ms = 0;

    /**
     * @brief Ground Control Station interface
     * 
     * @details Handles MAVLink communication with ground control stations.
     *          Use gcs() accessor function rather than direct _gcs access.
     */
    GCS_Sub _gcs;
    
    /**
     * @brief Accessor for GCS interface
     * @return Reference to GCS_Sub instance
     */
    GCS_Sub &gcs() { return _gcs; }

    // User-defined hook variables (optional custom extensions)
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    /**
     * @brief Vehicle status flags (bit-packed for memory efficiency)
     * 
     * @details Tracks vehicle initialization state, calibration status, and
     *          environmental detection (surface/bottom). Implemented as a union
     *          to allow both individual bit access and full uint32_t value access.
     * 
     * @note This pattern saves memory while providing fast bit-level access.
     */
    union {
        struct {
            uint8_t pre_arm_check       : 1;  ///< All pre-arm safety checks passed
            uint8_t logging_started     : 1;  ///< Binary logging has been initialized
            uint8_t compass_mot         : 1;  ///< Currently performing compass-motor calibration
            uint8_t motor_test          : 1;  ///< Currently running motor test mode
            uint8_t initialised         : 1;  ///< Vehicle initialization (init_ardupilot) complete
            uint8_t gps_base_pos_set    : 1;  ///< GPS base position established (RTK GPS)
            uint8_t at_bottom           : 1;  ///< Vehicle detected at seafloor/bottom
            uint8_t at_surface          : 1;  ///< Vehicle detected at water surface
            uint8_t depth_sensor_present: 1;  ///< Depth sensor (barometer) detected at boot
            uint8_t unused1             : 1;  ///< Reserved for future use (was compass_init_location)
        };
        uint32_t value;  ///< Full 32-bit value for bulk operations
    } ap;

    /**
     * @brief Current flight mode
     * 
     * @details The active flight control mode determining vehicle behavior:
     *          Manual, Stabilize, Acro, AltHold, Poshold, Guided, Auto, Circle,
     *          Surface, Surftrak, Motordetect. Mode changes trigger exit/enter
     *          logic for state management.
     */
    Mode::Number control_mode;

    /**
     * @brief Previous flight mode (before last mode change)
     * 
     * @details Used for mode transition logic and for restoring previous mode
     *          after temporary mode changes (e.g., failsafe recovery).
     */
    Mode::Number prev_control_mode;

#if RCMAP_ENABLED
    /**
     * @brief RC input channel remapping
     * 
     * @details Allows runtime remapping of RC channels to functions, supporting
     *          different transmitter configurations without code changes.
     */
    RCMapper rcmap;
#endif

    /**
     * @brief Failsafe status and timing tracking
     * 
     * @details Comprehensive failsafe state management for underwater vehicle safety.
     *          Tracks multiple independent failsafe conditions with timing information
     *          for hysteresis and warning rate limiting.
     * 
     *          **Failsafe Types:**
     *          - Leak detection: Water ingress into vehicle hull
     *          - Pilot input loss: Joystick/RC disconnected
     *          - GCS connection loss: Ground station communication timeout
     *          - EKF failure: Navigation estimator unhealthy
     *          - Terrain data loss: Missing terrain database
     *          - Battery failsafe: Low voltage or capacity
     *          - Internal pressure: Pressure vessel integrity
     *          - Internal temperature: Overheating electronics
     *          - Crash detection: Impact or upset condition
     *          - Sensor health: Critical sensor failure (e.g., depth sensor)
     * 
     *          **Failsafe Actions:**
     *          Actions depend on failsafe type and severity (configured per failsafe):
     *          - Warn: Send warning message to GCS
     *          - Surface: Automatically ascend to water surface
     *          - Disarm: Immediately disable motors
     * 
     * @warning This structure is critical for vehicle safety. All failsafe logic
     *          must be thoroughly tested in simulation before deployment. Incorrect
     *          failsafe behavior could result in vehicle loss, injury, or drowning.
     * 
     * @note Failsafe checks run in mainloop_failsafe_check() at 10Hz.
     */
    struct {
        // Timing tracking for rate-limited warnings (milliseconds since boot)
        uint32_t last_leak_warn_ms;             ///< Last leak warning sent to GCS
        uint32_t last_gcs_warn_ms;              ///< Last GCS connection warning
        uint32_t last_pilot_input_ms;           ///< Last valid pilot input received (MANUAL_CONTROL or RC_CHANNELS_OVERRIDE)
        uint32_t terrain_first_failure_ms;      ///< Initial terrain data failure timestamp (for timeout calculation)
        uint32_t terrain_last_failure_ms;       ///< Most recent terrain data failure
        uint32_t last_crash_warn_ms;            ///< Last crash detection warning
        uint32_t last_ekf_warn_ms;              ///< Last EKF health warning
        
#if AP_SUB_RC_ENABLED
        int8_t radio_counter;                   ///< Consecutive iterations with low throttle (RC loss detection)
        uint8_t radio               : 1;        ///< RC receiver failsafe triggered
#endif    
        // Failsafe status flags (bit-packed)
        uint8_t pilot_input          : 1;       ///< Pilot input lost (joystick disconnected)
        uint8_t gcs                  : 1;        ///< Ground control station connection lost
        uint8_t ekf                  : 1;        ///< Extended Kalman Filter unhealthy
        uint8_t terrain              : 1;        ///< Terrain database unavailable
        uint8_t leak                 : 1;        ///< Water leak detected in hull
        uint8_t internal_pressure    : 1;        ///< Internal pressure exceeded threshold
        uint8_t internal_temperature : 1;        ///< Internal temperature exceeded threshold
        uint8_t crash                : 1;        ///< Crash or violent impact detected
        uint8_t sensor_health        : 1;        ///< Critical sensor failure (depth sensor in depth-dependent modes)
    } failsafe;

    /**
     * @brief Check if any failsafe condition is currently active
     * 
     * @details Consolidated check for all failsafe types. Returns true if ANY
     *          failsafe is triggered, indicating the vehicle is in a degraded
     *          or unsafe state.
     * 
     * @return true if at least one failsafe is active, false if all clear
     * 
     * @note Used to determine if vehicle should allow mode changes, arming,
     *       or other operations that require normal vehicle state.
     */
    bool any_failsafe_triggered() const {
        return (
            failsafe.pilot_input
            || battery.has_failsafed()
            || failsafe.gcs
            || failsafe.ekf
            || failsafe.terrain
            || failsafe.leak
            || failsafe.internal_pressure
            || failsafe.internal_temperature
            || failsafe.crash
            || failsafe.sensor_health
        );
    }

    /**
     * @brief Sensor health status for logging and monitoring
     * 
     * @details Tracks individual sensor health for telemetry and logging.
     *          Used to identify which sensors are degraded or failed.
     */
    struct {
        uint8_t depth       : 1;    ///< Depth sensor (barometer) healthy
        uint8_t compass     : 1;    ///< Magnetometer/compass healthy
    } sensor_health;

    /**
     * @brief Index of barometer used as depth sensor
     * 
     * @details ArduSub can have multiple barometers. This index identifies which
     *          barometer instance is configured as the underwater depth sensor
     *          (typically an external water pressure sensor).
     * 
     * @note Critical for depth hold and altitude control modes.
     */
    uint8_t depth_sensor_idx;

    /**
     * @brief 6-DOF motor control system for underwater vehicle
     * 
     * @details Manages motor mixing and output for 6 degrees of freedom:
     *          - Roll, Pitch, Yaw rotations
     *          - Forward/Backward, Left/Right, Up/Down translations
     * 
     *          Supported thruster configurations: Vectored, BlueROV2, SimpleROV, custom
     * 
     * @warning Motor configuration must match actual vehicle thruster layout.
     *          Incorrect configuration can cause loss of control.
     */
    AP_Motors6DOF motors;

    /**
     * @brief Pilot yaw override in Circle mode
     * 
     * @details True when pilot is manually controlling yaw while in Circle mode,
     *          overriding the automatic yaw-towards-center behavior.
     */
    bool circle_pilot_yaw_override;

    /**
     * @brief Vehicle heading when motors were armed
     * 
     * @details Stores initial bearing (centidegrees, 0-36000) at arming time.
     *          Used by some modes as a reference heading.
     */
    int32_t initial_armed_bearing;

    /**
     * @brief Loiter mode timing control
     * 
     * @details Used by mission scripting to control duration of loiter at waypoints.
     */
    uint16_t loiter_time_max;                ///< Maximum loiter duration (seconds)
    uint32_t loiter_time;                    ///< Loiter start time (milliseconds since boot)

    /**
     * @brief Navigation command delay timing
     * 
     * @details Allows missions to insert delays between navigation commands for
     *          timed sequences or sensor settling.
     */
    uint32_t nav_delay_time_max_ms;          ///< Delay duration (milliseconds)
    uint32_t nav_delay_time_start_ms;        ///< Delay start time (milliseconds since boot)

    /**
     * @brief Battery monitoring and failsafe system
     * 
     * @details Monitors battery voltage, current, and remaining capacity.
     *          Triggers failsafe actions (warn, surface, disarm) based on
     *          configured thresholds.
     * 
     * @warning Battery failsafe is critical for preventing loss of vehicle due
     *          to power exhaustion underwater. Configure thresholds conservatively
     *          to allow safe ascent and recovery.
     */
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Sub::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    /**
     * @brief Arming safety check system
     * 
     * @details Implements pre-arm and arming checks to ensure vehicle is safe
     *          to operate. Checks include sensor health, calibration status,
     *          mode validity, and safety switch state.
     * 
     * @warning Vehicle must pass all arming checks before motors can be enabled.
     */
    AP_Arming_Sub arming;

    /**
     * @brief Vertical climb rate (cm/s, positive = ascending)
     * 
     * @details Filtered vertical velocity from inertial navigation. Used for
     *          depth rate control and surface/bottom detection.
     * 
     * @note In underwater context: positive = up (ascending), negative = down (descending)
     */
    int16_t climb_rate;

    /**
     * @brief Turn counting for full rotation tracking
     * 
     * @details Counts quarter-turn rotations to track total heading change beyond
     *          +/-180 degrees. Useful for operations requiring multiple full rotations.
     */
    int32_t quarter_turn_count;              ///< Number of 90-degree turns accumulated
    uint8_t last_turn_state;                 ///< Previous turn state for edge detection

    /**
     * @brief Input scaling gain for pilot commands
     * 
     * @details Multiplier applied to pilot inputs for sensitivity adjustment.
     *          Can be adjusted dynamically for precision control vs. aggressive maneuvering.
     */
    float gain;

    /**
     * @brief Input hold mode engagement status
     * 
     * @details True when input hold is active (maintaining position/attitude when
     *          pilot releases sticks). Used for hands-free operation.
     */
    bool input_hold_engaged;

    /**
     * @brief Roll/pitch direct control mode flag
     * 
     * @details When true, pilot inputs directly control roll and pitch angles
     *          instead of forward/lateral translation. Allows more aggressive
     *          maneuvering in tight spaces.
     */
    bool roll_pitch_flag = false;

    /**
     * @brief Current vehicle position
     * 
     * @details Geographic location (latitude, longitude, altitude/depth).
     *          Altitude is relative to home position (positive = up from home).
     *          Updated by inertial navigation and GPS fusion.
     */
    Location current_loc;

    /**
     * @brief Automatic yaw control mode and state
     * 
     * @details In autonomous modes (Auto, Guided), the yaw controller can operate
     *          in several modes:
     *          - Look at waypoint: Yaw towards destination
     *          - Look at ROI: Yaw towards Region of Interest (camera pointing)
     *          - Look ahead: Yaw in direction of travel
     *          - Hold heading: Maintain fixed heading
     *          - Rate control: Spin at specified rate
     */
    uint8_t auto_yaw_mode;                   ///< Current auto yaw mode
    
    bool yaw_rate_only;                      ///< True to control yaw rate instead of angle

    Vector3f roi_WP;                         ///< Region of Interest waypoint (NED frame, meters)

    float yaw_look_at_WP_bearing;            ///< Bearing to look-at waypoint (degrees)

    float yaw_xtrack_correct_heading;        ///< Cross-track error correction heading (degrees)

    int32_t yaw_look_at_heading;             ///< Target heading for look-at mode (centidegrees)

    int16_t yaw_look_at_heading_slew;        ///< Yaw rotation rate limit (deg/s)

    float yaw_look_ahead_bearing;            ///< Bearing for look-ahead mode (degrees)

    /**
     * @brief Mission scripting condition tracking
     * 
     * @details Used by conditional mission commands (delay, altitude check, etc.)
     *          to store target values and start times for condition evaluation.
     */
    int32_t condition_value;                 ///< Target value for condition (units depend on condition type)
    uint32_t condition_start;                ///< Condition start time (milliseconds since boot)

    /**
     * @brief Inertial navigation system
     * 
     * @details Fuses IMU, barometer, GPS, and other sensors to estimate position,
     *          velocity, and acceleration in 3D space. Provides high-rate position
     *          updates for control loops.
     * 
     * @note Critical for position hold and autonomous navigation.
     */
    AP_InertialNav inertial_nav;

    /**
     * @brief AHRS view for coordinate transformations
     * 
     * @details Provides coordinate frame transformations and attitude information
     *          from the Attitude and Heading Reference System. Used for converting
     *          between body frame and earth frame.
     */
    AP_AHRS_View ahrs_view;

    /**
     * @brief Attitude controller for underwater vehicle
     * 
     * @details Implements rate and angle control loops for roll, pitch, and yaw.
     *          Sub-specific variant handles underwater dynamics and 6-DOF control.
     * 
     * @note Runs at fast loop rate (typically 400Hz) for responsive control.
     * 
     * @warning Attitude control parameters must be tuned for vehicle mass, thruster
     *          characteristics, and water conditions. Poor tuning can cause oscillations
     *          or instability.
     */
    AC_AttitudeControl_Sub attitude_control;

    /**
     * @brief Position controller (lat/lon/depth)
     * 
     * @details Implements position and velocity control loops for autonomous
     *          navigation and position hold modes. Generates attitude targets
     *          for the attitude controller.
     */
    AC_PosControl pos_control;

    /**
     * @brief Waypoint navigation controller
     * 
     * @details Generates position targets for flying straight-line paths between
     *          waypoints. Handles speed profiles, acceleration/deceleration, and
     *          waypoint arrival detection.
     */
    AC_WPNav wp_nav;
    
    /**
     * @brief Loiter (position hold) controller
     * 
     * @details Maintains position at a target location, compensating for currents
     *          and disturbances. Used in Poshold mode and during mission loiter commands.
     */
    AC_Loiter loiter_nav;
    
    /**
     * @brief Circle mode navigation controller
     * 
     * @details Generates position targets for flying circular patterns around a
     *          center point. Used for surveying or maintaining position around a target.
     */
    AC_Circle circle_nav;

#if AP_CAMERA_ENABLED
    /**
     * @brief Camera trigger and control system
     * 
     * @details Manages camera triggering for mapping/survey missions and manual
     *          photo/video capture. Supports various trigger mechanisms (servo, relay, MAVLink).
     */
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

#if HAL_MOUNT_ENABLED
    /**
     * @brief Gimbal mount control and stabilization
     * 
     * @details Controls camera gimbal pointing and stabilization. Supports various
     *          gimbal protocols (MAVLink, PWM, CAN) and pointing modes (neutral,
     *          ROI, GPS, sysid target).
     */
    AP_Mount camera_mount;
#endif

#if AVOIDANCE_ENABLED
    /**
     * @brief Obstacle avoidance and geofencing
     * 
     * @details Prevents vehicle from entering restricted areas (fences) or
     *          colliding with detected obstacles. Integrates with proximity sensors.
     */
    AC_Avoid avoid;
#endif

#if HAL_RALLY_ENABLED
    /**
     * @brief Rally point management
     * 
     * @details Stores and manages rally points (safe return locations). Used for
     *          emergency return-to-launch alternatives.
     */
    AP_Rally rally;
#endif

#if AP_TERRAIN_AVAILABLE
    /**
     * @brief Terrain database and terrain following
     * 
     * @details Provides terrain height data for terrain following missions and
     *          terrain-relative altitude control. Can use onboard database or
     *          request data from GCS.
     */
    AP_Terrain terrain;
#endif

    /**
     * @brief Attitude control without position system (GPS-denied operation)
     * 
     * @details Stores MAVLink SET_ATTITUDE_TARGET commands for controlling attitude
     *          and depth when GPS or position estimation is unavailable. Allows
     *          external control systems to fly the vehicle using only attitude and
     *          depth (no horizontal position control).
     * 
     * @note Used for advanced external control integration or vision-based navigation.
     */
    struct attitude_no_gps_struct {
        uint32_t last_message_ms;            ///< Timestamp of last received message (milliseconds)
        mavlink_set_attitude_target_t packet; ///< Most recent attitude target message
    };

    attitude_no_gps_struct set_attitude_target_no_gps {0};

    /**
     * @brief Parameter loading and management system
     * 
     * @details Handles loading parameters from EEPROM, setting defaults, and
     *          managing parameter groups. Initializes the var_info[] table that
     *          defines all vehicle parameters.
     */
    AP_Param param_loader;

    /**
     * @brief Pilot yaw control state tracking
     * 
     * @details Tracks pilot heading commands and input timing for yaw control
     *          and terrain failsafe recovery logic.
     */
    uint32_t last_pilot_heading;             ///< Last commanded heading (centidegrees)
    uint32_t last_pilot_yaw_input_ms;        ///< Timestamp of last yaw input (milliseconds)
    uint32_t fs_terrain_recover_start_ms;    ///< Terrain failsafe recovery start time (milliseconds)

    /**
     * @brief Scheduler task table
     * 
     * @details Static table defining all scheduled tasks, their execution rates,
     *          and maximum allowed execution times. Tasks include sensor updates,
     *          control loops, logging, telemetry, and housekeeping functions.
     * 
     * @note Defined in Scheduler.cpp with full task list and timing budgets.
     */
    static const AP_Scheduler::Task scheduler_tasks[];
    
    /**
     * @brief Parameter information table
     * 
     * @details Static table defining all vehicle parameters for AP_Param system.
     *          Maps parameter names, types, storage locations, and default values.
     * 
     * @note Defined in Parameters.cpp. Modifications must maintain storage
     *       compatibility with existing vehicles.
     */
    static const AP_Param::Info var_info[];
    
    /**
     * @brief Binary log message structure definitions
     * 
     * @details Static table defining all log message types, formats, field names,
     *          and units for the dataflash logging system.
     * 
     * @note Defined in Log.cpp. Used for log parsing and analysis tools.
     */
    static const struct LogStructure log_structure[];

    // ========================================================================
    // Private Methods - Scheduler Callbacks and Control Loops
    // ========================================================================
    
    /**
     * @brief Run attitude rate controller
     * 
     * @details Executes rate control loops for roll, pitch, and yaw. Called from
     *          fast loop (400Hz) to generate motor commands from rate targets.
     * 
     * @note Critical timing - must complete within fast loop budget (~2.5ms).
     */
    void run_rate_controller();
    
    /**
     * @brief 50Hz scheduler loop
     * 
     * @details Medium-rate loop for RC input processing, position control updates,
     *          and other 50Hz tasks.
     */
    void fifty_hz_loop();
    
    /**
     * @brief Update battery and compass at 10Hz
     * 
     * @details Reads battery voltage/current and compass heading. Rate-limited
     *          to reduce CPU load while maintaining adequate update rate.
     */
    void update_batt_compass(void);
    
    /**
     * @brief 10Hz logging loop
     * 
     * @details Writes medium-rate log messages (attitude, position, velocity).
     */
    void ten_hz_logging_loop();
    
    /**
     * @brief 25Hz logging loop
     * 
     * @details Writes higher-rate log messages for detailed analysis.
     */
    void twentyfive_hz_logging();
    
    /**
     * @brief Log loop rate performance
     * 
     * @details Records scheduler timing and loop execution statistics for
     *          performance monitoring and debugging.
     */
    void loop_rate_logging();
    
    /**
     * @brief 3Hz slow loop
     * 
     * @details Updates compass, surface/bottom detection, and other slow tasks.
     */
    void three_hz_loop();
    
    /**
     * @brief 1Hz very slow loop
     * 
     * @details Handles logging, statistics, updates to GCS, and housekeeping tasks.
     */
    void one_hz_loop();
    
    /**
     * @brief Update rotation counter for full turn tracking
     * 
     * @details Increments quarter_turn_count to track cumulative heading changes
     *          beyond +/-180 degrees.
     */
    void update_turn_counter();
    
    /**
     * @brief Read AHRS attitude and navigation state
     * 
     * @details Updates attitude (roll, pitch, yaw) from AHRS. Called at fast loop
     *          rate to maintain current vehicle orientation for control loops.
     */
    void read_AHRS(void);
    
    /**
     * @brief Update altitude/depth estimation
     * 
     * @details Fuses barometer (depth sensor) with inertial navigation for vertical
     *          position and velocity estimation.
     */
    void update_altitude();
    
    /**
     * @brief Calculate input smoothing gain
     * 
     * @details Computes dynamic gain for pilot input filtering based on vehicle
     *          state and mode. Reduces control jitter while maintaining responsiveness.
     * 
     * @return Smoothing gain factor (0.0 to 1.0)
     */
    float get_smoothing_gain();
    
    /**
     * @brief Convert pilot stick inputs to desired lean angles
     * 
     * @details Scales raw pilot roll/pitch inputs to desired lean angles, applying
     *          expo curves and angle limits. Used in stabilize and position hold modes.
     * 
     * @param[in]  roll_in   Pilot roll input (-1.0 to 1.0)
     * @param[in]  pitch_in  Pilot pitch input (-1.0 to 1.0)
     * @param[out] roll_out  Desired roll angle (degrees)
     * @param[out] pitch_out Desired pitch angle (degrees)
     * @param[in]  angle_max Maximum allowed lean angle (degrees)
     */
    void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max);
    
    /**
     * @brief Convert pilot stick to desired yaw rate
     * 
     * @details Scales pilot yaw input to yaw rate with expo curve for smooth control.
     * 
     * @param[in] stick_angle Pilot yaw stick input (-4500 to 4500 centidegrees)
     * @return Desired yaw rate (deg/s)
     */
    float get_pilot_desired_yaw_rate(int16_t stick_angle) const;
    
    /**
     * @brief Check for EKF yaw reset and handle accordingly
     * 
     * @details Detects when EKF resets yaw (heading) estimate and updates tracking
     *          to prevent control discontinuities.
     */
    void check_ekf_yaw_reset();
    
    /**
     * @brief Calculate yaw angle to point at Region of Interest
     * 
     * @details Computes heading required to face the ROI waypoint from current position.
     * 
     * @return Yaw angle to ROI (degrees)
     */
    float get_roi_yaw();
    
    /**
     * @brief Calculate yaw angle for look-ahead mode
     * 
     * @details Computes heading in direction of travel for autonomous navigation.
     * 
     * @return Look-ahead yaw angle (degrees)
     */
    float get_look_ahead_yaw();
    
    /**
     * @brief Convert throttle stick to desired climb rate
     * 
     * @details Scales pilot throttle input to vertical velocity target for depth control.
     * 
     * @param[in] throttle_control Pilot throttle input (-1.0 to 1.0)
     * @return Desired climb rate (cm/s, positive = up)
     */
    float get_pilot_desired_climb_rate(float throttle_control);
    
    /**
     * @brief Convert channel input to desired horizontal rate
     * 
     * @details Scales pilot forward/lateral inputs to velocity targets.
     * 
     * @param[in] channel Pointer to RC channel (forward or lateral)
     * @return Desired horizontal rate (cm/s)
     */
    float get_pilot_desired_horizontal_rate(RC_Channel *channel) const;
    
    /**
     * @brief Rotate body frame vector to North-East frame
     * 
     * @details Transforms body-frame (forward/right) velocities to earth-frame
     *          (north/east) using current vehicle heading.
     * 
     * @param[in,out] x Forward (body) / North (earth) component
     * @param[in,out] y Right (body) / East (earth) component
     */
    void rotate_body_frame_to_NE(float &x, float &y);
    
    // ========================================================================
    // Private Methods - Logging System
    // ========================================================================
    
#if HAL_LOGGING_ENABLED
    /**
     * @brief Get logging bitmask parameter (AP_Vehicle override)
     * 
     * @details Returns reference to parameter controlling which log types are enabled.
     * @return Reference to log bitmask parameter
     */
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    
    /**
     * @brief Get log structure definitions (AP_Vehicle override)
     * 
     * @details Returns pointer to array of log message format definitions.
     * @return Pointer to LogStructure array
     */
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    
    /**
     * @brief Get number of log structure entries (AP_Vehicle override)
     * 
     * @details Returns count of log message types defined for ArduSub.
     * @return Number of log structures
     */
    uint8_t get_num_log_structures() const override;

    /**
     * @brief Log control tuning data
     * 
     * @details Writes CTUN message with attitude targets, angle errors, and
     *          control outputs for tuning analysis.
     */
    void Log_Write_Control_Tuning();
    
    /**
     * @brief Log vehicle attitude
     * 
     * @details Writes ATT message with current roll, pitch, yaw and rates.
     */
    void Log_Write_Attitude();
    
    /**
     * @brief Log generic data value (int32_t)
     * 
     * @param[in] id Data field identifier
     * @param[in] value Data value to log
     */
    void Log_Write_Data(LogDataID id, int32_t value);
    
    /**
     * @brief Log generic data value (uint32_t)
     * 
     * @param[in] id Data field identifier
     * @param[in] value Data value to log
     */
    void Log_Write_Data(LogDataID id, uint32_t value);
    
    /**
     * @brief Log generic data value (int16_t)
     * 
     * @param[in] id Data field identifier
     * @param[in] value Data value to log
     */
    void Log_Write_Data(LogDataID id, int16_t value);
    
    /**
     * @brief Log generic data value (uint16_t)
     * 
     * @param[in] id Data field identifier
     * @param[in] value Data value to log
     */
    void Log_Write_Data(LogDataID id, uint16_t value);
    
    /**
     * @brief Log generic data value (float)
     * 
     * @param[in] id Data field identifier
     * @param[in] value Data value to log
     */
    void Log_Write_Data(LogDataID id, float value);
    
    /**
     * @brief Log guided mode target
     * 
     * @details Writes GUID message with position and velocity targets for guided mode.
     * 
     * @param[in] target_type Type of target (position, velocity, etc.)
     * @param[in] pos_target  Position target vector (cm, NED frame)
     * @param[in] vel_target  Velocity target vector (cm/s, NED frame)
     */
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    
    /**
     * @brief Log vehicle startup messages
     * 
     * @details Writes initialization log messages including firmware version,
     *          board type, and configuration parameters.
     */
    void Log_Write_Vehicle_Startup_Messages();
#endif
    
    // ========================================================================
    // Private Methods - Parameter Loading and User Hooks
    // ========================================================================
    
    /**
     * @brief Load all vehicle parameters from EEPROM (AP_Vehicle override)
     * 
     * @details Loads stored parameter values, applies defaults for uninitialized
     *          parameters, and handles parameter format conversions for firmware updates.
     */
    void load_parameters(void) override;
    
    /**
     * @brief User hook called at initialization
     * 
     * @details Empty hook function for user customization without modifying core code.
     */
    void userhook_init();
    
    /**
     * @brief User hook called in fast loop (400Hz)
     * 
     * @details Empty hook function for high-rate user code.
     * @warning Keep execution time minimal to avoid timing overruns.
     */
    void userhook_FastLoop();
    
    /**
     * @brief User hook called at 50Hz
     * 
     * @details Empty hook function for medium-rate user code.
     */
    void userhook_50Hz();
    
    /**
     * @brief User hook called at 10Hz
     * 
     * @details Empty hook function for slow user code.
     */
    void userhook_MediumLoop();
    
    /**
     * @brief User hook called at 3Hz
     * 
     * @details Empty hook function for very slow user code.
     */
    void userhook_SlowLoop();
    
    /**
     * @brief User hook called at 1Hz
     * 
     * @details Empty hook function for infrequent user code.
     */
    void userhook_SuperSlowLoop();
    
    // ========================================================================
    // Private Methods - Home Position Management
    // ========================================================================
    
    /**
     * @brief Update home location from EKF origin
     * 
     * @details Sets home to EKF origin if origin has been set but home hasn't.
     *          Ensures home position is initialized for RTL functionality.
     */
    void update_home_from_EKF();
    
    /**
     * @brief Set home to current location while in flight
     * 
     * @details Updates home position to current location while armed/flying.
     *          Used for dynamic home relocation during mission.
     */
    void set_home_to_current_location_inflight();
    
    /**
     * @brief Set home to current location (AP_Vehicle override)
     * 
     * @details Sets home position to current vehicle location.
     * 
     * @param[in] lock If true, prevents home from being moved automatically
     * @return true if home was set successfully
     */
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Set home to specified location (AP_Vehicle override)
     * 
     * @details Sets home position to provided location coordinates.
     * 
     * @param[in] loc Location to set as home
     * @param[in] lock If true, prevents home from being moved automatically
     * @return true if home was set successfully
     */
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Get relative altitude above home
     * 
     * @details Returns vertical distance above home location.
     * 
     * @return Relative altitude (meters, positive = above home)
     */
    float get_alt_rel() const WARN_IF_UNUSED;
    
    /**
     * @brief Get altitude above mean sea level
     * 
     * @details Returns absolute altitude referenced to MSL.
     * 
     * @return MSL altitude (meters)
     */
    float get_alt_msl() const WARN_IF_UNUSED;
    
    // ========================================================================
    // Private Methods - Mission Management
    // ========================================================================
    
    /**
     * @brief Exit autonomous mission mode
     * 
     * @details Cleans up mission state when leaving auto mode. Stops mission
     *          execution and resets mission tracking variables.
     */
    void exit_mission();
    
    /**
     * @brief Set EKF origin for navigation
     * 
     * @details Initializes Extended Kalman Filter origin to specified location.
     *          Required for position estimation before first GPS fix.
     * 
     * @param[in] loc Origin location
     */
    void set_origin(const Location& loc);
    
    /**
     * @brief Ensure EKF has a valid origin
     * 
     * @details Checks if EKF origin is set, attempts to set it if not.
     * 
     * @return true if EKF has valid origin
     */
    bool ensure_ekf_origin();
    
    /**
     * @brief Verify loiter unlimited command completion
     * 
     * @details Checks if unlimited loiter mission command has completed.
     *          Never completes naturally (pilot must intervene).
     * 
     * @return Always false (infinite loiter)
     */
    bool verify_loiter_unlimited();
    
    /**
     * @brief Verify loiter time command completion
     * 
     * @details Checks if timed loiter mission command duration has elapsed.
     * 
     * @return true if loiter time has expired
     */
    bool verify_loiter_time();
    
    /**
     * @brief Verify delay command completion
     * 
     * @details Checks if mission delay command duration has elapsed.
     * 
     * @return true if delay time has expired
     */
    bool verify_wait_delay();
    
    /**
     * @brief Verify within distance command completion
     * 
     * @details Checks if vehicle has reached within specified distance of target.
     * 
     * @return true if within target distance
     */
    bool verify_within_distance();
    
    /**
     * @brief Verify yaw command completion
     * 
     * @details Checks if vehicle has rotated to target heading.
     * 
     * @return true if yaw target achieved within tolerance
     */
    bool verify_yaw();

    // ========================================================================
    // Private Methods - Failsafe and Safety Systems
    // ========================================================================
    
    /**
     * @brief Check sensor health for failsafe conditions
     * 
     * @details Monitors critical sensors (depth, compass) and triggers sensor
     *          health failsafe if sensors fail in depth-enabled modes.
     * 
     * @warning Called at 3Hz. Sensor failures can trigger immediate failsafe actions.
     */
    void failsafe_sensors_check(void);
    
    /**
     * @brief Check for crash/collision detection
     * 
     * @details Monitors IMU for impact signatures and unusual attitudes that
     *          indicate collision or vehicle crash.
     * 
     * @warning Crash detection triggers immediate disarm or surface action.
     */
    void failsafe_crash_check();
    
    /**
     * @brief Check Extended Kalman Filter health
     * 
     * @details Monitors EKF variance and innovation levels. Triggers failsafe
     *          if position/velocity estimate becomes unreliable.
     * 
     * @warning EKF failure in autonomous modes can cause loss of position control.
     */
    void failsafe_ekf_check(void);
    
    /**
     * @brief Handle battery failsafe action
     * 
     * @details Executes appropriate action (warn, surface, disarm) based on
     *          battery voltage/capacity failsafe level.
     * 
     * @param[in] type_str Description of battery failsafe type
     * @param[in] action Failsafe action to execute (Failsafe_Action enum)
     * 
     * @warning Battery failsafe can cause immediate disarm at critical levels.
     */
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    
    /**
     * @brief Check ground control station communication
     * 
     * @details Monitors GCS heartbeat messages. Triggers failsafe if no GCS
     *          communication received within timeout period.
     */
    void failsafe_gcs_check();
    
    /**
     * @brief Check pilot input (joystick/RC) timeout
     * 
     * @details Monitors pilot control input messages (MANUAL_CONTROL, RC_CHANNELS).
     *          Triggers failsafe if no input received within timeout.
     * 
     * @warning Pilot input loss triggers immediate hold/surface depending on mode.
     */
    void failsafe_pilot_input_check(void);
    
    /**
     * @brief Set all controls to neutral/safe values
     * 
     * @details Zeros pilot inputs and sets safe motor outputs. Used during
     *          failsafe conditions and mode transitions.
     */
    void set_neutral_controls(void);
    
    /**
     * @brief Check terrain data availability
     * 
     * @details Monitors terrain database access. Triggers failsafe if terrain
     *          data unavailable for terrain-following missions.
     */
    void failsafe_terrain_check();
    
    /**
     * @brief Update terrain failsafe status
     * 
     * @details Records terrain data availability and timing for failsafe logic.
     * 
     * @param[in] data_ok true if terrain data is currently available
     */
    void failsafe_terrain_set_status(bool data_ok);
    
    /**
     * @brief Execute terrain failsafe action
     * 
     * @details Triggers when terrain data becomes unavailable during terrain mission.
     *          Executes recovery action (typically hold position or surface).
     */
    void failsafe_terrain_on_event();
    
    /**
     * @brief Enable main loop failsafe monitoring
     * 
     * @details Activates failsafe that triggers if main loop stops executing
     *          (watchdog failsafe). Indicates severe system malfunction.
     * 
     * @warning Should always be enabled during flight. Disabling risks undetected hangs.
     */
    void mainloop_failsafe_enable();
    
    /**
     * @brief Disable main loop failsafe monitoring
     * 
     * @details Deactivates main loop watchdog. Only used during initialization
     *          before scheduler is fully operational.
     */
    void mainloop_failsafe_disable();
    
#if AP_FENCE_ENABLED
    /**
     * @brief Check geofence boundaries
     * 
     * @details Monitors vehicle position relative to configured geofence boundaries.
     *          Triggers fence breach action if vehicle exits allowed area.
     * 
     * @warning Fence breach can cause immediate surface or disarm depending on configuration.
     */
    void fence_check();
    
    /**
     * @brief Asynchronous fence checks (AP_Vehicle override)
     * 
     * @details Performs fence boundary checks that can run outside main loop.
     */
    void fence_checks_async() override;
#endif

    // ========================================================================
    // Private Methods - Flight Mode Management
    // ========================================================================
    
    /**
     * @brief Set flight mode by mode number
     * 
     * @details Changes vehicle to specified flight mode if transition is valid.
     * 
     * @param[in] mode Flight mode to enter
     * @param[in] reason Reason for mode change (logging/diagnostics)
     * @return true if mode change successful
     */
    bool set_mode(Mode::Number mode, ModeReason reason);
    
    /**
     * @brief Set flight mode by integer (AP_Vehicle override)
     * 
     * @details Changes vehicle to specified flight mode (uint8_t wrapper).
     * 
     * @param[in] new_mode Flight mode to enter (as uint8_t)
     * @param[in] reason Reason for mode change
     * @return true if mode change successful
     */
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    
    /**
     * @brief Get current flight mode (AP_Vehicle override)
     * 
     * @details Returns current control mode as integer.
     * 
     * @return Current mode number (uint8_t)
     */
    uint8_t get_mode() const override { return (uint8_t)control_mode; }
    
    /**
     * @brief Update flight mode controllers
     * 
     * @details Calls current mode's run() method to execute mode-specific control logic.
     *          Called from main loop (50Hz).
     */
    void update_flight_mode();
    
    /**
     * @brief Exit old mode and enter new mode
     * 
     * @details Handles cleanup of old mode and initialization of new mode during transitions.
     * 
     * @param[in] old_control_mode Mode being exited
     * @param[in] new_control_mode Mode being entered
     */
    void exit_mode(Mode::Number old_control_mode, Mode::Number new_control_mode);
    
    /**
     * @brief Notify GCS and onboard systems of mode change
     * 
     * @details Sends mode change messages to ground control and triggers LED/sound notifications.
     */
    void notify_flight_mode();
    
    // ========================================================================
    // Private Methods - Sensor Reading and State Detection
    // ========================================================================
    
    /**
     * @brief Read IMU (inertial measurement unit)
     * 
     * @details Updates accelerometer and gyroscope readings from IMU.
     *          Called at fast loop rate (400Hz).
     */
    void read_inertia();
    
    /**
     * @brief Update surface and bottom contact detection
     * 
     * @details Analyzes sensor data to detect if vehicle is at surface or
     *          touching bottom. Used for automatic descent/ascent limits.
     * 
     * @note Uses depth sensor, IMU, and motor output to detect contact.
     */
    void update_surface_and_bottom_detector();
    
    /**
     * @brief Set surface contact flag
     * 
     * @details Updates vehicle state to indicate surface contact detected.
     * 
     * @param[in] at_surface true if vehicle is at water surface
     */
    void set_surfaced(bool at_surface);
    
    /**
     * @brief Set bottom contact flag
     * 
     * @details Updates vehicle state to indicate bottom contact detected.
     * 
     * @param[in] at_bottom true if vehicle is touching bottom
     */
    void set_bottomed(bool at_bottom);
    
    /**
     * @brief Output motor commands to hardware
     * 
     * @details Sends calculated motor PWM values to motor outputs. Called at
     *          fast loop rate after control loop execution.
     * 
     * @note Motor outputs are safety-checked before being sent to ESCs.
     */
    void motors_output();
    
    // ========================================================================
    // Private Methods - RC and Joystick Initialization
    // ========================================================================
    
    /**
     * @brief Initialize RC input subsystem
     * 
     * @details Sets up RC receiver input processing, channel mapping, and failsafe settings.
     */
    void init_rc_in();
    
    /**
     * @brief Initialize RC output subsystem
     * 
     * @details Sets up servo/motor output channels, PWM frequencies, and output ranges.
     */
    void init_rc_out();
    
#if AP_SUB_RC_ENABLED
    /**
     * @brief RC input processing loop
     * 
     * @details Reads and processes traditional RC receiver input. Used when vehicle
     *          is controlled via RC transmitter instead of joystick/MAVLink.
     * 
     * @note Only compiled if AP_SUB_RC_ENABLED is defined.
     */
    void rc_loop();
    
    /**
     * @brief Read RC receiver channels
     * 
     * @details Reads PWM values from RC receiver and updates channel objects.
     */
    void read_radio();
    
    /**
     * @brief Timestamp of last valid RC input
     * 
     * @details Used for RC failsafe detection. If no update within timeout,
     *          RC failsafe is triggered.
     */
    uint32_t last_radio_update_ms;
    
    /**
     * @brief Set RC radio failsafe state
     * 
     * @details Activates or clears RC radio failsafe flag.
     * 
     * @param[in] b true to activate failsafe, false to clear
     */
    void set_failsafe_radio(bool b);
    
    /**
     * @brief Process throttle channel and check for failsafe
     * 
     * @details Reads throttle PWM value and triggers failsafe if value indicates
     *          RC signal loss (throttle below failsafe threshold).
     * 
     * @param[in] throttle_pwm Throttle channel PWM value (microseconds)
     */
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    
    /**
     * @brief Handle RC failsafe clearing
     * 
     * @details Actions to take when RC signal is restored after failsafe.
     */
    void failsafe_radio_off_event();
    
    /**
     * @brief Handle RC failsafe triggering
     * 
     * @details Actions to take when RC signal lost (hold position, surface, disarm).
     * 
     * @warning RC failsafe can cause immediate mode change or disarm.
     */
    void failsafe_radio_on_event();
#endif

    // ========================================================================
    // Private Methods - Joystick and Manual Control
    // ========================================================================
    
    /**
     * @brief Enable motor outputs (safety)
     * 
     * @details Arms motor outputs allowing them to spin. Safety-critical function
     *          that checks all arming prerequisites.
     * 
     * @warning Never call without proper arming checks.
     */
    void enable_motor_output();
    
    /**
     * @brief Initialize joystick/gamepad system
     * 
     * @details Sets up joystick button mapping and control input processing.
     */
    void init_joystick();
    
    /**
     * @brief Transform MAVLink MANUAL_CONTROL to RC override values
     * 
     * @details Converts joystick inputs from MAVLink MANUAL_CONTROL message into
     *          RC channel override values for vehicle control.
     * 
     * @param[in] x Roll input (-1000 to 1000)
     * @param[in] y Pitch input (-1000 to 1000)
     * @param[in] z Throttle/depth input (-1000 to 1000)
     * @param[in] r Yaw input (-1000 to 1000)
     * @param[in] buttons Button bitmask (first 16 buttons)
     * @param[in] buttons2 Button bitmask (buttons 17-32)
     * @param[in] enabled_extensions Bitmask of enabled extension axes
     * @param[in] s Lateral (strafe) input
     * @param[in] t Forward input
     * @param[in] aux1 Auxiliary axis 1
     * @param[in] aux2 Auxiliary axis 2
     * @param[in] aux3 Auxiliary axis 3
     * @param[in] aux4 Auxiliary axis 4
     * @param[in] aux5 Auxiliary axis 5
     * @param[in] aux6 Auxiliary axis 6
     */
    void transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions,
            int16_t s,
            int16_t t,
            int16_t aux1,
            int16_t aux2,
            int16_t aux3,
            int16_t aux4,
            int16_t aux5,
            int16_t aux6);
    
    /**
     * @brief Handle joystick button press
     * 
     * @details Executes assigned function when joystick button is pressed
     *          (arm/disarm, mode change, lights, camera, gain adjustment, etc.).
     * 
     * @param[in] button Button number (0-31)
     * @param[in] shift true if shift modifier active
     * @param[in] held true if button is held (long press)
     */
    void handle_jsbutton_press(uint8_t button,bool shift=false,bool held=false);
    
    /**
     * @brief Handle joystick button release
     * 
     * @details Executes actions on button release (if different from press action).
     * 
     * @param[in] button Button number (0-31)
     * @param[in] shift true if shift modifier active
     */
    void handle_jsbutton_release(uint8_t button, bool shift);
    
    /**
     * @brief Get JSButton object by index
     * 
     * @details Returns pointer to JSButton for querying/modifying button assignment.
     * 
     * @param[in] index Button index
     * @return Pointer to JSButton object
     */
    JSButton* get_button(uint8_t index);
    
    /**
     * @brief Reset joystick button assignments to defaults
     * 
     * @details Loads default button mapping configuration.
     */
    void default_js_buttons(void);
    
    /**
     * @brief Clear input hold state
     * 
     * @details Resets input hold flags used for momentary button actions.
     */
    void clear_input_hold();
    
    // ========================================================================
    // Private Methods - Sensor Reading and Updates
    // ========================================================================
    
    /**
     * @brief Read barometer (depth sensor)
     * 
     * @details Updates depth/pressure readings from barometer. Underwater vehicles
     *          use barometer as depth sensor by measuring water pressure.
     * 
     * @note Depth in meters = (pressure - surface_pressure) / (water_density * g)
     */
    void read_barometer(void);
    
    /**
     * @brief Initialize rangefinder subsystem
     * 
     * @details Sets up rangefinder/sonar for terrain following and obstacle detection.
     */
    void init_rangefinder(void);
    
    /**
     * @brief Read rangefinder distance measurements
     * 
     * @details Updates rangefinder readings for terrain following and bottom tracking.
     */
    void read_rangefinder(void);
    
#if AP_TERRAIN_AVAILABLE
    /**
     * @brief Update terrain database
     * 
     * @details Requests terrain data from GCS or onboard database for terrain-following missions.
     */
    void terrain_update();
    
    /**
     * @brief Log terrain data
     * 
     * @details Writes terrain height and status to dataflash log.
     */
    void terrain_logging();
#endif

    // ========================================================================
    // Private Methods - Initialization and Scheduler
    // ========================================================================
    
    /**
     * @brief Initialize ArduPilot vehicle (AP_Vehicle override)
     * 
     * @details Main initialization function called at startup. Initializes all
     *          subsystems, loads parameters, calibrates sensors, and prepares
     *          vehicle for operation.
     * 
     * @note This is the vehicle's main entry point after HAL initialization.
     */
    void init_ardupilot() override;
    
    /**
     * @brief Get scheduler task table (AP_Vehicle override)
     * 
     * @details Returns pointer to scheduler task array and task count for
     *          AP_Scheduler to execute.
     * 
     * @param[out] tasks Reference to task pointer
     * @param[out] task_count Reference to task count
     * @param[out] log_bit Reference to log bitmask
     */
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    
    /**
     * @brief Startup IMU calibration on ground/surface
     * 
     * @details Performs IMU (accelerometer/gyroscope) calibration at startup.
     *          Vehicle must be stationary for accurate calibration.
     * 
     * @warning Do not move vehicle during startup calibration.
     */
    void startup_INS_ground();
    
    // ========================================================================
    // Private Methods - Position Estimation Checks
    // ========================================================================
    
    /**
     * @brief Check if position estimate is available and healthy
     * 
     * @details Returns true if vehicle has valid position estimate from GPS,
     *          optical flow, or other positioning system.
     * 
     * @return true if position is available and reliable
     */
    bool position_ok();
    
    /**
     * @brief Check if EKF position estimate is healthy
     * 
     * @details Returns true if Extended Kalman Filter has converged position estimate
     *          with acceptable variance levels.
     * 
     * @return true if EKF position is reliable
     */
    bool ekf_position_ok();
    
    /**
     * @brief Check if optical flow position estimate is healthy
     * 
     * @details Returns true if optical flow sensor is providing valid position updates.
     * 
     * @return true if optical flow position is reliable
     */
    bool optflow_position_ok();
    
    /**
     * @brief Check if logging is enabled for specific message type
     * 
     * @details Tests log bitmask to determine if specified log type should be written.
     * 
     * @param[in] mask Log type bitmask to check
     * @return true if logging is enabled for this type
     */
    bool should_log(uint32_t mask);
    
    // ========================================================================
    // Private Methods - Mission Command Execution
    // ========================================================================
    
    /**
     * @brief Start execution of mission command
     * 
     * @details Initializes mission command and sets up navigation targets.
     * 
     * @param[in] cmd Mission command to start
     * @return true if command started successfully
     */
    bool start_command(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Verify mission command completion
     * 
     * @details Checks if current mission command has completed and vehicle is
     *          ready to advance to next command.
     * 
     * @param[in] cmd Mission command to verify
     * @return true if command is complete
     */
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Verify mission command via callback
     * 
     * @details Helper function for mission command verification.
     * 
     * @param[in] cmd Mission command to verify
     * @return true if command is complete
     */
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);

    // ========================================================================
    // Private Methods - Mission Navigation Commands (do_*)
    // ========================================================================
    
    /**
     * @brief Execute guided mode command
     * 
     * @param[in] cmd Guided mode mission command
     * @return true if command accepted
     */
    bool do_guided(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute waypoint navigation command
     * 
     * @details Sets target waypoint for autonomous navigation.
     * 
     * @param[in] cmd Waypoint mission command with target lat/lon/depth
     */
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute surface navigation command
     * 
     * @details Commands vehicle to ascend to water surface.
     * 
     * @param[in] cmd Surface mission command
     */
    void do_surface(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute Return to Launch
     * 
     * @details Commands vehicle to return to home/launch location.
     */
    void do_RTL(void);
    
    /**
     * @brief Execute unlimited loiter command
     * 
     * @details Commands vehicle to hold position indefinitely at target location.
     * 
     * @param[in] cmd Loiter unlimited mission command
     */
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute circle navigation command
     * 
     * @details Commands vehicle to fly circular pattern around center point.
     * 
     * @param[in] cmd Circle mission command with center and radius
     */
    void do_circle(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute timed loiter command
     * 
     * @details Commands vehicle to hold position for specified duration.
     * 
     * @param[in] cmd Loiter time mission command with duration
     */
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    
#if NAV_GUIDED
    /**
     * @brief Enable guided navigation mode
     * 
     * @details Enables external control via MAVLink guided messages.
     * 
     * @param[in] cmd Nav guided enable mission command
     */
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Set guided mode limits
     * 
     * @details Configures timeout and altitude limits for guided mode.
     * 
     * @param[in] cmd Guided limits mission command
     */
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    
    /**
     * @brief Execute navigation delay command
     * 
     * @details Delays mission progression for specified time or until condition met.
     * 
     * @param[in] cmd Nav delay mission command
     */
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute wait/delay command
     * 
     * @details Pauses mission execution for specified duration.
     * 
     * @param[in] cmd Wait delay mission command with duration
     */
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute within distance condition command
     * 
     * @details Sets condition that must be met before proceeding (distance to target).
     * 
     * @param[in] cmd Within distance mission command
     */
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute yaw/heading command
     * 
     * @details Commands vehicle to rotate to target heading.
     * 
     * @param[in] cmd Yaw mission command with target angle
     */
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute change speed command
     * 
     * @details Changes vehicle speed for subsequent waypoints.
     * 
     * @param[in] cmd Change speed mission command
     */
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute set home command
     * 
     * @details Sets new home location (for RTL destination).
     * 
     * @param[in] cmd Set home mission command
     */
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute Region of Interest command
     * 
     * @details Commands vehicle/camera to point at specified location.
     * 
     * @param[in] cmd ROI mission command with target location
     */
    void do_roi(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Execute mount/gimbal control command
     * 
     * @details Commands gimbal to specific orientation.
     * 
     * @param[in] cmd Mount control mission command
     */
    void do_mount_control(const AP_Mission::Mission_Command& cmd);

    // ========================================================================
    // Private Methods - Mission Command Verification (verify_*)
    // ========================================================================
    
    /**
     * @brief Verify waypoint navigation completion
     * 
     * @param[in] cmd Waypoint command being verified
     * @return true if waypoint reached
     */
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Verify surface command completion
     * 
     * @param[in] cmd Surface command being verified
     * @return true if surface reached
     */
    bool verify_surface(const AP_Mission::Mission_Command& cmd);
    
    /**
     * @brief Verify RTL completion
     * 
     * @return true if returned to home location
     */
    bool verify_RTL(void);
    
    /**
     * @brief Verify circle navigation completion
     * 
     * @param[in] cmd Circle command being verified
     * @return true if circle command complete
     */
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    
#if NAV_GUIDED
    /**
     * @brief Verify guided mode enable completion
     * 
     * @param[in] cmd Guided enable command being verified
     * @return true if guided mode active
     */
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    
    /**
     * @brief Verify navigation delay completion
     * 
     * @param[in] cmd Nav delay command being verified
     * @return true if delay expired or condition met
     */
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    // ========================================================================
    // Private Methods - Additional Failsafe Checks
    // ========================================================================
    
    /**
     * @brief Check for water leak detection
     * 
     * @details Monitors leak detector sensors. Triggers failsafe if water ingress detected.
     * 
     * @warning Leak detection triggers immediate surface action to prevent vehicle loss.
     */
    void failsafe_leak_check();
    
    /**
     * @brief Check internal pressure
     * 
     * @details Monitors pressure inside sealed enclosure. Triggers failsafe if
     *          pressure exceeds safe threshold (indicates seal compromise).
     */
    void failsafe_internal_pressure_check();
    
    /**
     * @brief Check internal temperature
     * 
     * @details Monitors temperature inside electronics enclosure. Triggers failsafe
     *          if temperature exceeds safe operating limits.
     */
    void failsafe_internal_temperature_check();

    /**
     * @brief Execute terrain failsafe action
     * 
     * @details Handles terrain data loss during terrain-following mission.
     *          Typically surfaces or holds position until data available.
     */
    void failsafe_terrain_act(void);

    // ========================================================================
    // Private Methods - Control Translation and Coordinate Conversion
    // ========================================================================

    /**
     * @brief Translate waypoint navigation to roll/pitch commands
     * 
     * @details Converts WPNav desired lateral/forward velocities to vehicle
     *          roll/pitch commands for 6-DOF underwater control.
     * 
     * @param[out] lateral_out Lateral (right) output command
     * @param[out] forward_out Forward output command
     */
    void translate_wpnav_rp(float &lateral_out, float &forward_out);
    
    /**
     * @brief Translate circle navigation to roll/pitch commands
     * 
     * @details Converts circle nav desired velocities to vehicle commands.
     * 
     * @param[out] lateral_out Lateral (right) output command
     * @param[out] forward_out Forward output command
     */
    void translate_circle_nav_rp(float &lateral_out, float &forward_out);
    
    /**
     * @brief Translate position control to roll/pitch commands
     * 
     * @details Converts PosControl desired velocities to vehicle commands.
     * 
     * @param[out] lateral_out Lateral (right) output command
     * @param[out] forward_out Forward output command
     */
    void translate_pos_control_rp(float &lateral_out, float &forward_out);

    // ========================================================================
    // Private Methods - Statistics, Parameters, and Testing
    // ========================================================================

    /**
     * @brief Update flight statistics
     * 
     * @details Accumulates flight time, distance traveled, and other operational
     *          statistics for logging and reporting.
     */
    void stats_update();

    /**
     * @brief Get pilot commanded descent speed
     * 
     * @details Returns maximum descent (down) speed based on pilot configuration.
     * 
     * @return Pilot descent speed (cm/s)
     */
    uint16_t get_pilot_speed_dn() const;

    /**
     * @brief Convert parameters from older ArduSub versions
     * 
     * @details Handles parameter migration when loading configuration from
     *          previous firmware versions with different parameter structures.
     */
    void convert_old_parameters(void);
    
    /**
     * @brief Handle MAVLink motor test command
     * 
     * @details Processes MAVLink COMMAND_INT for motor testing.
     * 
     * @param[in] command MAVLink motor test command
     * @return true if command accepted
     * 
     * @warning Motor testing can cause unexpected vehicle motion. Use with caution.
     */
    bool handle_do_motor_test(mavlink_command_int_t command);
    
    /**
     * @brief Initialize motor test sequence
     * 
     * @details Prepares vehicle for motor testing by checking arming and safety conditions.
     * 
     * @return true if motor test can proceed
     * 
     * @warning Ensure vehicle is secured before motor testing.
     */
    bool init_motor_test();
    
    /**
     * @brief Verify motor test completion
     * 
     * @details Checks if motor test duration has elapsed.
     * 
     * @return true if motor test complete
     */
    bool verify_motor_test();

    /** @brief Timestamp of last failed motor test (ms) */
    uint32_t last_do_motor_test_fail_ms = 0;
    
    /** @brief Timestamp of last motor test (ms) */
    uint32_t last_do_motor_test_ms = 0;

    /**
     * @brief Check barometer health for control
     * 
     * @details Verifies that barometer (depth sensor) is healthy and providing
     *          valid data for depth control.
     * 
     * @return true if barometer OK for control
     */
    bool control_check_barometer();

    // ========================================================================
    // Private Methods - Waypoint Information Helpers (AP_Vehicle overrides)
    // ========================================================================
    
    /**
     * @brief Get distance to active waypoint
     * 
     * @details Returns horizontal distance to current mission waypoint.
     * 
     * @param[out] distance Distance to waypoint (meters)
     * @return true if waypoint active and distance valid
     */
    bool get_wp_distance_m(float &distance) const override;
    
    /**
     * @brief Get bearing to active waypoint
     * 
     * @details Returns compass bearing from current position to waypoint.
     * 
     * @param[out] bearing Bearing to waypoint (degrees, 0=North)
     * @return true if waypoint active and bearing valid
     */
    bool get_wp_bearing_deg(float &bearing) const override;
    
    /**
     * @brief Get cross-track error to waypoint path
     * 
     * @details Returns perpendicular distance from vehicle to intended path
     *          between waypoints.
     * 
     * @param[out] xtrack_error Cross-track error (meters, positive = right of path)
     * @return true if path active and error valid
     */
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;

    // ========================================================================
    // Private Types - Failsafe Actions
    // ========================================================================
    
    /**
     * @enum Failsafe_Action
     * @brief Failsafe response actions for underwater vehicle
     * 
     * @details Defines escalating failsafe responses when system errors detected.
     *          Actions prioritized from most to least severe.
     */
    enum Failsafe_Action {
        Failsafe_Action_None    = 0,  ///< No action, continue normal operation
        Failsafe_Action_Warn    = 1,  ///< Log warning, notify pilot, continue flight
        Failsafe_Action_Disarm  = 2,  ///< Immediate motor disarm (highest priority)
        Failsafe_Action_Surface = 3   ///< Ascend to surface for recovery
    };

    /**
     * @brief Failsafe action priority list (most to least critical)
     * 
     * @details Ordered array defining which failsafe actions take precedence
     *          when multiple failures occur simultaneously. Terminated with -1 sentinel.
     */
    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Disarm,
                                                      Failsafe_Action_Surface,
                                                      Failsafe_Action_Warn,
                                                      Failsafe_Action_None,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };

    /** @brief Compile-time check that priority list has sentinel terminator */
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // ========================================================================
    // Private Methods - Flight Mode Management
    // ========================================================================

    /**
     * @brief Get Mode object from mode number
     * 
     * @details Returns pointer to Mode object for given mode number.
     * 
     * @param[in] num Mode number (from Mode::Number enum)
     * @return Pointer to Mode object, or nullptr if invalid
     */
    Mode *mode_from_mode_num(const Mode::Number num);
    
    /**
     * @brief Exit current mode and enter new mode
     * 
     * @details Handles mode transition cleanup and initialization.
     * 
     * @param[in,out] old_flightmode Pointer to mode being exited
     * @param[in,out] new_flightmode Pointer to mode being entered
     */
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

    // ========================================================================
    // Private Members - Flight Mode Instances
    // ========================================================================
    
    /** @brief Pointer to current active flight mode */
    Mode *flightmode;
    
    /** @brief Manual mode instance - direct pilot control, no stabilization */
    ModeManual mode_manual;
    
    /** @brief Stabilize mode instance - attitude stabilization with pilot input */
    ModeStabilize mode_stabilize;
    
    /** @brief Acro mode instance - rate control for advanced maneuvers */
    ModeAcro mode_acro;
    
    /** @brief Altitude hold mode instance - maintain depth with pilot horizontal control */
    ModeAlthold mode_althold;
    
    /** @brief Auto mode instance - execute mission waypoints autonomously */
    ModeAuto mode_auto;
    
    /** @brief Guided mode instance - external control via MAVLink */
    ModeGuided mode_guided;
    
    /** @brief Position hold mode instance - maintain 3D position */
    ModePoshold mode_poshold;
    
    /** @brief Circle mode instance - fly circular pattern */
    ModeCircle mode_circle;
    
    /** @brief Surface mode instance - ascend to water surface */
    ModeSurface mode_surface;
    
    /** @brief Motor detect mode instance - identify motor directions */
    ModeMotordetect mode_motordetect;
    
    /** @brief Surface tracking mode instance - maintain distance from surface using rangefinder */
    ModeSurftrak mode_surftrak;

    /** @brief Auto mode sub-mode - controls which auto controller is active (WP, RTL, Loiter, etc.) */
    AutoSubMode auto_mode;
    
    /** @brief Guided mode sub-mode - controls guided behavior type */
    GuidedSubMode guided_mode;

#if AP_SCRIPTING_ENABLED
    /** @brief Script button instances for Lua scripting interface (4 buttons) */
    ScriptButton script_buttons[4];
#endif // AP_SCRIPTING_ENABLED

// ============================================================================
// PUBLIC INTERFACE
// ============================================================================
public:
    
    /**
     * @brief Check all failsafe conditions in main loop
     * 
     * @details Monitors all failsafe sensors and triggers appropriate actions
     *          when failures detected. Called from main scheduler loop.
     * 
     * @warning Critical safety function - monitors battery, sensors, GCS, pilot input
     */
    void mainloop_failsafe_check();
    
    /**
     * @brief Check if rangefinder altitude is valid
     * 
     * @details Verifies rangefinder is healthy and providing usable altitude data.
     * 
     * @return true if rangefinder altitude can be used for control
     */
    bool rangefinder_alt_ok() const WARN_IF_UNUSED;

    // ========================================================================
    // Public Members - Singleton Pattern
    // ========================================================================
    
    /**
     * @brief Singleton instance pointer
     * 
     * @details Static pointer to the single Sub instance. Used for global
     *          access via get_singleton().
     * 
     * @note Only one Sub instance exists per vehicle. Initialized in Sub.cpp.
     */
    static Sub *_singleton;

    /**
     * @brief Get singleton instance
     * 
     * @details Provides global access to the Sub vehicle instance.
     * 
     * @return Pointer to the Sub singleton instance
     * 
     * @note Commonly accessed via global "sub" reference: extern Sub sub;
     */
    static Sub *get_singleton() {
        return _singleton;
    }

#if AP_SCRIPTING_ENABLED
    // ========================================================================
    // Public Methods - Lua Scripting Interface
    // ========================================================================
    
    /**
     * @brief Check if script button is pressed
     * 
     * @details Allows Lua scripts to query button state.
     * 
     * @param[in] index Button index (1-4, not 0-indexed)
     * @return true if button currently pressed
     * 
     * @note Index is 1-based for Lua compatibility (1, 2, 3, 4)
     */
    bool is_button_pressed(uint8_t index);

    /**
     * @brief Get and clear button press count
     * 
     * @details Returns number of times button was pressed since last call,
     *          then resets counter. For detecting button clicks in scripts.
     * 
     * @param[in] index Button index (1-4, not 0-indexed)
     * @return Number of button presses since last call
     * 
     * @note Index is 1-based for Lua compatibility (1, 2, 3, 4)
     */
    uint8_t get_and_clear_button_count(uint8_t index);

#if AP_RANGEFINDER_ENABLED
    /**
     * @brief Get surface tracking rangefinder target distance
     * 
     * @details Returns target distance from surface in surface tracking mode.
     * 
     * @return Target distance from surface (cm)
     */
    float get_rangefinder_target_cm() const WARN_IF_UNUSED { return mode_surftrak.get_rangefinder_target_cm(); }
    
    /**
     * @brief Set surface tracking rangefinder target distance
     * 
     * @details Sets desired distance from surface for surface tracking mode.
     * 
     * @param[in] new_target_cm New target distance (cm)
     * @return true if target set successfully
     */
    bool set_rangefinder_target_cm(float new_target_cm) { return mode_surftrak.set_rangefinder_target_cm(new_target_cm); }
#endif // AP_RANGEFINDER_ENABLED
#endif // AP_SCRIPTING_ENABLED
};

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================

/**
 * @brief Hardware Abstraction Layer reference
 * 
 * @details Global reference to HAL providing platform-independent access to
 *          hardware (UART, SPI, I2C, GPIO, timers, etc.). Defined by AP_HAL
 *          implementation for each platform (ChibiOS, Linux, ESP32, SITL).
 */
extern const AP_HAL::HAL& hal;

/**
 * @brief Global Sub vehicle instance
 * 
 * @details The single ArduSub vehicle instance accessible throughout the codebase.
 *          Initialized in Sub.cpp. Use this reference for global access rather than
 *          Sub::get_singleton() for cleaner code.
 * 
 * @note This is the primary interface to the vehicle - all vehicle state and
 *       control flows through this instance.
 */
extern Sub sub;
