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
 * @file Copter.h
 * @brief Main header file for ArduCopter multicopter vehicle implementation
 * 
 * @details This file defines the Copter class, which is the central vehicle class for all multicopter
 *          variants in ArduPilot (multirotors, helicopters, coaxial copters). The Copter class:
 *          
 *          - Inherits from AP_Vehicle base class providing common vehicle functionality
 *          - Implements the singleton pattern, accessed globally via the 'copter' object
 *          - Integrates all major subsystems: motors, attitude/position control, sensors, navigation, GCS
 *          - Manages the complete flight control system through a scheduler-based architecture
 *          - Supports multiple flight modes (Stabilize, AltHold, Loiter, Auto, RTL, etc.)
 *          - Handles safety systems including arming checks, failsafes, geofencing, and crash detection
 *          
 *          Architecture Overview:
 *          - Main loop runs at high rate (typically 400Hz) calling scheduler tasks
 *          - Scheduler executes tasks at different frequencies based on priority and timing requirements
 *          - Flight modes implement vehicle behavior through Mode class hierarchy
 *          - Controllers (attitude, position, waypoint nav) execute through library integrations
 *          - Sensor data flows through AHRS and EKF for state estimation
 *          - Motor outputs calculated and sent to ESCs/servos at main loop rate
 *          
 *          Key Integrations:
 *          - AP_Motors: Motor mixing and output for various frame configurations
 *          - AC_AttitudeControl: Inner loop attitude rate and angle control
 *          - AC_PosControl: Outer loop position and velocity control
 *          - AC_WPNav: Waypoint navigation and trajectory generation
 *          - AP_AHRS: Attitude and heading estimation (DCM or EKF backend)
 *          - AP_NavEKF: Extended Kalman Filter for position/velocity estimation
 *          - GCS_MAVLink: Ground control station communication
 *          - AP_Mission: Autonomous mission management
 *          
 * @note This file is specific to multicopter vehicles. Fixed-wing uses ArduPlane, ground vehicles use Rover.
 * @warning Safety-critical flight control system - modifications require thorough testing in SITL and on bench before flight
 * 
 * @see AP_Vehicle for base vehicle class
 * @see Mode for flight mode base class and mode implementations
 * @see Parameters.h for vehicle configuration parameters
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
 
#pragma once

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include <AP_HAL/AP_HAL.h>

// Common dependencies
#include <AP_Common/AP_Common.h>            // Common definitions and utility routines for the ArduPilot libraries
#include <AP_Common/Location.h>             // Library having the implementation of location class         
#include <AP_Param/AP_Param.h>              // A system for managing and storing variables that are of general interest to the system.
#include <StorageManager/StorageManager.h>  // library for Management for hal.storage to allow for backwards compatible mapping of storage offsets to available storage

// Application dependencies
#include <AP_Logger/AP_Logger.h>            // ArduPilot Mega Flash Memory Library
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_AccelCal/AP_AccelCal.h>        // interface and maths for accelerometer calibration
#include <AP_InertialSensor/AP_InertialSensor.h>                // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS/AP_AHRS.h>                                    // AHRS (Attitude Heading Reference System) interface library for ArduPilot
#include <AP_Mission/AP_Mission.h>                              // Mission command library
#include <AP_Mission/AP_Mission_ChangeDetector.h>               // Mission command change detection library
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>        // Attitude control library
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>   // 6DoF Attitude control library
#include <AC_AttitudeControl/AC_AttitudeControl_Heli.h>         // Attitude control library for traditional helicopter
#include <AC_AttitudeControl/AC_PosControl.h>                   // Position control library
#include <AC_AttitudeControl/AC_CommandModel.h>                 // Command model library
#include <AP_Motors/AP_Motors.h>            // AP Motors library
#include <Filter/Filter.h>                  // Filter library
#include <AP_Vehicle/AP_Vehicle.h>          // needed for AHRS build
#include <AC_WPNav/AC_WPNav.h>              // ArduCopter waypoint navigation library
#include <AC_WPNav/AC_Loiter.h>             // ArduCopter Loiter Mode Library
#include <AC_WPNav/AC_Circle.h>             // circle navigation library
#include <AP_Declination/AP_Declination.h>  // ArduPilot Mega Declination Helper Library
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <AP_BattMonitor/AP_BattMonitor.h>  // Battery monitor library
#include <AP_LandingGear/AP_LandingGear.h>  // Landing Gear library
#include <AC_InputManager/AC_InputManager.h>        // Pilot input handling library
#include <AC_InputManager/AC_InputManager_Heli.h>   // Heli specific pilot input handling library
#include <AP_Arming/AP_Arming.h>            // ArduPilot motor arming library
#include <AP_SmartRTL/AP_SmartRTL.h>        // ArduPilot Smart Return To Launch Mode (SRTL) library
#include <AP_TempCalibration/AP_TempCalibration.h>  // temperature calibration library
#include <AC_AutoTune/AC_AutoTune_Multi.h>  // ArduCopter autotune library. support for autotune of multirotors.
#include <AC_AutoTune/AC_AutoTune_Heli.h>   // ArduCopter autotune library. support for autotune of helicopters.
#include <AP_Parachute/AP_Parachute.h>      // ArduPilot parachute release library
#include <AC_Sprayer/AC_Sprayer.h>          // Crop sprayer library
#include <AP_Avoidance/AP_Avoidance.h>      // "ADSB" avoidance library
#include <AP_ADSB/AP_ADSB.h>                // ADS-B RF based collision avoidance module library
#include <AP_Proximity/AP_Proximity.h>      // ArduPilot proximity sensor library
#include <AC_PrecLand/AC_PrecLand_config.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Winch/AP_Winch_config.h>
#include <AP_SurfaceDistance/AP_SurfaceDistance.h>

// Configuration
#include "defines.h"
#include "config.h"

#if FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#else
 #define MOTOR_CLASS AP_MotorsMulticopter
#endif

#if MODE_AUTOROTATE_ENABLED
 #include <AC_Autorotation/AC_Autorotation.h> // Autorotation controllers
#endif

#include "RC_Channel_Copter.h"         // RC Channel Library

#include "GCS_MAVLink_Copter.h"
#include "GCS_Copter.h"
#include "AP_Rally.h"           // Rally point library
#include "AP_Arming_Copter.h"

#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Copter.h"
#endif

#include <AP_Beacon/AP_Beacon_config.h>
#if AP_BEACON_ENABLED
 #include <AP_Beacon/AP_Beacon.h>
#endif

#if AP_AVOIDANCE_ENABLED
 #include <AC_Avoidance/AC_Avoid.h>
#endif
#if AP_OAPATHPLANNER_ENABLED
 #include <AC_WPNav/AC_WPNav_OA.h>
 #include <AC_Avoidance/AP_OAPathPlanner.h>
#endif
#if AC_PRECLAND_ENABLED
 # include <AC_PrecLand/AC_PrecLand.h>
 # include <AC_PrecLand/AC_PrecLand_StateMachine.h>
#endif
#if MODE_FOLLOW_ENABLED
 # include <AP_Follow/AP_Follow.h>
#endif
#if AP_TERRAIN_AVAILABLE
 # include <AP_Terrain/AP_Terrain.h>
#endif
#if AP_RANGEFINDER_ENABLED
 # include <AP_RangeFinder/AP_RangeFinder.h>
#endif

#include <AP_Mount/AP_Mount.h>

#include <AP_Camera/AP_Camera.h>

#if HAL_BUTTON_ENABLED
 # include <AP_Button/AP_Button.h>
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
 #include <AP_OSD/AP_OSD.h>
#endif

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
 # include "afs_copter.h"
#endif
#if TOY_MODE_ENABLED
 # include "toy_mode.h"
#endif
#if AP_WINCH_ENABLED
 # include <AP_Winch/AP_Winch.h>
#endif
#include <AP_RPM/AP_RPM.h>

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#if AC_CUSTOMCONTROL_MULTI_ENABLED
#include <AC_CustomControl/AC_CustomControl.h>                  // Custom control library
#endif

#if AP_AVOIDANCE_ENABLED && !AP_FENCE_ENABLED
  #error AC_Avoidance relies on AP_FENCE_ENABLED which is disabled
#endif

#if AP_OAPATHPLANNER_ENABLED && !AP_FENCE_ENABLED
  #error AP_OAPathPlanner relies on AP_FENCE_ENABLED which is disabled
#endif

#if MODE_AUTOROTATE_ENABLED && !AP_RPM_ENABLED
  #error AC_Autorotation relies on AP_RPM_ENABLED which is disabled
#endif

#if HAL_ADSB_ENABLED
#include "avoidance_adsb.h"
#endif
// Local modules
#include "Parameters.h"
#if USER_PARAMS_ENABLED
#include "UserParameters.h"
#endif
#include "mode.h"

/**
 * @class Copter
 * @brief Main vehicle class for multicopter flight control in ArduPilot
 * 
 * @details The Copter class is the central orchestrator for all multicopter variants including:
 *          - Standard multirotors (quad, hex, octo, etc.)
 *          - Traditional helicopters
 *          - Coaxial helicopters
 *          - Y6 and other specialty configurations
 *          
 *          This class implements the singleton pattern and is accessed throughout the codebase via
 *          the global 'copter' object declared at the end of this file.
 *          
 *          ## Core Responsibilities
 *          
 *          **Flight Control Loop:**
 *          - Runs main scheduler at 400Hz (configurable)
 *          - Executes scheduler_tasks array with tasks at various frequencies
 *          - Reads sensors (IMU, GPS, baro, compass, rangefinder, optical flow)
 *          - Updates AHRS and EKF for state estimation
 *          - Processes pilot inputs from RC transmitter
 *          - Executes current flight mode logic
 *          - Calculates desired attitude, rates, and thrust
 *          - Outputs motor commands to ESCs
 *          - Handles GCS communication via MAVLink
 *          - Performs safety checks and failsafe monitoring
 *          
 *          **Flight Mode Management:**
 *          - Maintains current flight mode via flightmode pointer
 *          - Supports mode switching based on RC switch positions or GCS commands
 *          - Each mode implements specific vehicle behavior (manual, assisted, or autonomous)
 *          - Mode transitions validated for safety and feasibility
 *          - Modes include: Stabilize, AltHold, Loiter, Auto, Guided, RTL, Land, etc.
 *          
 *          **Control System Architecture:**
 *          - Inner loop: Rate controller (roll/pitch/yaw rates) runs at main loop frequency
 *          - Middle loop: Attitude controller (roll/pitch/yaw angles) at main loop frequency
 *          - Outer loop: Position controller (velocity and position) at 50-400Hz
 *          - Waypoint navigation: Generates smooth trajectories between waypoints
 *          - Motor mixing: Converts desired thrust and moments to individual motor outputs
 *          
 *          **Sensor Integration:**
 *          - AP_InertialSensor: Reads gyros and accelerometers (primary and backup IMUs)
 *          - AP_Compass: Reads magnetometers for heading
 *          - AP_Baro: Reads barometer for altitude
 *          - AP_GPS: Reads GPS for position and velocity
 *          - AP_RangeFinder: Reads distance sensors for terrain following and precision landing
 *          - AP_OpticalFlow: Reads optical flow for velocity estimation without GPS
 *          - All sensor data fused through AHRS and EKF for robust state estimation
 *          
 *          **Safety Systems:**
 *          - Arming checks: Pre-flight validation before motors can spin
 *          - Failsafes: Radio loss, GCS loss, battery low, EKF failure, terrain loss
 *          - Geofencing: Enforces altitude and boundary limits
 *          - Crash detection: Monitors for crashes and disarms automatically
 *          - Vibration monitoring: Warns of high vibration affecting sensors
 *          - Thrust loss detection: Identifies motor or propeller failures
 *          
 *          **Library Subsystem Integration:**
 *          - motors: AP_Motors object for motor mixing and output
 *          - attitude_control: AC_AttitudeControl for inner loop control
 *          - pos_control: AC_PosControl for position/velocity control
 *          - wp_nav: AC_WPNav for waypoint navigation and trajectory generation
 *          - loiter_nav: AC_Loiter for position hold with GPS
 *          - circle_nav: AC_Circle for circling waypoints
 *          - ahrs_view: AP_AHRS_View for attitude and position estimation
 *          - arming: AP_Arming for arming/disarming logic and pre-arm checks
 *          - battery: AP_BattMonitor for battery voltage/current monitoring
 *          
 *          ## Scheduler Architecture
 *          
 *          The scheduler_tasks static array defines all periodic tasks with their execution rates.
 *          Tasks run at various frequencies:
 *          - 400Hz: Rate controller, motor output, fast loop logging
 *          - 50-100Hz: Position controller, waypoint navigation, attitude updates
 *          - 10Hz: AHRS update, altitude update, battery monitoring
 *          - 3.3Hz: Compass updates, GPS processing
 *          - 1Hz: Slow tasks like parameter saves, terrain updates
 *          
 *          ## Coordinate Frame Conventions
 *          
 *          ArduCopter uses NED (North-East-Down) coordinate frame convention:
 *          - X axis: North (forward)
 *          - Y axis: East (right)
 *          - Z axis: Down (negative altitude)
 *          - Body frame: X forward, Y right, Z down relative to vehicle
 *          - Quaternions used internally for attitude representation to avoid gimbal lock
 *          
 *          ## Unit Conventions
 *          
 *          Common units used throughout the code:
 *          - Angles: Typically centidegrees (1/100th degree) or radians depending on context
 *          - Angular rates: Degrees per second or radians per second
 *          - Distance: Centimeters or meters depending on context
 *          - Velocity: Centimeters per second or meters per second
 *          - Altitude: Centimeters relative to home or meters above sea level
 *          - Time: Milliseconds (uint32_t) or microseconds (uint64_t)
 *          - PWM: Microseconds (typically 1000-2000)
 *          
 *          ## Singleton Access Pattern
 *          
 *          The Copter class uses singleton pattern. Access the global instance via:
 *          @code
 *          copter.flightmode->update();  // Access flight mode
 *          copter.motors->output();       // Access motors
 *          @endcode
 *          
 *          Libraries also use singletons accessed via AP:: namespace:
 *          @code
 *          AP::ahrs().get_attitude(attitude);
 *          AP::logger().Write_AHRS();
 *          @endcode
 *          
 *          ## Thread Safety
 *          
 *          Most Copter methods run in the main thread. Some considerations:
 *          - Fast rate controller may run in separate thread on capable boards
 *          - Use WITH_SEMAPHORE for multi-threaded data access
 *          - HAL scheduler handles interrupt context vs main thread context
 *          
 * @note The Copter class has many friend classes to allow mode implementations and subsystems
 *       to access private members. This is a pragmatic design choice for performance and clarity.
 * 
 * @warning This is safety-critical flight control code. All modifications must be:
 *          - Tested thoroughly in SITL simulation
 *          - Tested on bench with propellers removed
 *          - Tested in flight with caution and safety pilot
 *          - Reviewed by experienced ArduPilot developers
 * 
 * @see Mode for flight mode base class
 * @see AP_Vehicle for base vehicle functionality
 * @see Parameters for vehicle configuration
 * @see AP_Scheduler::Task for task scheduling structure
 * 
 * Source: ArduCopter/Copter.h
 */
class Copter : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Copter;
    friend class GCS_Copter;
    friend class AP_Rally_Copter;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Avoidance_Copter;

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    friend class AP_AdvancedFailsafe_Copter;
#endif
    friend class AP_Arming_Copter;
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif
    friend class ToyMode;
    friend class RC_Channel_Copter;
    friend class RC_Channels_Copter;

    friend class AutoTune;

    friend class Mode;
    friend class ModeAcro;
    friend class ModeAcro_Heli;
    friend class ModeAltHold;
    friend class ModeAuto;
    friend class ModeAutoTune;
    friend class ModeAvoidADSB;
    friend class ModeBrake;
    friend class ModeCircle;
    friend class ModeDrift;
    friend class ModeFlip;
    friend class ModeFlowHold;
    friend class ModeFollow;
    friend class ModeGuided;
    friend class ModeLand;
    friend class ModeLoiter;
    friend class ModePosHold;
    friend class ModeRTL;
    friend class ModeSmartRTL;
    friend class ModeSport;
    friend class ModeStabilize;
    friend class ModeStabilize_Heli;
    friend class ModeSystemId;
    friend class ModeThrow;
    friend class ModeZigZag;
    friend class ModeAutorotate;
    friend class ModeTurtle;

    friend class _AutoTakeoff;

    friend class PayloadPlace;

    Copter(void);

private:

    /**
     * @brief Common multicopter parameters shared with libraries
     * 
     * @details Contains parameters that are passed to multiple library objects,
     *          such as frame-specific characteristics and common tuning values.
     * 
     * @see AP_MultiCopter for parameter definitions
     */
    AP_MultiCopter aparm;

    /**
     * @brief Primary parameter group containing vehicle configuration
     * 
     * @details The 'g' object contains the original set of ArduCopter parameters including:
     *          - Flight mode settings
     *          - RC input mappings and calibration
     *          - Tuning parameters (PIDs, rate limits)
     *          - Failsafe thresholds and actions
     *          - Geofence settings
     *          - Battery monitoring configuration
     *          - Logging options
     *          
     *          These parameters are stored in EEPROM and persist across reboots.
     *          Users configure them via ground station software.
     * 
     * @note Legacy parameter group - new parameters go in g2
     * @see Parameters class definition in Parameters.h
     * @see AP_Param for parameter system
     */
    Parameters g;
    
    /**
     * @brief Secondary parameter group for newer parameters
     * 
     * @details The 'g2' object was added when the original 'g' parameter space filled up.
     *          It contains newer parameters added to ArduCopter including:
     *          - Advanced tuning options
     *          - Extended failsafe settings
     *          - Newer sensor configurations
     *          - Feature enable/disable flags
     *          
     *          New parameters should be added to g2 to maintain backward compatibility.
     * 
     * @see ParametersG2 class definition in Parameters.h
     */
    ParametersG2 g2;

    // used to detect MAVLink acks from GCS to stop compassmot
    uint8_t command_ack_counter;

    /**
     * @brief Primary RC input channels for manual flight control
     * 
     * @details These pointers reference the RC channels used for primary flight control inputs.
     *          The channels are assigned during initialization based on the configured RC mapping.
     *          Pilot stick movements on these channels control the vehicle in manual flight modes.
     *          
     *          Channel functions:
     *          - Roll: Controls bank angle (left/right tilt)
     *          - Pitch: Controls pitch angle (forward/back tilt)
     *          - Throttle: Controls climb rate or altitude
     *          - Yaw: Controls heading (rotation about vertical axis)
     *          
     *          Input processing:
     *          - Raw PWM values (typically 1000-2000 μs) read from receiver
     *          - Applied deadzone filtering to eliminate stick jitter
     *          - Scaled to control range (e.g., ±4500 centidegrees for angles)
     *          - Expo curves applied for smoother feel near center
     *          - Used by flight modes to determine pilot intent
     * 
     * @note Channel assignment can be remapped via RCMapper parameters
     * @see RC_Channel for channel implementation
     * @see rcmap for RC channel mapping
     */
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_yaw;

    /**
     * @brief RC channel used for in-flight parameter tuning
     * 
     * @details When transmitter-based tuning is enabled, this pointer references
     *          the RC channel (typically a knob or slider) used to adjust a parameter
     *          value in real-time during flight. Allows quick tuning of PID gains
     *          and other parameters without landing.
     * 
     * @note Null when tuning not active
     * @see tuning() for tuning implementation
     */
    RC_Channel *rc_tuning;

    // flight modes convenience array
    AP_Int8 *flight_modes;
    const uint8_t num_flight_modes = 6;

    AP_SurfaceDistance rangefinder_state {ROTATION_PITCH_270, 0U};
    AP_SurfaceDistance rangefinder_up_state {ROTATION_PITCH_90, 1U};

    // helper function to get inertially interpolated rangefinder height.
    bool get_rangefinder_height_interpolated_cm(int32_t& ret) const;

#if AP_RANGEFINDER_ENABLED
    class SurfaceTracking {
    public:

        // update_surface_offset - manages the vertical offset of the position controller to follow the
        //   measured ground or ceiling level measured using the range finder.
        void update_surface_offset();

        // target has already been set by terrain following so do not initialise again
        // this should be called by flight modes when switching from terrain following to surface tracking (e.g. ZigZag)
        void external_init();

        // get target and actual distances (in m) for logging purposes
        bool get_target_dist_for_logging(float &target_dist) const;
        float get_dist_for_logging() const;
        void invalidate_for_logging() { valid_for_logging = false; }

        // surface tracking surface
        enum class Surface {
            NONE = 0,
            GROUND = 1,
            CEILING = 2
        };
        // set surface to track
        void set_surface(Surface new_surface);
        // initialise surface tracking
        void init(Surface surf) { surface = surf; }

    private:
        Surface surface;
        uint32_t last_update_ms;    // system time of last update to target_alt_cm
        uint32_t last_glitch_cleared_ms;    // system time of last handle glitch recovery
        bool valid_for_logging;     // true if we have a desired target altitude
        bool reset_target;          // true if target should be reset because of change in surface being tracked
    } surface_tracking;
#endif

#if AP_RPM_ENABLED
    AP_RPM rpm_sensor;
#endif

    /**
     * @brief AHRS (Attitude and Heading Reference System) view object
     * 
     * @details Provides attitude and heading estimation from sensor fusion.
     *          The AHRS combines data from:
     *          - Gyroscopes (angular rates)
     *          - Accelerometers (gravity vector and acceleration)
     *          - Magnetometers (magnetic heading)
     *          - GPS (velocity and position)
     *          - Barometer (altitude)
     *          
     *          The ahrs_view is a specific viewpoint into the AHRS system, allowing
     *          multiple perspectives on the same state estimate. ArduCopter primarily
     *          uses EKF-based AHRS backends (EKF2 or EKF3) for state estimation.
     *          
     *          Key outputs:
     *          - Vehicle attitude (roll, pitch, yaw) as quaternion
     *          - Angular rates (body frame)
     *          - Position estimate (NED frame)
     *          - Velocity estimate (NED frame)
     *          - Acceleration (body and earth frames)
     *          
     * @note Accessed via AP::ahrs() singleton in most code
     * @see AP_AHRS for AHRS interface
     * @see AP_NavEKF3 for Extended Kalman Filter implementation
     */
    AP_AHRS_View *ahrs_view;

    /**
     * @brief Arming system for pre-flight safety checks and motor arming/disarming
     * 
     * @details The arming object manages the critical safety system that prevents motors
     *          from spinning until all pre-flight checks pass. This protects against:
     *          - Flying with miscalibrated sensors
     *          - Flying with GPS problems
     *          - Flying with failing subsystems
     *          - Accidental motor startup
     *          
     *          Pre-arm checks (must pass before arming is allowed):
     *          - RC radio connected and calibrated
     *          - Accelerometers calibrated and healthy
     *          - Compass calibrated and consistent
     *          - GPS lock acquired (for GPS-dependent modes)
     *          - Battery voltage sufficient
     *          - EKF initialized and healthy
     *          - Geofence valid (if enabled)
     *          - Home position set
     *          
     *          Arming checks (must pass at arming time):
     *          - Pre-arm checks still passing
     *          - Vehicle level (within limits)
     *          - Throttle at minimum
     *          - Flight mode valid for arming
     *          
     *          Disarming occurs automatically on:
     *          - Landing detection (after delay)
     *          - Critical failures (EKF failure, motor failure)
     *          - Pilot stick command (throttle low, yaw left)
     *          
     * @warning Disabling arming checks compromises flight safety
     * @see AP_Arming_Copter for copter-specific arming checks
     * @see AP_Arming for base arming functionality
     */
    AP_Arming_Copter arming;

    // Optical flow sensor
#if AP_OPTICALFLOW_ENABLED
    AP_OpticalFlow optflow;
#endif

    // external control library
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Copter external_control;
#endif


    // system time in milliseconds of last recorded yaw reset from ekf
    uint32_t ekfYawReset_ms;
    int8_t ekf_primary_core;

    // vibration check
    struct {
        bool high_vibes;    // true while high vibration are detected
        uint32_t start_ms;  // system time high vibration were last detected
        uint32_t clear_ms;  // system time high vibrations stopped
    } vibration_check;

    // EKF variances are unfiltered and are designed to recover very quickly when possible
    // thus failsafes should be triggered on filtered values in order to avoid transient errors 
    LowPassFilterFloat pos_variance_filt;
    LowPassFilterFloat vel_variance_filt;
    bool variances_valid;
    uint32_t last_ekf_check_us;

    // takeoff check
    uint32_t takeoff_check_warning_ms;  // system time user was last warned of takeoff check failure

    /**
     * @brief Ground Control Station communication system
     * 
     * @details Manages all MAVLink communication with ground control stations, companion
     *          computers, and other MAVLink-speaking devices. Handles:
     *          
     *          Telemetry Streaming:
     *          - Attitude, position, velocity data
     *          - Battery status and sensors
     *          - GPS information and satellite count
     *          - RC input and servo output values
     *          - System status and mode information
     *          - Configurable stream rates for bandwidth management
     *          
     *          Command Handling:
     *          - Mode change commands
     *          - Arm/disarm requests
     *          - Takeoff and landing commands
     *          - Mission upload/download
     *          - Parameter get/set operations
     *          - Guided mode position/velocity targets
     *          
     *          Mission Protocol:
     *          - Waypoint upload/download
     *          - Mission count and item requests
     *          - Mission clear operations
     *          - Rally point management
     *          
     *          Multiple Channels:
     *          - Serial telemetry radios
     *          - USB connection
     *          - WiFi/Ethernet links
     *          - Each channel independently configurable
     * 
     * @note Access via gcs() method, not directly
     * @see GCS_Copter for copter-specific GCS implementation
     * @see GCS_MAVLink for base MAVLink functionality
     */
    GCS_Copter _gcs; // avoid using this; use gcs()
    
    /**
     * @brief Accessor for GCS object
     * 
     * @return Reference to the GCS_Copter instance
     * @note Prefer this accessor over direct _gcs access
     */
    GCS_Copter &gcs() { return _gcs; }

    // User variables
#ifdef USERHOOK_VARIABLES
# include USERHOOK_VARIABLES
#endif

    /**
     * @brief Calculate bitmask representing current vehicle state
     * 
     * @details Combines various state flags from the 'ap' struct into a single 32-bit value
     *          for efficient logging and transmission. This allows the complete vehicle state
     *          to be captured in a single value.
     * 
     * @return uint32_t Bitmask of vehicle state flags
     * @note Used primarily for logging and debugging
     */
    uint32_t ap_value() const;

    /**
     * @brief Vehicle state flags structure
     * 
     * @details The 'ap' struct contains boolean flags representing the current state of the vehicle.
     *          These flags track critical states like arming status, landing detection, GPS health,
     *          and user control states. Many of these flags are used for safety interlocks and
     *          mode transition logic.
     *          
     * @warning The order and offsets of these variables must NEVER change as the logging system
     *          depends on this exact memory layout for binary log compatibility. New flags must
     *          be added at the end or use existing unused slots.
     * 
     * @note Marked PACKED to ensure consistent memory layout across compilers
     * @see ap_value() to get bitmask representation
     */
    struct PACKED {
        bool unused1;                        //  0
        bool unused_was_simple_mode_byte1;   //  1
        bool unused_was_simple_mode_byte2;   //  2
        bool pre_arm_rc_check;               //  3 true if rc input pre-arm checks have been completed successfully
        bool pre_arm_check;                  //  4 true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        bool auto_armed;                     //  5 stops auto missions from beginning until throttle is raised
        bool unused_log_started;             //  6
        bool land_complete;                  //  7 true if we have detected a landing
        bool new_radio_frame;                //  8 Set true if we have new PWM data to act on from the Radio
        bool unused_usb_connected;           //  9
        bool unused_receiver_present;        // 10
        bool compass_mot;                    // 11 true if we are currently performing compassmot calibration
        bool motor_test;                     // 12 true if we are currently performing the motors test
        bool initialised;                    // 13 true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
        bool land_complete_maybe;            // 14 true if we may have landed (less strict version of land_complete)
        bool throttle_zero;                  // 15 true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
        bool system_time_set_unused;         // 16 true if the system time has been set from the GPS
        bool gps_glitching;                  // 17 true if GPS glitching is affecting navigation accuracy
        bool using_interlock;                // 18 aux switch motor interlock function is in use
        bool land_repo_active;               // 19 true if the pilot is overriding the landing position
        bool motor_interlock_switch;         // 20 true if pilot is requesting motor interlock enable
        bool in_arming_delay;                // 21 true while we are armed but waiting to spin motors
        bool initialised_params;             // 22 true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
        bool unused_compass_init_location;   // 23
        bool unused2_aux_switch_rc_override_allowed; // 24
        bool armed_with_airmode_switch;      // 25 we armed using a arming switch
        bool prec_land_active;               // 26 true if precland is active
    } ap;

    /**
     * @brief Air mode configuration state
     * 
     * @details Air mode keeps motors spinning at a minimum throttle even when the throttle stick
     *          is at zero, allowing the attitude controllers to maintain control authority.
     *          
     *          Air mode states:
     *          - NOT_CONFIGURED (0): Air mode setting not yet determined
     *          - DISABLED (1): Air mode disabled - motors stop when throttle at zero
     *          - ENABLED (2): Air mode enabled - motors maintain minimum throttle
     *          
     *          When air mode is enabled, the vehicle can maintain attitude control even at
     *          zero throttle, enabling aggressive aerobatic maneuvers and better control
     *          during rapid descents. This is particularly useful for racing and acrobatic flight.
     *          
     *          Air mode can be enabled via parameter or auxiliary switch.
     * 
     * @note Air mode uses more battery but provides superior control
     * @warning With air mode enabled, motors spin when armed even at zero throttle
     * @see ap.armed_with_airmode_switch to check if armed via air mode switch
     */
    AirMode air_mode;
    
    /**
     * @brief Force flying state override flag
     * 
     * @details When set to true, forces the vehicle to be considered "flying" regardless of
     *          actual flight state detection. This overrides the automatic landing detection
     *          and prevents certain safety features from triggering.
     *          
     *          Use cases:
     *          - Testing flight control algorithms on the bench
     *          - Preventing unwanted landing detection during specific maneuvers
     *          - Maintaining full control authority in unusual flight conditions
     *          
     *          When force_flying is enabled:
     *          - Landing detector is bypassed
     *          - Land_complete flag is not set automatically
     *          - Throttle mix remains at flight levels
     *          - Full attitude control authority maintained
     * 
     * @warning Use with caution - disables automatic landing detection and safety features
     * @note Typically controlled via auxiliary switch or scripting
     * @see get_force_flying() to query this state
     * @see update_throttle_mix() for control mixing effects
     */
    bool force_flying;

    /**
     * @brief Current active flight mode
     * 
     * @details Pointer to the currently active Mode object that implements vehicle behavior.
     *          The flight mode determines how the vehicle responds to pilot inputs and executes
     *          autonomous behaviors. The flightmode pointer is changed when the pilot switches
     *          modes via RC transmitter or when the GCS commands a mode change.
     *          
     *          Common flight modes:
     *          - Stabilize: Manual flight with automatic attitude stabilization
     *          - AltHold: Stabilize with automatic altitude hold
     *          - Loiter: Position hold using GPS
     *          - Auto: Autonomous mission execution
     *          - Guided: Position/velocity control from GCS or companion computer
     *          - RTL: Return to launch and land
     *          - Land: Automated landing
     *          - Circle: Orbit around a point
     *          - Sport: High-rate manual flight
     *          
     *          Mode changes are validated for safety (e.g., can't enter Loiter without GPS).
     *          The mode's init() method is called on mode entry, and run() is called each loop.
     * 
     * @note Never access directly; use set_mode() to change modes
     * @warning Null check before dereferencing - should never be null after initialization
     * @see Mode for flight mode base class
     * @see set_mode() for mode change logic
     * @see update_flight_mode() called each main loop to execute mode logic
     */
    Mode *flightmode;

    /**
     * @brief RC channel to function mapper
     * 
     * @details Maps physical RC input channels to flight control functions.
     *          Allows users to assign roll/pitch/throttle/yaw to different RC channels
     *          to match their transmitter configuration. Default mapping is:
     *          - Channel 1: Roll
     *          - Channel 2: Pitch
     *          - Channel 3: Throttle
     *          - Channel 4: Yaw
     *          
     *          Users can reconfigure via RCMAP_* parameters to support different
     *          transmitter modes (Mode 1, Mode 2, etc.)
     * 
     * @see RC_Channel for channel implementation
     * @see AP_RCMapper for mapping functionality
     */
    RCMapper rcmap;

    /**
     * @brief Altitude at time of arming
     * 
     * @details Records the inertial navigation altitude when motors are armed.
     *          Used to calculate altitude gained during flight and for determining
     *          when to re-enable altitude-based failsafes after takeoff.
     * 
     * @note In meters, relative to EKF origin
     */
    float arming_altitude_m;

    /**
     * @brief Failsafe system state tracking
     * 
     * @details The failsafe struct tracks which failsafe conditions are currently active.
     *          Multiple failsafes can be active simultaneously, and the system responds
     *          with the highest priority action.
     *          
     *          Failsafe types and triggers:
     *          
     *          Radio Failsafe:
     *          - Triggered when RC receiver stops receiving valid signals
     *          - Detected by loss of PWM updates or failsafe bit from receiver
     *          - Can be configured to continue mission or RTL/Land
     *          
     *          GCS Failsafe:
     *          - Triggered when ground control station telemetry link lost
     *          - Timeout-based (no MAVLink heartbeat received)
     *          - Configurable action: Continue, RTL, Land
     *          
     *          EKF Failsafe:
     *          - Triggered when Extended Kalman Filter reports unacceptable errors
     *          - Indicates GPS glitch, compass error, or sensor failure
     *          - Usually results in Land mode to prevent flyaway
     *          
     *          Terrain Failsafe:
     *          - Triggered when terrain data unavailable for terrain following
     *          - Only active when terrain following enabled
     *          
     *          Battery Failsafe:
     *          - Triggered at configurable battery voltage/capacity thresholds
     *          - Handled separately via battery object, not in this struct
     *          
     *          ADSB Failsafe:
     *          - Triggered when potential collision detected via ADS-B
     *          - Initiates avoidance maneuvers
     *          
     *          Deadreckoning Failsafe:
     *          - Triggered when position estimate relies only on IMU integration
     *          - Indicates loss of all position sensors (GPS, optical flow, etc.)
     * 
     * @warning When multiple failsafes trigger, highest priority action executes
     * @see do_failsafe_action() for failsafe response logic
     * @see FailsafeAction enum for possible actions
     * @see _failsafe_priorities for action priority ordering
     */
    struct {
        uint32_t terrain_first_failure_ms;  // the first time terrain data access failed - used to calculate the duration of the failure
        uint32_t terrain_last_failure_ms;   // the most recent time terrain data access failed

        int8_t radio_counter;            // number of iterations with throttle below throttle_fs_value

        uint8_t radio               : 1; // A status flag for the radio failsafe
        uint8_t gcs                 : 1; // A status flag for the ground station failsafe
        uint8_t ekf                 : 1; // true if ekf failsafe has occurred
        uint8_t terrain             : 1; // true if the missing terrain data failsafe has occurred
        uint8_t adsb                : 1; // true if an adsb related failsafe has occurred
        uint8_t deadreckon          : 1; // true if a dead reckoning failsafe has triggered
    } failsafe;

    bool any_failsafe_triggered() const {
        return failsafe.radio || battery.has_failsafed() || failsafe.gcs || failsafe.ekf || failsafe.terrain || failsafe.adsb || failsafe.deadreckon;
    }

    /**
     * @brief Dead reckoning state tracking
     * 
     * @details Dead reckoning mode occurs when the EKF loses all position and velocity
     *          measurement sources (GPS, optical flow, external nav, etc.) but continues
     *          to estimate position using IMU integration and estimated airspeed.
     *          
     *          Dead reckoning is an emergency navigation mode that provides degraded
     *          position estimates when normal position sources are unavailable. The position
     *          estimate quality degrades over time due to IMU drift and airspeed estimation errors.
     *          
     *          State transitions:
     *          1. Normal operation → active=true when position sources lost
     *          2. active=true → timeout=true after configured timeout period
     *          3. timeout=true → Dead reckoning failsafe triggers
     *          
     *          When dead reckoning timeout occurs, position and velocity estimates should
     *          no longer be trusted for navigation, and appropriate failsafe actions are taken.
     * 
     * @warning Dead reckoning position estimates degrade rapidly - typically unreliable
     *          after 20-30 seconds without position measurements
     * 
     * @see failsafe_deadreckon_check() for dead reckoning monitoring
     * @see failsafe.deadreckon flag for failsafe state
     */
    struct {
        bool active;        /**< @brief True if EKF is in dead reckoning mode (no position/velocity sources) */
        bool timeout;       /**< @brief True if dead reckoning has exceeded timeout - position estimate unreliable */
        uint32_t start_ms;  /**< @brief System time (milliseconds) when EKF entered dead reckoning mode */
    } dead_reckoning;

    /**
     * @brief Motor control and mixing object
     * 
     * @details Pointer to the motors object that handles motor mixing and output.
     *          The actual type is either AP_MotorsMulticopter or AP_MotorsHeli depending
     *          on FRAME_CONFIG (defined via MOTOR_CLASS macro).
     *          
     *          Responsibilities:
     *          - Convert desired roll/pitch/yaw/throttle to individual motor outputs
     *          - Implement motor mixing for various frame types (quad, hex, octo, Y6, etc.)
     *          - Handle motor scaling and limiting
     *          - Manage motor output during arming/disarming
     *          - Apply motor curve and expo functions
     *          - Enforce minimum and maximum throttle limits
     *          - Support motor testing and ESC calibration
     *          
     *          The motors object is called at main loop rate (400Hz) via motors_output().
     *          It translates the commanded attitude rates and thrust from the controllers
     *          into PWM signals sent to individual ESCs.
     *          
     *          Frame-specific behavior:
     *          - Multirotors: Distribute thrust across motors based on arm geometry
     *          - Helicopters: Control swashplate servos and main rotor collective/cyclic
     *          - Coaxial: Handle upper and lower rotor independently
     * 
     * @note Allocated dynamically in allocate_motors() based on frame configuration
     * @warning Critical for flight safety - motor output errors can cause crashes
     * @see AP_MotorsMulticopter for multirotor implementation
     * @see AP_MotorsHeli for helicopter implementation
     * @see motors_output() for output function
     * @see MOTOR_CLASS macro definition based on FRAME_CONFIG
     */
    MOTOR_CLASS *motors;
    
    /**
     * @brief Parameter metadata for the motors object
     * 
     * @details Points to the var_info table for motor parameters, allowing the parameter
     *          system to discover and manage motor-related configuration values.
     */
    const struct AP_Param::GroupInfo *motors_var_info;

    /**
     * @brief Bearing from current position to home in centidegrees
     * 
     * @details Cached value updated by home_bearing() function.
     *          Bearing is in centidegrees (0-36000) from North.
     *          Used for distance and direction displays on telemetry.
     * 
     * @note Value is cached to avoid repeated calculations
     * @see home_bearing() for bearing calculation and update
     */
    int32_t _home_bearing;
    
    /**
     * @brief Distance from current position to home in centimeters
     * 
     * @details Cached value updated by home_distance() function.
     *          Horizontal distance ignoring altitude difference.
     *          Used for telemetry displays and RTL mode decisions.
     * 
     * @note Value is cached to avoid repeated calculations
     * @see home_distance() for distance calculation and update
     */
    uint32_t _home_distance;

    /**
     * @brief Simple mode state for simplified flight control
     * 
     * @details Simple mode helps novice pilots by transforming stick inputs relative to the vehicle's
     *          heading at arming (SIMPLE) or relative to home direction (SUPERSIMPLE).
     *          
     *          SIMPLE Mode: Stick inputs are relative to initial heading at arming.
     *          Roll stick always moves aircraft right/left relative to pilot's perspective.
     *          
     *          SUPERSIMPLE Mode: Stick inputs are relative to direction to home.
     *          Forward stick always moves aircraft away from pilot, backward toward pilot.
     *          Reference frame updates when vehicle moves >20m from home.
     * 
     * @note Value reset at each arming or when leaving 20m radius in SuperSimple
     * @see init_simple_bearing() for initialization
     * @see update_simple_mode() for continuous updates
     * @see update_super_simple_bearing() for SuperSimple heading updates
     */
    enum class SimpleMode {
        NONE = 0,         ///< Simple mode disabled (normal control)
        SIMPLE = 1,       ///< Simple mode - inputs relative to arming heading
        SUPERSIMPLE = 2,  ///< Super-Simple mode - inputs relative to home direction
    } simple_mode;

    /**
     * @brief Cosine of simple mode reference yaw angle
     * @details Pre-calculated for efficient coordinate transformations in simple mode
     * @see update_simple_mode() for calculation
     */
    float simple_cos_yaw;
    
    /**
     * @brief Sine of simple mode reference yaw angle
     * @details Pre-calculated for efficient coordinate transformations in simple mode
     * @see update_simple_mode() for calculation
     */
    float simple_sin_yaw;
    
    /**
     * @brief Last bearing to home used in SuperSimple mode (centidegrees)
     * @details Tracks previous bearing to detect significant changes requiring update
     * @see update_super_simple_bearing() for bearing tracking
     */
    int32_t super_simple_last_bearing;
    
    /**
     * @brief Cosine of SuperSimple mode bearing to home
     * @details Pre-calculated for efficient coordinate transformations in SuperSimple mode
     * @see update_super_simple_bearing() for calculation
     */
    float super_simple_cos_yaw;
    
    /**
     * @brief Sine of SuperSimple mode bearing to home
     * @details Pre-calculated for efficient coordinate transformations in SuperSimple mode
     * @see update_super_simple_bearing() for calculation
     */
    float super_simple_sin_yaw;

    /**
     * @brief Initial vehicle bearing when armed (radians)
     * 
     * @details Stores the vehicle's compass heading at arming time.
     *          Used for Simple mode reference frame which remains fixed.
     *          Not used for SuperSimple mode as that reference changes dynamically.
     * 
     * @note Value captured at arming and remains constant until disarmed
     * @see init_simple_bearing() for initialization at arming
     */
    float initial_armed_bearing_rad;

    /**
     * @brief Battery monitoring system
     * 
     * @details Monitors battery voltage, current, and capacity consumption to:
     *          - Provide real-time battery status to pilot and GCS
     *          - Trigger battery failsafes at configurable thresholds
     *          - Enable accurate flight time prediction
     *          - Support multiple battery monitoring backends
     *          
     *          Supported monitoring methods:
     *          - Analog voltage/current sensors
     *          - Smart battery protocols (SMBus, UAVCAN/DroneCAN)
     *          - ESC telemetry
     *          - Sum of multiple battery monitors
     *          
     *          Failsafe levels (configurable):
     *          - Low battery warning: Visual/audio alerts
     *          - Critical battery: RTL or Land
     *          - Emergency battery: Immediate Land
     *          
     *          Battery parameters tracked:
     *          - Voltage per cell
     *          - Total current draw
     *          - Consumed capacity (mAh)
     *          - Remaining capacity percentage
     *          - Time remaining estimate
     * 
     * @note Initialized with logging mask, failsafe callback, and priority array
     * @warning Accurate battery monitoring critical for preventing in-flight power loss
     * @see AP_BattMonitor for battery monitoring implementation
     * @see handle_battery_failsafe() for failsafe response
     */
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Copter::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

#if OSD_ENABLED || OSD_PARAM_ENABLED
    /**
     * @brief On-Screen Display (OSD) system for video overlay
     * 
     * @details Provides real-time flight data overlay on FPV video feed.
     *          Displays telemetry data including attitude, altitude, speed,
     *          battery status, GPS information, and flight mode on pilot's screen.
     *          
     *          Supports multiple OSD backends (MAVLink OSD, MSP OSD).
     *          Panel layout and content fully customizable via parameters.
     * 
     * @note Only compiled if OSD_ENABLED or OSD_PARAM_ENABLED
     * @see AP_OSD for OSD implementation
     */
    AP_OSD osd;
#endif

    /**
     * @brief Barometric altitude in centimeters above home location
     * 
     * @details Raw barometer-based altitude estimate relative to home.
     *          Updated by read_barometer() at scheduler rate (~10Hz).
     *          Used for telemetry display and altitude-based logic.
     *          
     *          For flight control, EKF-fused altitude from AHRS is used instead
     *          as it combines barometer, GPS, and rangefinder for better accuracy.
     * 
     * @note Units: centimeters (cm) above home
     * @see read_barometer() for altitude updates
     * @see update_altitude() for EKF altitude handling
     */
    int32_t baro_alt;
    
    /**
     * @brief Low-pass filtered earth-frame acceleration for landing detection
     * 
     * @details Filters vehicle acceleration in earth frame (NED coordinates) to detect:
     *          - Landing impacts and touchdown events
     *          - Crash detection via abnormal accelerations
     *          - Ground contact verification
     *          
     *          Filter smooths high-frequency vibrations while preserving real impacts.
     *          Used by land_detector and crash_check functions.
     * 
     * @note Earth frame (NED): North-East-Down coordinate system
     * @see update_land_detector() for landing detection logic
     * @see crash_check() for crash detection
     */
    LowPassFilterVector3f land_accel_ef_filter;

    /**
     * @brief Low-pass filtered pilot throttle input for landing detection
     * 
     * @details Filters RC throttle input to detect sustained throttle application.
     *          Used to determine if pilot is actively commanding thrust during landing.
     *          If filtered throttle exceeds threshold during landing, landing is canceled.
     *          
     *          Filtering prevents false triggers from momentary throttle spikes or noise.
     * 
     * @see update_land_detector() for usage in landing logic
     */
    LowPassFilterFloat rc_throttle_control_in_filter;

    /**
     * @brief Current 3D location of the vehicle
     * 
     * @details Contains latitude, longitude, and altitude of vehicle position.
     *          Altitude is relative to home location (not absolute/MSL).
     *          Updated continuously from EKF position estimate.
     *          
     *          Location struct includes:
     *          - lat: Latitude in degrees * 1e7 (int32_t)
     *          - lng: Longitude in degrees * 1e7 (int32_t)
     *          - alt: Altitude in centimeters relative to home (int32_t)
     *          - flags: Coordinate frame and type information
     * 
     * @note Altitude is relative to home, not mean sea level (MSL)
     * @note Lat/lng stored as degrees * 10^7 for precision
     * @see update_altitude() for altitude updates
     * @see Location class for coordinate handling
     */
    Location current_loc;

    /**
     * @brief Attitude control system objects for multicopter flight control
     * 
     * @details These objects form the inner and outer control loops:
     *          
     *          Inner Loop (Attitude Control):
     *          - Implements rate controllers (roll/pitch/yaw angular rates)
     *          - Implements attitude angle controllers
     *          - Provides input shaping and filtering
     *          - Runs at main loop rate (typically 400Hz)
     *          - Uses PID controllers with feedforward terms
     *          - Outputs desired angular rates to motor mixing
     *          
     *          Outer Loops (Position Control):
     *          - Velocity control in horizontal and vertical axes
     *          - Position control using velocity controller
     *          - Trajectory generation and following
     *          - Waypoint navigation with smooth paths
     *          - Typically runs at 50-400Hz depending on configuration
     */
    
    /**
     * @brief Attitude controller object for inner loop control
     * 
     * @details Manages attitude rate and angle control for roll, pitch, and yaw axes.
     *          Converts pilot inputs or autonomous commands into body-frame angular rates,
     *          then outputs motor commands via the motors object.
     *          
     *          Features:
     *          - Rate PID controllers for roll/pitch/yaw
     *          - Attitude angle controllers
     *          - Input shaping for smooth response
     *          - Angle limiting for safety
     *          - Feedforward from pilot input
     *          - Supports multiple frame types (multi, heli, 6DoF)
     * 
     * @note Allocated dynamically based on frame type
     * @see AC_AttitudeControl_Multi for standard multirotors
     * @see AC_AttitudeControl_Heli for helicopters
     */
    AC_AttitudeControl *attitude_control;
    
    /**
     * @brief Parameter metadata for attitude control
     */
    const struct AP_Param::GroupInfo *attitude_control_var_info;
    
    /**
     * @brief Position controller object for outer loop control
     * 
     * @details Implements velocity and position control in 3D space.
     *          Takes position or velocity targets and outputs attitude commands
     *          to the attitude controller.
     *          
     *          Control loops:
     *          - Position → Velocity (P controller with acceleration limits)
     *          - Velocity → Acceleration (PID with feedforward)
     *          - Acceleration → Attitude angles
     *          
     *          Used by most autonomous and assisted flight modes for:
     *          - Altitude hold
     *          - Position hold (Loiter)
     *          - Waypoint navigation
     *          - Trajectory following
     * 
     * @note Works in NED coordinate frame
     * @see AC_PosControl for implementation details
     */
    AC_PosControl *pos_control;
    
    /**
     * @brief Waypoint navigation controller
     * 
     * @details Generates smooth trajectories between waypoints for autonomous flight.
     *          Implements S-curve acceleration profiles for comfortable flight.
     *          Handles waypoint approach, arrival detection, and corner cutting.
     *          
     *          Used by Auto, Guided, and RTL modes for autonomous navigation.
     *          Provides position and velocity targets to pos_control.
     * 
     * @see AC_WPNav for waypoint navigation implementation
     */
    AC_WPNav *wp_nav;
    
    /**
     * @brief Loiter (position hold) controller
     * 
     * @details Specialized controller for holding position using GPS.
     *          Compensates for wind and handles pilot stick inputs for position adjustments.
     *          Uses horizontal position targets with pos_control.
     * 
     * @see AC_Loiter for loiter implementation
     */
    AC_Loiter *loiter_nav;

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    /**
     * @brief Custom control backend for user-defined control algorithms
     * 
     * @details Allows users to inject custom control logic into the flight control loop.
     *          Enables research, prototyping, and specialized control algorithms.
     *          Custom controller can override or augment standard attitude/position control.
     * 
     * @note Only available if AC_CUSTOMCONTROL_MULTI_ENABLED
     * @see AC_CustomControl for custom control interface
     */
    AC_CustomControl custom_control{ahrs_view, attitude_control, motors, scheduler.get_loop_period_s()};
#endif

#if MODE_CIRCLE_ENABLED
    /**
     * @brief Circle mode navigation controller
     * 
     * @details Generates smooth circular flight paths around a center point.
     *          Maintains constant radius and configurable rotation rate.
     *          Used by Circle flight mode for aerial photography and surveying.
     * 
     * @note Only available if MODE_CIRCLE_ENABLED
     * @see AC_Circle for circle navigation implementation
     * @see ModeCircle for circle flight mode
     */
    AC_Circle *circle_nav;
#endif

    /**
     * @brief System time when vehicle was armed (milliseconds)
     * 
     * @details Records the timestamp (from AP_HAL::millis()) when motors were armed.
     *          Used to:
     *          - Calculate flight time duration
     *          - Trigger time-based behaviors (e.g., auto-disarm timeout)
     *          - Log arming events
     *          - Display flight timer to pilot
     *          
     *          Value is set to 0 when disarmed, enabling simple armed/disarmed checks.
     * 
     * @note Units: milliseconds since system boot
     * @note Value is 0 when disarmed, non-zero when armed
     * @see auto_disarm_check() for timeout-based disarming
     */
    uint32_t arm_time_ms;

#if AP_CAMERA_ENABLED
    /**
     * @brief Camera control system for triggering and configuration
     * 
     * @details Provides camera trigger and control capabilities including:
     *          - Timed interval photography
     *          - Distance-based triggering
     *          - Manual trigger via RC or GCS
     *          - Camera configuration (zoom, focus, mode)
     *          - Geotagging support for photo coordinates
     *          - Video recording start/stop
     * 
     * @note Supports multiple camera backends (MAVLink, relay, servo, etc.)
     * @see AP_Camera for camera control implementation
     */
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

#if HAL_MOUNT_ENABLED
    /**
     * @brief Gimbal/camera mount control and stabilization
     * 
     * @details Controls camera gimbal for stable aerial photography/videography:
     *          - Pitch/roll/yaw stabilization compensating for vehicle motion
     *          - RC or GCS control of gimbal pointing
     *          - Region-of-Interest (ROI) tracking
     *          - Automated gimbal positioning in missions
     *          
     *          Supports multiple gimbal protocols:
     *          - MAVLink (Storm32, etc.)
     *          - Serial protocols
     *          - Direct servo control
     * 
     * @note Mount = Gimbal (terminology used interchangeably)
     * @see AP_Mount for mount control implementation
     */
    AP_Mount camera_mount;
#endif

#if AP_AVOIDANCE_ENABLED
    /**
     * @brief Obstacle avoidance system
     * 
     * @details Integrates proximity sensors and algorithms to avoid obstacles:
     *          - Proximity sensor fusion (lidar, sonar, radar)
     *          - Fence integration for boundary avoidance
     *          - Path adjustment to avoid detected obstacles
     *          - Stop-at-fence behavior
     *          
     *          Works with multiple avoidance backends:
     *          - Simple proximity-based stopping
     *          - Dijkstra path planning (AP_OAPathPlanner)
     *          - BendyRuler algorithm
     * 
     * @note Requires AP_Proximity sensor library
     * @see AC_Avoid for avoidance implementation
     * @see AP_Proximity for proximity sensor interface
     */
    AC_Avoid avoid;
#endif

#if HAL_RALLY_ENABLED
    /**
     * @brief Rally point system for alternate landing/RTL locations
     * 
     * @details Manages rally points as alternative return destinations:
     *          - Multiple pre-defined safe return locations
     *          - Automatic selection of nearest rally point
     *          - Used when RTL would return to distant home location
     *          - Provides safe landing zones in controlled areas
     * 
     * @see AP_Rally for rally point management
     * @see ModeRTL for rally point usage in RTL mode
     */
    AP_Rally_Copter rally;
#endif

#if HAL_SPRAYER_ENABLED
    /**
     * @brief Agricultural crop sprayer control system
     * 
     * @details Controls sprayer pump and nozzles for precision agriculture:
     *          - Pump on/off control based on flight speed and mode
     *          - Flow rate adjustment
     *          - Coverage area calculation
     *          - Integration with auto missions
     * 
     * @note Specialized for agricultural spray applications
     * @see AC_Sprayer for sprayer control implementation
     */
    AC_Sprayer sprayer;
#endif

#if HAL_PARACHUTE_ENABLED
    /**
     * @brief Emergency parachute release system
     * 
     * @details Controls parachute deployment for emergency vehicle recovery:
     *          - Manual deployment via RC switch or GCS
     *          - Automatic deployment on crash detection
     *          - Deployment on critical failsafe conditions
     *          - Motor disarm on deployment
     *          
     *          Parachute deployment triggers:
     *          - Pilot command
     *          - GCS command
     *          - Crash detected (if enabled)
     *          - TERMINATE failsafe (if enabled)
     * 
     * @warning Single-use deployment - cannot be retracted in flight
     * @see AP_Parachute for parachute control
     * @see parachute_check() for deployment logic
     */
    AP_Parachute parachute;
#endif

#if AP_LANDINGGEAR_ENABLED
    /**
     * @brief Landing gear retraction/extension controller
     * 
     * @details Controls retractable landing gear for fixed-wing or multirotor aircraft:
     *          - Automatic retraction after takeoff
     *          - Automatic extension before landing
     *          - Manual control via RC switch or GCS
     *          - Configurable deploy/retract altitudes
     *          
     *          Typical sequence:
     *          - Extended on ground and during arming
     *          - Retracts after reaching altitude threshold
     *          - Extends when descending for landing
     *          - Extends on failsafe conditions
     * 
     * @see AP_LandingGear for landing gear control
     * @see landinggear_update() for automatic control logic
     */
    AP_LandingGear landinggear;
#endif

#if AP_TERRAIN_AVAILABLE
    /**
     * @brief Terrain following and terrain database system
     * 
     * @details Provides terrain elevation data for terrain-relative operations:
     *          - Terrain database from SRTM (Shuttle Radar Topography Mission)
     *          - Terrain-relative altitude hold
     *          - Terrain following in Auto missions
     *          - Collision avoidance with terrain
     *          - Rangefinder-based terrain following
     *          
     *          Data sources (priority order):
     *          1. Rangefinder (most accurate, limited range)
     *          2. Terrain database (good coverage, ~30m resolution)
     *          3. Home altitude (fallback)
     * 
     * @note Requires terrain data files on SD card or GCS streaming
     * @see AP_Terrain for terrain data handling
     * @see terrain_update() for continuous terrain tracking
     */
    AP_Terrain terrain;
#endif

#if AC_PRECLAND_ENABLED
    /**
     * @brief Precision landing sensor interface
     * 
     * @details Interfaces with precision landing sensors for accurate landing:
     *          - IR-LOCK infrared beacon tracking
     *          - Companion computer vision systems
     *          - Fiducial marker detection
     *          - Landing target position estimation
     *          
     *          Sensor fusion provides:
     *          - Target position in body and earth frames
     *          - Target velocity estimation
     *          - Landing accuracy <10cm with good sensors
     * 
     * @see AC_PrecLand for precision landing interface
     * @see precland_statemachine for landing state management
     * @see update_precland() for sensor updates
     */
    AC_PrecLand precland;
    
    /**
     * @brief Precision landing state machine controller
     * 
     * @details Manages the precision landing sequence through distinct phases:
     *          - Approach: Move toward landing target
     *          - Descent: Controlled descent over target
     *          - Final: Final descent and touchdown
     *          
     *          Handles retries if target is lost and fallback to normal landing.
     * 
     * @see AC_PrecLand_StateMachine for state machine implementation
     */
    AC_PrecLand_StateMachine precland_statemachine;
#endif

#if FRAME_CONFIG == HELI_FRAME
    /**
     * @brief Helicopter-specific pilot input manager
     * 
     * @details Processes pilot inputs for traditional helicopters with unique requirements:
     *          - Collective pitch management
     *          - Throttle curve application
     *          - Swashplate mixing coordination
     *          - Rotor speed governor interface
     *          - Autorotation mode support
     * 
     * @note Only used for traditional helicopter frame configuration
     * @see AC_InputManager_Heli for helicopter input processing
     */
    AC_InputManager_Heli input_manager;
#endif

#if HAL_ADSB_ENABLED
    /**
     * @brief ADS-B (Automatic Dependent Surveillance-Broadcast) receiver
     * 
     * @details Receives ADS-B transponder signals from nearby aircraft:
     *          - Detects manned aircraft positions
     *          - Tracks aircraft altitude, heading, velocity
     *          - Provides collision warnings
     *          - Integrates with avoidance system
     *          
     *          Typical use: Detect and avoid manned aircraft in shared airspace.
     * 
     * @see AP_ADSB for ADS-B receiver interface
     * @see avoidance_adsb for ADS-B-based avoidance
     */
    AP_ADSB adsb;
#endif  // HAL_ADSB_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
    /**
     * @brief ADS-B collision avoidance system
     * 
     * @details Avoids ADS-B equipped aircraft (typically manned vehicles):
     *          - Calculates collision probability with detected aircraft
     *          - Generates avoidance maneuvers if collision imminent
     *          - Triggers failsafe actions if necessary
     *          - Resumes mission after threat passes
     *          
     *          Avoidance strategies:
     *          - Horizontal path deviation
     *          - Altitude changes
     *          - Loiter until clear
     *          - RTL if escape not possible
     * 
     * @note Requires HAL_ADSB_ENABLED and working ADS-B receiver
     * @see AP_Avoidance_Copter for avoidance implementation
     * @see avoidance_adsb_update() for continuous threat assessment
     */
    AP_Avoidance_Copter avoidance_adsb{adsb};
#endif

    /**
     * @brief Timestamp of last valid RC radio input (milliseconds)
     * 
     * @details Records when valid RC PWM input was last received from receiver.
     *          Used for:
     *          - RC failsafe detection (timeout triggers failsafe)
     *          - Determining if pilot has control authority
     *          - New radio frame detection
     *          
     *          Updated by read_radio() when valid RC data received.
     *          If current time exceeds this by failsafe threshold, RC failsafe triggers.
     * 
     * @note Units: milliseconds since system boot (from AP_HAL::millis())
     * @see read_radio() for RC input processing
     * @see set_throttle_and_failsafe() for failsafe logic
     */
    uint32_t last_radio_update_ms;

    /**
     * @brief Timestamp of last ESC calibration notification update (milliseconds)
     * 
     * @details Used to control notification update rate during ESC calibration.
     *          ESC calibration requires holding throttle high at startup.
     *          Periodic notifications inform pilot of calibration status.
     * 
     * @note Units: milliseconds since system boot
     * @see esc_calibration_notify() for notification logic
     * @see esc_calibration_passthrough() for calibration mode
     */
    uint32_t esc_calibration_notify_update_ms;

    /**
     * @brief Parameter system loader and table manager
     * 
     * @details Manages the parameter storage and retrieval system for ArduPilot.
     *          The AP_Param system provides:
     *          - Persistent storage of parameters in EEPROM/flash
     *          - Parameter loading at startup
     *          - Parameter table (var_info) registration
     *          - Default value initialization
     *          - Parameter conversion between firmware versions
     *          - MAVLink parameter protocol support
     *          
     *          All vehicle, library, and module parameters are registered through this
     *          central parameter loader using the var_info table structure.
     * 
     * @see AP_Param for parameter system implementation
     * @see var_info for the complete parameter table
     * @see load_parameters() for parameter loading sequence
     */
    AP_Param param_loader;

#if FRAME_CONFIG == HELI_FRAME
    /**
     * @brief Traditional helicopter status flags structure
     * 
     * @details Bit-packed flags tracking helicopter-specific flight states.
     *          Used to enable/disable helicopter-specific control features.
     */
    typedef struct {
        uint8_t dynamic_flight          : 1;    /**< @brief True if moving at significant speed (enables leaky I terms) */
        bool coll_stk_low                  ;    /**< @brief True when collective stick is on lower limit */
    } heli_flags_t;
    
    /**
     * @brief Traditional helicopter status flags instance
     * 
     * @details Tracks real-time helicopter flight states:
     *          - dynamic_flight: Enables integral term accumulation when flying
     *          - coll_stk_low: Indicates pilot has collective at minimum
     *          
     *          These flags modify control behavior for traditional helicopters.
     * 
     * @note Only used when FRAME_CONFIG == HELI_FRAME
     * @see check_dynamic_flight() for dynamic_flight flag updates
     */
    heli_flags_t heli_flags;

    /**
     * @brief Helicopter hover roll trim scalar slew rate limiter
     * 
     * @details Slew rate limited scalar for smoothly applying hover roll trim.
     *          Traditional helicopters require trim adjustments for stable hover.
     *          This variable implements gradual trim application to avoid sudden
     *          roll commands that could destabilize the aircraft.
     * 
     * @note Only used when FRAME_CONFIG == HELI_FRAME
     * @note Units: Implementation-specific scalar value
     */
    int16_t hover_roll_trim_scalar_slew;
#endif

    /**
     * @brief Ground effect detector state tracking
     * 
     * @details Tracks vehicle state relative to ground effect conditions.
     *          Ground effect occurs when rotors are close to the ground, increasing
     *          thrust efficiency and affecting vehicle behavior.
     *          
     *          This state information helps:
     *          - EKF terrain height stability estimation
     *          - Barometer correction during takeoff/landing
     *          - Control gain adjustments near ground
     *          - Landing detection
     */
    struct {
        bool takeoff_expected;      /**< @brief True if vehicle is expected to be taking off */
        bool touchdown_expected;    /**< @brief True if vehicle is expected to be touching down */
        uint32_t takeoff_time_ms;   /**< @brief System time of takeoff initiation (milliseconds) */
        float takeoff_alt_cm;       /**< @brief Altitude at takeoff in centimeters */
    } gndeffect_state;

    /**
     * @brief Standby mode active flag
     * 
     * @details True when vehicle is in standby mode - a low-power state where:
     *          - Vehicle is disarmed
     *          - Logging may be paused
     *          - Some sensors may be powered down
     *          - System is ready but not actively flying
     *          
     *          Standby mode conserves power while maintaining readiness for arming.
     * 
     * @see standby_update() for standby mode management
     */
    bool standby_active;

    /**
     * @brief Scheduler task table defining all periodic tasks and their execution rates
     * 
     * @details This array defines the complete task schedule for the ArduCopter main loop.
     *          Each task specifies:
     *          - Function pointer to the task
     *          - Desired rate in Hz (or rate divider)
     *          - Expected execution time in microseconds
     *          
     *          Tasks are executed by AP_Scheduler based on their rate requirements and available
     *          CPU time. The scheduler ensures critical high-rate tasks (rate controller, motor
     *          output) execute on time while lower priority tasks can slip if CPU is loaded.
     *          
     *          Typical task rates:
     *          - 400Hz: Rate controller (run_rate_controller_main), motor output, fast logging
     *          - 100-50Hz: Position control, waypoint navigation, attitude updates
     *          - 10Hz: AHRS update, altitude update, battery checks, surface tracking
     *          - 3.3Hz: Compass read, barometer read, GPS update
     *          - 1Hz: Parameter checks, terrain updates, telemetry status
     *          
     * @note Defined in Copter.cpp
     * @see AP_Scheduler::Task for task structure definition
     * @see get_scheduler_tasks() for accessing this array
     */
    static const AP_Scheduler::Task scheduler_tasks[];
    
    /**
     * @brief Parameter metadata table for AP_Param system
     * 
     * @details Defines all parameters that can be configured by users via ground station.
     *          Parameters are stored in EEPROM and persist across power cycles.
     *          This table provides metadata for parameter discovery, validation, and documentation.
     * 
     * @note Defined in Parameters.cpp
     * @see AP_Param for parameter system
     * @see Parameters.h for parameter definitions
     */
    static const AP_Param::Info var_info[];
    
    /**
     * @brief Logging structure definitions for dataflash/SD card logging
     * 
     * @details Defines the binary log message formats written by AP_Logger.
     *          Each structure specifies message type, field names, field types, and units.
     *          These logs are used for post-flight analysis and debugging.
     * 
     * @note Defined in Log.cpp
     * @see AP_Logger for logging system
     * @see LogStructure for message structure format
     */
    static const struct LogStructure log_structure[];

    /**
     * @brief ESC (Electronic Speed Controller) calibration modes
     * 
     * @details Defines how ESC calibration is performed at startup.
     *          ESC calibration teaches ESCs the throttle range (min/max PWM values).
     *          Different modes provide flexibility for various ESC types and user preferences.
     */
    enum ESCCalibrationModes : uint8_t {
        ESCCAL_NONE = 0,                            /**< @brief No calibration mode active */
        ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,    /**< @brief Passthrough mode if throttle high at boot (traditional method) */
        ESCCAL_PASSTHROUGH_ALWAYS = 2,              /**< @brief Always passthrough throttle to ESCs (bypasses flight controller) */
        ESCCAL_AUTO = 3,                            /**< @brief Automatic ESC calibration sequence */
        ESCCAL_DISABLED = 9,                        /**< @brief ESC calibration disabled */
    };

    /**
     * @brief Failsafe action types for various failure conditions
     * 
     * @details Defines actions the flight controller can take when a failsafe is triggered.
     *          Different failsafe conditions (radio loss, battery low, GPS loss, etc.) can
     *          be configured to execute these actions based on severity and user preference.
     *          
     *          Actions are prioritized (see _failsafe_priorities array) to ensure the most
     *          critical failsafe takes precedence when multiple conditions occur simultaneously.
     * 
     * @see _failsafe_priorities for action priority ordering
     * @see do_failsafe_action() for action execution
     */
    enum class FailsafeAction : uint8_t {
        NONE               = 0,     /**< @brief No action - continue current flight mode */
        LAND               = 1,     /**< @brief Land immediately at current location */
        RTL                = 2,     /**< @brief Return to launch location and land */
        SMARTRTL           = 3,     /**< @brief Return via recorded path, fallback to RTL */
        SMARTRTL_LAND      = 4,     /**< @brief Return via recorded path, fallback to land */
        TERMINATE          = 5,     /**< @brief Terminate flight immediately (disarm motors) */
        AUTO_DO_LAND_START = 6,     /**< @brief Jump to nearest DO_LAND_START mission item */
        BRAKE_LAND         = 7      /**< @brief Brake to stop, then land at current location */
    };

    /**
     * @brief Failsafe behavior option flags (bitmask)
     * 
     * @details Configurable options that modify default failsafe behavior.
     *          These flags allow users to customize failsafe actions for specific use cases.
     *          Multiple options can be combined using bitwise OR.
     *          
     *          Examples:
     *          - Continue mission if radio lost during autonomous flight
     *          - Continue if landing is already in progress
     *          - Release payload on specific failsafe conditions
     * 
     * @see failsafe_option() to check if specific option is enabled
     */
    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO             = (1<<0),   /**< @brief Continue Auto mode on RC failsafe (don't RTL) */
        GCS_CONTINUE_IF_AUTO            = (1<<1),   /**< @brief Continue Auto mode on GCS failsafe (don't RTL) */
        RC_CONTINUE_IF_GUIDED           = (1<<2),   /**< @brief Continue Guided mode on RC failsafe (don't RTL) */
        CONTINUE_IF_LANDING             = (1<<3),   /**< @brief Continue landing if failsafe during landing */
        GCS_CONTINUE_IF_PILOT_CONTROL   = (1<<4),   /**< @brief Continue if pilot has RC control on GCS failsafe */
        RELEASE_GRIPPER                 = (1<<5),   /**< @brief Release gripper/payload on failsafe */
    };


    /**
     * @brief Flight behavior option flags (bitmask)
     * 
     * @details Configurable options that modify flight behavior and safety checks.
     *          These flags allow users to disable certain safety checks or enable
     *          special behaviors for specific use cases (e.g., testing, specialized operations).
     *          Multiple options can be combined using bitwise OR.
     * 
     * @warning Disabling safety checks can lead to vehicle damage or injury.
     *          Only disable safety features if you fully understand the implications.
     * 
     * @see option_is_enabled() to check if specific option is enabled
     */
    enum class FlightOption : uint32_t {
        DISABLE_THRUST_LOSS_CHECK     = (1<<0),   /**< @brief Disable thrust loss detection (not recommended) */
        DISABLE_YAW_IMBALANCE_WARNING = (1<<1),   /**< @brief Disable yaw imbalance warnings */
        RELEASE_GRIPPER_ON_THRUST_LOSS = (1<<2),  /**< @brief Automatically release gripper/payload on thrust loss */
    };

    /**
     * @brief Fast rate attitude controller operating mode
     * 
     * @details Defines when the high-speed attitude control loop runs.
     *          The fast rate controller runs attitude updates at higher frequency
     *          than the main loop (typically 400Hz vs 400Hz main, or up to 2kHz fast rate).
     *          Higher rates improve attitude control performance, especially for racing/aerobatic flight.
     *          
     *          Tradeoffs:
     *          - DISABLED: Lower CPU usage, standard performance
     *          - DYNAMIC: Automatically enables when needed (optimal balance)
     *          - FIXED_ARMED: Always on when armed (consistent performance)
     *          - FIXED: Always on (maximum performance, highest CPU usage)
     */
    enum class FastRateType : uint8_t {
        FAST_RATE_DISABLED            = 0,      /**< @brief Fast rate controller disabled (main loop rate only) */
        FAST_RATE_DYNAMIC             = 1,      /**< @brief Enable fast rate dynamically based on flight conditions */
        FAST_RATE_FIXED_ARMED         = 2,      /**< @brief Fast rate always enabled when armed */
        FAST_RATE_FIXED               = 3,      /**< @brief Fast rate always enabled (armed or disarmed) */
    };

    /**
     * @brief Get the currently configured fast rate controller type
     * 
     * @return FastRateType The fast rate configuration from parameters
     * 
     * @see FastRateType for mode descriptions
     */
    FastRateType get_fast_rate_type() const { return FastRateType(g2.att_enable.get()); }

    /**
     * @brief Check if a specific flight option is enabled
     * 
     * @param[in] option The FlightOption flag to check
     * 
     * @return true if the specified option is enabled in flight_options parameter
     * @return false if the option is not enabled
     * 
     * @details Checks the g2.flight_options bitmask parameter to determine if
     *          the specified flight option has been enabled by the user.
     * 
     * @see FlightOption for available options
     */
    bool option_is_enabled(FlightOption option) const {
        return (g2.flight_options & uint32_t(option)) != 0;
    }

    /**
     * @brief Failsafe action priority order (highest to lowest priority)
     * 
     * @details When multiple failsafe conditions occur simultaneously, this priority
     *          array determines which action takes precedence. Higher priority actions
     *          override lower priority ones to ensure the most critical response.
     *          
     *          Priority order (highest to lowest):
     *          1. TERMINATE - Immediate motor shutdown (highest priority)
     *          2. LAND - Land immediately at current location
     *          3. RTL - Return to launch
     *          4. SMARTRTL_LAND - Smart RTL with land fallback
     *          5. SMARTRTL - Smart return to launch
     *          6. NONE - No action (lowest priority)
     *          
     *          The sentinel value (-1) marks the end of the priority list.
     * 
     * @note This array is used by the battery failsafe system and other
     *       failsafe handlers to resolve conflicting actions.
     * 
     * @see do_failsafe_action() for action execution
     * @see FailsafeAction for action descriptions
     */
    static constexpr int8_t _failsafe_priorities[] = {
                                                      (int8_t)FailsafeAction::TERMINATE,
                                                      (int8_t)FailsafeAction::LAND,
                                                      (int8_t)FailsafeAction::RTL,
                                                      (int8_t)FailsafeAction::SMARTRTL_LAND,
                                                      (int8_t)FailsafeAction::SMARTRTL,
                                                      (int8_t)FailsafeAction::NONE,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };

    #define FAILSAFE_LAND_PRIORITY 1
    static_assert(_failsafe_priorities[FAILSAFE_LAND_PRIORITY] == (int8_t)FailsafeAction::LAND,
                  "FAILSAFE_LAND_PRIORITY must match the entry in _failsafe_priorities");
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");



    // ========================================================================
    // Method Declarations Organized by Source File
    // ========================================================================

    // AP_State.cpp - Vehicle state management functions
    
    /**
     * @brief Set the auto-armed state
     * 
     * @param[in] b True to set auto-armed, false to clear
     * 
     * @details Auto-armed prevents auto missions from beginning until throttle is raised.
     *          This safety feature ensures the pilot is ready before autonomous flight starts.
     *          
     * @see ap.auto_armed flag in ap struct
     */
    void set_auto_armed(bool b);
    
    /**
     * @brief Set the Simple mode state
     * 
     * @param[in] b SimpleMode enum value (NONE, SIMPLE, or SUPERSIMPLE)
     * 
     * @details Simple mode adjusts pilot input interpretation relative to the vehicle's
     *          initial heading, making control more intuitive for new pilots.
     *          
     * @see SimpleMode enum for mode descriptions
     */
    void set_simple_mode(SimpleMode b);
    
    /**
     * @brief Set the radio failsafe state
     * 
     * @param[in] b True to trigger radio failsafe, false to clear
     * 
     * @details Updates the radio failsafe flag and triggers appropriate failsafe actions
     *          when RC link is lost or recovered.
     *          
     * @see failsafe.radio flag
     * @see failsafe_radio_on_event()
     */
    void set_failsafe_radio(bool b);
    
    /**
     * @brief Set the GCS (Ground Control Station) failsafe state
     * 
     * @param[in] b True to trigger GCS failsafe, false to clear
     * 
     * @details Updates the GCS failsafe flag and triggers appropriate failsafe actions
     *          when telemetry link is lost or recovered.
     *          
     * @see failsafe.gcs flag
     * @see failsafe_gcs_on_event()
     */
    void set_failsafe_gcs(bool b);
    
    /**
     * @brief Update motor interlock usage state
     * 
     * @details Checks if motor interlock auxiliary switch function is assigned and active.
     *          Motor interlock provides an additional safety mechanism to enable/disable
     *          motor output independently from arming.
     *          
     * @see ap.using_interlock flag
     * @see ap.motor_interlock_switch flag
     */
    void update_using_interlock();

    // Copter.cpp - Main vehicle class functions
    
    /**
     * @brief Get the scheduler task table for the main loop
     * 
     * @param[out] tasks Pointer to the scheduler task array
     * @param[out] task_count Number of tasks in the array
     * @param[out] log_bit Logging bitmask for scheduler performance
     * 
     * @details Provides the AP_Scheduler with the list of tasks to execute in the main loop.
     *          Tasks include sensor reads, navigation updates, control loops, and logging.
     *          
     * @see scheduler_tasks array for task definitions
     * @see AP_Scheduler for scheduler implementation
     */
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    // Scripting and External Control API Methods
    
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if MODE_GUIDED_ENABLED
    /**
     * @brief Set target location for guided mode
     * 
     * @param[in] target_loc Target location (lat/lon/alt)
     * 
     * @return true if target was successfully set
     * @return false if target could not be set (not in guided mode, invalid location, etc.)
     * 
     * @details Used by Lua scripts and external control to command the vehicle to fly to
     *          a specific location in guided mode. Altitude can be relative or absolute.
     *          
     * @note Vehicle must be in Guided mode for this to take effect
     * @see Location for coordinate structure
     */
    bool set_target_location(const Location& target_loc) override;
    
    /**
     * @brief Command takeoff to specified altitude
     * 
     * @param[in] alt Target altitude in meters (relative to home)
     * 
     * @return true if takeoff command was accepted
     * @return false if takeoff could not be initiated
     * 
     * @details Initiates a takeoff sequence to the specified altitude.
     *          Used by Lua scripts and external control systems.
     *          
     * @note Vehicle must be armed and in appropriate mode
     */
    bool start_takeoff(const float alt) override;
#endif // MODE_GUIDED_ENABLED
#endif // AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
#if MODE_GUIDED_ENABLED
    /**
     * @brief Get current target location in guided mode
     * 
     * @param[out] target_loc Current target location
     * 
     * @return true if target location retrieved successfully
     * @return false if no valid target (not in guided mode, etc.)
     * 
     * @details Allows Lua scripts to query the current guided mode target.
     */
    bool get_target_location(Location& target_loc) override;
    
    /**
     * @brief Update target location (move from old to new)
     * 
     * @param[in] old_loc Previous target location
     * @param[in] new_loc New target location
     * 
     * @return true if target updated successfully
     * @return false if update failed
     * 
     * @details Updates the target from old to new location, maintaining smooth
     *          transition in guided mode navigation.
     */
    bool update_target_location(const Location &old_loc, const Location &new_loc) override;
    
    /**
     * @brief Set target position in NED frame
     * 
     * @param[in] target_pos Target position vector in NED frame (meters)
     * @param[in] use_yaw True to use yaw_deg parameter
     * @param[in] yaw_deg Target yaw in degrees
     * @param[in] use_yaw_rate True to use yaw_rate_degs parameter
     * @param[in] yaw_rate_degs Target yaw rate in degrees/second
     * @param[in] yaw_relative True if yaw is relative to current heading
     * @param[in] terrain_alt True if altitude is terrain-relative
     * 
     * @return true if target set successfully
     * @return false if target rejected
     * 
     * @details Commands guided mode with position target in North-East-Down coordinates.
     *          Provides full control over position and yaw/yaw rate.
     */
    bool set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt) override;
    
    /**
     * @brief Set target position and velocity in NED frame
     * 
     * @param[in] target_pos Target position vector in NED frame (meters)
     * @param[in] target_vel Target velocity vector in NED frame (m/s)
     * 
     * @return true if target set successfully
     * @return false if target rejected
     * 
     * @details Commands guided mode with both position and velocity targets.
     *          Position controller will track the velocity while converging to position.
     */
    bool set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel) override;
    
    /**
     * @brief Set target position, velocity, and acceleration in NED frame
     * 
     * @param[in] target_pos Target position vector in NED frame (meters)
     * @param[in] target_vel Target velocity vector in NED frame (m/s)
     * @param[in] target_accel Target acceleration vector in NED frame (m/s²)
     * @param[in] use_yaw True to use yaw_deg parameter
     * @param[in] yaw_deg Target yaw in degrees
     * @param[in] use_yaw_rate True to use yaw_rate_degs parameter
     * @param[in] yaw_rate_degs Target yaw rate in degrees/second
     * @param[in] yaw_relative True if yaw is relative to current heading
     * 
     * @return true if target set successfully
     * @return false if target rejected
     * 
     * @details Full feedforward control with position, velocity, and acceleration targets.
     *          Provides the smoothest trajectory tracking for advanced scripting applications.
     */
    bool set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative) override;
    
    /**
     * @brief Set target velocity in NED frame
     * 
     * @param[in] vel_ned Target velocity vector in NED frame (m/s)
     * 
     * @return true if velocity target set successfully
     * @return false if target rejected
     * 
     * @details Commands guided mode with pure velocity control (no position target).
     *          Vehicle will maintain the specified velocity.
     */
    bool set_target_velocity_NED(const Vector3f& vel_ned) override;
    
    /**
     * @brief Set target velocity and acceleration in NED frame
     * 
     * @param[in] target_vel Target velocity vector in NED frame (m/s)
     * @param[in] target_accel Target acceleration vector in NED frame (m/s²)
     * @param[in] use_yaw True to use yaw_deg parameter
     * @param[in] yaw_deg Target yaw in degrees
     * @param[in] use_yaw_rate True to use yaw_rate_degs parameter
     * @param[in] yaw_rate_degs Target yaw rate in degrees/second
     * @param[in] relative_yaw True if yaw is relative to current heading
     * 
     * @return true if target set successfully
     * @return false if target rejected
     * 
     * @details Velocity control with feedforward acceleration for smooth tracking.
     */
    bool set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw) override;
    
    /**
     * @brief Set target attitude angles and climb rate
     * 
     * @param[in] roll_deg Target roll angle in degrees
     * @param[in] pitch_deg Target pitch angle in degrees
     * @param[in] yaw_deg Target yaw angle in degrees
     * @param[in] climb_rate_ms Target climb rate in m/s
     * @param[in] use_yaw_rate True to use yaw_rate_degs instead of yaw angle
     * @param[in] yaw_rate_degs Target yaw rate in degrees/second
     * 
     * @return true if target set successfully
     * @return false if target rejected
     * 
     * @details Low-level attitude control for advanced scripting.
     *          Directly commands roll, pitch, yaw, and climb rate.
     *          
     * @warning Direct attitude control requires careful tuning and monitoring
     */
    bool set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs) override;
    
    /**
     * @brief Set target body rates and throttle
     * 
     * @param[in] roll_rate_dps Target roll rate in degrees/second
     * @param[in] pitch_rate_dps Target pitch rate in degrees/second
     * @param[in] yaw_rate_dps Target yaw rate in degrees/second
     * @param[in] throttle Target throttle (0.0 to 1.0)
     * 
     * @return true if target set successfully
     * @return false if target rejected
     * 
     * @details Lowest-level control interface - directly commands body rates and throttle.
     *          Bypasses position and attitude controllers.
     *          
     * @warning This is acro-mode style control. Vehicle will not self-level.
     *          Use with extreme caution.
     */
    bool set_target_rate_and_throttle(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps, float throttle) override;

    /**
     * @brief Register a custom flight mode from Lua script
     * 
     * @param[in] number Mode number for the custom mode
     * @param[in] full_name Full descriptive name
     * @param[in] short_name Short name for displays
     * 
     * @return Pointer to custom mode state structure
     * @return nullptr if registration failed
     * 
     * @details Allows Lua scripts to register custom flight modes that can be
     *          selected like built-in modes.
     */
    AP_Vehicle::custom_mode_state* register_custom_mode(const uint8_t number, const char* full_name, const char* short_name) override;
#endif
#if MODE_CIRCLE_ENABLED
    /**
     * @brief Get current circle mode radius
     * 
     * @param[out] radius_m Circle radius in meters
     * 
     * @return true if radius retrieved successfully
     * @return false if not in circle mode or radius unavailable
     * 
     * @details Queries the current circle navigation radius.
     */
    bool get_circle_radius(float &radius_m) override;
    
    /**
     * @brief Set circle mode turn rate
     * 
     * @param[in] rate_dps Turn rate in degrees/second
     * 
     * @return true if rate set successfully
     * @return false if rate rejected or not in circle mode
     * 
     * @details Changes the circle turn rate. Positive is clockwise.
     */
    bool set_circle_rate(float rate_dps) override;
#endif
    /**
     * @brief Set desired speed for autonomous modes
     * 
     * @param[in] speed Desired speed in m/s
     * 
     * @return true if speed set successfully
     * @return false if speed rejected
     * 
     * @details Changes the target speed for waypoint navigation in Auto/Guided modes.
     */
    bool set_desired_speed(float speed) override;
#if MODE_AUTO_ENABLED
    /**
     * @brief Enable navigation scripting mode
     * 
     * @param[in] mode Scripting nav mode to enable
     * 
     * @return true if scripting navigation enabled
     * @return false if enable failed
     * 
     * @details Allows Lua scripts to take over waypoint navigation in Auto mode.
     */
    bool nav_scripting_enable(uint8_t mode) override;
    
    /**
     * @brief Get navigation script timing command
     * 
     * @param[out] id Command ID
     * @param[out] cmd Command type
     * @param[out] arg1 Command argument 1
     * @param[out] arg2 Command argument 2
     * @param[out] arg3 Command argument 3
     * @param[out] arg4 Command argument 4
     * 
     * @return true if command available
     * @return false if no command pending
     * 
     * @details Retrieves navigation commands from mission for script processing.
     */
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;
    
    /**
     * @brief Signal completion of navigation script command
     * 
     * @param[in] id Command ID being completed
     * 
     * @details Notifies mission controller that script has finished processing command.
     */
    void nav_script_time_done(uint16_t id) override;
#endif
    /**
     * @brief Check if EKF failsafe has triggered
     * 
     * @return true if EKF failsafe is active
     * @return false if EKF is operating normally
     * 
     * @details Allows Lua scripts to query EKF failsafe state for custom handling.
     */
    bool has_ekf_failsafed() const override;
#endif // AP_SCRIPTING_ENABLED
    
    /**
     * @brief Check if vehicle is currently landing
     * 
     * @return true if vehicle is in landing sequence
     * @return false if not landing
     * 
     * @details Indicates if the vehicle is actively landing (Land mode, RTL final descent, etc.)
     */
    bool is_landing() const override;
    
    /**
     * @brief Check if vehicle is currently taking off
     * 
     * @return true if vehicle is in takeoff sequence
     * @return false if not taking off
     * 
     * @details Indicates if the vehicle is actively taking off (Auto takeoff, throw mode, etc.)
     */
    bool is_taking_off() const override;
    
    /**
     * @brief Process RC (Radio Control) input at main loop rate
     * 
     * @details Reads and processes pilot RC inputs for roll, pitch, yaw, and throttle channels.
     *          Updates RC channel values, applies deadzone filtering, and sets flags for
     *          new radio frame availability. Called at main loop rate (typically 400Hz).
     *          
     *          This function is critical for responsive pilot control and must execute
     *          quickly to maintain loop timing.
     * 
     * @note Called from main scheduler at high rate
     * @see read_radio() for lower-level RC input reading
     */
    void rc_loop();
    
    /**
     * @brief Process throttle input and hover learning
     * 
     * @details Updates throttle-related calculations including hover throttle learning
     *          and throttle curve application. Runs at a lower rate than rc_loop to
     *          reduce computational load while still maintaining smooth throttle response.
     * 
     * @note Called from scheduler at reduced rate
     * @see update_throttle_hover() for hover learning algorithm
     */
    void throttle_loop();
    
    /**
     * @brief Update battery monitoring and compass at medium rate
     * 
     * @details Reads battery voltage/current measurements and updates compass sensors.
     *          Running these updates at medium rate (typically 10Hz) is sufficient for
     *          battery state tracking and compass accuracy while reducing CPU load.
     * 
     * @note Called from scheduler at ~10Hz
     */
    void update_batt_compass(void);
    
    /**
     * @brief Log rate controller performance at fast rate
     * 
     * @details Records rate controller timing and performance metrics for debugging
     *          and analysis. Runs at fast rate to capture high-frequency controller behavior.
     * 
     * @note Called from fast rate loop
     */
    void loop_rate_logging();
    
    /**
     * @brief Execute 10Hz logging and housekeeping tasks
     * 
     * @details Logs various telemetry data at 10Hz including attitude, position, and
     *          system status. Also performs periodic system health checks and updates.
     * 
     * @note Called from scheduler at 10Hz
     */
    void ten_hz_logging_loop();
    
    /**
     * @brief Execute 25Hz logging tasks
     * 
     * @details Logs higher-rate telemetry data that requires more frequent sampling
     *          than the 10Hz loop but less than main loop rate.
     * 
     * @note Called from scheduler at 25Hz
     */
    void twentyfive_hz_logging();
    
    /**
     * @brief Execute 3Hz periodic tasks
     * 
     * @details Performs low-frequency updates including terrain data checks,
     *          compass calibration updates, and other slow-changing system state.
     * 
     * @note Called from scheduler at ~3.3Hz
     */
    void three_hz_loop();
    
    /**
     * @brief Execute 1Hz periodic tasks
     * 
     * @details Performs very low-frequency updates including compass declination
     *          updates, system statistics logging, and other once-per-second tasks.
     * 
     * @note Called from scheduler at 1Hz
     */
    void one_hz_loop();
    
    /**
     * @brief Initialize simple mode bearing reference
     * 
     * @details Sets up the initial heading reference for simple and super-simple flight modes.
     *          Records the vehicle's current heading as the reference direction for
     *          pilot input transformations in simple mode.
     *          
     *          In simple mode, pilot inputs are interpreted relative to this reference
     *          heading rather than the vehicle's current heading.
     * 
     * @note Called when arming or when super-simple mode exceeds distance threshold
     * @see update_simple_mode() for simple mode operation
     */
    void init_simple_bearing();
    
    /**
     * @brief Update simple mode coordinate transformation
     * 
     * @details Updates the coordinate transformation used to interpret pilot inputs
     *          in simple mode. Transforms pilot stick inputs from the simple mode
     *          reference frame to the vehicle body frame.
     * 
     * @note Called each loop iteration when simple mode is active
     * @see init_simple_bearing() for initialization
     */
    void update_simple_mode(void);
    
    /**
     * @brief Update super-simple mode bearing reference
     * 
     * @details In super-simple mode, the reference bearing is continuously updated
     *          based on the vehicle's position relative to home. When the vehicle
     *          moves beyond a threshold distance from its last reference point,
     *          a new reference bearing is calculated.
     * 
     * @param[in] force_update If true, forces immediate update of reference bearing
     * 
     * @note Super-simple mode automatically adjusts the control reference as vehicle moves
     */
    void update_super_simple_bearing(bool force_update);
    
    /**
     * @brief Read and update AHRS (Attitude Heading Reference System)
     * 
     * @details Retrieves the latest attitude, heading, and position estimates from
     *          the AHRS/EKF system. Updates vehicle attitude quaternion, Euler angles,
     *          rotation matrices, and navigation solution.
     *          
     *          This is a critical function that provides the foundation for all
     *          attitude and position control algorithms.
     * 
     * @note Called at main loop rate (typically 400Hz)
     * @see AP_AHRS for AHRS/EKF interface
     */
    void read_AHRS(void);
    
    /**
     * @brief Update altitude estimates from all sources
     * 
     * @details Combines barometric altitude, rangefinder measurements, and EKF altitude
     *          estimates to provide robust altitude information for control and navigation.
     *          Manages fusion of multiple altitude sources and handles sensor failures.
     * 
     * @note Called at main loop rate
     * @see read_barometer(), read_rangefinder() for sensor updates
     */
    void update_altitude();
    
    /**
     * @brief Get distance to active waypoint in meters
     * 
     * @details Returns the horizontal distance from the vehicle's current position
     *          to the active navigation waypoint. Used for mission monitoring and
     *          waypoint arrival detection.
     * 
     * @param[out] distance Distance to waypoint in meters
     * @return true if distance is valid and waypoint is active, false otherwise
     * 
     * @note Overrides AP_Vehicle::get_wp_distance_m()
     */
    bool get_wp_distance_m(float &distance) const override;
    
    /**
     * @brief Get bearing to active waypoint in degrees
     * 
     * @details Returns the compass bearing from the vehicle's current position to
     *          the active navigation waypoint. Bearing is in degrees (0-360) where
     *          0/360 is north, 90 is east, 180 is south, 270 is west.
     * 
     * @param[out] bearing Bearing to waypoint in degrees (0-360)
     * @return true if bearing is valid and waypoint is active, false otherwise
     * 
     * @note Overrides AP_Vehicle::get_wp_bearing_deg()
     */
    bool get_wp_bearing_deg(float &bearing) const override;
    
    /**
     * @brief Get cross-track error from planned path in meters
     * 
     * @details Returns the perpendicular distance between the vehicle and the
     *          planned navigation path between waypoints. Positive values indicate
     *          the vehicle is to the right of the path, negative values to the left.
     *          
     *          Cross-track error is used for path following and to determine when
     *          the vehicle has strayed too far from the planned route.
     * 
     * @param[out] xtrack_error Cross-track error in meters (+ = right of path)
     * @return true if error is valid and path following is active, false otherwise
     * 
     * @note Overrides AP_Vehicle::get_wp_crosstrack_error_m()
     */
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;
    
    /**
     * @brief Get rate controller target rates in earth frame
     * 
     * @details Returns the target angular rates (roll/pitch/yaw) in earth frame
     *          coordinates (NED - North-East-Down). Used for telemetry and logging
     *          to show commanded vs actual vehicle rotation rates.
     * 
     * @param[out] rate_ef_targets Target rates in earth frame (rad/s)
     * @return true if rates are available, false otherwise
     * 
     * @note Overrides AP_Vehicle::get_rate_ef_targets()
     * @see AC_AttitudeControl for rate controller implementation
     */
    bool get_rate_ef_targets(Vector3f& rate_ef_targets) const override;

    // Attitude.cpp
    /**
     * @brief Update hover throttle estimate through learning
     * 
     * @details Uses a low-pass filter to learn the average throttle required to maintain
     *          hover based on actual throttle values when the vehicle is in a stable hover.
     *          The learned hover throttle improves altitude hold performance and position
     *          control by providing better throttle feed-forward.
     *          
     *          Learning occurs when:
     *          - Vehicle is in altitude hold or position control mode
     *          - Vertical velocity is near zero
     *          - Vehicle is not landing or taking off
     *          - Motors are armed and flying
     *          
     *          The learned hover throttle is stored in the THR_MID parameter over time.
     * 
     * @note Called periodically from throttle_loop()
     * @see get_pilot_desired_climb_rate() which uses hover throttle for altitude control
     */
    void update_throttle_hover();
    
    /**
     * @brief Calculate desired climb rate from pilot throttle input
     * 
     * @details Converts pilot throttle stick position to a desired climb rate in cm/s.
     *          Uses the learned hover throttle as the center point, with climb commanded
     *          above hover and descent below. Applies exponential curve and deadzone
     *          around hover position for precise altitude control.
     *          
     *          Calculation accounts for:
     *          - Configured climb rate limits (PILOT_SPEED_UP, PILOT_SPEED_DN)
     *          - Throttle expo curve for smooth control feel
     *          - Deadzone around hover position
     *          - Takeoff thrust boost when near ground
     * 
     * @return Desired climb rate in cm/s (positive = up, negative = down)
     * 
     * @note Called from altitude hold and position control modes
     * @see update_throttle_hover() for hover throttle learning
     */
    float get_pilot_desired_climb_rate();
    
    /**
     * @brief Get non-takeoff throttle value
     * 
     * @details Returns the throttle value to use before actual takeoff - typically
     *          a low value that spins motors but doesn't generate enough thrust to lift.
     *          Used during arming delay and pre-flight checks to verify motor operation
     *          without launching the vehicle.
     * 
     * @return Throttle value (0-1 range) for non-takeoff motor output
     * 
     * @note Used during arming sequence and motor tests
     */
    float get_non_takeoff_throttle();
    
    /**
     * @brief Set altitude controller throttle integrator from pilot input
     * 
     * @details When the pilot manually controls throttle, this function initializes
     *          the altitude controller's throttle integrator to match the pilot's
     *          input. This prevents sudden throttle changes when switching from
     *          manual to automatic altitude control modes.
     *          
     *          Ensures smooth transitions between flight modes by preserving
     *          throttle continuity.
     * 
     * @note Called when transitioning from manual to automatic altitude control
     * @see AC_PosControl for altitude controller implementation
     */
    void set_accel_throttle_I_from_pilot_throttle();
    
    /**
     * @brief Rotate body frame coordinates to North-East frame
     * 
     * @details Transforms coordinates from the vehicle body frame (forward-right-down)
     *          to the North-East horizontal plane of the earth frame. Uses current
     *          vehicle yaw angle for rotation. Commonly used to convert pilot stick
     *          inputs from body frame to earth frame for navigation control.
     *          
     *          Does not affect vertical (down) component - only rotates horizontal plane.
     * 
     * @param[in,out] x Body frame forward coordinate (becomes North in earth frame)
     * @param[in,out] y Body frame right coordinate (becomes East in earth frame)
     * 
     * @note Body frame: X=forward, Y=right; Earth frame: X=North, Y=East
     * @see rotate_body_frame_to_NE() is inverse of this transformation
     */
    void rotate_body_frame_to_NE(float &x, float &y);
    
    /**
     * @brief Get pilot's desired descent speed in cm/s
     * 
     * @details Returns the maximum descent speed configured by the PILOT_SPEED_DN
     *          parameter. Used to limit how fast the vehicle descends in response
     *          to pilot input, preventing excessive descent rates that could lead
     *          to hard landings or loss of control.
     * 
     * @return Maximum descent speed in cm/s (positive value)
     * 
     * @note Typical values are 100-250 cm/s (1-2.5 m/s)
     */
    uint16_t get_pilot_speed_dn() const;
    
    /**
     * @brief Execute main rate controller
     * 
     * @details Runs the attitude rate controller that converts desired angular rates
     *          into motor outputs. This is the innermost control loop running at the
     *          highest frequency (typically 400Hz or higher with fast rate loop).
     *          
     *          Rate controller:
     *          1. Compares desired rates to actual rates from gyros
     *          2. Runs PID controllers for roll/pitch/yaw rates
     *          3. Applies control limits and anti-windup
     *          4. Outputs motor commands via mixer
     *          
     *          This is a time-critical function that must execute with minimal jitter.
     * 
     * @warning Critical flight control function - any changes must be thoroughly tested
     * @note Called at main loop rate or fast rate loop (400-2000Hz)
     * @see AC_AttitudeControl_Multi for rate controller implementation
     */
    void run_rate_controller_main();

    // if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    /**
     * @brief Rate controller execution frequencies configuration
     * 
     * @details Holds the calculated execution rates for different parts of the fast
     *          rate controller system. When fast sample mode is enabled, the rate
     *          controller runs at a higher frequency than the main loop, requiring
     *          careful management of different update rates for logging, filtering,
     *          and control execution.
     */
    struct RateControllerRates {
        uint8_t fast_logging_rate;    /**< @brief Rate for fast logging (Hz) */
        uint8_t medium_logging_rate;  /**< @brief Rate for medium logging (Hz) */
        uint8_t filter_rate;          /**< @brief Rate for filter updates (Hz) */
        uint8_t main_loop_rate;       /**< @brief Main loop execution rate (Hz) */
    };

    /**
     * @brief Calculate gyro sample decimation factor
     * 
     * @details Computes how many gyro samples to skip (decimate) to achieve the
     *          desired output rate. Used to downsample high-frequency gyro data
     *          to match controller execution rates.
     * 
     * @param[in] gyro_decimation Current decimation setting
     * @param[in] rate_hz Desired output rate in Hz
     * @return Calculated decimation factor
     * 
     * @note Higher decimation = lower output rate
     */
    uint8_t calc_gyro_decimation(uint8_t gyro_decimation, uint16_t rate_hz);
    
    /**
     * @brief Fast rate controller thread execution
     * 
     * @details Main execution function for the fast rate controller thread. When
     *          enabled, this runs in a separate high-priority thread at rates up to
     *          2kHz, providing improved attitude control performance through higher
     *          frequency gyro sampling and rate controller execution.
     *          
     *          The fast rate thread runs the attitude rate controller at a multiple
     *          of the main loop rate, reducing phase lag and improving disturbance
     *          rejection for aggressive flight.
     * 
     * @warning High CPU load - only runs on capable autopilot hardware
     * @note Runs in separate thread when fast rate loop is enabled
     * @see enable_fast_rate_loop() to start fast rate operation
     */
    void rate_controller_thread();
    
    /**
     * @brief Update filters at fast rate
     * 
     * @details Updates gyro filters (notch filters, low-pass filters) at the fast
     *          rate controller frequency. Filter updates must run synchronously with
     *          gyro sampling to maintain filter stability and effectiveness.
     * 
     * @note Called from fast rate controller thread
     */
    void rate_controller_filter_update();
    
    /**
     * @brief Update logging at fast rate
     * 
     * @details Records rate controller performance data at fast rate for detailed
     *          analysis and tuning. Logs rate controller inputs, outputs, and errors
     *          at higher frequency than standard logging.
     * 
     * @note Called from fast rate controller thread at configured logging rate
     */
    void rate_controller_log_update();
    
    /**
     * @brief Configure rate controller execution frequencies
     * 
     * @details Calculates and sets the execution rates for different fast rate
     *          controller tasks based on the gyro decimation factor and available
     *          CPU resources. Balances performance with CPU load.
     * 
     * @param[in] rate_decimation Gyro sample decimation factor
     * @param[out] rates Calculated rate configuration
     * @param[in] warn_cpu_high If true, warns user if CPU load may be too high
     * 
     * @note Adjusts rates based on autopilot performance capabilities
     */
    void rate_controller_set_rates(uint8_t rate_decimation, RateControllerRates& rates, bool warn_cpu_high);
    
    /**
     * @brief Enable fast rate controller loop
     * 
     * @details Activates the fast rate controller thread running at higher frequency
     *          than the main loop. Configures thread priority, execution rates, and
     *          starts high-frequency gyro sampling. Improves control performance for
     *          aggressive flying but increases CPU load significantly.
     *          
     *          Enabling fast rate loop:
     *          - Creates high-priority thread for rate controller
     *          - Increases gyro sampling to match fast loop rate
     *          - Configures filter and logging rates
     *          - Validates CPU has sufficient performance
     * 
     * @param[in] rate_decimation Gyro decimation factor for fast loop
     * @param[in] rates Execution rate configuration
     * 
     * @warning High CPU usage - may not be supported on all hardware
     * @note Configured via INS_FAST_SAMPLE parameter
     * @see disable_fast_rate_loop() to stop fast rate operation
     */
    void enable_fast_rate_loop(uint8_t rate_decimation, RateControllerRates& rates);
    
    /**
     * @brief Disable fast rate controller loop
     * 
     * @details Stops the fast rate controller thread and returns to standard main
     *          loop rate control. Safely shuts down high-frequency operation and
     *          restores normal gyro sampling rates.
     * 
     * @param[in] rates Rate configuration to restore
     * 
     * @note Called when fast rate loop is disabled or on CPU overload
     * @see enable_fast_rate_loop() to start fast rate operation
     */
    void disable_fast_rate_loop(RateControllerRates& rates);
    
    /**
     * @brief Update dynamic notch filter at specified rate
     * 
     * @details Runs the dynamic notch filter analysis and update from the main loop.
     *          The dynamic notch filter automatically detects and filters vibration
     *          frequencies in the gyro data, improving attitude control by removing
     *          resonances from motor vibrations and structural modes.
     *          
     *          Dynamic notch filter continuously analyzes gyro FFT data to:
     *          - Detect dominant vibration frequencies
     *          - Position notch filters at those frequencies
     *          - Update filter parameters as vibration patterns change
     * 
     * @note Called at configured rate from main loop
     * @see AP_InertialSensor for dynamic notch filter implementation
     */
    void update_dynamic_notch_at_specified_rate_main();
    // endif AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

#if AC_CUSTOMCONTROL_MULTI_ENABLED
    /**
     * @brief Execute custom controller update
     * 
     * @details Runs the user-defined custom control system that can override or
     *          supplement the standard ArduPilot attitude and rate controllers.
     *          Custom controllers allow advanced users to implement specialized
     *          control algorithms for research or unique vehicle configurations.
     *          
     *          Custom control can operate in different modes:
     *          - Override: Replace standard controllers completely
     *          - Supplement: Add corrections to standard controller outputs
     *          - Selective: Override only certain axes
     * 
     * @note Only available when AC_CustomControl is compiled in
     * @see AC_CustomControl for custom controller implementation
     */
    void run_custom_controller() { custom_control.update(); }
#endif

    // avoidance.cpp
    /**
     * @brief Execute low altitude avoidance
     * 
     * @details Prevents vehicle from descending below a configured minimum altitude
     *          when flying in modes that don't inherently respect altitude limits.
     *          Uses rangefinder or barometric altitude to enforce minimum clearance
     *          above ground or obstacles.
     *          
     *          Low altitude avoidance:
     *          - Monitors current altitude against configured minimum
     *          - Limits descent rate when approaching minimum altitude
     *          - Prevents further descent when at minimum altitude
     *          - Provides gentle altitude floor without abrupt stops
     *          
     *          Used in manual flight modes to prevent ground strikes while
     *          still allowing pilot control.
     * 
     * @note Called from main loop in appropriate flight modes
     * @see AC_Avoidance for comprehensive avoidance system
     */
    void low_alt_avoidance();

#if HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
    // avoidance_adsb.cpp
    /**
     * @brief Update ADS-B based avoidance system
     * 
     * @details Monitors ADS-B transponder data from nearby aircraft and implements
     *          avoidance maneuvers to prevent mid-air collisions with manned aircraft.
     *          Analyzes relative positions, velocities, and trajectories to predict
     *          potential conflicts and execute avoidance actions.
     *          
     *          ADS-B avoidance system:
     *          1. Receives aircraft position/velocity from ADS-B receiver
     *          2. Predicts future positions and potential conflicts
     *          3. Calculates safe avoidance vectors
     *          4. Executes avoidance maneuvers (typically RTL or LAND)
     *          5. Monitors until conflict resolved
     *          
     *          Critical safety system for operation in airspace with manned traffic.
     * 
     * @warning Safety-critical function for airspace integration
     * @note Called periodically from main loop
     * @see AP_ADSB for ADS-B receiver interface
     * @see AP_Avoidance_Copter for avoidance action implementation
     */
    void avoidance_adsb_update(void);
#endif  // HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED

    // baro_ground_effect.cpp
    /**
     * @brief Update ground effect detector state
     * 
     * @details Monitors for aerodynamic ground effect - the increased lift and
     *          altered pressure field that occurs when a multicopter operates close
     *          to the ground (typically within 1 rotor diameter). Ground effect
     *          affects barometric altitude readings and vehicle performance.
     *          
     *          Ground effect detection tracks:
     *          - Takeoff state: Expecting ground effect during takeoff
     *          - Touchdown state: Expecting ground effect during landing
     *          - Altitude at takeoff: Reference for ground effect zone
     *          - Time in ground effect: Duration near ground
     *          
     *          This information is used to:
     *          - Adjust barometric altitude corrections
     *          - Modify altitude controller gains near ground
     *          - Improve landing detection accuracy
     *          - Compensate for ground effect in position control
     * 
     * @note Called periodically from main loop
     * @see gndeffect_state for ground effect state variables
     */
    void update_ground_effect_detector(void);
    
    /**
     * @brief Update EKF terrain height stability estimate
     * 
     * @details Monitors the stability of the EKF terrain height estimate and updates
     *          a flag indicating whether terrain following can safely rely on the
     *          current terrain data. Terrain height stability is critical for safe
     *          terrain following operations.
     *          
     *          Terrain height is considered stable when:
     *          - EKF has valid terrain estimate
     *          - Terrain estimate variance is low
     *          - Terrain data is recent and updating
     *          - No terrain database gaps detected
     *          
     *          Used by terrain following modes to determine if terrain-relative
     *          flight is safe or if mode should revert to altitude-above-home.
     * 
     * @note Called from terrain update functions
     * @see AP_Terrain for terrain database system
     */
    void update_ekf_terrain_height_stable();

    // commands.cpp
    /**
     * @brief Update home position from EKF origin
     * 
     * @details Synchronizes the stored home position with the EKF's origin when the
     *          EKF origin is set or reset. This ensures home position matches the
     *          coordinate system used by the navigation system.
     *          
     *          Called when:
     *          - EKF origin is first established (GPS lock acquired)
     *          - EKF origin is reset (lane switch or reset event)
     *          - Position estimate becomes valid after being invalid
     *          
     *          Maintaining home position alignment with EKF origin is critical for
     *          accurate RTL (Return To Launch) and position hold operations.
     * 
     * @note Called automatically when EKF origin changes
     * @see AP_AHRS for EKF origin management
     */
    void update_home_from_EKF();
    
    /**
     * @brief Set home position to current location while in flight
     * 
     * @details Updates the home position to the vehicle's current location while
     *          flying. Used for missions or operations where the takeoff point
     *          should be updated mid-flight. This affects RTL destination and
     *          altitude reference.
     *          
     *          In-flight home setting is typically triggered by:
     *          - Pilot command via RC switch
     *          - GCS command
     *          - Mission command
     *          - Scripting command
     *          
     *          Setting home in-flight updates:
     *          - Home position coordinates
     *          - Home altitude reference
     *          - RTL destination
     *          - Distance-from-home calculations
     * 
     * @warning Changes RTL destination - pilot must understand new return point
     * @note Only allowed when vehicle has valid position estimate
     * @see set_home() for setting home to specific location
     */
    void set_home_to_current_location_inflight();
    
    /**
     * @brief Set home position to current vehicle location
     * 
     * @details Sets the home position to the vehicle's current GPS location. This
     *          establishes the return point for RTL and the altitude reference for
     *          relative altitude control. Can be called on the ground or in flight.
     *          
     *          Home position is automatically set:
     *          - At arming if not previously set
     *          - When GPS first acquires lock
     *          - Can be manually set by pilot or GCS
     * 
     * @param[in] lock If true, prevents automatic home position updates
     * @return true if home position was successfully set, false if position invalid
     * 
     * @warning Changes RTL destination and altitude reference
     * @note Requires valid position estimate from EKF
     * @see set_home() to set home to specific coordinates
     */
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Set home position to specified location
     * 
     * @details Sets the home position to a specified GPS location rather than
     *          current vehicle position. Used when home should be at a different
     *          location than the takeoff point, such as a designated landing zone
     *          or recovery point.
     *          
     *          Setting custom home location:
     *          - Validates location has valid lat/lon/alt
     *          - Updates EKF home position
     *          - Recalculates distance and bearing to home
     *          - Updates RTL destination
     *          - Logs home position change
     * 
     * @param[in] loc Location to set as home position
     * @param[in] lock If true, prevents automatic home position updates
     * @return true if home position was successfully set, false if location invalid
     * 
     * @warning Changes RTL destination and altitude reference
     * @note Location must have valid coordinates
     * @see set_home_to_current_location() to use current position
     */
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;

    // compassmot.cpp
    /**
     * @brief Perform compass motor interference calibration
     * 
     * @details Executes the compassmot calibration procedure to measure and compensate
     *          for magnetic interference from motor currents. The vehicle slowly ramps
     *          up throttle while measuring compass readings and current draw to calculate
     *          compensation parameters.
     *          
     *          CompassMot calibration procedure:
     *          1. Vehicle must be disarmed and secured (can't fly)
     *          2. Slowly increases throttle from zero to full
     *          3. Measures compass heading changes vs motor current
     *          4. Calculates compensation matrix (COMPASS_MOT_X/Y/Z parameters)
     *          5. Stores compensation values for automatic correction
     *          
     *          Proper compass motor calibration is critical for accurate heading
     *          in high-power flight conditions.
     * 
     * @param[in] gcs_chan GCS MAVLink channel for progress reporting
     * @return MAV_RESULT indicating success or failure reason
     * 
     * @warning Vehicle must be secured - motors will spin up to full throttle
     * @note Requires current sensor for accurate calibration
     * @see AP_Compass for compass interference compensation
     */
    MAV_RESULT mavlink_compassmot(const GCS_MAVLINK &gcs_chan);

    // crash_check.cpp
    /**
     * @brief Check for vehicle crash and trigger crash failsafe
     * 
     * @details Monitors vehicle behavior to detect crash conditions during flight.
     *          A crash is detected when the vehicle experiences high angular rates
     *          or unusual attitudes while armed, indicating loss of control or
     *          impact with terrain or obstacles.
     *          
     *          Crash detection criteria:
     *          - High sustained roll or pitch angle (>30 degrees typically)
     *          - Low climb rate or descent
     *          - Motors at high output but not responding to control
     *          - Sustained duration to avoid false triggers
     *          
     *          When crash detected:
     *          - Motors are immediately disarmed
     *          - Prevents continued operation of damaged vehicle
     *          - Logs crash event for analysis
     *          - Notifies pilot/GCS of crash condition
     * 
     * @warning Safety-critical - immediately disarms on detection
     * @note Called periodically from main loop when armed
     * @see crash_check.cpp for crash detection algorithm details
     */
    void crash_check();
    
    /**
     * @brief Check for loss of thrust and trigger thrust loss failsafe
     * 
     * @details Detects complete or partial loss of thrust when motors are commanded
     *          high but vehicle is descending rapidly. Indicates motor failure, ESC
     *          failure, propeller loss, or severe power system problems.
     *          
     *          Thrust loss detection:
     *          - Motors commanded to high throttle (>50%)
     *          - Vehicle descending rapidly (>1 m/s)
     *          - Condition sustained for configured duration
     *          - Not in intentional descent (landing mode)
     *          
     *          Thrust loss failsafe actions:
     *          - Can trigger parachute deployment
     *          - Can trigger motor disarm
     *          - Logs thrust loss event
     *          - Alerts pilot/GCS
     *          
     *          Critical for detecting catastrophic failures in flight.
     * 
     * @warning Safety-critical failsafe function
     * @note Can be disabled via FLIGHT_OPTIONS parameter
     * @see FlightOption::DISABLE_THRUST_LOSS_CHECK to disable
     */
    void thrust_loss_check();
    
    /**
     * @brief Check for yaw imbalance indicating motor or ESC failure
     * 
     * @details Monitors yaw axis integrator buildup which can indicate asymmetric
     *          thrust from motor or ESC failure. When one motor produces less thrust,
     *          the yaw controller integrator builds up trying to compensate, providing
     *          early warning of motor system problems.
     *          
     *          Yaw imbalance detection:
     *          - Monitors yaw PID integrator magnitude
     *          - Filters integrator to avoid transient spikes
     *          - Sustained high integrator indicates asymmetric thrust
     *          - Warns pilot of potential motor failure
     *          
     *          Early detection allows pilot to land before complete failure.
     * 
     * @note Can be disabled via FLIGHT_OPTIONS parameter
     * @see FlightOption::DISABLE_YAW_IMBALANCE_WARNING to disable
     * @see yaw_I_filt for filtered integrator value
     */
    void yaw_imbalance_check();
    
    LowPassFilterFloat yaw_I_filt{0.05f};  /**< @brief Filtered yaw integrator for imbalance detection */
    uint32_t last_yaw_warn_ms;             /**< @brief System time of last yaw imbalance warning (ms) */
    
    /**
     * @brief Check conditions for automatic parachute deployment
     * 
     * @details Monitors for conditions that warrant automatic parachute deployment:
     *          thrust loss, high descent rates, or crash detection while airborne.
     *          Parachute provides emergency recovery when normal flight is impossible.
     *          
     *          Automatic deployment triggers:
     *          - Thrust loss detected (motors high, descending rapidly)
     *          - Crash detected while airborne
     *          - Extreme attitude while airborne
     *          - Configured to deploy on failsafe actions
     *          
     *          Once deployed, parachute cannot be retracted - this is a one-way
     *          emergency recovery action.
     * 
     * @warning One-time emergency deployment only
     * @note Requires parachute hardware and AP_Parachute enabled
     * @see parachute_release() for deployment execution
     */
    void parachute_check();
    
    /**
     * @brief Deploy parachute for emergency recovery
     * 
     * @details Triggers parachute deployment by activating the configured release
     *          mechanism (servo or relay). Also commands motors to low throttle
     *          and disarms to prevent interference with parachute descent.
     *          
     *          Deployment sequence:
     *          1. Activates release mechanism
     *          2. Reduces motor throttle
     *          3. Disarms motors after brief delay
     *          4. Logs deployment event
     *          5. Alerts pilot/GCS
     * 
     * @warning Irreversible emergency action - vehicle will descend under parachute
     * @note Called by parachute_check() or manual pilot command
     * @see AP_Parachute for release mechanism control
     */
    void parachute_release();
    
    /**
     * @brief Handle manual pilot-commanded parachute deployment
     * 
     * @details Processes pilot request to manually deploy parachute via RC switch
     *          or GCS command. Includes safety checks to prevent accidental deployment
     *          on ground or at low altitude.
     *          
     *          Manual deployment checks:
     *          - Vehicle must be armed
     *          - Vehicle must be airborne
     *          - Sufficient altitude for parachute deployment
     *          - Pilot must hold switch for configured duration
     * 
     * @warning Irreversible emergency action
     * @note Requires explicit pilot action (not automatic)
     * @see parachute_release() for actual deployment
     */
    void parachute_manual_release();

    // ekf_check.cpp
    /**
     * @brief Monitor EKF health and trigger failsafe if necessary
     * 
     * @details Continuously monitors the Extended Kalman Filter (EKF) health by checking
     *          variance levels, innovation test failures, and consistency checks. The EKF
     *          provides the vehicle's position and velocity estimates - if these become
     *          unreliable, flight control degrades and failsafe action is required.
     *          
     *          EKF health monitoring:
     *          - Position and velocity variance levels
     *          - Innovation (measurement prediction error) magnitudes
     *          - Consistency between multiple EKF cores/lanes
     *          - Timeout on position updates
     *          - Sensor health (GPS, baro, compass, rangefinder)
     *          
     *          Filtered variance checking prevents transient errors from triggering
     *          failsafe, but responds quickly to sustained problems.
     * 
     * @note Called periodically from main loop (typically 100Hz)
     * @see ekf_over_threshold() for threshold checking logic
     * @see failsafe_ekf_event() for failsafe trigger
     */
    void ekf_check();
    
    /**
     * @brief Check if EKF variance exceeds failsafe thresholds
     * 
     * @details Tests filtered EKF position and velocity variance against configured
     *          thresholds. Returns true if variance indicates the position estimate
     *          is no longer reliable enough for safe autonomous flight.
     *          
     *          Uses low-pass filtered variance to avoid false triggers from brief
     *          transients while still responding to sustained variance increases.
     * 
     * @return true if EKF variance exceeds failsafe threshold, false otherwise
     * 
     * @note Thresholds configured via FS_EKF_THRESH parameter
     * @see pos_variance_filt for filtered position variance
     * @see vel_variance_filt for filtered velocity variance
     */
    bool ekf_over_threshold();
    
    /**
     * @brief Trigger EKF failsafe event
     * 
     * @details Activates EKF failsafe when position/velocity estimates become unreliable.
     *          Takes immediate action to prevent vehicle from continuing autonomous
     *          navigation with bad position data.
     *          
     *          EKF failsafe actions (configured by FS_EKF_ACTION):
     *          - Land immediately at current location
     *          - RTL if alternate position source available
     *          - Altitude hold and drift (if position lost but altitude OK)
     *          - Disarm if on ground
     *          
     *          Failsafe remains active until EKF health recovers and recheck passes.
     * 
     * @warning Safety-critical - immediately affects flight mode and control
     * @note Logged as failsafe event for analysis
     * @see failsafe_ekf_off_event() for failsafe recovery
     */
    void failsafe_ekf_event();
    
    /**
     * @brief Handle EKF failsafe recovery
     * 
     * @details Called when EKF health recovers after a failsafe condition. Determines
     *          whether to clear failsafe and potentially restore previous flight mode
     *          or maintain failsafe action until pilot intervention.
     *          
     *          Recovery handling:
     *          - Verifies EKF health sustainably recovered
     *          - Checks if safe to resume normal operation
     *          - May restore previous flight mode or wait for pilot
     *          - Logs recovery event
     * 
     * @note Requires sustained good EKF health to clear failsafe
     * @see failsafe_ekf_recheck() for periodic recheck logic
     */
    void failsafe_ekf_off_event(void);
    
    /**
     * @brief Periodically recheck EKF health during failsafe
     * 
     * @details While in EKF failsafe, continues monitoring EKF health to detect
     *          recovery. If health improves sufficiently, may clear failsafe and
     *          allow resumption of normal navigation.
     *          
     *          Prevents unnecessary permanent failsafe state when EKF recovers
     *          from temporary issues like brief GPS glitch or sensor spike.
     * 
     * @note Called periodically while EKF failsafe is active
     * @see failsafe_ekf_off_event() called when recovery detected
     */
    void failsafe_ekf_recheck();
    
    /**
     * @brief Check for and handle EKF position reset events
     * 
     * @details Monitors for sudden position resets from the EKF (origin change, lane
     *          switch, GPS glitch recovery). When a reset occurs, updates navigation
     *          targets and controllers to prevent vehicle from attempting to fly back
     *          to pre-reset position.
     *          
     *          EKF reset handling:
     *          - Detects yaw, position, or velocity resets
     *          - Adjusts navigation targets by reset delta
     *          - Updates position controller setpoints
     *          - Logs reset events for analysis
     *          - Prevents control transients from reset
     *          
     *          Proper reset handling is critical for smooth flight through GPS
     *          glitches and EKF lane switches.
     * 
     * @note Called periodically from main loop
     * @see AP_AHRS for EKF reset detection
     */
    void check_ekf_reset();
    
    /**
     * @brief Check vibration levels and warn pilot if excessive
     * 
     * @details Monitors IMU vibration levels and sets flags when vibration exceeds
     *          acceptable limits. High vibration can cause:
     *          - Poor attitude control and drift
     *          - EKF failures and position errors
     *          - Accelerometer clipping
     *          - Compass interference
     *          
     *          Vibration monitoring:
     *          - Tracks vibration magnitude on each axis
     *          - Compares to threshold for normal operation
     *          - Sets warning flags when excessive
     *          - Alerts pilot via GCS and LED indicators
     *          - Logs vibration levels
     *          
     *          Persistent high vibration requires mechanical fixes (balance props,
     *          soft-mount FC, replace damaged components).
     * 
     * @warning High vibration degrades flight performance and safety
     * @note Vibration data logged in VIBE messages
     * @see vibration_check struct for vibration state
     */
    void check_vibration();

    // esc_calibration.cpp
    /**
     * @brief Check for ESC calibration mode at startup
     * 
     * @details During ArduPilot initialization, checks if ESC calibration mode is
     *          requested via the ESC_CALIBRATION parameter. If enabled, enters the
     *          appropriate calibration mode instead of normal flight operations.
     *          
     *          ESC calibration modes:
     *          - PASSTHROUGH_IF_THROTTLE_HIGH: Calibrate if throttle stick high
     *          - PASSTHROUGH_ALWAYS: Always enter passthrough calibration
     *          - AUTO: Automatic calibration sequence
     *          - DISABLED: Normal operation, no calibration
     *          
     *          ESC calibration teaches ESCs the PWM range (min/max throttle values)
     *          for proper motor control and synchronized startup.
     * 
     * @warning Vehicle must be disarmed and propellers removed for safety
     * @note Called once during init_ardupilot()
     * @see ESCCalibrationModes enum for calibration mode options
     */
    void esc_calibration_startup_check();
    
    /**
     * @brief Execute passthrough ESC calibration
     * 
     * @details Implements passthrough calibration mode where RC throttle input is
     *          passed directly to all ESCs simultaneously. Used for traditional ESC
     *          calibration procedure:
     *          1. Throttle stick high at power-on
     *          2. ESCs enter calibration mode and store max throttle
     *          3. Throttle stick to minimum
     *          4. ESCs store min throttle and exit calibration
     *          
     *          All motors receive identical PWM signal to ensure synchronized
     *          calibration across all ESCs.
     * 
     * @warning Remove propellers - motors will spin at full throttle
     * @warning Vehicle cannot fly during calibration - ESCs in special mode
     * @note Requires compatible ESCs that support PWM calibration
     * @see esc_calibration_startup_check() for mode selection
     */
    void esc_calibration_passthrough();
    
    /**
     * @brief Execute automatic ESC calibration sequence
     * 
     * @details Performs automated ESC calibration without requiring manual throttle
     *          manipulation. ArduPilot automatically generates the high-low throttle
     *          sequence needed for ESC calibration.
     *          
     *          Automatic calibration sequence:
     *          1. Sends high PWM signal to all ESCs
     *          2. ESCs recognize calibration mode
     *          3. Waits for ESC confirmation
     *          4. Sends low PWM signal to all ESCs
     *          5. ESCs store calibration and exit
     *          6. Normal operation resumes
     * 
     * @warning Remove propellers - motors will spin
     * @note More convenient than passthrough but less common in practice
     * @see esc_calibration_startup_check() for mode selection
     */
    void esc_calibration_auto();
    
    /**
     * @brief Update pilot notifications during ESC calibration
     * 
     * @details Provides visual and audible feedback to pilot during ESC calibration
     *          process. Updates LED patterns and tones to indicate calibration
     *          status and guide pilot through calibration steps.
     *          
     *          Notification patterns indicate:
     *          - Calibration mode active (distinct LED pattern)
     *          - Waiting for throttle high (passthrough mode)
     *          - Waiting for throttle low (passthrough mode)
     *          - Calibration complete
     *          - Calibration error
     * 
     * @note Called periodically during calibration
     * @see AP_Notify for LED and tone control
     */
    void esc_calibration_notify();
    
    /**
     * @brief Initialize ESC calibration configuration
     * 
     * @details Prepares the system for ESC calibration by disabling safety checks,
     *          configuring motor output, and setting up the appropriate calibration
     *          mode. Ensures vehicle is in safe state for calibration.
     *          
     *          Setup tasks:
     *          - Verify vehicle is disarmed
     *          - Disable arming checks that would prevent calibration
     *          - Configure motor output for direct PWM control
     *          - Initialize calibration state machine
     *          - Prepare notification system
     * 
     * @warning Only call when ESC calibration is explicitly requested
     * @note Called before entering calibration mode
     */
    void esc_calibration_setup();

    // events.cpp
    /**
     * @brief Check if specific failsafe option is enabled
     * 
     * @details Tests whether a specific failsafe behavior option is enabled in the
     *          FS_OPTIONS bitmask parameter. Failsafe options modify default failsafe
     *          behavior for specific flight modes or conditions.
     * 
     * @param[in] opt Failsafe option to check (from FailsafeOption enum)
     * @return true if option is enabled, false otherwise
     * 
     * @note Common options: continue mission if in Auto, continue if landing, etc.
     * @see FailsafeOption enum for available options
     */
    bool failsafe_option(FailsafeOption opt) const;
    
    /**
     * @brief Trigger radio control failsafe event
     * 
     * @details Activates radio failsafe when RC signal is lost. Takes immediate action
     *          to safely handle loss of pilot control input. Radio failsafe is one of
     *          the most critical safety mechanisms.
     *          
     *          Radio failsafe trigger conditions:
     *          - RC signal lost for configured duration (FS_THR_ENABLE)
     *          - Throttle below failsafe threshold
     *          - No valid RC updates for timeout period
     *          
     *          Failsafe action depends on flight mode and configuration:
     *          - Land immediately (safest default)
     *          - RTL (return to launch)
     *          - Continue mission (if FS_OPTIONS allows)
     *          - SmartRTL if path recorded
     * 
     * @warning Safety-critical - activates when pilot cannot control vehicle
     * @note Action configured by FS_THR_ENABLE parameter
     * @see failsafe_radio_off_event() for recovery
     */
    void failsafe_radio_on_event();
    
    /**
     * @brief Handle radio control failsafe recovery
     * 
     * @details Called when RC signal recovers after radio failsafe. Determines whether
     *          to restore pilot control or maintain failsafe action based on vehicle
     *          state and configuration.
     *          
     *          Recovery options:
     *          - Restore pilot control if still in appropriate mode
     *          - Continue failsafe landing if landing already initiated
     *          - Allow mode change via RC input
     *          - Log recovery event
     * 
     * @note May or may not restore previous mode depending on configuration
     * @see failsafe_radio_on_event() for failsafe trigger
     */
    void failsafe_radio_off_event();
    
    /**
     * @brief Handle battery failsafe event
     * 
     * @details Processes battery failsafe triggered by low voltage, low capacity, or
     *          high current draw. Battery failsafe prevents vehicle from losing power
     *          mid-flight and crashing.
     *          
     *          Battery failsafe levels:
     *          - Low battery: Warning, may RTL or continue mission
     *          - Critical battery: Immediate landing action
     *          - Emergency battery: Immediate landing at current location
     *          
     *          Actions taken depend on severity and configuration (BATT_FS_LOW_ACT,
     *          BATT_FS_CRT_ACT parameters).
     * 
     * @param[in] type_str Battery type/cell description for logging
     * @param[in] action Failsafe action to take (from FailsafeAction enum)
     * 
     * @warning Safety-critical - prevents power loss in flight
     * @note Can trigger on voltage, current, or capacity limits
     * @see AP_BattMonitor for battery monitoring
     */
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    
    /**
     * @brief Check for ground control station failsafe condition
     * 
     * @details Monitors for loss of communication with ground control station. GCS
     *          failsafe activates when telemetry link is lost for configured duration,
     *          indicating pilot cannot monitor or command vehicle.
     *          
     *          GCS failsafe conditions:
     *          - No MAVLink heartbeat from GCS for timeout period
     *          - No MAVLink RC override for timeout (if using RC override)
     *          - Timeout configured by FS_GCS_ENABLE parameter
     *          
     *          Unlike radio failsafe, GCS failsafe may allow continued mission
     *          execution since vehicle can still fly autonomously.
     * 
     * @note Called periodically from main loop
     * @see failsafe_gcs_on_event() when timeout detected
     */
    void failsafe_gcs_check();
    
    /**
     * @brief Trigger GCS failsafe event
     * 
     * @details Activates ground control station failsafe when telemetry link is lost.
     *          Action depends on flight mode and configuration - may continue mission
     *          autonomously or take defensive action.
     *          
     *          GCS failsafe actions (FS_GCS_ENABLE parameter):
     *          - Disabled: No action, continue current mode
     *          - Land: Immediate landing
     *          - RTL: Return to launch
     *          - SmartRTL: Return via recorded path
     *          - Continue Auto: Allow mission to complete
     * 
     * @note Generally less critical than radio failsafe
     * @see failsafe_gcs_off_event() for recovery
     */
    void failsafe_gcs_on_event(void);
    
    /**
     * @brief Handle GCS failsafe recovery
     * 
     * @details Called when GCS telemetry link is restored after failsafe. May restore
     *          normal operation or continue failsafe action depending on configuration.
     * 
     * @note Logs recovery event
     * @see failsafe_gcs_on_event() for failsafe trigger
     */
    void failsafe_gcs_off_event(void);
    
    /**
     * @brief Check terrain data availability for terrain failsafe
     * 
     * @details Monitors terrain database availability when terrain following is active.
     *          Terrain failsafe triggers if terrain data becomes unavailable while
     *          flying terrain-relative missions, preventing ground collision from
     *          missing terrain data.
     *          
     *          Checks:
     *          - Terrain database has data for current location
     *          - Terrain data is recent and valid
     *          - No gaps in terrain coverage along flight path
     * 
     * @note Only relevant when using terrain following
     * @see failsafe_terrain_on_event() when terrain data lost
     */
    void failsafe_terrain_check();
    
    /**
     * @brief Update terrain failsafe status
     * 
     * @details Records whether terrain data is currently available and updates failsafe
     *          state accordingly. Used by terrain following to safely handle terrain
     *          database gaps.
     * 
     * @param[in] data_ok true if terrain data is available and valid, false if missing
     * 
     * @see AP_Terrain for terrain database system
     */
    void failsafe_terrain_set_status(bool data_ok);
    
    /**
     * @brief Trigger terrain failsafe event
     * 
     * @details Activates terrain failsafe when terrain data becomes unavailable during
     *          terrain following operations. Switches from terrain-relative to
     *          altitude-above-home navigation to prevent ground collision.
     *          
     *          Terrain failsafe typically:
     *          - Switches mission from terrain-relative to absolute altitude
     *          - May RTL if unable to continue safely
     *          - Alerts pilot of terrain data loss
     * 
     * @warning Critical for terrain following safety
     * @note Only active when terrain following enabled
     */
    void failsafe_terrain_on_event();
    
    /**
     * @brief Check for GPS glitch condition
     * 
     * @details Monitors for sudden GPS position jumps or unrealistic velocity changes
     *          that indicate GPS glitch or interference. While EKF filters most glitches,
     *          severe glitches can briefly affect navigation.
     *          
     *          GPS glitch detection:
     *          - Sudden position jumps beyond realistic vehicle movement
     *          - Velocity changes exceeding vehicle capabilities
     *          - HDOP degradation indicating poor satellite geometry
     *          - Loss of GPS fix
     *          
     *          Sets glitch flag to inform pilot and potentially modify navigation
     *          behavior during glitch.
     * 
     * @note EKF generally handles glitches transparently
     * @see AP_GPS for GPS health monitoring
     */
    void gpsglitch_check();
    
    /**
     * @brief Check for dead reckoning failsafe condition
     * 
     * @details Monitors dead reckoning state and triggers failsafe if dead reckoning
     *          timeout is exceeded. Dead reckoning failsafe activates when EKF has
     *          been operating without position/velocity measurements for too long,
     *          indicating position estimate is no longer reliable.
     *          
     *          Dead reckoning monitoring:
     *          - Checks if dead reckoning active
     *          - Measures duration of dead reckoning
     *          - Compares to configured timeout
     *          - Triggers failsafe when timeout exceeded
     * 
     * @warning Position estimate unreliable after timeout
     * @note Called periodically from main loop
     * @see dead_reckoning struct for state tracking
     */
    void failsafe_deadreckon_check();
    
    /**
     * @brief Attempt RTL or land with pause as failsafe action
     * 
     * @details Tries to execute Return To Launch, but if RTL is not available or
     *          fails, lands at current location after brief pause. The pause allows
     *          vehicle to stabilize before landing.
     *          
     *          Execution sequence:
     *          1. Attempt to enter RTL mode
     *          2. If RTL unavailable (no home, no position), land with pause
     *          3. Pause provides time for pilot assessment
     *          4. Proceeds to landing if no pilot intervention
     * 
     * @param[in] reason Reason code for mode change logging
     * 
     * @note Common failsafe action providing return option
     */
    void set_mode_RTL_or_land_with_pause(ModeReason reason);
    
    /**
     * @brief Attempt SmartRTL or fall back to RTL
     * 
     * @details Tries to execute SmartRTL (return via recorded safe path), but falls
     *          back to standard RTL if SmartRTL path is not available or invalid.
     *          SmartRTL is safer when available as it retraces known-good path.
     * 
     * @param[in] reason Reason code for mode change logging
     * 
     * @note Requires SmartRTL path recording enabled and valid path
     * @see AP_SmartRTL for path recording
     */
    void set_mode_SmartRTL_or_RTL(ModeReason reason);
    
    /**
     * @brief Attempt SmartRTL or land with pause
     * 
     * @details Tries SmartRTL first, but if path unavailable, lands at current
     *          location after brief pause. More conservative than RTL fallback
     *          when position estimate may be degraded.
     * 
     * @param[in] reason Reason code for mode change logging
     * 
     * @note Used when RTL may not be safe (position uncertainty)
     */
    void set_mode_SmartRTL_or_land_with_pause(ModeReason reason);
    
    /**
     * @brief Execute Auto mode DO_LAND_START or RTL
     * 
     * @details If currently in Auto mode with a DO_LAND_START mission item defined,
     *          jumps to that landing sequence. Otherwise, executes RTL. Provides
     *          mission-defined landing approach when available.
     * 
     * @param[in] reason Reason code for mode change logging
     * 
     * @note Useful for missions with designated landing zones
     */
    void set_mode_auto_do_land_start_or_RTL(ModeReason reason);
    
    /**
     * @brief Attempt brake mode or land with pause
     * 
     * @details Tries to enter Brake mode (rapid deceleration), but if unavailable,
     *          lands with pause. Brake mode quickly stops vehicle movement before
     *          landing, useful for stopping runway before landing.
     * 
     * @param[in] reason Reason code for mode change logging
     * 
     * @note Brake provides controlled stop before landing
     */
    void set_mode_brake_or_land_with_pause(ModeReason reason);
    
    /**
     * @brief Determine if vehicle should disarm on failsafe
     * 
     * @details Evaluates whether immediate disarm is appropriate failsafe action.
     *          Disarming is only safe when vehicle is on ground - prevents disarm
     *          in flight which would cause crash.
     *          
     *          Disarm conditions:
     *          - Vehicle detected as landed
     *          - Not in flight
     *          - Failsafe configured to disarm
     *          - Safe to disarm (no altitude/climb rate)
     * 
     * @return true if safe to disarm immediately, false otherwise
     * 
     * @warning Never disarm in flight - causes immediate crash
     */
    bool should_disarm_on_failsafe();
    
    /**
     * @brief Execute specified failsafe action
     * 
     * @details Central failsafe action dispatcher that executes the configured failsafe
     *          response. Routes to appropriate mode change or action based on failsafe
     *          action parameter.
     *          
     *          Failsafe actions:
     *          - NONE: No action (continue current mode)
     *          - LAND: Immediate landing at current location
     *          - RTL: Return to launch
     *          - SMARTRTL: Return via recorded path
     *          - SMARTRTL_LAND: SmartRTL or land
     *          - TERMINATE: Immediate motor shutdown (emergency only)
     *          - AUTO_DO_LAND_START: Jump to landing sequence
     *          - BRAKE_LAND: Brake then land
     * 
     * @param[in] action Failsafe action to execute (from FailsafeAction enum)
     * @param[in] reason Reason code for logging and mode change
     * 
     * @warning TERMINATE action stops motors immediately - only for emergencies
     * @note Action selection based on failsafe type and configuration
     */
    void do_failsafe_action(FailsafeAction action, ModeReason reason);
    
    /**
     * @brief Announce failsafe to pilot and GCS
     * 
     * @details Sends failsafe notifications via GCS telemetry, speech synthesis,
     *          and logging. Ensures pilot is aware of failsafe condition and action
     *          being taken.
     *          
     *          Notification methods:
     *          - MAVLink STATUSTEXT message to GCS
     *          - Spoken announcement if text-to-speech available
     *          - LED pattern change
     *          - Log entry with failsafe details
     * 
     * @param[in] type Failsafe type description (e.g., "Radio", "Battery", "GPS")
     * @param[in] action_undertaken Description of action taken (e.g., "Land", "RTL")
     * 
     * @note Clear communication critical for pilot situational awareness
     */
    void announce_failsafe(const char *type, const char *action_undertaken=nullptr);

    // failsafe.cpp
    /**
     * @brief Enable all failsafe monitoring systems
     * 
     * @details Activates failsafe monitoring after arming. Enables checking for radio
     *          loss, GCS loss, battery critical, GPS failure, and other failsafe
     *          conditions. Failsafes are disabled while disarmed to prevent nuisance
     *          triggering.
     * 
     * @note Called automatically when vehicle arms
     * @see failsafe_disable() when disarming
     */
    void failsafe_enable();
    
    /**
     * @brief Disable all failsafe monitoring systems
     * 
     * @details Deactivates failsafe monitoring after disarming. Prevents failsafe
     *          actions while vehicle is safely on ground and disarmed.
     * 
     * @note Called automatically when vehicle disarms
     * @see failsafe_enable() when arming
     */
    void failsafe_disable();
    
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    /**
     * @brief Check advanced failsafe system (AFS)
     * 
     * @details Monitors advanced failsafe termination conditions for safety compliance.
     *          Advanced failsafe provides hardware-based termination capability for
     *          regulatory compliance or safety-critical operations.
     *          
     *          AFS monitors:
     *          - Geofence boundary breach
     *          - Altitude limit breach
     *          - GPS failure
     *          - Manual termination command
     *          
     *          When termination conditions met, AFS can trigger hardware termination
     *          pin to cut motor power via external relay/FET.
     * 
     * @warning Hardware termination stops motors immediately - aircraft will crash
     * @note Only compiled when AP_COPTER_ADVANCED_FAILSAFE_ENABLED
     * @see AP_AdvancedFailsafe_Copter for AFS implementation
     */
    void afs_fs_check(void);
#endif

    // fence.cpp
#if AP_FENCE_ENABLED
    /**
     * @brief Check geofence boundaries and handle breaches
     * 
     * @details Monitors vehicle position against configured geofence boundaries and
     *          takes action if fence is breached. Geofencing prevents vehicle from
     *          flying outside designated safe areas.
     *          
     *          Fence types:
     *          - Cylindrical: Maximum distance from home (horizontal fence)
     *          - Altitude: Maximum and minimum altitude limits
     *          - Polygon: Complex inclusion/exclusion zones
     *          - Circular: Multiple circular fence zones
     *          
     *          Fence breach actions (FENCE_ACTION parameter):
     *          - Report only: Alert pilot but no action
     *          - RTL: Return to launch
     *          - Land: Land at current location
     *          - Brake: Stop and loiter at fence boundary
     *          - SmartRTL: Return via recorded path
     *          
     *          Fence checking respects enable/disable state and arming conditions.
     * 
     * @warning Fence breach may cause sudden mode change and vehicle movement
     * @note Called periodically from main loop when fence enabled
     * @see AC_Fence for fence implementation
     */
    void fence_check();
    
    /**
     * @brief Perform asynchronous fence boundary checks
     * 
     * @details Executes fence checks that can be performed asynchronously without
     *          blocking main loop. Overrides AP_Vehicle base class method to provide
     *          vehicle-specific fence checking.
     *          
     *          Async checks allow complex polygon intersection tests and path
     *          prediction without impacting main loop timing.
     * 
     * @note Called from scheduler at lower priority than time-critical tasks
     */
    void fence_checks_async() override;
#endif

    // heli.cpp
    /**
     * @brief Initialize helicopter-specific systems
     * 
     * @details Sets up traditional helicopter specific control systems including
     *          swashplate servo configuration, rotor speed controller, and collective
     *          pitch management. Only used for FRAME_CONFIG == HELI_FRAME.
     *          
     *          Helicopter initialization:
     *          - Configure swashplate mixing
     *          - Initialize rotor speed controller (RSC)
     *          - Setup collective pitch limits
     *          - Configure tail rotor/yaw control
     *          - Initialize autorotation system if enabled
     * 
     * @note Only compiled for helicopter frame types
     * @see AP_MotorsHeli for helicopter motor control
     */
    void heli_init();
    
    /**
     * @brief Check if helicopter is in dynamic flight
     * 
     * @details Determines whether helicopter is flying dynamically (moving with
     *          significant speed) versus hovering or ground operations. Dynamic
     *          flight state affects control behavior, particularly "leaky I" term
     *          management in PID controllers.
     *          
     *          Dynamic flight detected by:
     *          - Forward airspeed above threshold
     *          - Sustained translational velocity
     *          - Cyclic stick deflection indicating forward flight
     *          
     *          Enables different control gains and I-term behavior for forward
     *          flight versus hover.
     * 
     * @note Helicopter-specific control optimization
     */
    void check_dynamic_flight(void);
    
    /**
     * @brief Check if landing swashplate configuration should be used
     * 
     * @details Determines whether to use landing-specific swashplate limits and
     *          collective pitch settings. Landing swashplate reduces collective
     *          range to prevent rotor blade strikes during touchdown.
     * 
     * @return true if landing swashplate should be active, false otherwise
     * 
     * @note Prevents rotor blade ground strikes during landing
     */
    bool should_use_landing_swash() const;
    
    /**
     * @brief Update helicopter control dynamics parameters
     * 
     * @details Adjusts control response characteristics based on flight conditions.
     *          Modifies PID gains, mixing parameters, and response curves for
     *          optimal control in different flight regimes (hover, forward flight,
     *          aggressive maneuvering).
     * 
     * @note Provides adaptive control for varying flight conditions
     */
    void update_heli_control_dynamics(void);
    
    /**
     * @brief Update swashplate to landing configuration
     * 
     * @details Transitions swashplate servo positions to landing-specific limits.
     *          Reduces collective pitch range and modifies cyclic limits to
     *          prevent blade strikes and provide stable landing.
     * 
     * @note Called when landing detected or landing mode active
     */
    void heli_update_landing_swash();
    
    /**
     * @brief Get pilot's desired rotor speed from RC input
     * 
     * @details Reads pilot's rotor speed control input (typically from throttle
     *          curve or dedicated rotor speed channel) and returns desired rotor
     *          RPM setpoint. Used for manual rotor speed control.
     * 
     * @return Desired rotor speed (RPM or normalized 0-1)
     * 
     * @note Rotor speed control method depends on helicopter configuration
     */
    float get_pilot_desired_rotor_speed() const;
    
    /**
     * @brief Update rotor speed controller targets
     * 
     * @details Sets rotor speed controller (RSC) target based on flight mode and
     *          pilot input. Manages rotor spool-up, normal flight speed, autorotation
     *          entry, and spool-down sequences.
     *          
     *          RSC states:
     *          - Shutdown: Rotor stopped
     *          - Idle: Rotor at idle speed
     *          - Spool-up: Rotor accelerating to flight speed
     *          - Flight: Normal flight rotor speed
     *          - Autorotation: Zero throttle, rotor freewheeling
     *          - Spool-down: Rotor decelerating
     * 
     * @note Critical for safe helicopter operation
     * @see AP_MotorsHeli::set_desired_rotor_speed()
     */
    void heli_update_rotor_speed_targets();
    
    /**
     * @brief Update autorotation control system
     * 
     * @details Manages helicopter autorotation state for emergency power-off landings.
     *          Autorotation allows helicopter to land safely after engine failure by
     *          using descending airflow to maintain rotor speed.
     *          
     *          Autorotation management:
     *          - Monitor for autorotation entry conditions
     *          - Control collective pitch for rotor speed management
     *          - Guide descent profile for safe landing
     *          - Execute flare and landing sequence
     * 
     * @warning Critical for helicopter emergency procedures
     * @note Only active in Autorotate flight mode
     * @see AC_Autorotation for autorotation controller
     */
    void heli_update_autorotation();
    
    /**
     * @brief Update collective stick low position flag
     * 
     * @details Tracks whether collective stick is at or near bottom position. Used
     *          for various helicopter control logic including rotor spool-up/down
     *          detection and autorotation entry.
     * 
     * @param[in] throttle_control Current collective/throttle control value
     * 
     * @note Sets heli_flags.coll_stk_low flag
     */
    void update_collective_low_flag(int16_t throttle_control);

    // inertia.cpp
    /**
     * @brief Read IMU data from inertial sensors
     * 
     * @details Retrieves accelerometer and gyroscope measurements from IMU sensors.
     *          This is the primary input for attitude estimation and control. IMU
     *          data is read at high frequency (typically 1000Hz or higher) and
     *          processed by the AHRS/EKF for state estimation.
     *          
     *          IMU reading includes:
     *          - Gyroscope rates (roll, pitch, yaw rates)
     *          - Accelerometer measurements (specific force)
     *          - Temperature compensation
     *          - Sensor calibration application
     *          - Multi-IMU consistency checking
     *          
     *          IMU data quality is critical for stable flight - sensor health
     *          monitoring and calibration ensure reliable measurements.
     * 
     * @note Called at high frequency from main loop or rate controller
     * @see AP_InertialSensor for IMU management
     */
    void read_inertia();

    // landing_detector.cpp
    /**
     * @brief Update landing and crash detection systems
     * 
     * @details Main entry point for running both landing detector and crash detector
     *          algorithms. Monitors vehicle state to determine if vehicle has landed
     *          or crashed, enabling appropriate responses (disarm, mode changes).
     *          
     *          Detection systems:
     *          - Landing detector: Identifies successful landings
     *          - Crash detector: Detects crashes or tip-overs
     *          - Takeoff detector: Verifies clean takeoffs
     *          
     *          Accurate detection prevents premature disarm or inappropriate mode
     *          changes while ensuring quick response to actual landings/crashes.
     * 
     * @note Called periodically from main loop
     * @see update_land_detector() for landing detection details
     * @see crash_check() for crash detection
     */
    void update_land_and_crash_detectors();
    
    /**
     * @brief Update landing detection state machine
     * 
     * @details Analyzes multiple sensor inputs and flight parameters to determine
     *          if vehicle has landed. Landing detection is critical for automatic
     *          disarm and preventing propeller strikes.
     *          
     *          Landing indicators:
     *          - Low vertical velocity (descent stopped)
     *          - Minimal acceleration (vehicle stationary)
     *          - Motors at lower thrust limit
     *          - Rangefinder shows ground contact
     *          - Barometer shows stable altitude
     *          - Low throttle input from pilot
     *          
     *          Two-stage detection:
     *          1. land_complete_maybe: Possibly landed (relaxed thresholds)
     *          2. land_complete: Definitely landed (strict thresholds sustained)
     *          
     *          Progressive detection prevents false positives during aggressive
     *          low-altitude flight while ensuring reliable detection on landing.
     * 
     * @note Updates ap.land_complete and ap.land_complete_maybe flags
     * @see set_land_complete() to set landing state
     */
    void update_land_detector();
    
    /**
     * @brief Set definite landing state
     * 
     * @details Marks vehicle as definitively landed (highest confidence). Triggers
     *          automatic disarm countdown and prevents takeoff until throttle cycled.
     *          Land complete state affects motor interlock, mode changes, and arming.
     * 
     * @param[in] b true if vehicle has landed, false if in flight
     * 
     * @note Setting land_complete=true initiates disarm sequence
     * @see update_land_detector() for automatic detection
     */
    void set_land_complete(bool b);
    
    /**
     * @brief Set probable landing state
     * 
     * @details Marks vehicle as probably landed (medium confidence). Used for
     *          relaxed landing detection that catches landings early but may have
     *          occasional false positives. Affects throttle mixing and some mode
     *          behaviors but doesn't trigger disarm.
     * 
     * @param[in] b true if vehicle probably landed, false if definitely flying
     * 
     * @note Less strict than land_complete, used for early landing indication
     * @see set_land_complete() for definite landing state
     */
    void set_land_complete_maybe(bool b);
    
    /**
     * @brief Update motor throttle mixing for landing
     * 
     * @details Adjusts throttle mixing ratio between altitude control and stability
     *          control based on landing detector state. During landing, reduces
     *          altitude control priority to prevent motor spool-up on touchdown.
     *          
     *          Throttle mix stages:
     *          - Flying: High altitude control priority
     *          - Landing: Gradually reduce altitude priority
     *          - Landed: Minimum altitude control, maximize stability
     *          
     *          Smooth throttle mix transition prevents abrupt control changes
     *          during landing sequence.
     * 
     * @note Affects motor mixing in AC_AttitudeControl
     * @see AC_AttitudeControl::set_throttle_mix_value()
     */
    void update_throttle_mix();
    
    /**
     * @brief Get force flying override state
     * 
     * @details Checks if flight is being forced despite landing detector indications.
     *          Force flying override prevents landing detector from triggering during
     *          intentional ground operations (motor testing, aggressive low flight).
     *          
     *          Force flying activated by:
     *          - Aux switch override
     *          - Certain flight modes (motor test, ESC cal)
     *          - Manual pilot override
     * 
     * @return true if flying is being forced, false for normal detection
     * 
     * @note Overrides automatic landing detection
     */
    bool get_force_flying() const;
    
#if HAL_LOGGING_ENABLED
    /**
     * @brief Landing detector state flags for logging
     * 
     * @details Bitmask flags representing individual landing detector conditions.
     *          Each flag indicates whether a specific landing criterion is met.
     *          Used in LDET (Landing Detector) log messages for detailed analysis
     *          of landing detection behavior and tuning.
     *          
     *          Flags track:
     *          - LANDED: Definite landing detected (land_complete)
     *          - LANDED_MAYBE: Probable landing detected (land_complete_maybe)
     *          - LANDING: Vehicle in landing descent
     *          - STANDBY_ACTIVE: Standby mode active (ground idle)
     *          - WOW: Weight-on-wheels detected
     *          - RANGEFINDER_BELOW_2M: Close to ground per rangefinder
     *          - DESCENT_RATE_LOW: Vertical velocity near zero
     *          - ACCEL_STATIONARY: Accelerations indicate stationary vehicle
     *          - LARGE_ANGLE_ERROR: Large attitude error (possible crash)
     *          - LARGE_ANGLE_REQUEST: Pilot requesting large attitude angle
     *          - MOTOR_AT_LOWER_LIMIT: Motors at minimum thrust
     *          - THROTTLE_MIX_AT_MIN: Throttle mix at minimum (landed mix)
     * 
     * @note Flags are bitwise OR'd together in log messages
     */
    enum class LandDetectorLoggingFlag : uint16_t {
        LANDED               = 1U <<  0,  /**< @brief Definite landing detected */
        LANDED_MAYBE         = 1U <<  1,  /**< @brief Probable landing detected */
        LANDING              = 1U <<  2,  /**< @brief Currently landing */
        STANDBY_ACTIVE       = 1U <<  3,  /**< @brief Standby mode active */
        WOW                  = 1U <<  4,  /**< @brief Weight on wheels */
        RANGEFINDER_BELOW_2M = 1U <<  5,  /**< @brief Rangefinder shows <2m altitude */
        DESCENT_RATE_LOW     = 1U <<  6,  /**< @brief Low descent rate */
        ACCEL_STATIONARY     = 1U <<  7,  /**< @brief Accelerations indicate stationary */
        LARGE_ANGLE_ERROR    = 1U <<  8,  /**< @brief Large attitude error */
        LARGE_ANGLE_REQUEST  = 1U <<  8,  /**< @brief Large attitude angle requested */
        MOTOR_AT_LOWER_LIMIT = 1U <<  9,  /**< @brief Motors at lower thrust limit */
        THROTTLE_MIX_AT_MIN  = 1U << 10,  /**< @brief Throttle mix at minimum */
    };
    
    /**
     * @brief Landing detector logging state
     * 
     * @details Tracks landing detector logging to prevent excessive log rate while
     *          capturing state transitions. Stores last logged state for comparison
     *          and rate limiting.
     */
    struct {
        uint32_t last_logged_ms;      /**< @brief System time of last LDET log message (ms) */
        uint32_t last_logged_count;   /**< @brief Landing detector update count at last log */
        uint16_t last_logged_flags;   /**< @brief Landing detector flags at last log */
    } land_detector;
    
    /**
     * @brief Write landing detector log message
     * 
     * @details Records landing detector state to dataflash for post-flight analysis.
     *          LDET messages capture all landing detection criteria and their states,
     *          enabling detailed tuning of landing detection parameters.
     *          
     *          Log message includes:
     *          - All landing detector flags
     *          - Landing detection counter
     *          - Timestamp for correlation with other logs
     *          
     *          Used for:
     *          - Tuning landing detection sensitivity
     *          - Diagnosing false landing detections
     *          - Analyzing landing sequences
     *          - Validating landing detector behavior
     * 
     * @param[in] logging_flags Bitmask of LandDetectorLoggingFlag values
     * @param[in] land_detector_count Number of landing detector update cycles
     * 
     * @note Logs written at controlled rate to prevent filling dataflash
     */
    void Log_LDET(uint16_t logging_flags, uint32_t land_detector_count);
#endif

#if AP_LANDINGGEAR_ENABLED
    // landing_gear.cpp
    /**
     * @brief Update landing gear deployment state
     * 
     * @details Controls automatic or manual deployment/retraction of landing gear.
     *          Landing gear state management includes:
     *          - Automatic deployment on landing approach
     *          - Automatic retraction after takeoff
     *          - Manual pilot override via RC switch
     *          - Servo position control for gear mechanism
     *          - Transition timing and safety interlocks
     *          
     *          Gear states:
     *          - Retracted: Gear up for cruise flight
     *          - Deploying: Gear extending in progress
     *          - Deployed: Gear fully down for landing
     *          - Retracting: Gear retracting in progress
     *          
     *          Automatic gear management improves efficiency (retracted cruise) while
     *          ensuring gear is down for landing safety.
     * 
     * @note Only compiled when AP_LANDINGGEAR_ENABLED
     * @see AP_LandingGear for gear control implementation
     */
    void landinggear_update();
#endif

    // standby.cpp
    /**
     * @brief Update standby (ground idle) mode
     * 
     * @details Manages standby state for multicopters sitting on ground with motors
     *          armed but not flying. Standby mode maintains motor spin at ground idle
     *          speed while preventing takeoff until pilot applies throttle.
     *          
     *          Standby characteristics:
     *          - Motors armed and spinning at ground idle
     *          - Minimal thrust output (vehicle on ground)
     *          - Attitude control active
     *          - Full control authority available
     *          - Quick response to throttle input
     *          
     *          Standby entry conditions:
     *          - Vehicle armed on ground
     *          - Throttle at zero
     *          - Landing detector confirms ground contact
     *          
     *          Standby exit conditions:
     *          - Pilot raises throttle above threshold
     *          - Vehicle lifts off ground
     *          
     *          Standby provides ready state for immediate takeoff while maintaining
     *          safety of low motor speed on ground.
     * 
     * @note Updates standby_active flag
     * @see update_land_detector() for ground detection
     */
    void standby_update();

#if HAL_LOGGING_ENABLED
    // methods for AP_Vehicle:
    /**
     * @brief Get logging bitmask parameter
     * 
     * @details Returns reference to parameter controlling which log messages are enabled.
     *          Bitmask allows selective logging to manage dataflash storage and
     *          logging performance. Each bit enables a specific log category.
     *          
     *          Overrides AP_Vehicle base class to provide Copter-specific bitmask.
     * 
     * @return Reference to LOG_BITMASK parameter
     * 
     * @note Used by logging system to determine which messages to write
     */
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    
    /**
     * @brief Get vehicle-specific log message structures
     * 
     * @details Returns pointer to array defining all Copter-specific log message
     *          formats. Log structures define message layout, field names, units,
     *          and multipliers for binary dataflash logging.
     *          
     *          Overrides AP_Vehicle base class to provide Copter log definitions.
     * 
     * @return Pointer to log_structure array
     * 
     * @note Used by logging system for message formatting
     */
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    
    /**
     * @brief Get count of vehicle-specific log structures
     * 
     * @details Returns number of Copter-specific log message definitions.
     *          Used with get_log_structures() to iterate message definitions.
     * 
     * @return Number of log structures in log_structure array
     */
    uint8_t get_num_log_structures() const override;

    // Log.cpp
    /**
     * @brief Write control tuning log message
     * 
     * @details Logs control system tuning data including desired vs actual values
     *          for attitude, position, and velocity control. Essential for PID
     *          tuning and control system analysis.
     *          
     *          Logged data:
     *          - Desired attitude (roll, pitch, yaw)
     *          - Actual attitude (roll, pitch, yaw)
     *          - Desired rates (roll, pitch, yaw rates)
     *          - Actual rates from gyros
     *          - Desired altitude and climb rate
     *          - Actual altitude and climb rate
     *          - Throttle in/out values
     *          
     *          Used for:
     *          - PID gain tuning
     *          - Control response analysis
     *          - Tracking error diagnosis
     * 
     * @note Logged at main loop rate when enabled
     */
    void Log_Write_Control_Tuning();
    
    /**
     * @brief Write attitude log message
     * 
     * @details Logs vehicle attitude (roll, pitch, yaw) and attitude targets.
     *          Primary message for analyzing attitude control performance.
     *          
     *          Logged data:
     *          - Current attitude (Euler angles)
     *          - Desired attitude
     *          - Attitude errors
     *          - Control mode
     * 
     * @note High-rate logging for detailed attitude analysis
     */
    void Log_Write_Attitude();
    
    /**
     * @brief Write rate controller log message
     * 
     * @details Logs angular rate control data (roll rate, pitch rate, yaw rate).
     *          Shows rate controller performance and gyro data.
     *          
     *          Logged data:
     *          - Desired rates (deg/s)
     *          - Actual rates from gyros (deg/s)
     *          - Rate errors
     *          - Rate PID outputs
     * 
     * @note High-frequency logging for rate controller tuning
     */
    void Log_Write_Rate();
    
    /**
     * @brief Write EKF position log message
     * 
     * @details Logs Extended Kalman Filter position estimates for navigation
     *          analysis. Shows EKF's position and velocity estimates.
     *          
     *          Logged data:
     *          - Position NED (North, East, Down in meters)
     *          - Velocity NED (m/s)
     *          - Position variances
     *          - EKF status flags
     * 
     * @note Used for navigation and EKF performance analysis
     */
    void Log_Write_EKF_POS();
    
    /**
     * @brief Write PID controller log message
     * 
     * @details Logs PID controller internal states for all attitude and position
     *          controllers. Shows P, I, D terms, desired values, and outputs.
     *          Essential for comprehensive PID tuning.
     *          
     *          Logged PIDs:
     *          - Roll rate PID
     *          - Pitch rate PID
     *          - Yaw rate PID
     *          - Roll angle PID
     *          - Pitch angle PID
     *          - Velocity PIDs (North, East, Down)
     *          - Position PIDs (North, East)
     * 
     * @note Detailed PID data for advanced tuning
     */
    void Log_Write_PIDS();
    
    /**
     * @brief Write generic data log message (int32_t)
     * 
     * @details Logs single integer value with identifier. Used for logging
     *          miscellaneous integer data points that don't fit other messages.
     * 
     * @param[in] id Data identifier (LogDataID enum)
     * @param[in] value 32-bit signed integer value to log
     */
    void Log_Write_Data(LogDataID id, int32_t value);
    
    /**
     * @brief Write generic data log message (uint32_t)
     * 
     * @details Logs single unsigned integer value with identifier.
     * 
     * @param[in] id Data identifier (LogDataID enum)
     * @param[in] value 32-bit unsigned integer value to log
     */
    void Log_Write_Data(LogDataID id, uint32_t value);
    
    /**
     * @brief Write generic data log message (int16_t)
     * 
     * @details Logs single 16-bit integer value with identifier.
     * 
     * @param[in] id Data identifier (LogDataID enum)
     * @param[in] value 16-bit signed integer value to log
     */
    void Log_Write_Data(LogDataID id, int16_t value);
    
    /**
     * @brief Write generic data log message (uint16_t)
     * 
     * @details Logs single 16-bit unsigned integer value with identifier.
     * 
     * @param[in] id Data identifier (LogDataID enum)
     * @param[in] value 16-bit unsigned integer value to log
     */
    void Log_Write_Data(LogDataID id, uint16_t value);
    
    /**
     * @brief Write generic data log message (float)
     * 
     * @details Logs single floating-point value with identifier.
     * 
     * @param[in] id Data identifier (LogDataID enum)
     * @param[in] value Floating-point value to log
     */
    void Log_Write_Data(LogDataID id, float value);
    
    /**
     * @brief Write parameter tuning log message
     * 
     * @details Logs in-flight parameter tuning activity. Records parameter being
     *          tuned, current value, and tuning range for analysis of tuning process.
     *          
     *          Used for:
     *          - In-flight PID tuning via transmitter knob
     *          - AutoTune progress tracking
     *          - Manual parameter adjustment logging
     * 
     * @param[in] param Parameter identifier being tuned
     * @param[in] tuning_val Current parameter value
     * @param[in] tune_min Minimum tuning range value
     * @param[in] tune_max Maximum tuning range value
     */
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max);
    
    /**
     * @brief Write video stabilization log message
     * 
     * @details Logs video stabilization data for camera gimbals. Records gimbal
     *          angles and rates used for stabilizing video footage.
     *          
     *          Logged data:
     *          - Gimbal angles (roll, pitch, yaw)
     *          - Gimbal rates
     *          - Stabilization mode
     * 
     * @note Used for gimbal tuning and video post-processing
     */
    void Log_Video_Stabilisation();
    
    /**
     * @brief Write Guided mode position target log message
     * 
     * @details Logs position/velocity/acceleration targets sent to Guided mode.
     *          Essential for analyzing Guided mode commanded trajectories from
     *          GCS, companion computer, or scripts.
     *          
     *          Logged data:
     *          - Target position (NED or terrain-relative)
     *          - Target velocity vector
     *          - Target acceleration vector
     *          - Guided sub-mode
     * 
     * @param[in] submode Guided mode sub-mode (position, velocity, accel, etc.)
     * @param[in] pos_target Target position vector (NED meters)
     * @param[in] terrain_alt true if position is terrain-relative altitude
     * @param[in] vel_target Target velocity vector (NED m/s)
     * @param[in] accel_target Target acceleration vector (NED m/s²)
     * 
     * @note Used for companion computer integration debugging
     */
    void Log_Write_Guided_Position_Target(ModeGuided::SubMode submode, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target);
    
    /**
     * @brief Write Guided mode attitude target log message
     * 
     * @details Logs attitude/rate/thrust targets sent to Guided mode. Used for
     *          analyzing low-level attitude commands from external controllers.
     *          
     *          Logged data:
     *          - Target attitude (roll, pitch, yaw angles)
     *          - Target angular velocity
     *          - Target thrust/climb rate
     * 
     * @param[in] target_type Guided mode target type
     * @param[in] roll Target roll angle (degrees)
     * @param[in] pitch Target pitch angle (degrees)
     * @param[in] yaw Target yaw angle (degrees)
     * @param[in] ang_vel Target angular velocity (deg/s)
     * @param[in] thrust Target thrust (0-1)
     * @param[in] climb_rate Target climb rate (m/s)
     */
    void Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate);
    
    /**
     * @brief Write System Identification setup log message
     * 
     * @details Logs SystemID mode configuration parameters. SystemID mode performs
     *          frequency sweeps for system identification and model parameter
     *          extraction. Setup message records test configuration.
     *          
     *          Logged parameters:
     *          - Axis under test (roll, pitch, yaw)
     *          - Waveform magnitude
     *          - Frequency sweep range (start/stop Hz)
     *          - Timing parameters (fade in, constant frequency, record, fade out)
     * 
     * @param[in] systemID_axis Axis being tested (0=roll, 1=pitch, 2=yaw)
     * @param[in] waveform_magnitude Input magnitude for frequency sweep
     * @param[in] frequency_start Starting frequency (Hz)
     * @param[in] frequency_stop Ending frequency (Hz)
     * @param[in] time_fade_in Fade-in duration (seconds)
     * @param[in] time_const_freq Constant frequency duration (seconds)
     * @param[in] time_record Recording duration (seconds)
     * @param[in] time_fade_out Fade-out duration (seconds)
     * 
     * @note Used with SystemID mode for advanced tuning
     */
    void Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out);
    
    /**
     * @brief Write System Identification data log message
     * 
     * @details Logs high-rate data during SystemID frequency sweep test. Captures
     *          input waveform and vehicle response for system identification analysis.
     *          
     *          Logged data:
     *          - Waveform time, sample value, and current frequency
     *          - Vehicle attitude angles (roll, pitch, yaw)
     *          - Vehicle accelerations (x, y, z)
     *          
     *          Data used for:
     *          - Transfer function identification
     *          - Model parameter extraction
     *          - Advanced PID tuning using frequency response
     * 
     * @param[in] waveform_time Current time in waveform (seconds)
     * @param[in] waveform_sample Current waveform amplitude
     * @param[in] waveform_freq Current waveform frequency (Hz)
     * @param[in] angle_x Roll angle response (degrees)
     * @param[in] angle_y Pitch angle response (degrees)
     * @param[in] angle_z Yaw angle response (degrees)
     * @param[in] accel_x X-axis acceleration (m/s²)
     * @param[in] accel_y Y-axis acceleration (m/s²)
     * @param[in] accel_z Z-axis acceleration (m/s²)
     * 
     * @note High-rate logging during SystemID test
     */
    void Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z);
    
    /**
     * @brief Write vehicle startup log messages
     * 
     * @details Logs vehicle configuration and status at startup/boot. Records
     *          firmware version, board type, frame configuration, and system
     *          parameters for flight log identification and analysis.
     *          
     *          Startup messages include:
     *          - Firmware version and commit hash
     *          - Frame type and configuration
     *          - Board hardware type
     *          - Key parameter values
     *          - Sensor configuration
     * 
     * @note Called once during init_ardupilot() initialization
     */
    void Log_Write_Vehicle_Startup_Messages();
    
    /**
     * @brief Write rate controller thread timing log message
     * 
     * @details Logs rate controller loop timing statistics. Monitors fast rate
     *          controller thread performance to ensure consistent high-rate
     *          execution. Critical for verifying rate controller is meeting
     *          timing requirements.
     *          
     *          Logged timing:
     *          - Current loop delta time (dt)
     *          - Average loop delta time
     *          - Maximum loop delta time
     *          - Minimum loop delta time
     *          
     *          Timing violations indicate CPU overload or scheduling issues.
     * 
     * @param[in] dt Current loop delta time (seconds)
     * @param[in] dtAvg Average loop delta time (seconds)
     * @param[in] dtMax Maximum loop delta time (seconds)
     * @param[in] dtMin Minimum loop delta time (seconds)
     * 
     * @note Used for fast rate controller performance monitoring
     */
    void Log_Write_Rate_Thread_Dt(float dt, float dtAvg, float dtMax, float dtMin);
#endif  // HAL_LOGGING_ENABLED

    // mode.cpp
    /**
     * @brief Set flight mode by mode number
     * 
     * @details Attempts to change vehicle to specified flight mode. Performs mode
     *          change validation, state transitions, and initialization of new mode.
     *          
     *          Mode change process:
     *          1. Validate new mode is available and can be entered
     *          2. Check arming/disarming requirements for mode
     *          3. Call old mode exit() function
     *          4. Switch to new mode
     *          5. Call new mode init() function
     *          6. Notify pilot and GCS of mode change
     *          
     *          Mode change can fail if:
     *          - Mode not available on this vehicle type
     *          - Mode requires GPS and GPS not available
     *          - Mode requires specific sensors not present
     *          - Arming checks fail for mode
     *          - Mode disabled via parameters
     * 
     * @param[in] mode Flight mode to enter (Mode::Number enum)
     * @param[in] reason Why mode change requested (for logging)
     * 
     * @return true if mode change successful, false if failed
     * 
     * @note Logs mode change and reason for post-flight analysis
     * @see mode_change_failed() if mode change unsuccessful
     */
    bool set_mode(Mode::Number mode, ModeReason reason);
    
    /**
     * @brief Set flight mode by mode number (AP_Vehicle override)
     * 
     * @details Wrapper for set_mode() that accepts uint8_t mode number to satisfy
     *          AP_Vehicle base class interface. Allows mode changes from MAVLink
     *          commands and other vehicle-agnostic code.
     * 
     * @param[in] new_mode Flight mode number (uint8_t)
     * @param[in] reason Why mode change requested
     * 
     * @return true if mode change successful, false if failed
     * 
     * @note Overrides AP_Vehicle::set_mode()
     */
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    
    /**
     * @brief Reason for last mode change
     * 
     * @details Stores reason why most recent mode change occurred. Used for
     *          logging and understanding mode transition causes (pilot, failsafe,
     *          GCS command, etc.)
     */
    ModeReason _last_reason;
    
    /**
     * @brief Handle unsuccessful mode change attempt
     * 
     * @details Called when attempt to enter flight mode fails. Logs failure,
     *          notifies pilot via audio/text, and sends failure message to GCS.
     *          Vehicle remains in current mode.
     *          
     *          Common failure reasons:
     *          - GPS required but not available
     *          - Mode requires position estimate not available
     *          - Rangefinder required but not working
     *          - Mode disabled via parameter
     *          - Battery too low for mode
     * 
     * @param[in] mode Mode that could not be entered
     * @param[in] reason Human-readable failure reason
     * 
     * @note Helps pilot understand why mode change rejected
     */
    void mode_change_failed(const Mode *mode, const char *reason);
    
    /**
     * @brief Get current flight mode number
     * 
     * @details Returns numeric identifier of active flight mode. Overrides
     *          AP_Vehicle base class to provide current Copter mode.
     * 
     * @return Flight mode number (0-255)
     * 
     * @note Used for telemetry and mode-dependent logic
     */
    uint8_t get_mode() const override { return (uint8_t)flightmode->mode_number(); }
    
    /**
     * @brief Check if current mode requires mission
     * 
     * @details Determines whether active flight mode requires valid mission to
     *          operate. Used to validate mission availability before mode entry
     *          and during mission execution.
     *          
     *          Modes requiring mission:
     *          - Auto mode
     *          - Guided mode with mission items
     *          - Some mission-based follow modes
     * 
     * @return true if current mode requires loaded mission, false otherwise
     * 
     * @note Overrides AP_Vehicle base class
     */
    bool current_mode_requires_mission() const override;
    
    /**
     * @brief Execute current flight mode control logic
     * 
     * @details Calls active mode's run() function to execute mode-specific control.
     *          Called every main loop iteration to update mode's control outputs.
     *          Each mode implements different control strategies:
     *          
     *          - Stabilize: Pilot controls attitude directly
     *          - AltHold: Pilot controls attitude, altitude held automatically
     *          - Loiter: Position held, pilot can reposition
     *          - Auto: Follows mission waypoints autonomously
     *          - RTL: Returns to launch location automatically
     *          - Land: Descends and lands autonomously
     *          
     *          Mode's run() function:
     *          1. Reads pilot input
     *          2. Calculates desired attitude/position/velocity
     *          3. Calls appropriate controllers
     *          4. Updates motor outputs
     * 
     * @note Called at main loop rate (typically 400Hz)
     * @see Mode::run() for mode-specific implementation
     */
    void update_flight_mode();
    
    /**
     * @brief Notify pilot and GCS of flight mode
     * 
     * @details Sends flight mode information to ground station and updates pilot
     *          notification systems (LED, buzzer, voice). Ensures pilot and GCS
     *          are aware of current mode.
     *          
     *          Notifications:
     *          - Send mode to GCS via MAVLink
     *          - Update LED patterns for mode
     *          - Play audio tone for mode (if available)
     *          - Display mode on OSD
     * 
     * @note Called after successful mode change
     */
    void notify_flight_mode();

    /**
     * @brief Check if flight mode can be entered from GCS
     * 
     * @details Validates whether specified mode is allowed to be entered via GCS
     *          command. Some modes may be disabled or restricted based on vehicle
     *          configuration, parameter settings, or safety constraints.
     *          
     *          Checks:
     *          - Mode available on this frame type
     *          - Mode not disabled via FLTMODE parameters
     *          - Mode not blocked by safety settings
     *          - Required sensors available for mode
     * 
     * @param[in] mode_num Flight mode to check
     * 
     * @return true if mode can be entered from GCS, false if blocked
     * 
     * @note Used to validate GCS mode change commands
     */
    bool gcs_mode_enabled(const Mode::Number mode_num);

    // mode_land.cpp
    /**
     * @brief Enter Land mode with descent pause
     * 
     * @details Initiates Land mode with brief pause before descent begins. Pause
     *          allows pilot to abort landing or for vehicle to stabilize position
     *          before committing to descent.
     *          
     *          Used when:
     *          - Failsafe triggers landing
     *          - Battery critical forces landing
     *          - Fence breach requires landing
     *          
     *          Pause duration gives pilot time to take control or move vehicle to
     *          better landing location before descent begins.
     * 
     * @param[in] reason Why landing was initiated
     * 
     * @note Provides emergency landing with pilot override opportunity
     */
    void set_mode_land_with_pause(ModeReason reason);
    
    /**
     * @brief Check if landing using GPS position hold
     * 
     * @details Determines whether Land mode is using GPS position control for
     *          landing. GPS landing maintains horizontal position during descent,
     *          preventing drift. Non-GPS landing descends without position hold.
     *          
     *          GPS landing used when:
     *          - GPS available with good accuracy
     *          - Position estimate available from EKF
     *          - GPS not glitching
     *          
     *          Non-GPS landing used when:
     *          - GPS unavailable or poor quality
     *          - Landing in GPS-denied environment
     *          - Position estimate not available
     * 
     * @return true if landing with GPS position control, false otherwise
     * 
     * @note GPS landing is safer but requires good position estimate
     */
    bool landing_with_GPS();

    // motor_test.cpp
    /**
     * @brief Output motor test signals to motors
     * 
     * @details Sends test signals to motors during motor test mode. Allows testing
     *          individual motors or all motors simultaneously to verify motor
     *          rotation direction, ESC calibration, and propeller mounting.
     *          
     *          Motor test modes:
     *          - Individual motor test: Tests one motor at a time
     *          - Sequence test: Tests motors in sequence
     *          - All motors: Tests all motors simultaneously
     *          
     *          Used for:
     *          - Verifying motor rotation directions
     *          - Checking motor mounting and wiring
     *          - ESC calibration verification
     *          - Propeller balance testing
     * 
     * @warning Vehicle must be safely secured before motor testing
     * @note Called from motor_test loop when motor test active
     */
    void motor_test_output();
    
    /**
     * @brief Check if motor control command is safe to execute
     * 
     * @details Validates safety conditions before allowing motor control via MAVLink.
     *          Ensures vehicle is safely secured and pilot is aware before allowing
     *          external motor control.
     *          
     *          Safety checks:
     *          - Vehicle disarmed
     *          - RC input check (if requested)
     *          - Safety switch pressed (if present)
     *          - Throttle at minimum
     *          
     *          Prevents accidental motor activation that could cause injury.
     * 
     * @param[in] gcs_chan MAVLink channel requesting motor control
     * @param[in] check_rc true to require RC radio active
     * @param[in] mode Motor test mode name (for error messages)
     * 
     * @return true if motor control allowed, false if safety checks fail
     * 
     * @warning Critical safety function - must verify safe conditions
     */
    bool mavlink_motor_control_check(const GCS_MAVLINK &gcs_chan, bool check_rc, const char* mode);
    
    /**
     * @brief Start motor test via MAVLink command
     * 
     * @details Initiates motor test in response to MAVLink command from GCS.
     *          Configures test parameters and starts outputting test signals.
     *          
     *          Test configuration:
     *          - Motor selection (specific motor or all)
     *          - Throttle type (PWM, percentage, or throttle curve)
     *          - Throttle value
     *          - Test duration with automatic timeout
     *          - Number of motors to test
     *          
     *          Test automatically stops after timeout for safety.
     * 
     * @param[in] gcs_chan MAVLink channel sending command
     * @param[in] motor_seq Motor sequence number (0=all, 1-N=specific motor)
     * @param[in] throttle_type How throttle_value is interpreted (0=%, 1=PWM, 2=pilot)
     * @param[in] throttle_value Throttle value in specified type
     * @param[in] timeout_sec Test duration before automatic stop (seconds)
     * @param[in] motor_count Number of motors on vehicle
     * 
     * @return MAV_RESULT_ACCEPTED if test started, error code if failed
     * 
     * @warning Spins motors - ensure vehicle secured before starting test
     * @note Test stops automatically after timeout for safety
     */
    MAV_RESULT mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, uint8_t motor_seq, uint8_t throttle_type, float throttle_value, float timeout_sec, uint8_t motor_count);
    
    /**
     * @brief Stop motor test
     * 
     * @details Immediately terminates motor test and stops motor outputs. Called
     *          when test completes, timeout occurs, or pilot intervenes.
     * 
     * @note Returns vehicle to normal disarmed state
     */
    void motor_test_stop();

    // motors.cpp
    /**
     * @brief Check for automatic disarming conditions
     * 
     * @details Monitors for conditions that should trigger automatic disarming.
     *          Automatic disarming prevents vehicle from remaining armed indefinitely
     *          on ground, reducing risk of accidental throttle application.
     *          
     *          Auto-disarm triggers:
     *          - Vehicle on ground with zero throttle for timeout period
     *          - Land mode complete and vehicle stationary
     *          - Emergency landing complete
     *          - Pilot requested disarm via switch or stick pattern
     *          
     *          Auto-disarm delay allows brief ground operations without re-arming
     *          but prevents armed vehicle being left unattended.
     * 
     * @note Auto-disarm disabled in some modes (e.g., Auto mission on ground waypoint)
     * @see Parameters: DISARM_DELAY controls timeout period
     */
    void auto_disarm_check();
    
    /**
     * @brief Send motor outputs to ESCs
     * 
     * @details Outputs final motor commands to motor ESCs via PWM, DShot, or other
     *          motor protocol. This is the final stage of control pipeline where
     *          calculated motor values are sent to hardware.
     *          
     *          Motor output process:
     *          1. Apply final safety checks
     *          2. Limit outputs to configured ranges
     *          3. Handle motor interlock state
     *          4. Apply emergency stops if needed
     *          5. Send PWM/DShot signals to ESCs
     *          
     *          Motor outputs blocked if:
     *          - Vehicle disarmed
     *          - Motor interlock disabled
     *          - Pre-arm checks failed
     *          - Crash detected
     * 
     * @param[in] full_push true to output even with low CPU (optional, default=true)
     * 
     * @warning Critical safety function - final motor output stage
     * @note Called every main loop iteration
     * @see AP_Motors::output() for motor library implementation
     */
    void motors_output(bool full_push = true);
    
    /**
     * @brief Main motor output function
     * 
     * @details Primary motor output function called from main loop. Coordinates
     *          motor output with safety checks and control updates.
     * 
     * @note Wrapper for motors_output() called from scheduler
     */
    void motors_output_main();
    
    /**
     * @brief Check for lost vehicle conditions
     * 
     * @details Monitors for conditions indicating vehicle control may be lost.
     *          Provides warnings to pilot when vehicle behavior indicates potential
     *          loss of control (high vibration, attitude errors, etc.).
     *          
     *          Lost vehicle indicators:
     *          - Large attitude errors persisting
     *          - High vibration levels affecting control
     *          - Control saturation
     *          - Motor failures detected
     *          - Uncommanded attitude changes
     *          
     *          Provides early warning to pilot so corrective action can be taken
     *          before complete loss of control.
     * 
     * @note Sends warnings to GCS and pilot notifications
     */
    void lost_vehicle_check();

    // navigation.cpp
    /**
     * @brief Run navigation updates
     * 
     * @details Updates all navigation controllers and position estimators. Called
     *          periodically to maintain navigation state for autonomous flight modes.
     *          
     *          Navigation updates include:
     *          - Waypoint navigation (wp_nav)
     *          - Loiter controller (loiter_nav)
     *          - Circle navigation (circle_nav)
     *          - Position controller state
     *          - Home position updates
     *          - Mission item processing
     *          
     *          Ensures navigation controllers have current data for autonomous
     *          flight mode operation.
     * 
     * @note Called from main loop when in navigation modes
     * @see AC_WPNav, AC_Loiter, AC_Circle for navigation controllers
     */
    void run_nav_updates(void);
    
    /**
     * @brief Get bearing to home location
     * 
     * @details Calculates bearing from current position to home location. Used for
     *          RTL navigation, pilot awareness displays, and simple mode calculations.
     *          
     *          Bearing calculated from:
     *          - Current position (from EKF/GPS)
     *          - Home position (set at arming or manually)
     *          
     *          Bearing is true heading (not magnetic) in earth frame.
     * 
     * @return Bearing to home in centidegrees (0-36000, 0=North)
     * 
     * @note Updated periodically for efficiency
     * @see home_distance() for distance to home
     */
    int32_t home_bearing();
    
    /**
     * @brief Get distance to home location
     * 
     * @details Calculates horizontal distance from current position to home location.
     *          Used for RTL navigation, geofence checks, and pilot awareness displays.
     *          
     *          Distance is horizontal 2D distance ignoring altitude difference.
     * 
     * @return Distance to home in centimeters
     * 
     * @note Updated periodically for efficiency
     * @see home_bearing() for direction to home
     */
    uint32_t home_distance();

    // Parameters.cpp
    /**
     * @brief Load all vehicle parameters from storage
     * 
     * @details Loads parameters from persistent storage (EEPROM/flash) during boot.
     *          Initializes all parameter values to stored values or defaults if
     *          parameters not previously saved.
     *          
     *          Parameter loading process:
     *          1. Read parameter format version
     *          2. Load parameter values from storage
     *          3. Apply default values for missing parameters
     *          4. Convert old parameter formats if needed
     *          5. Validate parameter ranges
     *          6. Apply parameter values to systems
     *          
     *          Handles parameter format changes between firmware versions.
     * 
     * @note Overrides AP_Vehicle::load_parameters()
     * @note Called once during init_ardupilot() initialization
     * @see AP_Param for parameter storage system
     */
    void load_parameters(void) override;
    
    /**
     * @brief Convert PID parameters from old format to new format
     * 
     * @details Converts PID controller parameters when parameter format changes
     *          between firmware versions. Ensures smooth firmware updates without
     *          losing tuning.
     *          
     *          Parameter conversion handles:
     *          - Old gain names → new gain names
     *          - Old scaling factors → new scaling factors
     *          - Combined parameters → separate parameters
     *          - Parameter tree restructuring
     *          
     *          Preserves user tuning when upgrading firmware.
     * 
     * @note Called during parameter loading if conversion needed
     */
    void convert_pid_parameters(void);
    
#if HAL_PROXIMITY_ENABLED
    /**
     * @brief Convert proximity sensor parameters from old format
     * 
     * @details Converts proximity sensor parameters when format changes. Preserves
     *          proximity sensor configuration across firmware updates.
     * 
     * @note Only compiled when proximity sensors enabled
     */
    void convert_prx_parameters();
#endif
    
    /**
     * @brief Convert landing gear parameters from old format
     * 
     * @details Converts landing gear parameters to new format. Handles parameter
     *          name changes and format updates for landing gear configuration.
     * 
     * @note Preserves landing gear configuration across updates
     */
    void convert_lgr_parameters(void);

    // precision_landing.cpp
    /**
     * @brief Initialize precision landing system
     * 
     * @details Initializes precision landing (PRECLAND) subsystem for landing on
     *          IR beacon or visual target. Sets up precision landing state machine
     *          and configures landing target detection.
     *          
     *          Precision landing supports:
     *          - IR-LOCK infrared beacon tracking
     *          - Visual target detection via companion computer
     *          - Rangefinder-based height control for accuracy
     *          
     *          Precision landing provides centimeter-level landing accuracy on
     *          marked targets.
     * 
     * @note Only active when precision landing enabled in parameters
     * @see AC_PrecLand for precision landing controller
     */
    void init_precland();
    
    /**
     * @brief Update precision landing system
     * 
     * @details Updates precision landing target tracking and control. Processes
     *          target detection, estimates target position, and provides position
     *          corrections to achieve precise landing on target.
     *          
     *          Precision landing process:
     *          1. Detect landing target (IR beacon or visual)
     *          2. Estimate target position from detections
     *          3. Filter target position estimate
     *          4. Calculate position corrections
     *          5. Apply corrections to Land mode controller
     *          
     *          Provides real-time guidance corrections during landing descent.
     * 
     * @note Called during Land mode when precision landing active
     */
    void update_precland();

    // radio.cpp
    /**
     * @brief Set default RC input dead zones
     * 
     * @details Configures default dead zone values for RC stick inputs. Dead zones
     *          prevent stick drift and noise from causing unintended control inputs.
     *          
     *          Dead zones applied to:
     *          - Roll stick
     *          - Pitch stick
     *          - Yaw stick
     *          - Throttle stick
     *          
     *          Dead zone size balances control precision versus noise immunity.
     * 
     * @note Called during RC initialization
     */
    void default_dead_zones();
    
    /**
     * @brief Initialize RC input system
     * 
     * @details Configures RC input channels and mapping. Sets up channel assignments
     *          for primary controls and auxiliary functions.
     *          
     *          RC input initialization:
     *          - Map RC channels to control functions
     *          - Configure input ranges
     *          - Set dead zones
     *          - Initialize aux switch functions
     *          - Configure RC failsafe
     * 
     * @note Called during vehicle initialization
     * @see RC_Channel for channel management
     */
    void init_rc_in();
    
    /**
     * @brief Initialize RC output channels
     * 
     * @details Configures RC output channels for servo/motor control. Sets up
     *          output ranges, trim values, and function assignments.
     *          
     *          RC output initialization:
     *          - Configure motor output channels
     *          - Set servo function assignments
     *          - Configure output ranges
     *          - Enable output drivers
     * 
     * @note Called during vehicle initialization
     */
    void init_rc_out();
    
    /**
     * @brief Read RC input values
     * 
     * @details Reads current RC receiver input values and processes them for flight
     *          control. Applies dead zones, scaling, and reverse settings.
     *          
     *          RC processing:
     *          1. Read raw PWM values from receiver
     *          2. Detect new frame arrival
     *          3. Apply dead zones
     *          4. Apply reverse settings
     *          5. Scale to control ranges
     *          6. Update control variables
     *          
     *          Also checks for RC failsafe conditions.
     * 
     * @note Called every main loop iteration
     * @see set_throttle_and_failsafe() for failsafe handling
     */
    void read_radio();
    
    /**
     * @brief Set throttle value and check for RC failsafe
     * 
     * @details Processes throttle channel input and monitors for RC signal loss.
     *          Updates throttle control value and triggers RC failsafe if signal
     *          lost or invalid.
     *          
     *          RC failsafe detection:
     *          - Throttle below failsafe threshold
     *          - RC signal timeout
     *          - Invalid RC data
     *          
     *          RC failsafe typically triggers RTL or Land mode for safety.
     * 
     * @param[in] throttle_pwm Raw throttle PWM value from receiver (microseconds)
     * 
     * @warning Critical safety function - detects RC signal loss
     * @see failsafe_radio_on_event() when failsafe triggers
     */
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    
    /**
     * @brief Update throttle zero flag
     * 
     * @details Determines whether pilot intends throttle to be at zero (motor shutoff).
     *          Detects pilot's intention to shut down motors versus temporarily
     *          reducing throttle. Used with debouncing to prevent accidental shutoff.
     *          
     *          Throttle zero detection considers:
     *          - Throttle stick position below threshold
     *          - Debounce time to avoid transients
     *          - Motor interlock state
     *          
     *          Throttle zero flag affects auto-disarming and motor interlock.
     * 
     * @param[in] throttle_control Current throttle control value
     * 
     * @note Sets ap.throttle_zero flag
     */
    void set_throttle_zero_flag(int16_t throttle_control);
    
    /**
     * @brief Pass RC inputs directly to motors (manual passthrough)
     * 
     * @details Bypasses normal flight controller and passes RC inputs directly to
     *          motor outputs. Used for ESC calibration where motors must respond
     *          to raw throttle input without stabilization.
     *          
     *          Manual passthrough mode:
     *          - Throttle channel → all motor outputs equally
     *          - No attitude stabilization
     *          - No mixing or compensation
     *          - Direct PWM passthrough
     * 
     * @warning Motors spin without stabilization - vehicle will not fly stable
     * @note Only used during ESC calibration procedure
     * @see esc_calibration_passthrough()
     */
    void radio_passthrough_to_motors();
    
    /**
     * @brief Get throttle mid position value
     * 
     * @details Returns throttle stick mid position (hover throttle). This is the
     *          throttle value where vehicle hovers with neutral attitude. Used as
     *          reference for throttle scaling in altitude hold modes.
     *          
     *          Hover throttle learned automatically during flight and stored in
     *          MOT_HOVER_LEARN parameter.
     * 
     * @return Throttle mid position (0-1000 range)
     * 
     * @note Typically 40-60% throttle for most multirotors
     */
    int16_t get_throttle_mid(void);

    // sensors.cpp
    /**
     * @brief Read barometer sensor
     * 
     * @details Reads pressure data from barometer sensor and calculates altitude.
     *          Barometer provides primary altitude reference for altitude hold and
     *          navigation.
     *          
     *          Barometer processing:
     *          1. Read raw pressure from sensor
     *          2. Apply temperature compensation
     *          3. Calculate altitude from pressure
     *          4. Apply calibration offset
     *          5. Update baro altitude variable
     *          
     *          Barometer reading includes multi-sensor averaging if multiple
     *          barometers installed.
     * 
     * @note Called periodically from main loop
     * @see AP_Baro for barometer management
     */
    void read_barometer(void);
    
    /**
     * @brief Initialize rangefinder system
     * 
     * @details Initializes distance sensor (rangefinder) for terrain following and
     *          precision landing. Configures downward-facing and upward-facing
     *          rangefinders if present.
     *          
     *          Rangefinder initialization:
     *          - Detect connected rangefinders
     *          - Configure measurement parameters
     *          - Set orientation and mounting
     *          - Initialize surface tracking
     * 
     * @note Called during vehicle initialization
     * @see AP_RangeFinder for rangefinder management
     */
    void init_rangefinder(void);
    
    /**
     * @brief Read rangefinder sensors
     * 
     * @details Reads distance measurements from rangefinders. Processes downward
     *          and upward rangefinders for terrain following and ceiling detection.
     *          
     *          Rangefinder processing:
     *          1. Read raw distance measurements
     *          2. Filter measurements for noise
     *          3. Check for valid data
     *          4. Update rangefinder state
     *          5. Calculate terrain offset
     *          
     *          Rangefinder data used for terrain following, precision landing,
     *          and surface tracking modes.
     * 
     * @note Called periodically from main loop
     */
    void read_rangefinder(void);
    
    /**
     * @brief Check if downward rangefinder altitude is valid
     * 
     * @details Determines whether downward-facing rangefinder is providing reliable
     *          altitude measurement. Checks data validity, range limits, and health.
     *          
     *          Rangefinder considered valid when:
     *          - Sensor is healthy
     *          - Distance within valid range
     *          - Data recently updated
     *          - Terrain type suitable for sensor
     * 
     * @return true if rangefinder altitude reliable, false otherwise
     * 
     * @note Used before relying on rangefinder for control
     */
    bool rangefinder_alt_ok() const;
    
    /**
     * @brief Check if upward rangefinder altitude is valid
     * 
     * @details Determines whether upward-facing rangefinder is providing reliable
     *          ceiling distance measurement. Used for ceiling tracking mode.
     * 
     * @return true if upward rangefinder reliable, false otherwise
     * 
     * @note Used for indoor ceiling following
     */
    bool rangefinder_up_ok() const;
    
    /**
     * @brief Update rangefinder terrain offset calculation
     * 
     * @details Updates terrain altitude offset using rangefinder measurements.
     *          Allows navigation relative to terrain rather than sea level.
     *          
     *          Terrain offset calculation:
     *          - Combine rangefinder distance with barometer altitude
     *          - Filter terrain estimate
     *          - Detect terrain changes
     *          - Provide terrain-relative altitude
     *          
     *          Enables terrain following in areas without terrain database.
     * 
     * @note Called when rangefinder data available
     */
    void update_rangefinder_terrain_offset();
    
    /**
     * @brief Update optical flow sensor
     * 
     * @details Reads and processes optical flow sensor for velocity estimation.
     *          Optical flow provides ground velocity measurements when GPS not
     *          available, enabling position hold indoors.
     *          
     *          Optical flow processing:
     *          1. Read flow rates from sensor
     *          2. Combine with rangefinder height
     *          3. Calculate ground velocity
     *          4. Provide to EKF for position estimation
     *          
     *          Optical flow enables GPS-free position control in FlowHold mode.
     * 
     * @note Only active when optical flow sensor installed
     * @see AP_OpticalFlow for flow sensor management
     */
    void update_optical_flow(void);

    // takeoff_check.cpp
    /**
     * @brief Check for unsafe takeoff conditions
     * 
     * @details Monitors vehicle state for conditions that indicate unsafe takeoff.
     *          Warns pilot if vehicle attempting to take off with configuration
     *          problems or environmental hazards.
     *          
     *          Takeoff safety checks:
     *          - High vibration levels
     *          - Large attitude errors
     *          - Compass inconsistencies
     *          - Low battery voltage
     *          - GPS glitching
     *          - EKF variances high
     *          
     *          Warnings allow pilot to abort takeoff and address issues before
     *          committing to flight.
     * 
     * @note Provides warnings but does not prevent takeoff
     * @warning Pilot should abort takeoff if warnings present
     */
    void takeoff_check();

    // system.cpp
    /**
     * @brief Initialize ArduPilot vehicle systems
     * 
     * @details Main initialization function called at boot. Initializes all vehicle
     *          subsystems, sensors, and control systems in correct sequence.
     *          
     *          Initialization sequence:
     *          1. Hardware initialization (HAL setup)
     *          2. Parameter loading from storage
     *          3. Sensor initialization (IMU, compass, baro, GPS)
     *          4. AHRS/EKF initialization
     *          5. Control system initialization
     *          6. RC and motor initialization
     *          7. Mission and navigation setup
     *          8. GCS communication setup
     *          9. Logging initialization
     *          10. Complete initialization and enable flight
     *          
     *          Sets ap.initialised flag when complete.
     * 
     * @note Overrides AP_Vehicle::init_ardupilot()
     * @warning Vehicle not ready for flight until init complete
     * @see ap.initialised flag for initialization status
     */
    void init_ardupilot() override;
    
    /**
     * @brief Perform ground-based IMU initialization
     * 
     * @details Performs IMU calibration and initialization while vehicle on ground.
     *          Calculates gyro offsets and accel calibration with vehicle stationary.
     *          
     *          Ground initialization process:
     *          1. Require vehicle stationary
     *          2. Sample IMU for several seconds
     *          3. Calculate gyro zero offsets
     *          4. Calculate accelerometer offsets
     *          5. Initialize AHRS with level attitude
     *          
     *          IMU initialization must complete before arming permitted.
     * 
     * @warning Vehicle must remain stationary during initialization
     * @note Takes several seconds to complete
     */
    void startup_INS_ground();
    
    /**
     * @brief Check if position estimate is reliable
     * 
     * @details Determines whether position estimate from EKF/GPS is accurate enough
     *          for navigation. Checks estimate quality, variance, and availability.
     *          
     *          Position OK requires:
     *          - EKF converged and healthy
     *          - Position estimate variance acceptable
     *          - Position source available (GPS, optical flow, etc.)
     *          - No GPS glitching
     *          
     *          Many flight modes require position OK before arming or mode entry.
     * 
     * @return true if position estimate reliable for navigation, false otherwise
     * 
     * @note Critical check for GPS-dependent modes
     */
    bool position_ok() const;
    
    /**
     * @brief Check if EKF has absolute position estimate
     * 
     * @details Determines whether EKF has absolute position reference (GPS or
     *          external navigation). Absolute position allows global waypoint
     *          navigation.
     *          
     *          Absolute position sources:
     *          - GPS with good fix
     *          - External vision system with global coordinates
     *          - Beacon positioning system
     * 
     * @return true if EKF has absolute position, false otherwise
     * 
     * @note Required for waypoint missions and RTL
     */
    bool ekf_has_absolute_position() const;
    
    /**
     * @brief Check if EKF has relative position estimate
     * 
     * @details Determines whether EKF has relative position estimate from any source.
     *          Relative position allows local position hold even without GPS.
     *          
     *          Relative position sources:
     *          - GPS
     *          - Optical flow + rangefinder
     *          - Visual odometry
     *          - Beacon system
     * 
     * @return true if EKF has relative position, false otherwise
     * 
     * @note Sufficient for Loiter mode, not sufficient for waypoint navigation
     */
    bool ekf_has_relative_position() const;
    
    /**
     * @brief Check if EKF altitude estimate is reliable
     * 
     * @details Determines whether EKF altitude estimate is accurate enough for
     *          altitude control. Checks barometer health, altitude variance, and
     *          estimate convergence.
     *          
     *          Altitude OK requires:
     *          - Barometer healthy
     *          - Altitude variance acceptable
     *          - EKF converged
     * 
     * @return true if altitude estimate reliable, false otherwise
     * 
     * @note Required for altitude hold modes
     */
    bool ekf_alt_ok() const;
    
    /**
     * @brief Update auto-armed state
     * 
     * @details Updates auto-armed flag which gates autonomous missions. Auto-armed
     *          prevents mission from starting until pilot raises throttle, ensuring
     *          pilot ready for autonomous flight.
     *          
     *          Auto-armed becomes true when:
     *          - Vehicle armed
     *          - Throttle raised above minimum
     *          - Pilot demonstrated readiness
     *          
     *          Prevents surprise autonomous takeoff at mission start.
     * 
     * @note Sets ap.auto_armed flag
     */
    void update_auto_armed();
    
    /**
     * @brief Check if specified log type should be logged
     * 
     * @details Determines whether specified log message type is enabled in logging
     *          bitmask. Allows selective logging to manage storage and performance.
     * 
     * @param[in] mask Log type bitmask to check
     * 
     * @return true if this log type should be logged, false otherwise
     * 
     * @note Checks against LOG_BITMASK parameter
     */
    bool should_log(uint32_t mask);
    
    /**
     * @brief Get frame type string for display
     * 
     * @details Returns human-readable string describing frame configuration.
     *          Used for display in GCS and logging.
     *          
     *          Frame types:
     *          - "Quad" - Quadcopter
     *          - "Hexa" - Hexacopter
     *          - "Octa" - Octocopter
     *          - "OctaQuad" - Octo-quad
     *          - "Y6" - Y6 configuration
     *          - "Heli" - Traditional helicopter
     *          - etc.
     * 
     * @return Pointer to frame type string
     * 
     * @note Used for vehicle identification
     */
    const char* get_frame_string() const;
    
    /**
     * @brief Allocate motor control object
     * 
     * @details Dynamically allocates appropriate motor control object based on
     *          frame configuration. Creates multicopter or helicopter motor
     *          controller as appropriate.
     *          
     *          Motor allocation:
     *          - Detect frame type from parameters
     *          - Allocate matching motor class
     *          - Initialize motor configuration
     *          - Setup motor mixing
     * 
     * @note Called during vehicle initialization
     * @see AP_Motors for motor control library
     */
    void allocate_motors(void);
    
    /**
     * @brief Check if vehicle is traditional helicopter
     * 
     * @details Determines whether vehicle is configured as traditional helicopter
     *          (single main rotor + tail rotor) versus multicopter.
     * 
     * @return true if traditional helicopter, false if multicopter
     * 
     * @note Used for helicopter-specific control logic
     */
    bool is_tradheli() const;

    // terrain.cpp
    /**
     * @brief Update terrain altitude database
     * 
     * @details Updates terrain altitude data from terrain database or rangefinder.
     *          Provides terrain-relative altitude for terrain following modes.
     *          
     *          Terrain data sources:
     *          - Terrain database (stored elevation data)
     *          - Rangefinder-derived terrain estimate
     *          - GCS-provided terrain data
     *          
     *          Terrain following enables low-altitude flight following ground
     *          contours for mapping and inspection.
     * 
     * @note Only active when terrain following enabled
     * @see AP_Terrain for terrain database
     */
    void terrain_update();
    
    /**
     * @brief Log terrain data
     * 
     * @details Logs terrain altitude and terrain-following status. Records terrain
     *          database health, current terrain altitude, and terrain-relative height.
     *          
     *          Used for analyzing terrain following performance and debugging
     *          terrain database issues.
     * 
     * @note Called periodically when terrain following active
     */
    void terrain_logging();

    // tuning.cpp
    /**
     * @brief Perform in-flight parameter tuning
     * 
     * @details Updates parameter values based on RC channel input for in-flight
     *          tuning. Allows real-time adjustment of PID gains and other parameters
     *          via transmitter knob or slider.
     *          
     *          Tuning process:
     *          1. Read tuning channel value
     *          2. Scale to parameter range
     *          3. Update parameter value
     *          4. Log tuning activity
     *          
     *          Enables field tuning without landing and reconnecting GCS.
     * 
     * @note Tuning channel configured via TUNE parameter
     * @see Log_Write_Parameter_Tuning() for tuning data logging
     */
    void tuning();

    // UserCode.cpp
    /**
     * @brief User initialization hook
     * 
     * @details Custom user code called once during vehicle initialization. Allows
     *          users to add custom initialization without modifying core code.
     *          
     *          User hook for:
     *          - Custom sensor initialization
     *          - Custom peripheral setup
     *          - User-specific configuration
     * 
     * @note Empty by default - users add code as needed
     * @warning Must not block - called during boot sequence
     */
    void userhook_init();
    
    /**
     * @brief User fast loop hook
     * 
     * @details Custom user code called at main loop rate (typically 400Hz). For
     *          time-critical user code that must run at high rate.
     *          
     *          Fast loop user code for:
     *          - High-rate sensor reading
     *          - Fast custom control algorithms
     *          - Time-critical processing
     * 
     * @note Empty by default - users add code as needed
     * @warning Must execute very quickly - affects main loop timing
     * @warning Called at ~400Hz - keep execution time under 100µs
     */
    void userhook_FastLoop();
    
    /**
     * @brief User 50Hz loop hook
     * 
     * @details Custom user code called at 50Hz. For moderate-rate user processing.
     * 
     * @note Empty by default - users add code as needed
     * @warning Keep execution time reasonable - affects scheduler timing
     */
    void userhook_50Hz();
    
    /**
     * @brief User medium loop hook
     * 
     * @details Custom user code called at medium rate (typically 100Hz). For
     *          medium-priority user processing.
     * 
     * @note Empty by default - users add code as needed
     */
    void userhook_MediumLoop();
    
    /**
     * @brief User slow loop hook
     * 
     * @details Custom user code called at slow rate (typically 10Hz). For
     *          low-priority user processing that doesn't need high rate.
     *          
     *          Slow loop user code for:
     *          - Status checks
     *          - Slow sensor reading
     *          - User interface updates
     * 
     * @note Empty by default - users add code as needed
     */
    void userhook_SlowLoop();
    
    /**
     * @brief User super slow loop hook
     * 
     * @details Custom user code called at very slow rate (typically 1Hz). For
     *          infrequent user processing.
     *          
     *          Super slow loop user code for:
     *          - Periodic status logging
     *          - Infrequent checks
     *          - Background tasks
     * 
     * @note Empty by default - users add code as needed
     */
    void userhook_SuperSlowLoop();
    
    /**
     * @brief User auxiliary switch 1 hook
     * 
     * @details Custom user code called when auxiliary switch 1 changes position.
     *          Allows users to add custom functions triggered by RC switches.
     * 
     * @param[in] ch_flag Switch position (LOW, MIDDLE, HIGH)
     * 
     * @note Empty by default - users add code as needed
     */
    void userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag);
    
    /**
     * @brief User auxiliary switch 2 hook
     * 
     * @details Custom user code called when auxiliary switch 2 changes position.
     * 
     * @param[in] ch_flag Switch position (LOW, MIDDLE, HIGH)
     * 
     * @note Empty by default - users add code as needed
     */
    void userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag);
    
    /**
     * @brief User auxiliary switch 3 hook
     * 
     * @details Custom user code called when auxiliary switch 3 changes position.
     * 
     * @param[in] ch_flag Switch position (LOW, MIDDLE, HIGH)
     * 
     * @note Empty by default - users add code as needed
     */
    void userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag);

    // Flight Mode Objects
    // Each flight mode has a dedicated object instance that implements mode-specific
    // control logic. Only enabled modes are compiled into firmware to save flash space.
    
#if MODE_ACRO_ENABLED
#if FRAME_CONFIG == HELI_FRAME
    ModeAcro_Heli mode_acro;  /**< @brief Acro mode for helicopters - rate control with manual collective */
#else
    ModeAcro mode_acro;  /**< @brief Acro mode - pilot controls rotation rates directly, no self-leveling */
#endif
#endif
    
    ModeAltHold mode_althold;  /**< @brief Altitude Hold mode - holds altitude, pilot controls attitude */
    
#if MODE_AUTO_ENABLED
    ModeAuto mode_auto;  /**< @brief Auto mode - executes programmed waypoint missions */
#endif
    
#if AUTOTUNE_ENABLED
    ModeAutoTune mode_autotune;  /**< @brief AutoTune mode - automatically tunes attitude control PIDs */
#endif
    
#if MODE_BRAKE_ENABLED
    ModeBrake mode_brake;  /**< @brief Brake mode - aggressive position hold with rapid deceleration */
#endif
    
#if MODE_CIRCLE_ENABLED
    ModeCircle mode_circle;  /**< @brief Circle mode - orbits around a center point */
#endif
    
#if MODE_DRIFT_ENABLED
    ModeDrift mode_drift;  /**< @brief Drift mode - simplified control for easy flying */
#endif
    
#if MODE_FLIP_ENABLED
    ModeFlip mode_flip;  /**< @brief Flip mode - performs automated aerobatic flip */
#endif
    
#if MODE_FOLLOW_ENABLED
    ModeFollow mode_follow;  /**< @brief Follow mode - follows another vehicle */
#endif
    
#if MODE_GUIDED_ENABLED
    ModeGuided mode_guided;  /**< @brief Guided mode - accepts real-time position/velocity commands from GCS */
#if AP_SCRIPTING_ENABLED
    ModeGuidedCustom *mode_guided_custom[5];  /**< @brief Custom flight modes registered via Lua scripting */
#endif
#endif
    
    ModeLand mode_land;  /**< @brief Land mode - automated landing sequence */
    
#if MODE_LOITER_ENABLED
    ModeLoiter mode_loiter;  /**< @brief Loiter mode - holds position using GPS */
#endif
    
#if MODE_POSHOLD_ENABLED
    ModePosHold mode_poshold;  /**< @brief Position Hold mode - holds position when sticks centered */
#endif
    
#if MODE_RTL_ENABLED
    ModeRTL mode_rtl;  /**< @brief RTL mode - Return to Launch and land autonomously */
#endif
    
#if FRAME_CONFIG == HELI_FRAME
    ModeStabilize_Heli mode_stabilize;  /**< @brief Stabilize mode for helicopters - self-levels with manual collective */
#else
    ModeStabilize mode_stabilize;  /**< @brief Stabilize mode - self-levels when sticks centered */
#endif
    
#if MODE_SPORT_ENABLED
    ModeSport mode_sport;  /**< @brief Sport mode - rate control in yaw, angle control in roll/pitch */
#endif
    
#if MODE_SYSTEMID_ENABLED
    ModeSystemId mode_systemid;  /**< @brief System ID mode - injects test signals for system identification */
#endif
    
#if AP_ADSB_AVOIDANCE_ENABLED
    ModeAvoidADSB mode_avoid_adsb;  /**< @brief ADSB Avoidance mode - avoids manned aircraft */
#endif  // AP_ADSB_AVOIDANCE_ENABLED
    
#if MODE_THROW_ENABLED
    ModeThrow mode_throw;  /**< @brief Throw mode - arms and starts motors when thrown */
#endif
    
#if MODE_GUIDED_NOGPS_ENABLED
    ModeGuidedNoGPS mode_guided_nogps;  /**< @brief Guided NoGPS mode - accepts velocity commands without GPS */
#endif
    
#if MODE_SMARTRTL_ENABLED
    ModeSmartRTL mode_smartrtl;  /**< @brief Smart RTL mode - returns via recorded path */
#endif
    
#if MODE_FLOWHOLD_ENABLED
    ModeFlowHold mode_flowhold;  /**< @brief FlowHold mode - holds position using optical flow sensor */
#endif
    
#if MODE_ZIGZAG_ENABLED
    ModeZigZag mode_zigzag;  /**< @brief ZigZag mode - records waypoints for crop spraying pattern */
#endif
    
#if MODE_AUTOROTATE_ENABLED
    ModeAutorotate mode_autorotate;  /**< @brief Autorotate mode - helicopter autorotation landing after engine failure */
#endif
    
#if MODE_TURTLE_ENABLED
    ModeTurtle mode_turtle;  /**< @brief Turtle mode - flips crashed multicopter right-side-up */
#endif

    // mode.cpp
    /**
     * @brief Get mode object from mode number
     * 
     * @details Converts flight mode number to pointer to corresponding mode object.
     *          Returns nullptr if mode number invalid or mode not enabled.
     *          
     *          Used for mode switching and mode access throughout codebase.
     * 
     * @param[in] mode Flight mode number
     * 
     * @return Pointer to mode object, or nullptr if mode invalid
     * 
     * @note Returns nullptr for disabled modes
     */
    Mode *mode_from_mode_num(const Mode::Number mode);
    
    /**
     * @brief Exit current mode and enter new mode
     * 
     * @details Handles flight mode transition. Calls exit() on old mode and enter()
     *          on new mode. Manages mode state cleanup and initialization.
     *          
     *          Mode transition process:
     *          1. Call old_flightmode->exit()
     *          2. Update flightmode pointer
     *          3. Call new_flightmode->enter()
     *          4. Update mode state
     *          
     *          Ensures clean transition between modes.
     * 
     * @param[in,out] old_flightmode Reference to old mode pointer
     * @param[in,out] new_flightmode Reference to new mode pointer
     * 
     * @note Both mode pointers updated by this function
     */
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

    bool started_rate_thread;  /**< @brief True if fast rate control thread has been started */
    bool using_rate_thread;    /**< @brief True if fast rate control thread is currently active */

public:
    /**
     * @brief Check all failsafe conditions
     * 
     * @details Master failsafe check function called periodically to monitor all
     *          failsafe conditions. Checks radio, GCS, battery, EKF, and terrain
     *          failsafes and triggers appropriate failsafe actions.
     *          
     *          Failsafe checks performed:
     *          - Radio signal loss
     *          - GCS connection loss
     *          - Battery voltage/capacity low
     *          - EKF variance excessive
     *          - Terrain data unavailable
     *          - Dead reckoning timeout
     *          - ADSB collision threat
     *          
     *          Each failsafe can trigger different actions (RTL, Land, etc.) based
     *          on configuration and priority.
     * 
     * @note Called from main loop at 100Hz
     * @warning Critical safety function - monitors all vehicle failsafes
     * @see any_failsafe_triggered() to check if any failsafe active
     */
    void failsafe_check();      // failsafe.cpp
};

/**
 * @brief Global Copter instance
 * 
 * @details Singleton instance of Copter class accessed throughout codebase.
 *          The copter object is the main vehicle controller and orchestrates
 *          all flight control systems.
 *          
 *          Access pattern:
 *          - Libraries access via copter.function()
 *          - Mode classes access via friend declaration
 *          - Global functions access directly
 *          
 *          This global instance implements the singleton pattern for vehicle control.
 * 
 * @note Single global instance - accessed throughout ArduCopter code
 */
extern Copter copter;

using AP_HAL::millis;
using AP_HAL::micros;
