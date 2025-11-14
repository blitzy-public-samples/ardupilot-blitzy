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
 * @file Blimp.h
 * @brief Main Blimp vehicle class definition for lighter-than-air flight control
 * 
 * @details This header defines the comprehensive Blimp class which serves as the main
 *          vehicle controller for lighter-than-air vehicles (blimps/airships). The class
 *          contains all subsystems required for blimp flight including:
 *          
 *          - Parameter containers (g, g2) for configuration storage
 *          - PID controllers for position and velocity control in all axes
 *          - Fins and Loiter control objects for actuator management
 *          - RC channel management for pilot input
 *          - Scheduler task arrays for real-time control loops
 *          - Failsafe priorities and state management
 *          - Flight mode system with Manual, Land, Velocity, Loiter, and RTL modes
 *          - Sensor interfaces (AHRS, inertial navigation, barometer)
 *          - Battery monitoring and failsafe handling
 *          - GCS (Ground Control Station) communication
 *          - Logging system integration
 *          
 *          The Blimp class inherits from AP_Vehicle and implements the vehicle-specific
 *          behavior for buoyant aircraft with vectored thrust control through fins.
 *          A global singleton instance 'blimp' provides system-wide access.
 * 
 * @note This implementation is designed for buoyant lighter-than-air vehicles
 * @warning Flight control modifications require thorough testing in SITL before hardware deployment
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
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>

// Application dependencies
#include <AP_Logger/AP_Logger.h>          // ArduPilot Mega Flash Memory Library
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
// #include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
// #include <AP_InertialSensor/AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS/AP_AHRS.h>
#include <Filter/Filter.h>             // Filter library
#include <AP_Vehicle/AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav/AP_InertialNav.h>     // inertial navigation library
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library
#include <AP_BattMonitor/AP_BattMonitor.h>     // Battery monitor library
#include <AP_Arming/AP_Arming.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AC_PID/AC_PID_2D.h>
#include <AC_PID/AC_PID_Basic.h>
#include <AC_PID/AC_PID.h>
#include <AP_Vehicle/AP_MultiCopter.h>

#include <Filter/NotchFilter.h>

// Configuration
#include "defines.h"
#include "config.h"

#include "Fins.h"
#include "Loiter.h"

#include "RC_Channel_Blimp.h"         // RC Channel Library

#include "GCS_MAVLink_Blimp.h"
#include "GCS_Blimp.h"
#include "AP_Arming_Blimp.h"

#include <AP_Mount/AP_Mount.h>

// Local modules

#include "Parameters.h"

#include "mode.h"

/**
 * @class Blimp
 * @brief Main vehicle class for lighter-than-air blimp/airship flight control
 * 
 * @details The Blimp class is the primary controller for lighter-than-air vehicles,
 *          implementing complete flight control for buoyant aircraft with vectored thrust.
 *          
 *          Key architectural components:
 *          
 *          **Control System Architecture:**
 *          - Cascaded PID control: Position → Velocity → Actuator commands
 *          - Separate controllers for XY (horizontal), Z (vertical), and yaw axes
 *          - Notch filters for velocity feedback to reject oscillations
 *          - Fins provide vectored thrust for attitude and position control
 *          
 *          **Flight Modes:**
 *          - Manual: Direct pilot control of velocities
 *          - Velocity: Velocity hold with pilot input
 *          - Loiter: Position hold using GPS
 *          - RTL: Return to launch point
 *          - Land: Controlled descent to ground
 *          
 *          **Sensor Fusion:**
 *          - AHRS for attitude estimation
 *          - Inertial navigation for position/velocity estimation in NED frame
 *          - EKF integration for sensor fusion
 *          - Barometer for altitude reference
 *          
 *          **Failsafe System:**
 *          - Radio failsafe (RC link loss)
 *          - GCS failsafe (telemetry link loss)
 *          - EKF failsafe (navigation failure)
 *          - Battery failsafe (low voltage/capacity)
 *          - Prioritized failsafe actions: Terminate > Land > None
 *          
 *          **Real-Time Scheduling:**
 *          - Main loop typically runs at 100Hz
 *          - Separate task scheduling for sensors, control, logging
 *          - RC input processed at frame rate
 *          - Multiple logging rates (full, 25Hz, 10Hz, 3Hz, 1Hz)
 *          
 *          **Coordinate Systems:**
 *          - NED (North-East-Down) for navigation and position control
 *          - Body frame for fin actuator commands
 *          - Transformations provided via rotate_BF_to_NE() and rotate_NE_to_BF()
 *          
 *          The class follows a singleton pattern with a global 'blimp' instance
 *          accessible throughout the codebase. Friend classes provide controlled
 *          access to private members for mode implementations, GCS communication,
 *          and parameter management.
 * 
 * @note Blimp-specific control accounts for buoyancy and slow dynamics compared to multirotors
 * @warning All position/velocity values use NED frame convention
 * @warning PID tuning significantly affects flight stability - test in SITL first
 * 
 * @see Mode Base class for flight mode implementations
 * @see Fins Fin/actuator control and mixing
 * @see Loiter Position hold controller
 */
class Blimp : public AP_Vehicle
{
public:
    friend class GCS_MAVLINK_Blimp;
    friend class GCS_Blimp;
    friend class Parameters;
    friend class ParametersG2;

    friend class AP_Arming_Blimp;
    friend class RC_Channel_Blimp;
    friend class RC_Channels_Blimp;

    friend class Mode;
    friend class ModeManual;
    friend class ModeLand;
    friend class ModeVelocity;
    friend class ModeLoiter;
    friend class ModeRTL;

    friend class Fins;
    friend class Loiter;

    /**
     * @brief Constructor for Blimp vehicle
     * 
     * @details Initializes the main Blimp object. Actual subsystem initialization
     *          occurs in init_ardupilot() called during startup sequence.
     */
    Blimp(void);

private:

    // ========================================================================
    // Parameter Storage
    // ========================================================================
    
    /**
     * @brief Multi-copter parameter structure shared with libraries
     * @details Contains key aircraft parameters passed to multiple control libraries
     */
    AP_MultiCopter aparm;

    /**
     * @brief Primary parameter group containing main vehicle configuration
     * @details Global parameters stored in EEPROM/storage including control gains,
     *          failsafe settings, logging configuration, and flight behavior parameters
     */
    Parameters g;
    
    /**
     * @brief Secondary parameter group for extended configuration
     * @details Additional parameters that don't fit in original parameter table,
     *          allowing parameter additions without EEPROM layout migration
     */
    ParametersG2 g2;

    // ========================================================================
    // RC Input Channels
    // ========================================================================
    
    /**
     * @brief RC channel for right/left (roll) control input
     * @details Pilot input for lateral movement in body frame
     */
    RC_Channel *channel_right;
    
    /**
     * @brief RC channel for forward/back (pitch) control input
     * @details Pilot input for forward/backward movement in body frame
     */
    RC_Channel *channel_front;
    
    /**
     * @brief RC channel for up/down (altitude) control input
     * @details Pilot input for vertical movement
     */
    RC_Channel *channel_up;
    
    /**
     * @brief RC channel for yaw (heading) control input
     * @details Pilot input for rotation around vertical axis
     */
    RC_Channel *channel_yaw;

    // ========================================================================
    // Flight Mode Management
    // ========================================================================
    
    /**
     * @brief Array of flight mode selections from RC switches
     * @details Convenience array for accessing mode selections, indexed by switch position
     */
    AP_Int8 *flight_modes;
    
    /**
     * @brief Number of available flight mode positions
     * @details Fixed at 6 mode slots selectable via RC switch
     */
    const uint8_t num_flight_modes = 6;

    // ========================================================================
    // Arming System
    // ========================================================================
    
    /**
     * @brief Arming/disarming management and pre-arm safety checks
     * @details Manages vehicle arming state, runs pre-arm checks (sensors, calibration,
     *          RC input, GPS lock), and enforces safety interlocks before flight
     * @warning Critical safety system - do not bypass checks
     */
    AP_Arming_Blimp arming;

    // ========================================================================
    // EKF State Tracking
    // ========================================================================
    
    /**
     * @brief System time in milliseconds of last recorded yaw reset from EKF
     * @details Used to detect and handle EKF yaw resets which require control response
     */
    uint32_t ekfYawReset_ms;
    
    /**
     * @brief Currently active EKF core/lane
     * @details Identifies which EKF instance is primary for navigation (-1 if none)
     */
    int8_t ekf_primary_core;

    // ========================================================================
    // Vibration Monitoring
    // ========================================================================
    
    /**
     * @brief Vibration detection and tracking state
     * @details Monitors high vibration levels which can affect sensor accuracy
     *          and control performance. Tracks start/clear times for logging.
     */
    struct {
        bool high_vibes;    ///< true while high vibration are detected
        uint32_t start_ms;  ///< system time high vibration were last detected
        uint32_t clear_ms;  ///< system time high vibrations stopped
    } vibration_check;

    // ========================================================================
    // Ground Control Station (GCS) Interface
    // ========================================================================
    
    /**
     * @brief Ground Control Station communication handler
     * @details Private GCS instance - use gcs() accessor method instead of direct access
     */
    GCS_Blimp _gcs;
    
    /**
     * @brief Get reference to GCS communication handler
     * @return Reference to the GCS_Blimp object for ground station communication
     * @note Prefer this accessor over direct _gcs access
     */
    GCS_Blimp &gcs()
    {
        return _gcs;
    }

    // ========================================================================
    // Vehicle State Flags (ap structure)
    // ========================================================================
    
    /**
     * @typedef ap_t
     * @brief Vehicle state flags packed into bitfield union
     * 
     * @details Compact representation of vehicle state using bitfields for efficiency.
     *          This union allows both individual bit access and whole-word operations.
     *          Size must be exactly uint32_t for efficient access.
     */
    typedef union {
        struct {
            uint8_t pre_arm_rc_check        : 1; ///< Bit 1: true if RC input pre-arm checks completed successfully
            uint8_t pre_arm_check           : 1; ///< Bit 2: true if all pre-arm checks (RC, sensors, GPS) passed
            uint8_t auto_armed              : 1; ///< Bit 3: true to prevent auto missions starting until throttle raised
            uint8_t logging_started         : 1; ///< Bit 4: true if data logging has started
            uint8_t land_complete           : 1; ///< Bit 5: true if landing detected (strict criteria)
            uint8_t new_radio_frame         : 1; ///< Bit 6: true when new PWM data received from RC receiver
            uint8_t rc_receiver_present_unused     : 1; ///< Bit 7: UNUSED - reserved for future use
            uint8_t compass_mot             : 1; ///< Bit 8: true during compass motor interference calibration
            uint8_t motor_test              : 1; ///< Bit 9: true during motor/fin output testing
            uint8_t initialised             : 1; ///< Bit 10: true after init_ardupilot() completes - enables GCS status
            uint8_t land_complete_maybe     : 1; ///< Bit 11: true if possibly landed (relaxed criteria)
            uint8_t throttle_zero           : 1; ///< Bit 12: true if throttle at zero (debounced) - pilot shutdown intent
            uint8_t gps_glitching           : 1; ///< Bit 13: true if GPS errors affecting navigation accuracy
            uint8_t in_arming_delay         : 1; ///< Bit 14: true while armed but waiting before spinning fins
            uint8_t initialised_params      : 1; ///< Bit 15: true when parameters loaded - enables GCS parameter ops
        };
        uint32_t value; ///< Access all flags as single 32-bit word
    } ap_t;

    /**
     * @brief Vehicle state flags instance
     * @details Contains all vehicle state bitflags for initialization, arming, landing detection, etc.
     */
    ap_t ap;

    static_assert(sizeof(uint32_t) == sizeof(ap), "ap_t must be uint32_t");

    // ========================================================================
    // Flight Mode State
    // ========================================================================
    
    /**
     * @brief Current active flight control mode
     * @details Determines which control mode is active (Manual, Velocity, Loiter, Land, RTL).
     *          Mode changes handled through set_mode() for proper state management.
     * @see set_mode()
     */
    Mode::Number control_mode;
    
    /**
     * @brief Reason for most recent mode change
     * @details Tracks why mode changed (pilot command, failsafe, GCS, etc.) for logging and GCS reporting
     */
    ModeReason control_mode_reason = ModeReason::UNKNOWN;

    /**
     * @brief RC input channel mapping
     * @details Maps RC channels to control functions, allowing flexible transmitter configuration
     */
    RCMapper rcmap;

    /**
     * @brief Inertial navigation altitude when vehicle was armed (meters)
     * @details Stored altitude at arming for reference in certain flight modes and checks
     */
    float arming_altitude_m;

    // ========================================================================
    // Failsafe System State
    // ========================================================================
    
    /**
     * @brief Failsafe state tracking structure
     * @details Monitors multiple failsafe conditions including RC link loss, GCS link loss,
     *          and EKF navigation failures. Each failsafe type has independent state tracking.
     * @warning Critical safety system - failsafe triggers initiate protective actions
     */
    struct {
        int8_t radio_counter;            ///< Number of consecutive iterations with throttle below failsafe threshold

        uint8_t radio               : 1; ///< Radio/RC link failsafe active flag
        uint8_t gcs                 : 1; ///< Ground Control Station link failsafe active flag
        uint8_t ekf                 : 1; ///< EKF navigation failsafe active flag (poor state estimation)
    } failsafe;

    /**
     * @brief Check if any failsafe condition is currently active
     * @return true if any failsafe (radio, battery, GCS, EKF) is triggered
     * @details Used to determine if vehicle is in failsafe state requiring protective action
     */
    bool any_failsafe_triggered() const
    {
        return failsafe.radio || battery.has_failsafed() || failsafe.gcs || failsafe.ekf;
    }

    // ========================================================================
    // Actuator Control Objects
    // ========================================================================
    
    /**
     * @brief Fin actuator control and mixing object
     * @details Controls fin outputs for thrust vectoring. Despite variable name "motors",
     *          this controls the fins that provide propulsion and attitude control for blimps.
     * @note Name "motors" inherited from multicopter architecture
     */
    Fins *motors;
    
    /**
     * @brief Position hold loiter controller
     * @details Implements position hold logic for maintaining station in Loiter mode
     */
    Loiter *loiter;

    // ========================================================================
    // Home Position Tracking
    // ========================================================================
    
    /**
     * @brief Bearing from current position to home in centidegrees
     * @details Updated periodically for RTL and display purposes (0-35999 centidegrees)
     */
    int32_t _home_bearing;
    
    /**
     * @brief Distance from current position to home in centimeters
     * @details Updated periodically for RTL navigation and display
     */
    uint32_t _home_distance;

    /**
     * @brief Initial bearing when vehicle was armed in centidegrees
     * @details Stored at arming for reference, independent of simple mode bearing adjustments
     */
    int32_t initial_armed_bearing;

    // ========================================================================
    // Battery Monitoring
    // ========================================================================
    
    /**
     * @brief Battery monitoring system with failsafe integration
     * @details Monitors battery voltage, current, and capacity. Triggers failsafe actions
     *          when battery reaches critical levels. Configured with logging mask,
     *          failsafe callback, and action priorities.
     * @warning Critical safety system - low battery triggers protective landing
     */
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                       FUNCTOR_BIND_MEMBER(&Blimp::handle_battery_failsafe, void, const char*, const int8_t),
                       _failsafe_priorities};

    // ========================================================================
    // Altitude and Vertical State
    // ========================================================================
    
    /**
     * @brief Barometric altitude in centimeters above home
     * @details Relative altitude from barometer, zeroed at arming location
     */
    int32_t baro_alt;

    /**
     * @brief Low-pass filtered throttle input for landing cancellation detection
     * @details Filtered pilot throttle used to detect if pilot intends to abort landing
     *          by advancing throttle
     */
    LowPassFilterFloat rc_throttle_control_in_filter;

    // ========================================================================
    // Position and Velocity State (NED Frame)
    // ========================================================================
    
    /**
     * @brief Current vehicle location in lat/lon/alt coordinates
     * @details GPS-based location with altitude relative to home position.
     *          Updated from EKF at control loop rate.
     */
    Location current_loc;
    
    /**
     * @brief Vehicle velocity in NED frame (North-East-Down) in m/s
     * @details Raw velocity estimate from inertial navigation system
     */
    Vector3f vel_ned;
    
    /**
     * @brief Filtered vehicle velocity in NED frame in m/s
     * @details Velocity after notch filtering to remove oscillations, used for control
     */
    Vector3f vel_ned_filtd;

    /**
     * @brief Vehicle position in NED frame relative to home in meters
     * @details 3D position vector: North, East, Down from home/origin
     */
    Vector3f pos_ned;
    
    /**
     * @brief Yaw rate (rotation around vertical axis) in rad/s
     * @details Raw yaw velocity estimate from gyroscope/AHRS
     */
    float vel_yaw;
    
    /**
     * @brief Filtered yaw rate in rad/s
     * @details Yaw velocity after notch filtering, used for control feedback
     */
    float vel_yaw_filtd;
    
    /**
     * @brief Notch filter for horizontal (XY) velocity feedback
     * @details Rejects oscillations at specific frequencies to improve control stability
     */
    NotchFilterVector2f vel_xy_filter;
    
    /**
     * @brief Notch filter for vertical (Z) velocity feedback
     * @details Rejects altitude oscillations to improve vertical control
     */
    NotchFilterFloat vel_z_filter;
    
    /**
     * @brief Notch filter for yaw rate feedback
     * @details Rejects yaw oscillations to improve heading control
     */
    NotchFilterFloat vel_yaw_filter;

    // ========================================================================
    // Inertial Navigation System
    // ========================================================================
    
    /**
     * @brief Inertial navigation system providing position/velocity estimates
     * @details Fuses accelerometer data with AHRS attitude and GPS/barometer corrections
     *          to provide continuous position and velocity estimates in NED frame
     */
    AP_InertialNav inertial_nav;

    // ========================================================================
    // Velocity Control PIDs (Inner Loop)
    // ========================================================================
    
    /**
     * @brief XY velocity controller (horizontal plane)
     * @details 2D PID controlling North and East velocities independently.
     *          Outputs desired acceleration commands to position controller.
     *          Default gains: P=3, I=0.2, D=0, FF=0, IMAX=0.2, FiltHz=3, FiltDHz=3
     * @note Tuning affects horizontal flight characteristics and disturbance rejection
     */
    AC_PID_2D pid_vel_xy{3, 0.2, 0, 0, 0.2, 3, 3};
    
    /**
     * @brief Z velocity controller (vertical axis)
     * @details PID controlling vertical velocity (positive down in NED frame).
     *          Outputs desired vertical acceleration.
     *          Default gains: P=7, I=1.5, D=0, FF=0, IMAX=1, FiltHz=3, FiltDHz=3
     * @note Higher gains than XY due to direct buoyancy control
     */
    AC_PID_Basic pid_vel_z{7, 1.5, 0, 0, 1, 3, 3};
    
    /**
     * @brief Yaw rate controller
     * @details PID controlling yaw angular velocity (rotation rate around vertical axis).
     *          Outputs desired yaw acceleration.
     *          Default gains: P=3, I=0.4, D=0, FF=0, IMAX=0.2, FiltHz=3, FiltDHz=3
     */
    AC_PID_Basic pid_vel_yaw{3, 0.4, 0, 0, 0.2, 3, 3};

    // ========================================================================
    // Position Control PIDs (Outer Loop)
    // ========================================================================
    
    /**
     * @brief XY position controller (horizontal plane)
     * @details 2D PID controlling North and East position errors.
     *          Outputs desired velocity commands to velocity controller.
     *          Default gains: P=1, I=0.05, D=0, FF=0, IMAX=0.1, FiltHz=3, FiltDHz=3
     * @note Cascaded with velocity controller - position → velocity → acceleration
     */
    AC_PID_2D pid_pos_xy{1, 0.05, 0, 0, 0.1, 3, 3};
    
    /**
     * @brief Z position controller (altitude)
     * @details PID controlling altitude error relative to target.
     *          Outputs desired vertical velocity.
     *          Default gains: P=0.7, I=0, D=0, FF=0, IMAX=0, FiltHz=3, FiltDHz=3
     * @note Lower gain than velocity loop for smooth altitude tracking
     */
    AC_PID_Basic pid_pos_z{0.7, 0, 0, 0, 0, 3, 3};
    
    /**
     * @brief Yaw position controller (heading hold)
     * @details PID controlling heading error relative to target yaw angle.
     *          Outputs desired yaw rate command.
     *          Default gains: P=1.2, I=0.5, D=0, FF=0, IMAX=2, FiltHz=3, FiltE=3, FiltD=3
     * @note Includes integral term for steady-state heading accuracy
     */
    AC_PID pid_pos_yaw{1.2, 0.5, 0, 0, 2, 3, 3, 3};

    // ========================================================================
    // System Timers
    // ========================================================================
    
    /**
     * @brief Time in milliseconds when vehicle was armed
     * @details Zero when disarmed. Used to track flight time and enforce time-based limits.
     */
    uint32_t arm_time_ms;

    /**
     * @brief Time in milliseconds of last valid RC input reception
     * @details Used for RC failsafe detection - triggers failsafe if too much time elapses
     *          since last valid RC frame
     */
    uint32_t last_radio_update_ms;

    // ========================================================================
    // Parameter System
    // ========================================================================
    
    /**
     * @brief Parameter loading and management system
     * @details Handles parameter storage/retrieval from non-volatile memory and
     *          manages parameter table registration
     */
    AP_Param param_loader;

    /**
     * @brief Standby mode active flag
     * @details True when vehicle is in standby (reduced power/activity) state
     */
    bool standby_active;

    // ========================================================================
    // Scheduler and Logging Tables
    // ========================================================================
    
    /**
     * @brief Scheduler task table defining periodic execution
     * @details Array of tasks with execution rates, function pointers, and timing budgets.
     *          Defines the real-time loop structure for control, sensor reading, and logging.
     * @see Blimp.cpp for task definitions
     */
    static const AP_Scheduler::Task scheduler_tasks[];
    
    /**
     * @brief Parameter information table for EEPROM storage
     * @details Defines all parameters, their types, storage locations, and metadata
     * @see Parameters.cpp
     */
    static const AP_Param::Info var_info[];
    
    /**
     * @brief Log message structure definitions
     * @details Defines binary log message formats for data logging system
     * @see Log.cpp
     */
    static const struct LogStructure log_structure[];

    // ========================================================================
    // Failsafe Action Definitions
    // ========================================================================
    
    /**
     * @enum Failsafe_Action
     * @brief Actions the vehicle can take when failsafe is triggered
     * 
     * @details Defines escalating levels of failsafe response:
     *          - None: Continue current operation
     *          - Land: Initiate controlled landing
     *          - Terminate: Immediate motor shutdown (most severe)
     * 
     * @warning Terminate action stops all motors immediately - use only as last resort
     */
    enum Failsafe_Action {
        Failsafe_Action_None           = 0,  ///< No action - continue current mode
        Failsafe_Action_Land           = 1,  ///< Switch to Land mode for controlled descent
        Failsafe_Action_Terminate      = 5   ///< Immediate motor termination - emergency only
    };

    /**
     * @enum FailsafeOption
     * @brief Configurable failsafe behavior options (bitfield)
     * 
     * @details Allows customization of when failsafes trigger based on current mode:
     *          - RC_CONTINUE_IF_AUTO: Don't trigger RC failsafe in Auto modes
     *          - GCS_CONTINUE_IF_AUTO: Don't trigger GCS failsafe in Auto modes
     *          - RC_CONTINUE_IF_GUIDED: Don't trigger RC failsafe in Guided mode
     *          - CONTINUE_IF_LANDING: Don't trigger failsafe during landing
     *          - GCS_CONTINUE_IF_PILOT_CONTROL: Don't trigger GCS failsafe with pilot input
     *          - RELEASE_GRIPPER: Release gripper on failsafe
     * 
     * @note Options can be combined using bitwise OR
     */
    enum class FailsafeOption {
        RC_CONTINUE_IF_AUTO             = (1<<0),   ///< Bit 0: Continue if in Auto mode (RC failsafe)
        GCS_CONTINUE_IF_AUTO            = (1<<1),   ///< Bit 1: Continue if in Auto mode (GCS failsafe)
        RC_CONTINUE_IF_GUIDED           = (1<<2),   ///< Bit 2: Continue if in Guided mode (RC failsafe)
        CONTINUE_IF_LANDING             = (1<<3),   ///< Bit 3: Continue if currently landing
        GCS_CONTINUE_IF_PILOT_CONTROL   = (1<<4),   ///< Bit 4: Continue if pilot has control (GCS failsafe)
        RELEASE_GRIPPER                 = (1<<5),   ///< Bit 5: Release gripper on any failsafe
    };

    /**
     * @brief Failsafe action priority ordering (most severe to least severe)
     * @details Array defining priority order for simultaneous failsafes.
     *          Terminate has highest priority (most severe), then Land, then None.
     *          Sentinel value -1 marks end of array.
     * @warning Do not modify without updating FAILSAFE_LAND_PRIORITY and static_asserts
     */
    static constexpr int8_t _failsafe_priorities[] = {
        Failsafe_Action_Terminate,  ///< Priority 0: Highest priority - immediate termination
        Failsafe_Action_Land,       ///< Priority 1: Second priority - controlled landing
        Failsafe_Action_None,       ///< Priority 2: Lowest priority - no action
        -1                          ///< Sentinel value marking end of array
    };

#define FAILSAFE_LAND_PRIORITY 1
    static_assert(_failsafe_priorities[FAILSAFE_LAND_PRIORITY] == Failsafe_Action_Land,
                  "FAILSAFE_LAND_PRIORITY must match the entry in _failsafe_priorities");
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // ========================================================================
    // State Management Methods (AP_State.cpp)
    // ========================================================================
    
    /**
     * @brief Set auto-armed state flag
     * @param[in] b New auto-armed state (true=auto armed, false=not auto armed)
     * @details Controls whether automatic missions can begin. Typically requires
     *          throttle to be raised after arming before auto missions proceed.
     */
    void set_auto_armed(bool b);
    
    /**
     * @brief Set radio failsafe state
     * @param[in] b Radio failsafe active state (true=failsafe active)
     * @details Activates or clears RC link loss failsafe. Triggers failsafe actions
     *          when RC signal is lost for configured timeout period.
     * @warning Safety-critical - triggers protective landing if RC link lost
     */
    void set_failsafe_radio(bool b);
    
    /**
     * @brief Set GCS failsafe state
     * @param[in] b GCS failsafe active state (true=failsafe active)
     * @details Activates or clears ground station telemetry link loss failsafe.
     * @note Behavior depends on FS_GCS_ENABLE parameter configuration
     */
    void set_failsafe_gcs(bool b);

    // ========================================================================
    // Main Loop and Scheduler Methods (Blimp.cpp)
    // ========================================================================
    
    /**
     * @brief Get scheduler task table for AP_Scheduler
     * @param[out] tasks Reference to task array pointer
     * @param[out] task_count Number of tasks in array
     * @param[out] log_bit Logging bitmask for scheduler performance logging
     * @details Provides task table to scheduler system for periodic task execution
     * @note Called during initialization to set up real-time task scheduling
     */
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    
    /**
     * @brief RC input processing loop
     * @details Reads and processes RC receiver input, updates control channels,
     *          checks for mode changes from RC switches
     * @note Called at RC frame rate (typically 50-100Hz)
     */
    void rc_loop();
    
    /**
     * @brief Throttle input processing and arming logic
     * @details Processes throttle channel, updates arming delays, handles
     *          throttle-based mode transitions
     */
    void throttle_loop();
    
    /**
     * @brief Update battery monitoring and compass
     * @details Reads battery sensors and compass, updates failsafe states
     * @note Typically called at 10Hz
     */
    void update_batt_compass(void);
    
    /**
     * @brief High-rate data logging
     * @details Logs time-critical data at main loop rate for detailed analysis
     * @note Called at full loop rate (typically 100Hz)
     */
    void full_rate_logging();
    
    /**
     * @brief 10Hz logging loop
     * @details Logs medium-rate data including attitude, position, PIDs
     * @note Called at 10Hz
     */
    void ten_hz_logging_loop();
    
    /**
     * @brief 25Hz logging loop
     * @details Logs control loop data at 25Hz for moderate-detail analysis
     * @note Called at 25Hz
     */
    void twentyfive_hz_logging();
    
    /**
     * @brief 3Hz periodic tasks
     * @details Executes low-rate tasks like home distance updates
     * @note Called at 3Hz
     */
    void three_hz_loop();
    
    /**
     * @brief 1Hz periodic tasks
     * @details Executes slow periodic tasks like health monitoring
     * @note Called at 1Hz
     */
    void one_hz_loop();
    
    /**
     * @brief Read AHRS attitude and heading data
     * @details Updates vehicle attitude (roll/pitch/yaw) from AHRS system
     * @note Called at main loop rate for current attitude estimates
     */
    void read_AHRS(void);
    
    /**
     * @brief Update altitude estimates
     * @details Fuses barometer and other altitude sources for current altitude estimate
     */
    void update_altitude();
    
    /**
     * @brief Rotate vector from NED frame to body frame
     * @param[in,out] vec 2D vector (North-East) to rotate to body frame (Forward-Right)
     * @details Transforms horizontal vectors from earth frame to body frame using
     *          current vehicle yaw angle. Used for converting desired earth-frame
     *          velocities to body-frame fin commands.
     * @note Input NED (North-East), output body frame (Forward-Right)
     */
    void rotate_NE_to_BF(Vector2f &vec);
    
    /**
     * @brief Rotate vector from body frame to NED frame
     * @param[in,out] vec 2D vector (Forward-Right) to rotate to NED frame (North-East)
     * @details Transforms horizontal vectors from body frame to earth frame using
     *          current vehicle yaw angle. Used for converting body-frame sensor
     *          readings to earth-frame navigation.
     * @note Input body frame (Forward-Right), output NED (North-East)
     */
    void rotate_BF_to_NE(Vector2f &vec);

    // ========================================================================
    // Home Position and Command Methods (commands.cpp)
    // ========================================================================
    
    /**
     * @brief Update home position from EKF estimate
     * @details Sets home position using current EKF position estimate.
     *          Used for emergency home position updates during flight.
     */
    void update_home_from_EKF();
    
    /**
     * @brief Set home to current location while in flight
     * @details Updates home position to current location during flight operation.
     *          Used for dynamic home position updates in certain flight scenarios.
     * @warning Only call when EKF has good quality position estimate
     */
    void set_home_to_current_location_inflight();
    
    /**
     * @brief Set home position to current vehicle location
     * @param[in] lock If true, lock home position from further auto-updates
     * @return true if home position set successfully, false if failed
     * @details Overrides base vehicle method to set home at current position
     * @warning Requires valid GPS fix and EKF initialization
     */
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Set home position to specified location
     * @param[in] loc Location object with latitude, longitude, altitude for home
     * @param[in] lock If true, lock home position from further auto-updates
     * @return true if home position set successfully, false if failed
     * @details Overrides base vehicle method to set home at arbitrary position
     * @note Altitude should be AMSL (Above Mean Sea Level)
     */
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;

    // ========================================================================
    // EKF Health Monitoring Methods (ekf_check.cpp)
    // ========================================================================
    
    /**
     * @brief Check EKF health and variance
     * @details Monitors EKF state estimation quality, checks innovation variances,
     *          and triggers EKF failsafe if quality degrades below thresholds
     * @note Called periodically at 10Hz
     * @warning Critical navigation safety check
     */
    void ekf_check();
    
    /**
     * @brief Check if EKF variance exceeds failsafe threshold
     * @return true if EKF variance over threshold (poor state estimate quality)
     * @details Tests EKF innovation variances against configured limits for failsafe triggering
     */
    bool ekf_over_threshold();
    
    /**
     * @brief Handle EKF failsafe activation event
     * @details Called when EKF state estimation quality falls below acceptable limits.
     *          Triggers appropriate failsafe action based on configuration.
     * @warning Safety-critical - may trigger emergency landing
     */
    void failsafe_ekf_event();
    
    /**
     * @brief Handle EKF failsafe clearance event
     * @details Called when EKF state estimation quality recovers to acceptable levels.
     *          Clears EKF failsafe state and may restore previous flight mode.
     */
    void failsafe_ekf_off_event(void);
    
    /**
     * @brief Check for EKF yaw reset and handle
     * @details Monitors for EKF yaw angle resets (discontinuities) and updates
     *          control system targets to prevent control transients
     * @note EKF resets can occur during initialization or when resolving ambiguities
     */
    void check_ekf_reset();
    
    /**
     * @brief Check vibration levels affecting sensor accuracy
     * @details Monitors IMU vibration levels and sets warning flags if excessive
     *          vibrations detected that could affect control performance
     */
    void check_vibration();

    // ========================================================================
    // Failsafe Event Handlers (events.cpp)
    // ========================================================================
    
    /**
     * @brief Check if specific failsafe option is enabled
     * @param[in] opt Failsafe option to check (bitfield)
     * @return true if specified failsafe option is enabled in configuration
     * @details Tests failsafe option parameters to determine behavior customization
     */
    bool failsafe_option(FailsafeOption opt) const;
    
    /**
     * @brief Handle radio failsafe activation event
     * @details Called when RC link is lost. Initiates failsafe action based on
     *          current mode and FS_OPTIONS configuration.
     * @warning Safety-critical - triggers protective landing or RTL
     */
    void failsafe_radio_on_event();
    
    /**
     * @brief Handle radio failsafe clearance event
     * @details Called when RC link is restored. Clears radio failsafe state
     *          and may restore pilot control depending on mode.
     */
    void failsafe_radio_off_event();
    
    /**
     * @brief Handle battery failsafe event
     * @param[in] type_str String describing failsafe type ("low voltage", "capacity")
     * @param[in] action Failsafe action code to execute
     * @details Called when battery reaches critical levels. Executes configured
     *          failsafe action (typically Land mode).
     * @warning Safety-critical - prevents complete power loss in flight
     */
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    
    /**
     * @brief Check GCS telemetry link health and trigger failsafe if needed
     * @details Monitors time since last valid GCS heartbeat. Triggers GCS failsafe
     *          if configured timeout exceeded without telemetry.
     * @note Behavior depends on FS_GCS_ENABLE and current flight mode
     */
    void failsafe_gcs_check();
    
    /**
     * @brief Check if vehicle should disarm on current failsafe condition
     * @return true if vehicle should disarm, false if should remain armed
     * @details Determines if current failsafe state warrants automatic disarming
     * @note Used to prevent prolonged operation in failed state
     */
    bool should_disarm_on_failsafe();
    
    /**
     * @brief Execute specified failsafe action
     * @param[in] action Failsafe_Action to execute (None, Land, Terminate)
     * @param[in] reason ModeReason explaining why action triggered
     * @details Central dispatcher for failsafe actions. Switches modes or terminates
     *          motors based on action severity.
     * @warning Terminate action stops motors immediately - use only as last resort
     */
    void do_failsafe_action(Failsafe_Action action, ModeReason reason);
    
    /**
     * @brief Check for GPS glitches affecting navigation
     * @details Monitors GPS health metrics and sets glitch flags when GPS
     *          data quality degrades significantly
     */
    void gpsglitch_check();

    // ========================================================================
    // Failsafe Enable/Disable Methods (failsafe.cpp)
    // ========================================================================
    
    /**
     * @brief Enable failsafe monitoring systems
     * @details Activates all failsafe checks including radio, GCS, battery, and EKF.
     *          Typically called during arming sequence.
     * @note Failsafe systems should be enabled whenever vehicle is armed
     */
    void failsafe_enable();
    
    /**
     * @brief Disable failsafe monitoring systems
     * @details Deactivates failsafe checks, typically called when disarming.
     *          Prevents nuisance failsafe triggers when vehicle is on ground.
     */
    void failsafe_disable();

    // ========================================================================
    // Geofence Methods (fence.cpp)
    // ========================================================================
    
    /**
     * @brief Check geofence boundaries and trigger breach actions
     * @details Monitors vehicle position against configured geofence boundaries
     *          (cylindrical, polygon, altitude). Executes breach actions if fence violated.
     * @note Typically called at 10Hz when fence enabled
     * @warning Fence breach may trigger automatic landing or mode changes
     */
    void fence_check();

    // ========================================================================
    // Inertial Sensor Reading Methods (inertia.cpp)
    // ========================================================================
    
    /**
     * @brief Read inertial navigation system data
     * @details Updates position and velocity estimates from inertial navigation,
     *          reads accelerometer and gyro data, updates NED frame state vectors
     * @note Called at main loop rate for current navigation state
     */
    void read_inertia();

    // ========================================================================
    // Landing Detection Methods (landing_detector.cpp)
    // ========================================================================
    
    /**
     * @brief Update landing and crash detection systems
     * @details Runs both landing detector and crash detector logic to identify
     *          ground contact or abnormal flight conditions
     * @note Called at main loop rate during flight
     */
    void update_land_and_crash_detectors();
    
    /**
     * @brief Update landing detection logic
     * @details Analyzes throttle, descent rate, and accelerometer data to determine
     *          if vehicle has landed. Sets land_complete and land_complete_maybe flags.
     * @note More sensitive detection sets land_complete_maybe first
     */
    void update_land_detector();

    // ========================================================================
    // Landing Gear Methods (landing_gear.cpp)
    // ========================================================================
    
    /**
     * @brief Update landing gear state and position
     * @details Controls landing gear deployment/retraction based on flight phase
     *          and altitude. Updates landing gear actuator outputs.
     * @note Only relevant if landing gear installed and configured
     */
    void landinggear_update();

#if HAL_LOGGING_ENABLED
    // ========================================================================
    // Logging Configuration Methods (AP_Vehicle overrides)
    // ========================================================================
    
    /**
     * @brief Get logging bitmask parameter
     * @return Reference to LOG_BITMASK parameter controlling which log types are enabled
     * @details Overrides AP_Vehicle method to provide access to logging configuration
     */
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    
    /**
     * @brief Get log structure definitions array
     * @return Pointer to array of LogStructure definitions for binary logging
     * @details Overrides AP_Vehicle method to provide Blimp-specific log formats
     */
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    
    /**
     * @brief Get number of log structure definitions
     * @return Count of log message types defined in log_structure array
     * @details Overrides AP_Vehicle method to provide structure count for logging system
     */
    uint8_t get_num_log_structures() const override;

    // ========================================================================
    // Data Logging Methods (Log.cpp)
    // ========================================================================
    
    /**
     * @brief Log vehicle attitude data
     * @details Writes roll, pitch, yaw, and rates to binary log for attitude analysis
     * @note Called at high rate (typically 25Hz) for detailed attitude tracking
     */
    void Log_Write_Attitude();
    
    /**
     * @brief Log PID controller state and outputs
     * @details Writes all PID controller values (desired, achieved, P/I/D terms) to log
     *          for tuning analysis and performance evaluation
     * @note Essential for PID tuning and control loop debugging
     */
    void Log_Write_PIDs();
    
    /**
     * @brief Log EKF position estimate data
     * @details Writes EKF position, velocity, and innovation data for navigation analysis
     * @note Used for EKF performance evaluation and problem diagnosis
     */
    void Log_Write_EKF_POS();
    
    /**
     * @brief Log integer data value with ID
     * @param[in] id LogDataID identifying the data type being logged
     * @param[in] value 32-bit signed integer value to log
     * @details Generic logging function for arbitrary signed integer data
     */
    void Log_Write_Data(LogDataID id, int32_t value);
    
    /**
     * @brief Log unsigned integer data value with ID
     * @param[in] id LogDataID identifying the data type being logged
     * @param[in] value 32-bit unsigned integer value to log
     * @details Generic logging function for arbitrary unsigned integer data
     */
    void Log_Write_Data(LogDataID id, uint32_t value);
    
    /**
     * @brief Log short integer data value with ID
     * @param[in] id LogDataID identifying the data type being logged
     * @param[in] value 16-bit signed integer value to log
     * @details Generic logging function for arbitrary signed short data
     */
    void Log_Write_Data(LogDataID id, int16_t value);
    
    /**
     * @brief Log unsigned short integer data value with ID
     * @param[in] id LogDataID identifying the data type being logged
     * @param[in] value 16-bit unsigned integer value to log
     * @details Generic logging function for arbitrary unsigned short data
     */
    void Log_Write_Data(LogDataID id, uint16_t value);
    
    /**
     * @brief Log floating point data value with ID
     * @param[in] id LogDataID identifying the data type being logged
     * @param[in] value Float value to log
     * @details Generic logging function for arbitrary floating point data
     */
    void Log_Write_Data(LogDataID id, float value);
    
    /**
     * @brief Log parameter tuning data
     * @param[in] param Parameter index being tuned
     * @param[in] tuning_val Current tuning value
     * @param[in] tune_min Minimum allowed tuning value
     * @param[in] tune_max Maximum allowed tuning value
     * @details Logs parameter adjustment data for auto-tuning and manual tuning analysis
     * @note Used during autotune operations
     */
    void Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max);

    /**
     * @brief Log vehicle startup messages
     * @details Writes initial vehicle state, configuration, and version information
     *          to log at startup. Essential for log analysis context.
     * @note Called once during initialization
     */
    void Log_Write_Vehicle_Startup_Messages();
    
    /**
     * @brief Log fin input values
     * @param[in] right Right fin input value
     * @param[in] front Front fin input value
     * @param[in] down Down fin input value (vertical thrust)
     * @param[in] yaw Yaw fin input value
     * @details Logs commanded fin inputs for lighter-than-air control analysis.
     *          Essential for understanding fin control commands and tuning.
     * @note Specific to Blimp fin control system
     */
    void Write_FINI(float right, float front, float down, float yaw);
    
    /**
     * @brief Log fin output values
     * @param[in] amp Array of fin amplitude values
     * @param[in] off Array of fin offset values
     * @details Logs actual fin outputs including oscillation amplitude and offsets.
     *          Used for analyzing fin actuation and control response.
     * @note Specific to Blimp oscillating fin control system
     */
    void Write_FINO(float *amp, float *off);
#endif

    // ========================================================================
    // Flight Mode Management Methods (mode.cpp)
    // ========================================================================
    
    /**
     * @brief Set flight mode by mode number
     * @param[in] mode Mode::Number enum value for desired flight mode
     * @param[in] reason ModeReason explaining why mode change requested
     * @return true if mode change successful, false if mode change rejected
     * @details Attempts to change flight mode after checking mode availability,
     *          arming state, and mode-specific entry conditions
     * @note Mode change may be rejected if vehicle not ready or mode unavailable
     */
    bool set_mode(Mode::Number mode, ModeReason reason);
    
    /**
     * @brief Set flight mode by integer mode number (AP_Vehicle override)
     * @param[in] new_mode Integer mode number (typically from MAVLink)
     * @param[in] reason ModeReason explaining why mode change requested
     * @return true if mode change successful, false if mode change rejected
     * @details Overrides AP_Vehicle method for mode changes from GCS or RC
     */
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override;
    
    /**
     * @brief Get current flight mode number (AP_Vehicle override)
     * @return Integer representation of current flight mode
     * @details Overrides AP_Vehicle method to provide current mode to base system
     */
    uint8_t get_mode() const override
    {
        return (uint8_t)control_mode;
    }
    
    /**
     * @brief Update flight mode control logic
     * @details Calls current mode's update method to execute mode-specific control.
     *          This is the main mode dispatch function called each control loop iteration.
     * @note Called at main loop rate (typically 100Hz)
     */
    void update_flight_mode();
    
    /**
     * @brief Send flight mode change notifications
     * @details Sends MAVLink mode change message to GCS, updates LED patterns,
     *          and triggers other mode change notifications
     * @note Called after successful mode change
     */
    void notify_flight_mode();

    // ========================================================================
    // Landing Mode Methods (mode_land.cpp)
    // ========================================================================
    
    /**
     * @brief Force vehicle into Land mode due to failsafe
     * @param[in] reason ModeReason explaining failsafe trigger
     * @details Emergency mode change to Land mode, bypassing normal mode checks.
     *          Used when failsafe requires immediate landing.
     * @warning Safety-critical - forces landing regardless of pilot input
     */
    void set_mode_land_failsafe(ModeReason reason);
    
    /**
     * @brief Check if vehicle is landing with GPS position control
     * @return true if landing under GPS guidance, false if landing without GPS
     * @details Determines if GPS is available and healthy for controlled landing descent
     */
    bool landing_with_GPS();

    // ========================================================================
    // Motor Control Methods (motors.cpp)
    // ========================================================================
    
    /**
     * @brief Check arming conditions and update motor arming state
     * @details Performs pre-arm and arming safety checks, enables motors if
     *          conditions satisfied. Checks battery, sensors, position estimate.
     * @warning Safety-critical - prevents arming with unsafe conditions
     */
    void arm_motors_check();
    
    /**
     * @brief Output motor/fin commands to hardware
     * @details Sends calculated motor and fin outputs to actuators via AP_Motors
     *          and Fins objects. Applies safety limits and failsafe constraints.
     * @note Called at main loop rate after control calculations complete
     */
    void motors_output();

    // ========================================================================
    // Parameter Management Methods (Parameters.cpp)
    // ========================================================================
    
    /**
     * @brief Load parameters from EEPROM (AP_Vehicle override)
     * @details Loads all vehicle parameters from persistent storage, performs
     *          parameter conversions for version updates, validates ranges
     * @note Called during initialization before parameter system used
     */
    void load_parameters(void) override;
    
    /**
     * @brief Convert legacy PID parameters to current format
     * @details Handles parameter format changes between firmware versions,
     *          converting old parameter names/scales to new format
     * @note Called during parameter load if old parameter format detected
     */
    void convert_pid_parameters(void);
    
    /**
     * @brief Convert legacy landing gear parameters to current format
     * @details Handles landing gear parameter format changes between versions
     */
    void convert_lgr_parameters(void);
    
    /**
     * @brief Convert legacy failsafe option parameters to current format
     * @details Handles failsafe option parameter format changes, converting
     *          old bitfield arrangements to new format
     */
    void convert_fs_options_params(void);

    // ========================================================================
    // RC Radio Input Methods (radio.cpp)
    // ========================================================================
    
    /**
     * @brief Set default RC input dead zones
     * @details Configures default dead zone values for roll, pitch, yaw, and
     *          throttle RC inputs to filter stick noise
     */
    void default_dead_zones();
    
    /**
     * @brief Initialize RC input system
     * @details Sets up RC receiver interface, configures channel mapping,
     *          initializes dead zones and trim values
     * @note Called during vehicle initialization
     */
    void init_rc_in();
    
    /**
     * @brief Initialize RC output system
     * @details Configures RC output channels for motor and servo control,
     *          sets initial output ranges and safety states
     * @note Called during vehicle initialization
     */
    void init_rc_out();
    
    /**
     * @brief Enable motor output after arming
     * @details Transitions motors from disarmed to armed state, enabling
     *          throttle control. Applies safety ramp-up procedures.
     * @warning Only call after all arming checks passed
     */
    void enable_motor_output();
    
    /**
     * @brief Read RC receiver inputs
     * @details Reads all RC channels, applies dead zones, detects mode switches,
     *          updates pilot input values for control loops
     * @note Called at RC frame rate (typically 50-100Hz)
     */
    void read_radio();
    
    /**
     * @brief Process throttle input and check for failsafe
     * @param[in] throttle_pwm Raw throttle channel PWM value in microseconds
     * @details Processes throttle channel, detects RC signal loss based on
     *          PWM value, triggers radio failsafe if signal invalid
     * @note RC failsafe typically triggers if PWM < 900 or > 2100 μs
     */
    void set_throttle_and_failsafe(uint16_t throttle_pwm);
    
    /**
     * @brief Set throttle-zero flag based on throttle input
     * @param[in] throttle_control Throttle control value (scaled)
     * @details Sets flag indicating throttle is at zero position for landing
     *          detection and mode transition logic
     */
    void set_throttle_zero_flag(int16_t throttle_control);

    // ========================================================================
    // Sensor Reading Methods (sensors.cpp)
    // ========================================================================
    
    /**
     * @brief Read barometer sensor
     * @details Updates barometric pressure and temperature readings, calculates
     *          pressure altitude for altitude estimation
     * @note Called at 50Hz typically
     */
    void read_barometer(void);
    
    /**
     * @brief Initialize rangefinder sensors
     * @details Detects and initializes downward and upward facing rangefinders,
     *          configures rangefinder parameters
     * @note Called during vehicle initialization
     */
    void init_rangefinder(void);
    
    /**
     * @brief Read rangefinder sensors
     * @details Updates rangefinder distance measurements, checks sensor health,
     *          provides terrain distance data to navigation system
     * @note Typically called at 20Hz
     */
    void read_rangefinder(void);
    
    /**
     * @brief Check if downward rangefinder data is valid
     * @return true if downward rangefinder providing valid altitude data
     * @details Checks rangefinder health, range validity, and data freshness
     */
    bool rangefinder_alt_ok();
    
    /**
     * @brief Check if upward rangefinder data is valid
     * @return true if upward rangefinder providing valid data
     * @details Checks upward rangefinder health for ceiling detection
     * @note Used for indoor flight and obstacle avoidance
     */
    bool rangefinder_up_ok();

    // ========================================================================
    // System Initialization and Status Methods (system.cpp)
    // ========================================================================
    
    /**
     * @brief Initialize ArduPilot vehicle system (AP_Vehicle override)
     * @details Main initialization function called at startup. Initializes all
     *          subsystems including sensors, navigation, control, communication.
     * @note Called once during boot sequence
     */
    void init_ardupilot() override;
    
    /**
     * @brief Perform INS ground startup calibration
     * @details Calibrates gyros and accelerometers while vehicle stationary on
     *          ground. Establishes initial sensor bias estimates.
     * @warning Vehicle must be completely stationary during this process
     * @note Typically takes 1-3 seconds
     */
    void startup_INS_ground();
    
    /**
     * @brief Check if position estimate is valid for navigation
     * @return true if position estimate good enough for position control
     * @details Checks EKF status and GPS fix quality to determine if position
     *          control modes can be safely used
     */
    bool position_ok() const;
    
    /**
     * @brief Check if EKF has absolute (GPS) position estimate
     * @return true if EKF is using GPS for absolute position
     * @details Indicates EKF has converged using GPS measurements
     */
    bool ekf_has_absolute_position() const;
    
    /**
     * @brief Check if EKF has relative (optical flow/visual) position
     * @return true if EKF is using non-GPS position sources
     * @details Indicates EKF has position estimate from optical flow, visual
     *          odometry, or other relative positioning sensors
     */
    bool ekf_has_relative_position() const;
    
    /**
     * @brief Check if EKF altitude estimate is valid
     * @return true if EKF altitude estimate is healthy
     * @details Checks EKF altitude innovation and variance for validity
     */
    bool ekf_alt_ok() const;
    
    /**
     * @brief Update auto-armed state based on throttle position
     * @details Checks if throttle has been raised above threshold after arming
     *          to enable auto modes and mission execution
     * @note Prevents unintended auto mission start on arming
     */
    void update_auto_armed();
    
    /**
     * @brief Check if specified log message type should be logged
     * @param[in] mask Bitmask for log message type to check
     * @return true if this log type is enabled in LOG_BITMASK parameter
     * @details Tests LOG_BITMASK parameter to determine logging enablement
     */
    bool should_log(uint32_t mask);
    
    /**
     * @brief Get MAVLink vehicle type identifier
     * @return MAV_TYPE enum value for this vehicle (MAV_TYPE_BLIMP)
     * @details Returns MAVLink vehicle type for proper GCS identification
     */
    MAV_TYPE get_frame_mav_type();
    
    /**
     * @brief Get vehicle frame type string
     * @return String describing vehicle frame type ("Blimp")
     * @details Returns human-readable frame type for display and logging
     */
    const char* get_frame_string();
    
    /**
     * @brief Allocate and initialize motors/fins objects
     * @details Creates motors and fins objects based on frame configuration,
     *          allocates required memory for control objects
     * @note Called during initialization
     */
    void allocate_motors(void);

    // ========================================================================
    // Flight Mode Objects
    // ========================================================================
    
    /**
     * @brief Pointer to current active flight mode
     * @details Points to the currently executing mode object. Updated during
     *          mode transitions to the new active mode.
     */
    Mode *flightmode;
    
    /**
     * @brief Manual flight mode instance
     * @details Direct pilot control with no stabilization assistance
     */
    ModeManual mode_manual;
    
    /**
     * @brief Land flight mode instance
     * @details Autonomous landing mode with controlled descent
     */
    ModeLand mode_land;
    
    /**
     * @brief Velocity flight mode instance
     * @details Velocity-controlled flight mode with position hold
     */
    ModeVelocity mode_velocity;
    
    /**
     * @brief Loiter flight mode instance
     * @details Position hold mode maintaining current location
     */
    ModeLoiter mode_loiter;
    
    /**
     * @brief Return to Launch (RTL) flight mode instance
     * @details Autonomous return to home location followed by landing
     */
    ModeRTL mode_rtl;

    // ========================================================================
    // Flight Mode Helper Methods (mode.cpp)
    // ========================================================================
    
    /**
     * @brief Get mode object pointer from mode number
     * @param[in] mode Mode::Number enum value
     * @return Pointer to Mode object for specified mode, or nullptr if invalid
     * @details Translates mode number to corresponding mode object instance
     */
    Mode *mode_from_mode_num(const Mode::Number mode);
    
    /**
     * @brief Handle mode exit and entry during mode transition
     * @param[in,out] old_flightmode Reference to pointer for mode being exited
     * @param[in,out] new_flightmode Reference to pointer for mode being entered
     * @details Calls exit() on old mode and enter() on new mode, handling
     *          state transitions and target resets
     * @note Called during mode changes to ensure clean transitions
     */
    void exit_mode(Mode *&old_flightmode, Mode *&new_flightmode);

public:
    // ========================================================================
    // Public Failsafe Methods (failsafe.cpp)
    // ========================================================================
    
    /**
     * @brief Main failsafe monitoring and response function
     * @details Checks all failsafe conditions (radio, GCS, battery, EKF, GPS)
     *          and triggers appropriate failsafe actions. Central failsafe dispatcher.
     * @note Called at 10Hz from scheduler
     * @warning Safety-critical - coordinates all failsafe systems
     */
    void failsafe_check();
};

/**
 * @brief Global Blimp vehicle instance
 * @details Singleton instance of the Blimp class representing the vehicle.
 *          Accessible throughout the codebase for vehicle state and control.
 * @note Declared extern for access from all translation units
 */
extern Blimp blimp;

using AP_HAL::millis;
using AP_HAL::micros;
