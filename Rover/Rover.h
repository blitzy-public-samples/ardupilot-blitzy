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
 * @file Rover.h
 * @brief Main Rover class declaration defining the vehicle singleton with all subsystems, modes, and global state
 * 
 * @details This file declares the Rover class which serves as the main vehicle controller
 *          for ground vehicles including wheeled rovers, boats, sailboats, and balance bots.
 *          The class integrates all vehicle subsystems (motors, sensors, navigation, communications)
 *          and coordinates the execution of the main scheduler loop and mode-specific behaviors.
 * 
 *          Key architectural components:
 *          - Singleton pattern: Single global 'rover' instance coordinates all systems
 *          - Scheduler integration: scheduler_tasks array defines periodic tasks and execution rates
 *          - Mode system: Multiple mode objects (manual, auto, guided, etc.) with active mode pointer
 *          - Library integration: AHRS, mission management, fence, rally, telemetry, logging
 *          - Parameter groups: g (Parameters) and g2 (ParametersG2) for vehicle configuration
 * 
 *          Supported vehicle types (frame classes):
 *          - Wheeled rovers (differential steering, skid steering, Ackermann steering)
 *          - Boats (differential thrust, vectored thrust)
 *          - Balance bots (inverted pendulum control)
 *          - Sailboats (sail and rudder control with apparent wind)
 * 
 * @note The global 'rover' instance (declared at end of file) provides access to all vehicle systems
 * @see Mode for flight mode base class and mode-specific implementations
 * @see Parameters.h for vehicle configuration parameter groups
 * 
 * Source: Rover/Rover.h
 */
#pragma once

#include <cmath>
#include <stdarg.h>
#include <stdint.h>

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>          // Battery monitor library
#include <AP_Camera/AP_Camera.h>                    // Camera triggering
#include <AP_Mount/AP_Mount.h>                      // Camera/Antenna mount
#include <AP_Param/AP_Param.h>
#include <AP_RangeFinder/AP_RangeFinder.h>          // Range finder library
#include <AP_RCMapper/AP_RCMapper.h>                // RC input mapping library
#include <AP_RPM/AP_RPM.h>                          // RPM input library
#include <AP_Scheduler/AP_Scheduler.h>              // main loop scheduler
#include <AP_Vehicle/AP_Vehicle.h>                  // needed for AHRS build
#include <AP_WheelEncoder/AP_WheelEncoder.h>
#include <AP_WheelEncoder/AP_WheelRateControl.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_OSD/AP_OSD.h>
#include <AR_Motors/AP_MotorsUGV.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Mission/AP_Mission_ChangeDetector.h>
#include <AR_WPNav/AR_WPNav_OA.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AC_PrecLand/AC_PrecLand_config.h>
#include <AP_Follow/AP_Follow_config.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Rover.h"
#endif

// Configuration
#include "defines.h"
#include "config.h"

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

// Local modules
#include "AP_Arming_Rover.h"
#include "sailboat.h"
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
#include "afs_rover.h"
#endif
#include "Parameters.h"
#include "GCS_MAVLink_Rover.h"
#include "GCS_Rover.h"
#include "AP_Rally.h"
#if AC_PRECLAND_ENABLED
#include <AC_PrecLand/AC_PrecLand.h>
#endif
#include "RC_Channel_Rover.h"                  // RC Channel Library

#include "mode.h"

/**
 * @class Rover
 * @brief Main vehicle class for ground vehicles (rovers, boats, balance bots, sailboats)
 * 
 * @details The Rover class is the central controller for ground vehicle operations, inheriting
 *          from AP_Vehicle and implementing the vehicle-specific logic for:
 * 
 *          **Scheduler and Main Loop:**
 *          - Defines scheduler_tasks array with task priorities and execution rates
 *          - Integrates with AP_Scheduler for deterministic task execution
 *          - Typical main loop rate: 50Hz for rovers, 400Hz for balance bots
 *          - Tasks include: fast_loop, update_current_mode, read_radio, update_GPS, etc.
 * 
 *          **Mode System:**
 *          - Multiple mode objects (mode_manual, mode_auto, mode_guided, mode_rtl, etc.)
 *          - Active mode tracked via control_mode pointer
 *          - Mode transitions validated through set_mode() with arming checks
 *          - Each mode implements specific control behaviors in Mode base class
 * 
 *          **Library Integration:**
 *          - AP_AHRS: Attitude and heading reference system with EKF integration
 *          - AP_Mission: Mission command execution and waypoint navigation
 *          - GCS_MAVLink: Ground control station communication via MAVLink protocol
 *          - AP_Logger: Binary dataflash logging of vehicle state and sensor data
 *          - AC_Fence: Geofencing with boundary violations and actions
 *          - AP_Rally: Rally point management for return-to-launch alternatives
 *          - AR_Motors (AP_MotorsUGV): Motor control and mixing for various frame types
 *          - AR_WPNav: Waypoint navigation with obstacle avoidance
 * 
 *          **Parameter Groups:**
 *          - g (Parameters): Primary configuration parameters (declared first for AP_Param)
 *          - g2 (ParametersG2): Extended parameters using GroupInfo for scalability
 *          - Parameters stored in EEPROM/flash, accessible via MAVLink parameter protocol
 * 
 *          **Global State:**
 *          - current_loc: Vehicle's current GPS position (Location structure)
 *          - control_mode: Pointer to active mode object determining vehicle behavior
 *          - failsafe: Failsafe state tracking (RC loss, GCS loss, EKF failure, battery)
 *          - have_position: Boolean indicating valid position estimate from AHRS
 *          - initialised: System initialization completion flag
 * 
 *          **Frame Types (g2.frame_type):**
 *          - Undefined/Rover: Standard wheeled rover with configurable steering
 *          - Boat: Differential or vectored thrust for marine vehicles
 *          - BalanceBot: Inverted pendulum with pitch control for stability
 *          - Sailboat: Wind-powered with sail angle and rudder control
 * 
 *          **Steering Configurations:**
 *          - Skid steering: Tank-style differential wheel speeds
 *          - Regular steering: Separate steering servo and throttle
 *          - Omni: Omnidirectional movement with lateral control
 *          - Walking robot: Articulated legs with height control
 * 
 *          **Singleton Pattern:**
 *          The global 'rover' instance declared at end of file provides access to all
 *          vehicle systems. Friend classes (modes, GCS, arming) access private members
 *          directly for performance and tight integration.
 * 
 * @note This is a safety-critical class coordinating flight control, arming, and failsafes
 * @warning Modifications to scheduler tasks, mode transitions, or failsafe logic can affect vehicle safety
 * 
 * @see AP_Vehicle base class for common vehicle functionality
 * @see Mode base class for flight mode implementations
 * @see Parameters for g parameter group structure
 * @see ParametersG2 for g2 parameter group structure
 * 
 * Source: Rover/Rover.h
 */
class Rover : public AP_Vehicle {
public:
    friend class GCS_MAVLINK_Rover;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Rally_Rover;
    friend class AP_Arming_Rover;
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    friend class AP_AdvancedFailsafe_Rover;
#endif
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Rover;
#endif
    friend class GCS_Rover;
    friend class Mode;
    friend class ModeAcro;
    friend class ModeAuto;
    friend class ModeCircle;
    friend class ModeGuided;
    friend class ModeHold;
    friend class ModeLoiter;
    friend class ModeSteering;
    friend class ModeManual;
    friend class ModeRTL;
    friend class ModeSmartRTL;
#if MODE_FOLLOW_ENABLED
    friend class ModeFollow;
#endif
    friend class ModeSimple;
#if MODE_DOCK_ENABLED
    friend class ModeDock;
#endif

    friend class RC_Channel_Rover;
    friend class RC_Channels_Rover;

    friend class Sailboat;

    Rover(void);

private:

    /**
     * @brief Parameter loader (must be first AP_Param variable)
     * 
     * @details Ensures parameter system initialization occurs before other AP_Param
     *          variables are constructed. Critical for proper parameter loading from storage.
     * 
     * @note Declaration order is critical - param_loader MUST be first AP_Param member
     */
    AP_Param param_loader;

    /**
     * @brief Primary parameter group (g)
     * 
     * @details Contains core vehicle configuration parameters accessible via MAVLink
     *          parameter protocol. Parameters stored in EEPROM/flash and persist across reboots.
     *          Includes steering/throttle limits, navigation gains, failsafe settings, etc.
     * 
     * @see Parameters for parameter definitions and default values
     * @see AP_Param for parameter storage system
     */
    Parameters g;
    
    /**
     * @brief Extended parameter group (g2)
     * 
     * @details Second parameter group using GroupInfo for better scalability and organization.
     *          Contains subsystem parameters: motors, RC channels, fence, rally, follow mode, etc.
     *          Allows adding parameters without breaking EEPROM compatibility.
     * 
     * @see ParametersG2 for extended parameter definitions
     */
    ParametersG2 g2;

    /// RC input channel mapping (maps physical RC channels to logical functions)
    RCMapper rcmap;

    /// Primary control channels mapped from RC inputs
    RC_Channel *channel_steer;       ///< Steering control input (typically RC channel 1)
    RC_Channel *channel_throttle;    ///< Throttle control input (typically RC channel 3)
    RC_Channel *channel_lateral;     ///< Lateral movement for omni vehicles
    RC_Channel *channel_roll;        ///< Roll control for balance bots
    RC_Channel *channel_pitch;       ///< Pitch control for balance bots
    RC_Channel *channel_walking_height; ///< Height control for walking robots

    // flight modes convenience array
    AP_Int8 *modes;
    const uint8_t num_modes = 6;

#if AP_RPM_ENABLED
    // AP_RPM Module
    AP_RPM rpm_sensor;
#endif

    // Arming/Disarming management class
    AP_Arming_Rover arming;

    // external control implementation
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Rover external_control;
#endif

#if AP_OPTICALFLOW_ENABLED
    AP_OpticalFlow optflow;
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif
#if AC_PRECLAND_ENABLED
    AC_PrecLand precland;
#endif
    /**
     * @brief Ground Control Station communication interface
     * 
     * @details Handles MAVLink protocol communication with ground control stations,
     *          mission upload/download, parameter access, and telemetry streaming.
     * 
     * @note Access via gcs() accessor method, not directly via _gcs
     */
    GCS_Rover _gcs;  ///< GCS implementation (private, use gcs() accessor)
    
    /// @brief Accessor for GCS interface
    /// @return Reference to GCS_Rover instance for MAVLink communications
    GCS_Rover &gcs() { return _gcs; }

    /// @brief Accessor for RC channel management
    /// @return Reference to RC_Channels_Rover for input processing
    RC_Channels_Rover &rc() { return g2.rc_channels; }

    /**
     * @brief Vehicle's current global position
     * 
     * @details Updated by AHRS at main loop rate from GPS and EKF position estimate.
     *          Used for navigation calculations, distance-to-waypoint, and logging.
     *          Validity indicated by have_position flag.
     * 
     * @note Coordinates in Location structure (lat/lon in degrees*1e7, alt in cm)
     */
    Location current_loc;

    // Camera
#if AP_CAMERA_ENABLED
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

    // Camera/Antenna mount tracking and stabilisation stuff
#if HAL_MOUNT_ENABLED
    AP_Mount camera_mount;
#endif

    /// System initialization complete flag (true after init_ardupilot() completes)
    bool initialised;

    /**
     * @brief Pointer to currently active flight mode object
     * 
     * @details Determines vehicle behavior by pointing to one of the mode objects
     *          (mode_manual, mode_auto, mode_guided, etc.). Updated by set_mode().
     *          Mode's update() method called each iteration of main control loop.
     * 
     * @note All mode objects instantiated at startup; pointer switches between them
     * @see set_mode() for mode transition logic
     * @see Mode base class for mode interface
     */
    Mode *control_mode;

    /**
     * @brief Previous RC mode switch position for edge detection
     * 
     * @details Tracks mode switch position to detect changes. Set to -1 to force
     *          re-reading the switch (useful after parameter changes or failsafe recovery).
     */
    uint8_t oldSwitchPosition;

    /**
     * @brief Failsafe state tracking structure
     * 
     * @details Monitors multiple failsafe conditions and coordinates failsafe actions:
     *          - RC signal loss (no valid RC input for FS_TIMEOUT seconds)
     *          - GCS communication loss (no MAVLink heartbeat for FS_GCS_TIMEOUT)
     *          - EKF failure (navigation estimate divergence or sensor failure)
     *          - Battery failsafe (voltage or capacity below FS_BATT thresholds)
     * 
     *          Failsafe actions configured per-type: None, Hold, RTL, SmartRTL, Terminate
     * 
     * @note Multiple simultaneous failsafes possible; highest priority action taken
     * @warning Failsafe actions override pilot commands for safety
     */
    struct {
        uint8_t bits;               ///< Bit flags of failsafes started (FS_RC=1, FS_GCS=2, FS_BATT=4, etc.)
        uint32_t start_time;        ///< System time (ms) of earliest active failsafe
        uint8_t triggered;          ///< Bit flags of failsafes that have triggered mode change action
        uint32_t last_valid_rc_ms;  ///< System time (ms) of most recent valid RC input from pilot
        bool ekf;                   ///< EKF failsafe active flag
    } failsafe;

    /**
     * @brief Valid position estimate available flag
     * 
     * @details Indicates AHRS/EKF has valid position estimate from GPS and sensors.
     *          Required for autonomous modes (Auto, Guided, RTL, Loiter).
     *          False during GPS initialization or when position estimate quality too low.
     * 
     * @note Checked by arming system and mode entry validation
     */
    bool have_position;

#if AP_RANGEFINDER_ENABLED
    // range finder last update for each instance (used for DPTH logging)
    uint32_t rangefinder_last_reading_ms[RANGEFINDER_MAX_INSTANCES];
#endif

    // Ground speed
    // The amount current ground speed is below min ground speed.  meters per second
    float ground_speed;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Rover::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    // flyforward timer
    uint32_t flyforward_start_ms;

    /**
     * @brief Scheduler task array defining periodic functions and their execution rates
     * 
     * @details This array configures the AP_Scheduler with tasks executed in the main loop.
     *          Each task specifies:
     *          - Function pointer to task handler
     *          - Rate divider relative to main loop frequency (50Hz for rovers, 400Hz for balance bots)
     *          - Maximum expected execution time in microseconds
     * 
     *          Key tasks and typical rates:
     *          - fast_loop: Main control loop (main loop rate, typically 50Hz)
     *          - update_current_mode: Execute active mode's update() method (50Hz)
     *          - read_radio: Process RC inputs (50Hz)
     *          - ahrs_update: Update AHRS/EKF state estimation (50Hz)
     *          - update_GPS: Process GPS measurements (10Hz)
     *          - update_compass: Read magnetometer (10Hz)
     *          - update_mission: Mission command processing (10Hz)
     *          - gcs_failsafe_check: Monitor GCS heartbeat (10Hz)
     *          - one_second_loop: Low-priority periodic tasks (1Hz)
     * 
     *          Task priorities are implicit in array order - earlier tasks execute first.
     *          Scheduler enforces timing budgets to prevent task overruns affecting subsequent tasks.
     * 
     * @note Defined in Rover.cpp with implementation details
     * @warning Modifying task rates or adding CPU-intensive tasks can affect control loop timing
     * 
     * @see AP_Scheduler for task scheduling implementation
     * @see Rover.cpp for scheduler_tasks array definition
     * 
     * Source: Rover/Rover.h:226
     */
    static const AP_Scheduler::Task scheduler_tasks[];

    static const AP_Param::Info var_info[];
#if HAL_LOGGING_ENABLED
    static const LogStructure log_structure[];
#endif

    // latest wheel encoder values
    float wheel_encoder_last_distance_m[WHEELENCODER_MAX_INSTANCES];    // total distance recorded by wheel encoder (for reporting to GCS)
    bool wheel_encoder_initialised;                                     // true once arrays below have been initialised to sensors initial values
    float wheel_encoder_last_angle_rad[WHEELENCODER_MAX_INSTANCES];     // distance in radians at time of last update to EKF
    uint32_t wheel_encoder_last_reading_ms[WHEELENCODER_MAX_INSTANCES]; // system time of last ping from each encoder
    uint8_t wheel_encoder_last_index_sent;                              // index of the last wheel encoder sent to the EKF

    // True when we are doing motor test
    bool motor_test;

    /**
     * @brief Mode objects defining available flight modes for the vehicle
     * 
     * @details Each mode object implements specific control behavior via the Mode base class.
     *          The active mode is referenced by control_mode pointer. Mode transitions occur
     *          through set_mode() which validates arming requirements and executes mode enter/exit logic.
     * 
     * @note All mode objects are instantiated at startup; control_mode pointer selects active mode
     * @see Mode base class for mode interface
     * @see set_mode() for mode transition logic
     */
    
    /// @brief Initializing mode: Vehicle startup phase before first mode selection
    ModeInitializing mode_initializing;
    
    /// @brief Hold mode: Stop all motors and hold position (no throttle or steering)
    ModeHold mode_hold;
    
    /// @brief Manual mode: Direct pilot control of throttle and steering with no stabilization
    ModeManual mode_manual;
    
    /// @brief Acro mode: Rate-based control for aggressive maneuvering (typically for balance bots)
    ModeAcro mode_acro;
    
    /// @brief Guided mode: Accept position/velocity targets from GCS or companion computer
    ModeGuided mode_guided;
    
    /// @brief Auto mode: Execute mission commands from AP_Mission (waypoints, conditional logic, camera triggers)
    ModeAuto mode_auto;
    
    /// @brief Loiter mode: Hold position using GPS and heading hold
    ModeLoiter mode_loiter;
    
    /// @brief Steering mode: Pilot controls steering, autonomous speed control to target speed
    ModeSteering mode_steering;
    
    /// @brief RTL mode: Return to launch position using direct path navigation
    ModeRTL mode_rtl;
    
    /// @brief SmartRTL mode: Return along recorded path (avoids obstacles encountered during mission)
    ModeSmartRTL mode_smartrtl;
    
#if MODE_FOLLOW_ENABLED
    /// @brief Follow mode: Track and follow another vehicle using MAVLink position data
    ModeFollow mode_follow;
#endif
    
    /// @brief Simple mode: Simplified control with heading reference (pilot stick directions relative to vehicle heading at arming)
    ModeSimple mode_simple;
    
#if MODE_DOCK_ENABLED
    /// @brief Dock mode: Autonomous docking using precision landing sensors
    ModeDock mode_dock;
#endif

    /**
     * @brief Cruise throttle and speed learning system
     * 
     * @details Learns relationship between throttle input and vehicle speed during steady-state
     *          cruising to improve autonomous speed control accuracy. Filters collect data during
     *          manual driving to build speed/throttle model for autonomous modes.
     * 
     * @note Learning activated by parameter or GCS command during manual driving
     */
    typedef struct {
        LowPassFilterFloat speed_filt{2.0f};    ///< Low-pass filter for speed measurements (Hz)
        LowPassFilterFloat throttle_filt{2.0f}; ///< Low-pass filter for throttle inputs (Hz)
        uint32_t learn_start_ms;                ///< System time (ms) when learning started
        uint32_t log_count;                     ///< Number of samples logged during learning
    } cruise_learn_t;
    cruise_learn_t cruise_learn; ///< Cruise learning state and filters

    // Rover.cpp
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
    /**
     * @brief Set guided mode target location for scripting or external control
     * 
     * @details Commands the vehicle to navigate to specified target location when in guided mode.
     *          Used by Lua scripting (AP_Scripting) and external control interfaces (companion computers).
     *          Target location is passed to mode_guided for waypoint navigation.
     * 
     * @param[in] target_loc Target location in global coordinates (latitude, longitude, altitude)
     * 
     * @return true if target was accepted and vehicle is/will navigate to target
     * @return false if vehicle is not in guided mode or target is invalid
     * 
     * @note Requires vehicle to be in guided mode for target to be accepted
     * @warning Ensure target location is within geofence boundaries if fence is enabled
     * 
     * @see ModeGuided for guided mode target handling
     * 
     * Source: Rover/Rover.h:272
     */
    bool set_target_location(const Location& target_loc) override;
#endif

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Set target velocity in North-East-Down frame for guided mode
     * 
     * @param[in] vel_ned Target velocity vector in m/s (NED frame: North, East, Down)
     * @return true if velocity target was accepted
     * @return false if not in appropriate mode or velocity invalid
     * 
     * Source: Rover/Rover.h:276
     */
    bool set_target_velocity_NED(const Vector3f& vel_ned) override;
    
    /**
     * @brief Directly set steering and throttle outputs for scripting control
     * 
     * @param[in] steering Steering output normalized (-1.0 to +1.0, negative=left, positive=right)
     * @param[in] throttle Throttle output normalized (-1.0 to +1.0, negative=reverse, positive=forward)
     * @return true if outputs were accepted and applied to motors
     * @return false if not in appropriate mode for direct control
     * 
     * @warning Bypasses normal navigation and control loops - use with caution
     * Source: Rover/Rover.h:277
     */
    bool set_steering_and_throttle(float steering, float throttle) override;
    
    /**
     * @brief Get current steering and throttle outputs for monitoring
     * 
     * @param[out] steering Current steering output normalized (-1.0 to +1.0)
     * @param[out] throttle Current throttle output normalized (-1.0 to +1.0)
     * @return true if outputs were retrieved successfully
     * @return false if outputs unavailable
     * 
     * Source: Rover/Rover.h:278
     */
    bool get_steering_and_throttle(float& steering, float& throttle) override;
    
    /**
     * @brief Set desired turn rate and forward speed for scripting navigation control
     * 
     * @param[in] turn_rate Desired turn rate in degrees/sec (positive=right, negative=left)
     * @param[in] speed Desired forward speed in m/s (positive=forward, negative=reverse)
     * @return true if turn rate and speed targets were accepted
     * @return false if not in appropriate mode or parameters invalid
     * 
     * @note Preferred method for scripting-based navigation control (rate + speed)
     * Source: Rover/Rover.h:280
     */
    bool set_desired_turn_rate_and_speed(float turn_rate, float speed) override;
    
    /**
     * @brief Set desired forward speed for autonomous speed control
     * 
     * @param[in] speed Desired speed in m/s (positive=forward, negative=reverse)
     * @return true if speed target was accepted
     * @return false if not in appropriate mode or speed invalid
     * 
     * Source: Rover/Rover.h:281
     */
    bool set_desired_speed(float speed) override;
    
    /**
     * @brief Get specific control output value for monitoring
     * 
     * @param[in] control_output Type of control output requested (throttle, steering, etc.)
     * @param[out] control_value Output value in appropriate units
     * @return true if control output value retrieved successfully
     * @return false if output type not available
     * 
     * Source: Rover/Rover.h:282
     */
    bool get_control_output(AP_Vehicle::ControlOutput control_output, float &control_value) override;
    
    /**
     * @brief Enable navigation scripting mode
     * 
     * @param[in] mode Navigation scripting mode to enable
     * @return true if scripting mode enabled successfully
     * @return false if mode invalid or cannot be enabled
     * 
     * Source: Rover/Rover.h:283
     */
    bool nav_scripting_enable(uint8_t mode) override;
    
    /**
     * @brief Get next navigation script command with timing
     * 
     * @param[out] id Script command identifier
     * @param[out] cmd Command type
     * @param[out] arg1 First command argument
     * @param[out] arg2 Second command argument
     * @param[out] arg3 Third command argument (integer)
     * @param[out] arg4 Fourth command argument (integer)
     * @return true if script command retrieved successfully
     * @return false if no script command available
     * 
     * Source: Rover/Rover.h:284
     */
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;
    
    /**
     * @brief Signal completion of navigation script command
     * 
     * @param[in] id Script command identifier that has completed
     * 
     * Source: Rover/Rover.h:285
     */
    void nav_script_time_done(uint16_t id) override;
#endif // AP_SCRIPTING_ENABLED
    void ahrs_update();
    void gcs_failsafe_check(void);
    void update_logging1(void);
    void update_logging2(void);
    void one_second_loop(void);
    void update_current_mode(void);

    // balance_bot.cpp
    /**
     * @brief Execute pitch stabilization control for balance bot
     * 
     * @details Implements inverted pendulum control using pitch angle feedback to maintain
     *          balance. Adjusts throttle output based on pitch error to prevent tipping.
     *          Critical for balance bot frame type stability.
     * 
     * @param[in,out] throttle Throttle value modified by pitch controller for balance
     * 
     * @note Called from main control loop when frame type is balance bot
     * @warning Balance bot requires continuous high-rate pitch control for stability
     * 
     * Source: Rover/Rover.h:295
     */
    void balancebot_pitch_control(float &throttle);
    
    /**
     * @brief Check if vehicle is configured as balance bot frame type
     * 
     * @return true if frame type is balance bot (inverted pendulum control)
     * @return false for all other frame types
     * 
     * @note Balance bots require higher main loop rate (typically 400Hz vs 50Hz)
     * @see get_frame_type() for frame type configuration
     * 
     * Source: Rover/Rover.h:296
     */
    bool is_balancebot() const;

    // commands.cpp
    /**
     * @brief Set home location to vehicle's current GPS position
     * 
     * @details Updates home location used for RTL (Return To Launch) mode and distance-from-home
     *          calculations. Home location also serves as EKF origin for local coordinate frame.
     * 
     * @param[in] lock If true, prevent further home location changes (for mission/auto modes)
     * 
     * @return true if home location set successfully with valid position
     * @return false if position invalid (no GPS lock) or home already locked
     * 
     * @note Requires valid GPS position estimate (have_position must be true)
     * @warning Changing home during flight affects RTL destination and distance calculations
     * 
     * Source: Rover/Rover.h:299
     */
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Set home location to specified position
     * 
     * @param[in] loc Home location in global coordinates (latitude, longitude, altitude)
     * @param[in] lock If true, prevent further home location changes
     * 
     * @return true if home location set successfully
     * @return false if location invalid or home already locked
     * 
     * @note Used by GCS to set home before GPS lock or for mission planning
     * 
     * Source: Rover/Rover.h:300
     */
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Update home location in AHRS/EKF if position has drifted
     * 
     * @details Periodic check to update EKF home origin if position estimate has changed significantly
     * 
     * Source: Rover/Rover.h:301
     */
    void update_home();

    // crash_check.cpp
    void crash_check();

    // cruise_learn.cpp
    void cruise_learn_start();
    void cruise_learn_update();
    void cruise_learn_complete();
    void log_write_cruise_learn() const;

    // ekf_check.cpp
    void ekf_check();
    bool ekf_over_threshold();
    bool ekf_position_ok();
    void failsafe_ekf_event();
    void failsafe_ekf_off_event(void);

    // failsafe.cpp
    void failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on);
    void handle_battery_failsafe(const char* type_str, const int8_t action);
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
    void afs_fs_check(void);
#endif
#if AP_FENCE_ENABLED
    // fence.cpp
    void fence_checks_async() override;
    void fence_check();
#endif
    // GCS_Mavlink.cpp
    void send_wheel_encoder_distance(mavlink_channel_t chan);

#if HAL_LOGGING_ENABLED
    // methods for AP_Vehicle:
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    uint8_t get_num_log_structures() const override;

    // Log.cpp
    void Log_Write_Attitude();
    void Log_Write_Depth();
    void Log_Write_GuidedTarget(uint8_t target_type, const Vector3f& pos_target, const Vector3f& vel_target);
    void Log_Write_Nav_Tuning();
    void Log_Write_Sail();
    void Log_Write_Steering();
    void Log_Write_Throttle();
    void Log_Write_RC(void);
    void Log_Write_Vehicle_Startup_Messages();
    void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page);
#endif

    // mode.cpp
    Mode *mode_from_mode_num(enum Mode::Number num);

    // Parameters.cpp
    void load_parameters(void) override;

    // precision_landing.cpp
    void init_precland();
    void update_precland();

    // radio.cpp
    void set_control_channels(void) override;
    void init_rc_in();
    void read_radio();
    void radio_failsafe_check(uint16_t pwm);

    // sensors.cpp
    void update_compass(void);
    void update_wheel_encoder();
#if AP_RANGEFINDER_ENABLED
    void read_rangefinders(void);
#endif

    // Steering.cpp
    void set_servos(void);

    // Rover.cpp
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;

    // system.cpp
    /**
     * @brief Initialize ArduPilot rover system and all subsystems
     * 
     * @details Complete system initialization sequence executed once at vehicle startup:
     *          1. Initialize HAL (Hardware Abstraction Layer) - board-specific setup
     *          2. Load parameters from EEPROM/flash storage
     *          3. Initialize RC input channels and map to control functions
     *          4. Initialize motors and servo outputs
     *          5. Initialize sensor systems (GPS, compass, barometer, rangefinders, IMU)
     *          6. Initialize AHRS/EKF state estimation
     *          7. Initialize mission management, fence, and rally systems
     *          8. Initialize GCS communication (MAVLink)
     *          9. Initialize logging system
     *          10. Enter first mode (typically HOLD or MANUAL)
     * 
     *          Startup sequence is critical for safety - failures halt initialization with error messages.
     * 
     * @note Called once from main() during vehicle boot
     * @warning Errors during initialization prevent vehicle from becoming operational
     * 
     * @see startup_ground() for ground-based initialization steps
     * @see AP_HAL for hardware initialization
     * 
     * Source: Rover/Rover.h:386
     */
    void init_ardupilot() override;
    
    /**
     * @brief Ground-based startup sequence after sensor initialization
     * 
     * Source: Rover/Rover.h:387
     */
    void startup_ground(void);
    
    /**
     * @brief Update AHRS with flyforward state for better state estimation
     * 
     * @details Informs AHRS whether vehicle is moving forward to improve EKF velocity estimation
     *          and GPS heading calculation. Important for transition between stationary and moving states.
     * 
     * Source: Rover/Rover.h:388
     */
    void update_ahrs_flyforward();
    
    /**
     * @brief Check if specified mode is enabled via GCS mode channels
     * 
     * @param[in] mode_num Mode number to check
     * @return true if mode is enabled and available
     * @return false if mode is disabled or unavailable
     * 
     * Source: Rover/Rover.h:389
     */
    bool gcs_mode_enabled(const Mode::Number mode_num) const;
    
    /**
     * @brief Set vehicle to new flight mode with validation and state transitions
     * 
     * @details Validates mode change, executes exit logic for current mode, enters new mode.
     *          Performs arming checks if mode change requires armed state.
     *          Updates control_mode pointer and notifies GCS of mode change.
     * 
     * @param[in] new_mode Reference to mode object to enter
     * @param[in] reason Reason for mode change (pilot command, failsafe, GCS command, etc.)
     * 
     * @return true if mode change successful and vehicle now in new mode
     * @return false if mode change rejected (arming required, mode unavailable, etc.)
     * 
     * @note Mode changes logged to dataflash with reason code for post-flight analysis
     * @warning Failsafe-triggered mode changes override pilot commands
     * 
     * @see Mode::enter() for mode-specific initialization
     * @see Mode::exit() for mode-specific cleanup
     * 
     * Source: Rover/Rover.h:390
     */
    bool set_mode(Mode &new_mode, ModeReason reason);
    
    /**
     * @brief Set vehicle mode by numeric mode identifier
     * 
     * @param[in] new_mode Mode number (0-255)
     * @param[in] reason Reason for mode change
     * @return true if mode change successful
     * @return false if mode number invalid or change rejected
     * 
     * Source: Rover/Rover.h:391
     */
    bool set_mode(const uint8_t new_mode, ModeReason reason) override;
    
    /**
     * @brief Set vehicle mode by Mode::Number enum
     * 
     * @param[in] new_mode Mode enum value
     * @param[in] reason Reason for mode change
     * @return true if mode change successful
     * @return false if mode unavailable or change rejected
     * 
     * Source: Rover/Rover.h:392
     */
    bool set_mode(Mode::Number new_mode, ModeReason reason);
    
    /**
     * @brief Get current mode number for reporting to GCS
     * 
     * @return uint8_t Current mode number
     * 
     * Source: Rover/Rover.h:393
     */
    uint8_t get_mode() const override { return (uint8_t)control_mode->mode_number(); }
    
    /**
     * @brief Check if current mode requires valid mission for operation
     * 
     * @return true if current mode is AUTO (requires mission commands)
     * @return false for all other modes
     * 
     * @note Used by arming checks to verify mission validity before allowing AUTO mode
     * 
     * Source: Rover/Rover.h:394
     */
    bool current_mode_requires_mission() const override {
        return control_mode == &mode_auto;
    }

    /**
     * @brief Initialize Inertial Navigation System (IMU and gyro calibration)
     * 
     * @details Performs IMU startup calibration during boot. Vehicle must be stationary
     *          during this phase for accurate gyro bias estimation.
     * 
     * @note Called during init_ardupilot() startup sequence
     * 
     * Source: Rover/Rover.h:398
     */
    void startup_INS(void);
    
    /**
     * @brief Notify systems of mode change (update LED, buzzer, GCS)
     * 
     * @param[in] new_mode Pointer to mode being entered
     * 
     * Source: Rover/Rover.h:399
     */
    void notify_mode(const Mode *new_mode);
    
    /**
     * @brief Check if digital pin is in valid range
     * 
     * @param[in] pin Pin number to validate
     * @return uint8_t Validated pin number or 0xFF if invalid
     * 
     * Source: Rover/Rover.h:400
     */
    uint8_t check_digital_pin(uint8_t pin);
    
    /**
     * @brief Check if specified logging type should be active
     * 
     * @param[in] mask Logging type bitmask to check
     * @return true if logging type enabled in g.log_bitmask parameter
     * @return false if logging type disabled
     * 
     * Source: Rover/Rover.h:401
     */
    bool should_log(uint32_t mask);
    
    /**
     * @brief Check if vehicle is configured as boat frame type
     * 
     * @return true if frame type is boat (marine vehicle with water-based control)
     * @return false for all other frame types (rover, balance bot, sailboat)
     * 
     * @note Affects control behavior, failsafe actions, and mode availability
     * @see get_frame_type() for frame type values
     * 
     * Source: Rover/Rover.h:402
     */
    bool is_boat() const;

    // vehicle specific waypoint info helpers
    bool get_wp_distance_m(float &distance) const override;
    bool get_wp_bearing_deg(float &bearing) const override;
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;

    /**
     * @enum FailsafeAction
     * @brief Available failsafe actions when failsafe conditions triggered
     * 
     * @details Defines vehicle behavior when failsafe conditions occur (RC loss, GCS loss,
     *          battery low, EKF failure, etc.). Action configured per failsafe type via parameters.
     */
    enum class FailsafeAction: int8_t {
        None          = 0, ///< No action taken (continue current mode)
        RTL           = 1, ///< Return to launch position via direct path
        Hold          = 2, ///< Stop vehicle immediately (motors to neutral)
        SmartRTL      = 3, ///< Return via recorded path (avoids obstacles)
        SmartRTL_Hold = 4, ///< SmartRTL if path available, else Hold
        Terminate     = 5, ///< Emergency termination (disarm motors immediately)
        Loiter_Hold   = 6, ///< Loiter in place if possible, else Hold
    };

    /**
     * @enum Failsafe_Options
     * @brief Failsafe option flags modifying failsafe behavior
     */
    enum class Failsafe_Options : uint32_t {
        Failsafe_Option_Active_In_Hold = (1<<0) ///< Allow failsafes to trigger even in Hold mode
    };

    /**
     * @brief Failsafe action priority array (highest priority first)
     * 
     * @details When multiple failsafes active simultaneously, highest priority action taken.
     *          Priority ordering: Terminate > Hold > RTL > SmartRTL_Hold > SmartRTL > None
     *          Array terminated with -1 sentinel value.
     * 
     * @note Used by failsafe_check() to determine action when multiple failsafes active
     */
    static constexpr int8_t _failsafe_priorities[] = {
                                                       (int8_t)FailsafeAction::Terminate,
                                                       (int8_t)FailsafeAction::Hold,
                                                       (int8_t)FailsafeAction::RTL,
                                                       (int8_t)FailsafeAction::SmartRTL_Hold,
                                                       (int8_t)FailsafeAction::SmartRTL,
                                                       (int8_t)FailsafeAction::None,
                                                       -1 // the priority list must end with a sentinel of -1
                                                      };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");


public:
    /**
     * @brief Check all failsafe conditions and trigger appropriate actions
     * 
     * @details Monitors multiple failsafe conditions each main loop iteration:
     *          - RC signal loss: No valid RC input for FS_TIMEOUT seconds
     *          - GCS communication loss: No MAVLink heartbeat for FS_GCS_TIMEOUT
     *          - EKF failure: Navigation estimate divergence or sensor failures
     *          - Battery failsafe: Voltage or capacity below configured thresholds
     * 
     *          When failsafe triggered:
     *          1. Sets failsafe.bits flags for active conditions
     *          2. Determines highest-priority action from _failsafe_priorities array
     *          3. Triggers mode change to failsafe mode (Hold, RTL, SmartRTL, etc.)
     *          4. Updates failsafe.triggered flags
     *          5. Sends MAVLink notification to GCS
     * 
     *          Failsafe actions cleared when conditions resolve and manual control restored.
     * 
     * @note Called every main loop iteration from scheduler
     * @warning Failsafe actions override pilot commands until condition clears
     * 
     * @see failsafe_trigger() for action execution
     * @see FailsafeAction enum for available actions
     * @see _failsafe_priorities for action priority ordering
     * 
     * Source: Rover/Rover.h:437
     */
    void failsafe_check();
    
    /**
     * @brief Output motor test PWM values during motor testing
     * 
     * @details Generates motor outputs for ground testing without arming vehicle.
     *          Used by MAVLink motor test commands to verify motor connections and directions.
     * 
     * @note Only active when motor_test flag is true
     * @warning Ensure vehicle is safely restrained during motor testing
     * 
     * Source: Rover/Rover.h:439
     */
    void motor_test_output();
    /**
     * @brief Check if motor test request is valid and safe to execute
     * 
     * @param[in] gcs_chan GCS MAVLink channel requesting motor test
     * @param[in] check_rc Whether to check RC inputs for safety
     * @param[in] motor_instance Which motor to test
     * @param[in] throttle_type Type of throttle value (PWM, percent, etc.)
     * @param[in] throttle_value Throttle value to command
     * 
     * @return true if motor test can proceed safely
     * @return false if unsafe conditions detected (armed, moving, etc.)
     * 
     * Source: Rover/Rover.h:440
     */
    bool mavlink_motor_test_check(const GCS_MAVLINK &gcs_chan, bool check_rc, AP_MotorsUGV::motor_test_order motor_instance, uint8_t throttle_type, int16_t throttle_value);
    
    /**
     * @brief Start motor test sequence from MAVLink command
     * 
     * @param[in] gcs_chan GCS MAVLink channel requesting motor test
     * @param[in] motor_instance Which motor to test
     * @param[in] throttle_type Type of throttle value
     * @param[in] throttle_value Throttle value to command
     * @param[in] timeout_sec Test duration in seconds (auto-stop after timeout)
     * 
     * @return MAV_RESULT Success/failure code for MAVLink command acknowledgment
     * 
     * @warning Vehicle must be restrained during motor testing
     * 
     * Source: Rover/Rover.h:441
     */
    MAV_RESULT mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, AP_MotorsUGV::motor_test_order motor_instance, uint8_t throttle_type, int16_t throttle_value, float timeout_sec);
    
    /**
     * @brief Stop motor test and return to normal operation
     * 
     * Source: Rover/Rover.h:442
     */
    void motor_test_stop();

    /**
     * @brief Get vehicle frame type configuration
     * 
     * @details Returns configured frame type determining vehicle control behavior:
     *          - 0: Undefined/Rover - Standard wheeled rover with configurable steering
     *          - 1: Boat - Marine vehicle with differential or vectored thrust
     *          - 2: BalanceBot - Inverted pendulum requiring active pitch stabilization
     *          - 3: Sailboat - Wind-powered with sail angle and rudder control
     * 
     *          Frame type affects:
     *          - Motor mixing and control allocation
     *          - Turn rate limits and handling characteristics
     *          - Failsafe behavior (boats don't "hold" position like rovers)
     *          - Available sensors and control modes
     * 
     * @return uint8_t Frame type identifier from g2.frame_type parameter
     * 
     * @see is_boat() for boat-specific behavior checks
     * @see is_balancebot() for balance bot identification
     * @see AP_MotorsUGV for motor control variations by frame type
     * 
     * Source: Rover/Rover.h:445
     */
    uint8_t get_frame_type() const { return g2.frame_type.get(); }
    
    /**
     * @brief Get wheel rate controller for closed-loop wheel speed control
     * 
     * @return Reference to AP_WheelRateControl for wheel encoder feedback control
     * 
     * Source: Rover/Rover.h:446
     */
    AP_WheelRateControl& get_wheel_rate_control() { return g2.wheel_rate_control; }

    // Simple mode
    float simple_sin_yaw;

#if AP_ROVER_AUTO_ARM_ONCE_ENABLED
    struct {
        uint32_t last_arm_attempt_ms;
        bool done;
    } auto_arm_once;
    void handle_auto_arm_once();
#endif  // AP_ROVER_AUTO_ARM_ONCE_ENABLED
};

/**
 * @brief Global Rover singleton instance
 * 
 * @details Single global instance of the Rover class providing access to all vehicle
 *          subsystems throughout the codebase. Instantiated at program startup and
 *          initialized via init_ardupilot(). All vehicle control, navigation, and
 *          communication flows through this instance.
 * 
 *          Friend classes (Mode subclasses, GCS_Rover, AP_Arming_Rover, etc.) access
 *          rover private members directly for tight integration and performance.
 * 
 * @note Singleton pattern - only one rover instance exists per vehicle
 * @warning Do not create additional Rover instances; use global 'rover' object
 * 
 * @see Rover class for vehicle controller implementation
 * 
 * Source: Rover/Rover.h:460
 */
extern Rover rover;

using AP_HAL::millis;
using AP_HAL::micros;
