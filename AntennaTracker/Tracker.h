/*
   Lead developers: Matthew Ridley and Andrew Tridgell

   Please contribute your ideas! See https://ardupilot.org/dev for details

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
 * @file Tracker.h
 * @brief Main Tracker class declaration for antenna tracker vehicle
 * 
 * @details This file declares the Tracker class which extends AP_Vehicle
 *          to implement a complete antenna tracking system. The tracker
 *          implements an alt-azimuth pointing system that automatically
 *          aims antennas at a moving vehicle using MAVLink telemetry.
 * 
 *          The system supports multiple tracking modes, various servo types,
 *          and can operate on both stationary and mobile platforms.
 * 
 * Source: AntennaTracker/Tracker.h:1-232
 */
#pragma once

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <Filter/Filter.h>                     // Filter library

#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>

#include <SRV_Channel/SRV_Channel.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library

// Configuration
#include "config.h"
#include "defines.h"

#include "RC_Channel_Tracker.h"
#include "Parameters.h"
#include "GCS_MAVLink_Tracker.h"
#include "GCS_Tracker.h"

#include "AP_Arming_Tracker.h"

#include "mode.h"

/**
 * @class Tracker
 * @brief Main antenna tracker vehicle class
 * 
 * @details This singleton class implements a complete antenna tracking system
 *          that automatically points antennas at a moving vehicle. The tracker
 *          receives vehicle position and velocity via MAVLink telemetry and
 *          calculates the required pan (yaw) and tilt (pitch) angles to keep
 *          the antenna aimed at the target.
 * 
 *          Key Features:
 *          - Automatic vehicle tracking using MAVLink GLOBAL_POSITION_INT messages
 *          - Multiple tracking modes: AUTO (tracking with scan fallback), MANUAL (RC control),
 *            GUIDED (external MAVLink control), SCAN (search pattern), SERVOTEST, STOP
 *          - Velocity prediction for improved tracking of fast-moving vehicles
 *          - Support for position servos (PID control), on/off servos (bang-bang), 
 *            and continuous rotation servos
 *          - Barometer or GPS altitude selection for elevation calculation
 *          - Mobile tracker support with coordinate transformations for moving platforms
 *          - Smooth servo motion using PID controllers and low-pass filters
 * 
 *          Tracking Algorithm:
 *          1. Receives vehicle position/velocity from MAVLink telemetry (50Hz update rate)
 *          2. Projects vehicle location forward using velocity compensation
 *          3. Calculates bearing and distance using spherical earth model
 *          4. Computes pitch angle based on altitude difference and distance
 *          5. Updates servo positions via PID controllers (position mode) or
 *             bang-bang control (on/off mode) to minimize tracking error
 * 
 *          Coordinate Systems:
 *          - GPS coordinates: Latitude/longitude in degrees * 1e7, altitude in cm
 *          - NED frame: Used for velocity vectors (North-East-Down in m/s)
 *          - Angles: Bearing in centidegrees (0-36000), pitch in degrees
 *          - Servo outputs: Angles converted to PWM via SRV_Channels
 * 
 * @note This class extends AP_Vehicle to inherit common vehicle functionality
 *       including parameter management, logging, GCS communication, and scheduling.
 * 
 * @note A single global instance 'tracker' is declared at the end of Tracker.cpp
 *       and is the only instance of this class that should exist.
 * 
 * @warning This is flight control software. Incorrect servo limits or PID tuning
 *          can cause rapid servo movements that may damage antenna equipment.
 * 
 * Source: AntennaTracker/Tracker.h:58-229
 */
class Tracker : public AP_Vehicle {
public:
    /**
     * @brief Friend class declarations allow controlled access to private members
     * @note These classes need access to internal state for MAVLink communication,
     *       parameter management, and mode control operations
     */
    friend class GCS_MAVLINK_Tracker;
    friend class GCS_Tracker;
    friend class Parameters;
    friend class ModeAuto;
    friend class ModeGuided;
    friend class Mode;

    /**
     * @brief Arms (enables) tracker servo outputs
     * @details Enables servo output channels after safety checks pass, allowing
     *          servos to move and track vehicle
     * @note Must be armed before servos will respond to tracking commands
     * @warning Servos will move immediately to tracking position when armed
     */
    void arm_servos();
    
    /**
     * @brief Disarms (disables) tracker servo outputs
     * @details Disables servo output channels, preventing servo movement
     * @note Servos will hold last position but will not respond to tracking
     */
    void disarm_servos();

private:
    Parameters g;  ///< @brief Configuration parameters loaded from EEPROM (see Parameters.h)

    uint32_t start_time_ms = 0;  ///< @brief System startup timestamp in milliseconds (for uptime calculation)

    /**
     * @brief Antenna control input/output channels
     * @details RC input channels for manual control and servo output channels for antenna positioning
     */
    RC_Channels_Tracker rc_channels;  ///< @brief RC receiver input channels (pitch/yaw sticks in MANUAL mode)
    SRV_Channels servo_channels;      ///< @brief Servo output channels (converts angles to PWM signals)

    LowPassFilterFloat yaw_servo_out_filt;    ///< @brief Low-pass filter for smooth yaw servo output
    LowPassFilterFloat pitch_servo_out_filt;  ///< @brief Low-pass filter for smooth pitch servo output

    bool yaw_servo_out_filt_init = false;     ///< @brief True after yaw filter initialized with first value
    bool pitch_servo_out_filt_init = false;   ///< @brief True after pitch filter initialized with first value

    GCS_Tracker _gcs;  ///< @brief Ground Control Station communication handler (avoid direct use)
    GCS_Tracker &gcs() { return _gcs; }  ///< @brief Accessor for GCS object (preferred access method)

    /**
     * @brief Battery monitoring system
     * @details Monitors battery voltage/current with failsafe callback integration
     * @note Configured with MASK_LOG_CURRENT for current logging and failsafe handler
     */
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Tracker::handle_battery_failsafe, void, const char*, const int8_t),
                           nullptr};
    Location current_loc;  ///< @brief Current tracker GPS position (updated by update_tracker_position())

    Mode *mode_from_mode_num(enum Mode::Number num);

    Mode *mode = &mode_initialising;  ///< @brief Pointer to currently active tracking mode

    // Mode instances - each implements a specific tracking behavior
    ModeAuto mode_auto;              ///< @brief AUTO mode: Automatic tracking with scan fallback if target lost
    ModeInitialising mode_initialising; ///< @brief INITIALISING mode: Startup mode during system initialization
    ModeManual mode_manual;          ///< @brief MANUAL mode: Direct RC control of pan/tilt servos
    ModeGuided mode_guided;          ///< @brief GUIDED mode: External MAVLink control of tracker pointing
    ModeScan mode_scan;              ///< @brief SCAN mode: Automated search pattern to locate vehicle
    ModeServoTest mode_servotest;    ///< @brief SERVOTEST mode: Direct PWM commands for servo testing
    ModeStop mode_stop;              ///< @brief STOP mode: Hold current position, disable tracking

    /**
     * @struct vehicle
     * @brief Tracked vehicle state information
     * 
     * @details Contains the current position, velocity, and status of the vehicle
     *          being tracked. Position data is received via MAVLink GLOBAL_POSITION_INT
     *          messages and updated at approximately 50Hz in AUTO mode.
     * 
     * @note Position coordinates use GPS convention: lat/lng in degrees * 1e7, altitude in cm
     * @note Velocity uses NED frame: North-East-Down in m/s
     */
    struct {
        bool location_valid;         ///< @brief True if vehicle position has been recently updated (within timeout period)
        Location location;           ///< @brief Vehicle GPS location: lat/lng in degrees*1e7, altitude in cm
        Location location_estimate;  ///< @brief Projected vehicle location compensated with velocity for reduced lag
        uint32_t last_update_us;     ///< @brief Timestamp of last position update in microseconds (for precise timing)
        uint32_t last_update_ms;     ///< @brief Timestamp of last position update in milliseconds (for timeout checks)
        Vector3f vel;                ///< @brief Vehicle velocity vector in m/s (NED frame: North, East, Down)
        int32_t relative_alt;        ///< @brief Vehicle altitude relative to home position in cm (positive = above home)
    } vehicle;

    /**
     * @struct NavStatus
     * @brief Navigation and tracking status information
     * 
     * @details Contains all calculated tracking angles, errors, and control flags used
     *          by the servo control system. These values are computed by update_bearing_and_distance()
     *          and update_tracking() at the main loop rate (typically 50Hz).
     * 
     * @note Bearing uses centidegrees (0-36000) for internal calculations
     * @note Pitch uses degrees with positive = vehicle above tracker, negative = below
     * @note Altitude differences are always in meters, positive = vehicle higher than tracker
     */
    struct NavStatus {
        float bearing;                   ///< @brief Bearing to vehicle in centidegrees (0-36000, 0=North, 9000=East)
        float distance;                  ///< @brief Horizontal distance to vehicle in meters (ground plane distance)
        float pitch;                     ///< @brief Elevation angle to vehicle in degrees (+up/-down, 0=horizon)
        float angle_error_pitch;         ///< @brief Pitch tracking error in centidegrees (target - current)
        float angle_error_yaw;           ///< @brief Yaw tracking error in centidegrees (target - current)
        float alt_difference_baro;       ///< @brief Altitude delta in meters per barometer (+vehicle higher)
        float alt_difference_gps;        ///< @brief Altitude delta in meters per GPS (+vehicle higher)
        float altitude_offset;           ///< @brief Barometer calibration offset in meters (added to tracker altitude)
        bool manual_control_yaw      : 1;///< @brief True if yaw servo is under manual RC control override
        bool manual_control_pitch    : 1;///< @brief True if pitch servo is under manual RC control override
        bool need_altitude_calibration : 1;///< @brief True until one-time barometer calibration performed at startup
        bool scan_reverse_pitch      : 1;///< @brief Scan pattern direction flag for pitch axis (reverses at limits)
        bool scan_reverse_yaw        : 1;///< @brief Scan pattern direction flag for yaw axis (reverses at limits)
    } nav_status;

    /**
     * @brief Parameter management system loader
     * @details Initializes parameter system with var_info table for EEPROM storage/retrieval
     */
    AP_Param param_loader{var_info};

    bool target_set = false;  ///< @brief True if tracker has received valid vehicle position data
    bool stationary = true;   ///< @brief True if tracker is stationary (false for mobile tracker platforms)

    /**
     * @brief Static data tables for system configuration
     * @note These are defined in respective .cpp files
     */
    static const AP_Scheduler::Task scheduler_tasks[];  ///< @brief Task table for AP_Scheduler (defines loop functions and rates)
    static const AP_Param::Info var_info[];             ///< @brief Parameter metadata table for AP_Param system
    static const struct LogStructure log_structure[];   ///< @brief Binary log message format definitions

    /**
     * @name Tracker.cpp - Main loop and scheduling functions
     * @{
     */
    
    /**
     * @brief Returns scheduler task table to AP_Scheduler
     * @param[out] tasks Pointer to task array
     * @param[out] task_count Number of tasks in array
     * @param[out] log_bit Logging bitmask for scheduler
     * @details Provides the list of periodic functions to be called by the main scheduler
     * Source: AntennaTracker/Tracker.cpp
     */
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;
    
    /**
     * @brief 1Hz housekeeping and statistics loop
     * @details Called once per second for low-priority tasks like statistics updates,
     *          compass learning, and periodic GCS updates
     * Source: AntennaTracker/Tracker.cpp
     */
    void one_second_loop();
    
    /**
     * @brief 10Hz logging loop
     * @details Writes attitude and vehicle position logs if logging is enabled
     * Source: AntennaTracker/Tracker.cpp
     */
    void ten_hz_logging_loop();
    
    /**
     * @brief Updates performance statistics
     * @details Tracks main loop timing and performance metrics
     * Source: AntennaTracker/Tracker.cpp
     */
    void stats_update();
    /** @} */  // End Tracker.cpp group

    /**
     * @name GCS_Mavlink.cpp - Ground Control Station communication
     * @{
     */
    
    /**
     * @brief Sends navigation controller output to ground station
     * @param[in] chan MAVLink channel to send on
     * @details Transmits NAV_CONTROLLER_OUTPUT message with bearing, distance,
     *          pitch angle, and tracking errors for ground station display
     * Source: AntennaTracker/GCS_Mavlink.cpp
     */
    void send_nav_controller_output(mavlink_channel_t chan);
    /** @} */  // End GCS_Mavlink.cpp group

#if HAL_LOGGING_ENABLED
    /**
     * @name Logging - AP_Vehicle interface implementations
     * @{
     */
    
    /**
     * @brief Returns logging bitmask parameter
     * @return Reference to LOG_BITMASK parameter
     * @details Controls which log types are enabled
     */
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }
    
    /**
     * @brief Returns log message structure definitions
     * @return Pointer to LogStructure array
     * @details Provides binary log format definitions for AP_Logger
     */
    const struct LogStructure *get_log_structures() const override {
        return log_structure;
    }
    
    /**
     * @brief Returns number of defined log structures
     * @return Count of log message types
     */
    uint8_t get_num_log_structures() const override;
    /** @} */  // End logging interface group
    
    /**
     * @name Log.cpp - Binary logging functions
     * @{
     */
    
    /**
     * @brief Writes tracker attitude to binary log
     * @details Logs current pitch and yaw angles of tracker platform
     * Source: AntennaTracker/Log.cpp
     */
    void Log_Write_Attitude();
    
    /**
     * @brief Writes vehicle barometer data to binary log
     * @param[in] pressure Vehicle barometric pressure
     * @param[in] altitude Vehicle barometric altitude
     * @details Logs vehicle baro data received via MAVLink SCALED_PRESSURE messages
     * Source: AntennaTracker/Log.cpp
     */
    void Log_Write_Vehicle_Baro(float pressure, float altitude);
    
    /**
     * @brief Writes vehicle position and velocity to binary log
     * @param[in] lat Vehicle latitude in degrees * 1e7
     * @param[in] lng Vehicle longitude in degrees * 1e7
     * @param[in] alt Vehicle altitude in centimeters
     * @param[in] vel Vehicle velocity vector in m/s (NED frame)
     * @details Logs vehicle position/velocity from MAVLink GLOBAL_POSITION_INT messages
     * Source: AntennaTracker/Log.cpp
     */
    void Log_Write_Vehicle_Pos(int32_t lat,int32_t lng,int32_t alt, const Vector3f& vel);
    
    /**
     * @brief Writes startup messages to binary log
     * @details Logs initial system state and configuration at boot
     * Source: AntennaTracker/Log.cpp
     */
    void Log_Write_Vehicle_Startup_Messages();
    /** @} */  // End Log.cpp group
#endif

    /**
     * @name Parameters.cpp - Parameter management
     * @{
     */
    
    /**
     * @brief Loads parameters from EEPROM
     * @details Reads all configuration parameters from persistent storage during initialization
     * Source: AntennaTracker/Parameters.cpp
     */
    void load_parameters(void) override;
    /** @} */  // End Parameters.cpp group

    /**
     * @name radio.cpp - RC receiver input
     * @{
     */
    
    /**
     * @brief Reads RC input from receiver
     * @details Updates RC channel values from receiver hardware, typically called at 50Hz
     * @note Input used for MANUAL mode control and mode switches
     * Source: AntennaTracker/radio.cpp
     */
    void read_radio();
    /** @} */  // End radio.cpp group

    /**
     * @name sensors.cpp - Sensor update and calibration
     * @{
     */
    
    /**
     * @brief Updates AHRS (Attitude and Heading Reference System)
     * @details Updates EKF or DCM attitude estimation with latest sensor data
     * @note Called at main loop rate (typically 50Hz) to maintain accurate attitude
     * Source: AntennaTracker/sensors.cpp
     */
    void update_ahrs();
    
    /**
     * @brief Updates compass readings
     * @details Reads magnetometer data and performs calibration if active
     * Source: AntennaTracker/sensors.cpp
     */
    void update_compass(void);
    
    /**
     * @brief Updates GPS position
     * @details Reads GPS data and handles ground start initialization logic
     * @note First GPS fix triggers initialization of home position
     * Source: AntennaTracker/sensors.cpp
     */
    void update_GPS(void);
    
    /**
     * @brief Handles battery failsafe conditions
     * @param[in] type_str Battery type identifier string
     * @param[in] action Failsafe action code
     * @details Battery failsafe callback (currently no-operation for tracker)
     * @note Tracker typically powered externally, battery monitoring optional
     * Source: AntennaTracker/sensors.cpp
     */
    void handle_battery_failsafe(const char* type_str, const int8_t action);
    /** @} */  // End sensors.cpp group

    /**
     * @name servos.cpp - Servo control and output
     * @{
     */
    
    /**
     * @brief Initializes servo output channels and filters
     * @details Configures servo channels, initializes low-pass filters for smooth output
     * Source: AntennaTracker/servos.cpp
     */
    void init_servos();
    
    /**
     * @brief Pitch servo output dispatcher
     * @param[in] pitch Target pitch angle in degrees
     * @details Routes to appropriate pitch servo implementation based on SERVO_TYPE parameter
     *          (position, on/off, or continuous rotation servo)
     * Source: AntennaTracker/servos.cpp
     */
    void update_pitch_servo(float pitch);
    
    /**
     * @brief PID position control for pitch servo
     * @details Implements smooth position control using PID controller and slew rate limiting
     * @note Used for standard position servos with ~180° range
     * Source: AntennaTracker/servos.cpp
     */
    void update_pitch_position_servo(void);
    
    /**
     * @brief Bang-bang (on/off) control for pitch servo
     * @param[in] pitch Target pitch angle in degrees
     * @details Implements simple on/off control with deadband for basic servos
     * @note Faster response but less smooth than position control
     * Source: AntennaTracker/servos.cpp
     */
    void update_pitch_onoff_servo(float pitch) const;
    
    /**
     * @brief Continuous rotation servo control for pitch
     * @param[in] pitch Target pitch angle in degrees
     * @details Controls continuous rotation (CR) servos that rotate continuously rather than positioning
     * @note Used for unlimited rotation pitch axis
     * Source: AntennaTracker/servos.cpp
     */
    void update_pitch_cr_servo(float pitch);
    
    /**
     * @brief Yaw servo output dispatcher
     * @param[in] yaw Target yaw angle in degrees
     * @details Routes to appropriate yaw servo implementation based on SERVO_TYPE parameter
     *          (position, on/off, or continuous rotation servo)
     * Source: AntennaTracker/servos.cpp
     */
    void update_yaw_servo(float yaw);
    
    /**
     * @brief PID position control for yaw servo
     * @details Implements smooth position control using PID controller and slew rate limiting
     * @note Used for standard position servos with ~180° or 360° range
     * Source: AntennaTracker/servos.cpp
     */
    void update_yaw_position_servo(void);
    
    /**
     * @brief Bang-bang (on/off) control for yaw servo
     * @param[in] yaw Target yaw angle in degrees
     * @details Implements simple on/off control with deadband for basic servos
     * @note Faster response but less smooth than position control
     * Source: AntennaTracker/servos.cpp
     */
    void update_yaw_onoff_servo(float yaw) const;
    
    /**
     * @brief Continuous rotation servo control for yaw
     * @param[in] yaw Target yaw angle in degrees
     * @details Controls continuous rotation (CR) servos for unlimited yaw rotation
     * @note Enables 360° continuous rotation for azimuth tracking
     * Source: AntennaTracker/servos.cpp
     */
    void update_yaw_cr_servo(float yaw);
    /** @} */  // End servos.cpp group

    /**
     * @name system.cpp - System initialization and configuration
     * @{
     */
    
    /**
     * @brief Main initialization function
     * @details Initializes all subsystems: HAL, parameters, sensors, servos, GCS, scheduler
     * @note Called once at startup before entering main loop
     * Source: AntennaTracker/system.cpp
     */
    void init_ardupilot() override;
    
    /**
     * @brief Reads home location from EEPROM
     * @param[out] loc Home location to populate
     * @return true if valid home location retrieved, false otherwise
     * @details Retrieves saved home position from persistent storage
     * Source: AntennaTracker/system.cpp
     */
    bool get_home_eeprom(Location &loc) const;
    
    /**
     * @brief Saves home location to EEPROM
     * @param[in] temp Home location to save
     * @return true if successfully saved, false on error
     * @details Writes home position to persistent storage for position reference
     * Source: AntennaTracker/system.cpp
     */
    bool set_home_eeprom(const Location &temp) WARN_IF_UNUSED;
    
    /**
     * @brief Sets home to current tracker GPS position
     * @param[in] lock True to lock home position (prevent changes)
     * @return true if successfully set, false if no GPS fix
     * @details Uses current GPS location as reference point for tracking
     * Source: AntennaTracker/system.cpp
     */
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Sets home to specified location
     * @param[in] temp Location to set as home
     * @param[in] lock True to lock home position
     * @return true if successfully set, false on error
     * @details Manually sets reference position for tracking calculations
     * Source: AntennaTracker/system.cpp
     */
    bool set_home(const Location &temp, bool lock) override WARN_IF_UNUSED;
    
    /**
     * @brief Prepares servos for operation
     * @details Arms/enables servo outputs after safety checks
     * Source: AntennaTracker/system.cpp
     */
    void prepare_servos();
    
    /**
     * @brief Changes tracking mode
     * @param[in] newmode Reference to new mode object
     * @param[in] reason Reason for mode change (for logging)
     * @details Exits current mode and enters new tracking mode
     * Source: AntennaTracker/system.cpp
     */
    void set_mode(Mode &newmode, ModeReason reason);
    
    /**
     * @brief Changes tracking mode by number
     * @param[in] new_mode Mode number to change to
     * @param[in] reason Reason for mode change
     * @return true if mode change successful, false if invalid mode
     * @details Looks up mode by number and calls set_mode()
     * Source: AntennaTracker/system.cpp
     */
    bool set_mode(uint8_t new_mode, ModeReason reason) override;
    
    /**
     * @brief Returns current mode number
     * @return Current tracking mode as uint8_t
     * @details Used by AP_Vehicle interface for status reporting
     */
    uint8_t get_mode() const override { return (uint8_t)mode->number(); }
    
    /**
     * @brief Determines if logging should occur
     * @param[in] mask Logging bitmask to check
     * @return true if this log type is enabled, false otherwise
     * @details Checks LOG_BITMASK parameter against requested log type
     * Source: AntennaTracker/system.cpp
     */
    bool should_log(uint32_t mask);
    
    /**
     * @brief Mission command start callback (unused)
     * @param[in] cmd Mission command to start
     * @return false (missions not supported on tracker)
     * @note Tracker does not support autonomous missions, only tracking modes
     */
    bool start_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    
    /**
     * @brief Mission exit callback (unused)
     * @note Tracker does not support autonomous missions
     */
    void exit_mission_callback() { return; }
    
    /**
     * @brief Mission command verify callback (unused)
     * @param[in] cmd Mission command to verify
     * @return false (missions not supported on tracker)
     * @note Tracker does not support autonomous missions
     */
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd) { return false; }
    /** @} */  // End system.cpp group

    /**
     * @name tracking.cpp - Vehicle tracking and servo control
     * @{
     */
    
    /**
     * @brief Projects vehicle position forward using velocity compensation
     * @details Estimates future vehicle position by extrapolating current position
     *          with velocity vector to compensate for telemetry and servo lag
     * @note Reduces tracking error for fast-moving vehicles
     * Source: AntennaTracker/tracking.cpp
     */
    void update_vehicle_pos_estimate();
    
    /**
     * @brief Updates tracker's own GPS position
     * @details Reads tracker GPS to support mobile tracker platforms that move
     * @note For stationary trackers, uses fixed home position
     * Source: AntennaTracker/tracking.cpp
     */
    void update_tracker_position();
    
    /**
     * @brief Calculates bearing and distance to vehicle
     * @details Computes horizontal bearing (0-360°), ground distance, and elevation
     *          angle using spherical earth model. Updates nav_status structure.
     * @note Called at main loop rate (50Hz) when vehicle position valid
     * Source: AntennaTracker/tracking.cpp
     */
    void update_bearing_and_distance();
    
    /**
     * @brief Main 50Hz tracking control loop
     * @details High-level tracking coordinator that:
     *          1. Updates vehicle position estimate with velocity compensation
     *          2. Updates tracker's own position (if mobile)
     *          3. Calculates bearing/distance/pitch to vehicle
     *          4. Calls current mode's update() to control servos
     *          5. Updates armed/disarmed LED status
     * @note This is the core tracking function called by scheduler
     * Source: AntennaTracker/tracking.cpp
     */
    void update_tracking(void);
    
    /**
     * @brief Handles GLOBAL_POSITION_INT MAVLink messages
     * @param[in] msg MAVLink message with vehicle position/velocity
     * @details Processes vehicle position updates from telemetry:
     *          - Extracts lat/lng/alt (GPS coordinates in degrees*1e7, cm)
     *          - Extracts velocity vector (NED frame in cm/s, converted to m/s)
     *          - Updates vehicle structure and marks location_valid
     *          - Timestamps update for timeout detection
     * @note Primary source of vehicle tracking data, typically 50Hz in AUTO mode
     * Source: AntennaTracker/tracking.cpp
     */
    void tracking_update_position(const mavlink_global_position_int_t &msg);
    
    /**
     * @brief Handles SCALED_PRESSURE MAVLink messages
     * @param[in] msg MAVLink message with vehicle barometer data
     * @details Processes vehicle barometer readings for altitude tracking when
     *          ALT_SOURCE parameter set to barometer mode
     * @note Alternative to GPS altitude for elevation angle calculation
     * Source: AntennaTracker/tracking.cpp
     */
    void tracking_update_pressure(const mavlink_scaled_pressure_t &msg);
    
    /**
     * @brief Handles MANUAL_CONTROL MAVLink messages
     * @param[in] msg MAVLink joystick/manual control message
     * @details Processes external manual control inputs from ground station for GUIDED mode
     * Source: AntennaTracker/tracking.cpp
     */
    void tracking_manual_control(const mavlink_manual_control_t &msg);
    
    /**
     * @brief Updates armed/disarmed LED indicator
     * @details Controls notification LED to show tracker arm status
     * Source: AntennaTracker/tracking.cpp
     */
    void update_armed_disarmed() const;
    
    /**
     * @brief Gets normalized pan/tilt angles for ONVIF cameras
     * @param[out] pan_norm Normalized pan angle (-1.0 to +1.0)
     * @param[out] tilt_norm Normalized tilt angle (-1.0 to +1.0)
     * @return true if angles valid, false otherwise
     * @details Converts current servo angles to normalized values for camera control
     * @note Used for integration with ONVIF PTZ (Pan-Tilt-Zoom) cameras
     * Source: AntennaTracker/tracking.cpp
     */
    bool get_pan_tilt_norm(float &pan_norm, float &tilt_norm) const override;
    /** @} */  // End tracking.cpp group

    /**
     * @brief Arming/disarming management system
     * @details Handles pre-arm checks and servo arming/disarming with safety interlocks
     * @note Tracker-specific arming checks ensure GPS lock and valid home position
     */
    AP_Arming_Tracker arming;

    /**
     * @brief Mission management library (unused)
     * @details Mission library present for AP_Vehicle interface compatibility but
     *          tracker does not support autonomous missions - only tracking modes
     * @note Callbacks return false/no-op since missions not applicable to tracker
     */
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Tracker::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Tracker::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Tracker::exit_mission_callback, void)};
};

/**
 * @brief Global tracker instance
 * @details Single global Tracker object instantiated in Tracker.cpp
 * @note This is the only instance of Tracker that should exist
 */
extern Tracker tracker;
