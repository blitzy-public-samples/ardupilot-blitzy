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
 * @file AP_Mount_Backend.h
 * @brief Abstract base class for mount/gimbal backend drivers
 * 
 * @details This file defines the AP_Mount_Backend class, which serves as the abstract
 *          interface for all mount/gimbal hardware backends (e.g., Servo, Alexmos, Gremsy,
 *          Siyi, etc.). Each physical gimbal implementation derives from this class.
 * 
 *          The backend handles:
 *          - Lifecycle: init() → update() loop → update_fast() for high-rate IMU data
 *          - Mode management: MAV_MOUNT_MODE enum (RETRACT, NEUTRAL, MAVLINK_TARGETING, RC_TARGETING, GPS_POINT, SYSID_TARGET, HOME_LOCATION)
 *          - Target storage: Angle targets (radians) and rate targets (rad/s) with earth-frame/body-frame yaw options
 *          - RC-to-target conversion: Pilot RC input → angle or rate targets based on configuration
 *          - ROI tracking: Region of Interest and SYSID target tracking with automatic angle calculation
 *          - MAVLink protocol: Gimbal Device and Gimbal Manager message handling/generation
 *          - Camera control: Integration with gimbals that include cameras (take_picture, record_video, zoom, focus, tracking)
 *          - Optional POI calculation: Threaded point-of-interest lat/lon/alt calculation when AP_MOUNT_POI_TO_LATLONALT_ENABLED
 *          - Logging: Mount attitude and target logging hooks
 * 
 * @note Update rates: update() should be called at ≥10Hz, update_fast() at INS rate (typically 400Hz+) for gimbals requiring high-rate IMU data
 * @note Coordinate frames: Yaw can be earth-frame (maintains heading like North) or body-frame (rotates with vehicle)
 * @warning This is an abstract base class - do not instantiate directly
 * 
 * @see AP_Mount (frontend manager class)
 * @see AP_Mount_Servo, AP_Mount_Alexmos, AP_Mount_Gremsy, AP_Mount_Siyi (example backend implementations)
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Camera/AP_Camera_shareddefs.h>
#include "AP_Mount.h"

/**
 * @class AP_Mount_Backend
 * @brief Abstract base class for all mount/gimbal backend implementations
 * 
 * @details This class provides the interface and common functionality for all mount/gimbal
 *          hardware backends. Derived classes implement specific communication protocols
 *          (e.g., PWM servo, MAVLink, serial) to control physical gimbal hardware.
 * 
 *          Backend Architecture:
 *          - init(): One-time initialization of hardware interfaces
 *          - update(): Main update loop called at ≥10Hz for mode logic and target calculation
 *          - update_fast(): Optional high-rate update (INS rate) for gimbals needing full-rate IMU data
 * 
 *          Mode/State Management:
 *          - Supports all MAV_MOUNT_MODE enum values (RETRACT, NEUTRAL, MAVLINK_TARGETING, RC_TARGETING, GPS_POINT, SYSID_TARGET, HOME_LOCATION)
 *          - Mode determines target source: RC pilot input, MAVLink commands, ROI location, vehicle tracking, home location
 *          - Mode transitions validated and logged
 * 
 *          Target Storage (mnt_target structure):
 *          - ANGLE mode: Stores target angles in radians (roll, pitch, yaw) with yaw_is_ef flag
 *          - RATE mode: Stores target rates in rad/s (roll, pitch, yaw) with yaw_is_ef flag
 *          - yaw_is_ef: true = earth-frame yaw (locks to compass heading), false = body-frame yaw (follows vehicle)
 * 
 *          RC-to-Target Conversion:
 *          - Reads RC_ROLL/RC_PITCH/RC_YAW parameters for RC channel assignments
 *          - Converts RC input (-1 to +1) to angle targets or rate targets based on TYPE parameter
 *          - Applies dead zones, rate limits, and exponential curves
 *          - Implements yaw_lock (follow vs lock) behavior
 * 
 *          ROI and SYSID Tracking:
 *          - set_roi_target(): Sets Location for GPS_POINT mode, calculates required gimbal angles to point at location
 *          - set_target_sysid(): Sets MAVLink sysid for SYSID_TARGET mode, tracks vehicle position from GLOBAL_POSITION_INT
 *          - get_angle_target_to_roi(), get_angle_target_to_sysid(), get_angle_target_to_home(): Helper functions calculate angles to targets
 * 
 *          GCS/MAVLink Helpers:
 *          - Gimbal Device protocol: handle_gimbal_device_information(), handle_gimbal_device_attitude_status(), send_gimbal_device_attitude_status()
 *          - Gimbal Manager protocol: handle_command_do_gimbal_manager_configure(), send_gimbal_manager_information(), send_gimbal_manager_status()
 *          - Legacy support: handle_command_do_mount_control(), handle_gimbal_report()
 *          - Capability flags: get_gimbal_manager_capability_flags() defines gimbal capabilities (axes, rates, etc.)
 * 
 *          Optional POI Thread (when AP_MOUNT_POI_TO_LATLONALT_ENABLED):
 *          - calculate_poi(): Runs in separate thread to calculate point-of-interest lat/lon/alt from gimbal attitude and location
 *          - get_poi(): Retrieves calculated POI with semaphore protection
 *          - Used by GCS to display gimbal pointing location on map
 * 
 *          Logging Hooks:
 *          - write_log(): Logs mount attitude, target, and mode to dataflash
 *          - Backends override to add hardware-specific log fields
 * 
 * @note Angles stored internally in radians, parameters use degrees for user convenience
 * @note Rate targets in rad/s internally, parameters in deg/s
 * @note Thread-safety: POI calculation uses semaphore protection when enabled
 * @warning Backends must enforce gimbal angle limits defined in parameters (ROLL_ANGLE_MIN/MAX, PITCH_ANGLE_MIN/MAX, YAW_ANGLE_MIN/MAX)
 * @warning Mode transitions may cause gimbal movement - validate safe transitions in set_mode()
 * 
 * @see AP_Mount (frontend manager)
 * @see AP_Mount_Params (parameter storage)
 * @see MAV_MOUNT_MODE enum (mode definitions in common MAVLink messages)
 */
class AP_Mount_Backend
{
public:
    /**
     * @brief Constructor for mount backend
     * 
     * @param[in] frontend Reference to AP_Mount frontend manager that created this backend
     * @param[in] params Reference to parameter storage for this mount instance
     * @param[in] instance Mount instance number (0-based, for multi-mount systems)
     */
    AP_Mount_Backend(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance) :
        _frontend(frontend),
        _params(params),
        _instance(instance)
    {}

    /**
     * @brief Performs any required initialization for this mount instance
     * 
     * @details Called once during vehicle startup after backend construction. Backends override
     *          to initialize hardware interfaces (serial ports, I2C/SPI devices, PWM outputs).
     *          Base implementation is empty.
     * 
     * @note Called from AP_Mount::init() after all backends are constructed
     */
    virtual void init();

    /**
     * @brief Set device ID for this mount instance
     * 
     * @param[in] id Device identifier to store in MNTx_DEVID parameter for hardware identification
     * 
     * @details Used to identify specific gimbal hardware models. Device ID follows AP_HAL::Device
     *          conventions (bus type, bus instance, address encoded in uint32_t).
     */
    void set_dev_id(uint32_t id);

    /**
     * @brief Main update loop for mount position control - must be called periodically
     * 
     * @details Pure virtual function that backends must implement. Called at ≥10Hz by AP_Mount::update().
     *          Responsible for:
     *          - Reading current gimbal attitude (if available)
     *          - Processing mode-specific target logic
     *          - Sending new targets/commands to gimbal hardware
     *          - Updating internal state
     * 
     * @note Call rate: ≥10Hz typical, exact rate depends on vehicle scheduler configuration
     * @warning Backends must not block in this function - use non-blocking I/O
     */
    virtual void update() = 0;

    /**
     * @brief High-rate update for gimbals needing full-rate INS data
     * 
     * @details Optional function for gimbals that require attitude updates at IMU rate (typically 400Hz+).
     *          Called from AP_Mount::update_fast() at INS rate. Base implementation is empty.
     *          Override only if gimbal needs high-frequency data.
     * 
     * @note Call rate: INS rate (typically 400Hz), higher than update()
     * @note Most gimbals do not need this - override only for high-performance stabilization
     */
    virtual void update_fast() {}

    /**
     * @brief Check if mount is operating normally
     * 
     * @return true if mount is healthy and responding, false if communication lost or fault detected
     * 
     * @details Base implementation returns true. Backends override to report actual health status
     *          based on communication timeouts, error flags, or gimbal-reported faults.
     * 
     * @note Health status displayed to pilot and logged
     */
    virtual bool healthy() const { return true; }

    /**
     * @brief Check if mount accepts roll angle targets
     * 
     * @return true if roll control available, false if roll axis unsupported or disabled by user
     * 
     * @details Checks ROLL_ANGLE_MIN/MAX parameters. Returns false if min >= max (user-disabled).
     *          Allows disabling roll control even on 3-axis gimbals.
     */
    virtual bool has_roll_control() const;

    /**
     * @brief Check if mount accepts pitch angle targets
     * 
     * @return true if pitch control available, false if pitch axis unsupported or disabled by user
     * 
     * @details Checks PITCH_ANGLE_MIN/MAX parameters. Returns false if min >= max (user-disabled).
     *          Allows disabling pitch control even on 3-axis gimbals.
     */
    virtual bool has_pitch_control() const;

    /**
     * @brief Check if mount can control yaw/pan axis
     * 
     * @return true if yaw control available, false if yaw axis unsupported
     * 
     * @details Pure virtual - backends must report yaw capability. Required for multicopters
     *          to determine if vehicle must yaw to point gimbal (if gimbal has no pan control).
     * 
     * @note Critical for flight controller to decide vehicle yaw behavior in GPS_POINT and ROI modes
     */
    virtual bool has_pan_control() const = 0;

    /**
     * @brief Get current gimbal attitude as quaternion
     * 
     * @param[out] att_quat Gimbal attitude quaternion (earth-frame with yaw in body-frame)
     * @return true on success, false if attitude unavailable
     * 
     * @details Pure virtual - backends must provide gimbal attitude. The quaternion represents:
     *          - Roll/Pitch: Earth-frame (relative to horizon/gravity)
     *          - Yaw: Body-frame (relative to vehicle heading)
     *          
     *          This convention allows consistent attitude reporting regardless of yaw_lock mode.
     * 
     * @note Attitude used for logging, POI calculation, and GCS display
     * @note Coordinate frame: NED (North-East-Down) convention
     */
    virtual bool get_attitude_quaternion(Quaternion& att_quat) = 0;

    /**
     * @brief Get gimbal angular velocity
     * 
     * @param[out] rates Angular velocity vector (rad/s) in gimbal body-frame: roll, pitch, yaw rates
     * @return true on success, false if angular velocity unavailable
     * 
     * @details Base implementation returns false. Only available on backends that report
     *          or measure gimbal rates (e.g., MAVLink gimbals with rate feedback).
     * 
     * @note Units: rad/s
     * @note Frame: Gimbal body-frame angular rates
     */
    virtual bool get_angular_velocity(Vector3f& rates) { return false; }

    /**
     * @brief Check if mode is valid for this mount
     * 
     * @param[in] mode MAV_MOUNT_MODE to validate
     * @return true if mode supported, false if unsupported mode
     * 
     * @details Validates mode against backend capabilities. All backends must support
     *          RETRACT, NEUTRAL, MAVLINK_TARGETING, and RC_TARGETING.
     */
    bool valid_mode(MAV_MOUNT_MODE mode) const;

    /**
     * @brief Get current mount operating mode
     * 
     * @return Current MAV_MOUNT_MODE (RETRACT, NEUTRAL, MAVLINK_TARGETING, RC_TARGETING, GPS_POINT, SYSID_TARGET, HOME_LOCATION)
     */
    enum MAV_MOUNT_MODE get_mode() const { return _mode; }

    /**
     * @brief Set mount operating mode
     * 
     * @param[in] mode Desired MAV_MOUNT_MODE
     * @return true if mode change successful, false if mode invalid or not supported
     * 
     * @details Validates mode with valid_mode() before changing. Mode determines target source:
     *          - RETRACT: Gimbal retracted/stowed position
     *          - NEUTRAL: Gimbal centered/neutral position
     *          - MAVLINK_TARGETING: Target from MAVLink commands (set_angle_target/set_rate_target)
     *          - RC_TARGETING: Target from pilot RC input
     *          - GPS_POINT: Point at ROI location set by set_roi_target()
     *          - SYSID_TARGET: Track vehicle with sysid set by set_target_sysid()
     *          - HOME_LOCATION: Point at home location
     * 
     * @note Mode changes logged and may cause gimbal movement
     */
    bool set_mode(enum MAV_MOUNT_MODE mode);

    /**
     * @brief Set yaw lock mode for RC_TARGETING
     * 
     * @param[in] yaw_lock true = lock yaw to earth-frame heading, false = follow vehicle yaw (body-frame)
     * 
     * @details Controls yaw behavior in RC_TARGETING mode:
     *          - true (lock): Gimbal yaw maintains compass heading (e.g., points North regardless of vehicle rotation)
     *          - false (follow): Gimbal yaw rotates with vehicle, maintains body-frame angle
     * 
     * @note Only affects RC_TARGETING mode
     * @note Called from MAVLink DO_MOUNT_CONTROL command param7 or from mode transitions
     */
    void set_yaw_lock(bool yaw_lock) { _yaw_lock = yaw_lock; }

    /**
     * @brief Set angle target for MAVLINK_TARGETING mode
     * 
     * @param[in] roll_deg Target roll angle in degrees (earth-frame, positive = right side down)
     * @param[in] pitch_deg Target pitch angle in degrees (earth-frame, positive = nose up)
     * @param[in] yaw_deg Target yaw angle in degrees (frame depends on yaw_is_earth_frame)
     * @param[in] yaw_is_earth_frame true = yaw is earth-frame (compass heading), false = yaw is body-frame (relative to vehicle nose)
     * 
     * @details Stores target in mnt_target.angle_rad structure (converted to radians internally).
     *          Used by MAVLINK_TARGETING mode. Targets clamped to parameter limits.
     * 
     * @note Units: Input in degrees, stored internally in radians
     * @note Frame: Roll/pitch earth-frame (horizon-relative), yaw frame depends on yaw_is_earth_frame
     * @note Called from MAVLink DO_MOUNT_CONTROL and GIMBAL_MANAGER_SET_ATTITUDE commands
     */
    void set_angle_target(float roll_deg, float pitch_deg, float yaw_deg, bool yaw_is_earth_frame);

    /**
     * @brief Set rate target for MAVLINK_TARGETING mode
     * 
     * @param[in] roll_degs Target roll rate in deg/s
     * @param[in] pitch_degs Target pitch rate in deg/s
     * @param[in] yaw_degs Target yaw rate in deg/s
     * @param[in] yaw_is_earth_frame true = yaw rate in earth-frame, false = yaw rate in body-frame
     * 
     * @details Stores rate target in mnt_target.rate_rads structure (converted to rad/s internally).
     *          Used by MAVLINK_TARGETING mode. Rates integrated into angle targets in update_angle_target_from_rate().
     * 
     * @note Units: Input in deg/s, stored internally in rad/s
     * @note Rates integrated with 50Hz assumption in update_angle_target_from_rate()
     * @note Called from MAVLink GIMBAL_MANAGER_SET_ATTITUDE command with rate fields
     */
    void set_rate_target(float roll_degs, float pitch_degs, float yaw_degs, bool yaw_is_earth_frame);

    /**
     * @brief Set Region of Interest location for GPS_POINT mode
     * 
     * @param[in] target_loc Location to point gimbal towards (lat, lon, alt)
     * 
     * @details Stores ROI location. In GPS_POINT mode, get_angle_target_to_roi() calculates
     *          required gimbal angles to point at this location from current vehicle position.
     * 
     * @note Called from MAVLink DO_SET_ROI command and mission DO_SET_ROI commands
     * @note Automatically switches to GPS_POINT mode if not already active
     */
    void set_roi_target(const Location &target_loc);

    /**
     * @brief Clear Region of Interest target
     * 
     * @details Clears stored ROI location. If in GPS_POINT mode, may switch to NEUTRAL or previous mode.
     * 
     * @note Called from MAVLink DO_SET_ROI with zero coordinates or mission ROI cancellation
     */
    void clear_roi_target();

    /**
     * @brief Set target MAVLink system ID for SYSID_TARGET mode
     * 
     * @param[in] sysid MAVLink system ID of vehicle to track
     * 
     * @details Stores target sysid. In SYSID_TARGET mode, mount tracks vehicle position from
     *          GLOBAL_POSITION_INT messages with matching sysid. Used for antenna tracking
     *          or cooperative vehicle operations.
     * 
     * @note Requires handle_global_position_int() to update target position
     * @note Mode must be set to SYSID_TARGET separately
     */
    void set_target_sysid(uint8_t sysid);

    /**
     * @brief Handle MAVLink DO_MOUNT_CONTROL command
     * 
     * @param[in] packet MAVLink command_int packet with mount control parameters
     * @return MAV_RESULT_ACCEPTED on success, MAV_RESULT_DENIED/UNSUPPORTED/FAILED on error
     * 
     * @details Legacy mount control command. Processes:
     *          - param1-3: pitch, roll, yaw targets (degrees or deg/s depending on param7)
     *          - param4: unused
     *          - param5: unused  
     *          - param6: unused
     *          - param7: mode (2=MAV_MOUNT_MODE_MAVLINK_TARGETING, other values set mount mode)
     * 
     *          If param7 indicates rate control, calls set_rate_target(), else set_angle_target().
     * 
     * @note Legacy command, prefer GIMBAL_MANAGER_SET_ATTITUDE for new implementations
     * @see handle_command_do_gimbal_manager_configure() for modern gimbal manager protocol
     */
    MAV_RESULT handle_command_do_mount_control(const mavlink_command_int_t &packet);

    /**
     * @brief Handle MAVLink DO_GIMBAL_MANAGER_CONFIGURE command
     * 
     * @param[in] packet MAVLink command_int packet with gimbal manager configuration
     * @param[in] msg Original MAVLink message (for extracting sender sysid/compid)
     * @return MAV_RESULT_ACCEPTED on success, MAV_RESULT_DENIED if unauthorized
     * 
     * @details Configures gimbal manager control authority:
     *          - param1: sysid of primary controller (or -1 to remove control)
     *          - param2: compid of primary controller (or -1 to remove control)
     *          - param3: sysid of secondary controller (or -1 to remove control)
     *          - param4: compid of secondary controller (or -1 to remove control)
     * 
     *          Sets mavlink_control_id to track authorized controller for GIMBAL_MANAGER_SET_ATTITUDE commands.
     * 
     * @note Part of MAVLink gimbal manager protocol (MAVLink v2)
     * @see send_gimbal_manager_status() for status reporting
     */
    MAV_RESULT handle_command_do_gimbal_manager_configure(const mavlink_command_int_t &packet, const mavlink_message_t &msg);

    /**
     * @brief Send GIMBAL_DEVICE_ATTITUDE_STATUS message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Generates MAVLink GIMBAL_DEVICE_ATTITUDE_STATUS message containing:
     *          - Current gimbal attitude (quaternion from get_attitude_quaternion())
     *          - Angular velocity (if available from get_angular_velocity())
     *          - Failure flags and gimbal device flags
     *          - Target lock flag (for SYSID tracking)
     * 
     * @note Called periodically by AP_Mount to stream gimbal status to GCS
     * @note Part of MAVLink gimbal device protocol
     */
    void send_gimbal_device_attitude_status(mavlink_channel_t chan);

    /**
     * @brief Get gimbal capability flags for GIMBAL_MANAGER_INFORMATION message
     * 
     * @return Bitmask of GIMBAL_MANAGER_CAP_FLAGS (roll/pitch/yaw control, rates, infinite angles, etc.)
     * 
     * @details Backends override to report actual capabilities:
     *          - GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL/PITCH/YAW_AXIS
     *          - GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL/PITCH/YAW_FOLLOW
     *          - GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL/PITCH/YAW_LOCK
     *          - GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW
     *          - GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME
     *          - GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL/GLOBAL
     * 
     * @note Used by GCS to configure gimbal control UI
     */
    virtual uint32_t get_gimbal_manager_capability_flags() const;

    /**
     * @brief Send GIMBAL_MANAGER_INFORMATION message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Generates MAVLink GIMBAL_MANAGER_INFORMATION message containing:
     *          - Gimbal capability flags (from get_gimbal_manager_capability_flags())
     *          - Roll/pitch/yaw angle min/max limits (from parameters)
     *          - Roll/pitch/yaw rate max limits
     * 
     * @note Sent on request or periodically to inform GCS of gimbal capabilities
     * @note Part of MAVLink gimbal manager protocol
     */
    void send_gimbal_manager_information(mavlink_channel_t chan);

    /**
     * @brief Send GIMBAL_MANAGER_STATUS message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Generates MAVLink GIMBAL_MANAGER_STATUS message containing:
     *          - Primary control sysid/compid (from mavlink_control_id)
     *          - Secondary control sysid/compid
     *          - Current gimbal mode and flags
     * 
     * @note Sent periodically to report gimbal manager control authority
     * @note Part of MAVLink gimbal manager protocol
     */
    void send_gimbal_manager_status(mavlink_channel_t chan);

    /**
     * @brief Handle GIMBAL_REPORT MAVLink message
     * 
     * @param[in] chan MAVLink channel message received on
     * @param[in] msg MAVLink message containing GIMBAL_REPORT
     * 
     * @details Legacy gimbal reporting. Base implementation is empty. Backends override
     *          for gimbals using old GIMBAL_REPORT protocol (e.g., Solo gimbal).
     * 
     * @note Deprecated - modern gimbals use GIMBAL_DEVICE_ATTITUDE_STATUS
     */
    virtual void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg) {}

    /**
     * @brief Handle PARAM_VALUE MAVLink message
     * 
     * @param[in] msg MAVLink message containing PARAM_VALUE
     * 
     * @details Receives parameter values from gimbal. Base implementation is empty.
     *          Backends override for gimbals with configurable parameters via MAVLink
     *          parameter protocol (e.g., Storm32, Gremsy).
     * 
     * @note Used for gimbal parameter configuration and status monitoring
     */
    virtual void handle_param_value(const mavlink_message_t &msg) {}

    /**
     * @brief Handle GLOBAL_POSITION_INT MAVLink message
     * 
     * @param[in] msg_sysid MAVLink system ID of message sender
     * @param[in] packet GLOBAL_POSITION_INT message data
     * @return true if message processed (sysid matches _target_sysid), false if ignored
     * 
     * @details Processes vehicle position updates for SYSID_TARGET tracking mode.
     *          If msg_sysid matches _target_sysid, updates _target_sysid_location
     *          for get_angle_target_to_sysid() calculations.
     * 
     * @note Only processes if in SYSID_TARGET mode and sysid matches
     * @note Called by AP_Mount for all received GLOBAL_POSITION_INT messages
     */
    bool handle_global_position_int(uint8_t msg_sysid, const mavlink_global_position_int_t &packet);

    /**
     * @brief Handle GIMBAL_DEVICE_INFORMATION MAVLink message
     * 
     * @param[in] msg MAVLink message containing GIMBAL_DEVICE_INFORMATION
     * 
     * @details Receives gimbal device information (capabilities, limits, version).
     *          Base implementation is empty. Backends override for MAVLink gimbals
     *          that report their capabilities (e.g., DroneCAN gimbals with MAVLink forwarding).
     * 
     * @note Part of MAVLink gimbal device protocol
     * @note Used to auto-configure gimbal capabilities and limits
     */
    virtual void handle_gimbal_device_information(const mavlink_message_t &msg) {}

    /**
     * @brief Handle GIMBAL_DEVICE_ATTITUDE_STATUS MAVLink message
     * 
     * @param[in] msg MAVLink message containing GIMBAL_DEVICE_ATTITUDE_STATUS
     * 
     * @details Receives gimbal attitude status from gimbal device. Base implementation
     *          is empty. Backends override for MAVLink gimbals (e.g., Siyi, Gremsy)
     *          to update internal attitude tracking from gimbal-reported attitude.
     * 
     * @note Part of MAVLink gimbal device protocol
     * @note Typically processed at 10-50Hz depending on gimbal configuration
     */
    virtual void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) {}

    /**
     * @brief Get current rate target
     * 
     * @param[out] roll_degs Roll rate target in deg/s
     * @param[out] pitch_degs Pitch rate target in deg/s
     * @param[out] yaw_degs Yaw rate target in deg/s
     * @param[out] yaw_is_earth_frame true if yaw rate in earth-frame, false if body-frame
     * @return true if rate target available, false if not in RATE mode
     * 
     * @details Retrieves rate target from mnt_target.rate_rads (converted to deg/s).
     *          Only valid when mnt_target.target_type == MountTargetType::RATE.
     * 
     * @note Units: Output in deg/s (converted from internal rad/s)
     */
    bool get_rate_target(float& roll_degs, float& pitch_degs, float& yaw_degs, bool& yaw_is_earth_frame);

    /**
     * @brief Get current angle target
     * 
     * @param[out] roll_deg Roll angle target in degrees
     * @param[out] pitch_deg Pitch angle target in degrees
     * @param[out] yaw_deg Yaw angle target in degrees
     * @param[out] yaw_is_earth_frame true if yaw in earth-frame, false if body-frame
     * @return true if angle target available, false if not in ANGLE mode
     * 
     * @details Retrieves angle target from mnt_target.angle_rad (converted to degrees).
     *          Only valid when mnt_target.target_type == MountTargetType::ANGLE.
     * 
     * @note Units: Output in degrees (converted from internal radians)
     * @note Frame: Roll/pitch earth-frame, yaw frame indicated by yaw_is_earth_frame
     */
    bool get_angle_target(float& roll_deg, float& pitch_deg, float& yaw_deg, bool& yaw_is_earth_frame);

    /**
     * @brief Get location target for scripting backends
     * 
     * @param[out] target_loc Location target (ROI or SYSID location)
     * @return true if location target available, false if unsupported or no target set
     * 
     * @details Base implementation returns false. Backends override to provide location
     *          target for Lua scripting access. Returns ROI target in GPS_POINT mode
     *          or tracked vehicle location in SYSID_TARGET mode.
     * 
     * @note Primarily for Lua scripting API access
     */
    virtual bool get_location_target(Location &target_loc) { return false; }

    /**
     * @brief Set gimbal attitude for scripting backends
     * 
     * @param[in] roll_deg Roll angle in degrees
     * @param[in] pitch_deg Pitch angle in degrees
     * @param[in] yaw_bf_deg Yaw angle in degrees (body-frame)
     * 
     * @details Base implementation is empty. Backends override to allow Lua scripting
     *          direct control of gimbal attitude. Bypasses normal mode logic.
     * 
     * @note Primarily for Lua scripting API
     * @note Yaw is body-frame only in this interface
     * @warning Bypasses safety checks - use with caution
     */
    virtual void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) {};

    /**
     * @brief Write mount attitude and target to dataflash log
     * 
     * @param[in] timestamp_us Timestamp in microseconds for log entry
     * 
     * @details Logs mount attitude (from get_attitude_quaternion()), angle target,
     *          rate target, mode, and instance. Called periodically by AP_Mount.
     * 
     * @note Logged to MOUNT message type in dataflash
     * @note Log rate typically 10-25Hz
     */
    void write_log(uint64_t timestamp_us);

    //
    // Camera controls for gimbals that include a camera
    //

    /**
     * @brief Trigger camera shutter to take a picture
     * 
     * @return true on success, false if unsupported or command failed
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          integrated cameras (e.g., Siyi, Viewpro). Sends shutter trigger command
     *          to gimbal camera.
     * 
     * @note May be called from MAVLink CAMERA_TRIGGER or DO_DIGICAM_CONTROL commands
     * @note Some gimbals require video recording stopped before taking pictures
     */
    virtual bool take_picture() { return false; }

    /**
     * @brief Start or stop video recording
     * 
     * @param[in] start_recording true to start recording, false to stop recording
     * @return true on success, false if unsupported or command failed
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          integrated cameras supporting video recording. Sends start/stop command
     *          to gimbal camera.
     * 
     * @note May be called from MAVLink VIDEO_START_CAPTURE/VIDEO_STOP_CAPTURE commands
     */
    virtual bool record_video(bool start_recording) { return false; }

    /**
     * @brief Set camera zoom
     * 
     * @param[in] zoom_type Type of zoom command (RATE for continuous, VALUE for absolute position, PCT for percentage)
     * @param[in] zoom_value Zoom value (rate in steps/s, position in steps, or percentage 0-100)
     * @return true on success, false if unsupported or command failed
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          zoom cameras. Zoom types:
     *          - ZoomType::RATE: Continuous zoom at zoom_value steps/s (negative = zoom out)
     *          - ZoomType::VALUE: Absolute zoom position
     *          - ZoomType::PCT: Zoom percentage (0 = wide, 100 = full tele)
     * 
     * @note May be called from MAVLink CAMERA_ZOOM or SET_CAMERA_ZOOM commands
     */
    virtual bool set_zoom(ZoomType zoom_type, float zoom_value) { return false; }

    /**
     * @brief Set camera focus
     * 
     * @param[in] focus_type Type of focus command (RATE, VALUE, PCT, or AUTO)
     * @param[in] focus_value Focus value (rate -1=in/0=hold/+1=out, position, or percentage)
     * @return SetFocusResult::ACCEPTED on success, FAILED on error, UNSUPPORTED if no focus control
     * 
     * @details Base implementation returns UNSUPPORTED. Backends override for gimbals with
     *          focus control. Focus types:
     *          - FocusType::RATE: Continuous focus (focus_value: -1=focus in, 0=hold, +1=focus out)
     *          - FocusType::VALUE: Absolute focus position
     *          - FocusType::PCT: Focus percentage
     *          - FocusType::AUTO: Auto-focus (one-shot or continuous depending on backend)
     * 
     * @note May be called from MAVLink CAMERA_FOCUS or SET_CAMERA_FOCUS commands
     */
    virtual SetFocusResult set_focus(FocusType focus_type, float focus_value) { return SetFocusResult::UNSUPPORTED; }

    /**
     * @brief Set camera tracking mode
     * 
     * @param[in] tracking_type Tracking type (NONE, POINT, or RECTANGLE)
     * @param[in] p1 Point 1 coordinates (x, y) in normalized 0-1 range (0=left/top, 1=right/bottom)
     * @param[in] p2 Point 2 coordinates (x, y) for RECTANGLE mode (bottom-right corner)
     * @return true on success, false if unsupported or command failed
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          object tracking. Tracking types:
     *          - TrackingType::NONE: Disable tracking
     *          - TrackingType::POINT: Track object at point p1 (p2 ignored)
     *          - TrackingType::RECTANGLE: Track object in rectangle (p1=top-left, p2=bottom-right)
     * 
     * @note Coordinates normalized to image dimensions (0,0)=top-left, (1,1)=bottom-right
     * @note May be called from MAVLink CAMERA_TRACKING commands
     */
    virtual bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) { return false; }

    /**
     * @brief Set camera lens
     * 
     * @param[in] lens Lens number (0-5, meaning depends on gimbal)
     * @return true on success, false if unsupported or invalid lens number
     * 
     * @details Base implementation returns false. Backends override for multi-lens gimbals
     *          (e.g., thermal + visible, wide + tele). Lens numbering is gimbal-specific.
     * 
     * @note May be called from MAVLink CAMERA_SET_LENS commands
     */
    virtual bool set_lens(uint8_t lens) { return false; }

#if HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED
    /**
     * @brief Set camera source (advanced multi-camera control)
     * 
     * @param[in] primary_source Primary camera source (AP_Camera::CameraSource enum cast to uint8_t)
     * @param[in] secondary_source Secondary camera source (AP_Camera::CameraSource enum cast to uint8_t)
     * @return true on success, false if unsupported or invalid source
     * 
     * @details Similar to set_lens() but uses typed camera sources (e.g., RGB, IR, etc.).
     *          Base implementation returns false. Backends override for gimbals supporting
     *          source-based camera selection with picture-in-picture or split-screen modes.
     * 
     * @note Only available when HAL_MOUNT_SET_CAMERA_SOURCE_ENABLED is defined
     * @see AP_Camera::CameraSource enum for source type definitions
     */
    virtual bool set_camera_source(uint8_t primary_source, uint8_t secondary_source) { return false; }
#endif

    /**
     * @brief Send CAMERA_INFORMATION MAVLink message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Base implementation is empty. Backends override for gimbals with cameras
     *          to send camera capabilities: resolution, FOV, sensor size, firmware version.
     * 
     * @note Part of MAVLink camera protocol
     * @note Sent on request from GCS REQUEST_MESSAGE
     */
    virtual void send_camera_information(mavlink_channel_t chan) const {}

    /**
     * @brief Send CAMERA_SETTINGS MAVLink message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Base implementation is empty. Backends override for gimbals with cameras
     *          to send current camera settings: mode (photo/video), zoom level, focus mode.
     * 
     * @note Part of MAVLink camera protocol
     * @note Sent periodically or on request
     */
    virtual void send_camera_settings(mavlink_channel_t chan) const {}

    /**
     * @brief Send CAMERA_CAPTURE_STATUS MAVLink message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Base implementation is empty. Backends override for gimbals with cameras
     *          to send capture status: recording state, available storage, image count.
     * 
     * @note Part of MAVLink camera protocol
     * @note Sent periodically to update GCS with recording/storage status
     */
    virtual void send_camera_capture_status(mavlink_channel_t chan) const {}

#if AP_MOUNT_SEND_THERMAL_RANGE_ENABLED
    /**
     * @brief Send CAMERA_THERMAL_RANGE MAVLink message to GCS
     * 
     * @param[in] chan MAVLink channel to send on
     * 
     * @details Base implementation is empty. Backends override for gimbals with thermal
     *          cameras to send thermal image parameters: min/max temperature range.
     * 
     * @note Only available when AP_MOUNT_SEND_THERMAL_RANGE_ENABLED is defined
     * @note Part of MAVLink camera protocol for thermal imaging
     */
    virtual void send_camera_thermal_range(mavlink_channel_t chan) const {}
#endif

    /**
     * @brief Change camera settings not normally controlled by autopilot
     * 
     * @param[in] setting Camera setting to change (from CameraSetting enum)
     * @param[in] value New value for setting (meaning depends on setting type)
     * @return true on success, false if unsupported or command failed
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          advanced camera settings: exposure, aperture, ISO, white balance, etc.
     *          Settings defined in CameraSetting enum.
     * 
     * @note May be called from MAVLink CAMERA_SETTINGS or SET_CAMERA_SETTINGS commands
     * @see CameraSetting enum in AP_Camera_shareddefs.h
     */
    virtual bool change_setting(CameraSetting setting, float value) { return false; }

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    /**
     * @brief Get point-of-interest information (gimbal pointing location)
     * 
     * @param[in] instance Mount instance number
     * @param[out] quat Gimbal attitude quaternion used for POI calculation
     * @param[out] loc Gimbal location used for POI calculation
     * @param[out] poi_loc Calculated point-of-interest location (lat/lon/alt where gimbal points)
     * @return true on success, false if POI calculation not available
     * 
     * @details Retrieves POI calculation results from poi_calculation structure with semaphore
     *          protection. POI is calculated in separate thread by calculate_poi() using gimbal
     *          attitude, location, and rangefinder distance (if available).
     * 
     * @note Only available when AP_MOUNT_POI_TO_LATLONALT_ENABLED is defined
     * @note POI calculation runs in separate thread to avoid blocking main loop
     * @note Used by GCS to display gimbal pointing location on map
     * @warning Thread-safe access via semaphore - do not access poi_calculation directly
     */
    bool get_poi(uint8_t instance, Quaternion &quat, Location &loc, Location &poi_loc);
#endif

    //
    // Rangefinder integration for gimbals with laser rangefinders
    //

    /**
     * @brief Get distance measurement from gimbal-mounted rangefinder
     * 
     * @param[out] distance_m Distance measurement in meters
     * @return true on success and valid measurement, false if unavailable or invalid
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          integrated laser rangefinders (e.g., Siyi). Used for terrain mapping,
     *          object distance measurement, and POI altitude calculation.
     * 
     * @note Units: meters
     * @note May be used by calculate_poi() to determine POI altitude
     */
    virtual bool get_rangefinder_distance(float& distance_m) const { return false; }

    /**
     * @brief Enable or disable gimbal-mounted rangefinder
     * 
     * @param[in] enable true to enable rangefinder, false to disable
     * @return true on success, false if unsupported or command failed
     * 
     * @details Base implementation returns false. Backends override for gimbals with
     *          controllable rangefinders. Disabling may save power or avoid interference.
     * 
     * @note Some gimbals automatically enable rangefinder when needed
     */
    virtual bool set_rangefinder_enable(bool enable) { return false; }

protected:

    /**
     * @enum MountTargetType
     * @brief Target type for MAVLINK_TARGETING mode
     * 
     * @details Determines whether mnt_target stores angles or rates:
     *          - ANGLE: mnt_target.angle_rad contains angle targets (radians)
     *          - RATE: mnt_target.rate_rads contains rate targets (rad/s), integrated into angles
     */
    enum class MountTargetType {
        ANGLE,  ///< Angle target mode (position control)
        RATE,   ///< Rate target mode (velocity control)
    };

    /**
     * @class MountTarget
     * @brief Container for angle or rate targets with yaw frame specification
     * 
     * @details Stores roll, pitch, yaw as either angles (radians) or rates (rad/s).
     *          The yaw_is_ef flag determines yaw frame:
     *          - true: Yaw is earth-frame (compass heading, e.g., 0=North, 90=East)
     *          - false: Yaw is body-frame (relative to vehicle nose, rotates with vehicle)
     * 
     * @note Units: All values in radians or rad/s (depending on MountTargetType)
     * @note Frame: Roll/pitch always earth-frame (horizon-relative), yaw frame set by yaw_is_ef
     */
    class MountTarget {
    public:
        float roll;      ///< Roll angle (rad) or rate (rad/s), earth-frame
        float pitch;     ///< Pitch angle (rad) or rate (rad/s), earth-frame
        float yaw;       ///< Yaw angle (rad) or rate (rad/s), frame depends on yaw_is_ef
        bool yaw_is_ef;  ///< true = yaw in earth-frame (compass), false = yaw in body-frame (vehicle-relative)

        /**
         * @brief Get body-frame yaw angle from target
         * 
         * @return Yaw angle in radians, body-frame
         * 
         * @details If yaw_is_ef is true, converts earth-frame yaw to body-frame using vehicle heading.
         *          If yaw_is_ef is false, returns yaw directly (already body-frame).
         */
        float get_bf_yaw() const;

        /**
         * @brief Get earth-frame yaw angle from target
         * 
         * @return Yaw angle in radians, earth-frame (compass heading)
         * 
         * @details If yaw_is_ef is false, converts body-frame yaw to earth-frame using vehicle heading.
         *          If yaw_is_ef is true, returns yaw directly (already earth-frame).
         */
        float get_ef_yaw() const;

        /**
         * @brief Set roll, pitch, yaw from Vector3f
         * 
         * @param[in] rpy Vector containing roll, pitch, yaw values (radians or rad/s)
         * @param[in] yaw_is_ef_in Yaw frame flag (true=earth-frame, false=body-frame)
         * 
         * @details Convenience method to populate MountTarget from Vector3f.
         */
        void set(const Vector3f& rpy, bool yaw_is_ef_in);
    };

    /**
     * @enum Options
     * @brief Mount behavior option flags (bitmask)
     * 
     * @details Configurable options from MNTx_OPTIONS parameter:
     *          - RCTARGETING_LOCK_FROM_PREVMODE: When entering RC_TARGETING mode, maintain
     *            yaw_lock state from previous mode instead of defaulting to follow mode
     *          - NEUTRAL_ON_RC_FS: Move gimbal to neutral position on RC failsafe instead
     *            of holding last commanded position
     */
    enum class Options : uint8_t {
        RCTARGETING_LOCK_FROM_PREVMODE = (1U << 0), ///< RC_TARGETING mode inherits lock/follow from previous mode
        NEUTRAL_ON_RC_FS               = (1U << 1), ///< Move to neutral position on RC failsafe
    };

    /**
     * @brief Check if option flag is set
     * 
     * @param[in] opt Option flag to test
     * @return true if option enabled in MNTx_OPTIONS parameter, false if disabled
     */
    bool option_set(Options opt) const { return (_params.options.get() & (uint8_t)opt) != 0; }

    /**
     * @brief Update mount target from RC pilot input (for RC_TARGETING mode)
     * 
     * @details Reads RC channels (from MNTx_RC_ROLL/PITCH/YAW parameters), converts to
     *          angle or rate targets based on MNTx_TYPE, and updates mnt_target structure.
     *          Applies dead zones, rate limits, and exponential curves. Updates yaw_lock
     *          from RC switch if configured.
     * 
     * @note Called from update() when in RC_TARGETING mode
     * @note Handles both angle and rate control based on gimbal type configuration
     */
    void update_mnt_target_from_rc_target();

    /**
     * @brief Check if roll angle range is valid (user has not disabled roll control)
     * 
     * @return true if ROLL_ANGLE_MIN < ROLL_ANGLE_MAX, false if roll disabled
     * 
     * @details Allows user to disable roll control by setting min >= max, even on 3-axis gimbal.
     */
    bool roll_range_valid() const { return (_params.roll_angle_min < _params.roll_angle_max); }

    /**
     * @brief Check if pitch angle range is valid (user has not disabled pitch control)
     * 
     * @return true if PITCH_ANGLE_MIN < PITCH_ANGLE_MAX, false if pitch disabled
     * 
     * @details Allows user to disable pitch control by setting min >= max, even on 3-axis gimbal.
     */
    bool pitch_range_valid() const { return (_params.pitch_angle_min < _params.pitch_angle_max); }

    /**
     * @brief Check if yaw angle range is valid (user has not disabled yaw control)
     * 
     * @return true if YAW_ANGLE_MIN < YAW_ANGLE_MAX, false if yaw disabled
     * 
     * @details Allows user to disable yaw control by setting min >= max, even on 3-axis gimbal.
     */
    bool yaw_range_valid() const { return (_params.yaw_angle_min < _params.yaw_angle_max); }

    /**
     * @brief Check if MAVLink heartbeat should be suppressed for this gimbal
     * 
     * @return true if heartbeat suppression required, false for normal heartbeat
     * 
     * @details Base implementation returns false. Only Solo gimbal overrides to return true.
     *          Used to avoid confusing GCS with gimbal heartbeats in legacy Solo configuration.
     */
    virtual bool suppress_heartbeat() const { return false; }

    /**
     * @brief Automatically switch to RC_TARGETING mode if RC inputs change
     * 
     * @details Monitors RC input channels. If inputs change by more than dead zone threshold,
     *          automatically switches to RC_TARGETING mode. Allows pilot to take manual control
     *          by moving sticks. Should be called every update() cycle.
     * 
     * @note Requires MNTx_RC_RATE parameter to enable automatic mode switching
     * @note Dead zone prevents jitter from triggering mode changes
     */
    void set_rctargeting_on_rcinput_change();

    /**
     * @brief Calculate angle targets to point at ROI location
     * 
     * @param[out] angle_rad Calculated gimbal angles (radians) to point at ROI
     * @return true on success, false if ROI not set or calculation failed
     * 
     * @details Uses _roi_target location, current vehicle position from AP::ahrs().get_location(),
     *          and get_angle_target_to_location() to calculate required gimbal angles.
     * 
     * @note Used by GPS_POINT mode
     * @note Calculation includes earth-frame to body-frame transformations
     * @warning WARN_IF_UNUSED attribute - caller must check return value
     */
    bool get_angle_target_to_roi(MountTarget& angle_rad) const WARN_IF_UNUSED;

    /**
     * @brief Calculate angle targets to point at home location
     * 
     * @param[out] angle_rad Calculated gimbal angles (radians) to point at home
     * @return true on success, false if home not set or calculation failed
     * 
     * @details Uses home location from AP::ahrs().get_home(), current vehicle position,
     *          and get_angle_target_to_location() to calculate required gimbal angles.
     * 
     * @note Used by HOME_LOCATION mode
     * @note Home must be set (armed at least once) for this to succeed
     * @warning WARN_IF_UNUSED attribute - caller must check return value
     */
    bool get_angle_target_to_home(MountTarget& angle_rad) const WARN_IF_UNUSED;

    /**
     * @brief Calculate angle targets to point at tracked vehicle
     * 
     * @param[out] angle_rad Calculated gimbal angles (radians) to point at target vehicle
     * @return true on success, false if target location not available or calculation failed
     * 
     * @details Uses _target_sysid_location (updated by handle_global_position_int()),
     *          current vehicle position, and get_angle_target_to_location() to calculate
     *          required gimbal angles for tracking another vehicle.
     * 
     * @note Used by SYSID_TARGET mode
     * @note Requires target vehicle sending GLOBAL_POSITION_INT messages
     * @warning WARN_IF_UNUSED attribute - caller must check return value
     */
    bool get_angle_target_to_sysid(MountTarget& angle_rad) const WARN_IF_UNUSED;

    /**
     * @brief Integrate rate target into angle target
     * 
     * @param[in] rate_rad Rate target (rad/s) to integrate
     * @param[in,out] angle_rad Angle target (radians) to update with integrated rate
     * 
     * @details Integrates rate into angle assuming 50Hz update rate (dt = 0.02s).
     *          Resulting angle_rad yaw frame matches rate_rad yaw frame.
     *          Used when mnt_target.target_type is RATE to maintain angle target
     *          for backends that need angle commands.
     * 
     * @note Assumes 50Hz update rate (dt = 0.02s)
     * @note Yaw angle wrapping handled correctly for both earth-frame and body-frame
     */
    void update_angle_target_from_rate(const MountTarget& rate_rad, MountTarget& angle_rad) const;

    /**
     * @brief Generate GIMBAL_DEVICE_FLAGS bitmask for MAVLink messages
     * 
     * @return Bitmask of GIMBAL_DEVICE_FLAGS
     * 
     * @details Generates flags for GIMBAL_DEVICE_ATTITUDE_STATUS message:
     *          - RETRACT: Gimbal in retract mode
     *          - NEUTRAL: Gimbal in neutral mode
     *          - ROLL/PITCH/YAW_LOCK: Axis locked to earth-frame
     *          - ROLL/PITCH/YAW_IN_VEHICLE_FRAME: Axis controlled in body-frame
     * 
     * @note Based on current mode and yaw_lock state
     */
    uint16_t get_gimbal_device_flags() const;

    /**
     * @brief Send warning message to GCS
     * 
     * @param[in] warning_str Warning text to send (null-terminated string)
     * 
     * @details Sends STATUSTEXT message to GCS with MAV_SEVERITY_WARNING. Rate-limited
     *          to avoid flooding GCS (minimum 3 seconds between warnings from _last_warning_ms).
     * 
     * @note Rate limited to one warning per 3 seconds
     * @note Appears in GCS message log
     */
    void send_warning_to_GCS(const char* warning_str);

    AP_Mount    &_frontend; ///< Reference to AP_Mount frontend manager (holds global state and configuration)
    AP_Mount_Params &_params; ///< Parameter storage for this mount instance (MNTx_* parameters)
    uint8_t     _instance;  ///< Mount instance number (0-based, for multi-mount systems)

    MAV_MOUNT_MODE  _mode;  ///< Current operating mode (RETRACT, NEUTRAL, MAVLINK_TARGETING, RC_TARGETING, GPS_POINT, SYSID_TARGET, HOME_LOCATION)

    /**
     * @brief Structure for MAVLINK_TARGETING and RC_TARGETING mode targets
     * 
     * @details Stores either angle targets (radians) or rate targets (rad/s) depending on target_type.
     *          Used by MAVLINK_TARGETING mode from set_angle_target()/set_rate_target() and
     *          RC_TARGETING mode from update_mnt_target_from_rc_target().
     */
    struct {
        MountTargetType target_type; ///< Target type (ANGLE or RATE)
        MountTarget angle_rad;       ///< Angle target in radians (roll, pitch, yaw with yaw frame flag)
        MountTarget rate_rads;       ///< Rate target in rad/s (roll, pitch, yaw with yaw frame flag)
    } mnt_target;

private:

    /**
     * @brief Get pilot RC input in normalized range
     * 
     * @param[out] roll_in Roll input -1 to +1 (left to right)
     * @param[out] pitch_in Pitch input -1 to +1 (down to up)
     * @param[out] yaw_in Yaw input -1 to +1 (left to right)
     * 
     * @details Reads RC channels configured in MNTx_RC_ROLL/PITCH/YAW parameters,
     *          normalizes to -1 to +1 range with center at 0. Returns 0 if channel not configured.
     * 
     * @note Called by get_rc_target() and set_rctargeting_on_rcinput_change()
     */
    void get_rc_input(float& roll_in, float& pitch_in, float& yaw_in) const;

    /**
     * @brief Get angle or rate targets from pilot RC input
     * 
     * @param[out] target_type Set to ANGLE or RATE based on MNTx_TYPE parameter
     * @param[out] rpy Target angles (degrees) or rates (deg/s) from RC input
     * 
     * @details Converts RC input to targets based on gimbal type configuration:
     *          - Type 0 (MAVLink): No RC control
     *          - Type 1 (RC angle): Returns angle targets in degrees
     *          - Type 2 (GPS point): Returns angle targets for GPS pointing
     *          - Type 3 (RC rate): Returns rate targets in deg/s
     * 
     * @note Output units in degrees or deg/s (converted to radians by caller)
     */
    void get_rc_target(MountTargetType& target_type, MountTarget& rpy) const;

    /**
     * @brief Calculate gimbal angles to point at a location
     * 
     * @param[in] loc Target location to point at (lat, lon, alt)
     * @param[out] angle_rad Calculated gimbal angles (radians) to point at location
     * @return true on success, false if calculation failed (no vehicle position)
     * 
     * @details Core calculation for GPS_POINT, SYSID_TARGET, and HOME_LOCATION modes.
     *          Computes bearing and elevation from current vehicle position to target location,
     *          then converts to required gimbal angles accounting for vehicle attitude.
     * 
     * @note Uses AP::ahrs().get_location() for current vehicle position
     * @note Accounts for vehicle roll, pitch, yaw in angle calculation
     * @warning WARN_IF_UNUSED attribute - caller must check return value
     */
    bool get_angle_target_to_location(const Location &loc, MountTarget& angle_rad) const WARN_IF_UNUSED;

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    /**
     * @brief Calculate point-of-interest location (where gimbal is pointing)
     * 
     * @details Runs in separate thread to calculate lat/lon/alt of the point the gimbal
     *          is pointing at. Uses gimbal attitude, vehicle location, and optional
     *          rangefinder distance. Results stored in poi_calculation structure with
     *          semaphore protection. Called periodically when POI calculation enabled.
     * 
     * @note Thread-safe via poi_calculation.sem semaphore
     * @note Results retrieved via get_poi()
     * @note Calculation uses rangefinder distance if available, otherwise projects to terrain
     */
    void calculate_poi();
#endif

    bool _yaw_lock; ///< Yaw lock state for RC_TARGETING mode (true=earth-frame/lock, false=body-frame/follow)

#if AP_MOUNT_POI_TO_LATLONALT_ENABLED
    /**
     * @brief Point-of-interest calculation data (thread-protected)
     * 
     * @details Shared between calculate_poi() (writer) and get_poi() (reader).
     *          Protected by semaphore for thread-safe access.
     */
    struct {
        HAL_Semaphore sem;        ///< Semaphore protecting this structure for thread-safe access
        uint32_t poi_request_ms;  ///< System time (ms) when POI calculation was last requested
        uint32_t poi_update_ms;   ///< System time (ms) when POI calculation was last completed
        Location loc;             ///< Vehicle/gimbal location used for POI calculation
        Location poi_loc;         ///< Calculated POI location (lat/lon/alt where gimbal points)
        Quaternion att_quat;      ///< Gimbal attitude quaternion used for POI calculation
    } poi_calculation;
#endif

    Location _roi_target;           ///< Region of Interest target location (for GPS_POINT mode)
    bool _roi_target_set;           ///< True if ROI target has been set by set_roi_target()

    uint8_t _target_sysid;          ///< MAVLink system ID to track (for SYSID_TARGET mode)
    Location _target_sysid_location;///< Last known location of tracked vehicle
    bool _target_sysid_location_set;///< True if target vehicle location has been received

    uint32_t _last_warning_ms;      ///< System time (ms) of last warning sent to GCS (for rate limiting)

    /**
     * @brief Last RC input values for change detection
     * 
     * @details Stores previous RC input to detect when pilot moves sticks.
     *          Used by set_rctargeting_on_rcinput_change() to auto-switch to RC_TARGETING mode.
     */
    struct {
        bool    initialised;  ///< True after first RC input read
        int16_t roll_in;      ///< Last roll RC input (raw PWM value)
        int16_t pitch_in;     ///< Last pitch RC input (raw PWM value)
        int16_t yaw_in;       ///< Last yaw RC input (raw PWM value)
    } last_rc_input;

    /**
     * @brief MAVLink gimbal manager control authority
     * 
     * @details Stores sysid/compid of GCS that has control authority over this gimbal.
     *          Set by DO_GIMBAL_MANAGER_CONFIGURE command. Used to validate
     *          GIMBAL_MANAGER_SET_ATTITUDE commands from authorized controller only.
     * 
     * @see handle_command_do_gimbal_manager_configure()
     * @see send_gimbal_manager_status()
     */
    struct mavlink_control_id_t {
        uint8_t sysid;   ///< MAVLink system ID of authorized controller (0=no controller)
        uint8_t compid;  ///< MAVLink component ID of authorized controller

        /// Check if two control IDs are equal
        bool operator==(const mavlink_control_id_t &rhs) const { return (sysid == rhs.sysid && compid == rhs.compid); }
        /// Check if two control IDs are not equal
        bool operator!=(const mavlink_control_id_t &rhs) const { return !(*this == rhs); }
    } mavlink_control_id; ///< Current gimbal manager control authority
};

#endif // HAL_MOUNT_ENABLED
