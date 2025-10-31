/**
 * @file AP_Mount_Scripting.h
 * @brief Lua scripting backend for custom gimbal control
 * 
 * @details This file implements a mount/gimbal driver backend that can be controlled
 *          via Lua scripting. It allows developers to implement custom gimbal drivers
 *          entirely in Lua scripts, providing flexibility for non-standard hardware
 *          or specialized control algorithms without modifying C++ code.
 * 
 *          The scripting backend exposes two key interfaces:
 *          - get_location_target(): Retrieves the current gimbal pointing target location (ROI)
 *          - set_attitude_euler(): Allows the Lua script to update the gimbal's actual attitude
 * 
 *          This enables a control loop where the script reads the desired target,
 *          computes appropriate gimbal angles (possibly with custom algorithms),
 *          and updates the mount backend with the actual gimbal position.
 * 
 * @note Typical usage pattern:
 *       1. Mount backend requests pointing at a location or angle
 *       2. Lua script calls get_location_target() or accesses target angles
 *       3. Script calculates required gimbal movement (custom control logic)
 *       4. Script sends commands to physical gimbal hardware
 *       5. Script calls set_attitude_euler() with actual gimbal position
 *       6. Repeat periodically to maintain gimbal pointing
 * 
 * @see AP_Scripting library documentation for Lua binding examples
 * @see AP_Mount_Backend for base mount driver interface
 */

#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SCRIPTING_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

/**
 * @class AP_Mount_Scripting
 * @brief Scripting interface for custom gimbal/mount control via Lua
 * 
 * @details AP_Mount_Scripting provides a bridge between ArduPilot's mount control
 *          system and Lua scripting, enabling custom gimbal drivers to be implemented
 *          entirely in Lua without C++ code modifications. This is particularly useful
 *          for prototype hardware, specialized control algorithms, or integrating
 *          non-standard gimbals that don't fit existing driver patterns.
 * 
 *          The class inherits from AP_Mount_Backend and implements the standard mount
 *          interface, but delegates actual hardware control to a Lua script through
 *          the scripting bindings.
 * 
 *          **Key Features:**
 *          - Full access to mount targeting modes (location, angle, rate, home)
 *          - Custom control algorithms implementable in Lua
 *          - Health monitoring via periodic script updates
 *          - Support for ROI (Region of Interest) targeting
 *          - Euler angle-based attitude reporting
 * 
 *          **Scripting Integration:**
 *          The Lua script must periodically call set_attitude_euler() to report the
 *          gimbal's actual position and maintain health status. Failure to call this
 *          method results in the mount being marked unhealthy after a timeout period.
 * 
 *          **Usage Pattern from Lua:**
 *          @code{.lua}
 *          -- Example Lua script for scripting mount control
 *          local mount_instance = 0  -- First mount
 *          
 *          function update()
 *              -- Get current target location if in ROI mode
 *              local target_loc = mount:get_location_target(mount_instance)
 *              
 *              -- Compute gimbal angles based on vehicle position and target
 *              -- (custom calculation logic here)
 *              local roll_deg = 0
 *              local pitch_deg = calculate_pitch_to_target(target_loc)
 *              local yaw_deg = calculate_yaw_to_target(target_loc)
 *              
 *              -- Send commands to physical gimbal hardware
 *              send_to_gimbal_hardware(roll_deg, pitch_deg, yaw_deg)
 *              
 *              -- Report actual gimbal position back to mount backend
 *              mount:set_attitude_euler(mount_instance, roll_deg, pitch_deg, yaw_deg)
 *              
 *              return update, 100  -- Reschedule in 100ms
 *          end
 *          
 *          return update()
 *          @endcode
 * 
 * @note Health reporting: The mount is considered healthy if set_attitude_euler()
 *       has been called within the timeout period (typically a few seconds). This
 *       ensures the Lua script is actively running and controlling the gimbal.
 * 
 * @note Thread safety: All methods are called from the scheduler thread context.
 *       Lua scripts run in the scripting thread, and the scripting backend handles
 *       thread-safe communication between contexts.
 * 
 * @warning The Lua script is responsible for all hardware communication and error
 *          handling. The C++ backend only tracks targeting requests and gimbal state.
 * 
 * @see AP_Scripting for Lua scripting infrastructure
 * @see AP_Mount_Backend for base mount interface documentation
 * @see examples/SCR_mount_custom.lua for example implementation
 */
class AP_Mount_Scripting : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Scripting);

    /**
     * @brief Update mount position - should be called periodically
     * 
     * @details This method is called periodically by the mount driver scheduler
     *          to update the gimbal's state and process targeting requests.
     *          For the scripting backend, this primarily handles:
     *          - Checking mount mode changes
     *          - Processing target updates from flight controller
     *          - Updating internal state for script access
     * 
     *          The actual gimbal control and movement is handled by the Lua script,
     *          not by this update method. This method prepares the targeting
     *          information that the script will read via get_location_target()
     *          or similar accessors.
     * 
     * @note Called at mount driver update rate (typically 10-50 Hz depending on scheduler load)
     * @note Does not directly control hardware - that's the script's responsibility
     */
    void update() override;

    /**
     * @brief Return true if mount is healthy
     * 
     * @details Health status for the scripting backend is determined by whether
     *          the Lua script has recently called set_attitude_euler() to report
     *          the gimbal's current position. If set_attitude_euler() hasn't been
     *          called within the timeout period, the mount is considered unhealthy,
     *          indicating the script may have stopped running or encountered an error.
     * 
     *          Health timeout period is typically 2-3 seconds, allowing for
     *          temporary script delays without triggering false unhealthy status.
     * 
     * @return true if Lua script has recently updated gimbal attitude (within timeout)
     * @return false if no recent updates from script (timeout expired)
     * 
     * @note The Lua script must call set_attitude_euler() periodically to maintain
     *       healthy status, even if gimbal position hasn't changed
     * @note Unhealthy status may trigger mount failsafe actions depending on configuration
     * 
     * @see set_attitude_euler() for method that updates health timestamp
     */
    bool healthy() const override;

    /**
     * @brief Check if this mount can control its pan/yaw axis
     * 
     * @details Returns true if the gimbal has yaw control capability, which is
     *          required for multicopters to properly implement ROI (Region of Interest)
     *          tracking. Multicopters need to know if the gimbal can pan independently
     *          or if the vehicle must yaw to point at the target.
     * 
     *          For the scripting backend, this queries whether a valid yaw range
     *          has been configured in the mount parameters, indicating the gimbal
     *          has yaw/pan capability.
     * 
     * @return true if yaw range is configured (gimbal can control pan)
     * @return false if no yaw range configured (vehicle must yaw for pointing)
     * 
     * @note Critical for multicopter ROI tracking behavior
     * @note Fixed-wing aircraft typically don't need independent pan control
     * 
     * @see yaw_range_valid() in AP_Mount_Backend for range validation logic
     */
    bool has_pan_control() const override { return yaw_range_valid(); };

    // Scripting backend accessors
    
    /**
     * @brief Get the target location for gimbal pointing (ROI mode)
     * 
     * @details Retrieves the current location target that the gimbal should point at,
     *          typically set via MAVLink DO_SET_ROI command or similar targeting commands.
     *          This allows the Lua script to access the desired pointing location and
     *          compute appropriate gimbal angles to aim at that position.
     * 
     *          The target location is valid when the mount is in location targeting mode
     *          (e.g., ROI mode). In other modes (angle, rate, home, etc.), this method
     *          returns false and the target location is invalid.
     * 
     *          **Typical Script Usage:**
     *          @code{.lua}
     *          local target = mount:get_location_target(mount_instance)
     *          if target then
     *              -- Valid target location received
     *              local vehicle_loc = ahrs:get_position()
     *              local pitch = calculate_pitch_angle(vehicle_loc, target)
     *              local yaw = calculate_yaw_angle(vehicle_loc, target)
     *              -- Send to gimbal and report back
     *              mount:set_attitude_euler(mount_instance, 0, pitch, yaw)
     *          end
     *          @endcode
     * 
     * @param[out] _target_loc Location object to be filled with target coordinates
     *                         (latitude, longitude, altitude)
     * 
     * @return true if valid target location is available (mount in location tracking mode)
     * @return false if no valid target (mount in angle/rate mode or target not set)
     * 
     * @note Target location is in global coordinates (lat/lon/alt)
     * @note Lua script must calculate gimbal angles from vehicle position to target
     * @note Called from Lua scripting thread context
     * 
     * @see set_attitude_euler() to report computed gimbal position
     * @see AP_Mount_Backend targeting modes for different control methods
     */
    bool get_location_target(Location& _target_loc) override;
    
    /**
     * @brief Set gimbal attitude in Euler angles (called by Lua script)
     * 
     * @details This method is called by the Lua script to report the gimbal's actual
     *          current attitude after the script has commanded the physical hardware.
     *          It serves two critical purposes:
     *          1. Updates the mount backend's knowledge of gimbal position for telemetry
     *          2. Resets the health timeout, marking the mount as healthy
     * 
     *          The Lua script should call this method periodically (typically 10-50 Hz)
     *          with the actual gimbal position read from hardware encoders or commanded
     *          position if no feedback is available.
     * 
     *          **Attitude Convention:**
     *          - Roll: Rotation about forward axis (typically limited to ±45°)
     *          - Pitch: Rotation about right axis (typically -90° to +45°, down is negative)
     *          - Yaw: Rotation about down axis in **body frame** (0° = forward, 90° = right)
     * 
     *          **Typical Script Usage:**
     *          @code{.lua}
     *          -- After sending commands to physical gimbal hardware
     *          local actual_roll = read_gimbal_roll()    -- Read from hardware
     *          local actual_pitch = read_gimbal_pitch()
     *          local actual_yaw = read_gimbal_yaw()
     *          
     *          -- Report actual position to mount backend
     *          mount:set_attitude_euler(mount_instance, actual_roll, actual_pitch, actual_yaw)
     *          @endcode
     * 
     * @param[in] roll_deg   Gimbal roll angle in degrees
     *                       Positive = roll right, negative = roll left
     *                       Typical range: -45° to +45°
     * 
     * @param[in] pitch_deg  Gimbal pitch angle in degrees
     *                       Positive = pitch up, negative = pitch down
     *                       Typical range: -90° (straight down) to +45° (up)
     * 
     * @param[in] yaw_bf_deg Gimbal yaw angle in body frame degrees
     *                       0° = pointing forward (vehicle nose direction)
     *                       90° = pointing right, -90° = pointing left
     *                       180°/-180° = pointing backward
     *                       Frame: Body frame (relative to vehicle orientation)
     * 
     * @note MUST be called periodically by Lua script to maintain health status
     * @note Updates last_update_ms timestamp for health monitoring
     * @note Angles stored in current_angle_deg for telemetry reporting
     * @note Called from Lua scripting thread context
     * @note Update rate recommendation: 10-50 Hz for smooth telemetry and health monitoring
     * 
     * @warning Failure to call this method regularly will result in mount being marked
     *          unhealthy after timeout period (typically 2-3 seconds)
     * 
     * @see healthy() for health status checking based on update timing
     * @see get_attitude_quaternion() for quaternion representation of attitude
     */
    void set_attitude_euler(float roll_deg, float pitch_deg, float yaw_bf_deg) override;

protected:

    /**
     * @brief Get gimbal attitude as a quaternion
     * 
     * @details Converts the stored Euler angle representation (roll, pitch, yaw)
     *          to a quaternion representation for use by the mount control system.
     *          This method is called internally by the mount backend when quaternion
     *          representation is needed for calculations or telemetry.
     * 
     *          The quaternion is constructed from the current_angle_deg vector,
     *          which is updated by the Lua script via set_attitude_euler().
     * 
     *          **Quaternion Convention:**
     *          - Represents rotation from vehicle body frame to gimbal frame
     *          - Follows ArduPilot's right-hand coordinate system convention
     *          - Normalized quaternion (magnitude = 1.0)
     * 
     * @param[out] att_quat Quaternion object to be filled with gimbal attitude
     *                      Format: w, x, y, z components representing the rotation
     *                      from body frame to gimbal frame
     * 
     * @return true if attitude is valid and quaternion successfully computed
     * @return false if gimbal attitude is invalid or not initialized
     * 
     * @note Converts from Euler angles stored in current_angle_deg
     * @note Used internally by mount backend for various calculations
     * @note Quaternion avoids gimbal lock issues present in Euler representations
     * 
     * @see set_attitude_euler() for how Euler angles are set by script
     * @see AP_Math quaternion utilities for conversion implementation
     */
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // Internal state variables
    
    /**
     * @brief System time of last attitude update from Lua script
     * 
     * Stores the timestamp (in milliseconds since system boot) when the Lua script
     * last called set_attitude_euler(). Used for health monitoring - if this timestamp
     * is too old (typically >2-3 seconds), the mount is marked unhealthy.
     * 
     * Updated by: set_attitude_euler()
     * Read by: healthy()
     */
    uint32_t last_update_ms;
    
    /**
     * @brief Current gimbal attitude in Euler angles
     * 
     * Stores the most recent gimbal attitude reported by the Lua script via
     * set_attitude_euler(). Components:
     * - x: Roll angle in degrees (rotation about forward axis)
     * - y: Pitch angle in degrees (rotation about right axis, down is negative)
     * - z: Yaw angle in degrees, body frame (0° = forward)
     * 
     * This vector is used for:
     * - Telemetry reporting to ground station
     * - Converting to quaternion representation
     * - Gimbal state tracking
     * 
     * Updated by: set_attitude_euler()
     * Read by: get_attitude_quaternion() and telemetry functions
     */
    Vector3f current_angle_deg;

    /**
     * @brief Target location for gimbal pointing (ROI mode)
     * 
     * Stores the geographic location (latitude, longitude, altitude) that the
     * gimbal should point toward when in location tracking mode. Set by mount
     * control commands (e.g., MAVLink DO_SET_ROI) and retrieved by Lua scripts
     * via get_location_target().
     * 
     * Only valid when target_loc_valid is true.
     * 
     * Updated by: Mount backend targeting commands
     * Read by: get_location_target()
     */
    Location target_loc;
    
    /**
     * @brief Validity flag for target_loc
     * 
     * True if target_loc contains a valid target location that the script should
     * point the gimbal toward. False if mount is in angle/rate mode or no target
     * has been set.
     * 
     * Updated by: Mount backend when target changes or mode switches
     * Read by: get_location_target()
     */
    bool target_loc_valid;
};

#endif // HAL_MOUNT_SCRIPTING_ENABLED
