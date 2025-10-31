/**
 * @file AP_Mount_Params.h
 * @brief Parameter management for camera mount and gimbal control systems
 * 
 * @details This file defines the AP_Mount_Params class which manages all configurable
 *          parameters for camera mounts and gimbals. Parameters control mount type selection,
 *          operational modes, control rate limits, angle constraints, default positions,
 *          and stabilization behavior. These parameters are persisted using the AP_Param
 *          system and are accessible via ground control stations for configuration.
 *          
 *          The parameter system supports multiple mount instances, each with independent
 *          configuration. Parameters are registered through the var_info[] table which
 *          defines parameter metadata including names, types, defaults, and valid ranges.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

/**
 * @class AP_Mount_Params
 * @brief Parameter storage and management for a single camera mount/gimbal instance
 * 
 * @details This class encapsulates all configurable parameters for one mount/gimbal instance.
 *          Multiple instances of this class can exist to support multiple independent mounts
 *          on a single vehicle.
 *          
 *          The class uses the AP_Param system for parameter storage, persistence, and access
 *          through ground control stations. The var_info[] table defines the parameter group
 *          structure that integrates with AP_Param's registration system.
 *          
 *          Parameters control:
 *          - Mount hardware type and backend selection
 *          - Default operational mode at startup
 *          - Pilot control rate limits for manual operation
 *          - Mechanical angle limits for each axis (roll, pitch, yaw)
 *          - Retract and neutral position definitions
 *          - Stabilization lead control gains (for servo backends)
 *          - Target system identification for follow modes
 *          - Device identification and option flags
 *          
 * @note All angle parameters are stored in degrees. Rate parameters are in degrees/second.
 * @note Parameters are persistent across power cycles via AP_Param storage mechanisms.
 * @warning Angle limit parameters must respect mechanical constraints of the physical gimbal
 *          to prevent damage. Verify limits match hardware specifications.
 */
class AP_Mount_Params {

public:

    /**
     * @brief Parameter group information table for AP_Param registration
     * 
     * @details This table defines all parameters in this group for the AP_Param system,
     *          including parameter names, types, default values, and metadata. Used by
     *          AP_Param to enable parameter storage, retrieval, and ground station access.
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Constructor for mount parameter instance
     * 
     * @details Initializes parameter object. Actual parameter values are loaded from
     *          persistent storage by the AP_Param system after construction.
     */
    AP_Mount_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Mount_Params);

    /**
     * @brief Mount hardware type and backend selection
     * 
     * @details Specifies the mount hardware type which determines which backend driver
     *          is used for control. Options include servo-based, MAVLink-controlled,
     *          SToRM32, Gremsy, Siyi, and other gimbal protocols.
     *          
     * @note Value corresponds to Mount::Type enumeration
     * @see AP_Mount::Type for available mount types
     */
    AP_Int8     type;

    /**
     * @brief Default operational mode for mount control
     * 
     * @details Defines the mode the mount enters on startup and when autopilot returns
     *          control from pilot or mission commands. Modes include retracted, neutral,
     *          MAVLink targeting, RC targeting, GPS point, and various tracking modes.
     *          
     * @note Value corresponds to MAV_MOUNT_MODE enumeration from MAVLink protocol
     * @see MAV_MOUNT_MODE for available operational modes
     */
    AP_Int8     default_mode;
    /**
     * @brief Maximum rate for pilot RC rate control mode
     * 
     * @details Defines the maximum angular rate (degrees/second) when pilot controls
     *          the mount in rate control mode using RC inputs. Set to zero to disable
     *          rate control and use angle control mode instead.
     *          
     * @note Units: degrees/second
     * @note Typical range: 0-90 deg/s. Zero disables rate control.
     */
    AP_Int16    rc_rate_max;
    /**
     * @brief Minimum roll angle limit
     * 
     * @details Defines the minimum (most negative) roll angle the mount can achieve.
     *          This limit prevents commanding the mount beyond its mechanical constraints
     *          and protects the gimbal hardware from damage.
     *          
     * @note Units: degrees
     * @note Typical range: -180 to 0 degrees
     * @warning Must match physical gimbal mechanical limits to prevent hardware damage
     */
    AP_Int16    roll_angle_min;

    /**
     * @brief Maximum roll angle limit
     * 
     * @details Defines the maximum (most positive) roll angle the mount can achieve.
     *          This limit prevents commanding the mount beyond its mechanical constraints
     *          and protects the gimbal hardware from damage.
     *          
     * @note Units: degrees
     * @note Typical range: 0 to 180 degrees
     * @warning Must match physical gimbal mechanical limits to prevent hardware damage
     */
    AP_Int16    roll_angle_max;

    /**
     * @brief Minimum pitch angle limit
     * 
     * @details Defines the minimum (most negative) pitch angle the mount can achieve.
     *          This limit prevents commanding the mount beyond its mechanical constraints.
     *          For camera gimbals, negative pitch typically points the camera downward.
     *          
     * @note Units: degrees
     * @note Typical range: -180 to 0 degrees. -90 is straight down.
     * @warning Must match physical gimbal mechanical limits to prevent hardware damage
     */
    AP_Int16    pitch_angle_min;

    /**
     * @brief Maximum pitch angle limit
     * 
     * @details Defines the maximum (most positive) pitch angle the mount can achieve.
     *          This limit prevents commanding the mount beyond its mechanical constraints.
     *          For camera gimbals, positive pitch typically points the camera upward.
     *          
     * @note Units: degrees
     * @note Typical range: 0 to 180 degrees. 90 is straight up.
     * @warning Must match physical gimbal mechanical limits to prevent hardware damage
     */
    AP_Int16    pitch_angle_max;

    /**
     * @brief Minimum yaw angle limit
     * 
     * @details Defines the minimum (most negative) yaw angle the mount can achieve
     *          relative to vehicle heading. This limit prevents commanding the mount
     *          beyond its mechanical constraints and cable wrap limits.
     *          
     * @note Units: degrees
     * @note Typical range: -180 to 0 degrees
     * @warning Must respect mechanical stops and cable routing to prevent damage
     */
    AP_Int16    yaw_angle_min;

    /**
     * @brief Maximum yaw angle limit
     * 
     * @details Defines the maximum (most positive) yaw angle the mount can achieve
     *          relative to vehicle heading. This limit prevents commanding the mount
     *          beyond its mechanical constraints and cable wrap limits.
     *          
     * @note Units: degrees
     * @note Typical range: 0 to 180 degrees
     * @warning Must respect mechanical stops and cable routing to prevent damage
     */
    AP_Int16    yaw_angle_max;

    /**
     * @brief Retracted position angles for all three axes
     * 
     * @details Defines the mount orientation when in retracted mode, typically used
     *          to position the camera in a safe, protected position during takeoff,
     *          landing, or transport. The mount moves to this position when retract
     *          mode is commanded.
     *          
     * @note Units: degrees
     * @note Vector components: x=roll, y=pitch, z=yaw
     * @note All angles relative to vehicle body frame
     */
    AP_Vector3f retract_angles;

    /**
     * @brief Neutral position angles for all three axes
     * 
     * @details Defines the mount orientation when in neutral mode, typically a
     *          forward-facing or default operational position. The mount moves to
     *          this position when neutral mode is commanded or as a startup default.
     *          
     * @note Units: degrees
     * @note Vector components: x=roll, y=pitch, z=yaw
     * @note All angles relative to vehicle body frame
     * @note Common neutral: roll=0, pitch=0, yaw=0 (forward facing, level)
     */
    AP_Vector3f neutral_angles;

    /**
     * @brief Roll axis stabilization lead control gain
     * 
     * @details Lead compensation gain for roll axis stabilization. Used only by
     *          servo-based mount backends to improve stabilization response by
     *          compensating for servo lag and mechanical dynamics. Higher values
     *          provide more aggressive compensation but may cause oscillation.
     *          
     * @note Only applicable to servo backend (AP_Mount_Servo)
     * @note Typical range: 0.0 to 1.0. Start with 0.0 and increase if needed.
     * @note Units: dimensionless gain factor
     */
    AP_Float    roll_stb_lead;

    /**
     * @brief Pitch axis stabilization lead control gain
     * 
     * @details Lead compensation gain for pitch axis stabilization. Used only by
     *          servo-based mount backends to improve stabilization response by
     *          compensating for servo lag and mechanical dynamics. Higher values
     *          provide more aggressive compensation but may cause oscillation.
     *          
     * @note Only applicable to servo backend (AP_Mount_Servo)
     * @note Typical range: 0.0 to 1.0. Start with 0.0 and increase if needed.
     * @note Units: dimensionless gain factor
     */
    AP_Float    pitch_stb_lead;

    /**
     * @brief Default target system ID for mount follow modes
     * 
     * @details Specifies the MAVLink system ID that the mount should track in follow
     *          modes. Used when mount is configured to track another vehicle or system.
     *          A value of 0 means follow the home vehicle/GCS.
     *          
     * @note MAVLink system ID (1-255)
     * @note 0 = follow home vehicle, >0 = follow specific system ID
     */
    AP_Int8     sysid_default;

    /**
     * @brief Device identification number including bus information
     * 
     * @details Unique device identifier that incorporates both the device type and
     *          the communication bus it's connected to. Used for device detection,
     *          identification, and to support multiple instances of the same hardware.
     *          Automatically populated during device probing.
     *          
     * @note Auto-populated during backend initialization
     * @note Used for device health monitoring and logging
     */
    AP_Int32    dev_id;

    /**
     * @brief Mount configuration options bitmask
     * 
     * @details Bitmask of option flags that modify mount behavior. Options may include
     *          reversing servo directions, enabling/disabling specific features, or
     *          selecting alternative control modes. Specific bit definitions are
     *          backend-dependent.
     *          
     * @note Bitmask field - multiple options can be enabled simultaneously
     * @note Bit definitions vary by mount backend type
     * @see AP_Mount backend implementation for specific option flags
     */
    AP_Int8     options;
};
