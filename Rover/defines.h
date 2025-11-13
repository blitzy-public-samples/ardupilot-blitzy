/**
 * @file defines.h
 * @brief Internal constants, enumerations, and bitmask definitions for ArduRover
 * 
 * @details This file contains core definitions used throughout the Rover vehicle code:
 * - Failsafe event type flags for radio and GCS telemetry loss
 * - Custom logging message identifiers and bitmask configuration
 * - MAVLink external control message masks for SET_POSITION_TARGET and SET_ATTITUDE_TARGET
 * - Failsafe configuration enumerations (radio, GCS, crash detection, EKF)
 * - Frame type classifications (rover, boat, balance bot)
 * - Pilot steering control options
 * 
 * @warning These are internal constants that should NOT be edited for custom builds.
 *          Use config.h for build-specific customizations instead.
 * 
 * @note This file is specific to ground vehicle (Rover) operations and is not shared
 *       with other vehicle types (Copter, Plane, Sub).
 */

#pragma once

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

/**
 * @def SERVO_MAX
 * @brief Maximum servo travel representation in centidegrees
 * 
 * @details This value represents 45 degrees (4500 centidegrees) and serves as an
 *          arbitrary normalization constant for servo travel calculations. Used
 *          throughout the rover steering code to convert between normalized steering
 *          commands (-1.0 to 1.0) and servo output values.
 * 
 * @note Value is in centidegrees (1 degree = 100 centidegrees)
 * @note Does not represent actual physical servo limits - those are configured
 *       per-channel via SERVOx_MIN and SERVOx_MAX parameters
 */
#define SERVO_MAX 4500.0  // This value represents 45 degrees and is just an arbitrary representation of servo max travel.

/**
 * @name Failsafe Event Flags
 * @brief Bitmask flags identifying which failsafe event triggered
 * 
 * @details These flags are used in the Rover failsafe system to track which
 *          type of failsafe condition has occurred. Multiple failsafes can be
 *          active simultaneously, represented as a bitmask.
 * @{
 */

/** @brief Radio/RC failsafe triggered by throttle signal loss (bit 0) */
#define FAILSAFE_EVENT_THROTTLE (1<<0)

/** @brief Ground Control Station telemetry link lost (bit 1) */
#define FAILSAFE_EVENT_GCS      (1<<1)

/** @} */ // end of Failsafe Event Flags

/**
 * @enum LoggingParameters
 * @brief Custom Rover-specific log message identifiers
 * 
 * @details ArduPilot allows each vehicle type to define up to 32 custom log message
 *          types beyond the common messages shared across all vehicles. These enums
 *          identify Rover-specific telemetry data structures logged to onboard storage.
 * 
 * @note These message IDs are used internally by AP_Logger to identify message types
 *       in the binary log format (.bin files)
 * @note Maximum of 32 vehicle-specific message types available
 */
enum LoggingParameters {
    LOG_THR_MSG,           ///< Throttle output and control data
    LOG_NTUN_MSG,          ///< Navigation tuning: desired vs achieved speed, heading errors
    LOG_STEERING_MSG,      ///< Steering control: commanded angle, rate, and servo outputs
    LOG_GUIDEDTARGET_MSG,  ///< Guided mode target: position and velocity goals from GCS
};

/**
 * @name Logging Bitmask Defines
 * @brief Bitmask values for LOG_BITMASK parameter to enable specific data logging
 * 
 * @details These bitmask flags control which data streams are logged to onboard storage.
 *          Multiple streams can be enabled by bitwise OR-ing the desired masks together.
 *          The LOG_BITMASK parameter accepts the combined value to configure logging.
 * 
 * @note Enabling more log streams increases storage usage and write frequency
 * @note Some log types have different rates (FAST vs MED) for performance tuning
 * @{
 */

/** @brief Log attitude data (roll, pitch, yaw) at fast rate (~50Hz) */
#define MASK_LOG_ATTITUDE_FAST  (1<<0)

/** @brief Log attitude data at medium rate (~10Hz) for reduced storage usage */
#define MASK_LOG_ATTITUDE_MED   (1<<1)

/** @brief Log GPS position, velocity, and fix quality data */
#define MASK_LOG_GPS            (1<<2)

/** @brief Log performance monitoring data: loop timing, CPU load, memory usage */
#define MASK_LOG_PM             (1<<3)

/** @brief Log throttle output and control (see LOG_THR_MSG) */
#define MASK_LOG_THR            (1<<4)

/** @brief Log navigation tuning data (see LOG_NTUN_MSG) */
#define MASK_LOG_NTUN           (1<<5)

//#define MASK_LOG_MODE         (1<<6) // Deprecated: mode changes now logged automatically

/** @brief Log IMU data: accelerometer and gyroscope measurements after filtering */
#define MASK_LOG_IMU            (1<<7)

/** @brief Log mission command execution and waypoint progress */
#define MASK_LOG_CMD            (1<<8)

/** @brief Log battery current, voltage, and power consumption */
#define MASK_LOG_CURRENT        (1<<9)

/** @brief Log rangefinder distance measurements for obstacle detection */
#define MASK_LOG_RANGEFINDER    (1<<10)

/** @brief Log magnetometer (compass) measurements and heading data */
#define MASK_LOG_COMPASS        (1<<11)

/** @brief Log camera trigger events and shutter timing */
#define MASK_LOG_CAMERA         (1<<12)

/** @brief Log steering control data (see LOG_STEERING_MSG) */
#define MASK_LOG_STEERING       (1<<13)

/** @brief Log RC radio input channels and signal quality */
#define MASK_LOG_RC             (1<<14)

// #define MASK_LOG_ARM_DISARM     (1<<15) // Deprecated: arm/disarm now logged automatically

/** @brief Log raw IMU data: unfiltered accelerometer and gyroscope at full rate */
#define MASK_LOG_IMU_RAW        (1UL<<19)

/** @brief Log video stabilization gimbal control outputs */
#define MASK_LOG_VIDEO_STABILISATION (1UL<<20)

/** @brief Log optical flow sensor data for velocity estimation */
#define MASK_LOG_OPTFLOW                (1UL<<21)

/** @} */ // end of Logging Bitmask Defines

/**
 * @name MAVLink SET_POSITION_TARGET Type Masks
 * @brief Ignore flags for MAVLink SET_POSITION_TARGET_LOCAL_NED and SET_POSITION_TARGET_GLOBAL_INT messages
 * 
 * @details These masks control which fields in external position/velocity commands are
 *          processed versus ignored. Used for guided mode and external control via
 *          companion computers or ground stations. Setting a bit to 1 causes that
 *          field to be ignored.
 * 
 * @note These correspond to the type_mask field in MAVLink position target messages
 * @note Multiple masks can be combined with bitwise OR
 * @{
 */

/** @brief Ignore position fields (X and Y / Lat and Lon) in position target message */
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1))

/** @brief Ignore velocity fields (VX and VY) in position target message */
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4))

/** @brief Ignore acceleration fields (AFX and AFY) in position target message */
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7))

/** @brief Interpret accelerations as force commands instead of acceleration setpoints */
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<9)

/** @brief Ignore yaw angle field in position target message */
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<10)

/** @brief Ignore yaw rate field in position target message */
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<11)

/** @} */ // end of MAVLink SET_POSITION_TARGET Type Masks

/**
 * @name MAVLink SET_ATTITUDE_TARGET Type Masks
 * @brief Ignore flags for MAVLink SET_ATTITUDE_TARGET messages
 * 
 * @details These masks control which fields in external attitude commands are processed
 *          versus ignored. Used for external control of vehicle attitude via companion
 *          computers. Setting a bit to 1 causes that field to be ignored.
 * 
 * @note For ground vehicles, roll and pitch rate control is typically not used
 * @note Yaw rate control affects steering/heading for rovers
 * @{
 */

/** @brief Ignore roll rate command in attitude target message */
#define MAVLINK_SET_ATT_TYPE_MASK_ROLL_RATE_IGNORE     (1<<0)

/** @brief Ignore pitch rate command in attitude target message */
#define MAVLINK_SET_ATT_TYPE_MASK_PITCH_RATE_IGNORE    (1<<1)

/** @brief Ignore yaw rate command in attitude target message */
#define MAVLINK_SET_ATT_TYPE_MASK_YAW_RATE_IGNORE      (1<<2)

/** @brief Ignore throttle command in attitude target message */
#define MAVLINK_SET_ATT_TYPE_MASK_THROTTLE_IGNORE      (1<<6)

/** @brief Ignore attitude quaternion in attitude target message, use rates only */
#define MAVLINK_SET_ATT_TYPE_MASK_ATTITUDE_IGNORE      (1<<7)

/** @} */ // end of MAVLink SET_ATTITUDE_TARGET Type Masks

/**
 * @enum fs_thr_enable
 * @brief Radio/throttle failsafe configuration options
 * 
 * @details Controls vehicle behavior when radio signal from transmitter is lost.
 *          Configured via FS_THR_ENABLE parameter. Failsafe is triggered when
 *          throttle PWM input drops below FS_THR_VALUE for more than FS_TIMEOUT seconds.
 * 
 * @warning DISABLED mode means vehicle will NOT respond to radio signal loss,
 *          potentially continuing uncontrolled operation. Recommended for testing only.
 * 
 * @note Failsafe action (HOLD vs RTL) is configured separately via FS_ACTION parameter
 */
enum fs_thr_enable {
    FS_THR_DISABLED = 0,                 ///< Radio failsafe disabled - no action on signal loss
    FS_THR_ENABLED,                      ///< Radio failsafe enabled - trigger configured FS_ACTION
    FS_THR_ENABLED_CONTINUE_MISSION,     ///< Continue mission on signal loss instead of FS_ACTION
};

/**
 * @enum fs_gcs_enable
 * @brief Ground Control Station telemetry failsafe configuration options
 * 
 * @details Controls vehicle behavior when MAVLink telemetry connection to ground control
 *          station is lost. Configured via FS_GCS_ENABLE parameter. Failsafe triggers
 *          when no MAVLink heartbeat received for FS_TIMEOUT seconds.
 * 
 * @warning DISABLED mode means vehicle will NOT respond to telemetry loss during
 *          autonomous missions, potentially continuing without operator awareness.
 * 
 * @note This failsafe is particularly important for long-range operations where
 *       telemetry may be the only link to the vehicle
 * @note Failsafe action (HOLD vs RTL) is configured separately via FS_ACTION parameter
 */
enum fs_gcs_enable {
    FS_GCS_DISABLED = 0,                 ///< GCS failsafe disabled - no action on telemetry loss
    FS_GCS_ENABLED,                      ///< GCS failsafe enabled - trigger configured FS_ACTION
    FS_GCS_ENABLED_CONTINUE_MISSION,     ///< Continue mission on telemetry loss instead of FS_ACTION
};

/**
 * @enum fs_crash_action
 * @brief Crash detection response configuration
 * 
 * @details Controls vehicle response when a crash or tip-over is detected through
 *          accelerometer monitoring. Configured via FS_CRASH_CHECK parameter.
 *          Crash detection triggers when sustained high lateral/vertical acceleration
 *          indicates vehicle has collided or tipped.
 * 
 * @warning DISABLE should only be used if crash detection causes false positives
 *          in your operating environment (e.g., very rough terrain)
 * @warning HOLD_AND_DISARM is recommended for safety to prevent continued motor
 *          operation after a crash
 * 
 * @note Crash detection sensitivity is tuned via FS_CRASH_CHECK parameter value
 */
enum fs_crash_action {
  FS_CRASH_DISABLE = 0,          ///< Crash detection disabled - no response to detected crash
  FS_CRASH_HOLD = 1,             ///< Stop vehicle but remain armed on crash detection
  FS_CRASH_HOLD_AND_DISARM = 2   ///< Stop vehicle and disarm motors on crash detection (safest)
};

/**
 * @enum fs_ekf_action
 * @brief Extended Kalman Filter (EKF) failure response configuration
 * 
 * @details Controls vehicle response when EKF (navigation filter) detects a serious
 *          error in position/velocity estimation. Configured via FS_EKF_ACTION parameter.
 *          EKF failures indicate GPS/sensor problems that compromise navigation accuracy.
 * 
 * @warning DISABLE is not recommended - EKF failures indicate serious navigation problems
 *          that can cause the vehicle to drive in wrong directions or get lost
 * @warning During autonomous missions, EKF failures should trigger immediate action
 *          to prevent navigation errors from causing vehicle loss
 * 
 * @note EKF health is monitored through innovation checking and consistency tests
 * @note REPORT_ONLY is useful for development/testing to identify sensor issues
 *       without stopping operations
 */
enum fs_ekf_action {
    FS_EKF_DISABLE = 0,      ///< EKF failsafe disabled - no response to navigation errors
    FS_EKF_HOLD = 1,         ///< Stop vehicle and hold position on EKF failure
    FS_EKF_REPORT_ONLY = 2,  ///< Report EKF failure via telemetry but continue operation
};

/**
 * @def DISTANCE_HOME_MINCHANGE
 * @brief Minimum distance threshold for updating home location
 * 
 * @details When the vehicle is armed and moves more than this distance from the current
 *          home position, the home location is updated to the new position. This prevents
 *          GPS noise from constantly adjusting the home position during stationary operation
 *          while allowing home to track the vehicle during significant movement.
 * 
 * @note Value is in meters
 * @note Home location is used as the return point for RTL (Return To Launch) failsafe
 * @note Smaller values increase sensitivity to GPS noise, larger values may not track
 *       intended launch location if vehicle drifts
 */
#define DISTANCE_HOME_MINCHANGE 0.5f  // minimum distance to adjust home location

/**
 * @enum PilotSteerType
 * @brief Pilot steering input interpretation modes
 * 
 * @details Configures how RC transmitter stick inputs are interpreted for vehicle
 *          steering control. Different modes suit different vehicle types and operator
 *          preferences. Configured via PILOT_STEER_TYPE parameter.
 * 
 * @note Selected mode affects manual, acro, and steering modes but not autonomous modes
 */
enum class PilotSteerType : uint8_t {
    /**
     * @brief Standard single-stick steering (default mode)
     * 
     * One stick controls steering, separate throttle control. Most common mode
     * for car-like vehicles. Left/right stick input commands steering angle or rate.
     */
    DEFAULT = 0,
    
    /**
     * @brief Tank-style dual throttle control (skid steer)
     * 
     * Left and right stick control left and right motor speeds independently,
     * like a tank or skid-steer vehicle. Steering achieved by differential throttle.
     * Suitable for tracked vehicles or rovers with independent motor control.
     */
    TWO_PADDLES = 1,
    
    /**
     * @brief Steering direction reverses when driving backwards
     * 
     * Steering input direction is reversed when vehicle is driving in reverse.
     * Left stick input turns right when reversing, like steering a vehicle from
     * behind. Natural for operators used to "pushing" the back of the vehicle.
     */
    DIR_REVERSED_WHEN_REVERSING = 2,
    
    /**
     * @brief Steering direction constant regardless of drive direction
     * 
     * Steering input maintains same directional effect whether driving forward
     * or backward. Left always commands left turn. Natural for operators treating
     * steering as an absolute directional command.
     */
    DIR_UNCHANGED_WHEN_REVERSING = 3,
};

/**
 * @enum frame_class
 * @brief Vehicle frame type classification
 * 
 * @details Identifies the physical vehicle type to enable frame-specific control
 *          algorithms and parameter defaults. Configured via FRAME_CLASS parameter.
 *          Different frame classes use different control strategies, sensor priorities,
 *          and failsafe behaviors.
 * 
 * @note Changing frame class may require re-tuning control parameters
 * @note Some features are only available for specific frame classes (e.g., pitch
 *       control for balance bots)
 */
enum frame_class {
    /**
     * @brief Undefined or default frame type
     * 
     * Used when frame class is not explicitly configured. System will use
     * standard rover control algorithms.
     */
    FRAME_UNDEFINED = 0,
    
    /**
     * @brief Wheeled or tracked ground vehicle (default)
     * 
     * Standard ground vehicle with wheels or tracks. Uses Ackermann or skid steering.
     * Most common configuration for rovers. Control focuses on steering angle/rate
     * and throttle. Does not attempt to control pitch or roll.
     */
    FRAME_ROVER = 1,
    
    /**
     * @brief Water surface vehicle
     * 
     * Boat or watercraft operating on water surface. Uses rudder and motor/throttle
     * control. May have different damping characteristics and uses water-specific
     * navigation strategies. Takes into account water current and drift.
     */
    FRAME_BOAT = 2,
    
    /**
     * @brief Self-balancing two-wheeled robot
     * 
     * Inverted pendulum style robot that actively balances on two wheels. Requires
     * continuous pitch control to maintain upright position. Uses specialized PID
     * controllers for balance maintenance. Cannot operate in HOLD mode without
     * active balancing.
     */
    FRAME_BALANCEBOT = 3,
};

/**
 * @enum ManualOptions
 * @brief Manual mode behavior option flags
 * 
 * @details Bitmask flags that modify behavior of manual mode operation. Multiple
 *          options can be enabled simultaneously by bitwise OR-ing flags together.
 *          Configured via MANUAL_OPTIONS parameter.
 * 
 * @note These options only affect manual/acro modes, not autonomous operation
 */
enum ManualOptions {
    /**
     * @brief Enable speed-scaled steering sensitivity
     * 
     * When enabled, steering sensitivity is automatically reduced at higher speeds
     * to improve control stability and prevent over-steering. At low speeds, full
     * steering authority is available for tight maneuvering. At high speeds, the
     * same stick deflection produces less aggressive steering changes.
     * 
     * Recommended for high-speed vehicles or vehicles with sensitive steering.
     * Scaling factor is internally calculated based on current speed vs maximum
     * configured speed.
     */
    SPEED_SCALING = (1 << 0),
};
