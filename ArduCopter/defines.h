/**
 * @file defines.h
 * @brief ArduCopter-specific preprocessor definitions, macros, and constants
 * 
 * @details This header file contains all ArduCopter-specific compile-time constants and
 *          enumerations that define vehicle behavior, configuration options, and
 *          operational parameters. These definitions are used throughout the ArduCopter
 *          codebase to configure:
 *          - Frame type identification (multicopter, helicopter)
 *          - Real-time tuning parameter mappings
 *          - Waypoint navigation yaw behavior modes
 *          - Flight control options (air mode, developer options)
 *          - Data logging configuration bitmasks
 *          - Failsafe action definitions for various failure scenarios
 *          - Pilot input behavior configuration
 * 
 *          All values are compile-time constants and do not consume RAM. These
 *          definitions establish the contract between the vehicle code and the
 *          parameter system, ground control stations, and logging infrastructure.
 * 
 * @note This file contains only preprocessor definitions and enumerations.
 *       No executable code or variable declarations are included.
 * 
 * @warning Modifying failsafe definitions or behavior flags can affect vehicle
 *          safety and must be done with extreme caution. All changes should be
 *          thoroughly tested in SITL before hardware deployment.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @defgroup frame_types Vehicle Frame Type Identifiers
 * @brief Frame type constants for vehicle identification and frame-specific logic
 * 
 * @details These constants identify the vehicle's mechanical configuration to enable
 *          frame-specific control algorithms, motor mixing, and parameter validation.
 *          The frame type is typically set during vehicle initialization and affects
 *          motor output mixing, control tuning, and available flight modes.
 * 
 * @{
 */
#define UNDEFINED_FRAME 0        ///< Frame type not yet determined or configured
#define MULTICOPTER_FRAME 1      ///< Standard multicopter (quad, hex, octo, etc.) with electric motors
#define HELI_FRAME 2             ///< Traditional helicopter with collective pitch control and tail rotor
/** @} */ // end of frame_types group

/**
 * @defgroup tuning_parameters Real-Time Tuning Parameter Enumeration
 * @brief Enumeration mapping tuning channel inputs to adjustable flight parameters
 * 
 * @details This enumeration defines the mapping between auxiliary RC channels configured
 *          for in-flight tuning and the actual flight control parameters they adjust.
 *          When a tuning channel is enabled via parameters (TUNE, TUNE_MIN, TUNE_MAX),
 *          these values determine which controller gain, rate limit, or behavior setting
 *          is modified in real-time based on pilot stick position.
 * 
 *          Primary use cases:
 *          - PID gain tuning during test flights
 *          - Rate limit adjustments for acro modes
 *          - Position/velocity controller tuning
 *          - AHRS and sensor fusion parameter adjustment
 * 
 *          Each enum value corresponds to a specific parameter or controller property
 *          that can be adjusted via the configured tuning channel (typically channel 6).
 *          The actual range and scaling are defined by TUNE_MIN and TUNE_MAX parameters.
 * 
 * @note Tuning changes are temporary and revert to parameter values on reboot unless
 *       parameters are explicitly saved via GCS commands.
 * 
 * @warning Improper tuning values can cause vehicle instability or loss of control.
 *          Always tune conservatively with sufficient altitude for recovery.
 * 
 * @{
 */
enum tuning_func {
    TUNING_NONE =                        0, //
    TUNING_STABILIZE_ROLL_PITCH_KP =     1, // stabilize roll/pitch angle controller's P term
    TUNING_STABILIZE_YAW_KP =            3, // stabilize yaw heading controller's P term
    TUNING_RATE_ROLL_PITCH_KP =          4, // body frame roll/pitch rate controller's P term
    TUNING_RATE_ROLL_PITCH_KI =          5, // body frame roll/pitch rate controller's I term
    TUNING_YAW_RATE_KP =                 6, // body frame yaw rate controller's P term
    TUNING_THROTTLE_RATE_KP =            7, // throttle rate controller's P term (desired rate to acceleration or motor output)
    TUNING_WP_SPEED =                   10, // maximum speed to next way point (0 to 10m/s)
    TUNING_LOITER_POSITION_KP =         12, // loiter distance controller's P term (position error to speed)
    TUNING_HELI_EXTERNAL_GYRO =         13, // TradHeli specific external tail gyro gain
    TUNING_ALTITUDE_HOLD_KP =           14, // altitude hold controller's P term (alt error to desired rate)
    TUNING_RATE_ROLL_PITCH_KD =         21, // body frame roll/pitch rate controller's D term
    TUNING_VEL_XY_KP =                  22, // loiter rate controller's P term (speed error to tilt angle)
    TUNING_ACRO_RP_RATE =               25, // acro controller's desired roll and pitch rate in deg/s
    TUNING_YAW_RATE_KD =                26, // body frame yaw rate controller's D term
    TUNING_VEL_XY_KI =                  28, // loiter rate controller's I term (speed error to tilt angle)
    TUNING_AHRS_YAW_KP =                30, // ahrs's compass effect on yaw angle (0 = very low, 1 = very high)
    TUNING_AHRS_KP =                    31, // accelerometer effect on roll/pitch angle (0=low)
    TUNING_ACCEL_Z_KP =                 34, // accel based throttle controller's P term
    TUNING_ACCEL_Z_KI =                 35, // accel based throttle controller's I term
    TUNING_ACCEL_Z_KD =                 36, // accel based throttle controller's D term
    TUNING_DECLINATION =                38, // compass declination in radians
    TUNING_CIRCLE_RATE =                39, // circle turn rate in degrees (hard coded to about 45 degrees in either direction)
    TUNING_ACRO_YAW_RATE =              40, // acro controller's desired yaw rate in deg/s
    TUNING_RANGEFINDER_GAIN =           41, // unused
    TUNING_EKF_VERTICAL_POS =           42, // unused
    TUNING_EKF_HORIZONTAL_POS =         43, // unused
    TUNING_EKF_ACCEL_NOISE =            44, // unused
    TUNING_RC_FEEL_RP =                 45, // roll-pitch input smoothing
    TUNING_RATE_PITCH_KP =              46, // body frame pitch rate controller's P term
    TUNING_RATE_PITCH_KI =              47, // body frame pitch rate controller's I term
    TUNING_RATE_PITCH_KD =              48, // body frame pitch rate controller's D term
    TUNING_RATE_ROLL_KP =               49, // body frame roll rate controller's P term
    TUNING_RATE_ROLL_KI =               50, // body frame roll rate controller's I term
    TUNING_RATE_ROLL_KD =               51, // body frame roll rate controller's D term
    TUNING_RATE_PITCH_FF =              52, // body frame pitch rate controller FF term
    TUNING_RATE_ROLL_FF =               53, // body frame roll rate controller FF term
    TUNING_RATE_YAW_FF =                54, // body frame yaw rate controller FF term
    TUNING_RATE_MOT_YAW_HEADROOM =      55, // motors yaw headroom minimum
    TUNING_RATE_YAW_FILT =              56, // yaw rate input filter
    UNUSED =                            57, // was winch control
    TUNING_SYSTEM_ID_MAGNITUDE =        58, // magnitude of the system ID signal
    TUNING_POS_CONTROL_ANGLE_MAX =      59, // position controller maximum angle
    TUNING_LOITER_MAX_XY_SPEED =        60, // maximum loiter horizontal speed
};
/** @} */ // end of tuning_parameters group

/**
 * @defgroup waypoint_yaw_behavior Waypoint Yaw Behavior Modes
 * @brief Yaw control behavior during autonomous waypoint navigation and RTL
 * 
 * @details These constants define how the autopilot controls vehicle yaw (heading)
 *          during autonomous missions and Return-To-Launch (RTL). The behavior is
 *          selected via the WP_YAW_BEHAVIOR parameter and affects pilot experience
 *          and camera pointing during autonomous flight.
 * 
 *          Yaw behavior modes:
 *          - Manual control: Pilot retains yaw control during missions
 *          - Look-at-next-WP: Vehicle points toward upcoming waypoint
 *          - Look-ahead: Vehicle points along flight path (ideal for forward cameras)
 * 
 *          These settings do not affect manual flight modes (Stabilize, AltHold, etc.)
 *          where pilot always has direct yaw control.
 * 
 * @note DO_CONDITIONAL_YAW mission commands always override the WP_YAW_BEHAVIOR setting
 *       and force the vehicle to point in a specified direction.
 * 
 * @{
 */
/// Pilot retains full yaw control during missions; autopilot never controls yaw except for explicit DO_CONDITIONAL_YAW commands
#define WP_YAW_BEHAVIOR_NONE                          0
/// Autopilot points vehicle nose toward next waypoint during missions and toward home during RTL
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP               1
/// Autopilot points toward next waypoint during missions but maintains last heading during RTL
#define WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL    2
/// Autopilot points along flight path tangent (look-ahead); optimized for traditional helicopters and forward-facing cameras
#define WP_YAW_BEHAVIOR_LOOK_AHEAD                    3
/** @} */ // end of waypoint_yaw_behavior group


/**
 * @defgroup airmode Air Mode Configuration
 * @brief Air mode enumeration controlling motor behavior at zero throttle
 * 
 * @details Air mode determines whether motors remain armed and controllable at zero
 *          throttle stick position. This is critical for acrobatic flight and maintaining
 *          control authority during aggressive maneuvers.
 * 
 *          Air mode options:
 *          - NONE: Air mode setting not initialized
 *          - DISABLED: Motors spin down at zero throttle (traditional behavior)
 *          - ENABLED: Motors maintain minimum spin at zero throttle for control authority
 * 
 *          When air mode is enabled, attitude control remains active even at zero throttle,
 *          allowing pilots to perform inverted maneuvers and maintain control during
 *          negative-G flight. This is essential for acro flight but increases risk of
 *          propeller strikes during ground operations.
 * 
 * @warning Air mode keeps motors armed at zero throttle. Use caution during ground
 *          operations to avoid injury from spinning propellers.
 * 
 * @{
 */
enum class AirMode {
    AIRMODE_NONE,        ///< Air mode not configured or initialized
    AIRMODE_DISABLED,    ///< Traditional behavior: motors spin down at zero throttle
    AIRMODE_ENABLED,     ///< Acro behavior: motors maintain minimum spin for control authority at zero throttle
};
/** @} */ // end of airmode group

/**
 * @defgroup dev_options Developer Option Flags
 * @brief Bitmask flags for enabling experimental or developer-specific features
 * 
 * @details Developer options are experimental features or non-standard behaviors that
 *          can be enabled via the DEV_OPTIONS bitmask parameter. These options are
 *          typically used for testing, integration with specific ground control stations,
 *          or enabling features that deviate from standard ArduPilot behavior.
 * 
 *          Options are combined using bitwise OR to enable multiple features simultaneously.
 *          Example: DEV_OPTIONS = 3 enables both ADSB MAVLink and VFR_HUD relative altitude.
 * 
 * @warning Developer options may not be thoroughly tested and could affect vehicle
 *          stability or compatibility with standard ground control stations.
 * 
 * @{
 */
enum DevOptions {
    DevOptionADSBMAVLink = 1,           ///< Bit 0: Enable ADSB vehicle forwarding via MAVLink for traffic awareness
    DevOptionVFR_HUDRelativeAlt = 2,    ///< Bit 1: Report altitude relative to home in VFR_HUD MAVLink message instead of absolute altitude
};
/** @} */ // end of dev_options group

/**
 * @defgroup logging_parameters Vehicle-Specific Logging Message IDs
 * @brief Enumeration of ArduCopter-specific log message types for binary logging
 * 
 * @details This enumeration defines the message type IDs for ArduCopter-specific log
 *          messages in the binary dataflash log format. These message IDs supplement
 *          the common ArduPilot log messages with vehicle-specific data such as
 *          control tuning values, guided mode targets, and helicopter-specific telemetry.
 * 
 *          The logging system is constrained to 32 vehicle-specific message types due
 *          to protocol limitations. Message structures are defined in Log.cpp and
 *          written to dataflash/SD card for post-flight analysis and debugging.
 * 
 *          Common use cases:
 *          - Post-flight control tuning analysis (CTUN messages)
 *          - System identification data collection (SYSIDD, SYSIDS)
 *          - Guided mode trajectory validation
 *          - Helicopter-specific rotor and servo data
 * 
 * @note Log message structures and field definitions are in ArduCopter/Log.cpp
 * 
 * @{
 */
enum LoggingParameters {
     LOG_CONTROL_TUNING_MSG,            ///< Control loop tuning data (desired vs achieved rates, angles)
     LOG_DATA_INT16_MSG,                ///< Generic 16-bit signed integer data logging
     LOG_DATA_UINT16_MSG,               ///< Generic 16-bit unsigned integer data logging
     LOG_DATA_INT32_MSG,                ///< Generic 32-bit signed integer data logging
     LOG_DATA_UINT32_MSG,               ///< Generic 32-bit unsigned integer data logging
     LOG_DATA_FLOAT_MSG,                ///< Generic floating-point data logging
     LOG_PARAMTUNE_MSG,                 ///< Parameter tuning history from in-flight adjustments
     LOG_HELI_MSG,                      ///< Traditional helicopter-specific telemetry (rotor speed, servo positions)
     LOG_GUIDED_POSITION_TARGET_MSG,    ///< Guided mode position targets from GCS or companion computer
     LOG_SYSIDD_MSG,                    ///< System identification frequency sweep data (desired input)
     LOG_SYSIDS_MSG,                    ///< System identification frequency sweep data (system response)
     LOG_GUIDED_ATTITUDE_TARGET_MSG,    ///< Guided mode attitude targets from GCS or companion computer
     LOG_RATE_THREAD_DT_MSG             ///< Rate controller loop timing diagnostics (delta-time measurements)
};
/** @} */ // end of logging_parameters group

/**
 * @defgroup logging_bitmasks Data Logging Enable Bitmasks
 * @brief Bitmask flags controlling which data categories are logged to dataflash
 * 
 * @details These bitmask constants control the LOG_BITMASK parameter, which determines
 *          what data is logged to the onboard dataflash or SD card. Each bit enables
 *          a category of log messages, allowing users to balance logging detail against
 *          storage capacity and write bandwidth.
 * 
 *          Bitmask values are combined using bitwise OR to enable multiple categories.
 *          Example: LOG_BITMASK = (MASK_LOG_ATTITUDE_MED | MASK_LOG_GPS | MASK_LOG_PM)
 *                   enables attitude, GPS, and performance monitoring logs.
 * 
 *          High-rate logging (FAST variants) significantly increases storage requirements
 *          and may reduce log duration. Use FAST logging only when needed for detailed
 *          analysis of high-frequency control dynamics or vibration issues.
 * 
 *          Special values:
 *          - MASK_LOG_ANY = 0xFFFF: Enable all logging categories (maximum detail)
 * 
 * @note The INAV log category (bit 14) is deprecated and no longer used.
 * 
 * @warning High-rate logging can fill SD cards quickly. Monitor free space and log
 *          duration when using multiple FAST logging categories simultaneously.
 * 
 * @{
 */
#define MASK_LOG_ATTITUDE_FAST          (1<<0)      ///< Log attitude at fast rate (400Hz typical) for detailed control analysis
#define MASK_LOG_ATTITUDE_MED           (1<<1)      ///< Log attitude at medium rate (25Hz typical) for general flight review
#define MASK_LOG_GPS                    (1<<2)      ///< Log GPS position, velocity, and accuracy data
#define MASK_LOG_PM                     (1<<3)      ///< Log performance monitoring: loop time, CPU load, memory usage
#define MASK_LOG_CTUN                   (1<<4)      ///< Log control tuning: desired vs achieved angles, rates, throttle
#define MASK_LOG_NTUN                   (1<<5)      ///< Log navigation tuning: position/velocity errors, navigation outputs
#define MASK_LOG_RCIN                   (1<<6)      ///< Log RC receiver input values from all channels
#define MASK_LOG_IMU                    (1<<7)      ///< Log IMU sensor data: gyros, accelerometers at standard rate
#define MASK_LOG_CMD                    (1<<8)      ///< Log mission command execution and waypoint transitions
#define MASK_LOG_CURRENT                (1<<9)      ///< Log battery voltage, current, and power consumption
#define MASK_LOG_RCOUT                  (1<<10)     ///< Log RC/servo output values to motors and servos
#define MASK_LOG_OPTFLOW                (1<<11)     ///< Log optical flow sensor velocity measurements
#define MASK_LOG_PID                    (1<<12)     ///< Log PID controller internal values: P, I, D terms for all axes
#define MASK_LOG_COMPASS                (1<<13)     ///< Log magnetometer readings and compass health status
#define MASK_LOG_INAV                   (1<<14)     ///< Deprecated: Inertial navigation logging (no longer used)
#define MASK_LOG_CAMERA                 (1<<15)     ///< Log camera trigger events and feedback
#define MASK_LOG_MOTBATT                (1UL<<17)   ///< Log motor-specific battery telemetry from ESCs (voltage per motor)
#define MASK_LOG_IMU_FAST               (1UL<<18)   ///< Log IMU at fast rate (400Hz typical) for vibration and control analysis
#define MASK_LOG_IMU_RAW                (1UL<<19)   ///< Log raw unfiltered IMU data for sensor evaluation and filter tuning
#define MASK_LOG_VIDEO_STABILISATION    (1UL<<20)   ///< Log video stabilization gimbal targets and achieved angles
#define MASK_LOG_FTN_FAST               (1UL<<21)   ///< Log FFT notch filter data at fast rate for dynamic notch tuning
#define MASK_LOG_ANY                    0xFFFF      ///< Enable all available logging categories (maximum logging detail)
/** @} */ // end of logging_bitmasks group

/**
 * @defgroup failsafe_radio Radio (RC Receiver) Failsafe Action Definitions
 * @brief Actions taken when RC receiver signal is lost or degraded (FS_THR parameter)
 * 
 * @details These constants define the vehicle's response when radio contact is lost
 *          between the RC transmitter and receiver. The failsafe is triggered when:
 *          - RC signal is lost completely
 *          - Throttle channel drops below FS_THR_VALUE for FS_THR_TIMEOUT duration
 *          - RC receiver reports failsafe condition via protocol flags
 * 
 *          Failsafe actions range from immediate emergency landing to intelligent
 *          return-to-launch with path planning. The appropriate action depends on:
 *          - Flight environment (indoor vs outdoor, obstacles present)
 *          - Vehicle capabilities (GPS quality, SmartRTL path availability)
 *          - Mission criticality (can mission continue autonomously)
 * 
 *          FS_OPTIONS bitmask parameter provides additional failsafe behavior control
 *          such as continuing missions on GCS failsafe (removed from main enums in 4.0+).
 * 
 * @warning Failsafe actions are safety-critical. Test thoroughly in SITL and open
 *          areas before relying on failsafe behavior with valuable hardware.
 * 
 * @note CONTINUE_MISSION option removed in ArduCopter 4.0+; use FS_OPTIONS parameter
 *       to configure mission continuation behavior instead.
 * 
 * @{
 */
#define FS_THR_DISABLED                            0    ///< Radio failsafe disabled; no action on signal loss
#define FS_THR_ENABLED_ALWAYS_RTL                  1    ///< Always execute Return-To-Launch (RTL) on radio signal loss
#define FS_THR_ENABLED_CONTINUE_MISSION            2    ///< [Deprecated 4.0+] Continue mission autonomously; see FS_OPTIONS instead
#define FS_THR_ENABLED_ALWAYS_LAND                 3    ///< Always land immediately at current position on radio signal loss
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL      4    ///< Use SmartRTL to retrace path if available, otherwise RTL on signal loss
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND     5    ///< Use SmartRTL if available, otherwise land immediately on signal loss
#define FS_THR_ENABLED_AUTO_RTL_OR_RTL             6    ///< RTL if in Auto mode, otherwise return home via RTL on signal loss
#define FS_THR_ENABLED_BRAKE_OR_LAND               7    ///< Brake (stop movement) then descend and land at brake position on signal loss
/** @} */ // end of failsafe_radio group

/**
 * @defgroup failsafe_gcs Ground Control Station Failsafe Action Definitions
 * @brief Actions taken when GCS telemetry link is lost (FS_GCS_ENABLE parameter)
 * 
 * @details These constants define the vehicle's response when the telemetry link to
 *          the ground control station (GCS) is lost. The failsafe is triggered when:
 *          - No MAVLink heartbeat received from GCS for FS_GCS_TIMEOUT duration
 *          - Telemetry radio link quality drops below threshold
 * 
 *          GCS failsafe is typically less aggressive than radio failsafe since pilots
 *          retain manual control via RC transmitter. However, for autonomous missions
 *          where GCS provides critical path updates or abort commands, GCS failsafe
 *          provides a safety net for telemetry link failures.
 * 
 *          Action selection considerations:
 *          - Is RC transmitter available as backup control method?
 *          - Is mission pre-loaded or requires real-time GCS updates?
 *          - Can vehicle safely complete mission without GCS oversight?
 * 
 * @warning If both RC and GCS failsafes trigger simultaneously, RC failsafe action
 *          takes precedence as the more critical failure mode.
 * 
 * @note CONTINUE_MISSION option removed in ArduCopter 4.0+; use FS_OPTIONS parameter
 *       to configure mission continuation behavior instead.
 * 
 * @{
 */
#define FS_GCS_DISABLED                        0    ///< GCS failsafe disabled; no action on telemetry loss
#define FS_GCS_ENABLED_ALWAYS_RTL              1    ///< Always execute Return-To-Launch (RTL) on GCS link loss
#define FS_GCS_ENABLED_CONTINUE_MISSION        2    ///< [Deprecated 4.0+] Continue mission autonomously; see FS_OPTIONS instead
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL  3    ///< Use SmartRTL to retrace path if available, otherwise RTL on GCS loss
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND 4    ///< Use SmartRTL if available, otherwise land immediately on GCS loss
#define FS_GCS_ENABLED_ALWAYS_LAND             5    ///< Always land immediately at current position on GCS link loss
#define FS_GCS_ENABLED_AUTO_RTL_OR_RTL         6    ///< RTL if in Auto mode, otherwise return home via RTL on GCS loss
#define FS_GCS_ENABLED_BRAKE_OR_LAND           7    ///< Brake (stop movement) then descend and land at brake position on GCS loss
/** @} */ // end of failsafe_gcs group

/**
 * @defgroup failsafe_ekf Extended Kalman Filter (EKF) Failsafe Action Definitions
 * @brief Actions taken when EKF navigation solution becomes unreliable (FS_EKF_ACTION parameter)
 * 
 * @details These constants define the vehicle's response when the Extended Kalman Filter
 *          (EKF) detects that its position/velocity estimate has become unreliable due to:
 *          - GPS signal loss or degradation
 *          - Excessive innovation (sensor disagreement)
 *          - Compass interference or magnetic anomalies
 *          - IMU sensor failures or high vibration
 *          - Optical flow sensor loss (if primary navigation source)
 * 
 *          EKF failsafe is one of the most critical safety mechanisms since position
 *          estimation errors can lead to flyaways or navigation failures. The EKF
 *          continuously monitors innovation sequences and covariance growth to detect
 *          estimation problems before they cause dangerous vehicle behavior.
 * 
 *          Trigger thresholds are controlled by FS_EKF_THRESH parameter (innovation
 *          velocity threshold in m/s). Conservative threshold: 0.8 m/s, aggressive: 1.5 m/s.
 * 
 *          Action selection depends on flight mode capabilities:
 *          - Autonomous modes (Auto, Guided, RTL) require position estimates
 *          - ALTHOLD requires vertical position but not horizontal
 *          - Stabilize requires no position estimate (attitude-only)
 * 
 * @warning EKF failsafe indicates fundamental navigation failure. LAND_EVEN_STABILIZE
 *          option forces landing even in manual modes, preventing pilot from continuing
 *          flight with degraded navigation quality.
 * 
 * @note EKF health is also reported via EKF_STATUS MAVLink message for GCS monitoring
 *       and can trigger pre-arm check failures if unhealthy before takeoff.
 * 
 * @{
 */
#define FS_EKF_ACTION_REPORT_ONLY           0       ///< Report EKF failsafe via telemetry but take no automatic action; pilot retains control
#define FS_EKF_ACTION_LAND                  1       ///< Switch to LAND mode on EKF failsafe; descend and land at current position
#define FS_EKF_ACTION_ALTHOLD               2       ///< Switch to ALTHOLD mode on EKF failsafe; maintain altitude but allow pilot horizontal control
#define FS_EKF_ACTION_LAND_EVEN_STABILIZE   3       ///< Force switch to LAND mode on EKF failsafe even if in manual flight modes (Stabilize, Acro)
/** @} */ // end of failsafe_ekf group

/**
 * @defgroup pilot_throttle_behavior Pilot Throttle Behavior Configuration Flags
 * @brief Bitmask flags configuring throttle stick behavior (PILOT_THR_BHV parameter)
 * 
 * @details These bitmask constants control how the throttle stick is interpreted in
 *          various flight modes and situations. These behavior modifications affect
 *          pilot experience, landing detection, and automatic disarming, allowing
 *          customization for different pilot preferences and operational scenarios.
 * 
 *          Flags are combined using bitwise OR to enable multiple behaviors.
 *          Example: PILOT_THR_BHV = 6 enables both high-throttle-cancels-land and
 *                   auto-disarm-on-land-detect.
 * 
 *          Behavior implications:
 *          - Mid-stick feedback affects altitude hold stick feel (center = hold vs climb/descend)
 *          - High throttle cancels land allows pilot to abort automatic landing
 *          - Auto disarm improves safety by disarming after detected landing
 * 
 * @note These behaviors only affect certain flight modes. Refer to mode-specific
 *       documentation for detailed behavior in each mode.
 * 
 * @{
 */
/// Bit 0: Altitude controller uses mid-stick as neutral; below mid = descend, above mid = climb
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
/// Bit 1: High throttle input (>80%) cancels automatic landing and switches to altitude hold
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
/// Bit 2: Automatically disarm motors when landing is detected instead of requiring pilot action
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
/** @} */ // end of pilot_throttle_behavior group
