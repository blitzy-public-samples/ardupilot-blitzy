/**
 * @file defines.h
 * @brief Compile-time constant definitions for ArduPilot Blimp (lighter-than-air vehicle)
 * 
 * @details This file centralizes compile-time constants and definitions specific to the Blimp vehicle type.
 *          It includes:
 *          - Developer debugging options (DevOptions enum)
 *          - Logging message type enumerations for vehicle-specific log messages
 *          - Logging mask bits for enabling/disabling log groups
 *          - Radio failsafe behavior definitions (FS_THR parameter values)
 *          - Ground Control Station failsafe definitions (FS_GCS parameter values)
 *          - Extended Kalman Filter failsafe action codes (FS_EKF_ACTION parameter values)
 *          - Pilot throttle behavior bit masks (PILOT_THR_BHV parameter)
 *          
 *          Blimps (lighter-than-air vehicles) have unique control characteristics due to their buoyancy,
 *          requiring specialized handling of thrust, altitude control, and wind compensation. Many of
 *          these definitions are adapted from multicopter but tuned for blimp-specific behaviors.
 * 
 * @note These are compile-time constants and cannot be changed at runtime. For adjustable behavior,
 *       use the corresponding ArduPilot parameters (e.g., FS_THR, FS_GCS, PILOT_THR_BHV).
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: Blimp/defines.h
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @enum DevOptions
 * @brief Developer options bit mask for enabling debugging features via DEV_OPTIONS parameter
 * 
 * @details These flags enable various developer/debugging features that are typically disabled
 *          in production builds. They can be combined using bitwise OR. The DEV_OPTIONS parameter
 *          uses these bit values to control optional development features.
 * 
 * @note These options are intended for development and testing only, not normal operation.
 */
enum DevOptions {
    /** Enable ADSB information via MAVLink for development testing */
    DevOptionADSBMAVLink = 1,
    /** Use relative altitude in VFR_HUD MAVLink message instead of absolute altitude */
    DevOptionVFR_HUDRelativeAlt = 2,
    /** Interpret SET_ATTITUDE_TARGET MAVLink thrust field as actual thrust instead of climb rate */
    DevOptionSetAttitudeTarget_ThrustAsThrust = 4,
};

/**
 * @enum LoggingParameters
 * @brief Vehicle-specific logging message type identifiers for Blimp
 * 
 * @details Defines the enumeration values for custom log messages specific to the Blimp vehicle.
 *          ArduPilot reserves 32 message slots per vehicle type for vehicle-specific logging.
 *          These identifiers are used by the AP_Logger system to identify different log message types.
 *          
 *          Log messages include control tuning data, raw sensor values, PID controller states,
 *          and system identification data for performance analysis and debugging.
 * 
 * @note Only 32 custom message types are available per vehicle. These enum values are indices
 *       into the vehicle's log message structure array.
 * 
 * @see AP_Logger for the logging system implementation
 * @see Blimp/Log.cpp for message structure definitions and logging implementations
 */
enum LoggingParameters {
    /** Control tuning message: attitude, velocity, and position controller performance data */
    LOG_CONTROL_TUNING_MSG,
    /** Generic 16-bit signed integer data message for debugging */
    LOG_DATA_INT16_MSG,
    /** Generic 16-bit unsigned integer data message for debugging */
    LOG_DATA_UINT16_MSG,
    /** Generic 32-bit signed integer data message for debugging */
    LOG_DATA_INT32_MSG,
    /** Generic 32-bit unsigned integer data message for debugging */
    LOG_DATA_UINT32_MSG,
    /** Generic floating-point data message for debugging */
    LOG_DATA_FLOAT_MSG,
    /** Parameter tuning message: logs parameter changes during auto-tuning procedures */
    LOG_PARAMTUNE_MSG,
    /** Helicopter/blimp-specific message: fin control outputs and mechanical state */
    LOG_HELI_MSG,
    /** Guided mode target message: logs target position, velocity, and attitude in guided modes */
    LOG_GUIDEDTARGET_MSG,
    /** System identification - desired values: logs desired inputs for frequency sweep analysis */
    LOG_SYSIDD_MSG,
    /** System identification - measured response: logs actual system response during sysid */
    LOG_SYSIDS_MSG,
    /** Fin input message: logs commanded fin angles and control surface inputs */
    LOG_FINI_MSG,
    /** Fin output message: logs actual fin positions and servo outputs */
    LOG_FINO_MSG,
    /** PID desired values: logs desired state for PID controller analysis */
    LOG_PIDD_MSG,
    /** PID velocity (north): logs velocity controller PID values for north axis */
    LOG_PIVN_MSG,
    /** PID velocity (east): logs velocity controller PID values for east axis */
    LOG_PIVE_MSG,
    /** PID velocity (down/altitude): logs velocity controller PID values for vertical axis */
    LOG_PIVD_MSG,
    /** PID yaw: logs yaw rate controller PID values for heading control */
    LOG_PIVY_MSG,

};

/**
 * @name Logging Mask Bits
 * @brief Bit mask definitions for LOG_BITMASK parameter to enable/disable log message groups
 * 
 * @details These bit flags are used with the LOG_BITMASK parameter to selectively enable or disable
 *          groups of related log messages. Multiple groups can be enabled by bitwise OR-ing the masks.
 *          Disabling unnecessary logs reduces storage requirements and improves logging performance.
 *          
 *          The LOG_BITMASK parameter in ArduPilot allows users to control which types of data are logged
 *          to the onboard dataflash or SD card during flight.
 * 
 * @note Bits 0-15 use standard (1<<n) notation, bits 17+ use (1UL<<n) for unsigned long to avoid overflow
 * @warning Disabling critical logs (GPS, IMU, ATTITUDE) may make post-flight analysis difficult
 * 
 * @{
 */

/** Enable fast attitude logging (high-rate attitude, 400Hz typical) */
#define MASK_LOG_ATTITUDE_FAST          (1<<0)
/** Enable medium-rate attitude logging (50Hz typical) */
#define MASK_LOG_ATTITUDE_MED           (1<<1)
/** Enable GPS position, velocity, and accuracy logging */
#define MASK_LOG_GPS                    (1<<2)
/** Enable performance monitoring: loop time, memory usage, CPU load */
#define MASK_LOG_PM                     (1<<3)
/** Enable control tuning logs: attitude and position controller states */
#define MASK_LOG_CTUN                   (1<<4)
/** Enable navigation tuning logs: waypoint tracking, position estimates */
#define MASK_LOG_NTUN                   (1<<5)
/** Enable RC input logging: pilot stick positions and switch states */
#define MASK_LOG_RCIN                   (1<<6)
/** Enable IMU logging: accelerometer and gyroscope data at main loop rate */
#define MASK_LOG_IMU                    (1<<7)
/** Enable mission command logging: waypoint execution and mode changes */
#define MASK_LOG_CMD                    (1<<8)
/** Enable current sensor logging: voltage, current, and power consumption */
#define MASK_LOG_CURRENT                (1<<9)
/** Enable RC output logging: servo/motor PWM output values */
#define MASK_LOG_RCOUT                  (1<<10)
/** Enable optical flow sensor logging: velocity measurements from optical flow */
#define MASK_LOG_OPTFLOW                (1<<11)
/** Enable PID controller logging: P, I, D terms for all control loops */
#define MASK_LOG_PID                    (1<<12)
/** Enable compass/magnetometer logging: raw magnetic field measurements */
#define MASK_LOG_COMPASS                (1<<13)
/** Enable inertial navigation logging (deprecated - now part of EKF logs) */
#define MASK_LOG_INAV                   (1<<14)
/** Enable camera trigger event logging: camera shutter events with GPS position */
#define MASK_LOG_CAMERA                 (1<<15)
/** Enable motor battery monitoring: per-motor voltage and current if available */
#define MASK_LOG_MOTBATT                (1UL<<17)
/** Enable fast IMU logging: raw IMU data at sensor rate (typically 1kHz+) */
#define MASK_LOG_IMU_FAST               (1UL<<18)
/** Enable raw IMU logging: unfiltered IMU data before notch filters */
#define MASK_LOG_IMU_RAW                (1UL<<19)
/** Enable all logging: bit mask with all bits set (logs everything) */
#define MASK_LOG_ANY                    0xFFFF

/** @} */ // End of Logging Mask Bits group

/**
 * @name Radio Failsafe Definitions
 * @brief Values for FS_THR parameter defining throttle failsafe behavior
 * 
 * @details These constants define the behavior when radio control signal is lost (throttle failsafe).
 *          The FS_THR parameter uses these values to determine what action the blimp should take
 *          when RC link is lost. Failsafe is triggered when no valid RC packets are received for
 *          the configured timeout period (FS_TIMEOUT parameter, typically 1.5 seconds).
 *          
 *          For blimps (lighter-than-air vehicles), RTL behavior considers wind and buoyancy to
 *          return to launch safely. Land mode gradually descends while maintaining horizontal position.
 * 
 * @warning Critical safety feature - ensure FS_THR is configured appropriately for your vehicle
 * @note Some options were removed in firmware 4.0+, use fs_options bitmask for advanced behaviors
 * 
 * @see FS_TIMEOUT parameter for failsafe trigger timing
 * @see FS_OPTIONS parameter for additional failsafe behavior customization
 * 
 * @{
 */

/** Radio failsafe disabled - no action taken on RC loss (not recommended for autonomous operation) */
#define FS_THR_DISABLED                            0
/** Always execute Return to Launch (RTL) on RC loss - flies back to launch point automatically */
#define FS_THR_ENABLED_ALWAYS_RTL                  1
/** Continue current mission on RC loss - removed in 4.0+, now use fs_options parameter */
#define FS_THR_ENABLED_CONTINUE_MISSION            2
/** Always execute Land mode on RC loss - descends vertically at current position */
#define FS_THR_ENABLED_ALWAYS_LAND                 3
/** Execute SmartRTL if available, otherwise RTL - uses recorded path for safer return */
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL      4
/** Execute SmartRTL if available, otherwise Land - uses recorded path or lands in place */
#define FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND     5

/** @} */ // End of Radio Failsafe Definitions group

/**
 * @name GCS Failsafe Definitions
 * @brief Values for FS_GCS_ENABLE parameter defining Ground Control Station failsafe behavior
 * 
 * @details These constants define the behavior when telemetry link to Ground Control Station (GCS)
 *          is lost. The FS_GCS_ENABLE parameter uses these values to determine failsafe action.
 *          GCS failsafe is triggered when no HEARTBEAT messages are received from GCS for the
 *          configured timeout period (FS_GCS_TIMEOUT parameter, typically 5 seconds).
 *          
 *          Unlike radio failsafe (FS_THR), GCS failsafe can be configured to allow mission continuation
 *          since the vehicle can still operate autonomously without telemetry link.
 * 
 * @warning Ensure GCS failsafe is configured appropriately for your operational environment
 * @note Some options were removed in firmware 4.0+, use fs_options bitmask for advanced behaviors
 * 
 * @see FS_GCS_TIMEOUT parameter for failsafe trigger timing
 * @see FS_OPTIONS parameter for additional failsafe behavior customization
 * 
 * @{
 */

/** GCS failsafe disabled - no action taken on telemetry loss */
#define FS_GCS_DISABLED                        0
/** Always execute Return to Launch (RTL) on GCS link loss */
#define FS_GCS_ENABLED_ALWAYS_RTL              1
/** Continue current mission on GCS link loss - removed in 4.0+, now use fs_options parameter */
#define FS_GCS_ENABLED_CONTINUE_MISSION        2
/** Execute SmartRTL if available, otherwise RTL on GCS link loss */
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_RTL  3
/** Execute SmartRTL if available, otherwise Land on GCS link loss */
#define FS_GCS_ENABLED_ALWAYS_SMARTRTL_OR_LAND 4
/** Always execute Land mode on GCS link loss */
#define FS_GCS_ENABLED_ALWAYS_LAND             5

/** @} */ // End of GCS Failsafe Definitions group

/**
 * @name EKF Failsafe Action Definitions
 * @brief Values for FS_EKF_ACTION parameter defining Extended Kalman Filter failsafe behavior
 * 
 * @details These constants define the behavior when the EKF (Extended Kalman Filter) detects
 *          a critical failure in state estimation. The FS_EKF_ACTION parameter uses these values
 *          to determine what action to take when EKF health degrades below acceptable thresholds.
 *          
 *          EKF failsafe is triggered when innovation (difference between predicted and measured values)
 *          exceeds configured thresholds, indicating GPS glitch, compass interference, or sensor failure.
 *          For blimps, accurate position estimation is critical for autonomous navigation and station-keeping.
 * 
 * @warning EKF failsafe is a critical safety mechanism - poor state estimation can lead to loss of control
 * @note The EKF continuously monitors sensor consistency and prediction accuracy
 * 
 * @see AP_NavEKF3 for EKF implementation and health monitoring
 * @see FS_EKF_THRESH parameter for EKF failsafe trigger threshold
 * 
 * @{
 */

/** Switch to LAND mode on EKF failsafe (except when in Manual mode - pilot retains control) */
#define FS_EKF_ACTION_LAND                  1
/** Switch to LAND mode on EKF failsafe even if in Manual mode (overrides pilot control) */
#define FS_EKF_ACTION_LAND_EVEN_MANUAL      3

/** @} */ // End of EKF Failsafe Action Definitions group

/**
 * @name Pilot Throttle Behavior Flags
 * @brief Bit mask values for PILOT_THR_BHV parameter controlling pilot throttle stick behavior
 * 
 * @details These bit flags configure how the pilot's throttle stick input is interpreted and processed.
 *          The PILOT_THR_BHV parameter uses bitwise OR of these flags to enable multiple behaviors.
 *          These options affect manual control feel and automatic mode interactions.
 *          
 *          For blimps, throttle behavior is adapted for vertical thrust control considering buoyancy.
 *          Unlike multirotors where throttle directly controls thrust, blimps may be neutrally buoyant
 *          and require different throttle mapping.
 * 
 * @note These flags can be combined using bitwise OR to enable multiple behaviors simultaneously
 * @warning Changing throttle behavior affects pilot control feel - test carefully in safe environment
 * 
 * @see PILOT_THR_BHV parameter documentation for detailed behavior descriptions
 * 
 * @{
 */

/** Enable altitude feedback from mid-stick: throttle stick center holds altitude instead of zero thrust */
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
/** Allow high throttle input to cancel automatic landing: pilot can abort land with throttle up */
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
/** Automatically disarm motors when landing is detected: reduces post-landing workload */
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)

/** @} */ // End of Pilot Throttle Behavior Flags group
