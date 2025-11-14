/**
 * @file defines.h
 * @brief Constant definitions and enumerations for antenna tracker
 * 
 * @details This file defines compile-time constants, enumerations for servo control types,
 *          altitude data sources, PWM disarmed behavior, and logging configuration.
 *          These definitions control the antenna tracker's behavior for pointing accuracy
 *          and tracking reliability.
 * 
 * Source: AntennaTracker/defines.h
 */

#pragma once

/**
 * @enum ServoType
 * @brief Servo control strategy selection for antenna tracker actuators
 * 
 * @details Defines the control algorithm used for antenna positioning servos.
 *          Different servo types require different control approaches for
 *          optimal tracking performance and hardware compatibility.
 */
enum ServoType {
    SERVO_TYPE_POSITION=0,  ///< Position-controlled servo with PID feedback for precise angle tracking
    SERVO_TYPE_ONOFF=1,     ///< Bang-bang control for limit-switch actuators (binary position control)
    SERVO_TYPE_CR=2         ///< Continuous rotation servo with speed control for geared systems
};

/**
 * @enum AltSource
 * @brief Altitude data source for pitch angle calculation
 * 
 * @details Determines which altitude sensor data to use when calculating
 *          the elevation angle (pitch) for pointing the antenna at the vehicle.
 *          Choice affects tracking accuracy in different environmental conditions.
 */
enum AltSource {
	ALT_SOURCE_BARO=0,           ///< Use barometric altitude difference between tracker and vehicle for pitch calculation
	ALT_SOURCE_GPS=1,            ///< Use GPS absolute altitudes (MSL) for both tracker and vehicle
	ALT_SOURCE_GPS_VEH_ONLY=2    ///< Use vehicle's GPS relative altitude only (assumes tracker at ground level)
};

/**
 * @enum PWMDisarmed
 * @brief Servo output behavior when antenna tracker is disarmed
 * 
 * @details Controls what PWM signal is sent to servos when the tracker
 *          is disarmed for safety or during initialization. This prevents
 *          unexpected antenna movement and reduces servo power consumption.
 */
enum class PWMDisarmed {
    ZERO = 0,  ///< Output 0 PWM (servo unpowered, no holding torque)
    TRIM,      ///< Output neutral/trim position (maintains 0° angle with holding torque)
};

/**
 * @brief Low-pass filter cutoff frequency for servo output smoothing
 * 
 * @details Cutoff frequency in Hertz for the servo output low-pass filter.
 *          This smooths rapid changes in commanded servo position to prevent
 *          mechanical oscillation and reduce wear on servo gears.
 *          Lower values = smoother but slower response (0.1 Hz = ~1.6 second time constant).
 * 
 * @note Value in Hz. Time constant τ ≈ 1/(2π * SERVO_OUT_FILT_HZ) ≈ 1.59 seconds
 */
#define SERVO_OUT_FILT_HZ               0.1f

/**
 * @brief Main loop delta time in seconds
 * 
 * @details Time step for the main control loop update cycle.
 *          50 Hz update rate (1/50 = 0.02 seconds = 20 milliseconds).
 *          This defines the servo control update frequency and affects
 *          PID controller tuning and filter behavior.
 * 
 * @note Value in seconds. Corresponds to 50 Hz loop rate.
 */
#define G_Dt                            0.02f

/**
 * @name Logging Bitmask Parameters
 * @brief Bit flags for enabling/disabling specific log message types
 * 
 * @details These masks control which data streams are written to the onboard log.
 *          Multiple masks can be OR'd together to enable multiple log types.
 *          Used with LOG_BITMASK parameter for selective logging to manage
 *          storage usage and logging performance.
 * 
 * @{
 */

/** @brief Enable attitude (roll/pitch/yaw) logging */
#define MASK_LOG_ATTITUDE               (1<<0)

/** @brief Enable GPS position and velocity logging */
#define MASK_LOG_GPS                    (1<<1)

/** @brief Enable RC input (pilot commands) logging */
#define MASK_LOG_RCIN                   (1<<2)

/** @brief Enable IMU (gyro/accel) raw sensor logging */
#define MASK_LOG_IMU                    (1<<3)

/** @brief Enable RC output (servo PWM commands) logging */
#define MASK_LOG_RCOUT                  (1<<4)

/** @brief Enable compass (magnetometer) logging */
#define MASK_LOG_COMPASS                (1<<5)

/** @brief Enable battery current and voltage logging */
#define MASK_LOG_CURRENT                (1<<6)

/** @brief Enable all available log message types (all bits set) */
#define MASK_LOG_ANY                    0xFFFF

/** @} */ // end of Logging Bitmask Parameters group

/**
 * @enum log_messages
 * @brief Custom log message identifiers for antenna tracker
 * 
 * @details Defines vehicle-specific binary log message types beyond the common
 *          ArduPilot log messages. Only 32 custom message slots are available
 *          per vehicle type for implementation-specific logging needs.
 * 
 * @warning Maximum of 32 custom log messages permitted per vehicle type
 */
enum log_messages {
    LOG_V_BAR_MSG,  ///< Vehicle barometric altitude message for tracking elevation data
    LOG_V_POS_MSG,  ///< Vehicle position message for tracking azimuth and distance data
};
