/**
 * @file config.h
 * @brief Compile-time configuration for ArduPilot Blimp (lighter-than-air vehicle)
 * 
 * @details This file defines default build-time configuration parameters for the Blimp
 *          vehicle variant. It includes feature flags, hardware configuration options,
 *          default parameter values, and safety-critical constants specific to lighter-than-air
 *          vehicle control.
 * 
 *          Configuration options are defined using preprocessor conditionals (#ifndef)
 *          to allow customization via build system defines without modifying this file.
 *          Values defined here serve as fallback defaults when not explicitly configured.
 * 
 *          Key configuration categories:
 *          - Hardware abstraction layer board selection
 *          - Arming and safety delays
 *          - PWM and RC input parameters
 *          - MAVLink system identification
 *          - GPS accuracy thresholds
 *          - Failsafe timing and behavior (radio, EKF, terrain, throttle)
 *          - Pre-arm validation thresholds
 *          - Default flight modes
 *          - Logging configuration
 *          - Frame type defaults specific to airship configurations
 * 
 * @note Lighter-than-air vehicles have unique control characteristics compared to
 *       heavier-than-air aircraft. Configuration defaults reflect buoyancy-based flight,
 *       slower response times, and wind sensitivity considerations.
 * 
 * @warning Modifying safety-critical values (failsafe timeouts, pre-arm checks, EKF thresholds)
 *          can affect vehicle safety. Changes should be thoroughly tested in SITL before
 *          deployment to physical hardware.
 * 
 * @see APM_Config.h for feature-level configuration options
 * @see defines.h for constant definitions and enumerations
 * 
 * Source: Blimp/config.h
 */
#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Hardware Abstraction Layer (HAL) board validation
 * 
 * @details CONFIG_HAL_BOARD must be defined by the build system to specify the target
 *          hardware platform (ChibiOS, Linux, ESP32, SITL, etc.). This compile-time check
 *          ensures that the build system has correctly configured the platform before
 *          attempting to build the Blimp firmware.
 * 
 * @note Valid CONFIG_HAL_BOARD values are defined in AP_HAL and include:
 *       - HAL_BOARD_CHIBIOS (ARM-based flight controllers)
 *       - HAL_BOARD_LINUX (Linux-based systems)
 *       - HAL_BOARD_SITL (Software-in-the-loop simulation)
 *       - HAL_BOARD_ESP32 (ESP32 platform)
 */
#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build Blimp
#endif

/**
 * @brief Arming delay safety timeout
 * 
 * @details Minimum time in seconds that must elapse between arming request and actual
 *          motor arming. This delay provides a safety buffer allowing operators to abort
 *          the arming sequence if accidental arming is detected.
 * 
 * @note Default value of 2.0 seconds is appropriate for lighter-than-air vehicles which
 *       have slower response characteristics and more time for pre-flight verification.
 * 
 * @warning Reducing this value below 1.0 second may not provide sufficient time for
 *          operators to react to accidental arming commands.
 */
#ifndef ARMING_DELAY_SEC
# define ARMING_DELAY_SEC 2.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default RC output PWM update frequency
 * 
 * @details Defines the default PWM refresh rate in Hertz for RC outputs (servo and motor
 *          control signals). This frequency determines how often motor commands are updated
 *          and affects control loop responsiveness.
 * 
 *          490 Hz is a common "fast" PWM rate supported by most modern ESCs and servos,
 *          providing good control responsiveness while maintaining compatibility with
 *          standard RC equipment.
 * 
 * @note For lighter-than-air vehicles, the relatively slow vehicle dynamics mean that
 *       lower PWM rates (50-400 Hz) are often acceptable, but 490 Hz is used as default
 *       for consistency with other ArduPilot vehicles and ESC compatibility.
 * 
 * @warning Not all ESCs support high PWM rates. Verify ESC specifications before increasing
 *          this value. Some ESCs may require rates of 50 Hz (traditional analog servos) or
 *          400 Hz (standard digital servos).
 */
#ifndef RC_FAST_SPEED
#   define RC_FAST_SPEED 490
#endif

/**
 * @brief Default MAVLink system identifier
 * 
 * @details Unique identifier for this vehicle in a MAVLink network. Used by ground control
 *          stations and companion computers to distinguish between multiple vehicles or systems
 *          communicating on the same MAVLink network.
 * 
 *          System ID 1 is the traditional default for a single vehicle system. Multi-vehicle
 *          operations require unique system IDs for each vehicle (typically 1, 2, 3, etc.).
 * 
 * @note This default can be overridden at runtime via the SYSID_THISMAV parameter.
 * 
 * @see MAVLink protocol specification for system ID allocation guidelines
 */
#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// GPS Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Pre-arm GPS HDOP (Horizontal Dilution of Precision) threshold
 * 
 * @details Minimum acceptable GPS HDOP value (in HDOP units * 100) for pre-arm validation.
 *          HDOP is a measure of GPS position accuracy - lower values indicate better accuracy.
 *          This check is enforced during pre-arm when geofencing is enabled to ensure sufficient
 *          position accuracy for safe fence operation.
 * 
 *          Value of 140 corresponds to HDOP of 1.40, which indicates good GPS accuracy
 *          suitable for autonomous flight with position hold and geofencing.
 * 
 * @note HDOP values: <1.0 = Ideal, 1-2 = Excellent, 2-5 = Good, 5-10 = Moderate, >10 = Poor
 *       Value is scaled by 100 for integer representation (140 = 1.40 HDOP)
 * 
 * @warning Loosening this threshold (higher values) reduces position accuracy requirements
 *          and may allow flight with inadequate GPS lock, increasing fence breach risk.
 */
#ifndef GPS_HDOP_GOOD_DEFAULT
# define GPS_HDOP_GOOD_DEFAULT         140
#endif

//////////////////////////////////////////////////////////////////////////////
// Radio Failsafe Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief RC override failsafe timeout
 * 
 * @details Time in milliseconds before radio failsafe triggers when using RC_OVERRIDE
 *          MAVLink messages from a ground control station. RC_OVERRIDE allows a GCS to inject
 *          manual control inputs, bypassing the physical RC receiver. This timeout ensures
 *          failsafe activation if the GCS control link is lost.
 * 
 *          1000 ms (1 second) timeout provides a balance between quick failsafe response
 *          and tolerance for brief communication dropouts on telemetry links.
 * 
 * @note This timeout only applies when actively using RC_OVERRIDE. Physical RC input uses
 *       FS_RADIO_TIMEOUT_MS instead.
 * 
 * @warning This is a safety-critical parameter. Increasing timeout delays failsafe response
 *          during GCS control loss. Decreasing may cause false failsafes on marginal links.
 */
#ifndef FS_RADIO_RC_OVERRIDE_TIMEOUT_MS
# define FS_RADIO_RC_OVERRIDE_TIMEOUT_MS  1000
#endif

/**
 * @brief Physical RC input failsafe timeout
 * 
 * @details Time in milliseconds of no valid RC input before radio failsafe triggers.
 *          This detects loss of RC receiver signal from the physical transmitter and
 *          initiates configured failsafe action (typically Land or RTL).
 * 
 *          500 ms timeout provides rapid response to RC link loss while being long enough
 *          to avoid false triggers from brief signal disruptions.
 * 
 * @note Most RC receivers output a predictable failsafe signal or no signal when the
 *       transmitter link is lost. This timeout detects the "no signal" condition.
 * 
 * @warning This is a safety-critical parameter. Must be long enough to avoid false positives
 *          from receiver glitches, but short enough to trigger before vehicle travels
 *          dangerously far from operator control. 500 ms is standard for multirotor vehicles.
 */
#ifndef FS_RADIO_TIMEOUT_MS
#define FS_RADIO_TIMEOUT_MS            500
#endif

//////////////////////////////////////////////////////////////////////////////
// Terrain Failsafe Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Terrain data timeout threshold
 * 
 * @details Time in milliseconds of missing terrain altitude data before terrain failsafe
 *          triggers. Used when terrain following or terrain-aware flight modes are active.
 *          Missing data could result from:
 *          - Loss of terrain database access
 *          - Flying outside terrain database coverage
 *          - Rangefinder failure (if used as terrain source)
 *          - GPS position uncertainty preventing terrain lookup
 * 
 *          Default action is typically RTL (Return to Launch) to guide vehicle back to
 *          known-safe terrain.
 * 
 *          5 second timeout balances safety (quick response) with tolerance for brief
 *          terrain data retrieval delays.
 * 
 * @note Less critical for lighter-than-air vehicles which maintain altitude via buoyancy
 *       rather than powered flight, but still important for obstacle avoidance.
 * 
 * @warning Terrain failsafe only activates when terrain following features are enabled.
 *          Does not trigger during normal altitude-hold flight modes.
 */
#ifndef FS_TERRAIN_TIMEOUT_MS
#define FS_TERRAIN_TIMEOUT_MS          5000
#endif

//////////////////////////////////////////////////////////////////////////////
// Pre-Arm Validation Thresholds
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Maximum barometer vs inertial navigation altitude difference
 * 
 * @details Maximum allowed discrepancy in centimeters between barometer-derived altitude
 *          and inertial navigation (EKF) altitude estimate during pre-arm checks. Large
 *          disparities indicate sensor issues or incomplete EKF initialization.
 * 
 *          This check validates that:
 *          - Barometer is providing reasonable pressure readings
 *          - EKF altitude estimate has converged correctly
 *          - No significant sensor calibration errors exist
 * 
 *          100 cm (1 meter) threshold is tight enough to catch significant sensor problems
 *          while allowing for normal initialization variations and minor calibration differences.
 * 
 * @note Particularly important for lighter-than-air vehicles where altitude control depends
 *       on accurate altitude sensing for buoyancy compensation and pressure-altitude correlation.
 * 
 * @warning Loosening this check may allow arming with faulty altitude sensors, leading to
 *          altitude control problems or incorrect terrain following. Tightening excessively
 *          may prevent valid arming due to normal sensor variance.
 */
#ifndef PREARM_MAX_ALT_DISPARITY_CM
# define PREARM_MAX_ALT_DISPARITY_CM       100
#endif

//////////////////////////////////////////////////////////////////////////////
// EKF Failsafe Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default EKF failsafe action
 * 
 * @details Defines the default vehicle behavior when Extended Kalman Filter (EKF) variance
 *          exceeds acceptable thresholds, indicating navigation state estimation failure.
 *          
 *          FS_EKF_ACTION_LAND causes immediate landing when EKF health degrades beyond limits.
 *          This is appropriate for lighter-than-air vehicles which can safely descend even
 *          with degraded navigation accuracy.
 * 
 * @note Alternative actions include:
 *       - FS_EKF_ACTION_LAND_EVEN_STABILIZE: Land immediately regardless of mode
 *       - FS_EKF_ACTION_ALT_HOLD: Switch to altitude hold
 *       
 *       Landing is the safest default as it ensures vehicle recovery even with complete
 *       position estimate failure.
 * 
 * @warning EKF failsafe indicates serious navigation problems. Attempting continued flight
 *          during EKF failsafe may result in loss of position hold, fence breaches, or
 *          uncontrolled flight.
 */
#ifndef FS_EKF_ACTION_DEFAULT
# define FS_EKF_ACTION_DEFAULT         FS_EKF_ACTION_LAND
#endif

/**
 * @brief EKF variance failsafe threshold
 * 
 * @details Normalized variance threshold for EKF compass and velocity estimates. When
 *          innovation variance (difference between predicted and measured values) exceeds
 *          this threshold, it indicates the EKF's state estimate is diverging from sensor
 *          measurements, suggesting navigation failure.
 * 
 *          Value of 0.8 is a moderate threshold providing reasonable tolerance for sensor
 *          noise and temporary GPS degradation while triggering on significant EKF problems.
 * 
 *          Lower values = more sensitive (trigger earlier on variance increase)
 *          Higher values = less sensitive (tolerate larger variances before triggering)
 * 
 * @note Variance monitoring includes:
 *       - Compass innovation variance (heading estimate accuracy)
 *       - Velocity innovation variance (speed estimate accuracy)
 *       - Position innovation variance (location estimate accuracy)
 * 
 * @warning This is a safety-critical parameter. Too low causes false failsafes during
 *          normal sensor variations. Too high delays failsafe until navigation has
 *          significantly degraded.
 */
#ifndef FS_EKF_THRESHOLD_DEFAULT
# define FS_EKF_THRESHOLD_DEFAULT      0.8f
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight Mode Defaults
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default flight mode assignments for mode switch positions
 * 
 * @details Defines the default flight modes assigned to the 6 mode switch positions
 *          (typically controlled by a 6-position switch or channel on the RC transmitter).
 *          All positions default to MANUAL mode for safety - requires explicit configuration
 *          by the operator before using automated flight modes.
 * 
 *          MANUAL mode provides direct pilot control of thrust vectoring fins and vertical
 *          thrust, appropriate for initial setup and testing of lighter-than-air vehicles.
 * 
 * @note These defaults can be overridden at runtime via FLTMODE1-FLTMODE6 parameters.
 *       Common mode progression for airships:
 *       - Position 1: MANUAL (direct control)
 *       - Position 2: STABILIZE (attitude stabilization)
 *       - Position 3: LOITER (position hold)
 *       - Position 4-6: Mission/Auto modes
 * 
 * @warning Starting with all modes as MANUAL ensures new users cannot accidentally engage
 *          autonomous modes without explicit parameter configuration. Always test new modes
 *          in SITL or controlled environment before field deployment.
 */
#ifndef FLIGHT_MODE_1
# define FLIGHT_MODE_1                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_2
# define FLIGHT_MODE_2                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_3
# define FLIGHT_MODE_3                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_4
# define FLIGHT_MODE_4                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_5
# define FLIGHT_MODE_5                  Mode::Number::MANUAL
#endif
#ifndef FLIGHT_MODE_6
# define FLIGHT_MODE_6                  Mode::Number::MANUAL
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Throttle failsafe trigger value
 * 
 * @details PWM value (in microseconds) below which throttle input is considered to indicate
 *          failsafe condition. Most RC receivers output a predictable low throttle value
 *          (typically 900-1000 µs) when transmitter signal is lost.
 * 
 *          Value of 975 µs is below the typical RC throttle minimum of 1000 µs but above
 *          common failsafe outputs of 900-950 µs, providing reliable failsafe detection
 *          while avoiding false triggers from low throttle commands.
 * 
 * @note This works in conjunction with FS_RADIO_TIMEOUT_MS. Both no-signal timeout and
 *       low-throttle detection provide redundant failsafe triggering.
 * 
 * @warning This value must be calibrated for the specific RC receiver in use. Some receivers
 *          may output different failsafe values. Verify receiver behavior during setup.
 */
#ifndef FS_THR_VALUE_DEFAULT
# define FS_THR_VALUE_DEFAULT             975
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle Control Defaults
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Throttle stick deadzone
 * 
 * @details Deadzone in PWM microseconds around mid-throttle (typically 1500 µs) where
 *          throttle input is treated as zero vertical velocity command. Applied in altitude-hold
 *          and loiter modes to prevent small throttle stick movements from causing unwanted
 *          altitude changes.
 * 
 *          100 µs deadzone (±50 µs from center) provides stable altitude hold while still
 *          allowing responsive vertical control with deliberate stick movements.
 * 
 * @note For lighter-than-air vehicles, this deadzone is particularly important as buoyancy
 *       provides natural altitude stability. Small throttle inputs should be filtered to
 *       avoid fighting the vehicle's natural equilibrium.
 * 
 * @warning Too large a deadzone reduces vertical control responsiveness. Too small causes
 *          altitude drift from stick centering imperfections and transmitter trim variations.
 */
#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100
#endif

/**
 * @brief Automatic disarming delay
 * 
 * @details Time in seconds of zero throttle and landed condition before automatic disarm
 *          occurs. Prevents accidental motor startup from armed vehicle that has landed
 *          and been forgotten.
 * 
 *          10 second delay provides enough time for deliberate re-takeoff after landing
 *          while automatically disarming after extended inactivity for safety.
 * 
 * @note Automatic disarming only occurs when vehicle detects landed condition (minimal
 *       movement, low throttle, no altitude change). Does not trigger during intentional
 *       low hover or altitude hold.
 * 
 * @warning Disabling auto-disarm (value 0) creates safety risk of leaving vehicle armed
 *          indefinitely. Extended arming increases risk of accidental motor activation.
 */
#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  10
#endif

//////////////////////////////////////////////////////////////////////////////
// Data Logging Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default logging bitmask
 * 
 * @details Defines which data streams are logged to onboard storage (SD card or flash) by
 *          default. Each bit corresponds to a different log message type. This bitmask
 *          enables comprehensive logging for flight analysis, tuning, and post-flight review.
 * 
 *          Enabled log types for Blimp:
 *          - MASK_LOG_ATTITUDE_MED: Attitude (roll, pitch, yaw) at medium rate
 *          - MASK_LOG_GPS: GPS position, velocity, and accuracy data
 *          - MASK_LOG_PM: Performance monitoring (loop timing, CPU usage)
 *          - MASK_LOG_CTUN: Control tuning data (controller outputs, errors)
 *          - MASK_LOG_NTUN: Navigation tuning data (position errors, navigation state)
 *          - MASK_LOG_RCIN: RC input values from receiver
 *          - MASK_LOG_IMU: Inertial Measurement Unit raw data (gyro, accel)
 *          - MASK_LOG_CMD: Mission command execution
 *          - MASK_LOG_CURRENT: Battery current and voltage
 *          - MASK_LOG_RCOUT: RC output values to motors/servos
 *          - MASK_LOG_OPTFLOW: Optical flow sensor data (if equipped)
 *          - MASK_LOG_COMPASS: Magnetometer data
 *          - MASK_LOG_CAMERA: Camera trigger events
 *          - MASK_LOG_MOTBATT: Motor and battery detailed telemetry
 * 
 * @note This configuration provides balanced logging suitable for most flight analysis needs
 *       while avoiding excessive storage consumption. Additional log types can be enabled
 *       via LOG_BITMASK parameter.
 * 
 * @warning High-rate logging (ATTITUDE_FAST, IMU_FAST) significantly increases storage
 *          usage and may exceed write bandwidth on some platforms. Use selectively for
 *          detailed analysis only.
 */
#ifndef DEFAULT_LOG_BITMASK
# define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif

//////////////////////////////////////////////////////////////////////////////
// RC Channel Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default RC mode control channel
 * 
 * @details Specifies the default RC receiver channel number (1-based) used for flight mode
 *          selection. Channel 5 is a common choice as it's typically mapped to a 3-position
 *          or 6-position mode switch on standard RC transmitters.
 * 
 *          Channel assignments:
 *          1 = Roll, 2 = Pitch, 3 = Throttle, 4 = Yaw (standard)
 *          5 = Mode switch (configurable)
 *          6+ = Auxiliary functions
 * 
 * @note This default can be overridden via the FLTMODE_CH parameter. Some operators prefer
 *       channel 6 or 8 for mode selection depending on transmitter configuration.
 * 
 * @warning Ensure the selected channel has a stable switch or knob input. Using a proportional
 *          stick can cause unintended mode changes during flight.
 */
#ifndef CH_MODE_DEFAULT
# define CH_MODE_DEFAULT   5
#endif

//////////////////////////////////////////////////////////////////////////////
// Frame Type Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Default airship frame configuration
 * 
 * @details Defines the default physical frame type for lighter-than-air vehicle control.
 *          MOTOR_FRAME_TYPE_AIRFISH specifies an airship configuration with:
 *          - Vectoring control surfaces (fins) for attitude control
 *          - Vertical thrust motor for altitude/buoyancy compensation
 *          - Lighter-than-air flight characteristics
 * 
 *          The Airfish configuration is designed for indoor/outdoor airship platforms with
 *          four control fins arranged for pitch, roll, and yaw control plus vertical thrust.
 * 
 * @note Frame type determines:
 *       - Motor/servo mixing algorithms
 *       - Control surface mapping
 *       - Default PID tuning parameters
 *       - Expected vehicle dynamics model
 * 
 * @warning Incorrect frame type configuration will result in improper control surface mixing
 *          and unstable flight. Always verify frame type matches physical vehicle configuration
 *          before first flight.
 * 
 * @see Fins::MOTOR_FRAME_TYPE for available airship frame configurations
 */
#ifndef HAL_FRAME_TYPE_DEFAULT
#define HAL_FRAME_TYPE_DEFAULT Fins::MOTOR_FRAME_TYPE_AIRFISH
#endif
