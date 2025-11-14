/**
 * @file AP_Arming.h
 * @brief ArduPilot arming and pre-arm check system
 * 
 * @details This file defines the AP_Arming class which implements the comprehensive
 *          vehicle arming safety check framework used across all ArduPilot vehicle types.
 *          The arming system performs critical safety validations before allowing motor
 *          startup and flight operations.
 * 
 *          The arming system operates in two stages:
 *          1. Pre-arm checks: Performed continuously, provide early warning of issues
 *          2. Arm checks: Performed during arming attempt, gate motor startup
 * 
 *          Key responsibilities:
 *          - Validate sensor health and calibration (GPS, compass, IMU, barometer)
 *          - Check RC transmitter connection and calibration
 *          - Verify battery voltage and capacity
 *          - Validate mission waypoints and geofence configuration
 *          - Ensure safety switch state and failsafe configuration
 *          - Provide user feedback via GCS status messages
 * 
 *          Vehicle-specific implementations extend this base class to add vehicle-specific
 *          checks (e.g., AP_Arming_Copter, AP_Arming_Plane, AP_Arming_Rover).
 * 
 * @note This is safety-critical code - modifications must be thoroughly tested
 * @warning Bypassing arming checks can result in vehicle crashes and injuries
 * 
 * @see AP_Arming_Copter in ArduCopter/AP_Arming_Copter.h
 * @see AP_Arming_Plane in ArduPlane/AP_Arming_Plane.h
 * @see AP_Arming_Rover in Rover/AP_Arming_Rover.h
 * @see AP_Arming_Sub in ArduSub/AP_Arming_Sub.h
 * 
 * Source: libraries/AP_Arming/AP_Arming.h:1-355
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS_config.h>
#include <AP_BoardConfig/AP_BoardConfig_config.h>

#include "AP_Arming_config.h"
#include "AP_InertialSensor/AP_InertialSensor_config.h"
#include "AP_Proximity/AP_Proximity_config.h"

/**
 * @class AP_Arming
 * @brief Base class for ArduPilot vehicle arming system
 * 
 * @details AP_Arming implements a comprehensive pre-flight safety check framework that
 *          validates vehicle sensor health, configuration, and readiness before allowing
 *          motor arming. This is a safety-critical system that prevents operation with
 *          faulty sensors or invalid configurations.
 * 
 *          Architecture:
 *          - Singleton pattern for global access via AP::arming()
 *          - Virtual methods allow vehicle-specific check customization
 *          - Bitmask-based check enable/disable via ARMING_CHECK parameter
 *          - Mandatory checks that cannot be bypassed
 *          - Configurable reporting and display options
 * 
 *          Check Types:
 *          - Pre-arm checks: Run continuously at ~1Hz, display failures to user
 *          - Arm checks: Run during arm attempt, block arming on failure
 *          - Mandatory checks: Cannot be disabled, always enforced
 * 
 *          State Machine:
 *          Disarmed → Pre-arm checks pass → Arm checks pass → Armed
 *          Armed → Disarm (various triggers) → Disarmed
 * 
 *          Parameter Configuration:
 *          - ARMING_REQUIRE: Controls if arming is required (NO/YES_MIN_PWM/YES_ZERO_PWM)
 *          - ARMING_CHECK: Bitmask of enabled checks (can disable non-mandatory checks)
 *          - ARMING_ACCTHRESH: Accelerometer consistency threshold
 *          - ARMING_RUDDER: Rudder arming enable (disabled/arm-only/arm-disarm)
 *          - ARMING_MIS_ITEMS: Required mission item checks
 *          - ARMING_OPTIONS: Behavior option flags
 * 
 *          Lifecycle:
 *          1. Construction: Vehicle creates arming object at startup
 *          2. init(): Initializes parameters and state
 *          3. update(): Called by scheduler at ~1Hz, runs pre-arm checks
 *          4. arm(): Called when user attempts to arm, runs arm checks
 *          5. disarm(): Called when vehicle disarms (manual or automatic)
 * 
 *          Thread Safety:
 *          - update() called from main scheduler thread
 *          - arm()/disarm() may be called from GCS handler threads
 *          - Semaphore protection for auxiliary authorization state
 * 
 *          Integration Points:
 *          - Sensor libraries (AP_GPS, AP_Compass, AP_InertialSensor, AP_Baro)
 *          - Safety systems (AC_Fence, AP_Mission, AP_Rally)
 *          - Communication (GCS_MAVLink for status reporting)
 *          - Vehicle motors library (holds actual armed state for some vehicles)
 * 
 * @note Copter stores armed state in motors library, not in this class
 * @warning Modifying check logic can compromise flight safety
 * @warning Disabling mandatory checks is not possible and should never be attempted
 * 
 * @see AP::arming() for singleton accessor
 * @see GCS_MAVLink for status message reporting
 */
class AP_Arming {
public:

    AP_Arming();

    CLASS_NO_COPY(AP_Arming);  /* Do not allow copies */

    static AP_Arming *get_singleton();

    /**
     * @brief Update arming state and perform pre-arm checks
     * 
     * @details Called by vehicle scheduler at approximately 1Hz. Aggregates pre-arm check
     *          results from all enabled subsystems and throttles GCS status message reporting
     *          to avoid overwhelming the telemetry link. When arming state changes or new
     *          failures are detected, status messages are sent immediately.
     * 
     *          This method coordinates:
     *          - Running pre-arm checks for all enabled subsystems
     *          - Aggregating check results into overall pass/fail state
     *          - Managing status message reporting throttling
     *          - Updating internal state variables
     * 
     * @note Called at ~1Hz by vehicle scheduler
     * @note Does NOT perform actual arming - use arm() method for that
     * 
     * @see pre_arm_checks() for the actual check implementation
     * @see arm() for arming state transition
     */
    void update();

    /**
     * @enum Check
     * @brief Bitmask enumeration of arming check types
     * 
     * @details Defines the individual subsystem checks that can be independently enabled
     *          or disabled via the ARMING_CHECK parameter. Each bit represents a specific
     *          category of safety validation. Setting ARMING_CHECK=0 disables all non-mandatory
     *          checks, while ARMING_CHECK=-1 enables all checks.
     * 
     *          Check categories validate:
     *          - Sensor health and calibration
     *          - Communication links
     *          - Power system status
     *          - Configuration validity
     *          - Mission/fence setup
     * 
     * @note Some checks are mandatory and cannot be disabled even with ARMING_CHECK=0
     * @see check_enabled() to test if a specific check is enabled
     * @see checks_to_perform parameter for bitmask storage
     */
    enum class Check {
        ALL         = (1U << 0),   ///< All checks enabled (meta-flag)
        BARO        = (1U << 1),   ///< Barometer: altitude sensor health, calibration. Failures: no barometer detected, unhealthy readings, excessive noise
        COMPASS     = (1U << 2),   ///< Compass: magnetometer health, calibration, interference. Failures: not calibrated, magnetic field inconsistent, excessive offsets
        GPS         = (1U << 3),   ///< GPS: fix quality, satellite count, HDOP. Failures: no fix, insufficient satellites, poor accuracy
        INS         = (1U << 4),   ///< Inertial Navigation System: IMU health, gyro/accel calibration, consistency. Failures: IMU unhealthy, inconsistent sensors, calibration required
        PARAMETERS  = (1U << 5),   ///< Parameters: configuration validity. Failures: invalid parameter values detected
        RC          = (1U << 6),   ///< RC receiver: transmitter connection, channel calibration. Failures: no RC input, uncalibrated channels, invalid trim
        VOLTAGE     = (1U << 7),   ///< Voltage: battery voltage level. Failures: voltage below minimum threshold
        BATTERY     = (1U << 8),   ///< Battery: capacity, monitor health. Failures: low capacity, monitor unhealthy, failsafe threshold exceeded
        AIRSPEED    = (1U << 9),   ///< Airspeed: sensor health (fixed-wing). Failures: sensor unhealthy, not calibrated, readings inconsistent
        LOGGING     = (1U << 10),  ///< Logging: SD card present and writable. Failures: no SD card, SD card full, logging unavailable
        SWITCH      = (1U << 11),  ///< Switch: safety switch state, disarm switch. Failures: safety switch not engaged when required
        GPS_CONFIG  = (1U << 12),  ///< GPS configuration: RTK, blending settings. Failures: invalid GPS configuration, blending misconfigured
        SYSTEM      = (1U << 13),  ///< System: CPU load, memory, board health. Failures: CPU overload, memory low, hardware error
        MISSION     = (1U << 14),  ///< Mission: waypoint validity, required items. Failures: no mission, invalid waypoints, missing required items (land, takeoff)
        RANGEFINDER = (1U << 15),  ///< Rangefinder: distance sensor health. Failures: sensor unhealthy, no data
        CAMERA      = (1U << 16),  ///< Camera: camera/gimbal ready. Failures: camera backend error
        AUX_AUTH    = (1U << 17),  ///< Auxiliary authorization: external auth required. Failures: authorization timeout, auth denied
        VISION      = (1U << 18),  ///< Vision: visual odometry health. Failures: vision system unhealthy, position estimate poor
        FFT         = (1U << 19),  ///< FFT: frequency analysis for notch filters. Failures: FFT not initialized when required
        OSD         = (1U << 20),  ///< OSD: on-screen display health. Failures: OSD backend error
    };

    /**
     * @enum Method
     * @brief Arming/disarming method and automatic disarm reason enumeration
     * 
     * @details Identifies how the vehicle was armed or disarmed. Values 0-4 are methods that
     *          can be used for both arming and disarming. Values 5+ are automatic disarm reasons
     *          triggered by failsafes, safety events, or normal operational completion.
     * 
     *          Arming Methods (0-4, 34-38):
     *          - User-initiated arming via pilot input or GCS command
     *          - System-initiated arming for special modes
     * 
     *          Disarm Reasons (5-33):
     *          - Failsafe triggers (battery, radio, GCS, EKF, terrain, CPU)
     *          - Safety events (crash, fence breach, termination)
     *          - Normal completion (landed, mission exit, auto-disarm)
     * 
     *          This enumeration is critical for:
     *          - Logging arm/disarm events with context
     *          - Determining appropriate status messages
     *          - Triggering vehicle-specific behaviors
     *          - Post-flight analysis and debugging
     * 
     * @note Values logged to dataflash for post-flight analysis
     * @see last_arm_method() to retrieve last arming method
     * @see last_disarm_method() to retrieve last disarming reason
     * @see Log_Write_Arm() and Log_Write_Disarm() for logging
     */
    enum class Method {
        RUDDER = 0,                     ///< Armed/disarmed via rudder stick (right rudder + zero throttle)
        MAVLINK = 1,                    ///< Armed/disarmed via MAVLink command from ground control station
        AUXSWITCH = 2,                  ///< Armed/disarmed via auxiliary RC switch
        MOTORTEST = 3,                  ///< Armed/disarmed for motor test sequence
        SCRIPTING = 4,                  ///< Armed/disarmed via Lua script command
        TERMINATION = 5,                ///< Automatic disarm: flight termination triggered
        CPUFAILSAFE = 6,                ///< Automatic disarm: CPU overload or timing failure
        BATTERYFAILSAFE = 7,            ///< Automatic disarm: battery voltage/capacity failsafe triggered
        SOLOPAUSEWHENLANDED = 8,        ///< Automatic disarm: Solo pause when landed (legacy)
        AFS = 9,                        ///< Automatic disarm: advanced failsafe system triggered
        ADSBCOLLISIONACTION = 10,       ///< Automatic disarm: ADS-B collision avoidance action
        PARACHUTE_RELEASE = 11,         ///< Automatic disarm: parachute deployed
        CRASH = 12,                     ///< Automatic disarm: crash detected by accelerometer
        LANDED = 13,                    ///< Automatic disarm: vehicle detected as landed (auto-disarm delay)
        MISSIONEXIT = 14,               ///< Automatic disarm: mission completed and exited
        FENCEBREACH = 15,               ///< Automatic disarm: geofence breach action
        RADIOFAILSAFE = 16,             ///< Automatic disarm: RC radio signal lost
        DISARMDELAY = 17,               ///< Automatic disarm: auto-disarm delay timeout
        GCSFAILSAFE = 18,               ///< Automatic disarm: ground control station link lost
        TERRRAINFAILSAFE = 19,          ///< Automatic disarm: terrain data unavailable (typo in original, kept for compatibility)
        FAILSAFE_ACTION_TERMINATE = 20, ///< Automatic disarm: general failsafe termination action
        TERRAINFAILSAFE = 21,           ///< Automatic disarm: terrain data unavailable (correct spelling)
        MOTORDETECTDONE = 22,           ///< Automatic disarm: motor direction detection completed
        BADFLOWOFCONTROL = 23,          ///< Automatic disarm: internal software error detected
        EKFFAILSAFE = 24,               ///< Automatic disarm: Extended Kalman Filter failure (position estimate lost)
        GCS_FAILSAFE_SURFACEFAILED = 25,///< Automatic disarm: sub failed to surface during GCS failsafe
        GCS_FAILSAFE_HOLDFAILED = 26,   ///< Automatic disarm: hold position failed during GCS failsafe
        TAKEOFFTIMEOUT = 27,            ///< Automatic disarm: takeoff timeout expired
        AUTOLANDED = 28,                ///< Automatic disarm: automatic landing completed
        PILOT_INPUT_FAILSAFE = 29,      ///< Automatic disarm: pilot input failsafe triggered
        TOYMODELANDTHROTTLE = 30,       ///< Automatic disarm: toy mode ended with throttle condition
        TOYMODELANDFORCE = 31,          ///< Automatic disarm: toy mode force-ended
        LANDING = 32,                   ///< Automatic disarm: landing sequence completed
        DEADRECKON_FAILSAFE = 33,       ///< Automatic disarm: dead reckoning navigation failsafe
        BLACKBOX = 34,                  ///< Armed/disarmed for blackbox logging mode
        DDS = 35,                       ///< Armed/disarmed via DDS (ROS2) command
        AUTO_ARM_ONCE = 36,             ///< Auto-arm once feature triggered
        TURTLE_MODE = 37,               ///< Armed for turtle mode (flip recovery)
        TOYMODE = 38,                   ///< Armed/disarmed for toy mode operation
        UNKNOWN = 100,                  ///< Unknown or uninitialized method
    };

    /**
     * @enum Required
     * @brief Arming requirement mode configuration
     * 
     * @details Controls whether arming is required before motor output and what throttle
     *          position is required for arming. Also enables automatic arming behavior for
     *          certain vehicle types. This is configured via the ARMING_REQUIRE parameter.
     * 
     *          Safety implications:
     *          - NO: Motors can spin without arming (dangerous, not recommended)
     *          - YES_MIN_PWM: Standard arming, throttle must be at minimum
     *          - YES_ZERO_PWM: Arming requires throttle at absolute zero
     *          - Auto-arm modes: Vehicle arms automatically when conditions met
     * 
     * @note NO mode is dangerous and should only be used in controlled environments
     * @warning Automatic arming modes require careful configuration to prevent unexpected motor startup
     * @see arming_required() to query current setting
     */
    enum class Required {
        NO           = 0,  ///< Arming not required - motors can spin while disarmed (unsafe)
        YES_MIN_PWM  = 1,  ///< Arming required with throttle at minimum PWM value
        YES_ZERO_PWM = 2,  ///< Arming required with throttle at zero PWM
        YES_AUTO_ARM_MIN_PWM = 3,  ///< Auto-arm when throttle raised to minimum (some vehicle types)
        YES_AUTO_ARM_ZERO_PWM = 4, ///< Auto-arm with throttle at zero (some vehicle types)
    };

    /**
     * @brief Initialize the arming system
     * 
     * @details Called during vehicle startup to initialize the arming subsystem. Sets up
     *          internal state, initializes parameters from storage, and prepares the check
     *          framework. Must be called before any arm/disarm operations.
     * 
     *          Initialization sequence:
     *          1. Reset internal state variables
     *          2. Load parameters from persistent storage
     *          3. Initialize check state tracking
     *          4. Configure GPIO outputs if applicable
     * 
     * @note Called once during vehicle initialization, before scheduler starts
     * @note Must complete before update() or arm() methods are called
     * @see update() for periodic checks after initialization
     */
    void init(void);

    /**
     * @brief Query the arming requirement configuration
     * 
     * @details Returns the current arming requirement mode from the ARMING_REQUIRE parameter.
     *          Determines whether arming is required and what throttle position is needed.
     * 
     * @return Required enum value indicating arming requirement mode
     * 
     * @note Copter vehicles hold armed state in motors library, not here
     * @see Required enum for possible values
     */
    Required arming_required() const;
    
    /**
     * @brief Attempt to arm the vehicle
     * 
     * @details Attempts to transition vehicle from disarmed to armed state, enabling motor
     *          output. Performs arming checks unless bypassed. Logs the arming event and
     *          reports status to ground control station.
     * 
     *          Arming sequence:
     *          1. If do_arming_checks=true, run arm_checks()
     *          2. If checks pass, set armed state
     *          3. Record arm time and method
     *          4. Log arming event
     *          5. Update GPIO outputs
     *          6. Send status to GCS
     * 
     * @param[in] method Method used for arming (RUDDER, MAVLINK, etc.)
     * @param[in] do_arming_checks If true, perform arming checks; if false, force arm
     * 
     * @return true if vehicle successfully armed, false if checks failed or arming denied
     * 
     * @note Copter vehicles use motors library for armed state, not this base class
     * @warning Arming with failed checks can result in loss of vehicle
     * @warning Vehicle-specific implementations may override this method
     * 
     * @see arm_checks() for validation performed
     * @see arm_force() to force arming without checks
     * @see disarm() for disarming
     */
    virtual bool arm(AP_Arming::Method method, bool do_arming_checks=true);
    
    /**
     * @brief Force arm the vehicle without performing checks
     * 
     * @details Bypasses all arming checks and forces the vehicle to armed state. This is
     *          dangerous and should only be used when the operator has verified vehicle
     *          readiness through other means (e.g., for factory testing, motor testing).
     * 
     * @param[in] method Method used for arming
     * 
     * @return true if vehicle successfully armed, false if arming denied
     * 
     * @warning Extremely dangerous - bypasses all safety checks
     * @warning Can result in immediate crash if sensors are not healthy
     * @warning Only use in controlled test environments
     * 
     * @see arm() for normal arming with checks
     */
    virtual bool arm_force(AP_Arming::Method method) { return arm(method, false); }
    
    /**
     * @brief Disarm the vehicle
     * 
     * @details Transitions vehicle from armed to disarmed state, stopping motor output.
     *          Can be initiated manually by pilot/GCS or automatically by failsafes.
     *          Logs the disarm event and updates internal state.
     * 
     *          Disarm sequence:
     *          1. If do_disarm_checks=true, verify disarm is safe
     *          2. Clear armed state
     *          3. Record disarm method
     *          4. Log disarming event
     *          5. Check if forced logging should continue
     *          6. Update GPIO outputs
     *          7. Send status to GCS
     * 
     * @param[in] method Reason for disarming (manual or failsafe trigger)
     * @param[in] do_disarm_checks If true, perform disarm safety checks
     * 
     * @return true if vehicle successfully disarmed, false if disarm denied
     * 
     * @note Copter vehicles use motors library for armed state
     * @warning Some failsafe disarms cannot be prevented even if checks fail
     * 
     * @see arm() for arming
     * @see Method enum for disarm reasons
     */
    virtual bool disarm(AP_Arming::Method method, bool do_disarm_checks=true);
    
    /**
     * @brief Check if vehicle is currently armed
     * 
     * @details Returns the current armed state. For most vehicles this reflects the state
     *          in this class. For Copter, this should query the motors library instead.
     * 
     * @return true if vehicle is armed, false if disarmed
     * 
     * @note Copter vehicles should query motors library, not this method
     * @see is_armed_and_safety_off() for armed state with safety switch consideration
     */
    bool is_armed() const;
    
    /**
     * @brief Check if vehicle is armed and hardware safety switch is disengaged
     * 
     * @details Returns true only if vehicle is armed AND the hardware safety switch (if present)
     *          is in the disengaged position. This is the complete "ready for motor output" state.
     * 
     * @return true if armed and safety off (motors can spin), false otherwise
     * 
     * @note Some boards do not have hardware safety switches
     * @see is_armed() for armed state without safety consideration
     */
    bool is_armed_and_safety_off() const;

    /**
     * @brief Get the time since boot when vehicle was armed
     * 
     * @details Returns the system microsecond timestamp when the vehicle successfully armed.
     *          Useful for determining flight time and triggering time-based behaviors.
     * 
     * @return Microseconds since boot when armed, or 0 if currently disarmed
     * 
     * @note Returns 0 if vehicle has never been armed or is currently disarmed
     * @note Uses system microsecond clock (wraps after ~70 days)
     */
    uint64_t arm_time_us() const { return is_armed() ? last_arm_time_us : 0; }

    /**
     * @brief Get bitmask of currently enabled arming checks
     * 
     * @details Returns the bitmask from ARMING_CHECK parameter indicating which check
     *          categories are currently enabled. Used for display and to filter check execution.
     * 
     * @return Bitmask of enabled checks (bitwise OR of Check enum values)
     * 
     * @note Value 0 means all non-mandatory checks disabled
     * @note Value -1 (all bits set) means all checks enabled
     * @see Check enum for bit definitions
     * @see check_enabled() to test individual check bits
     */
    uint32_t get_enabled_checks() const;

    /**
     * @brief Perform pre-arm safety checks
     * 
     * @details Executes comprehensive pre-flight validation checks across all enabled subsystems.
     *          Pre-arm checks are performed continuously (via update()) to provide early warning
     *          of issues before the pilot attempts to arm. These checks validate sensor health,
     *          calibration, configuration, and environmental conditions.
     * 
     *          Pre-arm checks include:
     *          - Sensor health (GPS, compass, IMU, barometer, airspeed, rangefinder)
     *          - Sensor calibration status and consistency
     *          - RC transmitter connection and calibration
     *          - Battery voltage and capacity
     *          - Mission and fence configuration
     *          - Logging availability
     *          - Safety switch state
     *          - Vehicle-specific checks
     * 
     *          Check reporting behavior:
     *          - If report=false: Checks run silently, result stored internally
     *          - If report=true: Failures sent as GCS status messages (throttled to avoid spam)
     *          - Reporting is throttled to prevent overwhelming telemetry link
     * 
     * @param[in] report If true, send status messages for failures; if false, silent check
     * 
     * @return true if all enabled pre-arm checks pass, false if any check fails
     * 
     * @note Virtual method - vehicle-specific implementations can add custom checks
     * @note Called automatically by update() at ~1Hz
     * @note Does not modify arming state - only validates readiness
     * @warning Some checks are mandatory and run even if ARMING_CHECK=0
     * 
     * @see arm_checks() for checks performed during arming attempt
     * @see mandatory_checks() for checks that cannot be bypassed
     * @see update() which calls this periodically
     * @see get_last_prearm_checks_result() to query cached result
     */
    virtual bool pre_arm_checks(bool report);
    
    /**
     * @brief Get result of last pre-arm check execution
     * 
     * @details Returns the cached result from the most recent pre_arm_checks() call.
     *          Useful for displaying pre-arm status without re-running all checks.
     * 
     * @return true if last pre-arm checks passed, false if they failed
     * 
     * @note Result updated by update() at ~1Hz
     * @see pre_arm_checks() for the actual check implementation
     */
    bool get_last_prearm_checks_result() const { return last_prearm_checks_result; }

    /**
     * @brief Perform arming-time safety checks
     * 
     * @details Executes checks that are specifically performed during an arming attempt.
     *          These checks have side-effects, require state changes, or validate conditions
     *          that only matter at the moment of arming (not continuously like pre-arm checks).
     * 
     *          Arm-time checks include:
     *          - RC input validation for the arming method
     *          - Mode-specific arming requirements
     *          - Throttle position validation
     *          - Vehicle-specific arming conditions
     * 
     *          These checks are separate from pre_arm_checks() because:
     *          - They depend on the arming method being used
     *          - They may have side-effects on vehicle state
     *          - They only need to be checked once during arming, not continuously
     * 
     * @param[in] method The method being used to arm (affects which checks apply)
     * 
     * @return true if all arm checks pass, false if any check fails
     * 
     * @note Virtual method - vehicle-specific implementations can add custom checks
     * @note Called by arm() during arming attempt
     * @note Failures reported as GCS status messages
     * 
     * @see arm() which calls this during arming
     * @see pre_arm_checks() for continuous validation
     * @see Method enum for possible arming methods
     */
    virtual bool arm_checks(AP_Arming::Method method);

    /**
     * @brief Get expected magnetic field strength for current location
     * 
     * @details Returns the expected magnetic field intensity (in milligauss) based on the
     *          world magnetic model for the vehicle's current GPS location. Used to validate
     *          compass calibration and detect magnetic interference.
     * 
     * @return Expected magnetic field strength in milligauss
     * 
     * @note Requires valid GPS position
     * @note Uses world magnetic model (WMM) lookup table
     * @see compass_checks() which uses this for validation
     */
    uint16_t compass_magfield_expected() const;

    /**
     * @enum RudderArming
     * @brief Rudder stick arming configuration options
     * 
     * @details Controls whether rudder stick input (right rudder + zero/low throttle) can be
     *          used to arm and/or disarm the vehicle. This is a traditional arming method but
     *          can be disabled to prevent accidental arming.
     * 
     *          Configured via ARMING_RUDDER parameter:
     *          - IS_DISABLED: Rudder arming completely disabled (safest)
     *          - ARMONLY: Rudder can arm but not disarm (requires switch/MAVLink to disarm)
     *          - ARMDISARM: Rudder can both arm and disarm
     * 
     * @note Rudder arming requires holding stick for ~1 second to prevent accidental triggering
     * @warning ARMDISARM mode can lead to accidental in-flight disarms
     * @see get_rudder_arming_type() to query current setting
     */
    enum class RudderArming {
        IS_DISABLED  = 0, ///< Rudder arming/disarming completely disabled (DISABLED macro conflicts)
        ARMONLY   = 1,    ///< Rudder can arm only, cannot disarm
        ARMDISARM = 2     ///< Rudder can both arm and disarm
    };

    /**
     * @brief Get the configured rudder arming mode
     * 
     * @details Returns the current rudder arming configuration from the ARMING_RUDDER parameter.
     * 
     * @return RudderArming enum value indicating rudder arming behavior
     * 
     * @see RudderArming enum for possible values
     */
    RudderArming get_rudder_arming_type() const { return (RudderArming)_rudder_arming.get(); }

#if AP_ARMING_AUX_AUTH_ENABLED
    /**
     * @brief Get auxiliary authorization ID for external auth system
     * 
     * @details Retrieves an authorization ID for use with external authorization systems.
     *          Allows integration with external security or approval mechanisms that must
     *          grant permission before vehicle can arm (e.g., air traffic control, range
     *          safety officer, automated verification systems).
     * 
     * @param[out] auth_id Assigned authorization ID (0 to aux_auth_count_max-1)
     * 
     * @return true if auth_id assigned successfully, false if no IDs available
     * 
     * @note Maximum 3 concurrent auxiliary authorizers
     * @note Thread-safe with semaphore protection
     * @see set_aux_auth_passed() to grant authorization
     * @see set_aux_auth_failed() to deny authorization
     */
    bool get_aux_auth_id(uint8_t& auth_id);
    
    /**
     * @brief Grant authorization for a specific auth ID
     * 
     * @details Marks the specified auxiliary authorizer as having granted permission to arm.
     *          Vehicle will not arm until all registered authorizers grant permission.
     * 
     * @param[in] auth_id Authorization ID from get_aux_auth_id()
     * 
     * @note Thread-safe with semaphore protection
     * @see get_aux_auth_id() to register as authorizer
     * @see aux_auth_checks() which validates all authorizations
     */
    void set_aux_auth_passed(uint8_t auth_id);
    
    /**
     * @brief Deny authorization for a specific auth ID
     * 
     * @details Marks the specified auxiliary authorizer as having denied permission to arm.
     *          Blocks arming and displays the provided failure message to the user.
     * 
     * @param[in] auth_id Authorization ID from get_aux_auth_id()
     * @param[in] fail_msg Error message to display (max 42 characters)
     * 
     * @note Thread-safe with semaphore protection
     * @note Message displayed as "PreArm: <fail_msg>"
     * @see get_aux_auth_id() to register as authorizer
     */
    void set_aux_auth_failed(uint8_t auth_id, const char* fail_msg);
    
    /**
     * @brief Reset all auxiliary authorizations to no-response state
     * 
     * @details Clears all auxiliary authorization states, requiring fresh authorization
     *          from all registered authorizers. Typically called when configuration changes
     *          or authorization timeout occurs.
     * 
     * @note Thread-safe with semaphore protection
     * @see get_aux_auth_id() to register authorizers
     */
    void reset_all_aux_auths();
#endif

    /**
     * @brief Parameter table for AP_Param system
     * 
     * @details Defines the parameter group structure for all arming configuration parameters.
     *          Loaded from persistent storage at init() and saved when parameters change.
     * 
     * @see AP_Param documentation for parameter system details
     */
    static const struct AP_Param::GroupInfo        var_info[];

    /**
     * @brief Get the method that was last used to disarm
     * 
     * @details Returns the method or reason for the most recent disarm event. Useful for
     *          logging, post-flight analysis, and determining if disarm was intentional or
     *          due to a failsafe.
     * 
     * @return Method enum value for last disarm (UNKNOWN if never disarmed)
     * 
     * @note Value is UNKNOWN until vehicle has been disarmed at least once
     * @see Method enum for possible disarm reasons
     * @see last_arm_method() for last arming method
     */
    Method last_disarm_method() const { return _last_disarm_method; }

    /**
     * @brief Get the method that was last used to arm
     * 
     * @details Returns the method used for the most recent successful arming. Useful for
     *          logging and understanding how the vehicle was armed.
     * 
     * @return Method enum value for last arm (UNKNOWN if never armed)
     * 
     * @note Value is UNKNOWN until vehicle has been armed at least once
     * @see Method enum for possible arming methods
     * @see last_disarm_method() for last disarming reason
     */
    Method last_arm_method() const { return _last_arm_method; }
    
    /**
     * @enum Option
     * @brief Arming behavior option flags (ARMING_OPTIONS parameter)
     * 
     * @details Bitmask options that modify arming system reporting and display behavior.
     *          Multiple options can be enabled simultaneously by bitwise OR.
     * 
     *          Use cases:
     *          - Reduce GCS message traffic for applications with limited bandwidth
     *          - Suppress pre-arm messages when running automated testing
     *          - Customize user feedback for specialized applications
     * 
     * @see option_enabled() to test if an option is enabled
     */
    enum class Option : int32_t {
        DISABLE_PREARM_DISPLAY             = (1U << 0), ///< Suppress periodic pre-arm failure messages (still sent on state change)
        DISABLE_STATUSTEXT_ON_STATE_CHANGE = (1U << 1), ///< Suppress arm/disarm status text messages completely
    };
    
    /**
     * @brief Check if a specific arming option is enabled
     * 
     * @details Tests the ARMING_OPTIONS parameter bitmask for the specified option bit.
     * 
     * @param[in] option The Option enum value to test
     * 
     * @return true if option is enabled, false if disabled
     * 
     * @see Option enum for available options
     */
    bool option_enabled(Option option) const {
        return (_arming_options & uint32_t(option)) != 0;
    }

    /**
     * @brief Send arm/disarm status text message to GCS
     * 
     * @details Sends a status text string to the ground control station, respecting the
     *          DISABLE_STATUSTEXT_ON_STATE_CHANGE option. Used internally for user feedback.
     * 
     * @param[in] string Status message to send (null-terminated C string)
     * 
     * @note Message suppressed if DISABLE_STATUSTEXT_ON_STATE_CHANGE option enabled
     * @note Const method - can be called from const contexts
     */
    void send_arm_disarm_statustext(const char *string) const;

    /**
     * @brief Check if arming method is from a ground control station
     * 
     * @details Determines if the specified method represents a GCS command (MAVLink or DDS).
     *          Useful for applying different validation or behavior for GCS vs pilot commands.
     * 
     * @param[in] method The Method enum value to test
     * 
     * @return true if method is MAVLINK or DDS, false otherwise
     * 
     * @note Static method - can be called without an instance
     * @see Method enum for all possible methods
     */
    static bool method_is_GCS(Method method) {
        return (method == Method::MAVLINK || method == Method::DDS);
    }

    /**
     * @enum RequireLocation
     * @brief GPS location requirement for arming
     * 
     * @details Controls whether a valid GPS position is required before the vehicle can arm.
     *          Configured via ARMING_REQUIRE_LOC parameter.
     * 
     *          Safety consideration:
     *          - YES: Ensures vehicle has position estimate before flight (recommended)
     *          - NO: Allows arming without GPS (dangerous for autonomous modes)
     * 
     * @warning Arming without GPS prevents position-based modes and failsafes
     * @see gps_checks() for GPS validation
     */
    enum class RequireLocation : uint8_t {
        NO = 0,   ///< GPS position not required for arming (unsafe for autonomous flight)
        YES = 1,  ///< GPS position required for arming (recommended)
    };

protected:

    // ========================================================================
    // Configuration Parameters (loaded from persistent storage)
    // ========================================================================
    
    /**
     * @brief ARMING_REQUIRE - Arming requirement mode
     * 
     * @details Controls whether arming is required and what throttle position is needed.
     *          See Required enum for possible values.
     * 
     * @note Parameter name: ARMING_REQUIRE
     */
    AP_Enum<Required>       require;
    
    /**
     * @brief ARMING_CHECK - Bitmask of enabled pre-arm checks
     * 
     * @details Bitmask controlling which check categories are active. Each bit corresponds
     *          to a Check enum value. Setting to 0 disables all non-mandatory checks.
     *          Setting to -1 (all bits) enables all checks.
     * 
     * @note Parameter name: ARMING_CHECK
     * @see Check enum for bit definitions
     */
    AP_Int32                checks_to_perform;
    
    /**
     * @brief ARMING_ACCTHRESH - Accelerometer consistency threshold
     * 
     * @details Maximum acceptable difference (in m/s²) between multiple accelerometers.
     *          Used to detect IMU hardware failures. Typical value: 0.75 m/s².
     * 
     * @note Parameter name: ARMING_ACCTHRESH
     * @note Range: 0.25 to 3.0 m/s²
     * @see ins_checks() for accelerometer consistency validation
     */
    AP_Float                accel_error_threshold;
    
    /**
     * @brief ARMING_RUDDER - Rudder arming enable/disable
     * 
     * @details Controls whether rudder stick can be used to arm/disarm.
     *          See RudderArming enum for possible values.
     * 
     * @note Parameter name: ARMING_RUDDER
     * @see RudderArming enum for options
     */
    AP_Int8                 _rudder_arming;
    
    /**
     * @brief ARMING_MIS_ITEMS - Required mission item checks
     * 
     * @details Bitmask of mission item types that must be present in mission for arming.
     *          See MIS_ITEM_CHECK enum in private section for bit definitions.
     *          Ensures missions include critical items like takeoff and land commands.
     * 
     * @note Parameter name: ARMING_MIS_ITEMS
     * @see mission_checks() for mission validation
     */
    AP_Int32                _required_mission_items;
    
    /**
     * @brief ARMING_OPTIONS - Arming behavior option flags
     * 
     * @details Bitmask of behavior modification options. See Option enum for bit definitions.
     *          Controls message reporting and display behavior.
     * 
     * @note Parameter name: ARMING_OPTIONS
     * @see Option enum for available options
     */
    AP_Int32                _arming_options;
    
    /**
     * @brief ARMING_MAGTHRESH - Compass magnetic field strength threshold
     * 
     * @details Maximum acceptable deviation (in percent) of measured magnetic field strength
     *          from expected value based on world magnetic model. Used to detect magnetic
     *          interference. Typical value: 30%.
     * 
     * @note Parameter name: ARMING_MAGTHRESH
     * @note Range: 0 to 100 percent
     * @note Value 0 disables magnetic field strength check
     * @see compass_checks() for compass validation
     * @see compass_magfield_expected() for expected field strength
     */
    AP_Int16                magfield_error_threshold;
    
    /**
     * @brief ARMING_REQUIRE_LOC - GPS location requirement
     * 
     * @details Controls whether valid GPS position is required for arming.
     *          See RequireLocation enum for possible values.
     * 
     * @note Parameter name: ARMING_REQUIRE_LOC
     * @see RequireLocation enum for options
     */
    AP_Enum<RequireLocation> require_location;

    // ========================================================================
    // Internal State Variables
    // ========================================================================
    
    /**
     * @brief Current armed state
     * 
     * @note For Copter, armed state stored in motors library, not here
     */
    bool                    armed;
    
    /**
     * @brief Timestamp of last successful accelerometer consistency check
     * 
     * @details System millisecond timestamp when accelerometers were last verified consistent.
     *          Used for rate-limiting accelerometer checks and detecting persistent failures.
     */
    uint32_t                last_accel_pass_ms;
    
    /**
     * @brief Timestamp of last successful gyroscope consistency check
     * 
     * @details System millisecond timestamp when gyroscopes were last verified consistent.
     *          Used for rate-limiting gyroscope checks and detecting persistent failures.
     */
    uint32_t                last_gyro_pass_ms;

    // ========================================================================
    // Protected Check Methods (vehicle-specific implementations may override)
    // ========================================================================

    /**
     * @brief Check barometer health and calibration
     * 
     * @details Validates:
     *          - At least one barometer is healthy and providing data
     *          - Barometer has been calibrated (altitude initialization)
     *          - Pressure readings are within reasonable bounds
     *          - Multi-barometer consistency if multiple sensors present
     * 
     *          Typical failures:
     *          - No barometer detected
     *          - Barometer unhealthy
     *          - Barometer not calibrated
     *          - Excessive altitude drift
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if barometer checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @see AP_Baro library for barometer implementation
     */
    virtual bool barometer_checks(bool report);

    /**
     * @brief Check airspeed sensor health (fixed-wing)
     * 
     * @details Validates:
     *          - Airspeed sensor is healthy if present and required
     *          - Sensor has been calibrated
     *          - Reading is within reasonable range
     * 
     *          Typical failures:
     *          - Required airspeed sensor not detected
     *          - Sensor unhealthy
     *          - Not calibrated
     *          - Reading stuck or unreasonable
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if airspeed checks pass, false on failure
     * 
     * @note Primarily used by fixed-wing vehicles
     * @see AP_Airspeed library for sensor implementation
     */
    bool airspeed_checks(bool report);

    /**
     * @brief Check logging system availability
     * 
     * @details Validates:
     *          - Logging is initialized and ready
     *          - SD card is present and writable (if required)
     *          - Sufficient storage space available
     * 
     *          Typical failures:
     *          - No SD card detected
     *          - SD card full
     *          - Logging system error
     *          - File system not writable
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if logging checks pass, false on failure
     * 
     * @note Some configurations require logging for arming
     * @see AP_Logger library for logging system
     */
    bool logging_checks(bool report);

#if AP_INERTIALSENSOR_ENABLED
    /**
     * @brief Check IMU (Inertial Measurement Unit) health
     * 
     * @details Validates:
     *          - At least one IMU is healthy and calibrated
     *          - Accelerometer calibration complete
     *          - Gyroscope calibration complete
     *          - Multiple IMU consistency (if multiple sensors)
     *          - No excessive vibration detected
     *          - Temperature within operating range
     * 
     *          Typical failures:
     *          - IMU not calibrated
     *          - Accelerometers inconsistent (hardware failure)
     *          - Gyroscopes inconsistent (hardware failure)
     *          - Excessive vibration
     *          - IMU unhealthy
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if INS checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @warning IMU failures are critical - vehicle will not have reliable attitude
     * @see AP_InertialSensor library for IMU implementation
     * @see accel_error_threshold parameter for consistency threshold
     */
    virtual bool ins_checks(bool report);
#endif

    /**
     * @brief Check compass health and calibration
     * 
     * @details Validates:
     *          - At least one compass is healthy
     *          - Compass has been calibrated (offsets learned)
     *          - Magnetic field strength matches expected value for location
     *          - Multiple compass consistency if multiple sensors
     *          - No excessive magnetic interference detected
     *          - Primary compass properly configured
     * 
     *          Typical failures:
     *          - Compass not calibrated
     *          - Magnetic field inconsistent with location (poor calibration)
     *          - Excessive magnetic offsets (interference)
     *          - Compasses inconsistent (hardware failure)
     *          - No compass detected
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if compass checks pass, false on failure
     * 
     * @warning Compass failures prevent reliable heading estimation
     * @see AP_Compass library for compass implementation
     * @see magfield_error_threshold parameter for field strength tolerance
     * @see compass_magfield_expected() for expected field strength
     */
    bool compass_checks(bool report);

    /**
     * @brief Check GPS health and fix quality
     * 
     * @details Validates:
     *          - GPS receiver is detected and healthy
     *          - GPS has valid 3D fix (if required)
     *          - Sufficient satellites tracked (typically 6+)
     *          - Position accuracy (HDOP) acceptable
     *          - GPS time initialized
     *          - Multiple GPS consistency if blending enabled
     * 
     *          Typical failures:
     *          - No GPS detected
     *          - No GPS fix (insufficient satellites)
     *          - Poor GPS accuracy (high HDOP)
     *          - GPS unhealthy
     *          - GPS time not valid
     *          - Blended GPS inconsistent
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if GPS checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @warning GPS failures prevent position-based flight modes
     * @see AP_GPS library for GPS implementation
     * @see require_location parameter for GPS requirement
     */
    virtual bool gps_checks(bool report);

    /**
     * @brief Check battery voltage and capacity
     * 
     * @details Validates:
     *          - Battery monitor is healthy and providing data
     *          - Battery voltage above minimum threshold
     *          - Sufficient battery capacity remaining
     *          - Battery failsafe thresholds not triggered
     *          - Multi-battery consistency if multiple batteries
     * 
     *          Typical failures:
     *          - Battery voltage too low
     *          - Insufficient capacity remaining
     *          - Battery monitor unhealthy
     *          - Battery failsafe triggered
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if battery checks pass, false on failure
     * 
     * @warning Low battery during arming can lead to mid-flight power loss
     * @see AP_BattMonitor library for battery monitoring
     */
    bool battery_checks(bool report);

    /**
     * @brief Check hardware safety switch state
     * 
     * @details Validates:
     *          - Safety switch is in correct state for arming
     *          - Switch hardware is functioning
     * 
     *          Typical failures:
     *          - Safety switch still engaged (not ready for motor output)
     *          - Safety switch hardware fault
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if safety switch checks pass, false on failure
     * 
     * @note Some boards do not have hardware safety switches
     * @warning Safety switch must be disengaged for motor output
     */
    bool hardware_safety_check(bool report);

    /**
     * @brief Check board power supply voltage
     * 
     * @details Validates:
     *          - Board power supply voltage is within safe operating range
     *          - No low-voltage conditions detected
     *          - Power supply is stable
     * 
     *          Typical failures:
     *          - Board voltage too low (brown-out risk)
     *          - Unstable power supply
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if voltage checks pass, false on failure
     * 
     * @note Virtual - board-specific implementations may add custom checks
     * @warning Low board voltage can cause processor resets
     */
    virtual bool board_voltage_checks(bool report);

    /**
     * @brief Check RC receiver calibration
     * 
     * @details Validates:
     *          - RC channels are properly calibrated (min/max/trim set)
     *          - Channel ranges are reasonable
     *          - No channels inverted incorrectly
     * 
     *          Typical failures:
     *          - RC not calibrated
     *          - Invalid channel ranges
     *          - Calibration data corrupted
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if calibration checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @see RC_Channel library for RC input
     */
    virtual bool rc_calibration_checks(bool report);

    /**
     * @brief Check if RC input calibration is in progress
     * 
     * @details Prevents arming while RC calibration procedure is active, as stick
     *          movements during calibration would be misinterpreted.
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if not calibrating, false if calibration in progress
     */
    bool rc_in_calibration_check(bool report);

    /**
     * @brief Check RC requirements for arming method
     * 
     * @details Performs arming-method-specific RC validations such as verifying stick
     *          positions for rudder arming or checking switch states for switch arming.
     * 
     * @param[in] method Arming method being attempted
     * 
     * @return true if RC requirements met, false otherwise
     * 
     * @see Method enum for arming methods
     */
    bool rc_arm_checks(AP_Arming::Method method);

    /**
     * @brief Check manual RC transmitter connection
     * 
     * @details Validates:
     *          - RC receiver is receiving valid input
     *          - Sufficient RC channels available
     *          - No RC failsafe triggered
     * 
     *          Typical failures:
     *          - No RC input detected
     *          - RC failsafe active
     *          - Insufficient channels
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if transmitter checks pass, false on failure
     */
    bool manual_transmitter_checks(bool report);

    /**
     * @brief Check mission configuration and validity
     * 
     * @details Validates:
     *          - Mission uploaded if required for flight mode
     *          - Mission contains required items (takeoff, land, rally, RTL)
     *          - Mission waypoints are valid
     *          - No conflicting mission commands
     * 
     *          Typical failures:
     *          - No mission uploaded when required
     *          - Missing required mission items (e.g., no LAND command)
     *          - Invalid mission waypoints
     *          - Mission configuration error
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if mission checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @see AP_Mission library for mission management
     * @see _required_mission_items parameter for required item configuration
     */
    virtual bool mission_checks(bool report);

    /**
     * @brief Check terrain database status
     * 
     * @details Validates:
     *          - Terrain database is initialized if required
     *          - Terrain data available for current location
     *          - Terrain system is healthy
     * 
     *          Typical failures:
     *          - No terrain data available
     *          - Terrain database not loaded
     *          - Terrain system error
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if terrain checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see AP_Terrain library for terrain database
     */
    bool terrain_checks(bool report) const;

    /**
     * @brief Query if terrain database is required to be fully loaded
     * 
     * @details Virtual method allowing vehicle-specific implementations to specify whether
     *          complete terrain data coverage is required for the current flight mode or
     *          vehicle configuration.
     * 
     * @return true if terrain database must be fully loaded, false otherwise
     * 
     * @note Virtual - default implementation returns false
     * @note Const method - does not modify state
     * @see terrain_checks() which uses this to determine requirements
     */
    virtual bool terrain_database_required() const;

    /**
     * @brief Check rangefinder (distance sensor) health
     * 
     * @details Validates:
     *          - Rangefinder is healthy if present and required
     *          - Sensor is providing valid range data
     *          - No persistent sensor errors
     * 
     *          Typical failures:
     *          - Required rangefinder not detected
     *          - Rangefinder unhealthy
     *          - No valid range data
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if rangefinder checks pass, false on failure
     * 
     * @see AP_RangeFinder library for sensor implementation
     */
    bool rangefinder_checks(bool report);

    /**
     * @brief Check geofence configuration
     * 
     * @details Validates:
     *          - Geofence is properly configured if enabled
     *          - Fence parameters are valid
     *          - Vehicle is within fence boundaries at arm time
     *          - No fence configuration errors
     * 
     *          Typical failures:
     *          - Invalid fence configuration
     *          - Vehicle outside fence at arming
     *          - Fence boundary conflicts
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if fence checks pass, false on failure
     * 
     * @see AC_Fence library for geofencing implementation
     */
    bool fence_checks(bool report);

#if HAL_HAVE_IMU_HEATER
    /**
     * @brief Check IMU heater minimum temperature requirement
     * 
     * @details Validates that IMU temperature has reached minimum operating temperature
     *          when heater is enabled. Prevents arming with cold IMU which would have
     *          degraded calibration accuracy.
     * 
     *          Typical failures:
     *          - IMU temperature below minimum threshold
     *          - Heater not functioning
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if temperature requirements met, false otherwise
     * 
     * @note Only compiled on boards with IMU heater support
     */
    bool heater_min_temperature_checks(bool report);
#endif

    /**
     * @brief Check camera/trigger system readiness
     * 
     * @details Validates camera backend is initialized and ready if camera features enabled.
     * 
     *          Typical failures:
     *          - Camera backend initialization error
     *          - Camera communication failure
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if camera checks pass, false on failure
     * 
     * @see AP_Camera library for camera control
     */
    bool camera_checks(bool display_failure);

    /**
     * @brief Check on-screen display system health
     * 
     * @details Validates OSD backend is initialized and functioning if OSD enabled.
     * 
     *          Typical failures:
     *          - OSD backend error
     *          - OSD communication failure
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if OSD checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see AP_OSD library for OSD implementation
     */
    bool osd_checks(bool display_failure) const;

    /**
     * @brief Check gimbal mount system readiness
     * 
     * @details Validates gimbal mount backend is initialized and ready if mount enabled.
     * 
     *          Typical failures:
     *          - Mount backend error
     *          - Mount communication failure
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if mount checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see AP_Mount library for gimbal control
     */
    bool mount_checks(bool display_failure) const;

#if AP_ARMING_AUX_AUTH_ENABLED
    /**
     * @brief Check auxiliary authorization status
     * 
     * @details Validates that all registered auxiliary authorizers have granted permission
     *          to arm. Blocks arming if any authorizer has denied or not responded.
     * 
     *          Typical failures:
     *          - Auxiliary authorization timeout
     *          - Authorization explicitly denied
     *          - Authorizer not responding
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if all authorizations granted, false otherwise
     * 
     * @see get_aux_auth_id() for registering authorizers
     * @see AuxAuthStates enum for authorization states
     */
    bool aux_auth_checks(bool display_failure);
#endif

    /**
     * @brief Check generator system status
     * 
     * @details Validates generator is healthy and providing adequate power if present.
     * 
     *          Typical failures:
     *          - Generator not running when required
     *          - Generator error state
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if generator checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see AP_Generator library for generator monitoring
     */
    bool generator_checks(bool report) const;

    /**
     * @brief Check OpenDroneID remote identification compliance
     * 
     * @details Validates OpenDroneID system is functional and broadcasting required
     *          identification information for regulatory compliance.
     * 
     *          Typical failures:
     *          - OpenDroneID not broadcasting
     *          - Missing required identification data
     *          - OpenDroneID hardware error
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if OpenDroneID checks pass, false on failure
     * 
     * @see AP_OpenDroneID library for remote ID implementation
     */
    bool opendroneid_checks(bool display_failure);
    
    /**
     * @brief Check serial protocol configuration
     * 
     * @details Validates required serial protocols are properly configured and initialized.
     * 
     *          Typical failures:
     *          - Required protocol not configured
     *          - Serial port initialization error
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if serial protocol checks pass, false on failure
     */
    bool serial_protocol_checks(bool display_failure);
    
    /**
     * @brief Check emergency stop (e-stop) status
     * 
     * @details Validates emergency stop is not currently triggered. E-stop completely
     *          disables vehicle until manually cleared.
     * 
     *          Typical failures:
     *          - E-stop button pressed
     *          - E-stop signal active
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if e-stop not active, false if triggered
     * 
     * @warning E-stop must be cleared before vehicle can arm
     */
    bool estop_checks(bool display_failure);

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    /**
     * @brief Check that crash dump has been acknowledged
     * 
     * @details If a previous crash dump exists, requires operator acknowledgment before
     *          allowing arming. Ensures operator is aware of previous crash/error condition.
     * 
     *          Typical failures:
     *          - Unacknowledged crash dump present
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if no pending crash dump or acknowledged, false otherwise
     * 
     * @note Only compiled when crash dump feature enabled
     */
    bool crashdump_checks(bool report);
#endif

    /**
     * @brief Check overall system health
     * 
     * @details Validates:
     *          - CPU load within acceptable limits
     *          - Sufficient free memory available
     *          - No critical system errors
     *          - Board health indicators normal
     * 
     *          Typical failures:
     *          - CPU overload
     *          - Low memory
     *          - Hardware fault detected
     *          - Critical system error
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if system checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @warning System failures can cause in-flight failures
     */
    virtual bool system_checks(bool report);

    /**
     * @brief Check CAN bus status
     * 
     * @details Validates CAN bus interfaces are functioning and required CAN peripherals
     *          are responding (if CAN is used).
     * 
     *          Typical failures:
     *          - CAN bus error
     *          - Required CAN device not responding
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if CAN checks pass, false on failure
     * 
     * @see AP_CANManager library for CAN bus management
     */
    bool can_checks(bool report);

    /**
     * @brief Check FETtec OneWire ESC status
     * 
     * @details Validates FETtec OneWire ESCs are responding and healthy if configured.
     * 
     *          Typical failures:
     *          - FETtec ESC not responding
     *          - ESC error state
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if FETtec checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see AP_FETtecOneWire library for ESC implementation
     */
    bool fettec_checks(bool display_failure) const;

#if HAL_PROXIMITY_ENABLED
    /**
     * @brief Check proximity sensor system health
     * 
     * @details Validates proximity sensors are healthy if present and required for
     *          obstacle avoidance.
     * 
     *          Typical failures:
     *          - Required proximity sensor not detected
     *          - Proximity sensor unhealthy
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if proximity checks pass, false on failure
     * 
     * @note Virtual - vehicle-specific implementations may add custom checks
     * @note Const method - does not modify state
     * @see AP_Proximity library for proximity sensors
     */
    virtual bool proximity_checks(bool report) const;
#endif

    /**
     * @brief Check servo output configuration
     * 
     * @details Validates servo outputs are properly configured and responding.
     * 
     *          Typical failures:
     *          - Servo configuration error
     *          - Servo not responding
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if servo checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see SRV_Channel library for servo management
     */
    bool servo_checks(bool report) const;
    
    /**
     * @brief RC checks specific to Copter and Sub vehicles
     * 
     * @details Performs RC validation specific to multicopter and submarine vehicles,
     *          checking the four primary control channels.
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * @param[in] channels Array of 4 RC channel pointers (roll, pitch, throttle, yaw)
     * 
     * @return true if RC checks pass, false on failure
     * 
     * @note Const method - does not modify state
     */
    bool rc_checks_copter_sub(bool display_failure, const class RC_Channel *channels[4]) const;

    /**
     * @brief Check visual odometry system health
     * 
     * @details Validates visual odometry is healthy if enabled and required for navigation.
     * 
     *          Typical failures:
     *          - Visual odometry not initialized
     *          - Position estimate quality poor
     *          - Vision sensor timeout
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if visual odometry checks pass, false on failure
     * 
     * @note Const method - does not modify state
     * @see AP_VisualOdom library for vision integration
     */
    bool visodom_checks(bool report) const;
    
    /**
     * @brief Check disarm switch status
     * 
     * @details Prevents arming if the disarm switch is in the active position.
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if disarm switch not active, false if active
     * 
     * @note Const method - does not modify state
     */
    bool disarm_switch_checks(bool report) const;

    /**
     * @brief Perform mandatory checks that cannot be bypassed
     * 
     * @details Executes safety-critical checks that must pass regardless of ARMING_CHECK
     *          parameter setting. These checks validate conditions essential for basic
     *          flight safety and cannot be disabled.
     * 
     *          Mandatory checks typically include:
     *          - Critical sensor health (IMU, GPS in certain modes)
     *          - Safety switch state
     *          - Critical configuration errors
     *          - Hardware failures
     * 
     *          This method is only called when:
     *          - ARMING_CHECK parameter is set to 0 (all checks disabled), OR
     *          - Force arming is used
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if all mandatory checks pass, false if any fail
     * 
     * @note Virtual - vehicle-specific implementations define mandatory checks
     * @warning These checks enforce minimum safety requirements
     * @warning Cannot be bypassed even with ARMING_CHECK=0
     * @warning Failures indicate serious safety issues that must be resolved
     * 
     * @see arm() which calls this when checks are bypassed
     */
    virtual bool mandatory_checks(bool report);

    /**
     * @brief Test if a specific check category is enabled
     * 
     * @details Queries the checks_to_perform bitmask to determine if the specified
     *          check category is currently enabled via ARMING_CHECK parameter.
     * 
     * @param[in] check The Check enum value to test
     * 
     * @return true if check is enabled, false if disabled
     * 
     * @note Const method - does not modify state
     * @see Check enum for check categories
     * @see checks_to_perform parameter for bitmask storage
     */
    bool check_enabled(const AP_Arming::Check check) const;
    
    /**
     * @brief Report a check failure with check category
     * 
     * @details Formats and sends a check failure message to the GCS if reporting is enabled
     *          and the specified check category is active. Automatically prepends "PreArm: "
     *          or "Arm: " prefix based on arming context.
     * 
     * @param[in] check The check category that failed
     * @param[in] report If true, send message to GCS; if false, silent
     * @param[in] fmt Printf-style format string for message
     * @param[in] ... Variable arguments for format string
     * 
     * @note Const method - does not modify state (only sends messages)
     * @note Message automatically throttled to prevent GCS spam
     * @see check_enabled() to test if check is enabled
     */
    void check_failed(const AP_Arming::Check check, bool report, const char *fmt, ...) const FMT_PRINTF(4, 5);
    
    /**
     * @brief Report a check failure without check category
     * 
     * @details Formats and sends a check failure message to the GCS if reporting is enabled.
     *          Used for failures not associated with a specific check category.
     * 
     * @param[in] report If true, send message to GCS; if false, silent
     * @param[in] fmt Printf-style format string for message
     * @param[in] ... Variable arguments for format string
     * 
     * @note Const method - does not modify state (only sends messages)
     * @note Message automatically throttled to prevent GCS spam
     */
    void check_failed(bool report, const char *fmt, ...) const FMT_PRINTF(3, 4);

    /**
     * @brief Log arming event to dataflash
     * 
     * @details Writes ARM log message recording:
     *          - Timestamp of arming
     *          - Arming method used
     *          - Whether checks were forced/bypassed
     *          - Current time since boot
     * 
     * @param[in] forced True if arming checks were bypassed
     * @param[in] method Method enum indicating how vehicle was armed
     * 
     * @note Called automatically by arm() method
     * @see Log_Write_Disarm() for corresponding disarm logging
     * @see AP_Logger for log message format
     */
    void Log_Write_Arm(bool forced, AP_Arming::Method method);
    
    /**
     * @brief Log disarming event to dataflash
     * 
     * @details Writes DISARM log message recording:
     *          - Timestamp of disarming
     *          - Disarm method/reason
     *          - Whether disarm was forced
     *          - Flight time duration
     * 
     * @param[in] forced True if disarm checks were bypassed
     * @param[in] method Method enum indicating disarm reason (user, failsafe, crash, etc.)
     * 
     * @note Called automatically by disarm() method
     * @see Log_Write_Arm() for corresponding arm logging
     * @see AP_Logger for log message format
     */
    void Log_Write_Disarm(bool forced, AP_Arming::Method method);

private:

    /**
     * @brief Singleton instance pointer
     * 
     * @details Static pointer to the single AP_Arming instance, accessed via get_singleton().
     *          Ensures only one arming state manager exists per vehicle.
     */
    static AP_Arming *_singleton;

#if AP_INERTIALSENSOR_ENABLED
    /**
     * @brief Check accelerometer consistency across multiple IMUs
     * 
     * @details Compares accelerometer readings from all available IMUs to ensure they
     *          agree within configured thresholds. Detects IMU failures or calibration
     *          errors by identifying outlier sensors.
     * 
     * @param[in] ins Reference to AP_InertialSensor instance
     * 
     * @return true if all accelerometers are consistent, false if divergence detected
     * 
     * @note Uses accel_error_threshold parameter for tolerance
     * @see ins_gyros_consistent() for gyro consistency check
     */
    bool ins_accels_consistent(const class AP_InertialSensor &ins);
    
    /**
     * @brief Check gyroscope consistency across multiple IMUs
     * 
     * @details Compares gyroscope readings from all available IMUs to ensure they
     *          agree within configured thresholds. Detects IMU failures or calibration
     *          errors by identifying outlier sensors.
     * 
     * @param[in] ins Reference to AP_InertialSensor instance
     * 
     * @return true if all gyroscopes are consistent, false if divergence detected
     * 
     * @see ins_accels_consistent() for accelerometer consistency check
     */
    bool ins_gyros_consistent(const class AP_InertialSensor &ins);
#endif

    /**
     * @brief Determine if logging should continue after disarming
     * 
     * @details Analyzes the disarm method/reason to determine if logging should be
     *          force-continued after disarm. Certain disarm reasons (crashes, failsafes)
     *          trigger extended logging to capture post-event data for analysis.
     * 
     * @param[in] method The Method enum value indicating disarm reason
     * 
     * @note Modifies logging behavior but does not return a value
     * @see Log_Write_Disarm() which calls this method
     */
    void check_forced_logging(const AP_Arming::Method method);

    /**
     * @enum MIS_ITEM_CHECK
     * @brief Mission item requirement flags
     * 
     * @details Bitmask flags specifying which mission items must be present in a valid
     *          mission. Used by mission_checks() to validate mission completeness based
     *          on vehicle type and configuration.
     */
    enum MIS_ITEM_CHECK {
        MIS_ITEM_CHECK_LAND          = (1 << 0),  ///< Mission must contain LAND command
        MIS_ITEM_CHECK_VTOL_LAND     = (1 << 1),  ///< Mission must contain VTOL_LAND command
        MIS_ITEM_CHECK_DO_LAND_START = (1 << 2),  ///< Mission must contain DO_LAND_START command
        MIS_ITEM_CHECK_TAKEOFF       = (1 << 3),  ///< Mission must contain TAKEOFF command
        MIS_ITEM_CHECK_VTOL_TAKEOFF  = (1 << 4),  ///< Mission must contain VTOL_TAKEOFF command
        MIS_ITEM_CHECK_RALLY         = (1 << 5),  ///< Rally points must be defined
        MIS_ITEM_CHECK_RETURN_TO_LAUNCH = (1 << 6),  ///< Mission must contain RTL command
        MIS_ITEM_CHECK_MAX                        ///< Maximum flag value (boundary marker)
    };

#if AP_ARMING_AUX_AUTH_ENABLED
    /**
     * @brief Maximum number of auxiliary authorizers
     * 
     * @details Limits the number of external systems that can independently authorize
     *          or deny arming requests.
     */
    static const uint8_t aux_auth_count_max = 3;
    
    /**
     * @brief Maximum length of auxiliary authorization failure message
     * 
     * @details Message length is 42 characters (50 total minus 8 for "PreArm: " prefix).
     */
    static const uint8_t aux_auth_str_len = 42;
    
    /**
     * @enum AuxAuthStates
     * @brief Auxiliary authorization state values
     * 
     * @details Tracks the response state of each registered auxiliary authorizer.
     */
    enum class AuxAuthStates : uint8_t {
        NO_RESPONSE = 0,  ///< Authorizer has not responded yet
        AUTH_FAILED,      ///< Authorizer explicitly denied arming
        AUTH_PASSED       ///< Authorizer granted arming permission
    } aux_auth_state[aux_auth_count_max] = {};  ///< State of each auxiliary authorizer
    
    uint8_t aux_auth_count;           ///< Number of registered auxiliary authorizers
    uint8_t aux_auth_fail_msg_source; ///< Authorization ID that set failure message
    char* aux_auth_fail_msg;          ///< Buffer holding authorization failure message
    bool aux_auth_error;              ///< True if too many authorizers registered
    HAL_Semaphore aux_auth_sem;       ///< Semaphore protecting aux_auth_state and aux_auth_fail_msg
#endif

    /**
     * @brief Last method used for disarming
     * 
     * @details Stores the Method enum value for the most recent disarm operation.
     *          Invalid (UNKNOWN) until vehicle has been disarmed at least once.
     * 
     * @see last_disarm_method() accessor
     */
    Method _last_disarm_method = Method::UNKNOWN;
    
    /**
     * @brief Last method used for arming
     * 
     * @details Stores the Method enum value for the most recent arm operation.
     *          Invalid (UNKNOWN) until vehicle has been armed at least once.
     * 
     * @see last_arm_method() accessor
     */
    Method _last_arm_method = Method::UNKNOWN;

    /**
     * @brief Timestamp of last successful arming (microseconds)
     * 
     * @details Stores micros64() timestamp when vehicle was armed. Zero when disarmed.
     *          Used to calculate flight time and enforce time-based constraints.
     * 
     * @see arm_time_us() accessor
     */
    uint64_t last_arm_time_us;

    /**
     * @brief Last time pre-arm messages were displayed (milliseconds)
     * 
     * @details Timestamp used to throttle pre-arm failure status text messages to GCS.
     *          Prevents excessive message spam during continuous pre-arm checking.
     */
    uint32_t last_prearm_display_ms;
    
    /**
     * @brief Flag indicating arming checks are for actual arm attempt
     * 
     * @details True when checks are running because vehicle is actively trying to arm.
     *          False during periodic pre-arm checks. Affects reporting behavior.
     */
    bool running_arming_checks;
    
    /**
     * @brief Result of most recent pre_arm_checks() call
     * 
     * @details Caches the result to detect transitions from pass to fail.
     * 
     * @see get_last_prearm_checks_result() accessor
     */
    bool last_prearm_checks_result;
    
    /**
     * @brief Flag to trigger immediate failure reporting
     * 
     * @details Set to true when check transitions from passing to failing, causing
     *          immediate GCS notification rather than waiting for throttle interval.
     */
    bool report_immediately;

    /**
     * @brief Update GPIO pin reflecting armed state
     * 
     * @details Sets hardware GPIO output to indicate armed/disarmed state for external
     *          hardware integration (lights, interlocks, etc.).
     * 
     * @note Called automatically during arm/disarm state changes
     */
    void update_arm_gpio();

#if !AP_GPS_BLENDED_ENABLED
    /**
     * @brief Check GPS blending auto-switch configuration
     * 
     * @details Validates GPS configuration when blending is not enabled. Ensures GPS
     *          switching behavior is appropriate for single GPS operation.
     * 
     * @param[in] report If true, send failure messages to GCS
     * 
     * @return true if GPS configuration valid, false otherwise
     * 
     * @note Only compiled when GPS blending is not enabled
     */
    bool blending_auto_switch_checks(bool report);
#endif

#if AP_ARMING_CRASHDUMP_ACK_ENABLED
    /**
     * @struct CrashDump
     * @brief Crash dump acknowledgment state
     * 
     * @details Manages crash dump acknowledgment requirement and reset logic.
     */
    struct CrashDump {
        /**
         * @brief Check and reset crash dump acknowledgment
         * 
         * @details Resets the acked state after successful acknowledgment and arming.
         */
        void check_reset();
        
        AP_Int8  acked;  ///< Parameter: 1 = crash dump acknowledged, 0 = unacknowledged
    } crashdump_ack;  ///< Crash dump acknowledgment state
#endif  // AP_ARMING_CRASHDUMP_ACK_ENABLED

};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Get reference to AP_Arming singleton instance
     * 
     * @details Provides global access to the arming system singleton. Preferred access
     *          method over get_singleton() for consistency with other AP subsystems.
     * 
     * @return Reference to the AP_Arming singleton
     * 
     * @note Throws assertion if called before singleton is initialized
     * @see AP_Arming::get_singleton() for alternative access method
     * 
     * Example usage:
     * @code
     * if (AP::arming().is_armed()) {
     *     // Vehicle is armed
     * }
     * @endcode
     */
    AP_Arming &arming();
};
