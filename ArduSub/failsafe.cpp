/**
 * @file failsafe.cpp
 * @brief Comprehensive failsafe monitoring and response system for underwater vehicles
 * 
 * @details This file implements the complete failsafe monitoring system for ArduSub,
 *          providing critical safety checks and automated responses to system failures
 *          that could endanger the underwater vehicle or mission.
 * 
 *          The ArduSub failsafe system monitors multiple critical subsystems:
 *          - Mainloop watchdog (CPU lockup detection)
 *          - Sensor health (depth sensor, EKF variance)
 *          - Power system (battery voltage and capacity)
 *          - Water intrusion (leak detection)
 *          - Internal enclosure conditions (pressure, temperature)
 *          - Communication links (pilot input, GCS heartbeat, RC link)
 *          - Vehicle attitude (crash/collision detection)
 *          - Terrain data availability (for terrain-following modes)
 * 
 *          Underwater-Specific Failsafe Adaptations:
 *          - No "Return to Launch" or automatic surface behavior by default
 *            (surfacing could cause decompression issues or surface hazards)
 *          - Leak detection triggers immediate action (SURFACE mode or disarm)
 *          - Internal pressure/temperature monitoring for watertight enclosure
 *          - Depth sensor is critical for any vertical control
 *          - Conservative failsafe actions prioritize stopping vehicle safely
 * 
 *          Failsafe Actions Available:
 *          - SURFACE mode: Controlled ascent to surface
 *          - ALT_HOLD mode: Maintain current depth
 *          - MANUAL mode: Pilot direct control
 *          - Disarm: Immediate motor shutdown
 *          - Warn only: Log and notify but continue operation
 * 
 *          All failsafe events are logged to dataflash and reported via MAVLink
 *          to ground control station for post-mission analysis.
 * 
 * @warning SAFETY-CRITICAL: This code protects against catastrophic failures.
 *          Modifications must be thoroughly tested in SITL and controlled water
 *          environments before deployment.
 * 
 * @note Failsafe parameter configuration (g.failsafe_*) determines behavior.
 *       Operators must configure appropriate actions for their mission profile.
 * 
 * Source: ArduSub/failsafe.cpp
 */

#include "Sub.h"

// Mainloop watchdog state variables
static bool failsafe_enabled = false;        ///< Mainloop failsafe monitoring enabled
static uint16_t failsafe_last_ticks;         ///< Last scheduler tick count seen
static uint32_t failsafe_last_timestamp;     ///< Last time mainloop was confirmed running (microseconds)
static bool in_failsafe;                     ///< Currently in mainloop failsafe state

/**
 * @brief Enable mainloop lockup failsafe monitoring
 * 
 * @details Activates the mainloop watchdog that monitors for CPU lockups or
 *          scheduler hangs. Once enabled, mainloop_failsafe_check() (called
 *          from 1kHz timer interrupt) will verify the main loop continues to
 *          execute. If the mainloop stops for more than 2 seconds, motors are
 *          automatically reduced to minimum output and then disarmed.
 * 
 *          This is typically called after vehicle initialization is complete
 *          and the main scheduler loop is running normally.
 * 
 * @note Called from main scheduler after successful initialization
 * 
 * @see mainloop_failsafe_disable()
 * @see mainloop_failsafe_check()
 */
void Sub::mainloop_failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = AP_HAL::micros();
}

/**
 * @brief Temporarily disable mainloop lockup failsafe monitoring
 * 
 * @details Disables the mainloop watchdog when deliberate delays are expected
 *          in the main loop (e.g., during compass calibration, parameter
 *          saving to flash, or other blocking operations). Must be re-enabled
 *          with mainloop_failsafe_enable() after the delay completes.
 * 
 * @warning Use sparingly - disabling watchdog removes critical safety protection.
 *          Always re-enable as soon as the blocking operation completes.
 * 
 * @note Typical use cases: Flash writes, calibration procedures, sensor initialization
 * 
 * @see mainloop_failsafe_enable()
 */
void Sub::mainloop_failsafe_disable()
{
    failsafe_enabled = false;
}

/**
 * @brief Monitor mainloop execution and trigger failsafe on CPU lockup
 * 
 * @details This function is called from the core timer interrupt at 1kHz to verify
 *          that the main scheduler loop continues to execute. It compares scheduler
 *          tick counts to detect if the mainloop has stopped or locked up.
 * 
 *          Detection mechanism:
 *          - Checks if scheduler.ticks() has incremented since last check
 *          - If ticks unchanged for 2+ seconds: Trigger failsafe (reduce motors to min)
 *          - If still locked after 1 more second: Disarm motors completely
 * 
 *          This protects against:
 *          - Infinite loops in flight code
 *          - Deadlocks from semaphore/mutex errors
 *          - Excessive CPU usage blocking scheduler
 *          - Hardware/peripheral hangs blocking I/O operations
 * 
 *          Failsafe response:
 *          1. After 2 sec lockup: Set motors to minimum output (allows logging)
 *          2. After 3 sec lockup: Disarm motors completely
 *          3. Log CPU failsafe event to dataflash
 * 
 * @warning SAFETY-CRITICAL: Called from 1kHz timer interrupt context.
 *          Must execute quickly and not block. Cannot use semaphores or
 *          blocking I/O operations.
 * 
 * @note Execution context: 1kHz timer interrupt (every 1ms)
 * @note Motors reduced to minimum before disarm to ensure failure is logged
 * 
 * @see mainloop_failsafe_enable()
 * @see mainloop_failsafe_disable()
 */
void Sub::mainloop_failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // the main loop is running, all is OK
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU,LogErrorCode::FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors.armed()) {
            motors.output_min();
        }
        LOGGER_WRITE_ERROR(LogErrorSubsystem::CPU,LogErrorCode::FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if (motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
}

/**
 * @brief Monitor critical sensor health and trigger failsafe on depth sensor failure
 * 
 * @details Monitors the depth sensor (barometer) which is absolutely critical for
 *          underwater vehicle operation. The depth sensor provides altitude
 *          information required for any vertical position control modes.
 * 
 *          Failure conditions:
 *          - Depth sensor reported as unhealthy by sensor_health.depth flag
 *          - Can occur from: I2C/SPI communication errors, sensor hardware failure,
 *            invalid pressure readings, sensor timeout
 * 
 *          Failsafe response:
 *          - If in ALT_HOLD, SURFACE, or GPS-dependent mode: Switch to MANUAL mode
 *          - MANUAL mode allows pilot to maintain vehicle control without altitude hold
 *          - If mode change fails: Emergency disarm (should never occur)
 *          - Send critical error message to GCS
 *          - Log sensor failsafe event
 * 
 * @warning SAFETY-CRITICAL: Depth sensor failure in underwater vehicle removes
 *          all vertical position control. Without depth sensor, vehicle cannot
 *          maintain depth, perform controlled ascent/descent, or use any
 *          altitude-dependent modes safely. Immediate pilot intervention required.
 * 
 * @warning UNDERWATER-SPECIFIC: Unlike aircraft, submarines cannot "land" on
 *          sensor failure. Loss of depth control is extremely dangerous as
 *          vehicle could surface uncontrollably or dive beyond safe limits.
 * 
 * @note Called from main loop sensor health monitoring
 * @note Failsafe triggers only once per failure event (state tracked in failsafe.sensor_health)
 * @note Recovery: Failsafe clears automatically when depth sensor becomes healthy again
 * 
 * @see sensor_health.depth
 * @see ap.depth_sensor_present
 */
void Sub::failsafe_sensors_check()
{
    if (!ap.depth_sensor_present) {
        return;
    }

    // We need a depth sensor to do any sort of auto z control
    if (sensor_health.depth) {
        if (failsafe.sensor_health) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_SENSORS, LogErrorCode::ERROR_RESOLVED);
            failsafe.sensor_health = false;
        }
        return;
    }

    // only report once
    if (failsafe.sensor_health) {
        return;
    }

    failsafe.sensor_health = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Depth sensor error!");
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_SENSORS, LogErrorCode::BAD_DEPTH);

    if (control_mode == Mode::Number::ALT_HOLD || control_mode == Mode::Number::SURFACE || sub.flightmode->requires_GPS()) {
        // This should always succeed
        if (!set_mode(Mode::Number::MANUAL, ModeReason::BAD_DEPTH)) {
            // We should never get here
            arming.disarm(AP_Arming::Method::BADFLOWOFCONTROL);
        }
    }
}

/**
 * @brief Monitor Extended Kalman Filter health and trigger failsafe on excessive variance
 * 
 * @details Monitors the EKF (Extended Kalman Filter) state estimation health by
 *          checking velocity and compass variance against configured thresholds.
 *          High variance indicates the EKF has lost confidence in its position
 *          and velocity estimates, which could lead to erratic vehicle behavior.
 * 
 *          Variance monitoring:
 *          - Velocity variance (vel_variance): Indicates uncertainty in velocity estimates
 *          - Compass variance (magVar.length()): Indicates uncertainty in heading
 *          - Thresholds configured by g.fs_ekf_thresh parameter
 *          - Requires 2 consecutive seconds of bad variance to trigger
 * 
 *          Common causes of high EKF variance:
 *          - Poor compass calibration or magnetic interference
 *          - GPS signal loss or poor GPS quality
 *          - Excessive vibration affecting IMU readings
 *          - Sensor disagreement (multiple IMU/compass conflict)
 *          - Vehicle flipping or tumbling (orientation uncertainty)
 * 
 *          Failsafe actions (configurable via g.fs_ekf_action):
 *          - FS_EKF_ACTION_DISABLED: No action, monitoring only
 *          - FS_EKF_ACTION_DISARM: Immediate motor disarm
 *          - Warning sent to GCS every 20 seconds while in failsafe
 * 
 * @warning High EKF variance indicates navigation system has lost reliability.
 *          Position and velocity estimates may be incorrect, affecting all
 *          autonomous modes (AUTO, GUIDED, POSHOLD).
 * 
 * @note Called from main loop at regular intervals (typically 10Hz)
 * @note Requires 2 solid seconds of bad variance to trigger failsafe (debouncing)
 * @note Visual/audio notification via AP_Notify::flags.ekf_bad
 * 
 * @see g.fs_ekf_action - Configured failsafe action
 * @see g.fs_ekf_thresh - Variance threshold parameter
 * @see ahrs.get_variances() - EKF variance retrieval
 */
void Sub::failsafe_ekf_check()
{
    static uint32_t last_ekf_good_ms = 0;

    if (g.fs_ekf_action == FS_EKF_ACTION_DISABLED) {
        last_ekf_good_ms = AP_HAL::millis();
        failsafe.ekf = false;
        AP_Notify::flags.ekf_bad = false;
        return;
    }

    float posVar, hgtVar, tasVar;
    Vector3f magVar;
    float compass_variance;
    float vel_variance;
    ahrs.get_variances(vel_variance, posVar, hgtVar, magVar, tasVar);
    compass_variance = magVar.length();

    if (compass_variance < g.fs_ekf_thresh && vel_variance < g.fs_ekf_thresh) {
        last_ekf_good_ms = AP_HAL::millis();
        failsafe.ekf = false;
        AP_Notify::flags.ekf_bad = false;
        return;
    }

    // Bad EKF for 2 solid seconds triggers failsafe
    if (AP_HAL::millis() < last_ekf_good_ms + 2000) {
        failsafe.ekf = false;
        AP_Notify::flags.ekf_bad = false;
        return;
    }

    // Only trigger failsafe once
    if (failsafe.ekf) {
        return;
    }

    failsafe.ekf = true;
    AP_Notify::flags.ekf_bad = true;

    LOGGER_WRITE_ERROR(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);

    if (AP_HAL::millis() > failsafe.last_ekf_warn_ms + 20000) {
        failsafe.last_ekf_warn_ms = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF bad");
    }

    if (g.fs_ekf_action == FS_EKF_ACTION_DISARM) {
        arming.disarm(AP_Arming::Method::EKFFAILSAFE);
    }
}

/**
 * @brief Handle battery failsafe event and execute configured action
 * 
 * @details Called by the battery monitor system when battery voltage or capacity
 *          crosses a failsafe threshold. Battery failsafe protects against power
 *          loss during operation by giving pilot warning and potentially forcing
 *          vehicle to surface or disarm before complete power failure.
 * 
 *          Trigger conditions (configured in battery monitor):
 *          - Low voltage threshold exceeded (BATT_LOW_VOLT)
 *          - Critical voltage threshold exceeded (BATT_CRT_VOLT)
 *          - Low capacity threshold exceeded (BATT_LOW_MAH)
 *          - Critical capacity threshold exceeded (BATT_CRT_MAH)
 * 
 *          Available actions:
 *          - Failsafe_Action_None: No action (monitoring only)
 *          - Failsafe_Action_Warn: Log and notify GCS but continue operation
 *          - Failsafe_Action_Surface: Switch to SURFACE mode for controlled ascent
 *          - Failsafe_Action_Disarm: Immediate motor disarm
 * 
 * @param[in] type_str Human-readable failsafe type description (e.g., "Low voltage", "Critical capacity")
 * @param[in] action Configured failsafe action from Failsafe_Action enum
 * 
 * @warning SAFETY-CRITICAL: Battery depletion underwater can lead to complete
 *          power loss, resulting in loss of control and potential vehicle loss.
 *          SURFACE action strongly recommended for underwater missions to ensure
 *          vehicle can surface before power exhaustion.
 * 
 * @warning UNDERWATER-SPECIFIC: Unlike aircraft that can land, ROVs losing power
 *          underwater will sink. Battery failsafe must trigger with sufficient
 *          margin to complete surface ascent.
 * 
 * @note Called from battery monitor check in main loop
 * @note Separate thresholds for "low" (warning) and "critical" (action) states
 * @note Action parameter typically comes from BATT_FS_LOW_ACT or BATT_FS_CRT_ACT
 * 
 * @see AP_BattMonitor for voltage/capacity monitoring
 * @see Failsafe_Action enum for available actions
 */
void Sub::handle_battery_failsafe(const char* type_str, const int8_t action)
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_BATT, LogErrorCode::FAILSAFE_OCCURRED);

    switch((Failsafe_Action)action) {
        case Failsafe_Action_Surface:
            set_mode(Mode::Number::SURFACE, ModeReason::BATTERY_FAILSAFE);
            break;
        case Failsafe_Action_Disarm:
            arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
            break;
        case Failsafe_Action_Warn:
        case Failsafe_Action_None:
            break;
    }
}

/**
 * @brief Monitor pilot input (joystick/gamepad) timeout and trigger failsafe
 * 
 * @details Ensures continuous pilot control input is being received from the
 *          joystick or gamepad. If pilot input stops updating for the configured
 *          timeout period, this indicates loss of communication with the pilot
 *          control station (topside computer or surface control box).
 * 
 *          Timeout detection:
 *          - Monitors time since last update to failsafe.last_pilot_input_ms
 *          - Timeout threshold configured by g.failsafe_pilot_input_timeout (seconds)
 *          - Typical timeout: 2-5 seconds
 * 
 *          Common causes of pilot input loss:
 *          - USB connection failure to joystick
 *          - Topside computer crash or freeze
 *          - Tether break (for tethered ROVs)
 *          - Control software crash
 * 
 *          Failsafe actions (g.failsafe_pilot_input):
 *          - FS_PILOT_INPUT_DISABLED: No failsafe action
 *          - FS_PILOT_INPUT_DISARM: Immediate motor disarm
 * 
 *          Response sequence:
 *          1. Set all control inputs to neutral (zero thrust/rotation)
 *          2. Log failsafe event
 *          3. Send critical message to GCS
 *          4. Execute configured action (disarm if enabled)
 * 
 * @warning Loss of pilot input in MANUAL or ACRO modes leaves vehicle without
 *          control. Setting controls to neutral stops vehicle motion but does
 *          not maintain position - vehicle will drift with currents.
 * 
 * @note Called from main loop at regular intervals
 * @note Failsafe triggers only once per event (state in failsafe.pilot_input)
 * @note Recovery: Automatic when pilot input resumes
 * 
 * @see failsafe.last_pilot_input_ms - Updated on joystick input
 * @see g.failsafe_pilot_input - Failsafe enable/disable
 * @see g.failsafe_pilot_input_timeout - Timeout in seconds
 * @see set_neutral_controls() - Zeros all pilot control inputs
 */
void Sub::failsafe_pilot_input_check()
{
    if (g.failsafe_pilot_input == FS_PILOT_INPUT_DISABLED) {
        failsafe.pilot_input = false;
        return;
    }

    if (AP_HAL::millis() < failsafe.last_pilot_input_ms + g.failsafe_pilot_input_timeout * 1000.0f) {
        failsafe.pilot_input = false; // We've received an update from the pilot within the timeout period
        return;
    }

    if (failsafe.pilot_input) {
        return; // only act once
    }

    failsafe.pilot_input = true;

    LOGGER_WRITE_ERROR(LogErrorSubsystem::PILOT_INPUT, LogErrorCode::FAILSAFE_OCCURRED);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Lost manual control");

    set_neutral_controls();

    if(g.failsafe_pilot_input == FS_PILOT_INPUT_DISARM) {
        arming.disarm(AP_Arming::Method::PILOT_INPUT_FAILSAFE);
    }
}

/**
 * @brief Monitor internal enclosure pressure and warn of seal breach or outgassing
 * 
 * @details Monitors the barometric pressure inside the watertight electronics
 *          enclosure to detect potential seal breaches or dangerous outgassing
 *          from batteries/electronics. Internal pressure rising above the
 *          threshold indicates either:
 *          - Water leaking into enclosure (slow leak, pressure equalizing)
 *          - Battery outgassing from thermal runaway or overcharge
 *          - Inadequate venting causing pressure buildup
 * 
 *          Pressure monitoring:
 *          - Uses barometer.get_pressure(0) to read internal barometer
 *          - Threshold configured by g.failsafe_pressure_max parameter
 *          - Requires 2 consecutive seconds above threshold to trigger
 *          - Warnings sent every 30 seconds while in failsafe state
 * 
 *          Typical internal pressure values:
 *          - Normal: Close to ambient pressure at surface (sea level ~101.3 kPa)
 *          - Surface dive prep: May be sealed at surface pressure
 *          - Excessive: >5 kPa above expected indicates problem
 * 
 * @warning SAFETY-CRITICAL: Rising internal pressure indicates potential
 *          catastrophic failure in progress. Operator must surface immediately
 *          and investigate. Continued operation risks total electronics failure,
 *          fire, or enclosure breach.
 * 
 * @warning UNDERWATER-SPECIFIC: Internal pressure monitoring unique to underwater
 *          vehicles. High internal pressure can indicate impending seal failure
 *          that would cause rapid flooding and vehicle loss. This is a critical
 *          early warning system.
 * 
 * @note Called from main loop at regular intervals
 * @note Currently warning-only (no automatic action taken)
 * @note Operator must configure appropriate FS_PRESS_MAX for their vehicle
 * @note Does not monitor for LOW pressure (would indicate enclosure breach already occurred)
 * 
 * @see g.failsafe_pressure - Enable/disable pressure monitoring
 * @see g.failsafe_pressure_max - Maximum safe internal pressure threshold
 * @see barometer.get_pressure() - Internal barometer reading
 */
void Sub::failsafe_internal_pressure_check()
{

    if (g.failsafe_pressure == FS_PRESS_DISABLED) {
        return; // Nothing to do
    }

    uint32_t tnow = AP_HAL::millis();
    static uint32_t last_pressure_warn_ms;
    static uint32_t last_pressure_good_ms;
    if (barometer.get_pressure(0) < g.failsafe_pressure_max) {
        last_pressure_good_ms = tnow;
        last_pressure_warn_ms = tnow;
        failsafe.internal_pressure = false;
        return;
    }

    // 2 seconds with no readings below threshold triggers failsafe
    if (tnow > last_pressure_good_ms + 2000) {
        failsafe.internal_pressure = true;
    }

    // Warn every 30 seconds
    if (failsafe.internal_pressure && tnow > last_pressure_warn_ms + 30000) {
        last_pressure_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_WARNING, "Internal pressure critical!");
    }
}

/**
 * @brief Monitor internal enclosure temperature and warn of overheating
 * 
 * @details Monitors the temperature inside the watertight electronics enclosure
 *          to detect overheating conditions that could lead to component failure,
 *          reduced battery life, or fire hazard. High internal temperature
 *          indicates inadequate cooling or excessive power dissipation.
 * 
 *          Temperature monitoring:
 *          - Uses barometer.get_temperature(0) to read internal temperature sensor
 *          - Threshold configured by g.failsafe_temperature_max parameter
 *          - Requires 2 consecutive seconds above threshold to trigger
 *          - Warnings sent every 30 seconds while in failsafe state
 * 
 *          Common causes of high internal temperature:
 *          - Inadequate thermal design or cooling
 *          - High ambient water temperature
 *          - Excessive power draw (high current electronics)
 *          - Battery thermal runaway (extremely dangerous)
 *          - Prolonged operation in warm water
 *          - Sealed enclosure with poor heat transfer
 * 
 *          Typical temperature ranges:
 *          - Normal operation: 20-40°C
 *          - Warning threshold: 45-55°C
 *          - Critical: >60°C (component damage possible)
 *          - Emergency: >70°C (fire/thermal runaway risk)
 * 
 * @warning SAFETY-CRITICAL: Excessive internal temperature can cause:
 *          - Electronic component failure or malfunction
 *          - Battery thermal runaway leading to fire/explosion
 *          - Reduced battery capacity and life
 *          - Seal degradation from thermal cycling
 *          Operator must surface and cool vehicle or risk catastrophic failure.
 * 
 * @warning UNDERWATER-SPECIFIC: Unlike air vehicles with good cooling, underwater
 *          vehicles in warm water or with poor thermal design can easily overheat
 *          in sealed enclosures. Water temperature may not be sufficient to cool
 *          high-power electronics.
 * 
 * @note Called from main loop at regular intervals
 * @note Currently warning-only (no automatic action taken)
 * @note Operator must configure appropriate FS_TEMP_MAX for their vehicle/mission
 * @note Temperature readings from barometer may lag actual hotspot temperatures
 * 
 * @see g.failsafe_temperature - Enable/disable temperature monitoring
 * @see g.failsafe_temperature_max - Maximum safe internal temperature threshold
 * @see barometer.get_temperature() - Internal temperature sensor reading
 */
void Sub::failsafe_internal_temperature_check()
{

    if (g.failsafe_temperature == FS_TEMP_DISABLED) {
        return; // Nothing to do
    }

    uint32_t tnow = AP_HAL::millis();
    static uint32_t last_temperature_warn_ms;
    static uint32_t last_temperature_good_ms;
    if (barometer.get_temperature(0) < g.failsafe_temperature_max) {
        last_temperature_good_ms = tnow;
        last_temperature_warn_ms = tnow;
        failsafe.internal_temperature = false;
        return;
    }

    // 2 seconds with no readings below threshold triggers failsafe
    if (tnow > last_temperature_good_ms + 2000) {
        failsafe.internal_temperature = true;
    }

    // Warn every 30 seconds
    if (failsafe.internal_temperature && tnow > last_temperature_warn_ms + 30000) {
        last_temperature_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_WARNING, "Internal temperature critical!");
    }
}

/**
 * @brief Monitor for water intrusion and trigger emergency surface
 * 
 * @details Monitors leak detector sensors for water intrusion into the electronics
 *          enclosure. Water leak is one of the most critical failures for underwater
 *          vehicles as it leads rapidly to complete electronics failure and vehicle
 *          loss. This failsafe provides early warning and automatic emergency ascent.
 * 
 *          Leak detection:
 *          - Monitors leak_detector.get_status() from leak sensor(s)
 *          - Leak detectors typically use conductivity probes inside enclosure
 *          - Even small amounts of water trigger detection
 *          - Multiple sensors can be configured for redundancy
 * 
 *          Failsafe actions (g.failsafe_leak):
 *          - FS_LEAK_DISABLED: Monitoring only, no automatic action
 *          - FS_LEAK_SURFACE: Immediately switch to SURFACE mode (recommended)
 * 
 *          Response sequence on leak detection:
 *          1. Set leak detected flag (visual/audio notification)
 *          2. Log leak failsafe event
 *          3. Send critical warning to GCS every 20 seconds
 *          4. If armed and action=SURFACE: Emergency ascent to surface
 * 
 *          Time criticality:
 *          - Water intrusion can cause total failure in seconds to minutes
 *          - Speed of failure depends on leak size and water conductivity
 *          - Salt water causes faster failure than fresh water
 *          - Immediate surfacing is critical to save vehicle
 * 
 * @warning SAFETY-CRITICAL: Water leak is a catastrophic failure for underwater
 *          vehicles. Once water enters electronics enclosure, vehicle may have
 *          only seconds before complete power/control loss. SURFACE action is
 *          STRONGLY RECOMMENDED to maximize chance of vehicle recovery.
 * 
 * @warning UNDERWATER-SPECIFIC: Leak detection is unique to underwater vehicles
 *          and represents the most critical single-point failure. Unlike aircraft
 *          failures, there is no "safe landing" option - vehicle must surface
 *          immediately or will be lost.
 * 
 * @warning Even with SURFACE mode engaged, vehicle may lose power/control before
 *          reaching surface if leak is severe. Positive buoyancy is critical
 *          backup to ensure vehicle surfaces even after power loss.
 * 
 * @note Called from main loop at regular intervals (typically 10Hz)
 * @note Continuous warnings sent every 20 seconds while leak detected
 * @note Visual notification via AP_Notify::flags.leak_detected
 * @note Recovery: Failsafe clears if leak detector status goes false (sensor dried)
 * 
 * @see leak_detector.get_status() - Leak sensor state
 * @see g.failsafe_leak - Configured failsafe action
 * @see Mode::Number::SURFACE - Emergency ascent mode
 */
void Sub::failsafe_leak_check()
{
    bool status = leak_detector.get_status();

    // Do nothing if we are dry, or if leak failsafe action is disabled
    if (status == false || g.failsafe_leak == FS_LEAK_DISABLED) {
        if (failsafe.leak) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_LEAK, LogErrorCode::FAILSAFE_RESOLVED);
        }
        AP_Notify::flags.leak_detected = false;
        failsafe.leak = false;
        return;
    }

    AP_Notify::flags.leak_detected = status;

    uint32_t tnow = AP_HAL::millis();

    // We have a leak
    // Always send a warning every 20 seconds
    if (tnow > failsafe.last_leak_warn_ms + 20000) {
        failsafe.last_leak_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Leak Detected");
    }

    // Do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.leak) {
        return;
    }

    failsafe.leak = true;

    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_LEAK, LogErrorCode::FAILSAFE_OCCURRED);

    // Handle failsafe action
    if (failsafe.leak && g.failsafe_leak == FS_LEAK_SURFACE && motors.armed()) {
        set_mode(Mode::Number::SURFACE, ModeReason::LEAK_FAILSAFE);
    }
}

/**
 * @brief Monitor Ground Control Station heartbeat and trigger failsafe on timeout
 * 
 * @details Monitors MAVLink heartbeat messages from the configured Ground Control
 *          Station to detect loss of communication with topside control system.
 *          GCS heartbeat timeout indicates the communication link (tether, wireless,
 *          or acoustic) has failed or the topside computer has stopped operating.
 * 
 *          Heartbeat monitoring:
 *          - Tracks time since last heartbeat from GCS with matching system ID (MAV_GCS_SYSID)
 *          - Timeout threshold configured by g.failsafe_gcs_timeout (seconds)
 *          - Only monitors if previous GCS contact has been established
 *          - Warnings sent every 30 seconds while in failsafe state
 * 
 *          Common causes of GCS heartbeat loss:
 *          - Tether break or disconnection (tethered ROVs)
 *          - Wireless link failure (if using RF/WiFi)
 *          - Acoustic modem communication loss
 *          - Topside computer crash or power loss
 *          - MAVLink communication software failure
 *          - Excessive RF interference or signal attenuation
 * 
 *          Failsafe actions (g.failsafe_gcs):
 *          - FS_GCS_DISABLED: No failsafe action
 *          - FS_GCS_HOLD: Switch to ALT_HOLD mode (maintain depth)
 *          - FS_GCS_SURFACE: Switch to SURFACE mode (emergency ascent)
 *          - FS_GCS_DISARM: Immediate motor disarm
 * 
 *          Action fallback logic:
 *          - If HOLD action fails to set ALT_HOLD: Disarm motors
 *          - If SURFACE action fails to set SURFACE mode: Disarm motors
 * 
 * @warning Loss of GCS communication means no mission updates, no telemetry
 *          monitoring, and no operator awareness of vehicle state. Conservative
 *          failsafe action recommended (SURFACE or HOLD) to ensure vehicle
 *          can be recovered.
 * 
 * @warning UNDERWATER-SPECIFIC: Unlike aircraft GCS failsafe (often RTL), underwater
 *          vehicles cannot automatically navigate to operator. SURFACE action brings
 *          vehicle to surface where it may be visible/recoverable. HOLD action
 *          maintains position if vehicle will regain communication.
 * 
 * @note Called from main loop at regular intervals
 * @note Failsafe only triggers if GCS was previously connected (prevents nuisance activation)
 * @note Recovery: Automatic when GCS heartbeat resumes within timeout period
 * @note Different from pilot input failsafe (joystick) - this is topside computer heartbeat
 * 
 * @see gcs().sysid_mygcs_last_seen_time_ms() - Last GCS heartbeat time
 * @see g.failsafe_gcs - Configured failsafe action
 * @see g.failsafe_gcs_timeout - Timeout in seconds
 * @see MAV_GCS_SYSID - Ground station system ID to monitor
 */
void Sub::failsafe_gcs_check()
{
    // return immediately if we have never had contact with a gcs, or if gcs failsafe action is disabled
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if (!g.failsafe_gcs && g.failsafe_gcs == FS_GCS_DISABLED) {
        return;
    }

    const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) {
        // we've never seen a GCS, so we don't failsafe if we stop seeing it
        return;
    }

    uint32_t tnow = AP_HAL::millis();

    // Check if we have gotten a GCS heartbeat recently (GCS sysid must match MAV_GCS_SYSID parameter)
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(g.failsafe_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));
    if (tnow - gcs_last_seen_ms < gcs_timeout_ms) {
        // Log event if we are recovering from previous gcs failsafe
        if (failsafe.gcs) {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_RESOLVED);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"GCS Failsafe Cleared");
        }
        failsafe.gcs = false;
        return;
    }

    //////////////////////////////
    // GCS heartbeat has timed out
    //////////////////////////////

    // Send a warning every 30 seconds
    if (tnow - failsafe.last_gcs_warn_ms > 30000) {
        failsafe.last_gcs_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_WARNING, "MYGCS: %u, heartbeat lost", unsigned(gcs().sysid_gcs()));
    }

    // do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.gcs || !motors.armed()) {
        return;
    }

    failsafe.gcs = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_GCS, LogErrorCode::FAILSAFE_OCCURRED);

    // handle failsafe action
    if (g.failsafe_gcs == FS_GCS_DISARM) {
        arming.disarm(AP_Arming::Method::GCSFAILSAFE);
    } else if (g.failsafe_gcs == FS_GCS_HOLD && motors.armed()) {
        if (!set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_FAILSAFE)) {
            arming.disarm(AP_Arming::Method::GCS_FAILSAFE_HOLDFAILED);
        }
    } else if (g.failsafe_gcs == FS_GCS_SURFACE && motors.armed()) {
        if (!set_mode(Mode::Number::SURFACE, ModeReason::GCS_FAILSAFE)) {
            arming.disarm(AP_Arming::Method::GCS_FAILSAFE_SURFACEFAILED);
        }
    }
}

/// Crash detection timing threshold - persistent attitude error for 2 seconds indicates crash
#define CRASH_CHECK_TRIGGER_MS          2000
/// Crash detection angle threshold - 30 degrees beyond commanded attitude indicates collision/inversion
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f

/**
 * @brief Detect vehicle collision or inversion by monitoring attitude error
 * 
 * @details Monitors attitude control error to detect crashes, collisions with
 *          obstacles, or vehicle inversion. Large persistent attitude errors
 *          indicate the vehicle cannot achieve commanded attitude, suggesting
 *          physical interference (collision, entanglement) or catastrophic
 *          control failure.
 * 
 *          Detection algorithm:
 *          - Compares actual attitude to commanded attitude from attitude controller
 *          - Triggers if attitude error exceeds 30 degrees for 2+ consecutive seconds
 *          - Only active in angle-stabilized modes (not ACRO or MANUAL)
 *          - Only monitors when motors are armed
 * 
 *          Common crash/collision scenarios:
 *          - Impact with underwater structure (rock, coral, wreck)
 *          - Entanglement in kelp, fishing line, or nets
 *          - Vehicle flipped upside-down (inversion)
 *          - Motor failure preventing attitude control
 *          - Stuck against surface or bottom
 *          - Frame damage affecting control authority
 * 
 *          Failsafe actions (g.fs_crash_check):
 *          - FS_CRASH_DISABLED: No crash detection
 *          - FS_CRASH_DISARM: Immediate motor disarm on detected crash
 * 
 *          Mode-specific behavior:
 *          - Disabled in ACRO/MANUAL modes (pilot expects large attitude deviations)
 *          - Active in STABILIZE, ALT_HOLD, AUTO, GUIDED, etc.
 * 
 * @warning Large attitude errors can also occur from:
 *          - Aggressive pilot input in STABILIZE mode
 *          - Very poor PID tuning
 *          - Extreme external forces (strong currents)
 *          Tune crash detection threshold for your environment to avoid false triggers.
 * 
 * @warning UNDERWATER-SPECIFIC: Underwater collisions often involve entanglement
 *          which prevents attitude control but doesn't cause structural damage.
 *          Disarming may prevent self-extraction. Consider carefully whether
 *          DISARM action is appropriate for your mission environment.
 * 
 * @note Called from main loop at regular intervals
 * @note Requires 2 full seconds of excessive attitude error to trigger (debouncing)
 * @note Warnings sent every 20 seconds while crash state persists
 * @note Recovery: Automatic if attitude error returns below threshold
 * 
 * @see attitude_control.get_att_error_angle_deg() - Current attitude error magnitude
 * @see g.fs_crash_check - Enable/disable and action configuration
 * @see CRASH_CHECK_ANGLE_DEVIATION_DEG - Angle error threshold (30 degrees)
 * @see CRASH_CHECK_TRIGGER_MS - Time threshold (2000 milliseconds)
 */
void Sub::failsafe_crash_check()
{
    static uint32_t last_crash_check_pass_ms;
    uint32_t tnow = AP_HAL::millis();

    // return immediately if disarmed, or crash checking disabled
    if (!motors.armed() || g.fs_crash_check == FS_CRASH_DISABLED) {
        last_crash_check_pass_ms = tnow;
        failsafe.crash = false;
        return;
    }

    // return immediately if we are not in an angle stabilized flight mode
    if (control_mode == Mode::Number::ACRO || control_mode == Mode::Number::MANUAL) {
        last_crash_check_pass_ms = tnow;
        failsafe.crash = false;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control.get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        last_crash_check_pass_ms = tnow;
        failsafe.crash = false;
        return;
    }

    if (tnow < last_crash_check_pass_ms + CRASH_CHECK_TRIGGER_MS) {
        return;
    }

    // Conditions met, we are in failsafe

    // Send warning to GCS
    if (tnow > failsafe.last_crash_warn_ms + 20000) {
        failsafe.last_crash_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_WARNING,"Crash detected");
    }

    // Only perform failsafe action once
    if (failsafe.crash) {
        return;
    }

    failsafe.crash = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);

    // disarm motors
    if (g.fs_crash_check == FS_CRASH_DISARM) {
        arming.disarm(AP_Arming::Method::CRASH);
    }
}

/**
 * @brief Monitor terrain data availability and trigger failsafe on persistent loss
 * 
 * @details Executes terrain failsafe if terrain altitude data is missing for longer
 *          than FS_TERRAIN_TIMEOUT_MS (typically 5 seconds) while in AUTO or GUIDED
 *          mode. Terrain data is critical when mission waypoints use ALT_ABOVE_TERRAIN
 *          frame or when terrain-following is enabled.
 * 
 *          Terrain data sources:
 *          - Pre-loaded terrain database (bathymetric maps)
 *          - Downward-facing rangefinder providing real-time sea floor distance
 *          - Terrain altitude in mission waypoints
 * 
 *          Trigger conditions:
 *          - Must be in AUTO or GUIDED mode (terrain-following modes)
 *          - Terrain data unavailable for consecutive FS_TERRAIN_TIMEOUT_MS duration
 *          - Failure duration tracked by failsafe_terrain_set_status()
 * 
 *          Failsafe response sequence:
 *          1. Send critical warning to GCS
 *          2. Call failsafe_terrain_on_event() to initiate recovery
 *          3. Attempt auto-recovery if rangefinder available (mode_auto.auto_terrain_recover_start())
 *          4. If recovery fails: Execute configured failsafe action (failsafe_terrain_act())
 * 
 *          Recovery:
 *          - Failsafe automatically clears when terrain data becomes available
 *          - Requires 0.1 seconds of valid data to clear (debouncing)
 * 
 * @note Called from main loop during AUTO and GUIDED modes
 * @note Failure duration tracked via failsafe.terrain_first_failure_ms and
 *       failsafe.terrain_last_failure_ms timestamps
 * @note FS_TERRAIN_TIMEOUT_MS typically set to 5000 milliseconds
 * 
 * @warning Operating in terrain-following modes without terrain data can cause
 *          collision with sea floor or unintended surfacing. Immediate action required.
 * 
 * @see failsafe_terrain_set_status() - Called by navigation code to report terrain data status
 * @see failsafe_terrain_on_event() - Initiates failsafe actions
 * @see failsafe_terrain_act() - Executes configured failsafe response
 * @see FS_TERRAIN_TIMEOUT_MS - Timeout threshold (5 seconds)
 */
void Sub::failsafe_terrain_check()
{
    // trigger with 5 seconds of failures while in AUTO mode
    bool valid_mode = (control_mode == Mode::Number::AUTO || control_mode == Mode::Number::GUIDED);
    bool timeout = (failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS;
    bool trigger_event = valid_mode && timeout;

    // check for clearing of event
    if (trigger_event != failsafe.terrain) {
        if (trigger_event) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,"Failsafe terrain triggered");
            failsafe_terrain_on_event();
        } else {
            LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::ERROR_RESOLVED);
            failsafe.terrain = false;
        }
    }
}

/**
 * @brief Report terrain data availability status from navigation code
 * 
 * @details Called by mission/navigation code when using ALT_ABOVE_TERRAIN frame
 *          waypoints or terrain-following modes. Tracks the duration of terrain
 *          data failures to determine when to trigger terrain failsafe.
 * 
 *          Terrain data is considered unavailable when:
 *          - Terrain database lookup fails (no data for current location)
 *          - Rangefinder is disabled or unhealthy (cannot measure to sea floor)
 *          - Terrain data is stale or outdated
 * 
 *          Failure duration tracking:
 *          - Records timestamp of first terrain data failure
 *          - Updates timestamp of most recent failure
 *          - failsafe_terrain_check() compares duration against timeout threshold
 * 
 *          Recovery logic:
 *          - Requires 0.1 seconds (100ms) of consecutive successful terrain data
 *          - Clears failure timestamps when recovery confirmed
 *          - Prevents rapid failsafe oscillation (debouncing)
 * 
 * @param[in] data_ok true if terrain data is valid and available,
 *                    false if terrain data lookup failed or is unavailable
 * 
 * @note Called from mission execution code and guided mode navigation
 * @note Only relevant when using ALT_ABOVE_TERRAIN altitude frame
 * @note 100ms debounce period prevents false recovery on intermittent failures
 * 
 * @see failsafe_terrain_check() - Monitors failure duration and triggers failsafe
 * @see failsafe.terrain_first_failure_ms - Timestamp of first terrain data failure
 * @see failsafe.terrain_last_failure_ms - Timestamp of most recent failure
 */
void Sub::failsafe_terrain_set_status(bool data_ok)
{
    uint32_t now = AP_HAL::millis();

    // record time of first and latest failures (i.e. duration of failures)
    if (!data_ok) {
        failsafe.terrain_last_failure_ms = now;
        if (failsafe.terrain_first_failure_ms == 0) {
            failsafe.terrain_first_failure_ms = now;
        }
    } else {
        // failures cleared after 0.1 seconds of persistent successes
        if (now - failsafe.terrain_last_failure_ms > 100) {
            failsafe.terrain_last_failure_ms = 0;
            failsafe.terrain_first_failure_ms = 0;
        }
    }
}

/**
 * @brief Initiate terrain failsafe response and attempt automatic recovery
 * 
 * @details Called when terrain failsafe is triggered after persistent terrain
 *          data loss. Attempts automatic recovery if rangefinder is available,
 *          otherwise executes configured failsafe action.
 * 
 *          Recovery strategy:
 *          - If rangefinder enabled: Attempt auto_terrain_recover_start()
 *          - Auto recovery: Vehicle continues mission using rangefinder altitude
 *            instead of terrain database until terrain data becomes available again
 *          - If rangefinder disabled or recovery fails: Execute failsafe action
 * 
 *          Recovery success conditions:
 *          - Rangefinder must be enabled (rangefinder_state.enabled == true)
 *          - Rangefinder must be healthy and providing valid measurements
 *          - Vehicle must be in mode that supports rangefinder altitude (AUTO)
 * 
 *          If recovery fails:
 *          - Calls failsafe_terrain_act() to execute configured failsafe response
 *          - Options: HOLD, SURFACE, or DISARM based on g.failsafe_terrain setting
 * 
 * @note Sets failsafe.terrain = true to indicate active terrain failsafe state
 * @note Logs terrain failsafe event to dataflash
 * @note Recovery attempt only made once per failsafe event
 * 
 * @warning If rangefinder not available, vehicle cannot continue terrain-following
 *          safely and must take protective action (hold position, surface, or disarm).
 * 
 * @see sub.mode_auto.auto_terrain_recover_start() - Automatic recovery using rangefinder
 * @see failsafe_terrain_act() - Execute configured failsafe action
 * @see rangefinder_state.enabled - Rangefinder availability flag
 * @see g.failsafe_terrain - Configured failsafe action
 */
void Sub::failsafe_terrain_on_event()
{
    failsafe.terrain = true;
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_TERRAIN, LogErrorCode::FAILSAFE_OCCURRED);

    // If rangefinder is enabled, we can recover from this failsafe
    if (!rangefinder_state.enabled || !sub.mode_auto.auto_terrain_recover_start()) {
        failsafe_terrain_act();
    }


}

/**
 * @brief Execute configured terrain failsafe action after recovery failed
 * 
 * @details Called when automatic terrain recovery is not possible (rangefinder
 *          unavailable or recovery attempt failed). Executes the operator-configured
 *          failsafe response based on g.failsafe_terrain parameter.
 * 
 *          Available failsafe actions:
 * 
 *          FS_TERRAIN_HOLD:
 *          - Attempt to switch to POSHOLD mode (maintain horizontal and vertical position)
 *          - If POSHOLD unavailable: Fall back to ALT_HOLD (maintain depth only)
 *          - Stops horizontal movement, maintains current depth
 *          - Allows pilot to take control and assess situation
 * 
 *          FS_TERRAIN_SURFACE:
 *          - Switch to SURFACE mode (controlled ascent to surface)
 *          - Vehicle ascends at configured rate until surface detected
 *          - Appropriate when terrain collision risk is high
 * 
 *          FS_TERRAIN_DISARM:
 *          - Immediate motor shutdown
 *          - Most conservative action - stops all vehicle movement
 *          - Vehicle will drift with current after disarm
 *          - Use when immediate stop is critical for safety
 * 
 *          Mode change notification:
 *          - Triggers AP_Notify::events.failsafe_mode_change for LED/buzzer alerts
 *          - GCS receives mode change notification via MAVLink
 * 
 * @note Called from failsafe_terrain_on_event() when recovery not possible
 * @note Action determined by g.failsafe_terrain parameter configuration
 * @note Default action is FS_TERRAIN_DISARM (most conservative)
 * 
 * @warning POSHOLD requires GPS, if GPS unavailable falls back to ALT_HOLD
 * @warning DISARM action leaves vehicle drifting - only use if collision imminent
 * @warning SURFACE action may not be appropriate in all underwater environments
 *          (e.g., under ice, in cave, under pier/structure)
 * 
 * @see g.failsafe_terrain - Configured failsafe action
 * @see FS_TERRAIN_HOLD - Hold position failsafe option
 * @see FS_TERRAIN_SURFACE - Surface failsafe option  
 * @see FS_TERRAIN_DISARM - Disarm failsafe option
 * @see AP_Notify::events.failsafe_mode_change - Visual/audio notification trigger
 */
void Sub::failsafe_terrain_act()
{
    switch (g.failsafe_terrain) {
    case FS_TERRAIN_HOLD:
        if (!set_mode(Mode::Number::POSHOLD, ModeReason::TERRAIN_FAILSAFE)) {
            set_mode(Mode::Number::ALT_HOLD, ModeReason::TERRAIN_FAILSAFE);
        }
        AP_Notify::events.failsafe_mode_change = 1;
        break;

    case FS_TERRAIN_SURFACE:
        set_mode(Mode::Number::SURFACE, ModeReason::TERRAIN_FAILSAFE);
        AP_Notify::events.failsafe_mode_change = 1;
        break;

    case FS_TERRAIN_DISARM:
    default:
        arming.disarm(AP_Arming::Method::TERRAINFAILSAFE);
    }
}

#if AP_SUB_RC_ENABLED
/**
 * @brief Update radio (RC) failsafe state and trigger appropriate events
 * 
 * @details Called by RC input processing code when radio signal is lost or regained.
 *          Detects state changes and triggers corresponding failsafe events.
 *          Radio failsafe indicates loss of communication between pilot's RC transmitter
 *          and vehicle's RC receiver.
 * 
 *          State transition handling:
 *          - Radio lost (b == true): Calls failsafe_radio_on_event()
 *          - Radio regained (b == false): Calls failsafe_radio_off_event()
 *          - No action if state unchanged (prevents duplicate event triggers)
 * 
 *          Radio signal loss detection:
 *          - No valid RC PWM signals received
 *          - RC receiver reports failsafe condition
 *          - RC signal below quality threshold
 *          - Timeout since last valid RC packet
 * 
 *          Notification:
 *          - Updates AP_Notify::flags.failsafe_radio for LED/buzzer indication
 *          - Visual feedback to pilot/operator of RC link status
 * 
 * @param[in] b true if radio signal lost (failsafe active),
 *              false if radio signal present (failsafe cleared)
 * 
 * @note Only compiled when AP_SUB_RC_ENABLED is defined
 * @note Called from RC input processing in main loop
 * @note State tracked in failsafe.radio flag
 * @note Debouncing performed by RC library before calling this function
 * 
 * @warning SAFETY-CRITICAL: Loss of RC control means pilot cannot command vehicle.
 *          Vehicle behavior during RC failsafe determined by g.failsafe_throttle setting.
 * 
 * @see failsafe_radio_on_event() - Actions when RC signal lost
 * @see failsafe_radio_off_event() - Actions when RC signal regained
 * @see AP_Notify::flags.failsafe_radio - Visual notification flag
 * @see g.failsafe_throttle - Configured RC failsafe action
 */
void Sub::set_failsafe_radio(bool b)
{
  // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();

        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}

/**
 * @brief Execute RC failsafe action when radio contact is lost
 * 
 * @details Called when RC radio signal from pilot's transmitter is lost. Executes
 *          operator-configured failsafe response based on g.failsafe_throttle parameter.
 *          Loss of RC control means pilot cannot command the vehicle, requiring
 *          automatic protective action.
 * 
 *          Available failsafe actions:
 * 
 *          FS_THR_DISABLED:
 *          - No automatic action taken
 *          - Vehicle continues in current mode with last commanded inputs
 *          - Only appropriate when alternate control available (joystick, GCS)
 *          - Warning logged and sent to GCS
 * 
 *          FS_THR_WARN:
 *          - Set all control inputs to neutral (zero thrust)
 *          - Vehicle stops but maintains current mode
 *          - Allows GCS or alternate control to take over
 *          - Does not change flight mode
 * 
 *          FS_THR_SURFACE:
 *          - Switch to SURFACE mode (controlled ascent)
 *          - Vehicle ascends to surface for recovery
 *          - Appropriate for most operational scenarios
 *          - Ensures vehicle can be recovered if RC link not restored
 * 
 *          Event logging:
 *          - Logs RC failsafe occurrence to dataflash
 *          - Sends warning message to GCS via MAVLink
 * 
 * @note Only compiled when AP_SUB_RC_ENABLED is defined
 * @note Called from set_failsafe_radio() when RC signal loss detected
 * @note Action determined by g.failsafe_throttle parameter configuration
 * 
 * @warning SAFETY-CRITICAL: Without RC control, pilot cannot command vehicle.
 *          Failsafe action must ensure vehicle safety and recoverability.
 * 
 * @warning UNDERWATER-SPECIFIC: Unlike aircraft which can return to launch,
 *          underwater vehicle should typically surface (FS_THR_SURFACE) to enable
 *          recovery. Continuing mission without pilot control (FS_THR_DISABLED)
 *          is only appropriate with robust alternate control (joystick, GCS).
 * 
 * @see set_failsafe_radio() - Detects RC signal loss
 * @see failsafe_radio_off_event() - Recovery when RC signal restored
 * @see g.failsafe_throttle - Configured RC failsafe action
 * @see set_neutral_controls() - Zero all control inputs
 * @see FS_THR_DISABLED - Disabled failsafe option
 * @see FS_THR_WARN - Neutral controls failsafe option
 * @see FS_THR_SURFACE - Surface mode failsafe option
 */
void Sub::failsafe_radio_on_event()
{
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_OCCURRED);
    gcs().send_text(MAV_SEVERITY_WARNING, "RC Failsafe");
        switch(g.failsafe_throttle) {
        case FS_THR_SURFACE:
            set_mode(Mode::Number::SURFACE, ModeReason::RADIO_FAILSAFE);
            break;
        case FS_THR_WARN:
            set_neutral_controls();
            break;
        case FS_THR_DISABLED:
            break;
    }    
}

/**
 * @brief Handle RC failsafe recovery when radio contact is regained
 * 
 * @details Called when RC radio signal from pilot's transmitter is restored after
 *          previous loss. Logs the recovery event and notifies GCS. No automatic
 *          mode change is performed - pilot can now manually control the vehicle
 *          and change flight modes as needed.
 * 
 *          Recovery behavior:
 *          - Logs failsafe resolved to dataflash
 *          - Sends recovery message to GCS
 *          - No automatic mode change (vehicle remains in failsafe mode)
 *          - Pilot can now:
 *            * Override roll, pitch, yaw, throttle inputs
 *            * Use flight mode switch to select different mode
 *            * Resume manual control or continue with autonomous mission
 * 
 *          Rationale for no automatic mode change:
 *          - Pilot should consciously assess situation before resuming control
 *          - Automatic mode reversion could be unexpected or unsafe
 *          - Vehicle may have surfaced (SURFACE mode) and pilot needs to dive again
 *          - Allows pilot to make informed decision about next action
 * 
 * @note Only compiled when AP_SUB_RC_ENABLED is defined
 * @note Called from set_failsafe_radio() when RC signal restoration detected
 * @note Vehicle remains in failsafe-entered mode until pilot manually changes it
 * @note RC inputs immediately available after recovery
 * 
 * @see set_failsafe_radio() - Detects RC signal restoration
 * @see failsafe_radio_on_event() - Actions when RC signal lost
 */
void Sub::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO, LogErrorCode::FAILSAFE_RESOLVED);
    gcs().send_text(MAV_SEVERITY_WARNING, "Radio Failsafe Cleared");
}
#endif
