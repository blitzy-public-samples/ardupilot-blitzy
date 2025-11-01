/**
 * @file compassmot.cpp
 * @brief Compass motor interference calibration for multicopter vehicles
 * 
 * @details This file implements in-flight compass calibration to measure and
 *          compensate for magnetic interference caused by motor currents and ESC
 *          operation. The calibration procedure measures the compass field at
 *          various throttle/current levels and calculates compensation vectors
 *          that are applied during normal flight to improve heading accuracy.
 *          
 *          Calibration Process:
 *          1. Record baseline compass readings at zero throttle
 *          2. Gradually increase throttle while measuring compass deviation
 *          3. Calculate interference vectors scaled by throttle or current
 *          4. Store compensation factors to persistent storage
 *          
 *          This is a SAFETY-CRITICAL procedure that requires:
 *          - Vehicle securely restrained or in controlled test environment
 *          - Propellers installed (motors must be loaded normally)
 *          - Clear area around vehicle
 *          - Experienced operator supervision
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Copter.h"

/**
 * @brief Perform compass motor interference calibration procedure
 * 
 * @details This function executes an interactive calibration routine to measure
 *          and compensate for magnetic interference from motor operation. The
 *          calibration runs in real-time while the operator manually controls
 *          throttle from zero to maximum, allowing the algorithm to measure
 *          compass field distortion across the throttle range.
 *          
 *          Algorithm Overview:
 *          - Baseline Measurement: Records compass field at zero throttle
 *          - Active Sampling: Measures deviation as throttle increases
 *          - Compensation Calculation: Builds interference model scaled by
 *            throttle position or current draw (whichever is available)
 *          - Filtering: Uses exponential moving average (99% old, 1% new) for
 *            smooth convergence and noise rejection
 *          
 *          Compensation Types:
 *          - AP_COMPASS_MOT_COMP_THROTTLE: Scales interference by throttle %
 *            (used when current sensing unavailable)
 *          - AP_COMPASS_MOT_COMP_CURRENT: Scales interference by current draw
 *            in amps (preferred method, more accurate across voltage changes)
 *          
 *          Safety Checks (performed before starting):
 *          - Compass enabled and healthy
 *          - RC radio calibrated
 *          - Throttle at zero position
 *          - Vehicle landed
 *          - Not already running compassmot
 *          
 *          Execution Flow:
 *          1. Validate preconditions (compass health, throttle zero, landed)
 *          2. Disable failsafes to prevent interruption
 *          3. Record baseline compass readings at zero throttle
 *          4. Arm motors and enable throttle passthrough
 *          5. Enter measurement loop (50Hz):
 *             - Read compass, throttle, and current
 *             - Calculate motor-induced magnetic field deviation
 *             - Update compensation vectors with exponential filter
 *             - Stream telemetry to ground station
 *          6. Exit on user command or error condition
 *          7. Save compensation factors to persistent storage
 *          8. Restore normal failsafe configuration
 * 
 * @param[in] gcs_chan MAVLink channel for telemetry and status messages
 *                     Used to send real-time calibration progress including:
 *                     - Current throttle position
 *                     - Battery current draw
 *                     - Calculated interference percentage
 *                     - Compensation vector components
 * 
 * @return MAV_RESULT indicating calibration outcome:
 *         - MAV_RESULT_ACCEPTED: Calibration completed successfully, 
 *           compensation factors saved
 *         - MAV_RESULT_UNSUPPORTED: Not supported on this frame type 
 *           (traditional helicopters)
 *         - MAV_RESULT_TEMPORARILY_REJECTED: Precondition failed (compass
 *           disabled, unhealthy compass, RC not calibrated, throttle not zero,
 *           not landed, or already running)
 * 
 * @note This calibration requires PROPELLERS INSTALLED as motors must be under
 *       normal load to generate representative magnetic interference patterns.
 *       Running without propellers will produce invalid calibration data.
 * 
 * @note Calibration runs at 50Hz (20ms loop) with 500ms telemetry updates
 * 
 * @note The function disables CPU and throttle failsafes during execution to
 *       prevent automatic safety interventions from interrupting calibration
 * 
 * @note Current-based compensation requires minimum 3A draw for measurements
 *       to ensure signal is above noise threshold
 * 
 * @warning SAFETY-CRITICAL PROCEDURE: This function arms motors and spins
 *          propellers while the vehicle is on the ground. Vehicle MUST be
 *          securely restrained or in a controlled test environment. Operator
 *          must maintain constant control and be prepared to cut throttle
 *          immediately if vehicle becomes unstable. Failure to properly secure
 *          vehicle can result in loss of control, property damage, or injury.
 * 
 * @warning This procedure temporarily DISABLES FAILSAFES including throttle
 *          failsafe and CPU failsafe to prevent interruption during calibration.
 *          Failsafes are restored after calibration completes.
 * 
 * @warning Magnetic interference can exceed 50% of Earth's magnetic field in
 *          extreme cases. High interference levels indicate poor compass mounting
 *          location and may result in degraded navigation performance even with
 *          compensation applied.
 * 
 * @warning DO NOT perform this calibration near ferromagnetic materials (steel
 *          structures, rebar, vehicles) as external interference will be
 *          incorrectly attributed to motors and result in invalid compensation.
 * 
 * @see AP_Compass::motor_compensation_type() for compensation mode configuration
 * @see AP_Compass::set_motor_compensation() for manual compensation vector setup
 * @see AP_Arming::compass_magfield_expected() for expected field strength reference
 * 
 * Source: ArduCopter/compassmot.cpp:8-276
 */
MAV_RESULT Copter::mavlink_compassmot(const GCS_MAVLINK &gcs_chan)
{
#if FRAME_CONFIG == HELI_FRAME
    // Traditional helicopters use different motor configurations and collective pitch
    // control that produce different magnetic interference patterns not suited to
    // this calibration algorithm. Heli-specific compensation may be added in future.
    return MAV_RESULT_UNSUPPORTED;
#else
    // Compensation method selection (throttle-based or current-based)
    int8_t   comp_type;                 // AP_COMPASS_MOT_COMP_THROTTLE or AP_COMPASS_MOT_COMP_CURRENT
    
    // Compass baseline and interference measurement vectors for each compass instance
    Vector3f compass_base[COMPASS_MAX_INSTANCES];           // baseline magnetic field at zero throttle (milliGauss)
    Vector3f motor_impact[COMPASS_MAX_INSTANCES];           // instantaneous interference vector (milliGauss)
    Vector3f motor_impact_scaled[COMPASS_MAX_INSTANCES];    // interference per unit throttle or per amp
    Vector3f motor_compensation[COMPASS_MAX_INSTANCES];     // final compensation vectors to negate interference
    
    // Throttle and current tracking for scaling calculations
    float    throttle_pct;              // current throttle position as percentage 0.0 to 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached during calibration (0.0 to 1.0)
    float    current_amps_max = 0.0f;   // maximum battery current in amps (for reporting)
    float    interference_pct[COMPASS_MAX_INSTANCES]{};       // interference magnitude as % of Earth's field (for telemetry)
    
    // Timing control for main loop and telemetry transmission
    uint32_t last_run_time;             // timestamp for 50Hz loop timing (milliseconds)
    uint32_t last_send_time;            // timestamp for 2Hz telemetry rate limiting (milliseconds)
    
    // Calibration state tracking
    bool     updated = false;           // true if at least one valid compensation sample obtained
    uint8_t  command_ack_start = command_ack_counter;  // initial value to detect user abort command

    // ============================================================================
    // SAFETY PRECONDITION CHECKS - Validate safe conditions before motor operation
    // ============================================================================
    
    // Prevent multiple simultaneous calibration attempts which could cause
    // confusion in state tracking and telemetry reporting
    if (ap.compass_mot) {
        // Already running compassmot - ignore restart messages
        return MAV_RESULT_TEMPORARILY_REJECTED;
    } else {
        // Set flag to prevent concurrent execution
        ap.compass_mot = true;
    }

    // Verify compass subsystem is enabled in vehicle configuration
    // Calibration cannot proceed without functional compass hardware
    if (!AP::compass().available()) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Compass disabled");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // Verify all configured compass instances are reporting valid data
    // Unhealthy compass indicates sensor failure, communication error, or
    // excessive magnetic interference that would invalidate calibration
    compass.read();
    for (uint8_t i=0; i<compass.get_count(); i++) {
        if (!compass.healthy(i)) {
            gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Check compass");
            ap.compass_mot = false;
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
    }

    // Verify RC radio has been properly calibrated with known min/max ranges
    // Uncalibrated radio could cause unintended throttle commands during procedure
    if (!arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "RC not calibrated");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // Verify throttle stick is at minimum position before arming motors
    // Starting with non-zero throttle could cause immediate propeller spin-up
    // creating unsafe conditions before operator is ready
    read_radio();
    if (channel_throttle->get_control_in() != 0) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Throttle not zero");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // Verify vehicle is on the ground based on landing detection sensors
    // This calibration must only run with vehicle secured on ground - running
    // in flight would be extremely dangerous and produce invalid results
    if (!ap.land_complete) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Not landed");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // ============================================================================
    // CALIBRATION SETUP AND INITIALIZATION
    // ============================================================================
    
    // Disable CPU failsafe to prevent scheduler overrun detection from aborting
    // the calibration loop. Calibration runs a tight 50Hz loop that could
    // trigger false failsafe events.
    failsafe_disable();

    float current;

    // Select compensation method based on available sensors
    // Current-based compensation is preferred as it accounts for voltage variations
    // and provides more accurate interference scaling. Falls back to throttle-based
    // if current sensing is unavailable.
    if (battery.current_amps(current)) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;  // Scale interference by amps drawn
    } else {
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;  // Scale interference by throttle %
    }

    // Send MAVLink acknowledgment to ground station confirming calibration start
    // This prevents GCS timeout while waiting for calibration to begin
    mavlink_msg_command_ack_send(gcs_chan.get_chan(), MAV_CMD_PREFLIGHT_CALIBRATION,0,
                                 0, 0, 0, 0);

    // Activate LED notification pattern to provide visual feedback that
    // calibration is in progress (typically flashing pattern)
    AP_Notify::flags.esc_calibration = true;

    // Inform operator that calibration procedure is beginning
    gcs_chan.send_text(MAV_SEVERITY_INFO, "Starting calibration");

    // Report which compensation method will be used so operator knows what
    // to expect in the calibration data
    if (comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Current");
    } else {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Throttle");
    }

    // Disable throttle failsafe to prevent RC signal loss from triggering
    // failsafe actions during calibration. Operator must maintain manual
    // control throughout procedure. Failsafe will be restored after completion.
    g.failsafe_throttle.set(FS_THR_DISABLED);

    // Disable any existing motor compensation to get raw compass readings
    // during calibration. Previous compensation values would interfere with
    // measurement of actual motor-induced magnetic fields.
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass.set_motor_compensation(i, Vector3f(0,0,0));
    }

    // Read compass to ensure fresh data before recording baseline
    compass.read();

    // Store baseline compass field measurements at zero throttle
    // These represent the ambient magnetic field without motor interference
    // and will be used as reference for calculating interference vectors
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass_base[i] = compass.get_field(i);  // Vector in milliGauss (body frame)
        interference_pct[i] = 0.0f;              // Initialize interference tracking
    }

    // Inform EKF and scheduler that we expect a delay for motor initialization
    // This prevents false timeout detections during motor arm sequence
    EXPECT_DELAY_MS(5000);

    // Arm the motors and initialize to minimum output (motors spinning at idle)
    // WARNING: Propellers will begin spinning at this point
    motors->output_min();  // Set ESC outputs to minimum (typically 1000us PWM)
    motors->armed(true);   // Set motor library armed state
    hal.util->set_soft_armed(true);  // Set system-level armed flag

    // Initialize timing variables for loop rate control and telemetry scheduling
    last_run_time = millis();   // 50Hz main loop timing
    last_send_time = millis();  // 2Hz telemetry update timing

    // ============================================================================
    // MAIN CALIBRATION LOOP - Real-time compass interference measurement
    // ============================================================================
    // Loop continues until:
    // - User sends abort command (command_ack_counter increments)
    // - Compass becomes unhealthy (sensor failure or extreme interference)
    // - Motors disarm due to error condition
    while (command_ack_start == command_ack_counter && compass.healthy() && motors->armed()) {
        // Inform scheduler that calibration loop iteration may take up to 5 seconds
        // This prevents false scheduler overrun detection during telemetry transmission
        EXPECT_DELAY_MS(5000);

        // Enforce 50Hz loop rate (20ms period) for consistent sampling
        // Faster rates would increase CPU load without improving accuracy
        // Slower rates would reduce calibration responsiveness
        if (millis() - last_run_time < 20) {
            hal.scheduler->delay(5);  // Sleep briefly to avoid busy-waiting
            continue;
        }
        last_run_time = millis();

        // Read current RC radio inputs for throttle control
        // Operator manually varies throttle during calibration
        read_radio();

        // Pass throttle input directly to motors (bypass normal flight controller)
        // This gives operator direct control over motor speed for calibration
        auto &srv = AP::srv();
        srv.cork();  // Batch servo outputs for atomic update
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() * 0.001f);
        srv.push();  // Commit all servo outputs simultaneously

        // Sample compass magnetic field measurements (all configured instances)
        // Reads raw sensor data and applies board orientation corrections
        compass.read();

        // Update battery current measurement for current-based compensation
        // Also provides current reading for telemetry reporting
        battery.read();

        // Convert throttle from integer range (0-1000) to float percentage (0.0-1.0)
        throttle_pct = (float)channel_throttle->get_control_in() * 0.001f;
        throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);  // Clamp to valid range

        // Track maximum throttle achieved during calibration for interference
        // percentage calculation (allows extrapolation to full throttle)
        throttle_pct_max = MAX(throttle_pct_max, throttle_pct);

        // Get current battery draw in amps (returns false if sensor unavailable)
        if (!battery.current_amps(current)) {
            current = 0;  // Default to zero if current sensing not available
        }
        // Track maximum current for full-power interference extrapolation
        current_amps_max = MAX(current_amps_max, current);

        // ========================================================================
        // COMPENSATION VECTOR CALCULATION - Core calibration algorithm
        // ========================================================================
        
        // When throttle is at zero (motors at idle), continuously update baseline
        // reference to track ambient magnetic field changes (e.g., vehicle rotation
        // on ground, nearby metal objects moving). Uses exponential moving average
        // for smooth tracking: 99% old value + 1% new measurement
        if (!is_positive(throttle_pct)) {
            for (uint8_t i=0; i<compass.get_count(); i++) {
                // Update baseline with low-pass filter to reject transient noise
                compass_base[i] = compass_base[i] * 0.99f + compass.get_field(i) * 0.01f;
            }
        } else {
            // Throttle is active - measure motor-induced magnetic interference
            
            // Calculate raw interference vector as difference between current reading
            // and zero-throttle baseline. This isolates the magnetic field component
            // caused by motor current flow through ESC and motor windings.
            for (uint8_t i=0; i<compass.get_count(); i++) {
                motor_impact[i] = compass.get_field(i) - compass_base[i];  // milliGauss
            }

            // THROTTLE-BASED COMPENSATION CALCULATION
            // Assumes interference scales linearly with throttle percentage
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // Normalize interference by current throttle to get interference per
                    // unit throttle. This allows compensation to scale across full throttle
                    // range: comp_vector = interference_per_throttle * current_throttle
                    motor_impact_scaled[i] = motor_impact[i] / throttle_pct;
                    
                    // Update compensation vector using exponential moving average filter
                    // (99% old, 1% new). Negative sign because compensation must OPPOSE
                    // interference to cancel it out. Filter smooths out measurement noise
                    // and provides gradual convergence to true compensation value.
                    motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled[i] * 0.01f;
                }
                updated = true;  // Flag that we have valid calibration data
                
            // CURRENT-BASED COMPENSATION CALCULATION (preferred method)
            // Scales interference by battery current draw, more accurate across
            // different battery voltages and ESC configurations
            } else {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // Only update compensation when drawing significant current (>3A)
                    // to ensure signal is well above measurement noise threshold.
                    // Below 3A, interference is too small for reliable measurement.
                    if (current >= 3.0f) {
                        // Normalize interference by current draw to get interference per amp
                        // This produces compensation that scales with motor current:
                        // comp_vector = interference_per_amp * battery_current
                        motor_impact_scaled[i] = motor_impact[i] / current;
                        
                        // Update compensation with exponential filter, opposing interference
                        motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled[i] * 0.01f;
                        updated = true;  // Valid sample obtained
                    }
                }
            }

            // ====================================================================
            // INTERFERENCE PERCENTAGE CALCULATION - For telemetry and diagnostics
            // ====================================================================
            // Calculate interference magnitude as percentage of Earth's magnetic field
            // to help operator assess compass mounting quality and expected accuracy
            
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // For throttle-based: compensation vector magnitude represents
                    // full-throttle interference. Divide by expected Earth field (mG)
                    // and multiply by 100 for percentage.
                    // >30% indicates poor compass location, >50% is problematic
                    interference_pct[i] = motor_compensation[i].length() / (float)arming.compass_magfield_expected() * 100.0f;
                }
            } else {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // For current-based: extrapolate to full-power interference by
                    // multiplying per-amp interference by max current at full throttle
                    // (estimated as max_current_seen / max_throttle_seen)
                    interference_pct[i] = motor_compensation[i].length() * (current_amps_max/throttle_pct_max) / (float)arming.compass_magfield_expected() * 100.0f;
                }
            }
        }

        // ====================================================================
        // TELEMETRY TRANSMISSION - Stream calibration progress to ground station
        // ====================================================================
        // Send updates at 2Hz (every 500ms) to avoid overwhelming MAVLink bandwidth
        // while providing responsive feedback to operator
        if (AP_HAL::millis() - last_send_time > 500) {
            last_send_time = AP_HAL::millis();
            
            // Send compassmot-specific status message with real-time calibration data
            // Includes throttle position, current draw, interference %, and compensation
            // vector components. Ground station displays this for operator monitoring.
            mavlink_msg_compassmot_status_send(gcs_chan.get_chan(),
                                               channel_throttle->get_control_in(),  // Throttle PWM
                                               current,                              // Battery amps
                                               interference_pct[0],                  // Primary compass interference %
                                               motor_compensation[0].x,              // X-axis compensation (mG)
                                               motor_compensation[0].y,              // Y-axis compensation (mG)
                                               motor_compensation[0].z);             // Z-axis compensation (mG)
#if HAL_WITH_ESC_TELEM
            // If ESC telemetry available, stream temperature and RPM data to allow
            // operator to monitor for overheating during extended calibration runs
            AP::esc_telem().send_esc_telemetry_mavlink(gcs_chan.get_chan());
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // SITL-specific messages required for autotest framework compatibility
            // These prevent autotest timeouts during long-running calibration tests
            gcs_chan.send_system_time();      // Maintain autotest heartbeat
            gcs_chan.send_rc_channels();      // Allow autotest to monitor RC inputs
#endif
        }
    }  // End of main calibration loop

    // ============================================================================
    // CLEANUP AND RESULT STORAGE - Restore normal operation
    // ============================================================================
    
    // Disarm motors immediately and return to idle output
    // SAFETY: Propellers will stop spinning at this point
    motors->output_min();               // Set ESC outputs to minimum PWM
    motors->armed(false);               // Clear motor library armed flag
    hal.util->set_soft_armed(false);    // Clear system-level armed state

    // Save calibration results to persistent storage if valid data obtained
    if (updated) {
        // At least one valid compensation sample was recorded
        
        // Enable the compensation type (throttle or current-based)
        compass.motor_compensation_type(comp_type);
        
        // Store compensation vectors for all compass instances
        // These will be applied automatically during normal flight to cancel
        // motor-induced magnetic interference
        for (uint8_t i=0; i<compass.get_count(); i++) {
            compass.set_motor_compensation(i, motor_compensation[i]);
        }
        
        // Write compensation parameters to EEPROM/flash for persistence across reboots
        // Parameters: MOT_COMP_TYPE, COMPASS_MOT_X/Y/Z for each compass instance
        compass.save_motor_compensation();
        
        // Notify operator that calibration completed successfully
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Calibration successful");
    } else {
        // No valid compensation samples obtained - calibration failed
        // This can happen if throttle never exceeded minimum or current never exceeded 3A
        gcs_chan.send_text(MAV_SEVERITY_NOTICE, "Failed");
        
        // Disable compensation to prevent using potentially invalid data
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // Deactivate LED notification pattern to signal calibration complete
    AP_Notify::flags.esc_calibration = false;

    // Re-enable CPU failsafe to restore normal scheduler overrun detection
    failsafe_enable();

    // Restore throttle failsafe configuration from saved parameter value
    // This re-enables RC signal loss protection that was disabled during calibration
    g.failsafe_throttle.load();

    // Clear compassmot in-progress flag to allow future calibration runs
    ap.compass_mot = false;

    // Return success to ground station (even if no compensation saved, procedure completed)
    return MAV_RESULT_ACCEPTED;
#endif  // FRAME_CONFIG != HELI_FRAME
}
