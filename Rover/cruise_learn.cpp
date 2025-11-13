/**
 * @file cruise_learn.cpp
 * @brief Implementation of cruise speed learning allowing operator to teach optimal cruise throttle/speed for current terrain
 * 
 * @details This module implements an adaptive cruise learning system that enables operators
 *          to teach the rover optimal throttle and speed settings for autonomous missions.
 *          The operator manually drives the vehicle at the desired cruise speed for the
 *          current terrain conditions, and the system learns and saves these values.
 * 
 *          Cruise Learning State Machine:
 *          - DISABLED: cruise_learn.learn_start_ms == 0 (learning not active)
 *          - LEARNING: cruise_learn.learn_start_ms > 0 (actively recording throttle/speed)
 *          - COMPLETED: After 2 seconds of learning, values are saved and state returns to DISABLED
 * 
 *          Learning Process:
 *          1. Operator initiates cruise learning (typically via RC switch)
 *          2. System captures current forward speed and throttle values
 *          3. Low-pass filters continuously update with current values at 50Hz
 *          4. After 2 seconds, filtered values are saved to parameters:
 *             - THROTTLE_CRUISE: Base throttle percentage for auto missions
 *             - SPEED_CRUISE: Target speed (m/s) for auto missions
 * 
 *          The learned values optimize performance for specific terrain conditions
 *          (e.g., steep hills, soft sand, mud) that may require different throttle
 *          settings than default values.
 * 
 * @note Cruise learning requires the vehicle to be armed and have valid speed feedback
 * @warning Only initiate cruise learning in safe, open areas with adequate control
 * 
 * @see g.throttle_cruise parameter for learned throttle value
 * @see g.speed_cruise parameter for learned speed value
 */
#include "Rover.h"

/**
 * @brief Start cruise throttle and speed learning process
 * 
 * @details Initiates the cruise learning state machine by capturing initial speed and
 *          throttle values. The system uses low-pass filters to smooth the measured
 *          values over the learning period.
 * 
 *          Prerequisites for starting cruise learn:
 *          - Vehicle must be armed (safety requirement)
 *          - Valid forward speed must be available from attitude controller
 * 
 *          On successful start:
 *          - Speed and throttle filters are initialized with current values
 *          - Learning timestamp (learn_start_ms) is set to current time
 *          - Log counter is reset for 10Hz logging during learning
 *          - State transitions from DISABLED to LEARNING
 * 
 *          If prerequisites not met:
 *          - learn_start_ms remains 0 (DISABLED state)
 *          - Critical message sent to ground station indicating failure
 * 
 * @note This function is typically called when operator activates cruise learn switch
 * @note Learning duration is fixed at 2 seconds (see cruise_learn_update)
 * 
 * @see cruise_learn_update() for ongoing filter updates during learning
 * @see cruise_learn_complete() for final value storage
 */
void Rover::cruise_learn_start()
{
    // Check prerequisites: vehicle must be armed and speed feedback must be available
    float speed;
    if (!arming.is_armed() || !g2.attitude_control.get_forward_speed(speed)) {
        cruise_learn.learn_start_ms = 0;  // Ensure state remains DISABLED
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Cruise Learning NOT started");
        return;
    }
    
    // Initialize learning: reset filters with current values and transition to LEARNING state
    cruise_learn.speed_filt.reset(speed);                          // Initialize speed filter with current forward speed
    cruise_learn.throttle_filt.reset(g2.motors.get_throttle());    // Initialize throttle filter with current throttle output
    cruise_learn.learn_start_ms = AP_HAL::millis();                // Set timestamp to transition to LEARNING state (non-zero value)
    cruise_learn.log_count = 0;                                     // Reset log counter for 10Hz logging
#if HAL_LOGGING_ENABLED
    log_write_cruise_learn();
#endif
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Cruise Learning started");
}

/**
 * @brief Update cruise learning with latest speed and throttle measurements
 * 
 * @details Continuously updates the low-pass filters with current speed and throttle
 *          values during the LEARNING state. This function implements the core
 *          learning algorithm by smoothing operator inputs over time.
 * 
 *          Filter Update Process:
 *          - Applies 0.02 second time constant (50Hz update rate) to smooth measurements
 *          - Speed filter: Tracks current forward speed from attitude controller
 *          - Throttle filter: Tracks current throttle output from motor controller
 * 
 *          Learning Duration:
 *          - Fixed 2-second learning period to capture stable operator input
 *          - Automatically completes learning after 2000ms elapsed
 * 
 *          Logging:
 *          - Records filtered values at 10Hz (every 5th call at 50Hz rate)
 *          - Logs CRSE message with current state and filtered values
 * 
 * @note MUST be called at 50Hz for correct filter time constant (0.02s)
 * @note Only active when in LEARNING state (cruise_learn.learn_start_ms > 0)
 * @note If speed feedback becomes unavailable during learning, updates are skipped
 * 
 * @see cruise_learn_start() for learning initiation
 * @see cruise_learn_complete() for automatic completion after 2 seconds
 */
void Rover::cruise_learn_update()
{
    float speed;
    // Only update if in LEARNING state and valid speed available
    if (cruise_learn.learn_start_ms > 0 && g2.attitude_control.get_forward_speed(speed)) {
        // Apply low-pass filters to smooth speed and throttle measurements over learning period
        cruise_learn.speed_filt.apply(speed, 0.02f);                        // 0.02s time constant (50Hz rate)
        cruise_learn.throttle_filt.apply(g2.motors.get_throttle(), 0.02f);  // 0.02s time constant (50Hz rate)
        
        // Log filtered values at 10Hz (every 5 calls at 50Hz = 10Hz logging rate)
#if HAL_LOGGING_ENABLED
        if (cruise_learn.log_count % 5 == 0) {
            log_write_cruise_learn();
        }
        cruise_learn.log_count += 1;
#endif
        
        // Check if 2-second learning period has elapsed
        if (AP_HAL::millis() - cruise_learn.learn_start_ms >= 2000) {
            cruise_learn_complete();  // Automatically complete learning and save parameters
        }
        return;
    }
}

/**
 * @brief Complete cruise learning and save results to parameters
 * 
 * @details Finalizes the cruise learning process by retrieving filtered values and
 *          saving them to persistent parameters. This function performs validation
 *          before updating parameters to ensure learned values are safe and reasonable.
 * 
 *          Validation Criteria:
 *          - Throttle: Must be between 10% and 100% (safety limits)
 *          - Speed: Must be positive (forward motion required)
 * 
 *          Parameter Updates (on successful validation):
 *          - THROTTLE_CRUISE (g.throttle_cruise): Saved to EEPROM/flash for persistence
 *            * Used as base throttle in AUTO, GUIDED, RTL, and STEERING modes
 *            * Represents optimal throttle for current terrain conditions
 *          - SPEED_CRUISE (g.speed_cruise): Saved to EEPROM/flash for persistence
 *            * Used as target speed in speed-controlled autonomous modes
 *            * Represents safe cruise speed for current terrain
 * 
 *          State Transition:
 *          - Resets learn_start_ms to 0 (returns to DISABLED state)
 *          - Final log message written with learned values
 *          - Ground station notified of success or failure
 * 
 * @note This function is called automatically after 2-second learning period
 * @note Can also be called manually if operator ends learning early
 * @note Parameter changes are saved to persistent storage (survive reboot)
 * 
 * @warning Learned values directly affect autonomous mode behavior
 * @warning Invalid learned values (outside safety limits) are rejected
 * 
 * @see g.throttle_cruise parameter used in autonomous throttle control
 * @see g.speed_cruise parameter used in speed controller
 */
void Rover::cruise_learn_complete()
{
    // Only complete if currently in LEARNING state
    if (cruise_learn.learn_start_ms > 0) {
        // Retrieve final filtered values from learning period
        const float thr = cruise_learn.throttle_filt.get();      // Learned throttle percentage
        const float speed = cruise_learn.speed_filt.get();        // Learned cruise speed (m/s)
        
        // Validate learned values are within safe operating limits
        if (thr >= 10.0f && thr <= 100.0f && is_positive(speed)) {
            // Update and save cruise parameters to persistent storage
            g.throttle_cruise.set_and_save(thr);      // Save THROTTLE_CRUISE parameter (used in auto throttle control)
            g.speed_cruise.set_and_save(speed);        // Save SPEED_CRUISE parameter (used in speed controller)
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Cruise Learned: Thr:%d Speed:%3.1f", (int)g.throttle_cruise, (double)g.speed_cruise);
        } else {
            // Learning failed validation - parameters not updated
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Cruise Learning failed");
        }
        
        // Transition back to DISABLED state regardless of validation result
        cruise_learn.learn_start_ms = 0;
        
#if HAL_LOGGING_ENABLED
        // Write final log entry showing completion state
        log_write_cruise_learn();
#endif
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Write cruise learning data to onboard log
 * 
 * @details Records the current state of cruise learning for post-flight analysis
 *          and debugging. Log entries capture the filtered speed and throttle values
 *          throughout the learning process.
 * 
 *          Log Message Format (CRSE):
 *          - TimeUS: Microsecond timestamp since system startup
 *          - State: Boolean indicating LEARNING state (true) or DISABLED (false)
 *          - Speed: Current filtered cruise speed in m/s
 *          - Throttle: Current filtered throttle percentage (0-100)
 * 
 *          Logging Frequency:
 *          - Initial: Logged once at start of learning
 *          - During Learning: Logged at 10Hz throughout 2-second period
 *          - Final: Logged once at completion (success or failure)
 * 
 * @note Logs are written to dataflash/SD card for post-flight analysis
 * @note CRSE logs can be analyzed to verify learning behavior and tuning
 * 
 * @see cruise_learn_update() for 10Hz logging calls during learning
 * @see https://ardupilot.org/rover/docs/rover-tuning-throttle-and-speed.html for tuning guide
 */
void Rover::log_write_cruise_learn() const
{
// @LoggerMessage: CRSE
// @Description: Cruise Learn messages
// @URL: https://ardupilot.org/rover/docs/rover-tuning-throttle-and-speed.html
// @Field: TimeUS: Time since system startup
// @Field: State: True if Cruise Learn has started
// @Field: Speed: Determined target Cruise speed in auto missions
// @Field: Throttle: Determined base throttle percentage to be used in auto missions 

    AP::logger().WriteStreaming("CRSE", "TimeUS,State,Speed,Throttle", "Qbff",
                       AP_HAL::micros64(),
                       cruise_learn.learn_start_ms > 0,
                       cruise_learn.speed_filt.get(),
                       cruise_learn.throttle_filt.get());
}
#endif
