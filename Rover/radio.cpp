/**
 * @file radio.cpp
 * @brief Implementation of RC (Radio Control) input reading and processing for Rover
 * 
 * @details This file handles all radio control input operations including:
 * - RC channel mapping and configuration (steering, throttle, auxiliary functions)
 * - RC input validation and failsafe detection
 * - Dead-zone handling and input filtering
 * - Mode switch and auxiliary channel processing
 * 
 * RC Input Channels (typical mapping via RCMAP parameters):
 * - Steering: Typically channel 1 (roll input) - controls vehicle lateral direction
 * - Throttle: Typically channel 3 - controls forward/reverse speed
 * - Lateral: Typically channel 4 (yaw input) - for skid-steer or lateral movement
 * - Mode Switch: Typically channel 8 - handled by RC_Channel library
 * - Auxiliary Functions: Variable channels for walking robots, sailboat control, etc.
 * 
 * Failsafe Conditions:
 * - RC signal loss detected when no valid input received for >500ms
 * - Out-of-range PWM values below configured FS_THR_VALUE threshold
 * - Failsafe triggers vehicle safety response via failsafe_trigger()
 * 
 * @note RC channel mapping is configurable via RCMAP parameters and can be changed at runtime
 * @warning Proper RC failsafe configuration is critical for vehicle safety
 * 
 * Source: Rover/radio.cpp
 */

#include "Rover.h"

/**
 * @brief Configure RC input channel mapping and initialize channel parameters
 * 
 * @details This function sets up the RC control channels by mapping them to vehicle
 * control functions based on RCMAP parameters. It configures:
 * - Primary control channels (steering, throttle, lateral)
 * - Channel angle ranges and scaling
 * - Walking robot specific channels (roll, pitch, height)
 * - Sailboat control inputs
 * - ESC output scaling
 * 
 * The function uses the RC_Channel library's get_roll_channel(), get_throttle_channel(),
 * and get_yaw_channel() which respect RCMAP parameter configuration, allowing users to
 * customize which physical RC channels control which vehicle functions.
 * 
 * Channel Ranges:
 * - Steering: ±SERVO_MAX angle (typically ±4500 for 45 degrees)
 * - Throttle: ±100% (normalized percentage)
 * - Lateral: ±100% (normalized percentage)
 * - Walking robot channels: ±SERVO_MAX with 30-unit dead-zone
 * 
 * @note This function can be called at runtime to reconfigure channel mapping
 * @note When not armed, motor outputs are reconfigured for safety
 * @warning Channel pointers are guaranteed non-null by RC_Channel library
 * 
 * Source: Rover/radio.cpp:6-52
 */
void Rover::set_control_channels(void)
{
    // Map RC input channels to vehicle control functions based on RCMAP parameters
    // The RC_Channel library guarantees that these channel pointers are non-null
    // Default mapping: roll=ch1 (steering), throttle=ch3, yaw=ch4 (lateral)
    channel_steer    = &rc().get_roll_channel();      // Primary steering control
    channel_throttle = &rc().get_throttle_channel();  // Forward/reverse speed control
    channel_lateral  = &rc().get_yaw_channel();       // Lateral movement (skid-steer, omni)

    // Configure RC channel input ranges for proper scaling
    // Steering uses angular range (SERVO_MAX = 4500 centidegrees = 45 degrees)
    // Throttle and lateral use percentage range (100 = 100%)
    channel_steer->set_angle(SERVO_MAX);
    channel_throttle->set_angle(100);
    if (channel_lateral != nullptr) {
        channel_lateral->set_angle(100);
    }

    // Initialize walking robot RC input channels (optional, only if configured)
    // These allow direct body orientation control for legged vehicles
    // Channels are identified by auxiliary function assignment (RCx_OPTION parameters)
    channel_roll = rc().find_channel_for_option(RC_Channel::AUX_FUNC::ROLL);
    channel_pitch = rc().find_channel_for_option(RC_Channel::AUX_FUNC::PITCH);
    channel_walking_height = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WALKING_HEIGHT);
    // Configure walking robot channels with angular range and dead-zone
    // Dead-zone of 30 (out of 1000) = 3% prevents drift from stick centering errors
    if (channel_roll != nullptr) {
        channel_roll->set_angle(SERVO_MAX);              // ±45 degree body roll
        channel_roll->set_default_dead_zone(30);         // 3% dead-zone for stick center
    }
    if (channel_pitch != nullptr) {
        channel_pitch->set_angle(SERVO_MAX);             // ±45 degree body pitch
        channel_pitch->set_default_dead_zone(30);        // 3% dead-zone for stick center
    }
    if (channel_walking_height != nullptr) {
        channel_walking_height->set_angle(SERVO_MAX);    // Body height adjustment range
        channel_walking_height->set_default_dead_zone(30); // 3% dead-zone
    }    

    // Initialize sailboat-specific RC inputs (mainsheet, wingsail angle, etc.)
    g2.sailboat.init_rc_in();

    // Reconfigure motor and servo outputs when disarmed (safe to change configuration)
    // This allows runtime changes to output mapping without safety risks
    if (!arming.is_armed()) {
        g2.motors.setup_servo_output();
        // For rovers, safety output is TRIM throttle (typically 1500 µs PWM = stopped)
        // This differs from multirotors where safety is motors off
        g2.motors.setup_safety_output();
    }
    
    // Configure ESC PWM scaling for proper throttle control
    // Default range: 1000-2000 µs PWM for standard hobby ESCs
    // DroneCAN/UAVCAN ESCs use proportion of speed (0.0-1.0) instead of PWM
    // This scaling is overridden for throttle channels by set_esc_scaling_for()
    hal.rcout->set_esc_scaling(1000, 2000);
    g2.servo_channels.set_esc_scaling_for(SRV_Channel::k_throttle);
}

/**
 * @brief Initialize RC input parameters including dead-zones
 * 
 * @details Sets up dead-zone values for primary RC input channels to prevent
 * control jitter from transmitter stick centering imperfections. Dead-zone is
 * the range around channel center (typically 1500 µs PWM) where input is treated
 * as zero/neutral.
 * 
 * Dead-zone value of 30 represents 3% of full range:
 * - Full RC range: 1000-2000 µs (1000 units)
 * - Dead-zone: 30 units = 30 µs around center
 * - Center PWM: 1500 µs ± 30 µs (1470-1530 µs) = neutral
 * 
 * This prevents unwanted vehicle movement from:
 * - Transmitter trim drift
 * - Stick mechanical centering errors
 * - RF noise on RC signal
 * 
 * @note Called during vehicle initialization after RC channels are configured
 * @note Dead-zones are applied in RC_Channel library during input reading
 * 
 * Source: Rover/radio.cpp:54-62
 */
void Rover::init_rc_in()
{
    // Configure input dead-zones for primary control channels
    // 30 units = 3% dead-zone around stick center to prevent control jitter
    channel_steer->set_default_dead_zone(30);      // Steering dead-zone
    channel_throttle->set_default_dead_zone(30);   // Throttle dead-zone
    if (channel_lateral != nullptr) {
        channel_lateral->set_default_dead_zone(30); // Lateral control dead-zone
    }
}

/**
 * @brief Read and validate RC input, check for failsafe conditions
 * 
 * @details This is the main RC input reading function, called at regular intervals
 * (typically 50 Hz) by the scheduler. It performs the following operations:
 * 
 * 1. Read RC input from receiver via RC_Channel library
 * 2. Validate received PWM values are in acceptable range
 * 3. Update last valid RC timestamp for failsafe detection
 * 4. Trigger failsafe if signal is lost or invalid
 * 
 * RC Input Validation:
 * - rc().read_input() returns false if no new data or receiver disconnected
 * - Valid PWM range: typically 1000-2000 µs (can vary by protocol)
 * - Failsafe threshold: configurable via FS_THR_VALUE parameter
 * 
 * Failsafe Detection:
 * - Signal loss: No valid input for >500ms (checked in radio_failsafe_check)
 * - Low throttle: PWM below FS_THR_VALUE (if FS_THR_ENABLE configured)
 * - Out-of-range: PWM values outside valid protocol range
 * 
 * After Failsafe Trigger:
 * - Vehicle enters configured failsafe action (Hold, RTL, SmartRTL, etc.)
 * - Notification system alerts user (LEDs, buzzer, GCS message)
 * - Mode changes restricted until RC recovered or manual intervention
 * 
 * @note Mode switch (flight mode selection) is processed by RC_Channel library
 * @note Auxiliary functions (camera trigger, lights, etc.) processed by RC_Channel library
 * @note This function does NOT directly process stick inputs - that's done in control loops
 * @warning Critical for vehicle safety - called every scheduler cycle
 * 
 * Source: Rover/radio.cpp:64-75
 */
void Rover::read_radio()
{
    // Attempt to read new RC input data from receiver
    // Returns false if no new data available or receiver connection lost
    if (!rc().read_input()) {
        // No valid RC input received - check if this triggers failsafe condition
        // Use last known throttle PWM value for failsafe threshold check
        radio_failsafe_check(channel_throttle->get_radio_in());
        return;
    }

    // Valid RC input received - update timestamp for failsafe timeout detection
    // This timestamp is checked in radio_failsafe_check() to detect signal loss >500ms
    failsafe.last_valid_rc_ms = AP_HAL::millis();
    
    // Validate received RC values are within acceptable range
    // Check throttle channel PWM against FS_THR_VALUE threshold
    // This detects out-of-range values that may indicate interference or malfunction
    radio_failsafe_check(channel_throttle->get_radio_in());
}

/**
 * @brief Check for radio failsafe conditions and trigger failsafe action if needed
 * 
 * @details Evaluates RC signal health and triggers failsafe response if signal is
 * lost or invalid. Two failsafe conditions are monitored:
 * 
 * 1. PWM Threshold Check:
 *    - Throttle PWM below FS_THR_VALUE parameter indicates failsafe pulse from receiver
 *    - Many RC receivers output low throttle (typically 900-950 µs) when signal lost
 *    - This allows detection of failsafe even if receiver continues outputting PWM
 * 
 * 2. Signal Timeout Check:
 *    - No valid RC input received for >500ms (half second)
 *    - Indicates complete signal loss or receiver power failure
 *    - Timeout value balances responsiveness vs false triggers from brief interference
 * 
 * Failsafe Configuration (via parameters):
 * - FS_THR_ENABLE: 0=disabled, 1=enabled, 2=enabled (continue in Auto modes)
 * - FS_THR_VALUE: PWM threshold in microseconds (typical: 910-950 µs)
 * - FS_ACTION: Action to take (0=Hold, 1=RTL, 2=Hold+Disarm, 3=SmartRTL)
 * 
 * Failsafe Behavior:
 * - Notification flags set (LEDs, buzzer, GCS message)
 * - failsafe_trigger() initiates configured action
 * - Vehicle enters safe state (typically Hold or RTL)
 * - Mode switching restricted until failsafe cleared
 * 
 * @param[in] pwm Current throttle channel PWM value in microseconds (typically 1000-2000)
 * 
 * @note Called by read_radio() every scheduler cycle (~50 Hz)
 * @note Failsafe state is sticky - requires valid RC signal to clear
 * @warning Critical safety function - do not modify without thorough testing
 * @warning 500ms timeout means vehicle could travel significant distance before failsafe triggers
 * 
 * Source: Rover/radio.cpp:77-91
 */
void Rover::radio_failsafe_check(uint16_t pwm)
{
    // Check if radio failsafe is enabled via FS_THR_ENABLE parameter
    if (!g.fs_throttle_enabled) {
        // Radio failsafe disabled by configuration - clear failsafe flag and return
        AP_Notify::flags.failsafe_radio = false;
        return;
    }

    // Check condition 1: Throttle PWM below failsafe threshold
    // Detects receiver failsafe output (low throttle pulse when signal lost)
    bool failed = pwm < static_cast<uint16_t>(g.fs_throttle_value);
    
    // Check condition 2: RC signal timeout (no valid input for >500ms)
    // Detects complete signal loss or receiver failure
    if (AP_HAL::millis() - failsafe.last_valid_rc_ms > 500) {
        failed = true;
    }
    
    // Update notification system with failsafe state (controls LEDs, buzzer, GCS alerts)
    AP_Notify::flags.failsafe_radio = failed;
    
    // Trigger failsafe action if conditions met (Hold, RTL, SmartRTL, etc.)
    // The failsafe_trigger() function handles mode transitions and pilot notifications
    failsafe_trigger(FAILSAFE_EVENT_THROTTLE, "Radio", failed);
}
