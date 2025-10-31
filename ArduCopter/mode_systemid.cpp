/**
 * @file mode_systemid.cpp
 * @brief System Identification (SystemID) flight mode implementation for ArduCopter
 * 
 * @details This file implements the SystemID flight mode, which is designed for
 *          system identification and dynamics analysis of multicopter vehicles.
 *          SystemID mode injects test signals (chirps, frequency sweeps, doublets)
 *          into various control loops to characterize vehicle response and dynamics.
 *          
 *          The mode is primarily used by:
 *          - Researchers studying vehicle dynamics and control
 *          - Advanced tuners optimizing PID parameters
 *          - Engineers validating mathematical models
 *          - Developers characterizing new vehicle configurations
 *          
 *          SystemID mode supports injection on multiple axes:
 *          - Attitude inputs (roll, pitch, yaw angles)
 *          - Rate loop inputs (roll, pitch, yaw rates)
 *          - Mixer outputs (direct actuator commands)
 *          - Position/velocity inputs (for position control testing)
 *          
 *          Test signals are typically logarithmic chirp sweeps that start at a
 *          low frequency and sweep to a high frequency, exciting the system across
 *          the frequency range of interest. The resulting sensor data is logged
 *          at high rate and can be post-processed with system identification tools
 *          to extract frequency response data (Bode plots, transfer functions).
 * 
 * @note This is an advanced flight mode requiring careful setup and supervision.
 *       The vehicle must be flying in a stabilized mode before switching to SystemID.
 *       
 * @warning SystemID mode deliberately injects disturbances into the control system
 *          and can cause significant vehicle motion. Only use in safe environments
 *          with adequate clearance. Always enable data logging before testing.
 * 
 * @see AP_Math/chirp.h for chirp signal generation implementation
 * @see Log_Write_SysID_Setup() for logged setup parameters
 * @see Log_Write_SysID_Data() for logged test data
 * 
 * Source: ArduCopter/mode_systemid.cpp
 */

#include "Copter.h"
#include <AP_Math/control.h>

#if MODE_SYSTEMID_ENABLED

const AP_Param::GroupInfo ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust, 14:Measured Lateral Position, 15:Measured Longitudinal Position, 16:Measured Lateral Velocity, 17:Measured Longitudinal Velocity, 18:Input Lateral Velocity, 19:Input Longitudinal Velocity
    AP_GROUPINFO_FLAGS("_AXIS", 1, ModeSystemId, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, ModeSystemId, waveform_magnitude, 15),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, ModeSystemId, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, ModeSystemId, frequency_stop, 40),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, ModeSystemId, time_fade_in, 15),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, ModeSystemId, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, ModeSystemId, time_fade_out, 2),

    AP_GROUPEND
};

ModeSystemId::ModeSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @def SYSTEM_ID_DELAY
 * @brief Delay in seconds after mode entry before starting test signal injection
 * 
 * @details This delay allows the vehicle to stabilize after switching to SystemID mode
 *          before the chirp/sweep signal begins. This ensures cleaner data collection
 *          by avoiding transients from the mode switch itself.
 */
#define SYSTEM_ID_DELAY     1.0f      // time in seconds waited after system id mode change for frequency sweep injection

/**
 * @brief Initialize SystemID flight mode
 * 
 * @details This function performs comprehensive initialization and validation before
 *          entering SystemID mode. It verifies that:
 *          - A valid axis is selected (SID_AXIS parameter is non-zero)
 *          - Vehicle is currently flying (armed, auto-armed, not landed)
 *          - Current flight mode is compatible with selected axis type
 *          - All test parameters are valid (frequencies, timing, magnitude)
 *          
 *          For attitude/rate axes (1-13), the vehicle must be in a mode with manual
 *          throttle control. For position/velocity axes (14-19), the vehicle must
 *          be in Loiter mode to ensure position controller is active.
 *          
 *          Initialization sequence:
 *          1. Validate axis selection and parameter configuration
 *          2. Check vehicle flight state (armed, flying)
 *          3. Verify mode compatibility with selected axis
 *          4. Initialize position controllers (for poscontrol axes)
 *          5. Configure chirp signal generator with sweep parameters
 *          6. Record initial feedforward state for later restoration
 *          7. Log test configuration to dataflash
 *          
 *          The chirp signal is configured to perform a logarithmic frequency sweep
 *          from SID_F_START_HZ to SID_F_STOP_HZ over SID_T_REC seconds, with
 *          fade-in and fade-out times to reduce transients.
 * 
 * @param[in] ignore_checks Not used in SystemID mode - checks are always performed
 *                          for safety (parameter retained for Mode interface compatibility)
 * 
 * @return true if initialization successful and mode can be entered
 * @return false if validation failed - mode entry denied with GCS warning message
 * 
 * @note For helicopter configurations (HELI_FRAME), stabilize collective is enabled
 *       to maintain altitude control during attitude axis testing.
 * 
 * @warning This function will reject mode entry if the vehicle is not flying.
 *          Never attempt to arm or takeoff in SystemID mode.
 * 
 * @see ModeSystemId::run() for test signal injection during mode execution
 * @see chirp_input.init() for chirp signal configuration
 * 
 * Source: ArduCopter/mode_systemid.cpp:76-151
 */
bool ModeSystemId::init(bool ignore_checks)
{
    // check if enabled
    if (axis == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No axis selected, SID_AXIS = 0");
        return false;
    }

    // ensure we are flying
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Aircraft must be flying");
        return false;
    }

    if (!is_poscontrol_axis_type()) {

        // System ID is being done on the attitude control loops
        // Axes 1-13: attitude inputs, rate inputs, or mixer outputs
        // These axes inject signals directly into attitude/rate controllers or motor mixing

        // Can only switch into System ID Axes 1-13 with a flight mode that has manual throttle
        // Manual throttle ensures pilot maintains altitude control during attitude excitation
        if (!copter.flightmode->has_manual_throttle()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Axis requires manual throttle");
            return false;
        }

#if FRAME_CONFIG == HELI_FRAME
        // Helicopter: Enable stabilize collective to maintain altitude during testing
        copter.input_manager.set_use_stab_col(true);
#endif

    } else {

        // System ID is being done on the position control loops
        // Axes 14-19: position/velocity disturbances or velocity inputs
        // These axes inject signals into the position controller to test outer loop response

        // Can only switch into System ID Axes 14-19 from Loiter flight mode
        // Loiter ensures position controller is active and vehicle is stabilized
        if (copter.flightmode->mode_number() != Mode::Number::LOITER) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Axis requires switch from Loiter");
            return false;
        }

        // Set horizontal speed and acceleration limits from waypoint navigation defaults
        // These limits constrain position controller response during testing
        pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
        pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

        // Initialize the horizontal position controller (North-East axes)
        // Ensures position controller is ready to accept target commands
        if (!pos_control->is_active_NE()) {
            pos_control->init_NE_controller();
        }

        // Set vertical speed and acceleration limits (Up axis, NED frame)
        // Separate limits for climb and descent rates
        pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
        pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

        // Initialize the vertical position controller
        // Maintains altitude during horizontal position testing
        if (!pos_control->is_active_U()) {
            pos_control->init_U_controller();
        }
        
        // Capture current position as reference target for position axes
        // Test signals will be added as disturbances or velocity inputs relative to this position
        Vector3f curr_pos;
        curr_pos = pos_control->get_pos_estimate_NEU_cm().tofloat();
        target_pos = curr_pos.xy();
    }

    // Store current body-frame feedforward state to restore on mode exit
    // Some recovery axes disable feedforward to test pure feedback response
    att_bf_feedforward = attitude_control->get_bf_feedforward();
    
    // Initialize test timing and state
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // Two full cycles at the starting frequency (initial constant-frequency dwell)
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0; // Reset log subsampling counter for high-rate data logging

    // Initialize chirp signal generator with user-configured sweep parameters
    // Creates logarithmic frequency sweep: frequency_start -> frequency_stop over time_record
    // Fade-in/fade-out reduce transients at start/end of sweep
    // time_const_freq: initial dwell period at start frequency for settling
    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

#if HAL_LOGGING_ENABLED
    copter.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);
#endif

    return true;
}

/**
 * @brief Clean up and restore settings when exiting SystemID mode
 * 
 * @details This function is called automatically when switching from SystemID mode
 *          to any other flight mode. It restores control system parameters that
 *          may have been modified during SystemID testing.
 *          
 *          Currently restores:
 *          - Body-frame feedforward state (some recovery axes disable this temporarily)
 *          
 *          Future cleanup tasks could include:
 *          - Clearing any accumulated position disturbances
 *          - Resetting rate controller integrator terms
 *          - Stopping any ongoing log streaming
 * 
 * @note This function is called even if SystemID test has not started or was aborted.
 *       All cleanup operations must be safe to execute in any state.
 * 
 * @see ModeSystemId::init() for parameter initialization
 * 
 * Source: ArduCopter/mode_systemid.cpp:154-158
 */
void ModeSystemId::exit()
{
    // Reset the feedforward enabled parameter to the initialized state
    // Recovery axes (RECOVER_ROLL, RECOVER_PITCH, RECOVER_YAW) temporarily disable
    // feedforward to test pure feedback controller response. Restore original setting.
    attitude_control->bf_feedforward(att_bf_feedforward);
}

/**
 * @brief Main execution function for SystemID mode - called every main loop iteration
 * 
 * @details This function implements the core SystemID test signal injection and control.
 *          It is called at the main loop rate (typically 400Hz) and performs:
 *          
 *          1. **Pilot Input Processing** (for non-poscontrol axes):
 *             - Reads pilot stick inputs for roll, pitch, yaw, throttle
 *             - Applies simple mode transformations if enabled
 *             - Manages motor spool state based on throttle input
 *             
 *          2. **Test Signal Generation**:
 *             - Updates chirp waveform based on elapsed time
 *             - Generates logarithmic frequency sweep signal
 *             - Applies fade-in/fade-out envelope to reduce transients
 *             - Tracks instantaneous frequency for logging
 *             
 *          3. **Signal Injection** (axis-dependent):
 *             - **Attitude Axes (1-6)**: Adds angle commands to pilot input
 *             - **Rate Axes (7-9)**: Injects rate commands into rate controllers
 *             - **Mixer Axes (10-13)**: Direct actuator/throttle commands
 *             - **Position Axes (14-17)**: Position/velocity disturbances
 *             - **Velocity Input Axes (18-19)**: Velocity target commands
 *             
 *          4. **Controller Execution**:
 *             - Runs appropriate control loops (attitude or position)
 *             - Generates motor/servo outputs
 *             - Maintains vehicle stability during excitation
 *             
 *          5. **Safety Monitoring**:
 *             - Detects landed condition (stops test)
 *             - Monitors lean angle vs maximum (stops if exceeded)
 *             - Validates parameter settings continuously
 *             - Checks for test completion
 *             
 *          6. **Data Logging**:
 *             - Logs SystemID data at high rate (subsampled based on LOG_BITMASK)
 *             - Records waveform state, sensor data, control outputs
 *             - Enables post-flight frequency response analysis
 *             
 *          **Test Signal Types**:
 *          The chirp signal is a logarithmic frequency sweep that excites the system
 *          across a range of frequencies. This allows identification of:
 *          - Frequency response (Bode magnitude and phase)
 *          - Resonances and anti-resonances
 *          - Control bandwidth and phase margins
 *          - Structural modes and flexibility effects
 *          
 *          **Axis Types Explained**:
 *          - INPUT axes: Inject into command path (what pilot would command)
 *          - RECOVER axes: Inject with feedforward disabled (test feedback only)
 *          - RATE axes: Bypass attitude controller, direct rate command
 *          - MIX axes: Bypass all controllers, direct actuator command
 *          - DISTURB axes: External disturbance to position controller
 *          - INPUT_VEL axes: Velocity command inputs
 * 
 * @note Should be called at 100Hz or higher for accurate signal generation and
 *       clean frequency content. Main loop rate of 400Hz is recommended.
 * 
 * @warning This function deliberately injects test signals that cause vehicle motion.
 *          Always monitor vehicle state and be ready to switch to a manual mode.
 *          Ensure adequate clearance from obstacles and terrain.
 * 
 * @warning Some axes (RATE, MIX) bypass safety features in the control system.
 *          Use extreme caution and start with small magnitudes (SID_MAGNITUDE).
 * 
 * @see ModeSystemId::log_data() for data logging implementation
 * @see chirp_input.update() for signal generation
 * @see is_poscontrol_axis_type() for axis classification
 * 
 * Source: ArduCopter/mode_systemid.cpp:162-399
 */
void ModeSystemId::run()
{
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rate_rads = 0.0f;
    float pilot_throttle_scaled = 0.0f;
    float target_climb_rate_cms = 0.0f;
    Vector2f input_vel;

    // For attitude/rate/mixer axes: process pilot input normally
    // Pilot maintains manual control of throttle and nominal attitude
    // Test signals are added to pilot commands
    if (!is_poscontrol_axis_type()) {

        // Apply simple mode transform to pilot inputs if enabled
        // Simple mode makes roll/pitch relative to pilot's initial heading
        update_simple_mode();

        // Convert pilot input to lean angles (roll and pitch commands in radians)
        // Respects maximum lean angle limits for safety
        get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

        // Get pilot's desired yaw rate from rudder stick input
        target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

        if (!motors->armed()) {
            // Motors should be Stopped
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        // Tradheli doesn't set spool state to ground idle when throttle stick is zero.  Ground idle only set when
        // motor interlock is disabled.
        } else if (copter.ap.throttle_zero && !copter.is_tradheli()) {
            // Attempting to Land
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        switch (motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
            // Motors Stopped
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->reset_rate_controller_I_terms();
            break;

        case AP_Motors::SpoolState::GROUND_IDLE:
            // Landed
            // Tradheli initializes targets when going from disarmed to armed state.
            // init_targets_on_arming is always set true for multicopter.
            if (motors->init_targets_on_arming()) {
                attitude_control->reset_yaw_target_and_rate();
                attitude_control->reset_rate_controller_I_terms_smoothly();
            }
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            // clear landing flag above zero throttle
            if (!motors->limit.throttle_lower) {
                set_land_complete(false);
            }
            break;

        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // do nothing
            break;
        }

        // get pilot's desired throttle
#if FRAME_CONFIG == HELI_FRAME
        pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());
#else
        pilot_throttle_scaled = get_pilot_desired_throttle();
#endif

    }

    // Validate test parameters continuously during execution
    // Stop test immediately if invalid configuration detected (protects against in-flight parameter changes)
    if ((systemid_state == SystemIDModeState::SYSTEMID_STATE_TESTING) &&
        (!is_positive(frequency_start) || !is_positive(frequency_stop) || is_negative(time_fade_in) || !is_positive(time_record) || is_negative(time_fade_out) || (time_record <= time_const_freq))) {
        systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID Parameter Error");
    }

    // Update waveform time (elapsed time since mode entry)
    waveform_time += G_Dt;
    
    // Generate test signal: logarithmic chirp sweep from frequency_start to frequency_stop
    // Time is adjusted by SYSTEM_ID_DELAY to allow vehicle stabilization after mode entry
    // Returns amplitude-scaled signal value in appropriate units (degrees, deg/s, or 0-1)
    waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
    
    // Track instantaneous frequency for logging (used in post-processing to generate Bode plots)
    waveform_freq_rads = chirp_input.get_frequency_rads();
    
    // Temporary storage for position/velocity disturbance vectors
    Vector2f disturb_state;
    switch (systemid_state) {
        case SystemIDModeState::SYSTEMID_STATE_STOPPED:
            attitude_control->bf_feedforward(att_bf_feedforward);
            break;
        case SystemIDModeState::SYSTEMID_STATE_TESTING:

            if (copter.ap.land_complete) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            }
            if (attitude_control->lean_angle_rad() > attitude_control->lean_angle_max_rad()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)degrees(attitude_control->lean_angle_rad()), (double)degrees(attitude_control->lean_angle_max_rad()));
                break;
            }
            if (waveform_time > SYSTEM_ID_DELAY + time_fade_in + time_const_freq + time_record + time_fade_out) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Finished");
                break;
            }

            // Inject test signal based on selected axis
            // Each axis type targets a different point in the control system hierarchy
            switch ((AxisType)axis.get()) {
                case AxisType::NONE:
                    // No axis selected - should have been caught in init(), but handle gracefully
                    systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
                    break;
                    
                // INPUT axes: Add angle/rate commands to pilot input (tests full control chain)
                case AxisType::INPUT_ROLL:
                    // Inject roll angle command (waveform in degrees, convert to radians)
                    target_roll_rad += radians(waveform_sample);
                    break;
                case AxisType::INPUT_PITCH:
                    // Inject pitch angle command (waveform in degrees, convert to radians)
                    target_pitch_rad += radians(waveform_sample);
                    break;
                case AxisType::INPUT_YAW:
                    // Inject yaw rate command (waveform in deg/s, convert to rad/s)
                    target_yaw_rate_rads += radians(waveform_sample);
                    break;
                    
                // RECOVER axes: Same as INPUT but with feedforward disabled
                // Tests pure feedback controller response (P, I, D terms only, no model-based feedforward)
                case AxisType::RECOVER_ROLL:
                    target_roll_rad += radians(waveform_sample);
                    attitude_control->bf_feedforward(false);  // Disable feedforward for this iteration
                    break;
                case AxisType::RECOVER_PITCH:
                    target_pitch_rad += radians(waveform_sample);
                    attitude_control->bf_feedforward(false);  // Disable feedforward for this iteration
                    break;
                case AxisType::RECOVER_YAW:
                    target_yaw_rate_rads += radians(waveform_sample);
                    attitude_control->bf_feedforward(false);  // Disable feedforward for this iteration
                    break;
                    
                // RATE axes: Bypass attitude controller, inject directly into rate controller
                // Tests rate loop response (inner loop only, no outer angle-to-rate conversion)
                case AxisType::RATE_ROLL:
                    attitude_control->rate_bf_roll_sysid_rads(radians(waveform_sample));
                    break;
                case AxisType::RATE_PITCH:
                    attitude_control->rate_bf_pitch_sysid_rads(radians(waveform_sample));
                    break;
                case AxisType::RATE_YAW:
                    attitude_control->rate_bf_yaw_sysid_rads(radians(waveform_sample));
                    break;
                    
                // MIX axes: Bypass all controllers, inject directly into motor mixer
                // Tests actuator response and vehicle dynamics without controller influence
                // Waveform is normalized 0-1 for mixer inputs
                case AxisType::MIX_ROLL:
                    attitude_control->actuator_roll_sysid(waveform_sample);
                    break;
                case AxisType::MIX_PITCH:
                    attitude_control->actuator_pitch_sysid(waveform_sample);
                    break;
                case AxisType::MIX_YAW:
                    attitude_control->actuator_yaw_sysid(waveform_sample);
                    break;
                case AxisType::MIX_THROTTLE:
                    // Direct throttle injection (waveform 0-1 added to pilot throttle)
                    pilot_throttle_scaled += waveform_sample;
                    break;
                    
                // DISTURB_POS axes: Position disturbance in body frame (meters)
                // Simulates external position error (e.g., GPS glitch, wind drift)
                case AxisType::DISTURB_POS_LAT:
                    // Lateral (right) disturbance in body frame
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample * 100.0f;  // Convert meters to cm
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);  // Rotate to NED frame
                    pos_control->set_disturb_pos_NE_cm(disturb_state);
                    break;
                case AxisType::DISTURB_POS_LONG:
                    // Longitudinal (forward) disturbance in body frame
                    disturb_state.x = waveform_sample * 100.0f;  // Convert meters to cm
                    disturb_state.y = 0.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);  // Rotate to NED frame
                    pos_control->set_disturb_pos_NE_cm(disturb_state);
                    break;
                    
                // DISTURB_VEL axes: Velocity disturbance in body frame (m/s)
                // Simulates external velocity error or wind gust
                case AxisType::DISTURB_VEL_LAT:
                    // Lateral (right) velocity disturbance in body frame
                    disturb_state.x = 0.0f;
                    disturb_state.y = waveform_sample * 100.0f;  // Convert m/s to cm/s
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);  // Rotate to NED frame
                    pos_control->set_disturb_vel_NE_cms(disturb_state);
                    break;
                case AxisType::DISTURB_VEL_LONG:
                    // Longitudinal (forward) velocity disturbance in body frame
                    disturb_state.x = waveform_sample * 100.0f;  // Convert m/s to cm/s
                    disturb_state.y = 0.0f;
                    disturb_state.rotate(attitude_control->get_att_target_euler_rad().z);  // Rotate to NED frame
                    pos_control->set_disturb_vel_NE_cms(disturb_state);
                    break;
                    
                // INPUT_VEL axes: Velocity command input in body frame (m/s)
                // Tests position controller response to velocity targets
                case AxisType::INPUT_VEL_LAT:
                    // Lateral (right) velocity command in body frame
                    input_vel.x = 0.0f;
                    input_vel.y = waveform_sample * 100.0f;  // Convert m/s to cm/s
                    input_vel.rotate(attitude_control->get_att_target_euler_rad().z);  // Rotate to NED frame
                    break;
                case AxisType::INPUT_VEL_LONG:
                    // Longitudinal (forward) velocity command in body frame
                    input_vel.x = waveform_sample * 100.0f;  // Convert m/s to cm/s
                    input_vel.y = 0.0f;
                    input_vel.rotate(attitude_control->get_att_target_euler_rad().z);  // Rotate to NED frame
                    break;
            }
            break;
    }

    // Execute appropriate control loops based on axis type
    if (!is_poscontrol_axis_type()) {
        // Attitude/Rate/Mixer axes: Use attitude controller with manual throttle
        
        // Call attitude controller with pilot input + injected test signal
        // Converts angle targets to rate targets, then rates to motor outputs
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

        // Output pilot's throttle (with any MIX_THROTTLE test signal added)
        // Pilot maintains manual altitude control during attitude excitation
        attitude_control->set_throttle_out(pilot_throttle_scaled, !copter.is_tradheli(), g.throttle_filt);
        
    } else {
        // Position/Velocity axes: Use position controller with automatic altitude hold
        
        // Relax loiter target if we might be landed
        // Prevents integrator windup if vehicle settles to ground during test
        if (copter.ap.land_complete_maybe) {
            pos_control->soften_for_landing_NE();
        }

        // Integrate velocity input to generate position target
        // For INPUT_VEL axes: target_pos tracks commanded velocity trajectory
        Vector2f accel;
        target_pos += input_vel * G_Dt;
        
        // Calculate acceleration from velocity derivative (for feedforward)
        if (is_positive(G_Dt)) {
            accel = (input_vel - input_vel_last) / G_Dt;
            input_vel_last = input_vel;
        }
        
        // Set position target with velocity and acceleration feedforward
        // Position controller will track this trajectory while rejecting disturbances
        pos_control->set_pos_vel_accel_NE_cm(target_pos.topostype(), input_vel, accel);

        // Run horizontal position controller
        // Generates desired thrust vector to track position/velocity targets
        pos_control->update_NE_controller();

        // Call attitude controller with thrust vector from position controller
        // Converts thrust vector to attitude commands, then to motor outputs
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), target_yaw_rate_rads, false);

        // Maintain altitude at current level (zero climb rate)
        // Position tests focus on horizontal dynamics
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);

        // Run the vertical position controller and set output throttle
        pos_control->update_U_controller();
    }

    // High-rate data logging for system identification
    // Subsampling rate depends on LOG_BITMASK settings to balance data quality vs storage
    if (log_subsample <= 0) {
        log_data();  // Write SystemID data, attitude, rates, PIDs to log
        
        // Set subsample interval based on logging configuration
        // Higher logging rates provide better frequency resolution but consume more storage
        if (copter.should_log(MASK_LOG_ATTITUDE_FAST) && copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;   // Log every loop iteration (400Hz) - best data quality
        } else if (copter.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;   // Log every 2nd iteration (200Hz)
        } else if (copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;   // Log every 4th iteration (100Hz)
        } else {
            log_subsample = 8;   // Log every 8th iteration (50Hz) - minimum recommended rate
        }
    }
    log_subsample -= 1;  // Decrement subsample counter
}

/**
 * @brief Log SystemID test data and vehicle state for post-flight analysis
 * 
 * @details This function logs comprehensive data required for system identification
 *          and frequency response analysis. It is called at high rate (subsampled
 *          based on LOG_BITMASK settings) during SystemID testing.
 *          
 *          Logged Data Includes:
 *          
 *          1. **SystemID-Specific Data** (SIDD message):
 *             - waveform_time: Elapsed time since mode entry (s)
 *             - waveform_sample: Current test signal amplitude (axis-dependent units)
 *             - waveform_freq: Instantaneous frequency of chirp signal (Hz)
 *             - Angular rates: Body-frame roll/pitch/yaw rates (deg/s)
 *             - Accelerations: Body-frame X/Y/Z accelerations (m/s²)
 *             
 *          2. **Attitude Data** (ATT message):
 *             - Desired vs actual roll/pitch/yaw
 *             - Attitude error and error rates
 *             
 *          3. **Rate Controller Data** (RATE message):
 *             - Desired vs actual body rates
 *             - Rate errors for roll/pitch/yaw
 *             
 *          4. **PID Controller Data** (PIDS message):
 *             - P, I, D terms for all rate controllers
 *             - Feedforward terms
 *             - Output saturation status
 *             
 *          5. **Position Controller Data** (for poscontrol axes):
 *             - Position/velocity errors
 *             - North/East velocity PID terms
 *             
 *          This data enables post-flight analysis to:
 *          - Generate Bode magnitude and phase plots
 *          - Identify resonances and anti-resonances
 *          - Measure control bandwidth and phase margin
 *          - Validate mathematical models
 *          - Optimize PID gains
 *          - Detect structural modes and flexibility
 *          
 *          **Post-Processing Workflow**:
 *          1. Extract log data using MAVExplorer or similar tool
 *          2. Correlate waveform_sample (input) with sensor data (output)
 *          3. Compute FFT or use system identification toolbox
 *          4. Generate frequency response (transfer function H(jω))
 *          5. Analyze magnitude/phase characteristics
 *          6. Compare to model predictions or tune controllers
 * 
 * @note High-rate logging (200-400Hz) is essential for accurate frequency response
 *       up to 40-50Hz. Enable ATTITUDE_FAST logging before SystemID tests.
 * 
 * @note Angular rates and accelerations are computed from delta_angle and delta_velocity
 *       to provide instantaneous measurements at the logging rate.
 * 
 * @see Log_Write_SysID_Data() for SystemID data message format
 * @see copter.ins.get_delta_angle() for IMU delta angle measurement
 * @see copter.ins.get_delta_velocity() for IMU delta velocity measurement
 * 
 * Source: ArduCopter/mode_systemid.cpp:402-427
 */
void ModeSystemId::log_data() const
{
    // Get IMU delta angle measurement (integrated gyro over one sample period)
    // Delta angles provide high-resolution angular velocity data
    Vector3f delta_angle;
    float delta_angle_dt;
    copter.ins.get_delta_angle(delta_angle, delta_angle_dt);

    // Get IMU delta velocity measurement (integrated accelerometer over one sample period)
    // Delta velocities provide high-resolution acceleration data
    Vector3f delta_velocity;
    float delta_velocity_dt;
    copter.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    // Compute instantaneous rates and accelerations from delta measurements
    // Log SystemID data: time, input signal, frequency, angular rates (deg/s), accelerations (m/s²)
    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        copter.Log_Write_SysID_Data(
            waveform_time,                                    // Elapsed time since mode entry (s)
            waveform_sample,                                  // Test signal amplitude (axis-dependent units)
            waveform_freq_rads / (2 * M_PI),                 // Instantaneous frequency (Hz)
            degrees(delta_angle.x / delta_angle_dt),         // Roll rate (deg/s)
            degrees(delta_angle.y / delta_angle_dt),         // Pitch rate (deg/s)
            degrees(delta_angle.z / delta_angle_dt),         // Yaw rate (deg/s)
            delta_velocity.x / delta_velocity_dt,            // X acceleration (m/s²)
            delta_velocity.y / delta_velocity_dt,            // Y acceleration (m/s²)
            delta_velocity.z / delta_velocity_dt);           // Z acceleration (m/s²)
    }

    // Full rate logging of attitude, rate and PID loops
    // These messages provide controller state for analyzing closed-loop response
    copter.Log_Write_Attitude();  // Desired vs actual attitude (ATT message)
    copter.Log_Write_Rate();       // Desired vs actual body rates (RATE message)
    copter.Log_Write_PIDS();       // PID terms for rate controllers (PIDS message)

    // For position control axes, log additional position controller data
    if (is_poscontrol_axis_type()) {
        pos_control->write_log();  // Position controller state (PSCN, PSCD messages)
        
        // Log velocity controller PID terms for North and East axes
        copter.logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_NE_pid().get_pid_info_x());  // North velocity PID
        copter.logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_NE_pid().get_pid_info_y());  // East velocity PID
    }
}

/**
 * @brief Check if selected axis requires position controller
 * 
 * @details This helper function classifies the selected axis type to determine
 *          which control loops need to be active during SystemID testing.
 *          
 *          **Position Control Axes (returns true)**:
 *          - DISTURB_POS_LAT (14): Lateral position disturbance
 *          - DISTURB_POS_LONG (15): Longitudinal position disturbance
 *          - DISTURB_VEL_LAT (16): Lateral velocity disturbance
 *          - DISTURB_VEL_LONG (17): Longitudinal velocity disturbance
 *          - INPUT_VEL_LAT (18): Lateral velocity input
 *          - INPUT_VEL_LONG (19): Longitudinal velocity input
 *          
 *          **Attitude/Rate/Mixer Axes (returns false)**:
 *          - INPUT_ROLL/PITCH/YAW (1-3): Attitude angle inputs
 *          - RECOVER_ROLL/PITCH/YAW (4-6): Attitude inputs with feedforward disabled
 *          - RATE_ROLL/PITCH/YAW (7-9): Rate loop inputs
 *          - MIX_ROLL/PITCH/YAW/THROTTLE (10-13): Mixer outputs
 *          
 *          The classification determines:
 *          - Whether position controller needs initialization (init function)
 *          - Whether pilot maintains manual throttle or auto altitude hold (run function)
 *          - Which control loops execute (attitude only vs position+attitude)
 *          - Which data gets logged (position controller PID terms)
 *          - Mode entry requirements (manual throttle vs Loiter mode)
 * 
 * @return true if axis requires position controller (axes 14-19)
 * @return false if axis uses only attitude/rate/mixer control (axes 1-13)
 * 
 * @note This function is const and can be called at any time to query axis type.
 * 
 * @see ModeSystemId::init() for mode entry validation based on axis type
 * @see ModeSystemId::run() for control execution based on axis type
 * 
 * Source: ArduCopter/mode_systemid.cpp:429-447
 */
bool ModeSystemId::is_poscontrol_axis_type() const
{
    bool ret = false;

    // Check if selected axis is one of the position control types
    switch ((AxisType)axis.get()) {
        case AxisType::DISTURB_POS_LAT:    // Axis 14: Lateral position disturbance
        case AxisType::DISTURB_POS_LONG:   // Axis 15: Longitudinal position disturbance
        case AxisType::DISTURB_VEL_LAT:    // Axis 16: Lateral velocity disturbance
        case AxisType::DISTURB_VEL_LONG:   // Axis 17: Longitudinal velocity disturbance
        case AxisType::INPUT_VEL_LAT:      // Axis 18: Lateral velocity input
        case AxisType::INPUT_VEL_LONG:     // Axis 19: Longitudinal velocity input
            ret = true;
            break;
        default:
            // All other axes (0-13) are attitude/rate/mixer types
            break;
        }

    return ret;
}

#endif
