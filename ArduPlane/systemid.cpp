/**
 * @file systemid.cpp
 * @brief System identification implementation for ArduPlane
 * 
 * @details This file implements in-flight system identification functionality for
 *          ArduPlane's quadplane modes. System identification (SysID) is used to 
 *          characterize aircraft dynamic response through controlled excitation signals,
 *          enabling data-driven control loop tuning and frequency response analysis.
 *          
 *          The implementation injects chirp (frequency sweep) signals into the control
 *          path and logs the aircraft response, allowing post-flight analysis to:
 *          - Identify transfer functions and frequency response
 *          - Tune PID controllers based on actual aircraft dynamics
 *          - Verify control loop stability margins
 *          - Validate vehicle models for advanced control design
 *          
 *          System identification can excite various control axes:
 *          - Attitude control (roll/pitch/yaw angles with feedforward)
 *          - Recovery modes (roll/pitch/yaw angles without feedforward)
 *          - Rate control (body frame angular rates in deg/s)
 *          - Motor mixer outputs (direct actuator commands 0-1)
 *          
 *          The chirp waveform sweeps from a start frequency to stop frequency over
 *          a specified time period, with configurable fade-in/fade-out to avoid
 *          discontinuities that could excite unmodeled dynamics or cause instability.
 *          
 *          Safety considerations:
 *          - Only supported in quadplane VTOL modes with position/velocity control
 *          - XY position control authority is reduced during system ID to prevent
 *            large translational movements
 *          - Requires armed state and appropriate flight mode
 *          - Pilot retains ability to terminate via auxiliary switch
 *          - Automatic termination when chirp completes
 *          
 * @note This feature is conditionally compiled with AP_PLANE_SYSTEMID_ENABLED
 * @warning System identification should only be performed by experienced pilots in
 *          safe environments with adequate altitude and clearance from obstacles
 * 
 * @see AP_SystemID library for core chirp generation algorithms
 * @see libraries/AC_AttitudeControl for attitude control integration points
 * @see libraries/AC_PosControl for position control scaling during system ID
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "systemid.h"

#if AP_PLANE_SYSTEMID_ENABLED

#include <AP_Math/control.h>
#include "Plane.h"

/**
 * @brief Parameter table for system identification configuration
 * 
 * @details Defines all configurable parameters for the system identification subsystem.
 *          These parameters control the chirp waveform characteristics and determine
 *          which control axis will be excited during system identification.
 *          
 *          The parameter group uses the existing ArduPilot @Param tag system for
 *          automatic parameter documentation generation and ground station integration.
 *          
 *          Key parameters include:
 *          - Axis selection (which control input to excite)
 *          - Chirp magnitude (amplitude of excitation)
 *          - Frequency sweep range (start and stop frequencies)
 *          - Timing parameters (fade in, recording duration, fade out)
 *          - XY control scaling (safety limit on horizontal movement)
 */
const AP_Param::GroupInfo AP_SystemID::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust
    AP_GROUPINFO_FLAGS("_AXIS", 1, AP_SystemID, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, AP_SystemID, waveform_magnitude, 5),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, AP_SystemID, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, AP_SystemID, frequency_stop, 15),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, AP_SystemID, time_fade_in, 5),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, AP_SystemID, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, AP_SystemID, time_fade_out, 1),

    // @Param: _XY_CTRL_MUL
    // @DisplayName: System identification XY control multiplier
    // @Description: A multiplier for the XY velocity and position controller when using systemID in VTOL modes that do horizontal position and velocity control
    // @Range: 0.05 1.0
    // @User: Standard
    AP_GROUPINFO("_XY_CTRL_MUL", 8, AP_SystemID, xy_control_mul, 0.1),
    
    AP_GROUPEND
};

/**
 * @brief Constructor for AP_SystemID class
 * 
 * @details Initializes the system identification subsystem and sets up default
 *          parameter values using the ArduPilot parameter system. This ensures
 *          all configuration parameters have valid initial values before first use.
 *          
 *          The constructor is called during vehicle initialization before parameters
 *          are loaded from persistent storage. Default values defined in var_info
 *          will be used until user-configured values are loaded.
 */
AP_SystemID::AP_SystemID(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

/**
 * @brief Start system identification sequence
 * 
 * @details Initiates the system identification process by validating prerequisites,
 *          initializing the chirp waveform generator, and configuring the control
 *          system for excitation signal injection.
 *          
 *          The function performs comprehensive safety checks before starting:
 *          - Verifies an axis is selected for excitation (AXIS parameter non-zero)
 *          - Confirms quadplane functionality is enabled
 *          - Validates current flight mode supports system identification
 *          - Ensures vehicle is armed (safety requirement)
 *          
 *          When checks pass, the function:
 *          1. Saves current attitude controller feedforward state for restoration
 *          2. Initializes chirp signal generator with configured parameters
 *          3. Calculates constant frequency period (2 cycles at start frequency)
 *          4. Zeros attitude and throttle offsets
 *          5. Logs system ID configuration to dataflash (SIDS message)
 *          6. Sets running flag to enable update() processing
 *          
 *          The chirp signal sweeps logarithmically from frequency_start to frequency_stop
 *          over time_record seconds, with smooth fade-in and fade-out periods to avoid
 *          exciting unmodeled high-frequency dynamics or causing control discontinuities.
 *          
 * @note Called when pilot activates system ID via auxiliary switch
 * @note Pilot receives GCS text messages indicating success or failure reason
 * @warning Should only be used in controlled test environments by experienced pilots
 * @warning Adequate altitude and clearance from obstacles required
 * 
 * @see stop() for cleanup and restoration of normal control
 * @see update() for per-loop execution during system ID
 */
void AP_SystemID::start()
{
    // Capture current axis selection - locked for duration of system ID run
    start_axis = axis;

    // Safety validation: Check if enabled
    if (start_axis == AxisType::NONE) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: No axis selected");
        return;
    }
    // Safety validation: System ID only supported for quadplane configurations
    if (!plane.quadplane.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: only for quadplane");
        return;
    }
    // Safety validation: Flight mode must support system ID (typically VTOL position/velocity modes)
    if (!plane.control_mode->supports_systemid()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: Not supported in mode %s", plane.control_mode->name());
        return;
    }
    // Safety validation: Must be armed to prevent unexpected behavior on ground
    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "SystemID: must be armed");
        return;
    }

    // Initialize control offsets to zero - no excitation until chirp begins
    attitude_offset_deg.zero();
    throttle_offset = 0;

    // Save current attitude controller configuration for restoration after system ID completes
    restore.att_bf_feedforward = plane.quadplane.attitude_control->get_bf_feedforward();

    // Initialize timing for chirp waveform generation
    waveform_time = 0;
    // Two full cycles at starting frequency allows transients to settle before sweep begins
    time_const_freq = 2.0 / frequency_start;

    // Initialize chirp signal generator with user-configured sweep parameters
    // Chirp sweeps logarithmically from frequency_start to frequency_stop over time_record seconds
    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("SIDS", "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout",
                                "s--ssssss", "F--------", "QBfffffff",
                                AP_HAL::micros64(),
                                uint8_t(start_axis),
                                waveform_magnitude.get(),
                                frequency_start.get(),
                                frequency_stop.get(),
                                time_fade_in.get(),
                                time_const_freq,
                                time_record.get(),
                                time_fade_out.get());
#endif // HAL_LOGGING_ENABLED

    running = true;
}

/**
 * @brief Stop system identification sequence and restore normal control
 * 
 * @details Terminates the system identification process and restores all control
 *          parameters to their pre-system-ID state. This function is called either
 *          when the chirp waveform completes naturally, or when the pilot manually
 *          terminates via auxiliary switch.
 *          
 *          The restoration process:
 *          1. Clears running flag to disable update() processing
 *          2. Zeros all attitude and throttle offsets (removes excitation)
 *          3. Restores attitude controller feedforward setting
 *          4. Clears rate controller system ID injection channels
 *          5. Restores XY position control authority to 100%
 *          6. Re-initializes position controller to current position as target
 *          
 *          The position controller re-initialization prevents the vehicle from
 *          attempting to return to its pre-system-ID position, which could cause
 *          large translational movements if the vehicle drifted during testing.
 *          
 * @note Safe to call multiple times (checks running flag)
 * @note Pilot receives GCS text confirmation when stop completes
 * 
 * @see start() for initialization of system ID sequence
 * @see update() for per-loop execution during system ID
 */
void AP_SystemID::stop()
{
    if (running) {
        // Clear running flag - disables update() processing
        running = false;
        
        // Remove all excitation signals from control path
        attitude_offset_deg.zero();
        throttle_offset = 0;

        // Restore attitude controller to pre-system-ID configuration
        auto *attitude_control = plane.quadplane.attitude_control;
        attitude_control->bf_feedforward(restore.att_bf_feedforward);
        
        // Clear rate controller system ID injection channels (remove rate excitation)
        attitude_control->rate_bf_roll_sysid_rads(0);
        attitude_control->rate_bf_pitch_sysid_rads(0);
        attitude_control->rate_bf_yaw_sysid_rads(0);
        
        // Restore full XY position control authority (was reduced during system ID for safety)
        plane.quadplane.pos_control->set_NE_control_scale_factor(1);

        // Re-initialize XY controller to use current position as target
        // This prevents vehicle from attempting to return to pre-system-ID position
        plane.quadplane.pos_control->init_NE_controller();

        gcs().send_text(MAV_SEVERITY_INFO, "SystemID stopped");
    }
}

/**
 * @brief Update system identification - main per-loop execution function
 * 
 * @details This function must be called at the main loop rate (typically 400Hz for
 *          quadplane VTOL modes) to generate the chirp excitation signal and inject
 *          it into the appropriate control path based on the selected axis.
 *          
 *          The function performs the following operations each loop:
 *          1. Checks if system ID is running (early exit if not)
 *          2. Checks if chirp has completed (calls stop() if finished)
 *          3. Updates chirp waveform time based on actual loop time
 *          4. Generates current chirp sample and instantaneous frequency
 *          5. Injects excitation signal into selected control axis
 *          6. Reduces XY position control authority (safety feature)
 *          7. Logs high-rate data for post-flight analysis
 *          
 *          Axis injection modes:
 *          - INPUT_* modes: Add offset to desired attitude (with feedforward enabled)
 *          - RECOVER_* modes: Add offset to desired attitude (feedforward disabled, tests controller response)
 *          - RATE_* modes: Inject signal directly into rate controller (bypasses angle control)
 *          - MIX_* modes: Inject signal directly into motor mixer (bypasses all control loops)
 *          
 *          The XY position control scaling (xy_control_mul, typically 0.1) prevents
 *          large horizontal movements during system ID by limiting the position controller
 *          authority. This is a safety feature to keep the vehicle near the starting position.
 *          
 *          Logging is performed at a rate determined by the configured attitude logging rate,
 *          with subsampling to match the available logging bandwidth. High-rate delta angle
 *          and delta velocity data from the IMU is logged along with the chirp waveform
 *          parameters for frequency response analysis.
 *          
 * @note Called at main loop rate (typically 400Hz) when system ID is active
 * @note Uses actual loop time for chirp progression to maintain timing accuracy
 * @warning Modifying loop rate or scheduler timing can affect chirp accuracy
 * 
 * @see start() for initialization and safety checks
 * @see stop() for cleanup when chirp completes
 * @see log_data() for detailed logging implementation
 */
void AP_SystemID::update()
{
    // Early exit if system ID not running
    if (!running) {
        return;
    }
    
    // Check if chirp waveform has completed - automatically stop if finished
    if (chirp_input.completed()) {
        stop();
        return;
    }

    // Get actual loop time for accurate chirp progression (handles variable loop rates)
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();

    // Update chirp waveform time and generate current sample
    waveform_time += last_loop_time_s;
    // Generate chirp sample scaled by magnitude parameter (deg, deg/s, or 0-1 depending on axis)
    waveform_sample = chirp_input.update(waveform_time, waveform_magnitude);
    // Get instantaneous frequency for logging (used in frequency response analysis)
    waveform_freq_rads = chirp_input.get_frequency_rads();

    auto *attitude_control = plane.quadplane.attitude_control;

    // Inject excitation signal into selected control axis
    // Different injection points test different aspects of the control system
    switch (start_axis) {
        case AxisType::NONE:
            // Not possible - validated in start()
            break;
            
        // INPUT modes: Add chirp to desired attitude angles (degrees)
        // Feedforward enabled - tests complete attitude control loop response
        case AxisType::INPUT_ROLL:
            attitude_offset_deg.x = waveform_sample;  // Roll angle offset in degrees
            break;
        case AxisType::INPUT_PITCH:
            attitude_offset_deg.y = waveform_sample;  // Pitch angle offset in degrees
            break;
        case AxisType::INPUT_YAW:
            attitude_offset_deg.z = waveform_sample;  // Yaw angle offset in degrees
            break;
            
        // RECOVER modes: Add chirp to desired attitude angles WITHOUT feedforward
        // Tests attitude controller's ability to reject disturbances
        case AxisType::RECOVER_ROLL:
            attitude_offset_deg.x = waveform_sample;
            attitude_control->bf_feedforward(false);  // Disable feedforward for recovery test
            break;
        case AxisType::RECOVER_PITCH:
            attitude_offset_deg.y = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
        case AxisType::RECOVER_YAW:
            attitude_offset_deg.z = waveform_sample;
            attitude_control->bf_feedforward(false);
            break;
            
        // RATE modes: Inject chirp directly into rate controller (deg/s)
        // Bypasses angle control loop - tests rate control loop only
        case AxisType::RATE_ROLL:
            attitude_control->rate_bf_roll_sysid_rads(radians(waveform_sample));
            break;
        case AxisType::RATE_PITCH:
            attitude_control->rate_bf_pitch_sysid_rads(radians(waveform_sample));
            break;
        case AxisType::RATE_YAW:
            attitude_control->rate_bf_yaw_sysid_rads(radians(waveform_sample));
            break;
            
        // MIX modes: Inject chirp directly into motor mixer (normalized 0-1)
        // Bypasses all control loops - identifies plant dynamics only
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
            throttle_offset = waveform_sample;  // Throttle mixer offset (normalized)
            break;
    }

    // Reduce XY position control authority during system ID to limit horizontal drift
    // This safety feature keeps vehicle near starting position during excitation
    plane.quadplane.pos_control->set_NE_control_scale_factor(xy_control_mul);

    // Adaptive logging subsample rate based on configured attitude logging levels
    // Higher logging rates consume more dataflash memory but provide better frequency resolution
    if (log_subsample <= 0) {
        log_data();
        
        // Set subsample interval based on configured logging rate
        // FAST + MED: Log every loop (subsample = 1)
        // FAST only: Log every 2nd loop (subsample = 2) 
        // MED only: Log every 4th loop (subsample = 4)
        // Neither: Log every 8th loop (subsample = 8) - minimum for frequency analysis
        if (plane.should_log(MASK_LOG_ATTITUDE_FAST) && plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (plane.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    // Decrement subsample counter - when reaches 0, log next sample
    log_subsample -= 1;
}

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

/**
 * @brief Log system identification data to dataflash
 * 
 * @details Records high-rate system identification data for post-flight frequency
 *          response analysis. This function logs the SIDD (System ID Data) message
 *          which contains both the excitation signal (chirp waveform) and the vehicle
 *          response (IMU delta angles and velocities).
 *          
 *          Data logged includes:
 *          - Chirp waveform parameters: time, target amplitude, instantaneous frequency
 *          - Vehicle angular response: delta angles (gyro data) in deg/s
 *          - Vehicle linear response: delta velocities (accelerometer data) in m/s
 *          
 *          The delta angle and delta velocity data are retrieved directly from the IMU
 *          at the IMU sample rate, then converted to rates by dividing by the delta time.
 *          This provides the highest fidelity measurement of vehicle response without
 *          aliasing or filtering artifacts.
 *          
 *          Post-flight analysis tools use this logged data to:
 *          1. Compute frequency response (Bode plots) via FFT or spectral analysis
 *          2. Identify transfer functions using system identification algorithms
 *          3. Extract stability margins (gain/phase margins) for controller tuning
 *          4. Validate vehicle models against measured dynamics
 *          
 *          The function also triggers attitude controller logging (Log_Write_AttRate)
 *          to capture control loop internal states synchronized with system ID data.
 *          
 * @note Called at subsampled rate based on configured logging level
 * @note Only logs when valid delta angle/velocity data available (positive dt)
 * @warning High logging rates can fill dataflash quickly - monitor available space
 * 
 * @see update() for subsample rate calculation
 * @see SIDD log message structure for detailed field descriptions
 */
void AP_SystemID::log_data() const
{
#if HAL_LOGGING_ENABLED
    // Get delta angle (integrated gyro) since last IMU sample
    // Provides highest fidelity angular rate measurement without filtering
    Vector3f delta_angle;
    float delta_angle_dt;
    plane.ins.get_delta_angle(delta_angle, delta_angle_dt);

    // Get delta velocity (integrated accelerometer) since last IMU sample
    // Provides highest fidelity acceleration measurement without filtering
    Vector3f delta_velocity;
    float delta_velocity_dt;
    plane.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    // Only log if valid IMU data available (positive delta times)
    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        // Pre-compute inverse delta times for efficiency
        const float dt_ang_inv = 1.0 / delta_angle_dt;
        const float dt_vel_inv = 1.0 / delta_velocity_dt;
        
        // Write SIDD message with chirp parameters and vehicle response
        // Angular rates converted from radians to degrees for standard logging units
        // Frequency converted from rad/s to Hz for readability
        AP::logger().WriteStreaming("SIDD", "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az",
                                    "ss-zkkkooo", "F---------", "Qfffffffff",
                                    AP_HAL::micros64(),
                                    waveform_time,                           // Chirp elapsed time (s)
                                    waveform_sample,                         // Chirp amplitude
                                    waveform_freq_rads / (2 * M_PI),        // Instantaneous frequency (Hz)
                                    degrees(delta_angle.x * dt_ang_inv),    // Roll rate (deg/s)
                                    degrees(delta_angle.y * dt_ang_inv),    // Pitch rate (deg/s)
                                    degrees(delta_angle.z * dt_ang_inv),    // Yaw rate (deg/s)
                                    delta_velocity.x * dt_vel_inv,          // X acceleration (m/s²)
                                    delta_velocity.y * dt_vel_inv,          // Y acceleration (m/s²)
                                    delta_velocity.z * dt_vel_inv);         // Z acceleration (m/s²)

        // Log attitude controller state synchronized with system ID data
        // Provides controller internals (rate targets, errors, outputs) for detailed analysis
        plane.quadplane.Log_Write_AttRate();
    }
#endif // HAL_LOGGING_ENABLED
}

/**
 * @note System identification functionality is conditionally compiled with
 *       AP_PLANE_SYSTEMID_ENABLED feature flag. This allows the feature to be
 *       disabled on resource-constrained boards or when not needed, reducing
 *       flash memory and RAM usage.
 */
#endif // AP_PLANE_SYSTEMID_ENABLED

