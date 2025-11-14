/**
 * @file mode_flowhold.cpp
 * @brief Implementation of FlowHold flight mode for ArduCopter
 * 
 * @details FlowHold is a GPS-free position hold mode that uses optical flow sensors
 *          for maintaining position relative to the ground. This mode enables indoor
 *          flight and operation in GPS-denied environments by utilizing visual odometry
 *          from a downward-facing camera or optical flow sensor.
 * 
 *          Key characteristics:
 *          - Provides position hold without GPS or external positioning
 *          - Maintains position relative to ground using visual flow measurements
 *          - Requires downward-facing optical flow sensor (e.g., PX4Flow, Cheerson CX-OF)
 *          - Benefits from rangefinder for improved height estimation
 *          - Useful for indoor flight, under bridges, or other GPS-denied areas
 *          - Limited to low altitudes (typically under 10m) where optical flow is reliable
 *          - Susceptible to failure over low-texture surfaces (uniform carpet, water, etc.)
 *          - Flow quality must exceed FHLD_QUAL_MIN parameter for position hold to activate
 * 
 *          Implementation uses PI controller to convert flow measurements to attitude
 *          targets, with braking behavior when pilot releases sticks. Height estimation
 *          integrates accelerometer data with optical flow for improved altitude hold.
 * 
 * @note This mode requires AP_OPTICALFLOW to be enabled and a functioning flow sensor
 * @warning Vehicle will behave like AltHold if flow quality drops below threshold
 * @warning Not suitable for outdoor flight where GPS is available - use Loiter instead
 * 
 * @see libraries/AP_OpticalFlow/
 * @see mode.h for Mode base class
 */

#include "Copter.h"
#include <utility>

#if MODE_FLOWHOLD_ENABLED

const AP_Param::GroupInfo ModeFlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: FlowHold filter on input to control
    // @Description: FlowHold (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeFlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: FlowHold Flow Rate Max
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: FlowHold Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, ModeFlowHold, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: FlowHold Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 4, ModeFlowHold, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    // @Param: _BRAKE_RATE
    // @DisplayName: FlowHold Braking rate
    // @Description: Controls deceleration rate on stick release
    // @Range: 1 30
    // @User: Standard
    // @Units: deg/s
    AP_GROUPINFO("_BRAKE_RATE", 6, ModeFlowHold, brake_rate_dps, 8),

    AP_GROUPEND
};

/**
 * @brief Constructor for FlowHold mode
 * 
 * @details Initializes FlowHold mode object and sets up parameter defaults.
 *          Parameters include PI controller gains, flow rate limits, filter
 *          frequencies, quality thresholds, and braking rates.
 */
ModeFlowHold::ModeFlowHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define CONTROL_FLOWHOLD_EARTH_FRAME 0

/**
 * @brief Initialize FlowHold mode
 * 
 * @details Performs initialization of the FlowHold flight mode controller, including:
 *          - Validates optical flow sensor is enabled and healthy
 *          - Configures vertical speed and acceleration limits for altitude control
 *          - Initializes vertical position controller if not already active
 *          - Sets up flow rate low-pass filter with configured cutoff frequency
 *          - Resets PI controller integrator terms for clean mode entry
 *          - Initializes height estimation using INS (Inertial Navigation System)
 * 
 *          The optical flow sensor must be both enabled and healthy for mode entry.
 *          If the sensor is unavailable, initialization fails and the mode change
 *          will be rejected, preventing the vehicle from entering an unsafe state.
 * 
 * @param[in] ignore_checks If true, skip pre-flight checks (currently unused for FlowHold)
 * 
 * @return true if initialization successful and mode can be entered
 * @return false if optical flow sensor is disabled or unhealthy
 * 
 * @note Called automatically by mode switching logic when pilot selects FlowHold
 * @note Position hold will not activate until flow quality exceeds FHLD_QUAL_MIN parameter
 * @note First 3 seconds after arming, mode behaves like AltHold regardless of flow quality
 * 
 * @warning Initialization failure indicates optical flow sensor is not available
 * @warning Do not force mode entry if init() returns false - vehicle cannot hold position
 * 
 * @see run() for main mode execution
 * @see copter.optflow for optical flow sensor interface
 */
bool ModeFlowHold::init(bool ignore_checks)
{
    // FlowHold requires functioning optical flow sensor - cannot hold position without it
    // Reject mode entry if sensor disabled or currently unhealthy (no recent valid measurements)
    if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
        return false;
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!copter.pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());

    quality_filtered = 0;
    flow_pi_xy.reset_I();
    limited = false;

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    // start with INS height
    last_ins_height_m = pos_control->get_pos_estimate_NEU_cm().z * 0.01;
    height_offset_m = 0;

    return true;
}

/**
 * @brief Calculate desired attitude angles from optical flow measurements
 * 
 * @details Converts optical flow sensor measurements into target attitude angles for
 *          position hold. This is the core algorithm of FlowHold mode that maintains
 *          the vehicle's position relative to the ground using visual odometry.
 * 
 *          Algorithm steps:
 *          1. Obtain raw flow rate and compensate for vehicle rotation (body rate)
 *          2. Limit sensor flow to prevent oscillation at low altitudes
 *          3. Apply low-pass filter to smooth flow measurements
 *          4. Scale by height estimate to convert flow (rad/s) to velocity (m/s)
 *          5. Rotate measurements to earth frame for consistent control
 *          6. Run PI controller to generate attitude correction
 *          7. Implement braking behavior when pilot releases sticks
 *          8. Convert earth frame output back to body frame attitude targets
 * 
 *          The function operates in three distinct states:
 *          - Active stick input: Pilot has direct attitude control, PI controller paused
 *          - Braking: 3-second deceleration phase after stick release
 *          - Position hold: PI controller actively maintains position
 * 
 *          Braking algorithm calculates lean angles based on current velocity to
 *          smoothly decelerate the vehicle. PI integrator is only updated during
 *          position hold state to prevent wind-up during pilot control.
 * 
 * @param[in,out] bf_angles_cd Body frame attitude angles in centidegrees. Input contains
 *                             pilot stick angles, output adds flow correction angles
 * @param[in] stick_input True if pilot is actively commanding attitude (non-zero roll/pitch)
 * 
 * @note Called at main loop rate (typically 400Hz) when flow sensor is healthy
 * @note Flow measurements are scaled by height estimate - accuracy depends on altitude
 * @note Anti-windup protection limits integrator when attitude angles saturate
 * @note Braking phase lasts 3 seconds or until velocity drops below 0.3 m/s
 * 
 * @warning Low-texture surfaces (uniform carpet, calm water) may cause flow sensor failure
 * @warning Flow rate limited to ±FHLD_FLOW_MAX to prevent instability at low altitudes
 * @warning Height estimate accuracy critical - poor altitude estimation causes position drift
 * 
 * @see run() for mode state machine and calling context
 * @see update_height_estimate() for height estimation using flow and accelerometers
 * @see flow_pi_xy PI controller configuration (FHLD_XY_P, FHLD_XY_I parameters)
 */
void ModeFlowHold::flowhold_flow_to_angle(Vector2f &bf_angles_cd, bool stick_input)
{
    uint32_t now = AP_HAL::millis();

    // get corrected raw flow rate
    Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);

    // filter the flow rate
    Vector2f sensor_flow = flow_filter.apply(raw_flow);

    // Scale flow rate by height estimate to convert angular rate (rad/s) to linear velocity (m/s)
    // This is critical: flow_rate × height = velocity
    // Height limited between height_min_m and height_max to prevent extreme scaling
    // INS height from position controller plus adaptive offset from update_height_estimate()
    float ins_height_m = pos_control->get_pos_estimate_NEU_cm().z * 0.01;
    float height_estimate_m = ins_height_m + height_offset_m;

    // Compensate for height, this converts flow (rad/s) to approximate velocity (m/s)
    sensor_flow *= constrain_float(height_estimate_m, height_min_m, height_max);

    // rotate controller input to earth frame
    Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);

    // run PI controller
    flow_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = flow_pi_xy.get_p();

    if (stick_input) {
        last_stick_input_ms = now;
        braking = true;
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now - last_stick_input_ms > 3000 || sensor_flow.length() < 0.3) {
            braking = false;
#if 0
            printf("braking done at %u vel=%f\n", now - last_stick_input_ms,
                   (double)sensor_flow.length());
#endif
        }
    }

    if (!stick_input && !braking) {
        // get I term
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_pi();
        }
    }

    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        for (uint8_t i=0; i<2; i++) {
            float &velocity = sensor_flow[i];
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * brake_rate_dps.get() + 95.0f) * 0.01f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles_cd[i] = lean_angle_cd;
        }
        ef_output.zero();
    }

    ef_output += xy_I;
    ef_output *= copter.aparm.angle_max;

    // convert to body frame
    bf_angles_cd += copter.ahrs.earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles_cd.x) > copter.aparm.angle_max || fabsf(bf_angles_cd.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles_cd.x = constrain_float(bf_angles_cd.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles_cd.y = constrain_float(bf_angles_cd.y, -copter.aparm.angle_max, copter.aparm.angle_max);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHLD
// @Description: FlowHold mode messages
// @URL: https://ardupilot.org/copter/docs/flowhold-mode.html
// @Field: TimeUS: Time since system startup
// @Field: SFx: Filtered flow rate, X-Axis
// @Field: SFy: Filtered flow rate, Y-Axis
// @Field: Ax: Target lean angle, X-Axis
// @Field: Ay: Target lean angle, Y-Axis
// @Field: Qual: Flow sensor quality. If this value falls below FHLD_QUAL_MIN parameter, FlowHold will act just like AltHold.
// @Field: Ix: Integral part of PI controller, X-Axis
// @Field: Iy: Integral part of PI controller, Y-Axis

    if (log_counter++ % 20 == 0) {
        AP::logger().WriteStreaming("FHLD", "TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy", "Qfffffff",
                                               AP_HAL::micros64(),
                                               (double)sensor_flow.x, (double)sensor_flow.y,
                                               (double)bf_angles_cd.x, (double)bf_angles_cd.y,
                                               (double)quality_filtered,
                                               (double)xy_I.x, (double)xy_I.y);
    }
#endif  // HAL_LOGGING_ENABLED
}

/**
 * @brief Main execution loop for FlowHold mode
 * 
 * @details Implements the complete FlowHold mode control loop, called at main loop rate
 *          (typically 400Hz but must be at least 100Hz). Manages altitude control,
 *          horizontal position hold using optical flow, and coordinates all subsystems.
 * 
 *          Execution sequence each loop:
 *          1. Update height estimate using optical flow and accelerometer integration
 *          2. Configure vertical speed and acceleration limits
 *          3. Apply SIMPLE mode transformation to pilot inputs (if enabled)
 *          4. Update flow filter cutoff frequency if parameter changed
 *          5. Process pilot climb rate and yaw rate commands
 *          6. Determine altitude hold state (landed, takeoff, flying, etc.)
 *          7. Filter optical flow quality measurement
 *          8. Execute state machine for motor control and altitude hold
 *          9. Calculate attitude targets from pilot input
 *          10. Add flow-based position hold correction (if quality sufficient)
 *          11. Apply avoidance adjustments (if enabled)
 *          12. Command attitude controller and update throttle
 * 
 *          State machine behavior:
 *          - MotorStopped: Disarmed or emergency stop - shutdown motors, reset controllers
 *          - Takeoff: Execute automated takeoff to PILOT_TKOFF_ALT
 *          - Landed_Ground_Idle/Landed_Pre_Takeoff: On ground, prepare for flight
 *          - Flying: Normal flight - full altitude and position control active
 * 
 *          Position hold activation requirements:
 *          - Flow quality ≥ FHLD_QUAL_MIN parameter
 *          - At least 3 seconds elapsed since arming (avoid takeoff transients)
 *          - Optical flow sensor healthy
 * 
 *          If position hold requirements not met, mode behaves like AltHold (altitude
 *          hold only, pilot controls horizontal position directly).
 * 
 * @note Must be called at ≥100Hz for stable control, typically runs at 400Hz
 * @note Flow correction limited to ±50% of maximum angle to preserve pilot authority
 * @note Surface tracking updates vertical offset if rangefinder available
 * @note Logged as FHLD message for position hold performance analysis
 * 
 * @warning Insufficient loop rate (<100Hz) may cause instability
 * @warning Low flow quality causes fallback to manual horizontal control (AltHold behavior)
 * @warning First 3 seconds after arming, no position hold regardless of flow quality
 * 
 * @see init() for mode initialization
 * @see flowhold_flow_to_angle() for position hold algorithm
 * @see update_height_estimate() for altitude estimation
 * @see get_alt_hold_state() for state machine determination
 */
void ModeFlowHold::run()
{
    update_height_estimate();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // check for filter change
    if (!is_equal(flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());
    }

    // get pilot desired climb rate
    float target_climb_rate_cms = copter.get_pilot_desired_climb_rate();
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), copter.g.pilot_speed_up);

    // get pilot's desired yaw rate
    float target_yaw_rate_cds = rad_to_cd(get_pilot_desired_yaw_rate_rads());

    // Flow Hold State Machine Determination
    AltHoldModeState flowhold_state = get_alt_hold_state(target_climb_rate_cms);

    // Apply heavy filtering to flow quality to prevent mode oscillation on quality threshold
    // Filter constant 0.95 provides slow response, preventing rapid enable/disable of position hold
    // Quality drops to 0 immediately if sensor becomes unhealthy
    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    // Flow Hold State Machine
    switch (flowhold_state) {

    case AltHoldModeState::MotorStopped:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->reset_yaw_target_and_rate();
        copter.pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        flow_pi_xy.reset_I();
        break;

    case AltHoldModeState::Takeoff:
        // set motors to full range
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate_cms);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // flowhold attitude target calculations
    Vector2f bf_angles_cd;

    // calculate alt-hold angles
    int16_t roll_in = copter.channel_roll->get_control_in();
    int16_t pitch_in = copter.channel_pitch->get_control_in();
    float angle_max_cd = copter.aparm.angle_max;

    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());
    bf_angles_cd.x = rad_to_cd(target_roll_rad);
    bf_angles_cd.y = rad_to_cd(target_pitch_rad);

    // Enable optical flow position hold only when quality sufficient and vehicle stabilized
    // - quality_filtered must exceed FHLD_QUAL_MIN parameter (default 10)
    // - 3 second delay after arming prevents flow from interfering with takeoff transients
    // If conditions not met, mode behaves like AltHold (altitude hold only, pilot controls horizontal)
    if (quality_filtered >= flow_min_quality &&
        AP_HAL::millis() - copter.arm_time_ms > 3000) {
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max_cd/2, angle_max_cd/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max_cd/2, angle_max_cd/2);
        bf_angles_cd += flow_angles;
    }
    bf_angles_cd.x = constrain_float(bf_angles_cd.x, -angle_max_cd, angle_max_cd);
    bf_angles_cd.y = constrain_float(bf_angles_cd.y, -angle_max_cd, angle_max_cd);

#if AP_AVOIDANCE_ENABLED
    // apply avoidance
    Vector2f bf_angles_rad = bf_angles_cd * cd_to_rad(1.0);
    copter.avoid.adjust_roll_pitch_rad(bf_angles_rad.x, bf_angles_rad.y, attitude_control->lean_angle_max_rad());
    bf_angles_cd = bf_angles_rad * rad_to_cd(1.0);
#endif

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(bf_angles_cd.x, bf_angles_cd.y, target_yaw_rate_cds);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();
}

/**
 * @brief Update height estimate using optical flow and accelerometer fusion
 * 
 * @details Implements a complementary height estimation algorithm that fuses inertial
 *          measurements with optical flow data to improve altitude accuracy. This is
 *          critical because optical flow measurements must be scaled by height to
 *          convert from angular rates (rad/s) to linear velocities (m/s).
 * 
 *          Algorithm principle:
 *          The relationship between optical flow rate and vehicle velocity is:
 *          velocity = flow_rate × height
 *          
 *          By integrating accelerometer measurements to get velocity change (delta_velocity)
 *          and comparing with flow rate changes (delta_flow), we can estimate height:
 *          height = delta_velocity / delta_flow
 * 
 *          Implementation steps:
 *          1. Get inertial height estimate from position controller (INS)
 *          2. Handle ground case: Reset height offset when disarmed or spooling up
 *          3. Integrate delta velocity from IMU in earth frame
 *          4. Convert integrated velocity to body frame to match flow sensor frame
 *          5. Get optical flow rate measurements (compensated for body rotation)
 *          6. Calculate instantaneous height from velocity/flow ratio for each axis
 *          7. Weight height estimates by flow magnitude (stronger flow = more reliable)
 *          8. Apply filtering and rate limiting to prevent noise-induced jumps
 *          9. Constrain height estimate to be above minimum altitude (height_min_m)
 *          10. Update height_offset_m which is added to INS height estimate
 * 
 *          The algorithm uses both X and Y flow axes independently, weighting each
 *          estimate by the flow magnitude. This provides redundancy and improved
 *          accuracy when the vehicle is moving in different directions.
 * 
 *          Noise rejection strategies:
 *          - Require minimum delta_velocity (0.04 m/s) and minimum delta_flow (0.04 rad/s)
 *          - Discard negative height estimates (physically impossible)
 *          - Limit height change per update to ±0.25m (height_delta_max_m)
 *          - Apply low-pass filter (0.8 * old + 0.2 * new)
 *          - Bias toward lower heights to prevent oscillation (2× gain for decreases)
 *          - Ignore flow updates more than 500ms apart (flow sensor timeout)
 * 
 * @note Called at main loop rate from run()
 * @note Height estimate critical for scaling flow measurements to velocities
 * @note Inaccurate height causes proportional error in velocity estimation
 * @note Logged as FHXY message for detailed analysis and tuning
 * 
 * @warning Requires healthy optical flow sensor - no update without flow data
 * @warning Poor flow quality or low-texture surfaces degrade height estimate accuracy
 * @warning Height estimation most accurate when vehicle is moving (provides flow variation)
 * @warning Accelerometer bias causes height estimation drift over time
 * 
 * @see run() for calling context
 * @see flowhold_flow_to_angle() for flow measurement usage
 * @see copter.ins.get_delta_velocity() for IMU integration
 * @see copter.optflow for optical flow sensor interface
 */
void ModeFlowHold::update_height_estimate(void)
{
    float ins_height_m = copter.pos_control->get_pos_estimate_NEU_cm().z * 0.01;

#if 1
    // assume on ground when disarmed, or if we have only just started spooling the motors up
    if (!hal.util->get_soft_armed() ||
        copter.motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED ||
        AP_HAL::millis() - copter.arm_time_ms < 1500) {
        height_offset_m = -ins_height_m;
        last_ins_height_m = ins_height_m;
        return;
    }
#endif

    // get delta velocity in body frame
    Vector3f delta_vel_ms;
    float delta_vel_dt;
    if (!copter.ins.get_delta_velocity(delta_vel_ms, delta_vel_dt)) {
        return;
    }

    // integrate delta velocity in earth frame
    const Matrix3f &rotMat = copter.ahrs.get_rotation_body_to_ned();
    delta_vel_ms = rotMat * delta_vel_ms;
    delta_velocity_ne_ms.x += delta_vel_ms.x;
    delta_velocity_ne_ms.y += delta_vel_ms.y;

    if (!copter.optflow.healthy()) {
        // can't update height model with no flow sensor
        last_flow_ms = AP_HAL::millis();
        delta_velocity_ne_ms.zero();
        return;
    }

    if (last_flow_ms == 0) {
        // just starting up
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne_ms.zero();
        height_offset_m = 0;
        return;
    }

    if (copter.optflow.last_update() == last_flow_ms) {
        // no new flow data
        return;
    }

    // convert delta velocity back to body frame to match the flow sensor
    Vector2f delta_vel_bf_ms = copter.ahrs.earth_to_body2D(delta_velocity_ne_ms);

    // and convert to an rate equivalent, to be comparable to flow
    Vector2f delta_vel_rate_ms(-delta_vel_bf_ms.y, delta_vel_bf_ms.x);

    // get body flow rate in radians per second
    Vector2f flow_rate_rads = copter.optflow.flowRate() - copter.optflow.bodyRate();

    uint32_t dt_ms = copter.optflow.last_update() - last_flow_ms;
    if (dt_ms > 500) {
        // too long between updates, ignore
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne_ms.zero();
        last_flow_rate_rads = flow_rate_rads;
        last_ins_height_m = ins_height_m;
        height_offset_m = 0;
        return;        
    }

    /*
      basic equation is:
      height_m = delta_velocity_ms / delta_flowrate_rads;
     */

    // get delta_flowrate_rps
    Vector2f delta_flowrate_rads = flow_rate_rads - last_flow_rate_rads;
    last_flow_rate_rads = flow_rate_rads;
    last_flow_ms = copter.optflow.last_update();

    /*
      update height estimate
     */
    const float min_velocity_change_ms = 0.04;
    const float min_flow_change = 0.04;
    const float height_delta_max_m = 0.25;

    /*
      for each axis update the height estimate
     */
    float delta_height_m = 0;
    uint8_t total_weight = 0;
    float height_estimate_m = ins_height_m + height_offset_m;

    for (uint8_t i=0; i<2; i++) {
        // only use height estimates when we have significant delta-velocity and significant delta-flow
        float abs_flow = fabsf(delta_flowrate_rads[i]);
        if (abs_flow < min_flow_change ||
            fabsf(delta_vel_rate_ms[i]) < min_velocity_change_ms) {
            continue;
        }
        // get instantaneous height estimate
        float height_m = delta_vel_rate_ms[i] / delta_flowrate_rads[i];
        if (height_m <= 0) {
            // discard negative heights
            continue;
        }
        delta_height_m += (height_m - height_estimate_m) * abs_flow;
        total_weight += abs_flow;
    }
    if (total_weight > 0) {
        delta_height_m /= total_weight;
    }

    if (delta_height_m < 0) {
        // bias towards lower heights, as we'd rather have too low
        // gain than have oscillation. This also compensates a bit for
        // the discard of negative heights above
        delta_height_m *= 2;
    }

    // don't update height by more than height_delta_max_m, this is a simple way of rejecting noise
    float new_offset_m = height_offset_m + constrain_float(delta_height_m, -height_delta_max_m, height_delta_max_m);

    // apply a simple filter
    height_offset_m = 0.8 * height_offset_m + 0.2 * new_offset_m;

    if (ins_height_m + height_offset_m < height_min_m) {
        // height estimate is never allowed below the minimum
        height_offset_m = height_min_m - ins_height_m;
    }

    // new height estimate for logging
    height_estimate_m = ins_height_m + height_offset_m;

#if HAL_LOGGING_ENABLED
// @LoggerMessage: FHXY
// @Description: Height estimation using optical flow sensor 
// @Field: TimeUS: Time since system startup
// @Field: DFx: Delta flow rate, X-Axis
// @Field: DFy: Delta flow rate, Y-Axis
// @Field: DVx: Integrated delta velocity rate, X-Axis
// @Field: DVy: Integrated delta velocity rate, Y-Axis
// @Field: Hest: Estimated Height
// @Field: DH: Delta Height
// @Field: Hofs: Height offset
// @Field: InsH: Height estimate from inertial navigation library
// @Field: LastInsH: Last used INS height in optical flow sensor height estimation calculations 
// @Field: DTms: Time between optical flow sensor updates. This should be less than 500ms for performing the height estimation calculations

    AP::logger().WriteStreaming("FHXY", "TimeUS,DFx,DFy,DVx,DVy,Hest,DH,Hofs,InsH,LastInsH,DTms", "QfffffffffI",
                                           AP_HAL::micros64(),
                                           (double)delta_flowrate_rads.x,
                                           (double)delta_flowrate_rads.y,
                                           (double)delta_vel_rate_ms.x,
                                           (double)delta_vel_rate_ms.y,
                                           (double)height_estimate_m,
                                           (double)delta_height_m,
                                           (double)height_offset_m,
                                           (double)ins_height_m,
                                           (double)last_ins_height_m,
                                           dt_ms);
#endif

    gcs().send_named_float("HEST", height_estimate_m);
    delta_velocity_ne_ms.zero();
    last_ins_height_m = ins_height_m;
}

#endif // MODE_FLOWHOLD_ENABLED
