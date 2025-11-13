/**
 * @file mode_althold.cpp
 * @brief AltHold (Altitude Hold) mode implementation for ArduSub
 * 
 * @details This file implements depth holding mode for underwater vehicles using
 *          pressure sensors (barometer) to maintain a constant depth. In ArduSub,
 *          "altitude" refers to depth control since the vehicle operates underwater.
 *          
 *          AltHold mode provides:
 *          - Automatic depth holding using pressure sensor feedback
 *          - Pilot-commanded depth changes via throttle input
 *          - Stabilized attitude control for roll, pitch, and yaw
 *          - Manual forward/lateral thrust control for horizontal translation
 *          - Special handling for surface and bottom detection
 *          
 *          The mode uses the vertical position controller (U axis) integrated with
 *          the inertial navigation system to maintain precise depth control.
 * 
 * @note This mode requires a functioning barometer/pressure sensor. The mode will
 *       fail to initialize if the barometer health check fails.
 * 
 * @note Coordinate System: Uses NED (North-East-Down) convention where depth is
 *       positive down. The inertial nav reports position_z_up (depth positive up),
 *       so conversions are applied throughout this implementation.
 * 
 * Source: ArduSub/mode_althold.cpp
 */

#include "Sub.h"

/**
 * @brief Initialize AltHold mode
 * 
 * @details Initializes the AltHold flight mode by configuring the vertical position
 *          controller and preparing the vehicle for depth holding operations. This
 *          function sets up:
 *          - Maximum vertical speeds (up and down) from pilot parameters
 *          - Vertical acceleration limits
 *          - Position controller initialization with current depth
 *          - Neutral attitude reference (current heading)
 *          
 *          The position controller is initialized to hold the current depth when
 *          the mode is entered, providing a smooth transition from other modes.
 * 
 * @param[in] ignore_checks If true, skip pre-arm checks (currently unused for AltHold)
 * 
 * @return true if initialization successful, false if barometer check fails
 * 
 * @note Barometer health check is mandatory - mode will not initialize without
 *       a functioning pressure sensor as depth control is impossible without it
 * 
 * @note The last_pilot_heading is initialized to current heading to provide smooth
 *       yaw control when pilot releases yaw stick
 */
bool ModeAlthold::init(bool ignore_checks) {
    if(!sub.control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cmss(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_U_controller();

    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

/**
 * @brief Main AltHold mode execution function
 * 
 * @details Executes the complete AltHold control loop combining depth holding with
 *          attitude stabilization and manual thrust control. This function orchestrates
 *          three control phases:
 *          
 *          1. run_pre(): Processes pilot inputs and updates attitude control for
 *             roll, pitch, and yaw stabilization
 *          2. control_depth(): Implements depth holding algorithm using position
 *             controller with pilot throttle input for depth changes
 *          3. run_post(): Applies forward and lateral thrust commands for horizontal
 *             translation while maintaining depth
 *          
 *          Control Flow:
 *          - Pilot throttle input → desired climb rate → position controller
 *          - Pilot roll/pitch input → attitude controller → motor mixing
 *          - Pilot yaw input → heading rate or heading hold
 *          - Pilot forward/lateral → direct thrust output
 *          
 *          The separation into pre/control/post phases allows proper ordering of
 *          control updates and motor output calculations.
 * 
 * @note This function should be called at 100Hz or higher for stable control.
 *       Lower rates may result in oscillations or poor depth holding performance.
 * 
 * @note The control loop integrates with AC_PosControl position controller for
 *       vertical axis and AC_AttitudeControl for roll/pitch/yaw stabilization
 */
void ModeAlthold::run()
{
    run_pre();
    control_depth();
    run_post();
}

/**
 * @brief Pre-processing phase for AltHold mode - handles attitude control
 * 
 * @details Processes pilot inputs for roll, pitch, and yaw control to maintain
 *          vehicle attitude stability while in depth hold mode. This function:
 *          
 *          1. Configures vertical speed and acceleration limits
 *          2. Handles disarmed state (relaxes all controllers)
 *          3. Processes external attitude commands (MAVLink SET_ATTITUDE_TARGET)
 *          4. Processes pilot stick inputs for lean angles (roll/pitch)
 *          5. Implements yaw control with rate control or heading hold
 *          6. Applies intelligent yaw deceleration after rapid maneuvers
 *          
 *          Yaw Control Logic:
 *          - If pilot commanding yaw: Use rate control and update heading reference
 *          - If pilot releases yaw: Decelerate for 250ms then hold last heading
 *          - Deceleration period prevents overshoot from vehicle inertia
 *          
 *          The attitude control integrates with the motor mixer to provide stable
 *          attitude while the position controller maintains depth.
 * 
 * @note When motors are not armed, all controllers are relaxed and throttle is
 *       set to neutral (0.5) to prevent unexpected movement on arming
 * 
 * @note External attitude commands (SET_ATTITUDE_TARGET_NO_GPS) take priority
 *       over manual pilot inputs if received within the last 5 seconds
 * 
 * @note The 250ms yaw deceleration period is tuned for typical underwater vehicle
 *       inertia characteristics and may need adjustment for different vehicles
 */
void ModeAlthold::run_pre()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(0.5,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->relax_U_controller(motors.get_throttle_hover());
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            sub.set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }
}

/**
 * @brief Post-processing phase for AltHold mode - handles horizontal thrust
 * 
 * @details Applies pilot commanded forward and lateral thrust for horizontal
 *          translation while maintaining depth hold. This function processes
 *          pilot stick inputs and passes them directly to the motor library
 *          for mixing with attitude and depth control outputs.
 *          
 *          Forward/Lateral Control:
 *          - Forward channel: Controls fore/aft translation (typically right stick Y)
 *          - Lateral channel: Controls left/right translation (typically right stick X)
 *          - Inputs are normalized (-1.0 to +1.0) before motor mixing
 *          
 *          The motor library combines these translation commands with attitude
 *          control and vertical thrust to produce final motor/thruster outputs
 *          appropriate for the configured frame type (vectored, ROV, etc.).
 * 
 * @note Forward and lateral thrust control is independent of depth holding,
 *       allowing the pilot to maneuver horizontally while the position
 *       controller maintains constant depth
 * 
 * @note The actual motor output depends on the frame configuration and motor
 *       mixing parameters defined in AP_Motors library
 */
void ModeAlthold::run_post()
{
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

/**
 * @brief Core depth holding control algorithm
 * 
 * @details Implements the depth holding algorithm using the position controller
 *          integrated with pilot throttle input for commanded depth changes. This
 *          function performs several critical operations:
 *          
 *          1. Surface Proximity Throttle Limiting:
 *             - Reduces maximum throttle near surface to prevent breaching
 *             - Scales from surface_max_throttle at surface to 1.0 at depth
 *             - Uses linear interpolation based on distance to surface
 *          
 *          2. Pilot Input Processing:
 *             - Converts throttle stick position to desired climb rate (cm/s)
 *             - Constrains to configured maximum ascent/descent speeds
 *             - Applies deadzone for precise depth holding
 *          
 *          3. Surface/Bottom Handling:
 *             - At surface: Limits target depth to prevent breaching
 *             - At bottom: Maintains safe clearance above bottom
 *             - Only active when pilot input is within deadzone
 *          
 *          4. Position Controller Update:
 *             - Converts climb rate to target position
 *             - Runs vertical position controller (U axis)
 *             - Outputs throttle command to motors
 *          
 *          Algorithm Flow:
 *          Pilot Throttle → Climb Rate → Position Target → PID Controller → Motor Output
 *          
 *          The position controller uses inertial navigation feedback (pressure sensor
 *          + IMU fusion) to maintain accurate depth with minimal oscillation.
 * 
 * @note Depth coordinate convention: Uses z_up (depth positive up) from inertial
 *       nav, where more negative values indicate deeper depth
 * 
 * @note Surface detection uses g.surface_depth parameter (typically 5cm below surface)
 *       to prevent oscillation at the air-water interface
 * 
 * @note Bottom detection maintains 10cm clearance to prevent collisions while
 *       allowing close bottom inspection
 * 
 * @note The throttle limiting near surface prevents accidental breaching and
 *       reduces surface disturbance for vehicles with exposed components
 */
void ModeAlthold::control_depth() {
    // Reduce maximum throttle when at the surface to prevent breaching
    // Scale linearly between surface_max_throttle and 1.0 as we approach the surface
    float distance_to_surface = (g.surface_depth - inertial_nav.get_position_z_up_cm()) * 0.01f;
    distance_to_surface = constrain_float(distance_to_surface, 0.0f, 1.0f);
    motors.set_max_throttle(g.surface_max_throttle + (1.0f - g.surface_max_throttle) * distance_to_surface);

    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // desired_climb_rate returns 0 when within the deadzone.
    //we allow full control to the pilot, but as soon as there's no input, we handle being at surface/bottom
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (sub.ap.at_surface) {
            position_control->set_pos_desired_U_cm(MIN(position_control->get_pos_desired_U_cm(), g.surface_depth)); // set target to 5 cm below surface level
        } else if (sub.ap.at_bottom) {
            position_control->set_pos_desired_U_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_desired_U_cm())); // set target to 10 cm above bottom
        }
    }

    position_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cm_s);
    position_control->update_U_controller();
}
