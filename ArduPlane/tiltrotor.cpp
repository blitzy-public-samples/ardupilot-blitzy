/**
 * @file tiltrotor.cpp
 * @brief Tilt-rotor and tilt-wing mechanism control for ArduPlane quadplanes
 * 
 * @details This file implements the Tiltrotor class which controls the tilting mechanism
 *          for quadplane aircraft that can transition between multicopter (VTOL) and fixed-wing
 *          flight modes. The tilt mechanism allows motors to rotate from vertical (hover) to
 *          horizontal (forward flight) orientations.
 * 
 *          Supported Tilt Types:
 *          - CONTINUOUS: Smooth tilt to any angle with rate limiting
 *          - BINARY: Two-position tilt (fully up or fully forward) 
 *          - VECTORED_YAW: Uses differential tilt for yaw control in hover
 *          - BICOPTER: Specialized control for bicopter tiltrotors
 * 
 *          Key Functionality:
 *          - Tilt angle scheduling based on flight mode and airspeed
 *          - Slew rate limiting for smooth transitions
 *          - Thrust compensation to account for motor tilt angle
 *          - Vectored thrust control for yaw authority
 *          - Servo output generation for tilt mechanisms
 *          - Integration with quadplane transition logic
 * 
 *          The tilt angle is represented as a value from 0.0 (fully vertical/VTOL) to 1.0
 *          (fully horizontal/forward flight), corresponding to physical angles from 0 to 90 degrees.
 * 
 *          Safety Considerations:
 *          - Tilt rate limiting prevents sudden motor angle changes that could destabilize aircraft
 *          - Thrust compensation maintains control authority during tilt transitions
 *          - Failsafe handling ensures safe motor positions when disarmed or in emergencies
 * 
 * @note All tilt functionality is conditionally compiled with HAL_QUADPLANE_ENABLED
 * @warning Improper tilt configuration can lead to loss of control during transitions
 * 
 * @see Tiltrotor class in tiltrotor.h for interface definitions
 * @see QuadPlane class for overall quadplane control architecture
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: ArduPlane/tiltrotor.cpp
 */

#include "tiltrotor.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED
const AP_Param::GroupInfo Tiltrotor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tiltrotor functionality
    // @Values: 0:Disable, 1:Enable
    // @Description: This enables Tiltrotor functionality
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0),

    // @Param: RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40),

    // @Param: MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45),

    // @Param: TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover, Bicopter tiltrotor must use the tailsitter frame class (10)
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw,3:Bicopter
    AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS),

    // @Param: RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0),

    // @Param: YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output (fully back). This needs to be set in addition to Q_TILT_TYPE=2, to enable vectored control for yaw in tilt quadplanes. This is also used to limit the forward travel of bicopter tilts(Q_TILT_TYPE=3) when in VTOL modes.
    // @Range: 0 30
    AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0),

    // @Param: FIX_ANGLE
    // @DisplayName: Fixed wing tiltrotor angle
    // @Description: This is the angle the motors tilt down when at maximum output for forward flight. Set this to a non-zero value to enable vectoring for roll/pitch in forward flight on tilt-vectored aircraft
    // @Units: deg
    // @Range: 0 30
    // @User: Standard
    AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0),

    // @Param: FIX_GAIN
    // @DisplayName: Fixed wing tiltrotor gain
    // @Description: This is the gain for use of tilting motors in fixed wing flight for tilt vectored quadplanes
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0),

    // @Param: WING_FLAP
    // @DisplayName: Tiltrotor tilt angle that will be used as flap
    // @Description: For use on tilt wings, the wing will tilt up to this angle for flap, transition will be complete when the wing reaches this angle from the forward fight position, 0 disables
    // @Units: deg
    // @Increment: 1
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0),

    AP_GROUPEND
};

/*
  control code for tiltrotors and tiltwings. Enabled by setting
  Q_TILT_MASK to a non-zero value
 */

/**
 * @brief Construct a Tiltrotor controller instance
 * 
 * @details Initializes the tiltrotor controller and establishes references to the parent
 *          quadplane and motor controller objects. Sets up default parameter values from
 *          the var_info parameter table.
 * 
 * @param[in] _quadplane Reference to parent QuadPlane object for accessing flight state
 * @param[in] _motors Reference to multicopter motor controller for thrust management
 * 
 * @note This constructor is called during QuadPlane initialization
 * @note Parameter defaults are loaded from var_info[] array
 */
Tiltrotor::Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Initialize and configure the tiltrotor system
 * 
 * @details Performs comprehensive tiltrotor setup including:
 *          - Auto-enabling tiltrotor if mask or bicopter type is configured
 *          - Detecting vectored yaw configuration
 *          - Identifying fixed forward motors vs tilting motors
 *          - Detecting permanent VTOL motors (non-tilting)
 *          - Configuring motor yaw torque settings for vectored types
 *          - Setting up thrust compensation callbacks
 *          - Initializing servo output ranges for tilt servos
 *          - Creating transition state machine object
 * 
 *          Motor Configuration Detection:
 *          - Fixed forward motors: k_throttle, k_throttleLeft, k_throttleRight
 *          - Tilting motors: Identified by tilt_mask bitmask
 *          - VTOL motors: Enabled motors not in tilt_mask
 * 
 * @note Must be called after motor and servo channel initialization
 * @note Sets quadplane.thrust_type to TILTROTOR when enabled
 * @note Allocates Tiltrotor_Transition object on heap
 * 
 * @warning Aborts with allocation_error if transition object allocation fails
 */
void Tiltrotor::setup()
{
    // Auto-enable tiltrotor functionality if tilt_mask or bicopter type is configured
    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    // Set quadplane thrust type to enable tiltrotor-specific control paths
    quadplane.thrust_type = QuadPlane::ThrustType::TILTROTOR;

    // Detect vectored yaw configuration (uses differential tilt for yaw control)
    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;

    // Determine if there are fixed forward motors (non-tilting motors for forward flight)
    // true if a fixed forward motor is configured, either throttle, throttle left  or throttle right.
    // bicopter tiltrotors use throttle left and right as tilting motors, so they don't count in that case.
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
                    ((SRV_Channels::function_assigned(SRV_Channel::k_throttleLeft) || SRV_Channels::function_assigned(SRV_Channel::k_throttleRight))
                        && (type != TILT_TYPE_BICOPTER));

    // Check if there are any permanent VTOL motors (motors that don't tilt)
    // These provide pure vertical thrust regardless of tilt state
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && !is_motor_tilting(i)) {
            // enabled motor not set in tilt mask
            _have_vtol_motor = true;
            break;
        }
    }

    if (_is_vectored) {
        // Disable motor yaw torque since we use differential tilt for yaw control
        motors->disable_yaw_torque();
    }

    if (tilt_mask != 0) {
        // Register thrust compensation callback to adjust motor outputs based on tilt angle
        // This maintains control authority as motors transition between vertical and horizontal
        motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
        if (type == TILT_TYPE_VECTORED_YAW) {
            // Configure tilt servo output ranges for vectored yaw control
            // Range of 1000 corresponds to 0-1000 PWM output range for tilt positioning
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRear,  1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearLeft, 1000);
            SRV_Channels::set_range(SRV_Channel::k_tiltMotorRearRight, 1000);
        }
    }

    // Allocate transition state machine object for managing VTOL <-> fixed-wing transitions
    transition = NEW_NOTHROW Tiltrotor_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tiltrotor transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
}

/**
 * @brief Calculate maximum tilt change per loop iteration
 * 
 * @details Computes the maximum tilt angle change allowed in this control loop based on
 *          configured tilt rates and current flight conditions. The return value is a
 *          proportion from 0.0 to 1.0 representing the fraction of the full 90-degree
 *          tilt range that can change this loop.
 * 
 *          Tilt Rate Selection:
 *          - Upward tilt (VTOL->FW): Uses Q_TILT_RATE_UP parameter
 *          - Downward tilt (FW->VTOL): Uses Q_TILT_RATE_DN if configured, else Q_TILT_RATE_UP
 * 
 *          Fast Tilt Mode:
 *          For continuous tilt types moving downward (not in flap range), enforces minimum
 *          90 deg/s rate in manual mode or non-stabilized forward flight to allow rapid
 *          pilot control response.
 * 
 * @param[in] up True if tilting toward VTOL (upward), false if tilting toward FW (forward)
 * @param[in] in_flap_range True if current tilt angle is in the flap deployment range
 * 
 * @return Maximum tilt change as proportion (0.0 to 1.0 corresponds to 0 to 90 degrees)
 * 
 * @note Calculation: (rate_deg_per_sec * loop_time_sec) / 90_degrees
 * @note Units: Input rates in deg/s, output in proportion per loop iteration
 */
float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const
{
    float rate;
    // Select appropriate tilt rate based on direction
    if (up || max_rate_down_dps <= 0) {
        rate = max_rate_up_dps;
    } else {
        rate = max_rate_down_dps;
    }
    // Apply fast tilt logic for continuous types when tilting forward (down)
    if (type != TILT_TYPE_BINARY && !up && !in_flap_range) {
        bool fast_tilt = false;
        if (plane.control_mode == &plane.mode_manual) {
            fast_tilt = true;
        }
        if (plane.arming.is_armed_and_safety_off() && !quadplane.in_vtol_mode() && !quadplane.assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            // allow a minimum of 90 DPS in manual or if we are not
            // stabilising, to give fast control
            rate = MAX(rate, 90);
        }
    }
    // Convert deg/s rate to proportion per loop: (deg/s * dt) / 90deg = proportion
    return rate * plane.G_Dt * (1/90.0);
}

/**
 * @brief Output a slew rate-limited tilt angle to tilt servos
 * 
 * @details Applies slew rate limiting to prevent sudden motor angle changes that could
 *          destabilize the aircraft. Constrains the change in tilt angle to the maximum
 *          allowed based on configured rates and current flight conditions.
 * 
 *          Algorithm:
 *          1. Calculate maximum tilt change for this loop iteration
 *          2. Constrain new tilt to be within max_change of current tilt
 *          3. Update angle_achieved flag indicating if target reached
 *          4. Output servo command scaled to 0-1000 range
 * 
 * @param[in] newtilt Desired tilt angle as proportion (0.0=vertical VTOL, 1.0=horizontal FW)
 * 
 * @note Updates current_tilt member variable with rate-limited value
 * @note Sets angle_achieved flag true when target angle is reached
 * @note Outputs to SRV_Channel::k_motor_tilt servo function
 * 
 * @warning Bypassing slew limiting can cause dangerous aircraft instability
 */
void Tiltrotor::slew(float newtilt)
{
    // Calculate maximum allowed tilt change based on direction and flight mode
    float max_change = tilt_max_change(newtilt<current_tilt, newtilt > get_fully_forward_tilt());
    // Constrain new tilt to current_tilt +/- max_change
    current_tilt = constrain_float(newtilt, current_tilt-max_change, current_tilt+max_change);

    // Set flag indicating whether we've reached the target angle
    angle_achieved = is_equal(newtilt, current_tilt);

    // translate to 0..1000 range and output to tilt servo
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}

/**
 * @brief Get the tilt value representing fully forward flight configuration
 * 
 * @details For tilt-wing aircraft, forward flight can be sustained with some wing tilt
 *          used as flap. This returns the tilt value (0-1) that represents the fully
 *          forward position accounting for flap angle configuration.
 * 
 * @return Tilt proportion (0.0=vertical, 1.0=fully horizontal without flap)
 * 
 * @note When flap_angle_deg=0, returns 1.0 (fully horizontal)
 * @note When flap_angle_deg>0, returns value less than 1.0 to allow flap deployment
 */
float Tiltrotor::get_fully_forward_tilt() const
{
    // Convert flap angle (degrees) to proportion and subtract from 1.0
    return 1.0 - (flap_angle_deg * (1/90.0));
}

/**
 * @brief Get the target tilt value for current forward flight conditions
 * 
 * @details Calculates target tilt angle for forward flight considering current flap
 *          deployment. When flaps are deployed, this reduces the forward tilt to allow
 *          the tilted motors/wings to function as flaps for increased lift.
 * 
 * @return Tilt proportion accounting for current flap position (0.0=vertical, 1.0=horizontal)
 * 
 * @note Scales with current flap servo position (k_flap_auto)
 * @note Used during forward flight to coordinate tilt with flap deployment
 */
float Tiltrotor::get_forward_flight_tilt() const
{
    // Calculate tilt position considering flap deployment (0-100%)
    return 1.0 - ((flap_angle_deg * (1/90.0)) * SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto) * 0.01);
}

/**
 * @brief Update motor tilt angle for continuous tilt servo systems
 * 
 * @details Implements tilt angle scheduling for continuous tilt-rotor systems based on
 *          current flight mode, demanded throttle, and transition state. Handles multiple
 *          control strategies:
 * 
 *          1. Fixed-Wing Mode (non-VTOL, non-assisted):
 *             - Tilts motors fully forward (or VTOL if disarmed with DISARMED_TILT_UP option)
 *             - Uses tilting motors as forward thrust motors
 *             - Slew-limits throttle changes during transition
 * 
 *          2. QAUTOTUNE Mode:
 *             - Maintains zero tilt for pure multicopter control
 * 
 *          3. Forward Throttle Control (Q_FWD_THR_GAIN in VTOL modes):
 *             - Tilts motors based on calculated forward throttle demand
 *             - Angle limited by Q_TILT_MAX parameter
 * 
 *          4. Manual Forward Throttle (QACRO/QSTABILIZE/QHOVER):
 *             - Manual RC control of tilt via forward throttle channel
 *             - Zero tilt if no RC forward throttle configured
 * 
 *          5. Transition Mode:
 *             - Tilts fully forward when transitioning to fixed-wing
 *             - Throttle-based tilt scheduling during transition
 * 
 * @note Called at main loop rate (typically 50-400 Hz depending on vehicle)
 * @note Sets _motors_active flag to prevent motor shutdown in forward flight
 * @note Applies slew rate limiting to both tilt angle and throttle changes
 * 
 * @warning Incorrect tilt scheduling can cause loss of control during transitions
 */
void Tiltrotor::continuous_update(void)
{
    // default to inactive (motors may be shut down if not actively controlled)
    _motors_active = false;

    // the maximum rate of throttle change per loop iteration
    float max_change;

    // FIXED-WING MODE: Pure forward flight without VTOL or assisted flight
    if (!quadplane.in_vtol_mode() && (!plane.arming.is_armed_and_safety_off() || !quadplane.assisted_flight)) {
        // we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
        // a forward motor

        // Safety feature: If disarmed and option enabled, tilt to VTOL position to prevent propeller
        // ground strikes. Exception: allow forward tilt in manual mode for pre-flight testing
        const bool disarmed_tilt_up = !plane.arming.is_armed_and_safety_off() && (plane.control_mode != &plane.mode_manual) && quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT_UP);
        slew(disarmed_tilt_up ? 0.0 : get_forward_flight_tilt());

        // Calculate maximum throttle change rate for smooth transitions
        max_change = tilt_max_change(false);

        // Get demanded forward throttle (0.0 to 1.0)
        float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
        if (current_tilt < get_fully_forward_tilt()) {
            // Still transitioning to forward flight - slew limit throttle to prevent sudden thrust changes
            current_throttle = constrain_float(new_throttle,
                                                    current_throttle-max_change,
                                                    current_throttle+max_change);
        } else {
            // Fully forward - apply throttle directly without slew limiting
            current_throttle = new_throttle;
        }
        if (!plane.arming.is_armed_and_safety_off()) {
            // Disarmed - zero throttle for safety
            current_throttle = 0;
        } else {
            // Armed - prevent motor library from shutting down motors
            _motors_active = true;
        }
        if (!quadplane.motor_test.running) {
            // Output throttle to tilting motors for forward thrust
            // Mask determines which motors receive throttle (only tilting motors when non-zero)
            const uint32_t mask = is_zero(current_throttle) ? 0U : tilt_mask.get();
            motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
        }
        return;
    }

    // VTOL MODE: Multicopter flight with potential forward thrust component
    // Track the throttle level being used for VTOL flight (for slew rate limiting)
    float motors_throttle = motors->get_throttle();
    max_change = tilt_max_change(motors_throttle<current_throttle);
    current_throttle = constrain_float(motors_throttle,
                                            current_throttle-max_change,
                                            current_throttle+max_change);

    /*
      we are in a VTOL mode. We need to work out how much tilt is
      needed. There are 5 strategies we will use:

      1) With use of a forward throttle controlled by Q_FWD_THR_GAIN in
         VTOL modes except Q_AUTOTUNE determined by Q_FWD_THR_USE. We set the angle based on a calculated
         forward throttle.

      2) With manual forward throttle control we set the angle based on the
         RC input demanded forward throttle for QACRO, QSTABILIZE and QHOVER.

      3) Without a RC input or calculated forward throttle value, the angle
         will be set to zero in QAUTOTUNE, QACRO, QSTABILIZE and QHOVER.
         This enables these modes to be used as a safe recovery mode.

      4) In fixed wing assisted flight or velocity controlled modes we will
         set the angle based on the demanded forward throttle, with a maximum
         tilt given by Q_TILT_MAX. This relies on Q_FWD_THR_GAIN or Q_VFWD_GAIN
         being set.

      5) if we are in TRANSITION_TIMER mode then we are transitioning
         to forward flight and should put the rotors all the way forward
    */

#if QAUTOTUNE_ENABLED
    // STRATEGY 3: QAUTOTUNE mode - maintain zero tilt for pure multicopter control
    // This allows accurate PID tuning without tilt-related dynamics
    if (plane.control_mode == &plane.mode_qautotune) {
        slew(0);
        return;
    }
#endif

    // STRATEGY 1: Forward throttle control via Q_FWD_THR_GAIN in VTOL modes
    // Uses calculated forward throttle demand to set tilt angle for forward thrust
    if (!quadplane.assisted_flight &&
        quadplane.get_vfwd_method() == QuadPlane::ActiveFwdThr::NEW &&
        quadplane.is_flying_vtol())
    {
        // We are using the rotor tilt functionality controlled by Q_FWD_THR_GAIN which can
        // operate in all VTOL modes except Q_AUTOTUNE. Forward rotor tilt is used to produce
        // forward thrust equivalent to what would have been produced by a forward thrust motor
        // set to quadplane.forward_throttle_pct()
        
        // Calculate tilt angle from forward thrust demand (as fraction of 1g acceleration)
        const float fwd_g_demand = 0.01 * quadplane.forward_throttle_pct();
        // Convert thrust demand to tilt angle: angle = atan(forward_thrust / vertical_thrust)
        const float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), (float)max_angle_deg);
        slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
        return;
    }
    // STRATEGY 2 & 3: Manual modes (QACRO/QSTABILIZE/QHOVER)
    else if (!quadplane.assisted_flight &&
               (plane.control_mode == &plane.mode_qacro ||
               plane.control_mode == &plane.mode_qstabilize ||
               plane.control_mode == &plane.mode_qhover))
    {
        if (quadplane.rc_fwd_thr_ch == nullptr) {
            // STRATEGY 3: No manual throttle control configured - zero tilt for safe recovery mode
            slew(0);
        } else {
            // STRATEGY 2: Manual RC control of forward throttle and tilt angle
            // Pilot controls tilt via forward throttle stick, limited by Q_TILT_MAX
            float settilt = 0.01f * quadplane.forward_throttle_pct();
            slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
        }
        return;
    }

    // STRATEGY 5: Active transition to fixed-wing flight
    if (quadplane.assisted_flight &&
        transition->transition_state >= Tiltrotor_Transition::TRANSITION_TIMER) {
        // we are transitioning to fixed wing - tilt the motors all
        // the way forward
        slew(get_forward_flight_tilt());
    } else {
        // STRATEGY 4: Velocity controlled modes or early transition phase
        // Tilt angle based on demanded forward throttle, limited by Q_TILT_MAX
        // until we have completed the transition we limit the tilt to
        // Q_TILT_MAX. Anything above 50% throttle gets
        // Q_TILT_MAX. Below 50% throttle we decrease linearly. This
        // relies heavily on Q_VFWD_GAIN being set appropriately.
       float settilt = constrain_float((SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)-MAX(plane.aparm.throttle_min.get(),0)) * 0.02, 0, 1);
       slew(MIN(settilt * max_angle_deg * (1/90.0), get_forward_flight_tilt())); 
    }
}


/**
 * @brief Output binary tilt position with internal rate limiting
 * 
 * @details For binary (two-position) tilt systems, the servo output is commanded to
 *          either fully forward (1000) or fully up (0) positions. However, the internal
 *          current_tilt state is rate-limited to track the estimated servo position.
 *          This rate limiting delays throttle application in binary_update() until
 *          the servo has physically reached the commanded position.
 * 
 *          Binary tilt assumes a retract-style servo that moves between two endpoints
 *          without intermediate positioning capability.
 * 
 * @param[in] forward True to command forward (horizontal) position, false for up (vertical)
 * 
 * @note Servo output is NOT rate limited (immediate 0 or 1000 command)
 * @note current_tilt IS rate limited to estimate actual servo position
 * @note Rate limiting prevents applying throttle before servo reaches position
 */
void Tiltrotor::binary_slew(bool forward)
{
    // Command servo to binary position (no slew limiting on servo output)
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

    // rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
    // This tracks estimated servo position to prevent throttle before servo reaches target
    float max_change = tilt_max_change(!forward);
    if (forward) {
        current_tilt = constrain_float(current_tilt+max_change, 0, 1);
    } else {
        current_tilt = constrain_float(current_tilt-max_change, 0, 1);
    }
}

/**
 * @brief Update motor tilt for binary (two-position) tilt servo systems
 * 
 * @details Implements simplified tilt control for binary tilt servos that only support
 *          two positions: fully up (VTOL) or fully forward (FW). Commands tilt position
 *          based on flight mode and applies forward throttle only when servos have reached
 *          the forward position.
 * 
 *          Control Logic:
 *          - VTOL mode: Command servos to up position (vertical thrust)
 *          - Fixed-wing mode: Command servos to forward position
 *          - Forward throttle applied only when current_tilt >= 1.0 (fully forward)
 * 
 * @note Motors marked as always active to prevent shutdown
 * @note Throttle application delayed until tilt completes (via current_tilt tracking)
 * @note Simpler than continuous_update() - only two tilt states
 */
void Tiltrotor::binary_update(void)
{
    // motors always active for binary tilt systems
    _motors_active = true;

    if (!quadplane.in_vtol_mode()) {
        // Fixed-wing mode: tilt motors all the way forward
        // we are in pure fixed wing mode. Move the tiltable motors
        // all the way forward and run them as a forward motor
        binary_slew(true);

        // Get demanded forward throttle
        float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
        if (current_tilt >= 1) {
            // Servos have reached forward position - safe to apply throttle
            const uint32_t mask = is_zero(new_throttle) ? 0 : tilt_mask.get();
            // the motors are all the way forward, start using them for fwd thrust
            motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
        }
    } else {
        // VTOL mode: tilt motors to vertical position
        binary_slew(false);
    }
}


/**
 * @brief Main update function for tiltrotor control (called each loop)
 * 
 * @details Entry point for tiltrotor control system, called at main loop rate.
 *          Dispatches to appropriate update function based on tilt type and
 *          applies vectored thrust control if configured.
 * 
 *          Update Sequence:
 *          1. Check if tiltrotor enabled and configured
 *          2. Call binary_update() or continuous_update() based on type
 *          3. Apply vectoring() if TILT_TYPE_VECTORED_YAW configured
 * 
 * @note Called from QuadPlane::update() at main loop rate
 * @note No-op if enable=0 or tilt_mask=0 (no tilting motors)
 */
void Tiltrotor::update(void)
{
    if (!enabled() || tilt_mask == 0) {
        // no motors to tilt - tiltrotor not configured
        return;
    }

    // Dispatch to appropriate tilt control based on configured type
    if (type == TILT_TYPE_BINARY) {
        binary_update();
    } else {
        continuous_update();
    }

    // Apply vectored thrust control for yaw/roll if configured
    if (type == TILT_TYPE_VECTORED_YAW) {
        vectoring();
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Write tiltrotor telemetry data to dataflash log
 * 
 * @details Logs current tilt angles for post-flight analysis and debugging.
 *          Log message type: TILT
 * 
 *          Logged Parameters:
 *          - time_us: Timestamp in microseconds
 *          - current_tilt: Overall tilt angle in degrees (0=vertical, 90=horizontal)
 *          - front_left_tilt: Left motor tilt angle (vectored yaw only)
 *          - front_right_tilt: Right motor tilt angle (vectored yaw only)
 * 
 *          For non-vectored types, left/right tilt angles are set to NaN since
 *          individual motor tilt angles are not independently controlled.
 * 
 * @note Only logs when tiltrotor is enabled
 * @note Called from QuadPlane logging at regular intervals
 */
void Tiltrotor::write_log()
{
    // Only valid on a tiltrotor
    if (!enabled()) {
        return;
    }

    // Initialize log packet with current tilt angle
    struct log_tiltrotor pkt {
        LOG_PACKET_HEADER_INIT(LOG_TILT_MSG),
        time_us      : AP_HAL::micros64(),
        current_tilt : current_tilt * 90.0,  // Convert proportion to degrees
    };

    if (type != TILT_TYPE_VECTORED_YAW) {
        // Left and right tilt are invalid for non-vectored types
        pkt.front_left_tilt = plane.logger.quiet_nanf();
        pkt.front_right_tilt = plane.logger.quiet_nanf();

    } else {
        // Calculate individual motor tilt angles from servo outputs for vectored yaw
        // total_angle = vertical(90) + yaw_range(tilt_yaw_angle) + fixed_angle
        const float total_angle = 90.0 + tilt_yaw_angle + fixed_angle;
        const float scale = total_angle * 0.001;  // Scale 0-1000 servo output to angle
        pkt.front_left_tilt = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft) * scale) - tilt_yaw_angle;
        pkt.front_right_tilt = (SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * scale) - tilt_yaw_angle;
    }

    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif

/**
 * @brief Apply thrust compensation for tilted motors based on tilt angle
 * 
 * @details When rotors are tilted, the effectiveness of thrust for roll/yaw control changes:
 *          - Roll authority decreases as motors tilt forward (thrust vector more horizontal)
 *          - Yaw authority increases as differential thrust becomes more effective
 * 
 *          Compensation Strategy:
 *          1) Transitioning to forward flight: Scale tilted rotors by 1/cos(angle)
 *             to maintain thrust while increasing forward component
 *          2) Transitioning to hover: Scale non-tilted rotors by cos(angle) 
 *             to reduce forward thrust component
 *          3) Roll equalization: Average tilted motor thrust in proportion to tilt
 *             to smoothly reduce roll control impact as tilt increases
 *          4) Yaw compensation: Add differential thrust based on yaw demand and sin(tilt)
 *          5) Saturation handling: Scale all motors if any exceed maximum thrust
 * 
 * @param[in,out] thrust        Array of motor thrust values (0.0 to 1.0+) to be modified
 * @param[in]     num_motors    Number of motors in thrust array
 * @param[in]     non_tilted_mul Multiplier for non-tilting motors (typically cos(angle))
 * @param[in]     tilted_mul     Multiplier for tilting motors (typically 1.0 or 1/cos(angle))
 * 
 * @note Modifies thrust array in-place
 * @note Uses current_tilt (0.0-1.0) for compensation calculations
 * @warning Thrust saturation is critical to prevent motor over-speed
 * 
 * @see tilt_compensate() for entry point that determines compensation direction
 */
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul)
{
    float tilt_total = 0;
    uint8_t tilt_count = 0;
    
    // Step 1: Apply basic thrust scaling factors to all motors
    // Non-tilting motors scaled down when transitioning to hover
    // Tilting motors scaled up when transitioning to forward flight
    for (uint8_t i=0; i<num_motors; i++) {
        if (!is_motor_tilting(i)) {
            thrust[i] *= non_tilted_mul;
        } else {
            thrust[i] *= tilted_mul;
            tilt_total += thrust[i];
            tilt_count++;
        }
    }

    float largest_tilted = 0;
    const float sin_tilt = sinf(radians(current_tilt*90));
    
    // yaw_gain relates the amount of differential thrust we get from
    // tilt, so that the scaling of the yaw control is the same at any
    // tilt angle (normalizes for vectored yaw range)
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    const float avg_tilt_thrust = tilt_total / tilt_count;

    // Step 2: Apply roll equalization and yaw compensation to tilting motors
    for (uint8_t i=0; i<num_motors; i++) {
        if (is_motor_tilting(i)) {
            // Roll equalization: As we tilt forward, blend toward average thrust
            // to reduce roll control authority. At current_tilt=0 (vertical), full
            // individual control. At current_tilt=1 (horizontal), all motors averaged.
            // This prevents excessive roll response when tilted forward.
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
            
            // Yaw compensation: Add differential thrust for yaw control
            // Scaled by sin(tilt_angle) since yaw effectiveness increases with tilt
            // get_roll_factor() provides the motor's position for differential thrust
            const float diff_thrust = motors->get_roll_factor(i) * (motors->get_yaw()+motors->get_yaw_ff()) * sin_tilt * yaw_gain;
            thrust[i] += diff_thrust;
            
            // Track maximum thrust for saturation handling
            largest_tilted = MAX(largest_tilted, thrust[i]);
        }
    }

    // Step 3: Handle thrust saturation to prevent motor over-speed
    // If any motor exceeds 1.0, scale all motors proportionally to keep
    // the largest at exactly 1.0. This maintains control ratios while
    // respecting physical limits.
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i=0; i<num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}

/**
 * @brief Entry point for tilt compensation - selects compensation strategy based on flight mode
 * 
 * @details Chooses appropriate thrust compensation based on whether we are transitioning
 *          to VTOL or fixed-wing mode:
 * 
 *          VTOL Mode (transitioning to hover):
 *          - Scale non-tilted motors by cos(angle) to reduce forward thrust
 *          - Keep tilted motors at normal thrust (factor=1.0)
 *          - Pushes vehicle toward hover with reduced forward velocity
 * 
 *          Fixed-Wing Mode (transitioning to forward flight):
 *          - Keep non-tilted motors at normal thrust (factor=1.0)
 *          - Scale tilted motors by 1/cos(angle) to increase effective thrust
 *          - Pushes vehicle toward forward flight with increased airspeed
 * 
 * @param[in,out] thrust      Array of motor thrust values to be compensated (0.0 to 1.0+)
 * @param[in]     num_motors  Number of motors in thrust array
 * 
 * @note No-op if current_tilt <= 0 (motors fully vertical)
 * @note Called via callback registered in setup() - executed by motors library
 * @note Limits tilt_factor calculation to 0.98 to avoid division by near-zero
 * 
 * @see tilt_compensate_angle() for detailed compensation algorithm
 */
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (current_tilt <= 0) {
        // the motors are not tilted, no compensation needed
        return;
    }
    
    if (quadplane.in_vtol_mode()) {
        // Transitioning to VTOL flight - reduce non-tilted motor thrust
        // This helps slow down and transition to hover
        const float tilt_factor = cosf(radians(current_tilt*90));
        tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
    } else {
        // Transitioning to fixed-wing flight - increase tilted motor thrust
        // This helps accelerate and transition to forward flight
        float inv_tilt_factor;
        if (current_tilt > 0.98f) {
            // Limit to avoid division by near-zero (cos(88.2°) = 0.0314)
            inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
        } else {
            inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
        }
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}

/**
 * @brief Check if rotors are fully tilted forward for fixed-wing flight
 * 
 * @details Returns true when motors have reached their maximum forward tilt position.
 *          For tilt-wings with flap functionality, this accounts for the flap angle
 *          which allows partial tilt while still being "fully forward" for flight.
 * 
 * @return true if current_tilt >= fully forward position
 * @return false if tiltrotor disabled, not configured, or not fully forward
 * 
 * @note Used to determine when to engage forward flight throttle control
 * @see get_fully_forward_tilt() for flap-adjusted forward tilt value
 */
bool Tiltrotor::fully_fwd(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt >= get_fully_forward_tilt());
}

/**
 * @brief Check if rotors are fully tilted up for VTOL flight
 * 
 * @details Returns true when motors are at their fully vertical position (0° tilt).
 *          This is the position used for pure multicopter hover flight.
 * 
 * @return true if current_tilt <= 0 (fully vertical)
 * @return false if tiltrotor disabled, not configured, or tilted forward
 * 
 * @note Used to determine when full multicopter control is available
 */
bool Tiltrotor::fully_up(void) const
{
    if (!enabled() || (tilt_mask == 0)) {
        return false;
    }
    return (current_tilt <= 0);
}

/**
 * @brief Control vectored thrust for differential tilt (TILT_TYPE_VECTORED_YAW)
 * 
 * @details Implements independent left/right motor tilt control for yaw authority
 *          in hover and roll/pitch control in forward flight. The tilt servos can
 *          move differentially to create yaw moments or symmetrically for pitch.
 * 
 *          Control Modes:
 *          1. VTOL (hover): Differential tilt for yaw control
 *             - Left/right tilt varies based on yaw demand
 *             - Roll demand also contributes when tilted forward
 *             - Throttle-based scaling maintains authority
 * 
 *          2. Fixed-Wing: Symmetric tilt for pitch/roll control  
 *             - Tilt acts like canards (front) or elevons (rear)
 *             - Scaled by Q_TILT_FIX_GAIN parameter
 *             - Accounts for elevon mixing
 * 
 *          3. Disarmed Testing: Manual rudder control of tilt
 *             - Allows ground testing of tilt range
 *             - Only active 3 seconds after disarm
 * 
 *          Tilt Range:
 *          - zero_out: Tilt position for vertical motors (includes yaw_angle)
 *          - level_out: Tilt position for level forward flight  
 *          - fixed_tilt_limit: Additional tilt for vectored control in FW
 * 
 * @note Only called when type == TILT_TYPE_VECTORED_YAW
 * @note Outputs to k_tiltMotorLeft, k_tiltMotorRight, k_tiltMotorRear channels
 * @warning Saturation limiting applied to prevent exceeding servo ranges
 * 
 * @see motors->get_yaw() for yaw demand
 * @see motors->get_roll() for roll demand used in tilt scaling
 */
void Tiltrotor::vectoring(void)
{
    // Calculate tilt range parameters
    // total_angle = full mechanical range: vertical(90°) + yaw_range + pitch_range
    const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
    
    // zero_out = servo output (0-1) to point motors straight up (90°)
    // Accounts for yaw_angle which allows tilt back past vertical for yaw control
    const float zero_out = tilt_yaw_angle / total_angle;
    
    // fixed_tilt_limit = additional tilt range for vectored pitch control in FW
    const float fixed_tilt_limit = fixed_angle / total_angle;
    
    // level_out = servo output for level forward flight (before vectoring)
    const float level_out = 1.0 - fixed_tilt_limit;

    // Calculate base tilt position before applying vectored control
    // Maps current_tilt (0-1) to servo output range (zero_out to level_out)
    float base_output = zero_out + (current_tilt * (level_out - zero_out));
    
    // Disarmed testing mode: Allow manual control of tilt servos via rudder/control sticks
    // This enables ground testing of tilt range and servo calibration
    // Wait TILT_DELAY_MS after disarming to allow propellers to spin down first for safety
    constexpr uint32_t TILT_DELAY_MS = 3000;
    uint32_t now = AP_HAL::millis();
    if (!plane.arming.is_armed_and_safety_off() && plane.quadplane.option_is_set(QuadPlane::OPTION::DISARMED_TILT)) {
        // this test is subject to wrapping at ~49 days, but the consequences are insignificant
        if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
            if (quadplane.in_vtol_mode()) {
                // VTOL disarmed testing: Rudder stick controls differential tilt for yaw
                float yaw_out = plane.channel_rudder->get_control_in();
                yaw_out /= plane.channel_rudder->get_range();  // Normalize to -1.0 to 1.0
                float yaw_range = zero_out;

                // Apply differential tilt: left increases with positive rudder, right decreases
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,  1000 * constrain_float(base_output + yaw_out * yaw_range,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, 1000 * constrain_float(base_output - yaw_out * yaw_range,0,1));
            } else {
                // Fixed-wing disarmed testing: Control sticks control vectored pitch/roll
                const float gain = fixed_gain * fixed_tilt_limit;
                
                // Base tilt on elevon mixing for roll/pitch, which accounts for MIXING_GAIN
                // Rear tilt based on elevator only (pitch), front tilts from elevon mix (roll+pitch)
                const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
                const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
                const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
                
                // Front tilt acts like canards (reversed sign), rear tilt acts like elevons
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
                SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
            }
        }
        return;
    }

    // Check if tilted beyond max VTOL angle - if so, use fixed-wing vectoring logic
    const bool no_yaw = tilt_over_max_angle();
    if (no_yaw) {
        // Fixed-wing vectored thrust mode (tilted past Q_TILT_MAX)
        // Use tilt servos for pitch/roll control like canards and elevons
        // Apply inverse throttle scaling and remove airspeed scaling
        // since we want constant control authority regardless of airspeed
        const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
        const float gain = fixed_gain * fixed_tilt_limit * scaler;
        
        // Calculate tilt offsets from control surface demands
        // Scale from servo output (-4500 to 4500) to tilt proportion
        const float right = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right) * (1/4500.0);
        const float left  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left) * (1/4500.0);
        const float mid  = gain * SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (1/4500.0);
        
        // Front motors: negative (canard-like effect), Rear motors: positive (elevon-like effect)
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,1000 * constrain_float(base_output - right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight,1000 * constrain_float(base_output - left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft,1000 * constrain_float(base_output + left,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight,1000 * constrain_float(base_output + right,0,1));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear,  1000 * constrain_float(base_output + mid,0,1));
    } else {
        // VTOL vectored yaw mode (tilted less than Q_TILT_MAX)
        // Use differential tilt for yaw control and combine with roll demand
        
        // Get control demands from multicopter controller (includes feedforward)
        const float yaw_out = motors->get_yaw()+motors->get_yaw_ff();
        const float roll_out = motors->get_roll()+motors->get_roll_ff();
        const float yaw_range = zero_out;  // Maximum differential tilt range for yaw

        // Throttle-based scaling: Increase yaw authority at low throttle to maintain control
        // At hover throttle, scaler=1.0; at lower throttle, scaler increases to compensate
        // for reduced thrust available for yaw control
        const float throttle = motors->get_throttle_out();
        const float scale_min = 0.5;  // Don't reduce below 50% (high throttle case)
        const float scale_max = 2.0;  // Don't increase above 200% (low throttle case)
        float throttle_scaler = scale_max;
        if (is_positive(throttle)) {
            throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
        }

        // Apply vectored thrust for combined yaw and roll control
        // As motors tilt forward, effectiveness changes:
        // - Yaw authority from differential tilt decreases (cos_tilt factor)
        // - Roll authority from differential tilt increases (sin_tilt factor)
        const float tilt_rad = radians(current_tilt*90);
        const float sin_tilt = sinf(tilt_rad);
        const float cos_tilt = cosf(tilt_rad);
        
        // MotorsMatrix library normalizes roll factor to 0.5, so use same factor
        // here to maintain consistent roll gains regardless of tilt angle
        const float avg_roll_factor = 0.5;
        
        // Combined tilt scaling: yaw demand (with throttle scaling) + roll demand
        float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

        // Limit and flag if saturated to prevent integrator windup
        if (fabsf(tilt_scale) > 1.0) {
            tilt_scale = constrain_float(tilt_scale, -1.0, 1.0);
            motors->limit.yaw = true;  // Signal controller that yaw is saturated
        }

        // Convert scaled demand to differential tilt offset
        const float tilt_offset = tilt_scale * yaw_range;

        // Calculate individual motor tilt positions: left increases, right decreases
        float left_tilt = base_output + tilt_offset;
        float right_tilt = base_output - tilt_offset;

        // Check for bilateral saturation: if BOTH left and right are out of range,
        // the yaw demand cannot be satisfied, so set limit flag
        // This is different from unilateral saturation where one side is limited
        if (((left_tilt > 1.0) || (left_tilt < 0.0)) &&
            ((right_tilt > 1.0) || (right_tilt < 0.0))) {
            motors->limit.yaw = true;
        }

        // Constrain to valid range [0.0, 1.0] and scale to servo output range [0, 1000]
        left_tilt = constrain_float(left_tilt,0.0,1.0) * 1000.0;
        right_tilt = constrain_float(right_tilt,0.0,1.0) * 1000.0;

        // Apply calculated tilt angles to all servo channels
        // Front motors: left and right get differential tilt for yaw/roll
        // Rear motors: center motor at base position, left/right follow front differential
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, 1000.0 * constrain_float(base_output,0.0,1.0));
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, left_tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, right_tilt);
    }
}

/**
 * @brief Control bicopter tiltrotor servo outputs
 * 
 * @details Bicopter tiltrotors (TILT_TYPE_BICOPTER) use left and right tilt servos
 *          for both thrust vectoring and control. Unlike standard tiltrotors, the
 *          bicopter continuously varies tilt angle for pitch/yaw control even while
 *          hovering. This function:
 * 
 *          1. Sets full forward tilt when in fixed-wing mode and fully transitioned
 *          2. Runs multicopter control to get base tilt servo outputs
 *          3. Scales tilt authority based on forward tilt angle (reduces as tilted forward)
 *          4. Combines base tilt position with control demands
 * 
 *          Tilt Servo Convention:
 *          - Negative values = tilt backward (more VTOL authority)
 *          - Zero = neutral position
 *          - Positive values = tilt forward (more forward thrust)
 * 
 *          Control Authority Scaling:
 *          As motors tilt forward, pitch/yaw authority from tilt servos decreases
 *          by cos(tilt_angle) since thrust vector becomes more horizontal.
 * 
 * @note Only called when type == TILT_TYPE_BICOPTER
 * @note Does not run during motor test to avoid interference
 * @note Bicopter assumes trim position is vertical (up), scales negative values by Q_TILT_YAW_ANGLE
 * 
 * @warning Bicopter requires frame class 10 (tailsitter) in Q_FRAME_CLASS
 * 
 * @see quadplane.motors_output() for multicopter control calculation
 * @see current_tilt for current tilt position (0=vertical, 1=forward)
 */
void Tiltrotor::bicopter_output(void)
{
    if (type != TILT_TYPE_BICOPTER || quadplane.motor_test.running) {
        // don't override motor test with motors_output
        return;
    }

    if (!quadplane.in_vtol_mode() && fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }

    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (quadplane.assisted_flight) {
        quadplane.hold_stabilize(throttle * 0.01f);
        quadplane.motors_output(true);
    } else {
        quadplane.motors_output(false);
    }

    // bicopter assumes that trim is up so we scale down so match
    float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
    float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

    if (is_negative(tilt_left)) {
        tilt_left *= tilt_yaw_angle * (1/90.0);
    }
    if (is_negative(tilt_right)) {
        tilt_right *= tilt_yaw_angle * (1/90.0);
    }

    // reduce authority of bicopter as motors are tilted forwards
    const float scaling = cosf(current_tilt * M_PI_2);
    tilt_left  *= scaling;
    tilt_right *= scaling;

    // add current tilt and constrain
    tilt_left  = constrain_float(-(current_tilt * SERVO_MAX) + tilt_left,  -SERVO_MAX, SERVO_MAX);
    tilt_right = constrain_float(-(current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft,  tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}

/**
 * @brief Update yaw target for coordinated turns during forward transition
 * 
 * @details During forward transition of tilt-vectored quadplanes, this function
 *          maintains coordinated flight by updating the yaw target. It implements
 *          two key behaviors:
 * 
 *          1. Yaw Lock on Transition Start:
 *             When transition begins (or every 100ms without pilot yaw input),
 *             locks current heading as the target. This enables straight-line
 *             transitions for tilt-vectored vehicles.
 * 
 *          2. Coordinated Turn Calculation:
 *             When banked (>10° roll) and airspeed available, calculates the
 *             yaw rate required for a coordinated turn at the current bank angle
 *             and airspeed. Integrates this rate to update the yaw target,
 *             allowing the vehicle to track coordinated turns smoothly.
 * 
 *          Algorithm:
 *          - If pilot commands yaw OR timeout (100ms): Lock to current heading
 *          - If banked with airspeed: Calculate turn rate = V * tan(bank) / R
 *          - Integrate turn rate to update yaw target over time delta
 * 
 * @note Only used for vectored yaw tiltrotors (TILT_TYPE_VECTORED_YAW)
 * @note Called during forward transition when multirotor control is active
 * @note Uses fixedwing_turn_rate() for coordinated turn rate calculation
 * @note Respects Q_ARSPD_MIN (minimum 5 m/s) to avoid unrealistic turn rates
 * 
 * @see transition_yaw_cd for the calculated yaw target in centidegrees
 * @see fixedwing_turn_rate() for turn rate physics calculation
 * @see Tiltrotor_Transition::use_multirotor_control_in_fwd_transition()
 */
void Tiltrotor::update_yaw_target(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - transition_yaw_set_ms > 100 ||
        !is_zero(quadplane.get_pilot_input_yaw_rate_cds())) {
        // lock initial yaw when transition is started or when
        // pilot commands a yaw change. This allows us to track
        // straight in transitions for tilt-vectored planes, but
        // allows for turns when level transition is not wanted
        transition_yaw_cd = quadplane.ahrs.yaw_sensor;
    }

    /*
      now calculate the equivalent yaw rate for a coordinated turn for
      the desired bank angle given the airspeed
     */
    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    if (have_airspeed && labs(plane.nav_roll_cd)>1000) {
        float dt = (now - transition_yaw_set_ms) * 0.001;
        // calculate the yaw rate to achieve the desired turn rate
        const float airspeed_min = MAX(plane.aparm.airspeed_min,5);
        const float yaw_rate_cds = fixedwing_turn_rate(plane.nav_roll_cd*0.01, MAX(aspeed,airspeed_min))*100;
        transition_yaw_cd += yaw_rate_cds * dt;
    }
    transition_yaw_set_ms = now;
}


/**
 * @brief Determine if multirotor control should be used during forward transition
 * 
 * @details For vectored-yaw tiltrotors, multirotor rate control continues to be
 *          used during the early stages of forward transition (up to TRANSITION_TIMER).
 *          This provides better yaw authority and stability during the critical
 *          transition phase when airspeed is building but not yet sufficient for
 *          aerodynamic control.
 * 
 *          Conditions for multirotor control:
 *          1. Vehicle must be vectored-yaw type (differential tilt for yaw)
 *          2. Transition state must be <= TRANSITION_TIMER (early transition phase)
 * 
 *          Once transition progresses beyond TRANSITION_TIMER, the vehicle switches
 *          to fixed-wing control as airspeed is sufficient.
 * 
 * @return true if multirotor control should be used in forward transition
 * @return false if fixed-wing control should be used
 * 
 * @note Only applies to TILT_TYPE_VECTORED_YAW configurations
 * @note Non-vectored tiltrotors switch to fixed-wing control immediately
 * 
 * @see tiltrotor.is_vectored() for vectored-yaw configuration check
 * @see transition_state for current transition phase
 */
bool Tiltrotor_Transition::use_multirotor_control_in_fwd_transition() const
{
    return tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER;
}

/**
 * @brief Update yaw target during forward transition (Tiltrotor_Transition wrapper)
 * 
 * @details This is a wrapper function that delegates to Tiltrotor::update_yaw_target()
 *          when appropriate. It provides the transition logic with an updated yaw
 *          target for coordinated turns during forward transition.
 * 
 *          Behavior:
 *          - If multirotor control not needed in forward transition: Returns false, no update
 *          - If multirotor control active: Calls tiltrotor.update_yaw_target() and returns target
 * 
 * @param[out] yaw_target_cd Updated yaw target in centidegrees if return true
 * 
 * @return true if yaw target was updated (multirotor control active in transition)
 * @return false if yaw target not updated (no multirotor control needed)
 * 
 * @note Called by transition state machine to get yaw target for rate controller
 * @note Only active during vectored-yaw forward transitions
 * 
 * @see use_multirotor_control_in_fwd_transition() for condition check
 * @see Tiltrotor::update_yaw_target() for actual yaw target calculation
 */
bool Tiltrotor_Transition::update_yaw_target(float& yaw_target_cd)
{
    if (!use_multirotor_control_in_fwd_transition()) {
        return false;
    }
    tiltrotor.update_yaw_target();
    yaw_target_cd = tiltrotor.transition_yaw_cd;
    return true;
}

/**
 * @brief Determine if VTOL view should be shown to pilot
 * 
 * @details Controls whether the ground control station (GCS) and OSD should display
 *          VTOL-style indicators vs fixed-wing indicators. This affects displayed
 *          information such as attitude representation and flight mode visualization.
 * 
 *          VTOL view is shown when:
 *          1. Vehicle is in a VTOL flight mode (hover, loiter, etc.), OR
 *          2. Vehicle is a vectored-yaw tiltrotor in early forward transition
 *             (transition_state <= TRANSITION_TIMER)
 * 
 *          The second condition ensures that during forward transition of vectored-yaw
 *          vehicles, the pilot sees VTOL-style displays since multirotor control is
 *          still active and the vehicle behavior is more VTOL-like than fixed-wing.
 * 
 * @return true if VTOL-style display should be shown
 * @return false if fixed-wing-style display should be shown
 * 
 * @note Affects GCS and OSD display modes
 * @note Vectored-yaw vehicles show VTOL view during early forward transition
 * @note Non-vectored tiltrotors switch to fixed-wing view immediately on transition
 * 
 * @see quadplane.in_vtol_mode() for VTOL mode check
 * @see tiltrotor.is_vectored() for vectored-yaw configuration check
 */
bool Tiltrotor_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (!show_vtol && tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER) {
        // we use multirotor controls during fwd transition for
        // vectored yaw vehicles
        return true;
    }

    return show_vtol;
}

/**
 * @brief Check if tilt angle exceeds maximum VTOL angle threshold
 * 
 * @details Determines if the motors are tilted beyond the angle where multicopter
 *          control is effective (Q_TILT_MAX). Beyond this threshold:
 *          - Multicopter attitude control is disabled
 *          - Vehicle flies as fixed-wing aircraft
 *          - For vectored tiltrotors: switches from VTOL yaw control to FW pitch/roll control
 * 
 *          The threshold is the minimum of:
 *          1. Q_TILT_MAX (maximum angle for VTOL control, typically 45°)
 *          2. Fully forward tilt (accounts for flap angle on tilt-wings)
 * 
 *          Calculation:
 *          - tilt_threshold = Q_TILT_MAX / 90.0 (normalized to 0-1 range)
 *          - Compare current_tilt against minimum of threshold and forward tilt
 * 
 * @return true if tilted beyond max VTOL angle (pure fixed-wing flight)
 * @return false if within VTOL angle range (multicopter control active)
 * 
 * @note Q_TILT_MAX typically set to 45° (threshold = 0.5)
 * @note Used by vectoring() to switch between VTOL and fixed-wing tilt control modes
 * @note Tilt-wings with flap angle may have reduced forward tilt, accounted for here
 * 
 * @see max_angle_deg (Q_TILT_MAX parameter)
 * @see get_forward_flight_tilt() for flap-adjusted forward tilt
 * @see current_tilt (0=vertical, 1=fully forward)
 */
bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}

/**
 * @brief Calculate effective forward throttle from tilting motors
 * 
 * @details Computes the average throttle of tilting motors for use in forward flight
 *          calculations. This is primarily used for vectored-yaw tiltrotors where the
 *          tilting motors contribute to forward thrust. The function:
 * 
 *          1. Iterates through all motors in the motor mask
 *          2. For each tilting motor, gets current thrust value
 *          3. Converts thrust to throttle using linearization curve
 *          4. Normalizes to 0-1 range (accounts for spin_min offset)
 *          5. Returns average throttle across all tilting motors
 * 
 *          Throttle Calculation:
 *          - Raw thrust → actuator output via thrust_to_actuator()
 *          - Subtract spin_min to get throttle above idle
 *          - Divide by throttle_range to normalize to 0-1
 *          - Average across all tilting motors
 * 
 * @param[out] throttle Average throttle of tilting motors (0.0 to 1.0) if return true
 * 
 * @return true if forward throttle calculated successfully
 * @return false if tiltrotor disabled, not vectored, invalid throttle range, or no motors
 * 
 * @note Only valid for vectored-yaw tiltrotors (_is_vectored == true)
 * @note Returns false for non-vectored and disabled tiltrotors
 * @note Uses thrust linearization curve (thr_lin) for accurate throttle estimation
 * @note Accounts for motor spin_min (idle throttle) in calculations
 * 
 * @see motors->get_thrust() for individual motor thrust values
 * @see motors->thr_lin.thrust_to_actuator() for thrust-to-throttle conversion
 * @see is_motor_tilting() to identify tilting motors
 */
bool Tiltrotor::get_forward_throttle(float &throttle) const
{
    if (!enabled() || !_is_vectored) {
        return false;
    }
    const float throttle_range = motors->thr_lin.get_spin_max() - motors->thr_lin.get_spin_min();
    if (!is_positive(throttle_range)) {
        return false;
    }
    float throttle_sum = 0.0f;
    uint8_t num_vectored_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (is_motor_tilting(i)) {
            float thrust;
            if (motors->get_thrust(i, thrust)) {
                throttle_sum += (motors->thr_lin.thrust_to_actuator(thrust) - motors->thr_lin.get_spin_min()) / throttle_range;
                num_vectored_motors ++;
            }
        }
    }
    if (num_vectored_motors > 0) {
        throttle = throttle_sum / (float)num_vectored_motors;
        return true;
    }
    return false;
}

#endif  // HAL_QUADPLANE_ENABLED
