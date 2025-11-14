/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file tailsitter.cpp
 * @brief Tailsitter aircraft configuration and control implementation for ArduPlane QuadPlane
 * 
 * @details This file implements control logic for tailsitter VTOL aircraft - vehicles that take off
 *          and land vertically on their tail, then transition to horizontal flight. Tailsitters are
 *          a unique QuadPlane configuration that sit vertically when landed and must manage complex
 *          transitions between vertical (VTOL) and horizontal (fixed-wing) flight orientations.
 *          
 *          Tailsitter aircraft types supported:
 *          - Dual motor tailsitters: Two motors with control surfaces (traditional tailsitter)
 *          - Copter-style tailsitters: Multi-motor configurations using AP_MotorsMatrix
 *          - Vectored tailsitters: Motors that can tilt for thrust vectoring
 *          - Control surface only: No thrust vectoring, pure aerodynamic control
 *          
 *          Configuration is enabled by:
 *          - Setting Q_FRAME_CLASS=10 for dedicated tailsitter frames, OR
 *          - Setting Q_TAILSIT_MOTMX nonzero with Q_FRAME_CLASS and Q_FRAME_TYPE 
 *            configured for a copter-style layout supported by AP_MotorsMatrix
 *          
 *          Key functionality:
 *          - Transition management: Smooth pitch transitions between vertical and horizontal flight
 *          - Orientation handling: Coordinate frame transformations between VTOL and FW attitudes
 *          - Input remapping: Different stick input conventions for VTOL vs FW control
 *          - Gain scheduling: Dynamic control gain scaling based on airspeed and attitude
 *          - Motor mixing: Selective motor control during forward flight transitions
 *          - Thrust vectoring: Optional tilt motor control for enhanced maneuverability
 *          
 * @note This code is conditionally compiled with HAL_QUADPLANE_ENABLED
 * @warning Tailsitter transitions are flight-critical operations requiring careful tuning
 * 
 * @see QuadPlane for the parent quadplane control architecture
 * @see Tiltrotor for alternative VTOL transition mechanisms
 * @see AP_MotorsMatrix for copter-style motor mixing on tailsitters
 * 
 * Source: ArduPlane/tailsitter.cpp
 */
#include "tailsitter.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

const AP_Param::GroupInfo Tailsitter::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Tailsitter
    // @Values: 0:Disable, 1:Enable, 2:Enable Always
    // @Description: This enables Tailsitter functionality. A value of 2 forces Qassist active and always stabilize in forward flight with airmode for stabilisation at 0 throttle, for use on vehicles with no control surfaces, vehicle will not arm in forward flight modes, see also Q_OPTIONS "Mtrs_Only_Qassist"
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Tailsitter, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE
    // @DisplayName: Tailsitter fixed wing transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Units: deg
    // @Range: 5 80
    AP_GROUPINFO("ANGLE", 2, Tailsitter, transition_angle_fw, 45),

    // @Param: ANG_VT
    // @DisplayName: Tailsitter VTOL transition angle
    // @Description: This is the pitch angle at which tailsitter aircraft will change from fixed wing control to VTOL control, if zero Q_TAILSIT_ANGLE will be used
    // @Units: deg
    // @Range: 5 80
    AP_GROUPINFO("ANG_VT", 3, Tailsitter, transition_angle_vtol, 0),

    // @Param: INPUT
    // @DisplayName: Tailsitter input type bitmask
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When PlaneMode is not enabled (bit0 = 0) the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When PlaneMode input is enabled, the roll and yaw sticks are swapped so that the roll stick controls earth-frame yaw and rudder controls earth-frame roll. When body-frame roll is enabled (bit1 = 1), the yaw stick controls earth-frame yaw rate and the roll stick controls roll in the tailsitter's body frame when flying level.
    // @Bitmask: 0:PlaneMode,1:BodyFrameRoll
    AP_GROUPINFO("INPUT", 4, Tailsitter, input_type, 0),

    // 5 was MASK
    // 6 was MASKCH

    // @Param: VFGAIN
    // @DisplayName: Tailsitter vector thrust gain in forward flight
    // @Description: This controls the amount of vectored thrust control used in forward flight for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("VFGAIN", 7, Tailsitter, vectored_forward_gain, 0),

    // @Param: VHGAIN
    // @DisplayName: Tailsitter vector thrust gain in hover
    // @Description: This controls the amount of vectored thrust control used in hover for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("VHGAIN", 8, Tailsitter, vectored_hover_gain, 0.5),

    // @Param: VHPOW
    // @DisplayName: Tailsitter vector thrust gain power
    // @Description: This controls the amount of extra pitch given to the vectored control when at high pitch errors
    // @Range: 0 4
    // @Increment: 0.1
    AP_GROUPINFO("VHPOW", 9, Tailsitter, vectored_hover_power, 2.5),

    // @Param: GSCMAX
    // @DisplayName: Maximum tailsitter gain scaling
    // @Description: Maximum gain scaling for tailsitter Q_TAILSIT_GSCMSK options
    // @Range: 1 5
    // @User: Standard
    AP_GROUPINFO("GSCMAX", 10, Tailsitter, throttle_scale_max, 2),

    // @Param: RLL_MX
    // @DisplayName: Maximum Roll angle
    // @Description: Maximum Allowed roll angle for tailsitters. If this is zero then Q_ANGLE_MAX is used.
    // @Units: deg
    // @Range: 0 80
    // @User: Standard
    AP_GROUPINFO("RLL_MX", 11, Tailsitter, max_roll_angle, 0),

    // @Param: MOTMX
    // @DisplayName: Tailsitter motor mask
    // @Description: Bitmask of motors to remain active in forward flight for a 'Copter' tailsitter. Non-zero indicates airframe is a Copter tailsitter and uses copter style motor layouts determined by Q_FRAME_CLASS and Q_FRAME_TYPE. This should be zero for non-Copter tailsitters.
    // @User: Standard
    // @Bitmask: 0:Motor 1, 1:Motor 2, 2:Motor 3, 3:Motor 4, 4:Motor 5, 5:Motor 6, 6:Motor 7, 7:Motor 8, 8:Motor 9, 9:Motor 10, 10:Motor 11, 11:Motor 12
    AP_GROUPINFO("MOTMX", 12, Tailsitter, motor_mask, 0),

    // @Param: GSCMSK
    // @DisplayName: Tailsitter gain scaling mask
    // @Description: Bitmask of gain scaling methods to be applied: Throttle: scale gains with throttle, ATT_THR: reduce gain at high throttle/tilt, 2:Disk theory velocity calculation, requires Q_TAILSIT_DSKLD to be set, ATT_THR must not be set, 3:Altitude correction, scale with air density
    // @User: Standard
    // @Bitmask: 0:Throttle,1:ATT_THR,2:Disk Theory,3:Altitude correction
    AP_GROUPINFO("GSCMSK", 13, Tailsitter, gain_scaling_mask, TAILSITTER_GSCL_THROTTLE),

    // @Param: GSCMIN
    // @DisplayName: Minimum tailsitter gain scaling
    // @Description: Minimum gain scaling for tailsitter Q_TAILSIT_GSCMSK options
    // @Range: 0.1 1
    // @User: Standard
    AP_GROUPINFO("GSCMIN", 14, Tailsitter, gain_scaling_min, 0.4),

    // @Param: DSKLD
    // @DisplayName: Tailsitter disk loading
    // @Description: This is the vehicle weight in kg divided by the total disk area of all propellers in m^2. Only used with Q_TAILSIT_GSCMSK = 4
    // @Units: kg/m/m
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DSKLD", 15, Tailsitter, disk_loading, 0),

    // @Param: RAT_FW
    // @DisplayName: Tailsitter VTOL to forward flight transition rate
    // @Description: The pitch rate at which tailsitter aircraft will pitch down in the transition from VTOL to forward flight
    // @Units: deg/s
    // @Range: 10 500
    AP_GROUPINFO("RAT_FW", 16, Tailsitter, transition_rate_fw, 50),

    // @Param: RAT_VT
    // @DisplayName: Tailsitter forward flight to VTOL transition rate
    // @Description: The pitch rate at which tailsitter aircraft will pitch up in the transition from forward flight to VTOL
    // @Units: deg/s
    // @Range: 10 500
    AP_GROUPINFO("RAT_VT", 17, Tailsitter, transition_rate_vtol, 50),

    // @Param: THR_VT
    // @DisplayName: Tailsitter forward flight to VTOL transition throttle
    // @Description: Throttle used during FW->VTOL transition, -1 uses hover throttle
    // @Units: %
    // @Range: -1 100
    AP_GROUPINFO("THR_VT", 18, Tailsitter, transition_throttle_vtol, -1),

    // @Param: VT_R_P
    // @DisplayName: Tailsitter VTOL control surface roll gain
    // @Description: Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains
    // @Range: 0 2
    AP_GROUPINFO("VT_R_P", 19, Tailsitter, VTOL_roll_scale, 1),

    // @Param: VT_P_P
    // @DisplayName: Tailsitter VTOL control surface pitch gain
    // @Description: Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains
    // @Range: 0 2
    AP_GROUPINFO("VT_P_P", 20, Tailsitter, VTOL_pitch_scale, 1),

    // @Param: VT_Y_P
    // @DisplayName: Tailsitter VTOL control surface yaw gain
    // @Description: Scale from PID output to control surface, for use where a single axis is actuated by both motors and Tilt/control surface on a copter style tailsitter, increase to favor control surfaces and reduce motor output by reducing gains
    // @Range: 0 2
    AP_GROUPINFO("VT_Y_P", 21, Tailsitter, VTOL_yaw_scale, 1),

    // @Param: MIN_VO
    // @DisplayName: Tailsitter Disk loading minimum outflow speed
    // @Description: Use in conjunction with disk theory gain scaling and Q_TAILSIT_DSKLD to specify minimum airspeed over control surfaces, this will be used to boost throttle, when descending for example, 0 disables
    // @Range: 0 15
    AP_GROUPINFO("MIN_VO", 22, Tailsitter, disk_loading_min_outflow, 0),

    AP_GROUPEND
};

/**
 * @brief Default parameter values optimized for tailsitter aircraft
 * 
 * @details This table provides tuned default parameters specifically for tailsitter configurations
 *          when Q_FRAME_CLASS is set to MOTOR_FRAME_TAILSITTER. These defaults are automatically
 *          applied during setup() to provide a better starting point for tailsitter flight than
 *          generic quadplane defaults.
 *          
 *          Key parameter categories:
 *          - Attitude rate control: Higher feedforward for responsive VTOL control
 *          - Pitch limits: Reduced to account for vertical takeoff/landing orientation
 *          - Position control: Tuned for tailsitter hover characteristics
 *          - Transition timing: Configured for typical tailsitter transition rates
 *          
 * @note These defaults are only applied if the individual parameters have not been manually configured
 * @see AP_Param::set_defaults_from_table()
 */
static const struct AP_Param::defaults_table_struct defaults_table_tailsitter[] = {
    { "KFF_RDDRMIX",       0.02 },
    { "Q_A_RAT_PIT_FF",    0.2 },
    { "Q_A_RAT_YAW_FF",    0.2 },
    { "Q_A_RAT_YAW_I",     0.18 },
    { "Q_A_ANGLE_BOOST",   0 },
    { "PTCH_LIM_MAX_DEG",  30 },
    { "PTCH_LIM_MIN_DEG", -30 },
    { "MIXING_GAIN",      1.0 },
    { "RUDD_DT_GAIN",      10 },
    { "Q_TRANSITION_MS",   2000 },
    { "Q_TRANS_DECEL",    6 },
    { "Q_A_ACCEL_P_MAX",    30000},
    { "Q_A_ACCEL_R_MAX",    30000},
    { "Q_P_POSXY_P",        0.5},
    { "Q_P_VELXY_P",        1.0},
    { "Q_P_VELXY_I",        0.5},
    { "Q_P_VELXY_D",        0.25},
    
};

/**
 * @brief Construct a new Tailsitter object
 * 
 * @details Initializes the tailsitter subsystem with references to the parent quadplane
 *          and motors objects. This constructor is called during QuadPlane initialization
 *          when tailsitter support is enabled.
 * 
 * @param[in] _quadplane Reference to parent QuadPlane object for accessing shared VTOL state
 * @param[in] _motors Reference to multicopter motors object for motor control
 * 
 * @note Parameter defaults are established using AP_Param::setup_object_defaults()
 * @see QuadPlane::QuadPlane() for parent initialization sequence
 */
Tailsitter::Tailsitter(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Initialize tailsitter configuration and detect vehicle type
 * 
 * @details Performs one-time setup for tailsitter operation including:
 *          - Auto-detection of tailsitter configuration from frame class and motor mask
 *          - Configuration of transition rates based on transition angles and timing
 *          - Detection of control surface availability (elevator, aileron, rudder, elevons, v-tail)
 *          - Detection of thrust vectoring capability (tilt motors)
 *          - Application of tailsitter-specific parameter defaults
 *          - Setup of special modes for control-surface-less operation (Q_TAILSIT_ENABLE=2)
 *          - Creation and initialization of Tailsitter_Transition object
 *          
 *          Tailsitter enable logic (Q_TAILSIT_ENABLE):
 *          - 0: Disabled
 *          - 1: Enabled (standard tailsitter operation)
 *          - 2: Enable Always - Forces Q_Assist active, enables airmode in forward flight,
 *               prevents arming in forward flight modes (for control-surface-less tailsitters)
 *          
 *          Auto-enable heuristic: If Q_TAILSIT_ENABLE is not configured and either
 *          Q_FRAME_CLASS is MOTOR_FRAME_TAILSITTER or Q_TAILSIT_MOTMX is nonzero
 *          (excluding bicopter tiltrotors), then tailsitter support is automatically enabled.
 * 
 * @note This must be called during QuadPlane::setup() before first flight operation
 * @warning Modifying configuration after setup may result in inconsistent behavior
 * 
 * @see QuadPlane::setup() for parent initialization
 * @see Tailsitter_Transition for transition state machine initialization
 */
void Tailsitter::setup()
{
    // Set tailsitter enable flag based on old heuristics
    if (!enable.configured() && (((quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) || (motor_mask != 0)) && (quadplane.tiltrotor.type != Tiltrotor::TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }

    if (enable <= 0) {
        return;
    }

    quadplane.thrust_type = QuadPlane::ThrustType::TAILSITTER;

    // Set tailsitter transition rate to match old calculation
    if (!transition_rate_fw.configured()) {
        transition_rate_fw.set_and_save(transition_angle_fw / (quadplane.transition_time_ms/2000.0f));
    }

    // TODO: update this if servo function assignments change
    // used by relax_attitude_control() to control special behavior for vectored tailsitters
    _is_vectored = (quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) &&
                   (!is_zero(vectored_hover_gain) &&
                    (SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft) ||
                     SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorRight)));

    _have_elevator = SRV_Channels::function_assigned(SRV_Channel::k_elevator);
    _have_aileron = SRV_Channels::function_assigned(SRV_Channel::k_aileron);
    _have_rudder = SRV_Channels::function_assigned(SRV_Channel::k_rudder);
    _have_elevon = SRV_Channels::function_assigned(SRV_Channel::k_elevon_left) || SRV_Channels::function_assigned(SRV_Channel::k_elevon_right);
    _have_v_tail = SRV_Channels::function_assigned(SRV_Channel::k_vtail_left) || SRV_Channels::function_assigned(SRV_Channel::k_vtail_right);

    // set defaults for dual/single motor tailsitter
    if (quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) {
        AP_Param::set_defaults_from_table(defaults_table_tailsitter, ARRAY_SIZE(defaults_table_tailsitter));
    }

    // Setup for control surface less operation
    if (enable == 2) {
        quadplane.assist.set_state(VTOL_Assist::STATE::FORCE_ENABLED);
        quadplane.air_mode = AirMode::ASSISTED_FLIGHT_ONLY;

        // Do not allow arming in forward flight modes
        // motors will become active due to assisted flight airmode, the vehicle will try very hard to get level
        quadplane.options.set(quadplane.options.get() | int32_t(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO));
    }

    transition = NEW_NOTHROW Tailsitter_Transition(quadplane, motors, *this);
    if (!transition) {
        AP_BoardConfig::allocation_error("tailsitter transition");
    }
    quadplane.transition = transition;

    setup_complete = true;
}

/**
 * @brief Check if this is a control-surface-only tailsitter (no thrust vectoring)
 * 
 * @details A control surface only tailsitter uses fixed motors and relies entirely on
 *          aerodynamic control surfaces (elevator, aileron, rudder) for attitude control
 *          in VTOL modes. This is true when:
 *          - Frame class is MOTOR_FRAME_TAILSITTER (dual motor configuration), AND
 *          - Either vectored_hover_gain is zero OR no tilt motors are assigned
 *          
 *          This distinction is important because control surface tailsitters have different
 *          control authority and response characteristics compared to vectored tailsitters,
 *          particularly at low airspeeds and during hover.
 * 
 * @return true if this is a control surface only tailsitter (no thrust vectoring)
 * @return false if thrust vectoring is available or not a dedicated tailsitter frame
 * 
 * @note Copter-style tailsitters (motor_mask != 0) always return false
 * @see vectored_hover_gain parameter for thrust vectoring control gain
 */
bool Tailsitter::is_control_surface_tailsitter(void) const
{
    return quadplane.frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER
           && ( is_zero(vectored_hover_gain) || !SRV_Channels::function_assigned(SRV_Channel::k_tiltMotorLeft));
}

/**
 * @brief Check if tailsitter control is currently active
 * 
 * @details Returns true when the vehicle is operating in tailsitter mode, meaning VTOL-style
 *          control is active with the vehicle in vertical orientation. This includes:
 *          - Any VTOL flight mode (QHOVER, QLOITER, QLAND, QRTL, etc.)
 *          - TRANSITION_ANGLE_WAIT_FW state (transitioning to FW but still in VTOL control)
 *          
 *          When active, the tailsitter uses multicopter-style attitude control with
 *          control surface mixing appropriate for vertical flight orientation.
 * 
 * @return true if tailsitter VTOL control is active
 * @return false if tailsitter is disabled or in fixed-wing flight mode
 * 
 * @note Returns false immediately if tailsitter is not enabled
 * @see enabled() for checking if tailsitter support is configured
 * @see in_vtol_transition() for checking transition state
 */
bool Tailsitter::active(void)
{
    if (!enabled()) {
        return false;
    }
    if (quadplane.in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
    if (transition->transition_state == Tailsitter_Transition::TRANSITION_ANGLE_WAIT_FW) {
        return true;
    }
    return false;
}

/**
 * @brief Main tailsitter motor and control surface output mixing function
 * 
 * @details This is the primary output function for tailsitter aircraft, called at the main loop rate
 *          (typically 400Hz) to convert attitude controller outputs into motor and servo commands.
 *          
 *          The function handles three distinct flight regimes:
 *          
 *          1. **Forward Flight (not active())**: 
 *             - Uses fixed-wing throttle and motor_mask to control specific motors
 *             - Applies thrust vectoring based on vectored_forward_gain if configured
 *             - Control surfaces follow standard fixed-wing mixing
 *          
 *          2. **VTOL to FW Transition (in_vtol_transition())**:
 *             - Overrides throttle to hover thrust or Q_TAILSIT_THR_VT setting
 *             - Centers rudder to prevent adverse yaw during attitude change
 *             - Gradually pitches nose down at Q_TAILSIT_RAT_FW rate
 *          
 *          3. **VTOL Flight (active() and not transitioning)**:
 *             - Maps copter attitude controller outputs to control surfaces:
 *               * Copter yaw → FW aileron (body frame roll in vertical orientation)
 *               * Copter pitch → FW elevator (body frame pitch)
 *               * Copter roll → FW rudder (body frame yaw in vertical orientation)
 *             - Applies gain scaling via speed_scaling() based on airspeed and throttle
 *             - Mixes elevons and v-tail with priority given to pitch control
 *             - Applies thrust vectoring with extra pitch authority at high pitch errors
 *          
 *          Special handling for Q_TAILSIT_Q_ASSIST_MOTORS_ONLY:
 *          - Motors provide VTOL stability assist in forward flight
 *          - Control surfaces remain under fixed-wing controller
 *          - Integrators coordinated between copter and plane controllers
 *          
 * @note This function is called every loop iteration when armed
 * @warning Incorrect mixing can result in loss of control - parameters must be carefully tuned
 * 
 * @see speed_scaling() for dynamic gain adjustment algorithm
 * @see QuadPlane::motors_output() for motor output execution
 * @see in_vtol_transition() for transition state determination
 * 
 * Source: ArduPlane/tailsitter.cpp:285-521
 */
void Tailsitter::output(void)
{
    if (!enabled() || quadplane.motor_test.running || !quadplane.initialised) {
        // if motor test is running we don't want to overwrite it with output_motor_mask or motors_output
        return;
    }

    if (!hal.util->get_soft_armed() ||
        SRV_Channels::get_emergency_stop()) {
        // Ensure motors stop on disarm
        motors->output_min();
    }

    // throttle 0 to 1
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01;

    // handle forward flight modes and transition to VTOL modes
    if (!active() || in_vtol_transition()) {
        // get FW controller throttle demand and mask of motors enabled during forward flight
        if (plane.arming.is_armed_and_safety_off() && in_vtol_transition() && !quadplane.throttle_wait) {
            /*
              during transitions to vtol mode set the throttle to hover thrust, center the rudder
            */
            if (!is_negative(transition_throttle_vtol)) { 
                // Q_TAILSIT_THR_VT is positive use it until transition is complete
                throttle = motors->thr_lin.actuator_to_thrust(MIN(transition_throttle_vtol*0.01,1.0));
            } else {
                throttle = motors->get_throttle_hover();
                // work out equivalent motors throttle level for cruise
                throttle = MAX(throttle,motors->thr_lin.actuator_to_thrust(plane.aparm.throttle_cruise.get() * 0.01));
            }

            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0.0);
            plane.rudder_dt = 0;

            // in assisted flight this is done in the normal motor output path
            if (!quadplane.assisted_flight) {

                // keep attitude control throttle level upto date, this value should never be output to motors
                // it is used to re-set the accel Z integrator term allowing for a smooth transfer of control
                quadplane.attitude_control->set_throttle_out(throttle, false, 0);

                // convert the hover throttle to the same output that would result if used via AP_Motors
                // apply expo, battery scaling and SPIN min/max.
                throttle = motors->thr_lin.thrust_to_actuator(throttle);

                // override AP_MotorsTailsitter throttles during back transition

                // apply PWM min and MAX to throttle left and right, just as via AP_Motors
                uint16_t throttle_pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * throttle;
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, throttle_pwm);
                SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, throttle_pwm);

                // throttle output is not used by AP_Motors so might have different PWM range, set scaled
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle * 100.0);
            }
        }

        if (!quadplane.assisted_flight) {
            // set AP_MotorsMatrix throttles for forward flight
            motors->output_motor_mask(throttle, uint32_t(motor_mask.get()), plane.rudder_dt);

            // No tilt output unless forward gain is set
            float tilt_left = 0.0;
            float tilt_right = 0.0;

            // in forward flight: set motor tilt servos and throttles using FW controller
            if (vectored_forward_gain > 0) {
                // remove scaling from surface speed scaling and apply throttle scaling
                const float scaler = plane.control_mode == &plane.mode_manual?1:(quadplane.FW_vector_throttle_scaling() / plane.get_speed_scaler());
                // thrust vectoring in fixed wing flight
                float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
                float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
                tilt_left  = (elevator + aileron) * vectored_forward_gain * scaler;
                tilt_right = (elevator - aileron) * vectored_forward_gain * scaler;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
            return;
        }
    }

    // handle Copter controller
    // the MultiCopter rate controller has already been run in an earlier call
    // to motors_output() from quadplane.update(), unless we are in assisted flight
    // tailsitter in TRANSITION_ANGLE_WAIT_FW is not really in assisted flight, its still in a VTOL mode
    if (quadplane.assisted_flight && (transition->transition_state != Tailsitter_Transition::TRANSITION_ANGLE_WAIT_FW)) {
        quadplane.hold_stabilize(throttle);
        quadplane.motors_output(true);

        if (quadplane.option_is_set(QuadPlane::OPTION::TAILSIT_Q_ASSIST_MOTORS_ONLY)) {
            // only use motors for Q assist, control surfaces remain under plane control. Zero copter I terms and use plane.
            // Smoothly relax to zero so there is no step change in output, must also set limit flags so integrator cannot build faster than the relax.
            // Assume there is always roll control surfaces, otherwise motors only assist should not be set.
            const float dt = quadplane.attitude_control->get_dt_s();

            // VTOL yaw / FW roll
            quadplane.attitude_control->get_rate_yaw_pid().relax_integrator(0.0, dt, AC_ATTITUDE_RATE_RELAX_TC);
            motors->limit.yaw = true;

            // VTOL and FW pitch
            if (_have_elevator || _have_elevon || _have_v_tail) {
                // have pitch control surfaces, use them
                quadplane.attitude_control->get_rate_pitch_pid().relax_integrator(0.0, dt, AC_ATTITUDE_RATE_RELAX_TC);
                motors->limit.pitch = true;
            } else {
                // no pitch control surfaces, zero plane I terms and use motors
                // We skip the outputting to surfaces for this axis from the copter controller but there are none setup
                plane.pitchController.reset_I();
            }

            // VTOL roll / FW yaw
            if (_have_rudder || _have_v_tail) {
                // there are yaw control  surfaces, zero motor I term
                quadplane.attitude_control->get_rate_roll_pid().relax_integrator(0.0, dt, AC_ATTITUDE_RATE_RELAX_TC);
                motors->limit.roll = true;
            } else {
                // no yaw control surfaces, zero plane I terms and use motors
                // We skip the outputting to surfaces for this axis from the copter controller but there are none setup
                plane.yawController.reset_I();
            }

            // output tilt motors

            // No output unless hover gain is set
            float tilt_left = 0.0;
            float tilt_right = 0.0;

            if (vectored_hover_gain > 0) {
                const float hover_throttle = motors->get_throttle_hover();
                const float output_throttle = motors->get_throttle();
                float throttle_scaler = throttle_scale_max;
                if (is_positive(output_throttle)) {
                    throttle_scaler = constrain_float(hover_throttle / output_throttle, gain_scaling_min, throttle_scale_max);
                }
                tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft) * vectored_hover_gain * throttle_scaler;
                tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight) * vectored_hover_gain * throttle_scaler;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);


            // skip remainder of the function that overwrites plane control surface outputs with copter
            return;
        }
    } else {
        quadplane.motors_output(false);
    }

    // In full Q assist it is better to use copter I and zero plane
    plane.pitchController.reset_I();
    plane.rollController.reset_I();
    plane.yawController.reset_I();

    // pull in copter control outputs
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, (motors->get_yaw()+motors->get_yaw_ff())*-SERVO_MAX*VTOL_yaw_scale);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, (motors->get_pitch()+motors->get_pitch_ff())*SERVO_MAX*VTOL_pitch_scale);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, (motors->get_roll()+motors->get_roll_ff())*SERVO_MAX*VTOL_roll_scale);

    if (plane.arming.is_armed_and_safety_off()) {
        // scale surfaces for throttle
        speed_scaling();
    } else if (tailsitter_motors != nullptr) {
        tailsitter_motors->set_min_throttle(0.0);
    }

    // No tilt output unless hover gain is set
    float tilt_left = 0.0;
    float tilt_right = 0.0;

    if (vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        float des_pitch_cd = quadplane.attitude_control->get_att_target_euler_cd().y;
        int32_t pitch_error_cd = (des_pitch_cd - quadplane.ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = 0;
        if (!is_zero(extra_pitch) && quadplane.in_vtol_mode()) {
            extra_elevator = extra_sign * powf(fabsf(extra_pitch), vectored_hover_power) * SERVO_MAX;
        }
        tilt_left  = extra_elevator + tilt_left * vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * vectored_hover_gain;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);

    // Check for saturated limits
    bool tilt_lim = _is_vectored && ((fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Function::k_tiltMotorLeft)) >= SERVO_MAX) || (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Function::k_tiltMotorRight)) >= SERVO_MAX));
    bool roll_lim = _have_rudder && (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Function::k_rudder)) >= SERVO_MAX);
    bool pitch_lim = _have_elevator && (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Function::k_elevator)) >= SERVO_MAX);
    bool yaw_lim = _have_aileron && (fabsf(SRV_Channels::get_output_scaled(SRV_Channel::Function::k_aileron)) >= SERVO_MAX);

    // Mix elevons and V-tail, always giving full priority to pitch
    float elevator_mix = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) * (100.0 - plane.g.mixing_offset) * 0.01 * plane.g.mixing_gain;
    float aileron_mix = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * (100.0 + plane.g.mixing_offset) * 0.01 * plane.g.mixing_gain;
    float rudder_mix = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * (100.0 + plane.g.mixing_offset) * 0.01 * plane.g.mixing_gain;

    const float headroom = SERVO_MAX - fabsf(elevator_mix);
    if (is_positive(headroom)) {
        if (fabsf(aileron_mix) > headroom) {
            aileron_mix *= headroom / fabsf(aileron_mix);
            yaw_lim |= _have_elevon;
        }
        if (fabsf(rudder_mix) > headroom) {
            rudder_mix *= headroom / fabsf(rudder_mix);
            roll_lim |= _have_v_tail;
        }
    } else {
        aileron_mix = 0.0;
        rudder_mix = 0.0;
        yaw_lim |= _have_elevon;
        pitch_lim |= _have_elevon || _have_v_tail;
        roll_lim |= _have_v_tail;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left, elevator_mix - aileron_mix);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right, elevator_mix + aileron_mix);
    SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_right, elevator_mix - rudder_mix);
    SRV_Channels::set_output_scaled(SRV_Channel::k_vtail_left, elevator_mix + rudder_mix);

    if (roll_lim) {
        motors->limit.roll = true;
    }
    if (pitch_lim || tilt_lim) {
        motors->limit.pitch = true;
    }
    if (yaw_lim || tilt_lim) {
        motors->limit.yaw = true;
    }

}


/**
 * @brief Check if transition from VTOL to fixed-wing flight is complete
 * 
 * @details Determines when the tailsitter has pitched down sufficiently to switch from
 *          multicopter-style attitude control to fixed-wing control. Transition is considered
 *          complete when ANY of the following conditions are met:
 *          
 *          1. **Pitch angle criterion**: Absolute pitch exceeds Q_TAILSIT_ANGLE (transition_angle_fw)
 *          2. **Roll error criterion**: Absolute roll exceeds roll limit + 5° (indicates loss of control)
 *          3. **Timeout criterion**: Time since transition start exceeds calculated duration based on
 *             transition angle and rate with 1.5x safety factor
 *          4. **Disarmed criterion**: Vehicle is disarmed (instant transition)
 *          
 *          The pitch-down transition is commanded at Q_TAILSIT_RAT_FW (degrees/second) by the
 *          Tailsitter_Transition state machine in TRANSITION_ANGLE_WAIT_FW state.
 * 
 * @return true when transition to fixed-wing control should be completed
 * @return false while still transitioning (continue VTOL control during pitch-down)
 * 
 * @note Sends GCS telemetry message when transition completes
 * @warning Excessive roll during transition triggers early completion with warning message
 * 
 * @see Tailsitter_Transition::update() for transition pitch rate control
 * @see transition_angle_fw parameter defining target pitch angle
 * @see transition_rate_fw parameter defining pitch rate in deg/s
 * 
 * Source: ArduPlane/tailsitter.cpp:527-547
 */
bool Tailsitter::transition_fw_complete(void)
{
    if (!plane.arming.is_armed_and_safety_off()) {
        // instant transition when disarmed, no message
        return true;
    }
    if (labs(quadplane.ahrs_view->pitch_sensor) > transition_angle_fw*100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
        return true;
    }
    if (labs(quadplane.ahrs_view->roll_sensor) > MAX(4500, plane.roll_limit_cd + 500)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition FW done, roll error");
        return true;
    }
    if (AP_HAL::millis() - transition->fw_transition_start_ms > ((transition_angle_fw+(transition->fw_transition_initial_pitch*0.01f))/transition_rate_fw)*1500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition FW done, timeout");
        return true;
    }
    // still waiting
    return false;
}


/**
 * @brief Check if transition from fixed-wing to VTOL flight is complete
 * 
 * @details Determines when the tailsitter has pitched up sufficiently to switch from fixed-wing
 *          control to multicopter-style attitude control. This transition occurs when entering
 *          VTOL modes from forward flight.
 *          
 *          Special case for vectored tailsitters at zero throttle:
 *          - If pilot throttle < 5% and groundspeed < 1 m/s (likely on ground)
 *          - Transition immediately to prevent propeller strikes during landing
 *          
 *          Standard transition completion criteria (ANY of):
 *          1. **Pitch angle criterion**: Absolute pitch exceeds get_transition_angle_vtol()
 *          2. **Roll error criterion**: Absolute roll exceeds roll limit + 5° (inverted flight accounted for)
 *          3. **Timeout criterion**: Time since transition start exceeds calculated duration
 *          4. **Disarmed criterion**: Vehicle is disarmed (instant transition)
 *          
 *          The pitch-up transition is commanded by the fixed-wing attitude controller at
 *          Q_TAILSIT_RAT_VT (degrees/second) rate in TRANSITION_ANGLE_WAIT_VTOL state.
 * 
 * @return true when transition to VTOL control should be completed
 * @return false while still transitioning (continue FW control during pitch-up)
 * 
 * @note Sends GCS telemetry message when transition completes
 * @warning For vectored tailsitters, zero throttle forces immediate transition to prevent prop strikes
 * 
 * @see get_transition_angle_vtol() for transition angle determination
 * @see transition_rate_vtol parameter defining pitch rate in deg/s
 * @see Tailsitter_Transition::set_FW_roll_pitch() for transition pitch control
 * 
 * Source: ArduPlane/tailsitter.cpp:553-586
 */
bool Tailsitter::transition_vtol_complete(void) const
{
    if (!plane.arming.is_armed_and_safety_off()) {
        // instant transition when disarmed, no message
        return true;
    }
    // for vectored tailsitters at zero pilot throttle
    if ((quadplane.get_pilot_throttle() < 0.05f) && _is_vectored) {
        // if we are not moving (hence on the ground?) or don't know
        // transition immediately to tilt motors up and prevent prop strikes
        if (quadplane.ahrs.groundspeed() < 1.0f) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done, zero throttle");
            return true;
        }
    }
    const float trans_angle = get_transition_angle_vtol();
    if (labs(plane.ahrs.pitch_sensor) > trans_angle*100) {
        gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL done");
        return true;
    }
    int32_t roll_cd = labs(plane.ahrs.roll_sensor);
    if (plane.fly_inverted()) {
        roll_cd = 18000 - roll_cd;
    }
    if (roll_cd > MAX(4500, plane.roll_limit_cd + 500)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition VTOL done, roll error");
        return true;
    }
    if (AP_HAL::millis() - transition->vtol_transition_start_ms >  ((trans_angle-(transition->vtol_transition_initial_pitch*0.01f))/transition_rate_vtol)*1500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Transition VTOL done, timeout");
        return true;
    }
    return false;
}

/**
 * @brief Remap pilot stick inputs based on tailsitter input type configuration
 * 
 * @details Tailsitters can use different pilot input conventions depending on Q_TAILSIT_INPUT
 *          configuration. When in VTOL modes with TAILSITTER_INPUT_PLANE bit set, this function
 *          swaps roll and yaw stick inputs to provide more intuitive control.
 *          
 *          Input remapping when TAILSITTER_INPUT_PLANE is set:
 *          - Roll stick → Controls yaw (earth-frame rotation)
 *          - Yaw stick (rudder) → Controls roll (inverted to match plane convention)
 *          
 *          This allows pilots familiar with flying planes to hover a tailsitter using similar
 *          stick inputs to how they would fly an airplane in knife-edge hover.
 *          
 *          Default behavior (TAILSITTER_INPUT_PLANE not set):
 *          - Roll stick → Controls roll in earth frame
 *          - Yaw stick → Controls yaw in earth frame (like a multicopter)
 *          
 * @note This function modifies RC channel control_in values, affecting all downstream control
 * @note Only active when tailsitter mode is active() and TAILSITTER_INPUT_PLANE bit is set
 * 
 * @see input_type parameter (Q_TAILSIT_INPUT) bitmask configuration
 * @see RC_Channel::get_control_in() and set_control_in() for input value access
 * 
 * Source: ArduPlane/tailsitter.cpp:589-601
 */
void Tailsitter::check_input(void)
{
    if (active() && (input_type & TAILSITTER_INPUT_PLANE)) {
        // the user has asked for body frame controls when tailsitter
        // is active. We switch around the control_in value for the
        // channels to do this, as that ensures the value is
        // consistent throughout the code
        int16_t roll_in = plane.channel_roll->get_control_in();
        int16_t yaw_in = plane.channel_rudder->get_control_in();
        plane.channel_roll->set_control_in(yaw_in);
        plane.channel_rudder->set_control_in(-roll_in);
    }
}

/**
 * @brief Check if tailsitter is currently transitioning from FW to VTOL flight
 * 
 * @details Returns true during the pitch-up transition when entering VTOL modes from forward
 *          flight. This state indicates the vehicle is still pitched forward but needs special
 *          handling as it transitions to vertical orientation.
 *          
 *          Transition is detected when:
 *          - Tailsitter is enabled and in a VTOL mode, AND
 *          - Either transition_state is TRANSITION_ANGLE_WAIT_VTOL, OR
 *          - Less than 1 second has elapsed since entering VTOL mode (recent mode change)
 *          
 *          During this transition period, special logic applies:
 *          - Fixed-wing attitude controller commands pitch-up at Q_TAILSIT_RAT_VT
 *          - Throttle may be overridden to Q_TAILSIT_THR_VT value
 *          - Stick mixing may be disabled for smoother autonomous transition
 * 
 * @param[in] now Current time in milliseconds (0 uses current time from AP_HAL::millis())
 * 
 * @return true if actively transitioning from forward to VTOL flight
 * @return false if in steady-state VTOL or FW flight, or tailsitter disabled
 * 
 * @note The 1-second grace period prevents control discontinuities during mode transitions
 * @see transition_vtol_complete() for transition completion detection
 * @see Tailsitter_Transition::VTOL_update() for transition state management
 * 
 * Source: ArduPlane/tailsitter.cpp:606-619
 */
bool Tailsitter::in_vtol_transition(uint32_t now) const
{
    if (!enabled() || !quadplane.in_vtol_mode()) {
        return false;
    }
    if (transition->transition_state == Tailsitter_Transition::TRANSITION_ANGLE_WAIT_VTOL) {
        return true;
    }
    if ((now != 0) && ((now - transition->last_vtol_mode_ms) > 1000)) {
        // only just come out of forward flight
        return true;
    }
    return false;
}

/**
 * @brief Check if tailsitter is in steady-state fixed-wing flight
 * 
 * @details Returns true only when the tailsitter is flying in fixed-wing mode with transitions
 *          fully completed. This indicates the vehicle is in horizontal orientation using
 *          fixed-wing control logic.
 *          
 *          Requires ALL of:
 *          - Tailsitter is enabled
 *          - NOT in a VTOL mode (flying in fixed-wing mode)
 *          - Transition state is TRANSITION_DONE (all transitions completed)
 * 
 * @return true if in steady-state fixed-wing flight
 * @return false if in VTOL mode, transitioning, or tailsitter disabled
 * 
 * @see active() for checking VTOL mode activity
 * @see Tailsitter_Transition::transition_state for current transition status
 * 
 * Source: ArduPlane/tailsitter.cpp:624-627
 */
bool Tailsitter::is_in_fw_flight(void) const
{
    return enabled() && !quadplane.in_vtol_mode() && transition->transition_state == Tailsitter_Transition::TRANSITION_DONE;
}

/**
 * @brief Get the pitch angle for completing VTOL transition
 * 
 * @details Returns the pitch angle threshold (in degrees) at which transition from fixed-wing
 *          to VTOL flight is considered complete. Provides fallback logic for configuration:
 *          
 *          - If Q_TAILSIT_ANG_VT is configured (non-zero): Use that value
 *          - If Q_TAILSIT_ANG_VT is zero (default): Use Q_TAILSIT_ANGLE as fallback
 *          
 *          This allows asymmetric transitions where VTOL-to-FW and FW-to-VTOL transitions
 *          can occur at different pitch angles if desired, or use the same angle if
 *          Q_TAILSIT_ANG_VT is left at default zero.
 * 
 * @return Transition angle in degrees for FW→VTOL transition
 * 
 * @note Typical values are 45-80 degrees depending on vehicle configuration
 * @see transition_angle_vtol parameter (Q_TAILSIT_ANG_VT)
 * @see transition_angle_fw parameter (Q_TAILSIT_ANGLE)
 * 
 * Source: ArduPlane/tailsitter.cpp:632-638
 */
int8_t Tailsitter::get_transition_angle_vtol() const
{
    if (transition_angle_vtol == 0) {
        return transition_angle_fw;
    }
    return transition_angle_vtol;
}


/**
 * @brief Apply dynamic gain scaling to control surfaces based on airspeed and flight conditions
 * 
 * @details This critical function adjusts control surface effectiveness in VTOL modes to account
 *          for varying airflow over the surfaces. At hover with low airspeed, surfaces are less
 *          effective than in forward flight, requiring gain scaling to maintain consistent control
 *          response. Multiple scaling methods are supported via Q_TAILSIT_GSCMSK bitmask.
 *          
 *          Gain Scaling Methods (bitmask options):
 *          
 *          1. **THROTTLE (bit 0)**: Scale gains proportionally with throttle
 *             - Assumes higher throttle creates more propwash over surfaces
 *             - Simple method, works well for direct motor-to-surface airflow
 *          
 *          2. **ATT_THR (bit 1)**: Reduce gains at high throttle/tilt angles
 *             - Prevents oscillations during high-speed VTOL flight
 *             - Attenuates gains when pitched away from vertical or throttle > 1.25× hover
 *             - Includes slew rate limiting for smooth scaling changes
 *          
 *          3. **DISK_THEORY (bit 2)**: Physics-based scaling using momentum theory
 *             - Estimates airflow velocity over surfaces using actuator disk theory
 *             - Requires Q_TAILSIT_DSKLD (disk loading in kg/m²) to be configured
 *             - Most accurate method, accounts for induced velocity from propellers
 *             - Can boost throttle to maintain minimum outflow speed (Q_TAILSIT_MIN_VO)
 *             - Formula: Ue² = (T / (0.5 × ρ × A)) + U0² where T=thrust, ρ=air density, A=disk area
 *          
 *          4. **ALTITUDE (bit 3)**: Correct for air density with altitude
 *             - Scales gains by air density ratio to maintain consistent control authority
 *             - Important for high-altitude operations
 *          
 *          The calculated scaling factor is applied to:
 *          - Aileron (VTOL yaw control)
 *          - Elevator (VTOL pitch control)
 *          - Rudder (VTOL roll control)
 *          - Tilt motors (always use throttle scaling)
 *          
 *          Scaling factors are constrained between Q_TAILSIT_GSCMIN and Q_TAILSIT_GSCMAX
 *          to prevent excessive attenuation or amplification.
 * 
 * @note Called from output() at main loop rate when armed in VTOL modes
 * @warning Incorrect gain scaling can cause oscillations or loss of control
 * @warning DISK_THEORY requires accurate Q_TAILSIT_DSKLD - measure vehicle mass and prop disk area
 * 
 * @see gain_scaling_mask parameter (Q_TAILSIT_GSCMSK) for method selection
 * @see disk_loading parameter (Q_TAILSIT_DSKLD) in kg/m² for disk theory calculations
 * @see throttle_scale_max parameter (Q_TAILSIT_GSCMAX) for maximum gain scaling
 * @see gain_scaling_min parameter (Q_TAILSIT_GSCMIN) for minimum gain scaling
 * 
 * Source: ArduPlane/tailsitter.cpp:644-798
 */
void Tailsitter::speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle_out();
    float spd_scaler = 1.0f;
    float disk_loading_min_throttle = 0.0;

    // Scaling with throttle
    float throttle_scaler = throttle_scale_max;
    if (is_positive(throttle)) {
        throttle_scaler = constrain_float(hover_throttle / throttle, gain_scaling_min, throttle_scale_max);
    }

    if ((gain_scaling_mask & TAILSITTER_GSCL_ATT_THR) != 0) {
        // reduce gains when flying at high speed in Q modes:

        // critical parameter: violent oscillations if too high
        // sudden loss of attitude control if too low
        const float min_scale = gain_scaling_min;
        float tthr = 1.25f * hover_throttle;

        // reduce control surface throws at large tilt
        // angles (assuming high airspeed)
        // ramp down from 1 to max_atten at tilt angles over trans_angle
        // (angles here are represented by their cosines)

        // Note that the cosf call will be necessary if trans_angle becomes a parameter
        // but the C language spec does not guarantee that trig functions can be used
        // in constant expressions, even though gcc currently allows it.
        constexpr float c_trans_angle = 0.9238795; // cosf(.125f * M_PI)

        // alpha = (1 - max_atten) / (c_trans_angle - cosf(radians(90)));
        const float alpha = (1 - min_scale) / c_trans_angle;
        const float beta = 1 - alpha * c_trans_angle;

        const float c_tilt = quadplane.ahrs_view->get_rotation_body_to_ned().c.z;
        if (c_tilt < c_trans_angle) {
            spd_scaler = constrain_float(beta + alpha * c_tilt, min_scale, 1.0f);
            // reduce throttle attenuation threshold too
            tthr = 0.5f * hover_throttle;
        }
        // if throttle is above hover thrust, apply additional attenuation
        if (throttle > tthr) {
            const float throttle_atten = 1 - (throttle - tthr) / (1 - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = constrain_float(spd_scaler, min_scale, 1.0f);
        }

        // limit positive and negative slew rates of applied speed scaling
        constexpr float posTC = 2.0f;   // seconds
        constexpr float negTC = 1.0f;   // seconds
        const float posdelta = plane.G_Dt / posTC;
        const float negdelta = plane.G_Dt / negTC;
        spd_scaler = constrain_float(spd_scaler, last_spd_scaler - negdelta, last_spd_scaler + posdelta);
        last_spd_scaler = spd_scaler;

        // also apply throttle scaling if enabled
        if ((spd_scaler >= 1.0f) && ((gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0)) {
            spd_scaler = MAX(throttle_scaler,1.0f);
        }

    } else if (((gain_scaling_mask & TAILSITTER_GSCL_DISK_THEORY) != 0) && is_positive(disk_loading.get())) {
        // Use disk theory to estimate the velocity over the control surfaces
        // https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html

        float airspeed;
        if (!quadplane.ahrs.airspeed_estimate(airspeed)) {
            // No airspeed estimate, use throttle scaling
            spd_scaler = throttle_scaler;

        } else {


            // use the equation: T = 0.5 * rho * A (Ue^2 - U0^2) solved for Ue^2:
            // Ue^2 = (T / (0.5 * rho *A)) + U0^2
            // We don't know thrust or disk area, use T = (throttle/throttle_hover) * weight
            // ((t / t_h ) * weight) / (0.5 * rho * A) = ((t / t_h) * mass * 9.81) / (0.5 * rho * A)
            // (mass / A) is disk loading DL so:
            // Ue^2 = (((t / t_h) * DL * 9.81)/(0.5 * rho)) + U0^2

            const float rho = SSL_AIR_DENSITY * quadplane.ahrs.get_air_density_ratio();
            float hover_rho = rho;
            if ((gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
                // if applying altitude correction use sea level density for hover case
                hover_rho = SSL_AIR_DENSITY;
            }

            // hover case: (t / t_h) = 1 and U0 = 0
            const float sq_hover_outflow = (disk_loading.get() * GRAVITY_MSS) / (0.5f * hover_rho);


            // calculate the true outflow speed
            const float sq_outflow = (((throttle/hover_throttle) *  disk_loading.get() * GRAVITY_MSS) / (0.5f * rho)) + sq(MAX(airspeed,0));

            // Scale by the ratio of squared hover outflow velocity to squared actual outflow velocity
            spd_scaler = throttle_scale_max;
            if (is_positive(sq_outflow)) {
                spd_scaler = constrain_float(sq_hover_outflow / sq_outflow, gain_scaling_min.get(), throttle_scale_max.get());
            }

            if (is_positive(disk_loading_min_outflow)) {
                // calculate throttle required to give minimum outflow speed over control surfaces
                if (is_positive(airspeed)) {
                    disk_loading_min_throttle = (((sq(disk_loading_min_outflow) - sq(airspeed)) * (0.5 * rho)) / (disk_loading.get() * GRAVITY_MSS)) * hover_throttle;
                } else {
                    // estimate backwards airspeed
                    float reverse_airspeed = 0.0;
                    Vector3f vel;
                    if (quadplane.ahrs.get_velocity_NED(vel)) {
                        reverse_airspeed = quadplane.ahrs.earth_to_body(vel - quadplane.ahrs.wind_estimate()).x;
                    }
                    // make sure actually negative
                    reverse_airspeed = MIN(reverse_airspeed, 0.0);
                    disk_loading_min_throttle = (((sq(disk_loading_min_outflow) + sq(reverse_airspeed)) * (0.5 * rho)) / (disk_loading.get() * GRAVITY_MSS)) * hover_throttle;
                }
                disk_loading_min_throttle = MAX(disk_loading_min_throttle, 0.0);
            }
        }

    } else if ((gain_scaling_mask & TAILSITTER_GSCL_THROTTLE) != 0) {
        spd_scaler = throttle_scaler;
    }

    if ((gain_scaling_mask & TAILSITTER_GSCL_ALTITUDE) != 0) {
        // air density correction
        spd_scaler /= quadplane.ahrs.get_air_density_ratio();
    }

    const SRV_Channel::Function functions[] = {
        SRV_Channel::Function::k_aileron,
        SRV_Channel::Function::k_elevator,
        SRV_Channel::Function::k_rudder,
        SRV_Channel::Function::k_tiltMotorLeft,
        SRV_Channel::Function::k_tiltMotorRight};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        float v = SRV_Channels::get_output_scaled(functions[i]);
        if ((functions[i] == SRV_Channel::Function::k_tiltMotorLeft) || (functions[i] == SRV_Channel::Function::k_tiltMotorRight)) {
            // always apply throttle scaling to tilts
            v *= throttle_scaler;
        } else {
            v *= spd_scaler;
        }
        SRV_Channels::set_output_scaled(functions[i], v);
    }

    if (tailsitter_motors != nullptr) {
        tailsitter_motors->set_min_throttle(disk_loading_min_throttle);
    }

    // Record for log
    log_data.throttle_scaler = throttle_scaler;
    log_data.speed_scaler = spd_scaler;
    log_data.min_throttle = disk_loading_min_throttle;

}

#if HAL_LOGGING_ENABLED
/**
 * @brief Write tailsitter-specific telemetry data to dataflash log
 * 
 * @details Logs tailsitter-specific parameters to the TSIT log message for post-flight analysis.
 *          This data is critical for tuning gain scaling and diagnosing control issues.
 *          
 *          Logged data includes:
 *          - throttle_scaler: Gain scaling factor based on throttle (hover/actual ratio)
 *          - speed_scaler: Final gain scaling factor applied to control surfaces
 *          - min_throttle: Minimum throttle override from disk loading calculations
 *          
 *          Log message format: LOG_TSIT_MSG
 *          
 *          This function is only compiled when HAL_LOGGING_ENABLED is defined.
 * 
 * @note Called once per loop iteration when tailsitter is enabled and armed
 * @note Does nothing if tailsitter is not enabled
 * 
 * @see speed_scaling() for gain scaling calculations that populate log_data
 * @see log_data struct for values being logged
 * 
 * Source: ArduPlane/tailsitter.cpp:802-817
 */
void Tailsitter::write_log()
{
    if (!enabled()) {
        return;
    }

    struct log_tailsitter pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TSIT_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_scaler     : log_data.throttle_scaler,
        speed_scaler        : log_data.speed_scaler,
        min_throttle        : log_data.min_throttle,
    };
    plane.logger.WriteBlock(&pkt, sizeof(pkt));
}
#endif  // HAL_LOGGING_ENABLED

/**
 * @brief Determine if pitch axis attitude control should be relaxed
 * 
 * @details Controls whether the pitch axis should be relaxed (integrators zeroed, control
 *          authority reduced) during certain flight phases. The decision accounts for
 *          propeller strike risk on vectored tailsitters.
 *          
 *          Pitch control is relaxed (returns true) when:
 *          - Tailsitter is not enabled, OR
 *          - This is not a vectored tailsitter (_is_vectored == false), OR
 *          - A recent VTOL transition has completed (vtol_limit_start_ms != 0)
 *          
 *          Pitch control is NOT relaxed (returns false) for vectored belly sitters to:
 *          - Keep thrust vector pointed upward during landing
 *          - Prevent propeller strikes on the ground
 *          - Maintain motor pointing angle for ground operations
 *          
 *          After any transition to VTOL, pitch is always relaxed temporarily to allow
 *          the attitude controller to settle to the new flight regime.
 * 
 * @return true if pitch control should be relaxed (safe to reduce control authority)
 * @return false if pitch control must remain active (vectored tailsitter, prop strike risk)
 * 
 * @note Called by attitude controller to determine control relaxation strategy
 * @warning Relaxing pitch on vectored tailsitters can cause propeller strikes during landing
 * 
 * @see _is_vectored flag indicating thrust vectoring capability
 * @see Tailsitter_Transition::vtol_limit_start_ms for post-transition timing
 * 
 * Source: ArduPlane/tailsitter.cpp:822-825
 */
bool Tailsitter::relax_pitch()
{
    return !enabled() || !_is_vectored || (transition->vtol_limit_start_ms != 0);
}

/**
 * @brief Main transition state machine update for VTOL to fixed-wing transition
 * 
 * @details This function implements the state machine for transitioning from VTOL to fixed-wing
 *          flight on tailsitter aircraft. Called at the main loop rate during forward flight modes.
 *          
 *          Transition States:
 *          
 *          **TRANSITION_ANGLE_WAIT_FW**: Actively pitching down from vertical to horizontal
 *          - Commands pitch-down at Q_TAILSIT_RAT_FW (degrees/second)
 *          - Maintains zero roll, disables yaw rate time constant
 *          - Holds throttle at hover or higher during transition
 *          - Uses synthetic airspeed in TECS to prevent throttle oscillations
 *          - Checks transition_fw_complete() to detect completion
 *          - On completion: Transitions to TRANSITION_DONE, begins pitch rate limiting
 *          
 *          **TRANSITION_ANGLE_WAIT_VTOL**: Handled by FW attitude controller
 *          - This state is managed in VTOL_update() and set_FW_roll_pitch()
 *          - Fixed-wing controller commands pitch-up to vertical
 *          
 *          **TRANSITION_DONE**: Steady-state forward flight
 *          - Normal fixed-wing control active
 *          - No special transition logic required
 *          
 *          Throughout all transition states, assisted flight is evaluated to determine if
 *          VTOL motors should provide stability assistance during forward flight.
 * 
 * @note This is called from QuadPlane::update() in fixed-wing flight modes
 * @note Synthetic airspeed prevents TECS from using unreliable airspeed during pitch transitions
 * 
 * @see Tailsitter::transition_fw_complete() for completion detection
 * @see VTOL_update() for handling FW→VTOL transitions
 * @see QuadPlane::assisted_flight for VTOL assist in forward flight
 * 
 * Source: ArduPlane/tailsitter.cpp:830-881
 */
void Tailsitter_Transition::update()
{
    const uint32_t now = millis();

    float aspeed;
    bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
    quadplane.assisted_flight = quadplane.assist.should_assist(aspeed, have_airspeed);

    if (transition_state < TRANSITION_DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }

    switch (transition_state) {

    case TRANSITION_ANGLE_WAIT_FW: {
        if (tailsitter.transition_fw_complete()) {
            transition_state = TRANSITION_DONE;
            if (plane.arming.is_armed_and_safety_off()) {
                fw_limit_start_ms = now;
                fw_limit_initial_pitch = constrain_float(quadplane.ahrs.pitch_sensor,-8500,8500);
                plane.nav_pitch_cd = fw_limit_initial_pitch;
                plane.nav_roll_cd = 0;
            }
            break;
        }
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        quadplane.assisted_flight = true;
        uint32_t dt = now - fw_transition_start_ms;
        // multiply by 0.1 to convert (degrees/second * milliseconds) to centi degrees
        plane.nav_pitch_cd = constrain_float(fw_transition_initial_pitch - (quadplane.tailsitter.transition_rate_fw * dt) * 0.1f * (plane.fly_inverted()?-1.0f:1.0f), -8500, 8500);
        plane.nav_roll_cd = 0;
        quadplane.disable_yaw_rate_time_constant();
        quadplane.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(plane.nav_roll_cd,
                                                                      plane.nav_pitch_cd,
                                                                      0);
        // set throttle at either hover throttle or current throttle, whichever is higher, through the transition
        quadplane.attitude_control->set_throttle_out(MAX(motors->get_throttle_hover(),quadplane.attitude_control->get_throttle_in()), true, 0);
        quadplane.motors_output();
        break;
    }

    case TRANSITION_ANGLE_WAIT_VTOL:
        // nothing to do, this is handled in the fixed wing attitude controller
        break;

    case TRANSITION_DONE:
        break;
    }
}

/**
 * @brief Handle transition state when entering VTOL modes from forward flight
 * 
 * @details Called when the aircraft is in a VTOL flight mode. Manages the transition from
 *          fixed-wing to VTOL control by detecting mode entry and initiating the pitch-up
 *          maneuver to vertical orientation.
 *          
 *          State Management:
 *          
 *          **Mode Entry Detection**:
 *          - Checks if more than 1 second has passed since last VTOL mode update
 *          - If true, aircraft is entering VTOL mode from forward flight
 *          - Sets transition_state to TRANSITION_ANGLE_WAIT_VTOL
 *          
 *          **During TRANSITION_ANGLE_WAIT_VTOL**:
 *          - Provides assisted flight during the pitch-up phase for stability
 *          - Continuously checks transition_vtol_complete() for completion
 *          - On completion:
 *            * Records vtol_limit_start_ms and vtol_limit_initial_pitch (if armed)
 *            * Clears inverted_flight flag for consistency with other quadplane types
 *            * Calls restart() to prepare for next FW transition
 *          
 *          **Mode Tracking**:
 *          - Updates last_vtol_mode_ms timestamp on every call
 *          - Used to detect transitions between FW and VTOL modes
 *          
 *          The vtol_limit_* variables are used to rate-limit pitch changes after transition
 *          completes, preventing abrupt attitude changes that could destabilize the aircraft.
 * 
 * @note Called from QuadPlane::update() when in VTOL flight modes
 * @note Resets assisted_flight logic when not actively checking transition completion
 * 
 * @see Tailsitter::transition_vtol_complete() for completion criteria
 * @see set_FW_roll_pitch() for pitch-up command generation during transition
 * @see restart() to prepare for the next VTOL→FW transition
 * 
 * Source: ArduPlane/tailsitter.cpp:883-920
 */
void Tailsitter_Transition::VTOL_update()
{
    const uint32_t now = AP_HAL::millis();

    if ((now - last_vtol_mode_ms) > 1000) {
        /*
          we are just entering a VTOL mode as a tailsitter, set
          our transition state so the fixed wing controller brings
          the nose up before we start trying to fly as a
          multicopter
         */
        transition_state = TRANSITION_ANGLE_WAIT_VTOL;
    }
    last_vtol_mode_ms = now;

    if (transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
        float aspeed;
        bool have_airspeed = quadplane.ahrs.airspeed_estimate(aspeed);
        // provide assistance in forward flight portion of tailsitter transition
        quadplane.assisted_flight = quadplane.assist.should_assist(aspeed, have_airspeed);
        if (!quadplane.tailsitter.transition_vtol_complete()) {
            return;
        }
        // transition to VTOL complete, if armed set vtol rate limit starting point
        if (plane.arming.is_armed_and_safety_off()) {
            vtol_limit_start_ms = now;
            vtol_limit_initial_pitch = quadplane.ahrs_view->pitch_sensor;
        }

        // clear inverted flight flag to make behaviour consistent
        // with other quadplane types
        plane.inverted_flight = false;
    } else {
        // Keep assistance reset while not checking
        quadplane.assist.reset();
    }
    restart();
}

/**
 * @brief Determine if OSD/telemetry should display VTOL or fixed-wing orientation
 * 
 * @details Controls the visual reference frame for the On-Screen Display (OSD) and ground
 *          control station attitude indicators during transitions. This ensures the display
 *          matches the actual aircraft orientation mode, accounting for transition phases.
 *          
 *          Decision Logic:
 *          
 *          **Base Decision**: Use VTOL view if in a VTOL flight mode
 *          
 *          **Transition Override 1**: During TRANSITION_ANGLE_WAIT_VTOL
 *          - Aircraft is in VTOL mode but still pitched forward
 *          - Return false (FW view) until transition completes
 *          - Prevents confusing display during pitch-up from horizontal
 *          
 *          **Transition Override 2**: During TRANSITION_ANGLE_WAIT_FW
 *          - Aircraft is in FW mode but still pitched vertical
 *          - Return true (VTOL view) until transition completes
 *          - Maintains vertical reference frame during pitch-down
 *          
 *          This provides intuitive horizon reference during transitions, preventing
 *          rapid OSD flips between vertical and horizontal orientations.
 * 
 * @return true if OSD should display VTOL (vertical) orientation reference
 * @return false if OSD should display fixed-wing (horizontal) orientation reference
 * 
 * @note Used by OSD code to select appropriate attitude display mode
 * @note Also affects GCS attitude indicator display orientation
 * 
 * @see quadplane.in_vtol_mode() for base mode detection
 * @see transition_state for current transition phase
 * 
 * Source: ArduPlane/tailsitter.cpp:922-936
 */
bool Tailsitter_Transition::show_vtol_view() const
{
    bool show_vtol = quadplane.in_vtol_mode();

    if (show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_VTOL)) {
        // in a vtol mode but still transitioning from forward flight
        return false;
    }
    if (!show_vtol && (transition_state == TRANSITION_ANGLE_WAIT_FW)) {
        // not in VTOL mode but still transitioning from VTOL
        return true;
    }
    return show_vtol;
}

/**
 * @brief Override fixed-wing pitch/roll commands during tailsitter transitions
 * 
 * @details This function is called by the fixed-wing attitude controller to modify pitch and
 *          roll commands during tailsitter transition phases. It implements rate-limited pitch
 *          changes to smoothly transition between vertical and horizontal flight attitudes.
 *          
 *          **During FW→VTOL Transition** (TRANSITION_ANGLE_WAIT_VTOL):
 *          - Calculates time since vtol_transition_start_ms
 *          - Commands pitch increase at Q_TAILSIT_RAT_VT (degrees/second) from initial pitch
 *          - Formula: pitch = initial_pitch + (rate × time_ms × 0.1)
 *          - Constrains pitch between -85° and +85° to prevent over-rotation
 *          - Forces roll to 0° to keep wings level during pitch-up
 *          - Updates vtol_transition_start_ms and initial_pitch when TRANSITION_DONE
 *          
 *          **Rate-Limited Pitch Down** (first seconds after VTOL→FW transition):
 *          - When fw_limit_start_ms is non-zero, limits initial pitch-down rate
 *          - Prevents sudden nose-drop when exiting vertical flight
 *          - Commands: pitch_limit = initial_pitch - (time × Q_TAILSIT_RAT_FW × 0.1)
 *          - Stops limiting when pitch reaches 0° or when limit would increase pitch
 *          - Forces roll to 0° during rate limiting for stability
 *          
 *          This provides smooth, controlled transitions between flight regimes while
 *          maintaining safe pitch rates and preventing loss of control during mode changes.
 * 
 * @param[in,out] nav_pitch_cd Desired pitch attitude in centidegrees (modified during transitions)
 * @param[in,out] nav_roll_cd Desired roll attitude in centidegrees (modified during transitions)
 * 
 * @note Called from fixed-wing attitude controller (plane.stabilize())
 * @note Modifies pitch/roll commands by reference - changes are directly applied
 * @warning Do not disable rate limiting - sudden pitch changes can cause loss of control
 * 
 * @see tailsitter.in_vtol_transition() to check if currently transitioning to VTOL
 * @see transition_rate_fw parameter (Q_TAILSIT_RAT_FW) for FW transition rate
 * @see transition_rate_vtol parameter (Q_TAILSIT_RAT_VT) for VTOL transition rate
 * 
 * Source: ArduPlane/tailsitter.cpp:938-968
 */
void Tailsitter_Transition::set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd)
{
    uint32_t now = AP_HAL::millis();
    if (tailsitter.in_vtol_transition(now)) {
        /*
          during transition to vtol in a tailsitter try to raise the
          nose while keeping the wings level
         */
        uint32_t dt = now - vtol_transition_start_ms;
        // multiply by 0.1 to convert (degrees/second * milliseconds) to centi degrees
        nav_pitch_cd = constrain_float(vtol_transition_initial_pitch + (tailsitter.transition_rate_vtol * dt) * 0.1f, -8500, 8500);
        nav_roll_cd = 0;

    } else if (transition_state == TRANSITION_DONE) {
        // still in FW, reset transition starting point
        vtol_transition_start_ms = now;
        vtol_transition_initial_pitch = constrain_float(plane.nav_pitch_cd,-8500,8500);

        // rate limit initial pitch down
        if (fw_limit_start_ms != 0) {
            const float pitch_limit_cd = fw_limit_initial_pitch - (now - fw_limit_start_ms) * tailsitter.transition_rate_fw * 0.1;
            if ((pitch_limit_cd <= 0) || (nav_pitch_cd >= pitch_limit_cd)) {
                // never limit past 0, never limit to a smaller pitch angle
                fw_limit_start_ms = 0;
            } else {
                nav_pitch_cd = pitch_limit_cd;
                nav_roll_cd = 0;
            }
        }
    }
}

/**
 * @brief Determine if pilot stick input should be mixed with transition commands
 * 
 * @details Controls whether pilot control stick inputs should be allowed to modify the
 *          commanded pitch and roll angles during automatic transition maneuvers. Stick
 *          mixing is disabled during critical transition phases to ensure the aircraft
 *          completes the orientation change safely without pilot interference.
 *          
 *          Stick Mixing Disabled (returns false) During:
 *          
 *          1. **FW→VTOL Transition Initial Pitch-Up**:
 *             - While tailsitter.in_vtol_transition() returns true
 *             - Prevents pilot from interfering with automatic pitch-up to vertical
 *             - Critical phase where precise pitch control is needed
 *          
 *          2. **Post-VTOL→FW Transition Rate Limiting**:
 *             - When transition_state == TRANSITION_DONE AND fw_limit_start_ms != 0
 *             - During initial pitch-down rate limiting after leaving vertical
 *             - Prevents pilot from commanding excessive nose-down pitch
 *          
 *          Stick Mixing Enabled (returns true):
 *          - All other flight phases
 *          - Normal VTOL flight
 *          - Steady-state forward flight
 *          - After transition rate limits have cleared
 *          
 *          This safety feature ensures transitions complete smoothly while allowing normal
 *          pilot control authority once the aircraft is in stable orientation.
 * 
 * @return true if pilot stick inputs should be mixed with commanded attitude
 * @return false if stick inputs should be ignored during critical transition phase
 * 
 * @note Called by attitude controller to determine input mixing strategy
 * @warning Disabling stick mixing during transitions is critical for safe operation
 * 
 * @see tailsitter.in_vtol_transition() for FW→VTOL transition detection
 * @see fw_limit_start_ms for post-transition rate limiting state
 * 
 * Source: ArduPlane/tailsitter.cpp:970-981
 */
bool Tailsitter_Transition::allow_stick_mixing() const
{
    // Transitioning into VTOL flight, initial pitch up
    if (tailsitter.in_vtol_transition()) {
        return false;
    }
    // Transitioning into fixed wing flight, levelling off
    if ((transition_state == TRANSITION_DONE) && (fw_limit_start_ms != 0)) {
        return false;
    }
    return true;
}

/**
 * @brief Apply rate limiting to VTOL pitch commands immediately after FW→VTOL transition
 * 
 * @details Prevents abrupt pitch changes when transitioning from forward flight to VTOL modes.
 *          After the aircraft pitches up to vertical orientation, this function rate-limits
 *          the pitch command toward level hover, preventing sudden attitude changes that could
 *          destabilize the aircraft or confuse the pilot.
 *          
 *          Rate Limiting Logic:
 *          
 *          **Activation**: When vtol_limit_start_ms is non-zero (set on transition completion)
 *          
 *          **Calculation**:
 *          - pitch_change_cd = (time_since_transition_ms × Q_TAILSIT_RAT_VT × 0.1)
 *          - Allows pitch to change at Q_TAILSIT_RAT_VT degrees/second toward 0°
 *          
 *          **Deactivation Conditions**:
 *          1. pitch_change_cd exceeds initial pitch angle (limit has crossed 0°)
 *          2. Desired pitch is already more extreme than the limit
 *          
 *          **Positive Initial Pitch** (pitched up):
 *          - Limit decreases from initial_pitch toward 0° at transition_rate_vtol
 *          - If desired pitch < limit, apply limit and force roll = 0
 *          
 *          **Negative Initial Pitch** (pitched down):
 *          - Limit increases from initial_pitch toward 0° at transition_rate_vtol
 *          - If desired pitch > limit, apply limit and force roll = 0
 *          
 *          When limiting is active, roll is forced to 0° for stability.
 * 
 * @param[in,out] nav_roll_cd Desired roll in centidegrees (set to 0 if limiting active)
 * @param[in,out] nav_pitch_cd Desired pitch in centidegrees (limited if rate too high)
 * 
 * @return true if rate limiting was applied (outputs were modified)
 * @return false if no limiting needed (outputs unchanged)
 * 
 * @note Called from VTOL attitude controller during multicopter flight modes
 * @note Clears vtol_limit_start_ms when limiting completes
 * @warning Disabling this rate limiting can cause sudden pitch changes and loss of control
 * 
 * @see vtol_limit_start_ms timestamp for rate limit activation
 * @see vtol_limit_initial_pitch for starting pitch angle
 * @see transition_rate_vtol parameter (Q_TAILSIT_RAT_VT) for rate limit value
 * 
 * Source: ArduPlane/tailsitter.cpp:983-1013
 */
bool Tailsitter_Transition::set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd)
{
    if (vtol_limit_start_ms == 0) {
        return false;
    }
    // prevent pitching towards 0 too quickly
    const float pitch_change_cd = (AP_HAL::millis() - vtol_limit_start_ms) * tailsitter.transition_rate_vtol * 0.1;
    if (pitch_change_cd > fabsf(vtol_limit_initial_pitch)) {
        // limit has passed 0, nothing to do
        vtol_limit_start_ms = 0;
        return false;
    }
    // continue limiting while limit angle is larger than desired angle
    if (is_negative(vtol_limit_initial_pitch)) {
        const float pitch_limit = vtol_limit_initial_pitch + pitch_change_cd;
        if (nav_pitch_cd > pitch_limit) {
            nav_pitch_cd = pitch_limit;
            nav_roll_cd = 0;
            return true;
        }
    } else {
        const float pitch_limit = vtol_limit_initial_pitch - pitch_change_cd;
        if (nav_pitch_cd < pitch_limit) {
            nav_pitch_cd = pitch_limit;
            nav_roll_cd = 0;
            return true;
        }
    }
    vtol_limit_start_ms = 0;
    return false;
}

/**
 * @brief Initialize state for VTOL→FW transition (pitch down to horizontal)
 * 
 * @details Called when the aircraft needs to transition from VTOL to fixed-wing flight.
 *          Sets up initial conditions for the pitch-down maneuver from vertical to
 *          horizontal orientation.
 *          
 *          Initialization Actions:
 *          
 *          1. **Set Transition State**: TRANSITION_ANGLE_WAIT_FW
 *             - Activates the FW transition state machine in update()
 *             - Begins commanding pitch-down at Q_TAILSIT_RAT_FW
 *          
 *          2. **Record Transition Start Time**: fw_transition_start_ms = current time
 *             - Used to calculate elapsed time for pitch rate integration
 *             - Used to detect transition timeout conditions
 *          
 *          3. **Capture Initial Pitch**: fw_transition_initial_pitch
 *             - Gets current attitude target pitch from VTOL controller (in degrees)
 *             - Converts quaternion to Euler angles
 *             - Converts radians to centidegrees (× 100)
 *             - Constrains to ±85° to prevent invalid starting conditions
 *             - This is the pitch angle from which the transition begins
 *          
 *          The captured initial pitch becomes the reference point for rate-controlled
 *          pitch-down commands during the transition to forward flight.
 * 
 * @note Called from VTOL_update() after completing FW→VTOL transition
 * @note Also called when entering forward flight modes from VTOL modes
 * @note Initial pitch is taken from attitude controller target, not actual attitude
 * 
 * @see update() for the transition state machine that uses these initialized values
 * @see transition_rate_fw parameter (Q_TAILSIT_RAT_FW) for pitch-down rate
 * 
 * Source: ArduPlane/tailsitter.cpp:1016-1021
 */
void Tailsitter_Transition::restart()
{
    transition_state = TRANSITION_ANGLE_WAIT_FW;
    fw_transition_start_ms = AP_HAL::millis();
    fw_transition_initial_pitch = constrain_float(quadplane.attitude_control->get_attitude_target_quat().get_euler_pitch() * degrees(100.0),-8500,8500);
}

/**
 * @brief Force immediate completion of VTOL→FW transition and prepare for FW→VTOL
 * 
 * @details Forcibly completes any in-progress VTOL→FW transition and sets up state for
 *          the next FW→VTOL transition. Used when the aircraft needs to immediately
 *          enter fixed-wing flight without waiting for the normal transition to complete,
 *          such as when switching to manual mode or during emergency situations.
 *          
 *          State Changes:
 *          
 *          1. **Set State to TRANSITION_DONE**:
 *             - Immediately exits any in-progress VTOL→FW transition
 *             - Indicates aircraft is in steady-state forward flight
 *          
 *          2. **Initialize FW→VTOL Transition Parameters**:
 *             - vtol_transition_start_ms = current time (reference for next transition)
 *             - vtol_transition_initial_pitch = current nav_pitch_cd (starting point)
 *             - Constrains initial pitch to ±85° for safety
 *          
 *          3. **Clear Post-Transition Rate Limiting**:
 *             - fw_limit_start_ms = 0 (disables pitch-down rate limiting)
 *             - Allows immediate full control authority in FW mode
 *          
 *          4. **Reset Assisted Flight**:
 *             - Clears VTOL assist state
 *             - Prevents unwanted motor assistance in FW flight
 *          
 *          This function is typically called when entering manual or stabilize mode
 *          from a VTOL mode, where the pilot takes immediate control without waiting
 *          for automatic transition completion.
 * 
 * @note Called when pilot overrides automatic transition (mode changes, etc.)
 * @note Does not command any attitude changes, just updates state variables
 * @warning Only use when it's safe to assume forward flight orientation
 * 
 * @see transition_state for current transition phase
 * @see VTOL_update() for normal transition state management
 * 
 * Source: ArduPlane/tailsitter.cpp:1024-1033
 */
void Tailsitter_Transition::force_transition_complete()
{
    transition_state = TRANSITION_DONE;
    vtol_transition_start_ms = AP_HAL::millis();
    vtol_transition_initial_pitch = constrain_float(plane.nav_pitch_cd,-8500,8500);
    fw_limit_start_ms = 0;

    quadplane.assist.reset();
}

/**
 * @brief Get MAVLink VTOL state for telemetry reporting
 * 
 * @details Translates the internal tailsitter transition state into a MAVLink standard
 *          VTOL state enumeration for reporting to ground control stations. This allows
 *          GCS software to display the correct transition phase and orientation mode.
 *          
 *          State Mapping:
 *          
 *          **TRANSITION_ANGLE_WAIT_VTOL** → MAV_VTOL_STATE_TRANSITION_TO_MC
 *          - Aircraft is transitioning from forward flight to multicopter mode
 *          - Pitching up from horizontal to vertical orientation
 *          - Fixed-wing controller commanding pitch increase
 *          
 *          **TRANSITION_DONE** → MAV_VTOL_STATE_FW
 *          - Aircraft is in steady-state forward flight
 *          - Horizontal orientation, fixed-wing control active
 *          - No active transition in progress
 *          
 *          **TRANSITION_ANGLE_WAIT_FW**:
 *          - If in VTOL flight mode → MAV_VTOL_STATE_MC
 *            * Still in vertical orientation, VTOL control active
 *            * About to begin or preparing for transition
 *          - If in forward flight mode → MAV_VTOL_STATE_TRANSITION_TO_FW
 *            * Actively pitching down from vertical to horizontal
 *            * VTOL controller commanding pitch decrease
 *          
 *          **Default** → MAV_VTOL_STATE_UNDEFINED
 *          - Should never occur in normal operation
 *          - Indicates invalid or uninitialized state
 *          
 *          The MAVLink VTOL state is used by:
 *          - Ground control station displays
 *          - Telemetry logging
 *          - Mission planning software
 *          - Automated flight monitoring systems
 * 
 * @return MAV_VTOL_STATE enum value indicating current VTOL/transition state
 * 
 * @note Called by MAVLink telemetry code for EXTENDED_SYS_STATE message
 * @note State mapping follows MAVLink VTOL state specification
 * 
 * @see transition_state for internal transition state
 * @see MAV_VTOL_STATE enum definition in MAVLink common.xml
 * 
 * Source: ArduPlane/tailsitter.cpp:1034-1052
 */
MAV_VTOL_STATE Tailsitter_Transition::get_mav_vtol_state() const
{
    switch (transition_state) {
        case TRANSITION_ANGLE_WAIT_VTOL:
            return MAV_VTOL_STATE_TRANSITION_TO_MC;

        case TRANSITION_DONE:
            return MAV_VTOL_STATE_FW;

        case TRANSITION_ANGLE_WAIT_FW: {
            if (quadplane.in_vtol_mode()) {
                return MAV_VTOL_STATE_MC;
            }
            return MAV_VTOL_STATE_TRANSITION_TO_FW;
        }
    }

    return MAV_VTOL_STATE_UNDEFINED;
}

/**
 * @brief Determine if weathervaning (yaw into wind) should be allowed in VTOL modes
 * 
 * @details Controls whether the weathervane feature can actively yaw the aircraft to align
 *          with the wind direction during VTOL hover and low-speed flight. Weathervaning
 *          is disabled during critical transition phases to prevent interference with
 *          automatic attitude control and ensure safe completion of orientation changes.
 *          
 *          Weathervaning Disabled (returns false) When:
 *          
 *          1. **During FW→VTOL Transition**:
 *             - While tailsitter.in_vtol_transition() returns true
 *             - Aircraft is actively pitching up from horizontal to vertical
 *             - Automatic yaw control would interfere with pitch-up maneuver
 *          
 *          2. **During Post-Transition Pitch Rate Limiting**:
 *             - When vtol_limit_start_ms != 0
 *             - Aircraft is rate-limiting pitch changes after reaching vertical
 *             - Attitude still settling to hover configuration
 *          
 *          Weathervaning Enabled (returns true) When:
 *          - Transition is complete (both conditions above are false)
 *          - Aircraft is in stable VTOL hover
 *          - Safe to allow yaw adjustments into wind
 *          
 *          The weathervane feature improves control in wind by aligning the aircraft's
 *          longitudinal axis with the wind direction, reducing the control effort needed
 *          to maintain position. However, it must not interfere with transition maneuvers
 *          where precise pitch control is critical.
 * 
 * @return true if weathervane should be active (safe to yaw into wind)
 * @return false if weathervane should be disabled (during transitions)
 * 
 * @note Called by weathervane controller to determine if yaw corrections are allowed
 * @note Weathervane feature is configured via Q_WVANE_* parameters
 * @warning Allowing weathervane during transitions can cause loss of attitude control
 * 
 * @see tailsitter.in_vtol_transition() for active transition detection
 * @see vtol_limit_start_ms for post-transition rate limiting state
 * @see QuadPlane weathervane controller for wind alignment logic
 * 
 * Source: ArduPlane/tailsitter.cpp:1055-1058
 */
bool Tailsitter_Transition::allow_weathervane()
{
    return !tailsitter.in_vtol_transition() && (vtol_limit_start_ms == 0);
}

#endif  // HAL_QUADPLANE_ENABLED
