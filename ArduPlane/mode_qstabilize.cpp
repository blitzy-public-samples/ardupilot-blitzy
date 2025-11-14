/**
 * @file mode_qstabilize.cpp
 * @brief QSTABILIZE flight mode implementation for QuadPlane VTOL operations
 * 
 * @details This file implements QSTABILIZE mode, which provides attitude stabilization
 *          for QuadPlane vehicles using multicopter-style control. QSTABILIZE allows
 *          direct pilot control of vehicle attitude (roll/pitch) and throttle while
 *          the autopilot stabilizes the aircraft using multicopter control algorithms.
 *          
 *          QSTABILIZE mode is typically used for:
 *          - Manual VTOL flight with stabilization assistance
 *          - Pilot training and familiarization with QuadPlane behavior
 *          - Testing and tuning of multicopter attitude controllers
 *          - ESC calibration procedures
 *          
 *          Mode Behavior:
 *          - Pilot stick inputs directly control target roll and pitch angles
 *          - Throttle controls vertical thrust (not altitude hold)
 *          - Yaw control via rudder (centered in normal operation)
 *          - Fixed-wing control surfaces provide additional stabilization
 *          - Supports both standard quadplanes and tailsitter configurations
 *          
 *          Special Handling:
 *          - Tailsitters: Uses tailsitter-specific angle limits and control mixing
 *          - Forward flight limits: Optionally respects fixed-wing angle constraints
 *          - VTOL transitions: Tailsitters in FW pull-up use fixed-wing controllers
 *          - ESC calibration: Special mode for calibrating electronic speed controllers
 *          
 * @note This mode requires HAL_QUADPLANE_ENABLED to be defined
 * @warning QSTABILIZE does not provide altitude hold - pilot must manage throttle
 * 
 * @see Mode base class for common flight mode interface
 * @see QuadPlane class for multicopter control implementation
 */

#include "mode.h"
#include "Plane.h"

/**
 * @def HAL_QUADPLANE_ENABLED
 * @brief Conditional compilation flag for QuadPlane functionality
 * 
 * @details This preprocessor definition controls whether QuadPlane (VTOL hybrid)
 *          code is compiled into the firmware. When undefined, all QuadPlane modes
 *          including QSTABILIZE are excluded, reducing firmware size for pure
 *          fixed-wing configurations.
 *          
 *          Defined by: Build system based on board capabilities and feature flags
 *          Scope: All QuadPlane-related modes and libraries
 */
#if HAL_QUADPLANE_ENABLED

/**
 * @brief Enter QSTABILIZE mode
 * 
 * @details Called when the vehicle transitions into QSTABILIZE mode from another
 *          flight mode. Initializes mode-specific state to prepare for manual
 *          attitude-stabilized flight.
 *          
 *          Initialization Actions:
 *          - Disables throttle_wait to allow immediate throttle response
 *          - Returns success (mode entry always succeeds for QSTABILIZE)
 *          
 *          The throttle_wait flag is used in some modes to prevent throttle
 *          response until the pilot moves the throttle stick to a safe position.
 *          In QSTABILIZE, immediate throttle response is desired for manual control.
 * 
 * @return true Always returns true - QSTABILIZE mode entry always succeeds
 * 
 * @note Called by mode management system during mode transitions
 * @note This is an override of the Mode base class _enter() method
 * 
 * @see Mode::_enter() for base class interface
 * @see QuadPlane::throttle_wait for throttle arming behavior
 */
bool ModeQStabilize::_enter()
{
    quadplane.throttle_wait = false;
    return true;
}

/**
 * @brief Update target attitude from pilot stick inputs
 * 
 * @details This method is called during the navigation update phase to convert pilot
 *          RC stick inputs into target attitude angles (roll and pitch) for the
 *          attitude controller. The method handles different vehicle configurations
 *          and applies appropriate angle limits.
 *          
 *          Processing Steps:
 *          1. Read pilot roll and pitch inputs from RC channels
 *          2. Normalize inputs to range [-1.0, 1.0]
 *          3. Apply vehicle-specific scaling and limits:
 *             - Tailsitters: Use tailsitter-specific angle limits
 *             - Standard quadplanes: Apply either FW limits or Q_ANGLE_MAX
 *          4. Set nav_roll_cd and nav_pitch_cd for attitude controller
 *          
 *          Input Processing:
 *          - Uses get_control_in() rather than norm_input for tailsitter compatibility
 *          - Tailsitter input mixing may alter control_in values before this method
 *          - Input normalization: control_in / channel_range → [-1.0, 1.0]
 *          
 *          Angle Limit Modes:
 *          - Tailsitter: Q_TAILSIT_ANG_MAX (roll), Q_ANGLE_MAX (pitch)
 *          - Limited mode: MIN(ROLL_LIMIT_CD, Q_ANGLE_MAX) and PTCH_LIM_MIN/MAX
 *          - Unlimited mode: Q_ANGLE_MAX for both axes (if INGORE_FW_ANGLE_LIMITS set)
 * 
 * @note Called at navigation update rate (typically 50Hz)
 * @note Sets plane.nav_roll_cd and plane.nav_pitch_cd in centidegrees
 * @note Tailsitter input may be pre-processed by QuadPlane::tailsitter_check_input
 * 
 * @warning Tailsitters require get_control_in() instead of norm_input due to
 *          input mixing that occurs in Plane::read_radio()
 * 
 * @see set_tailsitter_roll_pitch() for tailsitter-specific angle calculation
 * @see set_limited_roll_pitch() for forward-flight-limited angle calculation
 * @see QuadPlane::tailsitter_check_input() for tailsitter input mixing
 */
void ModeQStabilize::update()
{
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    const float roll_input = (float)plane.channel_roll->get_control_in() / plane.channel_roll->get_range();
    const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();

    // then scale to target angles in centidegrees
    if (plane.quadplane.tailsitter.active()) {
        // tailsitters are different
        set_tailsitter_roll_pitch(roll_input, pitch_input);
        return;
    }

    if (!plane.quadplane.option_is_set(QuadPlane::OPTION::INGORE_FW_ANGLE_LIMITS_IN_Q_MODES)) {
        // by default angles are also constrained by forward flight limits
        set_limited_roll_pitch(roll_input, pitch_input);
    } else {
        // use angle max for both roll and pitch
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    }
}

/**
 * @brief Execute QSTABILIZE mode control loop
 * 
 * @details This is the main execution method for QSTABILIZE mode, called at the
 *          main loop rate (typically 400Hz for multicopter control). It coordinates
 *          multicopter attitude control with fixed-wing surface stabilization to
 *          provide manual attitude-stabilized flight.
 *          
 *          Execution Flow:
 *          1. Check for special conditions (VTOL transition, ESC calibration)
 *          2. Assign tilt servos to forward throttle if applicable
 *          3. Get pilot throttle input
 *          4. Run multicopter attitude stabilization (hold_stabilize)
 *          5. Run fixed-wing surface stabilization
 *          6. Center rudder output
 *          
 *          Special Conditions Handled:
 *          
 *          VTOL Transition (Tailsitters):
 *          - During fixed-wing pull-up phase, delegates to fixed-wing controllers
 *          - Ensures smooth transition from VTOL to forward flight
 *          - Uses Mode::run() to execute fixed-wing control path
 *          
 *          ESC Calibration Mode:
 *          - Activated when quadplane.esc_calibration is non-zero
 *          - Runs special ESC calibration sequence (see run_esc_calibration)
 *          - Still stabilizes fixed-wing surfaces for safety
 *          - Bypasses normal throttle and attitude control
 *          
 *          Normal Operation:
 *          - Reads pilot throttle (scaled and optionally filtered)
 *          - Calls hold_stabilize() for multicopter attitude control
 *          - Stabilizes fixed-wing roll and pitch surfaces
 *          - Centers rudder (yaw control via multicopter motors)
 *          
 *          Control Integration:
 *          - Multicopter motors: Provide primary attitude control
 *          - Fixed-wing surfaces: Provide supplemental stabilization
 *          - Tilt servos: Assigned to forward thrust if configured
 *          - Rudder: Centered (not used in normal QSTABILIZE)
 * 
 * @note Called at main loop rate (typically 400Hz)
 * @note Sets motor outputs and control surface positions
 * @note Requires valid pilot input on roll, pitch, and throttle channels
 * 
 * @warning This is a safety-critical method - improper execution can result in
 *          loss of control. All control paths must maintain vehicle stabilization.
 * @warning ESC calibration mode should only be used on the ground with propellers
 *          removed or secured, as it bypasses normal safety checks.
 * 
 * @see QuadPlane::hold_stabilize() for multicopter attitude control implementation
 * @see Plane::stabilize_roll() for fixed-wing roll stabilization
 * @see Plane::stabilize_pitch() for fixed-wing pitch stabilization
 * @see QuadPlane::run_esc_calibration() for ESC calibration procedure
 * @see QuadPlane::Tailsitter::in_vtol_transition() for transition state checking
 */
void ModeQStabilize::run()
{
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        // Tailsitters in FW pull up phase of VTOL transition run FW controllers
        Mode::run();
        return;
    }

    plane.quadplane.assign_tilt_to_fwd_thr();

    // special check for ESC calibration in QSTABILIZE
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = quadplane.get_pilot_throttle();
    quadplane.hold_stabilize(pilot_throttle_scaled);

    // Stabilize with fixed wing surfaces
    plane.stabilize_roll();
    plane.stabilize_pitch();

    // Center rudder
    output_rudder_and_steering(0.0);
}

/**
 * @brief Set desired roll and pitch angles for tailsitter configurations
 * 
 * @details Calculates target attitude angles for tailsitter vehicles from normalized
 *          pilot inputs, applying tailsitter-specific angle limits. Tailsitters have
 *          unique handling requirements due to their vertical takeoff/landing orientation
 *          and may use different roll and pitch angle limits than standard quadplanes.
 *          
 *          Roll Angle Calculation:
 *          - If Q_TAILSIT_ANG_MAX > 0: Uses tailsitter-specific roll limit
 *          - Otherwise: Uses Q_ANGLE_MAX for roll
 *          - Note: Q_TAILSIT_ANG_MAX is in degrees, converted to centidegrees
 *          
 *          Pitch Angle Calculation:
 *          - Always uses Q_ANGLE_MAX for pitch
 *          - No separate tailsitter pitch limit parameter
 *          
 *          Transition Limiting:
 *          - Calls set_VTOL_roll_pitch_limit() to apply transition-specific constraints
 *          - Ensures attitude targets are achievable during VTOL transitions
 *          - May further reduce limits based on transition phase
 * 
 * @param[in] roll_input  Normalized pilot roll input in range [-1.0, 1.0]
 * @param[in] pitch_input Normalized pilot pitch input in range [-1.0, 1.0]
 * 
 * @note Sets plane.nav_roll_cd and plane.nav_pitch_cd in centidegrees
 * @note Called only when plane.quadplane.tailsitter.active() returns true
 * @note Q_TAILSIT_ANG_MAX is in degrees, Q_ANGLE_MAX is in centidegrees
 * 
 * @warning Tailsitter attitude limits must be configured appropriately for vehicle
 *          dynamics - excessive angles can lead to loss of control during transitions
 * 
 * @see QuadPlane::Tailsitter for tailsitter configuration and state
 * @see QuadPlane::Transition::set_VTOL_roll_pitch_limit() for transition limiting
 * @see update() for the method that calls this with normalized inputs
 */
void ModeQStabilize::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plane.nav_roll_cd = plane.quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

/**
 * @brief Set desired roll and pitch angles with fixed-wing flight limits
 * 
 * @details Calculates target attitude angles for standard quadplane vehicles from
 *          normalized pilot inputs, applying the more restrictive of either the
 *          multicopter angle limits (Q_ANGLE_MAX) or the fixed-wing flight envelope
 *          limits (ROLL_LIMIT_CD, PTCH_LIM_MIN/MAX). This ensures attitude commands
 *          remain within both VTOL control capability and aerodynamic constraints.
 *          
 *          Roll Angle Calculation:
 *          - Uses minimum of ROLL_LIMIT_CD and Q_ANGLE_MAX
 *          - Symmetric limits for positive and negative roll
 *          - ROLL_LIMIT_CD: Maximum bank angle for fixed-wing flight
 *          - Q_ANGLE_MAX: Maximum lean angle for multicopter control
 *          
 *          Pitch Angle Calculation:
 *          - Asymmetric limits: PTCH_LIM_MAX (nose up) and PTCH_LIM_MIN (nose down)
 *          - Positive input: Uses minimum of PTCH_LIM_MAX*100 and Q_ANGLE_MAX
 *          - Negative input: Uses minimum of -PTCH_LIM_MIN*100 and Q_ANGLE_MAX
 *          - Note: PTCH_LIM_MIN/MAX are in degrees, converted to centidegrees
 *          
 *          Rationale for Dual Limits:
 *          - Fixed-wing limits prevent excessive angles that reduce lift
 *          - Multicopter limits prevent exceeding control authority
 *          - Using minimum ensures vehicle stays within both envelopes
 *          - Important for vehicles that transition between VTOL and forward flight
 *          
 *          Parameter Interaction:
 *          - ROLL_LIMIT_CD: Fixed-wing maximum bank angle (centidegrees)
 *          - PTCH_LIM_MAX: Fixed-wing maximum pitch up angle (degrees)
 *          - PTCH_LIM_MIN: Fixed-wing maximum pitch down angle (degrees, negative)
 *          - Q_ANGLE_MAX: Multicopter maximum lean angle (centidegrees)
 * 
 * @param[in] roll_input  Normalized pilot roll input in range [-1.0, 1.0]
 * @param[in] pitch_input Normalized pilot pitch input in range [-1.0, 1.0]
 * 
 * @note Sets plane.nav_roll_cd and plane.nav_pitch_cd in centidegrees
 * @note Called when INGORE_FW_ANGLE_LIMITS_IN_Q_MODES option is NOT set
 * @note Pitch limits are asymmetric - nose up and nose down may have different limits
 * @note Unit conversion: PTCH_LIM parameters (degrees) multiplied by 100 for centidegrees
 * 
 * @warning Restrictive angle limits may reduce agility in QSTABILIZE mode but are
 *          necessary for safe transition to forward flight if initiated from VTOL
 * 
 * @see plane.roll_limit_cd for fixed-wing roll limit source
 * @see plane.aparm.pitch_limit_max and pitch_limit_min for pitch limit sources
 * @see plane.quadplane.aparm.angle_max for multicopter angle limit
 * @see update() for the method that calls this with normalized inputs
 */
void ModeQStabilize::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = roll_input * MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    // pitch is further constrained by PTCH_LIM_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max*100, plane.quadplane.aparm.angle_max);
    } else {
        plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min*100, plane.quadplane.aparm.angle_max);
    }
}

#endif
