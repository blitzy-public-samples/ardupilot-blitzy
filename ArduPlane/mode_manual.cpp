/**
 * @file mode_manual.cpp
 * @brief MANUAL flight mode implementation for ArduPlane
 * 
 * @details This file implements MANUAL flight mode for fixed-wing aircraft,
 *          providing direct RC pass-through to control surfaces without any
 *          stabilization or autonomous control. Pilot has complete manual
 *          control of all flight surfaces and throttle.
 * 
 *          In MANUAL mode:
 *          - Roll/pitch/rudder inputs pass directly to control surfaces
 *          - Expo curves are applied to inputs for smoother feel
 *          - No attitude stabilization or control loops active
 *          - Throttle passes through with optional limits (QuadPlane)
 * 
 * Source: ArduPlane/mode_manual.cpp:1-32
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Execute MANUAL mode control updates
 * 
 * @details Implements direct pass-through of pilot RC inputs to control surfaces
 *          without any stabilization. This function:
 *          - Applies expo curves to roll, pitch, and rudder inputs for smoother control feel
 *          - Passes roll input directly to ailerons via k_aileron channel
 *          - Passes pitch input directly to elevator via k_elevator channel
 *          - Passes rudder input to rudder/steering outputs
 *          - Passes throttle input directly to throttle channel
 *          - Sets nav_roll_cd and nav_pitch_cd to current attitude for telemetry consistency
 * 
 *          No control loops or stabilization are active in this mode. The pilot
 *          has complete manual authority over the aircraft.
 * 
 * @note This function is called at the main loop rate (typically 50Hz for planes)
 * @warning MANUAL mode requires pilot skill - no stabilization is provided
 * 
 * @see plane.roll_in_expo() - Roll input with expo curve
 * @see plane.pitch_in_expo() - Pitch input with expo curve
 * @see plane.rudder_in_expo() - Rudder input with expo curve
 */
void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    output_rudder_and_steering(plane.rudder_in_expo(false));

    const float throttle = plane.get_throttle_input(true);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

    plane.nav_roll_cd = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

/**
 * @brief Main loop execution for MANUAL mode
 * 
 * @details Resets all controllers since MANUAL mode does not use any
 *          stabilization or autonomous control loops. This ensures that
 *          controller integrators and state are cleared when entering
 *          or operating in MANUAL mode, preventing any residual control
 *          authority from affecting the direct pilot inputs.
 * 
 *          Controllers reset include:
 *          - Roll controller
 *          - Pitch controller
 *          - Yaw controller
 *          - Navigation controllers
 * 
 * @note Called once per scheduler loop cycle for mode initialization
 * 
 * @see reset_controllers() - Clears all controller states and integrators
 */
void ModeManual::run()
{
    reset_controllers();
}

/**
 * @brief Determine if throttle min/max limits should be applied in MANUAL mode
 * 
 * @details In standard fixed-wing MANUAL mode, throttle passes through without
 *          limits to give pilots complete control. However, for QuadPlane
 *          configurations, throttle limits can be optionally enforced to prevent
 *          motor damage or unsafe operation.
 * 
 *          Returns true if QuadPlane is available and OPTION::IDLE_GOV_MANUAL
 *          is set, which enables idle governor and throttle limiting even in
 *          MANUAL mode. This prevents VTOL motors from spinning down completely
 *          or exceeding safe limits during manual fixed-wing flight.
 * 
 *          For pure fixed-wing aircraft (non-QuadPlane), always returns false
 *          to allow unrestricted throttle pass-through.
 * 
 * @return true if throttle limits should be enforced (QuadPlane with IDLE_GOV_MANUAL)
 * @return false for direct throttle pass-through without limits (standard fixed-wing)
 * 
 * @note QuadPlane OPTION::IDLE_GOV_MANUAL must be explicitly enabled for limits
 * @warning Disabling limits on QuadPlane could allow motors to stop in flight
 * 
 * @see QuadPlane::OPTION::IDLE_GOV_MANUAL - Parameter controlling this behavior
 */
bool ModeManual::use_throttle_limits() const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.option_is_set(QuadPlane::OPTION::IDLE_GOV_MANUAL)) {
        return true;
    }
#endif
    return false;
}
