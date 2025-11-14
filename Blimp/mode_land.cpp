/**
 * @file mode_land.cpp
 * @brief Land mode implementation for Blimp airship landing
 * 
 * @details This file implements the LAND flight mode for lighter-than-air vehicles (blimps).
 *          Unlike heavier-than-air vehicles that require active descent control, blimp landing
 *          is achieved by simply stopping all motor outputs and allowing the vehicle's natural
 *          buoyancy to gently settle it to the ground. This mode can be entered manually or
 *          automatically via failsafe mechanisms.
 * 
 *          The landing behavior leverages the inherent stability of buoyant vehicles - when
 *          all thrust is removed, the blimp naturally descends or hovers safely depending on
 *          buoyancy configuration, making this an inherently safe mode for airships.
 * 
 * @note This mode is unique to lighter-than-air vehicles and differs significantly from
 *       multicopter or fixed-wing landing modes which require active altitude control.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Blimp.h"

/**
 * @brief Execute the land mode controller for airship landing
 * 
 * @details This function implements the landing algorithm for lighter-than-air vehicles by
 *          immediately zeroing all motor outputs across all control axes:
 *          - right_out: Lateral (left/right) thrust set to 0
 *          - front_out: Longitudinal (forward/back) thrust set to 0
 *          - yaw_out: Yaw (rotation) control set to 0
 *          - down_out: Vertical (up/down) thrust set to 0
 *          
 *          With all thrust removed, the blimp's inherent buoyancy characteristics determine
 *          the landing behavior. Properly configured blimps will gently settle to the ground
 *          or hover in a stable equilibrium. This passive approach is safer for airships than
 *          active descent control since there's no risk of excessive descent rates or impact.
 *          
 *          The simplicity of this implementation (zero all outputs) makes it an ideal failsafe
 *          mode - there are no complex control algorithms that could malfunction, and the
 *          vehicle naturally returns to a safe, low-energy state.
 * 
 * @note This function is called at the main loop rate (typically 50Hz for blimps) while in
 *       LAND mode. The continuous zeroing of outputs ensures that even if other systems
 *       attempt to write motor commands, they are immediately overridden.
 * 
 * @note Unlike multicopter or plane landing modes, no altitude monitoring or touchdown
 *       detection is performed - the pilot or ground crew determines when landing is complete.
 * 
 * @see Blimp::set_mode_land_failsafe() for failsafe entry into this mode
 * @see motors->output() for where these output values are applied to physical motors
 */
void ModeLand::run()
{
    //Stop moving
    motors->right_out = 0;
    motors->front_out = 0;
    motors->yaw_out = 0;
    motors->down_out = 0;
}

/**
 * @brief Initiate failsafe transition to LAND mode for airship safety
 * 
 * @details This function provides a dedicated entry point for transitioning to LAND mode
 *          specifically from failsafe conditions. It performs two critical operations:
 *          
 *          1. Calls set_mode() to transition to Mode::Number::LAND, passing through the
 *             reason parameter for mode change tracking and logging
 *          2. Sets the failsafe_mode_change event flag to trigger pilot notification
 *             through the AP_Notify system (buzzers, LEDs, GCS messages)
 *          
 *          Failsafe scenarios that may trigger this function include:
 *          - Radio control signal loss
 *          - Ground control station communication timeout
 *          - Battery voltage critically low
 *          - Geofence breach with configured LAND action
 *          - Internal system errors requiring safe mode
 *          
 *          The LAND mode is particularly safe for airships because it simply stops all thrust,
 *          allowing the vehicle's buoyancy to provide a gentle, controlled descent or hover.
 *          This passive safety mechanism reduces the risk of catastrophic failures compared
 *          to active failsafe modes.
 * 
 * @param[in] reason The ModeReason enum value indicating which failsafe condition triggered
 *                   the mode change. This is logged and used for post-flight analysis.
 *                   Common values: ModeReason::RADIO_FAILSAFE, ModeReason::BATTERY_FAILSAFE,
 *                   ModeReason::GCS_FAILSAFE, ModeReason::FENCE_BREACHED
 * 
 * @warning SAFETY-CRITICAL FUNCTION: This is a failsafe entry point that must execute
 *          reliably under all conditions including degraded system states. The function
 *          must never throw exceptions, block, or fail to transition to LAND mode.
 *          Any modifications must preserve guaranteed execution and mode transition.
 * 
 * @note The AP_Notify::events.failsafe_mode_change flag triggers immediate pilot alerts
 *       through all available notification mechanisms (buzzer patterns, LED flashing,
 *       GCS warning messages) to ensure the pilot is aware of the failsafe activation.
 * 
 * @note This function is always called from failsafe detection logic, never from normal
 *       pilot-commanded mode changes. For manual LAND mode selection, use set_mode()
 *       directly without setting the failsafe notification.
 * 
 * @see ModeLand::run() for the LAND mode implementation that executes after this transition
 * @see set_mode() for the underlying mode change mechanism
 * @see AP_Notify::events for notification event definitions
 */
void Blimp::set_mode_land_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}