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
 * @file reverse_thrust.cpp
 * @brief Reverse thrust support functions for fixed-wing aircraft braking and speed control
 * 
 * @details This module implements reverse thrust functionality for ArduPlane, allowing aircraft
 *          to use negative throttle values for braking during landing, slow-down in loiter,
 *          and controlled deceleration. Reverse thrust can be selectively enabled per flight
 *          mode through the USE_REV_THRUST parameter bitmask.
 * 
 * Architecture:
 * - Configuration: Reverse thrust is enabled when THROTTLE_MIN < 0
 * - Permission System: USE_REV_THRUST bitmask controls which modes allow reverse thrust
 * - Safety Interlocks: Reverse thrust is prohibited during takeoff to prevent accidents
 * - RC Integration: Supports RC channel reversal via reversed_throttle flag
 * 
 * Typical Use Cases:
 * - Landing approach: Aerodynamic braking to reduce landing distance
 * - Loiter speed control: Maintain slow speeds in circling patterns
 * - Waypoint precision: Fine speed control for accurate waypoint arrival
 * - Manual braking: Pilot-commanded deceleration in stabilized modes
 * 
 * Configuration Parameters:
 * - THROTTLE_MIN: Must be < 0 to enable reverse thrust (e.g., -25 for 25% reverse)
 * - USE_REV_THRUST: Bitmask parameter controlling reverse thrust availability by mode
 * - TRIM_THROTTLE: Center-stick value when using adjusted throttle input with curves
 * 
 * Safety Considerations:
 * - Reverse thrust is NEVER allowed during takeoff (MAV_CMD_NAV_TAKEOFF)
 * - Mode-specific enablement prevents unintended activation
 * - Requires valid RC input; returns 0% throttle if RC signal lost
 * - Throttle reversal logic applies to both normal and reverse thrust ranges
 * 
 * @warning Incorrect reverse thrust configuration can cause loss of control during takeoff
 *          or unexpected braking behavior. Always test in SITL before flight.
 * 
 * @note Reverse thrust effectiveness depends on aircraft type. Props are more effective
 *       than ducted fans. Not all aircraft support reverse thrust safely.
 * 
 * @see Plane::have_reverse_thrust()
 * @see Plane::allow_reverse_thrust()
 * @see Parameters::throttle_min (aparm.throttle_min)
 * @see Parameters::use_reverse_thrust (g.use_reverse_thrust)
 */
#include "Plane.h"

/**
 * @brief Check if a specific reverse thrust option is enabled in USE_REV_THRUST bitmask
 * 
 * @details This function tests the USE_REV_THRUST parameter bitmask to determine if a
 *          specific reverse thrust option is enabled. The USE_REV_THRUST parameter is
 *          a bitmask where each bit corresponds to a different flight mode or condition
 *          where reverse thrust should be allowed.
 * 
 *          The bitmask allows fine-grained control over reverse thrust availability,
 *          enabling it only in specific situations while keeping it disabled elsewhere
 *          for safety.
 * 
 * @param[in] use_reverse_thrust_option The specific UseReverseThrust enum value to test
 *                                      (e.g., AUTO_ALWAYS, AUTO_LAND_APPROACH, LOITER)
 * 
 * @return true if the specified option bit is set in USE_REV_THRUST parameter
 * @return false if the specified option bit is not set
 * 
 * @note This is a bitwise test - multiple options can be enabled simultaneously
 * @note Called frequently during flight control loops to check thrust permissions
 * 
 * @see allow_reverse_thrust() for complete mode-based reverse thrust logic
 * @see UseReverseThrust enum for available option flags
 * 
 * Source: ArduPlane/reverse_thrust.cpp:21-24
 */
bool Plane::reverse_thrust_enabled(UseReverseThrust use_reverse_thrust_option) const
{
    return (g.use_reverse_thrust.get() & int32_t(use_reverse_thrust_option)) != 0;
}

/**
 * @brief Determine if reverse thrust should be allowed in the current flight state
 * 
 * @details This function implements the core safety interlock system for reverse thrust,
 *          checking both configuration status and mode-specific permissions. It combines
 *          multiple safety checks to prevent reverse thrust activation in unsafe conditions.
 * 
 *          Safety Interlock Hierarchy:
 *          1. Configuration check: THROTTLE_MIN must be < 0 (have_reverse_thrust())
 *          2. Mode-based permission: Current mode must have reverse thrust enabled in USE_REV_THRUST
 *          3. Command-based prohibition: Explicit blocks (e.g., MAV_CMD_NAV_TAKEOFF)
 * 
 *          Mode-Specific Behavior:
 *          - AUTO: Fine-grained control based on mission command type and USE_REV_THRUST bitmask
 *                  Never allowed during takeoff commands (critical safety interlock)
 *          - TAKEOFF: Always prohibited (explicit safety measure)
 *          - Other Modes: Allowed based on USE_REV_THRUST bitmask setting for that mode
 *          - MANUAL and unspecified modes: Allowed by default if configured
 * 
 *          The bitmask approach allows operators to enable reverse thrust selectively,
 *          such as only during landing approaches or loitering, while keeping it disabled
 *          during cruise or waypoint navigation if desired.
 * 
 * @return true if reverse thrust is both configured and permitted in current flight state
 * @return false if reverse thrust is not configured, or not permitted in current mode/command
 * 
 * @note Called from throttle control loops to gate reverse thrust application
 * @note Takeoff safety: Returns false for MAV_CMD_NAV_TAKEOFF and Mode::Number::TAKEOFF
 * 
 * @warning This function is safety-critical. Never allow reverse thrust during takeoff
 *          as it can cause immediate loss of control and crash.
 * 
 * @see have_reverse_thrust() for configuration check
 * @see reverse_thrust_enabled() for bitmask testing
 * @see UseReverseThrust enum for available mode flags
 * 
 * Source: ArduPlane/reverse_thrust.cpp:29-119
 */
bool Plane::allow_reverse_thrust(void) const
{
    // Safety Interlock 1: Configuration check
    // Reverse thrust is not configured, so it should never be allowed
    if (!have_reverse_thrust()) {
        return false;
    }

    // check if we should allow reverse thrust based on current mode
    bool allow = false;

    switch (control_mode->mode_number()) {
    case Mode::Number::AUTO:
        {
        // Get current navigation command from active mission
        uint16_t nav_cmd = mission.get_current_nav_cmd().id;

        // Safety Interlock 2: CRITICAL - Never allow reverse thrust during takeoff
        // Applying reverse thrust during takeoff would cause immediate loss of lift
        // and catastrophic crash. This is a hard safety block that overrides all
        // bitmask settings.
        if (nav_cmd == MAV_CMD_NAV_TAKEOFF) {
            return false;
        }

        // Check USE_REV_THRUST bitmask for AUTO mode permissions
        // Multiple conditions can be true simultaneously (bitwise OR logic)

        // AUTO_ALWAYS: Allow reverse thrust for all mission commands except takeoff
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_ALWAYS);

        // AUTO_LAND_APPROACH: Allow during MAV_CMD_NAV_LAND for landing braking
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LAND_APPROACH) &&
                (nav_cmd == MAV_CMD_NAV_LAND);

        // AUTO_LOITER_TO_ALT: Allow during altitude-targeted loitering (descending circles)
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LOITER_TO_ALT) &&
                (nav_cmd == MAV_CMD_NAV_LOITER_TO_ALT);

        // AUTO_LOITER_ALL: Allow during any loiter command type for speed control
        // Useful for maintaining slow speeds in soaring or observation patterns
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LOITER_ALL) &&
                    (nav_cmd == MAV_CMD_NAV_LOITER_TIME ||
                     nav_cmd == MAV_CMD_NAV_LOITER_TO_ALT ||
                     nav_cmd == MAV_CMD_NAV_LOITER_TURNS ||
                     nav_cmd == MAV_CMD_NAV_LOITER_UNLIM);

        // AUTO_WAYPOINT: Allow during waypoint navigation for precise speed control
        // Both straight and curved (spline) waypoints supported
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_WAYPOINT) &&
                    (nav_cmd == MAV_CMD_NAV_WAYPOINT ||
                     nav_cmd == MAV_CMD_NAV_SPLINE_WAYPOINT);

        // AUTO_LANDING_PATTERN: Allow during entire landing sequence (approach + land)
        // Broader than AUTO_LAND_APPROACH - covers all landing sequence waypoints
        allow |= reverse_thrust_enabled(UseReverseThrust::AUTO_LANDING_PATTERN) &&
                mission.get_in_landing_sequence_flag();
        }
        break;

    // Non-AUTO modes: Check USE_REV_THRUST bitmask for mode-specific permission
    
    case Mode::Number::LOITER:
        // Allow in LOITER mode for slow-speed circling
        allow |= reverse_thrust_enabled(UseReverseThrust::LOITER);
        break;
    case Mode::Number::RTL:
        // Allow during Return-To-Launch for approach speed control
        allow |= reverse_thrust_enabled(UseReverseThrust::RTL);
        break;
    case Mode::Number::CIRCLE:
        // Allow in CIRCLE mode for maintaining circle radius at low speeds
        allow |= reverse_thrust_enabled(UseReverseThrust::CIRCLE);
        break;
    case Mode::Number::CRUISE:
        // Allow in CRUISE mode for speed adjustment
        allow |= reverse_thrust_enabled(UseReverseThrust::CRUISE);
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        // Allow in FBWB for pilot-commanded speed control
        allow |= reverse_thrust_enabled(UseReverseThrust::FBWB);
        break;
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
        // Allow in GUIDED and collision avoidance modes for GCS/companion computer control
        allow |= reverse_thrust_enabled(UseReverseThrust::GUIDED);
        break;
    case Mode::Number::TAKEOFF:
        // Safety Interlock 3: CRITICAL - Explicit takeoff mode prohibition
        // Never allow reverse thrust in TAKEOFF mode (independent of bitmask)
        allow = false;
        break;
    case Mode::Number::FLY_BY_WIRE_A:
        // Allow in FBWA for pilot-commanded speed control with stabilization
        allow |= reverse_thrust_enabled(UseReverseThrust::FBWA);
        break;
    case Mode::Number::ACRO:
        // Allow in ACRO mode for advanced aerobatics requiring thrust reversal
        allow |= reverse_thrust_enabled(UseReverseThrust::ACRO);
        break;
    case Mode::Number::STABILIZE:
        // Allow in STABILIZE mode for direct pilot control with attitude stabilization
        allow |= reverse_thrust_enabled(UseReverseThrust::STABILIZE);
        break;
    case Mode::Number::THERMAL:
        // Allow in THERMAL mode for slow-speed thermaling
        allow |= reverse_thrust_enabled(UseReverseThrust::THERMAL);
        break;
    default:
        // Default: Allow for all other modes (primarily MANUAL)
        // MANUAL mode gets reverse thrust if configured, independent of bitmask
        // This allows direct pilot control without restrictions
        allow = true;
        break;
    }

    return allow;
}

/**
 * @brief Check if reverse thrust is configured in vehicle parameters
 * 
 * @details This function determines if the vehicle is configured to support reverse thrust
 *          by checking the THROTTLE_MIN parameter. Reverse thrust is enabled when
 *          THROTTLE_MIN is set to a negative value, which allows the throttle output
 *          to command negative thrust (braking).
 * 
 *          Configuration Example:
 *          - THROTTLE_MIN = 0: No reverse thrust (normal forward-only operation)
 *          - THROTTLE_MIN = -25: Reverse thrust enabled, up to 25% negative thrust
 *          - THROTTLE_MIN = -50: Reverse thrust enabled, up to 50% negative thrust
 * 
 *          This is the foundational configuration check. Even if USE_REV_THRUST bitmask
 *          enables reverse thrust for certain modes, it will not be available unless
 *          THROTTLE_MIN < 0.
 * 
 * @return true if THROTTLE_MIN parameter is negative (reverse thrust configured)
 * @return false if THROTTLE_MIN parameter is zero or positive (no reverse thrust)
 * 
 * @note This check is performed before any mode-specific reverse thrust permissions
 * @note Called by allow_reverse_thrust() as the first safety interlock
 * 
 * @see allow_reverse_thrust() for complete reverse thrust permission logic
 * @see Parameters::throttle_min (aparm.throttle_min)
 * 
 * Source: ArduPlane/reverse_thrust.cpp:124-127
 */
bool Plane::have_reverse_thrust(void) const
{
    return aparm.throttle_min < 0;
}

/**
 * @brief Get throttle input from RC channel with optional reversal and deadzone handling
 * 
 * @details This function retrieves the pilot's throttle command from the RC receiver,
 *          applying optional deadzone filtering and throttle reversal. It serves as
 *          the primary interface for reading throttle stick position.
 * 
 *          Signal Flow:
 *          1. RC receiver → channel_throttle (raw PWM input)
 *          2. PWM → control_in value (typically -100 to +100 range)
 *          3. Optional deadzone filtering (if no_deadzone = false)
 *          4. Optional reversal (if reversed_throttle flag set via RC option)
 *          5. Return final throttle command
 * 
 *          Throttle Reversal Logic:
 *          The reversed_throttle flag (set via RC option switch) inverts the throttle
 *          input direction. This is independent of reverse thrust configuration and
 *          applies to the RC input interpretation itself. When combined with reverse
 *          thrust support, this allows flexible pilot interface configurations:
 *          
 *          - Normal: Stick forward = forward thrust, stick back = reverse thrust
 *          - Reversed: Stick back = forward thrust, stick forward = reverse thrust
 * 
 *          Deadzone Handling:
 *          - no_deadzone = false: Applies configured deadzone around center stick
 *                                 (small movements near center ignored for stability)
 *          - no_deadzone = true: Raw input without deadzone filtering
 *                                (used for precise control requirements)
 * 
 *          Safety: RC Signal Loss
 *          If RC signal is lost or invalid, this function returns 0.0 (neutral throttle)
 *          to prevent unexpected throttle commands from stale data.
 * 
 * @param[in] no_deadzone If true, disables deadzone filtering for precise control.
 *                        If false, applies configured deadzone around center stick.
 * 
 * @return Throttle input value (typically -100 to +100 range, units: percentage)
 *         Returns 0.0 if RC signal is invalid (safety feature)
 * 
 * @note This function does NOT enforce reverse thrust permissions - it only reads
 *       the RC input. Use allow_reverse_thrust() to check if reverse is permitted.
 * @note The reversed_throttle flag is set via RC options, not parameters
 * @note Called at main loop rate (typically 50-400 Hz depending on vehicle)
 * 
 * @warning Never assume RC input is valid - always returns 0.0 on signal loss
 * 
 * @see get_adjusted_throttle_input() for throttle input with exponential curve
 * @see allow_reverse_thrust() for reverse thrust permission checks
 * @see RC_Channel::get_control_in() for raw RC input retrieval
 * 
 * Source: ArduPlane/reverse_thrust.cpp:132-149
 */
float Plane::get_throttle_input(bool no_deadzone) const
{
    // Safety check: Return neutral throttle if RC signal is lost or invalid
    if (!rc().has_valid_input()) {
        // Return 0 if there is no valid input
        return 0.0;
    }
    
    // Retrieve RC throttle channel input with or without deadzone filtering
    float ret;
    if (no_deadzone) {
        // Get raw control input without deadzone (zero deadzone)
        ret = channel_throttle->get_control_in_zero_dz();
    } else {
        // Get control input with configured deadzone applied
        ret = channel_throttle->get_control_in();
    }
    
    // Apply throttle reversal if RC option switch is enabled
    // This inverts the stick direction (forward↔back) independent of reverse thrust config
    if (reversed_throttle) {
        // RC option for reverse throttle has been set
        ret = -ret;
    }
    
    return ret;
}

/**
 * @brief Get throttle input with exponential curve mapping center stick to TRIM_THROTTLE
 * 
 * @details This function provides an alternative throttle input method that applies an
 *          exponential curve to map the center stick position to TRIM_THROTTLE (cruise
 *          throttle). This makes it easier for pilots to maintain cruise speed as the
 *          stick naturally rests near center.
 * 
 *          Curve Mapping:
 *          - Stick at bottom: Minimum throttle (or reverse thrust if configured)
 *          - Stick at center: TRIM_THROTTLE (cruise power, typically 50-75%)
 *          - Stick at top: Maximum throttle (100%)
 * 
 *          This differs from linear throttle input where:
 *          - Center stick = 50% throttle (often less than cruise requires)
 *          - Pilot must hold stick offset from center to maintain cruise
 * 
 *          With adjusted input:
 *          - Center stick = cruise throttle (comfortable for long flights)
 *          - Easier to maintain constant speed without constant stick attention
 * 
 *          Activation Requirements:
 *          This curve is only applied when BOTH conditions are met:
 *          1. Throttle channel is configured as RANGE type (not ANGLE)
 *          2. CENTER_THROTTLE_TRIM flight option is enabled
 * 
 *          If either condition is false, falls back to standard get_throttle_input().
 * 
 *          Throttle Reversal Integration:
 *          The reversed_throttle flag is applied AFTER the curve calculation, so the
 *          curve mapping is preserved even when the stick direction is reversed.
 * 
 * @param[in] no_deadzone If true, uses zero deadzone input for curve calculation.
 *                        If false, applies deadzone (passed to get_throttle_input fallback).
 * 
 * @return Adjusted throttle input value (range scaled, units: percentage)
 *         Returns 0.0 if RC signal is invalid (safety feature)
 * 
 * @note Falls back to get_throttle_input() if curve conditions not met
 * @note The throttle_curve() function applies exponential mapping
 * @note TRIM_THROTTLE (aparm.throttle_cruise) must be properly configured for
 *       optimal curve performance
 * @note Called at main loop rate in modes using adjusted throttle input
 * 
 * @warning Incorrect TRIM_THROTTLE setting will result in poor throttle feel
 * @warning Only use with RANGE type RC channels, not ANGLE channels
 * 
 * @see get_throttle_input() for standard linear throttle input
 * @see throttle_curve() for exponential curve calculation
 * @see FlightOptions::CENTER_THROTTLE_TRIM for enabling this feature
 * @see Parameters::throttle_cruise (aparm.throttle_cruise) for cruise throttle setting
 * 
 * Source: ArduPlane/reverse_thrust.cpp:154-170
 */
float Plane::get_adjusted_throttle_input(bool no_deadzone) const
{
    // Safety check: Return neutral throttle if RC signal is lost or invalid
    if (!rc().has_valid_input()) {
        // Return 0 if there is no valid input
        return 0.0;
    }
    
    // Check if throttle curve should be applied
    // Requires: RANGE channel type AND CENTER_THROTTLE_TRIM flight option enabled
    if ((plane.channel_throttle->get_type() != RC_Channel::ControlType::RANGE) ||
        (flight_option_enabled(FlightOptions::CENTER_THROTTLE_TRIM)) == 0) {
       // Conditions not met - fall back to standard linear throttle input
       return  get_throttle_input(no_deadzone);
    }
    
    // Apply exponential throttle curve
    // Maps: bottom stick → min, center stick → TRIM_THROTTLE, top stick → max
    // channel_throttle->get_range() provides output scaling
    // channel_throttle->norm_input() provides normalized input (-1 to +1)
    // Expression 0.5 + 0.5*norm_input converts from (-1,+1) to (0,1) range for curve
    float ret = channel_throttle->get_range() * throttle_curve(aparm.throttle_cruise * 0.01, 0, 0.5 + 0.5*channel_throttle->norm_input());
    
    // Apply throttle reversal if RC option switch is enabled
    // Reversal applied AFTER curve to preserve curve mapping
    if (reversed_throttle) {
        // RC option for reverse throttle has been set
        return -ret;
    }
    
    return ret;
}
