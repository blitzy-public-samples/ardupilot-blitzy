/**
 * @file AP_ExternalControl_Plane.cpp
 * @brief External control interface implementation for ArduPlane
 * 
 * @details This file implements the external control interface for fixed-wing aircraft,
 *          enabling companion computers, ground control stations, and scripts to command
 *          the vehicle through standardized APIs. External control is used for:
 *          - MAVLink offboard control commands
 *          - DDS/ROS2 integration for autonomous operations
 *          - Lua scripting for mission customization
 * 
 *          The external control interface provides a safe abstraction layer that:
 *          - Validates mode requirements (typically requires GUIDED mode)
 *          - Performs safety checks before accepting commands
 *          - Integrates with existing flight mode logic
 *          - Respects arming state and pre-arm checks
 * 
 *          ArduPlane Implementation Status:
 *          - set_global_position(): Implemented - Sets target loiter location
 *          - set_airspeed(): Implemented (if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED)
 *          - set_linear_velocity_and_yaw_rate(): NOT implemented (uses base class default return false)
 *          - arm()/disarm(): Uses base class implementation with vehicle-specific checks
 * 
 *          Safety Considerations:
 *          - All commands require appropriate flight mode (typically GUIDED)
 *          - Commands are rejected if vehicle is not in correct state
 *          - No direct actuator control - commands go through flight mode logic
 *          - Standard failsafe mechanisms remain active
 * 
 * @note This entire module is conditionally compiled with AP_EXTERNAL_CONTROL_ENABLED
 * @warning External control commands bypass pilot RC input - ensure companion computer
 *          implements appropriate failsafe behavior if communication is lost
 * 
 * @see AP_ExternalControl base class in libraries/AP_ExternalControl/
 * @see Plane::set_target_location() for position command handling
 * @see Mode_Guided for guided mode implementation
 * 
 * Source: ArduPlane/AP_ExternalControl_Plane.cpp
 */


#include "AP_ExternalControl_Plane.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Plane.h"

/**
 * @brief Sets the target global position for a loiter point
 * 
 * @details This method commands the aircraft to navigate to and loiter at a specified
 *          global position. The command is processed through the standard guided mode
 *          target location mechanism, which handles:
 *          - Waypoint navigation to the target
 *          - Loiter circle setup at the target location
 *          - Altitude management (respects location altitude and frame type)
 *          - Integration with L1 navigation controller
 * 
 *          The command delegates to Plane::set_target_location() which performs
 *          safety validation:
 *          - Verifies vehicle is in GUIDED mode (required for external control)
 *          - Validates location parameters (lat/lon/alt within reasonable bounds)
 *          - Does NOT require vehicle to be armed or flying (pre-arm navigation setup allowed)
 * 
 *          Typical usage: Companion computer sends MAVLink SET_POSITION_TARGET_GLOBAL_INT
 *          message which is routed through this interface to command waypoint navigation.
 * 
 * @param[in] loc Target location in global coordinates (latitude, longitude, altitude)
 *                Supports multiple altitude frames:
 *                - Location::AltFrame::ABSOLUTE (MSL altitude in meters)
 *                - Location::AltFrame::ABOVE_HOME (meters above home)
 *                - Location::AltFrame::ABOVE_TERRAIN (meters above terrain with terrain following)
 * 
 * @return true if command was accepted and target location set successfully
 * @return false if command rejected due to:
 *         - Vehicle not in GUIDED mode
 *         - Invalid location parameters
 *         - Plane not ready for navigation commands
 * 
 * @note This method can be called when disarmed to pre-position navigation targets
 * @note The vehicle will navigate to and loiter at this position using configured
 *       loiter radius and direction parameters
 * 
 * @warning Ensure target location is within geofence boundaries if fence is enabled
 * @warning Does not validate terrain clearance - terrain following only active if enabled
 * 
 * @see Plane::set_target_location() for detailed validation and navigation setup
 * @see Mode_Guided::set_target_location() for guided mode specific handling
 * @see AP_Mission::Location for altitude frame documentation
 * 
 * Source: ArduPlane/AP_ExternalControl_Plane.cpp:14-20
 */
bool AP_ExternalControl_Plane::set_global_position(const Location& loc)
{

    // set_target_location already checks if plane is ready for external control.
    // It doesn't check if flying or armed, just that it's in guided mode.
    return plane.set_target_location(loc);
}

/**
 * @brief Sets the target airspeed for the aircraft
 * 
 * @details This method commands the aircraft to change its target airspeed, which is
 *          integrated into the Total Energy Control System (TECS) for energy management
 *          in fixed-wing flight. The airspeed command affects:
 *          - TECS energy distribution between altitude and airspeed
 *          - Throttle control to achieve target speed
 *          - Pitch control for speed management
 *          - Coordination with altitude hold or climb commands
 * 
 *          Implementation is conditionally compiled based on AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED.
 *          When enabled, the command:
 *          - Validates vehicle is in GUIDED mode (strict requirement)
 *          - Sets target airspeed with maximum acceleration (instantaneous change)
 *          - Integrates with guided mode airspeed slew rate limiting
 * 
 *          When disabled (feature not compiled), this method always returns false.
 * 
 *          Safety Checks Performed:
 *          - Mode validation: MUST be in GUIDED mode
 *          - Guided mode performs additional validation of airspeed limits
 * 
 *          Typical usage: MAVLink SET_POSITION_TARGET_LOCAL_NED with type_mask requesting
 *          velocity control, or direct airspeed commands from companion computer.
 * 
 * @param[in] airspeed Target airspeed in meters/second
 *                     Valid range typically 5-50 m/s depending on aircraft
 *                     Should be within ARSPD_FBW_MIN and ARSPD_FBW_MAX parameters
 * 
 * @return true if command accepted and target airspeed set successfully
 * @return false if command rejected due to:
 *         - AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED not defined (feature disabled)
 *         - Vehicle not in GUIDED mode
 *         - Airspeed value outside valid range (checked by guided mode)
 * 
 * @note Acceleration parameter is hardcoded to 0.0 (maximum acceleration/instantaneous change)
 * @note Target airspeed interacts with TECS tuning parameters (TECS_*)
 * @note Actual achieved airspeed depends on available thrust and atmospheric conditions
 * 
 * @warning Ensure target airspeed is within aircraft's safe operating envelope
 * @warning Very low airspeeds risk stall, very high airspeeds risk structural damage
 * @warning Does not check stall speed or Vne limits - companion computer must validate
 * 
 * @see Mode_Guided::handle_change_airspeed() for guided mode processing
 * @see AP_TECS for Total Energy Control System implementation
 * @see ARSPD_FBW_MIN and ARSPD_FBW_MAX parameters for airspeed limits
 * @see AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED feature flag
 * 
 * Source: ArduPlane/AP_ExternalControl_Plane.cpp:25-39
 */
bool AP_ExternalControl_Plane::set_airspeed(const float airspeed)
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // The command is only valid in guided mode.
    if (plane.control_mode != &plane.mode_guided) {
        return false;
    }

    // Assume the user wanted maximum acceleration.
    const float acc_instant = 0.0;
    return plane.mode_guided.handle_change_airspeed(airspeed, acc_instant);
#else 
  return false;
#endif
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
