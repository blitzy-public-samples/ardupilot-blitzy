/**
 * @file AP_ExternalControl_Plane.h
 * @brief External control interface for fixed-wing aircraft (ArduPlane)
 * 
 * @details This header defines the plane-specific implementation of the external
 *          control interface. External control allows offboard systems to directly
 *          control the aircraft through MAVLink commands or other interfaces, bypassing
 *          the normal pilot RC input path.
 *          
 *          The plane implementation supports:
 *          - Global position control (loiter point targeting)
 *          - Direct airspeed control
 *          
 *          This is typically used for:
 *          - Computer vision guided flight
 *          - Companion computer control
 *          - Advanced autonomous missions requiring dynamic waypoint updates
 *          - External guidance systems integration
 *          
 *          Safety Note: External control bypasses normal pilot input. The arming
 *          and mode systems still provide safety oversight, but external commands
 *          are executed with minimal validation.
 * 
 * @note Requires AP_EXTERNAL_CONTROL_ENABLED to be defined at compile time
 * @see AP_ExternalControl base class for common interface definition
 * @see ArduPlane mode_guided.cpp for implementation of external control integration
 */
#pragma once

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

#include <AP_Common/Location.h>

/**
 * @class AP_ExternalControl_Plane
 * @brief Fixed-wing aircraft specific implementation of external control interface
 * 
 * @details This class provides plane-specific overrides for the external control
 *          system. It inherits from AP_ExternalControl base class and implements
 *          vehicle-specific control methods suitable for fixed-wing flight.
 *          
 *          External control for planes differs from multicopters in several ways:
 *          - Airspeed control is available (not applicable to multicopters)
 *          - Position control typically commands loiter points rather than hover positions
 *          - Must respect minimum airspeed constraints for safe flight
 *          - Works primarily in GUIDED mode
 *          
 *          Control Authority:
 *          - External commands override pilot RC input when in GUIDED mode
 *          - Failsafe systems remain active (geofencing, battery, radio)
 *          - Mode changes can still be commanded by pilot or GCS
 *          - Arming/disarming logic is unaffected
 *          
 *          Thread Safety:
 *          - Methods are called from MAVLink command handler thread
 *          - Coordinates with main flight control loop through mode system
 *          - No direct motor control - commands are filtered through mode logic
 * 
 * @note All control methods return bool indicating acceptance/rejection of command
 * @warning External control bypasses normal pilot input validation. Offboard
 *          systems must implement their own safety checks and rate limiting.
 */
class AP_ExternalControl_Plane : public AP_ExternalControl {
public:
    /**
     * @brief Set the target global position for a loiter point
     * 
     * @details Commands the aircraft to fly to and loiter at the specified global
     *          position. This is the primary method for external position control
     *          of fixed-wing aircraft. The aircraft will navigate to the target
     *          using the L1 guidance controller and establish a loiter pattern.
     *          
     *          Behavior:
     *          - Target position becomes the new loiter center point
     *          - Aircraft uses current loiter radius parameter (WP_LOITER_RAD)
     *          - If altitude is included, aircraft will climb/descend to target altitude
     *          - Navigation uses L1 controller for smooth path following
     *          - Respects terrain following if enabled
     *          
     *          Flight Mode Requirements:
     *          - Aircraft must be in GUIDED mode for command to be accepted
     *          - Arming checks must be passed (cannot command disarmed aircraft)
     *          - Geofence (if enabled) will constrain navigation to target
     *          
     *          Coordinate Frame:
     *          - Location uses WGS84 geodetic coordinates (latitude/longitude)
     *          - Altitude is relative to home or absolute MSL based on Location.flags
     *          - Standard ArduPilot Location structure with terrain handling support
     * 
     * @param[in] loc Target location in global coordinates (WGS84 lat/lon)
     *                Altitude interpretation depends on Location.flags:
     *                - ABSOLUTE: Altitude relative to MSL (mean sea level)
     *                - RELATIVE: Altitude relative to home position
     *                - TERRAIN: Altitude relative to terrain (requires terrain data)
     * 
     * @return true if position command was accepted and aircraft will navigate to target
     * @return false if command rejected (wrong mode, disarmed, invalid location, etc.)
     * 
     * @note This method is typically called at 1-10 Hz from external control system
     * @note Calling repeatedly with same position maintains loiter at that point
     * @note Rapid position updates (>10 Hz) may cause navigation instability
     * 
     * @warning Does not validate if target position is reachable given current airspeed,
     *          wind conditions, or energy state. External system must ensure feasible targets.
     * @warning Target location must be within geofence boundaries if fence is enabled
     * 
     * @see Mode::guided_WP_loc for implementation of guided waypoint navigation
     * @see AP_L1_Control for path following controller used in navigation
     */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;

    /**
     * @brief Set the target airspeed for the aircraft
     * 
     * @details Commands the aircraft to fly at the specified indicated airspeed (IAS).
     *          This provides direct control over the aircraft's speed setpoint, overriding
     *          the normal cruise speed or speed parameters. The TECS (Total Energy Control
     *          System) will adjust throttle and pitch to achieve and maintain the target
     *          airspeed while balancing altitude control requirements.
     *          
     *          Control Integration:
     *          - Command is processed by TECS energy management system
     *          - Throttle and pitch adjusted automatically to achieve target speed
     *          - Altitude control objectives may temporarily override speed to maintain
     *            energy state and prevent stall or overspeed
     *          - Wind compensation is automatic (maintains IAS regardless of wind)
     *          
     *          Flight Mode Requirements:
     *          - Aircraft must be in GUIDED mode for command to be accepted
     *          - Must be armed and flying (disarmed or on ground will reject)
     *          - Airspeed sensor should be healthy (works without but less accurate)
     *          
     *          Safety Constraints:
     *          - Command is clamped to configured minimum/maximum airspeeds
     *          - ARSPD_FBW_MIN and ARSPD_FBW_MAX parameters define safe speed range
     *          - TECS will not allow speeds that would risk stall or structural damage
     *          - Stall prevention logic remains active even with external speed commands
     *          
     *          Typical Usage:
     *          - Slow flight for surveillance or inspection (near minimum safe speed)
     *          - High-speed transit between waypoints
     *          - Dynamic speed adjustment based on mission requirements
     *          - Coordination with vision systems requiring specific ground speeds
     * 
     * @param[in] airspeed Target indicated airspeed in meters per second (m/s)
     *                     Valid range: ARSPD_FBW_MIN to ARSPD_FBW_MAX (typically 10-30 m/s)
     *                     Values outside range will be clamped to safe limits
     * 
     * @return true if airspeed command was accepted and TECS will target this speed
     * @return false if command rejected (wrong mode, disarmed, on ground, invalid value)
     * 
     * @note Airspeed is indicated airspeed (IAS), not ground speed or true airspeed
     * @note This method can be called at 1-10 Hz for dynamic speed control
     * @note Airspeed changes are gradual - TECS applies acceleration limits
     * @note Without airspeed sensor, speed is estimated from GPS and wind estimate
     * 
     * @warning Setting airspeed too low may cause stall if TECS cannot maintain altitude
     * @warning Setting airspeed too high may exceed structural limits or available thrust
     * @warning External system should respect aircraft performance envelope
     * 
     * @see AP_TECS for total energy control system implementation
     * @see ARSPD_FBW_MIN and ARSPD_FBW_MAX parameters for configured speed limits
     * @see AP_Airspeed for airspeed measurement and estimation
     */
    bool set_airspeed(const float airspeed) override WARN_IF_UNUSED;

};

#endif // AP_EXTERNAL_CONTROL_ENABLED
