/**
 * @file AP_ExternalControl_Copter.cpp
 * @brief External control implementation for ArduCopter multirotor vehicles
 * 
 * @details This file implements the copter-specific external control interface,
 *          allowing external systems to command the vehicle through standardized
 *          APIs. External control is supported via multiple protocols:
 *          - MAVLink: SET_POSITION_TARGET_LOCAL_NED, SET_POSITION_TARGET_GLOBAL_INT
 *          - DDS/ROS2: Twist and pose commands
 *          - Lua scripting: vehicle:set_target_velocity_NED()
 * 
 *          The implementation provides coordinate frame transformations between
 *          external conventions (NED for MAVLink, ENU for ROS) and ArduPilot's
 *          internal representations. All external control commands require the
 *          vehicle to be in GUIDED mode and armed for safety.
 * 
 *          Key responsibilities:
 *          - Velocity and yaw rate control in earth frame
 *          - Global position targeting (GPS coordinates)
 *          - Coordinate frame conversions (NED↔NEU, meters↔centimeters)
 *          - Safety validation before accepting external commands
 *          - Integration with guided mode infrastructure
 * 
 * @note External control is only accepted when vehicle is in GUIDED mode and armed.
 *       Commands are rejected in other flight modes for safety.
 * 
 * @warning External control bypasses pilot stick inputs. Ensure external system
 *          implements appropriate failsafes and timeout mechanisms.
 * 
 * @see AP_ExternalControl base class for common interface
 * @see Mode_Guided for underlying guided mode implementation
 * @see libraries/AP_ExternalControl/AP_ExternalControl.h
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */


#include "AP_ExternalControl_Copter.h"
#if AP_EXTERNAL_CONTROL_ENABLED

#include "Copter.h"

/**
 * @brief Set target linear velocity and yaw rate for external control
 * 
 * @details Commands the vehicle to move at a specified velocity in earth frame with
 *          optional yaw rate control. This function is the primary velocity control
 *          interface for offboard control systems including MAVLink, ROS2/DDS, and
 *          Lua scripting.
 * 
 *          The function performs critical coordinate frame transformations:
 *          1. NED→NEU conversion: Flips Z-axis sign to match ArduPilot's internal
 *             convention where positive Z is upward (opposite of NED down convention)
 *          2. Unit conversion: Scales from meters/second to centimeters/second
 *          3. Yaw handling: NaN yaw_rate disables yaw control (position-only mode)
 * 
 *          Coordinate Frame Convention:
 *          - Input: Earth-fixed NED frame (North-East-Down) in meters/second
 *            * X: North (positive = forward), range typically ±10 m/s
 *            * Y: East (positive = right), range typically ±10 m/s  
 *            * Z: Down (positive = descend), range typically ±5 m/s
 *          - Internal: NEU frame (North-East-Up) in centimeters/second
 *            * Z-axis inverted from input for ArduPilot convention
 * 
 *          ROS2/DDS Compatibility:
 *          - ROS uses ENU frame; external systems should convert ENU→NED before calling
 *          - This function expects NED input as per MAVLink convention
 *          - The NEU internal frame facilitates integration with guided mode
 * 
 *          Safety Mechanisms:
 *          - Rejects commands if not in GUIDED mode (prevents mode confusion)
 *          - Rejects commands if vehicle not armed (prevents ground operation)
 *          - NaN yaw rate allows velocity-only control (yaw holds current heading)
 *          - Velocity commands are subject to guided mode acceleration limits
 * 
 *          Timeout Behavior:
 *          - External control commands timeout after 3 seconds (GCS_MAVLINK timeout)
 *          - Timeout triggers GUIDED mode failsafe (RTL or LAND depending on config)
 *          - Calling system must send updates faster than timeout period
 * 
 * @param[in] linear_velocity Target velocity in earth frame, NED convention [m/s]
 *                            Valid range: typically ±10 m/s horizontal, ±5 m/s vertical
 *                            Velocities are clamped by WPNAV_SPEED and PILOT_SPEED_UP/DN parameters
 * @param[in] yaw_rate_rads   Target yaw rate in earth frame [rad/s]
 *                            Positive = clockwise (right turn) when viewed from above
 *                            Pass NaN to disable yaw control and hold current heading
 *                            Valid range: typically ±0.5 rad/s (±28°/s)
 * 
 * @return true if command accepted and applied to guided mode controller
 *         false if command rejected (not in GUIDED mode, not armed, or other safety check failed)
 * 
 * @note Called at varying rates depending on external system (typically 10-50 Hz)
 * @note Velocity commands require continuous updates to maintain motion; timeout causes failsafe
 * @note Z-axis velocity is positive down (NED) on input, converted to positive up (NEU) internally
 * 
 * @warning Ensure external control system implements proper timeout detection and recovery.
 *          Loss of communication should trigger appropriate failsafe behavior.
 * @warning Do not mix external velocity commands with pilot stick inputs; guided mode
 *          will prioritize external commands but mode may be unpredictable.
 * 
 * @see ready_for_external_control() for safety validation checks
 * @see Mode_Guided::set_velocity() for underlying guided mode velocity control
 * @see GCS_MAVLINK::handle_set_position_target_local_ned() for MAVLink integration
 * @see libraries/AP_DDS/ for ROS2 DDS integration
 */
bool AP_ExternalControl_Copter::set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)
{
    // Safety check: Verify vehicle is in correct mode and state for external control
    // Returns false if vehicle is not in GUIDED mode or not armed
    if (!ready_for_external_control()) {
        return false;
    }
    
    // Handle NaN yaw rate: Convert to zero to disable yaw control
    // When yaw_rate is NaN, the vehicle will hold its current heading
    // This allows position-only control without affecting yaw
    const float checked_yaw_rate_rad = isnan(yaw_rate_rads)? 0: yaw_rate_rads;

    // Coordinate frame transformation: NED (input) → NEU (internal)
    // ArduPilot's internal convention uses positive-up Z-axis (NEU frame)
    // Input uses NED convention where positive Z is downward
    // Therefore: Z_neu = -Z_ned to flip the vertical axis sign
    // This transformation is critical for ROS compatibility where ENU is standard
    Vector3f velocity_NEU_ms {
        linear_velocity.x,  // North component unchanged (positive = forward)
        linear_velocity.y,  // East component unchanged (positive = right)
        -linear_velocity.z  // Z-axis sign flip: NED down → NEU up
    };
    
    // Unit conversion: meters/second → centimeters/second
    // ArduPilot guided mode expects velocity in cm/s for internal calculations
    // Scale factor: 1 m/s = 100 cm/s
    Vector3f velocity_up_cms = velocity_NEU_ms * 100;
    
    // Command guided mode velocity controller with transformed values
    // Parameters:
    //   velocity_up_cms: Target velocity in NEU frame [cm/s]
    //   false: use_yaw_rate flag (determines if next param is rate or angle)
    //   0: yaw angle (unused when use_yaw_rate is false)
    //   !isnan(yaw_rate_rads): Enable yaw rate control if yaw_rate was not NaN
    //   checked_yaw_rate_rad: Target yaw rate [rad/s] (0 if yaw disabled)
    copter.mode_guided.set_velocity(velocity_up_cms, false, 0, !isnan(yaw_rate_rads), checked_yaw_rate_rad);
    
    return true;
}

/**
 * @brief Set target global position for external control navigation
 * 
 * @details Commands the vehicle to navigate to a specified GPS coordinate with
 *          optional altitude. This is the primary position control interface for
 *          offboard waypoint navigation from external systems (MAVLink, ROS2, scripts).
 * 
 *          The function integrates with ArduPilot's guided mode waypoint navigation,
 *          which provides:
 *          - Automated path planning using waypoint navigation controller (AC_WPNav)
 *          - Smooth acceleration/deceleration profiles (S-curves)
 *          - Terrain following if enabled and altitude frame supports it
 *          - Position hold upon reaching target
 * 
 *          Position Command Behavior:
 *          - Vehicle navigates to target using WPNAV_SPEED horizontal speed limit
 *          - Altitude changes use PILOT_SPEED_UP/DN vertical speed limits
 *          - Position tolerance: WPNAV_RADIUS (typically 200cm) for arrival detection
 *          - Upon arrival, vehicle enters loiter mode at target location
 * 
 *          Coordinate System:
 *          - Latitude/Longitude: WGS84 datum in degrees (scaled integer internally)
 *          - Altitude: Depends on Location.alt_frame (MSL, AGL, or terrain-relative)
 *          - No coordinate transformation needed (global frame matches internal)
 * 
 *          Altitude Frame Options (Location.alt_frame):
 *          - ALT_FRAME_ABSOLUTE: Meters above mean sea level (GPS altitude)
 *          - ALT_FRAME_ABOVE_HOME: Meters above home/arming location
 *          - ALT_FRAME_ABOVE_ORIGIN: Meters above EKF origin
 *          - ALT_FRAME_ABOVE_TERRAIN: Meters above terrain (requires rangefinder or terrain DB)
 * 
 *          Safety Mechanisms:
 *          - Rejects commands if not in GUIDED mode (mode safety check)
 *          - Rejects commands if vehicle not armed (ground safety)
 *          - Position constrained by fence boundaries if enabled (AC_Fence)
 *          - Rally point system provides alternate safe locations
 * 
 *          Timeout Behavior:
 *          - Position commands do NOT timeout (waypoint persists until reached or changed)
 *          - Unlike velocity commands, single position command drives navigation until completion
 *          - New position command overrides previous target immediately
 * 
 *          Integration with External Systems:
 *          - MAVLink: SET_POSITION_TARGET_GLOBAL_INT message handler uses this function
 *          - ROS2/DDS: Global pose messages converted to Location and passed here
 *          - Lua Scripting: vehicle:set_target_location() binds to this function
 * 
 * @param[in] loc Target location in global coordinates (WGS84)
 *                Location fields:
 *                - lat: Latitude in degrees × 1e7 (e.g., 47.6062095° = 476062095)
 *                - lng: Longitude in degrees × 1e7 (e.g., -122.3320708° = -1223320708)
 *                - alt: Altitude in centimeters (frame determined by alt_frame field)
 *                - alt_frame: Altitude reference frame (MSL, AGL, terrain-relative)
 * 
 * @return true if command accepted and navigation initiated
 *         false if command rejected (not in GUIDED mode, not armed, or invalid location)
 * 
 * @note GPS lock required: Function fails if GPS does not have 3D fix (GPS_FIX_TYPE >= 3)
 * @note EKF position estimate required: Navigation requires valid EKF position (EKF_CHECK_MASK)
 * @note Position commands are not rate-limited; calling system controls update frequency
 * 
 * @warning Target location must be within geofence boundaries if fencing is enabled.
 *          Fence breach triggers fence failsafe action (RTL, LAND, or BRAKE).
 * @warning Altitude frame mismatch can cause unexpected behavior. Ensure external system
 *          uses consistent altitude frame throughout mission.
 * @warning Large position jumps (>50m) may trigger EKF position reset protections.
 * 
 * @see ready_for_external_control() for safety validation
 * @see Copter::set_target_location() for position command implementation
 * @see AC_WPNav for waypoint navigation controller
 * @see AC_Fence for geofence constraint checking
 * @see Location.h for coordinate system and altitude frame definitions
 */
bool AP_ExternalControl_Copter::set_global_position(const Location& loc)
{
    // Safety check: Verify vehicle is in correct mode and state for external control
    // Returns false if vehicle is not in GUIDED mode or not armed
    if (!ready_for_external_control()) {
        return false;
    }
    
    // Forward position command to copter's guided mode target location handler
    // This integrates with waypoint navigation (AC_WPNav) for smooth path following
    // Returns true if position accepted, false if GPS not available or location invalid
    return copter.set_target_location(loc);
}

/**
 * @brief Validate vehicle state for accepting external control commands
 * 
 * @details Performs safety checks to determine if the vehicle is in an appropriate
 *          state to accept external control commands. This function acts as a safety
 *          gate preventing external commands from affecting the vehicle in wrong modes
 *          or unsafe states.
 * 
 *          External control is only permitted when ALL conditions are met:
 *          1. Vehicle is in GUIDED mode (or AUTO with guided submode)
 *          2. Vehicle motors are armed
 * 
 *          Rationale for GUIDED Mode Requirement:
 *          - GUIDED mode is specifically designed for external/offboard control
 *          - Prevents external commands from interfering with pilot manual control
 *          - Ensures pilot explicitly enables external control by switching to GUIDED
 *          - Pilot can immediately regain control by switching to any other mode
 * 
 *          Rationale for Armed Requirement:
 *          - Prevents unexpected motor startup from external commands on ground
 *          - Ensures pilot has completed pre-flight checks (arm checks passed)
 *          - External control during disarmed state could be dangerous
 * 
 *          Mode Safety Philosophy:
 *          - External control commands silently rejected in wrong mode (no error logged)
 *          - This allows external systems to continuously send commands without spam
 *          - Mode switch to GUIDED immediately enables command acceptance
 *          - Mode switch away from GUIDED immediately disables external control
 * 
 *          Integration with Flight Mode System:
 *          - in_guided_mode() returns true for: GUIDED, AUTO (in guided submode)
 *          - Returns false for: STABILIZE, LOITER, RTL, LAND, and other modes
 *          - Mode transitions are handled by flight mode state machine
 * 
 *          Timeout and Failsafe Interaction:
 *          - This check is independent of GCS failsafe timeout (3 seconds)
 *          - GCS failsafe triggers if no MAVLink commands received within timeout
 *          - This function only validates instantaneous state for command acceptance
 *          - Failsafe action (RTL/LAND) determined by FS_GCS_ENABLE parameter
 * 
 * @return true if vehicle is ready to accept external control commands
 *              (in GUIDED mode AND motors armed)
 *         false if vehicle is not ready (wrong mode OR motors disarmed)
 * 
 * @note This is called before every external control command to validate state
 * @note Function has minimal computational cost (two boolean checks)
 * @note Disarming immediately blocks external control even if in GUIDED mode
 * @note Switching out of GUIDED immediately blocks external control even if armed
 * 
 * @warning External systems should monitor command rejection and implement appropriate
 *          failsafes. Continuous rejection indicates mode change or disarm event.
 * @warning Do not bypass this check. Attempting external control in wrong mode can
 *          cause unpredictable vehicle behavior and safety hazards.
 * 
 * @see Mode::in_guided_mode() for flight mode validation logic
 * @see AP_Motors::armed() for motor arming state
 * @see GCS_MAVLINK timeout handling for external control timeout behavior
 */
bool AP_ExternalControl_Copter::ready_for_external_control()
{
    // Check 1: Verify vehicle is in GUIDED mode (or AUTO with guided submode)
    // GUIDED mode is designed for external/offboard control and ensures pilot
    // has explicitly authorized external command acceptance
    // in_guided_mode() returns true for GUIDED and AUTO modes with guided submode active
    //
    // Check 2: Verify motors are armed (safety critical)
    // Arming indicates pilot has completed pre-flight checks and authorized flight
    // Prevents external commands from unexpectedly starting motors on ground
    //
    // Both conditions must be true (logical AND) for external control acceptance
    return copter.flightmode->in_guided_mode() && copter.motors->armed();
}

#endif // AP_EXTERNAL_CONTROL_ENABLED
