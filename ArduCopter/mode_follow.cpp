/**
 * @file mode_follow.cpp
 * @brief Follow mode implementation for ArduCopter
 * 
 * @details Follow mode enables a multicopter to autonomously follow another vehicle
 *          that is broadcasting its position via MAVLink. The following vehicle maintains
 *          a configurable offset distance and angle relative to the lead vehicle, with
 *          options for altitude matching or independent altitude control.
 * 
 *          Key Features:
 *          - Follows another MAVLink-enabled vehicle by system ID
 *          - Configurable follow distance and angle (spherical offset)
 *          - Multiple yaw behavior options (face lead, match heading, direction of flight)
 *          - Altitude matching or independent altitude control
 *          - Velocity and acceleration feedforward for smooth tracking
 *          - Automatic offset initialization to prevent starting on top of lead vehicle
 * 
 *          Applications:
 *          - Vehicle formations (multiple aircraft following a lead)
 *          - "Follow-me" mode with ground vehicles or other aircraft
 *          - Automated cinematography and tracking shots
 *          - Convoy operations
 * 
 *          The mode integrates with the AP_Follow library to receive target vehicle
 *          position, velocity, and acceleration data via MAVLink GLOBAL_POSITION_INT
 *          messages. Position control uses feedforward from target dynamics for
 *          responsive tracking.
 * 
 * @note Requires FOLL_ENABLE parameter set to 1
 * @warning Ensure adequate separation distance to avoid collisions with lead vehicle
 * 
 * @see libraries/AP_Follow for follow library implementation
 * 
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AP_AVOIDANCE_ENABLED is true because we rely on it velocity limiting functions
 */

#include "Copter.h"

#if MODE_FOLLOW_ENABLED

/**
 * @brief Initialize Follow mode
 * 
 * @details Sets up the position and velocity controllers for follow mode operation.
 *          Configures horizontal and vertical speed/acceleration limits, initializes
 *          the position controller state, and sets the default yaw behavior.
 * 
 *          Initialization sequence:
 *          1. Verify AP_Follow library is enabled (FOLL_ENABLE = 1)
 *          2. Configure gimbal mount to track target (if FOLL_OFS_TYPE includes mount option)
 *          3. Set horizontal speed and acceleration limits from waypoint navigation defaults
 *          4. Set vertical speed and acceleration limits (climb/descend rates)
 *          5. Initialize NE (North-East) and U (Up) position controllers
 *          6. Configure yaw behavior to default mode
 * 
 *          The position controller uses the same speed and acceleration limits as
 *          waypoint navigation to ensure consistent vehicle behavior. Controller
 *          initialization prepares PID loops and sets initial target positions.
 * 
 * @param[in] ignore_checks If true, skip pre-arm safety checks (currently unused in Follow mode)
 * 
 * @return true if initialization successful, false if AP_Follow library not enabled
 * 
 * @note Sends GCS warning message if FOLL_ENABLE parameter is not set to 1
 * @note Requires valid EKF position estimate for position controller initialization
 * 
 * @see AP_Follow library for target vehicle tracking configuration
 * @see AC_PosControl for position controller details
 */
bool ModeFollow::init(const bool ignore_checks)
{
    // Verify AP_Follow library is enabled via FOLL_ENABLE parameter
    // The AP_Follow library handles reception of MAVLink GLOBAL_POSITION_INT messages
    // from the target vehicle and calculates desired offset position
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    // Configure camera gimbal mount to track the lead vehicle if MOUNT_FOLLOW_ON_ENTER option enabled
    // This allows the camera to automatically point at the target being followed
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    // Configure horizontal (North-East) speed and acceleration limits
    // Uses waypoint navigation defaults for consistent vehicle behavior across flight modes
    // Speed in cm/s, acceleration in cm/s/s
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // Configure vertical (Up) speed and acceleration limits
    // Separate limits for climb (up) and descent (down) rates
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // Initialize position controller PID loops for vertical and horizontal control
    // Sets initial target position to current vehicle position
    pos_control->init_U_controller();
    pos_control->init_NE_controller();

    // Initialize yaw behavior to default configured mode
    // Yaw behavior can be: face lead vehicle, match lead heading, or direction of flight
    auto_yaw.set_mode_to_default(false);

    return true;
}

/**
 * @brief Perform cleanup when exiting Follow mode
 * 
 * @details Clears the follow offset if configured to reset on mode exit.
 *          This ensures that when re-entering Follow mode, the vehicle will
 *          reinitialize its offset position rather than maintaining the previous
 *          offset, which may no longer be valid.
 * 
 * @note Called automatically by the mode controller when switching away from Follow mode
 * 
 * @see AP_Follow::clear_offsets_if_required() for offset clearing behavior
 */
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

/**
 * @brief Main run function for Follow mode, called at main loop rate (typically 400Hz)
 * 
 * @details Implements autonomous vehicle following by tracking a lead vehicle's position,
 *          velocity, and acceleration received via MAVLink. The following vehicle maintains
 *          a configurable offset (distance and angle) relative to the lead vehicle in a
 *          spherical coordinate system.
 * 
 *          Control Flow:
 *          1. Safety check: If disarmed or landed, perform safe ground handling and return
 *          2. Initialize follow offset on first run (prevents starting on top of lead vehicle)
 *          3. Enable full throttle range for responsive control
 *          4. Retrieve target position, velocity, and acceleration from AP_Follow library
 *          5. Convert target data from meters (NED frame) to centimeters for position controller
 *          6. Send position, velocity, and acceleration commands to position controller (feedforward)
 *          7. Determine desired yaw based on configured behavior mode:
 *             - FACE_LEAD_VEHICLE: Point nose toward lead vehicle
 *             - SAME_AS_LEAD_VEHICLE: Match lead vehicle's heading
 *             - DIR_OF_FLIGHT: Face direction of travel
 *             - NONE: Maintain current heading
 *          8. Update position controllers (NE and U separately)
 *          9. Generate attitude and throttle commands from position controller output
 * 
 *          Target Tracking with AP_Follow Library:
 *          The AP_Follow library receives MAVLink GLOBAL_POSITION_INT messages from the
 *          target vehicle and maintains its position, velocity, and acceleration estimates.
 *          The offset position is calculated based on FOLL_DIST_MAX (distance), FOLL_OFS_X/Y/Z
 *          (cartesian offsets), and FOLL_ALT_TYPE (altitude tracking mode). Velocity and
 *          acceleration feedforward enables smooth tracking without lag.
 * 
 *          Coordinate Frames:
 *          - NED (North-East-Down): Earth-fixed frame used by AP_Follow for target position
 *          - Body Frame: Vehicle-relative frame for some calculations
 *          - Position controller operates in NED frame with centimeter units
 * 
 * @note Called at main loop rate (typically 400Hz) when Follow mode is active
 * @note Requires valid target data from AP_Follow library; holds position if target data invalid
 * @note Uses velocity and acceleration feedforward for responsive tracking
 * 
 * @warning Ensure adequate FOLL_DIST_MAX setting to maintain safe separation from lead vehicle
 * @warning Target data loss will cause vehicle to hold current position (loiter)
 * 
 * @see AP_Follow::get_ofs_pos_vel_accel_NED_m() for target data retrieval
 * @see AC_PosControl for position controller details
 */
void ModeFollow::run()
{
    // Safety check: if vehicle is disarmed or landed, disable motors and exit
    // Ensures safe ground handling and prevents unintended motor commands
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // Initialize follow offset if not yet set on first run
    // Prevents vehicle from starting directly on top of the lead vehicle by establishing
    // initial separation based on FOLL_DIST_MAX and FOLL_OFS_X/Y/Z parameters
    g2.follow.init_offsets_if_required();

    // Enable full throttle range for responsive altitude and position control
    // Allows motors to use complete output range for aggressive tracking maneuvers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Initialize yaw to current attitude target, may be overridden based on yaw behavior mode
    float yaw_rad = attitude_control->get_att_target_euler_rad().z;
    float yaw_rate_rads = 0.0f;

    // Target tracking data in NED (North-East-Down) frame relative to this vehicle
    Vector3p pos_ofs_ned_m;      // Desired position: lead vehicle position + configured offset (meters)
    Vector3f vel_ofs_ned_ms;     // Desired velocity: lead vehicle velocity (m/s, for feedforward)
    Vector3f accel_ofs_ned_mss;  // Desired acceleration: lead vehicle accel (m/s/s, for feedforward)
    
    // Retrieve target position, velocity, and acceleration from AP_Follow library
    // AP_Follow calculates offset position based on lead vehicle and FOLL_DIST_MAX, FOLL_OFS_X/Y/Z params
    // Returns false if target data is stale or invalid (no recent MAVLink GLOBAL_POSITION_INT messages)
    if (g2.follow.get_ofs_pos_vel_accel_NED_m(pos_ofs_ned_m, vel_ofs_ned_ms, accel_ofs_ned_mss)) {
        // Convert horizontal (North-East) target data from meters to centimeters for position controller
        Vector2p pos_ofs_ne_cm = pos_ofs_ned_m.xy() * 100.0;
        Vector2f vel_ofs_ne_cms = vel_ofs_ned_ms.xy() * 100.0;
        Vector2f accel_ofs_ne_cmss = accel_ofs_ned_mss.xy() * 100.0;

        // Retrieve lead vehicle heading and heading rate for yaw matching mode
        float target_heading_deg = 0.0f;
        float target_heading_rate_degs = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);
        g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);

        // Send horizontal position, velocity, and acceleration commands to position controller
        // Velocity and acceleration feedforward enables smooth tracking with minimal lag
        pos_control->input_pos_vel_accel_NE_cm(pos_ofs_ne_cm, vel_ofs_ne_cms, accel_ofs_ne_cmss, false);

        // Convert vertical (Up) target data from NED Down to Up coordinate (negate Z)
        // Position controller uses Up convention, NED uses Down convention
        float pos_ofs_u_cm = -pos_ofs_ned_m.z * 100.0;
        float vel_ofs_u_cms = -vel_ofs_ned_ms.z * 100.0;
        float accel_ofs_u_cmss = -accel_ofs_ned_mss.z * 100.0;
        pos_control->input_pos_vel_accel_U_cm(pos_ofs_u_cm, vel_ofs_u_cms, accel_ofs_u_cmss, false);

        // Determine desired yaw behavior based on FOLL_YAW_BEHAVE parameter
        // Different yaw modes optimize for different follow scenarios
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                // Point nose directly toward the lead vehicle (useful for camera tracking)
                // Calculate bearing from following vehicle to lead vehicle position
                Vector3p pos_ned_m;      // Vector to lead vehicle (without offset)
                Vector3f vel_ned_ms;     // Velocity of lead vehicle
                Vector3f accel_ned_mss;  // Acceleration of lead vehicle
                if (g2.follow.get_target_pos_vel_accel_NED_m(pos_ned_m, vel_ned_ms, accel_ned_mss))
                // Only update yaw if more than 1m away (avoid erratic behavior when very close)
                if (pos_ned_m.xy().length_squared() > 1.0) {
                    yaw_rad = (pos_ned_m.xy() - pos_control->get_pos_target_NEU_cm().xy()).tofloat().angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                // Match the heading of the lead vehicle (useful for formation flight)
                // Uses heading broadcast by lead vehicle in MAVLink messages
                // Includes heading rate for smooth yaw tracking
                yaw_rad = radians(target_heading_deg);
                yaw_rate_rads = radians(target_heading_rate_degs);
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                // Face the direction of travel (forward velocity vector)
                // Provides intuitive vehicle orientation during movement
                // Only update if velocity > 1 m/s to avoid erratic behavior at low speeds
                if (vel_ofs_ne_cms.length_squared() > (100.0 * 100.0)) {
                    yaw_rad = vel_ofs_ne_cms.angle();
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // Maintain current yaw (pilot retains yaw control via stick input)
                // Useful when pilot wants independent heading control
               break;

        }
    } else {
        // Target data is invalid or stale (no recent MAVLink messages from lead vehicle)
        // Failsafe behavior: Hold current position (loiter) with zero velocity commands
        // Vehicle will maintain altitude and horizontal position until target data resumes
        Vector2f vel_zero;
        Vector2f accel_zero;
        pos_control->input_vel_accel_NE_cm(vel_zero, accel_zero, false);
        float velz = 0.0;
        pos_control->input_vel_accel_U_cm(velz, 0.0, false);
        yaw_rate_rads = 0.0f;
    }

    // Update position controllers to calculate desired acceleration and thrust vector
    // Horizontal (NE) and vertical (U) controllers run independently
    pos_control->update_NE_controller();
    pos_control->update_U_controller();

    // Send thrust vector and desired yaw to attitude controller
    // Attitude controller converts desired thrust vector and yaw into motor commands
    // Runs attitude PID loops to achieve desired vehicle orientation and thrust
    attitude_control->input_thrust_vector_heading_rad(pos_control->get_thrust_vector(), yaw_rad, yaw_rate_rads);
}

/**
 * @brief Get distance to target vehicle in meters
 * 
 * @details Returns the 3D distance from the following vehicle to the lead vehicle's
 *          offset target position. This distance includes both horizontal and vertical
 *          separation. Used for GCS telemetry display and logging.
 * 
 * @return Distance to target in meters, or 0.0 if target data invalid
 * 
 * @note This reports distance to the offset target position (lead + offset), not raw lead position
 * @note Value used for MAVLink DISTANCE_SENSOR message and mission waypoint distance display
 * 
 * @see AP_Follow::get_distance_to_target_m() for calculation details
 */
float ModeFollow::wp_distance_m() const
{
    return g2.follow.get_distance_to_target_m();
}

/**
 * @brief Get bearing to target vehicle in centidegrees
 * 
 * @details Returns the horizontal bearing from the following vehicle to the lead
 *          vehicle's offset target position. Bearing is measured clockwise from North
 *          (0° = North, 90° = East, 180° = South, 270° = West).
 * 
 * @return Bearing to target in centidegrees (0-36000), or 0 if target data invalid
 * 
 * @note Returns centidegrees (degrees * 100) for consistency with ArduPilot angle conventions
 * @note Value used for MAVLink mission status reporting and GCS display
 * 
 * @see AP_Follow::get_bearing_to_target_deg() for calculation details
 */
float ModeFollow::wp_bearing_deg() const
{
    return g2.follow.get_bearing_to_target_deg() * 100;
}

/**
 * @brief Get target waypoint location for MAVLink reporting
 * 
 * @details Retrieves the target location (lead vehicle position + configured offset)
 *          for reporting to ground control station. The location is returned in global
 *          coordinates (latitude, longitude, altitude) for display on GCS map and
 *          mission planning interfaces.
 * 
 *          This function enables the GCS to display the follow target as a waypoint,
 *          showing where the vehicle is attempting to position itself relative to the
 *          lead vehicle.
 * 
 * @param[out] loc Target location in global coordinates (with offset applied)
 * 
 * @return true if target location is valid, false if target data is stale or invalid
 * 
 * @note Target location updates continuously as lead vehicle moves
 * @note Used for MAVLink MISSION_CURRENT and position target messages
 * @note Velocity parameter passed to underlying function but not returned to caller
 * 
 * @see AP_Follow::get_target_location_and_velocity_ofs() for implementation
 */
bool ModeFollow::get_wp(Location &loc) const
{
    Vector3f vel;
    return g2.follow.get_target_location_and_velocity_ofs(loc, vel);
}

#endif // MODE_FOLLOW_ENABLED
