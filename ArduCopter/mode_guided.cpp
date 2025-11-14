/**
 * @file mode_guided.cpp
 * @brief Guided flight mode implementation for ArduCopter
 * 
 * @details Guided mode enables external control of the vehicle through a Ground Control
 * Station (GCS), companion computer, or scripting API. This mode is fundamental for
 * autonomous operations, waypoint navigation commanded from GCS, and integration with
 * high-level control systems like DroneKit, MAVSDK, and ROS.
 * 
 * Guided mode supports multiple control paradigms:
 * - Position control: Fly to specific lat/lon/alt coordinates
 * - Velocity control: Maintain specified NED velocity vector
 * - Acceleration control: Follow acceleration commands
 * - Combined pos/vel/accel: Trajectory following with feedforward
 * - Attitude/angle control: Direct attitude commands with climb rate or thrust
 * 
 * MAVLink Integration:
 * Guided mode is primarily controlled via MAVLink SET_POSITION_TARGET_* messages:
 * - SET_POSITION_TARGET_LOCAL_NED: Local frame (North-East-Down relative to EKF origin)
 * - SET_POSITION_TARGET_GLOBAL_INT: Global frame (lat/lon/alt)
 * - SET_ATTITUDE_TARGET: Direct attitude control with quaternion or rates
 * 
 * Coordinate Frame Handling:
 * - Global coordinates: Latitude/longitude in degrees * 1E7, altitude in mm
 * - Local NED: North-East-Down in cm relative to EKF origin
 * - Body frame: For angular rates and some velocity commands
 * - Terrain following: Alt-above-terrain using rangefinder when available
 * 
 * Yaw Control Options:
 * - Fixed yaw angle (absolute or relative to current heading)
 * - Yaw rate (rotation in rad/s)
 * - Combined angle + rate for smooth transitions
 * - Auto yaw (face direction of travel)
 * - Pilot control (optional, controlled by GUID_OPTIONS)
 * 
 * External Control APIs:
 * This mode is used by:
 * - MAVProxy and Mission Planner for interactive control
 * - DroneKit Python/Android for application development
 * - MAVSDK C++/Python for cross-platform autonomy
 * - ROS/ROS2 via mavros and AP_DDS integration
 * - Lua scripting via AP_Scripting
 * 
 * Guided Sub-Modes:
 * - TakeOff: Automated takeoff to specified altitude
 * - WP: Waypoint navigation with path planning (optional)
 * - Pos: Position hold at target location
 * - VelAccel: Velocity control with acceleration feedforward
 * - Accel: Pure acceleration control
 * - PosVelAccel: Full trajectory control with pos/vel/accel
 * - Angle: Attitude control with climb rate or direct thrust
 * 
 * Key Distinctions:
 * - Waypoint (WP) mode uses path planning and obstacle avoidance
 * - Position (Pos) mode directly controls position without path planning
 * - Velocity mode enables direct velocity control for reactive behaviors
 * - Angle mode provides low-level attitude control for acrobatic maneuvers
 * 
 * Safety Features:
 * - Geofence checking on destination commands
 * - Timeout protection (reverts to position hold if no updates received)
 * - Altitude and horizontal distance limits (when invoked from AUTO mode)
 * - Arming restrictions configurable via GUID_OPTIONS parameter
 * 
 * @note This mode is called at the main loop rate (typically 400Hz)
 * @warning Guided mode bypasses many autonomous safety checks - external controller
 *          is responsible for collision avoidance and safe operation
 * 
 * @see GCS_MAVLINK_Copter.cpp for MAVLink message handlers
 * @see mode.h for Mode base class and common mode functionality
 * 
 * Source: ArduCopter/mode_guided.cpp
 */

#include "Copter.h"

#if MODE_GUIDED_ENABLED

// Guided mode target state variables (NED frame relative to EKF origin)
// These variables store the current commanded targets from GCS/companion computer
static Vector3p guided_pos_target_cm;       // position target in cm (used by Pos and PosVelAccel sub-modes)
static bool guided_pos_terrain_alt;         // true if guided_pos_target_cm.z is altitude above terrain (only for Pos/WP modes)
static Vector3f guided_vel_target_cms;      // velocity target in cm/s (used by VelAccel and PosVelAccel sub-modes)
static Vector3f guided_accel_target_cmss;   // acceleration feedforward in cm/s² (used by Accel, VelAccel, and PosVelAccel sub-modes)
static uint32_t update_time_ms;             // system time (ms) of last target update - used for timeout detection

// Guided angle control state (Angle sub-mode only)
// Stores commanded attitude and vertical control for direct attitude control mode
struct {
    uint32_t update_time_ms;        // time of last angle command (ms) - for timeout detection
    Quaternion attitude_quat;       // desired attitude quaternion (earth to body frame), zero = rate control mode
    Vector3f ang_vel_body;          // desired angular velocity in body frame (rad/s) - used as feedforward
    float climb_rate_cms;           // desired climb rate in cm/s (NED frame, positive=down). Used if use_thrust is false
    float thrust;                   // desired normalized thrust [0.0 to 1.0]. Used if use_thrust is true
    bool use_thrust;                // true = direct thrust control, false = climb rate control (altitude hold)
} static guided_angle_state;

// Guided limit structure - used when guided is invoked from AUTO mode (NAV_GUIDED_ENABLE)
// Provides safety boundaries for external control within autonomous missions
struct Guided_Limit {
    uint32_t timeout_ms;    // timeout in milliseconds from guided start (0 = no timeout)
    float alt_min_cm;       // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;       // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm;     // horizontal position limit in cm from start position (0 = no limit)
    uint32_t start_time;    // system time (ms) when guided control was initiated
    Vector3f start_pos;     // start position in NED frame relative to home (cm) - reference for horiz_max check
} static guided_limit;

// Guided sub-mode state - controls which controller runs in the main run() loop
ModeGuided::SubMode ModeGuided::guided_mode = SubMode::TakeOff;  // current guided sub-mode (TakeOff, WP, Pos, Accel, VelAccel, PosVelAccel, Angle)
bool ModeGuided::send_notification;     // flag to send one-time MISSION_ITEM_REACHED message to GCS when WP sub-mode reaches destination
bool ModeGuided::takeoff_complete;      // true once takeoff has completed - triggers landing gear retraction and fence enable

// Guided mode pause state - allows external pause/resume of guided flight
bool ModeGuided::_paused;               // true when guided mode is paused (holds position, ignores new commands)

/**
 * @brief Initialize guided flight mode
 * 
 * @details Called when the vehicle transitions into guided mode. Initializes the mode
 * to velocity/acceleration control (SubMode::VelAccel) with zero velocity and acceleration
 * targets. This provides a safe starting state where the vehicle will hold position until
 * receiving external commands.
 * 
 * The velocity/acceleration control mode is chosen as default because:
 * - Allows immediate response to velocity commands from GCS/companion computer
 * - Provides smooth transitions from position hold
 * - Compatible with most MAVLink control libraries (DroneKit, MAVSDK)
 * 
 * Initialization sequence:
 * 1. Start velocity/acceleration control sub-mode
 * 2. Zero velocity and acceleration targets (vehicle will hold current position)
 * 3. Clear notification flag (prevents spurious waypoint-reached messages)
 * 4. Unpause the mode (if previously paused)
 * 
 * @param[in] ignore_checks If true, skip pre-arm and other safety checks (typically
 *                          false for normal mode entry, true for emergency situations)
 * 
 * @return Always returns true (guided mode initialization cannot fail)
 * 
 * @note After init(), the mode waits for external commands via MAVLink. Without commands,
 *       the vehicle will maintain position using the position controller.
 * @note This is called by the mode selection logic in Copter::set_mode()
 * 
 * Source: ArduCopter/mode_guided.cpp:42-54
 */
bool ModeGuided::init(bool ignore_checks)
{
    // start in velaccel control mode
    velaccel_control_start();
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    send_notification = false;

    // clear pause state when entering guided mode
    _paused = false;

    return true;
}

/**
 * @brief Main guided mode update function - called every loop iteration
 * 
 * @details This is the primary execution function for guided mode, called at the main
 * loop rate (typically 400Hz). It dispatches to the appropriate sub-mode controller
 * based on the current guided_mode state, which is set by external commands via MAVLink.
 * 
 * Guided Sub-Mode Dispatch:
 * - TakeOff: Automated vertical climb to commanded altitude
 * - WP: Waypoint navigation with path planning and obstacle avoidance
 * - Pos: Direct position control to hold at target location
 * - Accel: Pure acceleration control (for dynamic maneuvers)
 * - VelAccel: Velocity control with acceleration feedforward
 * - PosVelAccel: Full trajectory following (position + velocity + acceleration)
 * - Angle: Direct attitude control with climb rate or thrust
 * 
 * Pause Functionality:
 * If the mode is paused (via pause() method), runs pause_control_run() which
 * commands zero horizontal velocity and stops yaw rotation while maintaining altitude.
 * This is used for temporary interruption of guided commands.
 * 
 * Waypoint Notification:
 * When in WP sub-mode and the waypoint is reached, sends a MISSION_ITEM_REACHED
 * MAVLink message to the GCS. This provides feedback for sequential waypoint navigation.
 * 
 * @note Called at main loop rate (typically 400Hz) from Copter::update_flight_mode()
 * @note The sub-mode is changed by MAVLink command handlers calling functions like
 *       set_destination(), set_velocity(), set_angle(), etc.
 * 
 * @warning This function must complete quickly to maintain real-time loop timing.
 *          Heavy computation should be avoided here.
 * 
 * Source: ArduCopter/mode_guided.cpp:58-104
 */
void ModeGuided::run()
{
    // run pause control if the vehicle is paused
    if (_paused) {
        pause_control_run();
        return;
    }

    // call the correct auto controller
    switch (guided_mode) {

    case SubMode::TakeOff:
        // run takeoff controller
        takeoff_run();
        break;

    case SubMode::WP:
        // run waypoint controller
        wp_control_run();
        if (send_notification && wp_nav->reached_wp_destination()) {
            send_notification = false;
            gcs().send_mission_item_reached_message(0);
        }
        break;

    case SubMode::Pos:
        // run position controller
        pos_control_run();
        break;

    case SubMode::Accel:
        accel_control_run();
        break;

    case SubMode::VelAccel:
        velaccel_control_run();
        break;

    case SubMode::PosVelAccel:
        posvelaccel_control_run();
        break;

    case SubMode::Angle:
        angle_control_run();
        break;
    }
 }

// returns true if the Guided-mode-option is set (see GUID_OPTIONS)
bool ModeGuided::option_is_enabled(Option option) const
{
    return (copter.g2.guided_options.get() & (uint32_t)option) != 0;
}

bool ModeGuided::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station or scripting
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return option_is_enabled(Option::AllowArmingFromTX);
};

#if WEATHERVANE_ENABLED
bool ModeGuided::allows_weathervaning() const
{
    return option_is_enabled(Option::AllowWeatherVaning);
}
#endif

// initialises position controller to implement take-off
// takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain if a rangefinder is available
bool ModeGuided::do_user_takeoff_start(float takeoff_alt_cm)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
#if AP_RANGEFINDER_ENABLED
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm < copter.rangefinder.max_distance_orient(ROTATION_PITCH_270)*100) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else
#endif
    {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    guided_mode = SubMode::TakeOff;

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

    // clear i term when we're taking off
    pos_control->init_U_controller();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff.start(alt_target_cm, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

// initialise guided mode's waypoint navigation controller
void ModeGuided::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_cm();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point_NEU_cm(stopping_point);
    if (!wp_nav->set_wp_destination_NEU_cm(stopping_point, false)) {
        // this should never happen because terrain data is not used
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

/**
 * @brief Run guided waypoint navigation controller
 * 
 * @details Executes waypoint navigation with full path planning, speed profiling,
 * and optional obstacle avoidance. This sub-mode is selected when GUID_OPTIONS
 * includes WPNavUsedForPosControl flag, providing smoother, more predictable
 * trajectories suitable for mission-style waypoint following.
 * 
 * Waypoint Navigation Features:
 * - S-curve trajectory generation for smooth acceleration/deceleration
 * - Corner cutting with configurable WP_RADIUS
 * - Speed limiting based on WPNAV_SPEED, WPNAV_SPEED_UP, WPNAV_SPEED_DN
 * - Terrain following if enabled (maintains constant height above terrain)
 * - Object avoidance integration (if AP_AVOIDANCE enabled)
 * 
 * Control Architecture:
 * 1. wp_nav->update_wpnav(): Updates desired position/velocity along path
 * 2. pos_control->update_U_controller(): Vertical position/velocity controller
 * 3. attitude_control->input_thrust_vector_heading(): Converts to motor commands
 * 
 * Path Planning vs Direct Position:
 * Unlike Pos sub-mode which commands position directly, WP sub-mode:
 * - Plans smooth paths between waypoints
 * - Limits acceleration and jerk for passenger comfort
 * - Enables corner cutting for efficient multi-waypoint navigation
 * - Integrates object avoidance (bendy ruler, simple avoidance)
 * - Provides predictable ETAs for mission planning
 * 
 * Terrain Following:
 * If terrain following enabled (via TERRAIN_ENABLE and TERRAIN_FOLLOW):
 * - Maintains constant altitude above ground using terrain database or rangefinder
 * - Failsafe triggers if terrain data becomes unavailable
 * - Critical for low-altitude operations over varied terrain
 * 
 * Auto Yaw Behavior:
 * Yaw control managed by auto_yaw object, typically set to:
 * - Face direction of travel (default, smooth yaw as path curves)
 * - Fixed heading (if commanded via set_yaw_state_rad)
 * - ROI/look-at point (if DO_SET_ROI command active)
 * 
 * Disarmed/Landed Handling:
 * If vehicle is disarmed or on ground:
 * - Zeros throttle for safety
 * - Traditional helicopters maintain rotor speed if interlock enabled
 * - Prevents ground resonance and rotor droop
 * 
 * @note Called at 400Hz when guided_mode == SubMode::WP
 * @note Slower command update rates (<10Hz) acceptable due to path planning
 * @note Waypoint reached when within WP_RADIUS horizontally and WP_RADIUS_Z vertically
 * 
 * @warning Terrain following requires valid terrain data - mission fails if unavailable
 * @warning Object avoidance may alter path - actual trajectory differs from commanded waypoint
 * 
 * Source: ArduCopter/mode_guided.cpp:201-221
 */
void ModeGuided::wp_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// initialise position controller
void ModeGuided::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // initialise velocity controller
    pos_control->init_U_controller();
    pos_control->init_NE_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    guided_pos_terrain_alt = false;
}

// initialise guided mode's position controller
void ModeGuided::pos_control_start()
{
    // set to position control mode
    guided_mode = SubMode::Pos;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's acceleration controller
void ModeGuided::accel_control_start()
{
    // set guided_mode to acceleration controller
    guided_mode = SubMode::Accel;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's velocity and acceleration controller
void ModeGuided::velaccel_control_start()
{
    // set guided_mode to velocity and acceleration controller
    guided_mode = SubMode::VelAccel;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's position, velocity and acceleration controller
void ModeGuided::posvelaccel_control_start()
{
    // set guided_mode to position, velocity and acceleration controller
    guided_mode = SubMode::PosVelAccel;

    // initialise position controller
    pva_control_start();
}

bool ModeGuided::is_taking_off() const
{
    return guided_mode == SubMode::TakeOff && !takeoff_complete;
}

bool ModeGuided::set_speed_xy_cms(float speed_xy_cms)
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_NE_cm(speed_xy_cms, wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(speed_xy_cms, wp_nav->get_wp_acceleration_cmss());
    return true;
}

bool ModeGuided::set_speed_up_cms(float speed_up_cms)
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), speed_up_cms, wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), speed_up_cms, wp_nav->get_accel_U_cmss());
    return true;
}

bool ModeGuided::set_speed_down_cms(float speed_down_cms)
{
    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_U_cm(speed_down_cms, wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(speed_down_cms, wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    return true;
}

// initialise guided mode's angle controller
void ModeGuided::angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::Angle;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());
    pos_control->set_correction_speed_accel_U_cmss(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_U_cmss());

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.attitude_quat.from_euler(Vector3f(0.0, 0.0, attitude_control->get_att_target_euler_rad().z));
    guided_angle_state.ang_vel_body.zero();
    guided_angle_state.climb_rate_cms = 0.0f;
}

/**
 * @brief Set guided mode target destination in local NED coordinates
 * 
 * @details Commands the vehicle to fly to a specific position specified in local
 * North-East-Down coordinates relative to the EKF origin. This is the primary position
 * command interface for guided mode, used by MAVLink SET_POSITION_TARGET_LOCAL_NED
 * messages and companion computer control APIs.
 * 
 * Coordinate Frame:
 * - North-East-Down (NED) frame centered at EKF origin (typically home position)
 * - Units: centimeters
 * - Z-axis: Positive DOWN (increasing Z = lower altitude)
 * 
 * Sub-Mode Selection:
 * Automatically selects between two position control strategies based on GUID_OPTIONS:
 * - WP sub-mode: Uses AC_WPNav for path planning and obstacle avoidance (slower updates)
 * - Pos sub-mode: Direct position control without path planning (faster, reactive)
 * 
 * Terrain Following:
 * If terrain_alt=true and rangefinder available, interprets destination.z as altitude
 * above terrain rather than above EKF origin. Enables low-altitude terrain following.
 * 
 * Yaw Control Options:
 * - use_yaw=false: Face direction of travel (auto yaw)
 * - use_yaw=true, relative_yaw=false: Face absolute yaw angle
 * - use_yaw=true, relative_yaw=true: Rotate by yaw_rad from current heading
 * - use_yaw_rate=true: Rotate continuously at yaw_rate_rads
 * 
 * Fence Checking:
 * Validates destination against geofence boundaries. If fence is enabled and destination
 * is outside fence, command is rejected and false is returned. This prevents commanded
 * positions from violating safety boundaries.
 * 
 * @param[in] destination     Target position in NED frame (cm) relative to EKF origin
 * @param[in] use_yaw         If true, use yaw_rad parameter for yaw control
 * @param[in] yaw_rad         Target yaw angle in radians (0 = North, PI/2 = East)
 * @param[in] use_yaw_rate    If true, use yaw_rate_rads for continuous rotation
 * @param[in] yaw_rate_rads   Yaw rotation rate in radians/second
 * @param[in] relative_yaw    If true, yaw_rad is relative to current heading
 * @param[in] terrain_alt     If true, interpret destination.z as altitude above terrain
 * 
 * @return true if destination accepted and within fence boundaries, false if rejected
 * 
 * @note Typically called at 10-50Hz by external controller. High-rate updates (>50Hz)
 *       should use velocity control instead for better performance.
 * @note Switches mode to SubMode::WP or SubMode::Pos depending on GUID_OPTIONS
 * 
 * @warning If fence is enabled, ensure destination is within fence or command will fail
 * @warning Terrain following requires functioning rangefinder with terrain data available
 * 
 * Source: ArduCopter/mode_guided.cpp:339-414
 */
bool ModeGuided::set_destination(const Vector3f& destination, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool terrain_alt)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if configured to use wpnav for position control
    if (use_wpnav_for_position_control()) {
        // ensure we are in position control mode
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        // set yaw state
        set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination_NEU_cm(destination, terrain_alt);

#if HAL_LOGGING_ENABLED
        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, destination, terrain_alt, Vector3f(), Vector3f());
#endif
        send_notification = true;
        return true;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // initialise terrain following if needed
    if (terrain_alt) {
        // get current alt above terrain
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset_cm(origin_terr_offset)) {
            // if we don't have terrain altitude then stop
            init(true);
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!guided_pos_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->init_pos_terrain_U_cm(origin_terr_offset);
        }
    } else {
        pos_control->init_pos_terrain_U_cm(0.0);
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set position target and zero velocity and acceleration
    guided_pos_target_cm = destination.topostype();
    guided_pos_terrain_alt = terrain_alt;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
#endif

    send_notification = true;

    return true;
}

bool ModeGuided::get_wp(Location& destination) const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::Pos:
        destination = Location(guided_pos_target_cm.tofloat(), guided_pos_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        return true;
    case SubMode::Angle:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        break;
    }

    return false;
}

/**
 * @brief Set guided mode target destination using global GPS coordinates
 * 
 * @details Commands the vehicle to fly to a specific global position specified as
 * latitude/longitude/altitude. This is used by MAVLink SET_POSITION_TARGET_GLOBAL_INT
 * messages and enables waypoint navigation using GPS coordinates.
 * 
 * Coordinate Frame:
 * - Latitude/longitude: WGS84 datum, degrees * 1E7 (as per MAVLink specification)
 * - Altitude frame: Determined by Location::AltFrame in dest_loc
 *   * ABOVE_HOME: Altitude in cm above home position
 *   * ABOVE_ORIGIN: Altitude in cm above EKF origin
 *   * ABOVE_TERRAIN: Altitude in cm above terrain (requires terrain database)
 * 
 * Location Conversion:
 * Internally converts global coordinates to local NED frame using wp_nav->get_vector_NEU_cm().
 * This conversion requires valid EKF solution and may fail if:
 * - EKF origin not initialized
 * - Terrain data unavailable (for ABOVE_TERRAIN frame)
 * - GPS lock lost or degraded
 * 
 * Sub-Mode Selection:
 * Like the Vector3f version, selects between WP and Pos sub-modes based on GUID_OPTIONS:
 * - WP mode: Full waypoint navigation with S-curves, speed profiles, obstacle avoidance
 * - Pos mode: Direct position control for responsive, high-update-rate applications
 * 
 * Terrain Altitude Handling:
 * If dest_loc uses ABOVE_TERRAIN frame, requires terrain data from:
 * - Rangefinder (real-time terrain following)
 * - Terrain database (pre-loaded elevation data)
 * Command fails if terrain data unavailable.
 * 
 * @param[in] dest_loc        Target location with lat/lon/alt in global frame
 * @param[in] use_yaw         If true, use yaw_rad parameter for yaw control
 * @param[in] yaw_rad         Target yaw angle in radians (0 = North)
 * @param[in] use_yaw_rate    If true, use yaw_rate_rads for continuous rotation
 * @param[in] yaw_rate_rads   Yaw rotation rate in radians/second
 * @param[in] relative_yaw    If true, yaw_rad is offset from current heading
 * 
 * @return true if destination accepted and converted successfully
 *         false if outside fence, terrain data missing, or coordinate conversion failed
 * 
 * @note This is the function used by Mission Planner "Fly To" and similar GCS commands
 * @note For high-rate control (>10Hz), use local NED coordinates to avoid conversion overhead
 * 
 * @warning Returns false if fence enabled and destination outside fence boundaries
 * @warning May fail if terrain data required but unavailable
 * @warning Coordinate conversion can fail if EKF not fully initialized
 * 
 * Source: ArduCopter/mode_guided.cpp:438-523
 */
bool ModeGuided::set_destination(const Location& dest_loc, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if using wpnav for position control
    if (use_wpnav_for_position_control()) {
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        if (!wp_nav->set_wp_destination_loc(dest_loc)) {
            // failure to set destination can only be because of missing terrain data
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
            // failure is propagated to GCS with NAK
            return false;
        }

        // set yaw state
        set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

#if HAL_LOGGING_ENABLED
        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), (dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
#endif

        send_notification = true;
        return true;
    }

    // set position target and zero velocity and acceleration
    Vector3f pos_target_f;
    bool terrain_alt;
    if (!wp_nav->get_vector_NEU_cm(dest_loc, pos_target_f, terrain_alt)) {
        return false;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // initialise terrain following if needed
    if (terrain_alt) {
        // get current alt above terrain
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset_cm(origin_terr_offset)) {
            // if we don't have terrain altitude then stop
            init(true);
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!guided_pos_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->init_pos_terrain_U_cm(origin_terr_offset);
        }
    } else {
        pos_control->init_pos_terrain_U_cm(0.0);
    }

    guided_pos_target_cm = pos_target_f.topostype();
    guided_pos_terrain_alt = terrain_alt;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    update_time_ms = millis();

    // log target
#if HAL_LOGGING_ENABLED
    copter.Log_Write_Guided_Position_Target(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
#endif

    send_notification = true;

    return true;
}

// set_velaccel - sets guided mode's target velocity and acceleration
void ModeGuided::set_accel(const Vector3f& acceleration, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool log_request)
{
    // check we are in acceleration control mode
    if (guided_mode != SubMode::Accel) {
        accel_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set velocity and acceleration targets and zero position
    guided_pos_target_cm.zero();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss = acceleration;
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // log target
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    }
#endif
}

/**
 * @brief Set guided mode target velocity in NED frame
 * 
 * @details Commands the vehicle to maintain a specified velocity vector in the
 * North-East-Down frame. This is the primary interface for velocity control mode,
 * enabling reactive behaviors and direct velocity command from companion computers.
 * 
 * Used by MAVLink SET_POSITION_TARGET_LOCAL_NED messages with velocity fields and
 * type_mask indicating velocity control. Common in:
 * - DroneKit velocity control commands
 * - MAVSDK offboard velocity mode
 * - ROS mavros velocity setpoints
 * - Reactive obstacle avoidance systems
 * 
 * Control Behavior:
 * - Vehicle attempts to maintain commanded velocity vector
 * - Position controller still active (unless disabled via GUID_OPTIONS)
 * - Zero velocity = position hold at current location
 * - Smooth transitions between velocity commands
 * 
 * Coordinate Frame:
 * - North-East-Down (NED) frame relative to EKF origin
 * - Units: cm/s
 * - Z velocity: Positive DOWN (positive Z velocity = descend)
 * 
 * Update Rate:
 * Designed for high-rate updates (10-100Hz). Timeout after GUID_TIMEOUT seconds
 * (default 3s) causes velocity to be zeroed and vehicle holds position.
 * 
 * Position Stabilization:
 * By default, position controller runs underneath velocity control to prevent drift.
 * Can be disabled via GUID_OPTIONS for pure velocity control without position correction.
 * 
 * @param[in] velocity        Desired velocity vector in NED frame (cm/s)
 * @param[in] use_yaw         If true, use yaw_rad parameter for yaw control
 * @param[in] yaw_rad         Target yaw angle in radians
 * @param[in] use_yaw_rate    If true, use yaw_rate_rads for continuous rotation
 * @param[in] yaw_rate_rads   Yaw rotation rate in radians/second
 * @param[in] relative_yaw    If true, yaw_rad is relative to current heading
 * @param[in] log_request     If true, log this command to dataflash
 * 
 * @note This is a convenience wrapper that calls set_velaccel() with zero acceleration
 * @note Velocity commands should be sent continuously at 10-50Hz for smooth control
 * @note Vehicle will hold position if no velocity commands received for >GUID_TIMEOUT seconds
 * 
 * @warning Ensure velocity limits are reasonable for vehicle capabilities
 * @warning High velocities near obstacles require external collision avoidance
 * @warning Pure velocity control (without position stabilization) can lead to drift
 * 
 * Source: ArduCopter/mode_guided.cpp:552-555
 */
void ModeGuided::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool log_request)
{
    set_velaccel(velocity, Vector3f(), use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw, log_request);
}

// set_velaccel - sets guided mode's target velocity and acceleration
void ModeGuided::set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw, bool log_request)
{
    // check we are in velocity and acceleration control mode
    if (guided_mode != SubMode::VelAccel) {
        velaccel_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    // set velocity and acceleration targets and zero position
    guided_pos_target_cm.zero();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms = velocity;
    guided_accel_target_cmss = acceleration;
    update_time_ms = millis();

#if HAL_LOGGING_ENABLED
    // log target
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    }
#endif
}

// set_destination_posvel - set guided mode position and velocity target
bool ModeGuided::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw)
{
    return set_destination_posvelaccel(destination, velocity, Vector3f(), use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);
}

// set_destination_posvelaccel - set guided mode position, velocity and acceleration target
bool ModeGuided::set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // check we are in position, velocity and acceleration control mode
    if (guided_mode != SubMode::PosVelAccel) {
        posvelaccel_control_start();
    }

    // set yaw state
    set_yaw_state_rad(use_yaw, yaw_rad, use_yaw_rate, yaw_rate_rads, relative_yaw);

    update_time_ms = millis();
    guided_pos_target_cm = destination.topostype();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms = velocity;
    guided_accel_target_cmss = acceleration;

#if HAL_LOGGING_ENABLED
    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
#endif
    return true;
}

// returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
bool ModeGuided::set_attitude_target_provides_thrust() const
{
    return option_is_enabled(Option::SetAttitudeTarget_ThrustAsThrust);
}

// returns true if GUIDED_OPTIONS param specifies position should be controlled (when velocity and/or acceleration control is active)
bool ModeGuided::stabilizing_pos_xy() const
{
    return !option_is_enabled(Option::DoNotStabilizePositionXY);
}

// returns true if GUIDED_OPTIONS param specifies velocity should  be controlled (when acceleration control is active)
bool ModeGuided::stabilizing_vel_xy() const
{
    return !option_is_enabled(Option::DoNotStabilizeVelocityXY);
}

// returns true if GUIDED_OPTIONS param specifies waypoint navigation should be used for position control (allow path planning to be used but updates must be slower)
bool ModeGuided::use_wpnav_for_position_control() const
{
    return option_is_enabled(Option::WPNavUsedForPosControl);
}

/**
 * @brief Set guided mode attitude target with angular rates and climb rate or thrust
 * 
 * @details Provides low-level attitude control for guided mode, enabling direct
 * commanding of vehicle orientation using quaternions and body-frame angular velocities.
 * This is used by MAVLink SET_ATTITUDE_TARGET messages and enables acrobatic maneuvers,
 * attitude stabilization from external controllers, and precise orientation control.
 * 
 * Control Modes:
 * Two distinct attitude control behaviors based on attitude_quat parameter:
 * 
 * 1. Rate-Only Control (attitude_quat is zero/identity):
 *    - Commands body-frame angular velocities directly (rad/s)
 *    - No attitude stabilization, pure rate command
 *    - Used for acrobatic flight, rate-based controllers
 *    - Similar to ACRO mode but with vertical position hold or thrust control
 * 
 * 2. Attitude + Rate Control (attitude_quat non-zero):
 *    - Commands target attitude quaternion with feedforward angular velocity
 *    - Attitude controller stabilizes to target orientation
 *    - Angular velocity provides rate feedforward for smooth tracking
 *    - Used for precise orientation control, camera pointing, coordinated turns
 * 
 * Vertical Control Options:
 * Configured by use_thrust flag and GUID_OPTIONS parameter:
 * - use_thrust=false: climb_rate_cms_or_thrust interpreted as vertical velocity (cm/s)
 *   * Vertical position controller maintains altitude rate
 *   * Safer, automatically levels off at altitude limits
 * - use_thrust=true: climb_rate_cms_or_thrust interpreted as normalized thrust [0,1]
 *   * Direct motor thrust control, no altitude stabilization
 *   * Enables more aggressive vertical maneuvers
 *   * Requires careful management to avoid altitude excursions
 * 
 * Quaternion Representation:
 * - Attitude represented as rotation from NED frame to body frame
 * - w,x,y,z components in standard Hamilton convention
 * - Zero/identity quaternion (w=1, x=y=z=0) triggers rate-only mode
 * 
 * Body Frame Angular Velocity:
 * - X: Roll rate (right wing down = positive)
 * - Y: Pitch rate (nose up = positive)
 * - Z: Yaw rate (nose right = positive)
 * - Units: radians/second
 * - Body frame: Right-handed coordinate system fixed to vehicle
 * 
 * Common Use Cases:
 * - Camera gimbal compensation: Point camera at ground target while moving
 * - Coordinated turns: Bank angle with appropriate yaw rate
 * - External attitude controller: Model Predictive Control, Neural Network control
 * - Acrobatic maneuvers: Flips, rolls with altitude hold
 * - Formation flight: Maintain relative orientation to leader
 * 
 * @param[in] attitude_quat           Target attitude quaternion (NED to body frame)
 *                                    Zero quaternion triggers rate-only control
 * @param[in] ang_vel_body            Body-frame angular velocity vector (rad/s)
 *                                    X=roll rate, Y=pitch rate, Z=yaw rate
 * @param[in] climb_rate_cms_or_thrust Vertical control: climb rate (cm/s) or thrust [0,1]
 *                                     Interpretation determined by use_thrust parameter
 * @param[in] use_thrust              true: climb_rate_cms_or_thrust is thrust [0,1]
 *                                    false: climb_rate_cms_or_thrust is climb rate (cm/s)
 * 
 * @note Requires continuous updates at >10Hz to maintain control authority
 * @note Timeout after GUID_TIMEOUT seconds returns to level attitude with zero climb rate
 * @note Switching between thrust and climb rate control reinitializes position controller
 * 
 * @warning Direct thrust control (use_thrust=true) bypasses altitude safety limits
 * @warning High angular rates can exceed vehicle stability limits - test carefully
 * @warning Rate-only control provides no attitude stabilization - vehicle may drift
 * @warning Pilot should be ready to take over in case of external controller failure
 * 
 * Source: ArduCopter/mode_guided.cpp:654-686
 */
void ModeGuided::set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel_body, float climb_rate_cms_or_thrust, bool use_thrust)
{
    // check we are in velocity control mode
    if (guided_mode != SubMode::Angle) {
        angle_control_start();
    } else if (!use_thrust && guided_angle_state.use_thrust) {
        // Already angle control but changing from thrust to climb rate
        pos_control->init_U_controller();
    }

    guided_angle_state.attitude_quat = attitude_quat;
    guided_angle_state.ang_vel_body = ang_vel_body;

    guided_angle_state.use_thrust = use_thrust;
    if (use_thrust) {
        guided_angle_state.thrust = climb_rate_cms_or_thrust;
        guided_angle_state.climb_rate_cms = 0.0f;
    } else {
        guided_angle_state.thrust = 0.0f;
        guided_angle_state.climb_rate_cms = climb_rate_cms_or_thrust;
    }

    guided_angle_state.update_time_ms = millis();

    // convert quaternion to euler angles
    float roll_rad, pitch_rad, yaw_rad;
    attitude_quat.to_euler(roll_rad, pitch_rad, yaw_rad);

#if HAL_LOGGING_ENABLED
    // log target
    copter.Log_Write_Guided_Attitude_Target(guided_mode, roll_rad, pitch_rad, yaw_rad, ang_vel_body, guided_angle_state.thrust, guided_angle_state.climb_rate_cms * 0.01);
#endif
}

/**
 * @brief Execute guided mode takeoff maneuver
 * 
 * @details Runs the automated takeoff sequence in guided mode, climbing vertically
 * to the commanded altitude. This is invoked by MAVLink TAKEOFF command or
 * do_user_takeoff_start() and provides a safe, controlled vertical ascent.
 * 
 * Takeoff Sequence:
 * 1. Vertical climb at configured takeoff speed (PILOT_SPEED_UP or WPNAV_SPEED_UP)
 * 2. Maintain horizontal position using position controller
 * 3. Hold current heading (yaw locked at takeoff orientation)
 * 4. Continue until target altitude reached
 * 
 * Altitude Target Interpretation:
 * - Can be alt-above-home or alt-above-terrain (if rangefinder available)
 * - Terrain following mode used if:
 *   * Rangefinder healthy and in use
 *   * Target altitude within rangefinder range
 *   * Enables takeoff from sloped terrain
 * 
 * Completion Actions:
 * When takeoff completes (target altitude reached):
 * - Sets takeoff_complete flag (triggers landing gear retraction if equipped)
 * - Auto-enables geofence (if configured with FENCE_AUTOENABLE)
 * - Vehicle remains in guided mode awaiting next command
 * 
 * Safety Features:
 * - WP_NAVALT_MIN altitude threshold prevents ground effect issues
 * - Position controller prevents horizontal drift during climb
 * - Smooth altitude controller prevents aggressive climb that could cause instability
 * 
 * Integration with Auto-Takeoff:
 * Uses copter.auto_takeoff helper which handles:
 * - Altitude ramping and speed profiling
 * - WP_NAVALT_MIN waypoint altitude logic
 * - Completion detection
 * - Terrain following when enabled
 * 
 * @note Called at main loop rate (400Hz) when guided_mode == SubMode::TakeOff
 * @note After takeoff completes, mode remains in TakeOff until external command changes it
 * @note Horizontal position is actively controlled - vehicle will not drift
 * 
 * @warning Do not use on slopes without rangefinder (may climb relative to home instead of ground)
 * @warning Ensure adequate clearance above takeoff point (no overhead obstacles)
 * 
 * Source: ArduCopter/mode_guided.cpp:690-703
 */
void ModeGuided::takeoff_run()
{
    auto_takeoff.run();
    if (auto_takeoff.complete && !takeoff_complete) {
        takeoff_complete = true;
#if AP_FENCE_ENABLED
        copter.fence.auto_enable_fence_after_takeoff();
#endif
#if AP_LANDINGGEAR_ENABLED
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();
#endif
    }
}

// pos_control_run - runs the guided position controller
// called from guided_run
void ModeGuided::pos_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments if target altitude is terrain-relative
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt && !wp_nav->get_terrain_offset_cm(terr_offset)) {
        // failure to get terrain data - trigger terrain failsafe
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Pos sub-mode is position-only control (velocity and acceleration are zero)
    // This creates a spring-like position hold behavior
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    // stop yaw rotation if no updates received within timeout (default 3 seconds)
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);  // freeze current yaw angle
        }
    }

    // Calculate vertical buffer for terrain following to prevent clipping obstacles
    float pos_offset_z_buffer = 0.0; // Vertical buffer size in cm
    if (guided_pos_terrain_alt) {
        // Use smaller of: configured terrain margin, or 50% of target altitude
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin_m() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_NEU_cm(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers (convert position error to acceleration commands)
    pos_control->update_NE_controller();  // horizontal (North-East)
    pos_control->update_U_controller();   // vertical (Up/altitude)

    // call attitude controller to convert acceleration commands to motor outputs
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// velaccel_control_run - runs the guided velocity controller
// called from guided_run
void ModeGuided::accel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // timeout handling - zero targets if no updates received for configured timeout (default 3 seconds)
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        // command timeout - go to safe state (zero velocity/acceleration)
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);  // freeze current yaw
        }
        // feed zero velocity/acceleration to controller to hold position
        pos_control->input_vel_accel_NE_cm(guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
        pos_control->input_vel_accel_U_cm(guided_vel_target_cms.z, guided_accel_target_cmss.z, false);
    } else {
        // Accel sub-mode: direct acceleration commands (open-loop acceleration control)
        // Position controller uses acceleration as feedforward only
        pos_control->input_accel_NE_cm(guided_accel_target_cmss);
        
        // Stabilization options allow disabling position/velocity feedback loops
        // (controlled by GUID_OPTIONS parameter bits)
        if (!stabilizing_vel_xy()) {
            // Disable both position and velocity stabilization (pure acceleration feedforward)
            pos_control->stop_vel_NE_stabilisation();
        } else if (!stabilizing_pos_xy()) {
            // Disable position stabilization only (velocity feedback still active)
            pos_control->stop_pos_NE_stabilisation();
        }
        pos_control->input_accel_U_cm(guided_accel_target_cmss.z);
    }

    // run position controllers (converts acceleration to attitude/thrust)
    pos_control->update_NE_controller();  // horizontal
    pos_control->update_U_controller();   // vertical

    // call attitude controller to generate motor outputs
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Run guided velocity and acceleration controller
 * 
 * @details Executes velocity control with acceleration feedforward, providing responsive
 * control for reactive behaviors and high-rate trajectory tracking. This is the default
 * guided sub-mode (set by init()) and most commonly used for companion computer control.
 * 
 * Control Law:
 * Implements cascaded position-velocity-acceleration control:
 * - Outer loop: Position stabilization (optional, controlled by GUID_OPTIONS)
 * - Middle loop: Velocity tracking with feedforward acceleration
 * - Inner loop: Attitude control to achieve desired thrust vector
 * 
 * Position Stabilization Options (via GUID_OPTIONS):
 * Three stabilization modes controlled by GUID_OPTIONS parameter:
 * 
 * 1. Full stabilization (default):
 *    - Position controller corrects for drift
 *    - Velocity controller tracks commanded velocity
 *    - Best for most applications, prevents runaway
 * 
 * 2. Velocity stabilization only (DoNotStabilizePositionXY):
 *    - No position correction, velocity tracking only
 *    - Prevents integration windup in pure velocity control
 *    - Used for applications where position reference is external
 * 
 * 3. No stabilization (DoNotStabilizePositionXY + DoNotStabilizeVelocityXY):
 *    - Pure acceleration feedforward only
 *    - Truly open-loop control
 *    - Expert mode for specialized control algorithms
 * 
 * Acceleration Feedforward:
 * Commanded acceleration (guided_accel_target_cmss) provides feedforward term:
 * - Improves tracking performance during aggressive maneuvers
 * - Reduces lag in velocity response
 * - Particularly effective for trajectory following
 * 
 * Timeout Protection:
 * If no velocity commands received for GUID_TIMEOUT seconds (default 3s):
 * - Zeros velocity and acceleration targets (vehicle holds position)
 * - Stops yaw rotation if in rate or angle-rate mode
 * - Provides safety net for communication loss or external controller failure
 * 
 * Obstacle Avoidance Integration:
 * If AP_AVOIDANCE enabled and obstacles detected:
 * - adjust_velocity() modifies commanded velocity to avoid obstacles
 * - Temporarily disables position/velocity stabilization during avoidance
 * - Uses simple avoidance (backup) or bendy-ruler path planning
 * - Ensures commanded velocities respect fence and proximity sensor limits
 * 
 * Coordinate Frame:
 * - Velocity: North-East-Down (NED) in cm/s relative to EKF origin
 * - Acceleration: NED in cm/s² (feedforward term)
 * - Down is positive Z (positive Z velocity = descend)
 * 
 * Update Rate Requirements:
 * - Designed for 10-50Hz update rate from external controller
 * - Higher rates (>50Hz) acceptable and improve performance
 * - Lower rates (<5Hz) may cause jerky motion, use waypoint mode instead
 * 
 * Common Applications:
 * - DroneKit vehicle.send_ned_velocity() commands
 * - MAVSDK Offboard velocity control
 * - ROS mavros /setpoint_velocity/cmd_vel topics
 * - Optical flow position hold
 * - GPS-denied navigation with external positioning
 * - Visual servoing and target tracking
 * 
 * @note Called at 400Hz when guided_mode == SubMode::VelAccel
 * @note Default sub-mode after init() - most common guided control mode
 * @note Zero velocity command causes position hold at current location
 * 
 * @warning Communication loss causes immediate position hold after GUID_TIMEOUT
 * @warning Disabling position stabilization can lead to position drift
 * @warning Ensure external controller sends continuous updates to prevent timeout
 * 
 * Source: ArduCopter/mode_guided.cpp:799-850
 */
void ModeGuided::velaccel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // timeout handling - zero targets if no updates received (default 3 seconds)
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        // command timeout - zero velocity and acceleration to hold position
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);  // freeze current yaw
        }
    }

    // integrate obstacle avoidance system if enabled
    bool do_avoid = false;
#if AP_AVOIDANCE_ENABLED
    // adjust velocity command to avoid obstacles and fence boundaries
    // avoidance system modifies guided_vel_target_cms directly
    copter.avoid.adjust_velocity(guided_vel_target_cms, pos_control->get_pos_NE_p().kP(), pos_control->get_max_accel_NE_cmss(), pos_control->get_pos_U_p().kP(), pos_control->get_max_accel_U_cmss(), G_Dt);
    do_avoid = copter.avoid.limits_active();  // check if avoidance is modifying commands
#endif

    // VelAccel sub-mode: velocity command with acceleration feedforward
    // Provides smooth acceleration transitions and better tracking

    // Handle stabilization options (position/velocity feedback loops)
    if (!stabilizing_vel_xy() && !do_avoid) {
        // No velocity stabilization: use current controller velocity as target (open-loop velocity)
        guided_vel_target_cms.xy() = pos_control->get_vel_desired_NEU_cms().xy();
    }
    pos_control->input_vel_accel_NE_cm(guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
    
    // Apply stabilization options (unless avoidance is active - avoidance needs full feedback)
    if (!stabilizing_vel_xy() && !do_avoid) {
        // disable both position and velocity feedback (pure velocity feedforward)
        pos_control->stop_vel_NE_stabilisation();
    } else if (!stabilizing_pos_xy() && !do_avoid) {
        // disable position feedback only (velocity feedback remains active)
        pos_control->stop_pos_NE_stabilisation();
    }
    pos_control->input_vel_accel_U_cm(guided_vel_target_cms.z, guided_accel_target_cmss.z, false);

    // run position controllers (converts velocity to attitude/thrust)
    pos_control->update_NE_controller();  // horizontal
    pos_control->update_U_controller();   // vertical

    // call attitude controller to generate motor outputs
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// pause_control_run - runs the guided mode pause controller
// called from guided_run
void ModeGuided::pause_control_run()
{
    // Pause mode: hold current position and ignore new guided commands
    // Activated via pause() method - used for temporary suspension of guided flight
    
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // command zero horizontal velocity and acceleration (holds current XY position)
    Vector2f vel_xy, accel_xy;  // default constructed to zero
    pos_control->input_vel_accel_NE_cm(vel_xy, accel_xy, false);

    // command zero vertical velocity and acceleration (holds current altitude)
    float vel_z = 0.0;
    pos_control->input_vel_accel_U_cm(vel_z, 0.0, false);

    // run position controllers to maintain position hold
    pos_control->update_NE_controller();  // horizontal
    pos_control->update_U_controller();   // vertical

    // call attitude controller with zero yaw rate (holds current heading)
    attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0);
}

/**
 * @brief Run guided position, velocity, and acceleration controller (full trajectory control)
 * 
 * @details Executes complete trajectory tracking with simultaneous position, velocity, and
 * acceleration commands. This enables the highest-fidelity trajectory following by providing
 * the complete desired state (position + velocity + acceleration) to the position controller.
 * 
 * Full Trajectory Control:
 * Provides all three trajectory components simultaneously:
 * - Position: Desired location in NED frame (cm)
 * - Velocity: Desired velocity at that position (cm/s)
 * - Acceleration: Desired acceleration (feedforward term, cm/s²)
 * 
 * This is the most sophisticated control mode, enabling:
 * - Minimum-snap trajectory following
 * - Time-optimal path execution
 * - Aggressive maneuvering with smooth dynamics
 * - Precise trajectory tracking for cinematography
 * - Formation flight with coordinated acceleration
 * 
 * Control Architecture:
 * Three-level cascaded controller with full state feedforward:
 * 1. Position error → desired velocity correction
 * 2. Velocity error + velocity setpoint → desired acceleration
 * 3. Acceleration setpoint + feedforward → thrust vector
 * 
 * Trajectory Generation:
 * Typically used with external trajectory planners that generate smooth paths:
 * - Polynomial trajectories (quintic, septic)
 * - Minimum-snap optimization
 * - Bezier curves with derivatives
 * - Time-parametrized splines
 * 
 * Stabilization Control (GUID_OPTIONS):
 * Three modes based on stabilization flags:
 * 
 * 1. Full stabilization (default):
 *    - Corrects for position and velocity tracking errors
 *    - Integral action prevents steady-state errors
 *    - Most robust, recommended for normal operation
 * 
 * 2. Velocity stabilization only (DoNotStabilizePositionXY):
 *    - Position reference tracks actual position
 *    - Only velocity error is controlled
 *    - Reduces overshoot in velocity tracking
 *    - Useful for velocity-centric applications
 * 
 * 3. No stabilization (both flags set):
 *    - Pure feedforward control
 *    - No error correction, open-loop tracking
 *    - Requires highly accurate trajectory generation
 *    - Expert mode only
 * 
 * Position Target Update:
 * Position target is continuously updated based on stabilization mode:
 * - Full stab: Position target maintained, errors integrated
 * - Vel stab: Position target follows actual position
 * - No stab: Position/velocity targets follow controller outputs
 * This prevents windup and ensures smooth control.
 * 
 * Timeout Behavior:
 * If no trajectory updates for GUID_TIMEOUT seconds (default 3s):
 * - Zeros velocity and acceleration (holds last commanded position)
 * - Stops yaw rotation
 * - Provides graceful degradation on communication loss
 * 
 * Coordinate Frames:
 * - Position: NED frame relative to EKF origin (cm)
 * - Velocity: NED frame (cm/s)
 * - Acceleration: NED frame (cm/s²)
 * - All in earth-fixed frame, not body frame
 * 
 * Terrain Altitude Restriction:
 * This mode does NOT support terrain-relative altitudes (guided_pos_terrain_alt must be false).
 * If terrain altitude is detected, triggers INTERNAL_ERROR.
 * Terrain following requires WP sub-mode instead.
 * 
 * Update Rate Requirements:
 * - Recommended: 50-100Hz for smooth trajectory tracking
 * - Minimum: 10Hz to prevent timeout
 * - Maximum: 400Hz (main loop rate)
 * High-rate updates critical for aggressive trajectories.
 * 
 * Common Applications:
 * - Motion capture / Vicon controlled flight
 * - Minimum-snap trajectory following
 * - Aggressive maneuver execution
 * - High-speed obstacle avoidance with planned trajectories
 * - Cinematic camera paths (smooth velocity profiles)
 * - Research platforms for advanced control algorithms
 * 
 * MAVLink Interface:
 * Commanded via SET_POSITION_TARGET_LOCAL_NED with type_mask indicating
 * position + velocity + acceleration fields are valid.
 * 
 * @note Called at 400Hz when guided_mode == SubMode::PosVelAccel
 * @note Most demanding control mode - requires accurate trajectory generation
 * @note Provides best tracking performance when properly tuned
 * 
 * @warning Terrain-relative altitude NOT supported in this mode
 * @warning Requires continuous high-rate updates (>10Hz) to prevent timeout
 * @warning Disabling stabilization requires expert-level trajectory planning
 * @warning Aggressive trajectories may exceed vehicle acceleration limits
 * 
 * Source: ArduCopter/mode_guided.cpp:884-939
 */
void ModeGuided::posvelaccel_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // timeout handling - zero velocity/acceleration if no updates received (default 3 seconds)
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        // command timeout - zero velocity and acceleration (position target remains)
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);  // freeze current yaw
        }
    }

    // PosVelAccel sub-mode: full trajectory control with position, velocity, and acceleration
    // This is the most comprehensive control mode, used for smooth trajectory following
    
    // Handle stabilization options for horizontal position/velocity
    if (!stabilizing_vel_xy()) {
        // No velocity stabilization: track current controller state (open-loop trajectory following)
        guided_pos_target_cm.xy() = pos_control->get_pos_desired_NEU_cm().xy();
        guided_vel_target_cms.xy() = pos_control->get_vel_desired_NEU_cms().xy();
    } else if (!stabilizing_pos_xy()) {
        // No position stabilization: track current position but use velocity feedback
        guided_pos_target_cm.xy() = pos_control->get_pos_desired_NEU_cm().xy();
    }
    pos_control->input_pos_vel_accel_NE_cm(guided_pos_target_cm.xy(), guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
    
    // Apply stabilization option flags
    if (!stabilizing_vel_xy()) {
        // disable both position and velocity feedback
        pos_control->stop_vel_NE_stabilisation();
    } else if (!stabilizing_pos_xy()) {
        // disable position feedback only
        pos_control->stop_pos_NE_stabilisation();
    }

    // PosVelAccel mode does not support terrain-relative altitudes (only absolute altitudes)
    // Terrain following would require continuous altitude adjustments incompatible with trajectory control
    if (guided_pos_terrain_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // vertical control: position, velocity, and acceleration
    float pz = guided_pos_target_cm.z;
    pos_control->input_pos_vel_accel_U_cm(pz, guided_vel_target_cms.z, guided_accel_target_cmss.z, false);
    guided_pos_target_cm.z = pz;  // update may have been modified by controller

    // run position controllers (converts trajectory to attitude/thrust)
    pos_control->update_NE_controller();  // horizontal
    pos_control->update_U_controller();   // vertical

    // call attitude controller to generate motor outputs
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

/**
 * @brief Run guided angle controller (direct attitude + climb rate/thrust control)
 * 
 * @details Executes direct attitude control with independent vertical control, bypassing
 * position and velocity controllers. Provides the most direct control authority over vehicle
 * orientation, used for acrobatic maneuvers, manual-like control, and specialized applications
 * requiring direct attitude commands.
 * 
 * Control Modes:
 * This controller supports two distinct control interfaces:
 * 
 * 1. Attitude + Angular Velocity Mode (attitude_quat non-zero):
 *    - Quaternion specifies desired orientation
 *    - Angular velocity (body frame) provides rate feedforward
 *    - Enables smooth attitude transitions
 *    - Used for coordinated maneuvers
 * 
 * 2. Pure Angular Velocity Mode (attitude_quat zero):
 *    - Direct body-frame angular velocity commands (rad/s)
 *    - Open-loop attitude control
 *    - Used for rate-based stick inputs or acrobatics
 *    - Vehicle attitude follows integration of rate commands
 * 
 * Vertical Control Options:
 * Configured via GUIDED_OPTIONS parameter bit SetAttitudeTarget_ThrustAsThrust:
 * 
 * A. Climb Rate Mode (use_thrust = false, default):
 *    - Vertical velocity controller maintains desired climb rate (cm/s)
 *    - Altitude controlled independently of attitude
 *    - Constrained to WPNAV speed limits for safety
 *    - Obstacle avoidance applied to climb rate
 *    - Most common configuration
 * 
 * B. Direct Thrust Mode (use_thrust = true):
 *    - Direct throttle control [0.0 to 1.0] bypasses altitude controller
 *    - Pilot/controller has complete throttle authority
 *    - Enables acrobatic maneuvers (loops, rolls, flips)
 *    - No altitude stabilization - pure attitude flight
 *    - Requires expert piloting or sophisticated controller
 * 
 * Timeout Protection:
 * If no attitude commands for GUID_TIMEOUT seconds (default 3s):
 * - Attitude: Levels roll/pitch, maintains current yaw
 * - Angular velocity: Zeroed (stops rotation)
 * - Climb rate: Zeroed (holds altitude)
 * - Thrust mode: Switches to climb rate mode for safety
 * Prevents flyaway on communication loss.
 * 
 * Takeoff Handling:
 * When landed with positive climb rate or thrust:
 * - Triggers auto-arm sequence
 * - Spools motors to full throttle
 * - Clears land_complete flag
 * - Initializes vertical controller
 * Enables smooth takeoff directly into angle control.
 * 
 * Coordinate Frames:
 * - Attitude quaternion: Earth-frame to body-frame rotation
 * - Angular velocity: Body frame (rad/s) - roll, pitch, yaw rates
 * - Climb rate: Earth frame NED (cm/s, positive = down)
 * - Thrust: Normalized [0.0, 1.0] unitless
 * 
 * Attitude Representation:
 * Uses quaternions (not Euler angles) to avoid gimbal lock:
 * - Quaternion: 4-element unit quaternion [w, x, y, z]
 * - Singularity-free at all orientations
 * - Smooth interpolation for continuous control
 * - Efficient computation for attitude controller
 * 
 * Angular Velocity Feedforward:
 * Provides rate feedforward to attitude controller:
 * - Improves tracking during attitude transients
 * - Reduces attitude lag during aggressive maneuvers
 * - Zero feedforward for static attitude holds
 * - Non-zero for coordinated turns and flips
 * 
 * Safety Considerations:
 * - Disarmed/landed state immediately cuts throttle
 * - Traditional helicopter: maintains motor interlock when on ground
 * - Positive thrust/climb triggers auto-arm for takeoff
 * - No position control - vehicle can drift horizontally
 * - Timeout returns to level attitude (not position hold)
 * 
 * Common Applications:
 * - Manual-like control via companion computer
 * - FPV acrobatic flight control
 * - External attitude stabilization algorithms
 * - Research platforms testing custom attitude controllers
 * - Tele-operation with direct attitude commands
 * - Gimbal-like orientation control (inspection, filming)
 * 
 * MAVLink Interface:
 * Commanded via SET_ATTITUDE_TARGET with:
 * - attitude_quat: Desired attitude quaternion (or zero for rate mode)
 * - body_roll_rate, body_pitch_rate, body_yaw_rate: Angular velocities
 * - thrust: Climb rate (m/s) or normalized thrust [0,1]
 * - type_mask: Indicates thrust interpretation
 * 
 * Comparison to Other Modes:
 * - Position/Velocity modes: Control position/velocity, attitude indirect
 * - Angle mode: Control attitude directly, position uncontrolled
 * - Manual modes: Pilot stick to attitude, similar control feel
 * This mode bridges autonomous and manual flight paradigms.
 * 
 * @note Called at 400Hz when guided_mode == SubMode::Angle
 * @note Only guided sub-mode without position stabilization
 * @note Horizontal drift expected - use position modes for station-keeping
 * 
 * @warning No horizontal position control - vehicle will drift with wind
 * @warning Direct thrust mode disables altitude controller - expert use only
 * @warning Timeout levels attitude but does NOT return to home position
 * @warning Aggressive attitudes may exceed motor/ESC limits
 * @warning Not suitable for autonomous waypoint navigation
 * 
 * Source: ArduCopter/mode_guided.cpp:943-1009
 */
void ModeGuided::angle_control_run()
{
    // Angle sub-mode: direct attitude control with climb rate or thrust
    // Used by SET_ATTITUDE_TARGET MAVLink message for low-level vehicle control
    
    float climb_rate_cms = 0.0f;
    if (!guided_angle_state.use_thrust) {
        // climb rate mode: constrain to configured speed limits
        climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms());

        // integrate obstacle avoidance for vertical velocity
        climb_rate_cms = get_avoidance_adjusted_climbrate_cms(climb_rate_cms);
    }

    // timeout handling - level out and stop climbing if no updates received (default 3 seconds)
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > get_timeout_ms()) {
        // command timeout - level vehicle and hold altitude
        guided_angle_state.attitude_quat.from_euler(Vector3f(0.0, 0.0, attitude_control->get_att_target_euler_rad().z));  // level roll/pitch, maintain yaw
        guided_angle_state.ang_vel_body.zero();  // zero rotation rates
        climb_rate_cms = 0.0f;  // hold altitude
        if (guided_angle_state.use_thrust) {
            // switch from thrust to climb rate control (requires controller initialization)
            pos_control->init_U_controller();
            guided_angle_state.use_thrust = false;
        }
    }

    // positive climb rate or thrust triggers takeoff arming
    const bool positive_thrust_or_climbrate = is_positive(guided_angle_state.use_thrust ? guided_angle_state.thrust : climb_rate_cms);
    if (motors->armed() && positive_thrust_or_climbrate) {
        copter.set_auto_armed(true);  // enable auto-armed state for takeoff
    }

    // safety check: disarm immediately if not armed or auto-armed
    if (!motors->armed() || !copter.ap.auto_armed || (copter.ap.land_complete && !positive_thrust_or_climbrate)) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // TODO: use get_alt_hold_state
    // takeoff sequence: if landed with positive thrust/climb command, spool up motors
    if (copter.ap.land_complete && positive_thrust_or_climbrate) {
        zero_throttle_and_relax_ac();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);  // clear landed flag
            pos_control->init_U_controller();  // initialize altitude controller
        }
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // attitude control: quaternion or rate mode
    if (guided_angle_state.attitude_quat.is_zero()) {
        // zero quaternion = pure rate control mode (body frame angular velocities)
        attitude_control->input_rate_bf_roll_pitch_yaw_rads(guided_angle_state.ang_vel_body.x, guided_angle_state.ang_vel_body.y, guided_angle_state.ang_vel_body.z);
    } else {
        // quaternion attitude control with angular velocity feedforward
        attitude_control->input_quaternion(guided_angle_state.attitude_quat, guided_angle_state.ang_vel_body);
    }

    // vertical control: thrust or climb rate mode
    if (guided_angle_state.use_thrust) {
        // direct thrust control (normalized 0.0-1.0)
        attitude_control->set_throttle_out(guided_angle_state.thrust, true, copter.g.throttle_filt);
    } else {
        // climb rate control (altitude hold with velocity command)
        pos_control->set_pos_target_U_from_climb_rate_cm(climb_rate_cms);
        pos_control->update_U_controller();
    }
}

// helper function to set yaw state and targets
void ModeGuided::set_yaw_state_rad(bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw_rad(yaw_rad, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_and_rate_rad(yaw_rad, yaw_rate_rads);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_and_rate_rad(yaw_rad, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate_rad(yaw_rate_rads);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeGuided::use_pilot_yaw(void) const
{
    return !option_is_enabled(Option::IgnorePilotYaw);
}

// Guided Limit code

// limit_clear - clear/turn off guided limits
void ModeGuided::limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// limit_set - set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void ModeGuided::limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = pos_control->get_pos_estimate_NEU_cm().tofloat();
}

// limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool ModeGuided::limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = pos_control->get_pos_estimate_NEU_cm().tofloat();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        const float horiz_move = get_horizontal_distance(guided_limit.start_pos.xy(), curr_pos.xy());
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}

const Vector3p &ModeGuided::get_target_pos() const
{
    return guided_pos_target_cm;
}

const Vector3f& ModeGuided::get_target_vel() const
{
    return guided_vel_target_cms;
}

const Vector3f& ModeGuided::get_target_accel() const
{
    return guided_accel_target_cmss;
}

float ModeGuided::wp_distance_m() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_distance_to_destination_cm() * 0.01f;
    case SubMode::Pos:
        return get_horizontal_distance(pos_control->get_pos_estimate_NEU_cm().xy().tofloat(), guided_pos_target_cm.xy().tofloat()) * 0.01f;
    case SubMode::PosVelAccel:
        return pos_control->get_pos_error_NE_cm() * 0.01f;
    default:
        return 0.0f;
    }
}

float ModeGuided::wp_bearing_deg() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return degrees(wp_nav->get_wp_bearing_to_destination_rad());
    case SubMode::Pos:
        return degrees(get_bearing_rad(pos_control->get_pos_estimate_NEU_cm().xy().tofloat(), guided_pos_target_cm.xy().tofloat()));
    case SubMode::PosVelAccel:
        return degrees(pos_control->get_bearing_to_target_rad());
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // these do not have bearings
        return 0;
    }
    // compiler guarantees we don't get here
    return 0.0;
}

float ModeGuided::crosstrack_error() const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->crosstrack_error();
    case SubMode::Pos:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        return pos_control->crosstrack_error();
    case SubMode::Angle:
        // no track to have a crosstrack to
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeGuided::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

// pause guide mode
bool ModeGuided::pause()
{
    _paused = true;
    return true;
}

// resume guided mode
bool ModeGuided::resume()
{
    _paused = false;
    return true;
}

#endif
