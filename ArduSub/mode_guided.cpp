/**
 * @file mode_guided.cpp
 * @brief Guided mode implementation for external control via MAVLink
 * 
 * @details Guided mode allows external control of the vehicle through MAVLink commands
 *          sent from a companion computer or ground control station (GCS). This mode
 *          provides several control interfaces:
 * 
 *          - Position Control (Guided_WP): Navigate to specific waypoints
 *          - Velocity Control (Guided_Velocity): Direct velocity commands
 *          - Position+Velocity Control (Guided_PosVel): Combined position and velocity targets
 *          - Attitude Control (Guided_Angle): Direct attitude and climb rate commands
 * 
 *          The mode integrates with MAVLink SET_POSITION_TARGET_LOCAL_NED and
 *          SET_ATTITUDE_TARGET messages, enabling sophisticated external control
 *          strategies for autonomous missions and companion computer integration.
 * 
 *          Safety features include:
 *          - Timeout detection (commands expire after 1-3 seconds without updates)
 *          - Geofence integration (destinations checked against fence boundaries)
 *          - Configurable movement limits (altitude and horizontal distance constraints)
 *          - Pilot override capability (pilot can regain control via RC inputs)
 * 
 * @warning External control systems must maintain command update rates to prevent
 *          timeout-induced failsafes. Position commands timeout after 3 seconds,
 *          attitude commands after 1 second.
 * 
 * @note This implementation is specific to underwater vehicles (ArduSub) and includes
 *       adaptations for 6-DOF control in the underwater environment.
 * 
 * Source: ArduSub/mode_guided.cpp
 */

#include "Sub.h"

/*
 * Init and run calls for guided flight mode
 */

/**
 * @brief Timeout period for position/velocity controller commands
 * @details If no updates received within 3 seconds, velocity commands are zeroed
 *          to prevent runaway behavior. External controllers must maintain update
 *          rate of at least 1Hz, preferably 10Hz or higher.
 */
#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates

/**
 * @brief Timeout period for attitude controller commands
 * @details If no updates received within 1 second, attitude targets are zeroed.
 *          Shorter timeout than position control due to higher safety criticality
 *          of direct attitude commands.
 */
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

/**
 * @brief Position target for combined position+velocity control mode (NEU frame, cm)
 * @details Used in Guided_PosVel mode to track the desired position that is
 *          continuously updated based on velocity commands. Stored in high-precision
 *          position type to prevent drift accumulation.
 */
static Vector3p posvel_pos_target_cm;

/**
 * @brief Velocity target for position+velocity control mode (NEU frame, cm/s)
 * @details Used in Guided_PosVel and Guided_Velocity modes to specify desired
 *          velocity in North-East-Up coordinate frame.
 */
static Vector3f posvel_vel_target_cms;

/**
 * @brief Timestamp of last position/velocity command update (milliseconds)
 * @details Used for timeout detection. Commands are considered stale if current
 *          time exceeds this value plus GUIDED_POSVEL_TIMEOUT_MS.
 */
static uint32_t update_time_ms;

/**
 * @brief State structure for attitude control mode
 * @details Stores the most recent attitude and climb rate commands received
 *          from external controller. Updated via guided_set_angle() API.
 *          All angles stored in centidegrees, climb rate in cm/s.
 */
struct {
    uint32_t update_time_ms;    ///< Timestamp of last attitude command (ms)
    float roll_cd;              ///< Desired roll angle (centidegrees)
    float pitch_cd;             ///< Desired pitch angle (centidegrees)
    float yaw_cd;               ///< Desired yaw angle (centidegrees, wrapped to [-180, 180])
    float climb_rate_cms;       ///< Desired vertical climb rate (cm/s)
} static guided_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f};

/**
 * @brief Structure defining safety limits for guided mode operation
 * @details Enables mission commands or external scripts to constrain guided mode
 *          behavior within safe boundaries. Used primarily when guided mode is
 *          invoked from AUTO mode via NAV_GUIDED_ENABLE mission command.
 * 
 * @note All altitude limits are relative to home position in NEU frame.
 *       Zero values indicate no limit is enforced for that parameter.
 */
struct Guided_Limit {
    uint32_t timeout_ms;  ///< Maximum duration of guided mode operation (ms, 0=no limit)
    float alt_min_cm;     ///< Lower altitude limit in cm above home (0=no limit)
    float alt_max_cm;     ///< Upper altitude limit in cm above home (0=no limit)
    float horiz_max_cm;   ///< Horizontal distance limit from start position (cm, 0=no limit)
    uint32_t start_time;  ///< System time when guided mode was initiated (ms)
    Vector3f start_pos;   ///< Initial position when guided started (NEU frame, cm from home)
} guided_limit;

/**
 * @brief Initialize guided mode controller
 * 
 * @details Called when entering guided mode from another flight mode. Performs
 *          pre-flight checks and initializes the position controller to accept
 *          waypoint commands. The mode starts in position control (Guided_WP)
 *          sub-mode by default, but can be transitioned to other sub-modes
 *          (velocity, posvel, angle) via subsequent MAVLink commands.
 * 
 *          Initialization sequence:
 *          1. Check position estimation validity (GPS, dead reckoning, or visual odometry)
 *          2. Initialize waypoint navigation controller
 *          3. Set initial target to current stopping point
 *          4. Configure yaw control mode based on WP_YAW_BEHAVIOR parameter
 * 
 * @param[in] ignore_checks If true, bypasses position validity check
 *                          (used for forced mode changes)
 * 
 * @return true if initialization successful and mode is ready
 * @return false if position estimate invalid and ignore_checks is false
 * 
 * @note Position validity check ensures vehicle has adequate state estimation
 *       (GPS lock, visual odometry, or dead reckoning) before accepting external
 *       position commands.
 * 
 * @warning Forcing mode entry with ignore_checks=true when position estimation
 *          is unavailable may result in unpredictable vehicle behavior.
 * 
 * @see guided_pos_control_start() for position controller initialization details
 * @see sub.position_ok() for position validity criteria
 */
bool ModeGuided::init(bool ignore_checks)
{
    if (!sub.position_ok() && !ignore_checks) {
        return false;
    }

    // start in position control mode
    guided_pos_control_start();
    return true;
}

/**
 * @brief Get default automatic yaw mode based on WP_YAW_BEHAVIOR parameter
 * 
 * @details Determines how the vehicle should control yaw during waypoint navigation
 *          in guided mode. Behavior is configured via WP_YAW_BEHAVIOR parameter:
 *          - NONE (0): Hold current heading (AUTO_YAW_HOLD)
 *          - LOOK_AT_NEXT_WP (1): Point towards next waypoint
 *          - LOOK_AT_NEXT_WP_EXCEPT_RTL (2): Point at waypoint except during RTL
 *          - LOOK_AHEAD (3): Look in direction of travel
 *          - CORRECT_XTRACK (4): Adjust yaw to minimize cross-track error
 * 
 * @param[in] rtl Set to true if vehicle is in RTL (Return to Launch) mode
 * 
 * @return autopilot_yaw_mode Yaw control mode to use for automatic yaw control
 * 
 * @note The rtl parameter only affects behavior when WP_YAW_BEHAVIOR is set to
 *       LOOK_AT_NEXT_WP_EXCEPT_RTL (value 2)
 * 
 * @see set_auto_yaw_mode() for yaw mode initialization
 */
autopilot_yaw_mode ModeGuided::get_default_auto_yaw_mode(bool rtl) const
{
    switch (g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return AUTO_YAW_HOLD;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return AUTO_YAW_HOLD;
        } else {
            return AUTO_YAW_LOOK_AT_NEXT_WP;
        }
        break;

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return AUTO_YAW_LOOK_AHEAD;
        break;

    case WP_YAW_BEHAVIOR_CORRECT_XTRACK:
        return AUTO_YAW_CORRECT_XTRACK;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return AUTO_YAW_LOOK_AT_NEXT_WP;
        break;
    }
}


/**
 * @brief Initialize position control sub-mode for guided mode
 * 
 * @details Configures guided mode for waypoint navigation, enabling external
 *          controllers to send position targets via MAVLink SET_POSITION_TARGET
 *          commands. This is the default sub-mode when entering guided mode.
 * 
 *          Initialization sequence:
 *          1. Set guided_mode to Guided_WP (position control)
 *          2. Initialize waypoint navigation controller with current state
 *          3. Calculate stopping point based on current velocity and deceleration limits
 *          4. Set initial waypoint destination to stopping point (prevents drift)
 *          5. Configure yaw control based on WP_YAW_BEHAVIOR parameter
 * 
 * @note Initial target is set to calculated stopping point rather than current
 *       position to ensure smooth transition and prevent immediate position
 *       correction jumps when first command is received.
 * 
 * @note Terrain following is not used in underwater applications, so terrain
 *       altitude parameter is set to false in wp_nav calls.
 * 
 * @see guided_set_destination() for sending new position targets
 * @see sub.wp_nav.get_wp_stopping_point_NEU_cm() for stopping point calculation
 */
void ModeGuided::guided_pos_control_start()
{
    // set to position control mode
    sub.guided_mode = Guided_WP;

    // initialise waypoint controller
    sub.wp_nav.wp_and_spline_init_cm();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    sub.wp_nav.get_wp_stopping_point_NEU_cm(stopping_point);

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination_NEU_cm(stopping_point, false);

    // initialise yaw
    sub.yaw_rate_only = false;
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

/**
 * @brief Initialize velocity control sub-mode for guided mode
 * 
 * @details Configures guided mode for direct velocity control, enabling external
 *          controllers to command velocity targets in the NEU (North-East-Up)
 *          coordinate frame via MAVLink SET_POSITION_TARGET commands with velocity
 *          fields populated.
 * 
 *          Initialization sequence:
 *          1. Set guided_mode to Guided_Velocity
 *          2. Configure vertical speed limits (pilot speed up/down parameters)
 *          3. Configure vertical acceleration limits
 *          4. Initialize vertical velocity controller (U-axis in NEU frame)
 *          5. Initialize horizontal velocity controllers (N and E axes)
 *          6. Set yaw to pilot hold mode (pilot retains yaw control)
 * 
 * @note Velocity limits are set based on pilot speed parameters to ensure
 *       external controllers respect the same constraints as manual pilot input.
 * 
 * @note In velocity control mode, pilot yaw input is always respected, allowing
 *       manual yaw override while external controller commands forward/lateral velocity.
 * 
 * @warning Velocity commands timeout after GUIDED_POSVEL_TIMEOUT_MS (3 seconds).
 *          External controllers must maintain update rate of at least 1Hz.
 * 
 * @see guided_set_velocity() for sending velocity commands
 * @see guided_vel_control_run() for velocity controller execution
 */
void ModeGuided::guided_vel_control_start()
{
    // set guided_mode to velocity controller
    sub.guided_mode = Guided_Velocity;

    // initialize vertical maximum speeds and acceleration
    position_control->set_max_speed_accel_U_cm(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_U_cmss(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise velocity controller
    position_control->init_U_controller();
    position_control->init_NE_controller();

    // pilot always controls yaw
    sub.yaw_rate_only = false;
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

/**
 * @brief Initialize combined position+velocity control sub-mode for guided mode
 * 
 * @details Configures guided mode for simultaneous position and velocity target
 *          tracking, enabling sophisticated trajectory control from external
 *          controllers. This mode allows specifying both desired position and
 *          desired velocity, which are integrated to generate smooth trajectories.
 * 
 *          This mode is useful for:
 *          - Trajectory tracking with specified velocity profiles
 *          - Smooth path following with velocity constraints
 *          - Coordination of multiple vehicles with position+velocity synchronization
 * 
 *          Initialization sequence:
 *          1. Set guided_mode to Guided_PosVel
 *          2. Configure vertical speed/acceleration from waypoint navigation defaults
 *          3. Initialize vertical velocity controller
 *          4. Initialize horizontal velocity controllers (N and E)
 *          5. Set yaw to pilot hold mode
 * 
 * @note Speed limits are derived from waypoint navigation parameters rather than
 *       pilot parameters, as this mode is intended for autonomous trajectory following.
 * 
 * @note Position target is continuously updated by integrating velocity commands,
 *       preventing position drift while following velocity profiles.
 * 
 * @see guided_set_destination_posvel() for sending position+velocity commands
 * @see guided_posvel_control_run() for controller execution details
 */
void ModeGuided::guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    sub.guided_mode = Guided_PosVel;

    // set vertical speed and acceleration
    position_control->set_max_speed_accel_U_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_U_cmss());
    position_control->set_correction_speed_accel_U_cmss(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_U_cmss());

    // initialise velocity controller
    position_control->init_U_controller();
    position_control->init_NE_controller();

    // pilot always controls yaw
    sub.yaw_rate_only = false;
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

/**
 * @brief Initialize attitude (angle) control sub-mode for guided mode
 * 
 * @details Configures guided mode for direct attitude control with independent
 *          climb rate command. External controllers send desired roll, pitch, yaw
 *          angles plus vertical climb rate via MAVLink SET_ATTITUDE_TARGET messages.
 * 
 *          This low-level control mode is useful for:
 *          - Manual control remapping from companion computers
 *          - Advanced stabilization algorithms
 *          - Research and development of custom control strategies
 *          - Direct attitude authority for aggressive maneuvers
 * 
 *          Initialization sequence:
 *          1. Set guided_mode to Guided_Angle
 *          2. Configure vertical speed/acceleration limits
 *          3. Initialize vertical velocity controller for climb rate
 *          4. Set initial attitude targets to current vehicle attitude (prevents jumps)
 *          5. Zero initial climb rate command
 *          6. Record initialization timestamp for timeout detection
 * 
 * @note Initial attitude targets are set to current vehicle attitude to ensure
 *       smooth transition when mode is entered. First external command will
 *       update these targets.
 * 
 * @warning This mode provides direct attitude authority and bypasses higher-level
 *          position stabilization. External controller is responsible for preventing
 *          excessive attitudes and maintaining vehicle stability.
 * 
 * @warning Attitude commands timeout after GUIDED_ATTITUDE_TIMEOUT_MS (1 second),
 *          shorter than other modes due to higher risk of direct attitude control.
 * 
 * @see guided_set_angle() for sending attitude commands
 * @see guided_angle_control_run() for attitude controller execution
 */
void ModeGuided::guided_angle_control_start()
{
    // set guided_mode to velocity controller
    sub.guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    position_control->set_max_speed_accel_U_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_U_cmss());
    position_control->set_correction_speed_accel_U_cmss(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_U_cmss());

    // initialise velocity controller
    position_control->init_U_controller();

    // initialise targets
    guided_angle_state.update_time_ms = AP_HAL::millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;

    // pilot always controls yaw
    sub.yaw_rate_only = false;
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

/**
 * @brief Set guided mode waypoint destination (Vector3f version)
 * 
 * @details Commands vehicle to navigate to specified position in NEU (North-East-Up)
 *          coordinate frame relative to home position. Automatically transitions
 *          to Guided_WP sub-mode if not already in position control mode.
 * 
 *          Position is checked against geofence boundaries (if enabled) before
 *          acceptance. Invalid destinations are rejected with NAK response to GCS.
 * 
 *          This is the primary API for MAVLink SET_POSITION_TARGET_LOCAL_NED
 *          commands with position fields populated (no velocity or yaw).
 * 
 * @param[in] destination Target position in NEU frame (cm from home)
 *                        - destination.x: North position (cm)
 *                        - destination.y: East position (cm)
 *                        - destination.z: Up position (cm above home)
 * 
 * @return true if destination accepted and waypoint set successfully
 * @return false if destination outside geofence boundaries (command rejected)
 * 
 * @note Yaw control behavior determined by WP_YAW_BEHAVIOR parameter
 * @note Position target logged to dataflash for post-flight analysis
 * 
 * @warning External controller must verify destination is within vehicle's
 *          operational limits before commanding. Fence check provides safety
 *          boundary but may not catch all hazardous destinations.
 * 
 * @see guided_set_destination(const Vector3f&, bool, float, bool, float, bool)
 *      for version with yaw control
 * @see sub.fence.check_destination_within_fence() for fence checking logic
 */
bool ModeGuided::guided_set_destination(const Vector3f& destination)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // ensure we are in position control mode
    if (sub.guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination_NEU_cm(destination, false);

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, Vector3f());
#endif

    return true;
}

/**
 * @brief Set guided mode waypoint destination (Location version)
 * 
 * @details Commands vehicle to navigate to specified GPS location with optional
 *          altitude specification. Supports multiple altitude reference frames
 *          (absolute, relative to home, above terrain).
 * 
 *          This version is used for MAVLink commands that specify destinations
 *          in GPS coordinates (latitude/longitude) rather than local NEU frame.
 *          Automatically handles coordinate frame conversions.
 * 
 *          Failure modes:
 *          - Destination outside geofence: Rejected with LogErrorCode::DEST_OUTSIDE_FENCE
 *          - Missing terrain data: Rejected with LogErrorCode::FAILED_TO_SET_DESTINATION
 * 
 * @param[in] dest_loc Target location with GPS coordinates and altitude reference
 *                     - dest_loc.lat: Latitude (degrees * 1e7)
 *                     - dest_loc.lng: Longitude (degrees * 1e7)
 *                     - dest_loc.alt: Altitude in specified reference frame (cm)
 *                     - dest_loc.get_alt_frame(): Altitude reference frame
 * 
 * @return true if destination accepted and waypoint set successfully
 * @return false if destination outside fence OR terrain data unavailable
 * 
 * @note Terrain altitude destinations require valid terrain database. Underwater
 *       vehicles typically use relative altitudes, so terrain failure is uncommon.
 * 
 * @note Failure condition is propagated to GCS as MAVLink command NAK
 * 
 * @warning GPS-based destinations may have meter-level accuracy limitations
 *          depending on GPS quality and multipath conditions.
 * 
 * @see sub.wp_nav.set_wp_destination_loc() for coordinate conversion details
 */
bool ModeGuided::guided_set_destination(const Location& dest_loc)
{
#if AP_FENCE_ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // ensure we are in position control mode
    if (sub.guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    if (!sub.wp_nav.set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
#endif

    return true;
}

/**
 * @brief Set guided mode waypoint destination with yaw control
 * 
 * @details Extended version of guided_set_destination that enables simultaneous
 *          position and yaw control. Supports three yaw control modes:
 *          1. Target yaw only (use_yaw=true, use_yaw_rate=false)
 *          2. Target yaw with specified rate (use_yaw=true, use_yaw_rate=true)
 *          3. Target yaw rate only (use_yaw=false, use_yaw_rate=true)
 *          4. Hold current yaw (use_yaw=false, use_yaw_rate=false)
 * 
 *          This is the API for MAVLink SET_POSITION_TARGET_LOCAL_NED commands
 *          that include both position and yaw/yaw_rate fields.
 * 
 * @param[in] destination Target position in NEU frame (cm from home)
 * @param[in] use_yaw If true, yaw_cd parameter specifies target yaw angle
 * @param[in] yaw_cd Target yaw angle in centidegrees (0=North, 9000=East, 18000=South)
 * @param[in] use_yaw_rate If true, yaw_rate_cds parameter specifies yaw rotation rate
 * @param[in] yaw_rate_cds Target yaw rate in centidegrees per second
 * @param[in] relative_yaw If true, yaw_cd is relative to current heading; if false, absolute
 * 
 * @return true if destination accepted and waypoint set successfully
 * @return false if destination outside geofence boundaries
 * 
 * @note Combining position and yaw control enables complex maneuvers like
 *       "move to position while rotating" or "orbit around a point"
 * 
 * @note Relative yaw mode is useful for incremental heading adjustments
 *       without requiring external controller to track current heading
 * 
 * @warning Rapid yaw rate commands combined with translation can stress
 *          attitude controllers and reduce position tracking accuracy
 * 
 * @see guided_set_yaw_state() for yaw mode configuration details
 */
bool ModeGuided::guided_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // ensure we are in position control mode
    if (sub.guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = AP_HAL::millis();

    // no need to check return status because terrain data is not used
    sub.wp_nav.set_wp_destination_NEU_cm(destination, false);

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, Vector3f());
#endif

    return true;
}

/**
 * @brief Set guided mode velocity target (simple version)
 * 
 * @details Commands vehicle to maintain specified velocity in NEU coordinate frame.
 *          Automatically transitions to Guided_Velocity sub-mode if not already
 *          in velocity control mode. Yaw control defaults to pilot hold.
 * 
 *          Velocity commands must be refreshed at least once every 3 seconds
 *          (GUIDED_POSVEL_TIMEOUT_MS) or velocity will be automatically zeroed
 *          as a safety failsafe.
 * 
 *          This is the API for MAVLink SET_POSITION_TARGET_LOCAL_NED commands
 *          with velocity fields populated (position fields ignored or zero).
 * 
 * @param[in] velocity Desired velocity in NEU frame (cm/s)
 *                     - velocity.x: North velocity (cm/s)
 *                     - velocity.y: East velocity (cm/s)
 *                     - velocity.z: Up velocity (cm/s, positive = ascending)
 * 
 * @note Velocity is constrained by configured speed limits from pilot parameters
 * @note Update timestamp is recorded for timeout detection
 * 
 * @warning External controller MUST maintain update rate >= 1Hz to prevent
 *          timeout failsafe from zeroing velocity commands
 * 
 * @warning No position stabilization in velocity mode - vehicle will drift
 *          if constant velocity commands are not updated based on position feedback
 * 
 * @see guided_set_velocity(const Vector3f&, bool, float, bool, float, bool)
 *      for version with yaw control
 * @see guided_vel_control_run() for velocity controller execution and timeout handling
 */
void ModeGuided::guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (sub.guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    update_time_ms = AP_HAL::millis();

    // set position controller velocity target
    position_control->set_vel_desired_NEU_cms(velocity);
}

/**
 * @brief Set guided mode velocity target with yaw control
 * 
 * @details Extended version of guided_set_velocity that enables simultaneous
 *          velocity and yaw control. Combines velocity commands with the same
 *          yaw control options as guided_set_destination.
 * 
 *          Useful for applications requiring coordinated translation and rotation:
 *          - Scanning patterns (translate while rotating)
 *          - Following moving targets with camera pointing
 *          - Coordinated multi-vehicle behaviors
 * 
 * @param[in] velocity Desired velocity in NEU frame (cm/s)
 * @param[in] use_yaw If true, yaw_cd parameter specifies target yaw angle
 * @param[in] yaw_cd Target yaw angle in centidegrees
 * @param[in] use_yaw_rate If true, yaw_rate_cds parameter specifies yaw rotation rate
 * @param[in] yaw_rate_cds Target yaw rate in centidegrees per second
 * @param[in] relative_yaw If true, yaw_cd is relative to current heading
 * 
 * @note Same timeout constraints as simple velocity mode (3 second timeout)
 * @note Yaw state is updated before velocity to ensure consistent control
 * 
 * @warning Coordinating yaw and velocity requires careful planning to avoid
 *          conflicting commands that could stress attitude control
 * 
 * @see guided_set_yaw_state() for yaw control mode details
 */
void ModeGuided::guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
   // check we are in velocity control mode
    if (sub.guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = AP_HAL::millis();

    // set position controller velocity target
    position_control->set_vel_desired_NEU_cms(velocity);

}

/**
 * @brief Set guided mode position target with velocity feedforward (simple version)
 * 
 * @details Combined position+velocity control mode enabling smooth trajectory following.
 *          The position target specifies where the vehicle should be, while the velocity
 *          parameter provides feedforward to the position controller, improving tracking
 *          of moving targets or planned trajectories.
 * 
 *          This mode is ideal for:
 *          - Following pre-computed trajectories with position and velocity waypoints
 *          - Tracking moving targets with known or estimated velocity
 *          - Smooth transitions between waypoints with continuous velocity commands
 * 
 *          Control algorithm:
 *          1. Position controller generates velocity command to reach target position
 *          2. Velocity feedforward is added to position controller output
 *          3. Combined velocity command sent to attitude controller
 * 
 * @param[in] destination Target position in NEU frame relative to origin (cm)
 * @param[in] velocity Velocity feedforward in NEU frame (cm/s)
 * 
 * @return true if destination accepted and waypoint set successfully
 * @return false if destination outside geofence boundaries (command rejected)
 * 
 * @note Transitions mode to Guided_PosVel if not already in that sub-mode
 * @note Subject to 3 second command timeout
 * @note Geofence checks apply only to position target, not velocity
 * @note Yaw control defaults to pilot hold mode
 * 
 * @warning High velocity feedforward values may cause overshoot if position
 *          and velocity are inconsistent with vehicle dynamics
 * 
 * @see guided_set_destination_posvel(const Vector3f&, const Vector3f&, bool, float, bool, float, bool)
 *      for version with yaw control
 * @see AC_PosControl::input_pos_vel_accel_NE_cm() for controller implementation
 */
bool ModeGuided::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity)
{
#if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // check we are in posvel control mode
    if (sub.guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    update_time_ms = AP_HAL::millis();
    posvel_pos_target_cm = destination.topostype();
    posvel_vel_target_cms = velocity;

    position_control->input_pos_vel_accel_NE_cm(posvel_pos_target_cm.xy(), posvel_vel_target_cms.xy(), Vector2f());
    float dz = posvel_pos_target_cm.z;
    position_control->input_pos_vel_accel_U_cm(dz, posvel_vel_target_cms.z, 0);
    posvel_pos_target_cm.z = dz;

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, velocity);
#endif

    return true;
}

/**
 * @brief Set guided mode position target with velocity feedforward and yaw control
 * 
 * @details Extended version of guided_set_destination_posvel that adds comprehensive
 *          yaw control options. Enables coordinated position, velocity, and heading
 *          control for sophisticated trajectory following.
 * 
 *          Useful for applications requiring full 6-DOF control:
 *          - Camera pointing while following trajectory
 *          - Orientation-aware inspection paths
 *          - Coordinated multi-vehicle formations
 *          - Scanning patterns with controlled vehicle orientation
 * 
 * @param[in] destination Target position in NEU frame relative to origin (cm)
 * @param[in] velocity Velocity feedforward in NEU frame (cm/s)
 * @param[in] use_yaw If true, yaw_cd parameter specifies target yaw angle
 * @param[in] yaw_cd Target yaw angle in centidegrees
 * @param[in] use_yaw_rate If true, yaw_rate_cds parameter specifies yaw rotation rate
 * @param[in] yaw_rate_cds Target yaw rate in centidegrees per second
 * @param[in] relative_yaw If true, yaw_cd is relative to current heading
 * 
 * @return true if destination accepted and controller configured
 * @return false if destination outside geofence boundaries
 * 
 * @note Yaw state is set before position/velocity to ensure consistent control
 * @note All position, velocity, and yaw commands subject to same timeout (3 seconds)
 * 
 * @warning Coordinating position, velocity, and yaw requires careful trajectory
 *          planning to avoid control conflicts and attitude controller saturation
 * 
 * @see guided_set_yaw_state() for yaw control mode details
 */
bool ModeGuided::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    #if AP_FENCE_ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!sub.fence.check_destination_within_fence(dest_loc)) {
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
    #endif

    // check we are in posvel control mode
    if (sub.guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    // set yaw state
    guided_set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = AP_HAL::millis();

    posvel_pos_target_cm = destination.topostype();
    posvel_vel_target_cms = velocity;

    position_control->input_pos_vel_accel_NE_cm(posvel_pos_target_cm.xy(), posvel_vel_target_cms.xy(), Vector2f());
    float dz = posvel_pos_target_cm.z;
    position_control->input_pos_vel_accel_U_cm(dz, posvel_vel_target_cms.z, 0);
    posvel_pos_target_cm.z = dz;

#if HAL_LOGGING_ENABLED
    // log target
    sub.Log_Write_GuidedTarget(sub.guided_mode, destination, velocity);
#endif

    return true;
}

/**
 * @brief Set guided mode attitude target with climb rate
 * 
 * @details Direct attitude control interface for external controllers using quaternion
 *          representation. Enables low-level attitude authority with independent vertical
 *          velocity control. This is the lowest-level control interface in guided mode.
 * 
 *          Typical applications:
 *          - Research platforms requiring direct attitude control
 *          - Advanced stabilization algorithms from companion computers
 *          - Manual control remapping through external processors
 *          - Aggressive maneuvers requiring direct attitude authority
 * 
 *          Control flow:
 *          1. Quaternion converted to Euler angles (roll, pitch, yaw)
 *          2. Angles converted to centidegrees for internal consistency
 *          3. Yaw wrapped to [-180, 180] degree range
 *          4. Climb rate stored directly in cm/s
 *          5. Update timestamp recorded for 1-second timeout detection
 * 
 * @param[in] q Target attitude as quaternion (vehicle body frame relative to NED)
 *              - Quaternion must be normalized (magnitude = 1.0)
 *              - Convention: Hamilton quaternion (w, x, y, z)
 * @param[in] climb_rate_cms Desired vertical velocity in cm/s (positive = ascending)
 * 
 * @note Automatically transitions to Guided_Angle sub-mode if needed
 * @note Attitude commands timeout after 1 second (GUIDED_ATTITUDE_TIMEOUT_MS)
 * @note Shorter timeout than position commands due to safety criticality
 * 
 * @warning External controller MUST maintain update rate >= 2Hz to prevent timeout
 * @warning Unnormalized quaternions will produce incorrect attitude commands
 * @warning Direct attitude control bypasses position stabilization - vehicle will
 *          drift if attitude is not coordinated with position feedback
 * 
 * @see guided_angle_control_run() for attitude controller execution
 * @see MAVLink SET_ATTITUDE_TARGET message for command protocol
 */
void ModeGuided::guided_set_angle(const Quaternion &q, float climb_rate_cms)
{
    // check we are in angle control mode
    if (sub.guided_mode != Guided_Angle) {
        guided_angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = degrees(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = degrees(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(degrees(guided_angle_state.yaw_cd) * 100.0f);

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = AP_HAL::millis();
}

/**
 * @brief Configure yaw control mode and targets
 * 
 * @details Internal helper function that configures yaw control behavior based on
 *          command parameters. Supports four distinct yaw control modes:
 * 
 *          1. Target yaw only (use_yaw=true, use_yaw_rate=false):
 *             Vehicle rotates to target heading, controller determines rate
 * 
 *          2. Target yaw with rate limit (use_yaw=true, use_yaw_rate=true):
 *             Vehicle rotates to target heading at specified maximum rate
 * 
 *          3. Yaw rate only (use_yaw=false, use_yaw_rate=true):
 *             Vehicle rotates continuously at specified rate (no target heading)
 * 
 *          4. Hold current yaw (use_yaw=false, use_yaw_rate=false):
 *             Vehicle maintains current heading
 * 
 *          The function automatically determines shortest rotation direction by
 *          comparing target heading to current heading (wrapping handled correctly).
 * 
 * @param[in] use_yaw If true, yaw_cd specifies target heading
 * @param[in] yaw_cd Target yaw angle in centidegrees
 * @param[in] use_yaw_rate If true, yaw_rate_cds specifies rotation rate
 * @param[in] yaw_rate_cds Target yaw rate in centidegrees per second
 * @param[in] relative_angle If true, yaw_cd is relative to current heading
 *                           If false, yaw_cd is absolute in NEU frame
 * 
 * @note Called internally by position and velocity setpoint functions
 * @note Yaw commands integrated into overall mode timeout (not separate)
 * 
 * @see set_auto_yaw_look_at_heading() for target heading control
 * @see set_yaw_rate() for rate-only control
 */
void ModeGuided::guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{    
    float current_yaw = wrap_2PI(AP::ahrs().get_yaw_rad());
    float euler_yaw_angle;
    float yaw_error;

    euler_yaw_angle = wrap_2PI((yaw_cd * 0.01f));
    yaw_error = wrap_PI(euler_yaw_angle - current_yaw);

    int direction = 0;
    if (yaw_error < 0){
        direction = -1;
    } else {
        direction = 1;
    }

    /*
    case 1: target yaw only
    case 2: target yaw and yaw rate
    case 3: target yaw rate only
    case 4: hold current yaw
    */
    if (use_yaw && !use_yaw_rate) {
        sub.yaw_rate_only = false;
        sub.mode_auto.set_auto_yaw_look_at_heading(yaw_cd * 0.01f, 0.0f, direction, relative_angle);
    } else if (use_yaw && use_yaw_rate) { 
        sub.yaw_rate_only = false;
        sub.mode_auto.set_auto_yaw_look_at_heading(yaw_cd * 0.01f, yaw_rate_cds * 0.01f, direction, relative_angle);
    } else if (!use_yaw && use_yaw_rate) {
        sub.yaw_rate_only = true;
        sub.mode_auto.set_yaw_rate(yaw_rate_cds * 0.01f);
    } else{
        sub.yaw_rate_only = false;
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

/**
 * @brief Main guided mode controller execution function
 * 
 * @details Called every control loop iteration (typically 50-400Hz depending on vehicle
 *          configuration) to execute the active guided sub-mode controller. Acts as a
 *          dispatcher that routes execution to the appropriate sub-mode implementation
 *          based on current guided_mode state.
 * 
 *          Execution flow:
 *          1. Check for pilot override (RC input requesting mode change)
 *          2. Check guided mode operational limits (altitude, horizontal distance, timeout)
 *          3. Dispatch to active sub-mode controller:
 *             - Guided_WP: Position waypoint following
 *             - Guided_Velocity: Pure velocity control
 *             - Guided_PosVel: Combined position+velocity control
 *             - Guided_Angle: Direct attitude control
 * 
 *          Each sub-mode controller:
 *          - Checks for command timeout and handles failsafe
 *          - Processes current targets and generates control outputs
 *          - Updates attitude controller with desired angles/rates
 *          - Calls motors library to convert attitude to motor outputs
 * 
 * @note Must be called at scheduler's main loop rate for proper control
 * @note Recommended minimum rate: 100Hz for stable control
 * @note Pilot override allows immediate exit to manual control via RC
 * 
 * @warning If this function is not called regularly (e.g., scheduler overload),
 *          vehicle control will degrade and safety may be compromised
 * 
 * @see guided_pos_control_run() for waypoint navigation implementation
 * @see guided_vel_control_run() for velocity control implementation
 * @see guided_posvel_control_run() for position+velocity implementation
 * @see guided_angle_control_run() for attitude control implementation
 * @see guided_limit_check() for limit enforcement details
 */
void ModeGuided::run()
{
    // call the correct auto controller
    switch (sub.guided_mode) {

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        guided_posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        guided_angle_control_run();
        break;
    }
}

/**
 * @brief Execute position waypoint control for guided mode
 * 
 * @details Implements waypoint navigation control for Guided_WP sub-mode. Commands
 *          vehicle to navigate to a specific position target using the waypoint
 *          navigation controller (wp_nav), which handles trajectory generation,
 *          velocity profiling, and position stabilization.
 * 
 *          Control loop execution sequence:
 *          1. Safety check: If disarmed, set motors to ground idle and exit
 *          2. Process pilot yaw input (allows pilot yaw override during guided navigation)
 *          3. Enable full motor throttle range
 *          4. Update waypoint navigation controller
 *          5. Convert waypoint controller output (NE velocities) to body frame roll/pitch
 *          6. Get altitude hold output from vertical position controller
 *          7. Update attitude controller with desired angles
 *          8. Convert attitude to motor outputs
 * 
 *          Pilot interaction:
 *          - Pilot can override yaw at any time using yaw stick
 *          - Yaw override switches to AUTO_YAW_HOLD mode
 *          - When pilot releases yaw stick, returns to commanded yaw mode
 * 
 * @note Called at main loop rate from run() when in Guided_WP mode
 * @note No command timeout in position mode - waypoint remains active until changed
 * @note Disarmed state disables all stabilization for underwater vehicle safety
 * 
 * @see AC_WPNav::update_wpnav() for trajectory following algorithm
 * @see translate_wpnav_rp() for coordinate frame transformations
 */
void ModeGuided::guided_pos_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init_cm();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_U_controller();

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

/**
 * @brief Execute velocity control for guided mode
 * 
 * @details Implements pure velocity control for Guided_Velocity sub-mode. Commands
 *          vehicle to maintain specified velocities in NEU frame without position
 *          stabilization. External controller must continuously update velocity
 *          commands to maintain desired trajectory.
 * 
 *          Control loop execution sequence:
 *          1. Safety check: If disarmed, set motors to ground idle and exit
 *          2. Process pilot yaw input (allows pilot yaw override)
 *          3. Enable full motor throttle range
 *          4. Timeout check: Zero velocities if no updates for 3 seconds
 *          5. Update position controller with velocity targets
 *          6. Convert velocity controller output to body frame roll/pitch angles
 *          7. Update attitude controller with desired angles
 *          8. Convert attitude to motor outputs
 * 
 *          Timeout failsafe behavior:
 *          - If no velocity command received within GUIDED_POSVEL_TIMEOUT_MS (3s)
 *          - All velocity targets set to zero
 *          - Vehicle will attempt to hold current position using position controller
 *          - Prevents runaway if external controller fails or connection lost
 * 
 * @note Called at main loop rate from run() when in Guided_Velocity mode
 * @note No position feedback in this mode - vehicle will drift without regular updates
 * @note External controller responsible for position stabilization via velocity loop
 * 
 * @warning Velocity commands timeout after 3 seconds for safety
 * @warning Vehicle will drift if velocity commands stop - no position hold
 * 
 * @see GUIDED_POSVEL_TIMEOUT_MS for timeout duration
 * @see position_control->update_NE_controller() for velocity tracking implementation
 */
void ModeGuided::guided_vel_control_run()
{
    // ifmotors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_U_controller();
        position_control->init_NE_controller();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !position_control->get_vel_desired_NEU_cms().is_zero()) {
        position_control->set_vel_desired_NEU_cms(Vector3f(0,0,0));
    }

    position_control->stop_pos_NE_stabilisation();
    // call velocity controller which includes z axis controller
    position_control->update_NE_controller();

    position_control->set_pos_target_U_from_climb_rate_cm(position_control->get_vel_desired_NEU_cms().z);
    position_control->update_U_controller();

    float lateral_out, forward_out;
    sub.translate_pos_control_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

/**
 * @brief Execute position+velocity control for guided mode
 * 
 * @details Implements combined position and velocity control for Guided_PosVel sub-mode.
 *          Provides smooth trajectory tracking by combining position target with velocity
 *          feedforward. The position controller generates velocity commands to reach the
 *          target position, while the velocity feedforward is added to improve tracking
 *          performance and reduce position lag.
 * 
 *          Control loop execution sequence:
 *          1. Safety check: If disarmed, set motors to ground idle and exit
 *          2. Process pilot yaw input (allows pilot yaw override)
 *          3. Enable full motor throttle range
 *          4. Timeout check: Zero velocities if no updates for 3 seconds
 *          5. Update position target by integrating velocity (prevents drift)
 *          6. Send combined position+velocity to position controller
 *          7. Update velocity controllers (NE horizontal, U vertical)
 *          8. Convert position controller output to body frame roll/pitch
 *          9. Update attitude controller with desired angles
 *          10. Convert attitude to motor outputs
 * 
 *          Key difference from pure velocity mode:
 *          - Position target continuously updated by integrating velocity
 *          - Provides position stabilization even if velocity commands stop
 *          - Better tracking of planned trajectories with known position and velocity
 * 
 * @note Called at main loop rate from run() when in Guided_PosVel mode
 * @note Position updated each cycle to prevent accumulation errors
 * @note Timeout failsafe zeros velocity but maintains last position target
 * 
 * @warning External controller must maintain >1Hz update rate to prevent timeout
 * 
 * @see GUIDED_POSVEL_TIMEOUT_MS for timeout duration
 * @see AC_PosControl::input_pos_vel_accel_NE_cm() for combined controller
 */
void ModeGuided::guided_posvel_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_U_controller();
        position_control->init_NE_controller();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        } else{
            if (sub.yaw_rate_only){
                set_auto_yaw_mode(AUTO_YAW_RATE);
            } else{
                set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
            }
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !posvel_vel_target_cms.is_zero()) {
        posvel_vel_target_cms.zero();
    }

    // advance position target using velocity target
    posvel_pos_target_cm += (posvel_vel_target_cms * position_control->get_dt_s()).topostype();

    // send position and velocity targets to position controller
    position_control->input_pos_vel_accel_NE_cm(posvel_pos_target_cm.xy(), posvel_vel_target_cms.xy(), Vector2f());
    float pz = posvel_pos_target_cm.z;
    position_control->input_pos_vel_accel_U_cm(pz, posvel_vel_target_cms.z, 0);
    posvel_pos_target_cm.z = pz;

    // run position controller
    position_control->update_NE_controller();
    position_control->update_U_controller();

    float lateral_out, forward_out;
    sub.translate_pos_control_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        // roll, pitch from pilot, yaw & yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_slew_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), target_yaw_rate);
    } else if (sub.auto_yaw_mode == AUTO_YAW_RATE) {
        // roll, pitch from pilot, and yaw_rate from auto_control
        target_yaw_rate = sub.yaw_look_at_heading_slew * 100.0;
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

/**
 * @brief Execute guided angle control mode (direct attitude and climb rate control)
 * 
 * @details Called from run() when sub.guided_mode == Guided_Angle. This function
 *          provides direct attitude control with independent climb rate control,
 *          enabling external systems to command specific roll, pitch, yaw angles
 *          and vertical velocity.
 * 
 *          Control sequence:
 *          1. Check motor armed state (disarm → zero throttle and exit)
 *          2. Constrain roll/pitch to configured angle limits (angle_max)
 *          3. Wrap yaw to ±180 degrees
 *          4. Constrain climb rate to vehicle speed limits
 *          5. Check for timeout (1 second) → zero angles and climb rate if expired
 *          6. Enable full motor range (THROTTLE_UNLIMITED)
 *          7. Command attitude controller with roll, pitch, yaw angles
 *          8. Command position controller U-axis with climb rate
 * 
 *          Safety features:
 *          - Angle limiting: Roll and pitch constrained to MIN(althold_lean_angle_max, angle_max)
 *          - Proportional limiting: If combined roll+pitch exceeds limit, both scaled proportionally
 *          - Climb rate limiting: Constrained to configured up/down speed limits
 *          - Timeout protection: 1 second timeout (GUIDED_ATTITUDE_TIMEOUT_MS) zeros all inputs
 * 
 * @note Angle commands update from guided_angle_state structure populated by
 *       guided_set_angle() from MAVLink SET_ATTITUDE_TARGET messages
 * 
 * @note Timeout is strict - external controller must update at >1Hz to maintain control
 * 
 * @warning Aggressive angle commands can stress attitude control and cause oscillations.
 *          External controllers should respect vehicle dynamic limits.
 * 
 * @warning Disarmed vehicles do NOT stabilize attitude - attitude_control is relaxed
 *          to prevent motor activation
 * 
 * @see guided_set_angle() for how angle targets are set from MAVLink
 * @see GUIDED_ATTITUDE_TIMEOUT_MS for timeout duration
 */
void ModeGuided::guided_angle_control_run()
{
    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0.0f,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        // initialise velocity controller
        position_control->init_U_controller();
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max_cd(), sub.aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms());

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = AP_HAL::millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(roll_in, pitch_in, yaw_in, true);

    // call position controller
    position_control->set_pos_target_U_from_climb_rate_cm(climb_rate_cms);
    position_control->update_U_controller();
}

// Guided Limit code

/**
 * @brief Clear/disable all guided mode movement limits
 * 
 * @details Resets all limit parameters in the guided_limit structure to zero,
 *          effectively disabling timeout, altitude, and horizontal distance
 *          constraints. Called when entering unrestricted guided mode or when
 *          exiting mission-invoked guided control.
 * 
 *          Cleared limits:
 *          - timeout_ms: Maximum time allowed in guided mode
 *          - alt_min_cm: Minimum altitude limit (cm above home)
 *          - alt_max_cm: Maximum altitude limit (cm above home)
 *          - horiz_max_cm: Maximum horizontal distance from guided start position
 * 
 * @note Setting limits to zero disables them - limit checking in guided_limit_check()
 *       treats zero values as "no limit"
 * 
 * @note Does NOT reset start_time or start_pos - these are set by guided_limit_init_time_and_pos()
 * 
 * @see guided_limit_set() to enable limits
 * @see guided_limit_check() for limit enforcement
 */
void ModeGuided::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}


/**
 * @brief Set the yaw control mode for autonomous operation
 * 
 * @details Configures how the vehicle's yaw (heading) is controlled during guided
 *          mode operation. Each yaw mode has different initialization requirements
 *          and control behaviors. This function handles mode transitions and performs
 *          necessary initialization for the selected mode.
 * 
 *          Available yaw modes:
 *          - AUTO_YAW_HOLD: Pilot controls heading via RC yaw input
 *          - AUTO_YAW_LOOK_AT_NEXT_WP: Point toward waypoint (initialized by wpnav)
 *          - AUTO_YAW_ROI: Point toward region of interest (yaw_look_at_WP)
 *          - AUTO_YAW_LOOK_AT_HEADING: Maintain fixed heading (yaw_look_at_heading)
 *          - AUTO_YAW_LOOK_AHEAD: Point in direction of travel
 *          - AUTO_YAW_RESETTOARMEDYAW: Return to heading at arming time
 *          - AUTO_YAW_RATE: Rotate at specified rate (yaw_look_at_heading_slew)
 *          - AUTO_YAW_CORRECT_XTRACK: Correct for cross-track error
 * 
 * @param[in] yaw_mode Desired autopilot yaw mode from autopilot_yaw_mode enum
 * 
 * @note Function returns immediately if requested mode matches current mode
 *       to avoid unnecessary reinitialization
 * 
 * @note Some modes require external setup:
 *       - AUTO_YAW_LOOK_AT_HEADING: Caller must set yaw_look_at_heading
 *       - AUTO_YAW_ROI: yaw_look_at_WP should be configured
 * 
 * @see get_auto_heading() for heading calculation based on active mode
 */
void ModeGuided::set_auto_yaw_mode(autopilot_yaw_mode yaw_mode)
{
    // return immediately if no change
    if (sub.auto_yaw_mode == yaw_mode) {
        return;
    }
    sub.auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (sub.auto_yaw_mode) {
    
    case AUTO_YAW_HOLD:
        // pilot controls the heading
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        sub.yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading
        // caller should set the yaw_look_at_heading
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        sub.yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;
    
    case AUTO_YAW_RATE:
        // set target yaw rate to yaw_look_at_heading_slew
        break;
    }
}

/**
 * @brief Get target heading based on current auto yaw mode
 * 
 * @details Calculates and returns the desired vehicle heading (yaw angle) based on
 *          the currently active yaw control mode. Called at 100Hz from attitude
 *          control functions to determine yaw setpoint.
 * 
 *          Heading calculation by mode:
 *          - AUTO_YAW_ROI: Bearing to region of interest waypoint
 *          - AUTO_YAW_LOOK_AT_HEADING: Fixed heading (yaw_look_at_heading)
 *          - AUTO_YAW_LOOK_AHEAD: Heading in direction of velocity
 *          - AUTO_YAW_RESETTOARMEDYAW: Heading at time of arming
 *          - AUTO_YAW_CORRECT_XTRACK: Track bearing with cross-track correction
 *          - AUTO_YAW_LOOK_AT_NEXT_WP (default): Heading from wp_nav controller
 * 
 * @return Target heading in centidegrees (0-36000, where 36000 = 360 degrees)
 * 
 * @note Called at 100Hz update rate - calculations must be efficient
 * 
 * @note AUTO_YAW_CORRECT_XTRACK implements sophisticated cross-track error
 *       correction by calculating angle between actual velocity and desired
 *       track, limiting correction to xtrack_angle_limit parameter
 * 
 * @note For AUTO_YAW_CORRECT_XTRACK: velocity must exceed 10% of max speed
 *       for correction to be applied (avoids erratic behavior at low speeds)
 * 
 * @see set_auto_yaw_mode() to change active yaw mode
 */
float ModeGuided::get_auto_heading()
{
    switch (sub.auto_yaw_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi_WP
        return sub.get_roi_yaw();
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        return sub.yaw_look_at_heading;
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return sub.get_look_ahead_yaw();
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return sub.initial_armed_bearing;
        break;

    case AUTO_YAW_CORRECT_XTRACK: {
        // TODO return current yaw if not in appropriate mode
        // Bearing of current track (centidegrees)
        float track_bearing = get_bearing_cd(sub.wp_nav.get_wp_origin_NEU_cm().xy(), sub.wp_nav.get_wp_destination_NEU_cm().xy());

        // Bearing from current position towards intermediate position target (centidegrees)
        const Vector2f target_vel_xy = position_control->get_vel_target_NEU_cms().xy();
        float angle_error = 0.0f;
        if (target_vel_xy.length() >= position_control->get_max_speed_NE_cms() * 0.1f) {
            const float desired_angle_cd = degrees(target_vel_xy.angle()) * 100.0f;
            angle_error = wrap_180_cd(desired_angle_cd - track_bearing);
        }
        float angle_limited = constrain_float(angle_error, -g.xtrack_angle_limit * 100.0f, g.xtrack_angle_limit * 100.0f);
        return wrap_360_cd(track_bearing + angle_limited);
    }
    break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the vehicle to turn too much during flight
        return sub.wp_nav.get_yaw();
        break;
    }
}
/**
 * @brief Configure guided mode timeout and movement limits
 * 
 * @details Establishes safety boundaries for guided mode operation, typically used
 *          when guided mode is invoked from AUTO mode missions via NAV_GUIDED_ENABLE
 *          command. Limits prevent the vehicle from straying too far or operating
 *          too long under external control during autonomous missions.
 * 
 *          Setting a parameter to zero disables that specific limit:
 *          - timeout_ms = 0: No time limit
 *          - alt_min_cm = 0: No minimum altitude limit
 *          - alt_max_cm = 0: No maximum altitude limit
 *          - horiz_max_cm = 0: No horizontal distance limit
 * 
 * @param[in] timeout_ms Maximum duration for guided control in milliseconds (0 = no limit)
 * @param[in] alt_min_cm Minimum altitude above home in centimeters (0 = no limit)
 * @param[in] alt_max_cm Maximum altitude above home in centimeters (0 = no limit)
 * @param[in] horiz_max_cm Maximum horizontal distance from start position in centimeters (0 = no limit)
 * 
 * @note Altitude limits are relative to home position, not current position
 * @note Horizontal limit is distance from position when guided_limit_init_time_and_pos() was called
 * @note Must call guided_limit_init_time_and_pos() after setting limits to establish reference point
 * 
 * @warning Limits are only checked if guided mode is invoked from AUTO mode missions.
 *          Direct guided mode entry (RC switch, GCS mode change) does NOT enforce these limits.
 * 
 * @see guided_limit_init_time_and_pos() to initialize reference time and position
 * @see guided_limit_check() for limit enforcement
 * @see guided_limit_clear() to disable all limits
 */
void ModeGuided::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

/**
 * @brief Initialize reference time and position for guided limit enforcement
 * 
 * @details Records the current time and vehicle position as the reference point for
 *          timeout and horizontal distance limit checking. Must be called after
 *          guided_limit_set() and before entering guided control to establish the
 *          baseline for limit calculations.
 * 
 *          Sets:
 *          - guided_limit.start_time: Current system time in milliseconds
 *          - guided_limit.start_pos: Current vehicle position in NEU frame (cm)
 * 
 *          This function is called from AUTO mode's auto_nav_guided_start() when
 *          a NAV_GUIDED_ENABLE mission command transfers control to an external
 *          system with configured safety limits.
 * 
 * @note Only called from AUTO mode's auto_nav_guided_start function
 * @note Position is recorded in NED (North-East-Down) frame relative to home
 * @note Timeout starts counting from this moment, not from when limits were set
 * 
 * @see guided_limit_set() to configure limits before initialization
 * @see guided_limit_check() which uses start_time and start_pos for limit checking
 */
void ModeGuided::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position_neu_cm();
}

/**
 * @brief Check if vehicle has breached any configured guided mode limits
 * 
 * @details Evaluates current vehicle state against configured limits and returns
 *          true if any limit has been exceeded. Used when guided mode is invoked
 *          from AUTO mode via NAV_GUIDED_ENABLE mission command to determine if
 *          mission should resume (limit breach triggers return to AUTO mode).
 * 
 *          Checked limits (in order):
 *          1. Timeout: Time elapsed since guided_limit.start_time exceeds timeout_ms
 *          2. Minimum altitude: Current altitude below alt_min_cm (if alt_min_cm != 0)
 *          3. Maximum altitude: Current altitude above alt_max_cm (if alt_max_cm != 0)
 *          4. Horizontal distance: Distance from start_pos exceeds horiz_max_cm (if > 0)
 * 
 * @return true if any limit has been breached, false if within all limits
 * 
 * @note Limits with value of zero are treated as disabled/unlimited
 * @note Altitude limits are in NED frame (up is negative, down is positive)
 * @note Horizontal limit uses 2D Euclidean distance in NE plane
 * @note Only used when guided is invoked from NAV_GUIDED_ENABLE mission command,
 *       not for direct guided mode entry
 * 
 * @note Function returns true (limit breached) as soon as first violation detected
 *       for efficiency - does not check remaining limits
 * 
 * @warning Breaching limits during mission-guided operation will cause AUTO mode
 *          to resume, potentially with abrupt mode transition
 * 
 * @see guided_limit_set() to configure limits
 * @see guided_limit_init_time_and_pos() to establish reference point
 */
bool ModeGuided::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (AP_HAL::millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position_neu_cm();

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
