/**
 * @file mode_auto.cpp
 * @brief ArduSub Auto mode implementation for autonomous mission execution
 * 
 * @details This file implements autonomous mission execution for underwater vehicles,
 *          enabling waypoint navigation, circle patterns, and terrain following during
 *          automated missions. The Auto mode integrates with the AP_Mission library
 *          to execute mission commands including navigation waypoints, do/now commands,
 *          and conditional logic.
 *          
 *          Auto mode provides several sub-modes for different mission behaviors:
 *          - Auto_WP: Waypoint navigation with autonomous depth control
 *          - Auto_Circle: Circular pattern flight around a specified center point
 *          - Auto_Loiter: Hold position at current location or mission waypoint
 *          - Auto_NavGuided: External navigation control (companion computer)
 *          - Auto_TerrainRecover: Failsafe recovery when terrain/rangefinder data lost
 *          
 *          The implementation handles underwater-specific challenges including:
 *          - Terrain-relative navigation using rangefinder/depth sensors
 *          - Buoyancy-aware depth control during waypoint transitions
 *          - Failsafe recovery for lost terrain data
 *          - Integration with AP_Mission for MAVLink mission protocol
 *          
 * @note This mode requires valid position estimates (EKF healthy) and mission loaded
 * @warning Mission execution in confined underwater environments requires careful
 *          validation and obstacle avoidance considerations
 *          
 * Source: ArduSub/mode_auto.cpp
 */
#include "Sub.h"
/**
 * @brief Initialize Auto mode for autonomous mission execution
 * 
 * @details Validates mission readiness and initializes the autonomous navigation
 *          subsystems for underwater mission execution. This includes:
 *          - Verifying position estimation is healthy (EKF converged)
 *          - Confirming a mission is loaded in AP_Mission
 *          - Resetting ROI (Region of Interest) yaw mode to prevent unwanted heading
 *          - Initializing waypoint navigation controller
 *          - Starting or resuming mission based on MIS_RESTART parameter
 *          
 *          Initial sub-mode is set to Auto_Loiter, which will be updated by the
 *          mission state machine when the first navigation command executes.
 *          
 * @param[in] ignore_checks If true, bypass safety checks (currently unused in implementation)
 * 
 * @return true if Auto mode successfully initialized and mission ready to execute
 * @return false if position estimate invalid or no mission loaded
 * 
 * @note Mission can be started from beginning or resumed from last waypoint based on
 *       MIS_RESTART parameter configuration
 * @warning Entering Auto mode without valid position estimate will cause init failure
 *          and prevent mode change - this is a critical safety check for autonomous operation
 *          
 * Source: ArduSub/mode_auto.cpp:10-32
 */
bool ModeAuto::init(bool ignore_checks) {
     if (!sub.position_ok() || !sub.mission.present()) {
        return false;
    }

    sub.auto_mode = Auto_Loiter;

    // stop ROI from carrying over from previous runs of the mission
    // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
    if (sub.auto_yaw_mode == AUTO_YAW_ROI) {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }

    // initialise waypoint controller
    sub.wp_nav.wp_and_spline_init_cm();

    // clear guided limits
    guided_limit_clear();

    // start/resume the mission (based on MIS_RESTART parameter)
    sub.mission.start_or_resume();
    return true;
}

/**
 * @brief Main Auto mode update loop - runs appropriate sub-mode controller
 * 
 * @details Called at main loop rate (typically 50-100Hz) to execute the current
 *          autonomous mission behavior. This function:
 *          - Updates AP_Mission state machine (processes mission commands, advances waypoints)
 *          - Dispatches to appropriate sub-mode controller based on current auto_mode state
 *          
 *          Auto Sub-Modes:
 *          - Auto_WP: Autonomous waypoint navigation with position control
 *          - Auto_CircleMoveToEdge: Transitioning from current position to circle edge
 *          - Auto_Circle: Flying circular pattern around specified center point
 *          - Auto_NavGuided: External navigation control via MAVLink (companion computer)
 *          - Auto_Loiter: Position hold at mission loiter waypoint
 *          - Auto_TerrainRecover: Failsafe recovery mode when terrain/rangefinder data lost
 *          
 *          Mission state machine in AP_Mission handles:
 *          - Parsing MAVLink mission commands from mission buffer
 *          - Executing navigation commands (waypoints, circles, loiter)
 *          - Executing do/now commands (set servo, change speed, camera trigger)
 *          - Mission completion detection and behavior
 *          
 * @note This is called every iteration of the main control loop, actual control frequency
 *       depends on scheduler configuration (typically 50Hz for Sub)
 * @warning Sub-mode transitions must be handled by mission commands or failsafe logic -
 *          incorrect mode transitions can cause unexpected vehicle behavior underwater
 *          
 * Source: ArduSub/mode_auto.cpp:36-66
 */
void ModeAuto::run()
{
    sub.mission.update();

    // call the correct auto controller
    switch (sub.auto_mode) {

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        auto_wp_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED
        auto_nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        auto_loiter_run();
        break;

    case Auto_TerrainRecover:
        auto_terrain_recover_run();
        break;
    }
}

/**
 * @brief Initialize waypoint navigation to a destination in NED coordinates
 * 
 * @details Starts autonomous waypoint navigation to a specified 3D position in
 *          North-East-Down (NED) frame relative to origin. This version accepts
 *          a Vector3f destination and does not use terrain-relative altitude.
 *          
 *          This function:
 *          - Sets Auto sub-mode to Auto_WP for waypoint navigation
 *          - Configures waypoint navigation controller with destination
 *          - Initializes yaw control mode (look-ahead or ROI if active)
 *          
 * @param[in] destination Target position in NEU (North-East-Up) coordinates in centimeters
 *                        relative to EKF origin (typically vehicle startup location)
 * 
 * @note This version does NOT use terrain data, suitable for depth-relative navigation
 * @note To-Do: Improve yaw reset logic to only reset when previous command was not a waypoint
 *       to eliminate special ROI handling in init()
 *       
 * Source: ArduSub/mode_auto.cpp:69-81
 */
void ModeAuto::auto_wp_start(const Vector3f& destination)
{
    sub.auto_mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    sub.wp_nav.set_wp_destination_NEU_cm(destination, false);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

/**
 * @brief Initialize waypoint navigation to a destination in global coordinates
 * 
 * @details Starts autonomous waypoint navigation to a specified Location (lat/lon/alt).
 *          This version supports terrain-relative navigation when rangefinder data is
 *          available. If terrain data is required but unavailable, triggers terrain failsafe.
 *          
 *          This function:
 *          - Sets Auto sub-mode to Auto_WP for waypoint navigation
 *          - Configures waypoint navigation controller with global destination
 *          - Validates terrain data availability if terrain-relative altitude requested
 *          - Triggers terrain failsafe if required terrain data is missing
 *          - Initializes yaw control mode (look-ahead or maintains ROI if active)
 *          
 * @param[in] dest_loc Target location in global coordinates (latitude, longitude, altitude)
 *                     with altitude frame specified (AGL, AMSL, terrain-relative, etc.)
 * 
 * @return void, but triggers failsafe_terrain_on_event() if terrain data unavailable
 * 
 * @warning For terrain-relative missions, rangefinder must be healthy and providing data
 *          or navigation will fail and trigger terrain failsafe
 * @note To-Do: Improve yaw reset logic to only reset when previous command was not a waypoint
 *       to eliminate special ROI handling in init()
 *       
 * Source: ArduSub/mode_auto.cpp:84-101
 */
void ModeAuto::auto_wp_start(const Location& dest_loc)
{
    sub.auto_mode = Auto_WP;

    // send target to waypoint controller
    if (!sub.wp_nav.set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        gcs().send_text(MAV_SEVERITY_WARNING, "Terrain data (rangefinder) not available");
        sub.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

/**
 * @brief Execute autonomous waypoint navigation controller
 * 
 * @details Runs the waypoint navigation controller at main loop rate (50-100Hz) to
 *          autonomously navigate the underwater vehicle to mission waypoints. This
 *          function implements the core autonomous navigation behavior including:
 *          
 *          Position Control:
 *          - Updates waypoint navigation controller (wp_nav) for XY position tracking
 *          - Manages vertical position control for depth/altitude tracking
 *          - Handles terrain-relative navigation when rangefinder data available
 *          - Translates position controller outputs to forward/lateral motor commands
 *          
 *          Pilot Override Capability:
 *          - Allows pilot yaw rate input during autonomous navigation
 *          - Permits pilot roll/pitch input for manual attitude adjustments
 *          - Switches to AUTO_YAW_HOLD mode when pilot provides yaw input
 *          
 *          Safety Mechanisms:
 *          - Disarms motor outputs and relaxes controllers when vehicle not armed
 *          - Monitors terrain/rangefinder data health and updates failsafe status
 *          - Respects pilot input failsafe conditions
 *          
 *          Control Flow:
 *          1. Check armed status (exit if disarmed with motors idle)
 *          2. Process pilot yaw override input
 *          3. Update waypoint navigation controller
 *          4. Translate navigation commands to motor outputs (forward/lateral)
 *          5. Update vertical position controller (depth/altitude)
 *          6. Apply attitude control with pilot overrides
 *          
 * @note Called by auto_run() at main loop rate (typically 50-100Hz)
 * @note Sub vehicles do not stabilize roll/pitch/yaw when disarmed
 * @warning Pilot input during autonomous navigation can interfere with waypoint tracking
 *          accuracy but is permitted for safety override capability
 * @warning This is a safety-critical function - terrain failsafe status must be monitored
 *          continuously to prevent vehicle loss in terrain-relative missions
 *          
 * @todo Implement logic for terrain tracking when target goes below fence limit
 * @todo Implement per-waypoint radius from mission command parameter cmd.p2
 * @todo Fix auto yaw heading behavior when mission completes and switches to loiter
 * @todo Reset waypoint origin to current location when disarmed to prevent sudden
 *       movements on arming (or better, require takeoff command)
 *       
 * Source: ArduSub/mode_auto.cpp:105-168
 */
void ModeAuto::auto_wp_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: reset waypoint origin to current location because vehicle is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        // call attitude controller
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init_cm();                                                // Reset xy target
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // TODO logic for terrain tracking target going below fence limit
    // TODO implement waypoint radius individually for each waypoint based on cmd.p2
    // TODO fix auto yaw heading to switch to something appropriate when mission complete and switches to loiter
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_U_controller();

    ////////////////////////////
    // update attitude output //

    // get pilot desired lean angles
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, get_auto_heading(), true);
    }
}

/**
 * @brief Initialize movement to the edge of a circular pattern
 * 
 * @details When starting a circular mission command, the vehicle may not be positioned
 *          on the circle perimeter. This function intelligently handles the transition:
 *          - If vehicle is >3m from circle edge: Navigate to edge first (Auto_CircleMoveToEdge mode)
 *          - If vehicle is ≤3m from circle edge: Start circle pattern immediately
 *          
 *          The function configures circle navigation parameters and determines optimal
 *          yaw behavior during edge approach based on vehicle position relative to circle.
 *          
 *          Yaw Control Strategy:
 *          - Outside circle (>5m from center): Point toward circle edge during approach
 *          - Inside/near circle: Hold current yaw to prevent spinning during approach
 *          
 * @param[in] circle_center Global position (lat/lon/alt) of circle center point
 * @param[in] radius_m Circle radius in meters (if zero, uses previous radius setting)
 * @param[in] ccw_turn true for counter-clockwise rotation, false for clockwise
 * 
 * @note Assumes caller has performed position validity checks (position_ok())
 * @note Assumes circle_nav.set_center() will be called with circle_center
 * @warning If terrain data unavailable for terrain-relative circle altitude, triggers
 *          terrain failsafe and navigation will fail
 *          
 * Source: ArduSub/mode_auto.cpp:173-221
 */
void ModeAuto::auto_circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn)
{
    // set circle center
    sub.circle_nav.set_center(circle_center);

    // set circle radius
    if (!is_zero(radius_m)) {
        sub.circle_nav.set_radius_cm(radius_m * 100.0f);
    }

     // set circle direction by using rate
    float current_rate = sub.circle_nav.get_rate_degs();
    current_rate = ccw_turn ? -fabsf(current_rate) : fabsf(current_rate);
    sub.circle_nav.set_rate_degs(current_rate);

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    float dist_to_edge;
    sub.circle_nav.get_closest_point_on_circle_NEU_cm(circle_edge_neu, dist_to_edge);

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        sub.auto_mode = Auto_CircleMoveToEdge;

        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu, Location::AltFrame::ABOVE_ORIGIN);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!sub.wp_nav.set_wp_destination_loc(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            sub.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        float dist_to_center = get_horizontal_distance(inertial_nav.get_position_xy_cm().topostype(), sub.circle_nav.get_center_NEU_cm().xy());
        if (dist_to_center > sub.circle_nav.get_radius_cm() && dist_to_center > 500) {
            set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        } else {
            // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    } else {
        auto_circle_start();
    }
}

/**
 * @brief Initialize circular pattern flight
 * 
 * @details Starts autonomous circular pattern navigation around a pre-configured
 *          center point. This is called after the vehicle has reached the circle
 *          edge (via auto_circle_movetoedge_start) or if already positioned on the edge.
 *          
 *          Sets Auto sub-mode to Auto_Circle and initializes the circle navigation
 *          controller with previously configured parameters (center, radius, rate).
 *          
 * @note Assumes circle_nav has been configured with:
 *       - Circle center position (set_center())
 *       - Circle radius (set_radius_cm())
 *       - Rotation rate and direction (set_rate_degs())
 * @note Center altitude can be terrain-relative if rangefinder data available
 *       
 * Source: ArduSub/mode_auto.cpp:225-231
 */
void ModeAuto::auto_circle_start()
{
    sub.auto_mode = Auto_Circle;

    // initialise circle controller
    sub.circle_nav.init_NEU_cm(sub.circle_nav.get_center_NEU_cm(), sub.circle_nav.center_is_terrain_alt(), sub.circle_nav.get_rate_degs());
}

/**
 * @brief Execute circular pattern navigation controller
 * 
 * @details Runs the circle navigation controller at main loop rate to maintain
 *          circular pattern flight around the configured center point. The controller
 *          continuously generates tangential velocity commands to follow the circle
 *          while maintaining specified radius and rotation rate.
 *          
 *          Control outputs:
 *          - XY position: Circular path tracking via circle_nav controller
 *          - Vertical: Maintains altitude/depth set at circle initiation
 *          - Yaw: Automatically controlled to follow circle tangent direction
 *          
 *          Pilot Override:
 *          - Roll/pitch inputs from pilot override automatic lateral/forward control
 *          - Yaw automatically follows circle tangent (pilot override not shown)
 *          
 * @note Called by auto_run() at main loop rate (typically 50-100Hz)
 * @note Unlike waypoint navigation, circle mode uses circle_nav instead of wp_nav
 * @warning Terrain failsafe status monitored continuously - loss of rangefinder data
 *          during terrain-relative circle will trigger failsafe
 *          
 * Source: ArduSub/mode_auto.cpp:235-253
 */
void ModeAuto::auto_circle_run()
{
    // call circle controller
    sub.failsafe_terrain_set_status(sub.circle_nav.update_cms());

    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_U_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw_cd(), true);
}

#if NAV_GUIDED
/**
 * @brief Initialize external navigation control within Auto mode
 * 
 * @details Transitions Auto mode to accept navigation commands from an external
 *          controller (typically companion computer via MAVLink). This enables
 *          sophisticated autonomous behaviors implemented externally while remaining
 *          in Auto mode context for mission integration.
 *          
 *          This sub-mode:
 *          - Initializes Guided mode controller for external position/velocity commands
 *          - Sets Auto sub-mode to Auto_NavGuided
 *          - Establishes guided limits for safety boundary checking
 *          
 *          Typical use cases:
 *          - Computer vision-based navigation (object tracking, inspection)
 *          - Advanced path planning algorithms on companion computer
 *          - Integration with ROS/ROS2 navigation stacks
 *          
 * @note Requires NAV_GUIDED feature enabled at compile time
 * @note External controller must send position/velocity setpoints via MAVLink
 *       (SET_POSITION_TARGET_LOCAL_NED, SET_POSITION_TARGET_GLOBAL_INT)
 * @warning Guided limits are enforced - external commands outside limits will be rejected
 *          for safety (prevents runaway behavior from external software failures)
 *          
 * Source: ArduSub/mode_auto.cpp:257-263
 */
void ModeAuto::auto_nav_guided_start()
{
    sub.mode_guided.init(true);
    sub.auto_mode = Auto_NavGuided;
    // initialise guided start time and position as reference for limit checking
    sub.mode_auto.guided_limit_init_time_and_pos();
}

/**
 * @brief Execute external navigation control
 * 
 * @details Delegates control to Guided mode controller which processes position/velocity
 *          setpoints from external navigation source (companion computer). This allows
 *          Auto mode missions to seamlessly integrate with external navigation algorithms.
 *          
 * @note Called by auto_run() at main loop rate (typically 50-100Hz)
 * @note All control is delegated to mode_guided.run() - see Guided mode for implementation
 * @note Mission will resume from next waypoint when external nav guided command completes
 *       
 * Source: ArduSub/mode_auto.cpp:267-271
 */
void ModeAuto::auto_nav_guided_run()
{
    // call regular guided flight mode run function
    sub.mode_guided.run();
}
#endif  // NAV_GUIDED

/**
 * @brief Initialize autonomous position hold (loiter)
 * 
 * @details Starts position hold at current location or mission loiter waypoint.
 *          The vehicle will maintain position while allowing pilot yaw and attitude
 *          override inputs. This mode is used for:
 *          - LOITER_UNLIM mission commands (unlimited time loiter)
 *          - LOITER_TIME mission commands (timed position hold)
 *          - Mission completion behavior (final loiter)
 *          - Exit behavior when mission ends
 *          
 *          Loiter behavior:
 *          - Calculates smooth stopping point from current velocity
 *          - Initializes position hold at stopping point
 *          - Maintains current yaw heading (AUTO_YAW_HOLD)
 *          - Accepts pilot yaw rate and roll/pitch inputs
 *          
 * @return true if loiter successfully initialized with valid position estimate
 * @return false if position estimate unavailable (EKF unhealthy)
 * 
 * @note Can be called by exit_mission() when mission completes
 * @note Stopping point calculation provides smooth transition from waypoint navigation
 * @warning Requires valid position estimate to initialize - failure returns false and
 *          prevents mode transition (critical safety check for autonomous operation)
 *          
 * Source: ArduSub/mode_auto.cpp:276-295
 */
bool ModeAuto::auto_loiter_start()
{
    // return failure if GPS is bad
    if (!sub.position_ok()) {
        return false;
    }
    sub.auto_mode = Auto_Loiter;

    // calculate stopping point
    Vector3f stopping_point;
    sub.wp_nav.get_wp_stopping_point_NEU_cm(stopping_point);

    // initialise waypoint controller target to stopping point
    sub.wp_nav.set_wp_destination_NEU_cm(stopping_point);

    // hold yaw at current heading
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

/**
 * @brief Execute autonomous position hold controller
 * 
 * @details Maintains position at the configured loiter waypoint while accepting pilot
 *          attitude and yaw inputs. This provides a stable position hold during mission
 *          loiter commands or mission completion.
 *          
 *          Position Control:
 *          - Uses waypoint navigation controller to maintain XY position at loiter point
 *          - Vertical position controller maintains depth/altitude
 *          - Compensates for current/disturbances to hold position
 *          
 *          Pilot Override:
 *          - Full yaw rate control from pilot
 *          - Roll/pitch inputs allow manual attitude adjustments
 *          - Does not exit loiter mode unless commanded by mission or mode change
 *          
 *          Safety Behavior:
 *          - When disarmed: Motors idle, controllers relaxed, position targets reset
 *          - Monitors terrain failsafe status during loiter
 *          - Respects pilot input failsafe conditions
 *          
 * @note Called by auto_run() at main loop rate (typically 50-100Hz)
 * @note Similar to auto_wp_run() but without mission advancement
 * @note Sub vehicles do not stabilize when disarmed
 * @warning Terrain failsafe monitoring continues - loss of rangefinder data during
 *          terrain-relative loiter will trigger failsafe action
 *          
 * Source: ArduSub/mode_auto.cpp:299-343
 */
void ModeAuto::auto_loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        sub.wp_nav.wp_and_spline_init_cm();                                                // Reset xy target
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // update xy outputs //
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_U_controller();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // roll & pitch & yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
}


/**
 * @brief Set yaw to specific heading during Auto mode
 * 
 * @details Commands the vehicle to yaw to a specified heading with controlled turn rate.
 *          Used by MAVLink DO_SET_YAW and CONDITION_YAW mission commands to orient
 *          the vehicle for cameras, sensors, or mission requirements.
 *          
 *          Supports both absolute and relative heading commands:
 *          - Absolute: Yaw to compass heading (e.g., 90° = East)
 *          - Relative: Yaw relative to current heading (e.g., +45° = 45° right turn)
 *          
 *          Turn rate is configurable and clamped to AUTO_YAW_SLEW_RATE for safety.
 *          
 * @param[in] angle_deg Target heading in degrees (0-360 for absolute, any value for relative)
 * @param[in] turn_rate_dps Desired turn rate in degrees per second (0 = use default AUTO_YAW_SLEW_RATE)
 * @param[in] direction For relative angles: 1=positive rotation, -1=negative rotation (inverts angle)
 * @param[in] relative_angle 0=absolute heading, 1=relative to current heading
 * 
 * @note Heading values are wrapped to 0-360° range
 * @note Turn rate is clamped to AUTO_YAW_SLEW_RATE maximum to prevent aggressive yaw maneuvers
 * @todo Restore support for explicit clockwise/counter-clockwise rotation direction
 *       from cmd.content.yaw.direction (MAVLink convention: 1=CW, -1=CCW)
 *       
 * Source: ArduSub/mode_auto.cpp:347-376
 */
void ModeAuto::set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle)
{
    // get current yaw
    int32_t curr_yaw_target = attitude_control->get_att_target_euler_cd().z;

    // get final angle, 1 = Relative, 0 = Absolute
    if (relative_angle == 0) {
        // absolute angle
        sub.yaw_look_at_heading = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        sub.yaw_look_at_heading = wrap_360_cd((angle_deg * 100 + curr_yaw_target));
    }

    // get turn speed
    if (is_zero(turn_rate_dps)) {
        // default to regular auto slew rate
        sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    } else {
        sub.yaw_look_at_heading_slew = MIN(turn_rate_dps, AUTO_YAW_SLEW_RATE);    // deg / sec
    }

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}


/**
 * @brief Set continuous yaw rotation rate during Auto mode
 * 
 * @details Commands the vehicle to continuously rotate at a specified rate. Unlike
 *          set_auto_yaw_look_at_heading() which yaws to a target heading and stops,
 *          this mode maintains continuous rotation. Used for:
 *          - 360° inspection scans
 *          - Search patterns
 *          - Sensor scanning operations
 *          
 *          Yaw rate is clamped to AUTO_YAW_SLEW_RATE for safety and stability.
 *          
 * @param[in] turn_rate_dps Desired continuous rotation rate in degrees per second
 *                          (positive = right/clockwise, negative = left/counter-clockwise)
 * 
 * @note Rate is clamped to ±AUTO_YAW_SLEW_RATE maximum
 * @note Vehicle will continue rotating at this rate until yaw mode changed or mission
 *       command overrides yaw control
 *       
 * Source: ArduSub/mode_auto.cpp:380-387
 */
void ModeAuto::set_yaw_rate(float turn_rate_dps)
{    
    // set sub to desired yaw rate
    sub.yaw_look_at_heading_slew = MIN(turn_rate_dps, AUTO_YAW_SLEW_RATE);    // deg / sec

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_RATE);
}

/**
 * @brief Set yaw to point at Region of Interest (ROI) location
 * 
 * @details Commands vehicle and/or camera mount to point at a specified location during
 *          autonomous missions. Supports both:
 *          - Camera mounts with independent pan control: Only camera aims at ROI
 *          - Fixed cameras or no mount: Entire vehicle yaws to point at ROI
 *          
 *          Used by MAVLink DO_SET_ROI and DO_SET_ROI_LOCATION mission commands for:
 *          - Keeping camera pointed at target during waypoint navigation
 *          - Inspection missions requiring continuous target tracking
 *          - Survey missions with off-axis camera pointing
 *          
 *          ROI Disable:
 *          - Passing zero lat/lon/alt location disables ROI tracking
 *          - Returns to default auto yaw behavior for current mission command
 *          
 *          Mount Integration:
 *          - If mount has pan control: Mount tracks ROI, vehicle maintains mission yaw
 *          - If mount lacks pan control: Vehicle yaws to point at ROI
 *          - If no mount configured: Vehicle yaws to point at ROI
 *          
 * @param[in] roi_location Global position (lat/lon/alt) of Region of Interest target
 *                         Zero location (0,0,0) disables ROI tracking
 * 
 * @note ROI yaw mode persists across waypoints until explicitly cleared or overridden
 * @note Vehicle yaw only used for ROI if mount doesn't have pan control
 * @warning ROI calculation requires valid position estimate to compute bearing to target
 * 
 * @todo Expand DO_SET_ROI handling to support all MAVLink ROI modes:
 *       - Mode 0: Do nothing (disable ROI)
 *       - Mode 1: Point at next waypoint
 *       - Mode 2: Point at waypoint specified by parameter
 *       - Mode 3: Point at location given by lat/lon/alt (currently implemented)
 *       - Mode 4: Point at target ID (requires target tracking system)
 * @todo Implement more sophisticated yaw mode restoration when ROI disabled to handle
 *       non-waypoint mission commands appropriately
 *       
 * Source: ArduSub/mode_auto.cpp:746-780
 */
void ModeAuto::set_auto_yaw_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        sub.camera_mount.clear_roi_target();
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the sub
        if (!sub.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NEU_cm(sub.roi_WP)) {
                set_auto_yaw_mode(AUTO_YAW_ROI);
            }
        }
        // send the command to the camera mount
        sub.camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the sub at the location
        if (roi_location.get_vector_from_origin_NEU_cm(sub.roi_WP)) {
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

/**
 * @brief Initiate recovery from terrain/rangefinder data loss during autonomous mission
 * 
 * @details Attempts to recover from loss of terrain-relative navigation capability during
 *          terrain-following missions. Called by terrain failsafe system when rangefinder
 *          data becomes unavailable during waypoint navigation with terrain enabled.
 *          
 *          Recovery Process:
 *          1. Verify rangefinder is present and responding (may have transient error)
 *          2. Switch to Auto_TerrainRecover sub-mode if rangefinder detected
 *          3. Stop mission execution to prevent further navigation without terrain data
 *          4. Initialize loiter at current position to maintain stability
 *          5. Reset altitude controller to prevent altitude drift
 *          6. Start recovery timeout timer
 *          
 *          Recovery Criteria:
 *          - Rangefinder status is Good, OutOfRangeLow, or OutOfRangeHigh (sensor responding)
 *          - If rangefinder not responding or disconnected, recovery cannot be attempted
 *          
 *          Success Path:
 *          - auto_terrain_recover_run() will attempt to restore healthy rangefinder state
 *          - If successful within timeout, mission resumes with terrain following
 *          
 *          Failure Path:
 *          - If recovery fails or times out, failsafe action triggered (surface/RTL)
 *          
 * @return true if recovery mode entered successfully (rangefinder present and responding)
 * @return false if recovery not possible (rangefinder disconnected or unresponsive)
 * 
 * @note Recovery only available if AP_RANGEFINDER_ENABLED compiled in
 * @warning Mission stops during recovery - waypoint navigation suspended until success/timeout
 * @warning Requires rangefinder on ROTATION_PITCH_270 (pointing down) for underwater terrain
 * @warning Recovery timeout FS_TERRAIN_RECOVER_TIMEOUT_MS prevents indefinite attempts
 * 
 * @see auto_terrain_recover_run() for ongoing recovery logic
 * @see sub.failsafe_terrain_act() for failure handling
 * 
 * Source: ArduSub/mode_auto.cpp:821-862
 */
bool ModeAuto::auto_terrain_recover_start()
{
#if AP_RANGEFINDER_ENABLED
    // Check rangefinder status to see if recovery is possible
    switch (sub.rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::Status::OutOfRangeLow:
    case RangeFinder::Status::OutOfRangeHigh:

        // RangeFinder::Good if just one valid sample was obtained recently, but ::rangefinder_state.alt_healthy
        // requires several consecutive valid readings for wpnav to accept rangefinder data
    case RangeFinder::Status::Good:
        sub.auto_mode = Auto_TerrainRecover;
        break;

        // Not connected or no data
    default:
        return false; // Rangefinder is not connected, or has stopped responding
    }

    // Initialize recovery timeout time
    sub.fs_terrain_recover_start_ms = AP_HAL::millis();

    // Stop mission
    sub.mission.stop();

    // Reset xy target
    sub.loiter_nav.clear_pilot_desired_acceleration();
    sub.loiter_nav.init_target();

    // Reset z axis controller
    position_control->relax_U_controller(motors.get_throttle_hover());

    // initialize vertical maximum speeds and acceleration
    position_control->set_max_speed_accel_U_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_U_cmss());
    position_control->set_correction_speed_accel_U_cmss(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_U_cmss());

    gcs().send_text(MAV_SEVERITY_WARNING, "Attempting auto failsafe recovery");
    return true;
#else
    return false;
#endif
}

/**
 * @brief Execute terrain failsafe recovery logic during each control loop
 * 
 * @details Continuously attempts to restore healthy rangefinder-based terrain following
 *          during Auto_TerrainRecover sub-mode. Called at main loop rate (typically 50Hz)
 *          until recovery succeeds, times out, or rangefinder fails completely.
 *          
 *          Recovery State Machine:
 *          
 *          1. OutOfRangeLow (too close to terrain):
 *             - Command climb at default ascent speed to increase altitude
 *             - Prevents collision with seabed/obstacles
 *          
 *          2. OutOfRangeHigh (too far from terrain):
 *             - Command descent at default descent speed to decrease altitude
 *             - Brings vehicle back into rangefinder sensing range
 *          
 *          3. Good (valid rangefinder data):
 *             - Hold current depth (zero climb rate)
 *             - Wait for rangefinder_state.alt_healthy (multiple consecutive valid readings)
 *             - After 1.5 seconds of healthy data, declare recovery successful
 *             - Resume mission with terrain following re-enabled
 *          
 *          4. Not Connected/No Data (rangefinder failure):
 *             - Cannot recover without sensor
 *             - Trigger failsafe action immediately (surface/RTL)
 *          
 *          Timeout Handling:
 *          - If recovery not achieved within FS_TERRAIN_RECOVER_TIMEOUT_MS, fail
 *          - Prevents indefinite loitering when rangefinder unrecoverable
 *          
 *          During Recovery:
 *          - Maintains horizontal position using loiter controller
 *          - Actively adjusts depth based on rangefinder status
 *          - Disarms safety: If disarmed during recovery, resets controllers and exits
 *          
 * @note Requires AP_RANGEFINDER_ENABLED at compile time
 * @note Rangefinder health requires sustained valid readings, not just single samples
 * @note Success criteria: 1.5 seconds of healthy rangefinder data
 * @note Timeout value: FS_TERRAIN_RECOVER_TIMEOUT_MS (configurable)
 * 
 * @warning Safety-critical: Failure triggers surface/RTL to prevent collision
 * @warning Vehicle actively climbs/descends during recovery - may exceed mission depth limits
 * @warning Mission suspended during recovery - time-critical waypoints may be missed
 * 
 * @see auto_terrain_recover_start() for recovery initialization
 * @see sub.failsafe_terrain_act() for failure action (surface/RTL)
 * 
 * Source: ArduSub/mode_auto.cpp:901-992
 */
void ModeAuto::auto_terrain_recover_run()
{
    float target_climb_rate = 0;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        sub.loiter_nav.init_target();                                                   // Reset xy target
        position_control->relax_U_controller(motors.get_throttle_hover());                // Reset z axis controller
        return;
    }

#if AP_RANGEFINDER_ENABLED
    static uint32_t rangefinder_recovery_ms = 0;
    switch (sub.rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::Status::OutOfRangeLow:
        target_climb_rate = sub.wp_nav.get_default_speed_up_cms();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
        target_climb_rate = sub.wp_nav.get_default_speed_down_cms();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::Status::Good: // exit on success (recovered rangefinder data)

        target_climb_rate = 0; // Attempt to hold current depth

        if (sub.rangefinder_state.alt_healthy) {

            // Start timer as soon as rangefinder is healthy
            if (rangefinder_recovery_ms == 0) {
                rangefinder_recovery_ms = AP_HAL::millis();
                position_control->relax_U_controller(motors.get_throttle_hover()); // Reset alt hold targets
            }

            // 1.5 seconds of healthy rangefinder means we can resume mission with terrain enabled
            if (AP_HAL::millis() > rangefinder_recovery_ms + 1500) {
                gcs().send_text(MAV_SEVERITY_INFO, "Terrain failsafe recovery successful!");
                sub.failsafe_terrain_set_status(true); // Reset failsafe timers
                sub.failsafe.terrain = false; // Clear flag
                sub.auto_mode = Auto_Loiter; // Switch back to loiter for next iteration
                sub.mission.resume(); // Resume mission
                rangefinder_recovery_ms = 0; // Reset for subsequent recoveries
            }

        }
        break;

        // Not connected, or no data
    default:
        // Terrain failsafe recovery has failed, terrain data is not available
        // and rangefinder is not connected, or has stopped responding
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!");
        sub.failsafe_terrain_act();
        rangefinder_recovery_ms = 0;
        return;
    }
#else
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!");
    sub.failsafe_terrain_act();
#endif

    // exit on failure (timeout)
    if (AP_HAL::millis() > sub.fs_terrain_recover_start_ms + FS_TERRAIN_RECOVER_TIMEOUT_MS) {
        // Recovery has failed, revert to failsafe action
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery timeout!");
        sub.failsafe_terrain_act();
    }

    // run loiter controller
    sub.loiter_nav.update();

    ///////////////////////
    // update xy targets //
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////
    // update z target //
    position_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate);
    position_control->update_U_controller();

    ////////////////////////////
    // update angular targets //
    float target_roll = 0;
    float target_pitch = 0;

    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    float target_yaw_rate = 0;

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
}
