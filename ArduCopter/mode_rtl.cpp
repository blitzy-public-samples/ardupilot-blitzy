#include "Copter.h"

#if MODE_RTL_ENABLED

/**
 * @file mode_rtl.cpp
 * @brief RTL (Return To Launch) flight mode implementation for ArduCopter
 * 
 * @details RTL mode implements autonomous return to home functionality, which is one of the most
 *          safety-critical flight modes in ArduPilot. It is automatically triggered by failsafe
 *          conditions (radio loss, battery failsafe, geofence breach) and can be manually invoked
 *          by the pilot via mode switch or GCS command.
 * 
 *          RTL executes a multi-stage sequence to safely return the vehicle to the launch point
 *          or a designated rally point and land. The sequence consists of:
 *          
 *          1. STARTING: Initial state, builds the return path
 *          2. INITIAL_CLIMB: Climbs to RTL_ALT to clear obstacles
 *          3. RETURN_HOME: Navigates horizontally to home/rally point at RTL_ALT
 *          4. LOITER_AT_HOME: Loiters for RTL_LOIT_TIME, aligns to armed yaw heading
 *          5. FINAL_DESCENT: Descends to RTL_ALT_FINAL (may be above ground)
 *          6. LAND: Final landing with landing detector
 * 
 *          RTL State Machine Diagram:
 *          ```mermaid
 *          stateDiagram-v2
 *              [*] --> STARTING: RTL Mode Activated
 *              STARTING --> INITIAL_CLIMB: build_path() complete
 *              INITIAL_CLIMB --> RETURN_HOME: Reached RTL_ALT
 *              RETURN_HOME --> LOITER_AT_HOME: Reached home/rally point
 *              LOITER_AT_HOME --> FINAL_DESCENT: RTL_LOIT_TIME elapsed & yaw aligned
 *              LOITER_AT_HOME --> LAND: RTL_ALT_FINAL=0 or radio failsafe
 *              FINAL_DESCENT --> LAND: Reached RTL_ALT_FINAL
 *              LAND --> [*]: Touchdown & disarm
 *              
 *              note right of INITIAL_CLIMB
 *                  Climbs to RTL_ALT (default 15m)
 *                  Respects RTL_CLIMB_MIN
 *                  Can use RTL_CONE_SLOPE for gradual climb
 *              end note
 *              
 *              note right of RETURN_HOME
 *                  Navigates to nearest rally point or home
 *                  Maintains RTL_ALT throughout
 *                  Supports terrain following if enabled
 *              end note
 *              
 *              note right of LOITER_AT_HOME
 *                  Loiters for RTL_LOIT_TIME seconds
 *                  Rotates to initial armed yaw
 *                  Allows pilot repositioning
 *              end note
 *          ```
 * 
 * Key Configuration Parameters:
 * - RTL_ALT (cm): Target altitude above home for return journey (default 1500cm = 15m)
 * - RTL_ALT_FINAL (cm): Altitude above home for final descent before land (default 0 = land immediately)
 * - RTL_CLIMB_MIN (cm): Minimum climb regardless of current altitude (default 0)
 * - RTL_CONE_SLOPE: Defines cone for gradual climb when close to home (default 3.0, 0=disabled)
 * - RTL_SPEED (cm/s): Horizontal speed during return (default from WPNAV_SPEED)
 * - RTL_LOIT_TIME (ms): Time to loiter at home before descending (default 5000ms = 5 seconds)
 * - RTL_ALT_TYPE: Altitude frame reference (0=relative to home, 1=relative to terrain)
 * 
 * Terrain Following:
 * When RTL_ALT_TYPE=1 (terrain), RTL maintains constant height above ground using either
 * rangefinder or terrain database. This is critical for operations in mountainous terrain
 * where a fixed altitude above home could result in ground collision.
 * 
 * Rally Point Support:
 * If rally points are enabled (HAL_RALLY_ENABLED), RTL selects the nearest rally point
 * instead of home. Rally points can be positioned for better landing zones or to avoid
 * obstacles near the launch point.
 * 
 * Safety Considerations:
 * - RTL is a primary failsafe mode - reliability is critical for vehicle recovery
 * - Terrain data loss during RTL triggers restart without terrain following
 * - Altitude fence limits are respected to prevent airspace violations
 * - Landing detector prevents premature disarm during descent
 * - Pilot can override with throttle during final descent (if THR_BEHAVE allows)
 * 
 * Thread Safety: Called from main scheduler loop at 400Hz
 * 
 * @warning Modifying RTL behavior affects primary failsafe recovery - changes must be
 *          thoroughly tested in SITL and on actual hardware with multiple scenarios
 * 
 * @see ModeRTL class definition in mode.h
 * @see AC_WPNav library for waypoint navigation
 * @see AP_Rally library for rally point management
 * @see Parameters documentation for RTL_* parameter details
 * 
 * Source: ArduCopter/mode_rtl.cpp
 */

/**
 * @brief Initialize RTL mode and prepare for autonomous return to launch
 * 
 * @details Initializes the RTL mode controller, setting up waypoint navigation and resetting
 *          the state machine to STARTING. This function performs critical pre-flight checks
 *          unless explicitly bypassed (e.g., during failsafe entry where immediate action
 *          is required).
 *          
 *          Initialization sequence:
 *          1. Verify home position is set (unless ignore_checks=true)
 *          2. Initialize waypoint and spline controller with RTL_SPEED
 *          3. Reset state machine to STARTING with _state_complete=true
 *          4. Determine terrain following eligibility based on terrain failsafe state
 *          5. Reset landing repositioning and precision landing flags
 *          6. Initialize precision landing state machine if enabled
 *          
 *          The terrain_following_allowed flag is set based on whether a terrain failsafe
 *          is currently active. If terrain data was lost, RTL will operate in altitude-above-home
 *          mode even if RTL_ALT_TYPE is set to terrain mode.
 * 
 * @param[in] ignore_checks If true, bypasses the home position check. Used when RTL is
 *                          triggered by failsafe conditions where immediate return is
 *                          required even if home is not optimally set.
 * 
 * @return true if initialization successful and RTL can proceed
 * @return false if initialization failed (home not set and checks not ignored)
 * 
 * @note Called by mode selection logic when pilot switches to RTL or when failsafe triggers RTL
 * @note If home is not set and ignore_checks=false, mode change will be rejected
 * @note This function is called at mode entry, not at high frequency
 * 
 * @warning Failing to set home position before RTL will prevent mode entry unless
 *          triggered by failsafe. Always wait for GPS fix and home position before
 *          arming in modes that may trigger RTL as failsafe.
 * 
 * @see ModeRTL::run() for the main RTL execution loop
 * @see ModeRTL::build_path() for RTL path construction
 */
bool ModeRTL::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_cm(g.rtl_speed_cms);
    _state = SubMode::STARTING;
    _state_complete = true; // see run() method below
    terrain_following_allowed = !copter.failsafe.terrain;
    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

#if AC_PRECLAND_ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

// re-start RTL with terrain following disabled
void ModeRTL::restart_without_terrain()
{
#if HAL_LOGGING_ENABLED
    LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
#endif
    terrain_following_allowed = false;
    _state = SubMode::STARTING;
    _state_complete = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
}

ModeRTL::RTLAltType ModeRTL::get_alt_type() const
{
    // sanity check parameter
    switch ((ModeRTL::RTLAltType)g.rtl_alt_type) {
    case RTLAltType::RELATIVE ... RTLAltType::TERRAIN:
        return g.rtl_alt_type;
    }
    // user has an invalid value
    return RTLAltType::RELATIVE;
}

/**
 * @brief Main RTL mode execution function - runs the return-to-launch state machine
 * 
 * @details This is the primary RTL controller function called at high frequency (typically 400Hz)
 *          from the main scheduler loop. It implements a two-level control architecture:
 *          
 *          1. High-level state machine: Manages transitions between RTL stages (climb, return,
 *             loiter, descend, land) based on completion of each stage
 *          2. Low-level controllers: Executes the appropriate controller (waypoint nav, loiter,
 *             landing) for the current state
 *          
 *          State Transition Logic:
 *          - Transitions occur when _state_complete flag is set by the current stage controller
 *          - Each transition calls a *_start() function to initialize the next stage
 *          - Some transitions have conditional logic (e.g., LOITER_AT_HOME can go to either
 *            LAND or FINAL_DESCENT depending on RTL_ALT_FINAL and radio failsafe state)
 *          
 *          State Execution:
 *          - STARTING: Should not be reached; immediately transitions to INITIAL_CLIMB
 *          - INITIAL_CLIMB & RETURN_HOME: Both use climb_return_run() (waypoint controller)
 *          - LOITER_AT_HOME: Uses loiterathome_run() for loiter and yaw alignment
 *          - FINAL_DESCENT: Uses descent_run() for controlled descent to RTL_ALT_FINAL
 *          - LAND: Uses land_run() for final landing sequence with landing detector
 *          
 *          The function first checks if motors are armed and returns immediately if not.
 *          Then it checks _state_complete flag to determine if state transition is needed.
 *          Finally, it calls the appropriate run function for the current state.
 * 
 * @param[in] disarm_on_land If true, automatically disarms motors when landing detector
 *                           confirms touchdown. Typically true for RTL, but may be false
 *                           in some autonomous mission contexts.
 * 
 * @note Called at main loop rate (typically 400Hz on modern flight controllers)
 * @note Early return if motors not armed - prevents control output when disarmed
 * @note State machine is designed to be robust to state completion flag timing
 * 
 * @warning This is a safety-critical function running in the main control loop.
 *          Any modifications must maintain deterministic execution time and not
 *          introduce blocking operations or excessive computation.
 * 
 * @see ModeRTL::climb_return_run() for climb and return stage implementation
 * @see ModeRTL::loiterathome_run() for loiter stage implementation
 * @see ModeRTL::descent_run() for final descent stage implementation
 * @see ModeRTL::land_run() for landing stage implementation
 */
void ModeRTL::run(bool disarm_on_land)
{
    if (!motors->armed()) {
        return;
    }

    // check if we need to move to next state
    if (_state_complete) {
        switch (_state) {
        case SubMode::STARTING:
            build_path();
            climb_start();
            break;
        case SubMode::INITIAL_CLIMB:
            return_start();
            break;
        case SubMode::RETURN_HOME:
            loiterathome_start();
            break;
        case SubMode::LOITER_AT_HOME:
            if (rtl_path.land || copter.failsafe.radio) {
                land_start();
            } else {
                descent_start();
            }
            break;
        case SubMode::FINAL_DESCENT:
            // do nothing
            break;
        case SubMode::LAND:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (_state) {

    case SubMode::STARTING:
        // should not be reached:
        _state = SubMode::INITIAL_CLIMB;
        FALLTHROUGH;

    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
        climb_return_run();
        break;

    case SubMode::LOITER_AT_HOME:
        loiterathome_run();
        break;

    case SubMode::FINAL_DESCENT:
        descent_run();
        break;

    case SubMode::LAND:
        land_run(disarm_on_land);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void ModeRTL::climb_start()
{
    _state = SubMode::INITIAL_CLIMB;
    _state_complete = false;

    // set the destination
    if (!wp_nav->set_wp_destination_loc(rtl_path.climb_target) || !wp_nav->set_wp_destination_next_loc(rtl_path.return_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }

    // hold current yaw during initial climb
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
}

/**
 * @brief Initialize the RETURN_HOME stage of RTL
 * 
 * @details Transitions RTL from INITIAL_CLIMB to RETURN_HOME state. This function configures
 *          the waypoint controller to navigate horizontally to the return target (home position
 *          or rally point) while maintaining the current altitude achieved during initial climb.
 *          
 *          The return target was previously computed by build_path() and stored in
 *          rtl_path.return_target. This target may be:
 *          - Home position (if no rally points configured)
 *          - Nearest rally point (if HAL_RALLY_ENABLED and rally points defined)
 *          
 *          If setting the waypoint destination fails (typically due to missing terrain data
 *          when using terrain-relative altitude mode), the function automatically restarts
 *          RTL without terrain following by calling restart_without_terrain().
 *          
 *          Yaw control is set to auto_yaw default mode, which typically means the vehicle
 *          points toward the return target during navigation.
 * 
 * @note Called automatically by run() when INITIAL_CLIMB stage completes
 * @note Sets _state_complete=false; will be set true when waypoint reached
 * @note Terrain following mode may be disabled if terrain data unavailable
 * 
 * @warning If waypoint setting fails, RTL restarts from STARTING state without terrain
 *          following. This is a safety mechanism to ensure RTL continues even with
 *          terrain data loss, though altitude reference changes to home-relative.
 * 
 * @see ModeRTL::build_path() for return target computation
 * @see ModeRTL::compute_return_target() for rally point selection logic
 * @see ModeRTL::restart_without_terrain() for terrain failsafe handling
 * @see ModeRTL::climb_return_run() for the execution of this stage
 */
void ModeRTL::return_start()
{
    _state = SubMode::RETURN_HOME;
    _state_complete = false;

    if (!wp_nav->set_wp_destination_loc(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    auto_yaw.set_mode_to_default(true);
}

/**
 * @brief Execute climb and return stages using waypoint controller
 * 
 * @details Implements the INITIAL_CLIMB and RETURN_HOME stages of RTL, both of which rely
 *          on the waypoint navigation controller (wp_nav). This function runs at high frequency
 *          (typically 400Hz) and performs the following control sequence:
 *          
 *          Control Flow:
 *          1. Safety check: If disarmed or landed, make motors safe and return
 *          2. Set motor spool state to THROTTLE_UNLIMITED for full control authority
 *          3. Update waypoint controller (wp_nav->update_wpnav())
 *             - Computes position and velocity targets along the path
 *             - Handles terrain following if enabled
 *             - Sets vertical position control targets
 *          4. Update vertical position controller (pos_control->update_U_controller())
 *             - Converts altitude targets to throttle output
 *             - Implements altitude hold with velocity feedforward
 *          5. Update attitude controller with thrust vector and yaw heading
 *             - Converts position controller output to attitude targets
 *             - Applies auto-yaw (typically pointing toward target)
 *          6. Check completion: Set _state_complete when waypoint reached
 *          
 *          During INITIAL_CLIMB:
 *          - Vehicle climbs vertically to rtl_path.climb_target altitude
 *          - Yaw typically held at current heading (AutoYaw::Mode::HOLD)
 *          - Minimal horizontal movement (climb mostly vertical)
 *          
 *          During RETURN_HOME:
 *          - Vehicle navigates horizontally to rtl_path.return_target
 *          - Altitude maintained at RTL_ALT (or terrain-relative equivalent)
 *          - Yaw typically points toward home/rally point
 *          
 *          Terrain Following:
 *          The wp_nav->update_wpnav() call returns terrain following status, which is
 *          passed to failsafe_terrain_set_status(). If terrain data is lost during
 *          navigation, a terrain failsafe may be triggered.
 * 
 * @note Called at main loop rate (typically 400Hz)
 * @note Shared by INITIAL_CLIMB and RETURN_HOME states for code efficiency
 * @note Sets _state_complete=true when wp_nav reports destination reached
 * @note Early exit with make_safe_ground_handling() if disarmed or landed
 * 
 * @warning Safety-critical function in main control loop - must execute quickly
 * @warning Terrain data loss during this stage may trigger failsafe actions
 * @warning Motor spool state changes affect control authority and must be appropriate
 *          for current flight phase
 * 
 * @see AC_WPNav::update_wpnav() for waypoint navigation implementation
 * @see AC_PosControl::update_U_controller() for vertical position control
 * @see AC_AttitudeControl::input_thrust_vector_heading() for attitude control
 */
void ModeRTL::climb_return_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// loiterathome_start - initialise return to home
void ModeRTL::loiterathome_start()
{
    _state = SubMode::LOITER_AT_HOME;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if (auto_yaw.default_mode(true) != AutoYaw::Mode::HOLD) {
        auto_yaw.set_mode(AutoYaw::Mode::RESET_TO_ARMED_YAW);
    } else {
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }
}

/**
 * @brief Execute loiter at home stage with optional yaw realignment
 * 
 * @details Implements the LOITER_AT_HOME stage of RTL, maintaining position over the home
 *          or rally point for a configurable duration (RTL_LOIT_TIME) while optionally
 *          rotating the vehicle back to its initial armed yaw heading. This stage serves
 *          multiple purposes:
 *          
 *          1. Position Stabilization: Ensures vehicle is centered over landing point before
 *             descent, compensating for any waypoint navigation overshoot or wind drift
 *          2. Yaw Realignment: Rotates vehicle back to takeoff heading for consistent landing
 *             orientation, which can be important for camera direction, landing gear, or
 *             operator situational awareness
 *          3. Pilot Verification: Provides time window for pilot to verify landing zone is
 *             clear and safe, or to switch to manual mode if needed
 *          
 *          Control Flow:
 *          1. Safety check: If disarmed or landed, make motors safe and return
 *          2. Set motor spool state to THROTTLE_UNLIMITED
 *          3. Update waypoint controller to maintain position
 *          4. Update vertical position controller for altitude hold
 *          5. Update attitude controller with thrust vector and yaw command
 *          6. Check completion conditions:
 *             a. RTL_LOIT_TIME duration elapsed AND
 *             b. Yaw aligned to armed heading (within 2 degrees) if yaw reset active
 *          
 *          Yaw Behavior:
 *          - If auto_yaw is in RESET_TO_ARMED_YAW mode: Vehicle rotates to initial takeoff
 *            heading and completion waits for yaw alignment within 2 degrees
 *          - If auto_yaw is in HOLD mode: Vehicle maintains current heading and completion
 *            only requires RTL_LOIT_TIME elapsed
 *          
 *          The loiter duration (RTL_LOIT_TIME) is configurable from 0 to prevent loitering
 *          if immediate descent is desired, though some loiter time is recommended for
 *          position stabilization.
 * 
 * @note Called at main loop rate (typically 400Hz) when in LOITER_AT_HOME state
 * @note Loiter start time recorded in _loiter_start_time by loiterathome_start()
 * @note Yaw alignment tolerance is hard-coded to 2 degrees (may be parameterized in future)
 * @note Position control uses same waypoint controller as return home stage
 * 
 * @warning The 2-degree yaw tolerance check uses actual heading (ahrs.get_yaw_rad()) rather
 *          than target heading. If yaw control is lost, this may prevent completion.
 *          Future improvement could use target heading for more robust operation.
 * 
 * @see ModeRTL::loiterathome_start() for initialization of this stage
 * @see Parameters: RTL_LOIT_TIME for loiter duration configuration
 * @see AutoYaw::Mode::RESET_TO_ARMED_YAW for yaw realignment behavior
 */
void ModeRTL::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw.mode() == AutoYaw::Mode::RESET_TO_ARMED_YAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            // todo: Use the target heading instead of the actual heading to allow landing even if yaw control is lost.
            if (fabsf(wrap_PI(ahrs.get_yaw_rad() - copter.initial_armed_bearing_rad)) <= radians(2.0)) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            _state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void ModeRTL::descent_start()
{
    _state = SubMode::FINAL_DESCENT;
    _state_complete = false;

    // initialise altitude target to stopping point
    pos_control->init_U_controller_stopping_point();

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif
}

/**
 * @brief Execute final descent stage from RTL_ALT to RTL_ALT_FINAL
 * 
 * @details Implements the FINAL_DESCENT stage of RTL, descending the vehicle from the return
 *          altitude (RTL_ALT) to the final altitude (RTL_ALT_FINAL) before initiating the
 *          final landing sequence. This stage provides a controlled, slower descent phase
 *          compared to the aggressive landing descent, and allows pilot intervention if needed.
 *          
 *          Key Behaviors:
 *          
 *          1. Controlled Descent: Descends at configured descent rate to RTL_ALT_FINAL
 *             (typically 0 to land immediately, or some height above ground for staged landing)
 *          
 *          2. Pilot Override Options:
 *             - Throttle Cancel: If THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND is set and pilot
 *               applies high throttle (>700), exits RTL to LOITER or ALT_HOLD mode
 *             - Horizontal Repositioning: If LAND_REPOSITION=1, pilot can use roll/pitch
 *               to reposition the vehicle horizontally during descent. This is useful for
 *               fine-tuning landing position when landing zone is slightly offset.
 *          
 *          3. Position Holding: Maintains horizontal position over landing point unless
 *             pilot provides repositioning input
 *          
 *          Control Flow:
 *          1. Safety checks: Disarmed/landed handling and pilot override detection
 *          2. Process pilot input:
 *             a. Check for throttle-based landing cancellation
 *             b. Apply horizontal repositioning if enabled and pilot provides input
 *          3. Set motor spool state to THROTTLE_UNLIMITED
 *          4. Update horizontal position controller with pilot velocity corrections
 *          5. Set vertical target to rtl_path.descent_target.alt with slew limiting
 *          6. Update vertical position controller
 *          7. Update attitude controller with thrust vector and yaw
 *          8. Check completion: Within 20cm of target altitude
 *          
 *          Landing Gear:
 *          Landing gear deployment is initiated at descent_start() if the vehicle has
 *          deployable landing gear (AP_LANDINGGEAR_ENABLED).
 *          
 *          Completion Criteria:
 *          Stage is complete when vehicle altitude is within 20cm of rtl_path.descent_target.alt.
 *          This relatively tight tolerance ensures vehicle is at correct altitude before
 *          transitioning to LAND stage.
 * 
 * @note Called at main loop rate (typically 400Hz) when in FINAL_DESCENT state
 * @note RTL_ALT_FINAL=0 causes this stage to be skipped (goes directly to LAND from loiter)
 * @note Pilot can cancel landing with high throttle if THR_BEHAVE configured appropriately
 * @note Horizontal repositioning requires LAND_REPOSITION=1 parameter
 * @note Completion tolerance is 20cm (hard-coded, not configurable)
 * 
 * @warning High throttle landing cancellation is a safety feature but may be unexpected
 *          behavior during autonomous operations. Parameter THR_BEHAVE controls this.
 * @warning Horizontal repositioning during descent can cause vehicle to drift from intended
 *          landing point if pilot input is excessive or sustained
 * @warning The 20cm completion tolerance must be achievable by the altitude controller,
 *          or vehicle may never transition to LAND stage
 * 
 * @see ModeRTL::descent_start() for initialization of this stage
 * @see Parameters: RTL_ALT_FINAL, LAND_REPOSITION, THR_BEHAVE
 * @see LogEvent::LAND_CANCELLED_BY_PILOT for logging when pilot cancels
 * @see LogEvent::LAND_REPO_ACTIVE for logging when pilot repositions during descent
 */
void ModeRTL::descent_run()
{
    Vector2f vel_correction;

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's input
    if (rc().has_valid_input()) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            LOGGER_WRITE_EVENT(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!copter.set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to reposition velocity
            vel_correction = get_pilot_desired_velocity(wp_nav->get_wp_acceleration_cmss() * 0.5);

            // record if pilot has overridden roll or pitch
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    LOGGER_WRITE_EVENT(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector2f accel;
    pos_control->input_vel_accel_NE_cm(vel_correction, accel);
    pos_control->update_NE_controller();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->set_alt_target_with_slew_cm(rtl_path.descent_target.alt);
    pos_control->update_U_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if we've reached within 20cm of final altitude
    _state_complete = labs(rtl_path.descent_target.alt - pos_control->get_pos_estimate_NEU_cm().z) < 20;
}

// land_start - initialise controllers to loiter over home
void ModeRTL::land_start()
{
    _state = SubMode::LAND;
    _state_complete = false;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->set_correction_speed_accel_NE_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialise loiter target destination
    if (!pos_control->is_active_NE()) {
        pos_control->init_NE_controller();
    }

    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

#if AP_LANDINGGEAR_ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif
}

bool ModeRTL::is_landing() const
{
    return _state == SubMode::LAND;
}

// land_run - run the landing controllers to put the aircraft on the ground
// called by rtl_run at 100hz or more
void ModeRTL::land_run(bool disarm_on_land)
{
    // check if we've completed this stage of RTL
    _state_complete = copter.ap.land_complete;

    // disarm when the landing detector says we've landed
    if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

/**
 * @brief Construct the complete RTL flight path from current position to landing
 * 
 * @details Builds the multi-stage RTL path including origin, climb target, return target,
 *          and descent target. This function is called once at RTL initialization (STARTING
 *          state) and computes all waypoints for the entire RTL sequence.
 *          
 *          Path Construction:
 *          1. origin_point: Vehicle's stopping point (predicted position when vehicle stops
 *             from current velocity), converted to altitude-above-home frame
 *          2. return_target: Home or nearest rally point with altitude set to RTL_ALT
 *             (computed by compute_return_target() with terrain/cone logic)
 *          3. climb_target: Directly above origin_point at return_target altitude
 *             (vehicle climbs vertically to this waypoint)
 *          4. descent_target: At return_target horizontal position, at RTL_ALT_FINAL altitude
 *             (converted to above-origin frame for position controller)
 *          
 *          The path forms a vertical-climb, horizontal-return, vertical-descent profile
 *          that ensures obstacle clearance during the return journey.
 *          
 *          Special Cases:
 *          - If RTL_ALT_FINAL <= 0: land flag is set true, causing direct transition from
 *            loiter to land, skipping FINAL_DESCENT stage
 *          - If close to home and RTL_CONE_SLOPE set: climb_target altitude may be reduced
 *            for gradual climb (cone algorithm in compute_return_target())
 *          - If terrain following enabled: altitudes may be terrain-relative rather than
 *            home-relative
 * 
 * @note Called once per RTL invocation during STARTING state transition
 * @note All waypoints stored in rtl_path structure for use by subsequent stages
 * @note Altitude frame conversions ensure position controller receives correct references
 * 
 * @see ModeRTL::compute_return_target() for return target altitude calculation
 * @see ModeRTL::get_stopping_point() for origin point prediction
 */
void ModeRTL::build_path()
{
    // origin point is our stopping point
    rtl_path.origin_point = get_stopping_point();
    rtl_path.origin_point.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // compute return target
    compute_return_target();

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target = Location(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location::AltFrame::ABOVE_HOME);

    // Target altitude is passed directly to the position controller so must be relative to origin
    rtl_path.descent_target.change_alt_frame(Location::AltFrame::ABOVE_ORIGIN);

    // set land flag - if RTL_ALT_FINAL is zero or negative, land immediately after loiter
    rtl_path.land = g.rtl_alt_final <= 0;
}

/**
 * @brief Compute the return target position and safe return altitude
 * 
 * @details This is one of the most complex functions in RTL mode, responsible for computing
 *          the target position (home or rally point) and determining a safe altitude for the
 *          return journey. The altitude calculation considers multiple factors:
 *          
 *          1. Rally Point Selection: If enabled, selects nearest rally point, otherwise uses home
 *          2. Altitude Type Selection: Determines if altitude should be relative to home or terrain
 *          3. Current Altitude Assessment: Determines current altitude in the chosen frame
 *          4. Minimum Safe Altitude: Ensures altitude is at least RTL_ALT, current_alt + RTL_CLIMB_MIN
 *          5. Cone Slope Reduction: If close to home, may reduce climb using RTL_CONE_SLOPE
 *          6. Fence Altitude Check: Ensures return altitude doesn't violate altitude fence
 *          7. Descent Prevention: Ensures vehicle never descends below current altitude
 *          
 *          Altitude Type Selection (RTL_ALT_TYPE and terrain following):
 *          - RELATIVE (0): Altitude above home position (default, always available)
 *          - TERRAIN (1): Altitude above terrain using rangefinder or terrain database
 *            * Requires terrain data availability or valid rangefinder
 *            * Falls back to RELATIVE if terrain data unavailable
 *            * Critical for operations in mountainous terrain
 *          
 *          Cone Slope Algorithm:
 *          If RTL_CONE_SLOPE is set (default 3.0), and vehicle is close to home, the return
 *          altitude may be reduced to create a gradual climb rather than climbing to full
 *          RTL_ALT before returning. This saves time and battery when already near home.
 *          Formula: target_alt = MIN(target_alt, horizontal_distance * RTL_CONE_SLOPE)
 *          Example: At 10m from home with slope=3.0, climb to 30m max instead of RTL_ALT
 *          
 *          RTL_CLIMB_MIN ensures minimum climb regardless of current altitude. For example,
 *          RTL_CLIMB_MIN=200cm ensures vehicle climbs at least 2m even if already at or above
 *          RTL_ALT. This provides obstacle clearance margin.
 *          
 *          Terrain Following Fallbacks:
 *          If terrain mode selected but data unavailable, function automatically falls back
 *          to altitude-above-home mode and logs error. This ensures RTL continues safely
 *          even with terrain data loss, though at reduced safety margin in mountainous terrain.
 * 
 * @note Called by build_path() during RTL initialization
 * @note Modifies rtl_path.return_target with both position and altitude
 * @note Altitude frame of return_target may be ABOVE_HOME or ABOVE_TERRAIN depending on mode
 * @note Complex function with multiple conditional paths based on configuration
 * 
 * @warning Terrain data loss during this computation will cause fallback to home-relative
 *          altitude, which may be unsafe in mountainous terrain if RTL_ALT is insufficient
 * @warning Altitude fence (if enabled) can reduce return altitude, potentially causing
 *          insufficient climb for obstacle clearance. Configure fence appropriately.
 * @warning Rally point altitudes above terrain must be set carefully - function respects
 *          rally point altitude which may be higher than RTL_ALT (e.g., for building top landing)
 * 
 * @see Parameters: RTL_ALT, RTL_ALT_TYPE, RTL_CLIMB_MIN, RTL_CONE_SLOPE
 * @see AP_Rally::calc_best_rally_or_home_location() for rally point selection
 * @see AC_WPNav::get_terrain_source() for terrain data source determination
 * @see AC_Fence for altitude fence implementation
 */
void ModeRTL::compute_return_target()
{
    // set return target to nearest rally point or home position
#if HAL_RALLY_ENABLED
    rtl_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
    rtl_path.return_target.change_alt_frame(Location::AltFrame::ABSOLUTE);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // get position controller Z-axis offset in cm above EKF origin
    // This offset accounts for position controller frame differences and must be subtracted
    // from altitude measurements to get consistent altitude reference
    int32_t pos_offset_z = pos_control->get_pos_offset_U_cm();

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    // Initially computed as altitude minus position controller offset
    int32_t curr_alt = copter.current_loc.alt - pos_offset_z;

    // determine altitude type of return journey (alt-above-home, alt-above-terrain using range finder or alt-above-terrain using terrain database)
    // This determines the reference frame for all subsequent altitude calculations
    ReturnTargetAltType alt_type = ReturnTargetAltType::RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == RTLAltType::TERRAIN)) {
        // Terrain following requested via RTL_ALT_TYPE=1 and not disabled by terrain failsafe
        // Convert RTL_ALT_TYPE and WPNAV_RFNG_USE parameters to specific terrain source type
        // This determines whether we use rangefinder or terrain database for terrain-relative altitude
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            // Terrain data not available - fall back to altitude-above-home for safety
            // This can occur if terrain database not loaded or rangefinder unhealthy
            alt_type = ReturnTargetAltType::RELATIVE;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            // Use rangefinder for terrain-relative altitude (good for low-altitude operations)
            alt_type = ReturnTargetAltType::RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            // Use terrain database for terrain-relative altitude (better for high-altitude/long-range)
            alt_type = ReturnTargetAltType::TERRAINDATABASE;
            break;
        }
    }

    // set curr_alt and return_target.alt from range finder
    if (alt_type == ReturnTargetAltType::RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_cm(curr_alt)) {
            // subtract position controller offset
            curr_alt -= pos_offset_z;
            // set return_target.alt
            rtl_path.return_target.set_alt_cm(MAX(curr_alt + MAX(0, g.rtl_climb_min), MAX(g.rtl_altitude, RTL_ALT_MIN)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            LOGGER_WRITE_ERROR(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
        }
    }

    // set curr_alt and return_target.alt from terrain database
    if (alt_type == ReturnTargetAltType::TERRAINDATABASE) {
        // set curr_alt to current altitude above terrain
        // convert return_target.alt from an abs (above MSL) to altitude above terrain
        //   Note: the return_target may be a rally point with the alt set above the terrain alt (like the top of a building)
        int32_t curr_terr_alt;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt) &&
            rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            curr_alt = curr_terr_alt - pos_offset_z;
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            LOGGER_WRITE_ERROR(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // for the default case we must convert return-target alt (which is an absolute alt) to alt-above-home
    if (alt_type == ReturnTargetAltType::RELATIVE) {
        if (!rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_path.return_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
            gcs().send_text(MAV_SEVERITY_WARNING, "RTL: unexpected error calculating target alt");
        }
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon rtl_alt_type
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    int32_t target_alt = MAX(rtl_path.return_target.alt, 0);

    // increase target to maximum of current altitude + climb_min and rtl altitude
    // This ensures vehicle climbs at least RTL_CLIMB_MIN above current position, and reaches at least RTL_ALT
    // min_rtl_alt is the absolute minimum altitude considering current position and minimum climb
    const float min_rtl_alt = MAX(RTL_ALT_MIN, curr_alt + MAX(0, g.rtl_climb_min));
    target_alt = MAX(target_alt, MAX(g.rtl_altitude, min_rtl_alt));

    // reduce climb if close to return target (cone slope algorithm)
    // If vehicle is near home, don't climb all the way to RTL_ALT - use proportional climb based on distance
    // This saves time and battery when close to home, while still providing obstacle clearance
    float rtl_return_dist_cm = rtl_path.return_target.get_distance(rtl_path.origin_point) * 100.0f;
    // don't allow really shallow slopes (must be at least RTL_MIN_CONE_SLOPE)
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        // Apply cone: altitude = distance * slope, but never less than min_rtl_alt
        // Example: 10m away with slope 3.0 = climb to 30m (if less than RTL_ALT)
        target_alt = MIN(target_alt, MAX(rtl_return_dist_cm * g.rtl_cone_slope, min_rtl_alt));
    }

    // set returned target alt to new target_alt (don't change altitude type)
    rtl_path.return_target.set_alt_cm(target_alt, (alt_type == ReturnTargetAltType::RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AP_FENCE_ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // This prevents RTL from violating airspace restrictions, but may reduce safety margin for obstacle clearance
    // Note: because the rtl_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target as alt-above-home so it can be compared to fence's alt
        // Fence altitude is always in home-relative frame
        if (rtl_path.return_target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt)) {
            float fence_alt = copter.fence.get_safe_alt_max()*100.0f;
            if (target_alt > fence_alt) {
                // reduce target alt to the fence alt to prevent fence breach
                // WARNING: This may reduce altitude below what's needed for obstacle clearance
                rtl_path.return_target.alt -= (target_alt - fence_alt);
            }
        }
    }
#endif

    // ensure we do not descend during RTL - return altitude must be at or above current altitude
    // This is a critical safety check to prevent descending into obstacles during return journey
    rtl_path.return_target.alt = MAX(rtl_path.return_target.alt, curr_alt);
}

bool ModeRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (_state) {
    case SubMode::STARTING:
    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
    case SubMode::LOITER_AT_HOME:
    case SubMode::FINAL_DESCENT:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

float ModeRTL::wp_distance_m() const
{
    return wp_nav->get_wp_distance_to_destination_cm() * 0.01f;
}

float ModeRTL::wp_bearing_deg() const
{
    return degrees(wp_nav->get_wp_bearing_to_destination_rad());
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeRTL::use_pilot_yaw(void) const
{
    const bool allow_yaw_option = (copter.g2.rtl_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
    const bool land_repositioning = g.land_repositioning && (_state == SubMode::FINAL_DESCENT);
    const bool final_landing = _state == SubMode::LAND;
    return allow_yaw_option || land_repositioning || final_landing;
}

bool ModeRTL::set_speed_xy_cms(float speed_xy_cms)
{
    copter.wp_nav->set_speed_NE_cms(speed_xy_cms);
    return true;
}

bool ModeRTL::set_speed_up_cms(float speed_up_cms)
{
    copter.wp_nav->set_speed_up_cms(speed_up_cms);
    return true;
}

bool ModeRTL::set_speed_down_cms(float speed_down_cms)
{
    copter.wp_nav->set_speed_down_cms(speed_down_cms);
    return true;
}

#endif
