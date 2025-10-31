/**
 * @file mode_zigzag.cpp
 * @brief ZigZag flight mode implementation for ArduCopter
 * 
 * @details ZigZag mode is designed for automated aerial surveying tasks such as 
 *          crop spraying and photogrammetry. The pilot manually flies to mark two 
 *          corner points (A and B), and the vehicle then executes an automated 
 *          back-and-forth pattern between these points with configurable line spacing.
 * 
 *          Key Features:
 *          - Manual A/B point recording via auxiliary switch
 *          - Automated survey pattern execution with configurable line spacing
 *          - Configurable sideways movement direction (forward/right/backward/left)
 *          - Manual override capability during pattern execution
 *          - Optional sprayer integration for agricultural applications
 *          - Configurable waypoint delays and number of lines
 * 
 *          Typical Workflow:
 *          1. Enter ZigZag mode
 *          2. Fly to first corner and store point A (via auxiliary switch)
 *          3. Fly to second corner and store point B (via auxiliary switch)
 *          4. Vehicle is ready for automated pattern or manual flight between A/B
 *          5. Trigger automated pattern execution (flies A→B, moves sideways, B→A, repeat)
 * 
 *          The mode maintains manual control authority for altitude, yaw, and can be
 *          interrupted to return to manual positioning control at any time.
 * 
 * @note This mode requires GPS position lock and is typically called at 100Hz
 * @warning Ensure adequate safety margins around survey area boundaries
 * 
 * Source: ArduCopter/mode_zigzag.cpp
 */

#include "Copter.h"

#if MODE_ZIGZAG_ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_CM 300
#define ZIGZAG_LINE_INFINITY -1

const AP_Param::GroupInfo ModeZigZag::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: ZigZag auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeZigZag, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if HAL_SPRAYER_ENABLED
    // @Param: SPRAYER
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPRAYER", 2, ModeZigZag, _spray_enabled, 0),
#endif // HAL_SPRAYER_ENABLED

    // @Param: WP_DELAY
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("WP_DELAY", 3, ModeZigZag, _wp_delay_s, 0),

    // @Param: SIDE_DIST
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("SIDE_DIST", 4, ModeZigZag, _side_dist_m, 4),

    // @Param: DIRECTION
    // @DisplayName: Sideways direction in ZigZag auto
    // @Description: The direction to move sideways in ZigZag mode
    // @Values: 0:forward, 1:right, 2:backward, 3:left
    // @User: Advanced
    AP_GROUPINFO("DIRECTION", 5, ModeZigZag, _direction, 0),

    // @Param: LINE_NUM
    // @DisplayName: Total number of lines
    // @Description: Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways
    // @Range: -1 32767
    // @User: Advanced
    AP_GROUPINFO("LINE_NUM", 6, ModeZigZag, _line_num, 0),

    AP_GROUPEND
};

ModeZigZag::ModeZigZag(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Initialize ZigZag flight mode controller
 * 
 * @details This function initializes the ZigZag mode when the pilot switches into it.
 *          It sets up the loiter controller for manual positioning, configures altitude
 *          control parameters, and resets the waypoint state to prepare for A/B point
 *          recording. The vehicle starts in STORING_POINTS stage, waiting for the pilot
 *          to mark corner positions A and B.
 * 
 *          Initialization sequence:
 *          1. Apply simple mode transform if enabled (converts pilot inputs to earth frame)
 *          2. Process current pilot stick inputs to establish initial target attitude
 *          3. Initialize loiter navigation target at current position
 *          4. Configure vertical speed/acceleration limits from pilot parameters
 *          5. Initialize vertical position controller if not already active
 *          6. Reset waypoint state machine to STORING_POINTS stage
 *          7. Clear any previously stored A/B destination points
 *          8. Initialize automated pattern execution state
 * 
 * @param[in] ignore_checks  When true, bypasses pre-arm checks (currently unused in ZigZag)
 * 
 * @return true  Always returns true - ZigZag mode initialization cannot fail
 * 
 * @note Called once when entering ZigZag mode from another flight mode
 * @note Pilot retains full manual control of position and altitude after initialization
 * @note Vehicle must have GPS lock for ZigZag mode to function properly
 * 
 * @see run() for main execution loop
 * @see save_or_move_to_destination() for A/B point recording
 */
bool ModeZigZag::init(bool ignore_checks)
{
    // apply simple mode transform to pilot inputs
    // Simple mode rotates pilot inputs from vehicle frame to earth frame
    // for easier control when vehicle is pointed away from pilot
    update_simple_mode();

    // convert pilot input to lean angles
    // Get current stick positions and convert to target roll/pitch angles (radians)
    // respecting configured angle limits for loiter and altitude hold modes
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    // Initialize loiter navigation with pilot's current input to prevent sudden movements
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // Initialize loiter target position at current vehicle location
    // This establishes the initial hover point for the mode
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    // Configure maximum climb/descent rates and acceleration based on pilot parameters
    // Parameters: max_speed_down (cm/s), max_speed_up (cm/s), max_accel_z (cm/s/s)
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_U_cmss(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    // Start altitude controller if not already running (e.g., from a previous alt-hold mode)
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise waypoint state
    // Start in STORING_POINTS stage - waiting for pilot to record corner points A and B
    stage = STORING_POINTS;
    
    // Clear any previously stored corner point positions
    // Destinations are stored as offsets from EKF origin in NE plane (centimeters)
    dest_A_ne_cm.zero();
    dest_B_ne_cm.zero();

    // initialize zigzag auto
    // Reset automated pattern execution state (line count, auto stage, suspension flags)
    init_auto();

    return true;
}

// perform cleanup required when leaving zigzag mode
void ModeZigZag::exit()
{
    // The sprayer will stop if the flight mode is changed from ZigZag to other
    spray(false);
}

/**
 * @brief Main execution loop for ZigZag flight mode
 * 
 * @details This function is called at 100Hz (every 10ms) and implements the ZigZag mode
 *          state machine. It handles three primary operational stages:
 * 
 *          1. STORING_POINTS: Pilot manually positions vehicle to record A/B corner points
 *          2. MANUAL_REGAIN: Both A/B points stored, pilot has manual control, can trigger auto
 *          3. AUTO: Vehicle executing automated survey pattern between A and B points
 * 
 *          Automated Survey Pattern Behavior:
 *          - Flies from current position to A (or B), perpendicular to AB line
 *          - Moves sideways by configured distance in configured direction
 *          - Flies to opposite point (B or A)
 *          - Repeats until configured line count reached or manually stopped
 *          - Optional sprayer control: ON during A↔B flight, OFF during sideways movement
 * 
 *          Safety Features:
 *          - Automatically reverts to manual control if disarmed or landed
 *          - Automatically reverts to manual control if motor interlock disabled
 *          - Pilot can interrupt automated pattern at any time
 *          - Respects configured waypoint delays at each corner
 * 
 * @note Called at 100Hz from main scheduler - timing critical for smooth control
 * @note Altitude control is always manual (pilot throttle stick) in all stages
 * @note Yaw control is always manual (pilot yaw stick) in all stages
 * @note Position control is automatic in AUTO stage, manual in other stages
 * 
 * @warning Vehicle must maintain GPS lock throughout operation
 * 
 * @see manual_control() for manual flight control implementation
 * @see auto_control() for automated waypoint following control
 * @see save_or_move_to_destination() for A/B point recording and movement initiation
 */
void ModeZigZag::run()
{
    // set vertical speed and acceleration limits
    // Update altitude control parameters each iteration in case pilot params changed
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // set the direction and the total number of lines
    // Read and constrain parameters for automated pattern execution
    // Direction: 0=forward, 1=right, 2=backward, 3=left (relative to initial vehicle heading)
    // Line count: -1=infinite, 0=sideways only, 1-32767=specific number of passes
    zigzag_direction = (Direction)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, 32767);

    // auto control
    // Handle automated survey pattern execution stage
    if (stage == AUTO) {
        // Safety check: return to manual control if vehicle is disarmed, landed, or interlock disabled
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // vehicle should be under manual control when disarmed or landed
            return_to_manual_control(false);
        } else if (reached_destination()) {
            // Vehicle has reached current waypoint (A, B, or sideways position)
            // Notify GCS and trigger waypoint complete event for telemetry/logging
            AP_Notify::events.waypoint_complete = 1;
            
            if (is_auto) {
                // Automated pattern execution mode - determine next action
                if (line_num == ZIGZAG_LINE_INFINITY || line_count < line_num) {
                    // Continue pattern: more lines to fly
                    if (auto_stage == AutoState::SIDEWAYS) {
                        // Just completed sideways movement, now fly to opposite corner
                        // If we stored destination A last, move to B; if B, move to A
                        save_or_move_to_destination((ab_dest_stored == Destination::A) ? Destination::B : Destination::A);
                    } else {
                        // Just completed A→B or B→A flight, now move sideways for next line
                        // Turn off sprayer during sideways movement
                        spray(false);
                        move_to_side();
                    }
                } else {
                    // Pattern complete: reached configured line count
                    // Reset auto state and return to manual control
                    init_auto();
                    return_to_manual_control(true);
                }
            } else {
                // Single A/B movement complete (not in automated pattern mode)
                // Return to manual control at destination
                return_to_manual_control(true);
            }
        } else {
            // Vehicle has not yet reached destination - continue automated flight
            auto_control();
        }
    }

    // manual control
    // Handle manual flight stages: initial A/B point recording or manual positioning between pattern runs
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        // Pilot has full control of horizontal position, altitude, and yaw
        manual_control();
    }
}

/**
 * @brief Save current position as corner point A or B, or initiate movement to specified corner
 * 
 * @details This dual-purpose function handles both A/B point recording and automated movement
 *          initiation. Its behavior depends on the current stage of ZigZag mode operation:
 * 
 *          STORING_POINTS Stage (Initial Setup):
 *          - Stores current vehicle position as corner point A or B
 *          - Sends confirmation message to GCS and logs event
 *          - Enables sprayer after first point stored (if configured)
 *          - Advances to MANUAL_REGAIN stage when both A and B are stored
 *          - Validates A and B are different positions (non-zero separation)
 * 
 *          MANUAL_REGAIN/AUTO Stages (Movement Initiation):
 *          - Calculates perpendicular approach path to specified corner
 *          - Projects current position onto line perpendicular to AB at destination
 *          - Initializes waypoint navigation to destination
 *          - Transitions to AUTO stage for automated flight
 *          - Enables sprayer during A↔B movement (if configured)
 *          - Tracks line count for automated pattern progress
 * 
 *          Typical Usage Sequence:
 *          1. Pilot triggers with ab_dest=A → stores point A at current location
 *          2. Pilot flies to new location and triggers with ab_dest=B → stores point B
 *          3. Mode advances to MANUAL_REGAIN, ready for automated pattern
 *          4. Automated pattern calls with ab_dest=A or B → initiates flight to corner
 * 
 * @param[in] ab_dest  Specifies whether to save/move to destination A or B
 * 
 * @note Positions are stored as NE offsets from EKF origin in centimeters
 * @note Called via auxiliary switch during manual setup or by automated pattern execution
 * @note Perpendicular approach ensures smooth entry into survey lines from any position
 * @note Sprayer control: ON during A↔B, OFF after both points stored or during manual control
 * 
 * @warning Ensure A and B points are sufficiently separated (>0.1m) for valid survey pattern
 * 
 * @see calculate_next_dest() for perpendicular projection algorithm
 * @see run() for automated pattern execution that calls this function
 */
void ModeZigZag::save_or_move_to_destination(Destination ab_dest)
{
    // get current position as an offset from EKF origin
    // Position is in NE plane (North-East), altitude handled separately
    // Units: centimeters from EKF origin
    const Vector2f curr_pos_neu_cm = pos_control->get_pos_desired_NEU_cm().xy().tofloat();

    // handle state machine changes
    // Function behavior changes based on whether we're recording points or initiating movement
    switch (stage) {

        case STORING_POINTS:
            // Initial setup phase: record corner point positions A and B
            if (ab_dest == Destination::A) {
                // store point A
                // Record current position as first corner of survey area
                dest_A_ne_cm = curr_pos_neu_cm;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point A stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_A);
            } else {
                // store point B
                // Record current position as second corner of survey area
                dest_B_ne_cm = curr_pos_neu_cm;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: point B stored", name());
                LOGGER_WRITE_EVENT(LogEvent::ZIGZAG_STORE_B);
            }
            
            // if both A and B have been stored advance state
            // Validate that both points exist and are separated (non-zero distance)
            // This check prevents degenerate survey patterns from invalid point pairs
            if (!dest_A_ne_cm.is_zero() && !dest_B_ne_cm.is_zero() && !is_zero((dest_B_ne_cm - dest_A_ne_cm).length_squared())) {
                // Both corners defined with valid separation - ready for automated pattern
                stage = MANUAL_REGAIN;
                spray(false);  // Turn off sprayer in manual control stage
            } else if (!dest_A_ne_cm.is_zero() || !dest_B_ne_cm.is_zero()) {
                // if only A or B have been stored, spray on
                // Enable sprayer while pilot flies to mark second point (if configured)
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            // Calculate target position and initiate automated waypoint navigation
            Vector3f next_dest;   // Target position in NEU frame (centimeters)
            bool terr_alt_cm;     // True if altitude is terrain-relative
            
            // Calculate perpendicular approach to destination A or B from current position
            // This ensures smooth survey line entry regardless of vehicle's current location
            if (calculate_next_dest(ab_dest, stage == AUTO, next_dest, terr_alt_cm)) {
                // Initialize waypoint navigation system for new destination
                wp_nav->wp_and_spline_init_cm();
                
                // Set destination and initiate automated flight
                if (wp_nav->set_wp_destination_NEU_cm(next_dest, terr_alt_cm)) {
                    // Successfully set waypoint - transition to automated control
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;  // Track that we're flying A↔B (not sideways)
                    ab_dest_stored = ab_dest;           // Remember which corner we're moving to
                    
                    // spray on while moving to A or B
                    // Enable sprayer during survey line flight (if configured)
                    spray(true);
                    
                    // Reset waypoint arrival timer for delay calculation
                    reach_wp_time_ms = 0;
                    
                    // Send telemetry with appropriate line count information
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        // Single line movement or infinite pattern - don't show line count
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), (ab_dest == Destination::A) ? "A" : "B");
                    } else {
                        // Finite automated pattern - show progress
                        line_count++;  // Increment line counter for pattern tracking
                        gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s (line %d/%d)", name(), (ab_dest == Destination::A) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

/**
 * @brief Initiate sideways movement to next survey line
 * 
 * @details This function calculates and initiates movement perpendicular to the A-B line
 *          to position the vehicle for the next survey pass. This is the key function that
 *          creates the "zigzag" survey pattern by spacing successive A-B passes.
 * 
 *          Sideways Movement Algorithm:
 *          1. Calculate vector perpendicular to A-B line
 *          2. Determine direction based on vehicle heading and configured direction parameter
 *          3. Scale perpendicular vector to configured line spacing distance
 *          4. Set waypoint at current position + perpendicular offset
 * 
 *          Direction Interpretation (relative to vehicle heading when A-B line established):
 *          - FORWARD (0): Move in direction vehicle was pointing
 *          - RIGHT (1): Move to vehicle's right
 *          - BACKWARD (2): Move opposite to direction vehicle was pointing
 *          - LEFT (3): Move to vehicle's left
 * 
 *          This creates a survey pattern where:
 *          - Vehicle flies A→B (survey line with sprayer on)
 *          - Vehicle moves sideways by configured distance (sprayer off)
 *          - Vehicle flies B→A (survey line with sprayer on)
 *          - Vehicle moves sideways again (sprayer off)
 *          - Pattern repeats for configured number of lines
 * 
 * @note Only executes if valid A and B points exist with non-zero separation
 * @note Sideways distance configured via ZIGZAG_SIDE_DIST parameter (0.1-100m)
 * @note Called during automated pattern execution after completing A→B or B→A flight
 * @note Sprayer is turned off during sideways movement (handled by calling code)
 * 
 * @see calculate_side_dest() for perpendicular vector calculation algorithm
 * @see run() for automated pattern state machine that calls this function
 */
void ModeZigZag::move_to_side()
{
    // Validate that both corner points A and B are stored with non-zero separation
    // Required for valid perpendicular vector calculation
    if (!dest_A_ne_cm.is_zero() && !dest_B_ne_cm.is_zero() && !is_zero((dest_B_ne_cm - dest_A_ne_cm).length_squared())) {
        Vector3f next_dest;   // Target position for sideways movement (NEU frame, cm)
        bool terr_alt;        // True if altitude is terrain-relative
        
        // Calculate perpendicular destination at configured spacing distance
        // Direction determined by vehicle heading and ZIGZAG_DIRECTION parameter
        if (calculate_side_dest(next_dest, terr_alt)) {
            // Initialize waypoint navigation system for new destination
            wp_nav->wp_and_spline_init_cm();
            
            // Set sideways destination and initiate automated flight
            if (wp_nav->set_wp_destination_NEU_cm(next_dest, terr_alt)) {
                // Successfully set waypoint - transition to automated control
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;  // Track that we're moving sideways (not A↔B)
                
                // Store destination for potential pattern resume after interruption
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                
                // Reset waypoint arrival timer for delay calculation
                reach_wp_time_ms = 0;
                
                // Send telemetry message indicating sideways movement direction
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    }
}

// return manual control to the pilot
void ModeZigZag::return_to_manual_control(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        spray(false);
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f& wp_dest = wp_nav->get_wp_destination_NEU_cm();
            loiter_nav->init_target_cm(wp_dest.xy());
#if AP_RANGEFINDER_ENABLED
            if (copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy()) {
                copter.surface_tracking.external_init();
            }
#endif
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: manual control", name());
    }
}

/**
 * @brief Execute automated waypoint navigation during survey pattern
 * 
 * @details This function implements automated flight control during the AUTO stage of
 *          ZigZag mode. It uses the waypoint navigation controller to fly the vehicle
 *          to destinations (A, B, or sideways positions) while maintaining pilot yaw
 *          control authority.
 * 
 *          Control Authority Distribution:
 *          - Roll/Pitch: Controlled by waypoint navigation controller (automated)
 *          - Altitude: Controlled by waypoint navigation controller (automated)
 *          - Yaw: Controlled by pilot input (manual override capability)
 * 
 *          This mixed control approach allows the pilot to orient the vehicle (e.g., for
 *          camera pointing) while the automated system handles navigation. This is useful
 *          for photogrammetry where camera orientation matters but flight path is automated.
 * 
 *          Safety Features:
 *          - Automatically reverts to manual control if waypoint navigation fails
 *          - Waypoint navigation failure typically indicates terrain data unavailability
 *          - Maintains full motor authority for responsive automated flight
 * 
 * @note Called at 100Hz during AUTO stage (automated waypoint following)
 * @note Pilot retains yaw control even during automated flight
 * @note Waypoint navigation handles obstacle avoidance and terrain following if configured
 * 
 * @warning If terrain following enabled, ensure terrain database is loaded for area
 * 
 * @see run() for mode state machine that calls this function
 * @see manual_control() for manual flight control implementation
 */
void ModeZigZag::auto_control()
{
    // process pilot's yaw input
    // Allow pilot to control vehicle heading even during automated navigation
    // Useful for pointing cameras/sensors during survey patterns
    const float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // set motors to full range
    // Enable full throttle authority for responsive waypoint following
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // Update waypoint navigation: calculates desired roll/pitch/altitude to reach destination
    // Returns false if navigation fails (typically due to missing terrain data)
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    // Execute altitude control to track waypoint navigation's altitude target
    pos_control->update_U_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    // This mixed control allows automated navigation with manual heading control
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(wp_nav->get_roll_rad(), wp_nav->get_pitch_rad(), target_yaw_rate_rads);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    // Safety feature: return control to pilot if automated navigation cannot proceed
    // Common failure cause: terrain following enabled but terrain data unavailable
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

/**
 * @brief Process manual flight control for ZigZag mode
 * 
 * @details This function implements manual control during STORING_POINTS and MANUAL_REGAIN
 *          stages of ZigZag mode. It provides the pilot with full control over horizontal
 *          position (via roll/pitch sticks), altitude (via throttle stick), and yaw (via
 *          yaw stick), similar to Loiter mode behavior.
 * 
 *          Manual control is used in two scenarios:
 *          1. STORING_POINTS: Pilot flies to mark corner points A and B
 *          2. MANUAL_REGAIN: Both corners stored, pilot controls positioning between
 *             automated pattern runs or after manual pattern interruption
 * 
 *          Control Implementation:
 *          - Roll/Pitch: Converted to target lean angles, processed by loiter controller
 *                       for position hold with pilot input overlay
 *          - Throttle: Converted to climb rate, processed by altitude hold state machine
 *          - Yaw: Pilot yaw rate command passed directly to attitude controller
 * 
 *          Altitude State Machine (same as AltHold mode):
 *          - MotorStopped: Motors off, controllers reset, zero throttle output
 *          - Landed_Pre_Takeoff: On ground, motors spooling, preparing for takeoff
 *          - Landed_Ground_Idle: Landed with motors at idle
 *          - Takeoff: Executing automated takeoff sequence to configured altitude
 *          - Flying: Normal flight, full pilot control with position hold
 * 
 *          Special Features:
 *          - Simple mode support: Rotates pilot inputs from vehicle to earth frame
 *          - Avoidance integration: Adjusts climb rate for obstacle avoidance (if enabled)
 *          - Surface tracking: Maintains height above terrain using rangefinder (if enabled)
 *          - Landing detection: Softens position hold when landing detected
 * 
 * @note Called at 100Hz during manual control stages
 * @note This function does NOT handle automated waypoint following (see auto_control())
 * @note Pilot can seamlessly transition between manual positioning and automated pattern
 * 
 * @warning Radio failsafe will be handled by failsafe system, not this function
 * 
 * @see auto_control() for automated waypoint following control
 * @see run() for mode state machine that calls this function
 */
void ModeZigZag::manual_control()
{
    float target_yaw_rate_rads = 0.0f;    // Desired yaw rotation rate from pilot
    float target_climb_rate_cms = 0.0f;   // Desired climb rate from pilot throttle

    // process pilot inputs unless we are in radio failsafe
    float target_roll_rad, target_pitch_rad;  // Desired lean angles from pilot sticks

    // apply SIMPLE mode transform to pilot inputs
    // Simple mode rotates stick inputs from vehicle body frame to earth frame
    // Makes control easier when vehicle is pointed away from pilot
    update_simple_mode();

    // convert pilot input to lean angles
    // Transform stick positions to target roll/pitch angles (radians)
    // Respects configured maximum lean angles for loiter and altitude hold
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    // Feed pilot's desired lean angles into loiter controller
    // Controller will maintain position when sticks centered, move when sticks deflected
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // get pilot's desired yaw rate
    // Convert yaw stick position to rotation rate (rad/s)
    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // get pilot desired climb rate
    // Convert throttle stick position to vertical velocity (cm/s)
    target_climb_rate_cms = get_pilot_desired_climb_rate();
    
    // make sure the climb rate is in the given range, prevent floating point errors
    // Constrain to configured maximum climb/descent rates
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    // Soften position hold to prevent aggressive corrections on ground
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    // Determine current flight state based on throttle input and vehicle status
    // States: MotorStopped, Landed_Pre_Takeoff, Landed_Ground_Idle, Takeoff, Flying
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate_cms);

    // althold state machine
    // Handle altitude control differently based on flight state
    switch (althold_state) {

    case AltHoldModeState::MotorStopped:
        // Motors off - reset all controllers for clean state
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        // Command zero attitude (level) and pass through pilot yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(loiter_nav->get_roll_rad(), loiter_nav->get_pitch_rad(), target_yaw_rate_rads);
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        // Start automated takeoff sequence if not already running
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        // Modify climb rate if obstacle avoidance system detects obstacles above
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

        // run loiter controller
        // Update horizontal position controller for position hold during takeoff
        loiter_nav->update();

        // call attitude controller
        // Command attitude from loiter controller plus pilot yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(loiter_nav->get_roll_rad(), loiter_nav->get_pitch_rad(), target_yaw_rate_rads);

        // set position controller targets adjusted for pilot input
        // Execute takeoff with pilot climb rate overlay
        takeoff.do_pilot_takeoff(target_climb_rate_cms);
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        // Landed with motors at idle - reset yaw to current heading
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        // On ground preparing for takeoff - smoothly reset controller I terms
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        // Use thrust vector control for smooth motor spool-up
        attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads);
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Flying:
        // Normal flight - full manual control with position hold
        
        // set motors to full range
        // Allow full throttle authority for responsive control
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        // Update horizontal position hold with pilot input overlay
        loiter_nav->update();

        // call attitude controller
        // Command roll/pitch from loiter, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(loiter_nav->get_roll_rad(), loiter_nav->get_pitch_rad(), target_yaw_rate_rads);

        // get avoidance adjusted climb rate
        // Integrate obstacle avoidance to prevent collisions
        target_climb_rate_cms = get_avoidance_adjusted_climbrate_cms(target_climb_rate_cms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        // Maintain constant height above terrain using rangefinder
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        // Convert climb rate to altitude target for position controller
        pos_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    // run the vertical position controller and set output throttle
    // Execute altitude control and generate motor throttle commands
    // Called regardless of state to ensure smooth controller operation
    pos_control->update_U_controller();
}

// return true if vehicle is within a small area around the destination
bool ModeZigZag::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination_cm() > ZIGZAG_WP_RADIUS_CM) {
        return false;
    }

    // wait at time which is set in zigzag_wp_delay
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) >= (uint16_t)constrain_int16(_wp_delay_s, 0, 127) * 1000);
}

/**
 * @brief Calculate perpendicular approach destination to corner A or B
 * 
 * @details This function calculates the optimal destination point for approaching corner A or B
 *          by projecting the vehicle's current position onto a line perpendicular to the A-B
 *          survey line. This ensures the vehicle enters the survey line at a right angle,
 *          creating clean, parallel survey passes regardless of where the vehicle is positioned.
 * 
 *          Geometric Algorithm:
 *          1. Create line perpendicular to A-B passing through destination (A or B)
 *          2. Extend perpendicular line to ensure it reaches vehicle's current position
 *          3. Project vehicle position onto perpendicular line (closest point calculation)
 *          4. Return projected position as next destination with appropriate altitude
 * 
 *          This approach solves several problems:
 *          - Vehicle can be anywhere when pattern starts - will approach cleanly
 *          - After sideways movement, vehicle re-enters survey line perpendicularly
 *          - Creates consistent survey line geometry regardless of vehicle position
 *          - Prevents skewed or curved survey lines from poor approach angles
 * 
 *          Altitude Handling:
 *          - If use_wpnav_alt=true: Maintain current waypoint navigation altitude
 *          - If use_wpnav_alt=false: Use position controller's current altitude target
 *          - Terrain-relative altitude supported if rangefinder available and healthy
 * 
 * @param[in]  ab_dest          Destination corner (A or B) to approach
 * @param[in]  use_wpnav_alt    True to use waypoint controller altitude, false for position controller
 * @param[out] next_dest_neu_cm Calculated destination position in NEU frame (centimeters from EKF origin)
 * @param[out] terrain_alt      Set to true if altitude is terrain-relative, false if absolute
 * 
 * @return true if calculation successful, false if A-B separation is zero (invalid)
 * 
 * @note NEU coordinate frame: North-East-Up from EKF origin
 * @note Perpendicular line is automatically extended if vehicle is far from A-B line
 * @note Called during pattern execution to determine approach paths to corners
 * 
 * @see save_or_move_to_destination() which uses this calculation to initiate movement
 */
bool ModeZigZag::calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest_neu_cm, bool& terrain_alt) const
{
    // define start_pos_ne_cm as either destination A or B
    // This is the corner point we're approaching
    Vector2f start_pos_ne_cm = (ab_dest == Destination::A) ? dest_A_ne_cm : dest_B_ne_cm;

    // calculate vector from A to B
    // This defines the survey line direction
    Vector2f AB_diff_ne_cm = dest_B_ne_cm - dest_A_ne_cm;

    // check distance between A and B
    // Zero separation means invalid survey line - cannot calculate perpendicular
    if (is_zero(AB_diff_ne_cm.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos_ne_cm
    // Current vehicle position in NE plane (centimeters from EKF origin)
    const Vector2f curr_pos_ne_cm = pos_control->get_pos_desired_NEU_cm().xy().tofloat();
    Vector2f veh_to_start_pos = curr_pos_ne_cm - start_pos_ne_cm;

    // lengthen AB_diff_ne_cm so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    // If vehicle is far from the A-B line, extend the perpendicular to reach it
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff_ne_cm.length_squared()) {
        scalar = veh_to_start_pos.length() / AB_diff_ne_cm.length();
    }

    // create a line perpendicular to AB but originating at start_pos_ne_cm
    // Perpendicular vector is 90° rotation of AB vector: (x,y) → (-y,x) or (y,-x)
    // Two points define the perpendicular line extending in both directions
    Vector2f perp1 = start_pos_ne_cm + Vector2f(-AB_diff_ne_cm[1] * scalar, AB_diff_ne_cm[0] * scalar);
    Vector2f perp2 = start_pos_ne_cm + Vector2f(AB_diff_ne_cm[1] * scalar, -AB_diff_ne_cm[0] * scalar);

    // find the closest point on the perpendicular line
    // Project vehicle's current position onto perpendicular line
    // This is the optimal approach point that enters survey line at right angle
    const Vector2f closest2d = Vector2f::closest_point(curr_pos_ne_cm, perp1, perp2);
    next_dest_neu_cm.x = closest2d.x;
    next_dest_neu_cm.y = closest2d.y;

    // Handle altitude component of destination
    if (use_wpnav_alt) {
        // get altitude target from waypoint controller
        // Use current waypoint navigation altitude (continuing automated flight)
        terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest_neu_cm.z = wp_nav->get_wp_destination_NEU_cm().z;
    } else {
        // Use position controller's current altitude target
        // Typically when initiating automated flight from manual control
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
        next_dest_neu_cm.z = pos_control->get_pos_desired_U_cm();
        if (!terrain_alt) {
            // Convert from terrain-relative to absolute altitude
            next_dest_neu_cm.z += pos_control->get_pos_terrain_U_cm();
        }
    }

    return true;
}

/**
 * @brief Calculate sideways destination for next survey line
 * 
 * @details This function calculates the perpendicular offset destination for moving to the
 *          next parallel survey line. It takes the A-B survey line vector, rotates it 90°
 *          based on vehicle heading and configured direction, scales it to the configured
 *          line spacing, and calculates the destination from the current position.
 * 
 *          Direction Determination Algorithm:
 *          1. Get A-B survey line vector (defines survey line direction)
 *          2. Calculate perpendicular vector (90° rotation of A-B)
 *          3. Use vehicle heading (yaw) to determine which perpendicular direction to use
 *          4. Apply configured direction: forward/right/backward/left relative to vehicle
 *          5. Scale perpendicular to configured spacing distance (ZIGZAG_SIDE_DIST)
 * 
 *          Direction Interpretation:
 *          - RIGHT/LEFT: Perpendicular movement relative to vehicle's left-right axis
 *          - FORWARD/BACKWARD: Perpendicular movement relative to vehicle's fore-aft axis
 * 
 *          The algorithm uses dot products with vehicle heading to determine which way is
 *          "forward" or "right" relative to the A-B line, ensuring consistent spacing
 *          direction regardless of how A and B were originally marked.
 * 
 *          Example Survey Pattern (RIGHT direction, 5m spacing):
 *          ```
 *          A ←─────────── B    (Line 1: Fly B to A)
 *                ↓ 5m
 *          A ─────────→ B      (Line 2: Fly A to B)
 *                ↓ 5m  
 *          A ←─────────── B    (Line 3: Fly B to A)
 *          ```
 * 
 * @param[out] next_dest_neu_cm  Calculated sideways destination in NEU frame (cm from EKF origin)
 * @param[out] terrain_alt       Set to true if altitude is terrain-relative
 * 
 * @return true if calculation successful, false if A-B line has zero length
 * 
 * @note Sideways distance configured by ZIGZAG_SIDE_DIST parameter (0.1-100 meters)
 * @note Direction configured by ZIGZAG_DIRECTION parameter (0-3: forward/right/backward/left)
 * @note Altitude maintained at current position controller target
 * @note Uses current vehicle heading (yaw) to interpret direction relative to survey line
 * 
 * @see move_to_side() which uses this calculation to initiate sideways movement
 */
bool ModeZigZag::calculate_side_dest(Vector3f& next_dest_neu_cm, bool& terrain_alt) const
{
    // calculate vector from A to B
    // This defines the survey line direction (one side of the "zigzag")
    Vector2f AB_diff_ne_cm = dest_B_ne_cm - dest_A_ne_cm;

    // calculate a vertical right or left vector for AB from the current yaw direction
    // Perpendicular vector provides direction for line spacing
    Vector2f AB_side_ne_cm;
    
    if (zigzag_direction == Direction::RIGHT || zigzag_direction == Direction::LEFT) {
        // RIGHT/LEFT mode: Perpendicular relative to vehicle's left-right axis
        // Calculate dot product between vehicle heading and perpendicular to A-B
        // This determines which perpendicular direction aligns with "right" vs "left"
        float yaw_ab_sign = (-ahrs.sin_yaw() * AB_diff_ne_cm[1]) + (ahrs.cos_yaw() * -AB_diff_ne_cm[0]);
        
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::RIGHT ? 1 : -1))) {
            // Use one perpendicular direction (90° clockwise rotation)
            AB_side_ne_cm = Vector2f(AB_diff_ne_cm[1], -AB_diff_ne_cm[0]);
        } else {
            // Use opposite perpendicular direction (90° counter-clockwise rotation)
            AB_side_ne_cm = Vector2f(-AB_diff_ne_cm[1], AB_diff_ne_cm[0]);
        }
    } else {
        // FORWARD/BACKWARD mode: Perpendicular relative to vehicle's fore-aft axis
        // Calculate dot product to determine which perpendicular aligns with forward
        float yaw_ab_sign = (ahrs.cos_yaw() * AB_diff_ne_cm[1]) + (ahrs.sin_yaw() * -AB_diff_ne_cm[0]);
        
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::FORWARD ? 1 : -1))) {
            AB_side_ne_cm = Vector2f(AB_diff_ne_cm[1], -AB_diff_ne_cm[0]);
        } else {
            AB_side_ne_cm = Vector2f(-AB_diff_ne_cm[1], AB_diff_ne_cm[0]);
        }
    }

    // check distance the vertical vector between A and B
    // Zero length indicates degenerate case - cannot calculate perpendicular
    if (is_zero(AB_side_ne_cm.length_squared())) {
        return false;
    }

    // adjust AB_side_ne_cm length to zigzag_side_dist
    // Scale perpendicular vector to configured line spacing distance
    // Convert meters to centimeters (× 100) and normalize vector length
    float scalar = constrain_float(_side_dist_m, 0.1f, 100.0f) * 100 / safe_sqrt(AB_side_ne_cm.length_squared());

    // get current vehicle position as offset from EKF origin
    const Vector2f curr_pos_ne_cm = pos_control->get_pos_desired_NEU_cm().xy().tofloat();
    
    // Calculate next destination: current position + perpendicular offset
    // This creates parallel survey line at configured spacing from current line
    next_dest_neu_cm.xy() = curr_pos_ne_cm + (AB_side_ne_cm * scalar);

    // if we have a downward facing range finder then use terrain altitude targets
    // Maintain terrain-relative altitude if rangefinder available for terrain following
    terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    
    // Use current altitude target (maintain height during sideways movement)
    next_dest_neu_cm.z = pos_control->get_pos_desired_U_cm();

    return true;
}

// run zigzag auto feature which is automate both AB and sideways
void ModeZigZag::run_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    // make sure both A and B point are registered and not when moving to A or B
    if (stage != MANUAL_REGAIN) {
        return;
    }

    is_auto = true;
    // resume if zigzag auto is suspended
    if (is_suspended && line_count <= line_num) {
        // resume the stage when it was suspended
        if (auto_stage == AutoState::AB_MOVING) {
            line_count--;
            save_or_move_to_destination(ab_dest_stored);
        } else if (auto_stage == AutoState::SIDEWAYS) {
            wp_nav->wp_and_spline_init_cm();
            if (wp_nav->set_wp_destination_NEU_cm(current_dest, current_terr_alt)) {
                stage = AUTO;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "%s: moving to %s", name(), dir[(uint8_t)zigzag_direction]);
            }
        }
    } else {
        move_to_side();
    }
}

// suspend zigzag auto
void ModeZigZag::suspend_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    if (auto_stage != AutoState::MANUAL) {
        is_suspended = true;
        return_to_manual_control(true);
    }
}

// initialize zigzag auto
void ModeZigZag::init_auto()
{
    is_auto = false;
    auto_stage = AutoState::MANUAL;
    line_count = 0;
    is_suspended = false;
}

// spray on / off
void ModeZigZag::spray(bool b)
{
#if HAL_SPRAYER_ENABLED
    if (_spray_enabled) {
        copter.sprayer.run(b);
    }
#endif
}

float ModeZigZag::wp_distance_m() const
{
    return is_auto ? wp_nav->get_wp_distance_to_destination_cm() * 0.01f : 0.0f;
}
float ModeZigZag::wp_bearing_deg() const
{
    return is_auto ? degrees(wp_nav->get_wp_bearing_to_destination_rad()) : 0;
}
float ModeZigZag::crosstrack_error() const
{
    return is_auto ? wp_nav->crosstrack_error() : 0;
}

#endif // MODE_ZIGZAG_ENABLED
