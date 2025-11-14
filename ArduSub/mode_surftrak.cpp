/**
 * @file mode_surftrak.cpp
 * @brief SurfTrak (Surface Tracking) flight mode implementation for ArduSub
 * 
 * @details SurfTrak is a specialized depth control mode for underwater vehicles that maintains
 *          a constant distance from a surface (typically the seabed or ceiling) using rangefinder
 *          measurements. This mode is particularly useful for underwater inspection, survey operations,
 *          and terrain-following missions where maintaining a consistent altitude above the bottom
 *          is critical for data quality.
 * 
 * Key Features:
 * - Maintains constant distance from detected surface using rangefinder
 * - Automatically adjusts depth as terrain elevation changes
 * - Smooth pilot override with minimal "bounce back" when releasing controls
 * - Robust handling of rangefinder glitches to prevent unwanted target changes
 * - Safety limits to prevent surface breaches or bottom contact
 * 
 * Mode States:
 * - Reset state: rangefinder_target_cm < 0, waiting for valid conditions
 * - Active state: Actively tracking surface at configured target distance
 * - Pilot control: Manual depth adjustment with automatic target update on release
 * 
 * Typical Use Cases:
 * - Benthic surveys requiring constant altitude above seafloor
 * - Pipeline and infrastructure inspection
 * - Archaeological site documentation
 * - Bathymetric mapping with consistent sensor height
 * - Cave and confined space exploration
 * 
 * @note Requires a functioning rangefinder with valid readings between min and max range
 * @note Vehicle must be below SURFTRAK_DEPTH parameter to engage tracking
 * @warning Rangefinder health is critical - ensure proper sensor operation before use
 * 
 * @see ModeSurftrak
 * @see ModeAlthold
 * @see AC_PosControl
 * 
 * Source: ArduSub/mode_surftrak.cpp
 */

#include "Sub.h"

/*
 * SURFTRAK (surface tracking) -- a variation on ALT_HOLD (depth hold)
 *
 * SURFTRAK starts in the "reset" state (rangefinder_target_cm < 0). SURFTRAK exits the reset state when these
 * conditions are met:
 * -- There is a good rangefinder reading (the rangefinder is healthy, the reading is between min and max, etc.)
 * -- The sub is below SURFTRAK_DEPTH
 *
 * During normal operation, SURFTRAK sets the offset target to the current terrain altitude estimate and calls
 * AC_PosControl to do the rest.
 *
 * We generally do not want to reset SURFTRAK if the rangefinder glitches, since that will result in a new rangefinder
 * target. E.g., if a pilot is running 1m above the seafloor, there is a glitch, and the next rangefinder reading shows
 * 1.1m, the desired behavior is to move 10cm closer to the seafloor, vs setting a new target of 1.1m above the
 * seafloor.
 *
 * If the pilot takes control, SURFTRAK uses the change in depth readings to adjust the rangefinder target. This
 * minimizes the "bounce back" that can happen as the slower rangefinder catches up to the quicker barometer.
 */

#define INVALID_TARGET (-1)
#define HAS_VALID_TARGET (rangefinder_target_cm > 0)

/**
 * @brief Constructor for ModeSurftrak
 * 
 * @details Initializes SurfTrak mode with invalid target state and pilot control flags.
 *          The mode starts in reset state and waits for valid conditions before engaging
 *          surface tracking.
 */
ModeSurftrak::ModeSurftrak() :
        rangefinder_target_cm(INVALID_TARGET),
        pilot_in_control(false),
        pilot_control_start_z_cm(0)
{ }

/**
 * @brief Initialize SurfTrak mode and prepare for surface tracking operation
 * 
 * @details This function initializes the SurfTrak mode by first initializing the base
 *          AltHold mode, then resetting the rangefinder target to invalid state. The mode
 *          performs pre-flight checks to ensure rangefinder availability and proper depth
 *          positioning before allowing surface tracking to engage.
 * 
 *          Initialization Sequence:
 *          1. Initialize parent AltHold mode (depth hold functionality)
 *          2. Reset rangefinder target to invalid state
 *          3. Check rangefinder health and availability
 *          4. Verify vehicle is below SURFTRAK_DEPTH threshold
 *          5. Send status messages to ground control station
 * 
 *          The mode will enter the "waiting" state if conditions are not met, and will
 *          automatically engage surface tracking once a valid rangefinder reading is obtained
 *          and the vehicle descends below the configured depth threshold.
 * 
 * @param[in] ignore_checks If true, skip pre-arm safety checks (used for forced mode changes)
 * 
 * @return true if initialization successful, false if base AltHold mode init fails
 * 
 * @note Requires AP_RANGEFINDER_ENABLED to be compiled in for full functionality
 * @note Vehicle must be below SURFTRAK_DEPTH parameter value to begin tracking
 * @note Rangefinder must return valid readings within configured min/max range limits
 * 
 * @warning If rangefinder is not healthy, pilot will receive "waiting for rangefinder" message
 * @warning If vehicle is above SURFTRAK_DEPTH, pilot must descend to engage tracking
 * 
 * @see ModeAlthold::init()
 * @see reset()
 * @see sub.rangefinder_alt_ok()
 * 
 * Source: ArduSub/mode_surftrak.cpp:32-49
 */
bool ModeSurftrak::init(bool ignore_checks)
{
    if (!ModeAlthold::init(ignore_checks)) {
        return false;
    }

    reset();

    if (!sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "waiting for a rangefinder reading");
#if AP_RANGEFINDER_ENABLED
    } else if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %f meters to hold range", sub.g.surftrak_depth * 0.01f);
#endif
    }

    return true;
}

/**
 * @brief Main loop execution for SurfTrak mode, called at main loop rate
 * 
 * @details This is the primary execution function for SurfTrak mode, called continuously
 *          at the main loop rate (typically 100Hz+). It orchestrates the terrain tracking
 *          algorithm by managing mode state and calling the appropriate control functions.
 * 
 *          Execution Flow:
 *          1. run_pre(): Execute pre-processing common to all modes
 *          2. State management:
 *             - If disarmed: Reset rangefinder target and clear tracking state
 *             - If armed: Execute active terrain tracking control
 *          3. run_post(): Execute post-processing common to all modes
 * 
 *          The terrain tracking algorithm maintains constant distance from the detected
 *          surface by continuously adjusting the depth target based on rangefinder
 *          measurements. When the vehicle is armed and conditions are valid, control_range()
 *          handles pilot input, rangefinder updates, and position control integration.
 * 
 * @note Called at main loop rate, typically 100Hz or higher
 * @note Automatically resets tracking state when motors are disarmed for safety
 * 
 * @warning Do not call this function directly - it is invoked by the mode manager
 * 
 * @see control_range()
 * @see reset()
 * @see run_pre()
 * @see run_post()
 * 
 * Source: ArduSub/mode_surftrak.cpp:51-63
 */
void ModeSurftrak::run()
{
    run_pre();

    if (!motors.armed()) {
        // Forget rangefinder target
        reset();
    } else {
        control_range();
    }

    run_post();
}

/**
 * @brief Set the desired distance from surface for terrain tracking
 * 
 * @details This function sets the target rangefinder distance that the vehicle will maintain
 *          from the detected surface. It performs comprehensive validation checks to ensure
 *          the target is safe and achievable. This function can be called from Lua scripting
 *          or internal mode logic, so it implements additional safety checks beyond normal
 *          operation.
 * 
 *          Validation Checks:
 *          1. Vehicle must be in SURFTRAK mode
 *          2. Vehicle must be below SURFTRAK_DEPTH threshold
 *          3. Target must be above rangefinder minimum range
 *          4. Target must be below rangefinder maximum range
 * 
 *          If all checks pass, the function:
 *          - Sets the new rangefinder target distance
 *          - Calculates the terrain offset (current_depth - target_distance)
 *          - Initializes AC_PosControl terrain tracking with calculated offset
 *          - Sends confirmation message to ground control station
 * 
 *          If any check fails, the mode is reset to invalid target state and an appropriate
 *          warning message is sent to the pilot.
 * 
 * @param[in] target_cm Desired distance from surface in centimeters (e.g., 100cm = 1 meter)
 * 
 * @return true if target was successfully set, false if validation failed
 * 
 * @note This function may be called from Lua scripting for automated mission control
 * @note Rangefinder limits are defined by hardware specifications and configuration parameters
 * @note Target distance is measured perpendicular to the detected surface
 * 
 * @warning Target must be within rangefinder operational range (typically 0.2m to 10m)
 * @warning Attempting to set target while above SURFTRAK_DEPTH will fail
 * @warning Invalid target values cause mode reset - ensure values are validated before calling
 * 
 * @see reset()
 * @see AC_PosControl::init_pos_terrain_U_cm()
 * 
 * Source: ArduSub/mode_surftrak.cpp:68-99
 */
bool ModeSurftrak::set_rangefinder_target_cm(float target_cm)
{
    bool success = false;

#if AP_RANGEFINDER_ENABLED
    if (sub.control_mode != Number::SURFTRAK) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "wrong mode, rangefinder target not set");
    } else if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %f meters to set rangefinder target", sub.g.surftrak_depth * 0.01f);
    } else if (target_cm < sub.rangefinder_state.min*100) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target below minimum, ignored");
    } else if (target_cm > sub.rangefinder_state.max*100) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target above maximum, ignored");
    } else {
        success = true;
    }

    if (success) {
        rangefinder_target_cm = target_cm;
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %.2f meters", rangefinder_target_cm * 0.01f);

        // Initialize the terrain offset
        auto terrain_offset_cm = sub.inertial_nav.get_position_z_up_cm() - rangefinder_target_cm;
        sub.pos_control.init_pos_terrain_U_cm(terrain_offset_cm);

    } else {
        reset();
    }
#endif

    return success;
}

/**
 * @brief Reset SurfTrak mode to invalid target state
 * 
 * @details This function resets the SurfTrak mode to its initial "waiting" state by
 *          invalidating the rangefinder target and clearing the terrain offset in the
 *          position controller. After reset, the mode will wait for valid rangefinder
 *          readings and proper depth positioning before re-engaging surface tracking.
 * 
 *          Reset is triggered in the following scenarios:
 *          - Mode initialization
 *          - Motors disarmed for safety
 *          - Invalid target conditions detected
 *          - Vehicle at surface or bottom limits
 *          - Failed target validation checks
 * 
 *          The reset operation:
 *          1. Sets rangefinder_target_cm to INVALID_TARGET (-1)
 *          2. Clears terrain offset in AC_PosControl (sets to 0)
 * 
 *          After reset, surface tracking will automatically re-engage when:
 *          - Valid rangefinder reading is obtained
 *          - Vehicle is below SURFTRAK_DEPTH threshold
 *          - Rangefinder measurement is within min/max limits
 * 
 * @note This is an internal function called by mode logic, not typically called directly
 * @note After reset, mode returns to basic depth hold until conditions are valid
 * 
 * @see init()
 * @see run()
 * @see set_rangefinder_target_cm()
 * 
 * Source: ArduSub/mode_surftrak.cpp:101-107
 */
void ModeSurftrak::reset()
{
    rangefinder_target_cm = INVALID_TARGET;

    // Reset the terrain offset
    sub.pos_control.init_pos_terrain_U_cm(0);
}

/**
 * @brief Main terrain tracking controller - call at 100Hz or higher
 * 
 * @details This is the core control function that implements the surface tracking algorithm.
 *          It manages pilot input, rangefinder-based terrain following, and smooth transitions
 *          between automatic tracking and manual control. The controller is designed to
 *          minimize "bounce back" when the pilot releases controls by tracking depth changes
 *          during manual operation.
 * 
 *          Control Algorithm:
 *          1. Get pilot's desired climb rate from throttle input
 *          2. Constrain climb rate to configured speed limits
 *          3. Determine control state:
 *             
 *             NO PILOT INPUT (throttle in deadzone):
 *             - If pilot just released control: Update target with accumulated depth change
 *             - If at surface: Set target 5cm below surface limit and reset tracking
 *             - If at bottom: Set target 10cm above current position and reset tracking
 *             - Normal operation: Update terrain offset from rangefinder readings
 *             
 *             PILOT INPUT (throttle active):
 *             - If not already in manual control: Record starting depth
 *             - Mark pilot as in control
 *             - Allow manual depth adjustment
 * 
 *          4. Apply climb rate to position controller (integrates with terrain offset)
 *          5. Execute PID depth controller to generate motor commands
 * 
 *          The depth change tracking during manual control prevents the vehicle from
 *          "bouncing back" to the old target when the pilot releases controls. Instead,
 *          the target is adjusted by the amount the pilot moved the vehicle, maintaining
 *          smooth operation.
 * 
 * @note Must be called at 100Hz or higher for stable control performance
 * @note Pilot climb rate is constrained to PILOT_SPEED_UP and PILOT_SPEED_DN parameters
 * @note Throttle deadzone is 0.05 cm/s (effectively zero for control logic)
 * 
 * @warning Surface and bottom detection triggers automatic reset for safety
 * @warning Ensure position controller is properly initialized before calling
 * 
 * @see update_surface_offset()
 * @see set_rangefinder_target_cm()
 * @see reset()
 * @see AC_PosControl::set_pos_target_U_from_climb_rate_cm()
 * @see AC_PosControl::update_U_controller()
 * 
 * Source: ArduSub/mode_surftrak.cpp:112-146
 */
void ModeSurftrak::control_range() {
    float target_climb_rate_cm_s = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // Desired_climb_rate returns 0 when within the deadzone
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (pilot_in_control) {
            // Pilot has released control; apply the delta to the rangefinder target
            set_rangefinder_target_cm(rangefinder_target_cm + inertial_nav.get_position_z_up_cm() - pilot_control_start_z_cm);
            pilot_in_control = false;
        }
        if (sub.ap.at_surface) {
            // Set target depth to 5 cm below SURFACE_DEPTH and reset
            position_control->set_pos_desired_U_cm(MIN(position_control->get_pos_desired_U_cm(), g.surface_depth - 5.0f));
            reset();
        } else if (sub.ap.at_bottom) {
            // Set target depth to 10 cm above bottom and reset
            position_control->set_pos_desired_U_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_desired_U_cm()));
            reset();
        } else {
            // Typical operation
            update_surface_offset();
        }
    } else if (HAS_VALID_TARGET && !pilot_in_control) {
        // Pilot has taken control; note the current depth
        pilot_control_start_z_cm = inertial_nav.get_position_z_up_cm();
        pilot_in_control = true;
    }

    // Set the target altitude from the climb rate and the terrain offset
    position_control->set_pos_target_U_from_climb_rate_cm(target_climb_rate_cm_s);

    // Run the PID controllers
    position_control->update_U_controller();
}

/**
 * @brief Update terrain offset based on current rangefinder measurements
 * 
 * @details This function integrates rangefinder measurements with the position controller
 *          to maintain constant distance from the detected surface. It handles initial
 *          target acquisition, continuous terrain tracking updates, and safety limiting
 *          to prevent surface breaches.
 * 
 *          Update Algorithm:
 *          1. Check if rangefinder provides valid altitude measurement
 *          2. Retrieve current terrain offset from rangefinder state
 *          
 *          FIRST READING / AFTER RESET:
 *          - If no valid target exists and vehicle is below SURFTRAK_DEPTH:
 *            * Calculate initial target as (current_depth - terrain_offset)
 *            * Sets up automatic surface tracking
 *          
 *          CONTINUOUS TRACKING:
 *          - If valid target exists:
 *            * Calculate desired depth: terrain_offset + target_distance
 *            * Check if tracking would cause ascent above SURFTRAK_DEPTH
 *            * If depth limit would be exceeded: Adjust terrain offset to stay below limit
 *            * Update AC_PosControl with corrected terrain offset
 *            * Position controller automatically adjusts depth to maintain target distance
 * 
 *          The terrain offset represents the estimated elevation of the terrain relative
 *          to the home position. By adding the desired target distance to this offset,
 *          the vehicle maintains constant altitude above changing terrain.
 * 
 *          Safety Limiting:
 *          To prevent surface breaches, the function checks if terrain-following would
 *          cause the vehicle to ascend above SURFTRAK_DEPTH. If so, it clamps the terrain
 *          offset to keep the vehicle below the depth limit. This is preferable to losing
 *          tracking or triggering failsafe conditions.
 * 
 * @note Only processes updates when rangefinder returns valid measurements
 * @note Requires AP_RANGEFINDER_ENABLED compile-time feature flag
 * @note Called continuously during normal SurfTrak operation from control_range()
 * @note Terrain offset is in centimeters, positive up (NED frame convention)
 * 
 * @warning Rangefinder glitches are filtered by rangefinder_alt_ok() health check
 * @warning Depth limiting prevents surface breaches but may temporarily lose target distance
 * 
 * @see control_range()
 * @see set_rangefinder_target_cm()
 * @see AC_PosControl::set_pos_terrain_target_U_cm()
 * @see sub.rangefinder_alt_ok()
 * 
 * Source: ArduSub/mode_surftrak.cpp:151-176
 */
void ModeSurftrak::update_surface_offset()
{
#if AP_RANGEFINDER_ENABLED
    if (sub.rangefinder_alt_ok()) {
        // Get the latest terrain offset
        float rangefinder_terrain_offset_cm = sub.rangefinder_state.rangefinder_terrain_offset_cm;

        // Handle the first reading or a reset
        if (!HAS_VALID_TARGET && sub.rangefinder_state.inertial_alt_cm < sub.g.surftrak_depth) {
            set_rangefinder_target_cm(sub.rangefinder_state.inertial_alt_cm - rangefinder_terrain_offset_cm);
        }

        if (HAS_VALID_TARGET) {
            // Will the new offset target cause the sub to ascend above SURFTRAK_DEPTH?
            float desired_z_cm = rangefinder_terrain_offset_cm + rangefinder_target_cm;
            if (desired_z_cm >= sub.g.surftrak_depth) {
                // Adjust the terrain offset to stay below SURFTRAK_DEPTH, this should avoid "at_surface" events
                rangefinder_terrain_offset_cm += sub.g.surftrak_depth - desired_z_cm;
            }

            // Set the offset target, AC_PosControl will do the rest
            sub.pos_control.set_pos_terrain_target_U_cm(rangefinder_terrain_offset_cm);
        }
    }
#endif  // AP_RANGEFINDER_ENABLED
}
