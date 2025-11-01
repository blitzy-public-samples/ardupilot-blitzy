/**
 * @file landing_gear.cpp
 * @brief Landing gear control integration for ArduCopter
 * 
 * @details This file provides the vehicle-level interface to the AP_LandingGear
 *          library, enabling automatic and manual control of retractable landing
 *          gear on multicopter vehicles. The landing gear system supports:
 *          - Automatic deployment/retraction based on altitude thresholds
 *          - Manual control via RC switches or GCS commands
 *          - Safety interlocks to prevent gear retraction at unsafe altitudes
 *          - Integration with rangefinder for precise ground clearance
 *          - Servo-based mechanical actuation
 * 
 *          The landing gear controller runs at 10Hz as part of the vehicle's
 *          scheduler loop and provides height information to the AP_LandingGear
 *          library which manages state transitions and servo outputs.
 * 
 * @note The actual gear deployment/retraction logic, state management, and
 *       servo control are implemented in the AP_LandingGear library. This file
 *       serves as the vehicle-specific integration point.
 * 
 * @see libraries/AP_LandingGear/AP_LandingGear.h
 * @see libraries/AP_LandingGear/AP_LandingGear.cpp
 * 
 * Source: ArduCopter/landing_gear.cpp
 */

#include "Copter.h"

#if AP_LANDINGGEAR_ENABLED

/**
 * @brief Update landing gear state based on current altitude and flight conditions
 * 
 * @details This function is the main entry point for landing gear control in ArduCopter,
 *          called at 10Hz from the vehicle scheduler. It performs the following operations:
 *          
 *          1. Verifies landing gear servo output is configured (exits if not assigned)
 *          2. Determines current height above ground using best available sensor:
 *             - Flight mode's altitude above ground estimate (EKF-based)
 *             - Rangefinder measurement if available and valid
 *          3. Provides height information to AP_LandingGear library for automatic control
 *          
 *          Height Sensor Priority:
 *          - Rangefinder (if enabled and returning valid data): Most accurate for low altitudes
 *          - Flight mode altitude estimate: Fallback using EKF position relative to home/terrain
 *          
 *          The AP_LandingGear library uses this height information to automatically:
 *          - Deploy gear when descending below LGR_DEPLOY_ALT parameter
 *          - Retract gear when climbing above LGR_RETRACT_ALT parameter
 *          - Enforce safety interlocks preventing retraction at low altitude
 *          - Handle manual override commands from pilot or GCS
 *          
 *          Servo Control:
 *          The landing gear position is output through the servo channel assigned
 *          to SRV_Channel::k_landing_gear_control function. The AP_LandingGear
 *          library manages the actual PWM output values for deploy/retract positions.
 *          
 *          Safety Considerations:
 *          - Gear will NOT retract below minimum safe altitude (safety interlock)
 *          - Manual pilot commands can override automatic behavior
 *          - Weight-on-wheels sensor (if configured) prevents gear changes on ground
 *          - Gear state transitions include delay timers to prevent servo chatter
 * 
 * @note This function must be called at 10Hz for proper operation. The scheduler
 *       task is configured in Copter::scheduler_tasks[] array.
 * 
 * @note If no servo channel is assigned to landing gear control function, this
 *       function exits immediately without processing.
 * 
 * @warning Incorrect landing gear configuration or servo assignment can result in
 *          gear not deploying before landing, potentially causing vehicle damage.
 *          Always test landing gear operation in safe conditions before flight.
 * 
 * @warning Rangefinder must be properly configured and oriented downward (ROTATION_PITCH_270)
 *          for automatic gear control to work correctly with terrain following.
 * 
 * @see AP_LandingGear::update() - Library function that receives height data
 * @see SRV_Channel::k_landing_gear_control - Servo output function assignment
 * @see Copter::scheduler_tasks - Scheduler configuration for 10Hz execution
 * 
 * Source: ArduCopter/landing_gear.cpp:6-38
 */
void Copter::landinggear_update()
{
    // Exit immediately if no landing gear servo output has been configured.
    // The k_landing_gear_control function must be assigned to a servo channel
    // (typically via SERVOn_FUNCTION parameter) for gear control to be active.
    // This check prevents unnecessary processing when landing gear hardware is not installed.
    if (!SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        return;
    }

    // Get current height above ground for automatic gear deployment/retraction decisions.
    // Start with the flight mode's altitude estimate, which uses EKF position data
    // relative to home altitude or terrain database (depending on configuration).
    // This provides the baseline height measurement used for gear control logic.
    int32_t height_cm = flightmode->get_alt_above_ground_cm();

    // Attempt to use rangefinder for more accurate height measurement if available.
    // Rangefinder provides direct ground distance measurement, which is more accurate
    // than EKF altitude estimates at low altitudes where landing gear control is critical.
    // The downward-facing rangefinder (ROTATION_PITCH_270 = pointing down) is queried
    // for its current status and measurement data.
#if AP_RANGEFINDER_ENABLED
    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::NotConnected:
    case RangeFinder::Status::NoData:
        // Rangefinder is not connected or providing no data.
        // Fall back to using the EKF-based altitude above ground estimate.
        // This maintains gear control functionality even without rangefinder hardware.
        break;

    case RangeFinder::Status::OutOfRangeLow:
        // Rangefinder reports distance below its minimum measurement range.
        // This indicates the vehicle is very close to the ground (typically < 20cm).
        // Force height to zero to ensure landing gear is deployed for landing.
        // This is a safety feature to guarantee gear deployment in close proximity to ground.
        height_cm = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
    case RangeFinder::Status::Good:
        // Rangefinder is providing valid distance measurements.
        // Use the filtered rangefinder altitude reading for precise height control.
        // The filtered value (alt_cm_filt) provides smoothed data to prevent
        // gear oscillation from rangefinder noise or momentary dropouts.
        // This is the preferred height source when available as it's most accurate
        // for landing gear control decisions at low altitudes.
        height_cm = rangefinder_state.alt_cm_filt.get();
        break;
    }
#endif  // AP_RANGEFINDER_ENABLED

    // Pass the determined height to the AP_LandingGear library for state management.
    // Convert from centimeters (ArduPilot internal altitude unit) to meters (expected by library).
    // 
    // The AP_LandingGear::update() method uses this height to:
    // 1. Compare against LGR_DEPLOY_ALT parameter - deploys gear when descending below this altitude
    // 2. Compare against LGR_RETRACT_ALT parameter - retracts gear when climbing above this altitude
    // 3. Enforce safety interlocks - prevents retraction below minimum safe altitude
    // 4. Manage state transitions - handles deploy/retract sequences with appropriate delays
    // 5. Control servo output - commands the landing gear servo to deployed or retracted position
    // 
    // The library also processes manual override commands from:
    // - RC switch inputs (if configured via auxiliary function)
    // - GCS MAVLink commands (COMMAND_LONG with MAV_CMD_DO_LAND_START or gear control)
    // - Scripting API calls (if AP_Scripting is enabled)
    // 
    // Manual commands take priority over automatic altitude-based control, allowing
    // the pilot to override automatic behavior if needed for specific flight situations.
    //
    // Note: The library maintains internal state (deployed/retracted/deploying/retracting)
    // and outputs the appropriate servo PWM values through the configured servo channel.
    landinggear.update(height_cm * 0.01f); // convert cm->m for update call
}

#endif // AP_LANDINGGEAR_ENABLED
