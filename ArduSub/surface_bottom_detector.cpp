/**
 * @file surface_bottom_detector.cpp
 * @brief Surface and bottom detection system for underwater vehicles
 * 
 * @details This module implements intelligent surface breach and bottom contact
 *          detection for ArduSub using sensor fusion of barometer (depth) data
 *          and velocity estimates from the AHRS system.
 *          
 *          Detection Approach:
 *          - **With external pressure sensor**: Uses accurate depth measurements
 *            from barometer combined with motor throttle limits and velocity
 *            to detect surface/bottom conditions
 *          - **Without external sensor**: Relies on velocity estimates and
 *            motor throttle saturation to infer surface/bottom contact
 *          
 *          Surface Detection:
 *          - Primarily used for safety to prevent vehicle from breaching surface
 *          - Triggers when depth exceeds configured surface_depth threshold
 *          - Includes hysteresis (5cm buffer) to prevent rapid state transitions
 *          - Without depth sensor: detects upper throttle limit with zero velocity
 *          
 *          Bottom Detection:
 *          - Used for landing operations and preventing over-descent
 *          - Triggers when lower throttle limit reached with stationary velocity
 *          - Requires sustained condition (time-based trigger threshold)
 *          - Critical for autonomous landing and terrain following
 *          
 *          State Management:
 *          - ap.at_surface flag indicates surface breach condition
 *          - ap.at_bottom flag indicates bottom contact condition
 *          - States logged to dataflash for post-flight analysis
 *          - Detection counters provide debouncing for stable state transitions
 * 
 * @note This system demonstrates sensor fusion approach: combining accurate
 *       depth measurements when available with fallback to motor behavior
 *       analysis when external sensors unavailable.
 * 
 * @warning Surface and bottom detection are safety-critical for underwater
 *          operations. False negatives could result in surface breach (dangerous
 *          for nearby vessels) or bottom collision (vehicle damage).
 * 
 * @author Jacob Walser (jacob@bluerobotics.com)
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Sub.h"

/**
 * @brief Counter for bottom contact detection debouncing
 * 
 * Incremented each loop iteration when bottom contact criteria are met
 * (lower throttle limit + stationary velocity). When count reaches
 * BOTTOM_DETECTOR_TRIGGER_SEC * MAIN_LOOP_RATE, bottom contact is confirmed.
 * Reset to zero when bottom state changes or criteria no longer met.
 */
static uint32_t bottom_detector_count = 0;

/**
 * @brief Counter for surface breach detection debouncing
 * 
 * Incremented each loop iteration when surface breach criteria are met
 * (upper throttle limit + stationary velocity, when no depth sensor available).
 * When count reaches SURFACE_DETECTOR_TRIGGER_SEC * MAIN_LOOP_RATE, surface
 * breach is confirmed. Reset to zero when surface state changes.
 */
static uint32_t surface_detector_count = 0;

/**
 * @brief Cached current depth reading from barometer
 * 
 * Stores the most recent barometer altitude reading (in cm) for use in
 * surface detection logic. Only valid when external pressure sensor is
 * present and healthy (ap.depth_sensor_present && sensor_health.depth).
 */
static float current_depth = 0;

/**
 * @brief Update surface and bottom contact detection states
 * 
 * @details This function implements the core detection algorithm for identifying
 *          surface breach and bottom contact conditions. It uses a hierarchical
 *          sensor fusion approach that prioritizes accurate depth sensor data
 *          when available, falling back to motor behavior analysis otherwise.
 *          
 *          Detection Algorithm (with external pressure sensor):
 *          1. Read current depth from barometer (cm)
 *          2. Surface detection: Compare depth to g.surface_depth threshold
 *             - Includes 5cm hysteresis when already at surface
 *             - Immediate detection when depth exceeds threshold
 *          3. Bottom detection: Check for lower throttle limit + stationary velocity
 *             - Requires sustained condition over BOTTOM_DETECTOR_TRIGGER_SEC
 *             - Velocity threshold: -0.05 to +0.05 m/s vertical
 *          
 *          Detection Algorithm (without external pressure sensor):
 *          1. Check for stationary vertical velocity (velocity.z within ±0.05 m/s)
 *          2. Surface detection: Upper throttle limit + stationary = pushing against surface
 *             - Requires sustained condition over SURFACE_DETECTOR_TRIGGER_SEC
 *          3. Bottom detection: Lower throttle limit + stationary = resting on bottom
 *             - Requires sustained condition over BOTTOM_DETECTOR_TRIGGER_SEC
 *          
 *          Stationary Velocity Criteria:
 *          - Vertical velocity must be between -0.05 and +0.05 m/s
 *          - This threshold prevents false triggers during ascent/descent
 *          - Obtained from AHRS velocity estimate in NED frame
 *          
 *          Motor Throttle Limits:
 *          - motors.limit.throttle_upper: Maximum throttle applied, no more upward force possible
 *          - motors.limit.throttle_lower: Minimum throttle applied, no more downward force possible
 *          - These limits indicate vehicle cannot move further in that direction
 *          
 *          State Updates:
 *          - Updates ap.at_surface flag via set_surfaced()
 *          - Updates ap.at_bottom flag via set_bottomed()
 *          - Logs state transitions to dataflash (SURFACED, BOTTOMED events)
 *          
 * @note Called at MAIN_LOOP_RATE (typically 50-400Hz depending on vehicle configuration).
 *       The high call rate enables responsive detection but counters provide debouncing.
 * 
 * @note Sensor Fusion Strategy: External pressure sensor provides absolute depth reference
 *       with cm-level accuracy. Without it, detection relies on motor saturation analysis
 *       which is less precise but still effective for safety-critical detection.
 * 
 * @warning This function only operates when motors are armed (motors.armed()).
 *          When disarmed, both detection flags are cleared to prevent false states.
 * 
 * @warning Surface breach detection is critical for safety in shared waterways.
 *          False negatives could result in unexpected surface breaches near vessels.
 * 
 * @warning Bottom contact detection prevents vehicle damage from over-descent.
 *          False negatives could result in collision with seafloor or obstacles.
 * 
 * @todo Consider reducing call rate - detection doesn't require main loop frequency.
 *       Could be called at 10-20Hz for efficiency without compromising safety.
 * 
 * @see set_surfaced() for surface state management
 * @see set_bottomed() for bottom state management
 * @see SURFACE_DETECTOR_TRIGGER_SEC configuration constant
 * @see BOTTOM_DETECTOR_TRIGGER_SEC configuration constant
 * 
 * Source: ArduSub/surface_bottom_detector.cpp:13-78
 */
void Sub::update_surface_and_bottom_detector()
{
    if (!motors.armed()) { // only update when armed
        set_surfaced(false);
        set_bottomed(false);
        return;
    }

    Vector3f velocity;
    UNUSED_RESULT(ahrs.get_velocity_NED(velocity));

    // check that we are not moving up or down
    bool vel_stationary = velocity.z > -0.05 && velocity.z < 0.05;

    if (ap.depth_sensor_present && sensor_health.depth) { // we can use the external pressure sensor for a very accurate and current measure of our z axis position
        current_depth = barometer.get_altitude(); // cm


        if (ap.at_surface) {
            set_surfaced(current_depth > g.surface_depth*0.01 - 0.05); // add a 5cm buffer so it doesn't trigger too often
        } else {
            set_surfaced(current_depth > g.surface_depth*0.01); // If we are above surface depth, we are surfaced
        }


        if (motors.limit.throttle_lower && vel_stationary) {
            // bottom criteria met - increment the counter and check if we've triggered
            if (bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                bottom_detector_count++;
            } else {
                set_bottomed(true);
            }

        } else {
            set_bottomed(false);
        }

        // with no external baro, the only thing we have to go by is a vertical velocity estimate
    } else if (vel_stationary) {
        if (motors.limit.throttle_upper) {

            // surface criteria met, increment counter and see if we've triggered
            if (surface_detector_count < ((float)SURFACE_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                surface_detector_count++;
            } else {
                set_surfaced(true);
            }

        } else if (motors.limit.throttle_lower) {
            // bottom criteria met, increment counter and see if we've triggered
            if (bottom_detector_count < ((float)BOTTOM_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                bottom_detector_count++;
            } else {
                set_bottomed(true);
            }

        } else { // we're not at the limits of throttle, so reset both detectors
            set_surfaced(false);
            set_bottomed(false);
        }

    } else { // we're moving up or down, so reset both detectors
        set_surfaced(false);
        set_bottomed(false);
    }
}

/**
 * @brief Update surface breach state flag and log state transitions
 * 
 * @details This function manages the ap.at_surface flag which indicates whether
 *          the vehicle has breached the water surface. It provides:
 *          - State change detection to avoid redundant updates
 *          - Detection counter reset for clean state transitions
 *          - Dataflash logging of surface breach events for analysis
 *          
 *          The surface state is used by:
 *          - Flight modes to modify behavior near surface
 *          - Failsafe systems to prevent uncontrolled surface breach
 *          - GCS telemetry to alert operators of surface proximity
 *          - Post-flight analysis to understand vehicle behavior
 *          
 *          State Transition Logging:
 *          - SURFACED event: Vehicle has breached surface (safety concern)
 *          - NOT_SURFACED event: Vehicle has descended below surface threshold
 *          
 * @param[in] at_surface  True if vehicle is at/above surface, false if submerged
 * 
 * @note This function includes state change detection optimization - if the
 *       requested state matches current state, function returns immediately
 *       without logging or counter reset.
 * 
 * @note The surface_detector_count is reset to zero on every state transition
 *       to ensure clean detection cycles for the next state change.
 * 
 * @warning Surface breach detection is safety-critical. This state may trigger
 *          automatic descent or operator alerts to prevent collisions with
 *          surface vessels or shore structures.
 * 
 * @see update_surface_and_bottom_detector() for detection algorithm
 * @see LogEvent::SURFACED and LogEvent::NOT_SURFACED for log message definitions
 * 
 * Source: ArduSub/surface_bottom_detector.cpp:80-97
 */
void Sub::set_surfaced(bool at_surface)
{


    if (ap.at_surface == at_surface) { // do nothing if state unchanged
        return;
    }

    ap.at_surface = at_surface;

    surface_detector_count = 0;

    if (ap.at_surface) {
        LOGGER_WRITE_EVENT(LogEvent::SURFACED);
    } else {
        LOGGER_WRITE_EVENT(LogEvent::NOT_SURFACED);
    }
}

/**
 * @brief Update bottom contact state flag and log state transitions
 * 
 * @details This function manages the ap.at_bottom flag which indicates whether
 *          the vehicle is in contact with the seafloor or underwater terrain. It provides:
 *          - State change detection to avoid redundant updates
 *          - Detection counter reset for clean state transitions
 *          - Dataflash logging of bottom contact events for analysis
 *          
 *          The bottom contact state is used by:
 *          - Landing mode to confirm successful touchdown
 *          - Altitude hold to prevent terrain collision
 *          - Failsafe systems to detect stuck/grounded conditions
 *          - Autonomous missions for bottom sampling or inspection tasks
 *          - Post-flight analysis to verify landing procedures
 *          
 *          State Transition Logging:
 *          - BOTTOMED event: Vehicle has made contact with seafloor/terrain
 *          - NOT_BOTTOMED event: Vehicle has lifted off from bottom
 *          
 * @param[in] at_bottom  True if vehicle is on bottom/terrain, false if free-floating
 * 
 * @note This function includes state change detection optimization - if the
 *       requested state matches current state, function returns immediately
 *       without logging or counter reset.
 * 
 * @note The bottom_detector_count is reset to zero on every state transition
 *       to ensure clean detection cycles for the next state change.
 * 
 * @warning Bottom contact detection is critical for preventing terrain collisions
 *          and enabling safe landing operations. False negatives could result in
 *          continued descent into obstacles or seafloor with potential vehicle damage.
 * 
 * @warning In strong currents or on sloped terrain, bottom contact may be intermittent.
 *          The time-based detection threshold (BOTTOM_DETECTOR_TRIGGER_SEC) provides
 *          debouncing for stable state determination.
 * 
 * @see update_surface_and_bottom_detector() for detection algorithm
 * @see LogEvent::BOTTOMED and LogEvent::NOT_BOTTOMED for log message definitions
 * 
 * Source: ArduSub/surface_bottom_detector.cpp:99-115
 */
void Sub::set_bottomed(bool at_bottom)
{

    if (ap.at_bottom == at_bottom) { // do nothing if state unchanged
        return;
    }

    ap.at_bottom = at_bottom;

    bottom_detector_count = 0;

    if (ap.at_bottom) {
        LOGGER_WRITE_EVENT(LogEvent::BOTTOMED);
    } else {
        LOGGER_WRITE_EVENT(LogEvent::NOT_BOTTOMED);
    }
}
