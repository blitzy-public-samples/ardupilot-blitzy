/**
 * @file turn_counter.cpp
 * @brief Turn counter for tracking cumulative vehicle rotation
 * 
 * @details This file implements a turn counting system specifically designed for
 *          tethered Remotely Operated Vehicles (ROVs) to track the total number
 *          of rotations the vehicle has made about its vertical axis. This is
 *          critical for tether management, as excessive rotation in one direction
 *          can cause the tether to twist, tangle, or become damaged.
 * 
 *          The turn counter uses a quadrant-based state machine to detect when
 *          the vehicle crosses between 90-degree sectors of rotation, accumulating
 *          quarter-turn increments to maintain a running total of rotation. This
 *          allows operators to:
 *          - Monitor total vehicle rotation in real-time
 *          - Implement automatic tether management strategies
 *          - Set alerts when rotation limits are approached
 *          - Command the vehicle to unwind by rotating in the opposite direction
 * 
 *          Algorithm Overview:
 *          The yaw angle range [-180°, +180°) is divided into four quadrants:
 *          - Quadrant 0: [0°, 90°)
 *          - Quadrant 1: [90°, 180°)
 *          - Quadrant 2: [-180°, -90°)
 *          - Quadrant 3: [-90°, 0°)
 * 
 *          The algorithm detects transitions between adjacent quadrants and
 *          increments or decrements a quarter-turn counter accordingly. Clockwise
 *          (right) turns increment the counter, while counter-clockwise (left)
 *          turns decrement it. By tracking these transitions across the angle
 *          wrap boundary at ±180°, the system maintains an accurate cumulative
 *          rotation count regardless of how many full rotations have occurred.
 * 
 * @note This implementation assumes the vehicle primarily rotates about the yaw
 *       axis. Complex 3D rotations may not be accurately represented by this
 *       single-axis tracking approach.
 * 
 * @warning Sudden large yaw changes (e.g., from GPS heading resets or compass
 *          calibration) may cause incorrect turn counts. Ensure AHRS is stable
 *          before relying on turn counter data for tether management.
 * 
 * @author Rustom Jehangir (rusty@bluerobotics.com)
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: ArduSub/turn_counter.cpp:1-58
 */

#include "Sub.h"

/**
 * @brief Update the turn counter based on current vehicle yaw
 * 
 * @details This function implements the core turn counting algorithm by tracking
 *          transitions between yaw angle quadrants. Called periodically (typically
 *          at the main loop rate), it compares the current yaw quadrant against
 *          the previous quadrant to detect rotation direction and accumulate
 *          quarter-turn increments.
 * 
 *          The algorithm works by:
 *          1. Determining which quadrant (0-3) the current yaw angle falls into
 *          2. Comparing current quadrant to previous quadrant (last_turn_state)
 *          3. Detecting valid transitions to adjacent quadrants
 *          4. Incrementing quarter_turn_count for clockwise (right) transitions
 *          5. Decrementing quarter_turn_count for counter-clockwise (left) transitions
 *          6. Storing current quadrant as last_turn_state for next iteration
 * 
 *          Angle Wrap Handling:
 *          The algorithm correctly handles the angle discontinuity at ±180° by
 *          treating the transition between Quadrant 1 (90° to 180°) and Quadrant 2
 *          (-180° to -90°) as a valid quarter-turn boundary. This allows continuous
 *          rotation tracking across the wrap point without reset or error.
 * 
 *          Quarter-Turn Accumulation:
 *          - quarter_turn_count increments by 1 for each 90° clockwise rotation
 *          - quarter_turn_count decrements by 1 for each 90° counter-clockwise rotation
 *          - Full 360° clockwise rotation results in quarter_turn_count += 4
 *          - Counter value can be positive (net clockwise) or negative (net counter-clockwise)
 *          - Value range is unlimited, accumulating over entire mission duration
 * 
 *          Use Cases:
 *          - Tether Management: Alert operator when |quarter_turn_count| exceeds threshold
 *          - Automatic Unwinding: Command vehicle to rotate opposite direction to reduce count
 *          - Mission Planning: Avoid flight paths requiring excessive rotation
 *          - Post-Flight Analysis: Review rotation patterns for operational insights
 * 
 * @note This function should be called at a consistent rate (main loop frequency)
 *       to ensure no quadrant transitions are missed. Calling frequency must be
 *       high enough relative to maximum yaw rate to detect all transitions.
 * 
 * @note The function relies on Sub class member variables:
 *       - quarter_turn_count: Cumulative quarter-turn counter (int16_t or similar)
 *       - last_turn_state: Previous quadrant state (uint8_t, values 0-3)
 * 
 * @warning Rapid yaw changes faster than the update rate may cause missed
 *          transitions and inaccurate turn counts. Ensure scheduler calls this
 *          function frequently enough for expected yaw rates.
 * 
 * @warning AHRS yaw resets (e.g., compass calibration, GPS heading initialization)
 *          will cause spurious turn count changes. Consider resetting quarter_turn_count
 *          when AHRS is reinitialized.
 * 
 * @see Sub::quarter_turn_count
 * @see Sub::last_turn_state
 * @see AP_AHRS::get_yaw_rad()
 */
void Sub::update_turn_counter()
{
    // Determine current quadrant state based on yaw angle
    // The yaw range [-180°, +180°) is divided into four 90-degree quadrants:
    // State 0: [0°, 90°)     - Forward right quadrant
    // State 1: [90°, 180°)   - Backward right quadrant
    // State 2: [-180°, -90°) - Backward left quadrant (wraps from +180°)
    // State 3: [-90°, 0°)    - Forward left quadrant
    //
    // This division creates natural boundaries at 0°, ±90°, and ±180° which
    // are used to detect quarter-turn transitions in the state machine below.
    uint8_t turn_state;
    if (ahrs.get_yaw_rad() >= 0.0f && ahrs.get_yaw_rad() < radians(90)) {
        turn_state = 0;  // [0°, 90°): Forward right
    } else if (ahrs.get_yaw_rad() >= radians(90)) {
        turn_state = 1;  // [90°, 180°): Backward right
    } else if (ahrs.get_yaw_rad() < -radians(90)) {
        turn_state = 2;  // [-180°, -90°): Backward left
    } else {
        turn_state = 3;  // [-90°, 0°): Forward left
    }

    // Detect transitions between adjacent quadrants to count quarter-turns.
    // Only transitions to neighboring quadrants are counted; direct jumps across
    // multiple quadrants are ignored to prevent false counts from AHRS resets.
    //
    // Clockwise (right turn) transitions increment the counter:
    //   0→1 (0° to 90° crossing), 1→2 (±180° wrap crossing),
    //   2→3 (-90° crossing), 3→0 (0° crossing)
    //
    // Counter-clockwise (left turn) transitions decrement the counter:
    //   0→3 (0° crossing), 3→2 (-90° crossing),
    //   2→1 (±180° wrap crossing), 1→0 (90° crossing)
    //
    // This state machine correctly handles the angle wrap at ±180° by treating
    // the 1↔2 transition as a valid quarter-turn boundary.
    switch (last_turn_state) {
    case 0:  // Was in forward right quadrant [0°, 90°)
        if (turn_state == 1) {
            quarter_turn_count++;  // Crossed 90° boundary clockwise (right turn)
        }
        if (turn_state == 3) {
            quarter_turn_count--;  // Crossed 0° boundary counter-clockwise (left turn)
        }
        break;
    case 1:  // Was in backward right quadrant [90°, 180°)
        if (turn_state == 2) {
            quarter_turn_count++;  // Crossed ±180° wrap boundary clockwise (right turn)
        }
        if (turn_state == 0) {
            quarter_turn_count--;  // Crossed 90° boundary counter-clockwise (left turn)
        }
        break;
    case 2:  // Was in backward left quadrant [-180°, -90°)
        if (turn_state == 3) {
            quarter_turn_count++;  // Crossed -90° boundary clockwise (right turn)
        }
        if (turn_state == 1) {
            quarter_turn_count--;  // Crossed ±180° wrap boundary counter-clockwise (left turn)
        }
        break;
    case 3:  // Was in forward left quadrant [-90°, 0°)
        if (turn_state == 0) {
            quarter_turn_count++;  // Crossed 0° boundary clockwise (right turn)
        }
        if (turn_state == 2) {
            quarter_turn_count--;  // Crossed -90° boundary counter-clockwise (left turn)
        }
        break;
    }
    
    // Store current quadrant for comparison on next update cycle
    last_turn_state = turn_state;
}
