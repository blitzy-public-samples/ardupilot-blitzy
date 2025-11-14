/**
 * @file mode_auto.cpp
 * @brief Auto mode implementation for antenna tracker
 * 
 * @details Automatically tracks vehicle position using MAVLink telemetry, with 
 *          fallback to scan mode if vehicle position unavailable. This is the 
 *          primary operational mode for antenna trackers, combining intelligent
 *          tracking and search capabilities.
 */

#include "mode.h"

#include "Tracker.h"

/**
 * @brief Updates auto tracking mode with intelligent fallback behavior
 * 
 * @details Called at 50Hz while in AUTO mode. Implements three-tier behavior:
 *          1. If vehicle location valid: track vehicle using update_auto()
 *          2. Else if target previously set OR scan-on-lost-vehicle enabled: search via update_scan()
 *          3. Else: hold current position (no update)
 *          
 *          The vehicle location validity is determined by recent MAVLink GLOBAL_POSITION_INT
 *          messages, with a timeout threshold defined by TRACKING_TIMEOUT_MS.
 * 
 * @note Auto mode is primary operational mode, combining tracking and search capabilities
 * @note Vehicle location validity determined by recent MAVLink telemetry (timeout: TRACKING_TIMEOUT_MS)
 * 
 * @see Mode::update_auto() - Performs actual tracking calculations (bearing, distance, pitch)
 * @see Mode::update_scan() - Implements scanning pattern to reacquire vehicle
 */
void ModeAuto::update()
{
    // Check tracker.vehicle.location_valid flag (set by MAVLink GLOBAL_POSITION_INT messages)
    if (tracker.vehicle.location_valid) {
        // Perform actual tracking calculations (bearing, distance, pitch)
        update_auto();
    } else if (tracker.target_set || (tracker.g.auto_opts.get() & (1 << 0)) != 0) {
        // Target was ever set OR auto_opts bit 0 enables scan-on-lost
        // Falls back to scanning pattern to reacquire vehicle
        update_scan();
    }
    // Implicit else: hold current position (no update)
}
