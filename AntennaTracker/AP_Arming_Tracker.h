#pragma once

/**
 * @file AP_Arming_Tracker.h
 * @brief Arming checks for antenna tracker
 * 
 * @details This file defines the antenna tracker-specific arming check
 *          implementation. While the tracker does not implement custom
 *          arming checks beyond the base AP_Arming class, this derived
 *          class is required for proper singleton instantiation within
 *          the ArduPilot architecture.
 * 
 *          The AntennaTracker vehicle uses minimal arming checks compared
 *          to flight vehicles since it is a ground-based tracking platform
 *          with no flight safety implications. The base AP_Arming class
 *          provides sufficient safety validation for servo movement.
 * 
 * @note This class exists primarily to prevent nullptr returns from the
 *       arming singleton accessor in the tracker vehicle code.
 * 
 * @see AP_Arming Base arming check implementation
 * @see Tracker Main tracker vehicle class
 */

#include <AP_Arming/AP_Arming.h>

/**
 * @class AP_Arming_Tracker
 * @brief Tracker-specific arming check implementation
 * 
 * @details This class extends the base AP_Arming class to provide arming
 *          functionality for the AntennaTracker vehicle. Unlike flight vehicles
 *          (Copter, Plane, Rover, Sub), the tracker does not require extensive
 *          pre-arm safety checks since it is a ground-based servo platform.
 * 
 *          The tracker inherits all base arming checks from AP_Arming without
 *          overriding any methods. Base checks include:
 *          - Battery voltage and capacity checks
 *          - Board voltage checks
 *          - Sensor health monitoring
 *          - Parameter validation
 *          - System health verification
 * 
 *          Architecture Note:
 *          This class serves as a vehicle-specific arming implementation
 *          required by the ArduPilot singleton pattern. Each vehicle type
 *          must provide its own AP_Arming-derived class, even if it doesn't
 *          add custom functionality beyond the base implementation.
 * 
 * @note The tracker vehicle has minimal arming requirements:
 *       - Basic system health (battery, sensors)
 *       - Valid servo output configuration
 *       - No GPS lock or complex state estimation required
 * 
 * @warning Even though arming checks are minimal, operators should verify
 *          that tracker servos are mechanically free to move and that the
 *          antenna system is properly balanced before arming.
 * 
 * @see AP_Arming Base arming check class with common safety validations
 * @see Tracker Vehicle class that uses this arming implementation
 */
// this class isn't actually used by Tracker; it's really just here so
// the singleton doesn't come back as nullptr
class AP_Arming_Tracker : public AP_Arming
{
public:
    /**
     * @brief Grant Tracker vehicle class access to private/protected members
     * 
     * @details The Tracker vehicle class requires friend access to initialize
     *          and manage the arming subsystem. This follows the standard
     *          ArduPilot pattern where the main vehicle class has privileged
     *          access to its subsystem implementations.
     */
    friend class Tracker;

private:
    // No private members or overrides - all functionality inherited from AP_Arming base class

};
