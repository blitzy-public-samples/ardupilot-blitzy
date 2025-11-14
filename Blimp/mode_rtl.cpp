/**
 * @file mode_rtl.cpp
 * @brief RTL (Return to Launch) mode implementation for airships
 * 
 * @details This file implements the Return-to-Launch flight mode for Blimp vehicles.
 *          RTL mode navigates the blimp back to the home position (0,0,0 in NED frame)
 *          using position control, providing a safe autonomous return capability for
 *          lighter-than-air vehicles. Unlike heavier-than-air vehicles, airships benefit
 *          from inherent buoyancy which makes RTL a particularly safe failsafe mode.
 * 
 * @note This is a safety-critical return mode used in failsafe scenarios
 */

#include "Blimp.h"

/**
 * @brief Position lag limit in seconds
 * 
 * @details Defines the number of seconds of movement that the target position can be
 *          ahead of the actual position. This constant matches the loiter mode constant
 *          for consistency across flight modes.
 * 
 * @note Currently defined but not actively used in this simple RTL implementation
 */
#define POS_LAG 1

/**
 * @brief Initialize RTL (Return to Launch) mode
 * 
 * @details Performs minimal initialization required for RTL mode. The mode assumes
 *          the home position is at the NED origin (0,0,0) and requires no complex
 *          setup or pre-flight checks specific to RTL operation.
 * 
 * @param[in] ignore_checks Standard mode initialization parameter (currently unused)
 * 
 * @return Always returns true indicating successful initialization
 * 
 * @note The home position is assumed to be at NED origin (0,0,0)
 */
bool ModeRTL::init(bool ignore_checks)
{
    return true;
}

/**
 * @brief Main RTL controller - executes the return to launch behavior
 * 
 * @details This function implements the core Return-to-Launch algorithm for airships:
 *          
 *          Algorithm:
 *          1. Sets target position to NED origin (0,0,0) representing home/launch location
 *          2. Sets target yaw to 0 (north-facing orientation)
 *          3. Delegates position control to the loiter controller to navigate to home
 *          
 *          The Vector4b parameter passed to loiter->run() contains four boolean flags
 *          controlling velocity feedforward behavior [x_vel_ff, y_vel_ff, z_vel_ff, yaw_ff].
 *          All are set to false for RTL, meaning no velocity feedforward is applied and
 *          the controller relies purely on position error feedback.
 *          
 *          For lighter-than-air vehicles like blimps, RTL is inherently safer than for
 *          heavier-than-air vehicles due to buoyancy - the vehicle will naturally maintain
 *          altitude even if propulsion fails during the return journey.
 * 
 * @note Called at the main loop rate (typically 50-100Hz depending on scheduler configuration)
 * @warning This is a safety-critical return mode used in failsafe scenarios including
 *          RC loss, battery failsafe, and geofence breach. The home position must be
 *          properly set before flight to ensure safe return.
 * 
 * Source: Blimp/mode_rtl.cpp:15-20
 */
void ModeRTL::run()
{
    Vector3f target_pos = {0,0,0};
    float target_yaw = 0;
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
