/**
 * @file mode_scan.cpp
 * @brief Scan mode implementation for antenna tracker
 * 
 * @details Implements scanning pattern to search for vehicle by sweeping 
 *          through yaw and pitch ranges. This mode is useful when the 
 *          vehicle position is unknown and the tracker needs to perform
 *          a systematic search pattern.
 */

#include "mode.h"

/**
 * @brief Updates scan mode by executing scanning pattern
 * 
 * @details Called at 50Hz while in SCAN mode. Delegates to Mode::update_scan() 
 *          which implements the scanning algorithm that sweeps through configured 
 *          yaw/pitch ranges to search for vehicle.
 *          
 *          The scanning pattern systematically searches the configured angular
 *          ranges to locate and acquire the target vehicle.
 * 
 * @note Scan mode useful for finding lost vehicles or initial acquisition
 * 
 * @see Mode::update_scan()
 * 
 * Source: AntennaTracker/mode_scan.cpp:3-6
 */
void ModeScan::update()
{
    update_scan();
}
