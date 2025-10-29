/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_OpticalFlow_Onboard.h
 * @brief HAL-provided onboard optical flow driver backend
 * 
 * This backend provides an interface to platform-specific optical flow implementations
 * available through the AP_HAL hardware abstraction layer. It is typically used on
 * specialized hardware with integrated optical flow capabilities, such as certain Linux
 * boards with camera-based flow processing.
 * 
 * Communication: Direct HAL interface via hal.opticalflow
 * Platforms: Currently used on specialized hardware with integrated optical flow support
 * 
 * @note By default, AP_OPTICALFLOW_ONBOARD_ENABLED=0 (disabled on most platforms)
 * @warning Requires platform-specific HAL optical flow driver implementation
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_ONBOARD_ENABLED

#include <AP_OpticalFlow/AP_OpticalFlow_Backend.h>

#include "AP_OpticalFlow.h"

/**
 * @class AP_OpticalFlow_Onboard
 * @brief HAL-provided onboard optical flow backend
 * 
 * @details This backend interfaces with the HAL optical flow driver (hal.opticalflow) to read
 *          optical flow data from platform-specific implementations. The driver reads Data_Frame
 *          structures containing integral motion data (pixel displacement * integration time),
 *          converts microsecond-based integrals to rad/s rates, and enforces a 10Hz minimum
 *          update rate via _last_read_ms gating to avoid excessive CPU usage.
 * 
 *          Data Flow:
 *          - HAL provides pixel integrals multiplied by microseconds
 *          - Conversion: Divide by integration_time_us to get pixel/s
 *          - Multiply by scaling factor to get rad/s
 *          - Output flow rates in rad/s for frontend consumption
 * 
 *          Coordinate frames: HAL provides sensor body frame; backend may apply orientation
 * 
 *          Platform Availability:
 *          - Default AP_OPTICALFLOW_ONBOARD_ENABLED=0 (disabled on most platforms)
 *          - Requires platform HAL to implement opticalflow interface
 *          - Used on specific Linux boards with integrated camera flow processing
 * 
 * @note Rate limited to 10Hz (100ms minimum interval) to avoid excessive CPU usage
 * @warning Requires platform-specific HAL optical flow driver implementation
 * @warning Typically disabled unless platform provides driver
 */
class AP_OpticalFlow_Onboard : public OpticalFlow_backend
{
public:
    /**
     * @brief Constructor inherits from OpticalFlow_backend base class
     * 
     * Uses the base class constructor to initialize the optical flow backend
     * with the required OpticalFlow reference and sensor state.
     */
    using OpticalFlow_backend::OpticalFlow_backend;

    /**
     * @brief Initialize HAL optical flow driver
     * 
     * Attempts to initialize the platform-specific HAL optical flow driver
     * via hal.opticalflow. If the HAL provides an optical flow implementation,
     * it will be enabled and made ready for reading.
     * 
     * @note This method is called once during sensor initialization
     * @warning If HAL does not implement opticalflow interface, this will have no effect
     */
    void init(void) override;

    /**
     * @brief Read optical flow data from HAL and update frontend
     * 
     * @details Reads data from hal.opticalflow->read(Data_Frame&) and processes it:
     *          1. Enforce 10Hz rate limiting using _last_read_ms gating (100ms minimum interval)
     *          2. Read Data_Frame containing integral pixel motion * microseconds
     *          3. Convert integral (pixel*microseconds) to rates (rad/s):
     *             - Divide by integration_time_us to get pixel/s
     *             - Multiply by scaling factor to get rad/s
     *          4. Update frontend via _update_frontend() with converted rates
     * 
     *          Units:
     *          - Input: pixel displacement integrals * microseconds
     *          - Output: rad/s flow rates
     * 
     *          Update Rate: Maximum 10Hz (enforced by 100ms minimum interval)
     * 
     * @note Called regularly by the optical flow library's update loop
     * @warning Rate limiting prevents updates faster than 10Hz
     */
    void update(void) override;

private:
    /**
     * @brief Last read timestamp in milliseconds for 10Hz rate gating
     * 
     * Stores the timestamp of the last successful read from the HAL optical flow
     * driver. Used to enforce a minimum 100ms interval between reads, limiting
     * the update rate to 10Hz to avoid excessive CPU usage.
     * 
     * Units: milliseconds (AP_HAL::millis())
     */
    uint32_t _last_read_ms;
};

#endif  // AP_OPTICALFLOW_ONBOARD_ENABLED
