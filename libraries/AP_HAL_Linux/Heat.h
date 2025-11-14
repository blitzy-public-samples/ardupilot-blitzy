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
 * @file Heat.h
 * @brief Abstract interface for IMU thermal management on Linux platforms
 * 
 * Defines the interface for IMU heater control implementations. Concrete implementations
 * (e.g., HeatPwm) provide platform-specific heater control strategies.
 */

#pragma once

namespace Linux {

/**
 * @class Linux::Heat
 * @brief Abstract base class for IMU temperature control
 * 
 * @details Provides interface for maintaining stable IMU operating temperature to reduce
 *          sensor bias drift caused by thermal variations. Temperature stability is critical
 *          for high-precision navigation where IMU bias directly affects position accuracy.
 *          
 *          Why IMU heating matters:
 *          - Gyroscope bias varies with temperature (typical: 0.01-0.1 deg/s per °C)
 *          - Accelerometer bias varies with temperature (typical: 1-10 mg per °C)
 *          - Stable temperature = stable bias = better EKF performance
 *          - Important for surveying, mapping, indoor navigation
 *          
 *          Implementation strategies:
 *          - PWM heater: Most common, uses resistive heating element
 *          - GPIO on/off: Simple bang-bang control for low-precision needs
 *          - External thermal management: Dedicated temperature controller IC
 *          
 *          Lifecycle:
 *          1. Initialize heater hardware during HAL init
 *          2. Register temperature update callback with scheduler (10-100Hz)
 *          3. Continuously monitor IMU temp via set_imu_temp()
 *          4. Adjust heater output to maintain target temperature
 *          5. Target temperature configurable via parameter (set_imu_target_temp)
 * 
 * @note Default implementation does nothing - safe for boards without heater
 * @note Heating increases power consumption - consider battery life impact
 * @see HeatPwm for PWM-based heater implementation
 */
class Heat {
public:
    /**
     * @brief Update current IMU temperature reading
     * 
     * @param[in] current Current IMU temperature in degrees Celsius
     * 
     * @note Called periodically by temperature monitoring task
     * @note Default implementation does nothing (no-op for boards without heater)
     * @note Override in derived class to implement actual temperature control
     */
    virtual void set_imu_temp(float current) { }
    
    /**
     * @brief Set target IMU temperature for control loop
     * 
     * @param[in] target Pointer to target temperature in degrees Celsius
     * 
     * @note Pointer allows runtime target changes via parameter updates
     * @note Default implementation does nothing (no-op for boards without heater)
     * @note Override in derived class to implement actual temperature control
     * @note Typical targets: 45-65°C depending on IMU specifications
     */
    virtual void set_imu_target_temp(int8_t *target) { }
};

}
