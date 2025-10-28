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
 * @file Heat_Pwm.h
 * @brief PWM-based IMU heater control implementation for Linux
 * 
 * Provides thermal management for IMU sensors using sysfs PWM output to control
 * heating elements. Implements PID-style temperature regulation to maintain stable
 * sensor operating temperature for improved accuracy and reduced bias drift.
 */

#pragma once

#include "AP_HAL_Linux.h"
#include "PWM_Sysfs.h"
#include "Heat.h"

namespace Linux {

/**
 * @class Linux::HeatPwm
 * @brief PWM-controlled IMU heater with PI temperature control
 * 
 * @details Implements closed-loop temperature control for IMU thermal management using
 *          Linux sysfs PWM interface. Maintains target temperature via PI (Proportional-
 *          Integral) control algorithm to reduce IMU bias drift caused by temperature
 *          variations.
 *          
 *          Temperature control strategy:
 *          - Proportional term (Kp): Responds to current temperature error
 *          - Integral term (Ki): Eliminates steady-state error
 *          - PWM duty cycle adjusted based on PI output
 *          - Update rate typically 10-100Hz to balance responsiveness and stability
 *          
 *          Typical use case: High-precision navigation systems where IMU thermal stability
 *          is critical. Common on indoor/surveying applications where ambient temperature
 *          varies significantly.
 *          
 *          Hardware requirements:
 *          - Sysfs PWM channel connected to heating element (resistor or film heater)
 *          - Temperature sensor integrated in IMU or nearby thermistor
 *          - Adequate power budget for heater (typically 1-5W)
 * 
 * @note Heating increases power consumption - balance precision vs battery life
 * @warning Excessive heating may damage IMU - verify thermal limits from datasheet
 * @see Heat base class for interface definition
 * @see PWM_Sysfs for Linux PWM sysfs interface
 */
class HeatPwm : public Heat {
public:
    /**
     * @brief Initialize PWM heater with PI control parameters
     * 
     * @param[in] pwm_num Linux sysfs PWM channel number (chip:channel identifier)
     * @param[in] Kp Proportional gain for temperature control (typical: 10-100)
     * @param[in] Ki Integral gain for temperature control (typical: 0.1-10)
     * @param[in] period_ns PWM period in nanoseconds (typical: 1000000 = 1kHz)
     * 
     * @note Kp/Ki tuning affects temperature stability and settling time
     * @note Higher Kp = faster response but more overshoot
     * @note Ki eliminates steady-state error but too high causes oscillation
     * @note PWM period affects heater smoothness - 500Hz-2kHz typical for thermal mass
     */
    HeatPwm(uint8_t pwm_num, float Kp, float Ki,
            uint32_t period_ns);
    
    /**
     * @brief Update current IMU temperature reading
     * 
     * @param[in] current Current IMU temperature in degrees Celsius
     * 
     * @details Calculates temperature error and updates PI controller to adjust PWM duty
     *          cycle. Called periodically (typically 10-100Hz) from scheduler task.
     *          
     *          Control algorithm:
     *          1. error = target - current
     *          2. sum_error += error * dt (integral term)
     *          3. output = Kp * error + Ki * sum_error
     *          4. Clamp output to 0-100% duty cycle
     *          5. Write duty cycle to PWM sysfs
     * 
     * @note Should be called at consistent rate for stable control
     * @warning Large temperature jumps may indicate sensor failure
     */
    void set_imu_temp(float current) override;
    
    /**
     * @brief Set target IMU temperature for control
     * 
     * @param[in] target Pointer to target temperature in degrees Celsius
     * 
     * @note Pointer allows dynamic target adjustment via parameter changes
     * @note Typical IMU target temperatures: 45-65°C depending on sensor type
     * @warning Target must be above ambient to enable heating
     * @warning Do not exceed IMU maximum operating temperature (usually 85°C)
     */
    void set_imu_target_temp(int8_t *target) override;

private:
    /**
     * PWM output interface for heater control via Linux sysfs
     */
    PWM_Sysfs_Base *_pwm;
    
    /**
     * Timestamp of last temperature update in milliseconds
     * Used to calculate dt for integral term
     */
    uint32_t _last_temp_update = 0;
    
    /**
     * Proportional gain for PI controller
     */
    float _Kp;
    
    /**
     * Integral gain for PI controller
     */
    float _Ki;
    
    /**
     * PWM period in nanoseconds (determines PWM frequency)
     */
    uint32_t _period_ns;
    
    /**
     * Accumulated integral error for PI controller
     * Reset on target change or saturation
     */
    float _sum_error;
    
    /**
     * Pointer to target temperature parameter
     * Allows runtime target adjustment without reboot
     */
    int8_t *_target = nullptr;
};

}
