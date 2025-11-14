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
 * @file OpticalFlow.h
 * @brief Optical flow sensor hardware abstraction layer interface
 * 
 * Defines abstract interface for optical flow sensors that measure ground velocity
 * by tracking surface features between consecutive camera frames. Used in conjunction
 * with rangefinder for GPS-denied navigation, precision position hold, and velocity
 * estimation in indoor or GPS-compromised environments.
 * 
 * @note Most optical flow implementations use the AP_OpticalFlow library which builds
 *       on this HAL interface. This provides platform-specific hardware abstraction.
 * @note Requires rangefinder data for velocity scaling: velocity = flow_rate × altitude
 * 
 * Source: libraries/AP_HAL/OpticalFlow.h
 */

#pragma once

/**
 * @class AP_HAL::OpticalFlow
 * @brief Abstract hardware interface for optical flow sensors
 * 
 * @details Optical flow sensors measure apparent motion of ground features between
 *          consecutive image frames to estimate horizontal velocity. This HAL interface
 *          abstracts platform-specific optical flow hardware implementations.
 *          
 *          **Common Optical Flow Sensors**:
 *          - PX4Flow: Integrated camera + IMU + sonar ranging
 *          - Cheerson CX-OF: Low-cost I2C optical flow sensor
 *          - CXOF: Serial optical flow sensor
 *          - MAVLink flow: External flow computer via MAVLink protocol
 *          
 *          **Flow Measurement Principles**:
 *          - Sensors output X/Y pixel motion (optical flow) between frames
 *          - Velocity conversion: flow_rate (rad/s) × altitude (m) = velocity (m/s)
 *          - Requires accurate altitude from rangefinder or barometer
 *          - Quality metric indicates surface texture suitability (0-255)
 *          
 *          **Integration with ArduPilot**:
 *          - Flow data processed by AP_OpticalFlow library with sensor-specific calibration
 *          - Velocity estimates fused into EKF (Extended Kalman Filter) for position control
 *          - Used for loiter, position hold, and autonomous modes in GPS-denied environments
 *          - Works best over textured surfaces at 1-5m altitude range
 *          
 *          **Coordinate Frames**:
 *          - Flow X/Y measured in sensor body frame
 *          - Gyro compensation accounts for vehicle rotation during frame capture
 *          - AP_OpticalFlow rotates flow to body frame using FLOW_ORIENT_YAW parameter
 * 
 * @note Flow quality degrades over smooth/uniform surfaces (calm water, blank concrete)
 * @note Works poorly in low light conditions or at excessive altitudes (>10m typically)
 * @warning Altitude measurement critical - incorrect altitude causes proportional velocity errors
 * @warning Flow data alone cannot determine position - only velocity. Requires integration over time.
 * 
 * Source: libraries/AP_HAL/OpticalFlow.h
 */
class AP_HAL::OpticalFlow {
public:
    /**
     * @class Data_Frame
     * @brief Optical flow measurement data structure
     * 
     * @details Contains integrated optical flow and gyro measurements over a time period.
     *          Integrals accumulated between successive read() calls to account for variable
     *          frame rates and processing latencies.
     *          
     *          **Data Processing**:
     *          - Pixel flow integrals represent accumulated apparent motion in image plane
     *          - Gyro integrals used for motion compensation (removes vehicle rotation from flow)
     *          - Quality indicates surface feature detection confidence
     *          - Delta time allows rate calculation: flow_rate = integral / delta_time
     *          
     *          **Gyro Compensation**:
     *          Optical flow includes both:
     *          1. Ground motion (desired velocity signal)
     *          2. Camera rotation (from vehicle attitude changes - must be removed)
     *          
     *          Gyro integrals subtracted to isolate ground motion:
     *          compensated_flow = pixel_flow - (gyro_integral × focal_length_factor)
     * 
     * @note All integral values accumulate between frames and reset after read()
     * @note Quality threshold typically 0-255, with >100 considered good tracking
     */
    class Data_Frame {
    public:
        /**
         * @brief Integrated optical flow in X direction
         * 
         * Accumulated pixel motion in sensor X-axis (typically forward) over delta_time.
         * Units: radians (flow angular displacement integrated over time period)
         * 
         * @note Positive X typically corresponds to forward motion
         * @note Includes both ground motion and rotational motion - requires gyro compensation
         */
        float pixel_flow_x_integral;
        
        /**
         * @brief Integrated optical flow in Y direction
         * 
         * Accumulated pixel motion in sensor Y-axis (typically right) over delta_time.
         * Units: radians (flow angular displacement integrated over time period)
         * 
         * @note Positive Y typically corresponds to rightward motion
         * @note Includes both ground motion and rotational motion - requires gyro compensation
         */
        float pixel_flow_y_integral;
        
        /**
         * @brief Integrated gyroscope X-axis rotation
         * 
         * Accumulated gyro rotation about sensor X-axis over delta_time period.
         * Used to compensate pixel_flow_x_integral for vehicle pitch rotation.
         * Units: radians
         * 
         * @note Gyro data from onboard IMU or flow sensor's integrated gyro
         * @note Subtracted from flow to isolate ground motion
         */
        float gyro_x_integral;
        
        /**
         * @brief Integrated gyroscope Y-axis rotation
         * 
         * Accumulated gyro rotation about sensor Y-axis over delta_time period.
         * Used to compensate pixel_flow_y_integral for vehicle roll rotation.
         * Units: radians
         * 
         * @note Gyro data from onboard IMU or flow sensor's integrated gyro
         * @note Subtracted from flow to isolate ground motion
         */
        float gyro_y_integral;
        
        /**
         * @brief Integration time period
         * 
         * Time duration over which flow and gyro integrals were accumulated.
         * Units: microseconds (μs)
         * 
         * @note Used to calculate rates: flow_rate = integral / delta_time
         * @note Typical values: 10,000-100,000μs (10-100ms) depending on sensor frame rate
         */
        uint32_t delta_time;
        
        /**
         * @brief Flow measurement quality indicator
         * 
         * Surface feature tracking quality metric from flow sensor.
         * Range: 0 (no features/poor tracking) to 255 (excellent tracking)
         * 
         * Quality affected by:
         * - Surface texture (higher for detailed textures, lower for uniform surfaces)
         * - Lighting conditions (degrades in low light)
         * - Altitude (degrades when too high or too low)
         * - Focus (some sensors have fixed focal distance)
         * 
         * @note Quality threshold for accepting measurements typically >100
         * @note AP_OpticalFlow library uses quality to weight EKF fusion
         * @warning Low quality (<50) indicates unreliable flow data - should not be used for navigation
         */
        uint8_t quality;
    };

    /**
     * @brief Initialize optical flow sensor hardware
     * 
     * @details Performs platform-specific initialization of optical flow sensor:
     *          - Configure I2C/SPI/UART communication interface
     *          - Power on sensor and verify communication
     *          - Configure sensor parameters (frame rate, resolution, exposure)
     *          - Verify sensor firmware version
     *          - Initialize sensor coordinate frame orientation
     *          
     *          Called once during AP_HAL initialization before sensor is used.
     *          
     * @note Typically called from AP_OpticalFlow::init() during vehicle startup
     * @note Initialization failure should be logged but not halt vehicle boot
     * @note Some sensors require warm-up time before providing valid data
     * 
     * Source: libraries/AP_HAL/OpticalFlow.h:29
     */
    virtual void init() = 0;
    
    /**
     * @brief Read latest optical flow measurement data
     * 
     * @details Retrieves accumulated flow and gyro integral data from sensor since last read.
     *          Data includes pixel flow integrals, gyro compensation integrals, integration
     *          time period, and quality metric. Integrals reset after each successful read.
     *          
     *          **Usage Pattern**:
     *          1. Called periodically (typically 10-50Hz) by AP_OpticalFlow library
     *          2. If new data available (returns true), process Data_Frame
     *          3. Calculate flow rates: rate = integral / delta_time
     *          4. Apply gyro compensation and sensor calibration
     *          5. Convert to body-frame velocity: velocity = flow_rate × altitude
     *          6. Provide to EKF for sensor fusion
     *          
     *          **Return Value Interpretation**:
     *          - true: New data available in frame, integrals have been updated
     *          - false: No new data since last read (frame contents unchanged)
     * 
     * @param[out] frame Data structure to populate with latest flow measurements
     * 
     * @return true if new flow data available and frame updated, false if no new data
     * 
     * @note Frame data unchanged if return value is false
     * @note Quality field should be checked before using flow data for navigation
     * @note Some sensors buffer multiple frames - may return true on successive calls
     * @warning Do not use flow data if quality < minimum threshold (typically 100)
     * 
     * Source: libraries/AP_HAL/OpticalFlow.h:30
     */
    virtual bool read(Data_Frame& frame) = 0;
    
    /**
     * @brief Provide gyroscope data for flow motion compensation
     * 
     * @details Pushes gyro rotation rates from vehicle IMU to flow sensor for improved
     *          motion compensation. Some flow sensors have integrated gyros, but using
     *          vehicle IMU gyros (typically higher quality) improves compensation accuracy.
     *          
     *          **Motion Compensation Process**:
     *          Optical flow measures both:
     *          1. Ground motion relative to camera (desired signal)
     *          2. Camera rotation (vehicle attitude changes - undesired)
     *          
     *          Gyro data subtracts camera rotation to isolate ground motion:
     *          ground_flow = measured_flow - (gyro_rotation × camera_geometry)
     *          
     *          **Integration**:
     *          - Gyro rates integrated over dt to match flow integral measurements
     *          - Integrated values provided in Data_Frame.gyro_x_integral / gyro_y_integral
     *          - AP_OpticalFlow calls this method at IMU update rate (typically 400Hz)
     * 
     * @param[in] gyro_x Gyroscope rotation rate about X-axis (sensor frame), radians/second
     * @param[in] gyro_y Gyroscope rotation rate about Y-axis (sensor frame), radians/second
     * @param[in] dt Time step for this gyro sample, seconds
     * 
     * @note Called at IMU rate (much faster than flow sensor frame rate)
     * @note Gyro rates should be in sensor body frame, not vehicle body frame
     * @note Sensor frame orientation defined by FLOW_ORIENT_YAW parameter
     * @note Some flow sensors ignore pushed gyro data and use only internal gyros
     * 
     * Source: libraries/AP_HAL/OpticalFlow.h:31
     */
    virtual void push_gyro(float gyro_x, float gyro_y, float dt) = 0;
    
    /**
     * @brief Provide gyroscope bias correction for improved compensation
     * 
     * @details Pushes gyro bias estimates from EKF to flow sensor for enhanced motion
     *          compensation accuracy. Gyroscope bias (constant offset error) causes
     *          systematic errors in flow compensation if not corrected.
     *          
     *          **Bias Correction**:
     *          compensated_gyro = measured_gyro - gyro_bias
     *          
     *          EKF estimates gyro bias as part of state estimation. Providing these
     *          estimates to flow sensor improves compensation, especially during:
     *          - Extended hover operations
     *          - Temperature changes (gyro bias drifts with temperature)
     *          - After vehicle warm-up
     *          
     * @param[in] gyro_bias_x Estimated gyroscope bias on X-axis, radians/second
     * @param[in] gyro_bias_y Estimated gyroscope bias on Y-axis, radians/second
     * 
     * @note Bias estimates provided by EKF3 after convergence (typically after 10-30 seconds)
     * @note Called periodically by AP_OpticalFlow when new bias estimates available
     * @note Typical bias magnitudes: 0.001-0.01 rad/s (0.06-0.6 deg/s)
     * @note Some flow sensors may ignore bias correction
     * @note Not all HAL implementations support bias correction
     * 
     * Source: libraries/AP_HAL/OpticalFlow.h:32
     */
    virtual void push_gyro_bias(float gyro_bias_x, float gyro_bias_y) = 0;
};
