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
#pragma once

/**
 * @file AP_OpticalFlow_Backend.h
 * @brief Optical flow sensor backend base class and interface contract
 * 
 * @details This file defines the abstract base class for all optical flow sensor
 *          implementations in ArduPilot. Optical flow sensors measure apparent motion
 *          of visual features to estimate velocity relative to the ground.
 *          
 *          Supported backend implementations include:
 *          - PX4Flow: I2C optical flow sensor with integrated sonar
 *          - Pixart: Pixart PAW3902/PAA5100 SPI sensors
 *          - CXOF: Cheerson CX-OF I2C optical flow sensor
 *          - MAVLink: Optical flow data received via MAVLink OPTICAL_FLOW messages
 *          - MSP: Optical flow data received via MSP protocol
 *          - HereFlow: Hex HereFlow CAN optical flow sensor
 *          - UPFLOW: UPFLOW I2C optical flow sensor
 *          - Onboard: Onboard camera-based optical flow
 *          - SITL: Software-in-the-loop simulation backend
 *          
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "AP_OpticalFlow.h"

#include <AP_HAL/Semaphores.h>

/**
 * @class OpticalFlow_backend
 * @brief Abstract base class for optical flow sensor backends
 * 
 * @details This class defines the interface contract that all optical flow sensor
 *          implementations must follow. It provides:
 *          - Pure virtual update() method that backends must implement
 *          - Virtual init() and message handler methods for optional override
 *          - Protected helper methods for parameter access and coordinate transformations
 *          - Semaphore protection for thread-safe frontend state access
 *          - Common yaw rotation and scaling logic
 *          
 *          Backend Implementation Pattern:
 *          1. Detect/probe hardware in static detect() factory method (in .cpp)
 *          2. Initialize communication in init() (UART baud, I2C/SPI setup)
 *          3. Periodic update() reads sensor, accumulates samples, converts units
 *          4. Process: Scale pixels to rad/s, compensate with gyro, apply yaw rotation
 *          5. Forward to frontend via _update_frontend() with semaphore protection
 *          
 *          Coordinate Frame Conventions:
 *          - Sensor provides: Body frame flow rates (X forward, Y right typically)
 *          - _applyYaw(): Rotates to vehicle body frame if sensor mounted at angle
 *          - Frontend: Forwards to AHRS/EKF in vehicle body frame
 *          
 *          Thread Safety:
 *          - _sem protects frontend state during _update_frontend()
 *          - Backends may have additional semaphores for internal state
 * 
 * @note All backends must implement the pure virtual update() method
 * @warning Backends must call _update_frontend() to forward measurements to the frontend
 */
class OpticalFlow_backend
{
    /**
     * @brief AP_OpticalFlow is a friend class to access protected/private members
     * @note Allows frontend to manage backend lifecycle and access internal state
     */
    friend class AP_OpticalFlow;

public:
    /**
     * @brief Initialize backend with frontend reference
     * 
     * @param[in] _frontend AP_OpticalFlow manager instance that owns this backend
     * 
     * @details Stores frontend reference for parameter access and state updates.
     *          Called by frontend when creating backend instance.
     */
    OpticalFlow_backend(AP_OpticalFlow &_frontend);
    
    /**
     * @brief Virtual destructor for proper polymorphic cleanup
     * 
     * @details Ensures derived backend classes are properly destroyed when deleted
     *          through base class pointer. Required for proper resource cleanup.
     */
    virtual ~OpticalFlow_backend(void);

    /**
     * @brief Prevent copy construction and assignment
     * 
     * @details Backends represent unique hardware interfaces and should not be copied.
     *          Each backend instance corresponds to a specific physical sensor.
     */
    CLASS_NO_COPY(OpticalFlow_backend);

    /**
     * @brief Virtual initialization method for sensor setup
     * 
     * @details Default implementation is no-op. Backends override this method to:
     *          - Configure hardware communication (I2C address, SPI settings, UART baud)
     *          - Initialize sensor registers and operating modes
     *          - Verify sensor presence and firmware version
     *          - Allocate buffers or resources
     *          
     *          Called by frontend after backend construction during system startup.
     * 
     * @note Not pure virtual - backends with simple initialization can use default
     */
    virtual void init() {}

    /**
     * @brief Pure virtual update method - backends must implement
     * 
     * @details Called periodically by frontend (typically 10-100Hz depending on sensor).
     *          Backend implementations must:
     *          1. Read sensor data (I2C/SPI/UART/MAVLink)
     *          2. Accumulate samples if sensor runs faster than update rate
     *          3. Convert raw sensor units (pixels, counts) to rad/s
     *          4. Apply gyro compensation if available (bodyRate)
     *          5. Apply scaling and yaw rotation using helper methods
     *          6. Call _update_frontend() with processed OpticalFlow_state
     *          
     *          Units:
     *          - flowRate: rad/s (angular flow rates in body frame)
     *          - bodyRate: rad/s (gyro-based body rotation rates for compensation)
     *          - quality: 0-255 (0 = invalid/no tracking, 255 = excellent tracking)
     * 
     * @note Must be implemented by all backends (pure virtual = 0)
     * @warning Failure to call _update_frontend() results in no data reaching navigation system
     */
    virtual void update() = 0;

    /**
     * @brief Virtual MAVLink message handler
     * 
     * @param[in] msg MAVLink message to process
     * 
     * @details Default implementation is no-op. MAVLink backend overrides this to:
     *          - Parse OPTICAL_FLOW (message ID 100) messages
     *          - Extract flow_x, flow_y, quality fields
     *          - Convert units and coordinate frames as needed
     *          - Call _update_frontend() with parsed data
     *          
     *          Called by frontend when OPTICAL_FLOW messages are received from telemetry.
     * 
     * @note Only MAVLink backend needs to override this method
     */
    virtual void handle_msg(const mavlink_message_t &msg) {}

#if HAL_MSP_OPTICALFLOW_ENABLED
    /**
     * @brief Virtual MSP message handler (conditional compilation)
     * 
     * @param[in] pkt MSP optical flow data packet structure
     * 
     * @details Default implementation is no-op. MSP backend overrides this to:
     *          - Parse MSP optical flow packet format
     *          - Extract motion_x, motion_y, quality fields
     *          - Convert MSP-specific units to rad/s
     *          - Call _update_frontend() with parsed data
     *          
     *          Called by frontend when MSP optical flow messages are received.
     *          Only compiled when HAL_MSP_OPTICALFLOW_ENABLED is defined.
     * 
     * @note Only MSP backend needs to override this method
     * @note Conditional compilation - only available on platforms with MSP support
     */
    virtual void handle_msp(const MSP::msp_opflow_data_message_t &pkt) {}
#endif

protected:
    /**
     * @brief Reference to AP_OpticalFlow manager for parameter and state access
     * 
     * @details Provides backends access to:
     *          - Configuration parameters (_flowScalerX, _flowScalerY, _yawAngle_cd, _address)
     *          - Frontend state for updates
     *          - Sensor enable/disable status
     *          
     *          Used by helper methods to retrieve parameters and forward measurements.
     */
    AP_OpticalFlow &frontend;

    /**
     * @brief Forward processed measurement to frontend
     * 
     * @param[in] state OpticalFlow_state structure containing:
     *                  - flowRate: Vector2f flow rates in rad/s (X forward, Y right)
     *                  - bodyRate: Vector2f gyro rates in rad/s for compensation
     *                  - quality: uint8_t quality metric 0-255 (0=invalid, 255=excellent)
     * 
     * @details Semaphore-protected write to frontend state. This method:
     *          - Acquires _sem semaphore for thread-safe access
     *          - Copies state to frontend for consumption by AHRS/EKF
     *          - Updates timestamp and sensor health status
     *          - Releases semaphore
     *          
     *          Must be called by backends after processing sensor data in update().
     * 
     * @note Thread-safe via _sem semaphore protection
     * @warning Backends must call this method to provide data to navigation system
     */
    void _update_frontend(const struct AP_OpticalFlow::OpticalFlow_state &state);

    /**
     * @brief Retrieve X/Y scaling parameters from frontend
     * 
     * @return Vector2f containing _flowScalerX and _flowScalerY
     * 
     * @details Flow scaler parameters provide per-axis calibration corrections:
     *          - Units: Parts-per-thousand (-800 to 800)
     *          - Default: 0 (interpreted as 1.0 scale factor)
     *          - Usage: Apply as (1.0 + scaler/1000.0) * measured_flow
     *          - Purpose: Compensate for lens distortion or mounting angle errors
     *          
     *          Example: _flowScalerX = 50 means multiply X flow by 1.05 (5% increase)
     * 
     * @note Inline accessor for performance-critical update loop
     */
    Vector2f _flowScaler(void) const { return Vector2f(frontend._flowScalerX, frontend._flowScalerY); }

    /**
     * @brief Retrieve sensor yaw orientation from frontend
     * 
     * @return float yaw angle in radians
     * 
     * @details Converts _yawAngle_cd parameter from centidegrees to radians:
     *          - Parameter range: -17999 to 18000 centidegrees
     *          - Return range: -π to +π radians
     *          - Purpose: Rotate sensor frame to vehicle body frame
     *          
     *          Used by _applyYaw() to transform flow measurements when sensor is
     *          mounted at an angle relative to vehicle body frame.
     * 
     * @note Inline conversion for performance-critical update loop
     */
    float _yawAngleRad(void) const { return cd_to_rad(float(frontend._yawAngle_cd)); }

    /**
     * @brief Apply yaw rotation to flow vector
     * 
     * @param[in,out] v Vector2f flow vector modified in-place
     * 
     * @details Rotates flow vector v by _yawAngleRad() using 2D rotation matrix:
     *          - Input: Flow in sensor body frame
     *          - Output: Flow in vehicle body frame
     *          - Rotation: v_rotated = R(yaw) * v_sensor
     *          
     *          Coordinate frame transformation:
     *          - Sensor X/Y axes may not align with vehicle forward/right
     *          - Yaw parameter specifies sensor mounting angle
     *          - This method applies correction to align frames
     * 
     * @note Modifies vector in-place for efficiency
     * @note Call this after scaling but before _update_frontend()
     */
    void _applyYaw(Vector2f &v);

    /**
     * @brief Retrieve I2C address parameter from frontend
     * 
     * @return uint8_t I2C address (0-127)
     * 
     * @details Used by I2C-based backends (PX4Flow, CXOF, UPFLOW, etc.) to:
     *          - Configure I2C communication address during probe/init
     *          - Support multiple sensors on same I2C bus
     *          - Allow address customization via parameters
     *          
     *          Typical addresses: 0x42 (PX4Flow), 0x08 (CXOF), varies by sensor.
     * 
     * @note SPI and UART backends do not use this parameter
     * @note Inline accessor for performance
     */
    uint8_t get_address(void) const { return frontend._address; }
    
    /**
     * @brief Semaphore for thread-safe frontend state access
     * 
     * @details Protects concurrent access to frontend state during _update_frontend():
     *          - Backend update() runs in scheduler task context
     *          - Frontend may be accessed by other threads (GCS, logging, EKF)
     *          - Semaphore prevents race conditions and data corruption
     *          
     *          Usage pattern:
     *          - _update_frontend() acquires semaphore with WITH_SEMAPHORE(_sem)
     *          - Critical section: Copy state to frontend
     *          - Automatic release when leaving scope
     * 
     * @warning Must be held when accessing frontend state variables
     * @note HAL_Semaphore provides platform-specific mutual exclusion
     */
    HAL_Semaphore _sem;
};
