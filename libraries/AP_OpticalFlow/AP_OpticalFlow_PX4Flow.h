/**
 * @file AP_OpticalFlow_PX4Flow.h
 * @brief PX4Flow I2C optical flow sensor backend driver
 * 
 * @details This driver implements support for the PX4Flow smart camera module,
 *          which provides optical flow measurements via I2C interface.
 *          
 *          Hardware: PX4Flow smart camera module with onboard processing
 *          - Onboard image processor computes optical flow
 *          - Integrated gyroscope for motion compensation
 *          - Integrated sonar rangefinder for ground distance
 *          - I2C interface at address 0x42 (PX4FLOW_BASE_I2C_ADDR)
 *          
 *          Protocol: I2C register-based interface
 *          - Reads REG_INTEGRAL_FRAME (0x16) register
 *          - Returns 22-byte i2c_integral_frame structure
 *          - Contains integral motion over time period
 *          - Onboard gyro compensation data included
 *          - Sonar rangefinder integration available
 *          
 *          Features:
 *          - Integral pixel motion (accumulated over integration period)
 *          - Onboard gyro integrals for motion compensation
 *          - Quality metric (0-255)
 *          - Ground distance from sonar (millimeters)
 *          - Automatic conversion to flow rates (rad/s)
 * 
 * @note Driver probes all I2C buses for PX4Flow device at address 0x42
 * @warning Requires adequate surface texture and lighting for reliable operation
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_PX4FLOW_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_HAL/utility/OwnPtr.h>

/**
 * @class AP_OpticalFlow_PX4Flow
 * @brief PX4Flow I2C optical flow sensor backend
 * 
 * @details This backend driver interfaces with the PX4Flow smart camera module
 *          to provide optical flow measurements for velocity estimation and
 *          position hold modes.
 *          
 *          Operation:
 *          1. Probes I2C buses for PX4Flow at address 0x42
 *          2. Sets up periodic timer callback (~10Hz) for I2C reads
 *          3. Reads REG_INTEGRAL_FRAME (0x16) containing 22-byte structure
 *          4. Converts integral pixel motion to flow rates (rad/s)
 *          5. Applies scaling and yaw compensation
 *          6. Updates frontend with flow rates, quality, and ground distance
 *          
 *          Coordinate Frames:
 *          - Sensor body frame: X forward, Y right
 *          - Backend converts to vehicle body frame as needed
 *          
 *          Integration:
 *          - Inherits from OpticalFlow_backend base class
 *          - Called periodically by AP_OpticalFlow frontend
 *          - Provides flow rates, quality metric, and ground distance
 * 
 * @note Timer callback runs at approximately 10Hz for continuous I2C polling
 * @note PX4Flow has onboard gyro - gyro integrals can be used for compensation
 * @warning Requires I2C device at address 0x42 to be present
 * @warning Surface texture and lighting requirements apply for reliable flow
 */
class AP_OpticalFlow_PX4Flow : public OpticalFlow_backend
{
public:
    /**
     * @brief Constructor - inherits from OpticalFlow_backend base class
     * 
     * @details Uses base class constructor via using declaration.
     *          Actual initialization occurs in detect() and setup_sensor().
     * 
     * @note CLASS_NO_COPY prevents copying of this class
     */
    using OpticalFlow_backend::OpticalFlow_backend;

    CLASS_NO_COPY(AP_OpticalFlow_PX4Flow);

    /**
     * @brief Initialize the sensor (no explicit initialization required)
     * 
     * @details Empty implementation as sensor detection and configuration
     *          happens in detect() static method and setup_sensor() private method.
     *          This override satisfies the pure virtual function requirement
     *          from OpticalFlow_backend base class.
     * 
     * @note Detection and setup occur during detect() call, not in init()
     */
    void init() override {}

    /**
     * @brief Periodic update called from main loop
     * 
     * @details This method is called periodically by the AP_OpticalFlow frontend.
     *          For PX4Flow, actual I2C reading happens in timer() callback which
     *          runs asynchronously at ~10Hz. This update() method is typically
     *          a no-op as timer() handles all I2C communication and data updates.
     * 
     * @note I2C reads are performed in timer() callback, not in this method
     * @note Override of pure virtual function from OpticalFlow_backend
     */
    void update(void) override;

    /**
     * @brief Static factory method to detect and create PX4Flow instance
     * 
     * @details Probes all available I2C buses for PX4Flow device at address 0x42.
     *          If found, creates backend instance and configures periodic timer
     *          for I2C reads. This is called by AP_OpticalFlow frontend during
     *          sensor auto-detection.
     *          
     *          Detection sequence:
     *          1. Calls scan_buses() to probe I2C buses
     *          2. If device found, creates AP_OpticalFlow_PX4Flow instance
     *          3. Calls setup_sensor() to configure timer callback
     *          4. Returns instance pointer or nullptr if not found
     * 
     * @param[in] _frontend Reference to AP_OpticalFlow manager for callbacks
     * 
     * @return Pointer to new AP_OpticalFlow_PX4Flow instance if detected,
     *         nullptr if device not found on any I2C bus
     * 
     * @note Scans all I2C buses for device at address 0x42
     * @note Only one PX4Flow instance is typically created
     */
    static AP_OpticalFlow_PX4Flow *detect(AP_OpticalFlow &_frontend);

private:
    /// I2C device handle for communication with PX4Flow sensor
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    /// I2C register address for integral frame data (0x16)
    static const uint8_t REG_INTEGRAL_FRAME = 0x16;
    
    /**
     * @struct i2c_integral_frame
     * @brief 22-byte data structure read from PX4Flow register 0x16
     * 
     * @details This packed structure contains integral (accumulated) motion data
     *          from the PX4Flow sensor over an integration period. Data includes
     *          pixel motion integrals, onboard gyro integrals, quality metric,
     *          and sonar rangefinder distance.
     *          
     *          Integration Method:
     *          - Values are accumulated over integration_timespan period
     *          - To get rates: divide integrals by integration_timespan
     *          - Example: flow_rate_x = pixel_flow_x_integral / integration_timespan
     *          
     *          Coordinate Frame:
     *          - X axis: Forward (positive = forward motion)
     *          - Y axis: Right (positive = rightward motion)
     *          - Z axis: Down (right-hand rule)
     * 
     * @note Structure is PACKED for direct I2C read into memory
     * @note All integral values must be divided by integration_timespan for rates
     */
    struct PACKED i2c_integral_frame {
        /// Frame counter since last readout - detects missed reads if not sequential
        uint16_t frame_count_since_last_readout;
        
        /// Accumulated pixel motion in X direction (integer counts over integration period)
        int16_t pixel_flow_x_integral;
        
        /// Accumulated pixel motion in Y direction (integer counts over integration period)
        int16_t pixel_flow_y_integral;
        
        /// Accumulated gyro X rate (rad*microseconds) - can be used for motion compensation
        int16_t gyro_x_rate_integral;
        
        /// Accumulated gyro Y rate (rad*microseconds) - can be used for motion compensation
        int16_t gyro_y_rate_integral;
        
        /// Accumulated gyro Z rate (rad*microseconds) - can be used for motion compensation
        int16_t gyro_z_rate_integral;
        
        /// Integration time period in microseconds - divide integrals by this for rates
        uint32_t integration_timespan;
        
        /// Timestamp of sonar reading in microseconds
        uint32_t sonar_timestamp;
        
        /// Ground distance from sonar rangefinder in millimeters
        uint16_t ground_distance;
        
        /// Onboard gyro temperature in raw sensor units
        int16_t gyro_temperature;
        
        /// Flow quality metric (0-255, higher is better, 0 = no valid flow)
        uint8_t qual;
    };
    
    /**
     * @brief Probe all I2C buses for PX4Flow device
     * 
     * @details Scans available I2C buses looking for PX4Flow device at
     *          address 0x42 (PX4FLOW_BASE_I2C_ADDR). Uses HAL I2C interface
     *          to probe each bus. If device responds, stores device handle
     *          in 'dev' member.
     *          
     *          Bus Scanning:
     *          - Tries all available I2C buses on the board
     *          - Uses standard I2C probe mechanism
     *          - Typically finds device on first external I2C bus
     * 
     * @return true if PX4Flow device found and device handle acquired,
     *         false if not found on any bus
     * 
     * @note Device must respond to I2C address 0x42
     * @note Sets dev member to device handle if successful
     */
    bool scan_buses(void);

    /**
     * @brief Configure sensor and setup periodic timer callback
     * 
     * @details Sets up periodic timer to call timer() method at approximately
     *          10Hz for continuous I2C reads. Timer callback runs asynchronously
     *          and handles all I2C communication with PX4Flow device.
     *          
     *          Configuration:
     *          - Registers timer() as periodic callback (~10Hz)
     *          - Timer runs in HAL scheduler context
     *          - Ensures non-blocking I2C communication
     * 
     * @return true if timer setup successful, false on failure
     * 
     * @note Must be called after scan_buses() succeeds
     * @note Timer frequency ~10Hz matches typical optical flow update rate
     * @warning Failure to setup timer means no data updates will occur
     */
    bool setup_sensor(void);

    /**
     * @brief Periodic timer callback for I2C reads
     * 
     * @details Called asynchronously at ~10Hz by HAL scheduler timer.
     *          Performs I2C read of REG_INTEGRAL_FRAME (0x16) register
     *          containing 22-byte i2c_integral_frame structure, converts
     *          integral values to rates, and updates frontend.
     *          
     *          Processing Steps:
     *          1. Read 22 bytes from register 0x16 into i2c_integral_frame
     *          2. Extract integration_timespan (microseconds)
     *          3. Convert pixel integrals to flow rates: rate = integral / timespan
     *          4. Apply scaling factors for sensor calibration
     *          5. Apply yaw compensation if configured
     *          6. Extract quality metric (0-255)
     *          7. Extract ground_distance from sonar (millimeters)
     *          8. Call frontend update with flow rates, quality, and distance
     *          
     *          Units Conversion:
     *          - Input pixel integrals: integer counts
     *          - Output flow rates: rad/s (radians per second)
     *          - Ground distance: millimeters → meters conversion
     *          - Integration timespan: microseconds for rate calculation
     * 
     * @note Runs in timer callback context, must be fast and non-blocking
     * @note Called at approximately 10Hz by HAL scheduler
     * @note I2C read errors are handled gracefully (skipped updates)
     * @warning Do not call blocking functions from timer callback
     * @warning Keep execution time minimal to avoid scheduler delays
     */
    void timer(void);
};

#endif  // AP_OPTICALFLOW_PX4FLOW_ENABLED
