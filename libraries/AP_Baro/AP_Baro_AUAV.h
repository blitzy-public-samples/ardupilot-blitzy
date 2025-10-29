/**
 * @file AP_Baro_AUAV.h
 * @brief AUAV barometer driver for ArduPilot
 * 
 * @details This driver provides barometric pressure sensing using the AUAV
 *          absolute pressure sensor. The driver wraps the AUAV_Pressure_sensor
 *          class from the AP_Airspeed library, enabling the AUAV sensor to be
 *          used as a barometer for altitude estimation.
 * 
 *          The driver operates by accumulating pressure and temperature samples
 *          via a periodic timer callback, then averaging them when the frontend
 *          requests an update. This provides noise reduction and improved
 *          stability for altitude hold and navigation.
 * 
 * @note This driver requires the AUAV airspeed driver to be enabled at compile
 *       time (AP_AIRSPEED_AUAV_ENABLED must be true).
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_AUAV_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

// Baro uses the airspeed AUAV Pressure sensor class from airspeed, airspeed must be enabled
#include <AP_Airspeed/AP_Airspeed_config.h>
#if !AP_AIRSPEED_AUAV_ENABLED
#error AUAV Baro requires AUAV Airspeed
#endif

#include <AP_Airspeed/AP_Airspeed_AUAV.h>


#ifndef HAL_BARO_AUAV_I2C_ADDR
 #define HAL_BARO_AUAV_I2C_ADDR 0x27  ///< Default I2C address for AUAV barometer sensor
#endif

/**
 * @class AP_Baro_AUAV
 * @brief Barometer backend driver for AUAV absolute pressure sensor
 * 
 * @details This driver wraps the AUAV_Pressure_sensor class from the AP_Airspeed
 *          library to provide barometric pressure measurements for altitude estimation.
 *          The AUAV sensor provides both pressure and temperature readings via I2C.
 * 
 *          The driver uses a sample accumulation strategy: the timer() callback reads
 *          sensor data periodically and accumulates pressure and temperature values.
 *          When update() is called by the frontend, the accumulated samples are averaged
 *          and pushed to the barometer frontend for processing.
 * 
 *          Typical operation:
 *          1. Sensor probed and initialized on I2C bus (default address 0x27)
 *          2. timer() callback registered with HAL scheduler
 *          3. timer() periodically reads sensor and accumulates samples
 *          4. update() computes averages and provides data to frontend
 * 
 * @note Pressure values are reported in Pascals (Pa)
 * @note Temperature values are reported in Celsius (°C)
 * @note Requires AP_AIRSPEED_AUAV_ENABLED to be defined at compile time
 * 
 * @see AP_Baro_Backend
 * @see AUAV_Pressure_sensor
 */
class AP_Baro_AUAV : public AP_Baro_Backend {
public:
    /**
     * @brief Construct a new AP_Baro_AUAV driver instance
     * 
     * @details Initializes the AUAV barometer driver with the provided barometer
     *          frontend and I2C device handle. The constructor stores the device
     *          pointer and prepares the driver for initialization.
     * 
     * @param[in] baro Reference to the main AP_Baro frontend instance
     * @param[in] dev  OwnPtr to the HAL I2C device for sensor communication
     * 
     * @note Actual sensor initialization occurs in init() method
     */
    AP_Baro_AUAV(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer frontend with accumulated sensor data
     * 
     * @details This method is called periodically by the AP_Baro frontend to retrieve
     *          new barometer measurements. It computes the average of accumulated pressure
     *          and temperature samples collected by the timer() callback, then pushes
     *          the averaged values to the barometer frontend.
     * 
     *          The accumulation and averaging approach reduces sensor noise and provides
     *          more stable altitude estimates for flight control.
     * 
     * @note Pressure values are in Pascals (Pa)
     * @note Temperature values are in Celsius (°C)
     * @note This method is called at the frontend's update rate
     * 
     * @see timer()
     */
    void update() override;

    /**
     * @brief Probe for AUAV barometer sensor on I2C bus
     * 
     * @details Attempts to detect and initialize an AUAV barometer sensor on the
     *          provided I2C device. If successful, creates and returns a new
     *          AP_Baro_AUAV backend instance. If detection fails or initialization
     *          is unsuccessful, returns nullptr.
     * 
     *          This method is typically called during system startup as part of the
     *          barometer auto-detection process.
     * 
     * @param[in] baro Reference to the main AP_Baro frontend instance
     * @param[in] dev  OwnPtr to the HAL I2C device to probe (typically at address 0x27)
     * 
     * @return AP_Baro_Backend* Pointer to new AP_Baro_AUAV instance if successful,
     *                          nullptr if sensor not detected or initialization failed
     * 
     * @note Default I2C address is 0x27 (defined by HAL_BARO_AUAV_I2C_ADDR)
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

protected:
    /**
     * @brief Initialize the AUAV barometer sensor
     * 
     * @details Performs sensor initialization including device configuration,
     *          backend registration with frontend, and timer callback setup.
     * 
     * @return true if initialization successful, false otherwise
     */
    bool init();

    /**
     * @brief Timer callback for periodic sensor reading and sample accumulation
     * 
     * @details This method is called periodically by the HAL scheduler to read
     *          pressure and temperature from the AUAV sensor. Each reading is
     *          accumulated into pressure_sum and temperature_sum for later averaging
     *          in the update() method. This accumulation approach provides noise
     *          reduction and improved measurement stability.
     * 
     * @note Called at high frequency by HAL scheduler (typically 20-50Hz)
     * @note Accumulated samples are averaged and reset in update() method
     */
    void timer();

    AP_HAL::OwnPtr<AP_HAL::Device> dev;  ///< Owned pointer to HAL device for sensor communication
    AP_HAL::I2CDevice *i2c_dev;          ///< Raw pointer to I2C device (for sensor initialization)

    /// AUAV absolute pressure sensor instance (wraps AP_Airspeed AUAV driver)
    AUAV_Pressure_sensor sensor { i2c_dev, AUAV_Pressure_sensor::Type::Absolute };

    uint8_t instance;  ///< Barometer instance number assigned by frontend

    // Sample accumulation variables for averaging
    uint32_t count;             ///< Number of accumulated samples since last update
    float pressure_sum;         ///< Accumulated pressure samples in Pascals (Pa)
    float temperature_sum;      ///< Accumulated temperature samples in Celsius (°C)

    bool measurement_requested;  ///< Flag indicating whether a new measurement has been requested
};

#endif  // AP_BARO_AUAV_ENABLED
