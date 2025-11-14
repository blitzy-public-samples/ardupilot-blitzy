/**
 * @file AP_Baro_BMP581.h
 * @brief Driver for the Bosch BMP581 ultra-high-precision barometric pressure sensor
 * 
 * @details This file implements support for the Bosch BMP581, a next-generation
 *          barometric pressure sensor offering improved accuracy and stability over
 *          previous BMP-series sensors. The BMP581 features advanced on-chip digital
 *          signal processing, temperature compensation, and low power consumption.
 *          
 *          Key Features:
 *          - Pressure range: 300-1250 hPa (9000m to -500m altitude)
 *          - Ultra-high relative accuracy: ±0.2 hPa (±1.7m)
 *          - Low noise: 0.03 Pa RMS
 *          - I2C and SPI interface support
 *          - On-chip temperature sensor for compensation
 *          - Configurable ODR (Output Data Rate) and oversampling
 *          
 *          The driver accumulates multiple pressure and temperature samples
 *          in the timer callback and provides averaged values to the AP_Baro
 *          frontend when update() is called.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_BMP581_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#ifndef HAL_BARO_BMP581_I2C_ADDR
 #define HAL_BARO_BMP581_I2C_ADDR  (0x46)  ///< Default I2C address (SDO pin low)
#endif
#ifndef HAL_BARO_BMP581_I2C_ADDR2
 #define HAL_BARO_BMP581_I2C_ADDR2 (0x47)  ///< Alternate I2C address (SDO pin high)
#endif

/**
 * @class AP_Baro_BMP581
 * @brief Backend driver for Bosch BMP581 ultra-high-precision barometric pressure sensor
 * 
 * @details The AP_Baro_BMP581 class implements support for the next-generation BMP581
 *          barometric pressure sensor from Bosch Sensortec. This sensor provides
 *          significantly improved accuracy and stability compared to earlier BMP-series
 *          sensors (BMP280, BMP388, etc.).
 *          
 *          Driver Architecture:
 *          - Inherits from AP_Baro_Backend for integration with AP_Baro frontend
 *          - Supports both I2C and SPI communication interfaces
 *          - Uses timer-based periodic sampling for consistent data acquisition
 *          - Accumulates multiple samples for noise reduction
 *          - Applies on-chip temperature compensation automatically
 *          
 *          Communication Interfaces:
 *          - I2C: Default address 0x46, alternate address 0x47 (SDO pin dependent)
 *          - SPI: Chip select controlled by Device abstraction
 *          
 *          Data Acquisition:
 *          The driver uses a two-stage approach:
 *          1. timer() callback: Called periodically by HAL scheduler, reads sensor
 *             and accumulates samples in pressure_sum/pressure_count
 *          2. update() method: Called by AP_Baro frontend, computes average and
 *             publishes to frontend
 *          
 *          Units:
 *          - Pressure: Pascals (Pa)
 *          - Temperature: Degrees Celsius (°C)
 *          - Altitude derived by AP_Baro frontend from pressure
 *          
 *          Thread Safety:
 *          - timer() runs in HAL scheduler context
 *          - update() called from main vehicle thread
 *          - Access to accumulation variables protected by sensor reading order
 * 
 * @note The BMP581 includes advanced filtering and IIR coefficients that are
 *       configured during initialization based on the desired performance profile.
 * 
 * @warning Ensure proper I2C pull-up resistors or SPI signal integrity for
 *          reliable communication, especially in electrically noisy environments.
 */
class AP_Baro_BMP581 : public AP_Baro_Backend
{
public:
    /**
     * @brief Construct a new AP_Baro_BMP581 driver instance
     * 
     * @details Constructor for the BMP581 barometer backend driver. This is called
     *          by the probe() static method after successful sensor detection and
     *          initialization. The constructor stores the device handle and registers
     *          the driver with the AP_Baro frontend.
     *          
     *          The actual hardware initialization is performed in the private init()
     *          method, which configures sensor sampling rates, filtering, and starts
     *          the timer-based data acquisition.
     * 
     * @param[in,out] baro Reference to the AP_Baro frontend manager
     * @param[in]     dev  OwnPtr to the HAL device (I2C or SPI) - ownership transferred
     * 
     * @note This constructor is typically not called directly. Use the probe() static
     *       factory method for driver instantiation.
     * 
     * @see probe()
     */
    AP_Baro_BMP581(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer readings from accumulated samples
     * 
     * @details This method is called periodically by the AP_Baro frontend (typically
     *          at the main loop rate) to retrieve updated pressure and temperature
     *          readings. It computes the average of all samples accumulated since the
     *          last update() call and publishes the results to the frontend.
     *          
     *          Processing Steps:
     *          1. Check if new samples are available (pressure_count > 0)
     *          2. Calculate average pressure: pressure_sum / pressure_count
     *          3. Use the most recent temperature reading
     *          4. Publish pressure (Pa) and temperature (°C) to frontend via
     *             _copy_to_frontend()
     *          5. Reset accumulation variables for next cycle
     *          
     *          Sample Averaging:
     *          Multiple samples are accumulated in the timer() callback running at
     *          the sensor's configured ODR (Output Data Rate). Averaging reduces
     *          measurement noise and provides more stable altitude estimates.
     * 
     * @note Called from the main vehicle thread. Thread-safe through atomic
     *       accumulation pattern.
     * 
     * @see timer()
     * @see AP_Baro_Backend::_copy_to_frontend()
     */
    void update() override;

    /**
     * @brief Probe and initialize a BMP581 sensor on the given device
     * 
     * @details Static factory method that attempts to detect and initialize a BMP581
     *          barometric pressure sensor on the provided I2C or SPI device. This method
     *          is called by the AP_Baro frontend during sensor auto-detection.
     *          
     *          Probing Sequence:
     *          1. Read and validate sensor chip ID register (expected: 0x50 for BMP581)
     *          2. Verify communication interface is working
     *          3. Read and validate calibration coefficients from NVM (Non-Volatile Memory)
     *          4. Configure sensor operating mode, ODR, and oversampling
     *          5. Configure IIR filter coefficients for noise reduction
     *          6. Perform initial self-test if available
     *          7. Create driver instance if all checks pass
     *          8. Register periodic timer callback for data acquisition
     *          
     *          Calibration Coefficients:
     *          The BMP581 stores factory calibration data in on-chip NVM. These
     *          coefficients are used for temperature compensation of pressure readings
     *          to achieve the specified accuracy. The coefficients are automatically
     *          applied by the sensor's internal DSP.
     *          
     *          Communication Interface:
     *          - I2C: Attempts addresses 0x46 and 0x47 based on SDO pin state
     *          - SPI: Uses provided chip select configuration
     * 
     * @param[in,out] baro Reference to the AP_Baro frontend manager
     * @param[in]     dev  OwnPtr to the HAL device (I2C or SPI) to probe
     * 
     * @return Pointer to newly created AP_Baro_BMP581 instance on success, nullptr on failure
     * 
     * @note This method transfers ownership of the device pointer if successful.
     *       On failure, the device pointer is automatically freed.
     * 
     * @warning Probing involves multiple I2C/SPI transactions. Ensure the device
     *          is not being accessed concurrently from other code.
     * 
     * @see AP_Baro::_add_backend()
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    /**
     * @brief Initialize the BMP581 sensor hardware
     * 
     * @details Performs hardware initialization including sensor configuration,
     *          calibration coefficient reading, and timer registration.
     * 
     * @return true if initialization successful, false otherwise
     */
    bool init(void);
    
    /**
     * @brief Timer callback for periodic sensor reading
     * 
     * @details Called by HAL scheduler at configured rate to read sensor and
     *          accumulate samples. Reads pressure and temperature from sensor
     *          registers and adds to accumulation variables.
     * 
     * @note Runs in scheduler/interrupt context. Keep processing minimal.
     */
    void timer(void);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< HAL device handle (I2C or SPI interface)

    uint8_t instance;          ///< Sensor instance number registered with AP_Baro frontend
    float pressure_sum;        ///< Accumulated pressure samples in Pascals (Pa) since last update()
    uint32_t pressure_count;   ///< Number of pressure samples accumulated since last update()
    float temperature;         ///< Most recent temperature reading in degrees Celsius (°C)
};

#endif  // AP_BARO_BMP581_ENABLED
