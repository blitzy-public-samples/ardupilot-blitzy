/**
 * @file AP_Baro_LPS2XH.h
 * @brief Driver for STMicroelectronics LPS2XH series barometric pressure sensors
 * 
 * @details This driver supports the STMicroelectronics LPS2XH family of MEMS
 *          absolute pressure sensors including:
 *          - LPS25H: 260-1260 hPa range, 24-bit pressure output
 *          - LPS22H: 260-1260 hPa range, 24-bit pressure output, improved accuracy
 *          - LPS22HB: Enhanced version of LPS22H with improved stability
 *          - LPS22HH: High-performance variant with dual full scale and FIFO
 * 
 *          These sensors communicate via I2C or SPI and provide both pressure
 *          and temperature measurements. The driver handles device detection,
 *          initialization, and continuous data acquisition.
 * 
 *          Units:
 *          - Pressure: Pascals (Pa)
 *          - Temperature: Celsius (°C)
 * 
 * @note The driver automatically detects LPS22H vs LPS25H variants via WHO_AM_I register
 * @note Some variants are accessible via auxiliary I2C on InvenSense IMUs
 * 
 * Source: libraries/AP_Baro/AP_Baro_LPS2XH.h
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_LPS2XH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>

#define HAL_BARO_LPS25H_I2C_BUS 0

#ifndef HAL_BARO_LPS25H_I2C_ADDR
# define HAL_BARO_LPS25H_I2C_ADDR 0x5D
#endif


/**
 * @class AP_Baro_LPS2XH
 * @brief Barometer backend driver for STMicroelectronics LPS2XH MEMS pressure sensors
 * 
 * @details This class implements the backend interface for the LPS2XH family of
 *          absolute pressure sensors from STMicroelectronics. The driver supports
 *          automatic device detection and handles differences between sensor variants.
 * 
 *          Key Features:
 *          - Automatic detection of LPS22H vs LPS25H via WHO_AM_I register
 *          - Support for both direct I2C/SPI and auxiliary I2C through InvenSense IMUs
 *          - Continuous pressure and temperature acquisition via timer callbacks
 *          - Pressure averaging for noise reduction
 * 
 *          Device Variants:
 *          - LPS22H: Newer generation with improved accuracy and lower power
 *          - LPS25H: Original variant with proven reliability
 *          - LPS22HB/LPS22HH: Enhanced variants with additional features
 * 
 *          The driver accumulates pressure samples and provides averaged readings
 *          to the main barometer library. Temperature is updated independently
 *          to support pressure compensation.
 * 
 * @note Inherits from AP_Baro_Backend base class for standard barometer interface
 * @note Driver uses timer-based sampling for consistent update rates
 * @warning Incorrect I2C address or bus configuration will cause initialization failure
 */
class AP_Baro_LPS2XH : public AP_Baro_Backend
{
public:
    /**
     * @enum LPS2XH_TYPE
     * @brief Enumeration of supported LPS2XH sensor variants
     * 
     * @details Used internally to track which specific device variant was detected.
     *          Device type is determined during initialization via WHO_AM_I register.
     *          Different variants may require variant-specific register sequences.
     */
    enum LPS2XH_TYPE {
        BARO_LPS22H = 0,  ///< LPS22H variant (newer generation, improved accuracy)
        BARO_LPS25H = 1,  ///< LPS25H variant (original generation)
    };

    /**
     * @brief Constructor for LPS2XH barometer driver
     * 
     * @details Initializes the driver instance with the provided device interface.
     *          Does not perform hardware initialization - that occurs in _init().
     * 
     * @param[in] baro      Reference to main AP_Baro instance for sensor registration
     * @param[in] dev       OwnPtr to HAL device (I2C or SPI) for sensor communication
     * 
     * @note Actual device detection and configuration occurs in _init()
     */
    AP_Baro_LPS2XH(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer readings from accumulated samples
     * 
     * @details Called by main barometer library to retrieve latest pressure data.
     *          This method processes accumulated pressure samples from timer callbacks,
     *          computes the average, and updates the barometer instance with new readings.
     *          
     *          Pressure values are in Pascals (Pa) and temperature in Celsius (°C).
     *          Sample accumulation provides noise reduction through averaging.
     * 
     * @note Called at main loop rate, typically 10-50 Hz
     * @note Pressure accumulation occurs in timer callback _timer()
     */
    void update() override;

    /**
     * @brief Probe for LPS2XH sensor on the given device interface
     * 
     * @details Static factory method that attempts to detect and initialize an
     *          LPS2XH sensor on the provided HAL device. Reads WHO_AM_I register
     *          to identify device variant (LPS22H vs LPS25H). If successful,
     *          returns initialized backend instance.
     * 
     * @param[in] baro  Reference to main AP_Baro instance
     * @param[in] dev   OwnPtr to HAL device (I2C or SPI) to probe
     * 
     * @return Pointer to initialized AP_Baro_Backend if successful, nullptr if detection failed
     * 
     * @note Device ownership transfers to returned backend instance on success
     * @note Failure cases: incorrect address, communication error, unrecognized WHO_AM_I
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
    
    /**
     * @brief Probe for LPS2XH sensor via auxiliary I2C on InvenSense IMU
     * 
     * @details Some LPS2XH sensors are connected to the auxiliary I2C bus of
     *          InvenSense IMUs (MPU9250, ICM20xxx series). This specialized probe
     *          function configures the IMU's I2C master to access the barometer.
     *          
     *          The function sets up I2C passthrough or master mode on the IMU,
     *          then attempts to detect the LPS2XH sensor at the expected address.
     * 
     * @param[in] baro         Reference to main AP_Baro instance
     * @param[in] dev          OwnPtr to HAL device representing the IMU
     * @param[in] imu_address  I2C address of the InvenSense IMU (typically 0x68 or 0x69)
     * 
     * @return Pointer to initialized AP_Baro_Backend if successful, nullptr if detection failed
     * 
     * @note Requires IMU to be properly initialized first
     * @note This is a specialized probe for boards with barometer behind IMU I2C master
     * @warning IMU I2C master configuration errors can prevent barometer detection
     */
    static AP_Baro_Backend *probe_InvensenseIMU(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, uint8_t imu_address);

private:
    /**
     * @brief Destructor
     * @note Virtual destructor for proper cleanup in derived class hierarchy
     */
    virtual ~AP_Baro_LPS2XH(void) {};

    /**
     * @brief Initialize the LPS2XH sensor hardware
     * 
     * @details Performs device detection via WHO_AM_I register, configures sensor
     *          operating mode and data rate, and registers timer callback for
     *          continuous data acquisition. Device-specific register sequences
     *          are applied based on detected variant (LPS22H vs LPS25H).
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Registers periodic timer callback at appropriate rate for sensor
     * @note Different variants may use different register configurations
     */
    bool _init(void);
    
    /**
     * @brief Timer callback for sensor data acquisition
     * 
     * @details Called periodically by HAL scheduler to read pressure and temperature
     *          from the sensor. Reads are performed at sensor output data rate.
     *          Pressure samples are accumulated for averaging, temperature is updated
     *          separately. Handles device-specific FIFO reading if available.
     * 
     * @note Called at timer rate (typically 25-75 Hz depending on sensor configuration)
     * @note LPS22HH variant may use FIFO mode for burst reading
     */
    void _timer(void);
    
    /**
     * @brief Read and update temperature measurement
     * 
     * @details Reads temperature registers and converts raw sensor value to Celsius.
     *          Temperature is stored in _temperature member variable. Used for both
     *          reporting and internal pressure temperature compensation.
     * 
     * @note Temperature output in Celsius (°C)
     * @note Temperature LSB varies by device variant
     */
    void _update_temperature(void);
    
    /**
     * @brief Read and accumulate pressure measurement
     * 
     * @details Reads pressure registers and converts raw 24-bit sensor value to Pascals.
     *          Accumulates samples into _pressure_sum for averaging. Increments
     *          _pressure_count for each valid sample.
     * 
     * @note Pressure output in Pascals (Pa)
     * @note Accumulation reset by update() when averaged value is reported
     */
    void _update_pressure(void);
    
    /**
     * @brief Initialize sensor access via IMU auxiliary I2C
     * 
     * @details Configures InvenSense IMU I2C master to access the LPS2XH sensor
     *          on auxiliary I2C bus. Sets up I2C passthrough or master mode,
     *          configures slave addressing, and verifies communication.
     * 
     * @param[in] imu_address  I2C address of the InvenSense IMU
     * 
     * @return true if IMU I2C master setup successful, false on failure
     * 
     * @note IMU must be initialized before calling this function
     * @warning Incorrect IMU address or IMU initialization failure will cause errors
     */
    bool _imu_i2c_init(uint8_t imu_address);

    /**
     * @brief Verify sensor identity via WHO_AM_I register
     * 
     * @details Reads WHO_AM_I register and compares against known LPS2XH values.
     *          Sets _lps2xh_type based on detected variant. Used during initialization
     *          to confirm correct sensor and determine variant-specific configuration.
     * 
     * @return true if recognized LPS2XH variant detected, false if unknown or read error
     * 
     * @note WHO_AM_I values: LPS22H = 0xB1, LPS25H = 0xBD
     */
    bool _check_whoami(void);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< HAL device interface (I2C or SPI) for sensor communication

    uint8_t _instance;          ///< Barometer instance number in AP_Baro
    float _pressure_sum;        ///< Accumulated pressure samples for averaging (Pa)
    uint32_t _pressure_count;   ///< Number of pressure samples accumulated
    float _temperature;         ///< Last temperature reading (°C)

    uint32_t CallTime = 0;      ///< Timestamp for timing control (microseconds)

    enum LPS2XH_TYPE _lps2xh_type;  ///< Detected sensor variant (LPS22H or LPS25H)
};

#endif  // AP_BARO_LPS2XH_ENABLED
