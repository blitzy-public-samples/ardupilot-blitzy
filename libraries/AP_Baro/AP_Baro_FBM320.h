/**
 * @file AP_Baro_FBM320.h
 * @brief Driver for Formosa Microsystems FBM320 barometric pressure sensor
 * 
 * @details This driver implements support for the FBM320 barometric pressure and
 *          temperature sensor from Formosa Microsystems. The FBM320 is a high-precision
 *          digital barometer with integrated temperature sensor, communicating via I2C.
 *          
 *          Key features:
 *          - Pressure range: 300-1100 hPa
 *          - Resolution: 0.01 hPa (0.1 m altitude)
 *          - Operating temperature: -40°C to +85°C
 *          - I2C interface (addresses 0x6C or 0x6D)
 *          
 *          The driver performs temperature-compensated pressure measurements using
 *          factory-calibrated coefficients stored in the device's non-volatile memory.
 * 
 * @note This driver requires AP_BARO_FBM320_ENABLED to be defined
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_FBM320_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#ifndef HAL_BARO_FBM320_I2C_ADDR
 #define HAL_BARO_FBM320_I2C_ADDR  0x6C  ///< Default I2C address for FBM320
#endif
#ifndef HAL_BARO_FBM320_I2C_ADDR2
 #define HAL_BARO_FBM320_I2C_ADDR2 0x6D  ///< Alternate I2C address for FBM320
#endif

/**
 * @class AP_Baro_FBM320
 * @brief Backend driver for Formosa Microsystems FBM320 barometric pressure sensor
 * 
 * @details This class implements the AP_Baro_Backend interface for the FBM320
 *          digital barometer. The driver manages device initialization, periodic
 *          sampling, and temperature-compensated pressure calculations.
 *          
 *          Device Communication:
 *          - I2C interface operating at standard or fast mode
 *          - Supports two I2C addresses: 0x6C (default) or 0x6D (alternate)
 *          - Asynchronous read operations via HAL scheduler callbacks
 *          
 *          Measurement Process:
 *          1. Device initialization and calibration coefficient retrieval
 *          2. Alternating temperature and pressure measurements via timer callback
 *          3. Raw ADC values compensated using factory calibration coefficients
 *          4. Results accumulated and averaged before reporting to AP_Baro frontend
 *          
 *          Calibration Coefficients:
 *          The FBM320 stores 13 factory-calibrated coefficients (C0-C12) in OTP memory.
 *          These coefficients are read once during initialization and used for all
 *          subsequent temperature and pressure compensation calculations.
 *          
 *          Output Units:
 *          - Pressure: Pascals (Pa)
 *          - Temperature: Degrees Celsius (°C)
 *          
 *          Thread Safety:
 *          All device access is serialized through the HAL Device interface.
 *          Timer callbacks execute in scheduler context with appropriate locking.
 * 
 * @note The FBM320 requires specific timing between measurement initiation and
 *       result readout. The driver manages this through the HAL scheduler.
 * 
 * @warning Altitude calculations depend on accurate pressure measurements.
 *          Ensure proper sensor mounting and static pressure port connection.
 */
class AP_Baro_FBM320 : public AP_Baro_Backend {
public:
    /**
     * @brief Construct a new FBM320 barometer backend driver
     * 
     * @param[in] baro Reference to the AP_Baro frontend instance
     * @param[in] dev  Ownership-transferring pointer to the HAL I2C/SPI device
     * 
     * @details Creates a new FBM320 driver instance and transfers ownership of the
     *          device handle. The constructor does not initialize the hardware; call
     *          init() to complete device setup.
     */
    AP_Baro_FBM320(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * @brief Update barometer state with latest accumulated measurements
     * 
     * @details Called periodically by AP_Baro frontend (typically 20 Hz). This method
     *          retrieves accumulated pressure and temperature samples from the timer
     *          callback, computes averages, and reports them to the frontend via
     *          _copy_to_frontend(). Implements the AP_Baro_Backend interface.
     * 
     * @note This method does not directly communicate with the sensor. Actual sensor
     *       reads occur asynchronously in the timer() callback at the scheduler rate.
     */
    void update() override;

    /**
     * @brief Probe for and initialize an FBM320 barometer on the given device
     * 
     * @param[in] baro Reference to the AP_Baro frontend instance
     * @param[in] dev  Ownership-transferring pointer to the HAL device to probe
     * 
     * @return Pointer to new AP_Baro_FBM320 instance if probe successful, nullptr otherwise
     * 
     * @details This static factory method attempts to detect and initialize an FBM320
     *          sensor on the provided device interface. The probe sequence:
     *          1. Reads and validates device ID registers
     *          2. Loads factory calibration coefficients from OTP memory
     *          3. Configures measurement parameters
     *          4. Registers periodic timer callback for sampling
     *          
     *          If successful, returns a fully initialized backend instance. On failure,
     *          returns nullptr and device ownership is released.
     * 
     * @note This method is called during AP_Baro driver probing phase at startup
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    /**
     * @brief Initialize the FBM320 sensor hardware
     * 
     * @return true if initialization successful, false otherwise
     * 
     * @details Performs complete device initialization sequence:
     *          1. Soft reset of the sensor
     *          2. Read and validate calibration coefficients
     *          3. Configure oversampling and measurement timing
     *          4. Register sensor instance with AP_Baro frontend
     *          5. Start periodic timer callback for sampling
     * 
     * @note Called once during probe sequence. Failure returns false, causing
     *       probe to return nullptr.
     */
    bool init(void);
    
    /**
     * @brief Read factory calibration coefficients from FBM320 OTP memory
     * 
     * @return true if calibration read and validated successfully, false otherwise
     * 
     * @details The FBM320 stores 13 factory-calibrated coefficients in one-time
     *          programmable (OTP) memory. This method reads all coefficients and
     *          stores them in the calibration struct for use in temperature and
     *          pressure compensation calculations.
     *          
     *          Calibration coefficients:
     *          - C0-C12: Device-specific compensation parameters
     *          - Different bit widths (16-bit and 32-bit values)
     *          - Must be read in specific sequence per datasheet
     * 
     * @note Calibration is read once during initialization and remains constant
     *       throughout operation. Values are unique to each sensor.
     */
    bool read_calibration(void);
    
    /**
     * @brief Periodic timer callback for sensor measurements
     * 
     * @details Called by HAL scheduler at regular intervals (typically 50-100 Hz).
     *          Implements state machine alternating between temperature and pressure
     *          measurements:
     *          - Even steps: Initiate and read temperature measurement
     *          - Odd steps: Initiate and read pressure measurement
     *          
     *          Raw ADC values are processed through calculate_PT() for compensation,
     *          then accumulated in pressure_sum and temperature_sum for averaging
     *          by the update() method.
     * 
     * @note Runs in scheduler callback context with device locking handled by HAL
     */
    void timer(void);
    
    /**
     * @brief Calculate compensated pressure and temperature from raw ADC values
     * 
     * @param[in]  UT          Raw temperature ADC value from sensor
     * @param[in]  UP          Raw pressure ADC value from sensor
     * @param[out] pressure    Compensated pressure in Pascals (Pa)
     * @param[out] temperature Compensated temperature in Celsius (°C)
     * 
     * @details Implements the FBM320 compensation algorithm using factory calibration
     *          coefficients. The algorithm performs multi-stage calculations:
     *          
     *          Temperature compensation:
     *          1. Calculate temperature offset using C0, C1 coefficients
     *          2. Apply temperature scaling with C2, C3 coefficients
     *          3. Convert to actual temperature in Celsius
     *          
     *          Pressure compensation:
     *          1. Calculate pressure base using C4, C5, C6 coefficients
     *          2. Apply temperature correction using C7, C8, C9 coefficients
     *          3. Apply second-order corrections using C10, C11, C12 coefficients
     *          4. Convert to pressure in Pascals
     *          
     *          The compensation equations are specified in the FBM320 datasheet
     *          and implement polynomial corrections for temperature effects on
     *          pressure sensing and sensor non-linearity.
     * 
     * @note Output units: pressure in Pascals (Pa), temperature in Celsius (°C)
     * @note Algorithm uses integer math for efficiency, with appropriate scaling
     */
    void calculate_PT(int32_t UT, int32_t UP, int32_t &pressure, int32_t &temperature);

    /// HAL device handle for I2C communication with FBM320
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    /// AP_Baro frontend instance number for this sensor
    uint8_t instance;

    /// Number of samples accumulated since last update() call
    uint32_t count;
    
    /// Accumulated pressure sum for averaging (Pascals)
    float pressure_sum;
    
    /// Accumulated temperature sum for averaging (Celsius)
    float temperature_sum;
    
    /// State machine step counter (alternates temperature/pressure measurements)
    uint8_t step;

    /// Most recent raw temperature ADC value, used for pressure compensation
    int32_t value_T;

    /**
     * @struct fbm320_calibration
     * @brief Factory calibration coefficients for FBM320 sensor
     * 
     * @details The FBM320 stores device-specific calibration coefficients in OTP
     *          (one-time programmable) memory during manufacturing. These coefficients
     *          are essential for accurate temperature and pressure calculations and
     *          must be read during initialization.
     *          
     *          Coefficient usage:
     *          - C0-C3:   Temperature compensation parameters
     *          - C4-C7:   Pressure base calculation parameters
     *          - C8-C12:  Pressure temperature correction and non-linearity compensation
     *          
     *          The specific compensation equations are detailed in the FBM320 datasheet.
     *          Coefficients have varying bit widths (16-bit and 32-bit) as specified
     *          by the manufacturer.
     * 
     * @note Values are unique to each sensor and must not be modified
     * @note Stored as raw register values; scaling applied in calculate_PT()
     */
    struct fbm320_calibration {
        uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12;  ///< 16-bit calibration coefficients
        uint32_t C4, C5, C7;                                  ///< 32-bit calibration coefficients
    } calibration;
};

#endif  // AP_BARO_FBM320_ENABLED
