/**
 * @file AP_Baro_SPL06.h
 * @brief Driver for Goertek SPL06 and SPA06 digital barometric pressure sensors
 * 
 * @details This driver implements support for the Goertek SPL06-001 and SPA06-007
 *          high-precision digital barometric pressure sensors. These devices provide
 *          pressure and temperature measurements over I2C interface with internal
 *          calibration coefficients for compensation.
 * 
 *          Key features:
 *          - Pressure range: 300-1200 hPa (SPL06), 300-1100 hPa (SPA06)
 *          - High accuracy: ±0.5 hPa (typ)
 *          - I2C interface (400 kHz max)
 *          - Internal FIFO for burst sampling (when available)
 *          - Device-specific compensation algorithms for SPL06 vs SPA06 variants
 * 
 *          The driver handles device detection, calibration coefficient loading,
 *          continuous pressure/temperature sampling, and applies manufacturer-specified
 *          compensation polynomials to raw sensor data.
 * 
 * @note Requires I2C bus support in HAL implementation
 * @note Default I2C addresses: 0x76, 0x77 (configurable via HAL defines)
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_SPL06_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#ifndef HAL_BARO_SPL06_I2C_ADDR
 #define HAL_BARO_SPL06_I2C_ADDR  (0x76)
#endif
#ifndef HAL_BARO_SPL06_I2C_ADDR2
 #define HAL_BARO_SPL06_I2C_ADDR2 (0x77)
#endif

/**
 * @class AP_Baro_SPL06
 * @brief Barometer driver for Goertek SPL06-001 and SPA06-007 digital pressure sensors
 * 
 * @details This backend implements the AP_Baro interface for SPL06 and SPA06 family
 *          barometric pressure sensors. The driver automatically detects device type
 *          (SPL06 vs SPA06) and applies appropriate compensation algorithms.
 * 
 *          Device Variants:
 *          - SPL06-001: Original version with specific register sequence requirements
 *          - SPA06-007: Updated version with improved FIFO and timing characteristics
 * 
 *          Compensation Algorithm:
 *          The driver reads 9 calibration coefficients (c0, c1, c00, c10, c01, c11,
 *          c20, c21, c30) from device registers and applies device-specific polynomials
 *          to compensate raw pressure and temperature readings.
 * 
 *          Temperature compensation polynomial:
 *          T_comp = c0 * 0.5 + c1 * T_raw_scaled
 * 
 *          Pressure compensation polynomial:
 *          P_comp = c00 + P_raw_scaled * (c10 + P_raw_scaled * (c20 + P_raw_scaled * c30))
 *                   + T_raw_scaled * c01 + T_raw_scaled * P_raw_scaled * (c11 + P_raw_scaled * c21)
 * 
 *          Sampling Strategy:
 *          - Continuous background sampling at ~25Hz via timer callback
 *          - Accumulated pressure samples averaged in update() for noise reduction
 *          - Split-transfer timing used to work around device quirks in register access
 *          - FIFO burst mode available on SPA06 for higher sample rates
 * 
 *          Thread Safety:
 *          - Timer callback runs in scheduler context (typically 1kHz)
 *          - update() called from main thread at ~50Hz
 *          - Uses atomic accumulation for pressure averaging
 * 
 * @note All pressure values output in Pascals (Pa)
 * @note All temperature values output in Celsius (°C)
 * @warning Device requires specific register access timing - do not modify _timer() sequence
 */
class AP_Baro_SPL06 : public AP_Baro_Backend
{
public:
    /**
     * @enum Type
     * @brief Device variant identification for SPL06 family sensors
     * 
     * @details Different variants require specific compensation algorithms and
     *          register access sequences. Type is auto-detected during probe().
     */
	enum class Type {
		UNKNOWN,  ///< Device type not yet identified
		SPL06,    ///< SPL06-001 original version
		SPA06,    ///< SPA06-007 updated version with improved characteristics
	};

    /**
     * @brief Constructor for SPL06/SPA06 barometer driver instance
     * 
     * @details Initializes the driver with a reference to the baro frontend and
     *          takes ownership of the I2C device handle. Actual hardware initialization
     *          is performed in _init() called from probe().
     * 
     * @param[in] baro Reference to AP_Baro frontend for sensor registration
     * @param[in] dev  I2C device handle (ownership transferred to driver)
     * 
     * @note Constructor does not perform I2C communication - hardware init in _init()
     * @see probe() for device detection and initialization
     */
    AP_Baro_SPL06(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    
    /**
     * @brief Update frontend with latest pressure and temperature readings
     * 
     * @details Called periodically (typically 50Hz) from main thread to transfer
     *          accumulated sensor data to the AP_Baro frontend. This method:
     *          1. Retrieves accumulated pressure samples from timer callback
     *          2. Computes average pressure to reduce measurement noise
     *          3. Updates frontend with pressure (Pa) and temperature (°C)
     *          4. Resets accumulation counters for next period
     * 
     *          Averaging reduces high-frequency noise from sensor and improves
     *          altitude hold performance. Typical accumulation: 2-5 samples between
     *          update() calls depending on timer frequency.
     * 
     * @note Called from main thread context at AP_Baro update rate
     * @note Must be called regularly to prevent sample accumulator overflow
     * @see _timer() for background sample acquisition
     */
    void update() override;

    /**
     * @brief Factory method to detect and initialize SPL06/SPA06 sensor
     * 
     * @details Static probe function that attempts to detect SPL06 or SPA06 device
     *          on the provided I2C bus. Probe sequence:
     *          1. Attempt I2C communication at standard addresses (0x76, 0x77)
     *          2. Read device ID register to identify SPL06 vs SPA06 variant
     *          3. Verify device responds correctly to register reads
     *          4. Create driver instance and initialize hardware (_init)
     *          5. Read and parse calibration coefficients from device
     *          6. Configure measurement rates and modes
     *          7. Register periodic timer callback for continuous sampling
     * 
     *          Device Detection:
     *          - SPL06: ID register 0x0D reads 0x10
     *          - SPA06: ID register 0x0D reads 0x12
     * 
     * @param[in] baro Reference to AP_Baro frontend for registration
     * @param[in] dev  I2C device handle to attempt probe (ownership transferred if successful)
     * 
     * @return Pointer to new AP_Baro_SPL06 instance if device detected and initialized successfully
     * @return nullptr if device not detected or initialization failed
     * 
     * @note This is the preferred method for creating SPL06 driver instances
     * @note Device handle ownership transferred only on successful probe
     * @see _init() for hardware initialization details
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:

    /**
     * @brief Initialize SPL06/SPA06 hardware and configure measurement parameters
     * 
     * @details Performs complete hardware initialization sequence:
     *          1. Read and verify device ID register
     *          2. Perform soft reset via control register
     *          3. Read calibration coefficients from device EEPROM (registers 0x10-0x21)
     *          4. Parse calibration data accounting for SPL06 vs SPA06 differences
     *          5. Configure pressure measurement rate and oversampling
     *          6. Configure temperature measurement rate and oversampling
     *          7. Set continuous measurement mode
     *          8. Apply device-specific register sequences for timing workarounds
     * 
     *          Calibration Coefficient Parsing:
     *          SPL06/SPA06 store 18 bytes of calibration data in signed/unsigned formats
     *          requiring careful byte order and sign extension handling per datasheet.
     * 
     * @return true if initialization successful (device responds and coefficients valid)
     * @return false if I2C communication fails or invalid calibration data detected
     * 
     * @note Must be called before starting timer callbacks
     * @warning Device-specific register access timing critical - follow datasheet sequence
     */
    bool _init(void);
    
    /**
     * @brief Timer callback for continuous pressure/temperature sampling
     * 
     * @details Periodic callback (typically 1kHz scheduler rate) that manages:
     *          - Alternating pressure and temperature measurements
     *          - Reading raw sensor values from data registers
     *          - Applying compensation algorithms via _update_pressure() and _update_temperature()
     *          - Accumulating pressure samples for averaging in update()
     *          - Implementing split-transfer timing for device quirks
     * 
     *          Measurement Sequence (repeated every N timer ticks):
     *          1. Read pressure data registers (0x00-0x02) as 24-bit value
     *          2. Apply pressure compensation polynomial with temperature correction
     *          3. Accumulate compensated pressure for averaging
     *          4. Periodically read temperature data registers (0x03-0x05)
     *          5. Update temperature compensation value
     * 
     *          Device Quirks:
     *          SPL06 requires specific timing between register reads - use split transfers
     *          to ensure minimum inter-byte delays per datasheet requirements.
     * 
     * @note Called from scheduler context at high frequency (~1kHz)
     * @note Uses atomic operations for accumulation accessed by update()
     * @warning Timing-sensitive - modifications may break device communication
     */
    void _timer(void);
    
    /**
     * @brief Update temperature compensation value from raw sensor reading
     * 
     * @details Applies device calibration coefficients to raw temperature value
     *          using manufacturer-specified compensation polynomial:
     *          T_compensated = c0 * 0.5 + c1 * T_raw_scaled
     * 
     *          Raw value scaling depends on configured oversampling rate.
     * 
     * @param[in] temp_raw Raw 24-bit temperature reading from sensor registers
     * 
     * @note Updates internal _temperature member used for pressure compensation
     * @note Temperature in Celsius (°C)
     */
    void _update_temperature(int32_t temp_raw);
    
    /**
     * @brief Update and accumulate compensated pressure from raw sensor reading
     * 
     * @details Applies full pressure compensation algorithm including temperature
     *          correction using calibration coefficients c00, c10, c20, c30, c01,
     *          c11, c21 per datasheet polynomial:
     * 
     *          P_comp = c00 + P_raw_scaled * (c10 + P_raw_scaled * (c20 + P_raw_scaled * c30))
     *                   + T_raw_scaled * c01 
     *                   + T_raw_scaled * P_raw_scaled * (c11 + P_raw_scaled * c21)
     * 
     *          Compensated pressure accumulated in _pressure_sum for averaging in update().
     * 
     * @param[in] press_raw Raw 24-bit pressure reading from sensor registers
     * 
     * @note Adds to _pressure_sum accumulator (atomically)
     * @note Increments _pressure_count for average calculation
     * @note Pressure in Pascals (Pa)
     */
    void _update_pressure(int32_t press_raw);

    /**
     * @brief Calculate raw value scale factor based on oversampling configuration
     * 
     * @details SPL06/SPA06 raw values must be scaled based on oversampling rate
     *          configured in measurement control registers. Scale factors are powers
     *          of 2 corresponding to oversampling rates per datasheet table.
     * 
     * @param[in] reg Oversampling configuration register value
     * 
     * @return Scale factor to apply to raw sensor readings
     * 
     * @note Different scale factors for pressure vs temperature measurements
     */
    int32_t raw_value_scale_factor(uint8_t reg);

    // Hardware interface
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< I2C device handle for sensor communication

    // State management
    int8_t _timer_counter;   ///< Counter for alternating pressure/temperature reads in timer
    uint8_t _instance;       ///< Baro frontend instance number for this sensor
    
    // Measurement data (intermediate and accumulated)
    float _temp_raw;         ///< Scaled raw temperature value for pressure compensation
    float _pressure_sum;     ///< Accumulated pressure samples (Pa) for averaging
    uint32_t _pressure_count; ///< Number of pressure samples accumulated since last update()
    float _temperature;      ///< Latest compensated temperature reading (°C)

    /**
     * @brief Device calibration coefficients loaded from EEPROM
     * 
     * @details These coefficients are device-specific values programmed during
     *          manufacturing and stored in sensor's internal EEPROM (registers 0x10-0x21).
     *          Used in compensation polynomials to convert raw sensor readings to
     *          calibrated pressure and temperature values.
     * 
     *          Coefficient Types:
     *          - c0, c1: Temperature compensation (c0 is 12-bit, c1 is 16-bit)
     *          - c00, c10: Pressure base and linear (20-bit values)
     *          - c01, c11, c20, c21, c30: Higher-order pressure correction terms (16-bit)
     *          - c31, c40: Extended coefficients (SPA06 only, may be unused in SPL06)
     * 
     * @note Coefficients parsed with careful attention to signed/unsigned types per datasheet
     * @note Invalid coefficients (all zeros or all 0xFF) indicate device failure
     */
    int32_t _c00, _c10;  ///< 20-bit pressure calibration coefficients
    int16_t _c0, _c1, _c01, _c11, _c20, _c21, _c30, _c31, _c40;  ///< Temperature and pressure correction coefficients

    Type type;  ///< Detected device variant (SPL06 or SPA06) from probe()
};

#endif  // AP_BARO_SPL06_ENABLED
