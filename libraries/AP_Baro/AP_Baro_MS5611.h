/**
 * @file AP_Baro_MS5611.h
 * @brief Driver for TE Connectivity MS56XX family barometric pressure sensors
 * 
 * @details This file implements support for the TE Connectivity (formerly Measurement
 *          Specialties) MS56XX family of high-resolution barometric pressure sensors:
 *          - MS5611: High-resolution barometric pressure sensor (150 mbar to 1200 mbar)
 *          - MS5607: Low-power variant with similar resolution
 *          - MS5637: Miniature barometric pressure sensor
 *          - MS5837: Water pressure sensor supporting both air and water pressure measurement
 * 
 *          Key features:
 *          - PROM (Programmable Read-Only Memory) calibration coefficients with CRC validation
 *          - Configurable OSR (OverSampling Ratio) for ADC conversion precision
 *          - Two-stage measurement: temperature conversion followed by pressure conversion
 *          - 2nd-order temperature compensation algorithm per datasheet specifications
 *          - I2C and SPI interface support (board-dependent)
 * 
 *          Measurement units:
 *          - Pressure: Pascals (Pa)
 *          - Temperature: Celsius (°C)
 * 
 * @note MS5837 supports both air and water pressure measurement modes for underwater vehicles
 * 
 * Source: libraries/AP_Baro/AP_Baro_MS5611.h
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_MS56XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_MS5611_I2C_ADDR
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5611_I2C_ADDR2
#define HAL_BARO_MS5611_I2C_ADDR2 0x76
#endif

#ifndef HAL_BARO_MS5607_I2C_ADDR
#define HAL_BARO_MS5607_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5837_I2C_ADDR
#define HAL_BARO_MS5837_I2C_ADDR 0x76
#endif

#ifndef HAL_BARO_MS5637_I2C_ADDR
#define HAL_BARO_MS5637_I2C_ADDR 0x76
#endif

#if AP_BARO_MS5837_ENABLED
// Determined in https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
#define MS5837_30BA_02BA_SELECTION_THRESHOLD 37000
#endif

/**
 * @class AP_Baro_MS56XX
 * @brief Base class for TE Connectivity MS56XX family barometric pressure sensors
 * 
 * @details This class provides common functionality for all MS56XX family sensors including:
 *          - PROM reading with CRC validation for factory calibration coefficients
 *          - Two-stage ADC conversion (temperature then pressure)
 *          - Sample accumulation and averaging for noise reduction
 *          - 2nd-order temperature compensation algorithm
 *          - I2C/SPI device communication abstraction
 * 
 *          Device operation sequence:
 *          1. Initialization: Read and validate PROM calibration coefficients (C1-C6)
 *          2. Periodic sampling: Trigger D1 (pressure) and D2 (temperature) ADC conversions
 *          3. Accumulation: Average multiple samples for improved resolution
 *          4. Calculation: Apply 2nd-order temperature compensation per datasheet
 *          5. Update: Report calibrated pressure and temperature to AP_Baro frontend
 * 
 *          Supported device variants:
 *          - MS5611: High-resolution, standard atmospheric pressure range
 *          - MS5607: Low-power variant with similar characteristics
 *          - MS5637: Miniature form factor
 *          - MS5837: Water pressure sensor with extended range for underwater applications
 * 
 * @note OSR (OverSampling Ratio) configures ADC precision vs conversion time trade-off
 * @warning PROM CRC validation is critical - device must not be used if CRC fails
 * 
 * Source: libraries/AP_Baro/AP_Baro_MS5611.h:36-97
 */
class AP_Baro_MS56XX : public AP_Baro_Backend
{
public:
    /**
     * @brief Update barometer readings with accumulated samples
     * 
     * @details Called by AP_Baro frontend to retrieve latest pressure and temperature readings.
     *          This method:
     *          - Retrieves accumulated D1 (pressure) and D2 (temperature) ADC samples
     *          - Averages samples to reduce measurement noise
     *          - Applies device-specific calibration and compensation algorithms
     *          - Reports final pressure (Pa) and temperature (°C) to frontend
     * 
     *          The averaging process improves resolution by combining multiple ADC conversions
     *          taken at high OSR (OverSampling Ratio) settings.
     * 
     * @note This method is called at the barometer update rate (typically 10-50 Hz)
     * @see _calculate() for device-specific compensation algorithms
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    void update() override;

    /**
     * @brief Construct MS56XX barometer backend
     * 
     * @param[in] baro  Reference to AP_Baro frontend managing all barometer instances
     * @param[in] dev   HAL device abstraction for I2C or SPI communication
     * 
     * @details Initializes the barometer backend with the specified communication device.
     *          Actual sensor initialization (PROM reading, validation) occurs in _init().
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    AP_Baro_MS56XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

protected:

    /**
     * @brief Factory method for probing and initializing MS56XX sensor
     * 
     * @param[in] baro    Reference to AP_Baro frontend
     * @param[in] sensor  Pre-constructed MS56XX sensor instance to initialize
     * 
     * @return Pointer to initialized backend on success, nullptr if probe/init fails
     * 
     * @details This method performs device detection and initialization:
     *          1. Reads PROM (Programmable Read-Only Memory) calibration coefficients
     *          2. Validates PROM CRC to ensure data integrity
     *          3. Configures ADC conversion timing and OSR settings
     *          4. Registers periodic timer callback for continuous sampling
     * 
     *          Device variants are distinguished by PROM contents and I2C addresses.
     * 
     * @note Returns nullptr if PROM CRC validation fails or device does not respond
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    static AP_Baro_Backend *_probe(AP_Baro &baro, AP_Baro_MS56XX *sensor);

    /**
     * @brief Initialize MS56XX sensor and validate PROM
     * 
     * @return true if initialization successful, false on failure
     * 
     * @details Reads factory calibration coefficients from PROM and validates CRC.
     *          Derived classes may override for device-specific initialization.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    virtual bool _init();

    /**
     * @brief Read PROM calibration coefficients for MS5611/MS5607 devices
     * 
     * @param[out] prom  Array of 8 words containing calibration data and CRC
     * 
     * @return true if PROM read and CRC validation successful, false otherwise
     * 
     * @details PROM layout for MS5611/MS5607:
     *          - prom[0]: Reserved/manufacturer data
     *          - prom[1]: C1 (Pressure sensitivity | SENS_T1)
     *          - prom[2]: C2 (Pressure offset | OFF_T1)
     *          - prom[3]: C3 (Temperature coefficient of pressure sensitivity | TCS)
     *          - prom[4]: C4 (Temperature coefficient of pressure offset | TCO)
     *          - prom[5]: C5 (Reference temperature | T_REF)
     *          - prom[6]: C6 (Temperature coefficient of temperature | TEMPSENS)
     *          - prom[7]: CRC bits[3:0] + serial code
     * 
     * @warning Device must not be used if CRC validation fails
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    bool _read_prom_5611(uint16_t prom[8]);

    /**
     * @brief Read PROM calibration coefficients for MS5637/MS5837 devices
     * 
     * @param[out] prom  Array of 8 words containing calibration data and CRC
     * 
     * @return true if PROM read and CRC validation successful, false otherwise
     * 
     * @details MS5637/MS5837 use different PROM read commands than MS5611/MS5607.
     *          Calibration coefficient structure is similar but command protocol differs.
     * 
     * @warning Device must not be used if CRC validation fails
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    bool _read_prom_5637(uint16_t prom[8]);

    /**
     * @brief Get device name string
     * @return Device name (e.g., "MS5611", "MS5837")
     */
    virtual const char *name() const = 0;

    /**
     * @brief Get device type identifier
     * @return DevTypes enumeration value for this sensor variant
     */
    virtual DevTypes devtype() const = 0;

    /// @brief AP_Baro instance identifier for this sensor
    uint8_t _instance;

    /**
     * @brief Last compensated ADC values from accumulated samples
     * @details _D1: Raw pressure ADC value averaged over accumulation period
     *          _D2: Raw temperature ADC value averaged over accumulation period
     *          These values are used in _calculate() for 2nd-order temperature compensation.
     */
    float _D1, _D2;

    /**
     * @brief Internal calibration registers from PROM
     * @details Factory-programmed calibration coefficients read from device PROM:
     *          - c1 (SENS_T1): Pressure sensitivity at reference temperature
     *          - c2 (OFF_T1):  Pressure offset at reference temperature
     *          - c3 (TCS):     Temperature coefficient of pressure sensitivity
     *          - c4 (TCO):     Temperature coefficient of pressure offset
     *          - c5 (T_REF):   Reference temperature
     *          - c6 (TEMPSENS): Temperature coefficient of temperature
     * 
     *          These coefficients are used in the 2nd-order temperature compensation
     *          algorithm to calculate accurate pressure and temperature values.
     * 
     * @note Coefficients are validated via CRC before use
     */
    struct {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;

private:

    /**
     * @brief Update accumulator with new sample and handle wrapping
     * 
     * @param[in,out] accum      Running sum of samples
     * @param[in]     val        New sample value to add
     * @param[in,out] count      Number of samples accumulated
     * @param[in]     max_count  Maximum samples before wrapping
     * 
     * @details Accumulates ADC samples for averaging. When max_count is reached,
     *          the accumulator and count are reset to prevent overflow while
     *          maintaining continuous sampling.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    /**
     * @brief Read single 16-bit word from PROM
     * 
     * @param[in] word  PROM word address (0-7)
     * @return 16-bit PROM value
     * 
     * @details PROM addresses 0-7 contain calibration coefficients and CRC.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    uint16_t _read_prom_word(uint8_t word);

    /**
     * @brief Read 24-bit ADC conversion result
     * 
     * @return 24-bit ADC value (D1 for pressure or D2 for temperature)
     * 
     * @details Reads the result of the previously triggered ADC conversion.
     *          Conversion time depends on OSR (OverSampling Ratio) setting:
     *          - OSR=256:  0.5 ms conversion time
     *          - OSR=512:  1.1 ms conversion time
     *          - OSR=1024: 2.1 ms conversion time
     *          - OSR=2048: 4.1 ms conversion time
     *          - OSR=4096: 8.2 ms conversion time
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    uint32_t _read_adc();

    /**
     * @brief Timer callback for two-stage ADC conversion and accumulation
     * 
     * @details Implements state machine for continuous sampling:
     *          1. Trigger D2 (temperature) ADC conversion
     *          2. Wait for conversion (OSR-dependent)
     *          3. Read D2 result and accumulate
     *          4. Trigger D1 (pressure) ADC conversion
     *          5. Wait for conversion (OSR-dependent)
     *          6. Read D1 result and accumulate
     *          7. Repeat
     * 
     *          This two-stage measurement is required because temperature affects
     *          pressure readings and must be measured for compensation.
     * 
     * @note Called at timer interrupt rate configured during initialization
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    void _timer();

    /// @brief HAL device abstraction for I2C or SPI communication
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Sample accumulation structure shared between timer and main threads
     * @details Thread-safe accumulator for averaging multiple ADC samples:
     *          - s_D1: Sum of D1 (pressure) samples
     *          - s_D2: Sum of D2 (temperature) samples
     *          - d1_count: Number of D1 samples accumulated
     *          - d2_count: Number of D2 samples accumulated
     * 
     *          Accessed from timer interrupt context (_timer) and main thread (update).
     *          Protected by device semaphore to ensure atomic updates.
     */
    struct {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    /// @brief State machine variable for two-stage ADC conversion sequence
    uint8_t _state;

    /// @brief Flag to discard next sample after configuration change
    bool _discard_next;

    /**
     * @brief Read and validate PROM calibration data
     * 
     * @param[out] prom  Array to store 8 PROM words
     * @return true if PROM read and CRC validation successful
     * 
     * @details Device-specific PROM reading implementation.
     *          Must validate CRC before accepting calibration data.
     */
    virtual bool _read_prom(uint16_t *prom) = 0;

    /**
     * @brief Calculate compensated pressure and temperature
     * 
     * @details Applies 2nd-order temperature compensation algorithm using:
     *          - Accumulated D1 (pressure) and D2 (temperature) samples
     *          - Factory calibration coefficients (c1-c6)
     *          - Device-specific compensation formulas from datasheet
     * 
     *          Algorithm corrects for temperature effects on pressure measurement
     *          and provides both compensated pressure (Pa) and temperature (°C).
     * 
     * @note Implementation varies by device (MS5611 vs MS5837 have different algorithms)
     */
    virtual void _calculate() = 0;
};

#if AP_BARO_MS5607_ENABLED
/**
 * @class AP_Baro_MS5607
 * @brief Driver for MS5607 low-power barometric pressure sensor
 * 
 * @details MS5607 is a low-power variant of MS5611 with similar characteristics:
 *          - Pressure range: 10 to 1200 mbar
 *          - Resolution: 0.012 mbar (10 cm altitude)
 *          - Low power consumption optimized for battery operation
 *          - Uses same PROM structure and compensation algorithm as MS5611
 * 
 * Source: libraries/AP_Baro/AP_Baro_MS5611.h:100-111
 */
class AP_Baro_MS5607 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;

    /**
     * @brief Factory method to detect and initialize MS5607 sensor
     * 
     * @param[in] baro  Reference to AP_Baro frontend
     * @param[in] dev   HAL device for I2C or SPI communication
     * 
     * @return Pointer to initialized backend on success, nullptr on failure
     * 
     * @details Attempts to communicate with MS5607 at configured I2C address,
     *          reads and validates PROM, then initializes sampling if successful.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    const char *name() const override { return "MS5607"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5611(prom); }
    DevTypes devtype() const override { return DEVTYPE_BARO_MS5607; }
    void _calculate() override;
};
#endif  // AP_BARO_MS5607_ENABLED

#if AP_BARO_MS5611_ENABLED
/**
 * @class AP_Baro_MS5611
 * @brief Driver for MS5611 high-resolution barometric pressure sensor
 * 
 * @details MS5611 is a high-resolution barometric pressure sensor with:
 *          - Pressure range: 10 to 1200 mbar (equivalent to altitude +9000m to -500m)
 *          - Operating temperature: -40°C to +85°C
 *          - Resolution: 0.012 mbar (10 cm altitude resolution)
 *          - Fast conversion time with configurable OSR
 *          - Factory-calibrated with PROM coefficients and CRC validation
 * 
 *          Interface support:
 *          - I2C: Standard (0x76) and alternate (0x77) addresses
 *          - SPI: Mode 0 or Mode 3 (board-dependent)
 * 
 *          Measurement process:
 *          1. Read D2 (digital temperature value) - 24-bit ADC result
 *          2. Read D1 (digital pressure value) - 24-bit ADC result
 *          3. Apply 2nd-order temperature compensation using PROM coefficients
 *          4. Output calibrated pressure (Pa) and temperature (°C)
 * 
 * @note This is the most commonly used sensor in the MS56XX family for multicopters
 * @warning PROM CRC must validate successfully before using sensor
 * 
 * Source: libraries/AP_Baro/AP_Baro_MS5611.h:114-125
 */
class AP_Baro_MS5611 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;

    /**
     * @brief Factory method to detect and initialize MS5611 sensor
     * 
     * @param[in] baro  Reference to AP_Baro frontend
     * @param[in] dev   HAL device for I2C or SPI communication
     * 
     * @return Pointer to initialized backend on success, nullptr on failure
     * 
     * @details Probes MS5611 at configured I2C addresses (0x76, 0x77) or via SPI.
     *          Performs device detection by:
     *          1. Reading PROM calibration coefficients
     *          2. Validating PROM CRC
     *          3. Initializing timer-based sampling
     * 
     *          Device variant is distinguished from other MS56XX family members
     *          by PROM contents and successful communication at MS5611 addresses.
     * 
     * @note Returns nullptr if device not found or PROM CRC validation fails
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    const char *name() const override { return "MS5611"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5611(prom); }
    DevTypes devtype() const override { return DEVTYPE_BARO_MS5611; }
    void _calculate() override;
};
#endif  // AP_BARO_MS5611_ENABLED

#if AP_BARO_MS5637_ENABLED
/**
 * @class AP_Baro_MS5637
 * @brief Driver for MS5637 miniature barometric pressure sensor
 * 
 * @details MS5637 is a miniature, low-power barometric pressure sensor:
 *          - Pressure range: 300 to 1200 mbar
 *          - Resolution: 0.012 mbar
 *          - Ultra-small 3x3mm QFN package
 *          - I2C interface at address 0x76
 *          - Different PROM read protocol than MS5611/MS5607
 * 
 *          Key differences from MS5611:
 *          - Uses _read_prom_5637() for PROM access (different command set)
 *          - Smaller package suitable for space-constrained applications
 *          - Similar compensation algorithm to MS5611
 * 
 * Source: libraries/AP_Baro/AP_Baro_MS5611.h:128-139
 */
class AP_Baro_MS5637 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;

    /**
     * @brief Factory method to detect and initialize MS5637 sensor
     * 
     * @param[in] baro  Reference to AP_Baro frontend
     * @param[in] dev   HAL device for I2C communication
     * 
     * @return Pointer to initialized backend on success, nullptr on failure
     * 
     * @details Probes MS5637 at I2C address 0x76 using device-specific PROM protocol.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    const char *name() const override { return "MS5637"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5637(prom); }
    DevTypes devtype() const override { return DEVTYPE_BARO_MS5637; }
    void _calculate() override;
};
#endif  // AP_BARO_MS5637_ENABLED

#if AP_BARO_MS5837_ENABLED
/**
 * @class AP_Baro_MS5837
 * @brief Driver for MS5837 water pressure sensor with air/water pressure support
 * 
 * @details MS5837 is a specialized pressure sensor designed for underwater applications:
 *          - Pressure range: 0 to 30 bar (MS5837-30BA) or 0 to 2 bar (MS5837-02BA)
 *          - Supports both air pressure (atmospheric) and water pressure measurement
 *          - Optimized for depth sensing in underwater vehicles (ArduSub)
 *          - I2C interface at address 0x76
 *          - Two variants with different pressure ranges and compensation algorithms
 * 
 *          MS5837 variants:
 *          - MS5837-02BA: 0-2 bar range, suitable for shallow water (0-20m depth)
 *          - MS5837-30BA: 0-30 bar range, suitable for deep water (0-300m depth)
 * 
 *          Device-specific features:
 *          - Automatic variant detection during initialization
 *          - Variant-specific compensation algorithms (_calculate_5837_02ba vs _calculate_5837_30ba)
 *          - Water density compensation for accurate depth measurement
 *          - Air pressure measurement when vehicle is surfaced
 * 
 *          Variant detection:
 *          During initialization, sensor variant (02BA vs 30BA) is automatically detected
 *          using pressure reading threshold (MS5837_30BA_02BA_SELECTION_THRESHOLD).
 * 
 * @note MS5837 supports both air and water pressure measurement modes for underwater vehicles
 * @note ArduSub uses MS5837 as primary depth sensor when configured
 * @warning Water pressure compensation assumes freshwater density - saltwater requires adjustment
 * 
 * Source: libraries/AP_Baro/AP_Baro_MS5611.h:142-158
 */
class AP_Baro_MS5837 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;

    /**
     * @brief Factory method to detect and initialize MS5837 water pressure sensor
     * 
     * @param[in] baro  Reference to AP_Baro frontend
     * @param[in] dev   HAL device for I2C communication
     * 
     * @return Pointer to initialized backend on success, nullptr on failure
     * 
     * @details Probes MS5837 at I2C address 0x76 and automatically detects variant:
     *          1. Reads PROM using MS5637 protocol
     *          2. Validates PROM CRC
     *          3. Takes initial pressure reading
     *          4. Determines 02BA vs 30BA variant based on pressure threshold
     *          5. Configures device-specific compensation algorithm
     * 
     * @note Variant detection allows single driver to support both 02BA and 30BA
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
protected:
    const char *name() const override { return "MS5837"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5637(prom); }

    /**
     * @brief Get device type, including detected subtype variant
     * @return DEVTYPE_BARO_MS5837_02BA or DEVTYPE_BARO_MS5837_30BA
     */
    DevTypes devtype() const override;

    /**
     * @brief Initialize MS5837 and detect 02BA vs 30BA variant
     * 
     * @return true if initialization and variant detection successful
     * 
     * @details Extended initialization:
     *          1. Call base _init() to read and validate PROM
     *          2. Trigger initial pressure conversion
     *          3. Compare pressure against MS5837_30BA_02BA_SELECTION_THRESHOLD
     *          4. Set _subtype to appropriate DevTypes value
     *          5. Configure variant-specific _calculate() behavior
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    bool _init() override;

    /**
     * @brief Calculate compensated pressure and temperature (variant dispatcher)
     * 
     * @details Calls appropriate variant-specific compensation algorithm:
     *          - _calculate_5837_02ba() for 0-2 bar range sensor
     *          - _calculate_5837_30ba() for 0-30 bar range sensor
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    void _calculate() override;

    /**
     * @brief Apply 2nd-order temperature compensation for MS5837-02BA
     * 
     * @details Implements datasheet algorithm optimized for 0-2 bar pressure range.
     *          Uses calibration coefficients and compensation formulas specific
     *          to the 02BA variant for shallow water depth measurement.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    void _calculate_5837_02ba();

    /**
     * @brief Apply 2nd-order temperature compensation for MS5837-30BA
     * 
     * @details Implements datasheet algorithm optimized for 0-30 bar pressure range.
     *          Uses calibration coefficients and compensation formulas specific
     *          to the 30BA variant for deep water depth measurement.
     * 
     * Source: libraries/AP_Baro/AP_Baro_MS5611.cpp
     */
    void _calculate_5837_30ba();

    /**
     * @brief Detected MS5837 subtype (02BA or 30BA)
     * @details Set during _init() based on initial pressure reading.
     *          Determines which compensation algorithm to use.
     */
    DevTypes _subtype;
};
#endif  // AP_BARO_MS5837_ENABLED

#endif  // AP_BARO_MS56XX_ENABLED
