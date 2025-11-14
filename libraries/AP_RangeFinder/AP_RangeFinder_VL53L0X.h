/**
 * @file AP_RangeFinder_VL53L0X.h
 * @brief Driver for ST Microelectronics VL53L0X time-of-flight (ToF) laser rangefinder
 * 
 * This driver provides support for the VL53L0X I2C-based infrared laser rangefinder.
 * The VL53L0X uses a 940nm VCSEL (Vertical Cavity Surface Emitting Laser) emitter
 * and SPAD (Single Photon Avalanche Diode) detector array to measure distance via
 * time-of-flight calculation.
 * 
 * Key specifications:
 * - Measurement range: 30mm to 2000mm
 * - I2C interface at default address 0x29
 * - Supports continuous and single-shot ranging modes
 * - Configurable measurement timing budget (accuracy vs speed trade-off)
 * - Maximum measurement rate depends on timing budget (minimum ~20ms for high accuracy)
 * 
 * @note The VL53L0X contains factory-calibrated SPAD reference data stored on-chip
 * @warning Minimum timing budget is ~20ms for reliable measurements; shorter budgets
 *          may result in reduced accuracy or failed readings
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_VL53L0X.h
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L0X_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

/**
 * @class AP_RangeFinder_VL53L0X
 * @brief Backend driver for VL53L0X I2C laser time-of-flight rangefinder
 * 
 * @details This class implements the ArduPilot rangefinder backend interface for the
 *          ST VL53L0X ToF sensor. The driver initializes the sensor with optimal
 *          settings for autopilot use, configures continuous ranging mode, and
 *          periodically reads distance measurements via I2C.
 * 
 *          Initialization sequence:
 *          1. Verify sensor chip ID over I2C
 *          2. Load tuning register configuration
 *          3. Retrieve SPAD reference calibration data
 *          4. Configure VCSEL pulse periods and timing budget
 *          5. Perform VHV and phase calibration
 *          6. Start continuous ranging mode
 * 
 *          The sensor operates in continuous mode with periodic I2C reads triggered
 *          by the HAL scheduler at the configured update rate.
 * 
 * @note Uses 940nm IR laser, eye-safe Class 1 laser product under normal operation
 * @warning I2C bus contention can cause measurement delays; ensure adequate timing
 *          budget configured. Multiple VL53L0X sensors require I2C address remapping.
 */
class AP_RangeFinder_VL53L0X : public AP_RangeFinder_Backend
{

public:
    /**
     * @brief Factory method to detect and initialize VL53L0X sensor on I2C bus
     * 
     * @details Attempts to communicate with VL53L0X sensor at default I2C address (0x29).
     *          Performs sensor ID verification by reading chip identification register.
     *          If sensor is detected and responds correctly, allocates and initializes
     *          a new AP_RangeFinder_VL53L0X backend instance.
     * 
     *          Detection sequence:
     *          1. Attempt I2C communication with provided device
     *          2. Read and verify sensor chip ID
     *          3. If valid, construct backend and initialize sensor
     *          4. Return backend pointer or nullptr on failure
     * 
     * @param[in] _state      Reference to RangeFinder state structure for this sensor instance
     * @param[in] _params     Reference to RangeFinder parameters for this sensor instance
     * @param[in] dev         OwnPtr to I2CDevice for communication (ownership transferred to backend)
     * 
     * @return Pointer to new AP_RangeFinder_VL53L0X backend if sensor detected, nullptr if detection failed
     * 
     * @note This method is called during rangefinder initialization to probe for VL53L0X sensors
     * @note Ownership of I2CDevice is transferred to the backend if detection succeeds
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Update rangefinder state with latest distance measurement
     * 
     * @details Called by the rangefinder library at the configured update rate (typically 10-20Hz).
     *          Retrieves accumulated distance measurements from periodic timer callbacks,
     *          averages multiple readings to reduce noise, and updates the rangefinder state
     *          structure with the filtered distance value and status.
     * 
     *          This method does not directly read the sensor; instead it processes data
     *          collected by the timer() callback which runs at higher frequency.
     * 
     * @note Override of AP_RangeFinder_Backend::update()
     * @note Called at main loop rate; actual sensor reads occur in timer() callback
     */
    void update(void) override;

protected:

    /**
     * @brief Return MAVLink distance sensor type identifier
     * 
     * @details Reports the sensor type as MAV_DISTANCE_SENSOR_LASER to ground control
     *          stations. This indicates an infrared laser-based time-of-flight sensor.
     *          The VL53L0X uses a 940nm IR laser (VCSEL) which classifies as a laser
     *          rangefinder in the MAVLink protocol.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER - MAVLink sensor type for IR laser ToF sensors
     * 
     * @note Override of AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @brief Private constructor for VL53L0X rangefinder backend
     * 
     * @param[in] _state   Reference to RangeFinder state structure
     * @param[in] _params  Reference to RangeFinder parameters
     * @param[in] dev      OwnPtr to I2CDevice for sensor communication (ownership transferred)
     * 
     * @note Constructor is private; use detect() factory method to instantiate
     */
    AP_RangeFinder_VL53L0X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Initialize VL53L0X sensor with optimal configuration for autopilot use
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Verify sensor chip ID (check_id)
     *          2. Load tuning register configuration from tuning_data array
     *          3. Retrieve SPAD reference calibration (get_SPAD_info)
     *          4. Configure measurement timing budget (setMeasurementTimingBudget)
     *          5. Perform VHV calibration (performSingleRefCalibration)
     *          6. Perform phase calibration (performSingleRefCalibration)
     *          7. Start continuous ranging mode (start_continuous)
     *          8. Register timer callback with HAL scheduler for periodic reads
     * 
     * @return true if initialization successful, false on any I2C communication failure
     * 
     * @note Marked with __INITFUNC__ to place in initialization memory section on some platforms
     * @note Timing budget configured for balance between accuracy and update rate
     */
    __INITFUNC__ bool init();
    
    /**
     * @brief Timer callback for periodic I2C sensor reads
     * 
     * @details Registered with HAL scheduler to run at high frequency (typically 50-100Hz).
     *          Checks if new measurement data is available, reads distance value over I2C,
     *          and accumulates readings for averaging in update() method.
     * 
     *          Accumulates sum_mm and counter for noise reduction through averaging.
     * 
     * @note Runs in timer interrupt context; keep execution time minimal
     * @note Update rate limited by sensor's measurement timing budget configuration
     */
    void timer();

    /**
     * @brief Verify sensor chip ID over I2C
     * 
     * @details Reads VL53L0X chip identification register and verifies expected value.
     *          Used during initialization to confirm correct sensor is present on I2C bus.
     * 
     * @return true if chip ID matches expected VL53L0X identifier, false otherwise
     * 
     * @note First step in initialization sequence to validate I2C communication
     */
    bool check_id(void);

    /**
     * @brief Read single distance measurement from sensor
     * 
     * @details Reads the latest ranging result register over I2C and returns distance
     *          in centimeters. Does not trigger a new measurement; reads most recent
     *          result from continuous ranging mode.
     * 
     * @param[out] reading_cm  Distance measurement in centimeters (cm)
     * 
     * @return true if read successful and measurement valid, false on I2C error or invalid data
     * 
     * @note Distance returned in centimeters for consistency with ArduPilot rangefinder API
     * @note Sensor native format is millimeters; driver performs mm to cm conversion
     */
    bool get_reading(uint16_t &reading_cm);
    /**
     * @brief Pointer to I2C device interface for sensor communication
     * @note Ownership transferred from detect() method; automatically cleaned up on destruction
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    /**
     * @brief Read 8-bit value from VL53L0X register
     * 
     * @param[in] reg  Register address to read (8-bit I2C register address)
     * 
     * @return 8-bit register value, or 0 on I2C communication failure
     * 
     * @note VL53L0X uses 8-bit register addressing
     */
    uint8_t read_register(uint8_t reg);
    
    /**
     * @brief Read 16-bit value from VL53L0X register (big-endian)
     * 
     * @param[in] reg  Register address to read (reads reg and reg+1)
     * 
     * @return 16-bit register value in host byte order, or 0 on I2C communication failure
     * 
     * @note VL53L0X stores 16-bit values in big-endian (MSB first) format
     * @note Reads two consecutive 8-bit registers and combines into 16-bit value
     */
    uint16_t read_register16(uint8_t reg);

    /**
     * @brief Write 8-bit value to VL53L0X register
     * 
     * @param[in] reg    Register address to write (8-bit I2C register address)
     * @param[in] value  8-bit value to write to register
     * 
     * @note Blocking I2C write operation
     */
    void write_register(uint8_t reg, uint8_t value);
    
    /**
     * @brief Write 16-bit value to VL53L0X register (big-endian)
     * 
     * @param[in] reg    Register address to write (writes to reg and reg+1)
     * @param[in] value  16-bit value to write
     * 
     * @note VL53L0X expects 16-bit values in big-endian (MSB first) format
     * @note Writes two consecutive 8-bit registers from 16-bit value
     */
    void write_register16(uint8_t reg, uint16_t value);

    /**
     * @struct SequenceStepEnables
     * @brief Bit flags indicating which VL53L0X measurement sequence steps are enabled
     * 
     * @details The VL53L0X ranging sequence consists of multiple measurement phases.
     *          This structure tracks which phases are active for timing budget calculations.
     * 
     * @var SequenceStepEnables::tcc
     *      Target CentreCheck - validates target detection
     * @var SequenceStepEnables::msrc
     *      Minimum Signal Rate Check - ensures sufficient return signal
     * @var SequenceStepEnables::dss
     *      Dynamic SPAD Selection - optimizes SPAD array configuration
     * @var SequenceStepEnables::pre_range
     *      Pre-range measurement phase for ambient light compensation
     * @var SequenceStepEnables::final_range
     *      Final distance measurement phase (always required)
     */
    struct SequenceStepEnables {
        bool tcc:1, msrc:1, dss:1, pre_range:1, final_range:1;
    };

    /**
     * @struct SequenceStepTimeouts
     * @brief Timing parameters for VL53L0X measurement sequence phases
     * 
     * @details Contains timing configuration for each measurement step. Used to calculate
     *          total measurement timing budget and configure VCSEL pulse periods.
     *          Timeouts specified in multiple units (PCLKs, MCLKs, microseconds) depending
     *          on hardware timing stage.
     * 
     * @var SequenceStepTimeouts::pre_range_vcsel_period_pclks
     *      Pre-range VCSEL period in PCLKs (PLL clocks), typical 12-18 PCLKs
     * @var SequenceStepTimeouts::final_range_vcsel_period_pclks
     *      Final range VCSEL period in PCLKs (PLL clocks), typical 8-14 PCLKs
     * @var SequenceStepTimeouts::msrc_dss_tcc_mclks
     *      MSRC/DSS/TCC timeout in MCLKs (macro period clocks)
     * @var SequenceStepTimeouts::pre_range_mclks
     *      Pre-range timeout in MCLKs (macro period clocks)
     * @var SequenceStepTimeouts::final_range_mclks
     *      Final range timeout in MCLKs (macro period clocks)
     * @var SequenceStepTimeouts::msrc_dss_tcc_us
     *      MSRC/DSS/TCC timeout in microseconds (μs)
     * @var SequenceStepTimeouts::pre_range_us
     *      Pre-range timeout in microseconds (μs)
     * @var SequenceStepTimeouts::final_range_us
     *      Final range timeout in microseconds (μs)
     * 
     * @note PCLK = PLL clock period; MCLK = macro period (multiple PCLKs)
     * @note Total timing budget = sum of enabled sequence step timeouts in microseconds
     */
    struct SequenceStepTimeouts {
        uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

        uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    /**
     * @struct RegData
     * @brief Register address and value pair for initialization tuning data
     * 
     * @details Used in tuning_data array to store factory-recommended register settings
     *          loaded during sensor initialization.
     * 
     * @var RegData::reg
     *      8-bit register address
     * @var RegData::value
     *      8-bit register value to write
     */
    struct RegData {
        uint8_t reg;
        uint8_t value;
    };

    /**
     * @brief Array of register tuning data loaded during initialization
     * @note Contains ST factory-recommended register settings for optimal performance
     */
    static const RegData tuning_data[];
    
    /**
     * @enum vcselPeriodType
     * @brief Specifies which VCSEL pulse period to configure
     * 
     * @details The VL53L0X uses different VCSEL (laser) pulse periods for pre-range
     *          and final range measurement phases. Pulse period affects ranging accuracy,
     *          maximum range, and ambient light immunity.
     * 
     * @var VcselPeriodPreRange
     *      Configure VCSEL period for pre-range measurement (ambient compensation)
     * @var VcselPeriodFinalRange
     *      Configure VCSEL period for final distance measurement
     * 
     * @note Longer VCSEL periods improve range but reduce accuracy
     * @note Valid periods: 12, 14, 16, 18 PCLKs for pre-range; 8, 10, 12, 14 PCLKs for final range
     */
    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };
    
    /**
     * @brief Retrieve SPAD (Single Photon Avalanche Diode) reference calibration data
     * 
     * @details Reads factory-calibrated SPAD configuration from sensor's non-volatile memory.
     *          SPAD array configuration affects sensor sensitivity and noise characteristics.
     *          This calibration data is unique to each sensor and must be read during initialization.
     * 
     * @param[out] count              Number of SPAD elements enabled (typically 32-64)
     * @param[out] type_is_aperture   True if aperture SPADs, false if non-aperture SPADs
     * 
     * @return true if SPAD info successfully retrieved, false on I2C communication error
     * 
     * @note SPAD = Single Photon Avalanche Diode detector array
     * @note Factory calibration stored in sensor NVM (non-volatile memory)
     */
    bool get_SPAD_info(uint8_t * count, bool *type_is_aperture);
    
    /**
     * @brief Read which measurement sequence steps are currently enabled
     * 
     * @param[out] enables  Pointer to SequenceStepEnables structure to populate
     * 
     * @note Used during timing budget calculations to determine active measurement phases
     */
    void getSequenceStepEnables(SequenceStepEnables * enables);
    
    /**
     * @brief Get current measurement timing budget in microseconds
     * 
     * @details Calculates total time allocated for a complete ranging measurement by
     *          summing all enabled sequence step timeouts. Timing budget determines
     *          trade-off between measurement accuracy and update rate.
     * 
     * @return Current timing budget in microseconds (μs)
     * 
     * @note Typical values: 20000-100000 μs (20-100 ms)
     * @note Longer timing budgets provide higher accuracy but lower update rates
     */
    uint32_t getMeasurementTimingBudget(void);
    
    /**
     * @brief Calculate timeout values for each measurement sequence step
     * 
     * @param[in]  enables   Pointer to enabled sequence steps configuration
     * @param[out] timeouts  Pointer to SequenceStepTimeouts structure to populate
     * 
     * @note Converts register timeout values to microseconds for budget calculations
     */
    void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
    
    /**
     * @brief Read VCSEL pulse period for specified measurement phase
     * 
     * @param[in] type  VcselPeriodPreRange or VcselPeriodFinalRange
     * 
     * @return VCSEL pulse period in PCLKs (PLL clocks)
     * 
     * @note Valid pre-range periods: 12, 14, 16, 18 PCLKs
     * @note Valid final range periods: 8, 10, 12, 14 PCLKs
     */
    uint8_t getVcselPulsePeriod(vcselPeriodType type);
    
    /**
     * @brief Convert timeout from MCLKs to microseconds
     * 
     * @param[in] timeout_period_mclks   Timeout in macro period clocks (MCLKs)
     * @param[in] vcsel_period_pclks     VCSEL period in PLL clocks (PCLKs)
     * 
     * @return Timeout duration in microseconds (μs)
     * 
     * @note MCLK duration depends on VCSEL period: MCLK_period = VCSEL_period × macro_period
     * @note Macro period = (2304 × VCSEL_period_pclks × 1000) / 40000 μs
     */
    uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    
    /**
     * @brief Decode timeout register value to MCLKs
     * 
     * @param[in] reg_val  Encoded timeout value from register (compressed format)
     * 
     * @return Timeout in MCLKs (macro period clocks)
     * 
     * @note VL53L0X uses compressed timeout encoding to fit large values in 16-bit registers
     */
    uint16_t decodeTimeout(uint16_t reg_val);
    
    /**
     * @brief Set measurement timing budget in microseconds
     * 
     * @details Configures sensor timing to achieve specified total measurement duration.
     *          Adjusts final range timeout while maintaining other sequence step timings.
     *          Longer budgets improve accuracy but reduce maximum update rate.
     * 
     * @param[in] budget_us  Desired timing budget in microseconds (μs), minimum ~20000 μs
     * 
     * @return true if budget successfully configured, false if requested budget too short
     * 
     * @note Minimum timing budget ~20000 μs (20 ms) for reliable measurements
     * @note Budget must accommodate all enabled sequence steps plus overhead
     * @warning Timing budgets below 20ms may result in measurement failures or reduced accuracy
     */
    bool setMeasurementTimingBudget(uint32_t budget_us);
    
    /**
     * @brief Convert timeout from microseconds to MCLKs
     * 
     * @param[in] timeout_period_us   Timeout in microseconds (μs)
     * @param[in] vcsel_period_pclks  VCSEL period in PLL clocks (PCLKs)
     * 
     * @return Timeout in MCLKs (macro period clocks)
     * 
     * @note Inverse operation of timeoutMclksToMicroseconds()
     */
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
    
    /**
     * @brief Encode timeout in MCLKs to compressed register format
     * 
     * @param[in] timeout_mclks  Timeout in MCLKs (macro period clocks)
     * 
     * @return Encoded timeout value for writing to register
     * 
     * @note VL53L0X uses compressed encoding: value = (LSB × 2^MSB) format
     */
    uint16_t encodeTimeout(uint16_t timeout_mclks);
    
    /**
     * @brief Perform single reference calibration sequence
     * 
     * @details Executes VHV (VCSEL High Voltage) or phase calibration by triggering
     *          calibration routine and waiting for completion. Calibrations compensate
     *          for temperature and component variations.
     * 
     * @param[in] vhv_init_byte  Calibration type: 0x40 for VHV calibration, 0x00 for phase calibration
     * 
     * @return true if calibration completed successfully, false on timeout or I2C error
     * 
     * @note VHV calibration optimizes VCSEL drive current
     * @note Phase calibration compensates for optical crosstalk
     * @note Both calibrations performed during initialization
     */
    bool performSingleRefCalibration(uint8_t vhv_init_byte);
    
    /**
     * @brief Start continuous ranging mode
     * 
     * @details Configures sensor for continuous distance measurements at rate determined
     *          by timing budget. In continuous mode, sensor automatically triggers new
     *          measurements after completing previous measurement.
     * 
     * @note Sensor remains in continuous mode until power cycle or explicit stop command
     * @note Timer callback reads new measurements as they become available
     */
    void start_continuous(void);
    
    /**
     * @brief Cached stop variable value for continuous ranging mode
     * @note Used to optimize I2C communication during ranging reads
     */
    uint8_t stop_variable;
    
    /**
     * @brief Current measurement timing budget in microseconds (μs)
     * @note Cached value to avoid recalculating from hardware registers
     */
    uint32_t measurement_timing_budget_us;
    
    /**
     * @brief Timestamp of initialization in milliseconds
     * @note Used for timeout detection during calibration sequences
     */
    uint32_t start_ms;

    /**
     * @brief Accumulated distance measurements in millimeters (mm)
     * @note Summed by timer() callback, averaged and reset by update() method
     */
    uint32_t sum_mm;
    
    /**
     * @brief Count of accumulated distance measurements
     * @note Incremented by timer() callback, used for averaging in update() method
     */
    uint32_t counter;
};

#endif  // AP_RANGEFINDER_VL53L0X_ENABLED
