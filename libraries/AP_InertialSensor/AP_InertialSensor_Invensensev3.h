#pragma once

/**
 * @file AP_InertialSensor_Invensensev3.h
 * @brief InvenSense ICM-4xxxx series (generation 3) IMU driver
 * 
 * @details Driver for the invensensev3 range of IMUs
 *          These are the ICM-4 series of IMUs representing InvenSense's
 *          third generation high-performance inertial sensors.
 *          
 *          Supported sensors include ICM-42688, ICM-42605, ICM-40609, IIM-42652,
 *          IIM-42653, ICM-42670, and ICM-45686 with advanced features including
 *          high-resolution modes (up to 20-bit), improved noise performance,
 *          and enhanced on-chip processing capabilities.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_Invensensev3
 * @brief Backend driver for InvenSense ICM-4xxxx generation 3 IMU family
 * 
 * @details State-of-the-art InvenSense IMU driver featuring the latest ICM-4 series sensors:
 *          - ICM-42688, ICM-42605, ICM-40609, IIM-42652, IIM-42653, ICM-42670, ICM-45686
 *          - High-resolution mode: Up to 20-bit resolution (vs 16-bit in earlier generations)
 *          - Ultra-low noise gyro: <0.004 deg/s/√Hz on ICM-42688
 *          - Advanced on-chip filtering and APEX motion processing engine
 *          - Programmable FIFO: Up to 2KB capacity (4x larger than v2)
 *          - Sample rates: Up to 32kHz internal sampling
 *          
 *          High-Resolution Mode:
 *          - 20-bit raw data on ICM-45686 (19-bit on other HiRes variants)
 *          - Improved low-speed precision for precision hovering and stabilization
 *          - Requires special register read sequence from extended FIFO format
 *          - Slightly higher processing overhead (~10-15% CPU increase)
 *          - Configurable via HAL_INS_HIGHRES_SAMPLE compile flag
 *          
 *          APEX Features (Application Execution Engine):
 *          - On-chip pedometer (not used by ArduPilot)
 *          - Tilt detection (not currently utilized)
 *          - Significant motion detection (potential future failsafe use)
 *          - Wake-on-motion for low power mode (not applicable to flight)
 *          
 *          FIFO Improvements over v2:
 *          - 2KB FIFO vs 512B in ICM-20xxx series
 *          - Hardware timestamp for each FIFO packet (improved temporal accuracy)
 *          - Temperature data in FIFO stream (reduces separate reads)
 *          - Configurable packet format (supports both 16-bit and high-res modes)
 *          - Better overflow handling and watermark interrupts
 *          
 *          Register Architecture:
 *          - Bank-switched register map (different from v2 layout)
 *          - ICM-456xy series uses 16-bit bank addresses (extended addressing)
 *          - Separate register sequences for standard vs high-res modes
 *          
 *          Performance Characteristics:
 *          - Gyro full scale: ±2000 dps or ±4000 dps (configurable)
 *          - Accel full scale: ±16g or ±32g (configurable, ArduPilot uses ±16g default)
 *          - Output data rate: Configurable 1-32kHz (ArduPilot typically 1-4kHz)
 *          - Power consumption: ~0.8mA gyro + 0.2mA accel (typical operating)
 * 
 * @note High-resolution mode provides best results for precision applications like
 *       surveying, photogrammetry, and low-speed stabilization
 * @note APEX motion processing features are not currently utilized by ArduPilot
 *       but may be integrated for future advanced motion detection
 * @warning Different register map from v2 (ICM-20xxx) - requires v3-specific initialization
 * @warning High-res mode incompatible with some sensor variants (ICM-40609, ICM-42605, ICM-40605)
 * @warning Fast sampling modes (>1kHz) require sufficient CPU headroom on flight controller
 * 
 * @see AP_InertialSensor_Backend Base class interface
 * @see AP_InertialSensor Main inertial sensor interface
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensensev3.h
 */
class AP_InertialSensor_Invensensev3 : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Virtual destructor for InvenSense v3 IMU backend
     * 
     * @details Cleans up device resources and FIFO buffer allocation
     */
    virtual ~AP_InertialSensor_Invensensev3();

    /**
     * @brief Probe and initialize InvenSense v3 IMU on given device interface
     * 
     * @details Factory method that attempts to detect and initialize an ICM-4xxxx series
     *          IMU on the provided SPI or I2C device. Performs WHO_AM_I register check
     *          to identify specific sensor variant, then initializes registers and FIFO
     *          configuration appropriate for the detected sensor.
     *          
     *          Probe sequence:
     *          1. Read WHO_AM_I register to identify sensor type
     *          2. Validate sensor is supported ICM-4xxxx variant
     *          3. Determine if sensor supports high-resolution mode
     *          4. Initialize backend with appropriate configuration
     *          5. Return backend instance or nullptr if probe fails
     * 
     * @param[in,out] imu Reference to main AP_InertialSensor instance for registration
     * @param[in] dev Ownership pointer to HAL device (SPI or I2C) for communication
     * @param[in] rotation Board rotation to apply to sensor readings (NED frame transformation)
     * 
     * @return Pointer to initialized backend instance on success, nullptr if probe fails
     * 
     * @note This is called during sensor initialization at boot time
     * @note Device ownership is transferred to backend on success, released on failure
     * @warning Probe must not assume device is powered - may need reset sequence
     * 
     * @see check_whoami() for sensor identification logic
     * @see hardware_init() for full initialization sequence
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation);

    /**
     * @brief Update accel and gyro state from accumulated samples
     * 
     * @details Called by main IMU update loop to publish accumulated sensor samples
     *          to the frontend. Applies rotation transformation and publishes integrated
     *          gyro and accel data since last update.
     *          
     *          This method is called after accumulate() has collected samples from FIFO.
     *          Typical call rate: 50Hz (main loop rate)
     * 
     * @return true if update successful and new samples published, false otherwise
     * 
     * @note Called at main loop rate (typically 50-400Hz depending on vehicle)
     * @see accumulate() for sample collection from FIFO
     */
    bool update() override;
    
    /**
     * @brief Accumulate samples from sensor FIFO
     * 
     * @details Reads and processes samples from hardware FIFO buffer. Called by periodic
     *          timer at fast rate (typically 1-4kHz) to prevent FIFO overflow. Extracts
     *          gyro, accel, and temperature data from FIFO packets and accumulates into
     *          backend sample buffer for later publishing via update().
     *          
     *          Processing depends on sensor mode:
     *          - Standard 16-bit mode: Read FIFOData packets
     *          - High-resolution mode: Read FIFODataHighRes packets (19-20 bit)
     *          
     *          Handles FIFO overflow by resetting FIFO and incrementing error counter.
     * 
     * @note Called at backend_rate_hz (typically 1-4kHz) via HAL periodic timer
     * @note Runs in timer interrupt context - must be fast and non-blocking
     * @warning FIFO overflow causes data loss - backend rate must match sensor output rate
     * 
     * @see read_fifo() for FIFO read implementation
     * @see accumulate_samples() for 16-bit sample processing
     * @see accumulate_highres_samples() for high-resolution sample processing
     */
    void accumulate() override;

    /**
     * @brief Start periodic FIFO reading
     * 
     * @details Registers periodic callback with HAL scheduler to read sensor FIFO at
     *          backend_rate_hz. This begins the continuous sample accumulation process.
     *          Called during sensor initialization after successful probe.
     * 
     * @note Called once during initialization, not during normal operation
     * @see accumulate() for the periodic callback function
     */
    void start() override;
    
    /**
     * @brief Get startup banner string for GCS display
     * 
     * @details Generates human-readable sensor identification string showing detected
     *          sensor type and operating mode for display on ground control station console.
     *          
     *          Example output: "ICM-42688 HiRes" or "ICM-40609"
     * 
     * @param[out] banner Buffer to write banner string into
     * @param[in] banner_len Maximum length of banner buffer in bytes
     * 
     * @return true if banner generated successfully, false if buffer too small
     * 
     * @note Called during sensor initialization for logging/display purposes
     */
    bool get_output_banner(char* banner, uint8_t banner_len) override;

    /**
     * @enum Invensensev3_Type
     * @brief Supported InvenSense v3 IMU sensor variants
     * 
     * @details Enumeration of all supported ICM-4xxxx and IIM-4xxxx series sensors
     *          with indication of high-resolution capability. High-resolution mode
     *          provides 19-bit (±0.002% precision) or 20-bit (±0.001% precision)
     *          data output compared to standard 16-bit (±0.015% precision).
     */
    enum class Invensensev3_Type : uint8_t {
        ICM40609 = 0, ///< ICM-40609: Entry-level, no high-res support, ±16g/±2000dps
        ICM42688,     ///< ICM-42688: Premium 19-bit high-res, ultra-low noise gyro <0.004 deg/s/√Hz
        ICM42605,     ///< ICM-42605: Standard, no high-res support, ±16g/±2000dps
        ICM40605,     ///< ICM-40605: Standard, no high-res support, ±16g/±2000dps
        IIM42652,     ///< IIM-42652: Industrial 19-bit high-res, extended temp range -40°C to +105°C
        IIM42653,     ///< IIM-42653: Industrial 19-bit high-res, extended temp range -40°C to +105°C
        ICM42670,     ///< ICM-42670: Compact 19-bit high-res, low power consumption
        ICM45686      ///< ICM-45686: Latest generation 20-bit high-res, best-in-class performance
    };

    /**
     * @brief Accelerometer overflow detection multiplier
     * 
     * @details Multiplier used to detect accelerometer saturation/overflow conditions.
     *          InvenSense v3 sensors can return values up to ±32G even when configured
     *          for ±16G range due to internal headroom. This multiplier is used to
     *          check if measured acceleration exceeds physical sensor limits, indicating
     *          potential clipping or extreme maneuvers.
     *          
     *          Value: INT16_MAX / (32 * 9.80665) = ~1041 LSB/G at 32G full scale
     * 
     * @note Used for error detection and data quality validation
     */
    const uint16_t multiplier_accel = INT16_MAX/(32*GRAVITY_MSS);

protected:
    /**
     * @brief Set this backend as primary IMU for the system
     * 
     * @details Called by frontend when this IMU instance is selected as the primary
     *          inertial sensor for vehicle state estimation. May adjust filtering or
     *          logging configuration based on primary status.
     * 
     * @param[in] _is_primary true if this backend is primary, false otherwise
     * 
     * @note Only one IMU backend is primary at a time
     * @see AP_InertialSensor::set_primary()
     */
    void set_primary(bool _is_primary) override;

private:
    /**
     * @brief Private constructor for InvenSense v3 backend
     * 
     * @details Initializes backend instance with device and rotation configuration.
     *          Only called by probe() factory method after successful sensor detection.
     * 
     * @param[in,out] imu Reference to main AP_InertialSensor for sample registration
     * @param[in] dev Ownership pointer to HAL device for sensor communication
     * @param[in] rotation Board rotation to apply (NED frame transformation)
     * 
     * @note Private constructor enforces factory pattern via probe()
     */
    AP_InertialSensor_Invensensev3(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation);

    /**
     * @brief Initialize sensor hardware registers and configuration
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Software reset via PWR_MGMT register
     *          2. Wait for sensor ready (up to 100ms)
     *          3. Configure gyro and accel full scale ranges
     *          4. Set output data rate and filtering
     *          5. Enable FIFO with appropriate packet format
     *          6. Allocate FIFO read buffer based on mode
     *          
     *          Initialization varies by sensor type - ICM-456xy series uses
     *          extended register addressing while other variants use standard
     *          bank-switched addressing.
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Called once during probe sequence
     * @note Failure typically indicates communication issue or wrong device
     * @warning Must complete within reasonable timeout for boot sequence
     * 
     * @see set_filter_and_scaling() for register configuration details
     */
    bool hardware_init();
    
    /**
     * @brief Check WHO_AM_I register to identify sensor variant
     * 
     * @details Reads WHO_AM_I register and matches against known ICM-4xxxx sensor IDs
     *          to determine exact sensor type. Sets inv3_type member to identified variant.
     *          
     *          Known WHO_AM_I values:
     *          - ICM-40609: 0x3B
     *          - ICM-42688: 0x47
     *          - ICM-42605: 0x42
     *          - ICM-40605: 0x33
     *          - IIM-42652: 0x6F
     *          - IIM-42653: 0x6E
     *          - ICM-42670: 0x67
     *          - ICM-45686: 0xE9
     * 
     * @return true if known sensor detected, false if unrecognized
     * 
     * @note First step in probe sequence
     * @warning Unknown sensor ID causes probe failure
     */
    bool check_whoami();

    /**
     * @brief Configure digital filtering and full-scale ranges for standard ICM-4xxxx sensors
     * 
     * @details Sets gyro/accel full scale ranges, output data rates, and on-chip digital
     *          filter configuration for majority of ICM-4xxxx sensors. Configures FIFO
     *          packet format based on whether high-resolution mode is available and enabled.
     *          
     *          Typical configuration:
     *          - Gyro: ±2000 dps (or ±4000 dps if configured)
     *          - Accel: ±16G
     *          - ODR: 1kHz, 2kHz, or 4kHz based on backend rate
     *          - Anti-aliasing filter: 3dB frequency at ~ODR/4
     * 
     * @note Called during hardware_init() for most sensor variants
     * @see set_filter_and_scaling_icm42670() for ICM-42670 specific config
     * @see set_filter_and_scaling_icm456xy() for ICM-456xy specific config
     */
    void set_filter_and_scaling(void);
    
    /**
     * @brief Configure filtering and scaling specific to ICM-42670 sensor
     * 
     * @details ICM-42670 has different register layout and filter options compared
     *          to other ICM-4xxxx variants. This method configures sensor-specific
     *          filtering while maintaining compatibility with standard backend interface.
     * 
     * @note Called during hardware_init() only for ICM-42670
     */
    void set_filter_and_scaling_icm42670(void);
    
    /**
     * @brief Configure filtering and scaling for ICM-456xy series (extended addressing)
     * 
     * @details ICM-456xy series (e.g., ICM-45686) uses 16-bit register addresses instead
     *          of 8-bit bank-switched addressing. This method handles the extended address
     *          space and configures the advanced filtering options available on these
     *          latest-generation sensors.
     * 
     * @note Called during hardware_init() only for ICM-456xy variants
     * @note 20-bit high-resolution mode only available on ICM-45686
     */
    void set_filter_and_scaling_icm456xy(void);
    
    /**
     * @brief Reset hardware FIFO to empty state
     * 
     * @details Clears FIFO contents and resets read/write pointers. Used during initialization
     *          and when recovering from FIFO overflow conditions. Also resets FIFO packet
     *          count to zero.
     * 
     * @note May take up to 1ms for FIFO to fully reset
     * @warning Discards any unread samples in FIFO
     */
    void fifo_reset();
    
    /**
     * @brief Calculate optimal backend sampling rate for fast sampling mode
     * 
     * @details When fast_sampling is enabled (>1kHz), calculates the backend rate that
     *          provides best balance between sample freshness and CPU overhead. Considers
     *          base rate requirements and maximum achievable rate given CPU constraints.
     *          
     *          Algorithm ensures:
     *          - Backend rate is multiple of base rate (for even integration)
     *          - Does not exceed max_backend_rate (CPU limit)
     *          - Matches sensor ODR configuration
     * 
     * @param[in] base_backend_rate Minimum required backend rate in Hz (typically 1000)
     * @param[in] max_backend_rate Maximum achievable backend rate in Hz (CPU dependent)
     * 
     * @return Calculated optimal backend rate in Hz
     * 
     * @note Used during initialization when fast_sampling enabled
     */
    uint16_t calculate_fast_sampling_backend_rate(uint16_t base_backend_rate, uint16_t max_backend_rate) const;

    /**
     * @brief Read and process samples from hardware FIFO
     * 
     * @details Reads FIFO packet count, retrieves FIFO data via block read, and processes
     *          samples based on mode (16-bit standard or high-resolution). Handles FIFO
     *          overflow by resetting FIFO and logging error. Extracts gyro, accel, and
     *          temperature data from FIFO packets.
     *          
     *          FIFO packet formats:
     *          - Standard: 8 bytes (header + 6-axis data + temp)
     *          - High-res: 16 bytes (header + 6-axis 20-bit data + temp + footer)
     *          
     *          Called from accumulate() at backend rate (typically 1-4kHz).
     * 
     * @note Runs in timer interrupt context - must be efficient
     * @warning FIFO overflow indicates backend rate insufficient for sensor ODR
     * 
     * @see accumulate_samples() for standard mode processing
     * @see accumulate_highres_samples() for high-res mode processing
     */
    void read_fifo();

    /**
     * @brief Read multiple consecutive registers via block read
     * 
     * @details Performs efficient multi-byte read from sequential register addresses.
     *          Used for reading FIFO data and multi-byte sensor values.
     * 
     * @param[in] reg Starting register address
     * @param[out] buf Buffer to receive read data
     * @param[in] size Number of bytes to read
     * 
     * @return true if read successful, false on communication error
     * 
     * @note More efficient than multiple single-byte reads
     */
    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    
    /**
     * @brief Read single register from current bank (Bank 0)
     * 
     * @details Reads one byte from specified register in current register bank.
     *          Most configuration registers are in Bank 0.
     * 
     * @param[in] reg Register address (0x00-0xFF)
     * 
     * @return Register value read
     * 
     * @note For Bank 0 registers only - use register_read_bank() for other banks
     */
    uint8_t register_read(uint8_t reg);
    
    /**
     * @brief Write single register in current bank (Bank 0)
     * 
     * @details Writes one byte to specified register. Optionally performs read-back
     *          verification to ensure write succeeded.
     * 
     * @param[in] reg Register address (0x00-0xFF)
     * @param[in] val Value to write
     * @param[in] checked If true, read back register to verify write (default: false)
     * 
     * @note checked=true adds ~100us delay for SPI round-trip
     * @note For Bank 0 registers only - use register_write_bank() for other banks
     */
    void register_write(uint8_t reg, uint8_t val, bool checked=false);

    /**
     * @brief Read register from specified bank (bank-switched addressing)
     * 
     * @details Switches to specified register bank, reads register, then switches back.
     *          Used for ICM-4xxxx sensors with bank-switched register map (most variants).
     *          
     *          Bank switching sequence:
     *          1. Write bank number to REG_BANK_SEL
     *          2. Read target register
     *          3. Restore Bank 0
     * 
     * @param[in] bank Bank number (0-4)
     * @param[in] reg Register address within bank
     * 
     * @return Register value read
     * 
     * @note Not used for ICM-456xy which has extended addressing
     * @warning Bank switching adds latency - minimize cross-bank access
     */
    uint8_t register_read_bank(uint8_t bank, uint8_t reg);
    
    /**
     * @brief Write register in specified bank (bank-switched addressing)
     * 
     * @details Switches to specified register bank, writes register, then switches back.
     * 
     * @param[in] bank Bank number (0-4)
     * @param[in] reg Register address within bank
     * @param[in] val Value to write
     * 
     * @note Not used for ICM-456xy which has extended addressing
     */
    void register_write_bank(uint8_t bank, uint8_t reg, uint8_t val);
    
    /**
     * @brief Read register using ICM-456xy extended 16-bit addressing
     * 
     * @details ICM-456xy series uses 16-bit register addresses instead of bank switching.
     *          Provides access to larger register space with simpler addressing model.
     * 
     * @param[in] bank_addr High byte of 16-bit register address
     * @param[in] reg Low byte of 16-bit register address
     * 
     * @return Register value read
     * 
     * @note Only used for ICM-456xy variants (e.g., ICM-45686)
     */
    uint8_t register_read_bank_icm456xy(uint16_t bank_addr, uint16_t reg);
    
    /**
     * @brief Write register using ICM-456xy extended 16-bit addressing
     * 
     * @details Writes to ICM-456xy 16-bit address space.
     * 
     * @param[in] bank_addr High byte of 16-bit register address
     * @param[in] reg Low byte of 16-bit register address
     * @param[in] val Value to write
     * 
     * @note Only used for ICM-456xy variants (e.g., ICM-45686)
     */
    void register_write_bank_icm456xy(uint16_t bank_addr, uint16_t reg, uint8_t val);

    /**
     * @brief Accumulate standard 16-bit FIFO samples into backend buffer
     * 
     * @details Processes standard resolution FIFO packets (8 bytes each) containing
     *          16-bit gyro, accel, and temperature data. Applies scaling factors to
     *          convert raw sensor values to physical units (rad/s for gyro, m/s² for accel).
     *          Accumulates samples into backend buffer for later publishing via update().
     *          
     *          FIFO packet format (8 bytes):
     *          - Header (1 byte)
     *          - Accel X,Y,Z (6 bytes, 16-bit each)
     *          - Gyro X,Y,Z (6 bytes, 16-bit each)
     *          - Temperature (1 byte, scaled)
     * 
     * @param[in] data Pointer to array of FIFOData structures read from hardware
     * @param[in] n_samples Number of samples in array to process
     * 
     * @return true if samples accumulated successfully, false on error
     * 
     * @note Called from read_fifo() in standard resolution mode
     * @note Temperature filtered with low-pass filter before use
     */
    bool accumulate_samples(const struct FIFOData *data, uint8_t n_samples);
    
    /**
     * @brief Accumulate high-resolution 19/20-bit FIFO samples into backend buffer
     * 
     * @details Processes high-resolution FIFO packets (16 bytes each) containing
     *          19-bit or 20-bit gyro/accel data for improved precision. Uses extended
     *          scale factors to maintain resolution. Only available on sensors with
     *          high-res capability (ICM-42688, IIM-42652/53, ICM-42670, ICM-45686).
     *          
     *          High-res FIFO packet format (16 bytes):
     *          - Header (1 byte)
     *          - Accel X,Y,Z high bytes (3 bytes)
     *          - Accel X,Y,Z low bytes (6 bytes, includes extra LSBs)
     *          - Gyro X,Y,Z high bytes (3 bytes)
     *          - Gyro X,Y,Z low bytes (6 bytes, includes extra LSBs)
     *          - Temperature (2 bytes)
     *          
     *          Resolution improvement:
     *          - 16-bit: 2^16 = 65,536 counts = ±0.015% precision
     *          - 19-bit: 2^19 = 524,288 counts = ±0.002% precision
     *          - 20-bit: 2^20 = 1,048,576 counts = ±0.001% precision
     * 
     * @param[in] data Pointer to array of FIFODataHighRes structures
     * @param[in] n_samples Number of samples in array to process
     * 
     * @return true if samples accumulated successfully, false on error
     * 
     * @note Only called when HAL_INS_HIGHRES_SAMPLE enabled and sensor supports high-res
     * @note Higher CPU overhead (~15%) due to additional data processing
     * @warning Requires sensor variant with high-res capability
     * 
     * @see Invensensev3_Type enum for sensor high-res capability
     */
    bool accumulate_highres_samples(const struct FIFODataHighRes *data, uint8_t n_samples);

    /**
     * @brief Get the gyro backend rate in Hz at which FIFO is being read
     * 
     * @details Returns the configured backend sampling rate for this IMU. Used by
     *          frontend to coordinate multi-IMU sampling and by filters to determine
     *          sample timing.
     * 
     * @return Backend rate in Hz (typically 1000, 2000, or 4000)
     * 
     * @note Overrides base class to provide sensor-specific rate
     */
    uint16_t get_gyro_backend_rate_hz() const override {
        return backend_rate_hz;
    }

    /**
     * @brief Cached FIFO CONFIG1 register value
     * 
     * @details Stores current FIFO configuration to avoid redundant register reads.
     *          Used during FIFO reset to restore configuration.
     */
    uint8_t fifo_config1;

    /**
     * @brief Temperature sensor sensitivity in LSB per °C
     * 
     * @details Conversion factor for raw temperature value to degrees Celsius.
     *          Varies by sensor variant (typically 132.48 or 207.6 LSB/°C).
     *          Set during hardware initialization based on detected sensor type.
     */
    float temp_sensitivity;
    
    /**
     * @brief Temperature sensor zero point offset
     * 
     * @details Temperature in degrees Celsius corresponding to temperature register
     *          value of zero. Always 25°C for InvenSense v3 sensors.
     */
    const float temp_zero = 25; // degC
    
    /**
     * @brief Board rotation to apply to sensor readings
     * 
     * @details Rotation transformation to convert from sensor frame to vehicle body frame,
     *          accounting for physical mounting orientation of IMU on flight controller.
     *          Applied during update() before publishing samples.
     *          
     *          Common rotations: ROTATION_NONE, ROTATION_YAW_90, ROTATION_YAW_180, etc.
     */
    const enum Rotation rotation;

    /**
     * @brief Scale range for 16-bit standard mode sensor data
     * 
     * @details Maximum positive value for signed 16-bit integer (2^15 = 32768).
     *          Used as divisor to convert 16-bit raw values to normalized range.
     */
    static constexpr float SCALE_RANGE_16BIT = 32768; // 2^15;
    
    /**
     * @brief Scale range for 20-bit high-resolution mode sensor data
     * 
     * @details Maximum positive value for signed 20-bit integer (2^19 = 524,288).
     *          Used for ICM-45686 which provides true 20-bit gyro and 19-bit accel.
     *          Provides 16x better resolution than 16-bit mode.
     */
    static constexpr float SCALE_RANGE_20BIT = 524288; // 2^19;
    
    /**
     * @brief Scale range for 19-bit high-resolution mode sensor data
     * 
     * @details Maximum positive value for signed 19-bit integer (2^18 = 262,144).
     *          Used for most high-res sensors (ICM-42688, IIM-42652/53, ICM-42670).
     *          Provides 8x better resolution than 16-bit mode.
     */
    static constexpr float SCALE_RANGE_19BIT = 262144; // 2^18;

    /**
     * @brief Gyro scale factor for ±2000 dps full scale range (16-bit mode)
     * 
     * @details Conversion from raw 16-bit gyro value to radians/second.
     *          At ±2000 dps: 16.4 LSB/dps, sensitivity = 32768/2000 = 16.4
     *          Result in rad/s = raw_value * GYRO_SCALE_2000DPS
     */
    static constexpr float GYRO_SCALE_2000DPS = radians(1) / (SCALE_RANGE_16BIT / 2000.0);
    
    /**
     * @brief Gyro scale factor for ±4000 dps full scale range (16-bit mode)
     * 
     * @details Conversion from raw 16-bit gyro value to radians/second at extended range.
     *          At ±4000 dps: 8.2 LSB/dps, sensitivity = 32768/4000 = 8.2
     *          Used when higher rotation rates expected (acrobatic flight).
     */
    static constexpr float GYRO_SCALE_4000DPS = radians(1) / (SCALE_RANGE_16BIT / 4000.0);
    
    /**
     * @brief Gyro scale factor for ±2000 dps in high-resolution mode
     * 
     * @details Conversion from raw 19/20-bit gyro value to radians/second.
     *          High-res mode provides 131 LSB/dps base sensitivity, modified by
     *          extended bit depth (SCALE_RANGE_20BIT for ICM-45686).
     *          8-16x better resolution than standard 16-bit mode.
     */
    static constexpr float GYRO_SCALE_HIGHRES_2000DPS = radians(1) / (SCALE_RANGE_20BIT / 2000.0);
    
    /**
     * @brief Gyro scale factor for ±4000 dps in high-resolution mode
     * 
     * @details High-res conversion for extended gyro range.
     */
    static constexpr float GYRO_SCALE_HIGHRES_4000DPS = radians(1) / (SCALE_RANGE_20BIT / 4000.0);
    
    /**
     * @brief Accel scale factor for ±16g full scale range (16-bit mode)
     * 
     * @details Conversion from raw 16-bit accel value to m/s².
     *          At ±16g: 2048 LSB/g, sensitivity = 32768/16 = 2048
     *          Result in m/s² = raw_value * ACCEL_SCALE_16G
     *          
     *          ArduPilot typically uses ±16g for good resolution with adequate headroom
     *          for aggressive maneuvers (15g sustained in tight turns).
     */
    static constexpr float ACCEL_SCALE_16G = (GRAVITY_MSS / (SCALE_RANGE_16BIT / 16));
    
    /**
     * @brief Accel scale factor for ±32g full scale range (16-bit mode)
     * 
     * @details Conversion at extended range with reduced resolution.
     *          At ±32g: 1024 LSB/g, sensitivity = 32768/32 = 1024
     *          May be used for extreme acrobatic flight or crash detection.
     */
    static constexpr float ACCEL_SCALE_32G = (GRAVITY_MSS / (SCALE_RANGE_16BIT / 32));
    
    /**
     * @brief Accel scale factor for ±16g in high-resolution mode
     * 
     * @details Conversion from 19/20-bit accel value to m/s².
     *          ICM-45686: 16384 LSB/g (20-bit scale)
     *          Other high-res: 8192 LSB/g (19-bit scale)
     *          Provides significantly better low-acceleration resolution for
     *          precision applications (surveying, photogrammetry).
     */
    static constexpr float ACCEL_SCALE_HIGHRES_16G = (GRAVITY_MSS / (SCALE_RANGE_20BIT / 16));
    
    /**
     * @brief Accel scale factor for ±32g in high-resolution mode
     * 
     * @details High-res conversion for extended accel range.
     */
    static constexpr float ACCEL_SCALE_HIGHRES_32G = (GRAVITY_MSS / (SCALE_RANGE_20BIT / 32));

    /**
     * @brief Current accelerometer scale factor in m/s² per LSB
     * 
     * @details Active scale factor used to convert raw accel values to physical units.
     *          Set during initialization based on configured full-scale range and mode.
     *          Default: ACCEL_SCALE_16G for ±16g range in 16-bit mode.
     */
    float accel_scale = ACCEL_SCALE_16G;
    
    /**
     * @brief Current gyroscope scale factor in rad/s per LSB
     * 
     * @details Active scale factor used to convert raw gyro values to physical units.
     *          Set during initialization based on configured full-scale range and mode.
     *          Default: GYRO_SCALE_2000DPS for ±2000 dps range in 16-bit mode.
     */
    float gyro_scale = GYRO_SCALE_2000DPS;

    /**
     * @brief Flag indicating fast sampling mode (>1kHz backend rate)
     * 
     * @details true if backend_rate_hz > 1000, false otherwise.
     *          Fast sampling mode uses higher FIFO read rate and may adjust
     *          filtering parameters for better high-frequency response.
     *          Requires sufficient CPU headroom.
     */
    bool fast_sampling;
    
#if HAL_INS_HIGHRES_SAMPLE
    /**
     * @brief Flag indicating high-resolution sampling mode active
     * 
     * @details true if sensor supports and is configured for 19/20-bit high-res mode.
     *          Only available when HAL_INS_HIGHRES_SAMPLE compile flag enabled and
     *          sensor variant supports high-res (see Invensensev3_Type enum).
     *          
     *          High-res mode provides better low-rate precision at cost of slightly
     *          higher CPU usage (~15% increase).
     */
    bool highres_sampling;
#endif

    /**
     * @brief Backend sampling rate in Hz
     * 
     * @details Rate at which FIFO is read and samples accumulated into backend buffer.
     *          Typical values: 1000 Hz (standard), 2000 Hz (fast), 4000 Hz (very fast).
     *          Must match or exceed sensor ODR to prevent FIFO overflow.
     */
    uint16_t backend_rate_hz;
    
    /**
     * @brief Backend period in microseconds (pre-calculated)
     * 
     * @details Period = 1,000,000 / backend_rate_hz
     *          Pre-calculated during init for efficient timing calculations.
     *          Example: 1000 Hz → 1000 μs period
     */
    uint32_t backend_period_us;

    /**
     * @brief Ownership pointer to HAL device for sensor communication
     * 
     * @details Provides SPI or I2C interface to sensor hardware.
     *          Ownership transferred from probe() to backend instance.
     *          Device released on backend destruction.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> dev;
    
    /**
     * @brief Handle for periodic FIFO read callback
     * 
     * @details Registered with HAL scheduler to call accumulate() at backend_rate_hz.
     *          Enables precise timing for FIFO reads to prevent overflow.
     */
    AP_HAL::Device::PeriodicHandle periodic_handle;

    /**
     * @brief Detected sensor type/variant
     * 
     * @details Specific ICM-4xxxx sensor model identified during probe via WHO_AM_I.
     *          Determines available features (high-res support, register layout, etc.).
     *          
     * @see Invensensev3_Type enum for supported variants
     * @see check_whoami() for detection logic
     */
    enum Invensensev3_Type inv3_type;

    /**
     * @brief Dynamically allocated buffer for FIFO reads
     * 
     * @details Buffer size depends on mode:
     *          - Standard 16-bit: Sized for max expected FIFO packet count
     *          - High-res: 2x size due to larger packets (16 bytes vs 8 bytes)
     *          
     *          Allocated during hardware_init(), freed in destructor.
     */
    void* fifo_buffer;

    /**
     * @brief Filtered temperature value in degrees Celsius
     * 
     * @details Low-pass filtered temperature for thermal compensation and monitoring.
     *          Updated each FIFO read, filtered to reduce noise.
     *          Used for gyro drift compensation in some flight modes.
     */
    float temp_filtered;
    
    /**
     * @brief Second-order low-pass filter for temperature smoothing
     * 
     * @details Removes high-frequency noise from temperature readings while maintaining
     *          response to real temperature changes. Cutoff frequency typically set to
     *          ~0.5 Hz to track environmental changes without noise.
     */
    LowPassFilter2pFloat temp_filter;
    
    /**
     * @brief Sensor internal sampling rate in Hz
     * 
     * @details The Output Data Rate (ODR) configured in the sensor registers.
     *          Determines how frequently the sensor updates internal registers and
     *          writes to FIFO. Typically matches backend_rate_hz.
     *          
     *          Common values: 1000, 2000, 4000, 8000 Hz
     */
    uint32_t sampling_rate_hz;
};
