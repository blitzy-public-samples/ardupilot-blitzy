#pragma once
/**
 * @file AP_InertialSensor_Invensensev2.h
 * @brief InvenSense ICM-20xxx series (generation 2) IMU driver
 * 
 * @details Implements backend driver for ICM-20xxx family including ICM-20602, ICM-20608,
 *          ICM-20689, ICM-20948, ICM-20648, ICM-20649 with banked register architecture
 *          and enhanced performance over previous MPU-6xxx generation.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include "AuxiliaryBus.h"

class AP_Invensensev2_AuxiliaryBus;
class AP_Invensensev2_AuxiliaryBusSlave;

/**
 * @class AP_InertialSensor_Invensensev2
 * @brief Backend driver for ICM-20xxx generation 2 IMUs
 * 
 * @details Next-generation InvenSense driver supporting:
 *          - ICM-20602, ICM-20608, ICM-20689, ICM-20948, ICM-20648, ICM-20649
 *          - Banked register architecture (User Bank 0-3)
 *          - Enhanced gyro performance: Lower noise, better stability
 *          - Gyro range: ±2000 deg/s (some variants ±4000 deg/s)
 *          - Accel range: ±16g (some variants ±30g)
 *          - Sample rates: Up to 9kHz internal, decimated to loop rate
 *          
 *          Register bank system:
 *          - Bank 0: Primary sensor data and FIFO
 *          - Bank 1: Self-test and factory trim
 *          - Bank 2: Gyro and accel configuration
 *          - Bank 3: I2C master and auxiliary sensors
 *          - Requires bank selection before register access
 *          
 *          FIFO enhancements:
 *          - Larger FIFO (up to 512 bytes)
 *          - Independent enable for accel/gyro/temp/slave
 *          - Timestamp register for synchronization
 *          
 *          Key improvements over MPU6xxx:
 *          - 3× lower gyro noise
 *          - Better temperature stability
 *          - Faster startup time
 *          
 *          Driver lifecycle:
 *          1. probe() - Detects and identifies sensor via WHO_AM_I
 *          2. _init() - Configures registers and FIFO
 *          3. start() - Begins periodic sampling
 *          4. accumulate() - Reads FIFO and accumulates samples
 *          5. update() - Publishes filtered samples to frontend
 *          
 * @note Bank selection overhead requires optimization for fast access
 * @note Different register map from MPU6xxx - not compatible
 * @warning Must use correct WHO_AM_I value per chip variant
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.h
 */
class AP_InertialSensor_Invensensev2 : public AP_InertialSensor_Backend
{
    friend AP_Invensensev2_AuxiliaryBus;
    friend AP_Invensensev2_AuxiliaryBusSlave;

public:
    /**
     * @brief Destructor - cleanup resources
     * 
     * @details Frees FIFO buffer and auxiliary bus resources
     */
    virtual ~AP_InertialSensor_Invensensev2();

    /**
     * @brief Cast backend reference to Invensensev2-specific type
     * 
     * @param[in] backend Generic backend reference
     * @return Reference to Invensensev2 backend
     * 
     * @note Used for accessing Invensensev2-specific functionality
     */
    static AP_InertialSensor_Invensensev2 &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_Invensensev2&>(backend);
    }

    /**
     * @brief Probe I2C bus for InvenSense V2 IMU
     * 
     * @details Attempts to detect ICM-20xxx sensor on I2C bus by:
     *          1. Reading WHO_AM_I register
     *          2. Validating chip ID against known variants
     *          3. Initializing sensor if detected
     * 
     * @param[in] imu        InertialSensor frontend reference
     * @param[in] dev        I2C device handle (ownership transferred)
     * @param[in] rotation   Board rotation for sensor orientation
     * 
     * @return Pointer to new backend instance if detected, nullptr if not found
     * 
     * @note Takes ownership of dev pointer on success
     * @note Tries multiple I2C addresses if initial probe fails
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation);

    /**
     * @brief Probe SPI bus for InvenSense V2 IMU
     * 
     * @details Attempts to detect ICM-20xxx sensor on SPI bus by:
     *          1. Configuring SPI mode and speed
     *          2. Reading WHO_AM_I register
     *          3. Validating chip ID against known variants
     *          4. Initializing sensor if detected
     * 
     * @param[in] imu        InertialSensor frontend reference
     * @param[in] dev        SPI device handle (ownership transferred)
     * @param[in] rotation   Board rotation for sensor orientation
     * 
     * @return Pointer to new backend instance if detected, nullptr if not found
     * 
     * @note Takes ownership of dev pointer on success
     * @note SPI mode 3 with up to 20MHz clock for ICM-20xxx
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    /**
     * @brief Update frontend with filtered accel and gyro samples
     * 
     * @details Called by frontend at main loop rate to publish accumulated
     *          and filtered sensor data. Applies:
     *          - Rotation correction for board orientation
     *          - Temperature filtering
     *          - Scale factor corrections
     * 
     * @return true if new samples published, false otherwise
     * 
     * @note Called at 400Hz (Copter) or 50Hz (Plane/Rover) typically
     */
    bool update() override;
    
    /**
     * @brief Read FIFO and accumulate raw samples
     * 
     * @details Called at fast loop rate (typically 1kHz+) to:
     *          1. Check data ready status
     *          2. Read FIFO contents
     *          3. Accumulate gyro and accel samples
     *          4. Apply downsampling if configured
     * 
     *          Supports two accumulation modes:
     *          - Fast sampling: Direct FIFO downsample
     *          - Sensor rate: Additional filtering for noise reduction
     * 
     * @note Called from scheduler at gyro backend rate
     * @note Must complete within ~100μs to avoid FIFO overflow
     */
    void accumulate() override;

    /**
     * @brief Return auxiliary I2C bus interface for external sensors
     * 
     * @details Provides access to ICM-20xxx I2C master for connecting:
     *          - External magnetometers (compass)
     *          - External barometers
     *          - Other I2C slave devices (up to 4 slaves)
     * 
     * @return Pointer to AuxiliaryBus interface, nullptr if not available
     * 
     * @note Uses Bank 3 registers for I2C master configuration
     * @note External sensor data can be read via FIFO for synchronization
     */
    AuxiliaryBus *get_auxiliary_bus() override;

    /**
     * @brief Start periodic sampling of IMU
     * 
     * @details Registers periodic callback with HAL scheduler to call
     *          accumulate() at configured backend rate. Enables FIFO
     *          and begins continuous sensor sampling.
     * 
     * @note Called once during sensor initialization
     * @note Sampling rate determined by _gyro_backend_rate_hz
     */
    void start() override;

    /**
     * @brief Get startup banner string for GCS display
     * 
     * @details Formats human-readable sensor identification including:
     *          - Chip variant (ICM-20xxx)
     *          - Bus type (SPI/I2C)
     *          - Device ID
     *          - Sample rates
     * 
     * @param[out] banner      Buffer for banner string
     * @param[in]  banner_len  Maximum banner buffer length
     * 
     * @return true if banner generated, false on error
     * 
     * @note Displayed in GCS during startup for sensor identification
     */
    bool get_output_banner(char* banner, uint8_t banner_len) override;

    /**
     * @brief Get gyro backend sampling rate
     * 
     * @details Returns the rate at which FIFO is being read and samples
     *          are accumulated. Typically 1kHz, 2kHz, or higher depending
     *          on fast_sampling configuration and FIFO downsample settings.
     * 
     * @return Backend rate in Hz (e.g., 1000, 2000, 4000)
     * 
     * @note Used by notch filter configuration
     * @note Different from internal sensor ODR which may be higher
     */
    uint16_t get_gyro_backend_rate_hz() const override {
        return _gyro_backend_rate_hz;
    }

    /**
     * @enum Invensensev2_Type
     * @brief Identifies specific ICM-20xxx chip variant
     * 
     * @details Different variants have unique WHO_AM_I values and may have
     *          different features or performance characteristics. Used to
     *          apply chip-specific configuration and workarounds.
     */
    enum Invensensev2_Type {
        Invensensev2_ICM20948 = 0,  ///< ICM-20948: 9-axis with magnetometer
        Invensensev2_ICM20648,      ///< ICM-20648: 6-axis high performance
        Invensensev2_ICM20649       ///< ICM-20649: 6-axis wide range (±30g)
    };

    /**
     * @brief Accelerometer scale factor for raw to m/s² conversion
     * 
     * @details Accelerometers on InvenSense sensors will return values up to
     *          24G, but they are not guaranteed to be remotely linear past
     *          16G. Scale factor converts 16-bit signed integer to m/s²
     *          assuming ±26G full-scale range for safety margin.
     * 
     * @note Formula: INT16_MAX / (26 * GRAVITY_MSS)
     * @warning Values beyond ±16G may be nonlinear - avoid high-G operation
     */
    const uint16_t multiplier_accel = INT16_MAX/(26*GRAVITY_MSS);

private:
    /**
     * @brief Private constructor - use probe() instead
     * 
     * @param[in] imu       InertialSensor frontend reference
     * @param[in] dev       HAL device handle (I2C or SPI)
     * @param[in] rotation  Board rotation for sensor orientation
     */
    AP_InertialSensor_Invensensev2(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    /**
     * @brief Initialize sensor registers and configuration
     * 
     * @details Performs complete sensor initialization:
     *          1. Hardware reset
     *          2. Register bank configuration
     *          3. Filter and sample rate setup
     *          4. FIFO configuration
     *          5. Auxiliary bus setup (if available)
     * 
     * @return true if initialization successful, false on error
     */
    bool _init();
    
    /**
     * @brief Low-level hardware initialization
     * 
     * @details Configures sensor registers including:
     *          - Power management (wake from sleep)
     *          - Clock source selection
     *          - Gyro and accel full-scale ranges
     *          - Sample rate dividers
     *          - FIFO modes and watermarks
     * 
     * @return true if hardware configured successfully, false on error
     */
    bool _hardware_init();
    
    /**
     * @brief Verify sensor identity via WHO_AM_I register
     * 
     * @details Reads WHO_AM_I register and validates against expected
     *          values for ICM-20xxx family. Sets _inv2_type based on
     *          detected chip variant.
     * 
     * @return true if valid ICM-20xxx detected, false otherwise
     * 
     * @note Must select Bank 0 before reading WHO_AM_I
     */
    bool _check_whoami();

    /**
     * @brief Configure digital filters and scale factors
     * 
     * @details Sets up:
     *          - Digital low-pass filter (DLPF) for gyro and accel
     *          - Sample rate divisors
     *          - Scale factor conversions (raw to physical units)
     *          - FIFO downsample rates
     * 
     * @note Called during _init() after chip identification
     */
    void _set_filter_and_scaling(void);
    
    /**
     * @brief Reset FIFO buffer
     * 
     * @details Performs FIFO reset sequence:
     *          1. Disable FIFO
     *          2. Assert FIFO reset bit
     *          3. Re-enable FIFO with configured modes
     * 
     * @note Called during init and if FIFO overflow detected
     */
    void _fifo_reset();
    
    /**
     * @brief Check if chip supports auxiliary I2C bus
     * 
     * @return true if auxiliary bus available, false otherwise
     * 
     * @note ICM-20948 and some variants have I2C master
     */
    bool _has_auxiliary_bus();

    /**
     * @brief Read samples from FIFO buffer
     * 
     * @details Main data acquisition routine:
     *          1. Read FIFO count registers
     *          2. Validate FIFO count is reasonable
     *          3. Read FIFO data in bulk transfer
     *          4. Parse FIFO packets (gyro, accel, temp)
     *          5. Pass samples to accumulator
     * 
     *          FIFO packet format (typical):
     *          - 2 bytes: Accel X
     *          - 2 bytes: Accel Y
     *          - 2 bytes: Accel Z
     *          - 2 bytes: Temp
     *          - 2 bytes: Gyro X
     *          - 2 bytes: Gyro Y
     *          - 2 bytes: Gyro Z
     * 
     * @note Must select Bank 0 before accessing FIFO
     * @warning FIFO overflow condition resets FIFO and logs error
     */
    void _read_fifo();

    /**
     * @brief Check if new data is available
     * 
     * @details Checks data ready status by:
     *          - Reading DRDY GPIO pin if configured, OR
     *          - Reading INT_STATUS register
     * 
     * @return true if new data available, false otherwise
     * 
     * @note GPIO DRDY is faster than register read
     */
    bool _data_ready();

    /**
     * @brief Poll for new data and read if available (non-blocking)
     * 
     * @details Called by accumulate() to check for and process new samples
     *          without blocking. Reads FIFO if data ready.
     * 
     * @note Alternative to interrupt-driven data acquisition
     */
    void _poll_data();

    /**
     * @brief Read multiple bytes from register (block read)
     * 
     * @details Performs burst read transaction accounting for bus type (SPI/I2C).
     *          Automatically selects correct register bank before read.
     * 
     * @param[in]  reg   16-bit register address (upper 8 bits = bank, lower 8 bits = register)
     * @param[out] buf   Buffer to store read data
     * @param[in]  size  Number of bytes to read
     * 
     * @return true if read successful, false on bus error
     * 
     * @note Bank selection overhead on first access after bank change
     */
    bool _block_read(uint16_t reg, uint8_t *buf, uint32_t size);
    
    /**
     * @brief Read single byte from register
     * 
     * @details Reads one register accounting for banked architecture.
     *          Selects appropriate register bank if needed before read.
     * 
     * @param[in] reg 16-bit register address (upper 8 bits = bank, lower 8 bits = register)
     * 
     * @return Register value (8-bit)
     * 
     * @note Caches current bank to minimize bank selection overhead
     */
    uint8_t _register_read(uint16_t reg);
    
    /**
     * @brief Write single byte to register
     * 
     * @details Writes one register accounting for banked architecture.
     *          Selects appropriate register bank if needed before write.
     * 
     * @param[in] reg     16-bit register address (upper 8 bits = bank, lower 8 bits = register)
     * @param[in] val     Value to write (8-bit)
     * @param[in] checked If true, read back and verify write (default: false)
     * 
     * @note Bank selection adds ~100μs overhead on bank change
     * @warning Critical configuration registers should use checked=true
     */
    void _register_write(uint16_t reg, uint8_t val, bool checked=false);
    
    /**
     * @brief Select active register bank
     * 
     * @details ICM-20xxx uses banked register architecture with 4 banks (0-3).
     *          Writes to REG_BANK_SEL register to switch active bank.
     *          Caches current bank to avoid unnecessary bank switches.
     * 
     * @param[in] bank Bank number (0-3)
     * 
     * @return true if bank selected, false on error
     * 
     * @note Bank 0: Sensor data, FIFO, interrupts
     * @note Bank 1: Self-test, factory trim
     * @note Bank 2: Gyro and accel configuration
     * @note Bank 3: I2C master, auxiliary sensors
     */
    bool _select_bank(uint8_t bank);

    /**
     * @brief Accumulate parsed FIFO samples into backend buffers
     * 
     * @details Processes raw FIFO samples:
     *          1. Applies scale factors (raw → physical units)
     *          2. Accumulates for downsampling if configured
     *          3. Publishes to frontend when accumulation complete
     * 
     * @param[in] samples   Buffer containing parsed FIFO samples
     * @param[in] n_samples Number of samples in buffer
     * 
     * @return true if samples processed successfully, false on error
     * 
     * @note Handles both fast_sampling and sensor_rate_sampling modes
     */
    bool _accumulate(uint8_t *samples, uint8_t n_samples);
    
    /**
     * @brief Accumulate samples with additional sensor-rate filtering
     * 
     * @details Alternative accumulation path for high-performance applications:
     *          - Accumulates at sensor ODR (e.g., 9kHz)
     *          - Applies additional low-pass filtering
     *          - Downsamples to backend rate
     *          - Reduces aliasing and high-frequency noise
     * 
     * @param[in] samples   Buffer containing parsed FIFO samples
     * @param[in] n_samples Number of samples in buffer
     * 
     * @return true if samples processed successfully, false on error
     * 
     * @note Used when fast_sampling=true for enhanced filtering
     * @see _accum structure for filter configuration
     */
    bool _accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples);

    /**
     * @brief Validate raw temperature reading for sensor health
     * 
     * @details Checks temperature sensor value for:
     *          - Reasonable range (typical: -40°C to +85°C)
     *          - Sensor responsiveness (not stuck)
     *          - Valid data (not 0x0000 or 0xFFFF)
     * 
     * @param[in] t2 Raw temperature value from sensor (16-bit signed)
     * 
     * @return true if temperature reading valid, false if suspicious
     * 
     * @note Used for sensor health monitoring
     * @warning Repeated failures indicate hardware problem
     */
    bool _check_raw_temp(int16_t t2);

    /// Raw temperature value from last FIFO read (16-bit signed)
    int16_t _raw_temp;
    
    /// Temperature sensitivity factor: 1/333.87 = 0.002995 °C/LSB
    float temp_sensitivity = 1.0f/333.87f;
    
    /// Temperature offset: 21°C at raw value 0
    float temp_zero = 21;
    
    /// Low-pass filtered temperature in °C
    float _temp_filtered;
    
    /// Accelerometer scale factor for configured range (m/s² per LSB)
    float _accel_scale;
    
    /// FIFO accelerometer scale factor (may differ if FIFO has different resolution)
    float _fifo_accel_scale;
    
    /// FIFO gyroscope scale factor (rad/s per LSB)
    float _fifo_gyro_scale;
    
    /// 2-pole low-pass filter for temperature smoothing
    LowPassFilter2pFloat _temp_filter;

    /// Board rotation for sensor orientation correction
    enum Rotation _rotation;

    /// GPIO pin for data ready interrupt (nullptr if not used)
    AP_HAL::DigitalSource *_drdy_pin;
    
    /// HAL device handle (I2C or SPI)
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    
    /// Auxiliary I2C bus interface (nullptr if not available)
    AP_Invensensev2_AuxiliaryBus *_auxiliary_bus;

    /// Detected sensor type (ICM-20948, ICM-20648, ICM-20649)
    enum Invensensev2_Type _inv2_type;

    /// Fast sampling mode: true = >1kHz backend rate with extra filtering
    bool _fast_sampling;

    /// Gyro FIFO downsample rate (1 = every sample, 2 = every other, etc.)
    uint8_t _gyro_fifo_downsample_rate;

    /// Accelerometer FIFO downsample rate (1 = every sample, 2 = every other, etc.)
    uint8_t _accel_fifo_downsample_rate;

    /// Gyro backend rate in Hz (rate at which samples published to frontend)
    uint16_t _gyro_backend_rate_hz;

    /// Accelerometer backend rate in Hz (typically same as gyro rate)
    uint16_t _accel_backend_rate_hz;

    /// Last read value of USER_CTRL register for state tracking
    uint8_t _last_stat_user_ctrl;

    /// Dynamically allocated buffer for FIFO bulk reads
    uint8_t *_fifo_buffer;

    /// Currently selected register bank (0xFF = unknown/uninitialized)
    uint8_t _current_bank = 0xFF;
    
    /**
     * @brief Accumulators for sensor-rate sampling mode
     * 
     * @details Used when fast_sampling enabled to accumulate samples at
     *          sensor ODR (up to 9kHz) with additional filtering before
     *          downsampling to backend rate. Reduces high-frequency noise.
     * 
     * @see _accumulate_sensor_rate_sampling()
     */
    struct {
        Vector3f accel;              ///< Accumulated accel samples (m/s²)
        Vector3f gyro;               ///< Accumulated gyro samples (rad/s)
        uint8_t accel_count;         ///< Number of accel samples accumulated
        uint8_t gyro_count;          ///< Number of gyro samples accumulated
        LowPassFilterConstDtVector3f accel_filter{4500, 188}; ///< Accel low-pass filter (4.5kHz sample, 188Hz cutoff)
    } _accum;
};

/**
 * @class AP_Invensensev2_AuxiliaryBusSlave
 * @brief Represents external I2C device connected via ICM-20xxx I2C master
 * 
 * @details Provides interface for external sensors (magnetometers, barometers)
 *          connected to ICM-20xxx auxiliary I2C bus. Supports:
 *          - Passthrough mode: Direct I2C access when IMU idle
 *          - Automatic mode: ICM reads sensor and stores in FIFO
 *          - Up to 4 external sensors
 *          - Data synchronized with IMU samples
 * 
 * @note Uses Bank 3 registers for I2C master configuration
 * @see AP_Invensensev2_AuxiliaryBus
 */
class AP_Invensensev2_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_Invensensev2_AuxiliaryBus;

public:
    /**
     * @brief Read from external sensor via passthrough mode
     * 
     * @details Temporarily enables I2C passthrough, performs direct I2C read
     *          from external sensor, then restores normal operation.
     * 
     * @param[in]  reg  External sensor register address
     * @param[out] buf  Buffer to store read data
     * @param[in]  size Number of bytes to read
     * 
     * @return Number of bytes read, or negative on error
     * 
     * @note Passthrough mode bypasses ICM I2C master
     * @warning May conflict with automatic sensor reads - use carefully
     */
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    
    /**
     * @brief Write to external sensor via passthrough mode
     * 
     * @details Temporarily enables I2C passthrough, performs direct I2C write
     *          to external sensor, then restores normal operation.
     * 
     * @param[in] reg External sensor register address
     * @param[in] val Value to write
     * 
     * @return 0 on success, negative on error
     * 
     * @note Used for sensor configuration during initialization
     */
    int passthrough_write(uint8_t reg, uint8_t val) override;

    /**
     * @brief Read cached data from external sensor
     * 
     * @details Reads data that ICM I2C master previously fetched from
     *          external sensor. Data stored in EXT_SENS_DATA registers.
     * 
     * @param[out] buf Buffer to store sensor data
     * 
     * @return Number of bytes read, or negative on error
     * 
     * @note Data synchronized with IMU FIFO samples
     * @note Must configure periodic read first via _configure_periodic_read()
     */
    int read(uint8_t *buf) override;

protected:
    /**
     * @brief Constructor for auxiliary bus slave
     * 
     * @param[in] bus      Parent auxiliary bus reference
     * @param[in] addr     I2C address of external sensor
     * @param[in] instance Slave instance number (0-3)
     */
    AP_Invensensev2_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    
    /**
     * @brief Configure I2C master passthrough mode
     * 
     * @param[in]  reg  External sensor register to read
     * @param[in]  size Number of bytes to read
     * @param[out] out  Buffer for read data (nullptr if not needed)
     * 
     * @return 0 on success, negative on error
     */
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint16_t _inv2_addr;  ///< I2C slave address register (Bank 3)
    const uint16_t _inv2_reg;   ///< I2C slave register address (Bank 3)
    const uint16_t _inv2_ctrl;  ///< I2C slave control register (Bank 3)
    const uint16_t _inv2_do;    ///< I2C slave data out register (Bank 3)

    uint8_t _ext_sens_data = 0; ///< Offset in EXT_SENS_DATA registers
};

/**
 * @class AP_Invensensev2_AuxiliaryBus
 * @brief Manages ICM-20xxx auxiliary I2C master for external sensors
 * 
 * @details Provides high-level interface for attaching external I2C sensors
 *          to ICM-20xxx I2C master. Features:
 *          - Up to 4 external I2C slaves
 *          - Automatic periodic reads synchronized with IMU
 *          - Data available via FIFO for timestamp synchronization
 *          - Up to 24 bytes total external sensor data
 *          
 *          Typical usage:
 *          1. get_auxiliary_bus() from main driver
 *          2. Request slave via request_next_slave()
 *          3. Configure periodic reads
 *          4. Read synchronized data via slave read()
 * 
 * @note Configured via Bank 3 registers
 * @note External sensor data read before FIFO write for synchronization
 * @see AP_Invensensev2_AuxiliaryBusSlave
 */
class AP_Invensensev2_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_Invensensev2;

public:
    /**
     * @brief Get semaphore for thread-safe access
     * 
     * @details Returns IMU device semaphore for coordinating access to
     *          I2C master between main driver and auxiliary bus users.
     * 
     * @return Pointer to HAL semaphore
     * 
     * @note Must hold semaphore during register access
     */
    AP_HAL::Semaphore *get_semaphore() override;
    
    /**
     * @brief Register periodic callback for auxiliary bus operations
     * 
     * @details Allows external sensor drivers to register callbacks that
     *          execute at specified intervals for sensor management.
     * 
     * @param[in] period_usec Callback period in microseconds
     * @param[in] cb          Callback function
     * 
     * @return Periodic callback handle (nullptr on failure)
     * 
     * @note Callback runs at IMU device scheduler priority
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

protected:
    /**
     * @brief Constructor for auxiliary bus
     * 
     * @param[in] backend Parent Invensensev2 backend reference
     * @param[in] devid   Device ID for auxiliary bus
     */
    AP_Invensensev2_AuxiliaryBus(AP_InertialSensor_Invensensev2 &backend, uint32_t devid);

    /**
     * @brief Create new slave instance for external sensor
     * 
     * @param[in] addr     I2C address of external sensor
     * @param[in] instance Slave number (0-3)
     * 
     * @return Pointer to new slave instance
     * 
     * @note Called by request_next_slave() in base class
     */
    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    
    /**
     * @brief Configure automatic periodic read from external sensor
     * 
     * @details Programs ICM I2C master to automatically read from external
     *          sensor at IMU sample rate. Read data stored in EXT_SENS_DATA
     *          registers and optionally written to FIFO.
     * 
     * @param[in] slave Slave device to read from
     * @param[in] reg   Starting register address on external sensor
     * @param[in] size  Number of bytes to read (max 24 total across all slaves)
     * 
     * @return 0 on success, negative on error
     * 
     * @note External sensor data read occurs before IMU data FIFO write
     * @warning Total external sensor data cannot exceed MAX_EXT_SENS_DATA (24 bytes)
     */
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    /**
     * @brief Apply slave configuration to ICM I2C master registers
     * 
     * @details Writes Bank 3 I2C master configuration registers for all
     *          configured slaves. Called after slave setup complete.
     */
    void _configure_slaves();

    /// Maximum external sensor data size in bytes (EXT_SENS_DATA_00 to _23)
    static const uint8_t MAX_EXT_SENS_DATA = 24;
    
    /// Current external sensor data usage in bytes (tracks allocation)
    uint8_t _ext_sens_data = 0;
};

