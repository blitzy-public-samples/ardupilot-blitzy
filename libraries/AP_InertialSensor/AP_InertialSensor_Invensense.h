#pragma once
/**
 * @file AP_InertialSensor_Invensense.h
 * @brief InvenSense MPU6000/MPU6050/MPU9250/ICM-206xx family IMU driver
 * 
 * @details Implements unified backend driver for InvenSense MPU6xxx/9xxx and ICM-206xx series IMUs,
 *          the most widely deployed sensors in ArduPilot flight controllers.
 *          
 *          Supported devices:
 *          - MPU6000, MPU6050: 6-axis IMU (gyro + accel)
 *          - MPU6500: 6-axis IMU with improved performance
 *          - MPU9250, MPU9255: 9-axis IMU (gyro + accel + magnetometer)
 *          - ICM-20601, ICM-20602, ICM-20608, ICM-20689, ICM-20789: Next-gen 6-axis IMUs
 *          
 *          Key features:
 *          - FIFO streaming mode for burst reads (up to 512 bytes)
 *          - Auxiliary I2C master for external sensors (magnetometer, barometer)
 *          - Digital low-pass filter (DLPF) with configurable bandwidth
 *          - Sample rates: 1kHz default, up to 8kHz for some variants
 *          - Flexible downsampling for optimized CPU usage
 *          
 *          FIFO operation:
 *          - Circular buffer accumulates sensor samples at hardware rate
 *          - Driver reads via burst transfer when sufficient samples available
 *          - Packet format: [Accel_X/Y/Z][Temp][Gyro_X/Y/Z] (12 bytes per sample)
 *          - DMA-safe aligned buffer for efficient transfer
 *          - Overflow detection and recovery
 *          
 *          Auxiliary bus support:
 *          - On-chip I2C master can poll up to 4 external sensors
 *          - Typically used for AK8963 magnetometer in MPU9250
 *          - Slave data appended to FIFO packets
 *          - Supports passthrough mode for sensor configuration
 *          
 *          Performance characteristics:
 *          - Gyro noise: ~0.005 deg/s/√Hz (MPU6000)
 *          - Accel noise: ~300 μg/√Hz (MPU6000)
 *          - Update rates: 1kHz, 2kHz, 4kHz, 8kHz depending on variant
 *          - SPI: Up to 20MHz (MPU6000/9250), 10MHz (ICM-206xx)
 *          - I2C: Up to 400kHz (not recommended for flight critical)
 *          
 *          Errata and workarounds:
 *          - Temperature sensitivity compensation applied
 *          - FIFO overflow recovery with reset
 *          - First sample discard after configuration change
 *          - ICM-20602 specific Y-axis offset handling
 *          - Register change monitoring for unexpected resets
 * 
 * @note Most common IMU family in Pixhawk and ChibiOS-based flight controllers
 * @note MPU6000 SPI preferred over MPU6050 I2C for lower latency and higher reliability
 * @note FIFO reset required if overflow detected (typically due to CPU starvation)
 * @warning First 50 samples after startup should be discarded for gyro settling
 * @warning Accel values above 16G are non-linear (hardware limit)
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.h
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

// Forward declarations
class AP_Invensense_AuxiliaryBus;
class AP_Invensense_AuxiliaryBusSlave;

/**
 * @class AP_InertialSensor_Invensense
 * @brief Backend driver for InvenSense MPU6000/MPU9250/ICM-206xx family
 * 
 * @details Comprehensive driver for InvenSense MPU and ICM series IMUs supporting:
 *          
 *          Hardware variants:
 *          - MPU6000, MPU6050: First-generation 6-axis (gyro + accel)
 *          - MPU6500: Improved 6-axis with better noise performance
 *          - MPU9250, MPU9255: 9-axis with integrated AK8963 magnetometer
 *          - ICM-20601, ICM-20602: Low-power 6-axis variants
 *          - ICM-20608, ICM-20689: High-performance 6-axis
 *          - ICM-20789: 6-axis with integrated barometer
 *          
 *          Driver architecture:
 *          - Inherits from AP_InertialSensor_Backend for integration with sensor subsystem
 *          - Uses HAL Device abstraction for SPI/I2C portability
 *          - Periodic callback model: scheduler calls _poll_data() at configured rate
 *          - FIFO-based batch reading minimizes interrupt overhead
 *          - Separate downsample rates for gyro and accel optimize performance
 *          
 *          Initialization sequence:
 *          1. probe() - Detects device via WHOAMI register, creates backend instance
 *          2. _init() - Performs software initialization, configures FIFO and filters
 *          3. _hardware_init() - Configures sensor ranges, sample rates, enables FIFO
 *          4. start() - Registers periodic callback for data polling
 *          
 *          Data flow:
 *          1. Hardware accumulates samples in FIFO at sensor rate (1-8kHz)
 *          2. _poll_data() called by scheduler at backend rate (typically 1kHz)
 *          3. _read_fifo() reads all available samples via burst transfer
 *          4. _accumulate() processes samples, applies rotation, temperature compensation
 *          5. Downsampled data pushed to frontend via _publish_* methods
 *          
 *          FIFO management:
 *          - FIFO size: 512 bytes (MPU6000/9250), 512-1024 bytes (ICM-206xx)
 *          - Sample size: 12 bytes (accel + temp + gyro)
 *          - Overflow detection: Monitor FIFO_OFLOW bit in INT_STATUS register
 *          - Recovery: _fifo_reset() or _fast_fifo_reset() on overflow
 *          - Buffer: Dynamically allocated _fifo_buffer for DMA-safe reads
 *          
 *          Auxiliary I2C bus:
 *          - On-chip I2C master polls external sensors (e.g., magnetometer)
 *          - Slave data written to EXT_SENS_DATA registers
 *          - Optionally appended to FIFO stream
 *          - Passthrough mode allows direct configuration of slaves
 *          - Managed via AP_Invensense_AuxiliaryBus helper class
 *          
 *          Temperature compensation:
 *          - Temperature read with every sample (16-bit signed)
 *          - Conversion: temp_degC = temp_zero + raw * temp_sensitivity
 *          - Default: temp_zero = 36.53°C, temp_sensitivity = 1/340 °C/LSB
 *          - Filtered via 2nd-order low-pass filter for noise reduction
 *          - Used for gyro temperature drift compensation in AP_InertialSensor
 *          
 *          Coordinate system:
 *          - Raw samples in sensor body frame (right-hand rule)
 *          - Rotation parameter transforms to vehicle body frame
 *          - Rotation applied in _accumulate() before publishing
 *          - Gyro: rad/s, Accel: m/s²
 *          
 *          Scaling and units:
 *          - Gyro: ±2000 deg/s full scale (MPU6000/9250), configurable on ICM
 *          - Accel: ±16G full scale (linear range), up to ±26G (non-linear)
 *          - Gyro scale: calculated from FS_SEL register setting
 *          - Accel scale: INT16_MAX / (26 * GRAVITY_MSS)
 *          - Temperature: See temp_sensitivity and temp_zero
 *          
 *          Error handling:
 *          - WHOAMI mismatch: probe() returns nullptr (device not detected)
 *          - FIFO overflow: _fifo_reset() with error logging
 *          - Unexpected register changes: Monitored via _check_register_change()
 *          - I2C/SPI errors: Logged via AP::logger() if enabled
 *          
 *          Performance optimization:
 *          - Fast sampling mode: >1kHz backend rates for racing/acro
 *          - Separate gyro/accel downsample ratios: e.g., 8kHz gyro, 1kHz accel
 *          - Fast FIFO reset: Skips full reset for faster recovery
 *          - Sensor-rate sampling: Optional accumulation at hardware rate
 *          - __RAMFUNC__: Critical paths in RAM for deterministic timing
 *          
 *          Hardware-specific quirks:
 *          - ICM-20602: Y-axis offset register caching required (_saved_y_ofs_high)
 *          - ICM-20789: Integrated barometer on auxiliary bus
 *          - MPU9250: AK8963 magnetometer requires auxiliary bus configuration
 *          
 *          Thread safety:
 *          - All public methods called from frontend (main thread)
 *          - _poll_data() called from scheduler interrupt/thread
 *          - Device bus lock held during register access
 *          - No explicit locking required (HAL Device provides synchronization)
 * 
 * @note Designed for real-time flight control with deterministic timing
 * @note SPI preferred over I2C for flight-critical applications
 * @note FIFO depth limits maximum latency between poll calls
 * @warning Do not call update() or accumulate() faster than backend rate
 * @warning FIFO overflow indicates scheduler overload - reduce backend rate
 * @warning Temperature reading shares ADC with gyro - simultaneous not possible
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.h
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp
 */
class AP_InertialSensor_Invensense : public AP_InertialSensor_Backend
{
    friend AP_Invensense_AuxiliaryBus;
    friend AP_Invensense_AuxiliaryBusSlave;

public:
    /**
     * @brief Virtual destructor
     * 
     * @details Cleans up resources including FIFO buffer and auxiliary bus.
     *          Called when backend is removed from sensor subsystem.
     */
    virtual ~AP_InertialSensor_Invensense();
    
    /**
     * @brief Cast backend reference to Invensense-specific type
     * 
     * @details Type-safe downcast from generic backend to Invensense driver.
     *          Used by auxiliary bus and test code to access driver-specific methods.
     * 
     * @param[in] backend Generic backend reference from AP_InertialSensor
     * 
     * @return Reference to Invensense backend
     * 
     * @warning Caller must ensure backend is actually an Invensense instance
     * @warning Undefined behavior if cast is incorrect
     */
    static AP_InertialSensor_Invensense &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_Invensense&>(backend);
    }

    /**
     * @brief Probe for InvenSense IMU on I2C bus
     * 
     * @details Detection sequence:
     *          1. Read WHOAMI register (0x75) to identify device type
     *          2. Verify WHOAMI matches known InvenSense values:
     *             - 0x68: MPU6000/6050
     *             - 0x70: MPU6500
     *             - 0x71: MPU9250
     *             - 0x12: ICM-20602
     *             - 0xAF: ICM-20608
     *             - etc.
     *          3. Create backend instance if detection successful
     *          4. Perform basic initialization via _init()
     *          
     *          Returns nullptr if:
     *          - WHOAMI read fails (device not responding)
     *          - WHOAMI value unrecognized (wrong device)
     *          - Initialization fails (hardware fault)
     * 
     * @param[in] imu           Reference to main AP_InertialSensor for backend registration
     * @param[in] dev           I2C device handle (ownership transferred to backend)
     * @param[in] rotation      Board rotation to transform sensor frame to vehicle frame
     * 
     * @return Pointer to new backend instance, or nullptr if detection failed
     * 
     * @note I2C mode not recommended for flight-critical applications (use SPI)
     * @note I2C address typically 0x68 or 0x69 (AD0 pin state)
     * @warning Detection requires functioning I2C bus and powered sensor
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation);

    /**
     * @brief Probe for InvenSense IMU on SPI bus
     * 
     * @details Detection sequence identical to I2C variant, but uses SPI transfers.
     *          SPI mode is preferred for flight controllers due to:
     *          - Lower latency (no clock stretching)
     *          - Higher reliability (no bus collisions)
     *          - Faster transfers (up to 20MHz vs 400kHz)
     *          - Deterministic timing
     *          
     *          SPI configuration:
     *          - Mode: SPI_MODE0 or SPI_MODE3 (device supports both)
     *          - Max speed: 20MHz (MPU6000/9250), 10MHz (ICM-206xx)
     *          - Bit order: MSB first
     *          - Read: Set bit 7 high in register address
     *          - Write: Set bit 7 low in register address
     * 
     * @param[in] imu           Reference to main AP_InertialSensor for backend registration
     * @param[in] dev           SPI device handle (ownership transferred to backend)
     * @param[in] rotation      Board rotation to transform sensor frame to vehicle frame
     * 
     * @return Pointer to new backend instance, or nullptr if detection failed
     * 
     * @note SPI strongly recommended over I2C for flight control
     * @note Typical SPI CS pin defined in board hwdef file
     * @warning Ensure correct SPI bus speed to avoid read errors
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation);

    /**
     * @brief Update frontend with latest sensor data (called from main thread)
     * 
     * @details Frontend update interface implementation.
     *          In FIFO-based backends like Invensense, actual data reading happens
     *          in accumulate() called from scheduler. This method typically does
     *          minimal work or nothing, as data is already pushed to frontend.
     *          
     *          Call frequency: Typically 50-400Hz from main loop
     * 
     * @return true if new data available, false otherwise
     * 
     * @note Marked __RAMFUNC__ for deterministic timing (runs from RAM)
     * @note Most work done in accumulate() for Invensense backend
     */
    bool update() override __RAMFUNC__;
    
    /**
     * @brief Accumulate sensor samples from FIFO (called from scheduler)
     * 
     * @details Main data acquisition method for Invensense backend.
     *          Called by scheduler at backend rate (typically 1kHz).
     *          
     *          Operations performed:
     *          1. Check data ready via _data_ready() (DRDY pin or register)
     *          2. Read FIFO via _read_fifo() if samples available
     *          3. Process all samples via _accumulate()
     *          4. Push downsampled data to frontend
     *          
     *          FIFO processing:
     *          - Reads all accumulated samples in single burst transfer
     *          - Processes each 12-byte sample (accel+temp+gyro)
     *          - Applies rotation transformation
     *          - Performs temperature filtering
     *          - Downsamples to backend rate
     *          - Publishes to frontend via _publish_gyro() and _publish_accel()
     *          
     *          Call frequency: Backend rate (1-8kHz depending on configuration)
     * 
     * @note Called from scheduler interrupt or thread context
     * @note Must complete within scheduler time budget to avoid FIFO overflow
     * @warning Do not call from main thread - use update() instead
     * @warning Long execution time can cause FIFO overflow and sample loss
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:accumulate()
     */
    void accumulate() override;

    /**
     * @brief Get auxiliary I2C bus for external sensor integration
     * 
     * @details Returns auxiliary bus interface if supported by this sensor.
     *          
     *          Auxiliary bus capabilities:
     *          - On-chip I2C master polls up to 4 external I2C slaves
     *          - Typically used for magnetometer (AK8963 in MPU9250)
     *          - Can also read barometer, compass, or other I2C devices
     *          - Slave data automatically read and appended to FIFO
     *          - Passthrough mode allows configuration of slaves
     *          
     *          Supported devices:
     *          - MPU9250/9255: Full auxiliary bus support for AK8963
     *          - MPU6000/6500: Auxiliary bus available but no integrated sensors
     *          - ICM-20789: Auxiliary bus for integrated barometer
     *          - ICM-20608: No auxiliary bus
     *          
     *          Usage pattern:
     *          1. Call get_auxiliary_bus() during backend initialization
     *          2. Use returned bus to configure slave devices
     *          3. Register periodic reads for sensor data
     *          4. Slave data automatically appears in EXT_SENS_DATA registers
     * 
     * @return Pointer to AP_Invensense_AuxiliaryBus if supported, nullptr otherwise
     * 
     * @note Only available if sensor hardware supports auxiliary I2C master
     * @note Auxiliary bus shares I2C pins with main IMU (SPI-based IMUs only)
     * @see AP_Invensense_AuxiliaryBus for usage details
     */
    AuxiliaryBus *get_auxiliary_bus() override;

    /**
     * @brief Start backend operation (register periodic callback)
     * 
     * @details Called by AP_InertialSensor after all backends initialized.
     *          Registers periodic callback with scheduler for data polling.
     *          
     *          Startup sequence:
     *          1. Calculate backend rates for gyro and accel
     *          2. Determine FIFO downsample ratios
     *          3. Configure sensor sample rates
     *          4. Enable FIFO
     *          5. Register _poll_data() callback at backend rate
     *          
     *          Callback period calculation:
     *          - gyro_backend_rate = sensor_rate / gyro_fifo_downsample_rate
     *          - accel_backend_rate = sensor_rate / accel_fifo_downsample_rate
     *          - Typical: 8kHz sensor → 1kHz backend (8x downsample)
     *          
     *          After start():
     *          - Scheduler calls accumulate() at backend rate
     *          - FIFO accumulates samples between calls
     *          - Driver reads and processes all samples each callback
     * 
     * @note Called once during initialization, before main loop starts
     * @note Backend not functional until start() called
     * @warning Do not call multiple times
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:start()
     */
    void start() override;

    /**
     * @brief Generate startup banner for GCS display
     * 
     * @details Creates human-readable string identifying detected IMU hardware.
     *          Displayed in GCS console during boot for diagnostics.
     *          
     *          Banner format examples:
     *          - "InvenSense MPU6000"
     *          - "InvenSense MPU9250"
     *          - "InvenSense ICM-20608"
     *          
     *          Banner includes:
     *          - Manufacturer name: "InvenSense"
     *          - Device model: "MPU6000", "ICM-20602", etc.
     *          - Sample rate information (if verbose)
     * 
     * @param[out] banner       Buffer to write banner string
     * @param[in]  banner_len   Size of banner buffer in bytes
     * 
     * @return true if banner generated successfully, false if buffer too small
     * 
     * @note Called during initialization for GCS logging
     * @note Banner appears in STATUSTEXT messages
     */
    bool get_output_banner(char* banner, uint8_t banner_len) override;

    /**
     * @brief Get gyroscope backend sample rate
     * 
     * @details Returns the rate at which gyro samples are published to frontend.
     *          This is the downsampled rate, not the raw sensor rate.
     *          
     *          Rate calculation:
     *          - Raw sensor rate: 1kHz, 2kHz, 4kHz, or 8kHz (hardware dependent)
     *          - Backend rate: raw_rate / downsample_ratio
     *          - Common configurations:
     *            * 8kHz raw → 1kHz backend (8x downsample)
     *            * 4kHz raw → 1kHz backend (4x downsample)
     *            * 1kHz raw → 1kHz backend (1x, no downsample)
     *          
     *          Used by:
     *          - Frontend to calculate delta-time for integration
     *          - Notch filters to match sample rate
     *          - Logging system to record sample rate
     * 
     * @return Gyro backend rate in Hz (typically 1000-8000)
     * 
     * @note Backend rate ≤ raw sensor rate
     * @note May differ from accel backend rate
     * @see get_gyro_backend_rate_hz() in AP_InertialSensor_Backend
     */
    uint16_t get_gyro_backend_rate_hz() const override {
        return _gyro_backend_rate_hz;
    }

    /**
     * @enum Invensense_Type
     * @brief Enumeration of supported InvenSense IMU variants
     * 
     * @details Identifies specific sensor hardware detected by WHOAMI register.
     *          Each type has different characteristics:
     *          
     *          - Invensense_MPU6000: Original 6-axis SPI, most common in Pixhawk
     *          - Invensense_MPU6500: Improved noise performance, lower power
     *          - Invensense_MPU9250: 9-axis with AK8963 magnetometer
     *          - Invensense_ICM20601: Low-cost 6-axis
     *          - Invensense_ICM20602: Mid-range 6-axis with Y-offset quirk
     *          - Invensense_ICM20608: High-performance 6-axis
     *          - Invensense_ICM20689: High-performance with improved temp stability
     *          - Invensense_ICM20789: 6-axis with integrated barometer
     *          
     *          Detection via WHOAMI register (0x75):
     *          - MPU6000: 0x68
     *          - MPU6500: 0x70
     *          - MPU9250: 0x71
     *          - ICM-20602: 0x12
     *          - ICM-20608: 0xAF
     *          - ICM-20689: 0x98
     *          - etc.
     * 
     * @note Type determines hardware-specific initialization and workarounds
     * @note Stored in _mpu_type member variable
     */
    enum Invensense_Type {
        Invensense_MPU6000=0,  ///< MPU6000: Original 6-axis, WHOAMI=0x68
        Invensense_MPU6500,    ///< MPU6500: Improved 6-axis, WHOAMI=0x70
        Invensense_MPU9250,    ///< MPU9250: 9-axis with magnetometer, WHOAMI=0x71
        Invensense_ICM20608,   ///< ICM-20608: High-performance 6-axis, WHOAMI=0xAF
        Invensense_ICM20602,   ///< ICM-20602: Mid-range 6-axis, WHOAMI=0x12
        Invensense_ICM20601,   ///< ICM-20601: Low-cost 6-axis
        Invensense_ICM20789,   ///< ICM-20789: 6-axis with barometer
        Invensense_ICM20689,   ///< ICM-20689: High-performance with temp stability, WHOAMI=0x98
    };

    /**
     * @brief Accelerometer scale multiplier for converting raw to m/s²
     * 
     * @details InvenSense accelerometers are configured for ±16G full scale range,
     *          but hardware can measure up to approximately ±26G before saturation.
     *          Values above ±16G are non-linear and not suitable for flight control.
     *          
     *          Scale calculation:
     *          - Raw range: INT16_MAX = 32767
     *          - Physical range: ±26G (approximate hardware limit)
     *          - Scale: 32767 / (26 * 9.80665) = ~128.4 LSB per G
     *          - Conversion: accel_mss = raw_value / multiplier_accel
     *          
     *          Linear range:
     *          - Guaranteed linear: ±16G (datasheet specification)
     *          - Usable for flight: ±16G maximum
     *          - Saturation: ~±26G (hardware limit, non-linear)
     * 
     * @note Value is const, shared across all instances
     * @note GRAVITY_MSS = 9.80665 m/s² (standard gravity)
     * @warning Accel values above ±16G are non-linear and unreliable
     * @warning Do not use for flight control beyond ±16G
     * 
     * Source: InvenSense datasheets (MPU6000/9250/ICM-206xx)
     */
    const uint16_t multiplier_accel = INT16_MAX/(26*GRAVITY_MSS);

private:
    /**
     * @brief Private constructor (use probe() for instantiation)
     * 
     * @details Creates backend instance for detected InvenSense IMU.
     *          Called by probe() after successful WHOAMI detection.
     *          Stores device handle and rotation for later initialization.
     * 
     * @param[in] imu        Reference to AP_InertialSensor for backend registration
     * @param[in] dev        Device handle (SPI or I2C), ownership transferred
     * @param[in] rotation   Board rotation for sensor-to-vehicle frame transform
     * 
     * @note Constructor does not access hardware (done in _init())
     * @note Private to enforce creation via probe() factory method
     */
    AP_InertialSensor_Invensense(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    /**
     * @brief Initialize sensor (software configuration)
     * 
     * @details Performs software initialization sequence:
     *          1. Verify WHOAMI register matches expected value
     *          2. Perform hardware initialization via _hardware_init()
     *          3. Allocate FIFO buffer
     *          4. Configure auxiliary bus if available
     *          5. Register gyro and accel instances with frontend
     *          
     *          Called by probe() after constructor.
     * 
     * @return true if initialization successful, false on failure
     * 
     * @note Must be called before start()
     * @note Failure typically means hardware fault or wrong device
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_init()
     */
    bool _init();
    
    /**
     * @brief Initialize sensor hardware (register configuration)
     * 
     * @details Configures IMU registers for operation:
     *          1. Software reset (PWR_MGMT_1)
     *          2. Wake from sleep mode
     *          3. Set clock source (PLL with gyro reference)
     *          4. Configure gyro full scale (±2000 deg/s)
     *          5. Configure accel full scale (±16G)
     *          6. Set sample rate divider
     *          7. Configure digital low-pass filter (DLPF)
     *          8. Enable FIFO for accel, gyro, temp
     *          9. Set FIFO mode (stream, not snapshot)
     *          
     *          Device-specific configuration:
     *          - MPU6000/9250: Standard configuration
     *          - ICM-20602: Y-offset register handling
     *          - ICM-20789: Barometer initialization
     * 
     * @return true if hardware configured successfully, false on error
     * 
     * @note Accesses hardware registers via _register_write()
     * @note Includes delays for sensor settling
     * @warning Register write failures indicate hardware fault
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_hardware_init()
     */
    bool _hardware_init();
    
    /**
     * @brief Check WHOAMI register to identify device
     * 
     * @details Reads WHOAMI register (0x75) and compares against known values:
     *          - 0x68: MPU6000 or MPU6050
     *          - 0x70: MPU6500
     *          - 0x71: MPU9250 or MPU9255
     *          - 0x12: ICM-20602
     *          - 0xAF: ICM-20608
     *          - 0x98: ICM-20689
     *          - etc.
     *          
     *          Sets _mpu_type based on detected value.
     *          Returns false if WHOAMI unrecognized or read fails.
     * 
     * @return true if device identified, false if unknown or read error
     * 
     * @note First hardware access during probe sequence
     * @note WHOAMI value burned into device at manufacture (read-only)
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_check_whoami()
     */
    bool _check_whoami();

    /**
     * @brief Configure digital low-pass filter (DLPF)
     * 
     * @details Writes CONFIG register to set DLPF bandwidth.
     *          DLPF reduces sensor noise but adds group delay.
     *          
     *          Typical settings:
     *          - DLPF_CFG=0: 260Hz bandwidth, 0ms delay (MPU6000)
     *          - DLPF_CFG=1: 184Hz bandwidth, 2.0ms delay
     *          - DLPF_CFG=2: 92Hz bandwidth, 3.0ms delay
     *          - DLPF_CFG=3: 41Hz bandwidth, 5.9ms delay
     *          - DLPF_CFG=4: 20Hz bandwidth, 9.9ms delay
     *          - DLPF_CFG=5: 10Hz bandwidth, 17.85ms delay
     *          - DLPF_CFG=6: 5Hz bandwidth, 33.48ms delay
     *          
     *          Choice based on:
     *          - Fast sampling: Low DLPF (260Hz or 184Hz)
     *          - Standard sampling: Medium DLPF (92Hz)
     *          - Noisy environment: High DLPF (41Hz or lower)
     * 
     * @note Called during _hardware_init()
     * @note Different DLPF configurations for gyro and accel
     * @warning Lower bandwidth increases latency - avoid for fast control loops
     * 
     * Source: InvenSense register map documentation
     */
    void _set_filter_register(void);
    
    /**
     * @brief Reset FIFO to clear overflow or corruption
     * 
     * @details Full FIFO reset sequence:
     *          1. Disable FIFO (USER_CTRL register)
     *          2. Clear FIFO overflow flag
     *          3. Reset FIFO pointer
     *          4. Re-enable FIFO
     *          5. Discard first sample (invalid after reset)
     *          
     *          Called when:
     *          - FIFO overflow detected (FIFO_OFLOW bit set)
     *          - Unexpected temperature reading (sensor glitch)
     *          - Register change detected (sensor reset)
     *          
     *          Reset tracking:
     *          - Increments reset_count for diagnostics
     *          - Logs error if log_error=true
     *          - Records last_reset_ms timestamp
     * 
     * @param[in] log_error  If true, log FIFO reset event via AP::logger()
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note Causes sample loss during reset (~10ms gap)
     * @warning Frequent resets indicate CPU overload or hardware fault
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_fifo_reset()
     */
    void _fifo_reset(bool log_error) __RAMFUNC__;
    
    /**
     * @brief Fast FIFO reset (optimized for quick recovery)
     * 
     * @details Lightweight FIFO reset that skips full re-initialization.
     *          Faster than _fifo_reset() but may not clear all error states.
     *          
     *          Fast reset sequence:
     *          1. Pulse FIFO reset bit in USER_CTRL
     *          2. Re-enable FIFO immediately
     *          3. Minimal delay for pointer reset
     *          
     *          Used when:
     *          - Rapid recovery needed (e.g., fast control loops)
     *          - FIFO overflow due to transient CPU load
     *          - _enable_fast_fifo_reset parameter is true
     *          
     *          Tracking:
     *          - Increments fast_reset_count
     *          - Reports if reset rate exceeds threshold
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note Faster than full reset (~1ms vs ~10ms)
     * @warning May not clear all FIFO error states
     * @warning Not suitable for persistent FIFO corruption
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_fast_fifo_reset()
     */
    void _fast_fifo_reset() __RAMFUNC__;

    /**
     * @brief Check if device supports auxiliary I2C bus
     * 
     * @details Determines if sensor hardware includes auxiliary I2C master.
     *          
     *          Support by device:
     *          - MPU6000, MPU6500, MPU9250: Yes
     *          - ICM-20789: Yes (for barometer)
     *          - ICM-20608, ICM-20602: No
     *          
     *          Used during initialization to create AP_Invensense_AuxiliaryBus.
     * 
     * @return true if auxiliary bus supported, false otherwise
     * 
     * @note Based on _mpu_type detected during probe
     */
    bool _has_auxiliary_bus();

    /**
     * @brief Read samples from FIFO in burst transfer
     * 
     * @details FIFO read sequence:
     *          1. Read FIFO_COUNT registers (0x72-0x73) to get byte count
     *          2. Calculate number of complete samples: count / 12
     *          3. Perform burst read from FIFO_R_W register (0x74)
     *          4. Parse samples from _fifo_buffer
     *          5. Call _accumulate() to process samples
     *          
     *          FIFO packet format (12 bytes per sample):
     *          - Bytes 0-1: Accel X (big-endian int16)
     *          - Bytes 2-3: Accel Y
     *          - Bytes 4-5: Accel Z
     *          - Bytes 6-7: Temperature
     *          - Bytes 8-9: Gyro X
     *          - Bytes 10-11: Gyro Y
     *          - Bytes 12-13: Gyro Z (if auxiliary data enabled, more bytes follow)
     *          
     *          Overflow handling:
     *          - If FIFO_OFLOW bit set in INT_STATUS, call _fifo_reset()
     *          - Discard partial samples (count % 12 != 0)
     *          
     *          Performance optimization:
     *          - Uses DMA-safe aligned _fifo_buffer
     *          - Burst read minimizes SPI/I2C overhead
     *          - Processes multiple samples per call
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note Called from _poll_data() when data ready
     * @warning Must complete before next FIFO overflow
     * @warning Buffer size limits maximum samples per read
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_read_fifo()
     */
    void _read_fifo() __RAMFUNC__;

    /**
     * @brief Check if new sensor data is available
     * 
     * @details Two detection methods:
     *          
     *          1. DRDY pin (if configured):
     *             - Hardware interrupt pin signals data ready
     *             - Low latency, preferred method
     *             - Pin state read via _drdy_pin->read()
     *          
     *          2. INT_STATUS register (fallback):
     *             - Bit 0 (RAW_DATA_RDY_INT) indicates new data
     *             - Requires register read (slower than DRDY pin)
     *             - Used when DRDY pin not available
     *          
     *          Used by _poll_data() to determine when to read FIFO.
     * 
     * @return true if new data available, false otherwise
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note DRDY pin method preferred for lower latency
     * @note Register method adds SPI/I2C transaction overhead
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_data_ready()
     */
    bool _data_ready() __RAMFUNC__;

    /**
     * @brief Poll for new data (scheduler periodic callback)
     * 
     * @details Main data acquisition callback registered with scheduler.
     *          Called at backend rate (typically 1kHz).
     *          
     *          Polling sequence:
     *          1. Check if data ready via _data_ready()
     *          2. If ready, read FIFO via _read_fifo()
     *          3. Process samples via _accumulate()
     *          4. Check for register changes via _check_register_change() (debug)
     *          
     *          Non-blocking operation:
     *          - Returns immediately if no data available
     *          - Does not wait for data
     *          - Scheduler calls again at next period
     *          
     *          Timing requirements:
     *          - Must complete within scheduler time budget
     *          - Typical execution: 100-500μs depending on sample count
     *          - Overrun causes scheduler warnings and FIFO overflow
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note Called from scheduler interrupt or thread context
     * @warning Long execution blocks other scheduler tasks
     * @warning Must not call blocking functions (delay, I/O wait)
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_poll_data()
     */
    void _poll_data() __RAMFUNC__;

    /**
     * @brief Debug function to detect unexpected register changes
     * 
     * @details Monitors USER_CTRL register for unexpected changes.
     *          Unexpected changes indicate:
     *          - Sensor reset (power glitch, EMI)
     *          - Register corruption (hardware fault)
     *          - Software bug (incorrect write)
     *          
     *          Detection method:
     *          - Read USER_CTRL register periodically
     *          - Compare with _last_stat_user_ctrl
     *          - Log warning if mismatch detected
     *          - Trigger FIFO reset if FIFO config changed
     *          
     *          Enabled when:
     *          - _enable_offset_checking = true
     *          - Debug builds
     *          - Diagnosing hardware issues
     * 
     * @note Marked __RAMFUNC__ for use in critical path
     * @note Performance impact: Adds one register read per poll cycle
     * @note Disabled by default in production builds
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_check_register_change()
     */
    void _check_register_change(void) __RAMFUNC__;

    /**
     * @brief Read multiple registers in burst transfer
     * 
     * @details Burst read optimization for sequential register access.
     *          SPI/I2C bus automatically increments register address.
     *          
     *          Used for:
     *          - FIFO reads (large data blocks)
     *          - Multi-byte registers (FIFO_COUNT, sensor data)
     *          - Sequential configuration reads
     *          
     *          Bus differences:
     *          - SPI: Set bit 7 high for read, transfer continues
     *          - I2C: Send register address, read multiple bytes
     * 
     * @param[in]  reg   Starting register address
     * @param[out] buf   Buffer to store read data
     * @param[in]  size  Number of bytes to read
     * 
     * @return true if read successful, false on bus error
     * 
     * @note Marked __RAMFUNC__ for use in critical path
     * @note Much faster than multiple single-byte reads
     * @warning Buffer must be size bytes or larger
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_block_read()
     */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size) __RAMFUNC__;
    
    /**
     * @brief Read single register
     * 
     * @details Reads one byte from specified register.
     *          Wrapper around HAL device transfer functions.
     *          
     *          Bus protocol:
     *          - SPI: Send (reg | 0x80), receive data byte
     *          - I2C: Write reg address, read data byte
     * 
     * @param[in] reg  Register address to read
     * 
     * @return Register value (8-bit)
     * 
     * @note Marked __RAMFUNC__ for use in critical path
     * @note Returns 0 on bus error (indistinguishable from valid 0)
     * @see _block_read() for multi-byte reads
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_register_read()
     */
    uint8_t _register_read(uint8_t reg) __RAMFUNC__;
    
    /**
     * @brief Write single register
     * 
     * @details Writes one byte to specified register.
     *          
     *          Bus protocol:
     *          - SPI: Send reg address (bit 7 low), send data byte
     *          - I2C: Write reg address, write data byte
     *          
     *          Optional verification:
     *          - If checked=true, read back register to verify write
     *          - Used for critical configuration (FIFO enable, sample rate)
     *          - Not used for FIFO data register (write-only)
     * 
     * @param[in] reg      Register address to write
     * @param[in] val      Value to write (8-bit)
     * @param[in] checked  If true, verify write by reading back (default: false)
     * 
     * @note Marked __RAMFUNC__ for use in critical path
     * @note Checked writes add read transaction overhead
     * @warning Critical for configuration registers that affect data integrity
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_register_write()
     */
    void _register_write(uint8_t reg, uint8_t val, bool checked=false) __RAMFUNC__;

    /**
     * @brief Process accumulated FIFO samples
     * 
     * @details Processes raw samples from FIFO buffer:
     *          1. Parse 12-byte packets into accel, temp, gyro
     *          2. Convert from big-endian to host byte order
     *          3. Check temperature for validity via _check_raw_temp()
     *          4. Apply scale factors (_fifo_accel_scale, _fifo_gyro_scale)
     *          5. Apply rotation transformation (sensor → vehicle frame)
     *          6. Downsample to backend rate
     *          7. Publish to frontend via _publish_gyro() and _publish_accel()
     *          
     *          Temperature processing:
     *          - Filter via _temp_filter (low-pass 2nd order)
     *          - Used for temperature drift compensation
     *          - Typical range: 0-85°C operation
     *          
     *          Downsampling:
     *          - Accumulate samples at sensor rate
     *          - Average every N samples (downsample ratio)
     *          - Publish averaged result to frontend
     *          - Reduces CPU load and filter aliasing
     *          
     *          Error handling:
     *          - Invalid temperature triggers FIFO reset
     *          - Sample count mismatch discards partial samples
     *          - Bus errors logged and counted
     * 
     * @param[in] samples    Pointer to raw FIFO buffer
     * @param[in] n_samples  Number of complete 12-byte samples
     * 
     * @return true if all samples processed successfully, false on error
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note Critical performance path - optimized for speed
     * @warning Must complete quickly to avoid FIFO overflow
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_accumulate()
     */
    bool _accumulate(uint8_t *samples, uint8_t n_samples) __RAMFUNC__;
    
    /**
     * @brief Process samples with sensor-rate accumulation
     * 
     * @details Alternative accumulation mode that maintains separate accumulators
     *          for gyro and accel to support different sample rates.
     *          
     *          Motivation:
     *          - Gyro benefits from high sample rate (reduced aliasing)
     *          - Accel can use lower rate (less CPU, adequate bandwidth)
     *          - Different rates require separate accumulation
     *          
     *          Implementation:
     *          - Accumulate gyro samples at sensor rate
     *          - Accumulate accel samples at lower rate (ratio)
     *          - Apply pre-filter to accel (anti-aliasing)
     *          - Publish when accumulator full
     *          
     *          Configuration:
     *          - _gyro_to_accel_sample_ratio determines rate ratio
     *          - Typical: 8:1 (8kHz gyro, 1kHz accel)
     *          - _accum structure holds accumulators and counts
     * 
     * @param[in] samples    Pointer to raw FIFO buffer
     * @param[in] n_samples  Number of complete 12-byte samples
     * 
     * @return true if samples processed successfully, false on error
     * 
     * @note Marked __RAMFUNC__ for deterministic timing
     * @note Used when INS_FAST_SAMPLE enabled
     * @see _accumulate() for standard accumulation mode
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_accumulate_sensor_rate_sampling()
     */
    bool _accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples) __RAMFUNC__;

    /**
     * @brief Validate raw temperature reading
     * 
     * @details Sanity check on temperature to detect sensor glitches.
     *          
     *          Validation criteria:
     *          - Temperature change < threshold from last sample
     *          - Absolute value within reasonable range (-40 to +85°C)
     *          - No sudden jumps indicating sensor reset
     *          
     *          Invalid temperature indicates:
     *          - Sensor reset or power glitch
     *          - FIFO corruption
     *          - Hardware fault
     *          
     *          Response to invalid temperature:
     *          - Trigger FIFO reset via _fifo_reset()
     *          - Discard current sample
     *          - Log error for diagnostics
     * 
     * @param[in] t2  Raw temperature value to check
     * 
     * @return true if temperature valid, false if invalid
     * 
     * @note Marked __RAMFUNC__ for use in critical path
     * @note Prevents using corrupted data for flight control
     * @warning False positives possible during rapid temperature changes
     * 
     * Source: libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp:_check_raw_temp()
     */
    bool _check_raw_temp(int16_t t2) __RAMFUNC__;

    // Temperature measurement
    int16_t _raw_temp;  ///< Last raw temperature reading from sensor (16-bit signed)
    
    /**
     * @brief Temperature sensitivity (conversion factor)
     * 
     * @details Converts raw temperature ADC counts to degrees Celsius.
     *          Formula: temp_degC = temp_zero + raw_temp * temp_sensitivity
     *          
     *          Value: 1.0 / 340 = 0.00294 °C/LSB
     *          Source: InvenSense datasheets (MPU6000/9250/ICM-206xx)
     */
    float temp_sensitivity = 1.0f/340;  ///< Temperature scale: °C per LSB (1/340)
    
    /**
     * @brief Temperature zero offset
     * 
     * @details Temperature reading at 0 raw counts.
     *          Formula: temp_degC = temp_zero + raw_temp * temp_sensitivity
     *          
     *          Value: 36.53°C (room temperature calibration point)
     *          Source: InvenSense datasheets
     */
    float temp_zero = 36.53f;  ///< Temperature offset: °C at raw=0 (36.53°C)
    
    float _temp_filtered;       ///< Low-pass filtered temperature in °C
    float _accel_scale;         ///< Accelerometer scale factor (m/s² per LSB)
    float _gyro_scale;          ///< Gyroscope scale factor (rad/s per LSB)

    float _fifo_accel_scale;    ///< FIFO accelerometer scale (m/s² per LSB)
    float _fifo_gyro_scale;     ///< FIFO gyroscope scale (rad/s per LSB)
    LowPassFilter2pFloat _temp_filter;  ///< 2nd-order low-pass filter for temperature
    
    // FIFO reset tracking
    uint32_t last_reset_ms;              ///< Timestamp of last FIFO reset (milliseconds)
    uint8_t reset_count;                 ///< Total number of FIFO resets (full resets)
    uint8_t fast_reset_count;            ///< Total number of fast FIFO resets
    uint8_t last_fast_reset_count;       ///< Fast reset count at last report
    uint32_t last_fast_reset_count_report_ms;  ///< Timestamp of last reset count report

    enum Rotation _rotation;  ///< Board rotation for sensor-to-vehicle frame transform

    /**
     * @brief Enable register change detection (debug feature)
     * 
     * @details When enabled, _check_register_change() monitors USER_CTRL register
     *          for unexpected changes indicating sensor reset or corruption.
     *          
     *          Enables detection of:
     *          - Sensor hardware resets
     *          - Register corruption
     *          - Power glitches
     *          
     *          Performance impact: Adds one register read per poll cycle
     */
    bool _enable_offset_checking;

    /**
     * @brief Enable fast FIFO reset optimization
     * 
     * @details When true, use _fast_fifo_reset() instead of _fifo_reset().
     *          Fast reset is quicker but may not clear all error states.
     *          
     *          Trade-off:
     *          - Fast reset: ~1ms recovery, may miss edge cases
     *          - Full reset: ~10ms recovery, clears all states
     *          
     *          Recommended: true for racing/acro, false for standard flight
     */
    bool _enable_fast_fifo_reset;

    /**
     * @brief Cached Y-axis offset register (ICM-20602 specific)
     * 
     * @details ICM-20602 has errata where Y-axis offset register can be
     *          corrupted by FIFO operations. Driver caches value and
     *          restores if corruption detected.
     *          
     *          Workaround:
     *          1. Read YA_OFFS_H register during initialization
     *          2. Save to _saved_y_ofs_high
     *          3. Periodically verify and restore if changed
     *          
     *          Only used for ICM-20602 variant.
     */
    uint8_t _saved_y_ofs_high;

    AP_HAL::DigitalSource *_drdy_pin;     ///< Data ready interrupt pin (optional)
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< SPI or I2C device handle
    AP_Invensense_AuxiliaryBus *_auxiliary_bus;  ///< Auxiliary I2C bus for external sensors (or nullptr)

    enum Invensense_Type _mpu_type;  ///< Detected sensor type (MPU6000, ICM-20608, etc.)

    /**
     * @brief Fast sampling mode flag
     * 
     * @details True if backend rate exceeds 1kHz (e.g., 2kHz, 4kHz, 8kHz).
     *          Enables optimizations for high-rate sampling:
     *          - Reduced per-sample processing
     *          - Fast FIFO reset
     *          - Batch processing
     */
    bool _fast_sampling;

    /**
     * @brief Gyroscope FIFO downsample rate
     * 
     * @details Factor by which gyro samples are downsampled from sensor rate.
     *          
     *          Example:
     *          - Sensor rate: 8kHz
     *          - Downsample rate: 8
     *          - Backend rate: 8000 / 8 = 1000Hz
     *          
     *          Typical values: 1 (no downsample), 2, 4, 8
     */
    uint8_t _gyro_fifo_downsample_rate;

    /**
     * @brief Accelerometer FIFO downsample rate
     * 
     * @details Factor by which accel samples are downsampled from sensor rate.
     *          Often higher than gyro downsample (lower backend rate).
     *          
     *          Example:
     *          - Sensor rate: 8kHz
     *          - Downsample rate: 8
     *          - Backend rate: 8000 / 8 = 1000Hz
     *          
     *          Typical values: 4, 8, 16 (lower than gyro rate)
     */
    uint8_t _accel_fifo_downsample_rate;

    /**
     * @brief Ratio of gyro to accel sample rates
     * 
     * @details How many gyro samples per accel sample when using sensor-rate sampling.
     *          Used by _accumulate_sensor_rate_sampling() for differential rates.
     *          
     *          Example:
     *          - Gyro backend: 8kHz
     *          - Accel backend: 1kHz
     *          - Ratio: 8:1
     *          
     *          Enables high-rate gyro with lower-rate accel for CPU efficiency.
     */
    uint8_t _gyro_to_accel_sample_ratio;

    uint16_t _gyro_backend_rate_hz;   ///< Gyro samples published to frontend (Hz)
    uint16_t _accel_backend_rate_hz;  ///< Accel samples published to frontend (Hz)

    /**
     * @brief Last USER_CTRL register value
     * 
     * @details Cached for change detection via _check_register_change().
     *          USER_CTRL controls FIFO enable, I2C master, and sensor reset.
     *          Unexpected changes indicate sensor reset or corruption.
     *          
     *          Typical value: 0x40 (FIFO enabled, I2C master enabled)
     */
    uint8_t _last_stat_user_ctrl;

    /**
     * @brief FIFO read buffer (dynamically allocated)
     * 
     * @details DMA-safe aligned buffer for burst FIFO reads.
     *          Size: Sufficient for maximum samples between polls
     *          
     *          Calculation:
     *          - Sample size: 12 bytes
     *          - Max samples: (backend_period * sensor_rate)
     *          - Example: (1ms * 8kHz) = 8 samples = 96 bytes
     *          - Actual: Allocated with safety margin (typically 512 bytes)
     *          
     *          Allocated in _init(), freed in destructor.
     */
    uint8_t *_fifo_buffer;

    /**
     * @struct _accum
     * @brief Accumulators for sensor-rate sampling mode
     * 
     * @details Used by _accumulate_sensor_rate_sampling() to maintain separate
     *          accumulation for gyro and accel at different rates.
     *          
     *          Structure members:
     *          - accel: Accumulated accel samples (Vector3f in m/s²)
     *          - gyro: Accumulated gyro samples (Vector3f in rad/s)
     *          - accel_count: Number of accel samples in accumulator
     *          - gyro_count: Number of gyro samples in accumulator
     *          - accel_filter: Anti-aliasing low-pass filter (4kHz sensor, 188Hz cutoff)
     *          
     *          Operation:
     *          1. Accumulate samples at sensor rate
     *          2. When count reaches downsample ratio, compute average
     *          3. Publish average to frontend
     *          4. Reset accumulator
     *          
     *          Filter parameters:
     *          - Sample rate: 4000Hz (typical)
     *          - Cutoff frequency: 188Hz (Nyquist for 400Hz backend)
     *          
     *          Only used when INS_FAST_SAMPLE parameter enabled.
     * 
     * @see _accumulate_sensor_rate_sampling()
     */
    struct {
        Vector3f accel;          ///< Accumulated accel samples (m/s²)
        Vector3f gyro;           ///< Accumulated gyro samples (rad/s)
        uint8_t accel_count;     ///< Number of accel samples accumulated
        uint8_t gyro_count;      ///< Number of gyro samples accumulated
        LowPassFilterConstDtVector3f accel_filter{4000, 188};  ///< Anti-aliasing filter for accel
    } _accum;
};

/**
 * @class AP_Invensense_AuxiliaryBusSlave
 * @brief Represents an I2C slave device on InvenSense auxiliary bus
 * 
 * @details Provides interface to external I2C devices connected to InvenSense
 *          auxiliary I2C master. Typical use case is magnetometer (AK8963) in MPU9250.
 *          
 *          Auxiliary bus architecture:
 *          - InvenSense IMU has on-chip I2C master
 *          - Master can poll up to 4 external I2C slave devices
 *          - Slave data written to EXT_SENS_DATA registers (0x49-0x60)
 *          - Data automatically read at sensor sample rate
 *          - Optionally appended to FIFO stream
 *          
 *          Slave configuration:
 *          - I2C address of slave device
 *          - Register to read from slave
 *          - Number of bytes to read
 *          - Read frequency (sample rate divider)
 *          
 *          Operation modes:
 *          1. Passthrough mode: Direct I2C access for configuration
 *          2. Automatic mode: Periodic reads by I2C master
 *          
 *          Typical sequence:
 *          1. Use passthrough mode to configure slave (write registers)
 *          2. Configure automatic periodic reads
 *          3. Read slave data from EXT_SENS_DATA registers
 *          
 *          Example (AK8963 magnetometer in MPU9250):
 *          - Slave address: 0x0C
 *          - Data register: 0x03 (HXL)
 *          - Read size: 8 bytes (HXL-HZH + ST2)
 *          - Frequency: Every sensor sample (1kHz)
 * 
 * @note Slave instance is managed by AP_Invensense_AuxiliaryBus
 * @note Maximum 4 slave devices supported by hardware
 * @see AP_Invensense_AuxiliaryBus for bus management
 * 
 * Source: InvenSense register map, section "I2C Master"
 */
class AP_Invensense_AuxiliaryBusSlave : public AuxiliaryBusSlave
{
    friend class AP_Invensense_AuxiliaryBus;

public:
    /**
     * @brief Read from slave device in passthrough mode
     * 
     * @details Direct I2C read from slave device. InvenSense I2C master disabled,
     *          main I2C/SPI interface acts as I2C master.
     *          
     *          Used for:
     *          - Initial slave configuration
     *          - Reading slave identification registers
     *          - One-time reads not in periodic loop
     *          
     *          Passthrough sequence:
     *          1. Disable I2C master (USER_CTRL.I2C_MST_EN = 0)
     *          2. Enable I2C bypass (INT_PIN_CFG.BYPASS_EN = 1)
     *          3. Perform direct I2C read from slave
     *          4. Restore I2C master state
     * 
     * @param[in]  reg   Register address on slave device
     * @param[out] buf   Buffer to store read data
     * @param[in]  size  Number of bytes to read
     * 
     * @return Number of bytes read, or negative error code
     * 
     * @note Temporarily disrupts automatic slave reads
     * @warning Do not call during flight (use periodic reads instead)
     */
    int passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size) override;
    
    /**
     * @brief Write to slave device in passthrough mode
     * 
     * @details Direct I2C write to slave device for configuration.
     *          
     *          Used for:
     *          - Configuring slave device registers
     *          - Setting slave operating mode
     *          - Calibration commands
     *          
     *          Example (AK8963 magnetometer):
     *          - Write CNTL1 register (0x0A) to set mode
     *          - Write ASTC register (0x0C) for self-test
     * 
     * @param[in] reg  Register address on slave device
     * @param[in] val  Value to write to register
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Uses passthrough mode (disables I2C master temporarily)
     * @warning Do not call during flight
     */
    int passthrough_write(uint8_t reg, uint8_t val) override;

    /**
     * @brief Read latest data from slave (from EXT_SENS_DATA registers)
     * 
     * @details Reads slave data that was automatically captured by I2C master.
     *          Data is stored in EXT_SENS_DATA registers (0x49-0x60).
     *          
     *          Data flow:
     *          1. I2C master reads slave periodically (e.g., 1kHz)
     *          2. Slave data written to EXT_SENS_DATA_XX registers
     *          3. This method reads EXT_SENS_DATA registers
     *          4. Returns latest captured data
     *          
     *          Data freshness:
     *          - Updated at sensor sample rate
     *          - No I2C transaction overhead (data pre-fetched)
     *          - Low latency access
     * 
     * @param[out] buf  Buffer to store slave data
     * 
     * @return Number of bytes read, or negative error code
     * 
     * @note Fast operation (register read, no I2C transaction)
     * @note Safe to call during flight
     * @see _configure_periodic_read() to setup automatic reads
     */
    int read(uint8_t *buf) override;

protected:
    /**
     * @brief Constructor (called by AP_Invensense_AuxiliaryBus)
     * 
     * @details Creates slave instance and calculates register addresses.
     *          Register addresses depend on slave instance number (0-3).
     *          
     *          Slave registers (base + instance):
     *          - I2C_SLV0_ADDR + instance: Slave I2C address
     *          - I2C_SLV0_REG + instance: Slave register to read
     *          - I2C_SLV0_CTRL + instance: Control (enable, byte count)
     *          - I2C_SLV0_DO + instance: Data out (for writes)
     * 
     * @param[in] bus       Reference to parent auxiliary bus
     * @param[in] addr      I2C address of slave device (7-bit)
     * @param[in] instance  Slave number (0-3)
     * 
     * @note Protected constructor - use via AP_Invensense_AuxiliaryBus::get_slave()
     */
    AP_Invensense_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr, uint8_t instance);
    
    /**
     * @brief Configure I2C master for passthrough or periodic read
     * 
     * @details Internal helper to setup I2C master registers for slave access.
     *          
     *          Configuration:
     *          - Slave address (I2C_SLV0_ADDR)
     *          - Register to read (I2C_SLV0_REG)
     *          - Number of bytes (I2C_SLV0_CTRL)
     *          - Enable bit (I2C_SLV0_CTRL.EN)
     *          
     *          If out != nullptr, reads data from EXT_SENS_DATA.
     * 
     * @param[in]  reg   Register address on slave
     * @param[in]  size  Number of bytes to transfer
     * @param[out] out   Optional buffer to read EXT_SENS_DATA (can be nullptr)
     * 
     * @return 0 on success, negative on error
     * 
     * @note Internal method used by passthrough_read/write
     */
    int _set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = nullptr);

private:
    const uint8_t _mpu_addr;  ///< I2C_SLV0_ADDR register address (0x25 + instance)
    const uint8_t _mpu_reg;   ///< I2C_SLV0_REG register address (0x26 + instance)
    const uint8_t _mpu_ctrl;  ///< I2C_SLV0_CTRL register address (0x27 + instance)
    const uint8_t _mpu_do;    ///< I2C_SLV0_DO register address (0x63 + instance)

    /**
     * @brief Offset into EXT_SENS_DATA for this slave
     * 
     * @details Each slave gets portion of EXT_SENS_DATA registers.
     *          Offset determines starting register for this slave's data.
     *          
     *          Example:
     *          - Slave 0: _ext_sens_data = 0, uses 0x49-0x4F (8 bytes)
     *          - Slave 1: _ext_sens_data = 8, uses 0x51-0x57 (8 bytes)
     *          
     *          Total EXT_SENS_DATA space: 24 bytes (0x49-0x60)
     */
    uint8_t _ext_sens_data = 0;
};

/**
 * @class AP_Invensense_AuxiliaryBus
 * @brief Manages InvenSense on-chip I2C master for external sensors
 * 
 * @details Provides high-level interface to InvenSense auxiliary I2C bus.
 *          Manages multiple slave devices and configures automatic polling.
 *          
 *          Auxiliary bus capabilities:
 *          - Up to 4 external I2C slave devices
 *          - Automatic periodic reads at sensor rate
 *          - Data stored in EXT_SENS_DATA registers (24 bytes total)
 *          - Optional FIFO integration
 *          - Passthrough mode for slave configuration
 *          
 *          Common use cases:
 *          - MPU9250: AK8963 magnetometer on auxiliary bus
 *          - ICM-20789: BMP388 barometer on auxiliary bus
 *          - Custom: External compass, barometer, or other I2C sensors
 *          
 *          Bus architecture:
 *          - I2C master clock: 400kHz typical
 *          - Master controlled by I2C_MST_CTRL register
 *          - Each slave has dedicated register set
 *          - Slave data written to consecutive EXT_SENS_DATA locations
 *          
 *          Initialization sequence:
 *          1. Create auxiliary bus via get_auxiliary_bus()
 *          2. Get slave instance via get_slave()
 *          3. Configure slave via passthrough_write()
 *          4. Setup periodic read via configure_periodic_read()
 *          5. Read data via slave->read() in application
 *          
 *          Example (MPU9250 + AK8963):
 *          ```
 *          auto *aux_bus = imu->get_auxiliary_bus();
 *          auto *mag = aux_bus->get_slave(0x0C);  // AK8963 address
 *          mag->passthrough_write(0x0A, 0x16);     // Continuous mode 2
 *          aux_bus->configure_periodic_read(mag, 0x03, 8);  // Read HXL-ST2
 *          // In periodic callback:
 *          uint8_t data[8];
 *          mag->read(data);  // Get latest magnetometer data
 *          ```
 *          
 *          Thread safety:
 *          - All operations protected by IMU device semaphore
 *          - Safe to call from different threads
 *          - Passthrough temporarily disables I2C master
 *          
 *          Performance:
 *          - Slave reads add minimal overhead (automatic)
 *          - No I2C transaction latency for data reads
 *          - Data pre-fetched at sensor rate
 * 
 * @note Only available on sensors with I2C master (MPU9250, ICM-20789, etc.)
 * @note Maximum 4 slaves, 24 bytes total data
 * @warning Auxiliary bus shares I2C pins with main IMU (SPI-based IMUs only)
 * 
 * Source: InvenSense register map, "I2C Master Interface"
 */
class AP_Invensense_AuxiliaryBus : public AuxiliaryBus
{
    friend class AP_InertialSensor_Invensense;

public:
    /**
     * @brief Get semaphore for thread-safe access
     * 
     * @details Returns IMU device semaphore for synchronization.
     *          All auxiliary bus operations should hold this semaphore.
     *          
     *          Usage pattern:
     *          ```
     *          auto *sem = aux_bus->get_semaphore();
     *          if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
     *              // Perform auxiliary bus operations
     *              sem->give();
     *          }
     *          ```
     * 
     * @return Pointer to HAL semaphore (never nullptr)
     * 
     * @note Semaphore shared with main IMU device
     * @note Must be held for passthrough operations
     */
    AP_HAL::Semaphore *get_semaphore() override;
    
    /**
     * @brief Register periodic callback for auxiliary bus operations
     * 
     * @details Registers callback to be called periodically by scheduler.
     *          Used for processing slave data or auxiliary bus management.
     *          
     *          Typical use:
     *          - Read slave data and publish to frontend
     *          - Monitor auxiliary bus status
     *          - Implement sensor-specific processing
     *          
     *          Example:
     *          ```
     *          auto handle = aux_bus->register_periodic_callback(
     *              25000,  // 25ms = 40Hz
     *              FUNCTOR_BIND_MEMBER(&MyClass::_aux_update, void)
     *          );
     *          ```
     * 
     * @param[in] period_usec  Callback period in microseconds
     * @param[in] cb           Callback function
     * 
     * @return Periodic handle for callback management
     * 
     * @note Callback runs at specified period independent of IMU rate
     * @note Multiple callbacks can be registered
     * @see AP_HAL::Device::PeriodicHandle for handle management
     */
    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb) override;

protected:
    /**
     * @brief Constructor (called by AP_InertialSensor_Invensense)
     * 
     * @details Creates auxiliary bus instance for InvenSense IMU.
     *          Initializes I2C master and prepares for slave configuration.
     * 
     * @param[in] backend  Reference to parent Invensense backend
     * @param[in] devid    Device ID for identification
     * 
     * @note Protected constructor - use via get_auxiliary_bus()
     */
    AP_Invensense_AuxiliaryBus(AP_InertialSensor_Invensense &backend, uint32_t devid);

    /**
     * @brief Create slave device instance
     * 
     * @details Factory method to create AP_Invensense_AuxiliaryBusSlave.
     *          Allocates slave instance and assigns register set.
     *          
     *          Slave assignment:
     *          - Instance 0: Uses I2C_SLV0_* registers
     *          - Instance 1: Uses I2C_SLV1_* registers
     *          - Instance 2: Uses I2C_SLV2_* registers
     *          - Instance 3: Uses I2C_SLV3_* registers
     *          
     *          EXT_SENS_DATA allocation:
     *          - Each slave assigned portion of 24-byte buffer
     *          - Offset tracked in _ext_sens_data
     * 
     * @param[in] addr      I2C address of slave device (7-bit)
     * @param[in] instance  Slave number (0-3)
     * 
     * @return Pointer to new slave instance, or nullptr if allocation failed
     * 
     * @note Maximum 4 slaves supported
     * @note Caller responsible for slave lifecycle management
     */
    AuxiliaryBusSlave *_instantiate_slave(uint8_t addr, uint8_t instance) override;
    
    /**
     * @brief Configure automatic periodic reads from slave
     * 
     * @details Sets up I2C master to automatically read from slave device.
     *          Data appears in EXT_SENS_DATA registers at sensor rate.
     *          
     *          Configuration registers (per slave):
     *          - I2C_SLV0_ADDR: Slave I2C address | read bit
     *          - I2C_SLV0_REG: Starting register address on slave
     *          - I2C_SLV0_CTRL: Enable bit + byte count
     *          
     *          Read frequency:
     *          - Same as sensor sample rate (typically 1kHz)
     *          - Configurable via I2C_MST_DELAY_CTRL (sample rate divider)
     *          
     *          Data flow:
     *          1. I2C master reads slave at sensor rate
     *          2. Data written to EXT_SENS_DATA registers
     *          3. Application reads via slave->read()
     *          4. No I2C transaction overhead for application
     *          
     *          Example (AK8963 magnetometer):
     *          - reg = 0x03 (HXL register)
     *          - size = 8 (HXL, HXH, HYL, HYH, HZL, HZH, ST2)
     *          - Reads mag data + status at 1kHz
     * 
     * @param[in] slave  Slave device to read from
     * @param[in] reg    Starting register address on slave
     * @param[in] size   Number of bytes to read (max depends on EXT_SENS_DATA space)
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Must have sufficient EXT_SENS_DATA space (24 bytes total)
     * @note Slave must be configured (via passthrough_write) before periodic reads
     * @warning Do not exceed MAX_EXT_SENS_DATA (24 bytes) across all slaves
     */
    int _configure_periodic_read(AuxiliaryBusSlave *slave, uint8_t reg,
                                 uint8_t size) override;

private:
    /**
     * @brief Configure all slaves (helper)
     * 
     * @details Applies slave configurations to I2C master registers.
     *          Called after all slaves added and configured.
     *          
     *          Operations:
     *          - Enable I2C master
     *          - Set I2C master clock (400kHz)
     *          - Configure each slave's register set
     *          - Enable automatic reads
     * 
     * @note Internal helper method
     */
    void _configure_slaves();

    /**
     * @brief Maximum EXT_SENS_DATA buffer size
     * 
     * @details Hardware limit on auxiliary bus data storage.
     *          Total 24 bytes shared across all slaves (0-3).
     *          
     *          Register range: 0x49-0x60 (EXT_SENS_DATA_00 to EXT_SENS_DATA_23)
     *          
     *          Example allocation:
     *          - Slave 0: 8 bytes (magnetometer)
     *          - Slave 1: 6 bytes (barometer)
     *          - Slave 2: 10 bytes (custom sensor)
     *          - Total: 24 bytes (at limit)
     */
    static const uint8_t MAX_EXT_SENS_DATA = 24;
    
    /**
     * @brief Current offset into EXT_SENS_DATA buffer
     * 
     * @details Tracks next available byte in EXT_SENS_DATA for slave allocation.
     *          Incremented as slaves are added and configured.
     *          
     *          Used by _instantiate_slave() to assign EXT_SENS_DATA portion.
     */
    uint8_t _ext_sens_data = 0;
};

/**
 * @def INS_INVENSENSE_20789_I2C_ADDR
 * @brief Default I2C address for ICM-20789
 * 
 * @details ICM-20789 I2C address when AD0 pin is low.
 *          Can be overridden in board configuration if non-standard address used.
 *          
 *          I2C addressing:
 *          - AD0 = 0 (low): 0x68 (default)
 *          - AD0 = 1 (high): 0x69 (alternate)
 *          
 *          ICM-20789 features:
 *          - 6-axis IMU (gyro + accel)
 *          - Integrated BMP388 barometer on auxiliary bus
 *          - Typical use: Space-constrained flight controllers
 * 
 * @note Can be defined in hwdef to override default
 * @note Only applies to I2C-based ICM-20789 (SPI doesn't use I2C address)
 */
#ifndef INS_INVENSENSE_20789_I2C_ADDR
#define INS_INVENSENSE_20789_I2C_ADDR 0x68
#endif
