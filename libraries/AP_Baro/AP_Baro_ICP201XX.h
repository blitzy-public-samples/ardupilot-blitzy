/**
 * @file AP_Baro_ICP201XX.h
 * @brief Driver for TDK InvenSense ICP-201XX high-accuracy barometric pressure sensor
 * 
 * @details This file implements support for the TDK InvenSense ICP-201XX series 
 *          barometric pressure sensors. The ICP-201XX represents the next generation
 *          of barometric sensors with enhanced accuracy, lower noise, and improved
 *          temperature stability compared to previous generations.
 *          
 *          Key Features:
 *          - High accuracy pressure measurement
 *          - Low noise for stable altitude estimation
 *          - Multiple operating modes for power/performance trade-offs
 *          - FIFO buffering for efficient data collection
 *          - On-chip temperature compensation
 *          - Factory calibration via OTP (One-Time Programmable) memory
 *          
 *          The driver handles device initialization, calibration data retrieval,
 *          continuous measurement operation, and data accumulation for the
 *          ArduPilot barometer subsystem.
 * 
 * @note Pressure values are reported in Pascals (Pa)
 * @note Temperature values are reported in degrees Celsius (°C)
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_ICP201XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

/**
 * @class AP_Baro_ICP201XX
 * @brief Backend driver for TDK InvenSense ICP-201XX barometric pressure sensors
 * 
 * @details This class provides the interface between the ArduPilot barometer subsystem
 *          and the ICP-201XX family of pressure sensors. The ICP-201XX series offers
 *          next-generation performance with enhanced features including:
 *          
 *          Enhanced Features:
 *          - Improved pressure accuracy for precise altitude estimation
 *          - Lower noise characteristics for stable readings in all conditions
 *          - Multiple bandwidth/ODR (Output Data Rate) modes for application optimization
 *          - Hardware FIFO for efficient data collection
 *          - Built-in temperature sensor with compensation
 *          - Factory-calibrated OTP coefficients for accurate measurements
 *          
 *          Device-Specific Calibration:
 *          The ICP-201XX uses factory-programmed OTP (One-Time Programmable) memory
 *          containing calibration coefficients. These coefficients are automatically
 *          read during initialization and applied to raw sensor readings to compensate
 *          for manufacturing variations and temperature effects.
 *          
 *          Measurement Process:
 *          1. Probe and detect sensor on I2C bus
 *          2. Perform soft reset and boot sequence
 *          3. Read OTP calibration data
 *          4. Configure operating mode and FIFO
 *          5. Start continuous measurements
 *          6. Periodically read and accumulate pressure/temperature data
 *          7. Report averaged values to barometer subsystem
 * 
 * @note All pressure measurements are in Pascals (Pa)
 * @note All temperature measurements are in degrees Celsius (°C)
 * @note This driver uses I2C communication protocol
 * 
 * @see AP_Baro_Backend
 * @see AP_Baro
 */
class AP_Baro_ICP201XX : public AP_Baro_Backend
{
public:
    /**
     * @brief Update barometer readings and push to frontend
     * 
     * @details Called periodically by the barometer subsystem to retrieve accumulated
     *          sensor data. This method calculates averages from the accumulated
     *          pressure and temperature readings, then reports them to the AP_Baro
     *          frontend for use by the navigation system.
     *          
     *          The accumulation approach reduces noise and provides stable readings
     *          by averaging multiple samples collected by the timer callback.
     * 
     * @note Must be called at regular intervals to maintain current readings
     * @note Thread-safe: Protected by semaphore for accumulation structure access
     */
    void update() override;

    /**
     * @brief Probe I2C bus for ICP-201XX sensor and create driver instance
     * 
     * @details Attempts to detect an ICP-201XX sensor on the provided I2C device.
     *          If successful, creates a new driver instance, initializes the sensor,
     *          and returns the backend pointer for registration with AP_Baro.
     *          
     *          Probe sequence:
     *          1. Attempt communication with sensor
     *          2. Verify device ID/signature
     *          3. Perform initialization sequence
     *          4. Configure for continuous operation
     *          5. Register periodic timer callback
     * 
     * @param[in] baro Reference to AP_Baro frontend instance
     * @param[in] dev  I2C device pointer with configured address and bus
     * 
     * @return Pointer to initialized AP_Baro_Backend on success, nullptr on failure
     * 
     * @note This is a static factory method called during sensor auto-detection
     * @note Caller takes ownership of returned pointer
     */
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

private:
    /**
     * @brief Private constructor for ICP-201XX driver instance
     * 
     * @details Creates a new driver instance with the specified I2C device.
     *          Called only by the probe() method after successful device detection.
     *          Initializes member variables and prepares for sensor initialization.
     * 
     * @param[in] baro Reference to AP_Baro frontend for data reporting
     * @param[in] dev  I2C device pointer (ownership transferred)
     * 
     * @note Constructor is private - use probe() static method to create instances
     */
    AP_Baro_ICP201XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Initialize ICP-201XX sensor hardware
     * 
     * @details Performs complete initialization sequence including:
     *          - Soft reset to known state
     *          - Boot sequence execution
     *          - OTP calibration data retrieval
     *          - Sensor configuration and mode selection
     *          - Timer callback registration for periodic reads
     * 
     * @return true if initialization successful, false on any error
     * 
     * @note Called by probe() after device detection
     * @note Failure will prevent driver registration
     */
    bool init();
    
    /**
     * @brief Perform dummy register read for timing/synchronization
     * 
     * @details Executes a dummy register read operation. This may be required
     *          by the ICP-201XX hardware for proper timing or to clear internal
     *          state in certain operating conditions.
     * 
     * @note Device-specific requirement for ICP-201XX operation
     */
    void dummy_reg();
    
    /**
     * @brief Read multiple bytes from sensor register
     * 
     * @details Reads consecutive bytes starting from specified register address.
     *          Used for multi-byte data retrieval such as pressure/temperature
     *          values and calibration coefficients.
     * 
     * @param[in]  reg Register address to read from
     * @param[out] buf Buffer to store read data
     * @param[in]  len Number of bytes to read
     * 
     * @return true if read successful, false on I2C error
     * 
     * @note I2C transaction protected by device semaphore
     */
    bool read_reg(uint8_t reg, uint8_t *buf, uint8_t len);
    
    /**
     * @brief Read single byte from sensor register
     * 
     * @details Convenience method for single-byte register reads.
     * 
     * @param[in]  reg Register address to read from
     * @param[out] val Pointer to store read value
     * 
     * @return true if read successful, false on I2C error
     */
    bool read_reg(uint8_t reg, uint8_t *val);
    
    /**
     * @brief Write single byte to sensor register
     * 
     * @details Writes a byte value to the specified register address.
     *          Used for sensor configuration and mode control.
     * 
     * @param[in] reg Register address to write to
     * @param[in] val Value to write
     * 
     * @return true if write successful, false on I2C error
     * 
     * @note I2C transaction protected by device semaphore
     */
    bool write_reg(uint8_t reg, uint8_t val);
    
    /**
     * @brief Select and configure sensor operating mode
     * 
     * @details Configures the ICP-201XX operating mode which determines the
     *          bandwidth, output data rate (ODR), and power consumption.
     *          Different modes provide trade-offs between update rate,
     *          noise performance, and power usage.
     * 
     * @param[in] mode Operating mode value (see OP_MODE enum)
     * 
     * @return true if mode selection successful, false on error
     * 
     * @see OP_MODE
     * @note Mode affects both measurement rate and power consumption
     */
    bool mode_select(uint8_t mode);
    
    /**
     * @brief Read OTP (One-Time Programmable) calibration data
     * 
     * @details Retrieves factory calibration coefficients from the sensor's OTP memory.
     *          These coefficients are essential for accurate pressure and temperature
     *          measurements as they compensate for manufacturing variations and
     *          provide device-specific correction factors.
     * 
     * @param[in]  addr OTP memory address to read
     * @param[in]  cmd  OTP read command
     * @param[out] val  Pointer to store read calibration value
     * 
     * @return true if OTP read successful, false on error
     * 
     * @note OTP data is read-only and programmed during manufacturing
     * @note Critical for measurement accuracy - initialization fails if OTP read fails
     */
    bool read_otp_data(uint8_t addr, uint8_t cmd, uint8_t *val);
    
    /**
     * @brief Retrieve current pressure and temperature measurements
     * 
     * @details Reads raw pressure and temperature data from the sensor,
     *          applies calibration coefficients, and converts to standard units.
     *          This is the core measurement function called by the timer callback.
     * 
     * @param[out] pressure    Compensated pressure measurement in Pascals (Pa)
     * @param[out] temperature Compensated temperature measurement in Celsius (°C)
     * 
     * @return true if valid data retrieved, false on read error or invalid data
     * 
     * @note Applies OTP calibration coefficients to raw readings
     * @note Temperature compensation is performed for pressure accuracy
     */
    bool get_sensor_data(float *pressure, float *temperature);
    
    /**
     * @brief Execute soft reset of sensor
     * 
     * @details Performs a software reset of the ICP-201XX, returning it to
     *          default power-on state. Used during initialization to ensure
     *          the sensor starts from a known configuration.
     * 
     * @note Clears all configuration registers
     * @note Sensor must be reconfigured after soft reset
     */
    void soft_reset();
    
    /**
     * @brief Execute sensor boot sequence
     * 
     * @details Performs the ICP-201XX specific boot/initialization sequence
     *          required after power-on or reset. This may include internal
     *          self-tests and readiness checks.
     * 
     * @return true if boot sequence completed successfully, false on timeout or error
     * 
     * @note Must be called after soft_reset() during initialization
     * @note Device-specific timing requirements must be observed
     */
    bool boot_sequence();
    
    /**
     * @brief Configure sensor for continuous operation
     * 
     * @details Applies complete configuration including:
     *          - Operating mode selection
     *          - Power mode configuration
     *          - Measurement mode (continuous vs forced)
     *          - FIFO readout mode
     *          - Data output rate
     * 
     * @return true if configuration successful, false on error
     * 
     * @note Called during initialization after boot sequence
     * @note Configuration determines sensor behavior during operation
     */
    bool configure();
    
    /**
     * @brief Wait for sensor data ready
     * 
     * @details Implements timing delay to ensure sensor has completed measurement
     *          before attempting to read data. Based on configured ODR and
     *          measurement mode.
     * 
     * @note Required to prevent reading incomplete or stale data
     * @note Timing depends on selected operating mode
     */
    void wait_read();
    
    /**
     * @brief Flush sensor FIFO buffer
     * 
     * @details Clears the hardware FIFO buffer by reading and discarding all
     *          stored samples. Used during initialization or error recovery to
     *          ensure fresh data on next read.
     * 
     * @return true if FIFO flushed successfully, false on error
     * 
     * @note Necessary after configuration changes
     * @note Ensures no stale data from previous mode
     */
    bool flush_fifo();
    
    /**
     * @brief Periodic timer callback for sensor data collection
     * 
     * @details Called at regular intervals by the HAL scheduler to:
     *          1. Read current pressure and temperature from sensor
     *          2. Accumulate readings for averaging
     *          3. Maintain measurement statistics
     *          
     *          Accumulated data is later retrieved by update() method
     *          for reporting to the navigation system.
     * 
     * @note Runs in timer context with strict timing requirements
     * @note Uses semaphore to protect accumulation structure
     * @note Failure to read does not stop timer - next cycle will retry
     */
    void timer();

    /// Barometer instance number assigned by AP_Baro frontend
    uint8_t instance;

    /// I2C device pointer for hardware communication
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    /**
     * @brief Data accumulation structure for averaging multiple samples
     * 
     * @details Accumulates pressure and temperature readings from the timer callback
     *          for averaging by the update() method. This reduces measurement noise
     *          and provides stable values to the navigation system.
     *          
     *          Protected by device semaphore for thread-safe access between
     *          timer context and update() method.
     */
    struct {
        float tsum;      ///< Accumulated temperature sum in Celsius (°C)
        float psum;      ///< Accumulated pressure sum in Pascals (Pa)
        uint32_t count;  ///< Number of accumulated samples
    } accum;

    /// Timestamp of last measurement command in microseconds (for timing control)
    uint32_t last_measure_us;

    /**
     * @enum OP_MODE
     * @brief Operating mode configuration for ICP-201XX
     * 
     * @details Defines available operating modes that control the sensor's bandwidth (Bw)
     *          and output data rate (ODR). Each mode provides different trade-offs between:
     *          - Update rate (how fast new measurements are available)
     *          - Noise performance (lower bandwidth = lower noise)
     *          - Power consumption (higher rates = more power)
     *          
     *          Mode selection affects altitude estimation quality and responsiveness.
     */
    enum class OP_MODE : uint8_t {
		OP_MODE0 = 0,   ///< Mode 0: Bandwidth 6.25 Hz, ODR 25Hz - Low noise, moderate update rate
		OP_MODE1,       ///< Mode 1: Bandwidth 30 Hz, ODR 120Hz - Fast updates, higher noise
		OP_MODE2,       ///< Mode 2: Bandwidth 10 Hz, ODR 40Hz - Balanced performance (default)
		OP_MODE3,       ///< Mode 3: Bandwidth 0.5 Hz, ODR 2Hz - Lowest noise, slow updates
		OP_MODE4,       ///< Mode 4: User configurable - Custom bandwidth/ODR settings
	} _op_mode{OP_MODE::OP_MODE2};

	/**
     * @enum FIFO_READOUT_MODE
     * @brief FIFO data readout configuration
     * 
     * @details Controls how pressure and temperature data are stored and retrieved
     *          from the hardware FIFO buffer. Affects data ordering and which
     *          measurements are stored.
     *          
     *          The FIFO buffer allows efficient burst reading of multiple samples,
     *          reducing I2C transaction overhead and improving timing consistency.
     */
	enum class FIFO_READOUT_MODE : uint8_t {
		FIFO_READOUT_MODE_PRES_TEMP = 0,   ///< Pressure and temperature pairs, pressure first (default)
		FIFO_READOUT_MODE_TEMP_ONLY = 1,   ///< Temperature only - saves FIFO space if pressure not needed
		FIFO_READOUT_MODE_TEMP_PRES = 2,   ///< Pressure and temperature pairs, temperature first
		FIFO_READOUT_MODE_PRES_ONLY = 3    ///< Pressure only - for high-rate altitude tracking
	} _fifo_readout_mode{FIFO_READOUT_MODE::FIFO_READOUT_MODE_PRES_TEMP};

	/**
     * @enum POWER_MODE
     * @brief Power management mode selection
     * 
     * @details Controls sensor power state and clock behavior:
     *          
     *          Normal Mode: Device enters standby between measurements, activating
     *          only during measurement execution. Reduces power consumption at the
     *          cost of slight measurement latency.
     *          
     *          Active Mode: Sensor remains powered with high-frequency clock always
     *          enabled. Provides fastest measurement response but higher power draw.
     */
	enum class POWER_MODE : uint8_t {
		POWER_MODE_NORMAL = 0,  ///< Normal: Standby with measurement-time activation (default, lower power)
		POWER_MODE_ACTIVE = 1   ///< Active: Continuous power and clock (faster response, higher power)
	} _power_mode{POWER_MODE::POWER_MODE_NORMAL};

	/**
     * @enum MEAS_MODE
     * @brief Measurement triggering mode
     * 
     * @details Selects how measurements are initiated:
     *          
     *          Forced Mode: Measurements triggered explicitly by software command.
     *          Provides precise control over measurement timing but requires active
     *          management. Useful for synchronized sampling or power optimization.
     *          
     *          Continuous Mode: Sensor automatically takes measurements at the rate
     *          specified by the operating mode ODR. Simplifies operation and ensures
     *          consistent timing without software intervention.
     */
	enum MEAS_MODE : uint8_t {
		MEAS_MODE_FORCED_TRIGGER = 0, ///< Forced: Software-triggered measurements for precise control
		MEAS_MODE_CONTINUOUS = 1   ///< Continuous: Auto-trigger at ODR rate (default for ArduPilot)
	} _meas_mode{MEAS_MODE::MEAS_MODE_CONTINUOUS};

	/**
     * @enum FORCED_MEAS_TRIGGER
     * @brief Forced measurement trigger control (used when MEAS_MODE_FORCED_TRIGGER selected)
     * 
     * @details When operating in forced measurement mode, this controls whether
     *          the sensor remains in standby or executes a measurement cycle.
     *          
     *          Standby: Sensor waits in low-power state for trigger command.
     *          
     *          Trigger: Initiates a single measurement cycle, after which sensor
     *          returns to standby unless continuous mode is enabled.
     * 
     * @note Only relevant when _meas_mode is MEAS_MODE_FORCED_TRIGGER
     * @note ArduPilot typically uses CONTINUOUS mode, so this is usually STANDBY
     */
	enum FORCED_MEAS_TRIGGER : uint8_t {
		FORCE_MEAS_STANDBY = 0,			///< Standby: Wait in low-power state (default)
		FORCE_MEAS_TRIGGER_FORCE_MEAS = 1	///< Trigger: Execute single measurement cycle
	} _forced_meas_trigger{FORCED_MEAS_TRIGGER::FORCE_MEAS_STANDBY};
};

#endif  // AP_BARO_ICP201XX_ENABLED 
