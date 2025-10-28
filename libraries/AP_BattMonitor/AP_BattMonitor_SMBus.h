/**
 * @file AP_BattMonitor_SMBus.h
 * @brief SMBus smart battery protocol implementation per Smart Battery Data Specification Rev 1.1
 * 
 * @details This file implements the base class for SMBus/I2C smart battery interfaces.
 *          SMBus (System Management Bus) is a two-wire communication protocol derived from I2C,
 *          commonly used for battery monitoring in laptops, UAVs, and other portable devices.
 *          
 *          The Smart Battery Data Specification defines standard registers for querying battery
 *          voltage, current, capacity, temperature, and other parameters. This implementation
 *          supports optional PEC (Packet Error Code) CRC-8 error checking for enhanced reliability.
 *          
 *          Key features:
 *          - Standard Smart Battery Data Specification Rev 1.1 register reads
 *          - Optional PEC (CRC-8) error detection for reliable communication
 *          - Periodic polling at approximately 10Hz
 *          - Support for internal and external I2C buses
 *          - Battery health monitoring with 5-second timeout
 * 
 * @note Default I2C address: 0x0B (standard SMBus smart battery address)
 * @note Timeout: 5 seconds - battery becomes unhealthy if no successful readings
 * @warning PEC error checking is required for reliable communication with some smart batteries
 * 
 * @author ArduPilot Development Team
 */

#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_SMBUS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

/// Internal I2C bus selection (typically built-in to flight controller)
#define AP_BATTMONITOR_SMBUS_BUS_INTERNAL           0
/// External I2C bus selection (typically external port/connector)
#define AP_BATTMONITOR_SMBUS_BUS_EXTERNAL           1
/// Standard SMBus smart battery I2C address
#define AP_BATTMONITOR_SMBUS_I2C_ADDR               0x0B
/// Timeout threshold - sensor becomes unhealthy if no successful readings for 5 seconds
#define AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS         5000000         // sensor becomes unhealthy if no successful readings for 5 seconds
/// Maximum SMBus block transfer size per Smart Battery Data Specification
#define AP_BATTMONITOR_SMBUS_READ_BLOCK_MAXIMUM_TRANSFER 0x20       // A Block Read or Write is allowed to transfer a maximum of 32 data bytes.

/**
 * @class AP_BattMonitor_SMBus
 * @brief Base class for SMBus/I2C smart battery interfaces implementing Smart Battery Data Specification Rev 1.1
 * 
 * @details This class provides the core SMBus communication infrastructure for smart battery monitoring.
 *          It implements standard register reads with optional PEC (Packet Error Code) CRC-8 error checking
 *          to ensure reliable communication with smart batteries.
 *          
 *          SMBus is a two-wire communication protocol (derived from I2C) that allows the autopilot to query
 *          battery parameters including voltage, current, remaining capacity, temperature, cycle count,
 *          and manufacturer information.
 *          
 *          Derived classes implement specific battery vendor protocols by overriding timer() and optionally
 *          providing vendor-specific capacity scaling via get_capacity_scaler().
 *          
 *          Communication protocol:
 *          - Periodic polling at ~10Hz via timer() callback
 *          - 16-bit word reads for most parameters
 *          - Block reads for string data (manufacturer name, etc.)
 *          - Optional PEC byte for error detection
 *          
 *          Thread safety: All reads executed in I2C bus scheduler context
 * 
 * @note Supports both internal and external I2C buses
 * @warning Ensure proper I2C pull-up resistors (typically 4.7kΩ) are present on hardware
 */
class AP_BattMonitor_SMBus : public AP_BattMonitor_Backend
{
public:

    /**
     * @enum BATTMONITOR_SMBUS
     * @brief Smart Battery Data Specification Rev 1.1 standard register addresses
     * 
     * @details These register addresses are defined by the Smart Battery Data Specification
     *          and are standardized across compliant smart battery implementations. Each register
     *          provides specific battery telemetry or identification data.
     */
    enum BATTMONITOR_SMBUS {
        BATTMONITOR_SMBUS_TEMP = 0x08,                 ///< Temperature register: Returns battery temperature in 0.1 Kelvin units (e.g., 2981 = 298.1K = 25°C)
        BATTMONITOR_SMBUS_VOLTAGE = 0x09,              ///< Voltage register: Returns battery voltage in millivolts (mV)
        BATTMONITOR_SMBUS_CURRENT = 0x0A,              ///< Current register: Returns battery current in milliamps (mA), signed 16-bit (positive = charging, negative = discharging)
        BATTMONITOR_SMBUS_REMAINING_CAPACITY = 0x0F,   ///< Remaining Capacity register: Returns remaining battery capacity in milliamp-hours (mAh)
        BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY = 0x10, ///< Full Charge Capacity register: Returns full charge capacity in milliamp-hours (mAh), accounts for battery degradation
        BATTMONITOR_SMBUS_CYCLE_COUNT = 0x17,          ///< Cycle Count register: Returns number of discharge cycles experienced by the battery
        BATTMONITOR_SMBUS_SPECIFICATION_INFO = 0x1A,   ///< Specification Info register: Returns Smart Battery Data Specification version supported by battery
        BATTMONITOR_SMBUS_SERIAL = 0x1C,               ///< Serial Number register: Returns unique battery serial number (16-bit)
        BATTMONITOR_SMBUS_MANUFACTURE_NAME = 0x20,     ///< Manufacturer Name register: Block read returns manufacturer name string (max 32 bytes)
        BATTMONITOR_SMBUS_MANUFACTURE_DATA = 0x23,     ///< Manufacturer Data register: Block read returns vendor-specific manufacturing data (max 32 bytes)
    };

    /**
     * @brief Constructor for SMBus smart battery monitor
     * 
     * @details Initializes the SMBus battery monitor backend with specified I2C bus configuration.
     *          Does not initiate I2C communication - call init() after construction to begin monitoring.
     * 
     * @param[in] mon Reference to parent AP_BattMonitor instance managing all battery monitors
     * @param[in,out] mon_state Reference to battery state structure for storing telemetry data
     * @param[in] params Reference to battery monitor configuration parameters
     * @param[in] i2c_bus I2C bus selection: AP_BATTMONITOR_SMBUS_BUS_INTERNAL (0) for internal bus,
     *                                       AP_BATTMONITOR_SMBUS_BUS_EXTERNAL (1) for external bus
     * 
     * @note Constructor does not perform I2C probe - init() must be called separately
     */
    AP_BattMonitor_SMBus(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_BattMonitor_Params &params,
                    uint8_t i2c_bus);

    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes
     * 
     * @details Empty implementation as I2C device cleanup is handled by HAL layer.
     *          Virtual to reduce compiler warnings and ensure proper polymorphic deletion.
     */
    virtual ~AP_BattMonitor_SMBus() {}

    /**
     * @brief Check if individual cell voltages are available
     * 
     * @details Some SMBus batteries provide individual cell voltage data through vendor-specific
     *          registers. This capability is detected and flagged by derived class implementations.
     * 
     * @return true if cell voltages have been successfully read from battery, false otherwise
     */
    bool has_cell_voltages() const override { return _has_cell_voltages; }

    /**
     * @brief Check if temperature measurement is available
     * 
     * @details Temperature support is detected during initialization by attempting to read
     *          the BATTMONITOR_SMBUS_TEMP register (0x08).
     * 
     * @return true if temperature has been successfully read from battery, false otherwise
     */
    bool has_temperature() const override { return _has_temperature; }

    /**
     * @brief Check if current measurement is available
     * 
     * @details All SMBus smart batteries conforming to Smart Battery Data Specification
     *          are required to provide current measurement via register 0x0A.
     * 
     * @return true (always available for SMBus batteries)
     */
    bool has_current() const override { return true; }

    /**
     * @brief Attempt to reset remaining battery capacity to specified percentage
     * 
     * @details SMBus smart batteries manage their own capacity tracking using internal
     *          fuel gauge ICs. External capacity reset is not supported as it would
     *          conflict with the battery's internal state estimation and could lead
     *          to inaccurate capacity reporting or unexpected shutdowns.
     * 
     * @param[in] percentage Desired capacity percentage (unused for SMBus)
     * 
     * @return false (capacity reset not supported for SMBus smart batteries)
     * 
     * @note To recalibrate a smart battery, perform a full discharge/charge cycle
     */
    bool reset_remaining(float percentage) override { return false; }

    /**
     * @brief Retrieve battery cycle count if available
     * 
     * @details Cycle count represents the number of charge/discharge cycles the battery
     *          has experienced. One cycle is approximately equal to one full DesignCapacity
     *          worth of discharge. Used for battery health monitoring and aging estimation.
     * 
     * @param[out] cycles Reference to store cycle count value
     * 
     * @return true if cycle count has been successfully read from battery, false otherwise
     */
    bool get_cycle_count(uint16_t &cycles) const override;

    /**
     * @brief Initialize SMBus battery monitor and start periodic polling
     * 
     * @details Performs the following initialization sequence:
     *          1. Obtains I2C device handle from HAL for specified bus and address
     *          2. Configures I2C transfer parameters (frequency, retries)
     *          3. Probes battery registers to verify communication
     *          4. Detects PEC (Packet Error Code) support by testing with/without CRC
     *          5. Registers periodic timer callback for ongoing battery monitoring at ~10Hz
     *          
     *          If initialization fails, timer callback will not be registered and the
     *          battery monitor will report unhealthy status.
     * 
     * @note Must be called after construction before battery monitoring will function
     * @note Executed in main thread context, timer callback runs in I2C scheduler context
     */
    virtual void init(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Periodic battery state update called at approximately 10Hz
     * 
     * @details This method is called periodically by the backend scheduler to refresh battery
     *          telemetry data. It reads current voltage and current, updates battery state,
     *          and manages the healthy/unhealthy status based on communication success.
     *          
     *          Typical read sequence performed by derived classes in their timer() implementation:
     *          1. Read voltage (register 0x09)
     *          2. Read current (register 0x0A)
     *          3. Read temperature if available (register 0x08)
     *          4. Periodically read capacity (registers 0x0F, 0x10)
     *          5. Occasionally read serial number and cycle count
     *          
     *          If no successful reads occur within AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS (5 seconds),
     *          the battery is marked unhealthy.
     * 
     * @note Executed in I2C scheduler task context, not main thread
     * @note Keep execution time minimal to avoid blocking I2C bus
     */
    void read(void) override;

    /**
     * @brief Read battery's full charge capacity accounting for degradation
     * 
     * @details Reads register 0x10 (FULL_CHARGE_CAPACITY) which returns the predicted battery
     *          capacity when fully charged, accounting for battery aging and degradation.
     *          This value typically decreases over the battery's lifetime as cells degrade.
     *          
     *          The capacity may be scaled by get_capacity_scaler() for vendor-specific corrections.
     * 
     * @note Value read in milliamp-hours (mAh)
     * @note Should be read periodically (not every cycle) to detect capacity degradation
     */
    void read_full_charge_capacity(void);

    /**
     * @brief Read battery's remaining capacity
     * 
     * @details Reads register 0x0F (REMAINING_CAPACITY) which returns the predicted remaining
     *          battery capacity in mAh. This is only valid after full_charge_capacity has been
     *          successfully read, as it's used to calculate the percentage of charge remaining.
     *          
     *          The capacity may be scaled by get_capacity_scaler() for vendor-specific corrections.
     * 
     * @note Value read in milliamp-hours (mAh)
     * @note Will not update state if full_charge_capacity is unknown
     */
    void read_remaining_capacity(void);

    /**
     * @brief Get vendor-specific capacity scaling factor
     * 
     * @details Some battery vendors report capacity values that need scaling to match actual
     *          capacity in mAh. For example, some batteries report capacity in units of 10mAh
     *          requiring a scaler of 10. Derived classes override this to provide vendor-specific
     *          scaling factors discovered through testing or manufacturer documentation.
     * 
     * @return Scaling factor to multiply by reported capacity values to get actual mAh (default: 1)
     * 
     * @note Base implementation returns 1 (no scaling)
     * @note Example: If battery reports 500 and scaler is 10, actual capacity is 5000 mAh
     */
    virtual uint16_t get_capacity_scaler() const { return 1; }

    /**
     * @brief Read temperature from battery
     * 
     * @details Reads register 0x08 (TEMP) which returns battery temperature in units of 0.1 Kelvin.
     *          For example, a value of 2981 represents 298.1K which is 25.0°C.
     *          
     *          Conversion: Celsius = (value / 10.0) - 273.15
     *          
     *          Virtual to allow derived classes to implement vendor-specific temperature reading.
     * 
     * @note Temperature in 0.1 Kelvin units (not Celsius or Fahrenheit)
     * @note Sets _has_temperature flag on first successful read
     */
    virtual void read_temp(void);

    /**
     * @brief Read battery serial number if not already known
     * 
     * @details Reads register 0x1C (SERIAL) which returns a unique 16-bit serial number
     *          identifying the specific battery pack. Serial number is only read once and
     *          cached in _serial_number. Used for battery identification and logging.
     * 
     * @note Only performs I2C read if _serial_number is -1 (unknown)
     * @note Serial number stored as int32_t to use -1 as sentinel value
     */
    void read_serial_number(void);

    /**
     * @brief Read battery's cycle count
     * 
     * @details Reads register 0x17 (CYCLE_COUNT) which returns the number of discharge cycles
     *          the battery has experienced. One cycle is approximately equal to one DesignCapacity
     *          worth of discharge. Used for battery health monitoring and end-of-life prediction.
     *          
     *          Cycle count is read periodically (not every update) and cached in _cycle_count.
     * 
     * @note Sets _has_cycle_count flag on first successful read
     * @note Typical battery lifespan: 300-500 cycles for lithium polymer/ion cells
     */
    void read_cycle_count();

    /**
     * @brief Read 16-bit word from SMBus register
     * 
     * @details Performs a standard SMBus "Read Word" transaction to read a 16-bit value from
     *          the specified register. Data is read as little-endian (low byte first, high byte second).
     *          
     *          If PEC is supported (_pec_supported == true), an additional CRC-8 byte is read and
     *          verified. Transaction fails if PEC check fails.
     *          
     *          SMBus protocol for Read Word:
     *          1. Master sends: [START][ADDR+W][COMMAND][REPEATED START][ADDR+R]
     *          2. Slave sends: [DATA_LOW][DATA_HIGH][PEC (optional)][STOP]
     * 
     * @param[in] reg Register address to read (e.g., BATTMONITOR_SMBUS_VOLTAGE = 0x09)
     * @param[out] data Reference to store 16-bit value read from register
     * 
     * @return true if read successful and PEC valid (if enabled), false if I2C error or PEC mismatch
     * 
     * @note Data returned in native uint16_t format (little-endian conversion handled internally)
     * @note Typical execution time: 1-2ms depending on I2C clock speed
     */
    bool read_word(uint8_t reg, uint16_t& data) const;

    /**
     * @brief Read block of bytes from SMBus register
     * 
     * @details Performs a SMBus "Read Block" transaction to read up to 32 bytes of data.
     *          Used for reading string data like manufacturer name or vendor-specific data.
     *          
     *          SMBus Read Block protocol:
     *          1. Master sends: [START][ADDR+W][COMMAND][REPEATED START][ADDR+R]
     *          2. Slave sends: [BYTE_COUNT][DATA_BYTE_1]...[DATA_BYTE_N][PEC (optional)][STOP]
     *          
     *          The first byte returned by the slave indicates the number of data bytes to follow.
     *          Maximum block size is 32 bytes per Smart Battery Data Specification.
     * 
     * @param[in] reg Register address to read (e.g., BATTMONITOR_SMBUS_MANUFACTURE_NAME = 0x20)
     * @param[out] data Pointer to buffer to store read data (must be at least len bytes)
     * @param[in] len Maximum number of bytes to read (must be ≤ AP_BATTMONITOR_SMBUS_READ_BLOCK_MAXIMUM_TRANSFER = 32)
     * 
     * @return Number of bytes successfully read (0 to len), or 0 if transaction failed
     * 
     * @note Buffer must be pre-allocated by caller with sufficient size
     * @note Null termination is NOT added automatically for string data
     * @warning Ensure buffer size is adequate - no bounds checking performed
     */
    uint8_t read_block(uint8_t reg, uint8_t* data, uint8_t len) const;

    /**
     * @brief Calculate PEC (Packet Error Code) CRC-8 for SMBus transaction
     * 
     * @details Calculates CRC-8 checksum used by SMBus for error detection. PEC is computed
     *          over the entire transaction including address, command, and data bytes.
     *          
     *          CRC-8 polynomial: x^8 + x^2 + x^1 + 1 (0x07)
     *          Initial value: 0x00
     *          
     *          For Read Word transaction, PEC is calculated over:
     *          [I2C_ADDR<<1 | 0][COMMAND][I2C_ADDR<<1 | 1][DATA_LOW][DATA_HIGH]
     *          
     *          This provides detection of:
     *          - Single-bit errors
     *          - Double-bit errors  
     *          - Odd number of bit errors
     *          - Most burst errors
     * 
     * @param[in] i2c_addr 7-bit I2C address of battery (typically 0x0B)
     * @param[in] cmd SMBus command/register address
     * @param[in] reading true if this is a read transaction, false if write
     * @param[in] buff Pointer to data buffer containing bytes read from or to be written to battery
     * @param[in] len Number of bytes in buff
     * 
     * @return 8-bit CRC value to compare against PEC byte returned by battery
     * 
     * @note PEC checking significantly improves communication reliability in noisy environments
     * @note Not all smart batteries support PEC - detected during init()
     */
    uint8_t get_PEC(const uint8_t i2c_addr, uint8_t cmd, bool reading, const uint8_t buff[], uint8_t len) const;

    /// I2C device handle for SMBus communication with battery (provided by HAL layer)
    AP_HAL::I2CDevice *_dev;
    
    /// Flag indicating if battery supports PEC (Packet Error Code) CRC-8 error checking - detected during init() by testing communication with/without PEC byte
    bool _pec_supported;

    /// Battery serial number (16-bit value from register 0x1C), -1 indicates not yet read
    int32_t _serial_number = -1;
    
    /// Full charge capacity in mAh accounting for battery degradation (register 0x10), used to calculate percentage remaining
    uint16_t _full_charge_capacity;
    
    /// Flag set true by derived classes once individual cell voltages have been successfully read
    bool _has_cell_voltages;
    
    /// Number of discharge cycles battery has experienced (register 0x17), approximately equal to total DesignCapacity discharged
    uint16_t _cycle_count = 0;
    
    /// Flag indicating if cycle count has been successfully retrieved from battery
    bool _has_cycle_count;
    
    /// Flag indicating if temperature sensor is present and has been successfully read
    bool _has_temperature;

    /**
     * @brief Pure virtual timer function to read battery telemetry data
     * 
     * @details Derived classes must implement this function to perform periodic battery reads.
     *          Called at approximately 10Hz by the I2C scheduler. Should read voltage, current,
     *          and other telemetry registers, then call read() to update battery state.
     * 
     * @note Executed in I2C scheduler context, not main thread
     * @note Keep execution time minimal to avoid blocking I2C bus
     */
    virtual void timer(void) = 0;

    /// Handle for periodic timer callback registered with I2C scheduler during init()
    AP_HAL::Device::PeriodicHandle timer_handle;

    // Parameters
    
    /// I2C bus selection parameter: 0 = internal bus, 1 = external bus (user configurable)
    AP_Int8  _bus;
    
    /// I2C address parameter (default: 0x0B per SMBus specification, user configurable for non-standard batteries)
    AP_Int8  _address;

};

#endif  // AP_BATTERY_SMBUS_ENABLED
