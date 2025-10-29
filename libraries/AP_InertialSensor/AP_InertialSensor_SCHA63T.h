/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_InertialSensor_SCHA63T.h
 * @brief Murata SCHA63T high-reliability dual-channel IMU driver
 * 
 * @details Implements backend driver for Murata SCHA63T automotive/industrial-grade IMU
 *          featuring redundant measurement channels for safety-critical applications.
 *          
 *          The SCHA63T is unusual as it has separate chip-select for accelerometer and
 *          gyroscope, which means it needs two SPIDevice pointers (similar to BMI055 architecture).
 *          
 *          Key Features:
 *          - Dual independent measurement channels per axis for redundancy
 *          - Cross-axis interference rejection
 *          - Automotive qualification (AEC-Q100)
 *          - Built-in self-test (BIST) capabilities
 *          - CRC protection on SPI transactions
 *          - Wide temperature range operation
 * 
 * @note Designed for safety-critical automotive and industrial applications
 * @see AP_InertialSensor_Backend
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor_SCHA63T.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

/**
 * @class AP_InertialSensor_SCHA63T
 * @brief Backend driver for SCHA63T dual-redundant IMU
 * 
 * @details Industrial-grade IMU with built-in redundancy:
 *          - Dual independent measurement channels per axis
 *          - Cross-axis interference rejection
 *          - Automotive qualification (AEC-Q100)
 *          - Gyro range: ±125 deg/s (high sensitivity) or ±300 deg/s
 *          - Accel range: ±4.5g
 *          - SPI interface with dual-channel readout
 *          
 *          Redundancy architecture:
 *          - Two independent gyro die per chip
 *          - Two independent accel die per chip
 *          - Driver compares channels for fault detection
 *          - Disagreement threshold triggers health flag
 *          
 *          Safety features:
 *          - Built-in self-test (BIST)
 *          - CRC on every SPI transaction
 *          - Status registers for fault indication
 *          - Temperature monitoring per channel
 *          
 *          Device architecture:
 *          - Requires two separate SPI chip-select lines (accel and gyro)
 *          - Each subsystem (accel/gyro) has independent register map
 *          - Bank-based register addressing via SEL_BANK
 *          - 16-bit signed sensor data with dual-channel readout
 * 
 * @note Designed for safety-critical automotive/industrial use
 * @note Lower measurement range trades for higher reliability
 * @warning Channel disagreement may indicate sensor fault or extreme vibration
 * @warning CRC failures must be treated as critical sensor faults
 * 
 * @see AP_InertialSensor_Backend
 * @see https://www.murata.com/en-global/products/sensor/gyro/scha63t
 */
class AP_InertialSensor_SCHA63T : public AP_InertialSensor_Backend
{
public:
    /**
     * @brief Probe for SCHA63T sensor on SPI bus and create backend instance
     * 
     * @details Factory method that detects the presence of a SCHA63T sensor
     *          and initializes the driver if found. The SCHA63T requires two
     *          separate SPI chip-select lines, one for accelerometer and one
     *          for gyroscope subsystems.
     *          
     *          Probe sequence:
     *          1. Verify SPI communication on both chip-selects
     *          2. Read device identification registers (T_ID0, T_ID1, T_ID2)
     *          3. Verify sensor status registers
     *          4. Create and initialize backend instance if successful
     *          
     *          The probe is non-invasive and will not affect sensor operation
     *          if probe fails.
     * 
     * @param[in,out] imu              Reference to main InertialSensor interface
     * @param[in]     dev_accel        SPI device pointer for accelerometer chip-select
     * @param[in]     dev_gyro         SPI device pointer for gyroscope chip-select
     * @param[in]     rotation         Board rotation for sensor orientation correction
     * 
     * @return Pointer to initialized backend instance if probe successful, nullptr if failed
     * 
     * @note Both SPI device pointers must be valid and configured for SCHA63T timing
     * @note This is called during sensor detection at boot time
     * @warning Ownership of SPI device pointers is transferred to backend if successful
     * 
     * @see AP_InertialSensor_Backend::probe()
     */
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_accel,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev_gyro,
                                            enum Rotation rotation);

    /**
     * @brief Configure the sensors and start periodic reading routine
     * 
     * @details Initializes the SCHA63T sensor configuration and registers
     *          periodic callbacks for data acquisition. Called once after
     *          successful probe during system initialization.
     *          
     *          Configuration sequence:
     *          1. Set operating mode (normal/low-power)
     *          2. Configure digital filters for both accel and gyro
     *          3. Enable CRC checking
     *          4. Perform startup self-test
     *          5. Register periodic read callbacks at sensor ODR
     *          
     *          The SCHA63T is configured for:
     *          - Maximum output data rate (ODR)
     *          - Digital low-pass filtering appropriate for flight control
     *          - Dual-channel readout for redundancy checking
     *          - CRC validation on all SPI transactions
     * 
     * @note Called from main thread during initialization
     * @note Must complete quickly to avoid delaying boot sequence
     * @warning Failure to start will disable this IMU instance
     * 
     * @see AP_InertialSensor_Backend::start()
     */
    void start() override;
    
    /**
     * @brief Update sensor data and push to frontend
     * 
     * @details Called periodically by scheduler to retrieve latest sensor data
     *          and publish to the InertialSensor frontend. Performs dual-channel
     *          readout and redundancy checking.
     *          
     *          Update sequence:
     *          1. Read gyroscope data from both channels
     *          2. Read accelerometer data from both channels
     *          3. Read temperature from both channels
     *          4. Validate CRC on all transactions
     *          5. Compare channel disagreement for fault detection
     *          6. Apply rotation correction
     *          7. Push samples to frontend if valid
     *          
     *          Health monitoring:
     *          - CRC failures increment error counter
     *          - Channel disagreement checked against threshold
     *          - Temperature monitoring for thermal drift
     *          - Status register checks for fault conditions
     * 
     * @return true if update successful and data published, false if sensor fault detected
     * 
     * @note Called at IMU sample rate (typically 1000+ Hz)
     * @note Must complete within timing budget to avoid scheduler overruns
     * @warning Return false triggers sensor health flag and may initiate failsafe
     * 
     * @see AP_InertialSensor_Backend::update()
     */
    bool update() override;

    /**
     * @enum reg_scha63t
     * @brief SCHA63T register address definitions
     * 
     * @details Register map for SCHA63T IMU. The sensor uses bank-based addressing
     *          via the SEL_BANK register to access different register pages.
     *          Most registers return 16-bit data with dual-channel support.
     *          
     *          Register organization:
     *          - 0x01-0x07: Sensor data registers (rate, acceleration, temperature)
     *          - 0x0E-0x15: Status and configuration registers
     *          - 0x16-0x1A: Filter and mode control registers
     *          - 0x1C-0x1E: Device identification registers
     *          - 0x1F: Bank selection register
     * 
     * @note All sensor data registers return signed 16-bit values
     * @note Status registers include CRC and error flags
     * @warning Register access timing must follow datasheet specifications
     */
    enum reg_scha63t {
        RATE_XZ     = 0x01,  ///< Gyro rate X and Z channels (combined register)
        RATE_Y      = 0x03,  ///< Gyro rate Y channel
        ACC_X       = 0x04,  ///< Accelerometer X axis
        ACC_Y       = 0x05,  ///< Accelerometer Y axis
        ACC_Z       = 0x06,  ///< Accelerometer Z axis
        TEMP        = 0x07,  ///< Temperature sensor data
        S_SUM       = 0x0E,  ///< Summary status register (error flags)
        R_S1        = 0x10,  ///< Rate sensor status channel 1
        A_S1        = 0x12,  ///< Accelerometer status channel 1
        C_S1        = 0x14,  ///< Common status channel 1
        C_S2        = 0x15,  ///< Common status channel 2
        G_FILT_DYN  = 0x16,  ///< Gyro filter configuration (dynamic range and bandwidth)
        RESCTRL     = 0x18,  ///< Reset control register
        MODE        = 0x19,  ///< Operating mode configuration (normal/low-power)
        A_FILT_DYN  = 0x1A,  ///< Accelerometer filter configuration (dynamic range and bandwidth)
        T_ID2       = 0x1C,  ///< Device identification register 2
        T_ID0       = 0x1D,  ///< Device identification register 0
        T_ID1       = 0x1E,  ///< Device identification register 1
        SEL_BANK    = 0x1F,  ///< Bank selection register for extended register access
    };

private:
    /**
     * @brief Private constructor for SCHA63T backend
     * 
     * @details Constructs backend instance with ownership of SPI device pointers.
     *          Constructor is private; instances are created only through probe() method.
     *          
     *          Stores device pointers and rotation matrix for later initialization
     *          during start() call. Does not perform any hardware access.
     * 
     * @param[in,out] imu              Reference to main InertialSensor interface
     * @param[in]     dev_accel        SPI device pointer for accelerometer (ownership transferred)
     * @param[in]     dev_gyro         SPI device pointer for gyroscope (ownership transferred)
     * @param[in]     rotation         Board rotation for sensor orientation correction
     * 
     * @note Called only from probe() static method
     * @note Hardware initialization deferred to init() and start() methods
     */
    AP_InertialSensor_SCHA63T(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_accel,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev_gyro,
                              enum Rotation rotation);

    /**
     * @brief Initialize SCHA63T sensor hardware
     * 
     * @details Performs hardware initialization and configuration of the SCHA63T sensor.
     *          Called from probe() to verify sensor presence and prepare for operation.
     *          
     *          Initialization sequence:
     *          1. Verify SPI communication on both chip-selects
     *          2. Read and validate device ID registers (T_ID0, T_ID1, T_ID2)
     *          3. Perform software reset via RESCTRL register
     *          4. Configure operating mode (normal mode for flight control)
     *          5. Set filter parameters (G_FILT_DYN, A_FILT_DYN)
     *          6. Check startup status registers for self-test pass
     *          7. Clear any error flags in summary status
     *          8. Register sensor instances with frontend
     *          
     *          Filter configuration:
     *          - Gyro: Low-pass filter appropriate for flight control bandwidth
     *          - Accel: Low-pass filter for vibration rejection
     *          - Dynamic range set per datasheet specifications
     * 
     * @return true if initialization successful, false if sensor not responding or failed self-test
     * 
     * @note Called during boot sequence from probe()
     * @warning Failure to initialize disables this IMU instance
     * @warning Must verify CRC on all configuration writes
     * 
     * @see check_startup()
     */
    bool init();

    /**
     * @brief Read accelerometer data from SCHA63T
     * 
     * @details Reads 3-axis accelerometer data from all channels via SPI.
     *          Performs dual-channel readout for redundancy checking.
     *          
     *          Read sequence:
     *          1. Read ACC_X register for both channels
     *          2. Read ACC_Y register for both channels
     *          3. Read ACC_Z register for both channels
     *          4. Verify CRC on each transaction
     *          5. Compare channel values for disagreement
     *          6. Convert raw values to m/s²
     *          7. Apply rotation matrix
     *          
     *          Data format:
     *          - 16-bit signed integer per axis
     *          - Range: ±4.5g
     *          - Sensitivity: ~13,653 LSB/g (0.073 mg/LSB)
     * 
     * @note Called from periodic scheduler callback at IMU sample rate
     * @note Uses dev_uno (accelerometer chip-select)
     * @warning CRC failure increments error counter and may mark sensor unhealthy
     * @warning Large channel disagreement indicates sensor fault
     */
    void read_accel();
    
    /**
     * @brief Read gyroscope data from SCHA63T
     * 
     * @details Reads 3-axis gyroscope data from all channels via SPI.
     *          Performs dual-channel readout for redundancy checking.
     *          
     *          Read sequence:
     *          1. Read RATE_XZ register (combined X and Z channels)
     *          2. Read RATE_Y register
     *          3. Verify CRC on each transaction
     *          4. Compare channel values for disagreement
     *          5. Convert raw values to rad/s
     *          6. Apply rotation matrix
     *          
     *          Data format:
     *          - 16-bit signed integer per axis
     *          - Range: ±125 deg/s or ±300 deg/s (configurable)
     *          - Sensitivity depends on configured range
     *          
     *          Note on X/Z combined register:
     *          - RATE_XZ register contains both X and Z gyro data
     *          - Requires special handling to extract individual axes
     * 
     * @note Called from periodic scheduler callback at IMU sample rate
     * @note Uses dev_due (gyroscope chip-select)
     * @warning CRC failure increments error counter and may mark sensor unhealthy
     * @warning Large channel disagreement indicates sensor fault
     */
    void read_gyro();

    /**
     * @brief Read register value from SCHA63T via SPI with CRC validation
     * 
     * @details Low-level register read with CRC checking. Performs SPI transaction
     *          to read 16-bit register value and validates CRC per SCHA63T protocol.
     *          
     *          SPI transaction format:
     *          - Command frame: Address + read bit
     *          - Response frame: 16-bit data + CRC
     *          - CRC polynomial per datasheet specification
     * 
     * @param[in]  tp   Device type/chip-select selector (0=accel, 1=gyro)
     * @param[in]  reg  Register address to read from reg_scha63t enum
     * @param[out] val  Pointer to buffer for 16-bit register value
     * 
     * @return true if read successful and CRC valid, false if SPI failure or CRC error
     * 
     * @note Increments error counter on CRC failure
     * @warning CRC failures indicate communication problems or sensor malfunction
     */
    bool read_register(uint8_t tp, reg_scha63t reg, uint8_t* val);
    
    /**
     * @brief Write register value to SCHA63T via SPI with CRC generation
     * 
     * @details Low-level register write with CRC generation. Performs SPI transaction
     *          to write 16-bit register value with appropriate CRC.
     *          
     *          SPI transaction format:
     *          - Command frame: Address + write bit + 16-bit data + CRC
     *          - Response frame: Status indication
     *          
     *          Configuration registers require specific timing between writes
     *          per datasheet specifications.
     * 
     * @param[in] tp   Device type/chip-select selector (0=accel, 1=gyro)
     * @param[in] reg  Register address to write from reg_scha63t enum
     * @param[in] val  16-bit value to write to register
     * 
     * @return true if write successful and acknowledged, false if SPI failure
     * 
     * @note Some registers require reset or mode change to take effect
     * @warning Incorrect writes to configuration registers may affect sensor operation
     * @warning Must follow register write timing requirements from datasheet
     */
    bool write_register(uint8_t tp, reg_scha63t reg, uint16_t val);
    
    /**
     * @brief Update temperature reading for sensor instance
     * 
     * @details Sets the temperature value for a sensor instance in the frontend.
     *          Temperature monitoring is important for:
     *          - Thermal drift compensation
     *          - Health monitoring (out-of-range detection)
     *          - Calibration validity checking
     *          
     *          Temperature format:
     *          - Raw value from TEMP register (signed 16-bit)
     *          - Conversion formula per datasheet
     *          - Resolution: typically 0.1°C per LSB
     * 
     * @param[in] instance  IMU instance number (gyro or accel)
     * @param[in] temper    Raw temperature value from TEMP register
     * 
     * @note Called during each sensor read cycle
     * @note Temperature used for thermal drift compensation in EKF
     */
    void set_temperature(uint8_t instance, int16_t temper);
    
    /**
     * @brief Verify sensor startup and self-test completion
     * 
     * @details Checks status registers to verify SCHA63T has completed power-on
     *          self-test (POST) successfully and is ready for operation.
     *          
     *          Startup checks:
     *          1. Read summary status register (S_SUM) for error flags
     *          2. Verify rate sensor status (R_S1) shows no faults
     *          3. Verify accelerometer status (A_S1) shows no faults
     *          4. Check common status registers (C_S1, C_S2) for warnings
     *          5. Ensure no CRC errors or communication faults
     *          
     *          Status flags indicate:
     *          - Self-test pass/fail status
     *          - Sensor saturation or clipping
     *          - Internal circuit faults
     *          - Temperature out of range
     * 
     * @return true if startup checks pass and sensor ready, false if self-test failed or faults detected
     * 
     * @note Called during init() sequence before sensor is marked operational
     * @warning Failure indicates hardware fault - sensor should not be used
     * @warning May require power cycle to clear persistent fault conditions
     * 
     * @see init()
     */
    bool check_startup();

    AP_HAL::OwnPtr<AP_HAL::Device> dev_uno;  ///< SPI device for accelerometer chip-select
    AP_HAL::OwnPtr<AP_HAL::Device> dev_due;  ///< SPI device for gyroscope chip-select

    enum Rotation rotation;  ///< Board rotation matrix for sensor orientation correction
};
