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
 * @file AP_Compass_MMC3416.h
 * @brief Driver for MEMSIC MMC3416xPJ 3-axis magnetoresistive magnetometer
 * 
 * @details This driver implements support for the MEMSIC MMC3416xPJ family of
 *          magnetometers, which use Anisotropic Magnetoresistance (AMR) technology
 *          to measure magnetic field strength. The MMC3416 provides high sensitivity
 *          magnetic field measurement with built-in SET/RESET degaussing capabilities
 *          to eliminate sensor offset drift.
 * 
 *          Key Features:
 *          - 3-axis magnetic field sensing
 *          - ±8 gauss field measurement range
 *          - 16-bit resolution per axis
 *          - I2C interface (address 0x30)
 *          - Built-in SET/RESET coils for offset elimination
 *          - Field measurements in milligauss units
 * 
 *          The driver implements a state machine that alternates between SET and RESET
 *          operations to compensate for AMR sensor offset drift, improving measurement
 *          accuracy and stability over time and temperature.
 * 
 * Source: libraries/AP_Compass/AP_Compass_MMC3416.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MMC3416_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_MMC3416_I2C_ADDR
# define HAL_COMPASS_MMC3416_I2C_ADDR 0x30
#endif

/**
 * @class AP_Compass_MMC3416
 * @brief Backend driver for MEMSIC MMC3416xPJ magnetometer using AMR technology
 * 
 * @details This class implements the AP_Compass backend interface for the MEMSIC
 *          MMC3416xPJ magnetometer. The sensor uses Anisotropic Magnetoresistance
 *          (AMR) technology, which measures changes in electrical resistance of
 *          ferromagnetic materials when exposed to magnetic fields.
 * 
 *          AMR Technology:
 *          AMR sensors provide high sensitivity and low power consumption but can
 *          experience offset drift due to temperature changes and magnetic field
 *          exposure. The MMC3416 addresses this through built-in SET and RESET
 *          coils that periodically degauss the sensor, restoring it to a known
 *          magnetic state and eliminating accumulated offset drift.
 * 
 *          Operating Principle:
 *          The driver implements a state machine that alternates between:
 *          1. SET operation: Applies positive saturation field to sensor
 *          2. Measurement after SET
 *          3. RESET operation: Applies negative saturation field to sensor
 *          4. Measurement after RESET
 *          5. Offset calculation from SET-RESET difference
 * 
 *          This SET/RESET cycle eliminates temperature-induced offset drift and
 *          provides stable, accurate magnetic field measurements over time.
 * 
 *          Technical Specifications:
 *          - Measurement range: ±8 gauss per axis
 *          - Resolution: 16-bit (0.0625 mG per LSB)
 *          - Output units: milligauss (mG)
 *          - Interface: I2C at address 0x30
 *          - Sample rate: Configurable via periodic callback
 * 
 * @note The SET/RESET degaussing procedure is essential for maintaining accuracy
 *       and should not be disabled or modified without understanding AMR sensor
 *       physics.
 * 
 * @warning Magnetic interference during SET/RESET operations can affect calibration
 *          accuracy. Sensor should be mounted away from strong magnetic sources.
 */
class AP_Compass_MMC3416 : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize MMC3416 magnetometer
     * 
     * @details This static factory method attempts to detect and initialize a
     *          MMC3416 magnetometer on the provided I2C device. The probe sequence:
     *          1. Reads device ID register to confirm MMC3416 presence
     *          2. Performs initial sensor configuration
     *          3. Initializes SET/RESET degaussing state machine
     *          4. Registers the compass instance with the AP_Compass frontend
     * 
     *          If detection succeeds, returns a heap-allocated instance of this
     *          driver. If detection fails (wrong device ID or I2C communication
     *          error), returns nullptr and releases the I2C device.
     * 
     * @param[in] dev I2C device pointer with ownership transfer - if probe fails,
     *                device is automatically released; if probe succeeds, driver
     *                takes ownership for sensor lifetime
     * @param[in] force_external Forces compass to be marked as externally mounted
     *                           (true) regardless of board definition, or allows
     *                           board hwdef to determine mounting (false)
     * @param[in] rotation Rotation/orientation of the sensor relative to vehicle
     *                     body frame (ROTATION_NONE, ROTATION_YAW_90, etc.)
     * 
     * @return Pointer to new AP_Compass_MMC3416 instance if probe succeeds,
     *         nullptr if device not detected or initialization fails
     * 
     * @note This method allocates memory on the heap using NEW_NOTHROW
     * @note Typically called during sensor auto-detection at boot
     * 
     * Source: libraries/AP_Compass/AP_Compass_MMC3416.cpp probe implementation
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Periodic read method to sample magnetic field data with SET/RESET cycles
     * 
     * @details This method is called periodically by the AP_Compass frontend to read
     *          magnetic field measurements. It implements a state machine that performs
     *          SET/RESET degaussing cycles to eliminate AMR sensor offset drift:
     * 
     *          State Machine Sequence:
     *          - STATE_REFILL1: Apply SET pulse (positive saturation)
     *          - STATE_REFILL1_WAIT: Wait for SET operation to complete
     *          - STATE_MEASURE_WAIT1: Trigger and read measurement after SET
     *          - STATE_REFILL2_WAIT: Apply RESET pulse (negative saturation)
     *          - STATE_MEASURE_WAIT2: Wait for RESET operation to complete
     *          - STATE_MEASURE_WAIT3: Trigger and read measurement after RESET
     * 
     *          The measurements taken after SET and RESET operations are compared
     *          to calculate and compensate for sensor offset drift. The corrected
     *          magnetic field vector is then published to the AP_Compass frontend.
     * 
     *          Field Measurements:
     *          - Raw data is 16-bit per axis with 0.0625 mG/LSB resolution
     *          - Converted to milligauss units
     *          - Rotation correction applied based on sensor mounting orientation
     *          - Offset compensation applied from SET/RESET difference
     * 
     * @note Called at compass update rate (typically 50-100 Hz)
     * @note Full SET/RESET cycle takes multiple read() calls to complete
     * @note Field measurements are accumulated and averaged during cycle
     * 
     * @warning Do not modify state machine timing without testing - SET/RESET
     *          operations require specific delays for magnetic saturation
     * 
     * Source: libraries/AP_Compass/AP_Compass_MMC3416.cpp read implementation
     */
    void read() override;

    static constexpr const char *name = "MMC3416";

private:
    /**
     * @brief Private constructor for MMC3416 driver instance
     * 
     * @details Constructs driver instance with I2C device ownership transfer.
     *          Only called from probe() factory method after successful detection.
     *          Initializes state machine and configuration parameters.
     * 
     * @param[in] dev I2C device with ownership transfer to this driver
     * @param[in] force_external Force external compass marking
     * @param[in] rotation Sensor mounting orientation
     */
    AP_Compass_MMC3416(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /// I2C device handle for sensor communication with exclusive ownership
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    /**
     * @brief State machine states for SET/RESET degaussing cycle
     * 
     * @details The state machine implements the periodic SET/RESET sequence
     *          required for AMR sensor offset compensation:
     * 
     *          - STATE_REFILL1: Apply SET pulse (positive saturation to sensor)
     *          - STATE_REFILL1_WAIT: Wait for SET magnetic field to stabilize
     *          - STATE_MEASURE_WAIT1: Trigger measurement and read data after SET
     *          - STATE_REFILL2_WAIT: Apply RESET pulse (negative saturation)
     *          - STATE_MEASURE_WAIT2: Wait for RESET magnetic field to stabilize
     *          - STATE_MEASURE_WAIT3: Trigger measurement and read data after RESET
     * 
     *          Each complete cycle through all states produces one offset-compensated
     *          magnetic field measurement by comparing SET and RESET readings.
     */
    enum {
        STATE_REFILL1,        ///< Apply SET degaussing pulse
        STATE_REFILL1_WAIT,   ///< Wait for SET operation completion
        STATE_MEASURE_WAIT1,  ///< Read measurement after SET
        STATE_REFILL2_WAIT,   ///< Apply RESET degaussing pulse
        STATE_MEASURE_WAIT2,  ///< Wait for RESET operation completion
        STATE_MEASURE_WAIT3,  ///< Read measurement after RESET
    } state;
    
    /**
     * @brief Initialize MMC3416 sensor registers and state machine
     * 
     * @details Performs sensor initialization sequence:
     *          1. Configure continuous measurement mode
     *          2. Set measurement rate and bandwidth
     *          3. Initialize SET/RESET control registers
     *          4. Register periodic timer callback for state machine
     * 
     * @return true if initialization successful, false on communication error
     * 
     * @note Called once during probe() after device detection
     */
    bool init();

    /**
     * @brief Periodic timer callback to execute state machine
     * 
     * @details Registered as HAL periodic callback, this method advances the
     *          SET/RESET state machine, triggers measurements, reads data from
     *          sensor registers, and publishes corrected field values to frontend.
     * 
     * @note Called at scheduled interval by HAL scheduler
     * @note This is the main device interaction point after initialization
     */
    void timer();

    /**
     * @brief Accumulate magnetic field measurement for averaging
     * 
     * @details Accumulates raw field measurements during SET/RESET cycle for
     *          averaging and noise reduction. Multiple samples are collected
     *          before computing final offset-compensated field vector.
     * 
     * @param[in,out] field Magnetic field vector in milligauss to accumulate
     * 
     * @note Field vector is in sensor frame before rotation correction
     */
    void accumulate_field(Vector3f &field);

    /// Compass instance ID registered with AP_Compass frontend
    uint8_t compass_instance;

    /// Force compass to be marked as externally mounted
    bool force_external;

    /// Calculated sensor offset from SET/RESET difference in milligauss
    Vector3f offset;

    /// Number of measurements accumulated in current cycle
    uint16_t measure_count;

    /// Flag indicating initial offset has been calculated from first SET/RESET cycle
    bool have_initial_offset;

    /// Timestamp when SET/RESET operation started (milliseconds)
    uint32_t refill_start_ms;

    /// Timestamp of last successful sample read (milliseconds)
    uint32_t last_sample_ms;
    
    /// Raw 16-bit sensor data buffer for X, Y, Z axes
    uint16_t data0[3];
    
    /// Sensor mounting rotation relative to vehicle body frame
    enum Rotation rotation;
};

#endif  // AP_COMPASS_MMC3416_ENABLED
