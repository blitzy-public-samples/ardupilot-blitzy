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
 * @file AP_Compass_MMC5xx3.h
 * @brief Driver for MEMSIC MMC5883/MMC5983 family magnetoresistive compass sensors
 * 
 * @details This driver supports the MEMSIC MMC5xx3 series of 3-axis magnetic sensors
 *          using advanced magnetoresistive (AMR) technology. The MMC5xx3 family includes
 *          the MMC5883 and MMC5983 sensors, which offer improved stability and lower
 *          drift compared to the earlier MMC3416 sensor.
 *          
 *          Key features:
 *          - Magnetoresistive (AMR) sensing technology with improved stability
 *          - SET/RESET degaussing pulses for offset cancellation
 *          - Field range: ±8 gauss typical (sufficient for Earth's magnetic field)
 *          - Resolution: 18-bit (MMC5883) or 20-bit (MMC5983) ADC
 *          - On-chip temperature compensation
 *          - Continuous measurement mode support
 *          - I2C interface (default address 0x30)
 *          
 *          The driver implements automatic offset cancellation using the sensor's
 *          built-in SET/RESET functionality, which applies degaussing pulses to
 *          eliminate residual magnetization and improve measurement accuracy.
 *          
 *          Field measurements are reported in milligauss units.
 * 
 * @note SET/RESET cycles are performed periodically to maintain calibration
 * @note Lower drift and better stability than MMC3416 predecessor
 * 
 * Source: libraries/AP_Compass/AP_Compass_MMC5xx3.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MMC5XX3_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_MMC5xx3_I2C_ADDR
# define HAL_COMPASS_MMC5xx3_I2C_ADDR 0x30
#endif

/**
 * @class AP_Compass_MMC5XX3
 * @brief Backend driver for MEMSIC MMC5883/MMC5983 magnetoresistive compass sensors
 * 
 * @details This backend implements support for the MEMSIC MMC5xx3 family of 3-axis
 *          magnetometers, which use advanced anisotropic magnetoresistive (AMR)
 *          technology for improved measurement stability and reduced drift.
 *          
 *          The MMC5xx3 sensors feature built-in SET/RESET functionality that applies
 *          degaussing pulses to the magnetoresistive sensing elements. This eliminates
 *          residual magnetization and cancels temperature-induced offsets, providing
 *          superior stability compared to earlier sensors like the MMC3416.
 *          
 *          Technical specifications:
 *          - Sensing technology: Anisotropic magnetoresistive (AMR)
 *          - Field range: ±8 gauss typical (16 gauss full scale)
 *          - Resolution: 18-bit (MMC5883) to 20-bit (MMC5983)
 *          - Field units: milligauss (mGa)
 *          - Noise: ~0.4 mGa RMS at 100 Hz ODR
 *          - Temperature compensation: On-chip
 *          - Operating modes: Single measurement, continuous mode
 *          
 *          SET/RESET Operation:
 *          The driver implements a state machine that periodically performs SET and
 *          RESET operations. During SET, a positive degaussing pulse is applied to
 *          the sensor's coil, followed by a field measurement. During RESET, a
 *          negative pulse is applied, followed by another measurement. The difference
 *          between SET and RESET measurements cancels temperature drift and other
 *          systematic offsets, significantly improving long-term stability.
 *          
 *          Measurement cycle sequence:
 *          1. STATE_SET - Apply SET pulse (positive degaussing)
 *          2. STATE_SET_MEASURE - Initiate measurement after SET
 *          3. STATE_SET_WAIT - Wait for measurement completion
 *          4. STATE_RESET_MEASURE - Apply RESET pulse and measure
 *          5. STATE_RESET_WAIT - Wait for RESET measurement
 *          6. STATE_MEASURE - Normal continuous measurements
 *          
 *          The driver automatically averages SET and RESET measurements to produce
 *          the final calibrated field vector, which is then rotated according to
 *          the configured board orientation and published to the AP_Compass frontend.
 * 
 * @note Improved stability over MMC3416: lower drift, better temperature performance
 * @note SET/RESET cycles run automatically to maintain calibration accuracy
 * @warning Ensure proper I2C bus speed and pull-up resistors for reliable operation
 */
class AP_Compass_MMC5XX3 : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe and initialize an MMC5xx3 magnetometer sensor
     * 
     * @details Attempts to detect and initialize an MMC5883 or MMC5983 sensor
     *          on the provided device interface. The probe sequence includes:
     *          - Reading and verifying the device Product ID register
     *          - Performing initial SET/RESET calibration cycle
     *          - Configuring continuous measurement mode
     *          - Setting up automatic SET/RESET intervals
     *          - Registering the sensor with the AP_Compass frontend
     *          
     *          If detection succeeds, this method creates and returns a new
     *          AP_Compass_MMC5XX3 backend instance. If detection fails (wrong
     *          Product ID, communication error, or initialization failure),
     *          returns nullptr and releases the device.
     * 
     * @param[in] dev Device interface (I2C) for communicating with the sensor
     * @param[in] force_external true to force sensor to be treated as external
     *                           (not affected by COMPASS_ORIENT), false to allow
     *                           automatic internal/external detection
     * @param[in] rotation Board rotation to apply to magnetic field measurements,
     *                     specified as enum Rotation (ROTATION_NONE, ROTATION_YAW_90,
     *                     etc.) to transform sensor frame to vehicle body frame
     * 
     * @return Pointer to new AP_Compass_MMC5XX3 backend if probe succeeds,
     *         nullptr if sensor not detected or initialization fails
     * 
     * @note Device ownership is transferred to the backend if probe succeeds
     * @note Probe may take several milliseconds due to SET/RESET calibration
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read and publish magnetic field measurements from the sensor
     * 
     * @details This method is called by the AP_Compass frontend to retrieve
     *          the latest magnetic field data. The actual sensor reading is
     *          performed asynchronously in the timer() callback at the configured
     *          sample rate. This method retrieves the accumulated field vector,
     *          applies board rotation, and publishes it to the compass frontend.
     *          
     *          The driver implements automatic SET/RESET offset cancellation:
     *          - Periodically applies SET pulse (positive degaussing)
     *          - Measures field after SET operation
     *          - Applies RESET pulse (negative degaussing)
     *          - Measures field after RESET operation
     *          - Averages SET and RESET measurements to cancel offsets
     *          
     *          This SET/RESET cycle runs every N measurements (configurable)
     *          to maintain calibration accuracy and compensate for temperature
     *          drift. Between SET/RESET cycles, the sensor runs in continuous
     *          measurement mode for maximum sampling efficiency.
     *          
     *          Field data is reported in milligauss units and includes all
     *          three magnetic field components (X, Y, Z) in the sensor frame,
     *          which are then transformed to vehicle body frame using the
     *          configured rotation.
     * 
     * @note Called by AP_Compass::read_sensors() at main loop rate
     * @note Actual sensor sampling occurs in timer() at higher rate
     * @note SET/RESET cycles provide superior offset stability vs single measurements
     */
    void read() override;

    /// Sensor name identifier for MMC5983 (also compatible with MMC5883)
    static constexpr const char *name = "MMC5983";

private:
    /**
     * @brief Private constructor for MMC5xx3 backend (use probe() to create instances)
     * 
     * @param[in] dev Device interface (I2C) for sensor communication
     * @param[in] force_external true to force external compass designation
     * @param[in] rotation Board rotation for field vector transformation
     */
    AP_Compass_MMC5XX3(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /// Device interface for I2C communication with the sensor
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    /**
     * @enum MMCState
     * @brief State machine for SET/RESET offset cancellation cycle
     * 
     * @details The MMC5xx3 sensors use SET/RESET degaussing pulses to eliminate
     *          residual magnetization and cancel temperature-induced offsets.
     *          This state machine coordinates the measurement sequence:
     *          
     *          SET Phase:
     *          - Apply positive degaussing pulse to sensor coil
     *          - Measure magnetic field with positive magnetization
     *          
     *          RESET Phase:
     *          - Apply negative degaussing pulse to sensor coil
     *          - Measure magnetic field with negative magnetization
     *          
     *          The average of SET and RESET measurements cancels systematic
     *          offsets, providing improved long-term stability.
     */
    enum class MMCState {
        STATE_SET,            ///< Apply SET pulse (positive degaussing)
        STATE_SET_MEASURE,    ///< Initiate measurement after SET pulse
        STATE_SET_WAIT,       ///< Wait for SET measurement to complete
        STATE_RESET_MEASURE,  ///< Apply RESET pulse and initiate measurement
        STATE_RESET_WAIT,     ///< Wait for RESET measurement to complete
        STATE_MEASURE,        ///< Normal continuous measurement mode (between SET/RESET cycles)
    } state;
    
    /**
     * @brief Initialize the MMC5xx3 sensor and configure measurement mode
     * 
     * @details Performs sensor initialization sequence:
     *          - Verify device Product ID (0x30 for MMC5983, 0x0C for MMC5883)
     *          - Configure continuous measurement mode with ODR
     *          - Enable automatic SET/RESET if supported
     *          - Set bandwidth and sample rate
     *          - Configure temperature compensation
     *          - Register periodic timer callback for data acquisition
     * 
     * @return true if initialization successful, false on communication error
     *         or unsupported device ID
     * 
     * @note Called during probe() before backend registration
     */
    bool init();
    
    /**
     * @brief Periodic timer callback to read sensor data and manage SET/RESET cycles
     * 
     * @details Device periodic callback executed at sensor sample rate (typically 100-200 Hz).
     *          Implements the SET/RESET state machine for offset cancellation:
     *          
     *          - STATE_SET: Apply positive degaussing pulse via SET command
     *          - STATE_SET_MEASURE: Start measurement after SET pulse settling
     *          - STATE_SET_WAIT: Read SET measurement data when ready
     *          - STATE_RESET_MEASURE: Apply negative degaussing and measure
     *          - STATE_RESET_WAIT: Read RESET measurement data
     *          - STATE_MEASURE: Normal measurements between SET/RESET cycles
     *          
     *          Field data is accumulated and averaged to reduce noise. SET and RESET
     *          measurements are automatically differenced to cancel temperature drift.
     * 
     * @note Called automatically by HAL scheduler at configured sample rate
     * @note SET/RESET cycle runs every ~1000 measurements for optimal stability
     */
    void timer();
    
    /**
     * @brief Accumulate raw magnetic field measurement into running average
     * 
     * @details Adds a new field sample to the accumulated field vector for averaging.
     *          Multiple samples are accumulated between read() calls to reduce noise
     *          and improve measurement precision. The accumulation accounts for the
     *          current SET/RESET state to properly combine offset-cancelled measurements.
     * 
     * @param[in,out] field Magnetic field vector in milligauss to add to accumulator
     *                      (modified to apply running average and offset correction)
     * 
     * @note Field is in sensor frame; rotation applied later in read()
     */
    void accumulate_field(Vector3f &field);

    /// AP_Compass instance ID for this magnetometer
    uint8_t compass_instance;
    
    /// Whether sensor should be treated as external compass
    bool force_external;
    
    /// Measured offset vector from SET/RESET averaging (milligauss)
    Vector3f offset;
    
    /// Count of measurements since last SET/RESET cycle (used to trigger recalibration)
    uint16_t measure_count;
    
    /// Flag indicating whether initial SET/RESET offset has been calculated
    bool have_initial_offset;
    
    /// Timestamp when sensor degaussing refill started (milliseconds)
    uint32_t refill_start_ms;
    
    /// Timestamp of last successful sample (milliseconds, for health monitoring)
    uint32_t last_sample_ms;
    
    /// Buffer for raw 6-byte magnetic field data (X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB)
    uint8_t data0[6];
    
    /// Board rotation to transform sensor frame to vehicle body frame
    enum Rotation rotation;
};

#endif  // AP_COMPASS_MMC5XX3_ENABLED
