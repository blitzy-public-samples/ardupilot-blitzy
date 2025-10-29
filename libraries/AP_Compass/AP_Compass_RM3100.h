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
 * @file AP_Compass_RM3100.h
 * @brief Driver for PNI RM3100 high-precision 3-axis magnetometer
 * 
 * @details This driver supports the PNI Sensor Corporation RM3100 magnetometer,
 *          which uses proprietary MagI2C™ technology to achieve exceptional
 *          noise performance and measurement resolution compared to traditional
 *          Hall-effect or AMR magnetometers.
 *          
 *          Key Technical Specifications:
 *          - Measurement Range: ±800 µT (±8 Gauss) on all axes
 *          - Noise Floor: Sub-nanoTesla (typ. 13 nT RMS at 200 Hz sample rate)
 *          - Resolution: 16-bit ADC with configurable cycle counts for enhanced resolution
 *          - Field Units: milligauss (mGauss) as per ArduPilot compass convention
 *          - Interface: I2C (addresses 0x20-0x23) or SPI
 *          - Sample Rates: Up to 440 Hz in continuous measurement mode (CMM)
 *          
 *          MagI2C™ Technology:
 *          The RM3100 uses inductive sensing rather than solid-state Hall or AMR
 *          elements. This provides superior noise immunity, wider temperature range,
 *          and stable calibration over time without degaussing requirements.
 *          
 *          Applications:
 *          - Precision heading and navigation systems
 *          - High-accuracy compass applications requiring sub-degree heading precision
 *          - Environments with high magnetic interference requiring superior SNR
 *          - Survey-grade UAV magnetometry applications
 *          
 * @note The RM3100 typically draws more power (3-8 mA) than traditional magnetometers
 *       (1-2 mA) due to the active MagI2C™ coil excitation. Cycle count configuration
 *       trades power consumption for measurement resolution.
 *       
 * @warning The RM3100 is a premium sensor with significantly higher cost than
 *          traditional HMC5883L or QMC5883L magnetometers. Recommended for applications
 *          where heading accuracy is critical and cost is less constrained.
 *          
 * @see AP_Compass_Backend for base compass driver interface
 * @see AP_Compass for compass subsystem architecture
 * 
 * Source: libraries/AP_Compass/AP_Compass_RM3100.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_RM3100_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_RM3100_I2C_ADDR
# define HAL_COMPASS_RM3100_I2C_ADDR1 0x20  ///< Primary I2C address (ADDR0=GND, ADDR1=GND)
# define HAL_COMPASS_RM3100_I2C_ADDR2 0x21  ///< Secondary I2C address (ADDR0=VCC, ADDR1=GND)
# define HAL_COMPASS_RM3100_I2C_ADDR3 0x22  ///< Tertiary I2C address (ADDR0=GND, ADDR1=VCC)
# define HAL_COMPASS_RM3100_I2C_ADDR4 0x23  ///< Quaternary I2C address (ADDR0=VCC, ADDR1=VCC)
#endif

/**
 * @class AP_Compass_RM3100
 * @brief Backend driver for PNI RM3100 high-precision magnetometer
 * 
 * @details This class implements the ArduPilot compass backend interface for the
 *          RM3100 3-axis magnetometer. The driver configures the sensor in
 *          Continuous Measurement Mode (CMM) for high-rate periodic sampling
 *          and processes raw magnetic field measurements into calibrated heading
 *          information.
 *          
 *          Initialization Sequence:
 *          1. probe() - Factory method attempts device detection via REVID register
 *          2. init() - Configures cycle counts for desired resolution/power trade-off
 *          3. Enable CMM for autonomous measurement at configured sample rate
 *          4. Register timer() callback for periodic data reading
 *          
 *          Measurement Pipeline:
 *          1. timer() - Invoked at scheduler rate to read MEAS registers
 *          2. Extract 24-bit signed magnetic field values (X, Y, Z axes)
 *          3. Apply cycle-count-dependent scaling factor to convert to milligauss
 *          4. Publish calibrated field vector to compass library via publish_field()
 *          
 *          Cycle Count Configuration:
 *          The RM3100 allows configurable cycle counts (50-400) that determine
 *          measurement duration and resolution. Higher cycle counts provide:
 *          - Better resolution (lower noise floor)
 *          - Lower maximum sample rate (longer measurement time)
 *          - Higher power consumption (longer coil excitation)
 *          Default: 200 cycles provides balance of resolution and sample rate
 *          
 *          Dynamic Range and Resolution:
 *          - Full Scale: ±800 µT (±8000 mGauss)
 *          - Sensitivity: ~75 LSB/µT at 200 cycle count
 *          - Noise: ~13 nT RMS (0.13 mGauss RMS) at 200 cycles
 *          - Resolution: Sub-nanoTesla with proper averaging
 *          
 * @note The RM3100's superior noise performance makes it ideal for precision
 *       applications where heading accuracy <0.5° is required. The high dynamic
 *       range also allows operation in magnetically noisy environments without
 *       saturation.
 *       
 * @warning This driver assumes continuous measurement mode. Single-measurement
 *          polling mode is not implemented. Ensure timer() callback rate does not
 *          exceed sensor sample rate to avoid reading stale data.
 *          
 * @see AP_Compass_Backend for backend interface requirements
 * @see AP_Compass::register_compass() for compass instance registration
 * 
 * Source: libraries/AP_Compass/AP_Compass_RM3100.{h,cpp}
 */
class AP_Compass_RM3100 : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize RM3100 magnetometer
     * 
     * @details Attempts to probe for an RM3100 magnetometer on the provided device
     *          interface (I2C or SPI). Detection is performed by reading the REVID
     *          register (0x36) and verifying the expected revision ID value (0x22).
     *          
     *          If detection succeeds, allocates an AP_Compass_RM3100 instance,
     *          calls init() to configure the sensor, and returns the backend pointer
     *          for registration with the compass library.
     *          
     *          Probe Sequence:
     *          1. Read REVID register to verify device presence and identity
     *          2. If valid, allocate AP_Compass_RM3100 instance
     *          3. Call init() to configure cycle counts and enable CMM
     *          4. Register periodic timer() callback for data acquisition
     *          5. Return backend pointer on success, nullptr on failure
     * 
     * @param[in] dev              HAL device interface (I2C or SPI) for sensor communication
     * @param[in] force_external   Force compass to be marked as external regardless of board
     *                             configuration (true for external module, false for onboard)
     * @param[in] rotation         Rotation/orientation of sensor relative to vehicle frame
     *                             (ROTATION_NONE if aligned with vehicle axes)
     * 
     * @return Pointer to initialized AP_Compass_Backend instance on successful probe,
     *         nullptr if device not detected or initialization failed
     * 
     * @note This method transfers ownership of the device pointer. On failure, the
     *       device is automatically cleaned up when the OwnPtr goes out of scope.
     *       
     * @see init() for sensor configuration details
     * @see AP_Compass::_probe_external_i2c_compasses() for probe invocation
     * 
     * Source: libraries/AP_Compass/AP_Compass_RM3100.cpp::probe()
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read current magnetic field measurement from sensor
     * 
     * @details This method is called by the compass library's main update loop
     *          to retrieve the latest magnetic field measurement. For the RM3100,
     *          actual data acquisition occurs in the timer() callback which runs
     *          at high rate. This method performs minimal work, typically just
     *          returning immediately as data is already published by timer().
     *          
     *          In Continuous Measurement Mode (CMM), the sensor autonomously
     *          acquires measurements and updates the MEAS registers. The timer()
     *          callback reads these registers and publishes data via publish_field(),
     *          making this read() method essentially a no-op.
     * 
     * @note This is called at the main compass update rate (typically 10-50 Hz),
     *       which is much slower than the timer() callback rate (100-400 Hz).
     *       The high timer rate ensures fresh data is always available.
     *       
     * @see timer() for actual data acquisition implementation
     * @see AP_Compass::read() for compass library update sequence
     * 
     * Source: libraries/AP_Compass/AP_Compass_RM3100.cpp::read()
     */
    void read() override;

    static constexpr const char *name = "RM3100";  ///< Sensor identification string

private:
    /**
     * @brief Private constructor for RM3100 backend instance
     * 
     * @details Constructs the RM3100 compass backend with the provided device
     *          interface and configuration parameters. This constructor is private
     *          and only called by the probe() factory method after successful
     *          device detection.
     * 
     * @param[in] dev              HAL device interface for sensor communication
     * @param[in] force_external   Mark compass as external module
     * @param[in] rotation         Sensor orientation relative to vehicle frame
     * 
     * @note Constructor does not perform sensor initialization. The init() method
     *       must be called after construction to configure the sensor hardware.
     */
    AP_Compass_RM3100(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /**
     * @brief Initialize RM3100 sensor and configure for continuous operation
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Configure cycle counts (CCX, CCY, CCZ registers) for desired
     *             resolution/sample rate trade-off (default: 200 cycles)
     *          2. Calculate scaling factor to convert raw ADC counts to milligauss
     *             based on cycle count configuration
     *          3. Enable Continuous Measurement Mode (CMM) via CMM register
     *          4. Register timer() callback for periodic data acquisition
     *          5. Register compass instance with AP_Compass library
     *          
     *          Cycle Count Configuration:
     *          The cycle count determines measurement resolution and duration:
     *          - 50 cycles: Fastest (440 Hz max), lowest resolution
     *          - 200 cycles: Balanced (150 Hz max), good resolution (default)
     *          - 400 cycles: Slowest (75 Hz max), best resolution
     *          
     *          Scaling Factor Calculation:
     *          Raw 24-bit ADC values are converted to milligauss using:
     *          field_mGauss = (ADC_value / gain) * 10.0
     *          where gain depends on cycle count (~75 LSB/µT at 200 cycles)
     * 
     * @return true if initialization successful and sensor ready for operation,
     *         false if configuration failed (communication error, device not responding)
     * 
     * @note Called by probe() factory method. Failure here causes probe() to
     *       return nullptr and the device instance to be discarded.
     *       
     * @warning Cycle count must be set before enabling CMM, as changing cycle
     *          counts during active measurement can cause incorrect scaling.
     * 
     * @see probe() for initialization invocation
     * @see timer() for data acquisition after initialization
     * 
     * Source: libraries/AP_Compass/AP_Compass_RM3100.cpp::init()
     */
    bool init();
    
    /**
     * @brief Periodic callback to read magnetic field data from sensor
     * 
     * @details This method is invoked by the AP_HAL scheduler at high rate
     *          (typically 100-400 Hz depending on cycle count configuration) to
     *          read fresh magnetic field measurements from the RM3100.
     *          
     *          Data Acquisition Sequence:
     *          1. Check DRDY (Data Ready) status bit to ensure new data available
     *          2. Read 9 bytes from MX, MY, MZ measurement registers (24-bit each)
     *          3. Combine bytes into signed 24-bit integer field values
     *          4. Apply scaling factor to convert from ADC counts to milligauss
     *          5. Apply sensor rotation/orientation transformation
     *          6. Publish calibrated field vector via publish_field()
     *          
     *          In CMM (Continuous Measurement Mode), the sensor autonomously
     *          performs measurements and updates the MEAS registers. This callback
     *          simply reads the latest values at each invocation.
     *          
     *          Error Handling:
     *          - If DRDY not set, skip reading (no new data available)
     *          - If I2C/SPI read fails, skip update (communication error)
     *          - If field magnitude exceeds reasonable bounds, log error
     * 
     * @note Invocation rate should match or slightly exceed sensor sample rate
     *       to ensure fresh data is always read. Timer rate is configured in
     *       init() via register_periodic_callback().
     *       
     * @warning Do not read faster than sensor sample rate, as this wastes CPU
     *          cycles reading duplicate data. Do not read slower than sample rate,
     *          as this causes MEAS register overrun and data loss.
     * 
     * @see init() for timer callback registration
     * @see AP_Compass_Backend::publish_field() for data publishing
     * 
     * Source: libraries/AP_Compass/AP_Compass_RM3100.cpp::timer()
     */
    void timer();

    AP_HAL::OwnPtr<AP_HAL::Device> dev;  ///< HAL device interface for I2C/SPI communication with sensor
    
    uint8_t compass_instance;  ///< Compass instance ID assigned by AP_Compass library (0-2 typically)
    bool force_external;       ///< True if compass forced to external (affects priority and auto-selection)
    enum Rotation rotation;    ///< Sensor orientation relative to vehicle frame (ROTATION_* enum)
    
    /**
     * @brief Scaling factor to convert raw ADC counts to milligauss
     * 
     * @details This factor converts the 24-bit signed ADC values from the RM3100
     *          into calibrated magnetic field measurements in milligauss units.
     *          
     *          The scaling factor depends on the configured cycle count:
     *          - Higher cycle counts provide better resolution (larger gain)
     *          - Typical value at 200 cycles: ~1.0 / 75 = 0.0133
     *          
     *          Conversion formula:
     *          field_mGauss = ADC_raw_counts * _scaler * 10.0
     *          
     *          The factor of 10.0 converts from µT to mGauss (1 µT = 10 mGauss)
     * 
     * @note Calculated during init() based on configured cycle count and stored
     *       for efficient runtime conversion in timer() callback.
     */
    float _scaler = 1.0;
};

#endif  // AP_COMPASS_RM3100_ENABLED
