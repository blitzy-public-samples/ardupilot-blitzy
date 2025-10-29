/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
 *
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
 * @file AP_Compass_IST8310.h
 * @brief Driver for iSentek IST8310 3-axis magnetometer
 * 
 * This driver supports the IST8310 digital 3-axis magnetic sensor from iSentek.
 * The IST8310 is commonly used in Pixhawk-family flight controllers and other
 * autopilot hardware for compass functionality.
 * 
 * Hardware Specifications:
 * - Magnetic field range: ±1600 µT (±16 Gauss)
 * - Resolution: 0.3 µT (3 milligauss)
 * - Output data rate: Up to 200 Hz
 * - Interface: I2C (default address 0x0E)
 * - Operating modes: Single measurement and continuous measurement
 * 
 * Driver Architecture:
 * - Follows standard ArduPilot compass driver pattern: probe → init → periodic read
 * - Uses timer-based periodic sampling via HAL scheduler
 * - Reports measurements in milligauss units
 * 
 * @note This driver is commonly used on internal compass modules in flight controllers
 *       and external compass/GPS combination units
 * 
 * Source: libraries/AP_Compass/AP_Compass_IST8310.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_IST8310_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_IST8310_I2C_ADDR
#define HAL_COMPASS_IST8310_I2C_ADDR 0x0E
#endif

#ifndef AP_COMPASS_IST8310_DEFAULT_ROTATION
#define AP_COMPASS_IST8310_DEFAULT_ROTATION ROTATION_PITCH_180
#endif

/**
 * @class AP_Compass_IST8310
 * @brief Backend driver for iSentek IST8310 3-axis magnetometer
 * 
 * @details This class implements the compass backend interface for the IST8310
 *          magnetometer, providing magnetic field measurements for vehicle heading
 *          estimation and navigation.
 * 
 * Driver Lifecycle:
 * 1. **Probe**: Static probe() method attempts I2C communication and device ID verification
 * 2. **Initialization**: Private init() configures sensor registers and operating mode
 * 3. **Periodic Reading**: Timer callback reads magnetic field data at regular intervals
 * 4. **Data Processing**: Raw sensor data converted to milligauss and published to frontend
 * 
 * Operating Modes:
 * - **Single Measurement Mode**: Triggers one measurement, then returns to standby
 * - **Continuous Measurement Mode**: Automatically repeats measurements at configured rate
 * 
 * Data Format:
 * - Magnetic field measurements reported in milligauss (mG)
 * - Three-axis data: X, Y, Z in sensor coordinate frame
 * - Coordinate frame rotation applied based on physical mounting orientation
 * 
 * Hardware Integration:
 * - Connected via I2C bus (default address 0x0E)
 * - Can be configured as internal or external compass
 * - Default rotation ROTATION_PITCH_180 for common mounting orientations
 * 
 * @note The IST8310 has a measurement range of ±1600 µT (±16000 milligauss) which
 *       is suitable for Earth's magnetic field (25-65 µT) with margin for local
 *       magnetic disturbances
 * 
 * @warning Periodic timer callback runs at high frequency - keep processing efficient
 *          to avoid scheduler overruns
 * 
 * @see AP_Compass_Backend
 * @see AP_Compass
 */
class AP_Compass_IST8310 : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe for IST8310 magnetometer on I2C bus
     * 
     * @details Attempts to detect and initialize an IST8310 magnetometer on the
     *          provided I2C device. This method performs device detection by reading
     *          the WHO_AM_I register and verifying the device ID. If successful,
     *          creates a new driver instance and completes initialization.
     * 
     * Probe Sequence:
     * 1. Attempt communication with device at I2C address
     * 2. Read and verify WHO_AM_I register (expected value 0x10)
     * 3. Create driver instance if device detected
     * 4. Configure sensor registers and operating mode
     * 5. Register periodic timer callback for data acquisition
     * 
     * @param[in] dev I2C device handle with configured bus and address
     * @param[in] force_external If true, register compass as external (not affected
     *                           by COMPASS_EXTERN parameter). If false, use auto-detection
     *                           based on bus location.
     * @param[in] rotation Physical mounting orientation of the sensor relative to vehicle
     *                     body frame (e.g., ROTATION_PITCH_180 for upside-down mounting)
     * 
     * @return Pointer to newly created AP_Compass_Backend instance on success,
     *         nullptr if device not detected or initialization failed
     * 
     * @note This is called during compass detection phase at startup. Multiple buses
     *       may be probed to find all connected magnetometers.
     * @note Ownership of the I2C device handle is transferred to the driver instance
     *       on successful probe
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read magnetic field data and publish to compass frontend
     * 
     * @details This method is called periodically by the main thread to retrieve
     *          accumulated magnetic field measurements from the sensor. The actual
     *          I2C communication and data sampling occurs in the timer() callback
     *          running on the scheduler thread.
     * 
     *          This two-stage approach (timer callback + read method) ensures I2C
     *          transactions don't block the main thread while allowing controlled
     *          publication of data to the compass frontend at the main loop rate.
     * 
     * Data Flow:
     * 1. timer() callback reads raw sensor data via I2C (runs on scheduler thread)
     * 2. Raw data buffered in driver instance
     * 3. read() retrieves buffered data on main thread
     * 4. Data converted from sensor units to milligauss
     * 5. Rotation applied based on physical mounting
     * 6. Published to compass frontend via publish_raw_field()
     * 
     * @note Called at main loop rate (typically 400 Hz for copter, 50-100 Hz for plane)
     * @note Returns immediately if no new data available from timer callback
     */
    void read() override;

    static constexpr const char *name = "IST8310";

private:
    /**
     * @brief Private constructor for IST8310 driver instance
     * 
     * @details Creates a new driver instance after successful probe. Constructor
     *          is private to enforce creation only through probe() method which
     *          verifies device presence before instantiation.
     * 
     * @param[in] dev I2C or SPI device handle (ownership transferred to driver)
     * @param[in] force_external Force registration as external compass
     * @param[in] rotation Physical mounting orientation
     * 
     * @note Constructor does not perform hardware initialization - init() must be
     *       called after construction
     */
    AP_Compass_IST8310(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /**
     * @brief Periodic timer callback for sensor data acquisition
     * 
     * @details This method runs on the HAL scheduler thread at regular intervals
     *          (typically 100 Hz) to read magnetic field data from the IST8310 via I2C.
     *          Running on a separate thread prevents I2C transactions from blocking
     *          the time-critical main thread.
     * 
     * Timer Callback Sequence:
     * 1. Check if sensor is ready (data ready bit or timing)
     * 2. Read 6 bytes of magnetic field data (X, Y, Z axes - 2 bytes each)
     * 3. Convert from raw ADC values to magnetic field strength
     * 4. Buffer data for retrieval by read() method on main thread
     * 5. Trigger next conversion if in single-measurement mode
     * 
     * @note Runs on scheduler thread - keep processing minimal and fast
     * @note I2C read failures are logged but don't stop periodic operation
     * 
     * @warning Do not call frontend methods directly from timer callback - buffer
     *          data for main thread to avoid race conditions
     */
    void timer();

    /**
     * @brief Initialize IST8310 sensor registers and configuration
     * 
     * @details Configures the IST8310 for operation by writing control registers
     *          and setting up the measurement mode. Called once during probe after
     *          device detection is confirmed.
     * 
     * Initialization Sequence:
     * 1. Soft reset sensor to known state
     * 2. Configure averaging and output data rate
     * 3. Set measurement mode (single or continuous)
     * 4. Configure interrupt/data ready settings
     * 5. Register compass instance with frontend
     * 6. Register periodic timer callback for data acquisition
     * 
     * Configuration Details:
     * - Measurement mode: Typically single-measurement with manual triggering
     * - Averaging: Configurable sample averaging for noise reduction
     * - Data rate: Up to 200 Hz depending on mode
     * 
     * @return true if initialization successful, false on I2C communication failure
     *         or unexpected register values
     * 
     * @note Initialization failure causes probe() to return nullptr, preventing
     *       driver instantiation
     */
    bool init();

    /**
     * @brief Trigger a new magnetic field measurement
     * 
     * @details Initiates a single measurement cycle on the IST8310. Used in
     *          single-measurement mode where each reading must be explicitly
     *          triggered. Writes to the sensor control register to start conversion.
     * 
     * Single-Measurement Mode Operation:
     * 1. Sensor idles in low-power standby state
     * 2. start_conversion() triggers one measurement
     * 3. Sensor performs measurement (typically 1-10 ms depending on averaging)
     * 4. Data ready for reading via timer() callback
     * 5. Sensor returns to standby, awaiting next trigger
     * 
     * @note In continuous measurement mode, this method may not be used as sensor
     *       automatically repeats measurements at configured rate
     * @note I2C write failures are handled gracefully - next timer() call will retry
     */
    void start_conversion();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;  ///< I2C device handle for sensor communication
    AP_HAL::Device::PeriodicHandle _periodic_handle;  ///< Handle for periodic timer callback

    enum Rotation _rotation;  ///< Physical mounting orientation relative to vehicle body frame
    uint8_t _instance;  ///< Compass instance ID assigned by frontend (used for multi-compass systems)
    bool _ignore_next_sample;  ///< Flag to skip next reading (e.g., after configuration change)
    bool _force_external;  ///< Force compass to be treated as external regardless of bus location
};

#endif  // AP_COMPASS_IST8310_ENABLED
