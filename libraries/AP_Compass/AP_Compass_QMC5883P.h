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
 *
 * Driver by Lokesh Ramina, Jan 2022
 */

/**
 * @file AP_Compass_QMC5883P.h
 * @brief QST QMC5883P 3-axis magnetometer driver
 * 
 * @details Driver for the QST QMC5883P digital 3-axis magnetic sensor, an improved
 *          variant of the QMC5883L with enhanced performance characteristics including
 *          lower noise, better temperature stability, and improved linearity.
 *          
 *          The QMC5883P is a high-sensitivity magnetic sensor with I2C interface,
 *          providing 16-bit magnetic field measurements in all three axes.
 *          
 *          Key improvements over QMC5883L:
 *          - Lower RMS noise (< 2 milligauss)
 *          - Improved temperature coefficient
 *          - Better cross-axis sensitivity
 *          - Enhanced field range linearity
 *          - More stable calibration over time
 *          
 *          Magnetic field measurements are provided in milligauss units.
 *          
 * @note This driver follows the standard ArduPilot magnetometer backend lifecycle:
 *       probe() → init() → register_periodic_callback() → timer() → read()
 * 
 * Source: libraries/AP_Compass/AP_Compass_QMC5883P.h
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass_config.h"

#ifdef AP_COMPASS_QMC5883P_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_QMC5883P_I2C_ADDR
#define HAL_COMPASS_QMC5883P_I2C_ADDR 0x2C
#endif

/*
  setup default orientations
 */
#ifndef HAL_COMPASS_QMC5883P_ORIENTATION_EXTERNAL
#define HAL_COMPASS_QMC5883P_ORIENTATION_EXTERNAL ROTATION_ROLL_180
#endif

#ifndef HAL_COMPASS_QMC5883P_ORIENTATION_INTERNAL
#define HAL_COMPASS_QMC5883P_ORIENTATION_INTERNAL ROTATION_ROLL_180_YAW_270
#endif

/**
 * @class AP_Compass_QMC5883P
 * @brief Backend driver for QST QMC5883P 3-axis magnetometer
 * 
 * @details This class implements the ArduPilot compass backend interface for the
 *          QMC5883P magnetometer, an improved variant of the QMC5883L sensor.
 *          
 *          Driver Lifecycle:
 *          1. probe() - Static factory method that attempts I2C communication and
 *             verifies device identity via WHO_AM_I register
 *          2. init() - Configures sensor registers for continuous measurement mode,
 *             sets output data rate, and registers the compass instance
 *          3. register_periodic_callback() - Schedules timer() to run periodically
 *             at sensor sampling rate
 *          4. timer() - Reads raw magnetic field data from device registers
 *          5. read() - Processes accumulated samples and publishes to AP_Compass
 *          
 *          Sensor Configuration:
 *          - I2C Address: 0x2C (default, configurable via HAL_COMPASS_QMC5883P_I2C_ADDR)
 *          - Measurement Range: ±30 gauss (±3000 milligauss)
 *          - Resolution: 16-bit per axis
 *          - Output Data Rate: Configurable (typically 200 Hz)
 *          - Measurement Mode: Continuous measurement
 *          
 *          Coordinate Frame:
 *          - Raw sensor data is in sensor body frame
 *          - Rotation applied via _rotation parameter (board-specific)
 *          - Default external orientation: ROTATION_ROLL_180
 *          - Default internal orientation: ROTATION_ROLL_180_YAW_270
 *          
 *          Improvements over QMC5883L:
 *          - Lower noise floor enabling better heading accuracy
 *          - Reduced temperature drift for stable outdoor operation
 *          - Improved cross-axis sensitivity for accurate 3D measurements
 *          - Better long-term stability reducing recalibration needs
 *          
 * @note All magnetic field measurements are in milligauss units
 * @note This driver requires I2C bus access and is typically probed during
 *       system startup by the compass library
 * 
 * @warning Magnetometers require careful calibration and should be mounted
 *          away from magnetic interference sources (power lines, motors, ferrous materials)
 */
class AP_Compass_QMC5883P : public AP_Compass_Backend
{
public:
    /**
     * @brief Probe for QMC5883P magnetometer on I2C bus
     * 
     * @details Static factory method that attempts to detect and initialize a
     *          QMC5883P magnetometer on the provided I2C device. This method:
     *          1. Takes ownership of the I2C device
     *          2. Attempts communication at device address 0x2C
     *          3. Reads and verifies WHO_AM_I register (chip ID)
     *          4. If successful, constructs and initializes driver instance
     *          5. Returns nullptr if device not found or communication fails
     *          
     *          This function is typically called during system startup by the
     *          compass library's bus scanning procedure.
     * 
     * @param[in] dev I2C device handle with ownership transfer - if probe fails,
     *                device handle is released. If successful, driver takes ownership.
     * @param[in] force_external If true, treat compass as externally mounted regardless
     *                           of detection method. External compasses use different
     *                           default orientation and priority. Set true for compasses
     *                           on GPS modules or external I2C buses.
     * @param[in] rotation Board-specific rotation to transform sensor frame to vehicle
     *                     frame. Use ROTATION_NONE if orientation matches vehicle axes,
     *                     or specify required rotation (e.g., ROTATION_ROLL_180_YAW_90).
     *                     Overrides default orientation constants if not ROTATION_NONE.
     * 
     * @return Pointer to initialized AP_Compass_QMC5883P backend if probe successful,
     *         nullptr if device not detected or initialization fails
     * 
     * @note Probe failure is normal - compass library probes multiple possible devices
     * @note This method transfers ownership of the I2C device to the created backend
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read and publish accumulated magnetometer samples
     * 
     * @details This method is called by the main compass library to retrieve the
     *          latest magnetic field measurements. It processes samples accumulated
     *          by the timer() callback, applies calibration offsets and scaling,
     *          and publishes the corrected magnetic field vector to the AP_Compass
     *          frontend for use by the AHRS/EKF systems.
     *          
     *          The magnetic field vector is published in the vehicle body frame
     *          (after applying the configured rotation) in milligauss units.
     *          
     *          This method is typically called at the main loop rate (e.g., 50-400 Hz)
     *          while the actual sensor sampling occurs in timer() at the configured
     *          sensor output data rate (typically 200 Hz).
     * 
     * @note Implements pure virtual method from AP_Compass_Backend
     * @note All measurements are in milligauss units
     * @note Called from main thread context, not interrupt context
     */
    void read() override;

    /**
     * @brief Driver identification string for logging and diagnostics
     * 
     * @details This name appears in system logs, parameter descriptions, and
     *          diagnostic outputs to identify the magnetometer type. It allows
     *          operators to verify which compass hardware is detected and active.
     */
    static constexpr const char *name = "QMC5883P";

private:
    /**
     * @brief Private constructor called by probe() on successful device detection
     * 
     * @param[in] dev I2C device handle with ownership transferred from probe()
     * @param[in] force_external External mounting flag passed from probe()
     * @param[in] rotation Board-specific rotation passed from probe()
     * 
     * @note Constructor is private - instances created only via probe() factory method
     */
    AP_Compass_QMC5883P(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                        bool force_external,
                        enum Rotation rotation);

    /**
     * @brief Dump all sensor registers to console for debugging
     * 
     * @details Reads and prints all QMC5883P configuration and status registers
     *          to aid in hardware debugging and sensor configuration verification.
     *          Used during development and troubleshooting.
     */
    void _dump_registers();
    
    /**
     * @brief Verify device identity via WHO_AM_I register
     * 
     * @details Reads the chip identification register and verifies it matches
     *          the expected QMC5883P device ID. This confirms correct device
     *          detection and I2C communication.
     * 
     * @return true if WHO_AM_I register contains expected QMC5883P ID, false otherwise
     * 
     * @note Called during probe() to validate device identity before initialization
     */
    bool _check_whoami();
    
    /**
     * @brief Periodic sampling callback executed at sensor data rate
     * 
     * @details This method is registered as a periodic callback with the I2C device
     *          and executes at the configured sensor output data rate (typically 200 Hz).
     *          It reads the latest magnetic field measurements from the sensor data
     *          registers and accumulates them for processing by read().
     *          
     *          Execution sequence:
     *          1. Check data ready status bit
     *          2. Read 6 bytes (X, Y, Z axes, 16-bit each) from data registers
     *          3. Accumulate raw samples for averaging
     *          4. Handle I2C communication errors
     * 
     * @note Executes in I2C bus thread context, not main thread
     * @note Must complete quickly to avoid blocking other I2C devices
     * @note Measurements read here are in sensor body frame, raw milligauss units
     */
    void timer();
    
    /**
     * @brief Initialize sensor configuration and register compass instance
     * 
     * @details Configures the QMC5883P sensor registers for continuous measurement
     *          mode and registers this compass instance with the AP_Compass frontend.
     *          
     *          Initialization sequence:
     *          1. Verify WHO_AM_I register (_check_whoami)
     *          2. Issue software reset
     *          3. Configure control registers:
     *             - Set measurement mode to continuous
     *             - Configure output data rate (ODR)
     *             - Set measurement range (±30 gauss)
     *             - Enable/configure oversampling
     *          4. Register compass instance with AP_Compass
     *          5. Set compass orientation rotation
     *          6. Register timer() as periodic callback
     * 
     * @return true if initialization successful and sensor operational, false on failure
     * 
     * @note Called by probe() after constructor
     * @note Returns false if WHO_AM_I check fails or I2C communication error
     */
    bool init();

    /**
     * @brief I2C device handle with exclusive ownership
     * 
     * @details Provides access to the I2C bus for sensor communication. This
     *          smart pointer manages device lifetime and ensures exclusive access.
     *          All register reads/writes go through this device interface.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief Rotation transformation from sensor frame to vehicle frame
     * 
     * @details Defines the rotation matrix to apply to raw sensor measurements
     *          to transform from sensor body frame to vehicle body frame. This
     *          accounts for the physical mounting orientation of the compass.
     *          Set during construction from board-specific defaults or probe parameter.
     */
    enum Rotation _rotation;
    
    /**
     * @brief Compass instance ID registered with AP_Compass frontend
     * 
     * @details Unique identifier for this compass instance used by the AP_Compass
     *          library to track multiple compass sensors. Assigned during init()
     *          when registering with the frontend. Used when publishing measurements.
     */
    uint8_t _instance;
    
    /**
     * @brief External mounting flag (1-bit bitfield for memory efficiency)
     * 
     * @details If true, compass is treated as externally mounted (e.g., on GPS mast
     *          or external I2C bus) which affects priority, default orientation, and
     *          compass selection logic. External compasses typically have higher
     *          priority as they experience less magnetic interference from vehicle
     *          power systems and motors.
     */
    bool _force_external:1;
};

#endif  // AP_COMPASS_QMC5883P_ENABLED
