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
 * @file AP_Compass_BMM350.h
 * @brief Driver for Bosch BMM350 3-axis magnetometer
 * 
 * @details The BMM350 is Bosch's next-generation magnetometer, succeeding the BMM150.
 *          It features improved sensitivity, lower power consumption, and enhanced
 *          temperature stability compared to its predecessor.
 * 
 *          Key specifications:
 *          - Measurement range: ±1300 µT (micro-Tesla) for X/Y axes, ±2500 µT for Z axis
 *          - Sensitivity: 0.3 µT/LSB typical (better than BMM150's 0.6 µT/LSB)
 *          - Output data rate: Up to 400 Hz
 *          - I2C interface with configurable addresses (0x14-0x17)
 *          - Temperature sensor integrated for compensation
 * 
 *          The driver implements:
 *          - Factory OTP (One-Time Programmable) calibration data loading
 *          - Cross-axis compensation for magnetic field interactions
 *          - Temperature compensation for drift reduction
 *          - Multiple power modes (suspend, normal, forced, forced-fast)
 * 
 *          Applications: Compass heading for navigation, attitude estimation (with IMU),
 *          magnetic field detection, and position estimation when combined with GPS.
 * 
 * Source: libraries/AP_Compass/AP_Compass_BMM350.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_BMM350_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#define BMM350_I2C_ADDR_MIN 0x14
#define BMM350_I2C_ADDR_MAX 0x17

/**
 * @class AP_Compass_BMM350
 * @brief Driver for Bosch BMM350 magnetometer interfacing via I2C
 * 
 * @details This class implements the AP_Compass_Backend interface for the BMM350
 *          magnetometer. The driver handles device initialization, OTP calibration
 *          data loading, and continuous magnetic field measurements.
 * 
 *          Driver lifecycle:
 *          1. probe() - Factory method attempts device detection and initialization
 *          2. Constructor - Stores device handle and configuration parameters
 *          3. init() - Reads chip ID, loads OTP compensation data, configures sensor
 *          4. timer() - Periodic callback reads magnetic field data at configured rate
 *          5. read() - Applies compensation and updates AP_Compass with field vector
 * 
 *          OTP Calibration Data:
 *          The BMM350 contains factory-programmed OTP (One-Time Programmable) memory
 *          storing calibration coefficients for:
 *          - Offset compensation (X, Y, Z axes and temperature)
 *          - Sensitivity compensation (X, Y, Z axes and temperature)
 *          - Temperature coefficient of offset (TCO)
 *          - Temperature coefficient of sensitivity (TCS)
 *          - Cross-axis compensation factors
 *          - Reference temperature (T0)
 * 
 *          These coefficients are loaded during init() and applied to all measurements
 *          to correct for manufacturing variations and temperature drift.
 * 
 *          Power Modes:
 *          - SUSPEND: Lowest power, sensor inactive
 *          - NORMAL: Continuous measurement mode for autopilot applications
 *          - FORCED: Single measurement on demand
 *          - FORCED_FAST: Fast single measurement with reduced accuracy
 * 
 *          Field Units:
 *          Raw sensor output is converted to micro-Tesla (µT) using OTP calibration,
 *          then scaled to milli-Gauss (mGauss) for AP_Compass (1 µT = 10 mGauss).
 * 
 * @note The BMM350 provides better noise performance than BMM150, especially important
 *       for indoor navigation and magnetic interference detection.
 * 
 * @warning Magnetic interference from motors, power cables, and metal structures can
 *          corrupt measurements. Perform compass calibration away from interference sources.
 */
class AP_Compass_BMM350 : public AP_Compass_Backend
{
public:
    /**
     * @brief Factory method to detect and initialize BMM350 magnetometer
     * 
     * @details Attempts to communicate with the BMM350 sensor via the provided I2C device.
     *          If successful, creates a new AP_Compass_BMM350 instance, reads the chip ID
     *          to verify device identity, loads OTP calibration data, configures the sensor
     *          for continuous measurement mode, and registers with the AP_Compass frontend.
     * 
     *          This method is called by the compass driver framework during sensor detection,
     *          typically at system startup. It tries multiple I2C addresses (0x14-0x17) to
     *          locate the device.
     * 
     * @param[in] dev           I2C device handle with ownership transfer
     * @param[in] force_external If true, treat as external compass (may affect priority/orientation)
     * @param[in] rotation      Board rotation to apply to readings for vehicle frame alignment
     * 
     * @return Pointer to new AP_Compass_BMM350 instance if probe successful, nullptr if failed
     * 
     * @note Probe failure is common if sensor is not present on the bus and is not an error
     * @note The returned backend is managed by AP_Compass and should not be manually deleted
     */
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    /**
     * @brief Read magnetic field data and update AP_Compass frontend
     * 
     * @details Called periodically by the scheduler to retrieve the latest magnetic field
     *          measurements. This method reads raw magnetometer data from the sensor,
     *          applies OTP calibration coefficients (offset, sensitivity, temperature
     *          compensation, and cross-axis corrections), rotates the field vector to
     *          vehicle frame, and publishes to AP_Compass for navigation use.
     * 
     *          In normal (continuous) mode, the BMM350 automatically updates measurements
     *          at the configured output data rate. This method retrieves the most recent
     *          available data without triggering a new measurement.
     * 
     *          Measurement units: Output is in milli-Gauss (mGauss) after conversion from
     *          micro-Tesla (µT). 1 µT = 10 mGauss.
     * 
     * @note Called at main sensor read rate (typically 100 Hz for compass)
     * @note If I2C communication fails, error counter is incremented but system continues
     * 
     * @see timer() for the periodic callback that triggers this method
     */
    void read() override;

    /**
     * @brief Sensor identification string for logging and diagnostics
     */
    static constexpr const char *name = "BMM350";

private:
    /**
     * @brief Private constructor called by probe() after successful device detection
     * 
     * @param[in] dev           I2C device handle with ownership transfer
     * @param[in] force_external If true, compass treated as external (may affect calibration priority)
     * @param[in] rotation      Board rotation applied to align sensor to vehicle frame
     */
    AP_Compass_BMM350(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    /**
     * @brief I2C device handle for communication with BMM350 sensor
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /**
     * @brief BMM350 offset/sensitivity coefficient structure for OTP calibration data
     * 
     * @details Stores 4-dimensional compensation coefficients loaded from OTP memory.
     *          These correct for manufacturing variations in sensor characteristics.
     */
    struct vector4f
    {
        float x;    ///< X axis coefficient (unitless scaling factor)
        float y;    ///< Y axis coefficient (unitless scaling factor)
        float z;    ///< Z axis coefficient (unitless scaling factor)
        float temp; ///< Temperature coefficient (degrees Celsius)
    };

    /**
     * @brief BMM350 magnetometer cross-axis compensation structure
     * 
     * @details Cross-axis factors correct for magnetic coupling between orthogonal axes
     *          due to sensor packaging and die alignment imperfections. Applied during
     *          field calculation to remove off-axis sensitivity.
     * 
     *          Example: cross_x_y represents the X-axis component caused by Y-axis field.
     */
    struct cross_axis
    {
        float cross_x_y; ///< X axis sensitivity to Y field (unitless ratio)
        float cross_y_x; ///< Y axis sensitivity to X field (unitless ratio)
        float cross_z_x; ///< Z axis sensitivity to X field (unitless ratio)
        float cross_z_y; ///< Z axis sensitivity to Y field (unitless ratio)
    };

    /**
     * @brief BMM350 magnetometer complete compensation structure
     * 
     * @details Comprehensive OTP calibration data loaded during initialization.
     *          All measurements are corrected using these factory-programmed coefficients
     *          to achieve specified accuracy across temperature and field strength ranges.
     * 
     *          Compensation algorithm:
     *          1. Apply offset correction based on temperature
     *          2. Apply sensitivity scaling based on temperature
     *          3. Correct cross-axis coupling effects
     *          4. Convert to physical units (micro-Tesla)
     */
    struct mag_compensate
    {
        struct vector4f offset_coef;    ///< Offset coefficients for X/Y/Z axes and temperature
        struct vector4f sensit_coef;    ///< Sensitivity coefficients for X/Y/Z axes and temperature
        Vector3f tco;                   ///< Temperature coefficient of offset (µT/°C per axis)
        Vector3f tcs;                   ///< Temperature coefficient of sensitivity (%/°C per axis)
        float t0_reading;               ///< Reference temperature T0 from OTP (degrees Celsius)
        struct cross_axis cross_axis;   ///< Cross-axis compensation factors (unitless)
    };

    /**
     * @brief BMM350 power mode enumeration
     * 
     * @details Power modes control measurement behavior and current consumption:
     * 
     *          SUSPEND (0): Sensor in sleep mode, < 2 µA current consumption.
     *                       All measurements stopped. Used when compass not needed.
     * 
     *          NORMAL (1): Continuous measurement mode, typical 200 µA consumption.
     *                      Sensor autonomously updates at configured ODR (Output Data Rate).
     *                      Standard mode for autopilot operation.
     * 
     *          FORCED (3): Single measurement on demand, returns to suspend after completion.
     *                      Reduces average power for low-rate applications. ~170 ms per sample.
     * 
     *          FORCED_FAST (4): Fast single measurement with reduced settling time.
     *                           Lower accuracy but faster response (~40 ms). For rapid scanning.
     * 
     * @note ArduPilot typically uses NORMAL mode for continuous heading updates
     * @note Power mode values match BMM350 datasheet register encoding
     */
    enum power_mode
    {
        POWER_MODE_SUSPEND     = 0, ///< Sleep mode, measurements stopped
        POWER_MODE_NORMAL      = 1, ///< Continuous measurement (standard autopilot mode)
        POWER_MODE_FORCED      = 3, ///< Single measurement on demand
        POWER_MODE_FORCED_FAST = 4  ///< Fast single measurement, reduced accuracy
    };

    /**
     * @brief Initialize BMM350 sensor hardware and load OTP calibration data
     * 
     * @details Initialization sequence:
     *          1. Perform soft reset to ensure clean state
     *          2. Wait for Power Management Unit (PMU) ready
     *          3. Read and verify chip ID (should be 0x33 for BMM350)
     *          4. Load OTP calibration data from non-volatile memory
     *          5. Configure power mode (typically NORMAL for continuous operation)
     *          6. Set output data rate and averaging
     *          7. Register periodic timer callback for data reading
     *          8. Register compass instance with AP_Compass frontend
     * 
     * @return true if initialization successful, false if sensor not responding or chip ID invalid
     * 
     * @note Called once during probe() after device handle is acquired
     * @note Failure typically indicates sensor not present or I2C communication issue
     */
    bool init();
    
    /**
     * @brief Periodic timer callback to read magnetic field data from sensor
     * 
     * @details Registered with HAL scheduler during init() to run at configured rate
     *          (typically 100 Hz). Reads raw magnetometer registers, applies compensation,
     *          and calls read() to publish data to AP_Compass frontend.
     * 
     *          This is the entry point for all periodic sensor reads in ArduPilot's
     *          sensor framework.
     * 
     * @note Runs in scheduler thread context, must complete quickly to avoid delays
     * @note I2C communication errors are handled gracefully without blocking
     */
    void timer();
    
    /**
     * @brief Read OTP (One-Time Programmable) calibration data from sensor non-volatile memory
     * 
     * @details The BMM350 stores factory calibration coefficients in OTP memory that must
     *          be loaded during initialization. This method:
     *          1. Enables OTP memory access via PMU command
     *          2. Reads offset coefficients (X, Y, Z, Temperature)
     *          3. Reads sensitivity coefficients (X, Y, Z, Temperature)
     *          4. Reads temperature compensation coefficients (TCO, TCS)
     *          5. Reads cross-axis compensation factors
     *          6. Reads reference temperature T0
     *          7. Stores all data in _mag_comp structure
     * 
     *          These coefficients are essential for accurate measurements and must be
     *          successfully loaded before normal operation.
     * 
     * @return true if OTP data read successfully, false if communication error
     * 
     * @note OTP read failure will cause init() to fail and sensor will not be used
     * @warning Never attempt to write OTP memory - it is factory programmed and one-time only
     */
    bool read_otp_data();
    
    /**
     * @brief Wait for Power Management Unit (PMU) command completion
     * 
     * @details The BMM350 PMU controls power state transitions and must acknowledge
     *          commands before proceeding. This method polls the PMU status register
     *          until the command is accepted or timeout occurs.
     * 
     *          Common PMU commands include reset, power mode changes, and OTP access.
     * 
     * @param[in] cmd     PMU command code being waited for (for logging purposes)
     * @param[in] timeout Maximum wait time in milliseconds
     * 
     * @return true if PMU ready within timeout, false if timeout expired
     * 
     * @note Typical PMU command completion is < 10 ms
     * @note Timeout prevents infinite loop if sensor malfunctions
     */
    bool wait_pmu_cmd_ready(const uint8_t cmd, const uint32_t timeout);
    
    /**
     * @brief Perform magnetometer soft reset and wait for sensor to be ready
     * 
     * @details Issues soft reset command to BMM350, which:
     *          - Resets all registers to default values
     *          - Clears any error conditions
     *          - Returns sensor to suspend mode
     *          - Takes approximately 5-10 ms to complete
     * 
     *          After reset, waits for PMU to indicate sensor is ready for configuration.
     * 
     * @return true if reset successful and sensor ready, false if timeout or error
     * 
     * @note Called during init() to ensure clean starting state
     * @note Does not affect OTP memory (calibration data preserved)
     */
    bool mag_reset_and_wait();
    
    /**
     * @brief Configure BMM350 power mode for measurement control
     * 
     * @details Writes power mode register and waits for PMU to acknowledge transition.
     *          Power mode changes can take up to 20 ms to complete as sensor stabilizes.
     * 
     *          Typical usage:
     *          - Set NORMAL mode during init() for continuous operation
     *          - Could set SUSPEND mode to save power when compass not needed
     *          - FORCED modes not typically used in ArduPilot (continuous mode preferred)
     * 
     * @param[in] mode Power mode from power_mode enum
     * 
     * @return true if mode set successfully, false if I2C error or timeout
     * 
     * @note Mode transitions from SUSPEND to NORMAL take longest (~20 ms)
     * @note Changing mode during operation may cause temporary data gaps
     */
    bool set_power_mode(const enum power_mode mode);
    
    /**
     * @brief Low-level I2C read operation for BMM350 register access
     * 
     * @details Reads consecutive registers starting from specified address.
     *          Handles I2C bus arbitration and error checking.
     * 
     * @param[in]  reg      Starting register address to read
     * @param[out] out      Buffer to store read data
     * @param[in]  read_len Number of bytes to read
     * 
     * @return true if I2C read successful, false if communication error
     * 
     * @note Used by all higher-level read operations (OTP, measurement data, status)
     * @note I2C failures increment error counter but don't halt system
     */
    bool read_bytes(const uint8_t reg, uint8_t *out, const uint16_t read_len);

    /**
     * @brief Compass instance ID assigned by AP_Compass frontend
     * 
     * @details Unique identifier for this compass instance used when publishing
     *          measurements to the frontend. Multiple compasses can be present
     *          (e.g., internal + external), each with different instance ID.
     */
    uint8_t _compass_instance;
    
    /**
     * @brief Flag indicating if compass should be treated as external
     * 
     * @details External compasses (mounted away from vehicle electronics) typically
     *          have less magnetic interference and may receive higher priority for
     *          heading estimation. Set during probe() based on detection method.
     */
    bool _force_external;
    
    /**
     * @brief Board rotation to align sensor frame to vehicle frame
     * 
     * @details Rotation matrix applied to raw measurements to account for physical
     *          sensor mounting orientation. Common rotations: ROTATION_NONE (0°),
     *          ROTATION_YAW_90 (90°), ROTATION_YAW_180 (180°), ROTATION_YAW_270 (270°),
     *          plus pitch and roll variants.
     * 
     * @note Rotation is applied AFTER compensation but BEFORE publishing to frontend
     */
    enum Rotation _rotation;
    
    /**
     * @brief OTP calibration and compensation coefficient storage
     * 
     * @details Loaded from sensor non-volatile memory during init() and applied to
     *          every measurement. Contains offset, sensitivity, temperature compensation,
     *          and cross-axis correction factors.
     * 
     * @see read_otp_data() for loading sequence
     * @see mag_compensate structure for field descriptions
     */
    struct mag_compensate _mag_comp;
};

#endif  // AP_COMPASS_BMM350_ENABLED
