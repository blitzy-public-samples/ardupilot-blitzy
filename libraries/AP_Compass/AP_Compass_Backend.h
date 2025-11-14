/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_Compass_Backend.h
 * @brief Abstract backend interface for compass driver implementations
 * 
 * This file defines the AP_Compass_Backend abstract base class that provides
 * the interface between specific compass sensor drivers and the AP_Compass
 * frontend. Each supported compass sensor type (HMC5883L, LSM303D, AK8963, etc.)
 * must implement a driver class derived from this backend.
 * 
 * The backend handles the measurement processing pipeline:
 * 1. rotate_field() - Transform sensor frame to body frame
 * 2. publish_raw_field() - Provide uncorrected samples for calibration
 * 3. correct_field() - Apply calibration corrections
 * 4. accumulate_sample() - Accumulate samples for averaging
 * 
 * All magnetic field measurements are in milligauss throughout the pipeline.
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_EXTERNALAHRS_ENABLED
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#endif

#if AP_COMPASS_MSP_ENABLED
#include <AP_MSP/msp.h>
#endif

#include <AP_Math/AP_Math.h>

class Compass;  // forward declaration

/**
 * @class AP_Compass_Backend
 * @brief Abstract base class for all compass sensor driver implementations
 * 
 * @details This class provides the interface between specific compass sensor hardware
 * drivers and the AP_Compass frontend. Each supported compass sensor must implement
 * a derived class that implements the read() method and uses the measurement
 * processing pipeline provided by this base class.
 * 
 * Measurement Processing Pipeline:
 * - rotate_field(): Applies board rotation and instance-specific rotation matrices
 *   to transform measurements from sensor frame → board frame → body frame
 * - publish_raw_field(): Provides uncorrected field samples to calibration algorithms
 * - correct_field(): Applies calibration corrections including:
 *   * Hard-iron offsets (magnetic bias from vehicle)
 *   * Soft-iron corrections (diagonals and offdiagonals for magnetic distortion)
 *   * Motor compensation (interference from current draw)
 * - accumulate_sample(): Thread-safe sample accumulation for averaging
 * - drain_accumulated_samples(): Publishes averaged samples to frontend
 * 
 * Driver Implementation Requirements:
 * - Implement read() method to acquire sensor data
 * - Call register_compass() during initialization to obtain instance number
 * - Use measurement pipeline methods in correct order
 * - Set device ID using set_dev_id() for calibration persistence
 * - Configure rotation with set_rotation() if needed
 * - Set external flag with set_external() for external compasses
 * 
 * Coordinate Frames:
 * - Sensor frame: Native orientation of compass chip
 * - Board frame: Orientation accounting for sensor mounting on board
 * - Body frame: Vehicle body frame (NED convention)
 * 
 * @note All field measurements are in milligauss throughout the pipeline
 * @note Thread safety provided by HAL_Semaphore for accumulated samples
 */
class AP_Compass_Backend
{
public:
    AP_Compass_Backend();

    /**
     * @brief Virtual destructor allowing custom cleanup in derived drivers
     * 
     * Drivers can override this destructor to perform driver-specific cleanup
     * such as releasing hardware resources or stopping periodic callbacks.
     */
    virtual ~AP_Compass_Backend(void) {}

    /**
     * @brief Read sensor data and update compass measurements
     * 
     * @details Pure virtual method that must be implemented by each compass driver.
     * This method is called periodically by the scheduler to acquire new magnetometer
     * measurements from the sensor hardware.
     * 
     * Implementation should:
     * 1. Read raw magnetic field data from sensor (via I2C, SPI, or other interface)
     * 2. Convert to milligauss units
     * 3. Pass through measurement pipeline: rotate_field() → publish_raw_field() →
     *    correct_field() → publish_filtered_field() OR use accumulate_sample()
     * 4. Handle sensor errors gracefully
     * 
     * @note Called at compass sample rate (typically 10-100 Hz depending on sensor)
     * @note Must be implemented by all derived driver classes
     */
    virtual void read(void) = 0;

    /**
     * @enum DevTypes
     * @brief Device type identifiers for compass sensors
     * 
     * @details These device type IDs are used to identify specific compass sensor
     * hardware and are stored in the devtype field of the device ID. The device ID
     * appears in COMPASS*ID* parameters (e.g., COMPASS_DEV_ID, COMPASS_DEV_ID2) that
     * users see in ground control stations.
     * 
     * The values are chosen for compatibility with existing PX4 firmware drivers to
     * enable calibration data portability between ArduPilot and PX4.
     * 
     * Calibration Persistence:
     * Device type IDs are used to match stored calibration parameters (offsets, scale
     * factors, motor compensation) to physical compass hardware. This ensures that
     * calibration data persists across reboots and parameter resets.
     * 
     * @warning If a driver change would invalidate existing calibration values
     * (e.g., changing axis mapping, units, or scale factors), the DevType value
     * MUST be changed to force users to recalibrate. Using the same DevType with
     * incompatible calibration data will result in incorrect compass readings and
     * potentially dangerous flight behavior.
     * 
     * @note Values must remain stable across firmware versions for calibration compatibility
     * @note Each sensor variant should have a unique DevType ID
     */
    enum DevTypes {
        DEVTYPE_HMC5883_OLD = 0x01, ///< @brief Honeywell HMC5883L (legacy driver variant)
        DEVTYPE_HMC5883 = 0x07,     ///< @brief Honeywell HMC5883L 3-axis magnetometer (current driver)
        DEVTYPE_LSM303D = 0x02,     ///< @brief STMicroelectronics LSM303D 3-axis accelerometer/magnetometer
        DEVTYPE_AK8963  = 0x04,     ///< @brief Asahi Kasei AK8963 3-axis magnetometer (used in MPU9250)
        DEVTYPE_BMM150  = 0x05,     ///< @brief Bosch BMM150 3-axis magnetometer
        DEVTYPE_LSM9DS1 = 0x06,     ///< @brief STMicroelectronics LSM9DS1 IMU with magnetometer
        DEVTYPE_LIS3MDL = 0x08,     ///< @brief STMicroelectronics LIS3MDL 3-axis magnetometer
        DEVTYPE_AK09916 = 0x09,     ///< @brief Asahi Kasei AK09916 3-axis magnetometer (used in MPU9250 variants)
        DEVTYPE_IST8310 = 0x0A,     ///< @brief Isentek IST8310 3-axis magnetometer
        DEVTYPE_ICM20948 = 0x0B,    ///< @brief TDK InvenSense ICM-20948 IMU with integrated AK09916 magnetometer
        DEVTYPE_MMC3416 = 0x0C,     ///< @brief MEMSIC MMC3416xPJ 3-axis magnetometer
        DEVTYPE_QMC5883L = 0x0D,    ///< @brief QST QMC5883L 3-axis magnetometer (HMC5883L compatible)
        DEVTYPE_MAG3110  = 0x0E,    ///< @brief NXP MAG3110 3-axis magnetometer
        DEVTYPE_SITL  = 0x0F,       ///< @brief Software-In-The-Loop simulated compass
        DEVTYPE_IST8308 = 0x10,     ///< @brief Isentek IST8308 3-axis magnetometer
        DEVTYPE_RM3100 = 0x11,      ///< @brief PNI RM3100 high-precision 3-axis magnetometer
        DEVTYPE_RM3100_2 = 0x12,    ///< @brief Unused - past driver mistake, do not reuse
        DEVTYPE_MMC5983 = 0x13,     ///< @brief MEMSIC MMC5983MA 3-axis magnetometer
        DEVTYPE_AK09918 = 0x14,     ///< @brief Asahi Kasei AK09918 3-axis magnetometer
        DEVTYPE_AK09915 = 0x15,     ///< @brief Asahi Kasei AK09915 3-axis magnetometer
    	DEVTYPE_QMC5883P = 0x16,    ///< @brief QST QMC5883P 3-axis magnetometer
        DEVTYPE_BMM350 = 0x17,      ///< @brief Bosch BMM350 3-axis magnetometer
        DEVTYPE_IIS2MDC = 0x18,     ///< @brief STMicroelectronics IIS2MDC 3-axis magnetometer
    };

#if AP_COMPASS_MSP_ENABLED
    /**
     * @brief Handle compass data received via MSP protocol
     * 
     * @param[in] pkt MSP compass data message containing magnetic field measurements
     * 
     * @details Virtual method for processing magnetometer data received through the
     * MultiWii Serial Protocol (MSP). Used primarily for external compasses connected
     * via MSP telemetry from external flight controllers or OSD devices.
     * 
     * Default implementation does nothing; override in derived classes that support MSP.
     */
    virtual void handle_msp(const MSP::msp_compass_data_message_t &pkt) {}
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    /**
     * @brief Handle compass data from external AHRS system
     * 
     * @param[in] pkt External AHRS magnetometer data message
     * 
     * @details Virtual method for processing magnetometer data from external Attitude
     * and Heading Reference Systems (e.g., VectorNav, LORD MicroStrain). External AHRS
     * devices often include integrated magnetometers and provide pre-processed measurements.
     * 
     * Default implementation does nothing; override in derived classes that support external AHRS.
     */
    virtual void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt) {}
#endif
    
protected:

    /*
     * Compass Measurement Processing Pipeline
     * 
     * A compass measurement is expected to pass through the following functions:
     * 1. rotate_field - this rotates the measurement in-place from sensor frame
     *      to body frame
     * 2. publish_raw_field - this provides an uncorrected point-sample for
     *      calibration libraries
     * 3. correct_field - this corrects the measurement in-place for hard iron,
     *      soft iron, motor interference, and non-orthogonality errors
     * 4. publish_filtered_field - legacy filtered magnetic field
     *
     * All those functions expect the mag field to be in milligauss.
     */

    /**
     * @brief Apply rotation transformations to magnetic field measurement
     * 
     * @param[in,out] mag Magnetic field vector in milligauss (modified in-place)
     * @param[in] instance Compass instance number (0-based)
     * 
     * @details Transforms the magnetic field measurement from sensor frame to body frame
     * by applying two rotation matrices in sequence:
     * 1. Board rotation: Accounts for sensor mounting orientation on the flight controller
     * 2. Instance rotation: Additional rotation specific to this compass instance
     * 
     * The rotation transforms measurements through: sensor frame → board frame → body frame
     * where body frame follows NED (North-East-Down) convention.
     * 
     * This must be the first step in the measurement pipeline to ensure all subsequent
     * processing operates in the correct coordinate frame.
     * 
     * @note Field must be in milligauss units
     * @note Rotation matrices are configured via set_rotation() and board orientation parameters
     */
    void rotate_field(Vector3f &mag, uint8_t instance);

    /**
     * @brief Publish uncorrected raw magnetic field sample for calibration
     * 
     * @param[in] mag Raw magnetic field vector in milligauss (after rotation)
     * @param[in] instance Compass instance number (0-based)
     * 
     * @details Provides raw, uncorrected field measurements to calibration algorithms.
     * Calibration libraries use these raw samples to calculate correction parameters:
     * - Hard-iron offsets (constant magnetic bias from vehicle frame)
     * - Soft-iron corrections (axis scaling and cross-coupling from magnetic distortion)
     * - Motor compensation (field variation correlated with motor current)
     * 
     * This should be called after rotate_field() but before correct_field() to ensure
     * calibration algorithms see measurements in body frame but without corrections applied.
     * 
     * @note Field must already be rotated to body frame via rotate_field()
     * @note Field units must be in milligauss
     */
    void publish_raw_field(const Vector3f &mag, uint8_t instance);

    /**
     * @brief Apply calibration corrections to magnetic field measurement
     * 
     * @param[in,out] mag Magnetic field vector in milligauss (modified in-place)
     * @param[in] i Compass instance number (0-based)
     * 
     * @details Applies stored calibration corrections to compensate for magnetic interference
     * and sensor imperfections:
     * 
     * 1. Hard-iron offset correction: Subtracts constant magnetic bias from vehicle frame
     *    (e.g., ferromagnetic materials, permanent magnets)
     * 
     * 2. Soft-iron correction: Applies 3x3 correction matrix to compensate for:
     *    - Diagonal elements: Per-axis scale factors
     *    - Off-diagonal elements: Cross-axis coupling from soft ferromagnetic materials
     * 
     * 3. Motor compensation: Subtracts magnetic field proportional to motor current draw,
     *    compensating for interference from motor magnetic fields
     * 
     * Correction formula:
     *   corrected = scale_matrix * (raw - offset) - motor_compensation
     * 
     * @note Must be called after publish_raw_field() in pipeline
     * @note Corrections are loaded from stored calibration parameters (COMPASS_OFS*, COMPASS_DIA*, COMPASS_ODI*, COMPASS_MOT*)
     * @note Field units must be in milligauss
     */
    void correct_field(Vector3f &mag, uint8_t i);

    /**
     * @brief Publish corrected and filtered magnetic field to frontend
     * 
     * @param[in] mag Corrected magnetic field vector in milligauss
     * @param[in] instance Compass instance number (0-based)
     * 
     * @details Legacy method that publishes the final corrected magnetic field measurement
     * to the AP_Compass frontend for use by navigation algorithms (EKF, DCM).
     * 
     * This is the final step in the measurement pipeline after rotation and correction.
     * Modern drivers often use accumulate_sample()/drain_accumulated_samples() instead
     * for better noise reduction through averaging.
     * 
     * @note Field must be fully corrected via correct_field() before calling
     * @note Field units must be in milligauss
     */
    void publish_filtered_field(const Vector3f &mag, uint8_t instance);

    /**
     * @brief Update timestamp of last measurement for this compass instance
     * 
     * @param[in] last_update Timestamp in microseconds (from AP_HAL::micros())
     * @param[in] instance Compass instance number (0-based)
     * 
     * @details Records the timestamp when the compass measurement was acquired.
     * Used by the EKF and other navigation algorithms to time-align measurements
     * and detect sensor timeouts.
     */
    void set_last_update_usec(uint32_t last_update, uint8_t instance);

    /**
     * @brief Accumulate magnetic field sample for averaging
     * 
     * @param[in,out] field Magnetic field vector in milligauss (passed by reference for efficiency)
     * @param[in] instance Compass instance number (0-based)
     * @param[in] max_samples Maximum number of samples to accumulate before forcing drain (default: 10)
     * 
     * @details Thread-safe accumulation of magnetic field samples for noise reduction through
     * averaging. This method is preferred over direct publish_filtered_field() calls for
     * high-rate sensors where averaging multiple samples improves measurement quality.
     * 
     * Operation:
     * 1. Acquires semaphore lock for thread-safe access to accumulator
     * 2. Adds field measurement to running sum
     * 3. Increments sample count
     * 4. If sample count reaches max_samples, automatically calls drain_accumulated_samples()
     * 
     * Typical usage pattern in driver read() method:
     * - Read sensor at high rate (e.g., 100 Hz)
     * - rotate_field() to body frame
     * - publish_raw_field() for calibration
     * - correct_field() to apply calibration
     * - accumulate_sample() instead of publish_filtered_field()
     * - drain is called automatically when max_samples reached, or manually at lower rate
     * 
     * @note Thread-safe via HAL_Semaphore (_sem)
     * @note Field should be corrected before accumulation
     * @note Accumulation happens in body frame after all transformations
     * @note Field units must be in milligauss
     */
    void accumulate_sample(Vector3f &field, uint8_t instance,
                           uint32_t max_samples = 10);

    /**
     * @brief Compute average of accumulated samples and publish to frontend
     * 
     * @param[in] instance Compass instance number (0-based)
     * @param[in] scale Optional scale factor to apply to averaged measurement (default: NULL for no scaling)
     * 
     * @details Computes the average of all accumulated magnetic field samples and publishes
     * the result to the AP_Compass frontend for use by navigation algorithms.
     * 
     * Operation:
     * 1. Acquires semaphore lock for thread-safe access
     * 2. Divides accumulated sum by sample count to get average
     * 3. Optionally applies scale factor if provided
     * 4. Publishes averaged field to frontend
     * 5. Resets accumulator sum and count to zero
     * 
     * This method should be called at the desired output rate (typically 10-50 Hz) after
     * multiple samples have been accumulated at higher sensor rate.
     * 
     * Scale factor usage:
     * If scale is non-NULL, applies per-axis scaling: field.x *= scale->x, etc.
     * Used by some drivers for additional sensor-specific corrections.
     * 
     * @note Thread-safe via HAL_Semaphore (_sem)
     * @note Safe to call even if no samples accumulated (does nothing)
     * @note Resets accumulator after publishing
     * @note Averaged field units are in milligauss
     */
    void drain_accumulated_samples(uint8_t instance, const Vector3f *scale = NULL);

    /**
     * @brief Register a new compass instance with the AP_Compass frontend
     * 
     * @param[in] dev_id Device ID encoding bus type, bus number, address, and device type
     * @param[out] instance Compass instance number assigned by frontend (0-based)
     * 
     * @return true if registration successful, false if maximum compass count exceeded
     * 
     * @details Registers this compass backend with the AP_Compass frontend during driver
     * initialization. The frontend assigns an instance number that uniquely identifies
     * this compass among multiple compass sensors in the system.
     * 
     * Device ID encoding (32-bit):
     * - Bits 0-7: Device type (from DevTypes enum)
     * - Bits 8-15: Device address on bus
     * - Bits 16-23: Bus number
     * - Bits 24-31: Bus type (I2C, SPI, etc.)
     * 
     * The device ID is used to:
     * - Match stored calibration parameters to physical hardware
     * - Persist compass configuration across reboots
     * - Identify specific sensors in multi-compass systems
     * - Detect hardware changes that require recalibration
     * 
     * Typical usage in driver probe/init:
     * @code
     * uint8_t instance;
     * if (!register_compass(dev_id, instance)) {
     *     // Maximum compass count reached
     *     return false;
     * }
     * // Use instance for subsequent backend calls
     * @endcode
     * 
     * @note Must be called during driver initialization before using instance number
     * @note Instance number is assigned sequentially by frontend
     * @note Maximum number of compasses defined by COMPASS_MAX_INSTANCES
     */
    bool register_compass(int32_t dev_id, uint8_t& instance) const;

    /**
     * @brief Set device ID for a compass instance
     * 
     * @param[in] instance Compass instance number (0-based)
     * @param[in] dev_id Device ID encoding hardware details (see register_compass())
     * 
     * @details Updates the device ID for an already-registered compass instance.
     * Typically used during initialization after register_compass() to update or
     * correct the device ID based on additional hardware detection.
     * 
     * @note Device ID changes will reset calibration if DevType changes
     */
    void set_dev_id(uint8_t instance, uint32_t dev_id);

    /**
     * @brief Save device ID to persistent storage
     * 
     * @param[in] instance Compass instance number (0-based)
     * 
     * @details Saves the current device ID to parameter storage. Used primarily
     * by SITL (Software-In-The-Loop) simulation to persist simulated compass
     * configuration across simulator restarts.
     * 
     * @note Normally not needed in hardware drivers (device ID saved automatically)
     */
    void save_dev_id(uint8_t instance);

    /**
     * @brief Configure whether compass is external to flight controller
     * 
     * @param[in] instance Compass instance number (0-based)
     * @param[in] external true if compass is external, false if internal
     * 
     * @details Sets the external flag for this compass instance. External compasses
     * are physically separated from the flight controller (e.g., GPS+compass module,
     * external magnetometer on mast) and typically experience less magnetic interference
     * from flight controller electronics.
     * 
     * External compass status affects:
     * - Compass priority in multi-compass systems (external often preferred)
     * - Magnetic declination handling
     * - Calibration quality assessment
     * - User interface display in ground control stations
     * 
     * @note Should be set during driver initialization based on hardware configuration
     */
    void set_external(uint8_t instance, bool external);

    /**
     * @brief Check if compass instance is configured as external
     * 
     * @param[in] instance Compass instance number (0-based)
     * 
     * @return true if compass is external to flight controller, false if internal
     * 
     * @details Queries whether this compass is configured as external. Used by
     * multi-compass selection algorithms and health monitoring.
     */
    bool is_external(uint8_t instance);

    /**
     * @brief Set rotation/orientation for compass instance
     * 
     * @param[in] instance Compass instance number (0-based)
     * @param[in] rotation Rotation enum value defining sensor orientation
     * 
     * @details Configures the rotation transformation applied to this compass instance
     * to account for non-standard sensor mounting orientation. The rotation is applied
     * by rotate_field() to transform sensor measurements to body frame.
     * 
     * Common rotations:
     * - ROTATION_NONE: Sensor aligned with board (default)
     * - ROTATION_YAW_90: Sensor rotated 90° clockwise
     * - ROTATION_ROLL_180: Sensor mounted upside down
     * 
     * Rotation combines with board-level rotation to achieve final transformation:
     *   body_frame = board_rotation * instance_rotation * sensor_frame
     * 
     * @note Rotation must be set before sensor measurements are processed
     * @note Incorrect rotation will cause compass heading errors
     */
    void set_rotation(uint8_t instance, enum Rotation rotation);

    /**
     * @brief Get board-level orientation configuration
     * 
     * @return Board rotation enum value from AHRS_ORIENTATION parameter
     * 
     * @details Returns the board-level rotation that accounts for flight controller
     * mounting orientation in the vehicle. Used primarily by SITL simulation to
     * match simulated sensor orientation to board configuration.
     * 
     * @note Board rotation is applied to all sensors on the flight controller
     */
    enum Rotation get_board_orientation(void) const;
    
    /**
     * @brief Reference to AP_Compass frontend
     * 
     * Provides backend access to the frontend for publishing measurements,
     * querying configuration, and accessing shared state.
     */
    Compass &_compass;

    /**
     * @brief Semaphore for thread-safe access to shared frontend data
     * 
     * @details Provides thread-safe synchronization when accessing shared state
     * between driver callbacks (often in interrupt/DMA context) and main thread.
     * Used primarily by accumulate_sample() and drain_accumulated_samples() to
     * protect the sample accumulator.
     * 
     * Usage with WITH_SEMAPHORE macro:
     * @code
     * WITH_SEMAPHORE(_sem);
     * // Critical section with exclusive access
     * @endcode
     * 
     * @note HAL_Semaphore is a recursive mutex allowing same thread to lock multiple times
     */
    HAL_Semaphore _sem;

    /**
     * @brief Validate magnetic field measurement is within reasonable range
     * 
     * @param[in] field Magnetic field vector in milligauss
     * 
     * @return true if field magnitude is reasonable, false if out of range
     * 
     * @details Validates that the measured magnetic field magnitude is within expected
     * range for Earth's magnetic field using a mean filter on vector length. Detects:
     * - Sensor hardware failures producing unrealistic values
     * - Extreme magnetic interference beyond calibration range
     * - Data corruption or communication errors
     * 
     * Earth's magnetic field ranges from ~250 mG (equator) to ~650 mG (poles).
     * This function uses a running average to adapt to local field strength and
     * rejects measurements that deviate significantly from the mean.
     * 
     * @note Should be called after rotate_field() but before accumulation
     * @note Failed validation increments error count returned by get_error_count()
     */
    bool field_ok(const Vector3f &field);
    
    /**
     * @brief Get count of invalid/dropped measurements for this backend
     * 
     * @return Number of measurements rejected due to field_ok() failures
     * 
     * @details Returns the cumulative count of measurements that failed validation.
     * High error counts indicate sensor problems or extreme magnetic interference.
     * 
     * Future use: Could be used by frontend to select most reliable compass in
     * multi-compass configurations.
     * 
     * @note Error count is maintained per backend instance
     */
    uint32_t get_error_count() const { return _error_count; }
private:
    /**
     * @brief Apply all calibration corrections to magnetic field measurement
     * 
     * @param[in,out] mag Magnetic field vector in milligauss (modified in-place)
     * @param[in] i Compass instance number (0-based)
     * 
     * @details Internal method that applies the complete correction pipeline:
     * hard-iron offsets, soft-iron matrix, and motor compensation. Called by
     * the public correct_field() method.
     * 
     * @note Private implementation detail - drivers should use correct_field()
     */
    void apply_corrections(Vector3f &mag, uint8_t i);
    
    /**
     * @brief Running mean of magnetic field magnitude for validation
     * 
     * Used by field_ok() to maintain an adaptive threshold for rejecting
     * outlier measurements. Updated with each valid measurement to track
     * local magnetic field strength.
     * 
     * Units: milligauss
     */
    float _mean_field_length;

    /**
     * @brief Count of measurements rejected by field_ok() validation
     * 
     * Incremented each time field_ok() rejects a measurement as out of range.
     * Not currently used by frontend but available for future compass health
     * monitoring and selection algorithms.
     * 
     * High error counts indicate sensor problems or extreme magnetic interference.
     */
    uint32_t _error_count;
};
