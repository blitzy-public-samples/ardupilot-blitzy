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
 * @file AP_InertialSensor_Backend.h
 * @brief IMU backend base class defining driver interface contract
 * 
 * Defines abstract base class that all IMU sensor drivers must inherit from,
 * providing common infrastructure for sensor registration, data publication,
 * filtering, and logging. Each supported gyro/accel sensor type needs to have
 * an object derived from this class.
 * 
 * Note that drivers can implement just gyros or just accels, and can also
 * provide multiple gyro/accel instances.
 */
#pragma once

#include <inttypes.h>

#include <AP_Math/AP_Math.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#include "AP_InertialSensor.h"

#ifndef HAL_INS_HIGHRES_SAMPLE
#define HAL_INS_HIGHRES_SAMPLE 0
#endif

class AuxiliaryBus;
class AP_Logger;

/**
 * @class AP_InertialSensor_Backend
 * @brief Abstract base class for all IMU sensor drivers
 * 
 * @details Backend infrastructure providing:
 *          - Sensor instance registration (gyro/accel with frontend)
 *          - Raw sample accumulation and delta angle/velocity calculation
 *          - Rotation matrix application for sensor orientation
 *          - Calibration offset/scale factor application
 *          - Filter pipeline integration (notch, low-pass)
 *          - Coning and sculling integral compensation
 *          - Logging hooks for raw and processed data
 *          - Temperature tracking per sensor instance
 *          
 *          Driver lifecycle (subclass responsibilities):
 *          1. **probe()**: Static factory method detecting sensor via bus scan
 *          2. **start()**: Initialize sensor hardware (registers, FIFO, interrupts)
 *          3. **accumulate()**: Collect raw samples in background (optional)
 *          4. **update()**: Publish accumulated data to frontend (required)
 *          5. **get_auxiliary_bus()**: Return I2C master for external sensors (optional)
 *          
 *          Data publication pattern:
 *          ```cpp
 *          void MyIMU::update() {
 *              Vector3f gyro, accel;
 *              read_sensor(gyro, accel);  // Get raw data from hardware
 *              _publish_gyro(gyro_instance, gyro);
 *              _publish_accel(accel_instance, accel);
 *          }
 *          ```
 *          
 *          Filter pipeline (applied by backend base):
 *          - Notch filters: Harmonic notch for propeller/motor noise
 *          - Low-pass filter: Configurable cutoff per INS_GYRO_FILTER/INS_ACCEL_FILTER
 *          - Downsampling: From sensor rate to vehicle loop rate
 *          
 *          Coning/sculling compensation:
 *          - Coning: Gyro cross-coupling correction during rotation
 *          - Sculling: Accel vibration rectification during linear motion
 *          - Applied automatically by _publish_delta_angle/velocity
 * 
 * @note Subclasses must implement update() pure virtual method
 * @note Always call _publish_gyro/_publish_accel, never direct frontend access
 * @warning Incorrect rotation matrix causes attitude errors - verify with vehicle testing
 */
class AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Backend(AP_InertialSensor &imu);
    AP_InertialSensor_Backend(const AP_InertialSensor_Backend &that) = delete;

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_InertialSensor_Backend(void) {}

    /**
     * @brief Update sensor data - main backend update hook (pure virtual)
     * 
     * @details Called by frontend to transfer accumulated sensor readings to frontend
     *          structure via _publish_gyro() and _publish_accel() functions. This is
     *          the primary method that all backend drivers must implement.
     *          
     *          Typical implementation pattern:
     *          - Read current sensor data from hardware
     *          - Apply any driver-specific processing
     *          - Call _publish_gyro() and _publish_accel() to propagate data
     *          
     *          Called at vehicle loop rate (typically 50Hz-400Hz depending on vehicle type).
     * 
     * @return true if update successful, false on sensor read failure
     * 
     * @note This method is called from the main vehicle thread
     * @see _publish_gyro(), _publish_accel()
     */
    virtual bool update() = 0; /* front end */

    // if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    /**
     * @brief Update filter parameters at runtime
     * 
     * @details Called by frontend to propagate filter parameter changes to backend
     *          via update_gyro_filters() and update_accel_filters() functions.
     *          Updates notch filter and low-pass filter configurations when parameters
     *          change (INS_GYRO_FILTER, INS_ACCEL_FILTER, INS_HNTCH_* parameters).
     * 
     * @note Marked __RAMFUNC__ for fast execution from RAM
     */
    void update_filters() __RAMFUNC__; /* front end */
    // endif AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

    /**
     * @brief Optional function to accumulate more samples
     * 
     * @details Provides hook for drivers that don't use a timer/interrupt to gather
     *          samples. Called frequently to allow polling-based drivers to collect
     *          data. Timer-based (interrupt-driven) drivers can leave default empty
     *          implementation.
     *          
     *          Example usage: SPI polling without DMA, I2C polling without interrupts.
     * 
     * @note Default implementation does nothing (suitable for interrupt-driven sensors)
     */
    virtual void accumulate() {}

    /**
     * @brief Configure and start all sensors
     * 
     * @details Initialize sensor hardware including:
     *          - Register configuration (sample rates, filters, ranges)
     *          - FIFO setup if supported
     *          - Interrupt configuration
     *          - Self-test procedures
     *          
     *          Empty default implementation allows subclasses to start sensors
     *          during probe/detect phase if needed.
     * 
     * @note Called once during AP_InertialSensor::_startup_default_sensors()
     */
    virtual void start() { }

    /**
     * @brief Return auxiliary I2C/SPI bus if backend provides one
     * 
     * @details Some IMU chips (e.g., MPU6000, MPU9250) have an auxiliary I2C master
     *          that can be used to connect external sensors (magnetometers, barometers).
     *          Override this method to export the auxiliary bus interface.
     * 
     * @return Pointer to AuxiliaryBus interface, or nullptr if not supported
     * 
     * @see AuxiliaryBus class for auxiliary bus interface details
     */
    virtual AuxiliaryBus *get_auxiliary_bus() { return nullptr; }

    /**
     * @brief Return unique backend identifier
     * 
     * @details Backend ID is the same for all sensors (gyro/accel) registered by
     *          this backend instance. Used to correlate sensor instances that come
     *          from the same physical hardware.
     * 
     * @return Backend ID (unique per hardware instance), or -1 if not identified
     */
    int16_t get_id() const { return _id; }

    /**
     * @brief Get accelerometer clip limit threshold
     * 
     * @return Clip limit in m/s/s (default: 15.5g = 152 m/s/s)
     * 
     * @note Samples exceeding this threshold are flagged as clipped
     */
    float get_clip_limit() const { return _clip_limit; }

    /**
     * @brief Generate startup banner message for GCS
     * 
     * @details Provides human-readable sensor identification string for display
     *          on ground control station during initialization.
     * 
     * @param[out] banner Buffer to write banner string
     * @param[in] banner_len Maximum banner buffer length in bytes
     * 
     * @return true if banner generated, false if not supported
     */
    virtual bool get_output_banner(char* banner, uint8_t banner_len) { return false; }

#if AP_EXTERNAL_AHRS_ENABLED
    /**
     * @brief Handle external AHRS IMU data
     * 
     * @details Process IMU data from external AHRS systems (e.g., VectorNav, LORD).
     *          Override to handle INS data from external AHRS hardware.
     * 
     * @param[in] pkt External AHRS INS data packet
     * 
     * @note Only compiled if AP_EXTERNAL_AHRS_ENABLED
     */
    virtual void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt) {}
#endif

#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    bool has_been_killed(uint8_t instance) const { return ((1U<<instance) & _imu.imu_kill_mask); }
#else
    bool has_been_killed(uint8_t instance) const { return false; }
#endif

    // get the backend update rate for the gyro in Hz
    // if the backend polling rate is the same as the sample rate or higher, return raw sample rate
    // override and return the backend rate in Hz if it is lower than the sample rate
    virtual uint16_t get_gyro_backend_rate_hz() const {
        return _gyro_raw_sample_rate(gyro_instance);
    }

    /**
     * @enum DevTypes
     * @brief Device driver type identifiers for IMU sensors
     * 
     * @details These IDs fill the devtype field of the device ID, appearing as
     *          INS*ID* parameters to users (e.g., INS_ACC_ID, INS_GYR_ID).
     *          Values chosen for compatibility with existing PX4 drivers.
     *          
     *          CRITICAL: If a driver change invalidates existing calibration values,
     *          this number MUST be changed to force recalibration.
     *          
     *          Naming conventions:
     *          - DEVTYPE_ACC_* : Accelerometer-only sensors
     *          - DEVTYPE_GYR_* : Gyroscope-only sensors  
     *          - DEVTYPE_INS_* : Combined IMU (gyro+accel)
     *          - DEVTYPE_BMI*, DEVTYPE_BMI270 : Bosch sensors
     *          - DEVTYPE_*ICM* : InvenSense/TDK sensors
     *          - DEVTYPE_*LSM* : STMicroelectronics sensors
     *          - DEVTYPE_SITL : Software-in-the-loop simulation
     *          - DEVTYPE_SERIAL : Generic serial protocol IMU
     */
    enum DevTypes {
        DEVTYPE_BMI160       = 0x09,  ///< Bosch BMI160 6-axis IMU
        DEVTYPE_L3G4200D     = 0x10,  ///< ST L3G4200D gyroscope
        DEVTYPE_ACC_LSM303D  = 0x11,  ///< ST LSM303D accelerometer
        DEVTYPE_ACC_BMA180   = 0x12,  ///< Bosch BMA180 accelerometer
        DEVTYPE_ACC_MPU6000  = 0x13,  ///< InvenSense MPU6000 accelerometer
        DEVTYPE_ACC_MPU9250  = 0x16,  ///< InvenSense MPU9250 accelerometer
        DEVTYPE_ACC_IIS328DQ = 0x17,  ///< ST IIS328DQ accelerometer
        DEVTYPE_ACC_LSM9DS1  = 0x18,  ///< ST LSM9DS1 accelerometer
        DEVTYPE_GYR_MPU6000  = 0x21,  ///< InvenSense MPU6000 gyroscope
        DEVTYPE_GYR_L3GD20   = 0x22,  ///< ST L3GD20 gyroscope
        DEVTYPE_GYR_MPU9250  = 0x24,  ///< InvenSense MPU9250 gyroscope
        DEVTYPE_GYR_I3G4250D = 0x25,  ///< ST I3G4250D gyroscope
        DEVTYPE_GYR_LSM9DS1  = 0x26,  ///< ST LSM9DS1 gyroscope
        DEVTYPE_INS_ICM20789 = 0x27,  ///< InvenSense ICM-20789 6-axis IMU
        DEVTYPE_INS_ICM20689 = 0x28,  ///< InvenSense ICM-20689 6-axis IMU
        DEVTYPE_INS_BMI055   = 0x29,  ///< Bosch BMI055 6-axis IMU
        DEVTYPE_SITL         = 0x2A,  ///< Software-in-the-loop simulation
        DEVTYPE_INS_BMI088   = 0x2B,  ///< Bosch BMI088 6-axis IMU (high performance)
        DEVTYPE_INS_ICM20948 = 0x2C,  ///< InvenSense ICM-20948 9-axis IMU
        DEVTYPE_INS_ICM20648 = 0x2D,  ///< InvenSense ICM-20648 6-axis IMU
        DEVTYPE_INS_ICM20649 = 0x2E,  ///< InvenSense ICM-20649 6-axis IMU
        DEVTYPE_INS_ICM20602 = 0x2F,  ///< InvenSense ICM-20602 6-axis IMU
        DEVTYPE_INS_ICM20601 = 0x30,  ///< InvenSense ICM-20601 6-axis IMU
        DEVTYPE_INS_ADIS1647X = 0x31, ///< Analog Devices ADIS16470 tactical grade IMU
        DEVTYPE_SERIAL       = 0x32,  ///< Generic serial protocol IMU
        DEVTYPE_INS_ICM40609 = 0x33,  ///< TDK ICM-40609 6-axis IMU
        DEVTYPE_INS_ICM42688 = 0x34,  ///< TDK ICM-42688-P 6-axis IMU (high performance)
        DEVTYPE_INS_ICM42605 = 0x35,  ///< TDK ICM-42605 6-axis IMU
        DEVTYPE_INS_ICM40605 = 0x36,  ///< TDK ICM-40605 6-axis IMU
        DEVTYPE_INS_IIM42652 = 0x37,  ///< TDK IIM-42652 6-axis IMU
        DEVTYPE_BMI270       = 0x38,  ///< Bosch BMI270 6-axis IMU (low power)
        DEVTYPE_INS_BMI085   = 0x39,  ///< Bosch BMI085 6-axis IMU
        DEVTYPE_INS_ICM42670 = 0x3A,  ///< TDK ICM-42670-P 6-axis IMU
        DEVTYPE_INS_ICM45686 = 0x3B,  ///< TDK ICM-45686 6-axis IMU
        DEVTYPE_INS_SCHA63T  = 0x3C,  ///< Murata SCHA63T-K03 automotive grade IMU
        DEVTYPE_INS_IIM42653 = 0x3D,  ///< TDK IIM-42653 6-axis IMU
    };

protected:
    /// Frontend reference - access to shared AP_InertialSensor state
    AP_InertialSensor &_imu;

    /// Semaphore protecting access to shared frontend data structures
    HAL_Semaphore _sem;

    /// Clip limit threshold in m/s/s - samples exceeding this are flagged (default: 15.5g)
    float _clip_limit = (16.0f - 0.5f) * GRAVITY_MSS;

    /// Gyro instance number assigned by frontend during registration
    uint8_t gyro_instance;
    
    /// Accelerometer instance number assigned by frontend during registration
    uint8_t accel_instance;
    
    /// True if this backend provides the primary IMU used for navigation
    bool is_primary = true;
    
    /// Timestamp in microseconds of last primary IMU update
    uint32_t last_primary_update_us;

    /**
     * @brief Rotate and correct accelerometer sample
     * 
     * @details Applies rotation matrix (sensor orientation) and calibration corrections
     *          (scale factors, offsets) to raw accelerometer sample. Called internally
     *          before publishing accel data.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in,out] accel Accelerometer vector in m/s/s (modified in place)
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (time-critical)
     */
    void _rotate_and_correct_accel(uint8_t instance, Vector3f &accel) __RAMFUNC__;
    
    /**
     * @brief Rotate and correct gyroscope sample
     * 
     * @details Applies rotation matrix (sensor orientation) and calibration corrections
     *          (offsets) to raw gyroscope sample. Called internally before publishing
     *          gyro data.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in,out] gyro Gyroscope vector in rad/s (modified in place)
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (time-critical)
     */
    void _rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro) __RAMFUNC__;

    /**
     * @brief Publish gyroscope sample to frontend
     * 
     * @details Rotates, corrects, filters, and publishes gyro data to frontend structure.
     *          This is the primary method for backends to provide gyro data. Handles:
     *          - Rotation matrix application
     *          - Calibration offset correction
     *          - Notch filter (harmonic notch)
     *          - Low-pass filter
     *          - Accumulation for delta-angle calculation
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] gyro Gyroscope vector in rad/s (body frame, uncorrected)
     * 
     * @note Call from update() or interrupt handler - marked __RAMFUNC__ for speed
     * @note Always use this method, never write directly to frontend
     */
    void _publish_gyro(uint8_t instance, const Vector3f &gyro) __RAMFUNC__; /* front end */

    /**
     * @brief Apply notch and low-pass filters to gyro sample
     * 
     * @details Runs harmonic notch filter and low-pass filter on gyro data,
     *          and collects samples for FFT analysis if enabled.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] gyro Filtered gyroscope vector in rad/s
     */
    void apply_gyro_filters(const uint8_t instance, const Vector3f &gyro);
    
    /**
     * @brief Save gyro sample in window for FFT analysis
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] gyro Gyroscope vector in rad/s
     * @param[in] phase Window phase identifier
     */
    void save_gyro_window(const uint8_t instance, const Vector3f &gyro, uint8_t phase);

    /**
     * @brief Notify backend of new raw gyro sample
     * 
     * @details Called every time a new raw gyro sample is available (published or not).
     *          Sample must already be rotated and corrected via _rotate_and_correct_gyro().
     *          Used for sample rate tracking and raw data logging.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] gyro Gyroscope vector in rad/s (rotated and corrected)
     * @param[in] sample_us Sample timestamp in microseconds (0 for FIFO sensors, 
     *                      actual timestamp for non-FIFO sensors)
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     * @note For FIFO-based sensors, set sample_us=0; for polled sensors provide timestamp
     */
    void _notify_new_gyro_raw_sample(uint8_t instance, const Vector3f &gyro, uint64_t sample_us=0) __RAMFUNC__;

    /**
     * @brief Publish integrated delta-angle sample
     * 
     * @details Alternative interface using delta-angles instead of instantaneous rates.
     *          Rotation and correction handled internally. Useful for sensors that
     *          provide integrated samples over a time interval.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] dangle Integrated rotation angle in radians (body frame, uncorrected)
     * 
     * @note Delta-angles improve coning correction for high sample rate sensors
     */
    void _notify_new_delta_angle(uint8_t instance, const Vector3f &dangle);
    
    /**
     * @brief Publish accelerometer sample to frontend
     * 
     * @details Rotates, corrects, filters, and publishes accel data to frontend structure.
     *          This is the primary method for backends to provide accel data. Handles:
     *          - Rotation matrix application
     *          - Calibration scale/offset correction
     *          - Low-pass filter
     *          - Accumulation for delta-velocity calculation
     *          - Clipping detection
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] accel Acceleration vector in m/s/s (body frame, uncorrected)
     * 
     * @note Call from update() or interrupt handler - marked __RAMFUNC__ for speed
     * @note Always use this method, never write directly to frontend
     */
    void _publish_accel(uint8_t instance, const Vector3f &accel) __RAMFUNC__; /* front end */

    /**
     * @brief Notify backend of new raw accelerometer sample
     * 
     * @details Called every time a new raw accel sample is available (published or not).
     *          Sample must already be rotated and corrected via _rotate_and_correct_accel().
     *          Used for sample rate tracking, clipping detection, and raw data logging.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] accel Acceleration vector in m/s/s (rotated and corrected)
     * @param[in] sample_us Sample timestamp in microseconds (0 for FIFO sensors,
     *                      actual timestamp for non-FIFO sensors)
     * @param[in] fsync_set True if FSYNC signal was active for this sample
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     * @note For FIFO-based sensors, set sample_us=0; for polled sensors provide timestamp
     */
    void _notify_new_accel_raw_sample(uint8_t instance, const Vector3f &accel, uint64_t sample_us=0, bool fsync_set=false) __RAMFUNC__;

    /**
     * @brief Publish integrated delta-velocity sample
     * 
     * @details Alternative interface using delta-velocities instead of instantaneous
     *          accelerations. Rotation and correction handled internally. Useful for
     *          sensors that provide integrated samples over a time interval.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] dvelocity Integrated velocity change in m/s (body frame, uncorrected)
     * 
     * @note Delta-velocities improve sculling correction for high sample rate sensors
     */
    void _notify_new_delta_velocity(uint8_t instance, const Vector3f &dvelocity);
    
    /**
     * @brief Set accelerometer oversampling factor
     * 
     * @details Indicates how many hardware samples are averaged per published sample.
     *          Used for calculating effective sample rate and filter parameters.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] n Oversampling factor (1 = no oversampling, >1 = n samples averaged)
     */
    void _set_accel_oversampling(uint8_t instance, uint8_t n);

    /**
     * @brief Set gyroscope oversampling factor
     * 
     * @details Indicates how many hardware samples are averaged per published sample.
     *          Used for calculating effective sample rate and filter parameters.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] n Oversampling factor (1 = no oversampling, >1 = n samples averaged)
     */
    void _set_gyro_oversampling(uint8_t instance, uint8_t n);

    /**
     * @brief Enable/disable sensor-rate sampling mode for accelerometer
     * 
     * @details Sensor-rate sampling provides samples at full hardware rate rather than
     *          downsampled to loop rate. Used for high-frequency analysis and logging.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] value true to enable sensor-rate sampling, false to disable
     */
    void _set_accel_sensor_rate_sampling_enabled(uint8_t instance, bool value) {
        const uint8_t bit = (1<<instance);
        if (value) {
            _imu._accel_sensor_rate_sampling_enabled |= bit;
        } else {
            _imu._accel_sensor_rate_sampling_enabled &= ~bit;
        }
    }

    /**
     * @brief Enable/disable sensor-rate sampling mode for gyroscope
     * 
     * @details Sensor-rate sampling provides samples at full hardware rate rather than
     *          downsampled to loop rate. Used for high-frequency analysis and FFT.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] value true to enable sensor-rate sampling, false to disable
     */
    void _set_gyro_sensor_rate_sampling_enabled(uint8_t instance, bool value) {
        const uint8_t bit = (1<<instance);
        if (value) {
            _imu._gyro_sensor_rate_sampling_enabled |= bit;
        } else {
            _imu._gyro_sensor_rate_sampling_enabled &= ~bit;
        }
    }

    /**
     * @brief Set raw sample multiplier for accelerometer
     * 
     * @details Used when backend provides samples at a multiple of the base rate.
     *          Frontend uses this for accurate timing calculations.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] mul Sample rate multiplier (typically 1, 2, 4, or 8)
     */
    void _set_raw_sample_accel_multiplier(uint8_t instance, uint16_t mul) {
        _imu._accel_raw_sampling_multiplier[instance] = mul;
    }
    
    /**
     * @brief Set raw sample multiplier for gyroscope
     * 
     * @details Used when backend provides samples at a multiple of the base rate.
     *          Frontend uses this for accurate timing calculations.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] mul Sample rate multiplier (typically 1, 2, 4, or 8)
     */
    void _set_raw_sample_gyro_multiplier(uint8_t instance, uint16_t mul) {
        _imu._gyro_raw_sampling_multiplier[instance] = mul;
    }

    /**
     * @brief Update observed sensor sample rate for FIFO sensors
     * 
     * @details Tracks actual sensor sample rate by measuring time between FIFO reads.
     *          Used to detect rate changes and adjust filtering accordingly.
     * 
     * @param[in,out] count Sample count accumulator
     * @param[in,out] start_us Rate measurement start time in microseconds
     * @param[out] rate_hz Calculated sample rate in Hz
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void _update_sensor_rate(uint16_t &count, uint32_t &start_us, float &rate_hz) const __RAMFUNC__;

    /**
     * @brief Check if sensor rates are still converging
     * 
     * @details Returns true during initial startup period when sensor sample rates
     *          may still be stabilizing. Used to defer filter initialization.
     * 
     * @return true if rates still converging, false if stable
     */
    bool sensors_converging() const;

    /**
     * @brief Get accelerometer raw sample rate
     * 
     * @param[in] instance Accelerometer instance number
     * @return Sample rate in Hz
     * 
     * @note Returns float even though rate is typically integer
     */
    float _accel_raw_sample_rate(uint8_t instance) const {
        return _imu._accel_raw_sample_rates[instance];
    }

    /**
     * @brief Set accelerometer raw sample rate
     * 
     * @details Sets the raw sample rate for this accelerometer instance. Called by
     *          backend during initialization to report hardware sampling rate.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] rate_hz Sample rate in Hz
     * 
     * @note Storage type is float to support fractional rates
     */
    void _set_accel_raw_sample_rate(uint8_t instance, uint16_t rate_hz) {
        _imu._accel_raw_sample_rates[instance] = rate_hz;
    }
    
    /**
     * @brief Get gyroscope raw sample rate
     * 
     * @param[in] instance Gyroscope instance number
     * @return Sample rate in Hz
     * 
     * @note Returns float even though rate is typically integer
     */
    float _gyro_raw_sample_rate(uint8_t instance) const {
        return _imu._gyro_raw_sample_rates[instance];
    }

    /**
     * @brief Set gyroscope raw sample rate
     * 
     * @details Sets the raw sample rate for this gyroscope instance. Called by
     *          backend during initialization to report hardware sampling rate.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] rate_hz Sample rate in Hz
     * 
     * @note Storage type is float to support fractional rates
     */
    void _set_gyro_raw_sample_rate(uint8_t instance, uint16_t rate_hz) {
        _imu._gyro_raw_sample_rates[instance] = rate_hz;
    }
    
    /**
     * @brief Publish temperature reading
     * 
     * @details Reports sensor die temperature for health monitoring and
     *          temperature-dependent calibration corrections.
     * 
     * @param[in] instance Sensor instance number (gyro or accel)
     * @param[in] temperature Temperature in degrees Celsius
     */
    void _publish_temperature(uint8_t instance, float temperature); /* front end */

    /**
     * @brief Increment accelerometer error counter
     * 
     * @details Called when sensor read error occurs. Error count tracked per instance
     *          for health monitoring and failover decisions.
     * 
     * @param[in] instance Accelerometer instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     */
    void _inc_accel_error_count(uint8_t instance) __RAMFUNC__;

    /**
     * @brief Increment gyroscope error counter
     * 
     * @details Called when sensor read error occurs. Error count tracked per instance
     *          for health monitoring and failover decisions.
     * 
     * @param[in] instance Gyroscope instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     */
    void _inc_gyro_error_count(uint8_t instance) __RAMFUNC__;
    
    /// Backend unique identifier (-1 if backend doesn't identify itself)
    int16_t _id = -1;

    /**
     * @brief Get accelerometer low-pass filter cutoff frequency
     * 
     * @return Filter cutoff in Hz (from INS_ACCEL_FILTER parameter)
     */
    uint16_t _accel_filter_cutoff(void) const { return _imu._accel_filter_cutoff; }

    /**
     * @brief Get gyroscope low-pass filter cutoff frequency
     * 
     * @return Filter cutoff in Hz (from INS_GYRO_FILTER parameter)
     */
    uint16_t _gyro_filter_cutoff(void) const { return _imu._gyro_filter_cutoff; }

    /**
     * @brief Get vehicle loop rate
     * 
     * @details Returns the rate at which samples will be made available to vehicle
     *          code. Sensors running faster than this are downsampled.
     * 
     * @return Loop rate in Hz (50, 100, 200, 250, 400 Hz depending on vehicle type)
     */
    uint16_t get_loop_rate_hz(void) const {
        // enum can be directly cast to Hz
        return (uint16_t)_imu._loop_rate;
    }

    /**
     * @brief Common gyro update function for all backends
     * 
     * @details Downsamples gyro data from sensor rate to loop rate, applies filters.
     *          Called by _publish_gyro().
     * 
     * @param[in] instance Gyroscope instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void update_gyro(uint8_t instance) __RAMFUNC__; /* front end */
    
    /**
     * @brief Update gyro filter configuration at runtime
     * 
     * @param[in] instance Gyroscope instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void update_gyro_filters(uint8_t instance) __RAMFUNC__; /* front end */

    /**
     * @brief Common accelerometer update function for all backends
     * 
     * @details Downsamples accel data from sensor rate to loop rate, applies filters.
     *          Called by _publish_accel().
     * 
     * @param[in] instance Accelerometer instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void update_accel(uint8_t instance) __RAMFUNC__; /* front end */
    
    /**
     * @brief Update accelerometer filter configuration at runtime
     * 
     * @param[in] instance Accelerometer instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void update_accel_filters(uint8_t instance) __RAMFUNC__; /* front end */

    /**
     * @brief Update primary IMU tracking
     * 
     * @details Called when primary IMU changes. Backends can override set_primary()
     *          to handle primary status changes.
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void update_primary() __RAMFUNC__; /* backend */
    
    /**
     * @brief Notification that primary status changed
     * 
     * @param[in] _is_primary true if this backend is now primary, false otherwise
     * 
     * @note Override to handle primary status changes (e.g., adjust sample rates)
     */
    virtual void set_primary(bool _is_primary) {}

    /// Last configured accelerometer filter cutoff (for detecting runtime changes)
    uint16_t _last_accel_filter_hz;
    
    /// Last configured gyroscope filter cutoff (for detecting runtime changes)
    uint16_t _last_gyro_filter_hz;

    /**
     * @brief Set gyroscope sensor orientation
     * 
     * @details Specifies rotation matrix to apply to raw gyro readings to align with
     *          vehicle frame. Set during backend initialization based on board hwdef.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] rotation Rotation enum (ROTATION_NONE, ROTATION_YAW_90, etc.)
     */
    void set_gyro_orientation(uint8_t instance, enum Rotation rotation) {
        _imu._gyro_orientation[instance] = rotation;
    }

    /**
     * @brief Set accelerometer sensor orientation
     * 
     * @details Specifies rotation matrix to apply to raw accel readings to align with
     *          vehicle frame. Set during backend initialization based on board hwdef.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] rotation Rotation enum (ROTATION_NONE, ROTATION_YAW_90, etc.)
     */
    void set_accel_orientation(uint8_t instance, enum Rotation rotation) {
        _imu._accel_orientation[instance] = rotation;
    }

    /**
     * @brief Get gyroscope instance number
     * 
     * @return Instance number assigned by frontend during registration
     */
    uint8_t get_gyro_instance() const {
        return gyro_instance;
    }

    /**
     * @brief Get accelerometer instance number
     * 
     * @return Instance number assigned by frontend during registration
     */
    uint8_t get_accel_instance() const {
        return accel_instance;
    }

    /**
     * @brief Increment accelerometer clipping count
     * 
     * @details Used by drivers that do decimation before supplying samples to the
     *          frontend. Clipping indicates samples exceeded sensor range.
     * 
     * @param[in] instance Accelerometer instance number
     * 
     * @note Clipping count used for vibration monitoring and health checks
     */
    void increment_clip_count(uint8_t instance) {
        _imu._accel_clip_count[instance]++;
    }

    /**
     * @brief Check if fast sampling should be enabled for this IMU
     * 
     * @details Fast sampling runs IMU at higher rate (typically 2-8 kHz) for improved
     *          filter performance and aliasing reduction.
     * 
     * @param[in] instance IMU instance number
     * @return true if INS_FAST_SAMPLE parameter enables this instance, false otherwise
     */
    bool enable_fast_sampling(uint8_t instance) const {
        return (_imu._fast_sampling_mask & (1U<<instance)) != 0;
    }

    /**
     * @brief Check if high-resolution sampling should be enabled for this IMU
     * 
     * @details High-res sampling preserves full sensor precision for specialized
     *          applications (FFT analysis, modal identification).
     * 
     * @param[in] instance IMU instance number
     * @return true if HAL_INS_HIGHRES_SAMPLE compile flag enables this instance
     */
    bool enable_highres_sampling(uint8_t instance) const {
        return (HAL_INS_HIGHRES_SAMPLE & (1U<<instance)) != 0;
    }

    /**
     * @brief Get fast sampling rate multiplier
     * 
     * @return Fast sampling rate in kHz (1, 2, 4, or 8 kHz depending on INS_FAST_SAMPLE)
     * 
     * @note Only meaningful if enable_fast_sampling() returns true
     */
    uint8_t get_fast_sampling_rate() const {
        return (1 << uint8_t(_imu._fast_sampling_rate));
    }

    /**
     * @brief Notify frontend of new accelerometer sample at sensor rate
     * 
     * @details Called by subclass when data is received from the sensor hardware at
     *          full sensor rate (before downsampling). Used for sample rate tracking.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] accel Accelerometer reading in m/s² (NED body frame)
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     */
    void _notify_new_accel_sensor_rate_sample(uint8_t instance, const Vector3f &accel) __RAMFUNC__;
    
    /**
     * @brief Notify frontend of new gyroscope sample at sensor rate
     * 
     * @details Called by subclass when data is received from the sensor hardware at
     *          full sensor rate (before downsampling). Used for sample rate tracking.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] gyro Gyroscope reading in rad/s (NED body frame)
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     */
    void _notify_new_gyro_sensor_rate_sample(uint8_t instance, const Vector3f &gyro) __RAMFUNC__;

    /**
     * @brief Notify that accelerometer FIFO was reset
     * 
     * @details Prevents bad data from being used to update observed sensor rate after
     *          FIFO overflow or other reset event.
     * 
     * @param[in] instance Accelerometer instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     */
    void notify_accel_fifo_reset(uint8_t instance) __RAMFUNC__;
    
    /**
     * @brief Notify that gyroscope FIFO was reset
     * 
     * @details Prevents bad data from being used to update observed sensor rate after
     *          FIFO overflow or other reset event.
     * 
     * @param[in] instance Gyroscope instance number
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     */
    void notify_gyro_fifo_reset(uint8_t instance) __RAMFUNC__;

    /**
     * @brief Log unexpected IMU register change
     * 
     * @details Logs when safety-critical IMU register changes unexpectedly, indicating
     *          possible hardware fault, radiation event, or communication error.
     * 
     * @param[in] bus_id Unique bus identifier for this sensor
     * @param[in] reg Register check structure with address and expected/actual values
     * 
     * @note Marked __RAMFUNC__ for execution from RAM (may be called from interrupt)
     * @warning Register changes can indicate sensor failure requiring immediate attention
     */
    void log_register_change(uint32_t bus_id, const AP_HAL::Device::checkreg &reg) __RAMFUNC__;

    /**
     * @note Each backend subclass is expected to implement a static detect() function
     *       which probes the bus (SPI/I2C) to detect the sensor and instantiates an
     *       instance of the backend driver if the sensor is found.
     * 
     * @par Example detect() signature:
     * @code
     * static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu,
     *                                           AP_HAL::OwnPtr<AP_HAL::Device> dev);
     * @endcode
     */

private:

    /**
     * @brief Check if raw IMU logging is enabled
     * 
     * @return true if LOG_BITMASK includes IMU_RAW logging, false otherwise
     */
    bool should_log_imu_raw() const ;
    
    /**
     * @brief Log raw accelerometer sample
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] sample_us Timestamp in microseconds
     * @param[in] accel Accelerometer reading in m/s²
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void log_accel_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &accel) __RAMFUNC__;
    
    /**
     * @brief Log raw gyroscope sample
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] sample_us Timestamp in microseconds
     * @param[in] raw_gyro Raw gyroscope reading in rad/s (before filtering)
     * @param[in] filtered_gyro Filtered gyroscope reading in rad/s
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void log_gyro_raw(uint8_t instance, const uint64_t sample_us, const Vector3f &raw_gyro, const Vector3f &filtered_gyro) __RAMFUNC__;

    /**
     * @brief Write accelerometer data packet to log
     * 
     * @details Writes ACC log message with raw accelerometer data for post-flight analysis.
     * 
     * @param[in] instance Accelerometer instance number
     * @param[in] sample_us Timestamp in microseconds
     * @param[in] accel Accelerometer reading in m/s²
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void Write_ACC(const uint8_t instance, const uint64_t sample_us, const Vector3f &accel) const __RAMFUNC__;

protected:
    /**
     * @brief Write gyroscope data packet to log
     * 
     * @details Writes GYR log message with raw gyroscope data for post-flight analysis.
     * 
     * @param[in] instance Gyroscope instance number
     * @param[in] sample_us Timestamp in microseconds
     * @param[in] gyro Gyroscope reading in rad/s
     * @param[in] use_sample_timestamp If true, use sample_us for timestamp; otherwise use current time
     * 
     * @note Marked __RAMFUNC__ for execution from RAM
     */
    void Write_GYR(const uint8_t instance, const uint64_t sample_us, const Vector3f &gyro, bool use_sample_timestamp=false) const __RAMFUNC__;

};
