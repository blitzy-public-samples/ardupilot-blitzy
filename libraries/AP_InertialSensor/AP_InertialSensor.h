#pragma once

#include "AP_InertialSensor_config.h"

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ     5.0f    // accel vibration floor filter hz
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ           2.0f    // accel vibration filter hz
#define AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS 500     // peak-hold detector timeout

#include <AP_HAL/AP_HAL_Boards.h>

#include <stdint.h>

#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <Filter/LowPassFilter.h>
#include <Filter/HarmonicNotchFilter.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include "AP_InertialSensor_Params.h"
#include "AP_InertialSensor_tempcal.h"

#ifndef AP_SIM_INS_ENABLED
#define AP_SIM_INS_ENABLED AP_SIM_ENABLED
#endif

#ifndef AP_SIM_INS_FILE_ENABLED
#define AP_SIM_INS_FILE_ENABLED AP_SIM_ENABLED
#endif

class AP_InertialSensor_Backend;
class AuxiliaryBus;
class AP_AHRS;
class FastRateBuffer;

/*
  forward declare AP_Logger class. We can't include logger.h
  because of mutual dependencies
 */
class AP_Logger;

/**
 * @file AP_InertialSensor.h
 * @brief ArduPilot IMU frontend - central interface for all inertial sensors
 * 
 * Defines the main AP_InertialSensor class (frontend) that vehicle code uses to access
 * gyroscope and accelerometer data. Manages multiple IMU backends, sensor fusion,
 * calibration, filtering, and health monitoring.
 * 
 * Source: libraries/AP_InertialSensor/AP_InertialSensor.h
 */

/**
 * @class AP_InertialSensor
 * @brief Central IMU management frontend supporting multiple sensor backends
 * 
 * @details Comprehensive IMU subsystem providing:
 *          - **Multi-IMU support**: Up to INS_MAX_INSTANCES (default 3) independent IMUs
 *          - **Backend management**: Auto-detection and initialization of sensor drivers
 *          - **Sensor fusion**: Blending data from multiple IMUs for redundancy
 *          - **Health monitoring**: Per-sensor health flags and consistency checking
 *          - **Calibration**: 6-point accel cal, gyro bias estimation, temp compensation
 *          - **Filtering**: Harmonic notch, low-pass, vibration monitoring
 *          - **Delta angle/velocity**: Integrated samples for precise attitude estimation
 *          - **Logging**: Binary log messages for post-flight analysis
 *          - **Fast sample buffer**: High-rate gyro samples for FFT analysis
 *          
 *          Subsystem Architecture:
 *          ```
 *          Vehicle Code (ArduCopter, ArduPlane)
 *                      ↓
 *          AP_InertialSensor (Frontend) ← Singleton via AP::ins()
 *           ├─ IMU 1: AP_InertialSensor_Backend (e.g., MPU6000)
 *           ├─ IMU 2: AP_InertialSensor_Backend (e.g., ICM20689)
 *           └─ IMU 3: AP_InertialSensor_Backend (e.g., BMI088)
 *          ```
 *          
 *          Initialization Sequence:
 *          1. **Probe phase**: Backends detect sensors via I2C/SPI bus scan
 *          2. **Registration**: Each backend registers gyro/accel instances
 *          3. **Calibration**: Gyro bias cal, accel calibration if requested
 *          4. **Start**: Backends configure sensors and enable FIFO/interrupts
 *          5. **Update loop**: periodic() triggers backend updates at main loop rate
 *          
 *          Sensor Data Flow:
 *          1. Backend reads raw sensor data (FIFO burst or register poll)
 *          2. Backend calls _publish_gyro/_publish_accel with raw samples
 *          3. Frontend applies rotation, calibration offsets/scales
 *          4. Frontend applies notch and low-pass filters
 *          5. Frontend accumulates delta angle/velocity integrals
 *          6. Vehicle code calls get_gyro/get_accel to retrieve filtered data
 *          
 *          Calibration System:
 *          - **Gyro**: Automatic bias estimation during initialization (vehicle stationary)
 *          - **Accel**: 6-point calibration via AP_AccelCal (user rotates vehicle)
 *          - **Temperature**: Optional polynomial compensation (HAL_INS_TEMPERATURE_CAL_ENABLE)
 *          - **Factory trim**: Loaded from persistent storage (AP_Param)
 *          
 *          Harmonic Notch Filtering:
 *          - Rejects propeller/motor vibration at fundamental + harmonics
 *          - Center frequency tracks throttle or ESC RPM telemetry
 *          - Multiple notch filters (INS_HNTCH_OPTS) for multi-motor vehicles
 *          - FFT-based frequency tracking (HAL_GYROFFT_ENABLED)
 *          
 *          Health Monitoring:
 *          - Per-sensor health flags (gyro_healthy, accel_healthy)
 *          - Consistency checking between IMUs (gyros_consistent, accels_consistent)
 *          - Clipping detection (saturation indicates mechanical overload)
 *          - Vibration metrics (RMS accel vibration levels per axis)
 *          
 *          Fast Sample Window (HAL_GYROFFT_ENABLED):
 *          - Circular buffer of high-rate gyro samples (typically 1-2kHz)
 *          - Used by AP_GyroFFT for real-time frequency analysis
 *          - Enables dynamic notch filter frequency tracking
 *          - Window size configurable via INS_FAST_SAMPLE parameter
 * 
 * @note Singleton access via AP::ins() - never construct directly
 * @note All gyro data in body frame, units: rad/s
 * @note All accel data in body frame, units: m/s²
 * @note Always check healthy() before trusting sensor data
 * 
 * @warning Incorrect calibration causes navigation errors - always calibrate before flight
 * @warning Harmonic notch misconfiguration can destabilize flight - tune carefully
 * @warning Filter settings (INS_GYRO_FILTER, INS_ACCEL_FILTER) affect control loop performance
 * 
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_InertialSensor : AP_AccelCal_Client
{
    friend class AP_InertialSensor_Backend;
    friend class FastRateBuffer;

public:
    /**
     * @brief Constructor - initializes IMU frontend state
     * 
     * @note Do not call directly - use AP::ins() singleton accessor
     */
    AP_InertialSensor();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_InertialSensor);

    /**
     * @brief Get singleton instance of AP_InertialSensor
     * 
     * @return Pointer to the singleton AP_InertialSensor instance
     * @note Thread-safe singleton access
     */
    static AP_InertialSensor *get_singleton();

    /**
     * @enum Gyro_Calibration_Timing
     * @brief Controls when gyroscope calibration is performed
     */
    enum Gyro_Calibration_Timing {
        GYRO_CAL_NEVER = 0,         ///< Never perform gyro calibration (use saved offsets)
        GYRO_CAL_STARTUP_ONLY = 1   ///< Calibrate gyros only at startup (default)
    };

    /**
     * @brief Perform startup initialization of IMU subsystem
     * 
     * @details Initializes the state of all IMUs:
     *          1. Detects and initializes all backend sensor drivers
     *          2. Performs gyro calibration (unless INS_GYRO_CAL is GYRO_CAL_NEVER)
     *          3. Loads accelerometer calibration from persistent storage
     *          4. Configures filtering (harmonic notch, low-pass)
     *          5. Starts sensor sampling at specified rate
     *          
     *          Gyro calibration requires vehicle to be stationary. Frontend accumulates
     *          samples and calculates bias offsets for each gyro axis.
     * 
     * @param[in] sample_rate_hz Desired main loop rate in Hz (e.g., 400 for copters)
     * 
     * @note Must be called once at system startup before using IMU data
     * @note Gyros calibrated unless INS_GYRO_CAL parameter is set to GYRO_CAL_NEVER
     * @note Blocking call - waits for gyro calibration to complete
     * 
     * @warning Vehicle must be completely stationary during gyro calibration
     * @warning Do not arm or attempt flight until init() completes successfully
     */
    __INITFUNC__ void init(uint16_t sample_rate_hz);

    /**
     * @brief Get the next available accelerometer instance number
     * 
     * @param[out] instance Next available accel instance number (0-based)
     * @return true if an instance is available, false if all instances allocated
     * 
     * @note Used by backends during registration phase
     */
    bool get_accel_instance(uint8_t &instance) const;

    /**
     * @brief Get the next available gyroscope instance number
     * 
     * @param[out] instance Next available gyro instance number (0-based)
     * @return true if an instance is available, false if all instances allocated
     * 
     * @note Used by backends during registration phase
     */
    bool get_gyro_instance(uint8_t &instance) const;

    /**
     * @brief Register a new gyroscope driver backend
     * 
     * @details Allocates an instance number for a new gyro sensor. Called by
     *          backend drivers during probe/initialization. Stores sensor ID for
     *          persistent identification across reboots.
     * 
     * @param[out] instance Allocated gyro instance number (0 to INS_MAX_INSTANCES-1)
     * @param[in] raw_sample_rate_hz Native sensor sample rate in Hz (e.g., 1000 for MPU6000)
     * @param[in] raw_sample_rate_hz Native sensor sample rate in Hz
     * @param[in] id Unique sensor identifier (e.g., I2C address + chip ID)
     * 
     * @return true if registration successful, false if no instances available
     * 
     * @note Backend must call this before publishing any gyro data
     * @note Sensor ID used to match calibration data across parameter changes
     */
    bool register_gyro(uint8_t &instance, uint16_t raw_sample_rate_hz, uint32_t id);

    /**
     * @brief Register a new accelerometer driver backend
     * 
     * @details Allocates an instance number for a new accel sensor. Called by
     *          backend drivers during probe/initialization. Stores sensor ID for
     *          persistent identification across reboots.
     * 
     * @param[out] instance Allocated accel instance number (0 to INS_MAX_INSTANCES-1)
     * @param[in] raw_sample_rate_hz Native sensor sample rate in Hz (e.g., 1000 for MPU6000)
     * @param[in] id Unique sensor identifier (e.g., I2C address + chip ID)
     * 
     * @return true if registration successful, false if no instances available
     * 
     * @note Backend must call this before publishing any accel data
     * @note Sensor ID used to match calibration data across parameter changes
     */
    bool register_accel(uint8_t &instance, uint16_t raw_sample_rate_hz, uint32_t id);

    /**
     * @brief Main loop periodic update function
     * 
     * @details Called by vehicle code at main loop rate (typically 400Hz for copters).
     *          Triggers all backend updates, processes new samples, applies filtering,
     *          and updates health status. Must be called regularly to maintain fresh data.
     * 
     * @note Call at main loop rate to ensure timely sensor updates
     * @note Backends publish data asynchronously; periodic() processes queued samples
     */
    void periodic();

    /**
     * @brief Check if gyro or accel calibration is currently in progress
     * 
     * @return true if any calibration is running (gyro bias cal or accel cal), false otherwise
     * 
     * @note Use to prevent arming or flight during calibration
     * @note Does not include temperature calibration - see temperature_cal_running()
     */
    bool calibrating() const;

    /**
     * @brief Check if temperature calibration is currently running
     * 
     * @return true if temperature calibration active, false otherwise
     * 
     * @note Temperature calibration requires HAL_INS_TEMPERATURE_CAL_ENABLE
     * @note Separate from gyro/accel calibration status
     */
    bool temperature_cal_running() const;
    
    /**
     * @brief Perform cold-start initialization for gyros only
     * 
     * @details Re-calibrates gyro biases without reinitializing entire IMU subsystem.
     *          Used for in-flight gyro recalibration or after parameter changes.
     *          Vehicle must be stationary during calibration.
     * 
     * @note Must call init() first - this does not perform full initialization
     * @note Blocking call - waits for gyro calibration to complete
     * 
     * @warning Vehicle must be completely stationary during calibration
     * @warning Incorrect calibration causes attitude estimation errors
     */
    void init_gyro(void);

    /**
     * @brief Get startup banner message for a specific IMU instance
     * 
     * @param[in] instance_id IMU instance number (0-based)
     * @param[out] banner Character buffer to receive banner text
     * @param[in] banner_len Maximum length of banner buffer
     * 
     * @return true if banner generated successfully, false if instance invalid
     * 
     * @note Used by vehicle code to report detected IMUs to ground station
     * @note Banner includes sensor type, bus info, and sample rate
     */
    bool get_output_banner(uint8_t instance_id, char* banner, uint8_t banner_len);

    /**
     * @brief Get current filtered gyroscope data for specified instance
     * 
     * @param[in] i Gyro instance number (0 to get_gyro_count()-1)
     * 
     * @return Vector3f containing filtered rotational rates in body frame (rad/s)
     *         Components: [roll_rate, pitch_rate, yaw_rate]
     * 
     * @note Data is filtered with low-pass and harmonic notch filters
     * @note Body frame: X forward, Y right, Z down
     * @note Units: radians per second
     * @note Check get_gyro_health(i) before trusting data
     */
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }

    /**
     * @brief Get current filtered gyroscope data from primary gyro
     * 
     * @return Vector3f containing filtered rotational rates in body frame (rad/s)
     * 
     * @note Uses first usable/healthy gyro (typically instance 0)
     * @note Check healthy() before trusting data
     */
    const Vector3f     &get_gyro(void) const { return get_gyro(_first_usable_gyro); }

    /**
     * @brief Get calibration offsets for specified gyro instance
     * 
     * @param[in] i Gyro instance number
     * 
     * @return Vector3f containing gyro bias offsets in rad/s
     * 
     * @note Offsets are subtracted from raw sensor data during processing
     * @note Calculated during gyro calibration at startup
     */
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset(i); }

    /**
     * @brief Get calibration offsets for primary gyro
     * 
     * @return Vector3f containing gyro bias offsets in rad/s
     */
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_first_usable_gyro); }

    /**
     * @brief Get integrated delta angle for specified gyro instance
     * 
     * @details Delta angle is the integrated rotational displacement since last
     *          retrieval. Used by EKF for precise attitude integration. More accurate
     *          than multiplying instantaneous rate by dt.
     * 
     * @param[in] i Gyro instance number
     * @param[out] delta_angle Integrated rotation vector (radians)
     * @param[out] delta_angle_dt Integration time period (seconds)
     * 
     * @return true if delta angle available, false if backend doesn't support it
     * 
     * @note Not all backends provide delta angles (requires FIFO or DMA)
     * @note Delta angle integrates samples between update() calls
     * @note Preferred over rate*dt for attitude estimation accuracy
     */
    bool get_delta_angle(uint8_t i, Vector3f &delta_angle, float &delta_angle_dt) const;

    /**
     * @brief Get integrated delta angle from primary gyro
     * 
     * @param[out] delta_angle Integrated rotation vector (radians)
     * @param[out] delta_angle_dt Integration time period (seconds)
     * 
     * @return true if delta angle available, false otherwise
     */
    bool get_delta_angle(Vector3f &delta_angle, float &delta_angle_dt) const {
        return get_delta_angle(_first_usable_gyro, delta_angle, delta_angle_dt);
    }

    /**
     * @brief Get integrated delta velocity for specified accel instance
     * 
     * @details Delta velocity is the integrated velocity change since last retrieval.
     *          Used by EKF for precise velocity/position integration. More accurate
     *          than multiplying instantaneous accel by dt.
     * 
     * @param[in] i Accel instance number
     * @param[out] delta_velocity Integrated velocity change vector (m/s)
     * @param[out] delta_velocity_dt Integration time period (seconds)
     * 
     * @return true if delta velocity available, false if backend doesn't support it
     * 
     * @note Not all backends provide delta velocity (requires FIFO or DMA)
     * @note Delta velocity integrates samples between update() calls
     * @note Preferred over accel*dt for navigation accuracy
     */
    bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity, float &delta_velocity_dt) const;

    /**
     * @brief Get integrated delta velocity from primary accelerometer
     * 
     * @param[out] delta_velocity Integrated velocity change vector (m/s)
     * @param[out] delta_velocity_dt Integration time period (seconds)
     * 
     * @return true if delta velocity available, false otherwise
     */
    bool get_delta_velocity(Vector3f &delta_velocity, float &delta_velocity_dt) const {
        return get_delta_velocity(_first_usable_accel, delta_velocity, delta_velocity_dt);
    }

    /**
     * @brief Get current filtered accelerometer data for specified instance
     * 
     * @param[in] i Accel instance number (0 to get_accel_count()-1)
     * 
     * @return Vector3f containing filtered accelerations in body frame (m/s²)
     *         Components: [accel_x, accel_y, accel_z]
     * 
     * @note Data is filtered with low-pass filters
     * @note Body frame: X forward, Y right, Z down
     * @note Units: meters per second squared
     * @note Includes gravity - stationary vehicle reads [0, 0, +9.81] m/s²
     * @note Check get_accel_health(i) before trusting data
     */
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }

    /**
     * @brief Get current filtered accelerometer data from primary accelerometer
     * 
     * @return Vector3f containing filtered accelerations in body frame (m/s²)
     * 
     * @note Uses first usable/healthy accel (typically instance 0)
     * @note Check healthy() before trusting data
     */
    const Vector3f     &get_accel(void) const { return get_accel(_first_usable_accel); }

    /**
     * @brief Check health status of specified gyro instance
     * 
     * @param[in] instance Gyro instance number
     * 
     * @return true if gyro is healthy and providing valid data, false otherwise
     * 
     * @note Health checks: sensor initialized, data fresh, no excessive errors
     * @note False if instance number invalid or sensor failed
     */
    bool get_gyro_health(uint8_t instance) const { return (instance<_gyro_count) ? _gyro_healthy[instance] : false; }

    /**
     * @brief Check health status of primary gyro
     * 
     * @return true if primary gyro healthy, false otherwise
     */
    bool get_gyro_health(void) const { return get_gyro_health(_first_usable_gyro); }

    /**
     * @brief Check if all gyros are healthy
     * 
     * @return true if all registered gyros are healthy, false if any unhealthy
     * 
     * @note Used for pre-arm checks and redundancy validation
     */
    bool get_gyro_health_all(void) const;

    /**
     * @brief Check consistency between multiple gyros
     * 
     * @details Compares gyro readings across all instances. Large differences
     *          indicate sensor failure, mounting issues, or calibration problems.
     * 
     * @param[in] threshold Maximum acceptable difference in deg/s
     * 
     * @return true if all gyros agree within threshold, false otherwise
     * 
     * @note Used for pre-arm checks with multiple IMUs
     * @note Typical threshold: 0.3 deg/s for quads, higher for larger vehicles
     */
    bool gyros_consistent(uint8_t threshold) const;

    /**
     * @brief Get total number of registered gyros
     * 
     * @return Number of gyro instances (0 to INS_MAX_INSTANCES)
     * 
     * @note Count includes all detected gyros, healthy or unhealthy
     */
    uint8_t get_gyro_count(void) const { return MIN(INS_MAX_INSTANCES, _gyro_count); }

    /**
     * @brief Check if specified gyro has valid calibration
     * 
     * @param[in] instance Gyro instance number
     * 
     * @return true if gyro calibration complete and valid, false otherwise
     * 
     * @note Calibration must be performed before flight
     */
    bool gyro_calibrated_ok(uint8_t instance) const { return _gyro_cal_ok[instance]; }

    /**
     * @brief Check if all gyros have valid calibration
     * 
     * @return true if all gyros calibrated, false otherwise
     * 
     * @note Used for pre-arm checks
     */
    bool gyro_calibrated_ok_all() const;

    /**
     * @brief Check if specified gyro instance should be used
     * 
     * @param[in] instance Gyro instance number
     * 
     * @return true if gyro enabled and should be used, false if disabled
     * 
     * @note Respects INS_USE parameter and INS_ENABLE_MASK
     */
    bool use_gyro(uint8_t instance) const;

    /**
     * @brief Get configured gyro calibration timing
     * 
     * @return Gyro_Calibration_Timing enum value (GYRO_CAL_NEVER or GYRO_CAL_STARTUP_ONLY)
     * 
     * @note Controlled by INS_GYRO_CAL parameter
     */
    Gyro_Calibration_Timing gyro_calibration_timing();

    /**
     * @brief Check health status of specified accelerometer instance
     * 
     * @param[in] instance Accel instance number
     * 
     * @return true if accelerometer is healthy and providing valid data, false otherwise
     * 
     * @note Health checks: sensor initialized, data fresh, no excessive errors
     * @note False if instance number invalid or sensor failed
     */
    bool get_accel_health(uint8_t instance) const { return (instance<_accel_count) ? _accel_healthy[instance] : false; }

    /**
     * @brief Check health status of primary accelerometer
     * 
     * @return true if primary accel healthy, false otherwise
     */
    bool get_accel_health(void) const { return get_accel_health(_first_usable_accel); }

    /**
     * @brief Check if all accelerometers are healthy
     * 
     * @return true if all registered accels are healthy, false if any unhealthy
     * 
     * @note Used for pre-arm checks and redundancy validation
     */
    bool get_accel_health_all(void) const;

    /**
     * @brief Check consistency between multiple accelerometers
     * 
     * @details Compares accel readings across all instances. Large differences
     *          indicate sensor failure, mounting issues, or calibration problems.
     * 
     * @param[in] accel_error_threshold Maximum acceptable difference in m/s²
     * 
     * @return true if all accels agree within threshold, false otherwise
     * 
     * @note Used for pre-arm checks with multiple IMUs
     * @note Typical threshold: 1.0 m/s² (about 0.1g)
     */
    bool accels_consistent(float accel_error_threshold) const;

    /**
     * @brief Get total number of registered accelerometers
     * 
     * @return Number of accel instances (0 to INS_MAX_INSTANCES)
     * 
     * @note Count includes all detected accels, healthy or unhealthy
     */
    uint8_t get_accel_count(void) const { return MIN(INS_MAX_INSTANCES, _accel_count); }

    /**
     * @brief Check if all accelerometers have valid calibration
     * 
     * @return true if all accels calibrated, false otherwise
     * 
     * @note Used for pre-arm checks
     * @note Accel calibration required for accurate attitude estimation
     */
    bool accel_calibrated_ok_all() const;

    /**
     * @brief Check if specified accel instance should be used
     * 
     * @param[in] instance Accel instance number
     * 
     * @return true if accel enabled and should be used, false if disabled
     * 
     * @note Respects INS_USE parameter and INS_ENABLE_MASK
     */
    bool use_accel(uint8_t instance) const;

    /**
     * @brief Get effective sample rate for specified gyro instance
     * 
     * @param[in] instance Gyro instance number
     * 
     * @return Effective sample rate in Hz (raw rate × oversampling)
     * 
     * @note Includes internal oversampling multiplier if backend uses it
     * @note Used to configure EKF timing and filter frequencies
     */
    uint16_t get_gyro_rate_hz(uint8_t instance) const { return uint16_t(_gyro_raw_sample_rates[instance] * _gyro_over_sampling[instance]); }

    /**
     * @brief Get effective sample rate for specified accel instance
     * 
     * @param[in] instance Accel instance number
     * 
     * @return Effective sample rate in Hz (raw rate × oversampling)
     * 
     * @note Includes internal oversampling multiplier if backend uses it
     */
    uint16_t get_accel_rate_hz(uint8_t instance) const { return uint16_t(_accel_raw_sample_rates[instance] * _accel_over_sampling[instance]); }

    /**
     * @brief Validate gyro backend sample rates for pre-arm check
     * 
     * @param[out] fail_msg Buffer for failure message if check fails
     * @param[in] fail_msg_len Maximum length of fail_msg buffer
     * 
     * @return true if all gyro sample rates valid, false with message if mismatch detected
     * 
     * @note Checks that all gyros report expected sample rates
     * @note Used during pre-arm checks to detect sensor configuration issues
     */
    bool pre_arm_check_gyro_backend_rate_hz(char* fail_msg, uint16_t fail_msg_len) const;

    // FFT support access
#if HAL_GYROFFT_ENABLED
    /**
     * @brief Get gyro data for FFT analysis
     * 
     * @return Vector3f containing thread-safe gyro data for FFT processing (rad/s)
     * 
     * @note Used by AP_GyroFFT for frequency analysis and dynamic notch tuning
     * @note Thread-safe copy separate from main control loop data
     */
    const Vector3f& get_gyro_for_fft(void) const { return _gyro_for_fft[_first_usable_gyro]; }

    /**
     * @brief Get raw gyro sample window buffer for FFT
     * 
     * @param[in] instance Gyro instance number
     * @param[in] axis Axis index (0=X, 1=Y, 2=Z)
     * 
     * @return Reference to FloatBuffer containing high-rate gyro samples
     * 
     * @note Window contains raw samples at sensor rate (typically 1-2kHz)
     * @note Used by AP_GyroFFT for spectrum analysis
     * @note Buffer is circular with INS_FAST_SAMPLE samples
     */
    FloatBuffer&  get_raw_gyro_window(uint8_t instance, uint8_t axis) { return _gyro_window[instance][axis]; }

    /**
     * @brief Get raw gyro sample window buffer for primary gyro
     * 
     * @param[in] axis Axis index (0=X, 1=Y, 2=Z)
     * 
     * @return Reference to FloatBuffer containing high-rate gyro samples
     */
    FloatBuffer&  get_raw_gyro_window(uint8_t axis) { return get_raw_gyro_window(_first_usable_gyro, axis); }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    /**
     * @brief Check if FFT-based harmonic notch is enabled
     * 
     * @return true if FFT notch configured, false otherwise
     * 
     * @note FFT notch dynamically tracks vibration frequencies
     */
    bool has_fft_notch() const;
#endif
#endif
    /**
     * @brief Get raw sample rate for specified gyro instance
     * 
     * @param[in] instance Gyro instance number
     * 
     * @return Native sensor sample rate in Hz (before oversampling)
     * 
     * @note This is the hardware FIFO rate, not effective rate
     */
    uint16_t get_raw_gyro_rate_hz(uint8_t instance) const { return _gyro_raw_sample_rates[_first_usable_gyro]; }

    /**
     * @brief Get raw sample rate for primary gyro
     * 
     * @return Native sensor sample rate in Hz
     */
    uint16_t get_raw_gyro_rate_hz() const { return get_raw_gyro_rate_hz(_first_usable_gyro); }

    /**
     * @brief Set size of gyro sample window for FFT
     * 
     * @param[in] size Window size in samples (typically 256-1024)
     * 
     * @return true if window size set successfully, false if allocation failed
     * 
     * @note Larger windows provide better frequency resolution
     * @note Window size limited by available RAM
     * @note Must be power of 2 for efficient FFT
     */
    bool set_gyro_window_size(uint16_t size);
    /**
     * @brief Get accelerometer calibration offsets
     * 
     * @param[in] i Accel instance number
     * 
     * @return Vector3f containing accel bias offsets in m/s²
     * 
     * @note Offsets subtracted from raw sensor data during processing
     * @note Set during 6-point accelerometer calibration
     */
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset(i); }

    /**
     * @brief Get accelerometer calibration offsets for primary accel
     * 
     * @return Vector3f containing accel bias offsets in m/s²
     */
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(_first_usable_accel); }

    /**
     * @brief Get accelerometer scale factors
     * 
     * @param[in] i Accel instance number
     * 
     * @return Vector3f containing scale factors for each axis (typically near 1.0)
     * 
     * @note Scale factors applied after offset removal
     * @note Compensates for sensor gain errors
     * @note Set during 6-point accelerometer calibration
     */
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale(i); }

    /**
     * @brief Get accelerometer scale factors for primary accel
     * 
     * @return Vector3f containing scale factors
     */
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(_first_usable_accel); }

    /**
     * @brief Get IMU position offset from vehicle center of gravity
     * 
     * @param[in] instance IMU instance number
     * 
     * @return Vector3f position offset in body frame (meters)
     *         Components: [X_forward, Y_right, Z_down]
     * 
     * @note Used by EKF to compensate for lever arm effects
     * @note Important for vehicles with offset IMUs (large planes, helicopters)
     * @note Set via INS_POS_X, INS_POS_Y, INS_POS_Z parameters
     */
    const Vector3f &get_imu_pos_offset(uint8_t instance) const {
        return _accel_pos(instance);
    }

    /**
     * @brief Get IMU position offset for primary IMU
     * 
     * @return Vector3f position offset in body frame (meters)
     */
    const Vector3f &get_imu_pos_offset(void) const {
        return _accel_pos(_first_usable_accel);
    }

    /**
     * @brief Get IMU temperature for specified instance
     * 
     * @param[in] instance IMU instance number
     * 
     * @return Temperature in degrees Celsius, or 0.0 if not supported
     * 
     * @note Not all IMU chips provide temperature measurement
     * @note Used for temperature calibration compensation
     */
    float get_temperature(uint8_t instance) const { return _temperature[instance]; }

    /**
     * @brief Get time period over which sensor data was collected
     * 
     * @return Delta time in seconds for most recent update
     * 
     * @note Capped at _loop_delta_t_max to prevent instability from timing glitches
     * @note Used for integration of gyro/accel data
     */
    float get_delta_time() const { return MIN(_delta_time, _loop_delta_t_max); }

    /**
     * @brief Get maximum expected gyro drift rate
     * 
     * @return Maximum drift rate in rad/s/s
     * 
     * @note Conservative estimate: 0.5 deg/min/min = 0.00014 rad/s/s
     * @note Used by EKF for gyro bias estimation
     * @note Depends on gyro chip characteristics (MEMS vs fiber optic)
     */
    float get_gyro_drift_rate(void) const { return radians(0.5f/60); }

    /**
     * @brief Update gyro and accel values from accumulated samples
     * 
     * @details Main data processing function called at loop rate:
     *          1. Retrieves accumulated samples from backends
     *          2. Applies calibration offsets and scales
     *          3. Applies rotation for board orientation
     *          4. Applies low-pass and notch filters
     *          5. Accumulates delta angle/velocity integrals
     *          6. Updates health status flags
     * 
     * @note Called from main scheduler loop (typically 400Hz for copters)
     * @note Must be called regularly for fresh data
     * @note __RAMFUNC__ for time-critical execution from RAM
     */
    void update(void) __RAMFUNC__;

    /**
     * @brief Wait for a new sample to be available from any IMU
     * 
     * @details Blocks until new data available from gyro/accel or timeout.
     *          Used to synchronize main loop with sensor sampling.
     * 
     * @note Blocking call - waits for sensor interrupt or polling interval
     * @note Returns when any enabled IMU has new data
     * @note __RAMFUNC__ for time-critical execution from RAM
     */
    void wait_for_sample(void) __RAMFUNC__;

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];
#if INS_AUX_INSTANCES
    AP_InertialSensor_Params params[INS_AUX_INSTANCES];
#endif

    /**
     * @brief Set overall board orientation
     * 
     * @param[in] orientation Rotation enum (ROTATION_NONE, ROTATION_YAW_90, etc.)
     * 
     * @note Applies rotation to all IMU data to align with vehicle frame
     * @note Must match physical board mounting (set via AHRS_ORIENTATION)
     * @note Board frame rotated to vehicle frame: X forward, Y right, Z down
     */
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    /**
     * @brief Get configured main loop rate
     * 
     * @return Loop rate in Hz (typically 400 for copters, 50-400 for planes)
     * 
     * @note Set during init() based on vehicle type
     * @note Determines sensor data update frequency
     */
    uint16_t get_loop_rate_hz(void) const { return _loop_rate; }

    /**
     * @brief Get main loop delta time
     * 
     * @return Loop period in seconds (1.0 / loop_rate_hz)
     * 
     * @note Used for control loop dt calculations
     * @note Nominal value - actual dt may vary slightly
     */
    float get_loop_delta_t(void) const { return _loop_delta_t; }

    /**
     * @brief Check if IMU subsystem is healthy overall
     * 
     * @return true if primary gyro AND primary accel both healthy, false otherwise
     * 
     * @note Most conservative health check - both sensors must be good
     * @note Use before trusting any IMU data
     * @note For individual sensor health, use get_gyro_health() or get_accel_health()
     */
    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    /**
     * @brief Get first usable accelerometer instance number
     * 
     * @return Accel instance number (0-based)
     * 
     * @note This is the primary accel used by default accessors
     * @note Selected based on INS_USE parameter and health status
     */
    uint8_t get_first_usable_accel(void) const { return _first_usable_accel; }

    /**
     * @brief Get first usable gyroscope instance number
     * 
     * @return Gyro instance number (0-based)
     * 
     * @note This is the primary gyro used by default accessors
     * @note Selected based on INS_USE parameter and health status
     */
    uint8_t get_first_usable_gyro(void) const { return _first_usable_gyro; }

    /**
     * @brief Get gyro low-pass filter cutoff frequency
     * 
     * @return Filter cutoff in Hz
     * 
     * @note Set via INS_GYRO_FILTER parameter (0 = use default)
     * @note Lower frequencies reduce noise but increase phase lag
     * @note Typical: 80-100Hz for copters, 20-40Hz for planes
     * 
     * @warning Too low causes control loop instability due to phase lag
     */
    uint16_t get_gyro_filter_hz(void) const { return _gyro_filter_cutoff; }

    /**
     * @brief Get accel low-pass filter cutoff frequency
     * 
     * @return Filter cutoff in Hz
     * 
     * @note Set via INS_ACCEL_FILTER parameter (0 = use default)
     * @note Lower frequencies reduce vibration noise
     * @note Typical: 20-30Hz for most vehicles
     */
    uint16_t get_accel_filter_hz(void) const { return _accel_filter_cutoff; }

#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    /**
     * @brief Setup harmonic notch filter for throttle-based frequency tracking
     * 
     * @param[in] center_freq_hz Base frequency in Hz (typically hover throttle RPM)
     * @param[in] lower_freq_hz Minimum tracking frequency in Hz
     * @param[in] ref Reference throttle value (0.0-1.0, typically hover throttle)
     * @param[in] harmonics Bitmask of harmonics to filter (bit 0 = fundamental)
     * 
     * @return true if setup successful, false on error
     * 
     * @details Configures dynamic notch filter that tracks motor/prop frequencies
     *          based on throttle position. Frequency = center_freq * (throttle/ref).
     * 
     * @note Used for vehicles without ESC RPM telemetry
     * @note For multi-motor vehicles, set INS_HNTCH_OPTS for per-motor tracking
     * @note Harmonics typically include 1x, 2x for props
     * 
     * @warning Incorrect center_freq causes poor vibration rejection
     */
    bool setup_throttle_gyro_harmonic_notch(float center_freq_hz, float lower_freq_hz, float ref, uint8_t harmonics);

    /**
     * @brief Write harmonic notch filter status to binary log
     * 
     * @note Logs current notch center frequencies for tuning analysis
     * @note Called periodically during flight
     */
    void write_notch_log_messages() const;
#endif

    /**
     * @brief Set LOG_BITMASK bit number for raw IMU data logging
     * 
     * @param[in] log_raw_bit Bit position in LOG_BITMASK parameter
     * 
     * @note Enables high-rate raw gyro/accel logging when bit is set
     * @note Used for FFT analysis and filter tuning
     */
    void set_log_raw_bit(uint32_t log_raw_bit) { _log_raw_bit = log_raw_bit; }

    /**
     * @brief Write filtered IMU data to binary log
     * 
     * @details Logs IMU messages containing:
     *          - Filtered gyro rates (rad/s)
     *          - Filtered accelerations (m/s²)
     *          - Delta angles and velocities
     *          - Timestamps
     * 
     * @note Called at main loop rate when logging enabled
     * @note Creates IMU, IMU2, IMU3 messages for multiple sensors
     */
    void Write_IMU() const;

    /**
     * @brief Write vibration metrics to binary log
     * 
     * @details Logs VIBE message containing:
     *          - RMS vibration levels per axis (m/s²)
     *          - Clipping counts
     *          - Vibration health indicators
     * 
     * @note Called periodically (typically 1Hz)
     * @note Used for mechanical diagnostics and PID tuning
     */
    void Write_Vibration() const;

    /**
     * @brief Calculate vibration levels and detect accelerometer clipping
     * 
     * @param[in] instance Accel instance number
     * @param[in] accel Raw acceleration vector in m/s²
     * @param[in] dt Time delta in seconds
     * 
     * @details Updates running vibration statistics:
     *          - Low-pass filtered RMS vibration per axis
     *          - Peak detection for clipping events
     *          - Vibration floor estimation
     * 
     * @note Called by backends for each raw sample
     * @note Clipping detected when accel exceeds sensor full-scale range
     * @note High vibration degrades altitude hold and position control
     */
    void calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt);

    /**
     * @brief Get latest vibration levels for primary accel
     * 
     * @return Vector3f containing RMS vibration per axis in m/s²
     * 
     * @note Low-pass filtered vibration magnitude
     * @note Typical good values: <15 m/s² per axis
     * @note Values >30 m/s² indicate mechanical issues
     */
    Vector3f get_vibration_levels() const { return get_vibration_levels(_first_usable_accel); }

    /**
     * @brief Get latest vibration levels for specified accel
     * 
     * @param[in] instance Accel instance number
     * 
     * @return Vector3f containing RMS vibration per axis in m/s²
     */
    Vector3f get_vibration_levels(uint8_t instance) const;

    /**
     * @brief Get and clear accelerometer clipping event count
     * 
     * @param[in] instance Accel instance number
     * 
     * @return Number of clipping events since last call
     * 
     * @note Clipping indicates sensor saturation (exceeded ±16g typically)
     * @note Causes brief loss of accurate accel data
     * @note Frequent clipping requires mechanical dampening improvement
     */
    uint32_t get_accel_clip_count(uint8_t instance) const;

    /**
     * @brief Check if vehicle is stationary (minimal vibration on all axes)
     * 
     * @return true if vibration levels below threshold on all axes
     * 
     * @note Used for gyro calibration checks
     * @note Threshold set via INS_STILL_THRESH parameter
     * @note Typical threshold: 0.05 m/s² RMS
     */
    bool is_still();

    /**
     * @brief Get auxiliary I2C bus for attaching external sensors to IMU
     * 
     * @param[in] backend_id Backend identifier
     * 
     * @return Pointer to AuxiliaryBus interface, or nullptr if not available
     * 
     * @note Some IMUs (e.g., MPU9250) provide auxiliary I2C master for external sensors
     * @note Used to attach magnetometers, barometers to IMU's I2C bus
     */
    AuxiliaryBus *get_auxiliary_bus(int16_t backend_id) { return get_auxiliary_bus(backend_id, 0); }

    /**
     * @brief Get auxiliary I2C bus for specific backend instance
     * 
     * @param[in] backend_id Backend identifier
     * @param[in] instance Backend instance number
     * 
     * @return Pointer to AuxiliaryBus interface, or nullptr if not available
     */
    AuxiliaryBus *get_auxiliary_bus(int16_t backend_id, uint8_t instance);

    /**
     * @brief Detect and initialize all available IMU backend drivers
     * 
     * @details Probes I2C/SPI buses for supported IMU chips:
     *          - Invensense (MPU6000, MPU9250, ICM20xxx)
     *          - Bosch (BMI088, BMI270)
     *          - ST (LSM9DS1)
     *          - Platform-specific (Raspberry Pi, BBB)
     * 
     * @note Called during init() to auto-detect hardware
     * @note Detection order and enabled sensors controlled by INS_ENABLE_MASK
     */
    void detect_backends(void);

    /**
     * @brief Update all backend sensor drivers
     * 
     * @note Triggers backend update() calls
     * @note Internal function - use update() for main loop
     */
    void update_backends();

    /**
     * @brief Update peak negative X acceleration detector
     * 
     * @param[in] instance Accel instance number
     * @param[in] accel Current acceleration vector in m/s²
     * 
     * @details Peak-hold detector for landing detection:
     *          - Tracks maximum negative X (forward deceleration)
     *          - Decays over AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS
     * 
     * @note Used for detecting hard landings and crashes
     */
    void set_accel_peak_hold(uint8_t instance, const Vector3f &accel);

    /**
     * @brief Get peak negative X acceleration value
     * 
     * @return Maximum negative X accel in m/s² (positive value = deceleration magnitude)
     * 
     * @note Used for landing detection in flight modes
     * @note Decays with timeout if no new peaks detected
     */
    float get_accel_peak_hold_neg_x() const { return _peak_hold_state.accel_peak_hold_neg_x; }

    /**
     * @brief Get accelerometer calibration interface object
     * 
     * @return Pointer to AP_AccelCal object managing calibration process
     * 
     * @note Used by GCS to start/monitor accel calibration
     * @note Implements 6-point calibration algorithm
     */
    AP_AccelCal* get_acal() const { return _acal; }

    /**
     * @brief Get averaged accelerometer data from specific calibration sample
     * 
     * @param[in] sample_num Sample number (0-5 for 6-point cal)
     * @param[out] ret Averaged acceleration vector for that orientation
     * 
     * @return true if sample data available, false otherwise
     * 
     * @note Used during accel calibration to retrieve orientation samples
     * @note Each sample represents vehicle held steady in one orientation
     */
    bool get_fixed_mount_accel_cal_sample(uint8_t sample_num, Vector3f& ret) const;

    /**
     * @brief Get averaged primary accel data from calibration sample
     * 
     * @param[in] sample_num Sample number (0-5 for 6-point cal)
     * @param[out] ret Averaged acceleration vector
     * 
     * @return true if sample data available, false otherwise
     */
    bool get_first_usable_accel_cal_sample_avg(uint8_t sample_num, Vector3f& ret) const;

    /**
     * @brief Retrieve newly calculated trim (level calibration) values
     * 
     * @param[out] trim_rad Trim angles in radians [roll, pitch, yaw]
     * 
     * @return true if new trim available, false otherwise
     * 
     * @note Trim compensates for board mounting errors (not perfectly level)
     * @note Calculated during accel calibration first step
     * @note Vehicle must be level during trim calibration
     */
    bool get_new_trim(Vector3f &trim_rad);

    /**
     * @brief Notify IMU subsystem of primary sensor change
     * 
     * @param[in] instance New primary IMU instance number
     * 
     * @note Updates filter states and internal tracking
     * @note Called by EKF when switching primary IMU
     */
    void set_primary(uint8_t instance);

#if HAL_INS_ACCELCAL_ENABLED
    /**
     * @brief Initialize and register accelerometer calibrator
     * 
     * @details Sets up AP_AccelCal object for 6-point calibration:
     *          - Allocates AccelCalibrator instances for each IMU
     *          - Registers calibration callbacks
     *          - Prepares for GCS-commanded calibration
     * 
     * @note Called automatically during accel calibration startup
     */
    void acal_init();

    /**
     * @brief Update accelerometer calibrator state machine
     * 
     * @details Processes calibration steps:
     *          - Collects samples for each orientation
     *          - Runs Gauss-Newton solver when all samples collected
     *          - Updates calibration parameters on success
     * 
     * @note Called periodically during active calibration
     */
    void acal_update();
#endif

#if HAL_GCS_ENABLED
    /**
     * @brief Perform gyroscope calibration
     * 
     * @return true if calibration successful, false on timeout or error
     * 
     * @details Gyro calibration procedure:
     *          1. Vehicle must be stationary (checked via is_still())
     *          2. Collect samples for several seconds
     *          3. Calculate average bias (offset) for each gyro
     *          4. Store bias to persistent parameters
     * 
     * @note Blocks for ~10 seconds during calibration
     * @note Vehicle must remain completely still
     * @note Automatically called on startup if INS_GYRO_CAL=1
     * 
     * @warning Moving vehicle during cal causes navigation errors
     */
    bool calibrate_gyros();

    /**
     * @brief Calibrate vehicle level trim (board mounting compensation)
     * 
     * @return MAV_RESULT indicating success/failure for MAVLink response
     * 
     * @details Trim calibration:
     *          - Vehicle must be level on ground
     *          - Measures deviation from expected 1g down
     *          - Calculates roll/pitch offset angles
     *          - Stores to AHRS_TRIM parameters
     * 
     * @note Quick calibration (~5 seconds)
     * @note Vehicle must be on level surface
     * @note Compensates for non-level board mounting only
     */
    MAV_RESULT calibrate_trim();

    /**
     * @brief Perform simple accelerometer calibration (level only)
     * 
     * @return MAV_RESULT indicating success/failure for MAVLink response
     * 
     * @details Simple 1-point accel calibration:
     *          - Vehicle level on ground
     *          - Measures Z-axis (should be -1g)
     *          - Updates accel offsets
     * 
     * @note Less accurate than full 6-point calibration
     * @note Use for quick field calibration
     * @note Does not calibrate X/Y axes or scale factors
     */
    MAV_RESULT simple_accel_cal();
private:
    uint32_t last_accel_cal_ms; ///< Timestamp of last accel cal for rate limiting
public:
#endif

    /**
     * @brief Check if accel calibration requires vehicle reboot
     * 
     * @return true if reboot needed to apply new calibration
     * 
     * @note Some calibration changes require restart to reinitialize filters
     */
    bool accel_cal_requires_reboot() const { return _accel_cal_requires_reboot; }

    /**
     * @brief Get timestamp of most recent update() call
     * 
     * @return Time in microseconds (system uptime)
     * 
     * @note Used to check data freshness
     * @note Compare with AP_HAL::micros() to detect stale data
     */
    uint32_t get_last_update_usec(void) const { return _last_update_usec; }

    /**
     * @brief Disable specific IMU for testing/debugging
     * 
     * @param[in] imu_idx IMU instance number to affect
     * @param[in] kill_it true to disable, false to re-enable
     * 
     * @note FOR TESTING ONLY - simulates IMU failure
     * @note Used to test multi-IMU redundancy and EKF lane switching
     * 
     * @warning Never use in flight - causes navigation degradation
     */
    void kill_imu(uint8_t imu_idx, bool kill_it);

#if AP_SERIALMANAGER_IMUOUT_ENABLED
    /**
     * @brief Configure UART for streaming raw IMU data to external process
     * 
     * @param[in] uart Pointer to configured UART driver
     * 
     * @note Used for external processing (e.g., external EKF, data logging)
     * @note Sends high-rate gyro/accel samples in binary format
     * @note Configured via SERIAL_PROTOCOL = IMU_Out
     */
    void set_imu_out_uart(AP_HAL::UARTDriver *uart);

    /**
     * @brief Send buffered IMU data over configured UART
     * 
     * @note Called periodically to stream data
     * @note Binary format for low overhead
     */
    void send_uart_data(void);

    struct {
        uint16_t counter; ///< Packet sequence counter
        AP_HAL::UARTDriver *imu_out_uart; ///< UART driver for IMU output
    } uart;
#endif // AP_SERIALMANAGER_IMUOUT_ENABLED

    /**
     * @enum IMU_SENSOR_TYPE
     * @brief Identifies sensor type for batch sampling
     */
    enum IMU_SENSOR_TYPE {
        IMU_SENSOR_TYPE_ACCEL = 0, ///< Accelerometer sensor
        IMU_SENSOR_TYPE_GYRO = 1,  ///< Gyroscope sensor
    };

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    class BatchSampler {
    public:
        BatchSampler(const AP_InertialSensor &imu) :
            type(IMU_SENSOR_TYPE_ACCEL),
            _imu(imu) {
            AP_Param::setup_object_defaults(this, var_info);
        };

        void init();
        void sample(uint8_t instance, IMU_SENSOR_TYPE _type, uint64_t sample_us, const Vector3f &sample) __RAMFUNC__;

        // a function called by the main thread at the main loop rate:
        void periodic();

        bool doing_sensor_rate_logging() const { return _doing_sensor_rate_logging; }
        bool doing_post_filter_logging() const {
            return (_doing_post_filter_logging && (post_filter || !_doing_sensor_rate_logging))
                || (_doing_pre_post_filter_logging && post_filter);
        }

        // Getters for arming check
        bool is_initialised() const { return initialised; }
        bool enabled() const { return _sensor_mask > 0; }

        // class level parameters
        static const struct AP_Param::GroupInfo var_info[];
    

        // Parameters
        AP_Int16 _required_count;
        uint16_t _real_required_count;
        AP_Int8 _sensor_mask;
        AP_Int8 _batch_options_mask;

        // Parameters controlling pushing data to AP_Logger:
        // Each DF message is ~ 108 bytes in size, so we use about 1kB/s of
        // logging bandwidth with a 100ms interval.  If we are taking
        // 1024 samples then we need to send 32 packets, so it will
        // take ~3 seconds to push a complete batch to the log.  If
        // you are running a on an FMU with three IMUs then you
        // will loop back around to the first sensor after about
        // twenty seconds.
        AP_Int16 samples_per_msg;
        AP_Int8 push_interval_ms;

        // end Parameters

    private:

        enum batch_opt_t {
            BATCH_OPT_SENSOR_RATE = (1<<0),
            BATCH_OPT_POST_FILTER = (1<<1),
            BATCH_OPT_PRE_POST_FILTER = (1<<2),
        };

        void rotate_to_next_sensor();
        void update_doing_sensor_rate_logging();

        bool should_log(uint8_t instance, IMU_SENSOR_TYPE type) __RAMFUNC__;
        void push_data_to_log();

        // Logging functions
        bool Write_ISBH(const float sample_rate_hz) const;
        bool Write_ISBD() const;

        bool has_option(batch_opt_t option) const { return _batch_options_mask & uint16_t(option); }

        uint64_t measurement_started_us;

        bool initialised;
        bool isbh_sent;
        bool _doing_sensor_rate_logging;
        bool _doing_post_filter_logging;
        bool _doing_pre_post_filter_logging;
        uint8_t instance; // instance we are sending data for
        bool post_filter; // whether we are sending post-filter data
        AP_InertialSensor::IMU_SENSOR_TYPE type;
        uint16_t isb_seqnum;
        int16_t *data_x;
        int16_t *data_y;
        int16_t *data_z;
        uint16_t data_write_offset; // units: samples
        uint16_t data_read_offset; // units: samples
        uint32_t last_sent_ms;

        // all samples are multiplied by this
        uint16_t multiplier; // initialised as part of init()

        const AP_InertialSensor &_imu;
    };
    BatchSampler batchsampler{*this};
#endif

#if AP_EXTERNAL_AHRS_ENABLED
    /**
     * @brief Process IMU data from external AHRS system
     * 
     * @param[in] pkt IMU data packet from external AHRS (e.g., VectorNav, LORD)
     * 
     * @details Integrates external IMU into ArduPilot sensor fusion:
     *          - Creates virtual IMU backend with external data
     *          - Bypasses normal sensor drivers
     *          - Used for high-end external INS systems
     * 
     * @note External AHRS provides pre-filtered gyro/accel data
     * @note May also include attitude, velocity, position estimates
     */
    void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt);
#endif

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    /**
     * @brief Get temperature calibration parameters for firmware migration
     * 
     * @param[out] str Expanding string to append parameter list
     * 
     * @details Extracts temperature calibration coefficients:
     *          - Polynomial coefficients for gyro temp compensation
     *          - Polynomial coefficients for accel temp compensation
     *          - Temperature at which calibration was performed
     * 
     * @note Used when switching between vehicle firmware types (Copter↔Plane)
     * @note Preserves hard-earned temperature calibration data
     * @note Formatted as parameter name=value pairs
     */
    void get_persistent_params(ExpandingString &str) const;
#endif

    /**
     * @brief Force save of current calibration as valid without full cal process
     * 
     * @details Emergency calibration save:
     *          - Writes current sensor values as calibration baseline
     *          - Bypasses normal calibration validation
     *          - Used for factory calibration or emergency recovery
     * 
     * @warning Use with extreme caution - can save bad calibration
     * @warning Vehicle must be level and stationary
     * @note FOR DEVELOPMENT/FACTORY USE ONLY
     */
    void force_save_calibration(void);

#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    // structure per harmonic notch filter. This is public to allow for
    // easy iteration
    class HarmonicNotch {
    public:
        HarmonicNotchFilterParams params;
        HarmonicNotchFilterVector3f filter[INS_MAX_INSTANCES];

        uint8_t num_dynamic_notches;

        // the current center frequency for the notch
        float calculated_notch_freq_hz[INS_MAX_NOTCHES];
        uint8_t num_calculated_notch_frequencies;

        // runtime update of notch parameters
        void update_params(uint8_t instance, bool converging, float gyro_rate);

        // Update the harmonic notch frequencies
        void update_freq_hz(float scaled_freq);
        void update_frequencies_hz(uint8_t num_freqs, const float scaled_freq[]);

        // enable/disable the notch
        void set_inactive(bool _inactive) {
            inactive = _inactive;
        }

        bool is_inactive(void) const {
            return inactive;
        }

    private:
        // support for updating harmonic filter at runtime
        float last_center_freq_hz[INS_MAX_INSTANCES];
        float last_bandwidth_hz[INS_MAX_INSTANCES];
        float last_attenuation_dB[INS_MAX_INSTANCES];
        bool inactive;
    } harmonic_notches[HAL_INS_NUM_HARMONIC_NOTCH_FILTERS];
#endif  // AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED

private:
    // load backend drivers
    bool _add_backend(AP_InertialSensor_Backend *backend);
    void _start_backends();
    AP_InertialSensor_Backend *_find_backend(int16_t backend_id, uint8_t instance);

    // gyro initialisation
    void _init_gyro();

    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    bool _calculate_trim(const Vector3f &accel_sample, Vector3f &trim_rad);

    // save gyro calibration values to eeprom
    void _save_gyro_calibration();

    // Logging function
    void Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance) const;
    
    // backend objects
    AP_InertialSensor_Backend *_backends[INS_MAX_BACKENDS]; ///< Array of registered sensor backend drivers

    // number of gyros and accel drivers. Note that most backends
    // provide both accel and gyro data, so will increment both
    // counters on initialisation
    uint8_t _gyro_count; ///< Number of registered gyroscopes (0 to INS_MAX_INSTANCES)
    uint8_t _accel_count; ///< Number of registered accelerometers (0 to INS_MAX_INSTANCES)
    uint8_t _backend_count; ///< Number of registered backend drivers

    // the selected loop rate at which samples are made available
    uint16_t _loop_rate; ///< Main loop rate in Hz (set during init, typically 400Hz copter, 50-400Hz plane)
    float _loop_delta_t; ///< Loop period in seconds (1.0 / _loop_rate)
    float _loop_delta_t_max; ///< Maximum allowed loop delta time for clamping

    // Most recent accelerometer reading
    Vector3f _accel[INS_MAX_INSTANCES]; ///< Latest filtered accel in m/s² (body frame, after rotation/cal/filtering)
    Vector3f _delta_velocity[INS_MAX_INSTANCES]; ///< Integrated delta velocity in m/s (for EKF)
    float _delta_velocity_dt[INS_MAX_INSTANCES]; ///< Time span of delta velocity integration in seconds
    bool _delta_velocity_valid[INS_MAX_INSTANCES]; ///< True if delta velocity available from backend
    // delta velocity accumulator
    Vector3f _delta_velocity_acc[INS_MAX_INSTANCES]; ///< Delta velocity accumulator for building integration period
    // time accumulator for delta velocity accumulator
    float _delta_velocity_acc_dt[INS_MAX_INSTANCES]; ///< Time accumulator for delta velocity in seconds

    // Low Pass filters for gyro and accel
    LowPassFilter2pVector3f _accel_filter[INS_MAX_INSTANCES]; ///< 2-pole low-pass filter for accel (INS_ACCEL_FILTER)
    LowPassFilter2pVector3f _gyro_filter[INS_MAX_INSTANCES]; ///< 2-pole low-pass filter for gyro (INS_GYRO_FILTER)
    Vector3f _accel_filtered[INS_MAX_INSTANCES]; ///< Intermediate filtered accel values
    Vector3f _gyro_filtered[INS_MAX_INSTANCES]; ///< Intermediate filtered gyro values
#if HAL_GYROFFT_ENABLED
    // Thread-safe public version of _last_raw_gyro
    Vector3f _gyro_for_fft[INS_MAX_INSTANCES]; ///< Thread-safe gyro copy for FFT analysis (rad/s)
    Vector3f _last_gyro_for_fft[INS_MAX_INSTANCES]; ///< Previous gyro for FFT to detect changes
    FloatBuffer _gyro_window[INS_MAX_INSTANCES][XYZ_AXIS_COUNT]; ///< Circular buffers for gyro FFT window per axis
    uint16_t _gyro_window_size; ///< Size of FFT window in samples (INS_FAST_SAMPLE)
    // capture a gyro window after the filters
    LowPassFilter2pVector3f _post_filter_gyro_filter[INS_MAX_INSTANCES]; ///< Optional post-filter for FFT analysis
    bool _post_filter_fft; ///< True if FFT uses post-filtered gyro data
    uint8_t _fft_window_phase; ///< Current phase of FFT window filling (0-3)
#endif
    bool _new_accel_data[INS_MAX_INSTANCES]; ///< True if new accel sample available since last update()
    bool _new_gyro_data[INS_MAX_INSTANCES]; ///< True if new gyro sample available since last update()

    // Most recent gyro reading
    Vector3f _gyro[INS_MAX_INSTANCES]; ///< Latest filtered gyro in rad/s (body frame, after rotation/cal/filtering)
    Vector3f _delta_angle[INS_MAX_INSTANCES]; ///< Integrated delta angle in radians (for EKF)
    float _delta_angle_dt[INS_MAX_INSTANCES]; ///< Time span of delta angle integration in seconds
    bool _delta_angle_valid[INS_MAX_INSTANCES]; ///< True if delta angle available from backend
    // time accumulator for delta angle accumulator
    float _delta_angle_acc_dt[INS_MAX_INSTANCES]; ///< Time accumulator for delta angle in seconds
    Vector3f _delta_angle_acc[INS_MAX_INSTANCES]; ///< Delta angle accumulator for building integration period
    Vector3f _last_delta_angle[INS_MAX_INSTANCES]; ///< Previous delta angle for consistency checking
    Vector3f _last_raw_gyro[INS_MAX_INSTANCES]; ///< Last raw gyro sample before filtering (rad/s)

    // bitmask indicating if a sensor is doing sensor-rate sampling:
    uint8_t _accel_sensor_rate_sampling_enabled; ///< Bitmask of accels logging at sensor rate (bit N = instance N)
    uint8_t _gyro_sensor_rate_sampling_enabled; ///< Bitmask of gyros logging at sensor rate (bit N = instance N)

    // multipliers for data supplied via sensor-rate logging:
    uint16_t _accel_raw_sampling_multiplier[INS_MAX_INSTANCES]; ///< Oversampling factor for raw accel logging
    uint16_t _gyro_raw_sampling_multiplier[INS_MAX_INSTANCES]; ///< Oversampling factor for raw gyro logging

    // IDs to uniquely identify each sensor: shall remain
    // the same across reboots
    AP_Int32 _accel_id_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES]; ///< Persistent accel sensor IDs (INS_ACCxID parameters)
    AP_Int32 _gyro_id_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES]; ///< Persistent gyro sensor IDs (INS_GYRxID parameters)

    // accelerometer scaling and offsets
    AP_Vector3f _accel_scale_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES]; ///< Accel scale factors from calibration (INS_ACCxSCAL)
    AP_Vector3f _accel_offset_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES]; ///< Accel offsets in m/s² from calibration (INS_ACCxOFF)
    AP_Vector3f _gyro_offset_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES]; ///< Gyro offsets in rad/s from calibration (INS_GYRxOFF)

    // accelerometer position offset in body frame
    AP_Vector3f _accel_pos_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES]; ///< IMU position offset in meters from CG (INS_POS_x) for centrifugal correction

    // Use Accessor methods to access above variables
#if INS_AUX_INSTANCES
    #define INS_PARAM_WRAPPER(var) \
        inline decltype(var##_old_param[0])& var(uint8_t i) { \
            if (i<(INS_MAX_INSTANCES-INS_AUX_INSTANCES)) { \
                return var##_old_param[i]; \
            } else { \
                return params[i-(INS_MAX_INSTANCES-INS_AUX_INSTANCES)].var; \
            } \
        } \
        inline decltype(var##_old_param[0])& var(uint8_t i) const { \
            return const_cast<AP_InertialSensor*>(this)->var(i); \
        }
#else
    #define INS_PARAM_WRAPPER(var) \
        inline decltype(var##_old_param[0])& var(uint8_t i) { \
            return var##_old_param[i]; \
        } \
        inline decltype(var##_old_param[0])& var(uint8_t i) const { \
            return const_cast<AP_InertialSensor*>(this)->var(i); \
        }
#endif

    // Accessor methods for old parameters
    INS_PARAM_WRAPPER(_accel_id);
    INS_PARAM_WRAPPER(_gyro_id);
    INS_PARAM_WRAPPER(_accel_scale);
    INS_PARAM_WRAPPER(_accel_offset);
    INS_PARAM_WRAPPER(_gyro_offset);
    INS_PARAM_WRAPPER(_accel_pos);

    // accelerometer and gyro raw sample rate in units of Hz
    float  _accel_raw_sample_rates[INS_MAX_INSTANCES]; ///< Backend raw accel sample rate in Hz (e.g., 1000Hz for MPU6000)
    float  _gyro_raw_sample_rates[INS_MAX_INSTANCES]; ///< Backend raw gyro sample rate in Hz (e.g., 1000Hz for MPU6000)

    // how many sensors samples per notify to the backend
    uint8_t _accel_over_sampling[INS_MAX_INSTANCES]; ///< Accel oversampling factor (raw_rate / loop_rate)
    uint8_t _gyro_over_sampling[INS_MAX_INSTANCES]; ///< Gyro oversampling factor (raw_rate / loop_rate)

    // last sample time in microseconds. Use for deltaT calculations
    // on non-FIFO sensors
    uint64_t _accel_last_sample_us[INS_MAX_INSTANCES]; ///< Timestamp of last accel sample in microseconds
    uint64_t _gyro_last_sample_us[INS_MAX_INSTANCES]; ///< Timestamp of last gyro sample in microseconds

    // sample times for checking real sensor rate for FIFO sensors
    uint16_t _sample_accel_count[INS_MAX_INSTANCES]; ///< Count of accel samples for rate validation
    uint32_t _sample_accel_start_us[INS_MAX_INSTANCES]; ///< Start timestamp for accel rate measurement
    uint16_t _sample_gyro_count[INS_MAX_INSTANCES]; ///< Count of gyro samples for rate validation
    uint32_t _sample_gyro_start_us[INS_MAX_INSTANCES]; ///< Start timestamp for gyro rate measurement
    
    // temperatures for an instance if available
    float _temperature[INS_MAX_INSTANCES]; ///< IMU die temperature in Celsius (for temperature calibration)

    // filtering frequency (0 means default)
    AP_Int16    _accel_filter_cutoff; ///< INS_ACCEL_FILTER parameter - accel LPF cutoff in Hz (0=default 20Hz)
    AP_Int16    _gyro_filter_cutoff; ///< INS_GYRO_FILTER parameter - gyro LPF cutoff in Hz (0=default 80Hz)
    AP_Int8     _gyro_cal_timing; ///< INS_GYRO_CAL parameter - when to calibrate gyros (0=never, 1=startup)

    // use for attitude, velocity, position estimates
    AP_Int8     _use_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES]; ///< INS_USE parameter - enable IMU for EKF (0=disabled, 1=enabled)
    INS_PARAM_WRAPPER(_use);

    // control enable of fast sampling
    AP_Int8     _fast_sampling_mask; ///< INS_FAST_SAMPLE bitmask - which IMUs push to fast rate buffer

    // control enable of fast sampling
    AP_Int8     _fast_sampling_rate; ///< FAST_SAMPLING_RATE parameter - fast loop decimation factor

    // control enable of detected sensors
    AP_Int8     _enable_mask; ///< INS_ENABLE_MASK bitmask - which detected IMUs to actually enable
    
    // board orientation from AHRS
    enum Rotation _board_orientation; ///< Board rotation from AHRS (applied after sensor-specific rotation)

    // per-sensor orientation to allow for board type defaults at runtime
    enum Rotation _gyro_orientation[INS_MAX_INSTANCES]; ///< Per-gyro rotation to align with body frame
    enum Rotation _accel_orientation[INS_MAX_INSTANCES]; ///< Per-accel rotation to align with body frame

    // calibrated_ok/id_ok flags
    bool _gyro_cal_ok[INS_MAX_INSTANCES]; ///< True if gyro calibration completed successfully
    bool _accel_id_ok[INS_MAX_INSTANCES]; ///< True if accel ID matches expected ID from parameters

    // first usable gyro and accel
    uint8_t _first_usable_gyro; ///< First healthy and enabled gyro instance (primary fallback)
    uint8_t _first_usable_accel; ///< First healthy and enabled accel instance (primary fallback)

    // primary instance
    uint8_t _primary; ///< Primary IMU instance selected by EKF or parameter

    // mask of accels and gyros which we will be actively using
    // and this should wait for in wait_for_sample()
    uint8_t _gyro_wait_mask; ///< Bitmask of gyro instances to wait for in wait_for_sample()
    uint8_t _accel_wait_mask; ///< Bitmask of accel instances to wait for in wait_for_sample()

    // bitmask bit which indicates if we should log raw accel and gyro data
    uint32_t _log_raw_bit; ///< LOG_BITMASK bit indicating raw IMU logging enabled

    // has wait_for_sample() found a sample?
    bool _have_sample:1; ///< True if wait_for_sample() has found new sample data

    bool _backends_detected:1; ///< True after detect_backends() has completed

    // are gyros or accels currently being calibrated
    bool _calibrating_accel; ///< True during accelerometer calibration
    bool _calibrating_gyro; ///< True during gyro bias calibration at startup
    bool _trimming_accel; ///< True during accel trim calculation

    // the delta time in seconds for the last sample
    float _delta_time; ///< Time delta between last two samples in seconds

    // last time a wait_for_sample() returned a sample
    uint32_t _last_sample_usec; ///< Timestamp when wait_for_sample() last returned (microseconds)

    // target time for next wait_for_sample() return
    uint32_t _next_sample_usec; ///< Target timestamp for next sample (microseconds)

    // time between samples in microseconds
    uint32_t _sample_period_usec; ///< Expected time between samples (1000000/loop_rate)

    // last time update() completed
    uint32_t _last_update_usec; ///< Timestamp when update() last completed (microseconds)

    // health of gyros and accels
    bool _gyro_healthy[INS_MAX_INSTANCES]; ///< Per-gyro health flag (false if backend reports errors)
    bool _accel_healthy[INS_MAX_INSTANCES]; ///< Per-accel health flag (false if backend reports errors)

    uint32_t _accel_error_count[INS_MAX_INSTANCES]; ///< Cumulative accel error count from backends
    uint32_t _gyro_error_count[INS_MAX_INSTANCES]; ///< Cumulative gyro error count from backends

    // vibration and clipping
    uint32_t _accel_clip_count[INS_MAX_INSTANCES]; ///< Count of accel saturation events (indicates mechanical overload)
    LowPassFilterVector3f _accel_vibe_floor_filter[INS_VIBRATION_CHECK_INSTANCES]; ///< Low-pass filter for vibration floor estimation
    LowPassFilterVector3f _accel_vibe_filter[INS_VIBRATION_CHECK_INSTANCES]; ///< Low-pass filter for RMS vibration levels

    // peak hold detector state for primary accel
    struct PeakHoldState {
        float accel_peak_hold_neg_x; ///< Peak negative X acceleration (used for crash detection)
        uint32_t accel_peak_hold_neg_x_age; ///< Time since peak accel last updated (ms)
    } _peak_hold_state;

    // threshold for detecting stillness
    AP_Float _still_threshold; ///< INS_STILL_THRESH parameter - vibration threshold for stillness detection

    // Trim options
    AP_Int8 _acc_body_aligned; ///< INS_ACC_BODYFIX parameter - treat accels as body-fixed (no calibration rotation)
    AP_Int8 _trim_option; ///< INS_TRIM_OPTION parameter - trim calculation options

    static AP_InertialSensor *_singleton; ///< Singleton instance pointer for AP::ins()
    AP_AccelCal* _acal; ///< Accelerometer calibration manager (6-point calibration)

    AccelCalibrator *_accel_calibrator; ///< Array of per-IMU calibrator objects

    //save accelerometer bias and scale factors
    void _acal_save_calibrations() override; ///< Save completed accel calibration to persistent storage
    void _acal_event_failure() override; ///< Handle calibration failure event

    // Returns AccelCalibrator objects pointer for specified acceleromter
    AccelCalibrator* _acal_get_calibrator(uint8_t i) override { return i<get_accel_count()?&(_accel_calibrator[i]):nullptr; } ///< Get calibrator for accel instance

    Vector3f _trim_rad; ///< Calculated vehicle trim angles in radians (roll, pitch, yaw)
    bool _new_trim; ///< True if new trim values have been calculated

    bool _accel_cal_requires_reboot; ///< True if accel cal changed params requiring reboot

    // sensor error count at startup (used to ignore errors within 2 seconds of startup)
    uint32_t _accel_startup_error_count[INS_MAX_INSTANCES]; ///< Accel errors at startup (baseline for health checks)
    uint32_t _gyro_startup_error_count[INS_MAX_INSTANCES]; ///< Gyro errors at startup (baseline for health checks)
    bool _startup_error_counts_set; ///< True after startup error counts captured
    uint32_t _startup_ms; ///< System time at initialization (milliseconds)

#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    uint8_t imu_kill_mask; ///< Bitmask of IMUs to disable for testing (simulates IMU failures)
#endif

#if HAL_INS_TEMPERATURE_CAL_ENABLE
public:
    // instance number for logging
#if INS_AUX_INSTANCES
    /**
     * @brief Get temperature calibration instance number for logging
     * @param[in] tc Temperature calibration object reference
     * @return Instance number (0-based)
     * @note Handles both old-style and auxiliary instance parameters
     */
    uint8_t tcal_instance(const AP_InertialSensor_TCal &tc) const {
        for (uint8_t i=0; i<INS_MAX_INSTANCES - INS_AUX_INSTANCES; i++) {
            if (&tc == &tcal_old_param[i]) {
                return i;
            }
        }
        for (uint8_t i=0; i<INS_AUX_INSTANCES; i++) {
            if (&tc == &params[i].tcal) {
                return i + INS_MAX_INSTANCES;
            }
        }
        return 0;
    }
#else
    /**
     * @brief Get temperature calibration instance number for logging
     * @param[in] tc Temperature calibration object reference
     * @return Instance number (0-based)
     */
    uint8_t tcal_instance(const AP_InertialSensor_TCal &tc) const {
        return &tc - &tcal(0);
    }
#endif
private:
    AP_InertialSensor_TCal tcal_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES]; ///< Temperature calibration data (polynomial coefficients)

    enum class TCalOptions : uint8_t {
        PERSIST_TEMP_CAL = (1U<<0), ///< Save temperature calibration across firmware changes
        PERSIST_ACCEL_CAL = (1U<<1), ///< Save accel calibration across firmware changes
    };

    // temperature that last calibration was run at
    AP_Float caltemp_accel_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES]; ///< INS_TCAL_ACC_TMAX - temperature at accel cal (°C)
    AP_Float caltemp_gyro_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES]; ///< INS_TCAL_GYR_TMAX - temperature at gyro cal (°C)

    INS_PARAM_WRAPPER(caltemp_accel);
    INS_PARAM_WRAPPER(caltemp_gyro);
    INS_PARAM_WRAPPER(tcal);

    AP_Int32 tcal_options; ///< INS_TCAL_OPTIONS parameter - temperature calibration persistence options
    bool tcal_learning; ///< True when actively learning temperature calibration coefficients
#endif

    // Raw logging options bitmask and parameter
    enum class RAW_LOGGING_OPTION {
        PRIMARY_GYRO_ONLY   = (1U<<0), ///< Log only primary gyro at sensor rate
        ALL_GYROS           = (1U<<1), ///< Log all gyros at sensor rate
        POST_FILTER         = (1U<<2), ///< Log post-filter gyro data
        PRE_AND_POST_FILTER = (1U<<3), ///< Log both pre and post-filter data
    };
    AP_Int16 raw_logging_options; ///< INS_RAW_LOG_OPT parameter - raw logging options bitmask
    bool raw_logging_option_set(RAW_LOGGING_OPTION option) const {
        return (raw_logging_options.get() & int32_t(option)) != 0;
    }
    // if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // Support for the fast rate thread in copter
    FastRateBuffer* fast_rate_buffer; ///< High-rate gyro sample buffer for copter fast rate loop
    bool fast_rate_buffer_enabled; ///< True when fast rate buffer is actively collecting samples

public:
    /**
     * @brief Enable fast rate gyro sample buffer
     * 
     * @details Activates high-rate circular buffer for gyro samples:
     *          - Used by multicopter fast rate loop (typically 4x main loop)
     *          - Reduces control loop latency
     *          - Provides more samples for rate controller
     * 
     * @note Only available with AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
     * @note Buffer size determined by backend FIFO capacity
     */
    void enable_fast_rate_buffer();

    /**
     * @brief Disable fast rate gyro sample buffer
     * 
     * @note Stops buffer updates and frees resources
     * @note Call when switching to modes not using fast rate loop
     */
    void disable_fast_rate_buffer();

    /**
     * @brief Retrieve next available gyro sample from fast rate buffer
     * 
     * @param[out] gyro Gyro sample in rad/s (body frame)
     * 
     * @return true if sample retrieved, false if buffer empty
     * 
     * @note Non-blocking - returns immediately if no data
     * @note Samples retrieved in FIFO order (oldest first)
     * @note Called by fast rate loop (typically 1-4kHz)
     */
    bool get_next_gyro_sample(Vector3f& gyro);

    /**
     * @brief Get count of available samples in fast rate buffer
     * 
     * @return Number of gyro samples ready for retrieval
     * 
     * @note Used to determine if fast loop can run
     * @note Buffer overflow if not consumed fast enough
     */
    uint32_t get_num_gyro_samples();

    /**
     * @brief Set sample rate decimation for fast rate buffer
     * 
     * @param[in] rdec Decimation factor (1 = all samples, 2 = every other, etc.)
     * 
     * @note Higher decimation reduces CPU load but increases latency
     * @note Typical: 1 for copter fast loop, 4+ for slower vehicles
     */
    void set_rate_decimation(uint8_t rdec);

    /**
     * @brief Push new gyro sample into fast rate buffer
     * 
     * @param[in] gyro Gyro sample in rad/s (body frame)
     * 
     * @return true if sample added, false if buffer full
     * 
     * @note Called by backend at sensor rate
     * @note Oldest sample discarded if buffer full
     * 
     * @warning Buffer overflow indicates fast loop not keeping up
     */
    bool push_next_gyro_sample(const Vector3f& gyro);

    /**
     * @brief Update filter parameters for all backends
     * 
     * @details Reconfigures filters when parameters changed:
     *          - Low-pass filter cutoff frequencies
     *          - Notch filter center frequencies
     *          - Filter coefficients
     * 
     * @note Called when INS_GYRO_FILTER or notch parameters modified
     * @note Ensures smooth filter transitions without glitches
     */
    void update_backend_filters();

    /**
     * @brief Check if fast rate loop sampling enabled for IMU instance
     * 
     * @param[in] instance IMU instance number
     * 
     * @return true if fast rate samples pushed to buffer for this IMU
     * 
     * @note Configured via INS_FAST_SAMPLE parameter bitmask
     */
    bool is_rate_loop_gyro_enabled(uint8_t instance) const;

    /**
     * @brief Check if dynamic FIFO enabled for IMU instance
     * 
     * @param[in] instance IMU instance number
     * 
     * @return true if backend adjusts FIFO depth dynamically
     * 
     * @note Dynamic FIFO optimizes latency vs data rate
     * @note Not all backend drivers support dynamic FIFO
     */
    bool is_dynamic_fifo_enabled(uint8_t instance) const;
    // endif AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
};

namespace AP {
    AP_InertialSensor &ins();
};
