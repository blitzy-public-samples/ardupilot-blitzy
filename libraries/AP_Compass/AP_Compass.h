/**
 * @file AP_Compass.h
 * @brief ArduPilot compass/magnetometer frontend manager
 * 
 * @details This file defines the Compass class, which implements a frontend/backend
 *          architecture for managing multiple magnetometer sensors:
 *          
 *          **Architecture Overview:**
 *          - Frontend (Compass class): Manages multiple compass instances, calibration,
 *            motor compensation, health monitoring, and provides unified interface
 *          - Backends (AP_Compass_Backend subclasses): Hardware-specific drivers for
 *            individual magnetometer chips (HMC5843, AK8963, BMM150, LSM303D, etc.)
 *            
 *          **Singleton Pattern:**
 *          - Accessed globally via AP::compass() accessor
 *          - Single Compass instance manages all magnetometer hardware
 *          - Supports up to COMPASS_MAX_INSTANCES simultaneous sensors
 *          
 *          **Key Responsibilities:**
 *          - Driver detection and initialization at boot
 *          - Multi-compass fusion and primary sensor selection
 *          - Calibration: hard-iron offsets, soft-iron correction, motor compensation
 *          - Health monitoring and consistency checking between sensors
 *          - Coordinate frame transformations (sensor frame → body frame → NED frame)
 *          - Parameter persistence (offsets, orientations, configuration)
 *          - MAVLink telemetry and calibration protocol
 *          
 *          **Magnetic Field Representation:**
 *          - All field values in milligauss (mGauss)
 *          - Field measured in sensor frame, rotated to body frame using COMPASS_ORIENTx
 *          - Heading calculated in NED (North-East-Down) frame after tilt compensation
 *          
 *          **Calibration Types:**
 *          - Sphere fitting: Traditional rotate-vehicle-in-all-axes calibration
 *          - Fixed-yaw: GPS-based calibration with known heading (fast, vehicle stationary)
 *          - Motor compensation: Throttle, current, or per-motor magnetic interference compensation
 *          - In-flight learning: Automatic offset adjustment during flight
 *          
 *          **Typical Usage Pattern:**
 *          ```cpp
 *          Compass &compass = AP::compass();
 *          compass.init();                              // Detect and initialize drivers
 *          
 *          // In main loop (10-100 Hz):
 *          compass.read();                              // Update all compass instances
 *          float heading = compass.calculate_heading(dcm_matrix);  // Get tilt-compensated heading
 *          
 *          if (!compass.healthy()) {
 *              // Handle compass failure
 *          }
 *          ```
 * 
 * @note Coordinate frames: Sensor frame → Body frame → NED frame (right-handed)
 * @note Units: Magnetic field in milligauss, heading in radians, offsets in milligauss
 * @warning Magnetic interference from motors, ESCs, power lines, and metal objects
 *          significantly affects compass accuracy and must be calibrated
 * 
 * Source: libraries/AP_Compass/AP_Compass.h, libraries/AP_Compass/AP_Compass.cpp
 */

#pragma once

#include "AP_Compass_config.h"

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_MSP/msp.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#include "AP_Compass_Backend.h"
#include "Compass_PerMotor.h"
#include <AP_Common/TSIndex.h>

/**
 * @defgroup compass_motor_comp Motor Compensation Types
 * @brief Magnetic interference compensation strategies
 * 
 * Motor currents generate magnetic fields that interfere with compass readings.
 * These compensation types define how to correct for motor-induced interference:
 * @{
 */

/** @brief No motor compensation - raw compass readings used */
#define AP_COMPASS_MOT_COMP_DISABLED    0x00

/** 
 * @brief Throttle-based compensation
 * @details Assumes magnetic interference proportional to throttle percentage.
 *          Compensation = throttle * COMPASS_MOTx_X/Y/Z factors
 *          Works for vehicles where motor current roughly tracks throttle.
 */
#define AP_COMPASS_MOT_COMP_THROTTLE    0x01

/**
 * @brief Current-based compensation  
 * @details Uses actual battery current measurement for compensation.
 *          Compensation = current * COMPASS_MOTx_X/Y/Z factors
 *          More accurate than throttle for varying motor loads.
 */
#define AP_COMPASS_MOT_COMP_CURRENT     0x02

/**
 * @brief Per-motor compensation
 * @details Individual compensation for each motor's magnetic field.
 *          Requires ESC telemetry providing per-motor current.
 *          Most accurate but requires compatible ESCs.
 *          See Compass_PerMotor class.
 */
#define AP_COMPASS_MOT_COMP_PER_MOTOR   0x03

/** @} */ // end of compass_motor_comp group

/**
 * @brief Default board orientation for magnetometers
 * @details Defines default rotation applied to all compasses unless overridden
 *          by individual COMPASS_ORIENTx parameters. Can be overridden at
 *          board level in hwdef.dat files.
 */
#ifndef MAG_BOARD_ORIENTATION
#define MAG_BOARD_ORIENTATION ROTATION_NONE
#endif

/**
 * @brief Enable motor compensation feature
 * @details Set to 1 to enable throttle/current/per-motor compensation.
 *          Can be disabled at board level to save flash on memory-constrained boards.
 */
#ifndef COMPASS_MOT_ENABLED
#define COMPASS_MOT_ENABLED 1
#endif

/**
 * @brief Enable compass offset learning
 * @details Enables in-flight offset learning and EKF-based learning modes.
 *          Automatically enabled if fixed-yaw calibration is available.
 */
#ifndef COMPASS_LEARN_ENABLED
#define COMPASS_LEARN_ENABLED AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
#endif

/**
 * @defgroup compass_cal_constants Calibration Quality Thresholds
 * @brief Constants defining acceptable calibration quality and sensor consistency
 * @{
 */

/**
 * @brief Default calibration fitness threshold
 * @details Maximum acceptable RMS error for sphere-fitting calibration in mGauss.
 *          Lower values indicate better fit. Typical good calibration: < 16 mGauss
 */
#define AP_COMPASS_CALIBRATION_FITNESS_DEFAULT 16.0f

/**
 * @brief Maximum angle difference between any two compass field vector axes
 * @details Used for consistency checking between multiple compasses.
 *          If any axis differs by more than 90°, compasses are inconsistent.
 */
#define AP_COMPASS_MAX_XYZ_ANG_DIFF radians(90.0f)

/**
 * @brief Maximum angle difference in horizontal plane (X-Y)
 * @details Stricter check for horizontal components which most affect heading.
 *          Compasses differing by > 60° in X-Y considered inconsistent.
 */
#define AP_COMPASS_MAX_XY_ANG_DIFF radians(60.0f)

/**
 * @brief Maximum field magnitude difference in horizontal plane
 * @details Maximum allowed difference in horizontal field strength (mGauss).
 *          Large differences indicate one compass has bad calibration or interference.
 */
#define AP_COMPASS_MAX_XY_LENGTH_DIFF 200.0f

/** @} */ // end of compass_cal_constants group

/**
 * @defgroup compass_instance_limits Compass Instance Configuration
 * @brief Platform-specific limits for compass sensor instances
 * 
 * @details Maximum number of compass instances available on this platform.
 *          Multiple compasses provide redundancy and consistency checking.
 *          
 *          Configuration varies by build:
 *          - Main flight controller: 3 compasses (typical)
 *          - AP_Periph CAN nodes: 1 compass (memory constrained)
 *          
 *          When multiple compasses available:
 *          - System monitors consistency between sensors
 *          - Automatically selects best compass based on health and priority
 *          - Can detect and reject faulty compass
 *          - Provides graceful degradation if compass fails
 * @{
 */
#ifndef HAL_BUILD_AP_PERIPH
/**
 * @brief Maximum compass sensors for main flight controller
 * @details Default 3 compasses: typically internal + 2 external
 *          Can be overridden in board hwdef.dat
 */
#ifndef HAL_COMPASS_MAX_SENSORS
#define HAL_COMPASS_MAX_SENSORS 3
#endif

/**
 * @brief Maximum unregistered compass devices to track
 * @details Allows detection of compasses that appear after boot
 *          Supports hot-plug CAN compasses and dynamic discovery
 */
#if HAL_COMPASS_MAX_SENSORS > 1
#define COMPASS_MAX_UNREG_DEV 5
#else
#define COMPASS_MAX_UNREG_DEV 0
#endif

#else  // HAL_BUILD_AP_PERIPH

/**
 * @brief Maximum compass sensors for AP_Periph node
 * @details Limited to 1 compass on CAN peripheral nodes to save memory
 *          AP_Periph forwards compass data to main flight controller via DroneCAN
 */
#ifndef HAL_COMPASS_MAX_SENSORS
#define HAL_COMPASS_MAX_SENSORS 1
#endif

/**
 * @brief No unregistered device tracking on AP_Periph
 * @details Memory-constrained peripherals don't support dynamic compass discovery
 */
#define COMPASS_MAX_UNREG_DEV 0
#endif

/**
 * @brief Maximum compass instances (frontend)
 * @note Matches HAL_COMPASS_MAX_SENSORS - one frontend instance per sensor
 */
#define COMPASS_MAX_INSTANCES HAL_COMPASS_MAX_SENSORS

/**
 * @brief Maximum compass backends (drivers)
 * @note One backend driver per physical sensor
 */
#define COMPASS_MAX_BACKEND   HAL_COMPASS_MAX_SENSORS

/**
 * @brief Total compass tracking capacity
 * @details Sum of registered instances plus unregistered device slots
 *          Allows system to remember and reacquire intermittent compasses
 */
#define MAX_CONNECTED_MAGS (COMPASS_MAX_UNREG_DEV+COMPASS_MAX_INSTANCES)

/** @} */ // end of compass_instance_limits group

#include "CompassCalibrator.h"

class CompassLearn;

/**
 * @file AP_Compass.h
 * @brief Compass driver frontend for magnetometer sensor management
 * 
 * @details This file defines the Compass frontend class, which manages multiple
 *          magnetometer sensors in a frontend/backend architecture pattern.
 *          
 *          Architecture Overview:
 *          - Frontend (Compass class): Manages multiple compass instances, sensor fusion,
 *            calibration, health monitoring, and provides unified API to vehicle code
 *          - Backend (AP_Compass_Backend): Individual sensor drivers implementing
 *            hardware-specific interfaces for various magnetometer chips
 *          
 *          The Compass class is accessed as a singleton via AP::compass() using the
 *          get_singleton() accessor method.
 *          
 *          Key Responsibilities:
 *          - Backend registration and lifecycle management
 *          - Multi-compass priority and selection (primary, fallback sensors)
 *          - Magnetic field measurement in milligauss
 *          - Calibration: offsets, diagonals, off-diagonals, motor compensation
 *          - Health monitoring and consistency checking across compasses
 *          - Coordinate transformations: sensor frame → body frame → NED frame
 *          - Heading calculation with tilt compensation
 *          - Magnetic declination handling (magnetic north → true north)
 *          - Parameter persistence via AP_Param system
 *          
 *          Coordinate Frames:
 *          - Sensor frame: Raw magnetometer chip measurements
 *          - Body frame: Vehicle coordinate system (forward-right-down)
 *          - NED frame: North-East-Down earth-fixed frame for heading
 *          
 *          Units Convention:
 *          - Magnetic field: milligauss (mGauss)
 *          - Heading/declination: radians
 *          - Motor compensation: milligauss per throttle/current unit
 *          
 * @note Typical loop rates: compass read() called at 10-100 Hz
 * @warning Magnetic interference from motors, power lines, metal objects affects accuracy
 * 
 * Source: libraries/AP_Compass/AP_Compass.h
 * Source: libraries/AP_Compass/AP_Compass.cpp
 */

/**
 * @class Compass
 * @brief Frontend manager for magnetometer sensors
 * 
 * @details The Compass class provides a unified interface for managing multiple magnetometer
 *          sensors (up to HAL_COMPASS_MAX_SENSORS instances). It implements:
 *          
 *          Backend Registration:
 *          - Probe and register sensor drivers during initialization
 *          - Support for I2C, SPI, DroneCAN, MSP, ExternalAHRS magnetometers
 *          - Runtime detection of hot-plugged sensors
 *          
 *          Multi-Compass Fusion:
 *          - Priority-based sensor selection (COMPASSx_USE parameters)
 *          - Primary compass selection via get_first_usable()
 *          - Consistency checking across multiple sensors
 *          - Automatic fallback to backup compass on failure
 *          
 *          Calibration System:
 *          - Hard-iron offset calibration (ellipsoid fitting)
 *          - Soft-iron correction with diagonals and off-diagonals
 *          - Motor interference compensation (throttle, current, or per-motor)
 *          - GPS-based fixed-yaw calibration
 *          - Real-time calibration via CompassCalibrator
 *          
 *          Parameter Storage:
 *          - Offsets: COMPASS_OFSx_X/Y/Z (milligauss)
 *          - Diagonals: COMPASS_DIAx_X/Y/Z (scale factors)
 *          - Off-diagonals: COMPASS_ODIx_X/Y/Z (cross-axis correction)
 *          - Motor compensation: COMPASS_MOTx_X/Y/Z (interference factors)
 *          - Device IDs: COMPASS_DEV_IDx (for sensor identification)
 *          - Orientation: COMPASS_ORIENTx (rotation from board frame)
 *          
 *          Health Monitoring:
 *          - Per-compass health status based on update rate
 *          - Consistency checking (field magnitude and direction agreement)
 *          - Detection of sensor failures and interference
 *          
 *          Singleton Pattern:
 *          - Access via AP::compass() or Compass::get_singleton()
 *          - Single instance manages all compass hardware
 *          
 * @note Configuration: HAL_COMPASS_MAX_SENSORS defines maximum compass count
 * @note Thread safety: Uses HAL_Semaphore for backend data protection
 * @warning Calibration required before first flight - uncalibrated compass causes navigation errors
 * @warning Motor compensation critical for multicopters - motor fields can be 100+ mGauss
 */
class Compass
{
friend class AP_Compass_Backend;
public:
    Compass();

    /* Do not allow copies */
    CLASS_NO_COPY(Compass);

    /**
     * @brief Get singleton instance for compass frontend
     * 
     * @details Returns the single Compass instance managing all magnetometer hardware.
     *          This implements the singleton pattern used throughout ArduPilot for
     *          accessing the compass subsystem via AP::compass().
     * 
     * @return Pointer to the singleton Compass instance, or nullptr if not created
     * 
     * @note Typically accessed via AP::compass() convenience accessor
     * @see AP::compass()
     */
    static Compass *get_singleton() {
        return _singleton;
    }

    friend class CompassLearn;

    /**
     * @brief Initialize compass subsystem and detect magnetometer hardware
     * 
     * @details Performs compass initialization sequence:
     *          1. Load parameters from AP_Param storage
     *          2. Detect and register I2C/SPI magnetometer backends
     *          3. Probe external compass interfaces (DroneCAN, MSP, ExternalAHRS)
     *          4. Initialize backend drivers with probe → init sequence
     *          5. Reorder compass priority based on COMPASSx_USE parameters
     *          6. Set up calibration framework if enabled
     *          
     *          Detection order:
     *          - Board-integrated compasses (via hwdef COMPASS definitions)
     *          - External I2C compasses (auto-probe on I2C bus)
     *          - SPI compasses (board-specific chip selects)
     *          - DroneCAN magnetometers (CAN bus enumeration)
     *          - MSP compasses (from flight controller telemetry)
     *          - External AHRS compasses (VectorNav, MicroStrain, etc.)
     * 
     * @return void (marked __INITFUNC__ for placement in initialization section)
     * 
     * @note Called once during vehicle initialization, typically from AP_AHRS::init()
     * @note __INITFUNC__ macro places this in a special memory section freed after boot
     * @warning Must be called before read() or any compass access methods
     * @warning Blocking operation - may take several seconds with I2C bus scanning
     * 
     * Source: libraries/AP_Compass/AP_Compass.cpp:init()
     */
    __INITFUNC__ void init();

    /**
     * @brief Read all compass backends and update magnetic field measurements
     * 
     * @details Polls all registered compass backends to update field measurements:
     *          1. Calls update() on each backend to accumulate sensor samples
     *          2. Averages accumulated samples since last read()
     *          3. Applies calibration corrections (offsets, diagonals, off-diagonals)
     *          4. Applies motor interference compensation if enabled
     *          5. Rotates field from sensor frame to body frame
     *          6. Updates health status based on sample freshness
     *          7. Performs consistency checking across multiple compasses
     *          8. Runs calibration update if calibration active
     *          9. Runs compass learning if enabled
     *          
     *          Motor Compensation Types:
     *          - THROTTLE: field = raw + (motor_comp * throttle)
     *          - CURRENT: field = raw + (motor_comp * current)
     *          - PER_MOTOR: field = raw + sum(per_motor_comp[i] * esc_rpm[i])
     * 
     * @return true if at least one compass provided valid data, false if all failed
     * 
     * @note Called at main loop rate (typically 50-400 Hz depending on vehicle)
     * @note Thread-safe: uses semaphore protection for backend data
     * @warning Must call init() first, otherwise returns false
     * @warning Check healthy() after read() to validate data quality
     * 
     * Source: libraries/AP_Compass/AP_Compass.cpp:read()
     */
    bool read();

    /**
     * @brief Check if compass is available for use
     * 
     * @return true if compass is enabled and has completed initialization
     * 
     * @note Returns false if COMPASS_ENABLE=0 or init() not called
     */
    bool available() const { return _enabled && init_done; }

    /**
     * @brief Calculate tilt-compensated magnetic heading for primary compass
     * 
     * @details Computes magnetic heading from compass field and vehicle attitude:
     *          1. Get magnetic field vector in body frame from compass
     *          2. Rotate to horizontal plane using DCM attitude matrix
     *          3. Calculate heading = atan2(-Y, X) in NED frame
     *          4. Apply magnetic declination to convert magnetic north → true north
     *          
     *          Tilt Compensation Algorithm:
     *          - DCM matrix converts body frame → NED frame
     *          - Magnetic field rotated to earth frame eliminates pitch/roll effects
     *          - Heading computed from horizontal (N-E) plane projection
     *          
     *          Uses primary compass selected by get_first_usable()
     * 
     * @param[in] dcm_matrix Current vehicle attitude as rotation matrix (body → NED)
     * 
     * @return Magnetic heading in radians (0 to 2π), 0 = true north with declination applied
     * 
     * @note Heading is undefined when vehicle is vertical (pitch/roll ±90°)
     * @note Declination applied if auto_declination enabled or set_declination() called
     * @warning Requires valid attitude from AHRS - garbage in DCM produces garbage heading
     * @warning Check healthy() before trusting heading calculation
     * 
     * @see calculate_heading(const Matrix3f&, uint8_t) for specific compass instance
     * @see set_declination() for magnetic declination configuration
     * 
     * Source: libraries/AP_Compass/AP_Compass.cpp:calculate_heading()
     */
    float calculate_heading(const Matrix3f &dcm_matrix) const {
        return calculate_heading(dcm_matrix, _first_usable);
    }
    
    /**
     * @brief Calculate tilt-compensated heading for specific compass instance
     * 
     * @param[in] dcm_matrix Current vehicle attitude as rotation matrix (body → NED)
     * @param[in] i Compass instance index (0 to get_count()-1)
     * 
     * @return Magnetic heading in radians for specified compass
     * 
     * @note Used for comparing headings across multiple compasses
     * @see calculate_heading(const Matrix3f&) for primary compass
     */
    float calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const;

    /**
     * @brief Set hard-iron offset calibration values for compass instance
     * 
     * @details Sets the hard-iron offset correction applied to raw magnetometer readings.
     *          Hard-iron offsets compensate for fixed magnetic fields from vehicle metal,
     *          electronics, and permanent magnets. The correction is:
     *          
     *          corrected_field = raw_field - offset
     *          
     *          These offsets are typically determined through compass calibration
     *          (sphere/ellipsoid fitting). Changes are applied immediately but NOT
     *          saved to persistent storage - use set_and_save_offsets() to persist.
     * 
     * @param[in] i Compass instance index (0 to get_count()-1)
     * @param[in] offsets Offset vector in milligauss for X/Y/Z axes
     * 
     * @note Changes take effect on next read() call
     * @note Offsets not saved to EEPROM - use set_and_save_offsets() to persist
     * @warning Large offsets (>1000 mGauss) may indicate severe magnetic interference
     * 
     * @see set_and_save_offsets() to persist changes
     * @see get_offsets() to retrieve current offsets
     * @see get_offsets_max() for configured maximum allowed offset
     */
    void set_offsets(uint8_t i, const Vector3f &offsets);

    /**
     * @brief Set and save hard-iron offset calibration values
     * 
     * @details Sets hard-iron offsets and immediately saves to persistent parameter storage.
     *          Stores values in COMPASS_OFSx_X, COMPASS_OFSx_Y, COMPASS_OFSx_Z parameters
     *          where x is the compass instance number.
     *          
     *          Automatic Saving:
     *          - Updates in-memory offset values
     *          - Writes to AP_Param parameter storage
     *          - Triggers EEPROM/filesystem write
     *          - Updates expected device ID for validation
     * 
     * @param[in] i Compass instance index (0 to get_count()-1)
     * @param[in] offsets Offset vector in milligauss for X/Y/Z axes
     * 
     * @note Blocking operation - EEPROM write may take several milliseconds
     * @note Called automatically at end of calibration process
     * @warning Frequent calls can wear out EEPROM - use set_offsets() for temporary changes
     * 
     * @see set_offsets() for temporary offset changes
     * @see save_offsets() to save current offsets
     */
    void set_and_save_offsets(uint8_t i, const Vector3f &offsets);
#if AP_COMPASS_DIAGONALS_ENABLED
    /**
     * @brief Set and save soft-iron diagonal scale factors
     * 
     * @details Diagonal elements of soft-iron correction matrix compensate for
     *          per-axis sensitivity variations and soft-iron effects (induced
     *          magnetization in ferrous materials). Applied as:
     *          
     *          corrected[i] = (raw[i] - offset[i]) * diagonal[i]
     *          
     *          where i = X, Y, or Z axis. Values near 1.0 are typical; large
     *          deviations indicate severe soft-iron distortion.
     * 
     * @param[in] i Compass instance index
     * @param[in] diagonals Diagonal scale factors for X/Y/Z (typically 0.8 to 1.2)
     * 
     * @note Requires AP_COMPASS_DIAGONALS_ENABLED feature flag
     * @note Saves to COMPASS_DIAx_X/Y/Z parameters
     * @warning Values far from 1.0 may indicate calibration error or hardware issue
     */
    void set_and_save_diagonals(uint8_t i, const Vector3f &diagonals);
    
    /**
     * @brief Set and save soft-iron off-diagonal correction factors
     * 
     * @details Off-diagonal elements correct for cross-axis coupling in soft-iron
     *          distortion. Full soft-iron correction applies a 3x3 matrix:
     *          
     *          [Dx  ODx  ODy] [raw_x - offset_x]
     *          [ODx Dy   ODz] [raw_y - offset_y]
     *          [ODy ODz  Dz ] [raw_z - offset_z]
     *          
     *          where D=diagonals, OD=off-diagonals. Off-diagonals are typically
     *          small (<0.1) except in severely distorted fields.
     * 
     * @param[in] i Compass instance index
     * @param[in] offdiagonals Off-diagonal correction factors (typically -0.1 to 0.1)
     * 
     * @note Requires AP_COMPASS_DIAGONALS_ENABLED feature flag
     * @note Saves to COMPASS_ODIx_X/Y/Z parameters
     * @note offdiagonals.x = matrix[0][1], .y = matrix[0][2], .z = matrix[1][2]
     */
    void set_and_save_offdiagonals(uint8_t i, const Vector3f &offdiagonals);
#endif
    
    /**
     * @brief Set and save compass scale factor
     * 
     * @param[in] i Compass instance index
     * @param[in] scale_factor Overall scale factor (typically near 1.0)
     */
    void set_and_save_scale_factor(uint8_t i, float scale_factor);
    
    /**
     * @brief Set and save compass orientation relative to vehicle
     * 
     * @param[in] i Compass instance index
     * @param[in] orientation Rotation enum (ROTATION_NONE, ROTATION_YAW_45, etc.)
     * 
     * @note Corrects for compass mounting orientation different from vehicle frame
     */
    void set_and_save_orientation(uint8_t i, Rotation orientation);

    /**
     * @brief Save current offset values to persistent storage for one compass
     * 
     * @details Writes in-memory offset values to AP_Param parameter storage.
     *          Typically used with compass learning (automatic offset tuning)
     *          to periodically persist learned offsets.
     * 
     * @param[in] i Compass instance index (0 to get_count()-1)
     * 
     * @note Blocking EEPROM write operation
     * @note Called automatically by compass learning at configured intervals
     * @warning Don't call at high frequency - EEPROM has limited write cycles
     * 
     * @see save_offsets() to save all compass offsets
     * @see learn_offsets_enabled() to check if learning is active
     */
    void save_offsets(uint8_t i);
    
    /**
     * @brief Save current offset values for all compass instances
     * 
     * @details Iterates through all registered compasses and saves offsets.
     *          Used at end of calibration or during compass learning.
     * 
     * @note Blocking operation - may take time with multiple compasses
     * @see save_offsets(uint8_t) to save single compass
     */
    void save_offsets(void);

    /**
     * @brief Get total number of registered compass instances
     * 
     * @return Number of detected and registered compasses (0 to HAL_COMPASS_MAX_SENSORS)
     * 
     * @note Includes both enabled and disabled compasses
     * @note Value set during init() based on hardware detection
     * @see get_num_enabled() for count of enabled compasses only
     */
    uint8_t get_count(void) const { return _compass_count; }

    /**
     * @brief Get number of enabled compass instances
     * 
     * @return Number of compasses with COMPASSx_USE parameter enabled
     * 
     * @note Only counts compasses available for navigation use
     * @note Disabled compasses (COMPASSx_USE=0) excluded from count
     * @see get_count() for total registered compasses
     */
    uint8_t get_num_enabled(void) const;
    
    /**
     * @brief Get corrected magnetic field vector for specific compass
     * 
     * @details Returns the calibration-corrected magnetic field in vehicle body frame.
     *          The field vector has had the following corrections applied:
     *          1. Hard-iron offset subtraction
     *          2. Soft-iron diagonal/off-diagonal correction (if enabled)
     *          3. Scale factor application
     *          4. Motor interference compensation (if enabled)
     *          5. Rotation from sensor frame to body frame
     *          
     *          Body Frame Convention: X=forward, Y=right, Z=down (right-handed)
     * 
     * @param[in] i Compass instance index (0 to get_count()-1)
     * 
     * @return Magnetic field vector in milligauss (body frame)
     * 
     * @note Field magnitude typically 250-650 mGauss depending on latitude
     * @note At magnetic equator: ~250-350 mGauss, at poles: ~500-650 mGauss
     * @warning Returns last read value - check healthy(i) for data validity
     * @warning Zero vector if compass not initialized or failed
     * 
     * @see get_field() for primary compass field
     */
    const Vector3f &get_field(uint8_t i) const { return _get_state(Priority(i)).field; }
    
    /**
     * @brief Get corrected magnetic field vector for primary compass
     * 
     * @return Magnetic field vector in milligauss (body frame) from first usable compass
     * 
     * @note Uses primary compass selected by COMPASSx_USE priority
     * @see get_field(uint8_t) for specific compass instance
     * @see get_first_usable() for primary compass index
     */
    const Vector3f &get_field(void) const { return get_field(_first_usable); }

    /**
     * @brief Check if scale factor has been set for compass
     * 
     * @param[in] i Compass instance index
     * @return true if scale factor configured (non-default value)
     * 
     * @note Used to detect if compass has been calibrated
     */
    bool have_scale_factor(uint8_t i) const;

#if COMPASS_MOT_ENABLED
    /**
     * @brief Start per-motor compass calibration
     * 
     * @details Initiates calibration to measure magnetic interference from each motor
     *          individually. This mode measures field distortion as a function of
     *          individual motor thrust/current rather than total throttle.
     *          
     *          Calibration Process:
     *          1. Call per_motor_calibration_start()
     *          2. Arm vehicle and hover/fly with varying motor outputs
     *          3. Call per_motor_calibration_update() periodically during flight
     *          4. Land and call per_motor_calibration_end() to compute compensation
     *          
     * @note Requires ESC telemetry or motor output monitoring
     * @note More accurate than throttle-based compensation for multirotors
     * @see per_motor_calibration_update()
     * @see per_motor_calibration_end()
     */
    void per_motor_calibration_start(void) {
        _per_motor.calibration_start();
    }
    
    /**
     * @brief Update per-motor calibration with current motor state
     * 
     * @details Accumulates samples correlating motor outputs with compass field changes.
     *          Called during flight while varying motor speeds.
     * 
     * @note Call repeatedly during calibration flight at main loop rate
     * @see per_motor_calibration_start()
     */
    void per_motor_calibration_update(void) {
        _per_motor.calibration_update();
    }
    
    /**
     * @brief Complete per-motor calibration and compute compensation factors
     * 
     * @details Analyzes accumulated samples and calculates per-motor interference
     *          vectors. Saves compensation parameters to persistent storage.
     * 
     * @note Call after landing to finalize calibration
     * @see per_motor_calibration_start()
     */
    void per_motor_calibration_end(void) {
        _per_motor.calibration_end();
    }
#endif

#if COMPASS_CAL_ENABLED
    /**
     * @brief Update compass calibration state machine
     * 
     * @details Advances the compass calibration process, must be called periodically
     *          during active calibration. Performs:
     *          1. Feeds compass samples to CompassCalibrator for each compass
     *          2. Monitors calibration fitness and completion status
     *          3. Checks for calibration timeout or cancellation
     *          4. Applies calibration results when complete
     *          5. Reports progress via MAVLink to ground station
     *          
     *          Calibration Algorithm (Ellipsoid Fitting):
     *          - Collects compass samples while vehicle rotates through orientations
     *          - Fits ellipsoid to sample distribution to determine offsets and scale
     *          - Requires samples distributed across sphere (rotate vehicle on all axes)
     *          
     * @note Called automatically by read() when calibration is active
     * @note Calibration requires 10-30 seconds of vehicle rotation in multiple orientations
     * @warning Vehicle must remain stationary in position while rotating for calibration
     * 
     * @see start_calibration_all() to begin calibration
     * @see is_calibrating() to check calibration status
     */
    void cal_update();

    /**
     * @brief Start compass calibration for all enabled compasses
     * 
     * @details Initiates sphere/ellipsoid fitting calibration process for all compasses
     *          with COMPASSx_USE enabled. User must rotate vehicle slowly through full
     *          range of orientations (all roll/pitch/yaw combinations) to collect
     *          well-distributed samples.
     *          
     *          Calibration Process:
     *          1. Clears previous calibration data
     *          2. Initializes CompassCalibrator for each compass
     *          3. Begins sample collection (indicated by LED patterns)
     *          4. User rotates vehicle through all orientations
     *          5. Algorithm fits ellipsoid to samples
     *          6. Computes offsets, diagonals, off-diagonals
     *          7. Optionally auto-saves and reboots
     *          
     *          LED Feedback (if AP_Notify enabled):
     *          - Collecting samples: Blinking pattern
     *          - Sufficient samples: Success indication
     *          - Calibration failed: Error indication
     * 
     * @param[in] retry If true, retry failed calibration automatically
     * @param[in] autosave If true, automatically save calibration on completion
     * @param[in] delay_sec Delay in seconds before starting calibration
     * @param[in] autoreboot If true, automatically reboot after successful autosave
     * 
     * @return true if calibration started, false if no compasses to calibrate
     * 
     * @note Blocks vehicle arming during calibration
     * @note Requires ~10-30 seconds of vehicle rotation
     * @note GPS lock recommended for better declination and consistency checking
     * @warning Vehicle must be away from metal structures and power lines
     * @warning Disable motor compensation during calibration (COMPASS_MOT_x = 0)
     * 
     * @see cal_update() for calibration state machine
     * @see cancel_calibration_all() to abort calibration
     * @see is_calibrating() to check status
     */
    bool start_calibration_all(bool retry=false, bool autosave=false, float delay_sec=0.0f, bool autoreboot = false);

    /**
     * @brief Cancel active compass calibration for all compasses
     * 
     * @details Aborts calibration in progress, discards partial results, and
     *          restores previous calibration values. Safe to call even if
     *          no calibration is active.
     * 
     * @note Previous calibration values remain unchanged
     * @note Clears calibration-in-progress flags and LED indicators
     * @see start_calibration_all()
     * @see is_calibrating()
     */
    void cancel_calibration_all();

    /**
     * @brief Check if compass calibration requires reboot
     * 
     * @return true if reboot required after calibration completion
     * 
     * @note Some calibration modes require reboot to apply changes
     */
    bool compass_cal_requires_reboot() const { return _cal_requires_reboot; }
    
    /**
     * @brief Check if compass calibration is currently active
     * 
     * @return true if any compass is currently being calibrated
     * 
     * @note Returns false if calibration completed or not started
     * @see start_calibration_all()
     * @see cancel_calibration_all()
     */
    bool is_calibrating() const;

#if HAL_MAVLINK_BINDINGS_ENABLED
    /*
      handle an incoming MAG_CAL command
    */
    MAV_RESULT handle_mag_cal_command(const mavlink_command_int_t &packet);

    bool send_mag_cal_progress(const class GCS_MAVLINK& link);
    bool send_mag_cal_report(const class GCS_MAVLINK& link);
#endif  // HAL_MAVLINK_BINDINGS_ENABLED
#endif  // COMPASS_CAL_ENABLED

    /**
     * @brief Set log bitmask bit for compass logging
     * 
     * @param[in] log_bit Bit position in LOG_BITMASK parameter for compass logging
     * 
     * @note Used by logging subsystem to determine when to log compass data
     */
    void set_log_bit(uint32_t log_bit) { _log_bit = log_bit; }

    /**
     * @brief Check if multiple compasses are measuring consistent field direction
     * 
     * @details Compares magnetic field vectors across all enabled compasses to detect:
     *          - Excessive angular differences between compass headings
     *          - Field magnitude differences indicating calibration issues
     *          - Individual compass failures or severe interference
     *          
     *          Consistency Thresholds:
     *          - Angular difference < AP_COMPASS_MAX_XY_ANG_DIFF (60°) in horizontal plane
     *          - Angular difference < AP_COMPASS_MAX_XYZ_ANG_DIFF (90°) in 3D
     *          - Field magnitude difference < AP_COMPASS_MAX_XY_LENGTH_DIFF (200 mGauss)
     *          
     *          Used by EKF to detect compass failures and reject bad measurements.
     * 
     * @return true if all enabled compasses show consistent readings
     * 
     * @note Only compares healthy compasses with COMPASSx_USE enabled
     * @note Returns true if only one compass enabled (no comparison possible)
     * @warning Inconsistency indicates calibration error, interference, or hardware failure
     * @warning EKF will reject compass if consistency check fails
     * 
     * @see healthy() to check individual compass health
     */
    bool consistent() const;

    /**
     * @brief Check if specific compass instance is healthy
     * 
     * @details Health criteria:
     *          - Compass has received updates recently (within timeout period)
     *          - Backend driver reports sensor functioning correctly
     *          - Calibration data loaded successfully
     *          - No communication errors with sensor hardware
     * 
     * @param[in] i Compass instance index (0 to get_count()-1)
     * 
     * @return true if compass is providing valid measurements
     * 
     * @note Unhealthy compass excluded from navigation calculations
     * @warning Using unhealthy compass for heading causes navigation errors
     * @see healthy() for primary compass health
     * @see consistent() to check multi-compass agreement
     */
    bool healthy(uint8_t i) const;
    
    /**
     * @brief Check if primary compass is healthy
     * 
     * @return true if first usable compass is healthy
     * 
     * @note Checks health of compass returned by get_first_usable()
     * @see healthy(uint8_t) for specific compass instance
     */
    bool healthy(void) const { return healthy(_first_usable); }
    
    /**
     * @brief Get bitmask of healthy compass instances
     * 
     * @return Bitmask where bit i indicates compass i is healthy
     * 
     * @note Bit 0 = compass 0, bit 1 = compass 1, etc.
     * @note Used for multi-compass health monitoring
     */
    uint8_t get_healthy_mask() const;

    /**
     * @brief Get current hard-iron offset calibration values
     * 
     * @param[in] i Compass instance index (0 to get_count()-1)
     * 
     * @return Offset vector in milligauss for X/Y/Z axes
     * 
     * @note Offsets loaded from COMPASS_OFSx_X/Y/Z parameters at startup
     * @see get_offsets() for primary compass
     * @see set_offsets() to change offset values
     */
    const Vector3f &get_offsets(uint8_t i) const { return _get_state(Priority(i)).offset; }
    
    /**
     * @brief Get offset calibration values for primary compass
     * 
     * @return Offset vector in milligauss for first usable compass
     * 
     * @see get_offsets(uint8_t) for specific compass instance
     */
    const Vector3f &get_offsets(void) const { return get_offsets(_first_usable); }

#if AP_COMPASS_DIAGONALS_ENABLED
    /**
     * @brief Get soft-iron diagonal scale factors
     * 
     * @param[in] i Compass instance index
     * @return Diagonal correction factors for X/Y/Z axes
     * 
     * @note Returns Vector3f(1,1,1) if diagonals not calibrated
     */
    const Vector3f &get_diagonals(uint8_t i) const { return _get_state(Priority(i)).diagonals; }
    const Vector3f &get_diagonals(void) const { return get_diagonals(_first_usable); }

    /**
     * @brief Get soft-iron off-diagonal correction factors
     * 
     * @param[in] i Compass instance index
     * @return Off-diagonal elements: x=[0][1], y=[0][2], z=[1][2] of correction matrix
     * 
     * @note Returns Vector3f(0,0,0) if off-diagonals not calibrated
     */
    const Vector3f &get_offdiagonals(uint8_t i) const { return _get_state(Priority(i)).offdiagonals; }
    const Vector3f &get_offdiagonals(void) const { return get_offdiagonals(_first_usable); }
#endif  // AP_COMPASS_DIAGONALS_ENABLED

    /**
     * @brief Check if in-flight compass offset learning is enabled
     * 
     * @return true if COMPASS_LEARN=INFLIGHT
     * 
     * @note In-flight learning automatically adjusts offsets using EKF innovations
     * @see LearnType enum for learning modes
     */
    bool learn_offsets_enabled() const { return _learn == LearnType::INFLIGHT; }

    /**
     * @brief Check if specific compass should be used for yaw calculations
     * 
     * @details Compass usability determined by:
     *          - COMPASSx_USE parameter set to 1 (enabled)
     *          - Compass is healthy and providing valid data
     *          - External vs internal compass priority (external preferred)
     *          
     *          Used by AHRS/EKF to select which compasses contribute to yaw estimate.
     * 
     * @param[in] i Compass instance index
     * 
     * @return true if compass should contribute to yaw estimation
     * 
     * @note Disabled compasses (COMPASSx_USE=0) return false
     * @see use_for_yaw() for primary compass
     * @see get_first_usable() for primary compass index
     */
    bool use_for_yaw(uint8_t i) const;
    
    /**
     * @brief Check if primary compass should be used for yaw
     * 
     * @return true if first usable compass enabled for yaw
     * 
     * @see use_for_yaw(uint8_t) for specific compass instance
     */
    bool use_for_yaw(void) const;

    /**
     * @brief Set magnetic declination for true north conversion
     * 
     * @details Magnetic declination is the angle between magnetic north (compass reading)
     *          and true north (geographic north pole). Declination varies by location:
     *          - Positive: Magnetic north is east of true north
     *          - Negative: Magnetic north is west of true north
     *          - Range: Typically -30° to +30°, up to ±180° at magnetic poles
     *          
     *          Applied in heading calculation:
     *          true_heading = magnetic_heading + declination
     *          
     *          Declination can be:
     *          - Set manually via this function
     *          - Auto-calculated from GPS position (if COMPASS_AUTODEC=1)
     *          - Obtained from world magnetic model (WMM)
     * 
     * @param[in] radians Declination angle in radians (positive = east)
     * @param[in] save_to_eeprom If true, save to COMPASS_DEC parameter in EEPROM
     * 
     * @note Most users should use auto-declination (COMPASS_AUTODEC=1) with GPS
     * @note Declination changes slowly over years (secular variation)
     * @warning Incorrect declination causes constant heading error affecting navigation
     * 
     * @see get_declination() to retrieve current setting
     * @see auto_declination_enabled() to check if auto mode active
     */
    void set_declination(float radians, bool save_to_eeprom = true);
    
    /**
     * @brief Get current magnetic declination setting
     * 
     * @return Declination in radians (positive = magnetic north east of true north)
     * 
     * @note Returns auto-calculated value if COMPASS_AUTODEC=1, otherwise COMPASS_DEC parameter
     * @see set_declination()
     */
    float get_declination() const;

    /**
     * @brief Check if automatic declination calculation is enabled
     * 
     * @return true if COMPASS_AUTODEC parameter is non-zero
     * 
     * @note Auto-declination requires GPS lock to determine vehicle location
     * @note Uses World Magnetic Model (WMM) for declination lookup
     */
    bool auto_declination_enabled() const { return _auto_declination != 0; }

    /**
     * @brief Set overall board orientation relative to vehicle
     * 
     * @param[in] orientation Board rotation (ROTATION_NONE, ROTATION_YAW_45, etc.)
     * 
     * @note Applied to all compasses unless individual COMPASS_ORIENTx set
     * @note Typically set by AHRS based on AHRS_ORIENTATION parameter
     * @see get_board_orientation()
     */
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    /**
     * @brief Get overall board orientation
     * 
     * @return Current board rotation setting
     * 
     * @see set_board_orientation()
     */
    enum Rotation get_board_orientation(void) const {
        return _board_orientation;
    }

    /**
     * @brief Set motor interference compensation type
     * 
     * @details Configures how motor magnetic interference is compensated. Motors
     *          and ESCs generate significant magnetic fields (often 100+ mGauss near
     *          compass) that vary with motor output. Compensation types:
     *          
     *          DISABLED (0): No compensation applied
     *          - Use only for vehicles without motors or with external compasses far from interference
     *          
     *          THROTTLE (1): Compensation based on total throttle
     *          - interference_field = motor_comp_factor * throttle_percentage
     *          - Simple, works for similar motor outputs
     *          - Less accurate for multirotors with differential thrust
     *          
     *          CURRENT (2): Compensation based on battery current
     *          - interference_field = motor_comp_factor * battery_current
     *          - More accurate than throttle, requires current sensor
     *          - Better for vehicles with varying loads
     *          
     *          PER_MOTOR (3): Individual motor compensation
     *          - interference_field = sum(per_motor_comp[i] * motor_output[i])
     *          - Most accurate, requires ESC telemetry or motor output data
     *          - Compensates for differential motor outputs
     * 
     * @param[in] comp_type Compensation mode (0-3, see AP_COMPASS_MOT_COMP_* defines)
     * 
     * @note Set via COMPASS_MOT_x parameters
     * @note Requires calibration flight to determine compensation factors
     * @warning Incorrect compensation worse than no compensation - calibrate carefully
     * 
     * @see get_motor_compensation_type()
     * @see set_motor_compensation() to set compensation factors
     */
    void motor_compensation_type(const uint8_t comp_type);

    /**
     * @brief Get current motor compensation type
     * 
     * @return Compensation type: 0=DISABLED, 1=THROTTLE, 2=CURRENT, 3=PER_MOTOR
     * 
     * @see motor_compensation_type() to set mode
     */
    uint8_t get_motor_compensation_type() const {
        return _motor_comp_type;
    }

    /**
     * @brief Set motor interference compensation factors
     * 
     * @details Sets the compensation vector applied based on motor output:
     *          
     *          For THROTTLE mode:
     *          corrected_field = raw_field + motor_comp_factor * throttle
     *          
     *          For CURRENT mode:
     *          corrected_field = raw_field + motor_comp_factor * current
     *          
     *          Factors typically determined through calibration flight while
     *          measuring compass variations vs. motor output.
     * 
     * @param[in] i Compass instance index
     * @param[in] motor_comp_factor Compensation vector in milligauss per unit throttle/current
     * 
     * @note Factors saved to COMPASS_MOTx_X/Y/Z parameters
     * @note Typically 100-500 mGauss per unit for internal compasses
     * @see get_motor_compensation() to retrieve factors
     * @see motor_compensation_type() to set compensation mode
     */
    void set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor);

    /**
     * @brief Get motor compensation factors for specific compass
     * 
     * @param[in] i Compass instance index
     * @return Compensation factors in milligauss per throttle/current unit
     * 
     * @see set_motor_compensation()
     */
    const Vector3f& get_motor_compensation(uint8_t i) const { return _get_state(Priority(i)).motor_compensation; }
    
    /**
     * @brief Get motor compensation factors for primary compass
     * 
     * @return Compensation factors for first usable compass
     */
    const Vector3f& get_motor_compensation(void) const { return get_motor_compensation(_first_usable); }

    /**
     * @brief Save motor compensation factors to persistent storage
     * 
     * @details Writes current compensation factors to COMPASS_MOTx_X/Y/Z parameters.
     *          Called at end of motor compensation calibration.
     * 
     * @note Blocking EEPROM write operation
     * @see set_motor_compensation()
     */
    void save_motor_compensation();

    /**
     * @brief Get current motor interference offset being applied
     * 
     * @details Returns the actual interference compensation being subtracted from
     *          raw compass readings based on current motor output and compensation factors.
     *          
     *          This is the real-time computed value:
     *          motor_offset = motor_compensation * motor_output
     * 
     * @param[in] i Compass instance index
     * 
     * @return Current motor interference offset in milligauss
     * 
     * @note Updated on every read() call based on set_throttle() or ESC telemetry
     * @note Zero when motors are off
     * @see get_motor_compensation() for compensation factors
     */
    const Vector3f &get_motor_offsets(uint8_t i) const { return _get_state(Priority(i)).motor_offset; }
    const Vector3f &get_motor_offsets(void) const { return get_motor_offsets(_first_usable); }

    /**
     * @brief Set current throttle for motor compensation
     * 
     * @details Provides throttle input for THROTTLE mode motor compensation.
     *          The compass read() function uses this to calculate motor interference:
     *          motor_offset = motor_compensation * throttle
     * 
     * @param[in] thr_pct Throttle as fraction from 0.0 (off) to 1.0 (full)
     * 
     * @note Only used when motor_compensation_type == AP_COMPASS_MOT_COMP_THROTTLE
     * @note Called by vehicle code at main loop rate with current throttle
     * @see motor_compensation_type()
     * @see set_motor_compensation()
     */
    void set_throttle(float thr_pct) {
        if (_motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            _thr = thr_pct;
        }
    }

#if COMPASS_MOT_ENABLED
    /**
     * @brief Set battery voltage for per-motor compensation calculations
     * 
     * @param[in] voltage Current battery voltage in volts
     * 
     * @note Used by per-motor compensation to account for voltage-dependent motor fields
     * @see per_motor_calibration_start()
     */
    void set_voltage(float voltage) {
        _per_motor.set_voltage(voltage);
    }
#endif
    
    /**
     * @brief Check if compass has been calibrated
     * 
     * @details Verifies compass configuration by checking:
     *          - Offsets have been saved (non-zero or explicitly configured)
     *          - Device ID matches expected sensor
     *          - COMPASSx_USE parameter configured appropriately
     *          
     *          Unconfigured compasses should not be used for navigation.
     * 
     * @param[in] i Compass instance index
     * 
     * @return true if compass has valid calibration data
     * 
     * @note Pre-arm checks use this to prevent flight with uncalibrated compass
     * @warning Flying with uncalibrated compass causes navigation failures
     * @see configured(char*, uint8_t) for detailed failure message
     */
    bool configured(uint8_t i);
    
    /**
     * @brief Check compass configuration with detailed error message
     * 
     * @param[out] failure_msg Buffer for error message if not configured
     * @param[in] failure_msg_len Length of failure_msg buffer
     * 
     * @return true if all enabled compasses configured, false with error message
     * 
     * @note Used by arming checks to provide user-friendly error messages
     */
    bool configured(char *failure_msg, uint8_t failure_msg_len);

    /**
     * @brief Get timestamp of last compass update for primary compass
     * 
     * @return Timestamp in microseconds from system boot
     * 
     * @note Used to detect compass timeout and health issues
     * @see last_update_usec(uint8_t) for specific compass
     */
    uint32_t last_update_usec(void) const { return last_update_usec(_first_usable); }
    
    /**
     * @brief Get timestamp of last compass update for specific compass
     * 
     * @param[in] i Compass instance index
     * @return Timestamp in microseconds from system boot
     * 
     * @note Updated by backend drivers when new samples available
     * @see last_update_ms() for millisecond resolution
     */
    uint32_t last_update_usec(uint8_t i) const { return _get_state(Priority(i)).last_update_usec; }

    /**
     * @brief Get timestamp of last compass update in milliseconds
     * 
     * @return Timestamp in milliseconds from system boot
     */
    uint32_t last_update_ms(void) const { return last_update_ms(_first_usable); }
    
    /**
     * @brief Get timestamp of last compass update in milliseconds for specific compass
     * 
     * @param[in] i Compass instance index
     * @return Timestamp in milliseconds from system boot
     */
    uint32_t last_update_ms(uint8_t i) const { return _get_state(Priority(i)).last_update_ms; }

    /**
     * @brief AP_Param variable information for compass parameters
     * 
     * @details Defines the parameter table for persistent compass configuration:
     *          - COMPASS_ENABLE: Enable/disable compass subsystem
     *          - COMPASS_OFSx_X/Y/Z: Hard-iron offsets in milligauss
     *          - COMPASS_DIAx_X/Y/Z: Soft-iron diagonal scale factors
     *          - COMPASS_ODIx_X/Y/Z: Soft-iron off-diagonal corrections
     *          - COMPASS_MOTx_X/Y/Z: Motor compensation factors
     *          - COMPASS_DEV_IDx: Device IDs for sensor identification
     *          - COMPASS_ORIENTx: Compass orientation rotation
     *          - COMPASSx_USE: Enable compass for yaw calculations
     *          - COMPASS_DEC: Manual declination setting (radians)
     *          - COMPASS_AUTODEC: Enable automatic declination
     *          - COMPASS_LEARN: Offset learning mode (LearnType enum)
     *          - COMPASS_OFS_MAX: Maximum allowed offset magnitude
     *          - COMPASS_CAL_FIT: Calibration fitness threshold
     *          - COMPASS_OPTIONS: Bitmask option flags
     *          
     *          Parameters persisted to EEPROM/filesystem and loaded at boot.
     * 
     * @note Generated documentation available via ground station parameter editor
     * @see LearnType enum for COMPASS_LEARN values
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @enum LearnType
     * @brief Compass offset learning modes
     * 
     * @details Defines automatic offset adjustment strategies:
     * 
     * @var NONE
     * No automatic learning - use only saved calibration values
     * 
     * @var COPY_FROM_EKF  
     * Copy offset innovations from EKF compass bias estimates
     * - Uses EKF magnetometer bias states
     * - Automatically adjusts offsets based on EKF innovations
     * - Requires EKF enabled and GPS lock
     * 
     * @var INFLIGHT
     * In-flight compass learning
     * - Continuously adjusts offsets during flight
     * - Uses algorithm to distinguish vehicle rotation from offset drift
     * - Automatically compensates for magnetic environment changes
     * - Saves learned offsets periodically
     * 
     * @note Set via COMPASS_LEARN parameter
     * @note In-flight learning can compensate for changing interference patterns
     * @warning Learning requires stable flight - avoid during aggressive maneuvers
     */
    enum class LearnType {
        NONE          = 0,
        // INTERNAL   = 1,  // Reserved for future use
        COPY_FROM_EKF = 2,
        INFLIGHT      = 3,
    };

    /**
     * @brief Get current compass offset learning mode
     * 
     * @return Current learning type (NONE, COPY_FROM_EKF, or INFLIGHT)
     * 
     * @see LearnType enum
     * @see set_learn_type()
     */
    LearnType get_learn_type(void) const {
        return (LearnType)_learn.get();
    }

    /**
     * @brief Set compass offset learning mode
     * 
     * @param[in] type Learning mode to enable
     * @param[in] save If true, save to COMPASS_LEARN parameter
     * 
     * @note Learning modes require EKF and GPS for best results
     * @see LearnType enum for mode descriptions
     */
    void set_learn_type(LearnType type, bool save) {
        if (save) {
            _learn.set_and_save(type);
        } else {
            _learn.set(type);
        }
    }
    
    /**
     * @brief Get maximum allowed compass offset magnitude
     * 
     * @return Maximum offset in milligauss (from COMPASS_OFS_MAX parameter)
     * 
     * @note Offsets exceeding this threshold indicate severe interference or calibration failure
     * @note Default typically 850-1000 mGauss
     * @warning Offsets near maximum suggest compass mounted too close to interference sources
     */
    uint16_t get_offsets_max(void) const {
        return (uint16_t)_offset_max.get();
    }

    /**
     * @brief Get compass filtering range parameter
     * 
     * @return Filter range value (implementation-specific)
     * 
     * @note Used by some compass drivers for sensor configuration
     */
    uint8_t get_filter_range() const { return uint8_t(_filter_range.get()); }

#if AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
    /**
     * @brief Perform GPS-based fixed-yaw compass calibration
     * 
     * @details Fast calibration using known vehicle heading and location:
     *          1. Uses GPS position to get expected magnetic field from World Magnetic Model
     *          2. Uses provided yaw angle as known heading reference
     *          3. Calculates required offsets to match expected field
     *          4. Much faster than traditional sphere fitting (seconds vs. minutes)
     *          
     *          Requirements:
     *          - Accurate vehicle yaw (from GPS course, external reference, or manual alignment)
     *          - GPS lock with good position accuracy
     *          - Vehicle stationary or moving straight
     *          - Known latitude/longitude
     *          
     *          Best for:
     *          - Field calibration updates
     *          - Vehicles that can't be rotated (large planes, boats)
     *          - Quick recalibration verification
     * 
     * @param[in] yaw_deg Vehicle yaw in degrees (0=north, 90=east, clockwise)
     * @param[in] compass_mask Bitmask of compass instances to calibrate (bit 0=compass 0, etc.)
     * @param[in] lat_deg Vehicle latitude in degrees
     * @param[in] lon_deg Vehicle longitude in degrees  
     * @param[in] force_use Force use even if conditions not ideal
     * 
     * @return true if calibration successful and offsets updated
     * 
     * @note Requires AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED feature flag
     * @note Yaw accuracy critical - 5° yaw error causes significant offset error
     * @note Works best with external yaw reference or GPS course from straight motion
     * @warning Poor yaw reference creates worse calibration than no calibration
     * @warning GPS required for position and magnetic model lookup
     * 
     * Source: libraries/AP_Compass/AP_Compass_Calibration.cpp:mag_cal_fixed_yaw()
     */
    bool mag_cal_fixed_yaw(float yaw_deg, uint8_t compass_mask,
                           float lat_deg, float lon_deg,
                           bool force_use=false);
#endif

#if AP_COMPASS_MSP_ENABLED
    /**
     * @brief Handle compass data from MSP protocol
     * 
     * @param[in] pkt MSP compass data message
     * 
     * @note Allows compass data to come from MSP OSD or external flight controller
     * @note Enables compass sharing between stacked flight controllers
     */
    void handle_msp(const MSP::msp_compass_data_message_t &pkt);
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    /**
     * @brief Handle compass data from external AHRS system
     * 
     * @param[in] pkt External AHRS magnetometer message
     * 
     * @note Allows compass data from external INS/AHRS like VectorNav, LORD MicroStrain
     * @note Supports high-quality compass integrated into external navigation system
     */
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt);
#endif

    /**
     * @brief Force save current calibration as valid
     * 
     * @details Immediately saves current offset/diagonal/offdiagonal parameters
     *          to persistent storage without waiting for normal save trigger.
     *          Useful for:
     *          - Forcing acceptance of manual calibration adjustments
     *          - Saving in-flight learned offsets on demand
     *          - Backup before attempting new calibration
     * 
     * @note Does not validate calibration quality - saves current values as-is
     * @warning May save poor calibration if called with bad offset values
     */
    void force_save_calibration(void);

    /**
     * @brief Get index of first compass marked for yaw use
     * 
     * @return Compass instance index of first usable compass (based on COMPASSx_USE)
     * 
     * @note Returns compass with highest priority where COMPASSx_USE != 0
     * @note Used as default compass for heading calculations
     * @see use_for_yaw()
     */
    uint8_t get_first_usable(void) const { return _first_usable; }

private:
    static Compass *_singleton;

    // Use Priority and StateIndex typesafe index types
    // to distinguish between two different type of indexing
    // We use StateIndex for access by Backend
    // and Priority for access by Frontend
    DECLARE_TYPESAFE_INDEX(Priority, uint8_t);
    DECLARE_TYPESAFE_INDEX(StateIndex, uint8_t);

    /// Register a new compas driver, allocating an instance number
    ///
    /// @param  dev_id                   Dev ID of compass to register against
    ///
    /// @return instance number saved against the dev id or first available empty instance number
    bool register_compass(int32_t dev_id, uint8_t& instance);

    // load backend drivers
    bool _add_backend(AP_Compass_Backend *backend);
    __INITFUNC__ void _probe_external_i2c_compasses(void);
    __INITFUNC__ void _detect_backends(void);
    __INITFUNC__ void probe_i2c_spi_compasses(void);
#if AP_COMPASS_DRONECAN_ENABLED
    void probe_dronecan_compasses(void);
#endif

#if COMPASS_CAL_ENABLED
    // compass cal
    void _update_calibration_trampoline();
    bool _accept_calibration(uint8_t i);
    bool _accept_calibration_mask(uint8_t mask);
    void _cancel_calibration(uint8_t i);
    void _cancel_calibration_mask(uint8_t mask);
    uint8_t _get_cal_mask();
    bool _start_calibration(uint8_t i, bool retry=false, float delay_sec=0.0f);
    bool _start_calibration_mask(uint8_t mask, bool retry=false, bool autosave=false, float delay_sec=0.0f, bool autoreboot=false);
    bool _auto_reboot() const { return _compass_cal_autoreboot; }
#if HAL_MAVLINK_BINDINGS_ENABLED
    Priority next_cal_progress_idx[MAVLINK_COMM_NUM_BUFFERS];
    Priority next_cal_report_idx[MAVLINK_COMM_NUM_BUFFERS];
#endif
#endif  // COMPASS_CAL_ENABLED

    // see if we already have probed a i2c driver by bus number and address
    bool _have_i2c_driver(uint8_t bus_num, uint8_t address) const;

#if AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
    /*
      get mag field with the effects of offsets, diagonals and
      off-diagonals removed
    */
    bool get_uncorrected_field(uint8_t instance, Vector3f &field) const;
#endif

#if COMPASS_CAL_ENABLED
    //keep track of which calibrators have been saved
    RestrictIDTypeArray<bool, COMPASS_MAX_INSTANCES, Priority> _cal_saved;
    bool _cal_autosave;

    //autoreboot after compass calibration
    bool _compass_cal_autoreboot;
    bool _cal_requires_reboot;
    bool _cal_has_run;
#endif  // COMPASS_CAL_ENABLED

    // enum of drivers for COMPASS_DISBLMSK
    enum DriverType {
#if AP_COMPASS_HMC5843_ENABLED
        DRIVER_HMC5843  =0,
#endif
#if AP_COMPASS_LSM303D_ENABLED
        DRIVER_LSM303D  =1,
#endif
#if AP_COMPASS_AK8963_ENABLED
        DRIVER_AK8963   =2,
#endif
#if AP_COMPASS_BMM150_ENABLED
        DRIVER_BMM150   =3,
#endif
#if AP_COMPASS_LSM9DS1_ENABLED
        DRIVER_LSM9DS1  =4,
#endif
#if AP_COMPASS_LIS3MDL_ENABLED
        DRIVER_LIS3MDL  =5,
#endif
#if AP_COMPASS_AK09916_ENABLED
        DRIVER_AK09916  =6,
#endif
#if AP_COMPASS_IST8310_ENABLED
        DRIVER_IST8310  =7,
#endif
#if AP_COMPASS_ICM20948_ENABLED
        DRIVER_ICM20948 =8,
#endif
#if AP_COMPASS_MMC3416_ENABLED
        DRIVER_MMC3416  =9,
#endif
#if AP_COMPASS_DRONECAN_ENABLED
        DRIVER_UAVCAN   =11,
#endif
#if AP_COMPASS_QMC5883L_ENABLED
        DRIVER_QMC5883L =12,
#endif
#if AP_COMPASS_SITL_ENABLED
        DRIVER_SITL     =13,
#endif
#if AP_COMPASS_MAG3110_ENABLED
        DRIVER_MAG3110  =14,
#endif
#if AP_COMPASS_IST8308_ENABLED
        DRIVER_IST8308  =15,
#endif
#if AP_COMPASS_RM3100_ENABLED
		DRIVER_RM3100   =16,
#endif
#if AP_COMPASS_MSP_ENABLED
        DRIVER_MSP      =17,
#endif
#if AP_COMPASS_EXTERNALAHRS_ENABLED
        DRIVER_EXTERNALAHRS   =18,
#endif
#if AP_COMPASS_MMC5XX3_ENABLED
        DRIVER_MMC5XX3  =19,
#endif
#if AP_COMPASS_QMC5883P_ENABLED
        DRIVER_QMC5883P =20,
#endif
#if AP_COMPASS_BMM350_ENABLED
        DRIVER_BMM350   =21,
#endif
#if AP_COMPASS_IIS2MDC_ENABLED
        DRIVER_IIS2MDC  =22,
#endif
};

    bool _driver_enabled(enum DriverType driver_type);
    
    // backend objects
    AP_Compass_Backend *_backends[COMPASS_MAX_BACKEND];
    uint8_t     _backend_count;

    // whether to enable the compass drivers at all
    AP_Int8     _enabled;

    // number of registered compasses.
    uint8_t     _compass_count;

    // number of unregistered compasses.
    uint8_t     _unreg_compass_count;

    // settable parameters
    AP_Enum<LearnType> _learn;

    // board orientation from AHRS
    enum Rotation _board_orientation = ROTATION_NONE;

    // declination in radians
    AP_Float    _declination;

    // enable automatic declination code
    AP_Int8     _auto_declination;

    // stores which bit is used to indicate we should log compass readings
    uint32_t _log_bit = -1;

    // motor compensation type
    // 0 = disabled, 1 = enabled for throttle, 2 = enabled for current
    AP_Int8     _motor_comp_type;

    // automatic compass orientation on calibration
    AP_Int8     _rotate_auto;

    // throttle expressed as a percentage from 0 ~ 1.0, used for motor compensation
    float       _thr;

    struct mag_state {
        AP_Int8     external;
        bool        healthy;
        bool        registered;
        Compass::Priority priority;
        AP_Int8     orientation;
        AP_Vector3f offset;
#if AP_COMPASS_DIAGONALS_ENABLED
        AP_Vector3f diagonals;
        AP_Vector3f offdiagonals;
#endif
        AP_Float    scale_factor;

        // device id detected at init.
        // saved to eeprom when offsets are saved allowing ram &
        // eeprom values to be compared as consistency check
        AP_Int32    dev_id;
        // Initialised when compass is detected
        int32_t detected_dev_id;
        // Initialised at boot from saved devid
        int32_t expected_dev_id;

        // factors multiplied by throttle and added to compass outputs
        AP_Vector3f motor_compensation;

        // latest compensation added to compass
        Vector3f    motor_offset;

        // corrected magnetic field strength
        Vector3f    field;

        // when we last got data
        uint32_t    last_update_ms;
        uint32_t    last_update_usec;

        // board specific orientation
        enum Rotation rotation;

        // accumulated samples, protected by _sem, used by AP_Compass_Backend
        Vector3f accum;
        uint32_t accum_count;
        // We only copy persistent params
        void copy_from(const mag_state& state);
    };

    //Create an Array of mag_state to be accessible by StateIndex only
    RestrictIDTypeArray<mag_state, COMPASS_MAX_INSTANCES+1, StateIndex> _state;

    //Convert Priority to StateIndex
    StateIndex _get_state_id(Priority priority) const;
    //Get State Struct by Priority
    const struct mag_state& _get_state(Priority priority) const { return _state[_get_state_id(priority)]; }
    //Convert StateIndex to Priority
    Priority _get_priority(StateIndex state_id) { return _state[state_id].priority; }
    //Method to detect compass beyond initialisation stage
    void _detect_runtime(void);
    // This method reorganises devid list to match
    // priority list, only call before detection at boot
#if COMPASS_MAX_INSTANCES > 1
    void _reorder_compass_params();
#endif
    // Update Priority List for Mags, by default, we just
    // load them as they come up the first time
    Priority _update_priority_list(int32_t dev_id);
    
    // method to check if the mag with the devid 
    // is a replacement mag
    bool is_replacement_mag(uint32_t dev_id);

    //remove the devid from unreg compass list
    void remove_unreg_dev_id(uint32_t devid);

    void _reset_compass_id();
    //Create Arrays to be accessible by Priority only
    RestrictIDTypeArray<AP_Int8, COMPASS_MAX_INSTANCES, Priority> _use_for_yaw;
#if COMPASS_MAX_INSTANCES > 1
    RestrictIDTypeArray<AP_Int32, COMPASS_MAX_INSTANCES, Priority> _priority_did_stored_list;
    RestrictIDTypeArray<int32_t, COMPASS_MAX_INSTANCES, Priority> _priority_did_list;
#endif

    AP_Int16 _offset_max;

    // bitmask of options
    enum class Option : uint16_t {
        CAL_REQUIRE_GPS = (1U<<0),
        ALLOW_DRONECAN_AUTO_REPLACEMENT = (1U<<1),
    };
    bool option_set(Option opt) const { return (_options.get() & uint16_t(opt)) != 0; }
    AP_Int16 _options;

#if COMPASS_CAL_ENABLED
    RestrictIDTypeArray<CompassCalibrator*, COMPASS_MAX_INSTANCES, Priority> _calibrator;
#endif

#if COMPASS_MOT_ENABLED
    // per-motor compass compensation
    Compass_PerMotor _per_motor{*this};
#endif
    
    AP_Float _calibration_threshold;

    // mask of driver types to not load. Bit positions match DEVTYPE_ in backend
    AP_Int32 _driver_type_mask;

#if COMPASS_MAX_UNREG_DEV
    // Put extra dev ids detected
    AP_Int32 extra_dev_id[COMPASS_MAX_UNREG_DEV];
    uint32_t _previously_unreg_mag[COMPASS_MAX_UNREG_DEV];
#endif

    AP_Int8 _filter_range;

    CompassLearn *learn;
    bool learn_allocated;

    /// Sets the initial location used to get declination
    ///
    /// @param  latitude             GPS Latitude.
    /// @param  longitude            GPS Longitude.
    ///
    void try_set_initial_location();
    bool _initial_location_set;

    bool _cal_thread_started;

#if AP_COMPASS_MSP_ENABLED
    uint8_t msp_instance_mask;
#endif
    bool init_done;

    bool suppress_devid_save;

    uint8_t _first_usable; // first compass usable based on COMPASSx_USE param
};

namespace AP {
    Compass &compass();
};
