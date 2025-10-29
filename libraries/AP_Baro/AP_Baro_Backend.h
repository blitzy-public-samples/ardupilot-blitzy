/**
 * @file AP_Baro_Backend.h
 * @brief Barometer backend base class defining the driver interface
 * 
 * This file defines AP_Baro_Backend, the abstract base class that all barometer
 * drivers must inherit from. It provides the common interface contract for
 * barometer sensor integration, including pressure/temperature data transfer,
 * health monitoring, and outlier rejection filtering.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Baro.h"

/**
 * @class AP_Baro_Backend
 * @brief Abstract base class defining contract for all barometer drivers
 * 
 * @details AP_Baro_Backend provides the common interface that all barometer sensor
 *          drivers must implement. It handles the interaction between hardware-specific
 *          drivers and the AP_Baro frontend, managing data transfer, health monitoring,
 *          and validation.
 *          
 *          Driver Lifecycle:
 *          1. probe() - Static factory method detects hardware (in derived class)
 *          2. Constructor - Initializes backend with reference to frontend
 *          3. Driver calls backend_update() periodically via scheduler callback
 *          4. update() - Pure virtual method where driver pushes accumulated data
 *          5. _copy_to_frontend() - Transfers validated pressure/temperature data
 *          
 *          Thread Safety:
 *          - Uses _sem (HAL_Semaphore) to protect concurrent access to frontend sensor array
 *          - Drivers must use WITH_SEMAPHORE(_sem) when accessing _frontend.sensors[]
 *          
 *          Data Validation:
 *          - pressure_ok() implements mean-filter based outlier rejection
 *          - update_healthy_flag() monitors sensor health and timeout conditions
 *          - _error_count tracks dropped samples for reliability metrics
 *          
 *          Device Identification:
 *          - Each driver sets unique BARO_DEVID via set_bus_id()
 *          - DevTypes enum provides device type identifiers visible to users
 * 
 * @note All derived drivers must implement the pure virtual update() method
 * @warning Accessing _frontend.sensors[] without _sem protection causes race conditions
 * 
 * @see AP_Baro for frontend management
 * @see DevTypes for device type identifier values
 */
class AP_Baro_Backend
{
public:
    /**
     * @brief Construct a new barometer backend instance
     * 
     * @param[in] baro Reference to the AP_Baro frontend that manages this backend
     * 
     * @note The frontend reference is stored for the lifetime of the backend
     * @note Initialization of hardware should occur in derived class constructors
     */
    AP_Baro_Backend(AP_Baro &baro);
    
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~AP_Baro_Backend(void) {};

    /**
     * @brief Push accumulated sensor data to the frontend (pure virtual)
     * 
     * @details Each driver must implement this method to transfer accumulated
     *          pressure and temperature measurements to the frontend. This is
     *          called periodically by backend_update() to maintain current data.
     *          
     *          Typical implementation:
     *          1. Read pressure and temperature from hardware or accumulated buffer
     *          2. Apply any driver-specific calibration or compensation
     *          3. Call _copy_to_frontend() to transfer data with semaphore protection
     * 
     * @note This method is called at the rate configured by the scheduler callback
     * @note Must be implemented by all derived driver classes
     * 
     * @see backend_update() for the caller of this method
     * @see _copy_to_frontend() for transferring data safely to frontend
     */
    virtual void update() = 0;

    /**
     * @brief Common backend update logic wrapper
     * 
     * @details Calls the driver's update() implementation and performs common
     *          backend processing such as health monitoring and timeout detection.
     *          This is typically registered as the scheduler callback for the driver.
     * 
     * @param[in] instance Sensor instance number (0-based index into frontend sensors array)
     * 
     * @note This is the entry point for periodic driver updates from the scheduler
     * @note Called at the frequency determined by the driver's registered callback rate
     * 
     * @see update() for the driver-specific implementation
     */
    void backend_update(uint8_t instance);

    /**
     * @brief Validate pressure reading using mean-filter outlier rejection
     * 
     * @details Implements a simple range filter to reject pressure readings that
     *          deviate too far from the running mean. This helps eliminate transient
     *          spikes and sensor glitches without complex filtering.
     *          
     *          Algorithm:
     *          - Maintains running mean of pressure values in _mean_pressure
     *          - Rejects readings outside filter_range from mean
     *          - Increments _error_count when rejecting samples
     *          - Updates mean when accepting valid samples
     * 
     * @param[in] press Pressure reading to validate in Pascals
     * 
     * @return true if pressure is within acceptable range and should be used
     * @return false if pressure is an outlier and should be rejected
     * 
     * @note Filter range threshold defined in AP_Baro frontend configuration
     * @note Rejected samples are counted in _error_count for reliability metrics
     * 
     * @see get_error_count() for accessing rejection statistics
     */
    bool pressure_ok(float press);
    
    /**
     * @brief Get count of rejected/dropped pressure samples
     * 
     * @return uint32_t Number of samples rejected by pressure_ok() filter
     * 
     * @note Can be used to assess sensor reliability and select best sensor
     * @note Count is cumulative since backend initialization
     */
    uint32_t get_error_count() const { return _error_count; }

#if AP_BARO_MSP_ENABLED
    /**
     * @brief Handle barometer data from MSP (MultiWii Serial Protocol)
     * 
     * @details Called by MSP protocol handler to inject external barometer data
     *          from MSP-compatible devices (e.g., MSP OSD). Default implementation
     *          is empty; MSP backend overrides to process the data.
     * 
     * @param[in] pkt MSP barometer data packet containing pressure and temperature
     * 
     * @note Only compiled when AP_BARO_MSP_ENABLED is defined
     * @note Default implementation does nothing; overridden by AP_Baro_MSP backend
     * 
     * @see AP_Baro_MSP for implementation
     */
    virtual void handle_msp(const MSP::msp_baro_data_message_t &pkt) {}
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    /**
     * @brief Handle barometer data from external AHRS system
     * 
     * @details Called by external AHRS handler to inject barometer data from
     *          external navigation systems (e.g., VectorNav, MicroStrain). Default
     *          implementation is empty; external AHRS backend overrides to process.
     * 
     * @param[in] pkt External AHRS barometer data packet with pressure and temperature
     * 
     * @note Only compiled when AP_BARO_EXTERNALAHRS_ENABLED is defined
     * @note Default implementation does nothing; overridden by external AHRS backend
     * 
     * @see AP_ExternalAHRS for external navigation system integration
     */
    virtual void handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt) {}
#endif

    /**
     * @enum DevTypes
     * @brief Device type identifiers for barometer sensors
     * 
     * @details These values are used to populate the devtype field of the device ID,
     *          which appears in BARO_DEVID parameters visible to users and ground
     *          control stations. Each barometer driver sets its unique device type
     *          to enable identification of the specific hardware in use.
     *          
     *          Device IDs are displayed in ground station configuration pages and
     *          logged for hardware configuration tracking and support diagnostics.
     * 
     * @note Values must remain stable across firmware versions for parameter compatibility
     * @note New device types should be appended to maintain backward compatibility
     * 
     * @see set_bus_id() for setting the device identification
     */
    enum DevTypes {
        DEVTYPE_BARO_SITL     = 0x01,  ///< Software In The Loop simulation barometer
        DEVTYPE_BARO_BMP085   = 0x02,  ///< Bosch BMP085 (legacy)
        DEVTYPE_BARO_BMP280   = 0x03,  ///< Bosch BMP280
        DEVTYPE_BARO_BMP388   = 0x04,  ///< Bosch BMP388
        DEVTYPE_BARO_DPS280   = 0x05,  ///< Infineon DPS280
        DEVTYPE_BARO_DPS310   = 0x06,  ///< Infineon DPS310
        DEVTYPE_BARO_FBM320   = 0x07,  ///< Formosa FBM320
        DEVTYPE_BARO_ICM20789 = 0x08,  ///< InvenSense ICM-20789 (IMU with integrated baro)
        DEVTYPE_BARO_KELLERLD = 0x09,  ///< Keller LD series pressure sensor
        DEVTYPE_BARO_LPS2XH   = 0x0A,  ///< ST LPS22H/LPS25H
        DEVTYPE_BARO_MS5611   = 0x0B,  ///< Measurement Specialties MS5611
        DEVTYPE_BARO_SPL06    = 0x0C,  ///< Goertek SPL06
        DEVTYPE_BARO_UAVCAN   = 0x0D,  ///< DroneCAN/UAVCAN barometer
        DEVTYPE_BARO_MSP      = 0x0E,  ///< MSP (MultiWii Serial Protocol) external baro
        DEVTYPE_BARO_ICP101XX = 0x0F,  ///< InvenSense ICP-101xx
        DEVTYPE_BARO_ICP201XX = 0x10,  ///< InvenSense ICP-201xx
        DEVTYPE_BARO_MS5607   = 0x11,  ///< Measurement Specialties MS5607
        DEVTYPE_BARO_MS5837_30BA = 0x12,  ///< Measurement Specialties MS5837-30BA (underwater 30 bar)
        DEVTYPE_BARO_MS5637   = 0x13,  ///< Measurement Specialties MS5637
        DEVTYPE_BARO_BMP390   = 0x14,  ///< Bosch BMP390
        DEVTYPE_BARO_BMP581   = 0x15,  ///< Bosch BMP581
        DEVTYPE_BARO_SPA06    = 0x16,  ///< Goertek SPA06
        DEVTYPE_BARO_AUAV     = 0x17,  ///< AUAV sensor module
        DEVTYPE_BARO_MS5837_02BA = 0x18,  ///< Measurement Specialties MS5837-02BA (underwater 2 bar)
    };

protected:
    /**
     * @brief Reference to frontend object managing all barometer instances
     * 
     * @details Used by backend to access frontend sensor array and configuration.
     *          Access to _frontend.sensors[] must be protected by _sem semaphore
     *          to prevent race conditions with the main thread.
     */
    AP_Baro &_frontend;

    /**
     * @brief Copy pressure and temperature data to frontend sensor array
     * 
     * @details Transfers validated sensor readings to the frontend with proper
     *          semaphore protection. Updates the sensor's pressure, temperature,
     *          and last update timestamp.
     *          
     *          This method handles the semaphore locking automatically and should
     *          be used by all drivers to push data to the frontend.
     * 
     * @param[in] instance Sensor instance number (0-based index into sensors array)
     * @param[in] pressure Barometric pressure reading in Pascals (Pa)
     * @param[in] temperature Temperature reading in degrees Celsius (°C)
     * 
     * @note Automatically acquires _sem for thread-safe access to frontend
     * @note Updates sensor last_update_ms timestamp to current time
     * 
     * @see update() for typical usage context
     */
    void _copy_to_frontend(uint8_t instance, float pressure, float temperature);

    /**
     * @brief Semaphore for thread-safe access to shared frontend data
     * 
     * @details Protects concurrent access to the _frontend.sensors[] array from
     *          multiple threads. Backend update callbacks run in scheduler context
     *          while frontend may be accessed from main thread or other tasks.
     *          
     *          Always use WITH_SEMAPHORE(_sem) when directly accessing frontend data.
     * 
     * @warning Accessing _frontend.sensors[] without this semaphore causes race conditions
     * 
     * @see _copy_to_frontend() for automatic semaphore-protected data transfer
     */
    HAL_Semaphore _sem;

    /**
     * @brief Update sensor health status based on data validity and timeouts
     * 
     * @details Called periodically to assess sensor health by checking:
     *          - Data update frequency (timeout detection via BARO_TIMEOUT_MS)
     *          - Data change detection (frozen sensor via BARO_DATA_CHANGE_TIMEOUT_MS)
     *          - Driver-specific health indicators
     *          
     *          Sets the healthy flag in the frontend sensor structure.
     * 
     * @param[in] instance Sensor instance number (0-based index into sensors array)
     * 
     * @note Timeout thresholds defined in AP_Baro.h
     * @note Derived classes may override to add device-specific health checks
     * 
     * @see BARO_TIMEOUT_MS in AP_Baro.h
     * @see BARO_DATA_CHANGE_TIMEOUT_MS in AP_Baro.h
     */
    virtual void update_healthy_flag(uint8_t instance);

    /**
     * @brief Running mean of pressure for outlier rejection filter
     * 
     * @details Maintains moving average of pressure readings used by pressure_ok()
     *          to detect and reject outliers. Updated with each valid pressure sample.
     *          
     *          Units: Pascals (Pa)
     * 
     * @see pressure_ok() for outlier rejection algorithm
     */
    float _mean_pressure; 
    
    /**
     * @brief Count of rejected/dropped pressure samples
     * 
     * @details Tracks number of samples rejected by pressure_ok() filter due to
     *          being outside acceptable range. Can be used to assess sensor reliability
     *          and potentially select the most reliable sensor in multi-baro systems.
     *          
     *          Incremented each time pressure_ok() returns false.
     * 
     * @note Not currently used for sensor selection but available for future use
     * @note Cumulative count since backend initialization
     * 
     * @see pressure_ok() for rejection criteria
     * @see get_error_count() for accessing this value
     */
    uint32_t _error_count;

    /**
     * @brief Set device identification for BARO_DEVID parameter
     * 
     * @details Configures the bus ID that appears in the BARO_DEVID parameter,
     *          allowing users and ground stations to identify the specific hardware
     *          in use. The bus ID encodes the device type, bus number, and address.
     *          
     *          Typically called during driver initialization after hardware detection.
     * 
     * @param[in] instance Sensor instance number (0-based index into sensors array)
     * @param[in] id Complete device ID encoding bus type, bus number, address, and devtype
     * 
     * @note Device ID is visible to users as BARO_DEVIDn parameter
     * @note Logged for hardware configuration tracking and diagnostics
     * 
     * @see DevTypes for device type identifier values
     * @see AP_HAL::Device::make_bus_id() for bus ID construction
     */
    void set_bus_id(uint8_t instance, uint32_t id) {
        _frontend.sensors[instance].bus_id.set(int32_t(id));
    }
};
