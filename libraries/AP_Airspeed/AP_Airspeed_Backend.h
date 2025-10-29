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
#pragma once

/**
 * @file AP_Airspeed_Backend.h
 * @brief Abstract base class for airspeed sensor backend drivers
 * 
 * @details This file defines the AP_Airspeed_Backend interface that all
 *          airspeed sensor drivers must implement. The backend provides
 *          hardware abstraction for various airspeed sensor types including
 *          differential pressure sensors (MS4525, MS5525, DLVR, SDP3X),
 *          analog sensors, and direct airspeed measurement devices.
 * 
 *          Backend drivers follow a standard lifecycle:
 *          1. probe() - Detect sensor presence on bus (in subclass)
 *          2. init() - Initialize sensor configuration
 *          3. register_periodic_callback() - Schedule regular updates
 *          4. get_differential_pressure() or get_airspeed() - Read measurements
 * 
 *          The frontend-backend architecture separates sensor-specific
 *          hardware access (backend) from sensor fusion, calibration, and
 *          parameter management (frontend).
 * 
 * @see AP_Airspeed for frontend implementation
 * @see libraries/AP_Airspeed/AP_Airspeed_*.cpp for specific sensor backends
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "AP_Airspeed.h"
#include <AP_MSP/msp_sensors.h>

/**
 * @class AP_Airspeed_Backend
 * @brief Abstract base class for airspeed sensor backend drivers
 * 
 * @details AP_Airspeed_Backend defines the interface contract that all
 *          airspeed sensor drivers must implement. This class provides
 *          hardware abstraction for various sensor types, allowing the
 *          frontend (AP_Airspeed) to manage multiple sensor instances
 *          uniformly without knowing sensor-specific details.
 * 
 *          **Architecture Pattern**: Frontend-Backend
 *          - Frontend: AP_Airspeed manages calibration, parameters, sensor fusion
 *          - Backend: Sensor-specific hardware access and data retrieval
 * 
 *          **Backend Lifecycle**:
 *          1. Construction: Backend instantiated by frontend with instance index
 *          2. Probe: Subclass detects sensor on bus (I2C/SPI/analog)
 *          3. init(): Configure sensor registers, measurement mode
 *          4. Periodic updates: Scheduled callback reads sensor data
 *          5. get_differential_pressure() or get_airspeed(): Frontend queries data
 * 
 *          **Thread Safety**:
 *          The sem (HAL_Semaphore) member protects access to shared frontend
 *          state. Backends should acquire this semaphore when accessing
 *          frontend data structures or parameters.
 * 
 *          **Sensor Types Supported**:
 *          - Differential pressure sensors (most common): Measure pitot tube pressure
 *          - Direct airspeed sensors: Calculate airspeed internally (rare)
 *          - MSP/External: Receive data from external flight controllers
 *          - Analog: Legacy voltage-based sensors
 * 
 *          **Coordinate Systems**: Not applicable - airspeed is scalar magnitude
 * 
 *          **Calibration**: Backend provides raw measurements; frontend handles
 *          offset calibration, ratio scaling, and temperature compensation
 * 
 * @note Backends are instantiated per sensor instance (up to AIRSPEED_MAX_SENSORS)
 * @note Most methods are virtual and have default implementations returning false
 * @warning Sensor initialization must complete quickly to avoid blocking vehicle startup
 * 
 * @see AP_Airspeed (frontend implementation)
 * @see AP_Airspeed_MS4525 (example differential pressure backend)
 * @see AP_Airspeed_NMEA (example direct airspeed backend)
 */
class AP_Airspeed_Backend {
public:
    /**
     * @brief Construct an airspeed sensor backend instance
     * 
     * @param[in] frontend Reference to AP_Airspeed frontend manager
     * @param[in] instance Sensor instance index (0-based, typically 0 to AIRSPEED_MAX_SENSORS-1)
     * 
     * @note Called by frontend when creating backend for detected sensor
     * @note Subclasses should perform minimal work in constructor; defer initialization to init()
     */
    AP_Airspeed_Backend(AP_Airspeed &frontend, uint8_t instance);
    
    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~AP_Airspeed_Backend() {}

    /**
     * @brief Probe for sensor presence and initialize sensor configuration
     * 
     * @details This pure virtual method must be implemented by all backend subclasses.
     *          Implementation should:
     *          1. Verify sensor presence on configured bus (I2C/SPI/analog pin)
     *          2. Read sensor ID registers to confirm sensor type
     *          3. Configure sensor measurement mode and resolution
     *          4. Register periodic callback for data acquisition
     *          5. Perform initial sensor health check
     * 
     *          This method is called during vehicle startup. It must complete
     *          quickly (typically <100ms) to avoid delaying vehicle initialization.
     * 
     * @return true if sensor successfully probed and initialized
     * @return false if sensor not found, communication failed, or configuration error
     * 
     * @note Called once during vehicle startup after backend construction
     * @note Returning false will disable this airspeed sensor instance
     * @warning Must not block indefinitely; use timeouts for bus communication
     * @warning Sensor not available after init() returns false
     */
    virtual bool init(void) = 0;

    /**
     * @brief Get current differential pressure measurement from pitot tube
     * 
     * @details Returns the pressure difference between pitot (dynamic) and static
     *          pressure ports. This is the primary measurement method for most
     *          airspeed sensors. The frontend converts differential pressure to
     *          airspeed using the relationship: velocity = sqrt(2 * dP / air_density).
     * 
     *          Positive pressure indicates forward airflow. Tube polarity is
     *          configurable via ARSPD_TUBE_ORDR parameter (handled by frontend).
     * 
     *          Default implementation returns false, indicating this backend
     *          does not provide differential pressure (e.g., direct airspeed sensors).
     * 
     * @param[out] pressure Differential pressure in Pascals (Pa)
     *                      Typical range: -1000 to +1000 Pa for ±100 m/s airspeed
     *                      1 Pa ≈ 0.01 mbar ≈ 0.004 inH2O
     * 
     * @return true if valid pressure measurement available
     * @return false if sensor error, no new data, or backend provides direct airspeed
     * 
     * @note Called at main loop rate (typically 50-400 Hz depending on vehicle type)
     * @note Frontend applies offset calibration and ratio scaling after retrieving raw pressure
     * @note Unit: Pascals (Pa), never millibar or inH2O
     * 
     * @see get_airspeed() for backends that measure airspeed directly
     */
    virtual bool get_differential_pressure(float &pressure) {return false;}

    /**
     * @brief Get current temperature measurement from sensor
     * 
     * @details Returns temperature measured by the airspeed sensor's internal
     *          thermometer. Temperature is used by the frontend for:
     *          - Air density calculation (affects pressure-to-airspeed conversion)
     *          - Temperature compensation of sensor characteristics
     *          - Diagnostic monitoring of sensor health
     * 
     *          This is a pure virtual method; all backends must implement it.
     *          Backends for sensors without temperature capability should return
     *          false or estimate temperature from other sources.
     * 
     * @param[out] temperature Air temperature in degrees Celsius (°C)
     *                         Typical range: -40°C to +85°C (sensor dependent)
     *                         Accuracy typically ±2°C to ±5°C
     * 
     * @return true if valid temperature measurement available
     * @return false if sensor error, no temperature sensor, or data not ready
     * 
     * @note Called at main loop rate alongside pressure measurement
     * @note Not all airspeed sensors include temperature measurement capability
     * @note Unit: Degrees Celsius (°C), never Fahrenheit or Kelvin
     * @warning Inaccurate temperature significantly affects airspeed calculation at high altitudes
     */
    virtual bool get_temperature(float &temperature) = 0;

    /**
     * @brief Query whether backend provides direct airspeed measurement
     * 
     * @details Most airspeed sensors measure differential pressure and rely on
     *          the frontend to convert pressure to airspeed. Some sensors
     *          (e.g., NMEA wind sensors, certain ultrasonic sensors) measure
     *          airspeed directly and return it via get_airspeed() instead.
     * 
     *          When this returns true, the frontend will call get_airspeed()
     *          instead of get_differential_pressure().
     * 
     * @return true if backend provides direct airspeed via get_airspeed()
     * @return false if backend provides differential pressure via get_differential_pressure()
     * 
     * @note Default implementation returns false (pressure-based sensor)
     * @note Only one of {get_differential_pressure(), get_airspeed()} should be implemented
     */
    virtual bool has_airspeed() {return false;}

    /**
     * @brief Get current airspeed measurement directly from sensor
     * 
     * @details For sensors that calculate airspeed internally rather than
     *          providing raw differential pressure. Examples include:
     *          - NMEA wind sensors (calculate true airspeed from wind data)
     *          - Ultrasonic airspeed sensors (time-of-flight measurement)
     *          - External flight controllers via MSP or external AHRS
     * 
     *          This method is only called if has_airspeed() returns true.
     *          Default implementation returns false for pressure-based sensors.
     * 
     * @param[out] airspeed Measured airspeed in meters per second (m/s)
     *                      Typical range: 0 to 100+ m/s depending on vehicle type
     *                      Should represent indicated airspeed (IAS) or true airspeed (TAS)
     *                      depending on sensor type
     * 
     * @return true if valid airspeed measurement available
     * @return false if sensor error, no new data, or backend uses pressure measurement
     * 
     * @note Unit: Meters per second (m/s), never km/h, mph, or knots
     * @note Frontend still applies ratio calibration scaling to direct airspeed
     * 
     * @see has_airspeed() to indicate backend provides direct airspeed
     * @see get_differential_pressure() for pressure-based measurement
     */
    virtual bool get_airspeed(float& airspeed) {return false;}

    /**
     * @brief Handle airspeed data received via MSP protocol
     * 
     * @details MultiWii Serial Protocol (MSP) integration allows receiving
     *          airspeed measurements from external flight controllers or
     *          sensors that communicate via MSP. This is used when ArduPilot
     *          is receiving sensor data from another flight controller.
     * 
     *          Default implementation is empty (no-op) for hardware sensors.
     *          Only AP_Airspeed_MSP backend implements this method.
     * 
     * @param[in] pkt MSP airspeed data packet containing:
     *                - Airspeed measurement (m/s or pressure depending on MSP message)
     *                - Temperature (if available)
     *                - Sensor status flags
     * 
     * @note Only called when MSP airspeed backend is active (ARSPD_TYPE = MSP)
     * @note MSP message rate determined by external controller
     * 
     * @see AP_Airspeed_MSP for MSP backend implementation
     * @see AP_MSP library for MSP protocol details
     */
    virtual void handle_msp(const MSP::msp_airspeed_data_message_t &pkt) {}

#if AP_AIRSPEED_EXTERNAL_ENABLED
    /**
     * @brief Handle airspeed data received from external AHRS system
     * 
     * @details Receives airspeed measurements from external Attitude and Heading
     *          Reference Systems (AHRS) that provide integrated sensor data.
     *          Used when ArduPilot delegates sensor processing to external hardware
     *          (e.g., VectorNav, LORD MicroStrain, Advanced Navigation devices).
     * 
     *          Default implementation is empty (no-op) for hardware sensors.
     *          Only backends supporting external AHRS implement this method.
     * 
     * @param[in] pkt External AHRS airspeed data packet containing:
     *                - Airspeed measurement (typically true airspeed in m/s)
     *                - Temperature (if available)
     *                - Measurement timestamp
     *                - Data quality indicators
     * 
     * @note Only available when AP_AIRSPEED_EXTERNAL_ENABLED is defined
     * @note External AHRS typically provides higher-rate, time-synchronized data
     * 
     * @see AP_ExternalAHRS for external AHRS integration details
     */
    virtual void handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt) {}
#endif

#if AP_AIRSPEED_HYGROMETER_ENABLE
    /**
     * @brief Get humidity and temperature from integrated hygrometer sensor
     * 
     * @details Some advanced airspeed sensors include integrated humidity
     *          measurement capability. Humidity data is used for:
     *          - Accurate air density calculation (water vapor affects density)
     *          - Improved airspeed accuracy in humid conditions
     *          - Environmental monitoring and logging
     * 
     *          This feature is optional and only available when compiled with
     *          AP_AIRSPEED_HYGROMETER_ENABLE. Most airspeed sensors do not
     *          include hygrometer capability.
     * 
     * @param[out] last_sample_ms Timestamp of last humidity measurement in milliseconds
     *                            (from AP_HAL::millis(), used to check data freshness)
     * @param[out] temperature Temperature measured by hygrometer in degrees Celsius (°C)
     *                         May differ slightly from main sensor temperature
     * @param[out] humidity Relative humidity as percentage (0.0 to 100.0%)
     *                     0% = completely dry air, 100% = fully saturated
     * 
     * @return true if valid hygrometer data available
     * @return false if no hygrometer, sensor error, or data stale
     * 
     * @note Default implementation returns false (no hygrometer)
     * @note Only specific sensor backends support this (check sensor datasheet)
     * @note Humidity measurements typically updated at lower rate than pressure (1-10 Hz)
     * @note Units: Temperature in °C, humidity in percent (%)
     */
    virtual bool get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity) { return false; }
#endif

protected:
    /**
     * @brief Get configured analog pin number for this sensor instance
     * 
     * @return int8_t Analog pin number configured via ARSPD_PIN parameter
     *                -1 if no pin configured (digital sensors use bus instead)
     * 
     * @note Only used by analog airspeed sensor backends
     * @note Digital sensors (I2C/SPI) return -1 and use get_bus() instead
     */
    int8_t get_pin(void) const;
    
    /**
     * @brief Get configured pressure range for this sensor instance
     * 
     * @return float Sensor pressure range in PSI (pounds per square inch)
     *               Configured via ARSPD_PSI_RANGE parameter
     *               Common values: 1.0 PSI, 2.0 PSI (depends on sensor model)
     * 
     * @note Used by backends to select appropriate sensor measurement mode
     * @note Higher range = lower resolution but higher maximum airspeed
     * @note Unit: PSI (pounds per square inch), not Pascals
     */
    float get_psi_range(void) const;
    
    /**
     * @brief Get configured I2C/SPI bus number for this sensor instance
     * 
     * @return uint8_t Bus number configured via ARSPD_BUS parameter
     *                 For I2C: bus number (0-3 typically)
     *                 For SPI: bus selection depends on board
     * 
     * @note Only used by digital sensor backends (I2C/SPI)
     * @note Analog sensors ignore this and use get_pin() instead
     */
    uint8_t get_bus(void) const;
    
    /**
     * @brief Check if bus/pin is configured for this sensor instance
     * 
     * @return true if valid bus number or pin configured
     * @return false if no bus/pin configured (sensor disabled)
     * 
     * @note Backends should check this before attempting sensor probe
     * @note Prevents unnecessary bus scanning when sensor not configured
     */
    bool bus_is_configured(void) const;
    
    /**
     * @brief Get sensor instance index
     * 
     * @return uint8_t Instance number (0-based, 0 to AIRSPEED_MAX_SENSORS-1)
     * 
     * @note Used to access instance-specific parameters and state
     * @note Typically 0 for single airspeed sensor systems
     */
    uint8_t get_instance(void) const {
        return instance;
    }

    /**
     * @brief Check if voltage correction should be disabled for this sensor
     * 
     * @details Some airspeed sensor measurements are affected by supply voltage
     *          variations. The frontend can apply voltage correction to compensate.
     *          This option allows disabling correction for sensors that don't need it
     *          or in systems with regulated supply voltage.
     * 
     * @return true if voltage correction disabled via ARSPD_OPTIONS bitmask
     * @return false if voltage correction should be applied (default)
     * 
     * @note Configured via ARSPD_OPTIONS parameter (DISABLE_VOLTAGE_CORRECTION bit)
     * @note Most digital sensors (I2C/SPI) don't require voltage correction
     * @note Analog sensors may benefit from voltage correction
     */
    bool disable_voltage_correction(void) const {
        return (frontend._options.get() & AP_Airspeed::OptionsMask::DISABLE_VOLTAGE_CORRECTION) != 0;
    }

    /**
     * @brief Get configured pitot tube connection order
     * 
     * @details Pitot tubes have two ports: dynamic (forward-facing) and static (side).
     *          Some sensors label ports differently, and tubes can be connected
     *          in either polarity. This parameter allows reversing the pressure
     *          measurement polarity to accommodate different tube connections.
     * 
     * @return AP_Airspeed::pitot_tube_order Tube order configuration:
     *         - PITOT_TUBE_ORDER_POSITIVE: Normal polarity (positive pressure = forward motion)
     *         - PITOT_TUBE_ORDER_NEGATIVE: Reversed polarity (swap ports)
     *         - PITOT_TUBE_ORDER_AUTO: Automatic detection (default)
     * 
     * @note Configured via ARSPD_TUBE_ORDR parameter
     * @note In AP_Periph builds, always returns AUTO (no parameter storage)
     * @note Incorrect polarity results in negative airspeed readings
     */
    AP_Airspeed::pitot_tube_order get_tube_order(void) const {
#ifndef HAL_BUILD_AP_PERIPH
        return AP_Airspeed::pitot_tube_order(frontend.param[instance].tube_order.get());
#else
        return AP_Airspeed::pitot_tube_order::PITOT_TUBE_ORDER_AUTO;
#endif
    }

    /**
     * @brief Semaphore for thread-safe access to shared frontend data
     * 
     * @details Protects concurrent access to frontend state, parameters, and
     *          calibration data. Backends should acquire this semaphore using
     *          WITH_SEMAPHORE(sem) when accessing frontend data structures
     *          from periodic callbacks or other threads.
     * 
     *          Thread-safety is critical because:
     *          - Backend reads sensor data in scheduled callbacks (potentially different thread)
     *          - Frontend accesses sensor data from main vehicle loop
     *          - Parameter updates can occur asynchronously from ground station
     * 
     * @note Use WITH_SEMAPHORE(sem) macro for automatic acquire/release
     * @note Semaphore should be held for minimal time (no blocking operations)
     * @warning Deadlock possible if semaphore held while calling frontend methods that acquire it
     */
    HAL_Semaphore sem;

    /**
     * @brief Get calibration ratio for airspeed scaling
     * 
     * @details The airspeed ratio is a calibration factor applied to compensate for:
     *          - Pitot tube installation position effects (local flow field disturbances)
     *          - Sensor-specific gain variations
     *          - Manufacturing tolerances
     *          - Vehicle-specific aerodynamic effects
     * 
     *          Ratio = 1.0 means no scaling. Typical range: 0.8 to 2.0
     *          Higher ratio = higher indicated airspeed for same pressure
     * 
     * @return float Airspeed ratio (dimensionless scaling factor)
     *               Configured via ARSPD_RATIO parameter
     *               Default typically 2.0 (historical ArduPilot convention)
     * 
     * @note Backend typically doesn't apply this ratio; frontend handles scaling
     * @note Can be auto-calibrated during flight using ARSPD_AUTOCAL
     */
    float get_airspeed_ratio(void) const {
        return frontend.get_airspeed_ratio(instance);
    }

    /**
     * @brief Configure sensor to use zero offset (no calibration required)
     * 
     * @details Some airspeed sensors have built-in offset compensation or are
     *          inherently accurate without requiring zero-offset calibration.
     *          This method informs the frontend that offset calibration should
     *          be skipped for this sensor.
     * 
     *          Sensors that should use zero offset:
     *          - Sensors with internal automatic zeroing
     *          - Direct airspeed measurement sensors (not pressure-based)
     *          - Pre-calibrated digital sensors
     * 
     * @note Called during backend init() if sensor doesn't require offset calibration
     * @note Sets calibration state to NOT_REQUIRED_ZERO_OFFSET
     * @note Prevents user from manually triggering offset calibration
     */
    void set_use_zero_offset(void) {
        frontend.state[instance].cal.state = AP_Airspeed::CalibrationState::NOT_REQUIRED_ZERO_OFFSET;
#ifndef HAL_BUILD_AP_PERIPH
        frontend.param[instance].offset.set(0.0);
#endif
    }

    /**
     * @brief Update ARSPD_USE parameter to enable/disable sensor
     * 
     * @details Allows backend to dynamically enable or disable the sensor instance
     *          based on runtime conditions (e.g., probe success, health monitoring).
     * 
     * @param[in] use Sensor usage setting:
     *                0 = disabled
     *                1 = enabled
     *                Negative values = backend-specific extended options
     * 
     * @note Typically called during init() to enable sensor after successful probe
     * @note In AP_Periph builds, no-op (no parameter storage)
     * @note Parameter persists across reboots
     */
    void set_use(int8_t use) {
#ifndef HAL_BUILD_AP_PERIPH
        frontend.param[instance].use.set(use);
#endif
    }

    /**
     * @brief Set hardware bus identifier for parameter storage and logging
     * 
     * @details Records the hardware bus ID (I2C address, SPI CS pin, etc.) in
     *          the ARSPD_DEVID parameter. This allows:
     *          - Parameter binding to specific hardware (survives sensor reordering)
     *          - Log analysis to identify which physical sensor produced data
     *          - Detection of sensor hardware changes between flights
     * 
     * @param[in] id Unique bus identifier encoding:
     *               - Bus type (I2C/SPI/analog)
     *               - Bus number
     *               - Device address or CS pin
     *               Format defined by AP_HAL device identification
     * 
     * @note Called during successful init() to record hardware identity
     * @note ID format matches other sensor subsystems (GPS, compass, etc.)
     * @note Allows GCS to warn if sensor hardware configuration changes
     */
    void set_bus_id(uint32_t id);

    /**
     * @enum DevType
     * @brief Device type identifiers for airspeed sensor backends
     * 
     * @details Unique identifier for each supported airspeed sensor type.
     *          Used in bus ID encoding for parameter storage and logging.
     *          Allows ground stations and log analysis tools to identify
     *          specific sensor hardware models.
     * 
     *          Each backend implementation uses one of these types when
     *          calling set_bus_id() during initialization.
     */
    enum class DevType {
        SITL     = 0x01,  ///< Software-in-the-loop simulated sensor
        MS4525   = 0x02,  ///< TE Connectivity MS4525DO differential pressure sensor (I2C)
        MS5525   = 0x03,  ///< TE Connectivity MS5525DSO differential pressure sensor (I2C)
        DLVR     = 0x04,  ///< All Sensors DLVR differential pressure sensor (I2C)
        MSP      = 0x05,  ///< MultiWii Serial Protocol airspeed data
        SDP3X    = 0x06,  ///< Sensirion SDP3x differential pressure sensor (I2C)
        UAVCAN   = 0x07,  ///< UAVCAN/DroneCAN airspeed sensor (CAN bus)
        ANALOG   = 0x08,  ///< Analog voltage-based airspeed sensor
        NMEA     = 0x09,  ///< NMEA wind sensor providing direct airspeed
        ASP5033  = 0x0A,  ///< TE Connectivity ASP5033 pressure sensor (SPI)
        AUAV     = 0x0B,  ///< AUAV airspeed sensor
    };
    
private:
    /**
     * @brief Reference to AP_Airspeed frontend manager
     * 
     * @details Provides backend access to frontend services including:
     *          - Parameter storage (ARSPD_* parameters)
     *          - Calibration state and offset data
     *          - Sensor fusion and filtering
     *          - Multi-sensor management
     * 
     * @note Backends access frontend via protected accessor methods, not directly
     * @note Frontend reference remains valid for backend lifetime
     */
    AP_Airspeed &frontend;
    
    /**
     * @brief Sensor instance index for this backend
     * 
     * @details Zero-based index identifying which sensor instance this backend
     *          represents. Used to access instance-specific parameters and state
     *          in the frontend's parameter and state arrays.
     * 
     *          Range: 0 to (AIRSPEED_MAX_SENSORS - 1)
     *          Typically 0 for systems with single airspeed sensor
     * 
     * @note Set during construction, immutable for backend lifetime
     * @note Used by get_instance() accessor method
     */
    uint8_t instance;
};

#endif  // AP_AIRSPEED_ENABLED
