/**
 * @file AP_Airspeed_SITL.h
 * @brief SITL (Software In The Loop) simulation airspeed sensor backend
 * 
 * @details This file implements a perfect airspeed sensor for SITL simulation testing.
 *          Unlike hardware backends that interface with physical I2C/SPI sensors, this
 *          backend retrieves ideal airspeed data directly from the SITL aircraft physics
 *          model, providing deterministic measurements for algorithm development, testing,
 *          and continuous integration validation.
 * 
 *          Key characteristics:
 *          - Perfect measurements by default (zero noise, zero latency)
 *          - No hardware initialization or communication overhead
 *          - Always available in SITL simulation environments
 *          - Deterministic for reproducible testing
 *          - Can be extended with configurable noise/delay for realistic testing
 * 
 * @note This backend is only available when AP_AIRSPEED_SITL_ENABLED is defined
 * @note Critical for automated testing, CI/CD validation, and algorithm development
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_SITL.h
 * Related: libraries/AP_Airspeed/AP_Airspeed_SITL.cpp, libraries/SITL/SITL.h
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SITL_ENABLED

#include "AP_Airspeed_Backend.h"

/**
 * @class AP_Airspeed_SITL
 * @brief SITL simulation airspeed sensor providing ideal airspeed measurements
 * 
 * @details This backend provides perfect airspeed sensor data for Software In The Loop
 *          (SITL) simulation testing. Unlike hardware-based airspeed sensors that must
 *          handle I2C/SPI communication, sensor initialization, calibration, and noise,
 *          this implementation retrieves airspeed data directly from the SITL aircraft
 *          physics simulation model.
 * 
 *          Architecture:
 *          - Inherits from AP_Airspeed_Backend for polymorphic sensor interface
 *          - Connects to SITL physics model via AP::sitl()->state
 *          - Retrieves differential pressure from airspeed_raw_pressure[] array
 *          - Calculates atmospheric temperature from simulated altitude
 * 
 *          Use Cases:
 *          - Algorithm development and validation without hardware
 *          - Automated testing in continuous integration pipelines
 *          - Deterministic testing for debugging and verification
 *          - Multi-sensor simulation (supports AIRSPEED_MAX_SENSORS instances)
 * 
 *          Simulation Characteristics:
 *          - Perfect measurements: No sensor noise by default
 *          - Instant response: No communication latency
 *          - Always available: No initialization failures
 *          - Deterministic: Same inputs produce same outputs
 *          - Multi-instance: Supports multiple simulated sensors
 * 
 *          Integration with SITL Framework:
 *          The SITL physics model (libraries/SITL/) computes true airspeed based on
 *          vehicle velocity relative to wind, then converts to differential pressure
 *          using the pitot tube equation. This backend simply reads that pre-computed
 *          pressure value, simulating a perfect pitot-static system.
 * 
 * @note This sensor is perfect by default - noise and delay can be added to SITL
 *       model for realistic sensor simulation if needed
 * @note Only available in SITL builds (controlled by AP_AIRSPEED_SITL_ENABLED)
 * @note Does not interact with any hardware or perform I2C/SPI communication
 * 
 * @warning This backend should NEVER be compiled into production firmware - it is
 *          strictly for simulation testing purposes
 * 
 * @see AP_Airspeed_Backend for base sensor interface
 * @see libraries/SITL/SITL.h for physics model state access
 * @see libraries/AP_Airspeed/AP_Airspeed.cpp for sensor selection logic
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_SITL.h:12-29
 */
class AP_Airspeed_SITL : public AP_Airspeed_Backend
{
public:

    /**
     * @brief Constructor - uses base class constructor
     * 
     * @details Inherits AP_Airspeed_Backend constructor for standard initialization.
     *          No additional initialization required for simulation backend.
     */
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    /**
     * @brief Initialize the SITL airspeed sensor backend
     * 
     * @details Registers the SITL airspeed interface with the sensor framework.
     *          For simulation backend, initialization always succeeds since no hardware
     *          probing, I2C communication, or sensor configuration is required. The
     *          SITL physics model is always available in simulation environments.
     * 
     * @return true - simulation sensor is always available and ready
     * 
     * @note Called during sensor initialization phase at boot
     * @note Unlike hardware backends, this never fails or requires retry logic
     * @note No delay or blocking operations - returns immediately
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_SITL.h:18-20
     */
    bool init(void) override {
        return true;
    }

    /**
     * @brief Retrieve current simulated differential pressure measurement
     * 
     * @details Returns the differential pressure computed by the SITL aircraft physics
     *          model. The pressure is calculated from the vehicle's true airspeed
     *          (ground velocity minus wind velocity) using the pitot tube equation:
     *          
     *          Differential Pressure = 0.5 * air_density * airspeed^2
     *          
     *          The SITL physics model handles this calculation and stores the result
     *          in state.airspeed_raw_pressure[] array. This method retrieves that
     *          pre-computed value for the sensor instance.
     *          
     *          Multi-Sensor Support:
     *          Validates instance number against AIRSPEED_MAX_SENSORS to support
     *          multiple simulated airspeed sensors on a single vehicle (useful for
     *          testing sensor fusion and redundancy logic).
     * 
     * @param[out] pressure Simulated differential pressure in Pascals (Pa)
     *                      Typical range: 0-1000 Pa for 0-50 m/s airspeed
     *                      Computed from SITL physics model, no sensor noise by default
     * 
     * @return true if pressure retrieved successfully, false if instance number invalid
     * 
     * @note Called at sensor update rate (typically 10-50 Hz depending on sensor type)
     * @note Pressure is in Pascals (SI unit) - conversion to airspeed happens in AP_Airspeed
     * @note Perfect measurement by default - SITL model can add noise if configured
     * @note For realistic testing, noise parameters can be added to SITL physics model
     * 
     * @see AP::sitl()->state.airspeed_raw_pressure[] for pressure source
     * @see libraries/SITL/SIM_Aircraft.cpp for pressure calculation
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_SITL.cpp:9-20
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Retrieve current simulated temperature measurement
     * 
     * @details Returns atmospheric temperature at the vehicle's current altitude,
     *          calculated using the standard atmospheric model. The temperature is
     *          computed from the simulated altitude using barometric temperature
     *          lapse rate formula.
     *          
     *          Temperature Calculation:
     *          Uses AP_Baro::get_temperatureC_for_alt_amsl() which implements the
     *          International Standard Atmosphere (ISA) model:
     *          - Sea level: 15°C (288.15 K)
     *          - Lapse rate: -6.5°C per 1000m up to 11km
     *          - Stratosphere: isothermal at -56.5°C above 11km
     *          
     *          Multi-Sensor Support:
     *          Validates instance number against AIRSPEED_MAX_SENSORS to support
     *          multiple simulated sensors.
     * 
     * @param[out] temperature Simulated atmospheric temperature in degrees Celsius (°C)
     *                         Calculated from ISA model based on simulated altitude
     *                         Typical range: -60°C to +50°C depending on altitude
     * 
     * @return true if temperature retrieved successfully, false if instance number invalid
     * 
     * @note Called at sensor update rate when temperature compensation is enabled
     * @note Temperature is in degrees Celsius (not Kelvin or Fahrenheit)
     * @note Uses atmospheric model - does not simulate sensor board self-heating
     * @note Future enhancement: Could add sensor board temperature offset parameter
     * 
     * @todo Add sensor board temperature offset parameter for more realistic simulation
     * 
     * @see AP_Baro::get_temperatureC_for_alt_amsl() for temperature model
     * @see libraries/AP_Baro/AP_Baro.cpp for ISA implementation
     * 
     * Source: libraries/AP_Airspeed/AP_Airspeed_SITL.cpp:23-40
     */
    bool get_temperature(float &temperature) override;

private:
    /**
     * @brief Private implementation section
     * 
     * @details This backend requires no private member variables or helper methods
     *          because all sensor data is retrieved directly from the SITL physics
     *          model state via the global AP::sitl() singleton accessor.
     *          
     *          Connection to SITL Physics Model:
     *          - Accesses AP::sitl()->state.airspeed_raw_pressure[instance]
     *          - Accesses AP::sitl()->state.altitude for temperature calculation
     *          - No local state storage or caching required
     *          - No hardware communication buffers or registers
     *          
     *          Perfect Sensor Characteristics:
     *          - Zero measurement noise (deterministic output)
     *          - Zero latency (instant access to physics state)
     *          - No calibration state or offsets needed
     *          - No communication error handling required
     *          - No filtering or smoothing needed
     *          
     *          For Realistic Testing:
     *          If sensor noise, bias, or latency simulation is desired, these
     *          characteristics should be added to the SITL physics model itself
     *          (in libraries/SITL/) rather than in this backend, ensuring all
     *          simulated sensors can share common noise models.
     * 
     * @note No private members needed - all data from SITL global state
     * @note Stateless design enables multiple instances without cross-talk
     * @note Perfect sensor model simplifies algorithm testing and validation
     */
};

#endif // AP_AIRSPEED_SITL_ENABLED
