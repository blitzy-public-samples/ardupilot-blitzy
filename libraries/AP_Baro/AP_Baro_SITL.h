/**
 * @file AP_Baro_SITL.h
 * @brief Software-In-The-Loop (SITL) barometer simulation backend
 * 
 * @details This file implements a simulated barometer sensor for use in
 *          Hardware-In-Loop (HIL) and Software-In-Loop (SITL) testing environments.
 *          It generates synthetic pressure and temperature data based on simulated
 *          vehicle altitude using the 1976 International Standard Atmosphere model.
 *          
 *          The simulation includes realistic sensor characteristics:
 *          - Configurable measurement delay buffer
 *          - Sensor drift over time
 *          - Gaussian noise injection
 *          - Temperature effects on pressure readings
 *          - Wind pressure corrections for dynamic flight
 *          
 *          This backend enables comprehensive autopilot testing without requiring
 *          physical hardware, supporting development, CI/CD testing, and algorithm
 *          validation in simulated environments.
 * 
 * @note This backend is only compiled when AP_SIM_BARO_ENABLED is defined
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_SIM_BARO_ENABLED

#include <AP_Math/vectorN.h>

#include <SITL/SITL.h>

/**
 * @class AP_Baro_SITL
 * @brief Simulated barometer backend for SITL and HIL testing
 * 
 * @details This class provides a software simulation of a barometric pressure sensor
 *          for testing ArduPilot without physical hardware. It synthesizes realistic
 *          pressure and temperature measurements based on the simulated vehicle's
 *          altitude and environmental conditions.
 *          
 *          Pressure Synthesis:
 *          - Uses the 1976 International Standard Atmosphere model to calculate
 *            pressure from altitude: P = P0 * (1 - L*h/T0)^(g*M/R*L)
 *            where P0 = 101325 Pa (sea level pressure)
 *          - Pressure values are in Pascals (Pa)
 *          - Realistic pressure lapse rate with altitude
 *          
 *          Temperature Synthesis:
 *          - Temperature decreases with altitude following standard lapse rate
 *          - Base temperature adjustable via SITL parameters
 *          - Temperature values in degrees Celsius
 *          - Accounts for board temperature effects on sensor readings
 *          
 *          Realistic Sensor Characteristics:
 *          - Measurement Delay: Configurable delay buffer simulates sensor latency
 *          - Drift: Integrated altitude drift over time (configurable via SITL params)
 *          - Noise: Gaussian noise injection for realistic measurements
 *          - Wind Effects: Dynamic pressure corrections for airspeed/wind
 *          
 *          Integration with SITL:
 *          - Retrieves simulated altitude from SITL::SIM physics model
 *          - Supports multiple barometer instances for redundancy testing
 *          - Updates at configurable rates matching real sensor behavior
 *          - Provides health monitoring and failure injection capabilities
 * 
 * @note This backend is only available when AP_SIM_BARO_ENABLED is defined,
 *       typically in SITL and HIL builds. It should never be used in production
 *       flight controller firmware as it requires simulation infrastructure.
 * 
 * @warning The simulated sensor includes intentional imperfections (noise, drift)
 *          to ensure algorithms are robust to real-world sensor characteristics.
 */
class AP_Baro_SITL : public AP_Baro_Backend {
public:
    /**
     * @brief Construct a new SITL barometer backend instance
     * 
     * @details Initializes the simulated barometer, registers with the SITL
     *          simulation framework, and sets up the measurement delay buffer.
     *          Retrieves a pointer to the global SITL simulation object for
     *          accessing simulated vehicle state (altitude, temperature, etc.).
     * 
     * @param[in] baro Reference to the AP_Baro frontend manager
     * 
     * @note Constructor registers this instance with the barometer frontend
     *       and initializes delay buffer to zero
     */
    AP_Baro_SITL(AP_Baro &baro);

    /**
     * @brief Update simulated barometer measurements
     * 
     * @details Called periodically by the scheduler to generate new simulated
     *          pressure and temperature readings. Retrieves current altitude from
     *          SITL physics simulation, applies standard atmosphere model, adds
     *          configured noise and drift, processes through delay buffer, and
     *          publishes to frontend.
     *          
     *          Update sequence:
     *          1. Read simulated altitude from SITL
     *          2. Calculate pressure using standard atmosphere model
     *          3. Calculate temperature with altitude lapse rate
     *          4. Apply board temperature effects
     *          5. Add wind pressure corrections
     *          6. Inject drift and noise per SITL configuration
     *          7. Store in delay buffer
     *          8. Retrieve delayed sample and publish to frontend
     * 
     * @note Called at main loop rate (typically 50-400 Hz depending on vehicle)
     * @note Overrides AP_Baro_Backend::update()
     */
    void update() override;

    /**
     * @brief Apply simulated board temperature effects to sensor readings
     * 
     * @details Simulates the temperature-dependent errors present in real barometric
     *          sensors. Physical barometers experience pressure reading drift due to
     *          changes in the sensor's own temperature (board heating from electronics,
     *          environmental temperature changes). This function applies a correction
     *          that mimics this real-world effect for realistic testing.
     *          
     *          The adjustment models the temperature coefficient of the sensor, where
     *          pressure readings are affected by the difference between current board
     *          temperature and calibration temperature.
     * 
     * @param[in,out] p Pressure reading in Pascals (Pa) - modified in place
     * @param[in,out] T Temperature reading in degrees Celsius - modified in place
     * 
     * @note Static method - can be called without instance for use by other SITL components
     * @note Correction magnitude is configurable via SITL parameters
     */
    static void temperature_adjustment(float &p, float &T);

    /**
     * @brief Calculate dynamic pressure correction due to wind and airspeed
     * 
     * @details Computes the dynamic pressure component caused by vehicle airspeed
     *          and wind velocity. In real flight, air flowing past the static port
     *          creates dynamic pressure errors (static source error). This simulation
     *          models that effect using: ΔP = 0.5 * ρ * v²
     *          where ρ is air density and v is airspeed.
     *          
     *          This correction is critical for accurate altitude hold and vertical
     *          velocity estimation in windy conditions or during aggressive maneuvers.
     * 
     * @param[in] instance Barometer instance number (for multi-baro configurations)
     * 
     * @return Dynamic pressure correction in Pascals (Pa)
     *         Positive values indicate pressure increase (airspeed effect)
     *         Zero if vehicle has no airspeed or wind disabled in simulation
     * 
     * @note Static method - can be called without instance for use by other SITL components
     * @note Correction is based on true airspeed from SITL physics model
     */
    static float wind_pressure_correction(uint8_t instance);

protected:
    /**
     * @brief Update the health status flag for this barometer instance
     * 
     * @details Updates the sensor health flag in the frontend based on simulated
     *          sensor status. Health can be affected by SITL configuration to
     *          simulate sensor failures, initialization delays, or intermittent faults.
     * 
     * @param[in] instance Barometer instance number
     * 
     * @note Overrides AP_Baro_Backend::update_healthy_flag()
     */
    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = healthy(instance); };

private:
    /// Barometer instance number registered with frontend
    uint8_t _instance;
    
    /// Pointer to SITL simulation object providing physics state (altitude, temperature, wind)
    SITL::SIM *_sitl;

    /**
     * @struct readings_baro
     * @brief Single barometer reading in the delay buffer
     * 
     * @details Stores a timestamped pressure reading for simulating sensor latency.
     *          Real barometric sensors have measurement delays due to sampling time,
     *          filtering, and communication lag. This buffer allows configurable delay
     *          to match real sensor characteristics.
     */
    struct readings_baro {
        uint32_t time;  ///< Timestamp in milliseconds when reading was generated
        float data;     ///< Pressure reading in Pascals (Pa)
    };
    
    /// Current write position in circular delay buffer
    uint8_t _store_index;
    
    /// Timestamp of last reading stored in buffer (milliseconds)
    uint32_t _last_store_time;
    
    /// Maximum number of readings in delay buffer (supports up to 1 second delay at 50Hz)
    static const uint8_t _buffer_length = 50;
    
    /// Circular buffer storing timestamped pressure readings for delay simulation
    VectorN<readings_baro, _buffer_length> _buffer;

    /**
     * @brief Check if barometer instance is healthy and usable for flight
     * 
     * @details Evaluates simulated sensor health based on SITL configuration.
     *          Can simulate various failure modes: initialization delays, intermittent
     *          faults, complete failures, or invalid data conditions. Used for testing
     *          failsafe logic and sensor redundancy algorithms.
     * 
     * @param[in] instance Barometer instance number
     * 
     * @return true if sensor is providing valid data and is usable for flight
     * @return false if sensor is failed, initializing, or producing invalid data
     * 
     * @note Health status can be controlled via SITL parameters for failure injection testing
     */
    bool healthy(uint8_t instance);
    
    /**
     * @brief Internal timer callback to generate new sensor readings
     * 
     * @details Called periodically to synthesize new pressure and temperature measurements.
     *          Retrieves altitude from SITL physics, applies atmospheric model, adds noise
     *          and drift, and stores in delay buffer. Separated from update() to allow
     *          for asynchronous sensor timing if needed.
     * 
     * @note Typically called at sensor's native sample rate (e.g., 50-100 Hz for real baros)
     */
    void _timer();
    
    /// Flag indicating new sample is available for publishing
    bool _has_sample;
    
    /// Timestamp of last sample generation in milliseconds
    uint32_t _last_sample_time;
    
    /// Most recently generated temperature reading in degrees Celsius (before delay)
    float _recent_temp;
    
    /// Most recently generated pressure reading in Pascals (before delay)
    float _recent_press;
    
    /// Previous altitude reading in meters (used for drift rate calculation)
    float _last_altitude;

    /// Time accumulator for drift integration in milliseconds
    /// Allows for integration of drift over time to simulate long-term sensor bias
    uint32_t last_drift_delta_t_ms;
    
    /// Accumulated altitude drift in meters
    /// Integrated over time based on SITL drift rate parameter (meters/second)
    float total_alt_drift;
};
#endif  // AP_SIM_BARO_ENABLED
