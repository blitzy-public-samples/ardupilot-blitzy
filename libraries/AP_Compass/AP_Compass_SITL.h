/**
 * @file AP_Compass_SITL.h
 * @brief Software-In-The-Loop (SITL) compass simulation driver
 * 
 * @details This file implements a simulated magnetometer for testing ArduPilot
 *          in the SITL environment without requiring physical hardware.
 *          The simulation provides realistic magnetic field data including
 *          earth's magnetic field characteristics, sensor noise, biases,
 *          and scale factors for comprehensive testing of navigation and
 *          calibration algorithms.
 * 
 * Source: libraries/AP_Compass/AP_Compass_SITL.h
 */

#pragma once

#include "AP_Compass.h"

#if AP_COMPASS_SITL_ENABLED

#include "AP_Compass_Backend.h"

#include <AP_Math/vectorN.h>
#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>
#include <SITL/SITL.h>

#define MAX_SITL_COMPASSES 3

/**
 * @class AP_Compass_SITL
 * @brief SITL magnetometer simulation backend for testing compass functionality
 * 
 * @details This class provides a complete simulation of magnetometer sensors
 *          for Software-In-The-Loop testing. It generates realistic magnetic
 *          field measurements based on vehicle position, orientation, and
 *          configured sensor characteristics.
 * 
 *          Key simulation features:
 *          - Earth's magnetic field modeling with declination and inclination
 *            based on geographic location using the World Magnetic Model (WMM)
 *          - Magnetic field intensity variation with location
 *          - Simulated sensor noise for realistic signal characteristics
 *          - Configurable bias and scale factor errors for calibration testing
 *          - Elliptical correction simulation for hard and soft iron effects
 *          - Support for multiple compass instances (up to MAX_SITL_COMPASSES)
 *          - Sensor data buffering with configurable delays for timing simulation
 * 
 *          The simulated magnetic field is computed in the NED (North-East-Down)
 *          reference frame and transformed to the sensor body frame based on
 *          vehicle attitude. Field values are provided in milligauss units,
 *          matching the output of real magnetometer hardware.
 * 
 *          This backend enables comprehensive testing of:
 *          - Compass calibration algorithms (including motor compensation)
 *          - Multi-compass selection and failover logic
 *          - Navigation algorithms dependent on heading information
 *          - Compass health monitoring and consistency checking
 * 
 *          Integration: This class interfaces with the SITL physics simulation
 *          (AP_HAL_SITL) to obtain vehicle state and configured sensor parameters.
 * 
 * @note Magnetic field units: milligauss (mGauss)
 * @note Coordinate frame: Sensor measurements in body frame, converted from NED
 * @note Update rate: Matches configured compass sample rate in SITL parameters
 * 
 * @warning This is a simulation-only driver. Do not use in flight hardware builds.
 * 
 * @see AP_Compass_Backend Base class for compass drivers
 * @see SITL::SIM SITL physics simulation parameters
 * @see AP_Declination World Magnetic Model for field computation
 */
class AP_Compass_SITL : public AP_Compass_Backend {
public:
    /**
     * @brief Construct SITL compass simulation backend
     * 
     * @details Initializes the simulated magnetometer backend for SITL testing.
     *          Creates and registers compass instances (up to MAX_SITL_COMPASSES)
     *          based on SITL configuration. Retrieves SITL simulation parameters
     *          including sensor noise characteristics, bias values, and scale
     *          factors. Initializes the delay buffer for realistic sensor timing
     *          simulation and sets up elliptical correction matrices for hard
     *          and soft iron effect modeling.
     * 
     * @note The number of simulated compass instances is determined by SITL
     *       configuration parameters at initialization time
     * @note Each compass instance is registered with the AP_Compass frontend
     *       with appropriate device IDs for multi-compass support
     * 
     * @see AP_Compass_Backend::register_compass() Instance registration
     * @see SITL::SIM Simulation parameter source
     */
    AP_Compass_SITL();

    /**
     * @brief Read simulated magnetic field data and update compass state
     * 
     * @details Fetches simulated magnetic field measurements from the SITL
     *          physics simulation and updates all registered compass instances.
     *          This method is called periodically by the compass frontend at
     *          the configured sample rate.
     * 
     *          Simulation process:
     *          1. Retrieves current vehicle position (lat/lon/alt) from SITL
     *          2. Computes earth's magnetic field vector for current location
     *             using World Magnetic Model (declination, inclination, intensity)
     *          3. Transforms magnetic field from NED frame to body frame using
     *             current vehicle attitude (roll, pitch, yaw)
     *          4. Applies simulated sensor characteristics:
     *             - Adds configured bias offsets for each axis
     *             - Applies scale factor errors and cross-coupling
     *             - Applies elliptical correction (hard/soft iron effects)
     *             - Adds Gaussian noise for realistic signal variation
     *          5. Buffers readings with configurable delay for timing simulation
     *          6. Updates compass frontend with processed measurements
     * 
     *          The magnetic field vector components are:
     *          - X: Forward in body frame (milligauss)
     *          - Y: Right in body frame (milligauss)
     *          - Z: Down in body frame (milligauss)
     * 
     * @note Called at main sensor update rate (typically 50-100 Hz)
     * @note Field values in milligauss to match real magnetometer output
     * @note Supports testing of calibration algorithms through configurable
     *       bias and scale factor errors
     * @note Multi-compass configurations can have different error characteristics
     *       per instance for testing compass selection and consistency checking
     * 
     * @warning Simulated data only - does not access physical hardware
     * 
     * @see _timer() Internal timer callback for simulation update
     * @see SITL::SIM::mag_bf Simulated magnetic field in body frame
     * @see AP_Declination::get_earth_field_ga() WMM field computation
     */
    void read(void) override;

private:
    /// Instance IDs for each simulated compass registered with AP_Compass frontend
    uint8_t _compass_instance[MAX_SITL_COMPASSES];
    
    /// Number of compass instances currently simulated
    uint8_t _num_compass;
    
    /// Pointer to SITL simulation object for physics and parameter access
    SITL::SIM *_sitl;

    /**
     * @brief Delay buffer for simulating sensor timing characteristics
     * 
     * @details Stores timestamped magnetic field readings to simulate realistic
     *          sensor delays and buffering behavior. Allows testing of timing-
     *          sensitive navigation algorithms.
     */
    struct readings_compass {
        uint32_t time;      ///< Timestamp in microseconds
        Vector3f data;      ///< Magnetic field vector in milligauss (body frame)
    };
    
    uint8_t store_index;                                    ///< Current buffer write position
    uint32_t last_store_time;                               ///< Timestamp of last buffer store (μs)
    static const uint8_t buffer_length = 50;                ///< Buffer size for delay simulation
    VectorN<readings_compass,buffer_length> buffer;         ///< Circular buffer for readings

    /**
     * @brief Internal timer callback for periodic simulation updates
     * 
     * @details Generates new magnetic field samples based on current vehicle
     *          state and stores them in the delay buffer for later retrieval.
     */
    void _timer();
    
    uint32_t _last_sample_time;     ///< Timestamp of last sample generation (μs)

    /**
     * @brief Setup elliptical correction matrix for hard/soft iron simulation
     * 
     * @details Configures the elliptical correction matrix that simulates
     *          hard iron (offset) and soft iron (scale/cross-coupling) effects
     *          for the specified compass instance. Used to test calibration
     *          algorithms with realistic magnetic interference patterns.
     * 
     * @param[in] i Compass instance index (0 to MAX_SITL_COMPASSES-1)
     */
    void _setup_eliptical_correcion(uint8_t i);
    
    Matrix3f _eliptical_corr;                       ///< Current elliptical correction matrix (soft iron)
    Vector3f _last_dia;                             ///< Last diagonal correction values
    Vector3f _last_odi;                             ///< Last off-diagonal correction values
    Vector3f _last_data[MAX_SITL_COMPASSES];        ///< Previous reading for each compass instance (mGauss)
};
#endif // AP_COMPASS_SITL_ENABLED
