/**
 * @file AP_OpticalFlow_SITL.h
 * @brief SITL (Software In The Loop) optical flow sensor simulation backend
 * 
 * @details This file implements a simulated optical flow sensor for use in SITL
 *          (Software In The Loop) simulation environments. It provides realistic
 *          optical flow measurements synthesized from simulated vehicle state,
 *          enabling indoor navigation testing and algorithm development without
 *          physical hardware.
 * 
 *          Features:
 *          - Synthesizes realistic flow from simulated vehicle state (position,
 *            velocity, attitude)
 *          - Configurable noise, scale factors, and sensor delay
 *          - Quality degradation with altitude and surface characteristics
 *          - Ring buffer implementation for realistic sensor latency
 * 
 *          Use Cases:
 *          - SITL simulation for GPS-denied navigation testing
 *          - Indoor flight mode development (position hold, loiter)
 *          - Optical flow EKF fusion algorithm validation
 *          - Sensor parameter tuning without hardware risk
 * 
 * @note Requires AP_SIM_ENABLED compile flag (automatically set in SITL builds)
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_SITL_ENABLED

#include "AP_OpticalFlow.h"

/**
 * @class AP_OpticalFlow_SITL
 * @brief SITL optical flow sensor simulation backend
 * 
 * @details This class implements a simulated optical flow sensor that generates
 *          synthetic flow measurements from SITL vehicle state. It provides a
 *          realistic sensor model for testing GPS-denied navigation algorithms.
 * 
 *          Simulation Algorithm:
 *          1. Read simulated vehicle velocity (NED frame) and altitude from SITL
 *          2. Transform velocity to sensor body frame using simulated attitude
 *          3. Compute flow rates: flowRate = velocity / altitude (rad/s)
 *          4. Add configurable noise (Gaussian or uniform distribution)
 *          5. Apply scale factor to simulate sensor calibration error
 *          6. Compute quality metric based on altitude (degrades above optimal range)
 *          7. Store in ring buffer with configurable delay (optflow_delay)
 *          8. Retrieve delayed measurement and forward to frontend
 * 
 *          The sensor model includes:
 *          - Realistic sensor delay via ring buffer (up to 2 seconds at 10Hz)
 *          - Configurable noise characteristics via SITL parameters
 *          - Quality degradation with altitude and motion
 *          - Scale factor errors for calibration testing
 * 
 *          Testing Capabilities:
 *          - Indoor flight testing in SITL without GPS
 *          - Position hold and altitude hold with optical flow
 *          - Optical flow EKF fusion validation
 *          - Surface texture and lighting sensitivity simulation
 * 
 * @note Ring buffer size of 20 allows up to 2 seconds delay at 10Hz update rate
 * @warning Simulation fidelity depends on SITL vehicle model accuracy
 * @warning Does not simulate all real-world sensor limitations (e.g., surface
 *          texture requirements, lighting conditions, maximum velocity limits)
 * 
 * @see AP_OpticalFlow for the main optical flow library
 * @see OpticalFlow_backend for the base backend interface
 */
class AP_OpticalFlow_SITL : public OpticalFlow_backend
{
public:
    /**
     * @brief Constructor - inherits from OpticalFlow_backend
     * 
     * @details Uses the base class constructor from OpticalFlow_backend.
     *          Initialization of simulation state (ring buffer, timing) is
     *          performed during the first update() call.
     * 
     * @see OpticalFlow_backend::OpticalFlow_backend
     */
    using OpticalFlow_backend::OpticalFlow_backend;

    /**
     * @brief Generate synthetic optical flow measurement from simulated vehicle state
     * 
     * @details This method is called periodically (typically at 10-20Hz) to generate
     *          optical flow measurements for SITL simulation. It performs the following:
     * 
     *          1. Retrieves simulated vehicle state from SITL::sitl:
     *             - Ground velocity (NED frame, m/s)
     *             - Altitude above ground (m)
     *             - Vehicle attitude (quaternion)
     *             - Gyroscope rates (rad/s)
     * 
     *          2. Computes expected optical flow:
     *             - Transforms velocity to sensor body frame
     *             - Calculates flow rates: flow = velocity / altitude (rad/s)
     *             - Applies small angle approximation for sensor model
     * 
     *          3. Applies sensor characteristics:
     *             - Adds configurable Gaussian or uniform noise
     *             - Applies scale factor from SITL parameters
     *             - Computes quality metric (0-255) based on:
     *               * Altitude (optimal range ~1-3m, degrades beyond)
     *               * Surface contrast (from SITL environment)
     *               * Excessive motion (quality drops with high rates)
     * 
     *          4. Simulates sensor delay:
     *             - Stores measurement in ring buffer (optflow_data[])
     *             - Applies delay specified by optflow_delay parameter
     *             - Retrieves delayed measurement for output
     * 
     *          5. Updates frontend via _update_frontend():
     *             - Forwards delayed flow measurement
     *             - Provides timestamp and quality information
     * 
     * @return void - Flow data returned via _update_frontend() call
     * 
     * @note Called at main scheduler rate, typically 10-20Hz for optical flow
     * @note First call initializes ring buffer and timing state
     * @note Quality value ranges from 0 (invalid) to 255 (perfect)
     * @note Flow rates are in rad/s, following OpticalFlow_state convention
     * 
     * @warning Simulation assumes flat horizontal surface; does not model terrain
     * @warning Does not simulate occlusion, shadows, or specular reflections
     * @warning Maximum measurable velocity limited by altitude (flow = vel/alt)
     * 
     * @see AP_OpticalFlow::OpticalFlow_state for measurement data structure
     * @see SITL::SIM for simulated vehicle state access
     */
    void update(void) override;

private:
    /**
     * @brief Last update timestamp in milliseconds
     * 
     * Stores the timestamp of the last optical flow measurement update.
     * Used for timing calculations and ensuring proper update intervals.
     * 
     * Units: milliseconds (AP_HAL::millis() timestamp)
     */
    uint32_t last_flow_ms;

    /**
     * @brief Ring buffer write index for delayed measurements
     * 
     * Current write position in the optflow_data ring buffer (0-19).
     * Increments with each update() call and wraps at buffer size.
     * Used with optflow_delay to implement realistic sensor latency.
     * 
     * Units: Array index (0-19)
     */
    uint8_t next_optflow_index;

    /**
     * @brief Configurable sensor delay in update cycles
     * 
     * Number of update cycles to delay optical flow measurements, simulating
     * real sensor processing latency. Configured via SITL parameters
     * (typically FLOW_DELAY). Delay is implemented using the ring buffer:
     * read_index = (next_optflow_index - optflow_delay) % buffer_size
     * 
     * Typical values:
     * - 0: No delay (instantaneous, unrealistic)
     * - 1-5: Typical sensor delays (10-50ms at 10Hz update rate)
     * - 10+: High latency sensors or processing delays
     * 
     * Units: Update cycles (1 cycle = 1/update_rate seconds, typically 100ms at 10Hz)
     * 
     * @note Maximum delay limited by ring buffer size (20 cycles = 2 seconds at 10Hz)
     */
    uint8_t optflow_delay;

    /**
     * @brief Ring buffer for delayed optical flow measurements
     * 
     * Stores recent optical flow measurements to simulate realistic sensor delay.
     * Buffer size of 20 allows up to 2 seconds delay at typical 10Hz update rate.
     * Each entry contains a complete OpticalFlow_state with:
     * - flowRate: Flow measurement in rad/s (X and Y axes)
     * - bodyRate: Gyroscope rates at measurement time (rad/s)
     * - surface_quality: Quality metric (0-255)
     * 
     * Buffer management:
     * - Write index: next_optflow_index (wraps at 20)
     * - Read index: (next_optflow_index - optflow_delay) % 20
     * - Circular buffer with automatic wrap-around
     * 
     * Units:
     * - flowRate: rad/s (angular rate of image motion)
     * - bodyRate: rad/s (vehicle rotation rates)
     * - surface_quality: 0-255 (0=invalid, 255=perfect)
     * 
     * @note Ring buffer initialized on first update() call
     * @note Buffer size chosen to support realistic delays (0-2 seconds)
     */
    AP_OpticalFlow::OpticalFlow_state optflow_data[20];
};

#endif  // AP_OPTICALFLOW_SITL_ENABLED
