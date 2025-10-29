/**
 * @file AP_RangeFinder_SITL.h
 * @brief SITL (Software-In-The-Loop) simulation backend for rangefinder sensors
 * 
 * @details This file implements a simulated rangefinder backend for ArduPilot's
 *          Software-In-The-Loop (SITL) testing environment. It provides realistic
 *          distance measurements by interfacing with SITL's physics simulation,
 *          enabling comprehensive testing of rangefinder-dependent features
 *          (terrain following, precision landing, obstacle avoidance) without
 *          requiring physical hardware.
 * 
 *          The SITL rangefinder backend simulates distance measurements using
 *          ray tracing techniques against the simulated environment, accounting
 *          for sensor orientation, vehicle attitude, and terrain geometry.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_SIM_ENABLED

#include "AP_RangeFinder_Backend.h"

/**
 * @class AP_RangeFinder_SITL
 * @brief Software-In-The-Loop simulation backend for rangefinder distance sensors
 * 
 * @details This class provides a simulated rangefinder implementation for SITL
 *          (Software-In-The-Loop) testing. It generates realistic distance
 *          measurements by performing ray tracing against the simulated environment,
 *          allowing comprehensive testing of rangefinder-dependent features without
 *          physical hardware.
 * 
 *          Key capabilities:
 *          - Simulates distance measurements using physics-based ray tracing
 *          - Accounts for sensor orientation and vehicle attitude
 *          - Supports terrain interaction and obstacle detection
 *          - Provides realistic sensor characteristics (noise, latency, limits)
 *          - Enables testing of terrain following, precision landing, and avoidance
 * 
 *          The SITL rangefinder interfaces with the SITL physics simulation to
 *          determine ground distance or obstacle range based on the sensor's
 *          configured orientation (downward-facing, forward-facing, etc.).
 * 
 *          Typical workflow:
 *          1. Constructor initializes sensor with instance-specific parameters
 *          2. update() is called periodically by the rangefinder driver
 *          3. Ray tracing is performed against simulated environment
 *          4. Distance measurement is stored in state structure
 *          5. MAVLink telemetry reports simulated distance to ground station
 * 
 * @note This backend is only available when AP_RANGEFINDER_SIM_ENABLED is defined
 * @note Used exclusively for simulation testing, not compiled for flight hardware
 * @note Supports multiple concurrent rangefinder instances (e.g., down + forward)
 * 
 * @see AP_RangeFinder_Backend
 * @see SITL::Aircraft for physics simulation interface
 */
class AP_RangeFinder_SITL : public AP_RangeFinder_Backend {
public:
    /**
     * @brief Constructor for SITL rangefinder backend
     * 
     * @details Initializes a simulated rangefinder sensor instance for Software-In-The-Loop
     *          testing. This constructor sets up the sensor state and parameters, and stores
     *          the instance number for identifying which simulated sensor to query from the
     *          SITL physics simulation.
     * 
     *          The constructor performs immediate initialization, preparing the sensor for
     *          distance measurement queries during subsequent update() calls. Multiple
     *          instances can be created to simulate multi-sensor configurations (e.g.,
     *          downward-facing for terrain following + forward-facing for obstacle avoidance).
     * 
     * @param[in,out] _state    Reference to rangefinder state structure for storing measurements.
     *                          Updated by update() with simulated distance, status, and timestamp.
     * @param[in]     _params   Reference to rangefinder parameter structure containing sensor
     *                          configuration (orientation, max range, offset, etc.)
     * @param[in]     instance  Sensor instance number (0-based). Used to identify which simulated
     *                          rangefinder to query from SITL physics simulation when multiple
     *                          sensors are configured. Typical values: 0=primary downward,
     *                          1=forward-facing, 2-9=additional sensors.
     * 
     * @note This constructor incorporates complete initialization - no separate init() required
     * @note Instance number must match SITL simulation sensor configuration
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp (implementation)
     */
    AP_RangeFinder_SITL(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, uint8_t instance);

    /**
     * @brief Update simulated rangefinder measurements from SITL physics
     * 
     * @details Retrieves the latest simulated distance measurement from the SITL
     *          physics simulation and updates the sensor state structure. This method
     *          is called periodically by the rangefinder driver (typically at 10-20 Hz)
     *          to provide fresh distance data to navigation and control systems.
     * 
     *          Update process:
     *          1. Query SITL physics simulation for current distance along sensor ray
     *          2. Apply sensor-specific characteristics (noise, latency, quantization)
     *          3. Check measurement validity against configured min/max range limits
     *          4. Update state structure with distance, status, and timestamp
     *          5. Handle out-of-range and no-target conditions appropriately
     * 
     *          The SITL backend performs ray tracing from the vehicle's current position
     *          in the direction defined by the sensor's configured orientation, accounting
     *          for vehicle attitude. Ray tracing checks intersection with terrain geometry
     *          and simulated obstacles to determine the measured distance.
     * 
     * @note Called at 10-20 Hz by rangefinder driver scheduler task
     * @note Distance measurements include simulated sensor noise and latency
     * @note Out-of-range conditions set appropriate status flags (NoData or OutOfRangeLow/High)
     * @note Ray tracing accuracy depends on SITL terrain resolution and physics fidelity
     * 
     * @warning SITL-only implementation - not available on flight hardware
     * 
     * @see AP_RangeFinder_Backend::update_status() for state structure updates
     * @see SITL::Aircraft::get_rangefinder() for physics simulation interface
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp (implementation details)
     */
    void update() override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier for telemetry
     * 
     * @details Returns the MAVLink DISTANCE_SENSOR message type identifier that
     *          describes this sensor technology. For SITL simulation sensors, this
     *          returns MAV_DISTANCE_SENSOR_UNKNOWN since the simulated sensor doesn't
     *          represent a specific real-world sensor technology (laser, sonar, etc.).
     * 
     *          This identifier is included in MAVLink DISTANCE_SENSOR messages sent to
     *          ground control stations to indicate the measurement technology being used.
     * 
     * @return MAV_DISTANCE_SENSOR_UNKNOWN indicating simulated/unspecified sensor type
     * 
     * @note Override from AP_RangeFinder_Backend base class
     * @note Real hardware backends return specific types (LASER, ULTRASOUND, RADAR, etc.)
     * @note Ground stations may display simulated sensors generically due to UNKNOWN type
     * 
     * @see MAV_DISTANCE_SENSOR enum in MAVLink common.xml message definitions
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

    /**
     * @brief Sensor instance number for SITL multi-sensor identification
     * 
     * @details Stores the rangefinder instance number (0-based) used to identify which
     *          simulated sensor to query from the SITL physics simulation. When multiple
     *          rangefinders are configured (e.g., downward + forward + backward), each
     *          instance corresponds to a specific simulated sensor with its own orientation
     *          and ray tracing configuration.
     * 
     *          Instance mapping (typical configuration):
     *          - 0: Primary downward-facing rangefinder (terrain following, landing)
     *          - 1: Forward-facing rangefinder (obstacle avoidance)
     *          - 2: Backward-facing rangefinder (proximity sensing)
     *          - 3-9: Additional sensors for 360° coverage or specialized applications
     * 
     *          The instance number is passed to SITL physics queries to retrieve the
     *          correct simulated distance measurement for this specific sensor.
     * 
     * @note Set during construction from instance parameter
     * @note Must match SITL parameter configuration (SIM_RNGFNDx_* parameters)
     * @note Maximum instances limited by AP_RANGEFINDER_MAX_INSTANCES (typically 10)
     */
    uint8_t _instance;

};

#endif  // AP_RANGEFINDER_SIM_ENABLED
