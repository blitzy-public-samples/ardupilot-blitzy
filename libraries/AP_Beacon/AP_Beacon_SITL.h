/**
 * @file AP_Beacon_SITL.h
 * @brief SITL (Software In The Loop) beacon simulation backend for testing
 * 
 * Provides simulated beacon positioning system for SITL simulation environment.
 * This backend generates synthetic beacon data for testing beacon-based positioning
 * algorithms without requiring physical hardware.
 * 
 * @note This backend is only compiled when AP_BEACON_SITL_ENABLED is defined,
 *       typically in SITL simulation builds.
 */

#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_SITL_ENABLED

#include <SITL/SITL.h>

/**
 * @class AP_Beacon_SITL
 * @brief Simulated beacon positioning system backend for SITL testing
 * 
 * @details This backend simulates a beacon positioning system with 4 beacons
 *          arranged in a rectangular pattern for testing purposes. The simulated
 *          beacons provide range measurements to the vehicle based on the SITL
 *          vehicle state, enabling testing of beacon-based positioning algorithms
 *          without physical hardware.
 * 
 *          Beacon Configuration:
 *          - 4 beacons arranged in rectangular pattern
 *          - Beacon 0: Position (0, 0) - Southwest corner
 *          - Beacon 1: Position (100, 0) - Southeast corner (100m east)
 *          - Beacon 2: Position (0, 100) - Northwest corner (100m north)
 *          - Beacon 3: Position (100, 100) - Northeast corner
 *          - All beacons at ground level (altitude = 0)
 * 
 *          Simulation Behavior:
 *          - Data sourced from AP::sitl() state for vehicle position
 *          - Performs geodetic to local NE (North-East) coordinate conversion
 *          - Calculates true ranges from vehicle to each beacon
 *          - Updates throttled to 10ms minimum interval
 *          - Health status based on 200ms timeout since last update
 * 
 *          Units:
 *          - Beacon positions: meters in local NE frame
 *          - Ranges: meters
 *          - Update intervals: milliseconds
 * 
 * @note This simulation provides ideal range measurements without noise or errors.
 *       Real-world beacon systems will have measurement noise, multipath effects,
 *       and other error sources not modeled in this simple simulation.
 * 
 * @warning SITL-only backend - not available in production firmware builds
 */
class AP_Beacon_SITL : public AP_Beacon_Backend
{

public:
    /**
     * @brief Constructor for SITL beacon simulation backend
     * 
     * @details Initializes the SITL beacon backend and obtains reference to the
     *          SITL simulation state. Sets up initial state for beacon simulation
     *          including next beacon index and last update timestamp.
     * 
     * @param[in] frontend Reference to the AP_Beacon frontend object
     */
    AP_Beacon_SITL(AP_Beacon &frontend);

    /**
     * @brief Check if beacon system is healthy and providing data
     * 
     * @details Returns true if the beacon system has received updates within
     *          the timeout period (200ms). This indicates the simulation is
     *          running and providing beacon range data.
     * 
     * @return true if beacon data is current and healthy
     * @return false if no updates received within timeout period
     * 
     * @note Health is based solely on data timeout, not measurement quality
     */
    bool healthy() override;

    /**
     * @brief Update simulated beacon measurements
     * 
     * @details Generates simulated beacon range measurements by:
     *          1. Checking update throttling (10ms minimum interval)
     *          2. Reading vehicle position from AP::sitl() state
     *          3. Converting geodetic position to local NE coordinates
     *          4. Calculating range to next beacon in sequence
     *          5. Reporting range measurement to frontend
     *          6. Advancing to next beacon (round-robin through 4 beacons)
     * 
     *          Beacons are updated sequentially one per call, cycling through
     *          all 4 beacons to simulate individual beacon measurements arriving
     *          at different times.
     * 
     * @note Called from main scheduler loop, typically at 10-50Hz
     * @note Update rate throttled internally to 10ms (100Hz max)
     */
    void update() override;

private:
    SITL::SIM *sitl;              ///< Pointer to SITL simulation state for vehicle position data
    uint8_t next_beacon;          ///< Index (0-3) of next beacon to update in round-robin sequence
    uint32_t last_update_ms;      ///< Timestamp of last update in milliseconds (for throttling and health monitoring)
};

#endif // AP_BEACON_SITL_ENABLED
