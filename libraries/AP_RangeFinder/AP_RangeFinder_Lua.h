/**
 * @file AP_RangeFinder_Lua.h
 * @brief Lua scripting backend for custom rangefinder implementations
 * 
 * @details This file implements a rangefinder backend that allows users to create
 *          custom rangefinder drivers using Lua scripting. This enables integration
 *          of rangefinders that are not natively supported by ArduPilot without
 *          requiring C++ code modifications.
 *          
 *          The Lua rangefinder backend provides a scripting interface where user
 *          scripts can feed distance measurements and full sensor state to the
 *          rangefinder library. This is particularly useful for:
 *          - Prototyping new rangefinder integrations
 *          - Supporting custom or proprietary rangefinders
 *          - Processing rangefinder data through custom algorithms
 *          - Integrating rangefinders via non-standard protocols
 *          
 *          Communication Flow:
 *          1. Lua script reads sensor data (via serial, I2C, or other means)
 *          2. Script calls handle_script_msg() with distance or full state
 *          3. Backend validates and stores the measurement
 *          4. update() method checks for timeout and publishes valid data
 * 
 * @note This backend requires AP_SCRIPTING to be enabled
 * @warning Lua scripts run at lower priority than flight-critical code; ensure
 *          script execution time is minimal to avoid impacting vehicle performance
 * 
 * @see libraries/AP_Scripting/ for Lua scripting documentation
 * @see AP_RangeFinder_Backend for base class interface
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LUA_ENABLED

#include "AP_RangeFinder_Backend.h"

/**
 * @brief Timeout for Lua rangefinder data in milliseconds
 * 
 * @details If no new measurement is received from the Lua script within this
 *          timeout period, the rangefinder will be marked as unhealthy and
 *          distance readings will be invalid. This prevents stale data from
 *          being used for navigation or collision avoidance.
 *          
 *          The 500ms timeout allows for update rates up to 2Hz, though higher
 *          rates (10-20Hz) are recommended for optimal performance.
 */
#define AP_RANGEFINDER_LUA_TIMEOUT_MS 500

/**
 * @class AP_RangeFinder_Lua
 * @brief Rangefinder backend for user-defined Lua script implementations
 * 
 * @details This backend enables custom rangefinder integration through Lua scripting,
 *          allowing users to implement rangefinder drivers without modifying C++ code.
 *          The Lua script is responsible for reading sensor data and calling the
 *          appropriate handle_script_msg() method to update the rangefinder state.
 *          
 *          Typical Usage Pattern in Lua:
 *          @code
 *          -- Get rangefinder instance
 *          local rngfnd = RangeFinder()
 *          
 *          -- In periodic callback (10-20Hz recommended):
 *          function update()
 *              local distance_m = read_custom_sensor()  -- User implementation
 *              if distance_m then
 *                  rngfnd:handle_script_msg(distance_m)
 *              end
 *          end
 *          @endcode
 *          
 *          The backend handles:
 *          - Data timeout detection (500ms)
 *          - State management and validation
 *          - Integration with ArduPilot's rangefinder library
 *          - MAVLink distance sensor reporting
 *          
 *          Thread Safety: Lua scripts execute in the scripting thread context,
 *          while update() is called from the sensor thread. The implementation
 *          uses atomic state updates to ensure thread-safe operation.
 * 
 * @note Lua rangefinder configured via RNGFNDx_TYPE = 36 (Scripting)
 * @warning Script must call handle_script_msg() regularly (>2Hz) or sensor
 *          will timeout and be marked unhealthy
 * 
 * @see AP_RangeFinder_Backend for inherited interface
 * @see AP_Scripting for Lua scripting engine
 */
class AP_RangeFinder_Lua : public AP_RangeFinder_Backend
{
public:

    /**
     * @brief Constructor for Lua rangefinder backend
     * 
     * @details Initializes a rangefinder backend that will receive measurements
     *          from Lua scripts. The backend is created when a rangefinder is
     *          configured with type SCRIPTING (36).
     * 
     * @param[in,out] _state  Rangefinder state structure for storing measurements
     * @param[in]     _params Rangefinder configuration parameters (min/max distance, etc.)
     * 
     * @note Constructor is typically called by AP_RangeFinder during initialization,
     *       not directly by user code
     */
    AP_RangeFinder_Lua(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Update rangefinder state and check for timeout
     * 
     * @details Called periodically by the rangefinder library to update sensor
     *          state. This method:
     *          1. Checks if new data from Lua script is available
     *          2. Validates data hasn't exceeded timeout threshold
     *          3. Updates the rangefinder state with pending measurements
     *          4. Marks sensor unhealthy if data is stale (>500ms old)
     *          
     *          The method transfers measurements from _state_pending (written
     *          by Lua script thread) to the active state structure in a
     *          thread-safe manner.
     * 
     * @note Called from sensor thread context at rangefinder update rate
     * @note If no script message received within AP_RANGEFINDER_LUA_TIMEOUT_MS,
     *       sensor will be marked unhealthy
     */
    void update(void) override;

    /**
     * @brief Receive distance measurement from Lua script (simple interface)
     * 
     * @details This is the simplified interface for Lua scripts to provide
     *          distance measurements. The script only needs to supply the
     *          distance in meters; all other state fields (timestamp, status)
     *          are automatically managed by the backend.
     *          
     *          Typical Lua usage:
     *          @code
     *          -- Simple distance update
     *          local distance_m = 5.2
     *          rangefinder:handle_script_msg(distance_m)
     *          @endcode
     *          
     *          This method is preferred for simple rangefinder implementations
     *          where only distance is measured. For sensors that provide
     *          additional information (signal quality, voltage, temperature),
     *          use the full state version of handle_script_msg().
     * 
     * @param[in] dist_m Distance measurement in meters
     * 
     * @return true if measurement was accepted and stored
     * @return false if measurement was rejected (e.g., out of configured range)
     * 
     * @note Called from Lua scripting thread context
     * @note Distance is validated against RNGFNDx_MIN_CM and RNGFNDx_MAX_CM parameters
     * @note Automatically sets status to RangeFinder::Status::Good if distance valid
     * 
     * @see handle_script_msg(const RangeFinder::RangeFinder_State &) for full state interface
     */
    bool handle_script_msg(float dist_m) override;

    /**
     * @brief Receive complete rangefinder state from Lua script (advanced interface)
     * 
     * @details This is the advanced interface for Lua scripts to provide complete
     *          rangefinder state including distance, status, signal quality, voltage,
     *          and other sensor-specific information. This allows full control over
     *          the rangefinder state from the Lua script.
     *          
     *          Typical Lua usage:
     *          @code
     *          -- Create state structure
     *          local state = {}
     *          state.distance_m = 5.2
     *          state.status = 0  -- Good status
     *          state.signal_quality_pct = 95
     *          state.voltage = 3.3
     *          rangefinder:handle_script_msg(state)
     *          @endcode
     *          
     *          This method is useful for:
     *          - Sensors that provide signal quality metrics
     *          - Custom error detection and status reporting
     *          - Sensors with multiple measurement modes
     *          - Advanced filtering or fusion in Lua scripts
     *          
     *          State Structure Fields:
     *          - distance_m: Distance measurement in meters (required)
     *          - status: RangeFinder::Status enum value (Good, NoData, OutOfRange, etc.)
     *          - signal_quality_pct: Signal quality 0-100% (optional)
     *          - voltage: Sensor supply voltage (optional)
     *          - last_reading_ms: Measurement timestamp (auto-set if not provided)
     * 
     * @param[in] state_arg Complete rangefinder state structure from Lua script
     * 
     * @return true if state was accepted and stored
     * @return false if state was rejected (e.g., invalid distance or status)
     * 
     * @note Called from Lua scripting thread context
     * @note Allows script to set custom status (NoData, OutOfRange, etc.)
     * @note Timestamp is automatically updated if not explicitly set in script
     * @note All fields except distance_m are optional
     * 
     * @warning Script must ensure distance_m is valid when status is set to Good
     * 
     * @see handle_script_msg(float) for simplified distance-only interface
     * @see RangeFinder::RangeFinder_State for complete state structure definition
     */
    bool handle_script_msg(const RangeFinder::RangeFinder_State &state_arg) override;

    /**
     * @brief Get MAVLink distance sensor type for this rangefinder
     * 
     * @details Returns the MAVLink distance sensor type identifier used in
     *          DISTANCE_SENSOR messages. For Lua-scripted rangefinders, the
     *          sensor type is unknown since the actual sensor hardware is
     *          abstracted by the Lua script.
     *          
     *          Ground control stations may use this type to display appropriate
     *          icons or apply sensor-specific processing.
     * 
     * @return MAV_DISTANCE_SENSOR_UNKNOWN indicating scripted/custom sensor
     * 
     * @note This is a MAVLink protocol requirement for distance sensor reporting
     * @note Override this in derived classes if specific sensor type is known
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

private:

    /**
     * @brief Pending rangefinder state from Lua script
     * 
     * @details Buffer to store rangefinder state received from Lua script via
     *          handle_script_msg() calls. This pending state is transferred to
     *          the active rangefinder state during update() calls.
     *          
     *          This two-stage buffering approach provides thread-safe communication
     *          between the Lua scripting thread (which writes to _state_pending)
     *          and the sensor thread (which reads from _state_pending in update()).
     *          
     *          The structure includes:
     *          - distance_m: Most recent distance measurement
     *          - last_reading_ms: Timestamp of most recent measurement
     *          - status: Sensor health/validity status
     *          - signal_quality_pct: Optional signal quality metric
     *          - voltage: Optional sensor supply voltage
     * 
     * @note Written by handle_script_msg() in scripting thread context
     * @note Read by update() in sensor thread context
     * @note State is atomically copied to prevent partial reads
     */
    RangeFinder::RangeFinder_State _state_pending = {};
};

#endif  // AP_RANGEFINDER_LUA_ENABLED
