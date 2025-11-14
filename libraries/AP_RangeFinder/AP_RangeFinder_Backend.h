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

/**
 * @file AP_RangeFinder_Backend.h
 * @brief RangeFinder backend base class interface and shared functionality
 * 
 * @details This file defines the abstract base class that all rangefinder/distance sensor
 *          drivers must inherit from. It provides the common interface for sensor operations,
 *          shared state management, parameter storage, and thread-safe access to sensor data.
 *          
 *          The backend architecture allows ArduPilot to support multiple distance sensor types
 *          through a unified interface while enabling sensor-specific optimizations in derived classes.
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include "AP_RangeFinder.h"

/**
 * @class AP_RangeFinder_Backend
 * @brief Abstract base class for all rangefinder/distance sensor drivers
 * 
 * @details This class defines the interface that all rangefinder backend implementations
 *          must provide. It manages shared state, parameters, and provides common functionality
 *          for sensor initialization, data updates, and status reporting.
 *          
 *          Backend Lifecycle:
 *          1. detect() - Static factory method in derived class checks for sensor presence
 *          2. constructor() - Initializes backend with state and parameter references
 *          3. init_serial() - Optional serial port initialization for UART-based sensors
 *          4. update() - Called periodically (typically 10-50Hz) to read sensor and update state
 *          
 *          Derived classes must implement:
 *          - update() - Perform sensor reading and update state.distance_m, state.status
 *          - _get_mav_distance_sensor_type() - Return MAVLink sensor type classification
 *          
 *          Thread Safety:
 *          The _sem semaphore protects shared state access. Backends should use WITH_SEMAPHORE(_sem)
 *          when accessing state members from different contexts (e.g., sensor IRQ handlers).
 *          
 * @note All distance values are in meters, voltage in millivolts, time in milliseconds
 * @warning Backends must update state.last_reading_ms even on failed readings to detect sensor timeouts
 */
class AP_RangeFinder_Backend
{
public:
    /**
     * @brief Construct a rangefinder backend instance
     * 
     * @details Initializes the backend with references to shared state and parameters.
     *          The constructor stores references and initializes the backend type.
     *          Actual sensor initialization happens in init_serial() or first update() call.
     * 
     * @param[in,out] _state Reference to RangeFinder_State structure containing distance_m,
     *                       signal_quality_pct, voltage_mv, status, range_valid_count, last_reading_ms
     * @param[in] _params Reference to AP_RangeFinder_Params containing type, orientation, min_distance,
     *                    max_distance, pos_offset, ground_clearance configuration
     * 
     * @note This incorporates initialization - no separate init() method required
     */
	AP_RangeFinder_Backend(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Virtual destructor for proper cleanup in derived classes
     * 
     * @details Allows rangefinder drivers to override with custom cleanup code if needed.
     *          Default implementation performs no action. Derived classes should release
     *          any allocated resources (e.g., DMA buffers, sensor handles) in their destructor.
     */
    virtual ~AP_RangeFinder_Backend(void) {}

    /**
     * @brief Update sensor state with new distance measurement (pure virtual)
     * 
     * @details This method MUST be implemented by all derived classes. It is called periodically
     *          by the scheduler to read the sensor and update the state structure.
     *          
     *          Implementation responsibilities:
     *          - Read distance from hardware sensor
     *          - Update state.distance_m with measured distance in meters
     *          - Update state.last_reading_ms with current system time (AP_HAL::millis())
     *          - Call update_status() to set state.status based on reading validity
     *          - Optionally update state.signal_quality_pct (0-100) and state.voltage_mv
     *          
     * @note Called at 10-50Hz depending on sensor type and system load
     * @warning Must update last_reading_ms even on failed readings to enable timeout detection
     */
    virtual void update() = 0;

    /**
     * @brief Initialize serial port for UART-based rangefinders
     * 
     * @details Optional method for sensors that communicate via serial UART. If implemented,
     *          configures baud rate, parity, stop bits, and begins communication with the sensor.
     *          Default implementation does nothing (for non-serial sensors like I2C, PWM, analog).
     * 
     * @param[in] serial_instance Serial port number (0-based) as configured in parameters
     * 
     * @note Only called if sensor type is serial-based (e.g., LightWare serial, MaxBotix UART)
     */
    virtual void init_serial(uint8_t serial_instance) {};

    /**
     * @brief Process incoming MAVLink DISTANCE_SENSOR message
     * 
     * @details Handles MAVLink distance sensor messages for MAVLink-based rangefinders.
     *          Default implementation does nothing. Override in derived classes that accept
     *          distance measurements via MAVLink (e.g., external sensors, companion computer).
     *          
     *          Typical implementation extracts distance from MAVLink message and updates state.
     * 
     * @param[in] msg MAVLink message to process (typically DISTANCE_SENSOR message ID 132)
     * 
     * @note Only relevant for MAVLink-based distance sensors
     */
    virtual void handle_msg(const mavlink_message_t &msg) { return; }

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Export current rangefinder state to Lua scripting
     * 
     * @details Copies current sensor state to output parameter for access from Lua scripts.
     *          Allows scripts to read distance, status, signal quality, and other sensor data.
     * 
     * @param[out] state_arg RangeFinder_State structure to populate with current sensor state
     * 
     * @note Thread-safe: uses semaphore internally to protect state access
     */
    void get_state(RangeFinder::RangeFinder_State &state_arg);

    /**
     * @brief Handle distance update from Lua script (legacy interface)
     * 
     * @details Allows Lua scripts to provide distance measurements to a scripting-backed rangefinder.
     *          This is the legacy single-parameter interface. Default implementation returns false.
     *          Override in AP_RangeFinder_Lua backend to accept script-provided distances.
     * 
     * @param[in] dist_m Distance measurement from script in meters
     * 
     * @return true if distance accepted and state updated, false if scripting backend not configured
     * 
     * @note Legacy interface - prefer handle_script_msg(const RangeFinder_State&) for new code
     */
    virtual bool handle_script_msg(float dist_m) { return false; }

    /**
     * @brief Handle full state update from Lua script
     * 
     * @details Allows Lua scripts to provide complete rangefinder state (distance, quality, voltage)
     *          to a scripting-backed rangefinder. Default implementation returns false.
     *          Override in AP_RangeFinder_Lua backend to accept script-provided state.
     * 
     * @param[in] state_arg Complete RangeFinder_State from script including distance_m, signal_quality_pct,
     *                      voltage_mv, status, range_valid_count
     * 
     * @return true if state accepted and internal state updated, false if scripting backend not configured
     * 
     * @note Preferred interface for Lua scripting - provides full state control
     */
    virtual bool handle_script_msg(const RangeFinder::RangeFinder_State &state_arg) { return false; }
#endif

#if HAL_MSP_RANGEFINDER_ENABLED
    /**
     * @brief Handle distance update from MSP protocol
     * 
     * @details Processes MSP (MultiWii Serial Protocol) rangefinder messages for MSP-based sensors.
     *          Default implementation does nothing. Override in AP_RangeFinder_MSP to handle
     *          distance updates from MSP OSD or flight controller integration.
     * 
     * @param[in] pkt MSP rangefinder data packet containing distance and quality information
     * 
     * @note Only relevant for MSP-based distance sensors (e.g., DJI FPV, MSP OSD integration)
     */
    virtual void handle_msp(const MSP::msp_rangefinder_data_message_t &pkt) { return; }
#endif

    /**
     * @brief Get configured sensor mounting orientation
     * 
     * @details Returns the rotation/orientation of the sensor relative to vehicle body frame.
     *          Common orientations: ROTATION_PITCH_270 (down-facing), ROTATION_NONE (forward-facing).
     * 
     * @return enum Rotation Sensor orientation (e.g., ROTATION_PITCH_270 for downward-facing)
     * 
     * @note Configured via RNGFNDx_ORIENT parameter
     */
    enum Rotation orientation() const { return (Rotation)params.orientation.get(); }

    /**
     * @brief Get current distance measurement
     * 
     * @details Returns the most recent distance reading from the sensor.
     *          Check status() to verify reading validity before using.
     * 
     * @return Current measured distance in meters
     * 
     * @note Returns last known distance even if sensor status is not Good
     * @warning Always check status() or has_data() before relying on distance value
     */
    float distance() const { return state.distance_m; }

    /**
     * @brief Get signal quality percentage
     * 
     * @details Returns signal quality/confidence of current distance measurement if sensor provides it.
     *          Quality indication varies by sensor type (e.g., IR reflectivity, ultrasonic echo strength).
     * 
     * @return Signal quality as percentage 0-100, or -1 if sensor doesn't provide quality metric
     * 
     * @warning WARN_IF_UNUSED - Caller should check return value (important for safety-critical applications)
     */
    int8_t signal_quality_pct() const  WARN_IF_UNUSED { return state.signal_quality_pct; }

    /**
     * @brief Get sensor supply voltage
     * 
     * @details Returns sensor operating voltage if sensor provides this diagnostic.
     *          Useful for detecting sensor power issues or brownout conditions.
     * 
     * @return Sensor voltage in millivolts (mV), or 0 if sensor doesn't report voltage
     */
    uint16_t voltage_mv() const { return state.voltage_mv; }

    /**
     * @brief Get maximum valid distance range
     * 
     * @details Returns the maximum distance the sensor can reliably measure.
     *          Readings beyond this distance will be marked OutOfRangeHigh.
     * 
     * @return Maximum distance in meters
     * 
     * @note Configured via RNGFNDx_MAX_CM parameter (stored as centimeters, returned as meters)
     * @note Virtual to allow sensor-specific overrides (e.g., temperature-dependent range)
     */
    virtual float max_distance() const { return params.max_distance; }

    /**
     * @brief Get minimum valid distance range
     * 
     * @details Returns the minimum distance the sensor can reliably measure.
     *          Readings below this distance will be marked OutOfRangeLow.
     * 
     * @return Minimum distance in meters
     * 
     * @note Configured via RNGFNDx_MIN_CM parameter (stored as centimeters, returned as meters)
     * @note Virtual to allow sensor-specific overrides
     */
    virtual float min_distance() const { return params.min_distance; }

    /**
     * @brief Get ground clearance offset
     * 
     * @details Returns the minimum altitude above ground level for safe operation.
     *          Used in landing detection and terrain following to account for sensor position.
     * 
     * @return Ground clearance in meters
     * 
     * @note Configured via RNGFNDx_GNDCLEAR parameter
     */
    float ground_clearance() const { return params.ground_clearance; }
    /**
     * @brief Get MAVLink distance sensor type classification
     * 
     * @details Returns the MAV_DISTANCE_SENSOR enum value for MAVLink DISTANCE_SENSOR messages.
     *          Type classification: laser, ultrasonic, infrared, radar (see MAVLink specification).
     *          Calls pure virtual _get_mav_distance_sensor_type() implemented by each backend.
     * 
     * @return MAV_DISTANCE_SENSOR enum (e.g., MAV_DISTANCE_SENSOR_LASER, MAV_DISTANCE_SENSOR_ULTRASOUND)
     * 
     * @note Used for telemetry and logging to identify sensor technology type
     */
    MAV_DISTANCE_SENSOR get_mav_distance_sensor_type() const;

    /**
     * @brief Get current sensor status
     * 
     * @details Returns the health/validity status of the sensor and its readings.
     *          Status values:
     *          - NotConnected: Sensor not detected or communication lost
     *          - NoData: Sensor detected but not providing measurements
     *          - OutOfRangeLow: Distance below min_distance threshold
     *          - OutOfRangeHigh: Distance above max_distance threshold  
     *          - Good: Valid distance measurement within range
     * 
     * @return RangeFinder::Status enum indicating sensor health and reading validity
     * 
     * @note Check this before using distance() for flight-critical decisions
     */
    RangeFinder::Status status() const;

    /**
     * @brief Get configured rangefinder type
     * 
     * @details Returns the sensor type from parameters (e.g., LightWare, MaxBotix, Analog).
     *          This is the configured type which may differ from allocated_type() if changed at runtime.
     * 
     * @return RangeFinder::Type enum indicating configured sensor type
     * 
     * @note Configured via RNGFNDx_TYPE parameter
     * @see allocated_type() for actual instantiated backend type
     */
    RangeFinder::Type type() const { return (RangeFinder::Type)params.type.get(); }

    /**
     * @brief Check if sensor is providing valid data
     * 
     * @details Returns true if sensor status is Good and sensor is actively providing measurements.
     *          This is a convenience method that combines status check and data freshness check.
     * 
     * @return true if sensor returning valid data, false if not connected, no data, or out of range
     * 
     * @note Preferred method for checking sensor health before using distance measurements
     */
    bool has_data() const;

    /**
     * @brief Get consecutive valid reading count
     * 
     * @details Returns number of consecutive Good status readings (maxes at 10).
     *          Used to implement hysteresis - require multiple good readings before trusting sensor.
     *          Count resets to 0 on any non-Good status.
     * 
     * @return Count of consecutive valid readings (0-10)
     * 
     * @note Provides confidence metric - higher count means more stable/reliable readings
     */
    uint8_t range_valid_count() const { return state.range_valid_count; }

    /**
     * @brief Get sensor position offset in body frame
     * 
     * @details Returns 3D position vector of sensor relative to vehicle center of gravity.
     *          Vector components in body frame: +X forward, +Y right, +Z down (NED convention).
     *          Used for terrain following, precision landing, and distance corrections.
     * 
     * @return Vector3f position offset in meters (X=forward, Y=right, Z=down in body frame)
     * 
     * @note Configured via RNGFNDx_POS_X, RNGFNDx_POS_Y, RNGFNDx_POS_Z parameters
     */
    const Vector3f &get_pos_offset() const { return params.pos_offset; }

    /**
     * @brief Get timestamp of last sensor reading
     * 
     * @details Returns system time when sensor was last successfully read (or read attempted).
     *          Used for timeout detection and data freshness checks. Updated by update() method.
     * 
     * @return System time in milliseconds since boot (from AP_HAL::millis())
     * 
     * @note Updated even on failed readings to enable timeout detection
     * @warning Time rolls over after ~49 days - use time deltas for comparisons
     */
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

    /**
     * @brief Get sensor temperature reading
     * 
     * @details Retrieves temperature from sensors that provide internal temperature measurement.
     *          Default implementation returns false (no temperature available).
     *          Override in sensor-specific backends that support temperature reading.
     * 
     * @param[out] temp Temperature in degrees Celsius
     * 
     * @return true if temperature reading successful and temp populated, false if not supported
     * 
     * @note Useful for temperature compensation and sensor health monitoring
     */
    virtual bool get_temp(float &temp) const { return false; }

    /**
     * @brief Get actual allocated backend type
     * 
     * @details Returns the actual backend type that was instantiated, as opposed to the
     *          parameter value which may be changed at runtime after backend creation.
     *          Set during backend construction and never changes.
     * 
     * @return RangeFinder::Type enum of actual instantiated backend (e.g., Type::LIGHTWARE_SERIAL)
     * 
     * @note Differs from type() when parameter is changed after initialization
     * @note Used for identifying actual hardware driver vs configured type
     */
    RangeFinder::Type allocated_type() const { return _backend_type; }

protected:

    /**
     * @brief Update sensor status based on distance vs thresholds
     * 
     * @details Updates status field by comparing distance measurement against min/max thresholds.
     *          Sets status to OutOfRangeLow, OutOfRangeHigh, or Good based on distance_m value.
     *          Also updates range_valid_count (increments on Good, resets on non-Good).
     * 
     * @param[in,out] state_arg RangeFinder_State to update with new status and valid_count
     * 
     * @note Called by backend update() after setting distance_m
     * @note Uses params.min_distance and params.max_distance for threshold comparison
     */
    void update_status(RangeFinder::RangeFinder_State &state_arg) const;

    /**
     * @brief Update status using internal state reference
     * 
     * @details Convenience wrapper that calls update_status(state) using the internal state reference.
     *          Use this after updating state.distance_m in backend update() implementations.
     */
    void update_status() { update_status(state); }

    /**
     * @brief Set status and update consecutive valid count
     * 
     * @details Directly sets sensor status and updates range_valid_count accordingly.
     *          Increments valid_count (max 10) if status is Good, resets to 0 otherwise.
     *          Static method allows updating external state structures.
     * 
     * @param[in,out] state_arg RangeFinder_State to update with new status
     * @param[in] status New RangeFinder::Status value (NotConnected, NoData, OutOfRangeLow, OutOfRangeHigh, Good)
     * 
     * @note Prefer update_status() if status depends on distance thresholds
     */
    static void set_status(RangeFinder::RangeFinder_State &state_arg, RangeFinder::Status status);

    /**
     * @brief Set status using internal state reference
     * 
     * @details Convenience wrapper that calls set_status(state, status) using internal state reference.
     *          Use when status is determined by factors other than distance thresholds.
     */
    void set_status(RangeFinder::Status status) { set_status(state, status); }

    /**
     * @brief Reference to shared sensor state structure
     * 
     * @details Contains current sensor measurements and status:
     *          - distance_m: Current distance in meters
     *          - signal_quality_pct: Quality 0-100 or -1 for unknown
     *          - voltage_mv: Sensor voltage in millivolts
     *          - status: RangeFinder::Status enum
     *          - range_valid_count: Consecutive good readings (0-10)
     *          - last_reading_ms: Timestamp in milliseconds
     * 
     * @note Protected by _sem semaphore for thread-safe access
     * @warning Always use WITH_SEMAPHORE(_sem) when accessing from IRQ or different context
     */
    RangeFinder::RangeFinder_State &state;

    /**
     * @brief Reference to sensor configuration parameters
     * 
     * @details Contains sensor configuration:
     *          - type: RangeFinder::Type enum
     *          - orientation: Rotation enum for sensor mounting
     *          - min_distance: Minimum valid range in meters
     *          - max_distance: Maximum valid range in meters
     *          - ground_clearance: Ground clearance offset in meters
     *          - pos_offset: 3D position in body frame (meters)
     * 
     * @note Parameters are live - changes via GCS take effect immediately
     */
    AP_RangeFinder_Params &params;

    /**
     * @brief Semaphore for thread-safe state access
     * 
     * @details Protects shared state structure from concurrent access.
     *          Use WITH_SEMAPHORE(_sem) when accessing state from different contexts
     *          (e.g., sensor interrupt handlers, I2C callbacks, timer callbacks).
     * 
     * @warning Critical for sensors using interrupts or DMA - race conditions can cause
     *          incorrect distance readings or vehicle instability
     */
    HAL_Semaphore _sem;

    /**
     * @brief Actual backend type instantiated
     * 
     * @details Stores the RangeFinder::Type that was used to create this backend instance.
     *          Set during construction and immutable thereafter. Used to identify actual
     *          hardware driver when parameter may have changed post-initialization.
     * 
     * @note Returned by allocated_type() method
     */
    RangeFinder::Type _backend_type;

    /**
     * @brief Get MAVLink distance sensor type for this backend (pure virtual)
     * 
     * @details Each backend must implement this to return appropriate MAV_DISTANCE_SENSOR type:
     *          - MAV_DISTANCE_SENSOR_LASER for LightWare, Benewake, etc.
     *          - MAV_DISTANCE_SENSOR_ULTRASOUND for MaxBotix, HC-SR04, etc.
     *          - MAV_DISTANCE_SENSOR_INFRARED for Sharp IR, GP2Y0A21YK, etc.
     *          - MAV_DISTANCE_SENSOR_RADAR for radar-based sensors
     * 
     * @return MAV_DISTANCE_SENSOR enum appropriate for this sensor technology
     * 
     * @note Used in MAVLink DISTANCE_SENSOR messages and logging
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const = 0;
};

#endif  // AP_RANGEFINDER_ENABLED
