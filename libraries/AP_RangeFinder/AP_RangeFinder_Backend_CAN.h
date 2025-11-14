/**
 * @file AP_RangeFinder_Backend_CAN.h
 * @brief CAN bus rangefinder backend base class for CAN-based distance sensors
 * 
 * @details This file defines the base class for all CAN bus rangefinder implementations.
 *          It provides common functionality for CAN frame reception, distance accumulation,
 *          and integration with the ArduPilot CAN bus infrastructure via AP_CANManager
 *          and the MultiCAN pattern. Derived classes implement sensor-specific CAN
 *          protocol parsing by overriding handle_frame().
 * 
 *          The MultiCAN architecture enables multiple CAN rangefinder sensors to coexist
 *          on a single CAN bus, with frames routed to the correct backend based on
 *          configured CAN message IDs (receive_id parameter).
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BACKEND_CAN_ENABLED

#include "AP_RangeFinder_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

class RangeFinder_MultiCAN;

/**
 * @class AP_RangeFinder_Backend_CAN
 * @brief Base class for CAN bus rangefinder backends
 * 
 * @details This class provides the infrastructure for CAN-based distance sensors,
 *          integrating with AP_CANManager and the MultiCAN registry system.
 *          
 *          **Architecture Overview:**
 *          - Registers with RangeFinder_MultiCAN for CAN frame routing
 *          - Receives CAN frames via handle_frame() callback
 *          - Accumulates distance measurements via accumulate_distance_m()
 *          - Averages accumulated measurements in update() at main loop rate
 *          - Filters frames by configured receive_id for multi-sensor support
 *          
 *          **Data Flow:**
 *          1. CAN driver receives frame from bus
 *          2. MultiCAN routes frame to backend based on CAN ID
 *          3. Derived class handle_frame() parses sensor-specific protocol
 *          4. handle_frame() calls accumulate_distance_m() for valid readings
 *          5. Main loop calls update() to average accumulator and publish
 *          
 *          **MultiCAN Architecture:**
 *          Multiple CAN rangefinders can operate on a single CAN bus by configuring
 *          unique receive_id values for each sensor. The MultiCAN registry maintains
 *          a linked list of backends and routes incoming frames to the backend(s)
 *          that match the frame's CAN ID.
 *          
 *          **Derived Class Implementation:**
 *          Sensor-specific backends must:
 *          - Override handle_frame() to parse sensor CAN protocol
 *          - Call accumulate_distance_m() for each valid distance measurement
 *          - Optionally override read_timeout_ms() for sensor-specific timeout
 *          - Optionally override _get_mav_distance_sensor_type() for MAVLink reporting
 * 
 * @note Coordinate system: Distance measurements are in meters in sensor forward direction
 * @warning CAN ID conflicts: Each sensor on the bus must have a unique receive_id
 *          to avoid frame misrouting. Duplicate IDs will cause multiple backends
 *          to process the same frames, resulting in erroneous duplicate measurements.
 */
class AP_RangeFinder_Backend_CAN : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Construct a CAN rangefinder backend and register with MultiCAN
     * 
     * @details This constructor initializes the CAN rangefinder backend and registers
     *          it with the RangeFinder_MultiCAN registry for CAN frame routing.
     *          The backend will receive CAN frames matching its configured receive_id.
     *          
     *          The constructor is typically called during sensor detection/initialization
     *          phase when the CAN bus is probed for connected sensors.
     * 
     * @param[in,out] _state         Reference to RangeFinder state structure for this sensor instance
     * @param[in,out] _params        Reference to sensor-specific parameters (min/max distance, orientation, etc.)
     * @param[in]     can_type       CAN protocol type from AP_CAN::Protocol enum (e.g., AP_CAN::Protocol::RangeFinder)
     * @param[in]     driver_name    Human-readable driver name for logging and identification (e.g., "Benewake", "LightWare")
     * 
     * @note The backend is added to a linked list maintained by RangeFinder_MultiCAN
     * @note Constructor does not initialize hardware - actual CAN bus setup is handled by AP_CANManager
     */
    AP_RangeFinder_Backend_CAN(RangeFinder::RangeFinder_State &_state,
                                AP_RangeFinder_Params &_params, AP_CAN::Protocol can_type,
                                const char *driver_name);

    /**
     * @brief Friend class declaration for MultiCAN registry access
     * 
     * @details RangeFinder_MultiCAN requires friend access to manage the linked list
     *          of CAN backends (via the 'next' pointer) and to call protected methods
     *          during frame routing. This enables the MultiCAN registry to:
     *          - Add backends to the linked list during construction
     *          - Route incoming CAN frames to appropriate backends
     *          - Access backend configuration (receive_id) for frame filtering
     */
    friend class RangeFinder_MultiCAN;

    /**
     * @brief Parameter table for CAN-specific configuration
     * 
     * @details Defines AP_Param variables for CAN rangefinder configuration:
     *          - receive_id: CAN message ID to listen for (filters which frames to process)
     *          - snr_min: Minimum signal-to-noise ratio threshold (sensor-specific interpretation)
     *          
     *          These parameters are stored in EEPROM and configurable via ground station.
     *          
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_Backend_CAN.cpp parameter definitions
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Update rangefinder state with averaged distance measurements
     * 
     * @details This method is called by the main scheduler loop to update the sensor
     *          state with the latest distance measurement. It reads the accumulated
     *          distance measurements (collected via accumulate_distance_m() calls from
     *          handle_frame()), calculates the average, publishes the result, and
     *          resets the accumulator for the next cycle.
     *          
     *          **Algorithm:**
     *          1. Check timeout condition (time since last reading)
     *          2. If timeout exceeded, set state to NoData
     *          3. Otherwise, call get_reading() to retrieve averaged distance
     *          4. Publish distance to state structure
     *          5. Reset accumulator (_distance_sum, _distance_count)
     *          
     *          **Timing:**
     *          Called at main loop rate (typically 50-400Hz depending on vehicle type)
     *          Averages all CAN frames received since last update() call
     * 
     * @note Override from AP_RangeFinder_Backend base class
     * @note Thread-safety: Should be called from main thread only
     */
    virtual void update(void) override;

    /**
     * @brief Get averaged distance reading from accumulator
     * 
     * @details Retrieves the averaged distance measurement from accumulated CAN frame
     *          data. This method calculates the mean of all distance values added via
     *          accumulate_distance_m() since the last update cycle.
     *          
     *          **Calculation:**
     *          reading_m = _distance_sum / _distance_count
     *          
     *          Returns false if no measurements have been accumulated (_distance_count == 0).
     * 
     * @param[out] reading_m  Averaged distance measurement in meters
     * 
     * @return true if valid reading available (at least one measurement accumulated)
     * @return false if no measurements since last update (accumulator empty)
     * 
     * @note Units: Output distance is in meters
     * @note Called internally by update() method
     */
    bool get_reading(float &reading_m);

    /**
     * @brief Parse sensor-specific CAN frame and extract distance measurement
     * 
     * @details Pure virtual method that must be implemented by derived sensor classes
     *          to parse sensor-specific CAN protocol. This method is called by the
     *          MultiCAN registry when a CAN frame matching this backend's receive_id
     *          is received from the bus.
     *          
     *          **Typical Implementation Pattern:**
     *          1. Validate frame ID matches expected sensor CAN ID
     *          2. Parse frame data bytes according to sensor protocol
     *          3. Extract distance measurement and quality metrics
     *          4. Validate measurement (range check, SNR check)
     *          5. Call accumulate_distance_m() if measurement valid
     *          6. Return true if frame was processed (even if rejected)
     *          
     *          **Example Implementations:**
     *          - Benewake TFmini: Parse 8-byte frame with distance and signal strength
     *          - LightWare SF-series: Parse vendor-specific CAN message format
     *          - UAVCAN/DroneCAN: Parse standard RangeSensor message
     * 
     * @param[in,out] frame  CAN frame to parse (AP_HAL::CANFrame with ID and 0-8 data bytes)
     * 
     * @return true if frame was recognized and processed by this backend
     * @return false if frame should be passed to other backends (wrong ID/format)
     * 
     * @note It is essential that derived classes implement this method when relying
     *       on the base-class update() implementation
     * @note Called from CAN driver context (may be interrupt or dedicated CAN thread)
     * @warning Must be thread-safe: uses atomic operations or locks when accessing shared state
     */
    virtual bool handle_frame(AP_HAL::CANFrame &frame) = 0;

    /**
     * @brief Get timeout threshold for transitioning to NoData state
     * 
     * @details Returns the maximum time allowed between valid distance readings before
     *          the sensor state transitions to NoData. If no valid measurements are
     *          received within this timeout period, the update() method will set the
     *          sensor status to RangeFinder::Status::NoData.
     *          
     *          The default timeout of 200ms is suitable for most CAN rangefinders that
     *          typically update at 10-100Hz. Derived classes can override this for
     *          sensors with different update rates or reliability characteristics.
     * 
     * @return Timeout threshold in milliseconds (default: 200ms)
     * 
     * @note Override this method for sensors with significantly different update rates
     * @note Units: milliseconds
     */
    virtual uint32_t read_timeout_ms() const { return 200; }

    /**
     * @brief Get MAVLink distance sensor type for telemetry reporting
     * 
     * @details Returns the sensor type for MAVLink DISTANCE_SENSOR messages sent to
     *          ground control stations. The default is MAV_DISTANCE_SENSOR_RADAR for
     *          CAN sensors, as many CAN rangefinders use radar or similar RF technology.
     *          
     *          Derived classes should override this to report the correct sensor type:
     *          - MAV_DISTANCE_SENSOR_LASER for laser rangefinders
     *          - MAV_DISTANCE_SENSOR_ULTRASOUND for ultrasonic sensors
     *          - MAV_DISTANCE_SENSOR_RADAR for RF-based sensors
     *          - MAV_DISTANCE_SENSOR_INFRARED for IR sensors
     * 
     * @return MAVLink sensor type enum (default: MAV_DISTANCE_SENSOR_RADAR)
     * 
     * @note Override from AP_RangeFinder_Backend base class
     * @note Used for MAVLink telemetry reporting to ground stations
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    /**
     * @brief Check if CAN frame ID matches configured receive_id
     * 
     * @details Filters incoming CAN frames by comparing the frame's CAN ID against
     *          the configured receive_id parameter. This enables multiple CAN
     *          rangefinders to coexist on a single CAN bus by ensuring each backend
     *          only processes frames from its designated sensor.
     *          
     *          **Usage Pattern:**
     *          Typically called at the beginning of handle_frame() implementations:
     *          ```cpp
     *          bool MyCANSensor::handle_frame(AP_HAL::CANFrame &frame) {
     *              if (!is_correct_id(frame.id)) {
     *                  return false;  // Not for this backend
     *              }
     *              // Parse frame data...
     *          }
     *          ```
     * 
     * @param[in] can_id  CAN identifier from received frame to check
     * 
     * @return true if can_id matches this backend's configured receive_id
     * @return false if can_id does not match (frame should be ignored)
     * 
     * @note Essential for multi-sensor CAN bus configurations
     * @note Comparison uses configured receive_id AP_Param value
     */
    bool is_correct_id(uint32_t can_id) const;

    /**
     * @brief Add distance measurement to accumulator for averaging
     * 
     * @details Accumulates a distance measurement received from a CAN frame for later
     *          averaging by the update() method. This accumulator pattern allows the
     *          base class update() to average multiple CAN frames received between
     *          main loop cycles, improving measurement stability.
     *          
     *          **Accumulator Pattern:**
     *          1. handle_frame() receives CAN frames asynchronously from CAN driver
     *          2. handle_frame() parses sensor data and calls accumulate_distance_m()
     *          3. Multiple frames may be processed between update() calls
     *          4. update() averages all accumulated measurements and resets counters
     *          
     *          **Thread Safety:**
     *          This method modifies shared state (_distance_sum, _distance_count) that
     *          is read by update(). Implementations should ensure proper synchronization
     *          if handle_frame() is called from interrupt or separate CAN thread.
     * 
     * @param[in] distance_m  Distance measurement to accumulate, in meters
     * 
     * @note Units: distance_m must be in meters
     * @note Typically called from handle_frame() after validating sensor data
     * @note No range checking is performed - caller should validate measurement
     * @warning Thread safety: Ensure proper synchronization if called from CAN ISR
     */
    void accumulate_distance_m(float distance_m) {
        _distance_sum += distance_m;
        _distance_count++;
    };

    /**
     * @brief Linked list pointer for MultiCAN backend registry
     * 
     * @details Pointer to the next CAN rangefinder backend in the linked list maintained
     *          by RangeFinder_MultiCAN. The MultiCAN registry uses this to iterate through
     *          all registered CAN backends when routing incoming CAN frames.
     *          
     *          **Linked List Structure:**
     *          RangeFinder_MultiCAN maintains a singly-linked list of all CAN backends:
     *          MultiCAN → Backend1 → Backend2 → Backend3 → nullptr
     *          
     *          When a CAN frame arrives, MultiCAN iterates the list calling handle_frame()
     *          on each backend until one returns true.
     * 
     * @note Managed by RangeFinder_MultiCAN (friend class)
     * @note nullptr indicates end of list
     */
    AP_RangeFinder_Backend_CAN *next;

    /**
     * @brief CAN message ID to listen for
     * 
     * @details Configures which CAN message ID this backend should process. Only frames
     *          with matching CAN IDs are handled by this backend, enabling multiple
     *          CAN rangefinders with different IDs to coexist on the same CAN bus.
     *          
     *          **Parameter:** RNGFND[n]_RECV_ID where [n] is the sensor instance number
     *          **Type:** AP_Int32
     *          **Default:** Sensor-specific (often 0 or manufacturer default ID)
     *          **Range:** 0 to 0x7FF (11-bit CAN ID) or 0 to 0x1FFFFFFF (29-bit extended)
     *          
     *          **Configuration Example:**
     *          - Sensor 1: receive_id = 0x100
     *          - Sensor 2: receive_id = 0x101
     *          Both sensors can operate on CAN bus 1 without conflicts
     * 
     * @note Must be unique among all rangefinders on the same CAN bus
     * @note Configurable via ground control station parameters
     * @warning Duplicate receive_id values will cause frame misrouting
     */
    AP_Int32 receive_id;

    /**
     * @brief Minimum signal-to-noise ratio threshold
     * 
     * @details Defines the minimum acceptable signal quality for distance measurements.
     *          Measurements below this threshold are rejected to prevent noisy or
     *          unreliable data from affecting vehicle navigation.
     *          
     *          **Interpretation:** Sensor-dependent - different sensors report SNR/quality
     *          in different ways:
     *          - Some use dB or dBm scale
     *          - Some use arbitrary quality metrics (0-255)
     *          - Some use signal strength indicators
     *          
     *          **Parameter:** RNGFND[n]_SNR_MIN where [n] is the sensor instance number
     *          **Type:** AP_Int32
     *          **Default:** Sensor-specific (consult sensor datasheet)
     *          **Units:** Sensor-dependent (dB, signal strength, quality metric, etc.)
     * 
     * @note Interpretation varies by sensor type - refer to derived class documentation
     * @note Set to minimum/disabled value if sensor doesn't report SNR
     * @note Configurable via ground control station parameters
     */
    AP_Int32 snr_min;

    /**
     * @brief Pointer to MultiCAN registry for multiple CAN sensors per bus
     * 
     * @details Reference to the RangeFinder_MultiCAN registry object that manages
     *          CAN frame routing for this backend. The MultiCAN system enables multiple
     *          CAN rangefinder sensors to operate on a single CAN bus by maintaining
     *          a registry of backends and routing frames based on CAN IDs.
     *          
     *          **MultiCAN Architecture:**
     *          - One MultiCAN instance per CAN bus
     *          - MultiCAN receives frames from AP_CANManager
     *          - Routes frames to backends via linked list traversal
     *          - Each backend filters frames by receive_id
     *          
     *          This pointer is set during backend construction when registering with
     *          the MultiCAN registry.
     * 
     * @note Allows for multiple CAN rangefinders on a single bus
     * @note Set during construction, managed by RangeFinder_MultiCAN
     */
    MultiCAN* multican_rangefinder;

private:

    /**
     * @brief Accumulated distance sum for averaging
     * 
     * @details Running sum of all distance measurements received via accumulate_distance_m()
     *          since the last update() cycle. Used in conjunction with _distance_count
     *          to calculate the average distance measurement.
     *          
     *          **Accumulator Algorithm:**
     *          - Reset to 0.0 after each update() cycle
     *          - Incremented by each call to accumulate_distance_m()
     *          - Averaged as: reading_m = _distance_sum / _distance_count
     *          
     *          **Thread Safety Consideration:**
     *          Modified by accumulate_distance_m() (called from CAN context) and
     *          read/reset by update() (called from main thread). Proper synchronization
     *          may be required depending on CAN driver implementation.
     * 
     * @note Units: meters
     * @note Reset to 0.0 after each update() call
     * @note Thread safety: Access may require synchronization primitives
     */
    float _distance_sum;

    /**
     * @brief Count of accumulated distance measurements for averaging
     * 
     * @details Number of distance measurements accumulated via accumulate_distance_m()
     *          since the last update() cycle. Used as the divisor when calculating
     *          average distance in get_reading().
     *          
     *          **Accumulator Algorithm:**
     *          - Reset to 0 after each update() cycle
     *          - Incremented by each call to accumulate_distance_m()
     *          - Used to calculate average: reading_m = _distance_sum / _distance_count
     *          
     *          If _distance_count is 0, get_reading() returns false indicating no
     *          measurements were received.
     * 
     * @note Units: count (dimensionless)
     * @note Reset to 0 after each update() call
     * @note Thread safety: Access may require synchronization primitives
     */
    uint32_t _distance_count;
};

#endif  // AP_RANGEFINDER_BACKEND_CAN_ENABLED
