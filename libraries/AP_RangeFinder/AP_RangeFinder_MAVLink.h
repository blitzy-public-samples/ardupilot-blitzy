/**
 * @file AP_RangeFinder_MAVLink.h
 * @brief MAVLink rangefinder backend for remote distance sensors
 * 
 * @details This header defines the AP_RangeFinder_MAVLink class, which implements
 *          a rangefinder backend that receives distance measurements via MAVLink
 *          DISTANCE_SENSOR messages. This enables integration of remote rangefinders
 *          connected to companion computers, external flight controllers, or other
 *          MAVLink-capable systems.
 *          
 *          The backend processes MAVLink DISTANCE_SENSOR messages (message ID 132)
 *          and extracts distance, min/max range, and sensor type information. This
 *          allows ArduPilot to use rangefinders that are not directly connected to
 *          the flight controller hardware.
 *          
 *          Common use cases:
 *          - Companion computer interfacing custom distance sensors
 *          - Secondary flight controller sharing rangefinder data
 *          - ROS nodes publishing distance measurements
 *          - External vision systems providing obstacle distance
 * 
 * @note This backend is enabled when AP_RANGEFINDER_MAVLINK_ENABLED is defined
 * @see AP_RangeFinder_Backend for base class interface
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.h
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_MAVLINK_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

/**
 * @brief Maximum time without receiving MAVLink distance data before timeout
 * 
 * @details If no DISTANCE_SENSOR messages are received within this timeout period,
 *          the rangefinder will be marked as unhealthy and distance readings will
 *          be considered stale. This prevents the system from using old data if
 *          the MAVLink connection is lost or the remote sensor stops transmitting.
 * 
 * @note Value is in milliseconds (500ms = 0.5 seconds)
 */
#define AP_RANGEFINDER_MAVLINK_TIMEOUT_MS 500

/**
 * @class AP_RangeFinder_MAVLink
 * @brief Rangefinder backend for receiving distance measurements via MAVLink protocol
 * 
 * @details This class implements a rangefinder backend that receives distance sensor
 *          data through MAVLink DISTANCE_SENSOR messages rather than directly reading
 *          hardware sensors. This enables "virtual" rangefinder integration where the
 *          actual sensor is connected to a remote system (companion computer, secondary
 *          flight controller, or external processor) that forwards measurements via
 *          MAVLink telemetry.
 *          
 *          **Architecture**:
 *          - Inherits from AP_RangeFinder_Backend for standard rangefinder interface
 *          - Receives DISTANCE_SENSOR messages (MAVLink message ID 132) via handle_msg()
 *          - Stores distance, min/max range, and sensor type from messages
 *          - Implements timeout checking to detect stale data
 *          - Provides standard update() interface for rangefinder state management
 *          
 *          **MAVLink Integration**:
 *          The backend processes DISTANCE_SENSOR messages containing:
 *          - current_distance: Current distance measurement (cm)
 *          - min_distance: Minimum measurable distance (cm)
 *          - max_distance: Maximum measurable distance (cm)
 *          - type: Sensor type (MAV_DISTANCE_SENSOR enum)
 *          - signal_quality: Optional signal quality indicator (0-100%)
 *          
 *          **Lifecycle**:
 *          1. detect() called during rangefinder initialization
 *          2. Constructor initializes backend with rangefinder state reference
 *          3. handle_msg() called when DISTANCE_SENSOR messages arrive
 *          4. update() called periodically to check for timeouts
 *          5. max_distance() and min_distance() queried by rangefinder library
 *          
 *          **Use Cases**:
 *          - Companion computer running custom sensor drivers
 *          - ROS nodes publishing range measurements to ArduPilot
 *          - External vision systems providing obstacle distances
 *          - Secondary flight controllers sharing sensor data
 *          - Network-connected rangefinders via MAVLink routing
 * 
 * @note This backend always reports as "detected" since presence is indicated
 *       by user configuration rather than hardware probing
 * @warning Ensure MAVLink message frequency is >2Hz to avoid timeout
 * @see AP_RangeFinder_Backend for base class interface
 * @see MAVLink DISTANCE_SENSOR message definition (message ID 132)
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.h
 */
class AP_RangeFinder_MAVLink : public AP_RangeFinder_Backend
{

public:

    /**
     * @brief Constructor - inherits from AP_RangeFinder_Backend
     * 
     * @details Uses the base class constructor to initialize the rangefinder backend
     *          with references to the rangefinder state and parameters. No additional
     *          initialization is required for MAVLink backend since it waits for
     *          incoming messages rather than initializing hardware.
     * 
     * @note Constructor is inherited using C++11 constructor inheritance syntax
     */
    using AP_RangeFinder_Backend::AP_RangeFinder_Backend;

    /**
     * @brief Detect presence of MAVLink rangefinder
     * 
     * @details Always returns true because MAVLink rangefinders cannot be detected
     *          through hardware probing. If the user has configured RANGEFINDER_TYPE
     *          to MAVLink, we assume an external system will send DISTANCE_SENSOR
     *          messages. Detection is based on configuration intent rather than
     *          hardware presence.
     *          
     *          Unlike physical sensors (I2C, serial, analog), MAVLink rangefinders
     *          are "virtual" sensors where data arrives over the telemetry link.
     *          The actual sensor may be connected to:
     *          - A companion computer (Raspberry Pi, Jetson, etc.)
     *          - A secondary flight controller
     *          - An external processor or microcontroller
     *          - A ROS node forwarding sensor data
     * 
     * @return true Always returns true to indicate MAVLink rangefinder is available
     * 
     * @note This detection approach means the rangefinder will appear "present" even
     *       if no messages are received. Use timeout checking in update() to determine
     *       if data is actually being received.
     * @see update() for timeout checking of received data
     */
    static bool detect() { return true; }

    /**
     * @brief Update rangefinder state and check for data timeout
     * 
     * @details Called periodically by the rangefinder library to update sensor state.
     *          For MAVLink rangefinders, this primarily checks whether fresh data has
     *          been received within the timeout period (AP_RANGEFINDER_MAVLINK_TIMEOUT_MS).
     *          
     *          **Update Process**:
     *          1. Check time since last DISTANCE_SENSOR message received
     *          2. If timeout exceeded, mark sensor as unhealthy
     *          3. If fresh data available, update rangefinder state with distance value
     *          4. Apply any necessary filtering or validation
     *          
     *          Unlike hardware sensors that actively read during update(), MAVLink
     *          sensors are passive - they only process data already received via
     *          handle_msg(). This method validates the age of that data.
     * 
     * @note Called at rangefinder library update rate (typically 10-20Hz)
     * @warning If no messages received for >500ms, sensor will be marked unhealthy
     * @see handle_msg() where distance data is actually received and stored
     * @see AP_RANGEFINDER_MAVLINK_TIMEOUT_MS for timeout threshold
     */
    void update(void) override;

    /**
     * @brief Process incoming MAVLink DISTANCE_SENSOR messages
     * 
     * @details This is the core method where distance measurements are received from
     *          the MAVLink telemetry stream. Called by the MAVLink message routing
     *          system when a DISTANCE_SENSOR message (message ID 132) arrives that
     *          matches this rangefinder's sensor ID.
     *          
     *          **Message Processing**:
     *          1. Validate message type is DISTANCE_SENSOR (ID 132)
     *          2. Extract distance measurement from current_distance field
     *          3. Store min_distance and max_distance sensor capabilities
     *          4. Record sensor type (MAV_DISTANCE_SENSOR enum)
     *          5. Extract optional signal quality indicator
     *          6. Update timestamp for timeout checking
     *          
     *          **DISTANCE_SENSOR Message Format** (MAVLink common.xml):
     *          - time_boot_ms: System time when measurement was taken
     *          - min_distance: Minimum measurable distance (cm)
     *          - max_distance: Maximum measurable distance (cm)
     *          - current_distance: Current distance reading (cm)
     *          - type: Sensor type (laser, ultrasonic, infrared, etc.)
     *          - id: Sensor instance ID (used for routing to correct backend)
     *          - orientation: Sensor facing direction (forward, down, etc.)
     *          - covariance: Measurement uncertainty (cm²)
     *          - horizontal_fov: Horizontal field of view (rad)
     *          - vertical_fov: Vertical field of view (rad)
     *          - quaternion: Sensor orientation quaternion (optional)
     *          - signal_quality: Signal quality 0-100% (0=invalid, 100=perfect)
     *          
     *          **Remote Rangefinder Integration**:
     *          This method enables complete integration of rangefinders connected to
     *          external systems. The remote system (companion computer, ROS node,
     *          secondary flight controller) reads the physical sensor and publishes
     *          measurements via MAVLink. ArduPilot treats these measurements identically
     *          to locally-connected sensors for obstacle avoidance, terrain following,
     *          altitude hold, and precision landing.
     *          
     *          **Typical Message Flow**:
     *          External System -> MAVLink DISTANCE_SENSOR -> GCS_MAVLink routing ->
     *          AP_RangeFinder_MAVLink::handle_msg() -> Distance stored ->
     *          update() validates timeout -> Distance available to vehicle
     * 
     * @param[in] msg MAVLink message structure containing DISTANCE_SENSOR data
     * 
     * @note This method is called from MAVLink receive context, not main thread
     * @note Message routing ensures only messages matching this sensor's ID are received
     * @warning Ensure message rate >2Hz to prevent timeout (500ms threshold)
     * @see update() for timeout checking after messages are received
     * @see MAVLink DISTANCE_SENSOR message definition for complete field descriptions
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp (implementation)
     */
    void handle_msg(const mavlink_message_t &msg) override;

    /**
     * @brief Get maximum measurable distance for this rangefinder
     * 
     * @details Returns the maximum distance capability reported by the remote sensor
     *          in the most recent DISTANCE_SENSOR message. This value comes from the
     *          max_distance field of the MAVLink message and represents the sensor's
     *          hardware limitation, not the current reading.
     *          
     *          The max distance is used by:
     *          - Obstacle avoidance to determine detection range
     *          - Terrain following for maximum lookahead distance
     *          - Rangefinder health checking (readings beyond max are invalid)
     *          - Mission planning for sensor capability assessment
     * 
     * @return Maximum distance in meters that the sensor can measure
     * 
     * @note Value is extracted from MAVLink DISTANCE_SENSOR max_distance field
     * @note Returns 0.0 if no messages have been received yet
     * @see handle_msg() where max_distance is extracted from MAVLink message
     */
    float max_distance() const override;
    
    /**
     * @brief Get minimum measurable distance for this rangefinder
     * 
     * @details Returns the minimum distance capability reported by the remote sensor
     *          in the most recent DISTANCE_SENSOR message. This value comes from the
     *          min_distance field of the MAVLink message and represents the sensor's
     *          minimum reliable range (below which readings are unreliable or invalid).
     *          
     *          The min distance is used by:
     *          - Altitude estimation to filter too-close readings
     *          - Landing detection to determine valid range
     *          - Rangefinder health checking (readings below min are invalid)
     *          - Sensor fusion to determine measurement validity
     * 
     * @return Minimum distance in meters that the sensor can reliably measure
     * 
     * @note Value is extracted from MAVLink DISTANCE_SENSOR min_distance field
     * @note Returns 0.0 if no messages have been received yet
     * @see handle_msg() where min_distance is extracted from MAVLink message
     */
    float min_distance() const override;

protected:

    /**
     * @brief Get the MAVLink sensor type classification for this rangefinder
     * 
     * @details Returns the sensor type reported by the remote rangefinder in the
     *          DISTANCE_SENSOR message. This classification indicates the sensing
     *          technology (laser, ultrasonic, infrared, radar, etc.) and is used
     *          for logging, GCS display, and sensor fusion decisions.
     *          
     *          **MAV_DISTANCE_SENSOR Types**:
     *          - MAV_DISTANCE_SENSOR_LASER: Laser rangefinder (lidar)
     *          - MAV_DISTANCE_SENSOR_ULTRASOUND: Ultrasonic rangefinder (sonar)
     *          - MAV_DISTANCE_SENSOR_INFRARED: Infrared rangefinder
     *          - MAV_DISTANCE_SENSOR_RADAR: Radar-based distance sensor
     *          - MAV_DISTANCE_SENSOR_UNKNOWN: Type not specified or not recognized
     *          
     *          The sensor type is extracted from the DISTANCE_SENSOR message's type
     *          field and stored when messages are received via handle_msg().
     * 
     * @return MAV_DISTANCE_SENSOR enum value indicating sensor technology type
     * 
     * @note Returns MAV_DISTANCE_SENSOR_UNKNOWN if no messages received yet
     * @see handle_msg() where sensor_type is extracted from MAVLink message
     * @see MAVLink MAV_DISTANCE_SENSOR enum definition for complete type list
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return sensor_type;
    }

private:

    /**
     * @brief Current distance measurement from MAVLink DISTANCE_SENSOR message
     * 
     * @details Stores the most recent distance reading received via handle_msg().
     *          This value is extracted from the current_distance field of the
     *          DISTANCE_SENSOR message and represents the actual measured distance
     *          from the remote sensor.
     * 
     * @note Units: meters (converted from centimeters in MAVLink message)
     * @note Updated each time handle_msg() processes a DISTANCE_SENSOR message
     * @see handle_msg() where this value is updated from MAVLink message
     */
    float distance;
    
    /**
     * @brief Maximum measurable distance capability of the remote sensor
     * 
     * @details Stores the maximum distance the sensor can measure, as reported
     *          in the max_distance field of DISTANCE_SENSOR messages. This represents
     *          the sensor's hardware maximum range, not the current reading.
     * 
     * @note Units: meters (converted from centimeters in MAVLink message)
     * @note Typically constant for a given sensor, but can change if sensor is reconfigured
     * @see max_distance() returns this value to rangefinder library
     */
    float _max_distance;
    
    /**
     * @brief Minimum reliable measurement distance of the remote sensor
     * 
     * @details Stores the minimum distance the sensor can reliably measure, as
     *          reported in the min_distance field of DISTANCE_SENSOR messages.
     *          Measurements below this threshold are considered unreliable due to
     *          sensor limitations (e.g., ultrasonic dead zone, laser minimum focus).
     * 
     * @note Units: meters (converted from centimeters in MAVLink message)
     * @note Typically constant for a given sensor type
     * @see min_distance() returns this value to rangefinder library
     */
    float _min_distance;
    
    /**
     * @brief Signal quality indicator from remote sensor (0-100%)
     * 
     * @details Optional quality metric provided by some rangefinders in the
     *          DISTANCE_SENSOR message signal_quality field. Interpretation:
     *          - 0: Invalid signal or quality measurement not supported
     *          - 1-99: Quality percentage (higher is better)
     *          - 100: Perfect signal quality
     *          
     *          Can be used for:
     *          - Sensor health monitoring
     *          - Filtering unreliable measurements
     *          - Logging and diagnostics
     *          - Dynamic sensor weighting in fusion algorithms
     * 
     * @note int8_t range -127 to 127, but valid range is 0-100
     * @note Not all sensors provide signal quality; may remain 0 if unsupported
     */
    int8_t signal_quality;

    /**
     * @brief Initiate a new distance measurement (not used for MAVLink backend)
     * 
     * @details This method is part of the rangefinder backend interface but is not
     *          applicable to MAVLink rangefinders since measurements are pushed via
     *          messages rather than pulled by polling. For hardware sensors, this
     *          would trigger a new reading, but MAVLink sensors operate passively.
     * 
     * @return bool Success status (implementation defined)
     * 
     * @note Legacy interface method, may not be actively used for MAVLink backend
     */
    static bool start_reading(void);
    
    /**
     * @brief Retrieve a distance reading (not used for MAVLink backend)
     * 
     * @details This method is part of the rangefinder backend interface but is not
     *          applicable to MAVLink rangefinders. For hardware sensors, this would
     *          retrieve a reading after start_reading(), but MAVLink sensors receive
     *          data asynchronously via handle_msg().
     * 
     * @param[out] reading_cm Distance reading in centimeters
     * 
     * @return bool Success status (implementation defined)
     * 
     * @note Legacy interface method, may not be actively used for MAVLink backend
     */
    static bool get_reading(uint16_t &reading_cm);

    /**
     * @brief Sensor technology type from MAVLink DISTANCE_SENSOR message
     * 
     * @details Stores the sensor type classification (laser, ultrasonic, infrared,
     *          radar, etc.) as reported in the type field of DISTANCE_SENSOR messages.
     *          Initialized to MAV_DISTANCE_SENSOR_UNKNOWN and updated when first
     *          message is received.
     *          
     *          This information is useful for:
     *          - Logging and diagnostics
     *          - GCS display of sensor type
     *          - Sensor-specific processing or filtering
     *          - Understanding sensor characteristics and limitations
     * 
     * @note Updated in handle_msg() from DISTANCE_SENSOR type field
     * @see _get_mav_distance_sensor_type() accessor method
     * @see MAVLink MAV_DISTANCE_SENSOR enum for possible values
     */
    MAV_DISTANCE_SENSOR sensor_type = MAV_DISTANCE_SENSOR_UNKNOWN;
};

#endif
