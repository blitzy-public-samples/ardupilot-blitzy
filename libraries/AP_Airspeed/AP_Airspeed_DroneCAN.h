/**
 * @file AP_Airspeed_DroneCAN.h
 * @brief DroneCAN/UAVCAN airspeed sensor backend for distributed CAN bus architecture
 * 
 * @details This file implements the DroneCAN (formerly UAVCAN) protocol backend for
 *          airspeed sensors connected via CAN bus. DroneCAN enables a distributed sensor
 *          architecture where airspeed sensors can be located remotely from the flight
 *          controller and communicate over the CAN bus network.
 *          
 *          Key features:
 *          - Hot-plug sensor support with automatic node detection
 *          - Multi-sensor redundancy through CAN bus
 *          - Thread-safe asynchronous message reception via Canard callbacks
 *          - Per-node data storage with freshness validation
 *          - Support for both pressure and temperature measurements
 *          - Optional hygrometer support for humidity measurements
 *          
 *          Protocol: Uses uavcan.equipment.air_data.RawAirData messages for pressure/temperature
 *                    and dronecan.sensors.hygrometer.Hygrometer for humidity data
 *          
 *          Freshness windows:
 *          - Differential pressure: 250ms timeout (flight-critical)
 *          - Temperature: 100ms timeout (supplementary data)
 * 
 * @note This backend does not perform I2C/SPI device probing - it waits for CAN messages
 * @warning Stale CAN data (exceeding freshness timeout) indicates node failure or bus issues
 * 
 * @see libraries/AP_DroneCAN/AP_DroneCAN.h for CAN protocol implementation
 * @see uavcan.equipment.air_data.RawAirData DroneCAN message specification
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_DroneCAN.h
 */

#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_DRONECAN_ENABLED

#include "AP_Airspeed_Backend.h"

#include <AP_DroneCAN/AP_DroneCAN.h>

/**
 * @class AP_Airspeed_DroneCAN
 * @brief DroneCAN/UAVCAN airspeed sensor backend implementation
 * 
 * @details This class implements airspeed sensor support over DroneCAN (UAVCAN) protocol,
 *          enabling distributed airspeed sensing via CAN bus. The backend uses a static
 *          registry pattern to support multiple CAN nodes, each representing a physical
 *          airspeed sensor on the bus.
 *          
 *          Architecture:
 *          - Static message subscription: All instances share DroneCAN message handlers
 *          - Per-node data storage: Each sensor instance maintains its own pressure/temperature data
 *          - Thread-safe access: Semaphores protect shared data from concurrent access
 *          - Asynchronous updates: CAN messages arrive via interrupt-driven callbacks
 *          
 *          Message handling flow:
 *          1. DroneCAN driver receives uavcan.equipment.air_data.RawAirData message
 *          2. Static handle_airspeed() callback invoked with message data
 *          3. Message parsed and stored in appropriate backend instance based on node_id
 *          4. Freshness timestamp updated for validation
 *          5. Flight code retrieves data via get_differential_pressure()/get_temperature()
 *          
 *          Advantages over analog sensors:
 *          - Hot-plug capability: Sensors can be added/removed while system is running
 *          - Distributed architecture: Sensors can be located remotely from flight controller
 *          - Built-in redundancy: Multiple sensors automatically supported through CAN bus
 *          - Digital precision: No ADC quantization noise
 *          - Bus ID persistence: Configuration persists across reboots via device ID
 *          
 *          Freshness checking:
 *          - Differential pressure: 250ms timeout (more critical for flight control)
 *          - Temperature: 100ms timeout (supplementary data, faster update expected)
 *          
 *          Thread safety:
 *          - _sem_airspeed: Protects per-instance pressure/temperature data
 *          - _sem_registry: Protects static _detected_modules registry
 * 
 * @note This backend does not probe hardware buses like I2C/SPI backends - it passively
 *       waits for CAN messages from configured node IDs
 * 
 * @warning Freshness checking is critical - stale CAN data indicates communication failure
 *          or sensor node malfunction. Flight code must respect freshness return values.
 * 
 * @see AP_Airspeed_Backend base class interface
 * @see AP_DroneCAN for CAN bus protocol stack
 * 
 * Source: libraries/AP_Airspeed/AP_Airspeed_DroneCAN.h
 */
class AP_Airspeed_DroneCAN : public AP_Airspeed_Backend {
public:

    /**
     * @brief Constructor - inherits from AP_Airspeed_Backend
     * 
     * @details Uses base class constructor via 'using' declaration. Initializes the
     *          DroneCAN airspeed backend with frontend reference and sensor instance number.
     * 
     * @param[in] frontend Reference to AP_Airspeed frontend managing this backend
     * @param[in] _instance Sensor instance number (0-based index for multiple sensors)
     * 
     * @note Constructor does not perform hardware initialization - init() handles registration
     * 
     * @see AP_Airspeed_Backend::AP_Airspeed_Backend()
     */
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    /**
     * @brief Initialize DroneCAN airspeed sensor backend
     * 
     * @details Registers this backend instance with the DroneCAN message subscription system.
     *          Unlike I2C/SPI backends, this does NOT probe hardware buses - it simply registers
     *          callbacks to receive uavcan.equipment.air_data.RawAirData messages from the CAN bus.
     *          
     *          Initialization steps:
     *          1. Verify DroneCAN subsystem is available
     *          2. Register static message handlers for airspeed and hygrometer messages
     *          3. Wait for CAN messages from sensor node
     *          
     *          The backend becomes active when the first message is received from a CAN node
     *          with a matching device ID. Hot-plug support means sensors can be connected
     *          after initialization completes.
     * 
     * @return true if registration successful, false if DroneCAN unavailable
     * 
     * @note Does NOT probe I2C/SPI buses - passive registration only
     * @note Actual sensor detection occurs when first CAN message arrives
     * @warning Must be called before attempting to read sensor data
     * 
     * @see subscribe_msgs() for message subscription details
     * @see handle_airspeed() for message reception callback
     */
    bool init(void) override;

    /**
     * @brief Get current differential pressure measurement from CAN sensor
     * 
     * @details Returns the most recently received differential pressure value from the
     *          DroneCAN airspeed sensor. The measurement is retrieved from cached data that
     *          was asynchronously received via handle_airspeed() callback when CAN messages
     *          arrived from the sensor node.
     *          
     *          Freshness validation:
     *          - Data is considered fresh if received within last 250ms
     *          - 250ms timeout chosen because pressure is flight-critical but CAN messages
     *            may have slight delays due to bus arbitration
     *          - Longer timeout than temperature (100ms) due to higher criticality
     *          
     *          Thread safety:
     *          - Acquires _sem_airspeed semaphore for atomic read of pressure and timestamp
     *          - Safe to call from main flight loop while CAN messages arrive asynchronously
     * 
     * @param[out] pressure Differential pressure in Pascals
     * 
     * @return true if pressure data is fresh (received within 250ms), false if stale or unavailable
     * 
     * @note Called at main loop rate by flight control algorithms
     * @warning Stale data (false return) indicates CAN node failure, bus congestion, or disconnection
     * @warning Flight code MUST check return value - using stale pressure is unsafe
     * 
     * @see handle_airspeed() for data update mechanism
     */
    bool get_differential_pressure(float &pressure) override;

    /**
     * @brief Get current temperature measurement from CAN sensor
     * 
     * @details Returns the most recently received temperature value from the DroneCAN
     *          airspeed sensor. Temperature data arrives in the same uavcan.equipment.air_data.RawAirData
     *          message as pressure but has a stricter freshness requirement since it's
     *          supplementary data expected to update faster.
     *          
     *          Freshness validation:
     *          - Data is considered fresh if received within last 100ms
     *          - Shorter timeout than pressure (250ms) because temperature is supplementary
     *          - Used for air density compensation and diagnostics
     *          
     *          Thread safety:
     *          - Acquires _sem_airspeed semaphore for atomic read
     *          - Safe concurrent access with CAN message reception
     * 
     * @param[out] temperature Air temperature in degrees Celsius
     * 
     * @return true if temperature data is fresh (received within 100ms), false if stale or unavailable
     * 
     * @note Temperature availability depends on sensor capabilities - not all CAN airspeed sensors provide it
     * @note Less critical than pressure - flight control can operate without temperature
     * 
     * @see handle_airspeed() for data update mechanism
     */
    bool get_temperature(float &temperature) override;

#if AP_AIRSPEED_HYGROMETER_ENABLE
    /**
     * @brief Get humidity and temperature data from DroneCAN hygrometer sensor
     * 
     * @details Returns hygrometer measurements received via dronecan.sensors.hygrometer.Hygrometer
     *          messages over CAN bus. This is an optional feature for advanced environmental
     *          sensing, typically used for weather monitoring or engine intake air analysis.
     *          
     *          Hygrometer data includes:
     *          - Temperature: Independent temperature sensor (may differ from airspeed sensor)
     *          - Humidity: Relative humidity percentage (0-100%)
     *          - Timestamp: Time of last valid sample for freshness checking
     *          
     *          Thread safety:
     *          - Acquires _sem_airspeed for atomic access to _hygrometer structure
     * 
     * @param[out] last_sample_ms Timestamp of last hygrometer sample in milliseconds
     * @param[out] temperature Temperature from hygrometer in degrees Celsius
     * @param[out] humidity Relative humidity in percentage (0-100%)
     * 
     * @return true if hygrometer data available, false if not supported or no data received
     * 
     * @note Optional feature - only compiled if AP_AIRSPEED_HYGROMETER_ENABLE defined
     * @note Requires compatible DroneCAN sensor with hygrometer support
     * 
     * @see handle_hygrometer() for message reception callback
     */
    bool get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity) override;
#endif

    /**
     * @brief Subscribe to DroneCAN airspeed and hygrometer messages
     * 
     * @details Registers static message handlers with the DroneCAN subsystem to receive
     *          uavcan.equipment.air_data.RawAirData and dronecan.sensors.hygrometer.Hygrometer
     *          messages. This is called once per DroneCAN interface during system initialization
     *          to set up message routing.
     *          
     *          Subscription setup:
     *          1. Register handle_airspeed() callback for RawAirData messages
     *          2. Register handle_hygrometer() callback for hygrometer messages (if enabled)
     *          3. Callbacks invoked asynchronously when matching messages arrive on CAN bus
     *          
     *          Static design rationale:
     *          - Single subscription serves all sensor instances
     *          - Reduces message routing overhead
     *          - Callbacks dispatch to appropriate backend instance based on node_id
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface to subscribe messages on
     * 
     * @return true if subscription successful, false on failure
     * 
     * @note Called during DroneCAN subsystem initialization, not per-sensor
     * @note Static method - operates on class level, not instance level
     * 
     * @see handle_airspeed() for message processing
     * @see handle_hygrometer() for hygrometer message processing
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    /**
     * @brief Probe for DroneCAN airspeed sensor and create backend instance
     * 
     * @details Attempts to create a DroneCAN airspeed backend instance for the specified
     *          sensor number. Unlike I2C/SPI backends that probe hardware buses, this creates
     *          a passive listener that waits for CAN messages from airspeed sensor nodes.
     *          
     *          Probing process:
     *          1. Create backend instance with frontend and instance number
     *          2. Call init() to register message handlers
     *          3. Store device ID for parameter persistence across reboots
     *          4. Return backend pointer for registration with frontend
     *          
     *          Device ID persistence:
     *          - previous_devid used to maintain parameter associations
     *          - Allows same sensor to retain calibration after reboot
     *          - Bus ID encodes CAN node ID and interface number
     *          
     *          Hot-plug support:
     *          - Backend created even if sensor not yet present on bus
     *          - Sensor auto-detected when first message arrives
     *          - Enables sensor connection after system boot
     * 
     * @param[in] _frontend Reference to AP_Airspeed frontend managing backends
     * @param[in] _instance Sensor instance number (0 to AIRSPEED_MAX_SENSORS-1)
     * @param[in] previous_devid Previously stored device ID for persistence, or 0 for new sensor
     * 
     * @return Pointer to created backend instance, or nullptr on failure
     * 
     * @note Static factory method - called by frontend during sensor detection
     * @note Does not guarantee sensor is physically present - waits for CAN messages
     * 
     * @see init() for backend initialization
     */
    static AP_Airspeed_Backend* probe(AP_Airspeed &_frontend, uint8_t _instance, uint32_t previous_devid);

private:

    /**
     * @brief DroneCAN message callback for airspeed sensor data
     * 
     * @details Static callback invoked by DroneCAN subsystem when uavcan.equipment.air_data.RawAirData
     *          messages arrive from airspeed sensor nodes on CAN bus. This is an asynchronous
     *          interrupt-driven callback that must execute quickly and safely.
     *          
     *          Message processing flow:
     *          1. Extract node_id from CAN transfer metadata
     *          2. Lookup appropriate backend instance for this node via get_dronecan_backend()
     *          3. Parse differential_pressure and static_air_temperature from message
     *          4. Acquire backend's _sem_airspeed semaphore for thread-safe access
     *          5. Update _pressure, _temperature, and _last_sample_time_ms
     *          6. Release semaphore and return
     *          
     *          Thread safety considerations:
     *          - Called from DroneCAN RX thread context (interrupt or high-priority task)
     *          - Must not block or perform lengthy operations
     *          - Semaphore protects concurrent access with get_differential_pressure()/get_temperature()
     *          - Registry lookup protected by _sem_registry
     *          
     *          Message format (uavcan.equipment.air_data.RawAirData):
     *          - differential_pressure: Pascals (float)
     *          - static_air_temperature: Kelvin (float) - converted to Celsius internally
     *          - static_pressure: Pascals (optional, not used for differential sensing)
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface that received the message
     * @param[in] transfer CAN message transfer metadata including source node_id and timestamp
     * @param[in] msg Parsed RawAirData message containing pressure and temperature
     * 
     * @note Static callback - dispatches to appropriate instance based on node_id
     * @note Called from DroneCAN thread context - must be fast and thread-safe
     * @warning Must not block or call functions that may sleep
     * 
     * @see get_dronecan_backend() for node-to-backend mapping
     * @see get_differential_pressure() for data retrieval
     */
    static void handle_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_RawAirData &msg);
    
    /**
     * @brief DroneCAN message callback for hygrometer sensor data
     * 
     * @details Static callback invoked when dronecan.sensors.hygrometer.Hygrometer messages
     *          arrive from environmental sensors on CAN bus. Processes humidity and temperature
     *          data for advanced weather sensing applications.
     *          
     *          Message processing:
     *          1. Extract node_id from transfer
     *          2. Lookup backend instance for node
     *          3. Parse temperature (Kelvin) and humidity (percentage) from message
     *          4. Acquire _sem_airspeed for thread-safe update
     *          5. Store in _hygrometer structure with timestamp
     *          
     *          Thread safety:
     *          - Called from DroneCAN RX thread
     *          - Semaphore ensures atomic updates
     *          - Must execute quickly to avoid blocking CAN reception
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface that received the message
     * @param[in] transfer CAN message transfer metadata including source node_id
     * @param[in] msg Parsed hygrometer message with temperature and humidity
     * 
     * @note Only compiled if AP_AIRSPEED_HYGROMETER_ENABLE defined
     * @note Static callback dispatched by node_id
     * 
     * @see get_hygrometer() for data retrieval
     */
    static void handle_hygrometer(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_hygrometer_Hygrometer &msg);

    /**
     * @brief Lookup backend instance for specific CAN node
     * 
     * @details Searches the static _detected_modules registry to find the AP_Airspeed_DroneCAN
     *          instance associated with a specific CAN node ID on a specific DroneCAN interface.
     *          This enables the static message callbacks to dispatch received data to the
     *          correct backend instance.
     *          
     *          Registry structure:
     *          - _detected_modules: Array of {ap_dronecan, node_id, driver} mappings
     *          - Populated during sensor detection and message reception
     *          - Supports multiple CAN interfaces and multiple sensors per interface
     *          
     *          Lookup algorithm:
     *          1. Acquire _sem_registry for thread-safe access
     *          2. Iterate through _detected_modules array
     *          3. Match both ap_dronecan interface pointer AND node_id
     *          4. Return driver pointer if found, nullptr if not registered
     *          
     *          Auto-registration:
     *          - First message from new node_id triggers registration
     *          - Enables hot-plug sensor support
     *          - Registry persists for lifetime of system
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface to search
     * @param[in] node_id CAN node ID to find (1-127 per UAVCAN spec)
     * 
     * @return Pointer to backend instance for this node, or nullptr if not found
     * 
     * @note Static method - operates on class-level registry
     * @note Thread-safe via _sem_registry protection
     * @note Called from message callbacks in DroneCAN thread context
     * 
     * @see _detected_modules for registry structure
     * @see handle_airspeed() for primary usage
     */
    static AP_Airspeed_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    /**
     * @brief Last received differential pressure value from CAN sensor
     * 
     * @details Cached pressure measurement in Pascals received via handle_airspeed() callback.
     *          Updated asynchronously when RawAirData messages arrive from CAN node.
     *          Protected by _sem_airspeed for thread-safe access between CAN RX thread
     *          and main flight loop.
     *          
     *          Units: Pascals (Pa)
     *          Typical range: -500 to +500 Pa for airspeed sensing
     *          
     * @note Access protected by _sem_airspeed semaphore
     * @note Freshness validated via _last_sample_time_ms in get_differential_pressure()
     */
    float _pressure; // Pascal
    
    /**
     * @brief Last received air temperature from CAN sensor
     * 
     * @details Cached temperature measurement in degrees Celsius received via handle_airspeed()
     *          callback. Arrives in same RawAirData message as pressure but stored separately
     *          for independent freshness validation.
     *          
     *          Units: Degrees Celsius (°C)
     *          Source: Converted from Kelvin in CAN message
     *          
     * @note Access protected by _sem_airspeed semaphore
     * @note Availability depends on sensor capabilities - check _have_temperature flag
     */
    float _temperature; // Celcius
    
    /**
     * @brief Timestamp of last CAN message reception
     * 
     * @details System millisecond timestamp when most recent RawAirData message was received
     *          and processed by handle_airspeed(). Used for freshness validation with different
     *          timeouts for pressure (250ms) and temperature (100ms).
     *          
     *          Units: Milliseconds since system boot
     *          
     *          Freshness checking:
     *          - Pressure considered fresh if (now - _last_sample_time_ms) < 250ms
     *          - Temperature considered fresh if (now - _last_sample_time_ms) < 100ms
     *          
     * @note Protected by _sem_airspeed semaphore
     * @note Shared timestamp for both pressure and temperature
     * 
     * @see get_differential_pressure() for 250ms pressure timeout
     * @see get_temperature() for 100ms temperature timeout
     */
    uint32_t _last_sample_time_ms;

    /**
     * @brief Hygrometer sensor data structure
     * 
     * @details Cached humidity and temperature measurements from optional hygrometer sensor.
     *          Receives data via handle_hygrometer() callback when dronecan.sensors.hygrometer.Hygrometer
     *          messages arrive. Separate from airspeed temperature measurement since hygrometer
     *          may use different sensor.
     *          
     *          Structure members:
     *          - temperature: Degrees Celsius from hygrometer sensor
     *          - humidity: Relative humidity percentage (0-100%)
     *          - last_sample_ms: Timestamp for freshness validation
     *          
     * @note Only compiled if AP_AIRSPEED_HYGROMETER_ENABLE defined
     * @note Access protected by _sem_airspeed semaphore
     * @note Temperature may differ from airspeed temperature measurement
     * 
     * @see get_hygrometer() for data retrieval
     * @see handle_hygrometer() for data updates
     */
    struct {
        float temperature;    ///< Temperature from hygrometer in degrees Celsius
        float humidity;       ///< Relative humidity percentage (0-100%)
        uint32_t last_sample_ms;  ///< Timestamp of last hygrometer sample in milliseconds
    } _hygrometer;

    /**
     * @brief Semaphore protecting per-instance sensor data
     * 
     * @details HAL semaphore ensuring thread-safe access to _pressure, _temperature,
     *          _last_sample_time_ms, and _hygrometer structure. Required because data
     *          is written by handle_airspeed() callback in DroneCAN RX thread context
     *          and read by get_differential_pressure()/get_temperature() in main flight loop.
     *          
     *          Protected data:
     *          - _pressure (Pascals)
     *          - _temperature (Celsius)
     *          - _last_sample_time_ms (milliseconds)
     *          - _hygrometer structure (if enabled)
     *          
     *          Lock hierarchy:
     *          - Acquired by message callbacks (write operations)
     *          - Acquired by getter methods (read operations)
     *          - Must not be held while calling blocking functions
     *          
     * @note Per-instance semaphore - each backend has its own
     * @note Critical for preventing race conditions between threads
     * 
     * @see handle_airspeed() for write-side locking
     * @see get_differential_pressure() for read-side locking
     */
    HAL_Semaphore _sem_airspeed;

    /**
     * @brief Static registry mapping CAN nodes to backend instances
     * 
     * @details Module Detection Registry enables routing of CAN messages to correct backend
     *          instances based on source node_id. Static array shared by all instances to
     *          support multiple sensors across multiple CAN interfaces.
     *          
     *          Registry structure:
     *          - ap_dronecan: Pointer to DroneCAN interface (CAN1, CAN2, etc.)
     *          - node_id: UAVCAN node ID (1-127) of sensor
     *          - driver: Pointer to AP_Airspeed_DroneCAN instance handling this node
     *          
     *          Array size: AIRSPEED_MAX_SENSORS (typically 2-4)
     *          Lifetime: Persistent for system lifetime after first message
     *          Access: Protected by _sem_registry for thread safety
     *          
     *          Registration flow:
     *          1. Sensor sends first CAN message
     *          2. handle_airspeed() receives message
     *          3. get_dronecan_backend() searches registry
     *          4. If not found, empty slot allocated and populated
     *          5. Future messages routed to registered driver
     *          
     * @note Static member - shared across all instances
     * @note Enables hot-plug support through dynamic registration
     * @note Protected by _sem_registry semaphore
     * 
     * @see get_dronecan_backend() for registry lookup
     * @see _sem_registry for access protection
     */
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;       ///< DroneCAN interface pointer (CAN1/CAN2)
        uint8_t node_id;                ///< UAVCAN node ID of sensor (1-127)
        AP_Airspeed_DroneCAN *driver;   ///< Backend instance handling this node
    } _detected_modules[AIRSPEED_MAX_SENSORS];

    /**
     * @brief Semaphore protecting static module registry
     * 
     * @details Static semaphore ensuring thread-safe access to _detected_modules registry.
     *          Required because registry is accessed from multiple contexts:
     *          - Message callbacks adding new registrations (DroneCAN RX thread)
     *          - Message callbacks looking up existing registrations (DroneCAN RX thread)
     *          - Potential cleanup operations
     *          
     *          Protected resource: _detected_modules array
     *          
     *          Lock scope: Must be held for entire registry search/modification operation
     *          to prevent race conditions during registration.
     * 
     * @note Static semaphore - single lock protects shared registry
     * @note Separate from _sem_airspeed to avoid lock ordering issues
     * 
     * @see _detected_modules for protected data structure
     * @see get_dronecan_backend() for typical usage
     */
    static HAL_Semaphore _sem_registry;
    
    /**
     * @brief Flag indicating if sensor provides temperature data
     * 
     * @details Boolean flag set to true when first valid temperature measurement received
     *          from CAN sensor. Not all DroneCAN airspeed sensors provide temperature -
     *          this flag prevents returning invalid data if temperature unavailable.
     *          
     *          Set conditions:
     *          - Temperature field present in RawAirData message
     *          - Temperature value valid (not NaN or out of range)
     *          - Updated in handle_airspeed() callback
     *          
     *          Used by get_temperature():
     *          - Returns false immediately if _have_temperature is false
     *          - Avoids unnecessary semaphore acquisition if no data available
     * 
     * @note Per-instance flag - different sensors may have different capabilities
     * @note Once set true, remains true for sensor lifetime
     * 
     * @see get_temperature() for usage
     */
    bool _have_temperature;
};


#endif  // AP_AIRSPEED_DRONECAN_ENABLED
