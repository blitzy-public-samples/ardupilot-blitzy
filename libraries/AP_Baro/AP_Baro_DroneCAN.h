/**
 * @file AP_Baro_DroneCAN.h
 * @brief DroneCAN/UAVCAN barometer backend driver
 * 
 * @details This file implements a barometer backend that receives atmospheric
 *          pressure and temperature data via the DroneCAN (UAVCAN) protocol.
 *          It subscribes to UAVCAN air_data messages from remote barometer
 *          sensors on the CAN bus, enabling distributed sensor architectures.
 *          
 *          UAVCAN Message Types:
 *          - uavcan.equipment.air_data.StaticPressure (pressure in Pascals)
 *          - uavcan.equipment.air_data.StaticTemperature (temperature in Celsius)
 *          
 *          The driver accumulates multiple samples from CAN messages and
 *          publishes averaged readings to the main barometer frontend.
 *          Supports multiple remote barometers via node ID tracking.
 * 
 * @note Allows remote barometers via CAN bus for distributed sensor architectures
 */

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#if AP_TEST_DRONECAN_DRIVERS
#include <SITL/SITL.h>
#endif

/**
 * @class AP_Baro_DroneCAN
 * @brief DroneCAN/UAVCAN barometer backend implementation
 * 
 * @details This backend receives barometric pressure and temperature data from
 *          remote sensors via the DroneCAN (UAVCAN) CAN bus protocol. It
 *          subscribes to UAVCAN equipment.air_data messages and accumulates
 *          samples for averaging.
 *          
 *          Architecture:
 *          - Static message subscription registers callbacks with AP_DroneCAN
 *          - Callbacks receive UAVCAN StaticPressure and StaticTemperature messages
 *          - Samples are accumulated in thread-safe buffers
 *          - update() processes accumulated samples and publishes to frontend
 *          - Module detection registry tracks multiple remote barometers by node ID
 *          
 *          Multi-Sensor Support:
 *          Each DroneCAN barometer is identified by its CAN node ID, allowing
 *          multiple remote barometers on the same CAN bus. The driver maintains
 *          a registry of detected modules and creates backend instances per sensor.
 *          
 *          Thread Safety:
 *          Uses semaphores to protect pressure/temperature accumulation from
 *          concurrent access by CAN receive callbacks and main update loop.
 *          
 *          Units:
 *          - Pressure: Pascals (Pa) per UAVCAN standard
 *          - Temperature: Celsius (°C) per UAVCAN standard
 * 
 * @note Enables distributed sensor architectures with barometers remote from autopilot
 */
class AP_Baro_DroneCAN : public AP_Baro_Backend {
public:
    /**
     * @brief Construct a DroneCAN barometer backend
     * 
     * @param[in] baro Reference to the main AP_Baro frontend
     */
    AP_Baro_DroneCAN(AP_Baro &baro);

    /**
     * @brief Update barometer readings and publish to frontend
     * 
     * @details Processes accumulated pressure and temperature samples received
     *          from DroneCAN messages. Takes semaphore, averages accumulated
     *          samples, and publishes the averaged readings to the AP_Baro
     *          frontend for sensor fusion.
     *          
     *          This method is called periodically by the scheduler and retrieves
     *          samples that were accumulated by the CAN receive callbacks
     *          (handle_pressure and handle_temperature).
     *          
     *          Sample Accumulation:
     *          Multiple CAN messages arriving between update() calls are
     *          accumulated and averaged to reduce measurement noise and
     *          handle varying CAN bus message rates.
     * 
     * @note Called at main loop rate (typically 50-400Hz depending on vehicle)
     */
    void update() override;

    /**
     * @brief Subscribe to DroneCAN barometer messages
     * 
     * @details Registers callbacks with the AP_DroneCAN manager to receive
     *          UAVCAN air_data messages. Subscribes to:
     *          - uavcan.equipment.air_data.StaticPressure
     *          - uavcan.equipment.air_data.StaticTemperature
     *          
     *          The callbacks (handle_pressure and handle_temperature) will be
     *          invoked when matching messages arrive on the CAN bus.
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance managing the CAN interface
     * 
     * @return true if subscription successful, false otherwise
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    
    /**
     * @brief Get or create a DroneCAN backend for a specific node
     * 
     * @details Searches the module detection registry for an existing backend
     *          instance associated with the specified DroneCAN interface and
     *          node ID. Creates a new backend instance if requested and not found.
     *          
     *          Multi-Sensor Support:
     *          This allows multiple remote barometers on the CAN bus, each
     *          identified by its unique node ID, to have separate backend instances.
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance
     * @param[in] node_id     UAVCAN node ID of the remote barometer (0-127)
     * @param[in] create_new  If true, create new backend if not found
     * 
     * @return Pointer to AP_Baro_DroneCAN backend, or nullptr if not found and !create_new
     */
    static AP_Baro_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, bool create_new);
    
    /**
     * @brief Probe and initialize DroneCAN barometer backend
     * 
     * @details Registers this backend with all available DroneCAN interfaces
     *          and subscribes to barometer messages. Called during sensor
     *          initialization to set up DroneCAN barometer support.
     *          
     *          This is the entry point for automatic detection of DroneCAN
     *          barometers on the system.
     * 
     * @param[in] baro Reference to the main AP_Baro frontend
     * 
     * @return Pointer to backend if successful, nullptr if initialization failed
     */
    static AP_Baro_Backend* probe(AP_Baro &baro);

    /**
     * @brief Handle incoming UAVCAN StaticPressure message
     * 
     * @details Callback invoked by AP_DroneCAN when a StaticPressure message
     *          is received on the CAN bus. Accumulates pressure samples in a
     *          thread-safe buffer for later averaging by update().
     *          
     *          UAVCAN Message Format:
     *          - uavcan.equipment.air_data.StaticPressure
     *          - Units: Pascals (Pa)
     *          - Contains: static_pressure (float)
     *          
     *          The node ID from the transfer identifies which remote barometer
     *          sent the message, enabling multi-sensor support.
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance that received the message
     * @param[in] transfer    CAN transfer metadata including source node ID
     * @param[in] msg         Decoded StaticPressure message with pressure in Pascals
     */
    static void handle_pressure(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticPressure &msg);
    
    /**
     * @brief Handle incoming UAVCAN StaticTemperature message
     * 
     * @details Callback invoked by AP_DroneCAN when a StaticTemperature message
     *          is received on the CAN bus. Accumulates temperature samples in a
     *          thread-safe buffer for later averaging by update().
     *          
     *          UAVCAN Message Format:
     *          - uavcan.equipment.air_data.StaticTemperature
     *          - Units: Celsius (°C) per UAVCAN standard (Kelvin in protocol - 273.15)
     *          - Contains: static_temperature (float)
     *          
     *          The node ID from the transfer identifies which remote barometer
     *          sent the message, enabling multi-sensor support.
     * 
     * @param[in] ap_dronecan Pointer to AP_DroneCAN instance that received the message
     * @param[in] transfer    CAN transfer metadata including source node ID
     * @param[in] msg         Decoded StaticTemperature message with temperature
     */
    static void handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticTemperature &msg);
    
#if AP_TEST_DRONECAN_DRIVERS
    /**
     * @brief Update sensor healthy flag for SITL testing
     * 
     * @details Override for software-in-the-loop (SITL) simulation that allows
     *          disabling individual barometer instances for failure mode testing.
     * 
     * @param[in] instance Barometer instance index to update
     */
    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = !AP::sitl()->baro[instance].disable; };
#endif

private:

    /**
     * @brief Update accumulator with new value and wrap if full
     * 
     * @details Helper function for accumulating samples from CAN messages.
     *          Adds the new value to the accumulator and increments the count.
     *          If count reaches max_count, wraps back to averaging mode by
     *          using a weighted average instead of simple accumulation.
     * 
     * @param[in,out] accum     Pointer to accumulator value
     * @param[in]     val       New value to add to accumulator
     * @param[in,out] count     Pointer to sample count
     * @param[in]     max_count Maximum samples before wrapping
     */
    static void _update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count);

    // Frontend barometer instance index (identifies this backend in AP_Baro sensor array)
    uint8_t _instance;

    // Sample accumulation buffers (protected by _sem_baro)
    // These accumulate multiple CAN messages between update() calls for averaging
    bool new_pressure;              ///< Flag indicating new pressure data available
    float _pressure;                ///< Accumulated pressure in Pascals
    float _temperature;             ///< Accumulated temperature in Celsius
    uint8_t  _pressure_count;       ///< Number of pressure samples accumulated
    HAL_Semaphore _sem_baro;        ///< Semaphore protecting pressure/temperature buffers

    // DroneCAN interface and node identification
    AP_DroneCAN* _ap_dronecan;      ///< Pointer to DroneCAN interface managing CAN bus
    uint8_t _node_id;               ///< UAVCAN node ID of remote barometer (0-127)

    /**
     * @brief Module Detection Registry
     * 
     * @details Static registry tracking all detected DroneCAN barometers across
     *          all CAN interfaces. Each entry maps a (DroneCAN interface, node ID)
     *          pair to its backend driver instance.
     *          
     *          Multi-Sensor Support:
     *          Allows multiple remote barometers on different CAN buses or with
     *          different node IDs to coexist. The registry prevents duplicate
     *          backend creation for the same physical sensor.
     *          
     *          Protected by _sem_registry semaphore for thread-safe access.
     */
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;   ///< DroneCAN interface pointer
        uint8_t node_id;            ///< UAVCAN node ID of the remote barometer
        AP_Baro_DroneCAN* driver;   ///< Backend instance for this barometer
    } _detected_modules[BARO_MAX_DRIVERS];

    static HAL_Semaphore _sem_registry;  ///< Semaphore protecting _detected_modules registry
};

#endif  // AP_BARO_DRONECAN_ENABLED
