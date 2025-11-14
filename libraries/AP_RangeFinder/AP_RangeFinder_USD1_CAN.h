/**
 * @file AP_RangeFinder_USD1_CAN.h
 * @brief USD1 CAN protocol ultrasonic rangefinder backend driver
 * 
 * @details This file implements the driver for ultrasonic distance sensors
 *          using the USD1 CAN protocol. USD1 is a CAN-based protocol for
 *          communicating with ultrasonic rangefinders that transmit distance
 *          measurements over CAN bus.
 *          
 *          The driver receives distance measurements via CAN frames and
 *          processes them into the standard RangeFinder interface for use
 *          by ArduPilot's navigation and control systems.
 * 
 * @note This driver requires CAN hardware support and must be enabled via
 *       AP_RANGEFINDER_USD1_CAN_ENABLED feature flag
 * 
 * @see AP_RangeFinder_Backend_CAN for the base CAN rangefinder implementation
 * @see AP_RangeFinder for the main rangefinder interface
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_USD1_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

/**
 * @class AP_RangeFinder_USD1_CAN
 * @brief USD1 CAN protocol ultrasonic rangefinder driver implementation
 * 
 * @details This class implements a rangefinder backend for ultrasonic distance
 *          sensors that communicate using the USD1 CAN protocol. The driver
 *          inherits from AP_RangeFinder_Backend_CAN which provides the CAN
 *          interface handling and message routing.
 *          
 *          Key features:
 *          - Processes distance measurements from USD1 CAN frames
 *          - Integrates with ArduPilot's CAN bus manager
 *          - Provides standard RangeFinder interface to navigation systems
 *          - Supports multiple USD1 sensors on the same CAN bus
 *          
 *          Lifecycle:
 *          1. Construction: Initialize with RangeFinder state and parameters
 *          2. Registration: Registered with CAN manager for USD1 protocol
 *          3. Operation: handle_frame() called for each incoming USD1 CAN frame
 *          4. Processing: Distance data extracted and published to RangeFinder state
 *          
 *          Thread Safety:
 *          - handle_frame() called from CAN bus interrupt/thread context
 *          - State updates protected by RangeFinder semaphore
 *          
 *          Hardware Requirements:
 *          - CAN bus hardware support (ChibiOS HAL or Linux socketcan)
 *          - USD1-compatible ultrasonic rangefinder connected to CAN bus
 *          - Proper CAN termination and bus configuration
 * 
 * @note USD1 sensors typically operate at 10-50Hz update rate
 * @warning Ensure CAN bus termination is correct to avoid communication errors
 * 
 * @see AP_RangeFinder_Backend_CAN
 * @see AP_CANManager
 */
class AP_RangeFinder_USD1_CAN : public AP_RangeFinder_Backend_CAN {
public:
    /**
     * @brief Construct a new USD1 CAN rangefinder backend
     * 
     * @details Initializes the USD1 CAN rangefinder driver by registering it
     *          with the CAN bus manager for the USD1 protocol. The constructor
     *          passes the protocol identifier (AP_CAN::Protocol::USD1) and
     *          driver name ("usd1") to the base class which handles CAN
     *          frame routing.
     *          
     *          The driver will automatically start receiving CAN frames that
     *          match the USD1 protocol after registration with the CAN manager.
     * 
     * @param[in,out] _state Reference to RangeFinder state structure for storing
     *                       distance measurements, status, and metadata. This state
     *                       is shared with the main RangeFinder library and updated
     *                       by handle_frame() when new measurements arrive.
     * 
     * @param[in,out] _params Reference to rangefinder parameters (configuration)
     *                        including sensor orientation, offset, scaling, etc.
     * 
     * @note Constructor is called during sensor initialization at boot time
     * @note The backend is automatically registered with AP_CANManager for USD1 frames
     * 
     * @see AP_RangeFinder_Backend_CAN::AP_RangeFinder_Backend_CAN()
     * @see AP_CANManager
     */
    AP_RangeFinder_USD1_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::USD1, "usd1")
    {
    }

    /**
     * @brief Process incoming USD1 CAN frames containing distance measurements
     * 
     * @details This method is called by the CAN bus manager when a CAN frame
     *          matching the USD1 protocol is received. The function extracts
     *          the distance measurement from the CAN frame payload, validates
     *          the data, and updates the rangefinder state.
     *          
     *          USD1 CAN Frame Format:
     *          - Standard CAN frame (11-bit or 29-bit identifier)
     *          - Payload contains distance measurement in sensor-specific format
     *          - May include additional metadata (signal quality, temperature)
     *          
     *          Processing Steps:
     *          1. Validate frame format and length
     *          2. Extract distance measurement from payload
     *          3. Apply scaling and offset corrections
     *          4. Validate distance is within sensor range
     *          5. Update RangeFinder state with new measurement
     *          6. Update timestamp and status flags
     *          
     *          Thread Safety:
     *          This method is called from CAN bus interrupt or thread context.
     *          State updates must be protected by RangeFinder semaphore.
     * 
     * @param[in] frame Reference to CAN frame containing USD1 sensor data.
     *                  Frame identifier and data payload contain the distance
     *                  measurement and metadata from the ultrasonic sensor.
     * 
     * @return true if frame was successfully parsed and distance updated,
     *         false if frame format invalid or data out of range
     * 
     * @note Called at sensor update rate (typically 10-50Hz)
     * @note This method runs in CAN thread context, keep processing minimal
     * 
     * @warning Do not perform blocking operations or lengthy calculations
     *          in this method as it runs in real-time CAN processing context
     * 
     * @see AP_RangeFinder_Backend_CAN::handle_frame()
     * @see AP_HAL::CANFrame
     */
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    /**
     * @brief Parameter table for USD1 CAN rangefinder configuration
     * 
     * @details This static array defines the parameter group for USD1-specific
     *          configuration options. Parameters are exposed through the AP_Param
     *          system and can be configured via ground control station or
     *          MAVLink parameter protocol.
     *          
     *          The parameter group may include:
     *          - CAN bus configuration (node ID, filtering)
     *          - Sensor-specific settings (update rate, filtering)
     *          - Protocol-specific options
     *          
     *          Parameters are stored in non-volatile memory and persist across
     *          reboots. Changes take effect immediately or after reinitialization
     *          depending on the specific parameter.
     * 
     * @note This table is defined in the corresponding .cpp file
     * @note Parameters follow ArduPilot naming convention: RNGFND<instance>_<PARAM>
     * 
     * @see AP_Param::GroupInfo
     * @see AP_RangeFinder_Params for common rangefinder parameters
     */
    static const struct AP_Param::GroupInfo var_info[];
};

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
