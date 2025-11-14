/**
 * @file AP_RangeFinder_TOFSenseP_CAN.h
 * @brief TOFSense-P CAN time-of-flight rangefinder backend driver
 * 
 * This file implements the rangefinder backend driver for TOFSense-P
 * time-of-flight distance sensors that communicate over CAN bus.
 * TOFSense-P provides high-precision distance measurements using
 * laser time-of-flight technology with CAN bus interface.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once
#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED

#include "AP_RangeFinder_Backend_CAN.h"

/**
 * @class AP_RangeFinder_TOFSenseP_CAN
 * @brief TOFSense-P CAN rangefinder backend driver
 * 
 * @details This class implements the rangefinder backend for TOFSense-P
 *          time-of-flight (ToF) distance sensors using CAN bus communication.
 *          
 *          TOFSense-P is a high-precision laser rangefinder that uses
 *          time-of-flight measurement technology to determine distance to
 *          objects. The sensor communicates via CAN bus using a custom
 *          protocol (AP_CAN::Protocol::TOFSenseP).
 *          
 *          Key Features:
 *          - Time-of-flight laser distance measurement
 *          - CAN bus interface for reliable communication
 *          - High precision distance readings
 *          - Integration with ArduPilot CAN manager
 *          
 *          The driver inherits from AP_RangeFinder_Backend_CAN which provides
 *          the CAN bus communication infrastructure and integration with the
 *          ArduPilot CAN driver framework.
 *          
 *          Communication Flow:
 *          1. Sensor sends distance measurements via CAN frames
 *          2. CAN manager routes frames to this driver via handle_frame()
 *          3. Driver decodes distance data and updates rangefinder state
 *          4. Main rangefinder library provides distance to flight controller
 *          
 * @note This driver requires CAN bus to be properly configured and the
 *       TOFSense-P sensor to be connected to a supported CAN interface.
 * 
 * @see AP_RangeFinder_Backend_CAN for base CAN rangefinder functionality
 * @see AP_CANManager for CAN bus management
 */
class AP_RangeFinder_TOFSenseP_CAN : public AP_RangeFinder_Backend_CAN {
public:
    /**
     * @brief Constructor for TOFSense-P CAN rangefinder backend
     * 
     * @details Initializes the TOFSense-P CAN rangefinder driver by registering
     *          it with the CAN bus manager using the TOFSenseP protocol identifier.
     *          The constructor chains to the base AP_RangeFinder_Backend_CAN
     *          constructor which handles CAN interface initialization and protocol
     *          registration.
     *          
     *          The driver name "tofsensep" is used for logging and parameter
     *          identification within the ArduPilot system.
     * 
     * @param[in,out] _state Reference to rangefinder state structure that stores
     *                       current distance readings, status, and health information.
     *                       This state is shared with the main rangefinder library
     *                       and updated by this backend when new measurements arrive.
     * @param[in,out] _params Reference to rangefinder parameter structure containing
     *                        configuration values such as orientation, offset,
     *                        minimum/maximum range, and scaling factors.
     * 
     * @note This constructor is typically called by the AP_RangeFinder library
     *       during sensor initialization when a TOFSense-P CAN sensor is detected
     *       or configured.
     * 
     * @see AP_RangeFinder_Backend_CAN::AP_RangeFinder_Backend_CAN()
     */
    AP_RangeFinder_TOFSenseP_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::TOFSenseP, "tofsensep")
    {
    }

    /**
     * @brief Process incoming CAN frame from TOFSense-P sensor
     * 
     * @details This method is called by the CAN manager whenever a CAN frame
     *          arrives that matches the TOFSenseP protocol. The handler is
     *          responsible for:
     *          - Validating the frame format and data integrity
     *          - Extracting distance measurement from the CAN frame payload
     *          - Converting raw sensor data to standard distance units (meters)
     *          - Updating the rangefinder state with new measurements
     *          - Updating sensor health and status indicators
     *          
     *          The TOFSense-P CAN protocol defines a specific frame format
     *          for transmitting distance measurements. This method decodes
     *          that protocol and integrates the measurements into the
     *          ArduPilot rangefinder system.
     *          
     *          Thread Safety: This method is called from the CAN manager thread
     *          context. State updates must be thread-safe with respect to the
     *          main sensor read operations.
     * 
     * @param[in,out] frame Reference to CAN frame containing sensor data.
     *                      The frame contains the CAN ID, data length code (DLC),
     *                      and up to 8 bytes of payload data from the TOFSense-P
     *                      sensor. Frame structure defined by TOFSense-P protocol.
     * 
     * @return true if frame was successfully processed and distance measurement
     *              was valid and within acceptable range
     * @return false if frame was invalid, malformed, or distance measurement
     *               was out of range or invalid
     * 
     * @note This method is called at the rate that CAN frames arrive from the
     *       sensor, which may be different from the main loop rate.
     * @note Invalid frames are logged for debugging but do not affect the
     *       last valid distance reading.
     * 
     * @see AP_RangeFinder_Backend_CAN::handle_frame()
     * @see AP_HAL::CANFrame for CAN frame structure
     */
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    /**
     * @brief Parameter group information for TOFSense-P rangefinder
     * 
     * @details Static parameter table defining configurable parameters specific
     *          to the TOFSense-P CAN rangefinder. This integrates with the
     *          ArduPilot parameter system (AP_Param) to provide persistent
     *          configuration storage and ground station parameter access.
     *          
     *          Parameters in this group may include sensor-specific configuration
     *          such as filtering options, update rates, or protocol-specific
     *          settings that are unique to TOFSense-P sensors.
     *          
     *          The parameter group is registered with the main rangefinder
     *          parameter table and appears in ground control station software
     *          with an appropriate prefix (e.g., RNGFNDx_).
     * 
     * @note Parameter definitions must remain stable across firmware versions
     *       to maintain parameter compatibility with saved configurations.
     * 
     * @see AP_Param::GroupInfo for parameter definition structure
     * @see AP_RangeFinder_Params for base rangefinder parameters
     */
    static const struct AP_Param::GroupInfo var_info[];
};

#endif  // AP_RANGEFINDER_USD1_CAN_ENABLED
