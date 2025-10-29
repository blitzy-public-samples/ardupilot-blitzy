/**
 * @file AP_RangeFinder_NRA24_CAN.h
 * @brief NRA24 24GHz FMCW radar rangefinder backend driver over CAN bus
 * 
 * @details This file implements the ArduPilot driver for NRA24 radar-based
 *          rangefinders that communicate via CAN bus protocol. The NRA24 uses
 *          24GHz FMCW (Frequency Modulated Continuous Wave) radar technology
 *          for distance measurement, providing reliable range detection in
 *          various environmental conditions including fog, dust, and darkness.
 *          
 *          The driver handles CAN frame reception, distance data extraction,
 *          and sensor health monitoring through heartbeat messages.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once
#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

/**
 * @class AP_RangeFinder_NRA24_CAN
 * @brief NRA24 24GHz FMCW radar rangefinder driver using CAN protocol
 * 
 * @details This class implements a rangefinder backend for NRA24 radar sensors
 *          that communicate via CAN bus. The NRA24 uses 24GHz FMCW radar technology
 *          which provides advantages over optical and ultrasonic sensors:
 *          - Works in fog, dust, and complete darkness
 *          - Immune to ambient light interference
 *          - Consistent performance across temperature ranges
 *          - Narrow beam pattern for precise targeting
 *          
 *          The sensor transmits distance measurements and status information via
 *          CAN frames conforming to the RadarCAN protocol. The driver monitors
 *          sensor health through periodic heartbeat messages and extracts radar
 *          ID from CAN frame identifiers for multi-sensor configurations.
 *          
 *          Typical applications include altitude measurement for precision landing,
 *          obstacle detection, and terrain following in challenging visibility.
 * 
 * @note This driver requires CAN bus hardware support and AP_CAN::Protocol::RadarCAN
 * @warning Radar beam characteristics and mounting orientation affect measurement accuracy
 */
class AP_RangeFinder_NRA24_CAN : public AP_RangeFinder_Backend_CAN {
public:
    /**
     * @brief Construct NRA24 CAN rangefinder backend
     * 
     * @details Initializes the NRA24 radar rangefinder backend by registering
     *          with the CAN manager using the RadarCAN protocol. The constructor
     *          sets up the base CAN backend with the sensor name "nra24" for
     *          identification in logs and parameter namespace.
     * 
     * @param[in,out] _state      Reference to rangefinder state structure for storing
     *                            distance measurements and sensor status
     * @param[in]     _params     Reference to rangefinder parameters including mounting
     *                            position, orientation, and filtering configuration
     * 
     * @note The RadarCAN protocol is used for all NRA24 CAN communication
     * @note Actual CAN frame reception begins when handle_frame() is called by CAN manager
     */
    AP_RangeFinder_NRA24_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::RadarCAN, "nra24")
    {
    }

    /**
     * @brief Periodic update of rangefinder state and health monitoring
     * 
     * @details Called by the rangefinder scheduler to update sensor state and
     *          perform health monitoring. This method checks for communication
     *          timeouts by comparing current time against last_heartbeat_ms and
     *          updates the sensor health status accordingly.
     *          
     *          If no heartbeat/data has been received within the timeout period,
     *          the sensor is marked as unhealthy and distance readings are
     *          invalidated to prevent stale data usage.
     * 
     * @note Called at the rangefinder update rate (typically 20Hz)
     * @note Override of AP_RangeFinder_Backend::update()
     * 
     * @see handle_frame() for CAN message reception
     * @see last_heartbeat_ms for heartbeat tracking
     */
    void update(void) override;

    /**
     * @brief Handle incoming CAN frames from NRA24 radar sensor
     * 
     * @details Processes CAN frames received from the NRA24 radar rangefinder.
     *          This method decodes the CAN frame to extract distance measurements,
     *          updates the sensor state with new readings, and tracks the last
     *          heartbeat timestamp for health monitoring.
     *          
     *          The NRA24 CAN protocol encodes radar ID in the CAN identifier field,
     *          which is extracted using get_radar_id() to support multiple radar
     *          sensors on the same CAN bus. Distance data is extracted from the
     *          frame payload according to the RadarCAN protocol specification.
     *          
     *          Valid frames update both the distance measurement and the
     *          last_heartbeat_ms timestamp to indicate sensor is actively transmitting.
     * 
     * @param[in,out] frame  Reference to CAN frame structure containing message ID
     *                       and data payload from NRA24 sensor
     * 
     * @return true if frame was successfully processed and contained valid data,
     *         false if frame was invalid, corrupted, or not from this sensor
     * 
     * @note Called by CAN driver interrupt context or CAN manager polling loop
     * @note Frame ID encodes radar sensor identifier in bits [7:4]
     * @note Override of AP_RangeFinder_Backend_CAN::handle_frame()
     * 
     * @warning This may be called from interrupt context - keep processing minimal
     * 
     * @see get_radar_id() for radar ID extraction from CAN frame identifier
     * @see update() for timeout and health monitoring
     */
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    /**
     * @brief Parameter table for NRA24 rangefinder configuration
     * 
     * @details Defines the AP_Param parameter table for NRA24-specific configuration
     *          parameters. This allows users to configure NRA24 radar settings through
     *          ground control stations and parameter files.
     *          
     *          The parameter group is registered with the rangefinder parameter system
     *          and uses the "nra24" prefix for parameter naming.
     * 
     * @note See implementation file for actual parameter definitions
     * @see AP_Param::GroupInfo for parameter system documentation
     */
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Extract radar ID from CAN frame identifier
     * 
     * @details Parses the CAN frame identifier to extract the radar sensor ID,
     *          which is encoded in bits [7:4] of the frame ID. This allows multiple
     *          NRA24 radar sensors to coexist on the same CAN bus, each with a
     *          unique identifier for message routing and sensor discrimination.
     *          
     *          The extraction uses bit masking (0xF0) to isolate bits [7:4],
     *          followed by a right shift of 4 bits to normalize the ID value
     *          to the range 0-15.
     * 
     * @param[in] id  CAN frame identifier (11-bit or 29-bit) containing encoded radar ID
     * 
     * @return Radar sensor ID extracted from frame identifier (0-15)
     * 
     * @note Radar ID is stored in bits [7:4] of the CAN identifier
     * @note Supports up to 16 unique radar sensors (ID 0-15) on single CAN bus
     * 
     * @see handle_frame() for CAN frame processing using this radar ID
     */
    uint32_t get_radar_id(uint32_t id) const { return ((id & 0xF0U) >> 4U); }
    
    /**
     * @brief Timestamp of last received message from sensor
     * 
     * @details Stores the timestamp (in milliseconds since boot) when the last
     *          valid CAN frame was received from the NRA24 radar sensor. This is
     *          used by the update() method to detect communication timeouts and
     *          mark the sensor as unhealthy if no messages are received within
     *          the expected heartbeat interval.
     *          
     *          Updated by handle_frame() on each successful frame reception.
     *          Compared against current time in update() for timeout detection.
     * 
     * @note Units: milliseconds since system boot (AP_HAL::millis())
     * @note Typical heartbeat interval is sensor-dependent (usually 10-100ms)
     * 
     * @see handle_frame() updates this timestamp on message reception
     * @see update() uses this for communication timeout detection
     */
    uint32_t last_heartbeat_ms;
};

#endif  // AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
