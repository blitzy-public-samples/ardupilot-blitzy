/**
 * @file AP_Frsky_SPort_Passthrough.h
 * @brief FrSky SPort Passthrough telemetry protocol implementation
 * 
 * @details This file implements the FrSky SPort Passthrough protocol, an advanced
 *          telemetry system designed for integration with OpenTX and EdgeTX radio firmware.
 *          
 *          Key features:
 *          - Weighted Fair Queueing (WFQ) scheduler for efficient telemetry transmission
 *          - Rich telemetry data: attitude, GPS, battery, home distance, terrain, wind, waypoints
 *          - Status text message transmission with chunking and repetition
 *          - Optional bidirectional support via MAVlite protocol (parameter read/write, mode changes, commands)
 *          - External data mode for FPort integration (RC and telemetry over single wire)
 *          - Conditional compilation via AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
 *          
 *          The Passthrough protocol uses compact binary encoding with bit-packed fields
 *          transmitted as FrSky sensor packets (0x5000-0x500D sensor IDs). Data is
 *          interpreted by OpenTX/EdgeTX telemetry scripts on the transmitter.
 * 
 * @see AP_RCTelemetry for WFQ scheduler interface
 * @see AP_Frsky_MAVlite.h for MAVlite protocol details
 * @see OpenTX/EdgeTX telemetry scripts for data format interpretation
 */

#pragma once

#include "AP_Frsky_SPort.h"

#if AP_FRSKY_SPORT_PASSTHROUGH_ENABLED

#include <AP_RCTelemetry/AP_RCTelemetry.h>

#include "AP_Frsky_SPortParser.h"
#include "AP_Frsky_MAVlite.h"

#include "AP_Frsky_Telem.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#include "AP_Frsky_MAVlite_SPortToMAVlite.h"
#include "AP_Frsky_MAVlite_MAVliteToSPort.h"
#include "AP_Frsky_MAVliteMsgHandler.h"

#define SPORT_TX_PACKET_DUPLICATES          1   // number of duplicates packets we send (fport only)
#endif

/**
 * @class AP_Frsky_SPort_Passthrough
 * @brief FrSky SPort Passthrough telemetry backend for OpenTX/EdgeTX integration
 * 
 * @details This class extends AP_Frsky_SPort and implements AP_RCTelemetry to provide
 *          the Passthrough telemetry protocol for OpenTX and EdgeTX radio firmware.
 *          
 *          Architecture:
 *          - Inherits SPort transport layer from AP_Frsky_SPort
 *          - Implements Weighted Fair Queueing (WFQ) scheduler via AP_RCTelemetry
 *          - Provides rich telemetry: attitude, GPS, battery, home distance, terrain, wind, waypoints
 *          - Transmits status text messages in 4-byte chunks with 3x repetition for reliability
 *          - Optional bidirectional support via MAVlite protocol for commands and parameters
 *          
 *          Telemetry Data Provided:
 *          - Attitude: Roll, pitch, yaw, and rangefinder distance (0x5006)
 *          - GPS: Latitude, longitude, altitude, velocity (0x800, 0x5005)
 *          - Battery: Voltage, current, capacity for up to 2 batteries (0x5008)
 *          - Navigation: Home distance/direction, waypoint info (0x5004, 0x500D)
 *          - Environment: Terrain altitude, wind speed/direction (0x500B, 0x500C)
 *          - Status: Autopilot status, GPS fix, mode, arming state (0x5001, 0x5002)
 *          - Parameters: Frame type, battery failsafe, features (0x5007)
 *          - Sensors: RPM sensors (0x500A)
 *          - Messages: Status text in chunks (0x5000)
 *          
 *          Bidirectional Features (conditional on HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL):
 *          - Parameter read and write operations
 *          - Flight mode changes
 *          - Autopilot commands
 *          - Implemented via MAVlite protocol over SPort uplink
 *          
 *          External Data Mode:
 *          - When use_external_data is true, operates in FPort mode
 *          - Telemetry exported for external transport (RC + telemetry on single wire)
 *          - Bidirectional telemetry can be injected from external source
 *          
 *          WFQ Scheduler:
 *          - Prioritizes fresh data (GPS position, attitude) over static data (parameters)
 *          - Adjusts packet weights dynamically based on data availability
 *          - Ensures critical telemetry is transmitted with appropriate frequency
 *          
 *          Packet Encoding:
 *          - Uses compact binary encoding with bit-packed fields
 *          - Sensor IDs in range 0x5000-0x500D identify data type
 *          - Distance in meters, angles in degrees/centidegrees
 *          - Timing in milliseconds
 * 
 * @note Sets singleton instance for AP::frsky_passthrough_telem() accessor
 * @note WFQ scheduling prioritizes fresh data over static data
 * @warning Bidirectional commands rejected when armed unless specifically allowed
 * @warning SPORT_PACKET_QUEUE_LENGTH must accommodate maximum message fragmentation
 * 
 * @see AP_RCTelemetry for WFQ scheduler interface
 * @see AP_Frsky_MAVlite.h for MAVlite protocol details
 * @see OpenTX/EdgeTX telemetry scripts for data format interpretation
 */
class AP_Frsky_SPort_Passthrough : public AP_Frsky_SPort, public AP_RCTelemetry
{
public:

    /**
     * @brief Initialize Passthrough telemetry backend
     * 
     * @param[in] port UART driver for SPort communication
     * @param[in] use_external_data True for FPort mode (external transport), false for direct SPort
     * @param[in,out] frsky_parameters Reference to FrSky parameter object for configuration
     * 
     * @details Constructs the Passthrough backend and sets the singleton instance.
     *          In FPort mode (use_external_data=true), telemetry is buffered for
     *          external transport rather than transmitted directly.
     * 
     * @note Sets singleton for AP::frsky_passthrough_telem() accessor
     */
    AP_Frsky_SPort_Passthrough(AP_HAL::UARTDriver *port, bool use_external_data, AP_Frsky_Parameters *&frsky_parameters) :
        AP_Frsky_SPort(port),
        AP_RCTelemetry(WFQ_LAST_ITEM),
        _frsky_parameters(frsky_parameters),
        _use_external_data(use_external_data)
    {
        singleton = this;
    }

    /**
     * @brief Get singleton instance of Passthrough telemetry
     * 
     * @return Pointer to singleton instance, or nullptr if not initialized
     * 
     * @note Used by AP::frsky_passthrough_telem() accessor
     */
    static AP_Frsky_SPort_Passthrough *get_singleton(void) {
        return singleton;
    }

    /**
     * @brief Initialize Passthrough telemetry with WFQ scheduler
     * 
     * @return true if initialization successful, false otherwise
     * 
     * @details Initializes the Weighted Fair Queueing scheduler for telemetry
     *          transmission. Sets up packet types, weights, and scheduling parameters.
     *          Must be called before telemetry transmission begins.
     */
    bool init() override;
    
    /**
     * @brief Configure UART and spawn telemetry thread
     * 
     * @return true if serial port configured successfully, false otherwise
     * 
     * @details Configures the UART port for SPort communication (57600 baud, inverted)
     *          and spawns the telemetry processing thread. In bidirectional mode,
     *          configures sensor IDs and initializes MAVlite handlers.
     */
    bool init_serial_port() override;

    // WFQ scheduler interface methods (from AP_RCTelemetry)
    
    /**
     * @brief Check if telemetry packet is ready for transmission
     * 
     * @param[in] idx Packet type index (PassthroughPacketType enum)
     * @param[in] queue_empty True if WFQ queue is empty
     * 
     * @return true if packet should be sent, false to skip
     * 
     * @details Determines if a packet of the specified type should be transmitted
     *          based on data availability and freshness. For example, GPS packets
     *          are only sent when GPS has a valid fix.
     */
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    
    /**
     * @brief Generate and transmit telemetry packet
     * 
     * @param[in] idx Packet type index (PassthroughPacketType enum)
     * 
     * @details Generates the telemetry packet for the specified type by calling
     *          the appropriate calc_* helper method and transmits it via SPort.
     *          Handles packet encoding and framing.
     */
    void process_packet(uint8_t idx) override;
    
    /**
     * @brief Adjust WFQ weights based on data freshness
     * 
     * @param[in] queue_empty True if WFQ queue is empty
     * 
     * @details Dynamically adjusts packet transmission weights to prioritize
     *          fresh data (e.g., GPS position updates, attitude changes) over
     *          static data (e.g., parameters). Called periodically by scheduler.
     */
    void adjust_packet_weight(bool queue_empty) override;
    
    /**
     * @brief Initialize WFQ queue with packet types and weights
     * 
     * @details Sets up the Weighted Fair Queueing scheduler with all Passthrough
     *          packet types (TEXT, ATTITUDE, GPS_LAT, etc.) and their initial
     *          transmission weights. Called during initialization.
     */
    void setup_wfq_scheduler(void) override;

    /**
     * @brief Get next text message chunk for transmission
     * 
     * @return true if chunk available, false if no message pending
     * 
     * @details Retrieves the next 4-byte chunk of a queued status text message.
     *          Messages are sent in chunks with 3x repetition for reliability.
     *          Manages chunk index and repetition counter internally.
     * 
     * @note Text messages sent in 4-byte chunks with 3x repetition
     */
    bool get_next_msg_chunk(void) override;

    /**
     * @brief Export telemetry data for external transport (FPort mode)
     * 
     * @param[out] packet_array Buffer to receive telemetry packets
     * @param[in,out] packet_count Number of packets exported
     * @param[in] max_size Maximum buffer size in packets
     * 
     * @return true if data available and exported, false otherwise
     * 
     * @details Exports buffered telemetry packets for external transport such as FPort.
     *          Used when use_external_data is true. Telemetry is generated internally
     *          but transmitted via external protocol (RC + telemetry on single wire).
     */
    bool get_telem_data(sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size) override;
    
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    /**
     * @brief Inject external telemetry data for bidirectional processing
     * 
     * @param[in] frame SPort frame byte
     * @param[in] appid Application ID (sensor ID)
     * @param[in] data 32-bit data payload
     * 
     * @return true if data queued for processing, false if queue full
     * 
     * @details Injects externally received SPort telemetry for bidirectional
     *          command processing. Used in FPort mode when uplink data is
     *          received via external transport and needs MAVlite processing.
     * 
     * @note Conditional on HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
     */
    bool set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data) override;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

    /**
     * @brief Queue status text message for transmission
     * 
     * @param[in] severity MAVLink severity level (MAV_SEVERITY enum)
     * @param[in] text Null-terminated status text string
     * 
     * @details Queues a status text message for chunked transmission via
     *          TEXT packet type (0x5000). Messages are sent in 4-byte chunks
     *          with 3x repetition for reliability. Delegates to AP_RCTelemetry
     *          queue_message() implementation.
     * 
     * @note Messages sent in 4-byte chunks with 3x repetition
     */
    void queue_text_message(MAV_SEVERITY severity, const char *text) override
    {
        AP_RCTelemetry::queue_message(severity, text);
    }

    /**
     * @enum PassthroughPacketType
     * @brief Telemetry packet types with corresponding SPort sensor IDs
     * 
     * @details Defines all telemetry packet types transmitted via Passthrough protocol.
     *          Each type corresponds to a specific FrSky sensor ID (0x5000-0x500D, 0x800)
     *          and carries specific telemetry data decoded by OpenTX/EdgeTX scripts.
     *          
     *          Sensor IDs in hex format, data encoding varies by type.
     */
    enum PassthroughPacketType : uint8_t {
        TEXT =          0,  ///< @brief Status text messages (0x5000) - sent in 4-byte chunks with 3x repetition
        ATTITUDE =      1,  ///< @brief Roll, pitch, yaw (centidegrees), rangefinder distance (cm) (0x5006)
        GPS_LAT =       2,  ///< @brief GPS latitude in degrees (0x800)
        GPS_LON =       3,  ///< @brief GPS longitude in degrees (0x800)
        VEL_YAW =       4,  ///< @brief Groundspeed (m/s) and yaw (centidegrees) (0x5005)
        AP_STATUS =     5,  ///< @brief Autopilot status: armed, mode, failsafe flags (0x5001)
        GPS_STATUS =    6,  ///< @brief GPS fix type, satellite count, HDOP (0x5002)
        HOME =          7,  ///< @brief Home distance (meters) and direction (degrees) (0x5004)
        BATT_2 =        8,  ///< @brief Battery 2: voltage, current, capacity (0x5008)
        BATT_1 =        9,  ///< @brief Battery 1: voltage, current, capacity (0x5008)
        PARAM =         10, ///< @brief Parameter values: frame type, battery failsafe, features (0x5007)
        RPM =           11, ///< @brief RPM sensor 1 and 2 values (0x500A)
        UDATA =         12, ///< @brief User-defined data for custom scripting
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        MAV =           13, ///< @brief MAVlite messages for bidirectional commands (conditional)
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        TERRAIN =       14, ///< @brief Terrain altitude difference from vehicle (meters) (0x500B)
        WIND =          15, ///< @brief Wind speed (m/s) and direction (degrees) (0x500C)
        WAYPOINT =      16, ///< @brief Next waypoint: distance (meters), bearing (degrees) (0x500D)
        WFQ_LAST_ITEM       ///< @brief Sentinel value - must be last in enum
    };

protected:

    /**
     * @brief Main telemetry send method (overrides AP_Frsky_SPort)
     * 
     * @details Called periodically to transmit telemetry. Processes WFQ scheduler
     *          to determine next packet to send, generates packet data, and transmits
     *          via SPort. In external data mode, buffers packets for export instead.
     */
    void send() override;

private:

    AP_Frsky_Parameters *&_frsky_parameters;  ///< @brief Reference to FrSky parameters configuration

    /**
     * @enum PassthroughParam
     * @brief Parameters transmitted via PARAM packet (0x5007)
     * 
     * @details Defines parameter types sent in the PARAM telemetry packet.
     *          These parameters provide vehicle configuration information to
     *          the transmitter for display and decision making in telemetry scripts.
     */
    enum PassthroughParam : uint8_t {
        NONE =                0,  ///< @brief No parameter (placeholder)
        FRAME_TYPE =          1,  ///< @brief Vehicle frame type (copter, plane, rover, etc.)
        BATT_FS_VOLTAGE =     2,  ///< @brief Battery failsafe voltage threshold
        BATT_FS_CAPACITY =    3,  ///< @brief Battery failsafe capacity threshold (mAh)
        BATT_CAPACITY_1 =     4,  ///< @brief Battery 1 total capacity (mAh)
        BATT_CAPACITY_2 =     5,  ///< @brief Battery 2 total capacity (mAh)
        TELEMETRY_FEATURES =  6   ///< @brief Feature flags (bidirectional, scripting, etc.)
    };

    /**
     * @enum PassthroughFeatures
     * @brief Feature flags transmitted in TELEMETRY_FEATURES parameter
     * 
     * @details Bit flags indicating available telemetry features, sent in the
     *          PARAM packet with TELEMETRY_FEATURES type. Allows transmitter
     *          scripts to enable/disable features based on capabilities.
     */
    enum PassthroughFeatures : uint8_t {
        BIDIR =                 0,  ///< @brief Bidirectional support available (MAVlite commands enabled)
        SCRIPTING =             1,  ///< @brief Lua scripting available on autopilot
    };

    // Helper methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    
    /**
     * @brief Calculate parameter value for transmission
     * 
     * @return Encoded parameter data (uint32_t)
     * 
     * @details Generates PARAM packet (0x5007) containing vehicle configuration
     *          parameters. Cycles through parameter types (frame type, battery
     *          failsafe settings, capacities, features) on successive calls.
     */
    uint32_t calc_param(void);
    
    /**
     * @brief Calculate battery telemetry
     * 
     * @param[in] instance Battery instance (0 or 1)
     * 
     * @return Encoded battery data (uint32_t): voltage, current, capacity
     * 
     * @details Generates BATT packet (0x5008) with bit-packed battery telemetry:
     *          voltage (decivolts), current (deciamps), consumed capacity (mAh).
     *          Supports up to 2 battery instances.
     */
    uint32_t calc_batt(uint8_t instance);
    
    /**
     * @brief Calculate autopilot status flags
     * 
     * @return Encoded status data (uint32_t): armed, mode, failsafe, etc.
     * 
     * @details Generates AP_STATUS packet (0x5001) with bit-packed status flags:
     *          - Armed state
     *          - Flight mode
     *          - Failsafe status
     *          - EKF status
     *          - IMU temperature
     */
    uint32_t calc_ap_status(void);
    
    /**
     * @brief Calculate home distance and direction
     * 
     * @return Encoded home data (uint32_t): distance (meters), direction (degrees)
     * 
     * @details Generates HOME packet (0x5004) with distance from vehicle to home
     *          position in meters and direction to home in degrees (0-360).
     */
    uint32_t calc_home(void);
    
    /**
     * @brief Calculate velocity and yaw
     * 
     * @return Encoded data (uint32_t): groundspeed (m/s), yaw (centidegrees)
     * 
     * @details Generates VEL_YAW packet (0x5005) with vehicle groundspeed in m/s
     *          and yaw angle in centidegrees (0-36000).
     */
    uint32_t calc_velandyaw(void);
    
    /**
     * @brief Calculate attitude and rangefinder distance
     * 
     * @return Encoded data (uint32_t): roll/pitch (centidegrees), range (cm)
     * 
     * @details Generates ATTITUDE packet (0x5006) with vehicle attitude:
     *          - Roll angle in centidegrees
     *          - Pitch angle in centidegrees
     *          - Rangefinder distance in cm (or 0 if unavailable)
     */
    uint32_t calc_attiandrng(void);
    
    /**
     * @brief Calculate RPM sensor values
     * 
     * @return Encoded RPM data (uint32_t): RPM1, RPM2
     * 
     * @details Generates RPM packet (0x500A) with values from up to 2 RPM sensors.
     *          Used for monitoring motor RPM, rotor speed, or other rotating components.
     */
    uint32_t calc_rpm(void);
    
    /**
     * @brief Calculate terrain altitude
     * 
     * @return Encoded terrain data (uint32_t): altitude difference (meters)
     * 
     * @details Generates TERRAIN packet (0x500B) with altitude difference between
     *          vehicle and terrain below. Positive values indicate height above terrain.
     *          Requires terrain database or rangefinder data.
     */
    uint32_t calc_terrain(void);
    
    /**
     * @brief Calculate wind speed and direction
     * 
     * @return Encoded wind data (uint32_t): speed (m/s), direction (degrees)
     * 
     * @details Generates WIND packet (0x500C) with estimated wind speed and direction.
     *          Wind estimation derived from groundspeed vs airspeed comparison.
     *          Direction in degrees (0-360).
     */
    uint32_t calc_wind(void);
    
    /**
     * @brief Calculate next waypoint information
     * 
     * @return Encoded waypoint data (uint32_t): distance (meters), bearing (degrees)
     * 
     * @details Generates WAYPOINT packet (0x500D) with distance and bearing to
     *          next mission waypoint. Only valid when mission is active.
     */
    uint32_t calc_waypoint(void);

    /**
     * @brief Flag for external data mode (FPort)
     * 
     * @details When true, telemetry is buffered for external transport (e.g., FPort)
     *          rather than transmitted directly on SPort. Enables RC and telemetry
     *          on single wire.
     */
    bool _use_external_data;

    /**
     * @struct external_data
     * @brief Buffered packet for external transport
     * 
     * @details Holds telemetry packet awaiting export in external data mode.
     *          Used when telemetry is transported via FPort or similar protocol.
     */
    struct {
        sport_packet_t packet;  ///< @brief Buffered SPort packet
        bool pending;           ///< @brief True if packet awaiting export
    } external_data;

    /**
     * @struct _msg_chunk
     * @brief Text message chunking state for status text transmission
     * 
     * @details Manages transmission of status text messages in 4-byte chunks.
     *          Each chunk is sent 3 times for reliability. Tracks current chunk,
     *          repetition count, and character position in message.
     */
    struct {
        uint32_t chunk;         ///< @brief Current 4-byte chunk to transmit
        uint8_t repeats;        ///< @brief Repetition counter (0-2, sends each chunk 3x)
        uint8_t char_index;     ///< @brief Character offset in source message string
    } _msg_chunk;

    /**
     * @brief Default sensor ID for downlink telemetry
     * 
     * @details Primary sensor ID (0x1B) used for Passthrough telemetry transmission.
     *          Can be configured via parameters for multiple receiver support.
     */
    uint8_t downlink_sensor_id = 0x1B;

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    /**
     * @struct _SPort_bidir
     * @brief Bidirectional telemetry configuration and queues
     * 
     * @details Contains configuration and packet queues for bidirectional SPort
     *          telemetry. Supports uplink commands (parameter read/write, mode changes)
     *          and downlink responses via MAVlite protocol over SPort transport.
     * 
     * @note Conditional on HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
     * @warning Queue depth (SPORT_PACKET_QUEUE_LENGTH=210) must accommodate max message fragmentation
     */
    struct {
        uint8_t uplink_sensor_id = 0x0D;        ///< @brief Sensor ID for receiving commands (default 0x0D)
        uint8_t downlink1_sensor_id = 0x34;     ///< @brief Additional downlink sensor ID (0x34)
        uint8_t downlink2_sensor_id = 0x67;     ///< @brief Additional downlink sensor ID (0x67)
        uint8_t tx_packet_duplicates;           ///< @brief Packet duplication count for reliability (FPort only)
        ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> rx_packet_queue{SPORT_PACKET_QUEUE_LENGTH};  ///< @brief Received packet queue (thread-safe)
        ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> tx_packet_queue{SPORT_PACKET_QUEUE_LENGTH};  ///< @brief Transmit packet queue (thread-safe)
    } _SPort_bidir;

    AP_Frsky_SPortParser _sport_handler;                ///< @brief SPort packet parser for uplink
    AP_Frsky_MAVlite_SPortToMAVlite sport_to_mavlite;   ///< @brief Uplink: SPort packets to MAVlite messages
    AP_Frsky_MAVlite_MAVliteToSPort mavlite_to_sport;   ///< @brief Downlink: MAVlite messages to SPort packets

    /**
     * @brief Enqueue received SPort packet for processing
     * 
     * @param[in] sp Received SPort packet
     * 
     * @details Queues uplink SPort packet for MAVlite reconstruction and command
     *          processing. Called when poll response received from transmitter.
     */
    void queue_rx_packet(const AP_Frsky_SPort::sport_packet_t sp);
    
    /**
     * @brief Process received packet queue
     * 
     * @details Processes queued uplink packets, reconstructs MAVlite messages,
     *          and dispatches to command handler. Called periodically from
     *          telemetry thread.
     */
    void process_rx_queue(void);
    
    /**
     * @brief Transmit queued response packets
     * 
     * @details Transmits queued downlink packets generated from MAVlite responses.
     *          Handles packet duplication for FPort reliability if configured.
     *          Called periodically from telemetry thread.
     */
    void process_tx_queue(void);

    /**
     * @brief Send MAVlite response message
     * 
     * @param[in] txmsg MAVlite message to transmit
     * 
     * @return true if message queued successfully, false if queue full
     * 
     * @details Callback method for MAVlite message handler to send responses.
     *          Converts MAVlite message to SPort packets and queues for transmission.
     *          
     * @note Callback for AP_Frsky_MAVliteMsgHandler
     * @warning Commands rejected when armed unless specifically allowed for safety
     */
    bool send_message(const AP_Frsky_MAVlite_Message &txmsg);
    
    /**
     * @brief MAVlite command handler
     * 
     * @details Handles incoming MAVlite commands for parameter operations, mode changes,
     *          and autopilot commands. Bound to send_message() callback for responses.
     *          Implements safety checks to reject dangerous commands when armed.
     */
    AP_Frsky_MAVliteMsgHandler mavlite{FUNCTOR_BIND_MEMBER(&AP_Frsky_SPort_Passthrough::send_message, bool, const AP_Frsky_MAVlite_Message &)};
#endif
    /**
     * @brief Configure sensor ID from parameter
     * 
     * @param[in] idx Parameter containing sensor ID value
     * @param[out] sensor Output sensor ID variable to set
     * 
     * @details Reads sensor ID from parameter and configures the specified
     *          sensor ID variable. Used to configure uplink/downlink sensor IDs
     *          for bidirectional communication from user parameters.
     */
    void set_sensor_id(AP_Int8 idx, uint8_t &sensor);
    
    /**
     * @brief Transmit SPort frame (overrides base class)
     * 
     * @param[in] frame Frame byte
     * @param[in] appid Application ID (sensor ID)
     * @param[in] data 32-bit data payload
     * 
     * @details Transmits a complete SPort frame with specified sensor ID and data.
     *          In external data mode, buffers frame for export. In direct mode,
     *          transmits immediately on UART. Overrides AP_Frsky_SPort::send_sport_frame().
     */
    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    /**
     * @brief Check if byte is passthrough poll request
     * 
     * @param[in] byte Received byte to check
     * 
     * @return true if byte matches passthrough sensor ID and should respond, false otherwise
     * 
     * @details Determines if received polling byte is addressed to this passthrough
     *          telemetry instance by comparing with configured downlink sensor ID.
     */
    bool is_passthrough_byte(const uint8_t byte) const;

    uint8_t _paramID;  ///< @brief Current parameter ID for cycling through PARAM packet types

    /**
     * @brief Calculate GPS status telemetry
     * 
     * @return Encoded GPS status (uint32_t): fix type, satellite count, HDOP
     * 
     * @details Generates GPS_STATUS packet (0x5002) with bit-packed GPS telemetry:
     *          - Fix type (0=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed)
     *          - Number of satellites
     *          - HDOP (Horizontal Dilution of Precision)
     */
    uint32_t calc_gps_status(void);

    static AP_Frsky_SPort_Passthrough *singleton;  ///< @brief Singleton instance pointer
};

/**
 * @namespace AP
 * @brief ArduPilot global accessor namespace
 */
namespace AP {
    /**
     * @brief Get singleton instance of Passthrough telemetry
     * 
     * @return Pointer to AP_Frsky_SPort_Passthrough singleton, or nullptr if not initialized
     * 
     * @details Global accessor for Passthrough telemetry singleton. Used throughout
     *          codebase for accessing telemetry features like queue_text_message().
     */
    AP_Frsky_SPort_Passthrough *frsky_passthrough_telem();
};


#endif  // AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
