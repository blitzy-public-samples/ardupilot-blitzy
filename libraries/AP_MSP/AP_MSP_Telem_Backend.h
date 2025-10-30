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

   MSP telemetry library backend base class
*/

/**
 * @file AP_MSP_Telem_Backend.h
 * @brief Abstract base class for MSP (MultiWii Serial Protocol) telemetry backends
 * 
 * @details This file defines the base class for MSP telemetry backend implementations.
 *          MSP backends support various OSD and telemetry display systems including:
 *          - Generic MSP OSD devices
 *          - DJI FPV OSD system
 *          - MSP DisplayPort protocol devices
 * 
 *          The backend provides core MSP protocol handling including packet framing,
 *          parsing, and a weighted-fair-queue scheduler for managing telemetry bandwidth.
 * 
 * @see AP_MSP_Telem_Generic, AP_MSP_Telem_DJI, AP_MSP_Telem_DisplayPort
 */
#pragma once

#include "AP_MSP_config.h"

#if HAL_MSP_ENABLED

#include <AP_RCTelemetry/AP_RCTelemetry.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_OSD/AP_OSD.h>

#include "msp.h"

#include <time.h>

/**
 * @brief Maximum number of telemetry packet slots in the WFQ scheduler
 * 
 * @details The weighted-fair-queue (WFQ) scheduler uses 12 time slots to manage
 *          bandwidth allocation across different telemetry packet types. This ensures
 *          fair distribution of limited serial bandwidth and prevents any single packet
 *          type from monopolizing the MSP link.
 */
#define MSP_TIME_SLOT_MAX 12

/**
 * @brief Full LiPo cell voltage in volts for cell count estimation
 * 
 * @details Standard voltage (4.35V) used to estimate battery cell count from total
 *          battery voltage. Divide total battery voltage by this value to determine
 *          number of cells in series.
 */
#define CELLFULL 4.35

/**
 * @brief Text buffer size including UTF-8 characters and null terminator
 * 
 * @details Buffer size of 15 bytes accommodates 11 visible characters plus up to
 *          3 UTF-8 multi-byte characters (such as directional arrows) plus null terminator.
 */
#define MSP_TXT_BUFFER_SIZE     15U // 11 + 3 utf8 chars + terminator

/**
 * @brief Number of visible ASCII characters displayable on OSD
 * 
 * @details Standard OSD text fields support 11 visible characters. UTF-8 multi-byte
 *          characters may reduce visible count but are accommodated in MSP_TXT_BUFFER_SIZE.
 */
#define MSP_TXT_VISIBLE_CHARS   11U

class AP_MSP;

/**
 * @class AP_MSP_Telem_Backend
 * @brief Abstract base class for MSP telemetry backend implementations
 * 
 * @details This class provides the foundation for MSP (MultiWii Serial Protocol) telemetry
 *          backends used in ArduPilot for OSD and telemetry display integration.
 * 
 *          Key responsibilities:
 *          - MSP protocol handling via msp_port_t for packet framing and parsing
 *          - Weighted-fair-queue (WFQ) scheduler for bandwidth-efficient telemetry
 *          - Integration with ArduPilot subsystems (GPS, Battery, AHRS, Navigation, etc.)
 *          - Virtual method interface for backend-specific implementations
 *          - Optional MSP DisplayPort protocol support for text-based OSD
 * 
 *          Derived classes implement specific MSP variants:
 *          - AP_MSP_Telem_Generic: Standard MSP OSD devices
 *          - AP_MSP_Telem_DJI: DJI FPV system with custom extensions
 *          - AP_MSP_Telem_DisplayPort: Character-based OSD protocol
 * 
 *          The backend inherits from AP_RCTelemetry to leverage the RC telemetry
 *          scheduling infrastructure and integrates with the AP_OSD system for
 *          display item management.
 * 
 * @note The WFQ scheduler ensures fair bandwidth allocation preventing any single
 *       telemetry packet type from saturating the serial link.
 * 
 * @warning Backends must use WITH_SEMAPHORE when accessing shared ArduPilot state
 *          to ensure thread-safety, especially when use_msp_thread() returns true.
 * 
 * @see AP_RCTelemetry, msp.h, AP_OSD
 */
class AP_MSP_Telem_Backend : AP_RCTelemetry
{
friend AP_MSP;
public:
    /**
     * @brief Constructor for MSP telemetry backend
     * 
     * @param[in] uart UART driver instance for MSP communication
     */
    AP_MSP_Telem_Backend(AP_HAL::UARTDriver *uart);

    /**
     * @struct battery_state_s
     * @brief Battery telemetry state structure
     * 
     * @details Aggregates battery monitoring data from AP_BattMonitor for transmission
     *          via MSP protocol. Supports multi-cell LiPo voltage monitoring and
     *          current/capacity tracking for OSD display and telemetry logging.
     */
    typedef struct battery_state_s {
        float batt_current_a;           ///< Battery current draw in amperes (A)
        float batt_consumed_mah;        ///< Consumed battery capacity in milliamp-hours (mAh)
        float batt_voltage_v;           ///< Total battery voltage in volts (V)
        int32_t batt_capacity_mah;      ///< Total battery capacity in milliamp-hours (mAh)
        uint8_t batt_cellcount;         ///< Number of LiPo cells in series (estimated from voltage)
        MSP::battery_state_e batt_state; ///< Battery state enum (OK, WARNING, CRITICAL, etc.)
    } battery_state_t;

    /**
     * @struct gps_state_s
     * @brief GPS telemetry state structure
     * 
     * @details Packed structure containing GPS position, velocity, and fix quality data
     *          from AP_GPS for MSP telemetry transmission. Uses scaled integer formats
     *          compatible with MSP protocol message definitions.
     */
    typedef struct PACKED gps_state_s {
        uint8_t fix_type;              ///< GPS fix type (0=No fix, 1=No fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed)
        uint8_t num_sats;              ///< Number of satellites used in solution
        int32_t lat;                   ///< Latitude in degrees * 1e7 (WGS84)
        int32_t lon;                   ///< Longitude in degrees * 1e7 (WGS84)
        uint16_t alt_m;                ///< Altitude above MSL in meters
        uint16_t speed_cms;            ///< Ground speed in centimeters per second (cm/s)
        uint16_t ground_course_dd;     ///< Ground course in degrees * 100 (0-35999 represents 0-359.99 degrees)
    } gps_state_t;

    /**
     * @struct airspeed_state_s
     * @brief Airspeed telemetry state structure
     * 
     * @details Contains airspeed estimate from AP_Airspeed or synthetic airspeed
     *          calculation. Primarily used for fixed-wing aircraft OSD display.
     */
    typedef struct airspeed_state_s {
        float airspeed_estimate_ms;    ///< Estimated airspeed in meters per second (m/s)
        bool  airspeed_have_estimate;  ///< True if airspeed estimate is valid
    } airspeed_state_t;

    /**
     * @struct home_state_s
     * @brief Home position telemetry state structure
     * 
     * @details Contains relative position data from current location to home position
     *          for return-to-launch (RTL) navigation display on OSD.
     */
    typedef struct home_state_s {
        bool home_is_set;              ///< True if home position has been established
        float home_bearing_cd;         ///< Bearing to home in centidegrees (0-35999)
        uint32_t home_distance_m;      ///< Horizontal distance to home in meters
        int32_t rel_altitude_cm;       ///< Altitude relative to home in centimeters (positive = above home)
    } home_state_t;

    /**
     * @brief Initialize the MSP telemetry backend
     * 
     * @details Performs backend initialization including UART setup, WFQ scheduler
     *          configuration, and MSP port state initialization. Called once during
     *          AP_MSP subsystem startup.
     * 
     * @return true if initialization successful, false otherwise
     * 
     * @note Overrides AP_RCTelemetry::init()
     */
    virtual bool init() override;
    
    /**
     * @brief Initialize UART driver for MSP communication
     * 
     * @details Configures the UART with appropriate baud rate, parity, stop bits,
     *          and flow control settings for MSP protocol communication.
     * 
     * @return true if UART initialization successful, false otherwise
     */
    virtual bool init_uart();
    
    /**
     * @brief Enable OSD warning message display
     * 
     * @details Enables display of system warnings (low battery, GPS loss, etc.)
     *          on the OSD. Backend-specific implementation may filter or format
     *          warnings based on display capabilities.
     */
    virtual void enable_warnings();
    
    /**
     * @brief Hide specific OSD items dynamically
     * 
     * @details Backend-specific method to hide certain OSD elements based on
     *          flight mode, vehicle state, or display system capabilities.
     *          Updates osd_hidden_items_bitmask for OSD rendering.
     */
    virtual void hide_osd_items(void);

    /**
     * @brief Process incoming MSP data from UART
     * 
     * @details Reads available data from UART, parses MSP frames using msp_port_t
     *          state machine, and dispatches complete packets to command handlers.
     *          Should be called regularly (typically at 10-50Hz) to maintain
     *          responsive MSP communication.
     * 
     * @note Handles both MSP v1 and MSP v2 protocol variants
     */
    void process_incoming_data();
    
    /**
     * @brief Process outgoing telemetry data to UART
     * 
     * @details Uses WFQ scheduler to select next telemetry packet for transmission,
     *          serializes data, frames as MSP packet, and writes to UART. Ensures
     *          fair bandwidth allocation across packet types.
     * 
     * @note Called regularly by scheduler or MSP thread to push telemetry updates
     */
    void process_outgoing_data();

#if HAL_WITH_MSP_DISPLAYPORT
    /**
     * @brief Send DisplayPort heartbeat to maintain connection
     * 
     * @details Keeps the MSP DisplayPort connection alive by sending periodic
     *          heartbeat messages. Should be called regularly (typically at 1-5Hz)
     *          to prevent the display device from timing out.
     * 
     * @note Based on betaflight/src/main/io/displayport_msp.c implementation
     */
    virtual void msp_displayport_heartbeat();
    
    /**
     * @brief Acquire exclusive control of the DisplayPort
     * 
     * @details Takes control of the DisplayPort interface, preventing other systems
     *          from writing to the display. Used when switching to MSP OSD mode.
     */
    virtual void msp_displayport_grab();
    
    /**
     * @brief Release exclusive control of the DisplayPort
     * 
     * @details Releases DisplayPort control, allowing other systems to access the
     *          display. Called when exiting MSP OSD mode.
     */
    virtual void msp_displayport_release();
    
    /**
     * @brief Clear the entire OSD screen
     * 
     * @details Clears all character positions on the OSD display, typically called
     *          before redrawing the complete screen or when initializing the display.
     */
    virtual void msp_displayport_clear_screen();
    
    /**
     * @brief Commit buffered screen updates to display
     * 
     * @details Finalizes and transmits all pending screen updates to the display
     *          device. Should be called after a series of write_string operations
     *          to minimize MSP message overhead.
     */
    virtual void msp_displayport_draw_screen();
    
    /**
     * @brief Write a text string to the OSD at specified position
     * 
     * @details Writes a text string to the DisplayPort character grid at the
     *          specified column and row position with optional blink attribute.
     * 
     * @param[in] col Column position (0-based, typically 0-29)
     * @param[in] row Row position (0-based, typically 0-15)
     * @param[in] blink True to enable character blinking for emphasis
     * @param[in] string Null-terminated string to display
     * @param[in] font_table Font lookup table index for character rendering
     * 
     * @note String length should not exceed display width minus column position
     * @note Based on betaflight/src/main/io/displayport_msp.c implementation
     */
    virtual void msp_displayport_write_string(uint8_t col, uint8_t row, bool blink, const char *string, const uint8_t font_table);
    
    /**
     * @brief Configure DisplayPort font and resolution options
     * 
     * @details Sets display rendering options including font selection and screen
     *          resolution mode for optimal OSD appearance.
     * 
     * @param[in] font_index Font variant index (0-3, device-specific)
     * @param[in] screen_resolution Resolution mode (0=PAL, 1=NTSC, 2=HD, device-specific)
     */
    virtual void msp_displayport_set_options(const uint8_t font_index, const uint8_t screen_resolution);
#endif
protected:
    /**
     * @enum msp_packet_type
     * @brief Telemetry packet types for WFQ scheduler slots
     * 
     * @details Defines the types of telemetry packets that can be scheduled for
     *          transmission via MSP. Each packet type corresponds to a specific
     *          MSP command and occupies one or more slots in the WFQ scheduler.
     */
    enum msp_packet_type : uint8_t {
        EMPTY_SLOT = 0,      ///< Unused scheduler slot
        NAME,                ///< Vehicle name/identifier (MSP_NAME)
        STATUS,              ///< Flight status and armed state (MSP_STATUS)
        CONFIG,              ///< OSD configuration (MSP_OSD_CONFIG)
        RAW_GPS,             ///< Raw GPS position data (MSP_RAW_GPS)
        COMP_GPS,            ///< Computed GPS data with home distance/bearing (MSP_COMP_GPS)
        ATTITUDE,            ///< Vehicle attitude (roll/pitch/yaw) (MSP_ATTITUDE)
        ALTITUDE,            ///< Barometric and climb rate data (MSP_ALTITUDE)
        ANALOG,              ///< Analog sensor data (battery voltage, rssi) (MSP_ANALOG)
        BATTERY_STATE,       ///< Detailed battery state (MSP_BATTERY_STATE)
#if HAL_WITH_ESC_TELEM
        ESC_SENSOR_DATA,     ///< ESC telemetry data (MSP_ESC_SENSOR_DATA)
#endif
        RTC_DATETIME,        ///< Real-time clock date/time (MSP_RTC)
    };

    /**
     * @brief Maps WFQ scheduler packet slots to MSP command IDs
     * 
     * @details Array mapping each msp_packet_type enum value to the corresponding
     *          MSP protocol command ID. Used by the WFQ scheduler to determine which
     *          MSP command to send when a particular packet slot is selected.
     * 
     * @note Array size is MSP_TIME_SLOT_MAX (12 slots)
     * @note Index 0 (EMPTY_SLOT) maps to 0 (no command)
     */
    const uint16_t msp_packet_type_map[MSP_TIME_SLOT_MAX] = {
        0,                    // EMPTY_SLOT
        MSP_NAME,             // NAME
        MSP_STATUS,           // STATUS
        MSP_OSD_CONFIG,       // CONFIG
        MSP_RAW_GPS,          // RAW_GPS
        MSP_COMP_GPS,         // COMP_GPS
        MSP_ATTITUDE,         // ATTITUDE
        MSP_ALTITUDE,         // ALTITUDE
        MSP_ANALOG,           // ANALOG
        MSP_BATTERY_STATE,    // BATTERY_STATE
#if HAL_WITH_ESC_TELEM
        MSP_ESC_SENSOR_DATA,  // ESC_SENSOR_DATA
#endif
        MSP_RTC               // RTC_DATETIME
    };

    /**
     * @brief UTF-8 directional arrow encoding table for OSD display
     * 
     * @details Third byte of UTF-8 three-byte encoding sequence for directional arrows.
     *          Full encoding is: 0xE2 0x86 <value_from_table>
     * 
     *          Arrow directions (clockwise from North):
     *          - Index 0: U+2191 ↑  (0x91) UPWARDS ARROW (North)
     *          - Index 1: U+2197 ↗  (0x97) NORTH EAST ARROW
     *          - Index 2: U+2192 →  (0x92) RIGHTWARDS ARROW (East)
     *          - Index 3: U+2198 ↘  (0x98) SOUTH EAST ARROW
     *          - Index 4: U+2193 ↓  (0x93) DOWNWARDS ARROW (South)
     *          - Index 5: U+2199 ↙  (0x99) SOUTH WEST ARROW
     *          - Index 6: U+2190 ←  (0x90) LEFTWARDS ARROW (West)
     *          - Index 7: U+2196 ↖  (0x96) NORTH WEST ARROW
     * 
     * @note Used for displaying vehicle heading, wind direction, and home bearing on OSD
     * @note Index corresponds to 45-degree increments starting from North (0°)
     */
    static constexpr uint8_t arrows[8] = {0x91, 0x97, 0x92, 0x98, 0x93, 0x99, 0x90, 0x96};

    static const uint8_t message_scroll_time_ms = 200;  ///< Message scroll rate in milliseconds per character
    static const uint8_t message_scroll_delay = 5;      ///< Delay count before starting message scroll

    /**
     * @brief Bitmask of OSD items to hide dynamically
     * 
     * @details Each bit corresponds to an AP_OSD item that should be hidden based
     *          on current vehicle state, flight mode, or backend display capabilities.
     *          Updated by hide_osd_items() virtual method.
     */
    uint64_t osd_hidden_items_bitmask;

    /**
     * @brief MSP protocol parser state machine
     * 
     * @details Maintains state for MSP packet framing, parsing, and checksum validation.
     *          Handles both MSP v1 (checksum) and MSP v2 (CRC) protocol variants.
     * 
     * @see msp.h for msp_port_t definition
     */
    MSP::msp_port_t _msp_port;

    /**
     * @brief Check if a telemetry packet is ready for transmission
     * 
     * @details WFQ scheduler callback to determine if the packet at slot idx should
     *          be transmitted in the current scheduling cycle.
     * 
     * @param[in] idx Packet slot index (0 to MSP_TIME_SLOT_MAX-1)
     * @param[in] queue_empty True if no packets are pending transmission
     * @return true if packet should be sent, false to skip this slot
     * 
     * @note Overrides AP_RCTelemetry::is_packet_ready()
     */
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    
    /**
     * @brief Process and transmit the telemetry packet at slot idx
     * 
     * @details WFQ scheduler callback to serialize and send the telemetry packet
     *          corresponding to the specified slot index.
     * 
     * @param[in] idx Packet slot index (0 to MSP_TIME_SLOT_MAX-1)
     * 
     * @note Overrides AP_RCTelemetry::process_packet()
     */
    void process_packet(uint8_t idx) override;
    
    /**
     * @brief Adjust packet scheduling weights (not used by MSP)
     * 
     * @details Placeholder for dynamic weight adjustment. MSP uses fixed packet
     *          priorities and does not implement dynamic weight adjustment.
     * 
     * @param[in] queue_empty True if no packets are pending transmission
     * 
     * @note Overrides AP_RCTelemetry::adjust_packet_weight()
     */
    void adjust_packet_weight(bool queue_empty) override {}
    
    /**
     * @brief Initialize the WFQ scheduler for MSP telemetry
     * 
     * @details Configures the weighted-fair-queue scheduler with MSP_TIME_SLOT_MAX slots
     *          and assigns packet types to slots via msp_packet_type_map.
     * 
     * @note Overrides AP_RCTelemetry::setup_wfq_scheduler()
     */
    void setup_wfq_scheduler(void) override;
    
    /**
     * @brief Get next message chunk (not used by MSP, returns true)
     * 
     * @details MSP packets are sent atomically, not chunked. This method always
     *          returns true to indicate the complete message is available.
     * 
     * @return true (complete packet always ready)
     * 
     * @note Overrides AP_RCTelemetry::get_next_msg_chunk()
     */
    bool get_next_msg_chunk(void) override
    {
        return true;
    }

    /**
     * @brief Calculate LiPo cell count from total battery voltage
     * 
     * @details Estimates the number of LiPo cells in series by dividing total voltage
     *          by CELLFULL (4.35V). Used for per-cell voltage display on OSD.
     * 
     * @param[in] battery_voltage Total battery voltage in volts
     * @return Estimated number of cells (1-12 typical range)
     * 
     * @note Assumes fully charged LiPo voltage of 4.35V per cell
     */
    uint8_t calc_cell_count(float battery_voltage);
    
    /**
     * @brief Get vertical speed in meters per second
     * 
     * @details Retrieves current vertical velocity from AHRS/EKF for OSD display
     *          and telemetry. Positive values indicate climbing.
     * 
     * @return Vertical speed in m/s (positive = climbing, negative = descending)
     */
    virtual float get_vspeed_ms(void) const;
    
    /**
     * @brief Get RSSI (Received Signal Strength Indicator) percentage
     * 
     * @details Retrieves RC link RSSI from AP_RSSI for signal quality display.
     * 
     * @param[out] rssi RSSI percentage (0-100, or -1 if unavailable)
     * @return true if RSSI available, false otherwise
     */
    virtual bool get_rssi(float &rssi) const;
    
    /**
     * @brief Update home position telemetry state
     * 
     * @details Retrieves current home position data from AP_AHRS including bearing,
     *          distance, and relative altitude for RTL navigation display.
     * 
     * @param[out] home_state Structure to populate with home position data
     */
    virtual void update_home_pos(home_state_t &home_state);
    
    /**
     * @brief Update battery telemetry state
     * 
     * @details Retrieves battery monitoring data from AP_BattMonitor including voltage,
     *          current, consumed capacity, and estimated cell count.
     * 
     * @param[out] _battery_state Structure to populate with battery data
     */
    virtual void update_battery_state(battery_state_t &_battery_state);
    
    /**
     * @brief Update GPS telemetry state
     * 
     * @details Retrieves GPS data from AP_GPS including fix type, satellite count,
     *          position, altitude, speed, and ground course.
     * 
     * @param[out] gps_state Structure to populate with GPS data
     */
    virtual void update_gps_state(gps_state_t &gps_state);
    
    /**
     * @brief Update airspeed telemetry state
     * 
     * @details Retrieves airspeed estimate from AP_Airspeed or calculates synthetic
     *          airspeed for fixed-wing aircraft. Returns false for rotorcraft.
     * 
     * @param[out] airspeed_state Structure to populate with airspeed data
     */
    virtual void update_airspeed(airspeed_state_t &airspeed_state);
    
    /**
     * @brief Update flight mode string for OSD display
     * 
     * @details Generates human-readable flight mode string with optional wind data
     *          for display on OSD. Formats mode name to fit OSD character limits.
     * 
     * @param[out] flight_mode_str Buffer to store mode string
     * @param[in] size Buffer size in bytes
     * @param[in] wind_enabled True to append wind speed/direction to mode string
     */
    virtual void update_flight_mode_str(char *flight_mode_str, uint8_t size, bool wind_enabled);

    /**
     * @brief Process a received MSP command packet
     * 
     * @details Parses complete MSP packet from _msp_port, validates checksum/CRC,
     *          dispatches to command handler, and generates reply if needed.
     *          Called by process_incoming_data() when complete packet received.
     */
    void msp_process_received_command();
    
    /**
     * @brief Process an MSP command and generate reply
     * 
     * @details Main command dispatcher that routes MSP commands to appropriate
     *          handlers based on command ID. Supports both incoming commands (SET)
     *          and outgoing data requests (GET).
     * 
     * @param[in] cmd Received command packet
     * @param[out] reply Reply packet to populate
     * @return MSP::MSPCommandResult (MSP_RESULT_ACK, MSP_RESULT_ERROR, MSP_RESULT_NO_REPLY)
     */
    MSP::MSPCommandResult msp_process_command(MSP::msp_packet_t *cmd, MSP::msp_packet_t *reply);
    
    /**
     * @brief Process MSP v2 sensor data command
     * 
     * @details Handles MSP v2 sensor data messages for external sensor integration
     *          (optical flow, rangefinder, GPS, compass, barometer, airspeed).
     * 
     * @param[in] cmd_msp MSP command ID
     * @param[in] src Source buffer containing sensor data
     * @return MSP::MSPCommandResult (MSP_RESULT_ACK, MSP_RESULT_ERROR, MSP_RESULT_NO_REPLY)
     */
    MSP::MSPCommandResult msp_process_sensor_command(uint16_t cmd_msp, MSP::sbuf_t *src);
    
    /**
     * @brief Process outgoing telemetry command
     * 
     * @details Generates MSP telemetry data response for the specified command ID.
     *          Called by WFQ scheduler and in response to host requests.
     * 
     * @param[in] cmd_msp MSP command ID to generate response for
     * @param[out] dst Destination buffer to write telemetry data
     * @return MSP::MSPCommandResult (MSP_RESULT_ACK, MSP_RESULT_ERROR, MSP_RESULT_NO_REPLY)
     */
    MSP::MSPCommandResult msp_process_out_command(uint16_t cmd_msp, MSP::sbuf_t *dst);

    /**
     * @brief Frame and send an MSP packet
     * 
     * @details Frames data as MSP v1 or MSP v2 packet with header, length, command ID,
     *          payload, and checksum/CRC. Writes framed packet to UART.
     * 
     * @param[in] cmd MSP command ID
     * @param[in] msp_version MSP protocol version (MSP_V1 or MSP_V2)
     * @param[in] p Pointer to payload data
     * @param[in] size Payload size in bytes
     * @param[in] is_request True for request packet (<), false for response packet (>)
     */
    void msp_send_packet(uint16_t cmd, MSP::msp_version_e msp_version, const void *p, uint16_t size, bool is_request);

    /**
     * @brief Handle MSP optical flow sensor data
     * 
     * @details Processes external optical flow data received via MSP and injects
     *          into AP_OpticalFlow for velocity estimation and position hold.
     * 
     * @param[in] pkt Optical flow data packet with quality, flow rates, and body rates
     */
    void msp_handle_opflow(const MSP::msp_opflow_data_message_t &pkt);
    
    /**
     * @brief Handle MSP rangefinder sensor data
     * 
     * @details Processes external rangefinder distance data received via MSP and injects
     *          into AP_RangeFinder for terrain following and precision landing.
     * 
     * @param[in] pkt Rangefinder data packet with distance measurement and quality
     */
    void msp_handle_rangefinder(const MSP::msp_rangefinder_data_message_t &pkt);
    
    /**
     * @brief Handle MSP GPS sensor data
     * 
     * @details Processes external GPS data received via MSP and injects into AP_GPS
     *          for navigation. Supports position, velocity, and fix quality data.
     * 
     * @param[in] pkt GPS data packet with position, velocity, fix type, and satellite count
     */
    void msp_handle_gps(const MSP::msp_gps_data_message_t &pkt);
    
    /**
     * @brief Handle MSP compass sensor data
     * 
     * @details Processes external magnetometer data received via MSP and injects
     *          into AP_Compass for heading estimation and navigation.
     * 
     * @param[in] pkt Compass data packet with magnetic field vector (X, Y, Z in milligauss)
     */
    void msp_handle_compass(const MSP::msp_compass_data_message_t &pkt);
    
    /**
     * @brief Handle MSP barometer sensor data
     * 
     * @details Processes external barometric pressure data received via MSP and injects
     *          into AP_Baro for altitude estimation and vertical velocity.
     * 
     * @param[in] pkt Barometer data packet with pressure (Pa) and temperature (°C)
     */
    void msp_handle_baro(const MSP::msp_baro_data_message_t &pkt);
    
    /**
     * @brief Handle MSP airspeed sensor data
     * 
     * @details Processes external airspeed data received via MSP and injects into
     *          AP_Airspeed for fixed-wing control and energy management (TECS).
     * 
     * @param[in] pkt Airspeed data packet with differential pressure and temperature
     */
    void msp_handle_airspeed(const MSP::msp_airspeed_data_message_t &pkt);

    /**
     * @brief Get OSD flight mode status bitmask
     * 
     * @details Returns bitmask encoding vehicle arming status and flight mode state
     *          for OSD display. Default implementation provides arming status;
     *          derived classes can add vendor-specific flags.
     * 
     * @return uint32_t bitmask with arming and mode status flags
     * 
     * @note Bitmask format is backend-specific (Generic MSP, DJI, etc.)
     */
    virtual uint32_t get_osd_flight_mode_bitmask(void);

    /**
     * @brief Check if push telemetry scheduler is enabled
     * 
     * @details Pure virtual method indicating whether this backend uses push-based
     *          telemetry (actively sends data) or only responds to requests.
     *          OSD backends typically enable scheduler for periodic updates.
     * 
     * @return true if WFQ scheduler should actively push telemetry, false for request/response only
     * 
     * @note Must be implemented by derived classes
     */
    virtual bool is_scheduler_enabled() const = 0;
    
    /**
     * @brief Check if backend is handled by dedicated MSP thread
     * 
     * @details Indicates whether this backend's processing is performed by a dedicated
     *          MSP thread or integrated into the main scheduler. Affects synchronization
     *          and thread-safety requirements.
     * 
     * @return true if processed by MSP thread (default), false if handled by main scheduler
     */
    virtual bool use_msp_thread() const {return true;};
    
    /**
     * @brief Get the serial protocol enum for this backend
     * 
     * @details Pure virtual method returning the AP_SerialManager::SerialProtocol
     *          value for this MSP backend variant (SerialProtocol_MSP, SerialProtocol_DJI_FPV, etc.).
     * 
     * @return AP_SerialManager::SerialProtocol enum value
     * 
     * @note Must be implemented by derived classes
     */
    virtual AP_SerialManager::SerialProtocol get_serial_protocol() const = 0;
    
    /**
     * @brief Check if post-flight statistics screen is being displayed
     * 
     * @details Indicates whether the OSD is currently showing post-flight statistics
     *          (flight time, distance, max altitude, etc.). Used to modify telemetry
     *          behavior when stats screen is active.
     * 
     * @return true if stats screen is displayed, false otherwise
     */
    virtual bool displaying_stats_screen() const;

    /**
     * @brief Generate MSP_API_VERSION response
     * 
     * @details Serializes MSP API protocol version (major, minor, patch) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_api_version(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_FC_VERSION response
     * 
     * @details Serializes flight controller firmware version (ArduPilot version) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_fc_version(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_FC_VARIANT response
     * 
     * @details Serializes flight controller variant identifier (e.g., "ARDU") to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_fc_variant(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_UID response
     * 
     * @details Serializes unique device identifier (board serial number) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_uid(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_BOARD_INFO response
     * 
     * @details Serializes board hardware information (board type, revision) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_board_info(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_BUILD_INFO response
     * 
     * @details Serializes firmware build date and time to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_build_info(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_NAME response
     * 
     * @details Serializes vehicle name string to dst buffer for OSD identification.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_name(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_STATUS response
     * 
     * @details Serializes vehicle status including arming state, flight mode, and flags to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_status(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_OSD_CONFIG response
     * 
     * @details Serializes OSD configuration including enabled items, positions, and settings to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_osd_config(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_RAW_GPS response
     * 
     * @details Serializes raw GPS data (position, velocity, fix, satellites) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_raw_gps(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_COMP_GPS response
     * 
     * @details Serializes computed GPS data including home distance and bearing to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_comp_gps(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_ATTITUDE response
     * 
     * @details Serializes vehicle attitude (roll, pitch, yaw) in decidegrees to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_attitude(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_ALTITUDE response
     * 
     * @details Serializes altitude data (barometric altitude, variometer) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_altitude(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_ANALOG response
     * 
     * @details Serializes analog sensor data (battery voltage, current, RSSI) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_analog(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_BATTERY_STATE response
     * 
     * @details Serializes detailed battery state (voltage, current, capacity, cell count) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_battery_state(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_ESC_SENSOR_DATA response
     * 
     * @details Serializes ESC telemetry data (RPM, voltage, current, temperature) to dst buffer.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     * 
     * @note Only available when HAL_WITH_ESC_TELEM is enabled
     */
    virtual MSP::MSPCommandResult msp_process_out_esc_sensor_data(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_RTC response
     * 
     * @details Serializes real-time clock date and time to dst buffer for OSD display.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_rtc(MSP::sbuf_t *dst);
    
    /**
     * @brief Generate MSP_RC response
     * 
     * @details Serializes RC channel values to dst buffer for display or configuration.
     * 
     * @param[out] dst Destination buffer for response data
     * @return MSP::MSPCommandResult (typically MSP_RESULT_ACK)
     */
    virtual MSP::MSPCommandResult msp_process_out_rc(MSP::sbuf_t *dst);
};

#endif  //HAL_MSP_ENABLED
