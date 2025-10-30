/**
 * @file AP_Frsky_SPort.h
 * @brief FrSky SPort (Smart Port) protocol implementation for X-series receivers
 * 
 * @details Implements the FrSky Smart Port telemetry protocol used by X-series receivers
 *          (X4R, X6R, X8R) which uses a poll-response model on a single-wire half-duplex
 *          UART connection operating at 57600 baud. The receiver polls sensor IDs and the
 *          autopilot responds with telemetry data using byte-stuffing for frame integrity.
 * 
 * @note SPort uses single-wire UART where receiver and transmitter are time-multiplexed
 * @note SPort checksum is sum of bytes with final inversion to 0xFF
 * 
 * @see AP_Frsky_Backend.h for base telemetry functionality
 * @see AP_Frsky_SPort_Passthrough.h for extended passthrough variant
 * @see AP_Frsky_SPortParser.h for low-level protocol parsing
 */

#pragma once

#include "AP_Frsky_Backend.h"

#if AP_FRSKY_SPORT_TELEM_ENABLED

/**
 * @class AP_Frsky_SPort
 * @brief FrSky SPort telemetry backend for X-series receivers
 * 
 * @details Implements FrSky SPort telemetry protocol for X-series receivers (X4R, X6R, X8R).
 *          
 *          Key characteristics:
 *          - Operates at 57600 baud on single-wire half-duplex UART
 *          - Poll-response model: receiver polls sensor IDs, autopilot responds with data
 *          - Uses byte-stuffing with FRAME_DLE (0x7D) escape and FRAME_XOR (0x20) mask
 *          - Supports external packet injection via sport_telemetry_push()
 *          - Singleton pattern for global access via AP::frsky_sport()
 *          - Conditional compilation via AP_FRSKY_SPORT_TELEM_ENABLED
 *          
 *          Protocol timing:
 *          - Must respond within receiver's timeout window (~3ms)
 *          - Receiver and transmitter time-multiplexed on single wire
 *          
 * @warning Must respond within receiver's timeout window to maintain telemetry link
 * @note Half-duplex communication requires careful timing management
 */
class AP_Frsky_SPort : public AP_Frsky_Backend
{

public:

    /**
     * @brief Initialize SPort backend and set singleton instance
     * 
     * @param[in] port UART driver for SPort communication (57600 baud, half-duplex)
     * 
     * @note Sets singleton pointer for AP::frsky_sport() global accessor
     */
    AP_Frsky_SPort(AP_HAL::UARTDriver *port) : AP_Frsky_Backend(port) {
        singleton = this;
    }

    /**
     * @brief Prevent copy construction and assignment (singleton pattern)
     * 
     * @note Copy operations disabled to enforce singleton access pattern
     */
    CLASS_NO_COPY(AP_Frsky_SPort);

    /**
     * @brief Main telemetry loop: poll UART and respond to sensor polls
     * 
     * @details Reads poll bytes from receiver and sequences sensor responses using call counters.
     *          This method implements the core poll-response protocol by:
     *          - Reading incoming poll bytes from the receiver
     *          - Determining which sensor data to send based on sequencing counters
     *          - Formatting and transmitting appropriate telemetry frames
     *          
     *          Timing constraints:
     *          - Checks port->receive_time_constraint_us before transmission
     *          - Requires txspace >= 19 bytes (worst-case with byte-stuffing)
     * 
     * @warning Must be called frequently to detect polls (timing-sensitive, ~3ms window)
     * @note This is the main entry point called by the scheduler
     */
    void send() override;
    
    /**
     * @brief Inject external telemetry packet for transmission
     * 
     * @details Thread-safe method allowing external modules to send custom telemetry packets.
     *          The packet is buffered and transmitted during the next appropriate poll cycle.
     *          Only one packet can be buffered at a time.
     * 
     * @param[in] sensor  Sensor ID (uint8_t)
     * @param[in] frame   Frame type (uint8_t)
     * @param[in] appid   Application ID (uint16_t)
     * @param[in] data    32-bit payload (int32_t)
     * 
     * @return true if packet successfully queued, false if buffer full
     * 
     * @warning Only one packet buffered; subsequent calls overwrite if not yet transmitted
     * @note Thread-safe via semaphore protection
     */
    bool sport_telemetry_push(const uint8_t sensor, const uint8_t frame, const uint16_t appid, const int32_t data);
    
    /**
     * @brief Compact number encoding for transmission
     * 
     * @details Scales number to fit in fewer bytes while preserving specified precision.
     *          Used to optimize telemetry bandwidth by encoding values compactly.
     * 
     * @param[in] number  Value to encode (int32_t)
     * @param[in] digits  Significant digits to preserve (uint8_t)
     * @param[in] power   Decimal exponent for scaling (uint8_t)
     * 
     * @return Encoded value as uint16_t
     */
    uint16_t prep_number(int32_t const number, const uint8_t digits, const uint8_t power);

    /**
     * @brief Get singleton instance pointer
     * 
     * @return Pointer to singleton instance, or nullptr if not initialized
     * 
     * @note Prefer using AP::frsky_sport() accessor from namespace
     */
    static AP_Frsky_SPort *get_singleton(void) {
        return singleton;
    }

protected:

    /**
     * @brief Transmit SPort frame with byte-stuffing
     * 
     * @details Generates 8-byte SPort frame with structure:
     *          - START (0x7E)
     *          - Sensor ID (1 byte)
     *          - Frame type (1 byte)
     *          - Application ID (2 bytes, little-endian)
     *          - Data payload (4 bytes, little-endian)
     *          - Checksum (1 byte)
     *          
     *          Applies byte-stuffing for 0x7E and 0x7D characters:
     *          - Escape byte: 0x7D (FRAME_DLE)
     *          - XOR mask: 0x20 (FRAME_XOR)
     * 
     * @param[in] frame  Frame type identifier (uint8_t)
     * @param[in] appid  Application/sensor ID (uint16_t)
     * @param[in] data   32-bit payload data (uint32_t)
     * 
     * @warning Requires txspace >= 19 bytes (worst-case with byte-stuffing)
     * @note Timing-sensitive; must complete within poll response window (~3ms)
     */
    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    /**
     * @brief Passthrough protocol state variables
     */
    struct {
        bool send_latitude;         ///< @brief Latitude/longitude alternator flag (alternates GPS coordinates to reduce bandwidth)
        bool send_airspeed;         ///< @brief Airspeed/groundspeed toggle for 0x5005 sensor (alternates between airspeed and groundspeed)
        uint32_t gps_lng_sample;    ///< @brief Cached longitude value for transmission
        uint8_t new_byte;           ///< @brief Most recently received poll byte from receiver
    } _passthrough;

    /**
     * @brief Compute GPS latitude/longitude for transmission
     * 
     * @details Formats GPS coordinates as DDDMM.MMMM (degrees + decimal minutes)
     *          with N/S/E/W hemisphere indicator. Alternates between latitude and
     *          longitude to reduce bandwidth usage.
     * 
     * @param[in,out] send_latitude  Alternator flag (true=send lat, false=send lon)
     * 
     * @return Encoded GPS coordinate as uint32_t
     * 
     * @note GPS coordinates sent alternately to reduce bandwidth
     */
    uint32_t calc_gps_latlng(bool &send_latitude);
    
    /**
     * @brief Calculate logical sensor ID from physical ID
     * 
     * @details Maps physical sensor ID (0-27) to SPort logical sensor ID
     *          using the FrSky sensor ID mapping algorithm.
     * 
     * @param[in] physical_id  Physical sensor identifier (0-27)
     * 
     * @return Logical sensor ID (uint8_t)
     * 
     * @note Static method for ID mapping algorithm
     */
    static uint8_t calc_sensor_id(const uint8_t physical_id);

    /**
     * @brief External packet injection buffer (for sport_telemetry_push)
     */
    struct {
        sport_packet_t packet;    ///< @brief Buffered external telemetry packet
        bool pending = false;     ///< @brief Packet awaiting transmission flag
        HAL_Semaphore sem;        ///< @brief Thread-safety semaphore for buffer access
    } _sport_push_buffer;

private:

    /**
     * @brief SPort protocol sequencing and status variables
     */
    struct {
        bool sport_status;         ///< @brief SPort communication link status
        bool gps_refresh;          ///< @brief GPS data refresh flag
        bool vario_refresh;        ///< @brief Variometer data refresh flag
        uint8_t fas_call;          ///< @brief FAS (fuel and sensor) transmission sequence counter (controls multi-packet sensor data sequencing)
        uint8_t gps_call;          ///< @brief GPS transmission sequence counter (controls multi-packet GPS data sequencing)
        uint8_t vario_call;        ///< @brief Variometer transmission sequence counter (controls multi-packet vario data sequencing)
        uint8_t various_call;      ///< @brief Miscellaneous sensors transmission sequence counter
        uint8_t rpm_call;          ///< @brief RPM sensor transmission sequence counter
    } _SPort;

    /**
     * @brief Static singleton instance pointer
     * 
     * @note Set by constructor, accessed via get_singleton() or AP::frsky_sport()
     */
    static AP_Frsky_SPort *singleton;

};

/**
 * @namespace AP
 * @brief Global accessor namespace for ArduPilot singletons
 */
namespace AP {
    /**
     * @brief Global accessor for FrSky SPort singleton
     * 
     * @return Pointer to AP_Frsky_SPort singleton instance, or nullptr if not initialized
     * 
     * @note Preferred method for accessing SPort telemetry instance
     */
    AP_Frsky_SPort *frsky_sport();
};

#endif  // AP_FRSKY_SPORT_TELEM_ENABLED
