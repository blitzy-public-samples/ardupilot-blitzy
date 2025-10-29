/**
 * @file AP_RangeFinder_BLPing.h
 * @brief Blue Robotics Ping1D underwater sonar rangefinder backend
 * 
 * This driver implements support for the Blue Robotics Ping1D underwater acoustic rangefinder
 * using the proprietary Ping Protocol over serial communication. The Ping1D is specifically
 * designed for subsea applications including ROV depth measurement, obstacle detection, and
 * underwater distance sensing.
 * 
 * The sensor communicates using a binary protocol with 'BR' header frames containing
 * message_id, payload_length, source/destination device IDs, payload data, and checksum.
 * 
 * Protocol Reference: https://github.com/bluerobotics/ping-protocol
 * 
 * @note This sensor is optimized for underwater use where acoustic speed of sound in water
 *       (~1500 m/s) differs significantly from air (~343 m/s)
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BLPING_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class PingProtocol
 * @brief Protocol parser for Blue Robotics Ping Protocol
 *
 * @details This class implements a byte-by-byte state machine parser for the proprietary
 *          Ping Protocol used by Blue Robotics underwater sensors. The protocol uses a
 *          structured binary frame format with fixed header bytes ('B' and 'R'), variable
 *          length payload, and checksum validation.
 *
 *          Frame Structure:
 *          - Header: Two-byte start sequence 'BR' (0x42, 0x52)
 *          - Payload Length: 16-bit length field (little-endian)
 *          - Message ID: 16-bit message identifier (little-endian)
 *          - Source/Destination IDs: Single-byte device addressing
 *          - Payload: Variable-length message data
 *          - Checksum: 16-bit sum of all non-checksum bytes (little-endian)
 *
 *          Protocol documentation: https://github.com/bluerobotics/ping-protocol
 *
 *  Byte     Type        Name            Description
 *  --------------------------------------------------------------------------------------------------------------
 *  0        uint8_t     start1          'B' (0x42) - First header byte
 *  1        uint8_t     start2          'R' (0x52) - Second header byte
 *  2-3      uint16_t    payload_length  Number of bytes in payload (low byte, high byte)
 *  4-5      uint16_t    message id      Message identifier (low byte, high byte)
 *  6        uint8_t     src_device_id   ID of device sending the message
 *  7        uint8_t     dst_device_id   ID of device of the intended recipient
 *  8-n      uint8_t[]   payload         Message payload data
 *  (n+1)-(n+2)  uint16_t    checksum    Sum of all non-checksum bytes in message (low byte, high byte)
 */
class PingProtocol {
    static constexpr uint8_t _frame_header1 = 0x42; // // header first byte ('B')
    static constexpr uint8_t _frame_header2 = 0x52; // // header first byte ('R')
    static constexpr uint16_t _src_id = 0; // vehicle's source id
    static constexpr uint16_t _dst_id = 1; // sensor's id

public:
    enum class MessageId {
        INVALID = 0,
        SET_PING_INTERVAL = 1004,
        DISTANCE_SIMPLE = 1211,
        CONTINUOUS_START = 1400,
    };

    /**
     * @brief Process a single byte received on serial port using state machine parser
     * 
     * @details Implements byte-by-byte parsing of Ping Protocol frames. The parser maintains
     *          state across multiple calls, accumulating bytes until a complete valid message
     *          is received. The state machine progresses through header detection, length/ID
     *          extraction, payload reception, and checksum validation.
     *
     * @param[in] b Single byte received from serial interface
     * 
     * @return MessageId indicating parsing result:
     *         - MessageId::INVALID (0): No complete message yet or invalid frame
     *         - MessageId::SET_PING_INTERVAL (1004): Ping interval configuration message
     *         - MessageId::DISTANCE_SIMPLE (1211): Distance measurement message
     *         - MessageId::CONTINUOUS_START (1400): Continuous mode start message
     */
    MessageId parse_byte(uint8_t b);

    /**
     * @brief Send a Ping Protocol message with specified payload to sensor
     *
     * @details Constructs and transmits a complete Ping Protocol frame including header bytes,
     *          length fields, message ID, device IDs, payload data, and computed checksum.
     *
     * @param[in] uart         UART driver instance for serial transmission
     * @param[in] msg_id       Message type identifier (SET_PING_INTERVAL, DISTANCE_SIMPLE, etc.)
     * @param[in] payload      Pointer to message payload data array, or nullptr if no payload
     * @param[in] payload_len  Length of payload data in bytes (0 if no payload)
     */
    void send_message(AP_HAL::UARTDriver *uart, PingProtocol::MessageId msg_id, const uint8_t *payload, uint16_t payload_len) const;

    /**
     * @brief Extract distance measurement from received DISTANCE_SIMPLE message
     *
     * @details Parses the distance value from the most recently received and validated
     *          DISTANCE_SIMPLE message payload. Distance represents acoustic time-of-flight
     *          measurement in underwater environment.
     *
     * @return Distance measurement in millimeters (mm)
     */
    uint32_t get_distance_mm() const;

    /**
     * @brief Extract confidence/quality metric from received message
     *
     * @details Returns the sensor's self-reported measurement confidence from the most
     *          recently received message. Higher values indicate stronger acoustic return
     *          signal and more reliable distance measurement.
     *
     * @return Confidence percentage: 0-100 (0=lowest confidence, 100=highest confidence)
     */
    uint8_t get_confidence() const;

    /**
     * @brief Get the message id available in bufffer
     *
     * @return MessageId
     */
    MessageId get_message_id() const { return static_cast<MessageId>(msg.id); };

protected:
    /**
     * @brief Parser state machine stages for byte-by-byte Ping Protocol frame parsing
     *
     * @details The parser progresses sequentially through these states as bytes are received,
     *          validating frame structure and accumulating message data. State transitions
     *          occur on each successfully parsed byte, with reset to HEADER1 on errors.
     *
     * State Machine Flow:
     * HEADER1 → HEADER2 → LEN_L → LEN_H → MSG_ID_L → MSG_ID_H → 
     * SRC_ID → DST_ID → PAYLOAD (repeated payload_length times) → CRC_L → CRC_H
     */
    enum class ParserState {
        HEADER1 = 0,  ///< Waiting for first header byte 'B' (0x42)
        HEADER2,      ///< Waiting for second header byte 'R' (0x52)
        LEN_L,        ///< Receiving payload length low byte
        LEN_H,        ///< Receiving payload length high byte
        MSG_ID_L,     ///< Receiving message ID low byte
        MSG_ID_H,     ///< Receiving message ID high byte
        SRC_ID,       ///< Receiving source device ID
        DST_ID,       ///< Receiving destination device ID
        PAYLOAD,      ///< Receiving payload bytes (repeated payload_length times)
        CRC_L,        ///< Receiving checksum low byte
        CRC_H         ///< Receiving checksum high byte (final state, validates message)
    };

    /**
     * @brief Structure holding the last message available with its state
     *
     */
    struct {
        ParserState state;      // state of incoming message processing
        bool done;              // inform if the message is complete or not
        uint8_t payload[20];    // payload
        uint16_t payload_len;   // latest message payload length
        uint16_t id;            // latest message's message id
        uint16_t payload_recv;  // number of message's payload bytes received so far
        uint16_t crc;           // latest message's crc
        uint16_t crc_expected;  // latest message's expected crc
    } msg;
};

/**
 * @class AP_RangeFinder_BLPing
 * @brief Blue Robotics Ping1D underwater acoustic rangefinder backend driver
 *
 * @details This driver interfaces with the Blue Robotics Ping1D underwater sonar sensor,
 *          which uses acoustic pulses for distance measurement in subsea environments.
 *          The sensor is optimized for ROV applications, depth measurement, obstacle
 *          detection, and underwater navigation.
 *
 *          Key characteristics:
 *          - Communication: Serial UART using proprietary Ping Protocol
 *          - Maximum update rate: 20 Hz (50ms interval)
 *          - Range: Typically 0.5m to 30m depending on water conditions
 *          - Beam width: 30° cone angle
 *          - Acoustic frequency: 115 kHz
 *
 * @warning This sensor is designed specifically for UNDERWATER operation:
 *          - Speed of sound in water (~1500 m/s) vs air (~343 m/s) affects timing
 *          - Water temperature significantly impacts speed of sound and accuracy
 *          - Suspended particles, bubbles, and turbidity affect acoustic signal quality
 *          - Marine growth or fouling on transducer degrades performance
 *          - Not suitable for air operation - use appropriate air-based rangefinders instead
 *
 * @note Sensor reports confidence/quality metrics to assess measurement reliability
 *       in varying underwater conditions (thermoclines, suspended sediment, etc.)
 */
class AP_RangeFinder_BLPing : public AP_RangeFinder_Backend_Serial
{
    /**
     * @brief Maximum sensor update rate interval
     * 
     * @details Sensor initialization and polling rate limit set to 50ms, providing maximum
     *          update frequency of 20 Hz. This rate balances measurement freshness with
     *          acoustic propagation time in water and sensor processing overhead.
     *          
     *          Units: milliseconds (ms)
     */
    static constexpr uint16_t _sensor_rate_ms = 50; // initialise sensor at no more than 20hz

public:

    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_BLPing(_state, _params);
    }

    /**
     * @brief Update sensor state by polling serial interface and processing Ping Protocol frames
     *
     * @details Called periodically by the rangefinder scheduler to:
     *          - Read available bytes from serial UART
     *          - Parse incoming Ping Protocol frames byte-by-byte
     *          - Process DISTANCE_SIMPLE messages to update range measurement
     *          - Initialize sensor if not yet configured (sends CONTINUOUS_START message)
     *          - Update sensor health status based on message reception
     *
     *          This method must be called regularly to maintain current distance readings.
     */
    void update(void) override;

    /**
     * @brief Get measurement confidence/signal quality percentage
     *
     * @details Returns the sensor's self-reported measurement quality based on acoustic
     *          return signal strength. In underwater environments, quality is affected by:
     *          - Target surface reflectivity and angle
     *          - Water turbidity and suspended particles
     *          - Acoustic noise and multipath reflections
     *          - Temperature gradients (thermoclines)
     *
     * @return Signal quality as percentage:
     *         - 100: Best quality (strong, clear acoustic return)
     *         - 0: Worst quality (weak or noisy return signal)
     *         - -1: Unknown/unavailable (sensor not reporting quality)
     */
    int8_t get_signal_quality_pct() const override WARN_IF_UNUSED;

protected:
    /**
     * @brief Return MAVLink sensor type classification for telemetry reporting
     *
     * @details Reports sensor type as ULTRASOUND for MAVLink DISTANCE_SENSOR messages.
     *          While technically an acoustic sonar operating at 115 kHz in water,
     *          MAVLink categorizes this as ultrasound for underwater acoustic rangefinders.
     *
     * @note This classification is used by ground control stations to interpret and
     *       display rangefinder data appropriately for underwater/ROV applications.
     *
     * @return MAV_DISTANCE_SENSOR_ULTRASOUND indicating underwater acoustic sonar type
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

    /**
     * @brief Sensor protocol class
     *
     */
    PingProtocol protocol;

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Do the necessary sensor initiation
     *
     */
    void init_sensor();

    /**
     * @brief Read serial interface and calculate new distance
     *
     * @param reading_m
     * @return true
     * @return false
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Timeout between messages
     *
     * @return uint16_t
     */
    uint16_t read_timeout_ms() const override { return 1000; }

    /**
     * @brief system time that sensor was last initialised
     *
     */
    uint32_t last_init_ms;
};

#endif
