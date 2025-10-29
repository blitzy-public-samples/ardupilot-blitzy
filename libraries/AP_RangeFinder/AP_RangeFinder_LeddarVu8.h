/**
 * @file AP_RangeFinder_LeddarVu8.h
 * @brief LeddarVu8 Modbus RTU serial LiDAR rangefinder backend driver
 * 
 * @details This driver implements support for the LeddarTech LeddarVu8 solid-state
 *          LiDAR sensor which uses Modbus RTU protocol over serial communication.
 *          The LeddarVu8 provides up to 8 independent detection segments in a single
 *          field of view, with this driver reporting the distance of the closest
 *          detection across all segments.
 *          
 *          Communication: Modbus RTU over serial (115200 baud, 8N1)
 *          Protocol: Request/response using Modbus function code 0x04 (Read Input Registers)
 *          Detection: Reports closest of up to 8 segment readings
 *          Update Rate: Configurable on sensor, typically 10-50 Hz
 *          
 * @note The sensor address is configured via the RNGFNDx_ADDR parameter
 * @note Timeout of 500ms is used to detect communication failures
 * 
 * @see https://leddartech.com/solutions/leddarvu8/
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_LeddarVu8.h
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LEDDARVU8_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/// Payload length in bytes for LeddarVu8 Modbus response (8 segments * 2 bytes per segment)
#define LEDDARVU8_PAYLOAD_LENGTH (8*2)

/**
 * @class AP_RangeFinder_LeddarVu8
 * @brief Rangefinder backend driver for LeddarTech LeddarVu8 Modbus serial LiDAR
 * 
 * @details This backend implements communication with the LeddarVu8 solid-state LiDAR
 *          sensor using Modbus RTU protocol over serial interface. The sensor provides
 *          8 independent detection segments within a single field of view, allowing
 *          detection of multiple objects at different distances simultaneously.
 *          
 *          The driver:
 *          - Sends periodic Modbus requests (function code 0x04) to read input registers
 *          - Parses Modbus RTU responses with CRC validation
 *          - Extracts distance measurements from all 8 segments
 *          - Reports the minimum (closest) distance across all valid segments
 *          - Handles communication timeouts and protocol errors
 *          
 *          Multi-segment operation: The LeddarVu8 can detect objects in up to 8 segments
 *          within its field of view. Each segment independently reports distance and
 *          amplitude. This driver extracts all segment distances and returns the closest
 *          valid reading for obstacle avoidance and altitude sensing.
 *          
 *          Protocol timing:
 *          - Requests sent periodically to poll for new measurements
 *          - 500ms timeout triggers NoData state if no response received
 *          - CRC-16 (Modbus) validates message integrity
 * 
 * @note Inherits serial communication handling from AP_RangeFinder_Backend_Serial
 * @note Sensor address configured via RNGFNDx_ADDR parameter (default 0x01)
 * @warning Incorrect baud rate or sensor address will prevent communication
 * 
 * @see AP_RangeFinder_Backend_Serial for base serial handling
 */
class AP_RangeFinder_LeddarVu8 : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create LeddarVu8 rangefinder backend instance
     * 
     * @details Creates a new AP_RangeFinder_LeddarVu8 object using dynamic memory
     *          allocation. This is the standard creation pattern for rangefinder
     *          backends in ArduPilot.
     * 
     * @param[in,out] _state    Reference to rangefinder state structure for storing measurements
     * @param[in]     _params   Reference to rangefinder parameters (orientation, address, etc.)
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW for safe memory allocation in resource-constrained environments
     * @note Called by rangefinder driver framework during initialization
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_LeddarVu8(_state, _params);
    }

protected:

    /**
     * @brief Get the serial communication baud rate for LeddarVu8 sensor
     * 
     * @details The LeddarVu8 uses a fixed baud rate of 115200 bps for Modbus RTU
     *          communication. This is the factory default setting for the sensor.
     *          Serial configuration: 115200 baud, 8 data bits, no parity, 1 stop bit (8N1)
     * 
     * @param[in] serial_instance Serial port instance number (unused for this sensor)
     * 
     * @return Baud rate in bits per second (always 115200)
     * 
     * @note Called during backend initialization to configure the serial port
     * @note Override from AP_RangeFinder_Backend_Serial
     */
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    /**
     * @brief Get the MAVLink distance sensor type classification for LeddarVu8
     * 
     * @details Returns the sensor type as LASER for MAVLink DISTANCE_SENSOR messages.
     *          The LeddarVu8 is a solid-state LiDAR using LED light source, classified
     *          as a laser-type distance sensor for MAVLink reporting purposes.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER sensor type classification
     * 
     * @note This classification is used in MAVLink telemetry messages
     * @note Override from AP_RangeFinder_Backend
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    /**
     * @brief Retrieve distance measurement from LeddarVu8 sensor
     * 
     * @details Implements the main measurement retrieval loop for the LeddarVu8:
     *          1. Sends periodic Modbus request to sensor (if needed)
     *          2. Reads and parses incoming serial data byte-by-byte
     *          3. Validates Modbus CRC and message structure
     *          4. Extracts distances from all 8 detection segments
     *          5. Returns the minimum (closest) valid distance
     *          
     *          The sensor can report up to 8 simultaneous detections (one per segment).
     *          This method processes all segments and returns the closest obstacle
     *          distance for safe navigation and obstacle avoidance.
     *          
     *          Communication flow:
     *          - Request sent using Modbus function 0x04 (Read Input Registers)
     *          - Response contains 8 distance values (16-bit each)
     *          - CRC-16 Modbus validates message integrity
     * 
     * @param[out] reading_m Distance measurement in meters (updated if valid reading obtained)
     * 
     * @return true if a valid distance reading was obtained, false otherwise
     * 
     * @note Called repeatedly by rangefinder framework at scheduler rate
     * @note Returns false if no data available, invalid CRC, or timeout occurred
     * @note Distance of 0 in a segment indicates no detection for that segment
     * @warning Requires RNGFNDx_ADDR parameter configured with correct sensor address
     * 
     * @see parse_byte() for Modbus message parsing details
     * @see request_distances() for Modbus request generation
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Get the maximum time between valid readings before entering NoData state
     * 
     * @details Returns the timeout period for detecting communication failures with
     *          the LeddarVu8 sensor. If no valid reading is received within this
     *          period, the rangefinder state transitions to NoData.
     *          
     *          500ms timeout allows for:
     *          - Sensor measurement cycle (10-50Hz typical)
     *          - Modbus request/response round-trip time
     *          - Multiple retry attempts
     *          - Detection of sensor disconnection or failure
     * 
     * @return Timeout period in milliseconds (500ms)
     * 
     * @note Override from AP_RangeFinder_Backend
     * @note Timeout triggers failsafe behavior in vehicle control
     */
    uint16_t read_timeout_ms() const override { return 500; }

private:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @enum FunctionCode
     * @brief Modbus RTU protocol function codes
     * 
     * @details Defines the standard Modbus function codes used for communication
     *          with the LeddarVu8 sensor. The LeddarVu8 primarily uses function
     *          code 0x04 (READ_INPUT_REGISTER) to retrieve distance measurements.
     *          
     *          This driver uses:
     *          - READ_INPUT_REGISTER (0x04): Read distance measurements and sensor status
     *          
     *          Other codes defined for potential future use or reference:
     *          - READ_HOLDING_REGISTER (0x03): Read configuration parameters
     *          - WRITE_HOLDING_REGISTER (0x06): Write single configuration register
     *          - WRITE_MULTIPLE_REGISTER (0x10): Write multiple configuration registers
     *          - READ_WRITE_MULTIPLE_REGISTER (0x17): Combined read/write operation
     * 
     * @note Function code 0x04 is used in all measurement requests
     * @note Response includes function code echo for validation
     * 
     * @see Modbus Application Protocol Specification V1.1b3 for function code details
     */
    enum class FunctionCode : uint8_t {
        READ_HOLDING_REGISTER = 0x03,        ///< Read configuration holding registers (not used)
        READ_INPUT_REGISTER = 0x04,          ///< Read input registers (used for distance measurements)
        WRITE_HOLDING_REGISTER = 0x06,       ///< Write single holding register (not used)
        WRITE_MULTIPLE_REGISTER = 0x10,      ///< Write multiple holding registers (not used)
        READ_WRITE_MULTIPLE_REGISTER = 0x17  ///< Combined read/write operation (not used)
    };

    /**
     * @enum RegisterNumber
     * @brief LeddarVu8 Modbus input register addresses for distance measurements
     * 
     * @details Defines the register addresses used to read distance measurements and
     *          sensor status from the LeddarVu8 via Modbus function code 0x04.
     *          The sensor organizes data into multiple registers containing detection
     *          information for up to 8 segments.
     *          
     *          Register layout (starting at address 1):
     *          - Status and configuration (registers 1-15)
     *          - Segment 0 first detection: distance (16), amplitude (24), flags (32)
     *          - Segments 1-7: Similar pattern at subsequent register offsets
     *          
     *          Distance registers (16-23): Distance in centimeters for each segment's
     *                                      first detection. Value of 0 = no detection.
     *          
     *          Amplitude registers (24-31): Signal amplitude * 64 for each segment.
     *                                       Higher values indicate stronger reflections.
     *          
     *          Flag registers (32-39): Detection validity flags for each segment.
     *                                  Bit 0: Valid detection
     *                                  Bit 1: Result of object demerging algorithm
     *                                  Bit 2: Reserved
     *                                  Bit 3: Signal saturated (too close or reflective)
     * 
     * @note This driver reads distance registers for all 8 segments (FIRST_DISTANCE0-7)
     * @note Amplitude and flag registers available but not currently used by driver
     * @note Additional registers exist for second/third detections per segment (unused)
     * @note Register numbers are 1-indexed per Modbus convention
     * 
     * @see LeddarVu8 User Guide for complete register map
     */
    enum class RegisterNumber : uint8_t {
        REGISTER_STATUS = 1,                    ///< Detection status: 0 = not ready, 1 = ready
        NUMBER_OF_SEGMENTS = 2,                 ///< Number of active segments (typically 8)
        NUMBER_OF_DETECTIONS = 11,              ///< Total detections across all segments
        PERCENTAGE_OF_LIGHT_SOURCE_POWER = 12,  ///< LED power percentage (0-100)
        TIMESTAMP_LOW = 14,                     ///< Measurement timestamp lower 16 bits
        TIMESTAMP_HIGH = 15,                    ///< Measurement timestamp upper 16 bits
        FIRST_DISTANCE0 = 16,                   ///< First detection distance for segment 0 (cm, 0=no detection)
        FIRST_AMPLITUDE0 = 24,                  ///< First detection amplitude * 64 for segment 0
        FIRST_FLAG0 = 32,                       ///< First detection flags for segment 0 (Bit0:Valid, Bit1:Demerged, Bit3:Saturated)
        // Additional registers for segments 1-7 follow same pattern (distance at +1, +2... amplitude at +1, +2... flags at +1, +2...)
        // Registers for second and third detections per segment also exist but are not used in this driver
    };

    /**
     * @enum ParseState
     * @brief State machine states for parsing Modbus RTU response messages
     * 
     * @details Defines the sequential states for byte-by-byte parsing of Modbus RTU
     *          responses from the LeddarVu8 sensor. The parser progresses through these
     *          states as each byte is received on the serial port, validating the
     *          message structure and extracting distance data.
     *          
     *          Modbus RTU message structure:
     *          1. Device address (1 byte)
     *          2. Function code (1 byte)
     *          3. Payload length (1 byte) - number of data bytes following
     *          4. Payload data (N bytes) - distance values for 8 segments
     *          5. CRC low byte (1 byte) - CRC-16 Modbus low byte
     *          6. CRC high byte (1 byte) - CRC-16 Modbus high byte
     *          
     *          State transitions:
     *          WAITING_FOR_ADDRESS → WAITING_FOR_FUNCTION_CODE (address matches)
     *          WAITING_FOR_FUNCTION_CODE → WAITING_FOR_PAYLOAD_LEN (function code valid)
     *          WAITING_FOR_PAYLOAD_LEN → WAITING_FOR_PAYLOAD (length valid)
     *          WAITING_FOR_PAYLOAD → WAITING_FOR_CRC_LOW (all payload received)
     *          WAITING_FOR_CRC_LOW → WAITING_FOR_CRC_HIGH
     *          WAITING_FOR_CRC_HIGH → WAITING_FOR_ADDRESS (message complete, restart)
     *          
     *          Invalid data at any stage returns to WAITING_FOR_ADDRESS.
     * 
     * @note Parser operates on single-byte input from serial port
     * @note CRC validation occurs after CRC_HIGH received
     * @note Failed validation discards message and restarts parsing
     * 
     * @see parse_byte() for state machine implementation
     */
    enum class ParseState : uint8_t {
        WAITING_FOR_ADDRESS,       ///< Waiting for device address byte (first byte)
        WAITING_FOR_FUNCTION_CODE, ///< Waiting for Modbus function code byte
        WAITING_FOR_PAYLOAD_LEN,   ///< Waiting for payload length byte
        WAITING_FOR_PAYLOAD,       ///< Accumulating payload data bytes
        WAITING_FOR_CRC_LOW,       ///< Waiting for CRC low byte
        WAITING_FOR_CRC_HIGH,      ///< Waiting for CRC high byte (final byte)
    };

    /**
     * @brief Get the Modbus device address for the LeddarVu8 sensor
     * 
     * @details Retrieves the sensor's Modbus device address from the RNGFNDx_ADDR
     *          parameter. The address is used in all Modbus requests and responses
     *          to identify the specific sensor on a multi-drop serial bus.
     *          
     *          Default LeddarVu8 address: 0x01
     *          Valid range: 0x01 to 0xF7 (1-247 decimal)
     * 
     * @return Modbus device address (1-247)
     * 
     * @note Configured via RNGFNDx_ADDR parameter in ground station
     * @note Address must match sensor's configured address or communication will fail
     * @note Multiple sensors on same bus must have unique addresses
     */
    uint8_t get_sensor_address() const;

    /**
     * @brief Send Modbus request to LeddarVu8 sensor to provide distance measurements
     * 
     * @details Constructs and transmits a Modbus RTU request message to read the
     *          distance registers from all 8 detection segments. The request uses
     *          Modbus function code 0x04 (Read Input Registers) to retrieve the
     *          distance data.
     *          
     *          Request message structure:
     *          - Device address (from RNGFNDx_ADDR parameter)
     *          - Function code: 0x04 (READ_INPUT_REGISTER)
     *          - Starting register: FIRST_DISTANCE0 (register 16)
     *          - Number of registers: 8 (one distance per segment)
     *          - CRC-16 Modbus (calculated and appended)
     *          
     *          The sensor responds with all 8 distance values in a single message.
     *          Response parsing is handled by parse_byte() called from get_reading().
     *          
     *          Request timing:
     *          - Sent periodically when new data needed
     *          - Throttled to avoid overwhelming sensor
     *          - Tracked via last_distance_request_ms
     * 
     * @note Called from get_reading() when request needed
     * @note Transmission is non-blocking via serial port
     * @note No return value - response handled asynchronously
     * 
     * @see parse_byte() for response handling
     */
    void request_distances();

    /**
     * @brief Process one byte of Modbus RTU response from serial port
     * 
     * @details Implements the Modbus RTU message parser state machine, processing
     *          incoming bytes sequentially to extract distance measurements from
     *          the LeddarVu8 sensor. Each byte advances the parser through states
     *          (address, function code, length, payload, CRC) until a complete
     *          message is received and validated.
     *          
     *          Parsing sequence:
     *          1. Match device address (validate this message is for us)
     *          2. Verify function code (should be 0x04)
     *          3. Read payload length (should be 16 bytes for 8 segments)
     *          4. Accumulate payload bytes (8 distance values, 2 bytes each)
     *          5. Receive CRC bytes (CRC-16 Modbus, low byte then high byte)
     *          6. Validate CRC against received message
     *          7. Extract distances, find minimum valid distance across all segments
     *          
     *          Distance extraction:
     *          - Each segment provides 16-bit distance in centimeters
     *          - Value of 0 indicates no detection in that segment
     *          - Driver reports minimum non-zero distance (closest obstacle)
     *          - If all segments are 0, no valid reading available
     *          
     *          Error handling:
     *          - Invalid address: restart parsing
     *          - Invalid function code: restart parsing
     *          - Invalid payload length: restart parsing
     *          - CRC mismatch: discard message, restart parsing
     *          - Timeout: handled by caller via last_distance_ms
     * 
     * @param[in]  b             Single byte received from serial port
     * @param[out] valid_reading Set to true if complete valid message with distance obtained
     * @param[out] reading_cm    Distance in centimeters (minimum across all segments)
     * 
     * @return true if a complete message was successfully parsed (valid or invalid CRC),
     *         false if still accumulating message bytes
     * 
     * @note Called repeatedly from get_reading() for each received byte
     * @note Uses parsed_msg structure to maintain parser state between calls
     * @note CRC-16 Modbus polynomial: 0xA001 (reflected/reversed)
     * @note Multi-segment detection: Returns closest obstacle distance
     * 
     * @see ParseState for state machine definition
     * @see parsed_msg for message accumulation structure
     */
    bool parse_byte(uint8_t b, bool &valid_reading, uint16_t &reading_cm);

    /**
     * @struct parsed_msg
     * @brief Structure holding Modbus RTU message parsing state and accumulated data
     * 
     * @details This structure maintains the state of the Modbus message parser and
     *          accumulates incoming bytes as they are received from the serial port.
     *          Fields are ordered to match the Modbus RTU message structure, enabling
     *          efficient CRC calculation over the entire message.
     *          
     *          Field ordering matches Modbus RTU wire format:
     *          1. address (1 byte) - included in CRC calculation
     *          2. function_code (1 byte) - included in CRC calculation
     *          3. payload_len (1 byte) - included in CRC calculation
     *          4. payload[N] (N bytes) - included in CRC calculation
     *          5. crc (2 bytes) - CRC-16 Modbus of preceding bytes
     *          
     *          The PACKED attribute ensures no padding between fields, maintaining
     *          exact correspondence with the wire format for CRC validation.
     *          
     *          Parser operation:
     *          - state: Tracks current position in message parsing
     *          - payload_recv: Counts payload bytes received so far
     *          - All fields populated sequentially by parse_byte()
     *          - CRC calculated over address through last payload byte
     *          - Structure reused for each message (not reset between messages)
     * 
     * @note PACKED ensures structure matches wire format exactly
     * @note Field order critical for CRC calculation
     * @note payload_recv and state not part of wire message
     * 
     * @see parse_byte() for usage in message parsing
     * @see ParseState for state machine values
     */
    struct PACKED {
        uint8_t address;                            ///< Modbus device address (required for CRC calculation)
        uint8_t function_code;                      ///< Modbus function code (0x04 for input registers, required for CRC)
        uint8_t payload_len;                        ///< Payload length in bytes (should be 16 for 8 segments)
        uint8_t payload[LEDDARVU8_PAYLOAD_LENGTH];  ///< Distance data for 8 segments (2 bytes per segment)
        uint16_t crc;                               ///< CRC-16 Modbus of message (received from sensor)
        uint16_t payload_recv;                      ///< Number of payload bytes received so far (parsing counter)
        ParseState state;                           ///< Current parser state machine position
    } parsed_msg;
    
    uint32_t last_distance_ms;        ///< System time (ms) of last successful distance measurement reception
    uint32_t last_distance_request_ms; ///< System time (ms) of last Modbus request sent to sensor
};

#endif  // AP_RANGEFINDER_LEDDARVU8_ENABLED
