/**
 * @file AP_RangeFinder_USD1_Serial.h
 * @brief USD1 Serial ultrasonic rangefinder driver
 * 
 * @details This driver implements support for the USD1 Serial ultrasonic rangefinder,
 *          which communicates over UART using a custom binary protocol. The driver
 *          supports two firmware versions with different data formats:
 * 
 *          **Firmware V0 (Beta)** - 3-byte protocol:
 *          - Byte 0: Header (0x48)
 *          - Bytes 1-2: 14-bit distance value (7 bits per byte, MSB masked)
 *          - No checksum validation
 *          - Resolution: 2.5 cm per count
 *          - Header recurs every 3 bytes to maintain synchronization
 * 
 *          **Firmware V1+** - 6-byte protocol:
 *          - Byte 0: Header (0xFE)
 *          - Byte 1: Firmware version number
 *          - Bytes 2-3: 16-bit distance value (little-endian)
 *          - Byte 4: Reserved/unused
 *          - Byte 5: Checksum (sum of bytes 1-4, lower 8 bits)
 *          - Resolution: 1 cm per count
 * 
 *          **Serial Configuration**:
 *          - Baudrate: 115200
 *          - Data format: 8N1 (8 data bits, no parity, 1 stop bit)
 *          - Buffer size: 128 bytes RX/TX
 * 
 *          **Auto-Detection**: The driver automatically detects the firmware version
 *          from the serial data stream by analyzing header byte patterns and data
 *          structure, allowing seamless support for both protocol versions.
 * 
 * @note Reports as MAV_DISTANCE_SENSOR_RADAR type to MAVLink ground stations
 * @see AP_RangeFinder_Backend_Serial for serial rangefinder base functionality
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_USD1_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_USD1_Serial
 * @brief USD1 Serial ultrasonic rangefinder backend driver
 * 
 * @details This class implements the serial protocol parser for USD1 ultrasonic
 *          rangefinders. It handles automatic firmware version detection and
 *          provides distance measurements by parsing binary serial data.
 * 
 *          **Operation**:
 *          1. On first data reception, auto-detects firmware version (V0 or V1+)
 *          2. Parses incoming serial data according to detected protocol format
 *          3. Validates checksums (V1+ only) and averages multiple readings
 *          4. Converts raw counts to meters with version-specific scaling
 * 
 *          **Protocol Handling**:
 *          - Synchronizes to header bytes in serial stream
 *          - Buffers complete messages before parsing
 *          - Averages all available readings in update cycle for noise reduction
 *          - Handles mixed V0/V1 detection logic for backward compatibility
 * 
 *          **Thread Safety**: Called from scheduler thread, uses UART driver
 *          thread-safe methods for serial I/O
 * 
 * @note Inherits serial port management and update timing from Backend_Serial
 * @see AP_RangeFinder_Backend_Serial for serial communication infrastructure
 * @see detect_version() for firmware version auto-detection algorithm
 */
class AP_RangeFinder_USD1_Serial : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create USD1 Serial rangefinder backend instance
     * 
     * @details Creates a new USD1 Serial driver instance using dynamic memory
     *          allocation. This factory pattern allows the rangefinder framework
     *          to instantiate backend drivers without compile-time coupling.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing
     *                       distance measurements and sensor health status
     * @param[in] _params Reference to configuration parameters including serial
     *                    port selection, orientation, and sensor limits
     * 
     * @return Pointer to new AP_RangeFinder_USD1_Serial instance, or nullptr if
     *         memory allocation fails
     * 
     * @note Uses NEW_NOTHROW for safe allocation in resource-constrained environments
     * @see AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial() for
     *      backend initialization
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_USD1_Serial(_state, _params);
    }

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the sensor type reported to ground control stations via
     *          MAVLink telemetry. The USD1 Serial is reported as RADAR type
     *          rather than ULTRASOUND due to its ultrasonic ranging principle
     *          and typical use cases.
     * 
     * @return MAV_DISTANCE_SENSOR_RADAR MAVLink distance sensor type enumeration
     * 
     * @note This classification affects how ground stations display and process
     *       the sensor data. RADAR type is used for consistency with similar
     *       serial ultrasonic rangefinders in the ArduPilot ecosystem.
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    /**
     * @brief Get initial serial port baudrate for USD1 sensor communication
     * 
     * @details Returns the fixed baudrate required by USD1 Serial rangefinders.
     *          This baudrate is applied during serial port initialization and
     *          must match the sensor's factory configuration.
     * 
     * @param[in] serial_instance Serial port instance number (unused, included
     *                            for interface compatibility)
     * 
     * @return 115200 Fixed baudrate in bits per second
     * 
     * @note The USD1 Serial sensor uses a fixed baudrate of 115200 baud with
     *       8N1 format (8 data bits, no parity, 1 stop bit). This baudrate
     *       cannot be changed without reprogramming the sensor firmware.
     * @warning Incorrect baudrate will result in garbled data and sensor failure
     */
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    /**
     * @brief Get required receive buffer size for serial port
     * 
     * @details Specifies the UART receive buffer size needed to handle incoming
     *          USD1 Serial data without loss. The 128-byte buffer provides
     *          sufficient buffering for multiple complete messages between
     *          update cycles at 115200 baud.
     * 
     * @return 128 Receive buffer size in bytes
     * 
     * @note Buffer sizing: At 115200 baud (~11520 bytes/sec), the sensor can
     *       transmit ~21 complete V1 messages (6 bytes each) or ~64 V0 messages
     *       (3 bytes each) per 50ms update cycle. 128 bytes provides headroom
     *       for timing jitter and prevents data loss.
     */
    uint16_t rx_bufsize() const override { return 128; }
    
    /**
     * @brief Get required transmit buffer size for serial port
     * 
     * @details Specifies the UART transmit buffer size. The USD1 Serial sensor
     *          is receive-only (no commands sent to sensor), but a minimal
     *          buffer is allocated for consistency with serial driver interface.
     * 
     * @return 128 Transmit buffer size in bytes
     * 
     * @note The USD1 Serial protocol is unidirectional (sensor to autopilot),
     *       so the TX buffer is not used during normal operation. The allocation
     *       prevents potential issues with zero-size buffer edge cases.
     */
    uint16_t tx_bufsize() const override { return 128; }

private:

    /**
     * @brief Inherit parent class constructors
     * 
     * @details Uses parent class AP_RangeFinder_Backend_Serial constructor for
     *          object initialization. This allows the factory create() method
     *          to construct instances using the standard backend constructor
     *          signature without defining a redundant constructor in this class.
     */
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Auto-detect USD1 Serial firmware version from data stream
     * 
     * @details Analyzes incoming serial data to determine which protocol version
     *          the connected USD1 sensor is using. Detection algorithm:
     * 
     *          **V0 Detection (header 0x48)**:
     *          1. Search for 0x48 header byte
     *          2. Collect next 3 bytes
     *          3. If 4th byte is also 0x48 (recurring header), identify as V0
     * 
     *          **V1+ Detection (header 0xFE)**:
     *          1. Search for 0xFE header byte
     *          2. Verify next byte is NOT 0x80+ (would indicate V0 data byte)
     *          3. Verify next byte is NOT 0x48 (would indicate V0 header)
     *          4. Accept byte as version number, identify as V1+
     * 
     *          Detection runs continuously until version is determined, then
     *          result is cached in _version_known flag. Processes up to 8192
     *          bytes per call to handle worst-case synchronization scenarios.
     * 
     * @return true if firmware version successfully detected and stored in
     *         _version and _header, false if more data needed or UART unavailable
     * 
     * @note This method is called before every get_reading() attempt until version
     *       is known. Once detected, returns immediately with cached result.
     * @note The detection algorithm handles edge cases where V1 header (0xFE)
     *       might appear in V0 data bytes, ensuring robust version identification.
     * @warning UART must be initialized before calling (checked via uart != nullptr)
     * 
     * @see get_reading() for usage of detected version in protocol parsing
     */
    bool detect_version(void);

    /**
     * @brief Parse USD1 Serial data and extract distance measurement
     * 
     * @details Reads available serial data and parses according to detected
     *          protocol version. Processing steps:
     * 
     *          1. **Version Detection**: Calls detect_version() until firmware
     *             version is identified
     * 
     *          2. **Message Synchronization**: Searches serial stream for header
     *             byte (0x48 for V0, 0xFE for V1+)
     * 
     *          3. **Data Buffering**: Accumulates message bytes in _linebuf until
     *             complete message received (3 bytes for V0, 6 bytes for V1+)
     * 
     *          4. **Parsing and Validation**:
     *             - **V0**: Extract 14-bit distance from two 7-bit data bytes
     *             - **V1+**: Validate checksum, then extract 16-bit little-endian
     *               distance value
     * 
     *          5. **Averaging**: Accumulates all valid readings in current update
     *             cycle and returns average for noise reduction
     * 
     *          6. **Unit Conversion**:
     *             - V0: Raw counts × 2.5 cm → meters (0.025 m per count)
     *             - V1+: Raw counts × 1 cm → meters (0.01 m per count)
     * 
     *          Processes up to 8192 bytes per call to handle message backlog
     *          and ensure minimal latency.
     * 
     * @param[out] reading_m Distance measurement in meters. Valid only if return
     *                       value is true. Contains averaged value if multiple
     *                       readings available.
     * 
     * @return true if valid distance measurement obtained and stored in reading_m,
     *         false if no data available, version unknown, UART unavailable, or
     *         no valid messages parsed
     * 
     * @note Called by base class update() method at scheduler rate (typically
     *       10-50 Hz depending on vehicle configuration)
     * @note Averaging multiple readings per update cycle improves measurement
     *       stability in the presence of sensor noise or echo interference
     * @note V1+ protocol checksum validation prevents corrupted measurements
     *       from propagating to flight control algorithms
     * 
     * @warning Invalid data due to electrical noise or baud rate mismatch will
     *          cause this method to return false, triggering sensor timeout
     *          and potential failsafe actions
     * 
     * @see detect_version() for firmware version identification
     * @see AP_RangeFinder_Backend_Serial::update() for update cycle timing
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Message buffer for accumulating incoming serial data
     * 
     * @details Temporary buffer for assembling complete protocol messages from
     *          the serial stream. Buffer size of 6 bytes accommodates the largest
     *          message format (V1+ protocol). For V0 protocol, only first 3 bytes
     *          are used.
     * 
     * @note Buffer is reset (via _linebuf_len) when header byte is detected,
     *       ensuring synchronization to message boundaries
     */
    uint8_t  _linebuf[6];
    
    /**
     * @brief Current number of bytes accumulated in message buffer
     * 
     * @details Tracks how many bytes have been stored in _linebuf since the last
     *          header detection. Used to determine when a complete message has
     *          been received and is ready for parsing.
     * 
     * @note Range: 0-6 bytes. Reset to 0 when header byte detected or after
     *       message parsing completes.
     */
    uint8_t  _linebuf_len;
    
    /**
     * @brief Flag indicating whether firmware version has been detected
     * 
     * @details Set to true once detect_version() successfully identifies the
     *          USD1 sensor firmware version. Used to skip version detection
     *          on subsequent update cycles for performance optimization.
     * 
     * @note Initialized to false (via default constructor). Once set true,
     *       remains true for lifetime of driver instance.
     */
    bool     _version_known;
    
    /**
     * @brief Detected protocol header byte value
     * 
     * @details Stores the header byte value for the detected firmware version:
     *          - 0x48 (72 decimal): Firmware V0 (beta) protocol
     *          - 0xFE (254 decimal): Firmware V1+ protocol
     * 
     *          Used during message parsing to synchronize to message boundaries
     *          in the serial data stream.
     * 
     * @note Only valid after _version_known is set to true
     * @see detect_version() for header detection algorithm
     */
    uint8_t  _header;
    
    /**
     * @brief Detected firmware version number
     * 
     * @details Stores the firmware version number extracted from the sensor:
     *          - 0: Firmware V0 (beta) - uses 3-byte format, no checksum
     *          - 1+: Firmware V1 and later - uses 6-byte format with checksum
     * 
     *          Version number affects:
     *          - Message length (3 vs 6 bytes)
     *          - Parsing algorithm (simple vs checksum-validated)
     *          - Unit scaling (2.5 cm vs 1 cm per count)
     * 
     * @note Only valid after _version_known is set to true
     * @note For V0 detection, this is set to 0 explicitly rather than extracted
     *       from data (V0 protocol has no version byte)
     * @see get_reading() for version-specific parsing logic
     */
    uint8_t  _version;
};

#endif  // AP_RANGEFINDER_USD1_SERIAL_ENABLED
