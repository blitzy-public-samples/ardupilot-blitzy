/**
 * @file AP_RangeFinder_LeddarOne.h
 * @brief LeddarOne serial LiDAR rangefinder backend driver
 * 
 * @details This file implements support for the LeddarOne industrial-grade
 *          solid-state LiDAR sensor from LeddarTech. The LeddarOne communicates
 *          via serial interface using the Modbus RTU protocol.
 * 
 *          Key Features:
 *          - Maximum range: 40 meters
 *          - Industrial-grade reliability and accuracy
 *          - Modbus RTU communication protocol
 *          - Serial interface (typically UART)
 *          - Multiple detection support (up to 3 simultaneous detections)
 *          - Narrow beam angle for precise measurements
 * 
 *          The driver implements the Modbus function code 0x04 (Read Input Registers)
 *          to retrieve distance measurements from the sensor. It includes CRC16
 *          validation for data integrity and supports the sensor's multi-detection
 *          capability.
 * 
 * @note This driver requires AP_RANGEFINDER_LEDDARONE_ENABLED to be defined
 * @see AP_RangeFinder_Backend_Serial for base serial rangefinder functionality
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LEDDARONE_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/// @defgroup LeddarOne_Protocol LeddarOne Modbus RTU Protocol Constants
/// @{

/// Default Modbus device address for LeddarOne sensor
#define LEDDARONE_DEFAULT_ADDRESS 0x01

/// Modbus function code 0x04: Read Input Registers
#define LEDDARONE_MODOBUS_FUNCTION_CODE 0x04

/// Starting register address for distance measurement data (register 20)
#define LEDDARONE_MODOBUS_FUNCTION_REGISTER_ADDRESS 20

/// Number of consecutive registers to read (10 registers)
#define LEDDARONE_MODOBUS_FUNCTION_READ_NUMBER 10

/// Maximum serial port identifier value
#define LEDDARONE_SERIAL_PORT_MAX 250

/// Size of read buffer for Modbus response (25 bytes)
#define LEDDARONE_READ_BUFFER_SIZE 25

/// @}

/// @defgroup LeddarOne_Detection LeddarOne Detection Data Constants
/// @{

/// Maximum number of simultaneous detections supported (3 detections)
#define LEDDARONE_DETECTIONS_MAX 3

/// Index in response buffer where number of detections is stored
#define LEDDARONE_DETECTION_DATA_NUMBER_INDEX 10

/// Offset to detection data index within response
#define LEDDARONE_DETECTION_DATA_INDEX_OFFSET 11

/// Byte offset between detection data fields
#define LEDDARONE_DETECTION_DATA_OFFSET 4

/// @}

/**
 * @enum LeddarOne_Status
 * @brief Status codes for LeddarOne sensor operations
 * 
 * @details These status codes indicate the result of communication and data
 *          parsing operations with the LeddarOne sensor. Positive values indicate
 *          success or in-progress states, while negative values indicate various
 *          error conditions that may occur during Modbus communication.
 */
enum LeddarOne_Status {
    /// Successful operation - valid data received and parsed
    LEDDARONE_STATE_OK = 0,
    
    /// Currently reading data from serial buffer (in-progress state)
    LEDDARONE_STATE_READING_BUFFER = 1,
    
    /// CRC16 checksum validation failed - data corruption detected
    LEDDARONE_STATE_ERR_BAD_CRC = -1,
    
    /// No response received from sensor within timeout period
    LEDDARONE_STATE_ERR_NO_RESPONSES = -2,
    
    /// Response received but format is invalid or unexpected
    LEDDARONE_STATE_ERR_BAD_RESPONSE = -3,
    
    /// Response packet is shorter than expected minimum length
    LEDDARONE_STATE_ERR_SHORT_RESPONSE = -4,
    
    /// Serial port communication error (hardware or configuration issue)
    LEDDARONE_STATE_ERR_SERIAL_PORT = -5,
    
    /// Number of detections exceeds maximum supported (LEDDARONE_DETECTIONS_MAX)
    LEDDARONE_STATE_ERR_NUMBER_DETECTIONS = -6
};

/**
 * @enum LeddarOne_ModbusStatus
 * @brief State machine states for Modbus RTU communication protocol
 * 
 * @details This enumeration tracks the current state of the Modbus communication
 *          state machine. The driver cycles through these states to implement
 *          proper request-response timing and ensure reliable communication with
 *          the LeddarOne sensor.
 * 
 *          State Flow: INIT → PRE_SEND_REQUEST → SENT_REQUEST → AVAILABLE → (repeat)
 */
enum LeddarOne_ModbusStatus {
    /// Initial state after driver initialization
    LEDDARONE_MODBUS_STATE_INIT = 0,
    
    /// Preparing to send Modbus request to sensor
    LEDDARONE_MODBUS_STATE_PRE_SEND_REQUEST,
    
    /// Modbus request has been transmitted, waiting for response
    LEDDARONE_MODBUS_STATE_SENT_REQUEST,
    
    /// Response data is available for processing
    LEDDARONE_MODBUS_STATE_AVAILABLE
};

/**
 * @class AP_RangeFinder_LeddarOne
 * @brief Driver for LeddarOne industrial-grade serial LiDAR rangefinder
 * 
 * @details This class implements the backend driver for the LeddarOne solid-state
 *          LiDAR sensor manufactured by LeddarTech. The sensor provides accurate
 *          distance measurements up to 40 meters and communicates via serial
 *          interface using the Modbus RTU protocol.
 * 
 *          Features:
 *          - Industrial-grade reliability and accuracy
 *          - Maximum range: 40 meters
 *          - Modbus RTU protocol communication
 *          - Multi-detection capability (up to 3 simultaneous detections)
 *          - CRC16 data integrity validation
 *          - Average distance calculation from multiple detections
 * 
 *          Communication Protocol:
 *          The driver implements Modbus function code 0x04 (Read Input Registers)
 *          to poll the sensor for distance measurements. The sensor can return
 *          multiple detection targets, and the driver averages these for the
 *          final distance reading.
 * 
 *          Hardware Connection:
 *          - Serial interface (UART)
 *          - Default baud rate: Configured via RangeFinder parameters
 *          - Modbus address: 0x01 (default)
 * 
 * @note This is a polling-based driver that sends periodic Modbus requests
 * @warning Ensure proper serial port configuration and termination for Modbus RTU
 * 
 * @see AP_RangeFinder_Backend_Serial for base serial rangefinder functionality
 * @see Modbus RTU specification for protocol details
 */
class AP_RangeFinder_LeddarOne : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create LeddarOne rangefinder instance
     * 
     * @details Creates a new instance of the LeddarOne driver with dynamic memory
     *          allocation. This method is called by the rangefinder detection and
     *          initialization system.
     * 
     * @param[in,out] _state     Reference to rangefinder state structure
     * @param[in]     _params    Reference to rangefinder parameters
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW for safe memory allocation without exceptions
     * @see AP_RangeFinder_Backend_Serial::create
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_LeddarOne(_state, _params);
    }

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink sensor type classification for telemetry reporting.
     *          The LeddarOne is classified as a laser rangefinder for MAVLink protocol.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser-based distance measurement
     * 
     * @note This method is used by the MAVLink telemetry system to properly identify
     *       the sensor type to ground control stations
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    /// Inherit constructor from base serial backend class
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Get distance reading from LeddarOne sensor
     * 
     * @details Implements the main measurement loop for the LeddarOne driver.
     *          This method manages the Modbus state machine, sends periodic
     *          requests to the sensor, receives and parses responses, and
     *          calculates the final distance measurement.
     * 
     *          The method:
     *          1. Manages Modbus state machine (INIT → PRE_SEND → SENT → AVAILABLE)
     *          2. Sends Modbus Read Input Registers request (function code 0x04)
     *          3. Reads response from serial buffer
     *          4. Validates CRC16 checksum
     *          5. Parses detection data (supports up to 3 detections)
     *          6. Averages multiple detections for final reading
     * 
     * @param[out] reading_m Distance measurement in meters
     * 
     * @return true if valid reading obtained, false on error or no data
     * 
     * @note Called periodically by the rangefinder update loop
     * @note Accumulates multiple detections and returns average distance
     * @warning Returns false if CRC validation fails or communication errors occur
     * 
     * @see parse_response for response parsing details
     * @see CRC16 for data integrity validation
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Validate or calculate CRC16 checksum for Modbus RTU
     * 
     * @details Implements CRC16 checksum calculation and validation according to
     *          the Modbus RTU specification. The CRC is calculated over the entire
     *          message except the CRC bytes themselves. This ensures data integrity
     *          during serial communication.
     * 
     *          Algorithm: Modbus CRC16 (polynomial 0xA001, initial value 0xFFFF)
     * 
     * @param[in,out] aBuffer Pointer to data buffer containing message
     * @param[in]     aLength Length of data to calculate CRC over (excluding CRC bytes)
     * @param[in]     aCheck  true to validate existing CRC, false to calculate and append
     * 
     * @return true if CRC is valid (aCheck=true) or calculation successful (aCheck=false)
     * @return false if CRC validation fails
     * 
     * @note When aCheck=false, the calculated CRC is appended to the buffer
     * @note When aCheck=true, the CRC in the buffer is validated against calculated value
     * @warning Buffer must have space for 2 CRC bytes when aCheck=false
     * 
     * @see Modbus RTU specification for CRC16 algorithm details
     */
    bool CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck);

    /**
     * @brief Parse Modbus response and extract detection data
     * 
     * @details Parses the Modbus response packet from the LeddarOne sensor and
     *          extracts distance detection information. The sensor can return up
     *          to 3 simultaneous detections, which are processed and accumulated
     *          for averaging.
     * 
     *          Response Format:
     *          - Modbus header (device address, function code)
     *          - Byte count
     *          - Register data (10 registers = 20 bytes)
     *          - Number of detections (at index 10)
     *          - Detection data for each detection (distance, amplitude)
     *          - CRC16 checksum (2 bytes)
     * 
     *          Parsing Steps:
     *          1. Validate response length
     *          2. Verify Modbus function code
     *          3. Extract number of detections
     *          4. Parse distance data for each detection
     *          5. Accumulate distances in sum_distance_mm for averaging
     * 
     * @param[out] number_detections Number of valid detections found in response
     * 
     * @return LEDDARONE_STATE_OK on successful parse
     * @return LEDDARONE_STATE_ERR_SHORT_RESPONSE if packet too short
     * @return LEDDARONE_STATE_ERR_BAD_RESPONSE if invalid Modbus response
     * @return LEDDARONE_STATE_ERR_NUMBER_DETECTIONS if too many detections
     * 
     * @note Updates sum_distance_mm member variable with accumulated distances
     * @note Detection data includes both distance and signal amplitude
     * @warning Returns error if number of detections exceeds LEDDARONE_DETECTIONS_MAX (3)
     * 
     * @see get_reading for caller context and averaging logic
     */
    LeddarOne_Status parse_response(uint8_t &number_detections);

    /// Timestamp of last Modbus request transmission (milliseconds)
    uint32_t last_sending_request_ms;
    
    /// Timestamp when response data became available (milliseconds)
    uint32_t last_available_ms;

    /// Accumulated distance sum from multiple detections (millimeters)
    /// @note Divided by number of detections to get average distance
    uint32_t sum_distance_mm;

    /// Current state of Modbus communication state machine
    LeddarOne_ModbusStatus modbus_status = LEDDARONE_MODBUS_STATE_INIT;
    
    /// Buffer for receiving Modbus response packets
    uint8_t read_buffer[LEDDARONE_READ_BUFFER_SIZE];
    
    /// Number of bytes currently stored in read_buffer
    uint32_t read_len;

    /**
     * @brief Pre-calculated Modbus request buffer for reading distance data
     * 
     * @details This buffer contains a complete Modbus RTU request packet for
     *          function code 0x04 (Read Input Registers). The request reads
     *          10 consecutive registers starting from register 20, which contains
     *          the distance detection data from the LeddarOne sensor.
     * 
     *          Buffer Structure:
     *          [0] Device address (0x01)
     *          [1] Function code (0x04 - Read Input Registers)
     *          [2] Register address high byte (0x00)
     *          [3] Register address low byte (0x14 = 20 decimal)
     *          [4] Number of registers high byte (0x00)
     *          [5] Number of registers low byte (0x0A = 10 decimal)
     *          [6] CRC low byte (0x30)
     *          [7] CRC high byte (0x09)
     * 
     * @note CRC is pre-calculated for this fixed request (0x0930)
     * @note Request is sent repeatedly to poll sensor for new measurements
     */
    const uint8_t send_request_buffer[8] = {
        LEDDARONE_DEFAULT_ADDRESS,                      // [0] Modbus device address
        LEDDARONE_MODOBUS_FUNCTION_CODE,                // [1] Function code 0x04
        0,                                              // [2] Register address high byte
        LEDDARONE_MODOBUS_FUNCTION_REGISTER_ADDRESS,   // [3] Start at register 20
        0,                                              // [4] Number of registers high byte
        LEDDARONE_MODOBUS_FUNCTION_READ_NUMBER,        // [5] Read 10 registers
        0x30,                                           // [6] CRC low byte
        0x09                                            // [7] CRC high byte
    };
};

#endif  // AP_RANGEFINDER_LEDDARONE_ENABLED
