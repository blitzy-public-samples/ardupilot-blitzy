/**
 * @file AP_OpticalFlow_UPFLOW.h
 * @brief UPixel UPFLOW UART optical flow sensor backend
 * 
 * @details Hardware: UPixel UPFLOW optical flow module with UART interface
 *          Protocol: 14-byte UART packet format
 *                    - Header: 0xFE 0x0A (2 bytes)
 *                    - Payload: 12 bytes (UpixelsOpticalFlow structure)
 *                    - Checksum: XOR of all payload bytes (1 byte)
 *          Baud rate: Typically 115200 8N1
 *          
 *          Packet format: [0xFE] [0x0A] [12-byte payload] [XOR checksum]
 *          
 *          This backend implements a framing state machine that:
 *          - Detects header bytes (0xFE, 0x0A)
 *          - Accumulates 12-byte payload
 *          - Validates XOR checksum
 *          - Decodes flow integrals and integration time
 *          - Validates quality flag (245 = valid, 0 = invalid)
 * 
 * @note Flow integrals are scaled by UPFLOW_PIXEL_SCALING (1e-4 rad)
 * @note Quality metric is binary: 245 (valid) or 0 (invalid)
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_UPFLOW_ENABLED

#include "AP_OpticalFlow_Backend.h"
#include <AP_HAL/utility/OwnPtr.h>

/**
 * @class AP_OpticalFlow_UPFLOW
 * @brief UPixel UPFLOW UART optical flow sensor backend
 * 
 * @details This backend parses UPFLOW 14-byte protocol packets received over UART.
 *          
 *          Protocol Implementation:
 *          - Implements framing state machine with header detection (0xFE 0x0A)
 *          - Validates packets using XOR checksum of payload bytes
 *          - Decodes flow_x_integral and flow_y_integral (signed 16-bit values)
 *          - Extracts integration_timespan (microseconds) for dt calculation
 *          - Validates quality flag (245 = valid, 0 = invalid)
 *          
 *          Data Processing:
 *          - Flow integrals scaled by UPFLOW_PIXEL_SCALING (1e-4 rad)
 *          - Averages gyro samples between flow updates for velocity compensation
 *          - Computes dt from integration_timespan (converted from microseconds)
 *          - Converts integrals to flow rates (rad/s): integral * 1e-4 / dt
 *          - Rejects stale data via timeout detection
 *          
 *          Output:
 *          - Flow rates in rad/s (sensor body frame)
 *          - Surface quality indication (binary: valid/invalid)
 *          - Integration time for accurate velocity computation
 * 
 * @note Quality metric differs from other sensors: 245 = valid, 0 = invalid
 *       (not a 0-255 continuous scale)
 * @note ground_distance field is reserved and always 999 (not used by firmware)
 * 
 * @warning XOR checksum validation is required - corrupt packets are rejected
 * @warning Surface texture and lighting requirements affect flow reliability
 * @warning Timeout detection via dt prevents stale data from being used
 */
class AP_OpticalFlow_UPFLOW : public OpticalFlow_backend
{
public:
    /**
     * @brief Initialize UPFLOW backend with frontend manager and UART device
     * 
     * @param[in] _frontend Reference to AP_OpticalFlow manager
     * @param[in] uart Pointer to UART device for serial communication with sensor
     */
    AP_OpticalFlow_UPFLOW(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *uart);

    /**
     * @brief Configure UART and reset packet framing state
     * 
     * @details Initializes UART communication (typically 115200 baud 8N1)
     *          and resets the packet framing state machine to await header bytes.
     *          Clears recv_count and sum for fresh packet reception.
     */
    void init() override;

    /**
     * @brief Read UART bytes, parse packets, validate and process flow data
     * 
     * @details Main update loop that processes incoming UART data:
     *          
     *          Packet Reception:
     *          1. Read available UART bytes
     *          2. Detect 14-byte packet with header 0xFE 0x0A
     *          3. Accumulate 12-byte payload and compute XOR checksum
     *          4. Validate checksum against received checksum byte
     *          
     *          Data Extraction (on valid packet):
     *          - flow_x_integral: Signed 16-bit value (units: 10^-4 radians)
     *          - flow_y_integral: Signed 16-bit value (units: 10^-4 radians)
     *          - integration_timespan: Microseconds for dt calculation
     *          - quality: Binary flag (245 = valid, 0 = invalid)
     *          
     *          Processing:
     *          - Average accumulated gyro samples for compensation
     *          - Compute dt from integration_timespan (convert μs to seconds)
     *          - Convert integrals to rates: rate = integral * UPFLOW_PIXEL_SCALING / dt
     *          - Where UPFLOW_PIXEL_SCALING = 1e-4 (converts 10^-4 rad units to radians)
     *          - Call _update_frontend() with flow rates (rad/s) and surface quality
     *          
     *          Error Handling:
     *          - Invalid checksum: Packet rejected, framing state reset
     *          - Invalid quality (not 245): Data rejected
     *          - Timeout: Stale data detected via dt check
     * 
     * @note Called at main loop rate to process available UART data
     * @note Gyro averaging provides velocity compensation for accurate flow
     * @warning Checksum failures indicate corrupt data or sync loss
     */
    void update(void) override;

    /**
     * @brief Static factory method to detect and instantiate UPFLOW sensor
     * 
     * @details Searches available serial ports via AP_SerialManager for
     *          UPFLOW optical flow sensor. If a compatible UART port is
     *          configured for optical flow use, creates and returns a new
     *          AP_OpticalFlow_UPFLOW instance.
     * 
     * @param[in] _frontend Reference to AP_OpticalFlow manager
     * 
     * @return Pointer to new AP_OpticalFlow_UPFLOW instance if sensor detected,
     *         nullptr if no compatible UART port found
     * 
     * @note Called during optical flow subsystem initialization
     */
    static AP_OpticalFlow_UPFLOW *detect(AP_OpticalFlow &_frontend);

private:
    /**
     * @struct UpixelsOpticalFlow
     * @brief PACKED 12-byte payload structure for UPFLOW sensor data
     * 
     * @details This structure represents the 12-byte payload within the
     *          14-byte UPFLOW packet (excluding 2-byte header and 1-byte checksum).
     *          All multi-byte fields are little-endian.
     *          
     *          Protocol specification:
     *          - Total packet: [0xFE] [0x0A] [12-byte payload] [XOR checksum]
     *          - Payload: This structure (PACKED, no padding)
     *          - Checksum: XOR of all 12 payload bytes
     */
    struct PACKED UpixelsOpticalFlow {
        /**
         * @brief Signed X-axis optical flow integral
         * 
         * Units: 10^-4 radians (multiply by 1e-4 to convert to radians)
         * Range: -32768 to +32767 (raw)
         *        -3.2768 to +3.2767 radians (scaled)
         */
        int16_t		flow_x_integral;
        
        /**
         * @brief Signed Y-axis optical flow integral
         * 
         * Units: 10^-4 radians (multiply by 1e-4 to convert to radians)
         * Range: -32768 to +32767 (raw)
         *        -3.2768 to +3.2767 radians (scaled)
         */
        int16_t		flow_y_integral;
        
        /**
         * @brief Integration period for flow measurement
         * 
         * Units: microseconds (μs)
         * Usage: Compute dt = integration_timespan / 1e6 (convert to seconds)
         * Purpose: Required for converting flow integrals to flow rates (rad/s)
         */
        uint16_t   	integration_timespan;
        
        /**
         * @brief Ground distance field (reserved, not used)
         * 
         * Value: Always 999
         * Status: Reserved by sensor firmware, not populated with actual distance
         * Note: Do not use for height estimation
         */
        uint16_t   	ground_distance;
        
        /**
         * @brief Flow measurement quality indicator
         * 
         * Values: 245 = valid flow measurement
         *         0 = invalid flow measurement
         * Note: Binary quality metric (not 0-255 continuous scale)
         * Usage: Reject data when quality != 245
         */
        uint8_t    	quality;
        
        /**
         * @brief Protocol version identifier
         * 
         * Purpose: Identifies sensor firmware protocol version
         */
        uint8_t    	version;
    };
    /**
     * @brief UART device pointer for serial communication with sensor
     * 
     * Configured via AP_SerialManager to appropriate serial port.
     * Typically runs at 115200 baud, 8N1 configuration.
     */
    AP_HAL::UARTDriver *uart;
    
    /**
     * @brief Received packet payload structure
     * 
     * Stores the 12-byte payload extracted from incoming UPFLOW packets.
     * Populated when complete packet with valid checksum is received.
     */
    struct UpixelsOpticalFlow updata;
    
    /**
     * @brief Framing state machine byte counter
     * 
     * Tracks position within 14-byte packet reception:
     * - 0-1: Header bytes (0xFE, 0x0A)
     * - 2-13: Payload bytes (12 bytes of UpixelsOpticalFlow structure)
     * - 14: XOR checksum byte
     * 
     * Reset to 0 when packet complete or framing error detected.
     */
    uint16_t recv_count;
    
    /**
     * @brief Running XOR checksum accumulator
     * 
     * Computed as XOR of all 12 payload bytes during reception.
     * Compared against received checksum byte (byte 14) for validation.
     * Reset to 0 at start of each packet.
     */
    uint8_t sum;
    
    /**
     * @brief Sum of gyro sensor values for velocity compensation
     * 
     * Accumulates gyro samples (rad/s) between flow sensor updates.
     * Averaged and used to compensate flow measurements for vehicle rotation.
     * Reset after each flow update is processed.
     */
    Vector2f gyro_sum;
    
    /**
     * @brief Number of gyro samples accumulated in gyro_sum
     * 
     * Used to compute average gyro: gyro_avg = gyro_sum / gyro_sum_count
     * Prevents division by zero and ensures accurate averaging.
     * Reset after each flow update is processed.
     */
    uint16_t gyro_sum_count;
};

#endif // AP_OPTICALFLOW_UPFLOW_ENABLED
