/**
 * @file AP_OpticalFlow_CXOF.h
 * @brief Cheerson CX-OF UART-based optical flow sensor backend
 * 
 * @details This backend implements support for the Cheerson CX-OF optical flow sensor,
 *          which communicates via UART using a proprietary binary protocol.
 * 
 * Protocol: 9-byte framed packets transmitted at 115200 baud (8N1)
 * Data format: Integral pixel motion (x/y) with quality metric (0-255)
 * Typical update rate: ~10Hz
 * Pixel scaling: 1.76e-3 rad/pixel (CXOF_PIXEL_SCALING)
 * 
 * The sensor provides frame-to-frame pixel motion which is converted to
 * angular flow rates (rad/s) and integrated with gyro data for velocity
 * compensation.
 * 
 * @warning Reliable flow detection requires adequate surface texture and
 *          lighting conditions. Performance degrades over uniform surfaces
 *          or in low-light environments.
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_CXOF_ENABLED

#include "AP_OpticalFlow_Backend.h"

/**
 * @class AP_OpticalFlow_CXOF
 * @brief Cheerson CX-OF optical flow sensor backend for UART communication
 * 
 * @details Implements the protocol parser for CX-OF 9-byte packet format.
 *          
 *          Key functionality:
 *          - Parses incoming UART byte stream for 9-byte framed packets
 *          - Validates packet framing and checksums
 *          - Extracts flow_x, flow_y (pixel motion), and quality metric
 *          - Accumulates gyro samples between frames for velocity compensation
 *          - Converts pixel motion to angular flow rates using CXOF_PIXEL_SCALING (1.76e-3 rad/pixel)
 *          - Computes inter-frame time delta (dt) for rate calculations
 *          - Detects sensor timeout if no valid frames received
 *          - Updates frontend via _update_frontend() with processed flow data
 *          
 *          Packet format: 9 bytes total (proprietary binary protocol)
 *          - Frame header/sync bytes
 *          - Flow X (pixel delta)
 *          - Flow Y (pixel delta)
 *          - Quality indicator (0-255)
 *          - Frame footer/checksum
 *          
 *          Thread safety: Called from main scheduler loop, no explicit locking required
 */
class AP_OpticalFlow_CXOF : public OpticalFlow_backend
{
public:
    /**
     * @brief Constructor for Cheerson CX-OF optical flow sensor backend
     * 
     * @param[in] _frontend Reference to the AP_OpticalFlow frontend manager
     * @param[in] uart Pointer to UART device for sensor communication (must be non-null)
     * 
     * @note Does not initialize hardware - call init() after construction
     */
    AP_OpticalFlow_CXOF(AP_OpticalFlow &_frontend, AP_HAL::UARTDriver *uart);

    /**
     * @brief Initialize the CX-OF sensor and configure UART communication
     * 
     * @details Configures UART to 115200 baud, 8 data bits, no parity, 1 stop bit (8N1).
     *          Resets internal packet buffer and state variables.
     *          Initializes gyro accumulation counters for velocity compensation.
     *          
     *          Called once during sensor backend initialization.
     */
    void init() override;

    /**
     * @brief Parse incoming UART data and update flow measurements
     * 
     * @details Performs the following operations:
     *          1. Reads available bytes from UART into packet buffer
     *          2. Validates 9-byte frame structure and synchronization
     *          3. Extracts flow_x and flow_y pixel motion values
     *          4. Extracts quality metric (0-255 scale)
     *          5. Computes inter-frame time delta (dt) in seconds
     *          6. Averages accumulated gyro samples for velocity compensation
     *          7. Converts pixel motion to angular flow rates (rad/s) using CXOF_PIXEL_SCALING
     *          8. Calls _update_frontend() with processed flow data
     *          9. Detects sensor timeout if no valid frames received
     *          
     *          Called at scheduler rate (typically 50Hz or faster) to process
     *          incoming sensor data stream (~10Hz from sensor).
     *          
     * @note Flow rates returned via _update_frontend() in rad/s units
     * @warning Timeout detection triggers if no valid frames for extended period
     */
    void update(void) override;

    /**
     * @brief Static factory method to detect and instantiate CX-OF sensor
     * 
     * @details Searches for Cheerson CX-OF optical flow sensor on available
     *          serial ports using AP_SerialManager. Queries serial manager for
     *          ports configured with SERIAL protocol type for optical flow.
     *          Creates new AP_OpticalFlow_CXOF instance if suitable UART found.
     *          
     *          Detection process:
     *          1. Query AP_SerialManager for optical flow protocol ports
     *          2. Check each port for CX-OF-compatible configuration
     *          3. Instantiate backend if valid port discovered
     *          4. Return nullptr if no suitable port found
     *          
     *          Called during optical flow subsystem initialization.
     * 
     * @param[in] _frontend Reference to the AP_OpticalFlow frontend manager
     * 
     * @return Pointer to new AP_OpticalFlow_CXOF instance if sensor detected,
     *         nullptr if no suitable UART port found or sensor not detected
     */
    static AP_OpticalFlow_CXOF *detect(AP_OpticalFlow &_frontend);

private:

    AP_HAL::UARTDriver *uart;           ///< Pointer to UART device connected to CX-OF sensor (115200 baud 8N1)
    uint64_t last_frame_us;             ///< System time of last valid frame received from sensor (microseconds), used for timeout detection
    uint8_t buf[10];                    ///< Packet assembly buffer for incoming 9-byte frames (10 bytes allocated for safety margin)
    uint8_t buf_len;                    ///< Current number of bytes accumulated in packet buffer (0-9 for valid packet assembly)
    Vector2f gyro_sum;                  ///< Accumulated gyro sensor values (rad/s) since last flow frame, used for velocity compensation
    uint16_t gyro_sum_count;            ///< Number of gyro samples accumulated in gyro_sum, used to compute average gyro rate
};

#endif  // AP_OPTICALFLOW_CXOF_ENABLED
