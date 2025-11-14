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
*/

/**
 * @file AP_MSP_Telem_DisplayPort.h
 * @brief MSP DisplayPort telemetry backend for character-based OSD rendering
 * 
 * @details This file implements the MSP DisplayPort protocol backend which provides
 *          character-mode OSD (On-Screen Display) rendering capability on MSP-compatible
 *          devices such as Betaflight OSD, INAV OSD, DJI FPV goggles, and other
 *          MSP DisplayPort devices.
 *          
 *          DisplayPort bridges ArduPilot's AP_OSD subsystem to MSP-capable OSD hardware,
 *          allowing real-time flight telemetry display using character-based rendering.
 *          
 *          Protocol flow:
 *          1. OSD device sends HEARTBEAT messages to maintain connection
 *          2. OSD sends CLEAR_SCREEN at the start of each frame
 *          3. OSD sends multiple WRITE_STRING commands for each screen character/position
 *          4. OSD sends DRAW_SCREEN to commit and display the frame buffer
 *          5. ArduPilot responds with ACK for each command to confirm receipt
 * 
 * @note This backend is compatible with the Betaflight/INAV MSP DisplayPort specification
 * @note DisplayPort uses the MAX7456 character set by default; INAV font mapping available via options
 * 
 * @warning DisplayPort timing is critical - OSD devices expect responses within timeout period
 *          (typically 500ms). Delayed responses may cause OSD disconnection or display freezing.
 */
#pragma once

#include "AP_MSP_Telem_Backend.h"

#if HAL_WITH_MSP_DISPLAYPORT

/**
 * @class AP_MSP_Telem_DisplayPort
 * @brief MSP DisplayPort protocol backend for OSD integration
 * 
 * @details DisplayPort provides character-mode OSD rendering on MSP-compatible devices
 *          including Betaflight OSD, INAV OSD, DJI FPV goggles, and other MSP DisplayPort
 *          hardware. This backend bridges AP_OSD to the MSP DisplayPort protocol, enabling
 *          real-time flight telemetry display on external OSD devices.
 *          
 *          Unlike standard MSP telemetry backends that push data continuously, DisplayPort
 *          operates synchronously with OSD rendering cycles. The OSD device acts as the
 *          master, requesting screen updates at its refresh rate (typically 10-30Hz), and
 *          ArduPilot responds with character data for each frame.
 *          
 *          DisplayPort supports screen sizes up to 30 columns x 16 rows using the MAX7456
 *          character set (256 characters). Each frame is transmitted as a series of MSP
 *          commands (CLEAR_SCREEN, WRITE_STRING, DRAW_SCREEN) with ArduPilot acknowledging
 *          each command to maintain synchronization.
 * 
 * @note This backend bridges AP_OSD to MSP DisplayPort protocol for OSD display
 * @note Compatible with Betaflight/INAV MSP DisplayPort specification
 * @note DisplayPort command flow:
 *       - OSD sends HEARTBEAT to maintain connection
 *       - OSD sends CLEAR_SCREEN at start of frame
 *       - OSD sends multiple WRITE_STRING commands for each character
 *       - OSD sends DRAW_SCREEN to commit frame
 *       - ArduPilot responds with ACK for each command
 * 
 * @warning DisplayPort timing requirements - OSD expects responses within timeout period
 *          (typically 500ms). Failure to respond promptly may cause OSD disconnection.
 * 
 * @note Font compatibility - DisplayPort uses MAX7456 character set by default,
 *       INAV font mapping available via AP_MSP options
 */
class AP_MSP_Telem_DisplayPort : public AP_MSP_Telem_Backend
{
    using AP_MSP_Telem_Backend::AP_MSP_Telem_Backend;
public:
    /**
     * @brief Disable WFQ (Weighted Fair Queueing) telemetry scheduler
     * 
     * @return false - DisplayPort uses request/response protocol for OSD updates
     * 
     * @note DisplayPort operates synchronously with AP_OSD rendering, not continuous push
     * @note The OSD device controls update timing by sending commands at screen refresh rate
     * 
     * @details Standard MSP telemetry uses a scheduler to periodically push telemetry data.
     *          DisplayPort is different - it responds to requests from the OSD device rather
     *          than pushing data on a schedule. This ensures screen updates synchronize with
     *          the OSD's refresh rate and prevents buffer overruns.
     */
    bool is_scheduler_enabled() const override { return false; }
    
    /**
     * @brief Disable MSP thread handling
     * 
     * @return false - DisplayPort handled in OSD thread instead
     * 
     * @note This ensures DisplayPort updates synchronize with OSD rendering at screen refresh rate
     * 
     * @details Standard MSP telemetry runs in a dedicated MSP thread for concurrent processing.
     *          DisplayPort is handled directly in the AP_OSD thread to ensure tight synchronization
     *          between OSD rendering and MSP DisplayPort command processing. This eliminates
     *          latency and ensures frames are transmitted atomically.
     */
    bool use_msp_thread() const override { return false; }
    
    /**
     * @brief Initialize UART with DisplayPort-specific buffer sizes
     * 
     * @return true on successful initialization, false on failure
     * 
     * @note Calls uart->begin(0, 256, 768) for optimized DisplayPort buffer allocation
     * 
     * @details Larger buffers accommodate complete screen updates (30x16 characters maximum).
     *          The 768-byte TX buffer allows an entire frame to be buffered, while the 256-byte
     *          RX buffer handles incoming commands from the OSD device. These buffer sizes are
     *          optimized for DisplayPort's burst transfer pattern during screen updates.
     *          
     *          Buffer sizing:
     *          - RX buffer: 256 bytes (sufficient for command reception)
     *          - TX buffer: 768 bytes (full screen frame: 30x16 chars + protocol overhead)
     */
    bool init_uart() override;
    
    /**
     * @brief Get serial protocol type for this backend
     * 
     * @return SerialProtocol_MSP_DisplayPort for DisplayPort OSD devices
     * 
     * @details This identifier is used by the serial manager to route DisplayPort traffic
     *          to this backend and configure appropriate serial port parameters.
     */
    AP_SerialManager::SerialProtocol get_serial_protocol() const override { return AP_SerialManager::SerialProtocol::SerialProtocol_MSP_DisplayPort; };

    /**
     * @brief Generate FC_VARIANT response with configurable identifier
     * 
     * @param[out] dst Output buffer for MSP response message
     * 
     * @return MSP_RESULT_ACK on success
     * 
     * @note Returns BETAFLIGHT or ARDUPILOT identifier based on AP_MSP options
     * 
     * @details Some OSD devices expect a BETAFLIGHT identifier for full feature support
     *          and may disable advanced features or use different font mappings when
     *          detecting other flight controller types. The AP_MSP configuration allows
     *          selecting the identifier to maximize OSD compatibility.
     *          
     *          FC_VARIANT identifies the flight controller type to the OSD device, affecting:
     *          - Feature availability (some OSD features are FC-specific)
     *          - Font character mapping (Betaflight vs INAV vs ArduPilot)
     *          - Protocol extensions and custom commands
     *          
     *          Configuration via AP_MSP parameters determines whether to report as
     *          BETAFLIGHT (for maximum compatibility) or ARDUPILOT (for accurate identification).
     */
    MSP::MSPCommandResult msp_process_out_fc_variant(MSP::sbuf_t *dst) override;
};

#endif //HAL_WITH_MSP_DISPLAYPORT
