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
 * @file AP_MSP_Telem_DJI.h
 * @brief DJI FPV Air Unit specific MSP telemetry backend
 * 
 * @details This file implements the DJI-specific MSP telemetry backend for
 *          communication with DJI FPV Air Units and goggles. The DJI Air Unit
 *          uses the MSP protocol for telemetry and OSD display, with DJI-specific
 *          extensions and formatting requirements.
 * 
 * @note DJI Air Unit Polling Behavior:
 *       The DJI Air Unit polls the flight controller for MSP messages at approximately 4Hz.
 *       Messages are polled in ascending hexadecimal ID order. The complete list of
 *       polled messages is:
 * 
 *       Hex | Dec | Name
 *       ----|-----|-------------------------
 *       03  | 03  | MSP_FC_VERSION
 *       0a  | 10  | MSP_NAME
 *       54  | 84  | MSP_OSD_CONFIG
 *       5c  | 92  | MSP_FILTER_CONFIG
 *       5e  | 94  | MSP_PID_ADVANCED
 *       65  | 101 | MSP_STATUS
 *       69  | 105 | MSP_RC
 *       6a  | 106 | MSP_RAW_GPS
 *       6b  | 107 | MSP_COMP_GPS
 *       6c  | 108 | MSP_ATTITUDE
 *       6d  | 109 | MSP_ALTITUDE
 *       6e  | 110 | MSP_ANALOG
 *       6f  | 111 | MSP_RC_TUNING
 *       70  | 112 | MSP_PID
 *       82  | 130 | MSP_BATTERY_STATE
 *       86  | 134 | MSP_ESC_SENSOR_DATA
 *       96  | 150 | MSP_STATUS_EX
 *       f7  | 247 | MSP_RTC
 * 
 * @warning DJI Air Unit firmware compatibility - some OSD features require specific
 *          firmware versions. Ensure DJI firmware is up to date for full feature support.
 */

#pragma once

#include "AP_MSP_Telem_Backend.h"

#if HAL_MSP_ENABLED

/**
 * @class AP_MSP_Telem_DJI
 * @brief DJI FPV Air Unit telemetry backend with DJI-specific OSD integration
 * 
 * @details This class provides MSP telemetry specifically formatted for DJI FPV Air Units
 *          and goggles. It extends the base AP_MSP_Telem_Backend with DJI-specific features:
 *          - Push telemetry with WFQ scheduler for continuous updates
 *          - OSD stats fusion (peak/min battery values)
 *          - Custom RSSI handling using AP::rssi() singleton
 *          - DJI flight mode flags mapped from ArduPilot modes
 *          - ESC telemetry aggregation formatted for DJI display
 *          - DJI-compatible coordinate and unit formatting
 * 
 * @note Unlike the generic MSP backend, the DJI backend enables the WFQ (Weighted Fair Queuing)
 *       scheduler for continuous telemetry push to maintain smooth OSD updates at the DJI Air
 *       Unit's 4Hz polling rate.
 */
class AP_MSP_Telem_DJI : public AP_MSP_Telem_Backend
{
    using AP_MSP_Telem_Backend::AP_MSP_Telem_Backend;
public:
    /**
     * @brief Initialize UART for DJI communication
     * 
     * @details Configures the UART port for communication with the DJI Air Unit,
     *          potentially setting DJI-specific UART parameters for optimal performance.
     * 
     * @return true on successful initialization, false otherwise
     * 
     * @note May configure DJI-specific UART parameters such as baud rate and buffer sizes
     */
    bool init_uart() override;
    
    // implementation specific helpers
    
    /**
     * @brief Enable push telemetry scheduler for DJI
     * 
     * @details The DJI backend requires continuous telemetry push using the WFQ scheduler,
     *          unlike the generic MSP backend which operates in poll-only mode.
     * 
     * @return true - DJI Air Unit requires continuous telemetry push
     */
    bool is_scheduler_enabled() const override;
    
    /**
     * @brief Get serial protocol type
     * 
     * @return SerialProtocol_DJI_FPV for DJI Air Unit communication
     */
    AP_SerialManager::SerialProtocol get_serial_protocol() const override { return AP_SerialManager::SerialProtocol::SerialProtocol_DJI_FPV; };
    
    /**
     * @brief Generate DJI-specific flight mode flags
     * 
     * @details Maps ArduPilot vehicle states and flight modes to DJI OSD flag positions.
     *          The returned bitmask uses DJI_FLAG_* bit positions to indicate:
     *          - Armed status (DJI_FLAG_ARM)
     *          - Stabilize mode (DJI_FLAG_STAB)
     *          - Horizontal position hold (DJI_FLAG_HOR)
     *          - Heading hold (DJI_FLAG_HEAD)
     *          - Failsafe active (DJI_FLAG_FS)
     *          - Rescue mode (DJI_FLAG_RESC)
     * 
     * @return Bitmask with DJI_FLAG_* bits set based on current vehicle state
     * 
     * @note Uses DJI flag bit positions defined in the enum below
     */
    uint32_t get_osd_flight_mode_bitmask(void) override;
    
    /**
     * @brief Hide DJI-incompatible OSD items
     * 
     * @details Disables display of OSD elements that the DJI firmware may not support
     *          or that are incompatible with the DJI OSD rendering system.
     * 
     * @note DJI firmware may not support all Betaflight OSD items that ArduPilot can provide
     */
    void hide_osd_items(void) override;

    /**
     * @brief Get RSSI using AP::rssi() singleton for DJI compatibility
     * 
     * @param[out] rssi Receiver signal strength indicator value (0.0 to 1.0)
     * 
     * @return true if RSSI value is available, false otherwise
     * 
     * @note Uses AP::rssi() singleton for consistent RSSI reporting to DJI Air Unit
     */
    bool get_rssi(float &rssi) const override;
    
    /**
     * @brief Update home position with DJI-specific formatting
     * 
     * @param[in,out] home_state Home position state structure to update
     * 
     * @note Formats home position data for DJI OSD display requirements
     */
    void update_home_pos(home_state_t &home_state) override;
    
    /**
     * @brief Update battery state with OSD stats (peak/min values) for DJI
     * 
     * @param[in,out] _battery_state Battery state structure to update
     * 
     * @details Provides battery telemetry including voltage, current, and capacity
     *          with OSD-specific statistics (peak current, minimum voltage) that
     *          the DJI OSD can display for pilot awareness.
     */
    void update_battery_state(battery_state_t &_battery_state) override;
    
    /**
     * @brief Update GPS state with DJI coordinate format
     * 
     * @param[in,out] gps_state GPS state structure to update
     * 
     * @note Formats GPS coordinates and satellite information for DJI display requirements
     */
    void update_gps_state(gps_state_t &gps_state) override;
    
    /**
     * @brief Update airspeed for fixed-wing
     * 
     * @param[in,out] airspeed_state Airspeed state structure to update
     * 
     * @note Provides airspeed telemetry for fixed-wing aircraft OSD display
     */
    void update_airspeed(airspeed_state_t &airspeed_state) override;
    
    /**
     * @brief Generate flight mode string with DJI-specific labels
     * 
     * @param[out] flight_mode_str Output buffer for flight mode string
     * @param[in] size Size of output buffer
     * @param[in] wind_enabled Whether wind information should be included
     * 
     * @note Generates human-readable flight mode labels compatible with DJI OSD text display
     */
    void update_flight_mode_str(char *flight_mode_str, uint8_t size, bool wind_enabled) override;

    /**
     * @brief Generate FC_VARIANT response
     * 
     * @param[out] dst Output buffer for MSP message data
     * 
     * @return MSP_RESULT_ACK on success
     * 
     * @note Returns DJI-compatible flight controller identifier string for proper OSD display
     */
    MSP::MSPCommandResult msp_process_out_fc_variant(MSP::sbuf_t *dst) override;
    
    /**
     * @brief Generate ESC telemetry in DJI format
     * 
     * @param[out] dst Output buffer for MSP message data
     * 
     * @return MSP_RESULT_ACK on success
     * 
     * @details Aggregates ESC data from AP::esc_telem() singleton and formats it
     *          for DJI OSD display, including:
     *          - ESC RPM for each motor
     *          - ESC temperature readings
     *          - ESC current draw
     *          - ESC error flags
     * 
     * @note Aggregates ESC data and formats for DJI display requirements
     */
    MSP::MSPCommandResult msp_process_out_esc_sensor_data(MSP::sbuf_t *dst) override;

    /**
     * @brief DJI OSD flight mode flag bit positions
     * 
     * @details These flags map ArduPilot flight modes and states to DJI OSD display logic.
     *          Each flag represents a bit position in the flight mode bitmask sent to the
     *          DJI Air Unit for OSD rendering.
     * 
     * @note These flags map ArduPilot modes to DJI OSD display logic. The DJI OSD uses
     *       these bits to determine which mode indicators to display on the pilot's goggles.
     */
    enum : uint8_t {
        DJI_FLAG_ARM = 0,   ///< Armed status bit position - indicates vehicle is armed and motors can spin
        DJI_FLAG_STAB,      ///< Stabilize mode bit - indicates basic attitude stabilization is active
        DJI_FLAG_HOR,       ///< Horizontal position hold bit - indicates GPS position hold is active
        DJI_FLAG_HEAD,      ///< Heading hold bit - indicates heading/yaw hold is active
        DJI_FLAG_FS,        ///< Failsafe active bit - indicates a failsafe condition has been triggered
        DJI_FLAG_RESC,      ///< Rescue mode bit - indicates RTL or other autonomous recovery mode is active
    };
};

#endif //HAL_MSP_ENABLED
