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
 * @file AP_MSP.h
 * @brief Main MSP (MultiWii Serial Protocol) manager implementing v1 and v2 protocols
 * 
 * @details This is the primary interface for MSP protocol integration in ArduPilot.
 *          MSP protocol is based on Betaflight/iNav implementations and provides:
 *          - Telemetry data transmission to MSP-compatible devices
 *          - OSD (On-Screen Display) integration with MSP DisplayPort
 *          - Support for DJI FPV system and generic MSP devices
 *          - Multi-backend architecture supporting up to 3 concurrent MSP interfaces
 * 
 * @author Alex Apostoli
 * @note Based on betaflight/iNav MSP implementations
 */

#pragma once

#include "AP_MSP_config.h"

#if HAL_MSP_ENABLED

#include <AP_OSD/AP_OSD.h>
#include "AP_MSP_Telem_Backend.h"

/**
 * @brief Enable INAV font translation support
 * @details When enabled (1), allows translation between ArduPilot OSD symbols and INAV font set
 * @note Default: 1 (enabled)
 */
#ifndef AP_MSP_INAV_FONTS_ENABLED
#define AP_MSP_INAV_FONTS_ENABLED 1
#endif

/**
 * @brief Maximum number of concurrent MSP backend instances
 * @details Supports up to 3 simultaneous MSP connections (e.g., Generic + DJI + DisplayPort)
 * @note Each backend requires a dedicated UART port configured via SerialManager
 */
#define MSP_MAX_INSTANCES 3

/**
 * @brief Base offset for MSP OSD item position encoding
 * @details MSP OSD uses linear position encoding starting at offset 2048
 * @note Positions below 2048 may have special meanings in MSP protocol
 */
#define MSP_OSD_START 2048

/**
 * @brief Horizontal step size for OSD position encoding (1 column per unit)
 */
#define MSP_OSD_STEP_X 1

/**
 * @brief Vertical step size for OSD position encoding (32 columns per row)
 * @details MSP OSD assumes 32 character columns per row for position calculation
 */
#define MSP_OSD_STEP_Y 32

/**
 * @brief Calculate MSP OSD position from x,y coordinates
 * @param osd_setting AP_OSD_Setting pointer with xpos/ypos members
 * @return Encoded position value for MSP protocol
 * @details MSP OSD uses linear position encoding: position = 2048 + x + y*32
 *          where x is horizontal position (0-31) and y is vertical position (0-15)
 * @note This macro converts 2D screen coordinates to MSP's 1D position format
 */
#define MSP_OSD_POS(osd_setting) (MSP_OSD_START + osd_setting->xpos*MSP_OSD_STEP_X + osd_setting->ypos*MSP_OSD_STEP_Y)

/**
 * @class AP_MSP
 * @brief Main MSP protocol manager and OSD integration singleton
 * 
 * @details Manages multiple MSP backend instances and provides OSD integration:
 *          - Discovers MSP-capable serial ports via AP_SerialManager
 *          - Creates and manages up to 3 concurrent MSP backends (Generic, DJI, DisplayPort)
 *          - Maintains OSD item mapping and configuration for MSP OSD devices
 *          - Runs dedicated MSP loop thread for processing telemetry
 *          - Integrates with AP_OSD for OSD item configuration and display
 * 
 * @note Singleton pattern - access via AP::msp()
 * @warning This class manages flight-critical telemetry and OSD display
 * 
 * Thread Safety: MSP loop runs in dedicated thread, backends use WFQ scheduler for bandwidth management
 */
class AP_MSP
{
    friend class AP_MSP_Telem_Generic;
    friend class AP_MSP_Telem_DJI;
    friend class AP_MSP_Telem_Backend;
#if HAL_WITH_MSP_DISPLAYPORT
    friend class AP_MSP_Telem_DisplayPort;
    friend class AP_OSD_MSP_DisplayPort;
#endif
public:
    /**
     * @brief Constructor for AP_MSP singleton
     * @note Private constructor for singleton pattern - use AP::msp() for access
     */
    AP_MSP();

    /**
     * @brief Prevent copying of AP_MSP singleton
     * @note Non-copyable class - only one instance should exist
     */
    CLASS_NO_COPY(AP_MSP);

    /**
     * @brief AP_Param parameter table for MSP configuration
     * @details Contains user-configurable parameters:
     *          - MSP_OPTIONS: Configuration options bitmask (see Option enum)
     *          - MSP_OSD_NCELLS: LiPo cell count override for battery display
     * @note Parameters are stored in EEPROM and configurable via ground station
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Initialize MSP subsystem and backends
     * @details Performs complete MSP initialization:
     *          1. Discovers serial ports via AP::serialmanager() with MSP protocols
     *          2. Creates backend instances based on protocol type (Generic, DJI, DisplayPort)
     *          3. Initializes OSD item mapping from AP_OSD configuration
     *          4. Starts dedicated MSP thread for telemetry processing
     * @note Should be called once during system initialization after AP_SerialManager init
     * @warning Must be called before any MSP functionality is available
     */
    void init();

    /**
     * @enum Option
     * @brief Configuration options for MSP behavior (bitmask)
     * @details Options configured via MSP_OPTIONS parameter:
     */
    enum class Option : uint8_t {
        TELEMETRY_MODE = 1U<<0,                         ///< Enable telemetry mode (bit 0)
        TELEMETRY_DISABLE_DJI_WORKAROUNDS = 1U<<1,      ///< Disable DJI-specific compatibility workarounds (bit 1)
        DISPLAYPORT_BTFL_SYMBOLS = 1U<<2,               ///< Use Betaflight symbol set for DisplayPort (bit 2)
        DISPLAYPORT_INAV_SYMBOLS = 1U<<3,               ///< Use INAV symbol set for DisplayPort (bit 3)
    };

    /**
     * @brief Check if a specific option is enabled
     * @param option Option enum value to check
     * @return true if the option bit is set in _options parameter
     * @note Used internally by backends to determine behavior
     */
    bool is_option_enabled(const Option option) const;

    /**
     * @brief Get AP_MSP singleton instance
     * @return Pointer to singleton instance, or nullptr if not initialized
     * @note Prefer using AP::msp() wrapper instead of calling directly
     */
    static AP_MSP *get_singleton(void)
    {
        return _singleton;
    }

private:
    /**
     * @brief Array of MSP backend instances
     * @details Holds pointers to up to 3 MSP backends (Generic, DJI, DisplayPort)
     * @note nullptr entries indicate unused slots
     */
    AP_MSP_Telem_Backend *_backends[MSP_MAX_INSTANCES];

    /**
     * @brief Configuration options bitmask (MSP_OPTIONS parameter)
     * @details Configures MSP behavior via Option enum flags
     * @see Option enum for available configuration flags
     */
    AP_Int8 _options;

    /**
     * @brief LiPo cell count override (MSP_OSD_NCELLS parameter)
     * @details Used for battery voltage display on OSD. When non-zero, overrides auto-detection
     * @note Typical values: 3 (11.1V), 4 (14.8V), 6 (22.2V)
     */
    AP_Int8 _cellcount;

    /**
     * @brief Pointers to AP_OSD item configurations for MSP OSD
     * @details Maps MSP OSD item indices to AP_OSD_Setting configurations
     * @note Array size matches MSP::OSD_ITEM_COUNT
     * @warning Must stay synchronized with AP_OSD screen configuration
     */
    AP_OSD_Setting* _osd_item_settings[MSP::OSD_ITEM_COUNT];

    /**
     * @brief MSP OSD configuration structure
     * @details Contains OSD resolution, units, and feature flags sent to MSP devices
     */
    MSP::osd_config_t _osd_config;

    /**
     * @brief MSP runtime status tracking
     * @details Maintains state for OSD display and backend management:
     */
    struct {
        bool flashing_on;                      ///< OSD item blink state at 1.4Hz for attention-getting items
        bool slow_flashing_on;                 ///< OSD item blink state at 0.5Hz for slow flashing items
        uint8_t last_flight_mode = 255;        ///< Previous flight mode for change detection
        uint32_t last_flight_mode_change_ms;   ///< Timestamp of last mode change for focus timeout
        bool flight_mode_focus;                ///< Priority display of flight mode change (steals focus from text messages)
        bool osd_initialized;                  ///< One-time initialization flag for OSD setup
        uint8_t backend_count;                 ///< Number of active backends (0-3)
        uint8_t current_screen;                ///< Current OSD screen index (0-3, defaults to screen 0)
    } _msp_status;

    /**
     * @brief Create and initialize a specific MSP backend
     * @param backend_idx Index in _backends array (0 to MSP_MAX_INSTANCES-1)
     * @param uart UART driver instance for this backend
     * @param protocol SerialProtocol type (MSP, DJI_FPV, or MSP_DisplayPort)
     * @return true on successful backend creation and initialization, false on failure
     * @details Creates appropriate backend type based on protocol and initializes UART
     * @note Called during init() for each discovered MSP serial port
     */
    bool init_backend(uint8_t backend_idx, AP_HAL::UARTDriver *uart, AP_SerialManager::SerialProtocol protocol);

    /**
     * @brief Refresh OSD item configuration from AP_OSD
     * @details Synchronizes _osd_item_settings array with current AP_OSD screen configuration
     * @note Called periodically to detect OSD configuration changes
     * @warning Critical for keeping MSP OSD display in sync with AP_OSD settings
     */
    void update_osd_item_settings();

    /**
     * @brief MSP thread main loop
     * @details Runs continuously in dedicated MSP thread at ~100Hz:
     *          1. Calls backend init_uart() for port setup
     *          2. Calls backend hide_osd_items() to manage visibility
     *          3. Calls backend process_incoming_data() for MSP message reception
     *          4. Calls backend process_outgoing_data() for telemetry transmission
     * @note Backends use WFQ (Weighted Fair Queuing) scheduler for bandwidth management
     * @warning Thread timing critical for smooth OSD updates and telemetry
     */
    void loop(void);

    /**
     * @brief Find backend by protocol type
     * @param protocol SerialProtocol to search for
     * @return Pointer to matching backend, or nullptr if not found
     * @details Searches _backends array for backend matching specified protocol
     * @note Used to locate specific backend types (e.g., DJI backend for special handling)
     */
    AP_MSP_Telem_Backend* find_protocol(const AP_SerialManager::SerialProtocol protocol) const;

    /**
     * @brief Singleton instance pointer
     * @note Accessed via get_singleton() or AP::msp()
     */
    static AP_MSP *_singleton;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP
{
/**
 * @brief Get AP_MSP singleton via AP namespace
 * @return Pointer to AP_MSP singleton instance
 * @details Preferred access method for AP_MSP singleton
 * @note Usage: AP::msp()->init();
 * @see AP_MSP::get_singleton()
 */
AP_MSP *msp();
};

#endif //HAL_MSP_ENABLED
