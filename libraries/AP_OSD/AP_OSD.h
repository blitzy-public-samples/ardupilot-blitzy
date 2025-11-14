/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file AP_OSD.h
 * @brief On-Screen Display (OSD) subsystem for ArduPilot autopilot
 * 
 * @details This file defines the complete OSD subsystem including the main manager,
 *          screen abstractions, and backend interfaces for rendering flight data
 *          overlays on video feeds. The system supports multiple OSD backends
 *          (MAX7456 analog, MSP DisplayPort, SITL simulation) and provides
 *          configurable multi-screen layouts with 50+ display panels.
 * 
 *          Architecture: AP_OSD manager → multiple screens → backend rendering
 *          - AP_OSD: Singleton manager coordinating backends, screen switching, statistics
 *          - AP_OSD_AbstractScreen: Base interface for screen types
 *          - AP_OSD_Screen: Display screen with configurable panel layout
 *          - AP_OSD_ParamScreen: Interactive parameter editor screen
 *          - AP_OSD_Backend: Hardware abstraction (MAX7456, MSP, SITL)
 * 
 *          Thread Model: Dedicated OSD thread runs update loop, protected by HAL_Semaphore
 *          Screen Switching: Toggle, PWM range, or auto-switch based on RC channel
 *          Panel System: Each panel has enable flag and position (x: 0-29, y: 0-15 char cells)
 * 
 * @note OSD runs in dedicated thread at backend-specific update rate
 * @warning Thread-safe access required - use get_semaphore() for concurrent access
 * @see AP_OSD_Backend for backend implementation interface
 */

#pragma once

#include "AP_OSD_config.h"

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS_config.h>
#include <AP_OLC/AP_OLC.h>
#include <AP_MSP/msp.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_RPM/AP_RPM_config.h>
#if HAL_GCS_ENABLED
#include <GCS_MAVLink/GCS_MAVLink.h>
#endif
#include <AC_Fence/AC_Fence_config.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>

class AP_OSD_Backend;
class AP_MSP;

#define AP_OSD_NUM_DISPLAY_SCREENS 4
#if OSD_PARAM_ENABLED
#define AP_OSD_NUM_PARAM_SCREENS 2
#else
#define AP_OSD_NUM_PARAM_SCREENS 0
#endif
#define AP_OSD_NUM_SCREENS (AP_OSD_NUM_DISPLAY_SCREENS + AP_OSD_NUM_PARAM_SCREENS)

#define PARAM_INDEX(key, idx, group) (uint32_t(uint32_t(key) << 23 | uint32_t(idx) << 18 | uint32_t(group)))
#define PARAM_TOKEN_INDEX(token) PARAM_INDEX(AP_Param::get_persistent_key(token.key), token.idx, token.group_element)

#define AP_OSD_NUM_SYMBOLS 107
#define OSD_MAX_INSTANCES 2

#if AP_OSD_LINK_STATS_EXTENSIONS_ENABLED
// For the moment, these extra panels only work with CRSF protocol based RC systems
#define AP_OSD_EXTENDED_LNK_STATS 1
#define AP_OSD_WARN_RSSI_DEFAULT -100   // Default value for OSD RSSI panel warning, in dbm
#else
#define AP_OSD_EXTENDED_LNK_STATS 0
#define AP_OSD_WARN_RSSI_DEFAULT 30     // Default value for OSD RSSI panel warning, in %
#endif

/**
 * @class AP_OSD_Setting
 * @brief Individual OSD panel configuration storing enable state and screen position
 * 
 * @details Each OSD display element (altitude, battery voltage, GPS satellites, etc.)
 *          is configured by an AP_OSD_Setting instance that controls visibility and
 *          positioning on the character grid overlay.
 * 
 *          Position coordinates use character cell units:
 *          - X range: 0-29 (30 columns, standard PAL/NTSC character width)
 *          - Y range: 0-15 (16 rows, standard PAL/NTSC character height)
 *          - Origin (0,0) at top-left corner of screen
 * 
 *          The setting is exposed through AP_Param for runtime configuration via
 *          ground control stations and parameter files.
 * 
 * @note Settings persist across reboots via AP_Param storage
 * @see AP_OSD_Screen for complete list of available panels
 */
class AP_OSD_Setting
{
public:
    AP_Int8 enabled;  ///< Panel enable flag: 0=hidden, 1=visible on screen
    AP_Int8 xpos;     ///< Horizontal position in character cells (0-29)
    AP_Int8 ypos;     ///< Vertical position in character cells (0-15)

    /**
     * @brief Construct OSD panel setting with default position and visibility
     * 
     * @param[in] enabled Default visibility state (false=hidden, true=visible)
     * @param[in] x Default horizontal position in character cells (0-29)
     * @param[in] y Default vertical position in character cells (0-15)
     * 
     * @note Default values are stored for parameter system initialization
     */
    AP_OSD_Setting(bool enabled = 0, uint8_t x = 0, uint8_t y = 0);

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    const float default_enabled;  ///< Default enable state for parameter reset
    const float default_xpos;     ///< Default X position for parameter reset
    const float default_ypos;     ///< Default Y position for parameter reset
};

class AP_OSD;

/**
 * @class AP_OSD_AbstractScreen
 * @brief Base interface for OSD screen types (display and parameter screens)
 * 
 * @details Abstract base class defining the screen interface for ArduPilot OSD system.
 *          Screens can be display screens (AP_OSD_Screen with flight data panels) or
 *          parameter screens (AP_OSD_ParamScreen for on-OSD configuration).
 * 
 *          Lifecycle:
 *          1. Screen constructed and added to AP_OSD manager
 *          2. Backend assigned via set_backend()
 *          3. draw() called periodically from OSD thread
 *          4. Screen switching controlled by RC channel or auto-switch
 * 
 *          Screen Activation: Controlled by enabled flag and PWM range (channel_min/max)
 *          - enabled=0: Screen completely disabled
 *          - enabled=1: Screen active within PWM range or via toggle
 *          - PWM range: Screen shown when RC channel value between min and max
 * 
 * @note Derived classes override draw() to implement screen-specific rendering
 * @see AP_OSD_Screen for flight data display implementation
 * @see AP_OSD_ParamScreen for parameter editor implementation
 */
class AP_OSD_AbstractScreen
{
    friend class AP_OSD;
public:
    /**
     * @brief Default constructor for abstract screen
     */
    AP_OSD_AbstractScreen() {}
    
    /**
     * @brief Render screen content to backend (virtual, overridden by derived classes)
     * 
     * @details Called periodically from OSD thread to update screen display.
     *          Default implementation does nothing; derived classes implement
     *          specific rendering logic for their screen type.
     * 
     * @note Runs in dedicated OSD thread, protected by HAL_Semaphore
     * @warning Do not block - keep execution time minimal for smooth updates
     */
    virtual void draw(void) {}

    /**
     * @brief Assign rendering backend to this screen
     * 
     * @param[in] _backend Pointer to hardware backend (MAX7456, MSP, SITL)
     * 
     * @note Called during initialization before screen becomes active
     */
    void set_backend(AP_OSD_Backend *_backend);

    AP_Int8 enabled;       ///< Screen enable flag: 0=disabled, 1=enabled
    AP_Int16 channel_min;  ///< Minimum RC PWM value for screen activation (μs)
    AP_Int16 channel_max;  ///< Maximum RC PWM value for screen activation (μs)

protected:
    /**
     * @brief Check if OSD option flag is enabled
     * 
     * @param[in] option Option bitmask to check (e.g., OPTION_DECIMAL_PACK)
     * @return true if option is enabled in global OSD options
     */
    bool check_option(uint32_t option);
    
#if HAL_WITH_MSP_DISPLAYPORT
    /**
     * @brief Get text resolution for MSP DisplayPort (virtual)
     * 
     * @return Text resolution identifier (0=SD, 1=HD)
     * @note Override in derived classes for resolution-specific rendering
     */
    virtual uint8_t get_txt_resolution() const {
        return 0;
    }
    
    /**
     * @brief Get font index for MSP DisplayPort (virtual)
     * 
     * @return Font index for backend font selection
     * @note Override in derived classes for font-specific rendering
     */
    virtual uint8_t get_font_index() const {
        return 0;
    }
#endif

    /**
     * @enum unit_type
     * @brief Unit categories for display conversion and symbol selection
     * 
     * @details Used by u_scale() and u_icon() to convert values and select
     *          appropriate unit symbols based on configured unit system
     *          (METRIC, IMPERIAL, SI, AVIATION).
     */
    enum unit_type {
        ALTITUDE=0,       ///< Altitude/height values (m, ft)
        SPEED=1,          ///< Horizontal speed (m/s, mph, kt)
        VSPEED=2,         ///< Vertical speed/climb rate (m/s, ft/min)
        DISTANCE=3,       ///< Short distance (m, ft)
        DISTANCE_LONG=4,  ///< Long distance/range (km, mi, nm)
        TEMPERATURE=5,    ///< Temperature (°C, °F)
        UNIT_TYPE_LAST=6, ///< Sentinel value for range checking
    };

    /**
     * @brief Get unit symbol character for display
     * 
     * @param[in] unit Unit category
     * @return Character code for unit symbol (e.g., 'm', 'ft', 'kt')
     * 
     * @note Returns appropriate symbol based on OSD units setting
     */
    char u_icon(enum unit_type unit);
    
    /**
     * @brief Scale value for configured unit system
     * 
     * @param[in] unit Unit category for conversion
     * @param[in] value Value in SI units (meters, m/s, etc.)
     * @return Scaled value in display units
     * 
     * @details Converts SI values to user-configured units:
     *          - METRIC: m, m/s, km, °C
     *          - IMPERIAL: ft, mph, mi, °F
     *          - SI: m, m/s, km, °C (no conversion)
     *          - AVIATION: ft, kt, nm, °C
     */
    float u_scale(enum unit_type unit, float value);

    AP_OSD_Backend *backend;  ///< Rendering backend (MAX7456, MSP, SITL)
    AP_OSD *osd;              ///< Pointer to parent OSD manager singleton

    static uint8_t symbols_lookup_table[AP_OSD_NUM_SYMBOLS];  ///< Symbol remapping table for fonts
};

#if OSD_ENABLED
/**
 * @class AP_OSD_Screen
 * @brief Flight data display screen with configurable panel layout
 * 
 * @details Implements complete OSD display screen with 50+ available panels showing
 *          flight data, navigation info, battery status, sensors, and system state.
 *          Each panel is individually configurable with enable flag and screen position.
 * 
 *          Panel Categories:
 *          - Flight: altitude, speed, heading, compass, artificial horizon
 *          - Battery: voltage, current, consumed mAh, cell voltage
 *          - Navigation: home distance/direction, waypoint info, GPS position
 *          - Sensors: GPS satellites, RSSI, link quality, airspeed, wind
 *          - System: flight mode, messages, clock, arming status
 *          - Statistics: max altitude/speed/distance, efficiency
 *          - ESC: temperature, RPM, current (per-ESC with ESC telemetry)
 * 
 *          Rendering: draw() method called from OSD thread, iterates enabled panels
 *          and calls corresponding draw_*() methods with configured position.
 * 
 *          Screen Switching: Multiple screens (1-4) can be configured with different
 *          layouts for different flight phases (armed/disarmed, failsafe, etc.).
 * 
 *          Position Grid: 30x16 character cells (typical PAL/NTSC overlay)
 *          Units: Converted based on OSD units setting (METRIC/IMPERIAL/SI/AVIATION)
 * 
 * @note Compiled only when OSD_ENABLED and backend supports bitmap/DisplayPort
 * @warning Panel positions outside 0-29 (x) or 0-15 (y) may be clipped by backend
 * @see AP_OSD_Setting for panel configuration structure
 */
class AP_OSD_Screen : public AP_OSD_AbstractScreen
{
public:
    /**
     * @brief Construct flight data display screen
     * 
     * @details Initializes all panel settings with default positions and enables
     *          commonly used panels (altitude, battery, speed, mode, etc.).
     */
    AP_OSD_Screen();

    /**
     * @brief Render all enabled panels to backend
     * 
     * @details Iterates through all configured panels, checks enabled flags,
     *          and calls corresponding draw_*() methods to render flight data
     *          at configured screen positions.
     * 
     *          Rendering order optimized for typical overlay priority.
     *          Skipped on non-font backends to save flash space.
     * 
     * @note Called from dedicated OSD thread at backend update rate
     * @warning Keep execution time minimal to maintain smooth OSD updates
     */
#if HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT
    void draw(void) override;
#endif

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];   ///< Parameter table for basic panels
    static const struct AP_Param::GroupInfo var_info2[];  ///< Parameter table for extended panels

#if HAL_WITH_MSP_DISPLAYPORT
    /**
     * @brief Get text resolution for MSP DisplayPort
     * @return Text resolution (0=SD, 1=HD)
     */
    uint8_t get_txt_resolution() const override {
        return txt_resolution;
    }
    
    /**
     * @brief Get font index for MSP DisplayPort
     * @return Font index for backend font selection
     */
    uint8_t get_font_index() const override {
        return font_index;
    }
#endif
private:
    friend class AP_MSP;
    friend class AP_MSP_Telem_Backend;
    friend class AP_MSP_Telem_DJI;

    static const uint8_t message_visible_width = 26;
    static const uint8_t message_scroll_time_ms = 200;
    static const uint8_t message_scroll_delay = 5;

    static constexpr float ah_max_pitch = DEG_TO_RAD * 20;
    //typical fpv camera has 80deg vertical field of view, 16 row of chars
    static constexpr float ah_pitch_rad_to_char = 16.0f/(DEG_TO_RAD * 80);

    enum class VoltageType {
        VOLTAGE,
        RESTING_VOLTAGE,
        AVG_CELL,
        RESTING_CELL,
    };

    AP_OSD_Setting altitude{true, 23, 8};
    AP_OSD_Setting bat_volt{true, 24, 1};
    AP_OSD_Setting rssi{true, 1, 1};
    AP_OSD_Setting link_quality{false,1,1};
    AP_OSD_Setting restvolt{false, 24, 2};
    AP_OSD_Setting avgcellvolt{false, 24, 3};
    AP_OSD_Setting avgcellrestvolt{false, 24, 4};
    AP_OSD_Setting current{true, 25, 2};
    AP_OSD_Setting batused{true, 23, 3};
    AP_OSD_Setting sats{true, 1, 3};
    AP_OSD_Setting fltmode{true, 2, 8};
    AP_OSD_Setting message{true, 2, 6};
    AP_OSD_Setting gspeed{true, 2, 14};
    AP_OSD_Setting horizon{true, 14, 8};
    AP_OSD_Setting home{true, 14, 1};
    AP_OSD_Setting throttle{true, 24, 11};
    AP_OSD_Setting heading{true, 13, 2};
    AP_OSD_Setting compass{true, 15, 3};
    AP_OSD_Setting wind{false, 2, 12};
    AP_OSD_Setting aspeed{false, 2, 13};
    AP_OSD_Setting aspd1;
    AP_OSD_Setting aspd2;
    AP_OSD_Setting vspeed{true, 24, 9};
#if AP_RPM_ENABLED
    AP_OSD_Setting rrpm{false, 2, 2};
#endif
#if HAL_WITH_ESC_TELEM
    AP_OSD_Setting esc_temp {false, 24, 13};
    AP_OSD_Setting esc_rpm{false, 22, 12};
    AP_OSD_Setting esc_amps{false, 24, 14};
#endif
    AP_OSD_Setting gps_latitude{true, 9, 13};
    AP_OSD_Setting gps_longitude{true, 9, 14};
    AP_OSD_Setting roll_angle;
    AP_OSD_Setting pitch_angle;
    AP_OSD_Setting temp;
#if BARO_MAX_INSTANCES > 1
    AP_OSD_Setting btemp;
#endif
    AP_OSD_Setting hdop;
    AP_OSD_Setting waypoint;
    AP_OSD_Setting xtrack_error;
    AP_OSD_Setting dist{false,22,11};
    AP_OSD_Setting stat{false,0,0};
    AP_OSD_Setting flightime{false, 23, 10};
    AP_OSD_Setting climbeff{false,0,0};
    AP_OSD_Setting eff{false, 22, 10};
    AP_OSD_Setting atemp;
    AP_OSD_Setting bat2_vlt;
    AP_OSD_Setting bat2used;
    AP_OSD_Setting current2;
    AP_OSD_Setting clk;
    AP_OSD_Setting callsign;
    AP_OSD_Setting vtx_power;
    AP_OSD_Setting hgt_abvterr{false, 23, 7};
    AP_OSD_Setting fence{false, 14, 9};
    AP_OSD_Setting rngf;
#if HAL_PLUSCODE_ENABLE
    AP_OSD_Setting pluscode;
#endif
    AP_OSD_Setting sidebars{false, 4, 5};

#if AP_OSD_EXTENDED_LNK_STATS
    // Extended link stats data panels
    AP_OSD_Setting rc_tx_power{false, 25, 12};
    AP_OSD_Setting rc_rssi_dbm{false, 6, 2};
    AP_OSD_Setting rc_snr{false, 23, 13};
    AP_OSD_Setting rc_active_antenna{false, 27, 13};
    AP_OSD_Setting rc_lq{false, 18, 2};
#endif

    // MSP OSD only
    AP_OSD_Setting crosshair;
    AP_OSD_Setting home_dist{true, 1, 1};
    AP_OSD_Setting home_dir{true, 1, 1};
    AP_OSD_Setting power{true, 1, 1};
    AP_OSD_Setting cell_volt{true, 1, 1};
    AP_OSD_Setting batt_bar{true, 1, 1};
    AP_OSD_Setting arming{true, 1, 1};

#if HAL_WITH_MSP_DISPLAYPORT
    // Per screen HD resolution options (currently supported only by DisplayPort)
    AP_Int8 txt_resolution;
    AP_Int8 font_index;
#endif
#if HAL_WITH_ESC_TELEM
    AP_Int8 esc_index;
#endif

    /**
     * @brief Draw altitude panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays relative altitude from home in configured units (m or ft)
     */
    void draw_altitude(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw battery voltage panel (with voltage type selection)
     * @param[in] instance Battery instance (0=primary, 1=secondary)
     * @param[in] type Voltage type (VOLTAGE, RESTING_VOLTAGE, AVG_CELL, RESTING_CELL)
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays battery voltage or per-cell voltage with configurable type
     */
    void draw_bat_volt(uint8_t instance,VoltageType  type,uint8_t x, uint8_t y);
    
    /**
     * @brief Draw primary battery voltage panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays total battery voltage in volts (V)
     */
    void draw_bat_volt(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw average cell voltage panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays average voltage per cell (total voltage / cell count) in V
     */
    void draw_avgcellvolt(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw average resting cell voltage panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays estimated resting voltage per cell (compensated for load) in V
     */
    void draw_avgcellrestvolt(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw resting battery voltage panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays estimated resting voltage (compensated for current draw) in V
     */
    void draw_restvolt(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw RSSI (signal strength) panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays RC signal strength as percentage (0-100%) or dBm with warning threshold
     */
    void draw_rssi(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw link quality panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays link quality metric from RC telemetry (percentage 0-100%)
     */
    void draw_link_quality(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw primary battery current panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays instantaneous current draw in amperes (A)
     */
    void draw_current(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw battery current panel (with instance selection)
     * @param[in] instance Battery instance (0=primary, 1=secondary)
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays instantaneous current draw for specified battery in amperes (A)
     */
    void draw_current(uint8_t instance, uint8_t x, uint8_t y);
    
    /**
     * @brief Draw primary battery consumed capacity panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays total energy consumed since boot in milliamp-hours (mAh)
     */
    void draw_batused(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw battery consumed capacity panel (with instance selection)
     * @param[in] instance Battery instance (0=primary, 1=secondary)
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays total energy consumed for specified battery in milliamp-hours (mAh)
     */
    void draw_batused(uint8_t instance, uint8_t x, uint8_t y);
    
    /**
     * @brief Draw GPS satellite count panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays number of GPS satellites with 3D fix and lock indicator
     */
    void draw_sats(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw flight mode panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays current flight mode name (e.g., STABILIZE, LOITER, AUTO)
     */
    void draw_fltmode(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw status message panel with scrolling
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays system messages and warnings with auto-scroll for long messages
     */
    void draw_message(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw ground speed panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays GPS ground speed in configured units (m/s, mph, kt)
     */
    void draw_gspeed(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw artificial horizon panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays pitch/roll attitude indicator with horizon line
     */
    void draw_horizon(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw home direction arrow panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays arrow pointing toward home position
     */
    void draw_home(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw throttle percentage panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays throttle output as percentage (0-100%)
     */
    void draw_throttle(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw heading panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays vehicle heading in degrees (0-359°)
     */
    void draw_heading(uint8_t x, uint8_t y);
#if AP_RPM_ENABLED
    /**
     * @brief Draw RPM sensor panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays rotational speed from RPM sensor in revolutions per minute
     */
    void draw_rrpm(uint8_t x, uint8_t y);
#endif
#ifdef HAL_OSD_SIDEBAR_ENABLE
    /**
     * @brief Draw altitude/speed sidebars panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays vertical bar graphs for altitude and speed on screen edges
     */
    void draw_sidebars(uint8_t x, uint8_t y);
#endif
    /**
     * @brief Draw compass rose panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays compass rose with cardinal directions and current heading
     */
    void draw_compass(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw wind direction and speed panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays estimated wind vector with arrow and speed in configured units
     */
    void draw_wind(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw primary airspeed panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays airspeed from primary sensor in configured units (m/s, mph, kt)
     */
    void draw_aspeed(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw airspeed sensor 1 panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays airspeed from sensor instance 1 in configured units
     */
    void draw_aspd1(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw airspeed sensor 2 panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays airspeed from sensor instance 2 in configured units
     */
    void draw_aspd2(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw vertical speed (climb rate) panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays vertical speed in configured units (m/s or ft/min), positive=climbing
     */
    void draw_vspeed(uint8_t x, uint8_t y);
    
#if HAL_PLUSCODE_ENABLE
    /**
     * @brief Draw Plus Code (Open Location Code) panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays current position as Plus Code for location sharing
     */
    void draw_pluscode(uint8_t x, uint8_t y);
#endif

    /**
     * @brief Helper function to draw speed vector with arrow
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @param[in] angle_rad Direction angle in radians
     * @param[in] magnitude Speed magnitude in m/s
     * @details Renders directional arrow and magnitude in configured units
     */
    void draw_speed(uint8_t x, uint8_t y, float angle_rad, float magnitude);
    
    /**
     * @brief Helper function to draw distance value
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @param[in] distance Distance in meters
     * @details Renders distance in appropriate units (m, km, ft, mi, nm) based on magnitude
     */
    void draw_distance(uint8_t x, uint8_t y, float distance);
    
    /**
     * @brief Get font character index for directional arrow
     * @param[in] angle_cd Angle in centidegrees (0-36000)
     * @return Character index for arrow pointing in specified direction
     * @details Selects appropriate arrow character from font based on angle
     */
    char get_arrow_font_index (int32_t angle_cd);
#if HAL_WITH_ESC_TELEM
    /**
     * @brief Draw ESC temperature panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays temperature from selected ESC in configured units (°C or °F)
     * @note Requires ESC telemetry support (BLHeli_32, FETtec, DroneCAN, etc.)
     */
    void draw_esc_temp(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw ESC RPM panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays motor RPM from selected ESC telemetry
     * @note Requires ESC telemetry support
     */
    void draw_esc_rpm(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw ESC current panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays current draw from selected ESC in amperes (A)
     * @note Requires ESC telemetry support
     */
    void draw_esc_amps(uint8_t x, uint8_t y);
#endif

    /**
     * @brief Draw GPS latitude panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays current latitude in degrees with N/S indicator
     */
    void draw_gps_latitude(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw GPS longitude panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays current longitude in degrees with E/W indicator
     */
    void draw_gps_longitude(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw roll angle panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays vehicle roll angle in degrees (-180 to +180)
     */
    void draw_roll_angle(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw pitch angle panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays vehicle pitch angle in degrees (-90 to +90)
     */
    void draw_pitch_angle(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw primary temperature sensor panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays temperature from primary sensor in configured units (°C or °F)
     */
    void draw_temp(uint8_t x, uint8_t y);
    
#if BARO_MAX_INSTANCES > 1
    /**
     * @brief Draw barometer temperature panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays temperature from barometer sensor in configured units
     */
    void draw_btemp(uint8_t x, uint8_t y);
#endif

    /**
     * @brief Draw GPS HDOP (horizontal dilution of precision) panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays GPS accuracy metric, lower is better (typically 0.5-3.0)
     */
    void draw_hdop(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw active waypoint info panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays waypoint number, distance, and bearing in AUTO/GUIDED modes
     */
    void draw_waypoint(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw cross-track error panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays lateral distance from desired path in configured units
     */
    void draw_xtrack_error(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw distance traveled panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays total distance traveled since arming in configured units
     */
    void draw_dist(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw flight statistics panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays max altitude, speed, distance, and other flight statistics
     */
    void draw_stat(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw flight time panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays total armed flight time in HH:MM:SS format
     */
    void draw_flightime(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw climb efficiency panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays energy efficiency during climbing (mAh per meter climbed)
     */
    void draw_climbeff(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw overall efficiency panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays overall energy efficiency (mAh per distance unit)
     */
    void draw_eff(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw airspeed temperature panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays temperature from airspeed sensor in configured units
     */
    void draw_atemp(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw secondary battery voltage panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays voltage from second battery in volts (V)
     */
    void draw_bat2_vlt(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw secondary battery consumed panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays energy consumed from second battery in mAh
     */
    void draw_bat2used(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw clock/time panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays system time or GPS time in HH:MM:SS format
     */
    void draw_clk(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw callsign panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays vehicle callsign from parameters (for regulations/identification)
     */
    void draw_callsign(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw secondary battery current panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays current draw from second battery in amperes (A)
     */
    void draw_current2(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw VTX (video transmitter) power panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays video transmitter power level in mW or power index
     */
    void draw_vtx_power(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw height above terrain panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays height above ground from rangefinder or terrain database
     */
    void draw_hgt_abvterr(uint8_t x, uint8_t y);
    
#if AP_FENCE_ENABLED
    /**
     * @brief Draw fence status panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays geofence status and breach warnings
     */
    void draw_fence(uint8_t x, uint8_t y);
#endif

#if AP_RANGEFINDER_ENABLED
    /**
     * @brief Draw rangefinder distance panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays distance to ground from rangefinder in configured units
     */
    void draw_rngf(uint8_t x, uint8_t y);
#endif

#if AP_OSD_EXTENDED_LNK_STATS
    /**
     * @brief Check if Betaflight-style fonts are in use
     * @return true if backend uses Betaflight font layout
     * @details Used to select appropriate characters for extended link stats panels
     */
    bool is_btfl_fonts();
    
    /**
     * @brief Draw RC transmitter power panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays TX power level in mW (CRSF telemetry)
     * @note Requires CRSF or compatible protocol with extended telemetry
     */
    void draw_rc_tx_power(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw RC RSSI in dBm panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays received signal strength in dBm (typically -120 to -40)
     * @note Requires CRSF or compatible protocol with extended telemetry
     */
    void draw_rc_rssi_dbm(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw RC signal-to-noise ratio panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays SNR in dB (higher is better)
     * @note Requires CRSF or compatible protocol with extended telemetry
     */
    void draw_rc_snr(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw active antenna indicator panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays which diversity antenna is currently active
     * @note Requires CRSF or compatible protocol with extended telemetry
     */
    void draw_rc_active_antenna(uint8_t x, uint8_t y);
    
    /**
     * @brief Draw RC link quality panel
     * @param[in] x Horizontal position in character cells (0-29)
     * @param[in] y Vertical position in character cells (0-15)
     * @details Displays link quality as percentage (0-100%)
     * @note Requires CRSF or compatible protocol with extended telemetry
     */
    void draw_rc_lq(uint8_t x, uint8_t y);
#endif

    /**
     * @brief Callsign cache structure
     * @details Stores loaded callsign string to avoid repeated parameter lookups
     */
    struct {
        bool load_attempted;  ///< true if callsign load was attempted
        const char *str;      ///< Cached callsign string pointer
    } callsign_data;
};
#endif // OSD_ENABLED

#if OSD_PARAM_ENABLED
/**
 * @class AP_OSD_ParamSetting
 * @brief Configuration for one editable parameter in OSD parameter screen
 * 
 * @details Binds an ArduPilot parameter to an on-screen display position for
 *          interactive editing via RC stick inputs. Stores parameter location
 *          (group/key/index), display position, and editing constraints (min/max/increment).
 * 
 *          Parameter Identification:
 *          - _param_group: AP_Param group identifier
 *          - _param_key: Parameter key within group
 *          - _param_idx: Array index for vector parameters
 * 
 *          Editing Constraints:
 *          - _param_min/_param_max: Valid value range for editing
 *          - _param_incr: Step size for value adjustment
 *          - _type: Parameter type for specialized value selection (enums)
 * 
 *          Common Use Cases:
 *          - Flight mode channel assignments
 *          - PID tuning parameters
 *          - Failsafe configuration
 *          - Servo/motor output functions
 * 
 * @note Requires OSD_PARAM_ENABLED feature flag
 * @warning Editing flight-critical parameters in-flight can be dangerous
 * @see AP_OSD_ParamScreen for screen implementation
 */
class AP_OSD_ParamSetting
{
public:
    /**
     * @enum Type
     * @brief Specialized parameter types with predefined value sets
     * 
     * @details Provides dropdown-style selection for parameters with
     *          discrete meaningful values (flight modes, functions, etc.)
     */
    enum class Type : uint8_t {
        NONE = 0,               ///< Generic numeric parameter
        SERIAL_PROTOCOL    =  1,  ///< Serial port protocol selection
        SERVO_FUNCTION     =  2,  ///< Servo output function assignment
        AUX_FUNCTION       =  3,  ///< Auxiliary switch function
        FLIGHT_MODE        =  4,  ///< Flight mode number
        FAILSAFE_ACTION    =  5,  ///< Failsafe action selection
        FAILSAFE_ACTION_1  =  6,  ///< Failsafe action type 1
        FAILSAFE_ACTION_2  =  7,  ///< Failsafe action type 2
        NUM_TYPES          =  8,  ///< Count of defined types
    };

    AP_Int8 enabled;   ///< Parameter slot enabled flag: 0=hidden, 1=visible
    AP_Int8 xpos;      ///< Horizontal position in character cells (0-29)
    AP_Int8 ypos;      ///< Vertical position in character cells (0-15)

    // Parameter identification
    AP_Int32 _param_group;  ///< AP_Param group identifier
    AP_Int16 _param_key;    ///< Parameter key within group
    AP_Int8  _param_idx;    ///< Array index for vector parameters
    
    // Editing constraints
    AP_Float _param_min;    ///< Minimum valid value for editing
    AP_Float _param_max;    ///< Maximum valid value for editing
    AP_Float _param_incr;   ///< Increment step size for adjustments
    AP_Enum<Type> _type;    ///< Specialized type for enum-like parameters

    // Runtime parameter binding
    uint8_t _param_number;              ///< Parameter slot number in screen (0-8)
    AP_Param* _param;                   ///< Pointer to bound AP_Param instance
    ap_var_type _param_type;            ///< AP_Param type (INT8, INT16, FLOAT, etc.)
    AP_Param::ParamToken _current_token;  ///< Token for parameter access

    /**
     * @struct ParamMetadata
     * @brief Static metadata defining constraints for specialized parameter types
     * 
     * @details Provides predefined ranges, increments, and value names for
     *          parameters with discrete meaningful values (enums).
     */
    struct ParamMetadata {
        float min_value;        ///< Minimum valid value
        float max_value;        ///< Maximum valid value
        float increment;        ///< Step size for adjustments
        uint8_t values_max;     ///< Number of named values
        const char** values;    ///< Array of value name strings
    };

    /**
     * @struct Initializer
     * @brief Compact initialization structure for static parameter binding
     * 
     * @details Used to initialize parameter settings with known tokens
     *          at compile time for common configuration parameters.
     */
    struct Initializer {
        uint8_t index;                ///< Parameter slot index
        AP_Param::ParamToken token;   ///< Parameter token for binding
        Type type;                    ///< Specialized type
    };

    static const ParamMetadata _param_metadata[];  ///< Static metadata table

    /**
     * @brief Default constructor for empty parameter slot
     */
    AP_OSD_ParamSetting() {};
    
    /**
     * @brief Construct parameter setting with slot number
     * @param[in] param_number Slot index (0-8) in parameter screen
     */
    AP_OSD_ParamSetting(uint8_t param_number);
    
    /**
     * @brief Construct parameter setting from static initializer
     * @param[in] initializer Predefined parameter binding
     */
    AP_OSD_ParamSetting(const Initializer& initializer);

    /**
     * @brief Update parameter binding from configuration
     * 
     * @details Resolves parameter group/key/index to actual AP_Param instance
     *          and initializes constraints. Called when parameter screen activates.
     */
    void update();
    
    /**
     * @brief Copy parameter name to buffer
     * @param[out] name Buffer for parameter name
     * @param[in] len Buffer length in bytes
     * @details Retrieves parameter name (e.g., "SERVO1_FUNCTION") and truncates to 16 chars
     */
    void copy_name(char* name, size_t len) const {
        _param->copy_name_token(_current_token, name, len);
        if (len > 16) name[16] = 0;
    }
    
    /**
     * @brief Copy parameter name in camel case format
     * @param[out] name Buffer for formatted name
     * @param[in] len Buffer length in bytes
     * @details Converts "FOO_BAR_BAZ" to "FooBarBaz" for compact display
     */
    void copy_name_camel_case(char* name, size_t len) const;
    
    /**
     * @brief Set editing ranges from static metadata table
     * @return true if metadata found and applied
     * @details Looks up parameter type in metadata table and applies constraints
     */
    bool set_from_metadata();
    
    /**
     * @brief Bind parameter by name with custom constraints
     * @param[in] name Parameter name (e.g., "SERVO1_FUNCTION")
     * @param[in] config_type Specialized type enum value
     * @param[in] pmin Minimum value (0 = use default)
     * @param[in] pmax Maximum value (0 = use default)
     * @param[in] pincr Increment (0 = use default)
     * @return true if parameter found and bound successfully
     */
    bool set_by_name(const char* name, uint8_t config_type, float pmin=0, float pmax=0, float pincr=0);
    
    /**
     * @brief Automatically determine reasonable editing ranges
     * @param[in] force Force range guessing even if already set
     * @details Examines parameter type and current value to set reasonable min/max/increment
     */
    void guess_ranges(bool force = false);
    
    /**
     * @brief Save current configuration as new parameter settings
     * @details Writes bound parameter and constraints to AP_Param storage
     */
    void save_as_new();

    /**
     * @brief Get static metadata for specialized parameter type
     * @return Pointer to metadata structure, or nullptr if type is NONE
     */
    const ParamMetadata* get_custom_metadata() const {
        return (_type > 0 ? &_param_metadata[_type - 1] : nullptr);
    }

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];  ///< Parameter table for AP_Param

private:
    float default_enabled;
    float default_ypos;
    float default_param_group;
    float default_param_idx;
    float default_param_key;
    float default_type;

};

/**
 * @class AP_OSD_ParamScreen
 * @brief Interactive parameter editor screen for on-OSD configuration
 * 
 * @details Implements on-screen parameter editing interface allowing pilots to
 *          modify ArduPilot parameters using RC stick inputs without ground station.
 *          Displays up to 9 configurable parameters with current values and allows
 *          navigation and editing via RC channel inputs mapped to menu events.
 * 
 *          State Machine:
 *          - PARAM_SELECT: Navigate between parameters with up/down
 *          - PARAM_VALUE_MODIFY: Adjust selected parameter value
 *          - PARAM_PARAM_MODIFY: Reconfigure which parameter is bound to slot
 * 
 *          User Interface:
 *          - Parameter name displayed with current value
 *          - Highlighted parameter shows selected item
 *          - Value modification with configurable increment
 *          - Save button to persist changes
 * 
 *          RC Input Mapping:
 *          - Pitch stick: Navigate up/down between parameters
 *          - Roll stick: Adjust parameter value (in modify mode)
 *          - Yaw stick: Enter/exit modify mode
 *          - Configurable button delays prevent accidental changes
 * 
 *          Safety Features:
 *          - Parameter validation against min/max ranges
 *          - Unsaved changes indicator
 *          - Timeout returns to select mode
 *          - Changes require explicit save action
 * 
 * @note Requires OSD_PARAM_ENABLED feature flag
 * @warning In-flight parameter changes can affect vehicle stability
 * @see AP_OSD_ParamSetting for parameter configuration
 */
class AP_OSD_ParamScreen : public AP_OSD_AbstractScreen
{
public:
    /**
     * @brief Construct parameter editor screen
     * @param[in] index Screen index (0 or 1 for two param screens)
     */
    AP_OSD_ParamScreen(uint8_t index);

    /**
     * @enum Event
     * @brief RC input events for parameter screen navigation
     * 
     * @details Events generated from RC stick positions and mapped
     *          to menu actions by map_rc_input_to_event()
     */
    enum class Event {
        NONE,        ///< No event (sticks centered)
        MENU_ENTER,  ///< Enter edit mode / confirm selection
        MENU_UP,     ///< Navigate up / increase value
        MENU_DOWN,   ///< Navigate down / decrease value
        MENU_EXIT    ///< Exit edit mode / cancel
    };

    /**
     * @enum MenuState
     * @brief State machine states for parameter editing
     * 
     * @details Three-level editing: select parameter, modify value, reconfigure binding
     */
    enum class MenuState {
        PARAM_SELECT,        ///< Selecting which parameter to edit
        PARAM_VALUE_MODIFY,  ///< Modifying parameter value
        PARAM_PARAM_MODIFY   ///< Reconfiguring parameter binding
    };

    static const uint8_t NUM_PARAMS = 9;            ///< Number of parameter slots per screen
    static const uint8_t SAVE_PARAM = NUM_PARAMS + 1;  ///< Save button index (10)

    /**
     * @brief Render parameter editor screen
     * 
     * @details Displays parameter list with names, values, and current selection.
     *          Highlights selected parameter and shows save button with modified indicator.
     *          Updates state machine and processes RC inputs.
     * 
     * @note Called from OSD thread at backend update rate
     */
#if OSD_ENABLED && (HAL_WITH_OSD_BITMAP || HAL_WITH_MSP_DISPLAYPORT)
    void draw(void) override;
#endif

#if HAL_GCS_ENABLED
    /**
     * @brief Handle MAVLink parameter configuration message
     * @param[in] packet OSD parameter configuration message
     * @param[in] link MAVLink link that received the message
     * @details Allows ground station to configure parameter screen bindings remotely
     */
    void handle_write_msg(const mavlink_osd_param_config_t& packet, const class GCS_MAVLINK& link);
    
    /**
     * @brief Handle MAVLink parameter query message
     * @param[in] packet OSD parameter query message
     * @param[in] link MAVLink link that received the message
     * @details Sends current parameter screen configuration to ground station
     */
    void handle_read_msg(const mavlink_osd_param_show_config_t& packet, const class GCS_MAVLINK& link);
#endif

    /**
     * @brief Get parameter setting by slot index
     * @param[in] param_idx Parameter slot number (0-8)
     * @return Pointer to parameter setting, or nullptr if invalid index
     */
    AP_OSD_ParamSetting* get_setting(uint8_t param_idx);
    
    AP_Int8 save_x;  ///< Save button horizontal position (character cells)
    AP_Int8 save_y;  ///< Save button vertical position (character cells)

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];  ///< Parameter table for AP_Param

private:
    AP_OSD_ParamSetting params[NUM_PARAMS];  ///< Array of 9 parameter slots

    /**
     * @brief Save all modified parameters to persistent storage
     * @details Writes parameter changes to EEPROM and clears requires_save flag
     */
    void save_parameters();
    
#if OSD_ENABLED
    /**
     * @brief Update editing state machine based on RC events
     * @details Processes RC input events and transitions between select/modify states
     */
    void update_state_machine();
    
    /**
     * @brief Draw single parameter line on screen
     * @param[in] param_number Parameter slot index (0-8)
     * @param[in] x Horizontal position in character cells
     * @param[in] y Vertical position in character cells
     * @details Renders parameter name and current value with highlight if selected
     */
    void draw_parameter(uint8_t param_number, uint8_t x, uint8_t y);
    
    /**
     * @brief Modify parameter value based on RC input event
     * @param[in] number Parameter slot index
     * @param[in] ev RC input event (MENU_UP/MENU_DOWN for increment/decrement)
     * @details Adjusts parameter value by configured increment within min/max range
     */
    void modify_parameter(uint8_t number, Event ev);
    
    /**
     * @brief Modify parameter binding configuration
     * @param[in] number Parameter slot index
     * @param[in] ev RC input event for binding navigation
     * @details Allows changing which ArduPilot parameter is bound to this slot
     */
    void modify_configured_parameter(uint8_t number, Event ev);

#if AP_RC_CHANNEL_ENABLED
    /**
     * @brief Map RC stick positions to menu events
     * @return Event corresponding to current RC input
     * @details Reads configured RC channel and maps stick deflections to menu actions
     * @note Uses button delay timeout to prevent accidental rapid changes
     */
    Event map_rc_input_to_event() const;
#endif

    uint8_t _selected_param = 1;                      ///< Currently selected parameter index (1-10, 10=save)
    MenuState _menu_state = MenuState::PARAM_SELECT;  ///< Current state machine state
    Event _last_rc_event = Event::NONE;               ///< Last processed RC event for edge detection

    uint32_t _transition_start_ms;    ///< Start time of current button press (ms)
    uint32_t _transition_timeout_ms;  ///< Timeout for current transition (ms)
    uint32_t _transition_count;       ///< Number of consecutive identical transitions
#endif
    uint16_t _requires_save;  ///< Bitmask of modified parameters requiring save
};

#endif // OSD_PARAM_ENABLED

/**
 * @class AP_OSD
 * @brief Main On-Screen Display manager coordinating multiple OSD backends and screens
 * 
 * @details Singleton manager class implementing ArduPilot's OSD subsystem with support
 *          for multiple display backends (MAX7456 analog, MSP DisplayPort, SITL simulation).
 *          Manages up to 4 display screens and 2 parameter editor screens, coordinates
 *          screen switching via RC input, and runs dedicated OSD thread for display updates.
 * 
 *          Architecture:
 *          - Singleton manager coordinating multiple backends
 *          - Dedicated thread for OSD updates (osd_thread)
 *          - Screen abstraction supporting multiple display types
 *          - Backend abstraction for hardware independence
 * 
 *          Supported Backends:
 *          - MAX7456: Analog video overlay chip (font-based)
 *          - MSP: Betaflight/iNav MSP protocol OSD
 *          - MSP_DISPLAYPORT: Digital MSP DisplayPort protocol
 *          - SITL: Software simulation OSD
 *          - TXONLY: Transmitter-based OSD (parameter config only)
 * 
 *          Screen Management:
 *          - 4 display screens (configurable panels)
 *          - 2 parameter editor screens (optional)
 *          - RC channel switching (toggle/PWM range/auto-switch)
 *          - Automatic switching on arm/disarm/failsafe events
 * 
 *          Display Update Flow:
 *          1. osd_thread() runs continuously in dedicated thread
 *          2. update_osd() called at backend-specific rate
 *          3. update_current_screen() handles screen switching logic
 *          4. current screen->draw() renders all enabled panels
 *          5. Backend flushes display buffer to hardware
 * 
 *          Thread Safety:
 *          - HAL_Semaphore protects concurrent access from multiple threads
 *          - Statistics updates protected by semaphore
 *          - Screen switching synchronized with display updates
 * 
 *          Statistics Tracking:
 *          - Maximum values: altitude, distance, speed, current, ESC temp
 *          - Minimum values: voltage, RSSI
 *          - Average current consumption
 *          - Flight time and efficiency metrics
 * 
 *          Unit System Support:
 *          - METRIC: meters, km/h, Celsius
 *          - IMPERIAL: feet, mph, Fahrenheit
 *          - SI: meters, m/s, Celsius
 *          - AVIATION: feet, knots, Celsius
 * 
 * @note Runs in dedicated thread to minimize latency impact on flight control
 * @warning OSD updates must complete within backend refresh interval (typically 20-50Hz)
 * @see AP_OSD_Backend for hardware abstraction interface
 * @see AP_OSD_Screen for display panel implementation
 */
class AP_OSD
{
public:
    friend class AP_OSD_Screen;
    friend class AP_MSP;
    friend class AP_MSP_Telem_Backend;
    friend class AP_OSD_ParamScreen;
    
    /**
     * @brief Construct OSD manager singleton
     * @details Initializes backend array, screen objects, and default parameters
     */
    AP_OSD();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OSD);

    /**
     * @brief Get singleton instance
     * @return Pointer to singleton AP_OSD instance, or nullptr if not initialized
     */
    static AP_OSD *get_singleton()
    {
        return _singleton;
    }

    /**
     * @brief Initialize OSD subsystem and start dedicated thread
     * 
     * @details Performs OSD initialization sequence:
     *          1. Creates backend instances based on osd_type parameters
     *          2. Initializes screen objects and parameter bindings
     *          3. Starts dedicated osd_thread() for display updates
     *          4. Registers with scheduler for statistics updates
     * 
     * @note Must be called after AP_Param::load_all() completes
     * @warning Called from vehicle init - must not block for extended time
     */
    void init();

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];  ///< Parameter table for AP_Param

    /**
     * @enum osd_types
     * @brief Supported OSD backend types
     * 
     * @details Defines available hardware/protocol backends for OSD display.
     *          Multiple backends can run simultaneously (osd_type and osd_type2).
     */
    enum osd_types {
        OSD_NONE=0,            ///< No OSD backend (disabled)
        OSD_MAX7456=1,         ///< MAX7456 analog video overlay chip
        OSD_SITL=2,            ///< Software-in-the-loop simulation OSD
        OSD_MSP=3,             ///< MSP protocol OSD (Betaflight/iNav style)
        OSD_TXONLY=4,          ///< Transmitter-based OSD (parameter config only, no display)
        OSD_MSP_DISPLAYPORT=5  ///< MSP DisplayPort digital protocol (DJI, HDZero)
    };

    /**
     * @brief Initialize OSD backend instance
     * @param[in] type Backend type to initialize
     * @param[in] instance Backend instance number (0 or 1 for dual OSD)
     * @return true if backend initialized successfully, false on failure
     * 
     * @details Creates backend object, initializes hardware, and registers with manager.
     *          Fails gracefully if hardware not detected or initialization errors occur.
     */
    bool init_backend(const osd_types type, const uint8_t instance);

    /**
     * @enum switch_method
     * @brief Screen switching methods via RC input
     * 
     * @details Configures how RC channel input controls screen selection:
     *          - TOGGLE: Each high pulse switches to next screen
     *          - PWM_RANGE: PWM value directly maps to screen number
     *          - AUTO_SWITCH: Automatic switching based on flight events
     */
    enum switch_method {
        TOGGLE=0,      ///< Toggle through screens on RC channel high pulse
        PWM_RANGE=1,   ///< Map RC PWM value to screen (e.g., 1000-1250=screen1, 1250-1500=screen2)
        AUTO_SWITCH=2  ///< Automatic screen switching on arm/disarm/failsafe
    };

    AP_Int8 osd_type;   ///< Primary OSD backend type (enum osd_types)
    AP_Int8 osd_type2;  ///< Secondary OSD backend active in parallel (enum osd_types)
    AP_Int8 font_num;   ///< Font selection for character-based displays (0=default)
    AP_Int32 options;   ///< Option flags bitmask for display behavior (OPTION_*)

#if OSD_ENABLED
    AP_Int8 rc_channel;  ///< RC channel for screen switching (0=disabled, 1-16=channel number)
    AP_Int8 sw_method;   ///< Screen switching method (enum switch_method)

    AP_Int8 v_offset;  ///< Vertical display offset in character rows (-15 to 15)
    AP_Int8 h_offset;  ///< Horizontal display offset in character columns (-15 to 15)

    AP_Int8 warn_rssi;          ///< RSSI warning threshold (% or dBm depending on link stats)
    AP_Int8 warn_nsat;          ///< GPS satellite count warning threshold (number of satellites)
    AP_Int32 warn_terr;         ///< Terrain altitude warning threshold in centimeters
    AP_Float warn_avgcellvolt;  ///< Average cell voltage warning threshold in volts
    AP_Float max_battery_voltage;  ///< Maximum battery voltage for percentage calculation in volts
    AP_Int8 cell_count;         ///< Battery cell count override (0=auto-detect, 1-12=fixed count)
    AP_Float warn_restvolt;     ///< Resting voltage warning threshold in volts
    AP_Float warn_avgcellrestvolt;  ///< Average resting cell voltage warning in volts
    AP_Float warn_batvolt;      ///< Battery voltage warning threshold for primary battery in volts
    AP_Float warn_bat2volt;     ///< Battery voltage warning threshold for secondary battery in volts
    AP_Int8 msgtime_s;          ///< Message display duration in seconds (0=until cleared)
    AP_Int8 arm_scr;            ///< Screen to display when vehicle arms (-1=no change, 0-3=screen)
    AP_Int8 disarm_scr;         ///< Screen to display when vehicle disarms (-1=no change, 0-3=screen)
    AP_Int8 failsafe_scr;       ///< Screen to display during failsafe (-1=no change, 0-3=screen)
    AP_Int32 button_delay_ms;   ///< RC input button delay in milliseconds for parameter editor

#if AP_OSD_EXTENDED_LNK_STATS
    AP_Int8 warn_lq;   ///< Link quality warning threshold in percent (CRSF link stats)
    AP_Int8 warn_snr;  ///< Signal-to-noise ratio warning threshold in dB (CRSF link stats)
#endif

#if HAL_OSD_SIDEBAR_ENABLE
    AP_Int8 sidebar_h_offset;  ///< Horizontal offset for artificial horizon sidebars
    AP_Int8 sidebar_v_ext;     ///< Vertical extension for sidebars (character rows)
#endif

    /**
     * @brief Display option flags bitmask
     * 
     * @details Configuration flags modifying OSD display behavior and appearance.
     *          Multiple options can be combined with bitwise OR.
     */
    enum {
        OPTION_DECIMAL_PACK = 1U<<0,         ///< Pack decimal values (12.3 vs 12.30) to save space
        OPTION_INVERTED_WIND = 1U<<1,        ///< Display wind as heading to fly, not wind from direction
        OPTION_INVERTED_AH_ROLL = 1U<<2,     ///< Invert artificial horizon roll display
        OPTION_IMPERIAL_MILES = 1U<<3,       ///< Use miles instead of nautical miles for imperial units
        OPTION_DISABLE_CROSSHAIR = 1U<<4,    ///< Hide center crosshair on screen
        OPTION_BF_ARROWS = 1U<<5,            ///< Use Betaflight-style arrow symbols
        OPTION_AVIATION_AH = 1U<<6,          ///< Use aviation-style artificial horizon (sky moves)
#if AP_OSD_EXTENDED_LNK_STATS
        OPTION_RF_MODE_ALONG_WITH_LQ = 1U<<7, ///< Display RF mode alongside link quality
#endif
    };

    /**
     * @brief Unit system selection
     * 
     * @details Defines measurement unit systems for OSD display.
     *          Affects altitude, speed, distance, and temperature units.
     */
    enum {
        UNITS_METRIC=0,    ///< Metric: meters, km/h, Celsius
        UNITS_IMPERIAL=1,  ///< Imperial: feet, mph, Fahrenheit
        UNITS_SI=2,        ///< SI: meters, m/s, Celsius
        UNITS_AVIATION=3,  ///< Aviation: feet, knots, Celsius
        UNITS_LAST=4,      ///< Sentinel value for unit count
    };

    AP_Int8 units;  ///< Unit system selection (enum UNITS_*)

    AP_OSD_Screen screen[AP_OSD_NUM_DISPLAY_SCREENS];  ///< Array of 4 display screens

    /**
     * @struct NavInfo
     * @brief Navigation information for waypoint display
     * 
     * @details Waypoint navigation data passed from vehicle navigation
     *          system to OSD for display on waypoint panels.
     */
    struct NavInfo {
        float wp_distance;      ///< Distance to active waypoint in meters
        int32_t wp_bearing;     ///< Bearing to waypoint in centidegrees (0-35999)
        float wp_xtrack_error;  ///< Cross-track error in meters (perpendicular distance from line)
        uint16_t wp_number;     ///< Current waypoint sequence number
    };

    /**
     * @struct StatsInfo
     * @brief Flight statistics tracking max/min/average values
     * 
     * @details Tracks flight statistics for display on statistics panel.
     *          Updated continuously during flight, reset on disarm.
     *          Used to show peak performance values and efficiency metrics.
     */
    struct StatsInfo {
        uint32_t last_update_ms;    ///< Last statistics update timestamp (milliseconds)
        float last_distance_m;      ///< Distance at last update for efficiency calculation
        float max_dist_m;           ///< Maximum distance from home in meters
        float max_alt_m;            ///< Maximum altitude in meters
        float max_speed_mps;        ///< Maximum ground speed in m/s
        float max_airspeed_mps;     ///< Maximum airspeed in m/s (fixed-wing)
        float max_current_a;        ///< Maximum current draw in amperes
        float avg_current_a;        ///< Average current consumption in amperes
        float min_voltage_v = FLT_MAX;  ///< Minimum battery voltage in volts
        float min_rssi = FLT_MAX;   ///< Minimum RSSI value (0-1 range)
        int16_t max_esc_temp;       ///< Maximum ESC temperature in Celsius
    };

    /**
     * @brief Update navigation information for waypoint display
     * @param[in] nav_info Navigation info structure with waypoint data
     * @details Called by vehicle navigation system to provide waypoint information
     *          for OSD display. Data includes distance, bearing, cross-track error,
     *          and waypoint number.
     */
    void set_nav_info(NavInfo &nav_info);
    
    /**
     * @brief Get current flight statistics
     * @return Constant reference to statistics structure
     * @details Provides access to max/min/average flight values for external use.
     *          Statistics are updated continuously during flight.
     * @note Returns volatile reference for thread-safe access
     */
    const volatile StatsInfo& get_stats_info() const {return _stats;};
    
    /**
     * @brief Disable OSD display
     * @details Temporarily disables OSD rendering without stopping backend.
     *          Useful for hiding OSD during certain flight conditions.
     */
    void disable() {
        _disable = true;
    }
    
    /**
     * @brief Enable OSD display
     * @details Re-enables OSD rendering after disable() call
     */
    void enable() {
        _disable = false;
    }

    /**
     * @brief Get screen object by index
     * @param[in] idx Screen index (0-3 for display screens, 4-5 for param screens)
     * @return Reference to screen object (AP_OSD_Screen or AP_OSD_ParamScreen)
     * @details Provides access to screen objects for external configuration
     */
    AP_OSD_AbstractScreen& get_screen(uint8_t idx) {
#if OSD_PARAM_ENABLED
        if (idx >= AP_OSD_NUM_DISPLAY_SCREENS) {
            return param_screen[idx - AP_OSD_NUM_DISPLAY_SCREENS];
        }
#endif
        return screen[idx];
    }

    /**
     * @brief Pre-arm safety check for OSD system
     * @param[out] failure_msg Buffer for failure message string
     * @param[in] failure_msg_len Maximum length of failure message buffer
     * @return true if OSD passes pre-arm checks, false if unsafe to arm
     * 
     * @details Verifies OSD system is ready for flight:
     *          - Backend initialized successfully
     *          - Display communication functioning
     *          - Critical panels configured if required
     * 
     * @warning Vehicle will refuse to arm if this check fails
     */
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;
    
    /**
     * @brief Check if current screen is read-only display screen
     * @return true if displaying flight data screen, false if parameter editor
     * @details Parameter screens are editable, display screens are read-only
     */
    bool is_readonly_screen() const { return current_screen < AP_OSD_NUM_DISPLAY_SCREENS; }
    
    /**
     * @brief Get currently active screen index
     * @return Current screen number (0-5)
     */
    uint8_t get_current_screen() const { return current_screen; };
#endif // OSD_ENABLED
#if OSD_PARAM_ENABLED
    AP_OSD_ParamScreen param_screen[AP_OSD_NUM_PARAM_SCREENS] { 0, 1 };  ///< Parameter editor screens (indices 0 and 1)
    
    /**
     * @brief Get parameter setting for transmitter-based OSD configuration
     * @param[in] screen_idx Parameter screen index (0 or 1)
     * @param[in] param_idx Parameter slot index (0-8)
     * @return Pointer to parameter setting, or nullptr if invalid indices
     * 
     * @details Provides access to parameter bindings for external configuration
     *          via transmitter-based OSD systems (TXONLY backend type).
     */
    AP_OSD_ParamSetting* get_setting(uint8_t screen_idx, uint8_t param_idx) {
        if (screen_idx >= AP_OSD_NUM_PARAM_SCREENS) {
            return nullptr;
        }
        return param_screen[screen_idx].get_setting(param_idx);
    }
#endif

#if HAL_GCS_ENABLED
    /**
     * @brief Handle MAVLink messages for OSD configuration
     * @param[in] msg MAVLink message to process
     * @param[in] link MAVLink link that received the message
     * 
     * @details Processes OSD-related MAVLink messages:
     *          - OSD_PARAM_CONFIG: Configure parameter screen bindings
     *          - OSD_PARAM_SHOW_CONFIG: Query current parameter configuration
     * 
     * @note Called from GCS message handling thread
     */
    void handle_msg(const mavlink_message_t &msg, const class GCS_MAVLINK& link);
#endif

    /**
     * @brief Get semaphore for thread-safe OSD access
     * @return Reference to HAL_Semaphore for OSD operations
     * 
     * @details Provides semaphore to protect concurrent access to OSD data
     *          from multiple threads. Must be held when accessing statistics,
     *          navigation info, or screen state from external threads.
     * 
     * @warning Always use WITH_SEMAPHORE macro to ensure proper cleanup:
     *          WITH_SEMAPHORE(osd->get_semaphore());
     * 
     * @note OSD thread already holds semaphore during display updates
     */
    HAL_Semaphore &get_semaphore(void) {
        return _sem;
    }

private:
    /**
     * @brief Main OSD thread function running continuously
     * 
     * @details Dedicated thread for OSD display updates that runs independently
     *          of main vehicle thread. Calls update_osd() at backend-specific rate
     *          to minimize latency impact on flight control loops.
     * 
     * @note Runs at lower priority than flight control
     * @warning Must complete within backend refresh interval
     */
    void osd_thread();
    
#if OSD_ENABLED
    /**
     * @brief Update OSD display for current frame
     * 
     * @details Main OSD update function called from osd_thread:
     *          1. Acquires semaphore for thread safety
     *          2. Updates screen switching logic
     *          3. Updates flight statistics
     *          4. Calls current screen draw() method
     *          5. Flushes display buffers to backends
     * 
     * @note Called at backend-specific rate (typically 20-50Hz)
     */
    void update_osd();
    
    /**
     * @brief Update flight statistics tracking
     * @details Updates max/min/average values in StatsInfo structure.
     *          Tracks altitude, speed, current, voltage, RSSI, temperature.
     *          Called from update_osd() every display frame.
     */
    void update_stats();
    
    /**
     * @brief Update current screen based on RC input and flight events
     * @details Handles screen switching logic:
     *          - RC channel input (toggle/PWM range/auto-switch)
     *          - Automatic switching on arm/disarm events
     *          - Failsafe screen activation
     *          - Screen restore after failsafe clears
     */
    void update_current_screen();
    
    /**
     * @brief Switch to next screen in sequence
     * @details Advances to next screen with wrap-around from last to first.
     *          Used by toggle screen switching mode.
     */
    void next_screen();

    // Variables for screen switching
    uint8_t current_screen;         ///< Currently displayed screen index (0-5)
    uint16_t previous_channel_value; ///< Previous RC channel value for edge detection
    bool switch_debouncer;          ///< Debounce flag to prevent rapid switching
    uint32_t last_switch_ms;        ///< Timestamp of last screen switch (milliseconds)
    struct NavInfo nav_info;        ///< Current waypoint navigation information
    int8_t previous_pwm_screen;     ///< Screen before PWM-based switch (-1=invalid)
    int8_t pre_fs_screen;           ///< Screen before failsafe activation (-1=invalid)
    bool was_armed;                 ///< Previous armed state for arm event detection
    bool was_failsafe;              ///< Previous failsafe state for failsafe event detection
    bool _disable;                  ///< OSD display disable flag

    StatsInfo _stats;  ///< Flight statistics structure
#endif
    AP_OSD_Backend *_backends[OSD_MAX_INSTANCES];  ///< Array of active backend pointers
    uint8_t _backend_count;  ///< Number of initialized backends (0-2)

    static AP_OSD *_singleton;  ///< Singleton instance pointer
    HAL_Semaphore _sem;  ///< Multi-thread access protection semaphore
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP
{
/**
 * @brief Get OSD singleton instance
 * @return Pointer to AP_OSD singleton, or nullptr if not initialized
 * 
 * @details Convenience accessor for OSD singleton instance.
 *          Equivalent to AP_OSD::get_singleton().
 * 
 * Example usage:
 * @code
 * AP_OSD *osd = AP::osd();
 * if (osd != nullptr) {
 *     WITH_SEMAPHORE(osd->get_semaphore());
 *     const auto& stats = osd->get_stats_info();
 *     // Use stats...
 * }
 * @endcode
 */
AP_OSD *osd();
};
