/**
 * @file msp_osd.h
 * @brief MSP OSD item enumerations and configuration structures
 * 
 * @details This header defines the OSD (On-Screen Display) protocol elements for
 *          MSP (MultiWii Serial Protocol) compatible OSD systems. It includes
 *          enumerations for all displayable OSD items, post-flight statistics,
 *          warning conditions, and configuration structures that are compatible
 *          with Betaflight and INAV OSD implementations.
 *          
 *          The MSP OSD protocol allows flight controllers to communicate with
 *          external OSD hardware (e.g., MAX7456-based OSDs) or digital video
 *          systems that support MSP DisplayPort protocol for rendering telemetry
 *          overlays on FPV video feeds.
 * 
 * @note Protocol compatibility is maintained with Betaflight/INAV OSD systems
 * @warning Changing enum values or ordering will break OSD protocol compatibility
 */
#pragma once

#include <stdint.h>

#if HAL_MSP_ENABLED

/**
 * @brief OSD feature flag for MSP capability reporting
 * 
 * @details This flag is set in the MSP_API_VERSION response to indicate that
 *          the flight controller supports OSD functionality via MSP protocol.
 *          Ground control stations and OSD configurators check this flag to
 *          determine if OSD configuration commands are available.
 */
#define OSD_FLAGS_OSD_FEATURE           (1U << 0)

/**
 * @namespace MSP
 * @brief MSP protocol OSD-related type definitions
 * 
 * @details This namespace contains all type definitions for MSP OSD protocol
 *          implementation, including item enumerations, statistics, warning
 *          flags, and configuration structures. These definitions maintain
 *          protocol compatibility with Betaflight and INAV OSD implementations.
 */
namespace MSP
{
/**
 * @enum osd_items_e
 * @brief Defines all displayable OSD items compatible with Betaflight/INAV OSDs
 * 
 * @details This enumeration defines every possible OSD element that can be displayed
 *          on MSP-compatible OSD hardware. Each enum value corresponds to a specific
 *          telemetry item or visual element. The ordering and count of items must
 *          remain fixed to maintain protocol compatibility with Betaflight and INAV
 *          ground control software and OSD configurators.
 *          
 *          OSD items can be positioned independently on the display canvas, and each
 *          item's visibility and position is configured via MSP_SET_OSD_CONFIG and
 *          MSP_SET_OSD_CANVAS commands.
 * 
 * @note OSD_ITEM_COUNT must be the last enum value and exactly equals 58 items
 * @warning Do not reorder, add, or remove items without updating protocol version
 *          and maintaining backward compatibility with existing OSD hardware
 */
typedef enum {
    OSD_RSSI_VALUE,                 ///< Received Signal Strength Indicator (0-100%)
    OSD_MAIN_BATT_VOLTAGE,          ///< Main battery voltage in volts
    OSD_CROSSHAIRS,                 ///< Center crosshair for aiming or camera alignment
    OSD_ARTIFICIAL_HORIZON,         ///< Artificial horizon attitude indicator showing pitch and roll
    OSD_HORIZON_SIDEBARS,           ///< Vertical sidebars for horizon reference
    OSD_ITEM_TIMER_1,               ///< User-configurable timer 1 (flight time, armed time, etc.)
    OSD_ITEM_TIMER_2,               ///< User-configurable timer 2
    OSD_FLYMODE,                    ///< Current flight mode (e.g., STABILIZE, LOITER, AUTO)
    OSD_CRAFT_NAME,                 ///< Vehicle/craft name as configured in parameters
    OSD_THROTTLE_POS,               ///< Throttle position (0-100%)
    OSD_VTX_CHANNEL,                ///< Video transmitter channel and band information
    OSD_CURRENT_DRAW,               ///< Current battery current draw in amperes
    OSD_MAH_DRAWN,                  ///< Cumulative battery capacity used in mAh
    OSD_GPS_SPEED,                  ///< GPS ground speed (units depend on OSD configuration)
    OSD_GPS_SATS,                   ///< Number of GPS satellites locked and HDOP indicator
    OSD_ALTITUDE,                   ///< Barometric or GPS altitude above home (units depend on configuration)
    OSD_ROLL_PIDS,                  ///< Roll axis PID values for tuning visualization
    OSD_PITCH_PIDS,                 ///< Pitch axis PID values for tuning visualization
    OSD_YAW_PIDS,                   ///< Yaw axis PID values for tuning visualization
    OSD_POWER,                      ///< Electrical power consumption in watts
    OSD_PIDRATE_PROFILE,            ///< Current PID and rate profile number
    OSD_WARNINGS,                   ///< Active warning messages (see osd_warnings_flags_e)
    OSD_AVG_CELL_VOLTAGE,           ///< Average battery cell voltage (total voltage / cell count)
    OSD_GPS_LON,                    ///< GPS longitude coordinate
    OSD_GPS_LAT,                    ///< GPS latitude coordinate
    OSD_DEBUG,                      ///< Debug values for development and troubleshooting
    OSD_PITCH_ANGLE,                ///< Vehicle pitch angle in degrees
    OSD_ROLL_ANGLE,                 ///< Vehicle roll angle in degrees
    OSD_MAIN_BATT_USAGE,            ///< Battery usage as graphical bar indicator
    OSD_DISARMED,                   ///< Disarmed indicator message
    OSD_HOME_DIR,                   ///< Direction arrow pointing toward home position
    OSD_HOME_DIST,                  ///< Distance to home position
    OSD_NUMERICAL_HEADING,          ///< Compass heading in degrees (0-359°)
    OSD_NUMERICAL_VARIO,            ///< Vertical speed indicator (climb/descent rate)
    OSD_COMPASS_BAR,                ///< Compass heading bar showing cardinal directions
    OSD_ESC_TMP,                    ///< Electronic Speed Controller temperature
    OSD_ESC_RPM,                    ///< Motor RPM from ESC telemetry
    OSD_REMAINING_TIME_ESTIMATE,    ///< Estimated remaining flight time based on battery
    OSD_RTC_DATETIME,               ///< Real-time clock date and time
    OSD_ADJUSTMENT_RANGE,           ///< In-flight adjustment range indicator
    OSD_CORE_TEMPERATURE,           ///< Flight controller core temperature
    OSD_ANTI_GRAVITY,               ///< Anti-gravity mode status indicator
    OSD_G_FORCE,                    ///< G-force measurement (acceleration in Gs)
    OSD_MOTOR_DIAG,                 ///< Motor diagnostics display for testing
    OSD_LOG_STATUS,                 ///< Data logging status (armed/disarmed, log number)
    OSD_FLIP_ARROW,                 ///< Flip direction arrow for acro flip mode
    OSD_LINK_QUALITY,               ///< Radio link quality indicator (LQ)
    OSD_FLIGHT_DIST,                ///< Total distance traveled during flight
    OSD_STICK_OVERLAY_LEFT,         ///< Left stick position overlay visualization
    OSD_STICK_OVERLAY_RIGHT,        ///< Right stick position overlay visualization
    OSD_DISPLAY_NAME,               ///< Display device name
    OSD_ESC_RPM_FREQ,               ///< ESC RPM as frequency (for FFT analysis)
    OSD_RATE_PROFILE_NAME,          ///< Current rate profile name
    OSD_PID_PROFILE_NAME,           ///< Current PID profile name
    OSD_PROFILE_NAME,               ///< Combined profile name
    OSD_RSSI_DBM_VALUE,             ///< RSSI in dBm units (signal strength)
    OSD_RC_CHANNELS,                ///< RC channel values display
    OSD_CAMERA_FRAME,               ///< Camera frame indicator
    OSD_ITEM_COUNT                  ///< Count of OSD items - MUST BE LAST
} osd_items_e;

/**
 * @brief Protocol compatibility assertion for OSD item count
 * 
 * @details This static assertion enforces that exactly 58 OSD items are defined,
 *          maintaining protocol compatibility with Betaflight and INAV OSD
 *          implementations. If OSD_ITEM_COUNT != 58, compilation will fail,
 *          preventing protocol version mismatches.
 */
static_assert(OSD_ITEM_COUNT == 58, "OSD_ITEM_COUNT != 58");

/**
 * @enum osd_stats_e
 * @brief Post-flight statistics displayed on OSD after disarming
 * 
 * @details This enumeration defines statistics items that are displayed on the OSD
 *          after the vehicle is disarmed, providing a summary of the flight session.
 *          Statistics can include maximum/minimum values recorded during flight,
 *          cumulative totals, and session information. These are typically displayed
 *          as a summary screen before the OSD returns to the normal display.
 *          
 *          Individual statistics can be enabled or disabled via the enabled_stats
 *          bitfield in osd_config_t structure, allowing users to customize which
 *          post-flight information is displayed.
 * 
 * @note OSD_STAT_COUNT must be the last enum value
 * @see osd_config_t::enabled_stats for configuration bitfield
 */
typedef enum {
    OSD_STAT_RTC_DATE_TIME,        ///< Flight date and time from real-time clock
    OSD_STAT_TIMER_1,              ///< Timer 1 final value
    OSD_STAT_TIMER_2,              ///< Timer 2 final value
    OSD_STAT_MAX_SPEED,            ///< Maximum speed achieved during flight
    OSD_STAT_MAX_DISTANCE,         ///< Maximum distance from home position
    OSD_STAT_MIN_BATTERY,          ///< Minimum battery voltage recorded during flight
    OSD_STAT_END_BATTERY,          ///< Battery voltage at end of flight (disarm)
    OSD_STAT_BATTERY,              ///< Battery statistics summary
    OSD_STAT_MIN_RSSI,             ///< Minimum RSSI value during flight
    OSD_STAT_MAX_CURRENT,          ///< Maximum current draw in amperes
    OSD_STAT_USED_MAH,             ///< Total battery capacity consumed in mAh
    OSD_STAT_MAX_ALTITUDE,         ///< Maximum altitude reached during flight
    OSD_STAT_BLACKBOX,             ///< Blackbox logging status
    OSD_STAT_BLACKBOX_NUMBER,      ///< Blackbox log file number
    OSD_STAT_MAX_G_FORCE,          ///< Maximum G-force experienced
    OSD_STAT_MAX_ESC_TEMP,         ///< Maximum ESC temperature recorded
    OSD_STAT_MAX_ESC_RPM,          ///< Maximum motor RPM achieved
    OSD_STAT_MIN_LINK_QUALITY,     ///< Minimum radio link quality percentage
    OSD_STAT_FLIGHT_DISTANCE,      ///< Total distance traveled during flight
    OSD_STAT_MAX_FFT,              ///< Maximum FFT frequency detected (for filter tuning)
    OSD_STAT_TOTAL_FLIGHTS,        ///< Cumulative number of flights (persistent across power cycles)
    OSD_STAT_TOTAL_TIME,           ///< Cumulative flight time (persistent across power cycles)
    OSD_STAT_TOTAL_DIST,           ///< Cumulative distance traveled (persistent across power cycles)
    OSD_STAT_MIN_RSSI_DBM,         ///< Minimum RSSI in dBm units
    OSD_STAT_COUNT                 ///< Count of statistics items - MUST BE LAST
} osd_stats_e;

/**
 * @enum osd_unit_e
 * @brief Unit system selection for OSD display
 * 
 * @details Defines the measurement unit system used for displaying telemetry values
 *          on the OSD. This setting affects how distances, speeds, altitudes, and
 *          temperatures are presented to the user.
 * 
 * @see osd_config_t::units for unit system configuration
 */
typedef enum : uint8_t {
    OSD_UNIT_IMPERIAL,  ///< US customary units (feet, miles per hour, Fahrenheit)
    OSD_UNIT_METRIC     ///< Metric units (meters, kilometers per hour, Celsius)
} osd_unit_e;

/**
 * @enum osd_timer_e
 * @brief Configurable on-screen timer identifiers
 * 
 * @details Defines the available on-screen timers that can be displayed on the OSD.
 *          Each timer can be independently configured to track different time values
 *          such as total flight time, armed time, or custom countdown/countup values.
 *          Timer configuration is stored in osd_config_t::timers array.
 * 
 * @note OSD_TIMER_COUNT is used to size timer configuration arrays
 * @see osd_config_t::timers for timer configuration
 */
typedef enum {
    OSD_TIMER_1,       ///< First configurable on-screen timer
    OSD_TIMER_2,       ///< Second configurable on-screen timer
    OSD_TIMER_COUNT    ///< Number of available timers (array size)
} osd_timer_e;

/**
 * @enum osd_warnings_flags_e
 * @brief Warning conditions displayed on OSD
 * 
 * @details This enumeration defines warning flags that can be displayed on the OSD
 *          to alert the pilot of critical or important vehicle states. Each warning
 *          can be independently enabled or disabled via the enabled_warnings bitfield
 *          in osd_config_t structure. When a warning condition is active and enabled,
 *          it is displayed prominently on the OSD, often with flashing or highlighted
 *          text to ensure pilot awareness.
 *          
 *          Warnings are essential for flight safety, alerting pilots to conditions
 *          such as low battery, signal loss, failsafe activation, or system failures.
 * 
 * @note OSD_WARNING_COUNT must be the last enum value
 * @warning Warning configuration affects pilot awareness of critical conditions
 * @see osd_config_t::enabled_warnings for warning configuration bitfield
 */
typedef enum {
    OSD_WARNING_ARMING_DISABLE,            ///< Vehicle cannot arm - pre-arm checks failed
    OSD_WARNING_BATTERY_NOT_FULL,          ///< Battery not fully charged before flight
    OSD_WARNING_BATTERY_WARNING,           ///< Battery voltage below warning threshold
    OSD_WARNING_BATTERY_CRITICAL,          ///< Battery voltage critically low - land immediately
    OSD_WARNING_VISUAL_BEEPER,             ///< Visual beeper alert indicator
    OSD_WARNING_CRASH_FLIP,                ///< Crash flip mode active
    OSD_WARNING_ESC_FAIL,                  ///< Electronic Speed Controller failure detected
    OSD_WARNING_CORE_TEMPERATURE,          ///< Flight controller temperature exceeds safe limits
    OSD_WARNING_RC_SMOOTHING,              ///< RC smoothing filter active
    OSD_WARNING_FAIL_SAFE,                 ///< Failsafe mode activated due to signal loss or other trigger
    OSD_WARNING_LAUNCH_CONTROL,            ///< Launch control mode active
    OSD_WARNING_GPS_RESCUE_UNAVAILABLE,    ///< GPS rescue mode cannot be activated (insufficient GPS fix)
    OSD_WARNING_GPS_RESCUE_DISABLED,       ///< GPS rescue mode is disabled in configuration
    OSD_WARNING_RSSI,                      ///< Radio signal strength (RSSI) below warning threshold
    OSD_WARNING_LINK_QUALITY,              ///< Radio link quality below warning threshold
    OSD_WARNING_RSSI_DBM,                  ///< Radio signal strength in dBm below warning threshold
    OSD_WARNING_COUNT                      ///< Count of warning types - MUST BE LAST
} osd_warnings_flags_e;

/**
 * @struct osd_config_s
 * @brief OSD configuration structure
 * 
 * @details This structure contains the complete OSD configuration including unit
 *          preferences, alarm thresholds, timer settings, and enabled features.
 *          The configuration is exchanged between the flight controller and ground
 *          control station or OSD configurator via MSP_OSD_CONFIG commands.
 *          
 *          This structure matches the Betaflight/INAV OSD configuration format
 *          for protocol compatibility, allowing ArduPilot to work with existing
 *          OSD configuration tools and hardware.
 * 
 * @note This structure must maintain binary compatibility with Betaflight/INAV
 * @warning Changing field order or sizes will break MSP protocol compatibility
 */
typedef struct osd_config_s {
    osd_unit_e units;               ///< Unit system selection (imperial or metric) for display formatting
    uint8_t rssi_alarm;             ///< RSSI alarm threshold (0-100%). Triggers warning when signal strength drops below this value
    uint16_t cap_alarm;             ///< Battery capacity alarm threshold in mAh. Triggers warning when consumed capacity exceeds this value
    uint16_t alt_alarm;             ///< Altitude alarm threshold. Units are meters or feet depending on @p units setting. Triggers warning when altitude exceeds threshold
    uint16_t timers[OSD_TIMER_COUNT]; ///< Timer configuration array for OSD_TIMER_1 and OSD_TIMER_2. Each value configures timer behavior (e.g., total time, armed time)
    uint32_t enabled_stats;         ///< Bitfield of enabled post-flight statistics. Each bit corresponds to an osd_stats_e value. If bit N is set, statistic N is displayed after disarming
    uint32_t enabled_warnings;      ///< Bitfield of enabled warning conditions. Each bit corresponds to an osd_warnings_flags_e value. If bit N is set, warning N is shown when condition is active
} osd_config_t;
}

#endif //HAL_MSP_ENABLED