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
 * @file AP_GPS_UBLOX.h
 * @brief u-blox GPS receiver UBX binary protocol driver
 * 
 * @details Comprehensive driver for u-blox GPS receivers supporting M8, M9, M10, and F9 series.
 *          Implements UBX binary protocol parser with automatic baud rate detection, configuration
 *          via legacy CFG messages (M8) and modern VALGET/VALSET commands (F9/M10). Provides full
 *          RTK support including moving baseline for dual-GPS heading, dual-antenna heading,
 *          automatic message rate configuration optimized for different vehicle types, and
 *          advanced features like jamming detection, spoofing detection, and hardware IMU integration
 *          (F9R/M10).
 * 
 * @note This is the most widely used GPS driver in the ArduPilot ecosystem due to u-blox
 *       receiver popularity and feature completeness.
 * 
 * Original implementation by Michael Smith, Jordi Munoz and Jose Julio, DIYDrones.com
 * 
 * Protocol references:
 * - u-blox 6: http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
 * - u-blox M8: Interface Description available from u-blox
 * - u-blox F9: Interface Description for F9 High Precision GNSS modules
 * - u-blox M10: Interface Description for M10 Standard Precision modules
 * 
 * Source: libraries/AP_GPS/AP_GPS_UBLOX.h
 * Source: libraries/AP_GPS/AP_GPS_UBLOX.cpp (implementation)
 */
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_UBLOX_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"

#include <AP_HAL/AP_HAL.h>

/*
 *  try to put a UBlox into binary mode. This is in two parts. 
 *
 * First we send a ubx binary message that enables the NAV_SOL message
 * at rate 1. Then we send a NMEA message to set the baud rate to our
 * desired rate. The reason for doing the NMEA message second is if we
 * send it first the second message will be ignored for a baud rate
 * change.
 * The reason we need the NAV_SOL rate message at all is some uBlox
 * modules are configured with all ubx binary messages off, which
 * would mean we would never detect it.

 * running a uBlox at less than 38400 will lead to packet
 * corruption, as we can't receive the packets in the 200ms
 * window for 5Hz fixes. The NMEA startup message should force
 * the uBlox into 230400 no matter what rate it is configured
 * for.
 */
#define UBLOX_SET_BINARY_115200 "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,115200,0*1C\r\n"

// a variant with 230400 baudrate
#define UBLOX_SET_BINARY_230400 "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,230400,0*1E\r\n"

// a variant with 460800 baudrate
#define UBLOX_SET_BINARY_460800 "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0023,0001,460800,0*11\r\n"

#define UBLOX_RXM_RAW_LOGGING 1
#define UBLOX_MAX_RXM_RAW_SATS 22
#define UBLOX_MAX_RXM_RAWX_SATS 32
#define UBLOX_MAX_EXTENSIONS 8
#define UBLOX_GNSS_SETTINGS 1
#ifndef UBLOX_TIM_TM2_LOGGING
    #define UBLOX_TIM_TM2_LOGGING (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif

#define UBLOX_MAX_GNSS_CONFIG_BLOCKS 7

#define UBX_TIMEGPS_VALID_WEEK_MASK 0x2

#define UBLOX_MAX_PORTS 6
#define UBLOX_MODULE_LEN 9

#define RATE_POSLLH 1
#define RATE_STATUS 1
#define RATE_SOL 1
#define RATE_TIMEGPS 5
#define RATE_PVT 1
#define RATE_VELNED 1
#define RATE_DOP 1
#define RATE_HW 5
#define RATE_HW2 5
#define RATE_TIM_TM2 1

#define CONFIG_RATE_NAV      (1<<0)
#define CONFIG_RATE_POSLLH   (1<<1)
#define CONFIG_RATE_STATUS   (1<<2)
#define CONFIG_RATE_SOL      (1<<3)
#define CONFIG_RATE_VELNED   (1<<4)
#define CONFIG_RATE_DOP      (1<<5)
#define CONFIG_RATE_MON_HW   (1<<6)
#define CONFIG_RATE_MON_HW2  (1<<7)
#define CONFIG_RATE_RAW      (1<<8)
#define CONFIG_VERSION       (1<<9)
#define CONFIG_NAV_SETTINGS  (1<<10)
#define CONFIG_GNSS          (1<<11)
#define CONFIG_SBAS          (1<<12)
#define CONFIG_RATE_PVT      (1<<13)
#define CONFIG_TP5           (1<<14)
#define CONFIG_RATE_TIMEGPS  (1<<15)
#define CONFIG_TMODE_MODE    (1<<16)
#define CONFIG_RTK_MOVBASE   (1<<17)
#define CONFIG_TIM_TM2       (1<<18)
#define CONFIG_F9            (1<<19)
#define CONFIG_M10           (1<<20)
#define CONFIG_L5            (1<<21)
#define CONFIG_LAST          (1<<22) // this must always be the last bit

#define CONFIG_REQUIRED_INITIAL (CONFIG_RATE_NAV | CONFIG_RATE_POSLLH | CONFIG_RATE_STATUS | CONFIG_RATE_VELNED)

#define CONFIG_ALL (CONFIG_RATE_NAV | CONFIG_RATE_POSLLH | CONFIG_RATE_STATUS | CONFIG_RATE_SOL | CONFIG_RATE_VELNED \
                    | CONFIG_RATE_DOP | CONFIG_RATE_MON_HW | CONFIG_RATE_MON_HW2 | CONFIG_RATE_RAW | CONFIG_VERSION \
                    | CONFIG_NAV_SETTINGS | CONFIG_GNSS | CONFIG_SBAS)

//Configuration Sub-Sections
#define SAVE_CFG_IO     (1<<0)
#define SAVE_CFG_MSG    (1<<1)
#define SAVE_CFG_INF    (1<<2)
#define SAVE_CFG_NAV    (1<<3)
#define SAVE_CFG_RXM    (1<<4)
#define SAVE_CFG_RINV   (1<<9)
#define SAVE_CFG_ANT    (1<<10)
#define SAVE_CFG_ALL    (SAVE_CFG_IO|SAVE_CFG_MSG|SAVE_CFG_INF|SAVE_CFG_NAV|SAVE_CFG_RXM|SAVE_CFG_RINV|SAVE_CFG_ANT)

class RTCM3_Parser;

/**
 * @class AP_GPS_UBLOX
 * @brief GPS backend driver for u-blox receivers using UBX binary protocol
 * 
 * @details Full-featured u-blox driver supporting multiple hardware generations and advanced features:
 * 
 * **Supported Hardware Generations:**
 * - M8N/M8P/M8T: Standard accuracy GNSS, RTK capable (M8P), timing support (M8T)
 * - F9P/F9R: Multi-band RTK with centimeter accuracy, moving baseline, hardware IMU (F9R)
 * - M9N: Standard accuracy with improved jamming resistance
 * - M10: Latest generation with multi-band GNSS and improved signal tracking
 * - ZED-F9P: Popular RTK module achieving 1cm accuracy with corrections
 * 
 * **Protocol Support:**
 * - UBX binary protocol with automatic parsing and checksum verification
 * - Legacy CFG-* message configuration for M8 generation
 * - Modern VALGET/VALSET configuration interface for F9/M9/M10 generations
 * - Automatic hardware detection and protocol adaptation
 * 
 * **Key Features:**
 * - **Auto-baud Detection**: Automatically detects receiver baud rate (115200/230400/460800)
 * - **Message Rate Optimization**: Configures message rates based on vehicle type and CPU load
 * - **RTK Base Station**: Can output RTCM3 corrections from raw measurements
 * - **RTK Rover**: Injects RTCM3 corrections for centimeter-level positioning
 * - **Moving Baseline RTK**: Dual F9P configuration for precise heading (base+rover)
 * - **Dual-Antenna Heading**: F9/M10 with second antenna for heading without movement
 * - **Time Pulse (PPS)**: Configurable pulse-per-second output for time synchronization
 * - **Jamming Detection**: Hardware jamming indicator from MON-HW messages
 * - **Spoofing Detection**: Available on F9/M10 hardware
 * - **Multi-GNSS**: GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC, SBAS
 * 
 * **State Machine Operation:**
 * The driver implements a configuration state machine that sequentially:
 * 1. Detects hardware version via MON-VER
 * 2. Configures GNSS constellations based on GPS_GNSS_MODE parameter
 * 3. Sets navigation model (airborne, automotive, etc.)
 * 4. Configures message output rates
 * 5. Enables RTK or moving baseline if configured
 * 6. Saves configuration to non-volatile memory
 * 
 * **Message Parsing:**
 * Primary navigation messages parsed:
 * - NAV-PVT: Position, velocity, time (F9/M10 primary message)
 * - NAV-POSLLH: Position lat/lon/height (M8)
 * - NAV-STATUS: Fix status and flags (M8)
 * - NAV-VELNED: Velocity in NED frame (M8)
 * - NAV-SOL: Navigation solution (M8 legacy)
 * - NAV-DOP: Dilution of precision values
 * - NAV-RELPOSNED: Relative position for moving baseline
 * - NAV-TIMEGPS: GPS time for synchronization
 * 
 * **RTK Configuration Modes:**
 * - **Rover**: Receives RTCM3 via inject_data(), achieves cm-level accuracy
 * - **Base**: Generates RTCM3 via get_RTCMV3() from RXM-RAWX measurements
 * - **Moving Baseline**: Two F9P modules (base+rover) provide accurate heading
 * 
 * **Coordinate Systems:**
 * - Position: WGS84 latitude/longitude in degrees, height above ellipsoid and MSL in meters
 * - Velocity: NED (North-East-Down) frame in m/s (internally cm/s in protocol)
 * - Relative Position: NED frame in millimeters for moving baseline
 * 
 * **Unit Conventions:**
 * - Position: 1e-7 degrees (lat/lon) in protocol, millimeters (height)
 * - Velocity: cm/s in UBX protocol, converted to m/s for ArduPilot
 * - Accuracy: millimeters in protocol, converted to meters for ArduPilot
 * - Time: milliseconds (GPS time of week - iTOW)
 * 
 * @warning GPS initialization timing affects EKF convergence. System requires valid GPS fix
 *          before arming to ensure proper navigation solution initialization.
 * 
 * @warning Moving baseline requires two identical F9P modules with matching firmware versions
 *          and proper separation distance (typically 30cm-2m between antennas).
 * 
 * @note This driver is called at the main GPS thread rate (typically 10Hz polling) and must
 *       process incoming serial data efficiently to avoid buffer overruns.
 * 
 * Source: libraries/AP_GPS/AP_GPS_UBLOX.h:127-911
 * Source: libraries/AP_GPS/AP_GPS_UBLOX.cpp (full implementation)
 */
class AP_GPS_UBLOX : public AP_GPS_Backend
{
public:
    /**
     * @brief Constructor for u-blox GPS driver instance
     * 
     * @param[in] _gps Reference to main AP_GPS object
     * @param[in] _params Reference to GPS parameters for this instance
     * @param[in] _state Reference to GPS state structure for this instance
     * @param[in] _port Pointer to HAL UART driver for GPS communication
     * @param[in] role GPS role (primary, secondary, RTK base, RTK rover, moving baseline base/rover)
     */
    AP_GPS_UBLOX(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port, AP_GPS::GPS_Role role);
    
    /**
     * @brief Destructor - cleans up RTCM3 parser if allocated
     */
    ~AP_GPS_UBLOX() override;

    // Methods
    
    /**
     * @brief Parse UBX binary messages from serial stream and update GPS state
     * 
     * @details Implements state machine parser for UBX protocol:
     *          1. Sync characters: 0xB5 0x62 (PREAMBLE1, PREAMBLE2)
     *          2. Header: message class, ID, and payload length (6 bytes total)
     *          3. Payload: variable length data
     *          4. Checksum: Fletcher-16 algorithm (2 bytes: ck_a, ck_b)
     * 
     *          Called repeatedly at GPS thread rate (typically 10Hz) to process incoming
     *          serial data. Reads available bytes from UART, feeds to state machine,
     *          and processes complete messages via _parse_gps().
     * 
     * @return true if a complete valid message was received and parsed, false otherwise
     * 
     * @note This method must be efficient as it's called frequently. Uses incremental
     *       parsing to avoid blocking on incomplete messages.
     * 
     * @see _parse_gps() for message processing after successful reception
     * 
     * Source: libraries/AP_GPS/AP_GPS_UBLOX.cpp
     */
    bool read() override;

    /**
     * @brief Returns the highest GPS fix status this driver can provide
     * 
     * @return GPS_OK_FIX_3D_RTK_FIXED indicating full RTK fixed solution support with
     *         heading capability (for moving baseline configurations)
     * 
     * @note u-blox F9P/M8P support RTK fixed solutions with centimeter accuracy
     */
    AP_GPS::GPS_Status highest_supported_status(void) override { return AP_GPS::GPS_OK_FIX_3D_RTK_FIXED; }

    /**
     * @brief Static detection method to identify u-blox GPS from serial stream
     * 
     * @param[in,out] state Detection state machine structure
     * @param[in] data Single byte from serial stream
     * 
     * @return true if u-blox UBX protocol detected (found valid UBX message preamble and structure)
     * 
     * @details Called during GPS auto-detection phase. Looks for UBX message preamble
     *          (0xB5 0x62) followed by valid message structure. Maintains detection state
     *          across multiple calls to handle byte-by-byte serial input.
     * 
     * Source: libraries/AP_GPS/AP_GPS_UBLOX.cpp
     */
    static bool _detect(struct UBLOX_detect_state &state, uint8_t data);

    /**
     * @brief Check if GPS has completed initial configuration sequence
     * 
     * @return true if all required configuration messages have been sent and acknowledged
     * 
     * @details Configuration is considered complete when _unconfigured_messages bitmask is zero,
     *          indicating all configuration steps (GNSS settings, message rates, nav settings,
     *          etc.) have been successfully applied. In SITL always returns true.
     * 
     * @note Auto-configuration can be disabled via GPS_AUTO_CONFIG parameter
     */
    bool is_configured(void) const override {
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        if (!gps._auto_config) {
            return true;
        } else {
            return !_unconfigured_messages;
        }
#else
        return true;
#endif // CONFIG_HAL_BOARD != HAL_BOARD_SITL
    }

    /**
     * @brief Get bitmask of unconfigured message types for debugging
     * 
     * @param[out] error_codes Bitmask of CONFIG_* flags for messages not yet configured
     * 
     * @return true always (error codes are always available)
     * 
     * @details Returns _unconfigured_messages bitmask where each bit represents a
     *          configuration step. Non-zero indicates incomplete configuration.
     *          Useful for diagnosing GPS initialization issues.
     */
    bool get_error_codes(uint32_t &error_codes) const override {
        error_codes = _unconfigured_messages;
        return true;
    };

    /**
     * @brief Send GCS message explaining why configuration failed
     * 
     * @details Broadcasts detailed failure reason via MAVLink STATUSTEXT when configuration
     *          cannot complete. Helps diagnose hardware or parameter issues.
     */
    void broadcast_configuration_failure_reason(void) const override;
    
#if HAL_LOGGING_ENABLED
    /**
     * @brief Write GPS configuration info to dataflash log at startup
     * 
     * @details Logs hardware version, firmware version, and configuration status for
     *          post-flight analysis and debugging.
     */
    void Write_AP_Logger_Log_Startup_messages() const override;
#endif

    /**
     * @brief Get velocity measurement lag compensation value
     * 
     * @param[out] lag_sec Velocity lag in seconds
     * 
     * @return true if driver is confident in lag value, false otherwise
     * 
     * @details u-blox receivers have known velocity measurement delays depending on
     *          configuration and message rates. This provides lag compensation for
     *          EKF velocity fusion. Typical lag is 0.22 seconds for 5Hz update rate.
     * 
     * @note Accurate lag compensation is important for high-speed vehicles to avoid
     *       navigation errors during maneuvers.
     */
    bool get_lag(float &lag_sec) const override;

    /**
     * @brief Get driver name string
     * 
     * @return "u-blox" constant string identifier
     */
    const char *name() const override { return "u-blox"; }

    /**
     * @brief Retrieve RTCMv3 correction data when operating as moving baseline base station
     * 
     * @param[out] bytes Pointer to RTCM3 data buffer (owned by driver, do not free)
     * @param[out] len Length of RTCM3 data in bytes
     * 
     * @return true if RTCM3 data is available, false otherwise
     * 
     * @details When configured as moving baseline base (GPS_TYPE=22), this driver generates
     *          RTCM3 messages from RXM-RAWX/RXM-SFRBX measurements for transmission to the
     *          rover GPS. The rover uses these corrections to calculate precise relative position.
     * 
     * @note Only functional in moving baseline base mode. RTCM data must be transmitted
     *       to rover GPS within 1 second for optimal performance.
     * 
     * @see clear_RTCMV3() to mark data as consumed
     */
    bool get_RTCMV3(const uint8_t *&bytes, uint16_t &len) override;
    
    /**
     * @brief Clear/acknowledge RTCM3 data has been consumed
     * 
     * @details Called after RTCM3 data from get_RTCMV3() has been transmitted to rover.
     *          Allows driver to free buffer and prepare for next RTCM3 message.
     * 
     * @see get_RTCMV3()
     */
    void clear_RTCMV3(void) override;

    /**
     * @brief u-blox specific health checks beyond standard GPS validation
     * 
     * @return true if GPS is healthy, false if issues detected
     * 
     * @details Performs u-blox specific checks including:
     *          - Hardware jamming indicator from MON-HW
     *          - Antenna status (open, short circuit)
     *          - Spoofing detection flags (F9/M10)
     *          - Configuration completeness
     * 
     * @note Supplements base class health checking with u-blox hardware monitoring features
     */
    bool is_healthy(void) const override;
    
private:
    // u-blox UBX protocol essentials
    
    /**
     * @struct ubx_header
     * @brief UBX protocol message header structure (6 bytes)
     * 
     * @details Every UBX message starts with this fixed header format:
     *          - Preamble: 0xB5 0x62 (sync characters for message detection)
     *          - msg_class: Message class (e.g., 0x01=NAV, 0x06=CFG, 0x0A=MON)
     *          - msg_id: Message ID within class
     *          - length: Payload length in bytes (little-endian uint16)
     * 
     * Header is followed by payload (variable length) and 2-byte checksum.
     */
    struct PACKED ubx_header {
        uint8_t preamble1;
        uint8_t preamble2;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t length;
    };
#if UBLOX_GNSS_SETTINGS
    struct PACKED ubx_cfg_gnss {
        uint8_t msgVer;
        uint8_t numTrkChHw;
        uint8_t numTrkChUse;
        uint8_t numConfigBlocks;
        PACKED struct configBlock {
            uint8_t gnssId;
            uint8_t resTrkCh;
            uint8_t maxTrkCh;
            uint8_t reserved1;
            uint32_t flags;
        } configBlock[UBLOX_MAX_GNSS_CONFIG_BLOCKS];
    };
#endif
    struct PACKED ubx_cfg_nav_rate {
        uint16_t measure_rate_ms;
        uint16_t nav_rate;
        uint16_t timeref;
    };
    struct PACKED ubx_cfg_msg {
        uint8_t msg_class;
        uint8_t msg_id;
    };
    struct PACKED ubx_cfg_msg_rate {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rate;
    };
    struct PACKED ubx_cfg_msg_rate_6 {
        uint8_t msg_class;
        uint8_t msg_id;
        uint8_t rates[6];
    };
    struct PACKED ubx_cfg_nav_settings {
        uint16_t mask;
        uint8_t dynModel;
        uint8_t fixMode;
        int32_t fixedAlt;
        uint32_t fixedAltVar;
        int8_t minElev;
        uint8_t drLimit;
        uint16_t pDop;
        uint16_t tDop;
        uint16_t pAcc;
        uint16_t tAcc;
        uint8_t staticHoldThresh;
        uint8_t res1;
        uint32_t res2;
        uint32_t res3;
        uint32_t res4;
    };
    struct PACKED ubx_cfg_tp5 {
        uint8_t tpIdx;
        uint8_t version;
        uint8_t reserved1[2];
        int16_t antCableDelay;
        int16_t rfGroupDelay;
        uint32_t freqPeriod;
        uint32_t freqPeriodLock;
        uint32_t pulseLenRatio;
        uint32_t pulseLenRatioLock;
        int32_t userConfigDelay;
        uint32_t flags;
    };
    struct PACKED ubx_cfg_prt {
        uint8_t portID;
    };
    struct PACKED ubx_cfg_sbas {
        uint8_t mode;
        uint8_t usage;
        uint8_t maxSBAS;
        uint8_t scanmode2;
        uint32_t scanmode1;
    };
    /**
     * @enum ConfigKey
     * @brief Configuration keys for F9/M9/M10 VALGET/VALSET interface
     * 
     * @details Modern u-blox receivers (F9 and later) use a key-value configuration system
     *          accessed via CFG-VALGET and CFG-VALSET messages instead of legacy CFG-* messages.
     *          Each key is a 32-bit identifier with embedded type information.
     * 
     * Key format (bits):
     * - [31:28]: Reserved
     * - [27:16]: Group ID
     * - [15:8]: Item ID within group  
     * - [7:0]: Reserved/Size
     * 
     * Configuration layers:
     * - RAM: Volatile, active immediately, lost on reboot
     * - BBR: Battery-backed RAM, survives hot start
     * - Flash: Non-volatile storage, survives power cycle
     * 
     * @note M8 and earlier use different CFG-* message format for configuration
     */
    enum class ConfigKey : uint32_t {
        TMODE_MODE = 0x20030001,
        CFG_RATE_MEAS                   = 0x30210001,

        CFG_UART1_BAUDRATE              = 0x40520001,
        CFG_UART1_ENABLED               = 0x10520005,
        CFG_UART1INPROT_UBX             = 0x10730001,
        CFG_UART1INPROT_NMEA            = 0x10730002,
        CFG_UART1INPROT_RTCM3X          = 0x10730004,
        CFG_UART1OUTPROT_UBX            = 0x10740001,
        CFG_UART1OUTPROT_NMEA           = 0x10740002,
        CFG_UART1OUTPROT_RTCM3X         = 0x10740004,

        CFG_UART2_BAUDRATE              = 0x40530001,
        CFG_UART2_ENABLED               = 0x10530005,
        CFG_UART2INPROT_UBX             = 0x10750001,
        CFG_UART2INPROT_NMEA            = 0x10750002,
        CFG_UART2INPROT_RTCM3X          = 0x10750004,
        CFG_UART2OUTPROT_UBX            = 0x10760001,
        CFG_UART2OUTPROT_NMEA           = 0x10760002,
        CFG_UART2OUTPROT_RTCM3X         = 0x10760004,

        MSGOUT_RTCM_3X_TYPE4072_0_UART1 = 0x209102ff,
        MSGOUT_RTCM_3X_TYPE4072_1_UART1 = 0x20910382,
        MSGOUT_RTCM_3X_TYPE1077_UART1   = 0x209102cd,
        MSGOUT_RTCM_3X_TYPE1087_UART1   = 0x209102d2,
        MSGOUT_RTCM_3X_TYPE1097_UART1   = 0x20910319,
        MSGOUT_RTCM_3X_TYPE1127_UART1   = 0x209102d7,
        MSGOUT_RTCM_3X_TYPE1230_UART1   = 0x20910304,
        MSGOUT_UBX_NAV_RELPOSNED_UART1  = 0x2091008e,

        MSGOUT_RTCM_3X_TYPE4072_0_UART2 = 0x20910300,
        MSGOUT_RTCM_3X_TYPE4072_1_UART2 = 0x20910383,
        MSGOUT_RTCM_3X_TYPE1077_UART2   = 0x209102ce,
        MSGOUT_RTCM_3X_TYPE1087_UART2   = 0x209102d3,
        MSGOUT_RTCM_3X_TYPE1097_UART2   = 0x2091031a,
        MSGOUT_RTCM_3X_TYPE1127_UART2   = 0x209102d8,
        MSGOUT_RTCM_3X_TYPE1230_UART2   = 0x20910305,
        MSGOUT_UBX_NAV_RELPOSNED_UART2  = 0x2091008f,

        // enable specific signals and constellations
        CFG_SIGNAL_GPS_ENA              = 0x1031001f,
        CFG_SIGNAL_GPS_L1CA_ENA         = 0x10310001,
        CFG_SIGNAL_GPS_L2C_ENA          = 0x10310003,
        CFG_SIGNAL_GPS_L5_ENA           = 0x10310004,
        CFG_SIGNAL_SBAS_ENA             = 0x10310020,
        CFG_SIGNAL_SBAS_L1CA_ENA        = 0x10310005,
        CFG_SIGNAL_GAL_ENA              = 0x10310021,
        CFG_SIGNAL_GAL_E1_ENA           = 0x10310007,
        CFG_SIGNAL_GAL_E5A_ENA          = 0x10310009,
        CFG_SIGNAL_GAL_E5B_ENA          = 0x1031000a,
        CFG_SIGNAL_BDS_ENA              = 0x10310022,
        CFG_SIGNAL_BDS_B1_ENA           = 0x1031000d,
        CFG_SIGNAL_BDS_B1C_ENA          = 0x1031000f,
        CFG_SIGNAL_BDS_B2_ENA           = 0x1031000e,
        CFG_SIGNAL_BDS_B2A_ENA          = 0x10310028,
        CFG_SIGNAL_QZSS_ENA             = 0x10310024,
        CFG_SIGNAL_QZSS_L1CA_ENA        = 0x10310012,
        CFG_SIGNAL_QZSS_L1S_ENA         = 0x10310014,
        CFG_SIGNAL_QZSS_L2C_ENA         = 0x10310015,
        CFG_SIGNAL_QZSS_L5_ENA          = 0x10310017,
        CFG_SIGNAL_GLO_ENA              = 0x10310025,
        CFG_SIGNAL_GLO_L1_ENA           = 0x10310018,
        CFG_SIGNAL_GLO_L2_ENA           = 0x1031001a,
        CFG_SIGNAL_NAVIC_ENA            = 0x10310026,
        CFG_SIGNAL_NAVIC_L5_ENA         = 0x1031001d,

        CFG_SIGNAL_L5_HEALTH_OVRD       = 0x10320001,

        // other keys
        CFG_NAVSPG_DYNMODEL             = 0x20110021,

    };

    // layers for VALSET
    #define UBX_VALSET_LAYER_RAM 0x1U
    #define UBX_VALSET_LAYER_BBR 0x2U
    #define UBX_VALSET_LAYER_FLASH 0x4U
    #define UBX_VALSET_LAYER_ALL 0x7U

    struct PACKED ubx_cfg_valset {
        uint8_t version;
        uint8_t layers;
        uint8_t transaction;
        uint8_t reserved[1];
        uint32_t key;
    };
    struct PACKED ubx_cfg_valget {
        uint8_t version;
        uint8_t layers;
        uint8_t reserved[2];
        // variable length data, check buffer length
    };
    /**
     * @struct ubx_nav_posllh
     * @brief NAV-POSLLH: Position solution in geodetic coordinates (M8 primary message)
     * 
     * @details Provides position in WGS84 coordinates. Used as primary position message
     *          on M8 generation receivers. F9/M10 use NAV-PVT instead which combines
     *          position, velocity, and time in one message.
     * 
     * Units:
     * - itow: GPS time of week in milliseconds
     * - longitude/latitude: degrees * 1e-7 (int32)
     * - altitude: millimeters above ellipsoid/MSL (int32)
     * - accuracy: millimeters (uint32)
     */
    struct PACKED ubx_nav_posllh {
        uint32_t itow;                                  // GPS msToW
        int32_t longitude;
        int32_t latitude;
        int32_t altitude_ellipsoid;
        int32_t altitude_msl;
        uint32_t horizontal_accuracy;
        uint32_t vertical_accuracy;
    };
    
    /**
     * @struct ubx_nav_status
     * @brief NAV-STATUS: Receiver navigation status (M8)
     * 
     * @details Provides fix type and status flags. Used on M8 receivers; F9/M10
     *          include this information in NAV-PVT.
     */
    struct PACKED ubx_nav_status {
        uint32_t itow;                                  // GPS msToW
        uint8_t fix_type;
        uint8_t fix_status;
        uint8_t differential_status;
        uint8_t res;
        uint32_t time_to_first_fix;
        uint32_t uptime;                                // milliseconds
    };
    struct PACKED ubx_nav_dop {
        uint32_t itow;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    };
    struct PACKED ubx_nav_solution {
        uint32_t itow;
        int32_t time_nsec;
        uint16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    };
    /**
     * @struct ubx_nav_pvt
     * @brief NAV-PVT: Navigation Position Velocity Time solution (F9/M9/M10 primary message)
     * 
     * @details Combined position, velocity, and time message. This is the primary navigation
     *          message for F9 and later receivers, replacing the separate POSLLH/VELNED/SOL
     *          messages used by M8. More efficient as it provides complete solution in single
     *          message, reducing serial bandwidth and processing overhead.
     * 
     * Contains:
     * - Complete UTC time (year/month/day/hour/min/sec)
     * - Position in WGS84 (lat/lon/height)
     * - Velocity in NED frame
     * - Ground speed and heading
     * - Accuracy estimates for position, velocity, heading
     * - Fix type and validity flags
     * - Number of satellites used
     * 
     * Units:
     * - itow: milliseconds (GPS time of week)
     * - lon/lat: degrees * 1e-7 (int32)
     * - h_ellipsoid/h_msl: millimeters (int32)
     * - velN/velE/velD/gspeed: mm/s (int32)
     * - head_mot: degrees * 1e-5 (int32)
     * - Accuracies: millimeters or mm/s (uint32)
     * 
     * @note Preferred message for F9/M10 due to atomicity - all data from same solution epoch
     */
    struct PACKED ubx_nav_pvt {
        uint32_t itow; 
        uint16_t year; 
        uint8_t month, day, hour, min, sec; 
        uint8_t valid; 
        uint32_t t_acc; 
        int32_t nano; 
        uint8_t fix_type; 
        uint8_t flags; 
        uint8_t flags2; 
        uint8_t num_sv; 
        int32_t lon, lat; 
        int32_t h_ellipsoid, h_msl;
        uint32_t h_acc, v_acc; 
        int32_t velN, velE, velD, gspeed; 
        int32_t head_mot; 
        uint32_t s_acc; 
        uint32_t head_acc; 
        uint16_t p_dop; 
        uint8_t flags3;
        uint8_t reserved1[5];
        int32_t headVeh;
        int16_t magDec;
        uint16_t magAcc;
    };
    
    /**
     * @struct ubx_nav_relposned
     * @brief NAV-RELPOSNED: Relative positioning information in NED frame
     * 
     * @details Provides relative position between two GPS receivers for moving baseline RTK.
     *          Used when two F9P modules are configured as base+rover to determine precise
     *          heading from baseline vector. Requires GPS_TYPE2=23 (moving baseline rover)
     *          and connected base station.
     * 
     * Contains:
     * - Relative position in NED frame (North, East, Down)
     * - Baseline length and heading
     * - High-precision components (HP fields add sub-millimeter precision)
     * - Accuracy estimates
     * - Status flags (fix type, carrier solution)
     * 
     * Units:
     * - relPosN/E/D: millimeters (int32)
     * - relPosHPN/E/D: 0.1mm (int8) - adds sub-millimeter precision
     * - relPosLength: millimeters (int32)
     * - relPosHeading: degrees * 1e-5 (int32)
     * - Accuracies: 0.1mm (uint32)
     * 
     * @note Requires RTK fixed solution for accurate heading. Typical accuracy is
     *       1-2cm position, 0.2° heading for 1m baseline at RTK fixed.
     * 
     * @warning Baseline length must be accurately known (configured via GPS_POS parameters)
     *          for correct heading calculation.
     */
    struct PACKED ubx_nav_relposned {
        uint8_t version;
        uint8_t reserved1;
        uint16_t refStationId;
        uint32_t iTOW;
        int32_t relPosN;
        int32_t relPosE;
        int32_t relPosD;
        int32_t relPosLength;
        int32_t relPosHeading;
        uint8_t reserved2[4];
        int8_t relPosHPN;
        int8_t relPosHPE;
        int8_t relPosHPD;
        int8_t relPosHPLength;
        uint32_t accN;
        uint32_t accE;
        uint32_t accD;
        uint32_t accLength;
        uint32_t accHeading;
        uint8_t reserved3[4];
        uint32_t flags;
    };

    struct PACKED ubx_nav_velned {
        uint32_t itow;                                  // GPS msToW
        int32_t ned_north;
        int32_t ned_east;
        int32_t ned_down;
        uint32_t speed_3d;
        uint32_t speed_2d;
        int32_t heading_2d;
        uint32_t speed_accuracy;
        uint32_t heading_accuracy;
    };

    struct PACKED ubx_nav_timegps {
        uint32_t itow;
        int32_t ftow;
        uint16_t week;
        int8_t leapS;
        uint8_t valid; //  leapsvalid | weekvalid | tow valid;
        uint32_t tAcc;
    };

    // Lea6 uses a 60 byte message
    struct PACKED ubx_mon_hw_60 {
        uint32_t pinSel;
        uint32_t pinBank;
        uint32_t pinDir;
        uint32_t pinVal;
        uint16_t noisePerMS;
        uint16_t agcCnt;
        uint8_t aStatus;
        uint8_t aPower;
        uint8_t flags;
        uint8_t reserved1;
        uint32_t usedMask;
        uint8_t VP[17];
        uint8_t jamInd;
        uint16_t reserved3;
        uint32_t pinIrq;
        uint32_t pullH;
        uint32_t pullL;
    };
    // Neo7 uses a 68 byte message
    struct PACKED ubx_mon_hw_68 {
        uint32_t pinSel;
        uint32_t pinBank;
        uint32_t pinDir;
        uint32_t pinVal;
        uint16_t noisePerMS;
        uint16_t agcCnt;
        uint8_t aStatus;
        uint8_t aPower;
        uint8_t flags;
        uint8_t reserved1;
        uint32_t usedMask;
        uint8_t VP[25];
        uint8_t jamInd;
        uint16_t reserved3;
        uint32_t pinIrq;
        uint32_t pullH;
        uint32_t pullL;
    };
    struct PACKED ubx_mon_hw2 {
        int8_t ofsI;
        uint8_t magI;
        int8_t ofsQ;
        uint8_t magQ;
        uint8_t cfgSource;
        uint8_t reserved0[3];
        uint32_t lowLevCfg;
        uint32_t reserved1[2];
        uint32_t postStatus;
        uint32_t reserved2;
    };
    struct PACKED ubx_mon_ver {
        char swVersion[30];
        char hwVersion[10];
        char extension[30*UBLOX_MAX_EXTENSIONS]; // extensions are not enabled
    };
    struct PACKED ubx_nav_svinfo_header {
        uint32_t itow;
        uint8_t numCh;
        uint8_t globalFlags;
        uint16_t reserved;
    };
#if UBLOX_RXM_RAW_LOGGING
    struct PACKED ubx_rxm_raw {
        int32_t iTOW;
        int16_t week;
        uint8_t numSV;
        uint8_t reserved1;
        struct ubx_rxm_raw_sv {
            double cpMes;
            double prMes;
            float doMes;
            uint8_t sv;
            int8_t mesQI;
            int8_t cno;
            uint8_t lli;
        } svinfo[UBLOX_MAX_RXM_RAW_SATS];
    };
    struct PACKED ubx_rxm_rawx {
        double rcvTow;
        uint16_t week;
        int8_t leapS;
        uint8_t numMeas;
        uint8_t recStat;
        uint8_t reserved1[3];
        PACKED struct ubx_rxm_rawx_sv {
            double prMes;
            double cpMes;
            float doMes;
            uint8_t gnssId;
            uint8_t svId;
            uint8_t reserved2;
            uint8_t freqId;
            uint16_t locktime;
            uint8_t cno;
            uint8_t prStdev;
            uint8_t cpStdev;
            uint8_t doStdev;
            uint8_t trkStat;
            uint8_t reserved3;
        } svinfo[UBLOX_MAX_RXM_RAWX_SATS];
    };
#endif

    struct PACKED ubx_ack_ack {
        uint8_t clsID;
        uint8_t msgID;
    };
    struct PACKED ubx_ack_nack {
        uint8_t clsID;
        uint8_t msgID;
    };


    struct PACKED ubx_cfg_cfg {
        uint32_t clearMask;
        uint32_t saveMask;
        uint32_t loadMask;
    };

    struct PACKED ubx_tim_tm2 {
        uint8_t ch;
        uint8_t flags;
        uint16_t count;
        uint16_t wnR;
        uint16_t wnF;
        uint32_t towMsR;
        uint32_t towSubMsR;
        uint32_t towMsF;
        uint32_t towSubMsF;
        uint32_t accEst;
    };

    // Receive buffer
    union PACKED {
        DEFINE_BYTE_ARRAY_METHODS
        ubx_nav_posllh posllh;
        ubx_nav_status status;
        ubx_nav_dop dop;
        ubx_nav_solution solution;
        ubx_nav_pvt pvt;
        ubx_nav_timegps timegps;
        ubx_nav_velned velned;
        ubx_cfg_msg_rate msg_rate;
        ubx_cfg_msg_rate_6 msg_rate_6;
        ubx_cfg_nav_settings nav_settings;
        ubx_cfg_nav_rate nav_rate;
        ubx_cfg_prt prt;
        ubx_mon_hw_60 mon_hw_60;
        ubx_mon_hw_68 mon_hw_68;
        ubx_mon_hw2 mon_hw2;
        ubx_mon_ver mon_ver;
        ubx_cfg_tp5 nav_tp5;
#if UBLOX_GNSS_SETTINGS
        ubx_cfg_gnss gnss;
#endif
        ubx_cfg_sbas sbas;
        ubx_cfg_valget valget;
        ubx_nav_svinfo_header svinfo_header;
        ubx_nav_relposned relposned;
#if UBLOX_RXM_RAW_LOGGING
        ubx_rxm_raw rxm_raw;
        ubx_rxm_rawx rxm_rawx;
#endif
        ubx_ack_ack ack;
        ubx_ack_nack nack;
        ubx_tim_tm2 tim_tm2;
    } _buffer;

    enum class RELPOSNED {
        gnssFixOK          = 1U << 0,
        diffSoln           = 1U << 1,
        relPosValid        = 1U << 2,
        carrSolnFloat      = 1U << 3,

        carrSolnFixed      = 1U << 4,
        isMoving           = 1U << 5,
        refPosMiss         = 1U << 6,
        refObsMiss         = 1U << 7,

        relPosHeadingValid = 1U << 8,
        relPosNormalized   = 1U << 9
    };

    /**
     * @enum ubs_protocol_bytes
     * @brief UBX protocol message class and ID constants
     * 
     * @details Defines byte values for UBX message identification. Every UBX message
     *          has a class (category) and ID (specific message within class).
     * 
     * Message Classes:
     * - NAV (0x01): Navigation results (position, velocity, time, DOP, satellites)
     * - RXM (0x02): Receiver manager (raw measurements for RTK base)
     * - ACK (0x05): Acknowledgment of configuration messages
     * - CFG (0x06): Configuration messages (rates, settings, GNSS, ports)
     * - MON (0x0A): Monitoring messages (hardware status, version info)
     * - TIM (0x0D): Timing messages (time pulse, external events)
     * 
     * Key Navigation Messages (NAV):
     * - POSLLH (0x02): Position Lat/Lon/Height [M8]
     * - STATUS (0x03): Fix status [M8]
     * - DOP (0x04): Dilution of Precision
     * - SOL (0x06): Navigation solution [M8 legacy]
     * - PVT (0x07): Position/Velocity/Time [F9/M10 primary]
     * - VELNED (0x12): Velocity NED [M8]
     * - TIMEGPS (0x20): GPS time
     * - RELPOSNED (0x3C): Relative position [moving baseline]
     * 
     * Configuration Messages (CFG):
     * - MSG (0x01): Set message rate
     * - RATE (0x08): Set navigation rate
     * - CFG (0x09): Save configuration
     * - NAV_SETTINGS (0x24): Navigation parameters
     * - GNSS (0x3E): GNSS constellation config [M8]
     * - VALGET (0x8B): Get config by key [F9/M10]
     * - VALSET (0x8A): Set config by key [F9/M10]
     * 
     * @note M8 uses CFG-GNSS, F9/M10 use VALGET/VALSET for constellation config
     */
    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
        CLASS_MON = 0x0A,
        CLASS_RXM = 0x02,
        CLASS_TIM = 0x0d,
        MSG_ACK_NACK = 0x00,
        MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_DOP = 0x4,
        MSG_SOL = 0x6,
        MSG_PVT = 0x7,
        MSG_TIMEGPS = 0x20,
        MSG_RELPOSNED = 0x3c,
        MSG_VELNED = 0x12,
        MSG_CFG_CFG = 0x09,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_MSG = 0x01,
        MSG_CFG_NAV_SETTINGS = 0x24,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_SBAS = 0x16,
        MSG_CFG_GNSS = 0x3E,
        MSG_CFG_TP5 = 0x31,
        MSG_CFG_VALSET = 0x8A,
        MSG_CFG_VALGET = 0x8B,
        MSG_MON_HW = 0x09,
        MSG_MON_HW2 = 0x0B,
        MSG_MON_VER = 0x04,
        MSG_NAV_SVINFO = 0x30,
        MSG_RXM_RAW = 0x10,
        MSG_RXM_RAWX = 0x15,
        MSG_TIM_TM2 = 0x03
    };
    
    /**
     * @enum ubx_gnss_identifier
     * @brief GNSS constellation identifiers in CFG-GNSS messages
     * 
     * @details Used in CFG-GNSS configuration (M8) to enable/disable GNSS systems.
     *          F9/M10 use ConfigKey enums instead for constellation configuration.
     */
    enum ubx_gnss_identifier {
        GNSS_GPS     = 0x00,
        GNSS_SBAS    = 0x01,
        GNSS_GALILEO = 0x02,
        GNSS_BEIDOU  = 0x03,
        GNSS_IMES    = 0x04,
        GNSS_QZSS    = 0x05,
        GNSS_GLONASS = 0x06
    };
    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1,
        NAV_STATUS_DGPS_USED = 2
    };
    /**
     * @enum ubx_hardware_version
     * @brief u-blox hardware generation detection
     * 
     * @details Hardware generation is detected from MON-VER message (hwVersion and
     *          swVersion strings). Determines which configuration protocol to use:
     *          - M8 and earlier: Legacy CFG-* messages
     *          - F9/M9/M10: Modern VALGET/VALSET interface
     * 
     * Hardware Capabilities by Generation:
     * - ANTARIS/5/6/7: Legacy, limited GNSS, basic features
     * - M8: Multi-GNSS, RTK capable (M8P), good accuracy
     * - F9: Multi-band GNSS, centimeter RTK, moving baseline, IMU (F9R)
     * - M9: Improved jamming resistance, modern config interface
     * - M10: Latest generation, enhanced multi-band, power efficiency
     * 
     * @note F9/M9/M10 values (0x80+) are driver-internal, not from u-blox spec
     */
    enum ubx_hardware_version {
        ANTARIS = 0,
        UBLOX_5,
        UBLOX_6,
        UBLOX_7,
        UBLOX_M8,
        UBLOX_F9 = 0x80, // comes from MON_VER hwVersion/swVersion strings
        UBLOX_M9 = 0x81, // comes from MON_VER hwVersion/swVersion strings
        UBLOX_M10 = 0x82,
        UBLOX_UNKNOWN_HARDWARE_GENERATION = 0xff // not in the ublox spec used for
                                                 // flagging state in the driver
    };

    /**
     * @enum ubx_hardware_variant
     * @brief Hardware variant within a generation (e.g., F9 ZED vs NEO)
     * 
     * @details Parsed from MON-VER extension strings. Variants have different
     *          feature sets (ZED typically has more features than NEO).
     */
    enum ubx_hardware_variant {
        UBLOX_F9_ZED, // comes from MON_VER extension strings
        UBLOX_F9_NEO, // comes from MON_VER extension strings
        UBLOX_UNKNOWN_HARDWARE_VARIANT = 0xff
    };

    /**
     * @enum config_step
     * @brief Configuration state machine steps
     * 
     * @details The driver sequences through these states during initialization to
     *          configure the GPS receiver. Each step polls or sets specific parameters.
     *          The order is designed to:
     *          1. Detect hardware version/capabilities (VERSION, F9, M10)
     *          2. Configure basic operation (NAV_RATE, PORT)
     *          3. Enable required messages (PVT/POSLLH/STATUS/VELNED/etc.)
     *          4. Set navigation parameters (NAV_SETTINGS, GNSS, SBAS)
     *          5. Configure role-specific features (TMODE, RTK_MOVBASE)
     *          6. Enable monitoring (MON_HW, MON_HW2, DOP)
     *          7. Enable advanced features (TIM_TM2, RAW/RAWX for RTK base, L5)
     * 
     * Configuration Steps:
     * - VERSION: Query MON-VER to detect hardware generation
     * - F9/F9_VALIDATE/M10: Detect and validate F9/M9/M10 receivers
     * - PVT/SOL/POSLLH/STATUS/VELNED: Enable navigation message outputs
     * - NAV_RATE: Set navigation solution rate (typically 5Hz)
     * - NAV_SETTINGS: Configure dynamic model (airborne, automotive, etc.)
     * - GNSS: Configure constellation usage (GPS, GLONASS, Galileo, BeiDou, etc.)
     * - SBAS: Configure SBAS (WAAS/EGNOS/MSAS) augmentation
     * - TMODE: Configure time mode for base station operation
     * - RTK_MOVBASE: Configure moving baseline base/rover relationship
     * - RAW/RAWX: Enable raw measurement output for RTK base station
     * - L5: Enable GPS L5 signal (if supported)
     * 
     * @note Configuration continues until STEP_LAST is reached and all
     *       _unconfigured_messages bits are cleared.
     */
    enum config_step {
        STEP_PVT = 0,
        STEP_NAV_RATE, // poll NAV rate
        STEP_SOL,
        STEP_PORT,
        STEP_STATUS,
        STEP_POSLLH,
        STEP_VELNED,
        STEP_TIMEGPS,
        STEP_POLL_SVINFO, // poll svinfo
        STEP_POLL_SBAS, // poll SBAS
        STEP_POLL_NAV, // poll NAV settings
        STEP_POLL_GNSS, // poll GNSS
        STEP_POLL_TP5, // poll TP5
        STEP_TMODE, // set TMODE-MODE
        STEP_DOP,
        STEP_MON_HW,
        STEP_MON_HW2,
        STEP_RAW,
        STEP_RAWX,
        STEP_VERSION,
        STEP_RTK_MOVBASE, // setup moving baseline
        STEP_TIM_TM2,
        STEP_F9,
        STEP_F9_VALIDATE,
        STEP_M10,
        STEP_L5,
        STEP_LAST
    };

    // Packet checksum accumulators
    uint8_t         _ck_a;
    uint8_t         _ck_b;

    // State machine state
    uint8_t         _step;
    uint8_t         _msg_id;
    uint16_t        _payload_length;
    uint16_t        _payload_counter;

    uint8_t         _class;
    bool            _cfg_saved;

    uint32_t        _last_vel_time;
    uint32_t        _last_pos_time;
    uint32_t        _last_cfg_sent_time;
    uint8_t         _num_cfg_save_tries;
    uint32_t        _last_config_time;
    uint32_t        _f9_config_time;
    uint16_t        _delay_time;
    uint8_t         _next_message { STEP_PVT };
    uint8_t         _ublox_port { 255 };
    bool            _have_version;
    struct ubx_mon_ver _version;
    char            _module[UBLOX_MODULE_LEN];
    uint32_t        _unconfigured_messages {CONFIG_ALL};
    uint8_t         _hardware_generation { UBLOX_UNKNOWN_HARDWARE_GENERATION };
    uint8_t         _hardware_variant;
    uint32_t        _last_pvt_itow;
    uint32_t        _last_relposned_itow;
    uint32_t        _last_relposned_ms;

    // the role set from GPS_TYPE
    AP_GPS::GPS_Role role;

    // do we have new position information?
    bool            _new_position:1;
    // do we have new speed information?
    bool            _new_speed:1;

    uint8_t         _disable_counter;

    /**
     * @brief Process complete received UBX message and update GPS state
     * 
     * @return true if message was successfully parsed and state updated
     * 
     * @details Called by read() after complete message received. Dispatches to
     *          message-specific handlers based on class/ID. Updates position,
     *          velocity, time, satellite info, and handles configuration responses.
     * 
     * Message Processing:
     * - NAV messages: Update position, velocity, time, satellites
     * - ACK messages: Track configuration acknowledgments
     * - MON messages: Hardware status, version info
     * - CFG messages: Configuration responses (VALGET results)
     * - RXM messages: Raw measurements for RTK base
     * 
     * @note This is where GPS state structure is updated for use by EKF and navigation
     * 
     * Source: libraries/AP_GPS/AP_GPS_UBLOX.cpp
     */
    bool        _parse_gps();

    // used to update fix between status and position packets
    AP_GPS::GPS_Status next_fix { AP_GPS::NO_FIX };

    bool _cfg_needs_save;

    bool noReceivedHdop { true };
    
    bool havePvtMsg;

    /**
     * @struct config_list
     * @brief Key-value pair for VALSET configuration (F9/M10)
     * 
     * @details Used to batch multiple configuration keys for efficient
     *          VALSET transactions. Assumes little-endian byte order.
     */
    struct PACKED config_list {
        ConfigKey key;
        // support up to 4 byte values, assumes little-endian
        uint32_t value;
    };

    /**
     * @brief Configure output rate for specific UBX message
     * 
     * @param[in] msg_class UBX message class (e.g., 0x01 for NAV)
     * @param[in] msg_id Message ID within class
     * @param[in] rate Output rate (0=disable, 1=every solution, N=every Nth solution)
     * 
     * @return true if rate configuration message sent successfully
     * 
     * @details Sends CFG-MSG message to configure message output frequency.
     *          Rate is relative to navigation solution rate (e.g., if nav rate is
     *          5Hz and message rate is 1, message outputs at 5Hz).
     * 
     * @note Different messages have different recommended rates based on their
     *       importance and data rate constraints
     */
    bool        _configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    
    /**
     * @brief Set single configuration key via VALSET (F9/M10)
     * 
     * @param[in] key Configuration key ID
     * @param[in] value Pointer to value data (size determined by key type)
     * @param[in] layers Storage layers to update (RAM/BBR/Flash)
     * 
     * @return true if VALSET message sent successfully
     * 
     * @details Modern configuration method for F9/M9/M10. More efficient than
     *          legacy CFG messages. Layers control where configuration is stored.
     */
    bool        _configure_valset(ConfigKey key, const void *value, uint8_t layers=UBX_VALSET_LAYER_ALL);
    
    /**
     * @brief Set multiple configuration keys via VALSET transaction
     * 
     * @param[in] list Array of key-value pairs
     * @param[in] count Number of items in list
     * @param[in] layers Storage layers to update
     * 
     * @return true if all VALSET messages sent successfully
     * 
     * @details Batches multiple configuration keys into single or multiple VALSET
     *          messages for efficiency. Used for role-specific configurations like
     *          moving baseline setup.
     */
    bool        _configure_list_valset(const config_list *list, uint8_t count, uint8_t layers=UBX_VALSET_LAYER_ALL);
    
    /**
     * @brief Query configuration key value via VALGET (F9/M10)
     * 
     * @param[in] key Configuration key to query
     * 
     * @return true if VALGET request sent successfully
     * 
     * @details Requests current value of configuration key. Response comes via
     *          CFG-VALGET message which is parsed in _parse_gps().
     */
    bool        _configure_valget(ConfigKey key);
    
    /**
     * @brief Configure navigation solution output rate
     * 
     * @details Sets measurement rate (typically 200ms for 5Hz) and navigation rate.
     *          Sends CFG-RATE message. Higher rates increase CPU load but improve
     *          navigation responsiveness.
     */
    void        _configure_rate(void);
    
    /**
     * @brief Configure SBAS (satellite-based augmentation system)
     * 
     * @param[in] enable true to enable SBAS (WAAS/EGNOS/MSAS), false to disable
     * 
     * @details SBAS can improve accuracy by 1-3 meters in coverage areas but may
     *          reduce reliability in areas with poor SBAS satellite visibility.
     */
    void        _configure_sbas(bool enable);
    
    /**
     * @brief Update UBX Fletcher-16 checksum
     * 
     * @param[in] data Data bytes to add to checksum
     * @param[in] len Number of bytes
     * @param[in,out] ck_a Checksum A accumulator
     * @param[in,out] ck_b Checksum B accumulator
     * 
     * @details UBX uses 8-bit Fletcher algorithm: ck_a += byte, ck_b += ck_a
     */
    void        _update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b);
    
    /**
     * @brief Send UBX message to GPS receiver
     * 
     * @param[in] msg_class UBX message class
     * @param[in] msg_id Message ID within class
     * @param[in] msg Pointer to message payload (can be NULL for poll messages)
     * @param[in] size Payload size in bytes (0 for poll messages)
     * 
     * @return true if message sent successfully to UART
     * 
     * @details Constructs complete UBX message: preamble + header + payload + checksum.
     *          Used for both configuration commands and message polls.
     */
    bool        _send_message(uint8_t msg_class, uint8_t msg_id, const void *msg, uint16_t size);
    
    /**
     * @brief Send next message rate configuration in sequence
     * 
     * @details Called iteratively during configuration to enable required navigation
     *          messages one at a time. Tracks which messages have been configured.
     */
    void	send_next_rate_update(void);
    
    /**
     * @brief Poll current output rate for specific message
     * 
     * @param[in] msg_class UBX message class
     * @param[in] msg_id Message ID
     * 
     * @return true if poll request sent successfully
     * 
     * @details Sends CFG-MSG poll (no payload) to query current rate. Used to
     *          verify configuration was applied correctly.
     */
    bool        _request_message_rate(uint8_t msg_class, uint8_t msg_id);
    
    /**
     * @brief Request next configuration in state machine sequence
     * 
     * @details Implements configuration state machine. Advances through config_step
     *          enum, sending appropriate configuration or poll messages for each step.
     *          Tracks progress via _unconfigured_messages bitmask.
     * 
     * @note This is the heart of the GPS configuration logic
     * 
     * Source: libraries/AP_GPS/AP_GPS_UBLOX.cpp
     */
    void        _request_next_config(void);
    
    /**
     * @brief Poll port configuration
     * 
     * @details Sends CFG-PRT poll to determine which UART port GPS is connected to.
     *          Needed for configuring port-specific settings.
     */
    void        _request_port(void);
    
    /**
     * @brief Poll hardware and firmware version
     * 
     * @details Sends MON-VER poll to get version strings. Used to detect hardware
     *          generation (M8/F9/M9/M10) and adapt configuration protocol.
     */
    void        _request_version(void);
    
    /**
     * @brief Save current configuration to non-volatile memory
     * 
     * @details Sends CFG-CFG message to save RAM configuration to Flash. Configuration
     *          survives power cycle. Saves I/O, message, navigation, and antenna settings.
     */
    void        _save_cfg(void);
    
    /**
     * @brief Verify message rate matches requested value
     * 
     * @param[in] msg_class Message class to verify
     * @param[in] msg_id Message ID to verify  
     * @param[in] rate Expected rate value
     * 
     * @details Compares received CFG-MSG response against requested rate. Retries
     *          configuration if mismatch detected.
     */
    void        _verify_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
    
    /**
     * @brief Check for GPS time rollover and update time tracking
     * 
     * @param[in] itow Current GPS time of week in milliseconds
     * 
     * @details Monitors iTOW for week rollovers and updates internal time tracking.
     *          Prevents incorrect time jumps in navigation solution.
     */
    void        _check_new_itow(uint32_t itow);

    /**
     * @brief Handle unexpected UBX message reception
     * 
     * @details Called when message received that wasn't requested. Logs warning
     *          and helps diagnose communication or configuration issues.
     */
    void unexpected_message(void);
    
    /**
     * @brief Log MON-HW hardware status to dataflash
     * 
     * @details Records jamming indicator, antenna status, and other hardware
     *          monitoring data for post-flight analysis.
     */
    void log_mon_hw(void);
    
    /**
     * @brief Log MON-HW2 extended hardware status
     * 
     * @details Records additional RF and oscillator status information.
     */
    void log_mon_hw2(void);
    
    /**
     * @brief Log TIM-TM2 time mark data
     * 
     * @details Records time pulse measurements for precision timing applications.
     */
    void log_tim_tm2(void);
    
    /**
     * @brief Log RXM-RAW raw measurement data
     * 
     * @param[in] raw RXM-RAW message structure with measurements
     * 
     * @details Logs carrier phase and pseudorange measurements. Used for RTK base
     *          station raw data recording (M8 receivers).
     */
    void log_rxm_raw(const struct ubx_rxm_raw &raw);
    
    /**
     * @brief Log RXM-RAWX extended raw measurement data
     * 
     * @param[in] raw RXM-RAWX message structure with measurements
     * 
     * @details Logs multi-GNSS raw measurements with extended precision. Used for
     *          RTK base station (F9/M10 receivers). More capable than RXM-RAW.
     */
    void log_rxm_rawx(const struct ubx_rxm_rawx &raw);

#if GPS_MOVING_BASELINE
    /**
     * @brief Check if moving baseline should use UART2 for corrections
     * 
     * @return true if GPS_DRV_OPTIONS bit UBX_MBUseUart2 is set
     * 
     * @details Some moving baseline configurations output RTCM on UART2 while
     *          control/telemetry remains on UART1. Controlled by driver option.
     */
    bool mb_use_uart2(void) const {
        return option_set(AP_GPS::DriverOptions::UBX_MBUseUart2)?true:false;
    }
#endif

    /**
     * @brief Get size in bytes of configuration key value
     * 
     * @param[in] key Configuration key
     * 
     * @return Size of value in bytes (1, 2, 4, or 8)
     * 
     * @details Determines value size from key type bits for VALSET/VALGET operations.
     */
    uint8_t config_key_size(ConfigKey key) const;

    /**
     * @brief Configure set of key/value pairs and track completion
     * 
     * @param[in] list Array of configuration key-value pairs
     * @param[in] count Number of items in list
     * @param[in] unconfig_bit Bit in _unconfigured_messages to clear when complete
     * @param[in] layers Storage layers (RAM/BBR/Flash)
     * 
     * @return true if configuration initiated successfully
     * 
     * @details Uses VALGET to verify current values, then VALSET to update any that
     *          don't match. Tracks progress and clears unconfig_bit when all keys set.
     *          Used for role-specific configurations (RTK base, rover, moving baseline).
     */
    bool _configure_config_set(const config_list *list, uint8_t count, uint32_t unconfig_bit, uint8_t layers=UBX_VALSET_LAYER_ALL);

    /**
     * @brief Find configuration key in active_config list
     * 
     * @param[in] key Configuration key to search for
     * 
     * @return Index in active_config.list, or -1 if not found
     * 
     * @details Used to match VALGET responses to pending configuration items.
     */
    int8_t find_active_config_index(ConfigKey key) const;

    /**
     * @brief Check if receiver supports F9-style VALGET/VALSET configuration
     * 
     * @return true for F9, M9, M10 hardware generations
     * 
     * @details F9/M9/M10 use modern configuration interface. M8 and earlier use
     *          legacy CFG-* messages.
     */
    bool supports_F9_config(void) const;

    /**
     * @brief Check if configuration key controls GNSS constellation
     * 
     * @param[in] key Configuration key to check
     * 
     * @return true if key is in CFG_SIGNAL_* family
     * 
     * @details Used to identify constellation enable/disable keys for proper
     *          configuration sequencing.
     */
    bool is_gnss_key(ConfigKey key) const;

    /**
     * @brief Build GNSS constellation configuration for F9P
     * 
     * @return Number of config items in config_GNSS array
     * 
     * @details Dynamically builds list of constellation enable/disable keys based
     *          on GPS_GNSS_MODE parameter. Supports GPS, GLONASS, Galileo, BeiDou,
     *          QZSS, SBAS, NavIC, and multi-band L5.
     */
    uint8_t populate_F9_gnss(void);
    
    /// @brief Tracks which GNSS configuration was last applied to avoid redundant updates
    uint8_t last_configured_gnss;

    /// @brief Configured PPS (pulse-per-second) output frequency in Hz
    uint8_t _pps_freq = 1;
    
#ifdef HAL_GPIO_PPS
    /**
     * @brief Interrupt handler for PPS pin transitions
     * 
     * @param[in] pin GPIO pin number that triggered interrupt
     * @param[in] high true if pin went high, false if low
     * @param[in] timestamp_us Microsecond timestamp of edge
     * 
     * @details Records precise timing of PPS pulses for time synchronization.
     *          Used to align system time with GPS time for precision applications.
     */
    void pps_interrupt(uint8_t pin, bool high, uint32_t timestamp_us);
    
    /**
     * @brief Set desired PPS output frequency
     * 
     * @param[in] freq Desired frequency in Hz (typically 1Hz for standard PPS)
     * 
     * @details Configures GPS to output time pulse at specified frequency via
     *          CFG-TP5 message. Used for external device synchronization.
     */
    void set_pps_desired_freq(uint8_t freq) override;
#endif

    /**
     * @struct active_config
     * @brief Tracks progress of multi-step VALSET configuration
     * 
     * @details Used when applying role-specific configurations (RTK base, rover,
     *          moving baseline). Manages iterative VALGET verification and VALSET
     *          application across multiple keys.
     */
    struct {
        const config_list *list;        ///< Pointer to config key-value array
        uint8_t count;                  ///< Number of items in list
        uint32_t done_mask;             ///< Bitmask of completed config items
        uint32_t unconfig_bit;          ///< Bit in _unconfigured_messages to clear
        uint8_t layers;                 ///< VALSET storage layers (RAM/BBR/Flash)
        int8_t fetch_index;             ///< Next index to VALGET verify (-1 if done)
        int8_t set_index;               ///< Next index to VALSET apply (-1 if done)
    } active_config;
    
    /// @brief Use individual VALGET for each key vs batched (compatibility mode)
    bool use_single_valget;

#if GPS_MOVING_BASELINE
    /**
     * @brief Configuration keys for moving baseline base on UART1
     * 
     * @details Configures F9P as RTK base outputting RTCM3 on UART1.
     *          Enables raw measurements (RAWX/SFRBX) for RTCM generation.
     */
    static const config_list config_MB_Base_uart1[];
    
    /**
     * @brief Configuration keys for moving baseline base on UART2
     * 
     * @details Configures F9P as RTK base outputting RTCM3 on UART2.
     *          Allows UART1 for telemetry while UART2 provides corrections.
     */
    static const config_list config_MB_Base_uart2[];

    /**
     * @brief Configuration keys for moving baseline rover on UART1
     * 
     * @details Configures F9P as RTK rover receiving RTCM3 on UART1.
     *          Enables RELPOSNED for relative position output.
     */
    static const config_list config_MB_Rover_uart1[];
    
    /**
     * @brief Configuration keys for moving baseline rover on UART2
     * 
     * @details Configures F9P as RTK rover receiving RTCM3 on UART2.
     *          Allows UART1 for telemetry while UART2 receives corrections.
     */
    static const config_list config_MB_Rover_uart2[];

    /**
     * @brief RTCM3 parser instance for moving baseline base mode
     * 
     * @details Dynamically allocated when operating as moving baseline base.
     *          Parses raw measurements into RTCM3 messages for rover transmission.
     *          Freed on destruction.
     */
    RTCM3_Parser *rtcm3_parser;
#endif // GPS_MOVING_BASELINE

    /// @brief True if receiver supports L5 band signals (F9/M10 with L5 capability)
    bool supports_l5;
    
    /**
     * @brief Configuration keys specific to M10 hardware
     * 
     * @details M10-specific settings for optimal performance and feature enablement.
     */
    static const config_list config_M10[];
    
    /**
     * @brief L5 signal health override enable configuration
     * 
     * @details Enables L5 health status override. Required for L5 usage on some
     *          receivers when L5 satellite health status is not yet broadcast.
     */
    static const config_list config_L5_ovrd_ena[];
    
    /**
     * @brief L5 signal health override disable configuration
     * 
     * @details Disables L5 health override once L5 constellation is fully operational.
     */
    static const config_list config_L5_ovrd_dis[];
    
    /**
     * @brief Dynamically allocated GNSS constellation configuration
     * 
     * @details Scratch space for building constellation enable/disable key-value pairs
     *          based on GPS_GNSS_MODE parameter. Allocated in populate_F9_gnss(),
     *          freed on destruction or reconfiguration.
     */
    config_list* config_GNSS;
};

#endif
