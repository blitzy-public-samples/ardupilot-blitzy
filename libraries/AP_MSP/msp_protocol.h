/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file msp_protocol.h
 * @brief Comprehensive MSP (MultiWii Serial Protocol) command ID and capability flag definitions
 * 
 * @details This header defines the complete MSP protocol interface used for communication between
 *          ArduPilot and MSP-compatible ground control stations, OSD devices, and telemetry systems.
 *          
 *          The MSP protocol originated from MultiWii and has been extended by Cleanflight, Betaflight,
 *          and other flight controller projects. ArduPilot implements MSP primarily for OSD compatibility
 *          (especially DJI OSD devices) and RC telemetry integration.
 *          
 *          This file contains:
 *          - Protocol version constants
 *          - Flight controller identification strings  
 *          - Capability flags for feature negotiation
 *          - MSP command IDs for all supported messages
 *          - Message direction indicators (in/out/bidirectional)
 *          
 * @note ArduPilot emulates Betaflight MSP API version 1.42 for DJI OSD compatibility
 * @warning Modifying command IDs or API version may break compatibility with external OSD devices
 * 
 * @see libraries/AP_MSP/AP_MSP.h for the implementation
 * @see https://github.com/iNavFlight/inav/wiki/MSP-V2 for MSP protocol specification
 */

/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#pragma once

/* Protocol numbers used both by the wire format, config system, and
   field setters.
*/

/**
 * @brief MSP protocol wire format version
 * @note Version 0 indicates MSP v1 protocol. MSP v2 is indicated by MSP_V2_FRAME (255) in the command byte.
 *       ArduPilot supports both MSP v1 and MSP v2 for maximum compatibility with OSD devices.
 */
#define MSP_PROTOCOL_VERSION                0

/**
 * @brief MSP API major version number
 * @note Major version 1 indicates Betaflight/Cleanflight compatible API.
 *       API clients should not attempt communication if they cannot handle this major version.
 * @warning Changing this value will break compatibility with existing MSP clients and OSD devices
 */
#define API_VERSION_MAJOR                   1

/**
 * @brief MSP API minor version number
 * @note Set to 42 to emulate Betaflight API v1.42 for maximum compatibility with DJI OSD devices.
 *       DJI OSD firmware expects this specific version for proper operation and feature detection.
 *       Minor version increases indicate backward-compatible additions to the API.
 * @warning DJI OSD devices may malfunction if this version is changed
 */
#define API_VERSION_MINOR                   42 // for compatibility with DJI OSD

/**
 * @brief Length of API version field in bytes (major + minor)
 */
#define API_VERSION_LENGTH                  2

/**
 * @brief Flight controller firmware identifier strings (4-character format)
 * @details These 4-character ASCII strings identify the flight controller firmware variant in MSP_FC_VARIANT response.
 *          Each firmware project uses a unique identifier to prevent confusion when different firmwares
 *          implement different MSP command sets at the same API version.
 * @note ArduPilot uses "ARDU" identifier to distinguish from Betaflight/Cleanflight implementations
 */
#define MULTIWII_IDENTIFIER "MWII";         ///< MultiWii - original MSP protocol creator
#define BASEFLIGHT_IDENTIFIER "BAFL";       ///< Baseflight - early fork of MultiWii
#define BETAFLIGHT_IDENTIFIER "BTFL"        ///< Betaflight - most common MSP implementation for racing quads
#define CLEANFLIGHT_IDENTIFIER "CLFL"       ///< Cleanflight - another major MSP implementation
#define INAV_IDENTIFIER "INAV"              ///< iNav - navigation-focused flight controller
#define RACEFLIGHT_IDENTIFIER "RCFL"        ///< Raceflight - racing-focused firmware
#define ARDUPILOT_IDENTIFIER "ARDU"         ///< ArduPilot - this firmware (identifies as ARDU to MSP clients)

/**
 * @brief Length of flight controller identifier string in bytes
 * @note Always 4 characters, not null-terminated in wire format
 */
#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4

/**
 * @brief Length of flight controller version field in bytes
 * @note Version encoded as 3 bytes in MSP_FC_VERSION response
 */
#define FLIGHT_CONTROLLER_VERSION_LENGTH    3

/**
 * @brief Mask for flight controller version number (12 bits)
 */
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

/**
 * @brief Length of board identifier string in bytes
 * @note 4 uppercase alphanumeric characters identifying the specific hardware board (e.g., "CUAV", "MATK")
 */
#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.

/**
 * @brief Length of board hardware revision field in bytes
 */
#define BOARD_HARDWARE_REVISION_LENGTH      2

/**
 * @brief MSP capability flags for feature negotiation (from MultiWii/Betaflight protocol)
 * @details These bit flags are sent in capability mask responses (originally in deprecated MSP_IDENT, now in MSP_STATUS_EX).
 *          They allow MSP clients to detect which features the flight controller supports.
 *          Bit positions are standardized across MultiWii-derived flight controllers.
 * @note ArduPilot inherits these definitions for protocol compatibility but may not implement all features
 * @warning Bit positions 28-31 have conflicting definitions (navigation version vs platform flags)
 */

// Baseflight-specific capability flags (bits 30-31)
// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)  ///< Bit 31: Flight controller uses 32-bit processor (vs 8-bit)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)  ///< Bit 30: Supports Baseflight configuration commands

// MultiWii 2.3 navigation version encoding in capability mask (bits 28-31)
// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)  ///< Bit 31: Navigation version bit 4 (most significant bit)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)  ///< Bit 30: Navigation version bit 3
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)  ///< Bit 29: Navigation version bit 2  
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)  ///< Bit 28: Navigation version bit 1 (least significant bit)

// Feature capability flags (bits 0-7)
#define CAP_DYNBALANCE              ((uint32_t)1 << 2)   ///< Bit 2: Supports dynamic balancing for multirotors
#define CAP_FLAPS                   ((uint32_t)1 << 3)   ///< Bit 3: Supports flaps control for fixed-wing
#define CAP_NAVCAP                  ((uint32_t)1 << 4)   ///< Bit 4: Supports navigation features (waypoints, RTH)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)   ///< Bit 5: Supports extended auxiliary channels

/**
 * @brief Version and Identification Commands (MSP 1-5)
 * @details These commands allow MSP clients to identify the flight controller and negotiate protocol compatibility.
 *          Clients should query these first to determine which subsequent commands are supported.
 */
#define MSP_API_VERSION                 1    ///< out message - Returns MSP API version (major, minor)
#define MSP_FC_VARIANT                  2    ///< out message - Returns flight controller identifier (e.g., "ARDU")
#define MSP_FC_VERSION                  3    ///< out message - Returns firmware version (major, minor, patch)
#define MSP_BOARD_INFO                  4    ///< out message - Returns board identifier and hardware revision
#define MSP_BUILD_INFO                  5    ///< out message - Returns build date and time

/**
 * @brief Board Name Commands (MSP 10-11)
 * @details Betaflight extension for user-customizable board names displayed in OSD
 */
#define MSP_NAME                        10   ///< out message - Returns user-set board name (up to 16 characters)
#define MSP_SET_NAME                    11   ///< in message - Sets board name

/**
 * @brief MSP commands for Cleanflight/Betaflight Configuration Features (MSP 32-99)
 * @details Extended configuration commands introduced by Cleanflight and adopted by Betaflight.
 *          These provide detailed configuration access beyond the original MultiWii command set.
 * @note Not all Cleanflight commands are implemented in ArduPilot - check AP_MSP implementation
 */

// Battery configuration
#define MSP_BATTERY_CONFIG              32   ///< out message - Returns battery configuration (voltage, capacity)
#define MSP_SET_BATTERY_CONFIG          33   ///< in message - Sets battery configuration

// Flight mode configuration
#define MSP_MODE_RANGES                 34    ///< out message - Returns all mode ranges (RC channel mappings to flight modes)
#define MSP_SET_MODE_RANGE              35    ///< in message - Sets a single mode range

// Feature flags
#define MSP_FEATURE_CONFIG              36    ///< out message - Returns enabled feature bitmap
#define MSP_SET_FEATURE_CONFIG          37    ///< in message - Sets enabled feature bitmap

// Board orientation configuration
#define MSP_BOARD_ALIGNMENT_CONFIG      38    ///< out message - Returns board alignment angles (roll, pitch, yaw offsets)
#define MSP_SET_BOARD_ALIGNMENT_CONFIG  39    ///< in message - Sets board alignment angles

// Current sensor configuration
#define MSP_CURRENT_METER_CONFIG        40    ///< out message - Returns current sensor configuration (scale, offset)
#define MSP_SET_CURRENT_METER_CONFIG    41    ///< in message - Sets current sensor configuration

// Motor mixer configuration
#define MSP_MIXER_CONFIG                42    ///< out message - Returns mixer configuration (frame type)
#define MSP_SET_MIXER_CONFIG            43    ///< in message - Sets mixer configuration

// RC receiver configuration
#define MSP_RX_CONFIG                   44    ///< out message - Returns RX configuration (protocol, channel mapping)
#define MSP_SET_RX_CONFIG               45    ///< in message - Sets RX configuration

// LED configuration
#define MSP_LED_COLORS                  46   ///< out message - Returns LED color configuration
#define MSP_SET_LED_COLORS              47   ///< in message - Sets LED color configuration

#define MSP_LED_STRIP_CONFIG            48   ///< out message - Returns LED strip configuration (position, function)
#define MSP_SET_LED_STRIP_CONFIG        49   ///< in message - Sets LED strip configuration

// RSSI configuration
#define MSP_RSSI_CONFIG                 50   ///< out message - Returns RSSI source and channel configuration
#define MSP_SET_RSSI_CONFIG             51   ///< in message - Sets RSSI configuration

// In-flight adjustment configuration
#define MSP_ADJUSTMENT_RANGES           52   ///< out message - Returns in-flight adjustment ranges (tune PIDs via RC)
#define MSP_SET_ADJUSTMENT_RANGE        53   ///< in message - Sets a single adjustment range

// Serial port configuration (Cleanflight-specific)
// private - only to be used by the configurator, the commands are likely to change
#define MSP_CF_SERIAL_CONFIG            54   ///< out message - Returns serial port configuration (Cleanflight format)
#define MSP_SET_CF_SERIAL_CONFIG        55   ///< in message - Sets serial port configuration

// Voltage sensor configuration
#define MSP_VOLTAGE_METER_CONFIG        56   ///< out message - Returns voltage sensor configuration (scale, offset)
#define MSP_SET_VOLTAGE_METER_CONFIG    57   ///< in message - Sets voltage sensor configuration

// Sonar/rangefinder data
#define MSP_SONAR_ALTITUDE              58   ///< out message - Returns sonar/rangefinder altitude in centimeters

// PID controller selection
#define MSP_PID_CONTROLLER              59   ///< out message - Returns active PID controller type
#define MSP_SET_PID_CONTROLLER          60   ///< in message - Sets PID controller type

// Arming configuration
#define MSP_ARMING_CONFIG               61   ///< out message - Returns arming configuration (auto-disarm delay, etc.)
#define MSP_SET_ARMING_CONFIG           62   ///< in message - Sets arming configuration

/**
 * @brief Baseflight MSP Commands (MSP 64-99 continued)
 * @details Original Baseflight commands, adopted by Cleanflight and successors
 */

// RC channel mapping
#define MSP_RX_MAP                      64   ///< out message - Returns RC channel map (AETR ordering) and channel count
#define MSP_SET_RX_MAP                  65   ///< in message - Sets RC channel map ordering

// DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
// DEPRECATED - #define MSP_BF_CONFIG                   66 //out message baseflight-specific settings that aren't covered elsewhere
// DEPRECATED - #define MSP_SET_BF_CONFIG               67 //in message baseflight-specific settings save

// System control
#define MSP_REBOOT                      68   ///< in message - Reboot flight controller

// Use MSP_BUILD_INFO instead
// DEPRECATED - #define MSP_BF_BUILD_INFO               69 //out message build date as well as some space for future expansion

// Onboard flash dataflash management
#define MSP_DATAFLASH_SUMMARY           70   ///< out message - Returns dataflash chip information (size, used space)
#define MSP_DATAFLASH_READ              71   ///< out message - Returns dataflash content at specified address
#define MSP_DATAFLASH_ERASE             72   ///< in message - Erases dataflash chip

// No-longer needed
// DEPRECATED - #define MSP_LOOP_TIME                   73 //out message         Returns FC cycle time i.e looptime parameter // DEPRECATED
// DEPRECATED - #define MSP_SET_LOOP_TIME               74 //in message          Sets FC cycle time i.e looptime parameter    // DEPRECATED

// Failsafe configuration
#define MSP_FAILSAFE_CONFIG             75   ///< out message - Returns failsafe settings (stage delays, actions)
#define MSP_SET_FAILSAFE_CONFIG         76   ///< in message - Sets failsafe configuration

// RX failsafe per-channel configuration
#define MSP_RXFAIL_CONFIG               77   ///< out message - Returns per-channel RX failsafe settings
#define MSP_SET_RXFAIL_CONFIG           78   ///< in message - Sets per-channel RX failsafe behavior

// SD card status
#define MSP_SDCARD_SUMMARY              79   ///< out message - Returns SD card state (inserted, size, free space)

// Blackbox logging configuration
#define MSP_BLACKBOX_CONFIG             80   ///< out message - Returns blackbox logging settings (device, rate)
#define MSP_SET_BLACKBOX_CONFIG         81   ///< in message - Sets blackbox logging configuration

// Transponder (IR lap timer) configuration
#define MSP_TRANSPONDER_CONFIG          82   ///< out message - Returns IR transponder settings (for lap timing)
#define MSP_SET_TRANSPONDER_CONFIG      83   ///< in message - Sets transponder configuration

/**
 * @brief OSD Configuration Commands (MSP 84-87)
 * @details Betaflight OSD commands for configuring on-screen display elements and fonts.
 *          Critical for DJI OSD compatibility in ArduPilot.
 * @note These commands are extensively used by DJI OSD devices and MSP-based OSD systems
 */
#define MSP_OSD_CONFIG                  84   ///< out message - Returns OSD item positions and visibility flags
#define MSP_SET_OSD_CONFIG              85   ///< in message - Sets OSD item positions and visibility

#define MSP_OSD_CHAR_READ               86   ///< out message - Returns OSD font character bitmap (for custom fonts)
#define MSP_OSD_CHAR_WRITE              87   ///< in message - Writes OSD font character bitmap

// Video transmitter configuration
#define MSP_VTX_CONFIG                  88   ///< out message - Returns VTX settings (band, channel, power)
#define MSP_SET_VTX_CONFIG              89   ///< in message - Sets VTX configuration

/**
 * @brief Betaflight Advanced Configuration Commands (MSP 90-99)
 * @details Extended configuration commands from Betaflight for advanced tuning
 */
#define MSP_ADVANCED_CONFIG             90   ///< out message - Returns advanced configuration settings
#define MSP_SET_ADVANCED_CONFIG         91   ///< in message - Sets advanced configuration

#define MSP_FILTER_CONFIG               92   ///< out message - Returns filter configuration (gyro/D-term filters)
#define MSP_SET_FILTER_CONFIG           93   ///< in message - Sets filter configuration

#define MSP_PID_ADVANCED                94   ///< out message - Returns advanced PID settings
#define MSP_SET_PID_ADVANCED            95   ///< in message - Sets advanced PID settings

#define MSP_SENSOR_CONFIG               96   ///< out message - Returns sensor configuration
#define MSP_SET_SENSOR_CONFIG           97   ///< in message - Sets sensor configuration

#define MSP_CAMERA_CONTROL              98   ///< in/out message - Camera control (RunCam protocol)

#define MSP_SET_ARMING_DISABLED         99   ///< in message - Disables arming (for safety)

/**
 * @brief OSD and DisplayPort Commands (MSP 180-187)
 * @details Advanced OSD commands for video configuration and DisplayPort protocol.
 *          DisplayPort enables external OSD devices to render telemetry overlays.
 * @note MSP_DISPLAYPORT is critical for DJI OSD integration - used for real-time text overlay
 */
#define MSP_OSD_VIDEO_CONFIG            180  ///< out message - Returns OSD video timing configuration (PAL/NTSC)
#define MSP_SET_OSD_VIDEO_CONFIG        181  ///< in message - Sets OSD video configuration

// External OSD displayport mode messages
#define MSP_DISPLAYPORT                 182  ///< in/out message - DisplayPort canvas drawing commands (for external OSD rendering)

#define MSP_COPY_PROFILE                183  ///< in message - Copies settings from one profile to another

#define MSP_BEEPER_CONFIG               184  ///< out message - Returns beeper warning configuration
#define MSP_SET_BEEPER_CONFIG           185  ///< in message - Sets beeper warning flags

#define MSP_SET_TX_INFO                 186  ///< in message - Receives runtime info from TX Lua scripts (RSSI, signal quality)
#define MSP_TX_INFO                     187  ///< out message - Sends telemetry data to TX Lua scripts

/**
 * @brief MultiWii Original MSP Commands (MSP 100-139)
 * @details Core status and telemetry commands from the original MultiWii protocol.
 *          These are the foundation of MSP and are supported by virtually all MSP implementations.
 *          Most critical for basic flight data display in OSD and ground stations.
 * @note These commands are polled at high rate (typically 10-50Hz) by MSP OSD devices
 */

// See MSP_API_VERSION and MSP_MIXER_CONFIG
//DEPRECATED - #define MSP_IDENT                100    //out message         mixerMode + multiwii version + protocol version + capability variable

// Core status and sensor commands
#define MSP_STATUS               101    ///< out message - Returns flight controller status (cycle time, errors, sensors, armed state, profile)
#define MSP_RAW_IMU              102    ///< out message - Returns raw IMU data (accel XYZ, gyro XYZ, mag XYZ in sensor units)
#define MSP_SERVO                103    ///< out message - Returns servo PWM outputs (up to 8 servos, in microseconds)
#define MSP_MOTOR                104    ///< out message - Returns motor PWM outputs (all motors, in microseconds)
#define MSP_RC                   105    ///< out message - Returns RC channel values (all configured channels, in microseconds)

// Navigation and position commands
#define MSP_RAW_GPS              106    ///< out message - Returns GPS data (fix type, satellites, lat, lon, alt, speed, ground course)
#define MSP_COMP_GPS             107    ///< out message - Returns computed GPS data (distance to home, direction to home in degrees)
#define MSP_ATTITUDE             108    ///< out message - Returns attitude (roll, pitch angles in decidegrees; yaw heading in degrees)
#define MSP_ALTITUDE             109    ///< out message - Returns altitude (estimated altitude in cm, climb rate in cm/s)
#define MSP_ANALOG               110    ///< out message - Returns analog sensors (battery voltage in 0.1V, mAh consumed, RSSI)

// Configuration and tuning commands
#define MSP_RC_TUNING            111    ///< out message - Returns RC tuning (RC rate, expo, roll/pitch/yaw rates)
#define MSP_PID                  112    ///< out message - Returns PID coefficients (P, I, D for roll, pitch, yaw, etc.)
// Legacy Multiicommand that was never used.
//DEPRECATED - #define MSP_BOX                  113    //out message         BOX setup (number is dependant of your setup)
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - #define MSP_MISC                 114    //out message         powermeter trig
// Legacy Multiicommand that was never used and always wrong
//DEPRECATED - #define MSP_MOTOR_PINS           115    //out message         which pins are in use for motors & servos, for GUI

// Flight mode and configuration names
#define MSP_BOXNAMES             116    ///< out message - Returns semicolon-separated flight mode names (for display)
#define MSP_PIDNAMES             117    ///< out message - Returns semicolon-separated PID controller names
#define MSP_WP                   118    ///< out message - Returns waypoint data (WP number in request; returns lat, lon, alt, flags)
#define MSP_BOXIDS               119    ///< out message - Returns permanent box/mode IDs (for mode identification)

// Servo and navigation configuration
#define MSP_SERVO_CONFIGURATIONS 120    ///< out message - Returns all servo configurations (min, max, middle, rate)
#define MSP_NAV_STATUS           121    ///< out message - Returns navigation status (mode, state, active waypoint, errors)
#define MSP_NAV_CONFIG           122    ///< out message - Returns navigation parameters (speeds, altitudes, timeouts)
#define MSP_MOTOR_3D_CONFIG      124    ///< out message - Returns 3D/reversible ESC configuration (deadband, neutral)

// Sensor and hardware configuration
#define MSP_RC_DEADBAND          125    ///< out message - Returns RC input deadbands (for each axis)
#define MSP_SENSOR_ALIGNMENT     126    ///< out message - Returns sensor orientation (roll, pitch, yaw rotation for gyro/acc/mag)
#define MSP_LED_STRIP_MODECOLOR  127    ///< out message - Returns LED strip color-per-mode configuration

// Power system telemetry
#define MSP_VOLTAGE_METERS       128    ///< out message - Returns voltage for each configured voltage sensor
#define MSP_CURRENT_METERS       129    ///< out message - Returns current for each configured current sensor
#define MSP_BATTERY_STATE        130    ///< out message - Returns battery state (cell count, voltage, current, mAh used, capacity remaining)

// Hardware configuration
#define MSP_MOTOR_CONFIG         131    ///< out message - Returns motor configuration (min/max throttle, pole count)
#define MSP_GPS_CONFIG           132    ///< out message - Returns GPS configuration (protocol, SBAS mode, auto-config)
#define MSP_COMPASS_CONFIG       133    ///< out message - Returns compass configuration (declination)

// ESC and motor telemetry
#define MSP_ESC_SENSOR_DATA      134    ///< out message - Returns ESC telemetry (temperature, RPM, voltage, current per ESC)
#define MSP_GPS_RESCUE           135    ///< out message - Returns GPS rescue configuration (return altitude, speed, sanity checks)
#define MSP_GPS_RESCUE_PIDS      136    ///< out message - Returns GPS rescue PID values (throttle P, velocity PIDs, yaw P)

// VTX table configuration
#define MSP_VTXTABLE_BAND        137    ///< out message - Returns VTX frequency table for specified band
#define MSP_VTXTABLE_POWERLEVEL  138    ///< out message - Returns VTX power level table entries
#define MSP_MOTOR_TELEMETRY      139    ///< out message - Returns per-motor telemetry (RPM, errors, temperature, voltage, current)

/**
 * @brief MultiWii Configuration SET Commands (MSP 200-228)
 * @details Write commands for configuring flight controller settings.
 *          These correspond to many of the GET commands (MSP 100-139).
 * @note Changes made via SET commands typically require MSP_EEPROM_WRITE (250) to persist across reboots
 * @warning Writing configuration while armed may cause unexpected behavior
 */

// Input override commands
#define MSP_SET_RAW_RC           200    ///< in message - Injects RC channel values (for simulator or failsafe testing)
#define MSP_SET_RAW_GPS          201    ///< in message - Injects GPS data (for simulator or testing)

// Tuning configuration
#define MSP_SET_PID              202    ///< in message - Sets PID coefficients (P, I, D for all controllers)
// Legacy multiiwii command that was never used.
//DEPRECATED - #define MSP_SET_BOX              203    //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204    ///< in message - Sets RC tuning (rates, expo for roll/pitch/yaw)

// Calibration commands
#define MSP_ACC_CALIBRATION      205    ///< in message - Triggers accelerometer calibration (no parameters)
#define MSP_MAG_CALIBRATION      206    ///< in message - Triggers magnetometer calibration (no parameters)

// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - #define MSP_SET_MISC             207    //in message          powermeter trig + 8 free for future use

// System commands
#define MSP_RESET_CONF           208    ///< in message - Resets configuration to defaults (no parameters)
#define MSP_SET_WP               209    ///< in message - Sets/uploads waypoint (WP#, lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210    ///< in message - Selects active profile number (0-2)
#define MSP_SET_HEADING          211    ///< in message - Sets new heading hold direction (in degrees)

// Servo and motor configuration
#define MSP_SET_SERVO_CONFIGURATION 212 ///< in message - Sets servo configuration (min, max, middle, rate for one servo)
#define MSP_SET_MOTOR            214    ///< in message - Sets motor output values (for testing, motor identification, or propeller balance)

// Navigation configuration
#define MSP_SET_NAV_CONFIG       215    ///< in message - Sets navigation configuration and writes to EEPROM

// Hardware configuration
#define MSP_SET_MOTOR_3D_CONFIG  217    ///< in message - Sets 3D/reversible ESC configuration
#define MSP_SET_RC_DEADBAND      218    ///< in message - Sets RC input deadbands
#define MSP_SET_RESET_CURR_PID   219    ///< in message - Resets current PID profile to defaults
#define MSP_SET_SENSOR_ALIGNMENT 220    ///< in message - Sets sensor orientation (rotation angles)
#define MSP_SET_LED_STRIP_MODECOLOR 221 ///< in message - Sets LED strip mode-color configuration
#define MSP_SET_MOTOR_CONFIG     222    ///< in message - Sets motor configuration (min/max throttle, pole count)
#define MSP_SET_GPS_CONFIG       223    ///< in message - Sets GPS configuration
#define MSP_SET_COMPASS_CONFIG   224    ///< in message - Sets compass configuration

// GPS rescue configuration
#define MSP_SET_GPS_RESCUE       225    ///< in message - Sets GPS rescue configuration
#define MSP_SET_GPS_RESCUE_PIDS  226    ///< in message - Sets GPS rescue PID values

// VTX configuration
#define MSP_SET_VTXTABLE_BAND    227    ///< in message - Sets VTX frequency table for one band
#define MSP_SET_VTXTABLE_POWERLEVEL 228 ///< in message - Sets VTX power level table entry

// #define MSP_BIND                 240    //in message          no param
// #define MSP_ALARMS               242

/**
 * @brief System Management Commands (MSP 250-255)
 * @details Special system commands for persistence, debugging, and protocol negotiation
 */
#define MSP_EEPROM_WRITE         250    ///< in message - Writes current configuration to persistent storage (no parameters)
#define MSP_RESERVE_1            251    ///< Reserved for system usage
#define MSP_RESERVE_2            252    ///< Reserved for system usage
#define MSP_DEBUGMSG             253    ///< out message - Returns debug string buffer (ASCII text)
#define MSP_DEBUG                254    ///< out message - Returns debug values (4x int16 debug values)
#define MSP_V2_FRAME             255    ///< MSPv2 protocol indicator (not a command - signals MSP v2 frame format)

/**
 * @brief Extended Commands Not in Original MultiWii (MSP 150-249)
 * @details Additional commands added by Cleanflight/Betaflight/ArduPilot for extended functionality
 */
#define MSP_STATUS_EX            150    ///< out message - Extended status (cycle time, errors, CPU load, sensors, arming flags)
#define MSP_UID                  160    ///< out message - Returns unique device ID (typically MCU UID)
#define MSP_GPSSVINFO            164    ///< out message - Returns GPS satellite signal strength (U-Blox specific format)
#define MSP_GPSSTATISTICS        166    ///< out message - Returns GPS statistics (HDOP, VDOP, EPE, fix count)
#define MSP_MULTIPLE_MSP         230    ///< in/out message - Request multiple MSP commands in one transaction (limited by TX buffer)
#define MSP_MODE_RANGES_EXTRA    238    ///< out message - Returns extra mode range configuration data
#define MSP_ACC_TRIM             240    ///< out message - Returns accelerometer angle trim values (for level calibration)
#define MSP_SET_ACC_TRIM         239    ///< in message - Sets accelerometer angle trim values
#define MSP_SERVO_MIX_RULES      241    ///< out message - Returns all servo mixer rules
#define MSP_SET_SERVO_MIX_RULE   242    ///< in message - Sets one servo mixer rule
#define MSP_SET_PASSTHROUGH      245    ///< in message - Enables passthrough mode to peripherals (ESC programming, serial forwarding)
#define MSP_SET_RTC              246    ///< in message - Sets real-time clock (for log timestamping)
#define MSP_RTC                  247    ///< out message - Returns real-time clock value
#define MSP_SET_BOARD_INFO       248    ///< in message - Sets board information string
#define MSP_SET_SIGNATURE        249    ///< in message - Sets board signature and serial number

/**
 * @brief MSP v2 Sensor Commands (MSP2 0x1F01-0x1F06)
 * @details Extended 16-bit command IDs for MSP v2 protocol, providing enhanced sensor telemetry.
 *          These commands use MSP v2 frame format (indicated by MSP_V2_FRAME in header).
 * @note Command IDs are 16-bit values, allowing for expanded command space beyond 255
 * @warning MSP v2 frame format is different from v1 - parsers must handle both protocols
 */
#define MSP2_SENSOR_RANGEFINDER     0x1F01  ///< out message - Returns rangefinder/sonar data (distance, quality)
#define MSP2_SENSOR_OPTIC_FLOW      0x1F02  ///< out message - Returns optical flow sensor data (velocity X/Y, quality)
#define MSP2_SENSOR_GPS             0x1F03  ///< out message - Returns enhanced GPS data (extended format with more fields)
#define MSP2_SENSOR_COMPASS         0x1F04  ///< out message - Returns compass/magnetometer data (heading, field strength)
#define MSP2_SENSOR_BAROMETER       0x1F05  ///< out message - Returns barometer data (pressure, temperature, altitude)
#define MSP2_SENSOR_AIRSPEED        0x1F06  ///< out message - Returns airspeed sensor data (differential pressure, airspeed)
