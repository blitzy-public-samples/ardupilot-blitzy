/**
 * @file AP_Frsky_config.h
 * @brief FrSky telemetry protocol feature configuration
 * 
 * @details This header defines compile-time configuration flags for the FrSky
 *          telemetry subsystem, which provides bidirectional communication with
 *          FrSky radio receivers and transmitters.
 *          
 *          FrSky telemetry enables the vehicle to send flight data (battery voltage,
 *          GPS position, attitude, etc.) to FrSky receivers/transmitters, and optionally
 *          receive commands from the ground station through the telemetry link.
 *          
 *          The configuration system supports three protocol variants:
 *          - FrSky D protocol: Legacy 8-bit protocol
 *          - FrSky SmartPort (SPort): Modern 16-bit protocol with higher data rates
 *          - SmartPort Passthrough: Enhanced protocol with MAVLink translation
 *          
 *          Configuration Flag Hierarchy:
 *          
 *          AP_FRSKY_TELEM_ENABLED (base)
 *              ├── AP_FRSKY_D_TELEM_ENABLED (D protocol)
 *              └── AP_FRSKY_SPORT_TELEM_ENABLED (SmartPort protocol)
 *                      └── AP_FRSKY_SPORT_PASSTHROUGH_ENABLED (Passthrough mode)
 *                              └── HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL (Bidirectional)
 *          
 *          These flags enable board-specific customization through hwdef files.
 *          Disabling unused protocols reduces binary size (estimated savings):
 *          - Base telemetry disabled: ~8-12 KB
 *          - D protocol only: ~2-3 KB savings vs full support
 *          - SmartPort only: ~1-2 KB savings vs full support
 *          - Passthrough disabled: ~3-4 KB savings
 *          - Bidirectional disabled: ~1-2 KB savings
 *          
 *          Override these flags in hwdef.dat files for boards with limited flash:
 *          define AP_FRSKY_TELEM_ENABLED 0  # Disable all FrSky telemetry
 *          define AP_FRSKY_D_TELEM_ENABLED 0  # Disable D protocol only
 *          
 * @note These are compile-time flags, not runtime parameters. Changes require rebuild.
 * @warning Disabling features will make corresponding telemetry functionality unavailable.
 * 
 * @see libraries/AP_Frsky_Telem/AP_Frsky_Telem.h for the main telemetry implementation
 * @see libraries/AP_Frsky_Telem/README.md for protocol details and usage
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @def AP_FRSKY_TELEM_ENABLED
 * @brief Master enable flag for all FrSky telemetry functionality
 * 
 * When enabled (1), includes FrSky telemetry support in the build.
 * When disabled (0), completely removes FrSky telemetry code, saving ~8-12 KB flash.
 * 
 * This is the base flag that controls all FrSky telemetry variants. Disabling this
 * flag will automatically disable all dependent protocols (D, SmartPort, Passthrough).
 * 
 * Default: 1 (enabled)
 * Override: Can be set to 0 in hwdef.dat for boards with limited flash
 * Dependencies: None (this is the root flag)
 * Dependents: AP_FRSKY_D_TELEM_ENABLED, AP_FRSKY_SPORT_TELEM_ENABLED
 */
#ifndef AP_FRSKY_TELEM_ENABLED
#define AP_FRSKY_TELEM_ENABLED 1
#endif

/**
 * @def AP_FRSKY_D_TELEM_ENABLED
 * @brief Enable FrSky D protocol telemetry support
 * 
 * Controls support for the legacy FrSky D protocol, an 8-bit telemetry protocol
 * used by older FrSky receivers (D4R-II, D8R-II series).
 * 
 * The D protocol transmits basic telemetry data including:
 * - GPS coordinates and altitude
 * - Battery voltage and current
 * - RSSI (signal strength)
 * - Flight mode
 * 
 * Default: Inherits from AP_FRSKY_TELEM_ENABLED (enabled if base is enabled)
 * Override: Set to 0 in hwdef.dat to disable D protocol support only
 * Example: define AP_FRSKY_D_TELEM_ENABLED 0
 * 
 * Dependencies: Requires AP_FRSKY_TELEM_ENABLED
 * Binary Size: Disabling saves ~1-2 KB flash
 * 
 * @note D protocol is unidirectional (vehicle to ground only)
 * @note Consider disabling if only SmartPort receivers are used
 */
#ifndef AP_FRSKY_D_TELEM_ENABLED
#define AP_FRSKY_D_TELEM_ENABLED AP_FRSKY_TELEM_ENABLED
#endif

/**
 * @def AP_FRSKY_SPORT_TELEM_ENABLED
 * @brief Enable FrSky SmartPort (SPort) protocol telemetry support
 * 
 * Controls support for the FrSky SmartPort protocol, a modern 16-bit bidirectional
 * telemetry protocol used by newer FrSky receivers (X series: X4R, X6R, X8R, etc.).
 * 
 * SmartPort provides enhanced telemetry capabilities over D protocol:
 * - Higher data rates and precision (16-bit vs 8-bit)
 * - More telemetry fields (30+ sensor IDs)
 * - Bidirectional communication support
 * - Better error detection
 * 
 * Telemetry data includes:
 * - GPS position, speed, altitude, heading, satellite count
 * - Battery voltage, current, consumption, cell voltages
 * - Attitude (roll, pitch, yaw)
 * - Variometer (climb rate)
 * - Flight mode, arming status, error flags
 * - Home distance and direction
 * 
 * Default: Inherits from AP_FRSKY_TELEM_ENABLED (enabled if base is enabled)
 * Override: Set to 0 in hwdef.dat to disable SmartPort support only
 * Example: define AP_FRSKY_SPORT_TELEM_ENABLED 0
 * 
 * Dependencies: Requires AP_FRSKY_TELEM_ENABLED
 * Dependents: AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
 * Binary Size: Disabling saves ~2-3 KB flash
 * 
 * @note SmartPort uses inverted UART signaling (requires hardware inverter or software inversion)
 * @note This is the recommended protocol for modern FrSky systems
 */
#ifndef AP_FRSKY_SPORT_TELEM_ENABLED
#define AP_FRSKY_SPORT_TELEM_ENABLED AP_FRSKY_TELEM_ENABLED
#endif

/**
 * @def AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
 * @brief Enable SmartPort Passthrough protocol with enhanced MAVLink integration
 * 
 * Controls support for the SmartPort Passthrough protocol, an enhanced telemetry
 * mode that provides more comprehensive data by translating MAVLink messages into
 * SmartPort format. This enables display of ArduPilot-specific data on FrSky
 * transmitters using OpenTX/EdgeTX telemetry screens and LUA scripts.
 * 
 * Passthrough mode extends basic SmartPort with:
 * - EKF status and variance information
 * - Detailed flight mode information
 * - Fence and rally point status
 * - Mission progress and waypoint information
 * - System health and pre-arm check status
 * - Enhanced GPS data (HDOP, fix type)
 * - Terrain altitude and rangefinder data
 * - Wind speed and direction estimates
 * 
 * This is the most feature-rich FrSky telemetry mode and is recommended for
 * modern OpenTX/EdgeTX transmitters with custom LUA telemetry scripts.
 * 
 * Default: Inherits from AP_FRSKY_SPORT_TELEM_ENABLED (enabled if SmartPort enabled)
 * Override: Set to 0 in hwdef.dat to use basic SmartPort only
 * Example: define AP_FRSKY_SPORT_PASSTHROUGH_ENABLED 0
 * 
 * Dependencies: Requires AP_FRSKY_SPORT_TELEM_ENABLED
 * Dependents: HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
 * Binary Size: Disabling saves ~3-4 KB flash
 * 
 * @note Passthrough is unidirectional by default (requires BIDIRECTIONAL flag for commands)
 * @note Compatible with Yaapu FrSky Telemetry Script and similar OpenTX/EdgeTX scripts
 * @see https://github.com/yaapu/FrskyTelemetryScript for popular telemetry script
 */
#ifndef AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
#define AP_FRSKY_SPORT_PASSTHROUGH_ENABLED AP_FRSKY_SPORT_TELEM_ENABLED
#endif

/**
 * @def HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
 * @brief Enable bidirectional FrSky telemetry for receiving commands from ground station
 * 
 * Controls support for receiving commands from the FrSky transmitter through the
 * SmartPort telemetry link. When enabled, the vehicle can receive and process
 * MAVLink commands sent from OpenTX/EdgeTX LUA scripts running on the transmitter.
 * 
 * Bidirectional capability enables:
 * - Parameter changes from transmitter
 * - Mission upload/download
 * - Flight mode changes via telemetry link
 * - Arming/disarming commands
 * - Camera and gimbal control
 * - Rally point and fence management
 * - Real-time tuning adjustments
 * 
 * This feature is particularly useful for:
 * - Field configuration without laptop/GCS
 * - Emergency parameter adjustments during flight
 * - Integration with advanced OpenTX/EdgeTX LUA scripts
 * - Backup command channel if primary RC fails
 * 
 * Default: Inherits from AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
 * Override: Set to 0 in hwdef.dat to disable command reception
 * Example: define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 0
 * 
 * Dependencies: Requires AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
 * Binary Size: Disabling saves ~1-2 KB flash
 * 
 * @note Bidirectional mode requires compatible OpenTX/EdgeTX LUA scripts
 * @note Commands are rate-limited to prevent overwhelming the telemetry link
 * @warning Command reception introduces slight additional latency in telemetry updates
 * @warning Ensure proper authentication/validation when using remote commands
 * 
 * @see libraries/GCS_MAVLink for MAVLink command handling implementation
 */
#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL AP_FRSKY_SPORT_PASSTHROUGH_ENABLED
#endif
