/**
 * @file AP_GPS_config.h
 * @brief Compile-time configuration for AP_GPS library and backends
 * 
 * @details Central configuration header defining feature flags for GPS subsystem.
 *          Controls which GPS backends are compiled, maximum receiver count, blending
 *          support, moving baseline RTK, and platform-specific defaults. Used to
 *          optimize binary size by excluding unused GPS drivers.
 *          
 *          This file enables conditional compilation of GPS backends based on:
 *          - Platform capabilities (flash size, hardware support)
 *          - Vehicle requirements (single vs redundant GPS)
 *          - Feature needs (RTK, blending, specific protocols)
 *          
 *          Disabling unused backends can save 2-10KB flash per backend.
 *          Reducing GPS_MAX_RECEIVERS from 2 to 1 saves approximately 1KB RAM.
 * 
 * @note All configuration options can be overridden in hwdef.dat or custom builds
 * 
 * Source: libraries/AP_GPS/AP_GPS_config.h:1-124
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS_config.h>

/**
 * @brief Master enable for GPS subsystem
 * 
 * @details Set to 0 to completely disable GPS support and save flash memory and RAM.
 *          When disabled, GPS-dependent features become unavailable:
 *          - EKF3 position estimation requires alternative sources (optical flow, beacon)
 *          - Autonomous flight modes (Auto, Guided, RTL) unavailable
 *          - Velocity and position estimation degraded
 *          
 *          Disabling GPS saves approximately 30-50KB flash depending on enabled backends.
 * 
 * @note Disabling GPS prevents most outdoor autonomous flight capabilities
 * @warning Vehicles without GPS cannot perform GPS-dependent failsafes (RTL, Land)
 */
#ifndef AP_GPS_ENABLED
#define AP_GPS_ENABLED 1
#endif

#if AP_GPS_ENABLED

/**
 * @brief Maximum number of simultaneous physical GPS receivers
 * 
 * @details Defines how many physical GPS receivers can be connected simultaneously.
 *          Common configurations:
 *          - GPS_MAX_RECEIVERS = 1: Single GPS (saves ~1KB RAM)
 *          - GPS_MAX_RECEIVERS = 2: Dual GPS for redundancy or moving baseline RTK
 *          
 *          Each GPS receiver requires approximately 1KB RAM for state storage including:
 *          - Parser state and buffers
 *          - Position, velocity, accuracy data
 *          - Satellite information
 *          - Hardware health monitoring
 *          
 *          Multiple GPS receivers enable:
 *          - Redundancy: Automatic failover if one GPS fails
 *          - GPS blending: Inverse-variance weighted combination for better accuracy
 *          - Moving baseline: Two RTK GPS (base+rover) for centimeter-precision heading
 * 
 * @note This does NOT include the virtual blended GPS instance (see GPS_MAX_INSTANCES)
 * @note Moving baseline RTK requires GPS_MAX_RECEIVERS >= 2 and compatible hardware
 * 
 * @see GPS_MAX_INSTANCES for total instances including blended GPS
 * @see AP_GPS_BLENDED_ENABLED for GPS blending feature
 */
#ifndef GPS_MAX_RECEIVERS
#define GPS_MAX_RECEIVERS 2 // maximum number of physical GPS sensors allowed - does not include virtual GPS created by blending receiver data
#endif

/**
 * @brief Maximum GPS backend instances including blended virtual GPS
 * 
 * @details Total number of GPS instances available to consumers (AHRS, EKF, etc.).
 *          This includes both physical receivers and the optional blended virtual GPS.
 *          
 *          Calculation:
 *          - If GPS_MAX_RECEIVERS > 1: GPS_MAX_INSTANCES = GPS_MAX_RECEIVERS + 1
 *            (physical receivers + 1 blended instance)
 *          - If GPS_MAX_RECEIVERS == 1: GPS_MAX_INSTANCES = 1
 *            (no blending with single receiver)
 *          
 *          Example with GPS_MAX_RECEIVERS = 2:
 *          - Instance 0: Physical GPS #1
 *          - Instance 1: Physical GPS #2
 *          - Instance 2: Blended virtual GPS (if AP_GPS_BLENDED_ENABLED)
 *          
 *          The blended GPS combines data from multiple physical receivers using
 *          inverse-variance weighting based on reported accuracy to provide a
 *          more accurate position estimate than any single receiver.
 * 
 * @note Usually equals GPS_MAX_RECEIVERS or GPS_MAX_RECEIVERS + 1 if blending enabled
 * @see GPS_BLENDED_INSTANCE for the index of the blended GPS instance
 */
#if !defined(GPS_MAX_INSTANCES)
#if GPS_MAX_RECEIVERS > 1
#define GPS_MAX_INSTANCES  (GPS_MAX_RECEIVERS + 1) // maximum number of GPS instances including the 'virtual' GPS created by blending receiver data
#else
#define GPS_MAX_INSTANCES 1
#endif // GPS_MAX_RECEIVERS > 1
#endif // GPS_MAX_INSTANCES

#if GPS_MAX_RECEIVERS <= 1 && GPS_MAX_INSTANCES > 1
#error "GPS_MAX_INSTANCES should be 1 for GPS_MAX_RECEIVERS <= 1"
#endif
#endif

/**
 * @brief Default enable state for GPS backends
 * 
 * @details Used as the default value for individual backend enable flags.
 *          When AP_GPS_ENABLED is 0, all backends are automatically disabled.
 *          Individual backends can override this default.
 * 
 * @note This is an internal helper macro, not typically overridden directly
 */
#ifndef AP_GPS_BACKEND_DEFAULT_ENABLED
#define AP_GPS_BACKEND_DEFAULT_ENABLED AP_GPS_ENABLED
#endif

/**
 * @brief Enable GPS blending for multi-receiver accuracy improvement
 * 
 * @details When multiple GPS receivers are available, blending combines their data
 *          using inverse-variance weighting to produce a more accurate position
 *          estimate than any single receiver. The blended GPS appears as an
 *          additional virtual GPS instance.
 *          
 *          Blending algorithm:
 *          - Weights each GPS by inverse of reported accuracy variance
 *          - More accurate GPS contributes more to blended solution
 *          - Automatically excludes unhealthy or degraded receivers
 *          - Updates at rate of fastest contributing GPS
 *          
 *          Benefits:
 *          - Improved position accuracy (typically 20-40% better than single GPS)
 *          - Smoother position estimates
 *          - Automatic degradation handling
 *          
 *          Requirements:
 *          - GPS_MAX_RECEIVERS >= 2
 *          - GPS_MAX_INSTANCES > GPS_MAX_RECEIVERS (for blended instance)
 *          
 *          Binary size: Enabling blending adds approximately 2KB flash
 * 
 * @note Blended GPS instance index is GPS_MAX_RECEIVERS (highest instance number)
 * @note EKF can be configured to use blended GPS as primary source
 * 
 * @see GPS_BLENDED_INSTANCE for the instance index of blended GPS
 */
#if !defined(AP_GPS_BLENDED_ENABLED) && defined(GPS_MAX_INSTANCES)
#define AP_GPS_BLENDED_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED && GPS_MAX_INSTANCES > GPS_MAX_RECEIVERS
#endif

#ifndef AP_GPS_BLENDED_ENABLED
#define AP_GPS_BLENDED_ENABLED 0
#endif

/**
 * @brief Instance index of the blended virtual GPS
 * 
 * @details The blended GPS always uses the highest instance number, which equals
 *          GPS_MAX_RECEIVERS. For example, with GPS_MAX_RECEIVERS = 2:
 *          - Instance 0 and 1 are physical GPS
 *          - Instance 2 is the blended GPS
 * 
 * @note Only defined when AP_GPS_BLENDED_ENABLED is true
 */
#if AP_GPS_BLENDED_ENABLED
#define GPS_BLENDED_INSTANCE GPS_MAX_RECEIVERS  // the virtual blended GPS is always the highest instance (2)
#endif

// ========================================
// GPS Backend Protocol Drivers
// ========================================
// Each backend can be individually disabled to save flash memory.
// Typical flash savings: 2-10KB per backend depending on complexity.
// Enable only the backends needed for your GPS hardware.

/**
 * @brief Enable DroneCAN (UAVCAN) GPS support
 * 
 * @details Supports GPS receivers connected via CAN bus using DroneCAN/UAVCAN protocol.
 *          Common DroneCAN GPS modules:
 *          - Zubax GNSS 2
 *          - CUAV NEO v2 CAN
 *          - Holybro DroneCAN GPS
 *          
 *          Features:
 *          - CAN bus connection (more reliable than serial in noisy environments)
 *          - Integrated compass often included
 *          - Hot-swappable with proper CAN topology
 *          - Firmware updates over CAN
 *          
 *          Requirements: HAL_ENABLE_DRONECAN_DRIVERS must be enabled
 *          Flash size: Approximately 8KB for DroneCAN GPS support
 * 
 * @note Requires CAN bus hardware support on flight controller
 * @see AP_GPS_DroneCAN.cpp for implementation
 */
#ifndef AP_GPS_DRONECAN_ENABLED
#define AP_GPS_DRONECAN_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

/**
 * @brief Enable Emlid Reach Binary (ERB) protocol support
 * 
 * @details Supports Emlid Reach RTK GPS receivers using their proprietary binary protocol.
 *          Emlid Reach modules (RS+, RS2, M+, M2) provide RTK precision positioning.
 *          
 *          Features:
 *          - RTK corrections support (base station and rover modes)
 *          - Centimeter-level accuracy with RTK fix
 *          - Moving baseline support for dual-GPS heading
 *          - Raw observation data for post-processing
 *          
 *          Flash size: Approximately 4KB
 * 
 * @note Emlid devices can also use NMEA protocol (less efficient)
 * @see AP_GPS_ERB.cpp for implementation
 */
#ifndef AP_GPS_ERB_ENABLED
  #define AP_GPS_ERB_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Trimble GSOF (General Serial Output Format) protocol support
 * 
 * @details Supports Trimble GPS receivers using GSOF binary protocol.
 *          Used with Trimble professional-grade GNSS receivers including:
 *          - Trimble BD9xx series
 *          - Trimble SPS series
 *          - Trimble R-series
 *          
 *          GSOF provides:
 *          - High-precision positioning data
 *          - Multiple constellation support (GPS, GLONASS, Galileo, BeiDou)
 *          - RTK correction support
 *          - Dual antenna heading (some models)
 *          
 *          Flash size: Approximately 3KB
 * 
 * @note Trimble receivers are commonly used in precision agriculture and surveying
 * @see AP_GPS_GSOF.cpp for implementation
 */
#ifndef AP_GPS_GSOF_ENABLED
  #define AP_GPS_GSOF_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable MAVLink GPS injection support
 * 
 * @details Allows GPS data to be injected via MAVLink GPS_INPUT messages from:
 *          - External positioning systems (motion capture, UWB, RTK base stations)
 *          - Companion computers providing GPS data
 *          - Ground control stations forwarding external GPS
 *          - Simulation environments
 *          
 *          Use cases:
 *          - Indoor positioning with external tracking systems
 *          - Cellular/network RTK corrections
 *          - GPS data from companion computer processing
 *          - Development and testing with injected GPS
 *          
 *          Requirements: HAL_GCS_ENABLED (MAVLink support)
 *          Flash size: Approximately 2KB
 * 
 * @note Injected GPS appears as a regular GPS instance to EKF and AHRS
 * @warning Ensure injected data has correct timestamps and coordinate frames
 * @see AP_GPS_MAV.cpp for implementation
 */
#ifndef AP_GPS_MAV_ENABLED
  #define AP_GPS_MAV_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

/**
 * @brief Enable MSP (MultiWii Serial Protocol) GPS support
 * 
 * @details Supports GPS data received via MSP protocol, commonly used with:
 *          - FPV flight controllers as external GPS source
 *          - MSP-based OSD systems with GPS capability
 *          - Third-party GPS modules using MSP protocol
 *          
 *          MSP GPS provides basic position, velocity, and satellite data.
 *          Typically used in mixed ArduPilot/MSP hardware configurations.
 *          
 *          Requirements: HAL_MSP_SENSORS_ENABLED
 *          Flash size: Approximately 2KB
 * 
 * @note MSP GPS is less common than other protocols
 * @see AP_GPS_MSP.cpp for implementation
 */
#ifndef HAL_MSP_GPS_ENABLED
#define HAL_MSP_GPS_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED && HAL_MSP_SENSORS_ENABLED
#endif

/**
 * @brief Enable NMEA 0183 protocol support
 * 
 * @details Supports GPS receivers using standard NMEA 0183 ASCII protocol.
 *          NMEA is the most universal GPS protocol, supported by nearly all receivers.
 *          
 *          Supported NMEA sentences:
 *          - GGA: Fix data (position, altitude, fix quality, satellites)
 *          - RMC: Recommended minimum data (position, velocity, date/time)
 *          - VTG: Velocity and track (ground speed and course)
 *          - HDT: Heading (if supported by receiver)
 *          - GRS: Range residuals (accuracy information)
 *          
 *          Common NMEA GPS modules:
 *          - Most consumer GPS modules
 *          - Many smartphones and tablets
 *          - Legacy GPS receivers
 *          - Marine GPS systems
 *          
 *          Advantages:
 *          - Universal compatibility
 *          - Human-readable ASCII format
 *          - Easy to debug with serial monitor
 *          
 *          Disadvantages:
 *          - Lower update rates than binary protocols (typically 1-10Hz)
 *          - Higher bandwidth usage than binary
 *          - Parsing overhead
 *          
 *          Flash size: Approximately 5KB
 * 
 * @note Most GPS receivers support both NMEA and a proprietary binary protocol
 * @note Binary protocols (u-blox, SBF) are preferred when available for efficiency
 * @see AP_GPS_NMEA.cpp for implementation
 */
#ifndef AP_GPS_NMEA_ENABLED
  #define AP_GPS_NMEA_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Unicore NMEA extensions
 * 
 * @details Supports Unicore-specific NMEA sentence extensions for their GPS receivers.
 *          Unicore (formerly Allystar) receivers provide additional proprietary NMEA
 *          sentences beyond standard NMEA 0183.
 *          
 *          Requires: AP_GPS_NMEA_ENABLED
 *          Flash size: Adds ~1KB to NMEA backend
 * 
 * @note Automatically enabled when NMEA support is enabled
 */
#ifndef AP_GPS_NMEA_UNICORE_ENABLED
  #define AP_GPS_NMEA_UNICORE_ENABLED AP_GPS_NMEA_ENABLED
#endif

/**
 * @brief Enable Novatel/Tersus/ComNav binary protocol support
 * 
 * @details Supports GPS receivers using Novatel binary protocol, also compatible with:
 *          - Novatel OEM series (OEM6, OEM7)
 *          - Tersus GNSS receivers
 *          - ComNav Technology receivers
 *          
 *          Protocol provides:
 *          - High-rate positioning updates (up to 20Hz)
 *          - RTK corrections support
 *          - Multiple constellation tracking
 *          - Detailed satellite and signal quality data
 *          - Dual antenna heading support
 *          
 *          Commonly used in professional surveying and precision agriculture.
 *          
 *          Flash size: Approximately 4KB
 * 
 * @note These receivers typically support multiple protocols; binary is most efficient
 * @see AP_GPS_NOVA.cpp for implementation
 */
#ifndef AP_GPS_NOVA_ENABLED
  #define AP_GPS_NOVA_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Septentrio Binary Format (SBF) protocol support
 * 
 * @details Supports Septentrio high-precision GNSS receivers using SBF binary protocol.
 *          Septentrio receivers are known for multi-frequency, multi-constellation tracking
 *          with advanced interference mitigation.
 *          
 *          Supported receivers:
 *          - AsteRx series (AsteRx-m2, AsteRx-U)
 *          - mosaic series (mosaic-X5, mosaic-H)
 *          - PolaRx series
 *          
 *          Features:
 *          - RTK with fast convergence and reacquisition
 *          - Advanced jamming and spoofing detection (AIM+ technology)
 *          - Multi-frequency GNSS (L1/L2/L5)
 *          - Dual antenna heading with high accuracy
 *          - Moving baseline RTK support
 *          - Raw observation data output
 *          
 *          Flash size: Approximately 6KB
 * 
 * @note Septentrio receivers are premium-grade, often used in demanding environments
 * @note SBF backend enables AP_GPS_GPS_RTK_SENDING for MAVLink RTK status
 * @see AP_GPS_SBF.cpp for implementation
 */
#ifndef AP_GPS_SBF_ENABLED
  #define AP_GPS_SBF_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Swift Binary Protocol version 1 support
 * 
 * @details Supports Swift Navigation Piksi GPS receivers using SBP v1 protocol.
 *          Swift Navigation Piksi was one of the first low-cost RTK GPS solutions.
 *          
 *          Supported devices:
 *          - Piksi Multi
 *          - Piksi V2 (legacy)
 *          
 *          Features:
 *          - RTK positioning
 *          - Raw observation data
 *          - Integrated radio for RTK corrections
 *          
 *          Flash size: Approximately 4KB
 * 
 * @note SBP v1 is legacy protocol; SBP v2 is preferred for newer devices
 * @see AP_GPS_SBP.cpp for implementation
 */
#ifndef AP_GPS_SBP_ENABLED
  #define AP_GPS_SBP_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Swift Binary Protocol version 2 support
 * 
 * @details Supports Swift Navigation receivers using SBP v2 protocol.
 *          SBP v2 is the current protocol for Swift Navigation products.
 *          
 *          Supported devices:
 *          - Piksi Multi (with updated firmware)
 *          - Duro (rugged enclosure)
 *          - Duro Inertial (GNSS/INS integration)
 *          
 *          Improvements over SBP v1:
 *          - Enhanced message set
 *          - Better error handling
 *          - Additional status information
 *          
 *          Flash size: Approximately 5KB
 * 
 * @note Preferred over SBP v1 for Swift Navigation devices
 * @see AP_GPS_SBP2.cpp for implementation
 */
#ifndef AP_GPS_SBP2_ENABLED
   #define AP_GPS_SBP2_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable SiRF binary protocol support
 * 
 * @details Supports legacy GPS receivers using SiRF binary protocol.
 *          SiRF was a popular GPS chipset manufacturer in early 2000s.
 *          
 *          SiRF GPS modules include:
 *          - SiRFstar III
 *          - SiRFstar IV
 *          - Various OEM modules using SiRF chipsets
 *          
 *          Flash size: Approximately 3KB
 * 
 * @note SiRF protocol is largely obsolete; most modern GPS use u-blox or other protocols
 * @note Consider disabling on flash-constrained boards if not using SiRF hardware
 * @see AP_GPS_SIRF.cpp for implementation
 */
#ifndef AP_GPS_SIRF_ENABLED
  #define AP_GPS_SIRF_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable u-blox UBX binary protocol support
 * 
 * @details Supports u-blox GPS receivers using UBX binary protocol.
 *          U-blox is the most common GPS receiver used with ArduPilot.
 *          
 *          Supported u-blox generations:
 *          - LEA-6 series (older, GPS-only)
 *          - NEO-M7 series (GPS+GLONASS)
 *          - NEO-M8 series (multi-constellation, most common)
 *          - NEO-M9 series (multi-band, improved urban performance)
 *          - ZED-F9P series (RTK capable, centimeter precision)
 *          - ZED-F9R series (dead reckoning integration)
 *          
 *          Features:
 *          - Auto-configuration of receiver settings
 *          - RTK support (F9P and newer)
 *          - Moving baseline for dual-GPS heading (F9P)
 *          - High update rates (up to 10Hz standard, 25Hz on some models)
 *          - AssistNow for faster satellite acquisition
 *          - Save configuration to GPS flash memory
 *          
 *          Common u-blox GPS modules:
 *          - Here2/Here3 (CubePilot)
 *          - Holybro M8N/M9N/F9P
 *          - mRo GPS modules
 *          - SparkFun GPS modules
 *          - Countless third-party modules
 *          
 *          Flash size: Approximately 10KB (largest GPS backend due to comprehensive features)
 * 
 * @note U-blox backend is the most widely used and best tested GPS driver
 * @note Recommended to keep enabled unless severely flash-constrained
 * @warning Disabling u-blox support eliminates compatibility with majority of GPS modules
 * @see AP_GPS_UBLOX.cpp for implementation
 */
#ifndef AP_GPS_UBLOX_ENABLED
  #define AP_GPS_UBLOX_ENABLED AP_GPS_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable RTCM3 correction data decoding and injection
 * 
 * @details Enables parsing and injection of RTCM3 RTK correction data to GPS receivers.
 *          RTCM3 (Radio Technical Commission for Maritime Services) is the standard
 *          protocol for transmitting RTK corrections from base station to rover.
 *          
 *          Use cases:
 *          - RTK base station corrections via telemetry radio
 *          - NTRIP (Network RTK) corrections via companion computer
 *          - Local base station corrections via MAVLink
 *          
 *          Supported RTCM3 message types:
 *          - 1005: Station coordinates
 *          - 1077, 1087, 1097, 1127: Multi-constellation MSM7 observations
 *          - 1230: GLONASS code-phase biases
 *          - 4072: Additional correction data
 *          
 *          RTCM injection path: GCS/Telemetry → MAVLink GPS_RTCM_DATA → AP_GPS → GPS receiver
 *          
 *          Flash size: Approximately 8KB for RTCM decoder
 *          
 *          Platform gating: Disabled on boards with < 1MB flash to save space
 * 
 * @note Requires RTK-capable GPS receiver (u-blox F9P, Septentrio, Trimble, etc.)
 * @note RTCM corrections must match GPS receiver constellation configuration
 * @see libraries/AP_GPS/RTCM3_Parser.cpp for RTCM3 decoder implementation
 */
#ifndef AP_GPS_RTCM_DECODE_ENABLED
  #define AP_GPS_RTCM_DECODE_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

// ========================================
// Platform-Specific Configuration
// ========================================

/**
 * @brief Default COM port for GPS connection
 * 
 * @details Specifies the default serial port number used for GPS communication.
 *          This is a platform-specific default that can be overridden by:
 *          - Board-specific hwdef configuration
 *          - GPS_TYPE parameter serial port assignment
 *          
 *          Typical values:
 *          - 1: First GPS serial port (SERIAL3 on many boards)
 *          - 2: Second GPS serial port (SERIAL4 on many boards)
 *          
 *          The actual UART hardware is mapped in board hwdef files.
 * 
 * @note This is an internal default; end users configure GPS port via parameters
 * @see AP_SerialManager for serial port management
 */
#ifndef HAL_GPS_COM_PORT_DEFAULT
#define HAL_GPS_COM_PORT_DEFAULT 1
#endif


// ========================================
// MAVLink GPS Message Sending Configuration
// ========================================
// Controls which GPS data messages are sent to ground control stations.
// Disabling unused messages saves bandwidth on telemetry links.

/**
 * @brief Enable sending MAVLink GPS_RAW_INT messages for primary GPS
 * 
 * @details Controls transmission of GPS_RAW_INT message (ID 24) containing:
 *          - Latitude, longitude, altitude
 *          - GPS fix type and satellites visible
 *          - HDOP, VDOP (dilution of precision)
 *          - Ground speed and course
 *          - Altitude above ellipsoid
 *          
 *          GPS_RAW_INT is the primary GPS status message sent to ground stations.
 *          Essential for displaying GPS health and position in GCS.
 *          
 *          Message rate: Typically 1-2 Hz
 *          Requirements: HAL_GCS_ENABLED (MAVLink support) and AP_GPS_ENABLED
 * 
 * @note Disabling prevents GCS from displaying GPS status
 * @see https://mavlink.io/en/messages/common.html#GPS_RAW_INT
 */
#ifndef AP_GPS_GPS_RAW_INT_SENDING_ENABLED
#define AP_GPS_GPS_RAW_INT_SENDING_ENABLED HAL_GCS_ENABLED && AP_GPS_ENABLED
#endif

/**
 * @brief Enable sending MAVLink GPS2_RAW messages for secondary GPS
 * 
 * @details Controls transmission of GPS2_RAW message (ID 124) for second GPS receiver.
 *          Contains same data as GPS_RAW_INT but for the secondary GPS instance.
 *          
 *          Used when GPS_MAX_RECEIVERS > 1 to monitor:
 *          - Redundant GPS health
 *          - GPS comparison for failure detection
 *          - Dual GPS performance
 *          
 *          Message rate: Typically 1 Hz
 *          Requirements: HAL_GCS_ENABLED, AP_GPS_ENABLED, and GPS_MAX_RECEIVERS > 1
 * 
 * @note Only relevant with multiple GPS receivers
 * @see https://mavlink.io/en/messages/common.html#GPS2_RAW
 */
#ifndef AP_GPS_GPS2_RAW_SENDING_ENABLED
#define AP_GPS_GPS2_RAW_SENDING_ENABLED HAL_GCS_ENABLED && AP_GPS_ENABLED && GPS_MAX_RECEIVERS > 1
#endif

/**
 * @brief Enable sending MAVLink GPS_RTK messages for primary GPS
 * 
 * @details Controls transmission of GPS_RTK message (ID 127) containing RTK status:
 *          - RTK baseline coordinates (base station relative position)
 *          - RTK accuracy metrics
 *          - Number of satellites used in RTK solution
 *          - Time since last baseline update
 *          - RTK fix type (none, float, fixed)
 *          
 *          Essential for monitoring RTK correction quality and troubleshooting.
 *          Only useful with RTK-capable GPS receivers.
 *          
 *          Message rate: Typically 1 Hz when RTK corrections active
 *          Requirements: HAL_GCS_ENABLED, AP_GPS_ENABLED, and RTK-capable backend
 *                       (currently SBF or ERB protocols)
 * 
 * @note Only enabled when SBF or ERB backends are compiled (RTK-capable protocols)
 * @see https://mavlink.io/en/messages/common.html#GPS_RTK
 */
#ifndef AP_GPS_GPS_RTK_SENDING_ENABLED
#define AP_GPS_GPS_RTK_SENDING_ENABLED HAL_GCS_ENABLED && AP_GPS_ENABLED && (AP_GPS_SBF_ENABLED || AP_GPS_ERB_ENABLED)
#endif

/**
 * @brief Enable sending MAVLink GPS2_RTK messages for secondary GPS
 * 
 * @details Controls transmission of GPS2_RTK message (ID 128) for second RTK GPS.
 *          Contains same RTK status data as GPS_RTK but for secondary receiver.
 *          
 *          Used with dual RTK GPS configurations:
 *          - Redundant RTK receivers
 *          - Moving baseline (base+rover on same vehicle for heading)
 *          
 *          Message rate: Typically 1 Hz when RTK corrections active
 *          Requirements: HAL_GCS_ENABLED, AP_GPS_ENABLED, GPS_MAX_RECEIVERS > 1,
 *                       and RTK-capable backend (SBF or ERB)
 * 
 * @note Only useful with multiple RTK-capable GPS receivers
 * @see https://mavlink.io/en/messages/common.html#GPS2_RTK
 */
#ifndef AP_GPS_GPS2_RTK_SENDING_ENABLED
#define AP_GPS_GPS2_RTK_SENDING_ENABLED HAL_GCS_ENABLED && AP_GPS_ENABLED && GPS_MAX_RECEIVERS > 1 && (AP_GPS_SBF_ENABLED || AP_GPS_ERB_ENABLED)
#endif
