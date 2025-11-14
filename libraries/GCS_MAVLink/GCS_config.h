/**
 * @file GCS_config.h
 * @brief Compile-time configuration for GCS/MAVLink subsystem feature flags
 * 
 * @details This file defines compile-time configuration options for the Ground Control Station
 *          (GCS) communication subsystem and MAVLink protocol support in ArduPilot. These feature
 *          flags allow conditional compilation of MAVLink messages, commands, and protocols to
 *          optimize flash memory usage on resource-constrained boards.
 *          
 *          Each feature flag can be overridden in board-specific hwdef files or project
 *          configuration to enable/disable specific MAVLink functionality. Many flags have
 *          dependencies on other subsystems (missions, relays, sensors) and may be automatically
 *          disabled if those dependencies are not available.
 *          
 *          Several legacy MAVLink messages and commands are being phased out and have documented
 *          deprecation timelines. These are marked with removal schedules across ArduPilot versions.
 * 
 * @note All feature flags default to enabled (1) unless constrained by board resources or dependencies
 * @note Board-specific configurations can override these defaults via hwdef files
 * 
 * @see GCS.h for the main GCS implementation
 * @see GCS_MAVLink.h for MAVLink message handling
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Relay/AP_Relay_config.h>
#include <AP_Mission/AP_Mission_config.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>
#include <AP_Arming/AP_Arming_config.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>

/**
 * @brief Master enable/disable switch for the entire GCS/MAVLink subsystem
 * 
 * @details Controls compilation of Ground Control Station communication and MAVLink protocol
 *          support. When disabled, removes all GCS telemetry, command handling, and MAVLink
 *          message processing from the build. This is the primary dependency for most other
 *          MAVLink-related feature flags.
 * 
 * @note Default: 1 (enabled)
 * @note Disabling this significantly reduces flash usage but removes all ground station communication
 * @note All AP_MAVLINK_* and HAL_MAVLINK_* flags typically depend on this being enabled
 */
#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

/**
 * @brief Enable MAVLink language bindings support
 * 
 * @details Controls compilation of MAVLink bindings for scripting languages and external interfaces.
 *          This enables integration with languages like Lua through the scripting subsystem.
 * 
 * @note Default: Follows HAL_GCS_ENABLED state
 * @note Automatically disabled if GCS subsystem is disabled
 */
#ifndef HAL_MAVLINK_BINDINGS_ENABLED
#define HAL_MAVLINK_BINDINGS_ENABLED HAL_GCS_ENABLED
#endif

/**
 * @brief Enable HIGH_LATENCY2 telemetry message support
 * 
 * @details Controls compilation of the HIGH_LATENCY2 MAVLink message for bandwidth-constrained
 *          communication links. This message provides compressed telemetry data suitable for
 *          satellite, LoRa, or other low-bandwidth connections where regular telemetry would
 *          consume too much bandwidth.
 * 
 * @note Default: 1 (enabled)
 * @note HIGH_LATENCY2 is the successor to the original HIGH_LATENCY message
 */
#ifndef HAL_HIGH_LATENCY2_ENABLED
#define HAL_HIGH_LATENCY2_ENABLED 1
#endif

/**
 * @brief Enable MISSION_SET_CURRENT message support (DEPRECATED - scheduled for removal)
 * 
 * @details Controls compilation of the legacy MISSION_SET_CURRENT MAVLink message handler.
 *          This message allows setting the current mission item index but has significant
 *          deficiencies compared to the MAV_CMD_DO_SET_CURRENT command, which provides
 *          proper acknowledgment and error handling.
 * 
 * @note Default: Follows AP_MISSION_ENABLED state
 * @note Automatically disabled if mission subsystem is disabled
 * 
 * @warning DEPRECATED - This feature is scheduled for removal in a future ArduPilot release
 * @warning Use MAV_CMD_DO_SET_CURRENT command instead (added to spec January 2019, ArduPilot 4.1.x)
 * @warning MAV_CMD_DO_SET_CURRENT provides ACK/NACK responses and better error handling
 */
#ifndef AP_MAVLINK_MISSION_SET_CURRENT_ENABLED
#define AP_MAVLINK_MISSION_SET_CURRENT_ENABLED AP_MISSION_ENABLED
#endif

/**
 * @brief Enable AUTOPILOT_VERSION_REQUEST message support (DEPRECATED - scheduled for removal)
 * 
 * @details Controls compilation of the legacy AUTOPILOT_VERSION_REQUEST MAVLink message handler.
 *          This message requests autopilot version information but lacks proper acknowledgment.
 *          The modern approach uses MAV_CMD_REQUEST_MESSAGE to request AUTOPILOT_VERSION,
 *          which provides ACK/NACK responses.
 * 
 * @note Default: 1 (enabled)
 * 
 * @warning DEPRECATED - This feature is scheduled for removal in a future ArduPilot release
 * @warning Use MAV_CMD_REQUEST_MESSAGE to request AUTOPILOT_VERSION instead
 * @warning The command-based approach provides proper ACK/NACK acknowledgment
 */
#ifndef AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED
#define AP_MAVLINK_AUTOPILOT_VERSION_REQUEST_ENABLED 1
#endif

/**
 * @brief Enable RC_CHANNELS_RAW message support
 * 
 * @details Controls compilation of the RC_CHANNELS_RAW MAVLink message for sending raw RC
 *          channel PWM values to ground control stations. This message provides unprocessed
 *          RC input values from the receiver before any mixing or processing.
 * 
 * @note Default: 1 (enabled)
 * @note This message is commonly used by ground stations to display RC input status
 * @note Channel values are in microseconds (typical range 1000-2000μs)
 */
#ifndef AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED
#define AP_MAVLINK_MSG_RC_CHANNELS_RAW_ENABLED 1
#endif

/**
 * @brief Enable MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES command (DEPRECATED - scheduled for removal)
 * 
 * @details Controls compilation of the legacy MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES command handler.
 *          This command requests the autopilot's capability flags but is redundant with the modern
 *          MAV_CMD_REQUEST_MESSAGE approach for requesting AUTOPILOT_VERSION.
 * 
 * @note Default: 1 (enabled)
 * 
 * @warning DEPRECATED - This feature is scheduled for removal in a future ArduPilot release
 * @warning Use MAV_CMD_REQUEST_MESSAGE to request AUTOPILOT_VERSION message instead
 * @warning The REQUEST_MESSAGE approach is the standardized method for all message requests
 */
#ifndef AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED
#define AP_MAVLINK_MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_ENABLED 1
#endif

/**
 * @brief Enable file-based MAVLink message interval configuration
 * 
 * @details Controls compilation of support for reading MAVLink message streaming intervals
 *          from configuration files on the filesystem. This allows dynamic configuration of
 *          telemetry rates without recompiling firmware.
 * 
 * @note Default: Enabled if filesystem support exists AND flash size > 1024KB
 * @note Requires FAT, LittleFS, or POSIX filesystem support
 * @note Automatically disabled on smaller boards due to flash constraints
 */
#ifndef HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
#define HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED ((AP_FILESYSTEM_FATFS_ENABLED || AP_FILESYSTEM_LITTLEFS_ENABLED || AP_FILESYSTEM_POSIX_ENABLED) && HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * @brief Enable RELAY_STATUS message support
 * 
 * @details Controls compilation of the RELAY_STATUS MAVLink message for reporting relay
 *          states to ground control stations. This message provides real-time status of
 *          all configured relay outputs.
 * 
 * @note Default: Enabled if both GCS and Relay subsystems are enabled
 * @note Automatically disabled if either GCS or Relay subsystem is disabled
 * @note Relays are typically used for camera triggers, parachute deployment, and other accessories
 */
#ifndef AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
#define AP_MAVLINK_MSG_RELAY_STATUS_ENABLED HAL_GCS_ENABLED && AP_RELAY_ENABLED
#endif

/**
 * @brief Enable developer failure injection commands
 * 
 * @details Controls compilation of MAVLink commands that allow intentional injection of
 *          failures for testing purposes. These developer-centric commands enable simulation
 *          of sensor failures, communication loss, and other fault conditions for robust
 *          testing of failsafe behaviors.
 * 
 * @note Default: 1 (enabled)
 * @note These commands should typically be disabled in production/safety-critical builds
 * @note Used primarily for development, testing, and validation of failsafe logic
 * 
 * @warning Production vehicles should consider disabling this feature for security
 */
#ifndef AP_MAVLINK_FAILURE_CREATION_ENABLED
#define AP_MAVLINK_FAILURE_CREATION_ENABLED 1
#endif

/**
 * @brief Enable legacy RALLY_POINT/RALLY_FETCH_POINT protocol (DEPRECATED - being removed)
 * 
 * @details Controls compilation of the legacy rally point upload/download protocol using
 *          RALLY_POINT and RALLY_FETCH_POINT MAVLink messages. Rally points provide
 *          alternative landing/RTL locations.
 * 
 * @note Default: 0 (disabled as of ArduPilot 4.7)
 * 
 * @warning DEPRECATED - Removal timeline:
 * @warning ArduPilot 4.6: Sends deprecation warnings for RALLY_POINT/RALLY_FETCH_POINT
 * @warning ArduPilot 4.7: Disabled by default (current state)
 * @warning ArduPilot 4.8: Code removed entirely
 * @warning Use mission protocol with MAV_CMD_DO_RALLY_LAND command for rally points instead
 */
#ifndef AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED
#define AP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED 0
#endif

/**
 * @brief Enable DEVICE_OP_READ and DEVICE_OP_WRITE message support
 * 
 * @details Controls compilation of DEVICE_OP_READ and DEVICE_OP_WRITE MAVLink messages
 *          for low-level device register access. These messages enable reading and writing
 *          device registers (I2C/SPI) remotely, primarily for debugging and diagnostics.
 * 
 * @note Default: Follows HAL_GCS_ENABLED state
 * @note Covers both read and write operations
 * @note Typically used for sensor diagnostics and hardware debugging
 */
#ifndef AP_MAVLINK_MSG_DEVICE_OP_ENABLED
#define AP_MAVLINK_MSG_DEVICE_OP_ENABLED HAL_GCS_ENABLED
#endif

/**
 * @brief Enable servo and relay control MAVLink commands
 * 
 * @details Controls compilation of MAVLink commands for controlling servos and relays,
 *          including DO_SET_SERVO, DO_SET_RELAY, DO_REPEAT_SERVO, and DO_REPEAT_RELAY.
 *          These commands allow direct control of servo outputs and relay switches.
 * 
 * @note Default: Enabled if both GCS and ServoRelayEvents subsystems are enabled
 * @note Automatically disabled if either GCS or ServoRelayEvents subsystem is disabled
 * @note Used for camera triggers, parachute deployment, landing gear, and custom accessories
 */
#ifndef AP_MAVLINK_SERVO_RELAY_ENABLED
#define AP_MAVLINK_SERVO_RELAY_ENABLED HAL_GCS_ENABLED && AP_SERVORELAYEVENTS_ENABLED
#endif

/**
 * @brief Enable SERIAL_CONTROL message support
 * 
 * @details Controls compilation of the SERIAL_CONTROL MAVLink message for tunneling
 *          serial data through MAVLink. This enables remote access to serial ports,
 *          allowing ground stations to communicate with peripherals connected to
 *          the autopilot's UART ports.
 * 
 * @note Default: Follows HAL_GCS_ENABLED state
 * @note Commonly used for configuring GPS receivers, companion computers, and other serial devices
 * @note Supports both blocking and non-blocking serial operations
 */
#ifndef AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED
#define AP_MAVLINK_MSG_SERIAL_CONTROL_ENABLED HAL_GCS_ENABLED
#endif

/**
 * @brief Enable UAVIONIX_ADSB_OUT_STATUS message support
 * 
 * @details Controls compilation of the UAVIONIX_ADSB_OUT_STATUS MAVLink message for
 *          reporting ADS-B transponder status. This message provides status information
 *          from uAvionix ADS-B OUT transponders, including transmission state and
 *          configuration status.
 * 
 * @note Default: Follows HAL_ADSB_ENABLED state
 * @note Automatically disabled if ADS-B subsystem is disabled
 * @note ADS-B OUT is required in many airspaces for collision avoidance
 */
#ifndef AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED
#define AP_MAVLINK_MSG_UAVIONIX_ADSB_OUT_STATUS_ENABLED HAL_ADSB_ENABLED
#endif

/**
 * @brief Enable MAVLink FTP (File Transfer Protocol) support
 * 
 * @details Controls compilation of MAVLink FTP protocol for file transfers between
 *          ground control station and autopilot. Enables uploading/downloading files,
 *          directory listing, and filesystem operations over MAVLink.
 * 
 * @note Default: Follows HAL_GCS_ENABLED state
 * @note Requires filesystem support on the board
 * @note Used for log download, parameter file upload, and terrain data transfer
 * @note Provides more efficient file transfer than SERIAL_CONTROL method
 */
#ifndef AP_MAVLINK_FTP_ENABLED
#define AP_MAVLINK_FTP_ENABLED HAL_GCS_ENABLED
#endif

/**
 * @brief Enable MISSION_REQUEST message support (DEPRECATED - being removed)
 * 
 * @details Controls compilation of the legacy MISSION_REQUEST MAVLink message handler.
 *          This message is used during mission download but is inefficient and wastes
 *          flash space. Ground control stations should use MISSION_REQUEST_INT instead,
 *          which provides better precision for waypoint coordinates.
 * 
 * @note Default: Follows AP_MISSION_ENABLED state
 * @note Automatically disabled if mission subsystem is disabled
 * @note MISSION_REQUEST was officially deprecated in June 2020
 * 
 * @warning DEPRECATED - Removal timeline:
 * @warning ArduPilot 4.4: Sends warnings to GCS if MISSION_REQUEST or MISSION_ITEM used (started Sep 2022)
 * @warning ArduPilot 4.8: Stops compiling MISSION_ITEM but still sends warnings
 * @warning ArduPilot 4.9: Removes code but sends "not supported" message
 * @warning ArduPilot 4.10: Stops sending the warning
 * @warning Use MISSION_REQUEST_INT instead for all mission downloads
 */
#ifndef AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED
#define AP_MAVLINK_MSG_MISSION_REQUEST_ENABLED AP_MISSION_ENABLED
#endif

/**
 * @brief Enable RANGEFINDER message sending
 * 
 * @details Controls compilation of the legacy RANGEFINDER MAVLink message for reporting
 *          distance sensor data. This message is a subset of the more comprehensive
 *          DISTANCE_SENSOR message which is also sent. The RANGEFINDER message provides
 *          minimal distance information and is considered redundant.
 * 
 * @note Default: Follows AP_RANGEFINDER_ENABLED state
 * @note Automatically disabled if rangefinder subsystem is disabled
 * @note DISTANCE_SENSOR message provides superset of this functionality
 * @note Client-side filtering can achieve same effect as Rover's send-minimum mode
 */
#ifndef AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED
#define AP_MAVLINK_MSG_RANGEFINDER_SENDING_ENABLED AP_RANGEFINDER_ENABLED
#endif

/**
 * @brief Enable COMMAND_LONG message support
 * 
 * @details Controls compilation of COMMAND_LONG MAVLink message handler for executing
 *          commands with floating-point parameters. All commands can alternatively be
 *          executed using COMMAND_INT (which supports both integer and float parameters),
 *          making COMMAND_LONG technically optional.
 * 
 * @note Default: 1 (enabled)
 * @note COMMAND_INT can execute all commands that COMMAND_LONG can, plus location-based commands
 * @note This option created for ArduPilot 4.5 (November 2023) to allow flash savings
 * @note Most ground control stations expect COMMAND_LONG support
 * 
 * @warning Disabling may cause compatibility issues with older ground control stations
 */
#ifndef AP_MAVLINK_COMMAND_LONG_ENABLED
#define AP_MAVLINK_COMMAND_LONG_ENABLED 1
#endif

/**
 * @brief Enable HIGHRES_IMU message support
 * 
 * @details Controls compilation of the HIGHRES_IMU MAVLink message for high-resolution
 *          inertial measurement unit data. This message provides detailed IMU sensor
 *          readings at higher precision than standard IMU messages, useful for advanced
 *          analysis and debugging.
 * 
 * @note Default: Enabled if flash size > 1024KB AND inertial sensor subsystem is enabled
 * @note Automatically disabled on smaller boards due to flash constraints
 * @note Provides 16-bit or 32-bit sensor values depending on sensor capabilities
 * @note Includes temperature, pressure, and magnetic field data in addition to IMU
 */
#ifndef AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
#define AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024) && AP_INERTIALSENSOR_ENABLED
#endif

/**
 * @brief Enable MAV_CMD_SET_HAGL command support
 * 
 * @details Controls compilation of the MAV_CMD_SET_HAGL (Height Above Ground Level) command
 *          handler. This command allows setting or overriding the vehicle's height above
 *          ground level estimate, useful for terrain following and obstacle avoidance.
 * 
 * @note Default: Enabled if flash size > 1024KB
 * @note Automatically disabled on smaller boards due to flash constraints
 * @note HAGL is critical for terrain following and precision landing operations
 */
#ifndef AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
#define AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * @brief Enable VIDEO_STREAM_INFORMATION message support
 * 
 * @details Controls compilation of the VIDEO_STREAM_INFORMATION MAVLink message for
 *          advertising available video streams from cameras attached to the vehicle.
 *          This message provides stream URLs, resolutions, and codec information to
 *          ground control stations.
 * 
 * @note Default: Follows HAL_GCS_ENABLED state
 * @note Used with onboard companion computers streaming video
 * @note Supports RTSP, HTTP, UDP, and other streaming protocols
 */
#ifndef AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
#define AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED HAL_GCS_ENABLED
#endif

/**
 * @brief Enable FLIGHT_INFORMATION message support
 * 
 * @details Controls compilation of the FLIGHT_INFORMATION MAVLink message for reporting
 *          flight session information including takeoff time, flight time, and arming time.
 *          This provides ground control stations with flight logging and timing data.
 * 
 * @note Default: Enabled if both GCS and Arming subsystems are enabled
 * @note Automatically disabled if either GCS or Arming subsystem is disabled
 * @note Useful for flight log analysis and flight time tracking
 */
#ifndef AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED
#define AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED HAL_GCS_ENABLED && AP_ARMING_ENABLED
#endif  // AP_MAVLINK_MSG_FLIGHT_INFORMATION_ENABLED

/**
 * @brief Enable SET_GPS_GLOBAL_ORIGIN message support (DEPRECATED - being removed)
 * 
 * @details Controls compilation of the legacy SET_GPS_GLOBAL_ORIGIN MAVLink message
 *          handler for setting the global coordinate system origin. This message has
 *          been replaced by the MAV_CMD_DO_SET_GLOBAL_ORIGIN command which provides
 *          proper acknowledgment.
 * 
 * @note Default: Enabled if both GCS and AHRS subsystems are enabled
 * @note Automatically disabled if either GCS or AHRS subsystem is disabled
 * @note Deprecated February 2025
 * 
 * @warning DEPRECATED - Removal timeline:
 * @warning ArduPilot 4.8: Starts warning if this message is used
 * @warning ArduPilot 4.9: Continues to warn if this message is used
 * @warning ArduPilot 4.10: Compiles support out by default
 * @warning ArduPilot 4.11: Removes the code entirely
 * @warning Use MAV_CMD_DO_SET_GLOBAL_ORIGIN command instead for proper ACK/NACK responses
 */
#ifndef AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
#define AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED (HAL_GCS_ENABLED && AP_AHRS_ENABLED)
#endif  // AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED
