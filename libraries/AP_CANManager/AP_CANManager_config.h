#pragma once

/**
 * @file AP_CANManager_config.h
 * @brief Compile-time feature configuration for CAN manager subsystem
 * 
 * @details Defines feature flags controlling SLCAN serial bridge and CAN frame logging.
 *          Configuration inherits from HAL board definitions (HAL_MAX_CAN_PROTOCOL_DRIVERS,
 *          HAL_NUM_CAN_IFACES) and AP_Logger availability. Boards without CAN hardware have
 *          HAL_MAX_CAN_PROTOCOL_DRIVERS=0, disabling entire CAN subsystem.
 *          
 *          Boards define HAL_MAX_CAN_PROTOCOL_DRIVERS in hwdef files. Set to 0 on boards
 *          without CAN hardware, typically 2-6 on boards with CAN transceivers.
 *          HAL_NUM_CAN_IFACES defines physical interface count (often 2 for redundancy).
 * 
 * @note Include this header first in AP_CANManager implementation files
 * @warning CAN subsystem adds ~50KB flash when enabled. Boards with limited flash (<512KB)
 *          may set HAL_MAX_CAN_PROTOCOL_DRIVERS=0 to disable entirely.
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Logger/AP_Logger_config.h>

/**
 * @def AP_CAN_SLCAN_ENABLED
 * @brief Enable SLCAN serial-to-CAN adapter functionality
 * 
 * @details When defined non-zero, compiles AP_SLCANIface class enabling serial port CAN
 *          bridging. Useful for debugging with CANable/SLCAN adapters. Defaults to
 *          HAL_MAX_CAN_PROTOCOL_DRIVERS (enabled if CAN hardware present). Set to 0 in
 *          hwdef to disable and save flash space.
 * 
 * @note Requires serial port configuration via CAN_SLCAN_CPORT parameter
 * @warning SLCAN parsing adds CPU overhead - disable if not needed for debugging
 * @see AP_SLCANIface.h
 */
#ifndef AP_CAN_SLCAN_ENABLED
#define AP_CAN_SLCAN_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS
#endif

/**
 * @def AP_CAN_LOGGING_ENABLED
 * @brief Enable CAN frame logging to dataflash/SD card
 * 
 * @details When defined non-zero, compiles frame logging callback registration and AP_Logger
 *          integration. Logs CAN frames as LOG_CAN (classic) and LOG_CAFD (CAN-FD) structures
 *          when CAN_Pn_OPTIONS LOG_ALL_FRAMES bit set. Defaults to HAL_MAX_CAN_PROTOCOL_DRIVERS
 *          && HAL_LOGGING_ENABLED (enabled if both CAN and logging available). Disable to save
 *          flash/CPU if logging not needed.
 * 
 * @note High-frequency logging can fill dataflash quickly - use selectively
 * @warning Logging all frames on busy bus (>1000 frames/sec) impacts scheduler performance
 * @see LogStructure.h
 * @see AP_CANManager::can_logging_callback()
 */
#ifndef AP_CAN_LOGGING_ENABLED
#define AP_CAN_LOGGING_ENABLED HAL_MAX_CAN_PROTOCOL_DRIVERS && HAL_LOGGING_ENABLED
#endif
