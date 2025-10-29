/**
 * @file AP_OpticalFlow_config.h
 * @brief Configuration file for optical flow sensor backend compilation control
 * 
 * @details This configuration header controls which optical flow sensor backends
 *          are compiled into the ArduPilot firmware. Optical flow sensors provide
 *          ground-relative velocity measurements used for position estimation and
 *          stabilization, particularly useful in GPS-denied environments.
 * 
 *          The configuration system uses a hierarchical approach to enable/disable
 *          backend drivers, allowing fine-grained control over which sensor drivers
 *          are included in the compiled binary. This is critical for memory-constrained
 *          flight controllers where binary size must be minimized.
 * 
 * Configuration Hierarchy:
 * 
 * AP_OPTICALFLOW_ENABLED (Master Switch):
 *   - Controls the entire optical flow subsystem
 *   - Default: 1 (enabled)
 *   - When disabled (0), removes all optical flow functionality from the build
 *   - Disabling this provides the maximum binary size reduction
 * 
 * AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED (Backend Default):
 *   - Base value for individual backend enable flags
 *   - Default: Inherits from AP_OPTICALFLOW_ENABLED
 *   - Individual backends default to this value unless explicitly overridden
 *   - Allows bulk enabling/disabling of backends while retaining fine-grained control
 * 
 * Backend-Specific Configuration Flags:
 * 
 * AP_OPTICALFLOW_CXOF_ENABLED:
 *   - Cheerson CxOf optical flow sensor support
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
 *   - Protocol: I2C communication
 *   - Hardware: CxOf flow sensor module
 * 
 * AP_OPTICALFLOW_HEREFLOW_ENABLED:
 *   - HereFlow optical flow sensor support via DroneCAN
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
 *   - Protocol: DroneCAN (UAVCAN) over CAN bus
 *   - Hardware: Hex HereFlow sensor
 *   - Dependency: Requires DroneCAN/UAVCAN driver support
 * 
 * AP_OPTICALFLOW_MAV_ENABLED:
 *   - MAVLink-based optical flow sensor support
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
 *   - Protocol: MAVLink OPTICAL_FLOW message
 *   - Hardware: External sensors providing flow data via MAVLink
 *   - Dependency: Requires ground control station (GCS) MAVLink support
 *   - Use case: Companion computer or external processor providing flow data
 * 
 * HAL_MSP_OPTICALFLOW_ENABLED:
 *   - MSP (MultiWii Serial Protocol) optical flow support
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED
 *   - Protocol: MSP serial protocol
 *   - Hardware: MSP-compatible optical flow sensors and OSD systems
 *   - Dependency: Requires MSP protocol support
 * 
 * AP_OPTICALFLOW_ONBOARD_ENABLED:
 *   - Onboard/builtin optical flow sensor support
 *   - Default: 0 (disabled)
 *   - Hardware: Board-integrated optical flow sensors (rare)
 *   - Note: Disabled by default as few boards have integrated flow sensors
 * 
 * AP_OPTICALFLOW_PIXART_ENABLED:
 *   - PixArt PAW3902/PAA5100 optical flow sensor support
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
 *   - Protocol: SPI communication
 *   - Hardware: PixArt PAW3902, PAA5100 chipsets
 *   - Common sensors: Various commercial flow sensors using PixArt chips
 * 
 * AP_OPTICALFLOW_PX4FLOW_ENABLED:
 *   - PX4Flow optical flow sensor support
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
 *   - Protocol: I2C or MAVLink
 *   - Hardware: PX4Flow smart camera module
 *   - Note: One of the most common optical flow sensors in ArduPilot
 * 
 * AP_OPTICALFLOW_SITL_ENABLED:
 *   - Software-in-the-loop (SITL) optical flow simulation
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && AP_SIM_ENABLED
 *   - Protocol: Simulated sensor data
 *   - Hardware: None (simulation only)
 *   - Dependency: Requires SITL simulation framework
 *   - Use case: Testing and development without physical hardware
 * 
 * AP_OPTICALFLOW_UPFLOW_ENABLED:
 *   - Holybro UPFlow optical flow sensor support
 *   - Default: AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
 *   - Protocol: I2C communication
 *   - Hardware: Holybro UPFlow sensor
 * 
 * AP_OPTICALFLOW_CALIBRATOR_ENABLED:
 *   - Optical flow sensor calibration functionality
 *   - Default: AP_OPTICALFLOW_ENABLED
 *   - Purpose: Calibrate flow sensor scale factors and offsets
 *   - Note: Required for accurate flow measurements
 * 
 * External Dependencies:
 * 
 * HAL_ENABLE_DRONECAN_DRIVERS:
 *   - Controls DroneCAN/UAVCAN driver availability
 *   - Required for: AP_OPTICALFLOW_HEREFLOW_ENABLED
 *   - Defined in: AP_HAL board configuration files
 * 
 * HAL_GCS_ENABLED:
 *   - Controls ground control station MAVLink support
 *   - Required for: AP_OPTICALFLOW_MAV_ENABLED
 *   - Defined in: GCS_MAVLink/GCS_config.h
 * 
 * HAL_MSP_ENABLED:
 *   - Controls MultiWii Serial Protocol support
 *   - Required for: HAL_MSP_OPTICALFLOW_ENABLED
 *   - Defined in: AP_MSP/AP_MSP_config.h
 * 
 * AP_SIM_ENABLED:
 *   - Controls SITL simulation framework availability
 *   - Required for: AP_OPTICALFLOW_SITL_ENABLED
 *   - Defined in: SITL simulation configuration
 * 
 * Binary Size Optimization:
 * 
 * Disabling unused backends significantly reduces compiled binary size, which is
 * critical for memory-constrained flight controllers. Each backend driver includes:
 * - Driver initialization and detection code
 * - Communication protocol handlers (I2C, SPI, MAVLink, etc.)
 * - Sensor data processing algorithms
 * - Backend-specific calibration routines
 * 
 * To minimize binary size:
 * 1. Set AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED to 0
 * 2. Explicitly enable only the backends needed for your hardware
 * 3. For boards without optical flow sensors, set AP_OPTICALFLOW_ENABLED to 0
 * 
 * Example - Enable only PX4Flow support:
 *   #define AP_OPTICALFLOW_ENABLED 1
 *   #define AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED 0
 *   #define AP_OPTICALFLOW_PX4FLOW_ENABLED 1
 * 
 * @note Backend drivers are conditionally compiled based on these flags.
 *       Disabled backends are completely excluded from the binary.
 * 
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow.h for the main optical flow interface
 * @see libraries/AP_OpticalFlow/OpticalFlow_backend.h for backend base class
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_PX4Flow.h for PX4Flow implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_Pixart.h for PixArt implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_HereFlow.h for HereFlow implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_CXOF.h for CxOf implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_Upflow.h for UPFlow implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.h for MAVLink implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_MSP.h for MSP implementation
 * @see libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.h for SITL implementation
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/AP_MSP_config.h>
#include <GCS_MAVLink/GCS_config.h>

#ifndef AP_OPTICALFLOW_ENABLED
#define AP_OPTICALFLOW_ENABLED 1
#endif

#ifndef AP_OPTICALFLOW_CALIBRATOR_ENABLED
#define AP_OPTICALFLOW_CALIBRATOR_ENABLED AP_OPTICALFLOW_ENABLED
#endif

#ifndef AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
#define AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED AP_OPTICALFLOW_ENABLED
#endif

#ifndef AP_OPTICALFLOW_CXOF_ENABLED
#define AP_OPTICALFLOW_CXOF_ENABLED AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_OPTICALFLOW_HEREFLOW_ENABLED
#define AP_OPTICALFLOW_HEREFLOW_ENABLED (AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS)
#endif

#ifndef AP_OPTICALFLOW_MAV_ENABLED
#define AP_OPTICALFLOW_MAV_ENABLED (AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED)
#endif

#ifndef HAL_MSP_OPTICALFLOW_ENABLED
#define HAL_MSP_OPTICALFLOW_ENABLED (AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED)
#endif

#ifndef AP_OPTICALFLOW_ONBOARD_ENABLED
#define AP_OPTICALFLOW_ONBOARD_ENABLED 0
#endif

#ifndef AP_OPTICALFLOW_PIXART_ENABLED
#define AP_OPTICALFLOW_PIXART_ENABLED AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_OPTICALFLOW_PX4FLOW_ENABLED
#define AP_OPTICALFLOW_PX4FLOW_ENABLED AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_OPTICALFLOW_SITL_ENABLED
#define AP_OPTICALFLOW_SITL_ENABLED (AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && AP_SIM_ENABLED)
#endif

#ifndef AP_OPTICALFLOW_UPFLOW_ENABLED
#define AP_OPTICALFLOW_UPFLOW_ENABLED AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED
#endif
