/**
 * @file AP_RCProtocol_config.h
 * @brief Compile-time configuration for RC (Radio Control) protocol support
 * 
 * @details This header defines feature gates that control which RC protocols
 *          are compiled into the firmware. Each protocol can be independently
 *          enabled or disabled to manage flash memory consumption and feature set.
 *          
 *          The configuration system uses a hierarchical approach:
 *          - AP_RCPROTOCOL_ENABLED: Master gate for entire RC protocol subsystem
 *          - AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED: Default for individual protocols
 *          - AP_RCPROTOCOL_XXX_ENABLED: Per-protocol feature gates
 *          
 *          Many protocols have HAL dependencies (DroneCAN, IOMCU) or are
 *          board-specific (SITL simulation protocols, Emlid RCIO). These
 *          dependencies are checked automatically in the configuration.
 *          
 * @warning Disabling unused protocols significantly reduces flash consumption.
 *          Each protocol backend adds 1-5 KB of code. On memory-constrained
 *          boards, disable protocols that are not needed for your RC receiver.
 *          
 * @note Supported RC protocols include:
 *       - SBUS (Futaba/FrSky standard)
 *       - CRSF (TBS Crossfire/ELRS)
 *       - DSM (Spektrum satellite)
 *       - IBUS (FlySky)
 *       - FPORT/FPORT2 (FrSky bidirectional)
 *       - PPM (traditional pulse position)
 *       - SRXL/SRXL2 (Spektrum remote receiver)
 *       - ST24/SUMD (Graupner)
 *       - GHST (ImmersionRC Ghost)
 *       - DroneCAN (CAN bus RC)
 *       - MAVLink Radio (GCS RC override)
 *       - SITL protocols (UDP, FDM, Joystick)
 *       
 * @see AP_RCProtocol.h for protocol implementation base class
 * @see RC_Channel.h for channel mapping and processing
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Frsky_Telem/AP_Frsky_config.h>
#include <GCS_MAVLink/GCS_config.h>
#include <AP_Radio/AP_Radio_config.h>

/**
 * @brief Master enable gate for entire RC protocol subsystem
 * 
 * @details Controls compilation of all RC protocol parsing code. When disabled,
 *          the vehicle will not be able to decode RC receiver input through
 *          serial protocols. Manual control will only be available through
 *          MAVLink RC override or other non-protocol sources.
 *          
 *          Disabling this on boards without RC receivers (e.g., companion
 *          computer controlled vehicles) can save significant flash space.
 *          
 * @note Default: Enabled (1) on all platforms
 * @warning Disabling this prevents all RC receiver input except MAVLink override
 */
#ifndef AP_RCPROTOCOL_ENABLED
#define AP_RCPROTOCOL_ENABLED 1
#endif

/**
 * @brief Default enable state for individual protocol backends
 * 
 * @details Provides a common default value for all protocol-specific enable
 *          flags. Individual protocols inherit this value unless explicitly
 *          overridden. This allows globally disabling all protocols by setting
 *          AP_RCPROTOCOL_ENABLED=0, or selectively enabling specific protocols
 *          by overriding individual AP_RCPROTOCOL_XXX_ENABLED flags.
 *          
 * @note Automatically inherits AP_RCPROTOCOL_ENABLED state
 * @see AP_RCPROTOCOL_ENABLED for master gate
 */
#ifndef AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#define AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED AP_RCPROTOCOL_ENABLED
#endif

/**
 * @brief Enable CRSF (Crossfire) RC protocol decoder
 * 
 * @details Enables support for TBS Crossfire and ExpressLRS (ELRS) receivers
 *          using the CRSF protocol. Provides up to 16 channels with telemetry
 *          support. Commonly used with long-range FPV systems.
 *          
 *          Protocol: 420000 baud serial, 8N1
 *          Channels: Up to 16 proportional channels
 *          Telemetry: Bidirectional (requires AP_CRSF_TELEM_ENABLED)
 *          
 * @note Flash cost: ~3-4 KB
 * @see AP_RCProtocol_CRSF.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_CRSF_ENABLED
#define AP_RCPROTOCOL_CRSF_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable DroneCAN RC protocol decoder
 * 
 * @details Enables RC input via DroneCAN (UAVCAN) nodes on the CAN bus.
 *          Allows using CAN-connected RC receivers or forwarding RC from
 *          other CAN nodes in the system.
 *          
 *          Protocol: DroneCAN equipment.actuator.ArrayCommand messages
 *          Channels: Up to 32 channels
 *          
 * @note Requires HAL_ENABLE_DRONECAN_DRIVERS to be enabled
 * @note Flash cost: Minimal (reuses existing DroneCAN infrastructure)
 * @warning Only available on boards with CAN bus support
 * @see AP_RCProtocol_DroneCAN.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_DRONECAN_ENABLED
#define AP_RCPROTOCOL_DRONECAN_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

/**
 * @brief Enable DSM (Spektrum satellite) RC protocol decoder
 * 
 * @details Enables support for Spektrum satellite receivers using DSM2/DSMX
 *          protocols. Supports both 10-bit and 11-bit resolution modes.
 *          
 *          Protocol: 115200 baud serial, 8N1
 *          Channels: Up to 12 channels (DSM2) or 20 channels (DSMX)
 *          Variants: DSM2, DSMX (auto-detected)
 *          
 * @note Flash cost: ~2-3 KB
 * @see AP_RCProtocol_DSM.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_DSM_ENABLED
#define AP_RCPROTOCOL_DSM_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Emlid RCIO protocol decoder
 * 
 * @details Enables RC input from the RCIO coprocessor on Emlid Navio and Navio2
 *          boards. This protocol is specific to Emlid hardware and handles RC
 *          input through the dedicated RCIO microcontroller.
 *          
 *          Protocol: Custom SPI protocol to RCIO processor
 *          Channels: Up to 8 channels
 *          
 * @note Default: Disabled (0) - board-specific protocol
 * @note Flash cost: ~1-2 KB
 * @warning Only enable on Emlid Navio/Navio2 boards with RCIO hardware
 * @see AP_RCProtocol_Emlid_RCIO.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_EMLID_RCIO_ENABLED
#define AP_RCPROTOCOL_EMLID_RCIO_ENABLED 0
#endif

/**
 * @brief Enable FPort (FrSky) RC protocol decoder
 * 
 * @details Enables support for FrSky FPort protocol, a bidirectional protocol
 *          combining RC control and telemetry in a single UART connection.
 *          Used with FrSky receivers like X8R, R9, and Archer series.
 *          
 *          Protocol: 115200 baud serial, 8N1, inverted
 *          Channels: Up to 16 channels
 *          Telemetry: Bidirectional (S.Port telemetry embedded)
 *          
 * @note Flash cost: ~2-3 KB
 * @see AP_RCProtocol_FPort.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_FPORT_ENABLED
#define AP_RCPROTOCOL_FPORT_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable FPort2 (FrSky bidirectional) RC protocol decoder
 * 
 * @details Enables support for FrSky FPort2 protocol, an enhanced version of
 *          FPort with improved telemetry integration. FPort2 requires S.Port
 *          telemetry to be enabled for full functionality.
 *          
 *          Protocol: 115200 baud serial, 8N1, inverted
 *          Channels: Up to 16 channels
 *          Telemetry: Enhanced bidirectional with S.Port
 *          
 * @note Requires AP_FRSKY_SPORT_TELEM_ENABLED to be enabled
 * @note Flash cost: ~2-3 KB
 * @see AP_RCProtocol_FPort2.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_FPORT2_ENABLED
#define AP_RCPROTOCOL_FPORT2_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_FRSKY_SPORT_TELEM_ENABLED
#endif

/**
 * @brief Enable IBUS (FlySky) RC protocol decoder
 * 
 * @details Enables support for FlySky IBUS protocol used by FlySky receivers
 *          like iA6B, iA10B, and compatible receivers. Provides digital serial
 *          protocol with checksum protection.
 *          
 *          Protocol: 115200 baud serial, 8N1
 *          Channels: Up to 14 channels
 *          Telemetry: Optional (requires separate implementation)
 *          
 * @note Flash cost: ~2 KB
 * @see AP_RCProtocol_IBUS.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_IBUS_ENABLED
#define AP_RCPROTOCOL_IBUS_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable IOMCU (Pixhawk IO processor) RC protocol decoder
 * 
 * @details Enables RC input from the IOMCU coprocessor on Pixhawk-style flight
 *          controllers. The IOMCU handles RC protocol decoding, PWM output,
 *          and safety switch on boards with a dedicated IO processor (STM32F1).
 *          
 *          Protocol: Custom protocol over dedicated UART to IOMCU
 *          Channels: Varies by RC protocol decoded by IOMCU (SBUS, PPM, etc.)
 *          
 * @note Requires HAL_WITH_IO_MCU to be enabled
 * @note Flash cost: Minimal (reuses existing IOMCU communication)
 * @warning Only available on Pixhawk boards with IO coprocessor
 * @see AP_IOMCU for IOMCU communication implementation
 */
#ifndef AP_RCPROTOCOL_IOMCU_ENABLED
#define AP_RCPROTOCOL_IOMCU_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && HAL_WITH_IO_MCU
#endif  // AP_RCPROTOCOL_IOMCU_ENABLED

/**
 * @brief Enable SFML Joystick RC input decoder (SITL only)
 * 
 * @details Enables RC input from USB joysticks and game controllers in SITL
 *          (Software In The Loop) simulation using the SFML library. Allows
 *          direct control of simulated vehicle with physical joystick.
 *          
 *          Protocol: USB HID joystick via SFML library
 *          Channels: Varies by joystick (typically 4-8 axes + buttons)
 *          
 * @note Automatically enabled when SFML_JOYSTICK is defined
 * @note Flash cost: N/A (SITL only)
 * @warning Only available in SITL simulation builds
 * @see AP_RCProtocol_Joystick_SFML.cpp for implementation
 */
#ifndef AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
#define AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED defined(SFML_JOYSTICK)
#endif

/**
 * @brief Enable PPM-SUM RC protocol decoder
 * 
 * @details Enables support for traditional PPM (Pulse Position Modulation)
 *          protocol, also known as PPM-SUM. This is one of the oldest RC
 *          protocols, using pulse timing on a single wire to encode multiple
 *          channels. Requires hardware timer input capture.
 *          
 *          Protocol: Pulse timing, 1-2ms pulses with sync gap
 *          Channels: Typically up to 8 channels
 *          Timing: 1000-2000μs per channel, >4ms sync gap
 *          
 * @note Flash cost: ~1-2 KB
 * @note Requires timer input capture capability on RC input pin
 * @see AP_RCProtocol_PPMSum.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_PPMSUM_ENABLED
#define AP_RCPROTOCOL_PPMSUM_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable custom AP_Radio RC input decoder
 * 
 * @details Enables RC input from the AP_Radio subsystem, which supports custom
 *          radio implementations like the Cypress CYRF6936-based radios. This
 *          protocol integrates with the AP_Radio driver infrastructure.
 *          
 *          Protocol: Custom radio-specific (varies by AP_Radio backend)
 *          Channels: Up to 16 channels
 *          
 * @note Requires AP_RADIO_ENABLED to be enabled
 * @note Flash cost: Minimal (reuses AP_Radio infrastructure)
 * @see AP_Radio.h for custom radio implementation
 */
#ifndef AP_RCPROTOCOL_RADIO_ENABLED
#define AP_RCPROTOCOL_RADIO_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_RADIO_ENABLED
#endif

/**
 * @brief Enable SBUS (Futaba) RC protocol decoder
 * 
 * @details Enables support for Futaba SBUS protocol, one of the most common
 *          digital RC protocols. Used by Futaba and FrSky receivers. SBUS
 *          uses inverted serial signal and provides 16 proportional channels
 *          plus 2 digital channels.
 *          
 *          Protocol: 100000 baud serial, 8E2, inverted
 *          Channels: 16 proportional + 2 digital
 *          Resolution: 11-bit (0-2047, center 1024)
 *          Frame rate: ~7ms (140Hz typical)
 *          
 * @note Flash cost: ~2 KB
 * @note SBUS signal is typically inverted and requires hardware or software inversion
 * @see AP_RCProtocol_SBUS.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_SBUS_ENABLED
#define AP_RCPROTOCOL_SBUS_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable SRXL (Spektrum Remote Receiver Link) RC protocol decoder
 * 
 * @details Enables support for Spektrum SRXL protocol, used for connecting
 *          Spektrum remote receivers and allowing multiple receivers to share
 *          diversity or range extension.
 *          
 *          Protocol: 115200 baud serial, 8N1
 *          Channels: Up to 12 channels
 *          
 * @note Flash cost: ~2 KB
 * @see AP_RCProtocol_SRXL.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_SRXL_ENABLED
#define AP_RCPROTOCOL_SRXL_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable SRXL2 (Spektrum Remote Receiver Link v2) RC protocol decoder
 * 
 * @details Enables support for Spektrum SRXL2 protocol, an enhanced version
 *          of SRXL with bidirectional telemetry support and improved features.
 *          Used with newer Spektrum receivers.
 *          
 *          Protocol: 115200 baud serial, 8N1
 *          Channels: Up to 16 channels
 *          Telemetry: Bidirectional
 *          
 * @note Flash cost: ~2-3 KB
 * @see AP_RCProtocol_SRXL2.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_SRXL2_ENABLED
#define AP_RCPROTOCOL_SRXL2_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable ST24 (Graupner) RC protocol decoder
 * 
 * @details Enables support for Graupner HoTT ST24 protocol used by Graupner
 *          GR-24 and similar receivers. Part of the HoTT telemetry ecosystem.
 *          
 *          Protocol: 115200 baud serial, 8N1
 *          Channels: Up to 12 channels
 *          
 * @note Flash cost: ~2 KB
 * @see AP_RCProtocol_ST24.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_ST24_ENABLED
#define AP_RCPROTOCOL_ST24_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable SUMD (Graupner) RC protocol decoder
 * 
 * @details Enables support for Graupner SUMD (Serial UniverSal MoDule) protocol
 *          used by Graupner receivers. Provides digital serial communication
 *          with checksum protection.
 *          
 *          Protocol: 115200 baud serial, 8N1
 *          Channels: Up to 16 channels
 *          
 * @note Flash cost: ~2 KB
 * @see AP_RCProtocol_SUMD.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_SUMD_ENABLED
#define AP_RCPROTOCOL_SUMD_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable SBUS Non-Inverted RC protocol decoder
 * 
 * @details Enables support for non-inverted SBUS protocol. Some receivers and
 *          flight controllers use non-inverted SBUS signals, which requires
 *          separate decoding from standard inverted SBUS.
 *          
 *          Protocol: 100000 baud serial, 8E2, non-inverted
 *          Channels: 16 proportional + 2 digital
 *          
 * @note Requires AP_RCPROTOCOL_SBUS_ENABLED to be enabled
 * @note Flash cost: Minimal (shares SBUS decoder)
 * @see AP_RCProtocol_SBUS.cpp for shared implementation
 */
#ifndef AP_RCPROTOCOL_SBUS_NI_ENABLED
#define AP_RCPROTOCOL_SBUS_NI_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_RCPROTOCOL_SBUS_ENABLED
#endif

/**
 * @brief Enable Fast SBUS RC protocol decoder
 * 
 * @details Enables support for Fast SBUS protocol, a higher-rate variant of
 *          SBUS with faster frame rates for reduced latency. Used by some
 *          FrSky receivers in high-speed mode.
 *          
 *          Protocol: 200000 baud serial, 8E2, inverted
 *          Channels: 16 proportional + 2 digital
 *          Frame rate: ~3.5ms (280Hz typical)
 *          
 * @note Requires AP_RCPROTOCOL_SBUS_ENABLED to be enabled
 * @note Flash cost: Minimal (shares SBUS decoder)
 * @see AP_RCProtocol_SBUS.cpp for shared implementation
 */
#ifndef AP_RCPROTOCOL_FASTSBUS_ENABLED
#define AP_RCPROTOCOL_FASTSBUS_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && AP_RCPROTOCOL_SBUS_ENABLED
#endif

/**
 * @brief Enable GHST (ImmersionRC Ghost) RC protocol decoder
 * 
 * @details Enables support for ImmersionRC Ghost protocol, a high-speed
 *          protocol designed for low-latency FPV racing with bidirectional
 *          telemetry support. Provides up to 12 channels with fast update rates.
 *          
 *          Protocol: 420000 baud serial, 8N1
 *          Channels: Up to 12 channels
 *          Telemetry: Bidirectional
 *          Frame rate: ~4ms (250Hz)
 *          
 * @note Flash cost: ~3 KB
 * @see AP_RCProtocol_GHST.cpp for protocol implementation
 */
#ifndef AP_RCPROTOCOL_GHST_ENABLED
#define AP_RCPROTOCOL_GHST_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable MAVLink RC override protocol decoder
 * 
 * @details Enables RC input via MAVLink RC_CHANNELS_OVERRIDE messages from
 *          ground control stations. Allows direct control from GCS without
 *          physical RC transmitter. Commonly used for companion computer
 *          control and autonomous missions with manual override capability.
 *          
 *          Protocol: MAVLink RC_CHANNELS_OVERRIDE messages
 *          Channels: Up to 18 channels (MAVLink v2)
 *          
 * @note Requires HAL_PROGRAM_SIZE_LIMIT_KB > 1024 (excluded from small boards)
 * @note Requires HAL_GCS_ENABLED for MAVLink communication
 * @note Flash cost: ~1-2 KB
 * @warning Security consideration: Ensure MAVLink stream is authenticated
 *          to prevent unauthorized control
 * @see GCS_MAVLink.cpp for MAVLink message handling
 */
#ifndef AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED
#define AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024 && HAL_GCS_ENABLED
#endif

/**
 * @brief Enable UDP RC input decoder (SITL only)
 * 
 * @details Enables RC input via UDP packets in SITL (Software In The Loop)
 *          simulation. Allows external programs to inject RC commands over
 *          network for testing and simulation scenarios.
 *          
 *          Protocol: Custom UDP packets on configurable port
 *          Channels: Up to 16 channels
 *          
 * @note Only available when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * @note Flash cost: N/A (SITL only)
 * @warning Only for simulation - not available on hardware
 * @see AP_RCProtocol_UDP.cpp for implementation
 */
#ifndef AP_RCPROTOCOL_UDP_ENABLED
#define AP_RCPROTOCOL_UDP_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

/**
 * @brief Enable FDM (Flight Dynamics Model) RC input decoder (SITL only)
 * 
 * @details Enables RC input from the Flight Dynamics Model in SITL simulation.
 *          The FDM provides simulated RC inputs synchronized with the physics
 *          simulation, typically from JSBSim, Gazebo, or other simulators.
 *          
 *          Protocol: Shared memory or UDP from physics simulator
 *          Channels: Up to 16 channels
 *          
 * @note Only available when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * @note Flash cost: N/A (SITL only)
 * @warning Only for simulation - not available on hardware
 * @see SITL/SIM_Multicopter.cpp for FDM integration
 */
#ifndef AP_RCPROTOCOL_FDM_ENABLED
#define AP_RCPROTOCOL_FDM_ENABLED AP_RCPROTOCOL_BACKEND_DEFAULT_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
