/**
 * @file AP_RangeFinder_config.h
 * @brief Compile-time configuration for AP_RangeFinder library and backend drivers
 * 
 * @details This file contains compile-time feature flags that control which rangefinder
 *          backend drivers are included in the build. These flags enable the build system
 *          to exclude unused rangefinder drivers, significantly reducing binary size on
 *          memory-constrained platforms.
 *          
 *          The configuration uses a hierarchical system:
 *          - AP_RANGEFINDER_ENABLED: Master enable flag for entire rangefinder subsystem
 *          - AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED: Default for most backends
 *          - AP_RANGEFINDER_BACKEND_CAN_ENABLED: Default for CAN-based backends
 *          - Individual AP_RANGEFINDER_*_ENABLED: Per-backend enable flags
 *          
 *          Backend drivers support various interfaces including:
 *          - Serial (UART): TFMini, LeddarOne, MaxBotix, USD1, etc.
 *          - I2C: LightWare I2C, VL53L0X, VL53L1X, MaxSonar I2C, etc.
 *          - CAN: DroneCAN, Benewake CAN, NRA24 CAN, USD1 CAN, etc.
 *          - PWM/GPIO: HC-SR04, PWM-based rangefinders
 *          - Analog: Voltage-based distance sensors
 *          - Protocol: MAVLink DISTANCE_SENSOR messages, Lua scripting interface
 * 
 * @note Hardware definition files (hwdef) can override these defaults to customize
 *       which backends are included for specific boards. This is the primary mechanism
 *       for managing flash memory usage on resource-constrained platforms.
 * 
 * @note Some backends have additional dependencies:
 *       - CAN backends require HAL_MAX_CAN_PROTOCOL_DRIVERS > 0
 *       - DroneCAN backend requires HAL_ENABLE_DRONECAN_DRIVERS
 *       - Lua backend requires AP_SCRIPTING_ENABLED
 *       - MAVLink backend requires HAL_GCS_ENABLED
 *       - MSP backend requires HAL_MSP_ENABLED
 *       - Some backends require HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 * 
 * @warning Disabling a backend that is referenced by vehicle code or mission plans
 *          will result in compilation errors or runtime failures. Ensure no active
 *          configuration depends on a disabled backend before removing it from the build.
 * 
 * @see libraries/AP_RangeFinder/AP_RangeFinder.h for the main rangefinder interface
 * @see libraries/AP_HAL_ChibiOS/hwdef/README.md for hardware definition file syntax
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_MSP/AP_MSP_config.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_CANManager/AP_CANManager_config.h>
#include <AP_MSP/AP_MSP_config.h>
#include <GCS_MAVLink/GCS_config.h>

/**
 * @brief Master enable flag for entire AP_RangeFinder subsystem
 * 
 * @details When set to 0, disables the entire rangefinder subsystem including all
 *          backend drivers. This provides maximum flash memory savings when distance
 *          sensing is not required.
 *          
 *          Default: 1 (enabled)
 * 
 * @note Disabling this will cause compilation errors if vehicle code attempts to
 *       use rangefinder functionality. Ensure all rangefinder references are removed
 *       or conditionally compiled before disabling.
 */
#ifndef AP_RANGEFINDER_ENABLED
#define AP_RANGEFINDER_ENABLED 1
#endif

/**
 * @brief Default enable flag for most rangefinder backends
 * 
 * @details This flag serves as the default value for most individual backend enable
 *          flags. It follows AP_RANGEFINDER_ENABLED, meaning when the rangefinder
 *          subsystem is disabled, all backends default to disabled as well.
 *          
 *          Individual backends can override this default by specifying their own
 *          enable flag in the hardware definition file.
 *          
 *          Default: AP_RANGEFINDER_ENABLED (typically 1)
 * 
 * @note Platform-specific backends (BBB_PRU, Bebop, PWM, SIM) do not use this
 *       default and have platform-conditional defaults instead.
 */
#ifndef AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#define AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED AP_RANGEFINDER_ENABLED
#endif

/**
 * @brief Default enable flag for CAN bus-based rangefinder backends
 * 
 * @details Enables support for rangefinders connected via CAN bus. This flag is
 *          conditional on HAL_MAX_CAN_PROTOCOL_DRIVERS being non-zero, ensuring
 *          CAN backends are only included when CAN hardware support is available.
 *          
 *          CAN-based backends include:
 *          - Benewake CAN protocol rangefinders
 *          - NRA24 CAN radar sensors
 *          - TOFSenseP CAN LiDAR
 *          - USD1 CAN distance sensors
 *          - HexSoon radar sensors
 *          
 *          Default: AP_RANGEFINDER_ENABLED && HAL_MAX_CAN_PROTOCOL_DRIVERS
 * 
 * @note This is separate from AP_RANGEFINDER_DRONECAN_ENABLED which uses the
 *       DroneCAN/UAVCAN protocol stack rather than custom CAN protocols.
 */
#ifndef AP_RANGEFINDER_BACKEND_CAN_ENABLED
#define AP_RANGEFINDER_BACKEND_CAN_ENABLED AP_RANGEFINDER_ENABLED && HAL_MAX_CAN_PROTOCOL_DRIVERS
#endif

/**
 * @brief Enable analog voltage-based rangefinder backend
 * 
 * @details Supports distance sensors that output analog voltage proportional to
 *          measured distance. Uses ADC (Analog-to-Digital Converter) input pins
 *          to read voltage and converts to distance based on configured scaling.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_ANALOG_ENABLED
#define AP_RANGEFINDER_ANALOG_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable BeagleBone Black PRU (Programmable Real-time Unit) rangefinder backend
 * 
 * @details Specialized backend for BeagleBone Black that uses the PRU subsystem for
 *          high-precision timing of ultrasonic rangefinder pulse measurements. This
 *          is platform-specific to BeagleBone Black Linux boards.
 *          
 *          Default: 0 (disabled by default, enable only on BBB platforms)
 * 
 * @note This backend is Linux-specific and requires BeagleBone Black hardware with
 *       PRU support. It will not compile on other platforms.
 */
#ifndef AP_RANGEFINDER_BBB_PRU_ENABLED
#define AP_RANGEFINDER_BBB_PRU_ENABLED 0
#endif

/**
 * @brief Base enable flag for Benewake rangefinder family
 * 
 * @details Master flag for the Benewake/Seeedstudio family of LiDAR rangefinders.
 *          Individual Benewake models (TF02, TF03, TFMini) check this flag in
 *          addition to their own enable flags.
 *          
 *          Benewake produces compact single-point LiDAR rangefinders using serial
 *          or CAN communication interfaces.
 *          
 *          Default: AP_RANGEFINDER_ENABLED
 */
#ifndef AP_RANGEFINDER_BENEWAKE_ENABLED
#define AP_RANGEFINDER_BENEWAKE_ENABLED AP_RANGEFINDER_ENABLED
#endif

/**
 * @brief Enable Benewake CAN protocol rangefinder backend
 * 
 * @details Supports Benewake rangefinders using proprietary CAN bus protocol.
 *          Requires CAN hardware support (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0).
 *          
 *          Default: HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 * 
 * @note This is distinct from Benewake serial protocol models. Check sensor
 *       specifications to determine which protocol your model uses.
 */
#ifndef AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
#define AP_RANGEFINDER_BENEWAKE_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable Benewake TF02 serial LiDAR rangefinder backend
 * 
 * @details TF02 is a serial (UART) LiDAR with 22m range at 100Hz update rate.
 *          Communicates using Benewake proprietary serial protocol.
 *          
 *          Default: AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TF02_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable Benewake TF03 serial LiDAR rangefinder backend
 * 
 * @details TF03 is a long-range serial (UART) LiDAR with 180m range.
 *          Suitable for fixed-wing and long-range applications.
 *          
 *          Default: AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_BENEWAKE_TF03_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TF03_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable Benewake TFMini serial LiDAR rangefinder backend
 * 
 * @details TFMini is a compact, low-cost serial (UART) LiDAR with 12m range.
 *          Popular for multicopter and rover applications due to small size.
 *          
 *          Default: AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TFMINI_ENABLED (AP_RANGEFINDER_BENEWAKE_ENABLED && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable Benewake TFMini Plus serial LiDAR rangefinder backend
 * 
 * @details TFMini Plus is an improved version of TFMini with 12m range and
 *          enhanced performance in bright sunlight and dark surfaces.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED
#define AP_RANGEFINDER_BENEWAKE_TFMINIPLUS_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Blue Robotics Ping1D underwater sonar rangefinder backend
 * 
 * @details Ping1D is an underwater sonar specifically designed for ROV/AUV applications.
 *          Measures distance underwater using acoustic ranging with 30m maximum range.
 *          Communicates via serial (UART) protocol.
 *          
 *          Primarily used with ArduSub for depth hold and terrain following underwater.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_BLPING_ENABLED
#define AP_RANGEFINDER_BLPING_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Parrot Bebop rangefinder backend
 * 
 * @details Platform-specific backend for Parrot Bebop drone's integrated ultrasonic
 *          rangefinder. Only applicable when running on Bebop hardware.
 *          
 *          Default: 0 (disabled by default, enable only on Bebop platform)
 * 
 * @note This is Bebop hardware-specific and will not work on other platforms.
 */
#ifndef AP_RANGEFINDER_BEBOP_ENABLED
#define AP_RANGEFINDER_BEBOP_ENABLED 0
#endif

/**
 * @brief Enable DroneCAN/UAVCAN rangefinder backend
 * 
 * @details Supports rangefinders using DroneCAN (formerly UAVCAN) protocol over CAN bus.
 *          DroneCAN provides standardized sensor interface with automatic node discovery
 *          and configuration.
 *          
 *          Requires HAL_ENABLE_DRONECAN_DRIVERS to be enabled for DroneCAN stack support.
 *          
 *          Default: HAL_ENABLE_DRONECAN_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 * 
 * @note This uses the standard DroneCAN protocol, distinct from manufacturer-specific
 *       CAN protocols like Benewake CAN or NRA24 CAN.
 */
#ifndef AP_RANGEFINDER_DRONECAN_ENABLED
#define AP_RANGEFINDER_DRONECAN_ENABLED (HAL_ENABLE_DRONECAN_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable GY-US42v2 I2C ultrasonic rangefinder backend
 * 
 * @details GY-US42v2 is an I2C ultrasonic rangefinder with 6m maximum range.
 *          Uses MaxSonar-compatible I2C protocol.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_GYUS42V2_ENABLED
#define AP_RANGEFINDER_GYUS42V2_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable HC-SR04 ultrasonic rangefinder backend
 * 
 * @details HC-SR04 is a low-cost ultrasonic rangefinder using trigger/echo GPIO pins.
 *          Requires precise timing of echo pulse width to measure distance (up to 4m).
 *          
 *          Uses GPIO pins for trigger output and echo pulse measurement via PWM input.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_HC_SR04_ENABLED
#define AP_RANGEFINDER_HC_SR04_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Lanbao PSK-CM8JL65-CC5 serial rangefinder backend
 * 
 * @details Lanbao PSK-CM8JL65-CC5 is a serial (UART) laser rangefinder with
 *          8m maximum range. Used in industrial and robotics applications.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_LANBAO_ENABLED
#define AP_RANGEFINDER_LANBAO_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable LeddarOne serial LiDAR rangefinder backend
 * 
 * @details LeddarOne is a serial (UART) LiDAR sensor with 40m range and high
 *          update rate. Provides reliable measurements in various lighting and
 *          weather conditions.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_LEDDARONE_ENABLED
#define AP_RANGEFINDER_LEDDARONE_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable LeddarVu8 Modbus serial rangefinder backend
 * 
 * @details LeddarVu8 is a multi-segment LiDAR using Modbus RTU serial protocol.
 *          Provides 8 independent measurement segments for wide field-of-view sensing.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_LEDDARVU8_ENABLED
#define AP_RANGEFINDER_LEDDARVU8_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable LightWare I2C LiDAR rangefinder backend
 * 
 * @details Supports LightWare SF series LiDAR rangefinders using I2C interface.
 *          LightWare sensors provide high-accuracy laser ranging up to 50m or more
 *          depending on model.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_LWI2C_ENABLED
#define AP_RANGEFINDER_LWI2C_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable LightWare serial LiDAR rangefinder backend
 * 
 * @details Supports LightWare SF series LiDAR rangefinders using serial (UART) interface.
 *          Serial interface typically provides faster update rates than I2C.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
#define AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Lua scripting interface rangefinder backend
 * 
 * @details Allows Lua scripts to provide rangefinder data, enabling custom
 *          rangefinder implementations or sensor fusion without recompiling firmware.
 *          
 *          Requires AP_SCRIPTING_ENABLED for Lua scripting support.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_SCRIPTING_ENABLED
 * 
 * @note Scripts must call appropriate rangefinder Lua bindings to inject distance data.
 */
#ifndef AP_RANGEFINDER_LUA_ENABLED
#define AP_RANGEFINDER_LUA_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_SCRIPTING_ENABLED
#endif

/**
 * @brief Enable MAVLink DISTANCE_SENSOR message rangefinder backend
 * 
 * @details Receives distance measurements via MAVLink DISTANCE_SENSOR messages from
 *          companion computers or external sensors. Enables integration of custom
 *          sensors or sensor fusion algorithms running on companion computers.
 *          
 *          Requires HAL_GCS_ENABLED for MAVLink communication support.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
 * 
 * @note Companion computer must send properly formatted DISTANCE_SENSOR messages
 *       at appropriate rate for reliable operation.
 */
#ifndef AP_RANGEFINDER_MAVLINK_ENABLED
#define AP_RANGEFINDER_MAVLINK_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED
#endif

/**
 * @brief Enable MaxBotix serial ultrasonic rangefinder backend
 * 
 * @details Supports MaxBotix serial output ultrasonic rangefinders (MB1xxx series).
 *          MaxBotix sensors use simple ASCII serial protocol for distance reporting.
 *          Typical range 0-7m depending on model.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED
#define AP_RANGEFINDER_MAXBOTIX_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable MaxSonar I2C XL ultrasonic rangefinder backend
 * 
 * @details Supports MaxBotix I2C-XL series ultrasonic rangefinders using I2C interface.
 *          I2C models provide long-range sensing (up to 10m) with weather resistance.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_MAXSONARI2CXL_ENABLED
#define AP_RANGEFINDER_MAXSONARI2CXL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable MSP (MultiWii Serial Protocol) rangefinder backend
 * 
 * @details Receives rangefinder data via MSP protocol from flight controllers or
 *          sensors supporting MSP. Enables integration with MSP-based accessories.
 *          
 *          Requires HAL_MSP_ENABLED for MSP protocol support.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED
 */
#ifndef HAL_MSP_RANGEFINDER_ENABLED
#define HAL_MSP_RANGEFINDER_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED
#endif

/**
 * @brief Enable NMEA protocol rangefinder backend
 * 
 * @details Supports depth sensors and sonar that output NMEA 0183 sentences (DBT - Depth
 *          Below Transducer). Commonly used with marine depth sounders and echosounders.
 *          
 *          Primarily used with ArduSub for underwater depth sensing.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_NMEA_ENABLED
#define AP_RANGEFINDER_NMEA_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Nooploop TOF (Time-of-Flight) rangefinder backend
 * 
 * @details Nooploop TOFSense-M and TOFSense-F series optical rangefinders using
 *          serial protocol. Provides millimeter accuracy with 12m+ range.
 *          
 *          Only included on boards with >1MB flash due to protocol complexity.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 */
#ifndef AP_RANGEFINDER_NOOPLOOP_ENABLED
#define AP_RANGEFINDER_NOOPLOOP_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable NRA24 CAN radar rangefinder backend
 * 
 * @details Supports NanoRadar NRA24 GHz mmWave radar sensors using CAN bus protocol.
 *          Provides long-range detection (80m+) with velocity measurement capability.
 *          
 *          Requires HAL_MAX_CAN_PROTOCOL_DRIVERS > 0 for CAN support.
 *          
 *          Default: HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 * 
 * @note Also used by HexSoon radar - see AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
 */
#ifndef AP_RANGEFINDER_NRA24_CAN_ENABLED
#define AP_RANGEFINDER_NRA24_CAN_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable PWM (pulse width) rangefinder backend
 * 
 * @details Supports rangefinders that output distance as PWM pulse width on a GPIO pin.
 *          Requires hardware timer capture capability for precise pulse width measurement.
 *          
 *          Only available on ChibiOS/ARM platforms with hardware PWM capture support.
 *          
 *          Default: CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 * 
 * @note Platform-specific: ChibiOS only
 */
#ifndef AP_RANGEFINDER_PWM_ENABLED
#define AP_RANGEFINDER_PWM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable PulsedLight LiDAR-Lite I2C rangefinder backend
 * 
 * @details Supports PulsedLight (Garmin) LiDAR-Lite series using I2C interface.
 *          LiDAR-Lite provides 40m range with high accuracy, widely used in commercial
 *          drones for precision altitude hold.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED
#define AP_RANGEFINDER_PULSEDLIGHTLRF_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable RDS02UF ultrasonic rangefinder backend
 * 
 * @details RDS02UF is a serial ultrasonic rangefinder with 3m range. Used in
 *          indoor positioning and proximity detection applications.
 *          
 *          Only included on boards with >1MB flash.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 */
#ifndef AP_RANGEFINDER_RDS02UF_ENABLED
#define AP_RANGEFINDER_RDS02UF_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable SITL (Software In The Loop) simulation rangefinder backend
 * 
 * @details Simulated rangefinder for SITL testing. Generates realistic distance
 *          measurements based on simulated terrain and vehicle position.
 *          
 *          Only available when building for SITL simulation platform.
 *          
 *          Default: CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 * 
 * @note Simulation-only: Not included in hardware builds
 */
#ifndef AP_RANGEFINDER_SIM_ENABLED
#define AP_RANGEFINDER_SIM_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Enable MSP (MultiWii Serial Protocol) rangefinder backend (duplicate definition)
 * 
 * @details This is a duplicate definition guard for HAL_MSP_RANGEFINDER_ENABLED.
 *          Ensures consistent enablement regardless of include order.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED
 * 
 * @note This duplicate definition is intentional for header include order safety
 */
#ifndef HAL_MSP_RANGEFINDER_ENABLED
#define HAL_MSP_RANGEFINDER_ENABLED (AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_MSP_ENABLED)
#endif

/**
 * @brief Enable TeraRanger serial LiDAR rangefinder backend
 * 
 * @details Supports TeraRanger Evo series and One LiDAR sensors using serial protocol.
 *          TeraRanger sensors provide fast update rates (up to 900Hz) with ranges
 *          from 2m to 60m depending on model.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
#define AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable TOFSenseP CAN LiDAR rangefinder backend
 * 
 * @details TOFSenseP series rangefinders with CAN bus interface. Provides industrial-grade
 *          TOF (Time-of-Flight) sensing with 30m+ range.
 *          
 *          Requires both CAN support and AP_RANGEFINDER_BACKEND_CAN_ENABLED.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_RANGEFINDER_BACKEND_CAN_ENABLED
 */
#ifndef AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED
#define AP_RANGEFINDER_TOFSENSEP_CAN_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_RANGEFINDER_BACKEND_CAN_ENABLED
#endif

/**
 * @brief Enable TOFSenseF I2C LiDAR rangefinder backend
 * 
 * @details TOFSenseF series compact rangefinders with I2C interface. Suitable for
 *          short to medium range applications (up to 12m).
 *          
 *          Only included on boards with >1MB flash.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 */
#ifndef AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
#define AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable TeraRanger I2C rangefinder backend
 * 
 * @details TeraRanger sensors using I2C interface. Alternative to serial interface
 *          for TeraRanger One and some Evo models.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_TRI2C_ENABLED
#define AP_RANGEFINDER_TRI2C_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable USD1 CAN rangefinder backend
 * 
 * @details Ultra-Small Digital (USD1) rangefinder with CAN bus interface.
 *          Compact sensor suitable for space-constrained installations.
 *          
 *          Requires both CAN support and AP_RANGEFINDER_BACKEND_CAN_ENABLED.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_RANGEFINDER_BACKEND_CAN_ENABLED
 */
#ifndef AP_RANGEFINDER_USD1_CAN_ENABLED
#define AP_RANGEFINDER_USD1_CAN_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && AP_RANGEFINDER_BACKEND_CAN_ENABLED
#endif

/**
 * @brief Enable USD1 serial rangefinder backend
 * 
 * @details Ultra-Small Digital (USD1) rangefinder with serial (UART) interface.
 *          Alternative to CAN interface version.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_USD1_SERIAL_ENABLED
#define AP_RANGEFINDER_USD1_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable STMicroelectronics VL53L0X I2C TOF rangefinder backend
 * 
 * @details VL53L0X is a popular low-cost I2C Time-of-Flight sensor with 2m range.
 *          Compact size makes it suitable for indoor and short-range applications.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_VL53L0X_ENABLED
#define AP_RANGEFINDER_VL53L0X_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable STMicroelectronics VL53L1X I2C TOF rangefinder backend
 * 
 * @details VL53L1X is an improved version of VL53L0X with 4m range and better
 *          performance in ambient light. Uses same I2C interface.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_VL53L1X_ENABLED
#define AP_RANGEFINDER_VL53L1X_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Wasp serial LiDAR rangefinder backend
 * 
 * @details Wasp LiDAR sensors using serial protocol. Designed for long-range
 *          sensing in industrial and surveying applications.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 */
#ifndef AP_RANGEFINDER_WASP_ENABLED
#define AP_RANGEFINDER_WASP_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Enable JRE serial rangefinder backend
 * 
 * @details JRE (Jiuxing Robotics) serial rangefinders for industrial applications.
 *          
 *          Only included on boards with >1MB flash.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 */
#ifndef AP_RANGEFINDER_JRE_SERIAL_ENABLED
#define AP_RANGEFINDER_JRE_SERIAL_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable Ainstein LR-D1 radar rangefinder backend
 * 
 * @details Ainstein LR-D1 is a 60/77GHz radar sensor providing long-range detection
 *          (up to 175m) with velocity measurement. Uses serial protocol.
 *          
 *          Only included on boards with >1MB flash due to protocol complexity.
 *          
 *          Default: AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 */
#ifndef AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED
#define AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED && HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @brief Enable HexSoon CAN radar rangefinder backend
 * 
 * @details HexSoon mmWave radar sensors using CAN bus protocol. Uses same driver
 *          infrastructure as NRA24 radars.
 *          
 *          Requires HAL_MAX_CAN_PROTOCOL_DRIVERS > 0 for CAN support.
 *          
 *          Default: HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
 * 
 * @note Shares CAN driver implementation with NRA24 - see AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED
 */
#ifndef AP_RANGEFINDER_HEXSOONRADAR_ENABLED
#define AP_RANGEFINDER_HEXSOONRADAR_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS && AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Combined enable flag for NRA24 CAN protocol driver
 * 
 * @details This flag enables the shared CAN driver implementation used by both
 *          NRA24 and HexSoon radar sensors. The driver is included if either
 *          AP_RANGEFINDER_NRA24_CAN_ENABLED or AP_RANGEFINDER_HEXSOONRADAR_ENABLED
 *          is enabled.
 *          
 *          This allows efficient code sharing between similar CAN radar backends.
 * 
 * @note This is a computed flag, not directly user-configurable
 */
#define AP_RANGEFINDER_NRA24_CAN_DRIVER_ENABLED (AP_RANGEFINDER_HEXSOONRADAR_ENABLED || AP_RANGEFINDER_NRA24_CAN_ENABLED)
