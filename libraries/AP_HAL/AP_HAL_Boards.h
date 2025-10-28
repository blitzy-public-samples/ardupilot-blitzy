/**
 * @file AP_HAL_Boards.h
 * @brief Board selection and platform capability definitions
 * 
 * @details Central header for board identification and platform-specific feature configuration.
 *          Defines HAL_BOARD constants, includes platform-specific headers, and computes
 *          capability macros based on board characteristics. Every source file indirectly
 *          includes this via AP_HAL.h.
 *          
 *          This file serves as the authoritative source for:
 *          - Board type enumeration (HAL_BOARD_* constants)
 *          - Board subtype identification (HAL_BOARD_SUBTYPE_* constants)
 *          - Platform-specific capability detection
 *          - Feature flag computation based on hardware capabilities
 *          - Default configuration values for cross-platform compatibility
 *          
 *          The CONFIG_HAL_BOARD macro is set by the build system (waf) based on the
 *          --board argument and determines which platform-specific header is included.
 *          
 * @note This is the authoritative source for board identification
 * @note Modification affects entire build - use caution when changing
 * @warning Any changes to board constants or capability macros require full rebuild
 * 
 * @see AP_HAL.h for the main HAL interface
 * @see libraries/AP_HAL_ChibiOS/hwdef/README.md for board definition system
 */
#pragma once

/**
 * @brief Board type enumeration
 * 
 * @details CONFIG_HAL_BOARD macro identifies the target platform at compile time.
 *          Set by build system (waf, make) based on --board argument.
 *          
 *          HAL_BOARD values and their platforms:
 *          
 *          - HAL_BOARD_CHIBIOS (10): ARM embedded boards using ChibiOS RTOS
 *            * Most Pixhawk variants (Pixhawk 1-6, CubeOrange, CubeBlack, etc.)
 *            * STM32F4/F7/H7 microcontrollers
 *            * 150+ board definitions in libraries/AP_HAL_ChibiOS/hwdef/
 *            * Real-time performance with hardware floating-point
 *            
 *          - HAL_BOARD_LINUX (7): Linux-based flight controllers
 *            * Navio/Navio2, BeagleBone Blue, Raspberry Pi
 *            * Uses Linux kernel for device access (sysfs, /dev)
 *            * POSIX I/O and socket support
 *            * Higher-level OS features but less real-time guarantees
 *            
 *          - HAL_BOARD_SITL (3): Software In The Loop simulation
 *            * Desktop/laptop simulation for testing
 *            * Physics simulation integration (JSBSim, Gazebo)
 *            * Network-based sensor simulation
 *            * Used extensively for automated testing
 *            
 *          - HAL_BOARD_ESP32 (12): ESP32-based boards
 *            * Low-cost WiFi/Bluetooth-enabled boards
 *            * ESP-IDF framework integration
 *            * Limited resources compared to ARM boards
 *            * Community-driven board support
 *            
 *          - HAL_BOARD_QURT (13): Qualcomm Hexagon DSP platform
 *            * Snapdragon Flight boards
 *            * QURT RTOS on Hexagon DSP
 *            * Specialized high-performance platform
 *            
 *          - HAL_BOARD_EMPTY (99): Template/stub implementation
 *            * Minimal HAL implementation for porting reference
 *            * Does not produce functional firmware
 *            * Starting point for new platform ports
 *          
 * @note CONFIG_HAL_BOARD is set by build system, not manually edited in code
 * @note Numeric values are historical and not sequential
 * @warning Changing HAL_BOARD values breaks binary compatibility with logs
 * 
 * @see CONFIG_HAL_BOARD for compile-time board selection
 * @see CONFIG_HAL_BOARD_SUBTYPE for fine-grained board identification
 */
// @LoggerEnum: HAL_BOARD
#define HAL_BOARD_SITL     3
// #define HAL_BOARD_SMACCM   4  // unused
// #define HAL_BOARD_PX4      5  // unused
#define HAL_BOARD_LINUX    7
// #define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_CHIBIOS  10
// #define HAL_BOARD_F4LIGHT  11 // reserved
#define HAL_BOARD_ESP32	   12
#define HAL_BOARD_QURT     13
#define HAL_BOARD_EMPTY    99
// @LoggerEnumEnd

/**
 * @brief Board subtype identification
 * 
 * @details CONFIG_HAL_BOARD_SUBTYPE provides fine-grained board identification within
 *          a HAL_BOARD family. This allows conditional compilation for board-specific
 *          quirks or features without creating separate HAL implementations.
 *          
 *          Subtype ranges by platform:
 *          - -1: No specific subtype (HAL_BOARD_SUBTYPE_NONE)
 *          - 1000-1999: Linux boards (Navio, BeagleBone, Navigator, etc.)
 *          - 5000-5999: ChibiOS boards (FMUv3, FMUv5, etc. - used sparingly)
 *          - 6000-6999: ESP32 boards (DIY, Icarus, Buzz, etc.)
 *          
 *          Design philosophy:
 *          Most boards do NOT need a subtype. Subtypes are only defined when
 *          platform-specific #ifdef code is required to handle hardware differences
 *          that cannot be detected at runtime. For ChibiOS boards, the hwdef system
 *          handles most configuration, making subtypes rarely necessary.
 *          
 *          For ChibiOS boards: The subtype is defined in hwdef.dat for boards that
 *          require special handling in shared code. Most ChibiOS boards use
 *          HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC or no subtype at all.
 *          
 * @note Subtype determines hwdef file selection in libraries/AP_HAL_ChibiOS/hwdef/
 * @note Query at runtime via AP_BoardConfig or hal.util->get_board_type()
 * @note Keep subtypes to minimum - prefer runtime detection when possible
 * 
 * @warning Adding subtypes without justification creates maintenance burden
 * @warning Subtype changes affect binary logging compatibility
 * 
 * @see hwdef.dat files for ChibiOS board-specific definitions
 * @see AP_BoardConfig for runtime board identification
 */
// @LoggerEnum: HAL_BOARD_SUBTYPE
/* Default board subtype is -1 */
#define HAL_BOARD_SUBTYPE_NONE -1

/* HAL Linux sub-types, starting at 1000 */
#define HAL_BOARD_SUBTYPE_LINUX_NONE       1000
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD  1001
#define HAL_BOARD_SUBTYPE_LINUX_PXF        1002
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO      1003
#define HAL_BOARD_SUBTYPE_LINUX_ZYNQ       1004
#define HAL_BOARD_SUBTYPE_LINUX_BBBMINI    1005
#define HAL_BOARD_SUBTYPE_LINUX_BEBOP      1006
#define HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 1009
#define HAL_BOARD_SUBTYPE_LINUX_BH         1010
#define HAL_BOARD_SUBTYPE_LINUX_PXFMINI    1012
#define HAL_BOARD_SUBTYPE_LINUX_NAVIO2     1013
#define HAL_BOARD_SUBTYPE_LINUX_DISCO      1014
#define HAL_BOARD_SUBTYPE_LINUX_AERO       1015
#define HAL_BOARD_SUBTYPE_LINUX_DARK       1016
#define HAL_BOARD_SUBTYPE_LINUX_BLUE       1018
#define HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ 1019
#define HAL_BOARD_SUBTYPE_LINUX_EDGE       1020
#define HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ   1021
#define HAL_BOARD_SUBTYPE_LINUX_POCKET     1022
#define HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR  1023
#define HAL_BOARD_SUBTYPE_LINUX_VNAV       1024
#define HAL_BOARD_SUBTYPE_LINUX_OBAL_V1    1025
#define HAL_BOARD_SUBTYPE_LINUX_CANZERO    1026
#define HAL_BOARD_SUBTYPE_LINUX_PILOTPI    1027
#define HAL_BOARD_SUBTYPE_LINUX_POCKET2    1028
/* HAL CHIBIOS sub-types, starting at 5000

   NOTE!! Do not add more subtypes unless they are really needed. Most
   boards do not need a subtype defined. It is only needed if we need
   to use #ifdef'd code to change behaviour
*/
// #define HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412	5000
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV3         5001
// #define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV4         5002
#define HAL_BOARD_SUBTYPE_CHIBIOS_GENERIC       5009
#define HAL_BOARD_SUBTYPE_CHIBIOS_FMUV5         5013
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V51   5016
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V52   5017
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRUBRAIN_V51  5018
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRCORE_V10    5019
// #define HAL_BOARD_SUBTYPE_CHIBIOS_VRBRAIN_V54   5020

#define HAL_BOARD_SUBTYPE_ESP32_DIY             6001
#define HAL_BOARD_SUBTYPE_ESP32_ICARUS          6002
#define HAL_BOARD_SUBTYPE_ESP32_BUZZ            6003
#define HAL_BOARD_SUBTYPE_ESP32_EMPTY           6004
#define HAL_BOARD_SUBTYPE_ESP32_TOMTE76         6005
#define HAL_BOARD_SUBTYPE_ESP32_NICK            6006
#define HAL_BOARD_SUBTYPE_ESP32_S3DEVKIT        6007
#define HAL_BOARD_SUBTYPE_ESP32_S3EMPTY         6008
#define HAL_BOARD_SUBTYPE_ESP32_S3M5STAMPFLY    6009
#define HAL_BOARD_SUBTYPE_ESP32_IMU_MODULE_V11  6010
// @LoggerEnumEnd

/**
 * @brief InertialSensor driver type identifiers
 * 
 * @details Legacy constants used for default IMU driver selection in board definitions.
 *          Modern boards typically detect IMU types at runtime through probe functions,
 *          but these constants remain for compatibility and default configuration.
 *          
 *          Common IMU types:
 *          - MPU60XX family: 6-axis IMU (3-axis gyro + 3-axis accel)
 *          - MPU9250: 9-axis IMU (adds 3-axis magnetometer)
 *          - INV2: Invensense ICM-20xxx second-generation IMUs
 *          
 *          Interface types:
 *          - SPI: High-speed serial interface (preferred for flight controllers)
 *          - I2C: Lower-speed bus interface (supports multiple devices)
 *          
 * @note Most boards auto-detect IMU at runtime via AP_InertialSensor probe functions
 * @note These constants are primarily for HAL_INS_DEFAULT in board configs
 * 
 * @see libraries/AP_InertialSensor/AP_InertialSensor_Backend.h for driver interface
 */
/* InertialSensor driver types */
#define HAL_INS_NONE         0
#define HAL_INS_MPU60XX_SPI  2
#define HAL_INS_MPU60XX_I2C  3
#define HAL_INS_HIL_UNUSED   4  // unused
#define HAL_INS_VRBRAIN      8
#define HAL_INS_MPU9250_SPI  9
#define HAL_INS_MPU9250_I2C 13
#define HAL_INS_MPU6500     19
#define HAL_INS_INV2_I2C    24
#define HAL_INS_INV2_SPI    25

/**
 * @brief Heat control types for IMU thermal management
 * 
 * @details Some boards include IMU heaters to maintain stable temperature for improved
 *          gyroscope bias stability. The heat type determines the control method.
 *          
 * @note HAL_LINUX_HEAT_PWM (1): PWM-based heater control on Linux boards
 * @see HAL_HAVE_IMU_HEATER capability flag
 */
/* Heat Types */
#define HAL_LINUX_HEAT_PWM 1

/**
 * @brief CPU performance classes for algorithm selection
 * 
 * @details Approximate CPU speed classification used to enable/disable computationally
 *          intensive algorithms. Not precise clock speeds, but capability indicators.
 *          
 *          HAL_CPU_CLASS_150 (3): ~150MHz ARM Cortex-M4/M7
 *          - STM32F4/F7 microcontrollers (most Pixhawk boards)
 *          - Hardware floating-point unit
 *          - Tens to hundreds of kilobytes RAM
 *          - Real-time operation with careful resource management
 *          
 *          HAL_CPU_CLASS_1000 (4): GHz-class processors
 *          - SITL simulation on desktop/laptop
 *          - Linux boards (Raspberry Pi, BeagleBone)
 *          - Megabytes of memory available
 *          - Can run advanced algorithms without optimization constraints
 *          
 * @note Used to conditionally enable features like advanced EKF options, FFT analysis
 * @note Does not reflect exact CPU frequency, only capability tier
 * 
 * @see HAL_GYROFFT_ENABLED for example of CPU-class-dependent features
 */
/* CPU classes, used to select if CPU intensive algorithms should be used
 * Note that these are only approximate, not exact CPU speeds. */

/* 150Mhz: STM32F4 or similar. Assumes:
 *  - hardware floating point
 *  - tens of kilobytes of memory available
*/
#define HAL_CPU_CLASS_150  3

/* GigaHz class: SITL, BeagleBone etc. Assumes megabytes of memory available. */
#define HAL_CPU_CLASS_1000 4

/**
 * @brief Memory class definitions for feature gating
 * 
 * @details Memory classes specify minimum RAM requirements in kilobytes.
 *          Used to conditionally compile features based on available memory.
 *          Board must have at least the specified amount of RAM to use that class.
 *          
 *          Memory classes and typical boards:
 *          - HAL_MEM_CLASS_20 (1):   20KB+   - Minimal microcontrollers
 *          - HAL_MEM_CLASS_64 (2):   64KB+   - Small ARM Cortex-M0/M3 boards
 *          - HAL_MEM_CLASS_192 (3):  192KB+  - STM32F4 entry-level
 *          - HAL_MEM_CLASS_300 (4):  300KB+  - STM32F4 mid-range (Pixhawk 1)
 *          - HAL_MEM_CLASS_500 (5):  500KB+  - STM32F7/H7 boards (Pixhawk 4+)
 *          - HAL_MEM_CLASS_1000 (6): 1000KB+ - Linux boards, advanced STM32H7
 *          
 *          Higher memory classes enable:
 *          - More simultaneous flight modes
 *          - Larger log buffers
 *          - Advanced navigation features (SmartRTL path storage)
 *          - More EKF cores/lanes
 *          - Larger parameter sets
 *          
 * @note Memory class set in board-specific configuration (hwdef.dat for ChibiOS)
 * @note Features test memory class: #if HAL_MEM_CLASS >= HAL_MEM_CLASS_300
 * 
 * @warning Underestimating memory class can cause out-of-memory crashes
 * @see HAL_MEM_CLASS in board configuration files
 */

/*
  memory classes, in kbytes. Board must have at least the given amount
  of memory
*/
#define HAL_MEM_CLASS_20   1
#define HAL_MEM_CLASS_64   2
#define HAL_MEM_CLASS_192  3
#define HAL_MEM_CLASS_300  4
#define HAL_MEM_CLASS_500  5
#define HAL_MEM_CLASS_1000 6

/**
 * @brief Operating system feature flags
 * 
 * @details Platform-specific headers define these flags to indicate OS capabilities.
 *          Used for conditional compilation of features requiring specific OS support.
 *          
 *          HAL_OS_POSIX_IO (0 or 1):
 *          - POSIX-like filesystem I/O available (open, read, write, close)
 *          - Enabled on Linux and SITL platforms
 *          - Allows standard file operations for logging, terrain data, scripts
 *          - Disabled on bare-metal embedded platforms (ChibiOS, ESP32)
 *          
 *          HAL_OS_SOCKETS (0 or 1):
 *          - POSIX-like socket API available (socket, bind, connect, send, recv)
 *          - Enabled on Linux and SITL platforms
 *          - Required for network communication (MAVLink over UDP/TCP, DDS)
 *          - Disabled on platforms without network stack
 *          
 * @note These flags are set by platform headers (sitl.h, linux.h, chibios.h, etc.)
 * @note Check flags with: #if HAL_OS_POSIX_IO ... #endif
 * 
 * @see AP_HAL/board/sitl.h for SITL OS feature definitions
 * @see AP_HAL/board/linux.h for Linux OS feature definitions
 */
/* Operating system features
 *
 * HAL implementations may define the following extra feature defines to 1 if
 * available:
 *
 * - HAL_OS_POSIX_IO : has posix-like filesystem IO
 * - HAL_OS_SOCKETS  : has posix-like sockets */

/**
 * @brief Board-specific header inclusion
 * 
 * @details CONFIG_HAL_BOARD macro (set by build system) determines which platform-specific
 *          header is included. Each header defines board capabilities, hardware resources,
 *          and platform-specific constants.
 *          
 *          Platform headers included:
 *          - AP_HAL/board/sitl.h: SITL simulation platform configuration
 *          - AP_HAL/board/linux.h: Linux board support (Navio, BeagleBone, etc.)
 *          - AP_HAL/board/empty.h: Stub implementation template
 *          - AP_HAL/board/chibios.h: ChibiOS/ARM embedded boards
 *          - AP_HAL/board/esp32.h: ESP32 platform configuration
 *          - AP_HAL/board/qurt.h: Qualcomm Hexagon DSP platform
 *          
 *          Each platform header defines:
 *          - Hardware capabilities (I2C count, UART count, DMA channels, etc.)
 *          - Feature flags (HAL_WITH_DSP, HAL_NUM_CAN_IFACES, etc.)
 *          - Memory regions and sizes (HAL_STORAGE_SIZE, flash limits)
 *          - Platform-specific constants and defaults
 *          - OS feature flags (HAL_OS_POSIX_IO, HAL_OS_SOCKETS)
 *          
 * @note BUILD SYSTEM sets CONFIG_HAL_BOARD via --board argument to waf
 * @note Only ONE platform header is included per build
 * @warning Build fails if CONFIG_HAL_BOARD is undefined or invalid
 * 
 * @see wscript for board selection and CONFIG_HAL_BOARD assignment
 * @see libraries/AP_HAL_ChibiOS/hwdef/ for ChibiOS board definitions
 */
/* DEFINITIONS FOR BOARDS */

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    #include <AP_HAL/board/sitl.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include <AP_HAL/board/linux.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY
    #include <AP_HAL/board/empty.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
	#include <AP_HAL/board/chibios.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    #include <AP_HAL/board/esp32.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_QURT
    #include <AP_HAL/board/qurt.h>
#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

#ifndef CONFIG_HAL_BOARD_SUBTYPE
#error "No CONFIG_HAL_BOARD_SUBTYPE set"
#endif

/**
 * @brief Capability macros computed from board configuration
 * 
 * @details This section defines default values for capability flags that may be
 *          set by board-specific headers. These macros enable conditional compilation
 *          throughout ArduPilot to adapt to hardware capabilities and limitations.
 *          
 *          Platform headers (sitl.h, chibios.h, linux.h, etc.) can define these macros
 *          before this file processes them. If not defined, sensible defaults are set here.
 *          
 *          Key capability categories:
 *          
 *          Hardware capabilities:
 *          - HAL_WITH_DSP: DSP/FFT hardware available for dynamic notch filters
 *          - HAL_NUM_CAN_IFACES: Number of CAN bus interfaces (0-3)
 *          - HAL_WITH_IO_MCU: Separate I/O coprocessor present (Pixhawk IOMCU)
 *          - HAL_HAVE_IMU_HEATER: IMU thermal management hardware
 *          - HAL_WITH_HARDWARE_DOUBLE: Hardware double-precision FPU
 *          
 *          Memory and storage:
 *          - HAL_STORAGE_SIZE: Parameter storage size in bytes
 *          - HAL_PROGRAM_SIZE_LIMIT_KB: Flash memory limit for firmware in KB
 *          - HAL_RAM_RESERVE_START: Reserved RAM region start address
 *          
 *          Peripheral features:
 *          - HAL_SUPPORT_RCOUT_SERIAL: Serial RC output protocol support
 *          - HAL_DSHOT_ENABLED: DShot ESC protocol support
 *          - HAL_SERIALLED_ENABLED: Serial LED (NeoPixel) support
 *          - HAL_HAVE_DUAL_USB_CDC: Dual USB CDC (serial) support
 *          
 *          Protocol and communication:
 *          - HAL_CANMANAGER_ENABLED: CAN bus manager enabled
 *          - HAL_ENABLE_DRONECAN_DRIVERS: DroneCAN/UAVCAN protocol support
 *          - AP_CAN_SLCAN_ENABLED: SLCAN bridge support
 *          
 *          Advanced features:
 *          - HAL_GYROFFT_ENABLED: Gyro FFT analysis for notch filter tuning
 *          - HAL_HNF_MAX_FILTERS: Maximum harmonic notch filters
 *          - HAL_WITH_POSTYPE_DOUBLE: Use double precision for position
 *          - HAL_ENABLE_THREAD_STATISTICS: Thread performance monitoring
 *          
 *          Build and debug features:
 *          - AP_SIGNED_FIRMWARE: Firmware signature verification
 *          - AP_BOOTLOADER_FLASHING_ENABLED: In-flight bootloader update
 *          - AP_CRASHDUMP_ENABLED: Crash dump generation
 *          - HAL_WITH_MCU_MONITORING: MCU health monitoring
 *          
 *          Usage pattern in libraries:
 *          @code
 *          #if HAL_WITH_DSP
 *              // Use hardware FFT for notch filter
 *              FFTWindowState* fft = hal.dsp->fft_init(512, 1000, 4);
 *          #else
 *              // DSP unavailable - skip advanced filtering
 *          #endif
 *          
 *          #if HAL_NUM_CAN_IFACES > 0
 *              // Initialize CAN bus manager
 *              AP_CANManager::get_singleton()->init();
 *          #endif
 *          @endcode
 *          
 * @note Capability macros enable libraries to adapt to platform capabilities
 * @note Changing capability macros requires full rebuild
 * @warning Setting capabilities beyond hardware support causes runtime failures
 * 
 * @see board/chibios.h for ChibiOS capability definitions
 * @see board/sitl.h for SITL capability definitions
 * @see board/linux.h for Linux capability definitions
 */

// HAL_PROGRAM_SIZE_LIMIT_KB is the amount of space we have for
// instructions.  on ChibiOS this is the sum of onboard and external
// flash.  BOARD_FLASH_SIZE is reserved for use in the HAL backends
// (usually only ChibiOS) and should not be used in general code.
#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#error HAL_PROGRAM_SIZE_LIMIT_KB must be defined
#endif

#ifndef HAL_OS_SOCKETS
#define HAL_OS_SOCKETS 0
#endif

#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 0
#endif

#ifndef HAL_PARAM_DEFAULTS_PATH
#define HAL_PARAM_DEFAULTS_PATH nullptr
#endif

#ifndef HAL_HAVE_IMU_HEATER
#define HAL_HAVE_IMU_HEATER 0
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

#ifndef HAL_WITH_IO_MCU_BIDIR_DSHOT
#define HAL_WITH_IO_MCU_BIDIR_DSHOT 0
#endif

#ifndef HAL_WITH_IO_MCU_DSHOT
#define HAL_WITH_IO_MCU_DSHOT HAL_WITH_IO_MCU_BIDIR_DSHOT
#endif

#ifndef HAL_REQUIRES_BDSHOT_SUPPORT
#define HAL_REQUIRES_BDSHOT_SUPPORT (defined(HAL_WITH_BIDIR_DSHOT) || HAL_WITH_IO_MCU_BIDIR_DSHOT)
#endif

#ifndef AP_NOTIFY_TONEALARM_ENABLED
#define AP_NOTIFY_TONEALARM_ENABLED 0
#endif

// support for Extended DShot Telemetry v2 is enabled only if any kind of such telemetry
// can in principle arrive, either from servo outputs or from IOMCU

// if not desired, set to 0 - and if IOMCU has bidirectional DShot enabled, recompile it too,
// otherwise the communication to IOMCU breaks!
#ifndef AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
#define AP_EXTENDED_DSHOT_TELEM_V2_ENABLED HAL_REQUIRES_BDSHOT_SUPPORT
#endif

#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

// enable AP_GyroFFT library only if required:
#ifndef HAL_WITH_DSP
#define HAL_WITH_DSP HAL_GYROFFT_ENABLED
#endif

#ifndef AP_HAL_UARTDRIVER_ENABLED
#define AP_HAL_UARTDRIVER_ENABLED 1
#endif

#ifndef HAL_OS_FATFS_IO
#define HAL_OS_FATFS_IO 0
#endif

#ifndef HAL_OS_LITTLEFS_IO
#define HAL_OS_LITTLEFS_IO 0
#endif

#ifndef HAL_BARO_DEFAULT
#define HAL_BARO_DEFAULT HAL_BARO_NONE
#endif

#ifndef HAL_INS_DEFAULT
#define HAL_INS_DEFAULT HAL_INS_NONE
#endif

#ifndef HAL_GPS1_TYPE_DEFAULT
#define HAL_GPS1_TYPE_DEFAULT 1
#endif

#ifndef HAL_CAN_DRIVER_DEFAULT
#define HAL_CAN_DRIVER_DEFAULT 0
#endif

#ifndef HAL_MAX_CAN_PROTOCOL_DRIVERS
    #define HAL_MAX_CAN_PROTOCOL_DRIVERS HAL_NUM_CAN_IFACES
#endif

#ifndef HAL_CANMANAGER_ENABLED
#define HAL_CANMANAGER_ENABLED (HAL_MAX_CAN_PROTOCOL_DRIVERS > 0)
#endif

#ifndef HAL_ENABLE_DRONECAN_DRIVERS
#define HAL_ENABLE_DRONECAN_DRIVERS HAL_CANMANAGER_ENABLED
#endif

#ifndef AP_TEST_DRONECAN_DRIVERS
#define AP_TEST_DRONECAN_DRIVERS 0
#endif

#ifdef HAVE_LIBDL
#define AP_MODULE_SUPPORTED 1
#else
#define AP_MODULE_SUPPORTED 0
#endif

#ifndef HAL_SUPPORT_RCOUT_SERIAL
#define HAL_SUPPORT_RCOUT_SERIAL 0
#endif

#ifndef HAL_HAVE_DUAL_USB_CDC
#define HAL_HAVE_DUAL_USB_CDC 0
#endif

#ifndef AP_CAN_SLCAN_ENABLED
#if HAL_NUM_CAN_IFACES && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#define AP_CAN_SLCAN_ENABLED 1
#else
#define AP_CAN_SLCAN_ENABLED 0
#endif
#endif

#ifndef AP_HAL_SHARED_DMA_ENABLED
#define AP_HAL_SHARED_DMA_ENABLED 1
#endif

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS 0
#endif

#ifndef AP_STATS_ENABLED
#define AP_STATS_ENABLED 1
#endif

#ifndef HAL_WITH_MCU_MONITORING
#define HAL_WITH_MCU_MONITORING 0
#endif

#ifndef AP_CRASHDUMP_ENABLED
#define AP_CRASHDUMP_ENABLED 0
#endif

#ifndef AP_SIGNED_FIRMWARE
#define AP_SIGNED_FIRMWARE 0
#endif

#ifndef HAL_DSHOT_ALARM_ENABLED
#define HAL_DSHOT_ALARM_ENABLED 0
#endif

#ifndef HAL_DSHOT_ENABLED
#define HAL_DSHOT_ENABLED 1
#endif

#ifndef HAL_SERIALLED_ENABLED
#define HAL_SERIALLED_ENABLED HAL_DSHOT_ENABLED
#endif

#ifndef HAL_SERIAL_ESC_COMM_ENABLED
#define HAL_SERIAL_ESC_COMM_ENABLED 1
#endif

#ifndef AP_BOOTLOADER_FLASHING_ENABLED
#define AP_BOOTLOADER_FLASHING_ENABLED 0
#endif

#ifndef HAL_HNF_MAX_FILTERS
// On an F7 The difference in CPU load between 1 notch and 24 notches is about 2%
// The difference in CPU load between 1Khz backend and 2Khz backend is about 10%
// So at 1Khz almost all notch combinations can be supported on F7 and certainly H7
#if defined(STM32H7) || CONFIG_HAL_BOARD == HAL_BOARD_SITL
// Enough for a double-notch per motor on an octa using three IMUs and one harmonics
// plus one static notch with one double-notch harmonics
#define HAL_HNF_MAX_FILTERS 54
#elif defined(STM32F7)
// Enough for a notch per motor on an octa using three IMUs and one harmonics
// plus one static notch with one harmonics
#define HAL_HNF_MAX_FILTERS 27
#else
// Enough for a notch per motor on an octa quad using two IMUs and one harmonic
// plus one static notch with one harmonic
// Or triple-notch per motor on one IMU with one harmonic
#define HAL_HNF_MAX_FILTERS 24
#endif
#endif // HAL_HNF_MAX_FILTERS

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL // allow SITL to have all the CANFD options
#define HAL_CANFD_SUPPORTED 8
#elif !defined(HAL_CANFD_SUPPORTED)
#define HAL_CANFD_SUPPORTED 0
#endif

#ifndef HAL_USE_QUADSPI
#define HAL_USE_QUADSPI 0
#endif

#ifndef HAL_USE_OCTOSPI
#define HAL_USE_OCTOSPI 0
#endif

#ifndef __RAMFUNC__
#define __RAMFUNC__
#endif

#ifndef __FASTRAMFUNC__
#define __FASTRAMFUNC__
#endif

#ifndef __EXTFLASHFUNC__
#define __EXTFLASHFUNC__
#endif

// Use __INITFUNC__ to mark functions which are only called once, at
// boot.  On some boards we choose to put such functions into areas of
// flash memory which are slower than others.
#ifndef __INITFUNC__
#define __INITFUNC__ __EXTFLASHFUNC__
#endif

#ifndef HAL_ENABLE_DFU_BOOT
#define HAL_ENABLE_DFU_BOOT 0
#endif


#ifndef HAL_ENABLE_SENDING_STATS
#define HAL_ENABLE_SENDING_STATS HAL_PROGRAM_SIZE_LIMIT_KB >= 256
#endif

#ifndef HAL_GPIO_LED_ON
#define HAL_GPIO_LED_ON 0
#elif HAL_GPIO_LED_ON == 0
#error "Do not specify HAL_GPIO_LED_ON if you are setting it to the default, 0"
#endif

#ifdef HAL_GPIO_LED_OFF
#error "HAL_GPIO_LED_OFF must not be defined, it is implicitly !HAL_GPIO_LED_ON"
#endif

#ifndef HAL_WITH_POSTYPE_DOUBLE
#define HAL_WITH_POSTYPE_DOUBLE HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

#ifndef HAL_INS_RATE_LOOP
#define HAL_INS_RATE_LOOP 0
#endif

#define HAL_GPIO_LED_OFF (!HAL_GPIO_LED_ON)

#ifndef HAL_REBOOT_ON_MEMORY_ERRORS
#define HAL_REBOOT_ON_MEMORY_ERRORS defined(IOMCU_FW)
#endif

/**
 * @brief Conditional compilation patterns and usage examples
 * 
 * @details This file defines numerous capability macros that enable conditional compilation
 *          throughout ArduPilot. Below are common patterns for using these macros effectively.
 *          
 * @section board_detection Board Type Detection
 * 
 * Test for specific board types to enable platform-specific code:
 * 
 * @code{.cpp}
 * #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
 *     // ChibiOS-specific initialization
 *     stm32_boardInitialize();
 * #elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
 *     // Linux-specific initialization
 *     linux_setup_sysfs();
 * #elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
 *     // SITL-specific debugging
 *     printf("SITL: Starting simulation\n");
 * #endif
 * @endcode
 * 
 * @section capability_flags Capability-Based Feature Gating
 * 
 * Enable features based on hardware capabilities:
 * 
 * @code{.cpp}
 * void setup_advanced_filtering() {
 * #if HAL_WITH_DSP
 *     // Use hardware FFT for gyro notch filtering
 *     FFTWindowState* fft = hal.dsp->fft_init(512, 1000, 4);
 *     if (fft != nullptr) {
 *         setup_dynamic_notch_filter(fft);
 *     }
 * #else
 *     // DSP hardware unavailable - skip advanced filtering
 *     gcs().send_text(MAV_SEVERITY_INFO, "Dynamic notch filtering unavailable");
 * #endif
 * }
 * @endcode
 * 
 * @section can_support CAN Bus Conditional Code
 * 
 * Conditionally compile CAN-related code:
 * 
 * @code{.cpp}
 * void init_can_peripherals() {
 * #if HAL_NUM_CAN_IFACES > 0
 *     // Initialize CAN manager for boards with CAN support
 *     AP::can().init();
 *     
 *     #if HAL_NUM_CAN_IFACES > 1
 *         // Configure second CAN interface if available
 *         AP::can().set_protocol(1, AP_CANManager::Driver_Type_UAVCAN);
 *     #endif
 * #endif
 * }
 * @endcode
 * 
 * @section memory_gating Memory-Based Feature Gating
 * 
 * Enable memory-intensive features on capable boards:
 * 
 * @code{.cpp}
 * void setup_navigation() {
 * #if HAL_MEM_CLASS >= HAL_MEM_CLASS_500
 *     // Enable SmartRTL on boards with sufficient memory
 *     g2.smart_rtl.init();
 * #endif
 * 
 * #if HAL_MEM_CLASS >= HAL_MEM_CLASS_300
 *     // Enable rally points (requires moderate memory)
 *     rally.init();
 * #endif
 * }
 * @endcode
 * 
 * @section os_features Operating System Feature Detection
 * 
 * Use POSIX features when available:
 * 
 * @code{.cpp}
 * bool save_to_filesystem(const char* filename, const uint8_t* data, size_t len) {
 * #if HAL_OS_POSIX_IO
 *     // Use POSIX file I/O on Linux/SITL
 *     int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
 *     if (fd == -1) return false;
 *     ssize_t written = write(fd, data, len);
 *     close(fd);
 *     return written == (ssize_t)len;
 * #elif HAL_OS_FATFS_IO
 *     // Use FATFS on embedded boards with SD card
 *     return AP::FS().write_file(filename, data, len);
 * #else
 *     // No filesystem support
 *     return false;
 * #endif
 * }
 * @endcode
 * 
 * @section board_subtype Board Subtype Handling
 * 
 * Conditionally compile for specific board variants:
 * 
 * @code{.cpp}
 * void board_specific_init() {
 * #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_FMUV5
 *     // FMUv5-specific initialization (Pixhawk 4)
 *     setup_fmuv5_power_management();
 * #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
 *     // Navio2-specific initialization
 *     setup_navio2_leds();
 * #endif
 * }
 * @endcode
 * 
 * @section multi_condition Multiple Condition Testing
 * 
 * Combine multiple capability checks:
 * 
 * @code{.cpp}
 * void setup_telemetry() {
 * #if HAL_OS_SOCKETS && (CONFIG_HAL_BOARD == HAL_BOARD_LINUX || CONFIG_HAL_BOARD == HAL_BOARD_SITL)
 *     // Enable UDP telemetry on platforms with socket support
 *     mavlink_udp.init(14550);
 * #endif
 * 
 * #if HAL_NUM_CAN_IFACES > 0 && HAL_ENABLE_DRONECAN_DRIVERS
 *     // Enable DroneCAN telemetry if CAN available
 *     dronecan_telemetry.init();
 * #endif
 * }
 * @endcode
 * 
 * @section program_size Flash Size Based Features
 * 
 * Gate large features based on available program memory:
 * 
 * @code{.cpp}
 * #if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
 *     // Enable full-featured EKF3 on boards with >1MB flash
 *     #define EKF3_FULL_FEATURES 1
 * #else
 *     // Use reduced-feature EKF3 on space-constrained boards
 *     #define EKF3_FULL_FEATURES 0
 * #endif
 * @endcode
 * 
 * @section runtime_queries Runtime Board Queries
 * 
 * For decisions that cannot be made at compile time, query board info at runtime:
 * 
 * @code{.cpp}
 * void log_board_info() {
 *     // Runtime board identification
 *     uint16_t board_type = hal.util->get_board_type();
 *     const char* board_name = hal.util->get_custom_log_directory();
 *     
 *     // Log board information
 *     logger.Write("BORD", "Type,Name", "IZ", board_type, board_name);
 * }
 * @endcode
 * 
 * @note Prefer compile-time checks (#if) for features to reduce binary size
 * @note Use runtime checks (if) for board identification when behavior varies per board
 * @note Capability macros are computed from board-specific headers (board/*.h files)
 * 
 * @warning Changing capability macros requires full rebuild of all source files
 * @warning Board-specific #ifdef code should be minimized - prefer runtime detection
 * 
 * @see libraries/AP_HAL_ChibiOS/hwdef/ for ChibiOS board definitions
 * @see libraries/AP_HAL/board/ for platform-specific capability definitions
 */
