/**
 * @file AP_BattMonitor_config.h
 * @brief Compile-time configuration for battery monitoring subsystem
 * 
 * @details This header defines feature flags for compile-time backend selection
 *          and feature gating in the AP_BattMonitor library. Configuration macros
 *          control which battery monitor backends are compiled into the firmware,
 *          enabling optimization of flash usage for memory-constrained boards.
 * 
 *          The battery monitoring subsystem supports diverse backends:
 *          - Analog voltage/current sensing via ADC
 *          - SMBus smart batteries with various manufacturer protocols
 *          - ESC telemetry aggregation
 *          - DroneCAN/UAVCAN battery messages
 *          - I2C power monitors (INA2xx, INA3221, LTC2946, AD7091R5)
 *          - SPI power monitors (INA239)
 *          - Fuel flow and level sensors for combustion engines
 *          - Generator and EFI system integration
 *          - Platform-specific implementations (Bebop, Solo)
 *          - Lua scripting interface for custom battery drivers
 * 
 * @note Flash Size Gating: Backends requiring >1MB flash are automatically
 *       disabled on smaller boards through HAL_PROGRAM_SIZE_LIMIT_KB checks
 *       to prevent build failures and conserve memory.
 * 
 * @note Hardware Definition Enablement: Some backends (INA239, INA3221, Solo,
 *       LTC2946) must be explicitly enabled in board hwdef.dat files with
 *       hardware bus/address configuration as they require board-specific setup.
 * 
 * @note Dependency Chaining: Many backends depend on AP_BATTERY_BACKEND_DEFAULT_ENABLED,
 *       which inherits from AP_BATTERY_ENABLED. SMBus variants depend on base
 *       AP_BATTERY_SMBUS_ENABLED. Generic/Maxell/Rotoye form an inheritance hierarchy.
 * 
 * @warning Disabling AP_BATTERY_ENABLED completely removes all battery monitoring
 *          functionality including voltage/current sensing and low-battery failsafes.
 *          This should only be done on boards without battery monitoring hardware.
 * 
 * @see libraries/AP_BattMonitor/AP_BattMonitor.h
 * @see libraries/AP_BattMonitor/AP_BattMonitor_Backend.h
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_ESC_Telem/AP_ESC_Telem_config.h>
#include <AP_EFI/AP_EFI_config.h>
#include <AP_Generator/AP_Generator_config.h>
#include <AP_Torqeedo/AP_Torqeedo_config.h>

/**
 * @brief Master enable for entire battery monitoring subsystem
 * 
 * Controls compilation of all battery monitoring functionality. When disabled,
 * removes all battery-related code including voltage/current sensing, capacity
 * tracking, and low-battery failsafes.
 * 
 * @note Default: 1 (enabled) - Required for normal vehicle operation with batteries
 * @warning Disabling removes critical battery failsafe functionality
 */
#ifndef AP_BATTERY_ENABLED
#define AP_BATTERY_ENABLED 1
#endif

/**
 * @brief Maximum number of simultaneous battery monitor instances
 * 
 * Defines array size limits for battery monitoring. Supports monitoring multiple
 * batteries simultaneously for redundancy, capacity aggregation, or multi-pack
 * configurations. Each instance consumes RAM for state tracking.
 * 
 * @note Default: 9 instances - Sufficient for most multi-battery configurations
 * @note Reducing this value saves RAM on memory-constrained boards
 */
#ifndef AP_BATT_MONITOR_MAX_INSTANCES
#define AP_BATT_MONITOR_MAX_INSTANCES       9
#endif

/**
 * @brief Default enable state for optional battery backends
 * 
 * Inherits from AP_BATTERY_ENABLED to provide a common default for most backends.
 * Individual backends can override this to enforce specific requirements.
 * Simplifies configuration by tying optional backends to master enable.
 * 
 * @note Default: Tied to AP_BATTERY_ENABLED
 * @note Acts as a master switch for most non-platform-specific backends
 */
#ifndef AP_BATTERY_BACKEND_DEFAULT_ENABLED
#define AP_BATTERY_BACKEND_DEFAULT_ENABLED AP_BATTERY_ENABLED
#endif

/**
 * @brief Analog voltage/current sensing via ADC
 * 
 * Enables traditional analog battery monitoring using voltage dividers and
 * current sense resistors connected to ADC inputs. Most common battery
 * monitoring method for simple autopilot configurations.
 * 
 * @note Default: Enabled (via AP_BATTERY_BACKEND_DEFAULT_ENABLED)
 * @note Required for boards using analog voltage/current sense pins
 */
#ifndef AP_BATTERY_ANALOG_ENABLED
#define AP_BATTERY_ANALOG_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief Bebop/Disco platform-specific battery monitoring
 * 
 * Specialized backend for Parrot Bebop and Disco platforms with proprietary
 * battery communication protocols. Not used on standard autopilot hardware.
 * 
 * @note Default: 0 (disabled) - Platform-specific, not general-purpose
 * @note Only relevant for Parrot Bebop/Disco flight controllers
 */
#ifndef AP_BATTERY_BEBOP_ENABLED
#define AP_BATTERY_BEBOP_ENABLED 0
#endif

/**
 * @brief EFI fuel system to battery capacity mapping
 * 
 * Enables battery monitoring backend that interfaces with Electronic Fuel
 * Injection systems to report fuel as battery capacity for combustion engines.
 * Allows unified battery/fuel monitoring through standard battery interface.
 * 
 * @note Default: Enabled if both battery monitoring and EFI are available
 * @note Requires: HAL_EFI_ENABLED
 * @see libraries/AP_EFI/AP_EFI_config.h
 */
#ifndef AP_BATTERY_EFI_ENABLED
#define AP_BATTERY_EFI_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && HAL_EFI_ENABLED
#endif

/**
 * @brief ESC telemetry aggregation for battery monitoring
 * 
 * Enables using ESC telemetry data (voltage, current, consumption) as battery
 * monitor source. Aggregates data from multiple ESCs to provide system-level
 * battery information without dedicated battery monitor hardware.
 * 
 * @note Default: Enabled if both battery monitoring and ESC telemetry are available
 * @note Requires: HAL_WITH_ESC_TELEM (BLHeli32, KISS, or other ESC with telemetry)
 * @see libraries/AP_ESC_Telem/AP_ESC_Telem_config.h
 */
#ifndef AP_BATTERY_ESC_ENABLED
#define AP_BATTERY_ESC_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && HAL_WITH_ESC_TELEM
#endif

/**
 * @brief Forward battery data to ESCs
 * 
 * Enables sending battery voltage/current information to ESCs via telemetry.
 * Allows ESCs to implement low-voltage protection or thrust limiting based on
 * battery state. Direction opposite of AP_BATTERY_ESC_ENABLED.
 * 
 * @note Default: 0 (disabled) - Requires ESC firmware support for battery data
 * @warning Experimental feature, not widely supported by ESC firmware
 */
#ifndef AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
#define AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED 0
#endif

/**
 * @brief Maximum power limiting feature
 * 
 * Enables BATT_WATT_MAX parameter for limiting total system power draw.
 * Useful for protecting power supply, preventing voltage sag, or enforcing
 * competition power limits. Throttle limited when power exceeds threshold.
 * 
 * @note Default: Enabled with battery monitoring
 * @note Requires accurate current sensing for proper operation
 */
#ifndef AP_BATTERY_WATT_MAX_ENABLED
#define AP_BATTERY_WATT_MAX_ENABLED AP_BATTERY_ENABLED
#endif

/**
 * @brief SMBus smart battery base support
 * 
 * Enables Smart Battery System (SMBus) protocol support for communicating with
 * intelligent batteries over I2C. Provides foundation for various SMBus battery
 * implementations including vendor-specific protocols. Smart batteries report
 * voltage, current, remaining capacity, temperature, and health information.
 * 
 * @note Default: Enabled (via AP_BATTERY_BACKEND_DEFAULT_ENABLED)
 * @note Required for: Solo, Generic, NeoDesign, SUI, Maxell, Rotoye batteries
 * @see AP_BATTERY_SMBUS_GENERIC_ENABLED and related vendor-specific backends
 */
#ifndef AP_BATTERY_SMBUS_ENABLED
#define AP_BATTERY_SMBUS_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief DroneCAN/UAVCAN battery information messages
 * 
 * Enables receiving battery status via DroneCAN (formerly UAVCAN) protocol over
 * CAN bus. Supports distributed battery monitoring with CAN-connected batteries
 * or power modules reporting standard uavcan.equipment.power.BatteryInfo messages.
 * 
 * @note Default: Enabled if both battery monitoring and DroneCAN drivers available
 * @note Requires: HAL_ENABLE_DRONECAN_DRIVERS
 * @see libraries/AP_DroneCAN
 */
#ifndef AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED
#define AP_BATTERY_UAVCAN_BATTERYINFO_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && HAL_ENABLE_DRONECAN_DRIVERS
#endif

/**
 * @brief Fuel flow rate sensors for combustion engines
 * 
 * Enables monitoring fuel consumption via fuel flow meters (typically turbine-style
 * or paddle-wheel sensors). Integrates flow rate over time to track fuel consumed
 * and remaining, reporting through battery capacity interface.
 * 
 * @note Default: Enabled on boards with >1MB flash
 * @note Flash size gated: Disabled on small boards to conserve memory
 * @note Useful for combustion engine aircraft and hybrid systems
 */
#ifndef AP_BATTERY_FUELFLOW_ENABLED
#define AP_BATTERY_FUELFLOW_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * @brief PWM-based fuel level sensors
 * 
 * Enables reading fuel tank level from sensors that output PWM signals proportional
 * to fuel quantity. Common in commercial fuel tank senders. Converts PWM duty cycle
 * or frequency to fuel level through calibration curve.
 * 
 * @note Default: Enabled on boards with >1MB flash
 * @note Flash size gated: Disabled on small boards to conserve memory
 * @note Requires PWM input capable hardware pins
 */
#ifndef AP_BATTERY_FUELLEVEL_PWM_ENABLED
#define AP_BATTERY_FUELLEVEL_PWM_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * @brief Analog fuel level sensors
 * 
 * Enables reading fuel tank level from analog resistive fuel level senders via
 * ADC inputs. Typical automotive-style fuel senders vary resistance with float
 * position. Converts ADC voltage to fuel level through calibration.
 * 
 * @note Default: Enabled on boards with >1MB flash
 * @note Flash size gated: Disabled on small boards to conserve memory
 * @note Requires ADC input pins
 */
#ifndef AP_BATTERY_FUELLEVEL_ANALOG_ENABLED
#define AP_BATTERY_FUELLEVEL_ANALOG_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * @brief Generator integration for battery charging monitoring
 * 
 * Enables monitoring battery charging from onboard generators. Tracks generator
 * output, charging current, and battery state during generator operation for
 * long-endurance hybrid-electric systems.
 * 
 * @note Default: Enabled when generator support is compiled in
 * @note Requires: HAL_GENERATOR_ENABLED
 * @see libraries/AP_Generator/AP_Generator_config.h
 */
#ifndef AP_BATTERY_GENERATOR_ENABLED
#define AP_BATTERY_GENERATOR_ENABLED HAL_GENERATOR_ENABLED
#endif

/**
 * @brief INA239 SPI power monitor
 * 
 * Enables Texas Instruments INA239 high-precision power monitor via SPI interface.
 * Provides 16-bit voltage/current measurements with programmable shunt resistor
 * configuration. Requires explicit SPI bus/CS definition in board hwdef.
 * 
 * @note Default: 0 (disabled) - Must be explicitly enabled in hwdef.dat
 * @note Requires: SPI device specification with SPIDEV INA239 in hwdef
 * @note Higher precision than I2C variants, suitable for lab power measurement
 */
#ifndef AP_BATTERY_INA239_ENABLED
#define AP_BATTERY_INA239_ENABLED 0  // SPI device must be specified in hwdef
#endif

/**
 * @brief INA2xx I2C power monitor family
 * 
 * Enables Texas Instruments INA2xx series power monitors via I2C (INA219, INA226,
 * etc.). Widely-used current/voltage/power monitors with integrated shunt measurement.
 * Auto-detects specific INA2xx variant and configures appropriately.
 * 
 * @note Default: Enabled on boards with >1MB flash
 * @note Flash size gated: Disabled on small boards to conserve memory
 * @note Supports: INA219, INA226, and other INA2xx family devices
 */
#ifndef AP_BATTERY_INA2XX_ENABLED
#define AP_BATTERY_INA2XX_ENABLED (AP_BATTERY_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024))
#endif

/**
 * @brief INA3221 triple-channel I2C power monitor
 * 
 * Enables Texas Instruments INA3221 three-channel current/voltage monitor.
 * Monitors three independent power rails simultaneously over single I2C interface.
 * Requires I2C bus/address configuration in hwdef, enabled for SITL testing.
 * 
 * @note Default: Enabled in hwdef.dat files or SITL
 * @note Requires: I2C configuration specified in board hwdef
 * @note Useful for multi-rail power monitoring (5V, servo rail, payload power)
 */
#ifndef AP_BATTERY_INA3221_ENABLED
// turned on in hwdefs (except for sim test), requires config
#define AP_BATTERY_INA3221_ENABLED (AP_BATTERY_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

/**
 * @brief LTC2946 I2C power/energy monitor
 * 
 * Enables Linear Technology/Analog Devices LTC2946 high-accuracy power monitor
 * with integrated energy accumulation. Requires HAL_BATTMON_LTC2946_BUS and
 * HAL_BATTMON_LTC2946_ADDR definitions in hwdef for I2C bus/address configuration.
 * 
 * @note Default: Enabled when HAL_BATTMON_LTC2946_BUS and ADDR defined in hwdef
 * @note Requires: Explicit I2C bus and address macros in board hwdef.dat
 * @note Features: 12-bit ADC, power/energy/charge accumulation, high precision
 */
#ifndef AP_BATTERY_LTC2946_ENABLED
#define AP_BATTERY_LTC2946_ENABLED (AP_BATTERY_BACKEND_DEFAULT_ENABLED && defined(HAL_BATTMON_LTC2946_BUS) && defined(HAL_BATTMON_LTC2946_ADDR))
#endif

/**
 * @brief Synthetic current estimation from voltage and throttle
 * 
 * Enables estimating battery current draw based on voltage sag and throttle
 * position when direct current sensing is unavailable. Uses battery voltage
 * drop and motor throttle to infer approximate current consumption.
 * 
 * @note Default: Enabled (via AP_BATTERY_BACKEND_DEFAULT_ENABLED)
 * @note Useful for: Systems with voltage sensing only, no current sensor
 * @warning Less accurate than direct current measurement, for estimation only
 */
#ifndef AP_BATTERY_SYNTHETIC_CURRENT_ENABLED
#define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED
#endif

/**
 * @brief 3DR Solo smart battery support
 * 
 * Enables specialized protocol for 3DR Solo smart battery packs with proprietary
 * SMBus extensions. Includes Solo-specific battery health monitoring, cell
 * balancing status, and authentication. Must be enabled in hwdef for Solo builds.
 * 
 * @note Default: 0 (disabled) - Must be explicitly enabled in hwdef.dat
 * @note Platform-specific: Only for 3DR Solo autopilot hardware
 * @note Requires: SMBus base support (AP_BATTERY_SMBUS_ENABLED)
 */
#ifndef AP_BATTERY_SMBUS_SOLO_ENABLED
#define AP_BATTERY_SMBUS_SOLO_ENABLED 0  // turned on in hwdefs
#endif

/**
 * @brief Multi-battery aggregation and summation
 * 
 * Enables virtual battery monitor that sums voltage/current/capacity from multiple
 * physical battery monitors. Useful for parallel battery packs or computing total
 * system power across multiple independent battery sources.
 * 
 * @note Default: Enabled on boards with >1MB flash
 * @note Flash size gated: Disabled on small boards to conserve memory
 * @note Aggregates: Voltage (average), current (sum), capacity (sum)
 */
#ifndef AP_BATTERY_SUM_ENABLED
#define AP_BATTERY_SUM_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * @brief Torqeedo electric outboard motor battery monitoring
 * 
 * Enables battery monitoring integration with Torqeedo electric marine propulsion
 * systems. Reads battery status from Torqeedo motor controllers for marine
 * applications (ArduRover-boat, ArduSub).
 * 
 * @note Default: Enabled when Torqeedo support compiled in
 * @note Requires: HAL_TORQEEDO_ENABLED
 * @see libraries/AP_Torqeedo/AP_Torqeedo_config.h
 */
#ifndef AP_BATTERY_TORQEEDO_ENABLED
#define AP_BATTERY_TORQEEDO_ENABLED HAL_TORQEEDO_ENABLED
#endif

/**
 * @brief AD7091R5 I2C 12-bit ADC for voltage/current sensing
 * 
 * Enables Analog Devices AD7091R5 4-channel 12-bit I2C ADC as battery monitor
 * frontend. Provides multi-channel analog sensing with I2C interface for boards
 * with limited native ADC inputs.
 * 
 * @note Default: Enabled on boards with >1MB flash
 * @note Flash size gated: Disabled on small boards to conserve memory
 * @note Useful for: Expanding analog sensing on I2C-capable boards
 */
#ifndef AP_BATTERY_AD7091R5_ENABLED 
#define AP_BATTERY_AD7091R5_ENABLED AP_BATTERY_BACKEND_DEFAULT_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

/**
 * SMBus-subclass backends: Vendor-specific smart battery implementations
 * 
 * The following backends implement vendor-specific SMBus protocols, all deriving
 * from the base AP_BATTERY_SMBUS_ENABLED support. Each handles manufacturer-specific
 * command extensions, register layouts, or authentication schemes.
 */

/**
 * @brief Generic SMBus smart battery protocol
 * 
 * Enables standard Smart Battery System specification implementation with common
 * SMBus commands (voltage, current, capacity, temperature, status). Supports most
 * SBS-compliant smart batteries without vendor-specific extensions. Base class for
 * Maxell and Rotoye variants.
 * 
 * @note Default: Enabled when SMBus base support enabled
 * @note Requires: AP_BATTERY_SMBUS_ENABLED
 * @note Compatibility: Standard SBS 1.1 smart batteries
 */
#ifndef AP_BATTERY_SMBUS_GENERIC_ENABLED
#define AP_BATTERY_SMBUS_GENERIC_ENABLED AP_BATTERY_SMBUS_ENABLED
#endif

/**
 * @brief NeoDesign SMBus battery protocol
 * 
 * Enables NeoDesign-specific SMBus battery protocol with vendor extensions for
 * enhanced diagnostics and cell monitoring. Used with NeoDesign smart battery packs.
 * 
 * @note Default: Enabled when SMBus base support enabled
 * @note Requires: AP_BATTERY_SMBUS_ENABLED
 * @note Vendor-specific: NeoDesign battery packs
 */
#ifndef AP_BATTERY_SMBUS_NEODESIGN_ENABLED
#define AP_BATTERY_SMBUS_NEODESIGN_ENABLED AP_BATTERY_SMBUS_ENABLED
#endif

/**
 * @brief SUI (Smart Battery Systems) SMBus protocol
 * 
 * Enables SUI (Smart Battery Systems) vendor-specific SMBus protocol extensions.
 * Supports SUI smart battery packs with manufacturer-specific commands.
 * 
 * @note Default: Enabled when SMBus base support enabled
 * @note Requires: AP_BATTERY_SMBUS_ENABLED
 * @note Vendor-specific: SUI smart battery packs
 */
#ifndef AP_BATTERY_SMBUS_SUI_ENABLED
#define AP_BATTERY_SMBUS_SUI_ENABLED AP_BATTERY_SMBUS_ENABLED
#endif

/**
 * Subclasses of the SMBus Generic backend: Specialized variants
 * 
 * The following backends derive from AP_BATTERY_SMBUS_GENERIC, inheriting standard
 * SBS command support while adding manufacturer-specific quirks or extensions.
 */

/**
 * @brief Maxell SMBus battery protocol
 * 
 * Enables Maxell-specific SMBus battery support, derived from Generic backend.
 * Handles Maxell battery pack quirks and manufacturer-specific registers while
 * maintaining standard SBS command compatibility.
 * 
 * @note Default: Enabled when Generic SMBus backend enabled
 * @note Requires: AP_BATTERY_SMBUS_GENERIC_ENABLED
 * @note Inheritance: Derives from Generic SMBus implementation
 * @note Vendor-specific: Maxell smart battery packs
 */
#ifndef AP_BATTERY_SMBUS_MAXELL_ENABLED
#define AP_BATTERY_SMBUS_MAXELL_ENABLED AP_BATTERY_SMBUS_GENERIC_ENABLED
#endif

/**
 * @brief Rotoye Batmon SMBus battery protocol
 * 
 * Enables Rotoye Batmon smart battery support, derived from Generic backend.
 * Rotoye Batmon provides SBS-compatible smart battery functionality with enhanced
 * cell monitoring and diagnostics. Maintains Generic command set with extensions.
 * 
 * @note Default: Enabled when Generic SMBus backend enabled
 * @note Requires: AP_BATTERY_SMBUS_GENERIC_ENABLED
 * @note Inheritance: Derives from Generic SMBus implementation
 * @note Vendor-specific: Rotoye Batmon smart battery system
 */
#ifndef AP_BATTERY_SMBUS_ROTOYE_ENABLED
#define AP_BATTERY_SMBUS_ROTOYE_ENABLED AP_BATTERY_SMBUS_GENERIC_ENABLED
#endif

/**
 * @brief Lua scripting interface for custom battery drivers
 * 
 * Enables battery monitoring backend that receives data from Lua scripts. Allows
 * implementation of custom battery protocols or integration with unsupported
 * hardware entirely in Lua without C++ code changes. Scripts call battery:handle_script_msg()
 * to update voltage, current, capacity, and other battery state.
 * 
 * @note Default: Enabled when both scripting and battery monitoring available
 * @note Requires: AP_SCRIPTING_ENABLED and AP_BATTERY_BACKEND_DEFAULT_ENABLED
 * @see libraries/AP_Scripting for Lua scripting documentation
 * @see libraries/AP_Scripting/examples for battery monitoring script examples
 */
#ifndef AP_BATTERY_SCRIPTING_ENABLED
#define AP_BATTERY_SCRIPTING_ENABLED (AP_SCRIPTING_ENABLED && AP_BATTERY_BACKEND_DEFAULT_ENABLED)
#endif

/**
 * @brief Battery options parameter availability
 * 
 * Controls compilation of BATT_OPTIONS parameter for configuring optional battery
 * monitor behaviors (ignore DroneCAN SoC, MPPT power off, etc.). Enabled for main
 * autopilot builds, and for AP_Periph builds that include battery sum support.
 * 
 * @note Default: Enabled except on minimal AP_Periph builds without Sum backend
 * @note AP_Periph: Only enabled if AP_BATTERY_SUM_ENABLED for multi-battery peripherals
 * @note Saves: Parameter table space on minimal peripheral builds
 */
#ifndef AP_BATTERY_OPTIONS_PARAM_ENABLED
#define AP_BATTERY_OPTIONS_PARAM_ENABLED (!defined(HAL_BUILD_AP_PERIPH) || AP_BATTERY_SUM_ENABLED)
#endif
