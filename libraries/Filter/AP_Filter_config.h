/**
 * @file AP_Filter_config.h
 * @brief Compile-time configuration for ArduPilot filter subsystem
 * 
 * @details This configuration header controls the availability and capacity of the
 *          AP_Filter subsystem, which provides runtime-configurable digital filters
 *          for sensor data processing, signal conditioning, and noise reduction.
 *          
 *          Configuration is automatically adjusted based on board program size limits
 *          to optimize memory usage across different hardware platforms. The file
 *          defines two primary configuration macros:
 *          
 *          - AP_FILTER_ENABLED: Master enable/disable for the entire subsystem
 *          - AP_FILTER_NUM_FILTERS: Number of configurable filter slots
 *          
 *          Build System Integration:
 *          The configuration uses HAL_PROGRAM_SIZE_LIMIT_KB to automatically determine
 *          appropriate defaults based on available program memory. This ensures optimal
 *          feature availability without exceeding flash storage constraints.
 *          
 *          Override Capability:
 *          All configuration values can be overridden via compiler flags for custom
 *          builds. For example: -DAP_FILTER_ENABLED=0 or -DAP_FILTER_NUM_FILTERS=16
 *          
 * @note This file controls the AP_Filter subsystem (runtime configurable filters)
 *       and does NOT affect core filter primitives (LowPassFilter, NotchFilter, etc.)
 *       which are always available. It also does NOT affect HarmonicNotchFilter
 *       which has separate configuration.
 * 
 * @see AP_Filter.h for the main filter subsystem implementation
 * @see Filter.h for core filter primitives
 * 
 * Source: libraries/Filter/AP_Filter_config.h:1-18
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @def AP_FILTER_ENABLED
 * @brief Master enable/disable for the entire AP_Filter subsystem
 * 
 * @details Controls whether the AP_Filter subsystem is compiled into the firmware.
 *          When enabled, provides runtime-configurable digital filters accessible
 *          through ground station parameters (FILT1_TYPE, FILT2_TYPE, etc.).
 *          
 *          Default Behavior:
 *          - Enabled (1) on boards with >1024KB program size limit
 *          - Disabled (0) on boards with ≤1024KB program size limit
 *          
 *          Effect When Enabled (AP_FILTER_ENABLED=1):
 *          - AP_Filter.h and related classes are included in compilation
 *          - Filter parameter tables are registered in EEPROM
 *          - Ground station can configure AP_FILTER_NUM_FILTERS filter slots
 *          - Adds approximately 10-20KB to program size depending on configuration
 *          
 *          Effect When Disabled (AP_FILTER_ENABLED=0):
 *          - Entire AP_Filter subsystem excluded from compilation
 *          - No filter parameters appear in ground station
 *          - Reduces code size for memory-constrained boards
 *          - Core filter primitives (LowPassFilter, NotchFilter) remain available
 *          
 *          Board Size Categories:
 *          - >1024KB: High-end boards (Pixhawk 4, Cube Orange, etc.) - Full support
 *          - ≤1024KB: Mid-range and low-end boards - Automatic feature reduction
 *          
 *          Override Usage:
 *          Define via compiler flag to force enable/disable regardless of board size:
 *          - Force enable:  -DAP_FILTER_ENABLED=1
 *          - Force disable: -DAP_FILTER_ENABLED=0
 *          
 * @note This only affects the AP_Filter subsystem. Core filter primitives used
 *       throughout ArduPilot (LowPassFilter, NotchFilter, etc.) are always available.
 *       HarmonicNotchFilter has separate configuration and is not affected.
 * 
 * @warning Disabling this feature removes all runtime-configurable filter functionality.
 *          Code that depends on AP_Filter must check AP_FILTER_ENABLED before use.
 * 
 * @see AP_FILTER_NUM_FILTERS for filter slot count configuration
 * @see AP_Filter.h for subsystem implementation
 */
#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

/**
 * @def AP_FILTER_NUM_FILTERS
 * @brief Number of configurable filter slots in the AP_Filters manager
 * 
 * @details Determines how many independent filter instances can be configured
 *          at runtime through ground station parameters. Each filter slot can
 *          be independently configured with different filter types and parameters.
 *          
 *          This macro is only defined when AP_FILTER_ENABLED=1. When the filter
 *          subsystem is disabled, no filter slots are allocated.
 *          
 *          Default Values (automatically selected based on board size):
 *          - 8 filters: Boards with >1024KB program size (e.g., STM32H7, Pixhawk 4)
 *          - 4 filters: Boards with ≤1024KB program size (e.g., STM32F4, Pixhawk 1)
 *          
 *          Effect on System:
 *          Determines the size of:
 *          - AP_Filters::filters[] array (runtime filter instances)
 *          - AP_Filters::params[] array (parameter storage)
 *          - Parameter table entries (FILT1_* through FILT<N>_*)
 *          
 *          Memory Impact:
 *          Each filter slot consumes:
 *          - EEPROM: ~20-30 bytes for parameter storage
 *          - RAM: ~40-60 bytes for filter state (varies by filter type)
 *          - Flash: Negligible (array size only, code shared)
 *          
 *          Parameter Naming:
 *          Filter slots are exposed as ground station parameters:
 *          - FILT1_TYPE, FILT1_FREQ, FILT1_BW, etc. (filter 1)
 *          - FILT2_TYPE, FILT2_FREQ, FILT2_BW, etc. (filter 2)
 *          - ...
 *          - FILT<N>_TYPE, FILT<N>_FREQ, FILT<N>_BW, etc. (filter N)
 *          
 *          Where N = AP_FILTER_NUM_FILTERS
 *          
 *          Board Size Categories:
 *          - >1024KB: High-end boards (Pixhawk 4, Cube Orange, etc.) - 8 filter slots
 *          - ≤1024KB: Mid-range boards (Pixhawk 1, Cube Black, etc.) - 4 filter slots
 *          - <1024KB with AP_FILTER_ENABLED=0: Low-end boards - no filter subsystem
 *          
 *          Override Usage:
 *          Define via compiler flag for custom filter count:
 *          - Increase slots: -DAP_FILTER_NUM_FILTERS=16
 *          - Decrease slots: -DAP_FILTER_NUM_FILTERS=2
 *          
 *          Conditional Compilation:
 *          This macro is only defined when AP_FILTER_ENABLED=1. If AP_FILTER_ENABLED=0,
 *          the entire filter subsystem is excluded and no filter slots are allocated.
 *          
 * @warning Changing AP_FILTER_NUM_FILTERS affects the parameter table layout.
 *          If you decrease the count, existing parameters for higher-numbered filters
 *          (e.g., FILT5_* through FILT8_* when reducing from 8 to 4) may become invalid.
 *          Users should reset parameters or back up settings after changing filter count.
 * 
 * @note This only controls the number of AP_Filter subsystem filter slots.
 *       Core filter primitives (LowPassFilter, NotchFilter) and HarmonicNotchFilter
 *       are configured separately and are not affected by this setting.
 * 
 * @see AP_FILTER_ENABLED for subsystem enable/disable
 * @see AP_Filters class for filter manager implementation
 */
#if AP_FILTER_ENABLED
 #ifndef AP_FILTER_NUM_FILTERS
 #if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
  #define AP_FILTER_NUM_FILTERS 8
 #else
  #define AP_FILTER_NUM_FILTERS 4
 #endif // HAL_PROGRAM_SIZE_LIMIT_KB
 #endif // AP_FILTER_NUM_FILTERS
#endif // AP_FILTER_ENABLED
