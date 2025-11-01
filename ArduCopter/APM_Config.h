/**
 * @file APM_Config.h
 * @brief User-customizable configuration override file for ArduCopter
 * 
 * @details This file serves as a user-specific configuration template that allows 
 *          local customization of ArduCopter build-time defaults without modifying 
 *          version-controlled source files. Any configuration option defined in 
 *          config.h can be overridden here by uncommenting and modifying the 
 *          relevant #define directives.
 * 
 * ## Purpose and Usage Pattern
 * 
 * This file is intended for:
 * - **Local Development Customization**: Developers can tailor the firmware build
 *   to their specific testing needs without affecting the repository
 * - **Flash Space Optimization**: Disable unused features to reduce firmware size
 *   on memory-constrained flight controllers
 * - **Feature Testing**: Temporarily enable/disable features during development
 * - **Custom Code Integration**: Hook custom user code into the main flight loop
 * 
 * ## Recommended Workflow
 * 
 * 1. **Copy and Customize**: Copy this file to your local workspace if needed
 * 2. **Override Selectively**: Uncomment only the specific #defines you want to change
 * 3. **Rebuild Firmware**: Recompile ArduCopter to apply your configuration changes
 * 4. **Version Control**: This file can be added to .gitignore for local-only customizations
 * 
 * ## Common Customization Examples
 * 
 * ### Example 1: Disable Logging to Save Flash Space
 * ```cpp
 * #define LOGGING_ENABLED 0  // Saves approximately 11KB on APM2 (more on Pixhawk)
 * ```
 * 
 * ### Example 2: Disable Unused Flight Modes
 * ```cpp
 * #define MODE_FLIP_ENABLED 0     // Disable flip mode
 * #define MODE_DRIFT_ENABLED 0    // Disable drift mode
 * #define MODE_SPORT_ENABLED 0    // Disable sport mode
 * ```
 * 
 * ### Example 3: Enable Advanced Failsafe
 * ```cpp
 * #define AP_COPTER_ADVANCED_FAILSAFE_ENABLED 1  // Enable mission continuation in failsafe
 * ```
 * 
 * ### Example 4: Integrate Custom User Code
 * ```cpp
 * #define USERHOOK_VARIABLES "UserVariables.h"
 * #define USERHOOK_INIT userhook_init();
 * #define USERHOOK_FASTLOOP userhook_FastLoop();  // Run custom code at 100Hz
 * ```
 * 
 * ## Configuration Override Mechanism
 * 
 * The ArduCopter build system processes configuration files in this order:
 * 1. **config.h** - Default configuration values for all features
 * 2. **APM_Config.h** (this file) - User overrides applied last
 * 
 * Because this file is included after config.h, any #define directives here will
 * take precedence over the defaults. The C preprocessor will use the last
 * definition encountered.
 * 
 * ## Flash Space Considerations
 * 
 * The flash savings estimates provided in comments (e.g., "save 11K") are
 * approximate and based on APM2 boards. Actual savings will vary by:
 * - Target hardware platform (Pixhawk, CubeOrange, etc.)
 * - Compiler version and optimization settings
 * - Other enabled features and their interactions
 * - Overall firmware size and memory layout
 * 
 * Modern flight controllers with larger flash (1-2MB) typically don't require
 * aggressive feature disabling unless building highly customized configurations.
 * 
 * @warning **SAFETY-CRITICAL CONFIGURATION WARNINGS**
 * 
 * @warning Disabling certain features can significantly impact flight safety:
 * - **MODE_RTL_ENABLED**: Disabling Return-to-Launch removes critical failsafe capability
 * - **LOGGING_ENABLED**: Disabling logging prevents post-flight analysis and crash investigation
 * - **MODE_LAND_ENABLED**: Disabling land mode removes controlled descent capability
 * - **GPS-dependent modes**: Disabling GPS modes on GPS-equipped vehicles limits recovery options
 * 
 * @warning Changing timing-related settings can affect flight stability:
 * - **THROTTLE_IN_DEADBAND**: Incorrect values can cause unexpected throttle behavior
 * - **Main loop rates**: Never modify loop rates without deep understanding of control implications
 * 
 * @warning Custom user hooks execute within flight-critical code paths:
 * - **USERHOOK_FASTLOOP**: Runs at 100Hz in main control loop - keep execution time minimal (<1ms)
 * - **Long-running code**: Can cause scheduler overruns and degraded flight performance
 * - **Blocking operations**: Never block in user hooks (no delays, no waiting for I/O)
 * - **Memory allocation**: Avoid dynamic memory allocation in user hooks
 * 
 * @warning Feature interaction considerations:
 * - Some features depend on others (e.g., SmartRTL depends on logging)
 * - Disabling a dependency may cause build failures or runtime errors
 * - Test thoroughly in SITL simulation before flying with custom configurations
 * 
 * @note **Testing Recommendations**
 * - Always test configuration changes in SITL (Software-In-The-Loop) simulation first
 * - Verify all required flight modes remain functional after disabling features
 * - Confirm failsafe behavior works as expected with your configuration
 * - Monitor CPU load and scheduler timing with custom user hooks enabled
 * 
 * @note **Build System Integration**
 * - This file is included by config.h during the preprocessing stage
 * - Changes require a full rebuild: `./waf clean && ./waf copter`
 * - Configuration applies only to ArduCopter vehicle type (not Plane, Rover, etc.)
 * 
 * @note **Parameter System vs Build-Time Configuration**
 * - This file controls build-time feature compilation (what code is included)
 * - Runtime parameters (tuning, limits, behavior) are configured via the parameter system
 * - Disabled features cannot be re-enabled without recompiling firmware
 * 
 * @see config.h for the complete list of configurable options and default values
 * @see UserCode.cpp for implementing custom user hook functions
 * @see UserVariables.h for defining custom user variables
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org. Licensed under GPLv3.
 */

// User specific config file.  Any items listed in config.h can be overridden here.

// uncomment the lines below to disable features (flash sizes listed are for APM2 boards and will underestimate savings on Pixhawk and other boards)
//#define LOGGING_ENABLED       0            // disable logging to save 11K of flash space
//#define MOUNT                 0            // disable the camera gimbal to save 8K of flash space
//#define AUTOTUNE_ENABLED      0            // disable the auto tune functionality to save 7k of flash
//#define NAV_GUIDED            0            // disable external navigation computer ability to control vehicle through MAV_CMD_NAV_GUIDED mission commands
//#define MODE_ACRO_ENABLED     0            // disable acrobatic mode support
//#define MODE_AUTO_ENABLED     0            // disable auto mode support
//#define MODE_BRAKE_ENABLED    0            // disable brake mode support
//#define MODE_CIRCLE_ENABLED   0            // disable circle mode support
//#define MODE_DRIFT_ENABLED    0            // disable drift mode support
//#define MODE_FLIP_ENABLED     0            // disable flip mode support
//#define MODE_FOLLOW_ENABLED   0            // disable follow mode support
//#define MODE_GUIDED_ENABLED   0            // disable guided mode support
//#define MODE_GUIDED_NOGPS_ENABLED   0      // disable guided/nogps mode support
//#define MODE_LOITER_ENABLED   0            // disable loiter mode support
//#define MODE_POSHOLD_ENABLED  0            // disable poshold mode support
//#define MODE_RTL_ENABLED      0            // disable rtl mode support
//#define MODE_SMARTRTL_ENABLED 0            // disable smartrtl mode support
//#define MODE_SPORT_ENABLED    0            // disable sport mode support
//#define MODE_SYSTEMID_ENABLED 0            // disable system ID mode support
//#define MODE_THROW_ENABLED    0            // disable throw mode support
//#define MODE_ZIGZAG_ENABLED   0            // disable zigzag mode support
//#define OSD_ENABLED           0            // disable on-screen-display support

// features below are disabled by default on all boards
//#define CAL_ALWAYS_REBOOT                         // flight controller will reboot after compass or accelerometer calibration completes
//#define DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE   // disable mode changes from GCS during Radio failsafes.  Avoids a race condition for vehicle like Solo in which the RC and telemetry travel along the same link
//#define AP_COPTER_ADVANCED_FAILSAFE_ENABLED     1             // enabled advanced failsafe which allows running a portion of the mission in failsafe events

// other settings
//#define THROTTLE_IN_DEADBAND   100                // redefine size of throttle deadband in pwm (0 ~ 1000)

// User Hooks : For User Developed code that you wish to run
// Put your variable definitions into the UserVariables.h file (or another file name and then change the #define below).
//#define USERHOOK_VARIABLES "UserVariables.h"
// Put your custom code into the UserCode.cpp with function names matching those listed below and ensure the appropriate #define below is uncommented below
//#define USERHOOK_INIT userhook_init();                      // for code to be run once at startup
//#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
//#define USERHOOK_50HZLOOP userhook_50Hz();                  // for code to be run at 50hz
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();            // for code to be run at 3.3hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz
//#define USERHOOK_AUXSWITCH 1                        // for code to handle user aux switches
//#define USER_PARAMS_ENABLED 1                       // to enable user parameters
