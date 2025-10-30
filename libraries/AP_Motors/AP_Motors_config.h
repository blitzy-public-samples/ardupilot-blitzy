/**
 * @file AP_Motors_config.h
 * @brief Compile-time configuration and feature flags for the AP_Motors library
 * 
 * @details This configuration header defines compile-time constants, feature enablement
 *          macros, and platform-specific motor count limits for the AP_Motors library.
 *          
 *          This file establishes:
 *          - Maximum number of motors per vehicle (AP_MOTORS_MAX_NUM_MOTORS)
 *          - Feature enablement flags (AP_MOTORS_*_ENABLED)
 *          - Frame type availability (AP_MOTORS_FRAME_*_ENABLED)
 *          - Platform-dependent motor output limits
 *          
 *          This header is processed first via forced include in AP_Motors.h before
 *          all other AP_Motors headers, ensuring configuration constants are available
 *          throughout the library.
 * 
 * @note This file is processed before all other AP_Motors headers via forced include
 * @note Changes to this file require a full rebuild of ArduPilot
 * @note Feature flags control frame type availability and affect flash usage
 * @note Board-specific limits in hwdef files may further restrict motor counts
 * 
 * @warning Modifying AP_MOTORS_MAX_NUM_MOTORS affects memory usage
 * @warning Feature flags must match vehicle configuration requirements
 * 
 * Related Configuration:
 * - SRV_Channel library provides underlying servo/motor output capabilities
 * - HAL (Hardware Abstraction Layer) determines physical output limits
 * - hwdef files (AP_HAL_ChibiOS/hwdef/*.hwdef) configure per-board output counts
 * 
 * Source: libraries/AP_Motors/AP_Motors_config.h
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <SRV_Channel/SRV_Channel_config.h>

/**
 * @brief Maximum number of motors per vehicle
 * 
 * @details Defines the compile-time limit for motor outputs that can be controlled
 *          by the AP_Motors library. This value is automatically adjusted based on:
 *          
 *          1. Scripting Support: 32 motors if AP_Scripting enabled, 12 otherwise
 *          2. Hardware Limits: Clamped to NUM_SERVO_CHANNELS (board-specific)
 *          3. Backend Requirements: Minimum of 12 motors for compatibility
 *          
 *          Typical configurations:
 *          - Standard multicopters: 4-8 motors
 *          - High motor-count frames (DodecaHexa): 12 motors
 *          - Scripting-enabled configurations: Up to 32 motors
 *          - ArduSub 6DOF: 8 thrusters
 *          
 *          The limit is platform-dependent, restricted by the number of servo output
 *          channels available on the hardware. Check NUM_SERVO_CHANNELS for your board
 *          in the corresponding hwdef file.
 * 
 * @note This value is clamped to NUM_SERVO_CHANNELS to match hardware capabilities
 * @note Minimum value is 12 to maintain compatibility with backends like AP_Motors6DOF
 * @note Increasing this value increases memory usage for motor output arrays
 * 
 * @warning Platform-specific: Verify NUM_SERVO_CHANNELS for your target board
 * @warning Changing this value affects memory allocation throughout the motors library
 * 
 * @see SRV_Channel for underlying servo output implementation
 * @see NUM_SERVO_CHANNELS in AP_HAL/AP_HAL_Boards.h for board-specific limits
 */
#ifndef AP_MOTORS_MAX_NUM_MOTORS
#if AP_SCRIPTING_ENABLED
#define AP_MOTORS_MAX_NUM_MOTORS 32
#else
#define AP_MOTORS_MAX_NUM_MOTORS 12
#endif

// doesn't make sense to have more motors than servo channels, so clamp:
#if NUM_SERVO_CHANNELS < AP_MOTORS_MAX_NUM_MOTORS
#undef AP_MOTORS_MAX_NUM_MOTORS
#define AP_MOTORS_MAX_NUM_MOTORS NUM_SERVO_CHANNELS
#endif

// various Motors backends will not compile if we don't have 16 motors
// available (eg. AP_Motors6DOF).  Until we stop compiling those
// backends in when there aren't enough motors to support those
// backends we will support a minimum of 12 motors, the limit before
// we moved to 32 motor support:
#if AP_MOTORS_MAX_NUM_MOTORS < 12
#undef AP_MOTORS_MAX_NUM_MOTORS
#define AP_MOTORS_MAX_NUM_MOTORS 12
#endif

#endif  // defined (AP_MOTORS_MAX_NUM_MOTORS)

/**
 * @brief Enable tricopter frame support
 * 
 * @details Controls compilation of AP_MotorsTri backend for tricopter frames.
 *          Tricopters use three motors with a servo-actuated rear motor for yaw control.
 *          
 *          When enabled (1): Tricopter frame type available in FRAME_CLASS parameter
 *          When disabled (0): Tricopter code excluded, reducing flash usage
 * 
 * @note Disabling unused frame types reduces flash usage on constrained boards
 * @note Set to 0 in board hwdef files to disable tricopter support
 * 
 * @see AP_MotorsTri class for tricopter implementation details
 */
#ifndef AP_MOTORS_TRI_ENABLED
#define AP_MOTORS_TRI_ENABLED 1
#endif  // AP_MOTORS_TRI_ENABLED

/**
 * @brief Default enablement state for multicopter frame types
 * 
 * @details Master enable/disable flag for all standard multicopter frame types.
 *          Individual frame types inherit this default unless explicitly overridden.
 *          
 *          Set to 0 in board hwdef files to disable all frame types by default,
 *          then selectively enable only required frames to minimize flash usage.
 * 
 * @note All AP_MOTORS_FRAME_*_ENABLED flags default to this value
 * @note Useful for resource-constrained boards to disable all frames, then enable specific ones
 * 
 * @see Individual AP_MOTORS_FRAME_*_ENABLED flags below
 */
#ifndef AP_MOTORS_FRAME_DEFAULT_ENABLED
#define AP_MOTORS_FRAME_DEFAULT_ENABLED 1
#endif

/**
 * @brief Enable quadcopter frame support (4 motors)
 * 
 * @details Quadcopter frame with 4 motors in X or + configuration.
 *          Most common multicopter configuration.
 * 
 * @note Disable to save flash on boards that don't support quadcopters
 */
#ifndef AP_MOTORS_FRAME_QUAD_ENABLED
#define AP_MOTORS_FRAME_QUAD_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

/**
 * @brief Enable hexacopter frame support (6 motors)
 * 
 * @details Hexacopter frame with 6 motors arranged in X or + configuration.
 *          Provides redundancy and higher payload capacity than quadcopters.
 * 
 * @note Disable to save flash if hexacopter support is not required
 */
#ifndef AP_MOTORS_FRAME_HEXA_ENABLED
#define AP_MOTORS_FRAME_HEXA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

/**
 * @brief Enable octocopter frame support (8 motors)
 * 
 * @details Octocopter frame with 8 motors in X or + configuration.
 *          Provides maximum redundancy and payload capacity for standard frames.
 * 
 * @note Disable to save flash if octocopter support is not required
 */
#ifndef AP_MOTORS_FRAME_OCTA_ENABLED
#define AP_MOTORS_FRAME_OCTA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

/**
 * @brief Enable decacopter frame support (10 motors)
 * 
 * @details Decacopter frame with 10 motors for very high payload applications.
 *          Uncommon configuration for specialized heavy-lift vehicles.
 * 
 * @note Disable to save flash if decacopter support is not required
 */
#ifndef AP_MOTORS_FRAME_DECA_ENABLED
#define AP_MOTORS_FRAME_DECA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

/**
 * @brief Enable dodeca-hexa frame support (12 motors)
 * 
 * @details Dodeca-hexacopter frame with 12 motors arranged in 6 coaxial pairs.
 *          Maximum standard frame motor count for extreme heavy-lift applications.
 *          Requires AP_MOTORS_MAX_NUM_MOTORS >= 12.
 * 
 * @note Disable to save flash if dodeca-hexa support is not required
 * @note Requires at least 12 motor outputs available on the board
 */
#ifndef AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#define AP_MOTORS_FRAME_DODECAHEXA_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

/**
 * @brief Enable Y6 frame support (6 motors in coaxial configuration)
 * 
 * @details Y6 frame with 6 motors arranged in 3 coaxial pairs in Y configuration.
 *          Provides compact hexacopter performance with reduced footprint.
 * 
 * @note Disable to save flash if Y6 support is not required
 */
#ifndef AP_MOTORS_FRAME_Y6_ENABLED
#define AP_MOTORS_FRAME_Y6_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif

/**
 * @brief Enable octa-quad frame support (8 motors in quadcopter layout)
 * 
 * @details Octa-quad frame with 8 motors arranged in 4 coaxial pairs in X configuration.
 *          Combines octocopter redundancy with quadcopter footprint.
 * 
 * @note Disable to save flash if octa-quad support is not required
 */
#ifndef AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#define AP_MOTORS_FRAME_OCTAQUAD_ENABLED AP_MOTORS_FRAME_DEFAULT_ENABLED
#endif
