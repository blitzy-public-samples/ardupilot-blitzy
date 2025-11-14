/**
 * @file SRV_Channel_config.h
 * @brief Compile-time configuration for servo/auxiliary output system
 * 
 * @details Defines constants controlling servo channel count, actuator allocation, and
 * feature enablement. Board-specific headers can override these defaults before
 * this file is included to adapt channel counts for memory-constrained boards.
 * 
 * Configuration hierarchy:
 * - Board hwdef files can set NUM_SERVO_CHANNELS based on hardware capabilities
 * - AP_HAL_Boards.h processes board-specific definitions
 * - This file provides defaults if not already defined
 * 
 * Memory implications:
 * - Each channel requires approximately 64 bytes (parameters, state, overrides)
 * - 32 channels = ~2KB RAM, 16 channels = ~1KB RAM
 * - Channels can be disabled individually via SERVOn_FUNCTION=0 to conserve processing time
 * 
 * Relationship to parameters:
 * - SERVOn_FUNCTION: Maps channel n to output function (motor, aileron, gripper, etc.)
 * - SERVOn_MIN/MAX/TRIM: PWM pulse width limits for channel n (typically 1000-2000µs)
 * - RCn_OUTPUT: Hardware pin assignment for servo channel n
 * 
 * @note This header must be included before SRV_Channel.h
 * @note Changes here affect memory allocation for entire output system
 * @warning Increasing NUM_SERVO_CHANNELS beyond board capabilities causes memory exhaustion
 * 
 * Source: libraries/SRV_Channel/SRV_Channel_config.h
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @def AP_SRV_CHANNELS_ENABLED
 * @brief Master enable flag for servo channel subsystem
 * 
 * @details Set to 0 to completely disable servo/actuator output system. This is extremely
 * rare and used only for minimal builds or non-vehicle applications where no PWM outputs
 * are required (e.g., pure sensor nodes, ground station relay devices).
 * 
 * When disabled:
 * - SRV_Channel class and all methods become no-ops
 * - ~10KB flash space saved on typical builds
 * - All SERVOn_* parameters hidden from parameter list
 * - Vehicle code must handle missing output capability gracefully
 * 
 * Typical usage:
 * - Copter, Plane, Rover, Sub: Always enabled (1) - motors/servos essential
 * - Custom applications: May disable (0) if no physical outputs needed
 * 
 * @note Default: 1 (enabled for all vehicle types)
 * @note Disabling this on flight vehicles will prevent any motor/servo control
 * @warning Setting to 0 on vehicles requiring outputs will cause complete loss of control
 */
#ifndef AP_SRV_CHANNELS_ENABLED
#define AP_SRV_CHANNELS_ENABLED 1
#endif

/**
 * @def ACTUATOR_CHANNELS
 * @brief Number of generic actuator channels available
 * 
 * @details Defines count of general-purpose actuator outputs (k_actuator1 through k_actuator6)
 * used for peripherals like grippers, winches, parachutes, and custom mechanisms. These are
 * separate from motor and control surface allocations.
 * 
 * Actuator channel usage:
 * - k_actuator1-6: Generic PWM/servo outputs controlled via RC_Channel or mission commands
 * - Typical applications: Camera shutters, drop mechanisms, sample collectors, lights
 * - Each actuator can be assigned to any physical servo channel via SERVOn_FUNCTION parameter
 * 
 * Relationship to servo functions:
 * - SERVOn_FUNCTION=51: Assigns physical channel n to Actuator 1
 * - SERVOn_FUNCTION=52: Assigns physical channel n to Actuator 2
 * - ... up to SERVOn_FUNCTION=56 for Actuator 6
 * 
 * Control methods:
 * - Direct RC passthrough: RCx_OPTION=201-206 maps RC input to actuator 1-6
 * - Mission commands: DO_SET_SERVO, DO_REPEAT_SERVO for autonomous control
 * - MAVLink: COMMAND_LONG with MAV_CMD_DO_SET_SERVO
 * - Scripting: SRV_Channels:set_output_pwm() for custom logic
 * 
 * @note Default: 6 channels (sufficient for most applications)
 * @note Independent of NUM_SERVO_CHANNELS (total channel count)
 * @note Increasing this value requires more flash space but does not significantly affect RAM
 */
#ifndef ACTUATOR_CHANNELS
#define ACTUATOR_CHANNELS 6
#endif

/**
 * @def ACTUATOR_DEFAULT_INCREMENT
 * @brief Default rate-of-change limit for actuator outputs
 * 
 * @details Fractional increment applied per main loop cycle when slew rate limiting is enabled
 * for actuator outputs. This prevents sudden jerky movements of mechanisms like grippers or
 * landing gear that could cause mechanical stress or load disturbances to the vehicle.
 * 
 * Calculation:
 * - Value of 0.01 = 1% change per loop cycle
 * - At 400Hz loop rate: Full 0-to-1 transition takes 100 cycles = 0.25 seconds
 * - At 50Hz loop rate: Full 0-to-1 transition takes 100 cycles = 2.0 seconds
 * - Actual rate depends on vehicle's main loop frequency
 * 
 * Usage:
 * - Applied when SRV_Channels::set_slew_rate() called for actuator functions
 * - Can be overridden per-channel with custom slew rate values
 * - Disabled by default; must be explicitly enabled in vehicle code
 * 
 * Example applications:
 * - Landing gear deployment: Slow extension to avoid airframe oscillation
 * - Gripper operation: Gentle closing to avoid crushing delicate objects
 * - Camera gimbal positioning: Smooth transitions for video recording
 * 
 * @note Units: Normalized value change per loop cycle (0.0-1.0 scale)
 * @note Applied when set_slew_rate() called for actuator functions
 * @note Does not apply to motor outputs (controlled separately by motor library)
 */
#define ACTUATOR_DEFAULT_INCREMENT 0.01

/**
 * @def NUM_SERVO_CHANNELS
 * @brief Total number of servo/PWM output channels supported
 * 
 * @details Determines array sizing for SRV_Channel objects and memory allocation for the entire
 * output management system. Selection is based on flash size to prevent memory exhaustion on
 * small boards while providing maximum capability on larger platforms.
 * 
 * Size thresholds:
 * - Boards >1024KB flash: 32 channels (full capability for large multicopters, complex planes)
 * - Boards ≤1024KB flash: 16 channels (sufficient for most vehicles, saves ~2KB RAM)
 * 
 * Channel usage by vehicle type:
 * - Copter: 8-12 motors + 4-8 aux (camera, parachute, gripper, lights)
 * - Plane: 4-8 control surfaces + 1-2 motors + 4-8 aux (flaps, landing gear, etc.)
 * - VTOL/Quadplane: 12-16 motors/tilt servos + 8 control surfaces + aux
 * - Rover: 2-4 drive motors + 2 steering servos + aux
 * - Helicopter: 4-7 swash servos + 1 motor + tail servo + aux
 * 
 * Channel numbering:
 * - Channels numbered 1-N (not 0-indexed) to match RCn parameter naming
 * - Servo channel 0 is traditionally not used in ArduPilot
 * - SERVOn_FUNCTION parameter maps channel n to output function
 * - Physical pins depend on board hardware (some boards support >32 with I/O expanders)
 * 
 * Memory implications:
 * - Each channel: ~64 bytes (parameters, state, output values, override flags)
 * - 32 channels: ~2KB RAM + ~4KB parameter storage in EEPROM
 * - 16 channels: ~1KB RAM + ~2KB parameter storage in EEPROM
 * - Processing overhead: ~0.5µs per enabled channel at 400Hz update rate
 * 
 * Board-specific configuration:
 * - hwdef files can override this before including AP_HAL_Boards.h
 * - Example: `define NUM_SERVO_CHANNELS 12` in hwdef.dat for 12-channel board
 * - Override typically used to match exact hardware capability or reduce RAM usage
 * 
 * Channel allocation best practices:
 * - Disable unused channels: Set SERVOn_FUNCTION=0 for channels not physically connected
 * - This conserves processing time without saving RAM (arrays still allocated)
 * - Critical channels (motors, control surfaces) typically use lower numbers
 * - Auxiliary functions (cameras, lights) typically use higher numbers
 * 
 * @note Must not exceed 32 (enforced by static_assert in SRV_Channel.h)
 * @note Servo channel 0 is not used (matches RCn parameter numbering starting at 1)
 * @note Board-specific hwdef files can override this before including AP_HAL_Boards.h
 * @note Increasing this constant rebuilds all SRV_Channel arrays at compile time
 * 
 * @warning Increasing beyond board capabilities causes memory exhaustion and crashes
 * @warning Decreasing on existing systems requires parameter reset (SERVOn_* params renumbered)
 * 
 * Source: libraries/SRV_Channel/SRV_Channel_config.h:15-21
 */
#ifndef NUM_SERVO_CHANNELS
    #if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
        #define NUM_SERVO_CHANNELS 32
    #else
        #define NUM_SERVO_CHANNELS 16
    #endif
#endif
