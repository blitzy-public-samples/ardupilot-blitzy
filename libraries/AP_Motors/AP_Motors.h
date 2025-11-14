/**
 * @file AP_Motors.h
 * @brief ArduPilot Motor Control Library - Umbrella Header
 * 
 * @details This is the primary include file for the AP_Motors library, providing
 *          a single inclusion point to access all motor control functionality.
 *          
 *          **Purpose**: Aggregates the complete AP_Motors API into one header,
 *          automatically including all frame-specific implementations based on
 *          enabled feature flags.
 *          
 *          **Usage**: Vehicle code (ArduCopter, ArduPlane QuadPlane, ArduSub, etc.)
 *          should include ONLY this header file to access motor control functionality.
 *          Do not include individual AP_Motors*.h headers directly.
 *          
 * ## Header Organization
 * 
 * This umbrella header includes components in the following order:
 * 
 * 1. **AP_Motors_config.h** - Configuration and feature flags (always first)
 *    - Defines AP_MOTORS_*_ENABLED feature flags
 *    - Controls which frame types are compiled
 *    - Must be included before any other AP_Motors headers
 * 
 * 2. **AP_Motors_Class.h** - Abstract base class
 *    - Defines the core motor control interface
 *    - Common functionality shared by all motor types
 *    - Output throttle/roll/pitch/yaw interface
 * 
 * 3. **AP_MotorsMulticopter.h** - Multicopter base class
 *    - Base class for all multicopter frame types
 *    - Implements attitude to motor output conversion
 *    - Thrust linearization and battery compensation
 * 
 * 4. **AP_MotorsMatrix.h** - Matrix mixing for standard multirotors
 *    - Used by Quad, Hexa, Octa, Y6, and custom matrix frames
 *    - Configurable motor mixing matrix
 *    - Most common frame implementation
 * 
 * 5. **Frame-Specific Headers** (conditionally included):
 *    - AP_MotorsTri.h - Tricopter with tilting rear servo (if AP_MOTORS_TRI_ENABLED)
 *    - AP_MotorsSingle.h - Single main rotor + yaw control
 *    - AP_MotorsCoax.h - Coaxial helicopter
 *    - AP_MotorsTailsitter.h - Tailsitter VTOL aircraft
 *    - AP_MotorsHeli_Single.h - Traditional single-rotor helicopter
 *    - AP_MotorsHeli_Dual.h - Dual-rotor helicopter (tandem, transverse, intermeshing)
 *    - AP_MotorsHeli_Quad.h - Quad helicopter (collective pitch)
 *    - AP_Motors6DOF.h - Six degrees of freedom (submarine/ROV)
 *    - AP_MotorsMatrix_6DoF_Scripting.h - Scripting-configurable 6DOF
 *    - AP_MotorsMatrix_Scripting_Dynamic.h - Dynamic scripting mixer
 * 
 * ## API Architecture Layers
 * 
 * The AP_Motors library is organized in a three-layer hierarchy:
 * 
 * **Layer 1: Abstract Interface** (AP_Motors_Class)
 * - Defines the standard motor control API
 * - Pure virtual methods for output(), init(), set_frame_class_and_type()
 * - Common parameter handling
 * - Battery voltage compensation interface
 * 
 * **Layer 2: Type-Specific Base Classes**
 * - AP_MotorsMulticopter - All multicopter types (matrix, tri, single, coax)
 * - AP_MotorsHeli - All helicopter types (single, dual, quad)
 * - AP_Motors6DOF - Underwater 6DOF vehicles
 * 
 * **Layer 3: Frame Implementations**
 * - AP_MotorsMatrix - Quad X, Quad +, Hexa, Octa, Y6, etc.
 * - AP_MotorsTri - Tricopter
 * - AP_MotorsSingle - Single + tail motor
 * - AP_MotorsCoax - Coaxial copter
 * - AP_MotorsTailsitter - Tailsitter VTOL
 * - AP_MotorsHeli_* - Various helicopter configurations
 * - AP_MotorsMatrix_*_Scripting - Lua-scriptable motor configurations
 * 
 * ## Usage Examples
 * 
 * **Basic multicopter initialization:**
 * @code
 * #include <AP_Motors/AP_Motors.h>
 * 
 * // Instantiate with 400Hz update rate
 * AP_MotorsMatrix motors(400);
 * 
 * // Initialize for Quad X frame
 * motors.init(MOTOR_FRAME_QUAD, MOTOR_FRAME_TYPE_X);
 * 
 * // Set motor limits
 * motors.set_throttle_range(1000, 2000);
 * 
 * // Main loop: Set desired attitude and throttle
 * motors.set_roll(pilot_roll_input);      // -1.0 to 1.0
 * motors.set_pitch(pilot_pitch_input);    // -1.0 to 1.0
 * motors.set_yaw(pilot_yaw_input);        // -1.0 to 1.0
 * motors.set_throttle(pilot_throttle);    // 0.0 to 1.0
 * 
 * // Calculate motor outputs and send to ESCs
 * motors.output();
 * @endcode
 * 
 * **Helicopter initialization:**
 * @code
 * #include <AP_Motors/AP_Motors.h>
 * 
 * // Instantiate single-rotor helicopter motor controller
 * AP_MotorsHeli_Single heli_motors(400);
 * 
 * // Initialize (frame type not applicable for helis)
 * heli_motors.init(MOTOR_FRAME_HELI, 0);
 * 
 * // Configure swash plate and RSC (Rotor Speed Control)
 * heli_motors.set_swash_plate_type(SWASH_TYPE_H3);
 * heli_motors.set_rsc_mode(RSC_MODE_PASSTHROUGH);
 * @endcode
 * 
 * **6DOF submarine/ROV initialization:**
 * @code
 * #include <AP_Motors/AP_Motors.h>
 * 
 * // Instantiate 6DOF motor controller
 * AP_Motors6DOF sub_motors(400);
 * 
 * // Initialize for vectored thrust configuration
 * sub_motors.init(MOTOR_FRAME_SUBMARINE, 0);
 * 
 * // Set 6DOF inputs (roll, pitch, yaw, forward, lateral, throttle)
 * sub_motors.set_roll(roll_input);
 * sub_motors.set_pitch(pitch_input);
 * sub_motors.set_yaw(yaw_input);
 * sub_motors.set_forward(forward_input);
 * sub_motors.set_lateral(lateral_input);
 * sub_motors.set_throttle(vertical_input);
 * 
 * sub_motors.output();
 * @endcode
 * 
 * ## Vehicle Integration
 * 
 * This header is used by multiple ArduPilot vehicle types:
 * - **ArduCopter**: All multicopter frames (matrix, tri, single, coax, heli)
 * - **ArduPlane**: QuadPlane VTOL motors
 * - **ArduSub**: 6DOF underwater thruster control
 * - **Rover**: (Limited) motor control for special configurations
 * 
 * ## Feature Flag System
 * 
 * Motor types can be disabled at compile-time to save flash space:
 * - AP_MOTORS_TRI_ENABLED - Tricopter support (conditionally included)
 * - Additional flags defined in AP_Motors_config.h
 * 
 * When a frame type is disabled, its header is not included and the
 * implementation is not compiled, reducing binary size.
 * 
 * @note **IMPORTANT**: Always include this umbrella header (AP_Motors.h) rather
 *       than individual component headers. This ensures correct initialization
 *       order and respects feature flag configuration.
 * 
 * @note This header provides the complete AP_Motors API in a single include,
 *       simplifying vehicle code and ensuring consistency across frame types.
 * 
 * @note Configuration (AP_Motors_config.h) is always included first to ensure
 *       feature flags are defined before conditional compilation.
 * 
 * @warning **DO NOT** include individual AP_Motors*.h headers directly in vehicle
 *          code. Always use this umbrella header to ensure proper configuration
 *          and feature flag handling.
 * 
 * @warning Include order matters: Always include AP_Motors.h AFTER AP_HAL headers
 *          but BEFORE vehicle-specific code that uses motor functionality.
 * 
 * @warning Feature flags (AP_MOTORS_*_ENABLED) must be consistent across all
 *          compilation units. Mixing enabled/disabled states will cause linker errors.
 * 
 * @see libraries/AP_Motors/README.md - Complete architecture and usage documentation
 * @see AP_Motors_Class.h - Base class API documentation
 * @see AP_MotorsMulticopter.h - Multicopter-specific API
 * @see AP_MotorsMatrix.h - Matrix mixer implementation
 * @see examples/ - Test harnesses and validation tools
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Motors/AP_Motors.h
 */

#pragma once

// Configuration and feature flags - MUST be included first
#include "AP_Motors_config.h"

// Abstract base class - defines core motor control interface
#include "AP_Motors_Class.h"

// Multicopter base class - common functionality for all multicopter types
#include "AP_MotorsMulticopter.h"

// Matrix mixer - most common multicopter implementation (Quad, Hexa, Octa, Y6, etc.)
#include "AP_MotorsMatrix.h"

// Tricopter - three motors with tilting rear servo for yaw control
// Conditionally included based on feature flag to save flash space
#if AP_MOTORS_TRI_ENABLED
#include "AP_MotorsTri.h"
#endif  // AP_MOTORS_TRI_ENABLED

// Traditional helicopter implementations
#include "AP_MotorsHeli_Single.h"     // Single main rotor + tail rotor
#include "AP_MotorsHeli_Dual.h"       // Dual rotor (tandem, transverse, intermeshing)
#include "AP_MotorsHeli_Quad.h"       // Quad helicopter with collective pitch

// Specialized multicopter frame types
#include "AP_MotorsSingle.h"          // Single main rotor + yaw control motors
#include "AP_MotorsCoax.h"            // Coaxial counter-rotating motors

// VTOL and underwater vehicle types
#include "AP_MotorsTailsitter.h"      // Tailsitter VTOL aircraft
#include "AP_Motors6DOF.h"             // Six degrees of freedom (submarine/ROV)

// Scripting-enabled motor configurations for custom frame types
#include "AP_MotorsMatrix_6DoF_Scripting.h"      // Lua-scriptable 6DOF mixer
#include "AP_MotorsMatrix_Scripting_Dynamic.h"   // Dynamic scripting mixer
