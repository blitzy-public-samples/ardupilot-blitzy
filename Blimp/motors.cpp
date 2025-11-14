/**
 * @file motors.cpp
 * @brief Motor and fin output processing for Blimp vehicle
 * 
 * @details This file implements the motor output pipeline for lighter-than-air blimp
 *          vehicles. Blimps use a combination of servo-controlled fins and motor outputs
 *          for thrust and directional control in 3D space.
 *          
 *          Key responsibilities:
 *          - Calculate servo PWM values for fin actuators via SRV_Channels
 *          - Coordinate atomic output updates using cork()/push() pattern
 *          - Route control outputs to appropriate hardware channels
 *          - Interface with AP_Motors library for motor control
 *          
 *          Output Architecture:
 *          1. SRV_Channels::calc_pwm() - Calculate PWM values from control inputs
 *          2. srv.cork() - Begin atomic update (buffer outputs)
 *          3. SRV_Channels::output_ch_all() - Update auxiliary channels
 *          4. motors->output() - Send signals to motor outputs
 *          5. srv.push() - Commit all outputs atomically to hardware
 *          
 *          The cork/push pattern ensures all servo and motor outputs are updated
 *          simultaneously, preventing transient control states that could cause
 *          instability in flight.
 * 
 * @note This file is specific to lighter-than-air blimp vehicle control
 * @see SRV_Channels for servo channel management
 * @see AP_Motors for motor output interface
 * 
 * Source: Blimp/motors.cpp
 */

#include "Blimp.h"

// Timing constants for state machine delays (called at 10Hz)
#define ARM_DELAY               20  ///< Arming delay in 10Hz ticks (2 seconds) - time required to complete arming sequence
#define DISARM_DELAY            20  ///< Disarming delay in 10Hz ticks (2 seconds) - time required to complete disarming sequence
#define LOST_VEHICLE_DELAY      10  ///< Lost vehicle delay in 10Hz ticks (1 second) - timeout before declaring vehicle connection lost

/**
 * @brief Send output commands to motors and fin servos for blimp vehicle control
 * 
 * @details This function coordinates the complete output pipeline for blimp vehicle
 *          actuation, routing control commands to both motor outputs and servo-controlled
 *          fin surfaces. It is called at the main control loop rate to update all
 *          actuator outputs.
 *          
 *          Output Processing Pipeline:
 *          
 *          1. **PWM Calculation** (SRV_Channels::calc_pwm):
 *             - Converts high-level servo positions to PWM microsecond values
 *             - Applies min/max limits, trim values, and reversing
 *             - Scales control inputs to configured PWM ranges (typically 1000-2000μs)
 *             - Processes all servo function mappings (fin control surfaces)
 *          
 *          2. **Atomic Update Begin** (srv.cork):
 *             - Initiates buffered output mode to prevent partial updates
 *             - Ensures all outputs are updated simultaneously on hardware
 *             - Critical for preventing transient control states during multi-channel updates
 *             - Prevents servo jitter and unstable flight during output transitions
 *          
 *          3. **Auxiliary Channel Output** (SRV_Channels::output_ch_all):
 *             - Updates all auxiliary servo channels to hardware
 *             - Handles manual passthrough for directly controlled channels
 *             - Processes channels not handled by motor mixer (camera mounts, etc.)
 *             - Applies output to physical servo pins via HAL
 *          
 *          4. **Motor Output** (motors->output):
 *             - Sends thrust commands to motor/thruster outputs
 *             - Applies motor mixing for blimp-specific configuration
 *             - Processes thrust vectoring if configured
 *             - Routes output through AP_Motors interface to ESCs
 *          
 *          5. **Atomic Update Commit** (srv.push):
 *             - Commits all buffered outputs to hardware simultaneously
 *             - Ensures synchronous update of all actuators
 *             - Releases cork() buffer and applies changes atomically
 *          
 *          **Timing Characteristics**:
 *          - Called at main control loop rate (typically 100-400Hz depending on vehicle)
 *          - PWM calculation: ~50-100μs
 *          - Cork/push overhead: <10μs
 *          - Total execution time: <200μs typical
 *          
 *          **Actuator Mapping for Blimp**:
 *          - Fins: Mapped to SRV_Channel functions for directional control
 *          - Thrusters: Controlled via motors pointer for propulsion
 *          - Altitude control: Combined fin and thruster coordination
 *          - Yaw control: Differential fin deflection
 *          
 * @note This function must be called regularly at the control loop rate for stable flight
 * @note Cork/push pattern prevents servo glitching during multi-channel updates
 * @note All servo and motor safety checks (arming, failsafe) are handled upstream
 * 
 * @warning This is a safety-critical function - atomic output updates prevent control
 *          surface desynchronization that could cause loss of vehicle control
 * @warning Do not add delays or blocking operations - called in time-critical path
 * 
 * @see SRV_Channels::calc_pwm() for PWM calculation algorithm
 * @see SRV_Channels::output_ch_all() for channel output implementation
 * @see AP_Motors::output() for motor mixing and output
 * @see HAL_RCOutput for low-level hardware PWM generation
 * 
 * Source: Blimp/motors.cpp:8-26
 */
void Blimp::motors_output()
{
    // Calculate PWM values for all servo channels from high-level control inputs
    // Applies scaling, limits, trim, and reversing to convert servo positions to microseconds
    SRV_Channels::calc_pwm();

    // Get reference to HAL RCOutput interface for atomic update operations
    auto &srv = AP::srv();

    // Begin atomic update: buffer all subsequent outputs until push() is called
    // This prevents partial updates where some servos move before others,
    // which could cause transient instability in blimp control
    srv.cork();

    // Output all auxiliary servo channels (fins, control surfaces)
    // Includes manual passthrough channels and any non-motor outputs
    SRV_Channels::output_ch_all();

    // Send thrust commands to motor/thruster outputs via AP_Motors library
    // Applies motor mixing specific to blimp configuration
    motors->output();

    // Commit all buffered outputs atomically to hardware
    // All servo and motor channels update simultaneously at hardware level
    srv.push();
}
