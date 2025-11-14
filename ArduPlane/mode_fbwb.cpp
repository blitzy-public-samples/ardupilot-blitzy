/**
 * @file mode_fbwb.cpp
 * @brief Implementation of FBWB (Fly By Wire B) flight mode for fixed-wing aircraft
 * 
 * @details FBWB mode provides assisted flight with automatic altitude hold via elevator control.
 *          The pilot controls roll via the aileron stick, and the mode automatically maintains
 *          altitude by adjusting elevator based on pitch stick input and airspeed.
 * 
 *          Key Features:
 *          - Manual roll control with limit enforcement
 *          - Automatic altitude hold with pitch stick commanding climb/descent rate
 *          - Automatic airspeed control via throttle
 *          - Load factor monitoring for structural limits
 *          - Optional soaring support for thermal detection and exploitation
 * 
 *          Control Inputs:
 *          - Roll stick: Direct roll angle control (limited by ROLL_LIMIT_DEG parameter)
 *          - Pitch stick: Commands altitude change rate (centered = hold current altitude)
 *          - Throttle: Automatic airspeed control (can be manually overridden)
 *          - Yaw: Automatic coordination
 * 
 *          Altitude Control:
 *          FBWB uses elevator to maintain altitude target computed from pitch stick deflection.
 *          The altitude controller (update_fbwb_speed_height) integrates TECS (Total Energy
 *          Control System) to balance airspeed and altitude control, preventing altitude loss
 *          during turns and managing energy state.
 * 
 * @note This is a commonly used training and cruise mode as it provides stability assistance
 *       while still giving the pilot direct control authority.
 * 
 * @warning Mode transitions from FBWB should be smooth as altitude and speed targets are
 *          maintained. However, switching to manual modes requires pilot readiness to
 *          assume full control.
 * 
 * @see Mode base class for common mode functionality
 * @see Plane::update_fbwb_speed_height() for altitude/speed controller implementation
 * @see AC_AttitudeControl for roll control implementation
 * 
 * Source: ArduPlane/mode_fbwb.cpp
 */

#include "mode.h"
#include "Plane.h"

/**
 * @brief Initialize FBWB mode upon entry
 * 
 * @details This method is called when the vehicle transitions into FBWB mode.
 *          It performs essential initialization:
 *          1. If soaring is enabled, initializes the soaring controller for cruising flight
 *          2. Sets the initial altitude target to the current altitude to prevent sudden
 *             altitude changes on mode entry
 * 
 *          The altitude target initialization is critical for smooth mode transitions,
 *          ensuring the aircraft maintains its current altitude rather than commanding
 *          an immediate climb or descent to a previous target.
 * 
 * @return true Always returns true indicating successful mode entry
 * 
 * @note This is called automatically by the mode state machine, not directly by user code
 * 
 * @warning Ensure aircraft is in stable flight before switching to FBWB, as the mode
 *          will immediately begin altitude hold from current position
 * 
 * @see plane.set_target_altitude_current() for altitude target initialization
 * @see plane.g2.soaring_controller.init_cruising() for thermal soaring initialization
 */
bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // Initialize soaring controller for autonomous thermal detection and exploitation
    // Sets cruising state expecting pilot or autopilot to command thermaling when detected
    plane.g2.soaring_controller.init_cruising();
#endif

    // Initialize altitude target to current altitude to prevent sudden altitude changes
    // This ensures smooth transition from previous mode without aggressive pitch commands
    plane.set_target_altitude_current();

    return true;
}

/**
 * @brief Update FBWB mode control outputs - called at main loop rate
 * 
 * @details This method executes the core FBWB control logic every scheduler cycle
 *          (typically 50-400 Hz depending on autopilot hardware). It performs three
 *          primary functions:
 * 
 *          1. **Roll Control**: Computes desired roll angle from pilot's roll stick input,
 *             scaled by the configured roll limit (ROLL_LIMIT_DEG parameter). This provides
 *             direct roll control while preventing excessive bank angles that could lead to
 *             stalls or structural limits.
 * 
 *          2. **Load Factor Monitoring**: Updates the current load factor (g-force) based on
 *             airspeed and bank angle. This is used for structural limit enforcement and can
 *             trigger warnings or protections if load limits are exceeded.
 * 
 *          3. **Altitude & Speed Control**: Executes the FBWB-specific altitude hold and
 *             airspeed control through TECS (Total Energy Control System). This controller:
 *             - Computes altitude target from pitch stick input (centered = hold altitude)
 *             - Uses elevator to maintain altitude target
 *             - Uses throttle to maintain airspeed target
 *             - Coordinates energy management between altitude and speed
 * 
 *          Control Flow:
 *          Pilot Roll Input → nav_roll_cd (limited) → Attitude Controller → Ailerons
 *          Pilot Pitch Input → Altitude Rate → TECS → Elevator
 *          Airspeed Error → TECS → Throttle
 * 
 * @note This method is called at main loop rate by the mode state machine. It sets
 *       control targets but does not directly command servos - that's handled by the
 *       attitude controller and servo output stages.
 * 
 * @note Roll limit code contributed by Yury MonZon to prevent excessive bank angles
 * 
 * @warning This method assumes valid sensor data (IMU, airspeed, barometer). Loss of
 *          critical sensors may degrade performance or trigger failsafes.
 * 
 * @see plane.update_fbwb_speed_height() for altitude/airspeed controller implementation
 * @see plane.update_load_factor() for structural load monitoring
 * @see plane.channel_roll for RC input interface
 * @see plane.nav_roll_cd for commanded roll angle output (centidegrees)
 * @see ROLL_LIMIT_DEG parameter for maximum roll angle configuration
 * 
 * Source: ArduPlane/mode_fbwb.cpp:16-23
 */
void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    
    // Compute commanded roll angle from pilot input, scaled by roll limit
    // nav_roll_cd is in centidegrees, norm_input() returns -1.0 to +1.0
    // This provides direct roll control while respecting configured limits
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    
    // Update load factor (g-force) based on current airspeed and bank angle
    // Used for structural limit monitoring and potential limit enforcement
    plane.update_load_factor();
    
    // Execute FBWB altitude hold and airspeed control via TECS
    // Computes altitude target from pitch stick, controls elevator for altitude
    // and throttle for airspeed, managing energy state coordination
    plane.update_fbwb_speed_height();

}

