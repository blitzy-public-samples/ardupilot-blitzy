/**
 * @file tiltrotor.h
 * @brief Tilt-rotor mechanism management for QuadPlane VTOL transitions
 * 
 * @details This header defines the Tiltrotor class which manages motor tilt mechanisms
 *          for QuadPlane aircraft during transitions between vertical takeoff/landing (VTOL)
 *          and fixed-wing forward flight modes. The tiltrotor system controls servo-actuated
 *          motor tilts to optimize thrust vectoring throughout the flight envelope.
 *          
 *          Key responsibilities:
 *          - Servo control for motor tilt angles (0° = up/VTOL, max_angle_deg = forward flight)
 *          - Slew rate limiting during tilt transitions to prevent abrupt attitude changes
 *          - Thrust compensation for tilted motors to maintain desired force vectors
 *          - Multiple tilt mechanism types: continuous, binary, vectored yaw, bicopter
 *          - Integration with QuadPlane transition state machine
 *          
 *          Coordinate system: Tilt angles in degrees from vertical (0° = motors pointing up)
 *          
 * @note This module is specific to tilt-rotor QuadPlane configurations and is not used
 *       by standard fixed-wing or multicopter vehicles.
 * 
 * @warning Incorrect tilt configuration can result in loss of control during transitions.
 *          Tilt rates and angles must be validated through extensive SITL and flight testing.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <AP_Logger/LogStructure.h>

class QuadPlane;
class AP_MotorsMulticopter;
class Tiltrotor_Transition;

/**
 * @class Tiltrotor
 * @brief Manages tilt-rotor servo mechanisms for QuadPlane VTOL transitions
 * 
 * @details The Tiltrotor class controls servo-actuated motor tilting mechanisms that enable
 *          QuadPlane aircraft to transition between vertical flight (VTOL) and forward flight
 *          (fixed-wing) modes. Motors can tilt from vertical (0°) for hover to forward-facing
 *          (max_angle_deg, typically 90°) for efficient cruise flight.
 *          
 *          Tilt Mechanism Types:
 *          - TILT_TYPE_CONTINUOUS (0): Smooth servo transitions with configurable slew rates
 *          - TILT_TYPE_BINARY (1): Two-position mechanism (up or forward, no intermediate)
 *          - TILT_TYPE_VECTORED_YAW (2): Tilts provide yaw control during transition
 *          - TILT_TYPE_BICOPTER (3): Bicopter-style differential tilt for yaw
 *          
 *          Motor Configuration:
 *          - tilt_mask parameter specifies which motors tilt (bitmask, bit N = motor N)
 *          - Motors not in tilt_mask remain fixed (either always up or always forward)
 *          - Supports asymmetric configurations (e.g., front motors tilt, rear fixed)
 *          
 *          Tilt Control:
 *          - Slew rates limited by max_rate_up_dps and max_rate_down_dps (degrees per second)
 *          - Current tilt angle tracked in current_tilt (degrees from vertical)
 *          - Thrust compensation applied to maintain desired force vectors during tilt
 *          
 *          Integration:
 *          - Called from QuadPlane::update() during transitions and forward flight
 *          - Coordinates with transition state machine (Tiltrotor_Transition class)
 *          - Outputs to servo channels configured for tilt mechanism
 *          
 * @note Tilt angles: 0° = vertical/up (VTOL), max_angle_deg = fully forward (fixed-wing)
 * @note Update rate: Called at main loop rate (typically 400Hz for QuadPlane)
 * 
 * @warning Tilt configuration errors can cause loss of control during transitions. Always
 *          validate tilt_mask, max_angle_deg, and slew rates in SITL before flight testing.
 * @warning Rapid tilt transitions can induce large attitude disturbances. Conservative
 *          slew rates (30-60 deg/s) recommended for initial tuning.
 * 
 * @see QuadPlane class for transition state machine integration
 * @see AP_MotorsMulticopter for motor output integration
 * @see Tiltrotor_Transition for transition-specific yaw control
 */
class Tiltrotor
{
friend class QuadPlane;
friend class Plane;
friend class Tiltrotor_Transition;
public:

    /**
     * @brief Construct a Tiltrotor object
     * 
     * @param[in] _quadplane Reference to parent QuadPlane object
     * @param[in] _motors Reference to motor control object
     */
    Tiltrotor(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    /**
     * @brief Check if tiltrotor system is enabled and ready
     * 
     * @return true if enable > 0 and setup is complete, false otherwise
     * 
     * @note Returns false if setup() has not been successfully called
     */
    bool enabled() const { return (enable > 0) && setup_complete;}

    /**
     * @brief Initialize tiltrotor system and validate configuration
     * 
     * @details Validates tilt_mask against available motors, determines if vehicle has
     *          fixed forward motors or fixed VTOL motors, and completes initialization.
     *          Must be called before tiltrotor system can be used.
     * 
     * @note Called once during QuadPlane initialization
     */
    void setup();

    /**
     * @brief Slew tilt servos toward target angle with rate limiting (continuous type)
     * 
     * @param[in] tilt Target tilt angle in degrees from vertical (0° = up, max_angle_deg = forward)
     * 
     * @details Applies slew rate limits (max_rate_up_dps, max_rate_down_dps) to smoothly
     *          transition tilt servos to target angle. Updates current_tilt based on time
     *          delta and configured rates.
     * 
     * @note Only used for TILT_TYPE_CONTINUOUS
     * @note Called at main loop rate during transitions
     */
    void slew(float tilt);

    /**
     * @brief Command binary tilt mechanism to up or forward position
     * 
     * @param[in] forward true = tilt fully forward, false = tilt fully up
     * 
     * @details For binary tilt mechanisms that have only two positions (no intermediate
     *          angles). Immediately commands servos to end position.
     * 
     * @note Only used for TILT_TYPE_BINARY
     */
    void binary_slew(bool forward);

    /**
     * @brief Main update function for tiltrotor system
     * 
     * @details Routes to type-specific update function based on configured tilt type.
     *          Updates servo outputs and thrust compensation. Called every loop.
     * 
     * @note Called from QuadPlane::update() at main loop rate (typically 400Hz)
     * @see continuous_update(), binary_update(), vectoring(), bicopter_output()
     */
    void update();

    /**
     * @brief Update continuous tilt servos during transitions and forward flight
     * 
     * @details Calculates desired tilt angle based on flight mode and transition state,
     *          applies slew rate limiting, outputs to tilt servos, and applies thrust
     *          compensation for tilted motors.
     * 
     * @note Only called when type == TILT_TYPE_CONTINUOUS
     */
    void continuous_update();

    /**
     * @brief Update binary tilt mechanism position
     * 
     * @details Commands tilts to fully up (VTOL) or fully forward based on flight mode
     *          and transition state. No intermediate positions.
     * 
     * @note Only called when type == TILT_TYPE_BINARY
     */
    void binary_update();

    /**
     * @brief Update vectored yaw tilt mechanism
     * 
     * @details Controls differential tilt for yaw control during transition, allowing
     *          tilt mechanisms to provide yaw authority when transitioning between VTOL
     *          and forward flight modes.
     * 
     * @note Only called when type == TILT_TYPE_VECTORED_YAW
     */
    void vectoring();

    /**
     * @brief Update bicopter-style differential tilt for yaw control
     * 
     * @details Controls left/right motor tilt differential to provide yaw control in
     *          bicopter configuration. Tilts move in opposite directions for yaw torque.
     * 
     * @note Only called when type == TILT_TYPE_BICOPTER
     */
    void bicopter_output();

    /**
     * @brief Apply thrust compensation for tilted motors with custom multipliers
     * 
     * @param[in,out] thrust Array of thrust values for each motor (0.0-1.0 range)
     * @param[in]     num_motors Number of motors in thrust array
     * @param[in]     non_tilted_mul Thrust multiplier for non-tilted motors
     * @param[in]     tilted_mul Thrust multiplier for tilted motors
     * 
     * @details Adjusts motor thrust values to compensate for tilt angles. When motors tilt
     *          forward, their vertical thrust component decreases (proportional to cos(tilt)).
     *          This function scales thrust to maintain desired vertical and forward forces.
     * 
     * @note Modifies thrust array in-place
     * @see tilt_compensate() for standard compensation
     */
    void tilt_compensate_angle(float *thrust, uint8_t num_motors, float non_tilted_mul, float tilted_mul);

    /**
     * @brief Apply standard thrust compensation for tilted motors
     * 
     * @param[in,out] thrust Array of thrust values for each motor (0.0-1.0 range)
     * @param[in]     num_motors Number of motors in thrust array
     * 
     * @details Applies thrust compensation using standard multipliers. Increases thrust
     *          on tilted motors to maintain vertical lift as they tilt toward horizontal.
     * 
     * @note Modifies thrust array in-place
     * @note Called during transitions and forward flight with tilted motors
     * @see tilt_compensate_angle() for custom multipliers
     */
    void tilt_compensate(float *thrust, uint8_t num_motors);

    /**
     * @brief Check if current tilt angle exceeds maximum configured angle
     * 
     * @return true if current_tilt > max_angle_deg, false otherwise
     * 
     * @note Used for transition state logic and safety checks
     */
    bool tilt_over_max_angle(void) const;

    /**
     * @brief Check if a specific motor has tilt capability
     * 
     * @param[in] motor Motor number (0-based index)
     * 
     * @return true if motor is in tilt_mask (can tilt), false if fixed
     * 
     * @note Motor numbers correspond to AP_Motors motor indexing
     */
    bool is_motor_tilting(uint8_t motor) const {
        return tilt_mask.get() & (1U<<motor);
    }

    /**
     * @brief Check if tilts are at fully forward position
     * 
     * @return true if current_tilt >= max_angle_deg, false otherwise
     * 
     * @note Used by transition logic to determine when forward flight can begin
     */
    bool fully_fwd() const;

    /**
     * @brief Check if tilts are at fully up/vertical position
     * 
     * @return true if current_tilt <= 0, false otherwise
     * 
     * @note Used by transition logic to determine when VTOL flight mode is safe
     */
    bool fully_up() const;

    /**
     * @brief Calculate maximum tilt angle change for current loop iteration
     * 
     * @param[in] up true if tilting toward vertical, false if tilting forward
     * @param[in] in_flap_range true if in flap angle range (default: false)
     * 
     * @return Maximum angle change in degrees for this iteration
     * 
     * @details Calculates slew-rate-limited angle change based on loop time delta and
     *          configured max_rate_up_dps or max_rate_down_dps parameters.
     * 
     * @note Flap range uses different rates when in_flap_range is true
     */
    float tilt_max_change(bool up, bool in_flap_range = false) const;

    /**
     * @brief Get the fully forward tilt angle setting
     * 
     * @return Tilt angle in degrees for fully forward flight (typically max_angle_deg)
     * 
     * @note May differ from max_angle_deg for certain configurations
     */
    float get_fully_forward_tilt() const;

    /**
     * @brief Get the tilt angle used during forward flight
     * 
     * @return Tilt angle in degrees for normal forward flight
     * 
     * @details Returns fixed_angle if configured, otherwise max_angle_deg.
     *          Allows different tilt angles for cruise vs maximum forward tilt.
     */
    float get_forward_flight_tilt() const;

    /**
     * @brief Update yaw target for tiltrotor transition coordination
     * 
     * @details Coordinates desired yaw heading during transition to minimize sideslip
     *          and maintain directional control as tilts move between VTOL and forward.
     * 
     * @note Called during transition phases
     * @see Tiltrotor_Transition::update_yaw_target()
     */
    void update_yaw_target();

    /**
     * @brief Check if tiltrotor is configured for vectored yaw control
     * 
     * @return true if enabled and vectored yaw type, false otherwise
     * 
     * @note Vectored configurations use tilt differential for yaw control
     */
    bool is_vectored() const { return enabled() && _is_vectored; }

    /**
     * @brief Check if vehicle has fixed forward motor (non-tilting)
     * 
     * @return true if vehicle has at least one motor that doesn't tilt (always forward)
     * 
     * @note Determined during setup() by analyzing tilt_mask vs available motors
     */
    bool has_fw_motor() const { return _have_fw_motor; }

    /**
     * @brief Check if vehicle has fixed VTOL motor (non-tilting)
     * 
     * @return true if vehicle has at least one motor that doesn't tilt (always up)
     * 
     * @note Determined during setup() by analyzing tilt_mask vs available motors
     */
    bool has_vtol_motor() const { return _have_vtol_motor; }

    /**
     * @brief Check if tiltrotor motors are actively providing thrust
     * 
     * @return true if enabled and motors are armed/active, false otherwise
     * 
     * @note Used for thrust compensation and control logic
     */
    bool motors_active() const { return enabled() && _motors_active; }

    /**
     * @brief Check if tilts have reached target angle (slewing complete)
     * 
     * @return true if target angle achieved or not applicable (disabled/non-continuous)
     * 
     * @details For continuous tilt types, returns true only when current_tilt matches
     *          commanded angle within tolerance. For binary types, always returns true.
     *          For disabled tiltrotor, always returns true.
     * 
     * @note Used by transition state machine to sequence flight mode changes
     */
    bool tilt_angle_achieved() const { return !enabled() || (type != TILT_TYPE_CONTINUOUS) || angle_achieved; }

    /**
     * @brief Get throttle level of tilting motors during forward flight
     * 
     * @param[out] throttle Throttle value of tilting motors (0.0-1.0)
     * 
     * @return true if throttle successfully retrieved, false if not in forward flight
     * 
     * @details Provides throttle information for tilting motors when used for forward
     *          propulsion. Used for power management and performance monitoring.
     */
    bool get_forward_throttle(float &throttle) const;

    /**
     * @brief Write tiltrotor telemetry to dataflash log
     * 
     * @details Logs current tilt angles and state for post-flight analysis and debugging.
     *          Creates TILT log messages with current_tilt and per-motor tilt angles.
     * 
     * @note Called periodically from main logging routine
     * @see log_tiltrotor structure for logged data format
     */
    void write_log();

    /**
     * @brief Enable/disable tiltrotor functionality
     * 
     * Parameter: Q_TILT_ENABLE
     * - 0 = Disabled
     * - 1 = Enabled
     * 
     * @note Must be non-zero for tiltrotor features to activate
     */
    AP_Int8 enable;

    /**
     * @brief Bitmask of motors that can tilt
     * 
     * Parameter: Q_TILT_MASK
     * Bit N corresponds to motor N (e.g., 0x03 = motors 0 and 1 tilt)
     * 
     * @note Motors not in mask remain fixed (either always up or always forward)
     * @note Must match physical tilt mechanism configuration
     */
    AP_Int16 tilt_mask;

    /**
     * @brief Maximum tilt rate when moving toward vertical (VTOL)
     * 
     * Parameter: Q_TILT_RATE_UP
     * Units: degrees per second
     * 
     * @note Conservative values (30-60 deg/s) recommended to avoid attitude disturbances
     * @note Only applies to TILT_TYPE_CONTINUOUS
     */
    AP_Int16 max_rate_up_dps;

    /**
     * @brief Maximum tilt rate when moving toward forward flight
     * 
     * Parameter: Q_TILT_RATE_DN
     * Units: degrees per second
     * 
     * @note Can be faster than max_rate_up since transition to forward flight less critical
     * @note Only applies to TILT_TYPE_CONTINUOUS
     */
    AP_Int16 max_rate_down_dps;

    /**
     * @brief Maximum tilt angle for fully forward flight
     * 
     * Parameter: Q_TILT_MAX
     * Units: degrees from vertical (typically 90° for horizontal)
     * Range: 0-90 degrees
     * 
     * @note 0° = vertical/up (VTOL), 90° = horizontal (forward flight)
     * @warning Incorrect angle can prevent transition or cause loss of lift
     */
    AP_Int8  max_angle_deg;

    /**
     * @brief Tilt mechanism type selection
     * 
     * Parameter: Q_TILT_TYPE
     * Values:
     * - 0 = TILT_TYPE_CONTINUOUS (smooth servo transition)
     * - 1 = TILT_TYPE_BINARY (two-position only)
     * - 2 = TILT_TYPE_VECTORED_YAW (differential tilt for yaw)
     * - 3 = TILT_TYPE_BICOPTER (bicopter-style yaw control)
     * 
     * @see Tilt type enum definitions below
     */
    AP_Int8  type;

    /**
     * @brief Tilt angle for yaw vectoring control
     * 
     * Parameter: Q_TILT_YAW_ANGLE
     * Units: degrees
     * 
     * @note Used by TILT_TYPE_VECTORED_YAW for differential yaw control
     */
    AP_Float tilt_yaw_angle;

    /**
     * @brief Fixed tilt angle for cruise flight
     * 
     * Parameter: Q_TILT_FIX_ANGLE
     * Units: degrees from vertical
     * 
     * @details If non-zero, uses this angle for cruise instead of max_angle_deg.
     *          Allows different tilt for efficient cruise vs maximum forward tilt.
     */
    AP_Float fixed_angle;

    /**
     * @brief Gain for fixed angle tilt control
     * 
     * Parameter: Q_TILT_FIX_GAIN
     * Range: 0.0-1.0
     * 
     * @note Controls blend between fixed angle and full forward tilt
     */
    AP_Float fixed_gain;

    /**
     * @brief Tilt angle for flap deployment in transitions
     * 
     * Parameter: Q_TILT_FLAP_ANGLE
     * Units: degrees from vertical
     * 
     * @details When tilts reach this angle during transition, aircraft can deploy
     *          control surfaces (flaps) for improved fixed-wing control authority.
     */
    AP_Float flap_angle_deg;

    /**
     * @brief Current tilt angle of motors
     * 
     * Units: degrees from vertical (0° = up, max_angle_deg = forward)
     * 
     * @note Updated by slew() functions based on commanded angle and rate limits
     * @note Read by thrust compensation and transition state machine
     */
    float current_tilt;

    /**
     * @brief Current throttle of tilting motors
     * 
     * Range: 0.0-1.0 (0% to 100% throttle)
     * 
     * @note Used for forward flight propulsion monitoring
     */
    float current_throttle;

    /**
     * @brief Flag indicating if tiltrotor motors are actively thrusting
     * 
     * @note Updated based on arming state and motor outputs
     */
    bool _motors_active:1;

    /**
     * @brief Target yaw heading during tiltrotor transition
     * 
     * Units: centidegrees (0-36000)
     * 
     * @note Coordinates yaw control during transition to minimize sideslip
     */
    float transition_yaw_cd;

    /**
     * @brief Timestamp when transition_yaw_cd was last set
     * 
     * Units: milliseconds (from AP_HAL::millis())
     * 
     * @note Used for transition yaw hold timing
     */
    uint32_t transition_yaw_set_ms;

    /**
     * @brief Flag indicating vectored yaw control is active
     * 
     * @note true if type == TILT_TYPE_VECTORED_YAW and enabled
     */
    bool _is_vectored;

    /**
     * @brief Tilt mechanism type enumeration
     * 
     * @details Defines the different tilt mechanism types supported by the tiltrotor system.
     *          Each type has different control characteristics and use cases.
     */
    enum {
        /**
         * @brief Continuous servo tilt with smooth transitions
         * 
         * @details Servos can move smoothly through full tilt range with configurable
         *          slew rates. Allows intermediate tilt angles for optimized transitions.
         *          Most common type for standard tilt-rotor QuadPlanes.
         * 
         * @note Requires max_rate_up_dps and max_rate_down_dps configuration
         */
        TILT_TYPE_CONTINUOUS    = 0,

        /**
         * @brief Binary tilt mechanism (two positions only)
         * 
         * @details Mechanism has only fully up or fully forward positions with no
         *          intermediate angles. Typically uses non-proportional actuators
         *          or mechanical locks. Transition is instantaneous.
         * 
         * @note Slew rates not applicable - transition is immediate
         * @warning Abrupt tilt changes can cause significant attitude disturbances
         */
        TILT_TYPE_BINARY        = 1,

        /**
         * @brief Vectored yaw control via differential tilt
         * 
         * @details Uses differential tilt angles between motors to provide yaw control
         *          during transitions. Allows yaw authority when traditional yaw control
         *          (tail rotor, rudder) may be ineffective during transition speeds.
         * 
         * @note Requires tilt_yaw_angle configuration
         * @see tilt_yaw_angle parameter
         */
        TILT_TYPE_VECTORED_YAW  = 2,

        /**
         * @brief Bicopter differential tilt for yaw control
         * 
         * @details Left and right motors tilt in opposite directions to generate yaw
         *          torque, similar to traditional bicopter yaw control mechanism.
         *          Used specifically for bicopter tilt-rotor configurations.
         * 
         * @note Requires bicopter motor layout (left/right motor pair)
         */
        TILT_TYPE_BICOPTER      = 3
    };

    /**
     * @brief Parameter table for tiltrotor configuration
     * 
     * @details Defines persistent parameters stored in EEPROM using AP_Param system.
     *          Includes enable, tilt_mask, rate limits, angles, and type selection.
     * 
     * @see AP_Param documentation for parameter system details
     */
    static const struct AP_Param::GroupInfo var_info[];

private:

    /**
     * @brief Dataflash log message structure for tiltrotor telemetry
     * 
     * @details Defines binary log format for TILT messages written to dataflash.
     *          Used for post-flight analysis and debugging of tilt mechanism behavior.
     * 
     * Fields:
     * - time_us: Timestamp in microseconds
     * - current_tilt: Overall current tilt angle in degrees
     * - front_left_tilt: Left front motor tilt angle in degrees
     * - front_right_tilt: Right front motor tilt angle in degrees
     * 
     * @note PACKED attribute ensures no padding for consistent binary format
     * @see write_log() for logging implementation
     */
    struct PACKED log_tiltrotor {
        LOG_PACKET_HEADER;
        uint64_t time_us;
        float current_tilt;
        float front_left_tilt;
        float front_right_tilt;
    };

    /**
     * @brief Flag indicating setup() has completed successfully
     * 
     * @note Used by enabled() to ensure system is ready before use
     */
    bool setup_complete;

    /**
     * @brief Flag indicating vehicle has at least one fixed forward motor
     * 
     * @details true if at least one motor is not in tilt_mask and configured for
     *          forward thrust. Affects transition behavior and thrust compensation.
     * 
     * @note Determined during setup() based on tilt_mask and motor configuration
     */
    bool _have_fw_motor;

    /**
     * @brief Flag indicating vehicle has at least one fixed VTOL motor
     * 
     * @details true if at least one motor is not in tilt_mask and configured for
     *          vertical thrust. Affects transition behavior and VTOL capabilities.
     * 
     * @note Determined during setup() based on tilt_mask and motor configuration
     */
    bool _have_vtol_motor;

    /**
     * @brief Flag indicating current tilt angle matches commanded angle
     * 
     * @details With slow tilt rates, actual angle can lag commanded angle during
     *          slewing. This flag is true when angles match within tolerance.
     * 
     * @note Used by tilt_angle_achieved() for transition sequencing
     * @note Only meaningful for TILT_TYPE_CONTINUOUS
     */
    bool angle_achieved;

    /**
     * @brief Reference to parent QuadPlane object
     * 
     * @note Allows access to vehicle state and transition management
     */
    QuadPlane& quadplane;

    /**
     * @brief Reference to motor control object
     * 
     * @note Used for motor output and thrust compensation
     */
    AP_MotorsMulticopter*& motors;

    /**
     * @brief Pointer to transition controller for tiltrotor-specific transition logic
     * 
     * @note Manages yaw coordination and flight mode view during transitions
     * @see Tiltrotor_Transition class
     */
    Tiltrotor_Transition* transition;

};

/**
 * @class Tiltrotor_Transition
 * @brief Transition controller for tiltrotor QuadPlanes with separate left thrust
 * 
 * @details Extends SLT_Transition (Separate Left Thrust Transition) class to provide
 *          tiltrotor-specific transition behavior. Manages yaw coordination, flight mode
 *          display, and control authority during transitions between VTOL and forward flight.
 *          
 *          Key responsibilities:
 *          - Coordinate yaw target during tilt transitions to minimize sideslip
 *          - Determine when to show VTOL vs fixed-wing flight mode display
 *          - Decide control authority (multirotor vs fixed-wing) during forward transition
 *          
 *          Tiltrotor-specific considerations:
 *          - Yaw control transitions as tail control surfaces become effective
 *          - VTOL view shown until tilts reach forward flight angles
 *          - Multirotor control used while tilts still provide vertical thrust
 * 
 * @note Inherits from SLT_Transition for separate thrust motor configurations
 * @see SLT_Transition for base transition functionality
 * @see Tiltrotor class for tilt mechanism control
 */
class Tiltrotor_Transition : public SLT_Transition
{
friend class Tiltrotor;
public:

    /**
     * @brief Construct a Tiltrotor_Transition controller
     * 
     * @param[in] _quadplane Reference to parent QuadPlane object
     * @param[in] _motors Reference to motor control object
     * @param[in] _tiltrotor Reference to tiltrotor control object
     * 
     * @note Forwards quadplane and motors to SLT_Transition base class
     */
    Tiltrotor_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tiltrotor& _tiltrotor):SLT_Transition(_quadplane, _motors), tiltrotor(_tiltrotor) {};

    /**
     * @brief Update yaw target during tiltrotor transition
     * 
     * @param[in,out] yaw_target_cd Desired yaw heading in centidegrees (0-36000)
     * 
     * @return true if yaw target updated, false if using default behavior
     * 
     * @details Coordinates yaw control during transition based on tilt angle and airspeed.
     *          Manages handoff from multirotor yaw control to rudder/tail control as
     *          tilts move forward and airspeed increases.
     * 
     * @note Called from transition state machine during forward and back transitions
     */
    bool update_yaw_target(float& yaw_target_cd) override;

    /**
     * @brief Determine if VTOL flight mode view should be displayed
     * 
     * @return true if should show VTOL mode display, false for fixed-wing display
     * 
     * @details Returns true when in VTOL modes or when tilts are not fully forward.
     *          Provides appropriate mode display and parameter set to pilot during transitions.
     * 
     * @note Affects which parameters (Q_ vs non-Q_) are shown to pilot
     * @note Affects flight mode name displayed on GCS
     */
    bool show_vtol_view() const override;

    /**
     * @brief Determine if multirotor control should be used during forward transition
     * 
     * @return true to use multirotor attitude control, false for fixed-wing control
     * 
     * @details Returns true while tilts still provide significant vertical thrust component.
     *          Ensures stable control authority during transition by using appropriate
     *          controller for current tilt configuration.
     * 
     * @note Affects which attitude controller is active during forward transition
     * @warning Incorrect switching can cause control authority loss during transition
     */
    bool use_multirotor_control_in_fwd_transition() const override;

private:

    /**
     * @brief Reference to tiltrotor control object
     * 
     * @note Provides access to current tilt angles and tilt state for transition decisions
     */
    Tiltrotor& tiltrotor;

};
