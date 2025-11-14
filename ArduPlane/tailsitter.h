/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file tailsitter.h
 * @brief Tailsitter aircraft configuration and transition management for ArduPlane
 * 
 * @details This file defines the Tailsitter and Tailsitter_Transition classes which handle
 *          the unique characteristics of tailsitter VTOL aircraft. Tailsitters are aircraft
 *          that take off and land vertically on their tail, then transition to conventional
 *          forward flight by pitching forward 90 degrees.
 * 
 *          The implementation supports two primary tailsitter configurations:
 *          - Control surface tailsitters: Use control surfaces (elevons, rudder) for VTOL control
 *          - Vectored thrust tailsitters: Use motor tilt or differential thrust for VTOL control
 * 
 *          Key responsibilities include:
 *          - Managing transitions between VTOL and forward flight modes
 *          - Handling different input types (plane-like vs multicopter-like controls)
 *          - Applying gain and speed scaling during transitions
 *          - Providing motor mixing for tailsitter-specific configurations
 * 
 * Source: ArduPlane/tailsitter.h
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include "transition.h"
#include <AP_Motors/AP_MotorsTailsitter.h>
#include <AP_Logger/LogStructure.h>

class QuadPlane;
class AP_MotorsMulticopter;
class Tailsitter_Transition;

/**
 * @class Tailsitter
 * @brief Manages tailsitter-specific VTOL aircraft configuration and behavior
 * 
 * @details The Tailsitter class handles the unique control and transition requirements
 *          of tailsitter VTOL aircraft. These aircraft take off and land vertically while
 *          resting on their tail, then transition to forward flight by pitching 90 degrees
 *          forward. This requires specialized handling for:
 * 
 *          - Transition management between VTOL and forward flight modes
 *          - Input mixing (supports both plane-style and body-frame roll inputs)
 *          - Gain and speed scaling based on airspeed and altitude
 *          - Motor control for vectored thrust configurations
 *          - Control surface management for control-surface-only configurations
 * 
 *          The class supports multiple tailsitter configurations:
 *          - Pure control surface: No motor vectoring, uses elevons/rudder for VTOL
 *          - Vectored thrust: Uses tilting motors or differential thrust
 *          - Hybrid: Combination of control surfaces and vectored thrust
 * 
 *          Coordinate frame considerations:
 *          - In VTOL mode, the aircraft body frame is vertical (nose up)
 *          - In forward flight, the aircraft body frame is horizontal
 *          - Transitions require careful management of attitude control frame changes
 * 
 * @note This class is tightly integrated with QuadPlane and requires proper
 *       initialization through the QuadPlane framework
 * 
 * @warning Tailsitter transitions are safety-critical operations. Improper
 *          configuration can result in loss of control during transition
 */
class Tailsitter
{
friend class QuadPlane;
friend class Plane;
public:

    /**
     * @brief Construct a new Tailsitter object
     * 
     * @param[in] _quadplane Reference to the parent QuadPlane object
     * @param[in] _motors Reference pointer to the motors object for VTOL control
     */
    /**
     * @brief Construct a new Tailsitter object
     * 
     * @param[in] _quadplane Reference to the parent QuadPlane object
     * @param[in] _motors Reference pointer to the motors object for VTOL control
     */
    Tailsitter(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors);

    /**
     * @brief Check if tailsitter functionality is enabled and ready
     * 
     * @return true if tailsitter is enabled (enable > 0) and setup is complete
     * @return false if tailsitter is disabled or setup incomplete
     * 
     * @note This must return true before tailsitter-specific features are active
     */
    bool enabled() const { return (enable > 0) && setup_complete;}

    /**
     * @brief Initialize and configure tailsitter-specific features
     * 
     * @details Performs one-time setup including:
     *          - Detecting control surface configuration (elevons, v-tail, etc.)
     *          - Initializing vectored thrust motors if applicable
     *          - Validating tailsitter-specific parameters
     *          - Creating transition state management object
     * 
     * @note Called during QuadPlane initialization, typically during arming checks
     */
    void setup();

    /**
     * @brief Check if this is a control-surface-only tailsitter
     * 
     * @return true if tailsitter uses only control surfaces (no vectored thrust)
     * @return false if tailsitter has vectored thrust capabilities
     * 
     * @details Control surface tailsitters rely entirely on aerodynamic surfaces
     *          (elevons, rudder) for VTOL control, similar to a 3D airplane.
     *          This affects control mixing and gain scheduling.
     */
    bool is_control_surface_tailsitter(void) const;

    /**
     * @brief Check if tailsitter VTOL control is currently active
     * 
     * @return true if in VTOL mode or transitioning to/from VTOL
     * @return false if in pure forward flight mode
     * 
     * @details Returns true when the aircraft is using tailsitter-specific
     *          VTOL control laws rather than conventional fixed-wing control.
     *          Used to switch between control modes.
     */
    bool active(void);
    
    /**
     * @brief Generate motor and servo outputs for tailsitter configuration
     * 
     * @details Creates appropriate outputs based on tailsitter type:
     *          - Control surface: Maps VTOL control to elevon/rudder/v-tail
     *          - Vectored thrust: Applies motor tilt and differential thrust
     *          - Applies gain scaling and speed scaling factors
     *          - Handles transition blending between VTOL and FW outputs
     * 
     * @note Called every control loop iteration when tailsitter is active
     * @warning Incorrect output configuration can cause control reversal
     */
    void output(void);

    /**
     * @brief Process and interpret pilot input based on configured input type
     * 
     * @details Handles different input conventions:
     *          - TAILSITTER_INPUT_PLANE: Traditional plane-like stick inputs
     *          - TAILSITTER_INPUT_BF_ROLL: Body-frame roll (multicopter-like)
     * 
     *          Input interpretation changes based on aircraft attitude and whether
     *          in VTOL or forward flight mode. Remaps pilot commands to appropriate
     *          control axes.
     * 
     * @note Called in the main flight control loop before attitude control
     */
    void check_input(void);

    /**
     * @brief Check if transition to fixed-wing flight is complete
     * 
     * @return true if aircraft has fully transitioned to forward flight
     * @return false if still transitioning or in VTOL mode
     * 
     * @details Transition is complete when aircraft pitch angle reaches
     *          the configured threshold (transition_angle_fw) and has stabilized.
     *          Used to switch from transition control laws to normal FW control.
     */
    bool transition_fw_complete(void);

    /**
     * @brief Check if aircraft is currently in forward flight mode
     * 
     * @return true if in fixed-wing flight (not VTOL, not transitioning)
     * @return false if in VTOL mode or transitioning
     * 
     * @details Used to determine which control laws and output mixing to apply.
     *          In FW flight, conventional fixed-wing control is used.
     */
    bool is_in_fw_flight(void) const;

    /**
     * @brief Check if transition to VTOL mode is complete
     * 
     * @return true if aircraft has fully transitioned to VTOL mode
     * @return false if still transitioning or in forward flight
     * 
     * @details Transition is complete when aircraft pitch angle reaches
     *          the configured threshold (transition_angle_vtol) and the aircraft
     *          is stable in vertical orientation.
     */
    bool transition_vtol_complete(void) const;

    /**
     * @brief Check if currently transitioning to VTOL flight
     * 
     * @param[in] now Current time in milliseconds (0 = use current time)
     * @return true if actively transitioning from FW to VTOL
     * @return false if not transitioning or transition complete
     * 
     * @details During VTOL transition, special control laws blend between
     *          forward flight and VTOL control to maintain stability while
     *          pitching the aircraft from horizontal to vertical attitude.
     */
    bool in_vtol_transition(uint32_t now = 0) const;

    /**
     * @brief Apply speed-based scaling to control surfaces during VTOL
     * 
     * @details Adjusts control surface effectiveness based on airspeed:
     *          - At low speeds (VTOL hover): Surfaces are less effective
     *          - As speed increases: Progressively increase surface authority
     *          - Multiple scaling methods supported via gain_scaling_mask
     * 
     *          Scaling methods include:
     *          - Throttle-based scaling
     *          - Disk loading theory (momentum theory)
     *          - Altitude-based scaling
     * 
     * @note Called before output generation to adjust control gains
     */
    void speed_scaling(void);

    /**
     * @brief Get the configured VTOL transition angle threshold
     * 
     * @return int8_t Pitch angle in degrees at which VTOL transition is complete
     * 
     * @details Returns the transition_angle_vtol parameter value. This is the
     *          pitch angle (typically near 90 degrees) at which the aircraft
     *          is considered to be in full VTOL orientation.
     */
    int8_t get_transition_angle_vtol() const;

    /**
     * @brief Determine if pitch control should be relaxed during transition
     * 
     * @return true if pitch control authority should be reduced
     * @return false if normal pitch control should be applied
     * 
     * @details During certain phases of transition, aggressive pitch control
     *          can cause instability. This function determines when to reduce
     *          pitch control authority for smoother transitions.
     * 
     * @note Used by attitude controller during transition phases
     */
    bool relax_pitch();

    /**
     * @brief Log tailsitter-specific telemetry data
     * 
     * @details Writes a tailsitter log message containing:
     *          - throttle_scaler: Current throttle scaling factor
     *          - speed_scaler: Current speed-based gain scaling factor
     *          - min_throttle: Minimum throttle value being used
     * 
     *          Used for post-flight analysis and tuning of tailsitter parameters.
     * 
     * @note Called periodically during flight when tailsitter is active
     */
    void write_log();

    /**
     * @brief Last applied speed scaler value for slew rate limiting
     * 
     * @details Stores the previously applied speed scaling factor to enable smooth
     *          transitions when speed scaling changes. Used with TAILSITTER_GSCL_ATT_THR
     *          option to slew rate limit gain changes and prevent sudden control changes.
     *          Range: 0.0 to 1.0 or higher
     */
    float last_spd_scaler = 1.0f;

    /**
     * @brief Parameter table definition for tailsitter configuration
     * 
     * @details Defines all user-configurable parameters for tailsitter operation.
     *          Parameters are accessible via ground control station and stored in EEPROM.
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @enum input
     * @brief Pilot input interpretation modes for tailsitter control
     * 
     * @details Defines how pilot stick inputs are interpreted in VTOL mode:
     * 
     * TAILSITTER_INPUT_PLANE:
     *   Traditional plane-like stick convention where pitch stick controls pitch
     *   regardless of aircraft orientation. More intuitive for fixed-wing pilots.
     * 
     * TAILSITTER_INPUT_BF_ROLL:
     *   Body-frame roll mode where roll stick always controls roll in body frame.
     *   More intuitive for multicopter pilots and for hover maneuvers.
     * 
     * These modes can be combined using bitwise OR.
     */
    enum input {
        TAILSITTER_INPUT_PLANE   = (1U<<0),  ///< Use plane-style input mapping
        TAILSITTER_INPUT_BF_ROLL = (1U<<1)   ///< Enable body-frame roll control
    };

    /**
     * @enum gscl_mask
     * @brief Gain scaling method selection bitmask
     * 
     * @details Defines available methods for scaling control gains based on flight
     *          conditions. Multiple methods can be combined using bitwise OR:
     * 
     * TAILSITTER_GSCL_THROTTLE:
     *   Scale gains based on throttle position. Simple but less accurate.
     * 
     * TAILSITTER_GSCL_ATT_THR:
     *   Scale gains based on attitude and throttle. Applies slew rate limiting
     *   to prevent sudden gain changes during transitions.
     * 
     * TAILSITTER_GSCL_DISK_THEORY:
     *   Use momentum (disk loading) theory to estimate propeller induced velocity
     *   and scale gains accordingly. Most accurate for hover performance.
     * 
     * TAILSITTER_GSCL_ALTITUDE:
     *   Adjust scaling based on altitude to compensate for air density changes.
     * 
     * @note Multiple scaling methods can be active simultaneously
     */
    enum gscl_mask {
        TAILSITTER_GSCL_THROTTLE = (1U<<0),      ///< Throttle-based gain scaling
        TAILSITTER_GSCL_ATT_THR = (1U<<1),       ///< Attitude and throttle scaling with slew limiting
        TAILSITTER_GSCL_DISK_THEORY = (1U<<2),   ///< Momentum theory based scaling
        TAILSITTER_GSCL_ALTITUDE = (1U<<3),      ///< Altitude compensated scaling
    };

    // Tailsitter Configuration Parameters
    // These parameters are user-configurable via ground control station

    /**
     * @brief Enable tailsitter functionality (0=disabled, 1=enabled, 2+=advanced modes)
     * @details Master enable for tailsitter-specific features. Must be non-zero for
     *          tailsitter control and transition management to be active.
     */
    AP_Int8 enable;

    /**
     * @brief Pitch angle threshold for fixed-wing transition completion (degrees)
     * @details When pitching forward from VTOL to FW flight, transition is complete
     *          when pitch reaches this angle. Typical value: 5-15 degrees
     *          (near horizontal). Lower values complete transition earlier.
     */
    AP_Int8 transition_angle_fw;

    /**
     * @brief Maximum pitch rate during transition to fixed-wing (deg/s)
     * @details Limits how fast the aircraft can pitch forward during FW transition.
     *          Prevents overly aggressive transitions that could cause loss of lift.
     *          Typical range: 10-30 deg/s
     */
    AP_Float transition_rate_fw;

    /**
     * @brief Pitch angle threshold for VTOL transition completion (degrees)
     * @details When pitching backward from FW to VTOL flight, transition is complete
     *          when pitch reaches this angle. Typical value: 75-90 degrees
     *          (near vertical). Higher values ensure full VTOL orientation.
     */
    AP_Int8 transition_angle_vtol;

    /**
     * @brief Maximum pitch rate during transition to VTOL (deg/s)
     * @details Limits how fast the aircraft can pitch backward during VTOL transition.
     *          Slower rates provide more controlled transitions but take longer.
     *          Typical range: 10-30 deg/s
     */
    AP_Float transition_rate_vtol;

    /**
     * @brief Throttle level to apply during VTOL transition (0.0-1.0)
     * @details Fixed throttle value used while transitioning from FW to VTOL.
     *          Higher values provide more lift authority during transition.
     *          Typical range: 0.5-0.9. Too low may cause altitude loss.
     */
    AP_Float transition_throttle_vtol;

    /**
     * @brief Input type bitmask (see input enum)
     * @details Controls how pilot inputs are interpreted. Bitwise combination of:
     *          - TAILSITTER_INPUT_PLANE (bit 0): Plane-like inputs
     *          - TAILSITTER_INPUT_BF_ROLL (bit 1): Body-frame roll mode
     */
    AP_Int8 input_type;

    /**
     * @brief Vectored thrust gain in forward flight (0.0-1.0)
     * @details For vectored thrust tailsitters, controls how much motor tilt or
     *          differential thrust is used to supplement control surfaces in FW flight.
     *          0.0 = no vectoring, 1.0 = full vectoring authority.
     */
    AP_Float vectored_forward_gain;

    /**
     * @brief Vectored thrust gain in hover (0.0-1.0)
     * @details Controls thrust vectoring authority in VTOL hover mode. Higher values
     *          provide more aggressive control response but may cause oscillations
     *          if too high. Typical range: 0.3-1.0
     */
    AP_Float vectored_hover_gain;

    /**
     * @brief Hover throttle exponent for vectored thrust calculation
     * @details Non-linear scaling factor applied to vectored thrust commands.
     *          Values > 1.0 provide more authority at high throttle, < 1.0 provides
     *          more authority at low throttle. Typical value: 2.5
     */
    AP_Float vectored_hover_power;

    /**
     * @brief Maximum throttle scaling factor in VTOL modes
     * @details Upper limit for throttle boost applied during VTOL flight to compensate
     *          for reduced efficiency in hover. Prevents excessive throttle.
     *          Typical range: 1.0-2.0. Values > 1.0 allow throttle boosting.
     */
    AP_Float throttle_scale_max;

    /**
     * @brief Minimum gain scaling factor (0.0-1.0)
     * @details Lower limit for speed/throttle-based gain scaling. Prevents control
     *          authority from becoming too weak at low speeds. Should be > 0 to
     *          maintain some control authority. Typical range: 0.1-0.5
     */
    AP_Float gain_scaling_min;

    /**
     * @brief Maximum roll angle in VTOL modes (degrees)
     * @details Limits the maximum roll angle commanded during VTOL flight to prevent
     *          loss of vertical thrust. 0 = no limit. Typical range: 30-60 degrees.
     *          Lower values are more conservative.
     */
    AP_Float max_roll_angle;

    /**
     * @brief Bitmask of motors to use for yaw control in VTOL
     * @details For multi-motor tailsitters, specifies which motors participate in
     *          yaw control via differential thrust. Each bit represents one motor.
     *          0 = auto-detect from motor configuration.
     */
    AP_Int16 motor_mask;

    /**
     * @brief Minimum airspeed for gain scaling (m/s)
     * @details Airspeed below which minimum gain scaling is applied. Below this speed,
     *          controls operate at minimum effectiveness. Typical value: 0-5 m/s.
     *          Used with speed-based scaling methods.
     */
    AP_Float scaling_speed_min;

    /**
     * @brief Maximum airspeed for gain scaling (m/s)
     * @details Airspeed above which full gain scaling is applied. Above this speed,
     *          controls operate at maximum effectiveness. Typical value: 10-20 m/s.
     *          Used with speed-based scaling methods.
     */
    AP_Float scaling_speed_max;

    /**
     * @brief Gain scaling method selection bitmask (see gscl_mask enum)
     * @details Bitwise combination of scaling methods:
     *          - TAILSITTER_GSCL_THROTTLE: Throttle-based
     *          - TAILSITTER_GSCL_ATT_THR: Attitude and throttle with slew limiting
     *          - TAILSITTER_GSCL_DISK_THEORY: Momentum theory
     *          - TAILSITTER_GSCL_ALTITUDE: Altitude compensated
     */
    AP_Int16 gain_scaling_mask;

    /**
     * @brief Disk loading for momentum theory calculations (kg/m^2)
     * @details Propeller disk loading (aircraft weight / propeller disk area) used
     *          for momentum theory based gain scaling. Higher values indicate
     *          higher wing loading. Typical range: 20-200 kg/m^2.
     */
    AP_Float disk_loading;

    /**
     * @brief Roll control scaling factor in VTOL modes
     * @details Multiplier applied to roll control outputs in VTOL. Values > 1.0
     *          increase roll authority, < 1.0 decrease it. Used to tune control
     *          response. Typical range: 0.5-2.0
     */
    AP_Float VTOL_roll_scale;

    /**
     * @brief Pitch control scaling factor in VTOL modes
     * @details Multiplier applied to pitch control outputs in VTOL. Values > 1.0
     *          increase pitch authority, < 1.0 decrease it. Used to tune control
     *          response. Typical range: 0.5-2.0
     */
    AP_Float VTOL_pitch_scale;

    /**
     * @brief Yaw control scaling factor in VTOL modes
     * @details Multiplier applied to yaw control outputs in VTOL. Values > 1.0
     *          increase yaw authority, < 1.0 decrease it. Used to tune control
     *          response. Typical range: 0.5-2.0
     */
    AP_Float VTOL_yaw_scale;

    /**
     * @brief Minimum outflow velocity for disk loading calculations (m/s)
     * @details Minimum induced velocity used in momentum theory calculations to
     *          prevent division by zero and unrealistic gain scaling at very low
     *          throttle. Typical value: 1-3 m/s.
     */
    AP_Float disk_loading_min_outflow;

    /**
     * @brief Pointer to tailsitter-specific motor control object
     * @details Manages motor mixing and output for tailsitter configurations.
     *          Handles both traditional multicopter-style motor arrangements
     *          and custom tailsitter motor configurations.
     */
    AP_MotorsTailsitter* tailsitter_motors;

private:

    /**
     * @struct log_tailsitter
     * @brief Binary log message structure for tailsitter telemetry
     * 
     * @details Defines the format of tailsitter-specific log messages written to
     *          dataflash. Used for post-flight analysis and parameter tuning.
     */
    struct PACKED log_tailsitter {
        LOG_PACKET_HEADER;
        uint64_t time_us;           ///< Timestamp in microseconds
        float throttle_scaler;      ///< Current throttle scaling factor
        float speed_scaler;         ///< Current speed-based gain scaling factor
        float min_throttle;         ///< Minimum throttle value being applied
    };

    /**
     * @brief Staging area for log data before writing to dataflash
     * @details Accumulates telemetry data during the control loop, which is then
     *          written to the log file by write_log() method.
     */
    struct {
        float throttle_scaler;
        float speed_scaler;
        float min_throttle;
    } log_data;

    /**
     * @brief Flag indicating tailsitter setup has been completed
     * @details Set to true after successful completion of setup() method.
     *          Used in conjunction with enable parameter to determine if
     *          tailsitter functionality is ready for use.
     */
    bool setup_complete;

    /**
     * @brief True when aircraft is configured with vectored thrust
     * @details Indicates whether the tailsitter uses motor tilt or differential
     *          thrust for VTOL control. Affects control mixing and output generation.
     */
    bool _is_vectored;

    /**
     * @brief True if elevator control surface is configured
     * @details Indicates availability of dedicated elevator output for pitch control
     */
    bool _have_elevator;

    /**
     * @brief True if aileron control surfaces are configured
     * @details Indicates availability of dedicated aileron outputs for roll control
     */
    bool _have_aileron;

    /**
     * @brief True if rudder control surface is configured
     * @details Indicates availability of rudder output for yaw control
     */
    bool _have_rudder;

    /**
     * @brief True if elevon control surfaces are configured
     * @details Indicates elevon (combined elevator/aileron) mixing is in use
     */
    bool _have_elevon;

    /**
     * @brief True if V-tail control surfaces are configured
     * @details Indicates V-tail (combined elevator/rudder) mixing is in use
     */
    bool _have_v_tail;

    /**
     * @brief Reference to parent QuadPlane object
     * @details Provides access to QuadPlane state, parameters, and functionality
     */
    QuadPlane& quadplane;

    /**
     * @brief Reference to motors object pointer for VTOL control
     * @details Points to the motor control object used for multicopter-style
     *          VTOL thrust management
     */
    AP_MotorsMulticopter*& motors;

    /**
     * @brief Pointer to transition state management object
     * @details Manages the transition logic between VTOL and fixed-wing flight modes.
     *          Created during setup() and handles transition timing and sequencing.
     */
    Tailsitter_Transition* transition;

};


/**
 * @class Tailsitter_Transition
 * @brief Manages transition state machine and control for tailsitter aircraft
 * 
 * @details This class implements the transition logic specific to tailsitter aircraft,
 *          which must pitch through 90 degrees when transitioning between VTOL and
 *          forward flight modes. It inherits from the base Transition class and
 *          provides tailsitter-specific implementations of transition behavior.
 * 
 *          Transition Phases:
 *          1. TRANSITION_ANGLE_WAIT_FW: Pitching forward from VTOL to FW, waiting
 *             to reach target pitch angle
 *          2. TRANSITION_ANGLE_WAIT_VTOL: Pitching backward from FW to VTOL, waiting
 *             to reach target pitch angle
 *          3. TRANSITION_DONE: Transition complete, in stable VTOL or FW flight
 * 
 *          The class manages:
 *          - Pitch angle progression with rate limiting
 *          - Control blending between VTOL and FW modes
 *          - Throttle management during transitions
 *          - Attitude limits during transition phases
 * 
 * @note Transitions are safety-critical. The state machine ensures the aircraft
 *       maintains sufficient airspeed and thrust throughout the transition.
 * 
 * @warning Improper transition rates or angles can result in stalls or loss of control
 */
class Tailsitter_Transition : public Transition
{
friend class Tailsitter;
public:

    /**
     * @brief Construct a new Tailsitter_Transition object
     * 
     * @param[in] _quadplane Reference to parent QuadPlane object
     * @param[in] _motors Reference to motors object for VTOL control
     * @param[in] _tailsitter Reference to parent Tailsitter object
     */
    Tailsitter_Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors, Tailsitter& _tailsitter):Transition(_quadplane, _motors), tailsitter(_tailsitter) {};

    /**
     * @brief Update transition state for forward flight transitions
     * 
     * @details Called each control loop iteration during FW transition. Manages:
     *          - Pitch angle progression toward horizontal
     *          - Transition completion detection
     *          - Control mode switching
     * 
     * @note Implements rate-limited pitch changes based on transition_rate_fw
     */
    void update() override;

    /**
     * @brief Update transition state for VTOL transitions
     * 
     * @details Called each control loop iteration during VTOL transition. Manages:
     *          - Pitch angle progression toward vertical
     *          - Throttle management during transition
     *          - Transition completion detection
     * 
     * @note Implements rate-limited pitch changes based on transition_rate_vtol
     */
    void VTOL_update() override;

    /**
     * @brief Force immediate completion of current transition
     * 
     * @details Bypasses normal transition sequencing and immediately marks
     *          transition as complete. Used in emergency situations or when
     *          manual override is required.
     * 
     * @warning Should only be used when necessary as it bypasses safety checks
     */
    void force_transition_complete() override;

    /**
     * @brief Check if transition is complete
     * 
     * @return true if transition_state is TRANSITION_DONE
     * @return false if still transitioning
     * 
     * @details Used by flight mode logic to determine when to fully switch
     *          from transition control laws to normal flight control laws.
     */
    bool complete() const override { return transition_state == TRANSITION_DONE; }

    /**
     * @brief Initialize and start transition back to fixed wing
     * 
     * @details Sets up state machine for FW transition:
     *          - Records initial pitch angle
     *          - Sets transition_state to TRANSITION_ANGLE_WAIT_FW
     *          - Initializes timing variables
     * 
     * @note Called when pilot commands transition from VTOL to FW mode
     */
    void restart() override;

    /**
     * @brief Get current transition state for logging
     * 
     * @return uint8_t Numeric representation of current transition_state
     * 
     * @details Returns transition state as integer for dataflash logging.
     *          Used for post-flight analysis of transition behavior.
     */
    uint8_t get_log_transition_state() const override { return static_cast<uint8_t>(transition_state); }

    /**
     * @brief Check if actively transitioning forward to FW flight
     * 
     * @return true if transition_state is TRANSITION_ANGLE_WAIT_FW
     * @return false otherwise
     * 
     * @details Used to determine if forward transition control laws should be applied
     */
    bool active_frwd() const override { return transition_state == TRANSITION_ANGLE_WAIT_FW; }

    /**
     * @brief Determine if VTOL-oriented view should be shown to pilot
     * 
     * @return true if GCS should display VTOL-oriented attitude indicators
     * @return false if GCS should display FW-oriented attitude indicators
     * 
     * @details Controls how attitude is displayed in ground station to match
     *          aircraft orientation. Switches based on transition progress.
     */
    bool show_vtol_view() const override;

    /**
     * @brief Set desired roll and pitch angles for fixed-wing flight during transition
     * 
     * @param[in,out] nav_pitch_cd Desired pitch angle in centidegrees
     * @param[in,out] nav_roll_cd Desired roll angle in centidegrees
     * 
     * @details Modifies commanded attitude angles during FW transition to blend
     *          between VTOL and FW control. Ensures smooth transition without
     *          abrupt attitude changes.
     */
    void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd) override;

    /**
     * @brief Determine if pilot stick mixing is allowed during transition
     * 
     * @return true if pilot inputs should be mixed with transition control
     * @return false if transition control should override pilot inputs
     * 
     * @details During critical phases of transition, pilot inputs may be limited
     *          to prevent interference with automatic transition control.
     */
    bool allow_stick_mixing() const override;

    /**
     * @brief Get MAVLink VTOL state for telemetry reporting
     * 
     * @return MAV_VTOL_STATE Current VTOL state for MAVLink SYS_STATUS message
     * 
     * @details Returns appropriate MAVLink VTOL state enum value based on
     *          current transition state. Used for ground station telemetry.
     */
    MAV_VTOL_STATE get_mav_vtol_state() const override;

    /**
     * @brief Apply roll and pitch limits appropriate for VTOL transition
     * 
     * @param[in,out] nav_roll_cd Desired roll angle in centidegrees (limited on output)
     * @param[in,out] nav_pitch_cd Desired pitch angle in centidegrees (limited on output)
     * 
     * @return true if limits were applied
     * @return false if no limiting was necessary
     * 
     * @details Restricts commanded attitudes during VTOL transition to prevent
     *          excessive angles that could cause loss of vertical thrust or control.
     *          Uses max_roll_angle parameter and transition-specific pitch limits.
     */
    bool set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd) override;

    /**
     * @brief Determine if weathervane control is allowed during transition
     * 
     * @return true if weathervane (yaw into wind) is allowed
     * @return false if weathervane should be disabled
     * 
     * @details Weathervane automatically yaws aircraft into wind during VTOL flight.
     *          May be disabled during certain transition phases to avoid interference
     *          with transition control.
     */
    bool allow_weathervane() override;

private:

    /**
     * @enum transition_state
     * @brief Tailsitter transition state machine states
     * 
     * @details Defines the current phase of transition:
     * 
     * TRANSITION_ANGLE_WAIT_FW:
     *   Actively transitioning from VTOL to fixed-wing. Aircraft is pitching
     *   forward from vertical to horizontal orientation. Waiting to reach
     *   transition_angle_fw threshold.
     * 
     * TRANSITION_ANGLE_WAIT_VTOL:
     *   Actively transitioning from fixed-wing to VTOL. Aircraft is pitching
     *   backward from horizontal to vertical orientation. Waiting to reach
     *   transition_angle_vtol threshold.
     * 
     * TRANSITION_DONE:
     *   No active transition. Aircraft is in stable VTOL or FW flight mode.
     */
    enum {
        TRANSITION_ANGLE_WAIT_FW,      ///< Transitioning to fixed-wing flight
        TRANSITION_ANGLE_WAIT_VTOL,    ///< Transitioning to VTOL flight
        TRANSITION_DONE                ///< Transition complete
    } transition_state;

    /**
     * @brief Timestamp when VTOL transition started (milliseconds)
     * @details Used to calculate elapsed time during VTOL transition for rate limiting
     *          and timeout detection. Set when transition to VTOL is initiated.
     */
    uint32_t vtol_transition_start_ms;

    /**
     * @brief Initial pitch angle when VTOL transition started (degrees)
     * @details Records starting pitch to calculate pitch change progress during
     *          rate-limited transition. Used with vtol_transition_start_ms to
     *          enforce transition_rate_vtol.
     */
    float vtol_transition_initial_pitch;

    /**
     * @brief Timestamp when VTOL rate limiting started (milliseconds)
     * @details Used for managing rate-limited pitch changes during VTOL stabilization
     *          after transition completes.
     */
    uint32_t vtol_limit_start_ms;

    /**
     * @brief Initial pitch angle when VTOL rate limiting started (degrees)
     * @details Starting pitch for rate-limited attitude changes after VTOL transition
     */
    float vtol_limit_initial_pitch;

    /**
     * @brief Timestamp when FW rate limiting started (milliseconds)
     * @details Used for managing rate-limited pitch changes during FW stabilization
     *          after transition completes.
     */
    uint32_t fw_limit_start_ms;

    /**
     * @brief Initial pitch angle when FW rate limiting started (degrees)
     * @details Starting pitch for rate-limited attitude changes after FW transition
     */
    float fw_limit_initial_pitch;

    /**
     * @brief Timestamp when FW transition started (milliseconds)
     * @details Used to calculate elapsed time during FW transition for rate limiting
     *          and timeout detection. Set when transition to FW is initiated.
     */
    uint32_t fw_transition_start_ms;

    /**
     * @brief Initial pitch angle when FW transition started (degrees)
     * @details Records starting pitch to calculate pitch change progress during
     *          rate-limited transition. Used with fw_transition_start_ms to
     *          enforce transition_rate_fw.
     */
    float fw_transition_initial_pitch;

    /**
     * @brief Timestamp of last time aircraft was in a VTOL control mode (milliseconds)
     * @details Used to determine if aircraft recently exited VTOL mode. Affects
     *          transition behavior and timeout logic. Updated whenever VTOL
     *          control is active.
     */
    uint32_t last_vtol_mode_ms;

    /**
     * @brief Reference to parent Tailsitter object
     * @details Provides access to tailsitter configuration, parameters, and state.
     *          Used to query tailsitter-specific settings during transition.
     */
    Tailsitter& tailsitter;

};
