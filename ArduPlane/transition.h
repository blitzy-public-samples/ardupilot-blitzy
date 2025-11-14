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
 * @file transition.h
 * @brief VTOL transition state machine classes for QuadPlane mode changes
 * 
 * @details This file defines the transition state machine framework used by ArduPlane's
 *          QuadPlane to manage transitions between VTOL (vertical takeoff and landing)
 *          and fixed-wing flight modes. Transitions are critical phases where the aircraft
 *          changes from multicopter control to fixed-wing control (forward transition)
 *          or vice versa (back transition).
 * 
 *          The transition state machine manages:
 *          - Airspeed requirements for safe mode transitions
 *          - Motor thrust blending during transition phases
 *          - Control surface authority handoff between VTOL and FW controllers
 *          - Tilt mechanism control for tiltrotor configurations
 *          - Roll/pitch/yaw control authority during hybrid flight phases
 * 
 *          Architecture:
 *          - Transition: Abstract base class defining the transition interface
 *          - SLT_Transition: Implementation for Separate Lift Thrust quadplanes
 *          - Additional transition types defined in separate files (tailsitter, tiltrotor)
 * 
 * @note Transitions are safety-critical operations requiring careful coordination
 *       between multiple control systems and propulsion elements
 * 
 * @warning Improper transition logic can result in loss of control during hybrid flight
 * 
 * @see QuadPlane class for integration with main vehicle control
 * @see ArduPlane/mode_qstabilize.cpp for VTOL mode implementations
 * @see ArduPlane/quadplane.cpp for QuadPlane initialization and management
 */
#pragma once
#include <GCS_MAVLink/GCS_MAVLink.h>

class QuadPlane;
class AP_MotorsMulticopter;

/**
 * @class Transition
 * @brief Abstract base class for QuadPlane VTOL-to-fixed-wing transition state machines
 * 
 * @details This class defines the interface for managing transitions between VTOL
 *          (multicopter) and fixed-wing flight modes in QuadPlane configurations.
 *          Transitions are complex, time-critical operations that require careful
 *          coordination of:
 *          - Lift motor thrust ramping (for separate lift thrust configurations)
 *          - Forward motor/propeller thrust application
 *          - Control surface authority blending
 *          - Tilt mechanism control (for tiltrotor configurations)
 *          - Airspeed monitoring and validation
 * 
 *          The transition state machine is called during hybrid flight phases where
 *          the vehicle operates partially as a multicopter and partially as a
 *          fixed-wing aircraft. Different QuadPlane configurations (separate lift
 *          thrust, tailsitter, tiltrotor) require different transition strategies,
 *          hence the abstract base class design.
 * 
 *          Lifecycle:
 *          1. Transition initiated when mode change requires switch between VTOL/FW
 *          2. update() called at main loop rate during forward transitions
 *          3. VTOL_update() called during back transitions to VTOL
 *          4. complete() indicates transition finished and full mode authority achieved
 * 
 * @note Derived classes must implement vehicle-specific transition logic
 * @warning Transition state directly affects flight safety - careful testing required
 * 
 * @see SLT_Transition for separate lift thrust implementation
 * @see QuadPlane::update() for transition state machine integration
 */
class Transition
{
public:

    /**
     * @brief Construct a new Transition state machine
     * 
     * @param[in] _quadplane Reference to the QuadPlane instance managing this transition
     * @param[in] _motors Reference to the multicopter motors object for VTOL thrust control
     * 
     * @note Constructor stores references to avoid pointer dereferencing overhead in
     *       time-critical transition update functions
     */
    Transition(QuadPlane& _quadplane, AP_MotorsMulticopter*& _motors):quadplane(_quadplane),motors(_motors) {};

    /**
     * @brief Update transition state during forward transition (VTOL to fixed-wing)
     * 
     * @details Called at main loop rate when transitioning from VTOL to fixed-wing mode.
     *          Manages the progressive handoff from multicopter control to fixed-wing
     *          control, including:
     *          - Monitoring airspeed to determine when FW flight is sustainable
     *          - Ramping down VTOL motor thrust as forward airspeed increases
     *          - Blending control authority between VTOL and FW controllers
     *          - Managing tilt mechanisms for tiltrotor configurations
     * 
     * @note Called at high frequency (typically 400Hz) - must be computationally efficient
     * @warning Must ensure vehicle maintains control authority throughout transition
     */
    virtual void update() = 0;

    /**
     * @brief Update transition state during back transition (fixed-wing to VTOL)
     * 
     * @details Called at main loop rate when transitioning from fixed-wing to VTOL mode.
     *          Manages the reverse transition, progressively increasing VTOL motor
     *          authority while reducing reliance on forward airspeed for control.
     *          Critical for safe landings and RTL operations.
     * 
     * @note Called at high frequency - must handle varying airspeeds and attitudes
     * @warning Back transition at low altitude requires careful thrust management
     */
    virtual void VTOL_update() = 0;

    /**
     * @brief Force immediate completion of transition regardless of current state
     * 
     * @details Used in emergency situations or when pilot manually overrides normal
     *          transition logic. Immediately sets transition to complete state and
     *          hands full control authority to the target flight mode.
     * 
     * @warning Can result in abrupt control changes - use only when necessary
     * @see complete() to check if transition finished normally
     */
    virtual void force_transition_complete() = 0;

    /**
     * @brief Check if transition has completed successfully
     * 
     * @return true if transition finished and full control authority transferred
     * @return false if still transitioning between modes
     * 
     * @note Used by mode controllers to determine when to apply full flight mode logic
     */
    virtual bool complete() const = 0;

    /**
     * @brief Restart the transition state machine from initial state
     * 
     * @details Resets all transition state variables and timers to begin a new
     *          transition sequence. Called when initiating a new mode change that
     *          requires transition management.
     * 
     * @note Does not change current motor outputs - only resets state tracking
     */
    virtual void restart() = 0;

    /**
     * @brief Get current transition state for logging purposes
     * 
     * @return uint8_t Numeric representation of current transition state for dataflash logs
     * 
     * @details Provides transition state value for QTUN log messages, enabling
     *          post-flight analysis of transition behavior and timing.
     * 
     * @note State encoding is implementation-specific to each derived class
     * @see AP_Logger for log message definitions
     */
    virtual uint8_t get_log_transition_state() const = 0;

    /**
     * @brief Check if forward transition is currently active
     * 
     * @return true if transitioning from VTOL to fixed-wing mode
     * @return false if not in forward transition (back transition or complete)
     * 
     * @details Used by control systems to determine which transition update path
     *          to execute and how to blend control authority between modes.
     */
    virtual bool active_frwd() const = 0;

    /**
     * @brief Determine whether to show VTOL view in ground station
     * 
     * @return true if GCS should display VTOL-style attitude indicators
     * @return false if GCS should display fixed-wing attitude indicators
     * 
     * @details Controls MAVLink messages to ground control stations to provide
     *          appropriate display mode during transitions. Typically returns true
     *          when vehicle is primarily using VTOL control authority.
     */
    virtual bool show_vtol_view() const = 0;

    /**
     * @brief Modify fixed-wing roll and pitch commands during transition
     * 
     * @param[in,out] nav_pitch_cd Navigation pitch command in centidegrees, modified by transition logic
     * @param[in,out] nav_roll_cd Navigation roll command in centidegrees, modified by transition logic
     * 
     * @details Allows transition state machine to adjust fixed-wing control surface
     *          commands to ensure smooth handoff from VTOL control. May limit rates
     *          of change or blend commands with previous VTOL attitudes.
     * 
     * @note Default implementation does nothing - override for specific transition behavior
     */
    virtual void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd) {};

    /**
     * @brief Set roll angle limit during transition
     * 
     * @param[in,out] roll_limit_cd Maximum roll angle in centidegrees, potentially modified by transition
     * 
     * @return true if roll limit was modified by transition logic
     * @return false if default roll limits should be used
     * 
     * @details Some transitions require restricted roll authority to maintain stability
     *          during critical phases. Default returns false (no special limits).
     */
    virtual bool set_FW_roll_limit(int32_t& roll_limit_cd) { return false; }

    /**
     * @brief Check if throttle mix can be updated during current transition phase
     * 
     * @return true if throttle mixing between VTOL and FW motors can be adjusted
     * @return false if throttle mix should be held constant during critical transition phase
     * 
     * @details Throttle mix determines the balance between lift motors and forward thrust.
     *          During some transition phases, mix must be held constant for stability.
     *          Default returns true (allow updates).
     */
    virtual bool allow_update_throttle_mix() const { return true; }

    /**
     * @brief Update yaw target angle during transition
     * 
     * @param[in,out] yaw_target_cd Desired yaw angle in centidegrees, potentially modified by transition
     * 
     * @return true if yaw target was modified by transition logic
     * @return false if default yaw control should be used
     * 
     * @details Some transitions (especially tailsitter) require specific yaw angles
     *          during transition phases. Default returns false (no yaw modification).
     */
    virtual bool update_yaw_target(float& yaw_target_cd) { return false; }

    /**
     * @brief Get current MAVLink VTOL state for telemetry reporting
     * 
     * @return MAV_VTOL_STATE Current VTOL state per MAVLink enum definition
     * 
     * @details Reports transition state to ground control station using standard
     *          MAVLink VTOL state enumeration. States include:
     *          - MAV_VTOL_STATE_TRANSITION_TO_FW: Forward transition active
     *          - MAV_VTOL_STATE_TRANSITION_TO_MC: Back transition active  
     *          - MAV_VTOL_STATE_FW: Fixed-wing mode
     *          - MAV_VTOL_STATE_MC: Multicopter mode
     * 
     * @note Used in EXTENDED_SYS_STATE MAVLink message
     * @see GCS_MAVLink for telemetry message definitions
     */
    virtual MAV_VTOL_STATE get_mav_vtol_state() const = 0;

    /**
     * @brief Set roll and pitch angle limits during VTOL phases of transition
     * 
     * @param[in,out] nav_roll_cd Navigation roll in centidegrees, potentially limited
     * @param[in,out] nav_pitch_cd Navigation pitch in centidegrees, potentially limited
     * 
     * @return true if limits were applied by transition logic
     * @return false if default VTOL limits should be used
     * 
     * @details Allows transition to restrict attitude angles during hybrid flight phases
     *          for safety and stability. Default returns false (no special limits).
     */
    virtual bool set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd) { return false; }

    /**
     * @brief Check if weathervane yaw control is allowed during transition
     * 
     * @return true if weathervane (wind-aligned yaw) can be active
     * @return false if yaw must be controlled manually or held constant
     * 
     * @details Weathervaning aligns vehicle with wind to minimize drift during VTOL
     *          operations. Some transition phases require fixed heading, disabling
     *          weathervane. Default returns true (weathervane allowed).
     * 
     * @see AC_AttitudeControl for weathervane implementation
     */
    virtual bool allow_weathervane() { return true; }

    /**
     * @brief Store current fixed-wing pitch for transition reference
     * 
     * @details Called when entering transition to capture the current FW mode pitch
     *          attitude. This reference is used during transition to ensure smooth
     *          attitude changes when blending between control modes.
     * 
     * @note Default implementation does nothing - override if FW pitch tracking needed
     */
    virtual void set_last_fw_pitch(void) {}

    /**
     * @brief Check if manual stick input mixing is allowed during transition
     * 
     * @return true if pilot stick inputs should be mixed into transition control
     * @return false if transition should be fully automated without pilot input
     * 
     * @details During critical transition phases, pilot inputs may be restricted to
     *          prevent interference with automated transition sequencing. Default
     *          returns true (stick mixing allowed).
     * 
     * @note Applies to transitions in automated modes (AUTO, GUIDED)
     */
    virtual bool allow_stick_mixing() const { return true; }

    /**
     * @brief Check if multirotor control should be used during forward transition
     * 
     * @return true if VTOL motors should provide primary control authority during forward transition
     * @return false if fixed-wing controls should be primary during forward transition
     * 
     * @details Some QuadPlane configurations (e.g., tailsitters) use multirotor control
     *          authority even during forward transition until sufficient airspeed is
     *          achieved. Default returns false (FW control during forward transition).
     */
    virtual bool use_multirotor_control_in_fwd_transition() const { return false; }

protected:

    /**
     * @brief Reference to the QuadPlane instance managing this transition
     * 
     * @details Provides access to QuadPlane parameters, state, and subsystems
     *          needed for transition logic. Stored as reference for efficiency
     *          in time-critical update functions.
     */
    QuadPlane& quadplane;
    
    /**
     * @brief Reference to multicopter motors object for VTOL thrust control
     * 
     * @details Provides access to motor output control during VTOL and transition
     *          phases. Stored as reference to pointer to allow QuadPlane to update
     *          the motors pointer while maintaining access through this reference.
     */
    AP_MotorsMulticopter*& motors;

};

/**
 * @class SLT_Transition
 * @brief Transition state machine for Separate Lift Thrust (SLT) QuadPlane configurations
 * 
 * @details Implements transition logic for QuadPlanes with separate lift motors (vertical thrust)
 *          and forward propulsion (pusher/puller prop or forward-tilting motors). This is the
 *          most common QuadPlane configuration.
 * 
 *          Transition Phases:
 *          
 *          Forward Transition (VTOL to FW):
 *          1. TRANSITION_AIRSPEED_WAIT: Accelerate with both lift and forward thrust until
 *             target airspeed achieved. Lift motors provide altitude hold, forward motor
 *             accelerates the aircraft.
 *          2. TRANSITION_TIMER: Ramp down lift motor thrust over configured time period
 *             while maintaining altitude with increasing wing lift from airspeed.
 *          3. TRANSITION_DONE: Full fixed-wing flight, lift motors off.
 * 
 *          Back Transition (FW to VTOL):
 *          - Progressive increase of lift motor thrust as airspeed decreases
 *          - Smooth handoff from wing lift to motor thrust for altitude control
 *          - Throttle blending to maintain consistent vertical thrust
 * 
 *          Key Features:
 *          - Airspeed monitoring to ensure safe transition timing
 *          - Time-based motor thrust ramping for smooth transitions
 *          - Tiltrotor support with tilt angle coordination
 *          - Pitch attitude blending during control authority handoff
 *          - Emergency forced transition capability
 * 
 * @note SLT configuration requires Q_TRANS_DECEL parameter tuning for back transitions
 * @warning Insufficient airspeed during forward transition can cause loss of altitude
 * 
 * @see QuadPlane::update_transition() for transition state machine integration
 * @see ArduPlane/quadplane.cpp for SLT QuadPlane initialization
 */
class SLT_Transition : public Transition
{
public:

    /**
     * @brief Inherit constructor from Transition base class
     * 
     * @details Uses base class constructor - no additional initialization needed
     *          for SLT-specific transition state. State variables initialized to
     *          default values by C++ default initialization.
     */
    using Transition::Transition;

    /**
     * @brief Update forward transition state (VTOL to fixed-wing)
     * 
     * @details Manages the three-phase forward transition:
     *          1. Wait for target airspeed while accelerating with forward motor
     *          2. Ramp down lift motors over transition timer period
     *          3. Complete transition to full FW mode
     * 
     *          During transition, maintains altitude using lift motors while forward
     *          airspeed builds. Once airspeed sufficient, progressively reduces lift
     *          motor thrust as wing provides increasing lift.
     * 
     * @note Called at main loop rate (typically 400Hz)
     * @warning Low airspeed at transition start can delay transition and waste battery
     * 
     * @see Q_TRANSITION_MS parameter for transition timer duration
     */
    void update() override;

    /**
     * @brief Update back transition state (fixed-wing to VTOL)
     * 
     * @details Manages progressive increase of lift motor thrust as vehicle decelerates
     *          from FW to VTOL mode. Smoothly blends motor thrust to maintain altitude
     *          as wing lift decreases with reducing airspeed.
     * 
     * @note Uses Q_TRANS_DECEL parameter to determine motor thrust ramp rate
     * @warning Aggressive deceleration without sufficient motor thrust can cause altitude loss
     */
    void VTOL_update() override;

    /**
     * @brief Force immediate completion of transition
     * 
     * @details Sets transition state directly to TRANSITION_DONE, bypassing normal
     *          airspeed and timer checks. Used when pilot manually overrides transition
     *          or in emergency situations requiring immediate mode authority transfer.
     * 
     * @warning Can result in abrupt thrust and control changes
     */
    void force_transition_complete() override;

    /**
     * @brief Check if transition has completed
     * 
     * @return true if transition_state is TRANSITION_DONE
     * @return false if still in TRANSITION_AIRSPEED_WAIT or TRANSITION_TIMER
     * 
     * @details Used by mode controllers to determine when full fixed-wing or VTOL
     *          control authority has been achieved.
     */
    bool complete() const override { return transition_state == TRANSITION_DONE; }

    /**
     * @brief Restart transition from initial airspeed wait state
     * 
     * @details Resets transition_state to TRANSITION_AIRSPEED_WAIT to begin a new
     *          forward transition sequence. Clears transition timers and state tracking.
     * 
     * @note Called when initiating forward transition from VTOL mode
     */
    void restart() override { transition_state = TRANSITION_AIRSPEED_WAIT; }

    /**
     * @brief Get numeric transition state for logging
     * 
     * @return uint8_t Transition state enum value (0=AIRSPEED_WAIT, 1=TIMER, 2=DONE)
     * 
     * @details Provides transition state value for QTUN dataflash log message,
     *          enabling post-flight analysis of transition timing and behavior.
     */
    uint8_t get_log_transition_state() const override { return static_cast<uint8_t>(transition_state); }

    /**
     * @brief Check if forward transition is currently active
     * 
     * @return true if in TRANSITION_AIRSPEED_WAIT or TRANSITION_TIMER state
     * @return false if TRANSITION_DONE or in back transition
     * 
     * @details Used to determine which transition update path to execute and
     *          how to blend control authority.
     */
    bool active_frwd() const override;

    /**
     * @brief Determine if VTOL view should be shown in ground station
     * 
     * @return true if vehicle is primarily under VTOL control (show VTOL attitude indicators)
     * @return false if vehicle is primarily under FW control (show FW attitude indicators)
     * 
     * @details Returns true during early transition phases when VTOL motors are
     *          providing significant control authority.
     */
    bool show_vtol_view() const override;

    /**
     * @brief Adjust fixed-wing pitch command during transition
     * 
     * @param[in,out] nav_pitch_cd Navigation pitch command in centidegrees, potentially blended
     * @param[in,out] nav_roll_cd Navigation roll command in centidegrees (not modified by SLT)
     * 
     * @details During transition, blends current FW pitch command with last known
     *          FW pitch to prevent abrupt pitch changes when switching between
     *          VTOL and FW control authority. Provides smooth pitch transitions.
     * 
     * @note Only modifies pitch - roll command passed through unchanged
     */
    void set_FW_roll_pitch(int32_t& nav_pitch_cd, int32_t& nav_roll_cd) override;

    /**
     * @brief Set roll limit during transition phases
     * 
     * @param[in,out] roll_limit_cd Maximum roll angle in centidegrees, potentially restricted
     * 
     * @return true if roll limit was modified
     * @return false if default roll limit should be used
     * 
     * @details May restrict roll authority during critical transition phases to
     *          maintain stability and prevent excessive bank angles when control
     *          authority is split between VTOL and FW systems.
     */
    bool set_FW_roll_limit(int32_t& roll_limit_cd) override;

    /**
     * @brief Check if throttle mix updates are allowed during transition
     * 
     * @return true if throttle mix can be adjusted
     * @return false if mix must be held constant
     * 
     * @details During airspeed wait phase, throttle mix must be held to maintain
     *          consistent thrust while accelerating. Returns false during
     *          TRANSITION_AIRSPEED_WAIT to prevent mix changes.
     */
    bool allow_update_throttle_mix() const override;

    /**
     * @brief Get current MAVLink VTOL state for telemetry
     * 
     * @return MAV_VTOL_STATE Current state: TRANSITION_TO_FW, TRANSITION_TO_MC, or FW
     * 
     * @details Maps internal transition_state to MAVLink standard VTOL state
     *          enumeration for ground control station display.
     */
    MAV_VTOL_STATE get_mav_vtol_state() const override;

    /**
     * @brief Set VTOL roll and pitch limits during transition
     * 
     * @param[in,out] nav_roll_cd Navigation roll in centidegrees, potentially limited
     * @param[in,out] nav_pitch_cd Navigation pitch in centidegrees, potentially limited
     * 
     * @return true if limits were applied
     * @return false if default limits should be used
     * 
     * @details May restrict attitude angles during back transition to maintain
     *          stability while transitioning from FW to VTOL control.
     */
    bool set_VTOL_roll_pitch_limit(int32_t& nav_roll_cd, int32_t& nav_pitch_cd) override;

    /**
     * @brief Store current fixed-wing pitch for transition reference
     * 
     * @details Captures current FW mode pitch attitude and timestamp when called.
     *          This reference is used during subsequent transitions to blend pitch
     *          commands smoothly between VTOL and FW control modes.
     * 
     * @note Stores both pitch angle (last_fw_nav_pitch_cd) and time (last_fw_mode_ms)
     */
    void set_last_fw_pitch(void) override;

protected:

    /**
     * @brief Transition state enumeration for forward transition sequencing
     * 
     * @details Defines the three-phase forward transition state machine:
     * 
     * - TRANSITION_AIRSPEED_WAIT: Initial phase where vehicle accelerates with both
     *   lift and forward motors active. Remains in this state until target airspeed
     *   (Q_TRANSITION_MS parameter) is achieved. Lift motors maintain altitude while
     *   forward motor provides acceleration.
     * 
     * - TRANSITION_TIMER: Second phase where lift motor thrust is progressively
     *   ramped down over the configured transition time (Q_TRANSITION_MS). Wing
     *   lift takes over altitude maintenance as airspeed provides sufficient lift.
     * 
     * - TRANSITION_DONE: Final state indicating transition complete and full
     *   fixed-wing flight mode active. Lift motors disabled, full FW control.
     * 
     * @note Back transitions (FW to VTOL) don't use explicit states - they
     *       progressively increase motor thrust based on deceleration rate
     */
    enum {
        TRANSITION_AIRSPEED_WAIT,  ///< Waiting for target airspeed before motor rampdown
        TRANSITION_TIMER,          ///< Ramping down lift motors over transition timer
        TRANSITION_DONE            ///< Transition complete, full FW mode
    } transition_state;

    /**
     * @brief Timestamp when transition timer phase started
     * 
     * @details Millisecond timestamp (from AP_HAL::millis()) marking the start of
     *          TRANSITION_TIMER phase. Used to calculate motor thrust ramp
     *          percentage: ramp = (now - start) / Q_TRANSITION_MS
     * 
     * @note Only valid during TRANSITION_TIMER state
     */
    uint32_t transition_start_ms;
    
    /**
     * @brief Timestamp when low airspeed condition last detected
     * 
     * @details Used to track duration of low airspeed conditions during transition.
     *          If airspeed remains below target for extended period, may trigger
     *          warnings or modified transition behavior.
     * 
     * @note Millisecond timestamp from AP_HAL::millis()
     */
    uint32_t transition_low_airspeed_ms;

    /**
     * @brief Last recorded throttle value during active transition
     * 
     * @details Stores throttle setting (0.0 to 1.0 range) during transition for
     *          reference in throttle blending calculations. Used to ensure smooth
     *          throttle changes when transitioning between control modes.
     */
    float last_throttle;

    /**
     * @brief Timestamp when last in fixed-wing control mode
     * 
     * @details Millisecond timestamp marking most recent FW mode operation. Used
     *          with last_fw_nav_pitch_cd to determine if stored FW pitch reference
     *          is still current for transition blending.
     */
    uint32_t last_fw_mode_ms;
    
    /**
     * @brief Last fixed-wing navigation pitch command
     * 
     * @details Pitch angle in centidegrees from most recent FW mode operation.
     *          Used during transition to blend between VTOL and FW pitch attitudes,
     *          preventing abrupt pitch changes during control authority handoff.
     * 
     * @note Units: centidegrees (100 centidegrees = 1 degree)
     */
    int32_t last_fw_nav_pitch_cd;

    /**
     * @brief Tiltrotor tilt angle when airspeed target reached
     * 
     * @details Records the tilt mechanism angle (in degrees) at the moment when
     *          target airspeed is achieved during TRANSITION_AIRSPEED_WAIT phase.
     *          Used to coordinate tilt and motor thrust rampdown for smooth
     *          tiltrotor transitions.
     * 
     * @note Only relevant for tiltrotor QuadPlane configurations
     * @note Units: degrees (0 = vertical, 90 = horizontal)
     */
    float airspeed_reached_tilt;

    /**
     * @brief Flag indicating transition was forced to complete
     * 
     * @details Set to true when force_transition_complete() is called, indicating
     *          transition was manually completed rather than progressing through
     *          normal state sequence. Used to modify post-transition behavior.
     * 
     * @warning Forced transitions bypass safety checks and may result in abrupt control changes
     */
    bool in_forced_transition;

};

