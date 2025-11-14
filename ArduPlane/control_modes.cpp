/**
 * @file control_modes.cpp
 * @brief Flight mode management and switching logic for ArduPlane
 * 
 * @details This file implements the flight mode control system for fixed-wing aircraft.
 *          It handles:
 *          - Mode number to mode object pointer conversion
 *          - RC channel mode switch reading with failsafe protection
 *          - Mode change detection and execution
 *          - Autotune system initialization and control
 *          - Inverted flight state management
 *          
 *          ArduPlane supports multiple flight modes including manual control modes
 *          (MANUAL, STABILIZE, ACRO), assisted modes (FLY_BY_WIRE_A/B, CRUISE),
 *          autonomous modes (AUTO, RTL, LOITER, GUIDED), and quadplane-specific
 *          modes (QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO).
 *          
 *          Mode Selection Process:
 *          1. RC transmitter sends PWM values on the mode switch channel
 *          2. PWM values are mapped to mode switch positions (0-5 typically)
 *          3. Each position corresponds to a flight mode number from flight_modes[] array
 *          4. mode_from_mode_num() converts the mode number to a mode object pointer
 *          5. The mode change is executed through the Mode interface
 *          
 *          Safety Considerations:
 *          - Mode switches only processed when RC signals are recent (<100ms)
 *          - Invalid mode numbers are rejected to prevent undefined behavior
 *          - Failsafe conditions may override manual mode selection
 *          
 * @note This file contains safety-critical flight mode control logic.
 *       Mode switching must be reliable and predictable for safe flight.
 * 
 * @see Mode.h for mode interface definitions
 * @see Plane.h for vehicle state and mode objects
 * @see RC_Channel.h for RC input handling
 */

#include "Plane.h"

#include "quadplane.h"
#include "qautotune.h"

/**
 * @brief Convert a mode number to a mode object pointer
 * 
 * @details This function maps flight mode enumeration values to their corresponding
 *          mode object instances. Each mode number corresponds to a specific flight
 *          control mode implementation (e.g., MANUAL, AUTO, STABILIZE).
 *          
 *          The function handles both standard fixed-wing modes and quadplane-specific
 *          modes. Some modes are conditionally compiled based on hardware capabilities:
 *          - AVOID_ADSB requires HAL_ADSB_ENABLED
 *          - Quadplane modes require HAL_QUADPLANE_ENABLED
 *          - QAUTOTUNE requires QAUTOTUNE_ENABLED
 *          - AUTOLAND requires MODE_AUTOLAND_ENABLED
 *          - THERMAL requires HAL_SOARING_ENABLED
 *          
 *          Mode Selection Logic:
 *          The mode number comes from the flight_modes[] parameter array, which is
 *          configured by the pilot. RC PWM values are mapped to array indices:
 *          - PWM < 1230: position 0 (flight_modes[0])
 *          - PWM 1230-1360: position 1 (flight_modes[1])
 *          - PWM 1361-1490: position 2 (flight_modes[2])
 *          - PWM 1491-1620: position 3 (flight_modes[3])
 *          - PWM 1621-1749: position 4 (flight_modes[4])
 *          - PWM >= 1750: position 5 (flight_modes[5])
 * 
 * @param[in] num Flight mode number from Mode::Number enumeration
 * 
 * @return Mode* Pointer to the mode object corresponding to the mode number,
 *              or nullptr if the mode number is invalid or the mode is not
 *              compiled into the firmware
 * 
 * @note Returns nullptr for unsupported or disabled modes - calling code must
 *       check for null pointer before using the returned mode object
 * 
 * @warning This is called during mode changes which are safety-critical operations.
 *          Invalid mode pointers could lead to undefined behavior or crashes.
 * 
 * @see Mode::Number enumeration for all available mode numbers
 * @see set_mode() for mode change execution
 */
Mode *Plane::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::CIRCLE:
        ret = &mode_circle;
        break;
    case Mode::Number::STABILIZE:
        ret = &mode_stabilize;
        break;
    case Mode::Number::TRAINING:
        ret = &mode_training;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::FLY_BY_WIRE_A:
        ret = &mode_fbwa;
        break;
    case Mode::Number::FLY_BY_WIRE_B:
        ret = &mode_fbwb;
        break;
    case Mode::Number::CRUISE:
        ret = &mode_cruise;
        break;
    case Mode::Number::AUTOTUNE:
        ret = &mode_autotune;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::AVOID_ADSB:
#if HAL_ADSB_ENABLED
        ret = &mode_avoidADSB;
        break;
#endif
    // if ADSB is not compiled in then fallthrough to guided
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initializing;
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QSTABILIZE:
        ret = &mode_qstabilize;
        break;
    case Mode::Number::QHOVER:
        ret = &mode_qhover;
        break;
    case Mode::Number::QLOITER:
        ret = &mode_qloiter;
        break;
    case Mode::Number::QLAND:
        ret = &mode_qland;
        break;
    case Mode::Number::QRTL:
        ret = &mode_qrtl;
        break;
    case Mode::Number::QACRO:
        ret = &mode_qacro;
        break;
#if QAUTOTUNE_ENABLED
    case Mode::Number::QAUTOTUNE:
        ret = &mode_qautotune;
        break;
#endif
#endif  // HAL_QUADPLANE_ENABLED
    case Mode::Number::TAKEOFF:
        ret = &mode_takeoff;
        break;
#if MODE_AUTOLAND_ENABLED
    case Mode::Number::AUTOLAND:
        ret = &mode_autoland;
        break;
#endif //MODE_AUTOLAND_ENABLED
    case Mode::Number::THERMAL:
#if HAL_SOARING_ENABLED
        ret = &mode_thermal;
#endif
        break;
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::LOITER_ALT_QLAND:
        ret = &mode_loiter_qland;
        break;
#endif  // HAL_QUADPLANE_ENABLED

    }
    return ret;
}

/**
 * @brief Read flight mode selection from RC transmitter mode switch channel
 * 
 * @details This method reads the current position of the mode switch on the RC transmitter
 *          and triggers mode changes when the switch position changes. It implements a
 *          critical safety check to prevent mode changes based on stale RC data.
 *          
 *          Safety Mechanism:
 *          The function only processes mode switch inputs when RC signals are recent
 *          (received within the last 100ms). This prevents inadvertent mode changes
 *          during RC signal loss or interference. If the RC signal is stale, the
 *          function returns immediately without reading the mode switch, preserving
 *          the current flight mode until valid RC signals resume.
 *          
 *          Execution Flow:
 *          1. Check time since last valid RC signal reception
 *          2. If > 100ms elapsed, abort mode switch reading (RC failsafe condition)
 *          3. If RC signals are recent, call parent class read_mode_switch()
 *          4. Parent class reads PWM value from mode switch channel
 *          5. PWM value is converted to switch position (0-5)
 *          6. If position changed, mode_switch_changed() is called
 *          
 *          Timing Requirements:
 *          This function is called at the main loop rate (typically 50-400Hz depending
 *          on vehicle configuration). The 100ms threshold allows for 5-40 missed packets
 *          before rejecting mode switch inputs.
 * 
 * @note This method overrides RC_Channels::read_mode_switch() to add plane-specific
 *       failsafe protection. The 100ms threshold is a safety margin to ensure mode
 *       changes only occur with current RC data.
 * 
 * @warning Removing or loosening the RC signal age check could allow mode changes
 *          based on stale or corrupted data, potentially causing unsafe flight behavior.
 *          The 100ms threshold should not be increased without careful safety analysis.
 * 
 * @see RC_Channel_Plane::mode_switch_changed() for mode change execution
 * @see failsafe.last_valid_rc_ms for RC signal freshness tracking
 * @see RC_Channels::read_mode_switch() for base implementation
 */
void RC_Channels_Plane::read_mode_switch()
{
    if (millis() - plane.failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }
    RC_Channels::read_mode_switch();
}

/**
 * @brief Handle flight mode switch position change from RC transmitter
 * 
 * @details This callback function is invoked when the RC mode switch changes position.
 *          It validates the new switch position and executes the corresponding flight
 *          mode change by looking up the configured mode in the flight_modes[] array.
 *          
 *          Mode Selection Process:
 *          The RC transmitter mode switch PWM value is converted to a discrete switch
 *          position (typically 0-5 for a 6-position switch). Each position corresponds
 *          to an index in the flight_modes[] parameter array, which is user-configurable.
 *          
 *          For example, with default configuration:
 *          - Position 0 (PWM < 1230): flight_modes[0] (commonly MANUAL)
 *          - Position 1 (PWM 1230-1360): flight_modes[1] (commonly CIRCLE)
 *          - Position 2 (PWM 1361-1490): flight_modes[2] (commonly STABILIZE)
 *          - Position 3 (PWM 1491-1620): flight_modes[3] (commonly FBWA)
 *          - Position 4 (PWM 1621-1749): flight_modes[4] (commonly FBWB)
 *          - Position 5 (PWM >= 1750): flight_modes[5] (commonly AUTO or RTL)
 *          
 *          Validation:
 *          The function validates that new_pos is within valid range (0 to num_flight_modes).
 *          Invalid positions are rejected silently to prevent array out-of-bounds access.
 *          This should not occur under normal operation but provides defensive programming
 *          against corrupted RC data or software errors.
 *          
 *          Mode Change Execution:
 *          After validation, the function retrieves the mode number from flight_modes[new_pos]
 *          and requests a mode change via set_mode_by_number() with reason code RC_COMMAND.
 *          The actual mode change goes through the mode change state machine which performs
 *          additional checks (arming state, mode availability, etc.).
 * 
 * @param[in] new_pos New mode switch position from RC receiver (0-5 typically)
 *                    Position is determined by PWM-to-position mapping in RC_Channel
 * 
 * @note This function is called from the RC input processing code when a mode switch
 *       position change is detected. It executes at main loop rate but only when the
 *       switch actually changes position.
 * 
 * @warning Invalid switch positions are silently ignored. Ensure num_flight_modes is
 *          correctly configured to match the number of switch positions being used.
 *          Position validation prevents crashes from array bounds violations.
 * 
 * @see set_mode_by_number() for mode change execution and validation
 * @see flight_modes[] parameter array for mode number configuration
 * @see ModeReason::RC_COMMAND for mode change reason tracking
 */
void RC_Channel_Plane::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > plane.num_flight_modes) {
        // should not have been called
        return;
    }

    plane.set_mode_by_number((Mode::Number)plane.flight_modes[new_pos].get(), ModeReason::RC_COMMAND);
}

/**
 * @brief Initialize and start the autotune process for configured control axes
 * 
 * @details This function is called when entering AUTOTUNE flight mode or when autotune
 *          is enabled via auxiliary switch in AUTO mode. It initializes the autotune
 *          process for the control axes specified in the AUTOTUNE_AXES parameter bitmask.
 *          
 *          Autotune Process:
 *          Autotune automatically determines optimal PID gains for the aircraft's
 *          roll, pitch, and/or yaw controllers by performing controlled oscillations
 *          and measuring the aircraft's response. The process:
 *          1. Checks which axes are enabled for tuning (g2.axis_bitmask)
 *          2. Initializes autotune for each enabled controller
 *          3. Sets the global autotuning flag
 *          4. Reports tuning status to ground control station
 *          
 *          Axis Selection:
 *          - Roll: Tunes roll rate and stabilization gains
 *          - Pitch: Tunes pitch rate and stabilization gains  
 *          - Yaw: Tunes yaw rate and stabilization gains
 *          
 *          Multiple axes can be tuned simultaneously or individually based on the
 *          AUTOTUNE_AXES bitmask parameter configuration.
 * 
 * @note Autotune requires the aircraft to be airborne and in stable flight conditions.
 *       The pilot should maintain altitude and heading during the autotune process.
 *       Autotune may take several minutes to complete depending on axes selected.
 * 
 * @warning Autotune performs aggressive maneuvers to characterize aircraft response.
 *          Ensure sufficient altitude (recommend >50m AGL) and clear airspace before
 *          starting autotune. Pilot must be ready to abort by switching flight modes
 *          if aircraft behavior becomes unsafe.
 * 
 * @see autotune_restore() for stopping and restoring original gains
 * @see autotune_enable() for auxiliary switch control
 * @see g2.axis_bitmask for axis selection configuration
 */
void Plane::autotune_start(void)
{
    const bool tune_roll = g2.axis_bitmask.get() & int8_t(AutoTuneAxis::ROLL);
    const bool tune_pitch = g2.axis_bitmask.get() & int8_t(AutoTuneAxis::PITCH);
    const bool tune_yaw = g2.axis_bitmask.get() & int8_t(AutoTuneAxis::YAW);
    if (tune_roll || tune_pitch || tune_yaw) {
        gcs().send_text(MAV_SEVERITY_INFO, "Started autotune");
        if (tune_roll) { 
            rollController.autotune_start();
        }
        if (tune_pitch) { 
            pitchController.autotune_start();
        }
        if (tune_yaw) { 
            yawController.autotune_start();
        }
        autotuning = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Autotuning %s%s%s", tune_roll?"roll ":"", tune_pitch?"pitch ":"", tune_yaw?"yaw":"");
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "No axis selected for tuning!");
    }        
}

/**
 * @brief Stop autotune and restore original controller gains
 * 
 * @details This function is called when exiting AUTOTUNE flight mode or when autotune
 *          is disabled via auxiliary switch. It stops the autotune process and restores
 *          the original PID controller gains that were active before autotune started.
 *          
 *          Restoration Process:
 *          The function calls autotune_restore() on each axis controller (roll, pitch, yaw)
 *          regardless of which axes were being tuned. Each controller restores its
 *          pre-autotune gain values from saved state.
 *          
 *          Gain Management:
 *          - If autotune completed successfully, the pilot can save the new gains via GCS
 *          - If autotune was aborted or failed, original gains are restored automatically
 *          - Restored gains take effect immediately for flight control
 *          - The global autotuning flag is cleared to disable autotune telemetry
 * 
 * @note This function always restores all three axes (roll, pitch, yaw) even if only
 *       some axes were selected for tuning. Controllers that weren't tuning simply
 *       restore their unchanged gains.
 * 
 * @warning After exiting autotune without saving, the newly calculated gains are lost.
 *          Pilots should save gains via GCS before exiting if autotune completed
 *          successfully and the new gains are desired.
 * 
 * @see autotune_start() for starting autotune process
 * @see autotune_enable() for auxiliary switch control
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
    yawController.autotune_restore();
    if (autotuning) {
        autotuning = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Stopped autotune");
    }
}

/**
 * @brief Enable or disable autotune via auxiliary switch in AUTO modes
 * 
 * @details This function provides auxiliary switch control of the autotune system
 *          when the aircraft is in AUTO or other autonomous flight modes. It allows
 *          pilots to enable/disable autotune without changing flight modes.
 *          
 *          Use Case:
 *          During autonomous missions (AUTO, GUIDED, etc.), the pilot can activate
 *          an RC auxiliary switch to enable autotune. This allows gain tuning during
 *          predetermined flight patterns or mission segments without manual mode control.
 *          
 *          Operation:
 *          - enable=true: Calls autotune_start() to begin tuning process
 *          - enable=false: Calls autotune_restore() to stop tuning and restore gains
 *          
 *          This provides the same functionality as entering/exiting AUTOTUNE mode
 *          but without requiring a flight mode change.
 * 
 * @param[in] enable true to start autotune, false to stop and restore original gains
 * 
 * @note Typically controlled by an RC auxiliary switch configured for autotune enable.
 *       The switch can be toggled during flight to start/stop the autotune process.
 * 
 * @warning Autotune should only be enabled when aircraft is in stable flight with
 *          sufficient altitude. Ensure mission flight pattern is suitable for the
 *          aggressive maneuvers autotune will perform.
 * 
 * @see autotune_start() for initialization and safety considerations
 * @see autotune_restore() for restoration process
 */
void Plane::autotune_enable(bool enable)
{
    if (enable) {
        autotune_start();
    } else {
        autotune_restore();
    }
}

/**
 * @brief Determine if aircraft should fly inverted (upside-down)
 * 
 * @details This function returns the current inverted flight state for the aircraft.
 *          Inverted flight (flying upside-down) is supported in ArduPlane for aerobatic
 *          maneuvers and specialized mission requirements. The function checks multiple
 *          sources to determine if inverted flight is currently active.
 *          
 *          Inverted Flight Sources:
 *          1. Manual mode: Never inverted (pilot has full control, no automated inversion)
 *          2. Auxiliary switch: inverted_flight flag controlled by RC switch
 *          3. AUTO mode commands: auto_state.inverted_flight set by mission commands
 *          
 *          Priority Order:
 *          - MANUAL mode always returns false (highest priority override)
 *          - Auxiliary switch inverted_flight flag (pilot control)
 *          - AUTO mode mission command inverted flight state
 *          
 *          Flight Control Implications:
 *          When flying inverted, the attitude controller inverts pitch and roll control
 *          responses to maintain proper flight control authority. Elevator and aileron
 *          deflections are reversed to account for the upside-down orientation.
 *          
 * @return true if aircraft should fly inverted (upside-down)
 * @return false if aircraft should fly normally (right-side up)
 * 
 * @note MANUAL mode always returns false to give pilot complete authority without
 *       automated inversions. In all other modes, inversion can be controlled via
 *       auxiliary switch or mission commands.
 * 
 * @warning Inverted flight requires properly configured aircraft, adequate altitude,
 *          and pilot readiness to recover. Not all airframes are capable of sustained
 *          inverted flight. Ensure aircraft has adequate power and control authority
 *          before enabling inverted flight features.
 * 
 * @see inverted_flight flag for auxiliary switch state
 * @see auto_state.inverted_flight for mission command state
 * @see Mode::MANUAL for manual control mode
 */
bool Plane::fly_inverted(void)
{
    if (control_mode == &plane.mode_manual) {
        return false;
    }
    if (inverted_flight) {
        // controlled with aux switch
        return true;
    }
    if (control_mode == &mode_auto && auto_state.inverted_flight) {
        return true;
    }
    return false;
}
