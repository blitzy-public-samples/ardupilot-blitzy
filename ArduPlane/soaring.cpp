/**
 * @file soaring.cpp
 * @brief ArduSoar thermal soaring and variometer support for ArduPlane
 * 
 * @details This file implements the integration between ArduPlane and the AP_Soaring
 *          library, providing autonomous soaring capabilities for fixed-wing aircraft.
 *          ArduSoar enables aircraft to detect and exploit thermal updrafts to gain
 *          altitude without motor power, significantly extending flight endurance.
 *          
 *          Key capabilities:
 *          - Thermal detection through variometer (climb rate) analysis
 *          - Automatic mode switching to THERMAL mode when updrafts detected
 *          - Throttle suppression during soaring flight
 *          - Integration with AUTO, CRUISE, FBWB and other autonomous modes
 *          - Vario simulation and energy state tracking for thermal detection
 *          
 *          The soaring system coordinates between multiple components:
 *          - AP_Soaring library (g2.soaring_controller) - Core soaring algorithms
 *          - Mode_Thermal - Specialized flight mode for circling in thermals
 *          - Mission system - Respects landing sequence restrictions
 *          - Throttle control - Manages powered vs unpowered flight
 *          
 *          Soaring operation flow:
 *          1. Check if soaring is active and not in landing sequence
 *          2. Update variometer with current climb rate data
 *          3. If in THERMAL mode, suppress throttle and update thermal tracking
 *          4. If in cruising mode with throttle suppressed, check for new thermals
 *          5. Switch to THERMAL mode when thermal criteria met
 *          
 * @note This module is only compiled when HAL_SOARING_ENABLED is defined
 * @note Soaring requires accurate airspeed, altitude and climb rate sensing
 * 
 * @author Peter Braswell, Samuel Tabor, Andrey Kolobov, and Iain Guilliard
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * @see libraries/AP_Soaring for core soaring algorithms and thermal detection
 * @see ArduPlane/mode_thermal.cpp for thermal-specific flight mode implementation
 */

#include "Plane.h"

#if HAL_SOARING_ENABLED

/**
 * @brief Main soaring update function called from the main loop
 * 
 * @details This function integrates the AP_Soaring library with ArduPlane's flight
 *          control system. It is called regularly from the main scheduler and manages
 *          the complete soaring state machine including thermal detection, mode
 *          transitions, and throttle suppression.
 *          
 *          The function implements a state-based control flow:
 *          
 *          State 1: Check soaring active state
 *          - Verifies soaring is enabled via parameters
 *          - Checks for landing sequence (soaring disabled during landing)
 *          - Updates throttle suppression state on activation changes
 *          - Returns early if soaring is not active
 *          
 *          State 2: Update variometer
 *          - Processes current altitude and climb rate
 *          - Updates moving average filters for thermal detection
 *          - Calculates energy state (kinetic + potential energy)
 *          - Prepares data for thermal detection algorithm
 *          
 *          State 3: THERMAL mode handling (if currently thermalling)
 *          - Ensures throttle remains suppressed during thermal soaring
 *          - Calls mode_thermal.update_soaring() to manage thermal tracking
 *          - Controls bank angle and turn rate to stay in updraft core
 *          - Monitors thermal strength and position
 *          
 *          State 4: Automatic thermal detection (if in compatible mode)
 *          - Only active in modes supporting automatic thermal switching
 *          - When throttle suppression active, monitors for thermal signatures
 *          - Updates cruising soaring parameters (McCready speed, etc.)
 *          - Switches to THERMAL mode when thermal criteria satisfied
 *          
 *          State 5: Non-soaring modes
 *          - Ensures throttle is not suppressed if mode doesn't support soaring
 *          - Allows normal powered flight operation
 *          
 * @note Called at main loop rate (typically 50-400 Hz depending on scheduler)
 * @note Thermal detection requires several seconds of climb rate data accumulation
 * @note Landing sequence always takes priority and disables soaring
 * 
 * @warning Incorrect variometer calibration can cause false thermal detections
 * @warning Soaring in turbulent conditions may trigger false thermal detection
 * @warning Always configure appropriate minimum altitude limits for safety
 * 
 * @see AP_Soaring::update_active_state() - Manages soaring enable/disable state
 * @see AP_Soaring::update_vario() - Updates variometer with sensor data
 * @see AP_Soaring::check_thermal_criteria() - Thermal detection algorithm
 * @see Mode_Thermal::update_soaring() - Thermal mode soaring control logic
 */
void Plane::update_soaring() {
    
    // Check if soaring is active. Also sets throttle suppressed
    // status on active state changes.
    // Soaring is always disabled during landing sequence for safety - we need powered
    // flight control for predictable landing approach and flare.
    bool override_disable = mission.get_in_landing_sequence_flag();

    // Update soaring active state based on parameters and landing sequence status.
    // This also manages throttle suppression state transitions when soaring is
    // enabled or disabled.
    plane.g2.soaring_controller.update_active_state(override_disable);

    // Early return if soaring is not active - no further processing needed.
    // This allows the rest of the function to assume soaring is enabled.
    if (!g2.soaring_controller.is_active()) {
        return;
    }
    
    // Update variometer with current flight state data (altitude, airspeed, climb rate).
    // The vario tracks total energy (kinetic + potential) changes to detect thermals
    // even during speed changes. This filtering is critical for reliable thermal detection.
    g2.soaring_controller.update_vario();

    // Branch 1: Already in THERMAL mode - continue thermalling
    if (control_mode == &mode_thermal) {
        // We are currently thermalling; suppress throttle and update
        // the thermal mode.

        // Never use throttle in THERMAL with soaring active.
        // Thermalling relies on circling in updrafts without motor power to maintain
        // efficient climb rate and avoid disturbing the thermal detection algorithm.
        g2.soaring_controller.set_throttle_suppressed(true);

        // Update THERMAL mode soaring logic.
        // This manages the circular flight path to stay centered in the thermal updraft,
        // monitors thermal strength, and determines when to exit the thermal.
        mode_thermal.update_soaring();
        return;
    }

    // Branch 2: In a mode that supports automatic thermal detection and switching
    // Modes like AUTO, CRUISE, FBWB can automatically switch to THERMAL when updrafts detected.
    if (control_mode->does_automatic_thermal_switch()) {
        // We might decide to start thermalling; if we're not under
        // powered flight then check for thermals and potentially
        // switch modes.

        // Check for throttle suppression - this indicates we're in unpowered soaring mode.
        // Throttle suppression is controlled by soaring parameters and altitude limits.
        if (g2.soaring_controller.suppress_throttle()) {
            // Throttle is suppressed, perform cruising modes update and check for mode switch.

            // Cruising modes update.
            // Updates optimal cruise speed based on MacCready theory, considering expected
            // thermal strength ahead and current altitude margin above minimum altitude.
            g2.soaring_controller.update_cruising();

            // Test for switch into THERMAL mode.
            // Thermal detection uses variometer data, checking for sustained climb rate
            // above threshold. This filtering avoids false triggers from turbulence.
            if (g2.soaring_controller.check_thermal_criteria()) {
                // Thermal detected - switch to THERMAL mode to exploit the updraft.
                // GCS notification helps pilots monitor autonomous soaring decisions.
                gcs().send_text(MAV_SEVERITY_INFO, "Soaring: Thermal detected, entering %s", mode_thermal.name());
                set_mode(mode_thermal, ModeReason::SOARING_THERMAL_DETECTED);
            }
        }
        return;
    }

    // Branch 3: In a mode that doesn't support automatic thermal switching
    // Modes like MANUAL, STABILIZE, ACRO, etc. don't support autonomous soaring.
    // We are not thermalling and won't start to from this mode. Allow throttle.
    // This ensures normal powered flight operation in non-autonomous modes.
    g2.soaring_controller.set_throttle_suppressed(false);
}

#endif // HAL_SOARING_ENABLED
