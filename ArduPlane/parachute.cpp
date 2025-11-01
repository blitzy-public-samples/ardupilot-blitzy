/**
 * @file parachute.cpp
 * @brief ArduPlane parachute deployment and monitoring support
 * 
 * @details This file implements the parachute deployment system for fixed-wing aircraft.
 *          The parachute system provides emergency descent capability in case of critical
 *          failures or loss of control. The system includes:
 *          - Continuous monitoring for automatic deployment conditions
 *          - Manual deployment with safety interlocks
 *          - Altitude-based safety checks to prevent ground deployment
 *          - Integration with landing gear deployment
 *          - Sink rate monitoring for automatic deployment
 * 
 *          The parachute can be triggered either automatically (based on configured conditions
 *          like excessive sink rate) or manually by pilot command. Safety interlocks prevent
 *          deployment when the aircraft is too close to the ground or already landed.
 * 
 * @note All parachute functionality is conditionally compiled based on HAL_PARACHUTE_ENABLED.
 *       If HAL_PARACHUTE_ENABLED is not defined, parachute support is disabled at compile time.
 * 
 * @warning This is safety-critical code. Parachute deployment is an emergency procedure that
 *          significantly affects vehicle behavior. Improper configuration or deployment can
 *          result in vehicle loss or damage.
 * 
 * @see AP_Parachute library for core parachute management functionality
 * @see AP_LandingGear for landing gear deployment coordination
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "Plane.h"


/**
 * @brief Monitor parachute system and check for automatic deployment conditions
 * 
 * @details This function is called periodically from the main scheduler to monitor
 *          the parachute system state and check if automatic deployment conditions
 *          have been met. It performs two main functions:
 *          1. Updates internal parachute library state
 *          2. Checks sink rate against configured thresholds for auto-deployment
 * 
 *          The parachute library handles the actual deployment logic including
 *          sink rate thresholds, deployment delays, and state management.
 * 
 * @note This function has no effect if HAL_PARACHUTE_ENABLED is not defined.
 * @note Called regularly at scheduler rate (typically 10-50 Hz depending on scheduler config).
 * 
 * @warning Automatic deployment can occur if configured conditions are met.
 *          Ensure parachute parameters are properly configured and tested in simulation.
 * 
 * @see AP_Parachute::update() for state machine updates
 * @see AP_Parachute::check_sink_rate() for automatic deployment trigger logic
 * 
 * Source: ArduPlane/parachute.cpp:7-13
 */
void Plane::parachute_check()
{
#if HAL_PARACHUTE_ENABLED
    parachute.update();
    parachute.check_sink_rate();
#endif
}

#if HAL_PARACHUTE_ENABLED

/**
 * @brief Trigger the release of the parachute deployment mechanism
 * 
 * @details This function initiates the parachute deployment sequence when called.
 *          It handles the following operations:
 *          1. Checks if release is already in progress (prevents duplicate commands)
 *          2. Sends appropriate status message to ground control station
 *          3. Commands the parachute library to physically release the parachute
 *          4. Deploys landing gear if available (AP_LANDINGGEAR_ENABLED)
 * 
 *          The function distinguishes between first-time deployment and repeated
 *          deployment attempts, sending different telemetry messages accordingly.
 * 
 *          Physical parachute release is handled by the AP_Parachute library, which
 *          controls the servo/relay mechanism configured for parachute deployment.
 * 
 * @note This function does not perform altitude or safety checks. It should be called
 *       from parachute_manual_release() for pilot-commanded deployment or from
 *       automatic deployment logic that has already validated conditions.
 * 
 * @note If landing gear is enabled (AP_LANDINGGEAR_ENABLED), the landing gear will be
 *       automatically deployed along with the parachute.
 * 
 * @warning This is a safety-critical function. Once called, the parachute will deploy
 *          and cannot be retracted. The vehicle will experience significant drag and
 *          descend rapidly with limited or no control authority.
 * 
 * @warning Do not call this function directly without altitude and flight state checks.
 *          Use parachute_manual_release() for pilot-commanded deployment which includes
 *          safety interlocks.
 * 
 * @see AP_Parachute::release() for physical deployment mechanism
 * @see AP_Parachute::release_in_progress() for deployment state checking
 * @see AP_Parachute::released() for checking if already deployed
 * @see parachute_manual_release() for safe manual deployment with checks
 * 
 * Source: ArduPlane/parachute.cpp:20-38
 */
void Plane::parachute_release()
{
    if (parachute.release_in_progress()) {
        return;
    }
    if (parachute.released()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released again");
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");
    }

    // release parachute
    parachute.release();

#if AP_LANDINGGEAR_ENABLED
    // deploy landing gear
    g2.landing_gear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
}

/**
 * @brief Trigger manual parachute release with safety interlocks and altitude checks
 * 
 * @details This function provides a safe interface for pilot-commanded parachute deployment.
 *          Unlike parachute_release(), this function implements multiple safety checks to
 *          prevent deployment in unsafe conditions:
 * 
 *          Safety Check Sequence:
 *          1. Verify parachute is enabled in parameters
 *          2. Verify parachute has not already been deployed
 *          3. Check altitude against minimum deployment altitude (CHUTE_ALT_MIN parameter)
 *          4. Verify aircraft has taken off (prevents accidental ground deployment)
 * 
 *          The altitude check is bypassed if the aircraft has never flown (auto_state.last_flying_ms == 0),
 *          allowing ground testing and bench testing without triggering altitude warnings.
 * 
 *          If all safety checks pass, the function calls parachute_release() to perform
 *          the actual deployment and also deploys landing gear if available.
 * 
 * @return true if parachute deployment was initiated successfully
 * @return false if deployment was prevented by safety checks or parachute not enabled
 * 
 * @note This is the recommended function for pilot-commanded deployment as it includes
 *       comprehensive safety interlocks.
 * 
 * @note Ground testing is supported: altitude checks are skipped if the aircraft has never
 *       detected flight (auto_state.last_flying_ms == 0).
 * 
 * @note Minimum altitude is configured via CHUTE_ALT_MIN parameter. If set to 0, altitude
 *       checking is disabled.
 * 
 * @warning Even with safety checks, manual deployment should only be used in emergency
 *          situations. The parachute cannot be retracted once deployed.
 * 
 * @warning The "Too low" warning (sent to GCS) indicates deployment was blocked because
 *          the aircraft is below minimum safe deployment altitude. This prevents parachute
 *          deployment too close to the ground where it may not have time to fully inflate.
 * 
 * @see parachute_release() for the actual deployment mechanism (without safety checks)
 * @see AP_Parachute::enabled() for checking if parachute is configured
 * @see AP_Parachute::released() for checking deployment state
 * @see AP_Parachute::alt_min() for minimum deployment altitude parameter
 * @see relative_ground_altitude() for altitude calculation
 * 
 * Source: ArduPlane/parachute.cpp:45-67
 */
bool Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return false;
    }

    if (parachute.alt_min() > 0 && relative_ground_altitude(RangeFinderUse::NONE) < parachute.alt_min() &&
            auto_state.last_flying_ms > 0) {
        // Allow manual ground tests by only checking if flying too low if we've taken off
        gcs().send_text(MAV_SEVERITY_WARNING, "Parachute: Too low");
        return false;
    }

    // if we get this far release parachute
    parachute_release();

#if AP_LANDINGGEAR_ENABLED
    // deploy landing gear
    g2.landing_gear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
    return true;    
}
#endif
