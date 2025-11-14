/**
 * @file AP_Arming_Sub.h
 * @brief ArduSub-specific arming checks and safety verification system
 * 
 * This file declares the AP_Arming_Sub class which extends the base AP_Arming
 * class with underwater vehicle-specific pre-arm and arming checks. ArduSub has
 * unique safety requirements compared to aerial vehicles, including different
 * sensor requirements, RC calibration needs, and failsafe considerations for
 * underwater operation.
 * 
 * The arming system is a critical safety feature that prevents vehicle operation
 * until all safety checks pass, protecting both the vehicle and surrounding environment.
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Arming/AP_Arming.h>

/**
 * @class AP_Arming_Sub
 * @brief ArduSub-specific implementation of arming checks for underwater vehicles
 * 
 * @details This class extends AP_Arming to provide ArduSub-specific pre-arm and
 *          arming verification. Underwater vehicles have different safety requirements
 *          than aerial vehicles:
 *          - No barometer altitude requirements for arming
 *          - Modified IMU health checks for underwater operation
 *          - Different RC calibration requirements (6 DOF control)
 *          - Joystick input validation for manual control
 *          - Depth sensor health verification
 *          - Buoyancy and trim considerations
 * 
 *          The arming system implements a two-stage safety verification:
 *          1. Pre-arm checks: Performed continuously, must pass before arming is allowed
 *          2. Arming checks: Final verification performed during arm() call
 * 
 *          Inherits from AP_Arming base class which provides common arming infrastructure
 *          including parameter storage, state management, and common check implementations.
 * 
 * @note Arming is a safety-critical system - all checks must pass before vehicle operation
 * @warning Modifying arming checks can affect vehicle safety and operator protection
 * 
 * @see AP_Arming for base class implementation
 * @see Sub::arm_checks() for vehicle-level arming coordination
 */
class AP_Arming_Sub : public AP_Arming {
public:

    /**
     * @brief Construct ArduSub arming check object
     * 
     * @details Initializes the Sub-specific arming system by calling the base
     *          AP_Arming constructor. Default constructor with no additional
     *          Sub-specific initialization required.
     */
    AP_Arming_Sub() : AP_Arming() { }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Sub);

    /**
     * @brief Verify RC (Radio Control) input calibration for underwater vehicle operation
     * 
     * @details Checks that all required RC channels are properly calibrated for ArduSub's
     *          6 degrees of freedom control (forward/back, left/right, up/down, yaw, pitch, roll).
     *          Underwater vehicles typically use joystick control requiring specific channel
     *          mapping and calibration validation.
     * 
     *          Validates:
     *          - Minimum channel count for 6DOF control
     *          - RC input ranges are within valid PWM bounds
     *          - Critical channels (throttle, yaw, pitch, roll) are properly configured
     *          - Joystick trim and centering
     * 
     *          This is a pre-arm check that must pass before arming is allowed.
     * 
     * @param[in] display_failure If true, send failure messages to GCS for pilot notification
     * 
     * @return true if all RC calibration checks pass, false if any check fails
     * 
     * @note Overrides base class implementation with Sub-specific RC requirements
     * @warning Improper RC calibration can result in unexpected vehicle motion when armed
     * 
     * @see AP_Arming::rc_calibration_checks() for base implementation
     */
    bool rc_calibration_checks(bool display_failure) override;
    
    /**
     * @brief Perform comprehensive pre-arm safety checks for underwater vehicle
     * 
     * @details Executes the complete suite of ArduSub pre-arm checks to verify vehicle
     *          is safe for arming and operation. Pre-arm checks are performed continuously
     *          and must all pass before the arm() function will allow arming.
     * 
     *          ArduSub pre-arm checks include:
     *          - RC input calibration (via rc_calibration_checks())
     *          - IMU health and calibration (via ins_checks())
     *          - Compass calibration and health
     *          - Battery voltage and capacity checks
     *          - Sensor health (pressure sensors, depth sensor)
     *          - Parameter validity checks
     *          - Mode-specific requirements
     *          - Leak detector status (if equipped)
     *          - Internal pressure/temperature sensor status
     * 
     *          Unlike aerial vehicles, ArduSub does not require:
     *          - GPS lock or position estimate
     *          - Barometer altitude validation
     *          - Airspeed sensor checks
     * 
     *          This function is called repeatedly while disarmed to provide real-time
     *          arming readiness status to the pilot via GCS messages.
     * 
     * @param[in] display_failure If true, send detailed failure messages to GCS to help
     *                            pilot identify and resolve pre-arm check failures
     * 
     * @return true if all pre-arm checks pass and vehicle is ready for arming,
     *         false if any check fails
     * 
     * @note Called continuously while disarmed; must pass before arming is allowed
     * @warning All pre-arm checks must pass to ensure safe vehicle operation
     * 
     * @see arm() for final arming verification
     * @see ins_checks() for IMU-specific checks
     * @see AP_Arming::pre_arm_checks() for base implementation
     */
    bool pre_arm_checks(bool display_failure) override;
    
    /**
     * @brief Check if RC transmitter has a dedicated disarm function assigned
     * 
     * @details Determines whether the pilot has configured an RC channel/switch
     *          function for explicit disarm control. This allows the pilot to
     *          disarm the vehicle using a dedicated switch or button on the
     *          transmitter rather than relying solely on:
     *          - Rudder stick disarm sequence (yaw left + throttle down)
     *          - GCS disarm command
     *          - Auto-disarm timeout
     * 
     *          A dedicated disarm function is useful in underwater operations where:
     *          - Quick emergency disarm may be needed
     *          - Stick sequences are difficult with joystick control
     *          - Explicit control state management is desired
     * 
     * @return true if an RC channel is configured with disarm function,
     *         false if no dedicated disarm function is assigned
     * 
     * @note This is a query function and does not modify any state
     * @see disarm() for actual disarm operation
     */
    bool has_disarm_function() const;

    /**
     * @brief Disarm the underwater vehicle and stop all motor outputs
     * 
     * @details Disarms the ArduSub vehicle by:
     *          1. Performing disarm safety checks (if enabled)
     *          2. Setting vehicle state to disarmed
     *          3. Shutting down all motor/thruster outputs
     *          4. Logging disarm event with method and reason
     *          5. Sending disarm notification to GCS
     * 
     *          Disarm can be initiated by:
     *          - Pilot RC input (stick sequence or dedicated switch)
     *          - GCS command via MAVLink
     *          - Auto-disarm after timeout on surface
     *          - Emergency disarm from failsafe
     *          - Land complete detection
     * 
     *          ArduSub-specific disarm considerations:
     *          - Immediately stops all thrusters to prevent uncontrolled motion
     *          - Can be disarmed underwater (unlike aircraft which must be landed)
     *          - May bypass checks for emergency disarm situations
     *          - Auto-disarm typically disabled for ROV applications
     * 
     * @param[in] method The disarm method/source (RC, GCS, auto, etc.) for logging
     *                   and method-specific handling
     * @param[in] do_disarm_checks If true, perform safety checks before disarming;
     *                             if false, force disarm without checks (emergency)
     * 
     * @return true if disarm successful, false if disarm checks failed (when enabled)
     * 
     * @note Disarming stops all motor outputs immediately
     * @warning Disarming underwater will cause vehicle to drift with currents
     * @warning Setting do_disarm_checks=false bypasses safety checks - use only for emergencies
     * 
     * @see arm() for arming operation
     * @see AP_Arming::Method for available disarm methods
     * @see AP_Arming::disarm() for base implementation
     */
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;
    
    /**
     * @brief Arm the underwater vehicle and enable motor outputs
     * 
     * @details Arms the ArduSub vehicle by:
     *          1. Performing pre-arm safety checks (via pre_arm_checks())
     *          2. Executing arming-time verification checks
     *          3. Running any mode-specific arming requirements
     *          4. Setting vehicle state to armed
     *          5. Enabling motor/thruster outputs
     *          6. Logging arm event with method
     *          7. Sending armed notification to GCS
     * 
     *          Arming can be initiated by:
     *          - Pilot RC input (stick sequence or dedicated switch)
     *          - GCS command via MAVLink
     *          - Auto-arm in some flight modes (typically disabled)
     * 
     *          ArduSub arming safety checks include:
     *          - All pre-arm checks must pass
     *          - RC input present and calibrated
     *          - IMU healthy and calibrated
     *          - Compass calibrated (if used)
     *          - Battery voltage sufficient
     *          - No active leak detector warnings
     *          - Mode allows arming
     *          - No critical failures present
     * 
     *          Safety interlock: Vehicle will not arm if any critical check fails,
     *          protecting pilot and vehicle from unsafe operation.
     * 
     * @param[in] method The arm method/source (RC, GCS, auto) for logging and
     *                   method-specific handling
     * @param[in] do_arming_checks If true, perform all safety checks before arming;
     *                             if false, force arm without checks (use with extreme caution)
     * 
     * @return true if arming successful, false if any arming check failed
     * 
     * @note Vehicle must be armed before thrusters will respond to pilot input
     * @warning Arming enables motor outputs - ensure vehicle is secured and clear of obstacles
     * @warning Bypassing arming checks (do_arming_checks=false) removes critical safety protections
     * @warning Never arm vehicle with active leak detector warning
     * 
     * @see pre_arm_checks() for pre-arm verification
     * @see disarm() for disarming operation
     * @see AP_Arming::Method for available arm methods
     * @see AP_Arming::arm() for base implementation
     */
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:
    /**
     * @brief Verify Inertial Navigation System (IMU) health and calibration
     * 
     * @details Performs ArduSub-specific IMU health and calibration checks as part of
     *          pre-arm verification. The inertial sensors (accelerometers and gyroscopes)
     *          are critical for attitude estimation, stability control, and safe operation.
     * 
     *          ArduSub INS checks verify:
     *          - IMU is detected and responding
     *          - Gyroscope calibration is valid and current
     *          - Accelerometer calibration is valid (offsets and scaling)
     *          - IMU temperature within operating range
     *          - No excessive vibration detected
     *          - IMU health flags indicate normal operation
     *          - Multi-IMU consistency (if multiple IMUs present)
     *          - No IMU sensor errors or failures
     * 
     *          ArduSub-specific considerations:
     *          - Underwater vehicles experience different vibration profiles than aircraft
     *          - Temperature stability is typically better underwater (thermal mass of water)
     *          - IMU must maintain accuracy during vehicle rotation and translation
     *          - Calibration validity critical for attitude hold and depth hold modes
     * 
     *          This function is called as part of the pre_arm_checks() sequence and must
     *          pass before arming is allowed.
     * 
     * @param[in] display_failure If true, send detailed IMU failure messages to GCS
     *                            to help pilot identify and resolve sensor issues
     * 
     * @return true if all IMU checks pass, false if any check fails
     * 
     * @note Protected method called by pre_arm_checks()
     * @warning IMU calibration must be valid for safe vehicle operation
     * @warning Excessive vibration can cause attitude estimation errors and instability
     * 
     * @see pre_arm_checks() for overall pre-arm check sequence
     * @see AP_InertialSensor for IMU implementation
     * @see AP_Arming::ins_checks() for base implementation
     */
    bool ins_checks(bool display_failure) override;
};
