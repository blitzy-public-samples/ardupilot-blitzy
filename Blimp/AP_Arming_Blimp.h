/**
 * @file AP_Arming_Blimp.h
 * @brief Blimp-specific arming checks and safety validation
 * 
 * This file defines the AP_Arming_Blimp class which implements vehicle-specific
 * pre-arm checks, arming logic, and disarming procedures for lighter-than-air
 * blimp vehicles. It extends the base AP_Arming class with blimp-specific
 * safety validations including buoyancy compensation checks, fin control
 * validation, and lighter-than-air specific sensor requirements.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_Arming/AP_Arming.h>

/**
 * @class AP_Arming_Blimp
 * @brief Blimp-specific arming and pre-arm safety check implementation
 * 
 * @details This class extends AP_Arming to provide blimp-specific safety checks
 *          before allowing the vehicle to arm and take flight. Blimps have unique
 *          requirements compared to other vehicle types due to their lighter-than-air
 *          nature and different control mechanisms (fins rather than rotors/propellers).
 *          
 *          The class implements a comprehensive set of safety validations:
 *          - Pre-arm checks: Sensor health, calibration, and configuration validation
 *          - Arming checks: Final safety validation before motors can be armed
 *          - Mandatory checks: Critical safety checks that cannot be bypassed
 *          - Disarming logic: Safe vehicle disarming with appropriate safeguards
 *          
 *          Key blimp-specific considerations:
 *          - Buoyancy compensation verification
 *          - Fin control system health
 *          - Altitude/pressure sensor requirements for vertical control
 *          - GPS requirements for position hold in wind
 *          - Motor/fin servo operational checks
 *          
 *          Safety-critical nature: All methods in this class directly affect
 *          flight safety. Improper implementation or bypassing of checks can
 *          result in vehicle loss or injury.
 * 
 * @note This class sets ARMING_REQUIRE default to YES_MIN_PWM as blimps
 *       do not have a vehicle-specific ARMING_REQUIRE parameter
 * 
 * @warning This is safety-critical code. All arming checks exist to prevent
 *          unsafe flight conditions. Modifications must be thoroughly tested
 *          and reviewed for safety implications.
 * 
 * @see AP_Arming for base class implementation
 * @see Blimp::init_ardupilot() for arming system initialization
 */
class AP_Arming_Blimp : public AP_Arming
{
public:
    friend class Blimp;
    friend class ToyMode;

    /**
     * @brief Construct a new AP_Arming_Blimp object
     * 
     * @details Initializes the blimp arming system by calling the base AP_Arming
     *          constructor and setting blimp-specific default parameters.
     *          
     *          Sets ARMING_REQUIRE default to YES_MIN_PWM (value 1) because the
     *          Blimp vehicle type does not have a vehicle-specific ARMING_REQUIRE
     *          parameter. This ensures minimum PWM signal validation is performed
     *          before arming is allowed.
     * 
     * @note This constructor is called during Blimp vehicle initialization
     * @see AP_Arming::AP_Arming() for base class initialization
     */
    AP_Arming_Blimp() : AP_Arming()
    {
        // default REQUIRE parameter to 1 (Blimp does not have an
        // actual ARMING_REQUIRE parameter)
        require.set_default((uint8_t)Required::YES_MIN_PWM);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Arming_Blimp);

    /**
     * @brief Perform RC radio calibration validation checks
     * 
     * @details Verifies that the RC radio control inputs are properly calibrated
     *          and within acceptable ranges before arming. This is critical for
     *          blimps as fin control commands originate from RC inputs and improper
     *          calibration could result in unexpected fin deflections.
     *          
     *          Validates:
     *          - RC channel min/max/trim values are reasonable
     *          - Control ranges are sufficient for safe operation
     *          - No reversed channels that should not be reversed
     * 
     * @param[in] display_failure If true, display failure messages to GCS
     * 
     * @return true if RC calibration is acceptable, false otherwise
     * 
     * @note Called during pre-arm check sequence
     * @warning Improper RC calibration can cause control reversal or limited authority
     */
    bool rc_calibration_checks(bool display_failure) override;

    /**
     * @brief Disarm the vehicle motors/fins
     * 
     * @details Safely disarms the blimp by shutting down motor control and fin servos.
     *          Performs optional disarm checks to prevent accidental disarming in
     *          unsafe conditions (e.g., while airborne with significant altitude).
     *          
     *          Disarm sequence:
     *          1. Optional pre-disarm safety checks
     *          2. Motor/servo output shutdown
     *          3. State transition to disarmed
     *          4. Notification to GCS
     * 
     * @param[in] method Disarm method (switch, GCS command, radio failsafe, etc.)
     * @param[in] do_disarm_checks If true, perform safety checks before disarming
     * 
     * @return true if disarm successful, false if disarm prevented by safety checks
     * 
     * @note For blimps, disarming while significantly airborne may be unsafe due
     *       to loss of altitude control
     * @warning Disarming in flight can result in vehicle crash
     * 
     * @see Blimp::motors_output() for motor control interface
     */
    bool disarm(AP_Arming::Method method, bool do_disarm_checks=true) override;

    /**
     * @brief Arm the vehicle motors/fins for flight
     * 
     * @details Attempts to arm the blimp by enabling motor control and fin servos.
     *          Performs comprehensive arming checks to ensure the vehicle is safe
     *          to fly before allowing arming. Arming is only permitted if all
     *          mandatory and optional (based on ARMING_CHECK) safety checks pass.
     *          
     *          Arming sequence:
     *          1. Pre-arm checks (if do_arming_checks is true)
     *          2. Arming checks (final validation)
     *          3. Motor/servo output enable
     *          4. State transition to armed
     *          5. Notification to GCS
     * 
     * @param[in] method Arming method (switch, GCS command, auto-arming, etc.)
     * @param[in] do_arming_checks If true, perform full arming check sequence
     * 
     * @return true if arming successful, false if prevented by failed safety checks
     * 
     * @note Arming can be forced in some modes, but mandatory checks still apply
     * @warning Arming with failed safety checks can result in unsafe flight conditions
     * 
     * @see pre_arm_checks() for pre-arm validation
     * @see arm_checks() for final arming validation
     */
    bool arm(AP_Arming::Method method, bool do_arming_checks=true) override;

protected:

    /**
     * @brief Execute comprehensive pre-arm safety checks
     * 
     * @details Performs all pre-arm validation checks required before the vehicle
     *          can be armed. This is the main entry point for pre-arm validation
     *          and coordinates execution of all individual check functions.
     *          
     *          Check categories performed:
     *          - Sensor health and calibration (INS, compass, barometer, GPS)
     *          - EKF initialization and attitude estimation
     *          - RC radio calibration and signal quality
     *          - Parameter configuration validation
     *          - Motor/fin servo functionality
     *          - Battery voltage and capacity
     *          - GCS connection and failsafe status
     *          - Altitude/pressure sensor health
     *          
     *          Checks are performed based on ARMING_CHECK parameter bitmask.
     *          Some checks are mandatory and cannot be bypassed.
     * 
     * @param[in] display_failure If true, send failure messages to GCS for pilot awareness
     * 
     * @return true if all applicable pre-arm checks pass, false otherwise
     * 
     * @note Called automatically when arming is attempted
     * @warning Bypassing pre-arm checks (ARMING_CHECK=0) significantly increases
     *          risk of unsafe flight conditions
     * 
     * @see run_pre_arm_checks() for actual check implementation
     * @see mandatory_checks() for checks that cannot be bypassed
     */
    bool pre_arm_checks(bool display_failure) override;

    /**
     * @brief Verify EKF attitude estimation is valid and reliable
     * 
     * @details Checks that the Extended Kalman Filter (EKF) has properly initialized
     *          attitude estimation and is providing reliable orientation data. For
     *          blimps, accurate attitude is critical for fin control commands and
     *          maintaining stable flight orientation.
     *          
     *          Validates:
     *          - EKF has completed initialization
     *          - Attitude variance is within acceptable bounds
     *          - No GPS glitches or sensor faults affecting attitude
     *          - Gyro bias has converged
     * 
     * @return true if EKF attitude estimation is healthy, false otherwise
     * 
     * @note Called as part of pre-arm check sequence
     * @warning Flying with poor attitude estimation can result in loss of control
     * 
     * @see AP_AHRS::pre_arm_check() for base attitude checks
     * @see AP_NavEKF::healthy() for EKF health status
     */
    bool pre_arm_ekf_attitude_check();

    /**
     * @brief Perform final arming checks before enabling motors
     * 
     * @details Executes final safety validation immediately before arming is allowed.
     *          These checks verify that conditions are still safe after pre-arm checks
     *          completed, as some time may have elapsed between pre-arm and arming.
     *          
     *          Final validations:
     *          - Vehicle is in stable state (not excessive vibration)
     *          - Control surfaces/fins respond correctly
     *          - Safety switch (if present) is pressed
     *          - No GCS failsafe active
     *          - Arming method is appropriate for current mode
     * 
     * @param[in] method Method used to initiate arming (switch, GCS, auto, etc.)
     * 
     * @return true if final arming checks pass, false otherwise
     * 
     * @note This is the last safety gate before motors can be armed
     * @warning These checks should not be bypassed as they verify current safety state
     * 
     * @see AP_Arming::arm_checks() for base class arming validation
     */
    bool arm_checks(AP_Arming::Method method) override;

    /**
     * @brief Execute mandatory safety checks that cannot be bypassed
     * 
     * @details Performs critical safety checks that must pass even when ARMING_CHECK
     *          is set to 0 (disabled) or when arming is being forced. These represent
     *          the absolute minimum safety validation required to prevent catastrophic
     *          failures.
     *          
     *          Mandatory checks include:
     *          - Critical sensor availability (INS, barometer for altitude control)
     *          - GPS health for outdoor flight (mandatory GPS checks)
     *          - Battery voltage above minimum safe threshold
     *          - No critical system failures
     *          
     *          This function is only called when ARMING_CHECK=0 or forced arming,
     *          as these checks are included in normal pre-arm validation.
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if all mandatory checks pass, false otherwise
     * 
     * @note These checks protect against flight-critical failures
     * @warning Failure of mandatory checks indicates unsafe flight conditions that
     *          should never be ignored
     * 
     * @see mandatory_gps_checks() for GPS-specific mandatory validation
     */
    bool mandatory_checks(bool display_failure) override;

    /**
     * @brief Validate inertial sensor (IMU) health and calibration
     * 
     * @details Checks that accelerometers and gyroscopes are healthy, properly
     *          calibrated, and providing consistent measurements. For blimps,
     *          IMU data is critical for attitude control and detecting vehicle
     *          motion for buoyancy compensation.
     *          
     *          Validates:
     *          - All configured IMUs are detecting motion
     *          - Gyro and accelerometer calibration is current
     *          - No excessive IMU inconsistency between redundant sensors
     *          - Vibration levels are acceptable
     *          - Temperature calibration applied if available
     *          
     *          Calls into base AP_Arming::ins_checks() after blimp-specific validation.
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if inertial sensors are healthy, false otherwise
     * 
     * @note This check calls base class AP_Arming::ins_checks()
     * @warning Poor IMU health can cause erratic attitude control
     * 
     * @see AP_InertialSensor::pre_arm_check() for detailed IMU validation
     */
    bool ins_checks(bool display_failure) override;

    /**
     * @brief Validate GPS health, fix quality, and configuration
     * 
     * @details Checks that GPS receiver is functioning, has acquired adequate
     *          satellite fix, and can provide position data suitable for navigation.
     *          For blimps operating outdoors, GPS is essential for position hold
     *          against wind drift.
     *          
     *          Validates:
     *          - GPS has valid 3D fix
     *          - Horizontal dilution of precision (HDOP) is acceptable
     *          - Sufficient satellites in view
     *          - GPS is configured correctly (baud rate, protocol)
     *          - No GPS glitches or jamming detected
     *          
     *          Calls into base AP_Arming::gps_checks() after blimp-specific validation.
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if GPS is healthy and providing adequate fix, false otherwise
     * 
     * @note This check calls base class AP_Arming::gps_checks()
     * @warning Flying without GPS fix can result in uncontrolled drift
     * 
     * @see AP_GPS::pre_arm_checks() for detailed GPS validation
     * @see mandatory_gps_checks() for GPS checks that cannot be bypassed
     */
    bool gps_checks(bool display_failure) override;

    /**
     * @brief Validate barometer health and calibration
     * 
     * @details Checks that barometric pressure sensors are healthy and providing
     *          consistent altitude measurements. For blimps, barometer is critical
     *          for altitude hold and vertical velocity control, as lighter-than-air
     *          vehicles are highly sensitive to pressure changes.
     *          
     *          Validates:
     *          - All configured barometers are healthy
     *          - Barometer readings are consistent between redundant sensors
     *          - No excessive altitude drift or noise
     *          - Ground pressure reference is reasonable
     *          
     *          Calls into base AP_Arming::barometer_checks() after blimp-specific validation.
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if barometers are healthy, false otherwise
     * 
     * @note This check calls base class AP_Arming::barometer_checks()
     * @warning Poor barometer health can cause altitude control failures
     * 
     * @see AP_Baro::pre_arm_checks() for detailed barometer validation
     */
    bool barometer_checks(bool display_failure) override;

    /**
     * @brief Validate board power supply voltage levels
     * 
     * @details Checks that the flight controller board is receiving adequate power
     *          and voltage levels are within safe operating ranges. Low voltage can
     *          cause brownouts, processor resets, or servo glitches.
     *          
     *          Validates:
     *          - Main power supply voltage is above minimum threshold
     *          - Servo rail voltage is adequate (if monitored)
     *          - No power supply flags or warnings
     *          
     *          Calls into base AP_Arming::board_voltage_checks() after blimp-specific validation.
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if board voltages are acceptable, false otherwise
     * 
     * @note This check calls base class AP_Arming::board_voltage_checks()
     * @warning Low board voltage can cause system instability and crashes
     * 
     * @see AP_HAL::AnalogIn for voltage monitoring
     */
    bool board_voltage_checks(bool display_failure) override;

    /**
     * @brief Validate vehicle parameter configuration
     * 
     * @details Checks that critical vehicle parameters are set to reasonable values
     *          for safe blimp operation. Verifies parameter ranges, consistency
     *          between related parameters, and that required parameters are configured.
     *          
     *          Validates blimp-specific parameters:
     *          - Fin servo configuration and limits
     *          - Buoyancy compensation parameters
     *          - Control gain parameters
     *          - Flight mode parameters
     *          - Failsafe thresholds
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if parameters are valid, false otherwise
     * 
     * @note This check does NOT call base class AP_Arming - blimp-specific only
     * @warning Invalid parameters can cause poor flight performance or instability
     * 
     * @see Blimp::Parameters for blimp parameter definitions
     */
    bool parameter_checks(bool display_failure);

    /**
     * @brief Validate motor and fin servo functionality
     * 
     * @details Checks that motors (if equipped) and fin control servos are properly
     *          configured, connected, and responding to commands. For blimps, fin
     *          servos are the primary control mechanism and must be operational.
     *          
     *          Validates:
     *          - Servo channels are configured and mapped correctly
     *          - Servos respond to test commands (if safe to test)
     *          - Motor outputs are within expected ranges
     *          - No servo or motor errors detected
     *          - Thrust vectoring fins (if equipped) are functional
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if motors/servos are functional, false otherwise
     * 
     * @note This check does NOT call base class AP_Arming - blimp-specific only
     * @warning Non-functional control surfaces will prevent vehicle control
     * 
     * @see SRV_Channels for servo output management
     */
    bool motor_checks(bool display_failure);

    /**
     * @brief Validate object avoidance system (if equipped)
     * 
     * @details Checks that object avoidance sensors and algorithms are healthy
     *          if the system is configured and enabled. For blimps with proximity
     *          sensors or avoidance systems, ensures they are operational before flight.
     *          
     *          Validates:
     *          - Proximity sensors are detecting objects correctly
     *          - Avoidance algorithms are initialized
     *          - No sensor failures or inconsistencies
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if object avoidance is healthy (or not configured), false otherwise
     * 
     * @note This check does NOT call base class AP_Arming - blimp-specific only
     * @see AP_Proximity for proximity sensor interface
     * @see AP_Avoidance for avoidance algorithms
     */
    bool oa_checks(bool display_failure);

    /**
     * @brief Execute mandatory GPS checks that cannot be bypassed
     * 
     * @details Performs GPS validation that must pass even when ARMING_CHECK=0.
     *          These checks ensure minimum GPS functionality required for outdoor
     *          blimp operation where position hold is critical.
     *          
     *          Mandatory GPS requirements:
     *          - GPS is enabled and detecting satellites
     *          - At least minimum 2D fix for basic position awareness
     *          - No complete GPS failure
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if mandatory GPS requirements met, false otherwise
     * 
     * @note This check does NOT call base class AP_Arming - blimp-specific only
     * @warning Flying outdoors without any GPS can result in uncontrolled drift
     * 
     * @see gps_checks() for comprehensive GPS validation
     */
    bool mandatory_gps_checks(bool display_failure);

    /**
     * @brief Validate GCS connection and failsafe status
     * 
     * @details Checks that ground control station (GCS) connection is healthy and
     *          no GCS failsafe conditions are active. Ensures pilot has communication
     *          with vehicle before arming.
     *          
     *          Validates:
     *          - GCS telemetry link is active
     *          - No GCS failsafe triggered
     *          - Heartbeat messages being received
     *          - Command acknowledgment working
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if GCS connection healthy, false if failsafe active
     * 
     * @note This check does NOT call base class AP_Arming - blimp-specific only
     * @warning Arming with GCS failsafe active may indicate communication loss
     * 
     * @see GCS_Blimp for GCS interface implementation
     */
    bool gcs_failsafe_check(bool display_failure);

    /**
     * @brief Validate altitude sensor health and position
     * 
     * @details Checks that altitude measurement systems are healthy and vehicle
     *          is at a reasonable altitude for arming. For blimps, altitude control
     *          is critical and must be functional before flight.
     *          
     *          Validates:
     *          - Barometer altitude is reasonable for arming
     *          - GPS altitude available (if GPS enabled)
     *          - Altitude sources agree (barometer vs GPS)
     *          - No rapid altitude changes indicating sensor issues
     * 
     * @param[in] display_failure If true, send failure messages to GCS
     * 
     * @return true if altitude sensors healthy, false otherwise
     * 
     * @note This check does NOT call base class AP_Arming - blimp-specific only
     * @warning Poor altitude sensing can prevent altitude hold functionality
     * 
     * @see AP_Baro for barometric altitude
     * @see AP_GPS for GPS altitude
     */
    bool alt_checks(bool display_failure);

    /**
     * @brief Set pre-arm check status
     * 
     * @details Internal method to update the pre-arm check status flag. Used
     *          to track whether pre-arm checks have been completed successfully
     *          and maintain state between check executions.
     * 
     * @param[in] b New pre-arm check status (true=passed, false=not passed)
     * 
     * @note This is an internal state management method
     * @see run_pre_arm_checks() for pre-arm check execution
     */
    void set_pre_arm_check(bool b);

private:

    /**
     * @brief Internal implementation of pre-arm check execution
     * 
     * @details This method contains the actual implementation of all pre-arm checks.
     *          It is wrapped by pre_arm_checks() to allow storing the success/failure
     *          result for state tracking purposes. This separation allows the system
     *          to remember pre-arm check status and avoid redundant check execution.
     *          
     *          Executes all configured pre-arm checks in sequence:
     *          1. Parameter validation
     *          2. RC calibration checks
     *          3. Sensor health checks (INS, compass, GPS, barometer)
     *          4. EKF initialization and attitude check
     *          5. Motor and servo checks
     *          6. Battery and power checks
     *          7. Altitude sensor validation
     *          8. GCS connection check
     *          9. Object avoidance checks (if enabled)
     *          
     *          Check execution is controlled by ARMING_CHECK parameter bitmask.
     *          Failed checks prevent arming and generate GCS notifications if
     *          display_failure is true.
     * 
     * @param[in] display_failure If true, send detailed failure messages to GCS
     *                            to inform pilot of specific issues preventing arming
     * 
     * @return true if all applicable pre-arm checks pass, false if any check fails
     * 
     * @note This is an internal method called by pre_arm_checks() wrapper
     * @note Check results are cached to avoid redundant execution
     * 
     * @warning This is safety-critical code - all checks must be executed correctly
     * 
     * @see pre_arm_checks() for public interface to pre-arm validation
     * @see set_pre_arm_check() for state management
     */
    bool run_pre_arm_checks(bool display_failure);

};
