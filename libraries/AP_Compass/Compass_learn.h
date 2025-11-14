/**
 * @file Compass_learn.h
 * @brief Inflight compass offset learning using GPS velocity and AHRS attitude
 * 
 * This module implements automatic compass offset correction during flight using
 * the GSF (Gaussian Sum Filter) yaw estimator from the EKF. The learning algorithm
 * correlates GPS course-over-ground with magnetometer readings to determine accurate
 * compass offsets without requiring pre-flight calibration routines.
 * 
 * The system operates during normal flight and applies learned offsets when convergence
 * criteria are met, significantly reducing the need for manual compass calibration.
 */

#pragma once

/**
 * @class CompassLearn
 * @brief Inflight compass offset learning using fixed-yaw calibration with GSF yaw estimator
 * 
 * @details This class implements automatic compass offset correction during flight by
 *          leveraging the EKF's GSF (Gaussian Sum Filter) yaw estimator. The learning
 *          algorithm uses GPS velocity vectors and AHRS attitude to establish accurate
 *          yaw references, then calculates compass offsets that align magnetometer
 *          readings with the estimated yaw.
 * 
 *          **Learning Algorithm:**
 *          - Continuously monitors GSF yaw estimator convergence during flight
 *          - When yaw estimate accuracy meets threshold, performs fixed-yaw calibration
 *          - Calculates offsets for all active compass sensors simultaneously
 *          - Applies learned offsets and disables further learning upon success
 * 
 *          **Convergence Criteria:**
 *          - GSF yaw variance < 5 degrees
 *          - Maximum 1 yaw estimate clip event
 *          - Positive (valid) yaw variance from GSF
 * 
 *          **Safety Gates:**
 *          - Vehicle must be armed (prevents ground-based interference)
 *          - Minimum 3000ms flight time (clears ground effect and launch dynamics)
 *          - Pitch angle < 50 degrees (ensures reliable yaw determination)
 *          - GSF yaw estimator available and converged
 * 
 *          **Integration:**
 *          - Uses AP_AHRS for attitude and GSF yaw estimator access
 *          - Uses AP_Vehicle for flight time tracking
 *          - Integrates with existing Compass::mag_cal_fixed_yaw() calibration
 *          - Respects magnetic declination from AP_Declination
 * 
 * @note This learning method requires GPS velocity for course-over-ground determination
 * @note Learning only occurs when Compass::LearnType is set to INFLIGHT
 * @note Visual feedback provided via AP_Notify::flags.compass_cal_running
 * @note GCS status messages indicate learning start and completion
 * 
 * @warning Inflight learning requires adequate GPS velocity (typically >3 m/s for reliable course)
 * @warning Learning requires reasonably level flight (pitch < 50°) for accurate yaw estimation
 * @warning GSF yaw estimator depends on healthy GPS and IMU data
 * @warning Tailsitters must transition to forward flight before learning can occur
 * 
 * Source: libraries/AP_Compass/Compass_learn.h
 * Source: libraries/AP_Compass/Compass_learn.cpp:29-71
 */
class CompassLearn {
public:
    /**
     * @brief Construct a new CompassLearn object
     * 
     * @param[in] compass Reference to the main Compass object for offset learning
     * 
     * @details Initializes the inflight compass learning system and sends a GCS
     *          notification message indicating successful initialization. The learning
     *          system remains inactive until flight conditions and convergence criteria
     *          are met.
     * 
     * @note Sends "CompassLearn: Initialised" message to ground control station
     * 
     * Source: libraries/AP_Compass/Compass_learn.cpp:17-21
     */
    CompassLearn(class Compass &compass);

    /**
     * @brief Periodic update for inflight compass offset learning algorithm
     * 
     * @details Called on each compass read cycle to evaluate learning conditions and
     *          perform offset calibration when criteria are met. The method implements
     *          a multi-stage gating process:
     * 
     *          **Stage 1 - Configuration and Flight State:**
     *          - Verifies Compass::LearnType is set to INFLIGHT
     *          - Confirms vehicle is armed (soft_armed flag)
     *          - Ensures minimum flight time of 3000ms has elapsed
     * 
     *          **Stage 2 - Attitude Constraints:**
     *          - Checks pitch angle is within ±50 degrees
     *          - Prevents learning during extreme attitudes (launch, landing, aerobatics)
     *          - Ensures yaw determination reliability
     * 
     *          **Stage 3 - GSF Yaw Convergence:**
     *          - Retrieves yaw estimate and variance from GSF yaw estimator
     *          - Validates yaw variance < 25 deg² (5 degree threshold)
     *          - Ensures maximum 1 clip event in GSF
     *          - Confirms positive (valid) variance value
     * 
     *          **Stage 4 - Calibration Execution:**
     *          - Calls mag_cal_fixed_yaw() with GSF yaw angle
     *          - Applies learning to all active compass sensors
     *          - On success: disables learning, clears running flag, notifies GCS
     * 
     *          **Execution Frequency:**
     *          Called at compass sample rate (typically 75-100 Hz), but actual calibration
     *          only occurs when all gating conditions are satisfied.
     * 
     * @note Sets AP_Notify::flags.compass_cal_running during active learning attempts
     * @note Sends "CompassLearn: Finished" message upon successful completion
     * @note Automatically disables further learning after successful calibration
     * 
     * @warning Requires GSF yaw estimator availability (depends on EKF configuration)
     * @warning Learning disabled during extreme pitch attitudes (>50°) for yaw accuracy
     * @warning Minimum 3 second flight time prevents ground-based magnetic interference
     * 
     * Source: libraries/AP_Compass/Compass_learn.cpp:29-71
     */
    void update(void);

private:
    /**
     * @brief Reference to main Compass object for offset learning and calibration
     * 
     * @details Provides access to compass configuration (learn type), sensor data,
     *          and calibration functions (mag_cal_fixed_yaw). All learned offsets
     *          are applied through this reference, ensuring integration with the
     *          main compass subsystem.
     */
    Compass &compass;
};
