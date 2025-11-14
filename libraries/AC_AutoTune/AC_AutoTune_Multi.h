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
/*
  Multirotor implementation of AutoTune. Based on the original ArduCopter
  autotune code by Leonard Hall.
 */

/**
 * @file AC_AutoTune_Multi.h
 * @brief Multicopter autotune implementation using twitch-based step response analysis
 * 
 * @details This file implements multicopter-specific autotune functionality that uses
 *          step input "twitch" tests for system identification and gain tuning.
 * 
 * Twitch Methodology:
 * - Apply step rate or angle command to the vehicle
 * - Measure peak response and overshoot (bounce-back) characteristics
 * - Adjust PID gains based on response behavior
 * 
 * Heuristic Tuning Algorithm (fixed sequence):
 * 1. RATE_D_UP: Increase D gain until small controlled overshoot appears
 * 2. RATE_D_DOWN: Reduce D gain to eliminate excessive overshoot
 * 3. RATE_P_UP: Increase P gain while reducing D if needed to achieve fast clean response
 * 4. ANGLE_P_DOWN: Reduce angle P gain to eliminate angle overshoot
 * 5. ANGLE_P_UP: Increase angle P gain to achieve fast attitude response
 * 
 * Integration:
 * - Uses AC_AttitudeControl for rate/angle control authority
 * - Interfaces with AP_Motors for motor mixer configuration
 * - Reads from AP_AHRS for attitude and rate feedback
 * 
 * Based on the original ArduCopter autotune algorithm by Leonard Hall, proven on
 * thousands of multicopter vehicles in the field. Uses time-domain step response
 * analysis rather than frequency-domain methods.
 * 
 * @note Typical tuning session: 15-30 minutes for all three axes (roll, pitch, yaw)
 * @warning Requires adequate altitude (>5m recommended), clear airspace, and stable hover capability
 * 
 * @see AC_AutoTune Base class providing state machine framework
 * @see AC_AttitudeControl_Multi Multicopter attitude controller
 * @see AC_AutoTune_Heli Helicopter autotune (uses frequency-response approach instead)
 * 
 * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h
 */

#pragma once

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune.h"

/**
 * @class AC_AutoTune_Multi
 * @brief Multicopter autotune using twitch-based step response analysis
 * 
 * @details Provides multicopter-specific implementation of the AutoTune framework.
 * 
 * Architecture:
 * - Inherits from AC_AutoTune base class, providing multicopter-specific tuning algorithms
 * - Uses step response (twitch) testing: apply step command, measure transient response
 * - Heuristic gain adjustment: compare measured response to target, adjust gains iteratively
 * - Multi-axis tuning: roll, pitch, yaw tested independently via axis_bitmask parameter
 * - Fixed tuning sequence: RATE_D_UP → RATE_D_DOWN → RATE_P_UP → ANGLE_P_DOWN → ANGLE_P_UP
 * 
 * Step Response Testing:
 * - Rate twitch: Command target rate (typically 180 deg/s roll/pitch, 90 deg/s yaw)
 * - Angle twitch: Command target angle (typically 20 deg roll/pitch, 45 deg yaw)
 * - Measure peak values, overshoot (bounce-back), and settling characteristics
 * - Duration: Typically 0.5-2 seconds per test depending on vehicle response
 * 
 * Gain Adjustment Strategy:
 * - Aggressiveness parameter controls target overshoot tolerance (0 = no overshoot, 0.1 = 10%)
 * - Rate D tuning: Increase until small controlled bounce appears, then reduce if excessive
 * - Rate P tuning: Increase for fast response while keeping bounce within tolerance
 * - Angle P tuning: Adjust to achieve fast attitude response without overshoot
 * - Typical gain changes: 10-50% per iteration depending on response characteristics
 * 
 * Safety Mechanisms:
 * - Abort limits on angle/rate excursions to prevent crashes
 * - Pilot override detection to cancel tuning
 * - Position hold requirement between twitches
 * - Parameter backup/restore on start/completion/failure
 * 
 * Progress Reporting:
 * - Real-time status updates via MAVLink STATUSTEXT messages
 * - GCS notifications of current axis, tune step, and estimated completion
 * - Final gain reports when tuning completes
 * 
 * Logging Support (when HAL_LOGGING_ENABLED):
 * - ATUN: Summary log with test results and calculated gains
 * - ATDE: Time history log with angle/rate data during twitch execution
 * 
 * Coordinate System:
 * - Body frame rates in centidegrees/s (cds)
 * - Angles in centidegrees (cd)
 * 
 * Thread Safety:
 * - Called from main loop at ~400Hz
 * - NOT thread-safe for concurrent access
 * 
 * Integration:
 * - Uses AC_AttitudeControl_Multi for rate/angle control
 * - Uses AP_Motors_Multi for motor mixer configuration
 * - Uses AP_AHRS for attitude/rate feedback
 * - Uses AP_Param for parameter persistence
 * 
 * Performance:
 * - Typical session: 15-30 minutes to tune all three axes
 * - Duration varies based on vehicle response characteristics
 * 
 * @note Based on original autotune by Leonard Hall, time-tested on thousands of vehicles
 * @warning Never run autotune with weak/damaged propellers or loose components
 * @warning Requires altitude >5m, clear airspace, and ability to maintain stable hover
 * 
 * @see AC_AutoTune Base class with state machine framework
 * @see AC_AttitudeControl_Multi Multicopter attitude controller  
 * @see AP_Motors_Multi Multicopter motor mixer
 * @see AC_AutoTune_Heli Alternative frequency-response approach for helicopters
 * 
 * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:28-213
 */
class AC_AutoTune_Multi : public AC_AutoTune
{
public:
    /**
     * @brief Constructor for multicopter autotune
     * 
     * @details Initializes base class, sets default parameters, and initializes
     *          measurement variables. Registers parameter group via var_info[]
     *          for AP_Param persistence.
     * 
     * @note Called during vehicle initialization before arming
     */
    AC_AutoTune_Multi();

    /**
     * @brief Save tuned gains to EEPROM on disarm
     * 
     * @details Persists tuned gains via AP_Param::save() when vehicle disarms
     *          after successful autotune. Only saves if tuning succeeded and
     *          tuned gains are currently active.
     * 
     * @note Called automatically on disarm by AC_AutoTune base class
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:35
     */
    void save_tuning_gains() override;

    /**
     * @brief Parameter group information for AP_Param registration
     * 
     * @details Defines multicopter-specific autotune parameters:
     * - AXES: Axis bitmask (0x7 = roll+pitch+yaw, bit 0=roll, bit 1=pitch, bit 2=yaw)
     * - AGGR: Aggressiveness (0.0-0.1, default 0.05 for 5% target overshoot)
     * - MIN_D: Minimum D gain floor (0.001-0.006, prevents excessively low damping)
     * 
     * Used by AP_Param::setup_object_defaults() for parameter initialization
     * and EEPROM persistence.
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:38
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:
    //
    // Gain management and initialization
    //

    /**
     * @brief Backup original gains and initialize tuning state
     * 
     * @details Stores current rate PID gains (P/I/D/FF, filters) and angle P gains
     *          for all axes in orig_* variables before tuning starts. Allows
     *          restoration if tuning fails or user aborts. Initializes tuning
     *          state machine variables and measurement arrays.
     * 
     * @warning Must be called before starting autotune to ensure original gains
     *          can be restored on abort or failure
     * 
     * @note Called once at autotune start before first twitch test
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:46
     */
    void backup_gains_and_initialise() override;

    /**
     * @brief Restore original pre-tune gains to attitude controller
     * 
     * @details Applies backed-up original gains from backup_gains_and_initialise().
     *          Called when user switches to original gains via AUX function or
     *          when autotune aborts/fails.
     * 
     * @note Restores rate P/I/D/FF, filters, and angle P for all axes
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:49
     */
    void load_orig_gains() override;

    /**
     * @brief Apply gains from last successful autotune
     * 
     * @details Applies stored tuned gains to attitude controller. Called when
     *          user switches to tuned gains via AUX function or when autotune
     *          completes successfully.
     * 
     * @note Only valid if a previous autotune session completed successfully
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:52
     */
    void load_tuned_gains() override;

    /**
     * @brief Load conservative intermediate gains used between twitch tests
     * 
     * @details Applies current working gains (partially tuned) during return-to-level
     *          phase between twitches. Ensures stable hover while waiting for next
     *          test. More conservative than test gains to prevent oscillation.
     * 
     * @note Called after each twitch completes and vehicle returns to level
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:55
     */
    void load_intra_test_gains() override;

    /**
     * @brief Load gains under test for current twitch
     * 
     * @details Applies test gains to attitude controller immediately before
     *          executing twitch. Test gains are the candidate gains being
     *          evaluated in current test iteration.
     * 
     * @note Called just before test_run() executes twitch
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:58
     */
    void load_test_gains() override;

    /**
     * @brief Reset vehicle-specific test state variables (unused for multirotor)
     * 
     * @details Empty implementation - multirotor doesn't need vehicle-specific
     *          test variable reset like helicopter does. Included for interface
     *          compatibility with AC_AutoTune base class.
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:61
     */
    void reset_vehicle_test_variables() override {};

    /**
     * @brief Reset vehicle-specific gain tracking state (unused for multirotor)
     * 
     * @details Empty implementation - multirotor doesn't need vehicle-specific
     *          gain variable reset like helicopter does. Included for interface
     *          compatibility with AC_AutoTune base class.
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:64
     */
    void reset_update_gain_variables() override {};

    /**
     * @brief Maximum target angle for roll-pitch axis angle twitch tests
     * 
     * @return Maximum target angle in centidegrees (typically 2000 cd = 20 degrees)
     * 
     * @details Defines target amplitude for roll and pitch angle twitch tests.
     *          Ensures test produces sufficient response for measurement without
     *          excessive excursion. Typical value: ±20 degrees.
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:67
     */
    float target_angle_max_rp_cd() const override;

    /**
     * @brief Maximum target angle for yaw axis angle twitch tests
     * 
     * @return Maximum target angle in centidegrees (typically 4500 cd = 45 degrees)
     * 
     * @details Defines target amplitude for yaw angle twitch tests. Yaw typically
     *          uses larger angle than roll/pitch due to slower yaw dynamics.
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:68
     */
    float target_angle_max_y_cd() const override;

    /**
     * @brief Minimum target angle for roll-pitch axis angle twitch tests
     * 
     * @return Minimum target angle in centidegrees (negative of maximum)
     * 
     * @details Defines negative target amplitude for roll and pitch tests.
     *          Typically the negative of target_angle_max_rp_cd().
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:69
     */
    float target_angle_min_rp_cd() const override;

    /**
     * @brief Minimum target angle for yaw axis angle twitch tests
     * 
     * @return Minimum target angle in centidegrees (negative of maximum)
     * 
     * @details Defines negative target amplitude for yaw tests. Typically
     *          the negative of target_angle_max_y_cd().
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:70
     */
    float target_angle_min_y_cd() const override;

    /**
     * @brief Abort angle limit for roll-pitch twitch safety
     * 
     * @return Maximum absolute angle in centidegrees beyond which test aborts (typically 9000 cd = 90 degrees)
     * 
     * @details Safety limit that aborts twitch if exceeded. Prevents crashes from
     *          runaway response due to bad gains. Test abort restores previous
     *          good gains and may require restarting axis tuning.
     * 
     * @warning Pilot must maintain altitude and position hold during twitches
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:73
     */
    float angle_lim_max_rp_cd() const override;

    /**
     * @brief Negative abort angle limit for all axes twitch safety
     * 
     * @return Maximum negative angle in centidegrees beyond which test aborts (typically -9000 cd = -90 degrees)
     * 
     * @details Safety limit for negative angles that aborts twitch if exceeded.
     *          Applied to roll, pitch, and yaw axes. Prevents vehicle from
     *          flipping or spinning excessively.
     * 
     * @warning Excessive excursion triggers abort and restores previous gains
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:74
     */
    float angle_lim_neg_rpy_cd() const override;

    /**
     * @brief Initialize state for new twitch test
     * 
     * @details Resets measurement variables (test_rate_min/max, test_angle_min/max),
     *          sets target rate or angle based on current tune type, configures
     *          abort threshold. Prepares for twitch execution.
     * 
     * @note Must complete before test_run() is called
     * @note Called once per twitch test during TESTING_INIT state
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:77
     */
    void test_init() override;

    /**
     * @brief Execute one twitch test iteration
     * 
     * @param test_axis Axis being tested (ROLL, PITCH, or YAW)
     * @param dir_sign Direction of test excitation (+1 or -1)
     * 
     * @details Core test execution called at ~400Hz during EXECUTING_TEST state.
     * 
     * Test Execution:
     * - Generate step command (rate or angle based on tune type)
     * - Apply command to attitude controller
     * - Monitor response (angle, rate, acceleration)
     * - Track peak values and detect test completion
     * - Abort if safety limits exceeded
     * 
     * Rate Twitch:
     * - Command target rate (typically 180 deg/s roll/pitch, 90 deg/s yaw)
     * - Measure peak rate and overshoot (bounce-back)
     * - Calls twitching_test_rate() for response monitoring
     * - Duration: 0.5-1 second typically
     * 
     * Angle Twitch:
     * - Command target angle (typically 20 deg roll/pitch, 45 deg yaw)
     * - Measure peak angle and rate
     * - Calls twitching_test_angle() for response monitoring
     * - Duration: 1-2 seconds typically
     * 
     * Abort Handling:
     * - Calls twitching_abort_rate() to check safety limits
     * - Restores previous gains if abort triggered
     * - May require restarting axis tuning
     * 
     * @warning Must be called consistently at ~400Hz for correct response measurement
     * @note Test duration varies based on vehicle response speed
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:80
     */
    void test_run(AxisType test_axis, const float dir_sign) override;

    /**
     * @brief Send periodic status updates to GCS during tuning
     * 
     * @details Transmits MAVLink STATUSTEXT messages with tuning progress:
     * - Current axis being tuned
     * - Current tune step (RATE_D_UP, RATE_P_UP, etc.)
     * - Success counter showing iterations completed
     * - Estimated completion time
     * 
     * Rate-limited to avoid flooding GCS. Keeps operator informed during
     * tuning session which may last 15-30 minutes.
     * 
     * @note Called periodically during tuning session
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:83
     */
    void do_gcs_announcements() override;

    /**
     * @brief Unused placeholder for post-test announcements
     * 
     * @details Empty implementation - multirotor provides status via
     *          do_gcs_announcements() instead. Included for interface
     *          compatibility with AC_AutoTune base class.
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:86
     */
    void do_post_test_gcs_announcements() override {};

    /**
     * @brief Report final tuned gains for axis to GCS
     * 
     * @param test_axis Axis to report (const to prevent modification)
     * 
     * @details Transmits final tuned gains via STATUSTEXT when tuning completes:
     * - Rate P/I/D gains
     * - Angle P gain
     * - Max angular acceleration
     * 
     * Operator can compare to original gains and assess tuning result.
     * 
     * @note Called once per axis when tuning sequence completes
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:89
     */
    void report_final_gains(AxisType test_axis) const override;

    /**
     * @brief Update rate P gain to achieve fast clean response
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Increase rate P until target rate is reached quickly (~0.5s to peak).
     *          Simultaneously reduce rate D if bounce-back (overshoot) exceeds
     *          aggressiveness threshold.
     * 
     * Goal: Fast response with minimal overshoot
     * - Typical P increase: 25-50% per iteration
     * - Stops when clean response achieved or D hits minimum
     * - May fail if D reaches minimum and clean response impossible
     * 
     * Calls updating_rate_p_up_d_down() with measured min/max rates from twitch.
     * 
     * @note Called during UPDATE_GAINS state after rate P twitch completes
     * @warning Can fail if vehicle characteristics don't allow clean response with minimum D
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:92
     */
    void updating_rate_p_up_all(AxisType test_axis) override;

    /**
     * @brief Update rate D gain to produce controlled overshoot
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Increase D until small controlled bounce-back appears, indicating
     *          damping is entering optimal range. Simultaneously adjust P to keep
     *          peak response just below target.
     * 
     * Target overshoot: aggressiveness parameter (typically 5%)
     * - Insufficient D: no bounce, sluggish response
     * - Excessive D: large bounce, noise sensitivity
     * - Typical D increase: 20-40% per iteration
     * 
     * Calls updating_rate_d_up() with measured response characteristics.
     * 
     * @note Called during UPDATE_GAINS state after rate D UP twitch completes
     * @note Bounce-back (overshoot in reverse direction) indicates damping level
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:93
     */
    void updating_rate_d_up_all(AxisType test_axis) override;

    /**
     * @brief Update rate D gain to eliminate excessive overshoot
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Decrease D if bounce-back exceeds aggressiveness threshold.
     *          Simultaneously adjust P to maintain response speed.
     * 
     * Goal: Clean response at target rate with acceptable overshoot
     * - Typical D decrease: 10-20% per iteration
     * - Stops when overshoot within tolerance
     * 
     * Calls updating_rate_d_down() with measured response.
     * 
     * @note Called during UPDATE_GAINS state after rate D DOWN twitch completes
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:94
     */
    void updating_rate_d_down_all(AxisType test_axis) override;

    /**
     * @brief Update angle P gain for fast attitude response
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Increase angle P until target angle is reached within timeout (~1s).
     * 
     * Tuning Strategy:
     * - Too low P: slow attitude response
     * - Too high P: potential oscillation or rate saturation
     * - Typical P increase: 25% per iteration
     * - Stops when target consistently achieved
     * 
     * Calls updating_angle_p_up() with measured angle extremes.
     * 
     * @note Called during UPDATE_GAINS state after angle P UP twitch completes
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:95
     */
    void updating_angle_p_up_all(AxisType test_axis) override;

    /**
     * @brief Update angle P gain to eliminate angle overshoot
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Decrease angle P if angle overshoot detected (exceeds target).
     * 
     * Goal: Reach target without overshoot
     * - Typical P decrease: 10% per iteration
     * - Ensures smooth attitude control without oscillation
     * 
     * Calls updating_angle_p_down() with measured angle max.
     * 
     * @note Called during UPDATE_GAINS state after angle P DOWN twitch completes
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:96
     */
    void updating_angle_p_down_all(AxisType test_axis) override;

    /**
     * @brief Unused tune type for multirotor (only used by helicopter)
     * 
     * @details Intentionally triggers internal error if called - rate feedforward
     *          tuning not part of multirotor sequence. Rate FF provides minimal
     *          benefit for step response testing used by multirotors.
     * 
     * @warning Internal error indicates logic bug if this is called
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:99
     */
    void updating_rate_ff_up_all(AxisType) override {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    /**
     * @brief Unused tune type for multirotor (only used by helicopter)
     * 
     * @details Intentionally triggers internal error if called - max gains testing
     *          not part of multirotor sequence. Heuristic approach doesn't require
     *          explicit stability boundary identification.
     * 
     * @warning Internal error indicates logic bug if this is called
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:102
     */
    void updating_max_gains_all(AxisType) override {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    /**
     * @brief Apply final tuned gains with safety backoff/margin
     * 
     * @param test_axis Axis for which to set final gains
     * 
     * @details Calculate final tuned gains by reducing measured optimal gains
     *          slightly for safety margin. Typical backoff: 10-15% reduction.
     *          Stores in tune_* variables for application to attitude controller.
     * 
     * Conservative approach prevents overly aggressive tuning that might
     * cause oscillation in varying conditions (wind, battery voltage, etc.).
     * 
     * @note Called at end of each tune type before moving to next step
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:107
     */
    void set_tuning_gains_with_backoff(AxisType test_axis) override;

    /**
     * @brief Determine whether to reverse next twitch direction
     * 
     * @return true to reverse direction (e.g., +dir to -dir), false to keep same direction
     * 
     * @details Returns opposite of current direction (!positive_direction).
     *          Multirotor alternates twitch direction to average out asymmetries
     *          and ensure gains work in both directions.
     * 
     * @note Direction alternation helps identify and compensate for vehicle asymmetry
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:110
     */
    bool reverse_test_direction() override { return !positive_direction; }

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log autotune summary data for current twitch test
     * 
     * @details Writes ATUN log message with test results:
     * - Axis tested (ROLL, PITCH, YAW)
     * - Tune step (RATE_D_UP, RATE_P_UP, etc.)
     * - Measured response characteristics (target, min, max)
     * - Calculated gains (rate P/D, angle P)
     * - Max angular acceleration
     * 
     * Called after each twitch completes. Enables post-flight analysis
     * of tuning progression and gain convergence.
     * 
     * @note Logged once per twitch test
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:113
     */
    void Log_AutoTune() override;

    /**
     * @brief Log autotune time history data during twitch execution
     * 
     * @details Writes ATDE log message with time-series data at ~400Hz during twitch:
     * - Measured angle from AHRS (centidegrees)
     * - Measured rate from gyro (centidegrees/s)
     * 
     * Enables detailed visualization of step response transients for
     * post-flight analysis and algorithm validation.
     * 
     * @note Logged at main loop rate (~400Hz) during test execution
     * @note Simpler than helicopter which also logs commands and targets separately
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:114
     */
    void Log_AutoTuneDetails() override;

    /**
     * @brief Unused sweep logging (only used by helicopter)
     * 
     * @details Intentionally triggers internal error if called - frequency sweep
     *          logging not used by multirotor. Multirotor uses time-domain step
     *          response instead of frequency-domain analysis.
     * 
     * @warning Internal error indicates logic bug if this is called
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:115
     */
    void Log_AutoTuneSweep() override {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    /**
     * @brief Write ATUN log message with test summary
     * 
     * @param axis Axis tested (ROLL, PITCH, YAW)
     * @param tune_step Current tune type (RATE_D_UP, RATE_P_UP, etc.)
     * @param meas_target Target rate or angle for test
     * @param meas_min Minimum measured rate or angle during test
     * @param meas_max Maximum measured rate or angle during test
     * @param new_gain_rp Calculated rate P gain
     * @param new_gain_rd Calculated rate D gain
     * @param new_gain_sp Calculated angle P gain
     * @param new_ddt Calculated max angular acceleration (centidegrees/s/s)
     * 
     * @details Formats and writes structured log entry for post-flight analysis.
     *          Allows visualization of gain convergence over tuning session.
     * 
     * @note All angle values in centidegrees, rates in centidegrees/s
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:119
     */
    void Log_Write_AutoTune(AxisType axis, TuneType tune_step,
                            float meas_target, float meas_min, float meas_max,
                            float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);

    /**
     * @brief Write ATDE log message with time history data
     * 
     * @param angle_cd Measured angle from AHRS (centidegrees)
     * @param rate_cds Measured rate from gyro (centidegrees/s)
     * 
     * @details Minimal time history logging for multirotor - simpler than helicopter
     *          which also logs commands and targets separately. Captures essential
     *          response data for post-flight transient analysis.
     * 
     * @note Called at ~400Hz during twitch execution
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:122
     */
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
#endif

    /**
     * @brief Define fixed tuning sequence for multicopter
     * 
     * @details Configures tune_seq[] array with multicopter-specific sequence:
     * 1. RATE_D_UP: Tune D gain to produce controlled bounce-back
     * 2. RATE_D_DOWN: Reduce D if bounce excessive
     * 3. RATE_P_UP: Increase P for fast response while controlling bounce
     * 4. ANGLE_P_DOWN: Reduce angle P to eliminate angle overshoot
     * 5. ANGLE_P_UP: Increase angle P for fast attitude response
     * 6. TUNE_COMPLETE: Mark sequence finished
     * 
     * Sequence is fixed (not configurable like helicopter). Order is critical:
     * tune D first to establish damping, then P for response speed, then
     * angle control last.
     * 
     * @note Unlike helicopter, multirotor sequence cannot be customized via parameter
     * @note Called once during autotune initialization
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:126
     */
    void set_tune_sequence() override {
        tune_seq[0] = TuneType::RATE_D_UP;
        tune_seq[1] = TuneType::RATE_D_DOWN;
        tune_seq[2] = TuneType::RATE_P_UP;
        tune_seq[3] = TuneType::ANGLE_P_DOWN;
        tune_seq[4] = TuneType::ANGLE_P_UP;
        tune_seq[5] = TuneType::TUNE_COMPLETE;
    }

    /**
     * @brief Get axes to be tuned
     * 
     * @return Bitmask of axes (bit 0=roll, bit 1=pitch, bit 2=yaw)
     * 
     * @details Returns axis_bitmask parameter value. Used by base class to
     *          determine which axes to tune. Typical: 0x7 (all three axes).
     * 
     * Examples:
     * - 0x7 (binary 111): Tune roll, pitch, and yaw
     * - 0x3 (binary 011): Tune roll and pitch only
     * - 0x1 (binary 001): Tune roll only
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:136
     */
    uint8_t get_axis_bitmask() const override { return axis_bitmask; }

    /**
     * @brief Get timeout for single twitch test
     * 
     * @return Timeout in milliseconds
     * 
     * @details Returns timeout for twitch test execution. Prevents infinite loops
     *          if test fails to complete naturally.
     * 
     * Typical timeouts:
     * - Rate tests: 1000-1500ms (fast response expected)
     * - Angle tests: 1500-2000ms (slower settling)
     * 
     * Timeout varies by tune type to account for different response speeds.
     * 
     * @note Called by base class when starting each twitch
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:139
     */
    uint32_t get_testing_step_timeout_ms() const override;

private:
    /**
     * @brief Monitor rate twitch test and track response extremes
     * 
     * @param angle Current measured angle (centidegrees)
     * @param rate Current measured rate (centidegrees/s)
     * @param rate_target Target rate for test (centidegrees/s)
     * @param[out] meas_rate_min Minimum rate reached (centidegrees/s, negative for reverse direction)
     * @param[out] meas_rate_max Maximum rate reached (centidegrees/s)
     * @param[out] meas_angle_min Minimum angle reached (centidegrees)
     * 
     * @details Called each loop iteration during rate twitch. Tracks peak rate
     *          in both directions:
     * - meas_rate_max: Peak rate in primary response direction
     * - meas_rate_min: Peak rate in bounce-back (overshoot) direction
     * - meas_angle_min: Angle excursion for abort detection
     * 
     * Updates output parameters as new extremes detected. Essential for
     * evaluating D gain effectiveness via bounce-back ratio.
     * 
     * @note Called at ~400Hz during rate twitch execution
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:144
     */
    void twitching_test_rate(float angle, float rate, float rate_target,
                             float &meas_rate_min, float &meas_rate_max, float &meas_angle_min);

    /**
     * @brief Check abort conditions for rate twitch safety
     * 
     * @param angle Current measured angle (centidegrees)
     * @param rate Current measured rate (centidegrees/s)
     * @param angle_max Maximum allowed angle (centidegrees)
     * @param meas_rate_min Minimum rate reached (centidegrees/s)
     * @param angle_min Minimum angle reached (centidegrees)
     * 
     * @details Evaluates whether twitch should abort due to excessive excursion.
     *          Aborts if:
     * - Angle exceeds angle_max safety limit
     * - Other safety thresholds violated
     * 
     * Called each loop iteration during test. Abort restores previous gains
     * and may log failure.
     * 
     * @warning Abort restores previous gains and may require restarting axis tuning
     * @note Called at ~400Hz during rate twitch execution
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:147
     */
    void twitching_abort_rate(float angle, float rate, float angle_max,
                              float meas_rate_min, float angle_min);

    /**
     * @brief Monitor angle twitch test and track response extremes
     * 
     * @param angle Current measured angle (centidegrees)
     * @param rate Current measured rate (centidegrees/s)
     * @param angle_target Target angle for test (centidegrees)
     * @param[out] meas_angle_min Minimum angle reached (centidegrees)
     * @param[out] meas_angle_max Maximum angle reached (centidegrees)
     * @param[out] meas_rate_min Minimum rate during test (centidegrees/s)
     * @param[out] meas_rate_max Maximum rate during test (centidegrees/s)
     * 
     * @details Called each loop iteration during angle twitch. Tracks:
     * - Peak angles: Indicate overshoot behavior
     * - Peak rates: Used for acceleration calculation and abort detection
     * 
     * Angle peaks determine whether angle P gain needs adjustment. Rate
     * peaks provide additional safety monitoring.
     * 
     * @note Called at ~400Hz during angle twitch execution
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:150
     */
    void twitching_test_angle(float angle, float rate, float angle_target,
                              float &meas_angle_min, float &meas_angle_max,
                              float &meas_rate_min, float &meas_rate_max);

    /**
     * @brief Calculate average angular acceleration during twitch
     * 
     * @param[in,out] accel_average Running average of acceleration (centidegrees/s/s)
     * @param rate Current measured rate (centidegrees/s)
     * @param[in,out] rate_max Maximum rate observed (centidegrees/s)
     * 
     * @details Estimates acceleration from rate change. Used to calculate
     *          max_accel parameter for attitude controller. Simple differencing
     *          method, averaged over twitch duration.
     * 
     * Max acceleration parameter limits attitude controller rate of change
     * to prevent abrupt maneuvers.
     * 
     * @note Acceleration in centidegrees/s/s (cdss)
     * @note Called at ~400Hz during twitch for continuous averaging
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:155
     */
    void twitching_measure_acceleration(float &accel_average, float rate, float &rate_max) const;

    /**
     * @brief Increase D gain to produce controlled bounce-back
     * 
     * @param[in,out] tune_d Current D gain being adjusted
     * @param tune_d_min Minimum D gain limit
     * @param tune_d_max Maximum D gain limit
     * @param tune_d_step_ratio D adjustment step size (typically 0.2-0.4 for 20-40%)
     * @param[in,out] tune_p Current P gain (adjusted to compensate)
     * @param tune_p_min Minimum P gain limit
     * @param tune_p_max Maximum P gain limit
     * @param tune_p_step_ratio P adjustment step size
     * @param rate_target Target rate for test (centidegrees/s)
     * @param meas_rate_min Minimum measured rate (bounce-back, centidegrees/s)
     * @param meas_rate_max Maximum measured rate (peak response, centidegrees/s)
     * 
     * @details Core D tuning algorithm that evaluates bounce-back ratio and adjusts gains:
     * 
     * Algorithm:
     * 1. Calculate bounce ratio = abs(meas_rate_min) / meas_rate_max
     * 2. If bounce too small (<aggressiveness): increase D
     * 3. Simultaneously adjust P to keep peak rate near target
     * 4. Success when controlled bounce achieved without exceeding aggressiveness threshold
     * 
     * Tuning Criteria:
     * - Insufficient bounce (<aggressiveness): Need more D for damping
     * - Controlled bounce (≈aggressiveness): D is optimal
     * - Excessive bounce (>aggressiveness): Too much D, move to D_DOWN step
     * 
     * Based on Leonard Hall's original autotune heuristic algorithm.
     * 
     * @note Bounce-back (overshoot in reverse direction) indicates damping level
     * @note Typical aggressiveness: 0.05 (5% overshoot target)
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:162
     */
    void updating_rate_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio,
                            float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio,
                            float rate_target, float meas_rate_min, float meas_rate_max);

    /**
     * @brief Decrease D gain to eliminate excessive bounce-back
     * 
     * @param[in,out] tune_d Current D gain being adjusted
     * @param tune_d_min Minimum D gain (floor at MIN_D parameter)
     * @param tune_d_step_ratio D adjustment step size (typically 0.1-0.2 for 10-20%)
     * @param[in,out] tune_p Current P gain (adjusted to compensate)
     * @param tune_p_min Minimum P gain limit
     * @param tune_p_max Maximum P gain limit
     * @param tune_p_step_ratio P adjustment step size
     * @param rate_target Target rate for test (centidegrees/s)
     * @param meas_rate_min Minimum measured rate (bounce-back, centidegrees/s)
     * @param meas_rate_max Maximum measured rate (peak response, centidegrees/s)
     * 
     * @details Reduces D if bounce-back exceeds aggressiveness threshold:
     * 
     * Algorithm:
     * 1. Calculate bounce ratio = abs(meas_rate_min) / meas_rate_max
     * 2. If bounce too large (>aggressiveness): decrease D
     * 3. Maintain P to keep response speed
     * 4. Success when bounce within tolerance or D reaches minimum
     * 
     * Iteration:
     * - Typical D decrease: 10-20% per iteration
     * - Smaller steps than D_UP for fine-tuning
     * - Stops when bounce acceptable
     * 
     * @note Called after RATE_D_UP if excessive bounce detected
     * @note Success when clean response with acceptable overshoot achieved
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:168
     */
    void updating_rate_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio,
                              float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio,
                              float rate_target, float meas_rate_min, float meas_rate_max);

    /**
     * @brief Increase P for fast response while reducing D if bounce excessive
     * 
     * @param[in,out] tune_d Current D gain being adjusted
     * @param tune_d_min Minimum D gain
     * @param tune_d_step_ratio D adjustment step size
     * @param[in,out] tune_p Current P gain being adjusted
     * @param tune_p_min Minimum P gain limit
     * @param tune_p_max Maximum P gain limit
     * @param tune_p_step_ratio P adjustment step size (typically 0.25-0.5 for 25-50%)
     * @param rate_target Target rate for test (centidegrees/s)
     * @param meas_rate_min Minimum measured rate (bounce-back, centidegrees/s)
     * @param meas_rate_max Maximum measured rate (peak response, centidegrees/s)
     * @param fail_min_d If true, fail tuning if D reaches minimum (optional, default true)
     * 
     * @details Most aggressive tuning step that prioritizes response speed:
     * 
     * Algorithm:
     * 1. Increase P until target rate reached quickly (~0.5s to peak)
     * 2. Reduce D if bounce-back exceeds aggressiveness threshold
     * 3. Balance P increase with D decrease for clean fast response
     * 4. Success when target reached quickly with acceptable overshoot
     * 5. Fail if D hits minimum and clean response impossible (indicates poorly tuned vehicle)
     * 
     * Tuning Strategy:
     * - Aggressive P increases: 25-50% per iteration
     * - Simultaneously reduce D to control bounce
     * - Goal: Fast clean response with minimal overshoot
     * 
     * Failure Conditions:
     * - D reaches minimum (MIN_D parameter) and clean response not achievable
     * - Indicates vehicle characteristics don't allow optimal tuning
     * - May require parameter adjustments or mechanical fixes
     * 
     * @warning Can fail if vehicle characteristics don't allow clean response with minimum D
     * @note Most complex gain update - balances competing objectives
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:175
     */
    void updating_rate_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio,
                                    float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio,
                                    float rate_target, float meas_rate_min, float meas_rate_max,
                                    bool fail_min_d = true);

    /**
     * @brief Decrease angle P to eliminate angle overshoot
     * 
     * @param[in,out] tune_p Current angle P gain
     * @param tune_p_min Minimum angle P gain
     * @param tune_p_step_ratio P adjustment step size (typically 0.1 for 10%)
     * @param angle_target Target angle for test (centidegrees)
     * @param meas_angle_max Maximum angle reached (centidegrees)
     * @param meas_rate_min Minimum rate for validation (centidegrees/s)
     * @param meas_rate_max Maximum rate for validation (centidegrees/s)
     * 
     * @details Reduce P if angle overshoot detected:
     * 
     * Algorithm:
     * 1. Compare meas_angle_max to angle_target
     * 2. If overshoot detected (meas_angle_max > angle_target): decrease P
     * 3. Success when target reached without overshoot
     * 
     * Overshoot Indication:
     * - Angle P too high causes attitude to overshoot target
     * - Results in oscillatory behavior in attitude control
     * - Need to reduce P for smooth control
     * 
     * Iteration:
     * - Conservative P decrease: 10% per iteration
     * - Smaller steps to avoid overcorrection
     * - Ensures smooth attitude control
     * 
     * @note Called during ANGLE_P_DOWN tuning step
     * @note Success when smooth attitude response without oscillation
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:182
     */
    void updating_angle_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio,
                               float angle_target, float meas_angle_max,
                               float meas_rate_min, float meas_rate_max);

    /**
     * @brief Increase angle P for fast attitude response
     * 
     * @param[in,out] tune_p Current angle P gain
     * @param tune_p_max Maximum angle P gain
     * @param tune_p_step_ratio P adjustment step size (typically 0.25 for 25%)
     * @param angle_target Target angle for test (centidegrees)
     * @param meas_angle_max Maximum angle reached (centidegrees)
     * @param meas_rate_min Minimum rate for validation (centidegrees/s)
     * @param meas_rate_max Maximum rate for validation (centidegrees/s)
     * 
     * @details Increase P until target angle reached consistently:
     * 
     * Algorithm:
     * 1. Increase P until meas_angle_max reaches angle_target within timeout
     * 2. Success when target angle consistently achieved
     * 3. Stop when fast response without rate saturation
     * 
     * Tuning Criteria:
     * - Too low P: Slow sluggish attitude response, doesn't reach target
     * - Optimal P: Fast response, reaches target quickly
     * - Too high P: Potential oscillation or rate saturation
     * 
     * Iteration:
     * - Typical P increase: 25% per iteration
     * - Stops when target consistently achieved
     * - Ensures responsive attitude control
     * 
     * @note Called during ANGLE_P_UP tuning step (final step)
     * @note Success completes autotune sequence for axis
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:188
     */
    void updating_angle_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio,
                             float angle_target, float meas_angle_max,
                             float meas_rate_min, float meas_rate_max);

    /**
     * @brief Format and send gain report message to GCS
     * 
     * @param axis_string Axis name string ("Roll", "Pitch", or "Yaw")
     * @param rate_P Rate P gain
     * @param rate_I Rate I gain
     * @param rate_D Rate D gain
     * @param angle_P Angle P gain
     * @param max_accel Maximum angular acceleration (centidegrees/s/s)
     * 
     * @details Formats gains into human-readable STATUSTEXT message for GCS display.
     *          Called by report_final_gains() to transmit final tuned gains when
     *          tuning completes for an axis.
     * 
     * Message format: "[Axis] Rate: P=X.XXX I=X.XXX D=X.XXXX Angle: P=X.X MaxAccel=XXXXX"
     * 
     * Operator can compare to original gains and assess tuning result quality.
     * 
     * @note Gains formatted with appropriate precision for readability
     * 
     * Source: libraries/AC_AutoTune/AC_AutoTune_Multi.h:193
     */
    void report_axis_gains(const char* axis_string, float rate_P, float rate_I,
                           float rate_D, float angle_P, float max_accel) const;

    //
    // Parameters (persisted via AP_Param)
    //
    
    /**
     * @brief Axis enable bitmask for selective tuning
     * 
     * @details Bitmask controlling which axes to tune:
     * - Bit 0 (0x01): Roll axis
     * - Bit 1 (0x02): Pitch axis
     * - Bit 2 (0x04): Yaw axis
     * 
     * Typical values:
     * - 0x07 (7): Tune all three axes (roll+pitch+yaw) [default]
     * - 0x03 (3): Tune roll and pitch only
     * - 0x01 (1): Tune roll only
     * 
     * Used by base class to determine tuning sequence. User can disable
     * specific axes if already well-tuned or problematic.
     * 
     * @note Parameter name: AUTOTUNE_AXES
     */
    AP_Int8  axis_bitmask;
    
    /**
     * @brief Target overshoot ratio for D gain tuning
     * 
     * @details Aggressiveness parameter controls target bounce-back (overshoot)
     *          during rate D tuning:
     * 
     * - Range: 0.0 to 0.1 (0% to 10% overshoot)
     * - Default: 0.05 (5% overshoot target)
     * - 0.0: No overshoot tolerated (very conservative, may result in sluggish response)
     * - 0.05: Balanced tuning with slight overshoot (recommended)
     * - 0.1: More aggressive tuning, higher overshoot acceptable (faster but less stable)
     * 
     * D gain is increased until bounce-back ratio ≈ aggressiveness value.
     * Higher values produce more aggressive tuning with potentially faster
     * response but higher oscillation risk.
     * 
     * @warning Values >0.1 can cause oscillation and instability
     * @note Parameter name: AUTOTUNE_AGGR
     * @note Critically affects D tuning behavior and final vehicle feel
     */
    AP_Float aggressiveness;
    
    /**
     * @brief Minimum allowed D gain floor
     * 
     * @details Minimum D gain limit prevents D from being reduced below safe level:
     * 
     * - Range: 0.001 to 0.006
     * - Default: 0.001
     * - Too low (<0.001): Noise sensitivity, jittery response
     * - Too high (>0.006): May prevent achieving clean response
     * 
     * Tuning fails if D reaches this minimum and clean response not achievable,
     * indicating vehicle requires mechanical fixes or different tuning approach.
     * 
     * @warning Values <0.001 can cause excessive noise sensitivity
     * @note Parameter name: AUTOTUNE_MIN_D
     * @note Failure at min_d suggests mechanical issues (imbalanced props, loose components)
     */
    AP_Float min_d;
    
    /**
     * @brief Flag to skip next test result
     * 
     * @details Used for rate overshoot handling during gain updates. When set,
     *          the next twitch result is ignored and flag cleared. Allows
     *          algorithm to skip transient results after large gain changes.
     * 
     * @note Internal state variable, not a parameter
     */
    bool     ignore_next;

    //
    // Test targets and measurement variables
    //
    
    /**
     * @brief Target angle for current angle twitch test (centidegrees)
     * 
     * @details Set by test_init() before angle twitch execution:
     * - Roll/Pitch: Typically ±2000 cd (±20 degrees)
     * - Yaw: Typically ±4500 cd (±45 degrees)
     * 
     * Target amplitude ensures sufficient response for measurement without
     * excessive excursion. Vehicle attempts to reach this angle during
     * ANGLE_P tuning tests.
     * 
     * @note Sign determines direction (+/- based on positive_direction flag)
     */
    float target_angle;
    
    /**
     * @brief Target rate for current rate twitch test (centidegrees/s)
     * 
     * @details Set by test_init() before rate twitch execution:
     * - Roll/Pitch: Typically ±18000 cds (±180 deg/s)
     * - Yaw: Typically ±9000 cds (±90 deg/s)
     * 
     * Target rate command applied during RATE_D and RATE_P tuning tests.
     * Peak response and bounce-back measured relative to this target.
     * 
     * @note Sign determines direction (+/- based on positive_direction flag)
     */
    float target_rate;
    
    /**
     * @brief Angle abort limit for test safety (centidegrees)
     * 
     * @details Maximum angle excursion before test aborts:
     * - Typical: ±9000 cd (±90 degrees)
     * 
     * Safety mechanism to prevent crashes from runaway response due to bad
     * gains. Test aborts and restores previous good gains if exceeded.
     * 
     * @warning Pilot must maintain altitude and position hold during twitches
     */
    float angle_abort;

    /**
     * @brief Minimum rate observed during test (centidegrees/s)
     * 
     * @details Tracks minimum (most negative) rate reached during twitch:
     * - For rate tests: Indicates bounce-back magnitude in reverse direction
     * - For angle tests: Used for rate monitoring and validation
     * 
     * Bounce-back ratio = abs(test_rate_min) / test_rate_max
     * 
     * Critical for D gain tuning - bounce-back indicates damping level.
     * 
     * @note Negative value indicates reverse direction from commanded rate
     */
    float test_rate_min;
    
    /**
     * @brief Maximum rate observed during test (centidegrees/s)
     * 
     * @details Tracks maximum (most positive) rate reached during twitch:
     * - For rate tests: Peak response in commanded direction
     * - For angle tests: Used for acceleration calculation
     * 
     * Compared to target_rate to determine if rate is achieved and whether
     * gain adjustments needed.
     */
    float test_rate_max;
    
    /**
     * @brief Minimum angle observed during test (centidegrees)
     * 
     * @details Tracks minimum angle excursion:
     * - For rate tests: Used for abort detection
     * - For angle tests: Indicates undershoot or reverse overshoot
     * 
     * Monitored for safety abort conditions.
     */
    float test_angle_min;
    
    /**
     * @brief Maximum angle observed during test (centidegrees)
     * 
     * @details Tracks maximum angle excursion:
     * - For rate tests: Used for abort detection
     * - For angle tests: Peak response, indicates overshoot
     * 
     * Compared to target_angle during ANGLE_P tuning. Overshoot
     * (test_angle_max > target_angle) triggers P reduction.
     */
    float test_angle_max;

    /**
     * @brief Maximum rate for acceleration calculation (centidegrees/s)
     * 
     * @details Tracks peak rate specifically for acceleration estimation.
     * Used by twitching_measure_acceleration() to calculate average
     * angular acceleration during twitch.
     * 
     * Acceleration = (rate - previous_rate) / dt
     * 
     * Result stored in max_accel parameter for attitude controller,
     * limiting rate of change to prevent abrupt maneuvers.
     * 
     * @note Used to populate AC_AttitudeControl max acceleration parameter
     */
    float accel_measure_rate_max;
};

#endif // AC_AUTOTUNE_ENABLED
