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
 * @file AC_AutoTune_Heli.h
 * @brief Helicopter-specific autotune implementation using frequency response analysis
 * 
 * @details This file implements helicopter autotune specialization that uses chirp/sweep or 
 *          dwell excitation for system identification. The frequency response methodology 
 *          injects sinusoidal or chirp signals, measures gain and phase response across 
 *          frequency range, and determines PID gains based on frequency response characteristics.
 * 
 *          Multi-stage tuning sequence:
 *          1. MAX_GAINS determination via gain margin testing to find instability boundaries
 *          2. RATE_FF tuning for accurate tracking (gain = 1.0 at DC)
 *          3. RATE_P tuning to specified gain margin below max allowable gain
 *          4. RATE_D tuning for minimum response gain (maximum damping)
 *          5. ANGLE_P tuning for specified control bandwidth
 * 
 *          Integration components:
 *          - AC_AutoTune_FreqResp: Measurement engine for frequency response calculations
 *          - Chirp class (AP_Math/chirp.h): Signal generation for sweep tests
 *          - Helicopter-specific gain calculation heuristics for safe, stable tuning
 * 
 *          Test methodology:
 *          - Sweep test: Frequency-varying chirp over 23 seconds for rapid broadband identification
 *          - Dwell test: Fixed-frequency sinusoid (5-10 cycles) for high-precision measurements
 * 
 *          Safety mechanisms:
 *          - Settle time before test initiation for vehicle stabilization
 *          - Attitude limits enforced during test execution
 *          - Automatic abort on pilot override
 *          - Parameter backup/restore on tuning start/completion/failure
 * 
 *          Coordinate system: Body frame rates (rad/s), angles in centidegrees for parameters
 * 
 * @note Called from main loop at ~400Hz. Not thread-safe for concurrent access.
 * @warning Requires adequate altitude, clear airspace, and pilot monitoring during tests
 * 
 * @see AC_AutoTune Base class with tuning state machine
 * @see AC_AutoTune_FreqResp Frequency response measurement engine
 * @see Chirp Chirp signal generation (AP_Math/chirp.h)
 * @see AC_AttitudeControl_Heli Helicopter attitude controller
 * @see AP_Motors_Heli Helicopter motor mixer
 * 
 * Source: libraries/AC_AutoTune/AC_AutoTune_Heli.h:1-313
 */

#pragma once

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include "AC_AutoTune.h"
#include <AP_Math/chirp.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_Scheduler/AP_Scheduler.h>

/**
 * @class AC_AutoTune_Heli
 * @brief Helicopter autotune using frequency response system identification
 * 
 * @details This class provides helicopter-specific tuning algorithms using frequency response
 *          analysis for system identification. It inherits from AC_AutoTune base class and
 *          implements helicopter-specific measurement and gain calculation methods.
 * 
 *          Architecture and operation:
 *          - Inherits from AC_AutoTune base class, provides helicopter-specific tuning algorithms
 *          - Uses frequency response analysis: inject known input signal (chirp or dwell),
 *            measure output response, calculate gain/phase at each frequency
 *          - Two measurement paths for comprehensive system characterization:
 *            * MOTOR path: Motor mixer input to output (open-loop system response)
 *            * TARGET path: Target input to output (closed-loop system response)
 *          - Multi-axis tuning: Roll, pitch, yaw tested independently with axis_bitmask parameter
 *          - Tuning sequence controlled by seq_bitmask parameter (RFF, RP, RD, SP, MAX_GAINS)
 * 
 *          Test methodology:
 *          - Sweep test: Frequency-varying chirp with 23 seconds fixed duration for rapid
 *            broadband identification across entire frequency range
 *          - Dwell test: Fixed-frequency sinusoid (5-10 cycles per frequency) for high-precision
 *            measurements at specific frequencies
 * 
 *          Safety mechanisms:
 *          - Settle time before test ensures vehicle is stable
 *          - Attitude limits enforced during test to prevent dangerous attitudes
 *          - Abort on pilot override for immediate manual control recovery
 *          - Parameter backup/restore on tuning start/completion/failure
 * 
 *          Progress reporting:
 *          - Real-time progress reporting via MAVLink STATUSTEXT messages
 *          - GCS messages provide test status, current frequency, estimated completion
 * 
 *          Logging support (when HAL_LOGGING_ENABLED):
 *          - ATUN: Summary log with test results (axis, frequency, gains calculated)
 *          - ATDE: Time history log with detailed test execution data at ~400Hz
 *          - ATSW: Sweep data log with frequency response across sweep range
 * 
 *          Coordinate system: Body frame rates in rad/s, angles in centidegrees for parameters
 * 
 * @note Thread safety: Called from main loop, not thread-safe for concurrent access
 * @note Loop rate: Designed for ~400Hz main loop execution
 * @warning MAX_GAINS test can excite instability - pilot must be ready to abort
 * @warning Requires stable hover before test initiation
 * 
 * @see AC_AutoTune Base class with tuning state machine
 * @see AC_AttitudeControl_Heli Helicopter attitude control interface
 * @see AP_Motors_Heli Helicopter motor mixer
 * @see AC_AutoTune_FreqResp Frequency response measurement engine
 * @see AC_AutoTune_Multi Multicopter autotune (alternative approach)
 */
class AC_AutoTune_Heli : public AC_AutoTune
{
public:
    /**
     * @brief Constructor for helicopter autotune
     * 
     * @details Initializes base class, sets default parameters, allocates frequency response
     *          objects (freqresp_mtr, freqresp_tgt), and initializes chirp generator. 
     *          Registers parameter group via var_info[] for AP_Param persistence.
     * 
     * @note Frequency response objects allocated for both MOTOR and TARGET measurement paths
     */
    AC_AutoTune_Heli();

    /**
     * @brief Save tuned gains to EEPROM on disarm
     * 
     * @details Persists tuned gains via AP_Param::save() when vehicle disarms after successful
     *          autotune. Only saves if tuning succeeded and tuned gains are currently active.
     *          Stores rate P/I/D/FF, angle P, and acceleration limits for all tuned axes.
     * 
     * @note Called automatically on disarm by AC_AutoTune base class
     * @note Parameters saved: rate P, rate I, rate D, rate FF, angle P, max accel for each axis
     */
    void save_tuning_gains() override;

    /**
     * @brief Parameter group information for AP_Param registration
     * 
     * @details Defines helicopter-specific autotune parameters for EEPROM persistence:
     *          - AXES (axis_bitmask): Bitmask of axes to tune (bit 0=roll, 1=pitch, 2=yaw)
     *          - SEQ (seq_bitmask): Tuning sequence bitmask (RFF, RP, RD, SP, MAX_GAINS)
     *          - FRQ_MIN (min_sweep_freq): Minimum sweep frequency (Hz)
     *          - FRQ_MAX (max_sweep_freq): Maximum sweep frequency (Hz)
     *          - GN_MAX (max_resp_gain): Maximum response gain threshold
     *          - VELXY_P (vel_hold_gain): Velocity hold gain for position stabilization
     *          - ACC_MAX (accel_max): Maximum angular acceleration (centideg/s/s)
     *          - RATE_MAX (rate_max): Maximum angular rate (deg/s)
     * 
     *          Used by AP_Param::setup_object_defaults() for parameter initialization and
     *          EEPROM persistence across power cycles.
     * 
     * @note Static member required for AP_Param parameter system integration
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    //
    // methods to load and save gains
    //

    /**
     * @brief Backup original gains and initialize tuning state
     * 
     * @details Stores current PID gains (rate P/I/D/FF, angle P, accel limits) for all axes
     *          in orig_* variables before tuning starts. Allows restoration if tuning fails
     *          or user aborts. Also initializes tuning state machine variables to prepare
     *          for test execution.
     * 
     *          Gains backed up for each axis:
     *          - Rate P, I, D, FF gains
     *          - Angle P gain
     *          - Maximum angular acceleration limit
     *          - Rate filter frequencies (target and error)
     *          - Slew rate maximum
     *          - Maximum rate limit
     * 
     * @warning Must be called before starting autotune to ensure original gains can be restored
     * @note Called automatically by base class state machine when entering WAITING state
     */
    void backup_gains_and_initialise() override;

    /**
     * @brief Load specified gain set to attitude controller
     * 
     * @param s_axis Axis (ROLL, PITCH, or YAW) to load gains for
     * @param rate_p Rate P gain (dimensionless or 1/s depending on controller)
     * @param rate_i Rate I gain (1/s)
     * @param rate_d Rate D gain (s)
     * @param rate_ff Rate feedforward gain (dimensionless)
     * @param angle_p Angle P gain (1/s)
     * @param max_accel Maximum angular acceleration limit (centideg/s/s)
     * @param rate_fltt Rate target filter frequency (Hz)
     * @param rate_flte Rate error filter frequency (Hz)
     * @param smax Slew rate maximum (deg/s)
     * @param max_rate Maximum angular rate limit (deg/s)
     * 
     * @details Applies specified gain set to AC_AttitudeControl rate and angle PIDs for the
     *          given axis. Used to switch between original, test, intra-test, and tuned gains
     *          during different phases of tuning sequence.
     * 
     * @note Modifies live attitude controller gains - affects flight behavior immediately
     */
    void load_gain_set(AxisType s_axis, float rate_p, float rate_i, float rate_d, float rate_ff, float angle_p, float max_accel, float rate_fltt, float rate_flte, float smax, float max_rate);

    /**
     * @brief Load original gains to attitude controller
     * 
     * @details Restores gains that were backed up before autotune started. Used when:
     *          - User aborts autotune
     *          - Tuning fails and needs to revert
     *          - Switching back from tuned gains to original
     * 
     * @note Restores all axes to pre-tuning state
     * @see backup_gains_and_initialise()
     */
    void load_orig_gains() override;

    /**
     * @brief Load tuned gains to attitude controller
     * 
     * @details Applies gains calculated from successful autotune tests. Called when:
     *          - Autotune completes successfully
     *          - User enables ATC_RAT_*_FLTT parameter (tuned gains)
     *          - Switching from original to tuned gains
     * 
     * @note Only valid after successful tuning completion
     * @see save_tuning_gains()
     */
    void load_tuned_gains() override;

    /**
     * @brief Load intra-test gains to attitude controller
     * 
     * @details Applies gains used during return-to-level step between tests. These gains
     *          provide stable control while returning vehicle to test start attitude before
     *          next test begins. Called during testing mode's update-gains step.
     * 
     * @note Typically uses conservative gains for stability during transitions
     */
    void load_intra_test_gains() override;

    /**
     * @brief Load test gains to attitude controller
     * 
     * @details Applies gains used during active test excitation. These may differ from
     *          normal gains to optimize test conditions and prevent interference with
     *          frequency response measurements.
     * 
     * @note Active only during EXECUTING_TEST state
     */
    void load_test_gains() override;

    /**
     * @brief Reset test variables for new test
     * 
     * @details Clears test state variables before starting new test to prevent stale data
     *          from affecting current test results. Resets:
     *          - Test timing counters
     *          - Measurement accumulators
     *          - Filter states
     *          - Cycle completion flags
     * 
     * @note Called by base class before each test execution
     */
    void reset_vehicle_test_variables() override;

    /**
     * @brief Reset update gain variables for new gain calculation
     * 
     * @details Clears gain update state variables before starting new update-gains step.
     *          Prevents stale data from previous gain calculations affecting current step.
     *          Resets frequency search state, peak detection flags, and previous gain values.
     * 
     * @note Called during UPDATE_GAINS state before gain calculation begins
     */
    void reset_update_gain_variables() override;

    /**
     * @brief Initialize test state for new axis test
     * 
     * @details Prepares state machine for test execution on specified axis:
     *          - Resets test counters and timing variables
     *          - Initializes filters (command, target rate, gyro)
     *          - Configures freqresp_mtr and freqresp_tgt objects
     *          - Sets up chirp or dwell parameters based on current tune type
     *          - Establishes test frequency range
     *          - Configures measurement calculation type (RATE/ANGLE/DRB)
     * 
     * @note Must complete successfully before test_run() is called
     * @note Called once per test at start of EXECUTING_TEST state
     */
    void test_init() override;

    /**
     * @brief Execute one iteration of autotune test
     * 
     * @param test_axis Axis being tested (ROLL, PITCH, or YAW)
     * @param dir_sign Direction of test excitation (+1 or -1 for bidirectional tests)
     * 
     * @details Core test execution function called at ~400Hz during EXECUTING_TEST state.
     *          Each iteration:
     *          1. Generates excitation signal (chirp or dwell) via chirp_input or sine wave
     *          2. Applies excitation to attitude controller as rate or angle command
     *          3. Reads response from AHRS (attitude) and gyros (rates)
     *          4. Passes input/output data to freqresp_mtr and freqresp_tgt via update()
     *          5. Checks for cycle completion (dwell) or time completion (sweep)
     *          6. Stores results when test completes
     * 
     *          Test types:
     *          - Sweep: Single 23-second chirp sweeping frequency range
     *          - Dwell: 5-10 cycles at fixed frequency for precise measurement
     * 
     * @warning Must be called consistently at ~400Hz for correct frequency response measurement
     * @warning Timing jitter degrades frequency response measurement accuracy
     * 
     * @note Updates freqresp objects with new data each call
     * @note Test completes when cycle_complete_tgt and cycle_complete_mtr both true
     */
    void test_run(AxisType test_axis, const float dir_sign) override;

    /**
     * @brief Update rate P gain based on max gain test results
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Calculates rate P gain that achieves specified gain margin below maximum
     *          allowable gain determined from max_gains test. Gain margin prevents instability
     *          by keeping operating point safely below instability boundary.
     * 
     *          Algorithm:
     *          1. Uses dwell tests at multiple frequencies
     *          2. Measures frequency response gain at each frequency
     *          3. Interpolates to find P gain that produces desired response gain at crossover
     *          4. Applies gain margin (typical: 6dB = factor of 2) for safety
     * 
     * @note Called during UPDATE_GAINS state after rate P dwell tests complete
     * @note Typical gain margin: 6dB (factor of 2 below max gain)
     * 
     * @see updating_max_gains_all() for max gain determination
     */
    void updating_rate_p_up_all(AxisType test_axis) override;

    /**
     * @brief Update rate D gain to minimize response gain (maximize damping)
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Searches for rate D gain that minimizes frequency response gain, indicating
     *          maximum damping and best disturbance rejection. 
     * 
     *          Algorithm:
     *          1. Uses dwell tests sweeping through frequency range
     *          2. Finds D gain where response gain is minimum (typically at notch frequency)
     *          3. Balances damping improvement vs noise amplification at high frequencies
     * 
     *          Physical interpretation:
     *          - Minimum gain point indicates optimal damping
     *          - Too much D causes noise amplification at high frequencies
     *          - Too little D allows oscillations and overshoot
     * 
     * @note Called during UPDATE_GAINS state after rate D dwell tests complete
     * @warning Excessive D gain amplifies sensor noise and can cause instability
     */
    void updating_rate_d_up_all(AxisType test_axis) override;

    /**
     * @brief Update rate D gain downward (not used for helicopters)
     * 
     * @param test_axis Axis being tuned
     * 
     * @details Empty implementation - helicopters use updating_rate_d_up_all() only.
     *          Rate D down tuning not applicable to helicopter frequency response method.
     * 
     * @note Required override but not used in helicopter autotune
     */
    void updating_rate_d_down_all(AxisType test_axis) override {};

    /**
     * @brief Update rate feedforward to achieve target tracking
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Adjusts rate FF gain until measured rate matches target rate (gain = 1.0 at DC).
     *          Feedforward improves tracking performance for rate commands by anticipating
     *          required control effort.
     * 
     *          Algorithm:
     *          1. Uses low-frequency dwell test (<5Hz) where phase lag is minimal
     *          2. Measures DC gain (rate_out / rate_cmd)
     *          3. Calculates FF = 1.0 / measured_DC_gain
     *          4. Iterates if necessary to achieve unity gain
     * 
     *          Physical meaning: FF=1.0 means 1 deg/s command produces 1 deg/s response at DC
     * 
     * @note Called during UPDATE_GAINS state after rate FF dwell tests complete
     * @note Typically first gain tuned in sequence after max_gains determination
     */
    void updating_rate_ff_up_all(AxisType test_axis) override;

    /**
     * @brief Update angle P gain for specified bandwidth
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Calculates angle P gain that achieves desired attitude control bandwidth.
     *          Bandwidth is frequency where closed-loop response gain drops to -3dB (0.707).
     * 
     *          Algorithm:
     *          1. Uses dwell tests measuring closed-loop attitude response
     *          2. Searches for P gain that achieves target bandwidth (typical: 2-5 Hz)
     *          3. Higher angle P = faster attitude response but requires more aggressive rate control
     * 
     *          Trade-offs:
     *          - Higher angle P: Faster response, more aggressive, requires good rate controller
     *          - Lower angle P: Slower response, more stable, less demanding on rate controller
     * 
     * @note Called during UPDATE_GAINS state after angle P dwell tests complete
     * @note Typically last gain tuned in sequence
     */
    void updating_angle_p_up_all(AxisType test_axis) override;

    /**
     * @brief Update angle P gain downward (not used for helicopters)
     * 
     * @param test_axis Axis being tuned
     * 
     * @details Empty implementation - helicopters use updating_angle_p_up_all() only.
     *          Angle P down tuning not applicable to helicopter frequency response method.
     * 
     * @note Required override but not used in helicopter autotune
     */
    void updating_angle_p_down_all(AxisType test_axis) override {};

    /**
     * @brief Determine maximum allowable rate P and D gains via gain margin testing
     * 
     * @param test_axis Axis being tuned (ROLL, PITCH, or YAW)
     * 
     * @details Critical safety test that finds instability boundaries before tuning begins.
     * 
     *          Algorithm:
     *          1. Increases rate P (with D=0) until response gain indicates approaching instability
     *          2. Increases rate D (with P=0) similarly until instability threshold detected
     *          3. Maximum gains stored and used to calculate safe operating gains with margin
     * 
     *          Instability detection:
     *          - Response gain exceeding threshold (typically >20dB)
     *          - Phase approaching -180 degrees (stability boundary)
     *          - Maximum frequency reached without finding limit
     * 
     *          Uses dwell tests at increasing frequencies until:
     *          - Instability detected (high gain/unfavorable phase)
     *          - Maximum test frequency reached
     *          - Excessive oscillation detected
     * 
     * @warning This test can excite instability - pilot must be ready to abort immediately
     * @warning Test aborts automatically if excessive oscillation detected
     * @warning Requires adequate altitude and clear airspace
     * 
     * @note Always performed first in tuning sequence to establish safe gain limits
     * @note Maximum gains used by subsequent tests to ensure safety margins
     * 
     * @see updating_rate_p_up_all() uses max_rate_p results
     * @see updating_rate_d_up_all() uses max_rate_d results
     */
    void updating_max_gains_all(AxisType test_axis) override;

    /**
     * @brief Apply final tuned gains with specified backoff/margin
     * 
     * @param test_axis Axis for which to set final gains
     * 
     * @details Calculates final tuned gains by applying backoff factors to measured optimal
     *          gains for safety margin. Backoff provides robustness against:
     *          - Measurement uncertainty
     *          - Vehicle mass changes (payload, fuel)
     *          - Environmental variations (temperature, altitude)
     *          - Model uncertainties
     * 
     *          Typical backoff: 20-30% reduction from theoretical optimal gains
     * 
     *          Stores results in tune_* variables for application to attitude controller
     * 
     * @note Called after all tests complete for an axis
     * @note Backoff percentage configurable via parameters
     */
    void set_tuning_gains_with_backoff(AxisType test_axis) override;

    /**
     * @brief Determine if next test should reverse direction
     * 
     * @return true if next test should use opposite direction, false otherwise
     * 
     * @details Returns positive_direction flag to control test direction alternation.
     *          Helicopter autotune uses positive direction for frequency response tests.
     * 
     * @note Helicopter tests typically don't require bidirectional testing like multirotors
     */
    bool reverse_test_direction() override { return positive_direction; }

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log autotune summary data for current test
     * 
     * @details Writes ATUN log message with test results after each test completes:
     *          - Axis tested (roll/pitch/yaw)
     *          - Tune step type (MAX_GAINS, RATE_P, RATE_D, RATE_FF, ANGLE_P)
     *          - Test frequency (Hz)
     *          - Measured gain (dimensionless)
     *          - Measured phase (degrees)
     *          - Calculated gains (rate FF, rate P, rate D, angle P)
     * 
     *          Enables post-flight analysis of tuning process and verification of results.
     * 
     * @note Called after each test completes successfully
     * @note ATUN log messages viewable in MAVExplorer and other log analysis tools
     */
    void Log_AutoTune() override;
    
    /**
     * @brief Write ATUN log message
     * 
     * @param _axis Axis tested (ROLL, PITCH, or YAW)
     * @param tune_step Current tune type (MAX_GAINS, RATE_P, RATE_D, RATE_FF, ANGLE_P)
     * @param dwell_freq Test frequency in Hz
     * @param meas_gain Measured frequency response gain (dimensionless)
     * @param meas_phase Measured frequency response phase in degrees
     * @param new_gain_rff Calculated rate feedforward gain
     * @param new_gain_rp Calculated rate P gain
     * @param new_gain_rd Calculated rate D gain
     * @param new_gain_sp Calculated angle P gain
     * @param max_accel Maximum measured angular acceleration (centideg/s/s)
     * 
     * @details Formats and writes structured ATUN log entry for post-flight analysis.
     *          Provides complete record of test conditions and calculated gains.
     * 
     * @note Log format compatible with ArduPilot log analysis tools
     */
    void Log_Write_AutoTune(AxisType _axis, TuneType tune_step, float dwell_freq, float meas_gain, float meas_phase, float new_gain_rff, float new_gain_rp, float new_gain_rd, float new_gain_sp, float max_accel);

    /**
     * @brief Log autotune time history data during test execution
     * 
     * @details Writes ATDE log message with time-series data at ~400Hz during test:
     *          - Motor command (dimensionless mixer input)
     *          - Target rate (rad/s)
     *          - Measured rate from gyro (rad/s)
     *          - Target angle (rad)
     *          - Measured angle from AHRS (rad)
     * 
     *          Enables detailed analysis of test execution:
     *          - Verify excitation signal applied correctly
     *          - Check measurement quality and noise levels
     *          - Analyze transient response
     *          - Debug test issues
     * 
     * @note Called at ~400Hz during EXECUTING_TEST state
     * @note Generates large log files - consider storage capacity
     * @warning High-rate logging may impact performance on resource-constrained boards
     */
    void Log_AutoTuneDetails() override;
    
    /**
     * @brief Write ATDE log message
     * 
     * @param motor_cmd Motor mixer command in dimensionless units (-1 to +1)
     * @param tgt_rate_rads Target rate in rad/s
     * @param rate_rads Measured rate from gyro in rad/s
     * @param tgt_ang_rad Target angle in radians
     * @param ang_rad Measured angle from AHRS in radians
     * 
     * @details Writes detailed time history data for post-flight analysis of test execution.
     *          All angles and rates in body frame.
     * 
     * @note Coordinate system: Body frame (roll/pitch/yaw)
     * @note Units: rad/s for rates, radians for angles
     */
    void Log_Write_AutoTuneDetails(float motor_cmd, float tgt_rate_rads, float rate_rads, float tgt_ang_rad, float ang_rad);

    /**
     * @brief Log frequency sweep response data
     * 
     * @details Writes ATSW log message with sweep test results showing frequency response
     *          across entire sweep range. Records both MOTOR and TARGET input path responses:
     *          - Frequency points across sweep range (Hz)
     *          - Gain at each frequency (dimensionless)
     *          - Phase at each frequency (degrees)
     * 
     *          Enables visualization of complete frequency response:
     *          - Bode plots (gain and phase vs frequency)
     *          - Resonance identification
     *          - System dynamics characterization
     * 
     * @note Called during sweep tests after chirp completes
     * @note Provides data for Bode plot generation in log analysis
     */
    void Log_AutoTuneSweep() override;
    
    /**
     * @brief Write ATSW log message
     * 
     * @param freq_mtr Frequency for motor path measurement (Hz)
     * @param gain_mtr Gain for motor path (dimensionless)
     * @param phase_mtr Phase for motor path (degrees)
     * @param freq_tgt Frequency for target path measurement (Hz)
     * @param gain_tgt Gain for target path (dimensionless)
     * @param phase_tgt Phase for target path (degrees)
     * 
     * @details Writes frequency response data for both measurement paths.
     *          Motor path: Motor mixer input to output (open-loop)
     *          Target path: Target input to output (closed-loop)
     * 
     * @note Both paths measured simultaneously during sweep test
     * @note Frequency arrays may differ slightly between paths due to measurement timing
     */
    void Log_Write_AutoTuneSweep(float freq_mtr, float gain_mtr, float phase_mtr, float freq_tgt, float gain_tgt, float phase_tgt);
#endif

    /**
     * @brief Send periodic status updates to GCS during tuning
     * 
     * @details Transmits MAVLink STATUSTEXT messages with tuning progress information:
     *          - Current axis being tuned
     *          - Current tune step (MAX_GAINS, RATE_P, etc.)
     *          - Current test frequency
     *          - Estimated time to completion
     * 
     *          Rate-limited to avoid flooding GCS with messages (typical: every few seconds)
     *          Keeps operator informed during long test sequences.
     * 
     * @note Called periodically during test execution
     * @note Messages appear in GCS message window
     */
    void do_gcs_announcements() override;

    /**
     * @brief Send test completion message to GCS
     * 
     * @details Transmits test results to GCS after each test completes:
     *          - Measured gain and phase
     *          - Calculated gains (rate P/D/FF, angle P)
     *          - Test frequency and conditions
     * 
     *          Allows real-time monitoring of tuning progress and verification of results.
     * 
     * @note Called after each test completes successfully
     * @note Provides immediate feedback without requiring log download
     */
    void do_post_test_gcs_announcements() override;

    /**
     * @brief Report final tuned gains for axis to GCS
     * 
     * @param test_axis Axis to report (ROLL, PITCH, or YAW) - const to prevent modification
     * 
     * @details Transmits final tuned gains via STATUSTEXT when tuning completes for an axis:
     *          - Rate P gain
     *          - Rate I gain
     *          - Rate D gain
     *          - Rate FF gain
     *          - Angle P gain
     *          - Maximum acceleration limit
     * 
     *          Operator can compare final gains to original gains to assess tuning changes.
     * 
     * @note Called when tuning completes for specified axis
     * @note Gains reported in same units as parameters for easy comparison
     */
    void report_final_gains(AxisType test_axis) const override;

    /**
     * @brief Configure tuning sequence based on seq_bitmask parameter
     * 
     * @details Builds tune_seq[] array of TuneType enums based on SEQ parameter bitmask.
     *          Typical sequence: MAX_GAINS → RFF → RP → RD → SP → TUNE_COMPLETE
     * 
     *          Operator can disable specific tuning steps via parameter:
     *          - Bit 0: MAX_GAINS determination
     *          - Bit 1: RATE_FF tuning
     *          - Bit 2: RATE_P tuning
     *          - Bit 3: RATE_D tuning
     *          - Bit 4: ANGLE_P tuning
     * 
     * @note Called during tuning initialization
     * @note Skipping MAX_GAINS may be unsafe - requires manual gain limits
     */
    void set_tune_sequence() override;

    /**
     * @brief Get axes to be tuned
     * 
     * @return Bitmask of axes (bit 0=roll, bit 1=pitch, bit 2=yaw)
     * 
     * @details Returns axis_bitmask parameter value controlling which axes are tuned.
     *          Used by base class state machine to determine tuning scope.
     * 
     * @note Typical values: 0x07 (all axes), 0x03 (roll+pitch only)
     */
    uint8_t get_axis_bitmask() const override { return axis_bitmask; }

    /**
     * @brief Get timeout for testing step in state machine
     * 
     * @return Timeout in milliseconds (depends on test type)
     * 
     * @details Returns appropriate timeout for current test type:
     *          - Sweep test: ~25 seconds (23s chirp + settle time)
     *          - Dwell test: depends on frequency and cycle count (typically 5-30 seconds)
     *          - Longer timeouts for low-frequency dwells (more cycles needed)
     * 
     *          Prevents infinite loops if test fails to complete normally.
     * 
     * @note Called by base class state machine for watchdog timeout
     * @note Test aborts if timeout exceeded
     */
    uint32_t get_testing_step_timeout_ms() const override;

private:
    /**
     * @brief Container for frequency response test results
     * 
     * @details Stores measured frequency response data from a single test:
     *          - freq: Test frequency in Hz
     *          - gain: Measured gain (dimensionless, output amplitude / input amplitude)
     *          - phase: Measured phase in degrees (positive = lead, negative = lag)
     * 
     *          Used throughout tuning process to pass frequency response data between
     *          test execution and gain calculation functions.
     */
    struct sweep_info {
        float freq;   ///< Test frequency (Hz)
        float gain;   ///< Measured gain (dimensionless)
        float phase;  ///< Measured phase (degrees)
    };

    /**
     * @brief Container for maximum gain test results
     * 
     * @details Stores instability boundary information from max gains test:
     *          - freq: Frequency where maximum gain found (Hz)
     *          - phase: Phase at maximum gain frequency (degrees)
     *          - gain: Maximum measured gain before instability (dimensionless)
     *          - max_allowed: Maximum allowable gain with safety margin applied
     * 
     *          max_allowed is calculated with gain margin for safety:
     *          max_allowed = gain / gain_margin_factor
     *          (typical gain_margin_factor = 2.0 for 6dB margin)
     * 
     * @warning max_allowed must not be exceeded to maintain stability
     */
    struct max_gain_data {
        float freq;         ///< Frequency of max gain (Hz)
        float phase;        ///< Phase at max gain (degrees)
        float gain;         ///< Maximum measured gain (dimensionless)
        float max_allowed;  ///< Maximum allowable gain with margin (dimensionless)
    };

    /**
     * @brief Type of calculation for frequency response measurement
     * 
     * @details Specifies how frequency response is calculated from measurements:
     *          - RATE: Measure rate response directly from gyros
     *          - ANGLE: Measure angle response from AHRS, differentiate to get rate
     *          - DRB: Direct rate feedback (helicopter-specific measurement path)
     * 
     * @note Calculation type affects measurement noise and bandwidth
     */
    enum FreqRespCalcType {
        RATE    = 0,  ///< Direct rate measurement from gyros
        ANGLE   = 1,  ///< Angle measurement differentiated to rate
        DRB     = 2,  ///< Direct rate feedback (helicopter-specific)
    };

    /**
     * @brief Input signal source for frequency response measurement
     * 
     * @details Specifies which input path to measure for frequency response:
     *          - MOTOR: Measure from motor mixer input (open-loop system response)
     *          - TARGET: Measure from target input (closed-loop system response)
     * 
     *          Two paths measured to characterize complete control system:
     *          - MOTOR path: Shows plant dynamics without controller influence
     *          - TARGET path: Shows closed-loop response with controller active
     * 
     * @note Both paths measured simultaneously during tests using separate freqresp objects
     */
    enum FreqRespInput {
        MOTOR    = 0,  ///< Motor mixer input to output (open-loop)
        TARGET   = 1,  ///< Target input to output (closed-loop)
    };

    /**
     * @brief Get maximum target angle for roll/pitch during tests
     * 
     * @return Maximum target angle in centidegrees
     * 
     * @details Returns positive limit for target attitude during test execution.
     *          Prevents excessive attitudes that could endanger vehicle.
     * 
     * @note Used by base class to limit test excitation
     */
    float target_angle_max_rp_cd() const override;

    /**
     * @brief Get maximum target angle for yaw during tests
     * 
     * @return Maximum target yaw angle in centidegrees
     * 
     * @details Returns positive limit for target yaw during test execution.
     *          Yaw limits typically more generous than roll/pitch.
     * 
     * @note Yaw tests use rate control primarily, angle limits less critical
     */
    float target_angle_max_y_cd() const override;

    /**
     * @brief Get minimum target angle for roll/pitch during tests
     * 
     * @return Minimum target angle in centidegrees (negative)
     * 
     * @details Returns negative limit for target attitude during test execution.
     *          Symmetric with target_angle_max_rp_cd() for bidirectional tests.
     */
    float target_angle_min_rp_cd() const override;

    /**
     * @brief Get minimum target angle for yaw during tests
     * 
     * @return Minimum target yaw angle in centidegrees (negative)
     * 
     * @details Returns negative limit for target yaw during test execution.
     */
    float target_angle_min_y_cd() const override;

    /**
     * @brief Get maximum attitude limit for roll/pitch
     * 
     * @return Maximum attitude limit in centidegrees
     * 
     * @details Hard limit on vehicle attitude - test aborts if exceeded.
     *          Provides safety boundary beyond target limits.
     * 
     * @warning Test aborts immediately if this limit is exceeded
     */
    float angle_lim_max_rp_cd() const override;

    /**
     * @brief Get negative attitude limit for all axes
     * 
     * @return Minimum attitude limit in centidegrees (negative)
     * 
     * @details Hard limit on vehicle attitude - test aborts if exceeded.
     *          Applies to roll, pitch, and yaw.
     * 
     * @warning Test aborts immediately if this limit is exceeded
     */
    float angle_lim_neg_rpy_cd() const override;

    /**
     * @brief Initialize dwell test or angle dwell test variables
     * 
     * @param start_frq Starting frequency for dwell sweep (Hz)
     * @param stop_frq Stopping frequency for dwell sweep (Hz)
     * @param amplitude Excitation amplitude (dimensionless or deg depending on type)
     * @param filt_freq Filter cutoff frequency for measurements (Hz)
     * @param freq_resp_input Input path to measure (MOTOR or TARGET)
     * @param calc_type Calculation method (RATE, ANGLE, or DRB)
     * @param resp_type Response type for freqresp object
     * @param waveform_input_type Waveform type (DWELL for fixed frequency)
     * 
     * @details Configures dwell test parameters for frequency response measurement:
     *          - Sets up freqresp objects with test configuration
     *          - Initializes filters for command and measurement signals
     *          - Configures frequency range and step size
     *          - Sets number of cycles per frequency (5-10 typical)
     * 
     * @note Called from test_init() before each dwell test sequence
     */
    void dwell_test_init(float start_frq, float stop_frq, float amplitude, float filt_freq, FreqRespInput freq_resp_input, FreqRespCalcType calc_type, AC_AutoTune_FreqResp::ResponseType resp_type, AC_AutoTune_FreqResp::InputType waveform_input_type);

    /**
     * @brief Execute dwell test at current frequency
     * 
     * @param[out] test_data Measured frequency response data (freq, gain, phase)
     * 
     * @details Performs fixed-frequency dwell test until sufficient cycles complete:
     *          1. Generates sinusoidal excitation at test frequency
     *          2. Applies to attitude controller
     *          3. Measures response from sensors
     *          4. Passes data to freqresp objects
     *          5. Waits for cycle completion
     *          6. Extracts gain and phase results
     * 
     *          Test completes when both freqresp objects indicate sufficient cycles.
     * 
     * @note Called repeatedly from test_run() until test completes
     * @note Typical: 5-10 cycles per frequency for good measurement
     */
    void dwell_test_run(sweep_info &test_data);

    /**
     * @brief Adjust rate FF gain to achieve unity DC gain
     * 
     * @param[in,out] tune_ff Current rate FF gain, updated with new value
     * @param[in,out] test_data Test results, updated with latest measurement
     * @param[in,out] next_freq Next test frequency, updated for frequency sweep
     * 
     * @details Iteratively adjusts FF until measured DC gain equals 1.0:
     *          - Uses low-frequency dwell tests (<5Hz) where phase lag minimal
     *          - Calculates FF = 1.0 / measured_gain
     *          - Iterates if necessary to converge on unity gain
     *          - Ensures rate command produces equal rate response
     * 
     * @note Converges in 2-4 iterations typically
     */
    void updating_rate_ff_up(float &tune_ff, sweep_info &test_data, float &next_freq);

    /**
     * @brief Determine rate P gain within max gain limits
     * 
     * @param[in,out] tune_p Current rate P gain, updated with new value
     * @param[in,out] test_data Test results, updated with latest measurement
     * @param[in,out] next_freq Next test frequency, updated for frequency sweep
     * @param[in] max_gain_p Maximum gain data from max_gains test
     * 
     * @details Uses dwell tests at multiple frequencies to find P gain that:
     *          - Provides good disturbance rejection (high bandwidth)
     *          - Stays below max_gain_p.max_allowed for stability
     *          - Achieves specified gain margin (typically 6dB)
     * 
     *          Sweeps frequency, measures response gain, interpolates to find P value
     *          that produces desired response gain at crossover frequency.
     * 
     * @note Uses max_gain_p from updating_max_gains() test
     */
    void updating_rate_p_up(float &tune_p, sweep_info &test_data, float &next_freq, max_gain_data &max_gain_p);

    /**
     * @brief Determine rate D gain for minimum response gain
     * 
     * @param[in,out] tune_d Current rate D gain, updated with new value
     * @param[in,out] test_data Test results, updated with latest measurement
     * @param[in,out] next_freq Next test frequency, updated for frequency sweep
     * @param[in] max_gain_d Maximum gain data from max_gains test
     * 
     * @details Searches for D gain that minimizes frequency response gain:
     *          - Sweeps through frequency range with dwell tests
     *          - Finds D value where response gain is minimum (notch frequency)
     *          - Balances damping improvement vs noise amplification
     *          - Stays below max_gain_d.max_allowed for stability
     * 
     *          Minimum gain indicates maximum damping and best disturbance rejection.
     * 
     * @note Uses max_gain_d from updating_max_gains() test
     * @warning Too much D amplifies sensor noise at high frequencies
     */
    void updating_rate_d_up(float &tune_d, sweep_info &test_data, float &next_freq, max_gain_data &max_gain_d);

    /**
     * @brief Determine angle P gain for target bandwidth
     * 
     * @param[in,out] tune_p Current angle P gain, updated with new value
     * @param[in,out] test_data Test results, updated with latest measurement
     * @param[in,out] next_freq Next test frequency, updated for frequency sweep
     * 
     * @details Searches for angle P that achieves desired attitude control bandwidth:
     *          - Uses dwell tests measuring closed-loop attitude response
     *          - Finds P where response gain = -3dB (0.707) at target bandwidth
     *          - Higher P = faster response, more aggressive
     *          - Lower P = slower response, more stable
     * 
     *          Typical target bandwidth: 2-5 Hz for helicopters
     * 
     * @note Requires rate controller to be well-tuned first
     */
    void updating_angle_p_up(float &tune_p, sweep_info &test_data, float &next_freq);

    /**
     * @brief Determine maximum allowable P and D gains
     * 
     * @param[in,out] test_data Test results, updated with latest measurement
     * @param[in,out] next_freq Next test frequency, updated for frequency sweep
     * @param[out] max_gain_p Maximum rate P gain data with stability margin
     * @param[out] max_gain_d Maximum rate D gain data with stability margin
     * @param[in,out] tune_p Test P gain, increased to find instability
     * @param[in,out] tune_d Test D gain, increased to find instability
     * 
     * @details Critical safety test finding instability boundaries:
     *          1. Test rate P (D=0): Increase P until instability indicators appear
     *          2. Test rate D (P=0): Increase D until instability indicators appear
     *          3. Store maximum safe gains with margin applied
     * 
     *          Instability indicators:
     *          - Response gain exceeding threshold (>20dB typical)
     *          - Phase approaching -180° (Nyquist stability boundary)
     *          - Excessive oscillation amplitude
     * 
     * @warning Can excite dangerous oscillations - pilot must be ready to abort
     * @warning Requires adequate altitude and clear airspace
     * 
     * @note Always performed first in tuning sequence
     * @note Results used by all subsequent gain calculations
     */
    void updating_max_gains(sweep_info &test_data, float &next_freq, max_gain_data &max_gain_p, max_gain_data &max_gain_d, float &tune_p, float &tune_d);

    /**
     * @brief Search for frequency that produces desired phase
     * 
     * @param test Current test measurement data
     * @param desired_phase Target phase in degrees
     * @param freq_incr Frequency increment for search (Hz)
     * @param[out] est_data Estimated data at desired phase
     * @param[out] new_freq Frequency for next test
     * 
     * @return true if desired phase bounded (ready for interpolation), false if still searching
     * 
     * @details General frequency search strategy:
     *          1. Sweeps frequency to bracket desired phase between two measurements
     *          2. Once bracketed, interpolates to estimate frequency at desired phase
     *          3. Used for finding crossover frequency, resonance peaks, etc.
     * 
     * @note Interpolation assumes phase varies monotonically with frequency
     */
    bool freq_search_for_phase(sweep_info test, float desired_phase, float freq_incr, sweep_info &est_data, float &new_freq);

    /**
     * @brief Reset max gains update variables
     * 
     * @details Clears state variables before max_gains test:
     *          - found_max_p and found_max_d flags
     *          - Maximum gain data structures
     *          - Test gain values
     * 
     * @note Called before updating_max_gains() execution
     */
    void reset_maxgains_update_gain_variables();

    /**
     * @brief Reset sweep test variables
     * 
     * @details Clears sweep-specific state before sweep test:
     *          - Sweep progress tracking
     *          - Maximum gain tracking
     *          - Phase milestone flags (180°, 270°)
     *          - Sweep completion flag
     * 
     * @note Called before sweep test execution
     */
    void reset_sweep_variables();

    /**
     * @brief Check if frequency exceeds configured limits
     * 
     * @param frequency Frequency to check (Hz)
     * 
     * @return true if frequency outside [min_sweep_freq, max_sweep_freq], false if valid
     * 
     * @details Ensures test frequencies remain within safe, configured range:
     *          - Below min_sweep_freq: Insufficient excitation, DC effects dominate
     *          - Above max_sweep_freq: Sensor noise, aliasing, hardware limits
     * 
     *          Typical range: 0.5 Hz to 20 Hz for helicopters
     * 
     * @note Test aborts if frequency limits exceeded
     */
    bool exceeded_freq_range(float frequency);

    /**
     * @brief Format and report gains for axis to GCS
     * 
     * @param axis_string Axis name string ("Roll", "Pitch", "Yaw")
     * @param rate_P Rate P gain value
     * @param rate_I Rate I gain value
     * @param rate_D Rate D gain value
     * @param rate_ff Rate FF gain value
     * @param angle_P Angle P gain value
     * @param max_accel Maximum acceleration limit (centideg/s/s)
     * 
     * @details Helper function for consistent gain reporting format across all axes.
     *          Formats gains into human-readable STATUSTEXT message.
     * 
     * @note Used by report_final_gains() and intermediate reporting functions
     */
    void report_axis_gains(const char* axis_string, float rate_P, float rate_I, float rate_D, float rate_ff, float angle_P, float max_accel) const;

    // Input type selection: DWELL (fixed frequency) or SWEEP (chirp) for entire tuning session
    AC_AutoTune_FreqResp::InputType input_type;
    
    sweep_info curr_data;                           ///< Current frequency response test results (freq, gain, phase)
    float    next_test_freq;                        ///< Next test frequency for upcoming test cycle (Hz)

    max_gain_data max_rate_p;                       ///< Maximum rate P gain data from max_gains test with stability margin
    max_gain_data max_rate_d;                       ///< Maximum rate D gain data from max_gains test with stability margin

    // Updating max gain variables - track progress of max_gains test
    bool found_max_p;                               ///< Flag: true when maximum safe rate P gain found
    bool found_max_d;                               ///< Flag: true when maximum safe rate D gain found

    // Updating angle P up variables - track peak finding for angle P optimization
    float phase_max;                                ///< Maximum phase observed during angle P search (degrees)
    float freq_max;                                 ///< Frequency at maximum phase (Hz)
    float sp_prev_gain;                             ///< Previous gain measurement for angle P iteration
    bool found_max_gain_freq;                       ///< Flag: true when frequency of maximum gain identified
    bool found_peak;                                ///< Flag: true when peak of gain response found

    // Updating rate D up variables
    float rd_prev_gain;                             ///< Previous gain measurement for rate D iteration

    // Frequency search for phase variables
    sweep_info prev_test;                           ///< Data from previous dwell test for interpolation

    // Dwell Test configuration variables
    AC_AutoTune_FreqResp::InputType test_input_type; ///< Input type for current test (DWELL or SWEEP)
    FreqRespCalcType test_calc_type;                ///< Calculation type: RATE, ANGLE, or DRB
    FreqRespInput test_freq_resp_input;             ///< Input path to measure: MOTOR or TARGET
    uint8_t num_dwell_cycles;                       ///< Number of cycles per frequency for dwell test (5-10 typical)
    float test_start_freq;                          ///< Starting frequency for current dwell sequence (Hz)
    float tgt_attitude;                             ///< Target attitude setpoint for test (rad or deg depending on test)
    
    float    pre_calc_cycles;                       ///< Number of cycles to complete before frequency response calculations begin
    float    command_out;                           ///< Test axis command output (dimensionless, -1 to 1 range)
    float    filt_target_rate;                      ///< Filtered target rate from attitude controller (rad/s)
    float    dwell_start_time_ms;                   ///< Start time of dwell test in milliseconds (AP_HAL::millis())

    sweep_info curr_test;                           ///< Current test sweep info (composite of motor and target paths)
    sweep_info curr_test_mtr;                       ///< Current test results for motor input path
    sweep_info curr_test_tgt;                       ///< Current test results for target input path

    uint32_t settle_time;                           ///< Time in milliseconds to allow aircraft stabilization before test starts (typically 4000ms)

    // Filters for dwell test - remove DC bias and noise from measurements
    LowPassFilterVector2f filt_att_fdbk_from_velxy_cd; ///< Filtered attitude feedback from velocity hold (centidegrees, NED frame)
    LowPassFilterFloat filt_command_reading;        ///< Filtered command reading to keep oscillation centered around trim
    LowPassFilterFloat filt_gyro_reading;           ///< Filtered gyro reading to keep oscillation centered around trim (rad/s, body frame)
    LowPassFilterFloat filt_tgt_rate_reading;       ///< Filtered target rate reading to keep oscillation centered (rad/s)

    // Trim variables - store pre-test conditions for yaw axis
    float trim_yaw_tgt_reading_cd;                  ///< Trim target yaw before starting test (centidegrees)
    float trim_yaw_heading_reading_cd;              ///< Trim heading before starting test (centidegrees)

    LowPassFilterFloat  command_filt;               ///< Filtered command signal - removes noise for frequency response measurement
    LowPassFilterFloat  target_rate_filt;           ///< Filtered target rate - removes noise for frequency response measurement (rad/s)

    /**
     * @brief Container for sweep test progress tracking
     * 
     * @details Tracks key frequency response characteristics during sweep:
     *          - maxgain: Maximum gain observed and frequency
     *          - ph180: Frequency where phase crosses -180° (neutral stability)
     *          - ph270: Frequency where phase crosses -270°
     *          - progress: Phase milestone tracking (0=start, 1=reached -180°, 2=reached -270°)
     */
    struct sweep_data {
        sweep_info maxgain;                         ///< Maximum gain sweep info (freq, gain, phase)
        sweep_info ph180;                           ///< -180° phase crossing sweep info
        sweep_info ph270;                           ///< -270° phase crossing sweep info
        uint8_t  progress;                          ///< Progress flag: 0=start, 1=reached -180°, 2=reached -270°
    };
    sweep_data sweep_mtr;                           ///< Sweep progress tracking for motor input path
    sweep_data sweep_tgt;                           ///< Sweep progress tracking for target input path
    bool sweep_complete;                            ///< Flag: true when sweep test completes

    const float sweep_time_ms = 23000;              ///< Fixed sweep duration in milliseconds (23 seconds for frequency range coverage)

    // Autotune parameters exposed via AP_Param for ground station configuration
    AP_Int8  axis_bitmask;                          ///< Bitmask of axes to tune: bit0=roll, bit1=pitch, bit2=yaw
    AP_Int8  seq_bitmask;                           ///< Tuning sequence bitmask: enables/disables specific tune types
    AP_Float min_sweep_freq;                        ///< Minimum sweep/dwell frequency (Hz, typical: 0.5)
    AP_Float max_sweep_freq;                        ///< Maximum sweep/dwell frequency (Hz, typical: 20)
    AP_Float max_resp_gain;                         ///< Maximum allowable response gain before instability (dimensionless, typical: 5-10)
    AP_Float vel_hold_gain;                         ///< Velocity hold gain for maintaining position during test (dimensionless)
    AP_Float accel_max;                             ///< Maximum angular acceleration during test (centideg/s/s)
    AP_Float rate_max;                              ///< Maximum angular rate during test (deg/s)

    // Frequency response measurement objects - two paths for complete system characterization
    AC_AutoTune_FreqResp freqresp_mtr;              ///< Frequency response: motor mixer input to output (open-loop plant)
    AC_AutoTune_FreqResp freqresp_tgt;              ///< Frequency response: target input to output (closed-loop system)

    // Cycle completion tracking for frequency response measurements
    bool cycle_complete_tgt;                        ///< Flag: true when freqresp_tgt completes sufficient cycles
    bool cycle_complete_mtr;                        ///< Flag: true when freqresp_mtr completes sufficient cycles

    Chirp chirp_input;                              ///< Chirp signal generator for sweep tests (frequency-varying sinusoid)
};

#endif  // AC_AUTOTUNE_ENABLED
