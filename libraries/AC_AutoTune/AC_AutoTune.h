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
  support for autotune of multirotors. Based on original autotune code from ArduCopter, written by Leonard Hall
  Converted to a library by Andrew Tridgell
 */

/**
 * @file AC_AutoTune.h
 * @brief Abstract base class for ArduPilot automated PID tuning system
 * 
 * @details This file defines the AC_AutoTune framework that provides automated in-flight PID gain tuning
 *          for ArduPilot vehicles. The core architecture consists of an abstract base class defining the state
 *          machine, gain management, safety mechanisms, and pure virtual methods for vehicle-specific test execution.
 *          
 *          High-Level State Machine:
 *          UNINITIALISED → TUNING (with sub-states: WAITING_FOR_LEVEL, EXECUTING_TEST, UPDATE_GAINS, ABORT) → FINISHED/FAILED/VALIDATING
 *          
 *          Tuning Sequence Concept:
 *          An ordered list of TuneType steps (e.g., RATE_D_UP → RATE_P_UP → ANGLE_P_UP) is executed for each axis.
 *          The specific sequence is vehicle-specific and defined by the subclass via set_tune_sequence().
 *          
 *          Safety Features:
 *          - Pilot override detection: Stick inputs beyond deadband suspend tuning
 *          - Level timeout: Vehicle must stabilize before test starts
 *          - Test abort limits: Excessive angles trigger immediate abort
 *          - Position hold: Optional GPS/non-GPS position hold maintains station during tuning
 *          - Parameter backup/restore: Original gains always preserved and restored on failure
 *          
 *          Gain Types:
 *          - ORIGINAL: Pre-tune gains from backup (safe known-good configuration)
 *          - TEST: Candidate gains under active test evaluation
 *          - INTRA_TEST: Conservative gains between tests (slower I-term buildup for stable return-to-level)
 *          - TUNED: Final gains from successful tuning (validated and ready for use)
 *          
 *          Integration:
 *          - AC_AttitudeControl: Attitude and rate control being tuned
 *          - AC_PosControl: Position hold during tuning
 *          - AP_AHRS: Attitude and rate feedback
 *          
 *          Based on original ArduCopter autotune by Leonard Hall, library conversion by Andrew Tridgell.
 * 
 * @author Leonard Hall (original algorithm)
 * @author Andrew Tridgell (library conversion)
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "AC_AutoTune_config.h"

#if AC_AUTOTUNE_ENABLED

#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AC_AutoTune_FreqResp.h"

/**
 * @brief Axis enable bitmask definitions for axis_bitmask parameter
 * @details These bitmask values are used to enable/disable tuning for specific axes:
 *          - AUTOTUNE_AXIS_BITMASK_ROLL = 1 (bit 0): Enable roll axis tuning
 *          - AUTOTUNE_AXIS_BITMASK_PITCH = 2 (bit 1): Enable pitch axis tuning
 *          - AUTOTUNE_AXIS_BITMASK_YAW = 4 (bit 2): Enable yaw axis tuning
 *          - AUTOTUNE_AXIS_BITMASK_YAW_D = 8 (bit 3): Enable yaw D tuning separately (helicopter only)
 * @note Combine with bitwise OR to enable multiple axes (e.g., 0x7 = roll+pitch+yaw)
 */
#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4
#define AUTOTUNE_AXIS_BITMASK_YAW_D           8

/**
 * @brief Number of consecutive successful test iterations required before freezing gains
 * @details Counter threshold for gain convergence. When gains produce acceptable response for SUCCESS_COUNT
 *          consecutive tests, those gains are accepted as final for current TuneType. Prevents premature
 *          convergence from single lucky test. Value 4 provides good balance between convergence time and confidence.
 */
#define AUTOTUNE_SUCCESS_COUNT                4     // The number of successful iterations we need to freeze at current gains

/**
 * @brief MAVLink message IDs for autotune status reporting to GCS
 * @details Used by update_gcs() to send STATUSTEXT messages:
 *          - STARTED = 0: Autotune initiated
 *          - STOPPED = 1: Autotune stopped by pilot
 *          - SUCCESS = 2: Axis tuning completed successfully
 *          - FAILED = 3: Axis tuning failed
 *          - SAVED_GAINS = 4: Tuned gains saved to EEPROM
 *          - TESTING = 5: User testing tuned gains
 *          - TESTING_END = 6: User finished testing tuned gains
 */
// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4
#define AUTOTUNE_MESSAGE_TESTING 5
#define AUTOTUNE_MESSAGE_TESTING_END 6

/**
 * @brief Minimum time between GCS status announcements
 * @details Rate limit for do_gcs_announcements() to prevent flooding GCS with status messages.
 *          2000ms = 2 seconds between updates provides adequate status visibility without excessive telemetry traffic.
 */
#define AUTOTUNE_ANNOUNCE_INTERVAL_MS 2000

/**
 * @class AC_AutoTune
 * @brief Abstract base class for automated PID gain tuning
 * 
 * @details Comprehensive class architecture explanation:
 *          
 *          Pure Virtual Base Class:
 *          This class defines the autotune framework. Vehicle subclasses (AC_AutoTune_Multi, AC_AutoTune_Heli)
 *          provide test execution implementation.
 *          
 *          State Machine Architecture:
 *          - mode (TuneMode): High-level state (UNINITIALISED, TUNING, FINISHED, FAILED, VALIDATING)
 *          - step (Step): Sub-state during TUNING (WAITING_FOR_LEVEL, EXECUTING_TEST, UPDATE_GAINS, ABORT)
 *          - tune_type (TuneType): Specific gain being tuned (RATE_D_UP, RATE_P_UP, ANGLE_P_UP, etc.)
 *          
 *          Multi-Axis Sequential Tuning:
 *          Each axis (roll, pitch, yaw) is tuned independently in sequence defined by axis_bitmask.
 *          
 *          Tuning Sequence:
 *          Ordered array tune_seq[] of TuneType steps executed for each axis. Sequence is vehicle-specific
 *          and defined by subclass via set_tune_sequence().
 *          
 *          Gain Management:
 *          - Backup original gains at start
 *          - Apply test gains during test
 *          - Apply intra-test gains between tests
 *          - Apply tuned gains on completion
 *          
 *          Safety Framework:
 *          - Pilot override suspend: Stick inputs beyond deadband pause tuning
 *          - Level timeout: Vehicle must stabilize before test starts
 *          - Test abort limits: Excessive angles trigger immediate abort
 *          - Position hold: Optional altitude+horizontal position hold during tuning
 *          - Automatic restoration: Safe gains restored on failure
 *          
 *          Position Hold:
 *          Optional altitude+horizontal position hold during tuning using AC_PosControl.
 *          Enables hands-off tuning for improved test consistency.
 *          
 *          Integration Hooks:
 *          Vehicle code calls run() at ~400Hz, handles mode entry/exit, disarm events.
 *          
 *          Parameter Persistence:
 *          Tuned gains saved to EEPROM on disarm if testing_switch_used flag is set.
 *          
 *          GCS Communication:
 *          Periodic status announcements, test results, final gains reporting.
 *          
 * @note Thread safety: Not thread-safe, must be called only from main loop
 * @note Execution rate: run() must be called at >=100Hz, preferably ~400Hz for accurate test response measurement
 * @note Position hold: Optional but highly recommended for hands-off tuning
 * @note Each axis: Tuned independently in sequence
 * @note Tuning session: Typically 15-30 minutes for all three axes
 * @note Success_counter: 4 consecutive good tests required for convergence
 * @note Pilot override: Instantly suspends tuning, resumes when pilot releases sticks
 * @note Testing_switch_used: Flag enables saving tuned gains on disarm
 * 
 * @warning Not thread-safe - must be called only from main loop
 * @warning Requires adequate altitude (>5m recommended) and clear airspace
 * @warning Pilot must monitor vehicle during tuning and be ready to abort
 * @warning Bad original gains can cause tuning failure - ensure flyable before autotune
 * @warning Position hold requires good EKF position estimate (GPS or non-GPS nav)
 * @warning Test abort limits are last line of defense - pilot must still monitor
 * @warning Never run autotune in tight spaces or near obstacles
 * @warning Wind gusts during test can affect results - calm conditions recommended
 * 
 * @see AC_AutoTune_Multi - Multicopter implementation
 * @see AC_AutoTune_Heli - Helicopter implementation
 * @see AC_AttitudeControl - Attitude controller being tuned
 * @see AC_PosControl - Position hold during tuning
 * @see AP_AHRS_View - Attitude and rate feedback
 * 
 * Integration: Requires AC_AttitudeControl for rate/angle control, AC_PosControl for position hold,
 *             AP_AHRS_View for attitude/rate feedback
 */
class AC_AutoTune
{
public:
    /**
     * @brief Constructor for autotune base class
     * @details Initializes state machine to UNINITIALISED, clears axis completion bitmask, nulls pointers.
     *          Subclass constructor must call init_internals() during mode entry to complete initialization.
     */
    AC_AutoTune();

    /**
     * @brief Main autotune update loop - must be called at >=100Hz
     * @details Core state machine execution. Handles all TuneMode states:
     *          - UNINITIALISED: No-op, waiting for init()
     *          - TUNING: Executes control_attitude() which runs Step state machine
     *          - FINISHED/FAILED: Maintains original gains, no active tuning
     *          - VALIDATING: User testing tuned gains, no active tuning
     *          
     *          Called every scheduler loop iteration from vehicle mode code. Must be called at >=100Hz for
     *          reliable performance, preferably ~400Hz for accurate test response measurement and safety checks.
     * 
     * @note Vehicle subclass can override for additional functionality but must call AC_AutoTune::run()
     * @warning Inconsistent call rate causes incorrect test response measurement and timing errors
     */
    // Main update loop for Autotune mode. Handles all states: tuning, testing, or idle.
    // Should be called at >=100Hz for reliable performance.
    virtual void run();

    /**
     * @brief Handle vehicle disarm event - possibly save tuned gains
     * @param in_autotune_mode true if vehicle is still in autotune mode when disarmed
     * @details Called by vehicle code when disarming. If testing_switch_used is true and tuned gains are
     *          currently active, calls save_tuning_gains() to persist gains to EEPROM. Allows user to
     *          flight-test tuned gains via AUX switch and save if acceptable by disarming.
     * @note Only saves if user explicitly tested tuned gains via AUX switch (testing_switch_used flag)
     */
    // Possibly save gains, called on disarm
    void disarmed(const bool in_autotune_mode);

    /**
     * @brief Stop autotune and revert to original gains
     * @details Abort tuning sequence, restore original gains via load_orig_gains(), set mode to FINISHED.
     *          Called when pilot exits autotune mode or triggers autotune stop via AUX switch.
     *          Safe to call at any time - will not corrupt state.
     * @note Original gains always restored, even if tuning was progressing well
     */
    // stop tune, reverting gains
    void stop();

    /**
     * @brief Handle autotune AUX switch toggle
     * @param ch_flag RC channel switch position (LOW/MIDDLE/HIGH)
     * @details Processes RC_OPTION for autotune auxiliary function. Switch positions:
     *          - LOW: Load original (pre-tune) gains, clear testing_switch_used
     *          - MIDDLE: No-op (neutral position)
     *          - HIGH: Load tuned gains (if available), set testing_switch_used = true
     *          
     *          Allows user to compare original vs tuned gains in flight by toggling switch.
     *          Setting testing_switch_used enables saving tuned gains on disarm.
     * 
     * @note Can be called anytime, not just during autotune mode
     * @warning Switching gains during aggressive maneuvers can cause control transients
     */
    // Autotune aux function trigger
    void do_aux_function(const RC_Channel::AuxSwitchPos ch_flag);

protected:

    /**
     * @brief Pure virtual: Save tuned gains to EEPROM
     * @details Vehicle subclass must implement to persist tuned gains via AP_Param::save().
     *          Called from disarmed() if testing_switch_used and tuned gains active.
     * @note Implementation should save all tuned axes' gains
     */
    virtual void save_tuning_gains() = 0;


    // reset Autotune so that gains are not saved again and autotune can be run again.
    void reset() {
        mode = TuneMode::UNINITIALISED;
        axes_completed = 0;
        testing_switch_used = false;
    }

    // axis that can be tuned
    enum class AxisType {
        ROLL = 0,                 // roll axis is being tuned (either angle or rate)
        PITCH = 1,                // pitch axis is being tuned (either angle or rate)
        YAW = 2,                  // yaw axis is being tuned using FLTE (either angle or rate)
        YAW_D = 3,                // yaw axis is being tuned using D (either angle or rate)
    };

    //
    // methods that must be supplied by the vehicle specific subclass
    //
    /**
     * @brief Pure virtual: Initialize autotune for mode entry
     * @return true if initialization successful, false on failure
     * @details Vehicle subclass must implement mode entry initialization: call init_internals(), set initial
     *          state, check pre-conditions (armed, position OK, etc.). Called when entering autotune flight mode.
     * @warning Must call init_internals() before any autotune operations
     */
    virtual bool init(void) = 0;

    /**
     * @brief Pure virtual: Get pilot's desired climb rate
     * @return Desired climb rate in cm/s (positive = up, negative = down)
     * @details Vehicle subclass reads pilot throttle/climb input. Used by position hold during tuning to
     *          maintain altitude while allowing pilot altitude adjustments.
     */
    // get pilot input for desired climb rate
    virtual float get_pilot_desired_climb_rate_cms(void) const = 0;

    /**
     * @brief Pure virtual: Get pilot's desired roll, pitch, and yaw rate
     * @param roll_rad [out] Desired roll angle in radians
     * @param pitch_rad [out] Desired pitch angle in radians
     * @param yaw_rate_rads [out] Desired yaw rate in radians/second
     * @details Vehicle subclass reads pilot stick inputs. Used to detect pilot override (inputs beyond deadband
     *          suspend tuning) and for position hold during tuning.
     */
    // get pilot input for designed roll and pitch, and yaw rate
    virtual void get_pilot_desired_rp_yrate_rad(float &roll_rad, float &pitch_rad, float &yaw_rate_rads) = 0;

    /**
     * @brief Pure virtual: Initialize position controller vertical velocity/accel limits
     * @details Vehicle subclass configures AC_PosControl altitude control limits appropriate for autotune.
     *          Conservative limits prevent excessive altitude excursions.
     */
    // init pos controller Z velocity and accel limits
    virtual void init_z_limits() = 0;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Pure virtual: Log PID data at full rate during test
     * @details Vehicle subclass logs rate PID telemetry (target, actual, P, I, D terms) for post-flight analysis.
     *          Only called during EXECUTING_TEST step when HAL_LOGGING_ENABLED.
     */
    // log PIDs at full rate for during test
    virtual void log_pids() = 0;
#endif

    //
    // methods to load and save gains
    //

    /**
     * @brief Virtual: Backup original gains and initialize tuning state
     * @details Default implementation backs up common gain variables. Vehicle subclass should override to backup
     *          vehicle-specific gains, then call AC_AutoTune::backup_gains_and_initialise(). Stores rate P/I/D/FF,
     *          angle P, accel limits for roll/pitch/yaw in orig_* variables.
     * @note Base implementation stores common gains, subclass extends for vehicle-specific gains
     */
    // backup original gains and prepare for start of tuning
    virtual void backup_gains_and_initialise();

    /**
     * @brief Pure virtual: Load original gains to attitude controller
     * @details Vehicle subclass applies pre-tune backup gains to AC_AttitudeControl PIDs.
     */
    // switch to use original gains
    virtual void load_orig_gains() = 0;

    /**
     * @brief Pure virtual: Load tuned gains to attitude controller
     * @details Vehicle subclass applies final result gains to AC_AttitudeControl PIDs.
     */
    // switch to gains found by last successful autotune
    virtual void load_tuned_gains() = 0;

    /**
     * @brief Pure virtual: Load intra-test gains to attitude controller
     * @details Vehicle subclass applies conservative between-test gains to AC_AttitudeControl PIDs.
     */
    // load gains used between tests. called during testing mode's update-gains step to set gains ahead of return-to-level step
    virtual void load_intra_test_gains() = 0;

    /**
     * @brief Pure virtual: Load test gains to attitude controller
     * @details Vehicle subclass applies candidate gains under evaluation to AC_AttitudeControl PIDs.
     */
    // load gains for next test.  relies on axis variable being set
    virtual void load_test_gains() = 0;

    /**
     * @brief Pure virtual: Reset test state variables
     * @details Vehicle subclass clears vehicle-specific test state before new test. Prevents stale data from
     *          affecting current operation.
     */
    // reset the test variables for each vehicle
    virtual void reset_vehicle_test_variables() = 0;

    /**
     * @brief Pure virtual: Reset update gain state variables
     * @details Vehicle subclass clears vehicle-specific gain update state before new gain update step.
     *          Prevents stale data from affecting current operation.
     */
    // reset the update gain variables for each vehicle
    virtual void reset_update_gain_variables() = 0;

    /**
     * @brief Pure virtual: Initialize state for new test
     * @details Vehicle subclass prepares for test execution: reset measurement variables, configure targets,
     *          set abort thresholds. Called before each test_run() sequence.
     */
    // test initialization and run methods that should be overridden for each vehicle
    virtual void test_init() = 0;
    
    /**
     * @brief Pure virtual: Execute one test iteration
     * @param test_axis Axis being tested (ROLL, PITCH, YAW, YAW_D)
     * @param dir_sign Test direction (+1.0 or -1.0)
     * @details Vehicle subclass implements test execution: generate excitation signal (step/chirp), apply to
     *          attitude controller, measure response, detect completion. Called at ~400Hz during EXECUTING_TEST
     *          step until test completes or times out.
     */
    virtual void test_run(AxisType test_axis, const float dir_sign) = 0;

    /**
     * @brief Check if roll axis is enabled for tuning
     * @return true if axis enabled in axis_bitmask parameter
     * @details Convenience accessor that checks bits in get_axis_bitmask(). Used by state machine to
     *          determine which axes to tune.
     */
    // return true if user has enabled autotune for roll, pitch or yaw axis
    bool roll_enabled() const;
    
    /**
     * @brief Check if pitch axis is enabled for tuning
     * @return true if axis enabled in axis_bitmask parameter
     * @details Convenience accessor that checks bits in get_axis_bitmask(). Used by state machine to
     *          determine which axes to tune.
     */
    bool pitch_enabled() const;
    
    /**
     * @brief Check if yaw axis is enabled for tuning
     * @return true if axis enabled in axis_bitmask parameter
     * @details Convenience accessor that checks bits in get_axis_bitmask(). Used by state machine to
     *          determine which axes to tune.
     */
    bool yaw_enabled() const;
    
    /**
     * @brief Check if yaw_d axis is enabled for tuning
     * @return true if axis enabled in axis_bitmask parameter
     * @details Convenience accessor that checks bits in get_axis_bitmask(). Used by state machine to
     *          determine which axes to tune.
     */
    bool yaw_d_enabled() const;

    /**
     * @brief Pure virtual: Update rate P based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for RATE_P_UP TuneType. Analyzes test
     *          response (peak, overshoot, settling), calculates new gains based on measured characteristics,
     *          stores in tune_* variables. Called during UPDATE_GAINS step.
     */
    // update gains for the rate p up tune type
    virtual void updating_rate_p_up_all(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Update rate D up based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for RATE_D_UP TuneType. Analyzes test
     *          response, calculates new gains, stores in tune_* variables. Called during UPDATE_GAINS step.
     */
    // update gains for the rate d up tune type
    virtual void updating_rate_d_up_all(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Update rate D down based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for RATE_D_DOWN TuneType. Analyzes test
     *          response, calculates new gains, stores in tune_* variables. Called during UPDATE_GAINS step.
     */
    // update gains for the rate d down tune type
    virtual void updating_rate_d_down_all(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Update angle P up based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for ANGLE_P_UP TuneType. Analyzes test
     *          response, calculates new gains, stores in tune_* variables. Called during UPDATE_GAINS step.
     */
    // update gains for the angle p up tune type
    virtual void updating_angle_p_up_all(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Update angle P down based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for ANGLE_P_DOWN TuneType. Analyzes test
     *          response, calculates new gains, stores in tune_* variables. Called during UPDATE_GAINS step.
     */
    // update gains for the angle p down tune type
    virtual void updating_angle_p_down_all(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Set final tuned gains with safety backoff
     * @param test_axis Axis being tuned
     * @details Vehicle subclass applies safety factor to tuned gains before finalizing. Called after successful
     *          convergence to ensure stable margins.
     */
    // set gains post tune for the tune type
    virtual void set_tuning_gains_with_backoff(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Reverse the direction of the next test
     * @return true if direction reversed, false if unable
     * @details Vehicle subclass toggles test direction flag. Used when test in one direction fails or for
     *          bidirectional testing requirements.
     */
    // reverse the direction of the next test
    virtual bool reverse_test_direction() = 0;


#if HAL_LOGGING_ENABLED
    virtual void Log_AutoTune() = 0;
    virtual void Log_AutoTuneDetails() = 0;
    virtual void Log_AutoTuneSweep() = 0;
#endif

    // internal init function, should be called from init()
    bool init_internals(bool use_poshold,
                        AC_AttitudeControl *attitude_control,
                        AC_PosControl *pos_control,
                        AP_AHRS_View *ahrs_view);

    // send intermittent updates to user on status of tune
    virtual void do_gcs_announcements() = 0;

    // send post test updates to user
    virtual void do_post_test_gcs_announcements() = 0;

    // send message with high level status (e.g. Started, Stopped)
    void update_gcs(uint8_t message_id) const;

    // send lower level step status (e.g. Pilot overrides Active)
    void send_step_string();

    // convert tune type to string for reporting
    const char *get_tune_type_name() const;

    // return current axis string
    const char *get_axis_name() const;

    // report final gains for a given axis to GCS
    virtual void report_final_gains(AxisType test_axis) const = 0;

    // Functions added for heli autotune

    /**
     * @brief Pure virtual: Update rate FF up based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for RATE_FF_UP TuneType. Analyzes test
     *          response, calculates new feedforward gains, stores in tune_* variables. Called during UPDATE_GAINS step.
     * @note Helicopter-specific tuning type
     */
    // Add additional updating gain functions specific to heli
    // generic method used by subclasses to update gains for the rate ff up tune type
    virtual void updating_rate_ff_up_all(AxisType test_axis)=0;

    /**
     * @brief Pure virtual: Update max gains based on test results
     * @param test_axis Axis being tuned
     * @details Vehicle subclass implements gain calculation algorithm for MAX_GAINS TuneType. Determines maximum
     *          stable gains through frequency sweep or other testing. Called during UPDATE_GAINS step.
     * @note Helicopter-specific tuning type
     */
    // generic method used by subclasses to update gains for the max gain tune type
    virtual void updating_max_gains_all(AxisType test_axis)=0;

    /**
     * @enum Step
     * @brief Sub-states within TUNING mode
     * @details State machine steps during active tuning:
     *          - WAITING_FOR_LEVEL = 0: Vehicle must stabilize at level attitude before test. Prevents starting
     *            test while oscillating or in aggressive maneuver. Timeout if not level within limit.
     *          - EXECUTING_TEST = 1: Test excitation active, monitoring response. State machine calls test_run()
     *            at ~400Hz. Completes when test duration elapsed or response settled.
     *          - UPDATE_GAINS = 2: Analyzing test results, calculating new gains. Calls appropriate updating_*_all()
     *            method based on tune_type. Increments success_counter if good response, else adjusts gains and repeats.
     *          - ABORT = 3: Test safety limit exceeded or pilot override. Restore safe gains (original or previous good),
     *            return to WAITING_FOR_LEVEL. Prevents crashes from bad gains.
     * 
     * @note Transitions: WAITING_FOR_LEVEL → EXECUTING_TEST → UPDATE_GAINS → (success: next TuneType, retry: WAITING_FOR_LEVEL, fail: ABORT)
     */
    // steps performed while in the tuning mode
    enum class Step {
        WAITING_FOR_LEVEL   = 0,    // Waiting for the vehicle to stabilize at level before starting a test.
        EXECUTING_TEST      = 1,    // Performing a test and monitoring the vehicle's response.
        UPDATE_GAINS        = 2,    // Updating gains based on test results.
        ABORT               = 3     // Aborting the current test; revert to safe gains and return to WAITING_FOR_LEVEL.
    };
    Step step;              // see StepType for what steps are performed

    /**
     * @enum TuneType
     * @brief Specific gain tuning operations in tune sequence
     * @details Individual gain tuning steps executed for each axis:
     *          - RATE_D_UP = 0: Increase rate D gain (damping)
     *          - RATE_D_DOWN = 1: Decrease rate D gain
     *          - RATE_P_UP = 2: Increase rate P gain (proportional)
     *          - RATE_FF_UP = 3: Increase rate feedforward gain (helicopter only)
     *          - ANGLE_P_DOWN = 4: Decrease angle P gain
     *          - ANGLE_P_UP = 5: Increase angle P gain
     *          - MAX_GAINS = 6: Determine maximum stable gains (helicopter only)
     *          - TUNE_CHECK = 7: Validate tuned gains with frequency sweep (helicopter only)
     *          - TUNE_COMPLETE = 8: Marker for end of sequence
     * 
     * @note Actual sequence vehicle-specific, defined by set_tune_sequence()
     */
    // mini steps performed while in Tuning mode, Testing step
    enum class TuneType {
        RATE_D_UP = 0,      // rate D is being tuned up
        RATE_D_DOWN = 1,    // rate D is being tuned down
        RATE_P_UP = 2,      // rate P is being tuned up
        RATE_FF_UP = 3,     // rate FF is being tuned up
        ANGLE_P_DOWN = 4,   // angle P is being tuned down
        ANGLE_P_UP = 5,     // angle P is being tuned up
        MAX_GAINS = 6,      // max allowable stable gains are determined
        TUNE_CHECK = 7,     // frequency sweep with tuned gains
        TUNE_COMPLETE = 8   // Reached end of tuning
    };
    TuneType tune_type;     // see TuneType
    TuneType tune_seq[6];   // holds sequence of tune_types to be performed
    uint8_t tune_seq_index; // current tune sequence step

    // get the next tune type
    void next_tune_type(TuneType &curr_tune_type, bool reset);

    // Sets customizable tune sequence for the vehicle
    virtual void set_tune_sequence() = 0;

    // get_axis_bitmask accessor
    virtual uint8_t get_axis_bitmask() const = 0;

    // get_testing_step_timeout_ms accessor
    virtual uint32_t get_testing_step_timeout_ms() const = 0;

    // get attitude for slow position hold in autotune mode
    void get_poshold_attitude_rad(float &roll_rad, float &pitch_rad, float &yaw_rad);

    /**
     * @enum GainType
     * @brief Types of gain sets that can be loaded
     * @details Gain set identifiers for load_gains():
     *          - ORIGINAL = 0: Pre-tune gains from backup, safe known-good configuration
     *          - TEST = 1: Candidate gains under active test evaluation
     *          - INTRA_TEST = 2: Conservative gains between tests, slower I-term buildup for stable return-to-level
     *          - TUNED = 3: Final gains from successful tuning, validated and ready for use
     * 
     * @note State machine switches between these as needed for safety and testing
     */
    // type of gains to load
    enum class GainType {
        ORIGINAL   = 0, // Gains as configured before autotune started
        TEST       = 1, // Gains applied during an active test
        INTRA_TEST = 2, // Gains applied between tests to maintain safe control while returning to level, with slower I-term buildup
        TUNED      = 3, // Gains discovered by the autotune process, used for flight testing or final use
    } loaded_gains;
    void load_gains(enum GainType gain_type);

    /**
     * @enum TuneMode
     * @brief High-level autotune operational mode
     * @details Top-level state machine states:
     *          - UNINITIALISED = 0: Autotune not started, init() not called
     *          - TUNING = 1: Active tuning in progress, executing Step state machine
     *          - FINISHED = 2: Tuning complete, original gains restored, can re-enter TUNING
     *          - FAILED = 3: Tuning failed (timeout, bad gains, etc.), original gains active
     *          - VALIDATING = 4: User flight-testing tuned gains via AUX switch, no active tuning
     * 
     * @note Transitions: UNINITIALISED → TUNING → (FINISHED/FAILED), can toggle VALIDATING via AUX switch
     */
    // TuneMode defines the high-level state of the autotune process.
    enum class TuneMode {
        UNINITIALISED = 0,  // Autotune has not yet been started.
        TUNING = 1,         // Autotune is actively running and tuning PID gains.
        FINISHED = 2,       // Tuning is complete, original (pre-tune) gains are restored.
        FAILED = 3,         // Tuning failed, vehicle is flying with original gains.
        VALIDATING = 4,     // Tuning complete, user is flight testing the newly tuned gains.
    };
    TuneMode mode;                       // see TuneMode for what modes are allowed

    // Object pointers for integration with vehicle subsystems
    AC_AttitudeControl *attitude_control;   // Attitude controller being tuned
    AC_PosControl *pos_control;             // Position controller for position hold during tuning
    AP_AHRS_View *ahrs_view;                // Attitude/rate feedback source
    AP_Motors *motors;                      // Motor controller for vehicle-specific limits

    // Current tuning state variables
    AxisType axis;                  // Current axis being tuned (ROLL, PITCH, YAW, YAW_D)
    bool     positive_direction;    // Test direction: false = negative (left for roll), true = positive (right for roll)
    bool     angle_step_commanded;  // true on first test iteration (signals attitude/rate target step required)
    uint8_t  axes_completed;        // Bitmask of successfully completed axes
    
    // Timing variables (all in milliseconds)
    uint32_t step_start_time_ms;    // Start time of current tuning step for timeout checks
    uint32_t step_timeout_ms;       // Time limit for current autotune process
    uint32_t level_start_time_ms;   // Start time of WAITING_FOR_LEVEL step
    
    // Test progress tracking
    int8_t   success_counter;       // Counter for consecutive successful tests (convergence tracking)
    float    start_angle;           // Test start angle in radians
    float    start_rate;            // Test start rate in radians/second
    float    test_accel_max_cdss;   // Maximum acceleration measured during test in centideg/s/s
    float    step_scaler;           // Scaler to reduce maximum target step

    LowPassFilterFloat  rotation_rate_filt; // Low-pass filtered rotation rate in radians/second

    // Backup of original (pre-tune) parameter values for safe restoration on abort/stop
    // Roll axis original gains
    float    orig_roll_rp;          // Original roll rate P gain
    float    orig_roll_ri;          // Original roll rate I gain
    float    orig_roll_rd;          // Original roll rate D gain
    float    orig_roll_rff;         // Original roll rate feedforward gain
    float    orig_roll_dff;         // Original roll D-term feedforward gain
    float    orig_roll_fltt;        // Original roll rate filter target in Hz
    float    orig_roll_smax;        // Original roll slew rate max in deg/s
    float    orig_roll_sp;          // Original roll angle P gain
    float    orig_roll_accel;       // Original roll angular acceleration limit in centideg/s/s
    float    orig_roll_rate;        // Original roll rate limit in deg/s
    
    // Pitch axis original gains
    float    orig_pitch_rp;         // Original pitch rate P gain
    float    orig_pitch_ri;         // Original pitch rate I gain
    float    orig_pitch_rd;         // Original pitch rate D gain
    float    orig_pitch_rff;        // Original pitch rate feedforward gain
    float    orig_pitch_dff;        // Original pitch D-term feedforward gain
    float    orig_pitch_fltt;       // Original pitch rate filter target in Hz
    float    orig_pitch_smax;       // Original pitch slew rate max in deg/s
    float    orig_pitch_sp;         // Original pitch angle P gain
    float    orig_pitch_accel;      // Original pitch angular acceleration limit in centideg/s/s
    float    orig_pitch_rate;       // Original pitch rate limit in deg/s
    
    // Yaw axis original gains
    float    orig_yaw_rp;           // Original yaw rate P gain
    float    orig_yaw_ri;           // Original yaw rate I gain
    float    orig_yaw_rd;           // Original yaw rate D gain
    float    orig_yaw_rff;          // Original yaw rate feedforward gain
    float    orig_yaw_dff;          // Original yaw D-term feedforward gain
    float    orig_yaw_fltt;         // Original yaw rate filter target in Hz
    float    orig_yaw_smax;         // Original yaw slew rate max in deg/s
    float    orig_yaw_rLPF;         // Original yaw rate low-pass filter frequency in Hz
    float    orig_yaw_sp;           // Original yaw angle P gain
    float    orig_yaw_accel;        // Original yaw angular acceleration limit in centideg/s/s
    float    orig_yaw_rate;         // Original yaw rate limit in deg/s
    
    bool     orig_bf_feedforward;   // Original body-frame feedforward enable flag

    // Tuned parameter values discovered by autotune process
    // Roll axis tuned gains
    float    tune_roll_rp;          // Tuned roll rate P gain
    float    tune_roll_rd;          // Tuned roll rate D gain
    float    tune_roll_sp;          // Tuned roll angle P gain
    float    tune_roll_accel;       // Tuned roll angular acceleration limit in centideg/s/s
    float    tune_roll_rff;         // Tuned roll rate feedforward gain (helicopter)
    
    // Pitch axis tuned gains
    float    tune_pitch_rp;         // Tuned pitch rate P gain
    float    tune_pitch_rd;         // Tuned pitch rate D gain
    float    tune_pitch_sp;         // Tuned pitch angle P gain
    float    tune_pitch_accel;      // Tuned pitch angular acceleration limit in centideg/s/s
    float    tune_pitch_rff;        // Tuned pitch rate feedforward gain (helicopter)
    
    // Yaw axis tuned gains
    float    tune_yaw_rp;           // Tuned yaw rate P gain
    float    tune_yaw_rd;           // Tuned yaw rate D gain (helicopter)
    float    tune_yaw_rLPF;         // Tuned yaw rate low-pass filter frequency in Hz
    float    tune_yaw_sp;           // Tuned yaw angle P gain
    float    tune_yaw_accel;        // Tuned yaw angular acceleration limit in centideg/s/s
    float    tune_yaw_rff;          // Tuned yaw rate feedforward gain (helicopter)

    // GCS communication rate limiting
    uint32_t last_announce_ms;      // Last GCS announcement timestamp in milliseconds (for rate limiting)
    
    // Attitude and rate feedback variables
    float   lean_angle;             // Current lean angle magnitude in radians
    float   rotation_rate;          // Current rotation rate in radians/second
    float   desired_roll_rad;       // Desired roll angle in radians (body frame)
    float   desired_pitch_rad;      // Desired pitch angle in radians (body frame)
    float   desired_yaw_rad;        // Desired yaw angle in radians (body frame)

    // Helicopter-specific frequency sweep variables
    float    start_freq;            // Start frequency for dwell test in Hz
    float    stop_freq;             // Ending frequency for dwell test in Hz

private:
    /**
     * @brief Virtual: Check if position estimate is adequate for position hold
     * @return true if EKF position estimate good enough for autotune
     * @details Default implementation checks EKF health flags. Vehicle subclass can override for stricter requirements.
     *          Position hold uses horizontal position estimate to maintain station during tuning.
     */
    // return true if we have a good position estimate
    virtual bool position_ok();

    /**
     * @brief Pure virtual: Get maximum target angle for roll/pitch test
     * @return Maximum target angle in centidegrees (positive)
     * @details Vehicle subclass defines test amplitude limit for roll and pitch axes.
     *          Determines test excursion magnitude. Typical values: 1000-2000 centidegrees (10-20 degrees).
     */
    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_max_rp_cd() const = 0;

    /**
     * @brief Pure virtual: Get maximum target angle for yaw test
     * @return Maximum target angle in centidegrees (positive)
     * @details Vehicle subclass defines test amplitude limit for yaw axis.
     *          Determines yaw test excursion magnitude. Typical values: 2000-4000 centidegrees (20-40 degrees).
     */
    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_max_y_cd() const = 0;

    /**
     * @brief Pure virtual: Get minimum target angle for roll/pitch test
     * @return Minimum target angle in centidegrees (negative)
     * @details Vehicle subclass defines test amplitude limit for negative roll and pitch.
     *          Typically negative of target_angle_max_rp_cd() for symmetric tests.
     */
    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_min_rp_cd() const = 0;

    /**
     * @brief Pure virtual: Get minimum target angle for yaw test
     * @return Minimum target angle in centidegrees (negative)
     * @details Vehicle subclass defines test amplitude limit for negative yaw.
     *          Typically negative of target_angle_max_y_cd() for symmetric tests.
     */
    // methods subclasses must implement to specify max/min test angles:
    virtual float target_angle_min_y_cd() const = 0;

    /**
     * @brief Pure virtual: Get maximum abort angle limit for roll/pitch
     * @return Maximum abort angle in centidegrees (positive)
     * @details Vehicle subclass defines safety abort limit for roll and pitch axes.
     *          Test aborted if angle exceeds this limit. Last line of defense against crashes.
     *          Typical values: 3000-4500 centidegrees (30-45 degrees).
     * @warning Must be larger than target_angle_max_rp_cd() to allow for overshoot
     */
    // methods subclasses must implement to specify max/min test angles:
    virtual float angle_lim_max_rp_cd() const = 0;

    /**
     * @brief Pure virtual: Get negative abort angle limit for all axes
     * @return Negative abort angle in centidegrees (negative)
     * @details Vehicle subclass defines safety abort limit for negative excursions on all axes.
     *          Test aborted if angle exceeds this limit. Typically negative of angle_lim_max_rp_cd().
     * @warning Must be more negative than target_angle_min values to allow for overshoot
     */
    // methods subclasses must implement to specify max/min test angles:
    virtual float angle_lim_neg_rpy_cd() const = 0;

    /**
     * @brief Initialize position controller for autotune
     * @return true if initialization successful
     * @details Configures AC_PosControl for position hold: sets velocity/accel limits via init_z_limits(),
     *          enables position hold mode, initializes position target to current location.
     */
    // initialise position controller
    bool init_position_controller();

    /**
     * @brief Main tuning state machine implementation
     * @details Implements Step state machine: WAITING_FOR_LEVEL checks currently_level() with timeout,
     *          EXECUTING_TEST calls test_run() and monitors completion/abort, UPDATE_GAINS calls appropriate
     *          updating_*_all() method and evaluates convergence, ABORT restores safe gains.
     *          Also handles position hold attitude generation via get_poshold_attitude_rad(), pilot override
     *          detection, test timeouts.
     * @note Core of autotune framework, orchestrates entire tuning process
     */
    // Main tuning state machine. Handles WAITING_FOR_LEVEL, EXECUTING_TEST, UPDATE_GAINS, ABORT.
    // Updates attitude controller targets and evaluates test responses to tune gains.
    void control_attitude();

    /**
     * @brief Check if vehicle attitude and rates are level (near zero)
     * @return true if roll/pitch angles < threshold and rates < threshold
     * @details Level check for WAITING_FOR_LEVEL step. Thresholds: angles < 250-500 centidegrees (2.5-5 deg),
     *          rates < 500-1000 centideg/s depending on vehicle. Must remain level for
     *          AUTOTUNE_REQUIRED_LEVEL_TIME_MS (250ms) before test starts.
     */
    // returns true if vehicle is close to level
    bool currently_level();

    // Pilot override and position hold state
    bool     pilot_override;             // true = pilot is overriding controls, suspend tuning temporarily
    bool     use_poshold;                // true = position hold enabled during tuning
    bool     have_position;              // true = start_position is valid
    Vector3f start_position;             // Position hold target as offset from EKF origin in cm, NEU frame

    // Timing variables for pilot override tracking
    uint32_t override_time;              // Last time pilot overrode controls in milliseconds

    // GCS warning rate limiting
    uint32_t last_pilot_override_warning; // Last pilot override warning timestamp in milliseconds

    // Tuned gain testing and saving state
    bool testing_switch_used;            // true = user tested tuned gains via AUX switch, enables save on disarm

};

#endif  // AC_AUTOTUNE_ENABLED
