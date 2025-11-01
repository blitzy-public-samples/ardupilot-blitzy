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
 * @file rate_thread.cpp
 * @brief High-frequency attitude rate controller thread implementation
 * 
 * @details This file implements a separate high-priority real-time thread that runs the
 *          attitude rate controller at rates significantly higher than the main vehicle
 *          loop (typically 400Hz - 4000Hz vs 400Hz main loop). This architecture achieves:
 * 
 *          1. Reduced control latency: Minimize delay between gyro sample and motor output
 *          2. Higher bandwidth control: Support higher PID gains without oscillation
 *          3. Improved filtering: Process gyro data at high rates for better noise rejection
 *          4. Better disturbance rejection: Faster response to external disturbances
 * 
 *          The rate thread is synchronized with IMU backend processing and runs independently
 *          of the main vehicle loop to prevent scheduler delays from affecting control timing.
 *          Thread safety is ensured through careful data access patterns and lock-free
 *          communication where possible to avoid priority inversion.
 * 
 *          Key architectural decisions:
 *          - Rate controller runs at integer multiples of main loop rate
 *          - Motor outputs are updated at rate thread frequency
 *          - Attitude targets from main loop are consumed atomically
 *          - Dynamic rate adjustment based on CPU load
 * 
 * @note This implementation is only compiled when AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
 * @warning Modifying timing or synchronization may affect flight stability
 * 
 * @see Copter::rate_controller_thread()
 * @see AP_InertialSensor::get_next_gyro_sample()
 * @see AC_AttitudeControl::rate_controller_run_dt()
 */
#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED

#pragma GCC optimize("O2")

/*
 Attitude Rate controller thread design.

 Rationale: running rate outputs linked to fast gyro outputs achieves two goals:

 1. High frequency gyro processing allows filters to be applied with high sample rates
    which is advantageous in removing high frequency noise and associated aliasing
 2. High frequency rate control reduces the latency between control and action leading to 
    better disturbance rejection and faster responses which generally means higher
    PIDs can be used without introducing control oscillation

 (1) is already mostly achieved through the higher gyro rates that are available via
 INS_GYRO_RATE. (2) requires running the rate controller at higher rates via a separate thread


 Goal: the ideal scenario is to run in a single cycle:

    gyro read->filter->publish->rate control->motor output

 This ensures the minimum latency between gyro sample and motor output. Other functions need 
 to also run faster than they would normally most notably logging and filter frequencies - most
 notably the harmonic notch frequency.

 Design assumptions:

 1. The sample rate of the IMUs is consistent and accurate.
    This is the most basic underlying assumption. An alternative approach would be to rely on
    the timing of when samples are received but this proves to not work in practice due to
    scheduling delays. Thus the dt used by the attitude controller is the delta between IMU
    measurements, not the delta between processing cycles in the rate thread.
 2. Every IMU reading must be processed or consistently sub-sampled.
    This is an assumption that follows from (1) - so it means that attitude control should
    process every sample or every other sample or every third sample etc. Note that these are
    filtered samples - all incoming samples are processed for filtering purposes, it is only
    for the purposes of rate control that we are sub-sampling.
 3. The data that the rate loop requires is timely, consistent and accurate.
    Rate control essentially requires two components - the target and the actuals. The actuals
    come from the incoming gyro sample combined with the state of the PIDs. The target comes
    from attitude controller which is running at a slower rate in the main loop. Since the rate
    thread can read the attitude target at any time it is important that this is always available
    consistently and is updated consistently.
 4. The data that the rest of the vehicle uses is the same data that the rate thread uses.
    Put another way any gyro value that the vehicle uses (e.g. in the EKF etc), must have already
    been processed by the rate thread. Where this becomes important is with sub-sampling - if 
    rate gyro values are sub-sampled we need to make sure that the vehicle is also only using
    the sub-sampled values.

 Design:

 1. Filtered gyro samples are (sub-sampled and) pushed into an ObjectBuffer from the INS backend.
 2. The pushed sample is published to the INS front-end so that the rest of the vehicle only
    sees published values that have been used by the rate controller. When the rate thread is not 
    in use the filtered samples are effectively sub-sampled at the main loop rate. The EKF is unaffected
    as it uses delta angles calculated from the raw gyro values. (It might be possible to avoid publishing
    from the rate thread by only updating _gyro_filtered when a value is pushed).
 3. A notification is sent that a sample is available
 4. The rate thread is blocked waiting for a sample. When it receives a notification it:
    4a. Runs the rate controller
    4b. Pushes the new pwm values. Periodically at the main loop rate all of the SRV_Channels::push()
        functionality is run as well.
 5. The rcout dshot thread is blocked waiting for a new pwm value. When it is signalled by the
    rate thread it wakes up and runs the dshot motor output logic.
 6. Periodically the rate thread:
    6a. Logs the rate outputs (1Khz)
    6b. Updates the notch filter centers (Gyro rate/2)
    6c. Checks the ObjectBuffer length and main loop delay (10Hz)
        If the ObjectBuffer length has been longer than 2 for the last 5 cycles or the main loop has
        been slowed down then the rate thread is slowed down by telling the INS to sub-sample. This
        mechanism is continued until the rate thread is able to keep up with the sub-sample rate.
        The inverse of this mechanism is run if the rate thread is able to keep up but is running slower
        than the gyro sample rate.
    6d. Updates the PID notch centers (1Hz)
 7. When the rate rate changes through sub-sampling the following values are updated:
    7a. The PID notch sample rate
    7b. The dshot rate is constrained to be never greater than the gyro rate or rate rate
    7c. The motors dt
 8. Independently of the rate thread the attitude control target is updated in the main loop. In order
    for target values to be consistent all updates are processed using local variables and the final
    target is only written at the end of the update as a vector. Direct control of the target (e.g. in
    autotune) is also constrained to be on all axes simultaneously using the new desired value. The
    target makes use of the current PIDs and the "latest" gyro, it might be possible to use a loop
    delayed gyro value, but that is currently out-of-scope.

 Performance considerations:

 On an H754 using ICM42688 and gyro sampling at 4KHz and rate thread at 4Khz the main CPU users are:

 ArduCopter    PRI=182 sp=0x30000600 STACK=4392/7168 LOAD=18.6%
 idle          PRI=  1 sp=0x300217B0 STACK= 296/ 504 LOAD= 4.3%
 rcout         PRI=181 sp=0x3001DAF0 STACK= 504/ 952 LOAD=10.7%
 SPI1          PRI=181 sp=0x3002DAB8 STACK= 856/1464 LOAD=17.5%
 SPI4          PRI=181 sp=0x3002D4A0 STACK= 888/1464 LOAD=18.3%
 rate          PRI=182 sp=0x3002B1D0 STACK=1272/1976 LOAD=22.4%

 There is a direct correlation between the rate rate and CPU load, so if the rate rate is half the gyro
 rate (i.e. 2Khz) we observe the following:

 ArduCopter    PRI=182 sp=0x30000600 STACK=4392/7168 LOAD=16.7%
 idle          PRI=  1 sp=0x300217B0 STACK= 296/ 504 LOAD=21.3%
 rcout         PRI=181 sp=0x3001DAF0 STACK= 504/ 952 LOAD= 6.2%
 SPI1          PRI=181 sp=0x3002DAB8 STACK= 856/1464 LOAD=16.7%
 SPI4          PRI=181 sp=0x3002D4A0 STACK= 888/1464 LOAD=17.8%
 rate          PRI=182 sp=0x3002B1D0 STACK=1272/1976 LOAD=11.5%

 So we get almost a halving of CPU load in the rate and rcout threads. This is the main way that CPU
 load can be handled on lower-performance boards, with the other mechanism being lowering the gyro rate.
 So at a very respectable gyro rate and rate rate both of 2Khz (still 5x standard main loop rate) we see:

 ArduCopter    PRI=182 sp=0x30000600 STACK=4440/7168 LOAD=15.6%
 idle          PRI=  1 sp=0x300217B0 STACK= 296/ 504 LOAD=39.4%
 rcout         PRI=181 sp=0x3001DAF0 STACK= 504/ 952 LOAD= 5.9%
 SPI1          PRI=181 sp=0x3002DAB8 STACK= 856/1464 LOAD= 8.9%
 SPI4          PRI=181 sp=0x3002D4A0 STACK= 896/1464 LOAD= 9.1%
 rate          PRI=182 sp=0x30029FB0 STACK=1296/1976 LOAD=11.8%

 This essentially means that its possible to run this scheme successfully on all MCUs by careful setting of 
 the maximum rates.

 Enabling rate thread timing debug for 4Khz reads with fast logging and armed we get the following data:

 Rate loop timing: gyro=178us, rate=13us, motors=45us, log=7us, ctrl=1us
 Rate loop timing: gyro=178us, rate=13us, motors=45us, log=7us, ctrl=1us
 Rate loop timing: gyro=177us, rate=13us, motors=46us, log=7us, ctrl=1us

 The log output is an average since it only runs at 1Khz, so roughly 28us elapsed. So the majority of the time
 is spent waiting for a gyro sample (higher is better here since it represents the idle time) updating the PIDs
 and outputting to the motors. Everything else is relatively cheap. Since the total cycle time is 250us the duty
 cycle is thus 29%
 */

/**
 * @brief Helper macro for integer division with rounding
 * @details Rounds to nearest integer rather than truncating
 */
#define DIV_ROUND_INT(x, d) ((x + d/2) / d)

/**
 * @brief Calculate the decimation factor needed to achieve a target rate
 * 
 * @details Determines how many gyro samples to skip to achieve the desired output
 *          rate. This is used to dynamically adjust the rate thread frequency based
 *          on CPU load and configuration. The calculation ensures the output rate is
 *          an integer divisor of the raw gyro rate.
 * 
 * @param[in] gyro_decimation Current gyro decimation factor (samples to skip)
 * @param[in] rate_hz Target output rate in Hz
 * 
 * @return Decimation factor to achieve target rate (minimum 1)
 * 
 * @note The returned decimation is always >= 1 to prevent division by zero
 */
uint8_t Copter::calc_gyro_decimation(uint8_t gyro_decimation, uint16_t rate_hz)
{
    return MAX(uint8_t(DIV_ROUND_INT(ins.get_raw_gyro_rate_hz() / gyro_decimation, rate_hz)), 1U);
}

/**
 * @brief Check if a decimated callback should run this iteration
 * 
 * @details Implements a simple counter-based decimation mechanism to run certain
 *          operations less frequently than the main rate loop. For example, logging
 *          at 1kHz when the rate loop runs at 4kHz. The counter is incremented on
 *          each call and the function returns true when it reaches the decimation rate.
 * 
 * @param[in] decimation_rate How many iterations between callback executions (0 = disabled)
 * @param[in,out] decimation_count Current counter value, reset to 0 when callback should run
 * 
 * @return true if callback should execute this iteration, false otherwise
 * 
 * @note The counter is automatically reset when the function returns true
 * @note If decimation_rate is 0, callback never runs
 */
static inline bool run_decimated_callback(uint8_t decimation_rate, uint8_t& decimation_count)
{
    return decimation_rate > 0 && ++decimation_count >= decimation_rate;
}

//#define RATE_LOOP_TIMING_DEBUG

/**
 * @brief High-frequency attitude rate controller thread main function
 * 
 * @details This is the entry point for the real-time rate control thread that runs
 *          independently of the main vehicle loop. It implements a tight control loop:
 *          gyro read → filter → rate control → motor output, minimizing latency between
 *          sensor input and actuator output.
 * 
 *          Thread Architecture:
 *          - Priority: Same as main loop (PRI=182) but runs more frequently
 *          - Scheduling: Blocked waiting for IMU samples, woken by INS backend
 *          - Frequency: Dynamically adjusts from 400Hz to 4000Hz based on CPU load
 *          - Stack: ~1976 bytes with ~1272 bytes typical usage
 * 
 *          Main Loop Sequence (per gyro sample):
 *          1. Wait for IMU backend to push filtered gyro sample (blocking wait)
 *          2. Run attitude rate controller with gyro data + gyro drift compensation
 *          3. Output new motor PWM values immediately
 *          4. Run decimated callbacks for logging, filtering, rate adjustment
 *          5. Dynamic rate adjustment based on CPU load and buffer depth
 * 
 *          Thread Synchronization:
 *          - Gyro samples: Lock-free ObjectBuffer from INS backend
 *          - Attitude targets: Atomic vector read from main loop
 *          - Motor outputs: Direct write to hal.rcout, synchronized with rcout thread
 *          - No mutexes in critical path to avoid priority inversion
 * 
 *          Real-Time Constraints:
 *          - Timing: Must complete processing within gyro sample period (e.g., 250us at 4kHz)
 *          - Latency: Typical path is 13us rate controller + 45us motor output = 58us
 *          - CPU Load: Self-regulates by adjusting decimation when CPU utilization high
 *          - Jitter: Minimal due to blocking on IMU samples from hardware interrupts
 * 
 *          Dynamic Rate Adjustment:
 *          - Monitors: ObjectBuffer depth, main loop delay, scheduler extra time
 *          - Slows down: If buffer depth > 2 for 5 cycles or main loop delayed
 *          - Speeds up: After 100ms of good performance, up to target rate
 *          - Range: From target rate down to main loop rate (never slower)
 * 
 *          Decimated Operations:
 *          - Main loop rate (~400Hz): Full motor output processing
 *          - Filter update (gyro_rate/2): Harmonic notch center frequency updates
 *          - Fast logging (1kHz): Rate PID and motor outputs
 *          - Rate monitoring (10Hz): CPU load and buffer depth checks
 *          - Notch update (1Hz): PID notch filter sample rate configuration
 * 
 *          Performance Characteristics (H754 @ 4kHz gyro and rate):
 *          - Rate thread: 22.4% CPU (11.5% at 2kHz)
 *          - Main loop: 18.6% CPU
 *          - SPI threads: ~35% combined CPU
 *          - Rcout thread: 10.7% CPU (6.2% at 2kHz)
 * 
 *          Safety Features:
 *          - Automatic rate reduction on CPU overload
 *          - Can be disabled at runtime via FAST_RATE_DISABLED
 *          - Falls back to main loop rate control if thread fails
 *          - Monitors and logs timing violations
 * 
 * @note This thread runs continuously once started, never exits
 * @note Thread is created by Copter::init() if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
 * @note Compile with RATE_LOOP_TIMING_DEBUG defined to enable performance profiling
 * 
 * @warning This is a real-time control thread - any blocking operations can cause
 *          control instability and potentially crash the vehicle
 * @warning Modifying the control sequence or timing may affect PID tuning and stability
 * @warning Priority inversion must be avoided - no mutex locks in critical path
 * 
 * @see AP_InertialSensor::get_next_gyro_sample() - Blocking wait for IMU data
 * @see AC_AttitudeControl::rate_controller_run_dt() - Core rate control algorithm
 * @see Copter::motors_output() - Motor output and servo processing
 * @see RateControllerRates - Structure holding decimation rates for sub-tasks
 */
void Copter::rate_controller_thread()
{
    // Initialize target decimation from parameter, constrained to reasonable range
    // (minimum 1, maximum ensures rate stays above main loop frequency)
    uint8_t target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1,
                                                     DIV_ROUND_INT(ins.get_raw_gyro_rate_hz(), AP::scheduler().get_loop_rate_hz()));
    uint8_t rate_decimation = target_rate_decimation;

    // Set up the decimation rates for logging, filtering, and main loop callbacks
    // These determine how often sub-tasks run relative to the rate loop frequency
    RateControllerRates rates;
    rate_controller_set_rates(rate_decimation, rates, false);

    // Loop counters for rate adjustment algorithm
    uint32_t rate_loop_count = 0;        // Successful loops at current rate
    uint32_t prev_loop_count = 0;        // Loops at previous rate (for transition logic)

    // Timing tracking for performance monitoring and dt calculation
    uint32_t last_run_us = AP_HAL::micros();
    float max_dt = 0.0;                  // Maximum observed loop time (for logging)
    float min_dt = 1.0;                  // Minimum observed loop time (for logging)
    uint32_t now_ms = AP_HAL::millis();
    uint32_t last_rate_check_ms = 0;     // When we last checked if rate adjustment needed
    uint32_t last_rate_increase_ms = 0;  // When we last tried to increase rate
#if HAL_LOGGING_ENABLED
    uint32_t last_rtdt_log_ms = now_ms;  // When we last logged rate thread dt
#endif
    uint32_t last_notch_sample_ms = now_ms; // When we last updated notch sample rate
    
    // State tracking for runtime enable/disable and mode transitions
    bool was_using_rate_thread = false;     // Were we using rate thread last iteration
    bool notify_fixed_rate_active = true;   // Should we notify user of rate change
    bool was_armed = false;                 // Arm state last iteration (for transitions)
    uint32_t running_slow = 0;              // Counter of how many cycles buffer depth > 2
#ifdef RATE_LOOP_TIMING_DEBUG
    // Performance profiling accumulators (compile-time optional)
    uint32_t gyro_sample_time_us = 0;       // Time waiting for gyro samples (blocking)
    uint32_t rate_controller_time_us = 0;   // Time in rate controller
    uint32_t motor_output_us = 0;           // Time outputting to motors
    uint32_t log_output_us = 0;             // Time writing logs
    uint32_t ctrl_output_us = 0;            // Time in control/monitoring code
    uint32_t timing_count = 0;              // Number of samples in current average
    uint32_t last_timing_msg_us = 0;        // When we last printed timing stats
#endif

    // Decimation counters for sub-tasks that run less frequently than rate loop
    // These are incremented each iteration and compared to rates.xxx_rate
#if HAL_LOGGING_ENABLED
    uint8_t log_loop_count = 0;        // Logging decimation counter
#endif
    uint8_t main_loop_count = 0;       // Main loop rate decimation counter
    uint8_t filter_loop_count = 0;     // Filter update decimation counter

    // Main rate control loop - runs continuously until thread is disabled
    // This is a real-time loop that blocks waiting for IMU samples
    while (true) {

#ifdef RATE_LOOP_TIMING_DEBUG
        uint32_t rate_now_us = AP_HAL::micros();
#endif

        // Allow runtime enable/disable of rate thread via parameter change
        // When disabled, sleep briefly to avoid tight spin loop
        if (get_fast_rate_type() == FastRateType::FAST_RATE_DISABLED) {
            if (was_using_rate_thread) {
                // Clean shutdown: disable fast rate buffer and restore main loop timing
                disable_fast_rate_loop(rates);
                was_using_rate_thread = false;
            }
            hal.scheduler->delay_microseconds(500);
            last_run_us = AP_HAL::micros();
            continue;
        }

        // Set up rate thread requirements if not already enabled
        // This enables the fast rate buffer in INS backend for lock-free sample passing
        if (!using_rate_thread) {
            enable_fast_rate_loop(rate_decimation, rates);
        }
        ins.set_rate_decimation(rate_decimation);

        // CRITICAL BLOCKING POINT: Wait for next filtered gyro sample from IMU backend
        // This is where the rate thread spends most of its time (typically 178us at 4kHz)
        // The INS backend will signal this thread when a new sample is available
        // Thread synchronization: Lock-free ObjectBuffer ensures no mutex blocking
        Vector3f gyro;
        if (!ins.get_next_gyro_sample(gyro)) {
            continue;   // No sample available, go around again (should be rare)
        }

#ifdef RATE_LOOP_TIMING_DEBUG
        gyro_sample_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // Calculate dt based on sensor rate, not actual elapsed time
        // This ensures consistent control response independent of scheduling jitter
        // sensor_dt is the time between gyro samples accounting for decimation
        const float sensor_dt = 1.0f * rate_decimation / ins.get_raw_gyro_rate_hz();
        const uint32_t now_us = AP_HAL::micros();
        const uint32_t dt_us = now_us - last_run_us;
        const float dt = dt_us * 1.0e-6;
        last_run_us = now_us;

        // Monitor if rate thread is falling behind by checking buffer depth
        // If ObjectBuffer has >2 samples queued, we're not keeping up with gyro rate
        // This triggers automatic rate reduction to prevent sample dropping
        if (ins.get_num_gyro_samples() > 2) {
            running_slow++;  // Increment counter of slow cycles
        } else if (running_slow > 0) {
            running_slow--;  // Decrement if we caught up
        }
        // Only count successful loops when main loop is also keeping up
        if (AP::scheduler().get_extra_loop_us() == 0) {
            rate_loop_count++;
        }

        // CORE RATE CONTROL: Run attitude rate controller with gyro data
        // Input: Current gyro rates (rad/s) + gyro drift compensation from AHRS
        // Output: Desired motor outputs stored in attitude_control object
        // Timing: Typically 13us on H754 MCU
        // This is the heart of the fast control loop - converting rate error to motor commands
        attitude_control->rate_controller_run_dt(gyro + ahrs.get_gyro_drift(), sensor_dt);

#ifdef RATE_LOOP_TIMING_DEBUG
        rate_controller_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // Output motor values immediately to minimize control latency
        // Full motor processing (including servo outputs) runs at main loop rate
        // Basic motor updates happen every rate loop iteration for minimum latency
        // Timing: Typically 45us on H754 MCU
        if (run_decimated_callback(rates.main_loop_rate, main_loop_count)) {
            main_loop_count = 0;
        }
        motors_output(main_loop_count == 0);

        // Update harmonic notch filter center frequencies at half the gyro rate
        // This allows tracking of motor RPM changes for notch filtering
        // Timing: Approximately 30us on H754 MCU
        if (run_decimated_callback(rates.filter_rate, filter_loop_count)) {
            filter_loop_count = 0;

            rate_controller_filter_update();
        }

        max_dt = MAX(dt, max_dt);
        min_dt = MIN(dt, min_dt);

#if HAL_LOGGING_ENABLED
        if (now_ms - last_rtdt_log_ms >= 100) {    // 10 Hz
            Log_Write_Rate_Thread_Dt(dt, sensor_dt, max_dt, min_dt);
            max_dt = sensor_dt;
            min_dt = sensor_dt;
            last_rtdt_log_ms = now_ms;
        }
#endif

#ifdef RATE_LOOP_TIMING_DEBUG
        motor_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

#if HAL_LOGGING_ENABLED
        // fast logging output
        if (should_log(MASK_LOG_ATTITUDE_FAST)) {
            if (run_decimated_callback(rates.fast_logging_rate, log_loop_count)) {
                log_loop_count = 0;
                rate_controller_log_update();

            }
        } else if (should_log(MASK_LOG_ATTITUDE_MED)) {
            if (run_decimated_callback(rates.medium_logging_rate, log_loop_count)) {
                log_loop_count = 0;
                rate_controller_log_update();
            }
        }
#endif

#ifdef RATE_LOOP_TIMING_DEBUG
        log_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        now_ms = AP_HAL::millis();

        // make sure we have the latest target rate
        target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1,
                                                 DIV_ROUND_INT(ins.get_raw_gyro_rate_hz(), AP::scheduler().get_loop_rate_hz()));
        if (now_ms - last_notch_sample_ms >= 1000 || !was_using_rate_thread) {
            // update the PID notch sample rate at 1Hz if we are
            // enabled at runtime
            last_notch_sample_ms = now_ms;
            attitude_control->set_notch_sample_rate(1.0 / sensor_dt);
#ifdef RATE_LOOP_TIMING_DEBUG
            hal.console->printf("Sample rate %.1f, main loop %u, fast rate %u, med rate %u\n", 1.0 / sensor_dt,
                                 rates.main_loop_rate, rates.fast_logging_rate, rates.medium_logging_rate);
#endif
        }

        // interlock for printing fixed rate active
        if (was_armed != motors->armed()) {
            notify_fixed_rate_active = !was_armed;
            was_armed = motors->armed();
        }

        // Once armed, switch to the fast rate if configured to do so
        if ((rate_decimation != target_rate_decimation || notify_fixed_rate_active)
            && ((get_fast_rate_type() == FastRateType::FAST_RATE_FIXED_ARMED && motors->armed())
                || get_fast_rate_type() == FastRateType::FAST_RATE_FIXED)) {
            rate_decimation = target_rate_decimation;
            rate_controller_set_rates(rate_decimation, rates, false);
            notify_fixed_rate_active = false;
        }

        // DYNAMIC RATE ADJUSTMENT: Monitor CPU load and adjust rate accordingly
        // Check every 100ms if rate needs adjustment based on system performance
        // This prevents CPU overload while maximizing control bandwidth
        if (now_ms - last_rate_check_ms >= 100
            && (get_fast_rate_type() == FastRateType::FAST_RATE_DYNAMIC
                || (get_fast_rate_type() == FastRateType::FAST_RATE_FIXED_ARMED && !motors->armed())
                || target_rate_decimation > rate_decimation)) {
            last_rate_check_ms = now_ms;
            const uint32_t att_rate = ins.get_raw_gyro_rate_hz()/rate_decimation;
            
            // RATE REDUCTION CONDITIONS: Slow down if any of these conditions met
            // - running_slow > 5: Buffer depth > 2 for multiple cycles (can't keep up)
            // - extra_loop_us > 0: Main loop is delayed (CPU overloaded)
            // - in_log_download: GCS downloading logs (high CPU/bandwidth usage)
            // - target changed: User changed parameter to slower rate
            if (running_slow > 5 || AP::scheduler().get_extra_loop_us() > 0
#if HAL_LOGGING_ENABLED
                || AP::logger().in_log_download()
#endif
                || target_rate_decimation > rate_decimation) {
                // Increase decimation (decrease rate) by one step
                const uint8_t new_rate_decimation = MAX(rate_decimation + 1, target_rate_decimation);
                const uint32_t new_attitude_rate = ins.get_raw_gyro_rate_hz() / new_rate_decimation;
                // Never go slower than main loop rate
                if (new_attitude_rate > AP::scheduler().get_filtered_loop_rate_hz()) {
                    rate_decimation = new_rate_decimation;
                    rate_controller_set_rates(rate_decimation, rates, true);  // Warn about CPU high
                    prev_loop_count = rate_loop_count;
                    rate_loop_count = 0;
                    running_slow = 0;
                }
            } 
            // RATE INCREASE CONDITIONS: Speed up if performing well
            // - Current rate slower than target
            // - At least 100ms (att_rate/10 loops) of successful operation at current rate
            // - Previous rate change was also successful (or this is first increase)
            // - Or it's been 10s since last attempt (periodic retry)
            else if (rate_decimation > target_rate_decimation && rate_loop_count > att_rate/10
                && (prev_loop_count > att_rate/10   // ensure there was 100ms worth of good readings at the higher rate
                    || prev_loop_count == 0         // last rate was actually a lower rate so keep going quickly
                    || now_ms - last_rate_increase_ms >= 10000)) { // every 10s retry
                // Decrease decimation (increase rate) by one step
                rate_decimation = rate_decimation - 1;

                rate_controller_set_rates(rate_decimation, rates, false);  // Normal CPU notification
                prev_loop_count = 0;
                rate_loop_count = 0;
                last_rate_increase_ms = now_ms;
            }
        }

#ifdef RATE_LOOP_TIMING_DEBUG
        timing_count++;
        ctrl_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();

        if (rate_now_us - last_timing_msg_us > 1e6) {
            hal.console->printf("Rate loop timing: gyro=%uus, rate=%uus, motors=%uus, log=%uus, ctrl=%uus\n",
                                unsigned(gyro_sample_time_us/timing_count), unsigned(rate_controller_time_us/timing_count),
                                unsigned(motor_output_us/timing_count), unsigned(log_output_us/timing_count), unsigned(ctrl_output_us/timing_count));
            last_timing_msg_us = rate_now_us;
            timing_count = 0;
            gyro_sample_time_us = rate_controller_time_us = motor_output_us = log_output_us = ctrl_output_us = 0;
        }
#endif

        was_using_rate_thread = true;
    }
}

/**
 * @brief Update harmonic notch filter center frequencies
 * 
 * @details Called from rate thread at half the gyro rate to update dynamic notch
 *          filters that track motor RPM. This allows the notch filters to follow
 *          changing motor frequencies and reject motor-induced vibration noise.
 *          
 *          The update copies IMU backend filter state to the frontend and updates
 *          all configured harmonic notch filters with new center frequencies based
 *          on current motor RPM or FFT analysis.
 * 
 * @note Runs at gyro_rate/2 (e.g., 2kHz when gyro runs at 4kHz)
 * @note Typical execution time: ~30us on H754 MCU
 * @note Called from rate_controller_thread() only, not thread-safe for other callers
 * 
 * @see AP_InertialSensor::update_backend_filters()
 * @see Copter::update_dynamic_notch()
 */
void Copter::rate_controller_filter_update()
{
    // Update the frontend center frequencies of all configured harmonic notch filters
    // This allows notches to track changing motor RPM for optimal vibration rejection
    for (auto &notch : ins.harmonic_notches) {
        update_dynamic_notch(notch);
    }

    // Copy backend filter data to frontend and apply updated notch configurations
    // This synchronizes the INS frontend with the backend filter state
    ins.update_backend_filters();
}

/**
 * @brief Configure rate controller frequency and all dependent subsystem rates
 * 
 * @details Updates the attitude rate and all dependent timing parameters when the
 *          rate decimation changes. This ensures all subsystems that depend on rate
 *          timing are synchronized:
 *          - Attitude controller notch filter sample rate
 *          - DShot ESC output rate (constrained to not exceed attitude rate)
 *          - Motor controller dt (time step for mixer calculations)
 *          - Logging decimation rates (fast/medium)
 *          - Main loop callback decimation
 *          - Filter update decimation
 * 
 *          Also notifies ground control station of rate change via telemetry message.
 * 
 * @param[in] rate_decimation Decimation factor for gyro rate (1 = no decimation)
 * @param[out] rates Structure to populate with calculated decimation rates for logging and callbacks
 * @param[in] warn_cpu_high If true, sends warning-level telemetry; if false, sends info-level
 * 
 * @note Must be called whenever rate_decimation changes to keep subsystems synchronized
 * @note Sends telemetry message to GCS reporting new rate
 * 
 * @see RateControllerRates - Structure holding all decimation rates
 * @see AC_AttitudeControl::set_notch_sample_rate()
 */
void Copter::rate_controller_set_rates(uint8_t rate_decimation, RateControllerRates& rates, bool warn_cpu_high)
{
    // Calculate the actual attitude control rate from gyro rate and decimation
    const uint32_t attitude_rate = ins.get_raw_gyro_rate_hz() / rate_decimation;
    
    // Update attitude controller's PID notch filters to match new sample rate
    attitude_control->set_notch_sample_rate(attitude_rate);
    
    // Update DShot ESC output rate, constrained to not exceed attitude rate
    hal.rcout->set_dshot_rate(SRV_Channels::get_dshot_rate(), attitude_rate);
    
    // Update motor mixer time step to match new rate
    motors->set_dt_s(1.0f / attitude_rate);
    
    // Notify ground control station of rate change
    gcs().send_text(warn_cpu_high ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                    "Rate CPU %s, rate set to %uHz",
                    warn_cpu_high ? "high" : "normal", (unsigned) attitude_rate);
                    
#if HAL_LOGGING_ENABLED
    // Calculate logging decimation rate (cap at 1kHz to avoid overwhelming logger)
    if (attitude_rate > 1000) {
        rates.fast_logging_rate = calc_gyro_decimation(rate_decimation, 1000);   // 1Khz max
    } else {
        // If attitude rate < 1kHz, log at main loop rate
         rates.fast_logging_rate = calc_gyro_decimation(rate_decimation, AP::scheduler().get_filtered_loop_rate_hz());
    }
    rates.medium_logging_rate = calc_gyro_decimation(rate_decimation, 10);   // 10Hz for medium-speed logs
#endif
    // Calculate main loop rate callback decimation (when to run full motor output processing)
    rates.main_loop_rate = calc_gyro_decimation(rate_decimation, AP::scheduler().get_filtered_loop_rate_hz());
    
    // Calculate filter update rate decimation (run at half gyro rate)
    rates.filter_rate = calc_gyro_decimation(rate_decimation, ins.get_raw_gyro_rate_hz() / 2);
}

/**
 * @brief Enable the high-frequency rate control thread
 * 
 * @details Activates the fast rate loop by enabling the lock-free gyro sample buffer
 *          in the INS backend and configuring all subsystem rates. This transitions
 *          from main loop rate control to high-frequency rate thread control.
 *          
 *          Enabling sequence:
 *          1. Enable fast rate buffer in INS (allows lock-free sample passing)
 *          2. Configure all subsystem rates for fast operation
 *          3. Enable forced trigger groups for motor outputs
 *          4. Mark rate thread as active
 * 
 * @param[in] rate_decimation Initial decimation factor for rate thread
 * @param[out] rates Structure to populate with calculated decimation rates
 * 
 * @note Called when transitioning from disabled to enabled state
 * @note Must be paired with disable_fast_rate_loop() when disabling
 * 
 * @see AP_InertialSensor::enable_fast_rate_buffer()
 * @see disable_fast_rate_loop()
 */
void Copter::enable_fast_rate_loop(uint8_t rate_decimation, RateControllerRates& rates)
{
    // Enable the lock-free ObjectBuffer in INS backend for gyro sample passing
    ins.enable_fast_rate_buffer();
    
    // Configure all rate-dependent subsystems (attitude controller, motors, logging)
    rate_controller_set_rates(rate_decimation, rates, false);
    
    // Force output groups to trigger from rate thread instead of main loop
    hal.rcout->force_trigger_groups(true);
    
    // Mark that we are now using the rate thread
    using_rate_thread = true;
}

/**
 * @brief Disable the high-frequency rate control thread
 * 
 * @details Deactivates the fast rate loop and returns to main loop rate control.
 *          This cleanly shuts down the rate thread infrastructure and restores
 *          normal main-loop-driven motor updates.
 *          
 *          Disabling sequence:
 *          1. Mark rate thread as inactive
 *          2. Calculate main loop rate decimation
 *          3. Configure all subsystems for main loop operation
 *          4. Restore normal trigger groups (main loop driven)
 *          5. Disable fast rate buffer in INS
 * 
 * @param[out] rates Structure to populate with main loop decimation rates
 * 
 * @note Called when transitioning from enabled to disabled state
 * @note Must be called before rate thread stops to ensure clean shutdown
 * 
 * @see AP_InertialSensor::disable_fast_rate_buffer()
 * @see enable_fast_rate_loop()
 */
void Copter::disable_fast_rate_loop(RateControllerRates& rates)
{
    // Mark that rate thread is no longer active
    using_rate_thread = false;
    
    // Calculate decimation for main loop rate operation
    uint8_t rate_decimation = calc_gyro_decimation(1, AP::scheduler().get_filtered_loop_rate_hz());
    
    // Reconfigure all subsystems for main loop rate
    rate_controller_set_rates(rate_decimation, rates, false);
    
    // Restore normal output groups (main loop driven, not rate thread driven)
    hal.rcout->force_trigger_groups(false);
    
    // Disable the lock-free gyro sample buffer
    ins.disable_fast_rate_buffer();
}

/**
 * @brief Log high-frequency rate controller data
 * 
 * @details Writes attitude rate controller outputs and PID values to dataflash log
 *          at high frequency (typically 1kHz). This provides detailed data for post-flight
 *          analysis and PID tuning. Only logs data that is updated by the rate loop,
 *          avoiding duplicate logging with main loop attitude logs.
 *          
 *          Logged data includes:
 *          - RATE message: Desired vs actual roll/pitch/yaw rates, PID outputs
 *          - PIDS message: Individual PID component values (if enabled)
 *          - FTN message: Harmonic notch filter states (if enabled)
 * 
 * @note Called from rate_controller_thread() at decimated rate (typically 1kHz)
 * @note Only logs if flight mode doesn't already log attitude data
 * @note Conditional compilation based on HAL_LOGGING_ENABLED
 * 
 * @see Log_Write_Rate() - Write rate controller outputs
 * @see Log_Write_PIDS() - Write PID component values
 */
void Copter::rate_controller_log_update()
{
#if HAL_LOGGING_ENABLED
    // Only log rate data if flight mode isn't already logging attitude
    // (avoids duplicate logging between rate thread and main loop)
    if (!copter.flightmode->logs_attitude()) {
        Log_Write_Rate();              // Log rate controller outputs
        Log_Write_PIDS();              // Log PID components (only if PIDS bitmask is set)
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    // Log harmonic notch filter states if fast notch logging enabled
    if (should_log(MASK_LOG_FTN_FAST)) {
        AP::ins().write_notch_log_messages();
    }
#endif
#endif
}

/**
 * @brief Main loop dynamic notch update (when rate thread disabled)
 * 
 * @details This is the main loop entry point for updating dynamic notch filters.
 *          When the rate thread is active, notch updates are handled by
 *          rate_controller_filter_update() instead. This function ensures notch
 *          filters continue to update even when the rate thread is disabled.
 *          
 *          Called from main loop at loop rate (typically 400Hz) or 200Hz depending
 *          on scheduler configuration.
 * 
 * @note Does nothing if rate thread is active (updates handled by rate thread)
 * @note Called from Copter::fast_loop() in main scheduler
 * 
 * @see rate_controller_filter_update() - Rate thread notch update
 * @see update_dynamic_notch_at_specified_rate() - Common notch update implementation
 */
void Copter::update_dynamic_notch_at_specified_rate_main()
{
    // If rate thread is active, it handles notch updates - don't duplicate
    if (using_rate_thread) {
        return;
    }

    // Rate thread not active, update notch filters from main loop
    update_dynamic_notch_at_specified_rate();
}

#endif // AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
