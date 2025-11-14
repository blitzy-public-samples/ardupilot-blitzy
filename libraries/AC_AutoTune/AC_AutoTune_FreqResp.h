#pragma once

/**
 * @file AC_AutoTune_FreqResp.h
 * @brief Frequency response measurement engine for AutoTune system identification
 * 
 * @details This module implements a real-time frequency response analysis engine that performs
 * gain and phase determination through DWELL (fixed-frequency) or SWEEP (frequency-varying)
 * excitation testing. The core algorithm uses time-domain peak detection to measure the
 * frequency response of the system (vehicle attitude dynamics) to commanded inputs.
 * 
 * **Algorithm Overview:**
 * The frequency response H(jω) = Y(jω) / U(jω) is determined by:
 * 1. Exciting the system with sinusoidal commands at test frequency
 * 2. Detecting peaks in both target (commanded) and measured (actual) responses
 * 3. Computing gain as amplitude ratio: |H(jω)| = A_measured / A_target
 * 4. Computing phase lag from time shift: ∠H(jω) = time_lag × frequency × 360°
 * 
 * **Peak Detection Method:**
 * Uses half-cycle detection rather than FFT for computational efficiency:
 * - Tracks max/min values in each half-cycle (zero-crossing to zero-crossing)
 * - Noise immunity via timing guards (new_target/new_meas) prevents false triggers
 * - Amplitude calculated as (max - min) / 2
 * - Peak times stored for phase calculation
 * 
 * **Buffering Mechanism:**
 * - Dual-buffer system: separate tracking for target command and measured response
 * - Ring buffers (ObjectBuffer, capacity 12) store peak data for multi-cycle processing
 * - DWELL mode: averages gain/phase over multiple cycles for improved SNR
 * - SWEEP mode: single cycle per frequency step for rapid broadband characterization
 * 
 * **Numerical Robustness:**
 * - Half-cycle sign change detection for cycle completion
 * - Counter-based peak validation (max_target_cnt, min_target_cnt)
 * - Temporary storage prevents overwriting in-progress cycle data
 * - Fixed timestep differentiation (dt ~0.0025s, 400Hz) for ANGLE→RATE conversion
 * 
 * **Usage Pattern:**
 * 1. init(input_type, response_type, cycles) - Initialize before test
 * 2. update(command, target, measured, freq) - Call every loop iteration (~400Hz)
 * 3. Check is_cycle_complete() after each update
 * 4. Read results: get_gain(), get_phase(), get_freq(), get_accel_max()
 * 5. Call reset_cycle_complete() to acknowledge and prepare for next cycle
 * 
 * @see AC_AutoTune Main AutoTune implementation that uses this frequency response engine
 * @see AC_AutoTune_Heli Helicopter autotune with chirp/sweep testing
 * @see AC_AutoTune_Multi Multicopter autotune with dwell testing
 * @see AP_Math/chirp.h Chirp signal generation for sweep tests
 * @see ObjectBuffer Ring buffer implementation for peak data storage
 */

#include <AP_Math/AP_Math.h>

/**
 * @class AC_AutoTune_FreqResp
 * @brief Frequency response measurement engine for system identification
 * 
 * @details This class implements a real-time frequency response analyzer that measures
 * the gain and phase characteristics of vehicle attitude dynamics. It processes commanded
 * and measured response signals to extract frequency response data used for PID tuning.
 * 
 * **Architecture and Data Flow:**
 * - Real-time peak detection: Tracks max/min values with noise immunity via timing guards
 * - Dual-buffer system: Separate tracking for target command (tgt_peak_info_buffer) and 
 *   measured response (meas_peak_info_buffer)
 * - Cycle completion detection: Half-cycle sign changes trigger peak storage
 * - Result calculation: Gain (amplitude ratio) and phase (time shift) computed when cycle complete
 * 
 * **Peak Detection Algorithm:**
 * 1. Monitor signal for zero crossings (sign changes) to detect half-cycles
 * 2. Track max/min within each half-cycle with noise filtering
 * 3. Store peak time and amplitude when half-cycle completes
 * 4. Buffer peaks for multi-cycle processing (DWELL) or single-cycle (SWEEP)
 * 5. Compute results when sufficient data collected
 * 
 * **Gain Calculation:**
 * - Gain = measured_amplitude / target_amplitude (dimensionless ratio)
 * - Amplitude = (peak_max - peak_min) / 2
 * - Averaged over multiple cycles (DWELL) or single cycle (SWEEP)
 * 
 * **Phase Calculation:**
 * - Phase lag (degrees) = (time_shift_between_peaks × test_frequency × 360°)
 * - Time shift measured from peak-to-peak timing
 * - Normalized to range [-180°, +180°]
 * - Negative phase = measured lags target (typical)
 * - Positive phase = measured leads target (unusual, may indicate instability)
 * 
 * **Max Acceleration Estimation:**
 * - Derived from max_command / max_meas_rate
 * - Used by AutoTune to validate response within expected limits
 * - Result in centi-degrees/s/s (cdss)
 * 
 * **Fixed Timestep Differentiation:**
 * - For ResponseType::ANGLE: internally differentiates angle to rate
 * - Assumes fixed dt ~0.0025s (400Hz loop rate)
 * - Rate = (angle[n] - angle[n-1]) / dt
 * - Variable loop rates not supported for angle differentiation
 * 
 * **Test Modes:**
 * - DWELL (InputType::DWELL): Fixed-frequency sinusoid, averages over multiple cycles
 *   * Pros: High SNR, accurate gain/phase at specific frequency
 *   * Cons: Slower (5-10 cycles per frequency), requires frequency stepping
 * 
 * - SWEEP (InputType::SWEEP): Frequency-varying chirp, single cycle per frequency
 *   * Pros: Rapid broadband characterization, continuous frequency coverage
 *   * Cons: Lower SNR, single-cycle measurement
 * 
 * **Thread Safety:**
 * - Designed for single-threaded use from main loop (~400Hz)
 * - No mutex/semaphore protection
 * - Not safe for concurrent access from multiple threads
 * 
 * **Performance Considerations:**
 * - Lightweight: O(1) computation per update() call
 * - Fixed memory: Peak buffers pre-allocated (capacity 12)
 * - No dynamic allocation during test
 * - Suitable for real-time 400Hz execution
 * 
 * **Usage Context:**
 * Called every scheduler loop iteration (~400Hz, 2.5ms period) with:
 * - command: Raw actuator command value
 * - target_response: Desired/commanded response after input shaping
 * - measured_response: Actual response from sensors (gyro/AHRS)
 * - test_frequency: Current excitation frequency (Hz)
 * 
 * @note Must call init() before starting each test sequence
 * @note Must call update() consistently at ~400Hz for correct differentiation
 * @warning Assumes consistent loop rate for angle differentiation
 * @warning Not thread-safe - single-threaded use only
 * @warning Buffer overflow not handled - assumes reasonable cycle counts (< 12 cycles)
 */
class AC_AutoTune_FreqResp {
public:
    /**
     * @brief Default constructor for frequency response analyzer
     * 
     * @details Initializes object with default values. Member variables are initialized
     * to safe defaults via in-class initializers. Must call init() before use.
     */
    AC_AutoTune_FreqResp()
{
}

    /**
     * @enum InputType
     * @brief Excitation input type for frequency response test
     * 
     * @details Determines the excitation signal type and measurement strategy:
     * 
     * **DWELL Mode:**
     * - Fixed-frequency sinusoidal excitation at constant frequency
     * - Measures gain/phase over multiple complete cycles (typically 5-10)
     * - Averages results for improved signal-to-noise ratio (SNR)
     * - Use case: High-precision measurements at specific frequencies of interest
     * - Test duration: Longer (multiple cycles × period per frequency)
     * - Typical application: Multicopter autotune with discrete frequency steps
     * 
     * **SWEEP Mode:**
     * - Frequency-varying chirp excitation (continuous frequency sweep)
     * - Measures gain/phase at each frequency step with single cycle
     * - Provides rapid broadband frequency response characterization
     * - Use case: Quick system identification across wide frequency range
     * - Test duration: Shorter (single cycle per frequency step)
     * - Typical application: Helicopter autotune with logarithmic frequency sweep
     */
    enum InputType {
        DWELL = 0,  ///< Fixed-frequency sinusoid, multi-cycle averaging for high SNR
        SWEEP = 1,  ///< Frequency-varying chirp, single-cycle per frequency for rapid characterization
    };

    /**
     * @enum ResponseType
     * @brief Type of response signal being measured
     * 
     * @details Specifies the type of response signal provided to update() and determines
     * internal signal processing:
     * 
     * **RATE Mode:**
     * - Input signals (tgt_resp, meas_resp) are angular rates (deg/s or rad/s)
     * - No internal differentiation required
     * - Signals used directly for peak detection and gain/phase calculation
     * - Use case: Rate controller tuning, gyro response measurement
     * - Typical units: deg/s (degrees per second)
     * 
     * **ANGLE Mode:**
     * - Input signals (tgt_resp, meas_resp) are angles (deg or rad)
     * - Internal differentiation converts angle to rate: rate = (angle[n] - angle[n-1]) / dt
     * - Fixed timestep dt ~0.0025s (400Hz loop rate) assumed for differentiation
     * - Differentiated rate used for peak detection and gain/phase calculation
     * - Use case: Angle controller tuning, attitude response measurement
     * - Typical units: deg (degrees)
     * 
     * @note ANGLE mode requires consistent 400Hz update rate for accurate differentiation
     * @warning Variable loop rates will cause incorrect rate calculations in ANGLE mode
     */
    enum ResponseType {
        RATE = 0,   ///< Measuring angular rate response (deg/s), input is rate command
        ANGLE = 1,  ///< Measuring angle response (deg), internal differentiation converts to rate
    };

    /**
     * @brief Initialize frequency response measurement for new test
     * 
     * @details Resets all internal state variables, buffers, counters, and flags to prepare
     * for a new frequency response test. Must be called before each test sequence (before
     * starting excitation signal). Configures test mode and averaging behavior.
     * 
     * **Initialization Actions:**
     * - Clears all peak tracking variables (max_target, min_target, max_meas, min_meas)
     * - Resets timing variables and counters to zero
     * - Empties peak data buffers (tgt_peak_info_buffer, meas_peak_info_buffer)
     * - Resets cycle_complete flag to false
     * - Stores input_type and response_type configuration
     * - Sets dwell_cycles parameter (used for DWELL mode averaging)
     * 
     * **DWELL Mode Configuration:**
     * - cycles parameter determines number of oscillation cycles for averaging
     * - Typical values: 5-10 cycles
     * - More cycles = better SNR, longer test duration
     * - Fewer cycles = faster test, lower SNR
     * 
     * **SWEEP Mode Configuration:**
     * - cycles parameter ignored (each frequency step is single cycle)
     * - Single-cycle measurement per frequency step regardless of cycles value
     * 
     * @param[in] input_type Excitation type: DWELL (fixed-frequency, multi-cycle averaging)
     *                       or SWEEP (frequency-varying chirp, single cycle per frequency)
     * @param[in] response_type Response signal type: RATE (angular rate input, no differentiation)
     *                          or ANGLE (angle input, internal differentiation to rate)
     * @param[in] cycles Number of oscillation cycles for DWELL averaging (typically 5-10).
     *                   Ignored for SWEEP mode. Higher values improve SNR at cost of test duration.
     * 
     * @note Must be called before starting excitation signal and calling update()
     * @note Can be called multiple times to restart test with different configuration
     */
    void init(InputType input_type, ResponseType response_type, uint8_t cycles);

    /**
     * @brief Process one sample of frequency response test data
     * 
     * @details Core measurement algorithm called every loop iteration at ~400Hz (2.5ms period).
     * Implements real-time peak detection, buffer management, and gain/phase calculation for
     * frequency response analysis. Must be called consistently every loop during test.
     * 
     * **Algorithm Flow (executed each call):**
     * 
     * 1. **Signal Conditioning (if ResponseType==ANGLE):**
     *    - Differentiate tgt_resp and meas_resp to get rates
     *    - Fixed timestep: dt ~0.0025s (400Hz assumed)
     *    - target_rate = (tgt_resp - prev_tgt_resp) / dt
     *    - measured_rate = (meas_resp - prev_meas_resp) / dt
     * 
     * 2. **Peak Detection (both target and measured signals):**
     *    - Track current max/min values in half-cycle
     *    - Noise filtering: new_target/new_meas timing guards prevent false peak triggers
     *    - When signal crosses zero (sign change): half-cycle complete
     *    - Store peak time and amplitude in buffer (push_to_tgt_buffer / push_to_meas_buffer)
     * 
     * 3. **Cycle Completion Detection:**
     *    - For DWELL: Wait until dwell_cycles complete oscillations accumulated in buffers
     *    - For SWEEP: Single cycle per frequency (one positive + one negative half-cycle)
     *    - Check buffer depth and cycle counters
     * 
     * 4. **Gain and Phase Calculation (when cycle complete):**
     *    - **Gain Calculation:**
     *      * Extract peak amplitudes from buffers
     *      * Target amplitude = (max_target - min_target) / 2
     *      * Measured amplitude = (max_meas - min_meas) / 2
     *      * Gain = measured_amplitude / target_amplitude (dimensionless ratio)
     *      * Average over multiple cycles for DWELL mode
     *    
     *    - **Phase Calculation:**
     *      * Time shift = time_of_measured_peak - time_of_target_peak
     *      * Phase lag (degrees) = time_shift × tgt_freq × 360°
     *      * Normalize to range [-180°, +180°]
     *      * Negative phase = measured lags target (typical)
     *      * Average over multiple cycles for DWELL mode
     *    
     *    - **Max Acceleration:**
     *      * Track max_meas_rate and corresponding max_command throughout test
     *      * max_accel = max_command / max_meas_rate (centi-degrees/s/s)
     *      * Used by AutoTune to validate response limits
     * 
     * 5. **Result Publication:**
     *    - Set cycle_complete = true when results ready
     *    - Store curr_test_freq, curr_test_gain, curr_test_phase, max_accel
     *    - Caller can retrieve via get_*() accessors
     * 
     * **Timing Requirements:**
     * - Must be called every loop iteration during test for correct peak detection
     * - Assumes consistent ~400Hz loop rate (2.5ms period)
     * - Variable loop rates will cause incorrect angle differentiation and phase calculation
     * 
     * **Signal Requirements:**
     * - command: Raw actuator command, dimensionless or actuator-specific units
     * - tgt_resp: Target response after input shaping, same units as meas_resp
     * - meas_resp: Measured response from sensors (AHRS angle or gyro rate)
     * - All signals should be oscillating at tgt_freq for meaningful results
     * 
     * @param[in] command Raw command value sent to actuator (dimensionless or actuator-specific).
     *                    Used for max acceleration calculation (max_command / max_meas_rate).
     * @param[in] tgt_resp Target/desired response value after input shaping.
     *                     Units: degrees (ANGLE type) or deg/s (RATE type).
     *                     This is the commanded response that the system should track.
     * @param[in] meas_resp Measured response from sensors (AHRS attitude or gyro rate).
     *                      Units: same as tgt_resp (degrees for ANGLE, deg/s for RATE).
     *                      This is the actual system response being measured.
     * @param[in] tgt_freq Test frequency in Hz. For DWELL: fixed frequency throughout test.
     *                     For SWEEP: current instantaneous chirp frequency (varies each call).
     * 
     * @note Must be called every loop iteration (~400Hz) during test for correct operation
     * @note Check is_cycle_complete() after each call to detect when results are ready
     * 
     * @warning Assumes consistent loop rate ~400Hz for ANGLE differentiation
     * @warning Variable dt not supported - will cause incorrect rate and phase calculations
     * @warning Signals must be oscillating near tgt_freq for meaningful frequency response
     * 
     * @see is_cycle_complete() Check if measurement cycle is complete and results available
     * @see get_gain() Retrieve calculated gain after cycle complete
     * @see get_phase() Retrieve calculated phase lag after cycle complete
     * @see reset_cycle_complete() Acknowledge results and prepare for next cycle
     */
    void update(float command, float tgt_resp, float meas_resp, float tgt_freq);

    /**
     * @brief Check if measurement cycle is complete and data ready
     * 
     * @details Returns true when sufficient data has been collected and gain/phase results
     * have been calculated. For DWELL mode: true after averaging over dwell_cycles complete
     * oscillations. For SWEEP mode: true after each single-cycle measurement.
     * 
     * When true, frequency response results are available via accessors:
     * - get_freq(): Test frequency (Hz)
     * - get_gain(): Frequency response gain (dimensionless)
     * - get_phase(): Phase lag (degrees)
     * - get_accel_max(): Maximum acceleration (cdss)
     * 
     * After reading results, caller must call reset_cycle_complete() to acknowledge
     * and prepare for next measurement cycle or frequency step.
     * 
     * @return true if gain/phase results available and ready to read
     * @return false if still collecting data, results not yet valid
     * 
     * @note Check after each update() call during test to detect result availability
     * @note Results remain valid until next init() call
     * 
     * @see reset_cycle_complete() Acknowledge results and prepare for next cycle
     * @see get_freq() Retrieve test frequency
     * @see get_gain() Retrieve gain result
     * @see get_phase() Retrieve phase result
     */
    bool is_cycle_complete() { return cycle_complete;}

    /**
     * @brief Clear cycle complete flag to start next measurement
     * 
     * @details Sets cycle_complete flag to false, acknowledging that results have been
     * read and processed. Must be called after reading results (when is_cycle_complete()
     * returns true) to prepare for next measurement cycle or frequency step.
     * 
     * **Usage Pattern:**
     * ```cpp
     * if (freq_resp.is_cycle_complete()) {
     *     float gain = freq_resp.get_gain();
     *     float phase = freq_resp.get_phase();
     *     // Process results...
     *     freq_resp.reset_cycle_complete();  // Acknowledge, ready for next cycle
     * }
     * ```
     * 
     * For DWELL mode: Prepares for next frequency step
     * For SWEEP mode: Prepares for next frequency in chirp
     * 
     * @note Required after each is_cycle_complete() == true before continuing test
     * @note Does not clear actual result values (curr_test_gain, curr_test_phase remain valid)
     */
    void reset_cycle_complete() { cycle_complete = false; }

    /**
     * @brief Get test frequency of completed measurement
     * 
     * @details Returns the test frequency (Hz) at which the current gain and phase
     * measurements were taken. This is the frequency of the excitation signal during
     * the measurement cycle.
     * 
     * For DWELL mode: Returns the fixed test frequency
     * For SWEEP mode: Returns the instantaneous frequency at which this result was measured
     * 
     * @return Frequency in Hz at which gain and phase were measured
     * 
     * @note Only valid after is_cycle_complete() == true
     * @note Result remains valid until next init() call
     */
    float get_freq() { return curr_test_freq; }
    
    /**
     * @brief Get measured frequency response gain
     * 
     * @details Returns the frequency response gain calculated as the ratio of measured
     * response amplitude to target response amplitude:
     * 
     * Gain = measured_amplitude / target_amplitude (dimensionless)
     * 
     * **Interpretation:**
     * - Gain > 1: System amplifies input (resonance or overshoot)
     * - Gain = 1: Perfect tracking (ideal response)
     * - Gain < 1: System attenuates input (typical at high frequencies)
     * 
     * **Typical Frequency Response Characteristics:**
     * - Low frequencies (< 1 Hz): Gain ≈ 1 (good tracking)
     * - Crossover frequency: Gain ≈ 0.7 (-3dB, where phase lag ≈ -90°)
     * - High frequencies (> 10 Hz): Gain << 1 (roll-off, limited bandwidth)
     * - Resonant peak: Gain > 1 (indicates underdamping, may need D-term increase)
     * 
     * For DWELL mode: Averaged over dwell_cycles oscillations
     * For SWEEP mode: Single-cycle measurement
     * 
     * @return Gain (dimensionless ratio): measured_amplitude / target_amplitude
     * 
     * @note Only valid after is_cycle_complete() == true
     * @note Result remains valid until next init() call
     * 
     * @see get_phase() Phase lag associated with this gain measurement
     * @see get_freq() Frequency at which this gain was measured
     */
    float get_gain() { return curr_test_gain; }
    
    /**
     * @brief Get measured frequency response phase lag
     * 
     * @details Returns the phase lag (degrees) between target and measured responses,
     * calculated from the time shift between corresponding peaks:
     * 
     * Phase lag = (time_shift × frequency × 360°), normalized to [-180°, +180°]
     * 
     * **Sign Convention:**
     * - Negative phase: Measured response LAGS target (typical, causal system)
     * - Positive phase: Measured response LEADS target (unusual, may indicate instability)
     * - Zero phase: Perfect synchronization (rare, only at very low frequencies)
     * 
     * **Interpretation:**
     * - Phase ≈ 0°: Excellent tracking (low frequency, well-tuned)
     * - Phase ≈ -90°: Typical at crossover frequency
     * - Phase < -180°: Poor stability margins, risk of instability
     * - Phase approaching -180°: System approaching instability limit
     * 
     * **Stability Considerations:**
     * - Phase margin = 180° + phase_lag (at gain crossover frequency where gain = 1)
     * - Adequate phase margin: > 45° (conservative: > 60°)
     * - At gain crossover: phase should be > -135° (better: > -120°) for stability
     * - Excessive phase lag indicates need for reduced gains or improved controller
     * 
     * For DWELL mode: Averaged over dwell_cycles oscillations
     * For SWEEP mode: Single-cycle measurement
     * 
     * @return Phase lag in degrees, range [-180, 180]
     *         Negative = measured lags target (typical)
     *         Positive = measured leads target (unusual)
     * 
     * @note Only valid after is_cycle_complete() == true
     * @note Result remains valid until next init() call
     * 
     * @see get_gain() Gain magnitude associated with this phase measurement
     * @see get_freq() Frequency at which this phase was measured
     */
    float get_phase() { return curr_test_phase; }
    
    /**
     * @brief Get maximum measured acceleration during test
     * 
     * @details Returns the maximum acceleration observed during the frequency response test,
     * calculated as:
     * 
     * max_accel = max_command / max_meas_rate (units: centi-degrees/s/s, cdss)
     * 
     * This value is used by AutoTune to:
     * 1. Validate that measured response is within expected acceleration limits
     * 2. Calculate gain limits that prevent excessive acceleration
     * 3. Ensure vehicle remains within safe operating envelope during tuning
     * 
     * **Physical Meaning:**
     * - Represents the maximum angular acceleration the vehicle experienced
     * - Higher values indicate more aggressive response (may be desirable or indicate instability)
     * - Lower values indicate sluggish response (may indicate too-low gains)
     * 
     * **Units:**
     * - centi-degrees/s/s (cdss): ArduPilot standard angular acceleration unit
     * - Conversion: 1 deg/s/s = 100 cdss
     * 
     * The maximum is tracked throughout the entire test (all cycles) and represents
     * the peak acceleration observed at any point.
     * 
     * @return Maximum acceleration in centi-degrees/s/s (cdss)
     * 
     * @note Only valid after is_cycle_complete() == true
     * @note Accumulated over entire test, not reset between cycles
     * @note Used by AutoTune for gain limit calculations and safety validation
     */
    float get_accel_max() { return max_accel; }

private:
    // **Target Signal Peak Tracking Variables**
    // These variables track peaks (max/min) in the target/commanded response signal
    
    /// Time when new target peak search started (ms). Provides noise immunity by preventing
    /// premature peak detection. Guards against false triggers from sensor noise or
    /// rapid oscillations. Typical guard period ~50-100ms.
    uint32_t new_tgt_time_ms;

    /// Flag indicating actively searching for new target peak. Set to true when half-cycle
    /// completes (sign change detected) and search begins for next peak. Prevents
    /// detecting multiple peaks in same half-cycle.
    bool new_target = false;

    /// Maximum target response value in current half-cycle. Updated continuously during
    /// positive half-cycle. Units: degrees (ANGLE type) or deg/s (RATE type).
    float max_target;

    /// Time of maximum target value in current half-cycle (milliseconds). Used for
    /// phase calculation by measuring time shift between target and measured peaks.
    uint32_t max_tgt_time;

    /// Counter tracking number of target maximum peaks detected. Increments each time
    /// positive half-cycle completes. Used to determine cycle completion (for DWELL:
    /// counter >= dwell_cycles indicates sufficient data collected).
    uint16_t max_target_cnt;

    /// Temporary storage for previous cycle's maximum target value. Holds max_target
    /// from completed half-cycle while current half-cycle is in progress. Prevents
    /// overwriting data before it's buffered.
    float temp_max_target;

    /// Temporary storage for time of previous cycle's maximum target. Holds max_tgt_time
    /// from completed half-cycle for phase calculation. Units: milliseconds.
    uint32_t temp_max_tgt_time;

    /// Minimum target response value in current half-cycle. Updated continuously during
    /// negative half-cycle. Units: degrees (ANGLE type) or deg/s (RATE type).
    float min_target;

    /// Counter tracking number of target minimum peaks detected. Increments each time
    /// negative half-cycle completes. Should equal max_target_cnt ± 1 during normal operation.
    uint16_t min_target_cnt;

    /// Temporary storage for previous cycle's minimum target value. Holds min_target
    /// from completed half-cycle while current half-cycle is in progress.
    float temp_min_target;

    /// Target value from previous update() call. Used for sign change detection to
    /// identify zero crossings (half-cycle boundaries). Units: same as max_target.
    float prev_target;

    /// Target response from previous update() call. Used for differentiation when
    /// ResponseType == ANGLE: target_rate = (tgt_resp - prev_tgt_resp) / dt.
    /// Units: degrees (ANGLE type) or deg/s (RATE type).
    float prev_tgt_resp;

    /// Target signal amplitude for current cycle gain calculation. Computed as
    /// (max_target - min_target) / 2. Used in denominator of gain ratio.
    /// Units: degrees or deg/s depending on ResponseType.
    float temp_tgt_ampl;

    // **Measured Signal Peak Tracking Variables**
    // These variables track peaks (max/min) in the measured/actual response signal from sensors
    
    /// Time when new measured peak search started (ms). Provides noise immunity for measured
    /// signal, preventing false peak triggers from sensor noise, vibration, or rapid transients.
    /// Independent timing guard from target signal for robustness.
    uint32_t new_meas_time_ms;

    /// Flag indicating actively searching for new measured peak. Set to true when half-cycle
    /// completes (sign change) and search begins for next measured peak. Prevents multiple
    /// peak detections in same half-cycle of measured response.
    bool new_meas = false;

    /// Maximum measured response value in current half-cycle. Updated continuously during
    /// positive half-cycle of measured signal. Units: degrees (ANGLE type) or deg/s (RATE type).
    /// Represents peak of actual vehicle response from sensors (AHRS/gyro).
    float max_meas;

    /// Time of maximum measured value in current half-cycle (milliseconds). Critical for
    /// phase calculation: phase_lag = (max_meas_time - max_tgt_time) × frequency × 360°.
    uint32_t max_meas_time;

    /// Counter tracking number of measured maximum peaks detected. Should track closely
    /// with max_target_cnt (within ±1). Large discrepancy indicates lost cycles or noise.
    uint16_t max_meas_cnt;

    /// Temporary storage for previous cycle's maximum measured value. Holds max_meas
    /// from completed half-cycle while current half-cycle is in progress. Prevents
    /// data corruption before buffering.
    float temp_max_meas;

    /// Temporary storage for time of previous cycle's maximum measured value. Holds
    /// max_meas_time from completed half-cycle for phase calculation. Units: milliseconds.
    uint32_t temp_max_meas_time;

    /// Minimum measured response value in current half-cycle. Updated continuously during
    /// negative half-cycle of measured signal. Units: degrees (ANGLE) or deg/s (RATE).
    float min_meas;

    /// Counter tracking number of measured minimum peaks detected. Should equal
    /// max_meas_cnt ± 1 during normal operation. Used for cycle completion detection.
    uint16_t min_meas_cnt;

    /// Temporary storage for previous cycle's minimum measured value. Holds min_meas
    /// from completed half-cycle while current half-cycle is in progress.
    float temp_min_meas;

    /// Measured value from previous update() call. Used for sign change detection to
    /// identify zero crossings (half-cycle boundaries) in measured response.
    float prev_meas;

    /// Measured response from previous update() call. Used for differentiation when
    /// ResponseType == ANGLE: measured_rate = (meas_resp - prev_meas_resp) / dt.
    /// Units: degrees (ANGLE type) or deg/s (RATE type). Source: sensors (AHRS/gyro).
    float prev_meas_resp;

    /// Measured signal amplitude for current cycle gain calculation. Computed as
    /// (max_meas - min_meas) / 2. Used in numerator of gain ratio: gain = temp_meas_ampl / temp_tgt_ampl.
    /// Units: degrees or deg/s depending on ResponseType.
    float temp_meas_ampl;

    // **Rate Calculation Variables (for ANGLE mode differentiation)**
    
    /// Target rate calculated from angle differentiation when ResponseType == ANGLE.
    /// Computed as: target_rate = (tgt_resp - prev_tgt_resp) / dt where dt ~0.0025s (400Hz).
    /// Used for peak detection in ANGLE mode. Units: deg/s or rad/s.
    float target_rate;

    /// Measured rate calculated from angle differentiation when ResponseType == ANGLE.
    /// Computed as: measured_rate = (meas_resp - prev_meas_resp) / dt where dt ~0.0025s.
    /// Used for peak detection in ANGLE mode. Units: deg/s or rad/s.
    float measured_rate;

    // **Timing and State Variables**
    
    /// Start time of test input (milliseconds). Records when excitation signal began.
    /// Used to track total test duration and for timing-based state management.
    uint32_t input_start_time_ms;

    /// Flag indicating one complete measurement cycle finished and results available.
    /// Set to true when sufficient cycles collected (DWELL) or single cycle complete (SWEEP).
    /// Caller checks via is_cycle_complete() and must clear via reset_cycle_complete().
    bool cycle_complete = false;

    /// Number of oscillation cycles to average for DWELL mode. Typical values: 5-10 cycles.
    /// Higher values = better SNR, longer test. Ignored for SWEEP mode (always single cycle).
    /// Set by init() cycles parameter.
    uint8_t dwell_cycles;

    // **Frequency Response Results**
    // These variables store the calculated frequency response data available after cycle_complete == true
    
    /// Test frequency (Hz) at which current gain and phase were measured. For DWELL: fixed
    /// frequency throughout test. For SWEEP: instantaneous chirp frequency at measurement time.
    float curr_test_freq; 
    
    /// Calculated frequency response gain (dimensionless ratio): measured_amplitude / target_amplitude.
    /// Gain > 1 indicates amplification/resonance, gain < 1 indicates attenuation/roll-off.
    /// For DWELL: averaged over dwell_cycles. For SWEEP: single-cycle measurement.
    float curr_test_gain;
    
    /// Calculated phase lag in degrees, range [-180, 180]. Negative = measured lags target (typical).
    /// Computed from time shift: phase = (time_shift × frequency × 360°). For DWELL: averaged
    /// over dwell_cycles. For SWEEP: single-cycle measurement.
    float curr_test_phase;

    // **Maximum Acceleration Tracking**
    // Used by AutoTune to validate response limits and calculate safe gain limits
    
    /// Maximum measured rate observed throughout entire test. Tracked continuously across
    /// all cycles. Units: deg/s or rad/s depending on ResponseType. Used in max accel calculation.
    float max_meas_rate;

    /// Command value associated with maximum measured rate. Tracked continuously to correlate
    /// command with response. Units: dimensionless or actuator-specific. Used in max accel calculation.
    float max_command;

    /// Maximum angular acceleration observed during test. Calculated as max_command / max_meas_rate.
    /// Units: centi-degrees/s/s (cdss). Used by AutoTune for gain limiting and safety validation.
    /// Conversion: 1 deg/s/s = 100 cdss.
    float max_accel;

    // **Test Configuration**
    
    /// Excitation input type: DWELL (fixed-frequency, multi-cycle averaging) or
    /// SWEEP (frequency-varying chirp, single cycle per frequency). Set by init().
    InputType excitation;

    /// Response signal type: RATE (angular rate, no differentiation) or
    /// ANGLE (angle, requires differentiation to rate). Set by init().
    ResponseType response;

    // **SWEEP Mode Peak Tracking**
    
    /**
     * @brief Peak tracking data structure for SWEEP mode
     * 
     * @details Stores previous cycle's peak information for SWEEP mode single-cycle
     * measurements. The "_m1" suffix indicates "minus 1" (previous cycle data).
     * Used to track state across frequency steps in chirp sweep testing.
     */
    struct sweep_peak_finding_data {
        uint16_t count_m1;      ///< Peak count from previous cycle, used to detect cycle boundaries
        float amplitude_m1;     ///< Peak amplitude from previous cycle (deg or deg/s)
        float max_time_m1;      ///< Time of peak from previous cycle (ms), used for phase calculation
    };

    /// Measured response peak tracking for SWEEP mode. Stores previous cycle's measured
    /// peak data to enable single-cycle gain/phase calculation as frequency varies in chirp.
    sweep_peak_finding_data sweep_meas;

    /// Target command peak tracking for SWEEP mode. Stores previous cycle's target
    /// peak data to correlate with measured response for gain/phase at each frequency step.
    sweep_peak_finding_data sweep_tgt;

    // **Peak Data Storage Structures**
    
    /**
     * @brief Peak information structure for ring buffer storage
     * 
     * @details Stores one peak's data (amplitude, timing, occurrence count) for
     * multi-cycle processing in DWELL mode or sequential storage in SWEEP mode.
     * Pushed to buffer when half-cycle completes, pulled for gain/phase calculation.
     */
    struct peak_info {
        uint16_t curr_count;  ///< Peak occurrence counter, used to match target/measured peaks
        float amplitude;      ///< Peak amplitude value: (max - min) / 2, units: deg or deg/s
        uint32_t time_ms;     ///< Time of peak occurrence (milliseconds), critical for phase calculation
    };

    // **Ring Buffers for Peak Data**
    
    /// Ring buffer storing measured response peak data. Capacity 12 entries allows storage
    /// of up to 12 half-cycle peaks (6 complete oscillations for DWELL averaging). FIFO
    /// behavior: oldest data automatically discarded when buffer full. Used to accumulate
    /// multi-cycle data for averaging (DWELL) or sequential processing (SWEEP).
    ObjectBuffer<peak_info> meas_peak_info_buffer{12};

    /// Ring buffer storing target command peak data. Capacity 12 entries matches measured
    /// buffer. Parallel storage with meas_peak_info_buffer enables correlation of target
    /// and measured peaks for gain/phase calculation. Each entry corresponds to one half-cycle.
    ObjectBuffer<peak_info> tgt_peak_info_buffer{12};

    // **Buffer Management Methods (Private)**
    
    /**
     * @brief Push measured peak data into ring buffer
     * 
     * @details Stores one measured response peak (amplitude, time, count) into the
     * measured peak ring buffer. Called when measured signal half-cycle completes
     * (zero crossing detected). Buffer automatically overwrites oldest entry if full.
     * 
     * @param[in] count Peak occurrence counter for matching with target peaks
     * @param[in] amplitude Measured peak amplitude: (max_meas - min_meas) / 2, units: deg or deg/s
     * @param[in] time_ms Time of measured peak occurrence (milliseconds) for phase calculation
     * 
     * @note Called internally by update() when measured half-cycle completes
     */
    void push_to_meas_buffer(uint16_t count, float amplitude, uint32_t time_ms);

    /**
     * @brief Pull measured peak data from ring buffer
     * 
     * @details Retrieves oldest measured response peak data from ring buffer (FIFO).
     * Used during gain/phase calculation to extract stored peak information.
     * Removes entry from buffer after retrieval.
     * 
     * @param[out] count Peak occurrence counter retrieved from buffer
     * @param[out] amplitude Measured peak amplitude retrieved from buffer (deg or deg/s)
     * @param[out] time_ms Time of measured peak retrieved from buffer (milliseconds)
     * 
     * @note Called internally by update() during gain/phase calculation
     * @note Caller should check buffer not empty before calling
     */
    void pull_from_meas_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

    /**
     * @brief Push target peak data into ring buffer
     * 
     * @details Stores one target command peak (amplitude, time, count) into the
     * target peak ring buffer. Called when target signal half-cycle completes
     * (zero crossing detected). Parallel storage with measured buffer for correlation.
     * 
     * @param[in] count Peak occurrence counter for matching with measured peaks
     * @param[in] amplitude Target peak amplitude: (max_target - min_target) / 2, units: deg or deg/s
     * @param[in] time_ms Time of target peak occurrence (milliseconds) for phase calculation
     * 
     * @note Called internally by update() when target half-cycle completes
     */
    void push_to_tgt_buffer(uint16_t count, float amplitude, uint32_t time_ms);

    /**
     * @brief Pull target peak data from ring buffer
     * 
     * @details Retrieves oldest target command peak data from ring buffer (FIFO).
     * Used during gain/phase calculation to extract stored peak information and
     * correlate with corresponding measured peaks. Removes entry from buffer after retrieval.
     * 
     * @param[out] count Peak occurrence counter retrieved from buffer
     * @param[out] amplitude Target peak amplitude retrieved from buffer (deg or deg/s)
     * @param[out] time_ms Time of target peak retrieved from buffer (milliseconds)
     * 
     * @note Called internally by update() during gain/phase calculation
     * @note Caller should check buffer not empty before calling
     */
    void pull_from_tgt_buffer(uint16_t &count, float &amplitude, uint32_t &time_ms);

};
