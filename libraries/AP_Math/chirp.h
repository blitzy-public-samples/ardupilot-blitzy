/**
 * @file chirp.h
 * @brief Chirp waveform generation for frequency sweep testing and system identification
 * 
 * @details This file implements a chirp (frequency sweep) signal generator that produces
 *          a sinusoidal waveform with time-varying frequency. The frequency transitions
 *          exponentially from a start frequency to an end frequency over a specified duration.
 *          
 *          Chirp signals are commonly used for:
 *          - System identification and frequency response analysis
 *          - Control system tuning and parameter optimization
 *          - Vibration analysis and structural testing
 *          - Filter characterization
 *          
 *          The implementation includes configurable fade-in/fade-out windowing to reduce
 *          transient effects at the start and end of the sweep, and supports a constant
 *          frequency dwell period before beginning the sweep.
 * 
 * @note Typical usage: Initialize with desired frequency range and duration, then call
 *       update() at regular intervals to generate the time-varying waveform output.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

/**
 * @class Chirp
 * @brief Generates exponential chirp (frequency sweep) signals for system identification
 * 
 * @details The Chirp class produces a sinusoidal waveform with exponentially varying
 *          frequency, suitable for frequency response analysis and system identification.
 *          The frequency sweeps from a user-defined start frequency to end frequency
 *          over a specified time period.
 *          
 *          Features:
 *          - Exponential frequency sweep (constant energy per octave)
 *          - Configurable amplitude fade-in and fade-out windows (reduces transients)
 *          - Optional constant frequency dwell period before sweep begins
 *          - Real-time frequency tracking via get_frequency_rads()
 *          - Completion detection for automated test sequences
 *          
 *          Algorithm Overview:
 *          1. Optional fade-in period: amplitude ramps from 0 to full magnitude
 *          2. Optional constant frequency dwell: remains at start frequency
 *          3. Exponential frequency sweep: frequency increases exponentially
 *          4. Optional fade-out period: amplitude ramps from full magnitude to 0
 *          
 *          The exponential sweep ensures equal energy distribution across frequency
 *          bands when analyzed on a logarithmic frequency scale, which is ideal for
 *          control system analysis where decades of frequency are of interest.
 * 
 * @note Thread-safe for read-only operations after initialization.
 *       update() should be called from a single thread or with external synchronization.
 * 
 * @warning During exponential sweeps with large frequency ratios, numerical precision
 *          may be reduced near the end of the sweep. For frequency ratios exceeding
 *          1000:1, consider splitting into multiple chirps or using higher precision
 *          time measurements.
 */
class Chirp {

public:

    /**
     * @brief Default constructor - initializes chirp object to safe defaults
     * 
     * @details Constructs a Chirp object with all parameters initialized to zero or false.
     *          The chirp must be initialized with init() before use.
     */
    Chirp();

    /**
     * @brief Initialize the chirp signal generator with frequency sweep parameters
     * 
     * @details Configures all parameters for the chirp waveform generation. The chirp will
     *          sweep exponentially from frequency_start_hz to frequency_stop_hz over the
     *          duration specified by time_record. Optional fade-in, fade-out, and constant
     *          frequency periods can be configured to shape the sweep behavior.
     *          
     *          Time Budget Breakdown:
     *          - [0, time_fade_in]: Amplitude ramps from 0 to full (fade-in window)
     *          - [time_fade_in, time_fade_in + time_const_freq]: Constant start frequency
     *          - [time_fade_in + time_const_freq, time_record - time_fade_out]: Exponential sweep
     *          - [time_record - time_fade_out, time_record]: Amplitude ramps to 0 (fade-out window)
     *          
     *          Resets the completion flag to false, allowing the chirp to be reused.
     * 
     * @param[in] time_record          Total chirp duration in seconds (must be > 0)
     * @param[in] frequency_start_hz   Starting frequency of the sweep in Hertz (must be > 0)
     * @param[in] frequency_stop_hz    Ending frequency of the sweep in Hertz (must be > 0)
     * @param[in] time_fade_in         Duration of amplitude fade-in window in seconds (≥ 0)
     * @param[in] time_fade_out        Duration of amplitude fade-out window in seconds (≥ 0)
     * @param[in] time_const_freq      Duration to dwell at start frequency before sweep in seconds (≥ 0)
     * 
     * @note frequency_start_hz and frequency_stop_hz are specified in Hz, but internally
     *       converted to rad/s for waveform generation.
     * 
     * @note Sum of time_fade_in, time_const_freq, and time_fade_out should not exceed
     *       time_record, or the exponential sweep phase will be very short or nonexistent.
     * 
     * @warning Calling init() resets the chirp state. Any in-progress chirp will restart.
     */
    void init(float time_record, float frequency_start_hz, float frequency_stop_hz, float time_fade_in, float time_fade_out, float time_const_freq);

    /**
     * @brief Calculate chirp signal output at the specified time
     * 
     * @details Generates the chirp waveform value for the given time instant. This method
     *          implements the complete chirp algorithm including:
     *          - Fade-in/fade-out amplitude windowing (Hann window function)
     *          - Constant frequency dwell period
     *          - Exponential frequency sweep with phase continuity
     *          - Automatic completion detection
     *          
     *          The output is a sinusoidal waveform: output = window * magnitude * sin(phase)
     *          where phase is computed to ensure smooth frequency transitions.
     *          
     *          This method updates internal state including current frequency (waveform_freq_rads),
     *          window amplitude, and completion flag. It should be called sequentially with
     *          increasing time values for proper phase continuity.
     * 
     * @param[in] time                 Current time in seconds relative to chirp start (≥ 0)
     * @param[in] waveform_magnitude   Desired peak amplitude of the chirp signal (any real number)
     * 
     * @return Chirp waveform output value at the specified time.
     *         Returns 0.0 if time exceeds time_record (chirp completed).
     *         Otherwise returns windowed sinusoid: amplitude * sin(phase(t))
     * 
     * @note Call this method at regular intervals (e.g., in a control loop) with monotonically
     *       increasing time values. The time parameter is typically the elapsed time since
     *       the chirp started.
     * 
     * @note The waveform_magnitude parameter allows real-time amplitude scaling. The actual
     *       output amplitude is waveform_magnitude * window(time), where window varies from
     *       0 to 1 during fade-in/fade-out periods.
     * 
     * @note For accurate frequency response measurements, sample this output at a rate
     *       significantly higher than the maximum chirp frequency (Nyquist criterion).
     */
    float update(float time, float waveform_magnitude);

    /**
     * @brief Get the current instantaneous frequency of the chirp waveform
     * 
     * @details Returns the current frequency at which the chirp is oscillating. This value
     *          is updated each time update() is called and reflects the instantaneous frequency
     *          of the generated waveform.
     *          
     *          During the constant frequency dwell period, this returns the start frequency.
     *          During the exponential sweep, this returns the time-varying frequency.
     *          After the chirp completes, this returns the last computed frequency.
     * 
     * @return Current waveform frequency in radians per second (rad/s).
     *         To convert to Hertz: frequency_hz = get_frequency_rads() / (2 * π)
     * 
     * @note This is a read-only accessor. The frequency is computed internally by update().
     * 
     * @note Useful for logging, real-time monitoring, or synchronizing other operations
     *       with the current sweep frequency.
     */
    float get_frequency_rads() {return waveform_freq_rads; }

    /**
     * @brief Check if the chirp sequence has completed
     * 
     * @details Returns true once the chirp time has exceeded the configured time_record
     *          duration. Once completed, subsequent calls to update() will return 0.0.
     *          
     *          The completion flag is set internally by update() when time >= time_record,
     *          and is reset to false when init() is called.
     * 
     * @return true if chirp has finished (time exceeded time_record), false otherwise
     * 
     * @note This method is const and thread-safe for reading after update() completes.
     * 
     * @note Useful for automated test sequences to detect when to proceed to the next
     *       test phase or to stop logging/data collection.
     */
    bool completed() const { return complete; }

private:
    // Total chirp duration in seconds - defines the complete time span from start to finish
    float record;

    // Peak amplitude of the chirp oscillation - scaled by window function during fade periods
    float magnitude;

    // Starting frequency of the sweep in rad/s (converted from Hz in init())
    float wMin;

    // Ending frequency of the sweep in rad/s (converted from Hz in init())
    float wMax;

    // Duration of amplitude fade-in window in seconds - uses Hann window to ramp from 0 to 1
    float fade_in;

    // Duration of amplitude fade-out window in seconds - uses Hann window to ramp from 1 to 0
    float fade_out;

    // Duration to maintain constant start frequency before beginning exponential sweep (seconds)
    float const_freq;

    // Frequency ratio coefficient (wMax/wMin) used in exponential sweep calculation
    // This determines the rate of exponential frequency increase
    float B;

    // Current instantaneous waveform frequency in rad/s - updated each call to update()
    // Tracks the time-varying frequency during the exponential sweep phase
    float waveform_freq_rads;

    // Current amplitude window value (0 to 1) - applies fade-in/fade-out envelope
    // Computed from Hann window function during fade periods, equals 1.0 during sweep
    float window;

    // Most recent chirp signal output value at the requested time instant
    // Calculated as: window * magnitude * sin(phase), where phase ensures frequency continuity
    float output;

    // Completion flag: true when time >= record duration, false otherwise
    // Reset to false on init() to allow chirp reuse
    bool complete;

};
