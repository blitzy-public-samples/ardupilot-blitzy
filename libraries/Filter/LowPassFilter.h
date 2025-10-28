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
 * @file LowPassFilter.h
 * @brief First-order (one-pole) low-pass filter implementation with constant or variable time step support
 * 
 * @details This file provides two primary filter classes for different use cases:
 * 
 * **LowPassFilter** - Variable dt (time step) version:
 * - Recalculates filter coefficient (alpha) for each sample based on actual time step
 * - Use when sample timing varies or for sporadic updates
 * - Less CPU efficient but more accurate for variable timing
 * - Usage pattern:
 * @code
 *   LowPassFilterFloat filter;
 *   filter.set_cutoff_frequency(cutoff_hz);
 *   // then on each sample:
 *   output = filter.apply(sample, dt_seconds);
 * @endcode
 * 
 * **LowPassFilterConstDt** - Constant dt version:
 * - Pre-computes alpha coefficient once during initialization
 * - Assumes uniform time steps (e.g., fixed main loop at 400Hz)
 * - More CPU efficient, recommended when sample rate is fixed
 * - Usage pattern:
 * @code
 *   LowPassFilterConstDtFloat filter;
 *   filter.set_cutoff_frequency(sample_freq_hz, cutoff_freq_hz);
 *   // then on each sample:
 *   output = filter.apply(sample);
 * @endcode
 * 
 * **Mathematical Foundation**:
 * - Implements first-order RC filter discrete approximation
 * - Transfer function: H(s) = 1 / (1 + s/(2*pi*fc)) where fc = cutoff frequency
 * - Discrete implementation: output = alpha * sample + (1-alpha) * previous_output
 * - Alpha coefficient: alpha = dt / (dt + 1/(2*pi*cutoff_freq))
 * - Single exponential smoothing with 6 dB/octave (20 dB/decade) rolloff
 * 
 * **Filter Characteristics**:
 * - No overshoot, monotonic step response
 * - Group delay approximately 1/(2*pi*cutoff_freq) seconds
 * - -3dB attenuation at cutoff frequency
 * - Phase lag increases with frequency
 * 
 * **Template Support**:
 * - Supports float, Vector2f, Vector3f, and int types
 * - Operates element-wise for vector types
 * 
 * **Units**:
 * - Frequencies: Hz
 * - Time step (dt): seconds
 * - Alpha coefficient: dimensionless [0, 1]
 * 
 * Source: libraries/Filter/LowPassFilter.h:1-132
 */

#pragma once

#include <AP_Math/AP_Math.h>

/**
 * @class DigitalLPF
 * @brief Core first-order low-pass filter implementation with exponential smoothing
 * 
 * @details This class provides the fundamental mathematical implementation of a
 * digital low-pass filter using exponential smoothing (single-pole IIR filter).
 * 
 * **Algorithm**:
 * @code
 *   output = alpha * sample + (1 - alpha) * previous_output
 * @endcode
 * 
 * where:
 * - alpha = smoothing factor in range [0, 1]
 * - alpha = dt / (dt + 1/(2*pi*cutoff_freq))
 * - Higher alpha = more responsive (less filtering)
 * - Lower alpha = more smoothing (more filtering)
 * 
 * **Lazy Initialization**:
 * - Filter initializes to the first sample value automatically
 * - Prevents startup transients from zero initialization
 * - Call reset() to re-initialize to next sample or reset(value) for specific value
 * 
 * **Template Parameter**:
 * @tparam T Data type: float, Vector2f, Vector3f, or int
 *           For vector types, filtering operates element-wise
 * 
 * **Thread Safety**:
 * Not thread-safe. Caller must ensure serialized access if used across threads.
 * 
 * @note This is a base class. Use LowPassFilter or LowPassFilterConstDt for
 *       actual filter instances.
 */
template <class T>
class DigitalLPF {
public:

    /**
     * @brief Default constructor
     * 
     * @details Initializes filter in uninitialized state. Filter will automatically
     *          initialize to the first sample value on first call to _apply().
     */
    DigitalLPF();

    CLASS_NO_COPY(DigitalLPF);

    /**
     * @brief Get latest filtered value from filter
     * 
     * @return Filtered output value (equal to value returned by latest call to _apply())
     * 
     * @note Returns uninitialized value if filter has not processed any samples yet
     */
    const T &get() const;

    /**
     * @brief Reset filter to a specific value
     * 
     * @param[in] value Value to reset filter output to
     * 
     * @details Sets the internal filter state to the specified value and marks
     *          filter as initialized. Next sample will be filtered from this value.
     */
    void reset(const T &value);

    /**
     * @brief Set reset flag to initialize filter to next applied sample
     * 
     * @details Marks filter as uninitialized. On next call to _apply(), the filter
     *          will initialize its output to the input sample value, providing
     *          smooth startup without transients.
     */
    void reset();

protected:
    /**
     * @brief Apply filter to new sample with given smoothing coefficient
     * 
     * @param[in] sample    Input sample value to filter
     * @param[in] alpha     Smoothing coefficient in range [0, 1]
     *                      - 0 = maximum smoothing (no change from previous output)
     *                      - 1 = no smoothing (output equals input)
     *                      - Typical range: 0.01 to 0.5
     * 
     * @return Filtered output value
     * 
     * @details Implements exponential smoothing algorithm:
     *          - If uninitialized: output = sample (first sample initialization)
     *          - Otherwise: output = alpha * sample + (1-alpha) * previous_output
     * 
     * @warning Alpha must be in range [0, 1]. Values outside this range will
     *          produce unstable filter behavior.
     */
    T _apply(const T &sample, const float &alpha);

private:
    T output;
    bool initialised;
};

/**
 * @class LowPassFilterConstDt
 * @brief Constant-dt low-pass filter, more CPU efficient for fixed sample rates
 * 
 * @details This filter variant assumes uniform time steps between samples,
 * allowing the alpha coefficient to be pre-computed once in set_cutoff_frequency().
 * This makes it more CPU efficient than the variable-dt version, as alpha calculation
 * is performed only when cutoff frequency changes rather than on every sample.
 * 
 * **Use Cases**:
 * - Fixed sample rate (e.g., main loop at 400Hz, sensor updates at 1kHz)
 * - Consistent timing between filter updates
 * - CPU-constrained applications requiring efficiency
 * 
 * **Filter Characteristics**:
 * - 20 dB/decade rolloff above cutoff frequency
 * - -3dB attenuation at cutoff frequency
 * - Phase delay approximately 1/(2*pi*cutoff_freq) seconds
 * 
 * **Alpha Calculation** (performed once in set_cutoff_frequency()):
 * @code
 *   dt = 1 / sample_freq
 *   alpha = dt / (dt + 1/(2*pi*cutoff_freq))
 * @endcode
 * 
 * **Typical Usage**:
 * @code
 *   LowPassFilterConstDtFloat gyro_filter;
 *   gyro_filter.set_cutoff_frequency(400.0f, 20.0f);  // 400Hz sample, 20Hz cutoff
 *   
 *   // In main loop (called at 400Hz):
 *   filtered_gyro = gyro_filter.apply(raw_gyro_sample);
 * @endcode
 * 
 * @tparam T Data type: float, Vector2f, Vector3f, or int
 * 
 * @warning Sample frequency must accurately represent actual sampling rate.
 *          Inconsistent timing will introduce frequency response errors.
 * 
 * @warning Cutoff frequency should be well below Nyquist frequency (sample_freq/2)
 *          for best results. Recommended: cutoff_freq < sample_freq/5
 */
template <class T>
class LowPassFilterConstDt : public DigitalLPF<T> {
public:

    /**
     * @brief Default constructor
     * 
     * @details Initializes filter with no cutoff frequency set.
     *          Must call set_cutoff_frequency() before use.
     */
    LowPassFilterConstDt() {};
    
    /**
     * @brief Constructor with frequency initialization
     * 
     * @param[in] sample_freq  Sample frequency in Hz (rate at which apply() will be called)
     * @param[in] cutoff_freq  Cutoff frequency in Hz (-3dB point)
     * 
     * @details Initializes filter and pre-computes alpha coefficient.
     *          Filter is ready to use immediately via apply().
     */
    LowPassFilterConstDt(const float &sample_freq, const float &cutoff_freq);

    CLASS_NO_COPY(LowPassFilterConstDt);

    /**
     * @brief Set or change filter cutoff frequency
     * 
     * @param[in] sample_freq  Sample frequency in Hz (rate at which apply() will be called)
     * @param[in] cutoff_freq  Cutoff frequency in Hz (-3dB point)
     * 
     * @details Computes and stores the alpha coefficient based on the sample and
     *          cutoff frequencies. This computation is performed once here rather
     *          than on every sample for efficiency.
     * 
     *          Alpha calculation: alpha = dt / (dt + 1/(2*pi*cutoff_freq))
     *          where dt = 1/sample_freq
     * 
     * @note Changing cutoff frequency does NOT reset the filter state.
     *       Call reset() if you want to clear filter history.
     * 
     * @warning sample_freq must be positive and should be >> cutoff_freq
     *          Recommended: sample_freq >= 5 * cutoff_freq
     */
    void set_cutoff_frequency(const float &sample_freq, const float &cutoff_freq);

    /**
     * @brief Get current cutoff frequency
     * 
     * @return Cutoff frequency in Hz
     */
    float get_cutoff_freq() const;

    /**
     * @brief Apply filter to new sample
     * 
     * @param[in] sample  Input sample value to filter
     * 
     * @return Filtered output value
     * 
     * @details Applies exponential smoothing using pre-computed alpha coefficient:
     *          output = alpha * sample + (1-alpha) * previous_output
     * 
     *          On first call, initializes filter output to input sample value.
     * 
     * @note This method assumes constant time step. Call apply() at consistent
     *       rate matching sample_freq specified in set_cutoff_frequency().
     */
    T apply(const T &sample);

private:
    float cutoff_freq;
    float alpha;
};

/// @typedef LowPassFilterConstDtFloat
/// @brief Constant-dt low-pass filter for scalar float values
typedef LowPassFilterConstDt<float>    LowPassFilterConstDtFloat;

/// @typedef LowPassFilterConstDtVector2f
/// @brief Constant-dt low-pass filter for 2D vector values (element-wise filtering)
typedef LowPassFilterConstDt<Vector2f> LowPassFilterConstDtVector2f;

/// @typedef LowPassFilterConstDtVector3f
/// @brief Constant-dt low-pass filter for 3D vector values (element-wise filtering)
typedef LowPassFilterConstDt<Vector3f> LowPassFilterConstDtVector3f;

/**
 * @class LowPassFilter
 * @brief Variable-dt low-pass filter, handles non-uniform sample rates accurately
 * 
 * @details This filter variant recalculates the alpha coefficient for each sample
 * based on the actual time step (dt) provided. This makes it more accurate for
 * non-uniform sample timing but less CPU efficient than the constant-dt version.
 * 
 * **Use Cases**:
 * - Variable or irregular sample rates
 * - Sporadic sensor updates (e.g., GPS at 5-10Hz with jitter)
 * - Filters shared across different timing domains
 * - Applications where timing accuracy is critical
 * 
 * **Filter Characteristics**:
 * - Adapts to actual time between samples
 * - Maintains consistent frequency response regardless of sample timing
 * - 20 dB/decade rolloff above cutoff frequency
 * - -3dB attenuation at cutoff frequency
 * 
 * **Alpha Calculation** (performed on each apply() call):
 * @code
 *   alpha = dt / (dt + 1/(2*pi*cutoff_freq))
 * @endcode
 * 
 * **Typical Usage**:
 * @code
 *   LowPassFilterFloat gps_velocity_filter;
 *   gps_velocity_filter.set_cutoff_frequency(2.0f);  // 2Hz cutoff
 *   
 *   // When new GPS data arrives (variable timing):
 *   float dt = (current_time_us - last_time_us) * 1.0e-6f;  // Convert to seconds
 *   filtered_velocity = gps_velocity_filter.apply(raw_velocity, dt);
 * @endcode
 * 
 * **Performance Consideration**:
 * Each apply() call performs:
 * - One division: 1/(2*pi*cutoff_freq)
 * - One addition: dt + 1/(2*pi*cutoff_freq)
 * - One division: dt / (...)
 * 
 * Use LowPassFilterConstDt if sample rate is constant for better efficiency.
 * 
 * @tparam T Data type: float, Vector2f, Vector3f, or int
 * 
 * @warning dt must be positive and reasonable (typically 0.0001 to 1.0 seconds).
 *          Very small dt (< 1e-6) may cause numerical precision issues.
 *          Very large dt (> 1.0) indicates missed samples or timing errors.
 * 
 * @warning Phase delay introduces approximately 1/(2*pi*cutoff_freq) seconds
 *          of group delay. Account for this in time-critical applications.
 */
template <class T>
class LowPassFilter : public DigitalLPF<T> {
public:

    /**
     * @brief Default constructor
     * 
     * @details Initializes filter with no cutoff frequency set.
     *          Must call set_cutoff_frequency() before use.
     */
    LowPassFilter() {};
    
    /**
     * @brief Constructor with cutoff frequency initialization
     * 
     * @param[in] cutoff_freq  Cutoff frequency in Hz (-3dB point)
     * 
     * @details Initializes filter with specified cutoff frequency.
     *          Filter is ready to use immediately via apply(sample, dt).
     */
    LowPassFilter(const float &cutoff_freq);

    CLASS_NO_COPY(LowPassFilter);

    /**
     * @brief Set or change filter cutoff frequency
     * 
     * @param[in] cutoff_freq  Cutoff frequency in Hz (-3dB point)
     * 
     * @details Sets the cutoff frequency used for alpha coefficient calculation
     *          on each apply() call. Unlike LowPassFilterConstDt, no pre-computation
     *          is performed here - alpha is calculated dynamically based on actual dt.
     * 
     * @note Changing cutoff frequency does NOT reset the filter state.
     *       Call reset() if you want to clear filter history.
     */
    void set_cutoff_frequency(const float &cutoff_freq);

    /**
     * @brief Get current cutoff frequency
     * 
     * @return Cutoff frequency in Hz
     */
    float get_cutoff_freq() const;

    /**
     * @brief Apply filter to new sample with specified time step
     * 
     * @param[in] sample  Input sample value to filter
     * @param[in] dt      Time step since last sample in seconds
     *                    Typical range: 0.001 to 0.1 seconds
     * 
     * @return Filtered output value
     * 
     * @details Calculates alpha based on actual dt, then applies exponential smoothing:
     *          - alpha = dt / (dt + 1/(2*pi*cutoff_freq))
     *          - output = alpha * sample + (1-alpha) * previous_output
     * 
     *          On first call, initializes filter output to input sample value.
     * 
     * @warning dt must be positive. Negative or zero dt will produce invalid results.
     *          Very small dt (< 1e-6 seconds) may cause numerical precision issues.
     * 
     * @warning For best anti-aliasing, ensure cutoff_freq < 1/(2*dt) (Nyquist frequency).
     *          Recommended: cutoff_freq < 1/(5*dt)
     * 
     * @note CPU cost: ~3-4 floating point operations per call (division, addition, multiply)
     *       compared to ~2 operations for LowPassFilterConstDt.
     */
    T apply(const T &sample, const float &dt);

private:
    float cutoff_freq;
};

/// @typedef LowPassFilterFloat
/// @brief Variable-dt low-pass filter for scalar float values
typedef LowPassFilter<float>    LowPassFilterFloat;

/// @typedef LowPassFilterVector2f
/// @brief Variable-dt low-pass filter for 2D vector values (element-wise filtering)
typedef LowPassFilter<Vector2f> LowPassFilterVector2f;

/// @typedef LowPassFilterVector3f
/// @brief Variable-dt low-pass filter for 3D vector values (element-wise filtering)
typedef LowPassFilter<Vector3f> LowPassFilterVector3f;
