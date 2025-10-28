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
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>


/**
 * @file   LowPassFilter2p.h
 * @brief  Second-order (two-pole) low-pass filter using digital biquad structure
 * 
 * @details This file implements a templated second-order low-pass filter using
 *          a digital biquad (biquadratic) IIR filter structure. The filter provides
 *          40 dB/decade (12 dB/octave) rolloff above the cutoff frequency, offering
 *          better attenuation than first-order filters at the cost of increased
 *          phase delay. The implementation uses a direct form II biquad structure
 *          with two delay elements for efficient computation.
 *          
 *          Mathematical Foundation: The filter is derived from a continuous-time
 *          Butterworth low-pass prototype using the bilinear transform (tustin method).
 *          Butterworth response provides maximally flat passband response with
 *          Q=0.707 for critical damping in second-order configuration.
 *          
 *          Transfer Function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 * 
 * @authors Leonard Hall <LeonardTHall@gmail.com>, template implementation: Daniel Frenzel <dgdanielf@gmail.com>
 */

/**
 * @class DigitalBiquadFilter
 * @brief Core biquad IIR filter implementation with direct form II structure
 * 
 * @details This template class implements a general-purpose digital biquad filter
 *          using the direct form II structure. A biquad filter is a second-order
 *          recursive (IIR) filter characterized by the difference equation:
 *          
 *          y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *          
 *          The filter maintains two internal delay elements (_delay_element_1,
 *          _delay_element_2) that store previous state for the recursive computation.
 *          This direct form II implementation is computationally efficient and
 *          numerically stable for most filter designs.
 *          
 *          The template parameter T allows filtering of various data types including
 *          scalar types (int, long, float) and vector types (Vector2f, Vector3f),
 *          enabling component-wise filtering of multi-dimensional signals.
 *          
 *          Algorithm: Direct Form II biquad structure
 *          - Maintains two delay elements for previous state
 *          - Computes output using biquad difference equation
 *          - Supports reset to zero or to steady-state for given DC input
 * 
 * @tparam T Data type to filter (int, long, float, Vector2f, Vector3f)
 * 
 * @warning Second-order filters are more sensitive to coefficient quantization
 *          than first-order filters. Ensure sufficient numerical precision.
 * 
 * Source: libraries/Filter/LowPassFilter2p.h:26-51
 */
template <class T>
class DigitalBiquadFilter {
public:
    /**
     * @struct biquad_params
     * @brief Biquad filter coefficient structure
     * 
     * @details Contains the filter coefficients and frequency parameters for a
     *          biquad filter. The coefficients define the transfer function:
     *          H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
     *          
     *          All frequencies are specified in Hz. The cutoff frequency must be
     *          less than the Nyquist frequency (sample_freq/2) for valid filter design.
     */
    struct biquad_params {
        float cutoff_freq;  ///< Cutoff frequency in Hz (-3dB point for low-pass configuration)
        float sample_freq;  ///< Sampling frequency in Hz
        float a1;           ///< Denominator coefficient for z^-1 term (recursive/feedback)
        float a2;           ///< Denominator coefficient for z^-2 term (recursive/feedback)
        float b0;           ///< Numerator coefficient for z^0 term (feedforward)
        float b1;           ///< Numerator coefficient for z^-1 term (feedforward)
        float b2;           ///< Numerator coefficient for z^-2 term (feedforward)
    };

    CLASS_NO_COPY(DigitalBiquadFilter);

    /**
     * @brief Default constructor
     * 
     * @details Initializes the filter with zero delay elements and sets
     *          initialised flag to false. The filter is not ready for use
     *          until apply() is called for the first time or reset() is used
     *          to establish initial conditions.
     */
    DigitalBiquadFilter();

    /**
     * @brief Apply filter to input sample
     * 
     * @details Computes the biquad filter output using the direct form II
     *          difference equation. On first call (initialised==false), the
     *          delay elements are initialized to zero and the sample is passed
     *          through unfiltered to avoid transient startup behavior.
     *          
     *          Subsequent calls apply the full biquad equation:
     *          y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
     * 
     * @param[in] sample Input sample to be filtered
     * @param[in] params Biquad filter coefficients and frequency parameters
     * @return Filtered output value
     * 
     * @note This method is typically called at a regular sample rate matching params.sample_freq
     */
    T apply(const T &sample, const struct biquad_params &params);
    
    /**
     * @brief Reset filter state to zero
     * 
     * @details Clears both internal delay elements to zero and sets the
     *          initialised flag to false. The next call to apply() will
     *          initialize the filter from zero initial conditions.
     *          
     *          Use this method to reset the filter when starting a new
     *          filtering operation or when discontinuities in the input
     *          signal make previous state invalid.
     */
    void reset();
    
    /**
     * @brief Reset filter to steady-state for given DC input
     * 
     * @details Initializes the filter delay elements to the steady-state values
     *          corresponding to a constant DC input. This prevents transient
     *          response when the filter starts with a known initial value.
     *          
     *          The delay elements are computed to satisfy the steady-state
     *          condition where output equals input for a constant signal.
     * 
     * @param[in] value DC value to initialize filter state to
     * @param[in] params Biquad filter coefficients used to compute steady-state
     * 
     * @note This is useful when resuming filtering after a known stable value
     */
    void reset(const T &value, const struct biquad_params &params);
    
    /**
     * @brief Compute biquad coefficients for low-pass filter design
     * 
     * @details Static helper function that computes biquad filter coefficients
     *          for a second-order Butterworth low-pass filter using the bilinear
     *          transform (tustin approximation). The resulting filter has:
     *          - Maximally flat passband response (Butterworth)
     *          - -3dB attenuation at cutoff frequency
     *          - 40 dB/decade (12 dB/octave) rolloff above cutoff
     *          - Q factor of 0.707 (critical damping)
     * 
     * @param[in]  sample_freq Sampling frequency in Hz
     * @param[in]  cutoff_freq Desired cutoff frequency in Hz (-3dB point)
     * @param[out] ret         Computed biquad parameters (coefficients a1, a2, b0, b1, b2)
     * 
     * @warning cutoff_freq must be less than Nyquist frequency (sample_freq/2)
     * @warning No validation is performed on input frequencies
     * 
     * @note This is a static method and can be called without a filter instance
     */
    static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);
    
private:
    T _delay_element_1;
    T _delay_element_2;
    bool initialised;
};

/**
 * @class LowPassFilter2p
 * @brief User-friendly wrapper for DigitalBiquadFilter configured as low-pass filter
 * 
 * @details This template class provides a simplified interface for using a second-order
 *          low-pass filter. It wraps the DigitalBiquadFilter class and manages the
 *          biquad parameter computation internally, allowing users to specify only
 *          the sample frequency and cutoff frequency.
 *          
 *          Filter Characteristics:
 *          - Second-order Butterworth low-pass response
 *          - 40 dB/decade (12 dB/octave) rolloff above cutoff frequency
 *          - Maximally flat passband (Butterworth response)
 *          - -3dB attenuation at cutoff frequency
 *          - Q factor of 0.707 (critical damping)
 *          - Better attenuation than first-order LowPassFilter but more phase delay
 *          
 *          Phase Delay: The filter introduces approximately 1/(2*pi*cutoff_freq)
 *          seconds of phase delay. For example, a 10 Hz cutoff has ~16 ms delay.
 *          
 *          Usage Pattern:
 *          1. Construct filter with sample and cutoff frequencies
 *          2. Call apply() for each new sample at regular sample rate
 *          3. Optionally call reset() when starting new data stream
 *          4. Optionally call set_cutoff_frequency() to change filter response
 *          
 *          The template parameter T allows filtering of various data types including
 *          scalar types (int, long, float) and vector types (Vector2f, Vector3f).
 * 
 * @tparam T Data type to filter (int, long, float, Vector2f, Vector3f)
 * 
 * @warning cutoff_freq must be less than Nyquist frequency (sample_freq/2) to avoid aliasing
 * @warning Introduces approximately 1/(2*pi*cutoff_freq) seconds of phase delay
 * @warning Second-order filters more sensitive to coefficient quantization than first-order
 * 
 * @note Typedefs provided: LowPassFilter2pInt, LowPassFilter2pLong, LowPassFilter2pFloat,
 *       LowPassFilter2pVector2f, LowPassFilter2pVector3f
 * 
 * Source: libraries/Filter/LowPassFilter2p.h:54-75
 */
template <class T>
class LowPassFilter2p {
public:
    /**
     * @brief Default constructor
     * 
     * @details Initializes the filter with zero parameters. The filter is not
     *          ready for use until set_cutoff_frequency() is called to establish
     *          valid filter coefficients.
     */
    LowPassFilter2p();
    
    /**
     * @brief Constructor with frequency initialization
     * 
     * @details Constructs the filter and immediately computes biquad coefficients
     *          for the specified sample and cutoff frequencies. The filter is
     *          ready for use immediately after construction.
     * 
     * @param[in] sample_freq Sampling frequency in Hz (rate at which apply() will be called)
     * @param[in] cutoff_freq Desired cutoff frequency in Hz (-3dB point)
     * 
     * @warning cutoff_freq must be less than Nyquist frequency (sample_freq/2)
     * 
     * @note All frequencies in Hz
     */
    LowPassFilter2p(float sample_freq, float cutoff_freq);
    
    /**
     * @brief Change filter cutoff frequency
     * 
     * @details Recomputes the biquad filter coefficients for new sample and cutoff
     *          frequencies. This allows dynamic adjustment of filter characteristics
     *          without creating a new filter instance. The internal filter state
     *          (delay elements) is preserved, so the filter continues from its
     *          current state with the new frequency response.
     * 
     * @param[in] sample_freq Sampling frequency in Hz
     * @param[in] cutoff_freq New cutoff frequency in Hz (-3dB point)
     * 
     * @warning cutoff_freq must be less than Nyquist frequency (sample_freq/2)
     * @warning Changing cutoff frequency during operation may cause transient response
     * 
     * @note All frequencies in Hz
     */
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);
    
    /**
     * @brief Get current cutoff frequency
     * 
     * @return Current cutoff frequency in Hz
     * 
     * @note Returns the frequency set by constructor or set_cutoff_frequency()
     */
    float get_cutoff_freq(void) const;
    
    /**
     * @brief Get current sample frequency
     * 
     * @return Current sample frequency in Hz
     * 
     * @note Returns the frequency set by constructor or set_cutoff_frequency()
     */
    float get_sample_freq(void) const;
    
    /**
     * @brief Apply filter to input sample
     * 
     * @details Processes one input sample through the second-order low-pass filter
     *          and returns the filtered output. This method should be called at
     *          a regular rate matching the sample_freq specified during initialization.
     *          
     *          The filter uses internal state from previous samples to compute the
     *          output. On first call, the filter initializes from zero state.
     * 
     * @param[in] sample Input sample to be filtered
     * @return Filtered output value
     * 
     * @note Call this method at the sample rate specified by sample_freq
     * @note First call may show transient behavior as filter initializes
     */
    T apply(const T &sample);
    
    /**
     * @brief Reset filter state to zero
     * 
     * @details Clears the internal filter state (delay elements) to zero. Use this
     *          when starting a new filtering operation or when the input signal has
     *          a discontinuity that makes previous state invalid.
     *          
     *          After reset, the next call to apply() will initialize from zero state.
     */
    void reset(void);
    
    /**
     * @brief Reset filter to steady-state for given DC value
     * 
     * @details Initializes the filter state to the steady-state condition for a
     *          constant DC input value. This prevents transient response when the
     *          filter is known to start at a specific value, such as resuming
     *          filtering after a stable reading.
     *          
     *          The filter state is set such that applying this value repeatedly
     *          would produce the same value as output (steady-state condition).
     * 
     * @param[in] value DC value to initialize filter state to
     * 
     * @note Useful when resuming filtering from a known stable value
     * @note Eliminates startup transient for constant initial conditions
     */
    void reset(const T &value);

    CLASS_NO_COPY(LowPassFilter2p);

protected:
    struct DigitalBiquadFilter<T>::biquad_params _params;
    
private:
    DigitalBiquadFilter<T> _filter;
};

// Uncomment this, if you decide to remove the instantiations in the implementation file
/*
template <class T>
LowPassFilter2p<T>::LowPassFilter2p() { 
    memset(&_params, 0, sizeof(_params) ); 
}

// constructor
template <class T>
LowPassFilter2p<T>::LowPassFilter2p(float sample_freq, float cutoff_freq) {
    // set initial parameters
    set_cutoff_frequency(sample_freq, cutoff_freq);
}
*/

/**
 * @brief Commonly used instantiations of LowPassFilter2p template
 * 
 * @details These typedefs provide convenient names for the most common filter types:
 *          - LowPassFilter2pInt: Integer filtering (quantized signals)
 *          - LowPassFilter2pLong: Long integer filtering (high-resolution quantized signals)
 *          - LowPassFilter2pFloat: Floating-point filtering (most common for sensor data)
 *          - LowPassFilter2pVector2f: 2D vector filtering (component-wise, e.g., optical flow)
 *          - LowPassFilter2pVector3f: 3D vector filtering (component-wise, e.g., accelerometer, gyro)
 *          
 *          Vector filtering applies the same filter independently to each component,
 *          using the same coefficients for all components.
 * 
 * @note These are the recommended types for general use
 */
typedef LowPassFilter2p<int>      LowPassFilter2pInt;      ///< Second-order low-pass filter for integer values
typedef LowPassFilter2p<long>     LowPassFilter2pLong;     ///< Second-order low-pass filter for long integer values
typedef LowPassFilter2p<float>    LowPassFilter2pFloat;    ///< Second-order low-pass filter for floating-point values (most common)
typedef LowPassFilter2p<Vector2f> LowPassFilter2pVector2f; ///< Second-order low-pass filter for 2D vectors (component-wise filtering)
typedef LowPassFilter2p<Vector3f> LowPassFilter2pVector3f; ///< Second-order low-pass filter for 3D vectors (component-wise filtering)
