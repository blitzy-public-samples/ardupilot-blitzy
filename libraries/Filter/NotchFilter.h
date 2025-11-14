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

/**
 * @file NotchFilter.h
 * @brief Configurable notch filter with settable sample rate, center frequency, bandwidth and attenuation
 * 
 * @details This file implements a digital biquad IIR notch filter designed to reject narrow-band
 *          noise at a specific frequency while passing all other frequencies with minimal attenuation.
 *          The filter is commonly used for rejecting motor noise, propeller harmonics, or other
 *          periodic disturbances in sensor data.
 * 
 *          Design by Leonard Hall
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>
#include <AP_Param/AP_Param.h>


template <class T>
class HarmonicNotchFilter;

/**
 * @class NotchFilter
 * @brief Biquad notch filter for rejecting narrow-band noise at a specific frequency
 * 
 * @details This class implements a second-order IIR (Infinite Impulse Response) notch filter
 *          using the biquad difference equation structure. The filter creates a deep notch
 *          (attenuation) at the center frequency while minimally affecting adjacent frequencies.
 * 
 *          **Algorithm**: Digital biquad IIR notch filter with configurable quality factor (Q)
 *          and attenuation (A). The filter places zeros at the center frequency to create the
 *          notch characteristic.
 * 
 *          **Transfer Function**: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 *          The coefficients are calculated such that zeros are placed at the center frequency.
 * 
 *          **Mathematical Foundation**: The filter implements the biquad difference equation:
 *          y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 *          where x[n] is the input signal and y[n] is the output signal.
 * 
 *          **Filter State Variables**:
 *          - ntchsig1, ntchsig2: Input signal history (x[n-1], x[n-2])
 *          - signal1, signal2: Output signal history (y[n-1], y[n-2])
 * 
 *          **Smooth Frequency Transitions**: Supports slew rate limiting for smooth center
 *          frequency transitions using the need_reset flag. This prevents discontinuities
 *          when the center frequency is changed during operation.
 * 
 *          **Relationship**: bandwidth_hz = center_freq_hz / Q
 *          Higher Q means narrower bandwidth (more selective notch).
 * 
 * @tparam T Template parameter supporting float, Vector2f, or Vector3f for scalar or
 *           multi-dimensional filtering
 * 
 * @note Available typedefs: NotchFilterFloat, NotchFilterVector2f, NotchFilterVector3f
 * 
 * @warning Center frequency must be less than the Nyquist frequency (sample_freq_hz / 2)
 *          to avoid aliasing and filter instability.
 * 
 * @warning Very high Q factors (narrow bandwidth) can cause numerical instability and
 *          increased sensitivity to coefficient quantization errors. Typical Q values
 *          are in the range 1.0 to 50.0.
 * 
 * @warning Notch filters introduce phase delay near the center frequency. This phase
 *          shift should be considered in control applications where phase margin is critical.
 */
template <class T>
class NotchFilter {
public:
    friend class HarmonicNotchFilter<T>;
    
    /**
     * @brief Initialize notch filter with frequency, bandwidth, and attenuation parameters
     * 
     * @details Calculates and sets the biquad filter coefficients (b0, b1, b2, a1, a2) based
     *          on the provided center frequency, bandwidth, and desired attenuation. This method
     *          first computes the intermediate parameters A (linear attenuation) and Q (quality
     *          factor) using calculate_A_and_Q(), then calls init_with_A_and_Q() to set the
     *          final coefficients.
     * 
     * @param[in] sample_freq_hz Sample frequency in Hertz (Hz) - the rate at which apply() will be called
     * @param[in] center_freq_hz Center/notch frequency in Hertz (Hz) - the frequency to attenuate.
     *                           Must be less than sample_freq_hz/2 (Nyquist frequency)
     * @param[in] bandwidth_hz   Bandwidth in Hertz (Hz) - the width of the notch. Smaller bandwidth
     *                           creates a narrower, more selective notch. Typical values: 5-50 Hz
     * @param[in] attenuation_dB Attenuation at center frequency in decibels (dB) - depth of the notch.
     *                           Typical values: 20-60 dB. Higher values create deeper attenuation
     * 
     * @note All frequency units are in Hz, attenuation in dB
     * @warning Must be called before apply() to initialize filter coefficients
     */
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
    
    /**
     * @brief Initialize notch filter with direct A and Q parameters
     * 
     * @details Directly calculates and sets the biquad filter coefficients using the provided
     *          attenuation factor (A) and quality factor (Q). This method provides lower-level
     *          control compared to init() and is typically used when A and Q are already known
     *          or have been pre-calculated.
     * 
     *          The filter coefficients are computed using the bilinear transform method with
     *          frequency pre-warping to maintain accurate center frequency placement.
     * 
     * @param[in] sample_freq_hz Sample frequency in Hertz (Hz) - the rate at which apply() will be called
     * @param[in] center_freq_hz Center/notch frequency in Hertz (Hz) - the frequency to attenuate.
     *                           Must be less than sample_freq_hz/2 (Nyquist frequency)
     * @param[in] A              Attenuation factor (linear scale, not dB). A = 10^(attenuation_dB/40).
     *                           Typical range: 0.01 to 0.1 for 40-20 dB attenuation
     * @param[in] Q              Quality factor (dimensionless) - controls bandwidth. Q = center_freq_hz / bandwidth_hz.
     *                           Higher Q means narrower notch. Typical range: 1.0 to 50.0
     * 
     * @note A is linear attenuation factor, Q is dimensionless
     * @warning Very high Q values (>50) may cause numerical instability
     */
    void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q);
    
    /**
     * @brief Apply the notch filter to an input sample
     * 
     * @details Processes one input sample through the biquad notch filter using the difference
     *          equation: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
     * 
     *          The method maintains internal state (ntchsig1, ntchsig2, signal1, signal2) to
     *          implement the recursive filter structure. If the filter is not initialized
     *          (initialised == false), the input is returned unchanged.
     * 
     * @param[in] sample Input signal sample to be filtered. Type T can be float, Vector2f, or Vector3f
     * 
     * @return Filtered output sample with notch applied at the center frequency. Returns input
     *         unchanged if filter is not initialized
     * 
     * @note This method should be called at the sample rate specified in init() for correct
     *       frequency response
     * @note For Vector2f/Vector3f types, filtering is applied independently to each component
     */
    T apply(const T &sample);
    
    /**
     * @brief Reset the filter state to zero
     * 
     * @details Clears all internal filter state variables (ntchsig1, ntchsig2, signal1, signal2)
     *          to zero, effectively restarting the filter with no history. This is useful when
     *          starting filtering on a new data stream or after a discontinuity in the input signal.
     * 
     *          The filter coefficients and configuration are preserved; only the signal history
     *          is cleared.
     * 
     * @note Does not change the initialized state or filter parameters
     * @note Calling reset() causes a transient response for the next few samples
     */
    void reset();
    /**
     * @brief Get the current center frequency of the notch filter
     * 
     * @return Current center/notch frequency in Hertz (Hz)
     * 
     * @note This returns the frequency that was set during initialization or last update
     */
    float center_freq_hz() const { return _center_freq_hz; }
    
    /**
     * @brief Get the sample frequency of the notch filter
     * 
     * @return Sample frequency in Hertz (Hz) at which apply() should be called
     * 
     * @note This returns the sample rate that was set during initialization
     */
    float sample_freq_hz() const { return _sample_freq_hz; }

    /**
     * @brief Calculate linear attenuation (A) and quality factor (Q) from frequency and bandwidth parameters
     * 
     * @details Static helper function that computes the intermediate filter design parameters A and Q
     *          from the more intuitive frequency-domain specifications (center frequency, bandwidth,
     *          and desired attenuation in dB). These A and Q values can then be used with
     *          init_with_A_and_Q() for filter initialization.
     * 
     *          Calculations:
     *          - A = 10^(attenuation_dB / 40)  [linear attenuation factor]
     *          - Q = center_freq_hz / bandwidth_hz  [quality factor]
     * 
     * @param[in]  center_freq_hz Center/notch frequency in Hertz (Hz)
     * @param[in]  bandwidth_hz   Bandwidth in Hertz (Hz) - width of the notch
     * @param[in]  attenuation_dB Attenuation at center frequency in decibels (dB)
     * @param[out] A              Computed linear attenuation factor (dimensionless)
     * @param[out] Q              Computed quality factor (dimensionless)
     * 
     * @note This is a static method and can be called without a NotchFilter instance
     * @note Useful for pre-computing filter parameters or analyzing filter characteristics
     */
    static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q); 

    /**
     * @brief Disable the notch filter
     * 
     * @details Marks the filter as uninitialized, which causes apply() to pass input samples
     *          through unchanged (bypass mode). The filter coefficients and state are preserved
     *          and filtering can be re-enabled by calling init() again.
     * 
     * @note After calling disable(), apply() will return input unchanged until init() is called again
     */
    void disable(void) {
        initialised = false;
    }

    /**
     * @brief Get the frequency to log for telemetry and analysis
     * 
     * @details Returns the appropriate frequency value to be logged for telemetry purposes.
     *          This is useful for tracking the filter's operating point in flight logs and
     *          for post-flight analysis of filter performance.
     * 
     * @return Frequency in Hertz (Hz) representing the current notch center frequency for logging
     * 
     * @note Used by logging subsystems to record filter configuration in flight logs
     */
    float logging_frequency(void) const;

protected:

    bool initialised, need_reset;
    float b0, b1, b2, a1, a2;
    float _center_freq_hz, _sample_freq_hz, _A;
    T ntchsig1, ntchsig2, signal2, signal1;
};

/**
 * @class NotchFilterParams
 * @brief Parameter container for notch filter configuration and enable/disable control
 * 
 * @details This class provides a parameter storage interface for notch filter configuration,
 *          integrating with the ArduPilot parameter system (AP_Param). It stores the filter
 *          enable flag, center frequency, bandwidth, and attenuation as persistent parameters
 *          that can be configured through ground control stations.
 * 
 *          The class provides read-only accessor methods for filter parameters and an enable
 *          control method. Parameters are stored as AP_Int8 and AP_Float types for persistence
 *          across reboots.
 * 
 * @note This class is typically used as a base class or member in vehicle-specific parameter groups
 * @note Parameter values can be modified through the ArduPilot parameter system
 */
class NotchFilterParams {
public:
    /**
     * @brief Get the configured center frequency parameter
     * @return Center frequency in Hertz (Hz) from stored parameter
     */
    float center_freq_hz(void) const { return _center_freq_hz; }
    
    /**
     * @brief Get the configured bandwidth parameter
     * @return Bandwidth in Hertz (Hz) from stored parameter
     */
    float bandwidth_hz(void) const { return _bandwidth_hz; }
    
    /**
     * @brief Get the configured attenuation parameter
     * @return Attenuation in decibels (dB) from stored parameter
     */
    float attenuation_dB(void) const { return _attenuation_dB; }
    
    /**
     * @brief Check if the notch filter is enabled
     * @return 1 if enabled, 0 if disabled
     */
    uint8_t enabled(void) const { return _enable; }
    
    /**
     * @brief Enable the notch filter by setting the enable parameter
     * @note This sets the persistent enable parameter to true
     */
    void enable() { _enable.set(true); }
    
protected:
    AP_Int8 _enable;
    AP_Float _center_freq_hz;
    AP_Float _bandwidth_hz;
    AP_Float _attenuation_dB;
};

/**
 * @typedef NotchFilterFloat
 * @brief Notch filter for scalar float values
 * @details Convenience typedef for NotchFilter<float> for filtering single-channel signals
 */
typedef NotchFilter<float> NotchFilterFloat;

/**
 * @typedef NotchFilterVector2f
 * @brief Notch filter for 2D vector values
 * @details Convenience typedef for NotchFilter<Vector2f> for filtering two-dimensional signals.
 *          Each vector component is filtered independently with the same filter coefficients.
 */
typedef NotchFilter<Vector2f> NotchFilterVector2f;

/**
 * @typedef NotchFilterVector3f
 * @brief Notch filter for 3D vector values
 * @details Convenience typedef for NotchFilter<Vector3f> for filtering three-dimensional signals
 *          such as accelerometer or gyroscope data. Each vector component (x, y, z) is filtered
 *          independently with the same filter coefficients.
 */
typedef NotchFilter<Vector3f> NotchFilterVector3f;

