/**
 * @file Butter.h
 * @brief Second-order Butterworth low-pass filter with pre-computed coefficients
 * 
 * @details This file provides a template-based second-order Butterworth filter implementation
 *          with pre-computed coefficients for common sample rate and cutoff frequency combinations.
 *          Butterworth filters provide maximally flat passband response and are widely used
 *          for sensor data smoothing in ArduPilot.
 * 
 *          The implementation uses a biquad IIR (Infinite Impulse Response) filter structure
 *          with compile-time coefficient selection for efficiency. Coefficients are pre-computed
 *          for sample rates of 100Hz, 50Hz, and 10Hz with various cutoff frequencies.
 * 
 *          Mathematical Foundation: Butterworth analog prototype bilinearly transformed to digital domain
 *          Filter Characteristics: Maximally flat passband, 40 dB/decade rolloff, Q=0.707 (critically damped)
 * 
 * @note All frequencies are in Hz, GAIN coefficients are dimensionless
 * @warning Coefficients are only valid for their specified sample rate - reusing at different rates produces incorrect cutoff
 * 
 * Source: libraries/Filter/Butter.h:1-106
 */

#pragma once

/**
 * @class Butter2
 * @brief Template-based second-order Butterworth low-pass filter using compile-time coefficients
 * 
 * @details This template class implements a second-order Butterworth digital low-pass filter
 *          using a biquad IIR (Infinite Impulse Response) structure. The filter applies a
 *          recursive difference equation using a history buffer to maintain filter state.
 * 
 *          Transfer Function: H(z) = (1 + 2z^-1 + z^-2) / (GAIN + A1*z^-1 + A2*z^-2)
 * 
 *          Algorithm:
 *          - Applies biquad filter: newhist = input + A1*hist[1] + A2*hist[0]
 *          - Computes output: output = (newhist + 2*hist[1] + hist[0]) / GAIN
 *          - Updates history buffer for next iteration
 * 
 *          Filter Properties:
 *          - Maximally flat passband response (Butterworth characteristic)
 *          - Second-order rolloff: 40 dB/decade attenuation in stopband
 *          - Q factor: 0.707 (critically damped, no overshoot)
 *          - Phase delay: approximately 1/(2*pi*cutoff) seconds group delay
 * 
 *          Pre-computed Coefficients:
 *          The template parameter Coefficients provides A1, A2, GAIN as constexpr floats,
 *          eliminating runtime calculation overhead. Coefficients are computed offline using
 *          bilinear transformation of the Butterworth analog prototype.
 * 
 *          History Buffer:
 *          The filter maintains a 2-element history buffer (hist[0], hist[1]) to store
 *          previous intermediate values. The buffer is initialized to zero, which may
 *          cause startup transients. For steady-state initialization, pre-fill with
 *          expected input values.
 * 
 * @tparam Coefficients Struct providing static constexpr float members:
 *                      - A1: First recursive (feedback) coefficient
 *                      - A2: Second recursive (feedback) coefficient  
 *                      - GAIN: DC gain normalization factor
 * 
 * @warning Uninitialized history buffer: Caller must be aware that history buffer initializes
 *          to zero, which may cause transients when filtering non-zero steady-state signals.
 *          Consider pre-initializing history for critical applications.
 * @warning Coefficient validity: Coefficients are only valid for their specified sample rate.
 *          Using coefficients at a different sample rate will produce incorrect cutoff frequency.
 * @warning Phase delay: Second-order Butterworth introduces approximately 1/(2*pi*cutoff) seconds
 *          group delay. Account for this delay in time-critical control loops.
 * 
 * @note This filter is used extensively throughout ArduPilot for sensor smoothing, particularly
 *       for IMU data, barometer readings, and other noisy sensor inputs.
 * 
 * Source: libraries/Filter/Butter.h:3-16
 */
template <typename Coefficients>
class Butter2
{
public:
  /**
   * @brief Apply Butterworth filter to input sample and return filtered output
   * 
   * @details This method applies the second-order Butterworth filter using the biquad
   *          IIR difference equation. The implementation uses the coefficient template
   *          parameters (A1, A2, GAIN) and maintains filter state in the history buffer.
   * 
   *          Algorithm Steps:
   *          1. Compute new history value: newhist = input + A1*hist[1] + A2*hist[0]
   *          2. Compute filtered output: output = (newhist + 2*hist[1] + hist[0]) / GAIN
   *          3. Shift history buffer: hist[0] = hist[1], hist[1] = newhist
   * 
   *          The recursive structure provides memory-efficient filtering with minimal
   *          computational overhead per sample.
   * 
   * @param[in] input New sample value to filter (any floating-point value in appropriate units)
   * 
   * @return Filtered output value in same units as input
   * 
   * @note This method is typically called at the sample rate for which coefficients were designed
   *       (e.g., 100Hz for butter100hz* variants, 50Hz for butter50hz*, 10Hz for butter10hz*)
   * @note Filter state persists between calls via the history buffer
   * 
   * Source: libraries/Filter/Butter.h:7-13
   */
  float filter(float input)
  {
        float newhist = input + Coefficients::A1*hist[1] + Coefficients::A2*hist[0];
        float ret = (newhist + 2*hist[1] + hist[0])/Coefficients::GAIN;
        hist[0] = hist[1]; hist[1] = newhist;
        return ret;
  }
private:
    float hist[2];  ///< History buffer storing previous two intermediate values [hist[0]=oldest, hist[1]=newest]
};

/**
 * @struct butter100_025_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 0.25Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          These coefficients implement a very aggressive low-pass filter suitable for
 *          removing high-frequency noise while preserving very slow trends.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 0.25 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 0.25Hz cutoff (butter100hz0_25)
 *       - 50Hz sample @ 0.125Hz cutoff (butter50hz0_125)
 *       - 10Hz sample @ 0.025Hz cutoff (butter10hz0_025)
 * 
 * Source: libraries/Filter/Butter.h:18-23
 */
struct butter100_025_coeffs
{
  static constexpr float A1 = 1.9777864838f;      ///< First recursive coefficient
  static constexpr float A2 = -0.9780305085f;     ///< Second recursive coefficient
  static constexpr float GAIN = 1.639178228e+04f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz0_25
 * @brief Butterworth filter: 100Hz sample rate, 0.25Hz cutoff frequency
 * 
 * @details Very low cutoff filter suitable for long-term trend extraction.
 *          Provides strong attenuation of signals above 0.25Hz.
 * 
 * Source: libraries/Filter/Butter.h:24
 */
typedef Butter2<butter100_025_coeffs> butter100hz0_25; //100hz sample, 0.25hz fcut

/**
 * @typedef butter50hz0_125
 * @brief Butterworth filter: 50Hz sample rate, 0.125Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/0.125Hz ≡ 100Hz/0.25Hz
 *          Same filter response as butter100hz0_25 but at half the sample rate.
 * 
 * Source: libraries/Filter/Butter.h:25
 */
typedef Butter2<butter100_025_coeffs> butter50hz0_125; //50hz sample, 0.125hz fcut

/**
 * @typedef butter10hz0_025
 * @brief Butterworth filter: 10Hz sample rate, 0.025Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.025Hz ≡ 100Hz/0.25Hz
 *          Same filter response as butter100hz0_25 but at 1/10th the sample rate.
 * 
 * Source: libraries/Filter/Butter.h:26
 */
typedef Butter2<butter100_025_coeffs> butter10hz0_025; //10hz sample, 0.025hz fcut

/**
 * @struct butter100_05_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 0.5Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          Double the cutoff frequency of butter100_025_coeffs, providing less aggressive
 *          filtering while still removing high-frequency noise.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 0.5 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 0.5Hz cutoff (butter100hz0_5)
 *       - 50Hz sample @ 0.25Hz cutoff (butter50hz0_25)
 *       - 10Hz sample @ 0.05Hz cutoff (butter10hz0_05)
 * 
 * Source: libraries/Filter/Butter.h:28-33
 */
struct butter100_05_coeffs
{
  static constexpr float A1 = 1.9555782403f;     ///< First recursive coefficient
  static constexpr float A2 = -0.9565436765f;    ///< Second recursive coefficient
  static constexpr float GAIN = 4.143204922e+03f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz0_5
 * @brief Butterworth filter: 100Hz sample rate, 0.5Hz cutoff frequency
 * 
 * @details Low cutoff filter for slow-varying signals with moderate noise rejection.
 * 
 * Source: libraries/Filter/Butter.h:34
 */
typedef Butter2<butter100_05_coeffs> butter100hz0_5; //100hz sample, 0.5hz fcut

/**
 * @typedef butter50hz0_25
 * @brief Butterworth filter: 50Hz sample rate, 0.25Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/0.25Hz ≡ 100Hz/0.5Hz
 * 
 * Source: libraries/Filter/Butter.h:35
 */
typedef Butter2<butter100_05_coeffs> butter50hz0_25; //50hz sample, 0.25hz fcut

/**
 * @typedef butter10hz0_05
 * @brief Butterworth filter: 10Hz sample rate, 0.05Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.05Hz ≡ 100Hz/0.5Hz
 * 
 * Source: libraries/Filter/Butter.h:36
 */
typedef Butter2<butter100_05_coeffs> butter10hz0_05; //10hz sample, 0.05hz fcut

/**
 * @struct butter100_1_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 1.0Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          Commonly used cutoff frequency providing good balance between noise rejection
 *          and signal responsiveness.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 1.0 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 1.0Hz cutoff (butter100hz1_0)
 *       - 50Hz sample @ 0.5Hz cutoff (butter50hz0_5)
 *       - 10Hz sample @ 0.1Hz cutoff (butter10hz0_1)
 * 
 * Source: libraries/Filter/Butter.h:38-43
 */
struct butter100_1_coeffs
{
  static constexpr float A1 = 1.9111970674f;     ///< First recursive coefficient
  static constexpr float A2 = -0.9149758348f;    ///< Second recursive coefficient
  static constexpr float GAIN = 1.058546241e+03f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz1_0
 * @brief Butterworth filter: 100Hz sample rate, 1.0Hz cutoff frequency
 * 
 * @details Commonly used filter for sensor data with good noise rejection.
 *          Balances responsiveness with smoothing.
 * 
 * Source: libraries/Filter/Butter.h:44
 */
typedef Butter2<butter100_1_coeffs> butter100hz1_0; //100hz sample, 1hz fcut

/**
 * @typedef butter50hz0_5
 * @brief Butterworth filter: 50Hz sample rate, 0.5Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/0.5Hz ≡ 100Hz/1.0Hz
 * 
 * Source: libraries/Filter/Butter.h:45
 */
typedef Butter2<butter100_1_coeffs> butter50hz0_5; //50hz sample, 0.5hz fcut

/**
 * @typedef butter10hz0_1
 * @brief Butterworth filter: 10Hz sample rate, 0.1Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.1Hz ≡ 100Hz/1.0Hz
 * 
 * Source: libraries/Filter/Butter.h:46
 */
typedef Butter2<butter100_1_coeffs> butter10hz0_1; //10hz sample, 0.1hz fcut

/**
 * @struct butter100_1_5_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 1.5Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          Moderate cutoff frequency allowing faster signal changes through while
 *          still providing noise reduction.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 1.5 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 1.5Hz cutoff (butter100hz1_5)
 *       - 50Hz sample @ 0.75Hz cutoff (butter50hz0_75)
 *       - 10Hz sample @ 0.15Hz cutoff (butter10hz0_15)
 * 
 * Source: libraries/Filter/Butter.h:48-53
 */
struct butter100_1_5_coeffs
{
  static constexpr float A1 = 1.8668922797f;    ///< First recursive coefficient
  static constexpr float A2 = -0.8752145483f;   ///< Second recursive coefficient
  static constexpr float GAIN = 4.806381793e+02f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz1_5
 * @brief Butterworth filter: 100Hz sample rate, 1.5Hz cutoff frequency
 * 
 * @details Moderate filtering with improved responsiveness over 1.0Hz cutoff.
 * 
 * Source: libraries/Filter/Butter.h:54
 */
typedef Butter2<butter100_1_5_coeffs> butter100hz1_5; //100hz sample, 1.5hz fcut

/**
 * @typedef butter50hz0_75
 * @brief Butterworth filter: 50Hz sample rate, 0.75Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/0.75Hz ≡ 100Hz/1.5Hz
 * 
 * Source: libraries/Filter/Butter.h:55
 */
typedef Butter2<butter100_1_5_coeffs> butter50hz0_75; //50hz sample, 0.75hz fcut

/**
 * @typedef butter10hz0_15
 * @brief Butterworth filter: 10Hz sample rate, 0.15Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.15Hz ≡ 100Hz/1.5Hz
 * 
 * Source: libraries/Filter/Butter.h:56
 */
typedef Butter2<butter100_1_5_coeffs> butter10hz0_15; //10hz sample, 0.15hz fcut

/**
 * @struct butter100_2_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 2.0Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          Higher cutoff frequency for applications requiring faster response while
 *          maintaining some noise suppression.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 2.0 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 2.0Hz cutoff (butter100hz2_0)
 *       - 50Hz sample @ 1.0Hz cutoff (butter50hz1_0)
 *       - 10Hz sample @ 0.2Hz cutoff (butter10hz0_2)
 * 
 * Source: libraries/Filter/Butter.h:58-63
 */
struct butter100_2_coeffs
{
  static constexpr float A1 = 1.8226949252f;    ///< First recursive coefficient
  static constexpr float A2 = -0.8371816513f;   ///< Second recursive coefficient
  static constexpr float GAIN = 2.761148367e+02f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz2_0
 * @brief Butterworth filter: 100Hz sample rate, 2.0Hz cutoff frequency
 * 
 * @details Higher cutoff for faster response with moderate noise rejection.
 * 
 * Source: libraries/Filter/Butter.h:64
 */
typedef Butter2<butter100_2_coeffs> butter100hz2_0; //100hz sample, 2hz fcut

/**
 * @typedef butter50hz1_0
 * @brief Butterworth filter: 50Hz sample rate, 1.0Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/1.0Hz ≡ 100Hz/2.0Hz
 * 
 * Source: libraries/Filter/Butter.h:65
 */
typedef Butter2<butter100_2_coeffs> butter50hz1_0; //50hz sample, 1hz fcut

/**
 * @typedef butter10hz0_2
 * @brief Butterworth filter: 10Hz sample rate, 0.2Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.2Hz ≡ 100Hz/2.0Hz
 * 
 * Source: libraries/Filter/Butter.h:66
 */
typedef Butter2<butter100_2_coeffs> butter10hz0_2; //10hz sample, 0.2hz fcut

/**
 * @struct butter100_3_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 3.0Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          Relatively high cutoff frequency providing minimal delay with light smoothing.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 3.0 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 3.0Hz cutoff (butter100hz3_0)
 *       - 50Hz sample @ 1.5Hz cutoff (butter50hz1_5)
 *       - 10Hz sample @ 0.3Hz cutoff (butter10hz0_3)
 * 
 * Source: libraries/Filter/Butter.h:68-73
 */
struct butter100_3_coeffs
{
  static constexpr float A1 = 1.7347257688f;    ///< First recursive coefficient
  static constexpr float A2 = -0.7660066009f;   ///< Second recursive coefficient
  static constexpr float GAIN = 1.278738361e+02f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz3_0
 * @brief Butterworth filter: 100Hz sample rate, 3.0Hz cutoff frequency
 * 
 * @details Light filtering with fast response for low-noise signals.
 * 
 * Source: libraries/Filter/Butter.h:74
 */
typedef Butter2<butter100_3_coeffs> butter100hz3_0; //100hz sample, 3hz fcut

/**
 * @typedef butter50hz1_5
 * @brief Butterworth filter: 50Hz sample rate, 1.5Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/1.5Hz ≡ 100Hz/3.0Hz
 * 
 * Source: libraries/Filter/Butter.h:75
 */
typedef Butter2<butter100_3_coeffs> butter50hz1_5; //50hz sample, 1.5hz fcut

/**
 * @typedef butter10hz0_3
 * @brief Butterworth filter: 10Hz sample rate, 0.3Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.3Hz ≡ 100Hz/3.0Hz
 * 
 * Source: libraries/Filter/Butter.h:76
 */
typedef Butter2<butter100_3_coeffs> butter10hz0_3; //10hz sample, 0.3hz fcut

/**
 * @struct butter100_4_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 4.0Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          High cutoff frequency for applications requiring minimal delay and maximum
 *          responsiveness with only light smoothing.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 4.0 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 4.0Hz cutoff (butter100hz4_0)
 *       - 50Hz sample @ 2.0Hz cutoff (butter50hz2_0)
 *       - 10Hz sample @ 0.4Hz cutoff (butter10hz0_4)
 * 
 * Source: libraries/Filter/Butter.h:78-83
 */
struct butter100_4_coeffs
{
  static constexpr float A1 = 1.6474599811f;   ///< First recursive coefficient
  static constexpr float A2 = -0.7008967812f;  ///< Second recursive coefficient
  static constexpr float GAIN = 7.485478157e+01f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz4_0
 * @brief Butterworth filter: 100Hz sample rate, 4.0Hz cutoff frequency
 * 
 * @details Very light filtering with minimal delay, suitable for clean signals.
 * 
 * Source: libraries/Filter/Butter.h:84
 */
typedef Butter2<butter100_4_coeffs> butter100hz4_0; //100hz sample, 4hz fcut

/**
 * @typedef butter50hz2_0
 * @brief Butterworth filter: 50Hz sample rate, 2.0Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/2.0Hz ≡ 100Hz/4.0Hz
 * 
 * Source: libraries/Filter/Butter.h:85
 */
typedef Butter2<butter100_4_coeffs> butter50hz2_0; //50hz sample, 2hz fcut

/**
 * @typedef butter10hz0_4
 * @brief Butterworth filter: 10Hz sample rate, 0.4Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.4Hz ≡ 100Hz/4.0Hz
 * 
 * Source: libraries/Filter/Butter.h:86
 */
typedef Butter2<butter100_4_coeffs> butter10hz0_4; //10hz sample, .4hz fcut

/**
 * @struct butter100_8_coeffs
 * @brief Butterworth filter coefficients for 100Hz sample rate with 8.0Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          Very high cutoff frequency providing minimal filtering, primarily removing
 *          only the highest frequency components while preserving signal dynamics.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 100 Hz
 *          - Cutoff Frequency: 8.0 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 100Hz sample @ 8.0Hz cutoff (butter100hz8_0)
 *       - 50Hz sample @ 4.0Hz cutoff (butter50hz4_0)
 *       - 10Hz sample @ 0.8Hz cutoff (butter10hz0_8)
 * 
 * Source: libraries/Filter/Butter.h:88-93
 */
struct butter100_8_coeffs
{
  static constexpr float A1 = 1.3072850288f;   ///< First recursive coefficient
  static constexpr float A2 = -0.4918122372f;  ///< Second recursive coefficient
  static constexpr float GAIN = 2.167702007e+01f; ///< DC gain normalization factor
};

/**
 * @typedef butter100hz8_0
 * @brief Butterworth filter: 100Hz sample rate, 8.0Hz cutoff frequency
 * 
 * @details Minimal filtering with very fast response, removes only highest frequencies.
 * 
 * Source: libraries/Filter/Butter.h:94
 */
typedef Butter2<butter100_8_coeffs> butter100hz8_0; //100hz sample, 8hz fcut

/**
 * @typedef butter50hz4_0
 * @brief Butterworth filter: 50Hz sample rate, 4.0Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 50Hz/4.0Hz ≡ 100Hz/8.0Hz
 * 
 * Source: libraries/Filter/Butter.h:95
 */
typedef Butter2<butter100_8_coeffs> butter50hz4_0; //50hz sample, 4hz fcut

/**
 * @typedef butter10hz0_8
 * @brief Butterworth filter: 10Hz sample rate, 0.8Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/0.8Hz ≡ 100Hz/8.0Hz
 * 
 * Source: libraries/Filter/Butter.h:96
 */
typedef Butter2<butter100_8_coeffs> butter10hz0_8; //10hz sample, .8hz fcut

/**
 * @struct butter50_8_coeffs
 * @brief Butterworth filter coefficients for 50Hz sample rate with 8.0Hz cutoff frequency
 * 
 * @details Pre-computed coefficients for a second-order Butterworth low-pass filter.
 *          This coefficient set represents a very high cutoff relative to sample rate
 *          (16% of Nyquist frequency), providing minimal phase delay with light smoothing.
 * 
 *          Filter Specifications:
 *          - Sample Rate: 50 Hz
 *          - Cutoff Frequency: 8.0 Hz
 *          - A1, A2: Recursive (feedback) coefficients
 *          - GAIN: DC gain normalization factor
 * 
 * @note These same coefficients can be proportionally scaled:
 *       - 50Hz sample @ 8.0Hz cutoff (butter50hz8_0)
 *       - 10Hz sample @ 1.6Hz cutoff (butter10hz1_6)
 * 
 * @warning High cutoff-to-sample-rate ratio (0.16) approaches aliasing concerns.
 *          Ensure input signal does not contain significant energy above Nyquist (25Hz).
 * 
 * Source: libraries/Filter/Butter.h:98-103
 */
struct butter50_8_coeffs
{
  static constexpr float A1 = 0.6710290908f;  ///< First recursive coefficient
  static constexpr float A2 = -0.2523246263f; ///< Second recursive coefficient
  static constexpr float GAIN = 6.881181354e+00f; ///< DC gain normalization factor
};

/**
 * @typedef butter50hz8_0
 * @brief Butterworth filter: 50Hz sample rate, 8.0Hz cutoff frequency
 * 
 * @details Very high cutoff frequency relative to sample rate, provides minimal delay.
 *          Useful when sample rate is constrained but fast response is required.
 * 
 * Source: libraries/Filter/Butter.h:104
 */
typedef Butter2<butter50_8_coeffs> butter50hz8_0; //50hz sample, 8hz fcut

/**
 * @typedef butter10hz1_6
 * @brief Butterworth filter: 10Hz sample rate, 1.6Hz cutoff frequency
 * 
 * @details Proportionally scaled version: 10Hz/1.6Hz ≡ 50Hz/8.0Hz
 *          High cutoff-to-sample-rate ratio, provides fast response for low-rate data.
 * 
 * Source: libraries/Filter/Butter.h:105
 */
typedef Butter2<butter50_8_coeffs> butter10hz1_6; //10hz sample, 1.6hz fcut
