#pragma once

/**
 * @file SlewLimiter.h
 * @brief Slew rate limiting filter for preventing controller oscillation
 * 
 * @details This file implements a slew rate limiter that prevents controller
 *          oscillation by constraining the rate of change of the controller output.
 *          The limiter tracks oscillation events and applies a modifier to maintain
 *          output changes within specified slew rate limits.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include <stdint.h>
#include "LowPassFilter.h"

#define SLEWLIMITER_N_EVENTS 2  // number of positive and negative consecutive slew rate exceedance events recorded where a value of 2 corresponds to a complete cycle 

/**
 * @class SlewLimiter
 * @brief Slew rate limiting filter for controller oscillation prevention
 * 
 * @details This filter prevents controller oscillation by limiting the rate of change
 *          (slew rate) of the controller output. It implements an adaptive limiting
 *          mechanism that:
 *          
 *          - Tracks positive and negative slew rate exceedance events
 *          - Applies a modifier value in the range [0, 1] to constrain output changes
 *          - Uses an internal derivative approximation with low-pass filtering at 25 Hz
 *          - Maintains event history (SLEWLIMITER_N_EVENTS=2) to detect complete oscillation cycles
 *          - Implements attack and decay smoothing mechanisms for the modifier
 *          
 *          **Algorithm Overview:**
 *          The limiter calculates the derivative of the input signal, applies low-pass
 *          filtering for smoothing, and compares the filtered derivative against the
 *          maximum allowed slew rate. When the slew rate is exceeded, the limiter
 *          reduces the modifier to constrain the output rate of change. The event
 *          tracking system detects oscillation patterns by monitoring consecutive
 *          positive and negative slew rate violations.
 *          
 *          **Mathematical Foundation:**
 *          - Derivative approximation: d_signal/dt ≈ (sample - last_sample) / dt
 *          - Low-pass filter: Discrete implementation at configured cutoff frequency (default 25 Hz)
 *          - Modifier calculation: Scales linearly between 0 and 1 based on slew rate ratio
 *          
 * @note This filter is commonly used in attitude and position controllers to
 *       prevent high-frequency oscillations that can destabilize the vehicle.
 * 
 * @warning Modifying slew_rate_max can significantly affect controller stability.
 *          Increasing the limit may allow oscillations, while decreasing it may
 *          reduce responsiveness. Changes should be validated through testing.
 */
class SlewLimiter {
public:
    /**
     * @brief Construct a new SlewLimiter filter
     * 
     * @details Initializes the slew rate limiter with specified maximum slew rate
     *          and time constant for the smoothing filter. The internal low-pass
     *          filter is configured at 25 Hz cutoff frequency for derivative smoothing.
     * 
     * @param[in] slew_rate_max Maximum allowed slew rate in units/second. This defines
     *                          the threshold above which the modifier will reduce the
     *                          output rate of change.
     * @param[in] slew_rate_tau Time constant in seconds for the attack/decay smoothing
     *                          of the modifier. Smaller values result in faster response
     *                          to slew rate violations.
     * 
     * @note The slew_rate_max and slew_rate_tau parameters are stored as references,
     *       allowing dynamic adjustment without reconstructing the filter.
     * 
     * @warning Ensure slew_rate_max is positive and non-zero to avoid undefined behavior.
     */
    SlewLimiter(const float &slew_rate_max, const float &slew_rate_tau);

    CLASS_NO_COPY(SlewLimiter);

    /**
     * @brief Apply slew rate limiting to input sample
     * 
     * @details Processes the input sample through the slew rate limiter and returns
     *          a modifier value that should be applied to the controller output to
     *          maintain the output rate of change within specified limits.
     *          
     *          The algorithm:
     *          1. Calculates derivative: (sample - last_sample) / dt
     *          2. Applies low-pass filtering to the derivative for smoothing
     *          3. Detects slew rate exceedance events (positive and negative)
     *          4. Tracks event history to identify oscillation patterns
     *          5. Computes modifier based on the ratio of actual to maximum slew rate
     *          6. Updates internal state for next iteration
     *          
     *          The modifier value is calculated to scale the controller output such
     *          that the resulting slew rate does not exceed slew_rate_max. A modifier
     *          of 1.0 means no limiting is applied, while values closer to 0.0 indicate
     *          aggressive limiting to prevent oscillation.
     * 
     * @param[in] sample Current input sample value in arbitrary units
     * @param[in] dt Time step since last call in seconds. Must be positive and non-zero.
     *               Typically this is the controller loop period (e.g., 0.0025s for 400Hz).
     * 
     * @return Multiplier value in range [0.0, 1.0] to apply to controller output.
     *         Multiply the controller output by this value to maintain slew rate limits.
     *         - 1.0: No limiting required, slew rate within bounds
     *         - 0.0-1.0: Limiting active, reduce output proportionally
     * 
     * @note This method should be called at a consistent rate for accurate derivative
     *       calculation and event timing. The dt parameter must reflect the actual
     *       time elapsed since the previous call.
     * 
     * @warning Do not call this method at rates significantly different from the expected
     *          controller loop rate, as this will affect the accuracy of the derivative
     *          calculation and the effectiveness of the low-pass filter.
     */
    float modifier(float sample, float dt);

    /**
     * @brief Get the last calculated oscillation slew rate
     * 
     * @details Returns the most recent slew rate calculated by the filter. This value
     *          represents the filtered derivative of the input signal and can be used
     *          for monitoring and diagnostics.
     * 
     * @return Last calculated slew rate in units/second (same units as input sample per second)
     * 
     * @note This value is updated each time modifier() is called and reflects the
     *       low-pass filtered derivative, not the instantaneous derivative.
     */
    float get_slew_rate(void) const {
        return _output_slew_rate;
    }

private:
    const float &slew_rate_max;           ///< Reference to maximum allowed slew rate in units/second
    const float &slew_rate_tau;           ///< Reference to time constant in seconds for modifier smoothing
    LowPassFilterFloat slew_filter;       ///< Low-pass filter at 25 Hz for derivative smoothing
    float _output_slew_rate;              ///< Last calculated slew rate (filtered derivative) in units/second
    float _modifier_slew_rate;            ///< Slew rate used for modifier calculation
    float last_sample;                    ///< Previous input sample value for derivative calculation
    float _max_pos_slew_rate;             ///< Maximum positive slew rate detected in current event
    float _max_neg_slew_rate;             ///< Maximum negative slew rate detected in current event (stored as positive value)
    uint32_t _max_pos_slew_event_ms;      ///< Timestamp in milliseconds of maximum positive slew rate event
    uint32_t _max_neg_slew_event_ms;      ///< Timestamp in milliseconds of maximum negative slew rate event
    uint8_t _pos_event_index;             ///< Circular buffer index for positive event timestamps (wraps at SLEWLIMITER_N_EVENTS)
    uint8_t _neg_event_index;             ///< Circular buffer index for negative event timestamps (wraps at SLEWLIMITER_N_EVENTS)
    uint32_t _pos_event_ms[SLEWLIMITER_N_EVENTS];  ///< Circular buffer of positive slew rate event timestamps for oscillation cycle detection
    uint32_t _neg_event_ms[SLEWLIMITER_N_EVENTS];  ///< Circular buffer of negative slew rate event timestamps for oscillation cycle detection
    bool _pos_event_stored;               ///< Flag indicating if at least one positive event has been recorded
    bool _neg_event_stored;               ///< Flag indicating if at least one negative event has been recorded
};
