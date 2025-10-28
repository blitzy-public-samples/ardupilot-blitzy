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
 * @file DerivativeFilter.h
 * @brief Timestamp-aware smoothing derivative filter implementation
 * 
 * @details This file implements a derivative (slope) filter based on Holoborodko 
 *          smooth low-noise differentiators. The filter computes smoothed derivatives 
 *          from non-uniformly sampled data by maintaining microsecond timestamps for 
 *          each sample. This approach provides superior noise rejection compared to 
 *          simple finite differences while handling variable sample rates.
 * 
 * Algorithm reference: 
 * http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
 * 
 * The filter maintains a circular buffer of samples with corresponding timestamps,
 * allowing accurate derivative computation even when samples arrive at irregular intervals.
 * Results are cached to avoid redundant computation when slope() is called multiple times
 * without new data.
 */
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

/**
 * @class DerivativeFilter
 * @brief Smoothing derivative filter with timestamp awareness for non-uniform sampling
 * 
 * @details This filter computes a smoothed derivative (slope) from discrete samples
 *          that may arrive at non-uniform time intervals. It uses Holoborodko smooth 
 *          differentiator coefficients to provide excellent noise rejection while 
 *          accurately computing the rate of change.
 * 
 *          Key features:
 *          - Timestamp-aware: Maintains microsecond timestamps for each sample to
 *            handle variable sample rates correctly
 *          - Smooth: Applies polynomial-based differentiator coefficients to reduce
 *            noise in the derivative estimate
 *          - Efficient: Caches the last computed slope to avoid redundant computation
 *          - Generic: Template supports float, Vector2f, Vector3f, and other numeric types
 * 
 *          Mathematical Foundation:
 *          The filter uses Holoborodko's smooth noise-robust differentiators, which
 *          combine differentiation with smoothing by fitting a polynomial to the data
 *          window. The non-uniform timestamp handling adjusts the differentiator
 *          coefficients based on actual time intervals between samples.
 * 
 *          Usage Pattern:
 *          1. Create filter with desired type and window size
 *          2. Call update(sample, timestamp) as new data arrives
 *          3. Call slope() to retrieve the computed derivative in units/second
 *          4. Call reset() to clear filter state if needed
 * 
 *          Performance Considerations:
 *          - Larger FILTER_SIZE provides smoother derivatives but increases phase delay
 *          - Typical FILTER_SIZE values: 5 (fast response), 7 (balanced), 9 (smooth)
 *          - Derivative computation only occurs when slope() is called after new data
 * 
 * @tparam T Data type being filtered (e.g., float for scalar, Vector2f/Vector3f for vectors)
 * @tparam FILTER_SIZE Number of samples in the filter window (typically 5, 7, 9, or 11)
 *                     Larger values increase smoothing but also increase phase delay
 * 
 * @warning Timestamps must be monotonically increasing and provided in microseconds.
 *          Non-monotonic timestamps will produce incorrect derivative calculations.
 * 
 * @warning Larger FILTER_SIZE values increase the phase delay (lag) between input
 *          changes and derivative output. Choose the smallest FILTER_SIZE that provides
 *          adequate noise rejection for your application.
 * 
 * @note Explicit template instantiations are provided for common configurations:
 *       - DerivativeFilterFloat_Size5: float data with 5-sample window
 *       - DerivativeFilterFloat_Size7: float data with 7-sample window  
 *       - DerivativeFilterFloat_Size9: float data with 9-sample window
 * 
 * Source: libraries/Filter/DerivativeFilter.h:28-51
 */
template <class T, uint8_t FILTER_SIZE>
class DerivativeFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    /**
     * @brief Default constructor - initializes filter buffer and state
     * 
     * @details Constructs a new DerivativeFilter with empty sample buffer, cleared
     *          timestamps, and no cached slope. The filter is ready to accept samples
     *          via update() calls immediately after construction.
     */
    DerivativeFilter() : FilterWithBuffer<T,FILTER_SIZE>() {
    };

    /**
     * @brief Add a new sample to the filter with its timestamp
     * 
     * @details This method adds a new raw data sample and its corresponding timestamp
     *          to the filter's circular buffer. The derivative is NOT recalculated 
     *          immediately - computation is deferred until slope() is called. This 
     *          allows multiple updates to be batched efficiently.
     * 
     *          The timestamp is critical for accurate derivative computation when
     *          samples arrive at non-uniform intervals. The filter uses timestamp
     *          differences to correctly scale the derivative calculation.
     * 
     * @param[in] sample New raw value to add to the filter buffer (type T)
     * @param[in] timestamp Microsecond timestamp for this sample (must be monotonically
     *                      increasing relative to previous samples)
     * 
     * @warning Timestamps must be in microseconds and monotonically increasing.
     *          Providing timestamps in other units or out-of-order will produce
     *          incorrect derivative values.
     * 
     * @note This method only updates the buffer and sets a flag indicating new data
     *       is available. The actual derivative computation occurs in slope().
     */
    void update(T sample, uint32_t timestamp);

    /**
     * @brief Compute and return the smoothed derivative (slope)
     * 
     * @details Calculates the derivative of the filtered signal using Holoborodko
     *          smooth differentiator coefficients. The calculation uses the timestamps
     *          associated with each sample to correctly handle non-uniform sampling.
     * 
     *          If no new data has been added since the last slope() call, the cached
     *          slope value is returned immediately without recomputation. This provides
     *          efficient access when slope is queried multiple times between updates.
     * 
     *          The derivative is computed in units per second, regardless of the actual
     *          time spacing of samples. For example, if filtering altitude in meters,
     *          slope() returns vertical velocity in m/s.
     * 
     * @return Smoothed derivative value in units/second. Returns cached value if no
     *         new samples have been added since last call. Returns 0.0 if insufficient
     *         samples are available (buffer not yet full).
     * 
     * @note The returned slope uses the most recent data in the buffer. For a 
     *       FILTER_SIZE of N, the derivative represents the smoothed rate of change
     *       over approximately the time span of the N samples.
     * 
     * @note Unit conversion is automatic: timestamps in microseconds are converted
     *       to seconds internally, so the output is always in units/second.
     */
    float slope(void);

    /**
     * @brief Clear all filter state and reset to initial conditions
     * 
     * @details Clears the sample buffer, timestamp buffer, and invalidates the cached
     *          slope value. After reset, the filter behaves as if newly constructed
     *          and will require FILTER_SIZE new samples before producing meaningful
     *          derivative values.
     * 
     *          Use reset when:
     *          - Switching to a new data stream with discontinuous values
     *          - Detecting a sensor dropout or data quality issue
     *          - Starting a new phase of operation where previous history is irrelevant
     * 
     * @note After reset(), slope() will return 0.0 until sufficient samples have been
     *       collected via update() calls.
     */
    virtual void        reset() override;

private:
    /// Flag indicating new data has been added since last slope calculation
    bool            _new_data;
    
    /// Cached slope value from most recent computation (units/second)
    float           _last_slope;

    /**
     * @brief Microsecond timestamps for samples in the filter buffer
     * 
     * @details This array maintains the microsecond timestamp corresponding to each
     *          sample in the inherited filter buffer. Timestamps are essential for
     *          computing accurate derivatives when samples arrive at non-uniform
     *          intervals. The timestamp buffer is managed in parallel with the sample
     *          buffer, using the same circular indexing scheme.
     * 
     *          Units: microseconds (uint32_t wraps at ~71.6 minutes, but derivative
     *          calculation uses differences which handle wrapping correctly for
     *          reasonable filter window durations)
     */
    uint32_t        _timestamps[FILTER_SIZE];
};

/// Explicit instantiation: float derivative filter with 5-sample window (fast response, less smoothing)
typedef DerivativeFilter<float,5> DerivativeFilterFloat_Size5;

/// Explicit instantiation: float derivative filter with 7-sample window (balanced response and smoothing)
typedef DerivativeFilter<float,7> DerivativeFilterFloat_Size7;

/// Explicit instantiation: float derivative filter with 9-sample window (smooth response, more lag)
typedef DerivativeFilter<float,9> DerivativeFilterFloat_Size9;
