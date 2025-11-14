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
 * @file AverageFilter.h
 * @brief Moving average filters for smoothing noisy data
 * 
 * @details This file provides two implementations of moving average filters:
 *          - AverageFilter: General-purpose moving average that recalculates sum each sample
 *          - AverageIntegralFilter: Optimized version using running sum for integral types
 * 
 *          Moving average filters are simple FIR (Finite Impulse Response) filters
 *          that compute the mean of the last FILTER_SIZE samples. They are effective
 *          for reducing high-frequency noise but introduce phase delay and have poor
 *          stopband attenuation compared to more sophisticated filter designs.
 * 
 * @note All operations are dimensionless and maintain input units
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */
#pragma once

#include "FilterClass.h"
#include "FilterWithBuffer.h"

/**
 * @class AverageFilter
 * @brief Simple moving average filter that computes mean of last FILTER_SIZE samples
 * 
 * @details This class implements a rectangular window moving average filter (FIR filter
 *          with all coefficients = 1/FILTER_SIZE). It maintains a circular buffer of
 *          samples and recalculates the sum on each apply() call.
 * 
 *          Algorithm:
 *          - Maintains circular buffer of FILTER_SIZE samples
 *          - On each new sample: stores in buffer, sums all values, returns mean
 *          - Gradually fills buffer (_num_samples tracks valid samples until full)
 *          - Once full, operates as true moving average
 * 
 *          Mathematical Foundation:
 *          - Rectangular window moving average
 *          - FIR filter with impulse response h[n] = 1/FILTER_SIZE for n=0..FILTER_SIZE-1
 *          - Linear phase response (symmetric FIR filter)
 *          - Frequency response: sinc function with first null at sample_freq/FILTER_SIZE
 * 
 *          Filter Characteristics:
 *          - Linear phase: introduces constant group delay of (FILTER_SIZE-1)/2 samples
 *          - Poor stopband attenuation: -13.5 dB at first null, side lobes persist
 *          - Simple and fast: suitable for basic noise reduction
 *          - No configuration required: single parameter (window size)
 * 
 *          Template Parameters:
 *          - Uses larger accumulator type U to prevent overflow during summation
 *          - U must be able to hold sum of FILTER_SIZE values of type T
 *          - Example: for T=int16_t with FILTER_SIZE=5, U=int32_t prevents overflow
 * 
 *          Typical Usage:
 *          - Sensor smoothing when phase delay is acceptable (not for real-time control)
 *          - Noise reduction on slowly-varying signals
 *          - Pre-filtering before higher-order filters
 *          - Common window sizes: 2-5 samples (balance noise reduction vs lag)
 * 
 * @tparam T Sample data type (int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, float)
 * @tparam U Accumulator type for summation - must be larger than T to prevent overflow
 * @tparam FILTER_SIZE Number of samples in moving average window (typically 2-5)
 * 
 * @warning Moving average introduces group delay of (FILTER_SIZE-1)/2 samples
 * @warning Frequency response has sinc shape with nulls at sample_freq/FILTER_SIZE
 * @warning Risk of overflow during summation - ensure U type can hold FILTER_SIZE * max(T)
 * 
 * @note This implementation recalculates sum on every sample (O(FILTER_SIZE) complexity)
 * @note For integral types, consider AverageIntegralFilter for better performance (O(1))
 * 
 * @see AverageIntegralFilter for optimized version using running sum
 * 
 * Source: libraries/Filter/AverageFilter.h:27-44,78-107
 */
template <class T, class U, uint8_t FILTER_SIZE>
class AverageFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    /**
     * @brief Default constructor - initializes filter with empty buffer
     * 
     * @details Sets _num_samples to 0, indicating buffer is empty.
     *          Filter will gradually fill until FILTER_SIZE samples accumulated.
     */
    AverageFilter() : FilterWithBuffer<T,FILTER_SIZE>(), _num_samples(0) {
    };

    /**
     * @brief Add a new raw value to the filter and retrieve the filtered result
     * 
     * @details Adds the new sample to the circular buffer, recalculates the sum
     *          of all samples in the buffer, and returns the mean value.
     *          
     *          During initial fill (_num_samples < FILTER_SIZE), returns average
     *          of samples received so far. Once full, returns true moving average
     *          of last FILTER_SIZE samples.
     *          
     *          Complexity: O(FILTER_SIZE) - iterates through entire buffer each call
     * 
     * @param[in] sample New raw value to add to the filter
     * 
     * @return Averaged value over current window (mean of valid samples)
     * 
     * @note Called at sensor update rate (varies by sensor: 100Hz-8kHz typical)
     * @warning Sum calculated using type U - ensure no overflow for FILTER_SIZE * max(T)
     */
    virtual T        apply(T sample) override;

    /**
     * @brief Clear the filter - resets buffer and sample count to zero
     * 
     * @details Clears the circular buffer by calling parent's reset() and
     *          resets _num_samples to 0. Next apply() will start fresh fill sequence.
     */
    virtual void        reset() override;

protected:
    /**
     * @brief Number of valid samples in the filter
     * 
     * @details Tracks how many samples have been added to the filter.
     *          Starts at 0, increments with each apply(), saturates at FILTER_SIZE.
     *          Used to compute correct average during initial fill period.
     */
    uint8_t        _num_samples;
};

/**
 * Convenience typedefs for common AverageFilter instantiations
 * 
 * @details Pre-instantiated filter types for common data type and size combinations.
 *          Naming convention: AverageFilter<Type>_Size<N>
 *          
 *          Type pairings (T, U) chosen to prevent overflow:
 *          - int8_t → int16_t (256x margin)
 *          - int16_t → int32_t (65536x margin)
 *          - int32_t → float (sufficient for typical filter sizes)
 *          - float → float (maintains precision)
 *          
 *          Common sizes: 2-5 samples
 *          - Size 2: Minimal smoothing, fastest response
 *          - Size 3-4: Balanced noise reduction and lag
 *          - Size 5: Maximum smoothing in common use
 *          
 * @note Larger FILTER_SIZE values provide more smoothing but increase phase delay
 */
typedef AverageFilter<int8_t,int16_t,2> AverageFilterInt8_Size2;
typedef AverageFilter<int8_t,int16_t,3> AverageFilterInt8_Size3;
typedef AverageFilter<int8_t,int16_t,4> AverageFilterInt8_Size4;
typedef AverageFilter<int8_t,int16_t,5> AverageFilterInt8_Size5;
typedef AverageFilter<uint8_t,uint16_t,2> AverageFilterUInt8_Size2;
typedef AverageFilter<uint8_t,uint16_t,3> AverageFilterUInt8_Size3;
typedef AverageFilter<uint8_t,uint16_t,4> AverageFilterUInt8_Size4;
typedef AverageFilter<uint8_t,uint16_t,5> AverageFilterUInt8_Size5;

typedef AverageFilter<int16_t,int32_t,2> AverageFilterInt16_Size2;
typedef AverageFilter<int16_t,int32_t,3> AverageFilterInt16_Size3;
typedef AverageFilter<int16_t,int32_t,4> AverageFilterInt16_Size4;
typedef AverageFilter<int16_t,int32_t,5> AverageFilterInt16_Size5;
typedef AverageFilter<uint16_t,uint32_t,2> AverageFilterUInt16_Size2;
typedef AverageFilter<uint16_t,uint32_t,3> AverageFilterUInt16_Size3;
typedef AverageFilter<uint16_t,uint32_t,4> AverageFilterUInt16_Size4;
typedef AverageFilter<uint16_t,uint32_t,5> AverageFilterUInt16_Size5;

typedef AverageFilter<int32_t,float,2> AverageFilterInt32_Size2;
typedef AverageFilter<int32_t,float,3> AverageFilterInt32_Size3;
typedef AverageFilter<int32_t,float,4> AverageFilterInt32_Size4;
typedef AverageFilter<int32_t,float,5> AverageFilterInt32_Size5;
typedef AverageFilter<uint32_t,float,2> AverageFilterUInt32_Size2;
typedef AverageFilter<uint32_t,float,3> AverageFilterUInt32_Size3;
typedef AverageFilter<uint32_t,float,4> AverageFilterUInt32_Size4;
typedef AverageFilter<uint32_t,float,5> AverageFilterUInt32_Size5;

typedef AverageFilter<float,float,5> AverageFilterFloat_Size5;

// Public Methods //////////////////////////////////////////////////////////////

/**
 * @brief Apply new sample and return filtered value (AverageFilter implementation)
 * 
 * @details Implementation of moving average apply():
 *          1. Store new sample in circular buffer (via parent's apply)
 *          2. Increment sample counter (saturates at FILTER_SIZE)
 *          3. Sum all FILTER_SIZE buffer values
 *          4. Divide by actual number of samples (_num_samples)
 *          5. Return result cast back to type T
 *          
 *          The summation uses type U to prevent overflow during accumulation.
 * 
 * @warning Risk of overflow if U cannot hold sum of FILTER_SIZE * max(T) values
 */
template <class T, class U, uint8_t FILTER_SIZE>
T AverageFilter<T,U,FILTER_SIZE>::        apply(T sample)
{
    U        result = 0;

    // call parent's apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

    // increment the number of samples so far
    _num_samples++;
    if( _num_samples > FILTER_SIZE || _num_samples == 0 )
        _num_samples = FILTER_SIZE;

    // get sum of all values - there is a risk of overflow here that we ignore
    for(uint8_t i=0; i<FILTER_SIZE; i++)
        result += FilterWithBuffer<T,FILTER_SIZE>::samples[i];

    return (T)(result / _num_samples);
}

/**
 * @brief Reset filter - clear all samples and restart fill sequence
 * 
 * @details Clears the circular buffer (via parent's reset) and resets sample counter.
 *          After reset, filter behaves as newly constructed - will gradually fill
 *          buffer and average over increasing number of samples until full.
 */
template <class T, class U, uint8_t FILTER_SIZE>
void AverageFilter<T,U,FILTER_SIZE>::        reset()
{
    // call parent's reset function to clear the sample buffer
    FilterWithBuffer<T,FILTER_SIZE>::reset();

    // clear our variable
    _num_samples = 0;
}

/**
 * @class AverageIntegralFilter
 * @brief Optimized moving average filter for integral types using running sum
 * 
 * @details This class provides a more efficient implementation of moving average
 *          for integral types by maintaining a running sum instead of recalculating
 *          the sum on every sample.
 * 
 *          Algorithm:
 *          - Maintains running sum _sum of samples in buffer
 *          - On each new sample:
 *            1. Subtract oldest sample from _sum (sample being replaced)
 *            2. Add new sample to _sum
 *            3. Store new sample in circular buffer
 *          - Result computed only when getf()/getd() called: _sum / _num_samples
 * 
 *          Performance:
 *          - O(1) complexity per sample (vs O(FILTER_SIZE) for AverageFilter)
 *          - More efficient for integral types where running sum doesn't lose precision
 *          - Suitable for high-rate sensors (gyros at 8kHz, accelerometers at 4kHz)
 * 
 *          Design Rationale:
 *          - Designed for integer types to avoid floating-point precision loss
 *          - Running sum in integral type U avoids accumulating rounding errors
 *          - Separates filtering (apply) from result retrieval (getf/getd)
 * 
 *          Usage Pattern:
 *          ```cpp
 *          AverageIntegralFilter<int16_t, int32_t, 5> gyro_filter;
 *          gyro_filter.apply(gyro_sample);  // Returns 0, just updates filter
 *          float filtered_gyro = gyro_filter.getf();  // Get filtered result
 *          ```
 * 
 * @tparam T Sample data type - typically integral types (int16_t, int32_t, etc.)
 * @tparam U Accumulator type - must hold FILTER_SIZE * max(T) without overflow
 * @tparam FILTER_SIZE Number of samples in moving average window
 * 
 * @warning apply() returns 0 (not the filtered value) - caller MUST use getf() or getd()
 * @warning U type must be large enough: requires capacity for FILTER_SIZE * max(T)
 * @warning For floating-point types, running sum may accumulate precision errors
 * 
 * @note Preferred over AverageFilter for integral sensor data (IMU, ADC, encoders)
 * @note Filter characteristics same as AverageFilter (phase delay, frequency response)
 * 
 * @see AverageFilter for general-purpose implementation with different tradeoffs
 * 
 * Source: libraries/Filter/AverageFilter.h:117-177
 */
template <class T, class U, uint8_t FILTER_SIZE>
class AverageIntegralFilter : public AverageFilter<T,U,FILTER_SIZE>
{
public:
    /**
     * @brief Add a new raw value to the filter using running sum algorithm
     * 
     * @details Updates the running sum by subtracting the oldest sample (being replaced)
     *          and adding the new sample. Stores new sample in circular buffer.
     *          
     *          IMPORTANT: Does NOT return the filtered value. Method signature maintained
     *          for compatibility with AverageFilter interface, but always returns 0.
     *          Caller must call getf() or getd() to retrieve the filtered result.
     *          
     *          Algorithm:
     *          1. Read oldest sample from buffer (sample_index position)
     *          2. Store new sample in buffer (via parent's apply)
     *          3. Update sample counter (saturates at FILTER_SIZE)
     *          4. Update running sum: _sum = _sum - oldest + new
     * 
     * @param[in] sample New raw value to add to the filter
     * 
     * @return 0 (always) - filtered value NOT returned, use getf() or getd()
     * 
     * @note Complexity: O(1) - constant time regardless of FILTER_SIZE
     * @warning Return value is 0, not the filtered result - call getf()/getd() instead
     */
    virtual T apply(T sample) override;

    /**
     * @brief Get the current filtered value as a float
     * 
     * @details Computes and returns the current moving average as a float:
     *          result = (float)_sum / _num_samples
     *          
     *          If no samples have been added (_num_samples == 0), returns 0.0f.
     * 
     * @return Current average as float, or 0.0f if filter is empty
     */
    virtual float getf();

    /**
     * @brief Get the current filtered value as a double
     * 
     * @details Computes and returns the current moving average as a double:
     *          result = (double)_sum / _num_samples
     *          
     *          If no samples have been added (_num_samples == 0), returns 0.0.
     *          Use when higher precision needed than float provides.
     * 
     * @return Current average as double, or 0.0 if filter is empty
     */
    virtual double getd();
protected:
    // the current sum of samples
    U _sum = 0;
};

/**
 * @brief Apply new sample using running sum (AverageIntegralFilter implementation)
 * 
 * @details Efficient O(1) implementation:
 *          1. Read oldest sample at current sample_index position
 *          2. Store new sample in buffer (via FilterWithBuffer::apply)
 *          3. Increment and saturate sample counter
 *          4. Update running sum: subtract old, add new
 *          
 *          Running sum maintained incrementally avoids full buffer iteration.
 */
template <class T, class U, uint8_t FILTER_SIZE>
T AverageIntegralFilter<T,U,FILTER_SIZE>::apply(T sample)
{
    T curr = this->samples[this->sample_index];

    // call parent's parent apply function to get the sample into the array
    FilterWithBuffer<T,FILTER_SIZE>::apply(sample);

    // increment the number of samples so far
    this->_num_samples++;
    if (this->_num_samples > FILTER_SIZE || this->_num_samples == 0) {
        this->_num_samples = FILTER_SIZE;
    }

    _sum -= curr;
    _sum += sample;

    // don't return the value: caller is forced to call getf() or getd()
    return 0;
}

/**
 * @brief Get filtered value as float (AverageIntegralFilter implementation)
 * 
 * @details Computes mean from running sum: (float)_sum / _num_samples
 *          Returns 0.0f if filter empty (no samples applied yet).
 */
template <class T, class U, uint8_t FILTER_SIZE>
float AverageIntegralFilter<T,U,FILTER_SIZE>::getf()
{
    if (this->_num_samples == 0) {
        return 0.f;
    }

    return (float)_sum / this->_num_samples;
}

/**
 * @brief Get filtered value as double (AverageIntegralFilter implementation)
 * 
 * @details Computes mean from running sum: (double)_sum / _num_samples
 *          Returns 0.0 if filter empty (no samples applied yet).
 *          Provides higher precision than getf() for large sums.
 */
template <class T, class U, uint8_t FILTER_SIZE>
double AverageIntegralFilter<T,U,FILTER_SIZE>::getd()
{
    if (this->_num_samples == 0) {
        return 0.f;
    }

    return (double)_sum / this->_num_samples;
}
