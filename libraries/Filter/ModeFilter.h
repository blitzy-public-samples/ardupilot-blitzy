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
 * @file ModeFilter.h
 * @brief Mode/median filter implementation using rank-order statistics
 * 
 * @details This file implements a mode filter that maintains a sorted buffer of samples
 *          and outputs a specified ranked element (typically the median). The filter uses
 *          an insertion sort algorithm with alternating high/low sample eviction to
 *          efficiently maintain a sorted buffer without re-sorting on every sample.
 *          
 *          The filter is particularly effective at rejecting impulse noise and outliers
 *          while introducing predictable group delay. Filter size should be odd for true
 *          median filtering.
 * 
 * Source: libraries/Filter/ModeFilter.h:1-74
 */
#pragma once

#include <inttypes.h>
#include "FilterClass.h"
#include "FilterWithBuffer.h"

/**
 * @class ModeFilter
 * @brief Mode filter using rank-order statistics to select median or other ranked element
 * 
 * @details This template class implements a mode/median filter by maintaining a sorted buffer
 *          of the most recent FILTER_SIZE samples and returning a specified ranked element
 *          from that buffer on each call to apply().
 *          
 *          **Algorithm**: The filter uses insertion sort to maintain buffer order. Each new
 *          sample is inserted into its correct position in the sorted buffer. To maintain
 *          constant buffer size, the filter alternates between evicting the highest and
 *          lowest samples (controlled by drop_high_sample flag). This approach is more
 *          efficient than re-sorting the entire buffer on each sample.
 *          
 *          **Filter Size**: Should always be an odd number (3, 5, 7) for true median filtering,
 *          where the median is the exact middle element. Even-sized buffers (4, 6) don't have
 *          a true median; the return_element parameter should be set to FILTER_SIZE/2 for the
 *          best median approximation.
 *          
 *          **Template Parameters**:
 *          - T: Data type for samples (int8_t, uint8_t, int16_t, uint16_t, float supported)
 *          - FILTER_SIZE: Number of samples in the buffer (typically 3-7)
 *          
 *          **Return Element Selection**: The return_element constructor parameter selects which
 *          ranked element to output:
 *          - 0 = minimum (lowest value in buffer)
 *          - FILTER_SIZE/2 = median (middle value, best for noise rejection)
 *          - FILTER_SIZE-1 = maximum (highest value in buffer)
 *          
 *          **Computational Complexity**: O(FILTER_SIZE) per sample for insertion sort
 *          
 *          **Impulse Rejection**: Excellent rejection of outliers and impulse noise, making it
 *          ideal for sensor data with occasional spikes. However, introduces group delay of
 *          approximately (FILTER_SIZE-1)/2 samples for step responses.
 *          
 *          **Usage Example**:
 *          @code
 *          ModeFilter<float, 5> median_filter(2);  // Return element 2 (median of 5 samples)
 *          float filtered = median_filter.apply(raw_sensor_value);
 *          @endcode
 * 
 * @tparam T Data type for filter samples (int8_t, uint8_t, int16_t, uint16_t, or float)
 * @tparam FILTER_SIZE Number of samples in buffer; recommend odd sizes (3,5,7) for true median
 * 
 * @warning Even-sized buffers (4, 6) don't have a true median. Use return_element = FILTER_SIZE/2
 *          for best median approximation with even sizes.
 * @warning Median filtering introduces group delay of (FILTER_SIZE-1)/2 samples, which delays
 *          step responses and phase-shifts frequency components.
 * 
 * @note Extensive typedef instantiations are provided for common type/size combinations:
 *       int8_t, uint8_t, int16_t, uint16_t, and float with sizes 3-7.
 * 
 * Source: libraries/Filter/ModeFilter.h:1-74
 */
template <class T, uint8_t FILTER_SIZE>
class ModeFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    /**
     * @brief Construct a new Mode Filter object
     * 
     * @details Initializes the mode filter with a specified return element index that
     *          determines which ranked sample from the sorted buffer will be returned.
     *          
     *          Common configurations:
     *          - return_element = 0: Returns minimum value (lowest in buffer)
     *          - return_element = FILTER_SIZE/2: Returns median value (middle of buffer)
     *          - return_element = FILTER_SIZE-1: Returns maximum value (highest in buffer)
     *          
     *          For true median filtering with odd-sized buffers (e.g., size 5), use
     *          return_element = FILTER_SIZE/2 = 2 to select the exact middle element.
     * 
     * @param[in] return_element Index of element to return from sorted buffer (0 to FILTER_SIZE-1)
     *                           - 0 returns minimum
     *                           - FILTER_SIZE/2 returns median
     *                           - FILTER_SIZE-1 returns maximum
     */
    ModeFilter(uint8_t return_element);

    /**
     * @brief Add a new raw sample to the filter and retrieve the filtered result
     * 
     * @details Inserts the new sample into the sorted buffer using insertion sort (isort()),
     *          maintaining buffer size by alternately dropping the highest or lowest existing
     *          sample. Returns the element at the rank specified by return_element.
     *          
     *          The filter maintains a sorted buffer without complete re-sorting, providing
     *          O(FILTER_SIZE) complexity per sample instead of O(FILTER_SIZE * log(FILTER_SIZE)).
     * 
     * @param[in] sample New raw value to add to the filter
     * 
     * @return T Filtered result: the element at rank return_element from the sorted buffer
     *           (typically the median when return_element = FILTER_SIZE/2)
     */
    virtual T        apply(T sample) override;

    /**
     * @brief Get the latest filtered value from the filter
     * 
     * @details Returns the most recently computed filtered output, which is the value
     *          returned by the most recent call to apply(). This allows retrieving the
     *          current filter output without adding a new sample.
     * 
     * @return T Latest filtered value (equal to the value returned by latest apply() call)
     */
    virtual T        get() const {
        return _output;
    }

private:
    // private methods
    uint8_t         _return_element;
    T               _output;
    void            isort(T sample, bool drop_high_sample);
    bool            drop_high_sample; // switch to determine whether to drop the highest or lowest sample when new value arrives
};

/**
 * @name Mode Filter Type Definitions
 * @brief Convenient typedefs for common ModeFilter instantiations
 * 
 * @details Pre-defined type aliases for frequently used combinations of data types
 *          (int8_t, uint8_t, int16_t, uint16_t, float) and filter sizes (3-7).
 *          These typedefs simplify declaration and improve code readability.
 *          
 *          Naming convention: ModeFilter{Type}_Size{N}
 *          - Type: Int8, UInt8, Int16, UInt16, or Float
 *          - N: Buffer size (3, 4, 5, 6, or 7)
 *          
 *          Odd sizes (3, 5, 7) are recommended for true median filtering.
 * @{
 */
typedef ModeFilter<int8_t,3> ModeFilterInt8_Size3;
typedef ModeFilter<int8_t,4> ModeFilterInt8_Size4;
typedef ModeFilter<int8_t,5> ModeFilterInt8_Size5;
typedef ModeFilter<int8_t,6> ModeFilterInt8_Size6;
typedef ModeFilter<int8_t,7> ModeFilterInt8_Size7;
typedef ModeFilter<uint8_t,3> ModeFilterUInt8_Size3;
typedef ModeFilter<uint8_t,4> ModeFilterUInt8_Size4;
typedef ModeFilter<uint8_t,5> ModeFilterUInt8_Size5;
typedef ModeFilter<uint8_t,6> ModeFilterUInt8_Size6;
typedef ModeFilter<uint8_t,7> ModeFilterUInt8_Size7;
typedef ModeFilter<int16_t,3> ModeFilterInt16_Size3;
typedef ModeFilter<int16_t,4> ModeFilterInt16_Size4;
typedef ModeFilter<int16_t,5> ModeFilterInt16_Size5;
typedef ModeFilter<int16_t,6> ModeFilterInt16_Size6;
typedef ModeFilter<int16_t,7> ModeFilterInt16_Size7;
typedef ModeFilter<uint16_t,3> ModeFilterUInt16_Size3;
typedef ModeFilter<uint16_t,4> ModeFilterUInt16_Size4;
typedef ModeFilter<uint16_t,5> ModeFilterUInt16_Size5;
typedef ModeFilter<uint16_t,6> ModeFilterUInt16_Size6;
typedef ModeFilter<uint16_t,7> ModeFilterUInt16_Size7;
typedef ModeFilter<float,3> ModeFilterFloat_Size3;
typedef ModeFilter<float,4> ModeFilterFloat_Size4;
typedef ModeFilter<float,5> ModeFilterFloat_Size5;
typedef ModeFilter<float,6> ModeFilterFloat_Size6;
typedef ModeFilter<float,7> ModeFilterFloat_Size7;
/** @} */ // End of Mode Filter Type Definitions group
