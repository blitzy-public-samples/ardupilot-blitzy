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
 * @file FilterWithBuffer.h
 * @brief Base class providing circular buffer management for filters requiring sample history
 * 
 * @details This class extends Filter<T> with a circular buffer to support filters that need
 *          to access multiple historical samples (e.g., moving average, mode filter, derivative).
 *          
 *          Template Restriction Workaround:
 *          This separate class hierarchy exists to handle different buffer sizes as distinct types.
 *          C++ template restrictions prevent Filter<T> from having variable-size buffers directly,
 *          as each template instantiation with a different FILTER_SIZE becomes a completely
 *          different type. This intermediate FilterWithBuffer class provides buffer management
 *          as a base for size-specific filter implementations.
 *          
 *          Design Pattern:
 *          Uses template-based static polymorphism for zero-overhead circular buffer management.
 *          The circular buffer automatically wraps sample_index from FILTER_SIZE-1 to 0.
 *          
 *          Subclasses:
 *          - AverageFilter: Computes moving average over buffered samples
 *          - ModeFilter: Finds most common value in buffered samples
 *          - DerivativeFilter: Computes rate of change from buffered samples
 *          
 *          Typedef Instantiations:
 *          Provides extensive typedef instantiations for common type/size combinations:
 *          int16_t, uint16_t, int32_t, uint32_t with buffer sizes 2-7.
 * 
 * Source: libraries/Filter/FilterWithBuffer.h:1-116
 */
#pragma once

#include "FilterClass.h"

/**
 * @class FilterWithBuffer
 * @brief Filter base class with circular buffer for storing sample history
 * 
 * @details Extends Filter<T> with automatic circular buffer management for filters that need
 *          to access multiple historical samples. This base class handles the bookkeeping of
 *          storing samples in a circular buffer and wrapping the index automatically.
 *          
 *          Purpose:
 *          Provides a samples[] array and sample_index for automatic circular buffer management.
 *          The base implementation of apply() simply stores the sample without filtering -
 *          subclasses must override apply() to implement actual filtering logic.
 *          
 *          Buffer Management:
 *          - Maintains fixed-size circular buffer of FILTER_SIZE samples
 *          - Automatically wraps sample_index from FILTER_SIZE-1 to 0
 *          - reset() clears all samples to zero (T(0))
 *          - get_sample(i) provides read access to any buffered sample
 *          
 *          Template Parameters:
 *          @tparam T Sample data type (typically int16_t, uint16_t, int32_t, uint32_t, or float)
 *          @tparam FILTER_SIZE Number of samples in circular buffer (compile-time constant)
 *          
 *          Design Rationale:
 *          Template restrictions prevent Filter<T> from having variable-size buffers, so this
 *          intermediate class provides buffer management as a distinct type for each size.
 *          This enables compile-time optimization while supporting different buffer sizes.
 *          
 *          Usage Pattern:
 *          Subclasses inherit from FilterWithBuffer<T, FILTER_SIZE> and override apply() to
 *          implement specific filtering algorithms using the buffered samples.
 *          
 * @warning Subclasses must override apply() to implement actual filtering logic.
 *          The base implementation only stores samples and returns them unfiltered.
 * 
 * @note Buffer initialization: reset() sets all samples to zero (T(0)), which may not be
 *       appropriate for all data types. Subclasses may need to override reset() for
 *       special initialization requirements.
 */
template <class T, uint8_t FILTER_SIZE>
class FilterWithBuffer : public Filter<T>
{
public:
    /**
     * @brief Constructor - initializes circular buffer management
     * 
     * @details Initializes sample_index to 0 and calls reset() to clear all buffered samples.
     *          After construction, the buffer is ready to accept samples via apply().
     */
    FilterWithBuffer();

    /**
     * @brief Add a new raw sample to the circular buffer
     * 
     * @details Base implementation stores the sample in the circular buffer and advances
     *          the sample_index with automatic wrapping. Returns the sample unfiltered.
     *          
     *          Circular Buffer Behavior:
     *          - Stores sample at samples[sample_index]
     *          - Increments sample_index
     *          - Wraps sample_index from FILTER_SIZE-1 to 0 automatically
     *          
     *          Base Class Behavior:
     *          This base implementation does not perform filtering - it only manages the buffer.
     *          Subclasses override this method to implement actual filtering algorithms using
     *          the buffered samples.
     * 
     * @param[in] sample New raw sample value to add to the buffer
     * @return Sample value unfiltered (base implementation returns input sample)
     * 
     * @warning Subclasses must override this method to implement filtering logic.
     *          The base implementation only provides buffer management.
     */
    virtual T apply(T sample) override;

    /**
     * @brief Clear all samples from the buffer and reset to initial state
     * 
     * @details Sets all samples in the buffer to zero (T(0)) and resets sample_index to 0.
     *          After reset(), the filter is in the same state as after construction.
     *          
     *          Buffer Initialization:
     *          All elements of samples[] array are set to zero using assignment: samples[i] = 0.
     *          For numeric types this produces zero value, but behavior depends on T's
     *          conversion from integer 0.
     * 
     * @note May not be appropriate for all data types if zero is not a valid neutral value.
     *       Subclasses can override reset() for custom initialization.
     */
    virtual void reset() override;

    /**
     * @brief Get the compile-time buffer size
     * 
     * @return Buffer size (FILTER_SIZE template parameter)
     * 
     * @note This is a compile-time constant, not the number of samples currently in buffer.
     */
    uint8_t get_filter_size() const {
        return FILTER_SIZE;
    };

    /**
     * @brief Access a sample from the circular buffer by index
     * 
     * @details Provides read-only access to any sample in the buffer. Index is relative
     *          to the samples[] array, not relative to insertion order. To access samples
     *          in chronological order, caller must account for sample_index wrapping.
     * 
     * @param[in] i Index into samples[] array (must be 0 <= i < FILTER_SIZE)
     * @return Sample at index i in the buffer
     * 
     * @warning Does NOT bounds-check index. Caller must ensure 0 <= i < FILTER_SIZE
     *          to avoid undefined behavior. Out-of-bounds access will read/write
     *          adjacent memory.
     */
    T get_sample(uint8_t i) const {
        return samples[i];
    }

protected:
    T               samples[FILTER_SIZE];       ///< Circular buffer of samples (stores FILTER_SIZE historical samples)
    uint8_t         sample_index;               ///< Index to next buffer slot to write (wraps from FILTER_SIZE-1 to 0)
};

/**
 * Typedef instantiations for common type and size combinations
 * 
 * These typedefs provide convenient names for commonly used FilterWithBuffer instantiations.
 * Each template instantiation with different parameters creates a distinct type, so these
 * typedefs reduce verbosity in code and ensure consistent type usage.
 * 
 * Supported types: int16_t, uint16_t, int32_t, uint32_t
 * Supported sizes: 2, 3, 4, 5, 6, 7 samples
 * 
 * Naming convention: FilterWithBuffer{Type}_Size{N}
 * Example: FilterWithBufferInt16_Size5 = FilterWithBuffer<int16_t, 5>
 */
typedef FilterWithBuffer<int16_t,2> FilterWithBufferInt16_Size2;
typedef FilterWithBuffer<int16_t,3> FilterWithBufferInt16_Size3;
typedef FilterWithBuffer<int16_t,4> FilterWithBufferInt16_Size4;
typedef FilterWithBuffer<int16_t,5> FilterWithBufferInt16_Size5;
typedef FilterWithBuffer<int16_t,6> FilterWithBufferInt16_Size6;
typedef FilterWithBuffer<int16_t,7> FilterWithBufferInt16_Size7;
typedef FilterWithBuffer<uint16_t,2> FilterWithBufferUInt16_Size2;
typedef FilterWithBuffer<uint16_t,3> FilterWithBufferUInt16_Size3;
typedef FilterWithBuffer<uint16_t,4> FilterWithBufferUInt16_Size4;
typedef FilterWithBuffer<uint16_t,5> FilterWithBufferUInt16_Size5;
typedef FilterWithBuffer<uint16_t,6> FilterWithBufferUInt16_Size6;
typedef FilterWithBuffer<uint16_t,7> FilterWithBufferUInt16_Size7;

typedef FilterWithBuffer<int32_t,2> FilterWithBufferInt32_Size2;
typedef FilterWithBuffer<int32_t,3> FilterWithBufferInt32_Size3;
typedef FilterWithBuffer<int32_t,4> FilterWithBufferInt32_Size4;
typedef FilterWithBuffer<int32_t,5> FilterWithBufferInt32_Size5;
typedef FilterWithBuffer<int32_t,6> FilterWithBufferInt32_Size6;
typedef FilterWithBuffer<int32_t,7> FilterWithBufferInt32_Size7;
typedef FilterWithBuffer<uint32_t,2> FilterWithBufferUInt32_Size2;
typedef FilterWithBuffer<uint32_t,3> FilterWithBufferUInt32_Size3;
typedef FilterWithBuffer<uint32_t,4> FilterWithBufferUInt32_Size4;
typedef FilterWithBuffer<uint32_t,5> FilterWithBufferUInt32_Size5;
typedef FilterWithBuffer<uint32_t,6> FilterWithBufferUInt32_Size6;
typedef FilterWithBuffer<uint32_t,7> FilterWithBufferUInt32_Size7;

/**
 * @brief Constructor implementation
 * 
 * @details Initializes sample_index to 0 in the initializer list, then calls reset()
 *          to clear all buffer elements to zero. This ensures the buffer is in a known
 *          initial state with no residual data.
 */
template <class T, uint8_t FILTER_SIZE>
FilterWithBuffer<T,FILTER_SIZE>::FilterWithBuffer() :
    sample_index(0)
{
    // Clear sample buffer to initial state (all zeros)
    reset();
}

/**
 * @brief Reset implementation - clears buffer and resets index
 * 
 * @details Loops through entire samples[] array setting each element to 0 (zero of type T).
 *          Then resets sample_index to 0 to restart circular buffer from the beginning.
 *          After reset(), the next apply() call will write to samples[0].
 */
template <class T, uint8_t FILTER_SIZE>
void FilterWithBuffer<T,FILTER_SIZE>::reset()
{
    // Clear all samples in buffer to zero
    for( int8_t i=0; i<FILTER_SIZE; i++ ) {
        samples[i] = 0;
    }

    // Reset circular buffer index back to beginning
    sample_index = 0;
}

/**
 * @brief Apply implementation - stores sample in circular buffer
 * 
 * @details Base class implementation that manages circular buffer without filtering:
 *          1. Stores sample at current sample_index position
 *          2. Increments sample_index (post-increment)
 *          3. Wraps sample_index to 0 if it reaches FILTER_SIZE (modulo behavior)
 *          4. Returns the raw sample unfiltered
 *          
 *          Circular Buffer Wrap:
 *          Uses modulo arithmetic via if-statement to wrap index from FILTER_SIZE-1 to 0.
 *          This ensures sample_index always stays in valid range [0, FILTER_SIZE-1].
 *          
 *          Subclass Override:
 *          Subclasses override this method to implement filtering algorithms using the
 *          buffered samples, typically computing some function of samples[] array.
 */
template <class T, uint8_t FILTER_SIZE>
T FilterWithBuffer<T,FILTER_SIZE>::        apply(T sample)
{
    // Store sample at current index, then increment (post-increment)
    samples[sample_index++] = sample;

    // Wrap index if it reached end of buffer (modulo FILTER_SIZE behavior)
    if( sample_index >= FILTER_SIZE )
        sample_index = 0;

    // Base class doesn't implement filtering - just return raw sample
    // Subclasses override to compute filtered output from samples[] array
    return sample;
}
