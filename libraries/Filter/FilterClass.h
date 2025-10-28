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
 * @file FilterClass.h
 * @brief Pure virtual interface defining the contract for all ArduPilot filters
 * 
 * @details This header defines Filter<T>, the abstract base class that provides
 *          a uniform API for all filtering operations in ArduPilot. All concrete
 *          filter implementations (low-pass, notch, average, derivative, etc.)
 *          inherit from this template class and implement its pure virtual methods.
 * 
 *          Purpose: Establish consistent interface for filter operations across
 *                   the entire ArduPilot codebase, enabling polymorphic filter usage
 * 
 *          Design Pattern: Template-based pure virtual interface
 *          - Templates enable type-safe filtering for scalars and vectors
 *          - Pure virtual methods enforce implementation by subclasses
 *          - Minimal interface (apply, reset) keeps filters lightweight
 * 
 *          Role in ArduPilot: Central abstraction for all digital filtering needs
 *          - Sensor data filtering (IMU, compass, barometer, GPS)
 *          - Control signal smoothing (pilot inputs, attitude setpoints)
 *          - Noise reduction in telemetry and logging
 *          - Derivative and rate estimation
 * 
 * @note This is a header-only library with template implementations
 * @see FilterWithBuffer<T, SIZE> for filters requiring delay element storage
 * @see LowPassFilter for first-order low-pass implementation
 * @see NotchFilter for second-order notch filter implementation
 */
#pragma once

#include <inttypes.h>

/**
 * @class Filter
 * @brief Pure virtual base class (interface) for all filter implementations in ArduPilot
 * 
 * @details Filter<T> defines the minimal interface that all filters must implement:
 *          - apply(): Process a new sample and return filtered output
 *          - reset(): Clear internal state (delay elements, history buffers, accumulators)
 * 
 *          This abstract class enables polymorphic filter usage throughout ArduPilot.
 *          Code can work with Filter<T>* pointers without knowing the specific filter
 *          type (low-pass, notch, average, etc.), enabling runtime filter selection
 *          and configuration.
 * 
 *          Design Philosophy:
 *          - Minimal Interface: Only apply() and reset() are universally required
 *          - Type Safety: Template parameter T ensures compile-time type checking
 *          - Zero Overhead: Pure virtual functions with template specialization
 *            enable aggressive compiler optimization
 *          - Extensibility: New filter types simply inherit and implement these methods
 * 
 *          Inheritance Hierarchy:
 *          - Filter<T> (this class) - Pure virtual interface
 *          - FilterWithBuffer<T, SIZE> - Adds circular buffer management for filters
 *            requiring delay elements (moving average, FIR, derivative, etc.)
 *          - Concrete filters - Implement specific algorithms:
 *            * LowPassFilter, LowPassFilter2p - First and second-order low-pass
 *            * NotchFilter - Second-order notch/band-stop filter
 *            * AverageFilter, ModeFilter - Statistical filters
 *            * DerivativeFilter - Numerical differentiation with filtering
 *            * HarmonicNotchFilter - Multi-frequency notch for motor noise rejection
 * 
 *          Typical Usage Pattern:
 *          @code
 *          // Create filter instance with concrete type
 *          LowPassFilterFloat my_filter;
 *          
 *          // Configure filter parameters (cutoff frequency, sample rate, etc.)
 *          my_filter.set_cutoff_frequency(sample_rate_hz, cutoff_freq_hz);
 *          
 *          // In main loop: filter incoming samples
 *          float raw_sensor_value = sensor.read();
 *          float filtered_value = my_filter.apply(raw_sensor_value);
 *          
 *          // When filter state must be cleared (mode change, sensor swap, etc.)
 *          my_filter.reset();
 *          @endcode
 * 
 *          Thread Safety: Filters are NOT thread-safe by default. If a filter is
 *          accessed from multiple threads or interrupt contexts, external
 *          synchronization (semaphores) must be used.
 * 
 *          Performance: apply() is typically called at high rates (100Hz-8kHz)
 *          depending on the application. Implementations must be computationally
 *          efficient. Most filters execute in <10 CPU cycles on ARM Cortex-M7.
 * 
 * @tparam T Data type of samples being filtered
 *           Supported types:
 *           - Scalar integers: int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t
 *           - Scalar floating-point: float (most common)
 *           - Vectors: Vector2f, Vector3f (from AP_Math library)
 *           
 *           Subclasses provide explicit template instantiations for supported types.
 *           Vector filtering applies the same filter coefficients independently to
 *           each vector component (element-wise filtering).
 * 
 * @note This class cannot be instantiated directly (pure virtual interface)
 * @warning Subclass implementations must ensure apply() is reentrant if called
 *          from multiple contexts (main loop + interrupt)
 * 
 * @see FilterWithBuffer<T, SIZE> for filters requiring delay element storage
 * @see libraries/Filter/LowPassFilter.h for first-order low-pass implementation
 * @see libraries/Filter/NotchFilter.h for second-order notch implementation
 */
template <class T>
class Filter
{
public:
    /**
     * @brief Apply filter to a new sample and retrieve the filtered result
     * 
     * @details This is the core filtering operation. Implementations update internal
     *          state (delay elements, accumulators, etc.) and return the filtered output.
     *          
     *          The method must be pure virtual (= 0) to enforce implementation by all
     *          subclasses. Each filter type implements its specific algorithm here.
     *          
     *          Calling Frequency: Typically called at regular intervals matching the
     *          sample rate used during filter configuration. Irregular sampling may
     *          degrade filter performance depending on the filter type.
     *          
     *          Algorithm Examples by Filter Type:
     *          - LowPassFilter: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
     *          - NotchFilter: Second-order IIR difference equation
     *          - AverageFilter: Sum of last N samples divided by N
     *          - DerivativeFilter: (x[n] - x[n-1]) / dt with filtering
     * 
     * @param[in] sample New raw input value to be filtered
     *                   Units depend on application (m/s², rad/s, Pa, etc.)
     *                   For vector filters, all components processed independently
     * 
     * @return Filtered output value in the same units as input
     *         The return value represents the filter's current estimate of the
     *         true signal after removing noise, transients, or unwanted frequency
     *         components according to the specific filter's design
     * 
     * @note PURE VIRTUAL - Must be implemented by all subclasses
     * @warning Implementation must not block or take excessive time (called at high rate)
     * @warning For vector types, ensure all components are filtered consistently
     */
    virtual T apply(T sample) = 0;

    /**
     * @brief Reset filter to initial state, clearing all internal history
     * 
     * @details Clears all internal state including:
     *          - Delay elements (previous input/output samples)
     *          - History buffers (circular buffers, sample arrays)
     *          - Accumulators (running sums, squared sums)
     *          - State variables (filter states in state-space representation)
     *          
     *          After reset(), the next apply() call effectively starts filtering
     *          from scratch with no memory of previous samples.
     *          
     *          When to Call reset():
     *          - Sensor calibration starts (clear old sensor data)
     *          - Flight mode changes requiring fresh estimates
     *          - Sensor failover occurs (switching to backup sensor)
     *          - EKF reset or position reset
     *          - Parameter changes affecting filter configuration
     *          - Manual user request (via MAVLink command or parameter)
     *          
     *          Transient Behavior: The first few samples after reset() may exhibit
     *          transient behavior as the filter "fills up" with new data. This is
     *          normal and depends on filter order and type.
     * 
     * @note PURE VIRTUAL - Must be implemented by all subclasses
     * @note Implementation should execute quickly (typically <1 microsecond)
     * @warning Do not call reset() unnecessarily during normal operation as it
     *          discards valuable filtering history and may cause temporary
     *          noise spikes in the output
     * 
     * @see apply() for the filtering operation
     */
    virtual void reset()  = 0;

};

/**
 * @name Convenience Type Definitions
 * @brief Predefined filter interface types for common integer data types
 * 
 * @details These typedefs provide convenient shorthand for Filter<T> instantiations
 *          with standard integer types. While most ArduPilot filtering uses float
 *          (FilterFloat defined in concrete filter implementations), integer filters
 *          are useful for:
 *          
 *          Use Cases for Integer Filters:
 *          - Memory-constrained systems (integer operations faster on some MCUs)
 *          - Fixed-point arithmetic in resource-limited environments
 *          - Filtering discrete values (mode detection, classification)
 *          - RC input PWM values (typically uint16_t microseconds)
 *          - Sensor raw counts before conversion to physical units
 *          
 *          Note: Most concrete filter implementations (LowPassFilter, NotchFilter, etc.)
 *          primarily focus on float and Vector types. Integer filter support varies
 *          by filter type. Check specific filter documentation for supported types.
 * 
 * @see Filter<T> for the base template interface
 * @see LowPassFilter.h, NotchFilter.h for concrete implementations with float typedefs
 * @{
 */

/** @brief Filter interface for signed 8-bit integer samples (-128 to 127) */
typedef Filter<int8_t> FilterInt8;

/** @brief Filter interface for unsigned 8-bit integer samples (0 to 255) */
typedef Filter<uint8_t> FilterUInt8;

/** @brief Filter interface for signed 16-bit integer samples (-32768 to 32767) */
typedef Filter<int16_t> FilterInt16;

/** @brief Filter interface for unsigned 16-bit integer samples (0 to 65535)
 *  @note Common for RC input PWM values (typically 1000-2000 microseconds) */
typedef Filter<uint16_t> FilterUInt16;

/** @brief Filter interface for signed 32-bit integer samples (-2^31 to 2^31-1) */
typedef Filter<int32_t> FilterInt32;

/** @brief Filter interface for unsigned 32-bit integer samples (0 to 2^32-1) */
typedef Filter<uint32_t> FilterUInt32;

/** @} */ // End of Convenience Type Definitions group
