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
 * @file HarmonicNotchFilter.h
 * @brief Harmonic notch filter bank management for multi-frequency rejection
 * 
 * @details This file implements a harmonic notch filter system that manages
 *          a bank of biquad notch filters targeting a fundamental frequency
 *          and its harmonics. The filter is designed for dynamic noise rejection
 *          in flight control systems, particularly for suppressing rotor noise
 *          and vibrations that occur at multiples of fundamental frequencies.
 *          
 *          Key features:
 *          - Dynamic frequency tracking (throttle, RPM, ESC telemetry, gyro FFT)
 *          - Support for up to 16 harmonics (HNF_MAX_HARMONICS)
 *          - Composite notch configurations (single, double, triple)
 *          - Runtime filter allocation and expansion
 *          - Telemetry logging via FCN messages
 *          
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/Filter/HarmonicNotchFilter.h
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <AP_Param/AP_Param.h>
#include "NotchFilter.h"

#define HNF_MAX_HARMONICS 16

class HarmonicNotchFilterParams;

/**
 * @class HarmonicNotchFilter
 * @brief Manages a bank of notch filters targeting fundamental frequency and harmonics
 * 
 * @details This template class implements a harmonic notch filter system that allocates
 *          and manages multiple biquad notch filters arranged in cascade. Each filter
 *          targets either the fundamental frequency or a harmonic multiple of it.
 *          
 *          **Algorithm Overview:**
 *          - Allocates a bank of NotchFilter<T> instances based on configuration
 *          - Supports composite notches: single, double (2x), or triple (3x) per harmonic
 *          - Calculates harmonic frequencies as: center_freq_hz * harmonic_multiplier
 *          - Cascades all filters: output of filter[n] becomes input to filter[n+1]
 *          - Dynamically tracks frequency changes via update() methods
 *          
 *          **Dynamic Tracking Modes:**
 *          - Fixed: Static center frequency (no tracking)
 *          - UpdateThrottle: Frequency varies with throttle input
 *          - UpdateRPM: Frequency tracks RPM sensor data
 *          - UpdateBLHeli: Frequency from BLHeli ESC telemetry
 *          - UpdateGyroFFT: Frequency derived from gyro FFT analysis
 *          - UpdateRPM2: Secondary RPM sensor tracking
 *          
 *          **Composite Notch Spreading:**
 *          When using double or triple notches, additional filters are placed at
 *          frequencies offset by a spreading factor to widen the rejection band.
 *          
 *          **Harmonic Configuration:**
 *          The harmonics parameter is a bitmask where bit N enables the Nth harmonic.
 *          For example, harmonics=0x05 (binary 0101) enables 1st and 3rd harmonics.
 *          Maximum of HNF_MAX_HARMONICS (16) harmonic multipliers are supported.
 *          
 *          **Telemetry:**
 *          Logs FCN (Filter Center Notch) messages containing center frequencies
 *          of all active notch filters for ground station monitoring and analysis.
 * 
 * @tparam T Data type for filter (typically float or Vector3f for gyro data)
 * 
 * @warning Filter allocation uses heap memory. If expand_filter_count() fails due to
 *          heap exhaustion, _alloc_has_failed flag is set and no additional filters
 *          are allocated. Check this flag after allocation attempts.
 * 
 * @warning All center_freq_hz parameters must be less than the Nyquist frequency
 *          (sample_freq_hz / 2) to avoid aliasing. Violating this constraint results
 *          in undefined filter behavior and potential instability.
 * 
 * @note This filter is computationally expensive. Each apply() call processes the
 *       sample through all allocated filters in cascade. Typical configurations use
 *       8-32 filters depending on harmonics and composite notch settings.
 * 
 * Source: libraries/Filter/HarmonicNotchFilter.h:30-93
 */
template <class T>
class HarmonicNotchFilter {
public:
    /**
     * @brief Destructor - deallocates filter bank
     */
    ~HarmonicNotchFilter();
    
    /**
     * @brief Allocate a bank of notch filters for this harmonic notch filter
     * 
     * @details Allocates num_notches NotchFilter<T> instances multiplied by the number
     *          of enabled harmonics and composite notch count. For example, with
     *          num_notches=2, harmonics=0x03 (2 harmonics), and composite_notches=2,
     *          this allocates 2 * 2 * 2 = 8 total filters.
     *          
     *          The harmonics bitmask determines which harmonic multiples are enabled.
     *          Bit position N corresponds to harmonic multiplier (N+1). For instance:
     *          - harmonics=0x01 (bit 0): fundamental only (1x)
     *          - harmonics=0x03 (bits 0,1): fundamental + 2nd harmonic (1x, 2x)
     *          - harmonics=0x15 (bits 0,2,4): 1st, 3rd, 5th harmonics (1x, 3x, 5x)
     * 
     * @param[in] num_notches Number of base notch filters to allocate per harmonic
     * @param[in] harmonics Bitmask of enabled harmonics (bit N = harmonic N+1)
     * @param[in] composite_notches Number of composite notches (1=single, 2=double, 3=triple)
     * 
     * @note Memory is allocated from heap using NEW_NOTHROW. Check _alloc_has_failed
     *       after calling if allocation success is critical.
     * @note Total filters allocated = num_notches * num_harmonics * composite_notches
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:35
     */
    void allocate_filters(uint8_t num_notches, uint32_t harmonics, uint8_t composite_notches);
    /**
     * @brief Expand filter bank with additional filters
     * 
     * @details Expands the existing filter bank to the specified total count by
     *          allocating additional NotchFilter<T> instances. This is used when
     *          the required number of filters increases dynamically, such as when
     *          multiple tracking sources become active simultaneously.
     *          
     *          If the new total is less than or equal to the current filter count,
     *          no action is taken. If heap allocation fails during expansion,
     *          _alloc_has_failed is set to true and the filter count remains unchanged.
     * 
     * @param[in] total_notches Total number of filters to expand to (not additional count)
     * 
     * @warning Can fail if heap memory is exhausted. Check _alloc_has_failed flag
     *          after calling to detect allocation failures. Failed expansion leaves
     *          the existing filter bank intact.
     * 
     * @note This performs reallocation and copying of existing filter states to
     *       the new array, which may be computationally expensive.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:37
     */
    void expand_filter_count(uint16_t total_notches);
    /**
     * @brief Initialize the underlying filters using the provided filter parameters
     * 
     * @details Initializes all allocated NotchFilter<T> instances with the specified
     *          sampling frequency and extracts configuration from the params object:
     *          - Center frequency (Hz)
     *          - Bandwidth (Hz)
     *          - Attenuation (dB)
     *          - Quality factor Q
     *          - Notch spread for composite notches
     *          
     *          This method must be called after allocate_filters() and before any
     *          apply() operations. The sample frequency determines the discrete-time
     *          biquad filter coefficients.
     * 
     * @param[in] sample_freq_hz Sampling frequency in Hertz (typically 400Hz-8000Hz for IMU data)
     * @param[in] params Reference to HarmonicNotchFilterParams containing filter configuration
     * 
     * @warning sample_freq_hz must be at least 2x the highest notch center frequency
     *          (Nyquist criterion) to avoid aliasing and filter instability.
     * 
     * @note Sets _initialised flag to true on successful initialization
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:39
     */
    void init(float sample_freq_hz, HarmonicNotchFilterParams &params);
    /**
     * @brief Update all filter center frequencies from a single fundamental frequency
     * 
     * @details Recalculates center frequencies for all allocated filters based on a
     *          single fundamental frequency. Harmonic filters are set to integer
     *          multiples of this fundamental: f_harmonic = center_freq_hz * harmonic_mul.
     *          
     *          Composite notches (double/triple) are spread around each harmonic center
     *          using the configured spread factor to widen the rejection band.
     *          
     *          This overload is used when tracking a single noise source (e.g., single
     *          motor, throttle-based tracking, or single RPM sensor).
     * 
     * @param[in] center_freq_hz Fundamental center frequency in Hertz
     * 
     * @warning center_freq_hz must be less than Nyquist frequency (sample_freq_hz / 2)
     *          and greater than zero. Invalid frequencies result in filter instability.
     * 
     * @note Called at loop rate (typically 400Hz) when dynamic tracking is enabled.
     *       Frequency clamped to minimum frequency if below threshold.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:41
     */
    void update(float center_freq_hz);
    
    /**
     * @brief Update filter center frequencies from multiple independent sources
     * 
     * @details Sets center frequencies for filters tracking multiple independent noise
     *          sources simultaneously (e.g., multi-motor tracking, multiple RPM sensors).
     *          Each source in the center_freq_hz array represents an independent fundamental
     *          frequency, and harmonics are calculated for each.
     *          
     *          The filter bank is divided among the sources, with harmonics and composite
     *          notches allocated for each source frequency.
     * 
     * @param[in] num_centers Number of independent center frequencies in the array
     * @param[in] center_freq_hz Array of fundamental center frequencies in Hertz
     * 
     * @warning Each frequency in center_freq_hz[] must satisfy Nyquist criterion.
     *          num_centers must not exceed the allocated filter bank capacity.
     * 
     * @note Typically used with multi-motor configurations or when combining multiple
     *       tracking modes (e.g., ESC telemetry from multiple motors).
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:43
     */
    void update(uint8_t num_centers, const float center_freq_hz[]);

    /**
     * @brief Set center frequency of a single notch filter in the bank
     * 
     * @details Directly configures the center frequency of one specific filter in the
     *          allocated bank. This low-level method is called by update() to distribute
     *          frequencies across the filter bank.
     *          
     *          The spread_mul parameter offsets the frequency for composite notch spreading:
     *          - spread_mul = 1.0: no offset (center of harmonic)
     *          - spread_mul < 1.0: frequency below harmonic center
     *          - spread_mul > 1.0: frequency above harmonic center
     *          
     *          The harmonic_mul determines which harmonic this filter targets:
     *          - harmonic_mul = 1: fundamental frequency
     *          - harmonic_mul = 2: second harmonic (2x fundamental)
     *          - harmonic_mul = 3: third harmonic (3x fundamental), etc.
     * 
     * @param[in] idx Index of the filter in the bank (0 to _num_filters-1)
     * @param[in] center_freq_hz Base center frequency in Hertz before spreading
     * @param[in] spread_mul Spreading multiplier for composite notches (typically 0.5 to 1.5)
     * @param[in] harmonic_mul Harmonic multiplier (1 = fundamental, 2 = 2nd harmonic, etc.)
     * 
     * @warning idx must be less than _num_filters or memory corruption will occur.
     *          No bounds checking is performed for performance reasons.
     * @warning Final frequency (center_freq_hz * spread_mul * harmonic_mul) must be
     *          less than Nyquist frequency.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:50
     */
    void set_center_frequency(uint16_t idx, float center_freq_hz, float spread_mul, uint8_t harmonic_mul);

    /**
     * @brief Apply a sample through all filters in cascade
     * 
     * @details Processes the input sample through all allocated NotchFilter<T> instances
     *          in series (cascade configuration). The output of filter[n] becomes the
     *          input to filter[n+1], providing cumulative notch rejection at all
     *          configured frequencies.
     *          
     *          This is the main filtering operation, called at sensor sample rate
     *          (typically 1kHz-8kHz for gyros). Computational cost scales linearly
     *          with the number of allocated filters.
     * 
     * @param[in] sample Input sample (raw sensor data or partially filtered data)
     * @return Filtered sample after cascading through all notch filters
     * 
     * @note For Vector3f (gyro data), all three axes are filtered identically with
     *       the same center frequencies. Each axis passes through the cascade independently.
     * @note Call update() before apply() when tracking dynamic frequencies to ensure
     *       notch centers are current.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:53
     */
    T apply(const T &sample);
    
    /**
     * @brief Reset all underlying filters to initial state
     * 
     * @details Clears the internal state (delay lines) of all NotchFilter<T> instances,
     *          removing any historical sample data. This is necessary when:
     *          - Sensor data stream is interrupted or invalid
     *          - Large discontinuity in input data occurs
     *          - Switching between flight modes with different filter requirements
     *          - Recovering from sensor errors or failures
     *          
     *          After reset(), filters restart from zero initial conditions.
     * 
     * @note Does not deallocate filters or change center frequencies, only clears state.
     * @note Typically called during sensor initialization or failure recovery.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:55
     */
    void reset();

    /**
     * @brief Log notch center frequencies for telemetry and analysis
     * 
     * @details Generates FCN (Filter Center Notch) telemetry messages containing the
     *          center frequencies of all active notch filters. This data is logged to
     *          onboard storage and can be streamed to ground control stations for
     *          real-time monitoring and post-flight analysis.
     *          
     *          Logged information includes:
     *          - Fundamental center frequency
     *          - First harmonic frequency (2x fundamental)
     *          - Filter instance identifier
     *          - Timestamp for correlation with flight events
     *          
     *          Useful for verifying dynamic tracking is working correctly and for
     *          tuning notch filter parameters based on actual noise frequencies.
     * 
     * @param[in] instance Filter instance identifier (0-based, for multiple filter banks)
     * @param[in] now_us Current timestamp in microseconds (for log message timestamp)
     * 
     * @note Called periodically (typically at 1Hz) to avoid excessive log bandwidth usage.
     * @note Only logs if filters are initialized and enabled.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:60
     */
    void log_notch_centers(uint8_t instance, uint64_t now_us) const;

private:
    // underlying bank of notch filters
    NotchFilter<T>*  _filters;
    // sample frequency for each filter
    float _sample_freq_hz;
    // base double notch bandwidth for each filter
    float _notch_spread;
    // attenuation for each filter
    float _A;
    // quality factor of each filter
    float _Q;
    // a bitmask of the harmonics to use
    uint32_t _harmonics;
    // number of notches that make up a composite notch
    uint8_t _composite_notches;
    // number of allocated filters
    uint16_t _num_filters;
    // pre-calculated number of harmonics
    uint8_t _num_harmonics;
    // number of enabled filters
    uint16_t _num_enabled_filters;
    bool _initialised;

    // have we failed to expand filters?
    bool _alloc_has_failed;

    // minimum frequency (from INS_HNTCH_FREQ * INS_HNTCH_FM_RAT)
    float _minimum_freq;

    // pointer to params object for this filter
    HarmonicNotchFilterParams *params;
};

/**
 * @enum HarmonicNotchDynamicMode
 * @brief Harmonic notch filter dynamic frequency tracking modes
 * 
 * @details Defines how the harmonic notch filter's center frequency is determined
 *          and updated during flight. Each mode tracks a different noise source
 *          or uses a different algorithm to estimate the fundamental frequency.
 * 
 * Source: libraries/Filter/HarmonicNotchFilter.h:96-103
 */
enum class HarmonicNotchDynamicMode {
    /**
     * @brief Fixed center frequency mode (no dynamic tracking)
     * 
     * Center frequency remains constant at the configured value (INS_HNTCH_FREQ).
     * Used when noise frequency is known and constant, or when dynamic tracking
     * is not required. Lowest computational cost.
     */
    Fixed           = 0,
    
    /**
     * @brief Update center frequency based on throttle input
     * 
     * Center frequency scales with throttle position: freq = base_freq * sqrt(throttle).
     * Assumes motor RPM (and associated noise) increases with throttle.
     * Reference throttle value (INS_HNTCH_REF) defines the throttle at which
     * base frequency occurs. Useful when RPM sensors are not available.
     */
    UpdateThrottle  = 1,
    
    /**
     * @brief Update center frequency from primary RPM sensor
     * 
     * Center frequency tracks RPM sensor data directly: freq = RPM / 60.
     * Provides accurate tracking for single motor/rotor configurations.
     * Requires RPM sensor to be configured and providing valid data.
     */
    UpdateRPM       = 2,
    
    /**
     * @brief Update center frequency from BLHeli ESC telemetry
     * 
     * Center frequency derived from ESC RPM telemetry via BLHeli passthrough protocol.
     * Supports multi-motor tracking by averaging or tracking individual motor frequencies.
     * Requires BLHeli-compatible ESCs with telemetry enabled.
     */
    UpdateBLHeli    = 3,
    
    /**
     * @brief Update center frequency from gyro FFT analysis
     * 
     * Center frequency determined by real-time FFT analysis of gyro data to detect
     * dominant noise peaks. Automatically identifies primary vibration frequencies.
     * Highest CPU cost but most adaptive. Requires AP_INERTIALSENSOR_HARMONICNOTCH_FFT.
     */
    UpdateGyroFFT   = 4,
    
    /**
     * @brief Update center frequency from secondary RPM sensor
     * 
     * Center frequency tracks a second RPM sensor, allowing independent tracking
     * of multiple rotors or noise sources. Used in conjunction with UpdateRPM
     * for dual-rotor configurations (e.g., coaxial helicopters, twin-engine planes).
     */
    UpdateRPM2      = 5,
};

/**
 * @class HarmonicNotchFilterParams
 * @brief Harmonic notch filter configuration parameters
 * 
 * @details Manages configuration parameters for the harmonic notch filter system,
 *          extending the base NotchFilterParams class with harmonic-specific settings.
 *          Parameters are stored using the AP_Param system for persistence and ground
 *          station configurability.
 *          
 *          Key configuration parameters:
 *          - Center frequency (Hz): Fundamental frequency or reference for tracking
 *          - Bandwidth (Hz): Width of each notch rejection band
 *          - Attenuation (dB): Depth of notch rejection (typical: 20-40 dB)
 *          - Harmonics bitmask: Which harmonic multiples to enable
 *          - Reference value: Calibration reference (e.g., hover throttle for throttle mode)
 *          - Tracking mode: Dynamic frequency tracking algorithm selection
 *          - Options flags: Composite notch configuration and feature enables
 *          - Frequency minimum ratio: Lower frequency bound for tracking modes
 *          
 *          These parameters are typically configured via ground control station and
 *          stored in non-volatile memory (EEPROM/Flash) for persistence across reboots.
 * 
 * @note Inherits center frequency, bandwidth, and attenuation from NotchFilterParams
 * @see HarmonicNotchFilter for usage of these parameters
 * @see HarmonicNotchDynamicMode for tracking mode details
 * 
 * Source: libraries/Filter/HarmonicNotchFilter.h:108-180
 */
class HarmonicNotchFilterParams : public NotchFilterParams {
public:
    /**
     * @enum Options
     * @brief Configuration option flags for harmonic notch filter behavior
     * 
     * @details Bitmask flags that enable optional features and modify filter behavior.
     *          Multiple options can be combined using bitwise OR. These flags affect
     *          filter allocation, update rate, and composite notch configuration.
     * 
     * Source: libraries/Filter/HarmonicNotchFilter.h:110-117
     */
    enum class Options {
        /**
         * @brief Enable double notch per harmonic (2x filters per harmonic)
         * 
         * Creates two notch filters per harmonic, spread symmetrically around the
         * center frequency to widen the rejection band. Provides broader frequency
         * coverage at the cost of 2x computational load. Cannot be combined with TripleNotch.
         */
        DoubleNotch = 1<<0,
        
        /**
         * @brief Enable dynamic harmonic selection based on measured frequencies
         * 
         * Automatically enables/disables harmonics based on detected noise peaks rather
         * than using a fixed harmonic bitmask. Adapts to changing noise characteristics
         * during flight. Requires gyro FFT or other frequency detection method.
         */
        DynamicHarmonic = 1<<1,
        
        /**
         * @brief Update filter frequencies at loop rate instead of slower rate
         * 
         * Increases filter update rate to match main control loop (typically 400Hz)
         * instead of the default lower rate (typically 50Hz). Provides faster tracking
         * of rapid frequency changes at the cost of increased CPU usage.
         */
        LoopRateUpdate = 1<<2,
        
        /**
         * @brief Apply filter to all IMU instances, not just primary
         * 
         * Enables harmonic notch filtering on all available IMUs (up to 3) rather than
         * only the primary IMU. Ensures consistent filtering across redundant sensors
         * for sensor fusion and failover. Multiplies filter count by number of IMUs.
         */
        EnableOnAllIMUs = 1<<3,
        
        /**
         * @brief Enable triple notch per harmonic (3x filters per harmonic)
         * 
         * Creates three notch filters per harmonic with wider spreading than double notch.
         * Provides maximum frequency coverage for wide-band noise at the cost of 3x
         * computational load. Cannot be combined with DoubleNotch.
         */
        TripleNotch = 1<<4,
        
        /**
         * @brief Treat frequencies below minimum as minimum frequency (clamp instead of disable)
         * 
         * When tracked frequency falls below the minimum threshold (FREQ * FM_RAT),
         * clamp the notch center to the minimum frequency instead of disabling the
         * filter. Maintains filtering during low-throttle/low-RPM conditions.
         */
        TreatLowAsMin = 1<<5,
    };

    /**
     * @brief Constructor - initializes parameter defaults
     */
    HarmonicNotchFilterParams(void);

    /**
     * @brief Initialize parameters from persistent storage
     * 
     * @details Loads parameter values from EEPROM/Flash and performs validation.
     *          Called during system initialization to restore saved configuration.
     */
    void init();

    /**
     * @brief Set the fundamental center frequency of the harmonic notch
     * 
     * @param[in] center_freq Center frequency in Hertz (Hz)
     * 
     * @note For dynamic tracking modes, this represents the reference frequency
     *       at which the reference value (e.g., hover throttle) was calibrated.
     */
    void set_center_freq_hz(float center_freq) { _center_freq_hz.set(center_freq); }

    /**
     * @brief Set the bandwidth of the harmonic notch
     * 
     * @param[in] bandwidth_hz Notch bandwidth in Hertz (Hz)
     * 
     * @note Bandwidth determines the width of the rejection band around each
     *       notch center frequency. Typical values: 10-50 Hz.
     */
    void set_bandwidth_hz(float bandwidth_hz) { _bandwidth_hz.set(bandwidth_hz); }

    /**
     * @brief Set the attenuation of the harmonic notch
     * 
     * @param[in] attenuation_dB Attenuation depth in decibels (dB)
     * 
     * @note Higher attenuation provides deeper rejection but narrower bandwidth
     *       and potential filter stability issues. Typical values: 20-40 dB.
     */
    void set_attenuation(float attenuation_dB) { _attenuation_dB.set(attenuation_dB); }
    
    /**
     * @brief Get the enabled harmonics bitmask
     * 
     * @return Bitmask where bit N enables harmonic (N+1)
     * 
     * @note Example: 0x05 (binary 0101) enables 1st and 3rd harmonics
     */
    uint32_t harmonics(void) const { return _harmonics; }

    /**
     * @brief Set the harmonics bitmask
     * 
     * @param[in] hmncs Bitmask of enabled harmonics
     */
    void set_harmonics(uint32_t hmncs) { _harmonics.set(hmncs); }

    /**
     * @brief Set the default harmonics value (used if user hasn't configured)
     * 
     * @param[in] hmncs Default harmonics bitmask
     */
    void set_default_harmonics(uint32_t hmncs) { _harmonics.set_default(hmncs); }

    /**
     * @brief Get the reference value for dynamic tracking
     * 
     * @return Reference value (interpretation depends on tracking mode)
     * 
     * @note For throttle mode: hover throttle (0.0-1.0)
     * @note For RPM mode: RPM at which center frequency was measured
     */
    float reference(void) const { return _reference; }
    
    /**
     * @brief Set the reference value for dynamic tracking
     * 
     * @param[in] ref Reference value (units depend on tracking mode)
     */
    void set_reference(float ref) { _reference.set(ref); }

    /**
     * @brief Check if a specific option flag is enabled
     * 
     * @param[in] option Option flag to check
     * @return true if option is enabled, false otherwise
     */
    bool hasOption(Options option) const { return _options & uint16_t(option); }
    
    /**
     * @brief Get the current dynamic tracking mode
     * 
     * @return Current HarmonicNotchDynamicMode
     * 
     * @see HarmonicNotchDynamicMode for mode descriptions
     */
    HarmonicNotchDynamicMode tracking_mode(void) const { return HarmonicNotchDynamicMode(_tracking_mode.get()); }
    
    /**
     * @brief AP_Param metadata for ground station parameter management
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get the minimum frequency ratio for throttle-based tracking
     * 
     * @return Minimum frequency ratio (typically 0.5-1.0)
     * 
     * @details Defines lower frequency bound as: min_freq = center_freq * freq_min_ratio.
     *          Prevents tracking unrealistically low frequencies during low throttle.
     */
    float freq_min_ratio(void) const {
        return _freq_min_ratio;
    }
    
    /**
     * @brief Set the minimum frequency ratio
     * 
     * @param[in] ratio Minimum frequency ratio
     */
    void set_freq_min_ratio(float ratio) { _freq_min_ratio.set(ratio); }

    /**
     * @brief Set options flags
     * 
     * @param[in] options Bitmask of Options flags
     */
    void set_options(uint16_t options) { _options.set(options); }

    /**
     * @brief Save current parameter values to persistent storage
     * 
     * @details Writes all modified parameters to EEPROM/Flash for persistence
     *          across reboots. Called after runtime parameter adjustments.
     */
    void save_params();

    /**
     * @brief Calculate number of composite notches based on option flags
     * 
     * @return 1 for single notch, 2 for double notch, 3 for triple notch
     * 
     * @note DoubleNotch and TripleNotch options are mutually exclusive.
     *       If both set, DoubleNotch takes precedence.
     */
    uint8_t num_composite_notches(void) const {
        return hasOption(Options::DoubleNotch) ? 2 : hasOption(Options::TripleNotch) ? 3: 1;
    }

private:
    // configured notch harmonics
    AP_Int32 _harmonics;
    // notch reference value
    AP_Float _reference;
    // notch dynamic tracking mode
    AP_Int8 _tracking_mode;
    // notch options
    AP_Int16 _options;

    // minimum frequency ratio for throttle based notches
    AP_Float _freq_min_ratio;
};

typedef HarmonicNotchFilter<Vector3f> HarmonicNotchFilterVector3f;

