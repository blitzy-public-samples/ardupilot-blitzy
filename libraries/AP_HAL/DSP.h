/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper
 */
/**
 * @file DSP.h
 * @brief Digital Signal Processing interface for FFT and frequency analysis
 * 
 * @details Provides Fast Fourier Transform (FFT) capabilities for analyzing gyroscope noise,
 *          detecting vibrations, and implementing dynamic notch filters. Platform implementations
 *          may use hardware accelerators (DSP cores, SIMD instructions) or optimized software FFT.
 * 
 *          Primary use case: Dynamic notch filtering for multicopter vibration rejection.
 *          FFT analysis identifies dominant frequency peaks in gyroscope data, enabling
 *          real-time notch filter placement to reject motor and propeller vibrations.
 * 
 * @author Andy Piper
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_HAL/DSP.h
 */
#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"
#include <AP_HAL/utility/RingBuffer.h>

/** @brief Memory region for DSP buffers - fast RAM close to CPU for optimal FFT performance */
#define DSP_MEM_REGION AP_HAL::Util::MEM_FAST
/** @brief Maximum tolerated number of cycles with missing signal before FFT analysis is reset */
#define FFT_MAX_MISSED_UPDATES 5

/**
 * @class AP_HAL::DSP
 * @brief Abstract interface for digital signal processing operations
 * 
 * @details Provides FFT analysis for time-domain sensor data, particularly gyroscope
 *          samples for vibration detection and dynamic notch filter placement. 
 *          Implementations may leverage:
 *          - Hardware DSP cores (ARM Cortex-M4/M7 FPU, Qualcomm Hexagon DSP)
 *          - SIMD instructions (ARM NEON, x86 SSE)
 *          - Optimized software FFT libraries (CMSIS-DSP, FFTW)
 *          
 *          Typical FFT workflow:
 *          1. fft_init() - Allocate FFTWindowState with specified window size and sample rate
 *          2. fft_start() - Begin analysis on new time-domain samples from sensor
 *          3. fft_analyse() - Execute FFT transform and identify frequency peaks
 *          4. Access peak frequency data from FFTWindowState::_peak_data
 *          5. Repeat steps 2-4 for continuous real-time analysis
 *          
 *          Memory allocation uses DSP_MEM_REGION (typically fast RAM near CPU core)
 *          for optimal performance during real-time FFT execution in scheduler callbacks.
 *          
 *          Frequency resolution determined by: resolution = sample_rate / window_size
 *          Example: 1000 Hz sample rate with 256 samples = 3.9 Hz resolution per bin
 * 
 * @note Controlled by HAL_WITH_DSP compile-time feature flag
 * @note FFT operations have strict timing constraints - must complete within scheduler slice (typically <1ms)
 * @warning Large window sizes may exceed memory or timing budgets on resource-constrained flight controllers
 * 
 * Source: libraries/AP_HAL/DSP.h
 */
class AP_HAL::DSP {
#if HAL_WITH_DSP
public:
    /**
     * @enum FrequencyPeak
     * @brief Identifies tracked frequency peaks in FFT analysis
     * 
     * @details FFT analysis tracks up to three significant frequency peaks:
     *          the primary center peak and its lower/upper shoulder peaks.
     *          This enables detection of harmonic vibrations and multi-modal
     *          frequency content (e.g., multiple motor harmonics).
     */
    enum FrequencyPeak : uint8_t {
        CENTER = 0,          ///< Primary frequency peak with highest energy
        LOWER_SHOULDER = 1,  ///< Secondary peak below center frequency
        UPPER_SHOULDER = 2,  ///< Secondary peak above center frequency
        MAX_TRACKED_PEAKS = 3, ///< Total number of tracked peaks (array size)
        NONE = 4             ///< Indicator for no valid peak detected
    };

    /**
     * @struct FrequencyPeakData
     * @brief Detected frequency peak information from FFT analysis
     * 
     * @details Contains interpolated peak frequency, bin location, and noise bandwidth.
     *          Frequency interpolation uses Quinn's second estimator or Jain's estimator
     *          for sub-bin accuracy, improving frequency estimation beyond bin resolution.
     *          Noise width indicates the bandwidth of the peak, useful for filter design.
     */
    struct FrequencyPeakData {
        float _freq_hz;          ///< Interpolated peak frequency in Hz (sub-bin resolution via Quinn's or Jain's estimator)
        uint16_t _bin;           ///< FFT bin index with maximum energy for this peak
        float _noise_width_hz;   ///< Bandwidth of the peak in Hz (distance between -3dB points or threshold crossings)
    };

    /** @brief Maximum size of sliding window for averaged FFT analysis (Welch's method) */
    static const uint8_t MAX_SLIDING_WINDOW_SIZE = 8;

    /**
     * @class FFTWindowState
     * @brief State and buffers for a single FFT analysis instance
     * 
     * @details Maintains all memory buffers and intermediate state for FFT computation:
     *          - Input samples with Hanning window applied for spectral leakage reduction
     *          - FFT output bins in frequency domain (complex data converted to magnitudes)
     *          - Averaged results using Welch's method (overlapping windowed FFT averaging)
     *          - Sliding window buffer for continuous analysis with reduced CPU overhead
     *          - Identified frequency peaks with interpolated frequencies and bandwidths
     *          
     *          Window size determines frequency resolution: resolution = sample_rate / window_size.
     *          Typical configurations:
     *          - 128 samples at 1kHz = 7.8 Hz resolution (fast, low resolution)
     *          - 256 samples at 1kHz = 3.9 Hz resolution (balanced)
     *          - 512 samples at 2kHz = 3.9 Hz resolution (high resolution, more CPU)
     *          
     *          Sliding window enables continuous averaging without full window overlap,
     *          reducing CPU load while maintaining good frequency resolution. Multiple
     *          overlapping FFT windows are computed and averaged (Welch's method) to
     *          reduce noise and improve peak detection reliability.
     *          
     *          Lifecycle:
     *          1. Construction allocates all buffers in DSP_MEM_REGION
     *          2. fft_start() populates input buffers with windowed samples
     *          3. fft_analyse() executes FFT and identifies peaks
     *          4. Destruction or free_data_structures() releases all memory
     * 
     * @note All buffers allocated in DSP_MEM_REGION (fast RAM) for real-time performance
     * @note Memory usage approximately: window_size * (sliding_window_size + 3) * sizeof(float) bytes
     * @warning Destructor frees all buffers - do not access FFTWindowState after destruction
     * 
     * Source: libraries/AP_HAL/DSP.h:52-92
     */
    class FFTWindowState {
    public:
        const float _bin_resolution;      ///< Frequency width of a single FFT bin in Hz (sample_rate / window_size)
        const uint16_t _bin_count;        ///< Number of FFT output bins (window_size / 2 for real FFT)
        const uint16_t _num_stored_freqs; ///< Total stored frequency values (_bin_count + 1 for DC component)
        const uint16_t _window_size;      ///< Size of the FFT window in samples (must be power of 2: 128, 256, 512)
        const uint8_t _sliding_window_size; ///< Number of overlapping windows to average (0 for single window, max 8)
        
        float* _freq_bins;                ///< Frequency domain magnitudes after FFT (length: _num_stored_freqs)
        float* _derivative_freq_bins;     ///< Derivative scratch space for peak finding algorithm (length: _num_stored_freqs)
        float* _rfft_data;                ///< Intermediate real FFT data buffer for platform-specific FFT implementation (length varies by implementation)
        float* _avg_freq_bins;            ///< Averaged frequency magnitudes via Welch's method (length: _num_stored_freqs)
        float* _sliding_window;           ///< Circular buffer of frequency data for sliding window averaging (length: _bin_count * _sliding_window_size)
        
        FrequencyPeakData _peak_data[MAX_TRACKED_PEAKS]; ///< Detected frequency peaks: center, lower shoulder, upper shoulder
        
        float* _hanning_window;           ///< Hanning window coefficients applied to time-domain input (length: _window_size) - see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
        float _window_scale;              ///< Scaling factor for power spectral density calculation (Heinz equations 20 & 21) - compensates for window energy loss
        
        bool _averaging;                  ///< True when Welch's method averaging is active (accumulating multiple FFT results)
        uint32_t _averaging_samples;      ///< Number of FFT windows accumulated in current average (for normalization)
        uint8_t _current_slice;           ///< Current position in sliding window circular buffer (0 to _sliding_window_size-1)
        /**
         * @brief Get frequency magnitude from appropriate buffer (current or averaged)
         * 
         * @param[in] idx FFT bin index (0 to _num_stored_freqs-1)
         * @return float Magnitude at specified bin (non-averaged or averaged depending on mode)
         * 
         * @details Returns _freq_bins[idx] for single-window mode, or _avg_freq_bins[idx]
         *          when sliding window averaging is enabled. Enables uniform access pattern.
         */
        float get_freq_bin(uint16_t idx) { return _sliding_window == nullptr ? _freq_bins[idx] : _avg_freq_bins[idx]; }

        /**
         * @brief Free all allocated FFT buffers
         * 
         * @details Releases all dynamically allocated memory for FFT processing buffers.
         *          Called by destructor or can be called explicitly for early cleanup.
         *          Safe to call multiple times (nulls pointers after freeing).
         */
        void free_data_structures();
        
        /**
         * @brief Destructor - frees all allocated FFT buffers
         * 
         * @warning After destruction, do not access any FFTWindowState members or buffers
         */
        virtual ~FFTWindowState();
        
        /**
         * @brief Construct FFT window state with specified parameters
         * 
         * @param[in] window_size Number of time-domain samples per FFT (power of 2: 128, 256, 512)
         * @param[in] sample_rate Sampling rate in Hz (must match actual sensor data rate)
         * @param[in] sliding_window_size Number of overlapping windows to average (0 for single window, max 8)
         * 
         * @note Allocates approximately window_size * (sliding_window_size + 3) * sizeof(float) bytes
         * @warning Construction may fail silently if memory allocation fails - check pointers before use
         */
        FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size);
    };
    
    /**
     * @brief Initialize FFT analysis instance with specified parameters
     * 
     * @param[in] window_size Number of time-domain samples per FFT (power of 2: 128, 256, 512)
     * @param[in] sample_rate Sampling rate in Hz (must match actual sensor data rate)
     * @param[in] sliding_window_size Number of overlapping windows to average using Welch's method (0 for single window, max 8)
     * 
     * @return FFTWindowState* Pointer to initialized state, or nullptr on allocation failure
     * 
     * @details Allocates FFTWindowState and all associated buffers in DSP_MEM_REGION (fast RAM).
     *          Memory requirement approximately: ceil(window_size * (sliding_window_size + 3)) * sizeof(float) bytes.
     *          
     *          Window size selection trade-offs:
     *          - Larger windows: Better frequency resolution, more CPU time, more memory
     *          - Smaller windows: Faster execution, coarser resolution, less memory
     *          
     *          Sliding window averaging (Welch's method) reduces noise and improves peak
     *          detection reliability at the cost of increased memory and processing time.
     *          Typical sliding_window_size values: 0 (no averaging), 4 (moderate), 8 (maximum).
     * 
     * @note Platform implementations may use hardware FFT accelerators or optimized libraries (CMSIS-DSP, FFTW)
     * @note Larger windows provide better frequency resolution but require more memory and CPU time
     * @warning Caller must call destructor or free_data_structures() to release memory when done
     * @warning Returns nullptr if memory allocation fails - always check return value before use
     * 
     * Source: libraries/AP_HAL/DSP.h:94
     */
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size = 0) = 0;
    
    /**
     * @brief Start FFT analysis on new time-domain samples
     * 
     * @param[in,out] state FFT window state containing buffers and configuration
     * @param[in] samples Ring buffer containing time-domain input samples (typically gyro data)
     * @param[in] advance Number of samples to advance in the ring buffer for next analysis (enables overlapping windows)
     * 
     * @details Copies samples from ring buffer into FFT input buffer, applies Hanning window
     *          to reduce spectral leakage, and prepares for FFT execution. The advance parameter
     *          controls window overlap:
     *          - advance = window_size: No overlap (consecutive windows)
     *          - advance = window_size/2: 50% overlap (Welch's method standard)
     *          - advance < window_size: Increased overlap, more CPU but smoother results
     *          
     *          Hanning window is applied to input samples to minimize spectral leakage
     *          from discontinuities at window boundaries. This improves frequency resolution
     *          and reduces sidelobe energy in the FFT output.
     *          
     *          After fft_start(), call fft_analyse() to execute the FFT transform and
     *          identify frequency peaks.
     * 
     * @note Typically called at regular intervals (e.g., every 25ms for 40Hz update rate)
     * @note samples buffer must contain at least window_size valid samples
     * @warning advance must be > 0 and <= window_size to avoid buffer overruns
     * 
     * Source: libraries/AP_HAL/DSP.h:96
     */
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) = 0;
    
    /**
     * @brief Perform FFT transform and identify frequency peaks
     * 
     * @param[in,out] state FFT window state with input samples from fft_start()
     * @param[in] start_bin First FFT bin to analyze (typically 2-5 to skip DC and very low frequencies)
     * @param[in] end_bin Last FFT bin to analyze (typically limited by Nyquist frequency or filter range)
     * @param[in] noise_att_cutoff Noise attenuation threshold in dB for peak width detection (typically 5-10 dB)
     * 
     * @return uint16_t Number of valid frequency peaks detected (0 to MAX_TRACKED_PEAKS)
     * 
     * @details Executes the complete FFT analysis pipeline:
     *          1. Platform-specific FFT transform (hardware or software implementation)
     *          2. Convert complex FFT output to frequency magnitude spectrum
     *          3. Search specified bin range [start_bin, end_bin] for energy peaks
     *          4. Interpolate peak frequencies using Quinn's or Jain's estimator for sub-bin accuracy
     *          5. Calculate peak bandwidths based on noise_att_cutoff threshold
     *          6. Store up to MAX_TRACKED_PEAKS peaks in state->_peak_data[]
     *          
     *          Bin range limits allow focusing analysis on relevant frequencies:
     *          - start_bin: Skip DC component and very low frequencies (< 10 Hz)
     *          - end_bin: Limit to filter operating range or Nyquist frequency
     *          
     *          noise_att_cutoff defines peak bandwidth measurement: distance between
     *          points where magnitude drops by this threshold (in dB) from peak value.
     *          Typical values: 5-10 dB for notch filter design.
     *          
     *          Detected peaks stored in decreasing energy order:
     *          state->_peak_data[CENTER] = highest energy peak
     *          state->_peak_data[LOWER_SHOULDER] = second highest below center
     *          state->_peak_data[UPPER_SHOULDER] = second highest above center
     * 
     * @note Must call fft_start() before fft_analyse() to populate input buffers
     * @note start_bin must be >= 1 (bin 0 is DC component), typically >= 2
     * @note end_bin must be <= state->_bin_count and > start_bin
     * @note Higher noise_att_cutoff values result in narrower reported peak widths
     * @warning FFT execution time scales with window_size - ensure completion within scheduler budget
     * @warning Invalid bin range may cause buffer overruns or incorrect results
     * 
     * Source: libraries/AP_HAL/DSP.h:98
     */
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) = 0;
    
    /**
     * @brief Start Welch's method averaging of FFT data
     * 
     * @param[in,out] fft FFT window state to begin averaging on
     * @return bool True if averaging started successfully, false if already averaging or nullptr
     * 
     * @details Initializes the averaging process for Welch's method, which computes multiple
     *          overlapping FFTs and averages the results to reduce noise and improve peak
     *          detection reliability. The sliding window buffer accumulates FFT results
     *          across multiple fft_analyse() calls.
     *          
     *          Typical usage pattern:
     *          1. fft_start_average() - initialize averaging
     *          2. Loop: fft_start() + fft_analyse() for each window
     *          3. fft_stop_average() - finalize and retrieve averaged peaks
     *          
     *          Averaging reduces noise variance by sqrt(N) where N is the number of
     *          averaged windows, improving signal-to-noise ratio and peak stability.
     * 
     * @note Only valid if fft was initialized with sliding_window_size > 0
     * @note Returns false if fft is nullptr or averaging already in progress
     * @see fft_stop_average()
     * 
     * Source: libraries/AP_HAL/DSP.h:100
     */
    bool fft_start_average(FFTWindowState* fft);
    
    /**
     * @brief Finalize Welch's method averaging and extract peaks
     * 
     * @param[in,out] fft FFT window state with accumulated averaging data
     * @param[in] start_bin First FFT bin to search for peaks (typically 2-5)
     * @param[in] end_bin Last FFT bin to search for peaks (limited by Nyquist or filter range)
     * @param[out] peaks Array to receive peak frequencies (length: MAX_TRACKED_PEAKS), in Hz
     * 
     * @return uint16_t Number of valid peaks found and written to peaks array (0 to MAX_TRACKED_PEAKS)
     * 
     * @details Completes the Welch's method averaging process by:
     *          1. Computing final averaged frequency spectrum from sliding window buffer
     *          2. Normalizing by number of accumulated FFT windows
     *          3. Searching for frequency peaks within [start_bin, end_bin] range
     *          4. Extracting peak frequencies with interpolation
     *          5. Writing detected peak frequencies to peaks array in decreasing energy order
     *          
     *          The averaged spectrum has improved signal-to-noise ratio compared to
     *          single FFT analysis, resulting in more stable and reliable peak detection.
     *          
     *          Must be called after fft_start_average() and one or more fft_analyse() calls.
     *          Resets averaging state, allowing a new averaging cycle to begin.
     * 
     * @note peaks array must have space for at least MAX_TRACKED_PEAKS floats
     * @note Returns 0 if fft is nullptr or no peaks detected in specified range
     * @see fft_start_average()
     * 
     * Source: libraries/AP_HAL/DSP.h:102
     */
    uint16_t fft_stop_average(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float* peaks);

protected:
    /**
     * @brief Convert complex FFT output to magnitude spectrum
     * 
     * @param[in,out] fft FFT window state with complex FFT results
     * @param[in] start_bin First bin to process
     * @param[in] end_bin Last bin to process  
     * @param[in] noise_att_cutoff Noise attenuation cutoff in dB for peak detection
     * 
     * @details Computes magnitude spectrum from complex FFT coefficients and applies
     *          noise floor estimation for subsequent peak detection.
     */
    void step_cmplx_mag(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff);
    
    /**
     * @brief Calculate bandwidth of a frequency peak
     * 
     * @param[in] freq_bins Frequency magnitude spectrum
     * @param[in] start_bin Search range start
     * @param[in] end_bin Search range end
     * @param[in] max_energy_bin Peak bin location
     * @param[in] cutoff Threshold in dB below peak for width measurement
     * @param[in] bin_resolution Frequency resolution per bin (Hz)
     * @param[out] peak_top Upper bin where magnitude drops below cutoff
     * @param[out] peak_bottom Lower bin where magnitude drops below cutoff
     * @return float Bandwidth in Hz between peak_bottom and peak_top
     * 
     * @details Measures peak width by finding bins where magnitude drops by cutoff dB
     *          from peak value. Used for notch filter bandwidth estimation.
     */
    float find_noise_width(float* freq_bins, uint16_t start_bin, uint16_t end_bin, uint16_t max_energy_bin, float cutoff,
        float bin_resolution, uint16_t& peak_top, uint16_t& peak_bottom) const;
    
    /**
     * @brief Identify frequency peaks and interpolate sub-bin frequencies
     * 
     * @param[in,out] fft FFT window state with magnitude spectrum
     * @param[in] start_bin First bin to search
     * @param[in] end_bin Last bin to search
     * @return uint16_t Number of peaks detected
     * 
     * @details Finds up to MAX_TRACKED_PEAKS peaks using algorithm from
     *          https://terpconnect.umd.edu/~toh/spectrum/PeakFindingandMeasurement.htm
     *          and interpolates frequencies using Quinn's or Jain's estimator.
     */
    uint16_t step_calc_frequencies(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin);
    
    /**
     * @brief Compute averaged spectrum from sliding window buffer
     * 
     * @param[in,out] fft FFT window state with accumulated sliding window data
     * 
     * @details Averages frequency data across sliding window slices (Welch's method)
     *          and stores result in _avg_freq_bins for peak detection.
     */
    void update_average_from_sliding_window(FFTWindowState* fft);
    
    /**
     * @brief Calculate interpolated frequency for a single peak
     * 
     * @param[in] fft FFT window state
     * @param[in] start_bin Search range start
     * @param[in] peak_bin Bin with maximum energy
     * @param[in] end_bin Search range end
     * @return uint16_t 1 if successful, 0 on failure
     * 
     * @details Uses Quinn's or Jain's estimator for sub-bin frequency interpolation,
     *          improving accuracy beyond bin resolution.
     */
    uint16_t calc_frequency(FFTWindowState* fft, uint16_t start_bin, uint16_t peak_bin, uint16_t end_bin);
    
    /**
     * @brief Find maximum value and its index in float array
     * 
     * @param[in] vin Input vector
     * @param[in] len Vector length
     * @param[out] max_value Maximum value found
     * @param[out] max_index Index of maximum value
     * 
     * @note Platform implementations may use SIMD instructions for acceleration
     */
    virtual void vector_max_float(const float* vin, uint16_t len, float* max_value, uint16_t* max_index) const = 0;
    
    /**
     * @brief Calculate mean of float array
     * 
     * @param[in] vin Input vector
     * @param[in] len Vector length
     * @return float Mean value
     * 
     * @note Platform implementations may use SIMD instructions for acceleration
     */
    virtual float vector_mean_float(const float* vin, uint16_t len) const = 0;
    
    /**
     * @brief Scale float vector by constant
     * 
     * @param[in] vin Input vector
     * @param[in] scale Scale factor
     * @param[out] vout Output vector (may alias vin)
     * @param[in] len Vector length
     * 
     * @note Platform implementations may use SIMD instructions for acceleration
     */
    virtual void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const = 0;
    
    /**
     * @brief Add two float vectors element-wise
     * 
     * @param[in] vin1 First input vector
     * @param[in] vin2 Second input vector
     * @param[out] vout Output vector (may alias inputs)
     * @param[in] len Vector length
     * 
     * @note Platform implementations may use SIMD instructions for acceleration
     */
    virtual void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const = 0;
    
    /**
     * @brief Peak finding algorithm for noisy spectral data
     * 
     * @param[in] input Input magnitude spectrum
     * @param[in] length Spectrum length
     * @param[out] output Smoothed derivative for peak detection
     * @param[out] peaks Indices of detected peaks
     * @param[in] peaklen Maximum number of peaks to find
     * @param[in] slopeThreshold Minimum slope for peak detection
     * @param[in] ampThreshold Minimum amplitude for peak detection
     * @param[in] smoothwidth Smoothing window width
     * @param[in] peakgroup Minimum separation between peaks
     * @return uint16_t Number of peaks found
     * 
     * @details Implements algorithm from https://terpconnect.umd.edu/~toh/spectrum/PeakFindingandMeasurement.htm
     *          using derivative, smoothing, and threshold-based peak detection.
     */
    uint16_t find_peaks(const float* input, uint16_t length, float* output, uint16_t* peaks, uint16_t peaklen, 
        float slopeThreshold, float ampThreshold, uint16_t smoothwidth, uint16_t peakgroup) const;
    
    /** @brief Find index of value in sorted vector (helper for peak finding) */
    uint16_t val2index(const float* vector, uint16_t n, float val) const;
    
    /** @brief Compute discrete derivative of input signal (helper for peak finding) */
    void derivative(const float* input, float* output, uint16_t n) const;
    
    /** @brief Fast smoothing filter using sliding average (helper for peak finding) */
    void fastsmooth(float* input, uint16_t n, uint16_t smoothwidth) const;

    /**
     * @brief Quinn's second estimator for sub-bin frequency interpolation
     * 
     * @param[in] fft FFT window state
     * @param[in] complex_fft Complex FFT coefficients
     * @param[in] k Peak bin index
     * @return float Fractional bin offset (-0.5 to +0.5) for interpolated frequency
     * 
     * @details Provides accurate frequency estimation between FFT bins using phase
     *          relationships of adjacent bins. Generally more accurate than Jain's
     *          estimator but requires complex FFT data.
     */
    float calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k) const;
    
    /** @brief Helper function for Quinn's estimator calculation */
    float tau(const float x) const;
    
    /**
     * @brief Jain's estimator for sub-bin frequency interpolation
     * 
     * @param[in] fft FFT window state
     * @param[in] real_fft Real FFT magnitudes
     * @param[in] k_max Peak bin index
     * @return float Fractional bin offset for interpolated frequency
     * 
     * @details Simpler alternative to Quinn's estimator using only magnitude data.
     *          Useful when complex FFT coefficients are not readily available.
     */
    float calculate_jains_estimator(const FFTWindowState* fft, const float* real_fft, uint16_t k_max);
    
    /**
     * @brief Initialize averaging buffers (internal)
     * 
     * @param[in,out] fft FFT window state
     * @return bool True if initialization successful
     * 
     * @details Sets up sliding window buffers for Welch's method averaging.
     *          Called internally by fft_start_average().
     */
    bool fft_init_average(FFTWindowState* fft);

#endif // HAL_WITH_DSP
};
