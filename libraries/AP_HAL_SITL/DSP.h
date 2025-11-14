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
 * @brief Digital Signal Processing (DSP) implementation for SITL simulation
 * 
 * @details This file provides DSP functionality for Software-In-The-Loop (SITL)
 *          simulation, specifically implementing Fast Fourier Transform (FFT)
 *          analysis used for vibration detection and notch filter tuning.
 *          
 *          The SITL DSP implementation uses standard C++ complex number math
 *          to simulate the FFT processing that would occur on hardware platforms.
 *          This allows developers to test gyro FFT analysis and dynamic notch
 *          filtering in simulation before deploying to hardware.
 *          
 *          Key features:
 *          - Cooley-Tukey in-place FFT algorithm implementation
 *          - Hanning window application for frequency analysis
 *          - Integration with AP_GyroFFT for vibration analysis
 *          - Support for various FFT window sizes (typically 32-1024 samples)
 *          - Configurable sample rates and frequency resolution
 *          
 *          Typical use case: Analyzing gyroscope data to detect vibration
 *          frequencies and automatically tune notch filters to reduce noise
 *          in attitude control loops.
 * 
 * @note This implementation is for SITL simulation only. Hardware platforms
 *       (STM32, etc.) use optimized DSP libraries in AP_HAL_ChibiOS.
 * 
 * @warning Computational cost in SITL is not representative of real hardware.
 *          Real-time constraints may not apply in simulation environment.
 * 
 * @see AP_GyroFFT for gyroscope FFT analysis integration
 * @see AP_HAL::DSP for the abstract DSP interface
 * 
 * Requires HAL_WITH_DSP compile-time flag to be enabled.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_DSP

#include "AP_HAL_SITL.h"

#include <complex>

/**
 * @typedef complexf
 * @brief Complex float type for FFT computations
 * 
 * @details Alias for std::complex<float>, used throughout FFT processing
 *          to represent complex numbers in frequency domain analysis.
 *          Each complex number has real and imaginary components used in
 *          the Fourier transform calculations.
 */
typedef std::complex<float> complexf;

/**
 * @class HALSITL::DSP
 * @brief SITL implementation of Digital Signal Processing for FFT analysis
 * 
 * @details This class provides Fast Fourier Transform (FFT) analysis capabilities
 *          for the Software-In-The-Loop (SITL) simulation environment. It implements
 *          the AP_HAL::DSP abstract interface using standard C++ complex number
 *          operations and the Cooley-Tukey FFT algorithm.
 *          
 *          The implementation simulates hardware DSP functionality that would
 *          normally be provided by optimized DSP libraries on ARM Cortex-M processors
 *          (such as CMSIS-DSP on STM32 platforms). This allows gyro FFT analysis
 *          and dynamic notch filter tuning to be tested in simulation.
 *          
 *          FFT Processing Pipeline:
 *          1. Initialize FFT state with window size and sample rate
 *          2. Apply Hanning window to input samples (reduces spectral leakage)
 *          3. Perform in-place Cooley-Tukey FFT
 *          4. Analyze frequency bins to find peak frequencies
 *          5. Output frequency and amplitude information
 *          
 *          Typical usage for vibration analysis:
 *          - Window sizes: 32, 64, 128, 256, 512, or 1024 samples
 *          - Sample rates: Match gyro update rate (typically 1000-8000 Hz)
 *          - Frequency resolution: sample_rate / window_size Hz per bin
 *          - Analysis range: DC to Nyquist frequency (sample_rate / 2)
 * 
 * @note This is a simulation-only implementation. Computational overhead
 *       does not reflect real hardware performance characteristics.
 * 
 * @warning Real-time timing constraints in simulation may differ from hardware.
 *          Use for algorithm validation, not performance benchmarking.
 * 
 * @see AP_HAL::DSP for the abstract interface definition
 * @see AP_GyroFFT for the primary consumer of FFT analysis
 * @see FFTWindowStateSITL for the FFT state container
 */
class HALSITL::DSP : public AP_HAL::DSP {
public:
    /**
     * @brief Initialize an FFT instance with specified parameters
     * 
     * @details Allocates and configures an FFT window state object for frequency
     *          analysis. The window size determines frequency resolution and
     *          computational cost. Must be a power of 2 for Cooley-Tukey algorithm.
     *          
     *          Frequency resolution = sample_rate / window_size Hz per bin
     *          Maximum frequency (Nyquist) = sample_rate / 2 Hz
     *          
     *          Example: window_size=256, sample_rate=1000Hz
     *          - Frequency resolution: 3.9 Hz per bin
     *          - Nyquist frequency: 500 Hz
     *          - Analysis bins: 0-127 (DC to 500Hz)
     *          
     *          The sliding window mechanism allows overlapping FFT windows for
     *          better temporal resolution of frequency changes.
     * 
     * @param[in] window_size Number of samples per FFT window (must be power of 2)
     *                        Typical values: 32, 64, 128, 256, 512, 1024
     * @param[in] sample_rate Sample rate in Hz (must match input data rate)
     *                        Typical values: 1000-8000 Hz for gyro data
     * @param[in] sliding_window_size Number of overlapping windows to maintain
     *                                 for improved frequency tracking
     * 
     * @return Pointer to initialized FFTWindowState object, or nullptr on failure
     * 
     * @note Caller is responsible for managing the lifetime of the returned object
     * @warning window_size must be power of 2, otherwise FFT algorithm will fail
     * 
     * @see FFTWindowStateSITL for the state object structure
     * @see fft_start() for beginning analysis with the initialized state
     */
    virtual FFTWindowState* fft_init(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size) override;
    
    /**
     * @brief Start FFT analysis on a new batch of samples
     * 
     * @details Begins FFT processing by applying a Hanning window to the input
     *          samples and preparing them for frequency domain transformation.
     *          The Hanning window reduces spectral leakage by tapering sample
     *          values at the edges of the analysis window.
     *          
     *          This method performs the first stage of FFT processing:
     *          1. Reads samples from the FloatBuffer
     *          2. Applies Hanning window coefficients (cosine taper)
     *          3. Stores windowed samples in FFT state buffer
     *          4. Advances buffer position for sliding window operation
     *          
     *          After calling this method, call fft_analyse() to complete the
     *          FFT computation and extract frequency information.
     *          
     *          Input data format: Floating-point samples (typically gyro rates
     *          in rad/s or deg/s). Values are typically in range [-1000, 1000]
     *          for gyroscope data.
     * 
     * @param[in,out] state Pointer to FFT state object from fft_init()
     * @param[in]     samples Buffer containing input time-domain samples
     *                        Must contain at least window_size values
     * @param[in]     advance Number of samples to advance for next window
     *                        (enables overlapping windows when < window_size)
     * 
     * @note This method must be called before fft_analyse()
     * @warning Input samples buffer must contain at least window_size samples
     * 
     * @see fft_init() for state initialization
     * @see fft_analyse() for completing FFT and extracting frequencies
     * @see step_hanning() for Hanning window implementation details
     */
    virtual void fft_start(FFTWindowState* state, FloatBuffer& samples, uint16_t advance) override;
    
    /**
     * @brief Complete FFT analysis and identify peak frequencies
     * 
     * @details Performs the Fast Fourier Transform on windowed samples and
     *          analyzes the resulting frequency spectrum to find dominant
     *          frequencies within the specified bin range. Uses Cooley-Tukey
     *          in-place FFT algorithm for computational efficiency.
     *          
     *          Analysis process:
     *          1. Execute in-place FFT on complex buffer (time → frequency domain)
     *          2. Compute magnitude spectrum from complex FFT output
     *          3. Search specified frequency bins for peaks above noise threshold
     *          4. Identify bin with maximum magnitude
     *          5. Store peak frequency and amplitude in state object
     *          
     *          Frequency bins represent discrete frequency ranges:
     *          - bin 0: DC component (0 Hz)
     *          - bin N: frequency = N * (sample_rate / window_size) Hz
     *          - Maximum useful bin: window_size / 2 (Nyquist frequency)
     *          
     *          The noise_att_cutoff parameter filters out low-amplitude noise
     *          by requiring peaks to exceed a threshold relative to the mean
     *          noise floor.
     *          
     *          Output format: Peak frequency stored in state->_freq (Hz),
     *          peak amplitude stored in state->_amplitude (arbitrary units).
     * 
     * @param[in,out] state FFT state object with windowed samples from fft_start()
     * @param[in]     start_bin First frequency bin to analyze (inclusive)
     *                          Typical: 1 (skip DC component)
     * @param[in]     end_bin Last frequency bin to analyze (inclusive)
     *                        Typical: window_size/2 (Nyquist limit)
     *                        Must be ≤ window_size/2
     * @param[in]     noise_att_cutoff Noise attenuation cutoff in dB
     *                                  Peaks below this threshold are rejected
     *                                  Typical values: 5-15 dB
     * 
     * @return Index of frequency bin containing peak energy (0 if no peak found)
     * 
     * @note Must be called after fft_start() to process windowed samples
     * @warning end_bin must not exceed window_size/2 (Nyquist frequency)
     * @warning Computational cost scales as O(N log N) where N = window_size
     * 
     * @see fft_start() for sample windowing
     * @see calculate_fft() for Cooley-Tukey FFT implementation
     * @see step_fft() for magnitude spectrum computation
     */
    virtual uint16_t fft_analyse(FFTWindowState* state, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff) override;

    /**
     * @class FFTWindowStateSITL
     * @brief SITL-specific FFT state container for frequency analysis
     * 
     * @details Maintains state information for FFT analysis including sample
     *          buffers, window parameters, and intermediate computation results.
     *          This class extends the generic FFTWindowState interface with
     *          SITL-specific implementation details.
     *          
     *          The state object stores:
     *          - Complex number buffer for FFT input/output (in-place transform)
     *          - Window size, sample rate, and sliding window configuration
     *          - Hanning window coefficients (inherited from base class)
     *          - Analysis results: peak frequency and amplitude
     *          
     *          Memory layout: Complex buffer contains window_size complex floats,
     *          requiring 2 * window_size * sizeof(float) bytes (8 bytes per sample).
     *          
     *          Example memory usage:
     *          - 128-sample window: 1,024 bytes
     *          - 256-sample window: 2,048 bytes
     *          - 512-sample window: 4,096 bytes
     * 
     * @note This object should be created via DSP::fft_init(), not directly
     * @see AP_HAL::DSP::FFTWindowState for base class interface
     */
    class FFTWindowStateSITL : public AP_HAL::DSP::FFTWindowState {
        friend class HALSITL::DSP;

    public:
        /**
         * @brief Construct FFT window state with specified parameters
         * 
         * @details Allocates complex buffer and initializes FFT analysis parameters.
         *          The complex buffer is used for in-place FFT computation, storing
         *          both time-domain input samples and frequency-domain output.
         * 
         * @param[in] window_size Number of samples per FFT window (power of 2)
         * @param[in] sample_rate Sample rate in Hz
         * @param[in] sliding_window_size Number of overlapping windows
         * 
         * @note Allocates window_size * sizeof(complexf) bytes for FFT buffer
         */
        FFTWindowStateSITL(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size);
        
        /**
         * @brief Destructor - releases allocated FFT buffer memory
         * 
         * @details Frees the complex number buffer allocated during construction.
         */
        virtual ~FFTWindowStateSITL();

    private:
        /**
         * @var buf
         * @brief Complex number buffer for in-place FFT computation
         * 
         * @details Stores time-domain samples (as complex numbers with zero
         *          imaginary component) before FFT, and frequency-domain
         *          coefficients (complex amplitudes) after FFT.
         *          
         *          Size: window_size complex floats
         *          Format: Array of std::complex<float> values
         *          
         *          Before FFT: buf[n] = complex(sample[n], 0)
         *          After FFT:  buf[k] = complex(real_k, imag_k) for frequency bin k
         */
        complexf* buf;
    };

private:
    /**
     * @brief Apply Hanning window to input samples
     * 
     * @details Reads samples from input buffer and applies Hanning window function
     *          to reduce spectral leakage in FFT analysis. The Hanning window is
     *          a cosine-shaped taper that smoothly reduces sample amplitudes to
     *          zero at the window edges.
     *          
     *          Hanning window formula: w(n) = 0.5 * (1 - cos(2*pi*n/N))
     *          where n is sample index [0, N-1] and N is window size.
     *          
     *          Windowed output: output[n] = input[n] * w(n)
     *          
     *          This reduces frequency leakage from discontinuities at window
     *          boundaries, improving frequency resolution and reducing sidelobes
     *          in the FFT output spectrum.
     * 
     * @param[in,out] fft FFT state containing Hanning coefficients and output buffer
     * @param[in]     samples Input sample buffer (time-domain data)
     * @param[in]     advance Number of samples to advance for next window
     * 
     * @note Hanning window coefficients are precomputed in FFTWindowState
     * @see fft_start() which calls this method
     */
    void step_hanning(FFTWindowStateSITL* fft, FloatBuffer& samples, uint16_t advance);
    
    /**
     * @brief Compute magnitude spectrum from FFT output
     * 
     * @details Converts complex FFT output to real-valued magnitude spectrum
     *          by computing |Z| = sqrt(real^2 + imag^2) for each frequency bin.
     *          The magnitude represents the amplitude of each frequency component.
     *          
     *          This step prepares the frequency data for peak detection and
     *          amplitude analysis in fft_analyse().
     * 
     * @param[in,out] fft FFT state with complex frequency data
     * 
     * @note Called internally by fft_analyse() after calculate_fft()
     */
    void step_fft(FFTWindowStateSITL* fft);
    
    /**
     * @brief Element-wise multiplication of two float vectors
     * 
     * @details Computes vout[i] = v1[i] * v2[i] for i in [0, len-1].
     *          Used for applying window functions and other element-wise operations.
     * 
     * @param[in]  v1 First input vector
     * @param[in]  v2 Second input vector
     * @param[out] vout Output vector (element-wise product)
     * @param[in]  len Vector length (number of elements)
     * 
     * @note Output vector may alias input vectors (in-place operation supported)
     */
    void mult_f32(const float* v1, const float* v2, float* vout, uint16_t len);
    
    /**
     * @brief Find maximum value and its index in float vector
     * 
     * @details Searches vector for maximum value and returns both the value
     *          and its index position. Used to identify peak frequency bin
     *          in magnitude spectrum.
     * 
     * @param[in]  vin Input vector to search
     * @param[in]  len Vector length (number of elements)
     * @param[out] maxValue Pointer to store maximum value found
     * @param[out] maxIndex Pointer to store index of maximum value
     * 
     * @note Overrides AP_HAL::DSP base class method
     */
    void vector_max_float(const float* vin, uint16_t len, float* maxValue, uint16_t* maxIndex) const override;
    
    /**
     * @brief Scale float vector by constant factor
     * 
     * @details Computes vout[i] = vin[i] * scale for i in [0, len-1].
     *          Used for normalizing FFT output and other scaling operations.
     * 
     * @param[in]  vin Input vector
     * @param[in]  scale Scaling factor to multiply
     * @param[out] vout Output vector (scaled values)
     * @param[in]  len Vector length (number of elements)
     * 
     * @note Output vector may alias input vector (in-place operation supported)
     * @note Overrides AP_HAL::DSP base class method
     */
    void vector_scale_float(const float* vin, float scale, float* vout, uint16_t len) const override;
    
    /**
     * @brief Calculate mean (average) of float vector
     * 
     * @details Computes arithmetic mean: sum(vin[i]) / len
     *          Used for noise floor estimation and DC offset calculation.
     * 
     * @param[in] vin Input vector
     * @param[in] len Vector length (number of elements)
     * 
     * @return Mean value of vector elements
     * 
     * @note Overrides AP_HAL::DSP base class method
     */
    float vector_mean_float(const float* vin, uint16_t len) const override;
    
    /**
     * @brief Element-wise addition of two float vectors
     * 
     * @details Computes vout[i] = vin1[i] + vin2[i] for i in [0, len-1].
     *          Used for accumulating FFT results and other vector operations.
     * 
     * @param[in]  vin1 First input vector
     * @param[in]  vin2 Second input vector
     * @param[out] vout Output vector (element-wise sum)
     * @param[in]  len Vector length (number of elements)
     * 
     * @note Output vector may alias input vectors (in-place operation supported)
     * @note Overrides AP_HAL::DSP base class method
     */
    void vector_add_float(const float* vin1, const float* vin2, float* vout, uint16_t len) const override;
    
    /**
     * @brief Perform in-place Cooley-Tukey FFT on complex array
     * 
     * @details Implements the Cooley-Tukey Fast Fourier Transform algorithm,
     *          which recursively divides the FFT into smaller DFTs for efficient
     *          computation. Transforms time-domain samples to frequency-domain
     *          coefficients in-place using radix-2 decimation.
     *          
     *          Algorithm complexity: O(N log N) where N = length
     *          
     *          The FFT computes: F[k] = sum(f[n] * e^(-2*pi*i*k*n/N)) for n=0..N-1
     *          where F[k] is the k-th frequency bin (complex amplitude).
     *          
     *          Input format: Array of complex numbers representing time samples
     *          Output format: Array of complex numbers representing frequency bins
     *          
     *          - Bin 0: DC component (0 Hz)
     *          - Bin k: Frequency = k * sample_rate / length Hz
     *          - Bins > length/2: Negative frequencies (mirror of positive)
     * 
     * @param[in,out] f Complex array to transform (time → frequency domain)
     * @param[in]     length Array length (must be power of 2)
     * 
     * @warning length must be a power of 2, otherwise algorithm fails
     * @warning Input array is modified in-place (original time data is destroyed)
     * 
     * @note This is the core FFT computation engine
     * @see https://en.wikipedia.org/wiki/Cooley%E2%80%93Tukey_FFT_algorithm
     */
    void calculate_fft(complexf* f, uint16_t length);
};

#endif
