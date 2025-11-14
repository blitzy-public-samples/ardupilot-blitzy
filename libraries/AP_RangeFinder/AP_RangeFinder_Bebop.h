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
 * @file AP_RangeFinder_Bebop.h
 * @brief Parrot Bebop ultrasonic rangefinder backend driver
 * 
 * @details This driver implements support for the Parrot Bebop drone's built-in
 *          ultrasonic rangefinder sensor using Linux Industrial I/O (IIO) subsystem
 *          and SPI waveform capture.
 *          
 *          The Bebop rangefinder uses an ultrasonic transducer that transmits
 *          pulses and captures echo reflections via an ADC through the IIO interface.
 *          The driver performs real-time waveform analysis to detect echoes and
 *          calculate distance based on time-of-flight measurements.
 *          
 *          Key Features:
 *          - Dual-mode operation: switches between short-range and long-range modes
 *            based on altitude
 *          - Echo detection with local maxima search algorithm
 *          - Digital filtering and averaging to reduce noise
 *          - SPI-based waveform transmission and GPIO control
 *          - Runs in dedicated Linux thread for real-time operation
 *          
 *          Platform Requirements:
 *          - Linux HAL with IIO support
 *          - GPIO control capability
 *          - SPI device interface
 *          - Specific to Parrot Bebop hardware
 *          
 *          Hardware Interface:
 *          - Ultrasonic transducer controlled via GPIO
 *          - ADC capture via IIO device (/dev/iio:deviceX)
 *          - SPI for waveform generation
 *          
 * @note This driver is platform-specific to the Parrot Bebop drone and requires
 *       Linux HAL with IIO subsystem support. It will not work on other platforms.
 * 
 * @warning The ultrasonic sensor has limited accuracy at extreme ranges and may
 *          produce false readings in certain acoustic environments.
 * 
 * @see AP_RangeFinder_Backend
 * @see AP_HAL_Linux::Thread
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BEBOP_ENABLED

#include <AP_HAL_Linux/Thread.h>

/**
 * @brief Maximum number of ultrasonic pulses in SPI waveform buffer
 * 
 * @details This defines the size of the buffer containing the waveform pattern
 *          sent over SPI to generate ultrasonic pulses. The buffer contains the
 *          timing pattern for the transducer excitation signal.
 */
#define RNFD_BEBOP_NB_PULSES_MAX 32

/**
 * @brief Size of the purge buffer for SPI initialization
 * 
 * @details A larger buffer is sent during initialization to clear any residual
 *          data in the SPI pipeline. This purge operation ensures clean startup
 *          of the ultrasonic measurement cycle.
 */
#define RNFD_BEBOP_NB_PULSES_PURGE 64

/**
 * @brief Default ultrasonic measurement frequency in Hz
 * 
 * @details The rangefinder performs 17 measurements per second by default,
 *          providing a good balance between update rate and acoustic interference.
 *          This rate allows sufficient time for echo returns between pulses.
 */
#define RNFD_BEBOP_DEFAULT_FREQ 17

/**
 * @brief Default ADC sampling frequency in Hz
 * 
 * @details The ADC samples at 160 kHz to capture the ultrasonic echo waveform
 *          with sufficient resolution. This sampling rate provides adequate
 *          temporal resolution for accurate time-of-flight measurement.
 * 
 * @note Higher sampling rates improve distance resolution but increase CPU
 *       processing load for waveform analysis.
 */
#define RNFD_BEBOP_DEFAULT_ADC_FREQ 160000

/**
 * @brief Averaging filter power-of-two exponent
 * 
 * @details The driver averages (1 << RNFD_BEBOP_FILTER_POWER) = 4 consecutive
 *          samples to reduce noise in the captured waveform. This simple moving
 *          average filter improves signal-to-noise ratio for echo detection.
 */
#define RNFD_BEBOP_FILTER_POWER 2

/**
 * @brief Speed of sound in meters per second
 * 
 * @details Used for time-of-flight distance calculation. Assumes standard
 *          atmospheric conditions (340 m/s at ~15°C). Distance calculation:
 *          distance = (time_of_flight * RNFD_BEBOP_SOUND_SPEED) / 2
 *          Division by 2 accounts for round-trip travel of the ultrasonic pulse.
 * 
 * @note This is a constant approximation and does not compensate for temperature,
 *       humidity, or altitude effects on sound velocity.
 */
#define RNFD_BEBOP_SOUND_SPEED 340

/**
 * @brief Altitude threshold for switching to short-range mode (meters)
 * 
 * @details When measured altitude exceeds 0.75 meters, the driver switches to
 *          mode 0 (short-range mode). This mode uses different pulse timing
 *          optimized for closer distances.
 * 
 * @see RNFD_BEBOP_TRANSITION_LOW_TO_HIGH
 * @see RNFD_BEBOP_TRANSITION_COUNT
 */
#define RNFD_BEBOP_TRANSITION_HIGH_TO_LOW 0.75

/**
 * @brief Altitude threshold for switching to long-range mode (meters)
 * 
 * @details When measured altitude drops below 1.5 meters, the driver switches to
 *          mode 1 (long-range mode). This mode uses longer pulse timing for
 *          extended range detection. Hysteresis between HIGH_TO_LOW and LOW_TO_HIGH
 *          prevents rapid mode oscillation.
 * 
 * @see RNFD_BEBOP_TRANSITION_HIGH_TO_LOW
 * @see RNFD_BEBOP_TRANSITION_COUNT
 */
#define RNFD_BEBOP_TRANSITION_LOW_TO_HIGH 1.5

/**
 * @brief Hysteresis count for mode switching stability
 * 
 * @details The driver requires 5 consecutive measurements crossing the threshold
 *          before actually switching modes. This hysteresis prevents oscillation
 *          between modes when altitude is near the transition thresholds.
 */
#define RNFD_BEBOP_TRANSITION_COUNT 5

/**
 * @brief Maximum number of echoes to store and analyze
 * 
 * @details The waveform analysis can detect up to 30 distinct echoes (reflections)
 *          in a single measurement cycle. Multiple echoes occur due to reflections
 *          from multiple surfaces, acoustic multipath, or noise artifacts. The
 *          algorithm selects the most reliable echo for distance reporting.
 */
#define RNFD_BEBOP_MAX_ECHOES 30

/**
 * @struct echo
 * @brief Echo detection result structure for ultrasonic waveform analysis
 * 
 * @details This structure stores information about a detected echo in the captured
 *          ultrasonic waveform. Each echo represents a potential reflection from a
 *          surface. The structure captures both the peak amplitude location and the
 *          rising edge threshold crossing, which provides more accurate distance
 *          estimation than using peak alone.
 *          
 *          The distance_index represents the first sample where the signal exceeds
 *          a threshold set relative to the maximum amplitude. This leading edge
 *          detection compensates for echo shape variations and provides more
 *          consistent distance measurement than peak detection alone.
 */
struct echo {
    /**
     * @brief Index in the capture buffer where maximum echo amplitude occurs
     * 
     * @details This is the sample index with the highest signal amplitude for this
     *          echo. Used to determine echo strength and to calculate the threshold
     *          for leading edge detection.
     */
    int max_index;
    
    /**
     * @brief Index in the capture buffer for leading edge threshold crossing
     * 
     * @details This is the first sample index where the signal amplitude exceeds a
     *          fixed threshold below the maximum amplitude. This index corresponds
     *          to the actual distance measurement because the leading edge of the
     *          echo pulse is less affected by amplitude variations than the peak.
     *          
     *          Distance calculation: distance = (distance_index / ADC_freq) * sound_speed / 2
     */
    int distance_index;
};

/**
 * @struct adc_capture
 * @brief ADC capture configuration for Linux IIO subsystem
 * 
 * @details This structure contains all configuration and state information for
 *          capturing ultrasonic echo waveforms through the Linux Industrial I/O
 *          (IIO) subsystem. The IIO framework provides a standardized interface
 *          to ADC devices in the Linux kernel.
 *          
 *          The ADC continuously samples the ultrasonic transducer output during
 *          the echo reception window, capturing the complete waveform for analysis.
 *          The captured waveform is processed to detect echo peaks and calculate
 *          distance based on time-of-flight.
 *          
 * @note This structure interfaces directly with the Linux IIO kernel subsystem
 *       via libiio library calls.
 */
struct adc_capture {
    /**
     * @brief Pointer to IIO device representing the ADC hardware
     * 
     * @details Handle to the IIO device node (typically /dev/iio:deviceX) used
     *          for ADC configuration and control. Initialized during driver setup.
     */
    struct iio_device *device;
    
    /**
     * @brief Pointer to IIO buffer for waveform capture
     * 
     * @details Ring buffer allocated in kernel space for high-speed ADC data
     *          capture. The buffer is memory-mapped into userspace for efficient
     *          access to captured waveform samples.
     */
    struct iio_buffer *buffer;
    
    /**
     * @brief Size of the ADC capture buffer in samples
     * 
     * @details Determines how many ADC samples can be captured in a single
     *          acquisition. Buffer size is calculated based on ADC frequency
     *          and maximum expected echo return time.
     */
    unsigned int buffer_size;
    
    /**
     * @brief Pointer to IIO channel for ADC input
     * 
     * @details Handle to the specific IIO channel connected to the ultrasonic
     *          receiver circuit. The channel configuration includes gain and
     *          sampling parameters.
     */
    struct iio_channel *channel;
    
    /**
     * @brief ADC sampling frequency in Hz
     * 
     * @details Configured sampling rate for the ADC. Higher frequencies provide
     *          better temporal resolution for distance measurement but increase
     *          processing load. Typically set to RNFD_BEBOP_DEFAULT_ADC_FREQ (160 kHz).
     */
    unsigned int freq;
    
    /**
     * @brief Time-based echo rejection threshold
     * 
     * @details Minimum time difference (in ADC samples) required between echoes
     *          in consecutive acquisitions to be considered as distinct echoes.
     *          This threshold helps match echoes across multiple measurement cycles
     *          and reject spurious detections. Prevents the same physical echo from
     *          being counted multiple times if it persists across acquisitions.
     */
    unsigned short threshold_time_rejection;
};

/**
 * @class AP_RangeFinder_Bebop
 * @brief Rangefinder backend for Parrot Bebop ultrasonic sensor
 * 
 * @details This class implements a rangefinder driver for the Parrot Bebop drone's
 *          integrated ultrasonic altitude sensor. The driver uses Linux-specific
 *          interfaces (IIO subsystem, GPIO, SPI) to control the ultrasonic
 *          transducer and capture echo waveforms for time-of-flight distance
 *          measurement.
 *          
 *          Architecture:
 *          - Runs in a dedicated Linux thread for real-time waveform capture
 *          - Uses SPI to generate ultrasonic pulse waveforms
 *          - Captures echo waveforms via ADC through Linux IIO subsystem
 *          - Performs digital signal processing to detect echoes
 *          - Implements dual-mode operation (short/long range) with hysteresis
 *          
 *          Measurement Cycle:
 *          1. Configure GPIO to enable transducer
 *          2. Send pulse waveform via SPI
 *          3. Capture ADC waveform via IIO buffer
 *          4. Apply averaging filter to reduce noise
 *          5. Search for local maxima (echo peaks)
 *          6. Select best echo based on amplitude and position
 *          7. Calculate distance from time-of-flight
 *          8. Update rangefinder state
 *          
 *          Mode Switching:
 *          The driver automatically switches between two modes based on altitude:
 *          - Mode 0 (short-range): altitude > 0.75m, optimized for close objects
 *          - Mode 1 (long-range): altitude < 1.5m, extended range detection
 *          Hysteresis prevents oscillation between modes.
 *          
 * @note This driver is platform-specific to Parrot Bebop and requires:
 *       - Linux HAL with IIO subsystem support
 *       - libiio library for ADC access
 *       - GPIO control for transducer enable
 *       - SPI device for waveform generation
 *       
 * @warning Thread-safety: The _loop() method runs in a separate thread and
 *          accesses shared state variables. Proper synchronization is required.
 *          
 * @see AP_RangeFinder_Backend
 * @see AP_HAL_Linux::Thread
 */
class AP_RangeFinder_Bebop : public AP_RangeFinder_Backend {
public:
    /**
     * @brief Constructor for Bebop rangefinder backend
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing
     *                       measurement results (distance, status, etc.)
     * @param[in] _params Reference to rangefinder configuration parameters
     *                    (offsets, scaling, limits, etc.)
     * 
     * @details Initializes the Bebop rangefinder driver and allocates resources.
     *          The constructor sets up initial state but does not configure hardware.
     *          Hardware initialization occurs in _init() called from the measurement
     *          thread.
     */
    AP_RangeFinder_Bebop(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Destructor for Bebop rangefinder backend
     * 
     * @details Cleans up resources including:
     *          - Stops the measurement thread
     *          - Closes IIO devices and buffers
     *          - Releases SPI device
     *          - Frees allocated memory for capture buffers
     */
    ~AP_RangeFinder_Bebop(void);
    
    /**
     * @brief Detect if Bebop ultrasonic hardware is present
     * 
     * @return true if Bebop rangefinder hardware is detected and accessible
     * @return false if hardware is not present or cannot be initialized
     * 
     * @details Probes for the presence of Bebop-specific hardware by attempting to:
     *          - Open IIO device for ADC access
     *          - Verify SPI device availability
     *          - Check GPIO availability for transducer control
     *          
     *          This static method is called during rangefinder initialization to
     *          determine if this backend should be instantiated.
     *          
     * @note This is a static method that can be called without instantiating the class.
     *       It performs minimal hardware probing without full initialization.
     */
    static bool detect();
    
    /**
     * @brief Update rangefinder state with latest measurement
     * 
     * @details Called periodically by the rangefinder library to update the sensor
     *          state with the most recent distance measurement. This method reads
     *          the latest distance calculated by the background thread and updates
     *          the rangefinder state structure.
     *          
     *          The actual measurement and waveform processing occurs in the
     *          background thread (_loop()). This method simply retrieves the
     *          result and updates the shared state.
     *          
     * @note This method is called from the main vehicle thread at the rangefinder
     *       update rate. The background measurement thread runs independently.
     *       
     * @see _loop()
     */
    void update(void) override;

protected:
    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser rangefinder type
     * 
     * @details Returns the MAVLink sensor type classification for this rangefinder.
     *          Although the Bebop uses ultrasonic technology, it reports as LASER
     *          type for MAVLink compatibility and ground station display purposes.
     *          
     *          This classification affects how ground control stations display and
     *          interpret the sensor data but does not affect the actual measurement
     *          mechanism.
     *          
     * @note The MAV_DISTANCE_SENSOR_LASER type is used for historical compatibility
     *       rather than technical accuracy (the sensor is actually ultrasonic).
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @brief Initialize hardware and allocate resources
     * 
     * @details Performs one-time initialization including:
     *          - Opening IIO device for ADC access
     *          - Configuring ADC channels and sampling rate
     *          - Allocating capture buffers
     *          - Opening SPI device for waveform generation
     *          - Initializing GPIO for transducer control
     *          - Configuring initial measurement mode
     *          
     *          Called once from the measurement thread on first execution.
     *          Failure in initialization will disable the rangefinder.
     *          
     * @note This method runs in the measurement thread context, not the main thread.
     */
    void _init(void);
    
    /**
     * @brief Launch an ultrasonic measurement cycle
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @details Initiates a complete measurement cycle:
     *          1. Configures GPIO to enable ultrasonic transducer
     *          2. Sends pulse waveform via SPI to generate ultrasonic pulse
     *          3. Prepares IIO buffer for echo capture
     *          
     *          This method starts the pulse transmission. Echo capture is handled
     *          separately by _capture().
     *          
     * @see _capture()
     * @see _configure_gpio()
     * @see _configure_wave()
     */
    int _launch(void);
    
    /**
     * @brief Capture and process ADC waveform data
     * 
     * @return 0 on success with valid measurement, negative on error
     * 
     * @details Captures the ultrasonic echo waveform via IIO and processes it:
     *          1. Reads ADC samples from IIO buffer
     *          2. Applies averaging filter to reduce noise
     *          3. Searches for local maxima (potential echoes)
     *          4. Selects best echo based on amplitude and position criteria
     *          5. Calculates distance from time-of-flight
     *          6. Updates rangefinder state with measurement result
     *          
     *          The waveform processing pipeline is the core of the distance
     *          measurement algorithm.
     *          
     * @see _apply_averaging_filter()
     * @see _search_local_maxima()
     * @see _search_maximum_with_max_amplitude()
     */
    int _capture(void);
    
    /**
     * @brief Update measurement mode based on altitude
     * 
     * @param[in] altitude Current altitude estimate in meters
     * 
     * @return 0 on success, negative on error
     * 
     * @details Implements dual-mode switching logic with hysteresis:
     *          - Mode 0 (short-range): Used when altitude > 0.75m
     *          - Mode 1 (long-range): Used when altitude < 1.5m
     *          
     *          Requires RNFD_BEBOP_TRANSITION_COUNT consecutive measurements
     *          crossing the threshold before switching to prevent oscillation.
     *          
     *          Mode switching affects pulse timing and echo detection parameters.
     *          
     * @note The hysteresis prevents rapid mode switching when hovering near
     *       transition thresholds.
     *       
     * @see RNFD_BEBOP_TRANSITION_HIGH_TO_LOW
     * @see RNFD_BEBOP_TRANSITION_LOW_TO_HIGH
     * @see RNFD_BEBOP_TRANSITION_COUNT
     */
    int _update_mode(float altitude);
    
    /**
     * @brief Configure GPIO pin for transducer control
     * 
     * @param[in] value GPIO state: 0 = disable transducer, 1 = enable transducer
     * 
     * @details Controls the GPIO pin connected to the ultrasonic transducer
     *          enable circuit. The transducer must be enabled before pulse
     *          transmission and disabled afterward to reduce power consumption
     *          and prevent continuous operation.
     *          
     * @note The GPIO pin number and polarity are hardware-specific to the
     *       Bebop platform.
     */
    void _configure_gpio(int value);
    
    /**
     * @brief Configure SPI waveform for ultrasonic pulse generation
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @details Prepares the SPI transmit buffer with the waveform pattern for
     *          generating the ultrasonic pulse. The waveform consists of a
     *          sequence of bits that, when clocked out via SPI, create the
     *          appropriate excitation signal for the ultrasonic transducer.
     *          
     *          The waveform pattern depends on the current measurement mode:
     *          - Mode 0: Short pulses for close-range detection
     *          - Mode 1: Longer pulses for extended range
     *          
     * @see _reconfigure_wave()
     */
    int _configure_wave();
    
    /**
     * @brief Reconfigure waveform when switching measurement modes
     * 
     * @details Updates the SPI waveform buffer when the measurement mode changes.
     *          Different modes use different pulse patterns optimized for their
     *          respective distance ranges.
     *          
     *          Called by _update_mode() when mode transition occurs.
     *          
     * @see _configure_wave()
     * @see _update_mode()
     */
    void _reconfigure_wave();
    
    /**
     * @brief Configure IIO buffer for ADC capture
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @details Configures the IIO subsystem for ADC waveform capture:
     *          - Sets buffer size based on expected echo return time
     *          - Configures sampling frequency
     *          - Enables IIO channel
     *          - Prepares buffer for data acquisition
     *          
     *          Must be called before each capture cycle to ensure buffer is ready.
     */
    int _configure_capture();
    
    /**
     * @brief Send purge waveform to initialize SPI pipeline
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @details Sends a longer waveform buffer during initialization to clear any
     *          residual data in the SPI hardware. This purge operation ensures
     *          clean startup and prevents corrupted first measurements.
     *          
     *          Uses RNFD_BEBOP_NB_PULSES_PURGE sized buffer instead of the normal
     *          RNFD_BEBOP_NB_PULSES_MAX.
     *          
     * @note Only called during initialization, not during normal operation.
     */
    int _launch_purge();
    
    /**
     * @brief Main measurement loop running in dedicated thread
     * 
     * @details This method runs continuously in a separate Linux thread, performing
     *          periodic ultrasonic measurements:
     *          
     *          Loop cycle:
     *          1. Initialize hardware (first iteration only)
     *          2. Update measurement mode based on altitude
     *          3. Launch ultrasonic pulse (_launch)
     *          4. Capture and process echo waveform (_capture)
     *          5. Calculate distance from time-of-flight
     *          6. Update shared state with result
     *          7. Sleep until next measurement cycle
     *          
     *          The loop runs at RNFD_BEBOP_DEFAULT_FREQ (17 Hz) by default.
     *          
     * @warning This method accesses shared state variables that are also read by
     *          update(). Proper synchronization mechanisms should be used to prevent
     *          race conditions.
     *          
     * @note This is the thread entry point. It runs until the driver is destroyed.
     *       
     * @see _init()
     * @see _launch()
     * @see _capture()
     */
    void _loop(void);

    /**
     * @brief Pointer to measurement thread object
     * 
     * @details Handle to the Linux thread that runs the measurement loop.
     *          Created during constructor, started immediately, and stopped
     *          in destructor.
     */
    Linux::Thread *_thread;
    
    /**
     * @brief Calculate threshold for echo detection at specific capture index
     * 
     * @param[in] i_capture Sample index in the capture buffer
     * 
     * @return Amplitude threshold value for echo detection at this position
     * 
     * @details Calculates the threshold above which a signal is considered a valid
     *          echo. The threshold may vary with position in the capture buffer to
     *          account for signal attenuation over time/distance.
     *          
     *          Used during echo detection to filter noise and identify true echoes.
     *          
     * @see _search_local_maxima()
     */
    unsigned short get_threshold_at(int i_capture);
    
    /**
     * @brief Apply averaging filter to captured waveform
     * 
     * @return 0 on success, negative on error
     * 
     * @details Applies a simple moving average filter to the raw ADC samples to
     *          reduce high-frequency noise. Averages (1 << RNFD_BEBOP_FILTER_POWER)
     *          consecutive samples.
     *          
     *          The filtered waveform is stored in _filtered_capture buffer and
     *          used for subsequent echo detection.
     *          
     *          Filtering improves signal-to-noise ratio at the cost of slight
     *          temporal resolution reduction.
     *          
     * @note This is a pre-processing step before echo detection.
     *       
     * @see _search_local_maxima()
     */
    int _apply_averaging_filter(void);
    
    /**
     * @brief Search for local maxima in filtered waveform
     * 
     * @return Number of echoes detected, or negative on error
     * 
     * @details Analyzes the filtered waveform to identify local maxima that
     *          represent potential echoes. For each local maximum:
     *          1. Verifies amplitude exceeds noise threshold
     *          2. Stores max_index (peak position)
     *          3. Searches backward for leading edge (distance_index)
     *          4. Stores echo in _echoes array
     *          
     *          Multiple echoes may be detected due to:
     *          - Multiple reflecting surfaces
     *          - Acoustic multipath
     *          - Noise artifacts
     *          
     *          The best echo is selected by _search_maximum_with_max_amplitude().
     *          
     * @note Populates the _echoes array with detected echoes and updates _nb_echoes.
     *       
     * @see _search_maximum_with_max_amplitude()
     * @see get_threshold_at()
     */
    int _search_local_maxima(void);
    
    /**
     * @brief Select the best echo from detected candidates
     * 
     * @return Index of selected echo in _echoes array, or negative if no valid echo
     * 
     * @details Analyzes all detected echoes and selects the most reliable one
     *          based on:
     *          - Echo amplitude (stronger echoes preferred)
     *          - Echo position (closer echoes preferred if amplitude is sufficient)
     *          - Consistency with previous measurements
     *          - Rejection of noise artifacts
     *          
     *          The selection algorithm balances accuracy and reliability by
     *          preferring strong echoes while avoiding obvious multipath or noise.
     *          
     * @note This is the final step in echo processing before distance calculation.
     *       
     * @see _search_local_maxima()
     */
    int _search_maximum_with_max_amplitude(void);

    /**
     * @brief SPI device handle for waveform transmission
     * 
     * @details Owned pointer to the SPI device used to transmit the ultrasonic
     *          pulse waveform. The SPI clock generates the timing pattern that
     *          drives the ultrasonic transducer.
     */
    AP_HAL::OwnPtr<AP_HAL::Device> _spi;
    
    /**
     * @brief GPIO interface for transducer control
     * 
     * @details Pointer to GPIO interface used to enable/disable the ultrasonic
     *          transducer. The GPIO controls power or enable pin to the
     *          transducer circuit.
     */
    AP_HAL::GPIO *_gpio;

    /**
     * @brief ADC capture configuration and state
     * 
     * @details Structure containing all IIO-related configuration for ADC waveform
     *          capture including device handles, buffer configuration, and sampling
     *          parameters.
     *          
     * @see adc_capture
     */
    struct adc_capture _adc;
    
    /**
     * @brief IIO context for Linux Industrial I/O subsystem
     * 
     * @details Handle to the IIO context providing access to IIO devices.
     *          This is the top-level handle for all IIO operations including
     *          device enumeration and access.
     */
    struct iio_context *_iio;

    /**
     * @brief SPI transmit buffers for two measurement modes
     * 
     * @details Two waveform buffers, one for each measurement mode:
     *          - _tx[0]: Short-range mode waveform
     *          - _tx[1]: Long-range mode waveform
     *          
     *          Each buffer contains RNFD_BEBOP_NB_PULSES_MAX bytes defining the
     *          pulse pattern for the ultrasonic transducer.
     */
    unsigned char _tx[2][RNFD_BEBOP_NB_PULSES_MAX];
    
    /**
     * @brief SPI purge buffer for initialization
     * 
     * @details Larger buffer used during initialization to clear the SPI pipeline.
     *          Size is RNFD_BEBOP_NB_PULSES_PURGE bytes (twice the normal buffer).
     *          
     * @see _launch_purge()
     */
    unsigned char _purge[RNFD_BEBOP_NB_PULSES_PURGE];
    
    /**
     * @brief Pointer to currently active transmit buffer
     * 
     * @details Points to either _tx[0] or _tx[1] depending on current measurement
     *          mode. Updated when mode switching occurs.
     */
    unsigned char* _tx_buf;
    
    /**
     * @brief Counter for mode switching hysteresis
     * 
     * @details Counts consecutive measurements crossing the mode transition
     *          threshold. Mode switch occurs when counter reaches
     *          RNFD_BEBOP_TRANSITION_COUNT.
     *          
     *          Positive values: counting transitions to long-range mode
     *          Negative values: counting transitions to short-range mode
     *          Zero: no pending mode transition
     *          
     * @see _update_mode()
     * @see RNFD_BEBOP_TRANSITION_COUNT
     */
    int _hysteresis_counter;
    
    /**
     * @brief Initial threshold for echo detection
     * 
     * @details Minimum signal amplitude (in ADC units) required to consider a
     *          waveform feature as a potential echo. This threshold filters out
     *          low-amplitude noise before local maxima search.
     *          
     *          Value of 1500 is calibrated for the Bebop hardware's ADC range
     *          and typical echo amplitudes.
     */
    const unsigned int threshold_echo_init = 1500;
    
    /**
     * @brief File descriptor for IIO device (legacy, may be unused)
     * 
     * @details File descriptor for direct IIO device access. Initialized to -1
     *          (invalid). Modern IIO access uses _iio context rather than raw
     *          file descriptor.
     *          
     * @note This may be legacy code; IIO operations primarily use _iio and _adc.
     */
    int _fd = -1;
    
    /**
     * @brief Timestamp of last successful measurement
     * 
     * @details Microsecond timestamp of the previous measurement, used to:
     *          - Calculate measurement rate
     *          - Detect measurement timeouts
     *          - Synchronize with other sensors
     *          
     *          Updated after each successful distance calculation.
     */
    uint64_t _last_timestamp;
    
    /**
     * @brief Current measurement mode
     * 
     * @details Operating mode for the rangefinder:
     *          - 0: Short-range mode (altitude > 0.75m)
     *          - 1: Long-range mode (altitude < 1.5m)
     *          
     *          Mode determines pulse waveform and echo detection parameters.
     *          
     * @see _update_mode()
     */
    int _mode;
    
    /**
     * @brief Number of echoes detected in current measurement
     * 
     * @details Count of local maxima identified in the waveform. May range from
     *          0 (no echoes) to RNFD_BEBOP_MAX_ECHOES (30). Multiple echoes occur
     *          due to reflections from multiple surfaces or acoustic multipath.
     *          
     * @see _search_local_maxima()
     */
    int _nb_echoes;
    
    /**
     * @brief Measurement frequency in Hz
     * 
     * @details Rate at which ultrasonic measurements are performed. Typically set
     *          to RNFD_BEBOP_DEFAULT_FREQ (17 Hz). Lower frequencies reduce
     *          acoustic interference; higher frequencies improve update rate.
     */
    int _freq;
    
    /**
     * @brief Current altitude estimate in meters
     * 
     * @details Most recent distance measurement used for mode switching logic.
     *          Updated after each successful measurement cycle.
     *          
     * @see _update_mode()
     */
    float _altitude;
    
    /**
     * @brief Buffer for filtered ADC waveform
     * 
     * @details Dynamically allocated buffer containing the averaged waveform after
     *          applying the moving average filter. Size is _filtered_capture_size.
     *          
     *          Memory is allocated during initialization based on ADC sampling rate
     *          and maximum echo return time.
     *          
     * @see _apply_averaging_filter()
     */
    unsigned int *_filtered_capture;
    
    /**
     * @brief Size of filtered capture buffer
     * 
     * @details Number of samples in _filtered_capture buffer. Calculated based on:
     *          - ADC sampling frequency
     *          - Maximum expected distance
     *          - Averaging filter reduction factor
     */
    unsigned int _filtered_capture_size;
    
    /**
     * @brief Array of detected echoes
     * 
     * @details Storage for up to RNFD_BEBOP_MAX_ECHOES (30) detected echo structures.
     *          Populated by _search_local_maxima() and analyzed by
     *          _search_maximum_with_max_amplitude().
     *          
     * @see echo
     * @see _search_local_maxima()
     */
    struct echo _echoes[RNFD_BEBOP_MAX_ECHOES];
    
    /**
     * @brief Number of samples to average in moving average filter
     * 
     * @details Set to (1 << RNFD_BEBOP_FILTER_POWER) = 4. Each output sample
     *          is the average of 4 consecutive input samples, reducing noise at
     *          the cost of slight temporal resolution loss.
     *          
     * @see _apply_averaging_filter()
     * @see RNFD_BEBOP_FILTER_POWER
     */
    unsigned int _filter_average = 4;
    
    /**
     * @brief Maximum detection range in meters
     * 
     * @details Upper limit for valid distance measurements. Default is 8.50 meters.
     *          Measurements exceeding this range are rejected as out-of-range or
     *          unreliable. Based on ultrasonic propagation limits and echo strength.
     */
    float _last_max_distance = 8.50;
    
    /**
     * @brief Minimum detection range in meters
     * 
     * @details Lower limit for valid distance measurements. Default is 0.32 meters.
     *          Measurements below this range are rejected due to:
     *          - Transmit pulse ring-down interference
     *          - Near-field acoustic effects
     *          - Insufficient time-of-flight resolution
     */
    float _last_min_distance = 0.32;
};


#endif  // AP_RANGEFINDER_BEBOP_ENABLED
