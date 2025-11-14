/**
 * @file RCInput.h
 * @brief RC receiver input interface for reading pilot control commands
 * 
 * @details Defines abstract interface for RC (Radio Control) receiver input from various protocols
 *          including PPM, SBUS, DSM, CRSF, and others. Provides channel values, signal quality
 *          monitoring, and protocol detection. This is a HAL (Hardware Abstraction Layer) interface
 *          that must be implemented by platform-specific code.
 * 
 * Source: libraries/AP_HAL/RCInput.h
 */

#pragma once

#include "AP_HAL_Namespace.h"

/**
 * @def RC_INPUT_MIN_PULSEWIDTH
 * @brief Minimum valid RC pulse width in microseconds
 * 
 * @details PWM pulse widths below this value (900μs) are considered invalid and are typically
 *          filtered out by the RC input system. Standard RC protocol minimum is 1000μs,
 *          but 900μs threshold provides margin for signal variations.
 */
#define RC_INPUT_MIN_PULSEWIDTH 900

/**
 * @def RC_INPUT_MAX_PULSEWIDTH
 * @brief Maximum valid RC pulse width in microseconds
 * 
 * @details PWM pulse widths above this value (2100μs) are considered invalid and are typically
 *          filtered out by the RC input system. Standard RC protocol maximum is 2000μs,
 *          but 2100μs threshold provides margin for signal variations.
 */
#define RC_INPUT_MAX_PULSEWIDTH 2100

/**
 * @class AP_HAL::RCInput
 * @brief Abstract interface for RC receiver input
 * 
 * @details Receives pilot commands from RC transmitter via various protocols.
 *          Platform-specific implementations decode protocol data and provide
 *          standardized channel values to the flight controller.
 *          
 *          Supported protocols (decoded by AP_RCProtocol in implementations):
 *          - PPM: Pulse Position Modulation (legacy, typically 8 channels)
 *          - SBUS/SBUS2: Futaba Serial Bus (16+ channels, digital, inverted UART)
 *          - DSM/DSM2/DSMX: Spektrum satellite (8-12 channels, serial)
 *          - SRXL/SRXL2: Multiplex SRXL (16+ channels, serial)
 *          - CRSF: Crossfire/ELRS (16 channels, bidirectional telemetry)
 *          - SUMD: Graupner SUMD (16 channels, serial)
 *          - ST24: Yuneec ST24 (18 channels, serial)
 *          - IBUS: FlySky IBUS (14 channels, serial)
 *          
 *          Standard channel mapping (typical pilot input):
 *          - Ch1 (0): Roll (aileron) - 1500μs centered
 *          - Ch2 (1): Pitch (elevator) - 1500μs centered
 *          - Ch3 (2): Throttle - 1000μs minimum
 *          - Ch4 (3): Yaw (rudder) - 1500μs centered
 *          - Ch5-Ch16 (4-15): Aux switches, knobs, flight modes
 *          
 *          PWM value ranges:
 *          - Standard range: 1000-2000μs
 *          - Centered (roll/pitch/yaw neutral): 1500μs
 *          - Low position (throttle min, switch low): 1000μs
 *          - High position (throttle max, switch high): 2000μs
 *          - Valid range (with margin): 900-2100μs (RC_INPUT_MIN/MAX_PULSEWIDTH)
 *          
 *          Implementation responsibilities:
 *          - Initialize hardware (UART, timer capture, GPIO)
 *          - Detect and decode RC protocol automatically
 *          - Validate pulse width ranges
 *          - Detect signal loss (timeout)
 *          - Provide thread-safe access to channel data
 * 
 * @note Channel numbering is zero-based: Ch1=0, Ch2=1, etc. (C convention)
 * @note Implementations typically run protocol decoder in interrupt context or high-priority thread
 * @note Signal loss detection critical for failsafe triggering
 * @warning RC input failure triggers vehicle failsafe - verify signal quality before arming
 * @warning Implementations must be thread-safe as read() may be called from multiple tasks
 * 
 * @see AP_RCProtocol for protocol decoding implementation
 * @see RC_Channel for mapping RC inputs to vehicle functions
 */
class AP_HAL::RCInput {
public:
    /**
     * @brief Initialize RC input hardware and protocol decoders
     * 
     * @details Called from platform HAL instance initialization to configure hardware
     *          for RC receiver input. Sets up timers, UARTs, DMA, and interrupt handlers
     *          as needed by the specific platform.
     *          
     *          Platform-specific initialization may include:
     *          - Configure UART for serial protocols (SBUS, DSM, CRSF, etc.)
     *          - Set up timer input capture for PPM
     *          - Initialize DMA for efficient data transfer
     *          - Register interrupt handlers for pin state changes
     *          - Enable protocol auto-detection logic
     *          - Allocate buffers for protocol decoding
     * 
     * @note Called once during vehicle startup
     * @note Must be called before any read() or new_input() calls
     * @note Implementation-specific arguments passed via platform HAL (e.g., ISRRegistry)
     * @note Type of RCInput implementation known at compile time, avoiding C++ type system complexity
     * 
     * @see teardown()
     */
    virtual void init() = 0;
    
    /**
     * @brief Shutdown RC input hardware and release resources
     * 
     * @details Optional cleanup method called when RC input is no longer needed.
     *          Default implementation does nothing. Platform implementations may
     *          override to disable interrupts, free buffers, or power down hardware.
     * 
     * @note Rarely used in practice as RC input typically active for vehicle lifetime
     * @note Default empty implementation avoids forcing all platforms to implement
     */
    virtual void teardown() {};

    /**
     * @brief Check if new RC data received since last call
     * 
     * @details Returns true if fresh RC channel data has arrived since the last call to
     *          this function. Used to detect signal presence and trigger failsafe on loss.
     *          
     *          Calling read() multiple times between new_input() calls returns the same
     *          (potentially stale) values. This function implements edge-detection to
     *          identify when protocol decoder has received and validated new frame.
     *          
     *          Typical usage pattern:
     *          - Called at regular intervals (e.g., 50Hz in main loop)
     *          - Returns true when new frame decoded
     *          - Returns false if no new data (signal loss, interference, or between frames)
     *          - Consecutive false returns indicate RC signal loss
     * 
     * @return bool true if fresh data available since last check, false otherwise
     * 
     * @note State is cleared on each call (edge-triggered, not level-triggered)
     * @note Used for failsafe detection: no new data for threshold period triggers failsafe
     * @note Different protocols have different frame rates (e.g., SBUS 7ms, DSM 11-22ms, PPM 20ms)
     * 
     * @warning Critical for safety: must accurately detect signal loss to trigger failsafe
     * 
     * @see read() to retrieve channel values
     */
    virtual bool new_input(void) = 0;

    /**
     * @brief Get number of valid channels in last received frame
     * 
     * @details Returns the count of channels decoded from the most recent RC frame.
     *          Channel count is protocol-dependent:
     *          - PPM: typically 8 channels (hardware-dependent)
     *          - SBUS: 16 channels standard, 18 with SBUS2
     *          - DSM2: 7-8 channels
     *          - DSMX: 8-12 channels
     *          - CRSF: 16 channels
     *          - SRXL2: up to 16 channels
     *          
     *          Channel count may vary frame-to-frame for some protocols or remain
     *          constant for others. Value reflects last successfully decoded frame.
     * 
     * @return uint8_t Number of valid channels (typically 4-18)
     * 
     * @note Zero return indicates no valid frame received yet
     * @note Use this to determine valid array bounds before calling read()
     * @note Some protocols always report fixed channel count, others variable
     * 
     * @see read() to retrieve individual channel values
     */
    virtual uint8_t  num_channels() = 0;

    /**
     * @brief Read latest RC channel value in microseconds
     * 
     * @details Returns the most recent PWM pulse width for the specified channel.
     *          Values typically range 1000-2000μs with 1500μs as center/neutral.
     *          
     *          Channel numbering is zero-based:
     *          - 0 = Ch1 (Roll/Aileron)
     *          - 1 = Ch2 (Pitch/Elevator)
     *          - 2 = Ch3 (Throttle)
     *          - 3 = Ch4 (Yaw/Rudder)
     *          - 4+ = Aux channels (modes, switches, knobs)
     * 
     * @param[in] ch Channel number (0-based, typically 0-15)
     * 
     * @return uint16_t Pulse width in microseconds (typically 1000-2000μs)
     * 
     * @note Returns last received value - may be stale if signal lost
     * @note Use new_input() to detect if value is fresh
     * @note Reading channel >= num_channels() returns undefined/last value
     * @note No error indication for invalid channel - caller must validate with num_channels()
     * @note Thread-safe implementations must protect access to shared channel buffer
     * 
     * @warning Do not use stale values after signal loss - check new_input() regularly
     * 
     * @see new_input() to check for fresh data
     * @see num_channels() to validate channel number
     */
    virtual uint16_t read(uint8_t ch) = 0;

    /**
     * @brief Read multiple RC channels into array
     * 
     * @details Efficiently reads multiple channels in single call, copying PWM values
     *          into caller-provided array. Returns actual count of channels copied,
     *          which may be less than requested length if fewer channels available.
     *          
     *          Typical usage:
     *          uint16_t channels[16];
     *          uint8_t count = read(channels, 16);
     *          // count now contains number of valid channels
     *          // channels[0..count-1] contain PWM values
     * 
     * @param[out] periods Array to receive PWM values in microseconds
     * @param[in]  len     Maximum number of channels to read (array size)
     * 
     * @return uint8_t Number of channels actually copied (min of len and num_channels())
     * 
     * @note More efficient than multiple read(ch) calls for reading all channels
     * @note Returns min(len, num_channels()) - won't overflow array
     * @note Values in periods[] are undefined beyond returned count
     * @note Thread-safe implementations must protect access during copy
     * 
     * @see read(uint8_t) for single-channel read
     * @see num_channels() for available channel count
     */
    virtual uint8_t read(uint16_t* periods, uint8_t len) = 0;

    /**
     * @brief Get receiver signal strength indicator (RSSI)
     * 
     * @details Returns receiver-reported signal strength if available from protocol.
     *          Only some protocols provide RSSI data (e.g., CRSF, SRXL2, some SBUS receivers).
     *          
     *          Value interpretation:
     *          - -1: RSSI not available/unknown (default for most protocols)
     *          -  0: No link / signal completely lost
     *          -  1-254: Signal strength (protocol-specific scaling)
     *          -  255: Maximum/perfect signal strength
     *          
     *          RSSI can be used for:
     *          - Range testing
     *          - Link quality monitoring
     *          - Predictive failsafe (trigger before complete loss)
     *          - Telemetry display
     * 
     * @return int16_t Signal strength: -1=unknown, 0=no link, 1-254=strength, 255=maximum
     * 
     * @note Most basic protocols (PPM, DSM, SBUS) return -1 (not supported)
     * @note Modern digital protocols (CRSF, SRXL2) typically provide RSSI
     * @note RSSI scaling is protocol-dependent (not standardized)
     * @note Default implementation returns -1 (unknown)
     * 
     * @see get_rx_link_quality() for alternative quality metric
     */
    virtual int16_t get_rssi(void) { return -1; }
    
    /**
     * @brief Get receiver link quality percentage
     * 
     * @details Returns receiver-reported link quality if available from protocol.
     *          Link quality typically represents packet success rate or similar metric.
     *          Only advanced protocols provide this data (e.g., CRSF/ELRS).
     *          
     *          Value interpretation:
     *          - -1: Link quality not available/unknown (default)
     *          -  0: No link / all packets lost
     *          -  1-99: Link quality percentage
     *          -  100: Perfect link / all packets received
     *          
     *          Link quality more directly indicates reliability than RSSI as it
     *          measures actual communication success rather than signal strength.
     * 
     * @return int16_t Link quality: -1=unknown, 0-100=percentage of successful packets
     * 
     * @note More meaningful than RSSI for evaluating link reliability
     * @note Primarily supported by CRSF/ELRS protocols
     * @note May represent different metrics depending on protocol
     * @note Default implementation returns -1 (unknown)
     * 
     * @see get_rssi() for signal strength metric
     */
    virtual int16_t get_rx_link_quality(void) { return -1; }
    
    /**
     * @brief Get human-readable RC protocol name
     * 
     * @details Returns string identifying the currently detected and active RC protocol.
     *          Useful for debugging, logging, and ground station display.
     *          
     *          Example return values:
     *          - "PPM"
     *          - "SBUS"
     *          - "DSM"
     *          - "DSMX"
     *          - "CRSF"
     *          - "SRXL2"
     *          - "FPort"
     *          - nullptr (no protocol detected yet)
     *          
     *          Protocol may change if receiver switches modes or different
     *          receiver connected.
     * 
     * @return const char* Protocol name string, or nullptr if unknown/not detected
     * 
     * @note Return value points to static string - do not free
     * @note nullptr indicates no protocol detected yet or not implemented
     * @note Protocol name matches those in AP_RCProtocol implementation
     * @note Useful for MAVLink RADIO_STATUS message and logging
     * @note Default implementation returns nullptr
     * 
     * @see AP_RCProtocol for protocol detection and decoding
     */
    virtual const char *protocol() const { return nullptr; }

    /**
     * @brief RC channel override functionality
     * 
     * @note Overrides allow external control (e.g., companion computer) to inject RC values
     * @note Override behavior:
     *       - v == -1: No change to this channel (leave existing override/value)
     *       - v == 0:  Clear override for this channel (use real RC input)
     *       - v > 0:   Set v as override value in microseconds
     * @note Override functionality implementation is platform-specific
     * @note Overrides used for companion computer control, failsafe testing, and automation
     * @warning Historical comment notes this is "grody" design - may be refactored in future
     */

    /**
     * @brief Enable or disable hardware pulse input
     * 
     * @details Allows disabling hardware timer/GPIO pulse capture when RC protocol
     *          is being decoded via UART instead (e.g., SBUS, CRSF). Reduces CPU/interrupt
     *          overhead by stopping unnecessary pulse measurement when not needed.
     *          
     *          Typical usage:
     *          - Disable pulse input when UART protocol detected (SBUS, DSM, CRSF, etc.)
     *          - Enable pulse input when PPM or PWM protocol expected
     *          - Platform may auto-detect and switch as needed
     *          
     *          Reduces system load by:
     *          - Stopping timer input capture interrupts
     *          - Disabling edge detection on GPIO pins
     *          - Freeing CPU cycles for other tasks
     * 
     * @param[in] enable true to enable pulse capture, false to disable
     * 
     * @note Only relevant for platforms supporting both pulse and UART protocols
     * @note Some platforms may ignore this (always enabled or always disabled)
     * @note Default implementation does nothing (no-op)
     * @note Particularly important on low-power platforms to reduce CPU load
     * 
     * @see AP_RCProtocol for protocol auto-detection logic
     */
    virtual void pulse_input_enable(bool enable) { }
};
