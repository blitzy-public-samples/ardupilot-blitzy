/**
 * @file RCInput.h
 * @brief QURT platform RC receiver input driver implementation
 * 
 * @details This file implements the RC (Radio Control) input interface for the
 *          Qualcomm QURT (QuRT Real-Time Operating System) platform running on
 *          Snapdragon Flight and similar DSP-based autopilot hardware.
 *          
 *          The implementation provides channel buffering with mutex-protected access
 *          and periodic polling of RC receiver data. RC input on QURT platform is
 *          polling-based rather than interrupt-driven, with the scheduler calling
 *          _timer_tick() at regular intervals (typically 100Hz) to update buffered
 *          channel values from the AP::RC singleton.
 *          
 *          Architecture: Maintains array of RC_INPUT_MAX_CHANNELS pulse width values
 *          (typically 1000-2000 microseconds) with thread-safe read access via mutex.
 *          
 * @note Platform-specific: On DSP-based systems, RC input hardware is typically on
 *       the applications processor, with values transferred to DSP via shared memory
 *       or RPC (Remote Procedure Call) mechanisms handled by AP::RC.
 * 
 * @see AP_RCProtocol library for RC protocol decoding details (PPM, SBUS, DSM, etc.)
 * @see Scheduler.h for _timer_tick callback registration mechanism
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_HAL_QURT.h"
#include <AP_RCProtocol/AP_RCProtocol.h>

/**
 * @def RC_INPUT_MAX_CHANNELS
 * @brief Maximum number of RC input channels supported by the QURT RC input driver
 * 
 * @details Defines the size of the internal channel pulse buffer array. This limits
 *          the maximum number of RC channels that can be read simultaneously.
 *          
 *          Value: 18 channels (default)
 *          
 *          Rationale: Typical RC systems use 8-16 channels. A value of 18 provides
 *          headroom for advanced RC protocols like SBUS (up to 16 channels) plus
 *          additional channels for future expansion.
 *          
 * @note Most RC transmitters use 8-12 channels. Some advanced protocols support:
 *       - PPM: Typically 8 channels
 *       - SBUS: Up to 16 channels
 *       - DSM/DSM2/DSMX: Up to 12 channels
 *       - CRSF: Up to 16 channels
 * 
 * @note This value can be overridden at compile time by defining RC_INPUT_MAX_CHANNELS
 *       before including this header, though reducing it may cause channel truncation
 *       with high-channel-count receivers.
 */
#ifndef RC_INPUT_MAX_CHANNELS
#define RC_INPUT_MAX_CHANNELS 18
#endif

/**
 * @class QURT::RCInput
 * @brief QURT platform RC receiver input driver with mutex-protected channel buffering
 * 
 * @details Implements the AP_HAL::RCInput interface to provide thread-safe buffering
 *          of RC channel PWM pulse widths (typically 1000-2000 microseconds) from
 *          radio control receivers on Qualcomm QURT/Snapdragon platforms.
 *          
 *          Architecture:
 *          - Maintains array of RC_INPUT_MAX_CHANNELS pulse width values
 *          - Mutex-protected access ensures thread safety between updates and reads
 *          - Polling-based update mechanism via _timer_tick() called by scheduler
 *          - No interrupt-driven RC input on QURT platform
 *          
 *          Update Mechanism:
 *          The _timer_tick() method is called periodically by the HAL scheduler timer
 *          thread (typically at 100Hz). On each call, it polls the AP::RC singleton
 *          for new channel data and copies updated values to the local buffer with
 *          mutex protection.
 *          
 *          Thread Safety:
 *          All channel read operations (read(), num_channels(), new_input()) are
 *          protected by a mutex to prevent race conditions between the scheduler's
 *          _timer_tick() updates and vehicle code reading channel values.
 *          
 *          Typical RC Channel Usage (vehicle-dependent):
 *          - Channel 0 (Throttle): 1000us = min throttle, 2000us = max throttle
 *          - Channel 1 (Roll): 1000-2000us with 1500us center
 *          - Channel 2 (Pitch): 1000-2000us with 1500us center
 *          - Channel 3 (Yaw): 1000-2000us with 1500us center
 *          - Channels 4+: Auxiliary functions (mode switch, camera trigger, etc.)
 *          
 *          Platform-Specific Implementation:
 *          On DSP-based systems (Snapdragon Flight), RC input hardware resides on
 *          the applications processor. Channel values are transferred to the DSP
 *          via shared memory or RPC mechanisms handled transparently by the AP::RC
 *          singleton and lower-level platform drivers.
 * 
 * @note Polling-based design: RC input is not interrupt-driven on QURT. The polling
 *       rate (scheduler frequency) directly affects RC input latency. At 100Hz polling,
 *       worst-case latency is approximately 10ms between RC signal change and
 *       availability to vehicle code.
 * 
 * @warning Polling rate considerations: The _timer_tick() frequency affects RC input
 *          responsiveness. Higher rates (e.g., 400Hz) reduce latency but increase CPU
 *          load. Tune based on vehicle control loop requirements and available CPU budget.
 * 
 * @warning RC Failsafe: If RC signal is lost, channel values freeze at last known state.
 *          Vehicle code must detect RC timeout by monitoring new_input() returning false
 *          over time. Do not rely solely on channel values to detect RC loss.
 * 
 * @warning Mutex overhead: Each channel read acquires the mutex (overhead ~10us). For
 *          efficiency when reading multiple channels, prefer read(buf, count) over
 *          multiple read(chan) calls to amortize mutex acquisition cost.
 * 
 * @note Actual channel mapping varies by vehicle type (Copter, Plane, Rover) and
 *       RC transmitter configuration. Above channel assignments are conventional
 *       defaults for ArduPilot.
 * 
 * @see AP_RCProtocol for RC protocol decoding (PPM, SBUS, DSM, CRSF, etc.)
 * @see AP_HAL::RCInput for interface contract documentation
 * @see Scheduler.h for timer callback registration mechanism
 */
class QURT::RCInput : public AP_HAL::RCInput
{
public:
    /**
     * @brief Initialize the RC input driver
     * 
     * @details Performs any necessary initialization of the RC input subsystem.
     *          On QURT platform, this typically involves minimal setup as the
     *          actual RC protocol decoding is handled by AP::RC singleton.
     *          
     * @note Called during HAL initialization sequence before the main vehicle
     *       code starts executing. Should not be called by vehicle code.
     * 
     * @note Overrides AP_HAL::RCInput::init()
     */
    void init() override;
    
    /**
     * @brief Check if new RC input data is available since last read
     * 
     * @details Returns true if RC channel data has been updated since the last
     *          call to this method. Used by vehicle code to detect fresh RC input
     *          and for RC failsafe detection.
     *          
     *          The updated flag is set by _timer_tick() when new data arrives from
     *          AP::RC and is cleared after this method returns true.
     * 
     * @return true if RC data has been updated since last check
     * @return false if no new RC data (may indicate RC signal loss if prolonged)
     * 
     * @note Thread-safe: Protected by mutex
     * 
     * @warning Critical for RC failsafe: If this returns false continuously for
     *          the configured failsafe timeout period (typically 1-3 seconds),
     *          vehicle code should trigger RC failsafe procedures.
     * 
     * @note Overrides AP_HAL::RCInput::new_input()
     */
    bool new_input() override;
    
    /**
     * @brief Get the number of active RC input channels
     * 
     * @details Returns the count of RC channels currently being received. This
     *          value is auto-detected based on the RC receiver protocol and may
     *          change if a different receiver is connected.
     *          
     *          Typical values:
     *          - 8 channels: Common for basic RC systems (PPM)
     *          - 12 channels: DSM/DSMX systems
     *          - 16 channels: SBUS, CRSF systems
     * 
     * @return Number of active RC channels (0 to RC_INPUT_MAX_CHANNELS)
     * 
     * @note Thread-safe: Protected by mutex
     * 
     * @note Returns 0 if no RC receiver connected or RC signal lost
     * 
     * @note Overrides AP_HAL::RCInput::num_channels()
     */
    uint8_t num_channels() override;
    
    /**
     * @brief Read pulse width for a single RC channel
     * 
     * @details Returns the most recent PWM pulse width for the specified channel.
     *          Pulse widths are typically in the range 1000-2000 microseconds:
     *          - 1000us: Minimum value (stick full down/left)
     *          - 1500us: Center value (stick centered)
     *          - 2000us: Maximum value (stick full up/right)
     * 
     * @param[in] ch Channel number (0-based, 0 = first channel)
     * 
     * @return Pulse width in microseconds (typically 1000-2000us)
     * @return Last known value if channel is inactive or out of range
     * 
     * @note Thread-safe: Protected by mutex
     * 
     * @note Channel numbering: 0-based (channel 0 is typically throttle)
     * 
     * @note If ch >= num_channels(), returns last buffered value which may be
     *       stale or zero if channel was never active.
     * 
     * @warning Returns last known value on RC signal loss - use new_input() to
     *          detect RC connection status
     * 
     * @note For reading multiple channels, prefer read(buf, count) variant for
     *       better efficiency (single mutex acquisition)
     * 
     * @note Overrides AP_HAL::RCInput::read(uint8_t)
     */
    uint16_t read(uint8_t ch) override;
    
    /**
     * @brief Read pulse widths for multiple RC channels into a buffer
     * 
     * @details Atomically reads multiple RC channel pulse widths into the provided
     *          buffer. More efficient than multiple read(ch) calls as the mutex is
     *          acquired only once for the entire read operation.
     * 
     * @param[out] periods Buffer to receive pulse width values (microseconds)
     * @param[in]  len     Number of channels to read (size of periods buffer)
     * 
     * @return Number of channels actually read (min of len and num_channels())
     * 
     * @note Thread-safe: Protected by mutex (single acquisition for all channels)
     * 
     * @note Preferred method for reading multiple channels - approximately 10us
     *       overhead per read(ch) call avoided by using this batch read method
     * 
     * @note If len exceeds num_channels(), only num_channels() values are written
     *       and the return value indicates actual count
     * 
     * @note Buffer must be pre-allocated with space for at least len uint16_t values
     * 
     * @warning No bounds checking on buffer size - caller must ensure periods buffer
     *          has space for len elements to avoid buffer overflow
     * 
     * @note Overrides AP_HAL::RCInput::read(uint16_t*, uint8_t)
     */
    uint8_t read(uint16_t* periods, uint8_t len) override;

    /**
     * @brief Periodic timer callback to poll RC input from AP::RC singleton
     * 
     * @details Called by the HAL scheduler timer thread at regular intervals (typically
     *          100Hz) to poll for updated RC channel data. This is the primary update
     *          mechanism for RC input on QURT platform.
     *          
     *          Operation sequence:
     *          1. Polls AP::RC singleton for new RC data
     *          2. If new data available, acquires mutex
     *          3. Copies channel values to local buffer
     *          4. Updates num_channels() and sets updated flag
     *          5. Releases mutex
     *          
     *          The AP::RC singleton handles platform-specific RC protocol decoding
     *          (PPM, SBUS, DSM, CRSF, etc.) and provides a unified interface to
     *          retrieve channel data regardless of underlying protocol.
     * 
     * @note Called by scheduler timer thread, not by vehicle code
     * 
     * @note Polling frequency: Typically 100Hz (10ms interval). This directly affects
     *       RC input latency - worst case latency is one polling interval.
     * 
     * @warning Must execute quickly: This runs in the scheduler timer thread context.
     *          Execution time should be <1ms to avoid delaying other timer callbacks.
     *          Heavy processing would affect scheduler timing accuracy.
     * 
     * @note Platform-specific: On DSP platforms, AP::RC abstracts the communication
     *       with the applications processor where RC hardware is physically located.
     *       This may involve shared memory access or RPC calls that are transparent
     *       to this driver.
     * 
     * @note Thread safety: Uses mutex to protect concurrent access to channel buffer
     *       from vehicle code read operations
     * 
     * @see AP_RCProtocol for details on RC protocol decoding
     * @see Scheduler.h for timer callback registration
     */
    void _timer_tick(void);

private:
    /**
     * @brief Mutex for thread-safe access to RC channel data
     * 
     * @details Protects concurrent access to values[], num_chan, and updated flag
     *          between _timer_tick() updates (scheduler thread) and vehicle code
     *          reads (main thread or other threads).
     *          
     * @note Acquired by: _timer_tick() during updates, read() methods during access
     * 
     * @note Typical hold time: <10 microseconds for channel reads
     */
    HAL_Semaphore mutex;
    
    /**
     * @brief Buffer storing RC channel pulse widths in microseconds
     * 
     * @details Array of RC_INPUT_MAX_CHANNELS elements storing the most recent
     *          pulse width value for each channel. Values typically range from
     *          1000-2000 microseconds.
     *          
     * @note Updated by: _timer_tick() with mutex held
     * @note Read by: read() methods with mutex held
     * 
     * @note Array size: RC_INPUT_MAX_CHANNELS (default 18)
     */
    uint16_t values[RC_INPUT_MAX_CHANNELS];
    
    /**
     * @brief Number of active RC channels currently being received
     * 
     * @details Auto-detected based on RC receiver protocol. Updated when RC
     *          protocol decoder detects channel count change.
     *          
     * @note Range: 0 to RC_INPUT_MAX_CHANNELS
     * @note Value of 0 indicates no RC receiver or signal loss
     */
    uint8_t num_chan;
    
    /**
     * @brief Flag indicating new RC data available since last new_input() check
     * 
     * @details Set to true by _timer_tick() when fresh RC data arrives.
     *          Cleared when new_input() is called and returns true.
     *          
     * @note Used for: RC failsafe detection, detecting fresh pilot input
     * 
     * @warning If remains false for extended period (>1-3 seconds), indicates
     *          RC signal loss and should trigger failsafe procedures
     */
    bool updated;
};
