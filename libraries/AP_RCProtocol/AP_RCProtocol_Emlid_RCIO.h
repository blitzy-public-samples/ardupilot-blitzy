#pragma once

/**
 * @file AP_RCProtocol_Emlid_RCIO.h
 * @brief Emlid RCIO RC input protocol handler for Navio/Navio2 boards
 * 
 * @details This file implements the RC protocol backend for Emlid's RCIO
 *          (Remote Control Input/Output) hardware interface found on Navio
 *          and Navio2 boards. The RCIO interface provides RC input through
 *          a Linux kernel module that exposes per-channel pulse width data
 *          via the sysfs filesystem.
 * 
 *          Hardware Integration:
 *          - Navio/Navio2 boards: RCIO kernel module handles PWM capture
 *          - Linux sysfs interface: /sys/kernel/rcio/rcin/ch<N> files
 *          - Per-channel text files contain PWM pulse widths in microseconds
 * 
 *          Data Flow:
 *          1. RCIO kernel module captures PWM signals from RC receiver
 *          2. Pulse widths written to sysfs character device files
 *          3. This backend reads text-formatted values via pread()
 *          4. Values converted from text μs to uint16_t PWM values
 *          5. Processed values passed to ArduPilot RC input system
 * 
 * @warning Platform-specific implementation - only available on Emlid boards
 *          with RCIO kernel module loaded. Requires Linux HAL (AP_HAL_Linux).
 * 
 * @note This backend is automatically selected when running on Navio/Navio2
 *       hardware with the appropriate kernel module.
 */

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EMLID_RCIO_ENABLED

#include "AP_RCProtocol_Backend.h"

/**
 * @class AP_RCProtocol_Emlid_RCIO
 * @brief RC protocol backend for Emlid RCIO hardware interface
 * 
 * @details This backend implements RC input handling for Emlid Navio and Navio2
 *          boards by reading PWM pulse width data from the Linux sysfs interface
 *          provided by the RCIO kernel module.
 * 
 *          Sysfs Interface Details:
 *          - Base path: /sys/kernel/rcio/rcin/
 *          - Per-channel files: ch0, ch1, ch2, ... ch15
 *          - File format: Text-encoded microseconds (e.g., "1500\n")
 *          - Reading method: pread() system call for efficient polling
 * 
 *          Channel Configuration:
 *          - Maximum channels: 16 (CHANNEL_COUNT constant)
 *          - Typical usage: 8 channels for standard RC receivers
 *          - Channel numbering: 0-indexed (ch0 = channel 1)
 *          - PWM range: Standard RC range (typically 1000-2000 μs)
 * 
 *          Operational Characteristics:
 *          - Called at RC protocol update rate (typically 50Hz)
 *          - File descriptors opened once during initialization
 *          - Efficient pread() avoids seek operations
 *          - Failed reads skip channels without blocking
 * 
 *          Hardware Dependencies:
 *          - Requires Emlid Navio or Navio2 board
 *          - Requires RCIO kernel module (emlid-rcio)
 *          - Linux HAL only (AP_HAL_Linux)
 *          - Sysfs filesystem must be mounted
 * 
 * @warning This backend is platform-specific and will only function on
 *          Emlid boards with the RCIO kernel module loaded. Do not attempt
 *          to use on other hardware platforms.
 * 
 * @note Channel file descriptors are cached after first open to avoid
 *       repeated filesystem operations during high-frequency polling.
 */
class AP_RCProtocol_Emlid_RCIO : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Poll RCIO sysfs interface for updated RC channel values
     * 
     * @details This method reads PWM pulse width values from the Linux sysfs
     *          interface provided by the RCIO kernel module. It performs the
     *          following operations:
     * 
     *          1. Ensures initialization is complete (calls init() if needed)
     *          2. Iterates through all configured channels (0 to CHANNEL_COUNT-1)
     *          3. Reads text-formatted pulse widths from /sys/kernel/rcio/rcin/ch<N>
     *          4. Converts ASCII text (e.g., "1500\n") to uint16_t PWM values
     *          5. Updates RC input system with new channel values
     * 
     *          File Reading Pattern:
     *          - Uses pread() for efficient reading without seek operations
     *          - Reads up to 16 bytes of text data per channel
     *          - Parses decimal ASCII values followed by newline
     *          - Invalid/failed reads: channel skipped, no error propagated
     * 
     *          Units and Conversion:
     *          - Sysfs files contain: Text microseconds (e.g., "1523")
     *          - Converted to: uint16_t PWM pulse width in μs
     *          - Standard RC range: 1000-2000 μs (nominal)
     *          - Extended range: Some receivers use 800-2200 μs
     * 
     *          Performance Characteristics:
     *          - Called at main RC protocol update rate (typically 50Hz)
     *          - File descriptors cached after first open (channels[] array)
     *          - Failed reads do not block or abort processing
     *          - Timestamp tracking prevents redundant processing
     * 
     * @note This method is called automatically by the RC protocol manager
     *       and should not be invoked directly by user code.
     * 
     * @warning Platform-specific - requires RCIO kernel module on Navio/Navio2.
     *          Will fail silently if sysfs interface is not available.
     * 
     * @see init() for initialization of sysfs file descriptors
     * @see open_channel() for per-channel file descriptor management
     */
    void update() override;

private:
    /**
     * @brief Open sysfs file descriptor for a specific RC channel
     * 
     * @details Opens the sysfs character device file for the specified channel
     *          at /sys/kernel/rcio/rcin/ch<channel>. The file descriptor is
     *          cached in the channels[] array for efficient repeated access.
     * 
     *          File Path Format:
     *          - Pattern: /sys/kernel/rcio/rcin/ch<N>
     *          - Example: /sys/kernel/rcio/rcin/ch0 (for channel 0)
     * 
     *          Open Behavior:
     *          - Opens in read-only mode (O_RDONLY)
     *          - Non-blocking mode may be used
     *          - Failed opens return invalid descriptor (typically -1)
     *          - File descriptor stored in channels[channel] for reuse
     * 
     * @param[in] channel Channel number (0-indexed, range 0 to CHANNEL_COUNT-1)
     * 
     * @return File descriptor for the channel's sysfs file, or -1 on failure
     * 
     * @note File descriptors are opened once and cached. Subsequent calls for
     *       the same channel return the cached descriptor without reopening.
     * 
     * @warning No bounds checking on channel parameter - caller must ensure
     *          channel < CHANNEL_COUNT to avoid buffer overflow.
     */
    int open_channel(int channel);

    /**
     * @brief Initialization complete flag
     * 
     * @details Set to true after init() has been called successfully.
     *          Prevents redundant initialization attempts on subsequent
     *          update() calls.
     */
    bool init_done;

    /**
     * @brief Initialize RCIO sysfs interface
     * 
     * @details Performs one-time initialization of the RCIO backend:
     *          - Initializes channels[] array with invalid descriptors
     *          - May attempt to open initial channel files
     *          - Sets init_done flag to prevent re-initialization
     * 
     *          Initialization is deferred until first update() call rather
     *          than constructor to allow filesystem to be fully mounted.
     * 
     * @note Called automatically by update() on first invocation.
     *       Safe to call multiple times due to init_done guard.
     */
    void init();

    /**
     * @brief Timestamp of last successful update
     * 
     * @details Stores the system timestamp (typically in microseconds) from
     *          the last time RC channel values were successfully read. Used
     *          to detect and handle timing issues, prevent redundant updates,
     *          and track update frequency.
     * 
     * @note Units: microseconds (μs) - system time
     */
    uint32_t _last_timestamp;

    /**
     * @brief Maximum number of RC channels supported
     * 
     * @details Defines the size of the channels[] array and the maximum
     *          number of sysfs channel files that will be opened. Set to 16
     *          to support extended channel receivers, though typical RC
     *          receivers provide 8 channels.
     * 
     * @note Standard RC systems: 4-8 channels
     *       Extended systems: up to 16 channels
     */
    static const size_t CHANNEL_COUNT = 16;

    /**
     * @brief File descriptors for per-channel sysfs files
     * 
     * @details Array of file descriptors corresponding to sysfs channel files.
     *          Each element stores the file descriptor for
     *          /sys/kernel/rcio/rcin/ch<index>.
     * 
     *          Array Contents:
     *          - channels[0]: fd for /sys/kernel/rcio/rcin/ch0
     *          - channels[1]: fd for /sys/kernel/rcio/rcin/ch1
     *          - ...
     *          - channels[15]: fd for /sys/kernel/rcio/rcin/ch15
     * 
     *          Descriptor States:
     *          - Valid descriptor (≥0): Channel file successfully opened
     *          - Invalid descriptor (-1): Channel not opened or open failed
     * 
     * @note File descriptors are opened on-demand by open_channel() and
     *       remain open for the lifetime of the backend to avoid repeated
     *       filesystem overhead.
     */
    int channels[CHANNEL_COUNT];

};

#endif  // AP_RCPROTOCOL_EMLID_RCIO_ENABLED
