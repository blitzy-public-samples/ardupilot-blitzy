/**
 * @file AP_RCProtocol_IOMCU.h
 * @brief RC protocol backend for boards with dedicated IO microcontroller (IOMCU)
 * 
 * @details This protocol backend provides RC input integration for Pixhawk-family
 *          flight controllers that feature a dedicated IO coprocessor. The IO MCU
 *          is a separate STM32F100 microcontroller that handles RC protocol decoding,
 *          offloading this real-time task from the main flight processor.
 * 
 *          Architecture:
 *          - IO MCU (STM32F100): Dedicated coprocessor handles RC signal decoding
 *          - Main CPU: Polls decoded RC channels from IOMCU via AP_IOMCU singleton
 *          - Protocol Detection: IOMCU performs automatic protocol detection
 *          - Channel Data: Pre-decoded PWM values transferred to main CPU
 * 
 *          Supported Hardware:
 *          - Pixhawk 1 (FMUv2)
 *          - Pixhawk 2 (Cube)
 *          - Pixhawk 4 (FMUv5)
 *          - Other boards with separate IO coprocessor
 * 
 *          Data Flow:
 *          1. RC receiver → IOMCU pins (SBUS/PPM/DSM/etc.)
 *          2. IOMCU decodes protocol → stores channels
 *          3. Main CPU queries AP_IOMCU::read_rc_input()
 *          4. This backend receives decoded channels
 * 
 * @warning This backend is only available on boards compiled with HAL_WITH_IO_MCU.
 *          It will not function on boards without a dedicated IO coprocessor.
 * 
 * @note The IOMCU handles all protocol-specific decoding (SBUS inversion, DSM binding,
 *       PPM timing, etc.). The main CPU only receives standardized channel values.
 * 
 * @see AP_IOMCU for the IOMCU communication interface
 * @see AP_RCProtocol_Backend for the base protocol interface
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_IOMCU_ENABLED

#include "AP_RCProtocol_Backend.h"

/**
 * @class AP_RCProtocol_IOMCU
 * @brief RC protocol backend that receives RC input from a dedicated IO coprocessor
 * 
 * @details This backend integrates with the IO microcontroller (IOMCU) subsystem
 *          present on Pixhawk-family flight controllers. Unlike other RC protocol
 *          backends that directly decode RC signals, this backend queries the
 *          AP_IOMCU singleton to retrieve channels that have already been decoded
 *          by the separate IO processor.
 * 
 *          The IOMCU architecture provides several advantages:
 *          - Offloads real-time RC decoding from main flight processor
 *          - Provides hardware isolation for RC input (safety feature)
 *          - Handles protocol-specific timing requirements in dedicated hardware
 *          - Supports protocol binding (DSM) via dedicated IO processor control
 * 
 *          Protocol Abstraction:
 *          The IOMCU handles automatic detection and decoding of multiple RC protocols
 *          (SBUS, DSM, PPM, etc.). This backend receives normalized channel values
 *          in standard PWM microseconds (1000-2000μs range), abstracting away the
 *          underlying protocol complexity.
 * 
 *          Failsafe Handling:
 *          The backend monitors IOMCU communication status. Loss of IOMCU updates
 *          triggers failsafe via the active() method returning false after 400ms timeout.
 * 
 * @note Only instantiated on boards with HAL_WITH_IO_MCU defined
 * @warning Requires functional IOMCU firmware and communication link
 */
class AP_RCProtocol_IOMCU : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Poll the IOMCU for new RC input data
     * 
     * @details Queries the AP_IOMCU singleton to retrieve decoded RC channel data
     *          from the IO coprocessor. The IOMCU performs all protocol-specific
     *          decoding (SBUS, DSM, PPM, etc.) and this method retrieves the
     *          pre-decoded channel values.
     * 
     *          Data Flow:
     *          1. Query AP_IOMCU::read_rc_input() for latest channels
     *          2. If new data available, store timestamp in last_iomcu_us
     *          3. Mark ever_seen_input = true on first valid reception
     *          4. Pass decoded channels to AP_RCProtocol frontend
     * 
     *          The method updates internal state to track communication health
     *          with the IOMCU. Loss of updates for >400ms triggers failsafe via
     *          active() returning false.
     * 
     * @note Called at main loop rate (typically 400Hz for Copter, 50-400Hz for other vehicles)
     * @note Channel values received in microseconds (1000-2000μs PWM range)
     * 
     * @see AP_IOMCU::read_rc_input() for IOMCU interface details
     * @see active() for communication health monitoring
     */
    void update() override;

    /**
     * @brief Initiate DSM/DSMX binding mode via IOMCU
     * 
     * @details Commands the IO coprocessor to enter DSM receiver binding mode.
     *          The IOMCU will pulse the RC input line in a protocol-specific
     *          pattern to put DSM/DSMX receivers into bind mode.
     * 
     *          Binding Process:
     *          1. Power off receiver
     *          2. Call this method to start bind pulse sequence
     *          3. Power on receiver while pulses active
     *          4. Receiver enters bind mode and pairs with transmitter
     * 
     *          The actual bind pulse generation is handled entirely by the
     *          IOMCU firmware. This method simply forwards the bind command
     *          via AP_IOMCU::start_bind().
     * 
     * @note Only applicable for DSM/DSMX protocol receivers
     * @note Requires physical receiver power cycle during bind process
     * @warning Binding requires specific timing - follow manufacturer procedures
     * 
     * @see AP_IOMCU::start_bind() for IOMCU binding interface
     */
    void start_bind() override;

    /**
     * @brief Check if IOMCU RC input is currently active and healthy
     * 
     * @details Determines if RC input from the IOMCU is available and recent.
     *          Returns true only if RC input has been seen at least once AND
     *          updates from the IOMCU are current (within last 400ms).
     * 
     *          Failsafe Detection:
     *          This method provides failsafe detection for IOMCU communication.
     *          If the main CPU stops receiving updates from the IO coprocessor
     *          for more than 400ms, this returns false, triggering RC failsafe.
     * 
     *          The 400ms timeout allows for:
     *          - Normal jitter in IOMCU update rate
     *          - Brief communication delays
     *          - Detection of IOMCU firmware hang or hardware failure
     * 
     * @return true if IOMCU has sent at least one RC update AND
     *              updates received within last 400ms (400000μs)
     * @return false if no RC ever received OR IOMCU communication timed out
     * 
     * @note Timeout value: 400ms = 400000 microseconds
     * @note Called by RC protocol frontend to determine active protocol
     * @warning Loss of IOMCU communication triggers RC failsafe
     * 
     * @see ever_seen_input for first-reception tracking
     * @see last_iomcu_us for timestamp of last update
     */
    bool active() const {
        return ever_seen_input && AP_HAL::micros() - last_iomcu_us < 400000;
    }

    /**
     * @brief Get the name of the RC protocol decoded by IOMCU
     * 
     * @details Queries the IOMCU for the detected RC protocol name (e.g., "SBUS",
     *          "DSM", "PPM"). The protocol detection is performed by the IO
     *          coprocessor firmware, which automatically identifies the incoming
     *          signal type.
     * 
     *          This method forwards the query to AP_IOMCU::get_rc_protocol()
     *          to retrieve the protocol name string from the IOMCU.
     * 
     * @return Pointer to null-terminated string with protocol name
     *         Examples: "SBUS", "DSM2", "DSMX", "PPM", "SUMD"
     * @return nullptr if IOMCU not available or protocol unknown
     * 
     * @note Protocol name reflects what IOMCU detected, not this backend name
     * @note Used for telemetry reporting and ground station display
     * 
     * @see AP_IOMCU::get_rc_protocol() for protocol detection details
     */
    const char *get_rc_protocol() const;

private:

    /**
     * @brief Timestamp of last successful IOMCU RC input reception
     * 
     * @details Stores the system microsecond timestamp (from AP_HAL::micros())
     *          when RC channel data was last successfully received from the IOMCU.
     *          Updated by update() method on each successful read from AP_IOMCU.
     * 
     *          Used by active() method to detect IOMCU communication timeout.
     *          If current time exceeds this timestamp by >400ms, RC input is
     *          considered lost and failsafe is triggered.
     * 
     * @note Units: microseconds (μs) from system boot
     * @note Wraps at 2^32 microseconds (~71.6 minutes), but difference calculation
     *       in active() handles wrap correctly for timeout detection
     * 
     * @see active() for timeout detection logic
     * @see update() for timestamp update on each IOMCU read
     */
    uint32_t last_iomcu_us;

    /**
     * @brief Flag indicating if any RC input has ever been received from IOMCU
     * 
     * @details Tracks whether at least one valid RC channel update has been
     *          received from the IO coprocessor since boot. Set to true by
     *          update() method on first successful RC input reception.
     * 
     *          Used by active() to distinguish between:
     *          - No RC receiver connected (ever_seen_input = false)
     *          - RC receiver connected but signal lost (ever_seen_input = true,
     *            but last_iomcu_us too old)
     * 
     *          This prevents false "RC active" indication before any RC equipment
     *          has been detected and provides proper startup behavior.
     * 
     * @note Remains true once set - never reset to false during runtime
     * @note Initialized to false on object construction
     * 
     * @see active() for RC availability determination
     * @see update() for first-reception detection
     */
    bool ever_seen_input;

};


#endif  // AP_RCPROTOCOL_IOMCU_ENABLED
