/*
 * Copyright (C) 2020 Siddharth B Purohit
 *
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
 */

/**
 * @file CANIface.h
 * @brief Controller Area Network (CAN) bus interface for vehicle peripherals
 * 
 * @details Defines the HAL (Hardware Abstraction Layer) interface for CAN bus communication
 *          supporting both standard CAN 2.0 and CAN-FD (Flexible Data-rate) protocols.
 *          Used for DroneCAN/UAVCAN peripherals, ESCs (Electronic Speed Controllers),
 *          sensors, and other CAN-connected devices.
 *          
 *          Platform-specific implementations handle hardware CAN controllers (e.g.,
 *          STM32 bxCAN, STM32 FDCAN, Linux SocketCAN, ESP32 TWAI).
 *          
 *          Key features:
 *          - Standard CAN 2.0A/B support (11-bit and 29-bit identifiers)
 *          - CAN-FD support with up to 64 bytes per frame (when HAL_CANFD_SUPPORTED)
 *          - Hardware filtering for efficient frame processing
 *          - Multiple operating modes (Normal, Silent, PassThrough, Filtered)
 *          - Non-blocking send/receive operations
 *          - Bus statistics and error monitoring
 *          
 * @note This is a pure virtual interface; actual implementations are in platform HAL directories
 * @see AP_HAL_ChibiOS for STM32 implementations
 * @see AP_HAL_Linux for SocketCAN implementations
 */

#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"

class ExpandingString;

/**
 * @struct AP_HAL::CANFrame
 * @brief Raw CAN frame representation for transmission and reception
 * 
 * @details Encapsulates a single CAN or CAN-FD frame with ID, data payload, and control flags.
 *          Supports both standard (11-bit) and extended (29-bit) CAN identifiers.
 *          Maximum data length is 8 bytes for classic CAN 2.0, up to 64 bytes for CAN-FD.
 *          
 *          Frame types:
 *          - Data frames: Normal message transmission with data payload
 *          - RTR frames: Remote Transmission Request (no data, requests data from another node)
 *          - Error frames: Bus error indication (passive/active error signaling)
 *          
 *          Frame formats:
 *          - Standard format (CAN 2.0A): 11-bit identifier (0x000-0x7FF)
 *          - Extended format (CAN 2.0B): 29-bit identifier (0x00000000-0x1FFFFFFF)
 *          
 *          The 'id' field packs the CAN identifier in the low bits with flag bits in the high bits:
 *          - Bit 31 (FlagEFF): Extended Frame Format indicator
 *          - Bit 30 (FlagRTR): Remote Transmission Request indicator
 *          - Bit 29 (FlagERR): Error Frame indicator
 *          - Bits 0-28: CAN identifier (bits 0-10 for standard, 0-28 for extended)
 *          
 * @note DLC (Data Length Code) may not equal actual data length in CAN-FD due to encoding
 * @note Use dlcToDataLength() to convert DLC to actual byte count for CAN-FD frames
 * @warning Accessing data[] beyond dlc results in undefined behavior
 * @warning Frame priority is determined by ID - lower numeric ID has higher priority
 */
struct AP_HAL::CANFrame {
    static const uint32_t MaskStdID = 0x000007FFU;
    static const uint32_t MaskExtID = 0x1FFFFFFFU;
    static const uint32_t FlagEFF = 1U << 31;                  ///< Extended frame format
    static const uint32_t FlagRTR = 1U << 30;                  ///< Remote transmission request
    static const uint32_t FlagERR = 1U << 29;                  ///< Error frame

#if HAL_CANFD_SUPPORTED
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 64;
#else
    static const uint8_t NonFDCANMaxDataLen = 8;
    static const uint8_t MaxDataLen = 8;
#endif
    uint32_t id;                ///< CAN ID with flags (above)
    union {
        uint8_t data[MaxDataLen];
        uint32_t data_32[MaxDataLen/4];
    };
    uint8_t dlc;                ///< Data Length Code
    bool canfd;

    /**
     * @brief Default constructor - initializes empty CAN frame
     * 
     * @details Creates a zero-initialized frame with ID 0, no data, standard format
     */
    CANFrame() :
        id(0),
        dlc(0),
        canfd(false)
    {
        memset(data,0, MaxDataLen);
    }

    /**
     * @brief Construct CAN frame with specified ID and data
     * 
     * @param[in] can_id CAN identifier with optional flag bits (FlagEFF, FlagRTR, FlagERR)
     * @param[in] can_data Pointer to data payload (copied into frame)
     * @param[in] data_len Length of data in bytes (0-8 for CAN, 0-64 for CAN-FD)
     * @param[in] canfd_frame true for CAN-FD frame, false for classic CAN frame
     * 
     * @note data_len is automatically limited to MaxDataLen for the frame type
     */
    CANFrame(uint32_t can_id, const uint8_t* can_data, uint8_t data_len, bool canfd_frame = false);

    bool operator!=(const CANFrame& rhs) const
    {
        return !operator==(rhs);
    }
    bool operator==(const CANFrame& rhs) const
    {
        return (id == rhs.id) && (dlc == rhs.dlc) && (memcmp(data, rhs.data, dlc) == 0);
    }

    /**
     * @brief Get CAN ID as signed integer for scripting interfaces
     * 
     * @details Returns the CAN identifier without flag bits, cast to signed int.
     *          Useful for Lua scripting where uint32_t operations are expensive.
     * 
     * @return Signed CAN ID (11-bit for standard, 29-bit for extended)
     */
    int32_t id_signed(void) const {
        return isExtended()? int32_t(id & MaskExtID) : int32_t(id & MaskStdID);
    }

    /**
     * @brief Check if frame uses extended (29-bit) identifier format
     * 
     * @return true if extended format (CAN 2.0B), false if standard format (CAN 2.0A)
     */
    bool isExtended()                  const
    {
        return id & FlagEFF;
    }

    /**
     * @brief Check if frame is a Remote Transmission Request
     * 
     * @details RTR frames request data from another node and carry no data payload
     * 
     * @return true if RTR frame, false if data frame
     */
    bool isRemoteTransmissionRequest() const
    {
        return id & FlagRTR;
    }

    /**
     * @brief Check if frame is an error frame
     * 
     * @details Error frames indicate bus errors detected by CAN controller
     * 
     * @return true if error frame, false if normal frame
     */
    bool isErrorFrame()                const
    {
        return id & FlagERR;
    }

    /**
     * @brief Set CAN-FD frame indicator
     * 
     * @param[in] canfd_frame true for CAN-FD frame, false for classic CAN frame
     */
    void setCanFD(bool canfd_frame)
    {
        canfd = canfd_frame;
    }

    /**
     * @brief Check if frame is CAN-FD format
     * 
     * @return true if CAN-FD frame, false if classic CAN 2.0 frame
     */
    bool isCanFDFrame() const
    {
        return canfd;
    }

    /**
     * @brief Convert DLC (Data Length Code) to actual data length in bytes
     * 
     * @details For CAN-FD, DLC values above 8 encode lengths: 9->12, 10->16, 11->20, 12->24, 13->32, 14->48, 15->64
     *          For classic CAN, DLC directly represents data length (0-8)
     * 
     * @param[in] dlc Data Length Code from CAN frame (0-15)
     * @return Actual data length in bytes
     */
    static uint8_t dlcToDataLength(uint8_t dlc);

    /**
     * @brief Convert data length in bytes to DLC (Data Length Code)
     * 
     * @details Inverse of dlcToDataLength() - encodes byte length as DLC value
     * 
     * @param[in] data_length Data length in bytes (0-64 for CAN-FD, 0-8 for classic CAN)
     * @return DLC value (0-15)
     */
    static uint8_t dataLengthToDlc(uint8_t data_length);
    /**
     * @brief Compare frame priority for CAN bus arbitration
     * 
     * @details CAN arbitration rules determine which frame wins bus access when multiple
     *          nodes transmit simultaneously. Lower numerical ID has higher priority.
     *          Standard frames (11-bit) have higher priority than extended frames (29-bit)
     *          with the same base ID bits due to the EFF bit position.
     *          
     *          Reference: Marco Di Natale - "Understanding and using the Controller Area Network"
     *          http://www6.in.tum.de/pub/Main/TeachingWs2013MSE/CANbus.pdf
     * 
     * @param[in] rhs Frame to compare against
     * @return true if this frame has higher priority (wins arbitration), false otherwise
     * 
     * @note RTR bit also affects arbitration - data frames beat RTR frames with same ID
     */
    bool priorityHigherThan(const CANFrame& rhs) const;

    /**
     * @brief Compare frame priority (inverse of priorityHigherThan)
     * 
     * @param[in] rhs Frame to compare against
     * @return true if this frame has lower priority (loses arbitration), false otherwise
     */
    bool priorityLowerThan(const CANFrame& rhs) const
    {
        return rhs.priorityHigherThan(*this);
    }
};

/**
 * @class AP_HAL::CANIface
 * @brief Abstract CAN bus interface for platform-independent CAN communication
 * 
 * @details Provides send/receive operations, filtering, and bus management for CAN peripherals.
 *          Platform-specific implementations handle hardware CAN controllers (STM32 bxCAN, FDCAN,
 *          Linux SocketCAN, ESP32 TWAI, etc.).
 *          
 *          Operating modes:
 *          - NormalMode: Standard transmit/receive with ACK - normal CAN bus operation
 *          - SilentMode: Receive-only, no ACK transmission - bus monitoring without affecting bus
 *          - PassThroughMode: Forward frames between CAN interfaces - routing/gateway functionality
 *          - FilteredMode: Hardware filtering of unwanted frames - reduces CPU load
 *          
 *          Thread safety: Methods may be called from multiple scheduler threads concurrently.
 *          Implementations must provide appropriate locking for TX/RX queues and hardware access.
 *          
 *          Typical initialization sequence:
 *          1. init() - Configure bitrate and operating mode
 *          2. configureFilters() - Set up hardware filtering (optional)
 *          3. send()/receive() - Exchange frames
 *          4. get_statistics() - Monitor bus health
 *          
 *          Error handling:
 *          - Bus-off state: Occurs after excessive error frames (TEC > 255)
 *          - Requires recovery: Some implementations need manual recovery from bus-off
 *          - Error counters: Monitor via getErrorCount() and get_statistics()
 * 
 * @note CAN bus timing is critical - missed TX deadlines can cause protocol failures
 * @note DroneCAN/UAVCAN typically uses 1Mbps bitrate with 125μs transfer intervals
 * @warning Bus-off state requires manual recovery in some hardware implementations
 * @warning Changing filters while receiving may drop frames during reconfiguration
 */
class AP_HAL::CANIface
{
public:

    /**
     * @enum OperatingMode
     * @brief CAN interface operating modes
     */
    enum OperatingMode {
        PassThroughMode,  ///< Forward frames between CAN interfaces (routing mode)
        NormalMode,       ///< Standard transmit and receive with bus participation
        SilentMode,       ///< Receive-only monitoring without ACK (listen-only)
        FilteredMode      ///< Hardware filtering enabled for selective frame reception
    };

    /**
     * @brief Get current operating mode
     * 
     * @return Current operating mode (Normal, Silent, PassThrough, or Filtered)
     */
    OperatingMode get_operating_mode() { return mode_; }

    /**
     * @typedef CanIOFlags
     * @brief Bitfield flags for CAN frame transmission and reception control
     */
    typedef uint16_t CanIOFlags;
    static const CanIOFlags Loopback = 1;           ///< Enable loopback mode (frame echoed to RX)
    static const CanIOFlags AbortOnError = 2;       ///< Abort transmission on bus error
    static const CanIOFlags IsForwardedFrame = 4;   ///< Frame is forwarded from another interface

    /**
     * @struct CanRxItem
     * @brief Received CAN frame with metadata
     * 
     * @details Contains a received CAN frame along with reception timestamp and flags.
     *          Used in RX queues and receive callbacks.
     */
    struct CanRxItem {
        uint64_t timestamp_us = 0;  ///< Reception timestamp in microseconds (monotonic clock)
        CanIOFlags flags = 0;       ///< Reception flags (Loopback, IsForwardedFrame, etc.)
        CANFrame frame;             ///< The received CAN frame data
    };

    /**
     * @struct CanTxItem
     * @brief Transmit CAN frame with scheduling and control metadata
     * 
     * @details Contains a frame to transmit along with deadline, priority, and control flags.
     *          Used in TX queues for prioritized frame transmission. Frames are sorted by
     *          CAN arbitration priority, with deadline as secondary sort key via index.
     */
    struct CanTxItem {
        uint64_t deadline = 0;      ///< Transmission deadline in microseconds (monotonic clock)
        CANFrame frame;             ///< The CAN frame to transmit
        uint32_t index = 0;         ///< Transmission sequence index for FIFO ordering within same priority
        bool loopback:1;            ///< Enable loopback (echo frame to RX queue)
        bool abort_on_error:1;      ///< Abort transmission on bus error
        bool aborted:1;             ///< Frame transmission was aborted
        bool pushed:1;              ///< Frame has been pushed to hardware TX queue
        bool setup:1;               ///< Frame setup is complete
        bool canfd_frame:1;         ///< Frame is CAN-FD format

        /**
         * @brief Compare transmission items for priority queue ordering
         * 
         * @details Priority queue uses CAN arbitration rules (lower ID = higher priority).
         *          Within same priority, higher index goes last (FIFO ordering).
         * 
         * @param[in] rhs Item to compare against
         * @return true if this item should be transmitted after rhs
         */
        bool operator<(const CanTxItem& rhs) const
        {
            if (frame.priorityLowerThan(rhs.frame)) {
                return true;
            }
            if (frame.priorityHigherThan(rhs.frame)) {
                return false;
            }
            return index > rhs.index;
        }
    };

    /**
     * @struct CanFilterConfig
     * @brief Hardware CAN acceptance filter configuration
     * 
     * @details Defines an ID/mask pair for hardware filtering. Only frames matching
     *          the filter are received, reducing CPU load from unwanted frames.
     *          
     *          Filter matching: (received_id & mask) == (id & mask)
     *          
     *          Examples:
     *          - Accept only ID 0x123: id=0x123, mask=0x7FF (standard) or 0x1FFFFFFF (extended)
     *          - Accept ID range 0x100-0x1FF: id=0x100, mask=0x700
     *          - Accept all: id=0x000, mask=0x000
     */
    struct CanFilterConfig {
        uint32_t id = 0;    ///< CAN identifier to match (with flags: FlagEFF for extended format)
        uint32_t mask = 0;  ///< Bit mask for matching (1=must match, 0=don't care)

        /**
         * @brief Compare filter configurations for equality
         * 
         * @param[in] rhs Filter configuration to compare
         * @return true if both ID and mask are equal
         */
        bool operator==(const CanFilterConfig& rhs) const
        {
            return rhs.id == id && rhs.mask == mask;
        }
    };

    /**
     * @brief Initialize CAN interface with CAN-FD bitrates
     * 
     * @details Initializes the CAN interface with separate bitrates for arbitration phase
     *          and data phase (CAN-FD). Falls back to standard init() if CAN-FD not supported.
     * 
     * @param[in] bitrate Arbitration phase bitrate in bits/second (typical: 1000000 for 1Mbps)
     * @param[in] fdbitrate Data phase bitrate in bits/second for CAN-FD (e.g., 4000000 for 4Mbps)
     * @param[in] mode Operating mode (Normal, Silent, PassThrough, Filtered)
     * 
     * @return true if initialization successful, false on hardware error or unsupported configuration
     * 
     * @note Default implementation ignores fdbitrate and calls standard init()
     */
    virtual bool init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode) {
        return init(bitrate, mode);
    }

    /**
     * @brief Initialize CAN interface with specified bitrate and operating mode
     * 
     * @details Configures the CAN controller hardware, sets timing parameters for the
     *          specified bitrate, enables the interface, and prepares TX/RX queues.
     *          
     *          Common bitrates:
     *          - 125000 (125 kbps) - Basic CAN applications
     *          - 250000 (250 kbps) - Some industrial protocols
     *          - 500000 (500 kbps) - Automotive applications
     *          - 1000000 (1 Mbps) - DroneCAN/UAVCAN standard
     *          
     *          Must be called before any send/receive operations. Calling init() again
     *          reinitializes the interface, resetting all queues and error counters.
     * 
     * @param[in] bitrate CAN bus bitrate in bits/second (typical: 1000000 for DroneCAN)
     * @param[in] mode Operating mode (Normal, Silent, PassThrough, Filtered)
     * 
     * @return true if initialization successful and interface ready, false on hardware error
     * 
     * @note Bitrate must be supported by hardware and match other nodes on the bus
     * @note Reinitializing resets TX/RX queues and clears error counters
     * @warning Changing bitrate while bus is active may cause bus errors on other nodes
     * @warning Incorrect bitrate configuration prevents communication and may cause bus-off
     */
    virtual bool init(const uint32_t bitrate, const OperatingMode mode) = 0;

    /**
     * @brief Wait for CAN interface RX/TX readiness with timeout
     * 
     * @details Blocks until RX buffer has data, TX buffer has space, or timeout expires.
     *          Used for efficient event-driven CAN processing without polling.
     *          
     *          On entry:
     *          - read_select: true to wait for RX data availability
     *          - write_select: true to wait for TX buffer space
     *          
     *          On return:
     *          - read_select: true if RX buffer has data available
     *          - write_select: true if TX buffer has space available
     * 
     * @param[in,out] read_select In: wait for RX, Out: RX data available
     * @param[in,out] write_select In: wait for TX space, Out: TX space available
     * @param[in] pending_tx Frame that will be sent (for priority checking), may be nullptr
     * @param[in] timeout Maximum wait time in microseconds (0 = no wait, check status only)
     * 
     * @return true if RX/TX event occurred, false if timeout expired
     * 
     * @note Default implementation returns false (not supported)
     * @note Some implementations may not support select and always return false
     */
    virtual bool select(bool &read_select, bool &write_select,
                        const CANFrame* const pending_tx, uint64_t timeout)
    {
        return false;
    }

    /**
     * @brief Register binary semaphore for event notification
     * 
     * @details Allows asynchronous notification of RX/TX events via semaphore.
     *          CAN driver signals semaphore when frames are received or TX completes.
     * 
     * @param[in] sem_handle Pointer to binary semaphore for event signaling
     * 
     * @return true if event handle registered successfully, false if not supported
     * 
     * @note Default implementation returns true (no-op, always succeeds)
     */
    virtual bool set_event_handle(AP_HAL::BinarySemaphore *sem_handle)
    {
        return true;
    }

    /**
     * @brief Queue CAN frame for transmission
     * 
     * @details Non-blocking call to enqueue a frame for transmission. Frame is queued
     *          according to CAN arbitration priority (lower ID = higher priority).
     *          Transmission occurs asynchronously when bus is available.
     *          
     *          The tx_deadline parameter specifies when the frame must be transmitted.
     *          If the deadline cannot be met, behavior depends on implementation:
     *          - Some discard the frame
     *          - Some transmit anyway
     *          - Some signal error via AbortOnError flag
     *          
     *          Flags control transmission behavior:
     *          - Loopback: Frame is echoed back to RX queue (for testing)
     *          - AbortOnError: Abort transmission if bus error occurs
     * 
     * @param[in] frame CAN frame to transmit
     * @param[in] tx_deadline Transmission deadline in microseconds (monotonic clock, 0=ASAP)
     * @param[in] flags Transmission control flags (Loopback, AbortOnError)
     * 
     * @return 1 if frame queued successfully
     * @return 0 if no space in TX queue (queue full, try again later)
     * @return Negative value if error occurred (e.g., bus-off, invalid frame)
     * 
     * @note This is a pure virtual method - must be implemented by platform HAL
     * @note Frame transmission order follows CAN arbitration (not FIFO)
     * @warning Missing deadline may cause protocol failures in time-critical applications
     * @warning TX queue overflow loses frames - monitor via get_statistics()
     */
    virtual int16_t send(const CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags);

    /**
     * @brief Receive CAN frame from RX queue
     * 
     * @details Non-blocking call to dequeue a received frame. Returns immediately
     *          whether or not a frame is available.
     *          
     *          The timestamp represents when the frame was received (start-of-frame time)
     *          in microseconds on the monotonic system clock. Useful for time-sync protocols.
     *          
     *          Flags indicate frame reception conditions:
     *          - Loopback: Frame was transmitted by this interface with loopback enabled
     *          - IsForwardedFrame: Frame was forwarded from another CAN interface
     * 
     * @param[out] out_frame Received CAN frame data
     * @param[out] out_ts_monotonic Reception timestamp in microseconds (monotonic clock)
     * @param[out] out_flags Reception flags (Loopback, IsForwardedFrame)
     * 
     * @return 1 if frame received successfully
     * @return 0 if no frame available in RX queue (queue empty)
     * @return Negative value if error occurred
     * 
     * @note This is a pure virtual method - must be implemented by platform HAL
     * @note Frames are typically ordered by reception time (FIFO)
     * @note RX queue overflow causes frame loss - monitor via get_statistics()
     */
    virtual int16_t receive(CANFrame& out_frame, uint64_t& out_ts_monotonic, CanIOFlags& out_flags);

    /**
     * @brief Configure hardware acceptance filters
     * 
     * @details Configures CAN controller hardware filters to accept only frames matching
     *          the specified ID/mask patterns. Rejected frames are discarded by hardware,
     *          reducing CPU load and RX queue usage.
     *          
     *          Each filter accepts frames where: (received_id & mask) == (id & mask)
     *          
     *          Number of filters is hardware-limited (check getNumFilters()).
     *          Typical hardware supports 8-28 filters depending on controller.
     *          
     *          Filter configuration examples:
     *          - DroneCAN node: Configure filters for service requests and broadcasts
     *          - ESC telemetry: Filter for specific ESC ID range
     *          - Single ID: Use full mask (0x7FF standard, 0x1FFFFFFF extended)
     *          
     *          Note: Some implementations may not support dynamic filter reconfiguration
     *          and return false. In this case, use software filtering.
     * 
     * @param[in] filter_configs Array of filter configurations (ID/mask pairs)
     * @param[in] num_configs Number of filters in array (must not exceed getNumFilters())
     * 
     * @return true if filters configured successfully
     * @return false if filtering not supported or configuration failed
     * 
     * @note Default implementation returns false (filtering not supported)
     * @note Configuring filters may briefly interrupt frame reception
     * @warning Incorrect filter configuration can block all reception
     * @warning Filters may be cleared on bus-off recovery in some hardware
     */
    virtual bool configureFilters(const CanFilterConfig* filter_configs, uint16_t num_configs)
    {
        return 0;
    }

    /**
     * @brief Get number of available hardware filters
     * 
     * @details Returns the maximum number of hardware acceptance filters supported
     *          by the CAN controller. This limits how many filter configurations
     *          can be passed to configureFilters().
     *          
     *          Typical values:
     *          - STM32 bxCAN: 14 filters (shared between CAN1/CAN2)
     *          - STM32 FDCAN: 28 standard + 8 extended filters
     *          - Linux SocketCAN: Software filtering (unlimited)
     * 
     * @return Number of available hardware filters, or 0 if filtering not supported
     * 
     * @note Default implementation returns 0 (hardware filtering not available)
     */
    virtual uint16_t getNumFilters() const
    {
        return 0;
    }

    /**
     * @brief Get total error count for CAN interface
     * 
     * @details Returns cumulative error count from CAN controller error counters.
     *          Typical CAN controllers maintain separate TX and RX error counters
     *          (TEC and REC). This method typically returns their sum.
     *          
     *          Error count interpretation:
     *          - 0-95: Error-active state (normal operation)
     *          - 96-127: Error-warning state (errors increasing)
     *          - 128-255: Error-passive state (high error rate)
     *          - >255: Bus-off state (disconnected from bus)
     * 
     * @return Total error count, or 0 if not supported
     * 
     * @note Default implementation returns 0 (error counting not available)
     * @note Error counters may reset on successful communication
     * @warning Rapidly increasing error count indicates bus problems
     */
    virtual uint32_t getErrorCount() const
    {
        return 0;
    }

    /**
     * @struct bus_stats_t
     * @brief CAN bus statistics for monitoring and logging
     * 
     * @details Comprehensive statistics for CAN bus health monitoring and debugging.
     *          All counters are cumulative since interface initialization.
     */
    typedef struct {
        uint32_t tx_requests;       ///< Total transmission requests via send()
        uint32_t tx_rejected;       ///< Frames rejected (invalid, bus-off, etc.)
        uint32_t tx_overflow;       ///< Frames dropped due to TX queue full
        uint32_t tx_success;        ///< Frames successfully transmitted to bus
        uint32_t tx_timedout;       ///< Frames that missed transmission deadline
        uint32_t tx_abort;          ///< Frames aborted (AbortOnError flag or manual abort)
        uint32_t rx_received;       ///< Frames successfully received
        uint32_t rx_overflow;       ///< Frames dropped due to RX queue full
        uint32_t rx_errors;         ///< Frames received with errors
        uint32_t num_busoff_err;    ///< Number of times bus-off state entered
        uint64_t last_transmit_us;  ///< Timestamp of last successful transmission (microseconds)
    } bus_stats_t;

#if !defined(HAL_BOOTLOADER_BUILD)
    /**
     * @brief Get human-readable status information
     * 
     * @details Appends formatted status information to an expanding string.
     *          Typically includes error counts, queue states, and bus statistics.
     *          Used for diagnostic output and debugging.
     * 
     * @param[out] str Expanding string to append status information to
     * 
     * @note Default implementation does nothing (no status available)
     */
    virtual void get_stats(ExpandingString &str) {}

    /**
     * @brief Get bus statistics structure for logging
     * 
     * @details Returns pointer to internal statistics structure for data logging
     *          and monitoring. Statistics include TX/RX counts, error counts,
     *          and overflow counters.
     *          
     *          Useful for:
     *          - Flight data logging (bus health monitoring)
     *          - Troubleshooting communication issues
     *          - Detecting bus overload conditions
     *          - Monitoring peripheral health
     * 
     * @return Pointer to statistics structure, or nullptr if not available
     * 
     * @note Default implementation returns nullptr (statistics not available)
     * @note Returned pointer remains valid until interface is destroyed
     * @warning Do not modify returned structure (const for a reason)
     */
    virtual const bus_stats_t *get_statistics(void) const { return nullptr; };
#endif

    /**
     * @brief Check if interface is in bus-off state
     * 
     * @details Bus-off occurs when error count exceeds 255, indicating severe
     *          bus problems (wrong bitrate, hardware failure, cable issues).
     *          
     *          In bus-off state:
     *          - Interface is disconnected from bus
     *          - No transmission or reception possible
     *          - Manual recovery may be required
     *          
     *          Recovery typically requires:
     *          - Wait 128 occurrences of 11 consecutive recessive bits
     *          - Reinitialize interface if automatic recovery fails
     * 
     * @return true if bus-off state active, false if operational
     * 
     * @note Default implementation returns false (bus-off detection not available)
     * @warning Bus-off indicates serious bus problems requiring investigation
     */
    virtual bool is_busoff() const
    {
        return false;
    }

    /**
     * @brief Flush all pending transmissions (testing only)
     * 
     * @details Discards all queued TX frames. Used only during testing
     *          and development. Not for normal operation.
     * 
     * @note Default implementation does nothing
     * @warning Using this in flight will lose queued telemetry and commands
     */
    virtual void flush_tx() {}

    /**
     * @brief Clear all received frames (testing only)
     * 
     * @details Discards all queued RX frames. Used only during testing
     *          and development. Not for normal operation.
     * 
     * @note Default implementation does nothing
     * @warning Using this in flight will lose received data
     */
    virtual void clear_rx() {}

    /**
     * @brief Check if interface has been successfully initialized
     * 
     * @details Returns true if init() was called and succeeded, false otherwise.
     *          Used to verify interface is ready for send/receive operations.
     * 
     * @return true if initialized and ready, false otherwise
     * 
     * @note This is a pure virtual method - must be implemented by platform HAL
     */
    virtual bool is_initialized() const = 0;

    /**
     * @typedef FrameCb
     * @brief Callback function type for frame reception notification
     * 
     * @details Signature: void callback(uint8_t iface_num, const CANFrame& frame, CanIOFlags flags)
     *          
     *          Callbacks are invoked when frames are received, allowing protocol
     *          handlers to process frames directly without polling receive queue.
     * 
     * @param iface_num CAN interface number (0-based)
     * @param frame Received CAN frame
     * @param flags Reception flags
     */
    FUNCTOR_TYPEDEF(FrameCb, void, uint8_t, const AP_HAL::CANFrame &, CanIOFlags);

    /**
     * @brief Register callback for frame reception
     * 
     * @details Registers a callback function to be invoked when frames are received.
     *          Up to 3 callbacks can be registered per interface. Useful for protocol
     *          handlers that need immediate frame notification.
     * 
     * @param[in] cb Callback function to register
     * @param[out] cb_id Assigned callback ID for later unregistration
     * 
     * @return true if callback registered successfully, false if all slots full
     * 
     * @note Callbacks are invoked from interrupt context or driver thread
     * @warning Callback execution time must be minimal to avoid blocking reception
     */
    virtual bool register_frame_callback(FrameCb cb, uint8_t &cb_id);

    /**
     * @brief Unregister frame reception callback
     * 
     * @param[in] cb_id Callback ID returned from register_frame_callback()
     */
    virtual void unregister_frame_callback(uint8_t cb_id);

protected:
    /**
     * @brief Get interface number
     * 
     * @return CAN interface number (0-based)
     * 
     * @note Pure virtual - must be implemented by platform HAL
     */
    virtual int8_t get_iface_num() const = 0;

    /**
     * @brief Add received frame to RX queue
     * 
     * @details Internal method for platform HAL to add frames to RX queue.
     *          Invokes registered callbacks after queueing.
     * 
     * @param[in] rx_item Received frame with metadata
     * 
     * @return true if frame queued successfully, false if queue full
     * 
     * @note Pure virtual - must be implemented by platform HAL
     */
    virtual bool add_to_rx_queue(const CanRxItem &rx_item) = 0;

    /**
     * @struct callbacks
     * @brief Frame reception callback management
     */
    struct {
#ifndef HAL_BOOTLOADER_BUILD
        HAL_Semaphore sem;  ///< Semaphore protecting callback array
#endif
        FrameCb cb[3];      ///< Up to 3 registered callbacks per interface
    } callbacks;

    uint32_t bitrate_;      ///< Configured bitrate in bits/second
    OperatingMode mode_;    ///< Current operating mode
};
