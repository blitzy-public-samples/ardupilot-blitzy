#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"
#include <AP_Logger/AP_Logger_config.h>

#ifndef HAL_UART_STATS_ENABLED
#define HAL_UART_STATS_ENABLED AP_HAL_UARTDRIVER_ENABLED
#endif

#ifndef AP_UART_MONITOR_ENABLED
#define AP_UART_MONITOR_ENABLED 0
#endif

class ExpandingString;
class ByteBuffer;

/**
 * @file UARTDriver.h
 * @brief Serial UART interface for telemetry, GPS, and peripherals
 * 
 * Defines abstract interface for serial port communication used extensively for
 * MAVLink telemetry, GPS receivers, companion computers, and peripheral devices.
 * Supports buffered I/O, flow control, port locking, and hardware feature configuration.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

/**
 * @class AP_HAL::UARTDriver
 * @brief Abstract serial port interface with advanced features
 * 
 * @details Provides buffered serial I/O inheriting from BetterStream for Arduino-like API.
 *          
 *          UART features:
 *          - Buffered TX/RX: Ring buffers prevent data loss during processing gaps
 *          - Flow control: Hardware (RTS/CTS) and software (XON/XOFF)
 *          - Port locking: Exclusive access for scripting or custom protocols
 *          - Baud rate: Configurable from 1200 to 2000000 bps
 *          - Options: Bit inversion, half-duplex, pin swapping, DMA control
 *          
 *          Serial port allocation (via AP_SerialManager):
 *          - Serial0: USB console (typically MAVLink)
 *          - Serial1-8: Hardware UART ports, assigned by SERIALn_PROTOCOL parameters
 *          - Common protocols: MAVLink (GCS/Telem), GPS, Rangefinder, Lidar, Gimbal
 *          
 *          Buffer sizing:
 *          - Default: Platform-specific (typically 128-512 bytes TX/RX)
 *          - Adjustable: Via begin(baud, rxSpace, txSpace) for high-bandwidth protocols
 *          - MAVLink: Typically 512-1024 bytes for message assembly
 *          
 *          Port locking mechanism (for Lua scripting):
 *          - lock_port(write_key, read_key): Obtain exclusive access
 *          - write_locked()/read_locked(): I/O with key verification
 *          - Prevents conflict between scripting and firmware protocols
 *          
 *          Thread safety:
 *          - write/read methods protected by internal locking
 *          - Multiple threads can safely access same port
 *          - Atomic operations for buffer management
 * 
 * @note UARTDriver operations are non-blocking (buffered) except wait_timeout()
 * @warning High baud rates (>115200) may overwhelm slow processors or cause buffer overruns
 * 
 * Source: libraries/AP_HAL/UARTDriver.h
 */
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    /* Do not allow copies */
    CLASS_NO_COPY(UARTDriver);

    /**
     * @brief Initialize serial port with specified baud rate
     * 
     * @details Initializes UART hardware, allocates default-sized TX/RX buffers,
     *          and configures 8N1 format (8 data bits, no parity, 1 stop bit).
     *          Clears any existing buffer contents. Safe to call multiple times
     *          to change baud rate. Port locking is checked by this top-level function.
     * 
     * @param[in] baud Baud rate in bits/second (common: 9600, 57600, 115200, 921600)
     * 
     * @note Uses platform-default buffer sizes (typically 128-512 bytes)
     * @note Calls protected _begin() method implemented by platform HAL
     * @see begin(uint32_t, uint16_t, uint16_t) for custom buffer sizes
     */
    void begin(uint32_t baud);

    /**
     * @brief Initialize serial port with custom buffer sizes
     * 
     * @details Initializes UART with specified TX/RX buffer sizes for high-bandwidth
     *          protocols or memory-constrained applications. Larger buffers prevent
     *          overruns during processing delays but consume more RAM.
     * 
     * @param[in] baud Baud rate in bits/second
     * @param[in] rxSpace Receive buffer size in bytes (minimum typically 16-32)
     * @param[in] txSpace Transmit buffer size in bytes (minimum typically 16-32)
     * 
     * @note Buffer sizes rounded to platform-specific alignment (often power-of-2)
     * @note MAVLink typically uses 512-1024 byte buffers for message assembly
     * @warning Requesting unavailable buffer size may fail silently or allocate less
     * @warning Excessive buffer sizes can exhaust available RAM on constrained platforms
     */
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace);

    /**
     * @brief Initialize port when already write-locked
     * 
     * @details Allows reinitializing a locked port without unlocking. Used by
     *          scripts that need to change baud rate while maintaining exclusive access.
     *          Does NOT lock the port - caller must already hold valid write_key
     *          from prior lock_port() call.
     * 
     * @param[in] baud Baud rate in bits/second
     * @param[in] rxSpace Receive buffer size in bytes
     * @param[in] txSpace Transmit buffer size in bytes
     * @param[in] write_key Valid write key from previous lock_port() call
     * 
     * @note Primarily for Lua scripting that needs to reconfigure locked ports
     * @warning Fails if write_key doesn't match current lock
     * @see lock_port()
     */
    void begin_locked(uint32_t baud, uint16_t rxSpace, uint16_t txSpace, uint32_t write_key);

    /**
     * @brief Write single byte to transmit buffer
     * 
     * @details Adds byte to TX ring buffer. Blocks if buffer full until space available
     *          (data transmitted by interrupt/DMA). Returns immediately if space available.
     *          Port locking checked - returns 0 if port locked with different key.
     * 
     * @param[in] c Byte value to transmit
     * 
     * @return size_t Number of bytes written (1 on success, 0 if port locked)
     * 
     * @note May block indefinitely if transmitter disabled or flow-control active
     * @see write_locked() for locked port access
     */
    size_t write(uint8_t c) override;

    /**
     * @brief Write buffer of bytes to transmit buffer
     * 
     * @details Copies bytes to TX ring buffer. Blocks if buffer full until sufficient
     *          space available. More efficient than multiple single-byte writes due to
     *          reduced locking overhead.
     * 
     * @param[in] buffer Pointer to data to transmit
     * @param[in] size Number of bytes to write
     * 
     * @return size_t Number of bytes written (may be less than requested if port locked)
     * 
     * @note May block until all data written or partial write if interrupted
     * @note Calls protected _write() method implemented by platform HAL
     */
    size_t write(const uint8_t *buffer, size_t size) override;

    /**
     * @brief Write null-terminated string to transmit buffer
     * 
     * @details Convenience method for string output. Does NOT send null terminator.
     *          Equivalent to write((uint8_t*)str, strlen(str)).
     * 
     * @param[in] str Null-terminated C string to transmit
     * 
     * @return size_t Number of bytes written (excluding null terminator)
     * 
     * @note Commonly used for debug output or text-based protocols
     */
    size_t write(const char *str) override;

    /**
     * @brief Read single byte from receive buffer
     * 
     * @details Non-blocking read from RX ring buffer. Returns immediately with
     *          data or -1 if no data available. Data removed from buffer after read.
     * 
     * @return int16_t Byte value (0-255) if data available, -1 if buffer empty
     * 
     * @note Always non-blocking - never waits for data
     * @note Returns -1 (not 0) for "no data" to distinguish from 0x00 byte
     * @see available() to check data count before reading
     */
    int16_t read(void) override;

    /**
     * @brief Read single byte via reference parameter
     * 
     * @details Alternative read interface that returns success via bool. Preferred
     *          for error checking since return value explicitly indicates success/failure.
     * 
     * @param[out] b Reference to receive byte if available
     * 
     * @return bool true if byte read successfully, false if buffer empty
     * 
     * @note WARN_IF_UNUSED attribute enforces checking return value
     * @see read() for int16_t return variant
     */
    bool read(uint8_t &b) override WARN_IF_UNUSED;

    /**
     * @brief Read multiple bytes from receive buffer
     * 
     * @details Non-blocking read of up to 'count' bytes. Returns immediately with
     *          whatever data is available (may be less than requested). More efficient
     *          than multiple single-byte reads due to reduced locking overhead.
     * 
     * @param[out] buffer Pointer to buffer to receive data
     * @param[in]  count  Maximum number of bytes to read
     * 
     * @return ssize_t Number of bytes actually read (0 if buffer empty, -1 on error)
     * 
     * @note Does not wait for 'count' bytes - returns immediately with available data
     * @note Calls protected _read() method implemented by platform HAL
     */
    ssize_t read(uint8_t *buffer, uint16_t count) override;
    
    /**
     * @brief Shutdown serial port and free resources
     * 
     * @details Stops UART hardware, deallocates TX/RX buffers, and disables interrupts.
     *          Port cannot be used until begin() called again. Any buffered data discarded.
     * 
     * @note Calls protected _end() method implemented by platform HAL
     * @see begin()
     */
    void end();

    /**
     * @brief Wait for transmit buffer to empty
     * 
     * @details Blocks until all buffered TX data has been transmitted. Does NOT
     *          guarantee data has been received - only that it left the UART transmitter.
     *          Useful before changing baud rate or shutting down port.
     * 
     * @note May block for extended time if flow control active or line disconnected
     * @note Calls protected _flush() method implemented by platform HAL
     */
    void flush();

    /**
     * @brief Check if port has been initialized
     * 
     * @return bool true if begin() has been called successfully, false otherwise
     * 
     * @note Pure virtual - implemented by platform HAL
     */
    virtual bool is_initialized() = 0;

    /**
     * @brief Check if transmit data is pending
     * 
     * @details Returns true if TX buffer contains data waiting to be transmitted.
     *          Useful for determining if flush() would block.
     * 
     * @return bool true if data in TX buffer, false if buffer empty
     * 
     * @note Pure virtual - implemented by platform HAL
     * @see tx_pending() for byte count (platform-specific)
     */
    virtual bool tx_pending() = 0;

    /**
     * @brief Lock port for exclusive scripting access
     * 
     * @details Prevents firmware code from accessing port, allowing Lua scripts
     *          or custom protocols exclusive control. Separate read/write keys
     *          enable asymmetric locking (e.g., script writes, firmware reads).
     *          Use key of 0 to unlock respective direction.
     * 
     * @param[in] write_key Non-zero key for write_locked() (0 to unlock writes)
     * @param[in] read_key  Non-zero key for read_locked() (0 to unlock reads)
     * 
     * @return bool true if lock acquired, false if already locked with different keys
     * 
     * @note Primarily for Lua scripting to claim UART for custom protocols
     * @note Firmware code cannot access locked port (write/read returns 0/-1)
     * @warning Locking GPS or GCS serial ports breaks navigation/telemetry
     * @warning Ensure script releases lock or provides fallback mechanism
     * 
     * @see write_locked(), read_locked(), is_write_locked()
     */
    bool lock_port(uint32_t write_key, uint32_t read_key);

    /**
     * @brief Count bytes available in receive buffer
     * 
     * @details Returns number of bytes that can be read immediately without blocking.
     *          Value may increase as data arrives via interrupt/DMA. Commonly used
     *          to check for complete packets before reading.
     * 
     * @return uint32_t Number of bytes in RX buffer (0 if empty or port locked)
     * 
     * @note Returns 0 if port is read-locked
     * @note Calls protected _available() method implemented by platform HAL
     * @see available_locked() for locked port access
     */
    uint32_t available() override;

    /**
     * @brief Count bytes available with lock key verification
     * 
     * @param[in] key Read key from lock_port()
     * 
     * @return uint32_t Number of bytes in RX buffer, 0 if key invalid
     * 
     * @see lock_port(), available()
     */
    uint32_t available_locked(uint32_t key);

    /**
     * @brief Discard all pending receive data
     * 
     * @details Clears RX buffer without reading data. Useful for protocol resync
     *          or flushing garbage data after configuration changes.
     * 
     * @return bool true if buffer cleared successfully, false on error
     * 
     * @note Calls protected _discard_input() method implemented by platform HAL
     */
    bool discard_input() override;

    /**
     * @brief Write to locked port with key verification
     * 
     * @details Allows writing to port locked by lock_port(). Validates write_key
     *          before permitting write. Used by Lua scripts to transmit on locked ports.
     * 
     * @param[in] buffer Pointer to data to transmit
     * @param[in] size   Number of bytes to write
     * @param[in] key    Write key from lock_port()
     * 
     * @return size_t Number of bytes written (0 if key invalid or port unlocked)
     * 
     * @note Returns 0 and discards data if key doesn't match lock
     * @see lock_port(), write()
     */
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key);

    /**
     * @brief Read from locked port with key verification
     * 
     * @details Allows reading from port locked by lock_port(). Validates read_key
     *          before permitting read. Used by Lua scripts to receive on locked ports.
     * 
     * @param[out] buf   Pointer to buffer to receive data
     * @param[in]  count Maximum number of bytes to read
     * @param[in]  key   Read key from lock_port()
     * 
     * @return ssize_t Number of bytes read, -1 if key invalid
     * 
     * @note WARN_IF_UNUSED attribute enforces checking return value
     * @see lock_port(), read()
     */
    ssize_t read_locked(uint8_t *buf, size_t count, uint32_t key) WARN_IF_UNUSED;

    /**
     * @brief Get current parity setting
     * 
     * @details Returns parity configuration for MAVLink passthrough or protocol
     *          compatibility checks. Parity set via configure_parity().
     * 
     * @return uint8_t Parity value (0=none, 1=odd, 2=even, platform-specific encoding)
     * 
     * @see configure_parity()
     */
    uint8_t get_parity(void);
    
    /**
     * @brief Configure optional hardware features
     * 
     * @details Sets bit flags controlling signal inversion, half-duplex mode,
     *          pin swapping, pullup/pulldown resistors, and DMA enable/disable.
     *          Platform-specific implementation - not all options supported on all boards.
     * 
     * @param[in] options Bitmask of Option enum flags
     * 
     * @return bool true if all options applied successfully, false if unsupported
     * 
     * @note Default implementation only accepts options==0 (disable all)
     * @note Typically configured via SERIALn_OPTIONS parameters
     * @see Option enum, get_options()
     */
    virtual bool set_options(uint16_t options) { _last_options = options; return options==0; }

    /**
     * @brief Get current option flags
     * 
     * @return uint16_t Current option bitmask
     * 
     * @see set_options(), Option enum
     */
    virtual uint16_t get_options(void) const { return _last_options; }

    /**
     * @enum Option
     * @brief Hardware option flags for serial port configuration
     * 
     * @details Bitmask values for configuring platform-specific UART features.
     *          Support varies by HAL implementation and hardware capabilities.
     */
    enum Option {
        OPTION_RXINV              = (1U<<0),  ///< Invert RX line (idle-low protocols)
        OPTION_TXINV              = (1U<<1),  ///< Invert TX line (idle-low protocols)
        OPTION_HDPLEX             = (1U<<2),  ///< Half-duplex (one-wire) mode for RS-485
        OPTION_SWAP               = (1U<<3),  ///< Swap RX and TX pins (board rework workaround)
        OPTION_PULLDOWN_RX        = (1U<<4),  ///< Apply pulldown resistor to RX pin
        OPTION_PULLUP_RX          = (1U<<5),  ///< Apply pullup resistor to RX pin (default for many protocols)
        OPTION_PULLDOWN_TX        = (1U<<6),  ///< Apply pulldown resistor to TX pin
        OPTION_PULLUP_TX          = (1U<<7),  ///< Apply pullup resistor to TX pin
        OPTION_NODMA_RX           = (1U<<8),  ///< Disable DMA for RX (use interrupts, debug timing issues)
        OPTION_NODMA_TX           = (1U<<9),  ///< Disable DMA for TX (use interrupts, debug timing issues)
        OPTION_MAVLINK_NO_FORWARD_old = (1U<<10), ///< Deprecated - moved to GCS_MAVLINK::Option
        OPTION_NOFIFO             = (1U<<11), ///< Disable hardware FIFO (workaround for buggy hardware)
        OPTION_NOSTREAMOVERRIDE_old   = (1U<<12), ///< Deprecated - moved to GCS_MAVLINK::Option
    };

    /**
     * @brief Check if specific option flag is enabled
     * 
     * @param[in] option Option enum value to check
     * 
     * @return bool true if option bit is set, false otherwise
     * 
     * @note Convenience method for testing individual option bits
     */
    bool option_is_set(Option option) const {
        return (_last_options & (uint16_t)option) != 0;
    }

    /**
     * @enum flow_control
     * @brief Flow control modes for serial communication
     * 
     * @details Controls hardware (RTS/CTS) or software (XON/XOFF) flow control
     *          to prevent buffer overruns. AUTO mode enables flow control only
     *          when USB connection detected (disables for hardware UARTs).
     */
    enum flow_control {
        FLOW_CONTROL_DISABLE=0,  ///< No flow control (default)
        FLOW_CONTROL_ENABLE=1,   ///< Hardware RTS/CTS flow control always enabled
        FLOW_CONTROL_AUTO=2,     ///< Enable flow control only for USB ports (auto-detect)
        FLOW_CONTROL_RTS_DE=3,   ///< RTS pin used as RS-485 driver enable (not flow control)
    };

    /**
     * @brief Configure flow control mode
     * 
     * @details Sets hardware RTS/CTS flow control behavior. Prevents buffer overruns
     *          on high-speed connections but requires hardware support (RTS/CTS pins).
     *          AUTO mode recommended - enables only for USB where beneficial.
     * 
     * @param[in] flow_control_setting Desired flow control mode
     * 
     * @note Default implementation does nothing (no flow control)
     * @note Not all platforms support all modes
     * @see get_flow_control(), flow_control_enabled()
     */
    virtual void set_flow_control(enum flow_control flow_control_setting) {};

    /**
     * @brief Get current flow control mode
     * 
     * @return enum flow_control Current flow control setting
     * 
     * @note Default implementation returns FLOW_CONTROL_DISABLE
     * @see set_flow_control()
     */
    virtual enum flow_control get_flow_control(void) { return FLOW_CONTROL_DISABLE; }

    /**
     * @brief Check if flow control is currently active
     * 
     * @details Helper that evaluates get_flow_control() to determine if
     *          flow control is actually enabled (not just AUTO).
     * 
     * @return bool true if flow control active, false otherwise
     * 
     * @see flow_control_enabled(enum), get_flow_control()
     */
    bool flow_control_enabled() { return flow_control_enabled(get_flow_control()); }

    /**
     * @brief Configure parity checking
     * 
     * @details Sets parity mode for serial communication. Common values:
     *          0=none (default), 1=odd, 2=even. Encoding is platform-specific.
     * 
     * @param[in] v Parity value (platform-specific encoding)
     * 
     * @note Default implementation does nothing (no parity)
     * @note Must be called before begin() on some platforms
     * @see get_parity()
     */
    virtual void configure_parity(uint8_t v){};

    /**
     * @brief Configure number of stop bits
     * 
     * @details Sets stop bit count. Common values: 1 (default), 2.
     *          Rarely needed - most protocols use 1 stop bit.
     * 
     * @param[in] n Number of stop bits (typically 1 or 2)
     * 
     * @note Default implementation does nothing (1 stop bit)
     * @note Must be called before begin() on some platforms
     */
    virtual void set_stop_bits(int n){};

    /**
     * @brief Enable unbuffered write mode
     * 
     * @details Bypasses TX ring buffer - writes go directly to hardware transmitter.
     *          Reduces latency for low-bandwidth protocols but may lose data if
     *          writer faster than transmitter. Platform-specific support.
     * 
     * @param[in] on true to enable unbuffered, false for normal buffered writes
     * 
     * @return bool true if mode set successfully, false if unsupported
     * 
     * @note Default implementation returns false (not supported)
     * @warning Unbuffered mode can lose data if write() called faster than transmission
     */
    virtual bool set_unbuffered_writes(bool on){ return false; };

    /**
     * @brief Wait for minimum byte count with timeout
     * 
     * @details Blocks until at least n bytes available in RX buffer or timeout expires.
     *          Unlike standard read(), this method DOES block. Used for protocols
     *          requiring complete packets before processing.
     * 
     * @param[in] n          Minimum number of bytes to wait for
     * @param[in] timeout_ms Maximum wait time in milliseconds
     * 
     * @return bool true if n bytes available, false if timeout
     * 
     * @note This is a BLOCKING call - unlike most UARTDriver methods
     * @note Default implementation returns false (not supported)
     * @warning Can block indefinitely if timeout_ms=0 and data never arrives
     */
    virtual bool wait_timeout(uint16_t n, uint32_t timeout_ms) { return false; }

    /**
     * @brief Optional timer tick for platform-specific processing
     * 
     * @details Called periodically by scheduler for HAL implementations requiring
     *          regular processing (e.g., software FIFOs, statistics). Most platforms
     *          use interrupt/DMA and don't need this.
     * 
     * @note Default implementation does nothing
     * @note Called from interrupt context on some platforms
     */
    virtual void _timer_tick(void) { }

    /**
     * @brief Estimate receive timestamp for protocol timing
     * 
     * @details Calculates upper bound timestamp (in microseconds) when nbytes packet
     *          started arriving. Accounts for baud rate and system buffering.
     *          
     *          Time constraint interpretation:
     *          - Packet guaranteed NOT to have started after returned timestamp
     *          - Packet MAY have started earlier (buffering delays)
     *          - Useful for time-sync protocols and packet ordering
     * 
     * @param[in] nbytes Size of received packet in bytes
     * 
     * @return uint64_t Estimated receive start time in microseconds since boot
     * 
     * @note For USB transports (no fixed baud rate), estimate less accurate
     * @note Calculation: current_time - (nbytes * bits_per_byte / baud_rate)
     * @see bw_in_bytes_per_second()
     */
    virtual uint64_t receive_time_constraint_us(uint16_t nbytes);

    /**
     * @brief Get effective bandwidth in bytes per second
     * 
     * @details Calculates actual data throughput accounting for start/stop bits
     *          and protocol overhead. Used for buffer sizing and rate limiting.
     *          
     *          Typical calculation: baud_rate / bits_per_byte
     *          8N1 format: 10 bits per byte (1 start + 8 data + 1 stop)
     * 
     * @return uint32_t Bandwidth in bytes/second (default 5760 = 57600 baud @ 8N1)
     * 
     * @note Default assumes 57600 baud (common GPS/telemetry rate)
     * @see get_baud_rate()
     */
    virtual uint32_t bw_in_bytes_per_second() const {
        return 5760;
    }

    /**
     * @brief Get current baud rate
     * 
     * @return uint32_t Baud rate in bits/second, 0 if not initialized or USB
     * 
     * @note Returns 0 for USB ports (no fixed baud rate)
     * @see begin()
     */
    virtual uint32_t get_baud_rate() const { return 0; }

    /**
     * @brief Check if DMA enabled for both TX and RX
     * 
     * @details DMA (Direct Memory Access) improves performance and reduces CPU load
     *          for high-speed serial communication. Not all platforms support DMA.
     * 
     * @return bool true if DMA enabled for both directions, false otherwise
     * 
     * @note Default returns false (interrupt-driven I/O)
     * @see Option::OPTION_NODMA_RX, Option::OPTION_NODMA_TX
     */
    virtual bool is_dma_enabled() const { return false; }

#if HAL_UART_STATS_ENABLED
    /**
     * @struct StatsTracker
     * @brief Helper for tracking UART data usage over time
     * 
     * @details Tracks TX/RX byte counts and dropped bytes for monitoring
     *          and diagnostics. Used by uart_info() to report statistics
     *          to @SYS/uarts.txt for ground station display.
     */
    struct StatsTracker {
        /**
         * @class ByteTracker
         * @brief Tracks byte count deltas
         * 
         * @details Maintains last-seen cumulative count to calculate
         *          bytes transferred since last update() call.
         */
        class ByteTracker {
        public:
            /**
             * @brief Update with new cumulative count
             * 
             * @param[in] bytes New cumulative byte count
             * 
             * @return uint32_t Bytes transferred since last update() call
             * 
             * @note Returns difference: bytes - last_bytes
             */
            uint32_t update(uint32_t bytes);
        private:
            uint32_t last_bytes;  ///< Last cumulative byte count
        };
        ByteTracker tx;          ///< Transmit byte tracker
        ByteTracker rx;          ///< Receive byte tracker
        ByteTracker rx_dropped;  ///< Dropped receive byte tracker
    };

    /**
     * @brief Generate UART statistics for diagnostics
     * 
     * @details Appends human-readable UART statistics to string for
     *          @SYS/uarts.txt MAVLink file. Includes baud rate, buffer usage,
     *          throughput, and error counts.
     * 
     * @param[out] str    ExpandingString to append statistics to
     * @param[in,out] stats StatsTracker for calculating deltas
     * @param[in]  dt_ms  Time interval in milliseconds since last call
     * 
     * @note Default implementation does nothing (no stats available)
     * @see StatsTracker
     */
    virtual void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) {}

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log UART statistics to dataflash
     * 
     * @details Writes UART performance metrics to binary log for post-flight
     *          analysis. Logs throughput, buffer usage, and error rates.
     * 
     * @param[in] inst  UART instance number (0-based)
     * @param[in,out] stats StatsTracker for calculating deltas
     * @param[in] dt_ms Time interval in milliseconds since last call
     * 
     * @see StatsTracker, uart_info()
     */
    void log_stats(const uint8_t inst, StatsTracker &stats, const uint32_t dt_ms);
#endif
#endif // HAL_UART_STATS_ENABLED

    /**
     * @brief Manually control RTS pin state
     * 
     * @details Software control of RTS (Request To Send) pin for custom protocols
     *          or RS-485 driver enable. Not related to automatic flow control.
     * 
     * @param[in] high true to set RTS high, false for low
     * 
     * @return bool true if pin controlled successfully, false if unavailable
     * 
     * @note Default returns false (not supported)
     * @note Pin state interpretation is platform-specific (may be inverted)
     * @see flow_control::FLOW_CONTROL_RTS_DE
     */
    virtual bool set_RTS_pin(bool high) { return false; };

    /**
     * @brief Manually control CTS pin state
     * 
     * @details Software control of CTS (Clear To Send) pin. Rarely used -
     *          CTS typically input for flow control.
     * 
     * @param[in] high true to set CTS high, false for low
     * 
     * @return bool true if pin controlled successfully, false if unavailable
     * 
     * @note Default returns false (not supported)
     */
    virtual bool set_CTS_pin(bool high) { return false; };

    /**
     * @brief Get baud rate requested by USB host
     * 
     * @details For USB CDC (virtual serial) ports, returns baud rate set by
     *          ground station or terminal program. Not actual hardware baud
     *          (USB has no fixed baud). Used for protocol auto-detection.
     * 
     * @return uint32_t Requested baud rate, 0 if not USB or not connected
     * 
     * @note Only meaningful for USB CDC ports
     * @see get_baud_rate()
     */
    virtual uint32_t get_usb_baud(void) const { return 0; }

    /**
     * @brief Get parity requested by USB host
     * 
     * @details For USB CDC ports, returns parity setting from host application.
     *          Used for protocol auto-detection or configuration mirroring.
     * 
     * @return uint8_t Requested parity value (encoding matches configure_parity())
     * 
     * @note Only meaningful for USB CDC ports
     * @see get_parity(), configure_parity()
     */
    virtual uint8_t get_usb_parity(void) const { return parity; }

    /**
     * @brief Disable TX/RX pins for unused UART
     * 
     * @details Tri-states or disables TX/RX pins to save power and prevent
     *          signal interference on unused serial ports. Platform-specific.
     * 
     * @note Default implementation does nothing
     * @note Useful for boards with many UARTs where some unused
     */
    virtual void disable_rxtx(void) const {}

#if AP_UART_MONITOR_ENABLED
    /**
     * @brief Enable read monitoring to external buffer
     * 
     * @details Copies all received data to specified ByteBuffer for debugging
     *          or protocol analysis. Used by AP_Periph for GPS debugging.
     *          Does not affect normal read operations.
     * 
     * @param[in] buffer Pointer to ByteBuffer to receive copy of RX data
     * 
     * @return bool true if monitoring enabled successfully
     * 
     * @note Only available if AP_UART_MONITOR_ENABLED=1
     * @note Monitor buffer overflow will drop oldest data
     * @warning Adds overhead to read path
     */
    bool set_monitor_read_buffer(ByteBuffer *buffer) {
        _monitor_read_buffer = buffer;
        return true;
    }
#endif

    /**
     * @brief Check if port is currently write-locked
     * 
     * @details Returns true if port has been locked via lock_port() with
     *          non-zero write_key. Used to check lock status before attempting
     *          normal writes (which would fail on locked port).
     * 
     * @return bool true if port locked for writing, false if unlocked
     * 
     * @see lock_port(), write_locked()
     */
    bool is_write_locked(void) const {
        return lock_write_key != 0;
    }

protected:
    /**
     * @name Port Locking State
     * @{
     */
    uint32_t lock_write_key;  ///< Non-zero if port write-locked, 0 if unlocked
    uint32_t lock_read_key;   ///< Non-zero if port read-locked, 0 if unlocked
    /** @} */

    uint8_t parity;  ///< Current parity setting (platform-specific encoding)

    /**
     * @brief Platform-specific port initialization (pure virtual)
     * 
     * @details Implemented by each HAL to initialize hardware, allocate buffers,
     *          configure pins, and enable interrupts/DMA. Called by public begin().
     * 
     * @param[in] baud    Baud rate in bits/second
     * @param[in] rxSpace RX buffer size in bytes
     * @param[in] txSpace TX buffer size in bytes
     * 
     * @note Must be implemented by platform HAL
     */
    virtual void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;

    /**
     * @brief Platform-specific write implementation (pure virtual)
     * 
     * @details Implemented by each HAL to copy data to TX buffer/hardware.
     *          Called by public write() after lock checking.
     * 
     * @param[in] buffer Pointer to data to transmit
     * @param[in] size   Number of bytes to write
     * 
     * @return size_t Number of bytes actually written
     * 
     * @note Must be implemented by platform HAL
     */
    virtual size_t _write(const uint8_t *buffer, size_t size) = 0;

    /**
     * @brief Platform-specific read implementation (pure virtual)
     * 
     * @details Implemented by each HAL to copy data from RX buffer/hardware.
     *          Called by public read() after lock checking.
     * 
     * @param[out] buffer Pointer to buffer to receive data
     * @param[in]  count  Maximum number of bytes to read
     * 
     * @return ssize_t Number of bytes read, 0 if empty, -1 on error
     * 
     * @note Must be implemented by platform HAL
     * @note WARN_IF_UNUSED enforces checking return value
     */
    virtual ssize_t _read(uint8_t *buffer, uint16_t count)  WARN_IF_UNUSED = 0;

    /**
     * @brief Platform-specific shutdown (pure virtual)
     * 
     * @details Implemented by each HAL to disable hardware, free buffers,
     *          and release resources. Called by public end().
     * 
     * @note Must be implemented by platform HAL
     */
    virtual void _end() = 0;

    /**
     * @brief Platform-specific flush (pure virtual)
     * 
     * @details Implemented by each HAL to wait for TX completion.
     *          Called by public flush().
     * 
     * @note Must be implemented by platform HAL
     */
    virtual void _flush() = 0;

    /**
     * @brief Platform-specific available count (pure virtual)
     * 
     * @details Implemented by each HAL to return RX buffer data count.
     *          Called by public available().
     * 
     * @return uint32_t Number of bytes in RX buffer
     * 
     * @note Must be implemented by platform HAL
     */
    virtual uint32_t _available() = 0;

    /**
     * @brief Platform-specific input discard (pure virtual)
     * 
     * @details Implemented by each HAL to clear RX buffer.
     *          Called by public discard_input().
     * 
     * @return bool true if successful, false on error
     * 
     * @note Must be implemented by platform HAL
     */
    virtual bool _discard_input(void) = 0;

    /**
     * @brief Check if flow control enabled for given setting
     * 
     * @details Helper to evaluate flow_control enum, handling AUTO mode
     *          and RTS_DE special case.
     * 
     * @param[in] flow_control_setting Flow control mode to evaluate
     * 
     * @return bool true if flow control active for given setting
     * 
     * @note AUTO mode returns true only if USB detected (platform-specific)
     */
    bool flow_control_enabled(enum flow_control flow_control_setting) const;

#if HAL_UART_STATS_ENABLED
    /**
     * @brief Get cumulative transmit byte count
     * 
     * @details Returns total bytes transmitted since port initialization.
     *          Used by StatsTracker to calculate throughput deltas.
     * 
     * @return uint32_t Total TX bytes (default 0 if stats not implemented)
     * 
     * @see uart_info(), log_stats()
     */
    virtual uint32_t get_total_tx_bytes() const { return 0; }

    /**
     * @brief Get cumulative receive byte count
     * 
     * @details Returns total bytes received since port initialization.
     *          Used by StatsTracker to calculate throughput deltas.
     * 
     * @return uint32_t Total RX bytes (default 0 if stats not implemented)
     * 
     * @see uart_info(), log_stats()
     */
    virtual uint32_t get_total_rx_bytes() const { return 0; }

    /**
     * @brief Get cumulative dropped receive byte count
     * 
     * @details Returns total bytes lost due to RX buffer overruns.
     *          Indicates communication problems or insufficient buffer sizing.
     * 
     * @return uint32_t Total dropped RX bytes (default 0 if stats not implemented)
     * 
     * @note Non-zero values indicate buffer overruns - increase rxSpace or reduce baud
     * @see uart_info(), log_stats()
     */
    virtual uint32_t get_total_dropped_rx_bytes() const { return 0; }
#endif

    uint16_t _last_options;  ///< Last set option flags (cached for get_options())

private:

#if AP_UART_MONITOR_ENABLED
    ByteBuffer *_monitor_read_buffer;  ///< Buffer for read monitoring (debug feature)
#endif
};
