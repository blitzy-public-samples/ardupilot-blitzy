/**
 * @file AP_Scripting_SerialAccess.h
 * @brief Uniform serial port abstraction for Lua scripts
 * 
 * @details This file provides a wrapper class that abstracts serial port access
 *          for Lua scripts, dispatching between HAL UART drivers (hardware serial
 *          ports) and virtual serial devices created by scripts. This allows Lua
 *          scripts to use a consistent API regardless of whether they're accessing
 *          physical UART hardware or script-created virtual serial ports.
 *          
 *          The implementation conditionally dispatches calls to either:
 *          - AP_HAL::UARTDriver for hardware serial ports
 *          - Script-created serial devices when AP_SCRIPTING_SERIALDEVICE_ENABLED
 * 
 * @note This abstraction is specifically designed for the ArduPilot scripting
 *       interface and provides a subset of UART functionality suitable for
 *       script-based communication protocols.
 */

#pragma once

#include "AP_Scripting_config.h"
#include "AP_Scripting.h"

#include <AP_HAL/UARTDriver.h>

/**
 * @class AP_Scripting_SerialAccess
 * @brief Wrapper providing unified serial port API for Lua scripts
 * 
 * @details This class provides a consistent interface for Lua scripts to access
 *          both hardware UART ports and script-created virtual serial devices.
 *          It encapsulates the dispatching logic between physical hardware serial
 *          ports (via AP_HAL::UARTDriver) and virtual serial devices.
 *          
 *          The class provides standard serial port operations including:
 *          - Baud rate configuration
 *          - Parity and stop bit settings
 *          - Read/write operations (byte and buffer)
 *          - Flow control configuration
 *          - Buffer availability queries
 *          
 *          When AP_SCRIPTING_SERIALDEVICE_ENABLED is defined, the class can
 *          represent either a hardware port or a script device, with behavior
 *          determined by the is_device_port flag.
 * 
 * @warning Thread-safety: Be cautious when accessing serial ports from both
 *          the script thread and main thread simultaneously. Coordinate access
 *          using appropriate locking mechanisms to prevent race conditions.
 * 
 * @note This class is non-copyable to prevent issues with multiple references
 *       to the same underlying serial port hardware or device.
 */
class AP_Scripting_SerialAccess {
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting_SerialAccess);

    /**
     * @brief Default constructor for serial access wrapper
     * 
     * @details Constructs an uninitialized serial access object. The actual
     *          serial port must be configured by setting the stream pointer
     *          and calling begin() with the desired baud rate.
     */
    AP_Scripting_SerialAccess() {}

    /**
     * @brief Initialize serial port with specified baud rate
     * 
     * @details Configures the serial port to operate at the specified baud rate.
     *          This method must be called before attempting to read or write data.
     *          The baud rate setting applies to both hardware UART ports and
     *          virtual serial devices.
     *          
     *          Common baud rates: 9600, 19200, 38400, 57600, 115200, 230400, 460800
     * 
     * @param[in] baud Baud rate in bits per second (e.g., 57600 for 57600 bps)
     * 
     * @note For hardware UART ports, ensure the baud rate is supported by the
     *       underlying hardware. Very high baud rates may not be achievable on
     *       all platforms.
     */
    void begin(uint32_t baud);

    /**
     * @brief Configure parity bit for serial communication
     * 
     * @details Sets the parity mode for error detection in serial communication.
     *          Parity adds an extra bit to each byte for basic error checking.
     *          This setting must match the parity configuration of the remote device.
     *          
     *          Parity values:
     *          - 0: No parity (most common)
     *          - 1: Odd parity
     *          - 2: Even parity
     * 
     * @param[in] parity Parity mode (0=none, 1=odd, 2=even)
     * 
     * @note Not all hardware platforms support all parity modes. Consult the
     *       HAL documentation for platform-specific limitations.
     */
    void configure_parity(uint8_t parity);

    /**
     * @brief Configure number of stop bits for serial communication
     * 
     * @details Sets the number of stop bits transmitted after each data byte.
     *          Stop bits provide timing separation between bytes. This setting
     *          must match the configuration of the remote device.
     *          
     *          Typical values:
     *          - 1: One stop bit (most common)
     *          - 2: Two stop bits (used for slower or more reliable communication)
     * 
     * @param[in] stop_bits Number of stop bits (typically 1 or 2)
     * 
     * @note Some protocols require specific stop bit configurations. Verify
     *       the requirements of your communication protocol.
     */
    void set_stop_bits(uint8_t stop_bits);

    /**
     * @brief Write a single byte to the serial port
     * 
     * @details Transmits a single byte through the serial port. The byte is
     *          queued in the output buffer and transmitted asynchronously.
     *          If the output buffer is full, this call may block briefly or
     *          fail to write the byte.
     * 
     * @param[in] c Single byte to transmit
     * 
     * @return Number of bytes successfully written (0 or 1)
     *         - 1: Byte successfully queued for transmission
     *         - 0: Output buffer full, byte not written
     * 
     * @note For time-critical applications, check the return value to ensure
     *       the byte was successfully queued.
     */
    size_t write(uint8_t c);

    /**
     * @brief Write a buffer of bytes to the serial port
     * 
     * @details Transmits multiple bytes through the serial port. Bytes are
     *          queued in the output buffer and transmitted asynchronously.
     *          If the output buffer has insufficient space, only a partial
     *          write may occur. The return value indicates how many bytes
     *          were actually queued.
     * 
     * @param[in] buffer Pointer to buffer containing bytes to transmit
     * @param[in] size Number of bytes to write from the buffer
     * 
     * @return Number of bytes successfully written (0 to size)
     *         May be less than size if output buffer has insufficient space
     * 
     * @note Always check the return value when writing buffers. If fewer bytes
     *       than requested were written, the caller must handle retrying or
     *       buffering the remaining data.
     * 
     * @warning The buffer pointer must remain valid and point to at least size
     *          bytes of readable memory.
     */
    size_t write(const uint8_t *buffer, size_t size);

    /**
     * @brief Read a single byte from the serial port
     * 
     * @details Reads one byte from the input buffer if data is available.
     *          This is a non-blocking call that returns immediately whether
     *          or not data is available.
     *          
     *          Typical usage pattern:
     *          @code
     *          int16_t byte = serial.read();
     *          if (byte >= 0) {
     *              // Process the byte
     *              handle_byte(byte);
     *          }
     *          @endcode
     * 
     * @return Byte value (0-255) or -1 if no data available
     *         - 0-255: Valid byte read from input buffer
     *         - -1: No data currently available
     * 
     * @note This method does not block. Use available() to check for data
     *       before calling read() to avoid unnecessary -1 returns.
     */
    int16_t read(void);

    /**
     * @brief Read multiple bytes from the serial port into a buffer
     * 
     * @details Reads up to count bytes from the input buffer into the provided
     *          buffer. This is a non-blocking call that reads as many bytes as
     *          are immediately available, up to the requested count.
     *          
     *          The method returns the actual number of bytes read, which may be:
     *          - Less than count if fewer bytes are available
     *          - 0 if no bytes are available
     *          - -1 on error conditions
     * 
     * @param[out] buffer Pointer to buffer where received bytes will be stored
     * @param[in]  count  Maximum number of bytes to read
     * 
     * @return Number of bytes actually read, or -1 on error
     *         - >0: Number of bytes successfully read into buffer
     *         - 0: No data currently available
     *         - -1: Error condition (e.g., invalid buffer pointer)
     * 
     * @note The buffer must have space for at least count bytes. The method
     *       does not null-terminate the data.
     * 
     * @warning The buffer pointer must remain valid and point to at least count
     *          bytes of writable memory.
     */
    ssize_t read(uint8_t *buffer, uint16_t count);

    /**
     * @brief Query number of bytes available in input buffer
     * 
     * @details Returns the number of bytes currently available for reading
     *          in the serial port's input buffer. This method is useful for
     *          determining whether data is available before calling read(),
     *          or for reading all available data in a loop.
     *          
     *          Typical usage pattern:
     *          @code
     *          while (serial.available() > 0) {
     *              int16_t byte = serial.read();
     *              process_byte(byte);
     *          }
     *          @endcode
     * 
     * @return Number of bytes available for reading in the input buffer
     *         - 0: No data available
     *         - >0: Number of bytes that can be read immediately
     * 
     * @note The buffer occupancy can change between the call to available()
     *       and subsequent read() calls if data continues to arrive or if
     *       other threads access the same port.
     */
    uint32_t available(void);

    /**
     * @brief Configure hardware flow control for the serial port
     * 
     * @details Enables or disables RTS/CTS hardware flow control. Flow control
     *          prevents buffer overruns by allowing the receiver to signal the
     *          transmitter to pause data transmission when buffers are full.
     *          
     *          Flow control modes (from AP_HAL::UARTDriver):
     *          - FLOW_CONTROL_DISABLE: No flow control (default)
     *          - FLOW_CONTROL_ENABLE: Enable RTS/CTS hardware flow control
     *          - FLOW_CONTROL_AUTO: Automatically enable if supported
     *          
     *          Hardware flow control requires physical connections:
     *          - RTS (Request To Send): Output from this device
     *          - CTS (Clear To Send): Input to this device
     * 
     * @param[in] fcs Flow control setting from AP_HAL::UARTDriver::flow_control enum
     * 
     * @note Hardware flow control requires additional signal lines beyond TX/RX.
     *       Verify that the UART port has RTS/CTS pins available and properly
     *       connected before enabling flow control.
     * 
     * @warning Not all hardware platforms support flow control on all serial
     *          ports. Consult board-specific documentation for capabilities.
     */
    void set_flow_control(enum AP_HAL::UARTDriver::flow_control fcs);

    /**
     * @brief Pointer to underlying HAL UART driver
     * 
     * @details Points to the AP_HAL::UARTDriver instance representing the
     *          actual hardware serial port. This pointer is set by the scripting
     *          backend when the serial port is allocated to a script.
     *          
     *          When AP_SCRIPTING_SERIALDEVICE_ENABLED is not defined, all serial
     *          operations dispatch directly through this stream pointer to the
     *          HAL UART driver.
     */
    AP_HAL::UARTDriver *stream;

#if AP_SCRIPTING_ENABLED
#if AP_SCRIPTING_SERIALDEVICE_ENABLED
    /**
     * @brief Flag indicating whether this is a script-created virtual device
     * 
     * @details When AP_SCRIPTING_SERIALDEVICE_ENABLED is defined, this flag
     *          determines the dispatching behavior for serial operations:
     *          
     *          - true: This represents a script-created virtual serial device.
     *                  Operations are dispatched to the script device implementation
     *                  rather than directly to HAL UART hardware.
     *          
     *          - false: This represents a physical hardware UART port.
     *                   Operations are dispatched to the HAL UART driver through
     *                   the stream pointer.
     *          
     *          This allows the same API to work with both hardware serial ports
     *          and software-emulated serial devices created by Lua scripts for
     *          custom protocol implementations or serial port virtualization.
     * 
     * @note This flag is only present when AP_SCRIPTING_SERIALDEVICE_ENABLED
     *       is defined at compile time.
     */
    bool is_device_port;
#endif
#endif
};
