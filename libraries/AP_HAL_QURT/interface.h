/**
 * @file interface.h
 * @brief SLPI link interface for inter-processor communication between Hexagon DSP and applications processor
 * 
 * @details This file defines the RPC (Remote Procedure Call) interface for communication between
 *          the Hexagon DSP running QURT RTOS (SLPI - Sensor Low Power Island) and the applications
 *          processor (typically running Linux). The SLPI link enables DSP code to access hardware
 *          peripherals (UART, SPI, I2C) that are physically connected to the applications processor.
 * 
 *          Architecture Overview:
 *          - Hexagon DSP (SLPI subsystem) runs ArduPilot HAL code on QURT RTOS
 *          - Applications processor provides hardware peripheral access via RPC
 *          - All peripheral operations (I2C, SPI, UART) cross processor boundary via message passing
 *          - DSP has no direct hardware access; all I/O is proxied through apps processor
 * 
 *          RPC Protocol:
 *          - Protocol defined in ap_host/src/protocol.h (qurt_rpc_msg wire format)
 *          - Message passing with command IDs and parameter marshaling
 *          - Synchronous RPC: DSP blocks until apps processor returns response
 * 
 *          Typical RPC Flow Example (SPI transfer):
 *          1. DSP calls sl_client_spi_transfer()
 *          2. RPC message sent to apps processor
 *          3. Apps processor performs actual SPI transaction
 *          4. RPC response sent back to DSP
 *          5. DSP returns result to caller
 * 
 * @note SLPI (Sensor Low Power Island) is Qualcomm's low-power subsystem running on Hexagon DSP
 * @note Supported platforms: Snapdragon Flight (801), VOXL (820), VOXL2 (865)
 * @note Apps processor implementation located in ap_host/ directory
 * @note RPC framing implementation in replace.cpp
 * 
 * @warning RPC Latency: Every hardware access incurs 50-500us roundtrip delay for inter-processor
 *          communication, compared to <1us for native hardware access. Batch operations when possible.
 * @warning Protocol Versioning: DSP and apps processor builds must use compatible SLPI link protocol
 *          versions to ensure ABI stability across processor boundary.
 * @warning Resource Management: Handles (UART, SPI, I2C) must be properly opened/closed to avoid
 *          resource leaks on apps processor side.
 * 
 * @see replace.cpp for RPC message framing (qurt_rpc protocol)
 * @see UARTDriver.cpp for example sl_client_uart_* usage
 * @see SPIDevice.cpp for example sl_client_spi_* usage
 * @see I2CDevice.cpp for example sl_client_i2c_* usage
 * @see ap_host/src/protocol.h for wire protocol definition
 */

/**
 * @def __EXPORT
 * @brief Marks functions as exported from DSP SLPI to applications processor
 * 
 * @details This macro applies the GCC visibility attribute to make functions visible across the
 *          processor boundary. Required for function declarations that the applications processor
 *          can invoke via RPC on the DSP side (slpi_link_client_* functions).
 * 
 * @note Used by function declarations in the "SLPI LINK server calls into AP client" section
 * @note Not required for sl_client_* functions (DSP calls into apps processor)
 */
#define __EXPORT __attribute__ ((visibility ("default")))

#ifndef __cplusplus
#error "C++ should be defined!!!"
#endif

#include <stdint.h>

/**
 * @defgroup slpi_link_client Functions called by SLPI LINK server into AP client
 * @brief Callback functions invoked by applications processor into DSP
 * 
 * @details These functions are implemented on the DSP side and called by the applications
 *          processor via RPC. They handle initialization and asynchronous data reception
 *          from the apps processor (e.g., RC input updates, sensor interrupts).
 * @{
 */
extern "C" {
    /**
     * @brief Initialize ArduPilot on DSP and start main loop
     * 
     * @details Called by the SLPI LINK server during system startup to initialize the
     *          ArduPilot HAL and begin execution on the Hexagon DSP. This function sets up
     *          the QURT HAL, initializes the scheduler, and starts the main vehicle loop.
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Called once during boot sequence before any other SLPI link operations
     * @note Blocks until ArduPilot initialization completes
     * @warning Must complete initialization within reasonable timeout to avoid watchdog triggers
     */
    int slpi_link_client_init(void) __EXPORT;

    /**
     * @brief Callback invoked when applications processor sends data to DSP
     * 
     * @details Used for asynchronous notifications from apps processor to DSP, such as:
     *          - RC input updates from serial receiver
     *          - Sensor interrupt notifications
     *          - Configuration changes
     *          - Emergency commands
     * 
     *          This function is called from the SLPI link layer thread context, not the
     *          main ArduPilot scheduler. Data is typically queued for processing by the
     *          appropriate HAL driver.
     * 
     * @param[in] data Received data buffer from applications processor
     * @param[in] data_len_in_bytes Length of received data in bytes
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Called from SLPI link layer thread, not main scheduler thread
     * @note Must return quickly to avoid blocking SLPI link message processing
     * @warning Do not perform blocking operations or lengthy processing in this callback
     * @warning Data buffer is only valid for the duration of this call; copy if needed
     * 
     * @see replace.cpp for message dispatching implementation
     */
    int slpi_link_client_receive(const uint8_t *data, int data_len_in_bytes) __EXPORT;
}
/** @} */ // end of slpi_link_client group

/**
 * @defgroup sl_client_api SLPI Link Client API (DSP calls into apps processor)
 * @brief Hardware peripheral access via RPC to applications processor
 * 
 * @details The sl_client_* API provides hardware peripheral access from Hexagon DSP to the
 *          applications processor. All functions in this group perform synchronous RPC calls,
 *          blocking until the apps processor completes the operation and returns a result.
 * 
 *          Function naming convention: sl_client_<peripheral>_<operation>
 * 
 *          RPC Overhead:
 *          - Each call has 50-500us latency (processor communication overhead)
 *          - Compare to native hardware access: <1us on Linux HAL
 *          - Batch operations when possible to minimize RPC calls
 * 
 *          Error Handling:
 *          - Functions return negative error codes on failure
 *          - Caller must check return values for error conditions
 *          - Common errors: invalid handle, timeout, hardware failure
 * 
 * @note All sl_client_* functions are synchronous (blocking until RPC completes)
 * @warning RPC latency: Each call adds significant overhead compared to direct hardware access
 * @warning Error checking: Always verify return values; RPC or hardware failures can occur
 * 
 * @see ap_host/ directory for applications processor side implementation
 * @{
 */
extern "C" {
    /**
     * @brief Send a message to the applications processor
     * 
     * @details Transmits arbitrary data from DSP to apps processor. Used for control commands,
     *          telemetry data, or other inter-processor communication not covered by specific
     *          peripheral APIs.
     * 
     * @param[in] data Data buffer to send
     * @param[in] data_len_in_bytes Length of data in bytes
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Data is copied during transmission; buffer can be reused immediately after return
     * @warning Maximum message size is protocol-dependent; large messages may fail
     */
    int sl_client_send_data(const uint8_t *data, int data_len_in_bytes);

    /**
     * @brief Register callback for fatal error conditions
     * 
     * @details Registers a function to be called when an unrecoverable error occurs in the
     *          SLPI link layer or apps processor. Used for crash logging and emergency shutdown.
     * 
     * @param[in] func Callback function to invoke on fatal error (no parameters)
     * 
     * @note Callback is invoked from error context; limited operations are safe
     * @warning Callback should not perform blocking operations or RPC calls
     */
    void sl_client_register_fatal_error_cb(void (*func)(void));

    /**
     * @brief Register callback for hardware interrupts from apps processor
     * 
     * @details Allows DSP code to receive notifications when hardware interrupts occur on the
     *          apps processor (e.g., GPIO pin changes, peripheral ready signals).
     * 
     * @param[in] func Callback function: int callback(int irq_num, void* arg, void* context)
     * @param[in] arg User-defined argument passed to callback
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Callback is invoked from interrupt context on DSP side
     * @warning Callback must be reentrant and avoid blocking operations
     */
    int sl_client_register_interrupt_callback(int (*func)(int, void*, void*), void* arg);

    /**
     * @brief Get DSP CPU utilization percentage
     * 
     * @details Queries current CPU usage on the Hexagon DSP for performance monitoring and
     *          load balancing decisions.
     * 
     * @return CPU utilization as percentage (0-100), or negative error code on failure
     * 
     * @note Utilization is averaged over recent time window (typically 1 second)
     * @note Useful for detecting performance issues or excessive processing load
     */
    int sl_client_get_cpu_utilization(void);

    /**
     * @name I2C Interface API
     * @brief I2C bus access via RPC to applications processor
     * @{
     */
    
    /**
     * @brief Configure and open I2C bus on applications processor
     * 
     * @details Opens an I2C bus device on the apps processor and configures it for communication
     *          with a specific device. Must be called before performing I2C transfers.
     * 
     * @param[in] bus_number I2C bus number (hardware-specific, typically 0-7)
     * @param[in] address Initial I2C device address (7-bit format, 0x00-0x7F)
     * @param[in] frequency Bus frequency in Hz (typical: 100000, 400000, or 1000000)
     * 
     * @return File descriptor for I2C bus on success (>= 0), negative error code on failure
     * 
     * @note Returned file descriptor is valid only on apps processor; used as handle for subsequent calls
     * @note Common frequencies: 100kHz (standard), 400kHz (fast), 1MHz (fast-plus)
     * @warning File descriptor must be properly closed to avoid resource leaks on apps processor
     * @warning RPC overhead: Initial configuration adds ~100-200us latency
     * 
     * @see I2CDevice.cpp for usage example
     */
    int sl_client_config_i2c_bus(uint8_t bus_number, uint8_t address, uint32_t frequency);
    
    /**
     * @brief Change target I2C device address for subsequent transfers
     * 
     * @details Updates the I2C device address for an already-opened I2C bus. Useful when
     *          communicating with multiple devices on the same bus.
     * 
     * @param[in] fd File descriptor returned by sl_client_config_i2c_bus()
     * @param[in] address New I2C device address (7-bit format, 0x00-0x7F)
     * 
     * @note No return value; assumes success (address change is local on apps processor)
     * @note Does not require RPC roundtrip; cached for next transfer
     */
    void sl_client_set_address_i2c_bus(int fd, uint8_t address);
    
    /**
     * @brief Perform combined I2C write/read transaction
     * 
     * @details Executes an I2C transaction with optional write phase followed by optional read phase.
     *          Common patterns:
     *          - Write only: send_len > 0, recv_len = 0 (e.g., write register value)
     *          - Read only: send_len = 0, recv_len > 0 (e.g., read status)
     *          - Write-then-read: both > 0 (e.g., write register address, read data)
     * 
     *          Uses I2C repeated start for write-then-read to maintain bus ownership.
     * 
     * @param[in]  fd File descriptor returned by sl_client_config_i2c_bus()
     * @param[in]  send Send buffer (data to write to device), NULL if send_len = 0
     * @param[in]  send_len Number of bytes to write from send buffer
     * @param[out] recv Receive buffer (data read from device), NULL if recv_len = 0
     * @param[in]  recv_len Number of bytes to read into recv buffer
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note Uses I2C repeated start between write and read phases
     * @note Typical error codes: -EIO (bus error), -ENXIO (device not found), -ETIMEDOUT
     * @warning RPC overhead: Each call adds 50-500us latency; batch register operations when possible
     * @warning recv buffer must be at least recv_len bytes; no bounds checking
     * 
     * @see I2CDevice.cpp for usage examples
     */
    int sl_client_i2c_transfer(int fd, const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len);
    
    /** @} */ // end of I2C Interface API

    /**
     * @name SPI Interface API
     * @brief SPI bus access via RPC to applications processor
     * @{
     */
    
    /**
     * @brief Perform full-duplex SPI transfer
     * 
     * @details Executes a simultaneous send and receive operation on the SPI bus. SPI is inherently
     *          full-duplex: data is transmitted while data is received on every clock cycle.
     * 
     *          Typical usage:
     *          - Read sensor: Send dummy bytes (e.g., 0x00), capture received data
     *          - Write register: Send command/data, ignore received data
     *          - Read register: Send register address, read response
     * 
     * @param[in]     fd File descriptor for SPI device (from sl_client_config_spi_bus())
     * @param[in]     send Send buffer (data to transmit), must be at least len bytes
     * @param[out]    recv Receive buffer (data captured during transmission), must be at least len bytes
     * @param[in]     len Number of bytes to transfer in both directions
     * 
     * @return 0 on success, negative error code on failure
     * 
     * @note SPI is full-duplex: send and recv buffers must both be len bytes
     * @note Chip select is automatically asserted before transfer and deasserted after
     * @note SPI mode, frequency, and bit order are configured by sl_client_config_spi_bus()
     * @warning RPC overhead: Each call adds 50-500us latency; batch transfers when possible
     * @warning send and recv buffers must not overlap (undefined behavior)
     * @warning recv buffer must be at least len bytes; no bounds checking
     * 
     * @see SPIDevice.cpp for usage examples
     */
    int sl_client_spi_transfer(int fd, const uint8_t *send, uint8_t *recv, const unsigned len);
    
    /**
     * @brief Configure and open SPI bus on applications processor
     * 
     * @details Initializes the SPI bus on the apps processor for subsequent transfers. Configuration
     *          includes SPI mode (CPOL/CPHA), frequency, and bit order. Specific configuration
     *          parameters are hardcoded in the apps processor implementation.
     * 
     * @return File descriptor for SPI bus on success (>= 0), negative error code on failure
     * 
     * @note Returned file descriptor is valid only on apps processor; used as handle for transfers
     * @note SPI configuration (mode, frequency) is platform-specific and hardcoded in ap_host/
     * @warning File descriptor must be properly closed to avoid resource leaks on apps processor
     * @warning RPC overhead: Initial configuration adds ~100-200us latency
     * 
     * @see SPIDevice.cpp for usage example
     */
    int sl_client_config_spi_bus(void);
    
    /** @} */ // end of SPI Interface API

    /**
     * @name UART Interface API
     * @brief UART (serial port) access via RPC to applications processor
     * @{
     */
    
    /**
     * @brief Configure and open UART device on applications processor
     * 
     * @details Opens a UART (serial port) on the apps processor and configures baud rate.
     *          Used for GPS, telemetry radios, RC receivers, and ESC communication.
     * 
     * @param[in] port_number UART port number (see QURT_UART_* defines below)
     * @param[in] speed Baud rate in bits per second (e.g., 9600, 57600, 115200, 921600)
     * 
     * @return File descriptor for UART on success (>= 0), negative error code on failure
     * 
     * @note Returned file descriptor is valid only on apps processor; used as handle for read/write
     * @note UART is configured with 8N1 (8 data bits, no parity, 1 stop bit)
     * @note Hardware flow control (RTS/CTS) may be enabled depending on port configuration
     * @warning File descriptor must be properly closed to avoid resource leaks on apps processor
     * @warning RPC overhead: Initial configuration adds ~100-200us latency
     * 
     * @see QURT_UART_GPS, QURT_UART_RCIN, QURT_UART_ESC_IO for predefined port numbers
     * @see UARTDriver.cpp for usage example
     */
    int sl_client_config_uart(uint8_t port_number, uint32_t speed);
    
    /**
     * @brief Write data to UART
     * 
     * @details Transmits data through the UART to the connected device. Function blocks until
     *          all data is written to the UART transmit buffer (but not necessarily transmitted
     *          over the wire).
     * 
     * @param[in] fd File descriptor returned by sl_client_config_uart()
     * @param[in] data Data buffer to transmit
     * @param[in] data_len Number of bytes to write from data buffer
     * 
     * @return Number of bytes written on success (>= 0), negative error code on failure
     * 
     * @note Function is synchronous: blocks until data copied to TX buffer
     * @note Return value may be less than data_len if buffer is full
     * @warning RPC overhead: Each call adds 50-500us latency; batch writes when possible
     * @warning Large writes may fail if they exceed buffer size; check return value
     * 
     * @see UARTDriver.cpp for usage example
     */
    int sl_client_uart_write(int fd, const char *data, const unsigned data_len);
    
    /**
     * @brief Read data from UART
     * 
     * @details Reads available data from the UART receive buffer. Function returns immediately
     *          with whatever data is available (non-blocking read).
     * 
     * @param[in]  fd File descriptor returned by sl_client_config_uart()
     * @param[out] buffer Buffer to store received data
     * @param[in]  buffer_len Maximum number of bytes to read (buffer capacity)
     * 
     * @return Number of bytes read on success (0 if no data available, >0 if data read),
     *         negative error code on failure
     * 
     * @note Function is non-blocking: returns 0 immediately if no data is available
     * @note Return value indicates actual bytes read; may be less than buffer_len
     * @warning RPC overhead: Each call adds 50-500us latency even if no data available
     * @warning buffer must be at least buffer_len bytes; no bounds checking
     * @warning Polling with frequent reads is inefficient due to RPC overhead; use sparingly
     * 
     * @see UARTDriver.cpp for usage example
     */
    int sl_client_uart_read(int fd, char *buffer, const unsigned buffer_len);
    
    /** @} */ // end of UART Interface API
}
/** @} */ // end of sl_client_api group

/**
 * @name UART Port Identifiers
 * @brief Predefined port numbers for UART devices on Qualcomm platforms
 * 
 * @details These port numbers map to specific UARTs on the applications processor hardware.
 *          Port assignments are platform-specific (vary by Snapdragon generation).
 * @{
 */

/**
 * @def QURT_UART_GPS
 * @brief UART port for GPS receiver
 * @details Typically used for UBLOX, NMEA, or other GPS protocols at 9600-115200 baud
 */
#define QURT_UART_GPS 6

/**
 * @def QURT_UART_RCIN
 * @brief UART port for RC (radio control) receiver input
 * @details Used for serial RC protocols like SBUS, DSM, CRSF at protocol-specific baud rates
 */
#define QURT_UART_RCIN 7

/**
 * @def QURT_UART_ESC_IO
 * @brief UART port for ESC (electronic speed controller) or IO board communication
 * @details Bridges to ESC for protocols like DShot-over-serial, BLHeli passthrough, or
 *          communicates with separate I/O processor board
 */
#define QURT_UART_ESC_IO 2 // UART for the ESC or IO board that bridges to ESC

/** @} */ // end of UART Port Identifiers
