/**
 * @file AP_RangeFinder_Backend_Serial.h
 * @brief Serial rangefinder backend base class for UART-based distance sensors
 * 
 * @details This file defines the base class for all rangefinder backends that communicate
 *          via UART serial ports. It provides the infrastructure for serial port initialization,
 *          baudrate configuration, buffer sizing, and basic update patterns for sensors that
 *          transmit distance measurements over serial interfaces.
 *          
 *          Serial rangefinders integrate with ArduPilot's AP_SerialManager system to acquire
 *          and configure UART hardware ports. Derived classes implement sensor-specific
 *          protocols for parsing distance data from serial frames.
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.h:1-47
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ENABLED

#include "AP_RangeFinder_Backend.h"

/**
 * @class AP_RangeFinder_Backend_Serial
 * @brief Base class for rangefinder backends that communicate via UART serial ports
 * 
 * @details This class provides common functionality for all serial-based rangefinder sensors,
 *          including UART port initialization via AP_SerialManager, baudrate configuration,
 *          and buffer size management. Derived classes implement sensor-specific protocol
 *          parsing to extract distance measurements from serial data streams.
 *          
 *          The typical usage pattern for serial rangefinder backends:
 *          1. init_serial() binds to an AP_SerialManager UART port and configures baudrate/buffers
 *          2. update() is called periodically by the scheduler (base class or override)
 *          3. update() calls uart->read() to retrieve incoming serial data
 *          4. Derived class parses sensor-specific protocol frames
 *          5. get_reading() extracts distance measurement from parsed frame
 *          6. get_signal_quality_pct() optionally reports signal quality
 *          
 *          Integration with AP_SerialManager ensures proper serial port allocation and
 *          prevents conflicts with other serial protocols (GPS, telemetry, etc.).
 * 
 * @note Not all serial backends use the base-class update() method; some implement
 *       custom update() logic directly for specialized protocol requirements.
 * 
 * @warning Only one backend can use a given serial port instance. Serial port contention
 *          will cause undefined behavior. AP_SerialManager handles port allocation to prevent conflicts.
 */
class AP_RangeFinder_Backend_Serial : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Construct a serial rangefinder backend
     * 
     * @details Initializes the serial rangefinder backend with state and parameter references.
     *          The constructor stores references to the rangefinder state structure and
     *          configuration parameters but does not initialize the UART port. Serial port
     *          initialization is deferred to init_serial() to allow AP_SerialManager to
     *          allocate and configure the appropriate UART hardware.
     * 
     * @param[in,out] _state    Reference to rangefinder state structure for distance/status updates
     * @param[in]     _params   Reference to rangefinder configuration parameters
     */
    AP_RangeFinder_Backend_Serial(RangeFinder::RangeFinder_State &_state,
                                  AP_RangeFinder_Params &_params);

    /**
     * @brief Initialize and bind to a serial port via AP_SerialManager
     * 
     * @details Configures the UART port for this rangefinder by requesting allocation from
     *          AP_SerialManager using the specified serial instance number. This method:
     *          - Acquires a UART port handle from AP_SerialManager
     *          - Configures the port baudrate via initial_baudrate()
     *          - Sets RX/TX buffer sizes via rx_bufsize() and tx_bufsize()
     *          - Stores the UART driver pointer for subsequent I/O operations
     *          
     *          After successful initialization, the uart pointer is valid and can be used
     *          for serial read/write operations in update().
     * 
     * @param[in] serial_instance  AP_SerialManager serial port instance number (0-based index)
     *                             identifying which configured serial port to use
     */
    void init_serial(uint8_t serial_instance) override;

protected:

    /**
     * @brief Get the initial baudrate for serial port configuration
     * 
     * @details Returns the baudrate to configure the UART port during init_serial().
     *          Derived classes override this method to specify sensor-specific baudrates
     *          appropriate for their communication protocol requirements.
     *          
     *          Common sensor baudrates:
     *          - Simple sensors: 9600 bits/sec (e.g., NMEA-based rangefinders)
     *          - High-speed LiDAR: 115200 bits/sec or higher
     *          - Legacy sensors: 19200, 38400, 57600 bits/sec
     * 
     * @param[in] serial_instance  AP_SerialManager serial port instance number
     * 
     * @return uint32_t  Baudrate in bits per second (e.g., 9600, 115200)
     * 
     * @note Backends must override this method to specify correct baudrate for their sensor
     */
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    /**
     * @brief Get the receive buffer size for the UART port
     * 
     * @details Returns the desired receive buffer size in bytes for the UART driver.
     *          The default value of 0 is special to the UARTDriver - it means "use default"
     *          buffer size (typically 128 bytes for most HAL implementations).
     *          
     *          Backends can override this method to request larger buffers for sensors that:
     *          - Transmit large data frames (multi-byte protocol packets)
     *          - Operate at high baudrates requiring increased buffering
     *          - Have bursty transmission patterns
     * 
     * @return uint16_t  Receive buffer size in bytes, or 0 to use AP_HAL default
     * 
     * @note High-speed LiDAR sensors may require buffers of 256-512 bytes or larger
     */
    virtual uint16_t rx_bufsize() const { return 0; }

    /**
     * @brief Get the transmit buffer size for the UART port
     * 
     * @details Returns the desired transmit buffer size in bytes for the UART driver.
     *          The default value of 0 is special to the UARTDriver - it means "use default"
     *          buffer size (typically 128 bytes for most HAL implementations).
     *          
     *          Most rangefinder sensors are receive-only or require minimal transmit capability,
     *          so the default buffer size is usually sufficient.
     * 
     * @return uint16_t  Transmit buffer size in bytes, or 0 to use AP_HAL default
     * 
     * @note Override this method if sensor requires command transmission or configuration
     */
    virtual uint16_t tx_bufsize() const { return 0; }

    /**
     * @brief UART driver pointer for serial I/O operations
     * 
     * @details Pointer to the AP_HAL::UARTDriver instance allocated by AP_SerialManager during
     *          init_serial(). This pointer is used for all serial communication operations:
     *          - uart->read() - Read bytes from receive buffer
     *          - uart->write() - Write bytes to transmit buffer
     *          - uart->available() - Check number of bytes available to read
     *          - uart->txspace() - Check available transmit buffer space
     *          
     *          The pointer is nullptr until init_serial() successfully allocates a port.
     *          Backends should check for nullptr before attempting I/O operations.
     */
    AP_HAL::UARTDriver *uart = nullptr;

    /**
     * @brief Base-class update implementation for serial rangefinder backends
     * 
     * @details Provides a standard update pattern for serial rangefinders that use the
     *          get_reading() and get_signal_quality_pct() interface. This method is called
     *          periodically by the scheduler (typically at 10-50 Hz) and performs:
     *          1. Calls get_reading() to retrieve the latest distance measurement
     *          2. Calls get_signal_quality_pct() to retrieve signal quality if available
     *          3. Updates rangefinder state with new distance and status
     *          4. Manages timeout detection via read_timeout_ms()
     *          
     *          The base-class update() implementation is sufficient for many simple sensors
     *          that follow the parse-on-demand pattern. More complex sensors may override
     *          update() entirely to implement custom buffering or protocol state machines.
     * 
     * @note Not all backends use this base-class update() method. Some sensors implement
     *       custom update() logic directly for specialized protocol requirements or to
     *       handle complex multi-stage communication sequences.
     */
    virtual void update(void) override;

    /**
     * @brief Parse sensor data and extract distance measurement (pure virtual)
     * 
     * @details Derived classes must implement this method to parse sensor-specific protocol
     *          frames and extract distance measurements. This method is called by the base-class
     *          update() implementation to retrieve the latest reading.
     *          
     *          Typical implementation pattern:
     *          1. Read available bytes from uart into parsing buffer
     *          2. Search for protocol frame start/sync bytes
     *          3. Validate frame checksum or CRC
     *          4. Extract distance field from frame
     *          5. Convert to meters and return via reading_m parameter
     *          
     *          The method should return true only when a new, valid measurement is available.
     *          Returning false indicates no new data or invalid data (failed checksum, out-of-range, etc.).
     * 
     * @param[out] reading_m  Distance measurement in meters (output parameter)
     * 
     * @return bool  true if new valid measurement available and written to reading_m,
     *               false if no new data or data invalid
     * 
     * @note It is essential that any backend relying on the base-class update() implement this method
     */
    virtual bool get_reading(float &reading_m) = 0;

    /**
     * @brief Get signal quality percentage for the current reading
     * 
     * @details Returns signal quality as a percentage (0-100) or -1 if unknown/unsupported.
     *          This method is a companion to get_reading() and is called by the base-class
     *          update() implementation to report signal strength or measurement confidence.
     *          
     *          Signal quality interpretation (sensor-dependent):
     *          - 0: Very weak signal or poor measurement confidence
     *          - 50: Moderate signal quality
     *          - 100: Excellent signal quality
     *          - -1 (SIGNAL_QUALITY_UNKNOWN): Sensor does not report quality metric
     *          
     *          Many simple rangefinders do not provide signal quality information and should
     *          return the default value of -1.
     * 
     * @return int8_t  Signal quality percentage (0-100) or -1 for unknown/unsupported
     * 
     * @note This method is called by the base-class update() after get_reading()
     */
    virtual int8_t get_signal_quality_pct() const WARN_IF_UNUSED
    { return RangeFinder::SIGNAL_QUALITY_UNKNOWN; }

    /**
     * @brief Get the read timeout for sensor data
     * 
     * @details Returns the maximum time in milliseconds between valid readings before the
     *          rangefinder transitions to NoData status. This timeout is used by the base-class
     *          update() to detect sensor disconnection or communication failures.
     *          
     *          The default timeout of 200ms is appropriate for sensors that update at 5-10 Hz.
     *          Faster sensors may use shorter timeouts (50-100ms), while slower sensors may
     *          require longer timeouts (500-1000ms).
     *          
     *          When no valid reading is received within this timeout period, the rangefinder
     *          status changes to NoData, alerting the flight controller that distance
     *          measurements are stale or unavailable.
     * 
     * @return uint16_t  Timeout in milliseconds (default: 200ms)
     * 
     * @note Override this method if sensor has update rate significantly different from 5-10 Hz
     */
    virtual uint16_t read_timeout_ms() const { return 200; }
};

#endif  // AP_RANGEFINDER_ENABLED
