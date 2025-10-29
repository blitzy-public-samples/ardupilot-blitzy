/**
 * @file AP_RangeFinder_LightWareI2C.h
 * @brief LightWare I2C laser rangefinder backend driver
 * 
 * @details This driver provides support for LightWare laser rangefinders
 *          that communicate via the I2C protocol. It supports both legacy
 *          devices (SF10/SF11) and newer SF20 models with different
 *          communication protocols.
 *          
 *          Supported devices:
 *          - SF10: Laser altimeter with 50m range
 *          - SF11: Laser altimeter with 100m range
 *          - SF20: Advanced laser rangefinder with stream-based protocol
 *          
 *          The driver automatically detects the device type and initializes
 *          the appropriate communication protocol. Legacy devices use simple
 *          read operations, while SF20 devices use a more sophisticated
 *          stream-based protocol with configurable data outputs.
 *          
 *          I2C Communication:
 *          - Default I2C address: 0x66 (configurable on device)
 *          - Legacy protocol: Simple distance reads
 *          - SF20 protocol: Command-response with text-based streaming
 *          
 * @note LightWare devices are high-performance laser rangefinders commonly
 *       used for precision altitude measurement and obstacle detection
 * 
 * @see AP_RangeFinder_Backend
 * @see AP_HAL::I2CDevice
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LWI2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

/**
 * @brief Number of data streams supported for SF20 devices
 * 
 * @details The SF20 protocol supports multiple simultaneous data streams
 *          (distance, signal strength, temperature, etc.). This constant
 *          defines how many streams we actively monitor. Currently set to 1
 *          to read only the primary distance measurement for performance.
 */
#define NUM_SF20_DATA_STREAMS 1

/**
 * @class AP_RangeFinder_LightWareI2C
 * @brief Backend driver for LightWare I2C laser rangefinders
 * 
 * @details This class implements the rangefinder backend interface for
 *          LightWare laser rangefinders using I2C communication. It provides
 *          support for multiple device generations:
 *          
 *          Legacy Devices (SF10/SF11):
 *          - Simple I2C read protocol
 *          - Direct distance register reads
 *          - 100Hz update rate capability
 *          - Range: SF10 (50m), SF11 (100m)
 *          
 *          SF20 Devices:
 *          - Advanced stream-based protocol
 *          - Text command interface
 *          - Configurable data streams
 *          - Enhanced error detection
 *          - Stream recovery mechanisms
 *          
 *          Communication Protocol:
 *          The driver uses periodic timer callbacks to request and read
 *          distance measurements. Legacy devices use simple register reads,
 *          while SF20 devices use a command-response protocol with continuous
 *          data streaming.
 *          
 *          Initialization Sequence:
 *          1. Probe I2C device
 *          2. Detect device type (legacy vs SF20)
 *          3. Configure device parameters
 *          4. Register periodic update callback
 *          5. Begin distance measurements
 *          
 * @note Thread Safety: Update methods called from HAL scheduler thread,
 *       protected by I2C device semaphore
 * 
 * @warning Ensure I2C bus speed is configured correctly (typically 400kHz)
 *          for reliable communication with high-update-rate sensors
 */
class AP_RangeFinder_LightWareI2C : public AP_RangeFinder_Backend
{

public:
    /**
     * @brief Detect and initialize a LightWare I2C rangefinder
     * 
     * @details This static factory method attempts to detect a LightWare
     *          rangefinder on the provided I2C device. It probes the device,
     *          determines whether it's a legacy (SF10/SF11) or SF20 model,
     *          and initializes the appropriate communication protocol.
     *          
     *          Detection Process:
     *          1. Attempt I2C communication with device
     *          2. Try SF20 initialization (newer protocol)
     *          3. Fall back to legacy initialization if SF20 fails
     *          4. Configure device parameters
     *          5. Register periodic callback for updates
     *          
     *          The method returns nullptr if detection fails at any stage,
     *          allowing the rangefinder library to try other drivers or
     *          I2C addresses.
     * 
     * @param[in,out] _state     Rangefinder state structure to be updated
     * @param[in]     _params    Rangefinder configuration parameters
     * @param[in]     dev        I2C device interface (ownership transferred)
     * 
     * @return Pointer to initialized backend instance, or nullptr on failure
     * 
     * @note This method takes ownership of the I2C device pointer
     * @note Called during rangefinder initialization at boot time
     * 
     * @see init()
     * @see legacy_init()
     * @see sf20_init()
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Update rangefinder state with latest measurement
     * 
     * @details This method is called periodically by the rangefinder library
     *          to update the sensor state with the most recent distance
     *          measurement. It retrieves the latest reading from the internal
     *          buffer (populated by timer callbacks) and updates the state
     *          structure with distance, status, and signal quality information.
     *          
     *          The update frequency is independent of the sensor reading
     *          frequency - this method simply publishes the most recent
     *          measurement obtained by the timer callback.
     * 
     * @note Called from main thread at rangefinder library update rate
     * @note Actual sensor reads occur in timer callback (legacy_timer/sf20_timer)
     * 
     * @see legacy_timer()
     * @see sf20_timer()
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink sensor type classification for this
     *          rangefinder. LightWare devices are laser-based rangefinders
     *          and are reported as MAV_DISTANCE_SENSOR_LASER to ground
     *          control stations.
     *          
     *          This classification affects how ground stations display and
     *          interpret the sensor data, and may influence sensor fusion
     *          algorithms that treat different sensor types differently.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser rangefinder type
     * 
     * @note This is a compile-time constant return value
     * @see MAVLink DISTANCE_SENSOR message documentation
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    /**
     * @brief Buffer for SF20 data stream values
     * 
     * @details Stores the most recent values from SF20 data streams. The SF20
     *          protocol supports multiple simultaneous data streams (distance,
     *          signal strength, temperature, etc.). This array holds the
     *          current values for all configured streams.
     *          
     *          Currently only the primary distance stream is used
     *          (NUM_SF20_DATA_STREAMS = 1), but the infrastructure supports
     *          multiple streams for future expansion.
     * 
     * @note Units: meters for distance stream
     * @note Updated in sf20_timer() callback
     */
    float sf20_stream_val[NUM_SF20_DATA_STREAMS];
    
    /**
     * @brief Current index in SF20 stream sequence
     * 
     * @details Tracks which data stream is being processed in the current
     *          read cycle. Used for round-robin reading of multiple streams
     *          in the SF20 protocol. Initialized to 0 and incremented as
     *          streams are processed.
     * 
     * @note Only relevant for SF20 devices
     */
    int currentStreamSequenceIndex = 0;

    /**
     * @brief Construct a LightWare I2C rangefinder backend
     * 
     * @details Private constructor called by detect() factory method after
     *          successful device detection. Initializes the backend with
     *          the provided state, parameters, and I2C device interface.
     *          
     *          The constructor stores references and takes ownership of the
     *          I2C device, but does not perform device initialization -
     *          that is done by init() which is called after construction.
     * 
     * @param[in,out] _state   Rangefinder state structure
     * @param[in]     _params  Rangefinder configuration parameters
     * @param[in]     dev      I2C device interface (ownership transferred)
     * 
     * @note Constructor is private - use detect() to create instances
     * @see detect()
     */
    AP_RangeFinder_LightWareI2C(RangeFinder::RangeFinder_State &_state,
                                AP_RangeFinder_Params &_params,
                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Write byte array to I2C device
     * 
     * @details Performs a blocking I2C write operation to send a buffer of
     *          bytes to the rangefinder. Used for sending commands and
     *          configuration data to the device.
     *          
     *          The method handles I2C protocol details and error checking,
     *          returning success/failure status.
     * 
     * @param[in] write_buf_u8  Buffer containing bytes to write
     * @param[in] len_u8        Number of bytes to write
     * 
     * @return true if write successful, false on I2C error
     * 
     * @note Blocks until write completes or timeout occurs
     * @note Called with I2C semaphore already held
     */
    bool write_bytes(uint8_t *write_buf_u8, uint32_t len_u8);
    
    /**
     * @brief Disable address tagging on SF20 device
     * 
     * @details Configures the SF20 to disable I2C address tagging in its
     *          response packets. This simplifies response parsing by removing
     *          the I2C address prefix from data streams.
     *          
     *          This is called during SF20 initialization to establish a
     *          consistent communication format.
     * 
     * @note Only applicable to SF20 devices
     * @note Part of SF20 initialization sequence
     * 
     * @see sf20_init()
     */
    void sf20_disable_address_tagging();
    
    /**
     * @brief Send command and verify expected response
     * 
     * @details Sends a text command to the SF20 device and waits for a
     *          specific response string. This implements the command-response
     *          pattern used by SF20's text-based protocol.
     *          
     *          The method:
     *          1. Sends the command string via I2C
     *          2. Waits for response with timeout
     *          3. Compares response to expected string
     *          4. Returns success if match found
     *          
     *          Used extensively during SF20 initialization and configuration.
     * 
     * @param[in] send            Command string to send (null-terminated)
     * @param[in] expected_reply  Expected response string (null-terminated)
     * 
     * @return true if expected response received, false on timeout or mismatch
     * 
     * @note Includes timeout protection against non-responsive devices
     * @note Only applicable to SF20 devices
     * 
     * @see sf20_init()
     * @see sf20_wait_on_reply()
     */
    bool sf20_send_and_expect(const char* send, const char* expected_reply);
    
    /**
     * @brief Configure SF20 lost signal confirmation count
     * 
     * @details Sets the number of consecutive failed readings required before
     *          the SF20 reports a lost signal condition. This helps filter
     *          transient signal losses and prevents false error reporting.
     *          
     *          A higher confirmation count provides more stable operation but
     *          increases the time to detect actual signal loss. The value is
     *          configured during initialization for optimal balance.
     * 
     * @return true if configuration successful, false on communication error
     * 
     * @note Only applicable to SF20 devices
     * @note Part of SF20 initialization sequence
     * 
     * @see sf20_init()
     */
    bool sf20_set_lost_signal_confirmations();
    
    /**
     * @brief Query device version information
     * 
     * @details Sends a version query command to the SF20 and retrieves the
     *          version string response. Used during initialization to identify
     *          device capabilities and for diagnostic logging.
     *          
     *          The version string format varies by device model but typically
     *          includes hardware and firmware version numbers.
     * 
     * @param[in]  send_msg      Version query command string
     * @param[in]  reply_prefix  Expected prefix in version response
     * @param[out] reply         Buffer to store version string
     * @param[in]  reply_len     Size of reply buffer in bytes
     * 
     * @note Only applicable to SF20 devices
     * @note Reply buffer must be large enough for version string + null terminator
     * 
     * @see sf20_init()
     */
    void sf20_get_version(const char* send_msg, const char *reply_prefix, char *reply, uint8_t reply_len);
    /**
     * @brief Wait for and read two-byte response from SF20
     * 
     * @details Polls the I2C device until two bytes are available or a timeout
     *          occurs. Used as a primitive for implementing command-response
     *          protocols with the SF20.
     *          
     *          The method repeatedly attempts to read two bytes, returning
     *          when data is available or the timeout expires. This is used
     *          to synchronize with SF20's asynchronous response timing.
     * 
     * @param[out] rx_two_bytes  Buffer to store two received bytes
     * 
     * @return true if two bytes received, false on timeout
     * 
     * @note Includes timeout protection (implementation-defined duration)
     * @note Only applicable to SF20 devices
     * 
     * @see sf20_send_and_expect()
     */
    bool sf20_wait_on_reply(uint8_t *rx_two_bytes);
    
    /**
     * @brief Initialize the rangefinder device
     * 
     * @details Master initialization method that attempts to detect and
     *          initialize the connected LightWare device. This method:
     *          
     *          1. Attempts SF20 initialization first (newer protocol)
     *          2. Falls back to legacy initialization if SF20 fails
     *          3. Registers periodic timer callback for readings
     *          4. Performs initial configuration
     *          
     *          The method returns true only if initialization succeeds for
     *          at least one protocol type.
     * 
     * @return true if device initialized successfully, false on failure
     * 
     * @note Called once during detect() process
     * @note Determines device type and communication protocol
     * 
     * @see detect()
     * @see legacy_init()
     * @see sf20_init()
     */
    bool init();
    
    /**
     * @brief Initialize legacy LightWare devices (SF10/SF11)
     * 
     * @details Performs initialization specific to legacy LightWare devices
     *          using the simple read protocol. This includes:
     *          
     *          - Verifying I2C communication
     *          - Reading device identification
     *          - Configuring basic parameters
     *          - Registering periodic callback
     *          
     *          Legacy devices use direct register reads for distance values
     *          without the command-response protocol of SF20.
     * 
     * @return true if legacy device initialized successfully, false otherwise
     * 
     * @note Called from init() if SF20 initialization fails
     * @note Registers legacy_timer() as periodic callback
     * 
     * @see init()
     * @see legacy_timer()
     * @see legacy_get_reading()
     */
    bool legacy_init();
    
    /**
     * @brief Initialize SF20 devices
     * 
     * @details Performs initialization specific to SF20 devices using the
     *          advanced stream-based protocol. This includes:
     *          
     *          1. Verifying device communication
     *          2. Querying device version
     *          3. Disabling address tagging
     *          4. Configuring lost signal confirmations
     *          5. Setting up data streams
     *          6. Initializing stream recovery mechanism
     *          7. Registering periodic callback
     *          
     *          The SF20 protocol is more complex but provides enhanced
     *          features like multiple data streams and better error detection.
     * 
     * @return true if SF20 device initialized successfully, false otherwise
     * 
     * @note Called from init() before trying legacy initialization
     * @note Registers sf20_timer() as periodic callback
     * 
     * @see init()
     * @see sf20_timer()
     * @see sf20_get_reading()
     */
    bool sf20_init();
    
    /**
     * @brief Initialize SF20 stream recovery mechanism
     * 
     * @details Configures the stream recovery system used to resynchronize
     *          with SF20 data streams if communication is disrupted. This
     *          sets up state variables and timeout parameters for detecting
     *          and recovering from stream corruption.
     *          
     *          Stream recovery is essential for maintaining reliable readings
     *          when I2C bus noise or timing issues cause packet corruption.
     * 
     * @note Only applicable to SF20 devices
     * @note Called during sf20_init()
     * 
     * @see sf20_init()
     * @see sf20_timer()
     */
    void sf20_init_streamRecovery();
    
    /**
     * @brief Periodic timer callback for legacy devices
     * 
     * @details Called periodically by the HAL scheduler to read distance
     *          measurements from legacy (SF10/SF11) devices. This callback:
     *          
     *          1. Performs simple register read for distance
     *          2. Validates reading quality
     *          3. Updates internal distance buffer
     *          4. Sets error status if read fails
     *          
     *          Legacy devices use straightforward register reads, making this
     *          callback simpler than the SF20 equivalent.
     * 
     * @note Called from HAL scheduler thread at ~10Hz
     * @note Protected by I2C device semaphore
     * @note Only registered for legacy devices
     * 
     * @see legacy_init()
     * @see legacy_get_reading()
     */
    void legacy_timer();
    
    /**
     * @brief Periodic timer callback for SF20 devices
     * 
     * @details Called periodically by the HAL scheduler to read and process
     *          data streams from SF20 devices. This callback:
     *          
     *          1. Reads stream data from I2C
     *          2. Parses text-based stream protocol
     *          3. Extracts distance values
     *          4. Handles stream recovery if corruption detected
     *          5. Updates internal distance buffer
     *          6. Manages stream sequence indexing
     *          
     *          The SF20 protocol uses continuous streaming with text-based
     *          identifiers, requiring more complex parsing than legacy devices.
     * 
     * @note Called from HAL scheduler thread at ~10Hz
     * @note Protected by I2C device semaphore
     * @note Only registered for SF20 devices
     * 
     * @see sf20_init()
     * @see sf20_get_reading()
     * @see sf20_parse_stream()
     */
    void sf20_timer();

    /**
     * @brief Acquire distance reading from legacy device
     * 
     * @details Reads a distance measurement from a legacy (SF10/SF11) device
     *          using the simple register read protocol. This method:
     *          
     *          1. Reads distance register via I2C
     *          2. Converts raw value to meters
     *          3. Validates reading is within sensor range
     *          4. Returns reading via output parameter
     *          
     *          Legacy devices store the distance in a fixed register that
     *          can be read directly without command sequences.
     * 
     * @param[out] reading_m  Distance measurement in meters
     * 
     * @return true if valid reading obtained, false on error or out-of-range
     * 
     * @note Reading units: meters
     * @note Called from legacy_timer() callback
     * @note Only applicable to legacy (SF10/SF11) devices
     * 
     * @see legacy_timer()
     */
    bool legacy_get_reading(float &reading_m);
    
    /**
     * @brief Acquire distance reading from SF20 device
     * 
     * @details Reads and processes distance measurements from SF20 data
     *          streams. This method:
     *          
     *          1. Reads stream data from I2C
     *          2. Parses text-based protocol
     *          3. Extracts distance identifier and value
     *          4. Handles multi-stream data
     *          5. Validates reading quality
     *          6. Returns reading via output parameter
     *          
     *          The SF20 uses a streaming protocol where measurements are
     *          continuously transmitted with text identifiers.
     * 
     * @param[out] reading_m  Distance measurement in meters
     * 
     * @return true if valid reading obtained, false on parse error or invalid data
     * 
     * @note Reading units: meters
     * @note Called from sf20_timer() callback
     * @note Only applicable to SF20 devices
     * 
     * @see sf20_timer()
     * @see sf20_parse_stream()
     */
    bool sf20_get_reading(float &reading_m);
    
    /**
     * @brief Parse SF20 text-based data stream
     * 
     * @details Parses the SF20's text-based streaming protocol to extract
     *          named data values. The SF20 sends data in format like
     *          "Distance: 12.34\n" which must be parsed to extract the value.
     *          
     *          This method:
     *          1. Searches for string identifier in stream buffer
     *          2. Extracts numeric value following identifier
     *          3. Converts text to floating-point number
     *          4. Tracks number of characters processed
     *          5. Handles partial/incomplete packets
     *          
     *          The parser is designed to be resilient to stream corruption
     *          and can resynchronize if data is interrupted or garbled.
     * 
     * @param[in]     stream_buf             Buffer containing stream data
     * @param[in,out] p_num_processed_chars  Number of chars processed (updated)
     * @param[in]     string_identifier      Data identifier to search for
     * @param[out]    val                    Parsed numeric value
     * 
     * @return true if identifier found and value parsed, false otherwise
     * 
     * @note Handles partial packets across multiple reads
     * @note Robust to stream corruption and misalignment
     * @note Only applicable to SF20 devices
     * 
     * @see sf20_get_reading()
     * @see sf20_timer()
     */
    bool sf20_parse_stream(uint8_t *stream_buf,
                           size_t *p_num_processed_chars,
                           const char *string_identifier,
                           float &val);
    
    /**
     * @brief I2C device interface pointer
     * 
     * @details Owns the I2C device object used for all communication with
     *          the rangefinder. This pointer is initialized during detect()
     *          and remains valid for the lifetime of the backend instance.
     *          
     *          The device provides:
     *          - I2C read/write operations
     *          - Semaphore protection for thread safety
     *          - Bus address management
     *          - Transfer timeout handling
     *          
     *          All I2C operations go through this device interface to ensure
     *          proper synchronization and bus sharing with other devices.
     * 
     * @note Ownership transferred during construction
     * @note Automatically deleted when backend destroyed
     * 
     * @see AP_HAL::I2CDevice
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};

#endif  // AP_RANGEFINDER_LWI2C_ENABLED
