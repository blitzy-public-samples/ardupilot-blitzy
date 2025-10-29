/**
 * @file AP_RangeFinder_LightWareSerial.h
 * @brief LightWare Serial LiDAR rangefinder backend driver
 * 
 * This file implements the ArduPilot backend driver for LightWare LiDAR rangefinders
 * that communicate via serial protocol. LightWare produces high-performance laser
 * rangefinders with ranges exceeding 100 meters, suitable for terrain following,
 * obstacle avoidance, and precision landing applications.
 * 
 * Supported Models:
 * - SF20/C: Compact LiDAR with 100m range
 * - SF40C: Scanning LiDAR with 360° coverage and 100m range
 * - SF45B: Microlidar with 50m range and lightweight design
 * - Legacy LW20: Earlier generation serial LiDAR
 * 
 * Protocol Support:
 * The driver automatically detects and switches between two protocol modes:
 * - Legacy Protocol: ASCII string distances (e.g., "12.34\r\n")
 * - Binary Protocol: Two-byte binary distances for higher update rates
 * 
 * Scanning LiDAR Support:
 * For scanning models like SF40C, this driver retrieves the primary forward-facing
 * distance measurement. Multi-point scan data can be accessed through the
 * proximity sensor interface in AP_Proximity_LightWareSerial.
 * 
 * @note Binary protocol provides faster update rates and is preferred for
 *       real-time flight control applications
 * @warning Ensure correct serial port baud rate configuration (typically 115200 or 921600)
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_LightWareSerial
 * @brief Backend driver for LightWare Serial LiDAR rangefinders
 * 
 * @details This class implements the serial communication protocol for LightWare
 *          laser rangefinders including SF20/C, SF40C, and SF45B models. The driver
 *          provides automatic protocol detection, switching between legacy ASCII
 *          string format and high-speed binary format depending on sensor configuration.
 *          
 *          Key Features:
 *          - Automatic protocol detection (legacy vs binary)
 *          - Signal quality reporting based on valid reading status
 *          - Lost signal detection for out-of-range conditions
 *          - Scanning LiDAR support (primary distance reading)
 *          - 100m+ range capability on supported models
 *          
 *          Initialization Sequence:
 *          1. Driver opens configured serial port
 *          2. Protocol state starts as UNKNOWN
 *          3. Incoming data analyzed to detect protocol type
 *          4. Once detected, driver locks to that protocol
 *          5. Distance readings processed according to protocol
 *          
 *          Thread Safety:
 *          This class is called from the scheduler thread during rangefinder updates.
 *          Serial communication is handled through the HAL UART driver which provides
 *          thread-safe buffering.
 *          
 * @note The LW20 model requires a brief initialization period to switch from
 *       I2C mode to serial mode on power-up
 * @warning Incorrect baud rate will prevent protocol detection and result in no readings
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h
 */
class AP_RangeFinder_LightWareSerial : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create a LightWare Serial rangefinder backend instance
     * 
     * @details This static factory method instantiates a new LightWare Serial backend
     *          driver using dynamic memory allocation. The method is called by the
     *          rangefinder driver manager when a LightWare Serial rangefinder is
     *          configured via parameters.
     *          
     *          The NEW_NOTHROW allocator is used to safely handle memory allocation
     *          failures, returning nullptr if heap memory is unavailable.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing measurements
     * @param[in] _params Reference to rangefinder parameters (serial port, orientation, etc.)
     * 
     * @return Pointer to new AP_RangeFinder_LightWareSerial instance, or nullptr on allocation failure
     * 
     * @note This method is called during rangefinder initialization at boot time
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h:15-19
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_LightWareSerial(_state, _params);
    }

protected:

    /**
     * @brief Inherit base class constructor
     * 
     * @details Uses the parent AP_RangeFinder_Backend_Serial constructor to initialize
     *          common serial backend state including UART port configuration.
     */
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Report the MAVLink distance sensor type for this rangefinder
     * 
     * @details Returns the MAVLink enumeration value identifying this as a laser-based
     *          distance sensor. This information is used in MAVLink DISTANCE_SENSOR
     *          messages sent to ground control stations for proper sensor identification.
     *          
     *          LightWare sensors use laser/LiDAR technology (time-of-flight measurement)
     *          as opposed to ultrasonic, infrared, or radar ranging methods.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser/LiDAR technology
     * 
     * @note This is a const method called during telemetry message generation
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h:25-27
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

    /**
     * @brief Get signal quality as a percentage (0-100%)
     * 
     * @details Reports the quality of the rangefinder signal based on whether a valid
     *          reading is currently available. LightWare sensors provide binary signal
     *          quality status:
     *          - SIGNAL_QUALITY_MAX (100%): Valid distance reading within sensor range
     *          - SIGNAL_QUALITY_MIN (0%): Lost signal or out-of-range condition
     *          
     *          Signal loss occurs when:
     *          - Target is beyond maximum sensor range (>100m for SF20/SF40C)
     *          - No reflective target in sensor field of view
     *          - Sensor reports special "lost signal" distance value
     *          - Communication errors on serial link
     *          
     *          This signal quality is reported to the flight controller for failsafe
     *          decisions and is included in MAVLink telemetry.
     * 
     * @return Signal quality percentage: 0% (no signal) or 100% (valid signal)
     * 
     * @note Unlike some sensors with gradual signal strength, LightWare provides
     *       binary signal status only
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h:29-31
     */
    int8_t get_signal_quality_pct() const override {
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_MAX;
    }

private:
    /**
     * @brief Read and parse a distance measurement from the LightWare sensor
     * 
     * @details This method is called periodically by the rangefinder scheduler to retrieve
     *          the latest distance reading from the sensor. It handles both legacy ASCII
     *          string protocol and binary protocol formats with automatic protocol detection.
     *          
     *          Protocol Detection Algorithm:
     *          1. On startup, protocol_state is UNKNOWN
     *          2. Incoming bytes are tested against both protocol parsers
     *          3. Each successful parse increments that protocol's valid count
     *          4. First protocol to reach threshold becomes active protocol
     *          5. Once detected, only that protocol is used
     *          
     *          Legacy Protocol Format:
     *          - ASCII string: "12.34\r\n" (distance in meters with CR+LF terminator)
     *          - Parsed into linebuf character array
     *          - Converted to float using string parsing
     *          
     *          Binary Protocol Format:
     *          - Two bytes: high byte followed by low byte
     *          - Distance in centimeters: distance_cm = (high_byte << 8) | low_byte
     *          - Provides faster update rate than legacy protocol
     *          
     *          Lost Signal Detection:
     *          Special distance values indicate sensor cannot measure valid distance.
     *          These are checked via is_lost_signal_distance() and set no_signal flag.
     *          
     *          Update Rate:
     *          Typically called at 10-20Hz by the rangefinder scheduler task.
     * 
     * @param[out] reading_m Distance measurement in meters (updated only on success)
     * 
     * @return true if valid distance reading obtained and parsed successfully
     * @return false if no data available, parse error, or lost signal condition
     * 
     * @note This method processes all available serial data each call to minimize latency
     * @warning Returns false on lost signal to prevent invalid distances from being used
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h:35
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Check if a distance value indicates lost signal condition
     * 
     * @details LightWare sensors report special distance values when they cannot measure
     *          a valid distance (target out of range, no reflective surface, etc.).
     *          This method identifies these sentinel values to distinguish between
     *          valid measurements and error conditions.
     *          
     *          Lost Signal Conditions:
     *          - Distance at or beyond maximum sensor range
     *          - Specific manufacturer-defined "no target" values
     *          - Prevents invalid distances from being used in flight control
     *          
     *          The no_signal member variable is updated based on this check and
     *          affects the signal quality reporting to the flight controller.
     * 
     * @param[in] distance_cm Measured distance in centimeters from sensor
     * @param[in] distance_cm_max Maximum valid range of sensor in centimeters
     * 
     * @return true if distance indicates lost signal (invalid measurement)
     * @return false if distance is within valid measurement range
     * 
     * @note Different LightWare models may use different sentinel values
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h:36
     */
    bool is_lost_signal_distance(int16_t distance_cm, int16_t distance_cm_max);

    /**
     * @brief Buffer for legacy ASCII protocol distance strings
     * 
     * @details Stores incoming characters for legacy protocol parsing. The buffer
     *          accumulates characters until a complete distance string with terminator
     *          is received (e.g., "12.34\r\n"). Maximum size accommodates typical
     *          distance strings plus terminator characters.
     *          
     *          Format: "ddd.dd\r\n" where d is a digit (up to 3 digits before decimal)
     */
    char linebuf[10];

    /**
     * @brief Current length of data in legacy protocol buffer
     * 
     * @details Tracks number of valid characters currently stored in linebuf.
     *          Reset to 0 after complete distance string is parsed or on buffer overflow.
     *          Used to detect complete messages and prevent buffer overruns.
     */
    uint8_t linebuf_len;

    /**
     * @brief Timestamp of last initialization attempt in milliseconds
     * 
     * @details Used specifically for LW20 model initialization sequence. The LW20
     *          sensor can operate in I2C or serial mode and requires a brief period
     *          after power-up to switch to serial mode when serial communication is
     *          detected. This timestamp tracks the initialization timing.
     *          
     *          Typical initialization period: 100-500ms after first serial communication.
     */
    uint32_t last_init_ms;

    /**
     * @brief Most significant byte of binary protocol distance measurement
     * 
     * @details In binary protocol mode, distance is transmitted as two consecutive bytes:
     *          - First byte: high_byte (bits 15-8 of distance in centimeters)
     *          - Second byte: low byte (bits 7-0 of distance in centimeters)
     *          
     *          This variable stores the high byte until the low byte arrives, then
     *          the complete 16-bit distance is reconstructed:
     *          distance_cm = (high_byte << 8) | low_byte
     */
    uint8_t high_byte;

    /**
     * @brief Flag indicating high byte has been received and is waiting for low byte
     * 
     * @details Tracks the state of binary protocol parsing. When true, the next byte
     *          received should be the low byte of the distance measurement. When false,
     *          the next byte should be a high byte.
     *          
     *          This flag prevents byte synchronization errors in the binary protocol
     *          stream and ensures proper distance reconstruction.
     */
    bool high_byte_received;

    /**
     * @enum ProtocolState
     * @brief State machine for automatic protocol detection and selection
     * 
     * @details LightWare sensors can be configured to output distances in either
     *          legacy ASCII string format or binary format. This driver automatically
     *          detects which protocol the sensor is using by analyzing incoming data.
     *          
     *          Detection Process:
     *          - Starts in UNKNOWN state
     *          - Attempts to parse incoming data as both protocols simultaneously
     *          - Counts successful parses for each protocol type
     *          - First protocol to exceed validation threshold becomes active
     *          - State locks to detected protocol for remainder of session
     *          
     *          The binary protocol is preferred for flight applications due to:
     *          - Higher update rate (less overhead)
     *          - More efficient parsing
     *          - Reduced serial bandwidth usage
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.h:45-49
     */
    enum class ProtocolState {
        UNKNOWN,    ///< Protocol not yet determined - testing both legacy and binary formats
        LEGACY,     ///< Legacy ASCII protocol active - distances as string: "12.34\r\n"
        BINARY      ///< Binary protocol active - distances as two bytes (high, low) in centimeters
    } protocol_state;

    /**
     * @brief Count of successfully parsed legacy protocol messages
     * 
     * @details Incremented each time a valid legacy ASCII distance string is successfully
     *          parsed during the UNKNOWN protocol state. Once this counter exceeds the
     *          detection threshold (typically 3-5 valid messages), the driver locks to
     *          legacy protocol mode.
     *          
     *          This prevents false protocol detection from noise or corrupt data.
     */
    uint8_t legacy_valid_count;

    /**
     * @brief Count of successfully parsed binary protocol messages
     * 
     * @details Incremented each time a valid binary distance measurement is successfully
     *          reconstructed during the UNKNOWN protocol state. Once this counter exceeds
     *          the detection threshold (typically 3-5 valid messages), the driver locks
     *          to binary protocol mode.
     *          
     *          Binary protocol typically locks faster than legacy due to higher message rate.
     */
    uint8_t binary_valid_count;

    /**
     * @brief Signal status flag indicating lost or invalid signal condition
     * 
     * @details Set to true when the sensor reports a lost signal condition (target out
     *          of range, no reflective surface detected, etc.). This flag directly affects
     *          signal quality reporting via get_signal_quality_pct().
     *          
     *          When true:
     *          - Signal quality reported as 0% (SIGNAL_QUALITY_MIN)
     *          - Distance reading marked as unhealthy
     *          - Flight controller may trigger rangefinder failsafe actions
     *          
     *          When false:
     *          - Signal quality reported as 100% (SIGNAL_QUALITY_MAX)
     *          - Distance reading considered valid for flight control
     *          
     *          This flag is updated on each call to get_reading() based on the
     *          distance value received from the sensor.
     */
    bool no_signal = false;
};

#endif  // AP_RANGEFINDER_LIGHTWARE_SERIAL_ENABLED
