/**
 * @file AP_RangeFinder_Ainstein_LR_D1.h
 * @brief Ainstein LR-D1 FMCW radar rangefinder backend
 * 
 * @details This driver provides support for the Ainstein LR-D1 frequency-modulated 
 *          continuous wave (FMCW) radar sensor for distance measurement. The radar 
 *          operates over a serial UART interface at 115200 baud and provides 
 *          distance readings along with signal quality metrics.
 * 
 *          Key Features:
 *          - FMCW radar technology for distance measurement
 *          - Serial UART communication at 115200 baud
 *          - Signal quality reporting (0-100%)
 *          - Malfunction alert monitoring (temperature, voltage, saturation, altitude)
 *          - 32-byte fixed packet protocol
 * 
 * @warning Radar sensors have wider beam patterns than laser rangefinders.
 *          The sensor requires monitoring of temperature and voltage malfunction alerts.
 *          Saturation and altitude reading errors must be handled appropriately.
 * 
 * @note Developer notes on protocol implementation:
 *       - Malfunction codes on protocol page 22 may have documentation inconsistencies
 *       - Fixed packet length of 28 bytes (with 32-byte buffer) needs verification
 *       - SNR field definition is not fully documented in vendor specifications
 *       - Roll/pitch limit specifications conflict between sections 3.2 and other areas
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_Ainstein_LR_D1.h:1-63
 */

#pragma once

#include "AP_RangeFinder_config.h"
#if AP_RANGEFINDER_AINSTEIN_LR_D1_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_Ainstein_LR_D1
 * @brief Backend driver for Ainstein LR-D1 FMCW radar distance sensor
 * 
 * @details This class implements the serial protocol interface for the Ainstein LR-D1
 *          frequency-modulated continuous wave (FMCW) radar sensor. Unlike laser-based
 *          rangefinders, radar sensors use radio frequency waves to measure distance,
 *          providing advantages in certain weather conditions but with wider beam patterns.
 * 
 *          The driver handles:
 *          - Serial communication at 115200 baud over UART
 *          - Distance measurement extraction from radar frames
 *          - Signal quality assessment (0-100% or unknown)
 *          - Malfunction alert monitoring and reporting
 *          - Temperature, voltage, saturation, and altitude error detection
 * 
 *          Protocol Characteristics:
 *          - Fixed 32-byte packet size
 *          - Distance readings in meters (reading_m)
 *          - Signal quality percentage (0-100) or -1 for unknown
 *          - Malfunction alert bitmask for error conditions
 * 
 * @warning Radar sensors have significantly wider beam patterns than laser rangefinders,
 *          which may result in reflections from objects not directly in front of the sensor.
 *          Applications requiring tight beam patterns should use laser-based sensors.
 * 
 * @note This sensor requires continuous monitoring of malfunction alerts for safe operation.
 *       Temperature and voltage alerts indicate hardware issues requiring attention.
 *       Signal saturation may occur in high-reflectivity environments.
 * 
 * @see AP_RangeFinder_Backend_Serial for base serial communication implementation
 */
class AP_RangeFinder_Ainstein_LR_D1 : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create an Ainstein LR-D1 rangefinder backend instance
     * 
     * @details Creates a new AP_RangeFinder_Ainstein_LR_D1 object using dynamic memory
     *          allocation with NEW_NOTHROW for safe instantiation. This factory pattern
     *          allows the rangefinder system to create backend instances without knowing
     *          the specific derived class type.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing measurements
     * @param[in] _params Reference to rangefinder parameters for configuration
     * 
     * @return Pointer to new AP_RangeFinder_Backend_Serial instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW to ensure safe behavior on allocation failure
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Ainstein_LR_D1(_state, _params);
    }

protected:

    /**
     * @brief Get the MAVLink distance sensor type identifier for this sensor
     * 
     * @details Returns MAV_DISTANCE_SENSOR_RADAR to indicate this sensor uses radar
     *          technology rather than laser, ultrasonic, or infrared. This type
     *          identification is used in MAVLink telemetry messages to inform ground
     *          control stations about the sensor technology.
     * 
     * @return MAV_DISTANCE_SENSOR_RADAR constant indicating radar-based distance sensing
     * 
     * @see MAVLink common message set for MAV_DISTANCE_SENSOR enum definition
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    /**
     * @brief Get the initial baud rate for serial communication with the sensor
     * 
     * @details Returns the fixed baud rate of 115200 required by the Ainstein LR-D1
     *          serial protocol. This baud rate is set during UART initialization and
     *          must match the sensor's communication speed.
     * 
     * @param[in] serial_instance Serial port instance number (unused for this sensor)
     * 
     * @return 115200 - Fixed baud rate in bits per second for UART communication
     * 
     * @note The sensor does not support auto-baud detection or alternative baud rates.
     *       The serial port must be configured to 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit).
     */
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

private:

    // Inherit constructor from base class
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Extract a distance reading from the radar sensor data stream
     * 
     * @details Parses the 32-byte protocol packets from the Ainstein LR-D1 radar sensor
     *          to extract distance measurements. This method is called by the base class
     *          update() method to retrieve new sensor readings from the serial data stream.
     * 
     *          The method processes the serial data buffer, validates packet structure,
     *          extracts the distance value, updates signal quality metrics, and checks
     *          for malfunction alerts.
     * 
     * @param[out] reading_m Distance measurement in meters extracted from sensor packet
     * 
     * @return true if valid reading was successfully extracted, false if no valid data available
     * 
     * @note This method also updates internal state including signal_quality_pct and
     *       malfunction_alert tracking. Called at the main sensor update rate.
     * 
     * @see AP_RangeFinder_Backend_Serial::update() for the calling context
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Get the signal quality percentage for the most recent radar reading
     * 
     * @details Returns the signal quality metric from the radar sensor, providing
     *          an indication of measurement reliability. The quality metric is derived
     *          from the radar return signal characteristics.
     * 
     * @return Signal quality as percentage:
     *         - 0 to 100: Valid signal quality percentage (0=poor, 100=excellent)
     *         - -1 (RangeFinder::SIGNAL_QUALITY_UNKNOWN): Quality information not available
     * 
     * @note This method is called by the base class update() as a companion to get_reading().
     *       Signal quality below certain thresholds may indicate unreliable distance measurements.
     */
    int8_t get_signal_quality_pct() const override { return signal_quality_pct; };

    /**
     * @brief Report malfunction alerts detected by the radar sensor
     * 
     * @details Compares current malfunction alert status with previous status to detect
     *          state changes and reports new malfunction conditions to the ground control
     *          station. This method generates appropriate warning messages when malfunction
     *          alerts are triggered or cleared.
     * 
     *          Monitored conditions include:
     *          - Temperature out of operating range
     *          - Voltage supply issues
     *          - IF signal saturation (high reflectivity environments)
     *          - Altitude reading errors
     * 
     * @param[in] _malfunction_alert_ Current malfunction alert bitmask from sensor
     * @param[in] _malfunction_alert_prev_ Previous malfunction alert bitmask for comparison
     * 
     * @note This is a static method that uses GCS messaging to report alerts.
     *       Malfunction alerts should not be ignored as they may indicate hardware
     *       issues or environmental conditions that compromise measurement accuracy.
     * 
     * @warning Critical malfunctions (temperature, voltage) require immediate attention
     *          to prevent sensor damage or unsafe vehicle operation.
     */
    static void report_malfunction(const uint8_t _malfunction_alert_, const uint8_t _malfunction_alert_prev_);

    /**
     * @enum MalfunctionAlert
     * @brief Bitmask flags for Ainstein LR-D1 radar sensor malfunction conditions
     * 
     * @details Defines the malfunction alert bits reported by the Ainstein LR-D1 sensor.
     *          Multiple alerts can be active simultaneously as this is a bitmask.
     *          These alerts are extracted from the sensor's protocol packets and monitored
     *          continuously during operation.
     */
    enum class MalfunctionAlert : uint8_t {
        Temperature        = (1U << 0),  ///< 0x01 - Temperature outside operating range (hardware issue)
        Voltage            = (1U << 1),  ///< 0x02 - Supply voltage out of specification (power issue)
        IFSignalSaturation = (1U << 6),  ///< 0x40 - Intermediate frequency signal saturated (high reflectivity)
        AltitudeReading    = (1U << 7),  ///< 0x80 - Altitude reading error (measurement validity issue)
    };

    /// Fixed packet size for Ainstein LR-D1 serial protocol (32 bytes)
    static constexpr uint8_t PACKET_SIZE = 32;
    
    /// Previous malfunction alert bitmask for detecting state changes
    uint8_t malfunction_alert_prev;
    
    /// Timestamp (milliseconds) of last malfunction alert message sent to GCS
    uint32_t malfunction_alert_last_send_ms;
    
    /// Current signal quality percentage (0-100) or RangeFinder::SIGNAL_QUALITY_UNKNOWN (-1)
    int8_t signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;    
};
#endif