/**
 * @file AP_RangeFinder_Wasp.h
 * @brief Wasp 200 LRF rangefinder backend driver
 * 
 * @details This file implements the serial protocol driver for the Attollo Engineering
 *          Wasp 200 LRF (Laser Range Finder). The Wasp 200 is a compact laser rangefinder
 *          that communicates via serial interface at a fixed baudrate of 115200.
 * 
 *          Serial Protocol:
 *          - Baudrate: 115200 (fixed)
 *          - Data format: ASCII text responses
 *          - Configuration: Text-based commands for setting operating parameters
 *          - Output: Distance readings in meters
 * 
 *          Configuration Sequence:
 *          The driver performs a multi-stage initialization sequence to configure
 *          the sensor for optimal operation including protocol type, update frequency,
 *          filtering parameters (moving average, median filter), and sensitivity settings.
 * 
 * @note Device documentation: http://www.attolloengineering.com/wasp-200-lrf.html
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_WASP_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_Wasp
 * @brief Driver for Wasp 200 LRF laser rangefinder with serial interface
 * 
 * @details This backend implements the serial communication protocol for the Wasp 200
 *          Laser Range Finder manufactured by Attollo Engineering. The driver handles:
 * 
 *          - Serial communication at fixed 115200 baudrate
 *          - Multi-stage configuration sequence for sensor initialization
 *          - ASCII text-based command protocol for configuration
 *          - Distance reading parsing from serial responses
 *          - Configurable filtering (moving average, median filter)
 *          - Configurable sensitivity and update frequency
 * 
 *          Device Characteristics:
 *          - Sensor type: Laser rangefinder
 *          - Interface: Serial UART
 *          - Baudrate: 115200 (fixed, not configurable)
 *          - Protocol: ASCII text-based commands and responses
 *          - Encoding: LBE (Little/Big Endian configurable)
 *          - Output format: Distance in meters as ASCII text
 * 
 *          Configuration State Machine:
 *          The driver implements a sequential configuration process through multiple
 *          stages (WASP_CFG_*) to properly initialize the sensor with desired parameters
 *          for filtering, sensitivity, and update rate.
 * 
 * @note Inherits serial communication infrastructure from AP_RangeFinder_Backend_Serial
 * @warning Requires dedicated serial port at 115200 baud - baudrate cannot be changed
 * 
 * @see AP_RangeFinder_Backend_Serial
 * @see http://www.attolloengineering.com/wasp-200-lrf.html
 */
class AP_RangeFinder_Wasp : public AP_RangeFinder_Backend_Serial {

public:

    /**
     * @brief Factory method to create Wasp rangefinder backend instance
     * 
     * @details Creates a new AP_RangeFinder_Wasp object using dynamic memory allocation.
     *          This is the standard factory pattern used across ArduPilot rangefinder
     *          backends to enable runtime instantiation based on configuration.
     * 
     * @param[in,out] _state Reference to rangefinder state structure for storing measurements
     * @param[in,out] _params Reference to rangefinder parameters for configuration
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Uses NEW_NOTHROW for memory allocation to handle out-of-memory conditions gracefully
     * @see AP_RangeFinder_Backend_Serial::create pattern
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Wasp(_state, _params);
    }

    /**
     * @brief Update rangefinder state by reading and processing serial data
     * 
     * @details Called periodically by the rangefinder library to update sensor readings.
     *          This method handles both configuration commands and distance reading parsing
     *          depending on the current configuration state. The method manages the
     *          configuration state machine to initialize the sensor on first use.
     * 
     * @note Called at scheduler rate, typically 10-50Hz depending on system configuration
     * @warning Must complete quickly to avoid blocking the scheduler
     * 
     * @see get_reading() for distance measurement parsing
     * @see parse_response() for serial data processing
     */
    void update(void) override;

    /**
     * @brief Parameter table for Wasp-specific configuration
     * 
     * @details Defines user-configurable parameters for the Wasp rangefinder including:
     *          - Moving average filter window size
     *          - Median filter window size
     *          - Update frequency
     *          - Multi-pulse averages
     *          - Sensitivity threshold
     *          - Baudrate setting (fixed to 115200 for Wasp)
     * 
     * @note Parameters are exposed through the AP_Param system for ground station configuration
     * @see AP_Param::GroupInfo for parameter definition structure
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Get initial baudrate for serial communication
     * 
     * @details The Wasp 200 LRF operates at a fixed baudrate of 115200 and does not
     *          support baudrate negotiation. This method returns the fixed baudrate
     *          required for communication with the device.
     * 
     * @param[in] serial_instance Serial port instance number (unused for Wasp)
     * 
     * @return Always returns 115200 (fixed baudrate for Wasp 200 LRF)
     * 
     * @note Overrides AP_RangeFinder_Backend_Serial::initial_baudrate()
     * @note The baudrate cannot be changed - it is a hardware limitation of the Wasp 200
     */
    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 115200;
    }

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink sensor type classification for telemetry reporting.
     *          The Wasp 200 is a laser-based rangefinder and is identified as such
     *          in MAVLink DISTANCE_SENSOR messages.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser rangefinder type
     * 
     * @note Overrides AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     * @see MAVLink DISTANCE_SENSOR message definition
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    /**
     * @brief Private constructor for Wasp rangefinder backend
     * 
     * @details Constructs a Wasp rangefinder backend instance. This constructor is private
     *          and should only be called through the create() factory method to ensure
     *          proper memory management and error handling.
     * 
     * @param[in,out] _state Reference to rangefinder state structure
     * @param[in,out] _params Reference to rangefinder parameters
     * 
     * @note Only accessible through create() factory method
     * @see create() for public instantiation interface
     */
    AP_RangeFinder_Wasp(RangeFinder::RangeFinder_State &_state,
                        AP_RangeFinder_Params &_params);

    /**
     * @enum wasp_configuration_stage
     * @brief Configuration state machine stages for Wasp initialization
     * 
     * @details The Wasp 200 LRF requires a sequential configuration process using
     *          text-based serial commands. This enum defines the stages of the
     *          configuration state machine that sets up the sensor parameters.
     * 
     *          Configuration Sequence:
     *          1. WASP_CFG_RATE: Configure serial baudrate (fixed 115200)
     *          2. WASP_CFG_ENCODING: Set byte encoding (Little/Big Endian)
     *          3. WASP_CFG_PROTOCOL: Set communication protocol type
     *          4. WASP_CFG_FRQ: Set measurement update frequency
     *          5. WASP_CFG_GO: Start/resume distance measurements
     *          6. WASP_CFG_AUT: Configure auto sensitivity threshold mode
     *          7. WASP_CFG_THR: Set manual sensitivity threshold
     *          8. WASP_CFG_MAVG: Configure moving average filter window size
     *          9. WASP_CFG_MEDF: Set median filter window size
     *          10. WASP_CFG_AVG: Configure multi-pulse averaging
     *          11. WASP_CFG_AUV: Enforce automatic voltage adjustment
     *          12. WASP_CFG_DONE: Configuration complete, normal operation
     * 
     * @note Each stage sends specific command(s) to configure sensor parameters
     * @note State machine advances through update() calls as commands are acknowledged
     */
    enum wasp_configuration_stage {
        WASP_CFG_RATE,     ///< Set the baudrate (115200 fixed)
        WASP_CFG_ENCODING, ///< Set the encoding to LBE (Little/Big Endian)
        WASP_CFG_PROTOCOL, ///< Set the protocol type used
        WASP_CFG_FRQ,      ///< Set the update frequency (Hz)
        WASP_CFG_GO,       ///< Start/resume readings
        WASP_CFG_AUT,      ///< Set the auto sensitivity threshold mode
        WASP_CFG_THR,      ///< Set the sensitivity threshold value
        WASP_CFG_MAVG,     ///< Set the moving average filter window
        WASP_CFG_MEDF,     ///< Set the median filter window size
        WASP_CFG_AVG,      ///< Set the multi-pulse averages count
        WASP_CFG_AUV,      ///< Enforce auto voltage adjustment
        WASP_CFG_DONE      ///< Configuration complete, sensor operational
    };

    /**
     * @brief Current stage in configuration state machine
     * 
     * @details Tracks the current configuration stage during sensor initialization.
     *          Initialized to WASP_CFG_PROTOCOL and advances through each stage
     *          until reaching WASP_CFG_DONE when sensor is fully configured.
     * 
     * @note State advances in update() method as configuration commands complete
     * @see wasp_configuration_stage for stage definitions
     */
    wasp_configuration_stage configuration_state = WASP_CFG_PROTOCOL;

    /**
     * @brief Read distance measurement from sensor
     * 
     * @details Processes incoming serial data to extract distance readings from the
     *          Wasp 200 LRF. Parses ASCII text responses containing distance values
     *          in meters. This method is called by the update loop after configuration
     *          is complete (WASP_CFG_DONE state).
     * 
     * @param[out] reading_m Distance reading in meters
     * 
     * @return true if valid reading obtained, false if no data or parse error
     * 
     * @note Overrides AP_RangeFinder_Backend_Serial::get_reading()
     * @note Only processes readings when configuration_state == WASP_CFG_DONE
     * @see parse_response() for serial data parsing details
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Parse serial response from Wasp sensor
     * 
     * @details Reads characters from the serial port and assembles them into complete
     *          response lines in the linebuf buffer. Handles ASCII text protocol used
     *          by the Wasp 200 for both configuration acknowledgments and distance
     *          readings. Processes line-by-line with newline/carriage return delimiters.
     * 
     * @note Called from get_reading() to process incoming serial data
     * @note Uses linebuf and linebuf_len for line assembly
     * @warning Buffer overflow protection needed - linebuf is fixed size (10 bytes)
     */
    void parse_response(void);

    /**
     * @brief Line buffer for serial response assembly
     * 
     * @details Fixed-size character buffer for assembling ASCII text responses from
     *          the Wasp sensor. Responses are typically short distance values or
     *          configuration acknowledgments.
     * 
     * @note Buffer size: 10 characters (sufficient for distance readings like "12.34\r\n")
     * @see parse_response() for buffer usage
     */
    char linebuf[10];

    /**
     * @brief Current length of data in line buffer
     * 
     * @details Tracks number of valid characters currently stored in linebuf.
     *          Reset to 0 when line is complete (newline received) or buffer full.
     * 
     * @note Range: 0 to sizeof(linebuf)-1
     * @see parse_response() for buffer management
     */
    uint8_t linebuf_len;

    /**
     * @brief Moving average filter window size parameter
     * 
     * @details User-configurable parameter for moving average filter applied to
     *          distance readings. Larger values provide more smoothing but increase
     *          lag in response to distance changes. Set during WASP_CFG_MAVG stage.
     * 
     * @note Type: AP_Int16 (16-bit integer parameter)
     * @note Configured via ground station parameter interface
     * @see WASP_CFG_MAVG configuration stage
     */
    AP_Int16 mavg;

    /**
     * @brief Median filter window size parameter
     * 
     * @details User-configurable parameter for median filter window size applied to
     *          distance readings. Median filter helps reject outliers and noise spikes.
     *          Set during WASP_CFG_MEDF configuration stage.
     * 
     * @note Type: AP_Int16 (16-bit integer parameter)
     * @note Configured via ground station parameter interface
     * @see WASP_CFG_MEDF configuration stage
     */
    AP_Int16 medf;

    /**
     * @brief Update frequency parameter (Hz)
     * 
     * @details User-configurable measurement update frequency in Hertz. Controls how
     *          often the Wasp sensor generates new distance measurements. Higher
     *          frequencies provide faster updates but may reduce effective range.
     *          Set during WASP_CFG_FRQ configuration stage.
     * 
     * @note Type: AP_Int16 (16-bit integer parameter)
     * @note Units: Hz (measurements per second)
     * @note Configured via ground station parameter interface
     * @see WASP_CFG_FRQ configuration stage
     */
    AP_Int16 frq;

    /**
     * @brief Multi-pulse averages count parameter
     * 
     * @details User-configurable number of laser pulses to average for each measurement.
     *          Multiple pulses can improve accuracy and reduce noise at the cost of
     *          slightly lower update rate. Set during WASP_CFG_AVG configuration stage.
     * 
     * @note Type: AP_Int16 (16-bit integer parameter)
     * @note Configured via ground station parameter interface
     * @see WASP_CFG_AVG configuration stage
     */
    AP_Int16 avg;

    /**
     * @brief Sensitivity threshold parameter
     * 
     * @details User-configurable sensitivity threshold for detection. Lower values
     *          increase sensitivity (detect weaker returns) but may increase noise.
     *          Higher values reduce sensitivity but provide more reliable detection
     *          of strong targets. Set during WASP_CFG_THR configuration stage.
     * 
     * @note Type: AP_Int16 (16-bit integer parameter)
     * @note Configured via ground station parameter interface
     * @note Interacts with WASP_CFG_AUT (auto sensitivity) setting
     * @see WASP_CFG_THR and WASP_CFG_AUT configuration stages
     */
    AP_Int16 thr;

    /**
     * @brief Baudrate setting parameter
     * 
     * @details Baudrate configuration parameter. For Wasp 200 LRF, this is fixed at
     *          115200 and cannot be changed. Parameter exists for consistency with
     *          other serial rangefinder backends.
     * 
     * @note Type: AP_Int8 (8-bit integer parameter)
     * @note Wasp always uses 115200 baud - this parameter has no effect
     * @warning Changing this parameter will NOT change the actual baudrate
     * @see initial_baudrate() which always returns 115200
     */
    AP_Int8  baud;
};

#endif
