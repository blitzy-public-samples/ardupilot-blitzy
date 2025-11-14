/**
 * @file AP_RangeFinder_JRE_Serial.h
 * @brief JRE Serial radar rangefinder backend driver
 * 
 * @details This driver implements support for JRE Serial radar altimeters, which
 *          provide distance measurements over a serial interface using a proprietary
 *          protocol. The rangefinder supports multiple measurement modes (1, 3, or 5
 *          data points per frame) with variable packet lengths (16, 32, or 48 bytes).
 *          
 *          The driver extracts distance measurements from the protocol frames and
 *          provides signal quality indication based on tracking status.
 * 
 * @note This is a radar-based distance sensor, suitable for altitude measurement
 * @see AP_RangeFinder_Backend_Serial for base serial rangefinder functionality
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_JRE_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @defgroup JRE_Serial_Protocol JRE Serial Protocol Documentation
 * @{
 * 
 * @brief JRE Serial radar altimeter protocol specification
 * 
 * @details The data received from the radio altimeter varies depending on the
 *          measurement mode configuration. Frames are stored with fixed lengths:
 *          - 1 data mode: 16 bytes
 *          - 3 data mode: 32 bytes
 *          - 5 data mode: 48 bytes
 *          
 *          All modes use bytes 4-5 for the primary altitude reading.
 *          Distance values are unsigned 16-bit integers with LSB = 0.01m (centimeters).
 *          Each frame includes header identification, version, frame count, FFT signal
 *          strength values, status bits, and CRC validation.

[1]
Measurement mode: 1 data mode
Packet length: 16 bytes
Altitude data used: 4,5 bytes
|----------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE |   0    |   1    |    2     |    3     |    4     |    5     | 6 | 7 | 8 | 9 |     10    |     11    |    12     |    13     |  14   |  15   |
|------|---------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | header | header | version  |  frame   | altitude | altitude |    reserved   | FFT value | FFT value |  status   |  status   |  CRC  |  CRC  |
|      |  (H)   |  (L)   |          |  count   |    (H)   |    (L)   |               |    (H)    |    (L)    |   (H)     |   (L)     |  (L)  |  (H)  |
|------|---------------------------------------------------------------------------------------------------------------------------------------------|
|      |        |        |          |          |                     |               |                       | BIT 15to8: reserved   |               |
|      |        |        |          | unsigned |       unsigned      |               |       unsigned        | BIT 7to4: GAIN detail |   CRC result  |
| DATA |  'R'   |  'A'   | 0 to 255 | 0 to 255 |      LSB;0.01 m     |      N/A      |        LSB:1          | BIT 3-2: reserved     |   from BYTE   |
|      |        |        |          |          |      0 to 65535     |               |      0 to 65535       | BIT 1: TRK / NTRK     |    0 to 13    |
|      |        |        |          |          |                     |               |                       | BIT 0: GAIN LOW/HIGH  |               |
|----------------------------------------------------------------------------------------------------------------------------------------------------|

[2]
Measurement mode: 3 data mode
Packet length: 32 bytes
Altitude data used: 4,5 bytes
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE |   0    |   1    |    2     |    3     |    4     |    5     | 6 | 7 | 8 | 9 |     10    |     11    |    12    |    13    | 14| 15| 16| 17|     18    |     19    |    20    |    21    | 22| 23| 24| 25|     26    |     27    |    28     |    29     |  30   |  31   |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | header | header | version  |  frame   | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value |  status   |  status   |  CRC  |  CRC  |
|      |  (H)   |  (L)   |          |  count   |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)    |    (L)    |  (L)  |  (H)  |
|------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       | BIT 15to8: reserved   |               |
|      |        |        |          | unsigned |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        | BIT 7to4: GAIN detail |   CRC result  |
| DATA |  'R'   |  'B'   | 0 to 255 | 0 to 255 |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          | BIT 3-2: reserved     |   from BYTE   |
|      |        |        |          |          |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       | BIT 1: TRK / NTRK     |    0 to 13    |
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       | BIT 0: GAIN LOW/HIGH  |               |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|

[3]
Measurement mode: 5 data mode
Packet length: 48 bytes
Altitude data used: 4,5 bytes
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| BYTE |   0    |   1    |    2     |    3     |    4     |    5     | 6 | 7 | 8 | 9 |     10    |     11    |    12    |    13    | 14| 15| 16| 17|     18    |     19    |    20    |    21    | 22| 23| 24| 25|     26    |     27    |    28    |    29    | 30| 31| 32| 33|     34    |     35    |    36    |    37    | 38| 39| 40| 41|     42    |     43    |    44     |    45     |  46   |  47   |
|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| NAME | header | header | version  |  frame   | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value | altitude | altitude |    reserved   | FFT value | FFT value |  status   |  status   |  CRC  |  CRC  |
|      |  (H)   |  (L)   |          |  count   |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)   |    (L)   |               |    (H)    |    (L)    |    (H)    |    (L)    |  (L)  |  (H)  |
|------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       | BIT 15to8: reserved   |               |
|      |        |        |          | unsigned |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        |       unsigned      |               |       unsigned        | BIT 7to4: GAIN detail |   CRC result  |
| DATA |  'R'   |  'C'   | 0 to 255 | 0 to 255 |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          |      LSB;0.01 m     |      N/A      |        LSB:1          | BIT 3-2: reserved     |   from BYTE   |
|      |        |        |          |          |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       |      0 to 65535     |               |      0 to 65535       | BIT 1: TRK / NTRK     |    0 to 13    |
|      |        |        |          |          |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       |                     |               |                       | BIT 0: GAIN LOW/HIGH  |               |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * 
 * @note Protocol Frame Format:
 *       - Header: 2 bytes identifying mode ('R'+'A' for 1-data, 'R'+'B' for 3-data, 'R'+'C' for 5-data)
 *       - Altitude values: 16-bit unsigned integers, LSB = 0.01m (range 0-655.35m)
 *       - FFT values: Signal strength indicators (0-65535)
 *       - Status bits: Include tracking status (TRK/NTRK) and gain setting (LOW/HIGH)
 *       - CRC: 16-bit checksum covering bytes 0 to N-2
 * 
 * @note Distance Extraction:
 *       Primary distance measurement is always extracted from bytes 4-5 (high byte, low byte)
 *       regardless of measurement mode. Additional distance measurements in multi-data modes
 *       are available but not currently used by this driver.
 * 
 * @note Signal Quality:
 *       Signal quality is derived from the tracking status bit (bit 1 of status byte).
 *       TRK (tracking) indicates reliable signal, NTRK (not tracking) indicates signal loss.
 * 
 * @} // end of JRE_Serial_Protocol group
 */

/**
 * @class AP_RangeFinder_JRE_Serial
 * @brief Rangefinder backend driver for JRE Serial radar altimeters
 * 
 * @details This class implements a serial rangefinder backend for JRE radar altimeters.
 *          It handles reception and parsing of proprietary protocol frames, extracts
 *          distance measurements with 1cm resolution, and monitors signal tracking status.
 *          
 *          Key Features:
 *          - Supports variable frame lengths (16/32/48 bytes)
 *          - Automatic frame synchronization using header detection
 *          - CRC validation for data integrity
 *          - Signal quality reporting based on tracking status
 *          - Distance resolution: 0.01m (1 centimeter)
 *          - Maximum range: 655.35 meters
 * 
 * @note This is a radar-based sensor reporting MAV_DISTANCE_SENSOR_RADAR type
 * @warning Ensure correct serial port configuration for 9600 or 115200 baud operation
 * 
 * @see AP_RangeFinder_Backend_Serial Base class for serial rangefinder implementations
 */
class AP_RangeFinder_JRE_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    /**
     * @brief Factory method to create a JRE Serial rangefinder backend instance
     * 
     * @details This static factory method creates a new instance of the JRE Serial
     *          rangefinder backend. Memory allocation uses NEW_NOTHROW to handle
     *          potential memory constraints gracefully.
     * 
     * @param[in,out] _state      Reference to rangefinder state structure for storing measurements
     * @param[in]     _params     Reference to rangefinder parameters (port config, offsets, etc.)
     * 
     * @return Pointer to newly created backend instance, or nullptr if allocation fails
     * 
     * @note Called by the rangefinder driver framework during sensor initialization
     * @see AP_RangeFinder_Backend_Serial::create for base class factory pattern
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_JRE_Serial(_state, _params);
    }

protected:

    /// @brief Inherit constructor from base class
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Get the MAVLink distance sensor type identifier
     * 
     * @details Reports this sensor as a radar-based distance sensor to ground control
     *          stations and logging systems. This affects how the sensor data is
     *          interpreted and displayed.
     * 
     * @return MAV_DISTANCE_SENSOR_RADAR indicating radar-based ranging technology
     * 
     * @note This is used for MAVLink DISTANCE_SENSOR messages and logging
     * @see MAV_DISTANCE_SENSOR enum in MAVLink common message set
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    /**
     * @brief Get signal quality as a percentage
     * 
     * @details Signal quality is determined from the tracking status bit in the
     *          protocol frame. When the radar is actively tracking (TRK bit set),
     *          signal quality is reported as maximum. When not tracking (NTRK),
     *          signal quality is reported as minimum.
     *          
     *          This binary quality indicator helps the EKF and control systems
     *          determine the reliability of distance measurements.
     * 
     * @return RangeFinder::SIGNAL_QUALITY_MAX (100%) if tracking valid target
     * @return RangeFinder::SIGNAL_QUALITY_MIN (0%) if signal lost or no valid target
     * 
     * @note Signal quality affects sensor fusion weight in navigation filters
     * @warning Low signal quality may trigger failsafe actions in terrain following mode
     * 
     * @see no_signal member variable for tracking state
     */
    int8_t get_signal_quality_pct() const override
    {
        return no_signal ? RangeFinder::SIGNAL_QUALITY_MIN : RangeFinder::SIGNAL_QUALITY_MAX;
    }

private:

    /**
     * @brief Get a distance reading from the rangefinder
     * 
     * @details This method reads serial data from the JRE rangefinder, parses protocol
     *          frames, validates CRC, and extracts distance measurements. It handles
     *          frame synchronization by searching for valid headers and manages the
     *          receive buffer.
     *          
     *          Processing steps:
     *          1. Read available serial data into buffer
     *          2. Search for frame header ('R' + 'A'/'B'/'C')
     *          3. Determine frame length based on header type
     *          4. Validate CRC checksum
     *          5. Extract distance from bytes 4-5 (16-bit unsigned, LSB=0.01m)
     *          6. Update tracking status from status byte
     *          7. Convert distance to meters (divide by 100)
     * 
     * @param[out] reading_m Distance measurement in meters (0.0 to 655.35m)
     * 
     * @return true if valid distance measurement obtained
     * @return false if no valid frame received, CRC error, or serial timeout
     * 
     * @note Called periodically by the rangefinder framework (typically 10-50Hz)
     * @warning Invalid CRC frames are discarded and buffer is re-synchronized
     * 
     * @see move_preamble_in_buffer() for frame synchronization logic
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Shift buffer to align frame header at start position
     * 
     * @details When a valid frame header is detected at a non-zero position in the
     *          buffer, this method shifts the remaining data to the beginning of the
     *          buffer. This maintains buffer alignment for frame processing and prevents
     *          buffer overflow.
     *          
     *          The method performs a memory move operation to shift bytes from
     *          search_start_pos to data_buff_ofs down to the beginning of data_buff.
     * 
     * @param[in] search_start_pos Index in data_buff where valid header was found
     * 
     * @note Modifies data_buff and updates data_buff_ofs to reflect new buffer state
     * @note Called internally by get_reading() during frame synchronization
     */
    void move_preamble_in_buffer(uint8_t search_start_pos);

    /**
     * @brief Receive buffer for serial data
     * 
     * @details Buffer sized to hold 3 complete frames of the longest packet type
     *          (48 bytes × 3 = 144 bytes). This provides sufficient buffering for
     *          frame synchronization and handling partial reads from the serial port.
     *          
     *          Multiple frame capacity allows the driver to:
     *          - Handle burst serial data arrival
     *          - Search for frame headers across frame boundaries
     *          - Maintain data during frame re-synchronization
     */
    uint8_t data_buff[48 * 3];

    /**
     * @brief Current write position in data_buff
     * 
     * @details Index indicating where the next received byte will be stored in
     *          data_buff. Incremented as serial data arrives and reset/adjusted
     *          during frame synchronization.
     *          
     *          Valid range: 0 to (48*3)-1
     */
    uint8_t data_buff_ofs;

    /**
     * @brief Signal tracking status flag
     * 
     * @details Indicates whether the radar is actively tracking a valid target.
     *          Set to true when status byte bit 1 indicates NTRK (not tracking).
     *          Set to false when status byte bit 1 indicates TRK (tracking).
     *          
     *          This flag directly affects the signal quality percentage reported
     *          to the navigation system through get_signal_quality_pct().
     * 
     * @note Updated on each successful frame parse
     * @see get_signal_quality_pct() for how this affects signal quality reporting
     */
    bool no_signal;
};
#endif  // AP_RANGEFINDER_JRE_SERIAL_ENABLED
