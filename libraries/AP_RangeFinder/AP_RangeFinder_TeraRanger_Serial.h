/**
 * @file AP_RangeFinder_TeraRanger_Serial.h
 * @brief TeraRanger Serial laser rangefinder backend for ArduPilot
 * 
 * This file implements support for TeraRanger laser rangefinder sensors that
 * communicate via serial protocol. TeraRanger sensors are compact laser distance
 * sensors manufactured by Terabee, commonly used for precision altitude hold,
 * terrain following, and obstacle detection in autonomous vehicles.
 * 
 * Protocol: 5-byte packets at configurable baud rate (default 115200)
 * Packet format: [Header(0x54), Dist_H, Dist_L, Status, CRC8]
 * 
 * Supported sensors: TeraRanger One, TeraRanger Evo series (Serial variants)
 * 
 * @note This driver handles the serial protocol variant. For I2C variants,
 *       see AP_RangeFinder_TeraRanger_I2C.h
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.h
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.cpp
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"

/**
 * @class AP_RangeFinder_TeraRanger_Serial
 * @brief Backend driver for TeraRanger serial laser rangefinder sensors
 * 
 * @details This class implements the serial communication protocol for TeraRanger
 *          laser distance sensors. It handles packet reception, CRC validation,
 *          distance extraction, and error handling for TeraRanger sensors
 *          connected via UART.
 *          
 *          Key features:
 *          - Serial packet parsing with CRC8 validation
 *          - Multi-sample averaging for noise reduction
 *          - Status bit checking for sensor health
 *          - Out-of-range detection and handling
 *          - Distance error detection (1mm special case)
 *          
 *          Communication Protocol:
 *          - Frame length: 5 bytes
 *          - Frame header: 0x54
 *          - Byte 1-2: Distance in millimeters (big-endian uint16)
 *          - Byte 3: Status/Strength/OverTemp flags
 *          - Byte 4: CRC8 checksum
 *          
 *          The driver processes up to 8192 bytes per update cycle, averaging
 *          all valid readings received since the last update. This provides
 *          robust distance estimates even at high sensor update rates.
 * 
 * @note Inherits serial communication handling from AP_RangeFinder_Backend_Serial
 * @warning Ensure correct baud rate configuration (typically 115200) in parameters
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.cpp:43-113
 */
class AP_RangeFinder_TeraRanger_Serial : public AP_RangeFinder_Backend_Serial
{

public:

    /**
     * @brief Factory method to create TeraRanger Serial rangefinder backend instance
     * 
     * @details Creates and returns a new instance of the TeraRanger Serial driver.
     *          This method is called by the rangefinder manager during sensor
     *          initialization. Uses NEW_NOTHROW for safe heap allocation in
     *          memory-constrained embedded environments.
     * 
     * @param[in,out] _state Reference to rangefinder state structure containing
     *                       current distance, status, signal quality, etc.
     * @param[in] _params Reference to rangefinder parameters including pin config,
     *                    orientation, min/max range, and serial port settings
     * 
     * @return Pointer to new AP_RangeFinder_TeraRanger_Serial instance on success,
     *         nullptr if heap allocation fails
     * 
     * @note Called during rangefinder driver probe/initialization phase
     * @note Memory allocation failure returns nullptr; caller must check
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.h:15-19
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_TeraRanger_Serial(_state, _params);
    }

protected:

    /**
     * @brief Inherit constructor from base class
     * 
     * @details Uses inherited constructor from AP_RangeFinder_Backend_Serial,
     *          which initializes the serial port connection and base rangefinder
     *          state. This simplifies initialization by reusing the parent class
     *          constructor that handles UART setup and parameter validation.
     * 
     * @note C++11 constructor inheritance syntax
     */
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink distance sensor type for this rangefinder,
     *          used in telemetry reporting to ground control stations. TeraRanger
     *          sensors use laser technology for distance measurement.
     *          
     *          This information is included in MAVLink DISTANCE_SENSOR messages
     *          to inform the GCS of the sensor technology in use, which can
     *          affect characteristics like beam width, accuracy, and environmental
     *          limitations.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser-based measurement
     * 
     * @note Overrides virtual method from AP_RangeFinder_Backend
     * @note Used by MAVLink telemetry subsystem for DISTANCE_SENSOR messages
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.h:25-27
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    /**
     * @brief Read and process distance measurement from TeraRanger sensor
     * 
     * @details Processes incoming serial data from the TeraRanger sensor, parsing
     *          5-byte packets, validating CRC checksums, and extracting distance
     *          measurements. This method implements the complete TeraRanger serial
     *          protocol handling.
     *          
     *          Algorithm:
     *          1. Read available bytes from UART (up to 8192 bytes per call)
     *          2. Scan for frame header (0x54) to synchronize packet reception
     *          3. Accumulate 5-byte packets into line buffer
     *          4. Validate packet CRC8 checksum
     *          5. Extract 16-bit distance value (big-endian)
     *          6. Check status byte for errors/overtemp/strength issues
     *          7. Average all valid readings since last update
     *          
     *          Error Handling:
     *          - Invalid CRC: Packet discarded, buffer cleared
     *          - Distance >= 30m: Treated as out-of-range
     *          - Status errors (bit 0-4): Reading rejected
     *          - Distance = 1mm: Special error code, reading rejected
     *          - No valid readings: Returns false (no update)
     *          - Out-of-range readings: Returns max_range + 10m
     *          
     *          Performance: Processes up to 8192 bytes per update cycle to handle
     *          high sensor data rates while maintaining real-time performance.
     * 
     * @param[out] reading_m Distance measurement in meters. Valid range 0-30m.
     *                       Set to max_range+10m for out-of-range detections.
     *                       Contains averaged distance if multiple valid readings
     *                       received since last update.
     * 
     * @return true if valid reading obtained (either in-range or out-of-range),
     *         false if no data available or all packets invalid
     * 
     * @note Called by AP_RangeFinder at configured update rate (typically 20Hz)
     * @note Averaging multiple readings per update improves accuracy and reduces noise
     * @warning Returns false if UART not initialized - sensor will be marked unhealthy
     * @warning Out-of-range returns true with distance = max(30m, max_range+10m)
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.cpp:43-113
     */
    bool get_reading(float &reading_m) override;

    /**
     * @brief Serial packet line buffer for TeraRanger frame assembly
     * 
     * @details Temporary buffer for accumulating bytes of the 5-byte TeraRanger
     *          serial packet. Buffer size is 10 bytes (2x packet size) to provide
     *          margin, though only 5 bytes are used per packet.
     *          
     *          Packet format stored in buffer:
     *          - linebuf[0]: Frame header (0x54)
     *          - linebuf[1]: Distance high byte (bits 15-8)
     *          - linebuf[2]: Distance low byte (bits 7-0)
     *          - linebuf[3]: Status/Strength/OverTemp flags
     *          - linebuf[4]: CRC8 checksum
     *          
     *          Buffer is cleared (linebuf_len=0) after each complete packet is
     *          processed or when CRC validation fails.
     * 
     * @note Buffer is stateful across multiple get_reading() calls for partial packets
     * @note Size 10 provides safety margin, actual packet length is 5 bytes
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.cpp:60-94
     */
    uint8_t linebuf[10];
    
    /**
     * @brief Current number of bytes stored in line buffer
     * 
     * @details Tracks the number of valid bytes currently in linebuf. Range 0-5.
     *          Reset to 0 after each complete packet is processed or on CRC error.
     *          Used for packet synchronization and frame boundary detection.
     *          
     *          State machine:
     *          - 0: Waiting for frame header (0x54)
     *          - 1-4: Accumulating packet bytes
     *          - 5: Complete packet ready for processing
     * 
     * @note Valid range: 0-5 (5-byte packet length)
     * @note Reset to 0 when packet processing completes or fails
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_TeraRanger_Serial.cpp:60-94
     */
    uint8_t linebuf_len;
};
#endif  // AP_RANGEFINDER_TERARANGER_SERIAL_ENABLED
