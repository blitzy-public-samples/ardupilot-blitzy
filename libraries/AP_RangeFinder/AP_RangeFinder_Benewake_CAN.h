/**
 * @file AP_RangeFinder_Benewake_CAN.h
 * @brief Benewake TF-series rangefinder driver for CAN bus interface
 * 
 * This file implements support for Benewake TF-series rangefinders (lidar distance sensors)
 * that communicate over CAN bus. Supports both standard CAN frames for basic TF sensors
 * and extended CAN frames for the H30 radar sensor.
 * 
 * Supported sensors:
 * - TF-series lidar sensors (standard CAN frames)
 * - H30 radar sensor (extended CAN frames with multi-target capability)
 * 
 * @note This driver reports MAV_DISTANCE_SENSOR_LASER type for MAVLink compatibility
 * 
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.h
 * Source: libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.cpp
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
#include "AP_RangeFinder_Backend_CAN.h"

/**
 * @class AP_RangeFinder_Benewake_CAN
 * @brief CAN bus backend driver for Benewake TF-series rangefinder sensors
 * 
 * @details This class implements the CAN bus protocol for Benewake rangefinders,
 *          processing incoming CAN frames at up to 100Hz and extracting distance
 *          measurements. Supports two frame formats:
 * 
 *          **Standard CAN Frame Format (TF-series lidar sensors):**
 *          - CAN ID: Configured via RNGFND_RECV_ID parameter
 *          - Frame type: Standard (11-bit ID)
 *          - Data format (8 bytes):
 *            * Bytes 0-1: Distance in centimeters (16-bit little-endian unsigned integer)
 *            * Bytes 2-3: Signal-to-Noise Ratio (SNR) (16-bit little-endian unsigned integer)
 *            * Bytes 4-7: Reserved/unused
 *          - Update rate: Typically 100Hz
 * 
 *          **Extended CAN Frame Format (H30 radar sensor):**
 *          - CAN ID: Configured via RNGFND_RECV_ID parameter
 *          - Frame type: Extended (29-bit ID)
 *          - Data format (8 bytes):
 *            * Bytes 0-1: Target 1 distance in centimeters (16-bit big-endian unsigned integer)
 *            * Bytes 2-3: Target 2 distance in centimeters (16-bit big-endian unsigned integer)
 *            * Bytes 4-5: Target 3 distance in centimeters (16-bit big-endian unsigned integer)
 *            * Bytes 6-7: Reserved/unused
 *          - Multi-target: Tracks up to 3 simultaneous targets (currently only target 1 used)
 *          - Update rate: Typically 100Hz
 * 
 * **Signal Quality Filtering:**
 * - For standard frames, SNR (signal-to-noise ratio) filtering can be configured
 * - Parameter RNGFND_SNR_MIN sets minimum acceptable SNR value
 * - Readings below SNR threshold are discarded
 * - SNR of 0 means no SNR filtering
 * 
 * **Distance Processing:**
 * - Distance measurements are accumulated and averaged in update() cycle
 * - Invalid readings (distance = 0) are rejected
 * - Distance reported in meters (converted from centimeters)
 * - Maximum timeout: 200ms (inherited from base class)
 * 
 * **Thread Safety:**
 * - handle_frame() methods protected by semaphore
 * - Safe for concurrent access from CAN receive thread
 * 
 * @note Reports as MAV_DISTANCE_SENSOR_LASER type despite H30 being radar
 * @warning Ensure CAN bus termination is properly configured for reliable operation
 * 
 * @see AP_RangeFinder_Backend_CAN
 * @see AP_HAL::CANFrame
 */
class AP_RangeFinder_Benewake_CAN : public AP_RangeFinder_Backend_CAN {
public:
    /**
     * @brief Constructor for Benewake CAN rangefinder backend
     * 
     * @details Initializes the Benewake CAN rangefinder driver by registering with
     *          the CAN manager as a Benewake protocol device. The driver is identified
     *          as "benewake" in the CAN subsystem for routing incoming frames.
     * 
     * @param[in,out] _state    Reference to rangefinder state structure for storing
     *                          distance measurements and sensor status
     * @param[in,out] _params   Reference to rangefinder parameters including CAN ID,
     *                          SNR threshold, and sensor configuration
     * 
     * @note Constructor automatically registers this sensor with the CAN manager
     * @note The CAN protocol type is set to AP_CAN::Protocol::Benewake
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.h:10-13
     */
    AP_RangeFinder_Benewake_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
        AP_RangeFinder_Backend_CAN(_state, _params, AP_CAN::Protocol::Benewake, "benewake")
    {
    }

    /**
     * @brief Main CAN frame handler for incoming rangefinder data
     * 
     * @details Processes incoming CAN frames from Benewake sensors at up to 100Hz.
     *          Automatically detects frame type (standard vs extended) and routes
     *          to appropriate handler:
     * 
     *          **Standard Frame Processing (TF-series sensors):**
     *          - Validates CAN ID against configured RNGFND_RECV_ID
     *          - Extracts distance (bytes 0-1, little-endian, centimeters)
     *          - Extracts SNR signal quality (bytes 2-3, little-endian)
     *          - Applies SNR threshold filtering if RNGFND_SNR_MIN > 0
     *          - Accumulates valid distance measurements for averaging
     * 
     *          **Extended Frame Processing (H30 radar):**
     *          - Identifies extended frames (29-bit CAN ID)
     *          - Routes to handle_frame_H30() for radar-specific processing
     *          - Handles multi-target radar data format
     * 
     *          **Frame Rejection Conditions:**
     *          - CAN ID does not match configured RNGFND_RECV_ID
     *          - SNR below RNGFND_SNR_MIN threshold (standard frames only)
     *          - Distance reading is zero (indicates no valid target)
     * 
     * @param[in,out] frame CAN frame containing rangefinder measurement data.
     *                      Frame format depends on sensor type (standard or extended).
     * 
     * @return true if frame was consumed and processed successfully
     * @return false if frame ID doesn't match or reading is invalid
     * 
     * @note This method is called from CAN receive thread context
     * @note Thread-safe: Protected by internal semaphore (_sem)
     * @note Typical call frequency: 100Hz
     * 
     * @warning Do not block in this handler - fast processing required for 100Hz rate
     * @warning SNR filtering discards low-quality measurements silently
     * 
     * @see handle_frame_H30() for extended frame processing
     * @see accumulate_distance_m() for distance accumulation
     * @see is_correct_id() for CAN ID validation
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.cpp:29-55
     */
    bool handle_frame(AP_HAL::CANFrame &frame) override;

    /**
     * @brief Extended CAN frame handler for H30 radar sensor
     * 
     * @details Processes extended CAN frames from Benewake H30 radar sensor which
     *          provides multi-target tracking capability. The H30 reports up to 3
     *          simultaneous targets in each CAN frame.
     * 
     *          **H30 Radar Frame Format:**
     *          - Frame type: Extended CAN (29-bit ID)
     *          - Bytes 0-1: Target 1 distance (16-bit big-endian, centimeters)
     *          - Bytes 2-3: Target 2 distance (16-bit big-endian, centimeters)
     *          - Bytes 4-5: Target 3 distance (16-bit big-endian, centimeters)
     *          - Bytes 6-7: Reserved
     * 
     *          **Current Implementation:**
     *          - Only Target 1 (closest/strongest) is processed
     *          - Targets 2 and 3 are currently ignored
     *          - Zero distance indicates no valid target
     * 
     *          **Processing Steps:**
     *          1. Extract target 1 distance (big-endian 16-bit unsigned integer)
     *          2. Reject if distance is zero (no target detected)
     *          3. Convert from centimeters to meters
     *          4. Accumulate distance for averaging in update cycle
     * 
     * @param[in,out] frame Extended CAN frame from H30 radar containing multi-target
     *                      distance data in big-endian format
     * 
     * @return true if valid target distance was extracted and accumulated
     * @return false if no valid target (distance = 0)
     * 
     * @note Endianness: H30 uses big-endian, unlike standard sensors (little-endian)
     * @note Multi-target: Only target 1 currently used; targets 2 and 3 available for future use
     * @note No SNR filtering applied to H30 radar data
     * 
     * @warning Zero distance readings are silently discarded
     * @warning Frame must be validated as extended frame before calling this method
     * 
     * @see handle_frame() for frame routing logic
     * @see accumulate_distance_m() for distance accumulation
     * 
     * Source: libraries/AP_RangeFinder/AP_RangeFinder_Benewake_CAN.cpp:9-26
     */
    bool handle_frame_H30(AP_HAL::CANFrame &frame);
};

#endif  // AP_RANGEFINDER_BENEWAKE_CAN_ENABLED
