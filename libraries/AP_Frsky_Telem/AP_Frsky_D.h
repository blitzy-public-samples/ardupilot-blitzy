/**
 * @file AP_Frsky_D.h
 * @brief FrSky D protocol implementation for legacy D-series receivers
 * 
 * This file implements the FrSky D telemetry protocol for older D-series
 * receivers (D4R, D8R) that do not support the newer SPort protocol.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Frsky_Backend.h"

#if AP_FRSKY_D_TELEM_ENABLED

/**
 * @class AP_Frsky_D
 * @brief FrSky D protocol telemetry backend for legacy receivers
 * 
 * @details Implements FrSky D telemetry protocol for older D-series receivers.
 * 
 * Protocol Characteristics:
 * - Operates at 9600 baud (vs 57600 for SPort)
 * - Uses START_STOP_D framing (0x5E) and BYTESTUFF_D escaping (0x5D)
 * - Transmits sensor data on fixed time intervals (200ms and 1000ms frames)
 * - No bidirectional capability (transmit-only protocol)
 * - Simplex communication (one-way), unlike SPort which supports bidirectional polling
 * 
 * Data Format:
 * - D protocol uses BP/AP (before point/after point) pairs for fractional values
 *   (e.g., GPS_ALT_BP + GPS_ALT_AP for altitude with decimal precision)
 * 
 * Byte Stuffing:
 * - D protocol escape sequence: 0x5D + XOR 0x60 (differs from SPort's 0x7D + XOR 0x20)
 * - Escapes frame delimiter 0x5E and escape character 0x5D itself
 * 
 * Frame Timing:
 * - High-priority sensors (GPS, altitude, battery) transmitted every 200ms
 * - Lower-priority sensors transmitted every 1000ms
 * - Time-based scheduling vs SPort's poll-response model
 * 
 * @note Conditional compilation via AP_FRSKY_D_TELEM_ENABLED
 * @warning Timing constraints must be respected in send() to avoid blocking backend thread
 * 
 * @see AP_Frsky_Backend.h for DATA_ID constants and base functionality
 * @see AP_Frsky_SPort.h for comparison with newer bidirectional protocol
 */
class AP_Frsky_D : public AP_Frsky_Backend
{

public:

    /**
     * @brief Inherit constructor from AP_Frsky_Backend base class
     * 
     * @details Inherits base class constructor to initialize UART, parameters,
     *          and telemetry backend state without additional D-specific setup.
     */
    using AP_Frsky_Backend::AP_Frsky_Backend;

protected:

    /**
     * @brief Transmit D protocol telemetry frame
     * 
     * @details Called from backend thread loop, generates time-based telemetry frames.
     *          Implements fixed-interval transmission schedule:
     *          - High-priority sensors (GPS position, altitude, battery) every 200ms
     *          - Lower-priority sensors (temperature, fuel) every 1000ms
     * 
     * @note Transmits high-priority sensors every 200ms for responsive display
     * @note Transmits lower-priority sensors every 1000ms to reduce bandwidth usage
     * @warning Must complete within thread loop period to avoid blocking other telemetry tasks
     */
    void send() override;
    
    /**
     * @brief Get D protocol baud rate
     * 
     * @return 9600 baud (FrSky D protocol standard)
     * 
     * @note Overrides SPort default of 57600 baud
     * @note D protocol operates at significantly lower baud rate than SPort
     */
    uint32_t initial_baud() const override
    {
        return 9600;
    }

private:

    /**
     * @brief Transmit single byte with byte-stuffing
     * 
     * @param[in] value Byte value to transmit (uint8_t)
     * 
     * @details Escapes frame delimiter (0x5E) and escape character (0x5D)
     *          using BYTESTUFF_D (0x5D) followed by (value XOR 0x60).
     *          This prevents telemetry data from being interpreted as
     *          frame boundaries or escape sequences.
     * 
     * @note D protocol byte stuffing differs from SPort (0x5D+XOR 0x60 vs 0x7D+XOR 0x20)
     */
    void send_byte(uint8_t value);
    
    /**
     * @brief Transmit sensor data frame
     * 
     * @param[in] id   DATA_ID constant identifying sensor type (uint16_t)
     * @param[in] data Sensor value to transmit (uint16_t)
     * 
     * @details Formats telemetry frame as:
     *          START_STOP_D (0x5E) + id + data_low + data_high
     *          with byte-stuffing applied to id and data bytes.
     *          
     *          Frame structure enables receiver to identify sensor type
     *          and extract 16-bit value split across two bytes.
     * 
     * @note Uses DATA_ID constants defined in AP_Frsky_Backend.h
     * @see AP_Frsky_Backend.h for complete DATA_ID enumeration
     */
    void send_uint16(uint16_t id, uint16_t data);

    /**
     * @brief Frame timing state for D protocol transmission scheduling
     * 
     * @details Used for time-based scheduling vs SPort's poll-response model.
     *          Timestamps stored in milliseconds from AP_HAL::millis().
     */
    struct {
        uint32_t last_200ms_frame;  ///< @brief Timestamp of last high-frequency frame (milliseconds)
        uint32_t last_1000ms_frame; ///< @brief Timestamp of last low-frequency frame (milliseconds)
    } _D;

};

#endif  // AP_FRSKY_D_TELEM_ENABLED
