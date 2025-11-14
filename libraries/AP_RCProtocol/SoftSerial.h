/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file SoftSerial.h
 * @brief Software serial byte reconstruction from pulse timing measurements
 * 
 * @details This file implements a software UART decoder that reconstructs serial
 *          byte streams from pulse width measurements. Used by RC protocol backends
 *          (SBUS, iBus, DSM, SRXL) to decode serial data from pulse train inputs.
 *          
 *          The implementation uses half-bit timing analysis to sample pulse widths
 *          at appropriate intervals, detecting start bits, accumulating data bits,
 *          validating stop bits, and optionally checking parity to reconstruct
 *          complete serial bytes from the timing measurements.
 * 
 * @note This software UART implementation is critical for RC receiver protocols
 *       that transmit serial data over pulse-width encoded channels.
 */

#pragma once

#include "AP_RCProtocol.h"

/**
 * @class SoftSerial
 * @brief Software UART decoder for reconstructing serial bytes from pulse timing
 * 
 * @details This utility class implements a software serial decoder that converts
 *          pulse width measurements (state0/state1 timing) into serial byte streams.
 *          It reconstructs UART framing by analyzing pulse durations to extract:
 *          - Start bit detection
 *          - Data bit accumulation (8 bits)
 *          - Parity bit validation (for even parity configurations)
 *          - Stop bit verification
 *          - Signal inversion handling (for inverted serial modes)
 *          
 *          Algorithm: Uses half-bit timing windows to sample pulse widths at precise
 *          intervals. Each pulse duration (width_s0 and width_s1) is divided by the
 *          half_bit period to determine how many bit periods elapsed, allowing
 *          reconstruction of the serial bit stream from timing measurements.
 *          
 *          Used by multiple RC protocol backends:
 *          - SBUS: 100000 baud, 8E2 inverted (8-bit even parity 2 stop bits)
 *          - DSM: 115200 baud, 8N1 (8-bit no parity 1 stop bit)
 *          - SRXL: 115200 baud, 8N1
 *          - iBus: 115200 baud, 8N1
 * 
 * @warning Requires precise pulse width measurements with <1% timing error for
 *          reliable byte reconstruction. Timing jitter or measurement inaccuracy
 *          will cause byte framing errors and data corruption.
 * 
 * @note This class maintains minimal state between pulse processing calls,
 *       accumulating bits progressively until a complete byte is received.
 */
class SoftSerial {
public:
    /**
     * @enum serial_config
     * @brief Serial port configuration options for different protocols
     * 
     * @details Defines the serial framing format including data bits, parity,
     *          stop bits, and signal inversion for various RC protocols.
     */
    enum serial_config {
        SERIAL_CONFIG_8N1  = 0, ///< 8 data bits, no parity, 1 stop bit (DSM, SRXL, iBus)
        SERIAL_CONFIG_8E2I = 1, ///< 8 data bits, even parity, 2 stop bits, inverted signal (SBUS)
        SERIAL_CONFIG_8N1I = 2, ///< 8 data bits, no parity, 1 stop bit, inverted signal (FPort)
    };

    /**
     * @brief Construct a software serial decoder with specified baud rate and configuration
     * 
     * @details Initializes the software UART decoder by computing the half-bit timing
     *          period from the specified baud rate and configuring the serial framing
     *          parameters (parity, stop bits, inversion) based on the configuration.
     *          
     *          The half_bit period is calculated as: (1000000 / (baudrate * 2)) microseconds
     *          This represents the time duration of half a bit period at the specified baud rate.
     *          
     *          Example: For 115200 baud, half_bit = 1000000 / (115200 * 2) = 4.34 μs
     * 
     * @param[in] baudrate Serial baud rate in bits per second (bps)
     *                     Common values: 100000 (SBUS), 115200 (DSM/SRXL/iBus)
     * @param[in] config   Serial configuration specifying data format and signal inversion
     *                     (SERIAL_CONFIG_8N1, SERIAL_CONFIG_8E2I, or SERIAL_CONFIG_8N1I)
     * 
     * @note The constructor computes timing parameters but does not allocate dynamic memory
     *       or acquire hardware resources. All configuration is const after construction.
     */
    SoftSerial(uint32_t baudrate, enum serial_config config);
    
    /**
     * @brief Process pulse width measurements to reconstruct serial bytes
     * 
     * @details Core decoding method that analyzes pulse width measurements (state0 and state1
     *          durations) to reconstruct serial byte framing. The algorithm:
     *          
     *          1. Converts pulse widths to bit periods using half-bit timing:
     *             bit_count = (pulse_width_us + half_bit/2) / half_bit
     *          
     *          2. Detects start bit (state transition at bit boundary)
     *          
     *          3. Accumulates data bits by tracking bit offset through the pulse train
     *          
     *          4. Validates stop bits to ensure proper byte framing
     *          
     *          5. Verifies parity for 8E2 configuration (even parity check)
     *          
     *          6. Applies signal inversion for inverted configurations (8E2I, 8N1I)
     *          
     *          The state machine maintains bit_ofs to track position within the current byte
     *          being reconstructed, accumulating bits progressively across multiple pulse
     *          measurements until a complete byte (start + 8 data + parity + stop) is received.
     * 
     * @param[in]  width_s0 Duration of state0 (low) pulse in microseconds
     * @param[in]  width_s1 Duration of state1 (high) pulse in microseconds
     * @param[out] b        Reconstructed serial byte (valid only when method returns true)
     * 
     * @return true if a complete valid byte was successfully reconstructed, false otherwise
     * 
     * @warning Requires accurate pulse width measurements with <1% timing error.
     *          Measurement jitter exceeding half_bit/2 will cause bit sampling errors
     *          and invalid byte reconstruction.
     * 
     * @note This method is called repeatedly with consecutive pulse measurements.
     *       State is maintained internally between calls to accumulate bits progressively.
     *       When a complete byte is decoded, the byte_timestamp_us is updated to the
     *       timestamp of the last pulse that completed the byte.
     */
    bool process_pulse(uint32_t width_s0, uint32_t width_s1, uint8_t &b);

    /**
     * @brief Get microsecond timestamp of the last successfully decoded byte
     * 
     * @details Returns the timestamp (in microseconds) when the last complete serial
     *          byte was successfully reconstructed by process_pulse(). This timestamp
     *          corresponds to the timing of the final pulse measurement that completed
     *          the byte framing.
     *          
     *          Used for:
     *          - Protocol timing analysis and debugging
     *          - Inter-byte gap detection
     *          - Packet boundary identification
     *          - Synchronization verification
     * 
     * @return Timestamp in microseconds (typically from AP_HAL::micros() or equivalent)
     *         of the last successfully decoded byte
     * 
     * @note The timestamp is only updated when process_pulse() successfully returns
     *       a complete byte. Intermediate pulse processing does not update this value.
     */
    uint32_t get_byte_timestamp_us(void) const {
        return byte_timestamp_us;
    }

    /**
     * @brief Get the configured baud rate
     * 
     * @details Returns the serial baud rate (in bits per second) configured during
     *          construction. This is the nominal bit rate used to compute half_bit
     *          timing for pulse width analysis.
     * 
     * @return Baud rate in bits per second (bps)
     */
    uint32_t baud() const { return baudrate; }

private:
    const uint32_t baudrate;        ///< Serial baud rate in bits per second (bps)
    const uint8_t half_bit;         ///< Width of half a bit period in microseconds, computed as (1000000 / (baudrate * 2))
    const enum serial_config config; ///< Serial configuration (8N1, 8E2I, or 8N1I)

    uint8_t data_width;             ///< Number of data bits (typically 8)
    uint8_t byte_width;             ///< Total byte width including start, data, parity, and stop bits
    uint16_t stop_mask;             ///< Bit mask for validating stop bit positions
    uint32_t timestamp_us;          ///< Current pulse timestamp in microseconds
    uint32_t byte_timestamp_us;     ///< Timestamp of last successfully decoded byte in microseconds

    /**
     * @brief Internal state for progressive byte reconstruction
     * 
     * @details Maintains the current byte being decoded and the bit offset
     *          position within that byte. The decoder accumulates bits across
     *          multiple process_pulse() calls until a complete byte is formed.
     */
    struct {
        uint32_t byte;              ///< Accumulated byte value with bit positions filled progressively
        uint16_t bit_ofs;           ///< Current bit offset position in the byte being reconstructed
    } state;
};
