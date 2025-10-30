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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

/**
 * @file AP_RCProtocol_PPMSum.h
 * @brief PPM Sum (CPPM - Compound PPM) analog pulse train RC protocol decoder
 * 
 * @details This file implements the PPM Sum protocol decoder backend for ArduPilot.
 *          PPM Sum is a legacy analog protocol used by traditional RC receivers to
 *          encode multiple RC channels into a single pulse train signal.
 *          
 *          Protocol Format:
 *          - Sequential pulse train with sync pulse followed by channel pulses
 *          - Sync pulse: >2700μs duration indicates start of frame
 *          - Channel pulses: Each pulse width directly represents channel PWM value
 *          - Valid channel range: 700-2300μs
 *          - Frame rate: Typically 50Hz (20ms frame period)
 *          - Typical channel count: 8-12 channels maximum (limited by frame rate)
 *          
 *          The protocol is still widely used with traditional RC receivers despite
 *          being considered legacy compared to modern serial protocols (SBUS, DSM, etc.).
 *          
 *          Timing Requirements:
 *          - Requires precise pulse width measurement (±10μs accuracy)
 *          - Timing-critical decoding must complete within frame period
 *          - Noise filtering rejects pulses outside valid range
 * 
 * @note This is a legacy protocol but remains widely deployed in traditional RC systems
 * @warning Timing-critical implementation - pulse width measurement accuracy critical
 * 
 * @author Andrew Tridgell and Siddharth Bharat Purohit
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_PPMSUM_ENABLED

#include "AP_RCProtocol_Backend.h"

/**
 * @class AP_RCProtocol_PPMSum
 * @brief Backend decoder for PPM Sum (CPPM) RC receiver protocol
 * 
 * @details This class decodes PPM Sum pulse trains from traditional RC receivers.
 *          PPM Sum encodes multiple RC channels as a sequential analog pulse train
 *          on a single signal wire, with each pulse width representing a channel value.
 *          
 *          Decoding Algorithm:
 *          1. Detect sync pulse (>2700μs) to identify frame start
 *          2. Subsequent pulses represent channel data (700-2300μs range)
 *          3. Each pulse width directly maps to channel PWM value
 *          4. Continue until next sync pulse or maximum channels reached
 *          5. Validate all pulses are within acceptable range
 *          
 *          Channel Extraction:
 *          - Pulse widths directly represent channel PWM values (no encoding)
 *          - Channel order: Sequential from first pulse after sync
 *          - Maximum channels: Typically 8-12, limited by frame rate constraints
 *          - Invalid pulses rejected to prevent noise-induced false channels
 *          
 *          State Machine:
 *          - Waiting for sync: Looking for pulse >2700μs
 *          - Capturing channels: Recording valid channel pulses
 *          - Frame complete: All channels captured or next sync detected
 *          
 *          Thread Safety: Called from interrupt context during pulse capture
 *          
 * @note Legacy protocol still widely used with traditional RC receivers
 * @warning Timing-critical: Requires ±10μs pulse width measurement accuracy
 * @warning Called from interrupt context - must be fast and non-blocking
 * 
 * @see AP_RCProtocol_Backend for base class interface
 */
class AP_RCProtocol_PPMSum : public AP_RCProtocol_Backend {
public:
    /**
     * @brief Construct PPM Sum protocol decoder backend
     * @param[in] _frontend Reference to parent AP_RCProtocol frontend
     */
    AP_RCProtocol_PPMSum(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend) {}
    
    /**
     * @brief Process captured pulse widths to decode PPM Sum RC channels
     * 
     * @details This method decodes PPM Sum pulse trains by analyzing pulse widths
     *          to extract RC channel data. The protocol uses a simple time-division
     *          encoding where each pulse width directly represents a channel value.
     *          
     *          Decoding Logic:
     *          - Sync pulse detection: width_s1 >2700μs indicates frame start
     *          - Channel pulse: width_s1 in range 700-2300μs is valid channel data
     *          - Pulses outside valid range are rejected as noise
     *          - Channel counter tracks position within frame
     *          - Frame complete triggers channel data update to frontend
     *          
     *          Pulse Validation:
     *          - Valid channel range: 700μs to 2300μs
     *          - Sync pulse threshold: >2700μs
     *          - Out-of-range pulses reset decoder state to prevent corruption
     *          
     *          Channel Extraction:
     *          - Each pulse width directly represents channel PWM value (no conversion)
     *          - Maximum channels limited by MAX_RCIN_CHANNELS
     *          - Typical operation: 8-12 channels per frame
     *          
     *          Performance:
     *          - Called at pulse edge rate (typically 1-2kHz for 8-12 channel system)
     *          - Must execute quickly as called from interrupt context
     *          - Timing accuracy critical: ±10μs for reliable decoding
     * 
     * @param[in] width_s0 Time since system start of previous pulse edge (μs)
     * @param[in] width_s1 Pulse width between previous and current edge (μs)
     *                     - Represents either sync pulse (>2700μs) or channel data (700-2300μs)
     * 
     * @note Called from interrupt context - must be fast and non-blocking
     * @warning Timing-critical: Requires precise pulse width measurement (±10μs accuracy)
     * @warning Invalid pulse widths will reset decoder state and discard partial frame
     * 
     * @see AP_RCProtocol_Backend::process_pulse() for interface definition
     */
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
private:
    /**
     * @brief PPM Sum decoder state tracking
     * 
     * @details Maintains decoder state across pulse captures to reconstruct
     *          RC channel data from sequential pulse train.
     */
    struct {
        int8_t _channel_counter;              ///< Current channel being decoded (0-based), -1 when waiting for sync
        uint16_t _pulse_capt[MAX_RCIN_CHANNELS]; ///< Captured channel pulse widths in microseconds (μs)
    } ppm_state;
};

#endif  // AP_RCPROTOCOL_PPMSUM_ENABLED
