/**
 * @file AP_RCProtocol_Radio.h
 * @brief RC protocol backend for AP_Radio integration with Cypress Semiconductor CYRF6936-based 2.4GHz radios
 * 
 * @details This backend interfaces with the AP_Radio library to receive RC channel data
 *          from custom 2.4GHz radio hardware based on the Cypress Semiconductor CYRF6936
 *          2.4GHz transceiver chipset. Unlike other RC protocol backends that decode data
 *          from UART streams, this backend acts as a bridge between the AP_Radio singleton
 *          (which handles the low-level RF protocol and packet decoding) and the RC input
 *          system.
 * 
 *          Data Flow: CYRF6936 Hardware → AP_Radio (RF protocol) → AP_RCProtocol_Radio
 *                     (channel extraction) → RC Input System
 * 
 *          The AP_Radio library handles all RF-specific operations including frequency
 *          hopping, packet framing, error correction, and channel decoding. This backend
 *          simply polls the AP_Radio singleton via AP::radio() to retrieve the decoded
 *          channel values.
 * 
 * @note This backend is typically used with custom autopilot designs that integrate
 *       CYRF6936-based radio hardware directly on the flight controller board.
 * 
 * @warning Requires specific radio hardware: This backend will only function on
 *          autopilot boards with CYRF6936 radio hardware and AP_RADIO_ENABLED
 *          compile-time support. On boards without this hardware, the backend
 *          will be disabled at compile time via AP_RCPROTOCOL_RADIO_ENABLED.
 * 
 * @see AP_Radio for the underlying radio hardware interface
 * @see AP_RCProtocol_Backend for the base protocol interface
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_RADIO_ENABLED

#include "AP_RCProtocol_Backend.h"

/**
 * @class AP_RCProtocol_Radio
 * @brief RC protocol backend that receives decoded channel data from AP_Radio singleton
 * 
 * @details This backend provides integration between the AP_Radio library (which handles
 *          CYRF6936 2.4GHz radio hardware) and the ArduPilot RC input system. Rather than
 *          decoding RC protocol data from a byte stream like most backends, this class
 *          polls the AP_Radio singleton to retrieve already-decoded channel values.
 * 
 *          Channel Data Conversion:
 *          - AP_Radio provides normalized channel values in the range 0-10000
 *          - This backend scales values to standard PWM range 1000-2000μs
 *          - Conversion formula: PWM = (radio_value * 5 / 32) + 1500
 *          - This centers the output at 1500μs (neutral position)
 * 
 *          Integration Pattern:
 *          1. Access AP_Radio singleton via AP::radio()
 *          2. Check if radio hardware is available and receiving data
 *          3. Read decoded channel count and values from AP_Radio
 *          4. Convert from AP_Radio's 0-10000 range to PWM 1000-2000μs
 *          5. Pass converted values to RC input system via add_input()
 * 
 *          Hardware Requirements:
 *          - Autopilot board must have integrated CYRF6936 radio hardware
 *          - AP_RADIO_ENABLED must be defined at compile time
 *          - Radio firmware must be loaded and functional
 * 
 *          Use Cases:
 *          - Custom autopilot designs with integrated 2.4GHz radio
 *          - Specialized applications requiring tight radio integration
 *          - Systems where radio hardware is part of the flight controller
 * 
 * @warning Hardware-Specific: This backend requires specific CYRF6936-based radio
 *          hardware. It will not function on standard autopilots without this hardware.
 *          The backend is automatically disabled via AP_RCPROTOCOL_RADIO_ENABLED on
 *          incompatible platforms.
 * 
 * @note Thread Safety: This backend accesses the AP_Radio singleton which handles
 *       its own thread synchronization for radio interrupt handling.
 * 
 * @see AP_Radio for the underlying radio hardware interface and channel decoding
 * @see AP_RCProtocol_Backend for the base protocol interface and add_input() method
 */
class AP_RCProtocol_Radio : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Poll AP_Radio singleton for decoded RC channel data
     * 
     * @details This method is called periodically by the RC input system to check for
     *          new RC data. It accesses the AP_Radio singleton via AP::radio() to
     *          retrieve decoded channel values from the CYRF6936 radio hardware.
     * 
     *          Update Sequence:
     *          1. Ensure AP_Radio is initialized (calls init() on first run)
     *          2. Access AP_Radio singleton via AP::radio()
     *          3. Check if radio is receiving valid data
     *          4. Read channel count and values (typically 8 or 16 channels)
     *          5. Convert from AP_Radio's 0-10000 range to PWM 1000-2000μs range
     *          6. Pass converted values to RC system via add_input()
     * 
     *          Channel Value Conversion:
     *          - Input: AP_Radio normalized values (0-10000, centered at 5000)
     *          - Output: Standard PWM μs values (1000-2000μs, centered at 1500μs)
     *          - Formula: PWM = (radio_value * 5 / 32) + 1500
     *          - This scales the 0-10000 range to ±500μs around 1500μs center
     * 
     *          Data Validation:
     *          - Checks for radio data timeout via last_input_us tracking
     *          - Validates channel count matches expected 8 or 16 channels
     *          - Ensures AP_Radio reports healthy reception status
     * 
     * @note Called at scheduler rate (typically 50-400Hz depending on vehicle type)
     * @note No input timeout checking - relies on AP_Radio's own timeout mechanisms
     * 
     * @warning Timing: This method should complete quickly as it's called in the
     *          scheduler loop. AP_Radio handles the heavy lifting of RF protocol.
     * 
     * @see AP_Radio::num_channels() for channel count
     * @see AP_Radio::read() for channel value access
     * @see AP_RCProtocol_Backend::add_input() for passing data to RC system
     */
    void update() override;

    /**
     * @brief Initiate transmitter binding mode for radio pairing
     * 
     * @details Commands the AP_Radio singleton to enter binding mode, allowing
     *          the radio receiver to pair with a new transmitter. This typically
     *          involves the radio entering a discovery mode where it listens for
     *          binding requests from transmitters.
     * 
     *          Binding Process:
     *          1. User initiates bind via ground control station or switch
     *          2. This method calls AP_Radio::start_bind()
     *          3. Radio enters binding mode (often indicated by LED pattern)
     *          4. User activates bind on transmitter
     *          5. Radio stores new transmitter ID and exits bind mode
     * 
     * @note Binding procedures are hardware-specific and handled by AP_Radio
     * @note Safety: Vehicle should be disarmed before initiating binding
     * 
     * @see AP_Radio::start_bind() for the underlying radio bind implementation
     */
    void start_bind(void) override;

private:

    /**
     * @brief Initialize AP_Radio singleton and verify hardware availability
     * 
     * @details Performs one-time initialization of the AP_Radio interface. Checks
     *          that the radio hardware is present and functional before allowing
     *          this backend to process channel data.
     * 
     * @return true if AP_Radio is available and ready, false if hardware unavailable
     * 
     * @note Called automatically on first update() if not already initialized
     */
    bool init();
    
    /**
     * @brief Flag indicating whether init() has been successfully called
     * 
     * @details Used to ensure init() is only called once. Set to true after
     *          successful initialization of AP_Radio interface.
     */
    bool init_done;

    /**
     * @brief Timestamp of last valid RC input from AP_Radio in microseconds
     * 
     * @details Tracks the last time valid channel data was received from the
     *          AP_Radio singleton. Used for detecting data timeouts and ensuring
     *          fresh RC input data.
     * 
     * @note Timestamp is in microseconds from system boot (AP_HAL::micros())
     */
    uint32_t last_input_us;
};


#endif  // AP_RCPROTOCOL_RADIO_ENABLED
