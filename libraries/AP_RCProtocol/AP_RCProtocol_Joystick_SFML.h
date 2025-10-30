/**
 * @file AP_RCProtocol_Joystick_SFML.h
 * @brief SITL joystick/gamepad RC input via SFML library
 * 
 * @details This file implements RC input decoding from physical joystick or
 *          gamepad devices connected to the development machine during SITL
 *          (Software In The Loop) simulation. The implementation uses the SFML
 *          (Simple and Fast Multimedia Library) to interface with system joystick
 *          APIs on Windows, Linux, and macOS.
 *          
 *          The backend polls connected joystick devices and converts their axis
 *          positions and button states into standard ArduPilot RC PWM channel
 *          values (1000-2000μs range), allowing realistic pilot input testing
 *          in simulation with actual hardware controllers.
 * 
 * @note This backend is only compiled when AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
 *       is defined, which requires SFML_JOYSTICK to be defined in the SITL build.
 * 
 * @warning SIMULATION ONLY - This code is never compiled for flight hardware.
 *          Only available in SITL builds with SFML library support enabled.
 * 
 * @see AP_RCProtocol_Backend Base class for RC protocol decoders
 * @see libraries/AP_HAL_SITL for SITL simulation infrastructure
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED

#include "AP_RCProtocol_Backend.h"

/**
 * @class AP_RCProtocol_Joystick_SFML
 * @brief RC protocol backend for physical joystick/gamepad input in SITL simulation
 * 
 * @details This backend provides RC input from physical joystick or gamepad devices
 *          connected to the SITL simulation host computer. It interfaces with the
 *          SFML (Simple and Fast Multimedia Library) joystick API to:
 *          - Detect and enumerate connected joystick/gamepad devices
 *          - Poll joystick axis positions (typically -100 to +100 range)
 *          - Read button states (pressed/released)
 *          - Convert axis values to standard RC PWM microseconds (1000-2000μs)
 *          - Map joystick axes to RC channels (roll, pitch, throttle, yaw, etc.)
 *          
 *          Typical axis mapping convention:
 *          - Axis 0 → Roll (Channel 1)
 *          - Axis 1 → Pitch (Channel 2)
 *          - Axis 2 → Throttle (Channel 3)
 *          - Axis 3 → Yaw (Channel 4)
 *          - Additional axes map to auxiliary channels
 *          
 *          Axis value conversion:
 *          - SFML provides axis positions as percentages: -100 (full left/down) to +100 (full right/up)
 *          - Backend converts to PWM microseconds: 1000μs (minimum) to 2000μs (maximum)
 *          - Center position (0) maps to 1500μs (neutral)
 *          
 *          Device Detection:
 *          - SFML supports up to 8 joystick devices (device IDs 0-7)
 *          - Each device can have multiple axes (typically 2-8) and buttons (typically 8-16)
 *          - Backend automatically detects connected devices on startup
 *          - Supports hot-plugging on platforms where SFML provides this capability
 * 
 * @note Requires SFML library to be installed on the SITL development system:
 *       - Ubuntu/Debian: sudo apt-get install libsfml-dev
 *       - macOS: brew install sfml
 *       - Windows: Download SFML SDK from https://www.sfml-dev.org/
 * 
 * @warning SIMULATION ONLY - This backend is exclusively for SITL testing and
 *          is never compiled for actual flight hardware. The SFML library is
 *          not available on embedded ARM platforms.
 * 
 * @see AP_RCProtocol_Backend Base class providing RC channel storage and update mechanisms
 */
class AP_RCProtocol_Joystick_SFML : public AP_RCProtocol_Backend {
public:

    /**
     * @brief Construct a new SFML joystick RC protocol backend
     * 
     * @details Initializes the SFML joystick backend for reading physical joystick/gamepad
     *          input during SITL simulation. The constructor sets up the connection to the
     *          frontend RC protocol manager.
     *          
     *          Joystick device detection and initialization is performed during the first
     *          update() call, not in the constructor, to allow for proper SFML subsystem
     *          initialization timing.
     * 
     * @param[in] _frontend Reference to the AP_RCProtocol frontend manager that owns this backend
     * 
     * @note Constructor is lightweight - actual SFML joystick detection happens in update()
     */
    AP_RCProtocol_Joystick_SFML(AP_RCProtocol &_frontend) :
        AP_RCProtocol_Backend(_frontend) {
    }

    /**
     * @brief Poll SFML joystick API and update RC channel values
     * 
     * @details This method is called periodically by the RC protocol manager to poll
     *          connected joystick devices and update RC channel PWM values. The update
     *          process includes:
     *          
     *          1. Check for connected joystick devices (SFML device IDs 0-7)
     *          2. For each connected device:
     *             - Read all axis positions (SFML provides values from -100 to +100)
     *             - Read all button states (pressed = true, released = false)
     *          3. Convert axis values to PWM microseconds:
     *             - SFML axis -100 → 1000μs (minimum)
     *             - SFML axis 0 → 1500μs (center/neutral)
     *             - SFML axis +100 → 2000μs (maximum)
     *          4. Map joystick axes to RC channels:
     *             - Typically axis 0 → Roll (CH1), axis 1 → Pitch (CH2),
     *               axis 2 → Throttle (CH3), axis 3 → Yaw (CH4)
     *          5. Map button states to additional RC channels if configured
     *          6. Call add_input() to update frontend with new RC channel values
     *          7. Update last_receive_ms timestamp for connection health monitoring
     *          
     *          The method queries SFML's joystick API which interfaces with:
     *          - Linux: /dev/input/js* devices via the Linux joystick API
     *          - Windows: Windows.Gaming.Input or DirectInput APIs
     *          - macOS: IOKit HID framework
     *          
     *          Axis Interpretation:
     *          - SFML uses a right-handed coordinate system for axes
     *          - Negative values typically mean left/backward/down depending on axis
     *          - Positive values typically mean right/forward/up depending on axis
     *          - Exact axis meanings depend on joystick device capabilities
     *          
     *          Performance Characteristics:
     *          - Called at main SITL loop rate (typically 1000Hz in simulation)
     *          - SFML joystick polling is non-blocking and fast (<1ms typically)
     *          - No sensor calibration required - uses SFML's calibrated values
     * 
     * @return void
     * 
     * @note Update frequency matches the SITL scheduler rate, providing high-resolution
     *       pilot input simulation for testing flight control response.
     * 
     * @note If no joystick is connected, the method returns immediately without error,
     *       allowing SITL to run with other RC input methods (MAVLink RC override, etc.)
     * 
     * @warning Only functional in SITL builds with SFML library linked. Will never be
     *          called on actual flight hardware as the code is conditionally compiled out.
     * 
     * @see AP_RCProtocol_Backend::add_input() Used to submit decoded RC channel values
     * @see SFML Joystick API documentation: https://www.sfml-dev.org/documentation/2.5.1/classsf_1_1Joystick.php
     */
    void update() override;

private:

    /**
     * @brief Timestamp of last successful joystick input read
     * 
     * @details Stores the system time in milliseconds when joystick data was last
     *          successfully read from the SFML joystick API. This timestamp is used for:
     *          - Connection health monitoring (detect if joystick becomes disconnected)
     *          - Timeout detection for RC failsafe logic
     *          - Determining data freshness for the RC protocol frontend
     *          
     *          Updated on every successful joystick poll in update() method when
     *          valid axis data is read from a connected joystick device.
     *          
     *          Units: milliseconds (ms) since system boot
     *          
     *          Typical update rate: 1000Hz in SITL (updates every 1ms during active joystick use)
     * 
     * @note Value of 0 indicates no joystick data has been received since initialization
     */
    uint32_t last_receive_ms;
};


#endif  // AP_RCPROTOCOL_JOYSTICK_SFML_ENABLED
