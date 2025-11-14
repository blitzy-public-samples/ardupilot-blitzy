/**
 * @file GPIO.h
 * @brief GPIO pin simulation implementation for SITL (Software In The Loop)
 * 
 * @details This file provides a simulated GPIO interface for SITL environments,
 *          allowing ArduPilot code to interact with virtual GPIO pins during
 *          simulation without requiring actual hardware. The implementation
 *          provides basic pin state tracking and special handling for certain
 *          simulated peripherals like weight-on-wheels detection for landing gear.
 *          
 *          The GPIO simulation is simplified compared to real hardware:
 *          - No interrupt support
 *          - Instant state changes (no electrical settling time)
 *          - Limited pin modes (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN)
 *          - Most operations are no-ops useful only for code compatibility
 * 
 * @note This is part of the AP_HAL_SITL hardware abstraction layer
 * @warning GPIO simulation does not model real hardware timing or electrical characteristics
 */

#pragma once

#include "AP_HAL_SITL.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

/**
 * @class HALSITL::GPIO
 * @brief Simulated GPIO interface for SITL environments
 * 
 * @details This class provides GPIO pin simulation for Software In The Loop (SITL)
 *          testing. It maintains virtual pin states and modes, allowing ArduPilot
 *          code to interact with GPIO pins during simulation without actual hardware.
 *          
 *          Pin Configuration:
 *          - Supports 16 virtual GPIO pins (pins 0-15)
 *          - pinMode tracking limited to pins 0-7 via 8-bit pin_mode_is_write mask
 *          - Pin states stored in SITL_State for access by simulation physics
 *          
 *          Supported Pin Modes:
 *          - HAL_GPIO_INPUT: Standard input pin
 *          - HAL_GPIO_OUTPUT: Standard output pin
 *          - HAL_GPIO_INPUT_PULLUP: Input with simulated pull-up resistor
 *          - HAL_GPIO_INPUT_PULLDOWN: Input with simulated pull-down resistor
 *          
 *          Special Features:
 *          - Weight-on-wheels GPIO simulation for landing detection
 *          - Integration with simulated vehicle state for realistic pin behavior
 *          - USB connection status simulation
 *          
 *          Limitations:
 *          - No interrupt support (interrupts not simulated)
 *          - Instant state changes (no electrical settling time)
 *          - pinMode only tracks write capability for pins 0-7
 *          - Most operations are no-ops (useful for code compatibility only)
 *          
 * @note Pin states are stored in SITL_State to allow simulation physics to
 *       interact with GPIO (e.g., landing gear weight-on-wheels sensors)
 * @warning This is a simplified simulation - it does not model real hardware
 *          timing, interrupt latency, or electrical characteristics
 */
class HALSITL::GPIO : public AP_HAL::GPIO {
public:
    /**
     * @brief Construct a new GPIO simulation object
     * 
     * @param[in] sitlState Pointer to SITL_State for accessing simulation state
     */
    explicit GPIO(SITL_State *sitlState): _sitlState(sitlState) {}
    
    /**
     * @brief Initialize the GPIO subsystem
     * 
     * @details Initializes the simulated GPIO hardware. In SITL, this typically
     *          resets pin states and prepares the GPIO system for use. Called once
     *          during HAL initialization.
     */
    void init() override;
    
    /**
     * @brief Set the mode of a GPIO pin
     * 
     * @details Configures a pin as input or output. Only pins 0-7 have their mode
     *          tracked in the pin_mode_is_write bitmask due to the 8-bit limitation.
     *          Pins 8-15 are valid but their pinMode state is not persistently tracked.
     *          
     *          Supported modes:
     *          - HAL_GPIO_INPUT: Configure as input
     *          - HAL_GPIO_OUTPUT: Configure as output (enables write operations)
     *          - HAL_GPIO_INPUT_PULLUP: Input with pull-up resistor (simulated)
     *          - HAL_GPIO_INPUT_PULLDOWN: Input with pull-down resistor (simulated)
     * 
     * @param[in] pin Pin number (0-15, but only 0-7 tracked in pin_mode_is_write)
     * @param[in] output Pin mode constant (HAL_GPIO_INPUT, HAL_GPIO_OUTPUT, etc.)
     * 
     * @note pinMode tracking limited to pins 0-7 due to 8-bit pin_mode_is_write mask
     * @warning Pins 8-15 are valid but mode is not persistently stored
     */
    void pinMode(uint8_t pin, uint8_t output) override;
    
    /**
     * @brief Read the current state of a GPIO pin
     * 
     * @details Reads the digital value of a GPIO pin. For input pins, returns the
     *          simulated pin state (may be driven by simulation physics like landing
     *          gear sensors). For output pins, returns the last written value.
     *          
     *          Special handling:
     *          - Weight-on-wheels GPIO pins return simulation landing state
     *          - Other pins return state from SITL_State pin storage
     * 
     * @param[in] pin Pin number (0-15)
     * 
     * @return uint8_t Pin state (0 = LOW, 1 = HIGH)
     * 
     * @note Pin states may be influenced by simulation physics for realistic behavior
     */
    uint8_t read(uint8_t pin) override;
    
    /**
     * @brief Write a digital value to a GPIO pin
     * 
     * @details Sets the output state of a GPIO pin. The value is stored in SITL_State
     *          and can be read back via read() or accessed by simulation physics.
     *          Writing to input pins is typically a no-op but won't cause errors.
     * 
     * @param[in] pin Pin number (0-15)
     * @param[in] value Pin state to write (0 = LOW, non-zero = HIGH)
     * 
     * @note Writes are instant with no electrical settling time
     * @warning Writing to pins not configured as OUTPUT may have no effect
     */
    void write(uint8_t pin, uint8_t value) override;
    
    /**
     * @brief Toggle the state of a GPIO pin
     * 
     * @details Inverts the current state of a GPIO pin (LOW->HIGH or HIGH->LOW).
     *          Implemented as read-modify-write operation.
     * 
     * @param[in] pin Pin number (0-15)
     * 
     * @note Useful for generating square waves or toggling indicators
     */
    void toggle(uint8_t pin) override;

    /**
     * @brief Create a DigitalSource object for a specific pin
     * 
     * @details Alternative interface for GPIO pin access. Returns a DigitalSource
     *          object that provides an abstraction for reading/writing a single pin.
     *          Useful for passing pin references to other code without exposing the
     *          full GPIO interface.
     * 
     * @param[in] n Pin number (0-15)
     * 
     * @return AP_HAL::DigitalSource* Pointer to newly allocated DigitalSource object
     *                                 (caller responsible for deletion)
     */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /**
     * @brief Check if USB cable is connected
     * 
     * @details Returns simulated USB connection status. In SITL, this typically
     *          returns true to simulate normal USB-connected operation during
     *          development and testing.
     * 
     * @return bool true if USB is connected (typically always true in SITL),
     *              false otherwise
     * 
     * @note Used by some subsystems to determine if USB power/data is available
     */
    bool usb_connected(void) override;

    /**
     * @brief Validate that a pin number is within acceptable range
     * 
     * @details Checks if a pin number is valid for this GPIO implementation.
     *          SITL GPIO supports pins 0-15 (16 total pins).
     * 
     * @param[in] pin Pin number to validate
     * 
     * @return bool true if pin is valid (0-15), false otherwise
     * 
     * @note Used internally to prevent out-of-bounds pin access
     */
    bool valid_pin(uint8_t pin) const override { return pin < 16; }
    
private:
    /**
     * @brief Pointer to SITL simulation state
     * 
     * @details Provides access to simulation state including pin values,
     *          vehicle physics state (for weight-on-wheels sensors), and
     *          other simulated hardware state.
     */
    SITL_State *_sitlState;

    /**
     * @brief 8-bit bitmask tracking which pins are configured for write (output)
     * 
     * @details Each bit represents one pin (0-7). When bit is set, the corresponding
     *          pin is configured as OUTPUT and can be written. When bit is clear,
     *          the pin is configured as INPUT.
     *          
     *          Bit mapping: bit 0 = pin 0, bit 1 = pin 1, ..., bit 7 = pin 7
     *          
     * @note Only tracks pins 0-7 due to 8-bit size limitation
     * @warning Pins 8-15 are valid but their pinMode state is not tracked here
     */
    uint8_t pin_mode_is_write;
};

/**
 * @class HALSITL::DigitalSource
 * @brief Abstraction for reading/writing a single simulated GPIO pin
 * 
 * @details DigitalSource provides an object-oriented interface for interacting
 *          with a single GPIO pin. This is useful for passing pin references to
 *          other code without exposing the full GPIO interface, and allows for
 *          polymorphic pin handling.
 *          
 *          Each DigitalSource instance is bound to a specific pin number and
 *          provides methods to read, write, toggle, and configure that pin.
 *          
 *          Typical usage:
 *          ```cpp
 *          AP_HAL::DigitalSource* led_pin = hal.gpio->channel(13);
 *          led_pin->mode(HAL_GPIO_OUTPUT);
 *          led_pin->write(1);  // Turn on LED
 *          led_pin->toggle();  // Toggle LED state
 *          ```
 *          
 *          Implementation details:
 *          - Stores pin number at construction time
 *          - Delegates operations to underlying GPIO implementation
 *          - Provides convenient single-pin interface
 * 
 * @note DigitalSource objects are created by GPIO::channel() method
 * @warning Caller is responsible for managing DigitalSource object lifetime
 */
class HALSITL::DigitalSource : public AP_HAL::DigitalSource {
public:
    /**
     * @brief Construct a DigitalSource for a specific pin
     * 
     * @details Creates a DigitalSource object bound to a specific GPIO pin number.
     *          The pin number is stored and used for all subsequent operations.
     * 
     * @param[in] pin Pin number (0-15) this DigitalSource will control
     * 
     * @note Typically created via GPIO::channel() rather than directly
     */
    explicit DigitalSource(uint8_t pin);
    
    /**
     * @brief Set the mode of the associated GPIO pin
     * 
     * @details Configures the pin as input or output. Delegates to the underlying
     *          GPIO implementation's pinMode() method.
     *          
     *          Supported modes:
     *          - HAL_GPIO_INPUT: Configure as input
     *          - HAL_GPIO_OUTPUT: Configure as output
     *          - HAL_GPIO_INPUT_PULLUP: Input with pull-up
     *          - HAL_GPIO_INPUT_PULLDOWN: Input with pull-down
     * 
     * @param[in] output Pin mode constant (HAL_GPIO_INPUT, HAL_GPIO_OUTPUT, etc.)
     * 
     * @note See GPIO::pinMode() for detailed mode behavior and limitations
     */
    void mode(uint8_t output) override;
    
    /**
     * @brief Read the current state of the associated GPIO pin
     * 
     * @details Reads the digital value of the pin. Delegates to the underlying
     *          GPIO implementation's read() method.
     * 
     * @return uint8_t Pin state (0 = LOW, 1 = HIGH)
     * 
     * @note See GPIO::read() for details on simulated pin behavior
     */
    uint8_t read() override;
    
    /**
     * @brief Write a digital value to the associated GPIO pin
     * 
     * @details Sets the output state of the pin. Delegates to the underlying
     *          GPIO implementation's write() method.
     * 
     * @param[in] value Pin state to write (0 = LOW, non-zero = HIGH)
     * 
     * @note See GPIO::write() for details on write behavior
     */
    void write(uint8_t value) override;
    
    /**
     * @brief Toggle the state of the associated GPIO pin
     * 
     * @details Inverts the current state of the pin (LOW->HIGH or HIGH->LOW).
     *          Delegates to the underlying GPIO implementation's toggle() method.
     * 
     * @note See GPIO::toggle() for details on toggle behavior
     */
    void toggle() override;

private:
    /**
     * @brief GPIO pin number associated with this DigitalSource
     * 
     * @details Stores the pin number (0-15) that this DigitalSource controls.
     *          Set at construction time and immutable thereafter.
     */
    uint8_t _pin;
};
#endif
