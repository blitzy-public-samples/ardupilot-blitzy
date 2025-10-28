/**
 * @file GPIO_Sysfs.h
 * @brief Linux sysfs-based GPIO implementation for digital I/O control
 * 
 * Provides GPIO access through Linux kernel sysfs interface (/sys/class/gpio/).
 * This is the standard userspace GPIO API on Linux systems without requiring
 * kernel modules or direct hardware register access.
 */

#pragma once

#include "AP_HAL_Linux.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "GPIO.h"

namespace Linux {

/**
 * @class Linux::DigitalSource_Sysfs
 * @brief Single GPIO pin wrapper using Linux sysfs interface
 * 
 * @details Represents one digital I/O pin controlled via Linux sysfs GPIO API.
 *          Each pin maps to files in /sys/class/gpio/gpioN/ where N is the
 *          kernel GPIO number.
 *          
 *          Sysfs GPIO interface:
 *          - Export: /sys/class/gpio/export (write GPIO number to enable)
 *          - Direction: /sys/class/gpio/gpioN/direction (write "in" or "out")
 *          - Value: /sys/class/gpio/gpioN/value (read/write "0" or "1")
 *          - Unexport: /sys/class/gpio/unexport (release GPIO)
 *          
 *          File descriptor caching:
 *          - Value file kept open for fast read/write operations
 *          - Avoids overhead of opening/closing sysfs files repeatedly
 *          - Critical for performance in timing-sensitive GPIO operations
 * 
 * @note Only GPIO_Sysfs can instantiate (friend class restriction)
 * @note File descriptors closed automatically in destructor
 * @warning GPIO numbers are kernel-specific, not board pin numbers
 * @see GPIO_Sysfs for factory method (channel())
 */
class DigitalSource_Sysfs : public AP_HAL::DigitalSource {
    friend class GPIO_Sysfs;
public:
    /**
     * @brief Destructor closes sysfs file descriptors and unexports GPIO
     * 
     * @note Automatically releases GPIO resources on scope exit
     * @note Unexports pin from sysfs (may fail if already unexported - ignored)
     */
    ~DigitalSource_Sysfs();
    
    /**
     * @brief Read current GPIO pin state
     * 
     * @return uint8_t Pin state: 0 for LOW, 1 for HIGH
     * 
     * @note Reads from cached value file descriptor (fast operation)
     * @note Electrical thresholds: HIGH typically >2.0V, LOW typically <0.8V
     */
    uint8_t read() override;
    
    /**
     * @brief Write GPIO pin state
     * 
     * @param[in] value Pin state: 0 for LOW, non-zero for HIGH
     * 
     * @note Writes to cached value file descriptor (fast operation)
     * @warning Pin must be configured as output (call mode() first)
     */
    void write(uint8_t value) override;
    
    /**
     * @brief Configure GPIO pin direction (input or output)
     * 
     * @param[in] output 0 for input (HAL_GPIO_INPUT), non-zero for output (HAL_GPIO_OUTPUT)
     * 
     * @note Writes to /sys/class/gpio/gpioN/direction file
     * @note Direction changes may briefly glitch pin state
     */
    void mode(uint8_t output) override;
    
    /**
     * @brief Toggle GPIO pin state (LOW→HIGH or HIGH→LOW)
     * 
     * @note Atomic read-modify-write operation
     * @note Useful for LED blinking and state toggling
     * @warning Pin must be configured as output
     */
    void toggle() override;
    
private:
    /* Only GPIO_Sysfs will be able to instantiate */
    DigitalSource_Sysfs(unsigned pin, int value_fd);
    int _value_fd;
    unsigned _pin;
};

/**
 * @class Linux::GPIO_Sysfs
 * @brief Linux sysfs GPIO implementation for boards without custom GPIO drivers
 * 
 * @details Generic GPIO implementation using Linux kernel sysfs interface.
 *          Works on any Linux board with sysfs GPIO support (most modern kernels).
 *          
 *          Pin mapping strategy:
 *          - pin_table[]: Maps ArduPilot virtual pin numbers to kernel GPIO numbers
 *          - Defined per-board in GPIO_*.cpp files (e.g., GPIO_Navio2.cpp)
 *          - Allows board-independent application code
 *          
 *          Sysfs GPIO workflow:
 *          1. Export GPIO via /sys/class/gpio/export
 *          2. Set direction via /sys/class/gpio/gpioN/direction
 *          3. Read/write via /sys/class/gpio/gpioN/value
 *          4. Unexport via /sys/class/gpio/unexport on cleanup
 *          
 *          Performance characteristics:
 *          - Slower than memory-mapped GPIO (kernel syscall overhead)
 *          - Adequate for LEDs, relays, mode switches
 *          - Not suitable for high-frequency PWM or bit-banging protocols
 *          - Typical latency: 10-50μs per operation
 *          
 *          Limitations:
 *          - No interrupt support (use GPIO backend with interrupt capability)
 *          - No hardware PWM (use PWM_Sysfs for that)
 *          - Requires sysfs GPIO enabled in kernel (CONFIG_GPIO_SYSFS)
 * 
 * @note Kernel may reserve some GPIOs for system use - export will fail
 * @warning GPIO numbers change between kernel versions and board revisions
 * @see GPIO_RPI, GPIO_BBB for board-specific implementations with better performance
 */
class GPIO_Sysfs : public AP_HAL::GPIO {
    friend class DigitalSource_Sysfs;
public:
    /**
     * Pin mapping table: virtual pin index → kernel GPIO number
     * Defined in board-specific GPIO_*.cpp files
     */
    static const unsigned pin_table[];
    
    /**
     * Number of pins in pin_table[]
     * Used for bounds checking virtual pin numbers
     */
    static const uint8_t n_pins;

    /**
     * @brief Downcast helper from generic GPIO pointer to GPIO_Sysfs
     * 
     * @param[in] gpio Pointer to AP_HAL::GPIO base class
     * @return GPIO_Sysfs* Downcasted pointer to sysfs implementation
     * 
     * @note Used internally for accessing sysfs-specific features
     * @warning No type checking - caller must ensure gpio is actually GPIO_Sysfs instance
     */
    static GPIO_Sysfs *from(AP_HAL::GPIO *gpio) {
        return static_cast<GPIO_Sysfs*>(gpio);
    }

    /**
     * @brief Initialize sysfs GPIO subsystem
     * 
     * @note Called once during HAL initialization
     * @note Does not export pins yet (deferred until first use)
     */
    void init() override;

    /**
     * @brief Configure virtual pin direction (input or output)
     * 
     * @param[in] vpin Virtual pin number (index into pin_table[])
     * @param[in] output 0 for input, non-zero for output
     * 
     * @note Exports pin if not already exported
     * @note Virtual pin mapped to kernel GPIO via pin_table[]
     * @warning Invalid vpin (>= n_pins) has undefined behavior
     */
    void pinMode(uint8_t vpin, uint8_t output) override;
    
    /**
     * @brief Read virtual pin state
     * 
     * @param[in] vpin Virtual pin number (index into pin_table[])
     * @return uint8_t Pin state: 0 for LOW, 1 for HIGH
     * 
     * @note Pin must be exported and configured as input
     */
    uint8_t read(uint8_t vpin) override;
    
    /**
     * @brief Write virtual pin state
     * 
     * @param[in] vpin Virtual pin number (index into pin_table[])
     * @param[in] value Pin state: 0 for LOW, non-zero for HIGH
     * 
     * @note Pin must be exported and configured as output
     */
    void write(uint8_t vpin, uint8_t value) override;
    
    /**
     * @brief Toggle virtual pin state
     * 
     * @param[in] vpin Virtual pin number (index into pin_table[])
     * 
     * @note Pin must be exported and configured as output
     * @note Atomic read-modify-write operation
     */
    void toggle(uint8_t vpin) override;

    /**
     * @brief Create DigitalSource handle for virtual pin
     * 
     * @param[in] vpin Virtual pin number (index into pin_table[])
     * @return AP_HAL::DigitalSource* Pointer to new DigitalSource_Sysfs instance
     * 
     * @note Exports pin and configures direction
     * @note Caller owns returned pointer - responsible for deletion
     * @note Returns nullptr on export failure (pin in use or invalid)
     */
    AP_HAL::DigitalSource *channel(uint16_t vpin) override;

    /**
     * @brief Check if USB is connected (not implemented)
     * 
     * @return bool Always returns false (no USB detection on sysfs GPIO)
     * 
     * @note Sysfs GPIO has no USB detection mechanism
     * @note Board-specific implementations may override with actual detection
     */
    bool usb_connected() override;

protected:
    void _pinMode(unsigned int pin, uint8_t output);
    int _open_pin_value(unsigned int pin, int flags);

    /*
     * Make pin available for use. This function should be called before
     * calling functions that use the pin number as parameter.
     *
     * Returns true if pin is exported successfully and false otherwise.
     *
     * Note: the pin is ignored if already exported.
     */
    static bool _export_pin(uint8_t vpin);

#ifdef HAL_GPIO_SCRIPT
    /*
      support for calling external scripts based on GPIO writes
     */
    void _gpio_script_write(uint8_t vpin, uint8_t value);

    /*
      thread to run scripts
     */
    void _gpio_script_thread(void);

    /*
      control structures for _gpio_script_write
     */
    typedef struct {
        uint8_t pin;
        uint8_t value;
    } pin_value_t;

    struct {
        bool thread_created;
        ObjectBuffer<pin_value_t> pending{10};
    } _script;
#endif // HAL_GPIO_SCRIPT
};

}
