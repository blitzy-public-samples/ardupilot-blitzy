/**
 * @file GPIO.h
 * @brief General Purpose Input/Output interface for digital I/O pins
 * 
 * @details Provides platform-independent digital pin control for LEDs, relays, sensors, and
 *          RC input decoding. Supports pin mode configuration, read/write operations, and
 *          interrupt-driven PWM input measurement via PWMSource class.
 *          
 *          Pin numbers are HAL-specific and defined in board hwdef files (ChibiOS) or
 *          HAL-specific headers (Linux, SITL). Not compatible with Arduino pin numbering.
 * 
 * @note This is part of the Hardware Abstraction Layer (HAL) interface contract
 * @see AP_HAL::GPIO for the main GPIO interface class
 * @see AP_HAL::PWMSource for PWM signal decoding
 * @see AP_HAL::DigitalSource for alternative pin interface
 */

#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"

/**
 * @def HAL_GPIO_INPUT
 * @brief Pin mode: Digital input with high impedance
 * 
 * @details Configures pin as input for reading digital signals. Pin has high impedance
 *          and does not source or sink current. May have internal pull-up/pull-down
 *          depending on platform (configured separately).
 */
#define HAL_GPIO_INPUT  0

/**
 * @def HAL_GPIO_OUTPUT
 * @brief Pin mode: Digital output (push-pull)
 * 
 * @details Configures pin as output for driving digital signals. Pin can source and
 *          sink current (typically 4-25mA depending on platform). Use for LEDs,
 *          relays (with appropriate driver circuits), and digital control signals.
 */
#define HAL_GPIO_OUTPUT 1

/**
 * @def HAL_GPIO_ALT
 * @brief Pin mode: Alternate function (hardware peripheral control)
 * 
 * @details Configures pin for hardware peripheral use (UART, SPI, I2C, PWM, etc.).
 *          Pin control is delegated to hardware peripheral, not software GPIO.
 *          Specific alternate function selected via platform-specific configuration.
 */
#define HAL_GPIO_ALT    2

/**
 * @class AP_HAL::DigitalSource
 * @brief Alternative interface for controlling a single digital I/O pin
 * 
 * @details Provides object-oriented access to individual GPIO pins through the
 *          GPIO::channel() method. Each DigitalSource instance represents one
 *          pin and maintains its configuration state.
 *          
 *          Useful for abstracting pin operations when passing pin control to
 *          other modules without exposing the entire GPIO interface.
 * 
 * @note Pure virtual interface - platform HALs provide concrete implementations
 * @see AP_HAL::GPIO::channel() for obtaining DigitalSource instances
 */
class AP_HAL::DigitalSource {
public:
    /**
     * @brief Configure pin direction (input or output)
     * 
     * @param[in] output Pin mode: 0=input (HAL_GPIO_INPUT), 1=output (HAL_GPIO_OUTPUT)
     * 
     * @note Must be called before read() or write() operations
     * @see HAL_GPIO_INPUT, HAL_GPIO_OUTPUT
     */
    virtual void    mode(uint8_t output) = 0;
    
    /**
     * @brief Read current digital state of pin
     * 
     * @return Pin state: 0=LOW (typically <0.8V), 1=HIGH (typically >2.0V)
     * 
     * @note Pin must be configured as input via mode(HAL_GPIO_INPUT) first
     * @note Voltage thresholds are platform-specific (typically 3.3V or 5V logic)
     */
    virtual uint8_t read() = 0;
    
    /**
     * @brief Write digital value to pin
     * 
     * @param[in] value Pin state to write: 0=LOW (0V), non-zero=HIGH (VCC)
     * 
     * @note Pin must be configured as output via mode(HAL_GPIO_OUTPUT) first
     * @warning Ensure load does not exceed pin current limits (typically 4-25mA)
     */
    virtual void    write(uint8_t value) = 0;
    
    /**
     * @brief Toggle pin state atomically
     * 
     * @details Changes pin from LOW to HIGH or HIGH to LOW in single operation.
     *          More efficient than read-modify-write for LED blinking applications.
     * 
     * @note Pin must be configured as output
     * @note Implementation is atomic on most platforms (no race conditions)
     */
    virtual void    toggle() = 0;
};

/**
 * @class AP_HAL::PWMSource
 * @brief Edge-triggered PWM pulse width measurement using GPIO interrupts
 * 
 * @details Measures pulse width of PWM signals (typically 1000-2000μs for RC receivers)
 *          by timestamping rising and falling edges via interrupt callbacks.
 *          
 *          **Typical Use Cases**:
 *          - RC receiver PWM/PPM decoding
 *          - RPM sensor pulse measurement
 *          - Frequency/duty cycle measurement of digital sensors
 *          
 *          **Measurement Algorithm**:
 *          1. Attaches interrupt to specified GPIO pin (both edges)
 *          2. On rising edge: Records timestamp as pulse start
 *          3. On falling edge: Calculates pulse width = (fall_time - rise_time)
 *          4. Stores result for retrieval by get_pwm_us()
 *          
 *          **Thread Safety**:
 *          Uses hal.scheduler->interrupt_disable/restore to safely copy values
 *          from interrupt context to application context. get_pwm_us() and
 *          get_pwm_avg_us() are safe to call from any thread.
 *          
 *          **Timing Accuracy**:
 *          - Resolution: Platform-dependent (typically 1μs on STM32, 10μs on Linux)
 *          - Jitter sources: Interrupt latency, other interrupts, CPU load
 *          - Expected jitter: ±5-50μs depending on platform and system load
 * 
 * @note Destructor automatically detaches interrupt
 * @warning Do not call from interrupt context - uses interrupt disable/restore
 * @see AP_HAL::GPIO::attach_interrupt() for underlying interrupt mechanism
 */
class AP_HAL::PWMSource {
public:

    /**
     * @brief Destructor detaches interrupt handler from pin
     * 
     * @details Ensures interrupt is cleanly removed when PWMSource is destroyed.
     *          Safe to call even if no pin is currently attached.
     */
    ~PWMSource();

    /**
     * @brief Attach PWM measurement to specified GPIO pin
     * 
     * @param[in] new_pin GPIO pin number (HAL-specific, from board hwdef)
     * @param[in] subsystem Descriptive name for debugging (e.g., "RCInput")
     * 
     * @return true if interrupt successfully attached, false on error
     * 
     * @details Detaches from previous pin (if any) and attaches to new pin.
     *          Configures pin as input with interrupt on both edges.
     *          Measurement begins immediately after successful attachment.
     * 
     * @note Can be called multiple times to switch pins dynamically
     * @note Pin number validity checked via GPIO::valid_pin()
     * @warning Invalid pin numbers return false without error message
     * 
     * @see pin() to retrieve currently attached pin
     */
    bool set_pin(int16_t new_pin, const char *subsystem);
    
    /**
     * @brief Get currently attached pin number
     * 
     * @return GPIO pin number, or -1 if no pin attached
     * 
     * @note Returns -1 if set_pin() has not been called or failed
     */
    int16_t pin() const { return _pin; }

    /**
     * @brief Get most recently measured pulse width
     * 
     * @return Pulse width in microseconds (typically 1000-2000μs for RC signals)
     * 
     * @details Returns the last complete pulse measurement (rising edge to falling edge).
     *          Value is updated by interrupt handler on each falling edge.
     *          
     *          Thread-safe: Uses interrupt disable/restore to atomically copy value
     *          from interrupt context.
     * 
     * @note Returns 0 if no pulse measured yet or if signal lost
     * @note For RC receivers, expect 1000-2000μs range (1500μs = center)
     * @note Update rate depends on input signal frequency (typically 50Hz = 20ms period)
     * 
     * @see get_pwm_avg_us() for averaged measurement with better noise rejection
     */
    uint16_t get_pwm_us();
    
    /**
     * @brief Get average pulse width since last call to this function
     * 
     * @return Average pulse width in microseconds, or 0 if no pulses measured
     * 
     * @details Calculates mean of all pulse measurements since last call to
     *          get_pwm_avg_us(). Provides better noise rejection than get_pwm_us()
     *          for jittery signals.
     *          
     *          **Averaging Behavior**:
     *          - Accumulates sum and count of pulses in interrupt handler
     *          - On call: Calculates average, resets accumulator
     *          - Returns 0 if no pulses since last call
     *          
     *          Thread-safe: Uses interrupt disable/restore for atomic access.
     * 
     * @note Resets accumulator after each call - subsequent calls measure fresh average
     * @note Useful for filtering RC input glitches and electrical noise
     * @note If called at 10Hz, averages ~2 pulses for typical 50Hz RC signals
     * 
     * @see get_pwm_us() for instantaneous measurement
     */
    uint16_t get_pwm_avg_us();

private:
    /**
     * @brief Last measured pulse width (microseconds), copied from interrupt
     * 
     * @details Updated by irq_handler() on each falling edge. Accessed by
     *          get_pwm_us() with interrupt protection.
     */
    uint16_t _irq_value_us;
    
    /**
     * @brief System timestamp of most recent rising edge (microseconds)
     * 
     * @details Used to calculate pulse width when falling edge occurs:
     *          pulse_width = falling_edge_time - _pulse_start_us
     */
    uint32_t _pulse_start_us;
    
    /**
     * @brief Currently attached GPIO pin number, or -1 if none
     */
    int16_t _pin = -1;

    /**
     * @brief Accumulator for pulse width averaging (sum of measurements in μs)
     * 
     * @details Incremented in interrupt handler, reset by get_pwm_avg_us()
     */
    uint32_t _irq_value_us_sum;
    
    /**
     * @brief Count of pulses for averaging
     * 
     * @details Incremented in interrupt handler, reset by get_pwm_avg_us()
     */
    uint32_t _irq_value_us_count;

    /**
     * @brief Flag indicating whether interrupt handler is currently attached
     */
    bool interrupt_attached;

    /**
     * @brief Interrupt callback for PWM edge detection
     * 
     * @param[in] pin GPIO pin number that triggered interrupt
     * @param[in] pin_state Current pin state: true=HIGH (rising), false=LOW (falling)
     * @param[in] timestamp System time in microseconds when edge occurred
     * 
     * @details Called by HAL interrupt dispatcher on both rising and falling edges.
     *          
     *          **Rising Edge** (pin_state=true):
     *          - Records timestamp in _pulse_start_us for pulse width calculation
     *          
     *          **Falling Edge** (pin_state=false):
     *          - Calculates pulse_width = timestamp - _pulse_start_us
     *          - Updates _irq_value_us for get_pwm_us()
     *          - Accumulates into _irq_value_us_sum/_irq_value_us_count for averaging
     * 
     * @note Called from interrupt context - must be fast and avoid blocking
     * @warning Do not call directly - managed internally by PWMSource
     */
    void irq_handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);
};

/**
 * @class AP_HAL::GPIO
 * @brief Abstract interface for general-purpose digital I/O pin operations
 * 
 * @details Provides platform-independent control of digital I/O pins for simple
 *          on/off outputs (LEDs, relays, indicators) and inputs (switches, digital
 *          sensors, RC receivers). Part of the Hardware Abstraction Layer (HAL)
 *          that isolates vehicle code from hardware-specific details.
 *          
 *          **Pin Numbering**:
 *          Pin numbers are HAL-specific identifiers defined in:
 *          - ChibiOS: Board hwdef files (libraries/AP_HAL_ChibiOS/hwdef/*)
 *          - Linux: Platform-specific headers (GPIO chip + offset)
 *          - SITL: Simulated pin numbers
 *          - NOT Arduino pin numbers - direct hardware identifiers
 *          
 *          **Pin Modes**:
 *          - HAL_GPIO_INPUT (0): High-impedance input for reading digital signals
 *          - HAL_GPIO_OUTPUT (1): Push-pull output for driving loads
 *          - HAL_GPIO_ALT (2): Alternate function (UART, SPI, I2C, PWM, etc.)
 *          
 *          **Typical Use Cases**:
 *          - LED indicators (arming status, GPS lock, errors)
 *          - Relay control (parachute deployment, camera trigger)
 *          - Digital sensor inputs (switches, optical sensors)
 *          - RC receiver PWM decoding (via PWMSource class)
 *          
 *          **Interrupt Support**:
 *          Enables edge-triggered callbacks for:
 *          - RC receiver pulse decoding (PWMSource)
 *          - RPM sensors (frequency counting)
 *          - External event detection (button presses, sensor triggers)
 *          
 *          **Thread Safety**:
 *          GPIO operations are generally NOT thread-safe. Use hal.scheduler
 *          semaphores or interrupt disable/restore when accessing from multiple
 *          threads or interrupt contexts.
 * 
 * @note Pure virtual interface - each platform HAL provides concrete implementation
 * @note Pin numbers are platform-specific - check hwdef files for valid ranges
 * @warning Writing to invalid pin numbers may have no effect or cause system crashes
 * @warning Exceeding pin current limits can damage hardware (typically 4-25mA max)
 * 
 * @see AP_HAL::PWMSource for PWM input measurement
 * @see AP_HAL::DigitalSource for object-oriented single-pin interface
 * @see AP_HAL_ChibiOS::GPIO for ARM/ChibiOS implementation
 * @see AP_HAL_Linux::GPIO for Linux implementation
 */
class AP_HAL::GPIO {
public:
    GPIO() {}
    
    /**
     * @brief Initialize GPIO hardware subsystem
     * 
     * @details Called once during HAL initialization to set up GPIO hardware,
     *          configure default pin states, and prepare interrupt handling.
     *          Platform-specific implementation may configure clock gates,
     *          GPIO registers, and interrupt controllers.
     * 
     * @note Called automatically by HAL::init() - do not call from vehicle code
     * @note Must be called before any other GPIO operations
     */
    virtual void    init() = 0;
    
    /**
     * @brief Configure pin direction and mode
     * 
     * @param[in] pin GPIO pin number (HAL-specific identifier from hwdef)
     * @param[in] output Pin mode: HAL_GPIO_INPUT (0), HAL_GPIO_OUTPUT (1), HAL_GPIO_ALT (2)
     * 
     * @details Configures pin electrical characteristics:
     *          - HAL_GPIO_INPUT: High-impedance input, does not drive pin
     *          - HAL_GPIO_OUTPUT: Push-pull output, can source/sink current
     *          - HAL_GPIO_ALT: Controlled by hardware peripheral (UART, SPI, etc.)
     *          
     *          Must be called before read() or write() operations on the pin.
     *          Can be called multiple times to reconfigure pin dynamically.
     * 
     * @note Pull-up/pull-down resistors are platform-specific and may require
     *       separate configuration via platform-specific GPIO extensions
     * @note Output drive strength is platform-specific (typically 2-25mA)
     * @warning Configuring a pin as output while external circuit drives it can
     *          cause hardware damage due to current conflict
     * 
     * @see HAL_GPIO_INPUT, HAL_GPIO_OUTPUT, HAL_GPIO_ALT
     */
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;

    /**
     * @brief Configure pin mode with specific alternate function selection
     * 
     * @param[in] pin GPIO pin number (HAL-specific identifier)
     * @param[in] output Pin mode: HAL_GPIO_INPUT, HAL_GPIO_OUTPUT, or HAL_GPIO_ALT
     * @param[in] alt Alternate function number (platform-specific, e.g., STM32 AF0-AF15)
     * 
     * @details Optional extended interface available on some platforms (primarily STM32)
     *          that support multiple alternate functions per pin. The 'alt' parameter
     *          selects which hardware peripheral controls the pin when output=HAL_GPIO_ALT.
     *          
     *          **STM32 Example**: Pin PA9 might support:
     *          - AF1: TIM1 (timer output)
     *          - AF7: USART1_TX (UART transmit)
     *          - AF10: OTG_FS (USB)
     * 
     * @note Not implemented on all platforms - check HAL implementation
     * @note Alternate function numbers are chip-specific (consult datasheet)
     * @note Default implementation does nothing (for platforms without AF selection)
     * 
     * @see pinMode(uint8_t, uint8_t) for standard two-parameter version
     */
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    /**
     * @brief Read current digital state of input pin
     * 
     * @param[in] pin GPIO pin number to read
     * 
     * @return Pin state: 0=LOW, 1=HIGH (or non-zero for HIGH)
     * 
     * @details Reads instantaneous voltage level on pin and returns digital logic level.
     *          
     *          **Electrical Thresholds** (typical for 3.3V logic):
     *          - LOW: 0V to ~0.8V (below V_IL threshold)
     *          - HIGH: ~2.0V to 3.3V (above V_IH threshold)
     *          - Undefined: 0.8V to 2.0V (may read as either 0 or 1)
     *          
     *          **Use Cases**:
     *          - Reading switch states (with pull-up/pull-down)
     *          - Digital sensor inputs (optical, hall effect, etc.)
     *          - Logic level detection from external circuits
     * 
     * @note Pin must be configured as input via pinMode(pin, HAL_GPIO_INPUT) first
     * @note Reading output pins returns the driven state, not actual pin voltage
     * @note Voltage thresholds are platform-specific (3.3V or 5V logic levels)
     * @warning Reading pins in alternate function mode returns undefined values
     * 
     * @see pinMode() to configure pin as input
     */
    virtual uint8_t read(uint8_t pin) = 0;
    
    /**
     * @brief Write digital value to output pin
     * 
     * @param[in] pin GPIO pin number to write
     * @param[in] value Pin state: 0=LOW (0V), non-zero=HIGH (VCC)
     * 
     * @details Drives pin to specified logic level.
     *          
     *          **Electrical Behavior**:
     *          - value=0: Pin driven to 0V (ground)
     *          - value≠0: Pin driven to VCC (typically 3.3V or 5V)
     *          - Push-pull mode: Can both source and sink current
     *          
     *          **Current Limits** (typical):
     *          - STM32: 25mA per pin, 120mA total per port
     *          - Linux GPIO: Depends on hardware (often 4-8mA)
     *          - Exceeding limits can damage GPIO port or MCU
     *          
     *          **Use Cases**:
     *          - LED control (with current-limiting resistor)
     *          - Relay driving (via transistor/MOSFET driver)
     *          - Logic signal generation for peripherals
     * 
     * @note Pin must be configured as output via pinMode(pin, HAL_GPIO_OUTPUT) first
     * @note Any non-zero value is treated as HIGH (both 1 and 255 produce same result)
     * @warning Do not exceed platform-specific current limits - use external drivers for high-current loads
     * @warning Writing to input pins has no effect (pin remains high-impedance)
     * 
     * @see pinMode() to configure pin as output
     * @see toggle() for atomic state inversion
     */
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    
    /**
     * @brief Toggle output pin state atomically
     * 
     * @param[in] pin GPIO pin number to toggle
     * 
     * @details Changes pin state from LOW to HIGH or HIGH to LOW in a single atomic
     *          operation. More efficient than read-modify-write sequence and avoids
     *          race conditions in interrupt-heavy environments.
     *          
     *          **Implementation**: Most platforms use hardware bit-toggle registers
     *          for true atomic operation without disabling interrupts.
     *          
     *          **Use Cases**:
     *          - LED blinking in fast loops
     *          - Heartbeat indicators
     *          - Debug signal generation (oscilloscope trigger)
     *          - Generating clock signals
     * 
     * @note Pin must be configured as output via pinMode(pin, HAL_GPIO_OUTPUT) first
     * @note Atomic on most platforms (no race conditions with interrupts)
     * @note More efficient than write(pin, !read(pin)) pattern
     * @warning Toggling pins in alternate function mode is undefined
     * 
     * @see write() for setting specific states
     */
    virtual void    toggle(uint8_t pin) = 0;
    
    /**
     * @brief Check if pin number is valid for this platform
     * 
     * @param[in] pin GPIO pin number to validate
     * 
     * @return true if pin exists and is accessible, false if invalid
     * 
     * @details Validates pin number against platform-specific valid pin ranges.
     *          Useful for runtime checking before attempting GPIO operations.
     *          
     *          **Default Implementation**: Returns true (assumes all pins valid).
     *          **Platform Implementations**: Check against hwdef configuration and
     *          hardware capabilities.
     * 
     * @note Invalid pin operations may crash or silently fail depending on HAL
     * @note Default implementation returns true - override for proper validation
     */
    virtual bool    valid_pin(uint8_t pin) const { return true; }

    /**
     * @brief Map GPIO pin to servo output channel number
     * 
     * @param[in]  pin GPIO pin number
     * @param[out] servo_ch Servo channel index (zero-based) if mapping exists
     * 
     * @return true if pin is mapped to servo channel (servo_ch filled in), false if no mapping
     * 
     * @details Some GPIO pins are multiplexed with servo/PWM output channels.
     *          This method determines if a GPIO pin corresponds to a servo output
     *          and returns the zero-based channel number.
     *          
     *          **Use Case**: Determining if GPIO operations conflict with servo outputs.
     *          If a pin is configured as GPIO, the corresponding servo output may be
     *          disabled or vice versa.
     * 
     * @note servo_ch uses zero-based indexing (0 = first servo channel)
     * @note Default implementation returns false (no GPIO/servo pin sharing)
     * @note ChibiOS platforms may support this via hwdef SERVO pin definitions
     * 
     * @see SRV_Channel library for servo output management
     */
    virtual bool    pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const { return false; }

    /**
     * @brief Save current pin mode and configuration
     * 
     * @param[in]  pin GPIO pin number
     * @param[out] mode Platform-specific mode encoding (direction, pull-up, drive strength, etc.)
     * 
     * @return true if mode successfully saved, false if not supported or invalid pin
     * 
     * @details Captures complete pin configuration including direction, pull resistors,
     *          drive strength, and alternate function settings. Used to temporarily
     *          reconfigure a pin and later restore original settings.
     *          
     *          **Use Case**: Bootloader updates, firmware flashing via GPIO pins,
     *          temporary peripheral mode changes.
     *          
     *          Mode encoding is platform-specific (e.g., STM32 GPIO register values).
     * 
     * @note Default implementation returns false (not supported)
     * @note Mode format is platform-specific - do not assume portability
     * 
     * @see set_mode() to restore saved configuration
     */
    virtual bool    get_mode(uint8_t pin, uint32_t &mode) { return false; }
    
    /**
     * @brief Restore previously saved pin mode and configuration
     * 
     * @param[in] pin GPIO pin number
     * @param[in] mode Platform-specific mode encoding from get_mode()
     * 
     * @details Restores pin configuration saved by get_mode(). Mode value is
     *          platform-specific and should only be used with the same platform
     *          that generated it.
     * 
     * @note Default implementation does nothing (not supported)
     * @note Mode must come from previous get_mode() call on same platform
     * @warning Using mode values from different platforms causes undefined behavior
     * 
     * @see get_mode() to capture current configuration
     */
    virtual void    set_mode(uint8_t pin, uint32_t mode) {}

    /**
     * @brief Get DigitalSource interface for specified pin
     * 
     * @param[in] n Pin number (interpretation varies by platform)
     * 
     * @return Pointer to DigitalSource object for this pin, never NULL
     * 
     * @details Provides object-oriented interface to individual pins through
     *          DigitalSource abstraction. Useful for passing pin control to
     *          other modules without exposing entire GPIO interface.
     *          
     *          **Use Cases**:
     *          - Abstraction: Pass pin control to libraries without GPIO dependency
     *          - Encapsulation: Restrict access to single pin
     *          - Compatibility: Support legacy code using DigitalSource interface
     * 
     * @note Pin numbering 'n' may differ from GPIO pin numbers (platform-specific)
     * @note Returned pointer is managed by HAL - do not delete
     * @note Some platforms may return dummy implementation for unsupported pins
     * 
     * @see AP_HAL::DigitalSource for alternative pin interface
     */
    virtual AP_HAL::DigitalSource* channel(uint16_t n) = 0;

    /**
     * @enum INTERRUPT_TRIGGER_TYPE
     * @brief Edge detection modes for GPIO interrupts
     * 
     * @details Specifies which pin state transitions trigger interrupt callbacks.
     *          Used for RC receiver decoding, RPM sensors, and external event detection.
     */
    enum INTERRUPT_TRIGGER_TYPE {
        /**
         * @brief No interrupt (used to detach interrupt handler)
         */
        INTERRUPT_NONE,
        
        /**
         * @brief Trigger on falling edge (HIGH to LOW transition)
         * 
         * @details Callback invoked when pin voltage drops below threshold.
         *          Typical uses: Active-low signals, button releases, pulse end detection.
         */
        INTERRUPT_FALLING,
        
        /**
         * @brief Trigger on rising edge (LOW to HIGH transition)
         * 
         * @details Callback invoked when pin voltage rises above threshold.
         *          Typical uses: Active-high signals, button presses, pulse start detection.
         */
        INTERRUPT_RISING,
        
        /**
         * @brief Trigger on both rising and falling edges
         * 
         * @details Callback invoked on any pin state change. Most common mode for
         *          PWM decoding (measure time between edges) and frequency counting.
         */
        INTERRUPT_BOTH,
    };

    /**
     * @brief Interrupt handler function signature with edge timestamp
     * 
     * @details Callback signature for GPIO interrupts with detailed edge information:
     * 
     * @param pin GPIO pin number that triggered interrupt
     * @param state Current pin state after edge: true=HIGH, false=LOW
     * @param timestamp System time in microseconds when edge occurred (from AP_HAL::micros64())
     * 
     * **Callback Context**:
     * - Called from interrupt context (high priority, preempts normal code)
     * - Must execute quickly (typically <10μs on STM32, faster on Linux)
     * - Cannot call blocking functions (delays, I/O, logging)
     * - Cannot allocate memory or use complex locks
     * 
     * **Thread Safety**:
     * - Use interrupt_disable/restore for shared data access
     * - Avoid calling most ArduPilot functions from interrupt context
     * - Signal events to main code via flags or queues
     * 
     * **Timing Accuracy**:
     * - timestamp captured by hardware or low-level interrupt handler
     * - Minimal jitter (typically <5μs) on most platforms
     * - Resolution limited by system timer (1μs on STM32)
     * 
     * @note Functor typedef created via FUNCTOR_TYPEDEF macro
     * @warning Keep interrupt handlers short - long execution blocks other interrupts
     * @see PWMSource for example usage in RC pulse decoding
     */
    FUNCTOR_TYPEDEF(irq_handler_fn_t, void, uint8_t, bool, uint32_t);
    
    /**
     * @brief Attach interrupt handler with edge timestamps to GPIO pin
     * 
     * @param[in] pin GPIO pin number to monitor
     * @param[in] fn Interrupt callback function (receives pin, state, timestamp)
     * @param[in] mode Edge detection mode (RISING, FALLING, BOTH, or NONE to detach)
     * 
     * @return true if interrupt successfully attached, false on error
     * 
     * @details Registers callback to be invoked when specified edge occurs on pin.
     *          Callback receives accurate timestamp of edge for precise timing measurements.
     *          
     *          **Setup Sequence**:
     *          1. Configure pin as input via pinMode(pin, HAL_GPIO_INPUT)
     *          2. Call attach_interrupt() with callback and trigger mode
     *          3. Interrupt fires on each matching edge
     *          4. To disable: attach_interrupt(pin, nullptr, INTERRUPT_NONE)
     *          
     *          **Use Cases**:
     *          - RC receiver PWM decoding (INTERRUPT_BOTH for pulse width)
     *          - RPM sensors (INTERRUPT_RISING for revolution counting)
     *          - Button detection (INTERRUPT_FALLING with pull-up)
     *          - External event timestamping
     * 
     * @note Default implementation returns false (not supported on all platforms)
     * @note Only one handler per pin - subsequent calls replace previous handler
     * @note Pin must be configured as input before attaching interrupt
     * @warning Interrupt handler called from interrupt context - keep execution short
     * @warning Ensure pin is not floating (use pull-up/pull-down) to avoid noise triggers
     * 
     * @see irq_handler_fn_t for callback signature requirements
     * @see PWMSource for RC receiver pulse decoding example
     * @see detach_interrupt() for safe interrupt removal
     */
    virtual bool    attach_interrupt(uint8_t pin,
                                     irq_handler_fn_t fn,
                                     INTERRUPT_TRIGGER_TYPE mode) {
        return false;
    }

    /**
     * @brief Attach simple interrupt handler without edge information
     * 
     * @param[in] pin GPIO pin number to monitor
     * @param[in] proc Simple callback function (no parameters)
     * @param[in] mode Edge detection mode (RISING, FALLING, BOTH, or NONE)
     * 
     * @return true if interrupt successfully attached, false on error
     * 
     * @details Legacy interrupt interface without edge state or timestamp information.
     *          Callback receives no parameters and must read pin state manually if needed.
     *          
     *          **Limitations**:
     *          - No timestamp: Cannot measure precise edge timing
     *          - No state: Must read pin manually (may have changed already)
     *          - Less useful for pulse width measurement
     *          
     *          **Use Cases**:
     *          - Simple event notification (button pressed)
     *          - Triggering deferred processing
     *          - Legacy code compatibility
     * 
     * @note Prefer irq_handler_fn_t version for new code (provides timestamp and state)
     * @note Default implementation returns false (not supported)
     * @warning Callback executes in interrupt context - same restrictions apply
     * 
     * @see attach_interrupt(uint8_t, irq_handler_fn_t, INTERRUPT_TRIGGER_TYPE) for preferred interface
     */
    virtual bool    attach_interrupt(uint8_t pin,
                                     AP_HAL::Proc proc,
                                     INTERRUPT_TRIGGER_TYPE mode) {
        return false;
    }
    
    /**
     * @brief Detach interrupt handler from pin
     * 
     * @param[in] pin GPIO pin number to disable interrupts on
     * 
     * @return true if interrupt successfully detached, false if not attached or error
     * 
     * @details Removes interrupt handler from pin, preventing further callbacks.
     *          Safe to call even if no interrupt is attached.
     *          
     *          **Implementation**: Calls attach_interrupt() with nullptr callback
     *          and INTERRUPT_NONE mode. Tries both function pointer and Proc variants.
     * 
     * @note Safe to call multiple times
     * @note Pin remains configured as input - call pinMode() to change
     * 
     * @see attach_interrupt() to register interrupt handlers
     */
    bool detach_interrupt(uint8_t pin) {
        if (attach_interrupt(pin, (irq_handler_fn_t)nullptr, AP_HAL::GPIO::INTERRUPT_NONE)) {
            return true;
        }
        return attach_interrupt(pin, (AP_HAL::Proc)nullptr, AP_HAL::GPIO::INTERRUPT_NONE);
    }

    /**
     * @brief Block waiting for pin state change with timeout
     * 
     * @param[in] pin GPIO pin number to monitor
     * @param[in] mode Edge type to wait for (RISING, FALLING, or BOTH)
     * @param[in] timeout_us Maximum wait time in microseconds (0 = wait forever)
     * 
     * @return true if pin changed state, false if timeout expired
     * 
     * @details Blocking function that waits for specified edge on pin. Useful for
     *          synchronizing with external events without using interrupts.
     *          
     *          **Behavior**:
     *          - Returns immediately if edge detected
     *          - Blocks calling thread until edge or timeout
     *          - timeout_us=0 waits indefinitely (dangerous - use with caution)
     *          
     *          **Implementation**: Platform-specific, may use:
     *          - Polling loop (SITL, simple platforms)
     *          - Interrupt with semaphore (efficient on RTOS platforms)
     *          - poll() system call (Linux)
     *          
     *          **Use Cases**:
     *          - Waiting for external ready signal
     *          - Synchronizing with hardware timing
     *          - Simple edge detection without interrupt overhead
     * 
     * @note Default implementation returns false (not supported)
     * @note Pin must be configured as input first
     * @note Blocks calling thread - do not call from time-critical code
     * @warning timeout_us=0 can hang forever if edge never occurs
     * @warning Not suitable for high-frequency signals (use attach_interrupt instead)
     * 
     * @see attach_interrupt() for non-blocking edge detection
     */
    virtual bool wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us) { return false; }

    /**
     * @brief Check if USB cable is connected to board
     * 
     * @return true if USB connected and enumerated, false otherwise
     * 
     * @details Detects USB connection state for determining if board is powered via USB
     *          or connected to ground station. Detection mechanism varies by platform:
     *          
     *          **Detection Methods**:
     *          - **VBUS Sense**: Hardware pin detects USB 5V presence (most reliable)
     *          - **USB Enumeration**: Checks if USB stack has enumerated with host
     *          - **DP/DM Pull**: Detects USB data line activity
     *          
     *          **Use Cases**:
     *          - Enabling USB serial console vs telemetry port
     *          - Preventing arming when connected via USB (safety)
     *          - Adjusting power management (USB provides 500mA)
     *          - Conditional logging via USB serial
     *          
     *          **Timing**:
     *          - May take 100-500ms after physical connection to return true
     *          - Immediately returns false on physical disconnection
     * 
     * @note Detection mechanism is platform-specific
     * @note Some boards without VBUS sense may always return false
     * @note USB serial activity does not guarantee this returns true
     * @warning Do not rely on this for safety-critical arming checks alone
     * 
     * @see AP_Arming for complete arming check implementation
     */
    virtual bool    usb_connected(void) = 0;

    /**
     * @brief Optional periodic timer tick for GPIO maintenance
     * 
     * @details Called periodically by scheduler (typically 1kHz) to perform platform-specific
     *          GPIO maintenance tasks such as:
     *          - Debouncing input pins
     *          - LED blinking sequences
     *          - Watchdog servicing
     *          - Polling-based pin monitoring
     *          
     *          Default implementation does nothing. Platform HALs override if needed.
     * 
     * @note Called from scheduler context, not interrupt context
     * @note Execution time should be minimal (< 50μs typical)
     * @note Not all platforms require timer_tick implementation
     */
    virtual void timer_tick(void) {};

    /**
     * @brief Pre-arming safety checks for GPIO subsystem
     * 
     * @param[in]  buflen Size of error message buffer
     * @param[out] buffer Buffer for error message (if check fails)
     * 
     * @return true if all GPIO checks pass, false if failure (message in buffer)
     * 
     * @details Called during vehicle arming sequence to verify GPIO subsystem is safe
     *          for flight. Checks may include:
     *          - Critical pins configured correctly
     *          - No pin conflicts detected
     *          - Hardware initialization completed
     *          - Safety-critical outputs in safe state
     *          
     *          Default implementation returns true (no checks). Platform HALs or
     *          board-specific code override to implement safety validation.
     *          
     *          **Use Cases**:
     *          - Detecting pin configuration errors before flight
     *          - Validating safety-critical output states
     *          - Checking for hardware faults
     * 
     * @note Called during arming - may block briefly
     * @note Error message should be concise (fits in buflen bytes)
     * @note Default implementation always passes
     * 
     * @see AP_Arming for overall arming check framework
     */
    virtual bool arming_checks(size_t buflen, char *buffer) const { return true; }

};
