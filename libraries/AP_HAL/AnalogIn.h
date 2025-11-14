/**
 * @file AnalogIn.h
 * @brief Analog input interface for ADC channels and voltage monitoring
 * 
 * @details Defines the HAL interface for analog input operations including ADC channel reading,
 *          voltage measurement, and power supply monitoring. Platform-specific implementations
 *          provide hardware ADC drivers that implement these pure virtual interfaces.
 *          
 *          The analog input subsystem supports:
 *          - Multi-channel ADC sampling with averaging
 *          - Voltage scaling with configurable references
 *          - Ratiometric sensor support
 *          - Board power supply monitoring
 *          - Servo rail voltage sensing
 *          - Power status flag reporting
 *          
 *          Common applications include battery voltage/current monitoring, analog sensor
 *          inputs (pressure, temperature), and system health monitoring.
 * 
 * @note Platform implementations: AP_HAL_ChibiOS, AP_HAL_Linux, AP_HAL_ESP32, AP_HAL_SITL
 * @see AP_HAL::AnalogIn for the subsystem manager interface
 * @see AP_HAL::AnalogSource for single channel interface
 */

#pragma once

#include <inttypes.h>

#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"

/**
 * @class AP_HAL::AnalogSource
 * @brief Interface for a single analog input channel
 * 
 * @details Represents one ADC channel that can be sampled to read analog voltages.
 *          Supports both raw ADC readings (0.0-1.0 normalized) and scaled voltage
 *          outputs with averaging to reduce noise.
 *          
 *          Averaging is typically performed over multiple samples collected at the
 *          platform-specific ADC sampling rate (often 10-100Hz). The averaging window
 *          varies by platform but is generally 50-200ms.
 *          
 *          Common use cases:
 *          - Battery voltage monitoring via voltage dividers
 *          - Current sensing through shunt resistors
 *          - Analog sensor inputs (pressure transducers, temperature sensors)
 *          - Board voltage monitoring for brownout detection
 *          
 *          Voltage scaling assumes a reference voltage (typically 3.3V or 5.0V depending
 *          on the microcontroller) and applies platform-specific calibration factors.
 *          
 * @note Implementations must be thread-safe if called from multiple contexts
 * @note ADC resolution is platform-specific (typically 10-16 bits)
 * @warning Voltage scaling factors are board-specific and configured in hwdef files
 * @warning Reading frequencies should not exceed platform ADC sampling rate
 */
class AP_HAL::AnalogSource {
public:
    /**
     * @brief Read averaged ADC value over recent samples
     * 
     * @details Returns the average of recent ADC samples, typically collected over the
     *          last 50-200ms depending on platform implementation and sampling rate.
     *          Averaging reduces noise and provides more stable readings for slowly-changing
     *          signals like battery voltage or pressure sensors.
     *          
     *          The returned value is normalized relative to the ADC full-scale range,
     *          where 0.0 represents 0V input and 1.0 represents the reference voltage
     *          (typically 3.3V or 5.0V depending on platform).
     * 
     * @return float Normalized ADC value in range 0.0 to 1.0, where 1.0 represents
     *         full-scale (reference voltage)
     * 
     * @note This is the preferred method for most analog input readings due to noise reduction
     * @note Averaging window size and sample rate are platform-specific
     * @note Thread-safe in all HAL implementations
     * 
     * @see voltage_average() for direct voltage reading with scaling applied
     */
    virtual float read_average() = 0;

    /**
     * @brief Read latest single ADC sample without averaging
     * 
     * @details Returns the most recent ADC sample without averaging. Useful for
     *          high-frequency signals or when instantaneous values are needed, but
     *          will be noisier than read_average().
     *          
     *          The returned value is normalized to 0.0-1.0 range like read_average().
     * 
     * @return float Normalized ADC value in range 0.0 to 1.0 from most recent sample
     * 
     * @note More susceptible to noise than read_average()
     * @note Use read_average() unless instantaneous readings are specifically required
     * @note Update rate matches platform ADC sampling rate (typically 10-100Hz)
     * 
     * @see read_average() for noise-reduced averaged reading
     */
    virtual float read_latest() = 0;

    /**
     * @brief Set the ADC pin number for this analog source
     * 
     * @details Configures which hardware ADC pin this AnalogSource object reads from.
     *          Pin numbers are platform-specific and defined in board hwdef files.
     *          Not all platforms support dynamic pin reassignment.
     *          
     *          Special pin values:
     *          - ANALOG_INPUT_BOARD_VCC (254): Board VCC voltage monitoring
     *          - ANALOG_INPUT_NONE (255): Disable analog input
     * 
     * @param[in] p Pin number to assign (0-253 for hardware pins, 254-255 for special values)
     * 
     * @return bool true if pin assignment successful, false if invalid pin or not supported
     * 
     * @note Pin numbering is board-specific - refer to hwdef files for valid pin numbers
     * @note Some platforms do not support runtime pin changes (will return false)
     * @warning Changing pins at high frequency may cause initialization overhead
     * @warning Must check return value to ensure pin was successfully assigned
     * 
     * @see ANALOG_INPUT_BOARD_VCC for board voltage monitoring
     * @see ANALOG_INPUT_NONE to disable input
     */
    virtual bool set_pin(uint8_t p) WARN_IF_UNUSED = 0;

    /**
     * @brief Read averaged voltage with scaling applied
     * 
     * @details Returns the averaged ADC reading converted to actual voltage in volts.
     *          Applies platform-specific voltage scaling based on reference voltage and
     *          any voltage dividers present in the hardware design.
     *          
     *          The voltage range depends on the input circuit design:
     *          - Direct ADC inputs: 0.0V to reference voltage (typically 3.3V or 5.0V)
     *          - With voltage divider: Can measure higher voltages (e.g., 0-50V for battery)
     *          
     *          Scaling factors are defined in board hwdef files and account for voltage
     *          dividers, reference voltage, and any calibration offsets.
     * 
     * @return float Voltage reading in volts, scaled according to hardware design
     * 
     * @note Voltage range and scaling are board-specific
     * @note Uses averaged samples for noise reduction (same averaging as read_average())
     * @note Calibration factors applied from board configuration
     * @warning Scaling assumes correct reference voltage configuration
     * 
     * @see voltage_latest() for non-averaged voltage reading
     * @see voltage_average_ratiometric() for ratiometric sensor support
     */
    virtual float voltage_average() = 0;

    /**
     * @brief Read latest voltage sample with scaling applied
     * 
     * @details Returns the most recent ADC sample converted to voltage. Like read_latest(),
     *          this provides instantaneous readings without averaging, making it more
     *          susceptible to noise.
     *          
     *          Uses the same voltage scaling as voltage_average().
     * 
     * @return float Latest voltage reading in volts, scaled according to hardware design
     * 
     * @note More susceptible to noise than voltage_average()
     * @note Use voltage_average() unless instantaneous voltage readings are required
     * @note Scaling factors same as voltage_average()
     * 
     * @see voltage_average() for noise-reduced averaged voltage reading
     * @see read_latest() for normalized ADC reading
     */
    virtual float voltage_latest() = 0;

    /**
     * @brief Read averaged voltage for ratiometric sensors
     * 
     * @details Returns voltage reading suitable for ratiometric sensors that are powered
     *          from the same reference as the ADC. Ratiometric sensors maintain a constant
     *          ratio to their supply voltage, making their output independent of actual
     *          supply voltage variations.
     *          
     *          Common ratiometric sensors:
     *          - Potentiometers powered from ADC reference
     *          - Pressure sensors with ratiometric output
     *          - Hall effect sensors with ratiometric mode
     *          
     *          The voltage calculation accounts for the ADC reference voltage, providing
     *          accurate readings even if the reference voltage varies.
     * 
     * @return float Voltage reading in volts, compensated for reference voltage variations
     * 
     * @note Only accurate for sensors powered from the same reference as the ADC
     * @note Uses averaged samples for noise reduction
     * @note For non-ratiometric sensors, use voltage_average() instead
     * @warning Incorrect use with non-ratiometric sensors will give inaccurate readings
     * 
     * @see voltage_average() for standard absolute voltage measurement
     */
    virtual float voltage_average_ratiometric() = 0;
};

/**
 * @class AP_HAL::AnalogIn
 * @brief Analog input subsystem manager
 * 
 * @details Manages the platform analog-to-digital converter (ADC) subsystem and provides
 *          access to individual analog input channels. Responsible for ADC initialization,
 *          channel allocation, and system-wide voltage monitoring.
 *          
 *          The AnalogIn manager provides:
 *          - ADC hardware initialization and configuration
 *          - Channel allocation via channel() method
 *          - Board power supply voltage monitoring
 *          - Servo rail voltage sensing
 *          - Power status flag reporting for fault detection
 *          - Optional MCU internal monitoring (temperature, voltage)
 *          
 *          Platform implementations configure ADC resolution, sampling rates, and
 *          available channels based on hardware capabilities defined in hwdef files.
 *          
 *          Typical initialization sequence:
 *          1. HAL calls init() during system startup
 *          2. Drivers request channels via channel() method
 *          3. Channels are sampled continuously in background
 *          4. Applications read values via AnalogSource interface
 * 
 * @note Accessed via AP::hal()->analogin singleton
 * @note ADC sampling typically runs in timer interrupt or DMA for consistent timing
 * @warning init() must be called before any channel operations
 * @warning Not all platforms support all features (e.g., servo rail voltage)
 * 
 * @see AP_HAL::AnalogSource for individual channel interface
 */
class AP_HAL::AnalogIn {
public:
    /**
     * @brief Initialize the analog input subsystem
     * 
     * @details Performs hardware initialization of the ADC peripheral, configures sampling
     *          rates, enables necessary clocks, and sets up interrupt/DMA handlers for
     *          continuous background sampling.
     *          
     *          Platform-specific initialization includes:
     *          - ADC peripheral clock enable and configuration
     *          - Pin muxing for analog input pins
     *          - DMA setup for efficient sample transfer
     *          - Timer configuration for periodic sampling
     *          - Calibration factor loading from storage
     *          
     *          This method is called once during HAL initialization before any channel
     *          access occurs.
     * 
     * @note Called automatically during HAL initialization
     * @note Must complete before any channel() calls
     * @warning Platform implementations must be non-blocking
     * @warning Failure to initialize may prevent all analog input operations
     */
    virtual void init() = 0;

    /**
     * @brief Get access to a specific analog input channel
     * 
     * @details Returns an AnalogSource interface for the requested ADC channel number.
     *          Channel numbers are platform-specific and defined in hwdef files. The
     *          returned pointer remains valid for the lifetime of the program.
     *          
     *          Some platforms support on-demand channel allocation, while others require
     *          all channels to be pre-configured. Invalid channel numbers may return
     *          nullptr or a dummy channel depending on implementation.
     *          
     *          Common channel allocations:
     *          - 0-7: General purpose ADC pins (varies by board)
     *          - Higher numbers: Internal channels (temp sensor, voltage reference)
     *          - ANALOG_INPUT_BOARD_VCC (254): Board VCC monitoring
     *          - ANALOG_INPUT_NONE (255): Null/disabled channel
     * 
     * @param[in] n Channel number to access (0-253 for hardware channels, 254-255 special)
     * 
     * @return AP_HAL::AnalogSource* Pointer to AnalogSource interface for this channel,
     *         or nullptr if channel is invalid
     * 
     * @note Channel numbers are board-specific - see hwdef files for valid channels
     * @note Returned pointer lifetime managed by HAL - do not delete
     * @note Multiple calls with same channel number return same AnalogSource instance
     * @warning nullptr return indicates invalid channel - always check before use
     * @warning Do not cache channel numbers across different board types
     * 
     * @see AP_HAL::AnalogSource for channel interface methods
     */
    virtual AP_HAL::AnalogSource* channel(int16_t n) = 0;

    /**
     * @brief Read board 5V rail voltage
     * 
     * @details Returns the current voltage of the main board power rail, typically a
     *          regulated 5V supply that powers the flight controller and peripherals.
     *          This voltage is monitored for brownout detection and power supply health.
     *          
     *          The measurement accounts for voltage dividers and calibration factors
     *          specific to each board design. Voltage below acceptable thresholds may
     *          trigger failsafe actions.
     *          
     *          On most boards this represents:
     *          - USB-powered: ~5.0V from USB
     *          - Battery-powered: Regulated 5V from onboard regulator
     *          - Both connected: Typically USB voltage (priority depends on design)
     * 
     * @return float Board voltage in volts, typically in range 4.5V-5.5V for healthy system
     * 
     * @note Updated continuously at ADC sampling rate
     * @note Critical for detecting brownout conditions and power supply problems
     * @note Some boards may return 0.0 if monitoring not implemented
     * @warning Voltage outside normal range (4.5-5.5V) indicates power supply issue
     * @warning Low voltage may cause processor resets or sensor failures
     * 
     * @see power_status_flags() for power supply health indicators
     */
    virtual float board_voltage(void) = 0;

    /**
     * @brief Read servo rail voltage
     * 
     * @details Returns the voltage of the servo power rail, which provides power to
     *          servos and ESCs. This rail may be separate from the board power supply
     *          and is often supplied by a BEC (Battery Eliminator Circuit) or separate
     *          power source.
     *          
     *          Not all boards have servo rail voltage monitoring capability. Boards
     *          without this feature return 0.0.
     *          
     *          Servo rail voltage monitoring is important for:
     *          - Detecting servo power supply failures
     *          - Monitoring BEC health
     *          - Validating adequate power for high-load servo operation
     *          - Detecting loose or disconnected servo power
     * 
     * @return float Servo rail voltage in volts, or 0.0 if not available/not implemented
     * 
     * @note Return value of 0.0 may indicate either no monitoring capability or no voltage present
     * @note Typical range when present: 4.8V-8.4V depending on servo power source
     * @note Updated at ADC sampling rate when supported
     * @warning Zero voltage with servos connected indicates power supply failure
     * 
     * @see power_status_flags() for SERVO_VALID status bit
     */
    virtual float servorail_voltage(void) { return 0; }

    /**
     * @brief Get current power supply status flags
     * 
     * @details Returns a bitmask of current power supply status indicators following
     *          the MAVLink MAV_POWER_STATUS enumeration. Flags indicate the health and
     *          configuration of various power sources and detect fault conditions like
     *          overcurrent.
     *          
     *          Status flags are used by:
     *          - Failsafe system for power-related fault detection
     *          - Ground control station for power monitoring displays
     *          - Logging system for post-flight power analysis
     *          - Pre-arm checks to validate adequate power before flight
     *          
     *          Flags are typically updated at 10-50Hz by platform-specific monitoring.
     * 
     * @return uint16_t Bitmask of current power status flags (see PowerStatusFlag enum),
     *         or 0 if power monitoring not implemented
     * 
     * @note Returns 0 on platforms without power monitoring capability
     * @note Flags match MAVLink MAV_POWER_STATUS enumeration for GCS compatibility
     * @note CHANGED flag indicates status has changed since boot
     * @warning OVERCURRENT flags indicate potentially unsafe operating conditions
     * 
     * @see PowerStatusFlag for flag definitions
     * @see accumulated_power_status_flags() for historical fault tracking
     */
    virtual uint16_t power_status_flags(void) { return 0; }

    /**
     * @brief Get accumulated power status flags since boot
     * 
     * @details Returns a bitmask of all power status flags that have been set at any
     *          point since boot. This allows detection and diagnosis of transient power
     *          problems that may have occurred briefly during flight.
     *          
     *          Accumulated flags are critical for:
     *          - Post-flight analysis of power glitches
     *          - Identifying intermittent power supply problems
     *          - Detecting brief overcurrent events that recovered
     *          - Validating power system reliability
     *          
     *          Once set, flags remain in the accumulated bitmask until next reboot,
     *          ensuring transient faults are not lost.
     * 
     * @return uint16_t Bitmask of all power status flags ever set since boot,
     *         or 0 if power monitoring not implemented
     * 
     * @note Flags never clear until reboot - allows forensic analysis
     * @note Critical for identifying intermittent power issues
     * @note Logged for post-flight review
     * @warning Accumulated overcurrent flags indicate power system may be undersized
     * 
     * @see power_status_flags() for current status
     * @see PowerStatusFlag for flag definitions
     */
    virtual uint16_t accumulated_power_status_flags(void) const { return 0; }

    /**
     * @enum PowerStatusFlag
     * @brief Power supply status indicator flags
     * 
     * @details Defines power supply status flags that match the MAVLink MAV_POWER_STATUS
     *          enumeration for ground control station compatibility. These flags indicate
     *          the current state of various power sources and detect fault conditions.
     *          
     *          Flags are returned as a bitmask by power_status_flags() and can be combined
     *          to indicate multiple conditions simultaneously.
     *          
     * @note This enumeration is 1:1 with MAVLink's MAV_POWER_STATUS enumeration
     * @note Values are bit positions, allowing combination via bitwise OR
     * @warning Do not modify values - must maintain MAVLink protocol compatibility
     * 
     * @see power_status_flags() to read current status
     * @see accumulated_power_status_flags() for historical status
     */
    enum class PowerStatusFlag : uint16_t {
        /**
         * @brief Main power brick supply valid
         * 
         * Indicates the primary power module (power brick) is connected and providing
         * valid voltage/current. Cleared if brick is disconnected, under-voltage, or failed.
         */
        BRICK_VALID = 1,

        /**
         * @brief Servo rail power supply valid
         * 
         * Indicates the servo rail has adequate voltage present to power servos and ESCs.
         * Cleared if servo power is disconnected or below minimum threshold.
         */
        SERVO_VALID = 2,

        /**
         * @brief USB power connected
         * 
         * Indicates USB cable is connected and providing power. Used to distinguish
         * between bench testing (USB) and flight operation (battery). Some features
         * behave differently when USB powered.
         */
        USB_CONNECTED = 4,

        /**
         * @brief Peripheral supply overcurrent
         * 
         * Indicates the peripheral power supply has exceeded current limit and may have
         * triggered overcurrent protection. This typically affects external peripheral
         * ports and accessories. Indicates potential short circuit or overload condition.
         */
        PERIPH_OVERCURRENT = 8,

        /**
         * @brief High-power peripheral supply overcurrent
         * 
         * Indicates the high-power peripheral supply (typically for high-current devices
         * like video transmitters or cameras) has exceeded current limit. May indicate
         * short circuit, device failure, or insufficient power capacity.
         */
        PERIPH_HIPOWER_OVERCURRENT = 16,

        /**
         * @brief Power status changed since boot
         * 
         * Indicates any power status flag has transitioned since boot. Used to detect
         * power events like USB connection/disconnection, power brick switching, or
         * transient overcurrent events. Remains set once any change occurs.
         */
        CHANGED = 32,
    };

#if HAL_WITH_MCU_MONITORING
    /**
     * @brief Read microcontroller internal temperature
     * 
     * @details Returns the current die temperature of the microcontroller as measured
     *          by the internal temperature sensor. This feature is only available on
     *          platforms that define HAL_WITH_MCU_MONITORING and have an internal
     *          temperature sensor.
     *          
     *          MCU temperature monitoring is useful for:
     *          - Detecting thermal stress or inadequate cooling
     *          - Validating environmental temperature ranges
     *          - Diagnosing performance issues related to overheating
     *          - Pre-flight checks in extreme temperature environments
     *          
     *          Typical operating range: -40°C to +85°C for most microcontrollers,
     *          though safe operating range may be narrower.
     * 
     * @return float MCU die temperature in degrees Celsius, or 0.0 if not available
     * 
     * @note Only available when HAL_WITH_MCU_MONITORING is defined
     * @note Accuracy typically ±5°C - suitable for monitoring but not precision measurement
     * @note Updated at relatively slow rate (1-10Hz) as temperature changes slowly
     * @warning Temperatures above 80°C may indicate inadequate cooling
     * @warning Extended operation at temperature extremes may affect reliability
     */
    virtual float mcu_temperature(void) { return 0; }

    /**
     * @brief Read microcontroller current supply voltage
     * 
     * @details Returns the current voltage supplied to the microcontroller's VCC pin,
     *          as measured by internal voltage monitoring. This is typically a regulated
     *          3.3V supply derived from the board 5V rail.
     *          
     *          MCU voltage monitoring helps detect:
     *          - Voltage regulator problems
     *          - Power supply instability
     *          - Inadequate input voltage causing regulator dropout
     *          - Board-level power distribution issues
     * 
     * @return float Current MCU supply voltage in volts (typically ~3.3V), or 0.0 if not available
     * 
     * @note Only available when HAL_WITH_MCU_MONITORING is defined
     * @note Should be stable at ~3.3V ±0.3V for healthy operation
     * @note Voltage outside normal range indicates power supply problems
     * @warning Low MCU voltage may cause processor resets or erratic behavior
     * 
     * @see mcu_voltage_min() for minimum voltage since boot
     * @see mcu_voltage_max() for maximum voltage since boot
     */
    virtual float mcu_voltage(void) { return 0; }

    /**
     * @brief Read maximum microcontroller voltage since boot
     * 
     * @details Returns the highest MCU supply voltage measured since boot. Used to
     *          detect transient voltage spikes or overvoltage conditions that may have
     *          occurred briefly during operation.
     *          
     *          Tracking maximum voltage helps identify:
     *          - Voltage regulator overshoot during transients
     *          - Power supply noise or instability
     *          - Potential overvoltage stress on the MCU
     * 
     * @return float Maximum MCU voltage in volts since boot, or 0.0 if not available
     * 
     * @note Only available when HAL_WITH_MCU_MONITORING is defined
     * @note Value never decreases, only updates when new maximum observed
     * @note Logged for post-flight power system analysis
     * @warning Voltage significantly above 3.6V may indicate regulator failure
     * 
     * @see mcu_voltage() for current voltage
     * @see mcu_voltage_min() for minimum voltage
     */
    virtual float mcu_voltage_max(void) { return 0; }

    /**
     * @brief Read minimum microcontroller voltage since boot
     * 
     * @details Returns the lowest MCU supply voltage measured since boot. Critical for
     *          detecting voltage dips or brownout conditions that may have caused brief
     *          processor instability or resets.
     *          
     *          Tracking minimum voltage helps identify:
     *          - Voltage sags during high-current draw
     *          - Inadequate power supply capacity
     *          - Voltage regulator dropout conditions
     *          - Transient brownout events that may affect reliability
     *          
     *          Voltage below approximately 3.0V may cause processor resets or data
     *          corruption on many platforms.
     * 
     * @return float Minimum MCU voltage in volts since boot, or 0.0 if not available
     * 
     * @note Only available when HAL_WITH_MCU_MONITORING is defined
     * @note Value never increases, only updates when new minimum observed
     * @note Critical for diagnosing brownout-related failures
     * @warning Voltage below 3.0V indicates serious power supply issues
     * @warning Multiple brownouts may cause data corruption or system instability
     * 
     * @see mcu_voltage() for current voltage
     * @see mcu_voltage_max() for maximum voltage
     */
    virtual float mcu_voltage_min(void) { return 0; }
#endif
};

/**
 * @def ANALOG_INPUT_BOARD_VCC
 * @brief Special pin number for board VCC voltage monitoring
 * 
 * @details Use this value with AnalogSource::set_pin() to configure a channel to read
 *          the board VCC voltage (typically 5V rail). This provides an alternative to
 *          calling AnalogIn::board_voltage() directly.
 *          
 *          Some platforms use this as a standard channel number for internal voltage
 *          monitoring that doesn't correspond to a physical external pin.
 * 
 * @note Value 254 reserved for this purpose across all platforms
 * @see AnalogIn::board_voltage() for direct board voltage reading
 * @see AnalogSource::set_pin() for pin assignment
 */
#define ANALOG_INPUT_BOARD_VCC 254

/**
 * @def ANALOG_INPUT_NONE
 * @brief Special pin number indicating no analog input
 * 
 * @details Use this value with AnalogSource::set_pin() to disable analog input on a
 *          channel or to indicate that no pin is assigned. Readings from channels set
 *          to this value will typically return 0.0.
 *          
 *          Common uses:
 *          - Disabling unused analog input channels
 *          - Placeholder for unconfigured channels
 *          - Default value when no pin has been assigned
 * 
 * @note Value 255 reserved for this purpose across all platforms
 * @see AnalogSource::set_pin() for pin assignment
 */
#define ANALOG_INPUT_NONE 255
