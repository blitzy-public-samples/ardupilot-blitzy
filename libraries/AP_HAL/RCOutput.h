/**
 * @file RCOutput.h
 * @brief PWM output interface for motors, servos, and ESCs
 * 
 * @details Defines the abstract HAL interface for PWM/ESC output supporting multiple protocols including
 *          standard PWM, OneShot, DShot, and other high-speed ESC protocols. This interface manages output 
 *          channels, safety state, protocol-specific timing, and features like bi-directional DShot telemetry.
 *          
 *          Platform-specific implementations (ChibiOS, Linux, ESP32, SITL) provide the concrete hardware
 *          control while maintaining this common interface contract.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_HAL_Namespace.h"
#include <stdint.h>

/** @brief Minimum valid pulse width in microseconds for RC output */
#define RC_OUTPUT_MIN_PULSEWIDTH 400
/** @brief Maximum valid pulse width in microseconds for RC output */
#define RC_OUTPUT_MAX_PULSEWIDTH 2100

/** 
 * @brief Define channel index constants CH_1 through CH_32
 * @note Channel indices are zero-based internally (CH_1 = 0, CH_2 = 1, etc.)
 * @note CH_NONE (255) indicates no channel assigned
 */
#ifndef CH_1
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15
#define CH_17 16
#define CH_18 17
#define CH_19 18
#define CH_20 19
#define CH_21 20
#define CH_22 21
#define CH_23 22
#define CH_24 23
#define CH_25 24
#define CH_26 25
#define CH_27 26
#define CH_28 27
#define CH_29 28
#define CH_30 29
#define CH_31 30
#define CH_32 31
#define CH_NONE 255
#endif

class ByteBuffer;

class ExpandingString;

/**
 * @class AP_HAL::RCOutput
 * @brief Abstract interface for motor/servo output channels
 * 
 * @details Controls PWM outputs for motors, servos, and other actuators across multiple protocols.
 *          This is the core HAL interface for all vehicle motor and servo control.
 *          
 *          **Supported Output Protocols:**
 *          - **PWM**: Standard 50Hz servo PWM (1000-2000μs pulse width)
 *          - **OneShot125**: High-rate ESC protocol (125-250μs pulse, 8x faster than PWM)
 *          - **OneShot42**: Ultra-high-rate ESC protocol (42-84μs pulse)
 *          - **DShot150/300/600/1200**: Digital bi-directional ESC protocol (150-1200kHz)
 *          - **ProfiLED**: LED strip control using separate clock and data lines
 *          - **NeoPixel/NeoPixelRGB**: WS2812B LED protocol (800kHz)
 *          
 *          **Output Channel Allocation:**
 *          - Main outputs: Motors (copter/rover), control surfaces (plane), steering (rover)
 *          - Aux outputs: Gimbal servos, camera trigger, parachute, gripper, lights, misc peripherals
 *          - Up to 32 channels supported (hardware dependent)
 *          
 *          **Safety State Management:**
 *          - SAFETY_DISARMED: Outputs forced to safe values (motors off, servos at trim)
 *          - SAFETY_ARMED: Normal output control enabled
 *          - Hardware safety switch on many boards provides physical enable/disable
 *          
 *          **DShot Protocol Features:**
 *          - Bi-directional: ESC telemetry feedback (RPM, voltage, current, temperature, errors)
 *          - Digital signaling: No pulse width jitter, immune to EMI/interference
 *          - High update rate: 1-4kHz motor updates for responsive control
 *          - Commands: ESC configuration, beeps, direction control, LED control
 *          - Extended telemetry: Enhanced ESC data reporting (BLHeli32)
 *          
 *          **Cork/Push Mechanism:**
 *          Use cork() before multiple write() calls and push() to commit all changes atomically.
 *          This reduces latency and ensures synchronized multi-channel updates.
 *          
 *          **Serial Passthrough:**
 *          Supports BLHeli/SimonK ESC configuration and firmware flashing via serial passthrough
 *          to ESC (BLHeliSuite/BLHeli Configurator integration).
 *          
 *          **Thread Safety:**
 *          Methods are called from main vehicle thread at loop rate (typically 50-400Hz).
 *          Cork/push should be used to ensure atomic multi-channel updates.
 * 
 * @note Output limits (min/max/trim) are managed by SRV_Channel layer, not this HAL interface
 * @note Actual output may be overridden by safety state, failsafe, or geofence
 * 
 * @warning Writing outputs before arming checks complete can spin motors - extremely dangerous!
 * @warning Incorrect frequency settings can damage servos or cause motor sync loss
 * @warning DShot commands can change ESC direction - verify before flight
 * 
 * @see SRV_Channel for output function mapping and limit management
 * @see AP_Motors for multicopter motor mixing
 * @see AP_BLHeli for BLHeli ESC integration
 */
class AP_HAL::RCOutput {
public:
    /**
     * @brief Initialize PWM hardware timers, DMA, and output channels
     * 
     * @details Sets up hardware timer peripherals, configures GPIO pins for PWM output,
     *          initializes DMA channels for high-speed protocols (DShot, NeoPixel),
     *          and prepares output channels for use. Called once during HAL initialization.
     *          
     *          Implementation responsibilities:
     *          - Configure timer clock sources and prescalers
     *          - Set up GPIO alternate functions for PWM
     *          - Initialize DMA for efficient output updates
     *          - Set default output state (typically disabled/safe values)
     *          - Register with scheduler for periodic updates if needed
     * 
     * @note Must be called before any other RCOutput methods
     * @note Does not enable outputs - safety state controls actual output enable
     */
    virtual void init() = 0;

    /**
     * @brief Set PWM frequency for specified output channels
     * 
     * @details Configures the PWM update frequency for a group of channels. Channels sharing
     *          the same hardware timer must use the same frequency. Higher frequencies provide
     *          more responsive control but may increase CPU load.
     *          
     *          Typical frequencies by application:
     *          - Standard servos: 50Hz (20ms period)
     *          - Digital servos: 200-333Hz
     *          - Multicopter motors: 400-500Hz
     *          - OneShot: 1-4kHz
     *          - DShot: Protocol-specific (150-1200kHz equivalent)
     * 
     * @param[in] chmask Bitmask of channels to configure (bit 0 = channel 0, bit 1 = channel 1, etc.)
     * @param[in] freq_hz Desired output frequency in Hz
     * 
     * @note All channels in mask are set to the same frequency due to hardware timer sharing
     * @note Channels on same timer group must use compatible frequencies
     * @warning Changing frequency during armed state may cause motor glitches or servo jitter
     * @warning Frequencies above 500Hz may not work with standard servos
     * 
     * @see get_freq() to query current channel frequency
     */
    virtual void     set_freq(uint32_t chmask, uint16_t freq_hz) = 0;
    
    /**
     * @brief Get current PWM frequency for a channel
     * 
     * @param[in] chan Channel number (0-based)
     * @return Current output frequency in Hz, or 0 if channel not initialized
     */
    virtual uint16_t get_freq(uint8_t chan) = 0;

    /**
     * @brief Enable PWM output on a specific channel
     * 
     * @details Activates the output pin, allowing PWM signals to be generated. On some
     *          hardware, this switches the pin from high-impedance (tri-state) to active drive.
     * 
     * @param[in] chan Channel number (0-based) to enable
     * 
     * @note Channel must be initialized before enabling
     * @note Actual output still controlled by safety state
     */
    virtual void     enable_ch(uint8_t chan) = 0;
    
    /**
     * @brief Disable PWM output on a specific channel
     * 
     * @details Deactivates the output pin, typically setting it to high-impedance state.
     *          Used when output channel is not needed or conflicts with other pin functions.
     * 
     * @param[in] chan Channel number (0-based) to disable
     */
    virtual void     disable_ch(uint8_t chan) = 0;

    /**
     * @brief Write pulse width to an output channel
     * 
     * @details Sets the output pulse width for standard PWM/OneShot protocols, or throttle
     *          value for DShot protocol. Changes may be buffered if cork() was called and
     *          will be applied on next push().
     *          
     *          **Protocol-specific interpretation:**
     *          - PWM/OneShot: period_us is pulse width in microseconds (typically 1000-2000)
     *          - DShot: period_us is throttle value (48-2047, where 48 = zero throttle)
     *          - Servo outputs: period_us typically 1000-2000μs (may be limited by SRV_Channel)
     *          
     *          **Safety override:**
     *          Written value may be overridden based on safety state:
     *          - DISARMED: Motors forced to zero, servos to trim/last position
     *          - ARMED: Value written as specified
     *          - FAILSAFE: May use failsafe PWM values set by set_failsafe_pwm()
     * 
     * @param[in] chan Channel number (0-based, typically 0-31)
     * @param[in] period_us Pulse width in microseconds for PWM, or throttle value for DShot
     * 
     * @note For DShot: Values 0-47 reserved for commands, 48 = zero throttle, 49-2047 = throttle
     * @note Actual hardware output depends on safety state and may differ from written value
     * @note Use cork()/push() around multiple writes for atomic synchronized updates
     * 
     * @warning Writing motor outputs before arming can be dangerous - motors may spin
     * 
     * @see read() to retrieve last written value
     * @see cork() and push() for atomic multi-channel updates
     */
    virtual void     write(uint8_t chan, uint16_t period_us) = 0;

    /**
     * @brief Mark channels as supporting reversible output
     * 
     * @details Indicates channels can accept negative thrust/reverse direction. Required for
     *          some ESC types (DShot, reversible brushed) to properly scale bi-directional output.
     *          Used primarily for rover steering/throttle and 3D-capable multicopter ESCs.
     * 
     * @param[in] chanmask Bitmask of channels supporting reversible output (ORed with existing mask)
     * 
     * @note Mask uses servo channel numbering (matches SRV_Channel)
     * @note Only affects output scaling, does not enable hardware reverse capability
     */
    virtual void     set_reversible_mask(uint32_t chanmask) {}
    
    /**
     * @brief Mark channels as currently reversed
     * 
     * @details Sets channels to operate in reversed direction. For DShot, triggers DSHOT_REVERSE
     *          command to ESC. For servos, reverses output direction in SRV_Channel layer.
     * 
     * @param[in] chanmask Bitmask of channels to reverse (ORed with existing mask)
     * 
     * @note Mask uses servo channel numbering
     * @note For DShot ESCs, sends DSHOT_REVERSE command requiring motor restart
     * 
     * @see set_reversible_mask() to mark channels as reverse-capable
     */
    virtual void     set_reversed_mask(uint32_t chanmask) {}
    
    /**
     * @brief Get current reversed channel mask
     * 
     * @return Bitmask of reversed channels
     */
    virtual uint32_t get_reversed_mask() { return 0; }

    /**
     * @brief Update channel configuration masks
     * 
     * @details Called at 1Hz to process pending channel mask changes, send DShot commands
     *          (direction changes, LED control), and update ESC telemetry configuration.
     *          Used for operations that require time to take effect.
     * 
     * @note Called by scheduler at low rate (1Hz)
     */
    virtual void     update_channel_masks() {}

    /**
     * @brief Suspend channel mask updates temporarily
     * 
     * @details Prevents update_channel_masks() from making changes. Used during critical
     *          operations like ESC calibration or motor testing when mask changes would interfere.
     */
    virtual void     disable_channel_mask_updates() {}
    
    /**
     * @brief Re-enable channel mask updates
     * 
     * @details Resumes normal update_channel_masks() operation after being suspended.
     */
    virtual void     enable_channel_mask_updates() {}

    /**
     * @brief Buffer subsequent write() calls for atomic commit
     * 
     * @details Delays hardware output updates to allow multiple write() calls to be grouped
     *          into a single atomic transaction. Call push() to commit all buffered changes
     *          simultaneously. This reduces latency jitter and ensures synchronized multi-channel
     *          updates (critical for motor mixing on multicopters).
     *          
     *          Typical usage pattern:
     *          ```cpp
     *          rcout->cork();
     *          rcout->write(0, pwm1);  // Buffered
     *          rcout->write(1, pwm2);  // Buffered
     *          rcout->write(2, pwm3);  // Buffered
     *          rcout->push();          // All outputs updated simultaneously
     *          ```
     * 
     * @note Cork/push must be balanced - every cork() requires a push()
     * @note Nested cork() calls not supported
     * @warning Forgetting push() will prevent output updates
     * 
     * @see push() to commit buffered changes
     */
    virtual void     cork() = 0;

    /**
     * @brief Commit all buffered write() calls to hardware
     * 
     * @details Applies all write() calls made since last cork() atomically in a single
     *          transaction. For DMA-based implementations, triggers DMA transfer. For
     *          timer-based implementations, updates all timer compare registers together.
     * 
     * @note Must be called after cork() to apply buffered writes
     * @note May trigger DMA transfer or timer register updates
     * 
     * @see cork() to begin buffering writes
     */
    virtual void     push() = 0;

    /**
     * @brief Read current output state for a channel
     * 
     * @details Returns the actual output value being sent to hardware. On boards with separate
     *          IO coprocessors (e.g., Pixhawk with IOMCU), this returns the last value confirmed
     *          by the IO controller, which may lag slightly behind write() calls.
     *          
     *          Note: During safety-disarmed state, the read value may differ from the last
     *          write() value, as safety overrides may be applied.
     * 
     * @param[in] chan Channel number (0-based) to read
     * @return Current output value in microseconds, or 0 if channel not initialized
     * 
     * @note May return safety-overridden value, not the last write() value
     * @see read_last_sent() to get last written value regardless of safety state
     */
    virtual uint16_t read(uint8_t chan) = 0;
    
    /**
     * @brief Read current output state for multiple channels
     * 
     * @param[out] period_us Array to store output values in microseconds
     * @param[in]  len Number of channels to read
     * 
     * @note Array must be pre-allocated with at least len elements
     */
    virtual void     read(uint16_t* period_us, uint8_t len) = 0;

    /**
     * @brief Read last value written to a channel
     * 
     * @details Returns the last value passed to write(), regardless of safety state or
     *          failsafe overrides. This is the "commanded" value, not necessarily the actual
     *          hardware output.
     * 
     * @param[in] chan Channel number (0-based)
     * @return Last written value in microseconds
     * 
     * @note Default implementation calls read(), but may be overridden for boards with IO coprocessor
     * @see read() for actual hardware output value
     */
    virtual uint16_t read_last_sent(uint8_t chan) { return read(chan); }
    
    /**
     * @brief Read last written values for multiple channels
     * 
     * @param[out] period_us Array to store last written values
     * @param[in]  len Number of channels to read
     */
    virtual void     read_last_sent(uint16_t* period_us, uint8_t len) { read(period_us, len); };

    /**
     * @brief Set failsafe PWM values for channels
     * 
     * @details Configures PWM values to output if main firmware stops responding (watchdog timeout,
     *          crash, infinite loop). On boards with IO coprocessor, these values are held in the
     *          IO controller and automatically applied if FMU communication is lost.
     *          
     *          Typical failsafe values:
     *          - Motors: 900μs (below minimum throttle, motors off)
     *          - Servos: Neutral/trim position
     * 
     * @param[in] chmask Bitmask of channels to configure (bit 0 = ch0, etc.)
     * @param[in] period_us Failsafe pulse width in microseconds
     * 
     * @note Only implemented on boards with IO coprocessor or hardware watchdog
     * @warning Incorrect failsafe values can cause loss of control during firmware failure
     */
    virtual void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us) {}

    /**
     * @brief Force safety state to disarmed
     * 
     * @details Disables PWM outputs by activating safety state, regardless of physical safety
     *          switch position. Used to ensure motors/servos are disabled during configuration,
     *          firmware updates, or emergency stop conditions.
     * 
     * @return true if safety state successfully forced on, false if not supported by hardware
     * 
     * @note Default returns false for boards without safety switch
     * @warning This is a safety-critical function - must reliably disable all outputs
     * 
     * @see force_safety_off() to re-enable outputs
     */
    virtual bool     force_safety_on(void) { return false; }

    /**
     * @brief Force safety state to armed
     * 
     * @details Enables PWM outputs by deactivating safety state, overriding physical safety
     *          switch if present. Used during motor tests, ESC calibration, and bench testing.
     *          
     *          **DANGER:** This bypasses the physical safety switch intended to prevent
     *          accidental motor spin-up.
     * 
     * @warning Bypasses hardware safety switch - motors can spin without physical safety off!
     * @warning Only use in controlled test environments with proper safety precautions
     * 
     * @see force_safety_on() to disable outputs
     */
    virtual void     force_safety_off(void) {}

    /**
     * @brief Configure ESC throttle scaling range
     * 
     * @details Sets up PWM-to-percentage scaling for ESCs that accept percentage throttle commands
     *          (e.g., UAVCAN/DroneCAN ESCs). Write values are converted from PWM microseconds to
     *          percentage using: percentage = (pwm - min_pwm) / (max_pwm - min_pwm)
     *          
     *          Standard PWM ESCs receive pulse width directly. Protocol ESCs (UAVCAN, DShot) may
     *          use this scaling to convert PWM values to their native throttle range.
     * 
     * @param[in] min_pwm Minimum PWM value in microseconds (typically 1000μs)
     * @param[in] max_pwm Maximum PWM value in microseconds (typically 2000μs)
     * 
     * @note Only affects protocol-based ESCs (UAVCAN, some DShot implementations)
     * @note Standard PWM outputs ignore these settings
     * 
     * @see scale_esc_to_unity() to convert PWM value to normalized range
     */
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) {
        _esc_pwm_min = min_pwm;
        _esc_pwm_max = max_pwm;
    }

    /**
     * @brief Scale PWM value to normalized range [-1, 1]
     * 
     * @details Converts PWM microsecond value to normalized throttle using ESC scaling range
     *          set by set_esc_scaling(). Used for protocol ESCs expecting percentage commands.
     *          
     *          Formula: unity = (pwm - min) / (max - min) * 2 - 1
     *          
     *          Examples (with 1000-2000μs range):
     *          - 1000μs → -1.0 (full reverse, if supported)
     *          - 1500μs →  0.0 (neutral/zero throttle)
     *          - 2000μs →  1.0 (full forward)
     * 
     * @param[in] pwm Pulse width in microseconds
     * @return Normalized value in range [-1.0, 1.0], unconstrained
     * 
     * @note Returns value without constraints - may be outside [-1, 1] if pwm outside scaling range
     */
    float scale_esc_to_unity(uint16_t pwm) const;

    /**
     * @brief Get ESC RPM telemetry for a channel
     * 
     * @details Returns electrical RPM (eRPM) from ESC telemetry if available. For DShot
     *          bi-directional telemetry, this is motor pole pairs times mechanical RPM.
     *          Requires bi-directional DShot or other telemetry-capable ESC protocol.
     *          
     *          eRPM = mechanical_RPM * pole_pairs
     *          mechanical_RPM = eRPM / pole_pairs
     * 
     * @param[in] chan Channel number (0-based)
     * @return Electrical RPM from ESC, or 0 if telemetry not available
     * 
     * @note Requires set_bidir_dshot_mask() or similar telemetry enable
     * @note Convert to mechanical RPM using motor pole count: set_motor_poles()
     * 
     * @see set_motor_poles() to configure pole count for RPM conversion
     * @see get_erpm_error_rate() for telemetry reliability
     */
    virtual uint16_t get_erpm(uint8_t chan) const { return 0; }
    
    /**
     * @brief Get ESC telemetry error rate
     * 
     * @details Returns percentage of lost/corrupted telemetry packets for reliability assessment.
     *          Values near 0% indicate good telemetry. Values above 10% suggest electrical noise,
     *          signal integrity issues, or ESC firmware problems.
     * 
     * @param[in] chan Channel number (0-based)
     * @return Error rate percentage (0-100%), default 100% if no telemetry
     * 
     * @note 100% error rate indicates no telemetry received
     */
    virtual float get_erpm_error_rate(uint8_t chan) const { return 100.0f; }
    
    /**
     * @brief Check if new ESC telemetry data available
     * 
     * @details Returns true if ESC telemetry has been updated since last read. Used primarily
     *          with IO coprocessor to detect when IOMCU has new telemetry data.
     * 
     * @return true if new telemetry available, false otherwise
     */
    virtual bool  new_erpm() { return false; }
    
    /**
     * @brief Read ESC telemetry for multiple channels
     * 
     * @param[out] erpm Array to store eRPM values
     * @param[in]  len Number of channels to read
     * @return Bitmask of channels with valid telemetry
     * 
     * @note Primarily used with IOMCU to bulk-read telemetry
     */
    virtual uint32_t  read_erpm(uint16_t* erpm, uint8_t len) { return 0; }

    /**
     * @brief Enable SBUS output on PX4IO coprocessor
     * 
     * @details Configures PX4IO board to output SBUS protocol (inverted serial) for controlling
     *          servos or secondary flight controllers. SBUS carries 16 channels in serial protocol.
     * 
     * @param[in] rate_hz SBUS frame rate in Hz (typically 50-100Hz)
     * @return true if SBUS output enabled successfully, false if not supported
     * 
     * @note Only supported on boards with PX4IO coprocessor (Pixhawk 1, etc.)
     */
    virtual bool enable_px4io_sbus_out(uint16_t rate_hz) { return false; }

    /**
     * @brief Periodic timer callback for output processing
     * 
     * @details Optional callback invoked by scheduler for implementations requiring periodic
     *          processing (DShot telemetry collection, LED updates, etc.). Rate depends on
     *          platform implementation.
     * 
     * @note Not all implementations require timer_tick()
     * @note Typical call rate: 1kHz for DShot implementations
     */
    virtual void timer_tick(void) { }

    /**
     * @brief Configure serial passthrough to ESC
     * 
     * @details Sets up bidirectional serial communication for ESC configuration and firmware
     *          flashing (BLHeliSuite, BLHeli Configurator, SimonK). Reconfigures output pin(s)
     *          as UART for direct ESC communication.
     *          
     *          Serial format: 8N1 (8 data bits, no parity, 1 stop bit, LSB first)
     *          
     *          **While serial passthrough is active:**
     *          - Normal PWM output suspended on all channels in chanmask
     *          - Channels sharing timer groups may also be suspended (hardware limitation)
     *          - No motor/servo control available on affected channels
     * 
     * @param[in] chan Primary channel for serial communication (ESC connection)
     * @param[in] baudrate Serial baud rate (typically 19200 for BLHeli, 38400 for newer ESCs)
     * @param[in] chanmask Additional channels to suspend during serial operation
     * @return true if serial mode configured successfully, false if not supported
     * 
     * @warning Disables normal output on chanmask channels - do not use during flight!
     * @note Call serial_end() to restore normal PWM output
     * 
     * @see serial_write_bytes() and serial_read_bytes() for communication
     * @see serial_end() to restore normal operation
     */
    virtual bool serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t chanmask) { return false; }

    /**
     * @brief Write data to ESC via serial passthrough
     * 
     * @details Blocking write of data bytes to ESC using serial configuration from
     *          serial_setup_output(). Used for BLHeli/SimonK configuration commands
     *          and firmware upload.
     * 
     * @param[in] bytes Data buffer to transmit
     * @param[in] len Number of bytes to write
     * @return true if write successful, false on error or timeout
     * 
     * @note Blocking call - waits for all bytes to transmit
     * @note Must call serial_setup_output() first
     * 
     * @see serial_setup_output() to configure serial mode
     */
    virtual bool serial_write_bytes(const uint8_t *bytes, uint16_t len) { return false; }

    /**
     * @brief Read data from ESC via serial passthrough
     * 
     * @details Blocking read of response data from ESC. Waits up to timeout for data,
     *          returns when buffer full or timeout expires.
     * 
     * @param[out] buf Buffer to store received data
     * @param[in]  len Maximum bytes to read
     * @param[in]  timeout_us Timeout in microseconds
     * @return Number of bytes actually read (0 to len)
     * 
     * @note Blocking call - waits up to timeout_us
     * @note Must call serial_setup_output() first
     * 
     * @see serial_setup_output() to configure serial mode
     */
    virtual uint16_t serial_read_bytes(uint8_t *buf, uint16_t len, uint32_t timeout_us) { return 0; }
    
    /**
     * @brief End serial passthrough and restore PWM output
     * 
     * @details Restores normal PWM/DShot output mode on channels that were configured for
     *          serial passthrough. Must be called after serial_setup_output() to resume
     *          normal motor/servo control.
     * 
     * @param[in] chanmask Channels to restore (should match serial_setup_output() mask)
     * 
     * @note Re-enables normal output on affected channels
     * @note May require re-initialization time before outputs are stable
     * 
     * @see serial_setup_output() to begin passthrough
     */
    virtual void serial_end(uint32_t chanmask) {}
    
    /**
     * @brief Reset serial passthrough DMA configuration
     * 
     * @details Re-initializes DMA channels for serial output without ending serial mode.
     *          Used to recover from DMA errors during ESC communication.
     * 
     * @param[in] chanmask Channels to reset
     * 
     * @note Does not restore PWM output - call serial_end() for that
     */
    virtual void serial_reset(uint32_t chanmask) {}

    /**
     * @enum output_mode
     * @brief Output protocol types for motors, servos, and LEDs
     * 
     * @details Defines supported output protocols. Each has different timing characteristics,
     *          update rates, and capabilities.
     *          
     *          **Protocol Characteristics:**
     *          - **PWM**: Standard analog servo protocol, 50-500Hz, 1000-2000μs pulse
     *          - **OneShot**: High-speed analog ESC protocol, 1-4kHz, 125-250μs (OneShot125) or 42-84μs (OneShot42)
     *          - **DShot**: Digital ESC protocol, 150-1200kHz, immune to jitter, supports telemetry
     *          - **Brushed**: Duty-cycle control for brushed DC motors
     *          - **NeoPixel**: WS2812B LED protocol at 800kHz
     *          - **ProfiLED**: Separate clock/data LED control
     * 
     * @warning This enum is used by BLH_OTYPE and ESC_PWM_TYPE parameters - changes affect parameter compatibility!
     */
    enum output_mode {
        MODE_PWM_NONE,        ///< No output configured on channel
        MODE_PWM_NORMAL,      ///< Standard PWM servo protocol (50-500Hz, 1000-2000μs)
        MODE_PWM_ONESHOT,     ///< OneShot ESC protocol (high-speed PWM variant)
        MODE_PWM_ONESHOT125,  ///< OneShot125 ESC protocol (125-250μs pulses)
        MODE_PWM_BRUSHED,     ///< Brushed motor PWM (duty cycle control)
        MODE_PWM_DSHOT150,    ///< DShot digital protocol at 150kHz
        MODE_PWM_DSHOT300,    ///< DShot digital protocol at 300kHz
        MODE_PWM_DSHOT600,    ///< DShot digital protocol at 600kHz (most common)
        MODE_PWM_DSHOT1200,   ///< DShot digital protocol at 1200kHz
        MODE_NEOPIXEL,        ///< WS2812B LED protocol (GRB color order)
        MODE_PROFILED,        ///< ProfiLED protocol (separate clock and data lines)
        MODE_NEOPIXELRGB,     ///< WS2812B LED protocol (RGB color order)
    };
    
    /**
     * @brief Check if output mode is a DShot variant
     * 
     * @param[in] mode Output mode to check
     * @return true if mode is DShot150/300/600/1200, false otherwise
     * 
     * @note Static to allow use in ChibiOS thread context
     */
    static bool is_dshot_protocol(const enum output_mode mode);

    /**
     * @brief Check if output mode is an LED protocol
     * 
     * @param[in] mode Output mode to check
     * @return true if mode is NeoPixel, NeoPixelRGB, or ProfiLED, false otherwise
     */
    static bool is_led_protocol(const enum output_mode mode) {
      switch (mode) {
      case MODE_NEOPIXEL:
      case MODE_NEOPIXELRGB:
      case MODE_PROFILED:
        return true;
      default:
        return false;
      }
    }

    /**
     * @enum BLHeliDshotCommand
     * @brief DShot special commands for BLHeli/BLHeli32 ESCs
     * 
     * @details Commands sent via DShot protocol for ESC configuration and control.
     *          Commands use throttle values 0-47 (below normal throttle range).
     *          ESCs require multiple identical commands to accept (typically 10 repetitions).
     *          
     *          **Command Categories:**
     *          - Beeps: Motor identification and testing
     *          - Direction: Normal/reverse motor direction
     *          - 3D mode: Bi-directional (forward/reverse) support
     *          - Telemetry: Enable/disable extended ESC data
     *          - LEDs: BLHeli32 LED control (if ESC has LEDs)
     *          - Save: Persist settings to ESC EEPROM
     *          
     *          **References:**
     *          - BLHeli32: https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
     *          - BLHeli_S: https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt
     * 
     * @warning Commands can change ESC direction or behavior - verify before flight
     * @warning SAVE command writes to ESC EEPROM - excessive use can wear out EEPROM
     * 
     * @see send_dshot_command() to transmit commands
     */
    enum BLHeliDshotCommand : uint8_t {
      DSHOT_RESET = 0,                          ///< Reset ESC to default state
      DSHOT_BEEP1 = 1,                          ///< Beep tone 1 for motor identification
      DSHOT_BEEP2 = 2,                          ///< Beep tone 2 for motor identification
      DSHOT_BEEP3 = 3,                          ///< Beep tone 3 for motor identification
      DSHOT_BEEP4 = 4,                          ///< Beep tone 4 for motor identification
      DSHOT_BEEP5 = 5,                          ///< Beep tone 5 for motor identification
      DSHOT_ESC_INFO = 6,                       ///< Request ESC version info via telemetry
      DSHOT_ROTATE = 7,                         ///< Rotate motor direction identification
      DSHOT_ROTATE_ALTERNATE = 8,               ///< Alternate rotation pattern
      DSHOT_3D_OFF = 9,                         ///< Disable 3D/bidirectional mode
      DSHOT_3D_ON = 10,                         ///< Enable 3D/bidirectional mode
      DSHOT_SAVE = 12,                          ///< Save current settings to ESC EEPROM
      DSHOT_EXTENDED_TELEMETRY_ENABLE = 13,    ///< Enable extended telemetry (voltage, current, temp)
      DSHOT_EXTENDED_TELEMETRY_DISABLE = 14,   ///< Disable extended telemetry
      DSHOT_NORMAL = 20,                        ///< Set motor direction to normal
      DSHOT_REVERSE = 21,                       ///< Set motor direction to reversed
      // BLHeli32-specific commands for onboard LED control
      DSHOT_LED0_ON = 22,                       ///< Turn on ESC LED 0 (BLHeli32 only)
      DSHOT_LED1_ON = 23,                       ///< Turn on ESC LED 1 (BLHeli32 only)
      DSHOT_LED2_ON = 24,                       ///< Turn on ESC LED 2 (BLHeli32 only)
      DSHOT_LED3_ON = 25,                       ///< Turn on ESC LED 3 (BLHeli32 only)
      DSHOT_LED0_OFF = 26,                      ///< Turn off ESC LED 0 (BLHeli32 only)
      DSHOT_LED1_OFF = 27,                      ///< Turn off ESC LED 1 (BLHeli32 only)
      DSHOT_LED2_OFF = 28,                      ///< Turn off ESC LED 2 (BLHeli32 only)
      DSHOT_LED3_OFF = 29,                      ///< Turn off ESC LED 3 (BLHeli32 only)
    };

    /** 
     * @brief DShot zero throttle value
     * @details DShot throttle values 0-47 are reserved for commands. 
     *          Value 48 is the first valid throttle (motors off), 49-2047 provide increasing throttle.
     */
    const uint8_t DSHOT_ZERO_THROTTLE = 48;

    /**
     * @enum DshotEscType
     * @brief DShot-capable ESC firmware types
     * 
     * @details Identifies ESC firmware variant for protocol optimization and feature support.
     *          Different ESC firmware types may support different telemetry formats and timing.
     */
    enum DshotEscType {
      DSHOT_ESC_NONE = 0,       ///< No DShot ESC or unknown type
      DSHOT_ESC_BLHELI = 1,     ///< BLHeli ESC firmware
      DSHOT_ESC_BLHELI_S = 2,   ///< BLHeli_S ESC firmware (SiLabs MCU)
      DSHOT_ESC_BLHELI_EDT = 3, ///< BLHeli with EDT (Enhanced DShot Telemetry)
      DSHOT_ESC_BLHELI_EDT_S = 4 ///< BLHeli_S with EDT
    };

    /**
     * @brief Set output protocol mode for channels
     * 
     * @details Configures the output protocol (PWM, OneShot, DShot, etc.) for specified channels.
     *          Channels sharing hardware resources (timers, DMA) may have protocol restrictions.
     *          Changing mode reconfigures timers, DMA, and output pins for the new protocol.
     * 
     * @param[in] mask Bitmask of channels to configure
     * @param[in] mode Desired output_mode protocol
     * 
     * @note Channels on same timer group may be required to use compatible modes
     * @warning Do not change output mode while armed - may cause motor/servo glitches
     * 
     * @see output_mode enum for protocol descriptions
     * @see get_output_mode() to query current mode
     */
    virtual void    set_output_mode(uint32_t mask, enum output_mode mode) {}

    /**
     * @brief Query current output mode
     * 
     * @param[out] mask Bitmask of channels using the returned mode
     * @return Current output_mode for channels in mask
     * 
     * @note Implementation may return primary mode if multiple modes in use
     */
    virtual enum output_mode get_output_mode(uint32_t& mask) {
      mask = 0;
      return MODE_PWM_NORMAL;
    }


    /**
     * @brief Generate human-readable output mode description
     * 
     * @details Creates text banner describing output configuration for user display
     *          (e.g., "Ch1-4:DShot600 Ch5-8:PWM"). Used in boot messages and status displays.
     * 
     * @param[out] banner_msg Buffer to store banner string
     * @param[in]  banner_msg_len Buffer size in bytes
     * @return true if banner generated successfully, false if not supported
     * 
     * @note Buffer should be at least 50 bytes for typical configurations
     */
    virtual bool get_output_mode_banner(char banner_msg[], uint8_t banner_msg_len) const { return false; }

    /**
     * @brief Get channels disabled due to digital protocol conflicts
     * 
     * @details Returns channels that must be disabled because they share hardware timers with
     *          digital protocols (DShot, NeoPixel). Some timer groups cannot mix analog and
     *          digital protocols.
     * 
     * @param[in] digital_mask Bitmask of channels using digital protocols
     * @return Bitmask of channels that must be disabled
     * 
     * @note Hardware limitation - timer groups often cannot mix PWM and DShot
     */
    virtual uint32_t get_disabled_channels(uint32_t digital_mask) { return 0; }

    /**
     * @brief Set default PWM update rate
     * 
     * @details Configures baseline update rate for PWM outputs. Actual rate may be modified
     *          by per-channel frequency settings or protocol requirements.
     * 
     * @param[in] rate_hz Default update rate in Hz
     * 
     * @note Typically 50Hz for servos, 400Hz for copter motors
     */
    virtual void    set_default_rate(uint16_t rate_hz) {}

    /**
     * @brief Enable DShot telemetry request for channels
     * 
     * @details Configures DShot channels to request telemetry data from ESCs. ESCs send
     *          eRPM data on each update cycle. Not all ESCs support telemetry.
     * 
     * @param[in] mask Bitmask of channels to enable telemetry request
     * 
     * @note Requires DShot-capable ESCs with telemetry support
     * @see get_erpm() to read telemetry data
     */
    virtual void set_telem_request_mask(uint32_t mask) {}

    /**
     * @brief Enable bi-directional DShot telemetry for channels
     * 
     * @details Configures DShot channels for bi-directional communication with ESCs.
     *          Bi-directional DShot provides higher telemetry data rate than standard
     *          telemetry request mode. Requires compatible ESC firmware.
     *          
     *          Bi-directional benefits:
     *          - Higher update rate (every cycle vs every ~10 cycles)
     *          - More reliable telemetry
     *          - Lower latency RPM data for motor control
     * 
     * @param[in] mask Bitmask of channels to enable bi-directional telemetry
     * 
     * @note Requires BLHeli32 or compatible ESC firmware
     * @note May increase CPU load due to higher telemetry rate
     * 
     * @see get_erpm() to read telemetry data
     */
    virtual void set_bidir_dshot_mask(uint32_t mask) {}

    /**
     * @brief Mark ESCs as active for DShot command sending
     * 
     * @details Identifies which channels have active ESCs that should receive DShot commands.
     *          Commands are only sent to channels in this mask. Used to avoid sending commands
     *          to non-motor outputs or disconnected ESCs.
     * 
     * @param[in] mask Bitmask of channels with active ESCs
     * 
     * @see send_dshot_command() to transmit commands
     */
    virtual void set_active_escs_mask(uint32_t mask) {}

    /**
     * @brief Set DShot update rate multiplier
     * 
     * @details Configures DShot output rate as a multiple of main loop rate.
     *          Higher rates provide more responsive motor control but increase CPU load.
     *          
     *          Typical configurations:
     *          - dshot_rate=1: Update every loop (highest response, highest CPU)
     *          - dshot_rate=2: Update every 2nd loop (balanced)
     *          - dshot_rate=4: Update every 4th loop (lowest CPU)
     * 
     * @param[in] dshot_rate Rate divisor (1-4 typical)
     * @param[in] loop_rate_hz Main loop frequency in Hz
     * 
     * @note Effective DShot rate = loop_rate_hz / dshot_rate
     */
    virtual void set_dshot_rate(uint8_t dshot_rate, uint16_t loop_rate_hz) {}

    /**
     * @brief Set DShot output period (IOMCU use only)
     * 
     * @details Configures DShot bit timing period. Only used by IO coprocessor implementations.
     * 
     * @param[in] period_us Period in microseconds
     * @param[in] dshot_rate Rate multiplier
     * 
     * @note Internal use - platform-specific
     */
    virtual void set_dshot_period(uint32_t period_us, uint8_t dshot_rate) {}
    
    /**
     * @brief Get configured DShot period
     * 
     * @return DShot bit period in microseconds, or 0 if DShot not configured
     */
    virtual uint32_t get_dshot_period_us() const { return 0; }

    /**
     * @brief Configure DShot ESC firmware type
     * 
     * @details Sets ESC firmware type for protocol optimization. Different ESC firmware
     *          supports different telemetry formats and timing requirements.
     * 
     * @param[in] esc_type ESC firmware variant
     * 
     * @see DshotEscType enum for firmware types
     */
    virtual void set_dshot_esc_type(DshotEscType esc_type) {}

    /**
     * @brief Get configured DShot ESC type
     * 
     * @return Current ESC firmware type, or DSHOT_ESC_NONE if not configured
     */
    virtual DshotEscType get_dshot_esc_type() const { return DSHOT_ESC_NONE; }

    /** @brief Special channel number indicating "all channels" for DShot commands */
    const static uint32_t ALL_CHANNELS = 255;
    
    /**
     * @brief Send DShot command to ESC(s)
     * 
     * @details Transmits a DShot command (beep, direction change, LED control, etc.) to one or
     *          all ESCs. Commands are repeated multiple times to ensure ESC reception. ESCs
     *          require multiple identical commands before accepting (typically 6-10).
     *          
     *          Command transmission:
     *          - If command_timeout_ms=0: Send command repeat_count times immediately
     *          - If command_timeout_ms>0: Send command for specified duration
     *          
     *          Common uses:
     *          - Motor identification (beeps)
     *          - Direction reversal (NORMAL/REVERSE)
     *          - Telemetry enable (EXTENDED_TELEMETRY_ENABLE)
     *          - Save settings (SAVE)
     * 
     * @param[in] command BLHeliDshotCommand to send
     * @param[in] chan Channel number (0-31), or ALL_CHANNELS for broadcast
     * @param[in] command_timeout_ms Duration to send command (0=use repeat_count)
     * @param[in] repeat_count Number of times to send (default 10, typical 6-10 required)
     * @param[in] priority If true, interrupt normal output to send command immediately
     * 
     * @warning DSHOT_REVERSE changes motor direction - verify before flight!
     * @warning DSHOT_SAVE writes to ESC EEPROM - limit usage to prevent wear
     * 
     * @see BLHeliDshotCommand enum for available commands
     */
    virtual void send_dshot_command(uint8_t command, uint8_t chan = ALL_CHANNELS, uint32_t command_timeout_ms = 0, uint16_t repeat_count = 10, bool priority = false) {}

    /**
     * @brief Configure motor pole count for RPM calculations
     * 
     * @details Sets number of motor poles (magnets) for converting electrical RPM (eRPM)
     *          from ESC telemetry to mechanical RPM.
     *          
     *          Conversion: mechanical_RPM = eRPM / (pole_pairs)
     *                     mechanical_RPM = eRPM / (poles / 2)
     *          
     *          Common motor pole counts:
     *          - Small multirotors: 12-14 poles
     *          - Large multirotors: 14-24 poles
     *          - Typical: 14 poles = 7 pole pairs
     * 
     * @param[in] poles Number of motor poles (typically 12-24)
     * 
     * @note Most motors have even pole count
     * @see get_erpm() for reading ESC telemetry
     */
    virtual void set_motor_poles(uint8_t poles) {}

    /**
     * @brief Configure serial LED chain on output channel
     * 
     * @details Sets up addressable LED control (WS2812B NeoPixel or ProfiLED) on an output
     *          channel. Converts output from motor/servo to LED protocol. Each LED in chain
     *          can be individually controlled for RGB color.
     *          
     *          LED protocols:
     *          - MODE_NEOPIXEL: WS2812B at 800kHz, GRB color order
     *          - MODE_NEOPIXELRGB: WS2812B at 800kHz, RGB color order  
     *          - MODE_PROFILED: Separate clock/data lines
     * 
     * @param[in] chan Output channel number to use for LED control
     * @param[in] num_leds Number of LEDs in the chain (typically 4-64)
     * @param[in] mode LED protocol type (NEOPIXEL, NEOPIXELRGB, or PROFILED)
     * @param[in] clock_mask Clock channel mask (for PROFILED mode only)
     * @return true if LED output configured successfully, false if not supported
     * 
     * @note Maximum LED count depends on update rate and available DMA
     * @note LED output disables normal PWM on the channel
     * 
     * @see set_serial_led_rgb_data() to set LED colors
     * @see serial_led_send() to update LEDs
     */
    virtual bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) { return false; }

    /**
     * @brief Set RGB color for serial LED(s)
     * 
     * @details Configures color for one or all LEDs in a serial LED chain. Colors are buffered
     *          until serial_led_send() is called to update the physical LEDs.
     *          
     *          LED numbering: LED 0 is first in chain (closest to output pin)
     * 
     * @param[in] chan Output channel with LED chain
     * @param[in] led LED index in chain (0-based), or -1 for all LEDs
     * @param[in] red Red intensity (0-255)
     * @param[in] green Green intensity (0-255)
     * @param[in] blue Blue intensity (0-255)
     * @return true if color set successfully, false on error
     * 
     * @note Colors are buffered - call serial_led_send() to apply
     * @note Color order (RGB vs GRB) determined by mode set in set_serial_led_num_LEDs()
     * 
     * @see set_serial_led_num_LEDs() to configure LED chain
     * @see serial_led_send() to update LEDs
     */
    virtual bool set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) { return false; }

    /**
     * @brief Transmit buffered LED data to LED chain
     * 
     * @details Triggers DMA transfer to send buffered RGB data to LED chain. Updates all LEDs
     *          on the channel simultaneously with colors set by set_serial_led_rgb_data().
     * 
     * @param[in] chan Output channel with LED chain
     * @return true if LED data sent successfully, false on error
     * 
     * @note Transmission takes ~30μs per LED (e.g., 16 LEDs = 480μs)
     * @see set_serial_led_rgb_data() to set LED colors
     */
    virtual bool serial_led_send(const uint16_t chan) { return false; }

    /**
     * @brief Get timer and output configuration information
     * 
     * @details Generates diagnostic string describing timer configuration, channel assignments,
     *          DMA usage, and protocol settings. Used for debugging and status display.
     * 
     * @param[out] str ExpandingString to append timer information
     */
    virtual void timer_info(ExpandingString &str) {}

    /**
     * @brief Check if driver supports GPIO output mode
     * 
     * @details Some RCOutput implementations can also function as general-purpose digital outputs
     *          for relay control, LED switching, etc.
     * 
     * @return true if GPIO output supported, false if PWM/protocol only
     * 
     * @see write_gpio() to control GPIO state
     */
    virtual bool supports_gpio() { return false; };

    /**
     * @brief Write GPIO state to output channel
     * 
     * @details Sets channel to digital high/low for relay/LED control. Only supported if
     *          supports_gpio() returns true. Disables PWM on the channel.
     * 
     * @param[in] chan Channel number to use as GPIO
     * @param[in] active true for high/active, false for low/inactive
     * 
     * @note Disables PWM output on the channel
     * @see supports_gpio() to check if GPIO mode available
     */
    virtual void write_gpio(uint8_t chan, bool active) {};

    /**
     * @brief Enable multi-threaded output triggering
     * 
     * @details Allows output updates from any thread, not just main thread. Used in some
     *          high-performance applications but increases complexity and potential races.
     * 
     * @param[in] onoff true to enable multi-threaded triggers, false for main thread only
     * 
     * @warning Enabling may cause timing jitter if not carefully managed
     */
    virtual void force_trigger_groups(bool onoff) {};

    /**
     * @brief Calculate timer prescaler for target bit rate
     * 
     * @details Computes timer prescaler value to achieve desired output frequency for digital
     *          protocols (DShot, NeoPixel). Used during protocol initialization to configure
     *          hardware timers.
     *          
     *          Formula: prescaler = timer_clock / (target_frequency * period_ticks)
     * 
     * @param[in] timer_clock Timer peripheral clock frequency in Hz
     * @param[in] target_frequency Desired output bit frequency in Hz
     * @param[in] at_least_freq If true, round prescaler down (frequency at least target)
     * @return Calculated prescaler value
     * 
     * @note Static utility function for timer configuration
     */
    static uint32_t calculate_bitrate_prescaler(uint32_t timer_clock, uint32_t target_frequency, bool at_least_freq = false);

    /**
     * @name DShot Bit Timing Constants
     * @brief Timer tick counts for DShot digital protocol bit encoding
     * 
     * @details DShot uses pulse-width encoding: bit 0 = short pulse, bit 1 = long pulse.
     *          The duty cycle (high time / total time) must match ESC expectations.
     *          
     *          **Timing Options (period/bit0/bit1 ticks, duty cycles):**
     *          - 8/3/6 ticks: 37.5% / 75% duty - Preferred, widely compatible
     *          - 11/4/8 ticks: 36.4% / 72.7% duty - Alternative for BLHeli_S
     *          - 20/7/14 ticks: 35% / 70% duty - Less common
     *          
     *          ESCs are sensitive to duty cycle - incorrect timing causes sync loss.
     *          Default (8/3/6) has best compatibility across ESC firmware variants.
     * 
     * @{
     */
    
    /** @brief Default DShot bit period in timer ticks (8 ticks = 37.5%/75% duty) */
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_DEFAULT = 8;
    /** @brief Default DShot bit-0 high time in ticks (3 ticks = 37.5% duty) */
    static constexpr uint32_t DSHOT_BIT_0_TICKS_DEFAULT = 3;
    /** @brief Default DShot bit-1 high time in ticks (6 ticks = 75% duty) */
    static constexpr uint32_t DSHOT_BIT_1_TICKS_DEFAULT = 6;
    
    /** @brief BLHeli_S DShot bit period in timer ticks (11 ticks = 36.4%/72.7% duty) */
    static constexpr uint32_t DSHOT_BIT_WIDTH_TICKS_S = 11;
    /** @brief BLHeli_S DShot bit-0 high time in ticks (4 ticks = 36.4% duty) */
    static constexpr uint32_t DSHOT_BIT_0_TICKS_S = 4;
    /** @brief BLHeli_S DShot bit-1 high time in ticks (8 ticks = 72.7% duty) */
    static constexpr uint32_t DSHOT_BIT_1_TICKS_S = 8;

    /** @brief Active DShot bit period (set at runtime based on ESC type) */
    static uint32_t DSHOT_BIT_WIDTH_TICKS;
    /** @brief Active DShot bit-0 timing (set at runtime) */
    static uint32_t DSHOT_BIT_0_TICKS;
    /** @brief Active DShot bit-1 timing (set at runtime) */
    static uint32_t DSHOT_BIT_1_TICKS;
    /** @} */

    /**
     * @name NeoPixel/WS2812B Bit Timing Constants
     * @brief Timer tick counts for WS2812B LED protocol bit encoding
     * 
     * @details WS2812B uses 800kHz signal with specific pulse widths for 0 and 1 bits.
     *          Timing derived from WS2812B datasheet specifications.
     *          
     *          WS2812B timing requirements:
     *          - Bit period: 1.25μs (800kHz)
     *          - Bit 0: 0.4μs high, 0.85μs low (32% duty)
     *          - Bit 1: 0.8μs high, 0.45μs low (64% duty)
     * 
     * @{
     */
    
    /** @brief NeoPixel bit period in timer ticks (1.25μs at 800kHz) */
    static constexpr uint32_t NEOP_BIT_WIDTH_TICKS = 8;
    /** @brief NeoPixel bit-0 high time in ticks (~0.4μs) */
    static constexpr uint32_t NEOP_BIT_0_TICKS = 2;
    /** @brief NeoPixel bit-1 high time in ticks (~0.8μs) */
    static constexpr uint32_t NEOP_BIT_1_TICKS = 6;
    /** @} */
    
    /**
     * @name ProfiLED Bit Timing Constants
     * @brief Timer tick counts for ProfiLED protocol with separate clock/data
     * 
     * @details ProfiLED uses separate clock and data lines, not pulse-width modulation.
     *          Clock timing defines bit rate, data line carries actual bit values.
     * 
     * @{
     */
    
    /** @brief ProfiLED bit-0 clock timing in ticks */
    static constexpr uint32_t PROFI_BIT_0_TICKS = 7;
    /** @brief ProfiLED bit-1 clock timing in ticks */
    static constexpr uint32_t PROFI_BIT_1_TICKS = 14;
    /** @brief ProfiLED bit period in ticks */
    static constexpr uint32_t PROFI_BIT_WIDTH_TICKS = 20;
    /** @} */

    /**
     * @brief LED output update period
     * @details Minimum time between LED chain updates in microseconds. Long period (10ms)
     *          allows support for large LED chains without overloading DMA/CPU.
     *          At 30μs per LED, 10ms supports ~330 LEDs maximum.
     */
    static constexpr uint32_t LED_OUTPUT_PERIOD_US = 10000;

protected:

    /**
     * @brief Helper to append output mode range to banner string
     * 
     * @details Formats and appends channel range description to output mode banner.
     *          Example: "Ch1-4:DShot600 " appended to banner string.
     *          Used by derived classes implementing get_output_mode_banner().
     * 
     * @param[in,out] banner_msg Banner string buffer to append to
     * @param[in]     banner_msg_len Total buffer size
     * @param[in]     out_mode Output mode for this channel range
     * @param[in]     low_ch First channel in range (1-based for display)
     * @param[in]     high_ch Last channel in range (1-based for display)
     * 
     * @note Protected helper for derived class use
     */
    void append_to_banner(char banner_msg[], uint8_t banner_msg_len, output_mode out_mode, uint8_t low_ch, uint8_t high_ch) const;
    
    /**
     * @brief Get human-readable output mode name
     * 
     * @details Returns string representation of output_mode enum for display.
     *          Examples: "PWM", "DShot600", "OneShot125", "NeoPixel"
     * 
     * @param[in] out_mode Output mode to convert to string
     * @return Constant string with mode name, never null
     * 
     * @note Protected helper for derived class use
     */
    const char* get_output_mode_string(enum output_mode out_mode) const;

    /** @brief Minimum PWM value for ESC scaling (microseconds) */
    uint16_t _esc_pwm_min = 1000;
    /** @brief Maximum PWM value for ESC scaling (microseconds) */
    uint16_t _esc_pwm_max = 2000;
};
