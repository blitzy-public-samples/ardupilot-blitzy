/**
 * @file RCOutput.h
 * @brief QURT RC output implementation using framed UART protocol for ESC/servo control with telemetry
 * 
 * @details This file implements motor and servo output control for the Qualcomm QURT platform.
 *          The implementation uses a custom UART-based protocol with Modbus CRC framing to communicate
 *          with an external ESC controller board. The ESC controller board is a separate microcontroller
 *          that converts UART PWM commands to actual PWM/DShot signals sent to motors and servos.
 *          
 *          Protocol Overview:
 *          - Sends PWM commands (1000-2000us range) to ESC controller board via UART
 *          - All frames are CRC-protected using 16-bit Modbus CRC
 *          - Receives ESC telemetry data (RPM, voltage, current, temperature per channel)
 *          - Performs version discovery handshake on initialization to detect protocol version
 *          
 *          DSP Communication:
 *          - UART communication uses sl_client_uart_* RPC calls (adds latency ~50-500us)
 *          - Update rate typically 100-400Hz depending on scheduler configuration
 *          
 * @note This is protocol-specific to certain ESC controller firmware and not a universal ESC protocol
 * @warning Safety critical: Incorrect configuration can cause unexpected motor operation when disarmed
 */

#pragma once

#include <string>
#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QURT.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

/**
 * @class QURT::RCOutput
 * @brief Motor and servo output driver with ESC telemetry using framed UART protocol
 * 
 * @details This class implements the AP_HAL::RCOutput interface for the Qualcomm QURT platform
 *          and extends AP_ESC_Telem_Backend to provide ESC telemetry data to the vehicle code.
 *          
 *          Architecture:
 *          - Buffers PWM commands from vehicle code
 *          - Periodically sends buffered commands to ESC controller via UART
 *          - Polls and parses telemetry responses from ESC controller
 *          - Forwards telemetry to AP_ESC_Telem for logging and display
 *          
 *          ESC Controller Board:
 *          The ESC controller is a separate microcontroller that receives PWM commands
 *          via UART and generates actual PWM/DShot signals to drive motors and servos.
 *          It also monitors motor telemetry (RPM, voltage, current, temperature) and
 *          sends this data back via UART.
 *          
 *          Protocol Handshake:
 *          On initialization, the driver performs version discovery by sending a version
 *          request and waiting for the ESC controller to respond with its protocol version.
 *          This allows the driver to support both esc_response_v2 and older protocol formats.
 *          
 *          Thread Safety:
 *          All PWM writes are buffered with atomic cork/push operations to prevent partial
 *          updates. The cork() method begins a transaction, and push() commits all buffered
 *          changes atomically.
 *          
 *          Safety System:
 *          The safety mask controls which channels are allowed to output when the vehicle
 *          is disarmed. Safety-disabled channels output zero PWM when disarmed to prevent
 *          accidental motor spin.
 *          
 * @note Telemetry data includes: RPM, voltage, current, temperature per ESC channel
 * @note Power status: Total system voltage, current, accumulated energy consumption
 * @note All telemetry is logged in ESC telemetry messages for flight analysis
 * 
 * @warning Protocol-specific: Works with specific ESC controller firmware, not universal
 * @warning Safety critical: Safety mask must be correctly configured to prevent unexpected
 *          motor spin when vehicle is disarmed
 * @warning Motor arming: Vehicle must pass all arming checks before motors can spin;
 *          safety system prevents accidental operation
 * 
 * @see AP_ESC_Telem_Backend for telemetry backend interface
 * @see AP_HAL::RCOutput for base class interface
 */
class QURT::RCOutput : public AP_HAL::RCOutput, AP_ESC_Telem_Backend
{
public:
    friend class QURT::Util;

    /**
     * @brief Initialize RC output driver and discover ESC protocol version
     * 
     * @details Performs the following initialization sequence:
     *          1. Opens UART connection to ESC controller board
     *          2. Configures UART baud rate (921600 for IO board, 2000000 for ESC)
     *          3. Sends version request to detect ESC controller protocol version
     *          4. Waits for version response to determine if using esc_response_v2 or older format
     *          5. Initializes internal data structures and safety state
     * 
     * @note Sends version request and waits for response to detect protocol capabilities
     * @note Called once during system startup before motors can be used
     * @note UART communication via DSP RPC adds 50-500us latency
     * 
     * @see scan_for_hardware() for hardware detection
     */
    void init() override;
    
    /**
     * @brief Set PWM output frequency for specified channels
     * 
     * @param chmask Bitmask of channels to configure (bit 0 = channel 0, etc.)
     * @param freq_hz Desired PWM frequency in Hz
     * 
     * @details Sets the PWM update rate for the specified channels. Not all ESC controllers
     *          support arbitrary frequencies. Common values:
     *          - 50Hz: Standard servos
     *          - 400Hz: Fast ESCs
     *          - 490Hz: High-speed digital servos
     * 
     * @note Not all ESC controllers support arbitrary frequencies
     * @note Actual frequency may be limited by ESC controller capabilities
     * @warning High frequencies may cause servo heating or ESC instability
     */
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    
    /**
     * @brief Get configured PWM frequency for a channel
     * 
     * @param ch Channel number (0-based)
     * @return Configured PWM frequency in Hz
     * 
     * @note Returns configured frequency, not actual measured frequency
     */
    uint16_t get_freq(uint8_t ch) override;
    
    /**
     * @brief Enable output on specified channel
     * 
     * @param ch Channel number (0-based, max 7)
     * 
     * @details Enables PWM output on the specified channel. Channels are disabled
     *          by default for safety and must be explicitly enabled before use.
     * 
     * @note Channels disabled by default for safety
     * @note Channel must be enabled before write() commands take effect
     */
    void enable_ch(uint8_t ch) override;
    
    /**
     * @brief Disable output on specified channel
     * 
     * @param ch Channel number (0-based, max 7)
     * 
     * @details Disables PWM output on the specified channel, forcing output to zero.
     * 
     * @note Disabled channels output zero PWM regardless of write() commands
     */
    void disable_ch(uint8_t ch) override;
    
    /**
     * @brief Write PWM pulse width to single channel
     * 
     * @param ch Channel number (0-based, 0-7 for IO board, 0-3 for ESC board)
     * @param period_us Pulse width in microseconds
     * 
     * @details Buffers a PWM command for the specified channel. The command is not
     *          sent immediately but is buffered until the next periodic update or
     *          until push() is called. Typical PWM range is 1000-2000us, with 1500us
     *          as neutral for servos or zero throttle for ESCs.
     *          
     *          PWM Range Translation:
     *          - Standard ArduPilot range: 1000-2000us
     *          - Extended range: 800-2200us for some servos
     *          - Protocol translates to ESC controller-specific range
     * 
     * @note Buffered - actual output sent on next periodic update or push()
     * @note Typical range: 1000-2000us (1500us = neutral/zero)
     * @note Extended range: 800-2200us supported for some servos
     * @warning Incorrect range can cause motors to not arm or servos to hit mechanical limits
     * @warning Values outside safe range may damage servos or cause unsafe motor behavior
     * 
     * @see cork() and push() for atomic multi-channel updates
     */
    void write(uint8_t ch, uint16_t period_us) override;
    
    /**
     * @brief Read last commanded PWM value for single channel
     * 
     * @param ch Channel number (0-based)
     * @return Last commanded pulse width in microseconds
     * 
     * @details Returns the last PWM value written to the specified channel via write().
     *          This is the commanded value, not the actual motor position or speed.
     * 
     * @note Returns last commanded value, not actual motor position
     * @note Does not read feedback from motor or servo
     */
    uint16_t read(uint8_t ch) override;
    
    /**
     * @brief Read last commanded PWM values for multiple channels
     * 
     * @param[out] period_us Array to store pulse widths in microseconds
     * @param len Number of channels to read (array size)
     * 
     * @details Copies the last commanded PWM values for multiple channels into
     *          the provided array.
     * 
     * @note Returns last commanded values, not actual motor positions
     */
    void read(uint16_t *period_us, uint8_t len) override;
    
    /**
     * @brief Begin buffering PWM updates (start transaction)
     * 
     * @details Begins a transaction for atomic multi-channel PWM updates.
     *          While corked, write() commands are buffered but not sent to the
     *          ESC controller. Call push() to commit all buffered changes atomically.
     *          
     *          This prevents partial updates where some channels are updated before
     *          others, which could cause unwanted transient behavior.
     * 
     * @note Used for atomic multi-channel updates
     * @note Must call push() to commit buffered changes
     * @warning Prolonged cork without push() will delay motor commands
     * 
     * @see push() to commit buffered updates
     */
    void cork(void) override;
    
    /**
     * @brief Flush buffered PWM updates (commit transaction)
     * 
     * @details Commits all PWM changes buffered since the last cork() call.
     *          All buffered channel updates are sent to the ESC controller
     *          in a single atomic operation.
     * 
     * @note Flushes all buffered updates to ESC controller
     * @note Updates sent in single atomic frame to prevent partial updates
     * 
     * @see cork() to begin buffering updates
     */
    void push(void) override;

    /**
     * @brief Get total ESC power supply voltage
     * 
     * @return Voltage in volts from ESC power status telemetry
     * 
     * @details Returns the most recent total input voltage reading from the
     *          ESC power status telemetry. This is the battery or power supply
     *          voltage measured by the ESC controller board.
     * 
     * @note Updated periodically from ESC telemetry (typically 100-400Hz)
     * @note Value is from esc_power_status telemetry structure
     */
    float get_voltage(void) const
    {
        return esc_voltage;
    }
    
    /**
     * @brief Get total ESC power supply current
     * 
     * @return Current in amperes from ESC power status telemetry
     * 
     * @details Returns the most recent total current draw reading from the
     *          ESC power status telemetry. This is the combined current of
     *          all ESC channels measured by the ESC controller board.
     * 
     * @note Updated periodically from ESC telemetry (typically 100-400Hz)
     * @note Value is from esc_power_status telemetry structure
     */
    float get_current(void) const
    {
        return esc_current;
    }

    /**
     * @brief Force safety switch on, disabling output from ESCs/servos
     * 
     * @return true if safety successfully enabled
     * 
     * @details Forces the safety system into the "on" (safe) state, which disables
     *          all motor and servo outputs. This is used for emergency shutdowns or
     *          when ground operations require motors to be disabled.
     *          
     *          When safety is on:
     *          - All PWM outputs forced to zero
     *          - Motors cannot spin even if armed
     *          - Servos do not respond to commands
     * 
     * @note Provides emergency shutdown capability
     * @note Can be overridden by force_safety_off() if needed
     * @warning Does not prevent vehicle from arming, only disables outputs
     */
    bool force_safety_on(void) override
    {
        safety_on = true;
        return true;
    }

    /**
     * @brief Force safety switch off, enabling output from ESCs/servos
     * 
     * @details Forces the safety system into the "off" (armed) state, which allows
     *          motor and servo outputs when the vehicle is armed. This overrides
     *          the default safety-on state.
     *          
     *          When safety is off:
     *          - PWM outputs enabled (if vehicle armed)
     *          - Motors can spin if armed and commanded
     *          - Servos respond to commands
     * 
     * @warning Allows motor spin when disarmed if safety checks are bypassed - use with extreme caution!
     * @warning Ensure vehicle is in safe state before calling (no people/objects near propellers)
     * @warning Should only be called after deliberate user action (safety button press)
     * 
     * @note Typically disabled automatically by AP_BoardConfig during normal operation
     * @see set_safety_mask() for configuring which channels respect safety
     */
    void force_safety_off(void) override
    {
        safety_on = false;
    }

private:
    /**
     * @brief UART baud rates for different hardware types
     * 
     * @note IO board uses 921600 baud (slower, more reliable over longer cables)
     * @note ESC board uses 2000000 baud (faster, optimized for high-frequency updates)
     */
    enum {
        IO_BAUDRATE = 921600,   ///< Baud rate for IO board (921.6 kbps)
        ESC_BAUDRATE = 2000000  ///< Baud rate for ESC board (2 Mbps)
    } baudrate;

    /**
     * @brief Hardware type detection for protocol selection
     * 
     * @note Detected during initialization via version handshake
     */
    enum class HWType {
        UNKNOWN = 0,            ///< Unknown board type (not yet detected)
        ESC = 1,                ///< ESC controller board (4 motor channels)
        IO = 2,                 ///< IO board (8 servo/motor channels)
    };
    HWType hw_type = HWType::UNKNOWN;  ///< Detected hardware type

    /**
     * @brief Scan for attached hardware and detect type
     * 
     * @details Probes UART ports to detect ESC controller board or IO board.
     *          Determines hardware type based on version response.
     */
    void scan_for_hardware(void);
    
    /**
     * @brief Send commands and receive responses from ESC controller
     * 
     * @details Main communication function that sends buffered PWM commands
     *          and polls for telemetry responses from the ESC controller.
     */
    void send_receive(void);
    
    /**
     * @brief Check and parse responses from ESC controller
     * 
     * @details Validates CRC and parses received telemetry packets including
     *          esc_response_v2 and esc_power_status structures.
     */
    void check_response(void);
    
    /**
     * @brief Send PWM output values to ESC controller
     * 
     * @details Formats and sends current PWM values to ESC controller with
     *          CRC protection. Applies safety mask before transmission.
     */
    void send_pwm_output(void);
    
    /**
     * @brief Send command to IO board
     * 
     * @details Formats and sends IO-specific commands (8 channels).
     */
    void send_io_command(void);
    
    /**
     * @brief Send command to ESC board
     * 
     * @details Formats and sends ESC-specific commands (4 motor channels).
     */
    void send_esc_command(void);
    
    /**
     * @brief Send framed packet with CRC to ESC controller
     * 
     * @param type Packet type identifier
     * @param data Pointer to packet payload data
     * @param size Size of payload in bytes
     * 
     * @details Wraps payload in protocol frame with:
     *          - Header with packet type
     *          - Payload data
     *          - 16-bit Modbus CRC checksum
     * 
     * @note All packets are CRC-protected to detect transmission errors
     */
    void send_packet(uint8_t type, uint8_t *data, uint16_t size);

    /**
     * @struct extended_version_info
     * @brief ESC controller firmware version and identification information
     * 
     * @details Received during initialization handshake to identify ESC controller
     *          hardware and firmware versions. Used to determine protocol capabilities
     *          and select appropriate communication format.
     * 
     * @note Sent by ESC controller in response to version request during init()
     * @note CRC field protects against corruption during transmission
     */
    struct PACKED extended_version_info {
        uint8_t  id;                           ///< ESC controller board ID
        uint16_t sw_version;                   ///< Software (firmware) version
        uint16_t hw_version;                   ///< Hardware revision
        uint8_t  unique_id[12];                ///< Unique device identifier (e.g., MCU serial number)
        char     firmware_git_version[12];     ///< Git commit hash of firmware
        char     bootloader_git_version[12];   ///< Git commit hash of bootloader
        uint16_t bootloader_version;           ///< Bootloader version number
        uint16_t crc;                          ///< 16-bit CRC for corruption detection
    };

    /**
     * @struct esc_response_v2
     * @brief ESC telemetry data structure with motor state and measurements
     * 
     * @details Contains real-time telemetry from a single ESC channel including motor speed,
     *          applied power, electrical measurements, and temperature. This is version 2 of
     *          the ESC response protocol, detected via version handshake during initialization.
     *          
     *          Telemetry Update Rate:
     *          - Typically 100-400Hz depending on scheduler configuration
     *          - Each packet contains data for one ESC channel
     *          - Channels polled sequentially in round-robin fashion
     *          
     *          Data Usage:
     *          - RPM: Used for motor failure detection and logging
     *          - Voltage/Current: Power consumption analysis and battery monitoring
     *          - Temperature: Thermal protection and warning generation
     *          - Power: Verify commanded throttle vs actual output
     * 
     * @note CRC protection: Structure received with CRC to detect corruption
     * @note Forwarded to AP_ESC_Telem backend for logging and display
     * @note Logged in ESC telemetry messages for post-flight analysis
     * 
     * @see handle_esc_feedback() for telemetry parsing
     * @see AP_ESC_Telem for telemetry backend interface
     */
    struct PACKED esc_response_v2 {
        uint8_t  id_state;     ///< bits 0:3 = ESC state, bits 4:7 = ESC ID/channel number

        uint16_t rpm;          ///< Current motor RPM (revolutions per minute)
        uint8_t  cmd_counter;  ///< Command sequence counter (increments with each command received)
        uint8_t  power;        ///< Applied motor power [0..100%]

        uint16_t voltage;      ///< ESC input voltage in millivolts (mV)
        int16_t  current;      ///< ESC current draw in 8mA resolution (multiply by 8 for actual mA)
        int16_t  temperature;  ///< ESC temperature in 0.01°C resolution (divide by 100 for actual °C)
    };

    /**
     * @struct esc_power_status
     * @brief Power supply telemetry with total voltage and current
     * 
     * @details Contains overall power system telemetry measuring total input voltage
     *          and combined current draw across all ESC channels. Used for battery
     *          monitoring, power budget management, and failsafe triggering.
     *          
     *          Update Rate:
     *          - Typically updated at lower rate than per-channel telemetry
     *          - Usually 10-50Hz depending on ESC controller configuration
     *          
     *          Data Usage:
     *          - Total voltage: Battery voltage monitoring, low voltage warnings
     *          - Total current: Power consumption tracking, current limit enforcement
     *          - Accumulated energy: Flight time estimation, battery capacity tracking
     * 
     * @note CRC protection: Structure includes 16-bit CRC to detect corruption
     * @note Used by battery monitoring system for voltage/current readings
     * @note Logged for post-flight power analysis
     * 
     * @see handle_power_status() for telemetry parsing
     */
    struct PACKED esc_power_status {
        uint8_t  id;       ///< ESC controller ID (could be used as system ID)
        uint16_t voltage;  ///< Total input voltage in millivolts (mV)
        int16_t  current;  ///< Total current draw in 8mA resolution (multiply by 8 for actual mA)
    };

    /**
     * @brief Handle version feedback from ESC controller
     * 
     * @param pkt Extended version information packet from ESC controller
     * 
     * @details Parses version information received during initialization to determine
     *          ESC controller capabilities and protocol version. Used to select between
     *          esc_response_v2 and older telemetry formats.
     * 
     * @note Called during initialization handshake
     * @see extended_version_info for packet structure
     */
    void handle_version_feedback(const struct extended_version_info &pkt);
    
    /**
     * @brief Handle ESC telemetry feedback
     * 
     * @param pkt ESC response v2 telemetry packet
     * 
     * @details Parses per-channel ESC telemetry (RPM, voltage, current, temperature)
     *          and forwards data to AP_ESC_Telem backend for logging and display.
     *          Updates internal telemetry state and triggers warnings if needed.
     * 
     * @note Called periodically when telemetry packets received (typically 100-400Hz per channel)
     * @note Forwards telemetry to AP_ESC_Telem for system-wide access
     * @see esc_response_v2 for packet structure
     */
    void handle_esc_feedback(const struct esc_response_v2 &pkt);
    
    /**
     * @brief Handle power status telemetry
     * 
     * @param pkt Power status packet with total voltage and current
     * 
     * @details Parses overall power system telemetry and updates internal voltage/current
     *          state. Data is used by battery monitoring system for failsafe decisions.
     * 
     * @note Called when power status packets received (typically 10-50Hz)
     * @note Updates esc_voltage and esc_current member variables
     * @see esc_power_status for packet structure
     */
    void handle_power_status(const struct esc_power_status &pkt);

    /**
     * @brief Convert board ID to human-readable name
     * 
     * @param board_id Numeric board identifier
     * @return String name of the board (e.g., "ESC_V1", "IO_V2")
     * 
     * @note Used for logging and diagnostics
     */
    std::string board_id_to_name(uint16_t board_id);

    int fd = -1;                                     ///< UART file descriptor (-1 if not opened)
    uint16_t enable_mask;                            ///< Bitmask of enabled channels
    static const uint8_t max_channel_count = 8;      ///< Maximum number of channels (IO board)
    static const uint8_t esc_channel_count = 4;      ///< Number of motor channels (ESC board)
    static const uint8_t io_channel_count = 8;       ///< Number of channels (IO board)
    uint16_t period[max_channel_count];              ///< Last commanded PWM values (us)
    uint16_t pwm_output[max_channel_count];          ///< Buffered PWM output values (us)
    volatile bool need_write;                        ///< Flag indicating buffered data needs transmission
    bool corked;                                     ///< Cork state for atomic multi-channel updates
    HAL_Semaphore mutex;                             ///< Mutex for thread-safe access to buffers
    uint8_t last_fb_idx;                             ///< Last feedback index for round-robin polling
    uint32_t last_fb_req_ms;                         ///< Timestamp of last feedback request (milliseconds)

    float esc_voltage;                               ///< Most recent total voltage from power status (volts)
    float esc_current;                               ///< Most recent total current from power status (amperes)

    /**
     * @brief Safety switch state
     * 
     * @details Controls whether outputs are enabled or disabled for safety.
     *          When true, all outputs forced to zero regardless of commanded values.
     *          Starts true (safe) and gets disabled by AP_BoardConfig during normal operation.
     * 
     * @note Start with safety on, gets disabled by AP_BoardConfig during initialization
     * @warning Safety-on prevents motor spin even when vehicle is armed
     */
    bool safety_on = true;
};
