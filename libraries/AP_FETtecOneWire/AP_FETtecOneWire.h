/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Initial protocol implementation was provided by FETtec */
/* Strongly modified by Amilcar Lucas, IAV GmbH */

/**
 * @file AP_FETtecOneWire.h
 * @brief FETtec OneWire ESC driver for bidirectional motor control and telemetry
 * 
 * @details This driver implements the FETtec OneWire protocol for electronic speed controller
 *          (ESC) communication. The protocol supports bidirectional full-duplex serial
 *          communication at 500Kbps (default) or optional half-duplex at 2Mbps.
 *          
 *          The OneWire protocol provides:
 *          - Motor throttle commands (11-bit precision packed into minimal bytes)
 *          - ESC telemetry (temperature, voltage, current, RPM, consumption, error counts)
 *          - ESC configuration and discovery
 *          - Optional beep commands for audible feedback
 *          - Optional RGB LED control
 *          
 *          Integration with ArduPilot:
 *          - Integrates with AP_HAL UART drivers for platform-independent serial communication
 *          - Uses SRV_Channels for motor output values
 *          - Registers with AP_SerialManager for serial port allocation
 *          - Provides telemetry to AP_ESC_Telem for logging and monitoring
 *          
 *          Feature flags control optional functionality:
 *          - HAL_AP_FETTEC_HALF_DUPLEX: Enable 2Mbps half-duplex mode (default: disabled)
 *          - HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO: Query ESC type/version/serial number (default: disabled)
 *          - HAL_AP_FETTEC_ESC_BEEP: Support audible beep commands (default: disabled)
 *          - HAL_AP_FETTEC_ESC_LIGHT: Support RGB LED control (default: disabled)
 *          - HAL_WITH_ESC_TELEM: Enable telemetry reception and processing (ArduPilot global flag)
 *          
 *          Protocol capabilities:
 *          - Supports up to 24 ESCs without telemetry
 *          - Supports up to 15 ESCs with telemetry enabled
 *          - Automatic ESC discovery and bootloader detection
 *          - Frame CRC using CRC-8 DVB-S2 algorithm (same as DShot)
 *          
 * @warning ESCs require continuous updates at minimum 4Hz (maximum 250ms between commands)
 *          or they will automatically disarm for safety
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_FETTEC_ONEWIRE_ENABLED
#define AP_FETTEC_ONEWIRE_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 1024
#endif

// Support both full-duplex at 500Kbit/s as well as half-duplex at 2Mbit/s (optional feature)
#ifndef HAL_AP_FETTEC_HALF_DUPLEX
#define HAL_AP_FETTEC_HALF_DUPLEX 0
#endif

// Get static info from the ESCs (optional feature)
#ifndef HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
#define HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO 0
#endif

// provide beep support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_BEEP
#define HAL_AP_FETTEC_ESC_BEEP 0
#endif

// provide light support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_LIGHT
#define HAL_AP_FETTEC_ESC_LIGHT 0
#endif

#if AP_FETTEC_ONEWIRE_ENABLED

#define FTW_DEBUGGING 0
#if FTW_DEBUGGING
#include <stdio.h>
#define fet_debug(fmt, args ...)  do {::fprintf(stderr,"FETtec: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define fet_debug(fmt, args ...)
#endif

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Param/AP_Param.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

/**
 * @class AP_FETtecOneWire
 * @brief FETtec OneWire ESC singleton driver for motor control and telemetry
 * 
 * @details This class implements a complete driver for FETtec OneWire ESCs, managing
 *          the entire lifecycle from ESC discovery through runtime operation.
 *          
 *          Protocol State Machine:
 *          The driver operates through several phases for each ESC:
 *          
 *          1. ESC Discovery and Bootloader Detection:
 *             - Send OK message to detect if ESC is in bootloader or firmware mode
 *             - If bootloader detected, send START_FW command to exit bootloader
 *          
 *          2. Firmware Startup Sequence:
 *             - Wait for ESC firmware to start and respond
 *             - ESC transitions from bootloader to normal firmware operation
 *          
 *          3. Configuration Phase:
 *             - (Optional) Query static info: ESC type, firmware version, serial number
 *             - Configure telemetry mode if HAL_WITH_ESC_TELEM enabled
 *             - Configure fast-throttle frame format with motor count and byte packing
 *          
 *          4. Runtime Operation:
 *             - Send fast-throttle commands containing all motor values
 *             - Receive and process telemetry responses from ESCs
 *             - Monitor ESC health and telemetry freshness
 *          
 *          Singleton Pattern:
 *          This driver uses a singleton pattern as only one OneWire ESC bus is supported
 *          per vehicle. Access via get_singleton().
 *          
 *          Thread Safety:
 *          This driver is called from SRV_Channels::push() at the main loop rate:
 *          - Copter: 400Hz typical
 *          - Plane/Rover: 50Hz typical
 *          No additional locking is required as all access is from main thread.
 *          
 *          Coordinate System:
 *          Throttle values use ArduPilot servo convention:
 *          - Range: 0-2000 (11-bit values)
 *          - 1001-2000: Positive (forward) rotation
 *          - 999-0: Reverse rotation
 *          - 1000: Neutral/stop
 *          
 * @warning Driver requires continuous updates at minimum 4Hz to prevent ESC auto-disarm.
 *          Maximum interval between fast-throttle commands is 250ms. If this timing is
 *          violated, ESCs will automatically disarm for safety and require re-initialization.
 * 
 * @note Inherits from AP_ESC_Telem_Backend to provide telemetry data to the logging
 *       and monitoring subsystems.
 */
class AP_FETtecOneWire : public AP_ESC_Telem_Backend
{

public:
    /**
     * @brief Constructs FETtec OneWire driver singleton and registers parameters
     * 
     * @details Initializes the singleton instance and registers parameter group with
     *          AP_Param system. Parameters include motor mask, reverse mask, and pole count.
     */
    AP_FETtecOneWire();

    /// Do not allow copies
    CLASS_NO_COPY(AP_FETtecOneWire);

    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Returns singleton instance of the FETtec OneWire driver
     * 
     * @return Pointer to singleton instance, or nullptr if not initialized
     */
    static AP_FETtecOneWire *get_singleton() {
        return _singleton;
    }

    /**
     * @brief Periodic update function called from SRV_Channels::push()
     * 
     * @details Manages complete ESC communication cycle:
     *          - Initializes UART on first call
     *          - Reads and processes incoming telemetry from ESCs
     *          - Runs ESC configuration state machines for unconfigured ESCs
     *          - Transmits fast-throttle commands with current motor values
     *          
     *          This function must be called regularly to maintain ESC communication
     *          and prevent automatic disarm.
     * 
     * @note Called at main loop rate (400Hz for Copter, 50Hz for other vehicles)
     * 
     * @warning Must be called at minimum 4Hz or ESCs will auto-disarm for safety.
     *          Maximum allowed interval is 250ms between calls.
     */
    void update();

    /**
     * @brief Pre-arm safety check to verify ESC readiness
     * 
     * @details Validates complete ESC system before allowing vehicle arming:
     *          - UART is available and configured correctly
     *          - SERVO_FWT_MASK parameter is valid (at least one ESC configured)
     *          - SERVO_FWT_POLES parameter is valid (used for RPM calculation)
     *          - All configured ESCs have been detected and are running
     *          - Telemetry is being received if HAL_WITH_ESC_TELEM enabled
     *          - No ESCs stuck in bootloader mode
     *          
     *          If any check fails, a descriptive error message is written to failure_msg
     *          and the function returns false to block arming.
     * 
     * @param[out] failure_msg Buffer to store failure message string if check fails
     * @param[in]  failure_msg_len Maximum length of failure message buffer (bytes)
     * 
     * @return true if all checks pass and arming is allowed, false if arming should be blocked
     * 
     * @warning Arming will be blocked if any ESC is not responding or telemetry is stale.
     *          This prevents attempting flight with non-functional motor control.
     * 
     * @note Called by AP_Arming during pre-arm check sequence
     */
    bool pre_arm_check(char *failure_msg, const uint8_t failure_msg_len) const;

#if HAL_AP_FETTEC_ESC_BEEP
    /**
     * @brief Commands all connected ESCs to emit audible beep
     * 
     * @details Sends BEEP message to all ESCs for user feedback or vehicle location
     *          identification. The beep frequency parameter controls the pitch of the
     *          tone produced by the ESC's motor commutation.
     *          
     *          Common uses:
     *          - Pre-flight checks to verify ESC presence
     *          - Lost vehicle location after crash
     *          - User feedback for configuration changes
     * 
     * @param[in] beep_frequency Beep tone frequency, 8-bit value (0-255).
     *                           Higher values produce higher pitch tones.
     * 
     * @note Only available when HAL_AP_FETTEC_ESC_BEEP is enabled at compile time
     * 
     * @warning Should not be called during flight as may distract pilot with unexpected
     *          audible tones. Intended for ground use only.
     */
    void beep(const uint8_t beep_frequency);
#endif

#if HAL_AP_FETTEC_ESC_LIGHT
    /**
     * @brief Sets RGB LED color on all connected ESCs
     * 
     * @details Sends SET_LED_TMP_COLOR message to configure ESC LED indicators.
     *          This provides visual feedback for vehicle status, orientation, or
     *          user-defined indications. The color setting is temporary and may
     *          revert to default on ESC reset or power cycle.
     *          
     *          Common uses:
     *          - Visual indication of arming state
     *          - Flight mode indication
     *          - Battery status warning
     *          - User-defined visual effects
     * 
     * @param[in] r Red brightness (0-255), 0 is off, 255 is maximum brightness
     * @param[in] g Green brightness (0-255), 0 is off, 255 is maximum brightness
     * @param[in] b Blue brightness (0-255), 0 is off, 255 is maximum brightness
     * 
     * @note Only available when HAL_AP_FETTEC_ESC_LIGHT is enabled at compile time
     * 
     * @note Temporary color setting. ESC may revert to default color on reset.
     *       For persistent color, ESC configuration must be changed via FETtec software.
     */
    void led_color(const uint8_t r, const uint8_t g, const uint8_t b);
#endif

private:
    static AP_FETtecOneWire *_singleton;
    AP_HAL::UARTDriver *_uart;

    AP_Int32 _motor_mask_parameter;
    AP_Int32 _reverse_mask_parameter;
#if HAL_WITH_ESC_TELEM
    AP_Int8 _pole_count_parameter;
#endif

    static constexpr uint8_t FRAME_OVERHEAD = 6;          ///< OneWire message frame overhead (header+tail bytes)
    static constexpr uint8_t MAX_RECEIVE_LENGTH = 12;     ///< OneWire max receive message payload length in bytes
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    static constexpr uint8_t SERIAL_NUMBER_LENGTH = 12;   ///< ESC serial number length in bytes
#endif

    /**
     * @brief Initialize driver, configure UART, allocate ESC array
     * 
     * @details Performs complete driver initialization sequence:
     *          - Validate motor mask parameter
     *          - Initialize UART via init_uart()
     *          - Allocate ESC array based on motor mask
     *          - Initialize per-ESC data structures
     *          - Calculate fast-throttle frame parameters
     *          
     *          Called automatically on first update() call.
     * 
     * @note Only called once, protected by _init_done flag
     */
    void init();

    /**
     * @brief Acquire and configure UART from AP_SerialManager
     * 
     * @details Searches for UART configured with SerialProtocol_ESCTelemetry
     *          and configures it for OneWire communication:
     *          - Baud rate: 500000 (full-duplex) or 2000000 (half-duplex)
     *          - Flow control: disabled
     *          - Parity: none
     *          - Timeout: configured for fast response
     * 
     * @note Sets _uart to nullptr if no suitable port found
     */
    void init_uart();

    /**
     * @brief Run configuration state machines for all ESCs
     * 
     * @details Scans OneWire bus and progresses each ESC through its configuration
     *          state machine as specified in the _motor_mask_parameter. Handles:
     *          - Bootloader detection and firmware startup
     *          - Static info queries (if enabled)
     *          - Telemetry configuration (if enabled)
     *          - Fast-throttle frame configuration
     *          
     *          ESCs must reach RUNNING state before receiving throttle commands.
     * 
     * @note Called from update() until all ESCs reach RUNNING state
     * @note Only sends configuration when vehicle is disarmed for safety
     */
    void configure_escs();

    /**
     * @enum ESCState
     * @brief State machine states for individual ESC configuration and operation
     * 
     * @details Each ESC progresses through a configuration sequence before entering
     *          normal runtime operation. The state machine manages this progression:
     *          
     *          Complete State Transition Flow:
     *          ===============================
     *          UNINITIALISED (5)
     *            ↓
     *          WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE (10) - Send OK to detect bootloader/firmware
     *            ↓
     *          WAITING_OK_FOR_RUNNING_SW_TYPE (11) - Wait for response
     *            ↓
     *          [If bootloader detected]
     *            ↓
     *          WANT_SEND_START_FW (20) - Command ESC to exit bootloader
     *            ↓
     *          WAITING_OK_FOR_START_FW (21) - Wait for firmware startup
     *            ↓
     *          [Optional: If HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO enabled]
     *            ↓
     *          WANT_SEND_REQ_TYPE (30) - Request ESC type
     *            ↓
     *          WAITING_ESC_TYPE (31) - Wait for type response
     *            ↓
     *          WANT_SEND_REQ_SW_VER (40) - Request firmware version
     *            ↓
     *          WAITING_SW_VER (41) - Wait for version response
     *            ↓
     *          WANT_SEND_REQ_SN (50) - Request serial number
     *            ↓
     *          WAITING_SN (51) - Wait for serial number response
     *            ↓
     *          [If HAL_WITH_ESC_TELEM enabled]
     *            ↓
     *          WANT_SEND_SET_TLM_TYPE (60) - Configure telemetry mode
     *            ↓
     *          WAITING_SET_TLM_TYPE_OK (61) - Wait for telemetry config acknowledgment
     *            ↓
     *          WANT_SEND_SET_FAST_COM_LENGTH (70) - Configure fast-throttle frame format
     *            ↓
     *          WAITING_SET_FAST_COM_LENGTH_OK (71) - Wait for frame config acknowledgment
     *            ↓
     *          RUNNING (100) - Normal operation: send throttle, receive telemetry
     *          
     *          State numbering uses gaps (5, 10, 20, etc.) to allow for future intermediate
     *          states without renumbering.
     * 
     * @note Only ESCs in RUNNING state receive throttle commands. ESCs in configuration
     *       states are still being initialized.
     * 
     * @warning If an ESC fails to respond during configuration, it remains in a WAITING_*
     *          state and will not progress to RUNNING. Pre-arm checks detect this condition.
     */
    enum class ESCState : uint8_t {
        UNINITIALISED = 5,  ///< Initial state when ESC first detected, no communication attempted

        WANT_SEND_OK_TO_GET_RUNNING_SW_TYPE = 10,  ///< Ready to send OK message to detect bootloader/firmware mode
        WAITING_OK_FOR_RUNNING_SW_TYPE = 11,       ///< Waiting for response to determine if ESC is in bootloader

        WANT_SEND_START_FW = 20,        ///< Ready to send START_FW command to exit bootloader
        WAITING_OK_FOR_START_FW = 21,   ///< Waiting for ESC to exit bootloader and start firmware

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        WANT_SEND_REQ_TYPE = 30,   ///< Ready to request ESC type information
        WAITING_ESC_TYPE = 31,     ///< Waiting for ESC type response

        WANT_SEND_REQ_SW_VER = 40,  ///< Ready to request firmware version
        WAITING_SW_VER = 41,        ///< Waiting for firmware version response

        WANT_SEND_REQ_SN = 50,  ///< Ready to request ESC serial number
        WAITING_SN = 51,        ///< Waiting for serial number response
#endif

#if HAL_WITH_ESC_TELEM
        WANT_SEND_SET_TLM_TYPE = 60,      ///< Ready to configure telemetry mode
        WAITING_SET_TLM_TYPE_OK = 61,     ///< Waiting for telemetry configuration acknowledgment
#endif

        WANT_SEND_SET_FAST_COM_LENGTH = 70,     ///< Ready to configure fast-throttle frame format
        WAITING_SET_FAST_COM_LENGTH_OK = 71,    ///< Waiting for fast-throttle frame configuration acknowledgment

        RUNNING = 100,  ///< Normal operation: ESC configured and ready for throttle commands
    };

    /**
     * @class ESC
     * @brief Per-ESC runtime state and telemetry tracking
     * 
     * @details Maintains state machine position, telemetry data, and static information
     *          for each individual physical ESC. An array of these objects is allocated
     *          based on the motor mask parameter.
     *          
     *          Each ESC instance tracks:
     *          - Configuration state machine position
     *          - Telemetry reception status and freshness
     *          - Static identification (ID, type, version, serial number)
     *          - Mapping to ArduPilot servo output channels
     *          
     *          The ESC ID is set in the ESC's firmware (typically 1-24) and uniquely
     *          identifies each ESC on the OneWire bus. The servo_ofs maps this ESC
     *          to a position in ArduPilot's servo output array.
     * 
     * @note This is an inner class of AP_FETtecOneWire, allocated dynamically based
     *       on the number of configured ESCs.
     */
    class ESC {
    public:

#if HAL_WITH_ESC_TELEM
        uint32_t last_telem_us;              ///< Timestamp (microseconds) of last telemetry reception from this ESC
        uint16_t unexpected_telem;           ///< Counter for telemetry received when not expected (protocol error)
        uint16_t error_count_at_throttle_count_overflow;  ///< CRC error counter snapshot at throttle counter overflow for delta calculation
        bool telem_expected;                 ///< This ESC is fully configured and should be sending telemetry
        bool telem_requested;                ///< This ESC has been requested to send telemetry at least once
#endif

        uint8_t id;         ///< FETtec ESC ID (1-based, configured in ESC firmware, typically 1-24)
        uint8_t servo_ofs;  ///< Offset into ArduPilot servo output array (0-based, maps ESC to motor output)
        bool is_awake;      ///< ESC has responded to communications (not stuck in bootloader or powered off)
        
        /**
         * @brief Updates ESC state machine position with debug logging
         * 
         * @param[in] _state New state to transition to
         * 
         * @note Logs state transitions when FTW_DEBUGGING is enabled for troubleshooting
         */
        void set_state(ESCState _state) {
            fet_debug("Moving ESC.id=%u from state=%u to state=%u", (unsigned)id, (unsigned)state, (unsigned)_state);
            state = _state;
        };
        ESCState state = ESCState::UNINITIALISED;  ///< Current position in configuration/operation state machine

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        uint8_t serial_number[SERIAL_NUMBER_LENGTH];  ///< ESC serial number (12 bytes, factory programmed)
        uint8_t firmware_version;                     ///< ESC firmware major version number
        uint8_t firmware_subversion;                  ///< ESC firmware minor version number
        uint8_t type;                                 ///< ESC hardware type identifier
#endif
    };

    uint32_t _min_fast_throttle_period_us;  ///< minimum allowed fast-throttle command transmit period
    uint32_t _motor_mask;                    ///< an un-mutable copy of the _motor_mask_parameter taken before _init_done goes true
    uint32_t _reverse_mask;                  ///< a copy of the _reverse_mask_parameter taken while not armed
    uint32_t _running_mask;                  ///< a bitmask of the actively running ESCs
    uint32_t _last_transmit_us;             ///< last time the transmit() function sent data
    ESC *_escs;
    uint8_t _esc_count;                ///< number of allocated ESCs
    uint8_t _fast_throttle_byte_count; ///< pre-calculated number of bytes required to send an entire packed throttle message

#if HAL_AP_FETTEC_HALF_DUPLEX
    uint8_t _ignore_own_bytes; ///< bytes to ignore while receiving, because we have transmitted them ourselves
    uint8_t _last_crc;         ///< the CRC from the last sent fast-throttle command
    bool _use_hdplex;          ///< use asynchronous half-duplex serial communication
#endif

    bool _init_done;     ///< device driver is initialized; ESCs may still need to be configured
    bool _invalid_mask;  ///< true if the mask parameter is invalid

    enum class FrameSource : uint8_t {
        MASTER     = 0x01,  ///< master is always 0x01
        BOOTLOADER = 0x02,
        ESC        = 0x03,
    };

    enum class MsgType : uint8_t
    {
        OK                  = 0,
        BL_PAGE_CORRECT     = 1,  ///< Bootloader only
        NOT_OK              = 2,
        BL_START_FW         = 3,  ///< Bootloader only - exit the boot loader and start the standard firmware
        BL_PAGES_TO_FLASH   = 4,  ///< Bootloader only
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        REQ_TYPE            = 5,  ///< ESC type
        REQ_SN              = 6,  ///< serial number
        REQ_SW_VER          = 7,  ///< software version
#endif
#if HAL_AP_FETTEC_ESC_BEEP
        BEEP                = 13, ///< make noise
#endif
        SET_FAST_COM_LENGTH = 26, ///< configure fast-throttle command
        SET_TLM_TYPE        = 27, ///< telemetry operation mode
#if HAL_AP_FETTEC_ESC_LIGHT
        SET_LED_TMP_COLOR   = 51, ///< set ESC's LED color
#endif
    };

    /**
     * @class PackedMessage
     * @brief Generic OneWire configuration message frame wrapper
     * 
     * @details Encapsulates any message payload type T into a complete OneWire protocol
     *          frame with header, addressing, and checksum.
     *          
     *          OneWire Frame Structure:
     *          =======================
     *          Byte 1: frame_source  - Message source identifier (master=0x01)
     *          Byte 2: esc_id        - Target ESC ID (5-bit, values 1-31)
     *          Byte 3-4: frame_type  - Frame type (0x0000 for compatibility with bootloader)
     *          Byte 5: frame_length  - Total frame length including all bytes
     *          Byte 6-X: msg         - Message payload of type T
     *          Byte X+1: checksum    - CRC-8 DVB-S2 checksum over all preceding bytes
     *          
     *          The frame format is compatible with FETtec bootloader protocol, which uses
     *          the frame_type field for bootloader-specific commands. In normal operation,
     *          this field is always 0x0000.
     * 
     * @tparam T Payload message type (OK, START_FW, SET_FAST_COM_LENGTH, etc.)
     * 
     * @note Frame uses PACKED attribute to prevent compiler padding and ensure exact
     *       byte layout for serial transmission.
     * 
     * @note Checksum uses CRC-8 DVB-S2 algorithm (polynomial 0xD5), same as DShot protocol.
     *       This provides robust error detection for serial communication.
     * 
     * @warning Frame must be transmitted atomically. Partial transmissions will cause
     *          CRC errors and message rejection by ESCs.
     * 
     * Source: libraries/AP_FETtecOneWire/AP_FETtecOneWire.h:249-268
     */
    template <typename T>
    class PACKED PackedMessage {
    public:
        /**
         * @brief Constructs a complete OneWire frame with payload
         * 
         * @param[in] _esc_id Target ESC ID (1-31)
         * @param[in] _msg Message payload of type T
         * 
         * @note Automatically calculates and sets checksum on construction
         */
        PackedMessage(uint8_t _esc_id, T _msg) :
            esc_id(_esc_id),
            msg(_msg)
        {
            update_checksum();
        }
        uint8_t frame_source { (uint8_t)FrameSource::MASTER };      ///< Message source (always MASTER=0x01 from autopilot)
        uint8_t esc_id;                                             ///< Target ESC ID (1-31)
        uint16_t frame_type { 0 };                                  ///< Frame type (0x0000, for bootloader compatibility)
        uint8_t frame_length {sizeof(T) + FRAME_OVERHEAD};          ///< Total frame length in bytes (payload + overhead)
        T msg;                                                      ///< Message payload
        uint8_t checksum;                                           ///< CRC-8 DVB-S2 checksum

        /**
         * @brief Recalculates CRC-8 checksum for the frame
         * 
         * @details Computes CRC-8 DVB-S2 over all frame bytes except checksum itself.
         *          Must be called after modifying any frame fields.
         */
        void update_checksum() {
            checksum = crc8_dvb_update(0, (const uint8_t*)this, frame_length-1);
        }
    };

    /**
     * @class OK
     * @brief Acknowledgment message payload
     * 
     * @details Generic acknowledgment message used in multiple contexts:
     *          - Response to detect ESC firmware vs bootloader mode
     *          - Acknowledgment of configuration commands
     *          
     *          This is the most common response message from ESCs.
     */
    class PACKED OK {
    public:
        uint8_t msgid { (uint8_t)MsgType::OK };  ///< Message ID: OK (0)
    };

    /**
     * @class START_FW
     * @brief Bootloader exit command payload
     * 
     * @details Commands an ESC in bootloader mode to exit the bootloader and start
     *          the main firmware. This is sent when bootloader mode is detected during
     *          ESC discovery phase.
     *          
     *          After sending this command, the autopilot must wait for the ESC to
     *          restart and begin responding as normal firmware.
     * 
     * @note Only used when ESC is detected in bootloader mode during startup
     */
    class PACKED START_FW {
    public:
        uint8_t msgid { (uint8_t)MsgType::BL_START_FW };  ///< Message ID: BL_START_FW (3)
    };

    /**
     * @class SET_FAST_COM_LENGTH
     * @brief Configure fast-throttle frame format message payload
     * 
     * @details Configures the ESC to expect fast-throttle commands with specific
     *          byte packing format. This optimization packs 11-bit throttle values
     *          into minimal bytes rather than using 16-bit values.
     *          
     *          Fast-throttle format calculation:
     *          - Each throttle value is 11 bits (0-2047)
     *          - byte_count = ceil(esc_count * 11 / 8) + 1 (for CRC)
     *          - Example: 4 ESCs = ceil(44/8) + 1 = 6 + 1 = 7 bytes total
     *          
     *          The min_esc_id and esc_count define the range of ESCs that will
     *          receive throttle commands in each fast-throttle frame.
     * 
     * @note This must be configured before entering RUNNING state
     * @note All ESCs must be configured with the same fast-throttle format
     */
    class PACKED SET_FAST_COM_LENGTH {
    public:
        /**
         * @brief Constructs fast-throttle configuration message
         * 
         * @param[in] _byte_count Total bytes in fast-throttle frame (including CRC)
         * @param[in] _min_esc_id Lowest ESC ID in the configured range
         * @param[in] _esc_count Number of consecutive ESCs from min_esc_id
         */
        SET_FAST_COM_LENGTH(uint8_t _byte_count, uint8_t _min_esc_id, uint8_t _esc_count) :
            byte_count{_byte_count},
            min_esc_id{_min_esc_id},
            esc_count{_esc_count}
        { }
        uint8_t msgid { (uint8_t)MsgType::SET_FAST_COM_LENGTH };  ///< Message ID: SET_FAST_COM_LENGTH (26)
        uint8_t byte_count;   ///< Total fast-throttle frame length in bytes
        uint8_t min_esc_id;   ///< Minimum ESC ID in range (typically 1)
        uint8_t esc_count;    ///< Number of ESCs receiving throttle commands
    };

#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    /**
     * @class REQ_TYPE
     * @brief Request ESC type query message payload
     * 
     * @details Queries the ESC for its hardware type identifier. Response is ESC_TYPE message.
     * 
     * @note Only available when HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO is enabled
     */
    class PACKED REQ_TYPE {
    public:
        uint8_t msgid { (uint8_t)MsgType::REQ_TYPE };  ///< Message ID: REQ_TYPE (5)
    };

    /**
     * @class REQ_SW_VER
     * @brief Request firmware version query message payload
     * 
     * @details Queries the ESC for its firmware version. Response is SW_VER message
     *          containing major and minor version numbers.
     * 
     * @note Only available when HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO is enabled
     */
    class PACKED REQ_SW_VER {
    public:
        uint8_t msgid { (uint8_t)MsgType::REQ_SW_VER };  ///< Message ID: REQ_SW_VER (7)
    };

    /**
     * @class REQ_SN
     * @brief Request serial number query message payload
     * 
     * @details Queries the ESC for its factory-programmed serial number. Response is
     *          SN message containing 12-byte serial number.
     * 
     * @note Only available when HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO is enabled
     */
    class PACKED REQ_SN {
    public:
        uint8_t msgid { (uint8_t)MsgType::REQ_SN };  ///< Message ID: REQ_SN (6)
    };

    /**
     * @class ESC_TYPE
     * @brief ESC type response message payload
     * 
     * @details Response to REQ_TYPE query, contains hardware type identifier.
     *          Type identifier is ESC model-specific.
     * 
     * @note Only available when HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO is enabled
     */
    class PACKED ESC_TYPE {
    public:
        /**
         * @brief Constructs ESC type response
         * @param[in] _type ESC hardware type identifier
         */
        ESC_TYPE(uint8_t _type) :
            type{_type} { }
        uint8_t type;  ///< ESC hardware type identifier (model-specific)
    };

    /**
     * @class SW_VER
     * @brief Firmware version response message payload
     * 
     * @details Response to REQ_SW_VER query, contains firmware version numbers.
     *          Version format: version.subversion (e.g., 2.3)
     * 
     * @note Only available when HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO is enabled
     */
    class PACKED SW_VER {
    public:
        /**
         * @brief Constructs firmware version response
         * @param[in] _version Major version number
         * @param[in] _subversion Minor version number
         */
        SW_VER(uint8_t _version, uint8_t _subversion) :
            version{_version},
            subversion{_subversion}
            { }
        uint8_t version;      ///< Firmware major version number
        uint8_t subversion;   ///< Firmware minor version number
    };

    /**
     * @class SN
     * @brief Serial number response message payload
     * 
     * @details Response to REQ_SN query, contains 12-byte factory-programmed serial number.
     *          Serial number uniquely identifies this specific ESC unit.
     * 
     * @note Only available when HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO is enabled
     */
    class PACKED SN {
    public:
        /**
         * @brief Constructs serial number response
         * @param[in] _sn Pointer to serial number bytes
         * @param[in] snlen Length of serial number (should be SERIAL_NUMBER_LENGTH)
         */
        SN(uint8_t *_sn, uint8_t snlen) {
            memcpy(sn, _sn, ARRAY_SIZE(sn));
        }
        uint8_t sn[SERIAL_NUMBER_LENGTH];  ///< Serial number bytes (12 bytes, factory programmed)
    };

#endif  // HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO

    /**
     * @brief Pack 11-bit throttle values into minimal byte array
     * 
     * @details Packs throttle values for all motors into a compact binary format
     *          where each throttle value uses exactly 11 bits instead of 16 bits.
     *          This reduces transmission time and bus utilization.
     *          
     *          Packing format:
     *          - Each motor: 11 bits (values 0-2047)
     *          - Multiple values packed across byte boundaries
     *          - Example 4 motors: 44 bits = 6 bytes (vs 8 bytes for 16-bit values)
     *          - Final byte: CRC-8 checksum
     *          
     *          The esc_id_to_request_telem_from parameter enables round-robin telemetry
     *          requests embedded in the throttle frame.
     * 
     * @param[in] motor_values Array of 16-bit throttle values (0-2000)
     * @param[out] buffer Output buffer for packed data (must be at least length bytes)
     * @param[in] length Size of output buffer in bytes
     * @param[in] esc_id_to_request_telem_from ESC ID to request telemetry from (0=none)
     * 
     * @note Buffer must be large enough: ceil(motor_count * 11 / 8) + 1 bytes
     */
    void pack_fast_throttle_command(const uint16_t *motor_values, uint8_t *buffer, const uint8_t length, const uint8_t esc_id_to_request_telem_from);

/*
 * Messages, methods and states for dealing with ESC telemetry
 */
#if HAL_WITH_ESC_TELEM
    /**
     * @brief Process received telemetry packet and forward to AP_ESC_Telem
     * 
     * @details Decodes TLM message from ESC and forwards telemetry data to
     *          AP_ESC_Telem backend for logging and monitoring:
     *          - Converts scaled integers to floating-point values
     *          - Calculates propeller RPM from electrical RPM using pole count
     *          - Updates telemetry freshness timestamp
     *          - Tracks CRC error deltas
     * 
     * @param[in,out] esc ESC object receiving telemetry data
     * 
     * @note Updates esc.last_telem_us timestamp for staleness detection
     */
    void handle_message_telem(ESC &esc);

    uint16_t _fast_throttle_cmd_count;     ///< number of fast-throttle commands sent by the flight controller

    /// the ESC at this offset into _escs should be the next to send a
    /// telemetry request for:
    uint8_t _esc_ofs_to_request_telem_from;

    /**
     * @class SET_TLM_TYPE
     * @brief Configure telemetry mode message payload
     * 
     * @details Configures the ESC's telemetry operation mode. The telemetry type
     *          determines when and how frequently the ESC sends telemetry data back
     *          to the autopilot.
     *          
     *          Common telemetry modes:
     *          - 0: Telemetry disabled
     *          - 1: Send telemetry when requested in fast-throttle frame
     *          
     *          Telemetry is requested from one ESC per fast-throttle cycle using a
     *          round-robin scheme to minimize bus congestion.
     * 
     * @note Only available when HAL_WITH_ESC_TELEM is enabled
     */
    class PACKED SET_TLM_TYPE {
    public:
        /**
         * @brief Constructs telemetry mode configuration message
         * @param[in] _tlm_type Telemetry operation mode (typically 1 for request-based)
         */
        SET_TLM_TYPE(uint8_t _tlm_type) :
            tlm_type{_tlm_type}
        { }
        uint8_t msgid { (uint8_t)MsgType::SET_TLM_TYPE };  ///< Message ID: SET_TLM_TYPE (27)
        uint8_t tlm_type;  ///< Telemetry mode: 0=disabled, 1=send when requested
    };

    /**
     * @class TLM
     * @brief ESC telemetry data packet
     * 
     * @details Complete telemetry data sent from ESC to autopilot, containing all critical
     *          ESC operating parameters for monitoring, logging, and failsafe detection.
     *          
     *          Telemetry Data:
     *          ===============
     *          - Temperature: ESC internal temperature monitoring for thermal protection
     *          - Voltage: Input voltage monitoring for battery failsafe detection
     *          - Current: Current draw for power consumption tracking
     *          - RPM: Motor electrical RPM for performance monitoring
     *          - Consumption: Cumulative energy consumption for battery capacity tracking
     *          - Error Count: CRC errors detected by ESC for communication health monitoring
     *          
     *          All values are scaled integers to avoid floating-point operations in the ESC:
     *          - Temperature: centi-degrees Celsius (°C * 100), e.g., 4523 = 45.23°C
     *          - Voltage: centi-Volts (V * 100), e.g., 1670 = 16.70V
     *          - Current: centi-Amperes (A * 100), e.g., 2350 = 23.50A
     *          - RPM: centi-RPM (eRPM * 100), e.g., 123456 = 1234.56 eRPM
     *          - Consumption: milli-Ampere-hours, e.g., 1500 = 1.5Ah
     *          - Error count: raw count of CRC errors detected by ESC
     *          
     *          RPM Calculation:
     *          ================
     *          The RPM field contains electrical RPM (eRPM), which must be divided by the
     *          motor pole count to get actual propeller RPM:
     *          
     *          Propeller RPM = eRPM / pole_count
     *          
     *          Example: eRPM=12000, 14-pole motor (7 pole pairs):
     *          Propeller RPM = 12000 / 14 = 857 RPM
     *          
     *          The pole count is configured via SERVO_FWT_POLES parameter.
     * 
     * @note Only available when HAL_WITH_ESC_TELEM is enabled
     * 
     * @note All telemetry values use fixed-point integer representation to avoid
     *       floating-point operations on the ESC microcontroller.
     * 
     * @warning Electrical RPM (eRPM) must be divided by motor pole count (SERVO_FWT_POLES)
     *          to obtain actual propeller RPM. Using eRPM directly will give incorrect
     *          RPM readings that are off by the pole count factor.
     * 
     * Source: libraries/AP_FETtecOneWire/AP_FETtecOneWire.h:359-375
     */
    class PACKED TLM {
    public:
        /**
         * @brief Constructs telemetry data packet
         * 
         * @param[in] _temp ESC temperature in centi-degrees Celsius (°C * 100)
         * @param[in] _voltage Input voltage in centi-Volts (V * 100)
         * @param[in] _current Current draw in centi-Amperes (A * 100)
         * @param[in] _rpm Electrical RPM in centi-RPM (eRPM * 100), divide by pole count for propeller RPM
         * @param[in] _consumption_mah Cumulative consumption in milli-Ampere-hours (mAh)
         * @param[in] _tx_err_count CRC error counter from ESC's perspective (errors in receiving autopilot transmissions)
         */
        TLM(int8_t _temp, uint16_t _voltage, uint16_t _current, int16_t _rpm, uint16_t _consumption_mah, uint16_t _tx_err_count) :
            temp{_temp},
            voltage{_voltage},
            current{_current},
            rpm{_rpm},
            consumption_mah{_consumption_mah},
            tx_err_count{_tx_err_count}
        { }
        int8_t temp;              ///< Temperature in centi-degrees Celsius (°C * 100), e.g., 4523 = 45.23°C
        uint16_t voltage;         ///< Input voltage in centi-Volts (V * 100), e.g., 1670 = 16.70V
        uint16_t current;         ///< Current draw in centi-Amperes (A * 100), e.g., 2350 = 23.50A
        int16_t rpm;              ///< Electrical RPM in centi-RPM (eRPM * 100), divide by SERVO_FWT_POLES for propeller RPM
        uint16_t consumption_mah; ///< Cumulative consumption in milli-Ampere-hours (mAh)
        uint16_t tx_err_count;    ///< CRC error count detected by ESC when receiving autopilot transmissions
    };

#endif  // HAL_WITH_ESC_TELEM

#if HAL_AP_FETTEC_ESC_BEEP
    /**
     * @class Beep
     * @brief Audible beep command message payload
     * 
     * @details Commands ESC to produce an audible beep by driving the motor windings
     *          at the specified frequency. Used for pre-flight checks, lost vehicle
     *          location, and user feedback.
     *          
     *          The spacer field provides additional bytes after the command to ensure
     *          all ESCs on the bus can receive and process the message, as no response
     *          acknowledgment is expected.
     * 
     * @note Only available when HAL_AP_FETTEC_ESC_BEEP is enabled
     * @note Broadcast message, sent to all ESCs simultaneously
     */
    class PACKED Beep {
    public:
        /**
         * @brief Constructs beep command message
         * @param[in] _beep_frequency Beep tone frequency (0-255), higher values produce higher pitch
         */
        Beep(uint8_t _beep_frequency) :
            beep_frequency{_beep_frequency}
        { }
        uint8_t msgid { (uint8_t)MsgType::BEEP };  ///< Message ID: BEEP (13)
        uint8_t beep_frequency;  ///< Tone frequency (0-255), higher = higher pitch
        uint16_t spacer = 0;     ///< Padding to ensure message reception (no response expected)
    };
#endif // HAL_AP_FETTEC_ESC_BEEP

#if HAL_AP_FETTEC_ESC_LIGHT
    /**
     * @class LEDColour
     * @brief RGB LED color command message payload
     * 
     * @details Commands ESC to set RGB LED color for visual feedback. Used for status
     *          indication, orientation marking, and user-defined visual effects.
     *          
     *          The color setting is temporary and may revert to default on ESC reset.
     *          For persistent color, ESC configuration must be changed via FETtec software.
     *          
     *          The spacer field provides additional bytes after the command to ensure
     *          all ESCs on the bus can receive and process the message, as no response
     *          acknowledgment is expected.
     * 
     * @note Only available when HAL_AP_FETTEC_ESC_LIGHT is enabled
     * @note Broadcast message, sent to all ESCs simultaneously
     * @note Temporary setting, may revert on ESC power cycle or reset
     */
    class PACKED LEDColour {
    public:
        /**
         * @brief Constructs LED color command message
         * @param[in] _r Red brightness (0-255), 0=off, 255=maximum
         * @param[in] _g Green brightness (0-255), 0=off, 255=maximum
         * @param[in] _b Blue brightness (0-255), 0=off, 255=maximum
         */
        LEDColour(uint8_t _r, uint8_t _g, uint8_t _b) :
            r{_r},
            g{_g},
            b{_b}
          { }
        uint8_t msgid { (uint8_t)MsgType::SET_LED_TMP_COLOR };  ///< Message ID: SET_LED_TMP_COLOR (51)
        uint8_t r;  ///< Red brightness (0-255)
        uint8_t g;  ///< Green brightness (0-255)
        uint8_t b;  ///< Blue brightness (0-255)
        uint16_t spacer = 0;  ///< Padding to ensure message reception (no response expected)
    };
#endif  // HAL_AP_FETTEC_ESC_LIGHT

    /*
     * Methods and data for transmitting data to the ESCSs:
     */

    /**
     * @brief Low-level UART transmission with buffer space checking
     * 
     * @details Transmits data to ESCs via UART with validation:
     *          - Checks UART buffer space availability
     *          - Writes data atomically
     *          - Updates transmission timestamp for timing enforcement
     *          - Handles half-duplex byte echo tracking if enabled
     * 
     * @param[in] bytes Pointer to data to transmit
     * @param[in] length Number of bytes to transmit
     * 
     * @return true if transmission successful, false if insufficient UART buffer space
     * 
     * @note Updates _last_transmit_us timestamp for minimum period enforcement
     */
    bool transmit(const uint8_t* bytes, const uint8_t length);

    /**
     * @brief Transmit PackedMessage wrapper (template specialization)
     * 
     * @tparam T Message payload type
     * @param[in] msg PackedMessage to transmit
     * @return true if transmission successful, false if insufficient buffer space
     */
    template <typename T>
    bool transmit(const PackedMessage<T> &msg) {
        return transmit((const uint8_t*)&msg, sizeof(msg));
    }

    /**
     * @brief Transmit configuration message with armed-state check
     * 
     * @details Transmits configuration request to ESCs with safety check:
     *          - Blocks transmission if vehicle is armed (safety)
     *          - Checks UART buffer space availability
     *          - Delegates to transmit() for actual transmission
     *          
     *          Configuration changes are only allowed when disarmed to prevent
     *          unexpected behavior during flight.
     * 
     * @param[in] bytes Pointer to configuration data to transmit
     * @param[in] length Number of bytes to transmit
     * 
     * @return false if vehicle armed or insufficient UART buffer space, true if sent
     * 
     * @warning Always returns false if vehicle is armed, regardless of buffer space
     */
    bool transmit_config_request(const uint8_t* bytes, const uint8_t length);

    /**
     * @brief Transmit configuration PackedMessage with armed-state check (template specialization)
     * 
     * @tparam T Message payload type
     * @param[in] msg PackedMessage containing configuration request
     * @return false if vehicle armed or insufficient buffer space, true if sent
     */
    template <typename T>
    bool transmit_config_request(const PackedMessage<T> &msg) {
        return transmit_config_request((const uint8_t*)&msg, sizeof(msg));
    }

    /**
     * @brief Send fast-throttle frame with all motor values
     * 
     * @details Transmits complete fast-throttle command containing throttle values
     *          for all configured ESCs in a single packed frame:
     *          - Packs 11-bit throttle values via pack_fast_throttle_command()
     *          - Includes round-robin telemetry request
     *          - Enforces minimum transmit period
     *          - Updates running ESC mask
     * 
     * @param[in] motor_values Array of throttle values (0-2000) where:
     *                         - 1001-2000: Positive (forward) rotation
     *                         - 1000: Neutral/stop
     *                         - 0-999: Reverse rotation
     * 
     * @note Array size must match _esc_count
     * @warning Must be called at minimum 4Hz to prevent ESC auto-disarm
     */
    void escs_set_values(const uint16_t *motor_values);

    /*
     * Methods and data for receiving data from the ESCs:
     */

    // FIXME: this should be tighter - and probably calculated.  Note
    // that we can't request telemetry faster than the loop interval,
    // which is 20ms on Plane, so that puts a constraint here.  When
    // using fast-throttle with 12 ESCs on Plane you could expect
    // 240ms between telem updates.  Why you have a Plane with 12 ESCs
    // is a bit of a puzzle.
    static const uint32_t max_telem_interval_us = 100000;

    /**
     * @brief Process received configuration response message
     * 
     * @details Handles configuration response messages from ESCs during state machine
     *          progression. Validates message type matches expected state and advances
     *          ESC through configuration sequence:
     *          - OK acknowledgments for configuration commands
     *          - ESC type, version, and serial number responses
     *          - Telemetry configuration acknowledgments
     *          - Fast-throttle configuration acknowledgments
     * 
     * @param[in,out] esc ESC object that sent the message
     * @param[in] length Message payload length in bytes
     * 
     * @note Updates ESC state machine on successful message handling
     * @note Increments error counters for unexpected messages
     */
    void handle_message(ESC &esc, const uint8_t length);

    /**
     * @brief Read incoming UART data and process complete frames
     * 
     * @details Reads available UART data and processes complete OneWire frames:
     *          - Reads bytes into receive buffer
     *          - Searches for frame header (frame_source byte)
     *          - Validates frame length and CRC
     *          - Dispatches complete frames to handle_message() or handle_message_telem()
     *          - Manages receive buffer (consumes processed bytes)
     *          
     *          Incomplete frames remain in buffer for next call.
     * 
     * @note Called from update() on every iteration
     * @note Handles partial frames across multiple calls
     */
    void read_data_from_uart();
    union MessageUnion {
        MessageUnion() { }
        PackedMessage<OK> packed_ok;
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
        PackedMessage<ESC_TYPE> packed_esc_type;
        PackedMessage<SW_VER> packed_sw_ver;
        PackedMessage<SN> packed_sn;
#endif
#if HAL_WITH_ESC_TELEM
        PackedMessage<TLM> packed_tlm;
#endif
        uint8_t receive_buf[FRAME_OVERHEAD + MAX_RECEIVE_LENGTH];
    } u;

    static_assert(sizeof(u.packed_ok) <= sizeof(u.receive_buf),"packed_ok does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
#if HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
    static_assert(sizeof(u.packed_esc_type) <= sizeof(u.receive_buf),"packed_esc_type does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
    static_assert(sizeof(u.packed_sw_ver) <= sizeof(u.receive_buf),"packed_sw_ver does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
    static_assert(sizeof(u.packed_sn) <= sizeof(u.receive_buf),"packed_sn does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
#endif
#if HAL_WITH_ESC_TELEM
    static_assert(sizeof(u.packed_tlm) <= sizeof(u.receive_buf),"packed_tlm does not fit in receive_buf. MAX_RECEIVE_LENGTH too small?");
#endif

    uint16_t _unknown_esc_message;
    uint16_t _message_invalid_in_state_count;
    uint16_t _period_too_short;
    uint16_t crc_rec_err_cnt;
    uint8_t _receive_buf_used;

    /**
     * @brief Align receive buffer to frame header
     * 
     * @details Searches receive buffer for valid frame_source magic byte (0x01, 0x02, or 0x03)
     *          and shifts data to start of buffer. Used to resynchronize after:
     *          - CRC errors
     *          - Partial frame reception
     *          - UART noise or corruption
     *          
     *          Discards any bytes before the frame header.
     * 
     * @param[in] search_start_pos Offset in buffer to start searching (default: 0)
     * 
     * @note Modifies _receive_buf_used to reflect new buffer content size
     */
    void move_frame_source_in_receive_buffer(const uint8_t search_start_pos = 0);

    /**
     * @brief Remove processed bytes from start of receive buffer
     * 
     * @details Removes n bytes from the beginning of the receive buffer and shifts
     *          remaining data forward. Used after successfully processing a complete frame.
     * 
     * @param[in] n Number of bytes to remove from buffer start
     * 
     * @note Updates _receive_buf_used to reflect reduced buffer content
     * @warning n must be <= _receive_buf_used
     */
    void consume_bytes(const uint8_t n);

    /**
     * @brief Check if receive buffer contains OK acknowledgment
     * 
     * @details Fast check for OK message at start of receive buffer:
     *          - Validates frame length matches OK message size
     *          - Checks msgid field equals MsgType::OK
     *          
     *          Used during configuration state machine to quickly detect acknowledgments
     *          without full frame parsing.
     * 
     * @param[in] length Expected frame length to validate
     * 
     * @return true if buffer starts with valid OK message, false otherwise
     * 
     * @note Does not validate CRC - caller must validate before acting on result
     */
    bool buffer_contains_ok(const uint8_t length);
};
#endif // AP_FETTEC_ONEWIRE_ENABLED
