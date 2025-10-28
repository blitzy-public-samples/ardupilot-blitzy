/**
 * @file RCOutput.h
 * @brief RC servo and motor output simulation for Software In The Loop (SITL)
 * 
 * @details This file implements the RC output interface for SITL simulation,
 *          providing simulated PWM output channels for controlling servos, motors,
 *          and other actuators in the simulated environment. The RCOutput class
 *          bridges ArduPilot's motor mixing and servo control subsystems with
 *          external physics simulators (JSBSim, Gazebo, etc.) and visualization
 *          tools.
 * 
 *          Key responsibilities:
 *          - Simulate PWM output channels for servo and motor control
 *          - Forward motor outputs to physics simulators for vehicle dynamics
 *          - Multicast servo outputs to visualization and monitoring tools
 *          - Emulate safety switch behavior (SAFETY_DISARMED/SAFETY_ARMED)
 *          - Implement cork/push buffering for atomic multi-channel updates
 *          - Provide ESC telemetry simulation via AP_ESC_Telem_SITL
 *          - Emulate serial LED protocols (NeoPixel, ProfiLED)
 *          - Support configurable output frequencies and dshot protocol emulation
 * 
 *          Integration points:
 *          - AP_Motors: Receives motor mixing outputs for multirotors
 *          - SRV_Channel: Receives servo positioning commands
 *          - SITL_State: Forwards outputs to physics simulation
 *          - AP_ESC_Telem_SITL: Provides simulated ESC telemetry feedback
 * 
 * @warning Output values written to this class directly drive the physics
 *          simulation. Incorrect output values will result in incorrect
 *          simulated vehicle behavior, potentially causing crashes in SITL.
 * 
 * @note This implementation is SITL-specific and only compiled when
 *       CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * 
 * Source: libraries/AP_HAL_SITL/RCOutput.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL.h"
#include <AP_ESC_Telem/AP_ESC_Telem_SITL.h>

/**
 * @class HALSITL::RCOutput
 * @brief SITL implementation of RC servo and motor output interface
 * 
 * @details RCOutput simulates PWM output channels for SITL simulation,
 *          translating high-level servo and motor commands from ArduPilot's
 *          control subsystems into simulated outputs that drive external
 *          physics simulators and visualization tools.
 * 
 *          Output Flow:
 *          1. AP_Motors/SRV_Channel writes PWM values (1000-2000μs typically)
 *          2. Cork/push mechanism allows atomic multi-channel updates
 *          3. Safety switch state determines if outputs are enabled
 *          4. Outputs forwarded to SITL_State for physics simulation
 *          5. ESC telemetry generated based on output values
 * 
 *          Thread Safety: This class is called from the main vehicle thread
 *          and relies on SITL_State for synchronization with simulator.
 * 
 *          Output Channels: Supports SITL_NUM_CHANNELS output channels
 *          (typically 16) for servos, motors, and other actuators.
 * 
 * @note Outputs are buffered when corked and only sent to simulator on push()
 * @warning This class drives simulated vehicle physics - output errors cause
 *          incorrect simulated flight behavior
 * 
 * @see AP_HAL::RCOutput Base class interface definition
 * @see SITL_State Physics simulator integration
 * @see AP_ESC_Telem_SITL ESC telemetry simulation
 */
class HALSITL::RCOutput : public AP_HAL::RCOutput {
public:
    /**
     * @brief Construct SITL RC output handler
     * 
     * @details Initializes the simulated RC output subsystem with a reference
     *          to the SITL state manager. Sets default output frequency to 50Hz
     *          (standard analog servo rate).
     * 
     * @param[in] sitlState Pointer to SITL_State for physics simulator integration
     * 
     * @note Constructor is explicit to prevent implicit conversions
     * @note Default frequency of 50Hz matches standard analog servo refresh rate
     */
    explicit RCOutput(SITL_State *sitlState): _sitlState(sitlState), _freq_hz(50) {}
    
    /**
     * @brief Initialize RC output subsystem
     * 
     * @details Performs initialization of the simulated RC output system,
     *          including ESC telemetry setup and initial state configuration.
     *          Called once during HAL initialization.
     * 
     * @note Called from HAL initialization sequence, not user code
     */
    void init() override;
    
    /**
     * @brief Set output frequency for specified channels
     * 
     * @details Configures the simulated PWM output frequency for the channels
     *          specified in the channel mask. Higher frequencies (200-400Hz)
     *          are typically used for motor ESCs, while lower frequencies
     *          (50Hz) are used for analog servos.
     * 
     * @param[in] chmask  Bitmask of channels to configure (bit 0 = channel 0, etc.)
     * @param[in] freq_hz Output frequency in Hertz (typically 50Hz for servos,
     *                    200-400Hz for ESCs)
     * 
     * @note In SITL, frequency affects timing of output updates to simulator
     * @note Digital protocols (DShot) use different frequency values
     * 
     * @see get_freq() Query current frequency for a channel
     */
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    
    /**
     * @brief Get output frequency for a channel
     * 
     * @details Returns the currently configured output frequency for the
     *          specified channel.
     * 
     * @param[in] ch Channel number (0-based, typically 0-15)
     * 
     * @return Current output frequency in Hertz
     * 
     * @note Returns most recently configured frequency for the channel
     */
    uint16_t get_freq(uint8_t ch) override;
    
    /**
     * @brief Enable output on a specific channel
     * 
     * @details Enables PWM output on the specified channel, allowing servo
     *          and motor commands to be sent to the physics simulator.
     * 
     * @param[in] ch Channel number to enable (0-based)
     * 
     * @note Channel must be enabled for outputs to affect simulation
     * @note Enabled state is separate from safety switch state
     */
    void enable_ch(uint8_t ch) override;
    
    /**
     * @brief Disable output on a specific channel
     * 
     * @details Disables PWM output on the specified channel, preventing
     *          servo and motor commands from being sent to the physics simulator.
     * 
     * @param[in] ch Channel number to disable (0-based)
     * 
     * @note Used for channels not connected to actuators
     */
    void disable_ch(uint8_t ch) override;
    
    /**
     * @brief Write PWM value to a single output channel
     * 
     * @details Writes a PWM pulse width to the specified output channel.
     *          The value is sent to the physics simulator to control simulated
     *          servos, motors, or other actuators.
     * 
     * @param[in] ch        Channel number (0-based, typically 0-15)
     * @param[in] period_us PWM pulse width in microseconds (typically 1000-2000μs
     *                      for servos, 1000-2000μs for ESCs)
     * 
     * @note If corked, value is buffered until push() is called
     * @note Values outside normal ranges may produce unexpected simulation behavior
     * @note Safety switch state affects whether outputs reach simulator
     * 
     * @warning Output values directly drive simulated vehicle dynamics
     * 
     * @see cork() Buffer multiple channel updates
     * @see push() Send buffered updates atomically
     */
    void write(uint8_t ch, uint16_t period_us) override;
    
    /**
     * @brief Read current PWM value from a single output channel
     * 
     * @details Returns the most recently written PWM value for the specified
     *          channel. This reflects the commanded output, not feedback from
     *          the actuator (use ESC telemetry for feedback).
     * 
     * @param[in] ch Channel number (0-based)
     * 
     * @return PWM pulse width in microseconds
     * 
     * @note Returns commanded value, not actual actuator position
     * @note If channel never written, returns 0
     */
    uint16_t read(uint8_t ch) override;
    
    /**
     * @brief Read PWM values from multiple output channels
     * 
     * @details Reads current PWM values for multiple output channels into
     *          the provided array.
     * 
     * @param[out] period_us Array to receive PWM values in microseconds
     * @param[in]  len       Number of channels to read
     * 
     * @note Reads up to len channels starting from channel 0
     * @note Array must be allocated by caller with sufficient space
     */
    void read(uint16_t* period_us, uint8_t len) override;
    
    /**
     * @brief Begin atomic multi-channel update (cork outputs)
     * 
     * @details Starts buffering output channel writes without sending them to
     *          the physics simulator. Subsequent write() calls will store values
     *          in a pending buffer until push() is called. This allows multiple
     *          channels to be updated atomically, which is critical for motor
     *          mixing where all motor outputs must change simultaneously to
     *          prevent control glitches in the simulation.
     * 
     * @note Must be paired with push() to send buffered outputs
     * @note Multiple write() calls between cork() and push() are accumulated
     * @note Used by AP_Motors to ensure all motor channels update simultaneously
     * 
     * @warning Outputs remain at previous values until push() is called
     * 
     * @see push() Send all buffered outputs to simulator
     * @see write() Buffer individual channel updates when corked
     */
    void cork(void) override;
    
    /**
     * @brief Send buffered outputs to simulator (push corked outputs)
     * 
     * @details Sends all output values buffered since cork() was called to the
     *          physics simulator. This ensures atomic multi-channel updates,
     *          which is essential for proper motor mixing simulation where all
     *          motor commands must take effect simultaneously.
     * 
     *          The push operation:
     *          1. Checks safety switch state (outputs disabled if SAFETY_DISARMED)
     *          2. Forwards enabled channel outputs to SITL_State
     *          3. Updates ESC telemetry simulation based on motor outputs
     *          4. Multicasts outputs to visualization tools
     *          5. Clears the corked flag
     * 
     * @note Has no effect if cork() was not called
     * @note Safety switch state is enforced during push
     * @note This is when outputs actually affect the physics simulation
     * 
     * @warning This is the point where motor outputs drive simulated vehicle dynamics
     * 
     * @see cork() Begin buffering outputs
     * @see force_safety_on() Disable outputs via safety switch
     */
    void push(void) override;

    /**
     * @brief Force safety switch to ON (disarmed) state
     * 
     * @details Simulates engaging the safety switch, which disables all PWM
     *          outputs from reaching the physics simulator. This emulates the
     *          hardware safety switch behavior on real flight controllers.
     *          When safety is on, motors and servos are disabled regardless
     *          of commanded outputs.
     * 
     * @return true Always succeeds in SITL
     * 
     * @note In SAFETY_DISARMED state, push() will not send outputs to simulator
     * @note Used to simulate pre-arm safety conditions
     * @note Different from disabling individual channels with disable_ch()
     * 
     * @warning All motor outputs cease when safety is engaged - vehicle will
     *          drop in simulation if engaged during flight
     * 
     * @see force_safety_off() Re-enable outputs
     * @see _safety_switch_state() Query current safety state
     */
    bool force_safety_on(void) override {
        safety_state = AP_HAL::Util::SAFETY_DISARMED;
        return true;
    }
    
    /**
     * @brief Force safety switch to OFF (armed) state
     * 
     * @details Simulates disengaging the safety switch, which enables PWM
     *          outputs to reach the physics simulator. This emulates pressing
     *          the hardware safety button on real flight controllers.
     *          When safety is off, motors and servos operate normally.
     * 
     * @note In SAFETY_ARMED state, push() will send outputs to simulator
     * @note Required before motors will spin in simulation
     * @note Arming checks must still pass separately
     * 
     * @see force_safety_on() Disable outputs
     * @see _safety_switch_state() Query current safety state
     */
    void force_safety_off(void) override {
        safety_state = AP_HAL::Util::SAFETY_ARMED;
    }

    /**
     * @brief Get current safety switch state
     * 
     * @details Returns the current state of the simulated safety switch,
     *          indicating whether outputs are enabled (SAFETY_ARMED) or
     *          disabled (SAFETY_DISARMED).
     * 
     * @return Current safety state (SAFETY_DISARMED or SAFETY_ARMED)
     * 
     * @note Used by AP_HAL::Util to report safety state to ground station
     * @note Safety state affects whether push() sends outputs to simulator
     * 
     * @see force_safety_on() Disable outputs
     * @see force_safety_off() Enable outputs
     */
    AP_HAL::Util::safety_state _safety_switch_state(void) { return safety_state; }

    /**
     * @brief Configure serial LED strip emulation
     * 
     * @details Configures emulation of serial LED protocols (NeoPixel, ProfiLED)
     *          for the specified output channel. SITL simulates addressable RGB
     *          LED strips for visualization purposes.
     * 
     * @param[in] chan       Output channel number for LED data signal
     * @param[in] num_leds   Number of LEDs in the strip (max depends on protocol)
     * @param[in] mode       Output mode (MODE_PWM_NONE for serial LED protocols)
     * @param[in] clock_mask Bitmask for clock signal channel (protocol-specific)
     * 
     * @return true if LED configuration successful, false if invalid parameters
     * 
     * @note Used by AP_Notify to configure status indicator LEDs
     * @note SITL may forward LED state to visualization tools
     * @note Not all serial LED protocols require clock signal
     * 
     * @see set_serial_led_rgb_data() Set individual LED colors
     * @see serial_led_send() Update LED strip
     */
    bool set_serial_led_num_LEDs(const uint16_t chan, uint8_t num_leds, output_mode mode = MODE_PWM_NONE, uint32_t clock_mask = 0) override;
    
    /**
     * @brief Set RGB color for individual LED in serial LED strip
     * 
     * @details Sets the color of a specific LED in an emulated serial LED strip.
     *          Color data is buffered until serial_led_send() is called.
     * 
     * @param[in] chan  Output channel number for LED strip
     * @param[in] led   LED index in strip (0-based, -1 for all LEDs)
     * @param[in] red   Red intensity (0-255)
     * @param[in] green Green intensity (0-255)
     * @param[in] blue  Blue intensity (0-255)
     * 
     * @return true if LED data accepted, false if invalid channel or LED index
     * 
     * @note led = -1 sets all LEDs to the same color
     * @note Color data is buffered until serial_led_send() is called
     * @note Used by AP_Notify for status indication (armed, GPS, etc.)
     * 
     * @see set_serial_led_num_LEDs() Configure LED strip
     * @see serial_led_send() Update LED strip with buffered colors
     */
    bool set_serial_led_rgb_data(const uint16_t chan, int8_t led, uint8_t red, uint8_t green, uint8_t blue) override;
    
    /**
     * @brief Send buffered LED data to serial LED strip
     * 
     * @details Transmits all buffered RGB data to the emulated serial LED strip,
     *          updating the simulated LED colors. In SITL, this may forward LED
     *          state to visualization tools for display.
     * 
     * @param[in] chan Output channel number for LED strip
     * 
     * @return true if LED update successful, false if invalid channel
     * 
     * @note Must call set_serial_led_rgb_data() first to buffer color data
     * @note Emulates timing of real serial LED protocols (WS2812, etc.)
     * @note Called by AP_Notify to update status LEDs
     * 
     * @see set_serial_led_rgb_data() Set LED colors
     * @see set_serial_led_num_LEDs() Configure LED strip
     */
    bool serial_led_send(const uint16_t chan) override;
    
private:
    /**
     * @brief Pointer to SITL state manager for physics simulator integration
     * 
     * @details Provides interface to forward RC outputs to the physics simulator
     *          (JSBSim, Gazebo, etc.) and multicast outputs to visualization tools.
     *          All output values pass through SITL_State to reach the simulation.
     */
    SITL_State *_sitlState;
    
    /**
     * @brief ESC telemetry simulator for motor feedback
     * 
     * @details Generates simulated ESC telemetry (RPM, voltage, current, temperature)
     *          based on motor output values. Provides realistic telemetry feedback
     *          to ArduPilot subsystems that monitor motor health and performance.
     */
    AP_ESC_Telem_SITL *esc_telem;

    /**
     * @brief Default output frequency in Hertz
     * 
     * @details Default PWM output frequency, typically 50Hz for analog servos.
     *          Individual channels may have different frequencies set via set_freq().
     */
    uint16_t _freq_hz;
    
    /**
     * @brief Bitmask of enabled output channels
     * 
     * @details Each bit represents one channel's enable state (bit 0 = channel 0).
     *          Only enabled channels send outputs to the physics simulator.
     */
    uint32_t _enable_mask;
    
    /**
     * @brief Cork state flag for atomic multi-channel updates
     * 
     * @details When true, write() operations buffer values in _pending[] instead
     *          of immediately sending to simulator. Cleared by push().
     */
    bool _corked;
    
    /**
     * @brief Pending output buffer for corked writes
     * 
     * @details Stores PWM values for all channels when corked. Values are sent
     *          to the physics simulator atomically when push() is called, ensuring
     *          simultaneous motor updates for proper control simulation.
     */
    uint16_t _pending[SITL_NUM_CHANNELS];

    /**
     * @brief Simulated safety switch state
     * 
     * @details Emulates hardware safety switch. When SAFETY_DISARMED, outputs
     *          are blocked from reaching the physics simulator. Must be set to
     *          SAFETY_ARMED for motors to operate in simulation.
     * 
     * @note Defaults to SAFETY_DISARMED for safety
     */
    AP_HAL::Util::safety_state safety_state = AP_HAL::Util::safety_state::SAFETY_DISARMED;
};

#endif
