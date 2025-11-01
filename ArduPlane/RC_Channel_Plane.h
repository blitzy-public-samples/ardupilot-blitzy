/**
 * @file RC_Channel_Plane.h
 * @brief RC channel handling for fixed-wing aircraft (ArduPlane)
 * 
 * @details This file defines plane-specific RC (Radio Control) channel behavior,
 *          extending the base RC_Channel and RC_Channels classes with fixed-wing
 *          specific functionality. This includes:
 *          - Plane-specific auxiliary switch functions (crow mode, flare, soaring)
 *          - QuadPlane transition assist handling
 *          - Fixed-wing mode switching logic
 *          - Plane-specific failsafe behavior
 * 
 *          The RC input system processes pilot commands from the radio transmitter
 *          and triggers appropriate flight mode changes, auxiliary functions, and
 *          failsafe responses specific to fixed-wing aircraft.
 * 
 * @note This implementation supports both traditional fixed-wing and QuadPlane
 *       configurations through conditional compilation (HAL_QUADPLANE_ENABLED).
 * 
 * @see RC_Channel base class in libraries/RC_Channel/RC_Channel.h
 * @see ArduPlane/mode.h for flight mode definitions
 */

#pragma once

#include <RC_Channel/RC_Channel.h>

/**
 * @class RC_Channel_Plane
 * @brief Plane-specific RC channel implementation
 * 
 * @details Extends the base RC_Channel class to provide fixed-wing aircraft specific
 *          RC input handling. This class overrides key methods to implement plane-specific
 *          auxiliary functions and mode switching behavior.
 * 
 *          Key plane-specific features handled by this class:
 *          - Crow flaps mode for aerobatic aircraft (split flaps for drag increase)
 *          - Flare function for landing approach
 *          - Soaring mode control (thermal detection and autonomous soaring)
 *          - QuadPlane Q_ASSIST state control (transition assistance)
 *          - Plane-specific flight mode switching
 * 
 *          Each auxiliary channel can be configured via parameters to trigger specific
 *          functions when switch positions change on the transmitter.
 * 
 * @note This class is instantiated for each RC input channel (typically 16 channels)
 *       by the RC_Channels_Plane container class.
 * 
 * @warning Auxiliary function handling is safety-critical - incorrect mode switching
 *          or function activation can lead to loss of control. All auxiliary functions
 *          are subject to arming and flight mode restrictions.
 */
class RC_Channel_Plane : public RC_Channel
{

public:

protected:

    /**
     * @brief Initialize an auxiliary function for this channel
     * 
     * @details Called during system initialization to set up the specified auxiliary
     *          function for this RC channel. This plane-specific implementation ensures
     *          that plane-relevant auxiliary functions are properly configured and that
     *          initial switch positions are handled correctly for fixed-wing aircraft.
     * 
     *          This method is called once per channel during startup to establish the
     *          initial state based on the transmitter switch position.
     * 
     * @param[in] ch_option  The auxiliary function type to initialize (e.g., FLIP, RTL, AUTO)
     * @param[in] ch_flag    The current switch position (LOW, MIDDLE, HIGH)
     * 
     * @note This is called during vehicle initialization before arming
     * 
     * @see do_aux_function() for runtime auxiliary function handling
     */
    void init_aux_function(AUX_FUNC ch_option,
                           AuxSwitchPos ch_flag) override;

    /**
     * @brief Execute a plane-specific auxiliary function
     * 
     * @details Handles auxiliary switch function execution for fixed-wing aircraft.
     *          This override processes plane-specific functions such as crow mode,
     *          flare, soaring control, and QuadPlane assist, delegating to specialized
     *          handler methods for each function type.
     * 
     *          Called by the RC input processing loop whenever an auxiliary switch
     *          position changes or requires processing. The method routes the request
     *          to appropriate plane-specific handlers based on function type.
     * 
     * @param[in] trigger  Auxiliary function trigger containing function type and switch state
     * 
     * @return true if the function was handled successfully, false otherwise
     * 
     * @note Called at RC input processing rate (typically 50Hz)
     * @warning Auxiliary functions may change flight mode or critical flight parameters -
     *          ensure proper arming checks and mode transition validation
     * 
     * @see init_aux_function() for initialization
     * @see do_aux_function_crow_mode(), do_aux_function_flare(), do_aux_function_soaring_3pos()
     */
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

    /**
     * @brief Handle flight mode switch position changes
     * 
     * @details Called when the configured mode switch changes position on the transmitter.
     *          This plane-specific implementation handles fixed-wing flight mode transitions
     *          (MANUAL, STABILIZE, FBWA, FBWB, AUTO, RTL, LOITER, etc.) based on the
     *          switch position and configured mode mappings.
     * 
     *          The method validates that the requested mode is available and appropriate
     *          for the current vehicle state before initiating the mode change.
     * 
     * @param[in] new_pos  New mode switch position (typically 6-position switch encoding)
     * 
     * @note This is a critical safety function - mode changes affect flight control behavior
     * @warning Mode transitions are restricted based on arming state and failsafe conditions
     * 
     * @see Plane::set_mode() for mode transition logic
     * @see RC_Channel::read_mode_switch() for switch reading
     */
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

private:

    /**
     * @brief Request a flight mode change via auxiliary function
     * 
     * @details Internal helper method to change flight mode when triggered by an
     *          auxiliary switch. Validates the mode change request and delegates to
     *          the plane's mode controller. This is used for auxiliary switches
     *          configured to directly select specific flight modes.
     * 
     * @param[in] number   Flight mode number to change to (e.g., Mode::Number::MANUAL)
     * @param[in] ch_flag  Switch position that triggered the change (for 3-position switches)
     * 
     * @note Mode changes are subject to arming checks and mode availability
     * @warning Critical flight safety function - mode changes affect control behavior
     * 
     * @see Plane::set_mode() for actual mode transition execution
     */
    void do_aux_function_change_mode(Mode::Number number,
                                     AuxSwitchPos ch_flag);

#if HAL_QUADPLANE_ENABLED
    /**
     * @brief Handle QuadPlane Q_ASSIST state auxiliary function
     * 
     * @details Controls the Q_ASSIST feature which provides automatic VTOL assistance
     *          during fixed-wing flight when needed (e.g., low airspeed, high attitude error).
     *          This function allows the pilot to enable/disable the automatic transition
     *          assistance via an auxiliary switch.
     * 
     *          Q_ASSIST automatically engages VTOL motors to assist fixed-wing flight when:
     *          - Airspeed drops below threshold
     *          - Attitude error exceeds limits
     *          - Altitude loss exceeds configured value
     * 
     * @param[in] ch_flag  Switch position (LOW=disable assist, HIGH=enable assist)
     * 
     * @note Only available when compiled with HAL_QUADPLANE_ENABLED
     * @warning Q_ASSIST behavior affects transition safety - ensure proper configuration
     * 
     * @see QuadPlane::assist_active for assist state logic
     */
    void do_aux_function_q_assist_state(AuxSwitchPos ch_flag);
#endif

    /**
     * @brief Handle crow flaps mode auxiliary function
     * 
     * @details Controls crow flaps configuration for aerobatic and landing maneuvers.
     *          Crow mode splits the flaps (ailerons move up, flaps move down) to create
     *          drag without pitching moment, useful for steep descents and spot landings.
     * 
     *          When activated, this function coordinates multiple servo outputs to achieve
     *          the crow flap configuration while maintaining roll control authority.
     * 
     * @param[in] ch_flag  Switch position controlling crow mode intensity
     *                     - LOW: Crow mode disabled
     *                     - MIDDLE: Partial crow (if configured)
     *                     - HIGH: Full crow flaps
     * 
     * @note Requires proper servo function assignments for crow flap operation
     * @warning Crow mode significantly increases drag - ensure sufficient altitude
     * 
     * @see SRV_Channel for servo output configuration
     */
    void do_aux_function_crow_mode(AuxSwitchPos ch_flag);

    /**
     * @brief Handle soaring mode 3-position switch auxiliary function
     * 
     * @details Controls the autonomous soaring system for sailplanes and motor gliders.
     *          Soaring mode uses thermal detection algorithms to identify and exploit
     *          rising air currents for maintaining or gaining altitude without power.
     * 
     *          Switch positions:
     *          - LOW: Soaring disabled
     *          - MIDDLE: Soaring enabled with automatic thermal detection
     *          - HIGH: Force enable soaring even without strong thermal indication
     * 
     * @param[in] ch_flag  Switch position selecting soaring mode
     * 
     * @note Requires proper soaring configuration and typically needs variometer/airspeed
     * @warning Soaring can deviate significantly from planned flight path
     * 
     * @see AP_Soaring library for soaring algorithm implementation
     */
    void do_aux_function_soaring_3pos(AuxSwitchPos ch_flag);

    /**
     * @brief Handle landing flare auxiliary function
     * 
     * @details Manually triggers or controls the landing flare maneuver. During landing,
     *          the flare raises the nose just before touchdown to reduce vertical speed
     *          and touchdown impact. This function allows manual flare triggering or
     *          manual override of automatic flare timing.
     * 
     *          Flare execution:
     *          - Reduces throttle to idle
     *          - Commands pitch-up to prescribed flare angle
     *          - Holds heading and minimizes roll
     * 
     * @param[in] ch_flag  Switch position (HIGH=activate flare, LOW=normal landing)
     * 
     * @note Typically used during AUTO landing mode
     * @warning Improper flare timing can cause hard landings or stalls - ensure adequate
     *          altitude and airspeed for manual flare activation
     * 
     * @see Mode::Auto for automatic landing and flare logic
     */
    void do_aux_function_flare(AuxSwitchPos ch_flag);
};

/**
 * @class RC_Channels_Plane
 * @brief Plane-specific RC channels container and manager
 * 
 * @details Manages the collection of RC input channels for fixed-wing aircraft,
 *          providing plane-specific implementations of failsafe detection, input
 *          validation, and mode switching. This class owns the array of individual
 *          RC_Channel_Plane objects (one per input channel) and coordinates their
 *          operation.
 * 
 *          Key responsibilities:
 *          - RC failsafe detection based on plane-specific criteria
 *          - Input validity checking for fixed-wing flight
 *          - Mode switch reading and processing
 *          - Arming channel identification for safety
 * 
 *          The RC input system operates at approximately 50Hz, reading PWM values
 *          from the receiver and processing auxiliary functions and mode changes.
 * 
 * @note ArduPlane typically supports 16 RC input channels (NUM_RC_CHANNELS)
 * @warning RC failsafe detection is critical for flight safety - loss of RC link
 *          triggers failsafe actions to protect the aircraft
 * 
 * @see RC_Channel_Plane for individual channel implementation
 * @see Plane::failsafe for failsafe response logic
 */
class RC_Channels_Plane : public RC_Channels
{
public:

    /**
     * @brief Array of plane-specific RC channel objects
     * 
     * @details Storage for all RC input channels. Each channel is a full
     *          RC_Channel_Plane object capable of processing plane-specific
     *          auxiliary functions and mode switching.
     */
    RC_Channel_Plane obj_channels[NUM_RC_CHANNELS];

    /**
     * @brief Get pointer to a specific RC channel
     * 
     * @details Provides access to individual RC channels by index. Returns a pointer
     *          to the RC_Channel_Plane object for the specified channel, or nullptr
     *          if the channel index is out of range.
     * 
     * @param[in] chan  Channel index (0-based, typically 0-15)
     * 
     * @return Pointer to RC_Channel_Plane object, or nullptr if chan >= NUM_RC_CHANNELS
     * 
     * @note Channel indexing is zero-based (channel 1 on transmitter = index 0)
     */
    RC_Channel_Plane *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    /**
     * @brief Check if RC input is in failsafe condition
     * 
     * @details Determines whether the RC input system is in a failsafe state for
     *          fixed-wing aircraft. Plane-specific failsafe criteria include:
     *          - Loss of RC signal (no pulses received)
     *          - RC signal quality below threshold
     *          - Throttle failsafe conditions for fixed-wing
     * 
     *          When failsafe is detected, the vehicle will execute configured
     *          failsafe actions (RTL, LAND, or continue mission).
     * 
     * @return true if RC input is in failsafe, false if RC is healthy
     * 
     * @note Called at main loop rate to continuously monitor RC health
     * @warning This is a critical safety function - false negatives could prevent
     *          failsafe activation, false positives could trigger unnecessary RTL
     * 
     * @see Plane::failsafe_check() for failsafe action execution
     * @see FS_SHORT_TIMEOUT, FS_LONG_TIMEOUT parameters for failsafe timing
     */
    bool in_rc_failsafe() const override;

    /**
     * @brief Check if RC input has valid data
     * 
     * @details Verifies that RC input is present and valid for fixed-wing flight
     *          control. This is less strict than failsafe checking - it confirms
     *          that usable RC data exists, even if signal quality is degraded.
     * 
     *          Valid input criteria:
     *          - RC pulses are being received
     *          - Pulse widths are within valid range (typically 900-2100μs)
     *          - Minimum required channels are present
     * 
     * @return true if RC input is valid and can be used for control, false otherwise
     * 
     * @note Used during initialization and arming checks
     * @see has_valid_input() is checked before allowing arming
     */
    bool has_valid_input() const override;

    /**
     * @brief Get the RC channel configured for arming control
     * 
     * @details Returns a pointer to the RC channel designated for arming/disarming
     *          control via auxiliary switch. ArduPlane supports arming via RC switch
     *          for safety and convenience. If no arming channel is configured, returns
     *          nullptr and arming must be done via other means (GCS command, button).
     * 
     * @return Pointer to arming RC channel, or nullptr if not configured
     * 
     * @note Configure arming channel via RCx_OPTION parameter (option 41 = Arm/Disarm)
     * @warning Arming channel should be configured on a reliable switch position to
     *          prevent accidental disarming during flight
     * 
     * @see AP_Arming for arming check logic
     */
    RC_Channel *get_arming_channel(void) const override;

    /**
     * @brief Read and process the mode switch position
     * 
     * @details Reads the current position of the flight mode switch and triggers
     *          mode changes if the position has changed. This plane-specific implementation
     *          handles the mapping between switch positions (typically 6-position encoding)
     *          and fixed-wing flight modes.
     * 
     *          The mode switch is typically configured on channel 8 (FLTMODE_CH parameter)
     *          with 6 positions corresponding to 6 configurable flight modes.
     * 
     * @note Called at RC processing rate (approximately 50Hz)
     * @warning Mode changes are safety-critical and subject to arming state validation
     * 
     * @see mode_switch_changed() for mode transition handling
     * @see FLTMODE_CH parameter for mode channel configuration
     */
    void read_mode_switch() override;

protected:

    /**
     * @brief Get the flight mode channel number
     * 
     * @details Returns the channel number configured for flight mode selection.
     *          Typically this is channel 8 (configured via FLTMODE_CH parameter).
     * 
     * @return Channel number for mode switch (1-based), or -1 if not configured
     * 
     * @note This callback is not presently used on Plane - mode channel is accessed
     *       directly via parameter lookup for performance reasons
     * @note Return value is 1-based (channel 8 = return value 8, not 7)
     */
    int8_t flight_mode_channel_number() const override;

};
