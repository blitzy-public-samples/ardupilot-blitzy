/**
 * @file RC_Channel_Copter.h
 * @brief Radio control channel interface for multicopter vehicles
 * 
 * @details This file implements multicopter-specific radio control (RC) input
 *          channel handling, extending the base RC_Channel library with copter-specific
 *          auxiliary functions, mode switching, and failsafe behavior. RC channels map
 *          pilot stick inputs (roll, pitch, throttle, yaw) and auxiliary switches to
 *          vehicle control commands and flight mode changes.
 * 
 *          Key responsibilities:
 *          - Process pilot RC stick inputs for attitude and throttle control
 *          - Handle auxiliary switch functions (mode changes, camera trigger, etc.)
 *          - Implement copter-specific failsafe behavior for RC signal loss
 *          - Manage flight mode switching via dedicated mode channel
 *          - Support RC input overrides for companion computer control
 *          - Provide arming/disarming via RC channel inputs
 *          - Enable AHRS trim adjustment using RC stick inputs
 * 
 *          Channel Mapping (default):
 *          - Channel 1: Roll input (aileron)
 *          - Channel 2: Pitch input (elevator)
 *          - Channel 3: Throttle/collective input
 *          - Channel 4: Yaw input (rudder)
 *          - Channel 5+: Auxiliary functions (configurable via RCx_OPTION parameters)
 * 
 *          Common Auxiliary Functions:
 *          - Mode switching (preset flight modes on switch positions)
 *          - RTL (Return to Launch) trigger
 *          - Auto trim enable/disable
 *          - Camera/gimbal control
 *          - Landing gear retract/deploy
 *          - Parachute release
 *          - Motor emergency stop
 *          - Acro trainer enable/disable
 *          - Fence enable/disable
 * 
 * @note RC input is processed at high rate (typically 50-400Hz depending on protocol)
 * @warning Loss of RC signal triggers failsafe behavior - ensure failsafe parameters
 *          are configured appropriately for safe vehicle recovery (RTL, LAND, etc.)
 * 
 * @see RC_Channel (base class in libraries/RC_Channel/)
 * @see Copter::read_radio() (main loop RC input processing)
 * @see Copter::set_mode() (flight mode change handling)
 */

#pragma once

#include <RC_Channel/RC_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include "mode.h"

/**
 * @class RC_Channel_Copter
 * @brief Multicopter-specific RC channel implementation
 * 
 * @details Extends the base RC_Channel class with copter-specific auxiliary function
 *          handling and mode switching logic. Each RC channel can be assigned an
 *          auxiliary function via RCx_OPTION parameter, allowing pilot control of
 *          features like flight mode changes, camera triggering, landing gear, etc.
 * 
 *          This class overrides virtual methods from RC_Channel to provide multicopter-specific
 *          behavior for:
 *          - Auxiliary function initialization and execution
 *          - Mode switch position change handling
 *          - Copter-specific switch functions (air mode, force flying)
 * 
 *          Auxiliary functions are triggered when RC channel PWM values cross defined
 *          thresholds (low/middle/high positions for 3-position switches, or binary
 *          low/high for 2-position switches).
 * 
 * @note Inherits standard channel properties: min/max/trim PWM values, reversed flag,
 *       dead zone, exponential curves, etc. from base RC_Channel class
 * @warning Auxiliary functions can trigger safety-critical actions (motor stop, parachute
 *          release) - verify switch positions before arming
 */
class RC_Channel_Copter : public RC_Channel
{

public:

protected:

    /**
     * @brief Initialize a copter-specific auxiliary function on this channel
     * 
     * @details Called once during vehicle initialization for each configured auxiliary
     *          function. Sets up initial state for functions that require startup
     *          configuration (e.g., mode switches, persistent features).
     * 
     *          The __INITFUNC__ attribute places this in a special init section that
     *          may be discarded after boot on memory-constrained targets.
     * 
     * @param[in] ch_option  The auxiliary function type assigned to this channel (from AUX_FUNC enum)
     * @param[in] AuxSwitchPos  Initial switch position (LOW, MIDDLE, HIGH)
     * 
     * @note Called before main loop starts, during Copter::init_ardupilot()
     * @note Override of RC_Channel::init_aux_function() with copter-specific initialization
     */
    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    
    /**
     * @brief Execute a copter-specific auxiliary function
     * 
     * @details Handles auxiliary function execution when RC channel crosses switch thresholds.
     *          Processes copter-specific functions including:
     *          - Flight mode changes (STABILIZE, ALT_HOLD, LOITER, RTL, AUTO, etc.)
     *          - Save trim (stores current stick positions as new trim values)
     *          - Save waypoint (stores current position as rally point or mission item)
     *          - Camera/gimbal control (trigger, mode changes)
     *          - Landing gear retract/deploy
     *          - Parachute release
     *          - Motor emergency stop
     *          - Precision loiter enable
     *          - Air mode enable (maintains attitude control at zero throttle)
     *          - Force flying mode (disables landing detection)
     *          - Acro trainer on/off
     *          - Auto trim via AHRS feedback
     * 
     *          Functions not handled by this copter-specific implementation are passed
     *          to the base RC_Channel::do_aux_function() for generic handling.
     * 
     * @param[in] trigger  Structure containing function type, switch position change,
     *                     and activation type (toggle, momentary, etc.)
     * 
     * @return true if function was handled successfully, false if function unknown or failed
     * 
     * @note Called at RC input rate (typically 50-400Hz) when switch position changes
     * @note Some functions are momentary (active only while switch held), others are latching
     * @warning Mode changes may fail if vehicle is not in correct state (e.g., cannot enter
     *          AUTO mode without a mission loaded)
     * 
     * @see RC_Channel::AuxFuncTrigger
     * @see Copter::set_mode()
     */
    bool do_aux_function(const AuxFuncTrigger &trigger) override;

private:

    /**
     * @brief Handle flight mode change from auxiliary function switch
     * 
     * @details Attempts to change vehicle flight mode based on auxiliary switch position.
     *          Validates that the requested mode is available and that vehicle state allows
     *          the mode change. Some modes have prerequisites (e.g., GPS lock for LOITER,
     *          mission loaded for AUTO).
     * 
     * @param[in] mode     The flight mode to change to (from Mode::Number enum)
     * @param[in] ch_flag  Switch position: LOW=disabled/off, MIDDLE=middle position, HIGH=enabled/on
     * 
     * @note Typically called from do_aux_function() when mode switch position changes
     * @note Mode change may fail if prerequisites not met - check GCS messages for failure reason
     * @warning Unexpected mode changes can be disorienting to pilot - ensure mode switches
     *          are in expected positions before arming
     * 
     * @see Copter::set_mode()
     * @see Mode::Number (mode enumeration in mode.h)
     */
    void do_aux_function_change_mode(const Mode::Number mode,
                                     const AuxSwitchPos ch_flag);
    
    /**
     * @brief Handle air mode enable/disable from auxiliary function switch
     * 
     * @details Air mode maintains full attitude control authority even at zero throttle,
     *          allowing aggressive acrobatic maneuvers and preventing "drops" during flips.
     *          When enabled, motors maintain minimum spin to provide control torque.
     * 
     *          Behavior:
     *          - HIGH: Enable air mode (motors maintain min throttle, full control at zero stick)
     *          - LOW: Disable air mode (normal throttle behavior, no control at zero throttle)
     * 
     * @param[in] ch_flag  Switch position: LOW=disable air mode, HIGH=enable air mode
     * 
     * @note Air mode is primarily useful for acrobatic flight and racing
     * @warning With air mode enabled, motors will spin at minimum throttle even at zero stick
     *          input - exercise caution during ground operations
     */
    void do_aux_function_change_air_mode(const AuxSwitchPos ch_flag);
    
    /**
     * @brief Handle force-flying mode enable/disable from auxiliary function switch
     * 
     * @details Force-flying mode disables landing detection, preventing the vehicle from
     *          automatically disarming when it thinks it has landed. Useful for:
     *          - Flying from moving platforms (boats, vehicles)
     *          - Landing on unstable surfaces
     *          - Preventing false landing detection during aggressive maneuvers
     *          - Bench testing with motors running
     * 
     *          Behavior:
     *          - HIGH: Force flying mode enabled (landing detection disabled)
     *          - LOW: Normal landing detection active
     * 
     * @param[in] ch_flag  Switch position: LOW=normal landing detection, HIGH=force flying
     * 
     * @note Landing detection normally auto-disarms motors after touchdown for safety
     * @warning With force-flying enabled, vehicle will NOT automatically disarm after landing -
     *          pilot must manually disarm to stop motors
     */
    void do_aux_function_change_force_flying(const AuxSwitchPos ch_flag);

    /**
     * @brief Handle dedicated flight mode switch position changes
     * 
     * @details Called when the dedicated flight mode channel (typically channel 5, configured
     *          via FLTMODE_CH parameter) changes position. Maps 6-position switch to 6
     *          configurable flight modes via FLTMODE1-FLTMODE6 parameters.
     * 
     *          Switch position mapping (PWM thresholds):
     *          - Position 0: PWM < 1230 → FLTMODE1
     *          - Position 1: 1230 ≤ PWM < 1360 → FLTMODE2
     *          - Position 2: 1360 ≤ PWM < 1490 → FLTMODE3
     *          - Position 3: 1490 ≤ PWM < 1620 → FLTMODE4
     *          - Position 4: 1620 ≤ PWM < 1749 → FLTMODE5
     *          - Position 5: PWM ≥ 1749 → FLTMODE6
     * 
     * @param[in] new_pos  New switch position (0-5) derived from PWM value
     * 
     * @note Override of RC_Channel::mode_switch_changed() with copter-specific behavior
     * @note Mode changes may fail if target mode prerequisites not met (GPS lock, mission loaded, etc.)
     * @warning Verify mode switch position matches desired flight mode before takeoff
     * 
     * @see Copter::read_mode_switch()
     * @see RC_Channel::mode_switch_changed()
     */
    void mode_switch_changed(modeswitch_pos_t new_pos) override;

};

/**
 * @class RC_Channels_Copter
 * @brief Collection manager for all RC channels on multicopter vehicles
 * 
 * @details Manages the complete set of radio control input channels for a multicopter,
 *          providing copter-specific implementations of:
 *          - RC input validity checking (signal present, not timed out)
 *          - RC failsafe detection and state management
 *          - Arming/disarming channel configuration
 *          - Throttle position checks for arming safety
 *          - Flight mode channel identification
 *          - AHRS auto-trim using RC stick inputs
 * 
 *          The class maintains an array of RC_Channel_Copter objects (one per channel)
 *          and provides channel access with bounds checking. Integrates with the vehicle's
 *          failsafe system to handle RC signal loss gracefully.
 * 
 *          Channel Array:
 *          - Contains NUM_RC_CHANNELS (typically 16) channel objects
 *          - Channels accessed via zero-based index (0-15)
 *          - Each channel can be independently configured via parameters
 * 
 *          Failsafe Behavior:
 *          - Monitors RC signal timeout (no updates received within FS_TIMEOUT period)
 *          - Checks for invalid PWM values indicating receiver issues
 *          - Triggers Copter failsafe actions when RC lost (RTL, LAND, or continue mission)
 * 
 * @note This is a singleton instance accessed via AP::RC() or copter.rc()
 * @warning Proper failsafe configuration is critical for safe vehicle recovery when
 *          RC signal is lost - test failsafe behavior before flying
 * 
 * @see RC_Channels (base class in libraries/RC_Channel/)
 * @see Copter::failsafe_radio_on_event() (failsafe handler)
 * @see Copter::read_radio() (RC input reading in main loop)
 */
class RC_Channels_Copter : public RC_Channels
{
public:

    /**
     * @brief Check if RC input is valid and current
     * 
     * @details Determines if RC receiver is providing valid input by checking:
     *          - RC update received recently (within timeout period)
     *          - PWM values are within valid range (not 0 or 0xFFFF)
     *          - Receiver reports healthy status
     * 
     *          Used to determine if pilot has control authority and whether failsafe
     *          actions should be triggered.
     * 
     * @return true if RC input is valid and current, false if lost or invalid
     * 
     * @note Called frequently during failsafe evaluation
     * @note Override of RC_Channels::has_valid_input() with copter-specific checks
     * 
     * @see in_rc_failsafe()
     * @see Copter::failsafe_radio_on_event()
     */
    bool has_valid_input() const override;
    
    /**
     * @brief Check if vehicle is currently in RC failsafe condition
     * 
     * @details Determines if the vehicle should be in RC failsafe state by checking:
     *          - RC signal timeout (no updates for FS_TIMEOUT milliseconds)
     *          - RC input validity (valid PWM values)
     *          - Failsafe enable parameter (FS_THR_ENABLE)
     *          - Throttle failsafe value trigger (if configured)
     * 
     *          When in RC failsafe, vehicle takes autonomous action based on FS_THR_ENABLE:
     *          - 0: Disabled (no action)
     *          - 1: Enabled Always RTL
     *          - 2: Enabled Continue with Mission in Auto
     *          - 3: Enabled Always Land
     * 
     * @return true if vehicle is in RC failsafe, false if RC input is healthy
     * 
     * @note Called every main loop iteration to monitor RC health
     * @warning RC failsafe triggers autonomous behavior - ensure failsafe parameters are
     *          properly configured for your operational environment
     * 
     * @see has_valid_input()
     * @see Copter::failsafe_radio_on_event()
     * @see FS_THR_ENABLE parameter
     */
    bool in_rc_failsafe() const override;

    /**
     * @brief Get the RC channel configured for arming/disarming
     * 
     * @details Returns pointer to the RC channel configured for arming control via
     *          RCx_OPTION = 153 (ARM/DISARM) auxiliary function. Allows arming/disarming
     *          via RC switch instead of rudder stick motion.
     * 
     * @return Pointer to RC channel configured for arming, nullptr if none configured
     * 
     * @note Arming channel must be configured to HIGH position to arm, LOW to disarm
     * @note If no channel configured, returns nullptr and arming uses rudder stick method
     * 
     * @see RC_Channel::AUX_FUNC::ARMDISARM
     */
    RC_Channel *get_arming_channel(void) const override;

    /**
     * @brief Array of all RC channel objects for this vehicle
     * 
     * @details Contains NUM_RC_CHANNELS RC_Channel_Copter instances representing all
     *          available radio control input channels. Each channel stores PWM input
     *          value, scaling parameters, auxiliary function assignment, and state.
     */
    RC_Channel_Copter obj_channels[NUM_RC_CHANNELS];
    
    /**
     * @brief Get pointer to specific RC channel with bounds checking
     * 
     * @details Provides safe access to individual RC channel objects from the channel array.
     *          Returns nullptr if channel index is out of range instead of causing buffer overrun.
     * 
     * @param[in] chan  Zero-based channel index (0 = channel 1, 1 = channel 2, etc.)
     * 
     * @return Pointer to RC_Channel_Copter object for requested channel, or nullptr if chan >= NUM_RC_CHANNELS
     * 
     * @note Override of RC_Channels::channel() returning copter-specific channel type
     * @note Channel index is zero-based (0-15) but displayed to users as 1-16
     */
    RC_Channel_Copter *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    /**
     * @brief Check if throttle arming checks should be performed
     * 
     * @details Determines if throttle position must be checked before allowing arming.
     *          Returns true if:
     *          - Throttle channel is present and valid
     *          - Vehicle is not in THROW mode (which arms with motors off)
     *          - ARMING_CHECK includes throttle check (ARMING_CHECK_THR)
     * 
     *          When throttle checks are enabled, pilot must have throttle stick at minimum
     *          position to arm the vehicle, preventing unexpected motor spin-up.
     * 
     * @return true if throttle position must be checked before arming, false to skip throttle checks
     * 
     * @note Override of RC_Channels::arming_check_throttle() with copter-specific logic
     * @warning Disabling throttle arming checks can allow arming with throttle raised,
     *          causing immediate motor spin-up - use extreme caution
     * 
     * @see AP_Arming_Copter::arm_checks()
     * @see ARMING_CHECK parameter
     */
    bool arming_check_throttle() const override;

    /**
     * @brief AHRS automatic trim using RC stick inputs (if enabled)
     * 
     * @details Auto-trim system uses accelerometer feedback while vehicle is stationary
     *          on the ground to automatically determine correct RC trim values. This
     *          corrects for RC transmitter trim offsets, ensuring level flight when
     *          sticks are centered.
     * 
     *          Process:
     *          1. Pilot enables auto-trim via auxiliary function switch (while disarmed)
     *          2. Pilot centers all sticks (roll, pitch, yaw) and holds steady
     *          3. System reads accelerometer data to detect any attitude error
     *          4. RC trim values adjusted to null out accelerometer readings
     *          5. New trim values saved to parameters
     * 
     *          This feature is only available if AP_COPTER_AHRS_AUTO_TRIM_ENABLED is defined.
     */
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    /**
     * @brief Handle AHRS auto-trim auxiliary function switch
     * 
     * @details Starts or stops the AHRS auto-trim procedure based on switch position.
     *          Auto-trim only runs when:
     *          - Vehicle is disarmed and on the ground
     *          - Pilot has centered all control sticks
     *          - AHRS has converged to stable attitude estimate
     * 
     * @param[in] ch_flag  Switch position: HIGH=start auto-trim, LOW=stop auto-trim
     * 
     * @note Auto-trim requires vehicle to be stationary on level surface
     * @warning Moving vehicle or control sticks during auto-trim will corrupt trim values
     * 
     * @see auto_trim_run()
     * @see auto_trim_cancel()
     */
    void do_aux_function_ahrs_auto_trim(const RC_Channel::AuxSwitchPos ch_flag);
    
    /**
     * @brief Auto-trim system state
     * 
     * @details Tracks whether AHRS auto-trim procedure is currently running.
     *          Used to maintain auto-trim state across main loop iterations.
     */
    struct {
        bool running;  ///< True if auto-trim procedure is currently active
    } auto_trim;
    
    /**
     * @brief Execute one iteration of AHRS auto-trim procedure
     * 
     * @details Called repeatedly in main loop while auto-trim is active. Reads AHRS
     *          accelerometer data and adjusts RC trim values to minimize attitude error.
     *          Accumulates trim corrections over multiple iterations for stability.
     * 
     * @note Called from Copter::update_auto_trim() in main loop
     * @note Automatically stops if vehicle arms or moves
     */
    void auto_trim_run();
    
    /**
     * @brief Cancel AHRS auto-trim procedure
     * 
     * @details Stops auto-trim process without saving trim values. Called when:
     *          - Auto-trim switch moved to off position
     *          - Vehicle is armed during auto-trim
     *          - Excessive stick movement detected during trim
     * 
     * @note Does not restore previous trim values - use with caution
     */
    void auto_trim_cancel();
#endif  // AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    
    /**
     * @brief Save current RC stick positions as new trim values
     * 
     * @details Stores current RC channel PWM input values as trim values for roll, pitch,
     *          and yaw channels. Trim values define the "center" stick position that
     *          commands zero rate/attitude. Called when:
     *          - Pilot triggers "Save Trim" auxiliary function (RCx_OPTION = 7)
     *          - Auto-trim procedure completes successfully
     * 
     *          Saved trim values persist across power cycles and are stored in parameters:
     *          - RC1_TRIM (roll)
     *          - RC2_TRIM (pitch)
     *          - RC4_TRIM (yaw)
     * 
     * @note Pilot should have sticks centered at desired neutral position before saving trim
     * @note Throttle (channel 3) trim is not saved, as throttle minimum is used instead
     * @warning Incorrect trim values can cause vehicle to drift when sticks centered
     * 
     * @see do_aux_function() (AUX_FUNC::SAVE_TRIM)
     */
    void save_trim();

protected:

    /**
     * @brief Get the channel number configured for flight mode switching
     * 
     * @details Returns the RC channel number (1-16) configured as the dedicated flight
     *          mode switch via FLTMODE_CH parameter. This channel's PWM value is divided
     *          into 6 segments, each mapped to a different flight mode (FLTMODE1-FLTMODE6).
     * 
     *          Common configurations:
     *          - FLTMODE_CH = 5: Channel 5 is mode switch (default)
     *          - FLTMODE_CH = 0: No dedicated mode switch (use RCx_OPTION mode selection instead)
     * 
     * @return Channel number (1-16) configured for mode switching, or -1 if none configured
     * 
     * @note Override of RC_Channels::flight_mode_channel_number()
     * @note If FLTMODE_CH = 0, flight modes must be changed via auxiliary function switches
     *       or GCS commands instead of dedicated mode channel
     * 
     * @see FLTMODE_CH parameter
     * @see Copter::read_mode_switch()
     */
    int8_t flight_mode_channel_number() const override;

};
