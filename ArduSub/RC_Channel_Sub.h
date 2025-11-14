/**
 * @file RC_Channel_Sub.h
 * @brief ArduSub-specific RC (Radio Control) channel handling
 * 
 * @details This file defines RC channel classes for the ArduSub underwater vehicle.
 *          RC_Channel_Sub extends the base RC_Channel class with Sub-specific
 *          auxiliary function handling and mode switching for underwater operations.
 *          RC_Channels_Sub manages the collection of RC channels with Sub-specific
 *          failsafe detection and input validation tailored for underwater environments.
 * 
 *          Key Sub-specific features:
 *          - Auxiliary switch functions for underwater-specific controls (lights, camera tilt, etc.)
 *          - Mode switching adapted for underwater flight modes (Manual, Stabilize, Depth Hold, etc.)
 *          - RC failsafe detection considering underwater communication constraints
 *          - Input validation accounting for joystick control patterns common in Sub operations
 * 
 * @note ArduSub uses joystick input (typically via MAVLink) in addition to traditional RC
 * @warning RC failsafe behavior is critical for safe underwater vehicle operation
 * 
 * @see RC_Channel (base class in libraries/RC_Channel/)
 * @see Sub.h (main ArduSub vehicle class)
 */

#pragma once

#include <RC_Channel/RC_Channel.h>
#include "config.h"

#if AP_SUB_RC_ENABLED

/**
 * @class RC_Channel_Sub
 * @brief ArduSub-specific RC channel implementation
 * 
 * @details Extends RC_Channel with Sub-specific auxiliary function handling and mode
 *          switching. This class handles the interpretation of RC input for underwater
 *          vehicle operations, including specialized auxiliary functions like lights,
 *          camera controls, and gain switches that are unique to submersible vehicles.
 * 
 *          Inheritance: RC_Channel_Sub -> RC_Channel (base class)
 * 
 *          The class overrides key methods to provide Sub-specific behavior:
 *          - Auxiliary function initialization for Sub-specific switches
 *          - Auxiliary function execution for underwater vehicle controls
 *          - Mode switch handling for Sub flight modes
 * 
 * @note Sub commonly uses joystick control via MAVLink in addition to traditional RC
 */
class RC_Channel_Sub : public RC_Channel
{

public:

protected:

    /**
     * @brief Initialize an auxiliary function for a Sub-specific RC channel
     * 
     * @details Sets up auxiliary switch functions that are specific to underwater vehicle
     *          operations. This includes specialized functions like lights control, camera
     *          tilt, servo outputs for grippers, and gain adjustments for different operating
     *          conditions (strong currents vs calm water).
     * 
     *          Called during RC channel initialization to configure the behavior of auxiliary
     *          switches based on vehicle parameters.
     * 
     * @param[in] ch_option The auxiliary function type to initialize (e.g., LIGHTS1, CAMERA_TILT)
     * @param[in] pos Initial switch position (not named to match base class signature)
     * 
     * @note Marked with __INITFUNC__ to place in init-only memory section for memory optimization
     * @see do_aux_function() for execution of initialized auxiliary functions
     */
    __INITFUNC__ void init_aux_function(AUX_FUNC ch_option, AuxSwitchPos) override;
    
    /**
     * @brief Execute a Sub-specific auxiliary function based on RC switch input
     * 
     * @details Processes auxiliary switch triggers for Sub-specific functions. Handles
     *          underwater vehicle controls such as:
     *          - Lights on/off/brightness control
     *          - Camera tilt servo positioning
     *          - Gain adjustments for different water conditions
     *          - Mount/gimbal control
     *          - Relay outputs for accessories (grippers, sampling tools, etc.)
     * 
     *          This method is called when an auxiliary switch changes position or needs
     *          to perform its configured action.
     * 
     * @param[in] trigger The auxiliary function trigger containing function type and position
     * 
     * @return true if the auxiliary function was handled successfully, false otherwise
     * 
     * @note Returns false for unsupported auxiliary functions (handled by base class)
     * @see init_aux_function() for initialization of auxiliary functions
     */
    bool do_aux_function(const AuxFuncTrigger &trigger) override;
    
private:
    /**
     * @brief Handle mode switch position changes for Sub flight modes
     * 
     * @details Called when the mode selection switch changes position. Translates RC switch
     *          positions to Sub-specific flight modes such as:
     *          - MANUAL: Direct pilot control without stabilization
     *          - STABILIZE: Attitude stabilization
     *          - DEPTH_HOLD: Automatic depth maintenance
     *          - POSITION_HOLD: Station keeping using positioning sensors
     *          - ALT_HOLD: Altitude/depth hold (legacy mode name)
     *          - AUTO: Autonomous mission execution
     * 
     * @param[in] new_pos New position of the mode switch
     * 
     * @note Mode switching in Sub is adapted for underwater flight modes
     * @warning Improper mode transitions can result in unexpected vehicle behavior underwater
     */
    void mode_switch_changed(modeswitch_pos_t new_pos) override;
};

/**
 * @class RC_Channels_Sub
 * @brief Manager class for all RC channels in ArduSub
 * 
 * @details Extends RC_Channels to manage the collection of RC_Channel_Sub objects with
 *          Sub-specific input validation and failsafe detection. This class handles:
 *          - RC input validation for underwater operations
 *          - RC failsafe detection considering underwater communication constraints
 *          - Throttle arming checks for safe vehicle operation
 *          - Flight mode channel identification
 * 
 *          Inheritance: RC_Channels_Sub -> RC_Channels (base class)
 * 
 *          ArduSub typically operates with joystick input transmitted via MAVLink, which
 *          is processed through the RC channel system. This class ensures proper handling
 *          of both traditional RC and joystick inputs.
 * 
 * @note Sub can operate with traditional RC, joystick via MAVLink, or both
 * @warning RC failsafe detection is critical for underwater vehicle safety
 */
class RC_Channels_Sub : public RC_Channels
{
public:
    /**
     * @brief Check if RC input is valid for vehicle control
     * 
     * @details Validates that RC or joystick input is present and within acceptable ranges
     *          for safe vehicle operation. This check is adapted for Sub's common use of
     *          joystick input via MAVLink in addition to traditional RC receivers.
     * 
     *          Valid input requires:
     *          - Recent input data (not timed out)
     *          - Input values within expected ranges
     *          - Proper signal quality (for traditional RC)
     * 
     * @return true if valid RC/joystick input is available, false otherwise
     * 
     * @note Sub commonly uses joystick input which has different validation criteria than RC
     * @warning Vehicle will enter RC failsafe if input becomes invalid during operation
     * @see in_rc_failsafe() for failsafe state detection
     */
    bool has_valid_input() const override;
    
    /**
     * @brief Determine if vehicle is in RC failsafe condition
     * 
     * @details Checks if the vehicle should enter RC failsafe mode due to loss of RC or
     *          joystick input. For underwater vehicles, failsafe behavior typically includes:
     *          - Disarming (if enabled via parameter)
     *          - Switching to a safe flight mode
     *          - Surface or hold position depending on configuration
     * 
     *          Failsafe detection considers:
     *          - Time since last valid input
     *          - Input signal quality
     *          - Arming state (different thresholds when armed vs disarmed)
     * 
     * @return true if in RC failsafe condition, false if RC input is good
     * 
     * @warning RC failsafe underwater can result in vehicle surfacing or disarming
     * @note Failsafe thresholds and actions are configurable via parameters
     * @see has_valid_input() for input validation
     */
    bool in_rc_failsafe() const override;
    
    /**
     * @brief Check if throttle position is safe for arming
     * 
     * @details Verifies that the throttle channel is in a safe position before allowing
     *          the vehicle to arm. For underwater vehicles, this typically requires:
     *          - Throttle at zero/neutral position
     *          - No large stick deflections
     * 
     *          This is a critical safety check to prevent the vehicle from immediately
     *          moving when armed due to non-centered controls.
     * 
     * @return true if throttle position allows arming, false if throttle must be adjusted
     * 
     * @warning Arming with non-zero throttle can cause immediate unintended vehicle motion
     * @note This check can be bypassed with arming parameters, but not recommended for safety
     * @see AP_Arming for complete arming check system
     */
    bool arming_check_throttle() const override;
    
    /**
     * @brief Array of RC channel objects for all Sub RC channels
     * 
     * @details Fixed-size array containing NUM_RC_CHANNELS individual RC_Channel_Sub objects.
     *          Each channel represents one RC input (stick, switch, or knob) that can be
     *          mapped to vehicle functions.
     * 
     * @note NUM_RC_CHANNELS is typically 16 for ArduSub
     */
    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    
    /**
     * @brief Get pointer to a specific RC channel by index
     * 
     * @details Provides access to individual RC channel objects from the obj_channels array.
     *          Used throughout the codebase to read channel values and configure channel
     *          behavior.
     * 
     * @param[in] chan Zero-based channel index (0 to NUM_RC_CHANNELS-1)
     * 
     * @return Pointer to RC_Channel_Sub object if chan is valid, nullptr if out of range
     * 
     * @note Channel indices are zero-based (0-15 for 16 channels)
     * @warning Always check for nullptr return before dereferencing
     */
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    /**
     * @brief Get the RC channel number used for flight mode selection
     * 
     * @details Returns the channel index (1-based) configured for mode switching via the
     *          FLTMODE_CH parameter. This allows users to configure which RC channel
     *          controls flight mode selection.
     * 
     * @return RC channel number (1-based) for flight mode switching, or -1 if not configured
     * 
     * @note Despite comment, this callback IS used in Sub for mode channel identification
     * @note Return value is 1-based to match RC channel numbering convention
     */
    int8_t flight_mode_channel_number() const override;

};

#else
// Stub implementations when AP_SUB_RC_ENABLED is not defined

/**
 * @class RC_Channel_Sub
 * @brief Stub RC channel class when Sub RC support is disabled
 * 
 * @details Empty stub implementation of RC_Channel_Sub used when AP_SUB_RC_ENABLED
 *          is not defined. Provides minimal base class inheritance to satisfy
 *          compilation requirements when Sub RC functionality is not compiled in.
 * 
 * @note This stub is used in minimal builds where RC support is excluded
 */
class RC_Channel_Sub : public RC_Channel
{

public:

protected:

private:

};

/**
 * @class RC_Channels_Sub
 * @brief Stub RC channels manager when Sub RC support is disabled
 * 
 * @details Minimal stub implementation of RC_Channels_Sub used when AP_SUB_RC_ENABLED
 *          is not defined. Provides basic channel access and always reports no RC
 *          failsafe to allow compilation in minimal build configurations.
 * 
 * @note This stub is used in minimal builds where RC support is excluded
 */
class RC_Channels_Sub : public RC_Channels
{
public:

    /**
     * @brief Array of stub RC channel objects
     * 
     * @details Fixed-size array for minimal builds. Channels have no functionality
     *          when Sub RC support is disabled.
     */
    RC_Channel_Sub obj_channels[NUM_RC_CHANNELS];
    
    /**
     * @brief Get pointer to a stub RC channel by index
     * 
     * @param[in] chan Zero-based channel index (0 to NUM_RC_CHANNELS-1)
     * 
     * @return Pointer to RC_Channel_Sub stub object if valid, nullptr if out of range
     */
    RC_Channel_Sub *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    /**
     * @brief Stub RC failsafe check - always returns false
     * 
     * @details In stub builds with RC disabled, always reports no failsafe to allow
     *          gimbal code and other systems to function. This tells dependent systems
     *          that RC input is acceptable (even though RC is not actually enabled).
     * 
     * @return false Always returns false (no RC failsafe)
     * 
     * @note This is a stub for builds without RC support
     */
    bool in_rc_failsafe() const override { return false; };
    
    /**
     * @brief Stub throttle arming check
     * 
     * @details Provides minimal arming check implementation for builds without full
     *          RC support.
     * 
     * @return true if throttle position allows arming (stub implementation)
     */
    bool arming_check_throttle() const override;

protected:

    /**
     * @brief Stub flight mode channel number
     * 
     * @details Returns the configured flight mode channel number even in stub builds.
     * 
     * @return RC channel number for flight mode switching, or -1 if not configured
     * 
     * @note Despite comment, this callback IS used in Sub
     */
    int8_t flight_mode_channel_number() const override;

};
#endif // AP_SUB_RC_ENABLED


