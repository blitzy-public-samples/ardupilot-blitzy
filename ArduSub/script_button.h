/**
 * @file script_button.h
 * @brief Joystick button interface for Lua scripting integration
 * 
 * @details This file defines the ScriptButton class which provides a bridge between
 *          ArduSub joystick button inputs and Lua scripts. It enables scripts to monitor
 *          and respond to button press events from MANUAL_CONTROL MAVLink messages.
 *          
 *          The class provides two complementary interfaces for button state access:
 *          - State query: Check if button is currently pressed
 *          - Event counting: Track number of button press events
 *          
 *          This allows scripts to implement both state-based and event-based button handling.
 * 
 * @note This file is only compiled when AP_SCRIPTING_ENABLED is defined
 * @see AP_Scripting for Lua scripting framework integration
 * 
 * Source: ArduSub/script_button.h:1-35
 */

#pragma once

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>

/**
 * @class ScriptButton
 * @brief Joystick button object for use in Lua scripts
 * 
 * @details Provides a simple interface for Lua scripts to monitor joystick button state
 *          and count button press events. The button state is updated from MANUAL_CONTROL
 *          MAVLink messages received from the ground control station.
 *          
 *          This class supports two usage patterns:
 *          
 *          1. **State-based monitoring**: Scripts can check if a button is currently pressed
 *             using is_pressed(). This reflects the button state from the most recent
 *             MANUAL_CONTROL message.
 *          
 *          2. **Event counting**: Scripts can count button press events using get_count()
 *             or get_and_clear_count(). Each transition from released to pressed increments
 *             the counter. This is useful for implementing toggle or multi-click behaviors.
 *          
 *          Thread Safety: This class is accessed from both the main thread (updating from
 *          MAVLink messages) and the scripting thread. The simple data types (bool, uint8_t)
 *          ensure atomic access on supported platforms.
 * 
 * @note Button state updates are rate-limited by MANUAL_CONTROL message frequency from GCS
 * @note Counter is uint8_t, so it wraps at 255 press events
 * @see GCS_MAVLINK for MANUAL_CONTROL message handling
 */
class ScriptButton {
public:
    /**
     * @brief Construct a new ScriptButton object
     * 
     * @details Initializes button in released state with zero press count
     */
    ScriptButton(): pressed(false), count(0) {}

    /**
     * @brief Mark button as pressed and increment press counter
     * 
     * @details Called when a button press event is detected from MANUAL_CONTROL message.
     *          This method transitions the button to pressed state and increments the
     *          event counter. If already pressed, the counter is not incremented again
     *          until after a release() call.
     * 
     * @note Increments counter only on transition from released to pressed
     */
    void press();

    /**
     * @brief Mark button as released
     * 
     * @details Called when a button release event is detected from MANUAL_CONTROL message.
     *          This method transitions the button to released state, allowing the next
     *          press() call to increment the counter.
     */
    void release();

    /**
     * @brief Check if button is currently pressed
     * 
     * @details Returns the current button state as of the most recent MANUAL_CONTROL
     *          message. This is useful for scripts that need to perform actions while
     *          a button is held down.
     * 
     * @return true if button is currently pressed
     * @return false if button is currently released
     * 
     * @note State reflects most recent MANUAL_CONTROL message, not real-time button state
     */
    bool is_pressed() const WARN_IF_UNUSED;

    /**
     * @brief Get number of button press events
     * 
     * @details Returns the count of button press events (transitions from released to
     *          pressed) since the last clear_count() or get_and_clear_count() call.
     *          The counter is not reset by this call.
     * 
     * @return uint8_t Number of press events (0-255, wraps on overflow)
     * 
     * @note Counter persists until explicitly cleared
     * @see get_and_clear_count() for atomic get-and-reset operation
     */
    uint8_t get_count() const WARN_IF_UNUSED;

    /**
     * @brief Clear the button press event counter
     * 
     * @details Resets the press event counter to zero. Typically called after processing
     *          button press events to start counting new events.
     */
    void clear_count();

    /**
     * @brief Get press count and reset counter atomically
     * 
     * @details Returns the current press event count and immediately resets the counter
     *          to zero. This is the recommended way for scripts to consume button press
     *          events, ensuring no events are missed or double-counted.
     *          
     *          Common usage pattern in Lua:
     *          - Call periodically from script update function
     *          - If count > 0, process the button press events
     *          - Counter automatically resets for next update cycle
     * 
     * @return uint8_t Number of press events since last call (0-255)
     * 
     * @note This is the preferred method for event-based button handling in scripts
     */
    uint8_t get_and_clear_count();

private:
    /**
     * @brief Current button state
     * 
     * @details true if button currently pressed, false if released.
     *          Updated by press() and release() methods from MANUAL_CONTROL messages.
     */
    bool pressed;
    
    /**
     * @brief Press event counter
     * 
     * @details Counts button press events (transitions from released to pressed).
     *          Incremented by press() method, reset by clear_count() or get_and_clear_count().
     *          Wraps at 255 due to uint8_t type.
     */
    uint8_t count;
};

#endif // AP_SCRIPTING_ENABLED
