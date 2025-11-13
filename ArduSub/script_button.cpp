/**
 * @file script_button.cpp
 * @brief Lua scripting button interface for custom control in ArduSub
 * 
 * @details This file implements the ScriptButton class, which provides a bridge between
 *          joystick button inputs and Lua scripting for custom control functionality.
 *          Lua scripts can access button state and press events to implement custom
 *          behaviors triggered by joystick buttons in MANUAL_CONTROL mode.
 *          
 *          The implementation provides two primary usage patterns:
 *          1. State checking: Scripts can poll if a button is currently pressed
 *          2. Event counting: Scripts can count button press events between polling intervals
 *          
 *          This enables custom control logic without modifying core ArduSub code,
 *          allowing operators to implement specialized behaviors like triggering
 *          scientific instruments, custom lighting sequences, or payload operations.
 * 
 * @note This functionality is only available when AP_SCRIPTING_ENABLED is defined
 * @note Button state is updated from MANUAL_CONTROL MAVLink messages
 * @warning Button counts are capped at 255 to prevent rollover issues in scripts
 * 
 * @see AP_Scripting for the Lua scripting framework integration
 * @see script_button.h for the ScriptButton class definition
 */

#include "AP_Scripting/AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <limits>
#include "script_button.h"

/**
 * @brief Register a button press event
 * 
 * @details This method is called when a button press is detected in the MANUAL_CONTROL
 *          message. It updates the pressed state and increments the press count for
 *          event-based script handling.
 *          
 *          The method implements press deduplication - repeated calls while the button
 *          is already pressed will not increment the count. This ensures accurate event
 *          counting even if MANUAL_CONTROL messages arrive at high frequency.
 *          
 *          The press counter saturates at 255 (uint8_t max) to prevent rollover,
 *          ensuring scripts don't receive incorrect count values. This is a safety
 *          feature to maintain predictable behavior in long-running missions.
 * 
 * @note This method is typically called by the joystick input handler, not directly by scripts
 * @note Multiple press() calls without an intervening release() only count as one press event
 * @warning Count saturates at 255 and will not increment further until cleared
 * 
 * @see release() for resetting the pressed state
 * @see get_count() for retrieving the current press count
 */
void ScriptButton::press()
{
    if (!pressed) {
        pressed = true;

        // The count will max out at 255, but it won't roll over to 0.
        if (count < std::numeric_limits<uint8_t>::max()) {
            count++;
        }
    }
}

/**
 * @brief Register a button release event
 * 
 * @details This method is called when a button release is detected in the MANUAL_CONTROL
 *          message. It clears the pressed state, allowing subsequent press() calls to
 *          increment the press count again.
 *          
 *          The release event resets only the pressed flag, not the press count. This
 *          allows scripts to accumulate multiple press events and retrieve them all
 *          at once using get_and_clear_count().
 * 
 * @note This method is typically called by the joystick input handler, not directly by scripts
 * @note The press count is NOT cleared by this method - use clear_count() or get_and_clear_count()
 * 
 * @see press() for registering press events
 * @see is_pressed() for checking current button state
 */
void ScriptButton::release()
{
    pressed = false;
}

/**
 * @brief Check if the button is currently pressed
 * 
 * @details Returns the current state of the button based on the most recent MANUAL_CONTROL
 *          message. This provides state-based button handling for Lua scripts that need
 *          to respond to buttons being held down rather than press events.
 *          
 *          Typical use case: Implementing "hold button to activate" behaviors, such as
 *          holding a button to continuously adjust a setting or maintain a mode.
 * 
 * @return true if button is currently pressed, false if released
 * 
 * @note This reflects the state at the time of the last MANUAL_CONTROL message update
 * @note For event-based handling, use get_and_clear_count() instead
 * 
 * @see press() for how the pressed state is set
 * @see get_and_clear_count() for event-counting approach
 */
bool ScriptButton::is_pressed() const
{
    return pressed;
}

/**
 * @brief Get the current press event count without clearing it
 * 
 * @details Returns the number of button press events that have occurred since the last
 *          clear_count() or get_and_clear_count() call. This allows scripts to check
 *          the count without consuming it, enabling multiple scripts or checks to see
 *          the same count value.
 *          
 *          The count accumulates each press event (transition from released to pressed)
 *          and saturates at 255 to prevent rollover.
 * 
 * @return Number of press events (0-255), where 255 indicates saturation
 * 
 * @note The count persists until explicitly cleared with clear_count() or get_and_clear_count()
 * @note A return value of 255 may indicate either exactly 255 presses or 255+ presses (saturated)
 * 
 * @see get_and_clear_count() for atomic read-and-clear operation
 * @see clear_count() for clearing without reading
 */
uint8_t ScriptButton::get_count() const
{
    return count;
}

/**
 * @brief Clear the press event count
 * 
 * @details Resets the press event counter to zero, discarding any accumulated press events.
 *          This is useful when a script has processed events through get_count() and wants
 *          to reset the counter for the next polling interval.
 *          
 *          Typical use case: A script checks get_count() to see if events occurred, performs
 *          some action based on the count, then calls clear_count() to start fresh for the
 *          next iteration.
 * 
 * @note This does NOT affect the pressed state - the button may still be pressed after clearing count
 * @note For atomic read-and-clear, use get_and_clear_count() instead
 * 
 * @see get_count() for reading the count
 * @see get_and_clear_count() for combined read and clear operation
 */
void ScriptButton::clear_count()
{
    count = 0;
}

/**
 * @brief Get the press event count and clear it atomically
 * 
 * @details This is the primary method for event-based button handling in Lua scripts.
 *          It atomically reads the current press count and resets it to zero, ensuring
 *          no press events are lost or double-counted between operations.
 *          
 *          Typical usage pattern in Lua scripts:
 *          ```lua
 *          function update()
 *              local presses = button:get_and_clear_count()
 *              if presses > 0 then
 *                  -- Handle button press event(s)
 *                  gcs:send_text(0, string.format("Button pressed %d times", presses))
 *              end
 *          end
 *          ```
 *          
 *          This pattern ensures reliable event counting even if the script's update rate
 *          is slower than the button press rate - all presses are counted and delivered
 *          as a batch in the next script iteration.
 * 
 * @return Number of press events since last clear (0-255)
 * 
 * @note This is the recommended method for Lua script button event handling
 * @note The count is guaranteed to be cleared after this call
 * @note Scripts calling this at regular intervals will never miss press events
 * 
 * @see get_count() for reading without clearing
 * @see is_pressed() for state-based button handling
 */
uint8_t ScriptButton::get_and_clear_count()
{
    auto result = get_count();
    clear_count();
    return result;
}

#endif // AP_SCRIPTING_ENABLED
