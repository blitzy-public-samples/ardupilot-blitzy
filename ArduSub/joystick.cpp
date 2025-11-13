/**
 * @file joystick.cpp
 * @brief Joystick and gamepad input processing for ArduSub ROV control
 * 
 * @details This file handles the conversion of joystick/gamepad inputs from MAVLink
 *          MANUAL_CONTROL messages to RC channel overrides for 6-DOF underwater vehicle control.
 *          
 *          The joystick system supports:
 *          - 32 configurable buttons (16 standard + 16 extended via MANUAL_CONTROL extensions)
 *          - 6 axes: roll, pitch, yaw, throttle (vertical), forward, lateral
 *          - Multiple button functions: arming, mode switching, camera/mount control,
 *            lights adjustment, gain tuning, trim control, relay/servo control
 *          - Shift modifier for dual-function buttons
 *          - Input hold feature for maintaining position
 *          - Configurable gain scaling for sensitivity adjustment
 *          
 *          Button states are transmitted as two 16-bit bitmasks (buttons and buttons2)
 *          which are merged into a single 32-bit value for processing. Each bit represents
 *          the state of one button (1=pressed, 0=released).
 *          
 *          Axis values are converted from joystick coordinates (-1000 to 1000 for most axes,
 *          0 to 1000 for throttle) to RC PWM values (typically 1100-1900 μs) with gain
 *          scaling applied.
 * 
 * @note This is called from GCS_MAVLINK when MANUAL_CONTROL messages are received
 * @warning Joystick control bypasses RC receiver input when active
 * 
 * Source: ArduSub/joystick.cpp:1-end
 */

#include "Sub.h"
#include "mode.h"

// Functions that will handle joystick/gamepad input
// ----------------------------------------------------------------------------

// Anonymous namespace to hold variables used only in this file
namespace {
float cam_tilt = 1500.0;
float cam_pan = 1500.0;
float lights1 = 0;
float lights2 = 0;
int16_t rollTrim = 0;
int16_t pitchTrim = 0;
int16_t zTrim = 0;
int16_t xTrim = 0;
int16_t yTrim = 0;
int16_t x_last, y_last, z_last;
uint32_t buttons_prev;

bool controls_reset_since_input_hold = true;
}

/**
 * @brief Initialize joystick control system for ArduSub
 * 
 * @details Performs initial setup of the joystick/gamepad control system:
 *          1. Sets default button mappings for standard ROV control
 *          2. Initializes vehicle to MANUAL mode
 *          3. Configures gain settings based on parameters
 *          4. Initializes lights and video switch outputs to off state
 *          
 *          The gain system allows stepped sensitivity adjustment with configurable
 *          min/max values and number of steps. If only one gain setting is configured,
 *          the default gain value is used. Otherwise, gain is initialized to the
 *          middle step value.
 * 
 * @note Called during vehicle initialization
 * @see default_js_buttons(), transform_manual_control_to_rc_override()
 * 
 * Source: ArduSub/joystick.cpp:24-45
 */
void Sub::init_joystick()
{
    default_js_buttons();

    set_mode(Mode::Number::MANUAL, ModeReason::RC_COMMAND); // Initialize flight mode

    if (g.numGainSettings < 1) {
        g.numGainSettings.set_and_save(1);
    }

    if (g.numGainSettings == 1 || (g.gain_default < g.maxGain + 0.01 && g.gain_default > g.minGain - 0.01)) {
        gain = constrain_float(g.gain_default, g.minGain, g.maxGain); // Use default gain parameter
    } else {
        // Use setting closest to average of minGain and maxGain
        gain = g.minGain + (g.numGainSettings/2 - 1) * (g.maxGain - g.minGain) / (g.numGainSettings - 1);
    }

    gain = constrain_float(gain, 0.1, 1.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, 0.0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_video_switch, 0.0);
}

/**
 * @brief Transform MAVLink MANUAL_CONTROL message to RC channel overrides
 * 
 * @details Converts joystick/gamepad input from MAVLink MANUAL_CONTROL messages into
 *          RC channel override values for 6-DOF underwater vehicle control. This function
 *          handles axis scaling with gain adjustment, button state processing with shift
 *          modifier support, and trim/hold functionality.
 *          
 *          Axis Mapping (ROV 6-DOF control):
 *          - s (pitch axis) → RC channel 0 (pitch)
 *          - t (roll axis) → RC channel 1 (roll)
 *          - z (throttle axis) → RC channel 2 (throttle/vertical)
 *          - r (yaw axis) → RC channel 3 (yaw)
 *          - x (forward axis) → RC channel 4 (forward movement)
 *          - y (lateral axis) → RC channel 5 (lateral movement)
 *          - Camera pan → RC channel 6
 *          - Camera tilt → RC channel 7
 *          
 *          Button Processing:
 *          - Supports 32 buttons via two 16-bit masks (buttons, buttons2)
 *          - Buttons are merged: all_buttons = buttons | (buttons2 << 16)
 *          - Each bit represents one button state (1=pressed, 0=released)
 *          - Detects press, hold, and release events
 *          - Shift modifier allows dual functions per button
 *          
 *          Scaling and Gains:
 *          - RPY axes scaled by 0.4 * gain (maps ±1000 to ±400 * gain)
 *          - Throttle scaled by 0.8 * gain * throttle_gain parameter
 *          - Final values constrained to 1100-1900 μs PWM range
 *          - Gain adjustable 0.1-1.0 via button controls
 *          
 *          Input Hold Feature:
 *          - When active, maintains last commanded forward/lateral/vertical input
 *          - Prevents control until operator returns sticks to neutral
 *          - Useful for maintaining position during other operations
 *          
 *          Attitude vs Movement Mode:
 *          - roll_pitch_flag=0: Standard ROV mode (x/y control forward/lateral)
 *          - roll_pitch_flag=1: Attitude mode (x/y adjust roll/pitch trim)
 * 
 * @param[in] x Forward/backward axis input (-1000 to 1000)
 * @param[in] y Lateral/strafe axis input (-1000 to 1000)
 * @param[in] z Throttle/vertical axis input (0 to 1000, 500=neutral)
 * @param[in] r Yaw/rotation axis input (-1000 to 1000)
 * @param[in] buttons Button state bitmask for buttons 0-15 (bit set = pressed)
 * @param[in] buttons2 Button state bitmask for buttons 16-31 (bit set = pressed)
 * @param[in] enabled_extensions MAVLink extension flags (currently unused)
 * @param[in] s Pitch axis input (-1000 to 1000)
 * @param[in] t Roll axis input (-1000 to 1000)
 * @param[in] aux1 Auxiliary axis 1 (currently unused)
 * @param[in] aux2 Auxiliary axis 2 (currently unused)
 * @param[in] aux3 Auxiliary axis 3 (currently unused)
 * @param[in] aux4 Auxiliary axis 4 (currently unused)
 * @param[in] aux5 Auxiliary axis 5 (currently unused)
 * @param[in] aux6 Auxiliary axis 6 (currently unused)
 * 
 * @note Called from GCS_MAVLINK::handle_manual_control() when MANUAL_CONTROL messages received
 * @note Typical update rate: 25-50 Hz depending on ground control station
 * @warning RC channel overrides bypass RC receiver input completely
 * 
 * @see handle_jsbutton_press(), handle_jsbutton_release(), RC_Channels::set_override()
 * 
 * Source: ArduSub/joystick.cpp:47-139
 */
void Sub::transform_manual_control_to_rc_override(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons, uint16_t buttons2, uint8_t enabled_extensions,
            int16_t s,
            int16_t t,
            int16_t aux1,
            int16_t aux2,
            int16_t aux3,
            int16_t aux4,
            int16_t aux5,
            int16_t aux6)
{

    float rpyScale = 0.4*gain; // Scale -1000-1000 to -400-400 with gain
    float throttleScale = 0.8*gain*g.throttle_gain; // Scale 0-1000 to 0-800 times gain
    int16_t rpyCenter = 1500;
    int16_t throttleBase = 1500-500*throttleScale;

    bool shift = false;

    // Neutralize camera tilt and pan speed setpoint
    cam_tilt = 1500;
    cam_pan = 1500;

    // Merge two 16-bit button masks into single 32-bit value
    // buttons contains bits 0-15, buttons2 contains bits 16-31
    uint32_t all_buttons = buttons | (buttons2 << 16);
    
    // First pass: Detect if any shift button is pressed
    // Shift modifier allows buttons to have alternate functions
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((all_buttons & (1 << i)) && get_button(i)->function() == JSButton::button_function_t::k_shift) {
            shift = true;
        }
    }

    // Second pass: Process all 32 button states
    // Detects press events (button newly pressed), hold events (button still pressed),
    // and release events (button newly released)
    // Only act upon pressing button and ignore holding. This provides compatibility with Taranis as joystick.
    for (uint8_t i = 0 ; i < 32 ; i++) {
        if ((all_buttons & (1 << i))) {
            // Button is currently pressed
            // Pass held flag (true if button was also pressed last cycle)
            handle_jsbutton_press(i,shift,(buttons_prev & (1 << i)));
            // buttonDebounce = tnow_ms;
        } else if (buttons_prev & (1 << i)) {
            // Button was pressed last cycle but not this cycle = release event
            handle_jsbutton_release(i, shift);
        }
    }

    // Store current button state for next cycle's edge detection
    buttons_prev = all_buttons;

    // attitude mode:
    if (roll_pitch_flag == 1) {
    // adjust roll/pitch trim with joystick input instead of forward/lateral
        pitchTrim = -x * rpyScale;
        rollTrim  =  y * rpyScale;
    }

    uint32_t tnow = AP_HAL::millis();

    int16_t zTot;
    int16_t yTot;
    int16_t xTot;

    if (!controls_reset_since_input_hold) {
        zTot = zTrim + 500; // 500 is neutral for throttle
        yTot = yTrim;
        xTot = xTrim;
        // if all 3 axes return to neutral, than we're ready to accept input again
        controls_reset_since_input_hold = (abs(z - 500) < 50) && (abs(y) < 50) && (abs(x) < 50);
    } else {
        zTot = z + zTrim;
        yTot = y + yTrim;
        xTot = x + xTrim;
    }

    RC_Channels::set_override(0, constrain_int16(s + pitchTrim + rpyCenter,1100,1900), tnow); // pitch
    RC_Channels::set_override(1, constrain_int16(t + rollTrim  + rpyCenter,1100,1900), tnow); // roll

    RC_Channels::set_override(2, constrain_int16((zTot)*throttleScale+throttleBase,1100,1900), tnow); // throttle
    RC_Channels::set_override(3, constrain_int16(r*rpyScale+rpyCenter,1100,1900), tnow);                 // yaw

    // maneuver mode:
    if (roll_pitch_flag == 0) {
        // adjust forward and lateral with joystick input instead of roll and pitch
        RC_Channels::set_override(4, constrain_int16((xTot)*rpyScale+rpyCenter,1100,1900), tnow); // forward for ROV
        RC_Channels::set_override(5, constrain_int16((yTot)*rpyScale+rpyCenter,1100,1900), tnow); // lateral for ROV
    } else {
        // neutralize forward and lateral input while we are adjusting roll and pitch
        RC_Channels::set_override(4, constrain_int16(xTrim*rpyScale+rpyCenter,1100,1900), tnow); // forward for ROV
        RC_Channels::set_override(5, constrain_int16(yTrim*rpyScale+rpyCenter,1100,1900), tnow); // lateral for ROV
    }

    RC_Channels::set_override(6, cam_pan, tnow);       // camera pan
    RC_Channels::set_override(7, cam_tilt, tnow);      // camera tilt

    // Store old x, y, z values for use in input hold logic
    x_last = x;
    y_last = y;
    z_last = z;
}

/**
 * @brief Handle joystick button press and hold events
 * 
 * @details Processes button press events based on the configured function for each button.
 *          Supports 32 buttons (0-31) with configurable functions including:
 *          
 *          Arming Functions:
 *          - k_arm_toggle, k_arm, k_disarm: Vehicle arming control
 *          
 *          Mode Selection (8 modes):
 *          - k_mode_manual, k_mode_stabilize, k_mode_depth_hold, k_mode_auto
 *          - k_mode_guided, k_mode_circle, k_mode_acro, k_mode_poshold
 *          - k_mode_surftrak (if rangefinder enabled)
 *          
 *          Camera/Mount Control:
 *          - k_mount_center, k_mount_tilt_up, k_mount_tilt_down
 *          - k_mount_pan_left, k_mount_pan_right
 *          - k_camera_trigger, k_camera_source_toggle
 *          
 *          Lights Control:
 *          - k_lights1_cycle, k_lights1_brighter, k_lights1_dimmer
 *          - k_lights2_cycle, k_lights2_brighter, k_lights2_dimmer
 *          
 *          Gain Adjustment:
 *          - k_gain_toggle (50% / 100%), k_gain_inc, k_gain_dec
 *          
 *          Trim Control:
 *          - k_trim_roll_inc, k_trim_roll_dec, k_trim_pitch_inc, k_trim_pitch_dec
 *          
 *          Input Hold:
 *          - k_input_hold_set: Captures current stick positions as trim offsets
 *          
 *          Relay Control (if AP_RELAY_ENABLED):
 *          - k_relay_1/2/3/4_on, k_relay_1/2/3/4_off
 *          - k_relay_1/2/3/4_toggle, k_relay_1/2/3/4_momentary
 *          
 *          Servo/Actuator Control (if AP_SERVORELAYEVENTS_ENABLED):
 *          - k_servo_1-6_inc, k_servo_1-6_dec, k_servo_1-6_center
 *          - k_servo_1-6_min, k_servo_1-6_max
 *          - k_servo_1-6_min_toggle, k_servo_1-6_max_toggle
 *          - k_servo_1-6_min_momentary, k_servo_1-6_max_momentary
 *          
 *          Mode Toggle:
 *          - k_roll_pitch_toggle: Switch between movement and attitude control
 *          
 *          Custom Functions:
 *          - k_custom_1 through k_custom_6: Reserved for future use
 *          
 *          Scripting Functions (if AP_SCRIPTING_ENABLED):
 *          - k_script_1 through k_script_4: Trigger Lua script button events
 *          
 *          The 'held' parameter prevents repeated actions on buttons that should only
 *          trigger once per press (toggles, increments). Some functions like momentary
 *          relays or continuous camera movement are active while held.
 * 
 * @param[in] _button Button index (0-31)
 * @param[in] shift True if shift modifier is active (enables alternate button functions)
 * @param[in] held True if button was already pressed in previous cycle (held down)
 * 
 * @note Most toggle/increment functions check !held to prevent repeated triggering
 * @note Camera tilt/pan values set here are applied to RC channels in parent function
 * @warning Mode changes while armed should be done carefully to avoid loss of control
 * 
 * @see handle_jsbutton_release(), get_button(), JSButton::function()
 * 
 * Source: ArduSub/joystick.cpp:141-637
 */
void Sub::handle_jsbutton_press(uint8_t _button, bool shift, bool held)
{
    // Act based on the function assigned to this button
    switch (get_button(_button)->function(shift)) {
    case JSButton::button_function_t::k_arm_toggle:
        if (motors.armed()) {
            arming.disarm(AP_Arming::Method::MAVLINK);
        } else {
            arming.arm(AP_Arming::Method::MAVLINK);
        }
        break;
    case JSButton::button_function_t::k_arm:
        arming.arm(AP_Arming::Method::MAVLINK);
        break;
    case JSButton::button_function_t::k_disarm:
        arming.disarm(AP_Arming::Method::MAVLINK);
        break;

    case JSButton::button_function_t::k_mode_manual:
        set_mode(Mode::Number::MANUAL, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_stabilize:
        set_mode(Mode::Number::STABILIZE, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_depth_hold:
        set_mode(Mode::Number::ALT_HOLD, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_auto:
        set_mode(Mode::Number::AUTO, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_guided:
        set_mode(Mode::Number::GUIDED, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_circle:
        set_mode(Mode::Number::CIRCLE, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_acro:
        set_mode(Mode::Number::ACRO, ModeReason::RC_COMMAND);
        break;
    case JSButton::button_function_t::k_mode_poshold:
        set_mode(Mode::Number::POSHOLD, ModeReason::RC_COMMAND);
        break;
#if AP_RANGEFINDER_ENABLED
    case JSButton::button_function_t::k_mode_surftrak:
        set_mode(Mode::Number::SURFTRAK, ModeReason::RC_COMMAND);
        break;
#endif

    case JSButton::button_function_t::k_mount_center:
#if HAL_MOUNT_ENABLED
        camera_mount.set_angle_target(0, 0, 0, false);
        // for some reason the call to set_angle_targets changes the mode to mavlink targeting!
        camera_mount.set_mode(MAV_MOUNT_MODE_RC_TARGETING);
#endif
        break;
    case JSButton::button_function_t::k_mount_tilt_up:
        cam_tilt = 1900;
        break;
    case JSButton::button_function_t::k_mount_tilt_down:
        cam_tilt = 1100;
        break;
    case JSButton::button_function_t::k_camera_trigger:
        break;
    case JSButton::button_function_t::k_camera_source_toggle:
        if (!held) {
            static bool video_toggle = false;
            video_toggle = !video_toggle;
            if (video_toggle) {
                SRV_Channels::set_output_scaled(SRV_Channel::k_video_switch, 1000);
                gcs().send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 2");
            } else {
                SRV_Channels::set_output_scaled(SRV_Channel::k_video_switch, 0.0);
                gcs().send_text(MAV_SEVERITY_INFO,"Video Toggle: Source 1");
            }
        }
        break;
    case JSButton::button_function_t::k_mount_pan_right:
        cam_pan = 1900;
        break;
    case JSButton::button_function_t::k_mount_pan_left:
        cam_pan = 1100;
        break;
    case JSButton::button_function_t::k_lights1_cycle:
        if (!held) {
            static bool increasing = true;
            uint16_t step = 1000.0 / g.lights_steps;
            if (increasing) {
                lights1 = constrain_float(lights1 + step, 0.0, 1000.0);
            } else {
                lights1 = constrain_float(lights1 - step, 0.0, 1000.0);
            }
            if (lights1 >= 1000.0 || lights1 <= 0.0) {
                increasing = !increasing;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, lights1);
        }
        break;
    case JSButton::button_function_t::k_lights1_brighter:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights1 = constrain_float(lights1 + step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, lights1);
        }
        break;
    case JSButton::button_function_t::k_lights1_dimmer:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights1 = constrain_float(lights1 - step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights1, lights1);
        }
        break;
    case JSButton::button_function_t::k_lights2_cycle:
       if (!held) {
            static bool increasing = true;
            uint16_t step = 1000.0 / g.lights_steps;
            if (increasing) {
                lights2 = constrain_float(lights2 + step, 0.0, 1000.0);
            } else {
                lights2 = constrain_float(lights2 - step, 0.0, 1000.0);
            }
            if (lights2 >= 1000.0 || lights2 <= 0.0) {
                increasing = !increasing;
            }
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, lights2);
        }
        break;
    case JSButton::button_function_t::k_lights2_brighter:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights2 = constrain_float(lights2 + step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, lights2);
        }
        break;
    case JSButton::button_function_t::k_lights2_dimmer:
        if (!held) {
            uint16_t step = 1000.0 / g.lights_steps;
            lights2 = constrain_float(lights2 - step, 0.0, 1000.0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_lights2, lights2);
        }
        break;
    case JSButton::button_function_t::k_gain_toggle:
        if (!held) {
            static bool lowGain = false;
            lowGain = !lowGain;
            if (lowGain) {
                gain = 0.5f;
            } else {
                gain = 1.0f;
            }
            gcs().send_text(MAV_SEVERITY_INFO,"#Gain: %2.0f%%",(double)gain*100);
        }
        break;
    case JSButton::button_function_t::k_gain_inc:
        if (!held) {
            // check that our gain parameters are in correct range, update in eeprom and notify gcs if needed
            g.minGain.set_and_save(constrain_float(g.minGain, 0.10, 0.80));
            g.maxGain.set_and_save(constrain_float(g.maxGain, g.minGain, 1.0));
            g.numGainSettings.set_and_save(constrain_int16(g.numGainSettings, 1, 10));

            if (g.numGainSettings == 1) {
                gain = constrain_float(g.gain_default, g.minGain, g.maxGain);
            } else {
                gain = constrain_float(gain + (g.maxGain-g.minGain)/(g.numGainSettings-1), g.minGain, g.maxGain);
            }

            gcs().send_text(MAV_SEVERITY_INFO,"#Gain is %2.0f%%",(double)gain*100);
        }
        break;
    case JSButton::button_function_t::k_gain_dec:
        if (!held) {
            // check that our gain parameters are in correct range, update in eeprom and notify gcs if needed
            g.minGain.set_and_save(constrain_float(g.minGain, 0.10, 0.80));
            g.maxGain.set_and_save(constrain_float(g.maxGain, g.minGain, 1.0));
            g.numGainSettings.set_and_save(constrain_int16(g.numGainSettings, 1, 10));

            if (g.numGainSettings == 1) {
                gain = constrain_float(g.gain_default, g.minGain, g.maxGain);
            } else {
                gain = constrain_float(gain - (g.maxGain-g.minGain)/(g.numGainSettings-1), g.minGain, g.maxGain);
            }

            gcs().send_text(MAV_SEVERITY_INFO,"#Gain is %2.0f%%",(double)gain*100);
        }
        break;
    case JSButton::button_function_t::k_trim_roll_inc:
        rollTrim = constrain_float(rollTrim+10,-200,200);
        break;
    case JSButton::button_function_t::k_trim_roll_dec:
        rollTrim = constrain_float(rollTrim-10,-200,200);
        break;
    case JSButton::button_function_t::k_trim_pitch_inc:
        pitchTrim = constrain_float(pitchTrim+10,-200,200);
        break;
    case JSButton::button_function_t::k_trim_pitch_dec:
        pitchTrim = constrain_float(pitchTrim-10,-200,200);
        break;
    case JSButton::button_function_t::k_input_hold_set:
        if(!motors.armed()) {
            break;
        }
        if (!held) {
            zTrim = abs(z_last-500) > 50 ? z_last-500 : 0;
            xTrim = abs(x_last) > 50 ? x_last : 0;
            yTrim = abs(y_last) > 50 ? y_last : 0;
            bool input_hold_engaged_last = input_hold_engaged;
            input_hold_engaged = zTrim || xTrim || yTrim;
            if (input_hold_engaged) {
                gcs().send_text(MAV_SEVERITY_INFO,"#Input Hold Set");
            } else if (input_hold_engaged_last) {
                gcs().send_text(MAV_SEVERITY_INFO,"#Input Hold Disabled");
            }
            controls_reset_since_input_hold = !input_hold_engaged;
        }
        break;
#if AP_RELAY_ENABLED
    case JSButton::button_function_t::k_relay_1_on:
        relay.on(0);
        break;
    case JSButton::button_function_t::k_relay_1_off:
        relay.off(0);
        break;
    case JSButton::button_function_t::k_relay_1_toggle:
        if (!held) {
            relay.toggle(0);
        }
        break;
    case JSButton::button_function_t::k_relay_1_momentary:
        if (!held) {
            relay.on(0);
        }
        break;
    case JSButton::button_function_t::k_relay_2_on:
        relay.on(1);
        break;
    case JSButton::button_function_t::k_relay_2_off:
        relay.off(1);
        break;
    case JSButton::button_function_t::k_relay_2_toggle:
        if (!held) {
            relay.toggle(1);
        }
        break;
    case JSButton::button_function_t::k_relay_2_momentary:
        if (!held) {
            relay.on(1);
        }
        break;
    case JSButton::button_function_t::k_relay_3_on:
        relay.on(2);
        break;
    case JSButton::button_function_t::k_relay_3_off:
        relay.off(2);
        break;
    case JSButton::button_function_t::k_relay_3_toggle:
        if (!held) {
            relay.toggle(2);
        }
        break;
    case JSButton::button_function_t::k_relay_3_momentary:
        if (!held) {
            relay.on(2);
        }
        break;
    case JSButton::button_function_t::k_relay_4_on:
        relay.on(3);
        break;
    case JSButton::button_function_t::k_relay_4_off:
        relay.off(3);
        break;
    case JSButton::button_function_t::k_relay_4_toggle:
        if (!held) {
            relay.toggle(3);
        }
        break;
    case JSButton::button_function_t::k_relay_4_momentary:
        if (!held) {
            relay.on(3);
        }
        break;
#endif

    ////////////////////////////////////////////////
    // Servo functions
#if AP_SERVORELAYEVENTS_ENABLED
    case JSButton::button_function_t::k_servo_1_inc:
        sub.g2.actuators.increase_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_inc:
        sub.g2.actuators.increase_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_inc:
        sub.g2.actuators.increase_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_inc:
        sub.g2.actuators.increase_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_inc:
        sub.g2.actuators.increase_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_inc:
        sub.g2.actuators.increase_actuator(5);
        break;

    case JSButton::button_function_t::k_servo_1_dec:
        sub.g2.actuators.decrease_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_dec:
        sub.g2.actuators.decrease_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_dec:
        sub.g2.actuators.decrease_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_dec:
        sub.g2.actuators.decrease_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_dec:
        sub.g2.actuators.decrease_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_dec:
        sub.g2.actuators.decrease_actuator(5);
        break;

    case JSButton::button_function_t::k_servo_1_min:
    case JSButton::button_function_t::k_servo_1_min_momentary:
        sub.g2.actuators.min_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_min:
    case JSButton::button_function_t::k_servo_2_min_momentary:
        sub.g2.actuators.min_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_min:
    case JSButton::button_function_t::k_servo_3_min_momentary:
        sub.g2.actuators.min_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_min:
    case JSButton::button_function_t::k_servo_4_min_momentary:
        sub.g2.actuators.min_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_min:
    case JSButton::button_function_t::k_servo_5_min_momentary:
        sub.g2.actuators.min_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_min:
    case JSButton::button_function_t::k_servo_6_min_momentary:
        sub.g2.actuators.min_actuator(5);
        break;

    case JSButton::button_function_t::k_servo_1_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(0);
        }
        break;
    case JSButton::button_function_t::k_servo_2_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(1);
        }
        break;
    case JSButton::button_function_t::k_servo_3_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(2);
        }
        break;
    case JSButton::button_function_t::k_servo_4_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(3);
        }
        break;
    case JSButton::button_function_t::k_servo_5_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(4);
        }
        break;
    case JSButton::button_function_t::k_servo_6_min_toggle:
        if (!held) {
            sub.g2.actuators.min_toggle_actuator(5);
        }
        break;

    case JSButton::button_function_t::k_servo_1_max:
    case JSButton::button_function_t::k_servo_1_max_momentary:
        sub.g2.actuators.max_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_max:
    case JSButton::button_function_t::k_servo_2_max_momentary:
        sub.g2.actuators.max_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_max:
    case JSButton::button_function_t::k_servo_3_max_momentary:
        sub.g2.actuators.max_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_max:
    case JSButton::button_function_t::k_servo_4_max_momentary:
        sub.g2.actuators.max_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_max:
    case JSButton::button_function_t::k_servo_5_max_momentary:
        sub.g2.actuators.max_actuator(4);
        break;

    case JSButton::button_function_t::k_servo_1_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(0);
        }
        break;
    case JSButton::button_function_t::k_servo_2_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(1);
        }
        break;
    case JSButton::button_function_t::k_servo_3_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(2);
        }
        break;
    case JSButton::button_function_t::k_servo_4_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(3);
        }
        break;
    case JSButton::button_function_t::k_servo_5_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(4);
        }
        break;
    case JSButton::button_function_t::k_servo_6_max_toggle:
        if (!held) {
            sub.g2.actuators.max_toggle_actuator(5);
        }
        break;

    case JSButton::button_function_t::k_servo_1_center:
        sub.g2.actuators.center_actuator(0);
        break;
    case JSButton::button_function_t::k_servo_2_center:
        sub.g2.actuators.center_actuator(1);
        break;
    case JSButton::button_function_t::k_servo_3_center:
        sub.g2.actuators.center_actuator(2);
        break;
    case JSButton::button_function_t::k_servo_4_center:
        sub.g2.actuators.center_actuator(3);
        break;
    case JSButton::button_function_t::k_servo_5_center:
        sub.g2.actuators.center_actuator(4);
        break;
    case JSButton::button_function_t::k_servo_6_center:
        sub.g2.actuators.center_actuator(5);
        break;
#endif  // AP_SERVORELAYEVENTS_ENABLED

    case JSButton::button_function_t::k_roll_pitch_toggle:
        if (!held) {
            roll_pitch_flag = !roll_pitch_flag;
            if (roll_pitch_flag) {
                gcs().send_text(MAV_SEVERITY_INFO, "#Attitude Control");
            }
            else {
                gcs().send_text(MAV_SEVERITY_INFO, "#Movement Control");
            }
        }
        break;

    case JSButton::button_function_t::k_custom_1:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_2:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_3:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_4:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_5:
        // Not implemented
        break;
    case JSButton::button_function_t::k_custom_6:
        // Not implemented
        break;

#if AP_SCRIPTING_ENABLED
    case JSButton::button_function_t::k_script_1:
        sub.script_buttons[0].press();
        break;
    case JSButton::button_function_t::k_script_2:
        sub.script_buttons[1].press();
        break;
    case JSButton::button_function_t::k_script_3:
        sub.script_buttons[2].press();
        break;
    case JSButton::button_function_t::k_script_4:
        sub.script_buttons[3].press();
        break;
#endif // AP_SCRIPTING_ENABLED
    }
}

/**
 * @brief Handle joystick button release events
 * 
 * @details Processes button release events for functions that require action on release.
 *          Most button functions are handled entirely in handle_jsbutton_press(), but
 *          momentary functions need release handling to return to neutral state:
 *          
 *          Momentary Relay Functions (if AP_RELAY_ENABLED):
 *          - k_relay_1/2/3/4_momentary: Turns relay off when button released
 *          
 *          Momentary Servo Functions (if AP_SERVORELAYEVENTS_ENABLED):
 *          - k_servo_1-6_min_momentary: Returns servo to center when released
 *          - k_servo_1-6_max_momentary: Returns servo to center when released
 *          
 *          Scripting Functions (if AP_SCRIPTING_ENABLED):
 *          - k_script_1 through k_script_4: Notifies Lua scripts of button release
 *          
 *          This provides spring-loaded behavior where the control returns to neutral
 *          when the operator releases the button, useful for manipulator arms, grippers,
 *          or other actuators that should hold position only while button is pressed.
 * 
 * @param[in] _button Button index (0-31)
 * @param[in] shift True if shift modifier is active (enables alternate button functions)
 * 
 * @note Only processes button functions that explicitly require release handling
 * @note Most button functions are fully handled in handle_jsbutton_press()
 * 
 * @see handle_jsbutton_press(), get_button(), JSButton::function()
 * 
 * Source: ArduSub/joystick.cpp:639-711
 */
void Sub::handle_jsbutton_release(uint8_t _button, bool shift) {

    // Act based on the function assigned to this button
    switch (get_button(_button)->function(shift)) {
#if AP_RELAY_ENABLED
    case JSButton::button_function_t::k_relay_1_momentary:
        relay.off(0);
        break;
    case JSButton::button_function_t::k_relay_2_momentary:
        relay.off(1);
        break;
    case JSButton::button_function_t::k_relay_3_momentary:
        relay.off(2);
        break;
    case JSButton::button_function_t::k_relay_4_momentary:
        relay.off(3);
        break;
#endif
#if AP_SERVORELAYEVENTS_ENABLED
    case JSButton::button_function_t::k_servo_1_min_momentary:
    case JSButton::button_function_t::k_servo_1_max_momentary:
    {
        sub.g2.actuators.center_actuator(0);
    }
        break;
    case JSButton::button_function_t::k_servo_2_min_momentary:
    case JSButton::button_function_t::k_servo_2_max_momentary:
    {
        sub.g2.actuators.center_actuator(1);
    }
        break;
    case JSButton::button_function_t::k_servo_3_min_momentary:
    case JSButton::button_function_t::k_servo_3_max_momentary:
    {
        sub.g2.actuators.center_actuator(2);
    }
        break;
    case JSButton::button_function_t::k_servo_4_min_momentary:
    case JSButton::button_function_t::k_servo_4_max_momentary:
    {
        sub.g2.actuators.center_actuator(3);
    }
        break;
    case JSButton::button_function_t::k_servo_5_min_momentary:
    case JSButton::button_function_t::k_servo_5_max_momentary:
    {
        sub.g2.actuators.center_actuator(4);
    }
        break;
    case JSButton::button_function_t::k_servo_6_min_momentary:
    case JSButton::button_function_t::k_servo_6_max_momentary:
    {
        sub.g2.actuators.center_actuator(5);
    }
        break;
#endif

#if AP_SCRIPTING_ENABLED
    case JSButton::button_function_t::k_script_1:
        sub.script_buttons[0].release();
        break;
    case JSButton::button_function_t::k_script_2:
        sub.script_buttons[1].release();
        break;
    case JSButton::button_function_t::k_script_3:
        sub.script_buttons[2].release();
        break;
    case JSButton::button_function_t::k_script_4:
        sub.script_buttons[3].release();
        break;
#endif // AP_SCRIPTING_ENABLED
    }
}

/**
 * @brief Get JSButton parameter object for specified button index
 * 
 * @details Returns a pointer to the JSButton parameter object that stores the
 *          configuration for the specified button. ArduSub supports 32 configurable
 *          buttons (jbtn_0 through jbtn_31) that can be mapped to different functions.
 *          
 *          Each JSButton object stores:
 *          - Normal function (default button action)
 *          - Shift function (alternate action when shift modifier pressed)
 *          
 *          Button indices map directly to bit positions in the button bitmasks:
 *          - Buttons 0-15: From MAVLink MANUAL_CONTROL 'buttons' field
 *          - Buttons 16-31: From MAVLink MANUAL_CONTROL 'buttons2' field (extension)
 * 
 * @param[in] index Button index (0-31)
 * 
 * @return JSButton* Pointer to button parameter object, or &g.jbtn_0 if index invalid
 * 
 * @note Returns jbtn_0 as safe fallback for invalid indices
 * @note Button parameters are stored in the g (parameters) object
 * 
 * @see JSButton, handle_jsbutton_press(), handle_jsbutton_release()
 * 
 * Source: ArduSub/joystick.cpp:713-786
 */
JSButton* Sub::get_button(uint8_t index)
{
    // Help to access appropriate parameter
    switch (index) {
    case 0:
        return &g.jbtn_0;
    case 1:
        return &g.jbtn_1;
    case 2:
        return &g.jbtn_2;
    case 3:
        return &g.jbtn_3;
    case 4:
        return &g.jbtn_4;
    case 5:
        return &g.jbtn_5;
    case 6:
        return &g.jbtn_6;
    case 7:
        return &g.jbtn_7;
    case 8:
        return &g.jbtn_8;
    case 9:
        return &g.jbtn_9;
    case 10:
        return &g.jbtn_10;
    case 11:
        return &g.jbtn_11;
    case 12:
        return &g.jbtn_12;
    case 13:
        return &g.jbtn_13;
    case 14:
        return &g.jbtn_14;
    case 15:
        return &g.jbtn_15;

    // add 16 more cases for 32 buttons with MANUAL_CONTROL extensions
    case 16:
        return &g.jbtn_16;
    case 17:
        return &g.jbtn_17;
    case 18:
        return &g.jbtn_18;
    case 19:
        return &g.jbtn_19;
    case 20:
        return &g.jbtn_20;
    case 21:
        return &g.jbtn_21;
    case 22:
        return &g.jbtn_22;
    case 23:
        return &g.jbtn_23;
    case 24:
        return &g.jbtn_24;
    case 25:
        return &g.jbtn_25;
    case 26:
        return &g.jbtn_26;
    case 27:
        return &g.jbtn_27;
    case 28:
        return &g.jbtn_28;
    case 29:
        return &g.jbtn_29;
    case 30:
        return &g.jbtn_30;
    case 31:
        return &g.jbtn_31;
    default:
        return &g.jbtn_0;
    }
}

/**
 * @brief Set default button mappings for standard ROV joystick control
 * 
 * @details Initializes the first 16 buttons with factory default function mappings
 *          optimized for typical ROV operations. Each button has two functions:
 *          - Index [0]: Normal function (no shift modifier)
 *          - Index [1]: Shift function (when shift button held)
 *          
 *          Default Button Layout:
 *          Button  Normal Function          Shift Function
 *          ------  ----------------------   ----------------------
 *          0       None                     None
 *          1       Manual Mode              None
 *          2       Depth Hold Mode          None
 *          3       Stabilize Mode           None
 *          4       Disarm                   None
 *          5       Shift Modifier           None
 *          6       Arm                      None
 *          7       Mount Center             None
 *          8       Input Hold Set           None
 *          9       Mount Tilt Down          Mount Pan Left
 *          10      Mount Tilt Up            Mount Pan Right
 *          11      Gain Increase            Trim Pitch Down
 *          12      Gain Decrease            Trim Pitch Up
 *          13      Lights1 Dimmer           Trim Roll Left
 *          14      Lights1 Brighter         Trim Roll Right
 *          15      None                     None
 *          
 *          Buttons 16-31 are not initialized by this function and default to "none".
 *          
 *          This layout provides quick access to:
 *          - Essential modes (Manual, Stabilize, Depth Hold)
 *          - Arming/disarming
 *          - Camera/mount control with shift modifier for pan
 *          - Lights brightness adjustment
 *          - Gain control for sensitivity tuning
 *          - Trim adjustment when shift is held
 * 
 * @note Called during init_joystick() at vehicle startup
 * @note Users can reconfigure button functions via parameters (BTN0_FUNCTION, etc.)
 * @note Button 5 (shift) should not be reassigned as it enables alternate functions
 * 
 * @see init_joystick(), get_button(), JSButton::set_default()
 * 
 * Source: ArduSub/joystick.cpp:788-815
 */
void Sub::default_js_buttons()
{
    JSButton::button_function_t defaults[16][2] = {
        {JSButton::button_function_t::k_none,                   JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mode_manual,            JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mode_depth_hold,        JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mode_stabilize,         JSButton::button_function_t::k_none},

        {JSButton::button_function_t::k_disarm,                 JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_shift,                  JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_arm,                    JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mount_center,           JSButton::button_function_t::k_none},

        {JSButton::button_function_t::k_input_hold_set,         JSButton::button_function_t::k_none},
        {JSButton::button_function_t::k_mount_tilt_down,        JSButton::button_function_t::k_mount_pan_left},
        {JSButton::button_function_t::k_mount_tilt_up,          JSButton::button_function_t::k_mount_pan_right},
        {JSButton::button_function_t::k_gain_inc,               JSButton::button_function_t::k_trim_pitch_dec},

        {JSButton::button_function_t::k_gain_dec,               JSButton::button_function_t::k_trim_pitch_inc},
        {JSButton::button_function_t::k_lights1_dimmer,         JSButton::button_function_t::k_trim_roll_dec},
        {JSButton::button_function_t::k_lights1_brighter,       JSButton::button_function_t::k_trim_roll_inc},
        {JSButton::button_function_t::k_none,                   JSButton::button_function_t::k_none},
    };

    for (int i = 0; i < 16; i++) {
        get_button(i)->set_default(defaults[i][0], defaults[i][1]);
    }
}

/**
 * @brief Reset all control channels to neutral/trim positions
 * 
 * @details Sets all six RC input channels to their configured trim values,
 *          effectively commanding zero movement on all axes. Also clears any
 *          accumulated pitch and roll trim offsets.
 *          
 *          Channels reset to trim:
 *          - Roll: Typically 1500 μs (neutral)
 *          - Pitch: Typically 1500 μs (neutral)
 *          - Yaw: Typically 1500 μs (neutral)
 *          - Throttle: Typically 1500 μs (neutral for vertical)
 *          - Forward: Typically 1500 μs (neutral)
 *          - Lateral: Typically 1500 μs (neutral)
 *          
 *          This is used when:
 *          - Switching to a mode that doesn't use manual control
 *          - Failsafe activation
 *          - Disarming
 *          - Clearing accumulated trims
 * 
 * @note Does not clear input hold trims (xTrim, yTrim, zTrim)
 * @see clear_input_hold()
 * 
 * Source: ArduSub/joystick.cpp:817-829
 */
void Sub::set_neutral_controls()
{
    channel_roll->set_radio_in(channel_roll->get_radio_trim());
    channel_pitch->set_radio_in(channel_pitch->get_radio_trim());
    channel_yaw->set_radio_in(channel_yaw->get_radio_trim());
    channel_throttle->set_radio_in(channel_throttle->get_radio_trim());
    channel_forward->set_radio_in(channel_forward->get_radio_trim());
    channel_lateral->set_radio_in(channel_lateral->get_radio_trim());

    // Clear pitch/roll trim settings
    pitchTrim = 0;
    rollTrim  = 0;
}

/**
 * @brief Clear input hold trim offsets and disengage hold mode
 * 
 * @details Resets all input hold trim values to zero and disengages the input hold
 *          feature. Input hold allows the operator to "lock" forward, lateral, and
 *          vertical inputs at their current values, maintaining position/velocity
 *          while freeing up hands for other controls.
 *          
 *          When input hold is engaged:
 *          - xTrim stores forward/backward offset
 *          - yTrim stores lateral/strafe offset  
 *          - zTrim stores vertical/throttle offset
 *          
 *          Clearing input hold:
 *          - Returns vehicle to direct joystick control
 *          - Removes all trim offsets
 *          - Allows immediate response to stick inputs
 *          
 *          This is typically called when:
 *          - Disarming the vehicle
 *          - Changing flight modes
 *          - Operator presses input hold button with sticks neutral
 * 
 * @note Separate from pitch/roll trim which is cleared by set_neutral_controls()
 * @see set_neutral_controls(), handle_jsbutton_press() k_input_hold_set case
 * 
 * Source: ArduSub/joystick.cpp:831-837
 */
void Sub::clear_input_hold()
{
    xTrim = 0;
    yTrim = 0;
    zTrim = 0;
    input_hold_engaged = false;
}

#if AP_SCRIPTING_ENABLED
/**
 * @brief Check if a script button is currently pressed
 * 
 * @details Allows Lua scripts to query the current pressed state of script buttons
 *          (k_script_1 through k_script_4). This provides real-time button state
 *          information for scripts that need to know if a button is being held.
 * 
 * @param[in] index Script button index (1-4, corresponding to k_script_1 through k_script_4)
 * 
 * @return true if script button is currently pressed, false otherwise
 * 
 * @note Index is 1-based (1-4) but internally maps to 0-based array (0-3)
 * @see get_and_clear_button_count(), handle_jsbutton_press() k_script_* cases
 * 
 * Source: ArduSub/joystick.cpp:840-843
 */
bool Sub::is_button_pressed(uint8_t index)
{
    return script_buttons[index - 1].is_pressed();
}

/**
 * @brief Get and reset the press count for a script button
 * 
 * @details Returns the number of times a script button has been pressed since the
 *          last call to this function, then resets the counter to zero. This allows
 *          Lua scripts to detect button presses even if they don't run at the same
 *          rate as button processing, ensuring no press events are missed.
 *          
 *          Useful for:
 *          - Counting button presses for multi-click detection
 *          - Ensuring all presses are processed even with slow script execution
 *          - Implementing button press accumulators in scripts
 * 
 * @param[in] index Script button index (1-4, corresponding to k_script_1 through k_script_4)
 * 
 * @return uint8_t Number of button presses since last call (counter is then reset to 0)
 * 
 * @note Index is 1-based (1-4) but internally maps to 0-based array (0-3)
 * @note Counter resets after reading, so subsequent calls return 0 until new presses
 * @see is_button_pressed(), handle_jsbutton_press() k_script_* cases
 * 
 * Source: ArduSub/joystick.cpp:845-848
 */
uint8_t Sub::get_and_clear_button_count(uint8_t index)
{
    return script_buttons[index - 1].get_and_clear_count();
}
#endif // AP_SCRIPTING_ENABLED
