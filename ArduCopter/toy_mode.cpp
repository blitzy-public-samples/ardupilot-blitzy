/**
 * @file toy_mode.cpp
 * @brief Toy mode implementation providing simplified user interface for mass-market drones
 * 
 * @details This module implements a simplified control interface designed for toy-grade
 *          consumer drones, specifically targeting the SkyViper V2450GPS (Version 1) and
 *          F412-based boards (Version 2). Toy mode abstracts complex flight control operations
 *          into simple button presses, automatic mode transitions, and LED feedback.
 *          
 *          Key Features:
 *          - Button-based control interface (left/right/action/power buttons)
 *          - Automatic stick trimming while disarmed
 *          - Simplified arming with throttle high/low detection
 *          - LED blink patterns for status feedback
 *          - Voltage-based thrust limiting for consistent flight characteristics
 *          - Mode toggling without RC mode switches
 *          - Automatic GPS/non-GPS mode handling
 *          
 *          Button Mapping:
 *          - V2450: Uses channels 5, 6, 7 with latching left button
 *          - F412: Uses channels 5, 6 with bit-encoded button states
 *          
 *          State Machine:
 *          The toy mode operates a simplified state machine with the following states:
 *          - Disarmed: Vehicle on ground, motors stopped
 *          - Armed (GPS mode): Flight with position hold capabilities
 *          - Armed (non-GPS mode): Flight with altitude hold only
 *          - RTL: Return to launch
 *          - Landing: Controlled descent
 *          
 * @note This is safety-critical code - modifications require extensive testing
 * @warning Toy mode bypasses many normal ArduPilot safety checks for simplified operation
 * 
 * Source: ArduCopter/toy_mode.cpp
 */

#include "Copter.h"

#if TOY_MODE_ENABLED

// Timing constants in 0.1s units (100ms)
#define TOY_COMMAND_DELAY 15          ///< Delay between repeated command actions (1.5 seconds)
#define TOY_LONG_PRESS_COUNT 15       ///< Long press threshold (1.5 seconds)
#define TOY_LAND_MANUAL_DISARM_COUNT 40  ///< Auto-disarm delay in manual modes (4 seconds)
#define TOY_LAND_DISARM_COUNT 1       ///< Auto-disarm delay in auto modes (0.1 seconds)
#define TOY_LAND_ARM_COUNT 1          ///< Throttle high duration to arm (0.1 seconds)
#define TOY_RIGHT_PRESS_COUNT 1       ///< Right button press threshold (0.1 seconds)
#define TOY_ACTION_DELAY_MS 200       ///< Minimum delay between action triggers (milliseconds)
#define TOY_DESCENT_SLOW_HEIGHT 5     ///< Height in meters to start slowing descent
#define TOY_DESCENT_SLOW_RAMP 3       ///< Ramp distance in meters for descent slowdown
#define TOY_DESCENT_SLOW_MIN 300      ///< Minimum throttle during slow descent
#define TOY_RESET_TURTLE_TIME 5000    ///< Time to hold buttons inverted for WiFi reset (milliseconds)

#define ENABLE_LOAD_TEST 0            ///< Enable motor load testing feature (factory use only)

const AP_Param::GroupInfo ToyMode::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: tmode enable 
    // @Description: tmode (or "toy" mode) gives a simplified user interface designed for mass market drones. Version1 is for the SkyViper V2450GPS. Version2 is for the F412 based boards
    // @Values: 0:Disabled,1:EnableVersion1,2:EnableVersion2
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, ToyMode, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MODE1
    // @DisplayName: Tmode first mode
    // @Description: This is the initial mode when the vehicle is first turned on. This mode is assumed to not require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:FlowHold
    // @User: Standard
    AP_GROUPINFO("_MODE1", 2, ToyMode, primary_mode[0], (float)Mode::Number::ALT_HOLD),

    // @Param: _MODE2
    // @DisplayName: Tmode second mode
    // @Description: This is the secondary mode. This mode is assumed to require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:FlowHold
    // @User: Standard
    AP_GROUPINFO("_MODE2", 3, ToyMode, primary_mode[1], (float)Mode::Number::LOITER),

    // @Param: _ACTION1
    // @DisplayName: Tmode action 1
    // @Description: This is the action taken when the left action button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION1", 4, ToyMode, actions[0], ACTION_TOGGLE_VIDEO),

    // @Param: _ACTION2
    // @DisplayName: Tmode action 2
    // @Description: This is the action taken when the right action button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION2", 5, ToyMode, actions[1], ACTION_TAKE_PHOTO),

    // @Param: _ACTION3
    // @DisplayName: Tmode action 3
    // @Description: This is the action taken when the power button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION3", 6, ToyMode, actions[2], ACTION_DISARM),

    // @Param: _ACTION4
    // @DisplayName: Tmode action 4
    // @Description: This is the action taken when the left action button is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION4", 7, ToyMode, actions[3], ACTION_NONE),

    // @Param: _ACTION5
    // @DisplayName: Tmode action 5
    // @Description: This is the action taken when the right action is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION5", 8, ToyMode, actions[4], ACTION_NONE),

    // @Param: _ACTION6
    // @DisplayName: Tmode action 6
    // @Description: This is the action taken when the power button is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION6", 9, ToyMode, actions[5], ACTION_NONE),

    // @Param: _LEFT
    // @DisplayName: Tmode left action
    // @Description: This is the action taken when the left (Mode) button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_LEFT", 10, ToyMode, actions[6], ACTION_TOGGLE_MODE),

    // @Param: _LEFT_LONG
    // @DisplayName: Tmode left long action
    // @Description: This is the action taken when the left (Mode) button is long-pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_LEFT_LONG", 11, ToyMode, actions[7], ACTION_NONE),

    // @Param: _TRIM_AUTO
    // @DisplayName: Stick auto trim limit
    // @Description: This is the amount of automatic stick trim that can be applied when disarmed with sticks not moving. It is a PWM limit value away from 1500
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_TRIM_AUTO", 12, ToyMode, trim_auto, 50),

    // @Param: _RIGHT
    // @DisplayName: Tmode right action
    // @Description: This is the action taken when the right (Return) button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest
    // @User: Standard
    AP_GROUPINFO("_RIGHT", 13, ToyMode, actions[8], ACTION_ARM_LAND_RTL),

    // @Param: _FLAGS
    // @DisplayName: Tmode flags
    // @Description: Bitmask of flags to change the behaviour of tmode. DisarmOnLowThrottle means to disarm if throttle is held down for 1 second when landed. ArmOnHighThrottle means to arm if throttle is above 80% for 1 second. UpgradeToLoiter means to allow takeoff in LOITER mode by switching to ALT_HOLD, then auto-upgrading to LOITER once GPS is available. RTLStickCancel means that on large stick inputs in RTL mode that LOITER mode is engaged
    // @Bitmask: 0:DisarmOnLowThrottle,1:ArmOnHighThrottle,2:UpgradeToLoiter,3:RTLStickCancel
    // @User: Standard
    AP_GROUPINFO("_FLAGS", 14, ToyMode, flags, FLAG_THR_DISARM),

    // @Param: _VMIN
    // @DisplayName: Min voltage for output limiting
    // @Description: This is the battery voltage below which no output limiting is done
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_VMIN", 15, ToyMode, filter.volt_min, 3.5),

    // @Param: _VMAX
    // @DisplayName: Max voltage for output limiting
    // @Description: This is the battery voltage above which thrust min is used
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_VMAX", 16, ToyMode, filter.volt_max, 3.8),
    
    // @Param: _TMIN
    // @DisplayName: Min thrust multiplier
    // @Description: This sets the thrust multiplier when voltage is high
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TMIN", 17, ToyMode, filter.thrust_min, 1.0),

    // @Param: _TMAX
    // @DisplayName: Max thrust multiplier
    // @Description: This sets the thrust multiplier when voltage is low
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TMAX", 18, ToyMode, filter.thrust_max, 1.0),

#if ENABLE_LOAD_TEST
    // @Param: _LOAD_MUL
    // @DisplayName: Load test multiplier
    // @Description: This scales the load test output, as a value between 0 and 1
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_LOAD_MUL", 19, ToyMode, load_test.load_mul, 1.0),
    
    // @Param: _LOAD_FILT
    // @DisplayName: Load test filter
    // @Description: This filters the load test output. A value of 1 means no filter. 2 means values are repeated once. 3 means values are repeated 3 times, etc
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_LOAD_FILT", 20, ToyMode, load_test.load_filter, 1),
    
    // @Param: _LOAD_TYPE
    // @DisplayName: Load test type
    // @Description: This sets the type of load test
    // @Values: 0:ConstantThrust,1:LogReplay1,2:LogReplay2
    // @User: Advanced
    AP_GROUPINFO("_LOAD_TYPE", 21, ToyMode, load_test.load_type, LOAD_TYPE_LOG1),
#endif
   
    AP_GROUPEND
};

/**
 * @brief Constructor for ToyMode class
 * 
 * @details Initializes the ToyMode object and sets up default parameter values
 *          from the var_info table. This ensures all configurable parameters
 *          have sensible defaults before user configuration is loaded.
 */
ToyMode::ToyMode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Main toy mode update function - called at 10Hz from main scheduler
 * 
 * @details This is the core of toy mode operation, handling:
 *          - Button input decoding from RC channels
 *          - Action mapping and execution
 *          - Automatic arming/disarming based on throttle position
 *          - LED blink pattern updates for user feedback
 *          - Mode transitions and GPS/non-GPS mode management
 *          - Safety interlocks (stick centering, compass checks)
 *          
 *          The function implements a simplified state machine for toy operation:
 *          
 *          State Machine Overview:
 *          ```
 *          [Disarmed] --throttle high--> [Armed]
 *          [Armed] --throttle low--> [Disarmed] (when landed)
 *          [Armed] --right button--> [RTL/Land based on GPS availability]
 *          [RTL/Land] --right button--> [Armed mode]
 *          [Any mode] --left button--> [Toggle between MODE1/MODE2]
 *          ```
 *          
 *          Button Processing Pipeline:
 *          1. Read RC channels 5, 6, 7 (button encoding varies by hardware version)
 *          2. Decode button states (left, right, action buttons, power button)
 *          3. Check for button combinations (e.g., left + action)
 *          4. Map to configured actions (ACTION1-ACTION6, LEFT, RIGHT)
 *          5. Apply timing constraints (debounce, long-press detection)
 *          6. Execute actions (mode changes, arm/disarm, camera control)
 *          
 * @note Called at 10Hz by the main scheduler task
 * @warning This function bypasses some normal pre-arm checks for user convenience
 */
void ToyMode::update()
{
    if (!enable) {
        // Toy mode disabled via parameter - exit early
        return;
    }

#if ENABLE_LOAD_TEST
    // Stop load test if vehicle disarms
    if (!copter.motors->armed()) {
        load_test.running = false;
    }
#endif

    // Apply exponential filter to battery voltage for smooth thrust limiting
    // Filter coefficient: 0.99 provides ~1 second time constant at 10Hz
    filtered_voltage = 0.99 * filtered_voltage + 0.01 * copter.battery.voltage();
    
    // Update LED blink patterns to provide visual feedback to user
    blink_update();
    
    // First-time initialization when toy mode starts
    if (!done_first_update) {
        done_first_update = true;
        // Set initial flight mode to primary_mode[0] (typically ALT_HOLD)
        copter.set_mode(Mode::Number(primary_mode[0].get()), ModeReason::TOY_MODE);
        // Register thrust limiting callback for voltage-based power management
        copter.motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&ToyMode::thrust_limiting, void, float *, uint8_t));
    }

    // Perform automatic stick trimming if enabled
    // This adjusts RC input trim when sticks are centered and vehicle is disarmed
    if (trim_auto > 0) {
        trim_update();
    }
            
    // Optionally set ALT_HOLD as indoors for the EKF (disables GPS vertical velocity fusion)
    // This feature is currently disabled but left for future consideration
#if 0
    copter.ahrs.set_indoor_mode(copter.flightmode->mode_number() == ALT_HOLD || copter.flightmode->mode_number() == FLOWHOLD);
#endif
    
    // Button state variables - decoded from RC channels
    bool left_button = false;           // Mode toggle button (latching on V2450)
    bool right_button = false;          // Return/Arm button
    bool left_action_button = false;    // Left shoulder button (camera left)
    bool right_action_button = false;   // Right shoulder button (camera right)
    bool power_button = false;          // Power/disarm button
    bool left_change = false;           // Detects left button press/release transitions
    
    // Read RC channels used for button encoding
    // Button encoding varies by hardware version (V2450 vs F412)
    uint16_t ch5_in = RC_Channels::get_radio_in(CH_5);
    uint16_t ch6_in = RC_Channels::get_radio_in(CH_6);
    uint16_t ch7_in = RC_Channels::get_radio_in(CH_7);

    // RC Failsafe Detection
    // If no valid RC input or channel 5 below threshold, enter failsafe mode
    if (!rc().has_valid_input() || ch5_in < 900) {
        // Toy mode does not handle failsafe actions - normal ArduPilot failsafe takes over
        // Set LED blink patterns to indicate no RC signal
        green_blink_pattern = BLINK_NO_RX;
        red_blink_pattern = BLINK_NO_RX;
        red_blink_index = green_blink_index;  // Synchronize LED blinks
        return;
    }

    uint32_t now = AP_HAL::millis();
    
    // Button Decoding - Hardware-specific button encoding on RC channels
    if (is_v2450_buttons()) {
        // V2450 SkyViper with Cypress radio button mapping
        // Channel 5: Left button with latching behavior (toggles between high/low)
        // Channel 6: Right button (simple high/low)
        // Channel 7: Three action buttons encoded as bit pattern (100μs per bit)
        
        // Detect left button state transitions for toggle behavior
        left_change = ((ch5_in > 1700 && last_ch5 <= 1700) || (ch5_in <= 1700 && last_ch5 > 1700));
        
        last_ch5 = ch5_in;  // Remember for next cycle
                        
        // Decode button states from channel values
        // Left button latches, so check both high (>2050) and low latch positions (1050-1150)
        left_button = (ch5_in > 2050 || (ch5_in > 1050 && ch5_in < 1150));
        right_button = (ch6_in > 1500);
        
        // Channel 7 encodes 3 buttons as bits in PWM offset from 1000μs
        // Each button adds 100μs: bit0=+100, bit1=+200, bit2=+400
        uint8_t ch7_bits = (ch7_in>1000)?uint8_t((ch7_in-1000)/100):0;
        left_action_button = (ch7_bits&1) != 0;   // Bit 0: Left shoulder
        right_action_button = (ch7_bits&2) != 0;  // Bit 1: Right shoulder
        power_button = (ch7_bits&4) != 0;         // Bit 2: Power button
    } else if (is_f412_buttons()) {
        // F412-based boards with CC2500 radio button mapping
        // Channel 5: 3 buttons encoded as bits (left, right, mode)
        // Channel 6: 3 buttons encoded as bits (action buttons)
        // No latching behavior - buttons read as momentary switches
        
        // Decode bit patterns from PWM values (100μs offset per bit)
        uint8_t ch5_bits = (ch5_in>1000)?uint8_t((ch5_in-1000)/100):0;
        uint8_t ch6_bits = (ch6_in>1000)?uint8_t((ch6_in-1000)/100):0;
        
        // Extract individual button states from bit fields
        left_button = (ch5_bits & 4) != 0;        // Bit 2 of ch5
        right_button = (ch5_bits & 2) != 0;       // Bit 1 of ch5
        right_action_button = (ch6_bits & 1) != 0;  // Bit 0 of ch6
        left_action_button = (ch6_bits & 2) != 0;   // Bit 1 of ch6
        power_button = (ch6_bits & 4) != 0;         // Bit 2 of ch6
        
        // Detect left button transitions (no latching)
        left_change = (left_button != last_left_button);
        last_left_button = left_button;
    }
    
    // Action Button Mapping
    // Map action buttons to action indices (1-based for parameter lookup)
    // Actions 1-3: Single button presses (ACTION1, ACTION2, ACTION3 parameters)
    // Actions 4-6: Button + left button combinations (ACTION4, ACTION5, ACTION6 parameters)
    uint8_t action_input = 0;    
    if (left_action_button) {
        action_input = 1;  // Maps to ACTION1 parameter
    } else if (right_action_button) {
        action_input = 2;  // Maps to ACTION2 parameter
    } else if (power_button) {
        action_input = 3;  // Maps to ACTION3 parameter
    }
    
    // Button Combinations
    // If left (mode) button held with action button, use combined actions (ACTION4-6)
    if (action_input != 0 && left_button) {
        action_input += 3;  // Convert to ACTION4, ACTION5, or ACTION6
        left_press_counter = 0;  // Reset long-press counter
    } else if (left_button) {
        // Left button held alone - count for long-press detection
        left_press_counter++;
    } else {
        // Left button released - reset counter
        left_press_counter = 0;
    }

    // Special WiFi Reset Feature (Turtle Mode Recovery)
    // If both shoulder buttons pressed while inverted for 5 seconds, reset WiFi credentials
    // This allows recovery if vehicle flips over and user can't reach it
    bool reset_combination = left_action_button && right_action_button;
    if (reset_combination && abs(copter.ahrs.roll_sensor) > 160) {
        // Vehicle is inverted (roll > 160 degrees) with both shoulders pressed
        if (reset_turtle_start_ms == 0) {
            reset_turtle_start_ms = now;  // Start timer
        }
        if (now - reset_turtle_start_ms > TOY_RESET_TURTLE_TIME) {
            // Held for full duration - send WiFi reset command
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: WiFi reset");
            reset_turtle_start_ms = 0;
            send_named_int("WIFIRESET", 1);  // MAVLink message to companion computer
        }
    } else {
        // Not in turtle reset condition - clear timer
        reset_turtle_start_ms = 0;
    }
    if (reset_combination) {
        // Block normal button actions during reset combination
        action_input = 0;
        left_press_counter = 0;
    }

    // Action Command Processing
    // Determine which action to execute based on button states and parameter configuration
    // actions[] array maps to ACTION1-6, LEFT, LEFT_LONG, RIGHT parameters
    enum toy_action action = action_input?toy_action(actions[action_input-1].get()):ACTION_NONE;
   
    // Long Press Detection for Left Button
    // If left button held for >1.5 seconds without other buttons, trigger LEFT_LONG action
    if (action == ACTION_NONE && left_press_counter > TOY_LONG_PRESS_COUNT) {
        left_press_counter = -TOY_COMMAND_DELAY;  // Reset with negative value to prevent repeat
        action = toy_action(actions[7].get());     // LEFT_LONG parameter (index 7)
        ignore_left_change = true;  // Prevent release from triggering left action
    }

    // Suppress left button release after long press
    // This prevents the release from triggering the normal LEFT action
    if (ignore_left_change && left_change) {
        left_change = false;
        ignore_left_change = false;
    }

    // Left Button Press/Release Handling (hardware-dependent)
    if (is_v2450_buttons()) {
        // V2450: Left button latches, so any state change triggers action
        if (action == ACTION_NONE && left_change) {
            action = toy_action(actions[6].get());  // LEFT parameter (index 6)
        }
    } else if (is_f412_buttons()) {
        // F412: Left button doesn't latch, so only trigger on release
        if (action == ACTION_NONE && left_change && !left_button) {
            action = toy_action(actions[6].get());  // LEFT parameter (index 6)
        }
    }

    // Right Button Handling
    // Right button triggers after short press (0.1 seconds at 10Hz = 1 count)
    if (action == ACTION_NONE && right_button) {
        right_press_counter++;
        if (right_press_counter >= TOY_RIGHT_PRESS_COUNT) {
            action = toy_action(actions[8].get());  // RIGHT parameter (index 8)
            right_press_counter = -TOY_COMMAND_DELAY;  // Prevent immediate repeat
        }
    } else {
        right_press_counter = 0;
    }

    // Action Repeat Prevention
    // Certain actions should not repeat if button held down or pressed too quickly
    // This prevents accidental mode changes and camera command flooding
    switch (action) {
    case ACTION_TOGGLE_VIDEO:
    case ACTION_TOGGLE_MODE:
    case ACTION_TOGGLE_SIMPLE:
    case ACTION_TOGGLE_SSIMPLE:
    case ACTION_ARM_LAND_RTL:
    case ACTION_LOAD_TEST:
    case ACTION_MODE_FLOW:
        // These toggle-type actions require button release before re-triggering
        if (last_action == action ||
            now - last_action_ms < TOY_ACTION_DELAY_MS) {
            last_action = action;  // Remember action to detect repeat
            action = ACTION_NONE;   // Block execution until released
        }
        break;
        
    case ACTION_TAKE_PHOTO:
        // Photo action allows continuous shooting (burst mode)
        // but still enforce minimum 200ms delay between shots
        if (now - last_action_ms < TOY_ACTION_DELAY_MS) {
            last_action = action;
            action = ACTION_NONE;
        }
        break;

    default:
        // Other actions can repeat freely
        last_action = action;
        break;
    }
    
    // Log action execution for debugging
    if (action != ACTION_NONE) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: action %u", action);
        last_action_ms = now;
    }

    // Throttle Position Detection for Auto-Arm/Disarm
    // Use generous threshold (150/1000) to handle varying RC transmitter calibrations
    bool throttle_at_min =
        copter.channel_throttle->get_control_in() < 150;

    // Throttle near maximum detection for throttle-based arming
    // Threshold at 70% (700/1000) to require deliberate high throttle
    bool throttle_near_max =
        copter.channel_throttle->get_control_in() > 700;
    
    // Automatic Disarm on Low Throttle (Safety Feature)
    // If throttle held low while landed, automatically disarm motors
    // Timing varies by mode: 4 seconds for manual modes, 0.1 seconds for auto modes
    if ((flags & FLAG_THR_DISARM) && throttle_at_min && copter.motors->armed() && copter.ap.land_complete) {
        throttle_low_counter++;
        // Manual throttle modes get longer timeout to prevent accidental disarm
        // Auto modes disarm quickly since throttle isn't actively controlled
        const uint8_t disarm_limit = copter.flightmode->has_manual_throttle()?TOY_LAND_MANUAL_DISARM_COUNT:TOY_LAND_DISARM_COUNT;
        if (throttle_low_counter >= disarm_limit) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: throttle disarm");
            copter.arming.disarm(AP_Arming::Method::TOYMODELANDTHROTTLE);
        }
    } else {
        throttle_low_counter = 0;
    }

    // Automatic Arm on High Throttle (Simplified Arming)
    // If throttle raised to >70% while disarmed, automatically arm motors
    // This provides intuitive "throttle to takeoff" behavior for toy users
    if ((flags & FLAG_THR_ARM) && throttle_near_max && !copter.motors->armed()) {
        throttle_high_counter++;
        if (throttle_high_counter >= TOY_LAND_ARM_COUNT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: throttle arm");
            // Check compass health and enable learning if needed
            arm_check_compass();
            
            // Attempt to arm in current mode
            if (!copter.arming.arm(AP_Arming::Method::MAVLINK) && (flags & FLAG_UPGRADE_LOITER) && copter.flightmode->mode_number() == Mode::Number::LOITER) {
                // LOITER Mode GPS Fallback Strategy
                // If arming fails in LOITER (usually due to no GPS), try ALT_HOLD instead
                // Then automatically upgrade back to LOITER once GPS becomes available
                if (set_and_remember_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Tmode: ALT_HOLD update arm");
#if AP_FENCE_ENABLED
                    // Disable fence in ALT_HOLD since it requires GPS
                    copter.fence.enable(false, AC_FENCE_ALL_FENCES);
#endif
                    if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                        // Even ALT_HOLD arming failed - revert to LOITER
                        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: ALT_HOLD arm failed");
                        set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE);
                    } else {
                        // Armed in ALT_HOLD - mark for upgrade to LOITER when GPS available
                        upgrade_to_loiter = true;
#if 0
                        AP_Notify::flags.hybrid_loiter = true;
#endif
                    }
                }
            } else {
                // Arming successful - record time for throttle soft-start
                throttle_arm_ms = AP_HAL::millis();
            }
        }
    } else {
        throttle_high_counter = 0;
    }

    // Automatic Upgrade from ALT_HOLD to LOITER (GPS Mode Transition)
    // When armed in ALT_HOLD as a fallback for LOITER, monitor for GPS availability
    // Once GPS position is good, automatically switch to LOITER for position hold
    if (upgrade_to_loiter) {
        if (!copter.motors->armed() || copter.flightmode->mode_number() != Mode::Number::ALT_HOLD) {
            // Cancel upgrade if disarmed or mode changed by pilot
            upgrade_to_loiter = false;
#if 0
            AP_Notify::flags.hybrid_loiter = false;
#endif
        } else if (copter.position_ok() && set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE)) {
            // GPS position now available - upgrade to LOITER
#if AP_FENCE_ENABLED
            copter.fence.enable(true, AC_FENCE_ALL_FENCES);
#endif
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: LOITER update");            
        }
    }

    // RTL Stick Cancel Feature (User Override)
    // Allow pilot to cancel RTL by raising throttle to max
    // This provides escape from unwanted RTL triggered by low battery or RC loss
    if (copter.flightmode->mode_number() == Mode::Number::RTL && (flags & FLAG_RTL_CANCEL) && throttle_near_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: RTL cancel");        
        set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE);
    }
    
    // Mode Transition State Machine
    // Track current and desired flight modes for state transitions
    enum Mode::Number old_mode = copter.flightmode->mode_number();
    enum Mode::Number new_mode = old_mode;

    // Action Execution Switch
    // Execute the determined action by mapping to flight modes, camera controls, or special functions
    // Most actions set new_mode which is applied at the end of update()
    switch (action) {
    case ACTION_NONE:
        // No action this cycle
        break;

    case ACTION_TAKE_PHOTO:
        // Send camera snapshot command to companion computer via MAVLink
        send_named_int("SNAPSHOT", 1);
        break;

    case ACTION_TOGGLE_VIDEO:
        // Toggle video recording on companion computer
        send_named_int("VIDEOTOG", 1);
        break;

    case ACTION_MODE_ACRO:
        // Switch to ACRO mode (rate-based manual control)
#if MODE_ACRO_ENABLED
        new_mode = Mode::Number::ACRO;
#else
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: ACRO is disabled");
#endif
        break;

    case ACTION_MODE_ALTHOLD:
        // Switch to ALT_HOLD mode (altitude stabilized, manual horizontal control)
        new_mode = Mode::Number::ALT_HOLD;
        break;

    case ACTION_MODE_AUTO:
        // Switch to AUTO mode (mission execution from waypoints)
        new_mode = Mode::Number::AUTO;
        break;

    case ACTION_MODE_LOITER:
        // Switch to LOITER mode (GPS position hold)
        new_mode = Mode::Number::LOITER;
        break;

    case ACTION_MODE_RTL:
        // Switch to RTL mode (Return To Launch)
        new_mode = Mode::Number::RTL;
        break;

    case ACTION_MODE_CIRCLE:
        // Switch to CIRCLE mode (fly in circles around position)
        new_mode = Mode::Number::CIRCLE;
        break;

    case ACTION_MODE_LAND:
        // Switch to LAND mode (autonomous landing)
        new_mode = Mode::Number::LAND;
        break;

    case ACTION_MODE_DRIFT:
        // Switch to DRIFT mode (like car racing controls)
        new_mode = Mode::Number::DRIFT;
        break;

    case ACTION_MODE_SPORT:
        // Switch to SPORT mode (high rate manual control with attitude limits)
        new_mode = Mode::Number::SPORT;
        break;

    case ACTION_MODE_AUTOTUNE:
        // Switch to AUTOTUNE mode (automatic PID tuning)
        new_mode = Mode::Number::AUTOTUNE;
        break;

    case ACTION_MODE_POSHOLD:
        // Switch to POSHOLD mode (position hold with brake on stick release)
        new_mode = Mode::Number::POSHOLD;
        break;

    case ACTION_MODE_BRAKE:
        // Switch to BRAKE mode (aggressive position hold)
        new_mode = Mode::Number::BRAKE;
        break;

    case ACTION_MODE_THROW:
        // Switch to THROW mode (throw to launch)
#if MODE_THROW_ENABLED
        new_mode = Mode::Number::THROW;
#else
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: THROW is disabled");
#endif
        break;

    case ACTION_MODE_FLIP:
        // Switch to FLIP mode (automated flip maneuver)
        new_mode = Mode::Number::FLIP;
        break;

    case ACTION_MODE_STAB:
        // Switch to STABILIZE mode (basic manual control with attitude stabilization)
        new_mode = Mode::Number::STABILIZE;
        break;

    case ACTION_MODE_FLOW:
        // Toggle FLOWHOLD mode (optical flow position hold) with ALT_HOLD
        if (old_mode != Mode::Number::FLOWHOLD) {
            new_mode = Mode::Number::FLOWHOLD;
        } else {
            new_mode = Mode::Number::ALT_HOLD;
        }
        break;
        
    case ACTION_DISARM:
        // Force disarm motors immediately (emergency stop)
        if (copter.motors->armed()) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: Force disarm");
            copter.arming.disarm(AP_Arming::Method::TOYMODELANDFORCE);
        }
        break;

    case ACTION_TOGGLE_MODE:
        // Toggle between primary mode 1 and primary mode 2 (configurable via parameters)
        // Allows simple two-mode switching (typically ALT_HOLD and LOITER)
        last_mode_choice = (last_mode_choice+1) % 2;
        new_mode = Mode::Number(primary_mode[last_mode_choice].get());
        break;

    case ACTION_TOGGLE_SIMPLE:
        // Toggle SIMPLE mode (yaw is relative to arming heading, simplifies control)
        copter.set_simple_mode(bool(copter.simple_mode)?Copter::SimpleMode::NONE:Copter::SimpleMode::SIMPLE);
        break;

    case ACTION_TOGGLE_SSIMPLE:
        // Toggle SUPER SIMPLE mode (yaw is relative to home position)
        copter.set_simple_mode(bool(copter.simple_mode)?Copter::SimpleMode::NONE:Copter::SimpleMode::SUPERSIMPLE);
        break;
        
    case ACTION_ARM_LAND_RTL:
        // Multi-function "smart button" that adapts based on vehicle state
        // This implements single-button autonomous flight control:
        // - Disarmed: Arm motors
        // - Armed in GPS mode: Trigger RTL (return to launch)
        // - Armed in non-GPS mode: Trigger LAND (vertical landing)
        // - In RTL: Switch to LOITER (cancel RTL)
        // - In LAND: Return to previous flight mode
        if (!copter.motors->armed()) {
            // Disarmed: arm the vehicle with safety checks
            action_arm();
        } else if (old_mode == Mode::Number::RTL) {
            // Already in RTL: switch to LOITER to cancel return
            new_mode = Mode::Number::LOITER;
        } else if (old_mode == Mode::Number::LAND) {
            // Already landing: abort landing and return to flight
            if (last_set_mode == Mode::Number::LAND || !copter.position_ok()) {
                // User requested land, or no GPS - go to ALT_HOLD
                new_mode = Mode::Number::ALT_HOLD;
            } else if (copter.landing_with_GPS()) {
                // Was in GPS mode before landing - return to LOITER
                new_mode = Mode::Number::LOITER;
            } else {
                // Was in non-GPS mode - return to ALT_HOLD
                new_mode = Mode::Number::ALT_HOLD;
            }
        } else if (copter.flightmode->requires_GPS()) {
            // Currently in GPS mode: trigger RTL (return home)
            new_mode = Mode::Number::RTL;
        } else {
            // Currently in non-GPS mode: trigger LAND (descend vertically)
            new_mode = Mode::Number::LAND;
        }
        break;

    case ACTION_LOAD_TEST:
        // Motor Load Test (Factory Testing Feature)
        // Runs motors through predetermined thrust patterns to test endurance
        // Used in manufacturing to validate motor/ESC/battery performance
#if ENABLE_LOAD_TEST
        if (copter.motors->armed() && !load_test.running) {
            // Already armed but test not running - don't start test
            break;
        }
        if (load_test.running) {
            // Stop currently running load test
            load_test.running = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test off");
            copter.init_disarm_motors();
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
        } else {
            // Start load test: arm motors and begin test sequence
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
#if AP_FENCE_ENABLED
            copter.fence.enable(false);
#endif
            if (copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                load_test.running = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test on");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test failed");
            }
        }
#endif
        break;
    }

    // Automatic Mode Reversion After Landing
    // When disarmed after an autonomous LAND or RTL, revert to the last primary flight mode
    // This prevents staying in LAND/RTL mode when rearming
    if (!copter.motors->armed() && (copter.flightmode->mode_number() == Mode::Number::LAND || copter.flightmode->mode_number() == Mode::Number::RTL)) {
        new_mode = Mode::Number(primary_mode[last_mode_choice].get());
    }
    
    // Mode Change Execution
    // Apply the new mode if different from current mode
    if (new_mode != copter.flightmode->mode_number()) {
        // Stop any running load test when changing modes
        load_test.running = false;
        
        // Disable geofence temporarily during mode transitions
#if AP_FENCE_ENABLED
        copter.fence.enable(false, AC_FENCE_ALL_FENCES);
#endif
        
        // Attempt mode change
        if (set_and_remember_mode(new_mode, ModeReason::TOY_MODE)) {
            // Mode change successful
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: mode %s", copter.flightmode->name4());
            
            // Re-enable geofence for GPS-based flight modes
            // GPS modes have position awareness to respect fence boundaries
#if AP_FENCE_ENABLED
            if (copter.flightmode->requires_GPS()) {
                copter.fence.enable(true, AC_FENCE_ALL_FENCES);
            }
#endif
        } else {
            // Mode change failed - handle RTL failure specially
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: %u FAILED", (unsigned)new_mode);
            if (new_mode == Mode::Number::RTL) {
                // RTL failure is critical - force LAND as emergency fallback
                // This ensures vehicle doesn't continue flying with failed RTL
                gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: LANDING");
                set_and_remember_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
#if AP_FENCE_ENABLED
                if (copter.landing_with_GPS()) {
                    // Enable fence if landing with GPS guidance
                    copter.fence.enable(true, AC_FENCE_ALL_FENCES);
                }
#endif
            }
        }
    }
}


/**
 * @brief Set flight mode and remember the mode change for later reference
 * 
 * @details This helper function attempts to change the flight mode and tracks the
 *          mode that was explicitly set by toy mode. This tracking allows the system
 *          to distinguish between user-requested mode changes and automatic mode
 *          changes (e.g., failsafe-triggered modes). The last_set_mode variable is
 *          used by ACTION_ARM_LAND_RTL to determine appropriate mode transitions.
 * 
 * @param[in] mode The desired flight mode to switch to
 * @param[in] reason The reason for the mode change (logged for analysis)
 * 
 * @return true if mode change succeeded or already in requested mode, false if mode change failed
 * 
 * @note This function does not throw exceptions - use return value to detect failures
 */
bool ToyMode::set_and_remember_mode(Mode::Number mode, ModeReason reason)
{
    if (copter.flightmode->mode_number() == mode) {
        // Already in requested mode - success
        return true;
    }
    if (!copter.set_mode(mode, reason)) {
        // Mode change failed (may be due to GPS requirements, arming checks, etc.)
        return false;
    }
    // Remember this mode for future reference
    last_set_mode = mode;
    return true;
}

/**
 * @brief Automatic RC stick trimming for transmitter calibration
 * 
 * @details Implements automatic stick centering detection and trim adjustment.
 *          When the vehicle is disarmed and RC sticks remain near center (within
 *          trim_auto PWM range) without movement for 4 seconds, the system assumes
 *          the sticks are physically centered and updates the trim values. This
 *          compensates for transmitter calibration drift or initial miscalibration.
 *          
 *          Algorithm:
 *          1. Check all 4 RC channels (roll, pitch, throttle, yaw)
 *          2. Verify all channels within trim_auto range of 1500μs (typically ±50μs)
 *          3. Monitor for stick movement (>2μs change)
 *          4. After 4 seconds of stability, save new trim values
 *          
 *          Special handling for throttle:
 *          - Throttle mid-point is tracked separately for altitude hold modes
 *          - Allows throttle stick to be set to hover point, not necessarily center
 * 
 * @note Only operates while disarmed to prevent in-flight trim changes
 * @warning Saved trim values persist across power cycles via AP_Param storage
 */
void ToyMode::trim_update(void)
{
    if (hal.util->get_soft_armed() || !rc().has_valid_input()) {
        // only when disarmed and with RC link
        trim.start_ms = 0;
        return;
    }

    // get throttle mid from channel trim
    uint16_t throttle_trim = copter.channel_throttle->get_radio_trim();
    if (abs(throttle_trim - 1500) <= trim_auto) {
        RC_Channel *c = copter.channel_throttle;
        uint16_t ch_min = c->get_radio_min();
        uint16_t ch_max = c->get_radio_max();
        // remember the throttle midpoint
        int16_t new_value = 1000UL * (throttle_trim - ch_min) / (ch_max - ch_min);
        if (new_value != throttle_mid) {
            throttle_mid = new_value;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: thr mid %d",
                                             throttle_mid);
        }
    }
    
    uint16_t chan[4];
    if (rc().get_radio_in(chan, 4) != 4) {
        trim.start_ms = 0;
        return;
    }

    const uint16_t noise_limit = 2;
    for (uint8_t i=0; i<4; i++) {
        if (abs(chan[i] - 1500) > trim_auto) {
            // not within limit
            trim.start_ms = 0;
            return;
        }
    }

    uint32_t now = AP_HAL::millis();
    
    if (trim.start_ms == 0) {
        // start timer
        memcpy(trim.chan, chan, 4*sizeof(uint16_t));
        trim.start_ms = now;
        return;
    }

    
    for (uint8_t i=0; i<4; i++) {
        if (abs(trim.chan[i] - chan[i]) > noise_limit) {
            // detected stick movement
            memcpy(trim.chan, chan, 4*sizeof(uint16_t));
            trim.start_ms = now;
            return;
        }
    }

    if (now - trim.start_ms < 4000) {
        // not steady for long enough yet
        return;
    }

    // reset timer so we don't trigger too often
    trim.start_ms = 0;
    
    uint8_t need_trim = 0;
    for (uint8_t i=0; i<4; i++) {
        RC_Channel *c = RC_Channels::rc_channel(i);
        if (c && abs(chan[i] - c->get_radio_trim()) > noise_limit) {
            need_trim |= 1U<<i;
        }
    }
    if (need_trim == 0) {
        return;
    }
    for (uint8_t i=0; i<4; i++) {
        if (need_trim & (1U<<i)) {
            RC_Channel *c = RC_Channels::rc_channel(i);
            c->set_and_save_radio_trim(chan[i]);
        }
    }

    gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: trim %u %u %u %u",
                                     chan[0], chan[1], chan[2], chan[3]);
}

/**
 * @brief Handle arming action with safety checks and mode-appropriate configuration
 * 
 * @details Attempts to arm the vehicle with appropriate checks based on flight mode
 *          requirements. Implements several safety features:
 *          - Stick centering check to prevent TX calibration issues
 *          - Compass health verification and in-flight learning enablement
 *          - GPS-dependent arming for modes that require position hold
 *          - Automatic geofence management (enabled for GPS modes, disabled otherwise)
 *          
 *          Arming behavior adapts to current flight mode:
 *          - GPS modes (LOITER, RTL, etc.): Requires GPS lock and enables geofence
 *          - Non-GPS modes (ALT_HOLD, STABILIZE): Arms without GPS, fence disabled
 * 
 * @note Called by ACTION_ARM_LAND_RTL button action when vehicle is disarmed
 * @warning Compass offsets are checked and in-flight learning may be enabled automatically
 */
void ToyMode::action_arm(void)
{
    // Determine if current flight mode requires GPS lock
    bool needs_gps = copter.flightmode->requires_GPS();

    // Safety check: Verify all control sticks are in center deadzone
    // This prevents arming if TX potentiometers are out of calibration or damaged,
    // which could cause immediate uncommanded movement upon arming
    bool sticks_centered =
        copter.channel_roll->get_control_in() == 0 &&
        copter.channel_pitch->get_control_in() == 0 &&
        copter.channel_yaw->get_control_in() == 0;

    if (!sticks_centered) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: sticks not centered");
        return;
    }

    // Check compass health and enable in-flight learning if needed
    arm_check_compass();
    
    // GPS mode arming path: Requires GPS lock and passing pre-arm checks
    if (needs_gps && copter.arming.gps_checks(false)) {
#if AP_FENCE_ENABLED
        // Enable geofencing for GPS modes to prevent flyaways
        copter.fence.enable(true, AC_FENCE_ALL_FENCES);
#endif
        // Attempt to arm with TOYMODE method identifier
        copter.arming.arm(AP_Arming::Method::TOYMODE);
        if (!copter.motors->armed()) {
            // Arming failed despite GPS checks passing - trigger notification
            AP_Notify::events.arming_failed = true;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS armed motors");
        }
    } else if (needs_gps) {
        // GPS mode requested but GPS checks failed (no lock, bad accuracy, etc.)
        AP_Notify::events.arming_failed = true;
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
    } else {
        // Non-GPS mode arming path (ALT_HOLD, STABILIZE, etc.)
#if AP_FENCE_ENABLED
        // Disable geofence for non-GPS modes (can't enforce position limits without GPS)
        copter.fence.enable(false, AC_FENCE_ALL_FENCES);
#endif
        // Attempt to arm without GPS requirements
        copter.arming.arm(AP_Arming::Method::TOYMODE);
        if (!copter.motors->armed()) {
            // Arming failed - could be battery, IMU, or other pre-arm check
            AP_Notify::events.arming_failed = true;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: non-GPS arming failed");
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: non-GPS armed motors");
        }
    }
}

/**
 * @brief Adjust throttle output to prevent sudden climbs and limit descent rate
 * 
 * @details Implements two safety features:
 *          1. Soft-start limiting after throttle-based arming to prevent sudden climbs
 *          2. Automatic descent rate limiting near the ground to prevent hard landings
 *          
 *          Soft-start phase: For 5 seconds after arming via throttle, output is ramped
 *          from 600+deadzone to 1000 following a linear profile. This gives the pilot
 *          time to establish hover before full throttle authority is available.
 *          
 *          Descent limiting: Below TOY_DESCENT_SLOW_HEIGHT (5m), throttle output is
 *          limited to slow descent rate. A ramp zone (TOY_DESCENT_SLOW_RAMP = 3m)
 *          provides smooth transition. Minimum throttle is limited to TOY_DESCENT_SLOW_MIN
 *          (300 = 30% thrust) to prevent hard impacts.
 * 
 * @param[in,out] throttle_control Throttle value in range 0-1000, modified by this function
 * 
 * @note Called during manual flight modes (ALT_HOLD, LOITER when throttle active)
 * @warning Descent limiting only works when relative altitude estimate is available
 */
void ToyMode::throttle_adjust(float &throttle_control)
{
    uint32_t now = AP_HAL::millis();
    const uint32_t soft_start_ms = 5000;  // 5 second ramp period
    const uint16_t throttle_start = 600 + copter.g.throttle_deadzone;  // Starting thrust level
    
    // Soft-start phase 1: Before arming with throttle-arm feature enabled
    if (!copter.motors->armed() && (flags & FLAG_THR_ARM)) {
        // Limit pre-arm throttle to prevent accidental high-throttle arming
        throttle_control = MIN(throttle_control, 500);
    } else if (now - throttle_arm_ms < soft_start_ms) {
        // Soft-start phase 2: Ramp throttle over 5 seconds after arming
        // Progress from 0.0 (just armed) to 1.0 (5 seconds elapsed)
        float p = (now - throttle_arm_ms) / float(soft_start_ms);
        // Linear interpolation: start at throttle_start, ramp to full 1000
        throttle_control = MIN(throttle_control, throttle_start + p * (1000 - throttle_start));
    }

    // Descent rate limiting near ground - early exit checks first
    float pos_d_m;  // Position down in meters (NED frame, positive = down)
    if (AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {  
        // No altitude estimate available - can't implement descent limiting safely
        return;  
    }  
    if (throttle_control >= 500) {  
        // Pilot commanding hover or climb (throttle > 50%) - don't interfere
        return;  
    }  
    // Calculate height above arming point (negative pos_d_m = above origin)
    const float height = -pos_d_m - copter.arming_altitude_m;  
    if (height >= TOY_DESCENT_SLOW_HEIGHT + TOY_DESCENT_SLOW_RAMP) {  
        // Above 8m (5m threshold + 3m ramp) - no descent limiting needed
        return;  
    }  
    if (!copter.motors->armed()) {  
        // Disarmed - don't modify throttle
        return;  
    }  
    if (copter.ap.land_complete) {  
        // Already on ground - no need to limit descent
        return;  
    }  
    
    // Calculate throttle limit based on height using linear interpolation
    // At 5m: limit = TOY_DESCENT_SLOW_MIN (300)
    // At 8m: limit = 0 (no limiting)
    // Between: linear ramp
    const float limit = linear_interpolate(TOY_DESCENT_SLOW_MIN, 0, height,  
                                            TOY_DESCENT_SLOW_HEIGHT,  
                                            TOY_DESCENT_SLOW_HEIGHT+TOY_DESCENT_SLOW_RAMP);  
    if (throttle_control >= limit) {  
        // Pilot commanding more throttle than the limit - allow it
        return;  
    }  
    
    // Pilot commanding descent below safe limit - override to safe descent rate
    throttle_control = limit; 
}


/**
 * @brief Update LED blink patterns for visual feedback to pilot
 * 
 * @details Implements a 16-bit pattern-based LED control system using two relays
 *          (relay 0 = green LED, relay 1 = red LED). Each bit position in the
 *          16-bit pattern controls whether the LED is on or off for one update cycle.
 *          
 *          Pattern system:
 *          - Each pattern is a 16-bit value (e.g., 0xFFFF = solid on, 0xAAAA = blink)
 *          - Pattern index increments each update (10Hz), wrapping at 16
 *          - Optional pattern count makes temporary patterns persist for N cycles
 *          
 *          Default patterns based on vehicle state:
 *          - Normal: BLINK_FULL (solid on when TX connected)
 *          - Battery failsafe: BLINK_8 (rapid blink)
 *          - No RX: BLINK_NO_RX pattern on both LEDs
 *          
 *          Special patterns can be set via MAVLink (BLINKR, BLINKG, VNOTIFY messages)
 *          for camera/video feedback or other notifications.
 * 
 * @note Called at 10Hz from update() function
 * @warning LED control uses relays 0 and 1 - ensure board has these configured
 */
void ToyMode::blink_update(void)
{
    // Update red LED (relay 1) based on current bit in the 16-bit pattern
    // Bit is tested at current index position; 1=LED on, 0=LED off
    if (red_blink_pattern & (1U<<red_blink_index)) {
        copter.relay.on(1);
    } else {
        copter.relay.off(1);
    }
    
    // Update green LED (relay 0) based on current bit in the 16-bit pattern
    if (green_blink_pattern & (1U<<green_blink_index)) {
        copter.relay.on(0);
    } else {
        copter.relay.off(0);
    }
    
    // Advance pattern indices (wrap at 16 for 16-bit patterns)
    // Called at 10Hz, so full pattern cycle takes 1.6 seconds
    green_blink_index = (green_blink_index+1) % 16;
    red_blink_index = (red_blink_index+1) % 16;
    
    // Decrement pattern persistence counters when pattern completes one full cycle
    // When count reaches 0, pattern reverts to default system status pattern
    if (green_blink_index == 0 && green_blink_count > 0) {
        green_blink_count--;
    }
    if (red_blink_index == 0 && red_blink_count > 0) {
        red_blink_count--;
    }

    // Update video recording flag for transmitter OSD display
    // Flag is set when VNOTIFY message received within last 1 second
    uint32_t now = AP_HAL::millis();
    if (now - last_video_ms < 1000) {
        AP_Notify::flags.video_recording = true;
    } else {
        AP_Notify::flags.video_recording = false;
    }
    
    // If both LEDs have active pattern counts, don't override with default patterns
    // This allows custom MAVLink-commanded patterns to persist
    if (red_blink_count > 0 && green_blink_count > 0) {
        return;
    }
    
    // Calculate default pattern based on vehicle state
    // Patterns use 16-bit values where each bit controls one LED update cycle
    uint16_t pattern = 0;

    // Battery failsafe: rapid blink (8 on/off cycles) to alert pilot
    if (copter.motors->armed() && AP_Notify::flags.failsafe_battery) {
        pattern = BLINK_8;
    } else if (!copter.motors->armed() && (blink_disarm > 0)) {
        // Brief rapid blink after disarm for visual confirmation (4 pattern cycles ~6.4s)
        pattern = BLINK_8;
        blink_disarm--;
    } else {
        // Normal operation: solid on (indicates good TX connection)
        pattern = BLINK_FULL;
    }
    
    // Initialize post-disarm blink counter when vehicle arms
    if (copter.motors->armed()) {
        blink_disarm = 4;  // 4 pattern cycles
    }
    
    // Apply default patterns to LEDs without active custom pattern counts
    if (red_blink_count == 0) {
        red_blink_pattern = pattern;
    }
    if (green_blink_count == 0) {
        green_blink_pattern = pattern;
    }
    
    // Synchronize LED patterns when both revert to default
    // This ensures both LEDs blink in unison for clearer status indication
    if (red_blink_count == 0 && green_blink_count == 0) {
        red_blink_index = green_blink_index;
    }
}

/**
 * @brief Handle MAVLink messages for Toy Mode control and feedback
 * 
 * @details Processes NAMED_VALUE_INT messages from companion computer (e.g., Sonix)
 *          for LED blink patterns, video recording notifications, WiFi channel
 *          control, and logging configuration. This provides a simple command
 *          interface for companion computer to control user-facing features.
 *          
 *          Supported message names:
 *          - BLINKR: Set red LED blink pattern (lower 16 bits) and count (upper 16 bits)
 *          - BLINKG: Set green LED blink pattern (lower 16 bits) and count (upper 16 bits)
 *          - VNOTIFY: Video recording notification (triggers green LED double-blink)
 *          - WIFICHAN: Set WiFi channel for coexistence with RC radio
 *          - LOGDISARM: Enable/disable logging when disarmed
 * 
 * @param[in] msg MAVLink message to process
 * 
 * @note Called from GCS message handling when NAMED_VALUE_INT received
 * @note Assumes messages originate from trusted companion computer
 */
void ToyMode::handle_message(const mavlink_message_t &msg)
{
    // Only process NAMED_VALUE_INT messages
    if (msg.msgid != MAVLINK_MSG_ID_NAMED_VALUE_INT) {
        return;
    }
    
    // Decode message payload
    mavlink_named_value_int_t m;
    mavlink_msg_named_value_int_decode(&msg, &m);
    
    // BLINKR: Set red LED blink pattern from companion computer
    // Value format: lower 16 bits = pattern, upper 16 bits = persistence count
    if (strncmp(m.name, "BLINKR", 10) == 0) {
        red_blink_pattern = (uint16_t)m.value;
        red_blink_count = m.value >> 16;
        red_blink_index = 0;  // Reset to start of pattern
    } 
    // BLINKG: Set green LED blink pattern from companion computer
    else if (strncmp(m.name, "BLINKG", 10) == 0) {
        green_blink_pattern = (uint16_t)m.value;
        green_blink_count = m.value >> 16;
        green_blink_index = 0;
    } 
    // VNOTIFY: Video recording notification from camera controller
    // Triggers visual feedback (double-blink) to confirm recording state change
    else if (strncmp(m.name, "VNOTIFY", 10) == 0) {
        // Reset green LED pattern index if not already showing video pattern
        // This ensures pattern starts from beginning for clear visual indication
        if (green_blink_pattern != BLINK_2) {
            green_blink_index = 0;
        }
        green_blink_pattern = BLINK_2;  // Double-blink pattern
        green_blink_count = 1;
        last_video_ms = AP_HAL::millis();
        // Immediately update AP_Notify recording flag for transmitter OSD
        AP_Notify::flags.video_recording = true;
    } 
    // WIFICHAN: WiFi channel configuration for radio coexistence
    // Companion computer requests autopilot radio to avoid interference
    else if (strncmp(m.name, "WIFICHAN", 10) == 0) {
#if AP_RADIO_ENABLED
        AP_Radio *radio = AP_Radio::get_singleton();
        if (radio) {
            radio->set_wifi_channel(m.value);
        }
#endif
    } 
    // LOGDISARM: Enable/disable logging when disarmed
    // Allows companion computer to control LOG_DISARMED parameter
    else if (strncmp(m.name, "LOGDISARM", 10) == 0) {
        enum ap_var_type vtype;
        AP_Int8 *log_disarmed = (AP_Int8 *)AP_Param::find("LOG_DISARMED", &vtype);
        if (log_disarmed) {
            log_disarmed->set(int8_t(m.value));
        }
    }
}

/**
 * @brief Send a NAMED_VALUE_INT MAVLink message to companion computer
 * 
 * @details Transmits simple command/event messages to companion computer (e.g., Sonix)
 *          via the primary telemetry channel. Used to trigger actions on companion
 *          computer such as taking photos, toggling video recording, or WiFi reset.
 *          
 *          This provides a lightweight command interface without requiring
 *          full MAVLink command protocol support in the companion computer.
 * 
 * @param[in] name Message name (max 10 characters), e.g., "SNAPSHOT", "VIDEOTOG", "WIFIRESET"
 * @param[in] value Integer parameter value (interpretation depends on message name)
 * 
 * @note Sends on MAVLINK_COMM_1 which is typically the primary telemetry port
 * @note Companion computer must monitor NAMED_VALUE_INT messages by name
 */
void ToyMode::send_named_int(const char *name, int32_t value)
{
    mavlink_msg_named_value_int_send(MAVLINK_COMM_1, AP_HAL::millis(), name, value);
}

/**
 * @brief Limit motor thrust based on battery voltage to prevent voltage sag
 * 
 * @details Implements voltage-based thrust limiting to extend flight time and
 *          prevent voltage collapse on weak batteries typical in consumer drones.
 *          As battery voltage drops, maximum thrust is progressively reduced using
 *          linear interpolation between configured voltage thresholds.
 *          
 *          Thrust multiplier calculation:
 *          - Above TMODE_VMAX: Use TMODE_TMIN (minimum/no limiting)
 *          - Below TMODE_VMIN: Use TMODE_TMAX (maximum limiting)
 *          - Between: Linear interpolation
 *          
 *          This callback is registered with the motor library and called during
 *          motor output calculation to modify thrust values before PWM conversion.
 *          
 *          Example: TMODE_VMIN=3.5V, TMODE_VMAX=3.8V, TMODE_TMIN=1.0, TMODE_TMAX=0.7
 *          - At 3.9V: 100% thrust available
 *          - At 3.65V: 85% thrust available (interpolated)
 *          - At 3.4V: 70% thrust available
 * 
 * @param[in,out] thrust Array of thrust values (0.0-1.0) to be limited, one per motor
 * @param[in] num_motors Number of motors in thrust array
 * 
 * @note Called at motor update rate (typically 400Hz) from motor mixing code
 * @note Uses filtered_voltage updated in main update() loop at 10Hz
 * @note Logs thrust data at 40Hz (every 10th call) to avoid overwhelming logger
 * 
 * @warning Aggressive limiting can make vehicle difficult to control in low battery
 * @warning Ensure TMODE_VMIN/VMAX match actual battery characteristics
 */
void ToyMode::thrust_limiting(float *thrust, uint8_t num_motors)
{
    // Calculate thrust multiplier based on current battery voltage
    // Uses linear interpolation: full thrust at high voltage, reduced at low voltage
    // This prevents voltage sag-induced brownouts while allowing full performance when battery is fresh
    float thrust_mul = linear_interpolate(filter.thrust_max, filter.thrust_min, filtered_voltage, filter.volt_min, filter.volt_max);
    
    // Apply thrust multiplier to all motors
    // Reduces maximum available thrust uniformly across all motors
    for (uint8_t i=0; i<num_motors; i++) {
        thrust[i] *= thrust_mul;
    }
    
    // Read current PWM outputs for logging (diagnostic purposes)
    // PWM values show actual motor drive signals after thrust limiting applied
    uint16_t pwm[4];
    hal.rcout->read(pwm, 4);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: THST
// @Description: Maximum thrust limitation based on battery voltage in Toy Mode
// @Field: TimeUS: Time since system startup
// @Field: Vol: Filtered battery voltage
// @Field: Mul: Thrust multiplier between 0 and 1 to limit the output thrust based on battery voltage
// @Field: M1: Motor 1 pwm output
// @Field: M2: Motor 2 pwm output
// @Field: M3: Motor 3 pwm output
// @Field: M4: Motor 4 pwm output

    // Log thrust limiting data at reduced rate (40Hz instead of 400Hz)
    // Decimation reduces log size while still capturing voltage/thrust relationship
    if (motor_log_counter++ % 10 == 0) {
        AP::logger().WriteStreaming("THST", "TimeUS,Vol,Mul,M1,M2,M3,M4", "QffHHHH",
                                               AP_HAL::micros64(),
                                               (double)filtered_voltage,
                                               (double)thrust_mul,
                                               pwm[0], pwm[1], pwm[2], pwm[3]);
    }
#endif
}

#if ENABLE_LOAD_TEST
/**
 * @brief Execute motor load test for factory endurance validation
 * 
 * @details Runs automated motor load test patterns for production testing and
 *          quality assurance. Bypasses normal flight control to directly command
 *          motor outputs according to predefined or constant patterns. Used to
 *          verify motor, ESC, and battery performance under controlled load profiles.
 *          
 *          Test modes:
 *          - LOAD_TYPE_LOG1: Replay recorded flight log PWM patterns to all motors
 *          - LOAD_TYPE_LOG2: Replay flight log, but use same PWM value for all motors
 *          - LOAD_TYPE_CONSTANT: Apply constant 50% throttle to all motors
 *          
 *          The test automatically terminates on battery failsafe to prevent
 *          over-discharge during extended test runs.
 * 
 * @note Called from main loop when load_test.running is true
 * @note Load test initiated via ACTION_LOAD_TEST button action
 * @note TMODE_LOAD_MUL scales output (0.0-1.0) for varying test intensity
 * @note TMODE_LOAD_FILT controls pattern playback speed (1=normal, higher=slower)
 * 
 * @warning Only enabled when ENABLE_LOAD_TEST compile flag is set (disabled by default)
 * @warning Bypasses normal safety checks - for factory use only
 * @warning Mount vehicle securely before running load test
 */
void ToyMode::load_test_run(void)
{
    // Initialize PWM values array
    uint16_t pwm[4] {};
    
    // Select load pattern based on configured test type
    switch ((enum load_type)load_test.load_type.get()) {
    case LOAD_TYPE_LOG1:
        // Replay recorded flight log data with individual motor values
        // Each motor gets unique PWM value from logged flight pattern
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = load_data1[load_test.row].m[i];
        }
        
        // Advance to next row in pattern at filtered rate
        // Filter counter slows down pattern playback for extended test duration
        load_test.filter_counter++;
        if (load_test.filter_counter >= load_test.load_filter.get()) {
            load_test.filter_counter = 0;
            load_test.row = (load_test.row + 1) % ARRAY_SIZE(load_data1);
        }
        break;
        
    case LOAD_TYPE_LOG2:
        // Replay flight log but apply same PWM to all motors
        // Tests symmetric load without differential throttle effects
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = load_data1[load_test.row].m[0];
        }
        load_test.filter_counter++;
        if (load_test.filter_counter >= load_test.load_filter.get()) {
            load_test.filter_counter = 0;
            load_test.row = (load_test.row + 1) % ARRAY_SIZE(load_data1);
        }
        break;

    case LOAD_TYPE_CONSTANT:
        // Apply constant 50% throttle equivalent to all motors
        // Simplest test mode for basic endurance checking
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = 500;  // 500 = 50% in 0-1000 range
        }
        break;
        
    default:
        return;
    }
    
    // Scale PWM values by load multiplier and output to motors
    // Multiplier allows intensity adjustment without changing pattern
    for (uint8_t i=0; i<4; i++) {
        pwm[i] *= load_test.load_mul;
        // Convert from 0-1000 range to standard 1000-2000 PWM range
        hal.rcout->write(i, 1000 + pwm[i]*2);
    }

    // Safety check: Terminate test on battery failsafe
    // Prevents battery over-discharge during extended test runs
    if (copter.failsafe.battery) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test off (battery)");
        copter.init_disarm_motors();
        load_test.running = false;
    }    
}
#endif // ENABLE_LOAD_TEST

/**
 * @brief Check compass calibration and enable in-flight learning if needed
 * 
 * @details Validates compass calibration state before arming and automatically
 *          enables in-flight compass learning if calibration appears poor or
 *          incomplete. This provides automatic recovery for consumer drones that
 *          may not have been properly calibrated by end users.
 *          
 *          Compass health checks:
 *          - Offset magnitude: Must be within compass_offsets_max limit
 *          - Field strength: Must be 200-800 milligauss (Earth's field is ~250-650mG)
 *          - Configuration: Compass must pass internal configuration checks
 *          
 *          If any check fails, enables INFLIGHT learning mode which:
 *          - Continuously updates compass offsets during flight
 *          - Uses EKF innovation consistency to validate corrections
 *          - Gradually improves heading accuracy over multiple flights
 *          
 *          Typical failure scenarios triggering auto-learning:
 *          - Factory default offsets (no user calibration performed)
 *          - Metallic interference from new payload/modifications
 *          - Temperature-dependent offset drift
 *          - Compass mounted in high-EMI environment
 * 
 * @note Called before every arm attempt in action_arm()
 * @note In-flight learning takes 2-5 minutes to converge
 * @note Learning persists across power cycles until calibration acceptable
 * 
 * @warning In-flight learning requires GPS and extended flight time
 * @warning Initial heading accuracy may be poor until learning converges
 * @warning Avoid aggressive maneuvers during first minute after enabling learning
 */
void ToyMode::arm_check_compass(void)
{
    // Get current compass calibration offsets
    // Offsets compensate for hard-iron interference from vehicle frame/electronics
    Vector3f offsets = copter.compass.get_offsets();
    
    // Get current magnetic field strength measurement
    // Should be close to Earth's magnetic field (250-650 milligauss depending on location)
    float field = copter.compass.get_field().length();
    
    // Check if compass calibration is acceptable
    // Buffer for error message (unused, but required by API)
    char unused_compass_configured_error_message[20];
    
    // Determine if compass calibration is poor or missing
    // Triggers learning if: offsets too large, field strength wrong, or not configured
    if (offsets.length() > copter.compass.get_offsets_max() ||
        field < 200 || field > 800 ||
        !copter.compass.configured(unused_compass_configured_error_message, ARRAY_SIZE(unused_compass_configured_error_message))) {
        
        // Enable in-flight learning if not already active
        // This allows compass to self-calibrate during flight using EKF innovations
        if (copter.compass.get_learn_type() != Compass::LearnType::INFLIGHT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: enable compass learning");
            copter.compass.set_learn_type(Compass::LearnType::INFLIGHT, false);
        }
    }
}

#endif // TOY_MODE_ENABLED
