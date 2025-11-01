#pragma once

#include <stdint.h>
#include <AP_Param/AP_Param.h>
#include "mode.h"

/**
 * @file toy_mode.h
 * @brief ToyMode system for simplified user interface on consumer quadcopters
 * 
 * @details This file implements the ToyMode system, which provides a highly simplified
 *          user interface designed specifically for large-volume consumer/toy quadcopters.
 *          The system is optimized for non-expert users who need intuitive single-button
 *          control without requiring knowledge of RC transmitters or flight modes.
 * 
 *          Key Features:
 *          - Single-button or dual-button operation for complete vehicle control
 *          - Automatic mode transitions based on flight state
 *          - Simplified arming/disarming via throttle stick gestures
 *          - Automatic altitude hold and position hold upgrades
 *          - Built-in safety features including automatic RTL on signal loss
 *          - Visual feedback through LED blink patterns
 *          - Automatic trim calibration on first flight
 * 
 *          Hardware Support:
 *          - V2450 button configuration (TMODE_ENABLE=1)
 *          - F412 button configuration (TMODE_ENABLE=2)
 *          - Compatible with minimal RC channels (4-5 channels sufficient)
 * 
 *          Typical Usage Workflow:
 *          1. User places quadcopter on level surface
 *          2. User arms via high throttle stick gesture
 *          3. System automatically calibrates trim on first arm
 *          4. Throttle controls altitude (automatic altitude hold)
 *          5. Optional automatic upgrade to loiter mode with GPS
 *          6. Button presses trigger photo/video or mode changes
 *          7. RTL automatically engaged on signal loss or low battery
 *          8. Disarm via low throttle stick gesture when landed
 * 
 *          Safety Features:
 *          - Automatic disarm on sustained low throttle
 *          - RTL cancellation on large stick input (pilot override)
 *          - Compass health checks before arming
 *          - Motor thrust limiting based on battery voltage
 *          - Automatic mode fallbacks on sensor failures
 * 
 * @note This system is designed for consumer products where ease of use
 *       is prioritized over manual control authority. It is NOT recommended
 *       for expert pilots or applications requiring precise manual control.
 * 
 * @warning The simplified interface reduces pilot authority in exchange for
 *          ease of use. Ensure all safety features are properly configured
 *          and tested before deployment in consumer products.
 * 
 * @see Copter::toy_mode
 * @see Mode class for underlying flight mode implementations
 */

/**
 * @class ToyMode
 * @brief Simplified user interface system for consumer/toy quadcopters
 * 
 * @details The ToyMode class implements a complete simplified flight interface
 *          designed for mass-market consumer quadcopters targeting non-expert users.
 *          Unlike traditional ArduCopter operation which requires understanding of
 *          multiple flight modes and RC transmitter operation, ToyMode provides
 *          intuitive single-button or dual-button control with automatic mode
 *          transitions and built-in safety features.
 * 
 *          State Machine Overview:
 *          The system operates through a simplified state machine:
 *          - DISARMED → ARM (via high throttle gesture or button)
 *          - ARMED → AUTO_TAKEOFF/ALTHOLD (automatic based on configuration)
 *          - ALTHOLD → LOITER (automatic upgrade when GPS available)
 *          - FLIGHT → RTL (on signal loss, low battery, or button press)
 *          - RTL → LAND (automatic when home reached)
 *          - LAND → DISARMED (automatic on touchdown + low throttle)
 * 
 *          Button Press Patterns:
 *          - Short press (< 1 sec): Take photo or toggle video recording
 *          - Medium press (1-2 sec): Mode toggle (configurable modes)
 *          - Long press (> 2 sec): RTL or emergency land
 *          - Dual button combinations: Additional actions (model specific)
 * 
 *          Automatic Mode Transitions:
 *          - Automatic upgrade from ALTHOLD to LOITER when GPS lock achieved
 *          - Automatic RTL on RC signal loss (if GPS available)
 *          - Automatic LAND if RTL not available (no GPS)
 *          - Automatic disarm after landing with low throttle
 * 
 *          Throttle Stick Gestures:
 *          - High throttle (> 90%) for 2 seconds → ARM
 *          - Low throttle (< 10%) for 2 seconds when landed → DISARM
 *          - Throttle in middle range → Normal altitude control
 * 
 *          Safety Features:
 *          - Automatic compass health checks before arming
 *          - Battery voltage based thrust limiting to prevent brown-out
 *          - Automatic RTL on signal loss or low battery
 *          - Large stick input cancels RTL (pilot override)
 *          - Automatic trim calibration on first flight
 *          - Configurable safety flags for arm/disarm behavior
 * 
 *          LED Blink Patterns (Visual Feedback):
 *          - Solid: Armed and flying
 *          - Slow blink: GPS searching
 *          - Fast blink: Ready to arm
 *          - Pattern blinks: Error conditions or mode indicators
 * 
 *          Hardware Requirements:
 *          - Minimum 4-channel RC receiver (throttle, roll, pitch, yaw)
 *          - Optional 5th channel for button/mode input
 *          - GPIO pins for button inputs (platform dependent)
 *          - LED outputs for visual feedback (platform dependent)
 *          - GPS recommended for position hold and RTL features
 *          - Compass required for yaw control and RTL
 * 
 *          Configuration Parameters:
 *          - TMODE_ENABLE: Enable/disable and select button configuration (0=off, 1=V2450, 2=F412)
 *          - TMODE_PRI_MODE: Primary flight modes accessible via button toggle
 *          - TMODE_ACT1-9: Configurable actions for various button patterns
 *          - TMODE_TRIM_AUTO: Enable automatic trim calibration
 *          - TMODE_FLAGS: Safety behavior flags (arm/disarm conditions, RTL cancel, etc.)
 * 
 *          Typical Consumer Use Case:
 *          1. User places quadcopter on flat surface outdoors
 *          2. User pushes throttle stick to maximum for 2 seconds (arms)
 *          3. LEDs indicate armed status, system enters ALTHOLD mode
 *          4. User raises throttle to take off, system maintains altitude automatically
 *          5. GPS lock achieved, system upgrades to LOITER for position hold
 *          6. User presses button to take photo
 *          7. User releases sticks, quadcopter holds position
 *          8. User long-presses button to trigger RTL
 *          9. Quadcopter flies home and lands automatically
 *          10. User lowers throttle to minimum for 2 seconds (disarms)
 * 
 * @note This system prioritizes ease of use over control authority. Expert pilots
 *       may find the automatic behaviors restrictive. For professional or research
 *       applications, standard ArduCopter modes are recommended.
 * 
 * @warning The automatic mode transitions can override pilot inputs in certain
 *          conditions. Thoroughly test all behaviors in SITL simulation before
 *          deploying to physical hardware. Ensure all safety features (geofencing,
 *          battery monitoring) are properly configured.
 * 
 * @warning Compass calibration is critical for this system. Poor compass health
 *          will prevent arming and cause navigation failures in LOITER and RTL modes.
 * 
 * Source: ArduCopter/toy_mode.h:12-185, ArduCopter/toy_mode.cpp
 */
class ToyMode
{
public:
    friend class Copter;

    /**
     * @brief Constructor for ToyMode system
     * 
     * @details Initializes the ToyMode system with default parameter values and
     *          resets all state variables. Does not perform hardware initialization;
     *          that occurs during the first update() call.
     */
    ToyMode();
    
    /**
     * @brief Check if ToyMode is enabled
     * 
     * @return true if ToyMode is enabled (TMODE_ENABLE != 0), false otherwise
     * 
     * @note This should be checked before calling update() or other ToyMode methods
     */
    bool enabled(void) const {
        return enable.get() != 0;
    }
    
    /**
     * @brief Main update function called every scheduler loop
     * 
     * @details This is the primary ToyMode state machine update function, called at
     *          the main loop rate (typically 400Hz). It performs:
     *          - Button input processing and debouncing
     *          - Throttle gesture detection (arm/disarm patterns)
     *          - Automatic mode transitions based on flight state
     *          - Trim calibration updates
     *          - LED blink pattern updates
     *          - Safety checks and failsafe monitoring
     *          - Battery voltage filtering for thrust limiting
     * 
     *          State Machine Processing:
     *          - Checks for arming conditions (throttle high + safety checks)
     *          - Monitors for disarming conditions (throttle low + landed)
     *          - Triggers automatic LOITER upgrade when GPS available
     *          - Processes button presses and executes configured actions
     *          - Updates LED feedback patterns based on vehicle state
     * 
     * @note Called at high frequency; all operations must be computationally efficient
     * @note Only active when enabled() returns true
     * 
     * @warning This function can trigger mode changes and arming state changes
     *          based on stick positions and button inputs. Ensure input filtering
     *          is properly configured to prevent spurious transitions.
     * 
     * @see enabled()
     * @see trim_update()
     * @see blink_update()
     */
    void update(void);

    /**
     * @brief Get the current throttle mid-point value
     * 
     * @return Throttle mid-point in PWM units (typically around 500, range 0-1000)
     * 
     * @details The throttle mid-point is dynamically calculated during flight to
     *          represent the hover throttle for the vehicle. This value is used
     *          for automatic altitude hold and smooth takeoff transitions.
     *          
     *          The mid-point is learned during the first few flights and adapts to:
     *          - Vehicle weight and configuration
     *          - Battery voltage (decreases as battery drains)
     *          - Altitude and air density
     * 
     * @note Initial default value is 500 (50% throttle)
     * @note Value is continuously updated during stabilized flight
     * 
     * @see throttle_adjust()
     */
    int16_t get_throttle_mid(void) {
        return throttle_mid;
    }

    /**
     * @brief Adjust throttle output for automatic takeoff and thrust limiting
     * 
     * @param[in,out] throttle_control Throttle control value (0.0 to 1.0) to be adjusted
     * 
     * @details This function applies two primary adjustments to the throttle:
     *          
     *          1. Automatic Takeoff Ramping:
     *             During the initial seconds after arming, throttle is smoothly ramped
     *             up to prevent sudden jumps that could destabilize the vehicle.
     *             This provides gentle, predictable takeoffs for inexperienced users.
     *          
     *          2. Battery Voltage Based Thrust Limiting:
     *             As battery voltage decreases, maximum thrust is progressively limited
     *             to prevent voltage sag and brown-outs that could cause loss of control.
     *             The limiting is based on filtered battery voltage and configured
     *             voltage/thrust curves (TMODE_FILT_VMIN, VMAX, TMIN, TMAX).
     *          
     *          The function modifies the throttle_control parameter in-place, scaling
     *          it according to the current flight phase and battery state.
     * 
     * @note Called from motor mixing code before output to ESCs
     * @note Thrust limiting is critical for preventing brown-out crashes on low battery
     * 
     * @warning Do not bypass this function in consumer applications; the thrust limiting
     *          prevents dangerous low-voltage conditions
     * 
     * @see get_throttle_mid()
     * @see thrust_limiting()
     */
    void throttle_adjust(float &throttle_control);

    /**
     * @brief Handle incoming MAVLink messages for ToyMode control
     * 
     * @param[in] msg MAVLink message to process
     * 
     * @details Processes MAVLink messages relevant to ToyMode operation:
     *          - NAMED_VALUE_INT: Receive commands and status updates from ground station
     *          - CAMERA_FEEDBACK: Track video recording state for LED feedback
     *          - Custom ToyMode control messages for button simulation
     *          
     *          Supported NAMED_VALUE_INT commands (by name):
     *          - "toy_action": Trigger specific ToyMode action remotely
     *          - "toy_test": Initiate load testing sequence
     *          - Status values sent: mode state, button press counts, battery status
     *          
     *          This allows ground station or companion computer control of ToyMode
     *          features, enabling testing, monitoring, and advanced integrations.
     * 
     * @note Only processes messages when ToyMode is enabled
     * @note Used for both receiving commands and sending status telemetry
     * 
     * @see send_named_int()
     * @see toy_action enum for available actions
     */
    void handle_message(const mavlink_message_t &msg);

    /**
     * @brief Execute motor load test sequence
     * 
     * @details Runs a predefined motor load profile for hardware validation and
     *          quality assurance testing in manufacturing or maintenance. The test
     *          applies controlled thrust patterns to all motors and logs the results.
     *          
     *          Test Sequence:
     *          1. Arms vehicle (bypasses normal arming checks)
     *          2. Applies load pattern from load_data1[] table
     *          3. Logs motor thrust, battery voltage, and current draw
     *          4. Disarms after test completion
     *          
     *          Supported Load Types (TMODE_LOAD_TYPE):
     *          - LOAD_TYPE_CONSTANT: Fixed thrust level
     *          - LOAD_TYPE_LOG1: Logged profile pattern 1
     *          - LOAD_TYPE_LOG2: Logged profile pattern 2
     *          
     *          Test Parameters:
     *          - TMODE_LOAD_MUL: Multiplier for load values (safety scaling)
     *          - TMODE_LOAD_FILT: Filtering level for load application
     * 
     * @warning This bypasses normal safety checks and should ONLY be used in
     *          controlled manufacturing/test environments with propellers removed
     *          or suitable safety containment.
     * 
     * @warning DO NOT run load tests with propellers installed or vehicle unsecured.
     *          The test applies significant motor thrust without stabilization.
     * 
     * @note Triggered via MAVLink command or configured button action
     * @note Test results logged to dataflash for analysis
     * 
     * @see load_data struct
     * @see load_type enum
     */
    void load_test_run(void);
    
    /**
     * @brief Parameter metadata for ToyMode configuration
     * 
     * @details Defines all user-configurable parameters for ToyMode system:
     *          - enable: ToyMode enable and button type selection
     *          - primary_mode[]: Primary flight modes for toggle
     *          - actions[]: Button action mappings
     *          - trim_auto: Automatic trim calibration enable
     *          - flags: Behavior flags (arm/disarm/RTL/loiter)
     *          - filter parameters: Battery voltage thrust limiting curves
     *          - load_test parameters: Manufacturing test configuration
     * 
     * @note These parameters are stored in EEPROM and configurable via ground station
     * @see AP_Param for parameter system documentation
     */
    static const struct AP_Param::GroupInfo var_info[];
    
private:

    /**
     * @brief Update automatic trim calibration
     * 
     * @details Performs automatic RC trim calibration during the first few seconds
     *          of flight after arming. Samples RC input while vehicle is hovering
     *          and stores neutral stick positions as trim values. This eliminates
     *          the need for manual trim adjustment by inexperienced users.
     *          
     *          Calibration process:
     *          - Activated on first arm if TMODE_TRIM_AUTO is enabled
     *          - Samples roll, pitch, yaw, and throttle inputs for ~3 seconds
     *          - Averages samples to find neutral positions
     *          - Applies trim offsets to subsequent inputs
     *          - Stores trim values for future flights
     * 
     * @note Only active during initial learning period after first arm
     * @note User should keep sticks centered during calibration
     */
    void trim_update(void);
    
    /**
     * @brief Execute arming sequence with ToyMode safety checks
     * 
     * @details Initiates vehicle arming with ToyMode-specific checks and setup:
     *          - Verifies compass health (critical for position modes)
     *          - Checks battery voltage is adequate
     *          - Ensures vehicle is on level surface
     *          - Sets initial flight mode (typically ALTHOLD)
     *          - Starts automatic trim calibration if configured
     *          - Initializes throttle mid-point tracking
     *          
     *          This provides additional safety validation beyond standard ArduCopter
     *          arming checks, specifically for consumer applications.
     * 
     * @note Called when throttle high gesture detected or arm button pressed
     * @see arm_check_compass()
     */
    void action_arm(void);
    
    /**
     * @brief Update LED blink patterns for visual feedback
     * 
     * @details Manages red and green LED outputs to provide visual status feedback
     *          to the user. LED patterns indicate vehicle state, GPS status, battery
     *          level, recording status, and error conditions.
     *          
     *          Pattern meanings:
     *          - Solid green: Armed and ready, good GPS
     *          - Slow green blink: GPS searching
     *          - Fast green blink: Ready to arm
     *          - Red blink count: Battery cell count or error code
     *          - Alternating: Video recording active
     *          - Special patterns: Various error conditions
     *          
     *          Patterns are defined in blink_patterns enum and updated at ~10Hz.
     * 
     * @note Called from main update() function
     * @note LED outputs are platform-specific (GPIO pins configured in hwdef)
     * 
     * @see blink_patterns enum
     */
    void blink_update(void);
    
    /**
     * @brief Send named integer value via MAVLink telemetry
     * 
     * @param[in] name Name string for the value (max 10 characters)
     * @param[in] value Integer value to send
     * 
     * @details Sends a NAMED_VALUE_INT MAVLink message for telemetry and debugging.
     *          Used to report ToyMode status to ground station or companion computer:
     *          - Button press counters
     *          - Current action states
     *          - Mode transition events
     *          - Error conditions
     *          - Test results
     * 
     * @note Message rate limited to avoid telemetry congestion
     * @see handle_message()
     */
    void send_named_int(const char *name, int32_t value);
    
    /**
     * @brief Set flight mode and remember it for future reference
     * 
     * @param[in] mode Flight mode to set (Mode::Number enum)
     * @param[in] reason Reason for mode change (for logging)
     * @return true if mode change successful, false if rejected
     * 
     * @details Attempts to change flight mode and, if successful, stores the mode
     *          for future restore operations (e.g., after RTL completes). This allows
     *          ToyMode to return to the user's selected mode after automatic safety
     *          mode interventions.
     *          
     *          Mode change may be rejected if:
     *          - Required sensors not available (e.g., GPS for LOITER)
     *          - Vehicle state not appropriate (e.g., not armed)
     *          - Mode not configured/compiled in
     * 
     * @note Internally calls Copter::set_mode()
     * @see Mode::Number enum for available modes
     */
    bool set_and_remember_mode(Mode::Number mode, ModeReason reason);

    /**
     * @brief Apply thrust limiting to motor outputs based on battery voltage
     * 
     * @param[in,out] thrust Array of thrust values (0.0 to 1.0) for each motor
     * @param[in] num_motors Number of motors in the array
     * 
     * @details Scales motor thrust outputs based on filtered battery voltage to
     *          prevent brown-out conditions. As battery voltage decreases, maximum
     *          available thrust is progressively reduced using configured curves:
     *          
     *          Voltage to thrust mapping:
     *          - Above TMODE_FILT_VMAX: No limiting (100% thrust available)
     *          - TMODE_FILT_VMAX to VMIN: Linear scaling between TMAX and TMIN
     *          - Below TMODE_FILT_VMIN: Maximum limiting (TMIN thrust available)
     *          
     *          This prevents the sudden voltage sag that occurs when drawing too much
     *          current from a depleted battery, which could cause loss of control or
     *          flight controller reboot.
     * 
     * @note Applied to all motor outputs before sending to ESCs
     * @note Critical safety feature for consumer applications
     * 
     * @warning Do not disable thrust limiting on battery-powered vehicles
     * 
     * @see throttle_adjust()
     */
    void thrust_limiting(float *thrust, uint8_t num_motors);
    
    /**
     * @brief Check compass health before allowing arming
     * 
     * @details Performs comprehensive compass health validation:
     *          - Checks compass calibration status
     *          - Verifies compass variance is within acceptable limits
     *          - Ensures compass is not failing consistency checks
     *          - Validates compass offsets are reasonable
     *          
     *          Compass health is critical for ToyMode because the system relies on
     *          automatic modes (LOITER, RTL) that require accurate heading information.
     *          Poor compass health will cause navigation failures and loss of control.
     *          
     *          If compass health check fails:
     *          - Arming is prevented
     *          - Error indicated via LED blink pattern
     *          - User must recalibrate compass before flight
     * 
     * @note Called from action_arm() before arming sequence
     * @note More strict than standard ArduCopter checks for consumer safety
     * 
     * @warning Compass calibration is CRITICAL for safe operation in position modes
     */
    void arm_check_compass(void);

    /**
     * @brief Check if V2450 button configuration is active
     * @return true if TMODE_ENABLE=1 (V2450 button hardware)
     * @note V2450 uses specific GPIO pins for button inputs
     */
    bool is_v2450_buttons(void) const {
        return enable == 1;
    }
    
    /**
     * @brief Check if F412 button configuration is active
     * @return true if TMODE_ENABLE=2 (F412 button hardware)
     * @note F412 uses alternative GPIO pins for button inputs
     */
    bool is_f412_buttons(void) const {
        return enable == 2;
    }
    
    /**
     * @enum toy_action
     * @brief Actions triggered by button presses or programmatic commands
     * 
     * @details Defines all possible actions that can be triggered in ToyMode through:
     *          - Button press patterns (short, medium, long press)
     *          - Dual-button combinations
     *          - MAVLink commands from ground station
     *          - Automatic triggers based on flight state
     *          
     *          Actions are configured via TMODE_ACT1 through TMODE_ACT9 parameters,
     *          allowing customization of button behavior for different products.
     *          
     *          Common button mapping examples:
     *          - Short press: ACTION_TAKE_PHOTO (capture image)
     *          - Medium press: ACTION_TOGGLE_MODE (cycle between modes)
     *          - Long press: ACTION_MODE_RTL (return to launch)
     *          - Both buttons: ACTION_DISARM (emergency stop)
     */
    enum toy_action {
        ACTION_NONE         = 0,   ///< No action (default/disabled)
        ACTION_TAKE_PHOTO   = 1,   ///< Trigger camera photo capture
        ACTION_TOGGLE_VIDEO = 2,   ///< Start/stop video recording
        ACTION_MODE_ACRO    = 3,   ///< Switch to ACRO mode (manual rate control)
        ACTION_MODE_ALTHOLD = 4,   ///< Switch to ALTHOLD mode (altitude hold)
        ACTION_MODE_AUTO    = 5,   ///< Switch to AUTO mode (mission execution)
        ACTION_MODE_LOITER  = 6,   ///< Switch to LOITER mode (position hold)
        ACTION_MODE_RTL     = 7,   ///< Switch to RTL mode (return to launch)
        ACTION_MODE_CIRCLE  = 8,   ///< Switch to CIRCLE mode (orbit point)
        ACTION_MODE_LAND    = 9,   ///< Switch to LAND mode (automatic landing)
        ACTION_MODE_DRIFT   = 10,  ///< Switch to DRIFT mode (car-like control)
        ACTION_MODE_SPORT   = 11,  ///< Switch to SPORT mode (fast manual)
        ACTION_MODE_AUTOTUNE= 12,  ///< Switch to AUTOTUNE mode (PID tuning)
        ACTION_MODE_POSHOLD = 13,  ///< Switch to POSHOLD mode (brake and hold)
        ACTION_MODE_BRAKE   = 14,  ///< Switch to BRAKE mode (rapid stop)
        ACTION_MODE_THROW   = 15,  ///< Switch to THROW mode (throw to start)
        ACTION_MODE_FLIP    = 16,  ///< Switch to FLIP mode (acrobatic flip)
        ACTION_MODE_STAB    = 17,  ///< Switch to STABILIZE mode (manual angle)
        ACTION_DISARM       = 18,  ///< Disarm motors immediately
        ACTION_TOGGLE_MODE  = 19,  ///< Toggle between configured primary modes
        ACTION_ARM_LAND_RTL = 20,  ///< Multi-function: ARM if disarmed, LAND if flying, RTL otherwise
        ACTION_TOGGLE_SIMPLE = 21, ///< Toggle simple mode (heading reference)
        ACTION_TOGGLE_SSIMPLE = 22,///< Toggle super simple mode (always forward)
        ACTION_LOAD_TEST = 23,     ///< Start motor load test sequence (manufacturing)
        ACTION_MODE_FLOW = 24,     ///< Switch to FLOWHOLD mode (optical flow hold)
    };

    /**
     * @brief Last action executed (for state tracking and telemetry)
     */
    enum toy_action last_action;

    /**
     * @enum toy_flags
     * @brief Configuration flags for ToyMode behavior (TMODE_FLAGS parameter)
     * 
     * @details Bitmask flags that control ToyMode safety and automation features.
     *          These flags are set via the TMODE_FLAGS parameter and can be combined
     *          using bitwise OR operations.
     *          
     *          Example configurations:
     *          - Basic safety: FLAG_THR_ARM | FLAG_THR_DISARM (throttle gestures only)
     *          - Full auto: All flags enabled (maximum automation)
     *          - Manual control: No flags (traditional RC control)
     *          
     *          Recommended consumer configuration:
     *          TMODE_FLAGS = 15 (all flags enabled) for maximum ease of use
     */
    enum toy_flags {
        FLAG_THR_DISARM     = 1<<0,  ///< Enable disarm via low throttle stick (2 sec hold when landed)
        FLAG_THR_ARM        = 1<<1,  ///< Enable arm via high throttle stick (2 sec hold before flight)
        FLAG_UPGRADE_LOITER = 1<<2,  ///< Automatically upgrade from ALTHOLD to LOITER when GPS lock acquired
        FLAG_RTL_CANCEL     = 1<<3,  ///< Allow pilot to cancel RTL with large stick input (override safety mode)
    };

    /**
     * @enum blink_patterns
     * @brief LED blink patterns for visual status feedback
     * 
     * @details 16-bit patterns where each bit represents an LED state in a repeating
     *          sequence. Patterns repeat at approximately 2 Hz (32 updates/sec, 16 bits
     *          per pattern = 0.5 sec per cycle). Bit 1 = LED on, bit 0 = LED off.
     *          
     *          Pattern format: 0xABCD where each hex digit controls 4 time slots
     *          - 0xF = 1111 (solid on for 4 slots)
     *          - 0xA = 1010 (alternating on/off)
     *          - 0x0 = 0000 (solid off for 4 slots)
     *          
     *          Visual feedback examples:
     *          - BLINK_FULL: Solid LED = Armed and flying normally
     *          - BLINK_SLOW_1: Slow single blink = GPS searching
     *          - BLINK_2: Two blinks = Two satellites or error code 2
     *          - BLINK_NO_RX: Fast alternating = No RC signal
     *          
     *          Red LED typically indicates:
     *          - Battery cell count (blink count matches cell count)
     *          - Error conditions (rapid patterns)
     *          - Recording status (when video active)
     *          
     *          Green LED typically indicates:
     *          - GPS status (solid = good lock, blinking = searching)
     *          - Arming state (fast blink = ready to arm, slow = not ready)
     *          - Flight mode status
     */
    enum blink_patterns {
        BLINK_FULL   = 0xFFFF,  ///< Solid on (armed and flying normally)
        BLINK_OFF    = 0x0000,  ///< Solid off (standby state)
        BLINK_1      = 0xBFFF,  ///< Single blink per cycle
        BLINK_2      = 0xAFFF,  ///< Two blinks per cycle
        BLINK_3      = 0xABFF,  ///< Three blinks per cycle
        BLINK_4      = 0xAAFF,  ///< Four blinks per cycle
        BLINK_6      = 0xAAAF,  ///< Six blinks per cycle
        BLINK_8      = 0xAAAA,  ///< Eight blinks per cycle (rapid)
        BLINK_NO_RX  = 0x1111,  ///< Fast alternating (RC signal loss)
        BLINK_SLOW_1 = 0xF0FF,  ///< Slow single blink (GPS searching)
        BLINK_VSLOW  = 0xF000,  ///< Very slow blink (long wait state)
        BLINK_MED_1  = 0xF0F0,  ///< Medium rate single blink (ready to arm)
    };

    // State tracking
    bool done_first_update;  ///< True after first update() call (initialization complete)
    
    // Configuration parameters (stored in EEPROM)
    AP_Int8 enable;          ///< ToyMode enable: 0=off, 1=V2450 buttons, 2=F412 buttons
    AP_Int8 primary_mode[2]; ///< Primary flight modes for toggle action (TMODE_PRI_MODE1/2)
    AP_Int8 actions[9];      ///< Button action mappings (TMODE_ACT1 through TMODE_ACT9)
    AP_Int8 trim_auto;       ///< Automatic trim calibration enable (TMODE_TRIM_AUTO)
    AP_Int16 flags;          ///< Behavior flags bitmask (TMODE_FLAGS) - see toy_flags enum

    /**
     * @brief Automatic trim calibration data
     * 
     * @details Stores RC input samples during initial flight for automatic trim
     *          calibration. Samples are collected over several seconds and averaged
     *          to determine neutral stick positions.
     */
    struct {
        uint32_t start_ms;   ///< Timestamp when trim calibration started (milliseconds)
        uint16_t chan[4];    ///< Accumulated trim values for throttle, roll, pitch, yaw
    } trim;
    
    // Arming/disarming state tracking
    uint32_t power_counter;        ///< Counter for arm button hold detection (increments each update)
    uint32_t throttle_low_counter; ///< Counter for low throttle disarm gesture (ms held low)
    uint32_t throttle_high_counter;///< Counter for high throttle arm gesture (ms held high)
    uint16_t last_ch5;             ///< Last channel 5 PWM value (for button change detection)
    bool last_left_button;         ///< Previous left button state (for edge detection)
    uint8_t last_mode_choice;      ///< Last selected mode index for toggle action
    int32_t left_press_counter;    ///< Left button press duration counter (for short/medium/long press)
    int32_t right_press_counter;   ///< Right button press duration counter (for short/medium/long press)
    bool ignore_left_change;       ///< Ignore next left button change (debouncing)
    int16_t throttle_mid = 500;    ///< Learned throttle mid-point for hover (PWM units, 0-1000)
    uint32_t throttle_arm_ms;      ///< Timestamp when throttle arm gesture initiated (for ramping)
    bool upgrade_to_loiter;        ///< Flag indicating pending automatic upgrade to LOITER mode
    uint32_t last_action_ms;       ///< Timestamp of last action execution (for rate limiting)
    uint32_t reset_turtle_start_ms;///< Timestamp for turtle mode reset detection (upside-down recovery)

    uint32_t last_video_ms;        ///< Timestamp when last video recording status received (for LED feedback)
    
    // LED blink state
    uint16_t red_blink_pattern;    ///< Current red LED blink pattern (16-bit pattern)
    uint16_t green_blink_pattern;  ///< Current green LED blink pattern (16-bit pattern)
    uint8_t red_blink_index;       ///< Current bit position in red pattern (0-15)
    uint8_t green_blink_index;     ///< Current bit position in green pattern (0-15)
    uint16_t red_blink_count;      ///< Number of blinks to show in red pattern (battery cells)
    uint16_t green_blink_count;    ///< Number of blinks to show in green pattern (status code)
    uint8_t blink_disarm;          ///< Disarm state for blink pattern generation

    /**
     * @brief Battery voltage based thrust limiting parameters
     * 
     * @details Configuration for voltage-to-thrust curve that prevents brown-out.
     *          Defines the linear mapping between battery voltage and maximum allowed thrust.
     *          
     *          Configuration parameters:
     *          - volt_min (TMODE_FILT_VMIN): Minimum voltage threshold (typically 3.3V per cell)
     *          - volt_max (TMODE_FILT_VMAX): Maximum voltage threshold (typically 4.0V per cell)
     *          - thrust_min (TMODE_FILT_TMIN): Minimum thrust limit at low voltage (typically 0.5 = 50%)
     *          - thrust_max (TMODE_FILT_TMAX): Maximum thrust limit at full voltage (typically 1.0 = 100%)
     *          
     *          Example: For 3S battery (3 cells):
     *          - volt_min = 9.9V (3.3V/cell), volt_max = 12.0V (4.0V/cell)
     *          - At 12.0V+: 100% thrust available
     *          - At 10.95V: 75% thrust limit (linear interpolation)
     *          - At 9.9V or below: 50% thrust limit
     */
    struct {
        AP_Float volt_min;   ///< Minimum voltage for thrust curve (TMODE_FILT_VMIN)
        AP_Float volt_max;   ///< Maximum voltage for thrust curve (TMODE_FILT_VMAX)
        AP_Float thrust_min; ///< Minimum thrust limit at low voltage (TMODE_FILT_TMIN)
        AP_Float thrust_max; ///< Maximum thrust limit at high voltage (TMODE_FILT_TMAX)
    } filter;
    
    float filtered_voltage = 4.0;  ///< Low-pass filtered battery voltage (volts) for thrust limiting

    uint8_t motor_log_counter;     ///< Counter for throttled motor output logging

    Mode::Number last_set_mode = Mode::Number::LOITER; ///< Last mode set by user (for restore after RTL)

    /**
     * @brief Motor load test data point
     * 
     * @details Single row of motor thrust values for load testing. Each test profile
     *          consists of an array of load_data structures defining thrust over time.
     */
    struct load_data {
        uint16_t m[4];  ///< Motor thrust values for 4 motors (PWM units or percentage)
    };

    /**
     * @enum load_type
     * @brief Motor load test profile types
     * 
     * @details Selects which predefined load profile to execute during load testing.
     *          Different profiles stress different aspects of motor/ESC/battery performance.
     */
    enum load_type {
        LOAD_TYPE_CONSTANT=0,  ///< Constant thrust level (steady state test)
        LOAD_TYPE_LOG1=1,      ///< Logged profile pattern 1 (variable thrust pattern)
        LOAD_TYPE_LOG2=2,      ///< Logged profile pattern 2 (alternative pattern)
    };
    
    /**
     * @brief Motor load test state and configuration
     * 
     * @details Tracks execution state of motor load testing sequence. Load tests
     *          are used for manufacturing QA and hardware validation by applying
     *          controlled thrust patterns and logging the results.
     */
    struct {
        bool running;            ///< True if load test currently executing
        uint32_t row;            ///< Current row index in load_data1[] table
        uint8_t filter_counter;  ///< Filtering state for load application
        AP_Float load_mul;       ///< Load multiplier for safety scaling (TMODE_LOAD_MUL)
        AP_Int8  load_filter;    ///< Filtering level for load application (TMODE_LOAD_FILT)
        AP_Int8  load_type;      ///< Load profile type selection (TMODE_LOAD_TYPE)
    } load_test;
    
    /**
     * @brief Predefined motor load test data table
     * 
     * @details Array of motor thrust values applied sequentially during load testing.
     *          Each row represents one time step of the test profile.
     * 
     * @note Defined in toy_mode.cpp implementation file
     */
    static const struct load_data load_data1[];
};
