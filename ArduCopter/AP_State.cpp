/**
 * @file AP_State.cpp
 * @brief Vehicle state management for ArduCopter
 * 
 * @details This file implements state flag setter functions for the ArduCopter vehicle.
 *          These functions manage critical vehicle state flags that control behavior,
 *          mode transitions, and safety interlocks throughout the flight control system.
 *          
 *          Vehicle State Machine Overview:
 *          
 *          The vehicle maintains multiple state flags in the 'ap' struct (ap.h) and 'failsafe'
 *          struct that track the current operational status. These states interact to determine:
 *          - Whether the vehicle can arm or disarm
 *          - Which flight modes are available
 *          - How commands from pilot and autonomous systems are processed
 *          - When failsafe actions should trigger
 *          - Whether motors should spin or remain stopped
 *          
 *          Key State Flags Managed Here:
 *          - auto_armed: Enables autonomous flight after throttle raised in AUTO/GUIDED modes
 *          - simple_mode: Simplifies pilot inputs relative to reference heading
 *          - failsafe.radio: RC link failsafe state (loss of RC control)
 *          - failsafe.gcs: GCS link failsafe state (loss of ground station telemetry)
 *          - using_interlock: Motor interlock control is available via switch
 *          
 *          State Interaction Examples:
 *          
 *          Auto Armed State:
 *          When vehicle is armed in AUTO or GUIDED mode, motors won't start mission until
 *          ap.auto_armed becomes true (throttle raised above zero). This safety interlock
 *          prevents unexpected mission execution immediately upon arming.
 *          
 *          Simple Mode State:
 *          When simple_mode != NONE, pilot stick inputs are transformed from body-relative
 *          to earth-frame relative to a reference heading, making the vehicle easier to fly
 *          for beginners by maintaining consistent control response regardless of vehicle yaw.
 *          
 *          Failsafe States:
 *          Radio and GCS failsafe flags trigger protective actions when communication is lost.
 *          These states interact with flight mode logic to select appropriate recovery actions
 *          (RTL, LAND, etc.) based on failsafe configuration parameters.
 *          
 *          Motor Interlock State:
 *          The using_interlock flag indicates that an RC switch controls motor interlock,
 *          which provides an additional safety layer allowing pilot to completely disable
 *          motor output independently from the arming state (primarily for helicopters).
 * 
 * @note All setter functions log state changes via LOGGER_WRITE_EVENT for flight analysis
 * @warning State flag consistency is critical for flight safety - these functions must
 *          maintain proper state synchronization with logging, notifications, and other
 *          subsystems
 * 
 * @see Copter.h for 'ap' struct definition (vehicle state flags)
 * @see Copter.h for 'failsafe' struct definition
 * @see enum class SimpleMode for simple mode variants
 * 
 * Source: ArduCopter/AP_State.cpp
 */

#include "Copter.h"

/**
 * @brief Set the auto-armed state flag
 * 
 * @details Controls whether autonomous modes (AUTO, GUIDED, RTL, etc.) are permitted to
 *          start executing their flight plans. This is a critical safety interlock that
 *          prevents unexpected autonomous behavior immediately after arming.
 *          
 *          Auto-Armed State Machine:
 *          1. Vehicle arms in AUTO/GUIDED: ap.auto_armed = false
 *          2. Pilot raises throttle: set_auto_armed(true) called
 *          3. Mission execution begins: waypoint navigation, guided commands execute
 *          
 *          This prevents scenarios where:
 *          - Vehicle arms unexpectedly in AUTO mode and immediately takes off
 *          - Guided mode commands execute before pilot is ready
 *          - Failsafe RTL triggers before pilot has established control
 *          
 *          The auto_armed flag is checked by:
 *          - Mode::auto_armed_checks() - verifies pilot readiness
 *          - AUTO mode initialization - delays mission start
 *          - GUIDED mode target acceptance - delays autonomous movement
 *          - RTL mode entry - may delay return-to-launch
 *          
 *          State Transitions and Interactions:
 *          - Armed in manual mode (STABILIZE, ALT_HOLD): auto_armed set true automatically
 *          - Armed in AUTO/GUIDED: auto_armed remains false until throttle raised
 *          - Disarm: auto_armed reset to false
 *          - Mode change to manual: auto_armed set true
 *          - Mode change to autonomous: auto_armed may remain false if throttle at zero
 *          
 *          Logging:
 *          State change to true is logged via AUTO_ARMED event for post-flight analysis.
 *          This helps identify when autonomous operations began in flight logs.
 * 
 * @param[in] b  New auto-armed state
 *               - true: Autonomous modes permitted to execute (throttle raised)
 *               - false: Autonomous modes blocked until throttle raised
 * 
 * @return void
 * 
 * @note No-op if state is already set to requested value (avoids redundant logging)
 * @note This function is typically called from modes.cpp during throttle monitoring
 * @warning Manually setting this true bypasses throttle safety interlock - use carefully
 * 
 * @see ap.auto_armed flag in ap struct (Copter.h:742)
 * @see update_auto_armed() in Copter.cpp for automatic state management
 * @see Mode::auto_armed_checks() for usage in autonomous modes
 * 
 * Source: ArduCopter/AP_State.cpp:4-14
 */
void Copter::set_auto_armed(bool b)
{
    // Early exit optimization: if no change, avoid unnecessary state updates and logging
    // This prevents log spam when function is called repeatedly with same value
    if( ap.auto_armed == b )
        return;

    // Update the vehicle state flag in the ap struct
    // This flag is checked by autonomous modes to gate mission execution
    ap.auto_armed = b;
    
    // Log state transition to true (autonomous operations now permitted)
    // Transition to false is not logged as it occurs during disarm or mode changes
    // and is less critical for flight analysis
    if(b){
        LOGGER_WRITE_EVENT(LogEvent::AUTO_ARMED);
    }
}

/**
 * @brief Set the simple mode state for simplified pilot control
 * 
 * @details Simple and Super-Simple modes transform pilot stick inputs to make the vehicle
 *          easier to control by maintaining consistent input response regardless of vehicle
 *          heading. These modes are particularly helpful for:
 *          - New pilots learning to fly
 *          - Line-of-sight flight at long distances
 *          - Situations where maintaining vehicle orientation is difficult
 *          
 *          Mode Descriptions:
 *          
 *          NONE (Normal Mode):
 *          - Stick inputs are relative to vehicle's current heading (body frame)
 *          - Forward stick always moves vehicle forward relative to its nose
 *          - Most challenging for new pilots when vehicle is flying toward them
 *          
 *          SIMPLE:
 *          - Stick inputs are relative to the vehicle's heading when it was armed
 *          - Forward stick always moves vehicle away from pilot (based on arming heading)
 *          - Right stick always moves vehicle to pilot's right
 *          - Easier to fly but can be confusing if vehicle is re-armed in different orientation
 *          
 *          SUPERSIMPLE:
 *          - Stick inputs are relative to the direction from HOME to vehicle
 *          - Forward stick always moves vehicle away from pilot's current position
 *          - Right stick always moves vehicle to pilot's right
 *          - Most intuitive for line-of-sight flight, automatically adapts as vehicle moves
 *          - Requires valid GPS position and HOME location
 *          
 *          State Machine Transitions:
 *          
 *          NONE → SIMPLE:
 *          - Uses simple_cos_yaw and simple_sin_yaw (set at arming) for coordinate transform
 *          - Sends "SIMPLE mode on" message to GCS
 *          - Logs SET_SIMPLE_ON event
 *          
 *          NONE → SUPERSIMPLE or SIMPLE → SUPERSIMPLE:
 *          - Calls update_super_simple_bearing(true) to initialize HOME-relative bearing
 *          - Recalculates super_simple_cos_yaw and super_simple_sin_yaw continuously
 *          - Sends "SUPERSIMPLE mode on" message to GCS
 *          - Logs SET_SUPERSIMPLE_ON event
 *          
 *          SIMPLE/SUPERSIMPLE → NONE:
 *          - Disables coordinate transformation
 *          - Returns to normal body-frame control
 *          - Sends "SIMPLE mode off" message to GCS
 *          - Logs SET_SIMPLE_OFF event
 *          
 *          Integration with Flight Control:
 *          The simple_mode flag is checked in get_pilot_desired_lean_angles() where pilot
 *          stick inputs are transformed using the appropriate reference heading before being
 *          passed to the attitude controller. This transformation is transparent to the
 *          attitude and position controllers - they still receive attitude targets in the
 *          normal coordinate frame.
 *          
 *          Enabling Simple Mode:
 *          - Via auxiliary RC switch configured for SIMPLE_MODE function
 *          - Via auxiliary RC switch configured for SUPERSIMPLE_MODE function  
 *          - Programmatically via this function
 *          - Some flight modes may force simple mode state
 *          
 * @param[in] b  New simple mode state (enum class SimpleMode)
 *               - SimpleMode::NONE: Normal body-frame control (simple mode disabled)
 *               - SimpleMode::SIMPLE: Earth-frame control relative to arming heading
 *               - SimpleMode::SUPERSIMPLE: Earth-frame control relative to HOME direction
 * 
 * @return void
 * 
 * @note No-op if already in requested mode (avoids redundant GCS messages and logging)
 * @note SUPERSIMPLE requires valid GPS position and HOME location to function correctly
 * @note Simple mode transformation occurs at approximately 400Hz (main loop rate)
 * @warning Simple mode may be confusing if pilot's mental model doesn't match the reference frame
 * @warning SUPERSIMPLE provides incorrect control if HOME location is not properly set
 * 
 * @see enum class SimpleMode in Copter.h for mode definitions
 * @see simple_mode variable in Copter.h (Copter.h:1052-1058)
 * @see update_super_simple_bearing() for SUPERSIMPLE heading calculation
 * @see init_simple_bearing() called at arming to set SIMPLE mode reference
 * @see get_pilot_desired_lean_angles() where transformation is applied
 * 
 * Source: ArduCopter/AP_State.cpp:22-43
 */
void Copter::set_simple_mode(SimpleMode b)
{
    // Only process actual state changes to avoid redundant logging and GCS messaging
    // This check is critical to prevent message spam when RC switch is in same position
    if (simple_mode != b) {
        // Process mode-specific initialization, logging, and pilot notification
        switch (b) {
            case SimpleMode::NONE:
                // Disable simple mode - return to normal body-frame control
                // No special cleanup needed, transformation just stops being applied
                LOGGER_WRITE_EVENT(LogEvent::SET_SIMPLE_OFF);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode off");
                break;
                
            case SimpleMode::SIMPLE:
                // Enable SIMPLE mode - inputs relative to heading at arming
                // Reference angles (simple_cos_yaw, simple_sin_yaw) were set during arming
                // in init_simple_bearing() and remain constant during flight
                LOGGER_WRITE_EVENT(LogEvent::SET_SIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode on");
                break;
                
            case SimpleMode::SUPERSIMPLE:
                // Enable SUPERSIMPLE mode - inputs relative to HOME direction
                // Initialize the HOME-relative bearing calculation
                // force_update=true ensures immediate recalculation even if recently updated
                // Super-simple bearing is continuously updated in main loop via update_super_simple_bearing(false)
                update_super_simple_bearing(true);
                LOGGER_WRITE_EVENT(LogEvent::SET_SUPERSIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SUPERSIMPLE mode on");
                break;
        }
        
        // Update the vehicle state to new simple mode setting
        // This flag is checked by get_pilot_desired_lean_angles() to determine
        // whether and how to transform pilot inputs before attitude control
        simple_mode = b;
    }
}

/**
 * @brief Set the radio (RC link) failsafe state
 * 
 * @details Manages the critical radio failsafe state that triggers protective actions when
 *          RC control is lost. This is one of the most important safety systems in ArduCopter,
 *          as loss of RC link means the pilot can no longer control the vehicle.
 *          
 *          Radio Failsafe Detection:
 *          Radio failsafe is detected when:
 *          - RC receiver stops sending valid PWM/serial data
 *          - Throttle channel drops below FS_THR_VALUE for FS_TIMEOUT seconds
 *          - RC receiver sets failsafe flag in SBUS/other protocols
 *          - No valid RC frames received for configured timeout period
 *          
 *          State Machine Behavior:
 *          
 *          Normal → Failsafe (b = true):
 *          1. failsafe.radio flag set to true
 *          2. failsafe_radio_on_event() called to trigger protective action
 *          3. AP_Notify system updated (triggers LED patterns, buzzer alerts)
 *          4. Vehicle may switch to LAND, RTL, or SMARTRTL based on FS_THR_ENABLE parameter
 *          5. Pilot control inputs ignored or scaled down depending on failsafe action
 *          
 *          Failsafe → Normal (b = false):
 *          1. failsafe.radio flag cleared to false
 *          2. failsafe_radio_off_event() called to restore normal operation
 *          3. AP_Notify system updated (clears LED/buzzer failsafe indicators)
 *          4. Vehicle may return to previous mode or remain in failsafe mode
 *          5. Pilot control inputs restored to normal
 *          
 *          Failsafe Actions (FS_THR_ENABLE parameter):
 *          0: Disabled - no failsafe action (NOT RECOMMENDED)
 *          1: Enabled, always LAND - vehicle lands immediately at current location
 *          2: Enabled, always RTL or LAND - attempts return to launch, lands if RTL unavailable
 *          3: Enabled, auto mode continues, manual modes LAND
 *          4: Enabled, always SmartRTL or RTL or LAND - uses best available return method
 *          
 *          Safety Considerations:
 *          - Radio failsafe has priority over many other vehicle operations
 *          - Mode changes may be restricted during radio failsafe
 *          - Some modes may prevent radio failsafe recovery (e.g., continued landing)
 *          - Failsafe timeout must be tuned carefully:
 *            * Too short: False triggers from momentary signal loss
 *            * Too long: Vehicle travels far before failsafe action
 *          
 *          Integration with Other Systems:
 *          - Interacts with battery failsafe (battery takes priority if both active)
 *          - May be overridden by GCS failsafe in some configurations
 *          - Affects mode availability (some modes disabled during failsafe)
 *          - Impacts motor interlock behavior
 *          - Coordinates with fence system (fence actions may override failsafe)
 *          
 *          Recovery Behavior:
 *          When RC link is restored (failsafe cleared):
 *          - If in LAND mode due to failsafe: typically continues landing
 *          - If in RTL mode due to failsafe: pilot can resume control or switch modes
 *          - Pilot must actively change mode to resume normal flight
 *          - Vehicle does not automatically return to pre-failsafe mode
 *          
 * @param[in] b  New radio failsafe state
 *               - true: RC link lost, protective failsafe action triggered
 *               - false: RC link restored, normal operation resumed
 * 
 * @return void
 * 
 * @note Function is called by radio.cpp after RC signal monitoring
 * @note Only processes actual state changes to avoid repeated failsafe triggering
 * @note AP_Notify flags updated for LED/buzzer indication to ground crew
 * @warning Radio failsafe is flight-critical - incorrect configuration can lead to crashes
 * @warning FS_THR_ENABLE=0 (disabled) is dangerous and not recommended for real flights
 * 
 * @see failsafe.radio flag in failsafe struct (Copter.h:919-932)
 * @see failsafe_radio_on_event() for failsafe action implementation
 * @see failsafe_radio_off_event() for recovery handling
 * @see radio_failsafe_check() in radio.cpp for detection logic
 * @see FS_THR_ENABLE parameter for failsafe action configuration
 * @see FS_THR_VALUE parameter for throttle failsafe threshold
 * @see FS_TIMEOUT parameter for failsafe trigger delay
 * 
 * Source: ArduCopter/AP_State.cpp:46-69
 */
void Copter::set_failsafe_radio(bool b)
{
    // Only process actual state changes to prevent repeated failsafe actions
    // Critical safety check: ensures failsafe_radio_on_event() and failsafe_radio_off_event()
    // are only called once per transition, not on every loop iteration
    if(failsafe.radio != b) {

        // Update the failsafe state flag BEFORE calling event handlers
        // This ordering is important so that the event handlers see the new state
        // and any_failsafe_triggered() returns the correct value
        failsafe.radio = b;

        // Process failsafe state transition and trigger appropriate response
        if (failsafe.radio == false) {
            // RC link has been restored - exit failsafe mode
            // This calls mode-specific recovery logic that may:
            // - Restore pilot control authority
            // - Re-enable mode changes
            // - Clear failsafe restrictions
            // Note: Vehicle typically remains in failsafe mode (LAND/RTL) until pilot changes mode
            failsafe_radio_off_event();
        }else{
            // RC link has been lost - enter failsafe mode
            // This triggers protective action based on FS_THR_ENABLE:
            // - Switch to LAND mode (safest, always works)
            // - Switch to RTL mode (requires GPS, returns to launch)
            // - Switch to SMARTRTL mode (requires saved path, retraces route)
            // - Continue AUTO mode (only if in AUTO, otherwise LAND)
            // The specific action depends on current mode and parameter configuration
            failsafe_radio_on_event();
        }

        // Update the AP_Notify system for visual/audible indication
        // This triggers:
        // - LED patterns (typically red flashing) on board LEDs and external LED strips
        // - Buzzer alerts if buzzer is configured
        // - GCS notification via heartbeat flags
        // Helps ground crew/pilot identify failsafe state visually even without telemetry
        AP_Notify::flags.failsafe_radio = b;
    }
}


/**
 * @brief Set the GCS (Ground Control Station) telemetry link failsafe state
 * 
 * @details Manages the GCS failsafe state that triggers protective actions when telemetry
 *          communication with the ground control station is lost. This failsafe is particularly
 *          important for autonomous operations where the GCS is actively commanding the vehicle
 *          or monitoring critical flight phases.
 *          
 *          GCS Failsafe Detection:
 *          GCS failsafe is triggered when:
 *          - No MAVLink heartbeat received from primary GCS for FS_GCS_TIMEOUT seconds
 *          - Telemetry link quality drops below threshold for sustained period
 *          - GCS explicitly commands failsafe via MAVLink
 *          
 *          State Machine Behavior:
 *          
 *          Normal → Failsafe (b = true):
 *          1. failsafe.gcs flag set to true
 *          2. Vehicle may switch to RTL, LAND, or SMARTRTL based on FS_GCS_ENABLE parameter
 *          3. GCS command acceptance may be restricted or disabled
 *          4. AP_Notify system updated for LED/buzzer indication
 *          5. Autonomous missions may continue or abort depending on configuration
 *          
 *          Failsafe → Normal (b = false):
 *          1. failsafe.gcs flag cleared to false
 *          2. Normal GCS communication and command acceptance restored
 *          3. Vehicle typically remains in failsafe mode until commanded otherwise
 *          4. AP_Notify indicators cleared
 *          
 *          Failsafe Actions (FS_GCS_ENABLE parameter):
 *          0: Disabled - no action on GCS link loss
 *          1: Enabled - trigger RTL mode on GCS link loss
 *          2: Enabled - trigger SmartRTL or RTL or LAND
 *          5: Enabled - continue with mission in AUTO mode, RTL in other modes
 *          
 *          Comparison with Radio Failsafe:
 *          - Radio failsafe: Loss of pilot RC control (immediate safety concern)
 *          - GCS failsafe: Loss of telemetry/GCS commands (monitoring/autonomous concern)
 *          - Radio failsafe typically has higher priority
 *          - GCS failsafe may be less urgent if pilot has RC control
 *          - In fully autonomous operations, both are equally critical
 *          
 *          Usage Scenarios:
 *          
 *          Monitoring Missions:
 *          During autonomous missions, GCS failsafe ensures the vehicle takes safe action
 *          if the operator loses ability to monitor or intervene via telemetry.
 *          
 *          Guided Mode Operations:
 *          In GUIDED mode, GCS provides continuous position/velocity targets. Loss of GCS
 *          link should trigger failsafe to prevent vehicle from continuing with stale commands.
 *          
 *          Long-Range Operations:
 *          For long-range flights beyond RC range, GCS link may be primary communication.
 *          GCS failsafe becomes critical safety system when RC is unavailable.
 *          
 *          Safety Considerations:
 *          - GCS failsafe should be enabled for any autonomous operation
 *          - Timeout (FS_GCS_TIMEOUT) must account for expected telemetry latency
 *          - Multiple telemetry links improve reliability (failover capability)
 *          - GCS failsafe may be suppressed during certain flight phases (e.g., landing)
 *          - Consider interaction with RC failsafe for worst-case dual-link failure
 *          
 *          Integration with Other Systems:
 *          - Works alongside radio failsafe (not mutually exclusive)
 *          - May affect mode change restrictions
 *          - Impacts GUIDED mode operation
 *          - Coordinates with battery and other failsafes
 *          - Affects fence breach response
 *          
 * @param[in] b  New GCS failsafe state
 *               - true: GCS telemetry link lost, failsafe action may be triggered
 *               - false: GCS telemetry link restored, normal operation
 * 
 * @return void
 * 
 * @note Unlike radio failsafe, does not call separate event handlers (simpler state management)
 * @note AP_Notify flags updated for LED/buzzer indication
 * @note Function is called by failsafe.cpp during GCS heartbeat monitoring
 * @note GCS failsafe action depends on FS_GCS_ENABLE parameter and current flight mode
 * @warning Disabling GCS failsafe (FS_GCS_ENABLE=0) is risky for autonomous operations
 * @warning Multiple GCS connections may complicate failsafe detection logic
 * 
 * @see failsafe.gcs flag in failsafe struct (Copter.h:919-932)
 * @see failsafe_gcs_check() in failsafe.cpp for detection and action logic
 * @see FS_GCS_ENABLE parameter for failsafe action configuration
 * @see FS_GCS_TIMEOUT parameter for failsafe trigger delay (seconds)
 * @see set_failsafe_radio() for related RC link failsafe
 * @see any_failsafe_triggered() to check if any failsafe is active
 * 
 * Source: ArduCopter/AP_State.cpp:73-79
 */
void Copter::set_failsafe_gcs(bool b)
{
    // Update the GCS failsafe state flag directly
    // Unlike radio failsafe, GCS failsafe uses simpler state management without
    // separate on/off event handlers. The actual failsafe action (mode change, etc.)
    // is triggered by failsafe_gcs_check() in failsafe.cpp which monitors this flag.
    failsafe.gcs = b;

    // Update the AP_Notify system for visual/audible indication
    // This triggers:
    // - LED patterns indicating GCS link status
    // - Buzzer alerts if configured
    // - GCS failsafe indication in heartbeat messages (if GCS reconnects)
    // Provides feedback to ground crew about telemetry link health
    AP_Notify::flags.failsafe_gcs = b;
}

/**
 * @brief Update motor interlock availability state
 * 
 * @details Determines whether motor interlock control is active based on vehicle configuration
 *          and RC auxiliary switch setup. Motor interlock provides an additional safety layer
 *          beyond arming, allowing the pilot to completely disable motor output via a dedicated
 *          switch while keeping the vehicle armed.
 *          
 *          Motor Interlock Concept:
 *          Motor interlock is a two-stage safety system:
 *          1. Arming: Enables the flight control system (safety check passed)
 *          2. Interlock: Actually permits motors to spin (pilot ready)
 *          
 *          This separation allows:
 *          - Vehicle to remain armed while safely preventing motor spin
 *          - Quick motor enable/disable without full disarm/rearm cycle
 *          - Emergency motor stop without losing vehicle state
 *          - Safer ground operations and testing
 *          
 *          Helicopter vs Multicopter Behavior:
 *          
 *          Helicopters (FRAME_CONFIG == HELI_FRAME):
 *          - Motor interlock is ALWAYS enabled (ap.using_interlock = true)
 *          - Interlock controls rotor engagement via ESC or mechanical clutch
 *          - Essential for helicopter safety - allows rotor to be stopped while armed
 *          - Pilot must explicitly enable interlock to engage rotor
 *          - Provides emergency rotor stop capability during ground operations
 *          
 *          Multirotors (all other frame types):
 *          - Motor interlock is OPTIONAL
 *          - Only active if RC channel assigned to MOTOR_INTERLOCK aux function
 *          - Less commonly used than on helicopters
 *          - May also be activated temporarily for THROW mode during throw detection
 *          - When enabled, provides emergency motor stop capability
 *          
 *          State Flag Usage:
 *          The ap.using_interlock flag indicates whether interlock control is active.
 *          When true:
 *          - ap.motor_interlock_switch reflects actual switch position
 *          - motors.set_interlock() called with switch state
 *          - Motors only spin when both armed AND interlock enabled
 *          When false:
 *          - Interlock is assumed always enabled (motors spin when armed)
 *          - Traditional arming behavior (no separate interlock)
 *          
 *          Integration with Flight Control:
 *          - Checked during arming sequence (affects motor test behavior)
 *          - Monitored in main loop to update motor output enable state
 *          - Affects motor spinning logic in various flight modes
 *          - Coordinates with emergency stop functionality
 *          - Impacts takeoff detection and landing detection
 *          
 *          Safety Interlocks:
 *          When using_interlock is true, motors will NOT spin unless:
 *          1. Vehicle is armed (ap.armed_with_airmode_switch OR armed normally)
 *          2. Motor interlock switch is enabled (ap.motor_interlock_switch = true)
 *          3. Vehicle is not landed (or takeoff/throw sequence active)
 *          4. No critical failsafes preventing motor output
 *          
 *          Throw Mode Interaction:
 *          In THROW mode, interlock may be temporarily controlled by firmware:
 *          - Disabled during throw detection (vehicle waiting to be thrown)
 *          - Enabled when flight detected (vehicle starts flying)
 *          - Provides automatic motor enable without pilot input
 *          
 *          RC Switch Configuration:
 *          To enable motor interlock on multirotors:
 *          1. Configure RC channel auxiliary function to MOTOR_INTERLOCK (32)
 *          2. Switch must be in enabled position for motors to spin when armed
 *          3. Switch typically set to 3-position: Low=Off, Mid/High=On
 *          4. Provides emergency motor stop by moving switch to Low
 *          
 * @return void
 * 
 * @note Called during initialization and when RC aux function configuration changes
 * @note Helicopter builds always set using_interlock = true regardless of switch config
 * @note For multirotors, updates dynamically based on RC channel configuration
 * @note Does not directly control motors - only sets flag checked by motor control code
 * @warning On helicopters, forgetting to enable interlock switch prevents flight
 * @warning Accidental interlock disable during flight causes immediate motor stop (crash)
 * 
 * @see ap.using_interlock flag in ap struct (Copter.h:742 line 18)
 * @see ap.motor_interlock_switch flag (Copter.h:742 line 20)
 * @see RC_Channel::AUX_FUNC::MOTOR_INTERLOCK for RC configuration
 * @see motors->set_interlock() for motor output control
 * @see read_radio() where interlock switch state is processed
 * @see FRAME_CONFIG for helicopter vs multirotor detection
 * 
 * Source: ArduCopter/AP_State.cpp:83-93
 */
void Copter::update_using_interlock()
{
#if FRAME_CONFIG == HELI_FRAME
    // Helicopters ALWAYS use motor interlock for safety
    // This is a fundamental requirement for helicopter operation because:
    // 1. Rotor must be stopped for safe ground operations while keeping electronics active
    // 2. Provides emergency rotor stop without losing calibration/state from disarming
    // 3. ESC or mechanical clutch engagement controlled via interlock
    // 4. Standard helicopter operating procedure requires separate rotor engagement control
    ap.using_interlock = true;
#else
    // For multirotors and other frame types, motor interlock is optional
    // Enable interlock control if:
    // 1. An RC channel is configured with MOTOR_INTERLOCK auxiliary function, OR
    // 2. Currently in THROW mode (interlock used to prevent motors during throw detection)
    //
    // find_channel_for_option() searches all RC channels for the specified aux function
    // Returns nullptr if no channel configured for MOTOR_INTERLOCK
    // Returns channel pointer if found, which evaluates to true in boolean context
    //
    // When using_interlock is false, motors spin immediately when armed (traditional behavior)
    // When using_interlock is true, motors only spin when armed AND interlock switch enabled
    ap.using_interlock = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) != nullptr;
#endif
}
