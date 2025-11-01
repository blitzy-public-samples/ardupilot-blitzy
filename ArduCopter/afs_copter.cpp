/**
 * @file afs_copter.cpp
 * @brief Copter-specific implementation of the Advanced Failsafe (AFS) system
 * 
 * @details This file implements the Copter vehicle-specific overrides for the
 *          AP_AdvancedFailsafe system. The Advanced Failsafe provides additional
 *          safety mechanisms beyond standard failsafes, including the ability to
 *          terminate the vehicle (stop all motors) in emergency situations.
 *          
 *          The Advanced Failsafe system is designed to meet regulatory requirements
 *          for flights over populated areas or in controlled airspace where an
 *          immediate vehicle shutdown capability is mandated. Common triggers include:
 *          - Sustained GPS loss beyond configured threshold
 *          - Prolonged Ground Control Station (GCS) communication loss
 *          - Geofence breach with no recovery possible
 *          - Manual termination command from GCS or RC transmitter
 *          
 *          Key Safety Characteristics:
 *          - Vehicle termination is IRREVERSIBLE without power cycle
 *          - Hardware termination pin provides independent shutdown path
 *          - Failsafe PWM values configured for safe de-energization
 *          - Two termination modes: LAND (controlled descent) or TERMINATE (immediate shutdown)
 * 
 * @note The core AFS logic (trigger conditions, timeouts) is in libraries/AP_AdvancedFailsafe
 * @warning This system can immediately shut down all motors. Misconfiguration can cause crashes.
 * 
 * @see libraries/AP_AdvancedFailsafe for base implementation and trigger logic
 * @see ArduCopter/failsafe.cpp for standard failsafe mechanisms
 * 
 * Source: ArduCopter/afs_copter.cpp
 */

#include "Copter.h"

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED

/**
 * @brief Execute vehicle termination sequence to stop all propulsion
 * 
 * @details This function implements the Copter-specific termination behavior when
 *          the Advanced Failsafe system determines that vehicle operation must cease
 *          immediately. This is a safety-critical function that can be triggered by:
 *          
 *          Termination Trigger Conditions:
 *          - Sustained GPS loss beyond AFS_GPS_LOSS threshold (default 30 seconds)
 *          - Prolonged GCS communication loss beyond AFS_RC_FAIL_TIME (default 3 seconds)
 *          - Geofence breach with AFS_TERM_ACTION set to terminate
 *          - Manual termination command via MAVLink or RC switch
 *          - Hardware termination pin activation (independent of software)
 *          
 *          Termination Modes:
 *          1. TERMINATE_ACTION_LAND: Initiates controlled LAND mode descent
 *          2. TERMINATE_ACTION_TERMINATE (default): Immediate motor shutdown
 *          
 *          In TERMINATE mode, the following sequence executes:
 *          - Motors commanded to SHUT_DOWN spool state (immediate stop)
 *          - Vehicle disarmed through AFS method (prevents re-arming without power cycle)
 *          - All auxiliary channels set to TRIM positions for safe de-energization:
 *            * Helicopter rotor speed controller (RSC) to idle
 *            * Tail RSC to idle
 *            * Engine run enable to off
 *            * Ignition to off
 *            * Manual and generic channels to safe positions
 *          - All servo channels output to apply termination values
 * 
 * @note This action is IRREVERSIBLE without a full power cycle of the flight controller
 * @note Hardware termination pin (if configured) provides independent shutdown even if
 *       software hangs or CPU is unresponsive
 * 
 * @warning SAFETY CRITICAL: This function immediately stops all motors, causing the
 *          vehicle to fall. Use only when continued flight poses greater risk than
 *          immediate descent (e.g., loss of control, flyaway, airspace violation).
 * @warning Once executed, vehicle cannot be recovered mid-flight. Plan mission profiles
 *          to avoid triggering AFS conditions during critical flight phases.
 * @warning Regulatory Requirement: Many aviation authorities require this capability
 *          for operations over populated areas or beyond visual line of sight (BVLOS).
 * 
 * @see AP_AdvancedFailsafe::check() for trigger condition evaluation
 * @see setup_IO_failsafe() for hardware failsafe PWM configuration
 * @see AP_Arming::Method::AFS for disarm method that prevents re-arm without power cycle
 * 
 * Source: ArduCopter/afs_copter.cpp:12-34
 */
void AP_AdvancedFailsafe_Copter::terminate_vehicle(void)
{
    // Check configured termination action: controlled landing vs immediate shutdown
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        // LAND mode provides controlled descent using altitude controller
        // Vehicle will descend at LAND_SPEED rate and disarm on ground contact
        copter.set_mode(Mode::Number::LAND, ModeReason::TERMINATE);
    } else {
        // TERMINATE mode: Immediate motor shutdown (default and most common)
        
        // Command motors to shut down immediately - bypasses normal spool-down logic
        // This sets motor outputs to minimum PWM values configured in setup_IO_failsafe()
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.motors->output();

        // Disarm the vehicle using AFS method - this is a special disarm type that
        // prevents re-arming without a power cycle, ensuring termination is irreversible
        copter.arming.disarm(AP_Arming::Method::AFS);
    
        // Set all auxiliary servo channels to safe TRIM positions for de-energization
        // This ensures helicopter RSCs, engine controls, and other peripherals are safed
        
        // Helicopter main rotor speed controller - set to idle/off
        SRV_Channels::set_output_limit(SRV_Channel::k_heli_rsc, SRV_Channel::Limit::TRIM);
        
        // Helicopter tail rotor speed controller - set to idle/off
        SRV_Channels::set_output_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::Limit::TRIM);
        
        // Engine run enable signal - typically kills internal combustion engines
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::TRIM);
        
        // Ignition system - disable spark for gas engines
        SRV_Channels::set_output_limit(SRV_Channel::k_ignition, SRV_Channel::Limit::TRIM);
        
        // Generic unassigned channels - set to safe middle position
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
        
        // Manual passthrough channels - set to safe middle position
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    }

    // Output all servo channel values to hardware - applies termination configuration
    // This ensures failsafe PWM values reach the servo rail even if main loop stops
    SRV_Channels::output_ch_all();
}

/**
 * @brief Configure hardware-level failsafe PWM values for Advanced Failsafe termination
 * 
 * @details This function programs the hardware I/O processor (IOMCU on boards with one,
 *          or main processor PWM peripherals) with failsafe PWM output values that will
 *          be automatically applied if the main processor becomes unresponsive or the
 *          hardware termination pin is activated.
 *          
 *          Hardware Termination Logic:
 *          The hardware failsafe provides an independent safety layer that operates
 *          without software intervention. When triggered (via termination pin or CPU
 *          watchdog timeout), the I/O hardware automatically applies the configured
 *          failsafe PWM values, ensuring vehicle termination even if:
 *          - Main CPU is locked up or crashed
 *          - Software is in infinite loop
 *          - Memory corruption prevents normal failsafe execution
 *          - Power supply to main CPU is interrupted (on systems with separate IOMCU power)
 *          
 *          Configuration Strategy:
 *          - Auxiliary channels (servos): Set to TRIM (middle position) for safe de-energization
 *          - Motor outputs: Set to minimum PWM (typically 1000μs) to stop ESCs
 *          - Helicopter-specific channels safed to prevent rotor spin-up
 *          
 *          This function is called during ArduCopter initialization to pre-program the
 *          hardware failsafe registers before any flight operations begin.
 * 
 * @note Hardware failsafe is independent of software failsafe - both layers provide
 *       redundant protection against different failure modes
 * @note IOMCU-equipped boards (e.g., Pixhawk) maintain failsafe outputs even if main
 *       FMU processor loses power, providing additional safety margin
 * 
 * @warning Incorrect failsafe PWM values could cause unintended servo movements during
 *          termination. Verify TRIM values are safe for all connected peripherals.
 * @warning On helicopters (HELI_FRAME), motor failsafe is NOT configured as rotor control
 *          is handled through RSC servo channels rather than direct motor outputs.
 * 
 * @see terminate_vehicle() for software-initiated termination sequence
 * @see AP_HAL::RCOutput::set_failsafe_pwm() for hardware PWM configuration
 * @see libraries/SRV_Channel for servo channel management
 * 
 * Source: ArduCopter/afs_copter.cpp:36-51
 */
void AP_AdvancedFailsafe_Copter::setup_IO_failsafe(void)
{
    // Configure failsafe output values for all auxiliary servo channels
    // TRIM position (typically 1500μs PWM) is safe middle position for most servos
    
    // Helicopter main rotor speed controller - idle position prevents rotor acceleration
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_rsc, SRV_Channel::Limit::TRIM);
    
    // Helicopter tail rotor speed controller - idle position maintains tail control authority during autorotation
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::Limit::TRIM);
    
    // Engine run enable - typically disables internal combustion engine ignition/fuel
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::TRIM);
    
    // Ignition system control - safe position disables spark for gas engines
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_ignition, SRV_Channel::Limit::TRIM);
    
    // Generic unassigned servo channels - middle position is mechanically safe for most servos
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    
    // Manual passthrough channels - middle position prevents unexpected control surface deflections
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);

#if FRAME_CONFIG != HELI_FRAME
    // For multicopters (non-helicopter frames), configure motor output failsafe values
    // Helicopters use RSC servos for rotor control, so motor failsafe doesn't apply
    
    // Get bitmask of active motor outputs from motor library
    uint32_t mask = copter.motors->get_motor_mask();
    
    // Program hardware PWM peripherals with minimum PWM value for all motors
    // This ensures ESCs receive stop signal (typically 1000μs) during hardware failsafe
    // Minimum PWM is configured via MOT_PWM_MIN parameter (default 1000)
    hal.rcout->set_failsafe_pwm(mask, copter.motors->get_pwm_output_min());
#endif
}

/**
 * @brief Query the current flight mode's Advanced Failsafe control mode classification
 * 
 * @details The Advanced Failsafe system needs to know the vehicle's control authority
 *          level to determine appropriate failsafe actions. Different flight modes have
 *          different levels of autonomy and recovery capability:
 *          
 *          AFS Control Mode Classifications:
 *          - AFS_AUTO: Fully autonomous modes (AUTO, GUIDED, RTL, SMART_RTL, LAND)
 *            Vehicle can navigate and recover without pilot input
 *          - AFS_STABILIZED: Pilot-controlled stabilized modes (STABILIZE, ALT_HOLD, LOITER)
 *            Requires active pilot control but provides stability assistance
 *          - AFS_MANUAL: Direct pilot control modes (ACRO)
 *            Minimal stabilization, pilot has direct authority
 *          
 *          The AFS system uses this classification to determine:
 *          - Whether to force AUTO mode on GCS failsafe (only from non-AUTO modes)
 *          - Whether pilot intervention is available to recover from failures
 *          - Appropriate timeout values for different failure conditions
 *          
 *          This function delegates to the current flight mode object, allowing each
 *          mode to report its own AFS classification. The classification is checked
 *          continuously during AFS monitoring to adapt failsafe behavior to current
 *          vehicle control state.
 * 
 * @return AP_AdvancedFailsafe::control_mode - Current mode's AFS classification
 *         (AFS_AUTO, AFS_STABILIZED, or AFS_MANUAL)
 * 
 * @note Each Copter flight mode implements its own afs_mode() method to report
 *       its classification - see mode.h for per-mode implementations
 * @note Mode classification affects failsafe timeout values and recovery actions
 * 
 * @see AP_AdvancedFailsafe::check() for how control mode affects failsafe logic
 * @see Mode::afs_mode() for base class implementation
 * @see set_mode_auto() for forced AUTO mode transition on datalink loss
 * 
 * Source: ArduCopter/afs_copter.cpp:56-59
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Copter::afs_mode(void)
{
    // Delegate to current flight mode's AFS classification
    // Each mode (AUTO, LOITER, STABILIZE, etc.) returns its appropriate AFS control level
    return copter.flightmode->afs_mode();
}

/**
 * @brief Force vehicle into AUTO mode as recovery action for datalink loss
 * 
 * @details When the Advanced Failsafe system detects prolonged Ground Control Station
 *          (GCS) communication loss and the vehicle is not already in an autonomous mode,
 *          this function transitions the vehicle to AUTO mode to execute the pre-programmed
 *          mission or return-to-launch sequence.
 *          
 *          Datalink Loss Trigger Conditions:
 *          - No MAVLink heartbeat received from GCS for AFS_RC_FAIL_TIME duration (default 3 seconds)
 *          - No RC input if AFS_RC_FAIL_TIME is configured and RC link is primary control
 *          - Manual termination command received but termination action set to AUTO recovery
 *          
 *          AUTO Mode Recovery Strategy:
 *          When datalink is lost, forcing AUTO mode allows the vehicle to:
 *          - Continue executing waypoint mission if one is loaded
 *          - Navigate to rally points if configured
 *          - Execute RTL (Return to Launch) if no mission is active
 *          - Maintain controlled flight without ground station input
 *          
 *          This provides graceful degradation: rather than immediately terminating or
 *          hovering in place, the vehicle attempts autonomous recovery. If the vehicle
 *          remains in datalink loss beyond additional timeout thresholds, more aggressive
 *          failsafe actions (LAND or TERMINATE) may be triggered.
 *          
 *          Mode Transition Safety:
 *          The transition uses ModeReason::GCS_FAILSAFE to indicate this is a failsafe
 *          action, which relaxes some mode entry checks (e.g., GPS quality requirements
 *          may be reduced to allow emergency recovery even with marginal GPS).
 * 
 * @note This function is only called when AFS determines AUTO mode recovery is appropriate
 *       - it is not called if already in AUTO, GUIDED, or RTL modes
 * @note If AUTO mode transition fails (e.g., no mission loaded, GPS lost), the AFS system
 *       will escalate to more aggressive failsafe actions after additional timeout
 * 
 * @warning This function does not validate pre-conditions for AUTO mode (GPS quality,
 *          mission validity) - it forces the transition regardless. This is intentional
 *          for emergency recovery, but may result in unpredictable behavior if GPS is poor.
 * @warning If no mission is loaded and no rally points configured, AUTO mode will typically
 *          execute RTL. Ensure mission planning accounts for datalink loss scenarios.
 * 
 * @see AP_AdvancedFailsafe::check() for datalink loss detection logic
 * @see afs_mode() for checking current control mode classification
 * @see terminate_vehicle() for ultimate termination action if recovery fails
 * @see Copter::set_mode() for mode transition implementation
 * 
 * Source: ArduCopter/afs_copter.cpp:62-65
 */
void AP_AdvancedFailsafe_Copter::set_mode_auto(void)
{
    // Force transition to AUTO mode with GCS_FAILSAFE reason
    // This indicates an emergency autonomous recovery attempt due to lost datalink
    // ModeReason::GCS_FAILSAFE relaxes some pre-arm checks to allow recovery with degraded state
    copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_FAILSAFE);
}
#endif // AP_COPTER_ADVANCED_FAILSAFE_ENABLED
