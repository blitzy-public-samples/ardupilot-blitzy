/**
 * @file mode.h
 * @brief ArduCopter flight mode framework and mode implementations
 * 
 * @details This file defines the complete flight mode system for ArduCopter, including:
 * - Mode base class that provides the polymorphic interface for all flight modes
 * - All flight mode implementations (STABILIZE, ACRO, ALT_HOLD, AUTO, GUIDED, etc.)
 * - Helper classes for takeoff, auto-yaw, and payload placement
 * - Mode lifecycle management (init → run repeatedly → exit)
 * - Mode capability queries (GPS requirements, manual throttle, arming permissions)
 * 
 * Architecture:
 * The mode system uses a polymorphic design where each flight mode inherits from the
 * Mode base class and implements mode-specific control logic. The Copter main class
 * maintains pointers to active mode instances and calls their virtual methods during
 * the main loop. Mode switching is handled through the set_mode() function which
 * calls exit() on the old mode, init() on the new mode, and updates the active pointer.
 * 
 * Mode Lifecycle:
 * 1. init(bool ignore_checks) - Called once when entering the mode
 *    - Returns true if mode entry successful, false to prevent mode change
 *    - ignore_checks parameter allows forcing mode entry (e.g., failsafe)
 * 2. run() - Called repeatedly at main loop rate (typically 400Hz) while in mode
 *    - Implements the mode's control logic
 * 3. exit() - Called once when leaving the mode
 *    - Cleanup and state reset
 * 
 * Mode Number Enumeration:
 * Each mode has a unique number (Mode::Number enum) used for:
 * - MAVLink protocol communication with ground stations
 * - Parameter storage and mission commands
 * - Mode switching via RC channels or GCS commands
 * 
 * Thread Safety:
 * Mode methods are called from the main vehicle thread. Some modes use
 * HAL_Semaphore for protection when accessing shared resources.
 * 
 * @note All angles in this system use NED (North-East-Down) coordinate frame
 * @note Units: distances in cm, velocities in cm/s, angles in radians unless noted
 * 
 * @see Copter.h for main vehicle class
 * @see AC_AttitudeControl for attitude controller integration
 * @see AC_PosControl for position controller integration
 * 
 * Source: ArduCopter/mode.h
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "Copter.h"
#include <AP_Math/chirp.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h> // TODO why is this needed if Copter.h includes this
#include <AP_HAL/Semaphores.h>

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
#include "afs_copter.h"
#endif

class Parameters;
class ParametersG2;

class GCS_Copter;

/**
 * @class _AutoTakeoff
 * @brief Automatic takeoff helper shared by Guided and Auto modes
 * 
 * @details This class implements automatic takeoff behavior where the position
 * controller manages horizontal and vertical position, but pilot retains yaw control.
 * Used by both AUTO and GUIDED modes for consistent takeoff behavior.
 * 
 * Takeoff Phases:
 * 1. No-Nav Phase: Below no_nav_alt_cm, only altitude control (no horizontal position hold)
 * 2. Full Nav Phase: Above no_nav_alt_cm, full 3D position control to target
 * 3. Completion: Reached complete_alt_cm altitude
 * 
 * @note Altitude can be specified relative to EKF origin or terrain
 * @warning complete flag must be checked to verify takeoff completion
 */
class _AutoTakeoff {
public:
    /**
     * @brief Execute auto-takeoff control logic (called at main loop rate)
     * 
     * @details Updates takeoff state machine, manages transition from no-nav to
     * full navigation, and sets complete flag when target altitude reached.
     * Should be called from mode's run() method during takeoff.
     */
    void run();
    
    /**
     * @brief Start automatic takeoff to specified altitude
     * 
     * @param[in] complete_alt_cm Target altitude in centimeters
     * @param[in] terrain_alt true if altitude is above terrain, false if above EKF origin
     * 
     * @note Resets complete flag and captures current position as takeoff origin
     */
    void start(float complete_alt_cm, bool terrain_alt);
    
    /**
     * @brief Get target completion position for takeoff
     * 
     * @param[out] pos_neu_cm Target position in NEU frame (cm from EKF origin)
     * @return true if completion position is valid
     * 
     * @note Used for displaying target position to GCS
     */
    bool get_completion_pos(Vector3p& pos_neu_cm);

    bool complete;          ///< true when takeoff is complete (target altitude reached)

private:
    /// true if currently in no-nav phase (below no_nav_alt_cm)
    bool no_nav_active;
    
    /// Altitude threshold (cm above EKF origin) below which horizontal position is not controlled
    /// Allows vehicle to lift off before engaging full position control
    float no_nav_alt_cm;

    /// Target completion altitude in cm (above EKF origin or terrain depending on terrain_alt)
    float complete_alt_cm;
    
    /// true if altitudes are relative to terrain, false if relative to EKF origin
    bool terrain_alt;
    
    /// Target takeoff position as offset from EKF origin in cm (NEU frame)
    Vector3p complete_pos;
};

#if AC_PAYLOAD_PLACE_ENABLED
/**
 * @class PayloadPlace
 * @brief Autonomous payload placement for precision delivery missions
 * 
 * @details Implements automated payload placement sequence used in AUTO mode
 * NAV_PAYLOAD_PLACE mission command. The sequence:
 * 1. FlyToLocation - Navigate to delivery location
 * 2. Descent_Start - Initialize descent parameters
 * 3. Descent - Descend while monitoring thrust to detect ground contact
 * 4. Release - Trigger payload release mechanism
 * 5. Releasing - Wait for release to complete
 * 6. Delay - Hold position after release
 * 7. Ascent_Start - Initialize climb parameters
 * 8. Ascent - Climb away from delivery location
 * 9. Done - Placement complete
 * 
 * Ground Detection:
 * Detects ground contact by monitoring thrust level during descent. When thrust
 * exceeds expected hover thrust (indicating vehicle is resting on ground), the
 * release sequence is triggered.
 * 
 * @note Requires AP_GRIPPER or similar payload release mechanism
 * @warning Descent speed and thrust thresholds must be tuned for vehicle mass and payload
 */
class PayloadPlace {
public:
    /**
     * @brief Execute payload place control logic (called at main loop rate)
     * 
     * @details Runs the appropriate control logic for current state. Manages
     * state transitions based on descent progress, ground contact detection,
     * and release completion.
     * 
     * @note Must be called from Auto mode's run() method when in NAV_PAYLOAD_PLACE
     */
    void run();
    
    /**
     * @brief Initialize descent phase parameters
     * 
     * @details Captures current altitude, configures descent speed, and initializes
     * thrust monitoring for ground contact detection.
     */
    void start_descent();
    
    /**
     * @brief Check if payload place sequence is complete
     * 
     * @return true if state is Done, false otherwise
     */
    bool verify();

    /**
     * @enum State
     * @brief Payload placement state machine states
     */
    enum class State : uint8_t {
        FlyToLocation,      ///< Navigate to target location
        Descent_Start,      ///< Initialize descent parameters
        Descent,            ///< Descending to ground with thrust monitoring
        Release,            ///< Trigger payload release mechanism
        Releasing,          ///< Wait for release to complete
        Delay,              ///< Hold position after release
        Ascent_Start,       ///< Initialize climb parameters
        Ascent,             ///< Climbing away from delivery location
        Done,               ///< Placement sequence complete
    };

    /// Current state of payload place sequence (set by Mission code)
    State state = State::Descent_Start;
    
    /// Maximum descent distance in cm (set by mission command parameter)
    float descent_max_cm;

private:
    /// Time when steady descent was established (milliseconds)
    uint32_t descent_established_time_ms;
    
    /// Time when payload place sequence started (milliseconds)
    uint32_t place_start_time_ms;
    
    /// Measured thrust level during descent (used for ground contact detection)
    float descent_thrust_level;
    
    /// Altitude when descent started in cm (above EKF origin)
    float descent_start_altitude_cm;
    
    /// Descent speed in cm/s (configurable via mission parameter)
    float descent_speed_cms;
};
#endif

/**
 * @class Mode
 * @brief Base class for all ArduCopter flight modes
 * 
 * @details The Mode class provides a polymorphic interface for all flight modes in ArduCopter.
 * Each flight mode inherits from this base class and implements mode-specific control logic
 * by overriding virtual methods.
 * 
 * Mode Architecture:
 * - Polymorphic design enables consistent mode switching and execution
 * - Each mode implements its own control logic in run() method
 * - Mode capabilities are queried through virtual methods (requires_GPS(), allows_arming(), etc.)
 * - Modes can share common helper functions and coordinate frame conversions
 * 
 * Mode Lifecycle:
 * 1. **Initialization**: init(ignore_checks) called when entering mode
 *    - Validates preconditions (GPS lock, altitude estimate, etc.)
 *    - Sets up initial control targets
 *    - Returns false to prevent mode entry if preconditions not met
 * 
 * 2. **Execution**: run() called repeatedly at main loop rate (typically 400Hz)
 *    - Reads pilot input (if applicable)
 *    - Computes control targets
 *    - Sends commands to attitude and position controllers
 *    - Updates mode-specific state
 * 
 * 3. **Cleanup**: exit() called when leaving mode
 *    - Resets mode-specific state
 *    - Stops active navigation commands
 * 
 * Mode Capability Queries:
 * Virtual methods allow the main vehicle code to query mode capabilities:
 * - requires_GPS(): true if mode needs valid GPS position
 * - has_manual_throttle(): true if pilot directly controls throttle
 * - allows_arming(): true if vehicle can be armed in this mode
 * - is_autopilot(): true if mode is fully autonomous
 * - allows_autotune(): true if autotune can run in this mode
 * 
 * Common Patterns:
 * - Manual modes (STABILIZE, ACRO): Pilot controls attitude, manual throttle
 * - Assisted modes (ALT_HOLD, LOITER): Pilot controls with automated assistance
 * - Autonomous modes (AUTO, RTL, GUIDED): Full autopilot control
 * 
 * Class Hierarchy (Mermaid Diagram):
 * ```mermaid
 * classDiagram
 *     class Mode {
 *         +init(ignore_checks) bool
 *         +run() void
 *         +exit() void
 *         +requires_GPS() bool
 *         +has_manual_throttle() bool
 *         +allows_arming() bool
 *         +is_autopilot() bool
 *         +mode_number() Number
 *     }
 *     
 *     Mode <|-- ModeStabilize : Manual Control
 *     Mode <|-- ModeAcro : Manual Control
 *     Mode <|-- ModeAltHold : Assisted
 *     Mode <|-- ModeLoiter : Assisted
 *     Mode <|-- ModePosHold : Assisted
 *     Mode <|-- ModeGuided : Autonomous
 *     Mode <|-- ModeAuto : Autonomous
 *     Mode <|-- ModeRTL : Autonomous
 *     Mode <|-- ModeLand : Autonomous
 *     Mode <|-- ModeCircle : Autonomous
 *     Mode <|-- ModeBrake : Assisted
 *     Mode <|-- ModeSport : Assisted
 *     Mode <|-- ModeFlip : Automated Maneuver
 *     Mode <|-- ModeAutoTune : Automated Tuning
 *     Mode <|-- ModeThrow : Launch
 *     Mode <|-- ModeDrift : Assisted
 *     Mode <|-- ModeZigZag : Semi-Auto
 *     Mode <|-- ModeSmartRTL : Autonomous
 *     ModeGuided <|-- ModeGuidedNoGPS : No GPS variant
 *     ModeGuided <|-- ModeAvoidADSB : Avoidance
 *     ModeGuided <|-- ModeFollow : Following
 *     ModeRTL <|-- ModeSmartRTL : Path retracing
 * ```
 * 
 * Thread Safety:
 * Mode methods are called from the main vehicle thread. Access to shared
 * resources (especially HAL peripherals) should use appropriate semaphores.
 * 
 * Coordinate Frames:
 * - All position targets in NED (North-East-Down) frame
 * - Body frame for attitude representation
 * - Earth frame for velocities
 * 
 * Units Convention:
 * - Distances: centimeters (cm)
 * - Velocities: centimeters/second (cm/s)
 * - Angles: radians (rad) unless suffixed with _cd (centidegrees)
 * - Angular rates: radians/second (rad/s)
 * 
 * @note Mode switching handled by Copter::set_mode()
 * @warning Modes must not block in run() - executes at main loop rate
 * @warning Mode implementations must handle edge cases like GPS loss gracefully
 * 
 * @see Copter::set_mode() for mode switching logic
 * @see AC_AttitudeControl for attitude controller interface
 * @see AC_PosControl for position controller interface
 * @see AC_WPNav for waypoint navigation
 */
class Mode {
    friend class PayloadPlace;

public:

    /**
     * @enum Number
     * @brief Flight mode enumeration for mode identification
     * 
     * @details Each mode has a unique number used for:
     * - MAVLink protocol communication (HEARTBEAT, SET_MODE messages)
     * - Parameter storage (e.g., FLTMODE1-6 parameters)
     * - Mission commands (e.g., DO_SET_MODE)
     * - RC channel mode switching
     * - Logging and telemetry
     * 
     * @note Mode numbers should never change once assigned (backward compatibility)
     * @note Some numbers are reserved (30 for offboard control, 127 for drone shows)
     * @warning Gaps in numbering are intentional - do not renumber
     */
    enum class Number : uint8_t {
        STABILIZE =     0,  ///< Manual airframe angle with manual throttle (most basic flight mode)
        ACRO =          1,  ///< Manual body-frame angular rate with manual throttle (for aerobatics)
        ALT_HOLD =      2,  ///< Manual airframe angle with automatic altitude hold
        AUTO =          3,  ///< Fully automatic waypoint control using mission commands
        GUIDED =        4,  ///< Fully automatic fly to coordinate or velocity using GCS commands
        LOITER =        5,  ///< Automatic position hold with manual pilot override
        RTL =           6,  ///< Automatic return to launch point
        CIRCLE =        7,  ///< Automatic circular flight with automatic throttle
        LAND =          9,  ///< Automatic landing with horizontal position control
        DRIFT =        11,  ///< Semi-autonomous position, yaw and throttle control (intuitive flight)
        SPORT =        13,  ///< Manual earth-frame angular rate control with automatic altitude
        FLIP =         14,  ///< Automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  ///< Automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  ///< Automatic position hold with manual override and braking
        BRAKE =        17,  ///< Full-brake using inertial/GPS system, no pilot input
        THROW =        18,  ///< Throw to launch mode using inertial/GPS system
        AVOID_ADSB =   19,  ///< Automatic avoidance of obstacles (e.g., full-sized aircraft)
        GUIDED_NOGPS = 20,  ///< Guided mode but only accepts attitude and altitude (no position)
        SMART_RTL =    21,  ///< Returns to home by retracing flight path
        FLOWHOLD  =    22,  ///< Position hold using optical flow without rangefinder
        FOLLOW    =    23,  ///< Follow another vehicle or ground station
        ZIGZAG    =    24,  ///< Fly in a zigzag pattern with predefined points A and B
        SYSTEMID  =    25,  ///< System identification mode for automated frequency sweeps
        AUTOROTATE =   26,  ///< Autonomous autorotation for helicopter emergency landing
        AUTO_RTL =     27,  ///< Auto RTL (not a true mode - AUTO reports this during DO_LAND_START)
        TURTLE =       28,  ///< Flip over after crash (turtle mode recovery)

        // Mode number 30 reserved for "offboard" for external/lua control.

        // Mode number 127 reserved for the "drone show mode" in the Skybrush
        // fork at https://github.com/skybrush-io/ardupilot
    };

    /**
     * @brief Mode base class constructor
     * 
     * @details Initializes convenience references to Copter singleton members
     * (g, g2, wp_nav, pos_control, etc.). Called by subclass constructors.
     */
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    friend class _AutoTakeoff;

    /**
     * @brief Get unique mode number for this mode
     * 
     * @return Mode::Number enum value identifying this mode
     * 
     * @note Pure virtual - must be implemented by all subclasses
     * @note Used for MAVLink communication, logging, and mode identification
     */
    virtual Number mode_number() const = 0;

    // child classes should override these methods
    
    /**
     * @brief Initialize the mode when entering
     * 
     * @param[in] ignore_checks If true, bypass safety checks (used for failsafe entry)
     * @return true if mode entry successful, false to prevent mode change
     * 
     * @details Called once when entering this mode. Implementations should:
     * - Check preconditions (GPS lock, sensors, etc.) unless ignore_checks is true
     * - Initialize mode-specific state variables
     * - Set initial control targets (position, velocity, attitude)
     * - Return false to prevent unsafe mode entry (unless forced by ignore_checks)
     * 
     * Default implementation returns true (mode entry always succeeds).
     * 
     * @note Called from Copter::set_mode() before mode becomes active
     * @warning If returning false, old mode remains active
     */
    virtual bool init(bool ignore_checks) {
        return true;
    }
    
    /**
     * @brief Clean up when exiting the mode
     * 
     * @details Called once when leaving this mode. Implementations should:
     * - Reset mode-specific state
     * - Release resources
     * - Clean up any timers or state machines
     * 
     * Default implementation does nothing.
     * 
     * @note Called from Copter::set_mode() after new mode has been initialized
     */
    virtual void exit() {};
    
    /**
     * @brief Execute mode control logic at main loop rate
     * 
     * @details Called repeatedly (typically at 400Hz) while in this mode.
     * Implementations should:
     * - Read pilot inputs (if applicable)
     * - Update navigation targets
     * - Run control loops (attitude, position, velocity)
     * - Call output_to_motors() to send commands to motors
     * - Update mode state machines
     * 
     * @note Pure virtual - must be implemented by all subclasses
     * @warning Must not block - executes in main loop
     * @warning Must complete quickly (< 2.5ms typically)
     * 
     * @see output_to_motors() for motor output
     */
    virtual void run() = 0;
    
    /**
     * @brief Check if this mode requires GPS for operation
     * 
     * @return true if GPS lock required, false if mode can operate without GPS
     * 
     * @details Used to:
     * - Prevent mode entry if GPS unavailable (unless forced)
     * - Trigger GPS failsafe if GPS lost during flight
     * - Display GPS requirements to pilot
     * 
     * Examples:
     * - STABILIZE: false (can fly without GPS)
     * - LOITER: true (needs GPS for position hold)
     * - AUTO: true (needs GPS for waypoint navigation)
     * 
     * @note Pure virtual - must be implemented by all subclasses
     */
    virtual bool requires_GPS() const = 0;
    
    /**
     * @brief Check if this mode uses manual throttle control
     * 
     * @return true if pilot controls throttle directly, false if automatic
     * 
     * @details Affects:
     * - Throttle stick input handling
     * - Hover throttle learning
     * - Arming checks (throttle position requirements)
     * - Altitude hold behavior
     * 
     * Examples:
     * - STABILIZE: true (pilot controls throttle)
     * - ALT_HOLD: false (automatic altitude control)
     * - LOITER: false (automatic altitude control)
     * 
     * @note Pure virtual - must be implemented by all subclasses
     */
    virtual bool has_manual_throttle() const = 0;
    
    /**
     * @brief Check if arming is allowed in this mode
     * 
     * @param[in] method Arming method (PWM, MAVLink, switch, etc.)
     * @return true if arming permitted, false if not allowed
     * 
     * @details Used to prevent arming in unsafe modes:
     * - AUTO: false (prevent accidental mission start)
     * - LAND: false (landing mode not for takeoff)
     * - BRAKE: false (transient mode)
     * - STABILIZE: true (safe for arming)
     * 
     * @note Pure virtual - must be implemented by all subclasses
     * @warning Returning false prevents arming regardless of pre-arm checks
     */
    virtual bool allows_arming(AP_Arming::Method method) const = 0;
    
    /**
     * @brief Check if this is an autopilot mode
     * 
     * @return true if autonomous mode, false if pilot-controlled
     * 
     * @details Autopilot modes:
     * - Make autonomous navigation decisions
     * - Typically require GPS
     * - Continue without pilot input
     * - Examples: AUTO, GUIDED, RTL, LAND
     * 
     * Non-autopilot modes:
     * - Require continuous pilot input
     * - Pilot has direct control
     * - Examples: STABILIZE, ACRO, ALT_HOLD, LOITER
     * 
     * Default: false (pilot-controlled)
     */
    virtual bool is_autopilot() const { return false; }
    
    /**
     * @brief Check if mode supports user-initiated takeoff
     * 
     * @param[in] must_navigate true if takeoff must include navigation to target position
     * @return true if mode supports user takeoff, false otherwise
     * 
     * @details User takeoff triggered by:
     * - Raising throttle above mid-stick
     * - Mode-specific takeoff logic
     * 
     * Modes supporting user takeoff:
     * - ALT_HOLD: true (altitude-based takeoff)
     * - LOITER: true (position-based takeoff)
     * - GUIDED: true (guided takeoff)
     * 
     * Default: false (no user takeoff support)
     * 
     * @see do_user_takeoff() for takeoff execution
     */
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    
    /**
     * @brief Check if currently in guided mode or guided-like control
     * 
     * @return true if in guided mode, false otherwise
     * 
     * @details Returns true for:
     * - GUIDED mode
     * - AUTO mode with NAV_GUIDED submodes
     * - Modes accepting external position/velocity commands
     * 
     * Default: false
     */
    virtual bool in_guided_mode() const { return false; }
    
    /**
     * @brief Check if mode should log attitude targets at higher rate
     * 
     * @return true to enable high-rate attitude logging, false for normal rate
     * 
     * @details Used by system identification and tuning modes to log
     * attitude data at higher frequency for analysis.
     * 
     * Default: false (normal logging rate)
     */
    virtual bool logs_attitude() const { return false; }
    
    /**
     * @brief Check if mode allows saving trim values
     * 
     * @return true if trim can be saved, false otherwise
     * 
     * @details Trim saving captures current stick positions as neutral.
     * Only allowed in stable, hovering modes:
     * - STABILIZE: true
     * - ACRO: true
     * - ALT_HOLD: true
     * - Autonomous modes: false
     * 
     * Default: false
     */
    virtual bool allows_save_trim() const { return false; }
    
    /**
     * @brief Check if mode allows automatic trim adjustment
     * 
     * @return true if auto-trim enabled, false otherwise
     * 
     * @details Auto-trim automatically adjusts trim based on pilot inputs
     * to minimize stick deflection needed to hover.
     * 
     * Default: false
     */
    virtual bool allows_auto_trim() const { return false; }
    
    /**
     * @brief Check if mode allows running autotune
     * 
     * @return true if autotune can run, false otherwise
     * 
     * @details Autotune can be initiated from modes with:
     * - Stable hover capability
     * - Manual pilot override ability
     * - Altitude control
     * 
     * Allowed modes: ALT_HOLD, LOITER, POSHOLD
     * 
     * Default: false
     */
    virtual bool allows_autotune() const { return false; }
    
    /**
     * @brief Check if mode allows flip maneuver
     * 
     * @return true if flip allowed, false otherwise
     * 
     * @details Flip can be initiated from modes that:
     * - Allow aerobatic maneuvers
     * - Have manual or assisted control
     * 
     * Allowed modes: STABILIZE, ACRO, ALT_HOLD, SPORT
     * 
     * Default: false
     */
    virtual bool allows_flip() const { return false; }
    
    /**
     * @brief Check if crash detection is enabled for this mode
     * 
     * @return true if crash check active, false if disabled
     * 
     * @details Crash detection monitors:
     * - Excessive lean angles
     * - High rotation rates while landed
     * - Impact detection
     * 
     * Disabled in: FLIP, ACRO, THROW (expected acrobatic motion)
     * Enabled in: most other modes
     * 
     * Default: true (crash check enabled)
     */
    virtual bool crash_check_enabled() const { return true; }

    /**
     * @brief Check if mode can be entered during RC failsafe
     * 
     * @return true if entry allowed without pilot input, false if pilot input required
     * 
     * @details RC failsafe occurs when RC link is lost. Safe modes allow failsafe entry:
     * - ALT_HOLD: true (maintains altitude, safe)
     * - RTL: true (returns home automatically)
     * - LAND: true (lands automatically)
     * - ACRO: false (requires pilot input)
     * - STABILIZE: false (requires pilot input)
     * 
     * Default: true (allows failsafe entry)
     * 
     * @note "no pilot input" means RC link lost or invalid
     */
    virtual bool allows_entry_in_rc_failsafe() const { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    /**
     * @brief Return advanced failsafe mode classification
     * 
     * @return AP_AdvancedFailsafe_Copter::control_mode enum value
     * 
     * @details Advanced Failsafe (AFS) classifies modes for failsafe behavior:
     * - AFS_STABILIZED: Manual modes (STABILIZE, ACRO)
     * - AFS_AUTO: Autonomous modes (AUTO, GUIDED, RTL)
     * 
     * Used by advanced failsafe to determine appropriate failsafe actions.
     * 
     * Default: AFS_STABILIZED
     */
    virtual AP_AdvancedFailsafe_Copter::control_mode afs_mode() const { return AP_AdvancedFailsafe_Copter::control_mode::AFS_STABILIZED; }
#endif

    /**
     * @brief Check if high-throttle arming check can be skipped for GCS/Script arming
     * 
     * @return true if high-throttle arming allowed, false if throttle must be low
     * 
     * @details Normal arming requires throttle at minimum. Some autonomous modes
     * allow high-throttle arming for immediate takeoff:
     * - AUTO: true (can arm and start mission immediately)
     * - GUIDED: true (can arm and execute commanded motion)
     * - STABILIZE: false (throttle must be low)
     * 
     * Only applies to GCS (MAVLink) and Scripting arming, not RC arming.
     * 
     * Default: false (throttle must be low)
     * 
     * @warning High-throttle arming can cause immediate takeoff
     */
    virtual bool allows_GCS_or_SCR_arming_with_throttle_high() const { return false; }

#if FRAME_CONFIG == HELI_FRAME
    /**
     * @brief Check if mode allows inverted flight (helicopters only)
     * 
     * @return true if inverted flight permitted, false otherwise
     * 
     * @details Some helicopter modes support inverted (upside-down) flight:
     * - STABILIZE: true (aerobatic capability)
     * - ALT_HOLD: true (altitude hold while inverted)
     * - LOITER: true (position hold while inverted)
     * - AUTO: true (inverted waypoint navigation)
     * 
     * Default: false (no inverted flight)
     * 
     * @note Only applicable to helicopter frame configurations
     */
    virtual bool allows_inverted() const { return false; };
#endif

    /**
     * @brief Get full mode name string
     * 
     * @return Null-terminated string with full mode name
     * 
     * @details Used for:
     * - Ground station display
     * - Logging
     * - User messages
     * 
     * Examples: "STABILIZE", "LOITER", "GUIDED"
     * 
     * @note Pure virtual - must be implemented by all subclasses
     */
    virtual const char *name() const = 0;
    
    /**
     * @brief Get 4-character abbreviated mode name
     * 
     * @return Null-terminated string with 4-character mode name
     * 
     * @details Used for:
     * - OSD display (limited space)
     * - Compact logging
     * - MAVLink telemetry
     * 
     * Examples: "STAB", "LOIT", "GUID"
     * 
     * @note Pure virtual - must be implemented by all subclasses
     * @note Should be exactly 4 characters for consistent display
     */
    virtual const char *name4() const = 0;

    /**
     * @brief Execute user-initiated takeoff
     * 
     * @param[in] takeoff_alt_cm Target altitude for takeoff in cm (above home or terrain)
     * @param[in] must_navigate true if takeoff requires navigation to target position
     * @return true if takeoff initiated, false if conditions not met
     * 
     * @details Initiates takeoff sequence when pilot raises throttle or commands takeoff.
     * Behavior varies by mode:
     * - ALT_HOLD: Climbs to target altitude, no horizontal navigation
     * - LOITER: Climbs to altitude while maintaining position
     * - GUIDED: Climbs to altitude, may navigate to target position
     * 
     * @note Calls do_user_takeoff_start() virtual method for mode-specific setup
     * @see has_user_takeoff() to check if mode supports user takeoff
     */
    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    
    /**
     * @brief Check if vehicle is currently taking off
     * 
     * @return true if in takeoff phase, false otherwise
     * 
     * @details Returns true during:
     * - Auto-takeoff sequence (AUTO, GUIDED)
     * - User takeoff climb (ALT_HOLD, LOITER)
     * - Throw mode launch detection and stabilization
     * 
     * Used to:
     * - Defer landing gear retraction until takeoff complete
     * - Modify control gains during takeoff
     * - Prevent certain actions during critical takeoff phase
     * 
     * Default: false (not taking off)
     */
    virtual bool is_taking_off() const;
    
    /**
     * @brief Stop user takeoff sequence
     * 
     * @details Stops the shared user takeoff state machine used by
     * modes that support has_user_takeoff().
     * 
     * @note Static method - affects shared takeoff state
     */
    static void takeoff_stop() { takeoff.stop(); }

    /**
     * @brief Check if vehicle is currently landing
     * 
     * @return true if in landing phase, false otherwise
     * 
     * @details Returns true during:
     * - LAND mode operation
     * - AUTO mode landing sequence
     * - RTL final descent and landing
     * 
     * Used to:
     * - Trigger landing detection
     * - Extend landing gear
     * - Modify control behavior near ground
     * - Enable precision landing
     * 
     * Default: false (not landing)
     */
    virtual bool is_landing() const { return false; }

    /**
     * @brief Check if mode requires terrain data for safe operation
     * 
     * @return true if terrain failsafe should trigger if terrain data lost, false otherwise
     * 
     * @details Modes requiring terrain data:
     * - AUTO: true (for terrain-following missions)
     * - GUIDED: true (for terrain-relative commands)
     * - RTL: true (for terrain-aware return paths)
     * 
     * If terrain data lost and this returns true, vehicle may:
     * - Switch to failsafe mode
     * - Abort terrain-relative navigation
     * - Use barometric altitude as backup
     * 
     * Default: false (terrain not required)
     * 
     * @note Terrain data from rangefinder or terrain database
     */
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    
    /**
     * @brief Get current waypoint or target location
     * 
     * @param[out] loc Location structure to fill with waypoint
     * @return true if waypoint available, false if mode has no waypoint
     * 
     * @details Returns current navigation target for:
     * - AUTO: Current mission waypoint
     * - GUIDED: Commanded target position
     * - RTL: Home or rally point location
     * - LOITER: Loiter center point
     * 
     * Used for GCS display and telemetry.
     * 
     * Default: false (no waypoint)
     */
    virtual bool get_wp(Location &loc) const { return false; };
    
    /**
     * @brief Get bearing to waypoint in degrees
     * 
     * @return Bearing from vehicle to waypoint (0-360 degrees, 0=North)
     * 
     * @details Compass bearing to current navigation target.
     * Used for GCS navigation display and telemetry.
     * 
     * Default: 0 (no bearing)
     */
    virtual float wp_bearing_deg() const { return 0; }
    
    /**
     * @brief Get distance to waypoint in meters
     * 
     * @return Horizontal distance to waypoint in meters
     * 
     * @details Distance to current navigation target.
     * Used for GCS distance-to-waypoint display.
     * 
     * Default: 0.0 (no distance)
     */
    virtual float wp_distance_m() const { return 0.0f; }
    
    /**
     * @brief Get crosstrack error in meters
     * 
     * @return Perpendicular distance from desired path in meters (positive = right of path)
     * 
     * @details Crosstrack error is the perpendicular distance from the vehicle
     * to the ideal straight-line path between waypoints. Used to evaluate
     * navigation accuracy.
     * 
     * Default: 0.0 (no crosstrack)
     * 
     * @note Typically provided by wp_nav or pos_control
     */
    virtual float crosstrack_error() const { return 0.0f;}

    // functions to support MAV_CMD_DO_CHANGE_SPEED
    
    /**
     * @brief Set horizontal speed limit
     * 
     * @param[in] speed_xy_cms Desired horizontal speed in cm/s
     * @return true if speed set successfully, false if not supported
     * 
     * @details Sets maximum horizontal velocity for navigation modes.
     * Supported in:
     * - AUTO: Overrides mission speed
     * - GUIDED: Sets navigation speed
     * - RTL: Overrides return speed
     * 
     * Not supported in manual modes (STABILIZE, ACRO, etc.)
     * 
     * Default: false (not supported)
     * 
     * @note Speed persists until changed or mode exit
     */
    virtual bool set_speed_xy_cms(float speed_xy_cms) {return false;}
    
    /**
     * @brief Set climb speed limit
     * 
     * @param[in] speed_up_cms Desired climb speed in cm/s (positive = up)
     * @return true if speed set successfully, false if not supported
     * 
     * @details Sets maximum climb velocity for navigation modes.
     * Supported in AUTO, GUIDED, RTL modes.
     * 
     * Default: false (not supported)
     * 
     * @note Parameter name is misleading - should be speed_up_cms not speed_xy_cms
     */
    virtual bool set_speed_up_cms(float speed_xy_cms) {return false;}
    
    /**
     * @brief Set descent speed limit
     * 
     * @param[in] speed_down_cms Desired descent speed in cm/s (positive = down)
     * @return true if speed set successfully, false if not supported
     * 
     * @details Sets maximum descent velocity for navigation modes.
     * Supported in AUTO, GUIDED, RTL modes.
     * 
     * Default: false (not supported)
     * 
     * @note Parameter name is misleading - should be speed_down_cms not speed_xy_cms
     */
    virtual bool set_speed_down_cms(float speed_xy_cms) {return false;}

    /**
     * @brief Get altitude above ground level
     * 
     * @return Altitude above ground in cm, or altitude above home if terrain unavailable
     * 
     * @details Returns altitude above:
     * - Terrain (if rangefinder or terrain database available)
     * - Home (as fallback)
     * 
     * Used for:
     * - Terrain following
     * - Landing height determination
     * - Obstacle clearance checks
     * - Display to pilot
     * 
     * @note May be overridden by modes with specific altitude sources (e.g., AUTO uses landing altitude)
     */
    virtual int32_t get_alt_above_ground_cm(void) const;

    // pilot input processing
    
    /**
     * @brief Convert pilot stick input to desired lean angles
     * 
     * @param[out] roll_out_rad Desired roll angle in radians
     * @param[out] pitch_out_rad Desired pitch angle in radians
     * @param[in] angle_max_rad Maximum lean angle allowed in radians
     * @param[in] angle_limit_rad Additional angle limit (e.g., fence limits) in radians
     * 
     * @details Reads pilot roll/pitch stick inputs and converts to desired lean angles:
     * - Applies expo curve for smooth control feel
     * - Respects angle_max (configured maximum lean)
     * - Respects angle_limit (additional constraints like fences)
     * - Accounts for simple mode and super-simple mode transformations
     * 
     * Input range: -1.0 to 1.0 from RC channels
     * Output: Constrained to min(angle_max_rad, angle_limit_rad)
     * 
     * @note Used by manual and assisted flight modes
     * @note Simple mode rotates inputs relative to pilot's view from home
     */
    void get_pilot_desired_lean_angles_rad(float &roll_out_rad, float &pitch_out_rad, float angle_max_rad, float angle_limit_rad) const;
    
    /**
     * @brief Get pilot's desired yaw rate from stick input
     * 
     * @return Desired yaw rate in radians/second
     * 
     * @details Reads pilot yaw stick and converts to desired rotation rate:
     * - Applies expo curve for smooth control
     * - Scales by configured maximum yaw rate
     * - Returns 0 if stick centered (within deadzone)
     * 
     * Input range: -1.0 to 1.0 from RC channel
     * Output: -max_yaw_rate to +max_yaw_rate (rad/s)
     * 
     * @note Used by modes with manual yaw control
     */
    float get_pilot_desired_yaw_rate_rads() const;
    
    /**
     * @brief Get pilot's desired velocity vector from stick input
     * 
     * @param[in] vel_max Maximum velocity magnitude in cm/s
     * @return Desired velocity vector in body frame (forward-right) in cm/s
     * 
     * @details Converts pilot roll/pitch stick inputs to desired velocity:
     * - Stick deflection maps to velocity magnitude (0 to vel_max)
     * - Stick direction maps to velocity direction (body frame)
     * - Used for velocity control modes (LOITER, POSHOLD)
     * 
     * @note Returns zero vector if sticks centered
     * @note Velocity in body frame: X=forward, Y=right
     */
    Vector2f get_pilot_desired_velocity(float vel_max) const;
    
    /**
     * @brief Get pilot's throttle input
     * 
     * @return Throttle value from 0.0 to 1.0
     * 
     * @details Reads throttle channel and returns normalized value:
     * - 0.0 = minimum throttle (channel PWM minimum)
     * - 0.5 = mid throttle (hover point)
     * - 1.0 = maximum throttle (channel PWM maximum)
     * 
     * Used by:
     * - Manual throttle modes (STABILIZE, ACRO)
     * - Throttle-based altitude change modes
     * 
     * @note Does not apply hover throttle learning
     */
    float get_pilot_desired_throttle() const;

    /**
     * @brief Adjust climb rate to avoid obstacles and altitude limits
     * 
     * @param[in] target_rate_cms Desired climb rate in cm/s (positive = up)
     * @return Adjusted climb rate in cm/s after applying avoidance
     * 
     * @details Modifies requested climb rate to:
     * - Avoid upward obstacles (detected by proximity sensors)
     * - Enforce altitude fence limits (maximum altitude)
     * - Prevent climbing into detected obstacles
     * - Allow descent even if obstacles above
     * 
     * Avoidance sources:
     * - Proximity sensors (lidar, sonar, etc.)
     * - Altitude fence configuration
     * - Object avoidance database
     * 
     * @note Returns 0 or negative if climb would violate limits
     * @note Always allows descent (negative rates pass through if no ground obstacle)
     */
    float get_avoidance_adjusted_climbrate_cms(float target_rate_cms);

    /**
     * @brief Send motor outputs to actuators
     * 
     * @details Sends computed motor commands to motors and servos:
     * - Calls motors->output() to send PWM signals
     * - Applies mixing (throttle + roll + pitch + yaw → individual motors)
     * - Enforces motor limits and saturation handling
     * - Manages motor spool state (stopped, ground idle, throttle unlimited)
     * 
     * Called at the end of mode's run() method.
     * 
     * Can be overridden by modes requiring special motor handling:
     * - TURTLE: Reverses motor directions for flip recovery
     * 
     * @note Default implementation calls motors->output()
     * @warning Must be called exactly once per loop iteration
     */
    virtual void output_to_motors();

    /**
     * @brief Check if pilot yaw input should control heading
     * 
     * @return true if pilot controls yaw, false if mode controls yaw automatically
     * 
     * @details Determines yaw control authority:
     * - true: Pilot yaw stick controls heading (most modes)
     * - false: Mode controls heading automatically (AUTO with IGNORE_PILOT_YAW option, some AUTO sub-modes)
     * 
     * When false:
     * - Pilot yaw input ignored
     * - Mode sets heading via auto_yaw system
     * - Used for precise automated maneuvers
     * 
     * Default: true (pilot controls yaw)
     * 
     * @note AUTO and GUIDED modes may override based on options
     */
    virtual bool use_pilot_yaw() const {return true; }

    /**
     * @brief Pause the current mode
     * 
     * @return true if mode paused successfully, false if not supported
     * 
     * @details Pauses autonomous navigation:
     * - AUTO: Stops mission progress, holds position
     * - GUIDED: Holds current position
     * 
     * While paused:
     * - Vehicle maintains position/altitude
     * - Mission waypoint sequence halted
     * - Can be resumed with resume()
     * 
     * Not supported by manual modes.
     * 
     * Default: false (pause not supported)
     */
    virtual bool pause() { return false; };
    
    /**
     * @brief Resume mode from paused state
     * 
     * @return true if mode resumed successfully, false if not paused or not supported
     * 
     * @details Resumes autonomous navigation from pause:
     * - AUTO: Continues mission from paused waypoint
     * - GUIDED: Resumes previous commanded action
     * 
     * Default: false (resume not supported)
     */
    virtual bool resume() { return false; };

    /**
     * @brief Configure safe motor/controller state for ground handling
     * 
     * @param[in] force_throttle_unlimited If true, force motors to throttle_unlimited spool state
     * 
     * @details Ensures safe vehicle state when on ground:
     * - Sets motors to ground idle or stopped (unless force_throttle_unlimited)
     * - Zeros integrators in attitude/position controllers to prevent windup
     * - Resets controller targets to current state
     * - Prepares for smooth takeoff transition
     * 
     * Called by modes during:
     * - Pre-takeoff waiting (armed, on ground, waiting for throttle)
     * - Ground idle state in altitude-hold modes
     * 
     * force_throttle_unlimited used when:
     * - User has raised throttle, ready for immediate takeoff
     * - Fast spool-up required for takeoff
     * 
     * @note Prevents sudden movements on takeoff by zeroing accumulated errors
     * @warning Do not call during flight - will cause attitude/altitude disruption
     */
    void make_safe_ground_handling(bool force_throttle_unlimited = false);

    /**
     * @brief Check if weathervaning is allowed in this mode
     * 
     * @return true if weathervaning permitted, false otherwise
     * 
     * @details Weathervaning automatically yaws vehicle into wind during position hold:
     * - Reduces drift in gusty conditions
     * - Minimizes control effort
     * - Improves hover efficiency
     * 
     * Allowed in modes with:
     * - Position control (LOITER, AUTO, GUIDED)
     * - No specific heading requirement
     * - Sufficient GPS quality
     * 
     * Not allowed in:
     * - Manual modes (STABILIZE, ACRO)
     * - Modes with heading control (AUTO waypoint approach)
     * - Modes with option flags disabling it
     * 
     * Default: false (not allowed)
     * 
     * @note Only available if WEATHERVANE_ENABLED is defined
     */
#if WEATHERVANE_ENABLED
    virtual bool allows_weathervaning() const { return false; }
#endif

protected:

    // helper functions
    
    /**
     * @brief Check if vehicle is disarmed or landed
     * 
     * @return true if motors disarmed or land_complete flag set, false otherwise
     * 
     * @details Used by modes to determine safe actions:
     * - Prevents in-flight maneuvers when on ground
     * - Allows mode-specific ground behavior
     * - Checks both arming state and land detection
     * 
     * Returns true if either:
     * - motors->armed() == false (vehicle disarmed)
     * - ap.land_complete == true (landed and settled)
     * 
     * @note Land_complete flag requires vehicle on ground with low throttle for timeout period
     */
    bool is_disarmed_or_landed() const;
    
    /**
     * @brief Zero throttle and relax attitude control
     * 
     * @param[in] spool_up If true, spool motors to ground idle; if false, stop motors
     * 
     * @details Configures controllers for safe ground state:
     * - Sets zero throttle to attitude controller
     * - Relaxes attitude control (no active stabilization)
     * - Spools motors to shutdown or ground idle
     * - Used when landed or landing complete
     * 
     * spool_up = false (default):
     * - Motors commanded to stop
     * - Used after landing complete
     * 
     * spool_up = true:
     * - Motors at ground idle
     * - Used during landing approach (ready to abort)
     * 
     * @note Does not disarm motors, only sets spool state
     */
    void zero_throttle_and_relax_ac(bool spool_up = false);
    
    /**
     * @brief Zero throttle but maintain attitude hold
     * 
     * @details Configures controllers for descent with attitude control:
     * - Sets zero climb rate to position controller
     * - Maintains active attitude stabilization
     * - Keeps motors at throttle_unlimited (ready for control)
     * 
     * Used during:
     * - Landing descent while maintaining attitude
     * - Ground contact detection
     * - Final landing phase before touchdown
     * 
     * Difference from zero_throttle_and_relax_ac():
     * - This maintains attitude control (can resist disturbances)
     * - zero_throttle_and_relax_ac() disables active control
     * 
     * @note Vehicle will settle onto ground with attitude stabilization active
     */
    void zero_throttle_and_hold_attitude();

    /**
     * @brief Calculate position where vehicle will stop if pilot input removed
     * 
     * @return Location of predicted stopping point (alt above EKF origin)
     * 
     * @details Predicts final position if controls released now:
     * - Uses current velocity and position
     * - Applies configured stopping deceleration
     * - Accounts for vehicle response time
     * - Calculates stopping distance in each axis
     * 
     * Used by:
     * - Fence breach detection (will vehicle stop inside fence?)
     * - GCS display of future position
     * - Terrain collision avoidance
     * 
     * Calculation assumes:
     * - Maximum deceleration applied
     * - No pilot input from now on
     * - Current altitude maintained
     * 
     * @note Returns current position if velocity near zero
     * @note Location altitude frame is above EKF origin
     */
    Location get_stopping_point() const;

    /**
     * @brief Execute horizontal position control during landing
     * 
     * @details Manages horizontal position during landing descent:
     * - Maintains position hold if GPS available
     * - Uses loiter controller for horizontal stabilization
     * - Gradually reduces horizontal control authority near ground
     * - Prepares for touchdown (reduces lean angles)
     * 
     * Behavior by altitude:
     * - High altitude: Full position control
     * - Near ground: Reduced lean angles for gentle touchdown
     * - On ground: Position control released
     * 
     * Called each loop iteration during landing
     * 
     * @note Requires GPS for position control; degrades gracefully without GPS
     */
    void land_run_horizontal_control();
    
    /**
     * @brief Execute vertical descent control during landing
     * 
     * @param[in] pause_descent If true, halt descent and hover at current altitude
     * 
     * @details Manages descent rate during landing:
     * - Starts with moderate descent rate (parameter LAND_SPEED)
     * - Slows descent as ground approaches (detected by rangefinder or expected altitude)
     * - Monitors motor output to detect ground contact
     * - Transitions to landed state when on ground
     * 
     * pause_descent = true:
     * - Halts descent (maintains current altitude)
     * - Used for obstacle detection or landing abort
     * - Maintains landing state (can resume descent)
     * 
     * Ground detection logic:
     * - Low motor output + low descent rate = ground contact
     * - Timeout at low altitude = ground contact
     * - Rangefinder shows ground proximity
     * 
     * @note Sets land_complete flag when ground contact confirmed
     */
    void land_run_vertical_control(bool pause_descent = false);
    
    /**
     * @brief Execute both horizontal and vertical landing control
     * 
     * @param[in] pause_descent If true, halt descent but maintain horizontal control
     * 
     * @details Convenience function combining horizontal and vertical control:
     * - Calls land_run_horizontal_control() for position hold
     * - Calls land_run_vertical_control() for descent management
     * - Maintains coordinated horizontal/vertical control during landing
     * 
     * @note Inline function - simply calls both control functions in sequence
     */
    void land_run_horiz_and_vert_control(bool pause_descent = false) {
        land_run_horizontal_control();
        land_run_vertical_control(pause_descent);
    }

#if AC_PAYLOAD_PLACE_ENABLED
    /**
     * @brief Payload place state machine instance
     * 
     * @details Shared payload place object used by AUTO and GUIDED modes:
     * - Manages automated payload delivery sequence
     * - Detects ground contact via thrust sensing
     * - Controls gripper/release mechanism
     * - Handles ascent after delivery
     * 
     * States: FlyToLocation → Descent → Release → Releasing → Delay → Ascent → Done
     * 
     * @note Static member - shared across all mode instances
     * @note Only available if AC_PAYLOAD_PLACE_ENABLED is defined
     */
    static PayloadPlace payload_place;
#endif

    /**
     * @brief Execute landing with optional precision landing
     * 
     * @param[in] pause_descent If true, halt descent but maintain position control
     * 
     * @details Selects landing strategy based on available sensors:
     * - If precision landing configured: Run precland_run() state machine
     * - Otherwise: Run standard land_run_horiz_and_vert_control()
     * 
     * Precision landing (if enabled):
     * - Uses IR-LOCK, optical flow, or companion computer for target tracking
     * - Guides vehicle to precise landing target
     * - Handles target loss and retry logic
     * - Falls back to normal landing if target lost
     * 
     * Normal landing:
     * - Maintains last known position
     * - Descends at configured rate
     * - Detects ground contact
     * 
     * @note Automatically selects appropriate landing method
     * @see precland_run() for precision landing state machine
     */
    void land_run_normal_or_precland(bool pause_descent = false);

#if AC_PRECLAND_ENABLED
    /**
     * @brief Navigate to precision landing retry position
     * 
     * @param[in] retry_pos Retry position in NED frame relative to EKF origin (meters)
     * 
     * @details Repositions vehicle for precision landing retry:
     * - Moves to new position for better target visibility
     * - Called by precision landing state machine after target loss
     * - Uses position controller for smooth repositioning
     * - Maintains altitude during horizontal reposition
     * 
     * Retry scenarios:
     * - Target lost during descent (move to last known good position)
     * - Target acquisition failed (move to search position)
     * - Target quality degraded (move for better angle/lighting)
     * 
     * @param retry_pos Expected format: Vector3f(North_m, East_m, Down_m)
     * 
     * @note Only called within precision landing state machine
     * @note Only available if AC_PRECLAND_ENABLED is defined
     */
    void precland_retry_position(const Vector3f &retry_pos);

    /**
     * @brief Execute precision landing state machine
     * 
     * @details Comprehensive precision landing controller:
     * - Manages all phases of precision landing
     * - Handles target acquisition, tracking, and approach
     * - Implements retry logic on target loss
     * - Falls back to normal landing if retries exhausted
     * 
     * State machine phases:
     * 1. Target acquisition (search for landing target)
     * 2. Target approach (horizontal correction to center on target)
     * 3. Descent (controlled descent while tracking target)
     * 4. Target loss recovery (retry or fallback)
     * 5. Final descent (commit to landing)
     * 
     * Safety features:
     * - Altitude-based retry limits (no retry below safe altitude)
     * - Retry count limits (prevent infinite retry loops)
     * - Automatic fallback to normal landing
     * - Maintains position control throughout
     * 
     * Supported target detection:
     * - IR-LOCK beacon
     * - Optical flow markers
     * - Companion computer vision
     * 
     * @note Call from any mode implementing precision landing (LAND, AUTO, RTL)
     * @note Only available if AC_PRECLAND_ENABLED is defined
     * @warning Requires valid position estimate (GPS or optical flow)
     */
    void precland_run();
#endif

    /**
     * @brief Get expected hover throttle value
     * 
     * @return Throttle value for hover (0.0 to 1.0 range)
     * 
     * @details Returns throttle needed to hover at current conditions:
     * - Typically 0.4 to 0.6 for most multirotors
     * - Learned automatically by throttle controller
     * - Accounts for battery voltage, air density, payload
     * - Updated continuously during flight
     * 
     * Used by:
     * - Altitude hold controller (feedforward term)
     * - Auto modes for efficient altitude maintenance
     * - Acro mode throttle curve (mid-stick position)
     * 
     * Default implementation:
     * - Returns motors->get_throttle_hover()
     * - Some modes may override (e.g., acro uses different source)
     * 
     * @note Virtual function - modes can override for special behavior
     * @note Value stored in motors library, persistent across modes
     */
    virtual float throttle_hover() const;

    /**
     * @enum AltHoldModeState
     * @brief Flight states for altitude-hold-based modes
     * 
     * @details State machine for modes with automatic altitude control:
     * - Used by ALT_HOLD, LOITER, SPORT modes
     * - Manages transitions from ground to flight
     * - Determines motor spool state and controller behavior
     * - Enables safe takeoff and landing sequences
     */
    enum class AltHoldModeState {
        MotorStopped,         ///< Motors stopped (disarmed or just landed)
        Takeoff,              ///< Active takeoff sequence (climbing to takeoff altitude)
        Landed_Ground_Idle,   ///< Landed, motors at ground idle, waiting for pilot input
        Landed_Pre_Takeoff,   ///< Landed, pilot raised throttle, ready for takeoff
        Flying                ///< In flight (normal altitude control active)
    };
    
    /**
     * @brief Determine current altitude-hold mode state
     * 
     * @param[in] target_climb_rate_cms Target climb rate in cm/s
     * 
     * @return Current AltHoldModeState based on flight conditions
     * 
     * @details Determines appropriate state based on:
     * - Arming state (disarmed → MotorStopped)
     * - Landed flag (on ground → Landed states)
     * - Throttle input (high throttle → Landed_Pre_Takeoff)
     * - Altitude and vertical velocity (climbing → Takeoff)
     * - Default → Flying
     * 
     * State transitions:
     * - MotorStopped → Landed_Ground_Idle (arm motors)
     * - Landed_Ground_Idle → Landed_Pre_Takeoff (pilot raises throttle)
     * - Landed_Pre_Takeoff → Takeoff (vehicle leaves ground)
     * - Takeoff → Flying (reach target altitude)
     * - Flying → Landed_Ground_Idle (land detected)
     * 
     * Used by modes to select appropriate controller behavior:
     * - MotorStopped: Zero output
     * - Takeoff: Climb rate control
     * - Landed: Ground idle with integrator reset
     * - Flying: Full altitude control
     * 
     * @note Common logic shared by ALT_HOLD, LOITER, SPORT modes
     */
    AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // ========================================================================
    // Convenience references to key vehicle subsystems
    // Initialized in Mode constructor from main Copter object
    // Avoid code churn from mode refactoring while maintaining access
    // ========================================================================
    
    Parameters &g;                          ///< Main parameter set (reference to copter.g)
    ParametersG2 &g2;                       ///< Secondary parameter set (reference to copter.g2)
    AC_WPNav *&wp_nav;                      ///< Waypoint navigation controller
    AC_Loiter *&loiter_nav;                 ///< Loiter position controller
    AC_PosControl *&pos_control;            ///< Position controller (XYZ)
    AP_AHRS &ahrs;                          ///< Attitude Heading Reference System
    AC_AttitudeControl *&attitude_control;  ///< Attitude controller (roll/pitch/yaw)
    MOTOR_CLASS *&motors;                   ///< Motor output mixing and control
    RC_Channel *&channel_roll;              ///< Roll stick input channel
    RC_Channel *&channel_pitch;             ///< Pitch stick input channel
    RC_Channel *&channel_throttle;          ///< Throttle stick input channel
    RC_Channel *&channel_yaw;               ///< Yaw stick input channel
    float &G_Dt;                            ///< Main loop delta time (seconds)

    // ========================================================================
    // Takeoff Infrastructure
    // ========================================================================
    // 
    // ArduCopter supports TWO distinct automatic takeoff systems:
    //
    // 1. "USER-TAKEOFF" - Simple altitude-based takeoff for manual modes
    //    - Available in: ALT_HOLD, LOITER, POSHOLD, SPORT, FLOWHOLD
    //    - Triggered by: Pilot raising throttle stick above mid
    //    - Behavior: Climb to fixed altitude, then resume normal control
    //    - Implementation: Mode::do_user_takeoff() and _TakeOff class below
    //    - Characteristics: No horizontal navigation, yaw controlled by pilot
    //
    // 2. "AUTO-TAKEOFF" - Waypoint navigation takeoff for autonomous modes
    //    - Available in: AUTO, GUIDED
    //    - Triggered by: Mission command or GCS command
    //    - Behavior: Navigate to target position while climbing
    //    - Implementation: _AutoTakeoff class (defined earlier in file)
    //    - Characteristics: Full 3D navigation, pilot yaw optionally permitted
    //
    // These systems are independent and serve different use cases.
    // ========================================================================

    /**
     * @class _TakeOff
     * @brief User-initiated takeoff state machine for manual modes
     * 
     * @details Implements simple altitude-based takeoff:
     * - Pilot raises throttle → start() called with target altitude
     * - Vehicle climbs at controlled rate to target altitude
     * - Upon reaching target, takeoff completes → stop() called
     * - Mode resumes normal altitude control
     * 
     * State shared across all mode instances (static member):
     * - Only one takeoff can be active at a time
     * - State persists across mode switches
     * - Allows takeoff to continue if mode changes during climb
     * 
     * Typical usage pattern:
     * ```cpp
     * if (do_user_takeoff(target_alt_cm, must_navigate)) {
     *     takeoff.do_pilot_takeoff(pilot_climb_rate);
     * }
     * ```
     */
    class _TakeOff {
    public:
        /**
         * @brief Start user takeoff sequence
         * @param[in] alt_cm Target altitude in cm above current position
         */
        void start(float alt_cm);
        
        /**
         * @brief Stop/abort takeoff sequence
         * @details Resets takeoff state, allows normal altitude control
         */
        void stop();
        
        /**
         * @brief Process pilot input during takeoff
         * @param[in,out] pilot_climb_rate Pilot's desired climb rate (modified for takeoff)
         * @details Overrides pilot climb rate during initial climb phase
         */
        void do_pilot_takeoff(float& pilot_climb_rate);
        
        /**
         * @brief Check if takeoff should trigger/continue
         * @param[in] target_climb_rate Current target climb rate
         * @return true if takeoff sequence should be active
         */
        bool triggered(float target_climb_rate) const;

        /**
         * @brief Check if takeoff is currently active
         * @return true if takeoff in progress
         */
        bool running() const { return _running; }
        
    private:
        bool _running;                  ///< true when takeoff sequence is active
        float take_off_start_alt;       ///< Altitude when takeoff started (cm)
        float take_off_complete_alt;    ///< Target altitude for takeoff completion (cm)
    };

    /**
     * @brief User-takeoff state (shared across all mode instances)
     * @details Static member ensures only one takeoff active at a time
     */
    static _TakeOff takeoff;

    /**
     * @brief Initialize user takeoff from derived mode
     * 
     * @param[in] takeoff_alt_cm Target takeoff altitude in cm
     * @return true if takeoff started successfully
     * 
     * @details Virtual function allowing modes to customize takeoff behavior.
     * Default implementation starts standard altitude-based takeoff.
     * Override in derived mode for special takeoff requirements.
     * 
     * @note Called by do_user_takeoff() if mode reports has_user_takeoff()==true
     */
    virtual bool do_user_takeoff_start(float takeoff_alt_cm);

    /**
     * @brief Auto-takeoff state (shared across AUTO and GUIDED modes)
     * @details Waypoint navigation with pilot yaw control during takeoff
     */
    static _AutoTakeoff auto_takeoff;

public:
    // ========================================================================
    // AutoYaw - Automatic Yaw Control for Autonomous Modes
    // ========================================================================
    
    /**
     * @class AutoYaw
     * @brief Automatic yaw control system for autonomous flight modes
     * 
     * @details Manages vehicle heading during autonomous navigation:
     * - Used by AUTO, GUIDED, RTL modes
     * - Supports multiple yaw control strategies
     * - Handles smooth transitions between yaw modes
     * - Integrates pilot yaw input when permitted
     * 
     * Common usage patterns:
     * - Waypoint navigation: LOOK_AT_NEXT_WP (point toward destination)
     * - Camera missions: ROI (point at region of interest)
     * - Panorama: FIXED (hold specific heading)
     * - Circle mode: CIRCLE (tangent to circle)
     * - Return home: LOOK_AHEAD (direction of travel)
     * 
     * Yaw mode selection:
     * - Set by mission commands (e.g., DO_SET_ROI)
     * - Set by GCS commands
     * - Automatically selected based on flight mode
     * - Pilot input can override in some modes
     * 
     * @note Instance shared across all autonomous modes
     * @note Yaw control independent of position control
     */
    class AutoYaw {

    public:

        /**
         * @enum Mode
         * @brief Yaw control modes for autonomous flight
         * 
         * @details Each mode implements different yaw behavior:
         * - HOLD: Maintain current heading (zero yaw rate)
         * - LOOK_AT_NEXT_WP: Point toward next waypoint continuously
         * - ROI: Point toward Region Of Interest (camera targeting)
         * - FIXED: Hold specific compass heading
         * - LOOK_AHEAD: Point in direction of motion
         * - RESET_TO_ARMED_YAW: Return to heading when armed
         * - ANGLE_RATE: Turn at specified rate to target angle
         * - RATE: Continuous rotation at specified rate
         * - CIRCLE: Tangent to circle (for orbit maneuvers)
         * - PILOT_RATE: Direct pilot yaw stick control
         * - WEATHERVANE: Point into wind (minimize drift)
         */
        enum class Mode {
            HOLD =             0,   ///< Hold current heading with zero yaw rate
            LOOK_AT_NEXT_WP =  1,   ///< Point toward next waypoint (updates as position changes)
            ROI =              2,   ///< Point toward Region Of Interest location
            FIXED =            3,   ///< Hold specific compass heading
            LOOK_AHEAD =       4,   ///< Point in direction of horizontal velocity
            RESET_TO_ARMED_YAW = 5, ///< Return to heading when motors were armed
            ANGLE_RATE =       6,   ///< Turn at rate from starting angle to target
            RATE =             7,   ///< Continuous turn at specified rate
            CIRCLE =           8,   ///< Tangent to circle path (from AC_Circle)
            PILOT_RATE =       9,   ///< Pilot yaw stick provides rate command
            WEATHERVANE =     10,   ///< Yaw into wind to minimize drift
        };

        /**
         * @brief Get current yaw control mode
         * @return Current AutoYaw::Mode
         */
        Mode mode() const { return _mode; }
        
        /**
         * @brief Set yaw mode to default for current flight mode
         * @param[in] rtl true if in RTL-type mode (affects default selection)
         */
        void set_mode_to_default(bool rtl);
        
        /**
         * @brief Set specific yaw control mode
         * @param[in] new_mode Desired yaw control mode
         */
        void set_mode(Mode new_mode);
        
        /**
         * @brief Get default yaw mode for current context
         * @param[in] rtl true if in RTL-type mode
         * @return Appropriate default Mode
         */
        Mode default_mode(bool rtl) const;

        /**
         * @brief Set continuous yaw rate
         * @param[in] turn_rate_rads Yaw rate in rad/s (positive = clockwise when viewed from above)
         * @details Used in RATE and ANGLE_RATE modes
         */
        void set_rate_rad(float turn_rate_rads);

        /**
         * @brief Set Region Of Interest for camera pointing
         * @param[in] roi_location Target location to point at (lat/lon/alt)
         * @details Sets mode to ROI, vehicle continuously points at this location
         */
        void set_roi(const Location &roi_location);

        /**
         * @brief Set fixed yaw target with controlled turn rate
         * 
         * @param[in] angle_rad Target heading in radians (0=North, clockwise positive)
         * @param[in] turn_rate_rads Maximum turn rate in rad/s
         * @param[in] direction Turn direction: 0=shortest, 1=clockwise, -1=counter-clockwise
         * @param[in] relative_angle true if angle is relative to current heading
         * 
         * @details Configures FIXED or ANGLE_RATE mode:
         * - If turn_rate_rads > 0: ANGLE_RATE mode (controlled rotation)
         * - If turn_rate_rads == 0: FIXED mode (jump to heading)
         * - Direction controls turn path when ambiguous
         */
        void set_fixed_yaw_rad(float angle_rad,
                               float turn_rate_rads,
                               int8_t direction,
                               bool relative_angle);

        /**
         * @brief Set yaw angle and rate together
         * @param[in] yaw_angle_rad Target heading in radians
         * @param[in] yaw_rate_rads Desired yaw rate in rad/s
         * @details Used for scripting and external control interfaces
         */
        void set_yaw_angle_and_rate_rad(float yaw_angle_rad, float yaw_rate_rads);

        /**
         * @brief Add offset to current yaw target
         * @param[in] yaw_angle_offset_deg Offset in degrees to add to current target
         * @details Allows incremental yaw adjustments without changing mode
         */
        void set_yaw_angle_offset_deg(const float yaw_angle_offset_deg);

        /**
         * @brief Check if fixed yaw target has been reached
         * @return true if vehicle heading is within tolerance of target
         * @details Used to determine waypoint completion in missions with yaw requirements
         */
        bool reached_fixed_yaw_target();

#if WEATHERVANE_ENABLED
        /**
         * @brief Update weathervaning yaw calculation
         * @param[in] pilot_yaw_rads Pilot's yaw rate input (rad/s)
         * @details Calculates heading to minimize wind drift, blends with pilot input
         * @note Only available if WEATHERVANE_ENABLED is defined
         */
        void update_weathervane(const float pilot_yaw_rads);
#endif

        /**
         * @brief Get heading command for attitude controller
         * @return AC_AttitudeControl::HeadingCommand structure
         * @details Converts AutoYaw state to attitude controller input
         */
        AC_AttitudeControl::HeadingCommand get_heading();

    private:

        /**
         * @brief Calculate desired yaw angle
         * @return Target yaw in radians (NED frame: 0=North, clockwise positive)
         * @details Main yaw calculation - switches based on current mode
         */
        float yaw_rad();

        /**
         * @brief Calculate desired yaw rate
         * @return Target yaw rate in rad/s
         * @details Computes rate for modes requiring specific rotation speed
         */
        float rate_rads();

        /**
         * @brief Calculate look-ahead yaw from velocity
         * @return Yaw angle in radians matching direction of horizontal motion
         * @details Computes heading from velocity vector, with smoothing filter
         */
        float look_ahead_yaw_rad();

        /**
         * @brief Calculate yaw to point at ROI
         * @return Yaw angle in radians to look at Region Of Interest
         * @details Computes bearing from current position to ROI location
         */
        float roi_yaw_rad() const;

        // ====================================================================
        // AutoYaw Internal State
        // ====================================================================
        
        Mode _mode = Mode::LOOK_AT_NEXT_WP;  ///< Current yaw control mode
        Mode _last_mode;                      ///< Previous mode (for transition detection)

        Vector3f roi;                         ///< ROI location in NED frame (cm from EKF origin)

        float _fixed_yaw_offset_rad;          ///< Target heading for FIXED mode (radians)
        float _fixed_yaw_slewrate_rads;       ///< Maximum yaw rate for ANGLE_RATE mode (rad/s)

        uint32_t _last_update_ms;             ///< Time of last yaw calculation (milliseconds)

        float _look_ahead_yaw_rad;            ///< Filtered heading from velocity (LOOK_AHEAD mode)

        float _yaw_angle_rad;                 ///< Target angle for ANGLE_RATE mode
        float _yaw_rate_rads;                 ///< Target rate for RATE/ANGLE_RATE modes
        float _pilot_yaw_rate_rads;           ///< Pilot's yaw rate input (PILOT_RATE mode)
    };
    
    /**
     * @brief AutoYaw instance (shared across all autonomous modes)
     * @details Static member provides consistent yaw control across mode switches
     */
    static AutoYaw auto_yaw;

    // ========================================================================
    // Pass-Through Utility Functions
    // ========================================================================
    // These functions provide convenient access to main Copter functionality
    // from within mode code. They forward calls to the main vehicle object.
    // Historical note: These exist to reduce code churn during mode refactoring
    // and are candidates for eventual integration into Mode base class.
    // ========================================================================
    
    /**
     * @brief Get pilot's desired climb rate from throttle stick
     * @return Climb rate in cm/s (positive = up, negative = down)
     * @details Converts throttle stick position to climb rate:
     * - Mid stick → 0 cm/s (hover)
     * - Full up → maximum climb rate (parameter PILOT_SPEED_UP)
     * - Full down → maximum descent rate (parameter PILOT_SPEED_DN)
     * - Applies deadzone and expo curves
     */
    float get_pilot_desired_climb_rate();
    
    /**
     * @brief Get throttle output for non-takeoff situations
     * @return Throttle value 0.0 to 1.0
     * @details Returns appropriate throttle when not in takeoff:
     * - Used by modes that need throttle feedforward
     * - Typically returns hover throttle
     */
    float get_non_takeoff_throttle(void);
    
    /**
     * @brief Update simple mode coordinate transformation
     * @details Updates earth-frame to simple-mode-frame transformation:
     * - SIMPLE mode: Controls relative to arming heading
     * - SUPER_SIMPLE mode: Controls relative to home direction
     * - Call once per loop when simple modes enabled
     */
    void update_simple_mode(void);
    
    /**
     * @brief Request flight mode change
     * @param[in] mode Desired mode number
     * @param[in] reason Reason for mode change (for logging)
     * @return true if mode change succeeded
     * @details Forwards to main Copter::set_mode() function
     */
    bool set_mode(Mode::Number mode, ModeReason reason);
    
    /**
     * @brief Set landed/complete flag
     * @param[in] b true if landed/mission complete
     * @details Updates vehicle landed state, affects:
     * - Motor output (ground idle vs flight)
     * - Integrator reset
     * - Landing detection
     */
    void set_land_complete(bool b);
    
    /**
     * @brief Get GCS (Ground Control Station) interface
     * @return Reference to Copter-specific GCS object
     * @details Access GCS for telemetry, messages, mission management
     */
    GCS_Copter &gcs();
    
    /**
     * @brief Get pilot's desired descent speed parameter
     * @return Maximum descent rate in cm/s
     * @details Returns PILOT_SPEED_DN parameter value
     */
    uint16_t get_pilot_speed_dn(void);
    
    // end pass-through functions
};


// ============================================================================
// MODE SUBCLASSES - Concrete Flight Mode Implementations
// ============================================================================
// Each subclass below implements a specific flight mode by overriding
// virtual methods from the Mode base class. All modes follow the pattern:
// - init(): Called when entering the mode (setup controllers, reset state)
// - run(): Called every main loop iteration (implement mode behavior)
// - exit(): Called when leaving the mode (cleanup, reset)
// - Query methods: requires_GPS(), has_manual_throttle(), etc.
// ============================================================================

#if MODE_ACRO_ENABLED
/**
 * @class ModeAcro
 * @brief ACRO mode - Manual body-frame angular rate control
 * 
 * @details Implements acrobatic flying mode:
 * - Pilot commands body-frame angular rates directly
 * - No attitude stabilization (unlike STABILIZE)
 * - Manual throttle control (no altitude hold)
 * - Maximum agility and control authority
 * 
 * Control behavior:
 * - Roll/Pitch/Yaw sticks → angular rates (deg/s)
 * - Center sticks → zero rotation rate (not level attitude)
 * - Throttle stick → direct motor throttle
 * - Vehicle can achieve any attitude (including inverted)
 * 
 * Trainer modes (for learning):
 * - OFF: Pure rate control, no limits
 * - LEVELING: Returns to level when sticks centered
 * - LIMITED: Maximum lean angle limits applied
 * 
 * Optional features:
 * - AIR_MODE: Maintains full control authority at zero throttle
 * - RATE_LOOP_ONLY: Bypasses angle loop for pure rate response
 * 
 * Typical use:
 * - Aerobatic maneuvers (flips, rolls, inverted flight)
 * - Racing (maximum responsiveness)
 * - Advanced pilots only (no stabilization safety net)
 * 
 * @warning Requires skilled pilot - vehicle will NOT self-level
 * @warning Can achieve attitudes that lead to loss of control
 * @note Most agile mode but least beginner-friendly
 */
class ModeAcro : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ACRO; }

    /**
     * @enum Trainer
     * @brief ACRO trainer modes for learning acrobatic control
     */
    enum class Trainer {
        OFF = 0,      ///< No training - pure rate control
        LEVELING = 1, ///< Auto-levels when sticks centered
        LIMITED = 2,  ///< Limits maximum lean angles
    };

    /**
     * @enum AcroOptions
     * @brief Optional ACRO mode features (bitmask)
     */
    enum class AcroOptions {
        AIR_MODE = 1 << 0,         ///< Maintain control authority at zero throttle
        RATE_LOOP_ONLY = 1 << 1,   ///< Skip angle loop, pure rate control
    };

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool init(bool ignore_checks) override;
    void exit() override;
    
    /**
     * @brief Notification that air mode auxiliary switch changed
     * @details Prevents automatic reset to default air mode state
     */
    void air_mode_aux_changed();
    
    bool allows_save_trim() const override { return true; }
    bool allows_flip() const override { return true; }
    bool crash_check_enabled() const override { return false; }
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    /**
     * @brief Convert pilot stick inputs to desired angular rates
     * 
     * @param[in]  roll_in_norm  Pilot roll input normalized -1.0 to 1.0
     * @param[in]  pitch_in_norm Pilot pitch input normalized -1.0 to 1.0
     * @param[in]  yaw_in_norm   Pilot yaw input normalized -1.0 to 1.0
     * @param[out] roll_out_rads  Desired roll rate in rad/s
     * @param[out] pitch_out_rads Desired pitch rate in rad/s
     * @param[out] yaw_out_rads   Desired yaw rate in rad/s
     * 
     * @details Applies:
     * - Expo curves for stick feel
     * - Maximum rate scaling from parameters
     * - Trainer mode modifications if enabled
     */
    void get_pilot_desired_rates_rads(float roll_in_norm, float pitch_in_norm, float yaw_in_norm, float &roll_out_rads, float &pitch_out_rads, float &yaw_out_rads);

    float throttle_hover() const override;

private:
    bool disable_air_mode_reset; ///< Prevents air mode reset after aux switch change
};
#endif

#if FRAME_CONFIG == HELI_FRAME
/**
 * @class ModeAcro_Heli
 * @brief Helicopter-specific ACRO mode implementation
 * 
 * @details Extends ModeAcro with helicopter-specific features:
 * - Virtual flybar emulation for scale helis
 * - Collective pitch management
 * - Tail rotor integration
 * - External gyro compensation
 * 
 * @note Only compiled for helicopter frame configurations
 */
class ModeAcro_Heli : public ModeAcro {

public:
    // inherit constructor
    using ModeAcro::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    
    /**
     * @brief Virtual flybar emulation for helicopters
     * 
     * @param[in,out] roll_out  Roll rate command (modified for flybar feel)
     * @param[in,out] pitch_out Pitch rate command (modified for flybar feel)
     * @param[in,out] yaw_out   Yaw rate command (modified for flybar feel)
     * @param[in]     pitch_leak Pitch axis leak (cyclic cross-coupling)
     * @param[in]     roll_leak  Roll axis leak (cyclic cross-coupling)
     * 
     * @details Simulates mechanical flybar stabilization:
     * - Adds damping to rate commands
     * - Implements cross-coupling compensation
     * - Provides scale helicopter "feel"
     */
    void virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak);

protected:
private:
};
#endif


/**
 * @class ModeAltHold
 * @brief ALT_HOLD mode - Manual attitude with automatic altitude control
 * 
 * @details Altitude Hold flight mode:
 * - Pilot controls roll and pitch angles (like STABILIZE)
 * - Automatic throttle maintains constant altitude
 * - Manual yaw control
 * - No GPS required (uses barometer/rangefinder)
 * 
 * Control behavior:
 * - Roll/Pitch sticks → lean angles (vehicle attitude)
 * - Throttle stick → climb/descend rate
 * - Mid throttle → hold current altitude
 * - Yaw stick → yaw rate
 * - Sticks centered → level hover at current altitude
 * 
 * Altitude control:
 * - Barometer primary altitude source
 * - Rangefinder used when close to ground (if available)
 * - Automatic compensation for air density changes
 * - Integrator prevents altitude drift
 * 
 * Takeoff support:
 * - User takeoff: Raise throttle to climb to set altitude
 * - After reaching altitude, normal alt-hold control resumes
 * 
 * Typical use:
 * - Learning to fly (easier than full manual)
 * - Hovering in place
 * - Video/photography (stable altitude platform)
 * - Intermediate skill level mode
 * 
 * @note Good beginner mode - automatic altitude, manual positioning
 * @note No horizontal position hold (vehicle will drift)
 */
class ModeAltHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ALT_HOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
    bool allows_auto_trim() const override { return true; }
    bool allows_save_trim() const override { return true; }
#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif
protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};

/**
 * @class ModeAuto
 * @brief AUTO mode - Autonomous waypoint navigation using mission commands
 * 
 * @details Fully autonomous mission execution mode:
 * - Executes pre-programmed mission from EEPROM/SD card
 * - Supports complex mission commands (waypoint, takeoff, land, etc.)
 * - Automatic waypoint navigation with path planning
 * - Integrates with mission planner ground station
 * 
 * Mission execution:
 * - Loads mission commands from AP_Mission
 * - Executes commands sequentially
 * - Each command has start/verify phases
 * - Supports conditional commands and branching
 * - Mission can be modified during flight via GCS
 * 
 * Supported mission commands:
 * - NAV_WAYPOINT: Fly to 3D position
 * - NAV_TAKEOFF: Automatic takeoff sequence
 * - NAV_LAND: Automatic landing (normal or precision)
 * - NAV_RTL: Return to launch point
 * - NAV_LOITER: Hold position for time/turns/unlimited
 * - NAV_CIRCLE: Circle around point
 * - NAV_SPLINE_WAYPOINT: Smooth spline path through waypoints
 * - NAV_GUIDED: Accept real-time guided commands
 * - NAV_SCRIPT_TIME: Lua scripting integration
 * - DO_* commands: Actions (set servo, camera trigger, etc.)
 * - CONDITION_* commands: Conditional execution
 * 
 * AUTO RTL variant:
 * - Activated by DO_LAND_START or return path
 * - Reports as AUTO_RTL mode to GCS
 * - Same AUTO controller, different mission segment
 * 
 * Pilot override:
 * - Yaw control available (unless disabled by parameter)
 * - Can pause/resume mission
 * - Can change mission speed during flight
 * 
 * Safety features:
 * - Geofence integration
 * - Failsafe actions during mission
 * - Terrain following support
 * - Battery failsafe awareness
 * 
 * Typical use:
 * - Survey missions (mapping, agriculture)
 * - Delivery missions
 * - Inspection flights
 * - Search patterns
 * - Fully autonomous operations
 * 
 * @note Requires valid position estimate (GPS or alternative)
 * @note Mission must be uploaded before flight
 * @warning Vehicle operates autonomously - ensure clear flight area
 */
class ModeAuto : public Mode {

public:
    friend class PayloadPlace;  // in case wp_run is accidentally required

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return auto_RTL? Number::AUTO_RTL : Number::AUTO; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override;
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return _mode == SubMode::NAVGUIDED || _mode == SubMode::NAV_SCRIPT_TIME; }
#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    /**
     * @enum SubMode
     * @brief AUTO sub-modes representing different autonomous behaviors
     * 
     * @details Each submode implements a specific mission command type:
     * - SubModes are internal states, not user-selectable
     * - Mode transitions automatically based on mission progression
     * - Current submode determines controller behavior
     */
    enum class SubMode : uint8_t {
        TAKEOFF,              ///< Executing NAV_TAKEOFF command
        WP,                   ///< Flying to waypoint (NAV_WAYPOINT)
        LAND,                 ///< Landing sequence (NAV_LAND)
        RTL,                  ///< Return to launch (NAV_RTL)
        CIRCLE_MOVE_TO_EDGE,  ///< Moving to circle start position
        CIRCLE,               ///< Circling around point (NAV_CIRCLE)
        NAVGUIDED,            ///< Accepting guided commands (NAV_GUIDED)
        LOITER,               ///< Loitering at position (NAV_LOITER)
        LOITER_TO_ALT,        ///< Loiter while climbing/descending to altitude
#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
        NAV_PAYLOAD_PLACE,    ///< Precision payload placement
#endif
        NAV_SCRIPT_TIME,      ///< Lua script control (NAV_SCRIPT_TIME)
        NAV_ATTITUDE_TIME,    ///< Attitude hold for duration (NAV_ATTITUDE_TIME)
    };

    // set submode.  returns true on success, false on failure
    void set_submode(SubMode new_submode);

    // pause continue in auto mode
    bool pause() override;
    bool resume() override;
    bool paused() const;

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    bool wp_start(const Location& dest_loc);
    void land_start();
    void circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);
    void circle_start();
    void nav_guided_start();

    bool is_landing() const override;

    bool is_taking_off() const override;
    bool use_pilot_yaw() const override;

    bool set_speed_xy_cms(float speed_xy_cms) override;
    bool set_speed_up_cms(float speed_up_cms) override;
    bool set_speed_down_cms(float speed_down_cms) override;

    bool requires_terrain_failsafe() const override { return true; }

    void payload_place_start();

    // for GCS_MAVLink to call:
    bool do_guided(const AP_Mission::Mission_Command& cmd);

    // Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
    bool jump_to_landing_sequence_auto_RTL(ModeReason reason);

    // Join mission after DO_RETURN_PATH_START waypoint, if succeeds pretend to be Auto RTL mode
    bool return_path_start_auto_RTL(ModeReason reason);

    // Try join return path else do land start
    bool return_path_or_jump_to_landing_sequence_auto_RTL(ModeReason reason);

    // lua accessors for nav script time support
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4);
    void nav_script_time_done(uint16_t id);

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

    // Mission change detector
    AP_Mission_ChangeDetector mis_change_detector;

    // true if weathervaning is allowed in auto
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

    // Get height above ground, uses landing height if available
    int32_t get_alt_above_ground_cm() const override;

protected:

    const char *name() const override { return auto_RTL? "AUTO RTL" : "AUTO"; }
    const char *name4() const override { return auto_RTL? "ARTL" : "AUTO"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}
    bool get_wp(Location &loc) const override;

private:

    enum class Option : int32_t {
        AllowArming                        = (1 << 0U),
        AllowTakeOffWithoutRaisingThrottle = (1 << 1U),
        IgnorePilotYaw                     = (1 << 2U),
        AllowWeatherVaning                 = (1 << 7U),
    };
    bool option_is_enabled(Option option) const;

    // Enter auto rtl pseudo mode
    bool enter_auto_rtl(ModeReason reason);

    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();

    bool check_for_mission_change();    // detect external changes to mission

    void takeoff_run();
    void wp_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();
    void loiter_to_alt_run();
    void nav_attitude_time_run();

    // return the Location portion of a command.  If the command's lat and lon and/or alt are zero the default_loc's lat,lon and/or alt are returned instead
    Location loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const;

    SubMode _mode = SubMode::TAKEOFF;   // controls which auto controller is run

    bool shift_alt_to_current_alt(Location& target_loc) const;

    // subtract position controller offsets from target location
    // should be used when the location will be used as a target for the position controller
    void subtract_pos_offsets(Location& target_loc) const;

    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
    void get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline);
#if AC_NAV_GUIDED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);
#if HAL_PARACHUTE_ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if AP_WINCH_ENABLED
    void do_winch(const AP_Mission::Mission_Command& cmd);
#endif
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
#if AP_SCRIPTING_ENABLED
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_to_alt() const;
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if AC_NAV_GUIDED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
#if AP_SCRIPTING_ENABLED
    bool verify_nav_script_time();
#endif
    bool verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    struct {
        bool reached_destination_xy : 1;
        bool loiter_start_done : 1;
        bool reached_alt : 1;
        float alt_error_cm;
        int32_t alt;
    } loiter_to_alt;

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start_ms;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    // Land within Auto state
    enum class State {
        FlyToLocation = 0,
        Descending = 1
    };
    State state = State::FlyToLocation;

    bool waiting_to_start;  // true if waiting for vehicle to be armed or EKF origin before starting mission

    // True if we have entered AUTO to perform a DO_LAND_START landing sequence and we should report as AUTO RTL mode
    bool auto_RTL;

#if AP_SCRIPTING_ENABLED
    // nav_script_time command variables
    struct {
        bool done;          // true once lua script indicates it has completed
        uint16_t id;        // unique id to avoid race conditions between commands and lua scripts
        uint32_t start_ms;  // system time nav_script_time command was received (used for timeout)
        uint8_t command;    // command number provided by mission command
        uint8_t timeout_s;  // timeout (in seconds) provided by mission command
        float arg1;         // 1st argument provided by mission command
        float arg2;         // 2nd argument provided by mission command
        int16_t arg3;       // 3rd argument provided by mission command
        int16_t arg4;       // 4th argument provided by mission command
    } nav_scripting;
#endif

    // nav attitude time command variables
    struct {
        int16_t roll_deg;   // target roll angle in degrees.  provided by mission command
        int8_t pitch_deg;   // target pitch angle in degrees.  provided by mission command
        int16_t yaw_deg;    // target yaw angle in degrees.  provided by mission command
        float climb_rate;   // climb rate in m/s. provided by mission command
        uint32_t start_ms;  // system time that nav attitude time command was received (used for timeout)
    } nav_attitude_time;

    // desired speeds
    struct {
        float xy;     // desired speed horizontally in m/s. 0 if unset
        float up;     // desired speed upwards in m/s. 0 if unset
        float down;   // desired speed downwards in m/s. 0 if unset
    } desired_speed_override;

    float circle_last_num_complete;
};

#if AUTOTUNE_ENABLED
/*
  wrapper class for AC_AutoTune
 */

#if FRAME_CONFIG == HELI_FRAME
class AutoTune : public AC_AutoTune_Heli
#else
class AutoTune : public AC_AutoTune_Multi
#endif
{
public:
    bool init() override;
    void run() override;

protected:
    bool position_ok() override;
    float get_pilot_desired_climb_rate_cms(void) const override;
    void get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads) override;
    void init_z_limits() override;
#if HAL_LOGGING_ENABLED
    void log_pids() override;
#endif
};

class ModeAutoTune : public Mode {

    // ParametersG2 sets a pointer within our autotune object:
    friend class ParametersG2;

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::AUTOTUNE; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return false; }

    AutoTune autotune;

protected:

    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }
};
#endif


/**
 * @class ModeBrake
 * @brief BRAKE mode - Rapid deceleration to stop
 * 
 * @details Emergency braking mode:
 * - Brings vehicle to full stop as quickly as possible
 * - Uses maximum deceleration limits
 * - Automatically transitions to LOITER after timeout
 * - Internal mode (typically not user-selectable)
 * 
 * Brake behavior:
 * - Calculates stopping point based on current velocity
 * - Applies maximum braking force
 * - Uses position controller for rapid deceleration
 * - Maintains altitude during braking
 * - Holds position once stopped
 * 
 * Deceleration algorithm:
 * - Predicts stopping position with velocity extrapolation
 * - Commands position controller to stopping point
 * - Controller limits acceleration to maximum configured
 * - Typically stops within 1-2 meters at cruise speeds
 * 
 * Automatic mode transitions:
 * - After timeout (default 1-2 seconds), switches to LOITER
 * - Prevents stuck in brake if stopping unsuccessful
 * - Timeout configurable via timeout_to_loiter_ms()
 * 
 * Typical use cases:
 * - PosHold brake phase (pilot releases sticks)
 * - Throw mode brake (after detecting throw)
 * - Internal transition mode between flight modes
 * - Emergency stop via switch
 * 
 * @note Requires GPS for position estimation
 * @note Cannot be armed in BRAKE mode
 * @note Automatic transition to LOITER prevents manual flying in brake
 * @warning Very aggressive deceleration - may exceed passenger comfort limits
 */
class ModeBrake : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::BRAKE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }

    /**
     * @brief Set timeout before automatic transition to LOITER
     * @param[in] timeout_ms Timeout in milliseconds
     * 
     * @details Configures brake duration before LOITER transition:
     * - Starts counting from brake mode entry
     * - After timeout, automatically switches to LOITER
     * - Default varies by use case (1-2 seconds typical)
     */
    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    const char *name() const override { return "BRAKE"; }
    const char *name4() const override { return "BRAK"; }

private:

    uint32_t _timeout_start;  ///< System time when brake started (milliseconds)
    uint32_t _timeout_ms;     ///< Timeout duration before transition to loiter (milliseconds)

};


/**
 * @class ModeCircle
 * @brief CIRCLE mode - Automated circular flight pattern
 * 
 * @details Autonomous circling mode:
 * - Flies in a circle around a center point
 * - Configurable radius and rate
 * - Automatic altitude maintenance
 * - Used for aerial photography, inspection, observation
 * 
 * Circle behavior:
 * - Center point set at mode entry (current position by default)
 * - Radius configurable via parameter (default 10m)
 * - Rate configurable (deg/s, positive=clockwise from above)
 * - Yaw can track center, forward direction, or pilot control
 * 
 * Pilot control during circle:
 * - Roll stick: Adjust circle rate (speed up/slow down)
 * - Pitch stick: Adjust radius (move closer/farther from center)
 * - Throttle: Climb/descend rate
 * - Yaw: Override yaw direction (if enabled)
 * 
 * Circle controller:
 * - Uses AC_Circle library for smooth circular motion
 * - Implements tangential velocity control
 * - Maintains constant radius via position corrections
 * - Compensates for wind drift
 * 
 * Mission integration:
 * - NAV_CIRCLE mission command enters this mode
 * - Can specify center point, radius, turns
 * - Auto-advances to next waypoint after completion
 * 
 * Typical use cases:
 * - Aerial photography (orbit around subject)
 * - Inspection (circle structure/tower)
 * - Surveillance (loiter with continuous view)
 * - Demonstration flights
 * 
 * @note Requires GPS for position control
 * @note Cannot arm in CIRCLE mode (must enter from another mode)
 */
class ModeCircle : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CIRCLE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;

private:

    bool speed_changing = false;  ///< True when pilot is adjusting circle rate (facilitates stopping at zero rate)
};


/**
 * @class ModeDrift
 * @brief DRIFT mode - Car-like maneuvering for multirotors
 * 
 * @details Semi-autonomous drift mode providing car-like controls:
 * - Roll stick: Controls yaw rate (like steering wheel)
 * - Pitch stick: Controls forward/backward speed
 * - Throttle: Automatic altitude hold with manual override
 * - Creates intuitive "driving" feel for multirotors
 * 
 * Control mapping:
 * - Roll input → Yaw rate (turn left/right)
 * - Pitch input → Forward velocity (accelerate/brake)
 * - Throttle → Climb rate override (otherwise automatic)
 * - Vehicle leans into turns like a car
 * 
 * Drift behavior:
 * - Forward velocity commands vehicle to accelerate forward
 * - Roll input causes coordinated turn (yaw + lean)
 * - Automatic altitude maintenance when throttle centered
 * - Wind compensation for consistent ground speed
 * 
 * Altitude management:
 * - Automatic altitude hold when throttle at mid-stick
 * - Throttle assistance based on vertical velocity
 * - Smooth blending between auto and manual altitude control
 * - Prevents altitude loss in aggressive maneuvers
 * 
 * Turn coordination:
 * - Roll input commands both yaw rate and lean angle
 * - Creates banked turn feel similar to fixed-wing
 * - Rate proportional to roll stick deflection
 * - Maximum yaw rate limited by parameters
 * 
 * Typical use cases:
 * - FPV flying with car-like controls
 * - Aerial videography (smooth panning shots)
 * - New pilot training (intuitive controls)
 * - Racing with automatic altitude hold
 * 
 * @note Requires GPS for velocity control and wind compensation
 * @note Not allowed in RC failsafe (pilot input required)
 * @note Different control paradigm from standard stabilize
 */
class ModeDrift : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::DRIFT; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    /**
     * @brief Calculate throttle assistance based on vertical velocity
     * @param[in] velz Current vertical velocity in cm/s (NED frame, negative=up)
     * @param[in] pilot_throttle_scaled Pilot throttle input scaled -1.0 to 1.0
     * @return Throttle adjustment value
     * 
     * @details Provides automatic throttle assistance:
     * - Compensates for vertical velocity to maintain altitude
     * - Blends with pilot throttle input
     * - Prevents altitude loss during forward acceleration
     */
    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


/**
 * @class ModeFlip
 * @brief FLIP mode - Automated aerobatic flip maneuver
 * 
 * @details Performs automated flip on roll or pitch axis:
 * - Initiated by roll or pitch stick input
 * - Executes 360-degree rotation
 * - Automatic recovery to level flight
 * - Requires minimum altitude for safety
 * 
 * Flip sequence (state machine):
 * 1. Start: Record initial attitude, check altitude/conditions
 * 2. Roll/Pitch: Apply maximum rate command to flip axis
 * 3. Pitch_A/Pitch_B: Continue rotation through inverted
 * 4. Recover: Level out and return control to pilot
 * 5. Abandon: Emergency exit if conditions not met
 * 
 * Flip axis determination:
 * - Roll stick > 50%: Roll right
 * - Roll stick < -50%: Roll left
 * - Pitch stick > 50%: Pitch forward
 * - Pitch stick < -50%: Pitch backward
 * - Direction locked at flip initiation
 * 
 * Safety checks:
 * - Minimum altitude required (typically 3-5m)
 * - Level attitude required before flip
 * - Throttle above minimum
 * - GPS not required (works indoor/GPS-denied)
 * 
 * Flip execution:
 * - Applies maximum configured rate (400+ deg/s typical)
 * - Full throttle during flip
 * - Rotation completes in ~1 second
 * - Automatic attitude recovery at completion
 * - Returns to original flight mode after recovery
 * 
 * Abandon conditions:
 * - Altitude too low during flip
 * - Excessive time without completing rotation
 * - Motor failure detected
 * - Immediately attempts to level vehicle
 * 
 * Typical use cases:
 * - Aerobatic demonstrations
 * - Fun/sport flying
 * - Testing vehicle agility
 * - Pilot entertainment
 * 
 * @note Cannot arm in FLIP mode (must switch while flying)
 * @note Crash check disabled during flip (allows inverted flight)
 * @note Returns to original mode after completion
 * @warning Requires significant altitude margin (5m+ recommended)
 * @warning Vehicle will be inverted during flip (motor mixing must support)
 * @warning Pilot must be experienced and prepared for rapid maneuver
 */
class ModeFlip : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::FLIP; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }
    bool crash_check_enabled() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:

    Vector3f orig_attitude_euler_rad;   ///< Vehicle attitude at flip initiation (radians, used for recovery reference)

    /**
     * @brief Flip state machine stages
     * 
     * State progression: Start → Roll/Pitch → Pitch_A → Pitch_B → Recover
     * Abandon can be entered from any state on safety violation
     */
    enum class FlipState : uint8_t {
        Start,      ///< Initial state - check conditions, record attitude
        Roll,       ///< Rolling 360 degrees on roll axis
        Pitch_A,    ///< First half of pitch flip (0-180 degrees)
        Pitch_B,    ///< Second half of pitch flip (180-360 degrees)
        Recover,    ///< Level out and return to original flight mode
        Abandon     ///< Emergency exit - attempt immediate recovery
    };
    FlipState _state;                   ///< Current flip state
    Mode::Number  orig_control_mode;    ///< Flight mode active when flip initiated (return here after flip)
    uint32_t start_time_ms;             ///< System time when flip began (milliseconds)
    int8_t roll_dir;                    ///< Roll direction: -1=left, +1=right
    int8_t pitch_dir;                   ///< Pitch direction: -1=forward, +1=backward
};


#if MODE_FLOWHOLD_ENABLED
/**
 * @class ModeFlowHold
 * @brief FLOWHOLD mode - Position hold using optical flow without rangefinder
 * 
 * @details GPS-independent position hold using optical flow:
 * - Uses optical flow sensor for velocity estimation
 * - No rangefinder required (uses accelerometer for height)
 * - Holds position relative to ground
 * - Manual pilot override with automatic return to hold
 * 
 * Flow-based position control:
 * - Optical flow measures ground velocity (rad/s)
 * - Converts flow to lean angles to counter drift
 * - PI controller maintains zero flow rate
 * - Works best at low altitudes (0.1m - 3m)
 * 
 * Height estimation:
 * - Integrates accelerometer for altitude
 * - No absolute altitude reference
 * - Altitude drift over time expected
 * - Height used to scale flow to velocity
 * 
 * Pilot control:
 * - Stick input: Override position hold, manually reposition
 * - Release sticks: Brake to stop, then resume hold
 * - Throttle: Manual altitude control
 * - Yaw: Manual heading control
 * 
 * Braking behavior:
 * - After pilot input, automatically brakes to stop
 * - Brake rate configurable (FHLD_BRAKE_RATE parameter)
 * - Once stopped, resumes position hold at new location
 * - Smooth transition between manual and hold
 * 
 * Flow quality monitoring:
 * - Requires minimum flow quality (FHLD_QUAL_MIN parameter)
 * - Low quality: Reverts to manual control
 * - Quality filtering prevents mode oscillation
 * - Surface texture dependent (works poorly over water, uniform surfaces)
 * 
 * Height-dependent scaling:
 * - Flow rate scales with altitude (higher = less sensitive)
 * - Maximum height 3m for reliable performance
 * - Minimum height 0.1m to prevent division by zero
 * - Automatic height estimation update
 * 
 * Advantages over GPS modes:
 * - Works indoors (GPS-denied environments)
 * - Works under bridges, in warehouses
 * - Lower latency than GPS
 * - Independent of satellite availability
 * 
 * Limitations:
 * - Requires textured surface (not uniform/water)
 * - Limited altitude range (typically <3m)
 * - No absolute position reference (drifts over time)
 * - Wind compensation limited
 * - Altitude estimate drifts
 * 
 * Typical use cases:
 * - Indoor flying
 * - Low-altitude inspection
 * - GPS-denied navigation
 * - Toy/small drones
 * 
 * @note Does not require GPS
 * @note Requires optical flow sensor (PX4Flow, Cheerson CX-OF, etc.)
 * @note Works best at low altitudes with good ground texture
 * @warning Altitude estimate will drift over time
 * @warning Flow quality critical - poor surface texture causes poor performance
 */
class ModeFlowHold : public Mode {
public:
    // need a constructor for parameters
    ModeFlowHold(void);
    Number mode_number() const override { return Number::FLOWHOLD; }

    bool init(bool ignore_checks) override;
    void run(void) override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_flip() const override { return true; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    const char *name() const override { return "FLOWHOLD"; }
    const char *name4() const override { return "FHLD"; }

private:

    /**
     * @brief FlowHold mode internal states
     * 
     * State machine for takeoff/flight/landing management
     */
    enum FlowHoldModeState {
        FlowHold_MotorStopped,  ///< Motors stopped, on ground
        FlowHold_Takeoff,       ///< Taking off, climbing to flight altitude
        FlowHold_Flying,        ///< Flying, actively holding position
        FlowHold_Landed         ///< Landed, motors spooling down
    };

    /**
     * @brief Calculate attitude angles from optical flow data
     * @param[out] bf_angle Body-frame lean angles (roll, pitch) in radians
     * 
     * @details Converts flow sensor readings to desired lean angles:
     * - Flow rate (rad/s) scaled by height to get velocity
     * - PI controller generates lean angle to counter velocity
     * - Body-frame output for attitude controller
     */
    void flow_to_angle(Vector2f &bf_angle);

    LowPassFilterConstDtVector2f flow_filter;  ///< Low-pass filter for flow measurements (reduces noise)

    bool flowhold_init(bool ignore_checks);
    void flowhold_run();
    void flowhold_flow_to_angle(Vector2f &angle, bool stick_input);
    void update_height_estimate(void);

    const float height_min_m = 0.1f;   ///< Minimum assumed height (prevents division by zero)
    const float height_max = 3.0f;     ///< Maximum height for reliable flow scaling

    AP_Float flow_max;           ///< Maximum flow rate before limiting (FHLD_FLOW_MAX parameter)
    AC_PI_2D flow_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};  ///< 2D PI controller for X-Y position hold
    AP_Float flow_filter_hz;     ///< Flow measurement filter frequency (FHLD_FILT_HZ parameter)
    AP_Int8  flow_min_quality;   ///< Minimum flow quality to use flow (FHLD_QUAL_MIN parameter, 0-255)
    AP_Int8  brake_rate_dps;     ///< Braking rate in degrees per second (FHLD_BRAKE_RATE parameter)

    float quality_filtered;      ///< Filtered flow quality (0-255, higher=better)

    uint8_t log_counter;         ///< Counter for logging rate control
    bool limited;                ///< True if flow rate is being limited
    Vector2f xy_I;              ///< Integral term for position hold PI controller

    Vector2f delta_velocity_ne_ms;    ///< Accumulated INS velocity change since last flow update (m/s, NE frame)
    Vector2f last_flow_rate_rads;     ///< Last flow rate measurement (rad/s, body frame)
    uint32_t last_flow_ms;            ///< Timestamp of last flow measurement (milliseconds)

    float last_ins_height_m;     ///< Last INS height estimate (meters)
    float height_offset_m;       ///< Height offset for altitude estimation (meters)

    bool braking;                ///< True if currently braking after pilot input
    uint32_t last_stick_input_ms;  ///< Timestamp of last significant stick input (milliseconds)
};
#endif // MODE_FLOWHOLD_ENABLED


/**
 * @class ModeGuided
 * @brief GUIDED mode - Real-time control from ground station or companion computer
 * 
 * @details External control mode for GCS/companion computer:
 * - Accepts real-time position/velocity/attitude commands
 * - Multiple control submodes (position, velocity, acceleration, angle)
 * - MAVLink command interface
 * - Used for autonomous missions, vision-guided flight, scripting
 * 
 * Control submodes:
 * - TakeOff: Climb to specified altitude
 * - WP: Navigate to waypoint with position control
 * - Pos: Hold specified position (earth frame)
 * - PosVelAccel: Combined position/velocity/acceleration targets
 * - VelAccel: Velocity and acceleration targets
 * - Accel: Acceleration targets only
 * - Angle: Direct attitude control with thrust/climb rate
 * 
 * Position control (Pos, WP submodes):
 * - Earth-frame NED coordinates or GPS location
 * - Optional yaw control (angle or rate)
 * - Optional terrain-relative altitude
 * - Waypoint navigation with path following
 * 
 * Velocity control (VelAccel submode):
 * - Body-frame or earth-frame velocities
 * - Acceleration feedforward for smooth response
 * - Used for vision-guided obstacle avoidance
 * - Dynamic replanning support
 * 
 * Attitude control (Angle submode):
 * - Quaternion or angular velocity targets
 * - Climb rate or thrust control
 * - Low-level control for aerobatics, research
 * - Direct motor control abstraction
 * 
 * MAVLink integration:
 * - SET_POSITION_TARGET_LOCAL_NED: Position/velocity/accel commands
 * - SET_POSITION_TARGET_GLOBAL_INT: GPS position commands
 * - SET_ATTITUDE_TARGET: Attitude/thrust commands
 * - Command timeouts prevent runaway on link loss
 * 
 * Command timeout handling:
 * - Position targets: Persistent (no timeout)
 * - Velocity targets: 3 second timeout → hover
 * - Acceleration targets: 1 second timeout → hover
 * - Attitude targets: 1 second timeout → hover
 * - Configurable via GUID_TIMEOUT parameter
 * 
 * Pilot override:
 * - Throttle stick: Override climb rate
 * - Yaw stick: Override yaw control (if not IgnorePilotYaw option set)
 * - Roll/Pitch: Typically ignored (guided controls position)
 * - Configurable via GUID_OPTIONS parameter
 * 
 * Guided options (GUID_OPTIONS bitmask):
 * - AllowArmingFromTX: Permit RC arming in guided mode
 * - IgnorePilotYaw: Disable pilot yaw override
 * - SetAttitudeTarget_ThrustAsThrust: Interpret thrust field as thrust not climb rate
 * - DoNotStabilizePositionXY: Allow position drift (velocity control only)
 * - DoNotStabilizeVelocityXY: No velocity stabilization
 * - WPNavUsedForPosControl: Use waypoint controller for position
 * - AllowWeatherVaning: Permit yaw into wind
 * 
 * Pause/Resume functionality:
 * - pause(): Stop at current position, hold
 * - resume(): Continue to previous target
 * - Used for obstacle avoidance, user intervention
 * 
 * Takeoff sequence:
 * - do_user_takeoff_start(): Initiate guided takeoff
 * - Climbs to specified altitude
 * - Maintains horizontal position during climb
 * - Completes when altitude reached
 * 
 * Position limits (for safety):
 * - limit_set(): Define horizontal/vertical boundaries
 * - limit_check(): Enforce boundaries, prevent excursions
 * - Used in indoor environments, testing
 * - Returns error if limits exceeded
 * 
 * Typical use cases:
 * - Ground station waypoint navigation
 * - Companion computer autonomous control
 * - Vision-guided flight (obstacle avoidance)
 * - Research and development testing
 * - Precision landing guidance
 * - Follow-me modes
 * - Scripting interface
 * 
 * @note Requires GPS for position/velocity control
 * @note Terrain failsafe active (prevents ground collision)
 * @note Can arm with throttle high (intended for auto-start)
 * @note External control safety timeout prevents runaway
 */
/**
 * @class ModeGuided
 * @brief Guided flight mode - fully automatic position control via GCS commands
 * 
 * @details Guided mode provides fully automated position, velocity, and attitude control
 *          based on real-time commands from the ground control station or companion computer.
 *          This is a highly flexible mode supporting multiple control paradigms:
 *          - Position waypoint navigation with yaw control
 *          - Velocity vector control with yaw control  
 *          - Acceleration control
 *          - Combined position/velocity/acceleration control
 *          - Attitude/angle control with climb rate or thrust
 * 
 *          Guided mode requires GPS for position-based control but can operate without
 *          GPS when using pure attitude control (see ModeGuidedNoGPS).
 * 
 *          Control can be provided via MAVLink commands including:
 *          - SET_POSITION_TARGET_LOCAL_NED
 *          - SET_POSITION_TARGET_GLOBAL_INT
 *          - SET_ATTITUDE_TARGET
 * 
 *          The mode supports safety limits (altitude, horizontal distance, timeout)
 *          and can be configured via GUID_OPTIONS parameter for various behaviors.
 * 
 * @note Guided mode is commonly used for:
 *       - Dronekit/MAVProxy scripted missions
 *       - Companion computer control
 *       - Interactive GCS waypoint flying
 *       - Autonomous obstacle avoidance integration
 * 
 * @warning Guided commands have timeout protection - vehicle will brake/hold position
 *          if commands stop arriving within timeout period (default 3 seconds for
 *          velocity/acceleration control)
 * 
 * @see ModeGuidedNoGPS for attitude-only guided control
 * @see ModeAuto for pre-planned mission execution
 */
class ModeGuided : public Mode {

public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::GUIDED; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    /**
     * @brief Set guided angle submode target (attitude + angular velocity + vertical)
     * 
     * @param[in] attitude_quat Target attitude quaternion (body frame)
     *                          IF zero: Pure rate control using ang_vel
     *                          IF non-zero: Attitude control with ang_vel as feedforward
     * @param[in] ang_vel Angular velocity target/feedforward (rad/s, body frame)
     * @param[in] climb_rate_cms_or_thrust Vertical control value:
     *                                      IF use_thrust=false: climb rate (cm/s)
     *                                      IF use_thrust=true: thrust [0-1] unitless
     * @param[in] use_thrust Interpretation of climb_rate_cms_or_thrust parameter
     * 
     * @details Low-level attitude control for advanced applications:
     * - Enters Angle submode
     * - Direct attitude quaternion or rate control
     * - Used for aerobatics, research, advanced scripting
     * - Bypasses position/velocity controllers
     * 
     * @note Requires careful use - incorrect inputs can crash vehicle
     * @note 1 second timeout - command must be sent continuously
     */
    void set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_cms_or_thrust, bool use_thrust);

    /**
     * @brief Set destination position target (earth frame NEU)
     * 
     * @param[in] destination Target position (cm, NEU frame relative to EKF origin)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians, 0=North)
     * @param[in] use_yaw_rate True to control yaw rate instead of heading
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw is relative to current heading
     * @param[in] terrain_alt True if altitude is above terrain (requires rangefinder)
     * @return true if destination accepted, false if rejected
     * 
     * @details Enters WP submode, navigates to position
     */
    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool terrain_alt = false);
    
    /**
     * @brief Set destination position target (GPS location)
     * 
     * @param[in] dest_loc Target location (GPS lat/lon/alt)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians, 0=North)
     * @param[in] use_yaw_rate True to control yaw rate instead of heading
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw is relative to current heading
     * @return true if destination accepted, false if rejected
     */
    bool set_destination(const Location& dest_loc, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);
    
    bool get_wp(Location &loc) const override;
    
    /**
     * @brief Set acceleration target (earth frame)
     * 
     * @param[in] acceleration Target acceleration (cm/s², NEU frame)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians)
     * @param[in] use_yaw_rate True to control yaw rate
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw relative to current
     * @param[in] log_request True to log the command
     * 
     * @details Enters Accel submode, direct acceleration control
     * @note 1 second timeout - requires continuous commands
     */
    void set_accel(const Vector3f& acceleration, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    
    /**
     * @brief Set velocity target (earth frame)
     * 
     * @param[in] velocity Target velocity (cm/s, NEU frame)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians)
     * @param[in] use_yaw_rate True to control yaw rate
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw relative to current
     * @param[in] log_request True to log the command
     * 
     * @details Enters VelAccel submode, velocity control with zero acceleration
     * @note 3 second timeout - requires continuous commands
     */
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    
    /**
     * @brief Set velocity and acceleration targets (earth frame)
     * 
     * @param[in] velocity Target velocity (cm/s, NEU frame)
     * @param[in] acceleration Acceleration feedforward (cm/s², NEU frame)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians)
     * @param[in] use_yaw_rate True to control yaw rate
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw relative to current
     * @param[in] log_request True to log the command
     * 
     * @details Enters VelAccel submode with acceleration feedforward
     * @note 3 second timeout - requires continuous commands
     */
    void set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    
    /**
     * @brief Set position and velocity targets (earth frame)
     * 
     * @param[in] destination Target position (cm, NEU frame)
     * @param[in] velocity Target velocity at destination (cm/s, NEU frame)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians)
     * @param[in] use_yaw_rate True to control yaw rate
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw relative to current
     * @return true if targets accepted
     * 
     * @details Enters PosVelAccel submode, combined position and velocity control
     */
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);
    
    /**
     * @brief Set position, velocity, and acceleration targets (earth frame)
     * 
     * @param[in] destination Target position (cm, NEU frame)
     * @param[in] velocity Target velocity (cm/s, NEU frame)
     * @param[in] acceleration Acceleration feedforward (cm/s², NEU frame)
     * @param[in] use_yaw True to control yaw heading
     * @param[in] yaw_rad Target yaw heading (radians)
     * @param[in] use_yaw_rate True to control yaw rate
     * @param[in] yaw_rate_rads Target yaw rate (rad/s)
     * @param[in] yaw_relative True if yaw relative to current
     * @return true if targets accepted
     * 
     * @details Enters PosVelAccel submode, full trajectory control
     */
    bool set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);

    /**
     * @brief Get current position target
     * @return Target position (cm, NEU frame relative to EKF origin)
     */
    const Vector3p& get_target_pos() const;
    
    /**
     * @brief Get current velocity target
     * @return Target velocity (cm/s, NEU frame)
     */
    const Vector3f& get_target_vel() const;
    
    /**
     * @brief Get current acceleration target
     * @return Target acceleration (cm/s², NEU frame)
     */
    const Vector3f& get_target_accel() const;

    /**
     * @brief Check if attitude target thrust field should be interpreted as thrust
     * @return true if GUID_OPTIONS SetAttitudeTarget_ThrustAsThrust bit set
     * 
     * @details Controls interpretation of SET_ATTITUDE_TARGET thrust field:
     * - false: Thrust field = climb rate (cm/s) - default
     * - true: Thrust field = normalized thrust [0-1]
     */
    bool set_attitude_target_provides_thrust() const;
    
    /**
     * @brief Check if currently stabilizing horizontal position
     * @return true if position controller actively stabilizing X-Y position
     */
    bool stabilizing_pos_xy() const;
    
    /**
     * @brief Check if currently stabilizing horizontal velocity
     * @return true if velocity controller actively stabilizing X-Y velocity
     */
    bool stabilizing_vel_xy() const;
    
    /**
     * @brief Check if using waypoint navigation for position control
     * @return true if GUID_OPTIONS WPNavUsedForPosControl bit set
     */
    bool use_wpnav_for_position_control() const;

    /**
     * @brief Clear position/altitude limits
     * @details Removes all safety boundary restrictions
     */
    void limit_clear();
    
    /**
     * @brief Initialize limits with current position and time
     * @details Records starting position for relative limit checking
     */
    void limit_init_time_and_pos();
    
    /**
     * @brief Set position and altitude safety limits
     * @param[in] timeout_ms Time limit for guided operation (milliseconds, 0=no timeout)
     * @param[in] alt_min_cm Minimum altitude limit (cm above home)
     * @param[in] alt_max_cm Maximum altitude limit (cm above home)
     * @param[in] horiz_max_cm Maximum horizontal distance from start point (cm)
     * 
     * @details Creates safety boundaries for guided operation:
     * - Prevents exceeding altitude limits
     * - Prevents flying beyond horizontal radius
     * - Enforces time limit for operation
     * - Used for indoor flights, testing, safety zones
     */
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    
    /**
     * @brief Check if vehicle within defined limits
     * @return true if within limits, false if limit exceeded
     * 
     * @details Checks against limits set by limit_set():
     * - Returns false if any limit exceeded
     * - Triggers failsafe action if limits violated
     */
    bool limit_check();

    bool is_taking_off() const override;
    
    bool set_speed_xy_cms(float speed_xy_cms) override;
    bool set_speed_up_cms(float speed_up_cms) override;
    bool set_speed_down_cms(float speed_down_cms) override;

    /**
     * @brief Initialize guided takeoff sequence
     * @param[in] takeoff_alt_cm Target altitude (cm above home or terrain)
     * @return true if takeoff started successfully
     * 
     * @details Climbs to specified altitude while maintaining horizontal position
     * @note Altitude is above-home or above-terrain depending on rangefinder availability
     */
    bool do_user_takeoff_start(float takeoff_alt_cm) override;

    /**
     * @enum SubMode
     * @brief Guided submodes indicating which controller is active
     * 
     * @details Guided mode supports multiple control methods via MAVLink:
     * - TakeOff: Climb to altitude while maintaining horizontal position
     * - WP: Navigate to waypoint using position controller
     * - Pos: Hold position target using position controller
     * - PosVelAccel: Combined position, velocity, acceleration control
     * - VelAccel: Velocity control with acceleration feedforward
     * - Accel: Direct acceleration control
     * - Angle: Direct attitude control (quaternion + angular velocity)
     */
    enum class SubMode {
        TakeOff,        ///< Takeoff to specified altitude
        WP,             ///< Waypoint navigation
        Pos,            ///< Position hold
        PosVelAccel,    ///< Position + velocity + acceleration
        VelAccel,       ///< Velocity + acceleration
        Accel,          ///< Acceleration only
        Angle,          ///< Attitude (angle) control
    };

    /**
     * @brief Get current guided submode
     * @return Active submode (TakeOff, WP, Pos, etc.)
     */
    SubMode submode() const { return guided_mode; }

    /**
     * @brief Initialize angle control submode
     * @details Sets up attitude controller for direct quaternion/rate control
     */
    void angle_control_start();
    
    /**
     * @brief Execute angle control submode (called at main loop rate)
     * @details Sends attitude target to attitude controller
     */
    void angle_control_run();

    /**
     * @brief Get timeout for velocity/acceleration/angle control submodes
     * @return Timeout in milliseconds (default 3000ms for vel, 1000ms for accel/angle)
     * 
     * @details Guided commands requiring continuous updates timeout if not refreshed.
     * When timeout occurs, vehicle switches to position hold or loiter.
     * Only applies to VelAccel, Accel, and Angle submodes.
     */
    uint32_t get_timeout_ms() const;

    bool use_pilot_yaw() const override;

    // pause continue in guided mode
    bool pause() override;
    bool resume() override;

    // true if weathervaning is allowed in guided
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error() const override;

private:

    // enum for GUID_OPTIONS parameter
    enum class Option : uint32_t {
        AllowArmingFromTX   = (1U << 0),
        // this bit is still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw      = (1U << 2),
        SetAttitudeTarget_ThrustAsThrust = (1U << 3),
        DoNotStabilizePositionXY = (1U << 4),
        DoNotStabilizeVelocityXY = (1U << 5),
        WPNavUsedForPosControl = (1U << 6),
        AllowWeatherVaning = (1U << 7)
    };

    // returns true if the Guided-mode-option is set (see GUID_OPTIONS)
    bool option_is_enabled(Option option) const;

    // wp controller
    void wp_control_start();
    void wp_control_run();

    void pva_control_start();
    void pos_control_start();
    void accel_control_start();
    void velaccel_control_start();
    void posvelaccel_control_start();
    void takeoff_run();
    void pos_control_run();
    void accel_control_run();
    void velaccel_control_run();
    void pause_control_run();
    void posvelaccel_control_run();
    void set_yaw_state_rad(bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_angle);

    // controls which controller is run (pos or vel):
    static SubMode guided_mode;
    static bool send_notification;     // used to send one time notification to ground station
    static bool takeoff_complete;      // true once takeoff has completed (used to trigger retracting of landing gear)

    // guided mode is paused or not
    static bool _paused;
};

#if AP_SCRIPTING_ENABLED
/**
 * @class ModeGuidedCustom
 * @brief Custom guided mode variant for Lua scripting with user-defined mode number and name
 * 
 * @details ModeGuidedCustom provides scripting support for creating custom flight modes that
 *          inherit guided mode's control framework while presenting custom mode numbers and
 *          names to the GCS and logging system. This enables Lua scripts to implement
 *          specialized autonomous behaviors while leveraging guided mode's position/velocity
 *          control infrastructure.
 * 
 *          Key features:
 *          - Custom mode number (typically 30 reserved for scripted modes)
 *          - Custom mode name (full and abbreviated 4-character versions)
 *          - Scriptable state object for maintaining custom mode state
 *          - Full access to guided mode control methods
 * 
 *          Common use cases:
 *          - Precision agricultural patterns
 *          - Custom inspection routines
 *          - Research algorithm testing
 *          - Specialized commercial operations
 * 
 * @note Lua scripts register custom modes via scripting bindings at initialization
 * @note Mode number 30 is reserved in the Number enum for scripting extensions
 * 
 * @warning Custom modes should implement proper safety checks in Lua scripts including
 *          timeout handling, position validity checks, and failsafe transitions
 * 
 * @see AP_Vehicle::custom_mode_state for scriptable state structure
 * @see ModeGuided for underlying control implementation
 */
class ModeGuidedCustom : public ModeGuided {
public:
    /**
     * @brief Constructor registers custom mode number and names
     * 
     * @param[in] _number      Custom mode number (typically 30 for scripting)
     * @param[in] _full_name   Full mode name string (for display and logging)
     * @param[in] _short_name  Abbreviated 4-character mode name (for compact display)
     */
    ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name);

    /**
     * @brief Initialize custom guided mode
     * 
     * @param[in] ignore_checks If true, bypass arming and pre-flight checks
     * 
     * @return true if initialization successful, false otherwise
     * 
     * @note Calls parent ModeGuided::init() then allows script-specific initialization
     */
    bool init(bool ignore_checks) override;

    /** @brief Return custom mode number */
    Number mode_number() const override { return number; }

    /** @brief Return custom full mode name */
    const char *name() const override { return full_name; }
    
    /** @brief Return custom abbreviated (4-char) mode name */
    const char *name4() const override { return short_name; }

    /**
     * @brief State object which can be edited by scripting
     * 
     * @details Provides scriptable state storage for maintaining custom mode logic state
     *          across update cycles. Lua scripts can read/write this state to implement
     *          complex multi-step behaviors.
     */
    AP_Vehicle::custom_mode_state state;

private:
    const Number number;        ///< Custom mode number assigned at construction
    const char* full_name;      ///< Full mode name string
    const char* short_name;     ///< Abbreviated 4-character mode name
};
#endif

/**
 * @class ModeGuidedNoGPS
 * @brief Guided mode variant for attitude and altitude control without GPS
 * 
 * @details ModeGuidedNoGPS provides guided control functionality when GPS is unavailable
 *          or unreliable. This mode accepts only attitude (roll, pitch, yaw) and altitude
 *          commands via MAVLink SET_ATTITUDE_TARGET messages, without position or velocity
 *          control.
 * 
 *          Supported control inputs:
 *          - Attitude quaternion or Euler angles
 *          - Body-frame angular rates
 *          - Climb rate or thrust
 *          - Yaw rate
 * 
 *          Typical use cases:
 *          - Indoor flight without GPS
 *          - GPS-denied environments
 *          - Optical flow-based control
 *          - External vision system control (e.g., motion capture)
 *          - Initial testing without GPS fix
 * 
 *          Position hold is NOT available - vehicle will drift with wind and
 *          momentum unless external stabilization is provided via attitude commands.
 * 
 * @note Uses barometer for altitude control
 * @note Can integrate with optical flow for velocity estimation if available
 * 
 * @warning Without GPS, geofencing and terrain following are disabled
 * @warning Vehicle will drift horizontally - continuous attitude commands required
 * @warning Emergency RTL and other GPS-dependent failsafes unavailable
 * 
 * @see ModeGuided for GPS-enabled guided control
 */
class ModeGuidedNoGPS : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    
    /** @brief Return GUIDED_NOGPS mode number (20) */
    Number mode_number() const override { return Number::GUIDED_NOGPS; }

    /**
     * @brief Initialize guided no-GPS mode
     * 
     * @param[in] ignore_checks If true, bypass arming and pre-flight checks
     * 
     * @return true if initialization successful, false otherwise
     * 
     * @note Does not require GPS fix for initialization
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main update function - runs attitude control without position hold
     * 
     * @details Processes attitude commands and maintains altitude control using barometer.
     *          Does not attempt position stabilization.
     */
    void run() override;

    /** @brief Guided NoGPS does not require GPS */
    bool requires_GPS() const override { return false; }
    
    /** @brief Throttle is automatic (altitude/thrust controlled) */
    bool has_manual_throttle() const override { return false; }
    
    /** @brief This is an autopilot mode */
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "GUIDED_NOGPS"; }
    const char *name4() const override { return "GNGP"; }

private:

};


/**
 * @class ModeLand
 * @brief Automatic landing mode with optional horizontal position control
 * 
 * @details ModeLand implements automatic landing sequences for multicopters.
 *          The mode can operate in two distinct control regimes:
 *          
 *          **GPS-Based Landing** (default):
 *          - Maintains horizontal position using GPS and position controller
 *          - Descends at controlled rate with altitude hold
 *          - Suitable for normal landing operations
 *          
 *          **Non-GPS Landing** (fallback):
 *          - No horizontal position control (pilot can control with sticks)
 *          - Descends at controlled rate based on pilot throttle input
 *          - Automatically selected if GPS quality insufficient
 *          - Can be manually triggered via do_not_use_GPS()
 *          
 *          Landing sequence handles:
 *          - Automatic descent rate adjustment based on altitude
 *          - Ground detection and motor shutdown
 *          - Precision landing integration (if enabled)
 *          - Landing gear retraction/extension
 *          - Rangefinder-based terrain following
 *          
 *          The mode supports pausing descent via set_land_pause() for obstacle
 *          avoidance or precision landing retry maneuvers.
 * 
 * @note This is an autopilot mode - pilot stick inputs are limited
 * @note Arming is not permitted in LAND mode
 * @note Mode automatically disarms after successful landing
 * 
 * @see land_run_horizontal_control() in Mode base class
 * @see land_run_vertical_control() in Mode base class
 * @see AC_PrecLand for precision landing integration
 */
class ModeLand : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Return mode number identifier
     * @return Number::LAND mode identifier
     */
    Number mode_number() const override { return Number::LAND; }

    /**
     * @brief Initialize LAND mode
     * 
     * @details Initializes landing sequence by:
     *          - Setting up position controller for GPS-based landing
     *          - Initializing descent rate and altitude targets
     *          - Enabling precision landing if configured and available
     *          - Recording landing start time for timeouts
     *          - Configuring landing gear for deployment
     * 
     * @param[in] ignore_checks If true, skip pre-flight checks (used in failsafe)
     * @return true if initialization successful, false otherwise
     * 
     * @note Mode will automatically select GPS or non-GPS landing based on
     *       GPS quality and LAND_REPOSITION parameter
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Execute LAND mode control loop (called at main loop rate)
     * 
     * @details Runs appropriate landing controller:
     *          - gps_run() if GPS available and position control enabled
     *          - nogps_run() if GPS unavailable or position control disabled
     *          
     *          Integrates precision landing if enabled and target visible.
     *          Handles automatic disarm after touchdown detection.
     */
    void run() override;

    /**
     * @brief Check if GPS is required for this mode
     * @return false - LAND mode can operate without GPS (falls back to non-GPS landing)
     */
    bool requires_GPS() const override { return false; }
    
    /**
     * @brief Check if pilot has direct throttle control
     * @return false - throttle is controlled by landing algorithm
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if arming is allowed in this mode
     * @param[in] method Arming method being attempted
     * @return false - arming not permitted in LAND mode
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    /**
     * @brief Check if this is an autopilot mode
     * @return true - LAND is fully autonomous
     */
    bool is_autopilot() const override { return true; }

    /**
     * @brief Check if vehicle is currently landing
     * @return true - LAND mode is always performing landing
     */
    bool is_landing() const override { return true; };

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    /**
     * @brief Return control mode type for advanced failsafe system
     * @return AFS_AUTO - treated as automatic mode by failsafe
     */
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    /**
     * @brief Force mode to use non-GPS landing controller
     * 
     * @details Disables horizontal position control and switches to
     *          non-GPS landing mode where pilot can manually control
     *          horizontal position with sticks while automatic descent
     *          continues. Used when GPS quality degrades during landing
     *          or when precision landing fails.
     * 
     * @note This is a one-way transition - cannot re-enable GPS control
     *       without exiting and re-entering LAND mode
     */
    void do_not_use_GPS();

    /**
     * @brief Check if LAND mode is controlling horizontal position
     * 
     * @details Returns true if mode is using GPS-based position control,
     *          false if in non-GPS mode where pilot has horizontal control.
     * 
     * @return true if controlling X/Y position, false if pilot has control
     * 
     * @note Useful for determining landing mode behavior and whether
     *       precision landing can be attempted
     */
    bool controlling_position() const { return control_position; }

    /**
     * @brief Pause or resume descent during landing
     * 
     * @details When paused, vehicle holds current altitude while continuing
     *          horizontal position control (if GPS-based landing). Used by:
     *          - Precision landing for repositioning maneuvers
     *          - Obstacle avoidance during descent
     *          - External control systems
     * 
     * @param[in] new_value true to pause descent, false to resume
     * 
     * @note Horizontal position control continues even when paused
     * @note Descent automatically resumes after timeout if pause too long
     */
    void set_land_pause(bool new_value) { land_pause = new_value; }

protected:

    /**
     * @brief Get mode name string
     * @return "LAND"
     */
    const char *name() const override { return "LAND"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "LAND"
     */
    const char *name4() const override { return "LAND"; }

private:

    /**
     * @brief Run GPS-based landing with horizontal position control
     * 
     * @details Implements landing with position hold:
     *          - Uses position controller to maintain horizontal location
     *          - Controls descent rate based on altitude above ground
     *          - Integrates precision landing if target visible
     *          - Slows descent rate near ground for smooth touchdown
     *          - Calls land_run_horiz_and_vert_control() for actual control
     */
    void gps_run();
    
    /**
     * @brief Run non-GPS landing without position control
     * 
     * @details Implements descent-only landing:
     *          - No horizontal position control (pilot controls with sticks)
     *          - Automatic descent rate based on altitude
     *          - Uses attitude controller for pilot lean angle inputs
     *          - Suitable for GPS-denied environments or GPS failures
     */
    void nogps_run();

    bool control_position; ///< true if using GPS to control horizontal position, false if pilot has control

    uint32_t land_start_time; ///< system time (ms) when landing started, used for timeouts and logging
    bool land_pause; ///< true if descent is paused (holds altitude), false for normal descent
};


/**
 * @class ModeLoiter
 * @brief Loiter flight mode - GPS position hold with automatic altitude control
 * 
 * @details Loiter mode provides GPS-based position hold with automatic throttle:
 *          - Maintains horizontal position using position controller
 *          - Automatic altitude hold at target altitude
 *          - Pilot can manually adjust position with stick inputs
 *          - Returns to center position when sticks released
 *          - Supports precision loiter (landing on visual target)
 *          - Supports user takeoff to specified altitude
 *          - Commonly used for hovering, observation, and autotune
 *          
 *          Control behavior:
 *          - Roll/pitch sticks: Move vehicle horizontally, returns to loiter position when released
 *          - Throttle stick: Climb/descend to new altitude, holds altitude when centered
 *          - Yaw stick: Rotate vehicle, rate control
 *          
 *          Requirements:
 *          - GPS lock (3D fix required)
 *          - Position controller initialized
 *          - EKF healthy with position estimate
 * 
 * @note This is one of the most commonly used flight modes for multirotors
 * @warning Vehicle will drift if GPS quality degrades - monitor GPS status
 */
class ModeLoiter : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::LOITER (5)
     */
    Number mode_number() const override { return Number::LOITER; }

    /**
     * @brief Initialize loiter mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful, false if GPS or position controller not ready
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for loiter mode, called at scheduler rate (typically 400Hz)
     * 
     * @details Executes loiter control logic:
     *          - Processes pilot inputs for position adjustments
     *          - Runs position controller to maintain horizontal position
     *          - Runs altitude controller for vertical position
     *          - Checks for precision landing target if enabled
     *          - Updates motor outputs
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return true - loiter absolutely requires GPS for position control
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - loiter uses automatic throttle (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return true - vehicle can be armed in loiter mode
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - loiter is pilot-controlled with automatic assistance
     */
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Check if mode supports user takeoff
     * @param[in] must_navigate True if takeoff requires waypoint navigation capability
     * @return true - loiter supports takeoff to altitude (no navigation required)
     */
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    
    /**
     * @brief Check if autotune can run in this mode
     * @return true - loiter is ideal for autotune (stable hovering)
     */
    bool allows_autotune() const override { return true; }
    
    /**
     * @brief Check if auto trim can run in this mode
     * @return true - loiter allows automatic trim adjustment
     */
    bool allows_auto_trim() const override { return true; }

#if FRAME_CONFIG == HELI_FRAME
    /**
     * @brief Check if mode allows inverted flight (helicopter only)
     * @return true - loiter supports inverted flight for helicopters
     */
    bool allows_inverted() const override { return true; };
#endif

#if AC_PRECLAND_ENABLED
    /**
     * @brief Enable/disable precision loiter on visual target
     * @param[in] value true to enable precision loiter, false to disable
     * 
     * @details When enabled, vehicle will attempt to loiter precisely over
     *          a detected visual target (e.g., IR beacon) for precision landing
     */
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    /**
     * @brief Get mode name string
     * @return "LOITER"
     */
    const char *name() const override { return "LOITER"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "LOIT"
     */
    const char *name4() const override { return "LOIT"; }

    /**
     * @brief Get distance to loiter target in meters
     * @return Distance from current position to loiter position in meters
     * 
     * @details Used for GCS reporting and mode monitoring. Returns distance
     *          vehicle has moved from the original loiter center position.
     */
    float wp_distance_m() const override;
    
    /**
     * @brief Get bearing to loiter target in degrees
     * @return Bearing from current position to loiter center (0-360 degrees)
     */
    float wp_bearing_deg() const override;
    
    /**
     * @brief Get crosstrack error from desired loiter position
     * @return Crosstrack error in meters
     */
    float crosstrack_error() const override { return pos_control->crosstrack_error();}

#if AC_PRECLAND_ENABLED
    /**
     * @brief Check if precision loiter should be active based on target detection
     * @return true if precision target visible and precision loiter should engage
     * 
     * @details Evaluates if precision landing target is detected and healthy,
     *          and if vehicle is in correct state to use precision loiter
     */
    bool do_precision_loiter();
    
    /**
     * @brief Execute precision loiter horizontal position control
     * 
     * @details Overrides normal position controller to track detected visual target.
     *          Uses precision landing backend to get target position and commands
     *          position controller to maintain position over target.
     */
    void precision_loiter_xy();
#endif

private:

#if AC_PRECLAND_ENABLED
    bool _precision_loiter_enabled;  ///< true if precision loiter feature is enabled (user setting)
    bool _precision_loiter_active;   ///< true if precision loiter is currently active (target visible and tracking)
#endif

};


/**
 * @class ModePosHold
 * @brief Position Hold mode - GPS position hold with brake and wind compensation
 * 
 * @details PosHold mode provides advanced position hold with intelligent control transitions:
 *          - Each axis (roll/pitch) independently transitions between pilot override,
 *            braking, and loiter modes
 *          - Smooth transitions prevent abrupt control changes
 *          - Active wind compensation for drift prevention
 *          - Brake mode engages when pilot releases sticks to stop quickly
 *          - Returns to loiter (position hold) after braking completes
 *          
 *          Control behavior:
 *          - Pilot stick input: Direct attitude control (PILOT_OVERRIDE)
 *          - Release sticks: Vehicle brakes to a stop (BRAKE)
 *          - After stopping: Holds position with wind compensation (LOITER)
 *          - Resume stick input: Smooth transition from controller to pilot
 *          
 *          Axis control states (independent for roll and pitch):
 *          1. PILOT_OVERRIDE: Pilot directly controls axis lean angle
 *          2. BRAKE: Actively decelerating to zero velocity
 *          3. BRAKE_READY_TO_LOITER: Brake complete, waiting for other axis
 *          4. BRAKE_TO_LOITER: Transitioning from brake to position hold
 *          5. LOITER: Active position hold with wind compensation
 *          6. CONTROLLER_TO_PILOT_OVERRIDE: Transitioning from loiter to pilot control
 *          
 *          Wind compensation:
 *          - Learns wind force during position hold
 *          - Applies feedforward compensation to reduce drift
 *          - Low-pass filtered for smooth wind estimate
 *          
 *          Requirements:
 *          - GPS lock (3D fix required)
 *          - Position controller initialized
 *          - EKF healthy with velocity estimate
 * 
 * @note More responsive than Loiter mode due to brake/transition logic
 * @note Wind compensation reduces position drift in windy conditions
 * @warning Complex state machine - ensure clean state transitions during testing
 */
class ModePosHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::POSHOLD (16)
     */
    Number mode_number() const override { return Number::POSHOLD; }

    /**
     * @brief Initialize PosHold mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful, false if GPS or position controller not ready
     * 
     * @details Initialization:
     *          - Starts both axes in PILOT_OVERRIDE mode
     *          - Initializes position controller at current location
     *          - Resets wind compensation estimates
     *          - Clears all state transition timers
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for PosHold mode, called at scheduler rate (typically 400Hz)
     * 
     * @details Executes PosHold control logic:
     *          - Evaluates pilot stick inputs for each axis
     *          - Updates state machine for roll and pitch independently
     *          - Executes control logic based on current axis states
     *          - Updates wind compensation estimates during loiter
     *          - Mixes controls during state transitions
     *          - Runs altitude controller for vertical position
     *          - Updates motor outputs
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return true - PosHold absolutely requires GPS for position control
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - PosHold uses automatic throttle (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return true - vehicle can be armed in PosHold mode
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - PosHold is pilot-controlled with automatic assistance
     */
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Check if mode supports user takeoff
     * @param[in] must_navigate True if takeoff requires waypoint navigation capability
     * @return true - PosHold supports takeoff to altitude (no navigation required)
     */
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    
    /**
     * @brief Check if autotune can run in this mode
     * @return true - PosHold is suitable for autotune (stable hovering with position hold)
     */
    bool allows_autotune() const override { return true; }
    
    /**
     * @brief Check if auto trim can run in this mode
     * @return true - PosHold allows automatic trim adjustment
     */
    bool allows_auto_trim() const override { return true;}

protected:

    /**
     * @brief Get mode name string
     * @return "POSHOLD"
     */
    const char *name() const override { return "POSHOLD"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "PHLD"
     */
    const char *name4() const override { return "PHLD"; }

private:

    /**
     * @brief Update pilot lean angle command with rate limiting for smooth transitions
     * @param[in,out] lean_angle_filtered Current filtered lean angle (deg x100), updated with rate limiting
     * @param[in] lean_angle_raw Raw pilot stick input converted to lean angle (deg x100)
     * 
     * @details Applies rate limiting to prevent abrupt changes when pilot moves stick.
     *          Slowly decays to zero when stick is centered for smooth return to hover.
     */
    void update_pilot_lean_angle_cd(float &lean_angle_filtered, float &lean_angle_raw);
    
    /**
     * @brief Mix two control outputs based on transition ratio
     * @param[in] mix_ratio Blend factor (0.0 = first_control, 1.0 = second_control)
     * @param[in] first_control First control value (deg x100)
     * @param[in] second_control Second control value (deg x100)
     * @return Blended control output (deg x100)
     * 
     * @details Used during state transitions (brake-to-loiter, controller-to-pilot)
     *          to smoothly blend control authority between modes.
     */
    float mix_controls(float mix_ratio, float first_control, float second_control);
    
    /**
     * @brief Calculate brake angle required to decelerate from current velocity
     * @param[out] brake_angle_cd Required brake lean angle in centidegrees
     * @param[in] velocity_cms Current velocity in cm/s on this axis
     * 
     * @details Computes proportional brake angle to efficiently stop vehicle.
     *          Larger velocities require larger brake angles. Updates peak angle
     *          tracking to detect when vehicle has slowed (angle decreases).
     */
    void update_brake_angle_from_velocity(float &brake_angle_cd, float velocity_cms);
    
    /**
     * @brief Initialize wind compensation estimator
     * 
     * @details Resets wind compensation vectors and starts timing for estimate.
     *          Called when entering loiter mode on an axis.
     */
    void init_wind_comp_estimate();
    
    /**
     * @brief Update wind compensation estimate based on position controller output
     * 
     * @details During stable loiter, observes lean angles required to hold position.
     *          These lean angles represent wind force. Low-pass filters the estimate
     *          to reject transients and provide smooth wind compensation.
     */
    void update_wind_comp_estimate();
    
    /**
     * @brief Get wind compensation lean angles in body frame
     * @param[out] roll_angle_cd Wind compensation roll angle in centidegrees
     * @param[out] pitch_angle_cd Wind compensation pitch angle in centidegrees
     * 
     * @details Transforms earth-frame wind estimate to body-frame lean angles.
     *          Applied as feedforward to position controller to reduce drift.
     */
    void get_wind_comp_lean_angles(float &roll_angle_cd, float &pitch_angle_cd);
    
    /**
     * @brief Transition roll axis from controller (loiter) to pilot override
     * 
     * @details Initiates smooth transition when pilot moves roll stick.
     *          Captures current controller output and begins time-based blend.
     */
    void roll_controller_to_pilot_override();
    
    /**
     * @brief Transition pitch axis from controller (loiter) to pilot override
     * 
     * @details Initiates smooth transition when pilot moves pitch stick.
     *          Captures current controller output and begins time-based blend.
     */
    void pitch_controller_to_pilot_override();

    /**
     * @enum RPMode
     * @brief Roll/Pitch axis control mode - each axis independently managed
     * 
     * @details Each axis (roll and pitch) has its own state machine that transitions
     *          between pilot control, braking, and position hold. This allows
     *          independent control - for example, roll axis can be braking while
     *          pitch axis is in pilot override.
     */
    enum class RPMode {
        PILOT_OVERRIDE=0,            ///< Pilot directly controlling this axis lean angle
        BRAKE,                       ///< Axis actively braking to zero velocity
        BRAKE_READY_TO_LOITER,       ///< Brake complete, waiting for other axis to finish braking
        BRAKE_TO_LOITER,             ///< Both axes transitioning from brake to loiter (blended control)
        LOITER,                      ///< Axis holding position with wind compensation
        CONTROLLER_TO_PILOT_OVERRIDE ///< Transitioning from loiter to pilot control (blended)
    };

    RPMode roll_mode;   ///< Current control mode for roll axis
    RPMode pitch_mode;  ///< Current control mode for pitch axis

    // pilot input related variables
    float pilot_roll_cd;   ///< Filtered roll lean angle commanded by pilot (deg x100), slowly decays to zero
    float pilot_pitch_cd;  ///< Filtered pitch lean angle commanded by pilot (deg x100), slowly decays to zero


    // braking related variables
    /**
     * @brief Brake state variables for deceleration control
     * 
     * @details Tracks brake timing and angles for both roll and pitch axes.
     *          Brake angles are computed from velocity and adjusted based on
     *          vehicle response. Peak angle tracking detects when vehicle slows.
     */
    struct {
        bool  time_updated_roll;            ///< true if braking timeout on roll axis has been re-estimated
        bool  time_updated_pitch;           ///< true if braking timeout on pitch axis has been re-estimated
        float gain;                         ///< Braking gain to convert velocity (cm/s) to lean angle (deg x100)
        float roll_cd;                      ///< Current braking roll angle in centidegrees
        float pitch_cd;                     ///< Current braking pitch angle in centidegrees
        uint32_t start_time_roll_ms;        ///< System time (ms) when roll axis braking started
        uint32_t start_time_pitch_ms;       ///< System time (ms) when pitch axis braking started
        float angle_max_roll_cd;            ///< Peak roll brake angle (deg x100), used to detect deceleration completion
        float angle_max_pitch_cd;           ///< Peak pitch brake angle (deg x100), used to detect deceleration completion
        uint32_t loiter_transition_start_time_ms;   ///< System time (ms) when brake-to-loiter transition started
    } brake;


    // loiter related variables
    uint32_t controller_to_pilot_start_time_roll_ms;   ///< System time (ms) when roll axis began transitioning to pilot control
    uint32_t controller_to_pilot_start_time_pitch_ms;  ///< System time (ms) when pitch axis began transitioning to pilot control

    float controller_final_roll_cd;   ///< Roll controller output (deg x100) captured at start of transition to pilot
    float controller_final_pitch_cd;  ///< Pitch controller output (deg x100) captured at start of transition to pilot

    // wind compensation related variables
    Vector2f wind_comp_ef;              ///< Wind compensation acceleration vector in earth frame (m/s²), low-pass filtered
    float wind_comp_roll_cd;            ///< Body-frame roll angle (deg x100) to compensate for wind
    float wind_comp_pitch_cd;           ///< Body-frame pitch angle (deg x100) to compensate for wind
    uint32_t wind_comp_start_time_ms;   ///< System time (ms) when wind compensation estimation started

    // final output
    float roll_cd;   ///< Final roll angle command (deg x100) sent to attitude controller
    float pitch_cd;  ///< Final pitch angle command (deg x100) sent to attitude controller
};


/**
 * @class ModeRTL
 * @brief Return to Launch mode - automated return home and landing sequence
 * 
 * @details RTL mode executes a multi-phase autonomous return to the launch location:
 *          
 *          RTL Phases:
 *          1. STARTING: Initialize RTL, compute return path
 *          2. INITIAL_CLIMB: Climb to RTL altitude if below target
 *          3. RETURN_HOME: Navigate horizontally to home or rally point
 *          4. LOITER_AT_HOME: Optional loiter at home before descending (if RTL_LOITER_TIME > 0)
 *          5. FINAL_DESCENT: Descend to landing altitude
 *          6. LAND: Execute landing sequence
 *          
 *          Path Planning:
 *          - Computes optimal return path considering current altitude
 *          - Can use rally points if closer than home
 *          - Supports terrain following if RTL_ALT_TYPE=TERRAIN
 *          - Respects cylinder/polygon fences during return
 *          
 *          Altitude Control:
 *          - RTL_ALT: Target altitude for return flight (cm above home or terrain)
 *          - RTL_ALT_FINAL: Final descent altitude before landing (cm)
 *          - RTL_CLIMB_MIN: Minimum climb before starting return (cm)
 *          - If already above RTL_ALT, maintains current altitude during return
 *          
 *          Home Selection:
 *          - Primary: Launch location (where vehicle was armed)
 *          - Alternative: Rally point if closer and RTL_ALT_TYPE allows
 *          - Home can be updated via MAVLink SET_HOME_POSITION
 *          
 *          Pilot Control:
 *          - Yaw: Pilot can control yaw during return (unless RTL_OPTIONS bit 2 set)
 *          - Roll/Pitch: No pilot control (autopilot managed)
 *          - Throttle: No pilot control (automatic altitude)
 *          
 *          Safety Features:
 *          - Automatic failsafe action (commonly triggered by RC loss, battery, GCS loss)
 *          - Continues if GPS quality degrades after RTL start
 *          - Can be combined with SmartRTL for path-retracing return
 *          - Terrain failsafe required - aborts RTL if terrain data unavailable when needed
 *          
 *          Requirements:
 *          - GPS lock (3D fix required)
 *          - Home position set (set at arming or via MAVLink)
 *          - EKF healthy with position estimate
 *          - Sufficient battery for return journey
 * 
 * @note RTL is the most common failsafe action - test thoroughly for your vehicle
 * @warning Vehicle will land at home automatically - ensure landing area is clear
 * @warning Ensure RTL_ALT provides obstacle clearance for your flight area
 */
class ModeRTL : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::RTL (6)
     */
    Number mode_number() const override { return Number::RTL; }

    /**
     * @brief Initialize RTL mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful, false if GPS or home position not available
     * 
     * @details Initialization:
     *          - Builds RTL path (climb, return, descent targets)
     *          - Determines if rally point should be used
     *          - Configures terrain following if enabled
     *          - Starts in STARTING or INITIAL_CLIMB state
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for RTL mode with automatic disarm on landing
     * 
     * @details Convenience wrapper that calls run(true) to automatically
     *          disarm motors after landing completes.
     */
    void run() override {
        return run(true);
    }
    
    /**
     * @brief Main loop for RTL mode, called at scheduler rate (typically 400Hz)
     * @param[in] disarm_on_land If true, automatically disarm motors after landing
     * 
     * @details Executes RTL state machine:
     *          - Progresses through RTL phases (climb, return, loiter, descend, land)
     *          - Updates waypoint navigation for return phase
     *          - Monitors position/altitude goals for phase completion
     *          - Handles pilot yaw input if allowed
     *          - Transitions to next phase when current phase complete
     */
    void run(bool disarm_on_land);

    /**
     * @brief Check if mode requires GPS
     * @return true - RTL absolutely requires GPS to navigate home
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - RTL uses automatic throttle (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return false - cannot arm in RTL mode (it's a failsafe/return mode)
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return true - RTL is fully autonomous
     */
    bool is_autopilot() const override { return true; }

    /**
     * @brief Check if mode requires terrain failsafe
     * @return true - RTL needs terrain failsafe if using terrain-based altitude
     * 
     * @details If RTL_ALT_TYPE=TERRAIN, vehicle needs terrain data for safe return.
     *          Terrain failsafe triggers if data becomes unavailable during RTL.
     */
    bool requires_terrain_failsafe() const override { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    /**
     * @brief Get advanced failsafe mode type
     * @return AFS_AUTO - RTL is treated as autonomous mode for advanced failsafe
     */
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    /**
     * @brief Get current target waypoint for GCS reporting
     * @param[out] loc Location of current RTL target waypoint
     * @return true if valid waypoint available, false otherwise
     * 
     * @details Returns appropriate target based on current RTL phase:
     *          - RETURN_HOME: Home or rally point location
     *          - LOITER_AT_HOME: Home location
     *          - Other phases: Current target from rtl_path structure
     */
    bool get_wp(Location &loc) const override;

    /**
     * @brief Check if pilot yaw control is allowed
     * @return true if pilot can control yaw, false if yaw is autopilot-controlled
     * 
     * @details Pilot yaw control can be disabled via RTL_OPTIONS bit 2.
     *          Default is to allow pilot yaw during RTL for situational awareness.
     */
    bool use_pilot_yaw() const override;

    /**
     * @brief Set horizontal speed override for RTL
     * @param[in] speed_xy_cms Desired horizontal speed in cm/s
     * @return true if speed set successfully
     * 
     * @details Overrides default RTL horizontal speed (WPNAV_SPEED).
     *          Used by MAVLink DO_CHANGE_SPEED command.
     */
    bool set_speed_xy_cms(float speed_xy_cms) override;
    
    /**
     * @brief Set climb speed override for RTL
     * @param[in] speed_up_cms Desired climb speed in cm/s
     * @return true if speed set successfully
     * 
     * @details Overrides default RTL climb speed (WPNAV_SPEED_UP).
     */
    bool set_speed_up_cms(float speed_up_cms) override;
    
    /**
     * @brief Set descent speed override for RTL
     * @param[in] speed_down_cms Desired descent speed in cm/s
     * @return true if speed set successfully
     * 
     * @details Overrides default RTL descent speed (WPNAV_SPEED_DN).
     */
    bool set_speed_down_cms(float speed_down_cms) override;

    /**
     * @enum SubMode
     * @brief RTL phase states - sequential progression through return sequence
     * 
     * @details RTL progresses through these phases in order. Each phase has
     *          specific altitude and position goals. Phase transitions when
     *          goals are achieved and state_complete flag is set.
     */
    enum class SubMode : uint8_t {
        STARTING,         ///< Initializing RTL, building path, starting position controller
        INITIAL_CLIMB,    ///< Climbing to RTL_ALT if currently below target altitude
        RETURN_HOME,      ///< Navigating horizontally to home/rally point at RTL altitude
        LOITER_AT_HOME,   ///< Loitering at home for RTL_LOITER_TIME seconds (if > 0)
        FINAL_DESCENT,    ///< Descending from RTL_ALT to RTL_ALT_FINAL
        LAND              ///< Final landing sequence to ground
    };
    
    /**
     * @brief Get current RTL phase
     * @return Current SubMode (STARTING, INITIAL_CLIMB, RETURN_HOME, etc.)
     */
    SubMode state() { return _state; }

    /**
     * @brief Check if current RTL phase is complete
     * @return true if current phase goals achieved and ready to transition
     * 
     * @note This accessor probably shouldn't be public - used for testing/debugging
     */
    bool state_complete() const { return _state_complete; }

    /**
     * @brief Check if vehicle is in landing phase
     * @return true if in LAND phase of RTL
     */
    virtual bool is_landing() const override;

    /**
     * @brief Restart RTL without terrain following after terrain data loss
     * 
     * @details If terrain data becomes unavailable during terrain-based RTL,
     *          this rebuilds the RTL path using relative (non-terrain) altitudes
     *          to allow RTL to continue safely.
     */
    void restart_without_terrain();

    /**
     * @enum RTLAltType
     * @brief RTL altitude reference type (parameter RTL_ALT_TYPE)
     * 
     * @details Determines how RTL_ALT is interpreted:
     *          - RELATIVE: RTL_ALT is altitude above home (EKF origin)
     *          - TERRAIN: RTL_ALT is altitude above terrain (requires terrain data)
     */
    enum class RTLAltType : int8_t {
        RELATIVE = 0,  ///< Altitude relative to home/EKF origin
        TERRAIN = 1    ///< Altitude above terrain (requires rangefinder or terrain database)
    };
    
    /**
     * @brief Get configured RTL altitude reference type
     * @return RELATIVE or TERRAIN based on RTL_ALT_TYPE parameter
     */
    ModeRTL::RTLAltType get_alt_type() const;

protected:

    /**
     * @brief Get mode name string
     * @return "RTL"
     */
    const char *name() const override { return "RTL"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "RTL " (with trailing space for alignment)
     */
    const char *name4() const override { return "RTL "; }

    /**
     * @brief Get distance to target waypoint in meters
     * @return Distance from current position to active RTL target in meters
     * 
     * @details Returns distance based on current RTL phase:
     *          - RETURN_HOME: Distance to home/rally point
     *          - Other phases: Distance from position controller
     */
    float wp_distance_m() const override;
    
    /**
     * @brief Get bearing to target waypoint in degrees
     * @return Bearing from current position to active RTL target (0-360 degrees)
     */
    float wp_bearing_deg() const override;
    
    /**
     * @brief Get crosstrack error from desired path
     * @return Crosstrack error in meters
     */
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

    /**
     * @brief Initialize descent phase
     * 
     * @details Sets up final descent from RTL_ALT to RTL_ALT_FINAL.
     *          Configures position and altitude controllers for descent.
     *          Called when transitioning to FINAL_DESCENT phase.
     */
    void descent_start();
    
    /**
     * @brief Execute descent phase control
     * 
     * @details Runs position hold with altitude descent:
     *          - Maintains horizontal position above home
     *          - Descends at RTL descent rate to RTL_ALT_FINAL
     *          - Monitors for descent completion
     *          - Transitions to LAND phase when target altitude reached
     */
    void descent_run();
    
    /**
     * @brief Initialize landing phase
     * 
     * @details Sets up final landing sequence. Configures land detector,
     *          precision landing if available, and landing descent rate.
     *          Called when transitioning to LAND phase.
     */
    void land_start();
    
    /**
     * @brief Execute landing phase control
     * @param[in] disarm_on_land If true, automatically disarm motors after landing
     * 
     * @details Runs landing sequence with position hold:
     *          - Maintains horizontal position (or precision landing if active)
     *          - Descends at landing rate with ground effect compensation
     *          - Slows descent near ground for soft touchdown
     *          - Detects landing and disarms if requested
     */
    void land_run(bool disarm_on_land);

    /**
     * @brief Set descent target altitude
     * @param[in] alt Altitude in cm for descent target
     * 
     * @details Updates rtl_path descent target altitude. Used when altitude
     *          needs adjustment during RTL (e.g., terrain following updates).
     */
    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    /**
     * @brief Initialize climb phase
     * 
     * @details Sets up initial climb to RTL_ALT if currently below target.
     *          Configures position controller to climb while maintaining horizontal position.
     *          Called when transitioning to INITIAL_CLIMB phase.
     */
    void climb_start();
    
    /**
     * @brief Initialize return home phase
     * 
     * @details Sets up waypoint navigation to home/rally point.
     *          Configures wp_nav with return target from rtl_path.
     *          Sets appropriate speed and acceleration limits.
     *          Called when transitioning to RETURN_HOME phase.
     */
    void return_start();
    
    /**
     * @brief Execute climb or return phase control
     * 
     * @details Runs control for INITIAL_CLIMB and RETURN_HOME phases:
     *          - INITIAL_CLIMB: Climbs to RTL_ALT while holding horizontal position
     *          - RETURN_HOME: Navigates to home while maintaining RTL_ALT
     *          - Monitors for phase completion (altitude or position goals reached)
     *          - Transitions to next phase when complete
     */
    void climb_return_run();
    
    /**
     * @brief Initialize loiter at home phase
     * 
     * @details Sets up loiter position controller at home location.
     *          Starts loiter timer for RTL_LOITER_TIME duration.
     *          Only called if RTL_LOITER_TIME > 0.
     */
    void loiterathome_start();
    
    /**
     * @brief Execute loiter at home phase control
     * 
     * @details Maintains position at home location:
     *          - Loiters at home using position controller
     *          - Counts down RTL_LOITER_TIME seconds
     *          - Allows pilot yaw control
     *          - Transitions to FINAL_DESCENT when loiter time expires
     */
    void loiterathome_run();
    
    /**
     * @brief Build complete RTL path with all waypoints
     * 
     * @details Computes full RTL path structure:
     *          - origin_point: Current vehicle position at RTL start
     *          - climb_target: Target for initial climb phase (if needed)
     *          - return_target: Home or rally point location
     *          - descent_target: Location for final descent
     *          - Considers terrain following if RTL_ALT_TYPE = TERRAIN
     *          - Determines if climb is needed based on current altitude
     *          - Selects between home and rally points
     */
    void build_path();
    
    /**
     * @brief Compute return target location (home vs rally point)
     * 
     * @details Determines where vehicle should return to:
     *          - Evaluates all rally points for distance and feasibility
     *          - Compares rally points to home location
     *          - Selects closest valid target
     *          - Stores result in rtl_path.return_target
     *          - Considers terrain constraints and fence boundaries
     */
    void compute_return_target();

    SubMode _state = SubMode::INITIAL_CLIMB;  ///< Current RTL phase (climb, return, loiter, descend, land)
    bool _state_complete = false; ///< true when current phase goals achieved, ready for next phase transition

    /**
     * @brief RTL path waypoints structure
     * 
     * @details Complete RTL path computed at initialization, used throughout RTL phases.
     *          All locations in NEU (North-East-Up) frame with altitudes in cm.
     *          Altitude reference depends on terrain following setting:
     *          - If terrain following disabled: Z = altitude above EKF origin (home)
     *          - If terrain following enabled: Z = altitude above terrain
     */
    struct {
        Location origin_point;    ///< Vehicle position when RTL started (reference point)
        Location climb_target;    ///< Target location for initial climb phase (if climb needed)
        Location return_target;   ///< Home or rally point location for return phase
        Location descent_target;  ///< Target location for final descent phase (usually same as return_target)
        bool land;                ///< true if RTL should land, false to loiter (normally true)
    } rtl_path;

    /**
     * @enum ReturnTargetAltType
     * @brief Internal altitude type for return target computation
     * 
     * @details Used during path building to determine altitude reference source.
     *          More detailed than RTLAltType parameter for internal path planning.
     */
    enum class ReturnTargetAltType {
        RELATIVE = 0,       ///< Altitude relative to home (EKF origin)
        RANGEFINDER = 1,    ///< Altitude above ground from rangefinder
        TERRAINDATABASE = 2 ///< Altitude above terrain from terrain database
    };

    uint32_t _loiter_start_time; ///< System time (ms) when loiter at home phase started

    bool terrain_following_allowed; ///< true if terrain following is configured and terrain data available

    /**
     * @enum Options
     * @brief RTL_OPTIONS parameter bit flags
     * 
     * @details Bitfield options to customize RTL behavior via RTL_OPTIONS parameter.
     *          Each bit enables/disables a specific feature.
     */
    enum class Options : int32_t {
        IgnorePilotYaw = (1U << 2),  ///< If set, pilot yaw input is ignored during RTL (autopilot controls yaw)
    };

};


/**
 * @class ModeSmartRTL
 * @brief Smart Return to Launch - returns by retracing outbound path
 * 
 * @details SmartRTL (also called Safe RTL) records the vehicle's path during flight
 *          and returns by following that path in reverse, avoiding obstacles
 *          encountered on the outbound journey:
 *          
 *          SmartRTL Phases:
 *          1. WAIT_FOR_PATH_CLEANUP: Path simplification in progress, loitering
 *          2. PATH_FOLLOW: Flying reverse path back toward launch
 *          3. PRELAND_POSITION: Positioning for precision landing if available
 *          4. DESCEND: Descending to landing altitude
 *          5. LAND: Final landing sequence
 *          
 *          Path Recording:
 *          - Continuously records position during flight (all modes)
 *          - Path stored in AP_SmartRTL library (finite memory buffer)
 *          - Path simplified to reduce memory usage (Douglas-Peucker algorithm)
 *          - Older path points dropped if buffer fills
 *          - Records 3D position waypoints at key locations
 *          
 *          Path Following:
 *          - Follows recorded path in reverse order (LIFO)
 *          - Uses waypoint navigation controller (wp_nav)
 *          - Maintains altitude from recorded path
 *          - Each waypoint popped from path as it's reached
 *          - Falls back to standard RTL if path unavailable
 *          
 *          Advantages over Standard RTL:
 *          - Avoids obstacles by retracing safe outbound route
 *          - More predictable return path
 *          - Useful in environments with obstacles (urban, forest, etc.)
 *          - Shorter return distance if outbound path was indirect
 *          
 *          Fallback Behavior:
 *          - If no path recorded: Switches to standard RTL mode
 *          - If path exhausted before home: Switches to standard RTL
 *          - If path retrieval fails: Switches to standard RTL
 *          - Timeout protection if stuck following path
 *          
 *          Memory Considerations:
 *          - Path stored in limited memory buffer (configurable)
 *          - Longer flights may lose early path points
 *          - Path simplification reduces memory usage
 *          - Monitor SRTL_POINTS and SRTL_ACCURACY parameters
 *          
 *          Requirements:
 *          - GPS lock (3D fix required)
 *          - SmartRTL path must be available (recorded during flight)
 *          - EKF healthy with position estimate
 *          - Sufficient battery for return journey
 * 
 * @note SmartRTL path is cleared on disarm - records fresh path each flight
 * @note Falls back to RTL if path unavailable, so RTL_ALT should still be set appropriately
 * @warning Path memory is finite - long flights may lose early path points
 * @warning Test thoroughly in safe area before relying on SmartRTL
 */
class ModeSmartRTL : public ModeRTL {

public:
    // inherit constructor
    using ModeRTL::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::SMART_RTL (21)
     */
    Number mode_number() const override { return Number::SMART_RTL; }

    /**
     * @brief Initialize SmartRTL mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful, false if path unavailable or GPS not ready
     * 
     * @details Initialization:
     *          - Requests path cleanup/simplification from AP_SmartRTL library
     *          - Waits for cleanup to complete before starting return
     *          - Falls back to standard RTL if path not available
     *          - Starts in WAIT_FOR_PATH_CLEANUP or PATH_FOLLOW state
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for SmartRTL mode, called at scheduler rate (typically 400Hz)
     * 
     * @details Executes SmartRTL state machine:
     *          - WAIT_FOR_PATH_CLEANUP: Loiters while path is simplified
     *          - PATH_FOLLOW: Navigates to next path waypoint in reverse order
     *          - PRELAND_POSITION: Positions over precision landing target
     *          - DESCEND: Descends to landing altitude
     *          - LAND: Executes landing sequence
     *          - Falls back to RTL if path following fails
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return true - SmartRTL absolutely requires GPS for path following
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - SmartRTL uses automatic throttle (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return false - cannot arm in SmartRTL mode (it's a failsafe/return mode)
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return true - SmartRTL is fully autonomous
     */
    bool is_autopilot() const override { return true; }

    /**
     * @brief Save current position to SmartRTL path
     * 
     * @details Called periodically during flight to record path waypoints.
     *          Position is added to AP_SmartRTL library's path buffer.
     *          Should be called from all flight modes to maintain complete path.
     * 
     * @note This is called by the main scheduler, not just by SmartRTL mode
     */
    void save_position();
    
    /**
     * @brief Exit SmartRTL mode
     * 
     * @details Cleanup when leaving SmartRTL mode:
     *          - Restores last popped waypoint back to path (if available)
     *          - Allows resuming SmartRTL from same point if mode is re-entered
     *          - Clears backup waypoint after restoration
     */
    void exit() override;

    /**
     * @brief Check if vehicle is in landing phase
     * @return true if in DESCEND or LAND phase of SmartRTL
     */
    bool is_landing() const override;
    
    /**
     * @brief Check if pilot yaw control is allowed
     * @return true if pilot can control yaw during SmartRTL
     * 
     * @details Inherits yaw control behavior from RTL base class.
     *          Typically allows pilot yaw unless configured otherwise.
     */
    bool use_pilot_yaw() const override;

    /**
     * @enum SubMode
     * @brief SmartRTL phase states - progression through return sequence
     * 
     * @details SmartRTL has different phases than standard RTL due to path following.
     *          Phases progress based on path following completion and altitude goals.
     */
    enum class SubMode : uint8_t {
        WAIT_FOR_PATH_CLEANUP, ///< Waiting for AP_SmartRTL to simplify path, loitering in place
        PATH_FOLLOW,           ///< Following recorded path in reverse, popping waypoints as reached
        PRELAND_POSITION,      ///< Positioning for precision landing over target
        DESCEND,               ///< Descending from current altitude to landing altitude
        LAND                   ///< Final landing sequence to ground
    };

protected:

    /**
     * @brief Get mode name string
     * @return "SMARTRTL"
     */
    const char *name() const override { return "SMARTRTL"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "SRTL"
     */
    const char *name4() const override { return "SRTL"; }

    /**
     * @brief Get current target waypoint for GCS reporting
     * @param[out] loc Location of current path following target waypoint
     * @return true if valid waypoint available, false otherwise
     * 
     * @details Returns the next waypoint from the SmartRTL path that vehicle
     *          is currently navigating toward. If path is exhausted or unavailable,
     *          returns false.
     */
    bool get_wp(Location &loc) const override;
    
    /**
     * @brief Get distance to target waypoint in meters
     * @return Distance from current position to next path waypoint in meters
     */
    float wp_distance_m() const override;
    
    /**
     * @brief Get bearing to target waypoint in degrees
     * @return Bearing from current position to next path waypoint (0-360 degrees)
     */
    float wp_bearing_deg() const override;
    
    /**
     * @brief Get crosstrack error from path following
     * @return Crosstrack error in meters from desired path
     */
    float crosstrack_error() const override { return wp_nav->crosstrack_error();}

private:

    /**
     * @brief Execute wait for cleanup phase control
     * 
     * @details Loiters in place while AP_SmartRTL library simplifies the path:
     *          - Maintains current horizontal position using position controller
     *          - Holds current altitude
     *          - Allows pilot yaw control
     *          - Monitors path cleanup completion
     *          - Transitions to PATH_FOLLOW when path is ready
     *          - Falls back to RTL if cleanup times out or fails
     */
    void wait_cleanup_run();
    
    /**
     * @brief Execute path following phase control
     * 
     * @details Navigates along recorded path in reverse:
     *          - Pops next waypoint from path (LIFO order)
     *          - Commands wp_nav to fly to waypoint
     *          - Monitors waypoint approach and arrival
     *          - Pops next waypoint when current one is reached
     *          - Falls back to RTL if path exhausted before home
     *          - Transitions to PRELAND_POSITION or LAND when path complete
     *          - Timeout protection: switches to RTL if stuck without progress
     */
    void path_follow_run();
    
    /**
     * @brief Execute precision landing position phase control
     * 
     * @details Positions vehicle over precision landing target:
     *          - Acquires precision landing target (IR beacon, visual marker)
     *          - Adjusts position to be directly over target
     *          - Maintains altitude while repositioning
     *          - Transitions to DESCEND when positioned correctly
     *          - Skipped if precision landing not available
     */
    void pre_land_position_run();
    
    /**
     * @brief Execute landing phase
     * 
     * @details Runs final landing sequence with position hold.
     *          Uses inherited land functionality from ModeRTL base class.
     *          Called when transitioning to LAND phase.
     */
    void land();
    
    SubMode smart_rtl_state = SubMode::PATH_FOLLOW; ///< Current SmartRTL phase state

    /**
     * @brief Timestamp of last failed path waypoint pop attempt
     * 
     * @details Tracks how long vehicle has been unable to retrieve next waypoint
     *          from path. Used for timeout detection - if too much time passes
     *          without progress, SmartRTL gives up and falls back to standard RTL
     *          or lands in place to avoid running out of battery.
     * 
     * System time in milliseconds, or 0 if no recent failures.
     */
    uint32_t path_follow_last_pop_fail_ms;

    /**
     * @brief Backup of last waypoint popped from path
     * 
     * @details Stores the most recently popped waypoint in NED frame (cm) relative
     *          to EKF origin. If vehicle exits SmartRTL mode before completing the
     *          return, this waypoint can be restored to the path, allowing SmartRTL
     *          to resume from the same position if re-entered.
     * 
     *          Set to zero vector (invalid) when no backup is stored.
     */
    Vector3f dest_NED_backup;
};


/**
 * @class ModeSport
 * @brief Sport flight mode - earth-frame angular rate control with automatic altitude
 * 
 * @details Sport mode provides intuitive earth-frame rate control for aggressive flying:
 *          - Roll/pitch sticks command earth-frame angular rates (deg/s)
 *          - Vehicle maintains orientation relative to pilot's viewpoint
 *          - Yaw stick commands yaw rate
 *          - Automatic altitude hold with throttle control
 *          - No GPS required - works indoors or GPS-denied
 *          
 *          Control behavior:
 *          - Roll stick: Earth-frame roll rate (vehicle rolls relative to horizon)
 *          - Pitch stick: Earth-frame pitch rate (vehicle pitches relative to horizon)
 *          - Yaw stick: Body-frame yaw rate (standard yaw rate control)
 *          - Throttle stick: Climb/descend, holds altitude when centered
 *          
 *          Earth-frame vs body-frame rate control:
 *          - ACRO mode: Body-frame rates (stick input relative to vehicle)
 *          - SPORT mode: Earth-frame rates (stick input relative to ground/horizon)
 *          - Sport mode feels more intuitive - forward pitch always moves away from pilot
 *          - Useful for FPV flying and aggressive maneuvers
 *          
 *          Use cases:
 *          - Aggressive manual flying without GPS
 *          - Indoor flying in large spaces
 *          - FPV racing and freestyle
 *          - Learning advanced manual control
 *          - Emergency GPS-denied flight
 *          
 *          Requirements:
 *          - Good attitude estimate (gyros + accelerometers)
 *          - Barometer for altitude hold
 *          - EKF healthy (GPS not required)
 * 
 * @note Sport mode does not prevent aggressive attitudes - pilot can flip vehicle
 * @note No position hold - vehicle will drift in wind
 * @warning Practice in safe area - Sport mode allows aggressive maneuvers
 * @warning Altitude hold only - no GPS position hold, vehicle will drift
 */
class ModeSport : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::SPORT (13)
     */
    Number mode_number() const override { return Number::SPORT; }

    /**
     * @brief Initialize Sport mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful, false if attitude controller not ready
     * 
     * @details Initialization:
     *          - Initializes altitude controller at current altitude
     *          - Configures attitude controller for rate control mode
     *          - Sets earth-frame rate limits
     *          - Resets throttle integrator
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for Sport mode, called at scheduler rate (typically 400Hz)
     * 
     * @details Executes Sport control logic:
     *          - Processes pilot stick inputs for earth-frame rates
     *          - Transforms earth-frame rates to body-frame rates
     *          - Runs attitude rate controller
     *          - Runs altitude controller for vertical position
     *          - Updates motor outputs
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return false - Sport mode works without GPS (uses altitude hold only)
     */
    bool requires_GPS() const override { return false; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - Sport uses automatic throttle (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return true - vehicle can be armed in Sport mode
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - Sport is pilot-controlled with altitude assist
     */
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Check if mode supports user takeoff
     * @param[in] must_navigate True if takeoff requires waypoint navigation capability
     * @return true if takeoff supported (no navigation), false if navigation required
     * 
     * @details Sport supports simple altitude-based takeoff without navigation.
     */
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    /**
     * @brief Get mode name string
     * @return "SPORT"
     */
    const char *name() const override { return "SPORT"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "SPRT"
     */
    const char *name4() const override { return "SPRT"; }

private:

};


/**
 * @class ModeStabilize
 * @brief Stabilize flight mode - manual attitude control with manual throttle
 * 
 * @details Stabilize is the most basic manual flight mode, providing pilot-commanded
 *          attitude angles with self-leveling assistance:
 *          
 *          Control behavior:
 *          - Roll stick: Commands roll angle (degrees left/right from level)
 *          - Pitch stick: Commands pitch angle (degrees forward/back from level)
 *          - Yaw stick: Commands yaw rate (degrees/second rotation)
 *          - Throttle stick: Direct throttle control (pilot manages altitude)
 *          
 *          Self-leveling:
 *          - When sticks centered: Vehicle automatically levels (0 roll, 0 pitch)
 *          - Attitude controller maintains commanded angles
 *          - No drift correction - vehicle will move with wind
 *          - Returns to level when pilot releases sticks
 *          
 *          Throttle management:
 *          - Manual throttle - pilot directly controls motor output
 *          - No altitude hold or automatic climb/descent
 *          - Throttle stick position = motor throttle (with hover throttle learning)
 *          - Pilot must actively manage altitude at all times
 *          
 *          Use cases:
 *          - Initial learning and training
 *          - Manual flying without GPS
 *          - Emergency GPS-denied flight
 *          - Indoor flying
 *          - Baseline mode for system testing
 *          
 *          Requirements:
 *          - Gyroscopes for attitude stabilization
 *          - Accelerometers for level reference
 *          - No GPS required
 *          - No barometer required (though helpful for pilot awareness)
 *          
 *          Failsafe behavior:
 *          - Cannot be entered during RC failsafe (pilot input required)
 *          - Manual throttle prevents safe autonomous landing
 *          - Must switch to altitude-hold mode for RC failsafe protection
 *          
 *          Training features:
 *          - Supports trim saving (saves level attitude offsets)
 *          - Supports auto-trim (automatically adjusts trim during flight)
 *          - Supports autotune (can run autotune in this mode)
 *          - Supports flip maneuver (automated flip capability)
 * 
 * @note Most basic flyable mode - recommended for initial flights
 * @note Requires active throttle management by pilot at all times
 * @warning No altitude hold - pilot must manage throttle constantly
 * @warning Vehicle will drift in wind - no position corrections
 */
class ModeStabilize : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::STABILIZE (0)
     */
    Number mode_number() const override { return Number::STABILIZE; }

    /**
     * @brief Main loop for Stabilize mode, called at scheduler rate (typically 400Hz)
     * 
     * @details Executes Stabilize control logic:
     *          - Reads pilot stick inputs for desired roll/pitch angles and yaw rate
     *          - Applies stick expo and angle limits
     *          - Commands attitude controller to achieve desired attitude
     *          - Passes pilot throttle directly to motors (with hover learning)
     *          - Updates motor outputs
     */
    virtual void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return false - Stabilize works without GPS
     */
    bool requires_GPS() const override { return false; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return true - Stabilize uses manual throttle (pilot controls altitude)
     */
    bool has_manual_throttle() const override { return true; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return true - vehicle can be armed in Stabilize mode
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - Stabilize is fully pilot-controlled
     */
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Check if mode allows saving trim values
     * @return true - trim can be saved to adjust level attitude reference
     * 
     * @details Saving trim adjusts the zero roll/pitch reference point to
     *          compensate for sensor mounting offsets or CG imbalance.
     */
    bool allows_save_trim() const override { return true; }
    
    /**
     * @brief Check if mode allows automatic trim adjustment
     * @return true - auto-trim can run to automatically adjust level reference
     * 
     * @details Auto-trim observes pilot inputs during flight and automatically
     *          adjusts trim to minimize required stick input for level flight.
     */
    bool allows_auto_trim() const override { return true; }
    
    /**
     * @brief Check if mode allows autotune
     * @return true - autotune can be initiated from Stabilize mode
     * 
     * @details Autotune automatically optimizes attitude controller PID gains
     *          for best performance on the specific vehicle.
     */
    bool allows_autotune() const override { return true; }
    
    /**
     * @brief Check if mode allows flip maneuver
     * @return true - automated flip can be triggered from Stabilize
     * 
     * @details Flip maneuver performs an automated roll or pitch flip and
     *          returns to Stabilize mode afterward.
     */
    bool allows_flip() const override { return true; }
    
    /**
     * @brief Check if mode can be entered during RC failsafe
     * @return false - cannot enter Stabilize during RC failsafe (manual throttle unsafe)
     * 
     * @details Manual throttle modes are not safe failsafe targets because pilot
     *          input is required for altitude control. Vehicle would fall without
     *          active throttle management.
     */
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    /**
     * @brief Get mode name string
     * @return "STABILIZE"
     */
    const char *name() const override { return "STABILIZE"; }
    
    /**
     * @brief Get mode short name (4 characters for displays)
     * @return "STAB"
     */
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
/**
 * @class ModeStabilize_Heli
 * @brief Helicopter-specific Stabilize mode with inverted flight support
 * 
 * @details Extends ModeStabilize for helicopter-specific control characteristics:
 *          
 *          Helicopter differences from multicopter Stabilize:
 *          - Supports inverted (negative collective) flight
 *          - Different control response due to rotor mechanics
 *          - Collective pitch control instead of motor speed variation
 *          - Rotor speed governor maintains constant RPM
 *          - Cyclic servos control attitude (roll/pitch)
 *          - Tail rotor servo controls yaw
 *          
 *          Inverted flight capability:
 *          - Allows negative collective pitch for inverted flight
 *          - Attitude controller adapts for inverted orientation
 *          - Pilot can fly helicopter upside down with control authority
 *          - Useful for advanced 3D aerobatics
 *          
 *          Helicopter control mechanics:
 *          - Collective stick: Blade pitch angle (controls vertical force)
 *          - Cyclic servos: Swashplate tilt (controls attitude)
 *          - Tail rotor: Anti-torque and yaw control
 *          - Governor: Maintains rotor RPM regardless of load
 *          
 *          Requirements:
 *          - Helicopter frame configuration (FRAME_CONFIG == HELI_FRAME)
 *          - Swashplate servo configuration
 *          - Tail rotor servo or tail motor
 *          - RSC (Rotor Speed Control) governor configured
 * 
 * @note Only available when compiled with HELI_FRAME configuration
 * @note Requires proper helicopter setup and swashplate calibration
 * @warning Inverted flight requires significant pilot skill and practice
 */
class ModeStabilize_Heli : public ModeStabilize {

public:
    // inherit constructor
    using ModeStabilize::Mode;

    /**
     * @brief Initialize helicopter Stabilize mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful
     * 
     * @details Helicopter-specific initialization:
     *          - Initializes swashplate servo positioning
     *          - Sets up rotor speed controller
     *          - Configures collective pitch limits
     *          - Prepares tail rotor control
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for helicopter Stabilize mode
     * 
     * @details Executes helicopter-specific control logic:
     *          - Processes pilot collective input for vertical control
     *          - Commands cyclic servos for attitude control
     *          - Commands tail rotor for yaw control
     *          - Manages rotor speed governor
     *          - Handles inverted flight control adaptation
     */
    void run() override;

    /**
     * @brief Check if inverted flight is allowed
     * @return true - helicopter Stabilize supports inverted (negative collective) flight
     * 
     * @details Helicopters with proper setup can fly inverted using negative collective
     *          pitch. The attitude controller adapts control authority for inverted orientation.
     */
    bool allows_inverted() const override { return true; };

protected:

private:

};
#endif

/**
 * @class ModeSystemId
 * @brief System identification mode - generates test signals for dynamic analysis
 * 
 * @details SystemID mode produces automated test signals (chirp waveforms) to analyze
 *          vehicle dynamic response and identify system characteristics:
 *          
 *          Purpose:
 *          - System identification for control system tuning
 *          - Frequency response analysis
 *          - Transfer function identification
 *          - Dynamic modeling and validation
 *          - PID gain optimization research
 *          
 *          Chirp waveform generation:
 *          - Sweeps frequency from start to stop over recording time
 *          - Configurable magnitude (excitation amplitude)
 *          - Fade-in/fade-out for smooth transitions
 *          - Constant frequency pre-chirp and post-chirp periods
 *          - High-rate logging for accurate frequency domain analysis
 *          
 *          Supported test configurations:
 *          - Angle input excitation (roll, pitch, yaw)
 *          - Rate input excitation (roll rate, pitch rate, yaw rate)
 *          - Mixer output excitation (roll, pitch, yaw, throttle)
 *          - Position controller excitation (velocity, position)
 *          - Recovery mode (angle stabilization with disturbance)
 *          
 *          Test axes (AxisType enum):
 *          - INPUT_ROLL/PITCH/YAW: Angle commands
 *          - RATE_ROLL/PITCH/YAW: Rate commands
 *          - MIX_ROLL/PITCH/YAW/THROTTLE: Direct mixer injection
 *          - INPUT_VEL_LAT/LONG: Velocity controller inputs
 *          - DISTURB_POS/VEL: Position/velocity disturbance rejection
 *          
 *          Configuration parameters (AP_Param):
 *          - axis: Which control axis to excite
 *          - waveform_magnitude: Excitation amplitude
 *          - frequency_start: Chirp start frequency (Hz)
 *          - frequency_stop: Chirp stop frequency (Hz)
 *          - time_fade_in: Fade-in duration (seconds)
 *          - time_record: Chirp duration (seconds)
 *          - time_fade_out: Fade-out duration (seconds)
 *          
 *          Logging:
 *          - High-rate SIDD (System ID Data) log messages
 *          - Captures commanded and achieved responses
 *          - Includes instantaneous frequency for analysis
 *          - Subsampling for manageable log sizes
 *          
 *          Typical workflow:
 *          1. Configure parameters (axis, frequency range, magnitude)
 *          2. Arm vehicle in hover (or suitable flight condition)
 *          3. Switch to SystemID mode to start test
 *          4. Chirp executes automatically with fade-in/out
 *          5. Mode exits automatically when complete
 *          6. Analyze log data in frequency domain (FFT, Bode plot)
 *          7. Use results to tune controller gains or validate models
 *          
 *          Safety features:
 *          - Manual throttle keeps pilot in control
 *          - Cannot arm in SystemID (must switch from another mode)
 *          - Fade-in/out prevents sudden excitations
 *          - Configurable magnitude limits
 *          
 *          Requirements:
 *          - Pre-tuned vehicle capable of stable flight
 *          - Sufficient battery for test duration
 *          - Safe test environment (space for possible drift)
 *          - High-rate logging enabled (SIDD messages)
 * 
 * @note Primarily used by developers and researchers for control system analysis
 * @note Requires understanding of frequency response analysis to interpret results
 * @warning Test in safe area - vehicle may drift or exhibit unusual behavior during test
 * @warning Do not use operationally - this is a diagnostic/tuning mode only
 */
class ModeSystemId : public Mode {

public:
    /**
     * @brief Constructor - initializes SystemID parameters
     * 
     * @details Sets up AP_Param variables for user configuration and
     *          initializes chirp waveform generator.
     */
    ModeSystemId(void);
    
    /**
     * @brief Get flight mode number
     * @return Number::SYSTEMID (25)
     */
    Number mode_number() const override { return Number::SYSTEMID; }

    /**
     * @brief Initialize SystemID mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful
     * 
     * @details Initialization:
     *          - Validates axis parameter configuration
     *          - Resets chirp waveform timing
     *          - Disables body-frame feedforward if needed
     *          - Initializes position controller if testing pos/vel axes
     *          - Starts logging subsystem
     *          - Sets state to TESTING
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for SystemID mode
     * 
     * @details Executes system identification test:
     *          - Generates chirp waveform sample
     *          - Applies excitation to selected axis
     *          - Runs appropriate controller (attitude, rate, position, or mixer)
     *          - Logs input and output signals at high rate
     *          - Manages test phases (fade-in, chirp, fade-out)
     *          - Exits mode automatically when test completes
     */
    void run() override;
    
    /**
     * @brief Exit SystemID mode
     * 
     * @details Cleanup when leaving SystemID:
     *          - Restores body-frame feedforward setting
     *          - Stops chirp generation
     *          - Finalizes logging
     *          - Resets state to STOPPED
     */
    void exit() override;

    /**
     * @brief Check if mode requires GPS
     * @return false - SystemID can run without GPS (most tests are attitude/rate)
     */
    bool requires_GPS() const override { return false; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return true - pilot controls altitude manually during test
     * 
     * @details Manual throttle allows pilot to maintain safe altitude while
     *          automated excitation is applied to other axes.
     */
    bool has_manual_throttle() const override { return true; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return false - cannot arm in SystemID (must switch from another mode after arming)
     * 
     * @details Vehicle must be armed and stable in another mode before entering SystemID.
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - SystemID has automated excitation but pilot controls throttle
     */
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Check if mode logs attitude data
     * @return true - SystemID generates high-rate attitude/rate logs (SIDD messages)
     */
    bool logs_attitude() const override { return true; }

    /**
     * @brief Set chirp magnitude from external source (scripting, GCS)
     * @param[in] input New magnitude value for waveform excitation
     */
    void set_magnitude(float input) { waveform_magnitude.set(input); }

    static const struct AP_Param::GroupInfo var_info[]; ///< Parameter definition table for SystemID configuration

    Chirp chirp_input; ///< Chirp waveform generator (from AP_Math library)

protected:

    const char *name() const override { return "SYSTEMID"; }
    const char *name4() const override { return "SYSI"; }

private:

    /**
     * @brief Log system identification data at high rate
     * 
     * @details Writes SIDD (System ID Data) log message containing:
     *          - Waveform sample (commanded excitation)
     *          - Measured response (attitude, rate, position, or velocity)
     *          - Instantaneous frequency
     *          - Time reference
     *          - Axis identifier
     *          - Test phase
     *          Subsampling is applied to reduce log size while maintaining
     *          sufficient resolution for frequency domain analysis.
     */
    void log_data() const;
    
    /**
     * @brief Check if selected axis type uses position controller
     * @return true if axis is position or velocity based (requires GPS/position estimate)
     */
    bool is_poscontrol_axis_type() const;

    /**
     * @enum AxisType
     * @brief Defines which control axis to excite with chirp signal
     * 
     * @details Different axis types inject the excitation signal at different points
     *          in the control chain, allowing analysis of different control loops.
     */
    enum class AxisType {
        NONE = 0,               ///< No excitation - mode disabled
        INPUT_ROLL = 1,         ///< Excite angle input roll axis (attitude controller input)
        INPUT_PITCH = 2,        ///< Excite angle input pitch axis (attitude controller input)
        INPUT_YAW = 3,          ///< Excite angle input yaw axis (attitude controller input)
        RECOVER_ROLL = 4,       ///< Excite angle roll with recovery (disturbance rejection test)
        RECOVER_PITCH = 5,      ///< Excite angle pitch with recovery (disturbance rejection test)
        RECOVER_YAW = 6,        ///< Excite angle yaw with recovery (disturbance rejection test)
        RATE_ROLL = 7,          ///< Excite rate roll axis (rate controller input)
        RATE_PITCH = 8,         ///< Excite rate pitch axis (rate controller input)
        RATE_YAW = 9,           ///< Excite rate yaw axis (rate controller input)
        MIX_ROLL = 10,          ///< Excite mixer roll axis (motor mixer input)
        MIX_PITCH = 11,         ///< Excite mixer pitch axis (motor mixer input)
        MIX_YAW = 12,           ///< Excite mixer yaw axis (motor mixer input)
        MIX_THROTTLE = 13,      ///< Excite mixer throttle axis (motor mixer input)
        DISTURB_POS_LAT = 14,   ///< Lateral body axis position disturbance (pos controller)
        DISTURB_POS_LONG = 15,  ///< Longitudinal body axis position disturbance (pos controller)
        DISTURB_VEL_LAT = 16,   ///< Lateral body axis velocity disturbance (vel controller)
        DISTURB_VEL_LONG = 17,  ///< Longitudinal body axis velocity disturbance (vel controller)
        INPUT_VEL_LAT = 18,     ///< Lateral body axis commanded velocity excitation
        INPUT_VEL_LONG = 19,    ///< Longitudinal body axis commanded velocity excitation
    };

    AP_Int8 axis;               ///< Controls which axis is being excited (see AxisType enum)
    AP_Float waveform_magnitude;///< Magnitude of chirp waveform (amplitude of excitation)
    AP_Float frequency_start;   ///< Frequency at the start of the chirp (Hz)
    AP_Float frequency_stop;    ///< Frequency at the end of the chirp (Hz)
    AP_Float time_fade_in;      ///< Time to reach maximum amplitude of chirp (seconds)
    AP_Float time_record;       ///< Time taken to complete the chirp waveform (seconds)
    AP_Float time_fade_out;     ///< Time to reach zero amplitude after chirp finishes (seconds)

    bool att_bf_feedforward;    ///< Saved setting of attitude_control body-frame feedforward (restored on exit)
    float waveform_time;        ///< Time reference for waveform generation (elapsed time in test)
    float waveform_sample;      ///< Current waveform sample output value
    float waveform_freq_rads;   ///< Instantaneous waveform frequency (rad/s) for logging
    float time_const_freq;      ///< Duration of constant frequency before chirp starts
    int8_t log_subsample;       ///< Subsample multiple for logging (reduces log size)
    Vector2f target_vel;        ///< Target velocity for position controller modes (m/s, body frame)
    Vector2f target_pos;        ///< Target position for position controller modes (m, NE frame)
    Vector2f input_vel_last;    ///< Last cycle input velocity (for derivative calculation)
    
    /**
     * @enum SystemIDModeState
     * @brief System identification mode state machine
     */
    enum class SystemIDModeState {
        SYSTEMID_STATE_STOPPED, ///< Not running test (before init or after exit)
        SYSTEMID_STATE_TESTING  ///< Actively generating chirp and logging data
    } systemid_state;
};

/**
 * @class ModeThrow
 * @brief Throw-to-start mode - launches vehicle by throwing it
 * 
 * @details Throw mode enables hand-launching of small multirotors by detecting the throw
 *          motion and automatically taking control in mid-air:
 *          
 *          Throw mode phases:
 *          1. Disarmed: Waiting for vehicle to be armed (motors typically stopped)
 *          2. Detecting: Armed, waiting for throw motion to be detected
 *          3. Wait_Throttle_Unlimited: Throw detected, waiting for motor spool-up
 *          4. Uprighting: Leveling vehicle attitude after throw
 *          5. HgtStabilise: Stabilizing altitude after throw
 *          6. PosHold: Holding position after successful launch
 *          
 *          Throw detection criteria:
 *          - Acceleration spike indicating rapid movement
 *          - Followed by period of low acceleration (free fall)
 *          - Vertical velocity indicates upward or downward motion
 *          - Vehicle attitude must be reasonable for flight
 *          
 *          Throw types:
 *          - Upward: Traditional throw upward (detects acceleration then free-fall)
 *          - Drop: Drop-and-catch launch (detects free-fall only)
 *          
 *          Pre-throw motor state:
 *          - STOPPED: Motors stopped before throw (safer for handling)
 *          - RUNNING: Motors spinning slowly before throw (faster response)
 *          
 *          Safety features:
 *          - Must arm in Throw mode before throwing
 *          - Throw detection timeout - won't activate if not thrown promptly
 *          - Attitude checks - won't stabilize if orientation too extreme
 *          - Height checks - must achieve minimum altitude for position hold
 *          - Position validity checks - needs GPS lock for final phase
 *          - Falls back to Land mode if conditions not met
 *          
 *          Typical workflow:
 *          1. Hold vehicle in hand, select Throw mode
 *          2. Arm vehicle (motors spin slowly or remain stopped depending on config)
 *          3. Throw vehicle upward (or drop from height)
 *          4. Vehicle detects throw motion
 *          5. Motors spin up rapidly
 *          6. Attitude controller levels vehicle
 *          7. Altitude controller stabilizes height
 *          8. Position controller engages for loiter
 *          9. Pilot can take manual control or vehicle holds position
 *          
 *          Requirements:
 *          - GPS lock (3D fix) for position hold after throw
 *          - Accelerometer calibrated for accurate throw detection
 *          - EKF healthy for position estimation
 *          - Small, lightweight vehicle (typically <2kg)
 *          - Sufficient throw velocity (typically >3 m/s upward)
 *          
 *          Configuration parameters:
 *          - THROW_MOT_START: Pre-throw motor state (stopped or running)
 *          - THROW_TYPE: Throw type (upward or drop)
 *          - THROW_NEXTMODE: Mode to switch to after successful throw
 *          
 *          Common use cases:
 *          - Hand-launching small indoor multirotors
 *          - Launching in confined spaces without takeoff room
 *          - Quick deployment from moving platform
 *          - Emergency deployment scenarios
 * 
 * @note Requires practice to throw with correct velocity and orientation
 * @note Vehicle must be lightweight enough to safely throw by hand
 * @warning Test thoroughly at low height over soft surface before operational use
 * @warning Do not throw too hard - excessive velocity can cause loss of control
 * @warning Ensure clear space around throw area - vehicle may drift initially
 */
class ModeThrow : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::THROW (18)
     */
    Number mode_number() const override { return Number::THROW; }

    /**
     * @brief Initialize Throw mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful
     * 
     * @details Initialization:
     *          - Checks GPS availability (required for final position hold)
     *          - Resets throw detection state machine to Disarmed or Detecting
     *          - Configures motor state based on THROW_MOT_START parameter
     *          - Initializes throw detection variables
     *          - Clears attempt flags
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for Throw mode
     * 
     * @details Executes throw detection and stabilization state machine:
     *          - Throw_Disarmed: Waiting for arming
     *          - Throw_Detecting: Monitoring accelerometer for throw motion
     *          - Throw_Wait_Throttle_Unlimited: Waiting for motors to spool up
     *          - Throw_Uprighting: Commanding attitude to level flight
     *          - Throw_HgtStabilise: Stabilizing altitude
     *          - Throw_PosHold: Holding position, ready for pilot control
     *          
     *          Logs throw detection events and state transitions for debugging.
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return true - Throw mode requires GPS for final position hold phase
     * 
     * @details While initial throw detection uses accelerometer only, the final
     *          PosHold phase requires GPS position estimate to maintain location.
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - Throw mode uses automatic throttle (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return true - vehicle can be armed in Throw mode (required for throw-to-start)
     * 
     * @details Arming in Throw mode is necessary to enable throw detection.
     *          Motors will be stopped or spinning slowly depending on configuration.
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - Throw is semi-autonomous (auto-stabilize after throw, then manual)
     */
    bool is_autopilot() const override { return false; }

    /**
     * @enum ThrowType
     * @brief Type of throw motion to detect
     */
    enum class ThrowType {
        Upward = 0, ///< Traditional upward throw - detects acceleration then free-fall
        Drop = 1    ///< Drop from height - detects free-fall only (no initial acceleration)
    };

    /**
     * @enum PreThrowMotorState
     * @brief Motor state before throw detection
     */
    enum class PreThrowMotorState {
        STOPPED = 0, ///< Motors stopped before throw (safer for handling, slower response)
        RUNNING = 1, ///< Motors spinning slowly before throw (faster response, requires care)
    };

protected:

    const char *name() const override { return "THROW"; }
    const char *name4() const override { return "THRW"; }

private:

    /**
     * @brief Check if throw motion has been detected
     * @return true if throw motion detected based on acceleration and velocity
     * 
     * @details Throw detection algorithm:
     *          - Upward throw: Detects initial acceleration spike followed by free-fall
     *          - Drop throw: Detects free-fall immediately
     *          - Monitors accelerometer for characteristic throw signature
     *          - Validates vertical velocity is appropriate for detected throw type
     *          - Records free-fall start time and velocity for subsequent phases
     */
    bool throw_detected();
    
    /**
     * @brief Check if position estimate is good enough for position hold
     * @return true if GPS has 3D fix and position estimate is valid
     * 
     * @details Validates that EKF position estimate is healthy and GPS quality
     *          is sufficient to maintain position after throw stabilization.
     */
    bool throw_position_good() const;
    
    /**
     * @brief Check if vehicle has achieved sufficient altitude
     * @return true if vehicle is above minimum altitude for position hold
     * 
     * @details Ensures vehicle has gained enough height above ground before
     *          transitioning to position hold. Prevents premature position
     *          hold engagement too close to ground.
     */
    bool throw_height_good() const;
    
    /**
     * @brief Check if vehicle attitude is acceptable for flight
     * @return true if roll and pitch angles are within safe limits for stabilization
     * 
     * @details Validates that vehicle orientation is close enough to level flight
     *          to begin attitude stabilization. Extreme angles may indicate
     *          tumbling or inappropriate throw.
     */
    bool throw_attitude_good() const;

    /**
     * @enum ThrowModeStage
     * @brief Throw mode state machine stages
     * 
     * @details Progression through throw sequence from arming to stable flight.
     *          Each stage has specific entry conditions and success criteria.
     */
    enum ThrowModeStage {
        Throw_Disarmed,                 ///< Vehicle not armed, waiting for arm command
        Throw_Detecting,                ///< Armed, monitoring sensors for throw motion
        Throw_Wait_Throttle_Unlimited,  ///< Throw detected, waiting for motors to spool up
        Throw_Uprighting,               ///< Leveling vehicle attitude to horizontal
        Throw_HgtStabilise,             ///< Stabilizing altitude at current height
        Throw_PosHold                   ///< Holding horizontal position, ready for pilot input
    };

    ThrowModeStage stage = Throw_Disarmed;      ///< Current throw mode stage
    ThrowModeStage prev_stage = Throw_Disarmed; ///< Previous stage for detecting transitions
    uint32_t last_log_ms;                       ///< Time of last throw event log (for rate limiting)
    bool nextmode_attempted;                    ///< True if automatic mode change has been attempted
    uint32_t free_fall_start_ms;                ///< System time when free fall was detected (milliseconds)
    float free_fall_start_velz;                 ///< Vertical velocity when free fall was detected (m/s)
};

#if MODE_TURTLE_ENABLED
/**
 * @class ModeTurtle
 * @brief Turtle mode - recovery from upside-down crash by reversing motors
 * 
 * @details Turtle mode enables recovery from an inverted crash (vehicle on its back)
 *          by temporarily reversing motor directions to flip the vehicle upright:
 *          
 *          Purpose:
 *          - Recover from upside-down landing/crash
 *          - Flip vehicle back to upright position without manual intervention
 *          - Enable continued flight after crash in racing or acrobatic scenarios
 *          - Reduce need for physical access to crashed vehicle
 *          
 *          How it works:
 *          1. Vehicle detects it's upside down (inverted attitude)
 *          2. Pilot switches to Turtle mode
 *          3. Motors automatically arm in Turtle mode
 *          4. Motor directions reversed (props spin opposite to normal)
 *          5. Pilot applies throttle and pitch/roll to flip vehicle
 *          6. Once upright, pilot switches back to normal flight mode
 *          7. Motor directions restore to normal, flight resumes
 *          
 *          Motor direction reversal:
 *          - All motor outputs reversed (clockwise becomes counter-clockwise)
 *          - Allows generating thrust away from ground when inverted
 *          - ESC must support bidirectional operation (BLHeli_S, BLHeli_32, etc.)
 *          - Motor direction change synchronized via semaphore protection
 *          
 *          Control inputs:
 *          - Throttle: Controls flip force/height
 *          - Roll/Pitch: Controls flip direction (which way to flip)
 *          - Yaw: Typically not used during flip
 *          - Pilot has direct control of motor output intensity
 *          
 *          Safety features:
 *          - Only allows arming when inverted (prevents accidental use)
 *          - Manual throttle prevents unintended motor spin-up
 *          - Periodic throttle warnings if motors running without input
 *          - Cannot enter from RC failsafe
 *          - Automatic disarm when exiting mode
 *          - Semaphore protection during motor direction changes
 *          
 *          ESC requirements:
 *          - Bidirectional ESC capable of motor reversal
 *          - BLHeli_S with bidirectional mode enabled
 *          - BLHeli_32 with bidirectional support
 *          - ESC must respond quickly to direction changes
 *          - PWM protocol supporting bidirectional operation
 *          
 *          Typical workflow:
 *          1. Vehicle crashes and lands upside down
 *          2. Pilot switches to Turtle mode
 *          3. Vehicle automatically arms motors (reversed)
 *          4. Pilot applies throttle and control input to flip
 *          5. Vehicle flips back to upright position
 *          6. Pilot switches to stabilize or other flight mode
 *          7. Motor directions restore, normal flight resumes
 *          8. Continue flying or land safely
 *          
 *          Flip technique:
 *          - Start with low throttle to verify motor direction
 *          - Apply roll or pitch input toward desired flip direction
 *          - Increase throttle to generate lift
 *          - Short burst typically sufficient for flip
 *          - Be ready to cut throttle once upright
 *          - Switch mode immediately after successful flip
 *          
 *          Configuration:
 *          - TURTLE mode must be enabled at compile time (MODE_TURTLE_ENABLED)
 *          - ESCs must be configured for bidirectional operation
 *          - Motor channels must support direction reversal
 *          - Verify ESC firmware supports required features
 *          
 *          Common use cases:
 *          - FPV racing (quick recovery from crashes)
 *          - Freestyle acrobatics (recovery from failed tricks)
 *          - Testing and development (multiple flight attempts)
 *          - Remote locations (no manual access to flip vehicle)
 *          
 *          Limitations:
 *          - Only works with bidirectional ESCs
 *          - Requires sufficient battery voltage for flip
 *          - May not work if vehicle very heavy or stuck
 *          - Grass or obstacles may prevent successful flip
 *          - Requires practice to master flip technique
 * 
 * @note Only available when MODE_TURTLE_ENABLED is defined at compile time
 * @note Requires bidirectional ESCs (BLHeli_S, BLHeli_32) configured properly
 * @warning Practice flip technique in safe area before operational use
 * @warning Ensure props won't strike ground during flip attempt
 * @warning Motor reversal can be disorienting - start with low throttle
 */
class ModeTurtle : public Mode {

public:
    // inherit constructors
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::TURTLE (28)
     */
    Number mode_number() const override { return Number::TURTLE; }

    /**
     * @brief Initialize Turtle mode
     * @param[in] ignore_checks If true, skips pre-arm checks (used for emergency mode changes)
     * @return true if initialization successful, false if not inverted or ESCs not compatible
     * 
     * @details Initialization:
     *          - Checks that vehicle is inverted (upside down)
     *          - Verifies ESCs support bidirectional operation
     *          - Reverses motor directions for inverted thrust
     *          - Automatically arms motors for immediate use
     *          - Initializes throttle warning timer
     *          - Clears shutdown flag
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for Turtle mode
     * 
     * @details Executes turtle flip control:
     *          - Reads pilot throttle, roll, and pitch inputs
     *          - Scales inputs for reversed motor operation
     *          - Commands motors with reversed direction
     *          - Monitors for successful flip (vehicle upright)
     *          - Issues periodic warnings if motors running without input
     *          - Handles motor output through custom output_to_motors()
     */
    void run() override;
    
    /**
     * @brief Exit Turtle mode
     * 
     * @details Cleanup when leaving Turtle mode:
     *          - Restores normal motor directions (un-reverses)
     *          - Disarms motors (safety measure)
     *          - Clears motor outputs
     *          - Synchronizes direction change with semaphore
     *          - Sets shutdown flag to prevent further output
     */
    void exit() override;

    /**
     * @brief Check if mode requires GPS
     * @return false - Turtle mode operates purely on IMU, no GPS needed
     */
    bool requires_GPS() const override { return false; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return true - pilot directly controls throttle for flip force
     */
    bool has_manual_throttle() const override { return true; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method (transmitter, GCS, scripting, etc.)
     * @return true only if vehicle is inverted (prevents misuse when upright)
     * 
     * @details Arms automatically when entering mode if inverted. This check
     *          prevents accidental arming in Turtle mode when vehicle is upright.
     */
    bool allows_arming(AP_Arming::Method method) const override;
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return false - Turtle mode requires manual pilot input for flip
     */
    bool is_autopilot() const override { return false; }
    
    /**
     * @brief Change motor rotation direction
     * @param[in] reverse If true, reverse motor directions; if false, restore normal
     * 
     * @details Changes motor direction for all motors:
     *          - reverse=true: Inverts motor rotation for upside-down thrust
     *          - reverse=false: Restores normal motor rotation
     *          - Protected by semaphore to prevent conflicts
     *          - Communicates direction change to ESCs via motor library
     */
    void change_motor_direction(bool reverse);
    
    /**
     * @brief Output motor commands (overridden for Turtle mode)
     * 
     * @details Custom motor output handling:
     *          - Uses reversed motor directions
     *          - Applies motor_output and motors_input directly
     *          - Bypasses normal motor mixing
     *          - Protected by semaphore during output
     *          - Checks shutdown flag before outputting
     */
    void output_to_motors() override;
    
    /**
     * @brief Check if mode allows entry during RC failsafe
     * @return false - cannot enter Turtle mode during RC failsafe (requires pilot input)
     */
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:
    const char *name() const override { return "TURTLE"; }
    const char *name4() const override { return "TRTL"; }

private:
    /**
     * @brief Arm motors in Turtle mode
     * 
     * @details Arms the vehicle for Turtle operation:
     *          - Bypasses normal arming checks (vehicle is crashed/inverted)
     *          - Enables motor output with reversed directions
     *          - Sets armed flag for motor library
     *          - Protected by semaphore for thread safety
     */
    void arm_motors();
    
    /**
     * @brief Disarm motors when exiting Turtle mode
     * 
     * @details Safely disarms vehicle:
     *          - Stops all motor output
     *          - Clears armed flag
     *          - Prepares for mode transition
     *          - Protected by semaphore for thread safety
     */
    void disarm_motors();

    float motors_output;                        ///< Motor output throttle level (0.0 to 1.0)
    Vector2f motors_input;                      ///< Roll/pitch input for flip control (x=roll, y=pitch)
    uint32_t last_throttle_warning_output_ms;   ///< Time of last throttle warning message (milliseconds)

    HAL_Semaphore msem;                         ///< Semaphore protecting motor operations during arming/direction changes
    bool shutdown;                              ///< If true, prevent motor output (set during exit)
};
#endif

// modes below rely on Guided mode so must be declared at the end (instead of in alphabetical order)

#if AP_ADSB_AVOIDANCE_ENABLED
/**
 * @class ModeAvoidADSB
 * @brief ADS-B avoidance mode - automatic collision avoidance with manned aircraft
 * 
 * @details ADS-B avoidance mode inherits from Guided mode and provides automatic
 *          collision avoidance with manned aircraft broadcasting ADS-B position data:
 *          
 *          Purpose:
 *          - Avoid collisions with manned aircraft detected via ADS-B receiver
 *          - Automatic evasive maneuvers when intrusion detected
 *          - Compliance with airspace regulations requiring see-and-avoid
 *          - Enhance safety in shared airspace with general aviation
 *          
 *          How it works:
 *          1. Vehicle continuously monitors ADS-B receiver for aircraft positions
 *          2. Avoidance library calculates collision threats based on closure rate and distance
 *          3. When threat detected, mode automatically engages
 *          4. System calculates safe evasive velocity vector
 *          5. Commands vehicle to move away from threat
 *          6. Monitors threat until clear
 *          7. Returns to previous mode when threat resolved
 *          
 *          Avoidance strategy:
 *          - Calculates predicted closest point of approach (CPA)
 *          - Determines if collision imminent based on time to CPA and miss distance
 *          - Generates velocity vector perpendicular to threat approach
 *          - Prioritizes vertical separation if possible (descend/climb)
 *          - Maintains horizontal separation as secondary measure
 *          - Adjusts avoidance intensity based on threat severity
 *          
 *          Threat assessment:
 *          - Uses ADS-B position, velocity, and altitude data
 *          - Predicts future aircraft positions
 *          - Accounts for both aircraft velocities (closure rate)
 *          - Considers vertical and horizontal separation
 *          - Multiple threat levels: warning, advisory, corrective action
 *          
 *          Integration with Guided mode:
 *          - Inherits from ModeGuided for velocity control
 *          - Uses velocity controller from Guided mode
 *          - Commands avoidance velocities through set_velocity()
 *          - Leverages Guided mode's position hold and navigation
 *          
 *          Requirements:
 *          - ADS-B receiver connected to autopilot (typically via serial/MAVLink)
 *          - GPS lock on both vehicle and detected aircraft
 *          - Valid position estimates from EKF
 *          - Sufficient flight envelope for evasive maneuvers
 *          - Adequate battery for extended avoidance if needed
 *          
 *          Configuration parameters:
 *          - ADSB_ENABLE: Enable ADS-B avoidance system
 *          - AVD_ENABLE: Enable avoidance library
 *          - AVD_F_DIST_XY: Horizontal distance for evasion (meters)
 *          - AVD_F_DIST_Z: Vertical distance for evasion (meters)
 *          - AVD_F_TIME: Time horizon for collision prediction (seconds)
 *          
 *          Typical workflow:
 *          1. Vehicle flying in Auto, Guided, or other autonomous mode
 *          2. ADS-B receiver detects manned aircraft
 *          3. Avoidance library calculates threat level
 *          4. When threshold exceeded, automatically switches to AVOID_ADSB mode
 *          5. Vehicle executes evasive maneuver
 *          6. Monitors threat aircraft position
 *          7. When clear, returns to previous autonomous mode
 *          8. Mission/operation continues
 *          
 *          Evasive maneuver types:
 *          - Vertical avoidance: Descend or climb to increase altitude separation
 *          - Horizontal avoidance: Lateral movement perpendicular to threat vector
 *          - Combined: Both vertical and horizontal components
 *          - Hover: If no safe evasion vector, maintain position (least preferred)
 *          
 *          Safety features:
 *          - Automatic engagement (no pilot action required)
 *          - Multiple threat handling (prioritizes closest/most severe)
 *          - Battery level monitoring (limits evasion distance)
 *          - Geofence integration (stays within boundaries during evasion)
 *          - Failsafe compatibility (avoidance overrides in emergency)
 *          - Does not allow manual arming (autonomous only)
 *          
 *          ADS-B receiver types supported:
 *          - uAvionix PingRX/PingStation
 *          - uAvionix PingUSB
 *          - Sagetech MXS transponder
 *          - Any MAVLink-compatible ADS-B receiver
 *          
 *          Common use cases:
 *          - Commercial drone operations in controlled airspace
 *          - Beyond visual line of sight (BVLOS) operations
 *          - Operations near airports or flight paths
 *          - High altitude flights where manned traffic common
 *          - Regulatory compliance for airspace integration
 *          
 *          Limitations:
 *          - Only avoids aircraft broadcasting ADS-B (not all aircraft equipped)
 *          - Requires line-of-sight to aircraft's ADS-B transmitter
 *          - Cannot detect non-cooperative aircraft (gliders, ultralights, etc.)
 *          - Latency in ADS-B updates (typically 1-2 seconds)
 *          - Limited to macro-scale avoidance (tens to hundreds of meters)
 *          - Does not replace pilot responsibility for see-and-avoid
 * 
 * @note Only available when AP_ADSB_AVOIDANCE_ENABLED is defined
 * @note Requires ADS-B receiver hardware connected to autopilot
 * @warning ADS-B avoidance is supplemental - does not replace visual observation
 * @warning Not all aircraft broadcast ADS-B - cannot detect all traffic
 * @warning System latency means very close/fast encounters may not be avoided
 */
class ModeAvoidADSB : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::AVOID_ADSB (19)
     */
    Number mode_number() const override { return Number::AVOID_ADSB; }

    /**
     * @brief Initialize ADS-B avoidance mode
     * @param[in] ignore_checks If true, skips pre-arm checks
     * @return true if initialization successful
     * 
     * @details Initialization:
     *          - Calls parent ModeGuided::init()
     *          - Verifies ADS-B receiver is connected and functioning
     *          - Checks that avoidance library is enabled
     *          - Validates GPS position estimate
     *          - Prepares velocity controller for evasive maneuvers
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for ADS-B avoidance mode
     * 
     * @details Executes collision avoidance:
     *          - Queries avoidance library for current threats
     *          - Calculates optimal evasive velocity vector
     *          - Commands velocity through inherited Guided functionality
     *          - Monitors threat resolution
     *          - Logs avoidance events for post-flight analysis
     *          - Returns to previous mode when threat cleared
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return true - ADS-B avoidance requires GPS for position-based collision prediction
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - avoidance is fully automatic (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method
     * @return false - ADS-B avoidance is autonomous only, entered automatically
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return true - ADS-B avoidance is fully autonomous
     */
    bool is_autopilot() const override { return true; }

    /**
     * @brief Set avoidance velocity vector
     * @param[in] velocity_neu Desired velocity in North-East-Up frame (m/s)
     * @return true if velocity command accepted
     * 
     * @details Commands evasive velocity calculated by avoidance library:
     *          - Validates velocity is within vehicle capabilities
     *          - Passes through to parent ModeGuided velocity controller
     *          - Applies geofence limits if active
     *          - Used by avoidance library to command evasive maneuvers
     */
    bool set_velocity(const Vector3f& velocity_neu);

protected:

    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

private:

};
#endif  // AP_ADSB_AVOIDANCE_ENABLED

#if MODE_FOLLOW_ENABLED
/**
 * @class ModeFollow
 * @brief Follow mode - automatically follows another vehicle or ground station
 * 
 * @details Follow mode enables a vehicle to autonomously track and follow another vehicle
 *          or ground station broadcasting its position via MAVLink:
 *          
 *          Purpose:
 *          - Follow another vehicle maintaining specified offset distance
 *          - Track moving ground station (handheld or vehicle-mounted)
 *          - Formation flying with lead vehicle
 *          - Automated camera platform following subject
 *          - Search and rescue following ground team
 *          
 *          How it works:
 *          1. Target vehicle/station broadcasts position via MAVLink
 *          2. Vehicle receives target position updates
 *          3. Follow library calculates desired offset position
 *          4. Vehicle navigates to maintain offset from target
 *          5. Continuously updates as target moves
 *          6. Adjusts speed to match target velocity
 *          7. Maintains specified distance and bearing offset
 *          
 *          Follow offset configuration:
 *          - Distance: How far behind/away from target (meters)
 *          - Bearing: Angle relative to target heading (degrees)
 *          - Altitude: Height offset above/below target (meters)
 *          - Can follow directly behind, to side, above, or any combination
 *          
 *          Offset examples:
 *          - Distance=10m, Bearing=0°: Follow 10m directly behind target
 *          - Distance=5m, Bearing=90°: Follow 5m to target's right side
 *          - Distance=10m, Bearing=0°, Alt=+5m: Follow 10m behind and 5m above
 *          - Distance=0m: Fly directly over target position
 *          
 *          Integration with Guided mode:
 *          - Inherits from ModeGuided for position/velocity control
 *          - Uses Guided mode's waypoint navigation
 *          - Leverages position hold when target stationary
 *          - Applies velocity controller for moving target
 *          
 *          Target sources:
 *          - Another ArduPilot vehicle broadcasting GLOBAL_POSITION_INT
 *          - Ground station with GPS broadcasting position
 *          - Companion computer providing position via MAVLink
 *          - Any MAVLink system ID configured as follow target
 *          
 *          Position update handling:
 *          - Receives target position via MAVLink messages
 *          - Calculates target velocity from position history
 *          - Predicts target future position based on velocity
 *          - Commands vehicle to offset position + predicted movement
 *          - Updates continuously as new positions received
 *          
 *          Speed adaptation:
 *          - Automatically matches target speed
 *          - Accelerates/decelerates to maintain distance
 *          - Limits maximum follow speed to vehicle capabilities
 *          - Slows when approaching desired offset distance
 *          - Hovers when target stopped
 *          
 *          Requirements:
 *          - GPS lock on following vehicle (for position control)
 *          - Target broadcasting position via MAVLink
 *          - Radio link to receive target position updates
 *          - Sufficient update rate (at least 1Hz, preferably 5Hz+)
 *          - Valid EKF position estimate
 *          
 *          Configuration parameters:
 *          - FOLL_ENABLE: Enable follow library
 *          - FOLL_SYSID: MAVLink system ID of target to follow
 *          - FOLL_DIST_MAX: Maximum follow distance (meters)
 *          - FOLL_OFS_X: Offset forward/back from target (meters)
 *          - FOLL_OFS_Y: Offset left/right from target (meters)
 *          - FOLL_OFS_Z: Offset up/down from target (meters)
 *          - FOLL_ALT_TYPE: Altitude frame (absolute, relative, terrain)
 *          - FOLL_YAW_BEHAVE: Yaw behavior (face target, same heading, etc.)
 *          - FOLL_POS_P: Position controller P gain
 *          
 *          Typical workflow:
 *          1. Configure follow parameters (target ID, offsets)
 *          2. Ensure target is broadcasting position
 *          3. Switch to Follow mode
 *          4. Vehicle navigates to offset position behind target
 *          5. Maintains offset as target moves
 *          6. Adjusts speed and heading automatically
 *          7. Continue following until mode changed
 *          
 *          Yaw behavior options:
 *          - Face target: Always point toward target vehicle/station
 *          - Same heading: Match target's heading
 *          - Look ahead: Face direction of travel
 *          - Manual control: Pilot controls yaw with stick
 *          
 *          Safety features:
 *          - Maximum follow distance limit (won't follow too far)
 *          - Timeout if target position not updated (switches to loiter)
 *          - Geofence integration (stays within boundaries)
 *          - Battery failsafe (returns home if battery low)
 *          - Does not allow manual arming (autonomous mode)
 *          - Requires GPS for position control
 *          
 *          Lost target handling:
 *          - If no position updates received for timeout period
 *          - Vehicle loiters at last known offset position
 *          - Waits for target to reappear
 *          - Can be configured to return home instead
 *          - Logs warning messages about lost target
 *          
 *          Common use cases:
 *          - Aerial photography following moving subject
 *          - Search and rescue following ground team
 *          - Formation flight for shows/demonstrations
 *          - Vehicle inspection (follow asset vehicle)
 *          - Wildlife tracking following tagged animals
 *          - Autonomous follow-me for personal video
 *          
 *          Camera control integration:
 *          - Can command gimbal to point at target
 *          - Maintains target in camera frame
 *          - Adjusts offset for optimal viewing angle
 *          - Coordinates with mount controller
 *          
 *          Limitations:
 *          - Requires continuous MAVLink position updates
 *          - Limited by radio range for receiving updates
 *          - Cannot predict non-linear target motion
 *          - May lag behind fast-moving targets
 *          - Update latency affects responsiveness
 *          - Target must broadcast compatible position messages
 * 
 * @note Only available when MODE_FOLLOW_ENABLED is defined
 * @note Requires follow library (AP_Follow) to be enabled
 * @warning Ensure adequate separation distance to avoid collision with target
 * @warning Monitor battery level - following can extend flight time unpredictably
 * @warning Lost radio link means lost target position updates
 */
class ModeFollow : public ModeGuided {

public:

    // inherit constructor
    using ModeGuided::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::FOLLOW (23)
     */
    Number mode_number() const override { return Number::FOLLOW; }

    /**
     * @brief Initialize Follow mode
     * @param[in] ignore_checks If true, skips pre-arm checks
     * @return true if initialization successful
     * 
     * @details Initialization:
     *          - Calls parent ModeGuided::init()
     *          - Verifies follow library is enabled and configured
     *          - Checks that target system ID is set
     *          - Validates GPS position estimate
     *          - Queries follow library for initial target position
     *          - Sets up position controller for following
     *          - Initializes velocity logging timer
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Exit Follow mode
     * 
     * @details Cleanup when leaving Follow mode:
     *          - Stops following behavior
     *          - Clears target tracking
     *          - Calls parent ModeGuided::exit()
     */
    void exit() override;
    
    /**
     * @brief Main loop for Follow mode
     * 
     * @details Executes follow behavior:
     *          - Queries follow library for target position and velocity
     *          - Calculates desired offset position from target
     *          - Accounts for target velocity in position calculation
     *          - Commands vehicle to follow position via Guided controller
     *          - Adjusts yaw based on configured behavior
     *          - Logs desired velocity periodically for telemetry
     *          - Handles timeout if target position not updated
     */
    void run() override;

    /**
     * @brief Check if mode requires GPS
     * @return true - Follow mode requires GPS for position-based navigation
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - follow is fully automatic (altitude controller)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method
     * @return false - Follow mode is autonomous, entered automatically
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return true - Follow mode is fully autonomous
     */
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "FOLLOW"; }
    const char *name4() const override { return "FOLL"; }

    /**
     * @brief Get current waypoint/target location
     * @param[out] loc Target location (follow offset position)
     * @return true if valid target location available
     * 
     * @details Returns the calculated follow position (target position + offset)
     *          for reporting to ground station. Used for displaying follow target
     *          on map and telemetry.
     */
    bool get_wp(Location &loc) const override;
    
    /**
     * @brief Get distance to target waypoint
     * @return Distance to follow offset position in meters
     * 
     * @details Returns horizontal distance from vehicle to desired follow position.
     *          Used for telemetry and determining if follow position reached.
     */
    float  wp_distance_m() const override;
    
    /**
     * @brief Get bearing to target waypoint
     * @return Bearing to follow offset position in degrees (0-360)
     * 
     * @details Returns compass bearing from vehicle to desired follow position.
     *          Used for telemetry and navigation display.
     */
    float wp_bearing_deg() const override;

    uint32_t last_log_ms;   ///< System time of last desired velocity log message (milliseconds)
};
#endif

/**
 * @class ModeZigZag
 * @brief ZigZag mode - automated path following in zigzag pattern for aerial surveying
 * 
 * @details ZigZag mode enables efficient surveying, mapping, and agricultural spraying by
 *          flying predefined zigzag patterns between manually-defined boundary points:
 *          
 *          Purpose:
 *          - Aerial surveying and mapping with camera
 *          - Agricultural spraying in straight parallel lines
 *          - Search and rescue grid patterns
 *          - Area coverage missions
 *          - Crop monitoring with consistent overlap
 *          
 *          Operation concept:
 *          1. Pilot manually flies to corner A, saves position
 *          2. Pilot manually flies to corner B, saves position (defines AB line)
 *          3. Vehicle automatically flies between A and B
 *          4. At end of line, vehicle moves sideways by configured distance
 *          5. Vehicle flies back in opposite direction
 *          6. Repeats zigzag pattern until complete or stopped
 *          
 *          Operating modes:
 *          - Manual point storage: Pilot saves A and B manually, then toggles to fly
 *          - Semi-automatic: Pilot controls when to switch lines
 *          - Full automatic: Vehicle completes entire pattern autonomously
 *          
 *          Point definition workflow:
 *          1. Enter ZigZag mode (vehicle in STORING_POINTS state)
 *          2. Pilot flies vehicle to first corner
 *          3. Pilot saves point A via switch/button
 *          4. Pilot flies vehicle to second corner (defines line direction)
 *          5. Pilot saves point B via switch/button
 *          6. AB line now defined - ready for automatic flight
 *          
 *          Automatic flight pattern:
 *          ```
 *          A -----------> B    Line 1 (fly forward)
 *                         |    (move sideways)
 *          A <----------- B    Line 2 (fly backward)
 *          |                   (move sideways)
 *          A -----------> B    Line 3 (fly forward)
 *                         |    (move sideways)
 *          A <----------- B    Line 4 (fly backward)
 *          ...continues until line_num reached or stopped
 *          ```
 *          
 *          Waypoint behavior:
 *          - Flies straight line from current A/B to opposite point
 *          - Uses waypoint controller (AC_WPNav) for smooth navigation
 *          - Maintains altitude throughout pattern
 *          - Adjusts speed based on configured waypoint speed
 *          - Waits configured delay at end of each line
 *          
 *          Sideways movement:
 *          - After reaching end of line, moves perpendicular to AB
 *          - Distance configurable via ZIGZ_SIDE_DIST parameter
 *          - Direction configurable (left or right from line)
 *          - Movement parallel to original AB line
 *          - Creates new virtual A' and B' points offset from original
 *          
 *          Direction control (ZIGZ_DIRECTION):
 *          - Forward: Move forward perpendicular to AB after each line
 *          - Right: Move right perpendicular to AB after each line
 *          - Backward: Move backward perpendicular to AB after each line
 *          - Left: Move left perpendicular to AB after each line
 *          
 *          Configuration parameters:
 *          - ZIGZ_AUTO: Enable automatic zigzag (vs manual switching)
 *          - ZIGZ_SIDE_DIST: Distance to move sideways between lines (meters)
 *          - ZIGZ_DIRECTION: Direction to move (forward/right/backward/left)
 *          - ZIGZ_LINE_NUM: Number of lines to fly in auto mode
 *          - ZIGZ_WP_DELAY: Delay at end of each line (seconds)
 *          - ZIGZ_SPRAY: Enable automatic sprayer control
 *          
 *          RC switch control:
 *          - Switch to ZigZag mode → Manual control, save points
 *          - Switch to Auto position → Begin automatic zigzag flight
 *          - Switch back to manual → Regain manual control, resume later
 *          - Button press → Save A or B point
 *          
 *          Manual control phases:
 *          1. STORING_POINTS: Saving A and B, full manual control
 *          2. MANUAL_REGAIN: Pilot took control mid-pattern, can resume
 *          
 *          Automatic control phases:
 *          1. AB_MOVING: Flying between A and B (or B and A)
 *          2. SIDEWAYS: Moving perpendicular between lines
 *          
 *          Sprayer integration (if ZIGZ_SPRAY enabled):
 *          - Automatically turns on sprayer when flying AB lines
 *          - Turns off sprayer during sideways movement
 *          - Turns off sprayer at end of pattern
 *          - Coordinates with AP_Sprayer library
 *          - Ensures consistent spray coverage
 *          
 *          Waypoint delay behavior:
 *          - At end of each line, vehicle holds position
 *          - Waits ZIGZ_WP_DELAY seconds before moving sideways
 *          - Allows time for photos, spray settling, etc.
 *          - Timer starts when reaching destination
 *          - Timer stored in reach_wp_time_ms
 *          
 *          Resume capability:
 *          - If pilot takes manual control mid-pattern
 *          - Current position and line number saved
 *          - When automatic mode re-entered, continues from saved position
 *          - Can resume to same line or next line
 *          - Stored in current_dest and line_count
 *          
 *          Suspend/Resume workflow:
 *          - Pilot switches to manual during auto flight
 *          - Vehicle stops and enters MANUAL_REGAIN state
 *          - Pilot can adjust position, avoid obstacles
 *          - Pilot switches back to auto
 *          - Vehicle resumes from current position to next waypoint
 *          
 *          Altitude handling:
 *          - Maintains altitude from when A/B points saved
 *          - Can use terrain following if enabled
 *          - Altitude frame stored with A/B points
 *          - current_terr_alt flag indicates terrain altitude mode
 *          
 *          Line counting:
 *          - line_count: Current line number (0-based)
 *          - line_num: Target line number (from ZIGZ_LINE_NUM parameter)
 *          - Auto mode completes when line_count >= line_num
 *          - After completion, vehicle loiters at final position
 *          
 *          Safety features:
 *          - Can always switch to manual control
 *          - GPS required for position control
 *          - Geofence integration (stays within boundaries)
 *          - Battery failsafe (RTL if low battery)
 *          - Allows user takeoff before starting pattern
 *          
 *          Common applications:
 *          - Agricultural spraying (crops, vineyards)
 *          - Aerial photography (mapping, surveys)
 *          - Search and rescue grid patterns
 *          - Crop health monitoring
 *          - Precision agriculture
 *          - Archaeological surveys
 *          
 *          Typical workflow:
 *          1. Switch to ZigZag mode
 *          2. Fly manually to field corner A, press save button
 *          3. Fly manually to opposite corner B, press save button
 *          4. Switch RC to auto position
 *          5. Vehicle flies A→B→sideways→B→A→sideways... pattern
 *          6. After configured number of lines, vehicle loiters
 *          7. Switch to different mode (RTL, Loiter, etc.)
 *          
 *          Advantages over mission waypoints:
 *          - Faster setup (only 2 points to define)
 *          - No ground station required for basic patterns
 *          - Easy to adjust during flight
 *          - Can pause and resume easily
 *          - Efficient for rectangular areas
 *          
 *          Limitations:
 *          - Only rectangular/parallel patterns
 *          - Cannot handle complex polygon boundaries
 *          - Requires manual point definition in field
 *          - Fixed sideways distance (no overlap adjustment)
 *          - All lines parallel to initial AB line
 * 
 * @note Requires GPS for position-based navigation
 * @note Useful for aerial surveying, mapping, and agricultural operations
 * @warning Ensure adequate clearance for sideways movements
 * @warning Monitor pattern completion to avoid overflying area
 */
class ModeZigZag : public Mode {        

public:
    /**
     * @brief Constructor for ZigZag mode
     * 
     * @details Initializes ZigZag-specific parameters and state variables.
     */
    ModeZigZag(void);

    // Inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::ZIGZAG (24)
     */
    Number mode_number() const override { return Number::ZIGZAG; }

    /**
     * @enum Destination
     * @brief Identifies which waypoint (A or B) is being referenced
     */
    enum class Destination : uint8_t {
        A,  ///< Destination A (first saved corner point)
        B,  ///< Destination B (second saved corner point, defines AB line)
    };

    /**
     * @enum Direction
     * @brief Direction to move sideways between lines
     * 
     * @details Direction is relative to vehicle heading when AB line was defined.
     *          Controls which way to offset for next parallel line.
     */
    enum class Direction : uint8_t {
        FORWARD,        ///< Move forward from the yaw direction (ahead of vehicle)
        RIGHT,          ///< Move right from the yaw direction (right perpendicular)
        BACKWARD,       ///< Move backward from the yaw direction (behind vehicle)
        LEFT,           ///< Move left from the yaw direction (left perpendicular)
    } zigzag_direction;  ///< Current configured sideways movement direction

    /**
     * @brief Initialize ZigZag mode
     * @param[in] ignore_checks If true, skips pre-arm checks
     * @return true if initialization successful
     * 
     * @details Initialization:
     *          - Calls parent Mode::init()
     *          - Initializes state to STORING_POINTS (manual control)
     *          - Resets waypoint and line counters
     *          - Clears any previous pattern data
     *          - Verifies GPS position estimate
     *          - Sets up waypoint controller
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Exit ZigZag mode
     * 
     * @details Cleanup when leaving ZigZag mode:
     *          - Stops automatic flight if active
     *          - Turns off sprayer if enabled
     *          - Saves current state for potential resume
     *          - Clears active destination
     */
    void exit() override;
    
    /**
     * @brief Main loop for ZigZag mode
     * 
     * @details Dispatches to appropriate handler based on current state:
     *          - STORING_POINTS or MANUAL_REGAIN: manual_control() - pilot flies
     *          - AUTO: auto_control() - automatic zigzag pattern
     */
    void run() override;

    /**
     * @brief Run automatic zigzag pattern
     * 
     * @details Executes automatic flight behavior:
     *          - AB_MOVING state: Flies between A and B waypoints
     *          - SIDEWAYS state: Moves perpendicular to AB line
     *          - Checks for waypoint delay timeout
     *          - Advances to next line when current line complete
     *          - Stops when all lines completed (line_count >= line_num)
     *          - Controls sprayer activation during line flight
     */
    void run_auto();
    
    /**
     * @brief Suspend automatic zigzag flight
     * 
     * @details Called when pilot takes manual control during auto flight:
     *          - Saves current destination for resume
     *          - Saves current line count
     *          - Sets is_suspended flag
     *          - Stops current waypoint navigation
     */
    void suspend_auto();
    
    /**
     * @brief Initialize automatic zigzag pattern
     * 
     * @details Sets up automatic flight:
     *          - Verifies A and B points are defined
     *          - Calculates initial destination
     *          - Resets line counter if starting fresh
     *          - Initializes state to AB_MOVING
     *          - Starts waypoint navigation to first destination
     */
    void init_auto();

    /**
     * @brief Check if mode requires GPS
     * @return true - ZigZag mode requires GPS for waypoint navigation
     */
    bool requires_GPS() const override { return true; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - ZigZag uses altitude controller
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method
     * @return true - can arm in ZigZag mode (for manual point definition)
     */
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    
    /**
     * @brief Check if mode is autopilot-controlled
     * @return true - ZigZag has autonomous flight capability
     */
    bool is_autopilot() const override { return true; }
    
    /**
     * @brief Check if mode has user takeoff capability
     * @param[in] must_navigate True if takeoff must include navigation
     * @return true - allows user-initiated takeoff
     */
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    /**
     * @brief Save current position as A or B, or move to specified destination
     * @param[in] ab_dest Which destination (A or B) to save or navigate to
     * 
     * @details Behavior depends on current state:
     *          - If A not saved: Saves current position as A
     *          - If A saved but not B: Saves current position as B
     *          - If both A and B saved: Navigates to specified destination
     *          - Stores position in NEU frame (cm) relative to EKF origin
     */
    void save_or_move_to_destination(Destination ab_dest);

    /**
     * @brief Return manual control to pilot
     * @param[in] maintain_target If true, maintains current position target
     * 
     * @details Transitions from automatic to manual control:
     *          - Stops automatic flight
     *          - Sets state to MANUAL_REGAIN
     *          - Turns off sprayer if active
     *          - Maintains position if maintain_target true, else pilot controls
     *          - Saves state for potential resume
     */
    void return_to_manual_control(bool maintain_target);

    static const struct AP_Param::GroupInfo var_info[];  ///< Parameter definitions

protected:

    const char *name() const override { return "ZIGZAG"; }
    const char *name4() const override { return "ZIGZ"; }
    
    /**
     * @brief Get distance to current waypoint
     * @return Distance to current A or B destination in meters
     * 
     * @details Returns horizontal distance from vehicle to current target waypoint
     *          (either A, B, or sideways position). Used for telemetry and determining
     *          when destination reached.
     */
    float wp_distance_m() const override;
    
    /**
     * @brief Get bearing to current waypoint
     * @return Bearing to current destination in degrees (0-360)
     * 
     * @details Returns compass bearing from vehicle to current target waypoint.
     *          Used for telemetry and navigation display.
     */
    float wp_bearing_deg() const override;
    
    /**
     * @brief Get crosstrack error from line
     * @return Distance from desired track line in meters (positive = right)
     * 
     * @details Returns perpendicular distance from desired AB line path.
     *          Used for monitoring track following accuracy during line flight.
     */
    float crosstrack_error() const override;

private:

    /**
     * @brief Execute automatic control logic
     * 
     * @details Main automatic zigzag pattern controller:
     *          - AB_MOVING: Navigates between A and B using waypoint controller
     *          - SIDEWAYS: Moves perpendicular to AB line for next pass
     *          - Manages waypoint delays at end of lines
     *          - Controls sprayer activation during line flight
     *          - Advances line counter after each completed line
     *          - Stops when target line count reached
     */
    void auto_control();
    
    /**
     * @brief Execute manual control logic
     * 
     * @details Manual flight mode allowing pilot to:
     *          - Fly vehicle freely with stick inputs
     *          - Save A and B positions when commanded
     *          - Adjust position before starting automatic pattern
     *          - Maintain position/altitude while setting up
     */
    void manual_control();
    
    /**
     * @brief Check if vehicle reached current destination
     * @return true if within acceptance radius of destination
     * 
     * @details Determines waypoint arrival by checking:
     *          - Horizontal distance to destination
     *          - Waypoint controller completion status
     *          - Sets reach_wp_time_ms when first reached
     */
    bool reached_destination();
    
    /**
     * @brief Calculate next destination based on A or B
     * @param[in] ab_dest Which destination to calculate (A or B)
     * @param[in] use_wpnav_alt If true, use waypoint navigation altitude
     * @param[out] next_dest Calculated destination vector (NED frame, cm)
     * @param[out] terrain_alt True if using terrain altitude reference
     * @return true if calculation successful
     * 
     * @details Calculates destination accounting for:
     *          - Current line offset (how many lines completed)
     *          - Sideways distance for this line number
     *          - Direction perpendicular to AB line
     *          - Altitude from original A/B or current altitude
     */
    bool calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const;
    
    /**
     * @brief Control sprayer on/off state
     * @param[in] b true to turn sprayer on, false to turn off
     * 
     * @details Interfaces with AP_Sprayer library to control spray system.
     *          Only active if HAL_SPRAYER_ENABLED and _spray_enabled parameter set.
     */
    void spray(bool b);
    
    /**
     * @brief Calculate sideways destination for next line
     * @param[out] next_dest Calculated sideways position (NED frame, cm)
     * @param[out] terrain_alt True if using terrain altitude reference
     * @return true if calculation successful
     * 
     * @details Calculates position perpendicular to AB line for moving between
     *          parallel passes. Uses _side_dist_m and _direction parameters.
     */
    bool calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const;
    
    /**
     * @brief Command vehicle to move to sideways position
     * 
     * @details Executes perpendicular movement between lines:
     *          - Calculates sideways destination
     *          - Commands waypoint controller to navigate
     *          - Turns off sprayer during sideways movement
     *          - Transitions to SIDEWAYS state
     */
    void move_to_side();

    Vector2f dest_A_ne_cm;    ///< Point A position in NE plane (cm) relative to EKF origin
    Vector2f dest_B_ne_cm;    ///< Point B position in NE plane (cm) relative to EKF origin
    Vector3f current_dest;    ///< Current target destination NED (cm), used for resume after suspend
    bool current_terr_alt;    ///< True if current_dest uses terrain altitude reference

    // Configuration parameters
    AP_Int8  _auto_enabled;    ///< Enable automatic zigzag pattern (vs manual line switching)
#if HAL_SPRAYER_ENABLED
    AP_Int8  _spray_enabled;   ///< Enable automatic sprayer control during line flight
#endif
    AP_Int8  _wp_delay_s;      ///< Delay at end of each line before moving sideways (seconds)
    AP_Float _side_dist_m;     ///< Distance to move sideways between parallel lines (meters)
    AP_Int8  _direction;       ///< Sideways movement direction (forward/right/backward/left)
    AP_Int16 _line_num;        ///< Total number of lines to fly in automatic mode

    /**
     * @enum ZigZagState
     * @brief High-level zigzag mode state
     */
    enum ZigZagState {
        STORING_POINTS, ///< Manual control: pilot saving A and B points
        AUTO,           ///< Automatic flight: vehicle flying zigzag pattern autonomously
        MANUAL_REGAIN   ///< Manual control: pilot regained control, can resume automatic
    } stage;           ///< Current zigzag mode state

    /**
     * @enum AutoState
     * @brief Automatic flight sub-state
     */
    enum AutoState {
        MANUAL,         ///< Not in automatic mode (manual control active)
        AB_MOVING,      ///< Flying between A and B (or B and A) waypoints
        SIDEWAYS,       ///< Moving perpendicular to AB for next parallel line
    } auto_stage;      ///< Current automatic flight state

    uint32_t reach_wp_time_ms = 0;  ///< System time when destination reached (ms), or zero if not reached
    Destination ab_dest_stored;     ///< Saved destination (A or B) for resume after manual control
    bool is_auto;                   ///< True if automatic zigzag pattern enabled (vs manual switching)
    uint16_t line_count = 0;        ///< Current line number (0-based, increments after each line)
    int16_t line_num = 0;           ///< Target line count from parameter (pattern stops when reached)
    bool is_suspended;              ///< True if automatic flight suspended by pilot
};

#if MODE_AUTOROTATE_ENABLED
/**
 * @class ModeAutorotate
 * @brief Autorotate mode - autonomous emergency landing for helicopters with engine failure
 * 
 * @details Autorotate mode implements emergency autorotation procedures for helicopters
 *          experiencing engine/motor failure, enabling safe powered-off landing:
 *          
 *          Purpose:
 *          - Emergency response to engine/motor failure on helicopters
 *          - Autonomous execution of autorotation emergency procedure
 *          - Maintain rotor RPM through descent for controlled landing
 *          - Perform final flare to reduce descent rate before touchdown
 *          - Maximize survivability of emergency landing
 *          
 *          What is autorotation:
 *          Autorotation is an emergency procedure where a helicopter descends
 *          with engine power off, using airflow through the rotor system to
 *          maintain rotor RPM. The kinetic energy stored in the rotating blades
 *          is used at the end to flare and reduce descent rate for landing.
 *          
 *          Physical principle:
 *          - During descent, upward airflow through rotor keeps blades spinning
 *          - Collective is lowered to maintain rotor RPM in efficient range
 *          - Forward speed provides additional lift and rotor energy
 *          - At final moment, collective raised (flare) to convert rotor
 *            kinetic energy into lift, dramatically slowing descent
 *          - After flare, rotor slows down as energy depleted
 *          
 *          Autorotation phases:
 *          1. ENTRY: Initial response, establish descent and rotor RPM
 *          2. GLIDE: Steady descent maintaining rotor RPM and forward speed
 *          3. FLARE: Pitch up to reduce descent rate using rotor energy
 *          4. TOUCH_DOWN: Final collective application and touchdown
 *          5. LANDED: Vehicle on ground, disarm
 *          
 *          Mode trigger conditions:
 *          - Engine/motor RPM drops below threshold
 *          - Power loss detected
 *          - Pilot manually enters mode (training)
 *          - Advanced failsafe triggers autorotation
 *          - Critical ESC/motor failure detected
 *          
 *          Entry phase (ENTRY):
 *          - Lower collective immediately to maintain rotor RPM
 *          - Establish forward airspeed for efficient autorotation
 *          - Stabilize attitude (slight nose down)
 *          - Monitor rotor RPM recovery
 *          - Transition to glide when rotor RPM stabilized
 *          - Duration typically 2-4 seconds
 *          
 *          Glide phase (GLIDE):
 *          - Maintain steady descent rate for maximum range
 *          - Control collective to keep rotor RPM in safe range
 *          - Maintain forward airspeed for lift and rotor energy
 *          - Navigate toward landing area if GPS available
 *          - Monitor altitude for flare initiation
 *          - Adjust heading if wind compensation enabled
 *          
 *          Target descent parameters:
 *          - Rotor RPM: Maintain in optimal range (typically 90-110% of normal)
 *          - Descent rate: 500-1500 cm/s depending on helicopter
 *          - Forward speed: 10-20 m/s for optimal glide ratio
 *          - Attitude: 5-15° nose down for forward flight
 *          
 *          Flare phase (FLARE):
 *          - Initiated at configured altitude AGL (typically 8-15 meters)
 *          - Pitch up aggressively to slow descent and forward speed
 *          - Convert forward speed and rotor energy into lift
 *          - Target near-zero descent rate at touchdown height
 *          - Rotor RPM decreases as energy extracted for lift
 *          - Critical timing - too early wastes energy, too late high impact
 *          
 *          Flare execution:
 *          - Rapid pitch-up to configured angle (typically 20-40°)
 *          - Collective held low initially, raised progressively
 *          - Monitors vertical velocity reduction
 *          - Transitions to touchdown phase when near ground
 *          - Duration typically 1-3 seconds
 *          
 *          Touchdown phase (TOUCH_DOWN):
 *          - Final collective increase to cushion landing
 *          - Level attitude for flat landing
 *          - Minimize lateral drift
 *          - Apply full collective if rotor RPM sufficient
 *          - Detect ground contact via landing detector
 *          
 *          Landed phase (LANDED):
 *          - Vehicle on ground, autorotation complete
 *          - Motors disarmed
 *          - Collective to minimum
 *          - Log completion
 *          - Wait for pilot intervention
 *          
 *          Rotor RPM management:
 *          - Critical parameter throughout autorotation
 *          - Too low: insufficient energy for flare (hard landing)
 *          - Too high: excessive stress, potential blade stall
 *          - Entry: Lower collective to prevent RPM decay
 *          - Glide: Modulate collective to maintain RPM in range
 *          - Flare: RPM naturally decreases as energy extracted
 *          - Monitored continuously via RPM sensor
 *          
 *          Attitude control:
 *          - Entry: Establish nose-down attitude for forward flight
 *          - Glide: Maintain attitude for optimal descent angle
 *          - Flare: Aggressive pitch-up to maximum flare angle
 *          - Touchdown: Level attitude for landing
 *          - Uses attitude controller throughout
 *          
 *          Position control (if GPS available):
 *          - Entry: Note current position
 *          - Glide: Can navigate toward landing area
 *          - Limited lateral maneuvering (energy conservation)
 *          - Prioritizes rotor RPM over precise positioning
 *          - GPS not required - mode works without navigation
 *          
 *          Configuration parameters:
 *          - AROT_ENABLE: Enable autorotation mode
 *          - AROT_RAMP_TIME: Entry phase duration (seconds)
 *          - AROT_HS_PCT: Target head speed percentage during glide
 *          - AROT_BAIL_TIME: Bailout time if rotor RPM too low (seconds)
 *          - AROT_FLARE_HGT: Altitude to initiate flare (meters)
 *          - AROT_FLARE_ANG: Maximum flare pitch angle (degrees)
 *          - AROT_LAND_HGT: Height to begin touchdown phase (meters)
 *          - AROT_COL_MIN: Minimum collective for glide phase
 *          
 *          Sensor requirements:
 *          - RPM sensor: Critical for monitoring rotor speed
 *          - Rangefinder or barometer: For altitude determination
 *          - IMU: For attitude and acceleration
 *          - GPS: Optional, for position awareness
 *          - Preferably rangefinder for accurate height AGL
 *          
 *          Safety features:
 *          - Bail-out detection: If rotor RPM too low, abort to cushion landing
 *          - Altitude validation: Uses both barometer and rangefinder
 *          - Automatic motor disarm after landing
 *          - Continuous logging for post-flight analysis
 *          - Conservative timing to avoid premature flare
 *          
 *          Limitations and considerations:
 *          - Helicopter-specific: Only for FRAME_CONFIG == HELI_FRAME
 *          - Requires proper configuration for specific helicopter
 *          - Flare timing critical - depends on helicopter characteristics
 *          - Wind affects descent rate and landing spot
 *          - Forward speed needed - minimal hover autorotation capability
 *          - No go-around possible once initiated
 *          - Success depends on sufficient altitude for entry
 *          
 *          Training considerations:
 *          - Should be practiced at altitude with safety pilot
 *          - Can be manually triggered for training
 *          - Monitor rotor RPM throughout procedure
 *          - Verify parameters tuned for specific helicopter
 *          - Practice bail-out procedures
 *          
 *          Typical sequence:
 *          1. Engine failure detected or mode manually entered
 *          2. ENTRY: Collective lowered, forward flight established (2-3 sec)
 *          3. GLIDE: Steady descent at optimal parameters (altitude dependent)
 *          4. FLARE: Initiated at 10m AGL, aggressive pitch-up (1-2 sec)
 *          5. TOUCH_DOWN: Final collective, cushion landing (0.5-1 sec)
 *          6. LANDED: On ground, disarmed
 *          
 *          Emergency bail-out conditions:
 *          - Rotor RPM drops below minimum threshold
 *          - Time in entry exceeds bail-out time without RPM recovery
 *          - Immediately applies full collective cushion
 *          - Attempts softest possible landing with remaining rotor energy
 *          - Likely harder landing than successful autorotation
 *          
 *          Logging:
 *          - Autorotation-specific log messages (AROT)
 *          - Records phase transitions
 *          - Logs rotor RPM, altitude, descent rate
 *          - Critical for post-flight analysis
 *          - Enables tuning of parameters
 *          
 *          Comparison to normal landing:
 *          - Much faster descent rate (500-1500 vs 50-200 cm/s)
 *          - Higher forward speed during descent
 *          - Aggressive flare maneuver
 *          - Motor completely off (vs controlled throttle)
 *          - Landing impact harder (emergency procedure)
 *          
 *          Success factors:
 *          - Sufficient altitude for entry and glide
 *          - Proper collective management for rotor RPM
 *          - Accurate flare altitude determination
 *          - Well-tuned parameters for helicopter type
 *          - Favorable landing area
 *          - Manageable wind conditions
 * 
 * @note Only available when MODE_AUTOROTATE_ENABLED is defined
 * @note Only applicable to helicopters (FRAME_CONFIG == HELI_FRAME)
 * @note Requires rotor RPM sensor for safe operation
 * @note Requires careful tuning for specific helicopter characteristics
 * @warning This is an EMERGENCY procedure - high risk of hard landing
 * @warning Proper training and parameter tuning essential before use
 * @warning Success depends on sufficient altitude and proper execution
 * @warning Flare timing critical - improper timing can result in crash
 */
class ModeAutorotate : public Mode {

public:

    // inherit constructor
    using Mode::Mode;
    
    /**
     * @brief Get flight mode number
     * @return Number::AUTOROTATE (26)
     */
    Number mode_number() const override { return Number::AUTOROTATE; }

    /**
     * @brief Initialize Autorotate mode
     * @param[in] ignore_checks If true, skips pre-arm checks
     * @return true if initialization successful
     * 
     * @details Initialization:
     *          - Validates autorotation is enabled and configured
     *          - Checks rotor RPM sensor availability
     *          - Initializes phase to ENTRY_INIT
     *          - Records entry time
     *          - Sets up attitude controller for entry
     *          - Configures collective for RPM recovery
     *          - Logs autorotation initiation
     */
    bool init(bool ignore_checks) override;
    
    /**
     * @brief Main loop for Autorotate mode
     * 
     * @details Executes appropriate behavior based on current phase:
     *          - ENTRY_INIT/ENTRY: Establish descent and rotor RPM
     *          - GLIDE_INIT/GLIDE: Maintain steady descent
     *          - FLARE_INIT/FLARE: Pitch up and reduce descent rate
     *          - TOUCH_DOWN_INIT/TOUCH_DOWN: Final cushion and land
     *          - LANDED_INIT/LANDED: Post-landing, disarm
     *          
     *          Phase transitions based on:
     *          - Time elapsed (entry phase)
     *          - Altitude AGL (flare initiation)
     *          - Rotor RPM status
     *          - Ground contact detection
     *          
     *          Continuous monitoring:
     *          - Rotor RPM vs target range
     *          - Vertical velocity vs target
     *          - Altitude above ground
     *          - Forward airspeed
     *          
     *          Controls outputs:
     *          - Collective: Manages rotor RPM and descent rate
     *          - Cyclic: Controls attitude and forward speed
     *          - Motors: Off (autorotation condition)
     *          
     *          Logging at reduced rate for post-flight analysis
     */
    void run() override;

    /**
     * @brief Check if mode is autopilot-controlled
     * @return true - Autorotate is fully autonomous emergency procedure
     */
    bool is_autopilot() const override { return true; }
    
    /**
     * @brief Check if mode requires GPS
     * @return false - autorotation works without GPS (though GPS helpful for position awareness)
     */
    bool requires_GPS() const override { return false; }
    
    /**
     * @brief Check if mode has manual throttle control
     * @return false - motor off during autorotation (emergency procedure)
     */
    bool has_manual_throttle() const override { return false; }
    
    /**
     * @brief Check if mode allows arming
     * @param[in] method Arming method
     * @return false - cannot arm in autorotate (emergency mode only)
     */
    bool allows_arming(AP_Arming::Method method) const override { return false; };

    static const struct AP_Param::GroupInfo  var_info[];  ///< Parameter definitions

protected:

    const char *name() const override { return "AUTOROTATE"; }
    const char *name4() const override { return "AROT"; }

private:

    uint32_t _entry_time_start_ms;  ///< System time when entry phase started (ms), used for timing transition to glide
    uint32_t _last_logged_ms;       ///< System time of last autorotation log message (ms), for rate-limiting log output

    /**
     * @enum Phase
     * @brief Autorotation phase state machine
     * 
     * @details Phases follow emergency autorotation procedure:
     *          INIT phases set up parameters for subsequent execution phase
     */
    enum class Phase {
        ENTRY_INIT,      ///< Initialize entry phase, set up for RPM recovery
        ENTRY,           ///< Establish descent and recover/maintain rotor RPM
        GLIDE_INIT,      ///< Initialize glide phase, set steady descent parameters
        GLIDE,           ///< Maintain steady descent with optimal rotor RPM
        FLARE_INIT,      ///< Initialize flare, prepare for pitch-up maneuver
        FLARE,           ///< Execute flare: pitch up, convert rotor energy to lift
        TOUCH_DOWN_INIT, ///< Initialize touchdown, prepare for ground contact
        TOUCH_DOWN,      ///< Final descent and landing cushion
        LANDED_INIT,     ///< Initialize landed state, prepare for shutdown
        LANDED,          ///< On ground, autorotation complete, disarmed
    } current_phase;    ///< Current autorotation phase

};
#endif
