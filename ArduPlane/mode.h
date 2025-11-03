/**
 * @file mode.h
 * @brief Flight mode base class and mode subclass definitions for ArduPlane
 * 
 * @details This file defines the Mode base class and all flight mode subclasses
 *          for the ArduPlane fixed-wing autopilot. Each mode implements specific
 *          control logic for different flight scenarios:
 *          
 *          - **Manual modes**: Direct pilot control (MANUAL, TRAINING, ACRO)
 *          - **Stabilized modes**: Attitude stabilization with pilot input (STABILIZE, FLY_BY_WIRE_A/B, CRUISE)
 *          - **Auto modes**: Autonomous navigation (AUTO, RTL, LOITER, GUIDED)
 *          - **QuadPlane modes**: VTOL operations (QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO, QAUTOTUNE)
 *          - **Special modes**: AUTOTUNE, TAKEOFF, THERMAL, AVOID_ADSB, AUTOLAND
 *          
 *          All modes inherit from the abstract Mode base class and implement the
 *          mode interface for lifecycle management, control execution, and navigation.
 * 
 * @note Mode instances are singletons accessed via plane.mode_* pointers
 * @note Mode transitions triggered by plane.set_mode() with ModeReason enum
 * 
 * Source: ArduPlane/mode.h:1-1039
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include "quadplane.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>
#include "config.h"
#include "pullup.h"
#include "systemid.h"

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED HAL_QUADPLANE_ENABLED
#endif

#ifndef MODE_AUTOLAND_ENABLED
#define MODE_AUTOLAND_ENABLED 1
#endif

#include <AP_Quicktune/AP_Quicktune.h>

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;

/**
 * @class Mode
 * @brief Base class for all ArduPlane flight modes
 * 
 * @details Abstract base class defining the flight mode interface. All modes inherit
 *          from this class and implement the following key methods:
 *          
 *          **Lifecycle Methods**:
 *          - enter() - Called when entering the mode, performs initialization
 *          - exit() - Called when leaving the mode, performs cleanup
 *          - _enter() - Subclass override for mode-specific entry logic
 *          - _exit() - Subclass override for mode-specific exit logic
 *          
 *          **Control Execution Methods**:
 *          - run() - Main control loop executed at main loop rate (typically 400Hz)
 *          - update() - Update navigation targets at nav rate (10-50Hz)
 *          - navigate() - Execute navigation logic for auto modes
 *          
 *          **Mode Identification**:
 *          - mode_number() - Returns unique mode number from Number enum
 *          - name() - Returns full mode name string
 *          - name4() - Returns 4-character abbreviated mode name
 *          
 *          **Mode Capabilities** (query methods):
 *          - is_vtol_mode() - True for QuadPlane VTOL modes
 *          - is_vtol_man_throttle() - True for manual throttle VTOL modes
 *          - is_vtol_man_mode() - True for manual VTOL modes
 *          - is_guided_mode() - True for GUIDED and AVOID_ADSB modes
 *          - is_landing() - True when mode is executing landing sequence
 *          - is_taking_off() - True when mode is executing takeoff sequence
 *          - allows_throttle_nudging() - True if pilot can adjust throttle in auto modes
 *          - allows_stick_mixing() - True if pilot inputs mixed with auto navigation
 *          - allows_terrain_disable() - True if terrain following can be disabled
 *          - does_auto_navigation() - True if mode controls horizontal navigation
 *          - does_auto_throttle() - True if mode controls throttle automatically
 *          - does_automatic_thermal_switch() - True if mode supports automatic thermal detection
 *          - mode_allows_autotuning() - True if mode supports autotune via switch
 *          
 *          **Common Functionality Provided**:
 *          - Pilot input processing (throttle, roll, pitch, yaw)
 *          - Rudder and steering output helpers
 *          - Altitude management and target altitude updates
 *          - Stick mixing for manual override in auto modes
 *          - Controller reset on mode entry
 *          - Pre-arm safety checks
 *          
 * @note Modes are singletons accessed via plane.mode_* pointers (e.g., plane.mode_manual, plane.mode_auto)
 * @note Mode transitions are triggered by plane.set_mode() which calls exit() on old mode and enter() on new mode
 * @note Some modes are conditionally compiled (QuadPlane Q-modes require HAL_QUADPLANE_ENABLED)
 * @note Mode number 30 is reserved for offboard/Lua script control
 * 
 * @warning Mode implementations must be thread-safe for scheduler callbacks
 * @warning Do not perform long-running operations in run() or update() methods
 * 
 * Source: ArduPlane/mode.h:29-206
 */
class Mode
{
public:

    /* Do not allow copies */
    CLASS_NO_COPY(Mode);

    /**
     * @enum Number
     * @brief Flight mode identifiers used for mode selection and switching
     * 
     * @details Defines unique numeric identifiers for all ArduPlane flight modes.
     *          Mode numbers are used for:
     *          - Mode parameter storage and persistence
     *          - MAVLink mode reporting to ground control stations
     *          - Flight mode switches and RC channel mode selection
     *          - Mode transition logic and validation
     *          
     *          **Mode Categories**:
     *          
     *          **Manual Modes** (0-4): Direct or minimally stabilized pilot control
     *          - MANUAL (0): Direct RC pass-through, no stabilization
     *          - CIRCLE (1): Automated circling around a point
     *          - STABILIZE (2): Attitude stabilization with pilot input
     *          - TRAINING (3): Manual control with automatic recovery from extreme attitudes
     *          - ACRO (4): Rate control for aerobatic flight
     *          
     *          **Fly-By-Wire Modes** (5-7): Computer-assisted manual flight
     *          - FLY_BY_WIRE_A (5): Roll/pitch limits, automatic altitude hold
     *          - FLY_BY_WIRE_B (6): Altitude and airspeed hold with pilot heading control
     *          - CRUISE (7): Heading and altitude hold with ground speed control
     *          
     *          **Autonomous Modes** (8-15): Automated navigation and control
     *          - AUTOTUNE (8): Automated PID tuning flight
     *          - AUTO (10): Waypoint navigation mission execution
     *          - RTL (11): Return to launch with automatic landing
     *          - LOITER (12): Loiter at current location with configurable radius
     *          - TAKEOFF (13): Automated takeoff sequence
     *          - AVOID_ADSB (14): ADS-B collision avoidance mode
     *          - GUIDED (15): External control via GCS or companion computer
     *          - INITIALISING (16): Startup initialization mode (cannot be selected)
     *          
     *          **QuadPlane VTOL Modes** (17-23): Vertical takeoff and landing modes
     *          - QSTABILIZE (17): QuadPlane attitude stabilization (manual throttle)
     *          - QHOVER (18): QuadPlane altitude hold (VTOL hover)
     *          - QLOITER (19): QuadPlane position hold (GPS loiter)
     *          - QLAND (20): QuadPlane vertical landing
     *          - QRTL (21): QuadPlane return to launch with VTOL landing
     *          - QAUTOTUNE (22): QuadPlane auto-tuning (conditional)
     *          - QACRO (23): QuadPlane rate control for aerobatics
     *          
     *          **Advanced/Special Modes** (24-26):
     *          - THERMAL (24): Soaring/thermal exploitation mode
     *          - LOITER_ALT_QLAND (25): Loiter with descent to VTOL landing
     *          - AUTOLAND (26): Automated landing approach and flare
     *          
     * @note Mode number 9 is skipped for backwards compatibility
     * @note Mode number 30 is reserved for offboard/Lua script control
     * @note QuadPlane modes (17-23, 25) only available when HAL_QUADPLANE_ENABLED
     * @note QAUTOTUNE (22) additionally requires QAUTOTUNE_ENABLED
     * @note THERMAL (24) requires HAL_SOARING_ENABLED
     * @note AVOID_ADSB (14) requires HAL_ADSB_ENABLED
     * @note AUTOLAND (26) requires MODE_AUTOLAND_ENABLED (default enabled)
     * 
     * @warning Do not renumber existing modes - this breaks parameter compatibility
     * @warning Ground station software depends on these mode numbers for telemetry
     * 
     * Source: ArduPlane/mode.h:38-75
     */
    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
#endif
#if MODE_AUTOLAND_ENABLED
        AUTOLAND      = 26,
#endif

    // Mode number 30 reserved for "offboard" for external/lua control.
    };

    // Constructor
    Mode();

    /**
     * @brief Enter this flight mode
     * 
     * @details Called when transitioning into this mode. Performs common initialization:
     *          - Resets rate, steering, and TECS controllers via reset_controllers()
     *          - Calls subclass _enter() method for mode-specific initialization
     *          - Sets up initial control targets and navigation state
     *          
     *          This method handles the mode entry lifecycle and delegates mode-specific
     *          setup to the virtual _enter() method that subclasses override.
     * 
     * @return Always returns true (mode entry always succeeds)
     * 
     * @note Called by plane.set_mode() during mode transitions
     * @note Always call reset_controllers() to ensure clean state
     * 
     * Source: ArduPlane/mode.h:81
     */
    bool enter();

    /**
     * @brief Exit this flight mode
     * 
     * @details Called when transitioning out of this mode. Performs cleanup:
     *          - Calls subclass _exit() method for mode-specific cleanup
     *          - Stops mode-specific timers or state machines
     *          - Resets mode-specific flags
     * 
     * @note Called by plane.set_mode() before entering new mode
     * @note Subclasses override _exit() for mode-specific cleanup
     * 
     * Source: ArduPlane/mode.h:84
     */
    void exit();

    /**
     * @brief Main control loop execution for this mode
     * 
     * @details Called at main loop rate (typically 400Hz) to execute mode-specific
     *          control logic. Default implementation calls stabilize() for basic
     *          attitude control. Modes override this to implement custom control:
     *          - Manual modes: Direct RC input to servo outputs
     *          - Stabilized modes: Attitude stabilization with pilot input
     *          - Auto modes: Execute navigation controllers and motor mixing
     *          
     *          This is the highest-frequency method in the mode interface and should
     *          execute quickly to maintain real-time control performance.
     * 
     * @note Called at main loop rate (typically 400Hz on most flight controllers)
     * @note Keep execution time minimal to avoid scheduler overruns
     * @note Default implementation calls stabilize() for basic attitude control
     * 
     * @warning Must complete within scheduler time budget (~2.5ms at 400Hz)
     * @warning Avoid blocking operations or long computations
     * 
     * Source: ArduPlane/mode.h:87
     */
    virtual void run();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    bool pre_arm_checks(size_t buflen, char *buffer) const;

    // Reset rate and steering and TECS controllers
    void reset_controllers();

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    /**
     * @brief Update navigation targets and high-level control
     * 
     * @details Pure virtual method called at navigation rate (typically 10-50Hz) to:
     *          - Convert user input (RC channels) to control targets
     *          - Update navigation waypoints and paths
     *          - Set attitude and throttle targets
     *          - Update mode-specific state machines
     *          - Process altitude, heading, and position targets
     *          
     *          This method implements the high-level control logic for each mode,
     *          translating pilot commands or autonomous navigation goals into
     *          concrete control targets that run() executes.
     *          
     *          **Mode-Specific Implementations**:
     *          - Manual modes: Process RC inputs to direct control outputs
     *          - Stabilized modes: Convert RC inputs to attitude targets
     *          - Auto modes: Update navigation targets from mission or guidance
     *          - VTOL modes: Manage quadplane position and velocity targets
     * 
     * @note Called at navigation rate (10-50Hz depending on scheduler config)
     * @note Slower than run() - use for target updates, not real-time control
     * @note Must be implemented by all concrete mode subclasses
     * 
     * @warning Do not perform high-frequency control in update() - use run() instead
     * 
     * Source: ArduPlane/mode.h:109
     */
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }
    
    // true if the mode supports autotuning (via switch for modes other
    // that AUTOTUNE itself
    virtual bool mode_allows_autotuning() const { return false; }

    // method for mode specific target altitude profiles
    virtual void update_target_altitude();

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

    // true if is landing 
    virtual bool is_landing() const { return false; }

    // true if is taking 
    virtual bool is_taking_off() const;

    // true if throttle min/max limits should be applied
    virtual bool use_throttle_limits() const;

    // true if voltage correction should be applied to throttle
    virtual bool use_battery_compensation() const;
 
#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    virtual bool allows_autoland_direction_capture() const { return false; }
#endif

#if AP_QUICKTUNE_ENABLED
    // does this mode support VTOL quicktune?
    virtual bool supports_quicktune() const { return false; }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    virtual bool supports_systemid() const { return false; }
#endif
    
protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // mode specific pre-arm checks
    virtual bool _pre_arm_checks(size_t buflen, char *buffer) const;

    // Helper to output to both k_rudder and k_steering servo functions
    void output_rudder_and_steering(float val);

    // Output pilot throttle, this is used in stabilized modes without auto throttle control
    void output_pilot_throttle();

    // makes the initialiser list in the constructor manageable
    uint8_t unused_integer;

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif
    AP_AHRS& ahrs;
};


/**
 * @class ModeAcro
 * @brief ACRO mode - Rate control for aerobatic flight
 * 
 * @details Provides direct rate control on all three axes (roll, pitch, yaw) for
 *          aerobatic maneuvers. Pilot inputs command angular rates rather than
 *          attitudes. Supports both rate control and quaternion-based attitude
 *          locking when sticks are centered.
 * 
 * Source: ArduPlane/mode.h:209-247
 */
class ModeAcro : public Mode
{
friend class ModeQAcro;
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    void stabilize();

    void stabilize_quaternion();

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

protected:

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
        Quaternion q;
        bool roll_active_last;
        bool pitch_active_last;
        bool yaw_active_last;
    } acro_state;

    bool _enter() override;
};

/**
 * @class ModeAuto
 * @brief AUTO mode - Waypoint navigation and mission execution
 * 
 * @details Executes autonomous missions defined by waypoint lists stored in AP_Mission.
 *          Supports full MAVLink mission command set including navigation commands,
 *          camera control, conditional logic, and DO commands. Handles automatic
 *          transitions between waypoints, altitude changes, and landing sequences.
 * 
 * Source: ArduPlane/mode.h:249-315
 */
class ModeAuto : public Mode
{
public:
    friend class Plane;

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;
    
    bool mode_allows_autotuning() const override { return true; }

    bool is_landing() const override;

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    bool verify_altitude_wait(const AP_Mission::Mission_Command& cmd);

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    bool in_pullup() const { return pullup.in_pullup(); }
#endif

protected:

    bool _enter() override;
    void _exit() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override;

private:

    // Delay the next navigation command
    struct {
        uint32_t time_max_ms;
        uint32_t time_start_ms;
    } nav_delay;

    // wiggle state and timer for NAV_ALTITUDE_WAIT
    void wiggle_servos();
    struct {
        uint8_t stage;
        uint32_t last_ms;
    } wiggle;

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    GliderPullup pullup;
#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
};


/**
 * @class ModeAutoTune
 * @brief AUTOTUNE mode - Automated PID tuning
 * 
 * @details Automatically tunes roll, pitch, and yaw PID controllers by commanding
 *          test maneuvers and measuring vehicle response. Uses frequency domain
 *          analysis to determine optimal PID gains for stable, responsive control.
 * 
 * Source: ArduPlane/mode.h:318-341
 */
class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
    
protected:

    bool _enter() override;
};

/**
 * @class ModeGuided
 * @brief GUIDED mode - External control via GCS or companion computer
 * 
 * @details Allows external systems to command navigation targets via MAVLink.
 *          Supports target location, heading, altitude, and airspeed commands.
 *          Used for dynamic mission updates, follow-me, and companion computer control.
 * 
 * Source: ArduPlane/mode.h:343-386
 */
class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // handle a guided airspeed command, typically from companion computer
    bool handle_change_airspeed(const float airspeed, const float acceleration);
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

    void set_radius_and_direction(const float radius, const bool direction_is_ccw);

    void update_target_altitude() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return true; }
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif

private:
    float active_radius_m;
};

/**
 * @class ModeCircle
 * @brief CIRCLE mode - Automated circling around a point
 * 
 * @details Flies in a circle around a fixed point at configurable radius.
 *          Useful for aerial photography, surveillance, or maintaining position
 *          while awaiting further instructions. Radius controlled by CIRCLE_RADIUS parameter.
 * 
 * Source: ArduPlane/mode.h:388-406
 */
class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};

/**
 * @class ModeLoiter
 * @brief LOITER mode - Loiter at current or commanded location
 * 
 * @details Flies in circles around a fixed point maintaining altitude.
 *          Can loiter at current location on mode entry or at commanded waypoint.
 *          Supports altitude changes via throttle stick and heading alignment checks.
 * 
 * Source: ArduPlane/mode.h:408-440
 */
class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    bool allows_terrain_disable() const override { return true; }

    void update_target_altitude() override;
    
    bool mode_allows_autotuning() const override { return true; }

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
/**
 * @class ModeLoiterAltQLand
 * @brief LOITER_ALT_QLAND mode - Loiter with descent to VTOL landing
 * 
 * @details QuadPlane mode that loiters at a fixed location while descending.
 *          Automatically transitions to QLAND when altitude threshold is reached,
 *          enabling precise vertical landing after horizontal positioning.
 * 
 * Source: ArduPlane/mode.h:442-463
 */
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLAND"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

/**
 * @class ModeManual
 * @brief MANUAL mode - Direct RC pass-through without stabilization
 * 
 * @details Pure manual control with no computer assistance. RC inputs are passed
 *          directly to servo outputs with only trim and reversing applied. Requires
 *          experienced pilot skills. No attitude stabilization or limits enforced.
 * 
 * Source: ArduPlane/mode.h:465-489
 */
class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    // true if throttle min/max limits should be applied
    bool use_throttle_limits() const override;

    // true if voltage correction should be applied to throttle
    bool use_battery_compensation() const override { return false; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};


/**
 * @class ModeRTL
 * @brief RTL mode - Return to launch with automatic landing
 * 
 * @details Automatically navigates back to launch location or nearest rally point,
 *          climbs to RTL_ALTITUDE if needed, and executes landing sequence upon arrival.
 *          For QuadPlane, can automatically switch to QRTL for VTOL landing if within range.
 * 
 * Source: ArduPlane/mode.h:492-520
 */
class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL();
};

/**
 * @class ModeStabilize
 * @brief STABILIZE mode - Attitude stabilization with pilot input
 * 
 * @details Provides attitude stabilization (bank and pitch limits) while pilot
 *          controls the aircraft. Automatically levels wings and maintains pitch
 *          when sticks are centered. Good for learning or windy conditions.
 * 
 * Source: ArduPlane/mode.h:522-543
 */
class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

private:
    void stabilize_stick_mixing_direct();

};

/**
 * @class ModeTraining
 * @brief TRAINING mode - Manual control with automatic recovery
 * 
 * @details Manual mode with safety limits. Allows full manual control but prevents
 *          excessive bank angles and provides automatic recovery if aircraft exceeds
 *          safe attitudes. Excellent for pilot training with safety backup.
 * 
 * Source: ArduPlane/mode.h:545-562
 */
class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
};

/**
 * @class ModeInitializing
 * @brief INITIALISING mode - Startup initialization mode
 * 
 * @details Transient mode used during system startup before valid mode is set.
 *          Cannot be manually selected. Prevents arming. Should transition to
 *          valid mode quickly during boot sequence.
 * 
 * Source: ArduPlane/mode.h:564-584
 */
class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

};

/**
 * @class ModeFBWA
 * @brief FLY_BY_WIRE_A mode - Stabilized flight with roll/pitch limits
 * 
 * @details Computer-assisted manual flight with attitude stabilization and limits.
 *          Pilot controls desired roll angle and pitch angle directly. Autopilot
 *          maintains these attitudes and provides automatic altitude hold option.
 * 
 * Source: ArduPlane/mode.h:586-606
 */
class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};

/**
 * @class ModeFBWB
 * @brief FLY_BY_WIRE_B mode - Altitude and airspeed hold with manual heading
 * 
 * @details Advanced fly-by-wire mode that holds altitude and airspeed automatically
 *          using TECS controller. Pilot controls bank angle for heading changes.
 *          Supports terrain following and automatic thermal detection.
 * 
 * Source: ArduPlane/mode.h:608-632
 */
class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }
    
    bool mode_allows_autotuning() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;
};

/**
 * @class ModeCruise
 * @brief CRUISE mode - Heading and altitude hold with ground speed control
 * 
 * @details Maintains heading, altitude, and ground speed automatically. Pilot can
 *          adjust heading with aileron stick. Locks heading when stick is centered.
 *          Similar to FBWB but with heading lock instead of continuous heading changes.
 * 
 * Source: ArduPlane/mode.h:634-664
 */
class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
/**
 * @class ModeAvoidADSB
 * @brief AVOID_ADSB mode - ADS-B collision avoidance
 * 
 * @details Automatically maneuvers to avoid aircraft detected by ADS-B receiver.
 *          Calculates avoidance paths based on threat locations and velocities.
 *          Can be triggered manually or automatically when collision risk detected.
 * 
 * Source: ArduPlane/mode.h:666-688
 */
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
/**
 * @class ModeQStabilize
 * @brief QSTABILIZE mode - QuadPlane attitude stabilization
 * 
 * @details QuadPlane equivalent of STABILIZE mode. Provides VTOL attitude stabilization
 *          with manual throttle control. Pilot controls desired attitude angles while
 *          autopilot stabilizes. Manual throttle for altitude control.
 * 
 * Source: ArduPlane/mode.h:690-723
 */
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    bool supports_systemid() const override { return true; }
#endif
    
protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

/**
 * @class ModeQHover
 * @brief QHOVER mode - QuadPlane altitude hold (VTOL hover)
 * 
 * @details QuadPlane altitude hold mode. Automatically maintains altitude using
 *          vertical position controller. Pilot controls horizontal movement with
 *          roll/pitch sticks. Useful for hovering in place or slow speed maneuvering.
 * 
 * Source: ArduPlane/mode.h:725-752
 */
class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    bool supports_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

/**
 * @class ModeQLoiter
 * @brief QLOITER mode - QuadPlane position hold (GPS loiter)
 * 
 * @details QuadPlane GPS position hold. Automatically maintains horizontal position
 *          and altitude using GPS and position controllers. Responds to pilot stick
 *          inputs for position adjustments. Primary mode for precise VTOL hovering.
 * 
 * Source: ArduPlane/mode.h:754-787
 */
class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
friend class Plane;

public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support systemid?
    bool supports_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
    uint32_t last_target_loc_set_ms;

#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

/**
 * @class ModeQLand
 * @brief QLAND mode - QuadPlane vertical landing
 * 
 * @details Executes automated vertical landing sequence using VTOL motors.
 *          Descends vertically while maintaining horizontal position. Includes
 *          ground detection and automatic motor shutoff. Used for precision landings.
 * 
 * Source: ArduPlane/mode.h:789-807
 */
class ModeQLand : public Mode
{
public:
    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }
};

/**
 * @class ModeQRTL
 * @brief QRTL mode - QuadPlane return to launch with VTOL landing
 * 
 * @details QuadPlane return to launch. Climbs to safe altitude, navigates to home
 *          or rally point using fixed-wing or VTOL flight, then executes vertical
 *          landing. Automatically switches between forward flight and VTOL as appropriate.
 * 
 * Source: ArduPlane/mode.h:809-843
 */
class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override;

    bool allows_throttle_nudging() const override;

    float get_VTOL_return_radius() const;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    enum class SubMode {
        climb,
        RTL,
    } submode;
};

/**
 * @class ModeQAcro
 * @brief QACRO mode - QuadPlane rate control for aerobatics
 * 
 * @details QuadPlane rate control mode for VTOL aerobatic flight. Pilot inputs
 *          command angular rates on all three axes. Manual throttle control.
 *          Suitable for experienced pilots and aerobatic maneuvers in VTOL mode.
 * 
 * Source: ArduPlane/mode.h:845-865
 */
class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACRO"; }
    const char *name4() const override { return "QACO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
/**
 * @class ModeQAutotune
 * @brief QAUTOTUNE mode - QuadPlane auto-tuning
 * 
 * @details Automatically tunes QuadPlane attitude controller PID gains. Performs
 *          test maneuvers in VTOL mode and analyzes vehicle response to determine
 *          optimal PID values for stable, responsive VTOL flight.
 * 
 * Source: ArduPlane/mode.h:867-889
 */
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

/**
 * @class ModeTakeoff
 * @brief TAKEOFF mode - Automated takeoff sequence
 * 
 * @details Executes automated takeoff for conventional fixed-wing aircraft.
 *          Controls pitch attitude during ground roll, rotation, and initial climb.
 *          Transitions to level flight at target altitude. Configurable via parameters.
 * 
 * Source: ArduPlane/mode.h:893-939
 */
class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 target_alt;
    AP_Int16 level_alt;
    AP_Float ground_pitch;

protected:
    AP_Int16 target_dist;
    AP_Int8 level_pitch;

    bool takeoff_mode_setup;
    Location start_loc;

    bool _enter() override;

private:

    // flag that we have already called autoenable fences once in MODE TAKEOFF
    bool have_autoenabled_fences;

};
#if MODE_AUTOLAND_ENABLED
/**
 * @class ModeAutoLand
 * @brief AUTOLAND mode - Automated landing approach and flare
 * 
 * @details Executes automated landing sequence: climb to pattern altitude, loiter
 *          to align with landing direction, then execute approach and flare. Can
 *          capture landing direction from takeoff heading for consistent operations.
 * 
 * Source: ArduPlane/mode.h:940-1004
 */
class ModeAutoLand: public Mode
{
public:
    ModeAutoLand();

    Number mode_number() const override { return Number::AUTOLAND; }
    const char *name() const override { return "AUTOLAND"; }
    const char *name4() const override { return "ALND"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }
    
    bool is_landing() const override;
    
    void check_takeoff_direction(void);

    // return true when lined up correctly from the LOITER_TO_ALT
    bool landing_lined_up(void);

    // see if we should capture the direction
    void arm_check(void);

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 final_wp_alt;
    AP_Int16 final_wp_dist;
    AP_Int16 landing_dir_off;
    AP_Int8  options;
    AP_Int16 terrain_alt_min;

    // Bitfields of AUTOLAND_OPTIONS
    enum class AutoLandOption {
        AUTOLAND_DIR_ON_ARM     = (1U << 0), // set dir for autoland on arm if compass in use.
    };

    enum class AutoLandStage {
        CLIMB,
        LOITER,
        LANDING
    };

    bool autoland_option_is_set(AutoLandOption option) const {
        return (options & int8_t(option)) != 0;
    }

protected:
    bool _enter() override;
    AP_Mission::Mission_Command cmd_climb;
    AP_Mission::Mission_Command cmd_loiter;
    AP_Mission::Mission_Command cmd_land;
    Location land_start;
    AutoLandStage stage;
    void set_autoland_direction(const float heading);
};
#endif
#if HAL_SOARING_ENABLED

/**
 * @class ModeThermal
 * @brief THERMAL mode - Soaring/thermal exploitation
 * 
 * @details Automatically detects and exploits thermals for extended flight time.
 *          Uses variometer data to identify lift, then circles to stay within thermal.
 *          Exits when lift diminishes or altitude limits reached. Returns to previous mode.
 * 
 * Source: ArduPlane/mode.h:1005-1039
 */
class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "THERMAL"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif
