#pragma once

// ============================================================================
//  AfsimL1Behavior.h  --  Facade / Service layer for the AfsimL1 reusable L1
//                         guidance service
//                         (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  ROLE (AAP 0.3.3 -- Facade / Service Layer):
//    AfsimL1Behavior presents a small, task-oriented API over the richer
//    AP_L1_Control interface, hiding the guidance controller's internal wiring
//    behind set_leg_ne / set_state_ne / execute / get_roll_deg / get_lat_accel.
//    It COMPOSES (owns) an AP_L1_Control instance, the AfsimL1_AHRS_Shim
//    adapter, and the current prev/next legs, and re-plumbs the controller's
//    inputs (legs, state, timing) and outputs (roll, lateral acceleration)
//    through explicit dependency-injection seams so that an external host --
//    for example the AFSIM simulator -- can drive L1 guidance without the
//    ArduPilot vehicle flight loop.
//
//    This realises the SHAPE of the user "initialize a simple leg" example
//    (AAP 0.7.2), which is illustrative pseudocode, using concrete, valid
//    ArduPilot types.
//
//  BEHAVIOR-PRESERVING, ADDITIVE-ONLY (AAP 0.1.1, 0.7.1):
//    The L1 guidance mathematics inside AP_L1_Control are unchanged. This
//    facade performs no control-law, gain, or geometry computation of its own;
//    it only relocates the controller's input/output boundaries to injection
//    points, so the numerical guidance output is preserved.
//
//  CENTRAL ENGINEERING DECISION (AAP 0.6.2):
//    AP_L1_Control binds a concrete `AP_AHRS &_ahrs` and the six accessors it
//    reads (get_location, groundspeed_vector, get_yaw_rad, get_pitch_rad,
//    get_EAS2TAS, yaw_sensor) are NON-VIRTUAL, so a plain subclass cannot
//    override them. The recommended realisation (Option B) builds the
//    standalone shared library with a COMPILE-TIME INCLUDE SEAM so the L1
//    translation unit binds `_ahrs.<method>` to AfsimL1_AHRS_Shim's identically
//    named methods, avoiding the EKF/DCM stack; Option A (linking the real
//    AP_AHRS in external mode) is the max-fidelity fallback. EITHER WAY this
//    facade owns the shim and pushes injected state into it before delegating
//    to the controller. The construction of `_l1` against `_ahrs_shim`
//    (+ nullptr TECS) lives in AfsimL1Behavior.cpp; this header only declares
//    the members in the order required for that member-initializer list to be
//    well-defined.
//
//  CONSTRAINTS (AAP 0.7.1):
//    - Do NOT modify AP_L1_Control or re-declare its methods here -- only
//      compose it. The additive, default-off `set_update_dt(float)` timing seam
//      was already added to AP_L1_Control by the parent-folder change; this
//      facade merely relies on it from AfsimL1Behavior.cpp.
//    - No vehicle-firmware dependency; no new third-party dependency.
//    - Exactly the three includes below (all additive; AAP 0.4.2).
// ============================================================================

#include <AP_L1_Control/AP_L1_Control.h>   // AP_L1_Control -- the wrapped guidance controller
#include "AfsimL1_AHRS_Shim.h"             // AfsimL1_AHRS_Shim -- the injected-state AHRS adapter
#include <AP_Common/Location.h>            // Location -- the prev/next leg endpoints

/// @brief Reusable, host-driven Facade over AP_L1_Control's L1 lateral guidance.
///
/// AfsimL1Behavior consolidates the Position, Navigation and Timing (PNT)
/// couplings that the ArduPilot vehicle loop normally supplies to AP_L1_Control
/// into a single, embeddable service. The host pushes in the active leg
/// (set_leg_ne), the platform state (set_state_ne) and the control-step time
/// (execute's dt); the facade delegates to the composed controller and exposes
/// its outputs as plain scalars (get_roll_deg, get_lat_accel). The flat
/// extern "C" boundary in l1_c_api.* wraps this class so that an external host
/// compiled with a different toolchain can consume it as a shared library.
///
/// Ownership / lifetime: the class owns its AfsimL1_AHRS_Shim and AP_L1_Control
/// by value. Because AP_L1_Control declares CLASS_NO_COPY and has no default
/// constructor, AfsimL1Behavior is (correctly) non-copyable and provides a
/// user-defined default constructor -- defined in AfsimL1Behavior.cpp -- that
/// builds the controller against the owned shim via a member-initializer list.
class AfsimL1Behavior {
public:
    /// Construct the service instance.
    ///
    /// Defined in AfsimL1Behavior.cpp because AP_L1_Control has no default
    /// constructor: the definition uses a member-initializer list to build
    /// `_l1` against the owned `_ahrs_shim` (as the AHRS) and a nullptr TECS,
    /// and seeds the L1 tuning parameters (PERIOD, DAMPING, XTRACK_I, LIM_BANK).
    /// This is the entry point the C ABI's L1_Create() invokes through
    /// `new AfsimL1Behavior()`.
    AfsimL1Behavior();

    /// Seed a simple default leg: prev = (0, 0) N/E, next = (0, 500) N/E
    /// (metres), matching the user "initialize a simple leg" example. Safe to
    /// call repeatedly to reset the leg. Invoked by the C ABI's L1_Init().
    void init();

    /// Advance the guidance by one control step using a host-supplied dt.
    ///
    /// The host owns the timebase: @p dt_seconds is injected into the controller
    /// (via AP_L1_Control::set_update_dt) instead of the controller computing
    /// its own delta from the hardware clock. The current injected state is
    /// pushed into the AHRS shim, and then the active leg is evaluated with
    /// AP_L1_Control::update_waypoint(prev, next).
    ///
    /// @param dt_seconds  Control-step interval in seconds (e.g. 0.02 for 50 Hz).
    void execute(double dt_seconds);

    /// Set the active navigation leg from previous/next North/East offsets.
    ///
    /// Offsets are in metres relative to the service datum; the endpoints are
    /// synthesised as Location values by offsetting the datum (stock ArduPilot
    /// has no two-argument Location constructor).
    ///
    /// @param prevN  Previous waypoint North offset (metres).
    /// @param prevE  Previous waypoint East  offset (metres).
    /// @param nextN  Next     waypoint North offset (metres).
    /// @param nextE  Next     waypoint East  offset (metres).
    void set_leg_ne(double prevN, double prevE, double nextN, double nextE);

    /// Inject the current platform state consumed by the guidance controller.
    ///
    /// Forwards to the AHRS shim's setters so the controller observes the same
    /// values it would read from a live AP_AHRS. The argument order matches the
    /// user example (East velocity component before North).
    ///
    /// @param n          Position North offset from datum   (metres).
    /// @param e          Position East  offset from datum   (metres).
    /// @param velE       Ground velocity, East  component   (m/s).
    /// @param velN       Ground velocity, North component   (m/s).
    /// @param yaw_cd     Heading / yaw                       (centidegrees).
    /// @param pitch_rad  Pitch                               (radians).
    void set_state_ne(double n, double e, double velE, double velN, double yaw_cd, double pitch_rad);

    /// Roll command produced by the most recent execute(), in degrees.
    /// Equivalent to AP_L1_Control::nav_roll_cd() / 100 (centidegrees -> deg).
    /// const: this reads the controller's last-computed output only.
    double get_roll_deg() const;

    /// Lateral-acceleration demand produced by the most recent execute(), m/s^2.
    /// Equivalent to AP_L1_Control::lateral_acceleration().
    /// const: this reads the controller's last-computed output only.
    double get_lat_accel() const;

private:
    // Member declaration ORDER is significant: C++ initialises members in
    // declaration order, so `_ahrs_shim` MUST precede `_l1`. AfsimL1Behavior.cpp
    // constructs `_l1` with a reference to `_ahrs_shim`, which therefore has to
    // be fully constructed first.

    /// Adapter presenting the AP_AHRS read surface from host-injected state.
    AfsimL1_AHRS_Shim _ahrs_shim;

    /// Composed L1 guidance controller. Constructed in AfsimL1Behavior.cpp
    /// against `_ahrs_shim` (as the AHRS) and a nullptr TECS; never copied
    /// (AP_L1_Control is CLASS_NO_COPY).
    AP_L1_Control _l1;

    /// Current leg start point (North/East), default-constructed to a zeroed
    /// Location and (re)seeded by init() / set_leg_ne().
    Location _prev{};

    /// Current leg end point (North/East), default-constructed to a zeroed
    /// Location and (re)seeded by init() / set_leg_ne().
    Location _next{};
};
