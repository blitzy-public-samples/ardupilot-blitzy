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
//  CENTRAL ENGINEERING DECISION (AAP 0.6.2) -- AP_AHRS decoupling:
//    AP_L1_Control binds a concrete `AP_AHRS &_ahrs` and the six accessors it
//    reads (get_location, groundspeed_vector, get_yaw_rad, get_pitch_rad,
//    get_EAS2TAS, yaw_sensor) are NON-VIRTUAL, so a plain subclass can neither
//    override them nor (their backing state being private) set them. AAP 0.6.2
//    describes two behavior-preserving ways to meet this state dependency:
//      * Option A -- compose the controller against a genuine AP_AHRS and have
//        the host drive that real AHRS through an external navigation source.
//        Injecting host state into a real AP_AHRS requires an external-state
//        backend (the EKF/DCM stack or AP_ExternalAHRS), which is OUT OF SCOPE
//        for this extraction, so Option A is NOT used here.
//      * Option B (IMPLEMENTED -- the single realisation this service uses): a
//        COMPILE-TIME INCLUDE SEAM (supplied by the build; see CMakeLists.txt /
//        the seam-enabled wscript) resolves the token `AP_AHRS` to
//        AfsimL1_AHRS_Shim, so AP_L1_Control's `_ahrs.<method>` call sites bind
//        to the shim's identically named methods and the controller reads the
//        host-injected state directly, with no EKF/DCM stack linked.
//    This facade therefore ALWAYS composes the controller against the owned
//    `_ahrs_shim` -- the SAME object set_state_ne() writes -- so host-injected
//    position/velocity/yaw/pitch always reach the guidance controller. That
//    binding is only well-typed when the include seam has made `AP_AHRS` denote
//    AfsimL1_AHRS_Shim, so the service REQUIRES the seam: the guard just below
//    emits a clear #error when AFSIML1_L1_USES_SHIM_AHRS is not defined, so no
//    build can EVER silently compose the controller against a different,
//    never-written state source (the state-injection landmine AAP 0.6.2 warns
//    of). The construction of `_l1` (+ nullptr TECS) lives in
//    AfsimL1Behavior.cpp; this header declares `_ahrs_shim` before `_l1` so the
//    member-initializer list is well-defined.
//
//  CONSTRAINTS (AAP 0.7.1):
//    - Do NOT modify AP_L1_Control or re-declare its methods here -- only
//      compose it. The additive, default-off `set_update_dt(float)` timing seam
//      was already added to AP_L1_Control by the parent-folder change; this
//      facade merely relies on it from AfsimL1Behavior.cpp.
//    - No vehicle-firmware dependency; no new third-party dependency.
//    - Exactly the three includes below (all additive; AAP 0.4.2).
// ============================================================================

// ---------------------------------------------------------------------------
// Seam requirement guard (AAP 0.6.2 Option B; see CENTRAL ENGINEERING DECISION).
// The facade composes AP_L1_Control against the owned AfsimL1_AHRS_Shim (the
// object set_state_ne() writes). That is only well-typed when the build's
// compile-time include seam has made the token `AP_AHRS` denote
// AfsimL1_AHRS_Shim, which every supported build signals by defining
// AFSIML1_L1_USES_SHIM_AHRS. This guard is placed BEFORE the includes so it
// fires FIRST -- with a clear directive -- rather than letting the real
// <AP_AHRS/AP_AHRS.h> stack (pulled transitively by AP_L1_Control.h without the
// seam) error out with unrelated messages. Failing loudly here is what prevents
// a build from silently composing the controller against a real, never-written
// AP_AHRS (the critical state-injection defect this design forbids).
// ---------------------------------------------------------------------------
#if !defined(AFSIML1_L1_USES_SHIM_AHRS)
#error "AfsimL1Behavior requires the AP_AHRS -> AfsimL1_AHRS_Shim compile-time include seam: define AFSIML1_L1_USES_SHIM_AHRS and place the seam directory first on the include path. Build the service via the provided CMakeLists.txt (standalone libafsim_l1.so) or the seam-enabled wscript. See README.md and AAP 0.6.2 Option B."
#endif

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
/// Ownership / lifetime: the class owns its AfsimL1_AHRS_Shim plus its
/// AP_L1_Control by value. Because AP_L1_Control declares CLASS_NO_COPY and has
/// no default constructor, AfsimL1Behavior is (correctly) non-copyable and
/// provides a user-defined default constructor -- defined in
/// AfsimL1Behavior.cpp -- that builds the controller against the owned
/// `_ahrs_shim` (the object set_state_ne() writes) via a member-initializer
/// list.
class AfsimL1Behavior {
public:
    /// Construct the service instance.
    ///
    /// Defined in AfsimL1Behavior.cpp because AP_L1_Control has no default
    /// constructor: the definition uses a member-initializer list to build
    /// `_l1` against the owned `_ahrs_shim` (the object set_state_ne() writes,
    /// which the build's include seam has made the `AP_AHRS` type) and a nullptr
    /// TECS. The stock L1 tuning (PERIOD, DAMPING, XTRACK_I, LIM_BANK) comes from
    /// the controller constructor's AP_Param defaults; init() re-asserts PERIOD.
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
    // declaration order, and the member-initializer list in AfsimL1Behavior.cpp
    // builds `_l1` from `_ahrs_shim`, so `_ahrs_shim` MUST precede `_l1`
    // (-Werror=reorder enforces this).

    /// Adapter presenting the AP_AHRS read surface from host-injected state.
    /// set_state_ne() pushes the host-injected position, velocity, yaw and pitch
    /// into this shim, and the controller is composed against it (see the
    /// constructor in AfsimL1Behavior.cpp), so it is BOTH the single canonical
    /// record of the injected state AND the AHRS the controller reads. Under the
    /// build's compile-time include seam (AAP 0.6.2 Option B) the token `AP_AHRS`
    /// denotes this type, which is what makes the `_l1(_ahrs_shim, nullptr)`
    /// binding well-typed.
    AfsimL1_AHRS_Shim _ahrs_shim;

    /// Composed L1 guidance controller. Constructed in AfsimL1Behavior.cpp
    /// against `_ahrs_shim` (the object set_state_ne() writes) and a nullptr
    /// TECS; never copied (AP_L1_Control is CLASS_NO_COPY). Binding the
    /// controller to the same shim that receives the injected state is the fix
    /// for the state-injection contract: host-supplied position/velocity/yaw/
    /// pitch are exactly what the guidance reads.
    AP_L1_Control _l1;

    /// Current leg start point (North/East), default-constructed to a zeroed
    /// Location and (re)seeded by init() / set_leg_ne().
    Location _prev{};

    /// Current leg end point (North/East), default-constructed to a zeroed
    /// Location and (re)seeded by init() / set_leg_ne().
    Location _next{};
};
