// ============================================================================
//  AfsimL1Behavior.cpp  --  Facade / Service-layer implementation for the
//                           AfsimL1 reusable L1 guidance service
//                           (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  ROLE (AAP 0.3.3 -- Facade / Service Layer):
//    Implements the AfsimL1Behavior facade declared in AfsimL1Behavior.h. The
//    facade COMPOSES an AP_L1_Control guidance controller and an
//    AfsimL1_AHRS_Shim state adapter, and re-plumbs the controller's Position,
//    Navigation and Timing (PNT) inputs/outputs -- normally supplied by the
//    ArduPilot vehicle flight loop -- to explicit dependency-injection seams so
//    that an external host (for example the AFSIM simulator) can drive L1
//    guidance stand-alone.
//
//  BEHAVIOR-PRESERVING, ADDITIVE-ONLY (AAP 0.1.1, 0.7.1):
//    This translation unit performs NO control-law, gain, or geometry
//    computation of its own. Every guidance result is produced by the composed
//    AP_L1_Control; the facade only (a) seeds the stock L1 tuning so standalone
//    output matches the vehicle default, (b) forwards the host-injected leg,
//    state and timing into the controller/shim, and (c) reads the controller's
//    outputs back out. It therefore preserves the L1 guidance mathematics and
//    numerical output exactly, delegating solely to update_waypoint(),
//    nav_roll_cd(), lateral_acceleration() and the additive set_update_dt()
//    timing seam.
//
//  CENTRAL ENGINEERING DECISION (AAP 0.6.2):
//    AP_L1_Control binds a concrete `AP_AHRS &_ahrs` and the six accessors it
//    reads are NON-VIRTUAL, so a plain subclass can neither override them nor
//    (their backing state being private) set them. AAP 0.6.2 therefore offers
//    two behavior-preserving realisations, selected by the compile-time seam
//    macro AFSIML1_L1_USES_SHIM_AHRS and reflected in the member-initialiser
//    list below:
//      * Option A (RECOMMENDED default; macro UNDEFINED -- e.g. the in-tree waf
//        `use='ap'` build): compose the controller against a genuine AP_AHRS
//        (the owned `_ahrs` member) so every type stays a real ArduPilot type
//        end-to-end and the service links the full ArduPilot stack. This is the
//        maximum-fidelity path and is what the in-tree example compiles, so the
//        member-initialiser list passes `_ahrs` to the controller.
//      * Option B (macro DEFINED by the standalone `.so` CMake build): a
//        COMPILE-TIME INCLUDE SEAM resolves the token `AP_AHRS` to
//        AfsimL1_AHRS_Shim, so the AP_L1_Control translation unit binds its
//        `_ahrs.<method>` call sites to the identically named shim methods and
//        the constructor parameter `AP_AHRS &` denotes the shim; the member-
//        initialiser list then passes `_ahrs_shim` directly, avoiding the
//        EKF/DCM stack. This is the minimal-footprint path.
//    EITHER WAY this facade owns the shim (`_ahrs_shim`) and pushes the host-
//    injected state into it via set_state_ne() before delegating.
//
//  CONSTRAINTS (AAP 0.7.1):
//    - Delegate EXACTLY to AP_L1_Control; do not re-implement L1 math.
//    - Do NOT modify AP_L1_Control (the additive, default-off set_update_dt
//      seam was already added by the parent-folder change; this file merely
//      relies on it).
//    - Build waypoints only via a datum + Location::offset(north, east); stock
//      ArduPilot has no two-argument Location(N, E) constructor.
//    - No vehicle-firmware dependency; no new third-party dependency.
// ============================================================================

// AfsimL1Behavior.h transitively provides the three (and only) dependencies
// this translation unit needs: <AP_L1_Control/AP_L1_Control.h> (the wrapped
// controller), "AfsimL1_AHRS_Shim.h" (the injected-state adapter) and
// <AP_Common/Location.h> (the leg endpoints, which in turn pulls in AP_Math).
#include "AfsimL1Behavior.h"

#include <cmath>   // std::isfinite -- boundary validation of external inputs

// ----------------------------------------------------------------------------
// Input validation helper (CWE-20 defense at the service boundary)
// ----------------------------------------------------------------------------
//
// The service is driven by an EXTERNAL, untrusted host (for example AFSIM)
// through the C ABI, which forwards plain `double` scalars straight into this
// facade. A non-finite value (NaN / +/-Inf) that reached the guidance math
// would propagate silently: NaN defeats the controller's inherited upper-bound
// clamps (every `NaN > limit` comparison is false), and a non-finite position
// would poison Location::offset() and the cross-track geometry. To keep such
// values from ever entering the controller, every externally supplied scalar is
// validated at this boundary before it is cast to float and stored/used.
//
// finite_or() returns the value unchanged when it is finite, otherwise the
// supplied safe fallback. It deliberately does NOT clamp legitimate finite
// magnitudes, so valid (possibly large) positions and velocities are preserved.
namespace {
inline double finite_or(double value, double fallback)
{
    return std::isfinite(value) ? value : fallback;
}
} // namespace

// ----------------------------------------------------------------------------
// Construction
// ----------------------------------------------------------------------------

// AP_L1_Control declares no default constructor (its only constructor is
// AP_L1_Control(AP_AHRS &ahrs, const AP_TECS *tecs)) and is marked
// CLASS_NO_COPY, so `_l1` MUST be built in the member-initialiser list. Members
// initialise in declaration order (AfsimL1Behavior.h guarantees every AHRS
// member precedes `_l1`), so the AHRS handed to the controller here is fully
// constructed first. A nullptr TECS is passed because this standalone service
// performs lateral (L1) guidance only; AP_L1_Control guards every TECS
// dereference with an `if (_tecs != nullptr)` check, so a null TECS is safe.
// `_prev` / `_next` use the in-class `{}` initialisers from the header (zeroed
// Locations) and are (re)seeded by init() / set_leg_ne().
//
// The AHRS the controller is composed against is selected by the compile-time
// seam (AAP 0.6.2; see AfsimL1Behavior.h):
//   * Option A (recommended default -- macro AFSIML1_L1_USES_SHIM_AHRS UNDEFINED,
//     e.g. the in-tree waf `use='ap'` build): a genuine AP_AHRS member `_ahrs`,
//     so every type stays a real ArduPilot type end-to-end.
//   * Option B (macro DEFINED by the standalone `.so` build): the owned
//     `_ahrs_shim`, which that build's include seam has made the `AP_AHRS` type,
//     so the controller reads the injected state directly with no EKF/DCM stack.
// EITHER WAY `_ahrs_shim` is the canonical injected-state record fed by
// set_state_ne().
//
// AP_L1_Control's constructor runs AP_Param::setup_object_defaults(this,
// var_info), which applies the stock L1 defaults (PERIOD=17, DAMPING=0.75,
// XTRACK_I=0.02, LIM_BANK=0.0), so `_l1` already carries vehicle-matching gains
// the moment it is constructed.
AfsimL1Behavior::AfsimL1Behavior()
    : _ahrs_shim()
#if defined(AFSIML1_L1_USES_SHIM_AHRS)
    // Option B: the standalone build's include seam has made
    // AP_AHRS == AfsimL1_AHRS_Shim, so the controller binds the shim directly and
    // reads the host-injected state through it.
    , _l1(_ahrs_shim, nullptr)
#else
    // Option A (default): compose against a genuine AP_AHRS for full-stack
    // fidelity. `_ahrs` is declared between `_ahrs_shim` and `_l1` in the header,
    // so this initialiser order matches the declaration order (-Werror=reorder).
    , _ahrs()
    , _l1(_ahrs, nullptr)
#endif
{
}

// ----------------------------------------------------------------------------
// Initialisation
// ----------------------------------------------------------------------------

// Seed a simple default leg and (re)assert the stock L1 tuning. Invoked by the
// C ABI's L1_Init(). Safe to call repeatedly.
void AfsimL1Behavior::init()
{
    // Seed the simple default leg from the user "initialize a simple leg"
    // example (AAP 0.7.2): prev = (0, 0) N/E, next = (0, 500) N/E, in metres.
    // set_leg_ne() builds both endpoints from the shared datum + offset
    // convention so the leg is consistent with the shim's synthesized position.
    set_leg_ne(0.0, 0.0, 0.0, 500.0);

    // Explicitly seed the stock L1 tracking period so standalone guidance
    // matches the vehicle default (AAP 0.4.1 "seed PERIOD/DAMPING/XTRACK_I/
    // LIM_BANK"). set_default_period() is the only public seeding hook on
    // AP_L1_Control; DAMPING (0.75), XTRACK_I (0.02) and LIM_BANK (0.0) are
    // already applied by the controller's constructor via
    // AP_Param::setup_object_defaults(this, var_info), so no further seeding is
    // required for them.
    _l1.set_default_period(17.0f);
}

// ----------------------------------------------------------------------------
// Per-step guidance advance
// ----------------------------------------------------------------------------

// Advance L1 guidance by one control step using a host-supplied dt (seconds).
// Mirrors the user example (AAP 0.7.2): the host owns the timebase and the
// current leg is (re)evaluated each step.
void AfsimL1Behavior::execute(double dt_seconds)
{
    // Validate the host-supplied dt at the service boundary (CWE-20) BEFORE it
    // reaches the controller. A non-finite (NaN/Inf) or negative dt must never
    // enter the guidance timing: a negative dt would run the cross-track
    // integrator backwards, and a NaN dt would slip past the controller's
    // inherited upper-bound clamps (every `NaN > limit` compares false) and
    // poison _L1_xtrack_i. Such values are collapsed to 0 -- a safe no-op step.
    // The controller's own UPPER-bound semantics (reinitialise the integrator
    // when dt > 1 s, cap at 0.1 s) are deliberately left INTACT and are neither
    // duplicated nor capped here, so numerical behavior is preserved for every
    // valid dt.
    double dt = finite_or(dt_seconds, 0.0);
    if (dt < 0.0) {
        dt = 0.0;
    }

    // The host drives timing: inject the validated dt so AP_L1_Control uses it
    // instead of its internal AP_HAL::micros() delta. This MUST precede
    // update_waypoint() because the controller consumes the override inside that
    // call's timing block; the controller's existing clamp semantics are
    // preserved, so the numerical behavior is unchanged.
    _l1.set_update_dt(static_cast<float>(dt));

    // The current platform state (North/East position, East/North velocity,
    // yaw, pitch) is pushed into the AHRS shim by set_state_ne() prior to this
    // call, so update_waypoint() reads exactly the injected state through the
    // shim's AP_AHRS-shaped accessors. Evaluate the active leg; the resulting
    // roll command and lateral-acceleration demand are then available through
    // get_roll_deg() / get_lat_accel().
    _l1.update_waypoint(_prev, _next);
}

// ----------------------------------------------------------------------------
// Input injection -- legs
// ----------------------------------------------------------------------------

// Set the active navigation leg from previous/next North/East offsets (metres).
void AfsimL1Behavior::set_leg_ne(double prevN, double prevE, double nextN, double nextE)
{
    // Build each waypoint from the SAME all-zero reference datum plus a
    // North/East offset that AfsimL1_AHRS_Shim::get_location() uses for the
    // current position. Sharing one origin keeps AP_L1_Control's
    // Location::get_distance_NE() and Location::get_bearing_to() geometry
    // between the current position and the leg endpoints self-consistent. A
    // default-constructed Location is zeroed (equator / prime meridian), so
    // offsetting it reproduces the datum + offset technique used by the shim.
    // (Stock ArduPilot has no two-argument Location(N, E) constructor.)
    Location prev_wp;
    prev_wp.offset(static_cast<float>(prevN), static_cast<float>(prevE));
    _prev = prev_wp;

    Location next_wp;
    next_wp.offset(static_cast<float>(nextN), static_cast<float>(nextE));
    _next = next_wp;
}

// ----------------------------------------------------------------------------
// Input injection -- platform state
// ----------------------------------------------------------------------------

// Inject the current platform state consumed by the guidance controller,
// forwarding each component to the AHRS shim's setters. The argument order
// matches the user example (AAP 0.7.2): East velocity precedes North velocity.
void AfsimL1Behavior::set_state_ne(double n, double e, double velE, double velN, double yaw_cd, double pitch_rad)
{
    // Sanitize every externally supplied scalar at the service boundary
    // (CWE-20) before it is cast to float and stored: a non-finite (NaN/Inf)
    // component would otherwise propagate through Location::offset() and the
    // controller's cross-track geometry. Each non-finite value is replaced with
    // 0 (a safe neutral); legitimate finite magnitudes are left unclamped so
    // valid large positions/velocities are preserved. One assignment per line
    // keeps the guard unambiguous.
    const double n_s         = finite_or(n, 0.0);
    const double e_s         = finite_or(e, 0.0);
    const double velE_s      = finite_or(velE, 0.0);
    const double velN_s      = finite_or(velN, 0.0);
    const double yaw_cd_s    = finite_or(yaw_cd, 0.0);
    const double pitch_rad_s = finite_or(pitch_rad, 0.0);

    // Position (North/East offset from the datum, metres) -> get_location().
    _ahrs_shim.set_location_NE(static_cast<float>(n_s), static_cast<float>(e_s));

    // Ground velocity (East/North components, m/s) -> groundspeed_vector().
    // The shim maps these onto ArduPilot's Vector2f(x = North, y = East)
    // convention internally.
    _ahrs_shim.set_velocity_EN(static_cast<float>(velE_s), static_cast<float>(velN_s));

    // Heading/yaw (centidegrees) -> get_yaw_rad() and the yaw_sensor member,
    // both of which the shim keeps in sync.
    _ahrs_shim.set_yaw_cd(static_cast<float>(yaw_cd_s));

    // Pitch (radians) -> get_pitch_rad().
    _ahrs_shim.set_pitch_rad(static_cast<float>(pitch_rad_s));
}

// ----------------------------------------------------------------------------
// Output extraction
// ----------------------------------------------------------------------------

// Roll command from the most recent execute(), in degrees. AP_L1_Control emits
// the roll demand in centidegrees via nav_roll_cd(); dividing by 100 converts
// to degrees (matching the user example's `l1.nav_roll_cd() / 100.0f`). The
// float division is widened to double for the C ABI boundary. const: reads the
// controller's last-computed output only.
double AfsimL1Behavior::get_roll_deg() const
{
    return static_cast<double>(_l1.nav_roll_cd() / 100.0f);
}

// Lateral-acceleration demand from the most recent execute(), in m/s^2. Direct
// passthrough of AP_L1_Control::lateral_acceleration(), widened to double for
// the C ABI boundary. const: reads the controller's last-computed output only.
double AfsimL1Behavior::get_lat_accel() const
{
    return static_cast<double>(_l1.lateral_acceleration());
}
