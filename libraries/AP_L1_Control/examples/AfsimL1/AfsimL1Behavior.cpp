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
//  CENTRAL ENGINEERING DECISION (AAP 0.6.2) -- AP_AHRS decoupling:
//    AP_L1_Control binds a concrete `AP_AHRS &_ahrs` and the six accessors it
//    reads are NON-VIRTUAL, so a plain subclass can neither override them nor
//    (their backing state being private) set them. Of the two behavior-
//    preserving approaches AAP 0.6.2 describes, this service implements Option B
//    exclusively: a COMPILE-TIME INCLUDE SEAM (supplied by the build; see
//    CMakeLists.txt / the seam-enabled wscript) resolves the token `AP_AHRS` to
//    AfsimL1_AHRS_Shim, so AP_L1_Control's `_ahrs.<method>` call sites bind to
//    the identically named shim methods and the controller's constructor
//    parameter `AP_AHRS &` denotes the shim. The member-initializer list below
//    therefore composes the controller against the owned `_ahrs_shim` -- the
//    SAME object set_state_ne() writes -- so host-injected state always reaches
//    the guidance controller. (Option A -- a genuine AP_AHRS driven by an
//    external-state EKF/DCM backend -- is out of scope for this extraction and
//    is NOT used; AfsimL1Behavior.h #errors if the seam is absent, so the
//    controller can never be composed against a different, never-written AHRS.)
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

#include <cmath>    // std::isfinite -- boundary validation of external inputs

// ----------------------------------------------------------------------------
// Input validation helper (CWE-20 defense at the service boundary)
// ----------------------------------------------------------------------------
//
// The service is driven by an EXTERNAL, untrusted host (for example AFSIM)
// through the C ABI, which forwards plain `double` scalars straight into this
// facade. Two failure modes must be stopped at this boundary before any value
// is narrowed to float and handed to the guidance math:
//   1. Non-finite input (NaN / +/-Inf). NaN defeats the controller's inherited
//      upper-bound clamps (every `NaN > limit` comparison is false), and a
//      non-finite position would poison Location::offset() and the cross-track
//      geometry.
//   2. Finite but out-of-range input. A large-magnitude double silently becomes
//      +/-Inf when narrowed to float (|x| > FLT_MAX), re-introducing failure
//      mode 1 downstream. Worse, a value that is finite *as a float* but still
//      astronomically large (|x| up to FLT_MAX ~= 3.4e38) overflows to +/-Inf --
//      and then NaN -- the moment the preserved L1 guidance arithmetic squares
//      it (the ground-speed vector is squared; squared position offsets are
//      summed), so clamping merely to the representable float range is not
//      sufficient to keep the guidance outputs finite.
//
// to_safe_float() is the SINGLE boundary-validation policy applied before EVERY
// narrowing static_cast<float> in this translation unit: it maps non-finite
// input to the supplied safe fallback (default 0) and clamps finite input to a
// domain-specific magnitude bound [-kMaxMagnitude, kMaxMagnitude] (1e18) that is
// deliberately tighter than the representable float range [-FLT_MAX, FLT_MAX].
// Because kMaxMagnitude^2 = 1e36 stays well within FLT_MAX (~3.4e38), the square
// of any accepted value -- and modest sums/products thereof in the guidance law
// -- cannot overflow to +/-Inf/NaN, so the service never returns a non-finite
// command however absurd the injected state. The bound sits ~11 orders of
// magnitude above any physically meaningful position (planet scale ~1e7 m) or
// velocity (~1e3 m/s), so legitimate finite magnitudes are returned unchanged:
// valid positions and velocities are preserved and the guidance math is
// unaffected. Domain-specific normalisation (e.g. yaw wrapping in the shim, the
// dt >= 0 rule in execute()) is layered on top of this finite/range guarantee by
// the respective setter.
namespace {
inline float to_safe_float(double value, float fallback = 0.0f)
{
    if (!std::isfinite(value)) {
        return fallback;
    }
    // Domain-specific magnitude bound (the CWE-20 "domain-specific bounds where
    // appropriate" step). Clamping to +/-kMaxMagnitude -- rather than to the
    // wider representable range +/-FLT_MAX -- guarantees that the square of any
    // accepted value (kMaxMagnitude^2 = 1e36) stays well within FLT_MAX
    // (~3.4e38). This prevents the preserved L1 guidance arithmetic (which
    // squares the ground-speed vector and sums squared position offsets) from
    // overflowing to +/-Inf and then NaN on absurd inputs, while remaining ~11
    // orders of magnitude above any physically meaningful position (planet scale
    // ~1e7 m) or velocity (~1e3 m/s) -- so no legitimate host input is altered.
    // A value beyond FLT_MAX is a fortiori beyond this bound, so this single
    // check also subsumes the representable-float-range guarantee.
    constexpr double kMaxMagnitude = 1.0e18;
    if (value > kMaxMagnitude) {
        return static_cast<float>(kMaxMagnitude);
    }
    if (value < -kMaxMagnitude) {
        return static_cast<float>(-kMaxMagnitude);
    }
    return static_cast<float>(value);
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
// The controller is composed against the owned `_ahrs_shim` (AAP 0.6.2 Option B;
// see AfsimL1Behavior.h). The build's compile-time include seam has made the
// token `AP_AHRS` denote AfsimL1_AHRS_Shim, so passing `_ahrs_shim` to the
// `AP_L1_Control(AP_AHRS &, const AP_TECS *)` constructor is well-typed, and the
// controller reads the injected state directly through the shim with no EKF/DCM
// stack. Crucially, `_ahrs_shim` is the SAME object set_state_ne() writes, so
// the guidance actually observes the host-injected state (this is the fix for
// the state-injection contract -- there is no separate, never-written AHRS).
//
// AP_L1_Control's constructor runs AP_Param::setup_object_defaults(this,
// var_info), which applies the stock L1 defaults (PERIOD=17, DAMPING=0.75,
// XTRACK_I=0.02, LIM_BANK=0.0), so `_l1` already carries vehicle-matching gains
// the moment it is constructed.
AfsimL1Behavior::AfsimL1Behavior()
    : _ahrs_shim()
    , _l1(_ahrs_shim, nullptr)
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
    // reaches the controller, applying the single to_safe_float() policy prior
    // to the narrowing to float. A non-finite (NaN/Inf) or out-of-float-range dt
    // is collapsed to 0, and a negative dt is then floored to 0 -- a safe no-op
    // step. This matters because a negative dt would run the cross-track
    // integrator backwards and a NaN dt would slip past the controller's
    // inherited upper-bound clamps (every `NaN > limit` compares false) and
    // poison _L1_xtrack_i. The controller's own UPPER-bound semantics
    // (reinitialise the integrator when dt > 1 s, cap at 0.1 s) are deliberately
    // left INTACT and are neither duplicated nor capped here, so numerical
    // behavior is preserved for every valid dt. (AP_L1_Control::set_update_dt()
    // independently re-validates finiteness/sign as a defense-in-depth backstop.)
    float dt = to_safe_float(dt_seconds);
    if (dt < 0.0f) {
        dt = 0.0f;
    }

    // The host drives timing: inject the validated dt so AP_L1_Control uses it
    // instead of its internal AP_HAL::micros() delta. This MUST precede
    // update_waypoint() because the controller consumes the override inside that
    // call's timing block; the controller's existing clamp semantics are
    // preserved, so the numerical behavior is unchanged.
    _l1.set_update_dt(dt);

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
    //
    // Every coordinate is passed through to_safe_float() before Location::offset
    // (CWE-20): the raw host doubles were previously cast straight to float, so a
    // non-finite or out-of-float-range coordinate would have produced a +/-Inf
    // offset and poisoned the leg geometry. Non-finite/oversize values collapse
    // to a safe finite value; legitimate finite coordinates are unchanged.
    Location prev_wp;
    prev_wp.offset(to_safe_float(prevN), to_safe_float(prevE));
    _prev = prev_wp;

    Location next_wp;
    next_wp.offset(to_safe_float(nextN), to_safe_float(nextE));
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
    // (CWE-20) with the single to_safe_float() policy, applied inline as the
    // narrowing to float. This closes two gaps: a non-finite (NaN/Inf) component
    // would propagate through Location::offset() and the controller's cross-track
    // geometry, AND a large finite double would previously overflow to a +/-Inf
    // float during the cast (the earlier finite-only guard did not stop this).
    // Non-finite/oversize values collapse to a safe finite value; legitimate
    // in-range finite magnitudes are preserved, so valid large positions and
    // velocities are unaffected. The shim's set_yaw_cd() additionally wraps yaw
    // to the canonical centidegree range before its int32_t cast.

    // Position (North/East offset from the datum, metres) -> get_location().
    _ahrs_shim.set_location_NE(to_safe_float(n), to_safe_float(e));

    // Ground velocity (East/North components, m/s) -> groundspeed_vector().
    // The shim maps these onto ArduPilot's Vector2f(x = North, y = East)
    // convention internally.
    _ahrs_shim.set_velocity_EN(to_safe_float(velE), to_safe_float(velN));

    // Heading/yaw (centidegrees) -> get_yaw_rad() and the yaw_sensor member,
    // both of which the shim keeps in sync.
    _ahrs_shim.set_yaw_cd(to_safe_float(yaw_cd));

    // Pitch (radians) -> get_pitch_rad().
    _ahrs_shim.set_pitch_rad(to_safe_float(pitch_rad));
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
