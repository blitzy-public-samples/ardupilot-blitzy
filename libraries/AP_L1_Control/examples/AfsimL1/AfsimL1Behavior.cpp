/// @file    AfsimL1Behavior.cpp
/// @brief   Implementation of AfsimL1Behavior -- the task-oriented FACADE over
///          ArduPilot's AP_L1_Control lateral-navigation guidance controller
///          used by the reusable AfsimL1 service.
///
/// This translation unit implements the facade declared in AfsimL1Behavior.h
/// (see Agent Action Plan sections 0.1.1, 0.3.3, 0.7.1). The AfsimL1 service
/// extracts ArduPilot's L1 lateral-navigation guidance (AP_L1_Control) into a
/// reusable, modular unit that an external host (e.g. the AFSIM simulation
/// environment) can drive through a stable C ABI. This facade composes the
/// guidance controller with a host-driven AHRS state shim and re-plumbs the
/// controller's inputs (legs, state, timing) and outputs (roll, lateral
/// acceleration) through dependency-injection seams instead of pulling them
/// implicitly from the vehicle flight loop.
///
/// STRICTLY BEHAVIOR-PRESERVING (AAP 0.1.1, 0.7.1): NO L1 mathematics live in
/// this file. Every guidance computation is delegated verbatim to the unchanged
/// AP_L1_Control instance through update_waypoint() / nav_roll_cd() /
/// lateral_acceleration(), with the host-supplied control-step dt injected via
/// the controller's additive, default-off set_update_dt() seam. The facade's
/// entire responsibility is to (a) seed the stock L1 tuning so standalone
/// guidance matches the vehicle, (b) inject legs / state / timing, and (c)
/// surface the commanded outputs in host-friendly units.
///
/// ---------------------------------------------------------------------------
/// Central engineering decision realized here (AAP 0.6.2)
/// ---------------------------------------------------------------------------
/// AP_L1_Control binds a concrete reference `AP_AHRS &_ahrs` and calls six
/// NON-VIRTUAL accessors on it, so ordinary subclassing cannot decouple the
/// state. The constructor's member-initializer list therefore wires the
/// controller directly to the owned AfsimL1_AHRS_Shim (conceptually
/// `_l1{_ahrs_shim, nullptr}`):
///   * Option B (RECOMMENDED, minimal-footprint standalone `.so`): a
///     compile-time include seam controlled by the standalone build makes
///     AP_L1_Control's `AP_AHRS` resolve to the shim (whose six read methods
///     are name/signature-identical), so passing the shim as the AHRS argument
///     type-checks and the EKF/DCM stack is never linked.
///   * Option A (maximum-fidelity fallback): the same argument position instead
///     receives a real AP_AHRS driven in an external-navigation mode.
/// Either way this file passes `_ahrs_shim`, and the shim is guaranteed to be
/// fully constructed before `_l1` because it is declared first in
/// AfsimL1Behavior.h (members are constructed in declaration order).

#include "AfsimL1Behavior.h"

// AfsimL1Behavior.h transitively provides everything this translation unit
// needs: <AP_L1_Control/AP_L1_Control.h> (the composed controller and its
// set_update_dt / update_waypoint / nav_roll_cd / lateral_acceleration /
// set_default_period surface), "AfsimL1_AHRS_Shim.h" (the injection setters),
// and <AP_Common/Location.h> (the Location type and its offset() method). No
// additional include is required, and no vehicle-firmware or new third-party
// dependency is introduced (AAP 0.4.2, 0.7.1).

namespace {

/// Stock NAVL1_PERIOD default, in seconds (AP_L1_Control var_info "PERIOD",
/// AP_L1_Control.cpp). Named here so the "standalone guidance must match the
/// vehicle" intent is explicit and traceable. It is (re)asserted through the
/// controller's public set_default_period() hook in init().
///
/// The remaining stock L1 gains are applied by the AP_L1_Control constructor's
/// AP_Param::setup_object_defaults(this, var_info) and are intentionally NOT
/// re-applied here (they are AP_Float parameters with no public per-field
/// seeding hook, and the constructor already installs exactly these values):
///   NAVL1_DAMPING  = 0.75  (var_info "DAMPING")
///   NAVL1_XTRACK_I = 0.02  (var_info "XTRACK_I")
///   NAVL1_LIM_BANK = 0.0   (var_info "LIM_BANK")
constexpr float AFSIM_L1_STOCK_PERIOD_S = 17.0f;

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
// The member-initializer list constructs the owned shim first, then composes
// the guidance controller against it with a null TECS pointer. Declaration
// order in AfsimL1Behavior.h (shim before controller) guarantees the shim is a
// fully-formed object by the time _l1's initializer references it, so the
// member-init order below intentionally matches that declaration order and
// triggers no -Wreorder diagnostic.
AfsimL1Behavior::AfsimL1Behavior()
    : _ahrs_shim()              // host-driven AP_AHRS read-surface adapter (see header)
    , _l1(_ahrs_shim, nullptr)  // compose the controller against the shim; nullptr TECS
{
    // _prev / _next are default-constructed (zeroed) Locations -- the origin of
    // the relative-navigation frame -- until a leg is supplied by init() or
    // set_leg_ne(). AP_Param::setup_object_defaults(this, var_info) has already
    // executed inside the AP_L1_Control constructor, so _l1 already carries the
    // stock NAVL1_* tuning (PERIOD=17, DAMPING=0.75, XTRACK_I=0.02,
    // LIM_BANK=0.0); init() additionally (re)asserts PERIOD for robustness.
}

// ---------------------------------------------------------------------------
// init() -- seed a simple default leg and (re)assert the stock L1 period
// ---------------------------------------------------------------------------
void AfsimL1Behavior::init()
{
    // Seed the user example's "simple leg": prev at the datum (0 N, 0 E) and
    // next 500 m along the East axis (0 N, 500 E). This lets a host exercise the
    // service before wiring in real routing; a later set_leg_ne() call overrides
    // whatever init() seeded here.
    set_leg_ne(0.0, 0.0, 0.0, 500.0);

    // Explicitly (re)assert the stock NAVL1_PERIOD so standalone guidance
    // matches the vehicle even if an external parameter load has run.
    // set_default_period() is the only public per-field seeding hook on
    // AP_L1_Control; DAMPING / XTRACK_I / LIM_BANK are already seeded to their
    // stock values by the controller's setup_object_defaults at construction
    // and require no (and expose no) additional public seeding (AAP 0.4.1,
    // 0.6.1).
    _l1.set_default_period(AFSIM_L1_STOCK_PERIOD_S);
}

// ---------------------------------------------------------------------------
// set_leg_ne() -- define the active navigation leg from N/E offsets (metres)
// ---------------------------------------------------------------------------
void AfsimL1Behavior::set_leg_ne(double prevN, double prevE, double nextN, double nextE)
{
    // Build each endpoint from a FIXED datum plus a North/East offset in metres.
    // A default-constructed Location is a zeroed datum (equator / prime
    // meridian) -- the SAME datum convention AfsimL1_AHRS_Shim::get_location
    // uses (`loc = _datum; loc.offset(north, east)`). Sharing the datum keeps
    // the horizontal helpers AP_L1_Control invokes on these Locations
    // (Location::get_distance_NE and Location::get_bearing_to) consistent
    // between the injected current position and the active leg.
    //
    // NOTE (AAP 0.7.1): a two-argument `Location{N, E}` literal does NOT exist
    // on the stock Location type; the datum + offset(north, east) form is the
    // correct realization of the example's shape. Offsets are cast to float to
    // match the shim's float-precision position storage exactly.
    Location p;
    p.offset(static_cast<float>(prevN), static_cast<float>(prevE));
    _prev = p;

    Location n;
    n.offset(static_cast<float>(nextN), static_cast<float>(nextE));
    _next = n;
}

// ---------------------------------------------------------------------------
// set_state_ne() -- inject the current platform state into the AHRS shim
// ---------------------------------------------------------------------------
void AfsimL1Behavior::set_state_ne(double n, double e, double velE, double velN, double yaw_cd, double pitch_rad)
{
    // Forward each component to the owned shim, which backs the exact AP_AHRS
    // read surface AP_L1_Control consumes. The argument order
    // (n, e, velE, velN, yaw_cd, pitch_rad) matches the user example (AAP
    // 0.7.2) and the C ABI's L1_SetStateNE entry point. Values are cast to float
    // (the shim's native storage precision); the shim stores velocity in
    // ArduPilot's (x = North, y = East) convention internally.
    _ahrs_shim.set_location_NE(static_cast<float>(n), static_cast<float>(e));
    _ahrs_shim.set_velocity_EN(static_cast<float>(velE), static_cast<float>(velN));
    _ahrs_shim.set_yaw_cd(static_cast<float>(yaw_cd));
    _ahrs_shim.set_pitch_rad(static_cast<float>(pitch_rad));
}

// ---------------------------------------------------------------------------
// execute() -- advance the L1 guidance by one host-timed control step
// ---------------------------------------------------------------------------
void AfsimL1Behavior::execute(double dt_seconds)
{
    // The current platform state should already have been pushed into the shim
    // for this step via set_state_ne() (the host maps its platform state into
    // the shim before / at execute()); the composed controller reads it back
    // through the shim's AP_AHRS-shaped accessors during update_waypoint().

    // Hand the host-supplied control-step dt to the controller FIRST. This
    // engages the additive, default-off timing seam so that update_waypoint()
    // uses this dt -- with the controller's stock clamp semantics preserved --
    // instead of the internal AP_HAL::micros() delta. The host (AFSIM) owns the
    // timebase.
    _l1.set_update_dt(static_cast<float>(dt_seconds));

    // Drive exactly one L1 guidance update for the current leg. This MUST run
    // AFTER set_update_dt() so the injected dt is in effect for this step. The
    // commanded outputs are not read here; they are exposed by the getters.
    _l1.update_waypoint(_prev, _next);
}

// ---------------------------------------------------------------------------
// Output getters -- surface the controller's last-computed commands
// ---------------------------------------------------------------------------
double AfsimL1Behavior::get_roll_deg() const
{
    // AP_L1_Control::nav_roll_cd() returns the commanded bank angle in
    // centidegrees (int32_t); divide by 100.0f to convert to degrees. The
    // integer numerator is promoted to float by the float divisor, so the
    // fractional part is preserved before the widening to double.
    return static_cast<double>(_l1.nav_roll_cd() / 100.0f);
}

double AfsimL1Behavior::get_lat_accel() const
{
    // AP_L1_Control::lateral_acceleration() is already in m/s^2 (+ve to the
    // right); return it verbatim, widened to double for the C ABI boundary.
    return static_cast<double>(_l1.lateral_acceleration());
}
