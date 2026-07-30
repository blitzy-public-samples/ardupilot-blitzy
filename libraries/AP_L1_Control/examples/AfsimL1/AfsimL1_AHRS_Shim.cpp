// ============================================================================
//  AfsimL1_AHRS_Shim.cpp  --  Adapter implementation for the AfsimL1 reusable
//                              L1 guidance service
//                              (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  This translation unit implements the AfsimL1_AHRS_Shim contract declared in
//  AfsimL1_AHRS_Shim.h. The shim is the ADAPTER layer (AAP 0.3.3) of the
//  reusable service: it presents the EXACT read surface that AP_L1_Control
//  consumes from its `AP_AHRS &_ahrs`, but sources every value from state
//  pushed in by an external host (for example the AFSIM simulator) rather than
//  from live sensor fusion.
//
//  BEHAVIOR-PRESERVING (AAP 0.1.1, 0.6.2): this file only stores/echoes the
//  host-injected state and synthesizes a Location from a fixed datum plus a
//  North/East offset. It performs NO L1 guidance mathematics; the controller
//  observes the same values it would read from a live AP_AHRS, so the guidance
//  output is unchanged -- only the data source is inverted to explicit
//  dependency injection.
//
//  DATUM CONVENTION (must stay consistent with AfsimL1Behavior):
//    Every injected North/East position is expressed in meters relative to a
//    single fixed reference datum, `_datum`, which is a default-constructed
//    (all-zero) Location at the equator / prime meridian. get_location() copies
//    `_datum` and applies the injected (north, east) offset. The facade
//    (AfsimL1Behavior::set_leg_ne / init) builds its `prev` / `next` waypoints
//    from the SAME all-zero-datum + offset technique, so the controller's
//    Location::get_distance_NE() and Location::get_bearing_to() computations
//    between the current position and the leg waypoints remain self-consistent.
//
//  CONSTRAINTS (AAP 0.7.1): no <AP_AHRS/AP_AHRS.h> include and no inheritance
//  from AP_AHRS (its accessors are non-virtual, so subclassing is ineffective,
//  and Option B deliberately avoids pulling in the EKF/DCM stack); no vehicle
//  firmware dependency; no new third-party dependency.
// ============================================================================

#include "AfsimL1_AHRS_Shim.h"

// Location and the AP_Math primitives (Vector2f, radians(), wrap_360_cd()) are
// already visible transitively through AfsimL1_AHRS_Shim.h, but including them
// explicitly keeps this translation unit self-documenting and robust to header
// refactoring.
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <cmath>   // std::isfinite -- reject non-finite yaw before conversion

// ----------------------------------------------------------------------------
// Read surface -- signature-identical to the AP_AHRS accessors AP_L1_Control
// reads. Under the compile-time include seam (Option B, AAP 0.6.2) the
// controller's `_ahrs.<method>` call sites bind to these definitions unchanged.
// ----------------------------------------------------------------------------

// Mirrors AP_AHRS::get_location (AP_AHRS.h:L103); read by AP_L1_Control at
// AP_L1_Control.cpp:L230 -- `if (_ahrs.get_location(_current_loc) == false)`.
//
// Synthesizes the current Location by copying the fixed reference datum and
// offsetting it by the injected North/East position. Using the `_datum` member
// (rather than a throwaway local) guarantees that every Location produced by
// the service shares one origin, which is exactly what keeps the controller's
// get_distance_NE() / get_bearing_to() geometry consistent between the current
// position and the leg waypoints built by the facade.
//
// Always returns true: injected state is by definition available, so there is
// no "no-GPS" condition to report. Returning true is required for the
// controller's validity gate at L230 to pass so that guidance actually runs.
bool AfsimL1_AHRS_Shim::get_location(Location &loc) const
{
    loc = _datum;                     // fixed all-zero datum (equator / prime meridian)
    loc.offset(_north_m, _east_m);    // apply injected N/E offset in meters (Location.h:L118)
    return true;
}

// Mirrors AP_AHRS::groundspeed_vector (AP_AHRS.h:L239); read at
// AP_L1_Control.cpp:L236. Returns a const reference to the stored member so the
// reference can never dangle. AP_AHRS itself hands back a reference to its own
// member (`state.ground_speed_vec`), and the controller copies the result into
// a local Vector2f, so a reference return is both correct and matching.
const Vector2f &AfsimL1_AHRS_Shim::groundspeed_vector() const
{
    return _groundspeed_vector;
}

// Mirrors AP_AHRS::get_yaw_rad (AP_AHRS.h:L586); read at AP_L1_Control.cpp:L59
// and L61. Yaw in radians, measured clockwise from North.
float AfsimL1_AHRS_Shim::get_yaw_rad() const
{
    return _yaw_rad;
}

// Mirrors AP_AHRS::get_pitch_rad (AP_AHRS.h:L585); read at AP_L1_Control.cpp:L91
// (nav_roll_cd) inside constrain_float(). Pitch in radians.
float AfsimL1_AHRS_Shim::get_pitch_rad() const
{
    return _pitch_rad;
}

// Mirrors AP_AHRS::get_EAS2TAS (AP_AHRS.h:L160); read at AP_L1_Control.cpp:L126
// (turn_distance) and L159 (loiter_radius). Equivalent-to-true airspeed ratio;
// defaults to unity (1.0) when the host injects nothing (AAP 0.6.1: "injected
// or unit default"), which leaves the sq(EAS2TAS) scaling neutral.
float AfsimL1_AHRS_Shim::get_EAS2TAS() const
{
    return _eas2tas;
}

// ----------------------------------------------------------------------------
// Injection setters -- the host pushes state in through these each step.
// AfsimL1Behavior::set_state_ne(...) forwards to them (AAP 0.7.2).
// ----------------------------------------------------------------------------

// Store the vehicle's North/East position, in meters, relative to `_datum`.
// Consumed by get_location() to synthesize the current Location.
void AfsimL1_AHRS_Shim::set_location_NE(float n, float e)
{
    _north_m = n;
    _east_m  = e;
}

// Store the ground-speed vector from host-supplied East/North components.
//
// NOTE the argument order is (velE, velN), matching the user example (AAP
// 0.7.2). ArduPilot's groundspeed-vector convention is Vector2f(x = North,
// y = East) -- AP_AHRS::groundspeed_vector() returns getVelNED().xy() -- so map
// North -> .x and East -> .y here. This keeps groundspeed_vector().length()
// (speed magnitude) and groundspeed_vector().angle() (course = atan2(y, x) =
// atan2(East, North), i.e. measured clockwise from North) consistent with the
// yaw reference the controller compares the course against at
// AP_L1_Control.cpp:L245.
void AfsimL1_AHRS_Shim::set_velocity_EN(float velE, float velN)
{
    _groundspeed_vector.x = velN;   // North component
    _groundspeed_vector.y = velE;   // East component
}

// Set the yaw from a centidegree value, keeping BOTH read paths the controller
// uses in sync: the radians accessor get_yaw_rad() (AP_L1_Control.cpp:L59/L61)
// and the public centidegree member yaw_sensor (AP_L1_Control.cpp:L70/L72).
//
// Input validation and normalisation (CWE-20): the previous implementation cast
// the host-derived float straight to int32_t (`(int32_t)yaw_cd`), so a
// non-finite yaw (NaN/+-Inf) or a finite value outside the int32_t range invoked
// undefined behavior, and an un-normalised large value produced non-useful yaw
// radians. This is hardened in two steps that leave valid inputs unchanged:
//   1. A non-finite yaw collapses to 0 centidegrees (due North) -- a safe
//      neutral -- BEFORE any arithmetic, because wrap_360_cd(NaN/Inf) is itself
//      NaN (fmodf of a non-finite is NaN).
//   2. wrap_360_cd() normalises the (now finite) value to the canonical
//      [0, 36000) centidegree range that a real AP_AHRS::yaw_sensor also holds.
//      That range is always representable exactly as float and well within
//      int32_t, so the cast can neither overflow nor invoke UB, and both read
//      paths stay finite and mutually consistent.
// For any already-normalised, finite yaw (e.g. 9000 cd = 90 deg) the result is
// identical to the original conversion, so guidance behavior is preserved; and
// because trig on yaw is periodic, wrapping an out-of-range angle is also
// behavior-neutral for the controller while making the int cast safe.
void AfsimL1_AHRS_Shim::set_yaw_cd(float yaw_cd)
{
    if (!std::isfinite(yaw_cd)) {
        yaw_cd = 0.0f;
    }
    const float wrapped_cd = wrap_360_cd(yaw_cd);   // -> [0, 36000)
    _yaw_rad   = radians(wrapped_cd * 0.01f);
    yaw_sensor = (int32_t)wrapped_cd;
}

// Store the pitch, in radians, returned by get_pitch_rad().
void AfsimL1_AHRS_Shim::set_pitch_rad(float pitch_rad)
{
    _pitch_rad = pitch_rad;
}
