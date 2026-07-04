/// @file    AfsimL1_AHRS_Shim.cpp
/// @brief   Definitions for AfsimL1_AHRS_Shim — the host-driven, AP_AHRS-shaped
///          state adapter used by the AfsimL1 reusable L1-guidance service.
///
/// This translation unit implements the ADAPTER layer of the AfsimL1 service
/// (see AfsimL1_AHRS_Shim.h and Agent Action Plan sections 0.1.1, 0.6.2). The
/// service extracts ArduPilot's L1 lateral-navigation guidance (AP_L1_Control)
/// into a reusable module an external host (e.g. the AFSIM simulator) can drive
/// through a stable C ABI. Where the vehicle firmware lets AP_L1_Control pull
/// its Position/Navigation state implicitly from a live, sensor-fused
/// `AP_AHRS &_ahrs`, this shim inverts that dependency: the host PUSHES the
/// platform state in each control step through the set_*() injectors, and the
/// composed AP_L1_Control instance reads the same values back through the
/// AP_AHRS-shaped accessors declared in the header.
///
/// The work is strictly BEHAVIOR-PRESERVING. Nothing here performs L1
/// mathematics — the file only stores the injected state and, for position,
/// materializes a real `Location` from a fixed datum plus the injected
/// North/East offset. All guidance math continues to live, unchanged, inside
/// AP_L1_Control.
///
/// ---------------------------------------------------------------------------
/// Datum / coordinate convention (MUST match the facade, AAP 0.6.1)
/// ---------------------------------------------------------------------------
/// All injected North/East positions are treated as offsets, in meters, from a
/// single fixed reference datum: the `_datum` member, a default-constructed
/// (all-zero) Location that sits at the equator / prime meridian. get_location()
/// reports `datum + offset(north, east)`. AfsimL1Behavior constructs its `prev`
/// and `next` waypoint Locations from the SAME datum convention, so that the
/// horizontal helpers AP_L1_Control calls on those Locations — Location::
/// get_distance_NE() and Location::get_bearing_to() — return consistent NE
/// distances and bearings between the current position and the active leg.
/// Because the datum is at latitude 0 the longitude scaling factor is ~1.0 and
/// small local offsets map cleanly to meters, keeping the relative-navigation
/// frame numerically well-behaved.

#include "AfsimL1_AHRS_Shim.h"

// The two headers below are already pulled in transitively by the shim header
// included above; they are restated explicitly here (include-what-you-use) to
// document this translation unit's direct dependencies:
//   * <AP_Common/Location.h> — Location (the out-parameter of get_location) and
//     its offset(ofs_north, ofs_east) method, used to build the reported
//     position from the datum.
//   * <AP_Math/AP_Math.h>    — Vector2f (the stored ground-speed vector type)
//     and radians() (used to convert the injected centidegree yaw to radians).
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>

// ===========================================================================
// AP_AHRS-compatible read surface
//
// Each definition below matches — byte-for-byte — the signature of the AP_AHRS
// accessor that AP_L1_Control invokes on its `_ahrs` reference. Under the
// Option B compile-time include seam (AAP 0.6.2) this lets every `_ahrs.<x>`
// call site inside AP_L1_Control bind to this shim without any change to the
// controller's source.
// ===========================================================================

/// Report the current vehicle position as a Location.
///
/// Mirrors AP_AHRS::get_location (AP_AHRS.h:L103); consumed at
/// AP_L1_Control.cpp:L260 as `if (_ahrs.get_location(_current_loc) == false)`.
///
/// The reported position is built from the fixed `_datum` plus the injected
/// North/East offset (set via set_location_NE). The method is const, so it
/// offsets a copy of the datum held in the caller-supplied out-parameter rather
/// than mutating the `_datum` member. It always returns true because, in the
/// injected model, a host-supplied position is always available — this ensures
/// the controller's L260 validity check passes and guidance runs every step.
bool AfsimL1_AHRS_Shim::get_location(Location &loc) const
{
    // Start from the fixed datum (a zeroed Location == equator/prime meridian),
    // then extrapolate by the injected North/East offset, in meters. Using the
    // shared _datum member (rather than a fresh local) centralizes the datum
    // convention that the facade's leg construction must also honor.
    loc = _datum;
    loc.offset(_north_m, _east_m);   // Location::offset(ofs_north, ofs_east) — meters
    return true;
}

/// Return the ground-speed vector, in meters/second.
///
/// Mirrors AP_AHRS::groundspeed_vector (AP_AHRS.h:L239); consumed at
/// AP_L1_Control.cpp:L266, which copies the returned reference into a local
/// before use. The vector follows ArduPilot's (x = North, y = East) convention
/// so that length() is the ground speed and angle() (== atan2(East, North)) is
/// directly comparable to get_yaw_rad().
///
/// A CONST REFERENCE to the stable `_groundspeed_vector` data member is
/// returned — never a reference to a temporary — so the reference cannot
/// dangle and stays valid for the lifetime of this shim.
const Vector2f &AfsimL1_AHRS_Shim::groundspeed_vector() const
{
    return _groundspeed_vector;
}

/// Return the current yaw (heading) in radians.
///
/// Mirrors AP_AHRS::get_yaw_rad (AP_AHRS.h:L586); consumed at
/// AP_L1_Control.cpp:L59 and L61. The value is maintained by set_yaw_cd().
float AfsimL1_AHRS_Shim::get_yaw_rad() const
{
    return _yaw_rad;
}

/// Return the current pitch in radians.
///
/// Mirrors AP_AHRS::get_pitch_rad (AP_AHRS.h:L585); consumed at
/// AP_L1_Control.cpp:L91. The value is maintained by set_pitch_rad().
float AfsimL1_AHRS_Shim::get_pitch_rad() const
{
    return _pitch_rad;
}

/// Return the equivalent-to-true airspeed scaling ratio (EAS2TAS).
///
/// Mirrors AP_AHRS::get_EAS2TAS (AP_AHRS.h:L160); consumed at
/// AP_L1_Control.cpp:L150 (as sq(_ahrs.get_EAS2TAS())). The AfsimL1 injection
/// surface does not currently supply an air-density correction, so the shim
/// returns the neutral value 1.0 (sea-level equivalence), matching the
/// "injected or unit default" policy in AAP 0.6.1.
float AfsimL1_AHRS_Shim::get_EAS2TAS() const
{
    return _eas2tas;
}

// ===========================================================================
// Injection setters
//
// The host pushes the platform state in through these once per control step
// (AfsimL1Behavior::set_state_ne forwards to them, AAP 0.7.2). They only store
// the supplied values; the AP_AHRS-shaped accessors above read them back.
// ===========================================================================

/// Inject the vehicle North/East position, in meters, relative to the datum.
/// Stored for later use by get_location() when it builds the reported Location.
/// @param n  North offset from the datum, in meters.
/// @param e  East offset from the datum, in meters.
void AfsimL1_AHRS_Shim::set_location_NE(float n, float e)
{
    _north_m = n;
    _east_m  = e;
}

/// Inject the ground velocity, in meters/second.
///
/// The arguments follow the user example's order — East first, then North
/// (AAP 0.7.2). They are stored into the ground-speed vector using ArduPilot's
/// (x = North, y = East) convention, i.e. the North component maps to `.x` and
/// the East component maps to `.y`. This keeps groundspeed_vector() consistent
/// with its consumers in AP_L1_Control.cpp, where length() is taken as the
/// ground speed and angle() (atan2(y, x) == atan2(East, North)) is compared
/// against get_yaw_rad().
/// @param velE  East velocity component, in meters/second.
/// @param velN  North velocity component, in meters/second.
void AfsimL1_AHRS_Shim::set_velocity_EN(float velE, float velN)
{
    // (velE, velN) in  ->  (x = North, y = East) stored, per ArduPilot convention.
    _groundspeed_vector = Vector2f(velN, velE);
}

/// Inject the vehicle yaw, supplied in centidegrees.
///
/// Both representations the controller reads are updated and kept in sync:
///   * the radian source returned by get_yaw_rad(), stored as
///     radians(yaw_cd * 0.01f) (centidegrees -> degrees -> radians); and
///   * the centidegree public field yaw_sensor, stored as the value truncated
///     toward zero to an integer.
/// Keeping both consistent is required because AP_L1_Control reads the radian
/// form (L59/L61) and the centidegree field (L70/L72) on different paths.
/// @param yaw_cd  Yaw in centidegrees (e.g. 4500 == 45 degrees).
void AfsimL1_AHRS_Shim::set_yaw_cd(float yaw_cd)
{
    _yaw_rad   = radians(yaw_cd * 0.01f);
    yaw_sensor = static_cast<int32_t>(yaw_cd);
}

/// Inject the vehicle pitch, in radians. Returned verbatim by get_pitch_rad().
/// @param pitch_rad  Pitch angle, in radians.
void AfsimL1_AHRS_Shim::set_pitch_rad(float pitch_rad)
{
    _pitch_rad = pitch_rad;
}
