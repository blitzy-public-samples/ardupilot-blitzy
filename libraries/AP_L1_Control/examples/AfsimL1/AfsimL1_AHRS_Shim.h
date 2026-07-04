#pragma once

// ============================================================================
//  AfsimL1_AHRS_Shim.h  --  Adapter layer for the AfsimL1 reusable L1 guidance
//                           service (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  ROLE (AAP 0.3.3 -- Adapter):
//    This shim presents the EXACT read surface that AP_L1_Control consumes from
//    its `AP_AHRS &_ahrs`, but sourced from HOST-INJECTED state (North/East
//    position, East/North velocity, yaw, pitch) instead of live sensor fusion.
//    It re-plumbs the controller's *inputs* only; it changes no L1 mathematics
//    and is therefore behavior-preserving.
//
//  CENTRAL ENGINEERING DECISION (AAP 0.6.2):
//    AP_L1_Control binds a concrete `AP_AHRS &_ahrs` (AP_L1_Control.h:L79) and
//    the six accessors it calls are NON-VIRTUAL on AP_AHRS, so a plain subclass
//    cannot override them. Two behavior-preserving realizations exist and this
//    shim is designed to support BOTH:
//
//      * Option B (RECOMMENDED for the minimal-footprint standalone `.so`, and
//        the shape this file follows): provide a small shim exposing the six
//        read APIs plus injection setters, and bind the L1 translation unit to
//        the shim via a COMPILE-TIME INCLUDE SEAM (the standalone CMake build
//        controls include resolution so `_ahrs.<method>` binds to this shim's
//        identically named methods). This avoids linking the EKF/DCM stack.
//
//      * Option A (max-fidelity fallback): link the real AP_AHRS in external
//        mode and push the same injected state each execute(); in that case this
//        shim still provides the injection surface.
//
//    Because the include seam relies on name/signature IDENTITY, the six read
//    members below are DELIBERATELY name- and signature-identical to the
//    AP_AHRS accessors, and `yaw_sensor` is a public data member (mirroring
//    AP_AHRS) -- NOT a getter. Do not rename or re-type any of them.
//
//    The user example (AAP 0.7.2) is illustrative pseudocode (it references a
//    shim `AHRS` type and a two-argument Location form that do not exist in
//    stock ArduPilot); this shim realizes that example's SHAPE using concrete,
//    valid ArduPilot types.
//
//  CONSTRAINTS (AAP 0.7.1):
//    - Do NOT #include <AP_AHRS/AP_AHRS.h> and do NOT inherit from AP_AHRS (the
//      point of Option B is to avoid the EKF/DCM stack, and the non-virtual
//      accessors make subclassing ineffective anyway).
//    - No dependency on any vehicle firmware; no new third-party dependency.
//    - This header declares the contract only; the logic lives in the companion
//      translation unit AfsimL1_AHRS_Shim.cpp.
// ============================================================================

#include <AP_Common/Location.h>   // Location -- synthesized by get_location(Location&)
#include <AP_Math/AP_Math.h>      // Vector2f, radians()

/// @brief Lightweight, host-injectable stand-in for the AP_AHRS read surface
///        consumed by AP_L1_Control.
///
/// AfsimL1_AHRS_Shim stores state pushed in by an external host (for example the
/// AFSIM simulator) and re-exposes exactly the accessors that AP_L1_Control reads
/// from its AHRS reference. Guidance behavior is preserved because the controller
/// observes the same values it would read from a live AP_AHRS; only the data
/// source changes -- from live sensor fusion to explicit dependency injection.
///
/// The public read members are intentionally name- and signature-identical to
/// their AP_AHRS counterparts so that, under the compile-time include seam
/// (Option B above), the controller's `_ahrs.<method>` call sites bind to this
/// shim unchanged.
class AfsimL1_AHRS_Shim {
public:
    // ------------------------------------------------------------------------
    // Read surface -- signature-identical to the AP_AHRS accessors that
    // AP_L1_Control calls. These signatures MUST NOT change (see the include-
    // seam note in the CENTRAL ENGINEERING DECISION above).
    // ------------------------------------------------------------------------

    /// Mirrors AP_AHRS::get_location (AP_AHRS.h:L103); read by AP_L1_Control at
    /// AP_L1_Control.cpp:L230 (`if (_ahrs.get_location(_current_loc) == false)`).
    /// Synthesizes a Location by offsetting the fixed reference datum by the
    /// injected North/East position. Returns true because injected state is
    /// always considered valid (there is no "no-GPS" condition to report).
    bool get_location(Location &loc) const;

    /// Mirrors AP_AHRS::groundspeed_vector (AP_AHRS.h:L239); read at
    /// AP_L1_Control.cpp:L236. RETURNS A CONST REFERENCE to the stored member
    /// `_groundspeed_vector` -- never to a temporary/local -- so the returned
    /// reference can never dangle.
    const Vector2f &groundspeed_vector() const;

    /// Mirrors AP_AHRS::get_yaw_rad (AP_AHRS.h:L586); read at
    /// AP_L1_Control.cpp:L59/L61. Yaw in radians, measured clockwise from North.
    float get_yaw_rad() const;

    /// Mirrors AP_AHRS::get_pitch_rad (AP_AHRS.h:L585); read at
    /// AP_L1_Control.cpp:L91 (nav_roll_cd). Pitch in radians.
    float get_pitch_rad() const;

    /// Mirrors AP_AHRS::get_EAS2TAS (AP_AHRS.h:L160); read at
    /// AP_L1_Control.cpp:L126 (turn_distance) and L159 (loiter_radius). This is
    /// the equivalent-to-true airspeed ratio; it defaults to unity (1.0) when
    /// not injected (AAP 0.6.1: "injected or unit default").
    float get_EAS2TAS() const;

    /// Mirrors AP_AHRS::yaw_sensor (AP_AHRS.h:L619) -- a PUBLIC DATA MEMBER, not
    /// a method -- read at AP_L1_Control.cpp:L70/L72 (`_ahrs.yaw_sensor`). Integer
    /// yaw in centidegrees (degrees * 100). Kept in sync with get_yaw_rad() by
    /// set_yaw_cd().
    int32_t yaw_sensor = 0;

    // ------------------------------------------------------------------------
    // Injection setters -- the host pushes state in through these each step.
    // AfsimL1Behavior::set_state_ne(...) forwards to them (AAP 0.7.2).
    // ------------------------------------------------------------------------

    /// Store the vehicle's North/East position, in meters, relative to the
    /// reference datum. Consumed by get_location() to build the current Location.
    void set_location_NE(float n, float e);

    /// Store the ground-speed vector from host-supplied East/North components.
    /// NOTE the argument order is (velE, velN), matching the user example
    /// (AAP 0.7.2). ArduPilot's groundspeed-vector convention is
    /// Vector2f(x = North, y = East) -- AP_AHRS::_groundspeed_vector() returns
    /// getVelNED().xy() -- so the companion .cpp maps x = velN (North) and
    /// y = velE (East). This mapping keeps groundspeed_vector().length() and
    /// groundspeed_vector().angle() consistent with the yaw reference (measured
    /// from North) that AP_L1_Control compares against.
    void set_velocity_EN(float velE, float velN);

    /// Set the yaw from a centidegree value. Updates BOTH sources the controller
    /// reads: the radians accessor get_yaw_rad() (stored as
    /// radians(yaw_cd * 0.01f)) AND the public yaw_sensor member (int32_t)yaw_cd.
    void set_yaw_cd(float yaw_cd);

    /// Store the pitch, in radians, returned by get_pitch_rad().
    void set_pitch_rad(float pitch_rad);

private:
    // Injected North/East position (meters) relative to `_datum`. get_location()
    // offsets `_datum` by these components to synthesize the current Location.
    float _north_m = 0.0f;
    float _east_m  = 0.0f;

    // Stored ground-speed vector using ArduPilot's NE convention (x = North,
    // y = East). Held as a member so groundspeed_vector() can hand back a
    // reference to it without ever returning a dangling temporary.
    Vector2f _groundspeed_vector{};

    // Injected attitude.
    float _yaw_rad   = 0.0f;   // yaw in radians (from North); returned by get_yaw_rad()
    float _pitch_rad = 0.0f;   // pitch in radians; returned by get_pitch_rad()

    // Airspeed scale factor. Unit default (1.0) preserves behavior when the host
    // does not inject a value (AAP 0.6.1: "injected or unit default").
    float _eas2tas   = 1.0f;

    // Fixed reference datum. get_location() copies this and offsets it by
    // (_north_m, _east_m) so that every synthesized Location shares a single
    // origin, keeping Location::get_distance_NE() / get_bearing_to() geometry
    // self-consistent across the current position and the leg waypoints.
    Location _datum{};
};
