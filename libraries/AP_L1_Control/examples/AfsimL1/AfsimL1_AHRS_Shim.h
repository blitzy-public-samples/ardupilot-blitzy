#pragma once

/// @file    AfsimL1_AHRS_Shim.h
/// @brief   Adapter layer for the AfsimL1 reusable L1-guidance service:
///          a lightweight, host-driven stand-in for AP_AHRS.
///
/// This header belongs to the AfsimL1 service that extracts ArduPilot's L1
/// lateral-navigation guidance (AP_L1_Control) into a reusable module an
/// external host (e.g. the AFSIM simulator) can drive through a stable C ABI.
/// Within that service this file is the ADAPTER (a.k.a. state shim): it presents
/// the EXACT read surface that AP_L1_Control consumes from its `AP_AHRS &_ahrs`,
/// but the values are pushed in by the host each control step instead of being
/// fused from live sensors. It is strictly behavior-preserving — it only
/// re-plumbs the controller's inputs and changes no L1 mathematics.
///
/// ---------------------------------------------------------------------------
/// CENTRAL ENGINEERING DECISION  (see Agent Action Plan section 0.6.2)
/// ---------------------------------------------------------------------------
/// AP_L1_Control binds a *concrete* reference `AP_AHRS &_ahrs`
/// (AP_L1_Control.h:L84), and the six accessors it calls on that reference are
/// NON-VIRTUAL on AP_AHRS. Because they are non-virtual, a plain subclass of
/// AP_AHRS cannot override them — so the state cannot be decoupled by ordinary
/// inheritance. Two behavior-preserving realizations exist, and this shim is
/// deliberately shaped to support BOTH:
///
///   * Option B (RECOMMENDED for a minimal-footprint standalone `.so`, and the
///     shape this file follows): provide a small shim that presents the six
///     read APIs plus injection setters, and bind the L1 translation unit to
///     this shim through a COMPILE-TIME INCLUDE SEAM. The standalone build
///     (CMakeLists.txt) controls include resolution so that every `_ahrs.<x>`
///     call inside AP_L1_Control binds to this shim's identically-named
///     members. This avoids linking the EKF/DCM sensor-fusion stack entirely.
///
///   * Option A (maximum-fidelity fallback): link the real AP_AHRS in an
///     external-navigation mode and push the same injected state into it every
///     execute(); in that arrangement this shim still supplies the injection
///     surface used to feed the real object.
///
/// The user-provided example (AAP section 0.7.2) is illustrative pseudocode: it
/// references a shim `AHRS` type, a non-existent two-argument `Location{N, E}`
/// form, and a parameterless AP_L1_Control constructor. We realize its SHAPE
/// through the concrete, verified techniques documented here.
///
/// CONSEQUENCE FOR THIS HEADER: the read methods below are declared with names
/// and signatures that are byte-identical to the AP_AHRS accessors AP_L1_Control
/// actually calls (verified against libraries/AP_AHRS/AP_AHRS.h). They MUST NOT
/// be renamed or have their signatures altered, or the include-seam binding in
/// Option B will fail to compile.
///
/// ---------------------------------------------------------------------------
/// Verified AP_AHRS contract mirrored by this shim
/// ---------------------------------------------------------------------------
///   AP_AHRS member (verified)                         Consumed at
///   ------------------------------------------------  --------------------------
///   bool     get_location(Location&) const            AP_L1_Control.cpp:L260
///   const Vector2f& groundspeed_vector() const        AP_L1_Control.cpp:L266
///   float    get_yaw_rad() const                      AP_L1_Control.cpp:L59,L61
///   float    get_pitch_rad() const                    AP_L1_Control.cpp:L91
///   float    get_EAS2TAS() const                       AP_L1_Control.cpp:L150
///   int32_t  yaw_sensor            (PUBLIC FIELD)      AP_L1_Control.cpp:L70,L72
///
/// Two verified caveats the pseudocode cannot express:
///   1. groundspeed_vector() returns a CONST REFERENCE. The shim therefore
///      stores a Vector2f member and returns a reference to it; it must never
///      return a reference to a temporary/local (that would dangle).
///   2. yaw_sensor is a PUBLIC DATA MEMBER on AP_AHRS, not a getter. It is
///      exposed here as a public field and kept in sync by set_yaw_cd().
///
/// Design constraints honored (AAP section 0.7.1):
///   * <AP_AHRS/AP_AHRS.h> is intentionally NOT included and this class does
///     NOT inherit from AP_AHRS — the whole point of Option B is to avoid the
///     EKF/DCM stack, and non-virtual accessors make subclassing ineffective.
///   * Only method DECLARATIONS live here; the definitions (including the
///     Location construction performed by get_location) live in the sibling
///     translation unit AfsimL1_AHRS_Shim.cpp.
///   * No dependency on any vehicle firmware and no new third-party dependency.

#include <AP_Common/Location.h>   // Location: out-parameter type of get_location()
#include <AP_Math/AP_Math.h>      // Vector2f (groundspeed vector), radians() (used in .cpp)

/// @class AfsimL1_AHRS_Shim
/// @brief Host-injected replacement for the AP_AHRS read surface used by
///        AP_L1_Control.
///
/// The host calls the set_*() injectors once per control step (typically routed
/// through AfsimL1Behavior::set_state_ne) and the composed AP_L1_Control instance
/// then reads the same values back through the AP_AHRS-shaped accessors. A
/// default-constructed shim is fully defined: position and attitude read as zero
/// and the equivalent-to-true airspeed ratio reads as the neutral value 1.0.
class AfsimL1_AHRS_Shim {
public:
    // -------------------------------------------------------------------------
    // AP_AHRS-compatible read surface.
    //
    // Every signature below is byte-identical to the corresponding AP_AHRS
    // accessor so that AP_L1_Control's `_ahrs.<x>` call sites bind to this shim
    // unchanged under the Option B include seam. Definitions are provided in
    // AfsimL1_AHRS_Shim.cpp.
    // -------------------------------------------------------------------------

    /// Return the current vehicle position as a Location.
    /// Mirrors AP_AHRS::get_location (AP_AHRS.h:L103); consumed at
    /// AP_L1_Control.cpp:L260 as `if (_ahrs.get_location(_current_loc) == false)`.
    /// The implementation constructs @p loc from the injected North/East offset
    /// (set_location_NE) relative to a fixed datum. It returns true because the
    /// host-supplied position is always considered valid.
    /// @param[out] loc  Receives the current location.
    /// @return          true when a location is available (always, for the shim).
    bool get_location(Location &loc) const;

    /// Return the ground-speed vector, in meters/second.
    /// Mirrors AP_AHRS::groundspeed_vector (AP_AHRS.h:L239); consumed at
    /// AP_L1_Control.cpp:L266. Following ArduPilot's convention the vector is
    /// oriented (x = North, y = East); the controller uses length() as ground
    /// speed and angle() (== atan2(East, North)) compared against get_yaw().
    /// @return  Const reference to the stored ground-speed vector member. The
    ///          reference stays valid for the lifetime of this shim and is
    ///          never bound to a temporary.
    const Vector2f &groundspeed_vector() const;

    /// Return the current yaw (heading) in radians.
    /// Mirrors AP_AHRS::get_yaw_rad (AP_AHRS.h:L586); consumed at
    /// AP_L1_Control.cpp:L59 and L61. Fed by set_yaw_cd().
    float get_yaw_rad() const;

    /// Return the current pitch in radians.
    /// Mirrors AP_AHRS::get_pitch_rad (AP_AHRS.h:L585); consumed at
    /// AP_L1_Control.cpp:L91. Fed by set_pitch_rad().
    float get_pitch_rad() const;

    /// Return the equivalent-to-true airspeed scaling ratio (EAS2TAS).
    /// Mirrors AP_AHRS::get_EAS2TAS (AP_AHRS.h:L160); consumed at
    /// AP_L1_Control.cpp:L150. Defaults to the neutral value 1.0 (sea-level
    /// equivalence) because the AfsimL1 injection surface does not currently
    /// supply an air-density correction.
    float get_EAS2TAS() const;

    /// Current yaw expressed in centidegrees.
    ///
    /// Mirrors the PUBLIC data member AP_AHRS::yaw_sensor (AP_AHRS.h:L619), read
    /// directly (not through a getter) at AP_L1_Control.cpp:L70 and L72. It is a
    /// public field so those call sites bind unchanged; it is kept consistent
    /// with get_yaw_rad() by set_yaw_cd(). Initialized to 0 so a default-
    /// constructed shim is well-defined.
    int32_t yaw_sensor = 0;

    // -------------------------------------------------------------------------
    // Injection setters.
    //
    // The host pushes the platform state in through these each control step;
    // AfsimL1Behavior::set_state_ne forwards to them (AAP section 0.7.2).
    // Definitions live in AfsimL1_AHRS_Shim.cpp.
    // -------------------------------------------------------------------------

    /// Inject the vehicle North/East position, in meters, relative to the datum.
    /// Stored and later used by get_location() to build the reported Location.
    /// @param n  North offset from the datum, in meters.
    /// @param e  East offset from the datum, in meters.
    void set_location_NE(float n, float e);

    /// Inject the ground velocity, in meters/second.
    ///
    /// The arguments follow the user example's order (East first, then North).
    /// They are stored into the ground-speed vector using ArduPilot's
    /// (x = North, y = East) convention, i.e. the implementation performs
    /// `_groundspeed_vector = Vector2f(velN, velE)` so that groundspeed_vector()
    /// returns a vector whose length() is the ground speed and whose angle()
    /// (atan2(East, North)) is comparable to get_yaw().
    /// @param velE  East velocity component, in meters/second.
    /// @param velN  North velocity component, in meters/second.
    void set_velocity_EN(float velE, float velN);

    /// Inject the vehicle yaw, supplied in centidegrees.
    ///
    /// Updates BOTH representations the controller reads: the radian source
    /// returned by get_yaw_rad() (stored as radians(yaw_cd * 0.01f)) and the
    /// centidegree public field yaw_sensor (stored as the truncated integer
    /// value). Keeping both in sync is required because AP_L1_Control reads each
    /// on different paths.
    /// @param yaw_cd  Yaw in centidegrees (e.g. 4500 == 45 degrees).
    void set_yaw_cd(float yaw_cd);

    /// Inject the vehicle pitch, in radians. Returned verbatim by get_pitch_rad().
    /// @param pitch_rad  Pitch angle, in radians.
    void set_pitch_rad(float pitch_rad);

private:
    // Injected North/East position (meters) relative to the datum. Consumed by
    // get_location() when constructing the reported Location.
    float _north_m = 0.0f;
    float _east_m  = 0.0f;

    // Stored ground-speed vector (x = North, y = East), in meters/second.
    // groundspeed_vector() returns a reference to THIS member, so it must be a
    // stable data member and never a temporary.
    Vector2f _groundspeed_vector{};

    // Injected attitude, in radians.
    float _yaw_rad   = 0.0f;
    float _pitch_rad = 0.0f;

    // Equivalent-to-true airspeed ratio. The neutral default of 1.0 means "no
    // air-density correction", matching the injected-or-unit-default policy in
    // AAP section 0.6.1 (there is no EAS2TAS injector in the current surface).
    float _eas2tas = 1.0f;

    // Fixed reference datum from which the injected North/East offset is applied
    // to produce the Location reported by get_location(). A default-constructed
    // (zeroed) Location is a well-defined origin for the relative-navigation
    // frame the AfsimL1 service operates in.
    Location _datum{};
};
