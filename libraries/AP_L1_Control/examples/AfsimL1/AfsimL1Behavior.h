#pragma once

/// @file    AfsimL1Behavior.h
/// @brief   Facade / Service layer for the AfsimL1 reusable L1-guidance service:
///          a small, task-oriented API over ArduPilot's AP_L1_Control.
///
/// This header belongs to the AfsimL1 service that extracts ArduPilot's L1
/// lateral-navigation guidance (AP_L1_Control) into a reusable, modular unit an
/// external host (e.g. the AFSIM simulation environment) can drive through a
/// stable C ABI. Within that service this file is the FACADE (a.k.a. service
/// layer): it presents a small, task-oriented interface --
/// set_leg_ne / set_state_ne / execute / get_roll_deg / get_lat_accel -- over
/// the richer AP_L1_Control controller, hiding the controller's internal wiring
/// behind dependency-injection seams.
///
/// AfsimL1Behavior COMPOSES (owns) three collaborators:
///   * an AfsimL1_AHRS_Shim adapter that supplies the state (position, velocity,
///     attitude) AP_L1_Control would normally read from live AP_AHRS fusion;
///   * an AP_L1_Control instance -- the unchanged guidance controller whose L1
///     mathematics are reused verbatim; and
///   * the current navigation leg endpoints (prev / next), as Locations.
///
/// The design is strictly BEHAVIOR-PRESERVING and ADDITIVE (AAP 0.1.1, 0.7.1):
/// the L1 guidance math is unchanged. The facade only re-plumbs the controller's
/// inputs (legs, state, timing) and outputs (roll, lateral acceleration) through
/// injection points instead of pulling them from the vehicle flight loop. This
/// realizes the SHAPE of the user-provided example (AAP 0.7.2), which is
/// illustrative pseudocode (it sketches a shim `AHRS` type, a two-argument
/// `Location{N, E}` form, and a parameterless AP_L1_Control constructor that do
/// not exist in stock ArduPilot); the concrete, verified techniques are applied
/// here and in the sibling translation unit AfsimL1Behavior.cpp.
///
/// ---------------------------------------------------------------------------
/// CENTRAL ENGINEERING DECISION  (see Agent Action Plan section 0.6.2)
/// ---------------------------------------------------------------------------
/// AP_L1_Control binds a *concrete* reference `AP_AHRS &_ahrs`
/// (AP_L1_Control.h:L84), and the six accessors it calls on that reference are
/// NON-VIRTUAL on AP_AHRS, so ordinary subclassing cannot decouple the state.
/// The service therefore supplies its own AfsimL1_AHRS_Shim, whose read methods
/// are name/signature-identical to those AP_AHRS accessors:
///
///   * Option B (RECOMMENDED, minimal-footprint standalone `.so`): the L1
///     translation unit is bound to the shim through a COMPILE-TIME INCLUDE SEAM
///     controlled by the standalone build (CMakeLists.txt), so every
///     `_ahrs.<method>` call inside AP_L1_Control resolves to the shim's
///     identically-named member. This avoids linking the EKF/DCM stack.
///   * Option A (maximum-fidelity fallback): link the real AP_AHRS in an
///     external-navigation mode and push the same injected state into it.
///
/// Either way this facade OWNS the shim and pushes injected state into it before
/// delegating to the controller. Because construction wires the controller to
/// the shim (conceptually `AP_L1_Control _l1{_ahrs_shim, nullptr}`) the member
/// DECLARATION ORDER below is significant: the shim is declared before the
/// controller so it is fully constructed before the controller's
/// member-initializer references it. See AfsimL1Behavior.cpp for the
/// member-initializer list that performs this construction.
///
/// ---------------------------------------------------------------------------
/// Timing seam dependency (already present on the wrapped controller)
/// ---------------------------------------------------------------------------
/// AP_L1_Control carries an additive, default-off `set_update_dt(float)` seam
/// (AP_L1_Control.h:L42) plus its private override state. execute() forwards the
/// host-supplied dt through that seam so the host (AFSIM), not AP_HAL::micros(),
/// owns the timebase. This facade only *uses* the seam; it neither re-declares
/// nor modifies AP_L1_Control (AAP 0.7.1).
///
/// Dependencies: this file depends only on the wrapped controller header, the
/// sibling shim header, and the Location type. No vehicle-firmware dependency
/// and no new third-party dependency are introduced (all includes are additive).

#include <AP_L1_Control/AP_L1_Control.h>   // AP_L1_Control: the wrapped guidance controller (composed as _l1)
#include "AfsimL1_AHRS_Shim.h"             // AfsimL1_AHRS_Shim: host-driven AP_AHRS read-surface adapter (composed as _ahrs_shim)
#include <AP_Common/Location.h>            // Location: type of the prev/next navigation-leg endpoints

/// @class AfsimL1Behavior
/// @brief Reusable L1 lateral-navigation guidance service facade.
///
/// A host constructs one AfsimL1Behavior, optionally seeds a leg with init(),
/// then each control step: injects the current leg (set_leg_ne) and platform
/// state (set_state_ne), advances the guidance (execute(dt)), and reads the
/// commanded outputs (get_roll_deg / get_lat_accel). The public method names are
/// FIXED: the C ABI translation unit (l1_c_api.cpp) and the user example bind to
/// exactly these signatures.
///
/// Ownership / copy semantics: the composed AP_L1_Control is declared
/// CLASS_NO_COPY, so AfsimL1Behavior is implicitly non-copyable and
/// non-move-assignable -- the correct behavior for a stateful, resource-owning
/// service facade. Instances are created on the heap by the C ABI
/// (`new AfsimL1Behavior()`) and released with the implicitly-generated
/// destructor.
class AfsimL1Behavior {
public:
    /// Construct the service facade.
    ///
    /// A user-provided constructor is REQUIRED: the composed AP_L1_Control has no
    /// default constructor (its only constructor is
    /// `AP_L1_Control(AP_AHRS&, const AP_TECS*)`) and is CLASS_NO_COPY, so the
    /// implicitly-declared default constructor would be deleted. The definition
    /// lives in AfsimL1Behavior.cpp, where the member-initializer list wires the
    /// controller to the owned shim and a null TECS
    /// (conceptually `_l1{_ahrs_shim, nullptr}`) and seeds the L1 tuning
    /// parameters. It leaves prev/next as zeroed Locations until a leg is set.
    AfsimL1Behavior();

    /// Seed a simple default navigation leg.
    ///
    /// Mirrors the user example's "initialize a simple leg": prev is placed at
    /// the datum (0 N, 0 E) and next is offset 500 m along the East axis
    /// (0 N, 500 E). Provided so a host can exercise the service before wiring in
    /// real routing. Calling init() is optional; set_leg_ne() overrides whatever
    /// init() seeded.
    void init();

    /// Advance the L1 guidance computation by one control step.
    ///
    /// The host owns the timebase. The implementation forwards the injected state
    /// into the owned AHRS shim, hands @p dt_seconds to the controller via
    /// `_l1.set_update_dt((float)dt_seconds)` (the additive, default-off timing
    /// seam), then drives one guidance update with
    /// `_l1.update_waypoint(_prev, _next)`. After this call the commanded outputs
    /// are available from get_roll_deg() and get_lat_accel().
    ///
    /// @param dt_seconds control-step duration in seconds (host-supplied).
    void execute(double dt_seconds);

    /// Set the active navigation leg from previous and next waypoints, each
    /// expressed as North/East offsets (metres) relative to the service datum.
    ///
    /// The endpoints are built from the datum plus the supplied offsets (NOT via
    /// a two-argument `Location{N, E}` literal, which does not exist on the stock
    /// Location type). The values are stored into the prev/next leg members and
    /// consumed by the next execute().
    ///
    /// @param prevN previous-waypoint North offset (m).
    /// @param prevE previous-waypoint East offset (m).
    /// @param nextN next-waypoint North offset (m).
    /// @param nextE next-waypoint East offset (m).
    void set_leg_ne(double prevN, double prevE, double nextN, double nextE);

    /// Inject the current platform state into the service.
    ///
    /// Forwards the arguments to the four owned-shim injectors that back the
    /// AP_AHRS read surface the controller consumes:
    ///   * set_location_NE(n, e)      -> get_location()
    ///   * set_velocity_EN(velE, velN)-> groundspeed_vector()
    ///   * set_yaw_cd(yaw_cd)         -> get_yaw_rad() and the yaw_sensor field
    ///   * set_pitch_rad(pitch_rad)   -> get_pitch_rad()
    /// The parameter order matches the user example and the C ABI's
    /// L1_SetStateNE entry point exactly.
    ///
    /// @param n         North position offset from the datum (m).
    /// @param e         East position offset from the datum (m).
    /// @param velE      East ground-velocity component (m/s).
    /// @param velN      North ground-velocity component (m/s).
    /// @param yaw_cd    yaw / heading in centidegrees.
    /// @param pitch_rad pitch in radians.
    void set_state_ne(double n, double e, double velE, double velN, double yaw_cd, double pitch_rad);

    /// Read the most recently computed commanded roll (bank) angle.
    ///
    /// Converts the controller's centidegree output to degrees:
    /// `static_cast<double>(_l1.nav_roll_cd() / 100.0f)`. Declared const because
    /// it only reads the controller's last-computed output (nav_roll_cd() is a
    /// const accessor on AP_L1_Control).
    ///
    /// @return commanded roll angle in degrees.
    double get_roll_deg() const;

    /// Read the most recently computed lateral acceleration demand.
    ///
    /// Returns `static_cast<double>(_l1.lateral_acceleration())` (m/s^2, +ve to
    /// the right). Declared const because it only reads the controller's
    /// last-computed output (lateral_acceleration() is a const accessor on
    /// AP_L1_Control).
    ///
    /// @return lateral acceleration in m/s^2.
    double get_lat_accel() const;

private:
    // -------------------------------------------------------------------------
    // Composed collaborators. DECLARATION ORDER IS SIGNIFICANT: members are
    // constructed in declaration order, and _l1's member-initializer (in
    // AfsimL1Behavior.cpp) references _ahrs_shim as its AHRS argument, so
    // _ahrs_shim MUST be declared -- and therefore constructed -- first.
    // -------------------------------------------------------------------------

    /// Host-driven AP_AHRS read-surface adapter. Populated by set_state_ne() and
    /// read back by the composed controller (see the central engineering
    /// decision above). Declared first so it exists before _l1 is constructed
    /// against it.
    AfsimL1_AHRS_Shim _ahrs_shim;

    /// The wrapped L1 guidance controller, reused unchanged. Constructed in the
    /// .cpp against _ahrs_shim (as the AHRS) and a null TECS pointer; its L1
    /// mathematics are behavior-preserving.
    AP_L1_Control _l1;

    /// Previous waypoint of the current navigation leg. Default-constructed to a
    /// zeroed Location (the relative-navigation frame origin) until set_leg_ne()
    /// or init() supplies a leg.
    Location _prev{};

    /// Next (target) waypoint of the current navigation leg. Default-constructed
    /// to a zeroed Location until set_leg_ne() or init() supplies a leg.
    Location _next{};
};
