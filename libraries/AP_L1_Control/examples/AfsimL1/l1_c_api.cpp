// ============================================================================
//  l1_c_api.cpp  --  Anti-Corruption / ABI-boundary implementation for the
//                    AfsimL1 reusable L1 guidance service
//                    (libraries/AP_L1_Control/examples/AfsimL1).
// ----------------------------------------------------------------------------
//  ROLE (AAP 0.3.3, 0.6.4 -- Anti-Corruption / ABI Boundary):
//    This translation unit implements the flat, mangling-free C entry points
//    declared in l1_c_api.h. It wraps a single AfsimL1Behavior instance inside
//    an opaque `struct L1_Context` so that NO C++ type ever crosses the public
//    boundary: an external host -- for example the AFSIM simulator -- holds
//    nothing but a `void*` and calls plain C functions that take/return only
//    C scalars (void* and double).
//
//    That is what makes libafsim_l1.so ABI-stable: C++ name mangling,
//    virtual-table layout, RTTI, and exception representation all differ across
//    compilers and compiler versions, whereas the flat C ABI is a stable
//    superset every toolchain agrees upon. Consequently AFSIM can bind the
//    shared library at run time regardless of the compiler or C++ standard
//    library it was itself built with (AAP 0.3.2, 0.6.4).
//
//  EXPORTED SURFACE (AAP 0.6.4, 0.7.1):
//    Exactly EIGHT symbols are exported, each annotated with
//    `__attribute__((visibility("default")))`. Built with `-fvisibility=hidden`
//    (the standalone CMake target), every other symbol -- including the entire
//    composed AP_L1_Control guidance stack -- stays hidden, so
//    `nm -D --defined-only libafsim_l1.so` lists precisely these eight names:
//        L1_Create  L1_Destroy  L1_Init  L1_Execute
//        L1_SetLegNE  L1_SetStateNE  L1_GetRollDeg  L1_GetLatAccel
//
//  SHAPE (AAP 0.7.2):
//    The structure below reproduces the user "initialize a simple leg" example's
//    "Mock C ABI" one-to-one: the opaque `struct L1_Context { AfsimL1Behavior*
//    self; }` and the eight exports that null-guard the handle and delegate 1:1
//    to the facade (init / execute / set_leg_ne / set_state_ne / get_roll_deg /
//    get_lat_accel).
//
//  INCLUDES (AAP 0.4.2 -- additive, facade header only):
//    Only "AfsimL1Behavior.h" (the sole service dependency this file needs) and
//    "l1_c_api.h" (to keep the public declarations and these definitions in
//    sync -- also required so the definitions have a prior declaration under
//    -Werror=missing-declarations). The AHRS shim and AP_L1_Control.h are NOT
//    included directly here; they arrive transitively through AfsimL1Behavior.h.
//
//  CONSTRAINTS (AAP 0.6.4, 0.7.1):
//    - Every function null-guards the incoming `void*` handle (returns early for
//      the void functions, returns 0.0 for the double getters), so a NULL or
//      already-destroyed handle can never be dereferenced.
//    - `struct L1_Context` is the ONLY place the concrete wrapper type is
//      defined; it is never exposed in the header.
//    - Memory: L1_Create allocates BOTH the context and the behavior; L1_Destroy
//      releases BOTH (the behavior first, then the context) -- no leak, no
//      double-free.
//    - No vehicle-firmware dependency; no new third-party dependency.
// ============================================================================

#include "AfsimL1Behavior.h"   // AfsimL1Behavior -- the C++ facade wrapped by this ABI
#include "l1_c_api.h"          // public C declarations (keeps decls/defs in sync)

// ----------------------------------------------------------------------------
// The public interface has C linkage: the definitions below live inside an
// `extern "C"` block so their symbol names are emitted unmangled, matching the
// declarations in l1_c_api.h byte-for-byte. `struct L1_Context` is defined here
// (and only here) as the concrete type behind the opaque `void*` handle callers
// hold.
// ----------------------------------------------------------------------------
extern "C" {

/// Concrete type behind the opaque handle returned by L1_Create().
///
/// A trivially-copyable aggregate that owns a single pointer to the C++ facade.
/// Callers never see this type -- they hold only a `void*` -- which is exactly
/// what keeps the boundary free of any C++ type (class, reference, template, or
/// exception). Defined here rather than in the header so the facade type stays
/// completely hidden from consumers of l1_c_api.h.
struct L1_Context { AfsimL1Behavior* self; };

/// Create a new AfsimL1 service instance.
///
/// Allocates the opaque context together with its underlying AfsimL1Behavior
/// facade and returns an opaque handle to it. The returned handle must later be
/// released with L1_Destroy() to avoid leaking memory.
///
/// @return  Opaque handle (non-NULL on success).
__attribute__((visibility("default"))) void* L1_Create() {
    auto* ctx = new L1_Context{new AfsimL1Behavior()};
    return ctx;
}

/// Destroy a service instance previously returned by L1_Create().
///
/// Releases the underlying facade first, then the opaque context. Passing NULL
/// is a safe no-op. After this call the handle is invalid and must not be reused.
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
__attribute__((visibility("default"))) void L1_Destroy(void* handle) {
    if (!handle) return;
    auto* ctx = static_cast<L1_Context*>(handle);
    delete ctx->self;
    delete ctx;
}

/// Initialise the service instance (seed a simple default leg / state).
///
/// Passing NULL is a safe no-op.
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
__attribute__((visibility("default"))) void L1_Init(void* handle) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->init();
}

/// Advance the guidance by one control step using a host-supplied time delta.
///
/// The host owns the timebase: @p dt_seconds is injected directly rather than
/// the controller computing its own delta from the hardware clock. Passing NULL
/// is a safe no-op.
///
/// @param handle      Opaque handle returned by L1_Create(), or NULL.
/// @param dt_seconds  Control-step interval in seconds (e.g. 0.02 for 50 Hz).
__attribute__((visibility("default"))) void L1_Execute(void* handle, double dt_seconds) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->execute(dt_seconds);
}

/// Set the active navigation leg from the previous and next waypoints.
///
/// Coordinates are North/East offsets in metres relative to the service datum.
/// Passing NULL is a safe no-op.
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
/// @param prevN   Previous waypoint, North offset (metres).
/// @param prevE   Previous waypoint, East  offset (metres).
/// @param nextN   Next     waypoint, North offset (metres).
/// @param nextE   Next     waypoint, East  offset (metres).
__attribute__((visibility("default"))) void L1_SetLegNE(void* handle, double prevN, double prevE, double nextN, double nextE) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->set_leg_ne(prevN, prevE, nextN, nextE);
}

/// Inject the current platform state consumed by the guidance controller.
///
/// Supplies the position, velocity, and attitude the vehicle loop would
/// normally read from AHRS sensor fusion. Passing NULL is a safe no-op.
///
/// @param handle     Opaque handle returned by L1_Create(), or NULL.
/// @param n          Position North offset from datum (metres).
/// @param e          Position East  offset from datum (metres).
/// @param velE       Ground velocity, East  component (m/s).
/// @param velN       Ground velocity, North component (m/s).
/// @param yaw_cd     Heading / yaw                     (centidegrees).
/// @param pitch_rad  Pitch                             (radians).
__attribute__((visibility("default"))) void L1_SetStateNE(void* handle, double n, double e, double velE, double velN, double yaw_cd, double pitch_rad) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->set_state_ne(n, e, velE, velN, yaw_cd, pitch_rad);
}

/// Read the roll command produced by the most recent L1_Execute().
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
/// @return        Commanded roll angle in degrees, or 0.0 if @p handle is NULL.
__attribute__((visibility("default"))) double L1_GetRollDeg(void* handle) {
    if (!handle) return 0.0;
    return static_cast<L1_Context*>(handle)->self->get_roll_deg();
}

/// Read the lateral-acceleration demand produced by the most recent L1_Execute().
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
/// @return        Demanded lateral acceleration in m/s^2, or 0.0 if @p handle is NULL.
__attribute__((visibility("default"))) double L1_GetLatAccel(void* handle) {
    if (!handle) return 0.0;
    return static_cast<L1_Context*>(handle)->self->get_lat_accel();
}

} // extern "C"
