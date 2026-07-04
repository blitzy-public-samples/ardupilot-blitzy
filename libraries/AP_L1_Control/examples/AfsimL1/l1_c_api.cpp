/// @file    l1_c_api.cpp
/// @brief   Stable C Application Binary Interface (ABI) for the AfsimL1
///          navigation-guidance service -- implementation and exported symbols.
///
/// This translation unit is the Anti-Corruption / ABI-boundary LAYER of the
/// AfsimL1 service (see the Agent Action Plan, sections 0.3.3 and 0.6.4). It
/// provides the concrete definitions for the eight `extern "C"` entry points
/// declared in l1_c_api.h and, crucially, is the ONE place where the concrete
/// type behind the public opaque handle is defined.
///
/// Why a C boundary (design rationale):
///   The C++ ABI is not stable across compilers or even compiler versions --
///   name mangling, virtual-table layout, RTTI, and exception representation
///   all differ, and the C++ ABI is a superset of the C ABI. By confining the
///   public surface to a flat set of `extern "C"` free functions plus an opaque
///   `void*` handle, an external host such as the AFSIM simulation environment
///   can load `libafsim_l1.so` at runtime regardless of its own toolchain or
///   C++ standard library. Only plain scalar types (`void*` and `double`) ever
///   cross this boundary; no C++ type, reference, or class is exposed.
///
/// Three mechanisms combine to realize that isolation here:
///   1. `extern "C"` linkage      -> no C++ name mangling on the exports.
///   2. an opaque handle          -> `struct L1_Context` (defined below) wraps
///                                   an `AfsimL1Behavior*`, so the C++ facade
///                                   type never appears in the public header.
///   3. `visibility("default")`   -> combined with `-fvisibility=hidden` in the
///                                   shared-library build, EXACTLY these eight
///                                   symbols are exported from the `.so`.
///
/// Layering / dependencies (Agent Action Plan section 0.4.2):
///   This file includes ONLY the facade header (which transitively provides the
///   wrapped controller, the AHRS shim, and the Location type) plus its own
///   declaration header. It deliberately does NOT include AfsimL1_AHRS_Shim.h or
///   AP_L1_Control.h directly -- those arrive through AfsimL1Behavior.h -- and
///   it introduces no vehicle-firmware dependency and no new third-party
///   dependency. Every call delegates 1:1 to an AfsimL1Behavior method.
///
/// Ownership / lifetime contract:
///   L1_Create() allocates BOTH the opaque context and the underlying
///   AfsimL1Behavior; L1_Destroy() releases BOTH (the behavior first, then the
///   context), leaving no leak and performing no double-free. Every other entry
///   point is null-safe: a NULL handle is ignored by the void-returning
///   functions and yields 0.0 from the getters.

// The facade is the ONLY service type this boundary wraps. Including it also
// transitively provides everything the facade composes (AP_L1_Control, the AHRS
// shim, and Location), so no other service header is needed here.
#include "AfsimL1Behavior.h"

// Include the public declarations these definitions implement. This keeps the
// header and the implementation in agreement (identical signatures and C
// linkage) and satisfies -Werror=missing-declarations: each exported function
// below has a preceding declaration from this header.
#include "l1_c_api.h"

// std::nothrow. L1_Create() below allocates with the non-throwing form of
// operator new so that an allocation failure is reported by RETURNING NULL --
// the contract documented in l1_c_api.h -- instead of letting a std::bad_alloc
// exception escape across the C ABI boundary, which would be undefined for a C
// caller (exception propagation is not part of the C ABI).
#include <new>

// ---------------------------------------------------------------------------
// C ABI boundary.
//
// Everything below lives inside a single `extern "C"` block so the eight entry
// points are exported with C linkage (no name mangling), matching the
// declarations in l1_c_api.h exactly. Each definition additionally carries the
// raw `__attribute__((visibility("default")))` -- the authoritative form
// required by Agent Action Plan section 0.6.4 -- so that, under a shared-library
// build compiled with -fvisibility=hidden, only these eight functions are
// exported from libafsim_l1.so.
// ---------------------------------------------------------------------------
extern "C" {

/// @brief Concrete type behind the public opaque `void*` handle.
///
/// This is the ONLY place the concrete handle type is defined; it is never
/// exposed in l1_c_api.h, so callers only ever hold a `void*`. It simply owns a
/// pointer to the C++ service facade. Defining it inside the `extern "C"` block
/// gives the tag C language linkage, which is harmless because the type is used
/// solely within this translation unit.
struct L1_Context { AfsimL1Behavior* self; };

/// Construct a new AfsimL1 service instance.
///
/// Allocates the opaque context and the underlying AfsimL1Behavior together and
/// returns the context as an opaque handle, or NULL if either allocation fails.
/// Both allocations use the non-throwing form of new, so allocation failure is
/// reported via a NULL return (the contract documented in l1_c_api.h) rather
/// than by throwing a C++ exception across the C ABI; any partial allocation is
/// unwound before returning NULL, so the failure path leaks nothing. On success
/// the returned handle must eventually be released with L1_Destroy() to avoid
/// leaking the underlying C++ object.
__attribute__((visibility("default"))) void* L1_Create() {
    // Honor the documented NULL-on-allocation-failure contract (l1_c_api.h) and
    // never allow a C++ exception to cross the C ABI: allocate BOTH objects with
    // the non-throwing form of new, check each result, and unwind any partial
    // allocation before returning. The allocation order (behavior first, then
    // context) mirrors the release order in L1_Destroy().
    auto* self = new (std::nothrow) AfsimL1Behavior();
    if (self == nullptr) {
        return nullptr;
    }
    auto* ctx = new (std::nothrow) L1_Context{self};
    if (ctx == nullptr) {
        // The context allocation failed after the behavior succeeded; release
        // the behavior so the failure path leaks nothing, then report failure.
        delete self;
        return nullptr;
    }
    return ctx;
}

/// Destroy a service instance previously returned by L1_Create().
///
/// Null-safe. Releases the underlying behavior first, then the context itself,
/// mirroring the allocation order in L1_Create() so there is no leak and no
/// double-free. After this call the handle is invalid and must not be reused.
__attribute__((visibility("default"))) void L1_Destroy(void* handle) {
    if (!handle) return;
    auto* ctx = static_cast<L1_Context*>(handle);
    delete ctx->self;
    delete ctx;
}

/// Initialize the service instance (seed the initial leg / platform state).
///
/// Null-safe. Delegates to AfsimL1Behavior::init().
__attribute__((visibility("default"))) void L1_Init(void* handle) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->init();
}

/// Advance the L1 guidance computation by one control step.
///
/// Null-safe. Delegates to AfsimL1Behavior::execute(); the host owns the
/// timebase and supplies @p dt_seconds directly.
__attribute__((visibility("default"))) void L1_Execute(void* handle, double dt_seconds) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->execute(dt_seconds);
}

/// Set the active navigation leg from previous and next waypoints, each
/// expressed as North/East offsets (metres) from the service datum.
///
/// Null-safe. Delegates to AfsimL1Behavior::set_leg_ne().
__attribute__((visibility("default"))) void L1_SetLegNE(void* handle, double prevN, double prevE, double nextN, double nextE) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->set_leg_ne(prevN, prevE, nextN, nextE);
}

/// Inject the current platform state into the service.
///
/// Null-safe. Delegates to AfsimL1Behavior::set_state_ne(); the parameter order
/// (n, e, velE, velN, yaw_cd, pitch_rad) matches the facade and the user example
/// exactly.
__attribute__((visibility("default"))) void L1_SetStateNE(void* handle, double n, double e, double velE, double velN, double yaw_cd, double pitch_rad) {
    if (!handle) return;
    static_cast<L1_Context*>(handle)->self->set_state_ne(n, e, velE, velN, yaw_cd, pitch_rad);
}

/// Read the most recently computed commanded roll (bank) angle, in degrees.
///
/// Returns 0.0 for a NULL handle; otherwise delegates to
/// AfsimL1Behavior::get_roll_deg().
__attribute__((visibility("default"))) double L1_GetRollDeg(void* handle) {
    if (!handle) return 0.0;
    return static_cast<L1_Context*>(handle)->self->get_roll_deg();
}

/// Read the most recently computed lateral acceleration demand, in m/s^2.
///
/// Returns 0.0 for a NULL handle; otherwise delegates to
/// AfsimL1Behavior::get_lat_accel().
__attribute__((visibility("default"))) double L1_GetLatAccel(void* handle) {
    if (!handle) return 0.0;
    return static_cast<L1_Context*>(handle)->self->get_lat_accel();
}

} // extern "C"
