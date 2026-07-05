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
//  HANDLE SAFETY (memory-safety hardening):
//    A C ABI cannot trust the `void*` a foreign host passes back. A NULL guard
//    alone does NOT protect against a *stale* non-NULL handle -- one already
//    passed to L1_Destroy (double-free) or reused after destruction
//    (use-after-free). Dereferencing such a pointer is undefined behaviour.
//    This translation unit therefore validates every handle against a
//    process-local registry of currently-live contexts BEFORE any dereference:
//      * L1_Create registers the new context and stamps it with a validity
//        cookie (L1_CONTEXT_MAGIC);
//      * every entry point resolves its handle through acquire(), which first
//        confirms membership in the live registry (a pure pointer-value lookup
//        that NEVER dereferences a dangling pointer) and then checks the cookie;
//      * L1_Destroy atomically retires the handle from the registry -- so a
//        second destroy of the same pointer finds it absent and becomes a safe
//        no-op -- then poisons the cookie and NULLs the owned pointer before
//        freeing. The registry is guarded by a mutex, so concurrent create /
//        destroy / use calls remain safe.
//    The net effect: NULL, already-destroyed, and foreign handles are all
//    rejected as safe no-ops (void functions return early, getters return 0.0)
//    with no use-after-free and no double-free.
//
//  CONSTRAINTS (AAP 0.6.4, 0.7.1):
//    - Every function validates the incoming `void*` handle (returns early for
//      the void functions, returns 0.0 for the double getters), so a NULL,
//      already-destroyed, or foreign handle can never be dereferenced.
//    - `struct L1_Context` is the ONLY place the concrete wrapper type is
//      defined; it is never exposed in the header.
//    - Memory: L1_Create allocates BOTH the context and the behavior; L1_Destroy
//      releases BOTH (the behavior first, then the context) -- no leak, no
//      double-free.
//    - No vehicle-firmware dependency; no new third-party dependency (the
//      registry uses only the C++ standard library: <cstdint>, <mutex>,
//      <unordered_set>).
// ============================================================================

#include <new>                 // std::nothrow -- contain allocation failure inside the C++ boundary
#include <cstdint>             // uint32_t -- the fixed-width validity cookie
#include <mutex>               // std::mutex / std::lock_guard -- serialise registry access
#include <unordered_set>       // std::unordered_set -- the live-handle registry

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
/// A trivially-copyable aggregate that owns a single pointer to the C++ facade
/// plus a validity cookie. Callers never see this type -- they hold only a
/// `void*` -- which is exactly what keeps the boundary free of any C++ type
/// (class, reference, template, or exception). Defined here rather than in the
/// header so the facade type stays completely hidden from consumers of
/// l1_c_api.h.
///
/// @c magic is stamped with L1_CONTEXT_MAGIC on creation and zeroed on
/// destruction, so a stale handle whose backing memory has not yet been reused
/// fails validation instead of being dereferenced.
struct L1_Context {
    uint32_t         magic;   ///< == L1_CONTEXT_MAGIC while live; 0 once destroyed
    AfsimL1Behavior* self;    ///< owned facade; nullptr once destroyed
};

} // extern "C" -- closed briefly so the internal-linkage helpers below may use
  //               C++ standard-library types; the exported entry points reopen
  //               the block further down.

// ----------------------------------------------------------------------------
// Internal handle-validation machinery (NOT exported -- internal linkage via the
// anonymous namespace). A foreign host may hand back any `void*`: NULL, a
// pointer already passed to L1_Destroy, or an unrelated address. Validating
// every handle through this registry BEFORE dereferencing turns each of those
// cases into a safe no-op rather than undefined behaviour.
// ----------------------------------------------------------------------------
namespace {

/// Sentinel stamped into a live L1_Context and cleared on destruction.
constexpr uint32_t L1_CONTEXT_MAGIC = 0xAF510C71u;

/// Mutex guarding the live-handle registry. Wrapped in a function-local static
/// (Meyers singleton) so it is constructed on first use and immune to static
/// initialisation-order problems in a shared library.
std::mutex& registry_mutex()
{
    static std::mutex m;
    return m;
}

/// Set of currently-live context pointers. Membership is tested by pointer
/// VALUE, so checking a stale/dangling/foreign pointer never dereferences it.
std::unordered_set<const void*>& live_handles()
{
    static std::unordered_set<const void*> handles;
    return handles;
}

/// Record a freshly created context as live.
void registry_add(const void* handle)
{
    std::lock_guard<std::mutex> lock(registry_mutex());
    live_handles().insert(handle);
}

/// Atomically remove @p handle from the live set. Returns true IFF it was live,
/// so exactly one L1_Destroy call ever proceeds to free a given pointer -- a
/// second (double) destroy, a never-created pointer, or a foreign pointer all
/// return false and are handled as safe no-ops WITHOUT any dereference.
bool registry_retire(const void* handle)
{
    std::lock_guard<std::mutex> lock(registry_mutex());
    return live_handles().erase(handle) != 0;
}

/// True IFF @p handle is currently registered as live (pointer-value lookup).
bool registry_contains(const void* handle)
{
    std::lock_guard<std::mutex> lock(registry_mutex());
    return live_handles().find(handle) != live_handles().end();
}

/// Resolve an incoming handle to its concrete context for USE, or nullptr if it
/// is not safe to dereference. The registry membership test runs first and is a
/// pure pointer-value comparison, so a dangling pointer is rejected before it is
/// ever dereferenced; the cookie check is defence-in-depth once membership (and
/// therefore a live object) is established.
L1_Context* acquire(void* handle)
{
    if (handle == nullptr) {
        return nullptr;
    }
    if (!registry_contains(handle)) {
        return nullptr;
    }
    auto* ctx = static_cast<L1_Context*>(handle);
    if (ctx->magic != L1_CONTEXT_MAGIC || ctx->self == nullptr) {
        return nullptr;
    }
    return ctx;
}

} // namespace

extern "C" {

/// Create a new AfsimL1 service instance.
///
/// Allocates the opaque context together with its underlying AfsimL1Behavior
/// facade and returns an opaque handle to it. The returned handle must later be
/// released with L1_Destroy() to avoid leaking memory.
///
/// @return  Opaque handle, or nullptr if allocation failed. No C++ exception
///          ever crosses this extern "C" boundary.
__attribute__((visibility("default"))) void* L1_Create() {
    // Keep all C++ allocation failure INSIDE the C++ boundary: use non-throwing
    // new so a failure yields nullptr (the documented NULL-on-failure contract)
    // instead of letting a std::bad_alloc cross the extern "C" ABI -- which is
    // undefined for a C caller -- or aborting under -fno-exceptions, where a
    // throwing new has no other way to report failure. Allocate the facade
    // first, then the context that owns it, deleting the facade if the second
    // allocation fails so no partial allocation is leaked.
    AfsimL1Behavior* self = new (std::nothrow) AfsimL1Behavior();
    if (self == nullptr) {
        return nullptr;
    }
    L1_Context* ctx = new (std::nothrow) L1_Context{L1_CONTEXT_MAGIC, self};
    if (ctx == nullptr) {
        delete self;
        return nullptr;
    }
    // Record the context as live so subsequent calls can validate the handle
    // by pointer value before dereferencing it.
    registry_add(ctx);
    return ctx;
}

/// Destroy a service instance previously returned by L1_Create().
///
/// Releases the underlying facade first, then the opaque context. Passing NULL,
/// an already-destroyed handle, or a foreign pointer is a safe no-op: the handle
/// is atomically retired from the live registry, so a double destroy finds it
/// absent and does nothing -- no dereference and no double-free occur. After a
/// successful destroy the handle is invalid and must not be reused.
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
__attribute__((visibility("default"))) void L1_Destroy(void* handle) {
    if (!handle) return;
    // Atomically remove the handle from the live set. If it was NOT live
    // (double destroy, never created, or foreign pointer) retire() returns false
    // and we stop here -- crucially WITHOUT dereferencing a possibly-dangling
    // pointer. Only the single caller that wins the retire proceeds to free.
    if (!registry_retire(handle)) return;
    auto* ctx = static_cast<L1_Context*>(handle);
    ctx->magic = 0u;          // poison: any lingering copy of this handle now fails acquire()
    delete ctx->self;
    ctx->self = nullptr;      // avoid a dangling owned pointer prior to freeing the context
    delete ctx;
}

/// Initialise the service instance (seed a simple default leg / state).
///
/// Passing NULL is a safe no-op.
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
__attribute__((visibility("default"))) void L1_Init(void* handle) {
    auto* ctx = acquire(handle);
    if (!ctx) return;
    ctx->self->init();
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
    auto* ctx = acquire(handle);
    if (!ctx) return;
    ctx->self->execute(dt_seconds);
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
    auto* ctx = acquire(handle);
    if (!ctx) return;
    ctx->self->set_leg_ne(prevN, prevE, nextN, nextE);
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
    auto* ctx = acquire(handle);
    if (!ctx) return;
    ctx->self->set_state_ne(n, e, velE, velN, yaw_cd, pitch_rad);
}

/// Read the roll command produced by the most recent L1_Execute().
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
/// @return        Commanded roll angle in degrees, or 0.0 if @p handle is NULL.
__attribute__((visibility("default"))) double L1_GetRollDeg(void* handle) {
    auto* ctx = acquire(handle);
    if (!ctx) return 0.0;
    return ctx->self->get_roll_deg();
}

/// Read the lateral-acceleration demand produced by the most recent L1_Execute().
///
/// @param handle  Opaque handle returned by L1_Create(), or NULL.
/// @return        Demanded lateral acceleration in m/s^2, or 0.0 if @p handle is NULL.
__attribute__((visibility("default"))) double L1_GetLatAccel(void* handle) {
    auto* ctx = acquire(handle);
    if (!ctx) return 0.0;
    return ctx->self->get_lat_accel();
}

} // extern "C"
