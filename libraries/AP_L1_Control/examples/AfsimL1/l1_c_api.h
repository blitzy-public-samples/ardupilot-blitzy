#pragma once

/// @file    l1_c_api.h
/// @brief   Stable C Application Binary Interface (ABI) for the AfsimL1
///          navigation-guidance service.
///
/// This header is the Anti-Corruption / ABI-boundary layer of the AfsimL1
/// service. It re-exposes ArduPilot's L1 lateral-navigation guidance
/// (AP_L1_Control, wrapped by the AfsimL1Behavior facade) behind a flat,
/// name-mangling-free `extern "C"` interface plus an opaque handle, so an
/// external host such as the AFSIM simulation environment can bind
/// `libafsim_l1.so` at runtime regardless of its own compiler or C++
/// standard library.
///
/// Design rationale (why a C boundary):
///   The C++ ABI is not stable across compilers or even compiler versions:
///   name mangling, virtual-table layout, RTTI, and exception representation
///   all differ. By contrast, the C ABI is stable and is the common subset of
///   every C++ ABI. Confining the public surface to C-style free functions and
///   an opaque pointer therefore lets a differently-compiled host consume the
///   library safely. Only plain scalar types (`void*` and `double`) ever cross
///   this boundary -- no C++ type, reference, or class is exposed here.
///
/// C compatibility:
///   This header contains NO C++-only constructs outside the `__cplusplus`
///   guards and includes NO C++ service headers. It is therefore includable
///   from a pure C translation unit as well as from C++.
///
/// Usage sketch (host side):
///   @code
///   void* h = L1_Create();
///   L1_Init(h);
///   L1_SetLegNE(h, 0.0, 0.0, 500.0, 0.0);          // prev(N,E) -> next(N,E)
///   L1_SetStateNE(h, 0, 0, 0, 25, 0, 0);           // n,e,velE,velN,yaw_cd,pitch_rad
///   L1_Execute(h, 0.02);                           // advance guidance by dt seconds
///   double roll_deg  = L1_GetRollDeg(h);           // commanded bank angle (deg)
///   double lat_accel = L1_GetLatAccel(h);          // lateral acceleration (m/s^2)
///   L1_Destroy(h);
///   @endcode

/*
 * Export-visibility macro.
 *
 * Marks each entry point with default visibility so that -- when the
 * translation unit is compiled with -fvisibility=hidden (as a well-behaved
 * shared library should be) -- exactly these functions are exported from
 * `libafsim_l1.so`. Under GCC/Clang this expands to the visibility attribute;
 * on toolchains without it the macro is a no-op, so the header stays portable.
 *
 * The `.cpp` implementation is the authoritative source of the exported
 * symbols and additionally carries `__attribute__((visibility("default")))`
 * on each definition; annotating the declarations here simply keeps the header
 * and implementation in agreement.
 */
#ifndef L1_API
#  if defined(__GNUC__)
#    define L1_API __attribute__((visibility("default")))
#  else
#    define L1_API
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Opaque handle to an AfsimL1 service instance.
 *
 * Callers hold only a `void*`; they never see `AfsimL1Behavior` or any other
 * C++ type. The concrete type (`struct L1_Context`, which wraps an
 * `AfsimL1Behavior*`) is defined only in l1_c_api.cpp -- treat this handle as
 * fully opaque. Every entry point below accepts a handle produced by
 * L1_Create() and is null-safe (a NULL handle is ignored by the void-returning
 * functions and yields 0.0 from the getters).
 */
typedef void* L1_Handle;

/**
 * Construct a new AfsimL1 service instance.
 *
 * @return an opaque handle to the newly allocated instance, or NULL on
 *         allocation failure. The returned handle must eventually be released
 *         with L1_Destroy() to avoid leaking the underlying C++ object.
 */
L1_API void*  L1_Create(void);

/**
 * Destroy a service instance previously returned by L1_Create().
 *
 * @param handle opaque handle from L1_Create(). A NULL handle is ignored.
 *               After this call the handle is invalid and must not be reused.
 */
L1_API void   L1_Destroy(void* handle);

/**
 * Initialize the service instance (seed the initial leg / platform state).
 *
 * @param handle opaque handle from L1_Create(). A NULL handle is ignored.
 */
L1_API void   L1_Init(void* handle);

/**
 * Advance the L1 guidance computation by one control step.
 *
 * The host owns the timebase: @p dt_seconds is supplied directly to the
 * underlying controller instead of being derived from a hardware clock.
 *
 * @param handle     opaque handle from L1_Create(). A NULL handle is ignored.
 * @param dt_seconds control-step duration in seconds.
 */
L1_API void   L1_Execute(void* handle, double dt_seconds);

/**
 * Set the active navigation leg from the previous and next waypoints, each
 * expressed as North/East offsets (metres) from the service datum.
 *
 * @param handle opaque handle from L1_Create(). A NULL handle is ignored.
 * @param prevN  previous-waypoint North offset (m).
 * @param prevE  previous-waypoint East offset (m).
 * @param nextN  next-waypoint North offset (m).
 * @param nextE  next-waypoint East offset (m).
 */
L1_API void   L1_SetLegNE(void* handle, double prevN, double prevE, double nextN, double nextE);

/**
 * Inject the current platform state into the service.
 *
 * @param handle    opaque handle from L1_Create(). A NULL handle is ignored.
 * @param n         North position offset from datum (m).
 * @param e         East position offset from datum (m).
 * @param velE      East ground-velocity component (m/s).
 * @param velN      North ground-velocity component (m/s).
 * @param yaw_cd    yaw / heading in centidegrees.
 * @param pitch_rad pitch in radians.
 */
L1_API void   L1_SetStateNE(void* handle, double n, double e, double velE, double velN, double yaw_cd, double pitch_rad);

/**
 * Read the most recently computed commanded roll (bank) angle.
 *
 * @param handle opaque handle from L1_Create().
 * @return commanded roll angle in degrees, or 0.0 if @p handle is NULL.
 */
L1_API double L1_GetRollDeg(void* handle);

/**
 * Read the most recently computed lateral acceleration demand.
 *
 * @param handle opaque handle from L1_Create().
 * @return lateral acceleration in m/s^2 (+ve to the right), or 0.0 if
 *         @p handle is NULL.
 */
L1_API double L1_GetLatAccel(void* handle);

#ifdef __cplusplus
}
#endif
