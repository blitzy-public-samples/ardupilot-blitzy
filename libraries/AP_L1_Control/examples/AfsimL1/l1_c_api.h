#pragma once

/**
 * @file    l1_c_api.h
 * @brief   Stable C Application Binary Interface (ABI) for the AfsimL1 service.
 *
 * This header is the Anti-Corruption / ABI-boundary layer of the AfsimL1
 * reusable navigation service (AAP 0.3.3, 0.6.4). It exposes a flat,
 * mangling-free "extern C" interface plus an opaque handle so that an external
 * host - for example the AFSIM simulation environment - can bind the shared
 * library libafsim_l1.so at run time regardless of the compiler or C++ standard
 * library it was built with.
 *
 * Only plain C scalar types (void* and double) cross this boundary. No C++
 * type, reference, class, template, or exception is ever exposed here. That is
 * what makes the interface ABI-stable: C++ name mangling, virtual-table layout,
 * RTTI, and exception representation all differ across compilers and compiler
 * versions, whereas the flat C ABI is a stable superset that every toolchain
 * agrees upon. Consequently the concrete C++ implementation (the
 * AfsimL1Behavior facade and the AP_L1_Control guidance controller it wraps) is
 * completely hidden behind this seam.
 *
 * The interface reproduces the shape of the user "initialize a simple leg"
 * example one-to-one:
 *
 *     void* h = L1_Create();                 -- construct the service instance
 *     L1_Init(h);                            -- seed a default leg / state
 *     L1_SetLegNE(h, 0, 0, 0, 500);          -- prev(N,E) -> next(N,E), metres
 *     L1_SetStateNE(h, 0, 0, 0, 20, 0, 0);   -- n,e, velE,velN, yaw_cd, pitch_rad
 *     L1_Execute(h, 0.02);                   -- one guidance step, host dt (s)
 *     double roll = L1_GetRollDeg(h);        -- roll command,       degrees
 *     double lat  = L1_GetLatAccel(h);       -- lateral accel,      m/s^2
 *     L1_Destroy(h);                         -- release the instance
 *
 * This header is intentionally pure C-compatible: it includes no other headers,
 * uses only /-style block comments, and uses no C++-only construct outside the
 * __cplusplus guards, so it can be consumed unchanged from a pure C host down to
 * the C89 standard.
 */

/*
 * Export-visibility helper.
 *
 * Building the shared library with `-fvisibility=hidden` hides every symbol by
 * default; only the entry points explicitly annotated with
 * `__attribute__((visibility("default")))` remain exported. Annotating the
 * declarations here (in addition to the definitions in l1_c_api.cpp) keeps the
 * header and the implementation in agreement, so a caller that includes this
 * header sees the same visibility the linker actually applies. On toolchains
 * that do not understand the attribute the macro expands to nothing, which is
 * harmless for callers that merely link against an already-built library.
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
 * Opaque handle to a single AfsimL1 service instance.
 *
 * The concrete type (struct L1_Context, which wraps an AfsimL1Behavior*) is
 * defined only in l1_c_api.cpp; callers must treat it as opaque. A host holds
 * nothing but a `void*` and never sees AfsimL1Behavior or any other C++ type.
 * L1_Handle is provided purely as a self-documenting alias for that `void*`;
 * the entry points below accept and return a raw `void*` so their signatures
 * match the C ABI byte-for-byte.
 */
typedef void* L1_Handle;

/**
 * Create a new AfsimL1 service instance.
 *
 * Allocates the opaque context together with its underlying guidance facade and
 * returns an opaque handle to it. The returned handle must eventually be
 * released with L1_Destroy() to avoid leaking memory.
 *
 * @return  Opaque handle (non-NULL on success), or NULL if allocation failed.
 */
L1_API void* L1_Create(void);

/**
 * Destroy a service instance previously returned by L1_Create().
 *
 * Releases the underlying guidance facade and the opaque context. Passing NULL
 * is a safe no-op. After this call the handle is invalid and must not be reused.
 *
 * @param handle  Opaque handle returned by L1_Create(), or NULL.
 */
L1_API void L1_Destroy(void* handle);

/**
 * Initialise the service instance.
 *
 * Seeds a default leg/state so the instance is ready to execute. Passing NULL
 * is a safe no-op.
 *
 * @param handle  Opaque handle returned by L1_Create(), or NULL.
 */
L1_API void L1_Init(void* handle);

/**
 * Advance the guidance by one control step using a host-supplied time delta.
 *
 * The host owns the timebase: @p dt_seconds is injected directly instead of the
 * controller computing its own delta from the hardware clock. Passing NULL is a
 * safe no-op.
 *
 * @param handle      Opaque handle returned by L1_Create(), or NULL.
 * @param dt_seconds  Control-step interval in seconds (e.g. 0.02 for 50 Hz).
 */
L1_API void L1_Execute(void* handle, double dt_seconds);

/**
 * Set the active navigation leg from the previous and next waypoints.
 *
 * Coordinates are North/East offsets in metres relative to the service datum.
 * Passing NULL is a safe no-op.
 *
 * @param handle  Opaque handle returned by L1_Create(), or NULL.
 * @param prevN   Previous waypoint, North offset  (metres).
 * @param prevE   Previous waypoint, East  offset  (metres).
 * @param nextN   Next     waypoint, North offset  (metres).
 * @param nextE   Next     waypoint, East  offset  (metres).
 */
L1_API void L1_SetLegNE(void* handle, double prevN, double prevE, double nextN, double nextE);

/**
 * Inject the current platform state consumed by the guidance controller.
 *
 * Supplies the position, velocity, and attitude that the vehicle loop would
 * normally read from AHRS sensor fusion. Passing NULL is a safe no-op.
 *
 * @param handle     Opaque handle returned by L1_Create(), or NULL.
 * @param n          Position North offset from datum   (metres).
 * @param e          Position East  offset from datum   (metres).
 * @param velE       Ground velocity, East  component   (m/s).
 * @param velN       Ground velocity, North component   (m/s).
 * @param yaw_cd     Heading / yaw                       (centidegrees).
 * @param pitch_rad  Pitch                               (radians).
 */
L1_API void L1_SetStateNE(void* handle, double n, double e, double velE, double velN, double yaw_cd, double pitch_rad);

/**
 * Read the roll command produced by the most recent L1_Execute().
 *
 * @param handle  Opaque handle returned by L1_Create(), or NULL.
 * @return        Commanded roll angle in degrees, or 0.0 if @p handle is NULL.
 */
L1_API double L1_GetRollDeg(void* handle);

/**
 * Read the lateral acceleration demand produced by the most recent L1_Execute().
 *
 * @param handle  Opaque handle returned by L1_Create(), or NULL.
 * @return        Demanded lateral acceleration in m/s^2, or 0.0 if @p handle is NULL.
 */
L1_API double L1_GetLatAccel(void* handle);

#ifdef __cplusplus
}
#endif
