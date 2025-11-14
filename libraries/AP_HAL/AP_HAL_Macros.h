#pragma once

/**
 * @file AP_HAL_Macros.h
 * @brief Compile-time policy macros for embedded vs. desktop builds
 * 
 * @details Enforces coding policies across ArduPilot codebase, particularly restricting
 *          double-precision floating point on embedded targets where it has significant
 *          performance penalties. Provides conditional macro definitions based on build target.
 *          
 *          Key policies enforced:
 *          - Single-precision math on embedded ARM targets (performance critical)
 *          - Double-precision allowed on SITL/Linux (simulation and desktop)
 *          - Compile-time errors for accidental double-precision usage
 *          
 *          This header is automatically included by AP_HAL.h and should not be included directly.
 * 
 * @note Critical for maintaining performance on resource-constrained embedded systems
 * @warning Violating double-precision restrictions causes intentional build errors
 * 
 * Source: libraries/AP_HAL/AP_HAL_Macros.h
 */

#include <AP_HAL/AP_HAL_Boards.h>

/**
 * @brief Enable double-precision math for simulation and desktop platforms
 * 
 * @details Automatically defines ALLOW_DOUBLE_MATH_FUNCTIONS for platforms where
 *          double-precision performance penalty is acceptable or necessary:
 *          
 *          - HAL_BOARD_SITL: Software-In-The-Loop simulation on desktop
 *          - HAL_BOARD_LINUX: Native Linux boards (desktop/server-class hardware)
 *          - HAL_WITH_EKF_DOUBLE: EKF configured for double-precision (testing)
 *          - AP_SIM_ENABLED: Physics simulation backends
 *          
 *          These platforms typically run on x86/x64 with hardware double-precision
 *          support, or are used for offline analysis where performance is not critical.
 *          This prevents conflicts with system math headers that expect double versions.
 * 
 * @note On embedded ARM targets, this is NOT defined, enforcing single-precision
 * @see ALLOW_DOUBLE_MATH_FUNCTIONS for manual override (rare cases only)
 */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX || HAL_WITH_EKF_DOUBLE || AP_SIM_ENABLED
  #if !defined(ALLOW_DOUBLE_MATH_FUNCTIONS)
    #define ALLOW_DOUBLE_MATH_FUNCTIONS
  #endif
#endif

/**
 * @brief Include standard math library before macro redefinitions
 * 
 * @details Newer compilers (GCC 7.3+ for ARM Cortex-M) require math.h to be included
 *          before any macro redefinitions. This ensures proper prototype declarations
 *          for single-precision variants (sinf, cosf, etc.) are available before we
 *          redefine double-precision versions to generate errors.
 *          
 *          Include order is critical: math.h must come before DO_NOT_USE_DOUBLE_MATHS macros.
 * 
 * @note This provides declarations for both float and double versions of math functions
 */
#include <math.h>

/**
 * @brief Double-precision math prevention for embedded builds
 * 
 * @details On embedded ARM targets, double-precision (64-bit) floating point operations
 *          have severe performance and memory penalties compared to single-precision (32-bit):
 *          
 *          **Performance Impact:**
 *          - Cortex-M4/M7 FPU: Single-precision in hardware, double-precision in software
 *          - Software emulation is 10-100× slower than hardware single-precision
 *          - Critical for real-time control loops running at 400Hz-1kHz
 *          - Flash memory: Double-precision constants consume 2× space
 *          - RAM: Double-precision variables consume 2× memory
 *          - Cache efficiency: Larger data reduces cache hit rates
 *          
 *          **Enforcement Mechanism:**
 *          When ALLOW_DOUBLE_MATH_FUNCTIONS is NOT defined (embedded targets), this header
 *          redefines standard double-precision math functions to DO_NOT_USE_DOUBLE_MATHS()
 *          which generates a compile error with a clear diagnostic message:
 *          
 *          ```
 *          error: 'DO_NOT_USE_DOUBLE_MATHS' was not declared in this scope
 *          ```
 *          
 *          **Correct Usage:**
 *          Always use single-precision variants with 'f' suffix on embedded targets:
 *          - sin(x)    → sinf(x)       // Trigonometric
 *          - cos(x)    → cosf(x)
 *          - tan(x)    → tanf(x)
 *          - asin(x)   → asinf(x)
 *          - acos(x)   → acosf(x)
 *          - atan(x)   → atanf(x)
 *          - atan2(y,x)→ atan2f(y,x)
 *          - sqrt(x)   → sqrtf(x)      // Power/exponential
 *          - pow(x,y)  → powf(x,y)
 *          - exp(x)    → expf(x)
 *          - log(x)    → logf(x)       // Logarithmic
 *          - log2(x)   → log2f(x)
 *          - log10(x)  → log10f(x)
 *          - fabs(x)   → fabsf(x)      // Utility
 *          - ceil(x)   → ceilf(x)
 *          - floor(x)  → floorf(x)
 *          - round(x)  → roundf(x)
 *          - fmax(x,y) → fmaxf(x,y)
 *          
 *          **Literal Constants:**
 *          Always use 'f' suffix on floating-point literals:
 *          - 1.0   → 1.0f
 *          - 3.14  → 3.14f
 *          - 0.5   → 0.5f
 *          
 *          Python build scripts check for double literals without 'f' suffix.
 *          
 *          **Exceptions (Rare):**
 *          If a specific source file genuinely requires double precision (e.g., high-precision
 *          coordinate transformations in offline tools), define ALLOW_DOUBLE_MATH_FUNCTIONS
 *          before including any ArduPilot headers:
 *          
 *          ```cpp
 *          #define ALLOW_DOUBLE_MATH_FUNCTIONS
 *          #include <AP_HAL/AP_HAL.h>
 *          ```
 *          
 *          This should be EXTREMELY rare in flight code. Most precision requirements
 *          are satisfied by single-precision (7 decimal digits, ~1mm position accuracy).
 *          
 *          **UAVCAN Exception:**
 *          log() and fabs() are NOT redefined when HAL_NUM_CAN_IFACES > 0 to avoid
 *          conflicts with UAVCAN library headers.
 * 
 * @note AP_Math library provides single-precision wrappers for all common operations
 * @note Build system checks catch accidental double literals (1.0 vs 1.0f)
 * @warning Using double-precision on embedded targets causes 10-100× performance degradation
 * @warning Critical control loops MUST complete within timing deadlines - double math can violate this
 * 
 * @par Example - WRONG (causes compile error on embedded):
 * @code
 * float angle = sin(1.0);           // ERROR: DO_NOT_USE_DOUBLE_MATHS
 * float distance = sqrt(x*x + y*y); // ERROR: DO_NOT_USE_DOUBLE_MATHS
 * float result = pow(2.0, 3.0);     // ERROR: DO_NOT_USE_DOUBLE_MATHS
 * @endcode
 * 
 * @par Example - CORRECT (single-precision):
 * @code
 * float angle = sinf(1.0f);               // OK: single-precision function and literal
 * float distance = sqrtf(x*x + y*y);      // OK: single-precision sqrt
 * float result = powf(2.0f, 3.0f);        // OK: single-precision power
 * float rad = sinf(radians(45.0f));       // OK: radians() returns float
 * @endcode
 * 
 * @see AP_Math.h for safe single-precision math wrappers and utilities
 * @see libraries/AP_Math/AP_Math.cpp for performance-optimized math implementations
 */
#if !defined(ALLOW_DOUBLE_MATH_FUNCTIONS)

// Trigonometric functions - use sinf(), cosf(), tanf(), asinf(), acosf(), atanf(), atan2f()
#define sin(x) DO_NOT_USE_DOUBLE_MATHS()
#define cos(x) DO_NOT_USE_DOUBLE_MATHS()
#define tan(x) DO_NOT_USE_DOUBLE_MATHS()
#define acos(x) DO_NOT_USE_DOUBLE_MATHS()
#define asin(x) DO_NOT_USE_DOUBLE_MATHS()
#define atan(x) DO_NOT_USE_DOUBLE_MATHS()
#define atan2(x,y) DO_NOT_USE_DOUBLE_MATHS()

// Exponential and power functions - use expf(), powf(), sqrtf()
#define exp(x) DO_NOT_USE_DOUBLE_MATHS()
#define pow(x,y) DO_NOT_USE_DOUBLE_MATHS()
#define sqrt(x) DO_NOT_USE_DOUBLE_MATHS()

// Logarithmic functions - use log2f(), log10f(), logf()
#define log2(x) DO_NOT_USE_DOUBLE_MATHS()
#define log10(x) DO_NOT_USE_DOUBLE_MATHS()

// Rounding and utility functions - use ceilf(), floorf(), roundf(), fmaxf(), fabsf()
#define ceil(x) DO_NOT_USE_DOUBLE_MATHS()
#define floor(x) DO_NOT_USE_DOUBLE_MATHS()
#define round(x) DO_NOT_USE_DOUBLE_MATHS()
#define fmax(x,y) DO_NOT_USE_DOUBLE_MATHS()

#if !HAL_NUM_CAN_IFACES
/**
 * @brief Conditional restrictions for log() and fabs()
 * 
 * @details These functions are only restricted when CAN/UAVCAN is not enabled.
 *          When HAL_NUM_CAN_IFACES > 0, the UAVCAN library headers contain
 *          templates that conflict with macro redefinitions of log() and fabs().
 *          
 *          On CAN-enabled builds, developers must be extra vigilant to use
 *          logf() and fabsf() instead of log() and fabs().
 * 
 * @note This is a known limitation to maintain UAVCAN compatibility
 * @warning On CAN builds, log() and fabs() will NOT generate compile errors if used incorrectly
 */
#define log(x) DO_NOT_USE_DOUBLE_MATHS()
#define fabs(x) DO_NOT_USE_DOUBLE_MATHS()
#endif

#endif // !defined(ALLOW_DOUBLE_MATH_FUNCTIONS)

