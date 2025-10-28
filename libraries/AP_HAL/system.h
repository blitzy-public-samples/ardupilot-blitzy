/**
 * @file system.h
 * @brief System-level timing and diagnostic functions
 * 
 * Defines platform-independent interface for timing functions (millis/micros) and
 * system diagnostics (panic, stack dumps, core dumps). These functions are fundamental
 * building blocks used throughout ArduPilot for timing-critical operations.
 * 
 * @note Functions declared in AP_HAL namespace for global access
 * @note Default implementations in system.cpp can be overridden by strong platform versions
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <stdint.h>

#include <AP_Common/AP_Common.h>

#include "AP_HAL_Macros.h"

namespace AP_HAL {

/**
 * @brief Initialize HAL system components
 * 
 * @details Performs platform-specific initialization of HAL subsystems.
 *          Called once at system startup before any other HAL functions.
 * 
 * @note Must be called before using any other AP_HAL functions
 * @note Platform-specific implementation varies by HAL backend
 */
void init();

/**
 * @brief Terminate execution with panic message
 * 
 * @details Displays panic message on console and halts system execution.
 *          Used for unrecoverable errors such as nullptr dereference,
 *          assertion failures, or critical system failures.
 * 
 * @param[in] errormsg Printf-style format string for error message
 * @param[in] ... Format arguments for error message
 * 
 * @warning Never returns - use only for fatal, unrecoverable errors
 * @note Message is displayed on all available consoles
 * @note System enters infinite loop after displaying message
 */
void panic(const char *errormsg, ...) FMT_PRINTF(1, 2) NORETURN;

/**
 * @brief Get microseconds since system boot (16-bit, wraps every 65ms)
 * 
 * @details Returns lower 16 bits of microsecond counter. Useful for fast
 *          loop timing with known bounds where wrap is acceptable.
 * 
 * @return uint16_t Microseconds modulo 2^16
 * 
 * @note Wraps at 65.536 milliseconds - use only for sub-100ms timing
 * @note Default implementation masks micros() to 16 bits
 * @note Useful for fast loop timing with known bounds
 * @warning Handle wrap carefully - time comparison must account for rollover
 */
uint16_t micros16();

/**
 * @brief Get microseconds since system boot (32-bit, wraps after 71.6 minutes)
 * 
 * @details Returns 32-bit microsecond counter since boot. Standard timing
 *          function for control loops and sensor sampling. Wraps at 2^32
 *          microseconds (approximately 71.6 minutes).
 * 
 * @return uint32_t Microseconds since boot
 * 
 * @note Wraps at 2^32 microseconds (71.6 minutes) - handle wrap carefully
 * @note Typical use: Control loop timing, sensor sampling timestamps
 * @note Prefer micros64() for timestamps spanning >1 hour
 * @warning Must handle wrap for timing intervals >71 minutes
 */
uint32_t micros();

/**
 * @brief Get milliseconds since system boot (32-bit, wraps after 49.7 days)
 * 
 * @details Returns 32-bit millisecond counter since boot. Standard timing
 *          function for timeouts and delays. Wraps at 2^32 milliseconds
 *          (approximately 49.7 days).
 * 
 * @return uint32_t Milliseconds since boot
 * 
 * @note Wraps at 2^32 milliseconds (49.7 days) - handle wrap in long-running systems
 * @note Typical use: Timeouts, delays, non-critical timing
 * @note For unwrapped time, use millis64()
 * @warning Must handle wrap for systems running >49 days continuously
 */
uint32_t millis();

/**
 * @brief Get milliseconds since system boot (16-bit, wraps every 65 seconds)
 * 
 * @details Returns lower 16 bits of millisecond counter. Useful for
 *          short-duration timing where wrap is acceptable.
 * 
 * @return uint16_t Milliseconds modulo 2^16
 * 
 * @note Wraps at 65.536 seconds - use only for short-duration timing
 * @note Default implementation masks millis() to 16 bits
 * @note Prefer millis() or millis64() for non-wrap-critical timing
 * @warning Handle wrap carefully - only suitable for intervals <60 seconds
 */
uint16_t millis16();

/**
 * @brief Get microseconds since system boot (64-bit, never wraps)
 * 
 * @details Returns 64-bit microsecond counter since boot. Never wraps in
 *          practical timescales (584,000 years). Preferred for absolute
 *          timestamps and protocol timing.
 * 
 * @return uint64_t Microseconds since boot
 * 
 * @note Never wraps in practical timescales (584,000 years)
 * @note Preferred for absolute timestamps and protocol timing
 * @note Standard for MAVLink and log timestamps
 * @note Slightly slower than micros() on 32-bit systems due to 64-bit arithmetic
 */
uint64_t micros64();

/**
 * @brief Get milliseconds since system boot (64-bit, never wraps)
 * 
 * @details Returns 64-bit millisecond counter since boot. Never wraps in
 *          practical timescales (584 million years). Preferred for absolute
 *          timestamps and long-duration timing.
 * 
 * @return uint64_t Milliseconds since boot
 * 
 * @note Never wraps in practical timescales (584 million years)
 * @note Preferred for absolute timestamps and long-duration timing
 * @note Use when timing intervals may exceed 49 days
 * @note Slightly slower than millis() on 32-bit systems due to 64-bit arithmetic
 */
uint64_t millis64();

/**
 * @brief Dump current stack trace to console
 * 
 * @details Outputs current call stack to console for debugging. Platform-specific
 *          implementation - may be no-op on some embedded targets. Useful for
 *          debugging deadlocks, infinite loops, and unexpected code paths.
 * 
 * @note Platform-specific implementation (may be no-op on some boards)
 * @note Useful for debugging deadlocks and crashes
 * @note Output format varies by platform (addresses, symbols if available)
 * @note SITL/Linux typically provide symbol names, embedded targets show addresses
 */
void dump_stack_trace();

/**
 * @brief Dump core file for post-mortem debugging
 * 
 * @details Generates core dump file for analysis with debugger tools like gdb.
 *          Primarily useful on SITL/Linux platforms with filesystem support.
 *          May be no-op on embedded targets without persistent storage.
 * 
 * @note Platform-specific (primarily SITL/Linux)
 * @note Generates core file for gdb analysis
 * @note May be no-op on embedded targets without filesystem
 * @note Core file can be analyzed with: gdb <binary> <corefile>
 */
void dump_core_file();

/**
 * @brief Reliably determine whether a timeout has expired
 * 
 * @details Template function to check timeout expiration using unsigned time types.
 *          Handles wrap-around correctly by using unsigned arithmetic properties.
 *          The comparison works correctly even if time counter has wrapped since
 *          past_time was recorded.
 * 
 * @tparam T Type of past_time (must be unsigned and match S)
 * @tparam S Type of now (must be unsigned and match T)
 * @tparam R Type of timeout interval (must be unsigned)
 * 
 * @param[in] past_time Starting time value
 * @param[in] now Current time value (must be same type as past_time)
 * @param[in] timeout Timeout interval
 * 
 * @return true if timeout has expired (now - past_time >= timeout)
 * @return false if timeout has not expired
 * 
 * @note Template enforces type safety - past_time and now must be same type
 * @note Handles wrap-around correctly using unsigned arithmetic
 * @note Works with millis16(), millis(), millis64(), micros16(), micros(), micros64()
 * 
 * Example usage:
 * @code
 * uint32_t start_time = millis();
 * // ... some operations ...
 * if (timeout_expired(start_time, millis(), 1000U)) {
 *     // 1 second has elapsed
 * }
 * @endcode
 */
template <typename T, typename S, typename R>
inline bool timeout_expired(const T past_time, const S now, const R timeout)
{
    static_assert(std::is_same<T, S>::value, "timeout_expired() must compare values of the same unsigned type");
    static_assert(std::is_unsigned<T>::value, "timeout_expired() must use unsigned times");
    static_assert(std::is_unsigned<R>::value, "timeout_expired() must use unsigned timeouts");
    const T dt = now - past_time;
    return (dt >= timeout);
}

/**
 * @brief Calculate how much time remains until timeout expires
 * 
 * @details Template function to compute remaining time until timeout using unsigned
 *          time types. Handles wrap-around correctly by using unsigned arithmetic
 *          properties. Returns 0 if timeout has already expired.
 * 
 * @tparam T Type of past_time (must be unsigned and match S)
 * @tparam S Type of now (must be unsigned and match T)
 * @tparam R Type of timeout interval (must be unsigned)
 * 
 * @param[in] past_time Starting time value
 * @param[in] now Current time value (must be same type as past_time)
 * @param[in] timeout Timeout interval
 * 
 * @return T Remaining time until timeout (0 if already expired)
 * 
 * @note Template enforces type safety - past_time and now must be same type
 * @note Handles wrap-around correctly using unsigned arithmetic
 * @note Returns 0 if timeout has already expired
 * @note Works with millis16(), millis(), millis64(), micros16(), micros(), micros64()
 * 
 * Example usage:
 * @code
 * uint32_t start_time = millis();
 * uint32_t timeout_ms = 5000;
 * // ... some operations ...
 * uint32_t remaining = timeout_remaining(start_time, millis(), timeout_ms);
 * if (remaining > 0) {
 *     // Still have 'remaining' milliseconds before timeout
 * }
 * @endcode
 */
template <typename T, typename S, typename R>
inline T timeout_remaining(const T past_time, const S now, const R timeout)
{
    static_assert(std::is_same<T, S>::value, "timeout_remaining() must compare values of the same unsigned type");
    static_assert(std::is_unsigned<T>::value, "timeout_remaining() must use unsigned times");
    static_assert(std::is_unsigned<R>::value, "timeout_remaining() must use unsigned timeouts");
    const T dt = now - past_time;
    return (dt >= timeout) ? T(0) : (timeout - dt);
}

} // namespace AP_HAL
