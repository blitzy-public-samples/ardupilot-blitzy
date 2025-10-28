/**
 * @file replace.h
 * @brief QURT platform helper functions, missing libc replacements, and RPC glue
 * 
 * @details This header provides ArduPilot-compatible implementations of standard C library
 *          functions that are missing from Qualcomm's QURT RTOS minimal libc, as well as
 *          platform-specific helpers for the Hexagon DSP environment.
 *          
 *          QURT Platform Constraints:
 *          - QURT RTOS provides only a minimal subset of POSIX libc functionality
 *          - Many standard C library functions are missing (asprintf, memmem, strndup, etc.)
 *          - This file provides compatible implementations for ArduPilot's requirements
 *          - Functions are implemented specifically for Hexagon DSP instruction set and QURT ABI
 *          
 *          Key Components:
 *          1. Missing libc functions: Reimplementations of common but non-standard functions
 *          2. HAP_PRINTF infrastructure: Hexagon Application Processor printf routing for debugging
 *          3. qurt_rpc glue: Message protocol helpers for MAVLink-over-UDP transport
 *          4. Debug macros: Line tracking for DSP crash diagnostics
 * 
 * @note QURT RTOS provides minimal libc (subset of POSIX)
 * @warning Many standard C library functions missing; this file fills critical gaps
 * @warning Functions are not portable to other platforms; use only within AP_HAL_QURT
 * 
 * @see replace.cpp for implementation details
 * @see malloc.cpp for QURT memory allocation wrappers
 * @see system.cpp for platform initialization that sets up HAP_PRINTF routing
 * @see UARTDriver.cpp for qurt_rpc usage in MAVLink transport
 */

#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <types.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>

#include <types.h>
#include <dirent.h>
#include "ap_host/src/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup qurt_libc_replacements Missing libc Function Replacements
 * @brief Implementations of standard C library functions missing from QURT
 * 
 * @details QURT's minimal libc lacks many common functions. These replacements provide
 *          ArduPilot-compatible implementations for the Hexagon DSP platform.
 * 
 * @note These implementations may differ from glibc (bug-for-bug compatibility not guaranteed)
 * @warning Some functions allocate memory; caller must free() as documented
 * @warning Threading: Check individual function documentation for thread-safety guarantees
 * @note Memory allocation: Functions that allocate use fc_heap_alloc (see malloc.cpp)
 * @{
 */

/**
 * @brief Get the length of a string, limited to maximum length
 * 
 * @param[in] s String to measure
 * @param[in] maxlen Maximum length to check
 * 
 * @return Length of string or maxlen, whichever is smaller
 * 
 * @note Standard C function normally provided by string.h
 * @note Missing from QURT libc despite being POSIX standard
 */
size_t strnlen(const char *s, size_t maxlen);

/**
 * @brief Duplicate string with length limit
 * 
 * @details Creates a copy of the string s, copying at most n characters.
 *          The copied string is always null-terminated.
 * 
 * @param[in] s Source string to duplicate
 * @param[in] n Maximum length to copy
 * 
 * @return Pointer to newly allocated string, or NULL on allocation failure
 * 
 * @warning Caller must free() returned pointer to avoid memory leak
 * @note Not in standard C but common POSIX extension
 * @note Uses fc_heap_alloc internally on QURT platform
 */
char *strndup(const char *s, size_t n);

/**
 * @brief Allocate and format string (GNU extension)
 * 
 * @details Allocates a string large enough to hold the formatted output and
 *          writes the formatted string to it. Similar to sprintf but handles
 *          allocation automatically.
 * 
 * @param[out] strp Pointer to char* that will be set to allocated string
 * @param[in] fmt printf-style format string
 * @param[in] ... Variable arguments for format string
 * 
 * @return Number of characters written (excluding null terminator), or -1 on error
 * 
 * @warning Caller must free() *strp to avoid memory leak
 * @note Mimics GNU asprintf() behavior
 * @note Missing from QURT libc; this is a custom implementation
 */
int asprintf(char **strp, const char *fmt, ...);

/**
 * @brief Reposition file offset
 * 
 * @param[in] fd File descriptor
 * @param[in] offset Offset value
 * @param[in] whence SEEK_SET, SEEK_CUR, or SEEK_END
 * 
 * @return Resulting offset from beginning of file, or -1 on error
 * 
 * @note Standard POSIX function, declared here for QURT compatibility
 */
off_t lseek(int fd, off_t offset, int whence);

/**
 * @brief Open a directory for reading
 * 
 * @param[in] name Path to directory
 * 
 * @return Pointer to DIR structure, or NULL on error
 * 
 * @note Standard POSIX function, declared here for QURT compatibility
 * @note Caller must call closedir() to free resources
 */
DIR *opendir(const char *name);

/**
 * @brief Delete a file
 * 
 * @param[in] pathname Path to file to delete
 * 
 * @return 0 on success, -1 on error
 * 
 * @note Standard POSIX function, declared here for QURT compatibility
 */
int unlink(const char *pathname);

/**
 * @brief Find byte sequence in memory
 * 
 * @details Searches for the first occurrence of the byte sequence 'needle'
 *          within the memory area 'haystack'.
 * 
 * @param[in] haystack Memory buffer to search
 * @param[in] haystacklen Size of haystack buffer in bytes
 * @param[in] needle Pattern to search for
 * @param[in] needlelen Size of needle pattern in bytes
 * 
 * @return Pointer to first occurrence of needle in haystack, or NULL if not found
 * 
 * @note Not in standard C but common POSIX extension (GNU libc)
 * @note Missing from QURT libc; this is a custom implementation
 */
void *memmem(const void *haystack, size_t haystacklen,
             const void *needle, size_t needlelen);

/**
 * @brief Get process ID
 * 
 * @return Process ID of calling process
 * 
 * @note Standard POSIX function, declared here for QURT compatibility
 * @note QURT process model differs from Unix; PID may have limited meaning
 */
pid_t getpid(void);

/** @} */ // end of qurt_libc_replacements

/**
 * @defgroup qurt_hap_printf HAP Printf Infrastructure
 * @brief Hexagon Application Processor printf routing and debugging support
 * 
 * @details Routes printf() calls to Qualcomm's HAP logging system for visibility
 *          on the applications processor. Essential for debugging DSP code since
 *          the DSP has no direct console output.
 * 
 * @warning Not guaranteed to be real-time safe; use for debugging only
 * @note Output visible via logcat or QURT console (platform-dependent)
 * @{
 */

/**
 * @brief Hexagon Application Processor printf with file/line tracking
 * 
 * @details Routes formatted output to HAP logging system, including source location
 *          information for debugging. This is the underlying implementation called
 *          by the HAP_PRINTF macro.
 * 
 * @param[in] file Source file name (__FILE__)
 * @param[in] line Source line number (__LINE__)
 * @param[in] fmt printf-style format string
 * @param[in] ... Variable arguments for format string
 * 
 * @note Typically called via HAP_PRINTF macro, not directly
 * @note Output routed to apps processor log system
 * @warning May introduce latency; not suitable for time-critical code paths
 * 
 * @see HAP_PRINTF macro for convenient usage
 */
void HAP_printf(const char *file, int line, const char *fmt, ...);

/**
 * @brief Printf wrapper that routes to HAP logging
 * 
 * @details Wraps standard printf() calls to redirect to HAP logging system.
 *          Linker flag --wrap=printf routes all printf calls to this function.
 * 
 * @param[in] fmt printf-style format string
 * @param[in] ... Variable arguments for format string
 * 
 * @return Number of characters written, or negative on error
 * 
 * @note Automatically called when --wrap=printf linker flag is used
 * @note Provides printf() functionality on platform without console
 */
int __wrap_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

/**
 * @brief Convenient macro for HAP printf with automatic file/line tracking
 * 
 * @details Wraps HAP_printf to automatically include __FILE__ and __LINE__,
 *          providing source location context for all debug messages.
 * 
 * @param ... printf-style format string and arguments
 * 
 * @note Use like printf: HAP_PRINTF("value = %d\n", x);
 * @note Automatically includes source file and line number in output
 * 
 * Example:
 * @code
 * HAP_PRINTF("Sensor initialized: %s\n", sensor_name);
 * // Output includes file:line prefix for debugging
 * @endcode
 */
#define HAP_PRINTF(...) HAP_printf(__FILE__, __LINE__, __VA_ARGS__)

/**
 * @defgroup qurt_debug_tracking DSP Debug Line Tracking
 * @brief Variables and macros for tracking DSP execution location on crashes
 * 
 * @details When DSP crashes, these variables preserve the last known execution
 *          location, aiding post-mortem debugging. The HAP_LINE() macro updates
 *          tracking variables at strategic code locations.
 * 
 * @note Updated via HAP_LINE() macro at key execution points
 * @warning Adds overhead; use selectively in complex or crash-prone code
 * @{
 */

/**
 * @brief Last recorded DSP source line number
 * 
 * @note Updated by HAP_LINE() macro
 * @note Preserved across crashes for post-mortem analysis
 */
extern volatile int _last_dsp_line;

/**
 * @brief Last recorded DSP source file name
 * 
 * @note Updated by HAP_LINE() macro
 * @note Preserved across crashes for post-mortem analysis
 */
extern volatile const char *_last_dsp_file;

/**
 * @brief Execution counter incremented at each HAP_LINE() call
 * 
 * @note Provides indication of execution progress before crash
 * @note Helps identify if code is stuck in loop or progressing
 */
extern volatile uint32_t _last_counter;

/**
 * @brief Record current source location for crash diagnostics
 * 
 * @details Updates debug tracking variables with current file, line, and counter.
 *          Place this macro at strategic locations in complex code to aid debugging
 *          if DSP crashes.
 * 
 * @note Use at function entry points and before complex operations
 * @warning Adds overhead (volatile writes); don't use in tight loops
 * 
 * Example:
 * @code
 * void complex_function() {
 *     HAP_LINE();  // Track entry
 *     // ... complex operations ...
 * }
 * @endcode
 */
#define HAP_LINE() do { _last_dsp_line = __LINE__; _last_dsp_file = __FILE__; _last_counter++; } while (0)

/** @} */ // end of qurt_debug_tracking
/** @} */ // end of qurt_hap_printf

/**
 * @defgroup qurt_math_constants Missing Math Constants
 * @brief Mathematical constants missing from QURT math.h
 * 
 * @details QURT's math.h omits some standard mathematical constants.
 *          Define them here for ArduPilot's requirements.
 * @{
 */

/**
 * @brief Square root of 1/2 (1/sqrt(2))
 * 
 * @details Mathematical constant equal to 0.707106781...
 *          Useful for 45-degree rotations and normalization.
 * 
 * @note Standard constant normally defined in math.h as M_SQRT1_2
 * @note Missing from QURT math.h; defined here for compatibility
 * @note Value: √(1/2) = 1/√2 ≈ 0.70710678118654752440
 */
#define M_SQRT1_2 0.70710678118654752440

/** @} */ // end of qurt_math_constants

#ifdef __cplusplus

/**
 * @defgroup qurt_rpc_glue QURT RPC Message Protocol
 * @brief Helper functions for qurt_rpc message protocol used in MAVLink-over-UDP
 * 
 * @details Supports MAVLink transport over UDP using custom framing protocol between
 *          the DSP (SLPI) and applications processor. The qurt_rpc protocol adds headers
 *          to UDP packets for reliable MAVLink message delivery across the processor boundary.
 * 
 * @note Protocol defined in ap_host/src/protocol.h (qurt_rpc_msg wire format)
 * @warning Protocol-specific: Only works with matching apps processor implementation
 * @note Used by UARTDriver_MAVLinkUDP for packet framing
 * 
 * @see protocol.h for qurt_rpc_msg structure definition
 * @see UARTDriver.cpp for usage in MAVLink transport layer
 * @{
 */

/**
 * @brief Send RPC message to host (applications processor)
 * 
 * @details Transmits a qurt_rpc protocol message from the DSP to the applications
 *          processor. Used primarily for MAVLink-over-UDP transport, where MAVLink
 *          packets are wrapped in qurt_rpc frames for delivery.
 *          
 *          Typical Usage:
 *          - UARTDriver_MAVLinkUDP wraps outgoing MAVLink packets in qurt_rpc_msg
 *          - qurt_rpc_send() transmits the framed message to apps processor
 *          - Apps processor extracts MAVLink payload and forwards to ground station
 * 
 * @param[in] msg qurt_rpc message structure with payload and metadata
 * 
 * @return true if message sent successfully, false on transmission failure
 * 
 * @note Message structure defined in ap_host/src/protocol.h
 * @note Integrates with UARTDriver_MAVLinkUDP for MAVLink receive path
 * @warning Transmission may fail if apps processor link is down
 * @warning Not real-time safe; may block or introduce latency
 * 
 * @see struct qurt_rpc_msg in protocol.h for message format
 * @see UARTDriver::write() for typical usage pattern
 */
bool qurt_rpc_send(struct qurt_rpc_msg &msg);

/** @} */ // end of qurt_rpc_glue

#endif

