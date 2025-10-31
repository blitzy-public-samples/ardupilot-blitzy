/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file lua_scripts.h
 * @brief Core Lua VM lifecycle manager and script execution scheduler for ArduPilot scripting subsystem
 * 
 * @details This file defines the lua_scripts class which manages the complete lifecycle of the Lua
 *          virtual machine used for ArduPilot scripting. It handles:
 *          - MultiHeap-backed Lua state creation and memory management
 *          - Script sandbox creation for security and isolation
 *          - Script loading from filesystem with CRC verification
 *          - Priority-based script scheduling (based on next_run_ms)
 *          - Instruction budget enforcement through lua_sethook callbacks
 *          - Error recovery using panic handlers with jmp_buf
 *          - Thread-safe error message reporting across scripting/main threads
 *          
 *          The lua_scripts class runs in a dedicated scripting thread and provides
 *          the foundation for all Lua-based vehicle automation in ArduPilot.
 * 
 * @note This is a safety-critical component - scripts can directly control vehicle behavior
 * @warning All script execution is subject to instruction budget limits to prevent infinite loops
 */
#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <setjmp.h>

#include <AP_Filesystem/posix_compat.h>
#include <AP_Scripting/AP_Scripting.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/Semaphores.h>
#include <AP_MultiHeap/AP_MultiHeap.h>
#include "lua_common_defs.h"

#include "lua/src/lua.hpp"

/**
 * @class lua_scripts
 * @brief VM engine managing MultiHeap-backed Lua state, sandbox creation, script loading/scheduling, and error recovery
 * 
 * @details The lua_scripts class is the core execution engine for ArduPilot's Lua scripting system.
 *          It manages a single Lua VM instance with the following responsibilities:
 *          
 *          **Memory Management:**
 *          - Allocates Lua state from MultiHeap (separate heap from main ArduPilot heap)
 *          - Provides custom allocator (alloc()) that enforces memory limits
 *          - Tracks memory usage per script execution for diagnostics
 *          
 *          **Script Lifecycle:**
 *          - Loads scripts from filesystem directories (load_all_scripts_in_dir)
 *          - Maintains CRC checksums for loaded and running scripts
 *          - Creates isolated sandbox environments per script (create_sandbox)
 *          - Manages linked list of script_info structures sorted by next_run_ms
 *          
 *          **Execution Scheduling:**
 *          - Runs highest-priority script (earliest next_run_ms) each iteration
 *          - Enforces instruction budget via lua_sethook callback (hook())
 *          - Reschedules scripts after execution based on their requested interval
 *          - Removes crashed or errored scripts from execution queue
 *          
 *          **Safety and Error Handling:**
 *          - Panic handler (atpanic()) uses jmp_buf for non-local exit recovery
 *          - Instruction counting prevents infinite loops (overtime flag)
 *          - Thread-safe error message buffering with semaphore protection
 *          - Configurable debug options for runtime behavior control
 *          
 *          **Threading Model:**
 *          The lua_scripts instance runs in a dedicated scripting thread separate from
 *          the main vehicle control thread. Error messages and checksums are accessed
 *          across threads and require semaphore protection.
 * 
 * @warning Scripts run with real vehicle control authority - sandbox restrictions are critical
 * @warning Instruction budget enforcement is essential to prevent vehicle control lockup
 * @warning Error message buffer access across threads requires semaphore protection
 * 
 * @see AP_Scripting for integration with ArduPilot's main scheduler
 * @see script_info for per-script metadata and scheduling information
 */
class lua_scripts
{
public:
    /**
     * @brief Construct Lua VM manager with specified execution budget and memory allocation
     * 
     * @details Initializes the lua_scripts engine with resource limits and runtime options.
     *          The constructor sets up references to configuration parameters but does not
     *          create the Lua state - that occurs during run() when the scripting thread starts.
     * 
     * @param[in] vm_steps       Instruction budget per script execution (number of Lua VM instructions
     *                           before hook() callback triggers). Typical value: 10000-100000.
     *                           Higher values allow longer script runs but increase risk of control delays.
     * @param[in] heap_size      Memory allocation for Lua heap in bytes. Scripts share this heap.
     *                           Typical value: 32768-262144 bytes depending on platform capabilities.
     * @param[in] debug_options  Runtime behavior flags (bitmask of AP_Scripting::DebugOption values).
     *                           Controls verbose logging, error suppression, and debug features.
     * 
     * @note Constructor does not allocate Lua state - actual VM creation happens in run()
     * @note vm_steps directly affects responsiveness - too high can cause control system delays
     * @warning heap_size must be sufficient for all loaded scripts or loading will fail
     */
    lua_scripts(const AP_Int32 &vm_steps, const AP_Int32 &heap_size, AP_Int8 &debug_options);

    ~lua_scripts();

    CLASS_NO_COPY(lua_scripts);

    /**
     * @brief Check if Lua heap initialization was successful
     * 
     * @details Verifies that the MultiHeap allocation for the Lua VM succeeded.
     *          This should be called after construction to ensure the scripting
     *          subsystem has sufficient memory to operate.
     * 
     * @return true if heap was successfully allocated and is available for use
     * @return false if heap allocation failed (insufficient memory or heap creation error)
     * 
     * @note Heap allocation failure means scripting cannot run - vehicle will operate without scripts
     * @warning If this returns false, run() should not be called as VM cannot be created
     */
    bool heap_allocated() const { return _heap.available(); }

    /**
     * @brief Main execution loop for script scheduling and execution (runs continuously in scripting thread)
     * 
     * @details This is the primary entry point for the scripting thread. It performs the following:
     *          1. Creates Lua state with custom allocator backed by MultiHeap
     *          2. Sets up panic handler (atpanic) with jmp_buf for error recovery
     *          3. Loads all scripts from configured directories
     *          4. Enters infinite loop executing run_next_script() to process scheduled scripts
     *          5. If panic occurs, jumps back to beginning and attempts recovery
     *          
     *          The function does not return under normal operation. It only returns if
     *          an unrecoverable error occurs (e.g., Lua state creation fails).
     *          
     *          **Execution Flow:**
     *          - Calls run_next_script() to execute highest priority script (earliest next_run_ms)
     *          - Enforces instruction budget through hook() callback set by lua_sethook
     *          - Reschedules scripts after execution based on their requested interval
     *          - Removes scripts that crash or error during execution
     * 
     * @note This function blocks indefinitely and runs in the dedicated scripting thread
     * @note Uses setjmp/longjmp for panic recovery to prevent thread termination
     * @warning Never call this from the main vehicle control thread - runs in separate thread only
     * @warning If this function returns, scripting has failed and will not recover
     * 
     * @see run_next_script() for per-script execution logic
     * @see atpanic() for panic handler and recovery mechanism
     */
    void run(void);

    /**
     * @brief Flag indicating current script exceeded its instruction budget and execution is being terminated
     * 
     * @details Set to true by hook() callback when instruction count exceeds vm_steps limit.
     *          Used to signal that script execution should be aborted immediately.
     *          Reset to false before each script execution begins.
     *          
     *          When overtime is true:
     *          - Current script execution is terminated via lua_error()
     *          - Script is removed from execution queue
     *          - Error message is logged indicating instruction budget exceeded
     * 
     * @warning Must be static to be accessible from static hook() callback function
     * @warning This flag protects vehicle control responsiveness from infinite loops in scripts
     * 
     * @see hook() for instruction budget enforcement mechanism
     * @see reset_loop_overtime() for flag reset before script execution
     */
    static bool overtime; // script exceeded it's execution slot, and we are bailing out

private:

    void create_sandbox(lua_State *L);

    /**
     * @struct script_info
     * @brief Linked-list node containing script metadata, scheduling information, and Lua environment references
     * 
     * @details Each loaded script has an associated script_info structure that maintains:
     *          - Lua registry references to the script's isolated environment and run function
     *          - Scheduling information (next execution time in milliseconds since boot)
     *          - CRC32 checksum for script verification and change detection
     *          - Filename for error reporting and diagnostics
     *          - Linked list pointer for priority queue implementation
     *          
     *          Scripts are maintained in a sorted linked list ordered by next_run_ms (earliest first).
     *          This allows efficient O(1) selection of the next script to execute.
     *          
     *          **Lifecycle:**
     *          1. Created by load_script() when script file is loaded
     *          2. Inserted into sorted list via reschedule_script()
     *          3. Removed from list during execution by run_next_script()
     *          4. Re-inserted after execution based on script's requested interval
     *          5. Permanently removed by remove_script() on error or crash
     * 
     * @warning env_ref and run_ref are Lua registry references - must be freed with luaL_unref on removal
     * @warning next_run_ms comparison must handle uint64_t wraparound (occurs after 584 million years)
     * 
     * @see load_script() for script_info creation
     * @see reschedule_script() for priority queue insertion
     * @see run_next_script() for script execution and rescheduling
     */
    typedef struct script_info {
       int env_ref;          // reference to the script's environment table
       int run_ref;          // reference to the function to run
       uint64_t next_run_ms; // time (in milliseconds) the script should next be run at
       uint32_t crc;         // crc32 checksum
       char *name;           // filename for the script // FIXME: This information should be available from Lua
       script_info *next;
    } script_info;

    /**
     * @brief Load Lua script from filesystem and create script_info metadata with CRC tracking
     * 
     * @details Loads a script file and prepares it for execution:
     *          1. Reads script file from filesystem using posix_compat layer
     *          2. Calculates CRC32 checksum of file contents for verification
     *          3. Compiles Lua source using luaL_loadbuffer
     *          4. Creates isolated sandbox environment via create_sandbox()
     *          5. Stores registry references to environment and compiled function
     *          6. Allocates and initializes script_info structure
     *          7. Updates loaded_checksum with XOR of script CRC
     *          
     *          On success, returns script_info ready for insertion into execution queue.
     *          On failure (file not found, compilation error, out of memory), returns nullptr
     *          and logs appropriate error message.
     * 
     * @param[in] L         Lua state into which script will be loaded
     * @param[in] filename  Full path to script file (e.g., "/APM/scripts/example.lua")
     * 
     * @return Pointer to newly allocated script_info structure on success
     * @return nullptr on failure (file error, compilation error, or allocation failure)
     * 
     * @note Caller is responsible for inserting returned script_info into execution queue
     * @note CRC is used for detecting script changes and verifying loaded vs running scripts
     * @warning Compilation errors in script will cause load to fail - check error messages
     * @warning Returned script_info must be freed with remove_script() to prevent memory leak
     * 
     * @see create_sandbox() for script environment isolation
     * @see reschedule_script() for inserting script into execution queue
     * @see get_loaded_checksum() for CRC-based script verification
     */
    script_info *load_script(lua_State *L, char *filename);

    void reset_loop_overtime(lua_State *L);

    void load_all_scripts_in_dir(lua_State *L, const char *dirname);

    /**
     * @brief Execute highest-priority script (earliest next_run_ms) from the scheduled script queue
     * 
     * @details Implements the core script scheduling algorithm:
     *          1. Checks if any scripts are ready to run (next_run_ms <= current time)
     *          2. If no scripts ready, sleeps until next script's scheduled time
     *          3. Removes highest-priority script from linked list
     *          4. Resets overtime flag and sets instruction count hook
     *          5. Pushes script's run function and environment onto Lua stack
     *          6. Calls lua_pcall to execute script in protected mode
     *          7. Handles script return value to determine next_run_ms:
     *             - Integer return: reschedule after N milliseconds
     *             - No return: remove script from queue (run once)
     *             - Error: remove script and log error message
     *          8. Updates running_checksum and performance statistics
     *          9. Re-inserts script into sorted queue via reschedule_script()
     *          
     *          The function enforces instruction budget through lua_sethook callback.
     *          If script exceeds vm_steps instructions, hook() sets overtime=true and
     *          script execution terminates.
     * 
     * @param[in] L  Lua state in which to execute the script
     * 
     * @note This function blocks if no scripts are ready to run (sleeps until next scheduled time)
     * @note Scripts are executed in priority order: earliest next_run_ms runs first
     * @warning Script errors cause script removal - no automatic retry or recovery
     * @warning Instruction budget exceeded (overtime) results in script termination and removal
     * 
     * @see script_info for script metadata and scheduling information
     * @see reschedule_script() for priority queue insertion after execution
     * @see hook() for instruction budget enforcement
     * @see overtime flag for budget exceeded indication
     */
    void run_next_script(lua_State *L);

    void remove_script(lua_State *L, script_info *script);

    // reschedule the script for execution. It is assumed the script is not in the list already
    void reschedule_script(script_info *script);

    script_info *scripts; // linked list of scripts to be run, sorted by next run time (soonest first)

    /**
     * @brief Lua hook callback for instruction-count enforcement (cpu time budget limiter)
     * 
     * @details Registered with lua_sethook as LUA_MASKCOUNT callback to enforce instruction budget.
     *          Called automatically by Lua VM after every vm_steps instructions executed.
     *          
     *          When called:
     *          1. Sets overtime flag to true
     *          2. Generates Lua error using luaL_error to terminate script execution
     *          3. Error propagates up to lua_pcall in run_next_script()
     *          4. Script is removed from execution queue
     *          
     *          This mechanism prevents infinite loops or excessively long-running scripts
     *          from blocking the scripting thread and delaying vehicle control updates.
     * 
     * @param[in] L   Lua state in which the hook was triggered
     * @param[in] ar  Lua debug structure (unused in this implementation)
     * 
     * @note Must be static to satisfy lua_sethook C function pointer requirement
     * @note Hook is set with vm_steps count - called after that many instructions
     * @warning This function does not return normally - uses luaL_error for non-local exit
     * @warning Instruction budget enforcement is critical for vehicle control responsiveness
     * 
     * @see overtime flag for budget exceeded indication
     * @see run_next_script() for hook registration via lua_sethook
     */
    static void hook(lua_State *L, lua_Debug *ar);

    /**
     * @brief Lua panic handler for unprotected error recovery using jmp_buf non-local exit
     * 
     * @details Registered with lua_atpanic to handle errors that occur outside protected calls.
     *          Lua panics occur when errors happen in contexts where pcall/xpcall cannot catch them
     *          (e.g., memory allocation failures, C API misuse, __gc metamethod errors).
     *          
     *          When panic occurs:
     *          1. Prints error message from Lua stack using print_error()
     *          2. Executes longjmp(panic_jmp, 1) to jump back to setjmp in run()
     *          3. Execution resumes at beginning of run() for recovery attempt
     *          4. Lua state is recreated and scripts are reloaded
     *          
     *          This prevents panic from terminating the scripting thread entirely,
     *          allowing recovery and continued operation after catastrophic script errors.
     * 
     * @param[in] L  Lua state that experienced the panic
     * 
     * @return Never returns normally - uses longjmp for non-local exit
     * 
     * @note Must be static to satisfy lua_atpanic C function pointer requirement
     * @note Panic is last-resort error handling - indicates serious VM or script problem
     * @warning All Lua state is lost on panic - scripts must be reloaded after recovery
     * @warning panic_jmp must be set via setjmp before any script execution begins
     * 
     * @see panic_jmp for setjmp/longjmp buffer
     * @see run() for setjmp call and recovery loop
     * @see print_error() for error message handling
     */
    static int atpanic(lua_State *L);
    static jmp_buf panic_jmp;

    lua_State *lua_state;

    const AP_Int32 & _vm_steps;
    AP_Int8 & _debug_options;

    bool option_is_set(AP_Scripting::DebugOption option) const {
        return (uint8_t(_debug_options.get()) & uint8_t(option)) != 0;
    }

    /**
     * @brief Custom Lua allocator backed by MultiHeap for memory isolation and limit enforcement
     * 
     * @details Lua allocator function implementing the standard Lua allocation interface.
     *          All Lua memory allocations (tables, strings, functions, userdata) go through
     *          this function. Allocations are serviced from the dedicated MultiHeap (_heap)
     *          rather than the main ArduPilot heap, providing:
     *          - Memory isolation (script memory separate from flight control memory)
     *          - Hard memory limits (scripts cannot exhaust system memory)
     *          - Separate heap debugging and statistics
     *          
     *          Allocation operations handled:
     *          - nsize == 0: Free memory (calls _heap.deallocate)
     *          - ptr == nullptr && nsize > 0: Allocate new block (_heap.allocate)
     *          - ptr != nullptr && nsize > 0: Reallocate/resize block (_heap.reallocate)
     *          
     *          Returns nullptr on allocation failure (out of heap space).
     *          Lua VM handles nullptr by raising memory allocation error.
     * 
     * @param[in] ud    User data pointer (unused - nullptr)
     * @param[in] ptr   Pointer to existing block (nullptr for new allocation)
     * @param[in] osize Old size of block in bytes (for tracking, may be approximate)
     * @param[in] nsize New size in bytes (0 means free, >0 means allocate/resize)
     * 
     * @return Pointer to allocated/reallocated memory block on success
     * @return nullptr on allocation failure (out of memory)
     * 
     * @note Must be static to satisfy lua_newstate allocator function pointer requirement
     * @note All allocations are O(1) or O(log n) depending on MultiHeap implementation
     * @warning nullptr return causes Lua to raise memory error - scripts should catch these
     * @warning Heap exhaustion can prevent new scripts from loading or running
     * 
     * @see _heap for MultiHeap instance
     * @see heap_allocated() for heap initialization check
     */
    static void *alloc(void *ud, void *ptr, size_t osize, size_t nsize);

    static MultiHeap _heap;

    // helper for print and log of runtime stats
    void update_stats(const char *name, uint32_t run_time, int total_mem, int run_mem);

    // must be static for use in atpanic
    static void print_error(MAV_SEVERITY severity);
    static char *error_msg_buf;
    static HAL_Semaphore error_msg_buf_sem;
    static uint8_t print_error_count;
    static uint32_t last_print_ms;

    // XOR of crc32 of running scripts
    static uint32_t loaded_checksum;
    static uint32_t running_checksum;
    static HAL_Semaphore crc_sem;

public:
    /**
     * @brief Set new error message and print to GCS with specified severity (thread-safe)
     * 
     * @details Formats and stores error message in thread-safe buffer for cross-thread access.
     *          Used by Lua bindings and internal error handling to report script errors to GCS.
     *          
     *          Thread Safety:
     *          - Acquires error_msg_buf_sem semaphore before modifying buffer
     *          - Safe to call from scripting thread or binding functions
     *          - get_last_error_message() requires same semaphore for reading
     *          
     *          Rate Limiting:
     *          - Limits error printing to prevent GCS telemetry flooding
     *          - Tracks last_print_ms and print_error_count for throttling
     *          
     *          Usage:
     *          - Fatal errors: Use MAV_SEVERITY_ERROR
     *          - Warnings: Use MAV_SEVERITY_WARNING
     *          - Info: Use MAV_SEVERITY_INFO
     * 
     * @param[in] severity  MAVLink severity level (ERROR, WARNING, INFO, etc.)
     * @param[in] fmt       Printf-style format string
     * @param[in] ...       Variable arguments for format string
     * 
     * @note Must be static for use in atpanic() panic handler
     * @note Public to allow Lua bindings to issue non-fatal warnings to GCS
     * @warning Semaphore must be held when calling get_last_error_message() to read buffer
     * @warning Buffer has fixed size - very long messages will be truncated
     * 
     * @see get_last_error_message() for retrieving error message
     * @see get_last_error_semaphore() for semaphore access
     * @see error_msg_buf for message storage buffer
     */
    static void set_and_print_new_error_message(MAV_SEVERITY severity, const char *fmt, ...) FMT_PRINTF(2,3);

    /**
     * @brief Return last error message from scripting subsystem (requires semaphore protection)
     * 
     * @details Provides read access to error message buffer that is updated by scripting thread.
     *          Caller MUST hold error_msg_buf_sem semaphore when calling this function and
     *          while using the returned pointer, as the buffer may be modified by another thread.
     *          
     *          Typical usage pattern:
     *          ```cpp
     *          WITH_SEMAPHORE(*lua_scripts::get_last_error_semaphore());
     *          const char *msg = lua_scripts::get_last_error_message();
     *          if (msg != nullptr) {
     *              // Use message safely within semaphore scope
     *          }
     *          ```
     * 
     * @return Pointer to null-terminated error message string
     * @return nullptr if no error message is currently stored
     * 
     * @warning MUST hold error_msg_buf_sem semaphore before calling and while using returned pointer
     * @warning Returned pointer is only valid while semaphore is held - do not store it
     * @warning Message buffer is shared across threads - race conditions possible without semaphore
     * 
     * @see set_and_print_new_error_message() for setting error messages
     * @see get_last_error_semaphore() for required semaphore
     */
    static const char* get_last_error_message() { return error_msg_buf; }

    /**
     * @brief Get semaphore protecting error message buffer for thread-safe access
     * 
     * @details Returns pointer to semaphore that must be held when accessing error_msg_buf.
     *          Required because error messages are set by scripting thread but read by
     *          main thread (for GCS reporting, logging, etc.).
     * 
     * @return Pointer to HAL semaphore protecting error_msg_buf
     * 
     * @note Always use WITH_SEMAPHORE macro for automatic acquire/release
     * @warning Must hold this semaphore when calling get_last_error_message()
     * 
     * @see get_last_error_message() for reading error messages
     * @see set_and_print_new_error_message() for writing error messages
     */
    static AP_HAL::Semaphore* get_last_error_semaphore() { return &error_msg_buf_sem; }

    /**
     * @brief Return XOR checksum of all loaded script CRC32 values for verification
     * 
     * @details Maintains cumulative XOR of CRC32 checksums for all scripts loaded from filesystem.
     *          Updated by load_script() when each script is successfully loaded.
     *          Used to detect script changes or verify script integrity.
     *          
     *          XOR properties allow incremental updates:
     *          - Adding script: loaded_checksum ^= new_script_crc
     *          - Removing script: loaded_checksum ^= removed_script_crc
     *          - XOR is commutative: order doesn't matter
     *          
     *          Compare with get_running_checksum() to detect if loaded scripts differ
     *          from currently executing scripts (e.g., script reload needed).
     * 
     * @return uint32_t XOR of all loaded script CRC32 checksums
     * 
     * @note Checksum is protected by crc_sem semaphore for thread-safe access
     * @note Checksum of zero means no scripts loaded (or checksums XOR to zero)
     * 
     * @see get_running_checksum() for checksum of currently executing scripts
     * @see load_script() for CRC calculation and checksum update
     * @see crc_sem for thread-safe checksum access
     */
    static uint32_t get_loaded_checksum();

    /**
     * @brief Return XOR checksum of all running script CRC32 values for verification
     * 
     * @details Maintains cumulative XOR of CRC32 checksums for all scripts currently in
     *          the execution queue. Updated by run_next_script() when script successfully
     *          executes. Used to verify that running scripts match loaded scripts.
     *          
     *          Differs from loaded_checksum when:
     *          - Scripts are loaded but not yet executed
     *          - Scripts have been removed from execution after errors
     *          - Script files changed on filesystem but not yet reloaded
     *          
     *          Monitoring difference between loaded and running checksums can trigger
     *          script reload to pick up filesystem changes.
     * 
     * @return uint32_t XOR of all running script CRC32 checksums
     * 
     * @note Checksum is protected by crc_sem semaphore for thread-safe access
     * @note Checksum of zero means no scripts running (or checksums XOR to zero)
     * @warning Running checksum may differ from loaded checksum if scripts crashed or were removed
     * 
     * @see get_loaded_checksum() for checksum of loaded scripts from filesystem
     * @see run_next_script() for checksum update during execution
     * @see crc_sem for thread-safe checksum access
     */
    static uint32_t get_running_checksum();

};

#endif  // AP_SCRIPTING_ENABLED
