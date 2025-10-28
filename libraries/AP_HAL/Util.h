/**
 * @file Util.h
 * @brief Utility functions for system info, memory, safety, and platform services
 * 
 * Defines abstract interface for miscellaneous platform services including hardware
 * identification, memory allocation, safety state management, and system information.
 * Accessed via hal.util singleton.
 */

#pragma once

#include <stdarg.h>
#include <AP_Common/AP_Common.h> // for FMT_PRINTF
#include "AP_HAL_Namespace.h"
#include <AP_Logger/AP_Logger_config.h>

class ExpandingString;

/**
 * @class AP_HAL::Util
 * @brief Platform utility functions and system services
 * 
 * @details Provides diverse services that don't fit other HAL interfaces:
 *          - Hardware identification: Board type, serial numbers, chip IDs
 *          - Memory management: Type-aware allocation for DMA, fast RAM regions
 *          - Safety state: Safety switch monitoring for motor arming
 *          - System information: Available RAM, reset reason, persistent data
 *          - String utilities: snprintf variants, printf redirection
 *          - Persistent storage: Last flight data across reboots
 *          - Command line: Arguments passed to firmware (SITL)
 *          - Tone/buzzer: Audio feedback control
 *          
 *          Memory regions (Memory_Type enum):
 *          - MEM_FAST: Fast access RAM (CCM on STM32, DTCM on Cortex-M7)
 *          - MEM_DMA_SAFE: DMA-accessible RAM (excludes CCM/TCM)
 *          - MEM_FILESYSTEM: Heap for filesystem operations
 *          
 *          Safety state: Tracks hardware safety switch preventing motor output.
 *          Used during vehicle setup, after crash, or emergency stop.
 * 
 * @note Most methods have sensible defaults for platforms without hardware features
 * @warning malloc_type() returns nullptr on allocation failure - always check
 */
class AP_HAL::Util {
public:
    /**
     * @brief Formatted string output with bounds checking
     * 
     * @param[out] str      Destination buffer
     * @param[in]  size     Buffer size in bytes
     * @param[in]  format   printf-style format string
     * @param[in]  ...      Variable arguments matching format
     * 
     * @return Number of characters written (excluding null terminator)
     * 
     * @note Always null-terminates output if size > 0
     */
    int snprintf(char* str, size_t size,
                 const char *format, ...) FMT_PRINTF(4, 5);

    /**
     * @brief Formatted string output with va_list arguments
     * 
     * @param[out] str      Destination buffer
     * @param[in]  size     Buffer size in bytes
     * @param[in]  format   printf-style format string
     * @param[in]  ap       Variable argument list
     * 
     * @return Number of characters written (excluding null terminator)
     * 
     * @note Always null-terminates output if size > 0
     */
    int vsnprintf(char* str, size_t size,
                  const char *format, va_list ap);

    /**
     * @brief Set software armed state
     * 
     * @param[in] b  true to arm, false to disarm
     * 
     * @details Software armed state indicates vehicle code is ready to output
     *          motor/servo commands. Different from hardware safety switch.
     *          Prevents actuator output before vehicle initialization complete.
     * 
     * @note Records timestamp of armed state change
     * @see get_soft_armed(), get_last_armed_change()
     */
    virtual void set_soft_armed(const bool b);
    
    /**
     * @brief Get current software armed state
     * 
     * @return true if software armed, false if disarmed
     * 
     * @note Software armed state != hardware safety switch state
     */
    bool get_soft_armed() const { return soft_armed; }

    /**
     * @brief Get timestamp of last armed state change
     * 
     * @return Milliseconds since boot when armed state last changed
     * 
     * @note Used to enforce minimum disarmed time, detect rapid arm/disarm cycles
     */
    uint32_t get_last_armed_change() const { return last_armed_change_ms; };

    /**
     * @brief Check if reboot was caused by watchdog reset
     * 
     * @return true if last reboot was watchdog timeout, false otherwise
     * 
     * @details Watchdog reset indicates firmware froze or crashed. Used to:
     *          - Restore persistent_data from previous flight
     *          - Log crash information for debugging
     *          - Trigger failsafe behaviors on restart
     * 
     * @note Platform-specific implementation checks reset reason register
     * @see persistent_data, last_persistent_data
     */
    virtual bool was_watchdog_reset() const { return false; }

    /**
     * @brief Check if watchdog reset occurred with safety switch armed
     * 
     * @return true if last boot was watchdog reset with motors enabled
     * 
     * @warning Indicates vehicle crashed with motors potentially running
     */
    bool was_watchdog_safety_off() const {
        return was_watchdog_reset() && persistent_data.safety_state == SAFETY_ARMED;
    }

    /**
     * @brief Check if watchdog reset occurred while vehicle was armed
     * 
     * @return true if last boot was watchdog reset while armed
     * 
     * @warning Indicates in-flight or armed crash condition
     */
    bool was_watchdog_armed() const { return was_watchdog_reset() && persistent_data.armed; }

    /**
     * @brief Get platform-specific log storage directory path
     * 
     * @return Custom log directory path, or nullptr for default
     * 
     * @note Linux/SITL: Allows logs in custom location (e.g., /var/APM/logs)
     * @note Embedded: Returns nullptr, uses onboard flash/SD card
     */
    virtual const char* get_custom_log_directory() const { return nullptr; }
    
    /**
     * @brief Get platform-specific terrain data directory path
     * 
     * @return Custom terrain directory path, or nullptr for default
     * 
     * @note Used for terrain database storage (SRTM elevation data)
     */
    virtual const char* get_custom_terrain_directory() const { return nullptr;  }
    
    /**
     * @brief Get platform-specific parameter/script storage directory
     * 
     * @return Custom storage directory path, or nullptr for default
     * 
     * @note Used for Lua scripts, parameter files on filesystem-enabled platforms
     */
    virtual const char *get_custom_storage_directory() const { return nullptr;  }

    /**
     * @brief Get path to default parameter file
     * 
     * @return Path to defaults.parm file, or nullptr if not used
     * 
     * @details Default parameter file loaded on first boot or parameter reset.
     *          Allows board-specific parameter defaults for tuning, hardware config.
     * 
     * @note Defined by HAL_PARAM_DEFAULTS_PATH at compile time
     */
    virtual const char* get_custom_defaults_file() const {
        return HAL_PARAM_DEFAULTS_PATH;
    }

    /**
     * @brief Apply command-line parameters to EEPROM on startup
     * 
     * @details SITL/Linux only: Allows setting parameters via command line
     *          for automated testing without modifying stored parameters.
     * 
     * @note Format: --param NAME=VALUE on command line
     */
    virtual void set_cmdline_parameters() {};

    /**
     * @enum safety_state
     * @brief Hardware safety switch state
     * 
     * @details Safety switch prevents motor/servo output even when armed.
     *          Common on multicopters for pre-flight safety.
     */
    enum safety_state : uint8_t {
        SAFETY_NONE,      ///< No safety switch hardware present
        SAFETY_DISARMED,  ///< Safety switch engaged - motors inhibited
        SAFETY_ARMED,     ///< Safety switch disengaged - motors can run
    };

    /**
     * @struct PersistentData
     * @brief Data preserved across watchdog resets for crash recovery
     * 
     * @details Stored in special RAM region that survives watchdog reset.
     *          Contains flight state at moment of crash for:
     *          - Crash log generation (attitude, position, fault address)
     *          - Fault analysis (thread name, scheduler task, fault type)
     *          - Mission resume (waypoint number)
     *          - Debugging (last MAVLink messages, peripheral counters)
     * 
     * @warning STM32 platforms limit this structure to 76 bytes total
     * @note Only valid if was_watchdog_reset() returns true
     * @see was_watchdog_reset(), last_persistent_data
     */
    struct PersistentData {
        float roll_rad, pitch_rad, yaw_rad;       ///< Vehicle attitude at crash (radians, NED frame)
        int32_t home_lat, home_lon, home_alt_cm;  ///< Home position (degrees*1e7, cm)
        uint32_t fault_addr;                      ///< Memory address of fault (ARM fault registers)
        uint32_t fault_icsr;                      ///< Interrupt Control/State Register at fault
        uint32_t fault_lr;                        ///< Link Register (return address) at fault
        uint32_t internal_errors;                 ///< Bitmask of internal error flags
        uint16_t internal_error_count;            ///< Total internal errors detected
        uint16_t internal_error_last_line;        ///< Source line of last internal error
        uint32_t spi_count;                       ///< SPI transaction counter
        uint32_t i2c_count;                       ///< I2C transaction counter
        uint32_t i2c_isr_count;                   ///< I2C interrupt count
        uint16_t waypoint_num;                    ///< Current mission waypoint number
        uint16_t last_mavlink_msgid;              ///< Last MAVLink message ID received
        uint16_t last_mavlink_cmd;                ///< Last MAVLink command ID processed
        uint16_t semaphore_line;                  ///< Source line if fault in semaphore
        uint16_t fault_line;                      ///< Source line where fault occurred
        uint8_t fault_type;                       ///< Fault type code (ARM exception number)
        uint8_t fault_thd_prio;                   ///< Thread priority at fault
        char thread_name4[4];                     ///< First 4 chars of thread name at fault
        int8_t scheduler_task;                    ///< Scheduler task ID running at fault (-1 if none)
        bool armed;                               ///< true if vehicle was armed at crash
        enum safety_state safety_state;           ///< Safety switch state at crash
        bool boot_to_dfu;                         ///< true to reboot to DFU bootloader mode
    };
    
    struct PersistentData persistent_data;        ///< Current flight persistent data (updated continuously)
    
    /**
     * @brief Persistent data from previous boot
     * 
     * @details Only valid if was_watchdog_reset() is true. Contains complete
     *          vehicle state at moment of crash for post-crash analysis.
     * 
     * @note Copied from persistent_data early in boot sequence
     */
    struct PersistentData last_persistent_data;

    /**
     * @brief Get hardware safety switch state
     * 
     * @return Current safety switch state
     * 
     * @details Hardware safety switch (external button) prevents motor output.
     *          Must be pressed to transition from SAFETY_DISARMED to SAFETY_ARMED.
     *          Returns SAFETY_NONE if platform has no safety switch.
     * 
     * @note Different from software armed state (get_soft_armed)
     * @note Common on Pixhawk-series flight controllers
     * @see safety_state enum
     */
    virtual enum safety_state safety_switch_state(void) { return SAFETY_NONE; }

    /**
     * @brief Set hardware real-time clock
     * 
     * @param[in] time_utc_usec  UTC time in microseconds since Unix epoch
     * 
     * @details Sets hardware RTC for platforms with battery-backed clock.
     *          Time typically sourced from GPS once fix is acquired.
     * 
     * @note Pure virtual - must be implemented by platform HAL
     */
    virtual void set_hw_rtc(uint64_t time_utc_usec) = 0;

    /**
     * @brief Get hardware real-time clock value
     * 
     * @return UTC time in microseconds since Unix epoch
     * 
     * @details Reads hardware RTC. Returns 0 if RTC not set or unavailable.
     * 
     * @note Pure virtual - must be implemented by platform HAL
     */
    virtual uint64_t get_hw_rtc() const = 0;

    /**
     * @enum FlashBootloader
     * @brief Result codes for bootloader update operation
     */
    enum class FlashBootloader {
        OK=0,              ///< Bootloader successfully updated
        NO_CHANGE=1,       ///< Bootloader already up-to-date
        FAIL=2,            ///< Update failed (flash error, verification failed)
        NOT_AVAILABLE=3,   ///< Bootloader update not supported on this platform
        NOT_SIGNED=4,      ///< Bootloader not signed (secure boot enabled)
    };

    /**
     * @brief Update bootloader from ROMFS or storage
     * 
     * @return Result code indicating success/failure
     * 
     * @details Overwrites bootloader in flash with new version from:
     *          - ROMFS (embedded in firmware)
     *          - SD card file
     * 
     * @warning Interrupting this operation can brick the board
     * @note Only supported on select platforms (STM32 with bootloader support)
     */
    virtual FlashBootloader flash_bootloader() { return FlashBootloader::NOT_AVAILABLE; }

    /**
     * @brief Get system identifier as printable string
     * 
     * @param[out] buf  Buffer for formatted serial number (50 bytes)
     * 
     * @return true if system ID available, false otherwise
     * 
     * @details Returns board-specific unique identifier:
     *          - STM32: 96-bit unique device ID formatted as hex string
     *          - Linux: System hostname or machine ID
     *          Used for vehicle identification in logs, MAVLink AUTOPILOT_VERSION
     * 
     * @note Buffer always null-terminated on success
     */
    virtual bool get_system_id(char buf[50]) { return false; }
    
    /**
     * @brief Get system identifier as raw binary
     * 
     * @param[out] buf  Buffer for unformatted ID bytes
     * @param[out] len  Length of ID data written to buffer
     * 
     * @return true if system ID available, false otherwise
     * 
     * @note Useful for cryptographic operations requiring raw chip ID
     */
    virtual bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) { return false; }

    /**
     * @brief Get command line arguments passed to firmware
     * 
     * @param[out] argc  Number of arguments
     * @param[out] argv  Array of argument strings
     * 
     * @details SITL/Linux only: Returns argc/argv passed to main().
     *          Used for parameter overrides, simulation options, debug flags.
     * 
     * @note Embedded platforms: Returns argc=0 (no command line)
     * @see set_cmdline_parameters()
     */
    virtual void commandline_arguments(uint8_t &argc, char * const *&argv) { argc = 0; }

    /**
     * @brief Initialize tone alarm/buzzer hardware
     * 
     * @param[in] types  Bitmask of tone alarm types to enable
     * 
     * @return true if initialization successful
     * 
     * @note Platform-specific: GPIO buzzer, PWM buzzer, or external I2C/DroneCAN device
     */
    virtual bool toneAlarm_init(uint8_t types) { return false;}
    
    /**
     * @brief Play tone on buzzer
     * 
     * @param[in] frequency    Tone frequency in Hz (0 = silence)
     * @param[in] volume       Volume 0.0-1.0
     * @param[in] duration_ms  Tone duration in milliseconds
     * 
     * @note Used for arming tones, warnings, battery alerts
     */
    virtual void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) {}

    /**
     * @brief Report current IMU temperature for heater control
     * 
     * @param[in] current  Current IMU temperature in degrees Celsius
     * 
     * @details Used by IMU heater control loop on boards with temperature sensors.
     *          Enables IMU thermal calibration and temperature stabilization.
     */
    virtual void set_imu_temp(float current) {}

    /**
     * @brief Set target temperature for IMU heater
     * 
     * @param[in] target  Pointer to target temperature in degrees Celsius
     * 
     * @details Configures IMU heater target temperature. Heater PID controller
     *          maintains IMU at constant temperature for reduced bias drift.
     * 
     * @note Only on boards with IMU heater hardware (select Pixhawk variants)
     */
    virtual void set_imu_target_temp(int8_t *target) {}
    
    /**
     * @enum Memory_Type
     * @brief Memory region types for type-aware allocation
     * 
     * @details ARM Cortex-M processors have multiple RAM regions with different properties:
     *          - CCM (Core Coupled Memory): Fast but not DMA-accessible on STM32F4
     *          - DTCM (Data Tightly Coupled Memory): Fast, not DMA-safe on STM32F7/H7
     *          - Main RAM: Slower but DMA-accessible
     *          - External RAM: Large but slowest (SDRAM on some boards)
     */
    enum Memory_Type {
        MEM_DMA_SAFE,      ///< DMA-accessible RAM (required for peripheral I/O buffers)
        MEM_FAST,          ///< Fastest RAM (CCM/DTCM) for performance-critical data structures
        MEM_FILESYSTEM     ///< Heap for filesystem operations (may be external SDRAM)
    };
    
    /**
     * @brief Allocate memory from specified memory region
     * 
     * @param[in] size      Number of bytes to allocate
     * @param[in] mem_type  Memory region constraint
     * 
     * @return void* Pointer to allocated memory, or nullptr on failure
     * 
     * @details Platform HAL routes allocation to appropriate memory region:
     *          - MEM_FAST: CCM on STM32F4, DTCM on F7/H7 (for FFT buffers, sensor data)
     *          - MEM_DMA_SAFE: Main RAM accessible by DMA peripherals (SPI, I2C, UART buffers)
     *          - MEM_FILESYSTEM: May use external SDRAM if available
     * 
     * @note Default implementation: calloc() from heap (zeroed memory)
     * @warning Always check for nullptr - allocation can fail on memory exhaustion
     * @see free_type()
     */
    virtual void *malloc_type(size_t size, Memory_Type mem_type) { return calloc(1, size); }
    
    /**
     * @brief Free memory allocated by malloc_type
     * 
     * @param[in] ptr       Pointer to memory to free (from malloc_type)
     * @param[in] size      Original allocation size in bytes
     * @param[in] mem_type  Original memory type (must match malloc_type call)
     * 
     * @note size and mem_type must match original malloc_type() call
     * @note Platform-specific allocators may use size for statistics or validation
     * @see malloc_type()
     */
    virtual void free_type(void *ptr, size_t size, Memory_Type mem_type) { return free(ptr); }

    /**
     * @brief Get available free memory
     * 
     * @return Free memory in bytes
     * 
     * @details Returns estimate of available heap memory. Used for:
     *          - Memory health monitoring
     *          - Logging memory exhaustion events
     *          - Limiting memory-intensive operations
     * 
     * @note Default returns 4096 if platform cannot determine free memory
     * @note May not include all memory regions (e.g., may report only main heap)
     */
    virtual uint32_t available_memory(void) { return 4096; }

    /**
     * @brief Trigger processor trap/breakpoint for debugger attachment
     * 
     * @return true if trap successful, false if not supported
     * 
     * @details Attempts to halt processor execution to allow debugger attachment:
     *          - ARM: Executes BKPT instruction
     *          - SITL: Raises SIGTRAP signal
     * 
     * @note Only effective if hardware debugger (JTAG/SWD) connected
     */
    virtual bool trap() const { return false; }

    /**
     * @brief Get information on running threads/tasks
     * 
     * @param[out] str  Expanding string to append thread information
     * 
     * @details Appends human-readable thread information:
     *          - Thread name and priority
     *          - Stack usage and high-water mark
     *          - CPU time consumed
     *          - Current state (running, blocked, sleeping)
     * 
     * @note Used by THREADS MAVLink message and @SYS/threads.txt logging
     */
    virtual void thread_info(ExpandingString &str) {}

    /**
     * @brief Get DMA channel contention statistics
     * 
     * @param[out] str  Expanding string to append DMA information
     * 
     * @details Reports DMA channel usage and contention:
     *          - Active DMA streams/channels
     *          - Transfer counts and errors
     *          - Contention events (multiple peripherals requesting same channel)
     * 
     * @note Helps diagnose SPI/I2C performance issues
     */
    virtual void dma_info(ExpandingString &str) {}

    /**
     * @brief Get memory allocation statistics
     * 
     * @param[out] str  Expanding string to append memory information
     * 
     * @details Reports memory usage by region:
     *          - Total/free/used for each memory type (FAST, DMA_SAFE, FILESYSTEM)
     *          - Largest free block
     *          - Allocation failures
     * 
     * @note Critical for diagnosing memory exhaustion issues
     */
    virtual void mem_info(ExpandingString &str) {}

    /**
     * @brief Load persistent parameters from bootloader sector
     * 
     * @param[out] str  Expanding string to append loaded parameter info
     * 
     * @return true if parameters loaded successfully
     * 
     * @details Some platforms store parameters in bootloader flash sector
     *          (separate from main parameter storage) for:
     *          - Board configuration (CAN node ID, hardware variant)
     *          - Serial numbers and calibration data
     * 
     * @note Typically called early in boot sequence
     */
    virtual bool load_persistent_params(ExpandingString &str) const { return false; }

    /**
     * @brief Retrieve persistent parameter value by name
     * 
     * @param[in]  name   Parameter name to retrieve
     * @param[out] value  Buffer for parameter value string
     * @param[out] len    Length of value written
     * 
     * @return true if parameter found, false otherwise
     * 
     * @note Used for bootloader-stored configuration separate from AP_Param
     */
    virtual bool get_persistent_param_by_name(const char *name, char* value, size_t& len) const {
        return false;
    }

#if HAL_UART_STATS_ENABLED
    /**
     * @brief Get UART I/O statistics
     * 
     * @param[out] str  Expanding string to append UART information
     * 
     * @details Reports per-UART statistics:
     *          - Bytes transmitted/received
     *          - Buffer overruns and framing errors
     *          - Baud rate and flow control status
     *          - DMA vs interrupt transfer mode
     * 
     * @note Used for diagnosing telemetry link issues
     */
    virtual void uart_info(ExpandingString &str) {}

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log UART statistics for all serial ports
     * 
     * @details Writes UART statistics to dataflash log (UART message type).
     *          Called periodically by logging system for performance analysis.
     */
    virtual void uart_log() {};
#endif
#endif // HAL_UART_STATS_ENABLED

    /**
     * @brief Get timer/PWM frequency information
     * 
     * @param[out] str  Expanding string to append timer information
     * 
     * @details Reports timer configuration:
     *          - Timer frequencies for PWM outputs
     *          - Scheduler timing source configuration
     *          - Timer conflicts or sharing issues
     * 
     * @note Helps diagnose ESC compatibility and PWM output issues
     */
    virtual void timer_info(ExpandingString &str) {}

    /**
     * @brief Generate random data (non-cryptographic)
     * 
     * @param[out] data  Buffer for random data
     * @param[in]  size  Number of random bytes to generate
     * 
     * @return true if random data generated, false if not available
     * 
     * @details Uses platform random number generator (may be pseudo-random).
     *          Suitable for non-security purposes (random delays, testing).
     * 
     * @note NOT cryptographically secure - use get_true_random_vals() for security
     */
    virtual bool get_random_vals(uint8_t* data, size_t size) { return false; }

    /**
     * @brief Generate cryptographically secure random data
     * 
     * @param[out] data        Buffer for random data
     * @param[in]  size        Number of random bytes to generate
     * @param[in]  timeout_us  Maximum time to wait for entropy (microseconds)
     * 
     * @return true if random data generated, false on timeout or not available
     * 
     * @details Uses hardware random number generator (RNG) with entropy source.
     *          Blocks until sufficient entropy available or timeout expires.
     *          Required for cryptographic operations (key generation, nonces).
     * 
     * @warning May block for extended periods if entropy pool is depleted
     * @note STM32: Uses hardware RNG peripheral
     * @note SITL/Linux: Uses /dev/urandom
     */
    virtual bool get_true_random_vals(uint8_t* data, size_t size, uint32_t timeout_us) { return false; }

    /**
     * @brief Log stack usage information for all threads
     * 
     * @details Writes stack high-water marks to dataflash log.
     *          Used to determine appropriate stack sizes and detect overflow risk.
     * 
     * @note Logged periodically during flight for post-flight analysis
     */
    virtual void log_stack_info(void) {}

#if AP_CRASHDUMP_ENABLED
    /**
     * @brief Get size of crash dump from previous boot
     * 
     * @return Size of crash dump in bytes, or 0 if no crash dump available
     * 
     * @details If previous boot ended in crash, returns size of preserved
     *          crash dump data (stack trace, register state, memory snapshot).
     * 
     * @see last_crash_dump_ptr()
     */
    virtual size_t last_crash_dump_size() const { return 0; }
    
    /**
     * @brief Get pointer to crash dump from previous boot
     * 
     * @return Pointer to crash dump data, or nullptr if unavailable
     * 
     * @details Crash dump contains:
     *          - CPU registers at fault
     *          - Stack trace
     *          - Thread state
     *          - Partial memory snapshot
     * 
     * @note Data valid only if last_crash_dump_size() > 0
     * @see last_crash_dump_size()
     */
    virtual void* last_crash_dump_ptr() const { return nullptr; }
#endif

#if HAL_ENABLE_DFU_BOOT
    /**
     * @brief Reboot into DFU (Device Firmware Update) bootloader mode
     * 
     * @details Forces reboot into USB DFU bootloader for firmware update.
     *          Used for recovery when normal bootloader communication fails.
     * 
     * @warning Does not return - triggers immediate reboot
     * @note STM32 only: Enters ROM bootloader supporting DFU protocol
     */
    virtual void boot_to_dfu(void) {}
#endif
protected:
    /**
     * @brief Software armed state
     * 
     * @details Initialized false to prevent actuator output before vehicle
     *          code fully initialized. Prevents motor spin-up during boot.
     */
    bool soft_armed = false;
    
    /**
     * @brief Timestamp of last armed state change
     * 
     * @details Milliseconds since boot when soft_armed last changed.
     *          Used to enforce minimum disarmed time before re-arming.
     */
    uint32_t last_armed_change_ms;
};

/**
 * @brief Stack overflow handler callback
 * 
 * @param[in] thread_name  Name of thread with stack overflow
 * 
 * @details Called when thread stack overflow detected by:
 *          - Stack canary corruption
 *          - Stack pointer out of bounds
 *          - Memory guard violation
 * 
 * @warning Critical error - typically triggers reboot or panic
 * @note Implemented by vehicle code to log error and trigger failsafe
 */
extern "C" {
    void AP_stack_overflow(const char *thread_name);
    
    /**
     * @brief Memory guard violation handler
     * 
     * @param[in] size  Size of memory access that triggered violation
     * 
     * @details Called when memory guard detects:
     *          - Buffer overrun
     *          - Heap corruption
     *          - Invalid memory access
     * 
     * @warning Critical error - indicates memory corruption
     */
    void AP_memory_guard_error(uint32_t size);
}
