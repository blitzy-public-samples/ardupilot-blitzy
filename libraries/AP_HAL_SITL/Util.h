/**
 * @file Util.h
 * @brief Utility functions implementation for SITL (Software In The Loop) HAL
 * 
 * @details This file provides the SITL-specific implementation of the AP_HAL::Util
 *          interface, offering utility functions for simulation environments. Unlike
 *          real hardware implementations, SITL Util simulates hardware features like
 *          RTC (Real-Time Clock), system ID generation, random number generation,
 *          and memory availability without actual hardware constraints.
 * 
 *          Key simulation features:
 *          - Hardware RTC simulation using system time
 *          - Multi-instance SITL system ID generation
 *          - Unbounded memory reporting (simulation assumption)
 *          - Command-line parameter override mechanism
 *          - Safety switch state simulation
 *          - Tone alarm interface for audio feedback simulation
 *          - Random number generation for stochastic simulation
 * 
 *          This implementation integrates with SITL_State to access simulation
 *          parameters and configuration overrides from command-line arguments.
 * 
 * @note This is a simulation-only implementation and behavior may differ
 *       significantly from real hardware HAL implementations
 * 
 * @see AP_HAL::Util for the abstract interface definition
 * @see SITL_State for simulation state management
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_SITL_Namespace.h"
#include "AP_HAL_SITL.h"
#include "Semaphores.h"
#include "ToneAlarm_SF.h"
#include <AP_Logger/AP_Logger_config.h>

#if !defined(__CYGWIN__) && !defined(__CYGWIN64__)
#include <sys/types.h>
#include <signal.h>
#endif

/**
 * @class HALSITL::Util
 * @brief SITL-specific implementation of utility functions for simulation environments
 * 
 * @details This class provides simulation-specific implementations of utility functions
 *          defined in the AP_HAL::Util interface. It simulates hardware features without
 *          the constraints of real embedded systems, enabling software-in-the-loop testing
 *          of ArduPilot code.
 * 
 *          Unlike real hardware implementations:
 *          - Memory is reported as unlimited (512KB default)
 *          - Hardware RTC uses system time instead of hardware clock
 *          - Random number generation uses system pseudo-random functions
 *          - System ID is generated from SITL instance identifier
 *          - Safety switch state can be simulated via environment variables
 * 
 *          The class integrates with SITL_State to access simulation parameters,
 *          custom defaults files, and command-line configuration overrides. This
 *          enables flexible simulation scenarios without recompiling.
 * 
 *          Thread Safety: Methods that access SITL_State should be thread-safe
 *          as SITL_State is shared across the simulation.
 * 
 * @note This implementation is only compiled for SITL builds and is not available
 *       on real hardware platforms
 */
class HALSITL::Util : public AP_HAL::Util {
public:
    /**
     * @brief Constructor for SITL Util implementation
     * 
     * @param[in] _sitlState Pointer to SITL simulation state object that provides
     *                       access to simulation parameters, defaults file path,
     *                       and other simulation-specific configuration
     * 
     * @note The SITL_State pointer must remain valid for the lifetime of this object
     */
    Util(SITL_State *_sitlState) :
        sitlState(_sitlState) {}
    
    /**
     * @brief Report available memory in bytes (simulation reports unlimited memory)
     * 
     * @details In SITL simulation, memory is not constrained like embedded hardware.
     *          This method returns a fixed value of 512KB to satisfy callers that
     *          check memory availability, but actual memory usage is limited only
     *          by the host system running the simulation.
     * 
     *          On real hardware, this would report actual free heap or RAM, but
     *          SITL assumes unlimited memory availability for testing purposes.
     * 
     * @return uint32_t Always returns 524288 (512 * 1024) bytes
     * 
     * @note This is a simulation convenience - actual memory usage can exceed
     *       this value without issues, unlike real hardware implementations
     * 
     * @see AP_HAL::Util::available_memory() for the interface definition
     */
    uint32_t available_memory(void) override {
        // SITL is assumed to always have plenty of memory. Return 512k for now
        return 512*1024;
    }

    /**
     * @brief Get path to custom defaults file for AP_Param system
     * 
     * @details Returns the path to a custom parameter defaults file specified
     *          via SITL command-line arguments (typically --defaults option).
     *          This allows simulation scenarios to load custom parameter sets
     *          without modifying default values in the code.
     * 
     *          The defaults file contains parameter name=value pairs that
     *          override built-in defaults during AP_Param initialization.
     * 
     * @return const char* Path to defaults file, or nullptr if no custom
     *                     defaults file was specified on command line
     * 
     * @note The returned pointer references SITL_State storage and remains
     *       valid for the simulation lifetime
     * 
     * @see SITL_State::defaults_path for command-line argument handling
     */
    const char* get_custom_defaults_file() const override {
        return sitlState->defaults_path;
    }

    /**
     * @brief Return command-line arguments passed to SITL simulator
     * 
     * @details Provides access to the original command-line arguments used to
     *          launch the SITL simulation. This enables runtime configuration
     *          and parameter override without recompilation. Arguments are
     *          saved during init() and remain available throughout simulation.
     * 
     *          Common SITL arguments include:
     *          - Vehicle type and model selection
     *          - Parameter defaults file path
     *          - Home location coordinates
     *          - Simulation speed multiplier
     *          - Network configuration
     * 
     * @param[out] argc Number of command-line arguments (including program name)
     * @param[out] argv Pointer to array of argument strings (null-terminated C strings)
     * 
     * @note Arguments are stored in their original form without parsing or modification
     * @see init() for argument storage during initialization
     */
    void commandline_arguments(uint8_t &argc, char * const *&argv) override;
    
    /**
     * @brief Get hardware real-time clock value (simulated using system time)
     * 
     * @details In SITL, hardware RTC is simulated using the host system's clock.
     *          This returns the current system time in microseconds since Unix epoch
     *          (00:00:00 UTC on 1 January 1970), simulating what a hardware RTC
     *          would provide on real flight controllers.
     * 
     *          Unlike real hardware where RTC maintains time during power-off, SITL
     *          RTC always reflects the current system time when queried.
     * 
     * @return uint64_t Current system time in microseconds since Unix epoch
     * 
     * @note This is a simulation convenience - time jumps if system time changes,
     *       unlike battery-backed hardware RTC which maintains continuous time
     * 
     * @see set_hw_rtc() for the complementary setter (no-op in SITL)
     */
    uint64_t get_hw_rtc() const override;
    
    /**
     * @brief Set hardware real-time clock value (no-op in SITL simulation)
     * 
     * @details In real hardware, this would set the battery-backed RTC to a specific
     *          time (typically from GPS). In SITL, this is a no-op since system time
     *          is used directly and cannot be modified by the simulation.
     * 
     *          This method exists to satisfy the AP_HAL::Util interface but has no
     *          effect in simulation environments.
     * 
     * @param[in] time_utc_usec Time to set in microseconds since Unix epoch (ignored)
     * 
     * @note Fails silently as indicated by empty implementation
     * @see get_hw_rtc() for reading simulated hardware RTC
     */
    void set_hw_rtc(uint64_t time_utc_usec) override { /* fail silently */ }


    /**
     * @brief Get system identifier string for this SITL instance (formatted)
     * 
     * @details Generates a unique system ID string for multi-instance SITL scenarios
     *          where multiple simulated vehicles run simultaneously. The ID is derived
     *          from the SITL instance number and formatted as a human-readable string.
     * 
     *          This enables identification and differentiation of multiple simulated
     *          vehicles in testing environments, logs, and network communications.
     *          Format is implementation-specific but typically includes instance number.
     * 
     * @param[out] buf Buffer to store formatted system ID string (null-terminated).
     *                 Must be at least 50 bytes to accommodate full ID string.
     * 
     * @return bool True if system ID was successfully generated and stored,
     *              false on failure
     * 
     * @note Buffer size is fixed at 50 bytes per AP_HAL::Util interface
     * @see get_system_id_unformatted() for raw binary system ID
     */
    bool get_system_id(char buf[50]) override;
    
    /**
     * @brief Get system identifier in unformatted binary form
     * 
     * @details Provides the system ID as raw binary data rather than formatted string.
     *          This is useful for protocols requiring binary identifiers or when
     *          minimizing data size is important. The binary format matches what
     *          hardware implementations would provide from chip unique IDs.
     * 
     * @param[out] buf Buffer to store unformatted system ID bytes
     * @param[in,out] len Input: Maximum buffer size in bytes
     *                    Output: Actual number of bytes written to buffer
     * 
     * @return bool True if system ID was successfully generated, false on failure
     *              (e.g., buffer too small)
     * 
     * @see get_system_id() for formatted string version
     */
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;
    
    /**
     * @brief Dump current stack trace for debugging
     * 
     * @details Prints the current call stack to aid in debugging crashes, hangs,
     *          or unexpected code paths during SITL simulation. Uses platform-specific
     *          mechanisms (e.g., backtrace() on Linux) to unwind the stack and
     *          display function names and addresses.
     * 
     *          This is particularly useful when debugging complex control flow or
     *          investigating where execution is when a watchdog or timeout occurs.
     * 
     * @note Output goes to stderr or simulation console
     * @note Stack trace quality depends on debug symbols and platform support
     */
    void dump_stack_trace();

#ifdef WITH_SITL_TONEALARM
    /**
     * @brief Initialize tone alarm subsystem for audio feedback simulation
     * 
     * @details Initializes the SITL tone alarm (buzzer) interface to simulate
     *          audio feedback that would be generated by a buzzer on real hardware.
     *          In SITL, this may play audio through the host system's sound card
     *          or log tone commands for analysis.
     * 
     *          Tone alarms are used for status indication (arming, failsafe, etc.)
     *          and user feedback during vehicle operation.
     * 
     * @param[in] types Bitmask of tone alarm types to enable (currently unused,
     *                  reserved for future multi-output support)
     * 
     * @return bool True if tone alarm initialized successfully, false on failure
     * 
     * @note Only available when compiled WITH_SITL_TONEALARM
     * @see toneAlarm_set_buzzer_tone() for playing tones
     */
    bool toneAlarm_init(uint8_t types) override { return _toneAlarm.init(); }
    
    /**
     * @brief Play a tone on the simulated buzzer
     * 
     * @details Commands the tone alarm to play a specific frequency for a given
     *          duration at specified volume. In SITL, this may generate actual
     *          audio output or log the tone for verification in automated testing.
     * 
     *          Used for audio feedback patterns like arming confirmation,
     *          battery warnings, GPS lock indication, and failsafe alerts.
     * 
     * @param[in] frequency Tone frequency in Hertz (Hz). Typical range: 100-10000 Hz.
     *                      Common values: 1000 Hz for neutral, higher for urgency.
     * @param[in] volume Volume level from 0.0 (silent) to 1.0 (maximum).
     *                   May be used for attenuation if host audio output is loud.
     * @param[in] duration_ms Duration to play tone in milliseconds. 0 stops current tone.
     * 
     * @note Only available when compiled WITH_SITL_TONEALARM
     * @note Overlapping calls may interrupt the current tone depending on implementation
     * @see ToneAlarm_SF for the underlying tone generation implementation
     */
    void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override {
        _toneAlarm.set_buzzer_tone(frequency, volume, duration_ms);
    }
#endif

    /**
     * @brief Check if reboot was caused by watchdog reset
     * 
     * @details Returns true if the current SITL simulation was started after a
     *          simulated watchdog reset. In SITL, this is controlled by the
     *          SITL_WATCHDOG_RESET environment variable, enabling testing of
     *          watchdog recovery code paths without actual hardware resets.
     * 
     *          On real hardware, this would read reset reason registers. In SITL,
     *          set the environment variable before launch to simulate watchdog reset:
     *          
     *              export SITL_WATCHDOG_RESET=1
     *              sim_vehicle.py ...
     * 
     * @return bool True if SITL_WATCHDOG_RESET environment variable is set,
     *              false otherwise (normal startup)
     * 
     * @note Used to test watchdog recovery logic and crash handling code
     * @see AP_HAL::Util::was_watchdog_reset() for interface definition
     */
    bool was_watchdog_reset() const override { return getenv("SITL_WATCHDOG_RESET") != nullptr; }

#if !defined(HAL_BUILD_AP_PERIPH)
    /**
     * @brief Get current state of simulated safety switch
     * 
     * @details Returns the state of the safety switch simulation in SITL. The safety
     *          switch is a critical hardware safety feature that prevents motor arming
     *          until explicitly enabled. In SITL, the switch state can be controlled
     *          via simulation parameters or environment variables for testing.
     * 
     *          Safety switch states:
     *          - SAFETY_NONE: No safety switch present
     *          - SAFETY_DISARMED: Safety engaged, motors cannot spin
     *          - SAFETY_ARMED: Safety released, motors can spin when armed
     * 
     *          This enables testing of safety switch integration without physical
     *          hardware, including safety switch failsafe behavior.
     * 
     * @return enum safety_state Current simulated safety switch state
     * 
     * @note Only available in non-AP_Periph builds
     * @note State may be controlled through SITL_State or interactive commands
     * @see AP_HAL::Util::safety_state for state enumeration
     */
    enum safety_state safety_switch_state(void) override;
    
    /**
     * @brief Apply command-line parameters to override default values
     * 
     * @details Processes command-line arguments to set ArduPilot parameters at
     *          simulation startup. This mechanism allows parameter customization
     *          without modifying defaults files or using GCS commands, streamlining
     *          automated testing and development workflows.
     * 
     *          Command-line parameters are typically specified as:
     *              --param NAME=VALUE
     * 
     *          This method parses stored command-line arguments and applies parameter
     *          overrides through the AP_Param system. Called during initialization
     *          after AP_Param is ready but before main vehicle code starts.
     * 
     * @note Only available in non-AP_Periph builds
     * @note Parameter changes from command line take precedence over defaults file
     * @see commandline_arguments() for accessing raw argument data
     * @see SITL_State for parameter storage mechanism
     */
    void set_cmdline_parameters() override;
#endif

    /**
     * @brief Trigger debugger trap / breakpoint for debugging
     * 
     * @details Sends a SIGTRAP signal to break into an attached debugger, enabling
     *          interactive debugging at specific code locations. This is useful for
     *          debugging SITL simulations under GDB or other debuggers without
     *          manually setting breakpoints.
     * 
     *          On Cygwin, this operation is not supported and returns false.
     *          On Linux/macOS, sends SIGTRAP to the current process group.
     * 
     *          Typical usage: Call when a specific condition is met to halt
     *          execution and inspect state in debugger.
     * 
     * @return bool True if SIGTRAP was successfully sent (debugger should break),
     *              false if operation failed or is unsupported on this platform
     * 
     * @note Returns false on Cygwin platforms where SIGTRAP is not reliably supported
     * @note Process must be running under a debugger or have a signal handler installed
     * @see dump_stack_trace() for non-interactive debugging information
     */
    bool trap() const override {
#if defined(__CYGWIN__) || defined(__CYGWIN64__)
        return false;
#else
        if (kill(0, SIGTRAP) == -1) {
            return false;
        }
        return true;
#endif
    }

    /**
     * @brief Initialize Util with command-line arguments
     * 
     * @details Stores command-line arguments passed to the SITL simulator for later
     *          access by commandline_arguments() and parameter processing. This must
     *          be called early in SITL initialization before any code needs access
     *          to command-line configuration.
     * 
     *          Arguments are stored as-is without parsing or validation. Parsing
     *          happens later during set_cmdline_parameters() after AP_Param is ready.
     * 
     * @param[in] argc Number of command-line arguments including program name
     * @param[in] argv Array of null-terminated argument strings. Pointer must remain
     *                 valid for the lifetime of the simulation as it is not copied.
     * 
     * @note Arguments are stored by reference, not copied - caller must ensure
     *       argv remains valid throughout simulation lifetime
     * @see commandline_arguments() for retrieving stored arguments
     * @see set_cmdline_parameters() for parameter application
     */
    void init(int argc, char *const *argv) {
        saved_argc = argc;
        saved_argv = argv;
    }

    /**
     * @brief Fill buffer with pseudo-random values for stochastic simulation
     * 
     * @details Generates random bytes for use in simulation, such as sensor noise,
     *          communication jitter, or stochastic disturbances. Uses the host
     *          system's pseudo-random number generator (PRNG) to fill the provided
     *          buffer with random data.
     * 
     *          In SITL, this enables realistic simulation of:
     *          - Sensor measurement noise and drift
     *          - Communication timing variations
     *          - Environmental disturbances (wind gusts, turbulence)
     *          - Monte Carlo testing scenarios
     * 
     *          The PRNG may be seeded for reproducible simulations or left unseeded
     *          for varied testing.
     * 
     * @param[out] data Pointer to buffer to fill with random bytes
     * @param[in] size Number of random bytes to generate
     * 
     * @return bool True if random values were successfully generated, false on failure
     * 
     * @warning DO NOT use for cryptographic purposes or security-sensitive operations.
     *          This uses a pseudo-random number generator suitable only for simulation
     *          and testing, not for generating keys, tokens, or security-critical values.
     * 
     * @note Random quality depends on host system's PRNG implementation
     * @see AP_HAL::Util::get_random_vals() for interface definition
     */
    bool get_random_vals(uint8_t* data, size_t size) override;

private:
    /**
     * @brief Pointer to SITL simulation state
     * 
     * @details Provides access to simulation configuration, parameters, defaults
     *          file path, and other SITL-specific state. This pointer is set during
     *          construction and must remain valid for the object's lifetime.
     */
    SITL_State *sitlState;

#ifdef WITH_SITL_TONEALARM
    /**
     * @brief Static tone alarm instance for audio feedback simulation
     * 
     * @details Single shared tone alarm for all SITL Util instances. Static to
     *          avoid multiple audio streams when running multiple simulated vehicles.
     */
    static ToneAlarm_SF _toneAlarm;
#endif

    /**
     * @brief Saved count of command-line arguments
     * 
     * @details Number of arguments in saved_argv, including program name.
     *          Stored during init() for later retrieval by commandline_arguments().
     */
    int saved_argc;
    
    /**
     * @brief Saved pointer to command-line argument array
     * 
     * @details Array of null-terminated argument strings passed at SITL startup.
     *          Stored during init() and must remain valid for simulation lifetime
     *          as it is not copied. Accessed by commandline_arguments() and
     *          set_cmdline_parameters().
     */
    char *const *saved_argv;

#if HAL_UART_STATS_ENABLED
    /**
     * @brief Generate UART I/O statistics report string
     * 
     * @details Collects and formats statistics for all simulated serial ports,
     *          including bytes transmitted/received, buffer utilization, and
     *          error counts. Used for performance monitoring and debugging
     *          communication issues in SITL.
     * 
     *          Output includes per-port statistics:
     *          - TX/RX byte counts
     *          - Buffer overflow events
     *          - Communication errors
     *          - Timing information
     * 
     * @param[in,out] str Expanding string buffer to append UART statistics.
     *                    Statistics are formatted as human-readable text.
     * 
     * @note Only available when HAL_UART_STATS_ENABLED is defined
     * @see uart_log() for logging statistics to dataflash
     */
    void uart_info(ExpandingString &str) override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log UART statistics for each serial port to dataflash
     * 
     * @details Records UART I/O statistics to binary logs for post-flight analysis
     *          and performance troubleshooting. Creates log messages containing
     *          byte counts, error rates, and timing information for each configured
     *          serial port.
     * 
     *          Logged data enables analysis of:
     *          - Communication bandwidth usage
     *          - Buffer overflow frequency
     *          - Protocol efficiency
     *          - Timing characteristics
     * 
     * @note Only available when both HAL_UART_STATS_ENABLED and
     *       HAL_LOGGING_ENABLED are defined
     * @note Called periodically by scheduler, not every loop
     * @see uart_info() for human-readable statistics output
     */
    void uart_log() override;
#endif
#endif // HAL_UART_STATS_ENABLED

private:
#if HAL_UART_STATS_ENABLED
    /**
     * @struct uart_stats
     * @brief UART statistics tracking structure
     * 
     * @details Holds statistics for all serial ports and timing information.
     *          Two instances are maintained: one for system info reporting
     *          (sys_uart_stats) and one for dataflash logging (log_uart_stats).
     *          Separate instances allow different update rates and prevent
     *          interference between reporting mechanisms.
     */
    struct uart_stats {
        /** Statistics tracker for each serial port (0 to num_serial-1) */
        AP_HAL::UARTDriver::StatsTracker serial[AP_HAL::HAL::num_serial];
        
        /** Timestamp of last statistics update in milliseconds */
        uint32_t last_ms;
    };
    
    /**
     * @brief UART statistics for system information queries
     * 
     * @details Used by uart_info() to generate status reports. Updated when
     *          statistics are requested to provide current state information.
     */
    uart_stats sys_uart_stats;
    
#if HAL_LOGGING_ENABLED
    /**
     * @brief UART statistics for dataflash logging
     * 
     * @details Used by uart_log() to record statistics to binary logs.
     *          Updated periodically by scheduler at a rate suitable for logging.
     */
    uart_stats log_uart_stats;
#endif
#endif // HAL_UART_STATS_ENABLED
};
