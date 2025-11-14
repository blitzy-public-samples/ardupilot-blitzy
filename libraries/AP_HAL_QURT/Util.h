/**
 * @file Util.h
 * @brief QURT platform utility functions for Qualcomm Hexagon DSP
 * 
 * @details This file implements the AP_HAL::Util interface for the QURT
 *          (Qualcomm Real-Time) platform, providing platform-specific
 *          utility functions and system information for Hexagon DSP.
 *          
 *          The QURT HAL runs on Qualcomm's Hexagon DSP (QDSP6) which is
 *          an embedded real-time processor, not a full Unix-like OS.
 *          Many standard utility functions are stubbed or limited due to
 *          platform constraints.
 * 
 * @note Platform limitations: QURT/Hexagon DSP environment has significant
 *       differences from typical Linux/ChibiOS platforms:
 *       - No filesystem access (no SD card support on DSP)
 *       - No console I/O (HAL_OS_POSIX_IO not available)
 *       - Very limited memory (~32MB total, 10-20MB available heap)
 *       - Limited system information APIs
 * 
 * @warning Many AP_HAL::Util methods return default/stub values on QURT
 *          due to these platform limitations.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_QURT_Namespace.h"

/**
 * @class QURT::Util
 * @brief Platform utility functions for Hexagon DSP
 * 
 * @details Implements the AP_HAL::Util interface to provide QURT-specific
 *          system utilities including safety state management, reboot control,
 *          memory queries, and system information.
 *          
 *          This class provides essential platform services:
 *          - Safety state management (software safety interlock)
 *          - System information queries (memory, system ID)
 *          - Hardware RTC access (stubbed on QURT)
 *          
 *          Due to the embedded nature of the Hexagon DSP platform, many
 *          utility functions have minimal implementations compared to
 *          full-featured platforms like Linux or ChibiOS.
 * 
 * @note Minimal implementation: Many Util methods are stubbed on QURT due
 *       to DSP platform limitations. The Hexagon DSP is a specialized
 *       coprocessor, not a general-purpose computing platform.
 * 
 * @warning DSP memory is very limited (typically <1MB free heap for
 *          ArduPilot operations after system reserves). Code should
 *          minimize dynamic allocation.
 * 
 * @see AP_HAL::Util for the full interface definition
 * @see Scheduler.h for reboot() implementation
 */
class QURT::Util : public AP_HAL::Util
{
public:
    /**
     * @brief Set hardware real-time clock in UTC microseconds
     * 
     * @details Attempts to set the hardware RTC to the specified time.
     *          On QURT platform, this is a stub implementation as the
     *          Hexagon DSP typically does not have direct RTC hardware
     *          control. Time synchronization is handled at the system level
     *          by the main application processor.
     * 
     * @param[in] time_utc_usec Time in microseconds since Unix epoch (UTC)
     * 
     * @note Stub implementation: This function does nothing on QURT.
     *       Time is typically synchronized from the main processor via IPC.
     * 
     * @warning Do not rely on this function for time synchronization on QURT.
     *          Use system-level time synchronization mechanisms instead.
     */
    void set_hw_rtc(uint64_t time_utc_usec) override {}

    /**
     * @brief Get hardware real-time clock in UTC microseconds
     * 
     * @details Retrieves the current hardware RTC time. On QURT platform,
     *          this returns 0 as a stub implementation. The Hexagon DSP
     *          does not have independent RTC hardware accessible to
     *          ArduPilot code.
     *          
     *          Actual time tracking on QURT is done through the system
     *          monotonic clock (AP_HAL::micros64()) which provides relative
     *          timing, and absolute time is synchronized from the main
     *          application processor.
     * 
     * @return uint64_t Time in microseconds since Unix epoch (UTC), or 0
     *         on QURT indicating RTC not available
     * 
     * @note Stub implementation: Always returns 0 on QURT platform.
     *       Use AP_HAL::micros64() for monotonic relative timing instead.
     * 
     * @warning Do not use this for absolute time on QURT. Return value of
     *          0 indicates RTC is not available on this platform.
     */
    uint64_t get_hw_rtc() const override
    {
        return 0;
    }

    /**
     * @brief Get available free memory in bytes
     * 
     * @details Returns the amount of free heap memory available on the
     *          Hexagon DSP. This queries the fc_heap (flight controller heap)
     *          which is a limited memory region dedicated to ArduPilot
     *          operations.
     *          
     *          The Hexagon DSP typically has approximately 32MB of total RAM,
     *          with roughly 10-20MB available for heap allocation after
     *          accounting for code, static data, and stack space. Free memory
     *          can vary significantly based on loaded features and runtime
     *          allocation patterns.
     * 
     * @return uint32_t Number of free bytes in DSP heap
     * 
     * @note DSP memory is very limited compared to typical autopilot platforms.
     *       Free heap may be less than 1MB during normal operations depending
     *       on enabled features and sensor drivers.
     * 
     * @warning DSP has ~32MB total RAM with ~10-20MB available for heap after
     *          code/stack. ArduPilot code should minimize dynamic allocation
     *          on memory-constrained platforms. Frequent allocation/deallocation
     *          can lead to heap fragmentation.
     * 
     * @note Use sparingly: This function is primarily for diagnostics and
     *       pre-flight checks. Do not call in high-frequency loops as memory
     *       queries may have non-trivial overhead.
     */
    uint32_t available_memory(void) override;

    /**
     * @brief Get hardware safety switch state
     * 
     * @details Returns the current state of the hardware safety switch,
     *          if present on the platform. The safety switch is a physical
     *          button or switch that provides a hardware interlock preventing
     *          motor output until explicitly enabled by the operator.
     *          
     *          On QURT platform, safety switch support depends on the specific
     *          board configuration. Some boards may have a safety switch
     *          connected via GPIO, while others may report a constant state
     *          (always armed or always safe) if no physical switch is present.
     *          
     *          Safety states:
     *          - SAFETY_NONE: No safety switch available (always allows arming)
     *          - SAFETY_DISARMED: Safety switch active, motors cannot spin
     *          - SAFETY_ARMED: Safety switch disarmed, motors can spin if armed
     * 
     * @return enum safety_state Current safety switch state
     * 
     * @note Safety state controls whether motors can spin (software safety
     *       interlock separate from main arming logic). Both safety switch
     *       and arming must be enabled for motors to spin.
     * 
     * @note May return constant value if no physical safety switch is present
     *       on the board. Check board-specific configuration (hwdef) to
     *       determine if safety switch is available.
     * 
     * @warning Safety-critical function: This is part of the motor safety
     *          interlock system. If safety switch is missing or reports
     *          SAFETY_NONE, vehicle can arm without physical safety switch
     *          check (software arming checks still apply).
     * 
     * @warning If safety_switch_state() always returns SAFETY_NONE, ensure
     *          operators are aware that there is no hardware safety interlock
     *          and rely entirely on software arming checks and propeller
     *          safety procedures.
     * 
     * @note Thread-safe: This function can be called from any thread.
     * 
     * @see AP_Arming for main arming logic and safety checks
     * @see AP_HAL::Util::safety_state for enum definition
     */
    enum safety_state safety_switch_state(void) override;
};
