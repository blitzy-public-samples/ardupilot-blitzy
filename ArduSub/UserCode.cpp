/**
 * @file UserCode.cpp
 * @brief User customization hooks for ArduSub vehicle-specific code
 * 
 * @details This file provides a set of empty hook functions that allow users to add
 *          custom code to their ArduSub vehicle without modifying core autopilot files.
 *          These hooks are called at various points in the main scheduler loop, providing
 *          integration points at different execution rates.
 *          
 *          To enable these hooks, define the corresponding preprocessor macros in your
 *          build configuration (e.g., in APM_Config.h or via build flags):
 *          - USERHOOK_INIT: Enable userhook_init() for one-time initialization
 *          - USERHOOK_FASTLOOP: Enable userhook_FastLoop() for 100Hz execution
 *          - USERHOOK_50HZLOOP: Enable userhook_50Hz() for 50Hz execution
 *          - USERHOOK_MEDIUMLOOP: Enable userhook_MediumLoop() for 10Hz execution
 *          - USERHOOK_SLOWLOOP: Enable userhook_SlowLoop() for 3.3Hz execution
 *          - USERHOOK_SUPERSLOWLOOP: Enable userhook_SuperSlowLoop() for 1Hz execution
 *          
 *          These hooks integrate with the ArduSub scheduler system and are called from
 *          the main loop at their specified rates. Users should be mindful of execution
 *          time to avoid impacting overall system performance and timing.
 * 
 * @note User code should avoid blocking operations and long computations
 * @note Keep execution time minimal to maintain real-time performance
 * @note Use appropriate hook rate for your task - avoid putting slow operations in fast loops
 * 
 * @warning Poorly written user code can affect flight stability and safety
 * @warning Always test custom code thoroughly in controlled environments
 * @warning Excessive CPU usage in hooks may cause scheduler overruns
 */

#include "Sub.h"

/**
 * @brief User initialization hook called once during vehicle startup
 * 
 * @details This hook is called during the Sub::init_ardupilot() startup sequence,
 *          after core systems are initialized but before the main scheduler loop begins.
 *          Use this for one-time initialization of custom hardware, variables, or
 *          subsystems that your user code requires.
 *          
 *          Called after:
 *          - HAL initialization
 *          - Parameter loading
 *          - Sensor initialization
 *          - Arming checks setup
 *          
 *          Safe operations:
 *          - Initialize custom variables
 *          - Set up additional hardware interfaces
 *          - Configure custom parameters
 *          - Register additional callbacks (if needed)
 * 
 * @note This function is only called once at startup
 * @note Enabled by defining USERHOOK_INIT in build configuration
 * @note Blocking operations here will delay vehicle startup
 * 
 * @see Sub::init_ardupilot()
 */
#ifdef USERHOOK_INIT
void Sub::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

/**
 * @brief User hook called at 100Hz (every 10ms) in the fast loop
 * 
 * @details This hook is called from the main fast loop at 100Hz, making it suitable
 *          for time-critical operations that require high-frequency execution. This is
 *          the fastest user hook available and runs in the same context as core attitude
 *          control loops.
 *          
 *          Typical use cases:
 *          - High-frequency sensor sampling
 *          - Fast custom control loops
 *          - Time-critical signal processing
 *          
 *          Timing considerations:
 *          - Target execution rate: 100Hz (10ms period)
 *          - Keep execution time well under 10ms
 *          - Typical budget: <1ms to avoid impacting main loop
 * 
 * @note Enabled by defining USERHOOK_FASTLOOP in build configuration
 * @note Called from Sub::fast_loop() in the main scheduler
 * @note Avoid blocking operations - they will impact attitude control performance
 * 
 * @warning Excessive CPU usage here directly affects flight stability
 * @warning This is the most timing-critical hook - keep code minimal and fast
 * 
 * @see Sub::fast_loop()
 */
#ifdef USERHOOK_FASTLOOP
void Sub::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

/**
 * @brief User hook called at 50Hz (every 20ms)
 * 
 * @details This hook is called at 50Hz from the scheduler, providing a medium-high
 *          frequency execution rate suitable for control operations that don't require
 *          the full 100Hz fast loop rate. This rate balances CPU usage with responsive
 *          execution for many custom control tasks.
 *          
 *          Typical use cases:
 *          - Custom control loops (position, velocity)
 *          - Sensor fusion and filtering
 *          - Moderate-frequency data logging
 *          - Custom navigation algorithms
 *          
 *          Timing considerations:
 *          - Target execution rate: 50Hz (20ms period)
 *          - Keep execution time under 10ms
 *          - Runs at same priority as other 50Hz scheduler tasks
 * 
 * @note Enabled by defining USERHOOK_50HZLOOP in build configuration
 * @note Called from scheduler at 50Hz
 * @note Good balance between responsiveness and CPU overhead
 * @note Avoid long computations or blocking calls
 * 
 * @see Sub::fifty_hz_loop()
 */
#ifdef USERHOOK_50HZLOOP
void Sub::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

/**
 * @brief User hook called at 10Hz (every 100ms) in the medium loop
 * 
 * @details This hook is called at 10Hz from the scheduler, providing a moderate
 *          execution rate suitable for non-critical operations, telemetry processing,
 *          and tasks that don't require high-frequency updates. This is a good rate
 *          for most user customizations that interact with external systems.
 *          
 *          Typical use cases:
 *          - Custom telemetry processing
 *          - External sensor polling
 *          - State machine updates
 *          - Non-critical calculations
 *          - Communication with external devices
 *          
 *          Timing considerations:
 *          - Target execution rate: 10Hz (100ms period)
 *          - Keep execution time under 50ms
 *          - More tolerance for variable execution time
 * 
 * @note Enabled by defining USERHOOK_MEDIUMLOOP in build configuration
 * @note Called from scheduler at 10Hz
 * @note Suitable for most user customizations
 * @note Lower frequency reduces CPU overhead for non-critical tasks
 * 
 * @see Sub::update_batt_compass()
 */
#ifdef USERHOOK_MEDIUMLOOP
void Sub::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

/**
 * @brief User hook called at 3.3Hz (approximately every 300ms) in the slow loop
 * 
 * @details This hook is called at approximately 3.3Hz from the scheduler, providing
 *          a low-frequency execution rate suitable for infrequent operations, background
 *          tasks, and operations that can tolerate longer update intervals. Use this for
 *          tasks that don't need frequent execution to reduce CPU overhead.
 *          
 *          Typical use cases:
 *          - Background calculations
 *          - Infrequent sensor checks
 *          - System health monitoring
 *          - Periodic data logging
 *          - Low-priority communications
 *          
 *          Timing considerations:
 *          - Target execution rate: ~3.3Hz (~300ms period)
 *          - Keep execution time under 200ms
 *          - Suitable for longer-running operations
 * 
 * @note Enabled by defining USERHOOK_SLOWLOOP in build configuration
 * @note Called from scheduler at approximately 3.3Hz
 * @note Use for tasks that don't require frequent updates
 * @note More computational headroom available at this rate
 * 
 * @see Sub::update_mount()
 */
#ifdef USERHOOK_SLOWLOOP
void Sub::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

/**
 * @brief User hook called at 1Hz (every 1000ms) in the super slow loop
 * 
 * @details This hook is called at 1Hz from the scheduler, providing the lowest-frequency
 *          execution rate for operations that need infrequent execution. This is ideal
 *          for housekeeping tasks, periodic system checks, and operations that should
 *          run once per second or less frequently.
 *          
 *          Typical use cases:
 *          - System health checks
 *          - Periodic status logging
 *          - Infrequent telemetry updates
 *          - Long-running background tasks
 *          - File system operations
 *          - Network communications
 *          
 *          Timing considerations:
 *          - Target execution rate: 1Hz (1000ms period)
 *          - Keep execution time under 500ms
 *          - Most computational headroom of all hooks
 *          - Suitable for operations that can span multiple milliseconds
 * 
 * @note Enabled by defining USERHOOK_SUPERSLOWLOOP in build configuration
 * @note Called from scheduler at 1Hz
 * @note Best choice for infrequent, non-critical operations
 * @note Can accommodate longer-running tasks with minimal system impact
 * 
 * @see Sub::Log_Write_Performance()
 */
#ifdef USERHOOK_SUPERSLOWLOOP
void Sub::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
