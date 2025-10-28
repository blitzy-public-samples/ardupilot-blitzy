/**
 * @file Scheduler.h
 * @brief SITL (Software In The Loop) scheduler implementation
 * 
 * @details This file implements the HAL Scheduler interface for SITL simulation.
 *          Unlike real-time RTOS implementations (ChibiOS, Linux), this scheduler
 *          operates in simulation time which may run faster or slower than wall-clock
 *          time depending on simulation performance.
 *          
 *          Key SITL-specific characteristics:
 *          - Uses pthread-based threading model (not bare-metal threads)
 *          - Simulation time can be stopped and advanced in discrete steps
 *          - Timer processes execute based on simulation time, not real-time
 *          - Thread priorities are handled by host OS pthread scheduler
 *          - Stack overflow detection using sentinel values (0xEB pattern)
 *          - Supports time-controlled execution for deterministic testing
 *          
 * @note This implementation is only compiled when CONFIG_HAL_BOARD == HAL_BOARD_SITL
 * @warning Timing behavior differs significantly from hardware platforms - delays
 *          may complete instantly in simulation time, and task scheduling follows
 *          pthread semantics rather than real-time guarantees
 * 
 * Source: libraries/AP_HAL_SITL/Scheduler.h
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "AP_HAL_SITL_Namespace.h"
#include <sys/time.h>
#include <pthread.h>

/** @brief Maximum number of timer processes that can be registered */
#define SITL_SCHEDULER_MAX_TIMER_PROCS 8

/**
 * @class Scheduler
 * @brief SITL scheduler implementation using pthread-based threading
 * 
 * @details Implements the AP_HAL::Scheduler interface for software-in-the-loop
 *          simulation. This scheduler provides time-controlled execution where
 *          simulation time can advance independently of real wall-clock time.
 *          
 *          **Architecture**:
 *          - Main thread: Runs vehicle code and main loop
 *          - Timer thread: Executes registered timer processes
 *          - IO thread: Executes registered IO processes
 *          - Custom threads: Created via thread_create() for specific tasks
 *          
 *          **Time Control**:
 *          The scheduler can stop and advance simulation time in discrete steps,
 *          enabling deterministic testing and replay. The stopped_clock mechanism
 *          allows external control of time progression.
 *          
 *          **Threading Model**:
 *          Uses POSIX pthreads with host OS scheduling. Thread priorities are
 *          mapped to pthread priorities but behavior differs from real-time RTOS.
 *          Preemption and timing guarantees depend on host OS scheduler.
 *          
 *          **Stack Monitoring**:
 *          Thread stacks are filled with sentinel value 0xEB and monitored for
 *          overflow detection. Stack usage can be checked without hardware support.
 *          
 *          **Atomic Operations**:
 *          Provides atomic sections via sitl_begin_atomic()/sitl_end_atomic() to
 *          prevent race conditions in simulation environment.
 *          
 * @warning This is a simulation-only implementation. Timing, threading, and
 *          scheduling behavior differs from hardware platforms. Code tested in
 *          SITL may exhibit different timing on real hardware.
 * 
 * @note Simulation time may run faster or slower than real-time depending on
 *       host CPU performance and simulation complexity
 * 
 * @see AP_HAL::Scheduler for base interface definition
 * @see SITL_State for simulation state management
 */
class HALSITL::Scheduler : public AP_HAL::Scheduler {
public:
    /**
     * @brief Construct SITL scheduler with simulation state
     * 
     * @param[in] sitlState Pointer to SITL_State object managing simulation state
     * 
     * @note Constructor initializes pthread contexts and prepares threading infrastructure
     */
    explicit Scheduler(SITL_State *sitlState);
    
    /**
     * @brief Cast generic scheduler pointer to SITL Scheduler implementation
     * 
     * @param[in] scheduler Generic AP_HAL::Scheduler pointer
     * @return HALSITL::Scheduler* Pointer to SITL-specific scheduler implementation
     * 
     * @note Used for accessing SITL-specific methods from generic scheduler interface
     */
    static Scheduler *from(AP_HAL::Scheduler *scheduler) {
        return static_cast<HALSITL::Scheduler*>(scheduler);
    }

    /* AP_HAL::Scheduler methods */

    /**
     * @brief Initialize the SITL scheduler
     * 
     * @details Sets up pthread infrastructure, initializes timing mechanisms,
     *          and prepares the scheduler for operation. Called once during
     *          HAL initialization before vehicle code starts.
     *          
     * @note Must be called before any other scheduler methods
     */
    void init() override;
    
    /**
     * @brief Delay for specified milliseconds in simulation time
     * 
     * @param[in] ms Delay duration in milliseconds
     * 
     * @details In SITL, this advances simulation time by the specified amount.
     *          Unlike hardware delays, this may complete instantly in wall-clock
     *          time or may take longer depending on simulation speed. The delay
     *          yields to the simulation time advancement mechanism.
     *          
     * @warning Delay duration is in simulation time, not real-time. Actual
     *          wall-clock delay may be shorter or longer depending on simulation
     *          performance and time scaling.
     *          
     * @note This may yield the thread and allow other simulation components to run
     */
    void delay(uint16_t ms) override;
    
    /**
     * @brief Delay for specified microseconds in simulation time
     * 
     * @param[in] us Delay duration in microseconds
     * 
     * @details Advances simulation time by the specified microsecond amount.
     *          For very short delays, may busy-wait or yield depending on
     *          simulation timing requirements.
     *          
     * @warning Microsecond precision in SITL depends on simulation time step
     *          granularity. May not match hardware timing characteristics.
     *          
     * @note Short delays may complete instantly in wall-clock time
     */
    void delay_microseconds(uint16_t us) override;

    /**
     * @brief Register a timer process to run at main loop rate
     * 
     * @param[in] proc Member function pointer to register as timer process
     * 
     * @details Timer processes execute at the main loop rate (typically 400Hz for
     *          multicopter, varies by vehicle). In SITL, timer processes run based
     *          on simulation time advancement. Multiple timer processes can be
     *          registered up to SITL_SCHEDULER_MAX_TIMER_PROCS limit.
     *          
     *          Timer processes are called from a dedicated timer thread context
     *          and should complete quickly to avoid blocking subsequent processes.
     *          
     * @warning Maximum of SITL_SCHEDULER_MAX_TIMER_PROCS (8) timer processes
     *          can be registered. Exceeding this limit will be ignored.
     *          
     * @note Timer processes execute in simulation time, not real-time. Execution
     *       rate follows simulation clock advancement.
     *       
     * @see _run_timer_procs() for execution mechanism
     */
    void register_timer_process(AP_HAL::MemberProc) override;
    
    /**
     * @brief Register an IO process for periodic execution
     * 
     * @param[in] proc Member function pointer to register as IO process
     * 
     * @details IO processes execute less frequently than timer processes and are
     *          used for lower-priority periodic tasks. In SITL, IO processes run
     *          in a separate thread context based on simulation time.
     *          
     *          Typical uses: SD card logging, telemetry transmission, sensor
     *          background processing.
     *          
     * @warning Maximum of SITL_SCHEDULER_MAX_TIMER_PROCS (8) IO processes
     *          can be registered. Exceeding this limit will be ignored.
     *          
     * @note IO processes execute at lower rate than timer processes and should
     *       handle longer-duration operations
     *       
     * @see _run_io_procs() for execution mechanism
     */
    void register_io_process(AP_HAL::MemberProc) override;

    /**
     * @brief Register a failsafe callback with specified period
     * 
     * @param[in] proc Function pointer for failsafe callback
     * @param[in] period_us Execution period in microseconds (simulation time)
     * 
     * @details Registers a high-priority failsafe function that executes
     *          periodically to check for safety-critical conditions. In SITL,
     *          this runs based on simulation time advancement.
     *          
     * @warning Only one failsafe callback can be registered. Subsequent calls
     *          will override the previous registration.
     *          
     * @note Failsafe timing is based on simulation time, which may differ from
     *       real-time hardware behavior
     */
    void register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

    /**
     * @brief Check if currently executing in the main thread
     * 
     * @return true if called from the main thread, false otherwise
     * 
     * @details Compares current pthread context against stored main thread context.
     *          Used to verify thread-safety requirements and enforce single-threaded
     *          access to certain resources.
     *          
     * @note Main thread is the thread that runs vehicle initialization and main loop
     */
    bool in_main_thread() const override;
    
    /**
     * @brief Check if system initialization is complete
     * 
     * @return true if system fully initialized, false otherwise
     * 
     * @details Returns the initialization state set by set_system_initialized().
     *          Used to gate operations that require full system initialization.
     */
    bool is_system_initialized() override { return _initialized; };
    
    /**
     * @brief Mark system initialization as complete
     * 
     * @details Called after all subsystems have completed initialization.
     *          Enables operations that require fully initialized system state.
     *          
     * @note In SITL, this signals that simulation is ready to run vehicle code
     */
    void set_system_initialized() override;

    /**
     * @brief Reboot the simulation
     * 
     * @param[in] hold_in_bootloader If true, stop at bootloader (SITL exits instead)
     * 
     * @details In SITL, this terminates the simulation process. Unlike hardware
     *          platforms, SITL cannot truly reboot - it simply exits and must be
     *          restarted externally.
     *          
     * @warning This will terminate the SITL process. Any unsaved data will be lost.
     * 
     * @note The hold_in_bootloader parameter is ignored in SITL
     */
    void reboot(bool hold_in_bootloader) override;

    /**
     * @brief Check if interrupts/atomic section is active
     * 
     * @return true if currently in an atomic section, false otherwise
     * 
     * @details Returns whether the scheduler is currently in an atomic section
     *          (interrupts disabled equivalent). Uses a nested counter to support
     *          multiple levels of atomic sections.
     *          
     * @note In SITL, this doesn't disable hardware interrupts (no hardware exists),
     *       but provides equivalent mutual exclusion semantics
     *       
     * @see sitl_begin_atomic(), sitl_end_atomic()
     */
    bool interrupts_are_blocked(void) const {
        return _nested_atomic_ctr != 0;
    }

    /**
     * @brief Begin an atomic section (disable interrupts equivalent)
     * 
     * @details Increments nested atomic counter. While in atomic section, timer
     *          and IO processes should not execute. Supports nested atomic sections
     *          through reference counting.
     *          
     * @note Must be paired with sitl_end_atomic(). Unbalanced calls will leave
     *       the scheduler in atomic state permanently.
     *       
     * @warning In SITL, this provides mutual exclusion but doesn't prevent host
     *          OS thread preemption. Use proper locking for thread safety.
     *          
     * @see sitl_end_atomic(), interrupts_are_blocked()
     */
    void sitl_begin_atomic() {
        _nested_atomic_ctr++;
    }
    
    /**
     * @brief End an atomic section (enable interrupts equivalent)
     * 
     * @details Decrements nested atomic counter. When counter reaches zero,
     *          atomic section is exited and normal scheduling resumes.
     *          
     * @note Must be paired with sitl_begin_atomic()
     * 
     * @see sitl_begin_atomic()
     */
    void sitl_end_atomic();

    /**
     * @brief Execute timer and IO process callbacks
     * 
     * @details Static method called to trigger execution of all registered timer
     *          and IO processes. Runs timer processes first, then IO processes.
     *          Called based on simulation time advancement.
     *          
     * @note This is a static method that operates on static process lists, allowing
     *       external time control mechanisms to drive process execution
     *       
     * @see register_timer_process(), register_io_process()
     */
    static void timer_event() {
        _run_timer_procs();
        _run_io_procs();
    }

    /**
     * @brief Get the stopped clock time in microseconds
     * 
     * @return uint64_t Stopped clock time in microseconds (simulation time)
     * 
     * @details Returns the current stopped clock value set by stop_clock().
     *          When simulation time is stopped, this represents the frozen time.
     *          Used for time-controlled deterministic execution and replay.
     *          
     * @note Returns simulation time, not wall-clock time
     * 
     * @see stop_clock()
     */
    uint64_t stopped_clock_usec() const { return _stopped_clock_usec; }

    /**
     * @brief Execute all registered IO processes
     * 
     * @details Static method that iterates through registered IO processes and
     *          executes them. Called less frequently than timer processes.
     *          Tracks execution context via _in_io_proc flag.
     *          
     * @note Static method allows external control of IO process execution timing
     */
    static void _run_io_procs();
    
    /** @brief Flag to signal scheduler should exit (terminate simulation) */
    static bool _should_exit;

    /**
     * @brief Create a new thread for background task execution
     * 
     * @param[in] proc Member function to execute in new thread
     * @param[in] name Thread name for debugging and identification (max 16 chars)
     * @param[in] stack_size Requested stack size in bytes
     * @param[in] base Priority base class (PRIORITY_BOOST, PRIORITY_NORMAL, etc.)
     * @param[in] priority Priority offset within base class (-1 to 255)
     * 
     * @return true if thread created successfully, false on failure
     * 
     * @details Creates a new pthread with specified stack size and priority.
     *          The stack is allocated and filled with sentinel value 0xEB for
     *          overflow detection. Stack monitoring checks for corruption of
     *          sentinel values to detect overflow.
     *          
     *          **Stack Overflow Detection**:
     *          - Stack allocated with requested size
     *          - All bytes initialized to 0xEB sentinel pattern
     *          - stack_min pointer tracks lowest expected address
     *          - check_thread_stacks() periodically verifies sentinel integrity
     *          - Overflow detected if sentinel bytes are overwritten
     *          
     *          **Threading Model**:
     *          Uses POSIX pthreads mapped to host OS scheduler. Priority handling
     *          depends on host OS pthread scheduling policy. Preemption behavior
     *          differs from real-time RTOS like ChibiOS.
     *          
     *          **Stack Size Requirements**:
     *          Ensure adequate stack size for thread execution. SITL can detect
     *          overflow through sentinel checking. Typical sizes: 32KB-128KB.
     *          
     * @warning Priority mapping to pthread priorities is approximate and depends
     *          on host OS scheduling. Real-time guarantees from hardware platforms
     *          do not apply in SITL.
     *          
     * @note Thread name is used for debugging. Use descriptive names to aid in
     *       thread identification during development.
     *       
     * @see thread_create_trampoline(), check_thread_stacks()
     */
    bool thread_create(AP_HAL::MemberProc, const char *name,
                       uint32_t stack_size, priority_base base, int8_t priority) override;

    /**
     * @brief Set flag indicating thread is waiting in semaphore take
     * 
     * @param[in] value true if entering semaphore wait, false if exiting
     * 
     * @details Used by Semaphore implementation to coordinate with scheduler's
     *          time advancement mechanism. Allows scheduler to detect when time
     *          should advance despite thread being blocked on semaphore.
     *          
     * @note SITL-specific mechanism for handling simulation time during blocking operations
     */
    void set_in_semaphore_take_wait(bool value) { _in_semaphore_take_wait = value; }
    
    /**
     * @brief Check if semaphore wait hack is required for time advancement
     * 
     * @return true if time should advance despite being in timer/IO thread, false otherwise
     * 
     * @details SITL-specific helper to determine if simulation time should advance
     *          even when scheduler is pretending to be timer or IO thread context.
     *          Required when threads are blocked on semaphores and time needs to
     *          progress to unblock them.
     *          
     *          This handles a SITL-specific challenge: simulation time is normally
     *          advanced by main thread, but if main thread blocks on a semaphore,
     *          time must still advance for other threads to potentially release
     *          the semaphore.
     *          
     * @note This is a workaround for SITL's time-stepped execution model and does
     *       not exist in hardware HAL implementations
     */
    bool semaphore_wait_hack_required() const;

    /**
     * @brief Get name of currently executing thread
     * 
     * @return const char* Thread name string, or nullptr if thread not found
     * 
     * @details Searches thread list for current pthread and returns the name
     *          registered during thread_create(). Useful for debugging and logging
     *          to identify which thread is executing.
     *          
     * @note Returns nullptr for threads not created through thread_create()
     *       (e.g., main thread, external threads)
     */
    const char *get_current_thread_name(void) const;

private:
    /** @brief Pointer to SITL state object managing simulation */
    SITL_State *_sitlState;
    
    /** @brief Nested atomic section counter (0 = not atomic, >0 = atomic depth) */
    uint8_t _nested_atomic_ctr;
    
    /** @brief Registered failsafe callback function */
    static AP_HAL::Proc _failsafe;

    /**
     * @brief Execute all registered timer processes
     * 
     * @details Iterates through registered timer processes and executes them.
     *          Sets _in_timer_proc flag during execution to track context.
     *          Called at main loop rate based on simulation time.
     */
    static void _run_timer_procs();

    /** @brief Flag indicating timer event was missed (overrun) */
    static volatile bool _timer_event_missed;
    
    /** @brief Array of registered timer process callbacks */
    static AP_HAL::MemberProc _timer_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    
    /** @brief Array of registered IO process callbacks */
    static AP_HAL::MemberProc _io_proc[SITL_SCHEDULER_MAX_TIMER_PROCS];
    
    /** @brief Number of currently registered timer processes */
    static uint8_t _num_timer_procs;
    
    /** @brief Number of currently registered IO processes */
    static uint8_t _num_io_procs;
    
    /** @brief Flag indicating currently executing timer process */
    static bool _in_timer_proc;
    
    /** @brief Flag indicating currently executing IO process */
    static bool _in_io_proc;

    /**
     * @brief Flag indicating thread is waiting for semaphore timeout
     * 
     * @details Set by Semaphore code to indicate it's currently waiting for
     *          a take-timeout to occur. Used by scheduler to determine when
     *          time advancement is needed despite blocking operation.
     */
    static bool _in_semaphore_take_wait;

    /**
     * @brief Stop simulation clock at specified time
     * 
     * @param[in] time_usec Time in microseconds to stop clock at (simulation time)
     * 
     * @details Stops simulation time progression at the specified time. Used for
     *          time-controlled execution and deterministic replay. Clock can be
     *          advanced externally in discrete steps.
     *          
     *          This enables:
     *          - Deterministic testing with controlled time progression
     *          - Log replay with exact timing reproduction
     *          - Debugging with time stopped at specific points
     *          - Performance analysis without real-time pressure
     *          
     * @note This is a SITL-specific feature not available on hardware platforms
     * 
     * @see stopped_clock_usec()
     */
    void stop_clock(uint64_t time_usec) override;

    /**
     * @brief Trampoline function for new pthread execution
     * 
     * @param[in] ctx Context pointer (thread_attr structure)
     * @return void* Thread return value (always nullptr)
     * 
     * @details Static trampoline that receives pthread_create context and
     *          extracts the thread_attr structure to call the actual member
     *          function. Sets up thread environment and handles cleanup.
     *          
     * @note Required because pthread_create requires a static function pointer
     */
    static void *thread_create_trampoline(void *ctx);
    
    /**
     * @brief Check all thread stacks for overflow
     * 
     * @details Iterates through all created threads and checks stack sentinel
     *          values (0xEB pattern). If sentinel bytes have been overwritten,
     *          reports stack overflow error. Called periodically to detect
     *          stack corruption.
     *          
     *          Stack overflow detection mechanism:
     *          1. Stack allocated and filled with 0xEB sentinel
     *          2. stack_min tracks lowest valid address
     *          3. Periodically verify sentinel bytes are unchanged
     *          4. Report overflow if sentinel corrupted
     *          
     * @warning Stack overflow detected by this function indicates serious error
     *          that would cause crash or undefined behavior on hardware
     *          
     * @note SITL-specific feature - hardware platforms may use MPU/MMU or
     *       other mechanisms for stack protection
     */
    static void check_thread_stacks(void);
    
    /** @brief System initialization complete flag */
    bool _initialized;
    
    /** @brief Stopped clock time in microseconds (simulation time) */
    uint64_t _stopped_clock_usec;
    
    /** @brief Last time IO processes ran (microseconds, simulation time) */
    uint64_t _last_io_run;
    
    /** @brief Main thread pthread context for thread identification */
    pthread_t _main_ctx;

    /** @brief Semaphore protecting thread list modifications */
    static HAL_Semaphore _thread_sem;
    
    /**
     * @struct thread_attr
     * @brief Thread attribute structure for stack monitoring and management
     * 
     * @details Contains all information needed to manage a created thread:
     *          - Linked list management (next pointer)
     *          - Thread function pointer
     *          - pthread attributes
     *          - Stack allocation and size
     *          - Stack sentinel tracking for overflow detection
     *          - Thread name for debugging
     *          - pthread handle
     */
    struct thread_attr {
        struct thread_attr *next;      ///< Next thread in linked list
        AP_HAL::MemberProc *f;         ///< Thread function pointer
        pthread_attr_t attr;           ///< pthread attributes
        uint32_t stack_size;           ///< Allocated stack size in bytes
        void *stack;                   ///< Pointer to allocated stack memory
        const uint8_t *stack_min;      ///< Lowest valid stack address (for overflow check)
        const char *name;              ///< Thread name (for debugging)
        pthread_t thread;              ///< pthread handle
    };
    
    /** @brief Linked list of all created threads */
    static struct thread_attr *threads;
    
    /**
     * @brief Stack fill sentinel value for overflow detection
     * 
     * @details All allocated thread stacks are filled with this sentinel value
     *          (0xEB) before thread execution begins. Stack overflow is detected
     *          by checking if these sentinel bytes have been overwritten.
     *          
     * @note Value 0xEB chosen to be recognizable in memory dumps and unlikely
     *       to occur naturally in stack data
     */
    static const uint8_t stackfill = 0xEB;
};
#endif  // CONFIG_HAL_BOARD
