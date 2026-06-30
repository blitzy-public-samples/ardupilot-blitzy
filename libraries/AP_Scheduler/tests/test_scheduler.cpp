/*
   Unit tests for the ArduPilot main-loop scheduler (AP_Scheduler) and its
   per-loop / per-task performance accounting helper (AP::PerfInfo).

   Part of AAP Directive D1 (PNT subsystem unit-test hardening; blueprint
   sec 0.4.2 "Component: AP_Scheduler"). The cases below cover:
     * AP::PerfInfo::TaskInfo overrun / min / max / tick accounting
     * AP::PerfInfo loop-time max + long-running (overtime) detection
     * AP::PerfInfo dynamically-allocated per-task statistics
     * AP_Scheduler loop-rate -> loop-period math (1000000 / rate)
     * AP_Scheduler [50, 2000] Hz loop-rate clamp (low and high endpoints)
     * AP_Scheduler fast-task dispatch through run()

   The system-under-test is READ-ONLY (AAP sec 0.10): this file never modifies
   AP_Scheduler.{h,cpp} or PerfInfo.{h,cpp}; it only reads their public (and,
   for the scheduler-owner singleton, protected-via-`using`) interfaces.

   Test-double policy (AAP sec 0.10): the only stub is TestVehicle : AP_Vehicle.
   It is required because AP_Scheduler is owned by the AP_Vehicle singleton and
   AP_Vehicle's embedded `scheduler` member IS the one AP_Scheduler singleton.
   No generalized mocking framework is introduced.
 */

// The AP_Param GOBJECT()/GSCALAR() macros expand to references qualified by
// this vehicle-name macro (e.g. &testvehicle.scheduler), so it MUST be defined
// before any header that pulls in AP_Param.h.
#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_Scheduler/AP_Scheduler.h>   // AP_Scheduler + AP::PerfInfo (via PerfInfo.h)
#include <AP_Vehicle/AP_Vehicle.h>       // AP_Vehicle base for TestVehicle + AP::vehicle()
#include <AP_Param/AP_Param.h>           // AP_Param::set_by_name, GOBJECT, AP_VAREND

// Mandatory global HAL singleton reference declared by every ArduPilot unit
// test; numerous library translation units `extern` this symbol.
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Trap any firmware AP_HAL::panic() (for example the scheduler's "Too many
// schedulers" guard or AP_Vehicle's "Too many Vehicles" guard) and turn it
// into a deterministic abort()/test failure rather than an infinite loop.
AP_GTEST_PANIC()

// ---------------------------------------------------------------------------
// Phase 2: minimal parameter scaffolding + the single sanctioned stub vehicle
// ---------------------------------------------------------------------------

// AP_Param's GOBJECT() macro references Parameters::k_param_<name> as the
// index enum for each registered group; only the scheduler entry is needed.
class Parameters {
public:
    enum {
        k_param_scheduler,
    };
};

// A tiny task owner whose member function records that it executed. It is
// defined at file scope (NOT inside a TEST) because FAST_TASK_CLASS /
// FUNCTOR_BIND take the member-function pointer as a non-type template
// argument, which requires a non-local class type.
class TaskOwner {
public:
    void my_task() { ran = true; }
    bool ran = false;
};

// The ONLY test double (AAP sec 0.10). AP_Scheduler cannot be constructed
// stand-alone (its constructor panics "Too many schedulers" if a second
// instance is created), and AP_Scheduler::init() consults AP::vehicle() for the
// common scheduler task table. AP_Vehicle embeds `AP_Scheduler scheduler;`
// which therefore becomes THE scheduler singleton. Exactly one TestVehicle is
// created at file scope and the scheduler is reached through AP::scheduler().
//
// The override set mirrors the proven, compiling stub in
// libraries/AP_Param/tests/test_find_by_name.cpp (it satisfies every
// AP_Vehicle pure-virtual in the current tree). The single addition is
// `using AP_Vehicle::scheduler;`, which re-exposes the protected `scheduler`
// member at public scope so the file-scope GOBJECT(scheduler, ...) entry
// (which expands to &testvehicle.scheduler) compiles.
class TestVehicle : public AP_Vehicle {
public:
    using AP_Vehicle::scheduler;   // expose protected member for GOBJECT registration

    void load_parameters(void) override {};
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override {
        tasks = nullptr;
        task_count = 0;
        log_bit = 0;
    };
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; }
    uint8_t get_mode() const override { return 0; }

    AP_Int32 unused_log_bitmask;
    struct LogStructure log_structure[256] = {};

protected:
    const AP_Int32 &get_log_bitmask() override { return unused_log_bitmask; }
    const struct LogStructure *get_log_structures() const override { return log_structure; }
    uint8_t get_num_log_structures() const override { return uint8_t(ARRAY_SIZE(log_structure)); }
    void init_ardupilot() override {};

public:
    static const AP_Param::Info var_info[];
    AP_Param param_loader{var_info};
};

// File-scope instance: establishes both the AP::vehicle() singleton and (via
// its embedded member) the AP_Scheduler singleton for the whole binary.
static TestVehicle testvehicle;

// Register ONLY the scheduler param group so "SCHED_LOOP_RATE" resolves by
// name. AP_Scheduler::var_info declares AP_GROUPINFO("LOOP_RATE", 1, ...,
// _loop_rate_hz, ...), so nesting it under the "SCHED_" prefix yields the full
// parameter name "SCHED_LOOP_RATE".
const AP_Param::Info TestVehicle::var_info[] = {
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),
    AP_VAREND
};

// ---------------------------------------------------------------------------
// Phase 3: AP::PerfInfo unit tests (no scheduler/vehicle dependency)
// ---------------------------------------------------------------------------

// TaskInfo::update() accounting: the first sample seeds min==max==value and
// marks the overrun; a later smaller, non-overrunning sample lowers min, leaves
// max and overrun_count unchanged, and advances tick_count / elapsed_time.
TEST(PerfInfo, TaskInfoOverrunAccounting)
{
    AP::PerfInfo::TaskInfo ti{};

    ti.update(500, /*overrun=*/true);
    EXPECT_EQ(ti.max_time_us, 500u);
    EXPECT_EQ(ti.min_time_us, 500u);
    EXPECT_EQ(ti.elapsed_time_us, 500u);
    EXPECT_EQ(ti.tick_count, 1u);
    EXPECT_EQ(ti.overrun_count, 1u);

    ti.update(100, /*overrun=*/false);
    EXPECT_EQ(ti.max_time_us, 500u);       // max unchanged (500 > 100)
    EXPECT_EQ(ti.min_time_us, 100u);       // min lowered to 100
    EXPECT_EQ(ti.elapsed_time_us, 600u);   // 500 + 100
    EXPECT_EQ(ti.tick_count, 2u);
    EXPECT_EQ(ti.overrun_count, 1u);       // no new overrun recorded
}

// set_loop_rate(50) => overtime_threshold = 1000000/50*1.2 = 24000 us. A
// 100000 us loop exceeds that, so it is recorded as the maximum loop time and
// counted as a long-running (overtime) loop.
TEST(PerfInfo, CheckLoopTimeMaxAndLongRunning)
{
    // PerfInfo's default constructor is empty and does NOT zero its scalar
    // accumulators (max_time, long_running, ...). In firmware PerfInfo is only
    // ever a member of the AP_Scheduler singleton (static storage => zero
    // initialised), so it always starts at zero. Mirror that here with static
    // storage; a stack instance would carry indeterminate accumulators and the
    // "100000 > max_time" comparison below would be unreliable.
    static AP::PerfInfo perf;
    perf.set_loop_rate(50);
    perf.check_loop_time(100000);

    EXPECT_EQ(perf.get_max_time(), 100000u);
    EXPECT_GE(perf.get_num_long_running(), 1u);
}

// Dynamically-allocated per-task statistics: a valid index updates the slot,
// while an out-of-range read safely yields nullptr.
TEST(PerfInfo, AllocatedTaskInfoUpdate)
{
    // Static storage so the PerfInfo bookkeeping members start zero-initialised
    // (the empty constructor does not zero them); see CheckLoopTime above.
    static AP::PerfInfo perf;
    perf.allocate_task_info(2);

    // Only valid indices may be passed to update_task_info(); an out-of-range
    // index would raise an INTERNAL_ERROR inside the SUT.
    perf.update_task_info(0, 500, /*overrun=*/true);

    const AP::PerfInfo::TaskInfo *tip = perf.get_task_info(0);
    ASSERT_NE(tip, nullptr);
    EXPECT_EQ(tip->max_time_us, 500u);
    EXPECT_EQ(tip->overrun_count, 1u);

    // Out-of-range read is a pure, side-effect-free guard returning nullptr.
    EXPECT_EQ(perf.get_task_info(99), nullptr);

    perf.free_task_info();
}

// ---------------------------------------------------------------------------
// Phase 4: AP_Scheduler loop-rate clamp + loop-period math
// ---------------------------------------------------------------------------
//
// The private _loop_rate_hz (the LOOP_RATE parameter) can only be driven
// through the AP_Param mechanism. AP_Scheduler::init() then clamps it to
// [50, 2000] Hz and (re)computes _loop_period_us = 1000000 / _loop_rate_hz and
// _active_loop_rate_hz on every call. Repeated init() calls are safe because
// the SITL HAL_Semaphore taken by init() is recursive. Passing
// (tasks=nullptr, num_tasks=0) is safe: the vehicle's common tasks are only
// registered, never executed, during init().

TEST(Scheduler, LoopPeriodMath)
{
    const uint16_t rates[] = { 50, 100, 400, 2000 };
    for (const uint16_t rate : rates) {
        EXPECT_TRUE(AP_Param::set_by_name("SCHED_LOOP_RATE", rate));
        AP::scheduler().init(nullptr, 0, 0);
        EXPECT_EQ(AP::scheduler().get_loop_rate_hz(), rate);
        EXPECT_EQ(AP::scheduler().get_loop_period_us(), uint32_t(1000000UL / rate));
    }
}

TEST(Scheduler, LoopRateClampLow)
{
    // Any value below 50 Hz clamps up to the 50 Hz floor.
    EXPECT_TRUE(AP_Param::set_by_name("SCHED_LOOP_RATE", 10));
    AP::scheduler().init(nullptr, 0, 0);
    EXPECT_EQ(AP::scheduler().get_loop_rate_hz(), uint16_t(50));
    EXPECT_EQ(AP::scheduler().get_loop_period_us(), uint32_t(1000000UL / 50));
}

TEST(Scheduler, LoopRateClampHigh)
{
    // Any value above 2000 Hz clamps down to the 2000 Hz ceiling.
    EXPECT_TRUE(AP_Param::set_by_name("SCHED_LOOP_RATE", 5000));
    AP::scheduler().init(nullptr, 0, 0);
    EXPECT_EQ(AP::scheduler().get_loop_rate_hz(), uint16_t(2000));
    EXPECT_EQ(AP::scheduler().get_loop_period_us(), uint32_t(1000000UL / 2000));
}

// ---------------------------------------------------------------------------
// Phase 5: fast-task dispatch through run()
// ---------------------------------------------------------------------------
//
// A task whose priority is <= MAX_FAST_TASK_PRIORITIES (a "fast" task) is
// dispatched unconditionally by run(), independent of the available time
// budget. We register a single fast vehicle task and call run(0): our task
// runs while every common (non-fast) task is skipped because its
// max_time_micros exceeds the zero time budget. (loop() is never called; it
// would block waiting for an INS sample.)
TEST(Scheduler, FastTaskDispatch)
{
    static TaskOwner owner;
    owner.ran = false;

    // static storage so the task table outlives any pointer the scheduler
    // singleton retains after init().
    static const AP_Scheduler::Task tasks[] = {
        FAST_TASK_CLASS(TaskOwner, &owner, my_task),
    };

    EXPECT_TRUE(AP_Param::set_by_name("SCHED_LOOP_RATE", 400));   // any valid rate
    AP::scheduler().init(tasks, ARRAY_SIZE(tasks), 0);
    AP::scheduler().run(0);   // fast task runs; common tasks skip (max_time > 0)

    EXPECT_TRUE(owner.ran);
}

// ---------------------------------------------------------------------------
// Phase 6: standalone test entry point
// ---------------------------------------------------------------------------
AP_GTEST_MAIN()
