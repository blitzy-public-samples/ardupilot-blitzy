/*
 * GoogleTest unit tests for AP_Mission storage semantics.
 *
 * Hardens the ArduPilot Mission subsystem's storage round-trip and boundary
 * behavior:
 *   - StorageRoundTrip   : a command written via the public add_cmd() path is
 *                          read back byte-for-byte by read_cmd_from_storage(),
 *                          and the auto-home-on-first-add behavior is exercised.
 *   - EmptyMissionBoundary: an empty mission reports zero commands and yields no
 *                          navigation command.
 *   - CapacityBoundary    : the mission fills exactly to num_commands_max() and
 *                          an over-capacity add fails gracefully (no crash).
 *
 * System Under Test (READ-ONLY, in the parent folder): AP_Mission.{h,cpp}.
 * This file only consumes the SUT's existing, unchanged public headers; it does
 * not modify any production source.
 *
 * Test-double policy: no mocking framework. The three constructor callbacks are
 * minimal no-op stubs bound with FUNCTOR_BIND, and AP_GTEST_PANIC() installs a
 * WEAK AP_HAL::panic override so any firmware panic becomes a deterministic test
 * failure. The storage compares are integer/enum only (no exact float
 * comparisons), so no float-equality diagnostic pragma is required.
 */

#include <AP_gtest.h>

#include <AP_Mission/AP_Mission.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Common/Location.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Firmware singletons required at runtime by the mission storage path:
//  - the first add_cmd() writes the home command at index 0, which calls
//    AP::ahrs().get_home(); without an AP_AHRS singleton that path would crash.
//  - the firmware expects a GCS singleton for status-text emission.
//  - AP_Terrain is declared for parity with the proven test_location.cpp
//    template; ABSOLUTE-frame round-trips never exercise the terrain database.
AP_AHRS ahrs{};
AP_Terrain terrain;
GCS_Dummy _gcs;

// Minimal no-op callback holder. This is NOT a mocking framework -- just empty
// stubs satisfying the three functors that AP_Mission's constructor requires:
//   FUNCTOR_TYPEDEF(mission_cmd_fn_t,      bool, const Mission_Command&)
//   FUNCTOR_TYPEDEF(mission_complete_fn_t, void)
class DummyMissionHooks {
public:
    bool start_cmd(const AP_Mission::Mission_Command& cmd) { (void)cmd; return true; }
    bool verify_cmd(const AP_Mission::Mission_Command& cmd) { (void)cmd; return true; }
    void mission_complete(void) {}
};
static DummyMissionHooks mission_hooks;

// EXACTLY ONE AP_Mission for the whole test process. On HAL_BOARD_SITL the
// constructor panics ("Mission must be singleton") if a second instance is ever
// constructed, so this single file-scope instance is shared across every TEST()
// block and must never be constructed inside a test.
//
// FUNCTOR_BIND (not FUNCTOR_BIND_MEMBER) is used because we bind against an
// explicit object pointer at file scope; FUNCTOR_BIND_MEMBER expands using
// `this`/decltype(*this) and is only valid inside a member context.
static AP_Mission mission{
    FUNCTOR_BIND(&mission_hooks, &DummyMissionHooks::start_cmd,        bool, const AP_Mission::Mission_Command &),
    FUNCTOR_BIND(&mission_hooks, &DummyMissionHooks::verify_cmd,       bool, const AP_Mission::Mission_Command &),
    FUNCTOR_BIND(&mission_hooks, &DummyMissionHooks::mission_complete, void)
};

// (Re)initialise and clear the single shared mission before each test so cases
// are independent and order-insensitive. init() reads AP_MISSION_EEPROM_VERSION
// (0x65AE) and reformats storage on mismatch (idempotent); clear() truncates to
// zero commands while disarmed. On the host (linux/sitl) build the sibling
// StorageManager backs hal.storage with RAM/file storage, so writes and reads
// work without any hardware.
static void reset_mission()
{
    mission.init();
    mission.clear();
}

// TEST 1 -- Happy path: a command survives a storage round-trip unchanged, and
// the auto-home-on-first-add behavior is observed.
TEST(AP_Mission, StorageRoundTrip)
{
    reset_mission();
    EXPECT_EQ(mission.num_commands(), 0u);

    // Author a NAV_WAYPOINT command. MAV_CMD_NAV_WAYPOINT (16) is stored_in_location
    // and <= 255, so it round-trips its location fields without tripping the SITL
    // "May not store location for 16-bit commands" panic.
    AP_Mission::Mission_Command cmd{};
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    cmd.content.location = Location{ -353632620, 1491652373, 10000, Location::AltFrame::ABSOLUTE };

    ASSERT_TRUE(mission.add_cmd(cmd));

    // The first add auto-writes the home command at index 0 and then stores the
    // authored command at index 1, so num_commands() jumps 0 -> 2 and the
    // authored command lands at AP_MISSION_FIRST_REAL_COMMAND (1).
    EXPECT_EQ(cmd.index, 1u);
    EXPECT_EQ(mission.num_commands(), 2u);

    // Read the authored command back out of storage and verify every persisted
    // field survives the round-trip. read_cmd_from_storage() sets out.index to
    // the requested index.
    AP_Mission::Mission_Command out{};
    ASSERT_TRUE(mission.read_cmd_from_storage(cmd.index, out));

    EXPECT_EQ(out.id, (uint16_t)MAV_CMD_NAV_WAYPOINT);
    EXPECT_EQ(out.index, cmd.index);
    EXPECT_EQ(out.content.location.lat, cmd.content.location.lat);
    EXPECT_EQ(out.content.location.lng, cmd.content.location.lng);
    EXPECT_EQ(out.content.location.alt, cmd.content.location.alt);

    // Each subsequent add increments the count by exactly one; the second
    // authored command lands at index 2.
    AP_Mission::Mission_Command cmd2{};
    cmd2.id = MAV_CMD_NAV_WAYPOINT;
    cmd2.content.location = Location{ -353631000, 1491650000, 20000, Location::AltFrame::ABSOLUTE };

    ASSERT_TRUE(mission.add_cmd(cmd2));
    EXPECT_EQ(cmd2.index, 2u);
    EXPECT_EQ(mission.num_commands(), 3u);
}

// TEST 2 -- Edge: the empty-mission boundary. With no commands stored,
// get_next_nav_cmd()'s search loop never executes (start_index >= _cmd_total)
// and the call reports that no navigation command exists.
TEST(AP_Mission, EmptyMissionBoundary)
{
    reset_mission();
    EXPECT_EQ(mission.num_commands(), 0u);

    AP_Mission::Mission_Command cmd{};
    EXPECT_FALSE(mission.get_next_nav_cmd(AP_MISSION_FIRST_REAL_COMMAND, cmd));
}

// TEST 3 -- Error/boundary: fill the mission to its exact capacity and confirm
// that one command beyond the cap fails gracefully instead of overflowing.
TEST(AP_Mission, CapacityBoundary)
{
    reset_mission();

    // Capacity is storage-derived ((storage_size - 4) / 15) and may differ by
    // build, so query it rather than hard-coding a value.
    const uint16_t cmd_max = mission.num_commands_max();
    ASSERT_GT(cmd_max, 1u);

    // Fill the mission up to capacity, guarding every add. The first add brings
    // in home@0 + authored@1, after which each add appends exactly one command.
    AP_Mission::Mission_Command cmd{};
    cmd.id = MAV_CMD_NAV_WAYPOINT;
    int32_t alt = 0;
    while (mission.num_commands() < cmd_max) {
        cmd.content.location = Location{ 0, 0, alt++, Location::AltFrame::ABSOLUTE };
        if (!mission.add_cmd(cmd)) {
            break;
        }
    }
    EXPECT_EQ(mission.num_commands(), cmd_max);

    // One command beyond capacity must fail gracefully: add_cmd() delegates to
    // the private write_cmd_to_storage(), which returns false once
    // index >= _commands_max. The stored count must not exceed the cap.
    cmd.content.location = Location{ 0, 0, 999, Location::AltFrame::ABSOLUTE };
    EXPECT_FALSE(mission.add_cmd(cmd));
    EXPECT_LE(mission.num_commands(), cmd_max);

    // Sanity: a previously stored slot still round-trips. The first authored
    // command lives at AP_MISSION_FIRST_REAL_COMMAND (index 1) with the alt we
    // stored on the first fill iteration (0).
    AP_Mission::Mission_Command stored{};
    ASSERT_TRUE(mission.read_cmd_from_storage(AP_MISSION_FIRST_REAL_COMMAND, stored));
    EXPECT_EQ(stored.content.location.alt, 0);
}

AP_GTEST_PANIC()

AP_GTEST_MAIN()
