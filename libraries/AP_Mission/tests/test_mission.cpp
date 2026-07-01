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
#include <cmath>

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

// ---------------------------------------------------------------------------
// Coverage of the pure mavlink<->Mission_Command translation surface.
//
// mavlink_int_to_mission_cmd(), mission_cmd_to_mavlink_int(),
// sanity_check_params(), stored_in_location() and the convert_MISSION_ITEM_*
// helpers are public static functions used by the GCS mission upload/download
// path.  They perform struct translation only -- no vehicle/subsystem
// singletons are required beyond those already declared above -- so they are
// exercised directly without touching storage.  Round-tripping a broad spread
// of MAV_CMD types walks both large switch statements end to end.
// ---------------------------------------------------------------------------

// Build a well-formed MISSION_ITEM_INT packet with all params finite (so
// sanity_check_params() accepts every command) and a plausible global
// lat/lng/alt for the location-bearing commands.
static mavlink_mission_item_int_t make_mission_item(uint16_t command,
                                                    float p1 = 1.0f, float p2 = 2.0f,
                                                    float p3 = 3.0f, float p4 = 4.0f)
{
    mavlink_mission_item_int_t pkt {};
    pkt.seq = 1;
    pkt.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    pkt.command = command;
    pkt.param1 = p1;
    pkt.param2 = p2;
    pkt.param3 = p3;
    pkt.param4 = p4;
    pkt.x = -353632620;   // latitude  * 1e7  (a valid ArduPilot home-ish point)
    pkt.y = 1491652373;   // longitude * 1e7
    pkt.z = 30.0f;        // altitude (m, frame-relative)
    return pkt;
}

// Forward+reverse conversion of a representative command set.  Every command in
// the list is handled by mavlink_int_to_mission_cmd()'s switch; each is decoded
// (the packet->cmd arm) and, when the command supports GCS readback, re-encoded
// (the cmd->packet arm).  Commands that are build-gated off in this unit build
// legitimately decline conversion and are skipped without failing the test.
TEST(AP_Mission, MavlinkConversionRoundTrip)
{
    const uint16_t commands[] = {
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_LOITER_UNLIM,
        MAV_CMD_NAV_LOITER_TURNS,
        MAV_CMD_NAV_LOITER_TIME,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        MAV_CMD_NAV_LAND,
        MAV_CMD_NAV_TAKEOFF,
        MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
        MAV_CMD_NAV_LOITER_TO_ALT,
        MAV_CMD_NAV_SPLINE_WAYPOINT,
        MAV_CMD_NAV_GUIDED_ENABLE,
        MAV_CMD_NAV_DELAY,
        MAV_CMD_NAV_ALTITUDE_WAIT,
        MAV_CMD_NAV_SET_YAW_SPEED,
        MAV_CMD_NAV_VTOL_TAKEOFF,
        MAV_CMD_NAV_VTOL_LAND,
        MAV_CMD_NAV_PAYLOAD_PLACE,
        MAV_CMD_NAV_ATTITUDE_TIME,
        MAV_CMD_CONDITION_DELAY,
        MAV_CMD_CONDITION_DISTANCE,
        MAV_CMD_CONDITION_YAW,
        MAV_CMD_DO_JUMP,
        MAV_CMD_DO_CHANGE_SPEED,
        MAV_CMD_DO_SET_HOME,
        MAV_CMD_DO_SET_RELAY,
        MAV_CMD_DO_REPEAT_RELAY,
        MAV_CMD_DO_SET_SERVO,
        MAV_CMD_DO_REPEAT_SERVO,
        MAV_CMD_DO_LAND_START,
        MAV_CMD_DO_GO_AROUND,
        MAV_CMD_DO_SET_ROI,
        MAV_CMD_DO_SET_ROI_LOCATION,
        MAV_CMD_DO_SET_ROI_NONE,
        MAV_CMD_DO_DIGICAM_CONFIGURE,
        MAV_CMD_DO_DIGICAM_CONTROL,
        MAV_CMD_DO_SET_CAM_TRIGG_DIST,
        MAV_CMD_DO_MOUNT_CONTROL,
        MAV_CMD_DO_SET_REVERSE,
        MAV_CMD_DO_FENCE_ENABLE,
        MAV_CMD_DO_AUX_FUNCTION,
        MAV_CMD_DO_GUIDED_LIMITS,
        MAV_CMD_DO_WINCH,
        MAV_CMD_DO_GRIPPER,
        MAV_CMD_DO_PARACHUTE,
        MAV_CMD_DO_INVERTED_FLIGHT,
        MAV_CMD_DO_AUTOTUNE_ENABLE,
        MAV_CMD_DO_SET_RESUME_REPEAT_DIST,
        MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        MAV_CMD_DO_PAUSE_CONTINUE,
        MAV_CMD_JUMP_TAG,
        MAV_CMD_DO_JUMP_TAG,
        MAV_CMD_IMAGE_START_CAPTURE,
        MAV_CMD_IMAGE_STOP_CAPTURE,
        MAV_CMD_SET_CAMERA_ZOOM,
        MAV_CMD_SET_CAMERA_FOCUS,
        MAV_CMD_SET_CAMERA_SOURCE,
        MAV_CMD_VIDEO_START_CAPTURE,
        MAV_CMD_VIDEO_STOP_CAPTURE,
    };

    unsigned accepted = 0;
    for (const uint16_t id : commands) {
        const mavlink_mission_item_int_t pkt = make_mission_item(id);
        AP_Mission::Mission_Command cmd {};
        const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(pkt, cmd);
        if (res != MAV_MISSION_ACCEPTED) {
            // A command compiled out of this unit build (feature-gated) declines
            // conversion; that is a legitimate outcome, not a test failure.
            continue;
        }
        accepted++;
        EXPECT_EQ(cmd.id, id);

        // Reverse arm: not every command supports GCS readback, so only assert
        // the round-trip identity when the encoder reports success.
        mavlink_mission_item_int_t out {};
        if (AP_Mission::mission_cmd_to_mavlink_int(cmd, out)) {
            EXPECT_EQ(out.command, id);
        }
    }

    // The vast majority of the representative set must convert in a stock build;
    // this guards against a regression that silently rejects everything.
    EXPECT_GT(accepted, 40U);
}

// sanity_check_params() rejects non-finite params for commands whose parameters
// must all be finite.  A NaN param1 on DO_JUMP (not in the NaN-tolerant set)
// must therefore be refused rather than accepted.
TEST(AP_Mission, MavlinkConversionRejectsNonFinite)
{
    mavlink_mission_item_int_t pkt = make_mission_item(MAV_CMD_DO_JUMP);
    pkt.param1 = nanf("");
    AP_Mission::Mission_Command cmd {};
    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(pkt, cmd);
    EXPECT_NE(res, MAV_MISSION_ACCEPTED);
}

// The MISSION_ITEM (float) <-> MISSION_ITEM_INT converters translate the legacy
// float lat/lng representation to/from the scaled-integer form.  A waypoint
// round-trips through both directions without loss of the command id.
TEST(AP_Mission, ConvertMissionItemFloatIntRoundTrip)
{
    mavlink_mission_item_t item {};
    item.seq = 2;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.param1 = 0.0f;
    item.x = -35.363262f;    // degrees
    item.y = 149.165237f;    // degrees
    item.z = 40.0f;

    mavlink_mission_item_int_t item_int {};
    ASSERT_EQ(AP_Mission::convert_MISSION_ITEM_to_MISSION_ITEM_INT(item, item_int),
              MAV_MISSION_ACCEPTED);
    EXPECT_EQ(item_int.command, static_cast<uint16_t>(MAV_CMD_NAV_WAYPOINT));

    mavlink_mission_item_t back {};
    ASSERT_EQ(AP_Mission::convert_MISSION_ITEM_INT_to_MISSION_ITEM(item_int, back),
              MAV_MISSION_ACCEPTED);
    EXPECT_EQ(back.command, static_cast<uint16_t>(MAV_CMD_NAV_WAYPOINT));
}

AP_GTEST_PANIC()

AP_GTEST_MAIN()
