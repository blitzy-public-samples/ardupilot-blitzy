#pragma GCC diagnostic ignored "-Wfloat-equal"
/*
 * Unit tests for Location altitude-frame (AltFrame) conversions.
 * Additive coverage for AAP Directive 1 (AP_Common AltFrame).
 * SUT: libraries/AP_Common/Location.{h,cpp} (READ-ONLY).
 *
 * NOTE: this file is NOT in the wscript's DOUBLE_PRECISION_SOURCES list, so it
 * compiles at default (single) float precision -> use float-epsilon compares.
 */

#include <AP_gtest.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Singletons required by Location's frame-conversion paths (mirrors test_location.cpp).
AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};
AP_Terrain terrain;
GCS_Dummy _gcs;

// Valid, initialised reference coordinates. get_alt_cm() panics on SITL for an
// uninitialised (all-zero) Location, so always use non-zero lat/lng.
static const int32_t TEST_LAT = -35362938;
static const int32_t TEST_LNG = 149165085;

// Happy path: the four AltFrame enumerants are distinct (values 0..3).
TEST(AltFrame, EnumerantsAreDistinct)
{
    EXPECT_EQ(0, (int)static_cast<uint8_t>(Location::AltFrame::ABSOLUTE));
    EXPECT_EQ(1, (int)static_cast<uint8_t>(Location::AltFrame::ABOVE_HOME));
    EXPECT_EQ(2, (int)static_cast<uint8_t>(Location::AltFrame::ABOVE_ORIGIN));
    EXPECT_EQ(3, (int)static_cast<uint8_t>(Location::AltFrame::ABOVE_TERRAIN));
}

// Happy path: set_alt_cm sets the frame; get_alt_frame reports it.
TEST(AltFrame, SetAndGetFrame)
{
    Location loc{TEST_LAT, TEST_LNG, 100, Location::AltFrame::ABSOLUTE};
    EXPECT_EQ(Location::AltFrame::ABSOLUTE, loc.get_alt_frame());

    loc.set_alt_cm(420, Location::AltFrame::ABOVE_HOME);
    EXPECT_EQ(Location::AltFrame::ABOVE_HOME, loc.get_alt_frame());
    EXPECT_EQ(1, loc.relative_alt);

    loc.set_alt_cm(420, Location::AltFrame::ABOVE_ORIGIN);
    EXPECT_EQ(Location::AltFrame::ABOVE_ORIGIN, loc.get_alt_frame());
    EXPECT_EQ(1, loc.origin_alt);

    loc.set_alt_cm(420, Location::AltFrame::ABOVE_TERRAIN);
    EXPECT_EQ(Location::AltFrame::ABOVE_TERRAIN, loc.get_alt_frame());
    EXPECT_EQ(1, loc.terrain_alt);
    EXPECT_EQ(1, loc.relative_alt);  // terrain implies relative_alt
}

// Happy path: ABSOLUTE <-> ABOVE_HOME round-trip (home set) within epsilon.
TEST(AltFrame, RoundTripAbsoluteAboveHome)
{
    const Location home{TEST_LAT, TEST_LNG, 10000, Location::AltFrame::ABSOLUTE}; // 100 m
    EXPECT_TRUE(ahrs.set_home(home));

    Location loc{TEST_LAT, TEST_LNG, 5000, Location::AltFrame::ABSOLUTE};         // 50 m
    const int32_t orig_cm = loc.alt;

    EXPECT_TRUE(loc.change_alt_frame(Location::AltFrame::ABOVE_HOME));
    EXPECT_EQ(Location::AltFrame::ABOVE_HOME, loc.get_alt_frame());

    EXPECT_TRUE(loc.change_alt_frame(Location::AltFrame::ABSOLUTE));
    EXPECT_EQ(Location::AltFrame::ABSOLUTE, loc.get_alt_frame());

    EXPECT_EQ(orig_cm, loc.alt);                 // exact integer round-trip

    float alt_m = 0.0f;
    EXPECT_TRUE(loc.get_alt_m(Location::AltFrame::ABSOLUTE, alt_m));
    EXPECT_FLOAT_EQ(50.0f, alt_m);               // float-epsilon compare
}

// Edge: LOCATION_ALT_MAX_M boundary. Location does NOT clamp (the 83km limit is
// enforced by callers AP_Mission/GCS_Common). Assert faithful storage, NOT a reduction.
TEST(AltFrame, AltMaxBoundaryPassthrough)
{
    EXPECT_EQ(83000, LOCATION_ALT_MAX_M);

    Location loc{TEST_LAT, TEST_LNG, 0, Location::AltFrame::ABSOLUTE};

    loc.set_alt_m(82999, Location::AltFrame::ABSOLUTE);  // just under the limit
    int32_t ret_cm = 0;
    EXPECT_TRUE(loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ret_cm));
    EXPECT_EQ(8299900, ret_cm);                          // 82999 m * 100, faithful

    loc.set_alt_cm(LOCATION_ALT_MAX_M * 100, Location::AltFrame::ABSOLUTE);
    EXPECT_TRUE(loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ret_cm));
    EXPECT_EQ(LOCATION_ALT_MAX_M * 100, ret_cm);         // boundary round-trips exactly
}

// Error: terrain conversion with unavailable terrain DB is handled gracefully.
TEST(AltFrame, TerrainUnavailableGraceful)
{
    AP::terrain()->set_enabled(false);

    Location loc{TEST_LAT, TEST_LNG, 5000, Location::AltFrame::ABSOLUTE};
    EXPECT_FALSE(loc.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN));
    EXPECT_EQ(Location::AltFrame::ABSOLUTE, loc.get_alt_frame()); // unchanged on failure

    Location terr_loc{TEST_LAT, TEST_LNG, 5000, Location::AltFrame::ABOVE_TERRAIN};
    int32_t ret_cm = 0;
    EXPECT_FALSE(terr_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ret_cm));
}

AP_GTEST_PANIC()
AP_GTEST_MAIN()
