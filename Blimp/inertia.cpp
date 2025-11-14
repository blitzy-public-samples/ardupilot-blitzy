#include "Blimp.h"

/**
 * @file inertia.cpp
 * @brief Inertial navigation integration for Blimp vehicle
 * 
 * @details This file contains the read_inertia() function which integrates
 *          inertial navigation data from the EKF and AHRS systems to update
 *          the vehicle's current position and altitude state. The function
 *          handles coordinate frame conversions, altitude reference frame
 *          management, and failsafe behavior for degraded navigation conditions.
 *          
 *          The inertial navigation system provides position and velocity estimates
 *          in the NED (North-East-Down) coordinate frame, which are used to
 *          maintain the vehicle's current_loc (Location) state for navigation
 *          and control purposes.
 */

/**
 * @brief Read and update inertial navigation state from EKF and AHRS
 * 
 * @details This function performs the following operations to maintain current vehicle state:
 *          
 *          1. Updates inertial navigation system with latest IMU data, accounting for
 *             high vibration conditions that may degrade position estimates
 *          2. Retrieves current horizontal position (latitude/longitude) from AHRS
 *          3. Updates current_loc with horizontal position in NED frame
 *          4. Retrieves vertical position (altitude) from inertial navigation if available
 *          5. Converts altitude from EKF origin reference to home reference frame
 *          
 *          Altitude Frame Handling:
 *          - Primary: Altitude relative to EKF origin (from inertial_nav.get_position_z_up_cm())
 *          - Target: Altitude relative to home (ABOVE_HOME frame)
 *          - Fallback: If home not set, treats origin altitude as home altitude
 *          
 *          Coordinate System:
 *          - Position data is in NED (North-East-Down) earth frame
 *          - Altitude positive up (z-up convention in get_position_z_up_cm())
 *          - Horizontal position from AHRS (fused GPS/compass/EKF)
 *          
 *          Failsafe Behavior:
 *          - If vertical position estimate unavailable (no EKF altitude), function
 *            exits early after updating horizontal position only
 *          - Relies on ahrs.has_status(VERT_POS) to validate altitude availability
 * 
 * @note Called from main scheduler loop at regular intervals (typically 50Hz or faster)
 * @note High vibration detection affects altitude estimation method (switches to barometer)
 * @note Updates global current_loc state used by navigation and control systems
 * 
 * @warning Do not call before AHRS initialization complete
 * @warning Altitude will not update if EKF has no vertical position solution
 * 
 * @see AP_InertialNav::update()
 * @see AP_AHRS::get_location()
 * @see Location::change_alt_frame()
 * 
 * Source: Blimp/inertia.cpp:4-27
 */
void Blimp::read_inertia()
{
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    inertial_nav.update(vibration_check.high_vibes);

    // pull position from ahrs
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    if (!ahrs.has_status(AP_AHRS::Status::VERT_POS)) {
        return;
    }

    // current_loc.alt is alt-above-home, converted from inertial nav's alt-above-ekf-origin
    const int32_t alt_above_origin_cm = inertial_nav.get_position_z_up_cm();
    current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_ORIGIN);
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_HOME);
    }
}
