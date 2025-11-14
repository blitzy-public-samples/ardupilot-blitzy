/**
 * @file tracking.cpp
 * @brief Core antenna tracking algorithms and MAVLink position update handlers
 * 
 * @details Implements main 50Hz tracking loop, vehicle position estimation with velocity projection,
 *          bearing/distance calculations, and altitude source selection. This file coordinates
 *          all tracking functionality including position updates, servo control, and telemetry handling.
 */

#include "Tracker.h"

/**
 * @brief Updates estimated vehicle position using velocity projection
 * 
 * @details Called at 50Hz to maintain vehicle position estimate between MAVLink updates.
 *          This function compensates for telemetry latency and lost packets by projecting
 *          the vehicle location forward using the last known velocity vector.
 *          
 *          Algorithm:
 *          - Calculates time delta since last MAVLink GLOBAL_POSITION_INT message
 *          - If delta < TRACKING_TIMEOUT_SEC (5.0s), projects position using velocity
 *          - Projects horizontal position: offset(vx*dt, vy*dt) in NED frame
 *          - Projects altitude: alt += vz*dt (vel.z in m/s, down positive in NED)
 *          - Declares location invalid if >5 seconds since last update
 *          - Invalid location triggers AUTO mode to fall back to scan mode
 * 
 * @note Velocity projection compensates for MAVLink latency (typical 50-200ms)
 * @note Location validity checked by AUTO mode to switch to scan if vehicle lost
 * @warning Assumes constant velocity - accuracy degrades for maneuvering vehicles
 * 
 * @see tracking_update_position() for MAVLink position message handler
 * @see update_tracking() for main tracking loop that calls this function
 */
void Tracker::update_vehicle_pos_estimate()
{
    // Calculate time delta since last MAVLink GLOBAL_POSITION_INT message
    float dt = (AP_HAL::micros() - vehicle.last_update_us) * 1.0e-6f;

    // TRACKING_TIMEOUT_SEC = 5.0 defines staleness threshold for position validity
    if (dt < TRACKING_TIMEOUT_SEC) {
        // project the vehicle position to take account of lost radio packets
        vehicle.location_estimate = vehicle.location;
        // Project position using velocity: offset(vx*dt, vy*dt) in NED frame (North, East in m/s)
        float north_offset = vehicle.vel.x * dt;
        float east_offset = vehicle.vel.y * dt;
        vehicle.location_estimate.offset(north_offset, east_offset);
        // Altitude projection: altitude in cm, vel.z in m/s (down positive in NED frame)
    	vehicle.location_estimate.alt += vehicle.vel.z * 100.0f * dt;
        // Set valid flag - allows AUTO mode to track the vehicle
        vehicle.location_valid = true;
    } else {
        // Set invalid flag to trigger scan mode fallback when vehicle is lost >5 seconds
        vehicle.location_valid = false;
    }
}

/**
 * @brief Updates tracker's own position from AHRS/GPS
 * 
 * @details Called at 50Hz to maintain tracker location (required for moving tracker installations).
 *          This function queries AHRS for the current GPS-derived position and updates the
 *          reference location used for bearing/distance calculations.
 *          
 *          Behavior:
 *          - Queries AHRS for current GPS-derived position
 *          - Sets stationary flag false if position valid (supports mobile tracker platforms)
 *          - Updates current_loc used as reference for bearing/distance calculations
 *          - current_loc is set to home on startup, updated continuously if GPS available
 * 
 * @note Supports stationary and mobile tracker installations
 * @note current_loc set to home on startup, updated continuously if GPS available
 * 
 * @see update_bearing_and_distance() for usage of current_loc
 * @see init_ardupilot() for home position initialization
 */
void Tracker::update_tracker_position()
{
    Location temp_loc;

    // TODO: Handle loss of GPS lock during mission when tracker is moving
    if (ahrs.get_location(temp_loc)) {
        stationary = false;
        current_loc = temp_loc;
    }
}

/**
 * @brief Calculates bearing, distance, and pitch to tracked vehicle
 * 
 * @details Called at 50Hz to update navigation status with target angles.
 *          Computes great-circle bearing, horizontal distance, altitude difference,
 *          and elevation pitch angle to the tracked vehicle.
 *          
 *          Calculations:
 *          - Bearing: Great-circle azimuth from tracker to vehicle (degrees, 0-360)
 *          - Distance: Horizontal distance in meters (Haversine formula)
 *          - Altitude difference: Via GPS or barometer based on ALT_SOURCE parameter
 *          - Pitch: Elevation angle calculated using atan2(alt_diff, distance)
 *          
 *          ALT_SOURCE parameter selection:
 *          - ALT_SOURCE_GPS: Uses absolute altitudes (tracker GPS + vehicle GPS)
 *          - ALT_SOURCE_GPS_VEH_ONLY: Uses vehicle's relative_alt (relative to vehicle home)
 *          - ALT_SOURCE_BARO: Uses barometric altitude from SCALED_PRESSURE messages
 * 
 * @note Bearing in degrees (0-360), pitch in degrees (+up, -down)
 * @note ALT_SOURCE=GPS uses absolute altitudes; GPS_VEH_ONLY uses vehicle's relative_alt
 * @note BARO altitude source requires SCALED_PRESSURE messages from vehicle
 * 
 * @see tracking_update_pressure() for baro altitude source
 * @see update_tracking() for main tracking loop that calls this function
 */
void Tracker::update_bearing_and_distance()
{
    // exit immediately if we do not have a valid vehicle position
    if (!vehicle.location_valid) {
        return;
    }

    // Bearing calculation skipped in SCAN mode and manual yaw control
    // To-Do: remove need for check of control_mode
    if (mode != &mode_scan && !nav_status.manual_control_yaw) {
        nav_status.bearing  = current_loc.get_bearing_to(vehicle.location_estimate) * 0.01f;
    }

    // get_distance() uses Haversine formula for great-circle distance
    nav_status.distance = current_loc.get_distance(vehicle.location_estimate);

    // ALT_SOURCE selection: GPS (tracker+vehicle absolute) vs GPS_VEH_ONLY (vehicle relative)
    if (g.alt_source == ALT_SOURCE_GPS){
        // Uses absolute altitude from both tracker and vehicle GPS (converted cm to m)
        nav_status.alt_difference_gps = (vehicle.location_estimate.alt - current_loc.alt) * 0.01f;
    } else {
        // g.alt_source == ALT_SOURCE_GPS_VEH_ONLY: Uses vehicle relative altitude only
        nav_status.alt_difference_gps = vehicle.relative_alt * 0.01f;
    }

    // Pitch calculation skipped in SCAN mode and manual pitch control
    // To-Do: remove need for check of control_mode
    if (mode->number() != Mode::Number::SCAN && !nav_status.manual_control_pitch) {
        // Pitch from altitude source: BARO vs GPS
    	if (g.alt_source == ALT_SOURCE_BARO) {
            // atan2f(vertical, horizontal) gives elevation angle from barometric altitude
    	    nav_status.pitch = degrees(atan2f(nav_status.alt_difference_baro, nav_status.distance));
    	} else {
            // atan2f(vertical, horizontal) gives elevation angle from GPS altitude
            nav_status.pitch = degrees(atan2f(nav_status.alt_difference_gps, nav_status.distance));
    	}
    }
}

/**
 * @brief Main tracking loop coordinating all tracking functions
 * 
 * @details Called at 50Hz by scheduler. Executes complete tracking sequence in this order:
 *          1. Update vehicle position estimate (velocity projection)
 *          2. Update tracker position (mobile platform support)
 *          3. Calculate bearing/distance/pitch to vehicle
 *          4. Enforce startup delay (prevents servo movement during initialization)
 *          5. Check safety switch (hardware safety interlock)
 *          6. Handle disarmed state (TRIM vs ZERO PWM output per DISARM_PWM parameter)
 *          7. Call mode update() to command servos when armed
 *          8. Convert servo angles to PWM and output to hardware
 *          
 *          Safety interlocks:
 *          - Startup delay allows GPS/compass initialization before servo motion
 *          - Hardware safety switch must be armed for servo movement
 *          - Soft-arm state controls whether servos move or hold disarm position
 *          
 *          Disarmed behavior options (DISARM_PWM parameter):
 *          - TRIM: Outputs neutral position (0° yaw, 0° pitch via set_output_scaled)
 *          - ZERO: Outputs 0 PWM (complete servo power-down via set_output_pwm)
 * 
 * @note This is the main 50Hz tracking scheduler task registered in scheduler_tasks[]
 * @note STARTUP_DELAY parameter prevents servo motion during initialization (default 3 seconds)
 * @warning Safety switch and soft-arm state must both be enabled for servo movement
 * 
 * @see init_ardupilot() for initialization sequence
 * @see scheduler_tasks[] for task registration
 * @see Mode::update() for mode-specific servo control logic
 */
void Tracker::update_tracking(void)
{
    // update vehicle position estimate
    update_vehicle_pos_estimate();

    // update antenna tracker position from GPS
    update_tracker_position();

    // update bearing and distance to vehicle
    update_bearing_and_distance();

    // Startup delay allows GPS/compass initialization before servo motion
    if (g.startup_delay > 0 &&
        AP_HAL::millis() - start_time_ms < g.startup_delay*1000) {
        return;
    }

    // Hardware safety switch must be armed for servo movement
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        return;
    }
    // Soft-arm state controls servo movement vs disarm position
    if (!hal.util->get_soft_armed()) {
        // Disarmed behavior - TRIM outputs neutral (0°), ZERO outputs 0 PWM
        switch ((PWMDisarmed)g.disarm_pwm.get()) {
        case PWMDisarmed::TRIM:
            // TRIM mode: Output neutral position (0° yaw, 0° pitch)
            SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0);
            break;
        default:
        case PWMDisarmed::ZERO:
            // ZERO mode: Output 0 PWM (complete servo power-down)
            SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, 0);
            break;
        }
    } else {
        // Mode update() commands servos based on mode-specific logic (AUTO, MANUAL, SCAN, etc.)
        mode->update();
    }

    // Convert servo_out angles to hardware PWM values and output to servos
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    return;
}

/**
 * @brief Handles MAVLink GLOBAL_POSITION_INT message from tracked vehicle
 * 
 * @details Updates vehicle location and velocity from MAVLink telemetry.
 *          This is the primary position source for tracking - typically sent at 1-10Hz by vehicle.
 *          
 *          Message processing:
 *          - Stores lat/lng/alt in vehicle.location
 *          - Stores relative altitude (above home) in vehicle.relative_alt
 *          - Converts velocity from cm/s (MAVLink) to m/s (internal representation)
 *          - Updates timestamp for timeout detection in update_vehicle_pos_estimate()
 *          - Logs position as VPOS message for analysis
 *          
 *          Unit conversions:
 *          - Altitude: mm (MAVLink) to cm (internal)
 *          - Relative altitude: mm (MAVLink) to cm (internal)
 *          - Velocity: cm/s (MAVLink) to m/s (internal), NED frame
 * 
 * @param[in] msg MAVLink GLOBAL_POSITION_INT message structure
 * 
 * @note Primary position source for tracking - typically sent at 1-10Hz by vehicle
 * @note Velocity used for position prediction between updates
 * 
 * @see update_vehicle_pos_estimate() for velocity-based position projection
 */
void Tracker::tracking_update_position(const mavlink_global_position_int_t &msg)
{
    // Reject invalid (0,0) coordinates - indicates invalid GPS fix
    if (!msg.lat && !msg.lon) {
        return;
    }

    vehicle.location.lat = msg.lat;
    vehicle.location.lng = msg.lon;
    // Altitude conversion from mm to cm (msg.alt is mm above MSL)
    vehicle.location.alt = msg.alt/10;
    // Relative altitude from mm to cm (above vehicle home)
    vehicle.relative_alt = msg.relative_alt/10;
    // Velocity from cm/s to m/s (NED frame: North, East, Down)
    vehicle.vel = Vector3f(msg.vx*0.01f, msg.vy*0.01f, msg.vz*0.01f);
    // Timestamp used by update_vehicle_pos_estimate() for timeout detection
    vehicle.last_update_us = AP_HAL::micros();
    vehicle.last_update_ms = AP_HAL::millis();
#if HAL_LOGGING_ENABLED
    // log vehicle as VPOS
    if (should_log(MASK_LOG_GPS)) {
        Log_Write_Vehicle_Pos(vehicle.location.lat, vehicle.location.lng, vehicle.location.alt, vehicle.vel);
    }
#endif
}


/**
 * @brief Handles MAVLink SCALED_PRESSURE message for barometric altitude
 * 
 * @details Calculates altitude difference using barometer readings from tracker and vehicle.
 *          Barometric altitude provides more accurate short-term tracking than GPS altitude.
 *          
 *          Processing steps:
 *          - Compares vehicle pressure to tracker's local barometer
 *          - Calculates altitude difference using barometric formula
 *          - Applies altitude offset calibration (one-time on first message)
 *          - Stores result in nav_status.alt_difference_baro
 *          - Used when ALT_SOURCE=BARO for pitch calculations
 *          
 *          Calibration:
 *          - One-time calibration performed on first SCALED_PRESSURE message
 *          - Assumes tracker and vehicle at same altitude initially
 *          - Zeros initial altitude offset to eliminate systematic errors
 * 
 * @param[in] msg MAVLink SCALED_PRESSURE message structure
 * 
 * @note Barometric altitude more accurate than GPS for short-term tracking
 * @note One-time calibration performed on first SCALED_PRESSURE message
 * @warning Baro altitude drifts over time due to weather - GPS altitude more stable long-term
 * 
 * @see update_bearing_and_distance() for usage in pitch calculation
 */
void Tracker::tracking_update_pressure(const mavlink_scaled_pressure_t &msg)
{
    // Read local barometer pressure (Pa)
    float local_pressure = barometer.get_pressure();
    // Convert vehicle pressure from hPa to Pa
    float aircraft_pressure = msg.press_abs*100.0f;

    // Barometric altitude formula based on pressure difference
    float alt_diff = barometer.get_altitude_difference(local_pressure, aircraft_pressure);
    if (!isnan(alt_diff) && !isinf(alt_diff)) {
        nav_status.alt_difference_baro = alt_diff + nav_status.altitude_offset;

		if (nav_status.need_altitude_calibration) {
			// One-time calibration zeros initial altitude offset
			// Calibration assumes tracker and vehicle at same altitude initially
			nav_status.altitude_offset = -alt_diff;
			nav_status.alt_difference_baro = 0;
			nav_status.need_altitude_calibration = false;
		}
    }

#if HAL_LOGGING_ENABLED
    // log vehicle baro data
    Log_Write_Vehicle_Baro(aircraft_pressure, alt_diff);
#endif
}

/**
 * @brief Handles MAVLink MANUAL_CONTROL message for external control
 * 
 * @details Allows GCS or companion computer to directly command tracker angles.
 *          Provides override mechanism for testing, setup, and external control applications.
 *          
 *          Message mapping:
 *          - msg.x mapped to bearing (yaw) command in degrees
 *          - msg.y mapped to pitch (elevation) command in degrees
 *          - 0x7FFF indicates axis not controlled (fall back to automatic tracking)
 *          - Distance set to 0 when manually controlled
 *          
 *          Control modes:
 *          - Can control yaw only (x != 0x7FFF, y == 0x7FFF)
 *          - Can control pitch only (x == 0x7FFF, y != 0x7FFF)
 *          - Can control both simultaneously (x != 0x7FFF, y != 0x7FFF)
 * 
 * @param[in] msg MAVLink MANUAL_CONTROL message structure
 * 
 * @note Manual control overrides automatic tracking for specified axes
 * @note Can control yaw only, pitch only, or both simultaneously
 * @note Used for testing, setup, and external control applications
 * 
 * @see update_bearing_and_distance() for automatic tracking that checks manual_control flags
 */
void Tracker::tracking_manual_control(const mavlink_manual_control_t &msg)
{
    nav_status.bearing = msg.x;
    nav_status.pitch   = msg.y;
    // Distance set to 0 when manually controlled
    nav_status.distance = 0.0;
    // 0x7FFF is MAVLink convention for "not controlled" - fall back to automatic tracking
    nav_status.manual_control_yaw   = (msg.x != 0x7FFF);
    nav_status.manual_control_pitch = (msg.y != 0x7FFF);
    // z, r and buttons are not used
}

/**
 * @brief Updates armed LED based on recent vehicle telemetry
 * 
 * @details Sets AP_Notify armed flag to indicate tracking status.
 *          Provides visual indication of whether tracker is receiving valid position updates.
 *          
 *          LED behavior:
 *          - Armed (LED on) if position update received within last 5 seconds
 *          - Disarmed (LED off) if no recent updates or never received
 *          - Provides visual indication of tracking health
 * 
 * @note Armed LED indicates active tracking, not servo arm state
 * @note Timeout matches TRACKING_TIMEOUT_MS used in update_vehicle_pos_estimate()
 * 
 * @see update_vehicle_pos_estimate() for location validity logic
 */
void Tracker::update_armed_disarmed() const
{
    if (vehicle.last_update_ms != 0 && (AP_HAL::millis() - vehicle.last_update_ms) < TRACKING_TIMEOUT_MS) {
        AP_Notify::flags.armed = true;
    } else {
        AP_Notify::flags.armed = false;
    }
}

/**
 * @brief Converts tracker angles to normalized pan/tilt for ONVIF cameras
 * 
 * @details Provides standardized output for scripting control of ONVIF cameras.
 *          Maps tracker angles from configured parameter ranges to -1..1 normalized values.
 *          
 *          Normalization formulas:
 *          - Tilt: Maps pitch from PITCH_MIN..PITCH_MAX to -1..1
 *            * Constrains pitch to limits, applies PITCH_TRIM offset
 *            * Formula: (((pitch+trim - min)*2)/(max-min)) - 1
 *          - Pan: Maps yaw from 0..YAW_RANGE to -1..1
 *            * Wraps yaw to 0-360°, applies YAW_TRIM offset
 *            * Formula: (wrap_360(bearing+trim)*2/range) - 1
 * 
 * @param[out] pan_norm Normalized pan value -1 to 1 (left to right)
 * @param[out] tilt_norm Normalized tilt value -1 to 1 (down to up)
 * 
 * @return true (always successful)
 * 
 * @note Used by Lua scripting to control gimbal cameras via ONVIF protocol
 * @note Normalized output suitable for standard camera control interfaces
 * 
 * @see AP_Scripting for Lua scripting interface
 */
bool Tracker::get_pan_tilt_norm(float &pan_norm, float &tilt_norm) const
{
    float pitch = nav_status.pitch;
    float bearing = nav_status.bearing;
    // Constrain pitch to limits, apply trim, scale to -1..1
    tilt_norm = (((constrain_float(pitch+g.pitch_trim, g.pitch_min, g.pitch_max) - g.pitch_min)*2.0f)/(g.pitch_max - g.pitch_min)) - 1;
    // Wrap yaw to 0-360°, apply trim, scale to -1..1
    pan_norm = (wrap_360(bearing+g.yaw_trim)*2.0f/(g.yaw_range)) - 1;
    return true;
}
