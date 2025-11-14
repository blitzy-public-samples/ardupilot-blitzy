/**
 * @file mode.cpp
 * @brief Mode base class implementation with shared tracking algorithms
 * 
 * @details Implements common functions used by multiple tracking modes including:
 *          - Automatic vehicle tracking with angle error calculation
 *          - Scanning pattern for vehicle search
 *          - Coordinate frame transformations (earth frame to body frame)
 *          - Yaw direction determination for non-continuous rotation servos
 * 
 *          These shared functions support AUTO, SCAN, and other tracking modes
 *          by providing core tracking mathematics and servo control logic.
 * 
 * @note Coordinate frames used:
 *       - Earth Frame (EF): North-referenced absolute angles (yaw 0-360°, pitch in pitch_min to pitch_max)
 *       - Body Frame (BF): Relative to tracker mount orientation, accounts for tracker platform attitude
 * 
 * @warning Angle calculations are timing-sensitive and called at servo update rate (typically 50Hz)
 * 
 * Source: AntennaTracker/mode.cpp
 */

#include "mode.h"

#include "Tracker.h"

/**
 * @brief Implements automatic vehicle tracking
 * 
 * @details Calculates target angles based on vehicle position and commands servos to track:
 *          1. Computes target yaw from nav_status.bearing plus yaw trim
 *          2. Computes target pitch from nav_status.pitch plus pitch trim, constrained to limits
 *          3. Determines if yaw direction should be reversed (for limited rotation servos)
 *          4. Calculates angle errors between current attitude and targets
 *          5. Converts earth frame angles to body frame
 *          6. Commands yaw and pitch servos (only if vehicle beyond minimum distance)
 * 
 *          This function is called by AUTO mode when vehicle position is valid,
 *          and also by SCAN mode to command servos after updating scan angles.
 * 
 * @note Called at servo update rate (typically 50Hz)
 * @note Servo commands are inhibited if vehicle is closer than distance_min parameter
 * 
 * @see Tracker::update_bearing_and_distance() - Updates nav_status bearing and pitch
 * @see Mode::get_ef_yaw_direction() - Determines yaw direction reversal
 * @see Mode::calc_angle_error() - Calculates tracking errors
 * @see Mode::convert_ef_to_bf() - Converts angles to body frame
 * 
 * Source: AntennaTracker/mode.cpp:5-27
 */
void Mode::update_auto(void)
{
    struct Tracker::NavStatus &nav_status = tracker.nav_status;

    Parameters &g = tracker.g;

    // Calculate target yaw in centidegrees: bearing (0-360°) + yaw_trim, wrapped to ±180°
    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100); // target yaw in centidegrees
    
    // Calculate target pitch in centidegrees: pitch + pitch_trim, constrained to configured limits
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, g.pitch_min, g.pitch_max) * 100; // target pitch in centidegrees

    // Determine if yaw direction should be reversed (for "Ballerina" algorithm on limited rotation servos)
    bool direction_reversed = get_ef_yaw_direction();

    // Calculate angle errors between current AHRS attitude and targets, store in nav_status
    calc_angle_error(pitch, yaw, direction_reversed);

    // Convert earth frame target angles to body frame for servo commands
    float bf_pitch;
    float bf_yaw;
    convert_ef_to_bf(pitch, yaw, bf_pitch, bf_yaw);

    // Only move servos if target is at least distance_min away (or distance_min disabled or no valid location)
    // This prevents servo jitter when vehicle is very close to tracker
    if ((g.distance_min <= 0) || (nav_status.distance >= g.distance_min) || !tracker.vehicle.location_valid) {
        tracker.update_pitch_servo(bf_pitch);
        tracker.update_yaw_servo(bf_yaw);
    }
}

/**
 * @brief Implements scanning pattern to search for vehicle
 * 
 * @details Sweeps antenna through yaw and pitch ranges to search for lost vehicle:
 *          
 *          Yaw Scan Pattern (unless manual control active):
 *          - Increments bearing at SCAN_SPEED_YAW rate (deg/s * 0.02 for 50Hz update)
 *          - Scans from 0° to 360° then reverses direction
 *          - Reverses at limits: scan_reverse_yaw flag toggles
 *          
 *          Pitch Scan Pattern (unless manual control active):
 *          - Increments/decrements pitch at SCAN_SPEED_PITCH rate
 *          - Scans between pitch_min and pitch_max limits
 *          - Reverses direction at limits: scan_reverse_pitch flag toggles
 *          
 *          After updating scan angles, calls update_auto() to command servos
 *          with the new target angles. Continues indefinitely until vehicle
 *          found or mode changed.
 * 
 * @note Used by SCAN mode and AUTO mode fallback when vehicle position lost
 * @note Scan speeds configured by SCAN_SPEED_YAW and SCAN_SPEED_PITCH parameters (deg/s)
 * @note Manual control flags allow pilot override of yaw/pitch during scan
 * @note Called at 50Hz servo update rate, delta calculations use 0.02s time step
 * 
 * @warning Ensure pitch_min and pitch_max are properly configured to prevent mechanical limits
 * 
 * Source: AntennaTracker/mode.cpp:29-64
 */
void Mode::update_scan(void)
{
    struct Tracker::NavStatus &nav_status = tracker.nav_status;

    Parameters &g = tracker.g;

    // Yaw scan state machine: sweep 0-360° and reverse at limits
    if (!nav_status.manual_control_yaw) {
        // Calculate yaw increment for this update cycle (scan_speed_yaw in deg/s * 0.02s)
        float yaw_delta = g.scan_speed_yaw * 0.02f;
        
        // Increment bearing in current direction (forward or reverse)
        nav_status.bearing   += yaw_delta   * (nav_status.scan_reverse_yaw?-1:1);
        
        // Check for lower limit: if bearing < 0° while scanning reverse, switch to forward
        if (nav_status.bearing < 0 && nav_status.scan_reverse_yaw) {
            nav_status.scan_reverse_yaw = false;
        }
        
        // Check for upper limit: if bearing > 360° while scanning forward, switch to reverse
        if (nav_status.bearing > 360 && !nav_status.scan_reverse_yaw) {
            nav_status.scan_reverse_yaw = true;
        }
        
        // Constrain bearing to valid range [0, 360]
        nav_status.bearing = constrain_float(nav_status.bearing, 0, 360);
    }

    // Pitch scan state machine: sweep between pitch_min and pitch_max limits
    if (!nav_status.manual_control_pitch) {
        // Calculate pitch increment for this update cycle (scan_speed_pitch in deg/s * 0.02s)
        const float pitch_delta = g.scan_speed_pitch * 0.02f;
        
        if (nav_status.scan_reverse_pitch) {
            // Scanning downward: decrement pitch
            nav_status.pitch -= pitch_delta;
            
            // Check for lower pitch limit: switch to upward scan
            if (nav_status.pitch < g.pitch_min) {
                nav_status.scan_reverse_pitch = false;
            }
        } else {
            // Scanning upward: increment pitch
            nav_status.pitch += pitch_delta;
            
            // Check for upper pitch limit: switch to downward scan
            if (nav_status.pitch > g.pitch_max) {
                nav_status.scan_reverse_pitch = true;
            }
        }
        
        // Constrain pitch to configured limits
        nav_status.pitch = constrain_float(nav_status.pitch, g.pitch_min, g.pitch_max);
    }

    // Command servos to track the updated scan position
    update_auto();
}

/**
 * @brief Calculates angle tracking errors between current attitude and targets
 * 
 * @details Computes pitch and yaw errors in earth frame, then converts to body frame:
 *          1. Calculate pitch error: target pitch - current AHRS pitch (centidegrees)
 *             - Positive error: target is above current pitch
 *             - Negative error: target is below current pitch
 *          2. Calculate yaw error: target yaw - current AHRS yaw (centidegrees, wrapped ±180°)
 *             - Positive error: target is right of current yaw
 *             - Negative error: target is left of current yaw
 *          3. Handle direction reversal for limited rotation servos (adjusts yaw error by ±360°)
 *          4. Convert earth frame errors to body frame using current tracker attitude
 *          5. Store body frame errors in nav_status for use by servo controllers
 *          6. Update PID controller targets for logging
 * 
 * @param[in] pitch Target pitch in centidegrees (earth frame, -9000 to 9000)
 * @param[in] yaw Target yaw in centidegrees (earth frame, ±18000 representing ±180°)
 * @param[in] direction_reversed Whether yaw direction should be reversed (from get_ef_yaw_direction)
 * 
 * @note Errors stored in nav_status.angle_error_pitch and nav_status.angle_error_yaw (body frame)
 * @note PID set_target_rate/set_actual_rate calls are for logging, not control (angles, not rates)
 * @note Direction reversal implements "Ballerina" algorithm for non-continuous rotation servos
 * 
 * @see Mode::get_ef_yaw_direction() - Determines if direction reversal needed
 * @see Mode::convert_ef_to_bf() - Converts earth frame to body frame
 * 
 * Source: AntennaTracker/mode.cpp:66-102
 */
void Mode::calc_angle_error(float pitch, float yaw, bool direction_reversed)
{
    // Calculate pitch angle error in centidegrees (earth frame)
    // Positive error means the target is above current pitch
    // Negative error means the target is below current pitch
    const AP_AHRS &ahrs = AP::ahrs();
    float ahrs_pitch = ahrs.pitch_sensor;
    int32_t ef_pitch_angle_error = pitch - ahrs_pitch;

    // Calculate yaw angle error in centidegrees (earth frame)
    // Positive error means the target is right of current yaw
    // Negative error means the target is left of current yaw
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t ef_yaw_angle_error = wrap_180_cd(yaw - ahrs_yaw_cd);
    
    // Handle direction reversal for limited rotation servos ("Ballerina" algorithm)
    // When reversed, take the long way around to avoid hitting mechanical limits
    if (direction_reversed) {
        if (ef_yaw_angle_error > 0) {
            // Target is to the right, but we need to go left (the long way)
            // Subtract 360° to make error negative
            ef_yaw_angle_error = (yaw - ahrs_yaw_cd) - 36000;
        } else {
            // Target is to the left, but we need to go right (the long way)
            // Add 360° to make error positive
            ef_yaw_angle_error = 36000 + (yaw - ahrs_yaw_cd);
        }
    }

    // Convert earth frame angle errors to body frame for servo control
    // Body frame accounts for tracker platform roll/pitch/yaw attitude
    float bf_pitch_err;
    float bf_yaw_err;
    convert_ef_to_bf(ef_pitch_angle_error, ef_yaw_angle_error, bf_pitch_err, bf_yaw_err);
    
    // Store body frame errors in nav_status for servo controllers to use
    struct Tracker::NavStatus &nav_status = tracker.nav_status;
    nav_status.angle_error_pitch = bf_pitch_err;
    nav_status.angle_error_yaw = bf_yaw_err;

    // Set actual and desired for logging (note: using angles not rates despite function names)
    // Convert from centidegrees to degrees for logging
    Parameters &g = tracker.g;
    g.pidPitch2Srv.set_target_rate(pitch * 0.01);
    g.pidPitch2Srv.set_actual_rate(ahrs_pitch * 0.01);
    g.pidYaw2Srv.set_target_rate(yaw * 0.01);
    g.pidYaw2Srv.set_actual_rate(ahrs_yaw_cd * 0.01);
}

/**
 * @brief Converts angles from earth frame to body frame
 * 
 * @details Transforms pitch and yaw angles from earth frame (North-referenced absolute angles)
 *          to body frame (relative to tracker mount orientation). Uses rotation matrix
 *          based on current tracker platform roll and pitch from AHRS.
 *          
 *          Earth Frame: North-referenced absolute angles (yaw 0° = North, pitch relative to horizon)
 *          Body Frame: Relative to tracker mount orientation, accounting for platform attitude
 *          
 *          Transformation uses simplified 2D rotation accounting for tracker roll and pitch:
 *          - bf_pitch = cos(roll) * pitch + sin(roll) * cos(pitch) * yaw
 *          - bf_yaw = -sin(roll) * pitch + cos(pitch) * cos(roll) * yaw
 * 
 * @param[in]  pitch    Earth frame pitch angle (degrees or centidegrees)
 * @param[in]  yaw      Earth frame yaw angle (degrees or centidegrees)
 * @param[out] bf_pitch Body frame pitch output (same units as input)
 * @param[out] bf_yaw   Body frame yaw output (same units as input)
 * 
 * @note Required for mobile tracker platforms where mount orientation changes
 * @note For stationary trackers with level mounting, body frame ≈ earth frame
 * @note Units preserved: if inputs in centidegrees, outputs in centidegrees
 * 
 * @see Mode::convert_bf_to_ef() - Inverse transformation
 * 
 * Source: AntennaTracker/mode.cpp:104-110
 */
void Mode::convert_ef_to_bf(float pitch, float yaw, float& bf_pitch, float& bf_yaw)
{
	// Earth frame to body frame pitch and yaw conversion using rotation matrix
    const AP_AHRS &ahrs = AP::ahrs();
    bf_pitch = ahrs.cos_roll() * pitch + ahrs.sin_roll() * ahrs.cos_pitch() * yaw;
    bf_yaw = -ahrs.sin_roll() * pitch + ahrs.cos_pitch() * ahrs.cos_roll() * yaw;
}

/**
 * @brief Converts angles from body frame to earth frame
 * 
 * @details Transforms pitch and yaw angles from body frame (relative to tracker mount)
 *          to earth frame (North-referenced absolute angles). Uses inverse rotation
 *          matrix based on current tracker platform roll and pitch from AHRS.
 *          
 *          Body Frame: Relative to tracker mount orientation, accounting for platform attitude
 *          Earth Frame: North-referenced absolute angles (yaw 0° = North, pitch relative to horizon)
 *          
 *          Inverse transformation from earth-to-body conversion:
 *          - ef_pitch = cos(roll) * pitch - sin(roll) * yaw
 *          - ef_yaw = (sin(roll) / cos(pitch)) * pitch + (cos(roll) / cos(pitch)) * yaw
 *          
 *          Division by cos(pitch) requires singularity check at pitch = ±90°
 * 
 * @param[in]  pitch    Body frame pitch angle (degrees or centidegrees)
 * @param[in]  yaw      Body frame yaw angle (degrees or centidegrees)
 * @param[out] ef_pitch Earth frame pitch output (same units as input)
 * @param[out] ef_yaw   Earth frame yaw output (same units as input)
 * 
 * @return true if conversion successful, false if pitch = ±90° (singularity, division by zero)
 * 
 * @note Required for mobile tracker platforms where mount orientation changes
 * @note For stationary trackers with level mounting, earth frame ≈ body frame
 * @note Units preserved: if inputs in centidegrees, outputs in centidegrees
 * @note Returns false at pitch = ±90° gimbal lock condition
 * 
 * @warning Check return value before using output - conversion fails at pitch = ±90°
 * 
 * @see Mode::convert_ef_to_bf() - Forward transformation
 * 
 * Source: AntennaTracker/mode.cpp:112-123
 */
bool Mode::convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw)
{
    const AP_AHRS &ahrs = AP::ahrs();
    
    // Avoid divide by zero at pitch = ±90° (gimbal lock singularity)
    if (is_zero(ahrs.cos_pitch())) {
        return false;
    }
    
    // Convert body frame angles to earth frame using inverse rotation matrix
    ef_pitch = ahrs.cos_roll() * pitch - ahrs.sin_roll() * yaw;
    ef_yaw = (ahrs.sin_roll() / ahrs.cos_pitch()) * pitch + (ahrs.cos_roll() / ahrs.cos_pitch()) * yaw;
    return true;
}

/**
 * @brief Determines if yaw direction should be reversed for tracking
 * 
 * @details Implements "Ballerina" algorithm for antenna trackers with limited rotation servos
 *          (non-continuous rotation). Determines whether to take shortest path or long way
 *          around to avoid hitting mechanical yaw limits.
 *          
 *          Algorithm:
 *          1. Calculate body frame distance to lower/upper yaw and pitch limits
 *          2. Convert body frame limits to earth frame limits
 *          3. Calculate angle error to target in both directions (CW and CCW)
 *          4. Check if vehicle is outside tracker's yaw range
 *          5. If outside range, determine which direction gets closer without hitting limits
 *          
 *          Returns true (direction reversed) when:
 *          - Vehicle is outside yaw range AND
 *          - Taking the long way around gets closer to vehicle without hitting limits
 *          
 *          This prevents the tracker from continuously trying to reach an unreachable
 *          target by moving in a direction that hits mechanical limits.
 * 
 * @return true if yaw direction should be reversed (take long way around)
 * @return false if normal shortest direction should be used
 * 
 * @note "Ballerina" algorithm name: servo rotates back and forth like a ballerina spin
 * @note Only relevant for servos with limited rotation range (not continuous rotation)
 * @note For continuous rotation servos, this function should return false
 * @note Called by update_auto() and calc_angle_error() every servo update cycle
 * 
 * @warning Requires proper yaw_range parameter configuration to match mechanical limits
 * 
 * Source: AntennaTracker/mode.cpp:126-176
 */
// Return value is true if taking the long road to the target, false if normal, shortest direction should be used
bool Mode::get_ef_yaw_direction()
{
    // Calculate distances from current pitch/yaw to lower and upper limits in centidegrees (body frame)
    Parameters &g = tracker.g;
    // Calculate body frame limits: yaw_range centered at 0, pitch from pitch_min to pitch_max
    // Subtract current filtered servo output to get distance remaining to limits
    float yaw_angle_limit_lower =   (-g.yaw_range * 100.0f / 2.0f) - tracker.yaw_servo_out_filt.get();
    float yaw_angle_limit_upper =   (g.yaw_range * 100.0f / 2.0f) - tracker.yaw_servo_out_filt.get();
    float pitch_angle_limit_lower = (g.pitch_min * 100.0f) - tracker.pitch_servo_out_filt.get();
    float pitch_angle_limit_upper = (g.pitch_max * 100.0f) - tracker.pitch_servo_out_filt.get();

    // Convert body frame limits to earth frame limits
    // Initialize with body frame values, then transform
    float ef_yaw_limit_lower = yaw_angle_limit_lower;
    float ef_yaw_limit_upper = yaw_angle_limit_upper;
    float ef_pitch_limit_lower = pitch_angle_limit_lower;
    float ef_pitch_limit_upper = pitch_angle_limit_upper;
    convert_bf_to_ef(pitch_angle_limit_lower, yaw_angle_limit_lower, ef_pitch_limit_lower, ef_yaw_limit_lower);
    convert_bf_to_ef(pitch_angle_limit_upper, yaw_angle_limit_upper, ef_pitch_limit_upper, ef_yaw_limit_upper);

    // Get current earth frame yaw and calculate target yaw
    const AP_AHRS &ahrs = AP::ahrs();
    float ef_yaw_current = wrap_180_cd(ahrs.yaw_sensor);
    struct Tracker::NavStatus &nav_status = tracker.nav_status;
    float ef_yaw_target = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100);
    
    // Calculate yaw angle error using shortest path (±180°)
    float ef_yaw_angle_error = wrap_180_cd(ef_yaw_target - ef_yaw_current);

    // Calculate angle error to target in BOTH directions (clockwise and counter-clockwise)
    // This allows us to compare which direction gets closer without hitting limits
    float yaw_angle_error_upper;  // Error if moving right/clockwise
    float yaw_angle_error_lower;  // Error if moving left/counter-clockwise
    
    if (ef_yaw_angle_error >= 0) {
        // Shortest path is right/clockwise (positive error)
        yaw_angle_error_upper = ef_yaw_angle_error;
        yaw_angle_error_lower = ef_yaw_angle_error - 36000;  // Alternative: go left (negative by 360°)
    } else {
        // Shortest path is left/counter-clockwise (negative error)
        yaw_angle_error_upper = 36000 + ef_yaw_angle_error;  // Alternative: go right (positive by 360°)
        yaw_angle_error_lower = ef_yaw_angle_error;
    }

    // Check if vehicle is outside the tracker's yaw range
    // If BOTH directional errors exceed limits, vehicle is unreachable by shortest path
    if ((yaw_angle_error_lower < ef_yaw_limit_lower) && (yaw_angle_error_upper > ef_yaw_limit_upper)) {
        
        // Vehicle is outside range - determine which direction gets closer
        // Compare distance to lower limit vs distance to upper limit
        
        // If tracker is trying to move clockwise (positive error) to reach vehicle,
        // but counter-clockwise direction would get closer without hitting limits, reverse
        if (ef_yaw_angle_error>0 && ((ef_yaw_limit_lower - yaw_angle_error_lower) < (yaw_angle_error_upper - ef_yaw_limit_upper))) {
            return true;  // Reverse direction: go counter-clockwise instead
        }
        
        // If tracker is trying to move counter-clockwise (negative error) to reach vehicle,
        // but clockwise direction would get closer without hitting limits, reverse
        if (ef_yaw_angle_error<0 && ((ef_yaw_limit_lower - yaw_angle_error_lower) > (yaw_angle_error_upper - ef_yaw_limit_upper))) {
            return true;  // Reverse direction: go clockwise instead
        }
    }

    // Vehicle is within range or shortest path doesn't hit limits - use normal shortest path
    return false;
}
