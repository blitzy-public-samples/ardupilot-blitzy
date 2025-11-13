/**
 * @file inertia.cpp
 * @brief Inertial navigation system integration for ArduSub
 * 
 * @details This file handles the integration of inertial navigation estimates
 *          with the ArduSub vehicle state. It updates position and velocity
 *          estimates by fusing data from the AP_InertialNav library with
 *          AHRS (Attitude and Heading Reference System) outputs.
 *          
 *          The inertial navigation system provides:
 *          - 3D position estimates in NED (North-East-Down) frame
 *          - 3D velocity estimates in NED frame
 *          - Vertical position referenced from water surface
 *          
 *          For underwater vehicles, the inertial navigation system is critical
 *          for depth hold, position hold, and autonomous navigation modes where
 *          GPS is unavailable.
 * 
 * @note This is called at the main loop rate (typically 400Hz for ArduSub)
 * @warning Accurate position estimates depend on proper sensor calibration
 *          and EKF convergence. Always verify EKF health before autonomous missions.
 * 
 * @see AP_InertialNav for inertial navigation library implementation
 * @see AP_AHRS for attitude and heading reference system
 * @see AC_PosControl for position controller that consumes these estimates
 * 
 * Source: ArduSub/inertia.cpp
 */

#include "Sub.h"

/**
 * @brief Read and update inertial navigation position and velocity estimates
 * 
 * @details This function updates the vehicle's position and velocity estimates
 *          by integrating data from the inertial navigation system (AP_InertialNav)
 *          with AHRS position estimates. The function performs the following:
 *          
 *          1. Updates inertial navigation estimates (accelerometer integration)
 *          2. Updates position controller estimates for control loop
 *          3. Retrieves horizontal position (lat/lng) from AHRS
 *          4. Checks for valid vertical position estimate availability
 *          5. Retrieves vertical position (depth) in z-up coordinate frame
 *          6. Retrieves vertical velocity (climb rate) in z-up coordinate frame
 *          
 *          Coordinate Frame Conventions:
 *          - Horizontal position: Latitude/Longitude (WGS84 datum)
 *          - Vertical position: Z-up frame (positive = up from water surface)
 *          - NED frame is used internally by EKF, converted to z-up for Sub
 *          - Altitude reference: Water surface (0 = surface, negative = underwater)
 *          
 *          Units:
 *          - Latitude/Longitude: degrees * 1e7 (ArduPilot Location format)
 *          - Altitude: centimeters (cm)
 *          - Climb rate: centimeters per second (cm/s)
 * 
 * @note Called at main loop rate (400Hz) from Sub::fast_loop()
 * @note Vertical position estimates require EKF vertical position health
 * @note Horizontal position may be available even without vertical estimate
 * 
 * @warning If vertical position estimate is unavailable (AP_AHRS::Status::VERT_POS false),
 *          vertical position and velocity will not be updated. This can occur during:
 *          - EKF initialization phase
 *          - Loss of depth sensor (barometer or external depth sensor)
 *          - EKF divergence or health failure
 *          
 *          In such cases, depth hold and autonomous modes should be avoided.
 * 
 * Integration with AP_InertialNav:
 * - AP_InertialNav fuses accelerometer data with EKF position corrections
 * - Provides high-rate position estimates between slower EKF updates
 * - Maintains position history for control loop lag compensation
 * - Uses complementary filter to blend IMU integration with EKF corrections
 * 
 * @see AP_InertialNav::update() for inertial navigation update implementation
 * @see AP_InertialNav::get_position_z_up_cm() for vertical position retrieval
 * @see AP_InertialNav::get_velocity_z_up_cms() for vertical velocity retrieval
 * @see AP_AHRS::get_location() for horizontal position from AHRS
 * @see AP_AHRS::has_status() for position estimate health checking
 * @see AC_PosControl::update_estimates() for position controller estimate updates
 * 
 * Source: ArduSub/inertia.cpp:4-26
 */
void Sub::read_inertia()
{
    // Update inertial altitude and position estimates
    // AP_InertialNav integrates accelerometer data with EKF corrections
    // to provide high-rate position estimates for the control loops
    inertial_nav.update();
    
    // Update position controller's internal position estimates
    // This ensures the position controller has current estimates for
    // the next control loop iteration
    sub.pos_control.update_estimates();

    // Retrieve horizontal position (latitude/longitude) from AHRS
    // AHRS provides fused position from EKF, which blends GPS (when available),
    // visual odometry, and other position sources
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;  // Latitude in degrees * 1e7
    current_loc.lng = loc.lng;  // Longitude in degrees * 1e7

    // Position accuracy check: Verify vertical position estimate is available
    // Without vertical position health, we cannot trust depth or altitude readings
    // This check prevents using invalid position data that could cause control issues
    // 
    // AP_AHRS::Status::VERT_POS indicates EKF has converged on vertical position
    // and the estimate meets accuracy thresholds for control use
    if (!AP::ahrs().has_status(AP_AHRS::Status::VERT_POS)) {
        // Exit without updating vertical position or velocity
        // Depth hold and altitude-dependent modes should check this status
        // before attempting position control
        return;
    }

    // Retrieve vertical position in z-up coordinate frame
    // Z-up convention: positive = above water surface, negative = underwater
    // Position is referenced from water surface (0 = surface)
    // Unit: centimeters (cm)
    current_loc.alt = inertial_nav.get_position_z_up_cm();

    // Retrieve vertical velocity (climb rate) in z-up coordinate frame
    // Positive climb_rate = ascending (moving toward surface)
    // Negative climb_rate = descending (moving deeper)
    // Unit: centimeters per second (cm/s)
    // 
    // Note: Altitude is always in absolute frame, referenced from water's surface
    // This differs from aircraft where altitude may be relative to home or terrain
    climb_rate = inertial_nav.get_velocity_z_up_cms();
}
