/**
 * @file sensors.cpp
 * @brief Implementation of rover sensor reading and EKF integration
 * 
 * @details This file handles periodic sensor updates for the Rover vehicle including:
 *          - Compass (magnetometer) readings for heading estimation
 *          - Wheel encoder odometry for precise position estimation
 *          - Rangefinder updates for obstacle detection and terrain following
 * 
 *          Sensor data is read from hardware drivers and fed to the AHRS/EKF
 *          for sensor fusion and state estimation. Data flows from hardware
 *          through AP_HAL device drivers, into sensor libraries, and finally
 *          to the navigation filter (EKF3).
 * 
 * @note Coordinate frames used:
 *       - Body frame: Forward-Right-Down relative to vehicle
 *       - NED frame: North-East-Down earth-fixed frame used by EKF
 *       - Wheel encoder positions specified in body frame (XYZ in meters)
 * 
 * Source: Rover/sensors.cpp
 */

#include "Rover.h"

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

/**
 * @brief Update compass (magnetometer) readings from hardware
 * 
 * @details This method polls the compass hardware for new magnetometer data
 *          and updates the internal compass state. Called at 10Hz from the
 *          main scheduler. The compass data is automatically fed to the AHRS/EKF
 *          by the AP_Compass library for heading estimation and position correction.
 * 
 *          The compass.read() call:
 *          - Reads raw magnetic field data from all configured magnetometers
 *          - Applies calibration offsets and motor compensation
 *          - Updates healthy/unhealthy status for each compass
 *          - Provides data to AHRS for magnetic heading calculation
 * 
 * @note Called at 10Hz from main scheduler task
 * @note Compass data in body frame is automatically transformed to earth frame by AHRS
 * 
 * @see AP_Compass::read()
 * @see AP_AHRS::update()
 */
void Rover::update_compass(void)
{
    // Poll all enabled compass hardware for new magnetic field readings
    // Data automatically integrated into AHRS heading estimation
    compass.read();
}

/**
 * @brief Update wheel encoder readings and provide odometry data to EKF3
 * 
 * @details This method reads wheel encoder data from hardware and sends wheel
 *          odometry measurements to EKF3 for improved position and velocity estimation.
 *          Wheel encoders provide accurate relative motion measurement that is particularly
 *          useful when GPS is degraded or unavailable.
 * 
 *          Algorithm flow:
 *          1. Poll wheel encoder hardware for rotation data (counts/angles)
 *          2. Save cumulative distance for GCS telemetry reporting
 *          3. Initialize state on first call to establish baseline
 *          4. Round-robin through multiple encoders if present
 *          5. Calculate delta angle (rotation since last measurement)
 *          6. Calculate delta time with sensor timestamp validation
 *          7. Send odometry data to EKF3 for sensor fusion
 * 
 *          The EKF3 integrates wheel odometry as visual odometry measurements,
 *          fusing them with IMU, GPS, and compass data for robust state estimation.
 *          This improves position accuracy during GPS dropouts and reduces position
 *          drift in challenging environments.
 * 
 * @note Called from main scheduler, should not exceed 50Hz to avoid overloading EKF
 * @note Supports multiple wheel encoders - sends one encoder per call in round-robin
 * @note Coordinate frame: Wheel positions specified in body frame (Forward-Right-Down)
 * @note Positive delta angle indicates forward vehicle motion
 * 
 * @warning Wheel encoder radius and position offset must be correctly configured
 *          for accurate velocity and position estimation
 * 
 * @see AP_WheelEncoder::update()
 * @see NavEKF3::writeWheelOdom()
 */
void Rover::update_wheel_encoder()
{
    // Exit immediately if no wheel encoders are configured
    // Avoids unnecessary processing when feature is disabled
    if (g2.wheel_encoder.num_sensors() == 0) {
        return;
    }

    // Poll all wheel encoder hardware for new rotation measurements
    // Updates internal state with latest counts/angles from sensors
    g2.wheel_encoder.update();

    // Save cumulative distance traveled for each encoder (in meters)
    // Used for GCS telemetry reporting and mission monitoring
    for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
        wheel_encoder_last_distance_m[i] = g2.wheel_encoder.get_distance(i);
    }

    // Initialize wheel encoder state on first call to establish baseline values
    // This prevents sending invalid delta measurements to EKF on startup
    // Rate limit: Should not send data to EKF at more than 50Hz to avoid overload
    if (!wheel_encoder_initialised) {
        wheel_encoder_initialised = true;
        // Store initial angle and timestamp for each encoder
        for (uint8_t i = 0; i < g2.wheel_encoder.num_sensors(); i++) {
            wheel_encoder_last_angle_rad[i] = g2.wheel_encoder.get_delta_angle(i);
            wheel_encoder_last_reading_ms[i] = g2.wheel_encoder.get_last_reading_ms(i);
        }
        return;
    }

    // Round-robin through multiple wheel encoders, sending one per iteration
    // This distributes EKF processing load when multiple encoders are present
    // and ensures all encoders contribute to state estimation
    wheel_encoder_last_index_sent++;
    if (wheel_encoder_last_index_sent >= g2.wheel_encoder.num_sensors()) {
        wheel_encoder_last_index_sent = 0;
    }

    // Get current wheel encoder data for the selected sensor:
    // - Total rotation angle since startup (radians, cumulative)
    // - Timestamp of last sensor reading (milliseconds)
    // - Current system time for delta time calculation
    const float curr_angle_rad = g2.wheel_encoder.get_delta_angle(wheel_encoder_last_index_sent);
    const uint32_t sensor_reading_ms = g2.wheel_encoder.get_last_reading_ms(wheel_encoder_last_index_sent);
    const uint32_t now_ms = AP_HAL::millis();

    // Calculate change in wheel rotation angle since last measurement
    // Positive delta angle indicates forward vehicle motion in body frame
#if HAL_NAVEKF3_AVAILABLE
    const float delta_angle = curr_angle_rad - wheel_encoder_last_angle_rad[wheel_encoder_last_index_sent];
#endif
    // Update stored angle for next iteration's delta calculation
    wheel_encoder_last_angle_rad[wheel_encoder_last_index_sent] = curr_angle_rad;

    // Calculate time interval for this measurement with validation
    // Use sensor timestamp delta if valid, otherwise use time since last EKF send
    // This handles cases where sensor update rate differs from this function's call rate
    uint32_t sensor_diff_ms = sensor_reading_ms - wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent];
    if (sensor_diff_ms == 0 || sensor_diff_ms > 100) {
        // No sensor update (0ms) or stale data (>100ms) detected
        // Fall back to time since last EKF update to maintain valid delta time
        sensor_diff_ms = now_ms - wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent];
        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent] = now_ms;
    } else {
        // Sensor timestamp is valid and recent, use sensor reading time
        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent] = sensor_reading_ms;
    }
    
#if HAL_NAVEKF3_AVAILABLE
    // Convert delta time from milliseconds to seconds for EKF
    const float delta_time = sensor_diff_ms * 0.001f;

    // Send wheel odometry data to EKF3 for sensor fusion
    // EKF3 processes this as visual odometry input, fusing with IMU/GPS/compass
    // 
    // Parameters passed to EKF3:
    // - delAng: Change in wheel rotation angle (radians) - positive = forward motion
    // - delTime: Time interval for measurement (seconds)
    // - timeStamp_ms: Time when rotation was measured (milliseconds)
    // - posOffset: XYZ body frame position of wheel hub center (meters, Forward-Right-Down)
    // - radius: Wheel radius (meters) for converting rotation to linear displacement
    //
    // EKF uses wheel radius and delta angle to calculate linear velocity in body frame,
    // then transforms to NED frame for integration with other navigation sensors
    ahrs.EKF3.writeWheelOdom(delta_angle,
                        delta_time,
                        wheel_encoder_last_reading_ms[wheel_encoder_last_index_sent],
                        g2.wheel_encoder.get_pos_offset(wheel_encoder_last_index_sent),
                        g2.wheel_encoder.get_wheel_radius(wheel_encoder_last_index_sent));
#endif
}

#if AP_RANGEFINDER_ENABLED
/**
 * @brief Update rangefinder (distance sensor) readings for obstacle detection
 * 
 * @details This method polls all configured rangefinder sensors to get distance
 *          measurements for obstacle avoidance, terrain following, and depth sensing
 *          (for boats). Rangefinder data is automatically integrated into the EKF
 *          for terrain altitude estimation when terrain following is enabled.
 * 
 *          The rangefinder.update() call:
 *          - Polls all enabled rangefinder hardware (lidar, sonar, radar, etc.)
 *          - Applies sensor-specific filtering and validation
 *          - Updates distance measurements and health status
 *          - Provides data to object avoidance systems
 *          - Feeds terrain altitude to EKF when configured
 * 
 *          After updating sensor data, depth information is logged for:
 *          - Post-mission analysis of terrain following performance
 *          - Obstacle detection event logging
 *          - Water depth recording for boat operations
 * 
 * @note Called from main scheduler at sensor-dependent rate (typically 10-20Hz)
 * @note Rangefinder coordinate frame: Downward in body frame for terrain sensing,
 *       forward/sideways for obstacle detection depending on mounting
 * @note Conditional compilation: Only included if AP_RANGEFINDER_ENABLED is defined
 * 
 * @warning Rangefinder mounting position and orientation must be correctly configured
 *          in parameters for accurate obstacle detection and terrain following
 * 
 * @see AP_RangeFinder::update()
 * @see Rover::Log_Write_Depth()
 * @see AC_Avoidance for obstacle avoidance integration
 */
void Rover::read_rangefinders(void)
{
    // Poll all configured rangefinder hardware for new distance measurements
    // Updates sensor state with latest readings and health status
    // Data automatically available to object avoidance and terrain following systems
    rangefinder.update();
    
#if HAL_LOGGING_ENABLED
    // Log rangefinder depth/distance data for post-flight analysis
    // Critical for reviewing terrain following and obstacle detection performance
    Log_Write_Depth();
#endif
}
#endif
