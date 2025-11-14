/**
 * @file sensors.cpp
 * @brief Sensor update functions for antenna tracker
 * 
 * @details Handles periodic updates of AHRS, compass, GPS, and battery monitoring.
 *          These functions are called by the scheduler at configured rates to maintain
 *          tracker's own attitude and position awareness.
 */

#include "Tracker.h"

/**
 * @brief Updates AHRS (Attitude and Heading Reference System)
 * 
 * @details Called by scheduler at configured rate (typically 50Hz). Delegates to 
 *          AP_AHRS::update() which:
 *          - Integrates IMU measurements (gyro, accel)
 *          - Updates attitude estimate (roll, pitch, yaw) using selected backend (DCM or EKF)
 *          - Fuses sensor data for optimal attitude solution
 * 
 * @note AHRS provides tracker's own attitude, not vehicle attitude
 * @see AP_AHRS::update()
 */
void Tracker::update_ahrs()
{
    ahrs.update();
}

/**
 * @brief Reads and updates compass (magnetometer)
 * 
 * @details Called by scheduler at configured rate. Reads magnetometer hardware
 *          and updates heading measurement used by AHRS.
 * 
 * @note Compass used to determine tracker's own heading orientation
 * @see AP_Compass::read()
 */
void Tracker::update_compass(void)
{
    compass.read();
}

/**
 * @brief Reads GPS and performs initial ground start sequence
 * 
 * @details Called by scheduler at configured rate. Updates GPS state and implements
 *          startup logic:
 *          - Updates GPS driver to read new messages
 *          - Waits for 5 consecutive 3D fixes during ground start
 *          - Sets home location to GPS position once countdown complete
 *          - Ensures accurate initial altitude by requiring multiple fixes
 * 
 * @note Ground start countdown ensures accurate initial altitude before setting home
 * @note Home location determines altitude reference for tracking calculations
 * @warning Requires stable GPS with good altitude accuracy before setting home
 * @see AP_GPS::update(), set_home()
 */
void Tracker::update_GPS(void)
{
    gps.update();

    // Static variables maintain ground-start countdown state across calls
    static uint32_t last_gps_msg_ms;
    static uint8_t ground_start_count = 5;
    
    // Detect new GPS message with valid 3D fix
    if (gps.last_message_time_ms() != last_gps_msg_ms && 
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();
        
        // Count down from 5 to ensure altitude stability
        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            
            // Validate that home hasn't been set yet (lat/lng both zero)
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else {
                // Now have an initial GPS position
                // use it as the HOME position in future startups
                
                // Set home to GPS location and complete ground start
                current_loc = gps.location();
                IGNORE_RETURN(set_home(current_loc, false));
                ground_start_count = 0;
            }
        }
    }
}

/**
 * @brief Battery failsafe handler (currently not implemented)
 * 
 * @details Called when battery failsafe triggered. Currently NOP (no operation).
 * 
 * @param[in] type_str Battery failsafe type description string
 * @param[in] action Failsafe action code
 * 
 * @todo Future enhancement: recall tracked vehicle before tracker loses power
 * @note Placeholder for future power-loss handling functionality
 */
void Tracker::handle_battery_failsafe(const char* type_str, const int8_t action)
{
    // NOP
    // useful failsafes in the future would include actually recalling the vehicle
    // that is tracked before the tracker loses power to continue tracking it
}
