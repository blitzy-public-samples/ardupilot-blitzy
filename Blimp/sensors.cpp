/**
 * @file sensors.cpp
 * @brief Sensor integration and reading functionality for Blimp vehicle
 * 
 * @details This file implements sensor reading and integration for the Blimp (lighter-than-air) vehicle type.
 *          Currently handles barometric altitude sensing with unit conversion from meters to centimeters
 *          for internal altitude representation. The barometer provides altitude estimation which is
 *          critical for blimp altitude control and buoyancy management.
 *          
 *          Key responsibilities:
 *          - Read and update barometric pressure sensor data
 *          - Convert barometer altitude readings from meters (AP_Baro output) to centimeters (internal format)
 *          - Store altitude in baro_alt member variable for use by control loops
 *          
 *          Coordinate Frame: Altitude is measured in NED (North-East-Down) frame where positive altitude
 *          is above the reference datum (typically sea level or takeoff altitude).
 *          
 *          Blimp-Specific Considerations:
 *          - Blimps operate at low speeds and rely heavily on barometric altitude for vertical control
 *          - Accurate altitude sensing is essential for buoyancy compensation
 *          - Unlike multirotors, blimps have slower altitude response requiring smooth sensor updates
 * 
 * @note This file is part of the Blimp vehicle-specific implementation
 * @see AP_Baro for barometer sensor abstraction
 * @see Blimp.h for main Blimp class definition
 * 
 * Source: Blimp/sensors.cpp
 */

#include "Blimp.h"

/**
 * @brief Read barometric pressure sensor and update altitude estimate
 * 
 * @details Triggers an update of the barometric pressure sensor and retrieves the current
 *          altitude estimate. The altitude is converted from meters (standard AP_Baro output)
 *          to centimeters (ArduPilot internal altitude representation) for consistency with
 *          other altitude and position values used throughout the flight control system.
 *          
 *          Algorithm:
 *          1. Call barometer.update() to read latest sensor data and update altitude calculation
 *          2. Retrieve altitude in meters using get_altitude()
 *          3. Convert to centimeters by multiplying by 100.0f
 *          4. Store result in baro_alt member variable
 *          
 *          The barometer update includes:
 *          - Reading raw pressure sensor data
 *          - Temperature compensation
 *          - Altitude calculation from pressure using atmospheric model
 *          - Multi-barometer sensor fusion if multiple barometers are available
 *          
 *          Unit Conversion Details:
 *          - Input: meters (float from AP_Baro)
 *          - Output: centimeters (float stored in baro_alt)
 *          - Conversion factor: 100.0f (1 meter = 100 centimeters)
 *          
 *          Update Frequency:
 *          This function is typically called at the main loop rate from the scheduler,
 *          which for Blimp is generally synchronized with the IMU update rate.
 *          The actual barometer sensor update rate may be slower (typically 20-50 Hz)
 *          but calling update() at higher rates is safe as the sensor driver handles
 *          rate limiting internally.
 *          
 *          Blimp-Specific Usage:
 *          - Altitude data is used by altitude hold modes for vertical position control
 *          - Critical for buoyancy management in lighter-than-air vehicles
 *          - Provides primary altitude reference as GPS altitude can be less accurate
 *          - Slower altitude changes in blimps allow for simple barometer filtering
 *          
 *          Health Monitoring:
 *          Sensor health is monitored by the AP_Baro library. If barometer becomes unhealthy,
 *          the EKF and altitude controller will be notified through the sensor health system.
 *          The vehicle may trigger failsafe actions if altitude sensing is lost.
 * 
 * @return void (altitude stored in member variable baro_alt)
 * 
 * @note Called from main scheduler loop at regular intervals
 * @note Altitude is in centimeters to maintain consistency with position controller units
 * @note The barometer object is a reference to the singleton AP_Baro instance
 * @warning Altitude estimate accuracy depends on barometer calibration and stable temperature
 * @warning In rapidly changing pressure conditions, altitude readings may lag actual altitude
 * 
 * @see AP_Baro::update() for sensor update implementation
 * @see AP_Baro::get_altitude() for altitude calculation details
 * @see Blimp::baro_alt member variable where result is stored
 * 
 * Source: Blimp/sensors.cpp:4-9
 */
void Blimp::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}
