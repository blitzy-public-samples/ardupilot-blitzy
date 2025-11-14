/**
 * @file precision_landing.cpp
 * @brief Implementation of rover precision landing/docking using IR beacons or visual markers
 * 
 * @details This file provides rover-specific integration with the AC_PrecLand library
 *          for precision landing and docking operations. Precision landing enables
 *          autonomous docking with charging stations, parking markers, or other
 *          designated targets identified by IR beacons or visual markers.
 *          
 *          The precision landing system uses the AC_PrecLand library (shared with
 *          ArduCopter) which interfaces with:
 *          - IR-LOCK sensors for infrared beacon tracking
 *          - Companion computer vision systems for visual marker detection
 *          - Other supported precision landing backends
 *          
 *          For rovers, precision landing is primarily used for:
 *          - Autonomous docking with charging stations
 *          - Precise parking at designated locations
 *          - Target approach and alignment operations
 *          
 * @note This functionality is conditionally compiled based on AC_PRECLAND_ENABLED.
 *       When disabled, these functions are not included in the build, reducing
 *       memory footprint for applications that don't require precision landing.
 *       
 * @note The altitude parameter in update_precland() is not used for ground vehicles
 *       since rovers operate on a 2D plane. This differs from multicopter usage
 *       where altitude is critical for landing approach.
 *       
 * @see libraries/AC_PrecLand/ for the shared precision landing library implementation
 * @see Rover.h for the Rover class definition and precland member
 * 
 * Source: Rover/precision_landing.cpp
 */

#include "Rover.h"

#if AC_PRECLAND_ENABLED

/**
 * @brief Initialize the precision landing subsystem for rover operations
 * 
 * @details Initializes the AC_PrecLand library with appropriate update rate for
 *          ground vehicle operations. The precision landing system requires periodic
 *          updates to process target detection data from sensors (IR-LOCK, companion
 *          computer vision, etc.) and maintain target tracking state.
 *          
 *          Update Rate Selection:
 *          The scheduler table specifies a desired rate of 400Hz for precision landing
 *          updates, but the actual rate is limited by the main scheduler loop rate to
 *          avoid impossible timing requirements. The effective rate is the minimum of:
 *          - 400Hz (desired precision landing update rate)
 *          - Scheduler loop rate (actual achievable rate)
 *          
 *          For most rover configurations, the scheduler runs at 50Hz, so the precision
 *          landing system will update at 50Hz. This is sufficient for ground vehicle
 *          docking operations which involve slower approach speeds than aerial vehicles.
 *          
 * @note This function is called during Rover initialization (setup phase) to prepare
 *       the precision landing subsystem for use during autonomous docking operations.
 *       
 * @note The update rate affects target tracking responsiveness and latency. Higher
 *       rates provide smoother tracking but consume more CPU time.
 *       
 * @see Rover::update_precland() for the periodic update function
 * @see libraries/AC_PrecLand/AC_PrecLand.cpp for library initialization details
 */
void Rover::init_precland()
{
    // Scheduler table specifies 400Hz, but we can call it no faster
    // than the scheduler loop rate. Use the minimum of the two to ensure
    // we don't request impossible update rates.
    rover.precland.init(MIN(400, scheduler.get_loop_rate_hz()));
}

/**
 * @brief Periodic update of precision landing target tracking and state estimation
 * 
 * @details Called at scheduler loop rate to update the precision landing subsystem.
 *          This function processes new sensor data from precision landing backends
 *          (IR-LOCK, companion computer vision, etc.), updates target position estimates,
 *          and maintains target tracking state for autonomous docking operations.
 *          
 *          Rover-Specific Behavior:
 *          Unlike multicopters which use altitude information for descent control,
 *          rovers operate on a 2D plane and do not require altitude data. The altitude
 *          parameter is passed as 0 with the altitude-use flag set to false, instructing
 *          the AC_PrecLand library to ignore altitude in its calculations.
 *          
 *          Target Tracking:
 *          The precision landing library performs:
 *          - Target detection from sensor backends
 *          - Position filtering and noise reduction
 *          - Target lost/acquired state management
 *          - Velocity estimation for moving targets (if applicable)
 *          
 *          The updated target information is then available to guidance modes
 *          (such as Guided or Auto) for steering and throttle control to approach
 *          and center on the target.
 *          
 * @note This function should be called periodically at the scheduler loop rate
 *       (typically 50Hz for rovers) to maintain responsive target tracking.
 *       
 * @note The altitude parameter (0) and altitude-use flag (false) are rover-specific.
 *       Multicopters pass true altitude and enable altitude processing for landing.
 *       
 * @note Target position data is available through the precland object after this
 *       update completes and can be queried by navigation and guidance code.
 *       
 * @warning Precision landing requires an active sensor backend (IR-LOCK or companion
 *          computer) to be configured and operational. Without a working backend,
 *          the system will not detect targets and docking operations will fail.
 *       
 * @see Rover::init_precland() for initialization
 * @see libraries/AC_PrecLand/AC_PrecLand.cpp for update implementation details
 * @see ModeGuided for example usage of precision landing target data
 */
void Rover::update_precland()
{
    // Altitude parameter is unused for ground vehicles (passed as 0).
    // Second parameter (false) indicates altitude should not be used in calculations.
    // Rovers operate on a 2D plane and do not require altitude information for docking,
    // unlike multicopters which use altitude for descent control during landing.
    return precland.update(0, false);
}
#endif
