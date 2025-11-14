/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file Copter.cpp
 * @brief Main implementation file for the ArduCopter flight controller
 * 
 * @details This file contains the core implementation of the Copter class, which serves as
 *          the main flight controller for multicopter vehicles. It implements the primary
 *          control loops, scheduler task definitions, and the overall execution architecture
 *          for the ArduCopter autopilot system.
 * 
 *          Key responsibilities include:
 *          - Scheduler task table definition (scheduler_tasks array)
 *          - Main control loops at various rates (fast loop, rc_loop, throttle_loop, etc.)
 *          - Sensor update coordination (IMU, compass, barometer, GPS)
 *          - Flight mode execution through the flightmode state machine
 *          - Logging coordination at multiple rates
 *          - Singleton instance initialization
 * 
 *          Architecture Overview:
 *          The Copter class uses a multi-rate scheduler (AP_Scheduler) to execute tasks
 *          at different frequencies. The fastest tasks run in the "fast loop" (typically 400Hz),
 *          while slower tasks are scheduled at rates appropriate for their function (e.g.,
 *          GPS updates at 50Hz, battery checks at 10Hz, housekeeping at 1Hz).
 * 
 *          Main Loop Structure:
 *          - Fast Loop (400Hz): IMU update, rate controllers, motor output, AHRS update,
 *            attitude control, flight mode update
 *          - RC Loop (250Hz): Radio input reading and mode switch processing
 *          - Throttle Loop (50Hz): Throttle mixing, auto-arming checks, ground effect
 *          - Various slower loops for sensors, logging, and housekeeping
 * 
 *          Singleton Pattern:
 *          The Copter class is instantiated as a global singleton (copter) which is
 *          accessible throughout the codebase via the AP:: namespace accessor pattern.
 * 
 * @note This file is safety-critical - all scheduler tasks must maintain their timing
 *       budgets to ensure stable flight control. Any modifications to task rates or
 *       execution times require careful testing.
 * 
 * @warning Do not add computationally expensive operations to fast loop tasks without
 *          profiling and ensuring timing constraints are met.
 */

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Andy Piper          :Harmonic notch, In-flight FFT, Bi-directional DShot, various drivers
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel         :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland :PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

/**
 * @file Copter.cpp
 * @brief Main implementation file for ArduCopter multicopter flight controller
 * 
 * @details This file contains the core execution architecture and main loop implementation
 *          for ArduCopter. It defines the task scheduler configuration, loop functions at
 *          various rates, and the singleton Copter object that coordinates all subsystems.
 * 
 * EXECUTION ARCHITECTURE OVERVIEW:
 * ================================
 * 
 * ArduCopter uses a cooperative multitasking scheduler (AP_Scheduler) that executes tasks
 * at specified rates with priority ordering. The architecture consists of:
 * 
 * 1. FAST LOOP (400Hz default - LOOP_RATE define)
 *    Highest priority tasks for flight-critical control:
 *    - Update IMU (gyro/accel sensor reading)
 *    - Run rate controller (innermost PID loop, converts rate targets to motor commands)
 *    - Output to motors (PWM/DShot commands sent to ESCs)
 *    - Read AHRS/EKF (attitude and position estimation from sensor fusion)
 *    - Update flight mode (mode-specific control logic)
 *    - Land/crash detection (critical safety monitoring)
 *    
 *    These tasks run every 2.5ms (at 400Hz) and must complete within their time budget
 *    to maintain stable flight control. Total fast loop budget is typically <1.5ms.
 * 
 * 2. SCHEDULED TASKS (various rates: 250Hz down to 0.1Hz)
 *    Lower priority periodic tasks defined in scheduler_tasks array:
 *    - rc_loop() at 250Hz: Read pilot input from RC receiver
 *    - throttle_loop() at 50Hz: Altitude hold and climb rate control
 *    - update_batt_compass() at 10Hz: Sensor updates with motor interference compensation
 *    - GPS, optical flow, rangefinder updates at sensor-specific rates
 *    - GCS telemetry send/receive at 400Hz (high bandwidth communication)
 *    - Logging at multiple rates (fast attitude, medium navigation, slow status)
 *    - Failsafe checks at 10-25Hz (battery, GPS, GCS, EKF monitoring)
 *    - Housekeeping tasks at 1-3Hz (configuration updates, status reporting)
 * 
 * 3. TASK PRIORITY SYSTEM
 *    Each task has a priority number (0-255, lower = higher priority). The scheduler
 *    executes tasks in priority order within each time slice. If a higher priority task
 *    overruns its time budget, lower priority tasks may be delayed to next cycle.
 *    
 *    Critical tasks (fast loop, rate controller) have lowest priority numbers.
 *    Non-critical tasks (logging, telemetry) have higher priority numbers.
 * 
 * 4. MAIN CONTROL FLOW
 *    ArduCopter/Copter.h defines the main vehicle class
 *    ArduCopter/system.cpp contains setup() and loop() entry points
 *    This file (Copter.cpp) defines:
 *    - scheduler_tasks array: Complete task list with rates and priorities
 *    - get_scheduler_tasks(): Provides task array to AP_Scheduler
 *    - Loop functions: rc_loop, throttle_loop, update_batt_compass, etc.
 *    - Constructor: Initializes Copter singleton with default values
 * 
 * 5. SINGLETON PATTERN
 *    Global singleton `copter` object (line 981) provides access to vehicle state
 *    from anywhere in the codebase. This is the traditional ArduPilot architecture
 *    pattern that allows libraries to access vehicle-specific functionality.
 * 
 * 6. COORDINATE SYSTEMS AND UNITS
 *    - NED frame: North-East-Down earth-fixed reference (navigation, waypoints)
 *    - Body frame: Roll-Pitch-Yaw relative to vehicle orientation (attitude control)
 *    - Angles: Typically in centidegrees (0.01 degrees) or radians
 *    - Distances: Typically in centimeters or meters (explicitly specified)
 *    - Rates: Typically in deg/s or rad/s (explicitly specified)
 * 
 * 7. FLIGHT MODE ARCHITECTURE
 *    Flight modes (Stabilize, AltHold, Loiter, Auto, etc.) are implemented as separate
 *    classes derived from Mode base class. The current mode is accessed via `flightmode`
 *    pointer. Each mode implements update() called from fast loop and provides mode-
 *    specific control logic (attitude targets, position targets, autonomous navigation).
 * 
 * 8. SENSOR FUSION AND STATE ESTIMATION
 *    AP_AHRS integrates IMU, GPS, compass, barometer data through Extended Kalman Filter
 *    (EKF3) to provide attitude, position, velocity estimates. EKF runs in fast loop
 *    and provides the state estimates used by all control loops.
 * 
 * 9. CONTROL LOOP HIERARCHY (nested loop structure)
 *    Outer → Inner control loops with increasing rates:
 *    - Position controller (10-50Hz): GPS waypoint → velocity target
 *    - Velocity controller (50-100Hz): Velocity target → attitude target  
 *    - Attitude controller (400Hz): Attitude target → rate target
 *    - Rate controller (400Hz): Rate target → motor output
 *    
 *    Each loop feeds its output as the setpoint to the next inner loop.
 * 
 * 10. SAFETY ARCHITECTURE
 *     Multiple layers of safety monitoring:
 *     - Pre-arm checks: Comprehensive system health validation before arming
 *     - Arming checks: Final safety validation before enabling motors
 *     - In-flight failsafes: Battery, GPS, GCS, EKF, vibration monitoring
 *     - Crash detection: Monitors for vehicle crash and disarms automatically
 *     - Land detection: Detects touchdown and adjusts control accordingly
 *     - Geofence: Enforces altitude and position boundaries
 * 
 * For detailed documentation on specific subsystems, see:
 * - Mode implementations: ArduCopter/mode.h and mode_*.cpp files
 * - Attitude control: libraries/AC_AttitudeControl/
 * - Position control: libraries/AC_PosControl/
 * - Motor mixing: libraries/AP_Motors/
 * - Navigation: libraries/AC_WPNav/
 * - State estimation: libraries/AP_AHRS/ and libraries/AP_NavEKF3/
 * 
 * @see AP_Scheduler for task scheduling implementation
 * @see Copter::scheduler_tasks for complete task list with rates and priorities
 * @see ArduCopter/system.cpp for setup() and loop() entry points
 * @see Mode class hierarchy for flight mode implementations
 */

#include "Copter.h"
#include <AP_InertialSensor/AP_InertialSensor_rate_config.h>

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)

/**
 * @brief Scheduler task table defining all periodic functions and their execution rates
 * 
 * @details This array defines the complete set of tasks that the ArduCopter scheduler
 *          executes at various rates. Tasks are organized by priority, with the most
 *          critical (fast loop) tasks executed first. The scheduler ensures each task
 *          runs at its specified rate while maintaining overall timing constraints.
 * 
 *          Task Execution Architecture:
 *          - FAST_TASK: Runs every main loop iteration (typically 400Hz)
 *          - SCHED_TASK: Runs at specified rate (rate_hz parameter)
 *          - Priority determines execution order within each loop iteration
 *          - Tasks must complete within their allocated time budget (max_time_micros)
 * 
 *          Task Categories:
 *          1. Fast Loop Tasks (FAST_TASK): IMU update, rate controllers, motor output,
 *             AHRS, attitude control, flight mode execution - all at loop rate (400Hz)
 *          2. RC Processing: rc_loop at 250Hz for pilot input responsiveness
 *          3. Control Updates: throttle_loop at 50Hz, nav updates at 50Hz
 *          4. Sensor Updates: GPS 50Hz, optical flow 200Hz, rangefinder 20Hz, etc.
 *          5. Communication: GCS MAVLink at 400Hz, telemetry streaming
 *          6. Safety Systems: Fence checks, failsafe monitoring, arming checks
 *          7. Logging: Multiple rates (400Hz, 25Hz, 10Hz) for different data sets
 *          8. Housekeeping: Battery/compass 10Hz, parameter updates, slow diagnostics
 * 
 *          Priority Ordering:
 *          All entries MUST be ordered by priority (lower number = higher priority).
 *          This table is interleaved with AP_Vehicle base class tasks to determine
 *          final execution order. Violating priority ordering can cause scheduler
 *          malfunction and unstable flight.
 * 
 *          Timing Constraints:
 *          Each task has an expected execution time (max_time_micros). The scheduler
 *          monitors actual execution time and logs overruns. Sum of all task times
 *          must leave sufficient margin for jitter and unexpected delays.
 * 
 * @note Adding new tasks requires careful analysis of CPU budget and timing impact.
 *       Test thoroughly in SITL and on hardware with scheduler timing logging enabled.
 * 
 * @warning Modifying task rates or priorities can destabilize flight. The fast loop
 *          tasks are especially critical - any changes must maintain 400Hz execution.
 *          Never add blocking operations or long-running computations to fast tasks.
 * 
 * SCHED_TASK macro arguments:
 *  - func: Name of Copter member function to call
 *  - rate_hz: Frequency in Hertz at which function should be called
 *  - max_time_micros: Expected maximum execution time in microseconds
 *  - priority: Priority level (0-255, lower number = higher priority)
 * 
 * SCHED_TASK_CLASS macro arguments:
 *  - class: Class name of the object containing the method
 *  - instance: Pointer to the instance on which to call the method
 *  - method: Name of the method to call
 *  - rate_hz: Frequency in Hertz at which method should be called
 *  - max_time_micros: Expected maximum execution time in microseconds
 *  - priority: Priority level (0-255, lower number = higher priority)
 * 
 * FAST_TASK macro arguments:
 *  - func: Name of Copter member function to call every loop iteration
 * 
 * FAST_TASK_CLASS macro arguments:
 *  - class: Class name of the object containing the method
 *  - instance: Pointer to the instance
 *  - method: Name of the method to call every loop iteration
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // ========================================================================
    // FAST LOOP TASKS - Execute every main loop iteration (typically 400Hz)
    // These are the most time-critical tasks for stable flight control
    // ========================================================================
    
    // Update INS (Inertial Navigation System) immediately to get fresh gyro/accel data
    // This MUST be first to ensure all subsequent tasks have current IMU measurements
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    
    // Run low-level rate controllers (angular velocity control) using fresh IMU data
    // Converts desired body rates to motor commands via PID controllers
    FAST_TASK(run_rate_controller_main),
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    // Execute custom controller backend if enabled (experimental feature)
    FAST_TASK(run_custom_controller),
#endif
#if FRAME_CONFIG == HELI_FRAME
    // Update helicopter autorotation state machine for engine failure handling
    FAST_TASK(heli_update_autorotation),
#endif //HELI_FRAME
    
    // Send motor commands to ESCs/servos - time-critical for smooth control
    // Must occur after rate controller to minimize latency
    FAST_TASK(motors_output_main),
    
    // Run Extended Kalman Filter (EKF) state estimator - computationally expensive
    // Fuses IMU, GPS, baro, compass to estimate attitude, position, velocity
    // Note: We skip INS update in AHRS since we already did it above
    FAST_TASK(read_AHRS),
#if FRAME_CONFIG == HELI_FRAME
    // Update helicopter-specific control dynamics (swashplate mixing, rotor speed)
    FAST_TASK(update_heli_control_dynamics),
#endif //HELI_FRAME
    
    // Update Inertial Navigation position/velocity estimates from EKF
    // Provides position and velocity in NED frame for navigation controllers
    FAST_TASK(read_inertia),
    
    // Check if EKF has reset target heading or position (due to GPS glitch, etc.)
    // Adjusts navigation targets to prevent sudden jumps in control commands
    FAST_TASK(check_ekf_reset),
    
    // Execute current flight mode's control algorithm (Stabilize, Loiter, Auto, etc.)
    // This is where high-level navigation commands are converted to attitude targets
    FAST_TASK(update_flight_mode),
    
    // Update home position from EKF if it has better estimate (e.g., after GPS lock improves)
    FAST_TASK(update_home_from_EKF),
    
    // Detect landing and crash conditions for automatic disarm and failsafe triggers
    // Monitors vertical acceleration, climb rate, and motor output
    FAST_TASK(update_land_and_crash_detectors),
    
    // Update rangefinder-based terrain following offset for precision altitude control
    FAST_TASK(update_rangefinder_terrain_offset),
#if HAL_MOUNT_ENABLED
    // Fast update for camera gimbal mount stabilization (requires loop-rate updates)
    FAST_TASK_CLASS(AP_Mount, &copter.camera_mount, update_fast),
#endif
#if HAL_LOGGING_ENABLED
    // Log video stabilization data for post-processing
    FAST_TASK(Log_Video_Stabilisation),
#endif

    // ========================================================================
    // RC INPUT PROCESSING - 250Hz for responsive pilot control
    // ========================================================================
    
    // Read RC receiver input and process mode switch
    // High rate ensures minimal latency between pilot stick movement and response
    SCHED_TASK(rc_loop,              250,    130,  3),
    
    // ========================================================================
    // CONTROL UPDATES - 50Hz for navigation and throttle management
    // ========================================================================
    
    // Update throttle mixing, auto-arming, rotor speed, ground effect compensation
    // 50Hz is sufficient for these slower-changing control aspects
    SCHED_TASK(throttle_loop,         50,     75,  6),
    
    // ========================================================================
    // SAFETY SYSTEMS - Periodic checks for geofencing and failsafes
    // ========================================================================
    
#if AP_FENCE_ENABLED
    // Check if vehicle is outside geofence boundaries and trigger breach actions
    SCHED_TASK(fence_check,           25,    100,  7),
#endif

    // ========================================================================
    // SENSOR UPDATES - Various rates based on sensor capabilities
    // ========================================================================
    
    // Update GPS position/velocity (50Hz typical for modern GPS modules)
    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),
#if AP_OPTICALFLOW_ENABLED
    // Update optical flow sensor for velocity estimation (200Hz for high-speed tracking)
    SCHED_TASK_CLASS(AP_OpticalFlow,          &copter.optflow,             update,         200, 160,  12),
#endif
    
    // Read battery voltage/current and compass (10Hz sufficient for these slow-changing values)
    // Battery voltage may be used for compass motor interference compensation
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    
    // Read auxiliary RC channels (mode switches, camera triggers, etc.)
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),
#if TOY_MODE_ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50,  24),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50,  27),
#if AP_COPTER_AHRS_AUTO_TRIM_ENABLED
    SCHED_TASK_CLASS(RC_Channels_Copter,   &copter.g2.rc_channels,      auto_trim_run,   10,  75,  30),
#endif
#if AP_RANGEFINDER_ENABLED
    // Read rangefinder/lidar distance sensors for terrain following
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif
#if HAL_PROXIMITY_ENABLED
    // Update proximity sensors for obstacle avoidance (high rate for rapid detection)
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50,  36),
#endif
#if AP_BEACON_ENABLED
    // Update beacon positioning system for GPS-denied navigation
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50,  39),
#endif
    
    // ========================================================================
    // NAVIGATION AND ALTITUDE CONTROL
    // ========================================================================
    
    // Read barometer and log control tuning data
    SCHED_TASK(update_altitude,       10,    100,  42),
    
    // Update position and velocity controllers for waypoint navigation
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    
    // Update hover throttle estimate for more accurate altitude hold
    SCHED_TASK(update_throttle_hover,100,     90,  48),
#if MODE_SMARTRTL_ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL,         &copter.mode_smartrtl,       save_position,    3, 100,  51),
#endif
#if HAL_SPRAYER_ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,               update,         3,  90,  54),
#endif
    // ========================================================================
    // PERIODIC HOUSEKEEPING AND SLOW LOOPS
    // ========================================================================
    
    // 3Hz loop: GCS failsafe, terrain failsafe, deadreckoning check, tuning
    SCHED_TASK(three_hz_loop,          3,     75, 57),
    
#if AP_SERVORELAYEVENTS_ENABLED
    // Process servo and relay events (camera triggers, etc.)
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
#endif
#if AC_PRECLAND_ENABLED
    // Update precision landing target tracking (high rate for accurate landing)
    SCHED_TASK(update_precland,      400,     50,  69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    // Check if helicopter is in dynamic flight for collective management
    SCHED_TASK(check_dynamic_flight,  50,     75,  72),
#endif

    // ========================================================================
    // LOGGING - Multiple rates for different data priorities
    // ========================================================================
    
#if HAL_LOGGING_ENABLED
    // Log attitude, rate, and PID data at main loop rate (if enabled)
    SCHED_TASK(loop_rate_logging, LOOP_RATE,    50,  75),
#endif
    
    // 1Hz loop: Slow housekeeping, terrain logging, motor setup when disarmed
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    // ========================================================================
    // SAFETY MONITORING - Continuous health checks
    // ========================================================================
    
    // Monitor EKF health and trigger failsafe if variance exceeds limits
    SCHED_TASK(ekf_check,             10,     75,  84),
    
    // Check vibration levels and warn if excessive (can degrade sensor performance)
    SCHED_TASK(check_vibration,       10,     50,  87),
    
    // Detect GPS glitches and position jumps
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
    
    // Monitor takeoff progress and detect failures
    SCHED_TASK(takeoff_check,         50,     50,  91),
    
#if AP_LANDINGGEAR_ENABLED
    // Retract/deploy landing gear based on altitude and flight mode
    SCHED_TASK(landinggear_update,    10,     75,  93),
#endif
    
    // Update standby mode processing
    SCHED_TASK(standby_update,        100,    75,  96),
    
    // Check for lost vehicle beeper activation
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    
    // ========================================================================
    // GROUND CONTROL STATION COMMUNICATION - MAVLink protocol
    // ========================================================================
    
    // Receive and parse MAVLink messages from ground station (high rate for low latency)
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    
    // Send telemetry streams to ground station (high rate for smooth data flow)
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
    // ========================================================================
    // CAMERA AND GIMBAL CONTROL
    // ========================================================================
    
#if HAL_MOUNT_ENABLED
    // Update camera gimbal mount pointing and stabilization (slower rate for smooth updates)
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
#endif
#if AP_CAMERA_ENABLED
    // Process camera triggers and control commands
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
#endif

    // ========================================================================
    // ADDITIONAL LOGGING TASKS - Different rates for different data sets
    // ========================================================================
    
#if HAL_LOGGING_ENABLED
    // 10Hz logging: AHRS, attitude targets, motors, RC, position, sensors
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    
    // 25Hz logging: EKF position, IMU, gyro FFT analysis
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    
    // Logger periodic maintenance: flush buffers, manage storage
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif
    
    // ========================================================================
    // SYSTEM MAINTENANCE AND PERIODIC UPDATES
    // ========================================================================
    
    // Periodic IMU tasks: calibration updates, health monitoring
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

#if HAL_LOGGING_ENABLED
    // Log scheduler performance metrics (very slow rate, just for diagnostics)
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#endif
#if AP_RPM_ENABLED
    // Read RPM sensors for rotor/engine speed monitoring
    SCHED_TASK_CLASS(AP_RPM,               &copter.rpm_sensor,          update,          40, 200, 129),
#endif
#if AP_TEMPCALIBRATION_ENABLED
    // Apply temperature calibration to IMU sensors
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#endif
#if HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
    // Update ADSB collision avoidance system
    SCHED_TASK(avoidance_adsb_update, 10,    100, 138),
#endif  // HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Check advanced failsafe system (for commercial operators)
    SCHED_TASK(afs_fs_check,          10,    100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    // Update terrain altitude database for terrain following
    SCHED_TASK(terrain_update,        10,    100, 144),
#endif
#if AP_WINCH_ENABLED
    // Update winch control (for cargo delivery, rescue operations)
    SCHED_TASK_CLASS(AP_Winch,             &copter.g2.winch,            update,          50,  50, 150),
#endif
    // ========================================================================
    // USER HOOKS - Custom user code integration points (optional)
    // ========================================================================
    
#ifdef USERHOOK_FASTLOOP
    // User-defined fast loop hook (100Hz)
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    // User-defined 50Hz loop hook
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    // User-defined medium rate loop hook (10Hz)
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    // User-defined slow loop hook (3.3Hz)
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    // User-defined very slow loop hook (1Hz)
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif

    // ========================================================================
    // HARDWARE INTERFACE
    // ========================================================================
    
#if HAL_BUTTON_ENABLED
    // Read button inputs for user interaction
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // Update dynamic notch filter with high-rate gyro data
    // Note: There is an equivalent virtual task in AP_Vehicle for non-rate-loop case
    SCHED_TASK(update_dynamic_notch_at_specified_rate_main,                       LOOP_RATE, 200, 215),
#endif
};

/**
 * @brief Retrieve the scheduler task table for the AP_Scheduler
 * 
 * @details This function provides the scheduler with access to the Copter-specific
 *          task table. The AP_Scheduler base class calls this during initialization
 *          to merge vehicle-specific tasks with common AP_Vehicle tasks, creating
 *          the final execution schedule.
 * 
 *          The returned task table (scheduler_tasks) contains all periodic functions
 *          that need to execute at various rates to maintain stable flight control,
 *          sensor updates, communication, logging, and safety monitoring.
 * 
 *          Task Execution Model:
 *          The AP_Scheduler interleaves tasks from this table with tasks from
 *          AP_Vehicle::scheduler_tasks, ordering them by priority. It tracks
 *          execution time for each task and logs overruns for performance analysis.
 * 
 * @param[out] tasks       Reference to pointer that will point to scheduler_tasks array
 * @param[out] task_count  Reference to variable that will receive number of tasks in array
 * @param[out] log_bit     Reference to variable that will receive the logging bitmask
 *                         for scheduler performance (MASK_LOG_PM for performance monitoring)
 * 
 * @note This function is called once during initialization by AP_Scheduler.
 *       The returned pointer remains valid for the lifetime of the program as it
 *       points to a static const array.
 */
void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];


#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
#if MODE_GUIDED_ENABLED
/**
 * @brief Set target GPS location for guided flight (scripting and external control API)
 * 
 * @details Commands the vehicle to fly to a specified GPS location using Guided mode
 *          position control. This is the primary interface for Lua scripts and companion
 *          computers to command autonomous waypoint navigation.
 * 
 *          Mode Requirements:
 *          Vehicle must be in Guided mode or a mode that accepts guided commands (Auto
 *          mode during NAV_GUIDED mission command). Function fails if not in appropriate
 *          mode, preventing unexpected vehicle behavior.
 * 
 *          Target Location:
 *          Location specifies:
 *          - Latitude/longitude: GPS coordinates in degrees
 *          - Altitude: Can be relative (to home), absolute (MSL), or terrain-following
 *          - Altitude frame: Specified in Location.alt_frame (RELATIVE, ABSOLUTE, TERRAIN)
 * 
 *          Navigation Behavior:
 *          Once target set, Guided mode:
 *          - Calculates bearing and distance to target
 *          - Plans smooth trajectory respecting speed limits (WPNAV_SPEED parameter)
 *          - Maintains altitude during horizontal transit
 *          - Arrives at target and holds position (loiter)
 *          - Does NOT automatically proceed to next waypoint (unlike Auto mode)
 * 
 *          Use Cases:
 *          - Lua scripting: Custom autonomous behaviors (search patterns, surveys, etc.)
 *          - Companion computers: Vision-based navigation, obstacle avoidance
 *          - External control: MAVLink partners commanding vehicle movements
 *          - Dynamic missions: Modify flight path based on real-time conditions
 * 
 *          Error Handling:
 *          Returns false if mode check fails. Caller should verify return value and
 *          handle failure appropriately (may need to change mode first).
 * 
 * @param[in] target_loc GPS location to fly to (latitude, longitude, altitude, frame)
 * 
 * @return true Target location accepted and vehicle will navigate to it
 * @return false Failed - not in appropriate mode for guided commands
 * 
 * @note Requires Guided mode active (or Auto mode executing NAV_GUIDED command)
 * @note Conditional compilation: Only available if scripting or external control enabled
 * @see start_takeoff() for initiating takeoff to altitude before navigation
 * @see set_target_pos_NED() for position commands in local NED frame
 * @see ModeGuided::set_destination() for underlying implementation
 */
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

/**
 * @brief Initiate takeoff to specified altitude (scripting and external control API)
 * 
 * @details Commands the vehicle to take off vertically to a target altitude using Guided
 *          mode takeoff logic. Automatically enables auto-arming to allow motors to spin
 *          during the takeoff sequence.
 * 
 *          Mode Requirements:
 *          Vehicle must be in Guided mode or a mode accepting guided commands. Fails if
 *          not in appropriate mode, preventing unexpected takeoff commands.
 * 
 *          Takeoff Sequence:
 *          1. Validates mode and altitude target
 *          2. Arms motors (auto-armed flag set to true)
 *          3. Spools up motors to takeoff throttle
 *          4. Climbs vertically at configured takeoff speed (PILOT_TKOFF_ALT parameter)
 *          5. Transitions to position hold when target altitude reached
 *          6. Remains in Guided mode awaiting next command
 * 
 *          Auto-Arming:
 *          Automatically sets auto_armed flag, allowing the vehicle to arm and takeoff
 *          without manual pilot intervention. This is essential for fully autonomous
 *          operation initiated by scripts or companion computers.
 * 
 *          Altitude Reference:
 *          Target altitude is relative to home position (takeoff location). For example,
 *          alt=10.0 means "climb to 10 meters above takeoff point."
 * 
 *          Unit Conversion:
 *          Input altitude in meters (standard for scripting API), converted to centimeters
 *          (×100) for internal ArduPilot position control which uses cm as standard unit.
 * 
 *          Safety Considerations:
 *          - Verify sufficient clearance above takeoff point
 *          - Ensure pre-arm checks have passed
 *          - Confirm GPS lock and EKF health
 *          - Consider geofence altitude limits
 * 
 *          Use Cases:
 *          - Lua scripts: Autonomous takeoff in custom mission scripts
 *          - Companion computers: Computer vision applications requiring flight
 *          - External control: MAVLink partners initiating autonomous operations
 * 
 * @param[in] alt Target altitude in meters above home (relative altitude)
 * 
 * @return true Takeoff initiated successfully, vehicle will climb to altitude
 * @return false Failed - not in appropriate mode or takeoff command rejected
 * 
 * @note Automatically sets auto_armed flag to enable motor arming
 * @note Input altitude in meters, converted to centimeters internally (×100)
 * @note Altitude is relative to home position (takeoff location)
 * @note Conditional compilation: Only available if scripting or external control enabled
 * @see set_target_location() for navigation after takeoff
 * @see ModeGuided::do_user_takeoff_start() for underlying takeoff implementation
 */
bool Copter::start_takeoff(const float alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start(alt * 100.0f)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}
#endif //MODE_GUIDED_ENABLED
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
#if MODE_GUIDED_ENABLED
/**
 * @brief Set target position in NED frame with optional yaw control (Lua scripting API)
 * 
 * @details Commands the vehicle to fly to a position specified in local North-East-Down
 *          (NED) coordinate frame, relative to the EKF origin (typically home position).
 *          Provides precise position control with optional independent yaw control.
 * 
 *          Coordinate Frame - NED (North-East-Down):
 *          - X-axis (North): Positive northward in meters
 *          - Y-axis (East): Positive eastward in meters
 *          - Z-axis (Down): Positive downward in meters (negative = altitude gain)
 * 
 *          Frame Conversion:
 *          Function converts NED input to internal NEU (North-East-Up) representation:
 *          - North and East unchanged (×100 for cm)
 *          - Down inverted to Up: -target_pos.z → altitude above origin
 * 
 *          Yaw Control Options:
 *          - use_yaw=false: Vehicle yaws toward direction of travel (default behavior)
 *          - use_yaw=true, use_yaw_rate=false: Face absolute heading (yaw_deg)
 *          - use_yaw=true, use_yaw_rate=true: Rotate at yaw_rate_degs while moving
 *          - yaw_relative=true: yaw_deg relative to current heading (not north)
 *          - yaw_relative=false: yaw_deg absolute (0=north, 90=east)
 * 
 *          Altitude Frame:
 *          - terrain_alt=false: Altitude relative to EKF origin (home)
 *          - terrain_alt=true: Altitude above terrain (requires rangefinder or terrain data)
 * 
 *          Navigation Behavior:
 *          Position controller calculates smooth trajectory to target position, respecting:
 *          - Maximum velocity limits (WPNAV_SPEED parameter)
 *          - Maximum acceleration (WPNAV_ACCEL parameter)
 *          - S-curve trajectory for smooth motion
 * 
 *          Use Cases:
 *          - Precision positioning: Vision-based landing, object approach
 *          - Local navigation: Relative movements without GPS coordinates
 *          - Terrain following: Maintain constant height above ground
 *          - Custom flight patterns: Lua scripts with precise position control
 * 
 * @param[in] target_pos Target position in NED frame (meters): {north, east, down}
 * @param[in] use_yaw True to specify yaw, false to yaw toward direction of travel
 * @param[in] yaw_deg Target yaw angle in degrees (0=north, 90=east, 180=south, 270=west)
 * @param[in] use_yaw_rate True to specify yaw rate instead of target angle
 * @param[in] yaw_rate_degs Yaw rotation rate in degrees per second (if use_yaw_rate=true)
 * @param[in] yaw_relative True if yaw_deg is relative to current heading, false if absolute
 * @param[in] terrain_alt True for altitude above terrain, false for altitude above origin
 * 
 * @return true Position command accepted, vehicle will navigate to target
 * @return false Failed - not in Guided mode
 * 
 * @note Input position in meters (NED), converted to centimeters (NEU) internally
 * @note Down coordinate is negated to Up: target_pos.z = -10 means 10m altitude gain
 * @note Yaw angles converted from degrees to radians internally
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_posvel_NED() for position+velocity commands
 * @see set_target_posvelaccel_NED() for full position+velocity+acceleration commands
 */
bool Copter::set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);

    return mode_guided.set_destination(pos_neu_cm, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), yaw_relative, terrain_alt);
}

/**
 * @brief Set target position and velocity in NED frame (Lua scripting API)
 * 
 * @details Commands the vehicle to track a moving target by specifying both desired
 *          position and velocity simultaneously. Position controller uses velocity
 *          feedforward for smooth tracking of moving targets.
 * 
 *          Coordinate Frame - NED (North-East-Down):
 *          Position and velocity both specified in NED frame relative to EKF origin:
 *          - X-axis (North): Positive northward
 *          - Y-axis (East): Positive eastward
 *          - Z-axis (Down): Positive downward (negative = altitude/climb rate)
 * 
 *          Frame Conversion:
 *          Both position and velocity converted from NED to internal NEU:
 *          - North and East: ×100 (meters → centimeters)
 *          - Down: Negated and ×100 (-meters → centimeters altitude)
 * 
 *          Velocity Feedforward:
 *          The velocity parameter enables feedforward control, improving tracking of
 *          moving targets. Position controller calculates:
 *          - Position error: current_pos - target_pos
 *          - Velocity command: target_vel + PID_correction(position_error)
 * 
 *          This approach reduces tracking lag compared to position-only control.
 * 
 *          Use Cases:
 *          - Moving target tracking: Follow moving objects (vehicles, people, etc.)
 *          - Smooth trajectory tracking: Execute pre-planned smooth paths
 *          - Velocity guidance: Combine position constraint with velocity directive
 *          - Landing on moving platforms: Track moving landing pad position+velocity
 * 
 *          Yaw Control:
 *          Yaw not independently controlled in this variant. Vehicle yaws toward
 *          velocity vector direction for intuitive heading during motion.
 * 
 *          Safety Consideration:
 *          Velocity feedforward can cause rapid movements if target velocity is large.
 *          Ensure velocity values are within vehicle capabilities (WPNAV_SPEED_DN/UP).
 * 
 * @param[in] target_pos Target position in NED frame (meters): {north, east, down}
 * @param[in] target_vel Target velocity in NED frame (m/s): {north_vel, east_vel, down_vel}
 * 
 * @return true Position and velocity command accepted, vehicle will track target
 * @return false Failed - not in Guided mode
 * 
 * @note Position in meters (NED), velocity in m/s (NED), both converted to cm/cm/s (NEU)
 * @note Down coordinate/velocity negated to Up representation internally
 * @note Acceleration assumed zero (uses default controller acceleration limits)
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_pos_NED() for position-only commands
 * @see set_target_posvelaccel_NED() for full position+velocity+acceleration
 * @see AC_PosControl for position controller implementation with feedforward
 */
bool Copter::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

/**
 * @brief Set target position, velocity, and acceleration in NED frame (Lua scripting API)
 * 
 * @details Commands complete trajectory tracking with position, velocity, and acceleration
 *          targets. Provides full feedforward control for aggressive trajectory tracking
 *          or implementation of custom control algorithms via scripting.
 * 
 *          Coordinate Frame - NED (North-East-Down):
 *          All three vectors specified in NED frame relative to EKF origin:
 *          - Position: Meters from origin {north, east, down}
 *          - Velocity: m/s in NED frame {north_vel, east_vel, down_vel}
 *          - Acceleration: m/s² in NED frame {north_accel, east_accel, down_accel}
 * 
 *          Frame Conversion:
 *          All vectors converted from NED to internal NEU representation:
 *          - North and East: ×100 (meters/m/s/m/s² → centimeters/cm/s/cm/s²)
 *          - Down: Negated and ×100 (down → up)
 * 
 *          Full Feedforward Control:
 *          Position controller uses complete feedforward:
 *          - Acceleration feedforward: Commanded directly to attitude controller
 *          - Velocity feedforward: Added to PID velocity command
 *          - Position target: Used for error calculation only
 * 
 *          Control Flow:
 *          1. Position error calculated: current_pos - target_pos
 *          2. Position PID generates velocity correction
 *          3. Total velocity: target_vel + PID_correction
 *          4. Velocity controller generates acceleration command
 *          5. Total acceleration: target_accel + velocity_correction
 *          6. Acceleration command sent to attitude controller
 * 
 *          This provides the most responsive and accurate trajectory tracking.
 * 
 *          Yaw Control Options:
 *          - use_yaw=false: Yaw toward velocity direction
 *          - use_yaw=true, use_yaw_rate=false: Face absolute heading (yaw_deg)
 *          - use_yaw=true, use_yaw_rate=true: Rotate at yaw_rate_degs
 *          - yaw_relative=true: yaw_deg relative to current heading
 * 
 *          Use Cases:
 *          - Aggressive trajectory tracking: Race course, aerobatic maneuvers
 *          - Custom controllers: Implement control algorithms in Lua with full authority
 *          - Model predictive control: Execute pre-computed optimal trajectories
 *          - Motion planning: Track outputs from motion planner with full dynamics
 * 
 *          Safety Considerations:
 *          - Acceleration feedforward bypasses safety limits - use with caution
 *          - Excessive acceleration commands can destabilize vehicle
 *          - Verify commanded values are within vehicle physical capabilities
 *          - Consider enabling crash detection for aggressive maneuvers
 * 
 * @param[in] target_pos Target position in NED frame (meters): {north, east, down}
 * @param[in] target_vel Target velocity in NED frame (m/s): {north_vel, east_vel, down_vel}
 * @param[in] target_accel Target acceleration in NED frame (m/s²): {north_accel, east_accel, down_accel}
 * @param[in] use_yaw True to specify yaw, false to yaw toward velocity direction
 * @param[in] yaw_deg Target yaw angle in degrees (0=north, 90=east, if use_yaw=true)
 * @param[in] use_yaw_rate True to specify yaw rate instead of target angle
 * @param[in] yaw_rate_degs Yaw rotation rate in degrees per second (if use_yaw_rate=true)
 * @param[in] yaw_relative True if yaw_deg is relative to current heading, false if absolute
 * 
 * @return true Complete trajectory command accepted, vehicle will track targets
 * @return false Failed - not in Guided mode
 * 
 * @warning Acceleration feedforward bypasses safety limits - use with extreme caution
 * @note All inputs converted: meters→cm, m/s→cm/s, m/s²→cm/s², degrees→radians, NED→NEU
 * @note Down coordinates/velocity/acceleration negated to Up representation
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_pos_NED() for position-only commands (safest)
 * @see set_target_posvel_NED() for position+velocity commands
 * @see AC_PosControl::set_pos_vel_accel_xy() for controller implementation
 */
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cms, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), yaw_relative);
}

/**
 * @brief Set target velocity in NED frame (Lua scripting API)
 * 
 * @details Commands the vehicle to maintain a constant velocity vector in North-East-Down
 *          frame. Vehicle will accelerate to the target velocity and maintain it until
 *          a new command is received. This provides velocity-based control without
 *          specifying explicit position targets.
 * 
 *          Coordinate Frame - NED (North-East-Down):
 *          Velocity specified in earth-fixed NED frame relative to ground:
 *          - X-axis (North): Positive northward velocity in m/s
 *          - Y-axis (East): Positive eastward velocity in m/s
 *          - Z-axis (Down): Positive downward velocity in m/s (negative = climb rate)
 * 
 *          Frame Conversion:
 *          NED velocity converted to internal NEU (North-East-Up) representation:
 *          - North and East: ×100 (m/s → cm/s)
 *          - Down: Negated and ×100 (-down_vel → up_vel in cm/s)
 * 
 *          Control Behavior:
 *          - Position controller disabled (no position target maintained)
 *          - Velocity controller attempts to maintain commanded velocity
 *          - Vehicle will drift with wind/disturbances while maintaining velocity
 *          - Altitude maintained only if vertical velocity component is zero
 *          - Horizontal position drifts continuously if horizontal velocity non-zero
 * 
 *          Yaw Control:
 *          Yaw automatically follows velocity direction (nose points in direction of travel).
 *          For independent yaw control, use set_target_velaccel_NED() instead.
 * 
 *          Use Cases:
 *          - Manual velocity control: Joystick or game controller input translation
 *          - Constant velocity flight: Maintain steady flight speed for surveys
 *          - Wind compensation: Adjust velocity to counter wind drift
 *          - Simple navigation: Velocity-based waypoint following
 * 
 *          Safety Considerations:
 *          - No position limits enforced - vehicle will continue moving indefinitely
 *          - Geofence still active and will trigger if boundaries crossed
 *          - Velocity limits from WPNAV_SPEED parameters still apply
 *          - Consider implementing position monitoring in calling script
 * 
 *          Stopping:
 *          To stop, command zero velocity: set_target_velocity_NED(Vector3f(0,0,0))
 *          Vehicle will decelerate and hold current position.
 * 
 * @param[in] vel_ned Target velocity in NED frame (m/s): {north_vel, east_vel, down_vel}
 * 
 * @return true Velocity command accepted, vehicle will maintain this velocity
 * @return false Failed - not in Guided mode
 * 
 * @note Input velocity in m/s (NED), converted to cm/s (NEU) internally
 * @note Down velocity negated to Up: vel_ned.z = -1.0 means climb at 1 m/s
 * @note No position hold - vehicle drifts if velocity non-zero
 * @note Yaw automatically follows velocity direction
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_velaccel_NED() for velocity control with acceleration feedforward and yaw
 * @see set_target_pos_NED() for position-based control (holds position at target)
 */
bool Copter::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, -vel_ned.z * 100.0f);
    mode_guided.set_velocity(vel_neu_cms);
    return true;
}

/**
 * @brief Set target velocity and acceleration in NED frame with yaw control (Lua scripting API)
 * 
 * @details Commands velocity with acceleration feedforward and independent yaw control.
 *          Provides responsive velocity tracking with acceleration feedforward to reduce
 *          lag during velocity changes, plus full yaw authority for orientation control.
 * 
 *          Coordinate Frame - NED (North-East-Down):
 *          Both velocity and acceleration specified in earth-fixed NED frame:
 *          - Velocity: m/s in NED frame {north_vel, east_vel, down_vel}
 *          - Acceleration: m/s² in NED frame {north_accel, east_accel, down_accel}
 * 
 *          Frame Conversion:
 *          Both vectors converted from NED to internal NEU:
 *          - North and East: ×100 (m/s→cm/s, m/s²→cm/s²)
 *          - Down: Negated and ×100 (down→up)
 * 
 *          Acceleration Feedforward:
 *          The acceleration parameter provides feedforward to the velocity controller:
 *          - Improves tracking during velocity changes
 *          - Reduces lag when accelerating or decelerating
 *          - Allows more aggressive velocity profiles
 *          - Controller: actual_accel = target_accel + PID_correction(velocity_error)
 * 
 *          Yaw Control Options:
 *          Unlike set_target_velocity_NED(), this function allows independent yaw control:
 *          - use_yaw=false: Yaw toward velocity direction (default behavior)
 *          - use_yaw=true, use_yaw_rate=false: Face absolute heading (yaw_deg)
 *          - use_yaw=true, use_yaw_rate=true: Rotate at yaw_rate_degs continuously
 *          - relative_yaw=true: yaw_deg relative to current heading (not north)
 *          - relative_yaw=false: yaw_deg absolute (0=north, 90=east)
 * 
 *          This enables applications like:
 *          - Flying sideways (velocity east, yaw north)
 *          - Orbiting while facing inward (tangential velocity, radial yaw)
 *          - Scanning while moving (sweep yaw, constant velocity)
 * 
 *          Use Cases:
 *          - Aggressive velocity control: Fast velocity changes with feedforward
 *          - Custom control algorithms: Velocity-based control with dynamics compensation
 *          - Orbiting maneuvers: Circular velocity with center-facing yaw
 *          - Scanning missions: Linear flight with sweeping camera yaw
 * 
 *          Safety Considerations:
 *          - No position hold - vehicle continues moving indefinitely
 *          - Acceleration feedforward can cause rapid movements
 *          - Geofence and velocity limits still enforced
 *          - Script must implement position monitoring if needed
 * 
 * @param[in] target_vel Target velocity in NED frame (m/s): {north_vel, east_vel, down_vel}
 * @param[in] target_accel Target acceleration in NED frame (m/s²): {north_accel, east_accel, down_accel}
 * @param[in] use_yaw True to specify yaw, false to yaw toward velocity direction
 * @param[in] yaw_deg Target yaw angle in degrees (0=north, 90=east, if use_yaw=true)
 * @param[in] use_yaw_rate True to specify yaw rate instead of target angle
 * @param[in] yaw_rate_degs Yaw rotation rate in degrees per second (if use_yaw_rate=true)
 * @param[in] relative_yaw True if yaw_deg is relative to current heading, false if absolute
 * 
 * @return true Velocity and acceleration command accepted, vehicle will track targets
 * @return false Failed - not in Guided mode
 * 
 * @note Velocity in m/s, acceleration in m/s², both converted to cm/s and cm/s² internally
 * @note Down velocity/acceleration negated to Up representation
 * @note Yaw angles converted from degrees to radians internally
 * @note No position hold - vehicle drifts continuously
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_velocity_NED() for simple velocity control without acceleration/yaw
 * @see set_target_posvelaccel_NED() for full position+velocity+acceleration control
 */
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    mode_guided.set_velaccel(vel_neu_cms, accel_neu_cms, use_yaw, radians(yaw_deg), use_yaw_rate, radians(yaw_rate_degs), relative_yaw);
    return true;
}

/**
 * @brief Set target attitude angles with climb rate (Lua scripting API)
 * 
 * @details Commands the vehicle to achieve specific roll, pitch, and yaw angles while
 *          maintaining a target vertical climb/descent rate. Provides low-level attitude
 *          control for scripts requiring direct attitude authority.
 * 
 *          Attitude Control - Euler Angles:
 *          Attitude specified as Euler angles in degrees:
 *          - Roll: Bank angle (positive = right wing down)
 *          - Pitch: Nose angle (positive = nose up)
 *          - Yaw: Heading angle (0=north, 90=east, 180=south, 270=west)
 * 
 *          Euler to Quaternion Conversion:
 *          Function converts Euler angles to quaternion representation for internal use.
 *          Conversion uses standard aerospace rotation sequence (roll-pitch-yaw).
 *          Angles converted from degrees to radians before quaternion generation.
 * 
 *          Climb Rate Control:
 *          Vertical velocity specified independently from attitude:
 *          - climb_rate_ms > 0: Climb (ascend)
 *          - climb_rate_ms < 0: Descend
 *          - climb_rate_ms = 0: Hold current altitude
 *          - Input in m/s, converted to cm/s (×100) for internal altitude controller
 * 
 *          Control Authority:
 *          This provides direct attitude command with altitude rate control:
 *          - Attitude controller maintains commanded angles
 *          - Altitude controller maintains commanded climb rate
 *          - Horizontal position NOT controlled (will drift based on attitude)
 * 
 *          Yaw Rate Option:
 *          - use_yaw_rate=false: Maintain absolute yaw heading (yaw_deg)
 *          - use_yaw_rate=true: Rotate continuously at yaw_rate_degs (yaw_deg ignored)
 * 
 *          Use Cases:
 *          - Manual angle control: Direct attitude commands from external controller
 *          - Flight testing: Commanded angle inputs for control tuning
 *          - Aerobatic maneuvers: Scripted attitude sequences
 *          - Stabilized descent: Controlled angles during landing approach
 * 
 *          Safety Considerations:
 *          - Large roll/pitch angles can cause rapid horizontal movement
 *          - No horizontal position control - vehicle will drift
 *          - Attitude limits (ANGLE_MAX) still enforced by attitude controller
 *          - Ensure adequate altitude before commanding aggressive attitudes
 * 
 *          Angular Velocity:
 *          This function commands attitude angles (not rates). For rate control,
 *          use set_target_rate_and_throttle() instead.
 * 
 * @param[in] roll_deg Target roll angle in degrees (positive = right wing down)
 * @param[in] pitch_deg Target pitch angle in degrees (positive = nose up)
 * @param[in] yaw_deg Target yaw angle in degrees (0=north, 90=east) if use_yaw_rate=false
 * @param[in] climb_rate_ms Target climb rate in m/s (positive=climb, negative=descend)
 * @param[in] use_yaw_rate True to use yaw_rate_degs instead of yaw_deg
 * @param[in] yaw_rate_degs Continuous yaw rate in deg/s (if use_yaw_rate=true)
 * 
 * @return true Attitude and climb rate command accepted
 * @return false Failed - not in Guided mode
 * 
 * @note Euler angles converted to quaternion internally (radians conversion)
 * @note Climb rate in m/s, converted to cm/s (×100) internally
 * @note No horizontal position control - vehicle drifts based on attitude
 * @note Angular velocity set to zero (angle control, not rate control)
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_rate_and_throttle() for angular rate control instead of angles
 * @see set_target_pos_NED() for position-controlled flight
 */
bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    mode_guided.set_angle(q, Vector3f{}, climb_rate_ms*100, false);
    return true;
}

/**
 * @brief Set target angular rates with throttle (Lua scripting API)
 * 
 * @details Commands angular velocity (rate) control with direct throttle authority.
 *          Provides the lowest-level rotational control for scripts implementing custom
 *          stabilization, aerobatics, or specialized flight behaviors.
 * 
 *          Rate Control - Body Frame Angular Velocities:
 *          Angular rates specified in body frame (relative to vehicle):
 *          - Roll rate: Rotation about nose-tail axis (deg/s, positive = right wing down)
 *          - Pitch rate: Rotation about left-right axis (deg/s, positive = nose up)
 *          - Yaw rate: Rotation about vertical axis (deg/s, positive = clockwise viewed from above)
 * 
 *          Body Frame Reference:
 *          Rates are relative to vehicle, not earth frame. For example, roll_rate_dps=10
 *          rotates the vehicle right at 10 deg/s regardless of current orientation.
 * 
 *          Unit Conversion:
 *          Input rates in degrees per second, converted to radians per second (×π/180)
 *          for internal rate controller which uses rad/s as standard unit.
 * 
 *          Throttle Control:
 *          Direct throttle authority (0.0 to 1.0):
 *          - 0.0: Minimum throttle (motors at hover or minimum)
 *          - 0.5: Approximately hover throttle
 *          - 1.0: Maximum throttle
 *          - No altitude hold - pilot/script responsible for vertical control
 * 
 *          Zero Quaternion Convention:
 *          Quaternion set to zero signals rate control mode (not angle control).
 *          Guided mode checks quaternion: if zero→rate control, else→angle control.
 * 
 *          Control Authority:
 *          Most direct rotational control available:
 *          - Rate controller directly commands angular accelerations
 *          - No angle limits enforced (can rotate inverted if physically capable)
 *          - No position or altitude control
 *          - Full pilot/script responsibility for stability
 * 
 *          Use Cases:
 *          - Acro mode via scripting: Custom aerobatic maneuvers
 *          - Custom stabilization: Implement alternative control algorithms
 *          - Rate-based FPV flight: Script-controlled agile flight
 *          - Control algorithm research: Test novel rate control approaches
 *          - Manual rate input: External controller (joystick) rate commands
 * 
 *          Safety Considerations:
 *          - No attitude limits - vehicle can enter any orientation
 *          - No altitude hold - must manage throttle manually
 *          - No position control - horizontal drift unconstrained
 *          - Requires experienced piloting or robust control algorithm
 *          - Consider crash detection and altitude failsafes
 *          - Test thoroughly in SITL before hardware flight
 * 
 *          Comparison to Angle Control:
 *          - Angle control (set_target_angle_and_climbrate): Stabilized, self-leveling
 *          - Rate control (this function): Direct rates, no stabilization
 * 
 * @param[in] roll_rate_dps Target roll rate in deg/s (positive = right wing down)
 * @param[in] pitch_rate_dps Target pitch rate in deg/s (positive = nose up)
 * @param[in] yaw_rate_dps Target yaw rate in deg/s (positive = clockwise from above)
 * @param[in] throttle Direct throttle value (0.0=min, 0.5≈hover, 1.0=max)
 * 
 * @return true Rate and throttle command accepted
 * @return false Failed - not in Guided mode
 * 
 * @warning No attitude limits enforced - vehicle can enter any orientation
 * @warning No altitude hold - manual throttle control required
 * @warning Requires experienced piloting or robust control algorithm
 * @note Angular rates in deg/s, converted to rad/s (×π/180) internally
 * @note Zero quaternion signals rate control mode to guided mode handler
 * @note Rates specified in body frame (relative to vehicle orientation)
 * @note Conditional compilation: Only available if scripting enabled
 * @see set_target_angle_and_climbrate() for stabilized angle control with altitude hold
 * @see AC_AttitudeControl for rate controller implementation
 */
bool Copter::set_target_rate_and_throttle(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps, float throttle)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // Zero quaternion indicates rate control
    Quaternion q;
    q.zero();

    // Convert from degrees per second to radians per second
    Vector3f ang_vel_body { roll_rate_dps, pitch_rate_dps, yaw_rate_dps };
    ang_vel_body *= DEG_TO_RAD;

    // Pass to guided mode
    mode_guided.set_angle(q, ang_vel_body, throttle, true);
    return true;
}

/**
 * @brief Register a custom flight mode for scripting (Lua scripting API)
 * 
 * @details Allows Lua scripts to define custom flight modes with unique mode numbers and
 *          names. These custom modes appear as selectable flight modes in ground control
 *          stations and provide a state structure for the script to implement mode logic.
 * 
 *          Mode Registration Process:
 *          1. Check if mode already registered (allows scripting restarts without errors)
 *          2. Verify mode number not used by built-in modes
 *          3. Allocate mode slot in mode_guided_custom array
 *          4. Duplicate name strings (prevent dangling pointers after script reload)
 *          5. Create ModeGuidedCustom instance with mode number and names
 *          6. Notify GCS that available modes have changed
 * 
 *          Mode Number Assignment:
 *          Custom mode numbers should be in the range reserved for user modes (typically
 *          100-255) to avoid conflicts with built-in ArduPilot modes. Script must ensure
 *          unique mode numbers across all registered custom modes.
 * 
 *          Mode Names:
 *          - full_name: Complete descriptive name (shown in GCS mode lists)
 *          - short_name: 4-character abbreviation (shown on displays with limited space)
 * 
 *          State Structure:
 *          Returns pointer to custom_mode_state structure that script uses to track mode:
 *          - Script stores mode-specific state in this structure
 *          - Structure persists across script updates (allows mode state continuity)
 *          - Script implements mode behavior in update loop based on this state
 * 
 *          Scripting Restart Handling:
 *          If script reloads and re-registers same mode (number and names match), returns
 *          existing state structure. This prevents duplicate registrations and preserves
 *          mode state across script restarts.
 * 
 *          Memory Management:
 *          - Allocates memory for name strings using strdup/strndup
 *          - Allocates ModeGuidedCustom instance using NEW_NOTHROW
 *          - Cleans up on allocation failure to prevent memory leaks
 *          - Name strings remain valid for lifetime of mode registration
 * 
 *          GCS Integration:
 *          Calls gcs().available_modes_changed() to notify ground control stations that
 *          flight mode list has changed. GCS will re-request mode list and display new
 *          custom mode as selectable option.
 * 
 *          Capacity Limits:
 *          Limited number of custom mode slots (defined by mode_guided_custom array size).
 *          Registration fails if all slots occupied. Check return value to detect failure.
 * 
 *          Use Cases:
 *          - Custom autonomous behaviors: Implement specialized mission logic
 *          - Research flight modes: Test novel control algorithms
 *          - Application-specific modes: Domain-specific flight behaviors
 *          - Mode extensions: Add functionality beyond built-in modes
 * 
 * @param[in] num Mode number (typically 100-255 for user modes)
 * @param[in] full_name Complete mode name string (displayed in GCS)
 * @param[in] short_name 4-character mode abbreviation (for compact displays)
 * 
 * @return Pointer to custom_mode_state structure for this mode (use to track mode state)
 * @return nullptr if registration failed (mode number conflict, no free slots, allocation failure)
 * 
 * @note Checks for duplicate registration (allows scripting restarts)
 * @note Name strings duplicated internally (safe across script reloads)
 * @note Notifies GCS of mode list change after successful registration
 * @note Fails if mode number already used by built-in mode
 * @note Fails if no free custom mode slots available
 * @note Conditional compilation: Only available if scripting and guided mode enabled
 * @see ModeGuidedCustom for custom mode implementation
 * @see AP_Vehicle::custom_mode_state for mode state structure
 */
AP_Vehicle::custom_mode_state* Copter::register_custom_mode(const uint8_t num, const char* full_name, const char* short_name)
{
    const Mode::Number number = (Mode::Number)num;

    // See if this mode has been registered already, if it has return the state for it
    // This allows scripting restarts
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if (mode_guided_custom[i] == nullptr) {
            break;
        }
        if ((mode_guided_custom[i]->mode_number() == number) &&
            (strcmp(mode_guided_custom[i]->name(), full_name) == 0) &&
            (strncmp(mode_guided_custom[i]->name4(), short_name, 4) == 0)) {
            return &mode_guided_custom[i]->state;
        }
    }

    // Number already registered to existing mode
    if (mode_from_mode_num(number) != nullptr) {
        return nullptr;
    }

    // Find free slot
    for (uint8_t i = 0; i < ARRAY_SIZE(mode_guided_custom); i++) {
        if (mode_guided_custom[i] == nullptr) {
            // Duplicate strings so were not pointing to unknown memory
            const char* full_name_copy = strdup(full_name);
            const char* short_name_copy = strndup(short_name, 4);
            if ((full_name_copy != nullptr) && (short_name_copy != nullptr)) {
                mode_guided_custom[i] = NEW_NOTHROW ModeGuidedCustom(number, full_name_copy, short_name_copy);
            }
            if (mode_guided_custom[i] == nullptr) {
                // Allocation failure
                free((void*)full_name_copy);
                free((void*)short_name_copy);
                return nullptr;
            }

            // Registration successful, notify the GCS that it should re-request the available modes
            gcs().available_modes_changed();

            return &mode_guided_custom[i]->state;
        }
    }

    // No free slots
    return nullptr;
}
#endif // MODE_GUIDED_ENABLED

#if MODE_CIRCLE_ENABLED
/**
 * @brief Get current circle mode radius (Lua scripting API)
 * 
 * @details Retrieves the current radius of the circular flight pattern in Circle mode.
 *          Circle mode flies the vehicle in a circular pattern around a center point.
 * 
 *          Unit Conversion:
 *          Internal circle radius stored in centimeters, converted to meters (÷100)
 *          for scripting API which uses SI units (meters).
 * 
 *          Circle Mode Behavior:
 *          In Circle mode, vehicle orbits around a center point at the current radius:
 *          - Center point: Usually vehicle position when Circle mode entered
 *          - Direction: Controlled by CIRCLE_DIR parameter (CW or CCW)
 *          - Speed: Controlled by rate (see set_circle_rate())
 *          - Altitude: Maintains altitude at which mode was entered
 * 
 *          Use Cases:
 *          - Monitor orbit parameters: Check current circle configuration
 *          - Dynamic circle adjustment: Read before modifying radius
 *          - Mission planning: Verify circle size for survey patterns
 * 
 * @param[out] radius_m Current circle radius in meters
 * 
 * @return true Radius retrieved successfully
 * @return false Should not fail (always returns true)
 * 
 * @note Radius in meters (converted from internal centimeters)
 * @note Conditional compilation: Only available if Circle mode and scripting enabled
 * @see set_circle_rate() for controlling circle rotation rate
 * @see ModeCircle for Circle mode implementation
 */
bool Copter::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav->get_radius_cm() * 0.01f;
    return true;
}

/**
 * @brief Set circle mode rotation rate (Lua scripting API)
 * 
 * @details Sets the rate at which the vehicle orbits in Circle mode, specified in
 *          degrees per second. Positive rates orbit clockwise (viewed from above),
 *          negative rates orbit counter-clockwise.
 * 
 *          Rate Units:
 *          Input rate in degrees per second:
 *          - Positive: Clockwise rotation (viewed from above)
 *          - Negative: Counter-clockwise rotation
 *          - Magnitude: Speed of rotation around circle
 * 
 *          Relationship to Linear Speed:
 *          Angular rate and radius determine linear (tangential) speed:
 *          - linear_speed = radius × rate × (π/180)
 *          - Example: 10 m radius, 10 deg/s rate = ~1.75 m/s tangential speed
 * 
 *          Circle Mode Behavior:
 *          Rate setting affects how fast vehicle orbits the circle:
 *          - Higher rate: Faster orbit, higher centripetal acceleration
 *          - Lower rate: Slower orbit, more stable
 *          - Zero rate: Vehicle holds position at current point on circle
 * 
 *          Dynamic Adjustment:
 *          Rate can be changed while Circle mode active. Vehicle smoothly transitions
 *          to new rotation rate. Useful for:
 *          - Adaptive orbit speed based on mission requirements
 *          - Slow down for detailed observation
 *          - Speed up for rapid area coverage
 * 
 *          Safety Considerations:
 *          - High rates combined with large radius can exceed velocity limits
 *          - Excessive rates may exceed vehicle's centripetal acceleration capability
 *          - Consider WPNAV_SPEED and vehicle performance limits
 * 
 *          Use Cases:
 *          - Survey missions: Adjust orbit speed for camera frame rate
 *          - Object tracking: Match orbit speed to object movement
 *          - Dynamic patterns: Script-controlled rate changes
 * 
 * @param[in] rate_degs Circle rotation rate in degrees per second (positive=CW, negative=CCW)
 * 
 * @return true Rate set successfully
 * @return false Should not fail (always returns true)
 * 
 * @note Rate in deg/s: positive=clockwise, negative=counter-clockwise (viewed from above)
 * @note Linear speed = radius × rate × (π/180)
 * @note Can be changed dynamically while Circle mode active
 * @note Conditional compilation: Only available if Circle mode and scripting enabled
 * @see get_circle_radius() for retrieving current circle radius
 * @see ModeCircle for Circle mode implementation
 */
bool Copter::set_circle_rate(float rate_degs)
{
    circle_nav->set_rate_degs(rate_degs);
    return true;
}
#endif

/**
 * @brief Set desired horizontal speed for current flight mode (Lua scripting API)
 * 
 * @details Sets the target horizontal speed for modes that support speed control.
 *          The speed setting affects how fast the vehicle moves toward waypoints or
 *          position targets in autonomous modes.
 * 
 *          Unit Conversion:
 *          Input speed in m/s (SI units), converted to cm/s (×100) for internal
 *          position controller which uses centimeters as standard unit.
 * 
 *          Mode Support:
 *          Not all flight modes support speed adjustment. Mode's set_speed_xy_cms()
 *          method determines if speed can be changed:
 *          - Supported: Auto, Guided, RTL, Smart RTL, etc. (position-controlled modes)
 *          - Not supported: Stabilize, AltHold, Loiter (manual or fixed-speed modes)
 * 
 *          Speed Application:
 *          Speed affects horizontal movement toward position targets:
 *          - Auto mode: Speed traveling between waypoints
 *          - Guided mode: Speed approaching commanded position
 *          - RTL mode: Speed returning to home
 *          - Does NOT affect vertical speed (climb/descent rates separate)
 * 
 *          Speed Limits:
 *          Actual speed constrained by vehicle configuration:
 *          - WPNAV_SPEED parameter: Maximum waypoint navigation speed
 *          - WPNAV_ACCEL parameter: Acceleration limits affect speed changes
 *          - Vehicle dynamics: Physical capability limits maximum speed
 *          - If commanded speed > limits, vehicle uses maximum allowed speed
 * 
 *          Use Cases:
 *          - Dynamic mission speed: Adjust speed based on conditions
 *          - Slow approach: Reduce speed near sensitive areas
 *          - Fast transit: Increase speed in open areas
 *          - Adaptive navigation: Speed based on obstacle proximity
 * 
 * @param[in] speed Desired horizontal speed in m/s
 * 
 * @return true Speed set successfully in current flight mode
 * @return false Speed setting not supported in current flight mode
 * 
 * @note Speed in m/s, converted to cm/s (×100) internally
 * @note Not all flight modes support speed adjustment
 * @note Actual speed constrained by WPNAV_SPEED and vehicle capability
 * @note Does not affect vertical speed (climb/descent rates)
 * @note Conditional compilation: Available if scripting enabled
 * @see WPNAV_SPEED parameter for maximum navigation speed limit
 * @see set_target_location() which moves vehicle at this speed
 */
bool Copter::set_desired_speed(float speed)
{
    return flightmode->set_speed_xy_cms(speed * 100.0f);
}

#if MODE_AUTO_ENABLED
/**
 * @brief Check if flight mode supports NAV_SCRIPT_TIME mission commands (Lua scripting API)
 * 
 * @details Determines whether the specified flight mode supports NAV_SCRIPT_TIME mission
 *          commands, which allow Lua scripts to control mission execution timing.
 * 
 *          NAV_SCRIPT_TIME Support:
 *          Currently only Auto mode supports NAV_SCRIPT_TIME commands:
 *          - Auto mode: Returns true (script can control mission waypoint timing)
 *          - All other modes: Returns false (command not supported)
 * 
 *          NAV_SCRIPT_TIME Workflow:
 *          When mission contains NAV_SCRIPT_TIME command:
 *          1. Auto mode pauses at waypoint
 *          2. Script retrieves command with nav_script_time()
 *          3. Script executes custom behavior
 *          4. Script signals completion with nav_script_time_done()
 *          5. Mission advances to next waypoint
 * 
 *          Use Cases:
 *          - Custom mission behaviors: Script-controlled actions at waypoints
 *          - Conditional mission logic: Script decides when to proceed
 *          - Data collection: Wait for sensor readings before continuing
 *          - Adaptive missions: Adjust mission based on real-time conditions
 * 
 * @param[in] mode Flight mode number to check
 * 
 * @return true Mode supports NAV_SCRIPT_TIME commands (currently only Auto mode)
 * @return false Mode does not support NAV_SCRIPT_TIME commands
 * 
 * @note Currently only Auto mode returns true
 * @note Conditional compilation: Only available if Auto mode and scripting enabled
 * @see nav_script_time() to retrieve active NAV_SCRIPT_TIME command
 * @see nav_script_time_done() to signal command completion
 */
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

/**
 * @brief Retrieve active NAV_SCRIPT_TIME mission command parameters (Lua scripting API)
 * 
 * @details Retrieves the parameters of the currently active NAV_SCRIPT_TIME mission
 *          command. This allows Lua scripts to read command data and implement custom
 *          behaviors during mission execution.
 * 
 *          NAV_SCRIPT_TIME Command Structure:
 *          Mission command contains ID and parameters that script uses to determine action:
 *          - id: Command identifier (script uses to distinguish different commands)
 *          - cmd: Command type/subcommand (script-defined meaning)
 *          - arg1, arg2: Float arguments (script-defined meaning)
 *          - arg3, arg4: Integer arguments (script-defined meaning)
 * 
 *          Execution Flow:
 *          1. Mission reaches NAV_SCRIPT_TIME waypoint
 *          2. Auto mode pauses mission execution
 *          3. Script calls this function to retrieve command parameters
 *          4. Script interprets parameters and executes corresponding behavior
 *          5. Script calls nav_script_time_done(id) to signal completion
 *          6. Mission advances to next waypoint
 * 
 *          Mode Requirement:
 *          Only works when vehicle in Auto mode. Returns false if in any other mode.
 *          Script should check vehicle mode before calling this function.
 * 
 *          Parameter Usage Patterns:
 *          Script defines meaning of parameters based on application:
 *          - id=1, cmd=0: Survey pattern with arg1=altitude, arg2=spacing
 *          - id=2, cmd=1: Photo capture with arg3=count, arg4=interval_ms
 *          - id=3, cmd=2: Custom maneuver with arg1=radius, arg2=speed
 * 
 *          Use Cases:
 *          - Mission scripting: Execute complex behaviors at waypoints
 *          - Data collection: Custom sensor collection routines
 *          - Adaptive missions: Conditional logic during mission
 *          - Specialized maneuvers: Scripted flight patterns within mission
 * 
 * @param[out] id Command identifier (matches id in nav_script_time_done() call)
 * @param[out] cmd Command type/subcommand (script-defined meaning)
 * @param[out] arg1 Float argument 1 (script-defined meaning)
 * @param[out] arg2 Float argument 2 (script-defined meaning)
 * @param[out] arg3 Integer argument 3 (script-defined meaning)
 * @param[out] arg4 Integer argument 4 (script-defined meaning)
 * 
 * @return true Command parameters retrieved successfully
 * @return false Failed - not in Auto mode or no active NAV_SCRIPT_TIME command
 * 
 * @note Only works in Auto mode
 * @note Parameter meanings defined by script application
 * @note Mission paused until nav_script_time_done() called
 * @note Conditional compilation: Only available if Auto mode and scripting enabled
 * @see nav_scripting_enable() to check if mode supports NAV_SCRIPT_TIME
 * @see nav_script_time_done() to signal command completion
 */
bool Copter::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2, arg3, arg4);
}

/**
 * @brief Signal completion of NAV_SCRIPT_TIME mission command (Lua scripting API)
 * 
 * @details Notifies Auto mode that the script has completed executing the NAV_SCRIPT_TIME
 *          command, allowing the mission to advance to the next waypoint.
 * 
 *          Completion Protocol:
 *          1. Script retrieves command with nav_script_time(), receives command ID
 *          2. Script executes custom behavior defined by command parameters
 *          3. Script calls this function with matching ID to signal completion
 *          4. Auto mode verifies ID matches active command
 *          5. Mission advances to next waypoint
 * 
 *          ID Matching:
 *          The id parameter must match the ID from the active NAV_SCRIPT_TIME command.
 *          This prevents race conditions where:
 *          - Mission advances while script still processing old command
 *          - Script completes command for wrong waypoint
 *          - Multiple scripts interact with same mission
 * 
 *          Mode Requirement:
 *          Only effective when vehicle in Auto mode. Silently ignored if in other modes.
 *          No return value to indicate success/failure (void function).
 * 
 *          Mission Continuation:
 *          After this call, mission immediately begins executing next waypoint:
 *          - Script should complete all command-related actions before calling
 *          - Any cleanup or logging should be done before calling
 *          - Vehicle may begin moving immediately after return
 * 
 *          Error Handling:
 *          If ID mismatch detected:
 *          - Command completion ignored
 *          - Mission remains paused at current waypoint
 *          - Script must retry with correct ID
 * 
 *          Use Cases:
 *          - Signal survey complete: Finish data collection, advance mission
 *          - Confirm action: Verify hardware operation, continue mission
 *          - Timed delays: Wait specified duration, resume mission
 *          - Conditional proceed: Check conditions, continue when met
 * 
 * @param[in] id Command identifier from nav_script_time() call
 * 
 * @return void No return value (silently ignored if not in Auto mode)
 * 
 * @note id must match active NAV_SCRIPT_TIME command ID
 * @note Mission advances immediately after this call
 * @note Only effective in Auto mode (ignored in other modes)
 * @note No return value to indicate success/failure
 * @note Conditional compilation: Only available if Auto mode and scripting enabled
 * @see nav_script_time() to retrieve command ID and parameters
 * @see nav_scripting_enable() to check mode support
 */
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}
#endif

/**
 * @brief Check if EKF failsafe has triggered (Lua scripting API)
 * 
 * @details Returns the current state of the EKF (Extended Kalman Filter) failsafe.
 *          This allows Lua scripts to detect navigation system failures and implement
 *          custom failsafe behaviors or notifications.
 * 
 *          EKF Failsafe Triggers:
 *          The EKF failsafe activates when navigation estimate becomes unreliable:
 *          - Position variance exceeds threshold (EKF position drift detected)
 *          - Velocity variance exceeds threshold (EKF velocity estimate unreliable)
 *          - GPS glitch detected (sudden position jump)
 *          - IMU inconsistency detected (sensor disagreement)
 *          - Compass interference detected (magnetic anomaly)
 * 
 *          Failsafe State Persistence:
 *          Once triggered, failsafe state persists until:
 *          - EKF variances return below threshold AND
 *          - Stable navigation restored for minimum duration (typically 1 second)
 *          
 *          This prevents rapid failsafe cycling during transient conditions.
 * 
 *          Vehicle Behavior When Triggered:
 *          When EKF failsafe active, vehicle typically:
 *          - Switches to stabilize/altitude-hold mode (loses position control)
 *          - Triggers Land mode if configured (EKF_FAILSAFE parameter)
 *          - Disables position-dependent modes (Loiter, Auto, Guided)
 *          - Activates GCS warnings and pre-arm checks
 * 
 *          Script Use Cases:
 *          - Custom failsafe actions: Script implements application-specific responses
 *          - Data logging: Record EKF failsafe events with custom metadata
 *          - User notifications: Send telemetry warnings via custom protocols
 *          - Conditional mission logic: Abort mission if navigation unreliable
 *          - Safety monitoring: Track failsafe frequency for health assessment
 * 
 *          Integration with Script Logic:
 *          Script can check this status and implement custom behaviors:
 *          - If failsafe active, command immediate landing at current location
 *          - If failsafe active, abort survey mission and return using GPS only
 *          - If failsafe active, switch to manual control mode
 *          - Log failsafe events with position/time for post-flight analysis
 * 
 * @return true EKF failsafe has triggered (navigation unreliable)
 * @return false EKF operating normally (navigation reliable)
 * 
 * @note const function - does not modify vehicle state
 * @note Only reads failsafe state, does not trigger failsafe
 * @note Primarily intended for Lua scripting API
 * @warning Do not use as sole navigation reliability indicator - check EKF status messages
 * @see EKF_FAILSAFE parameter configures vehicle response to EKF failures
 * @see EKF_CHECK parameter configures variance thresholds for failsafe trigger
 * 
 * Source: ArduCopter/Copter.cpp:1792-1796
 */
bool Copter::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

/**
 * @brief Get current target location from active flight mode (Lua scripting API)
 * 
 * @details Retrieves the waypoint/target location that the current flight mode is
 *          navigating towards. This allows scripts to read navigation state and make
 *          decisions based on current target.
 * 
 *          Mode-Specific Target Behavior:
 *          Different flight modes report different target locations:
 *          
 *          - Auto mode: Reports next waypoint in mission
 *          - Guided mode: Reports commanded target location (MAVLink or script)
 *          - RTL/SmartRTL: Reports home location or rally point
 *          - Loiter mode: Reports loiter center point
 *          - PosHold mode: Reports current hold position
 *          - Circle mode: Reports circle center point
 *          - Land mode: Reports landing location
 *          - Stabilize/AltHold: No target location (returns false)
 * 
 *          Location Information:
 *          The returned Location structure contains:
 *          - Latitude/longitude (WGS84 coordinate system)
 *          - Altitude (various reference frames: AMSL, terrain, home-relative)
 *          - Altitude frame (indicates which reference frame)
 *          - Flags indicating validity of components
 * 
 *          Coordinate Frame:
 *          Location uses geodetic coordinates (latitude/longitude in degrees * 1e7):
 *          - Latitude: Positive north, negative south (-90° to +90°)
 *          - Longitude: Positive east, negative west (-180° to +180°)
 *          - Altitude: Depends on altitude frame (typically cm or m)
 * 
 *          Altitude Reference Frames:
 *          Location may use different altitude references:
 *          - ABSOLUTE: Altitude above mean sea level (AMSL)
 *          - TERRAIN_RELATIVE: Altitude above terrain (requires terrain database)
 *          - HOME_RELATIVE: Altitude above home location
 * 
 *          Use Cases:
 *          - Mission monitoring: Check if vehicle approaching target waypoint
 *          - Adaptive missions: Adjust script behavior based on target location
 *          - Geofencing: Verify target is within allowed area before executing
 *          - Telemetry: Report target location to external systems
 *          - Conditional logic: Execute script actions when near specific locations
 * 
 *          Example Usage Pattern:
 *          ```lua
 *          local target = Location()
 *          if vehicle:get_target_location(target) then
 *              local distance = target:get_distance(current_location)
 *              if distance < 50 then  -- Within 50m of target
 *                  -- Execute custom action near target
 *              end
 *          end
 *          ```
 * 
 * @param[out] target_loc Location structure to receive target waypoint coordinates
 * 
 * @return true Target location retrieved successfully
 * @return false Failed - mode has no target location or target not available
 * 
 * @note Mode must support waypoint navigation to return valid target
 * @note Returned location includes altitude frame information
 * @note Call fails in modes without navigation target (Stabilize, AltHold)
 * @see set_target_location() to command new target location in Guided mode
 * @see update_target_location() to modify existing target with race condition protection
 * 
 * Source: ArduCopter/Copter.cpp:1798-1802
 */
bool Copter::get_target_location(Location& target_loc)
{
    return flightmode->get_wp(target_loc);
}

/**
 * @brief Update target location with race condition protection (Lua scripting API)
 * 
 * @details Safely updates the vehicle's target location by verifying the current target
 *          matches the expected old location before applying the update. This prevents
 *          race conditions in scripting scenarios where multiple scripts or commands
 *          might modify the target simultaneously.
 * 
 *          Race Condition Prevention:
 *          Without verification, race conditions could occur:
 *          1. Script A reads target location
 *          2. Script B or pilot changes target
 *          3. Script A tries to update based on stale target
 *          4. Script A unknowingly overrides Script B's command
 *          
 *          This function prevents such scenarios by requiring the caller to provide
 *          the expected current target. Update only proceeds if it matches.
 * 
 *          Verification Process:
 *          Function performs two checks before updating target:
 *          1. Location match: old_loc coordinates must match current target location
 *          2. Altitude frame match: old_loc altitude frame must match new_loc frame
 *          
 *          If either check fails, update rejected and function returns false.
 * 
 *          Update Workflow:
 *          Typical script usage pattern:
 *          1. Call get_target_location() to read current target
 *          2. Calculate desired new target based on mission logic
 *          3. Call update_target_location(old, new) with verification
 *          4. Check return value - retry if false (target changed by another source)
 * 
 *          Altitude Frame Consistency:
 *          The altitude frame check ensures coordinate system consistency:
 *          - Cannot update from AMSL to terrain-relative without explicit conversion
 *          - Cannot update from home-relative to AMSL without proper handling
 *          - Prevents altitude reference frame mismatches that could cause altitude errors
 * 
 *          Mode Requirements:
 *          Update only succeeds in modes that support commanded targets:
 *          - Guided mode: Primary use case (MAVLink or scripting commands)
 *          - Auto mode (guided sub-mode): When temporarily controlled by script
 *          - Other modes: Update typically fails (no external target control)
 * 
 *          Use Cases:
 *          - Adaptive missions: Gradually adjust waypoint based on conditions
 *          - Path smoothing: Interpolate between waypoints in guided mode
 *          - Wind compensation: Shift target upwind for approach
 *          - Dynamic mission editing: Modify upcoming waypoints in real-time
 *          - Collision avoidance: Adjust target to avoid detected obstacles
 * 
 *          Example Usage Pattern:
 *          ```lua
 *          local current_target = Location()
 *          if vehicle:get_target_location(current_target) then
 *              local new_target = current_target:copy()
 *              new_target:offset(10, 0)  -- Shift 10m north
 *              
 *              if not vehicle:update_target_location(current_target, new_target) then
 *                  -- Update failed - target changed by another source
 *                  -- Retry or abort operation
 *              end
 *          end
 *          ```
 * 
 *          Thread Safety:
 *          While this function provides some race condition protection, it cannot
 *          guarantee atomic updates in multi-threaded scenarios. Scripts should:
 *          - Minimize time between get_target_location and update_target_location
 *          - Implement retry logic if update fails
 *          - Avoid multiple scripts modifying same target simultaneously
 * 
 * @param[in] old_loc Expected current target location (for verification)
 * @param[in] new_loc Desired new target location to command
 * 
 * @return true Target updated successfully (old_loc matched, new_loc commanded)
 * @return false Update failed - target location mismatch or altitude frame mismatch
 * 
 * @note Wrapper for set_target_location() with race condition protection
 * @note Both location and altitude frame must match for update to proceed
 * @note Primarily intended for Guided mode operation
 * @warning If update fails, check if pilot or GCS changed target
 * @warning Altitude frame mismatch prevents update even if coordinates match
 * @see get_target_location() to retrieve current target for verification
 * @see set_target_location() for direct target update without verification
 * 
 * Source: ArduCopter/Copter.cpp:1804-1822
 */
bool Copter::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      by checking the caller has provided the correct old target
      location we prevent a race condition where the user changes mode
      or commands a different target in the controlling lua script
    */
    Location next_WP_loc;
    flightmode->get_wp(next_WP_loc);
    if (!old_loc.same_loc_as(next_WP_loc) ||
         old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }

    return set_target_location(new_loc);
}

#endif // AP_SCRIPTING_ENABLED

/**
 * @brief Check if vehicle is currently executing a landing maneuver
 * 
 * @details Queries the current flight mode to determine if the vehicle is actively
 *          landing. This is distinct from being landed (ap.land_complete); this
 *          indicates a landing is in progress.
 * 
 *          Landing Detection by Mode:
 *          - Land mode: Always true (dedicated landing mode)
 *          - Auto mode: True during DO_LAND mission command execution
 *          - RTL mode: True during final descent phase after reaching home
 *          - Smart RTL: True during landing phase
 *          - Guided mode: True if commanded to land via MAVLink
 *          - Other modes: False
 * 
 *          Landing Behavior Modifications:
 *          When is_landing() returns true, several systems adjust behavior:
 *          - Landing detector: More aggressive landing detection thresholds
 *          - Position controller: Accepts larger position errors near ground
 *          - Throttle controller: Reduces throttle as ground contact detected
 *          - Compass: May disable compass motor compensation near ground
 *          - EKF: May relax innovation checks during final touchdown
 * 
 *          Use Cases:
 *          - Adjust control gains for smooth touchdown
 *          - Enable landing-specific sensor processing
 *          - Trigger landing gear deployment
 *          - Modify failsafe behavior (e.g., don't trigger low altitude fence)
 *          - Telemetry reporting to ground station
 * 
 * @return true Vehicle is actively executing landing sequence
 * @return false Vehicle is not landing (may be flying, landed, or taking off)
 * 
 * @note This indicates landing in progress, not landed state (use ap.land_complete for that)
 * @note Mode-dependent; delegated to current flight mode implementation
 * @see is_taking_off() for takeoff detection
 * @see ap.land_complete for detection of already-landed state
 * @see Mode::is_landing() for mode-specific landing logic
 */
bool Copter::is_landing() const
{
    return flightmode->is_landing();
}

/**
 * @brief Check if vehicle is currently executing a takeoff maneuver
 * 
 * @details Queries the current flight mode to determine if the vehicle is actively
 *          taking off. This indicates the vehicle is in a special takeoff state with
 *          modified control behavior for safe ground departure.
 * 
 *          Takeoff Detection by Mode:
 *          - Guided mode: True during takeoff command execution (NAV_TAKEOFF or MAVLink)
 *          - Auto mode: True during NAV_TAKEOFF or NAV_VTOL_TAKEOFF mission commands
 *          - Throw mode: True during throw detection and initial climb
 *          - Other modes: Generally false (manual modes don't have explicit takeoff)
 * 
 *          Takeoff Phase Behavior:
 *          During takeoff, several control modifications are active:
 *          - Position controller: Prioritizes vertical climb over horizontal position
 *          - Attitude limits: May use reduced tilt limits for stability
 *          - Compass: Delays compass use until clear of magnetic interference
 *          - EKF: May use relaxed innovation thresholds during initial motion
 *          - Throttle: Special throttle ramp for smooth liftoff
 *          - Takeoff complete detection: Monitors altitude and climb rate
 * 
 *          Takeoff Complete Criteria:
 *          Takeoff ends when:
 *          - Target altitude reached (typically set by NAV_TAKEOFF or parameter)
 *          - Climb rate drops below threshold (indicating altitude hold achieved)
 *          - Mode changes to non-takeoff mode
 * 
 *          Use Cases:
 *          - Enable takeoff-specific control parameters
 *          - Delay GPS position use until motion established
 *          - Trigger landing gear retraction
 *          - Modify failsafe behavior (e.g., don't RTL immediately after takeoff)
 *          - Telemetry reporting to ground station
 * 
 * @return true Vehicle is actively executing takeoff sequence
 * @return false Vehicle is not taking off (may be flying, landed, or landing)
 * 
 * @note Mode-dependent; delegated to current flight mode implementation
 * @note Primarily relevant for guided takeoff commands, not manual throttle-up
 * @see is_landing() for landing detection
 * @see Mode::is_taking_off() for mode-specific takeoff logic
 * @see auto_armed flag for indication of autonomous arming during takeoff
 */
bool Copter::is_taking_off() const
{
    return flightmode->is_taking_off();
}

/**
 * @brief Check if current flight mode requires a mission to be loaded
 * 
 * @details Determines whether the current flight mode needs mission waypoints to operate.
 *          This is used for pre-arm checks and mode change validation to prevent entering
 *          modes that would have no behavior without mission commands.
 * 
 *          Mission-Dependent Modes:
 *          - Auto mode: REQUIRES mission (returns true)
 *            Executes sequential mission commands from onboard waypoint storage
 *          
 *          Non-Mission Modes (all return false):
 *          - Stabilize, AltHold, Loiter: Manual or semi-autonomous without mission
 *          - RTL: Uses home position, not mission waypoints
 *          - Guided: Uses real-time commands from GCS, not pre-planned mission
 *          - Land: Simple landing without mission
 *          - All other modes: Independent of mission system
 * 
 *          Pre-Arm Check Integration:
 *          If user attempts to arm in Auto mode without a valid mission, pre-arm checks
 *          fail with "No Mission Loaded" error. This prevents arming when Auto mode
 *          would have undefined behavior.
 * 
 *          Mode Change Validation:
 *          Switching to Auto mode is blocked if no mission is loaded, preventing
 *          unexpected behavior during flight. Ground station must upload mission first.
 * 
 *          Conditional Compilation:
 *          Auto mode can be disabled at compile time (MODE_AUTO_ENABLED=0) to save
 *          flash memory on small boards. In this case, function always returns false
 *          since no modes require missions.
 * 
 * @return true Current mode is Auto and requires a mission
 * @return false Current mode doesn't require a mission, or Auto mode is disabled
 * 
 * @note Only Auto mode requires a mission in ArduCopter
 * @note Used by pre-arm checks to validate mission availability before arming
 * @note Conditional compilation: Returns false if MODE_AUTO_ENABLED not defined
 * @see Mode::auto_armed() for autonomous arming that may occur during mission execution
 * @see AP_Mission for mission storage and command execution
 * @see pre_arm_checks() for pre-arm validation using this function
 */
bool Copter::current_mode_requires_mission() const
{
#if MODE_AUTO_ENABLED
        return flightmode == &mode_auto;
#else
        return false;
#endif
}

/**
 * @brief Process RC (Radio Control) input from transmitter/receiver
 * 
 * @details Scheduled task that reads pilot inputs from the radio receiver and
 *          processes flight mode switch changes. This is a critical input path
 *          for pilot control of the vehicle.
 * 
 *          Execution Flow:
 *          1. read_radio() - Reads all RC channel PWM values from receiver
 *          2. read_mode_switch() - Checks 3-position flight mode switch and
 *             initiates mode changes if switch position has changed
 * 
 *          This function is called at 250Hz as defined in scheduler_tasks table
 *          to ensure responsive pilot input handling with minimal latency.
 * 
 * @note Called by AP_Scheduler at 250Hz (scheduler_tasks priority 3)
 * @see read_radio() for RC channel value processing
 * @see RC_Channels::read_mode_switch() for flight mode switching logic
 */
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

/**
 * @brief Update altitude control, throttle, and auto-arming status
 * 
 * @details Scheduled task that manages throttle-related states and altitude control
 *          parameters. This runs at a lower rate than the fast loop since throttle
 *          changes can be applied more gradually than attitude corrections.
 * 
 *          Key Functions:
 *          - update_throttle_mix(): Adjusts throttle_low_comp value which controls
 *            the priority between throttle response and attitude control during
 *            aggressive maneuvers
 *          - update_auto_armed(): Checks if vehicle should transition to auto-armed
 *            state based on throttle position and other conditions
 *          - Helicopter-specific updates (HELI_FRAME only):
 *            * heli_update_rotor_speed_targets(): Manages rotor RPM governor
 *            * heli_update_landing_swash(): Adjusts swashplate collective during landing
 *          - update_ground_effect_detector(): Compensates for increased lift when
 *            operating in ground effect (< 1 rotor diameter altitude)
 *          - update_ekf_terrain_height_stable(): Updates EKF with terrain height
 *            stability for better altitude hold near ground
 * 
 * @note Called by AP_Scheduler at 50Hz (scheduler_tasks priority 6)
 * @note Ground effect compensation improves altitude hold accuracy during takeoff/landing
 * @warning Throttle mix adjustments affect vehicle responsiveness - improper tuning
 *          can lead to altitude loss during aggressive maneuvers
 */
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}

/**
 * @brief Update battery monitor and compass readings
 * 
 * @details Scheduled task that reads battery voltage/current and compass (magnetometer)
 *          data. The execution order is critical: battery is read first because its
 *          voltage measurement is used for compass motor interference compensation.
 * 
 *          Battery Monitoring:
 *          - Reads voltage and current from all configured battery monitors
 *          - Updates fuel gauge estimates and failsafe thresholds
 *          - Provides data for power consumption logging
 * 
 *          Compass Processing:
 *          - set_throttle(): Provides current throttle level for CompassMot compensation
 *            (compensates for magnetic interference from motor currents)
 *          - set_voltage(): Provides battery voltage for interference compensation model
 *          - read(): Samples magnetometer and applies calibration + interference compensation
 * 
 *          CompassMot Interference Compensation:
 *          Motor currents create magnetic fields that interfere with compass readings.
 *          By providing throttle and voltage, the compass library can subtract the
 *          learned interference pattern, significantly improving heading accuracy
 *          during flight.
 * 
 * @note Called by AP_Scheduler at 10Hz (scheduler_tasks priority 15)
 * @note Battery must be read before compass for proper motor interference compensation
 * @see AP_BattMonitor::read() for battery monitoring implementation
 * @see Compass::read() for magnetometer reading and compensation
 * @warning Compass interference compensation requires proper CompassMot calibration
 *          for accurate heading, especially at high throttle
 */
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief High-rate logging at main loop frequency (typically 400Hz)
 * 
 * @details Scheduled task that logs time-critical flight data at the full loop rate
 *          for detailed performance analysis, tuning, and post-flight review. This
 *          function generates the highest-volume logs and significantly impacts
 *          SD card write performance and CPU load.
 * 
 *          Logging Selection (controlled by LOG_BITMASK parameter):
 *          
 *          MASK_LOG_ATTITUDE_FAST (bit set):
 *          - Logs vehicle attitude (roll, pitch, yaw) at loop rate
 *          - Logs rate controller outputs (rate setpoints and achieved rates)
 *          - Logs PID controller internals if PIDS bitmask also set
 *          - Skipped if flight mode handles its own attitude logging (e.g., autotune)
 * 
 *          MASK_LOG_FTN_FAST (bit set, harmonic notch enabled):
 *          - Logs harmonic notch filter state and center frequencies
 *          - Critical for diagnosing gyro noise and filter performance
 *          - Only logged when not using dedicated rate thread
 * 
 *          MASK_LOG_IMU_FAST (bit set):
 *          - Logs raw IMU data (gyro, accel) at loop rate
 *          - Essential for vibration analysis and sensor health monitoring
 * 
 *          Rate Thread Interaction:
 *          Rate and PID logging is skipped if using_rate_thread is true, as the
 *          dedicated rate controller thread logs these directly for better timing
 *          accuracy.
 * 
 * @note Called by AP_Scheduler at LOOP_RATE (typically 400Hz, scheduler_tasks priority 75)
 * @note High-rate logging can fill SD cards quickly - use selectively for troubleshooting
 * @note Log write performance depends on SD card speed and can affect loop timing
 * @warning Enabling too many high-rate logs simultaneously can cause scheduler overruns
 * @see should_log() for bitmask checking
 * @see Log_Write_Attitude() for attitude log format
 * @see Log_Write_Rate() for rate controller log format
 * @see Log_Write_PIDS() for PID internal state logging
 */
void Copter::loop_rate_logging()
{
   if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        if (!using_rate_thread) {
            Log_Write_Rate();
            Log_Write_PIDS(); // only logs if PIDS bitmask is set
        }
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_FTN_FAST) && !using_rate_thread) {
        AP::ins().write_notch_log_messages();
    }
#endif
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

/**
 * @brief Medium-rate logging at 10Hz for general flight data
 * 
 * @details Scheduled task that logs comprehensive flight state information at 10Hz,
 *          providing a good balance between data completeness and log file size.
 *          This rate captures most flight dynamics while being sustainable for
 *          long flights without filling SD cards excessively.
 * 
 *          Always Logged (regardless of bitmask):
 *          - AHRS attitude estimate with target attitude (10Hz baseline for analysis)
 * 
 *          Conditional Logging (controlled by LOG_BITMASK parameter):
 *          
 *          MASK_LOG_ATTITUDE_MED:
 *          - Logs attitude, rate, and PID data at 10Hz if not already logging at FAST rate
 *          - Provides medium-resolution flight dynamics without high-rate overhead
 * 
 *          MASK_LOG_MOTBATT or HELI_FRAME:
 *          - Logs motor outputs, mixer calculations, battery voltage/current effects
 *          - Always logged for helicopters due to complex rotor dynamics
 * 
 *          MASK_LOG_RCIN:
 *          - Logs pilot RC inputs (stick positions, switches)
 *          - Logs RSSI (signal strength) if available
 * 
 *          MASK_LOG_RCOUT:
 *          - Logs PWM outputs to motors and servos
 * 
 *          MASK_LOG_NTUN (navigation tuning):
 *          - Logs position controller data for GPS-based modes
 *          - Includes desired vs achieved position, velocity, acceleration
 * 
 *          MASK_LOG_IMU:
 *          - Logs vibration levels and clipping counts for health monitoring
 * 
 *          MASK_LOG_CTUN (control tuning):
 *          - Logs proximity sensor distances (obstacle detection)
 *          - Logs beacon positioning data if available
 * 
 *          MASK_LOG_ANY:
 *          - Logs winch state if winch is installed
 * 
 *          MASK_LOG_CAMERA:
 *          - Logs camera mount pointing and stabilization data
 * 
 *          Hierarchical Logging:
 *          Some logs are skipped at this rate if MASK_LOG_ATTITUDE_FAST is enabled,
 *          as they're being logged at higher rate by loop_rate_logging() or
 *          twentyfive_hz_logging(). This prevents duplicate data.
 * 
 * @note Called by AP_Scheduler at 10Hz (scheduler_tasks priority 114)
 * @note 10Hz logging is recommended for general flight analysis and tuning
 * @note EKF position logs move to 25Hz if ATTITUDE_FAST is enabled
 * @see should_log() for bitmask checking
 * @see Log_Write_Attitude() for attitude log details
 * @see pos_control->write_log() for navigation controller logging
 */
void Copter::ten_hz_logging_loop()
{
    // always write AHRS attitude at 10Hz
    ahrs.Write_Attitude(attitude_control->get_att_target_euler_rad() * RAD_TO_DEG);
    // log attitude controller data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
        if (!using_rate_thread) {
            Log_Write_Rate();
        }
    }
    if (!should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
    // log at 10Hz if PIDS bitmask is selected, even if no ATT bitmask is selected; logs at looprate if ATT_FAST and PIDS bitmask set
        if (!using_rate_thread) {
            Log_Write_PIDS();
        }
    }
    // log EKF attitude data always at 10Hz unless ATTITUDE_FAST, then do it in the 25Hz loop
    if (!should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if ((FRAME_CONFIG == HELI_FRAME) || should_log(MASK_LOG_MOTBATT)) {
        // always write motors log if we are a heli
        motors->Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
#if AP_RSSI_ENABLED
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
#endif
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
#if HAL_PROXIMITY_ENABLED
        g2.proximity.log();  // Write proximity sensor distances
#endif
#if AP_BEACON_ENABLED
        g2.beacon.log();
#endif
    }
#if AP_WINCH_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/**
 * @brief Intermediate-rate logging at 25Hz for high-resolution analysis
 * 
 * @details Scheduled task that logs selected high-rate data at 25Hz when full
 *          loop-rate logging would be excessive but 10Hz is insufficient.
 *          This rate is optimal for EKF analysis and FFT-based tuning.
 * 
 *          Logging Selection (controlled by LOG_BITMASK parameter):
 *          
 *          MASK_LOG_ATTITUDE_FAST:
 *          - Logs EKF position and velocity estimates at 25Hz
 *          - Moved here from 10Hz logging to capture higher-rate position dynamics
 *          - Essential for analyzing GPS position jumps and EKF innovations
 * 
 *          MASK_LOG_IMU (without MASK_LOG_IMU_FAST):
 *          - Logs processed IMU data (gyro, accel) at 25Hz
 *          - Intermediate rate suitable for vibration analysis without full-rate overhead
 *          - Skipped if IMU_FAST is enabled (already logging at loop rate)
 * 
 *          MASK_LOG_FTN_FAST (with gyro FFT enabled):
 *          - Logs gyro FFT analysis results for harmonic notch tuning
 *          - Identifies dominant vibration frequencies in real-time
 *          - Critical for optimizing harmonic notch filters on noisy vehicles
 * 
 *          Hierarchical Logging Strategy:
 *          This function captures data at an intermediate rate between the 10Hz
 *          baseline and the full loop rate (400Hz). It's activated when users
 *          enable ATTITUDE_FAST logging but provides selected data at this
 *          intermediate rate to balance resolution with log file size.
 * 
 * @note Called by AP_Scheduler at 25Hz (scheduler_tasks priority 117)
 * @note 25Hz rate is chosen to capture EKF updates (typically 10-50Hz) with margin
 * @note FFT analysis benefits from 25Hz logging to capture multi-rotor blade passage frequencies
 * @see Log_Write_EKF_POS() for EKF position log format
 * @see gyro_fft.write_log_messages() for FFT analysis logging
 */
void Copter::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU) && !(should_log(MASK_LOG_IMU_FAST))) {
        AP::ins().Write_IMU();
    }

#if HAL_GYROFFT_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        gyro_fft.write_log_messages();
    }
#endif
}
#endif  // HAL_LOGGING_ENABLED

/**
 * @brief Slow-rate housekeeping tasks running at 3Hz
 * 
 * @details Scheduled task that handles low-priority monitoring and tuning functions
 *          that don't require high-rate execution. Running at 3Hz reduces CPU load
 *          while still providing timely responses for these less time-critical tasks.
 * 
 *          Failsafe Monitoring:
 *          - failsafe_gcs_check(): Detects loss of ground station telemetry connection
 *            and triggers appropriate failsafe action (RTL or Land)
 *          - failsafe_terrain_check(): Monitors terrain data availability for terrain
 *            following modes, triggers failsafe if terrain data is lost
 *          - failsafe_deadreckon_check(): Detects extended periods of position estimation
 *            based only on dead reckoning (no GPS/external positioning), triggers failsafe
 * 
 *          In-Flight Adjustments:
 *          - tuning(): Processes transmitter-based tuning adjustments, allowing pilots
 *            to adjust PID gains and other parameters in flight using assigned RC channels
 * 
 *          Avoidance:
 *          - low_alt_avoidance(): Enables/disables obstacle avoidance based on altitude,
 *            typically disabling avoidance at very low altitudes to prevent ground detection
 *            interference during landing
 * 
 * @note Called by AP_Scheduler at 3Hz (scheduler_tasks priority 57)
 * @note Failsafe checks at this rate provide ~330ms response time
 * @see failsafe_gcs_check() for GCS timeout detection
 * @see failsafe_terrain_check() for terrain data monitoring
 * @see failsafe_deadreckon_check() for position estimation health
 * @warning Failsafe triggers can initiate autonomous mode changes - critical for safety
 */
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

    // check for deadreckoning failsafe
    failsafe_deadreckon_check();

    //update transmitter based in flight tuning
    tuning();

    // check if avoidance should be enabled based on alt
    low_alt_avoidance();
}

/**
 * @brief Encode vehicle state flags into a 32-bit bitmask for logging
 * 
 * @details Converts the ap (autopilot) state structure into a compact 32-bit bitmask
 *          for efficient logging to dataflash. Each bit represents one boolean state
 *          flag (armed, simple mode, land complete, etc.). This encoding allows the
 *          entire vehicle state to be logged in a single 32-bit value.
 * 
 *          The ap structure contains critical state flags:
 *          - ap.armed: Motors are armed and can spin
 *          - ap.simple_mode: Simple or super simple mode active
 *          - ap.land_complete: Vehicle has detected landing
 *          - ap.pre_arm_check: Pre-arm checks have passed
 *          - ap.auto_armed: Autonomous arming logic has enabled motors
 *          - ap.using_interlock: Using interlock switch (typically for heli)
 *          - ap.motor_test: Motor test mode active
 *          - ap.initialised: Initialization complete
 *          - ap.land_complete_maybe: Possibly landed (transitional state)
 *          - ap.throttle_zero: Throttle stick at zero position
 *          - ap.compass_mot: Compass motor interference compensation active
 *          - ap.usb_connected: USB connection detected
 *          - ap.rc_receiver_present: RC receiver detected and providing input
 *          - ap.new_radio_frame: New RC frame received (consumed by simple mode)
 * 
 *          Implementation:
 *          Treats the ap structure as an array of boolean bytes and sets corresponding
 *          bits in the return value. Bit N is set if byte N of the ap structure is true.
 * 
 *          Historical Note:
 *          This replaces a legacy global variable approach. The bitmask encoding provides
 *          efficient storage in dataflash logs and easy decoding for post-flight analysis.
 * 
 * @return uint32_t Bitmask with each bit representing one state flag from ap structure
 * 
 * @note Called by one_hz_loop() for periodic state logging
 * @note Bit positions correspond to byte offsets in ap structure
 * @note Log analysis tools decode this bitmask to display individual state flags
 * @see one_hz_loop() where this is logged as LogDataID::AP_STATE
 * @see Copter.h for ap structure definition and complete list of state flags
 */
uint32_t Copter::ap_value() const
{
    uint32_t ret = 0;

    const bool *b = (const bool *)&ap;
    for (uint8_t i=0; i<sizeof(ap); i++) {
        if (b[i]) {
            ret |= 1U<<i;
        }
    }

    return ret;
}

/**
 * @brief Very low rate housekeeping tasks running at 1Hz
 * 
 * @details Scheduled task that handles lowest-priority periodic maintenance functions
 *          that only need to execute once per second. This includes configuration updates,
 *          status logging, and adaptive tuning adjustments that don't require high rates.
 * 
 *          Logging:
 *          - Logs current AP state bitmask (armed, simple mode, land complete, etc.)
 *          - Logs terrain data if terrain following is active
 * 
 *          Disarmed Configuration Updates (safety-critical timing):
 *          When motors are disarmed, allows safe configuration changes:
 *          - update_using_interlock(): Updates interlock state for heli motors
 *          - set_frame_class_and_type(): Applies any user parameter changes to motor
 *            frame configuration (quad, hex, octa, X, +, etc.)
 *          - update_throttle_range(): Updates PWM output ranges for throttle channels
 * 
 *          Servo Management:
 *          - enable_aux_servos(): Enables auxiliary servo outputs and updates function
 *            assignments (camera mounts, grippers, etc.)
 * 
 *          Status Reporting:
 *          - Updates ADSB transponder with flying state
 *          - Updates AP_Notify flying flag for LED/buzzer indicators
 * 
 *          Adaptive Filter Tuning:
 *          - Updates harmonic notch filter sample rates based on measured loop rate
 *          - Adjusts PID controller notch filters for gyro noise rejection
 *          - Applies to attitude control, position control, and custom control PIDs
 * 
 *          Rate Thread Management (advanced feature):
 *          - Attempts to start dedicated high-priority thread for rate controller
 *          - Separates rate control from main loop for improved timing consistency
 *          - Only enabled if fast sample window feature is compiled in
 * 
 * @note Called by AP_Scheduler at 1Hz (scheduler_tasks priority 81)
 * @note Configuration changes only occur when disarmed for safety
 * @note Rate thread creation is attempted once and cached in started_rate_thread flag
 * @warning Frame configuration changes while armed could cause instability - only
 *          applied when disarmed
 * @see ap_value() for AP state bitmask encoding
 * @see AP_Motors::set_frame_class_and_type() for motor configuration
 */
void Copter::one_hz_loop()
{
#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap_value());
    }
#endif

    if (!motors->armed()) {
        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->update_throttle_range();
#endif
    }

    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // log terrain data
    terrain_logging();
#endif

#if HAL_ADSB_ENABLED
    adsb.set_is_flying(!ap.land_complete);
#endif

    AP_Notify::flags.flying = !ap.land_complete;

    // slowly update the PID notches with the average loop rate
    if (!using_rate_thread) {
        attitude_control->set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    }
    pos_control->get_accel_U_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#if AC_CUSTOMCONTROL_MULTI_ENABLED
    custom_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
#endif

#if AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED
    // see if we should have a separate rate thread
    if (!started_rate_thread && get_fast_rate_type() != FastRateType::FAST_RATE_DISABLED) {
        if (hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&Copter::rate_controller_thread, void),
                                         "rate",
                                         1536, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
            started_rate_thread = true;
        } else {
            AP_BoardConfig::allocation_error("rate thread");
        }
    }
#endif
}

/**
 * @brief Initialize simple and super simple mode heading references
 * 
 * @details Captures the current vehicle heading to establish the reference frame for
 *          simple mode and super simple mode control transformations. This function
 *          must be called when the pilot activates simple or super simple mode to
 *          "lock in" the orientation reference.
 * 
 *          Simple Mode Reference:
 *          Captures current vehicle heading (yaw) as the reference "north". After
 *          initialization, pilot stick inputs are interpreted relative to this fixed
 *          heading rather than the vehicle's current orientation. Forward stick always
 *          moves the vehicle in the direction it was facing at initialization.
 * 
 *          Super Simple Mode Reference:
 *          Initializes the home bearing reference to 180 degrees from current heading.
 *          In super simple mode, forward stick always moves toward home and back stick
 *          always moves away from home, regardless of vehicle orientation.
 * 
 *          Precomputed Trigonometry:
 *          Stores cos(yaw) and sin(yaw) values to avoid repeated trigonometric
 *          calculations in the high-rate update_simple_mode() function. These values
 *          are used for efficient 2D rotation matrix operations on pilot inputs.
 * 
 *          Logging:
 *          Records the initialization heading to dataflash for post-flight analysis
 *          and troubleshooting of simple mode behavior.
 * 
 * @note Called when pilot activates simple or super simple mode via flight mode switch
 * @note Simple mode heading is fixed at initialization; super simple updates dynamically
 * @note Precomputed trig values provide ~10x speedup vs computing per frame
 * @see update_simple_mode() for input transformation using these references
 * @see update_super_simple_bearing() for dynamic super simple heading updates
 * @see SimpleMode enum for mode types (NONE, SIMPLE, SUPERSIMPLE)
 */
void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

#if HAL_LOGGING_ENABLED
    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
#endif
}

/**
 * @brief Transform pilot stick inputs for simple or super simple mode control
 * 
 * @details Implements the coordinate transformation that makes simple mode and super
 *          simple mode work. This function rotates pilot roll/pitch inputs from the
 *          pilot's reference frame into the vehicle's current body frame, allowing
 *          intuitive "orientation-independent" control for beginner pilots.
 * 
 *          Early Exit Conditions:
 *          - Returns immediately if not in simple mode (SimpleMode::NONE)
 *          - Returns if no new RC frame received (prevents duplicate processing)
 *          - Returns if RC input invalid (during binding or signal loss)
 * 
 *          Simple Mode Transformation:
 *          When SimpleMode::SIMPLE is active:
 *          1. Rotates pilot inputs by -initial_heading (captured at init_simple_bearing)
 *          2. This transforms inputs from the "locked" reference frame to north-facing
 *          3. Forward stick always moves in the direction vehicle faced at mode entry
 *          4. Makes it easier for beginners who lose track of vehicle orientation
 * 
 *          Super Simple Mode Transformation:
 *          When SimpleMode::SUPERSIMPLE is active:
 *          1. Rotates pilot inputs by -(home_bearing + 180°)
 *          2. Forward stick always moves vehicle toward home position
 *          3. Back stick always moves vehicle away from home
 *          4. Home bearing updates dynamically as vehicle moves (see update_super_simple_bearing)
 *          5. Especially helpful for line-of-sight flying at distance
 * 
 *          Two-Stage Rotation Process:
 *          Stage 1: Pilot frame → North frame (using simple or super simple reference)
 *          Stage 2: North frame → Vehicle body frame (using current yaw)
 * 
 *          Mathematical Implementation:
 *          Uses 2D rotation matrices with precomputed sin/cos values for efficiency:
 *          [rollx ]   [cos -sin] [roll_in ]
 *          [pitchx] = [sin  cos] [pitch_in]
 * 
 *          Performance Optimization:
 *          Precomputed trig values (simple_cos_yaw, etc.) from init_simple_bearing()
 *          eliminate expensive trigonometric calculations from the high-rate loop.
 * 
 * @note Called from main loop when new RC frame available and simple mode active
 * @note Modifies channel_roll and channel_pitch control_in values in-place
 * @note Yaw control is unaffected - pilot still controls yaw directly
 * @note ap.new_radio_frame flag prevents double-processing of same RC frame
 * @warning Coordinate transformations can feel unintuitive if mode activated while moving
 * @see init_simple_bearing() for reference frame initialization
 * @see update_super_simple_bearing() for dynamic home bearing updates
 * @see rc_loop() which calls this function
 */
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    // avoid processing bind-time RC values:
    if (!rc().has_valid_input()) {
        return;
    }

    if (simple_mode == SimpleMode::SIMPLE) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

/**
 * @brief Update super simple mode heading reference based on changing home bearing
 * 
 * @details Dynamically adjusts the heading reference used by super simple mode as the
 *          vehicle moves and its bearing to home changes. This keeps the "forward =
 *          toward home" behavior accurate as the vehicle flies around the home position.
 * 
 *          Update Conditions (unless force_update is true):
 *          - Only updates if currently in SimpleMode::SUPERSIMPLE
 *          - Only updates if distance from home > SUPER_SIMPLE_RADIUS (typically 10m)
 *            (prevents erratic behavior when circling very close to home)
 *          - Only updates if bearing has changed by >= 5 degrees (500 centidegrees)
 *            (hysteresis prevents jitter from GPS noise)
 * 
 *          Bearing Calculation:
 *          Computes angle from vehicle to home position using GPS coordinates. The
 *          reference heading is set to (home_bearing + 180°) so that forward stick
 *          moves toward home (opposite of the bearing angle which points from home
 *          to vehicle).
 * 
 *          Trigonometry Precomputation:
 *          Like init_simple_bearing(), this function precomputes cos() and sin() of
 *          the new reference angle and caches them in super_simple_cos_yaw and
 *          super_simple_sin_yaw. These cached values are used by update_simple_mode()
 *          for efficient coordinate transformations.
 * 
 *          Force Update Mode:
 *          When force_update=true, bypasses all conditional checks and updates
 *          immediately. Used during mode initialization and position resets.
 * 
 *          Dead Zone Near Home:
 *          SUPER_SIMPLE_RADIUS creates a dead zone around home where bearing doesn't
 *          update. This prevents the heading reference from spinning rapidly when
 *          hovering directly over home position, which would cause confusing control
 *          reversals.
 * 
 *          Hysteresis for Stability:
 *          The 5-degree threshold prevents the reference heading from updating on
 *          every small GPS position change. This provides smoother, more predictable
 *          control behavior, especially important with GPS position jitter.
 * 
 * @param[in] force_update If true, bypasses all conditions and updates immediately
 * 
 * @note Should be called after home_bearing() has been updated with latest GPS position
 * @note Called periodically from navigation update functions when super simple mode active
 * @note SUPER_SIMPLE_RADIUS is typically 10 meters (1000cm)
 * @note Bearing hysteresis of 5° (500 centidegrees) balances responsiveness vs stability
 * @see update_simple_mode() which uses the updated reference for input transformation
 * @see init_simple_bearing() for initial reference setup
 * @see home_bearing() for bearing calculation from GPS position
 */
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (simple_mode != SimpleMode::SUPERSIMPLE) {
            return;
        }
        if (home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    const int32_t bearing = home_bearing();

    // check the bearing to home has changed by at least 5 degrees
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    super_simple_last_bearing = bearing;
    const float angle_rad = cd_to_rad(super_simple_last_bearing + 18000);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

/**
 * @brief Update AHRS (Attitude and Heading Reference System) state estimation
 * 
 * @details Fast loop task that updates the AHRS with the latest IMU measurements
 *          to compute vehicle attitude, heading, and position estimates. This is
 *          one of the most critical tasks as accurate state estimation is fundamental
 *          to stable flight control.
 * 
 *          The 'true' parameter tells AHRS to skip its internal INS update because
 *          we've already updated the INS in an earlier FAST_TASK (AP_InertialSensor::update).
 *          This prevents duplicate processing and ensures optimal performance.
 * 
 *          AHRS Update Process:
 *          - Runs EKF prediction step using latest gyro/accel data
 *          - Integrates GPS, compass, barometer measurements if available
 *          - Updates attitude quaternion, euler angles, rotation matrices
 *          - Estimates position, velocity, and biases
 *          - Detects and flags estimation failures for failsafe system
 * 
 * @note Called in fast loop (FAST_TASK) at main loop rate (typically 400Hz)
 * @note This function is performance-critical - EKF is computationally expensive
 * @see AP_InertialSensor::update() which must be called first to update IMU data
 * @see AP_AHRS_NavEKF::update() for EKF state estimation implementation
 * @warning AHRS update failure can trigger EKF failsafe - critical for flight safety
 */
void Copter::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in FAST_TASK.
    ahrs.update(true);
}

/**
 * @brief Update barometric altitude and log control tuning data
 * 
 * @details Scheduled task that reads barometer pressure measurements and logs
 *          control loop performance data for analysis and tuning. The barometer
 *          provides altitude estimates that complement GPS and are critical for
 *          altitude hold and terrain following.
 * 
 *          Barometer Processing:
 *          - Reads raw pressure from barometer sensor(s)
 *          - Applies temperature compensation and calibration
 *          - Converts pressure to altitude estimate
 *          - Feeds altitude data to EKF for sensor fusion
 * 
 *          Control Tuning Logging:
 *          When MASK_LOG_CTUN is enabled, logs:
 *          - Desired vs actual altitude, climb rate, throttle
 *          - Position controller states (P, I, D terms)
 *          - Attitude controller states
 *          - Navigation targets and errors
 * 
 *          Harmonic Notch and FFT Logging:
 *          If not logging at fast rate (MASK_LOG_FTN_FAST), also logs:
 *          - Harmonic notch filter state (for vibration rejection)
 *          - Gyro FFT analysis (for filter tuning and health monitoring)
 * 
 * @note Called by AP_Scheduler at 10Hz (scheduler_tasks priority 42)
 * @note Control tuning logs are essential for PID tuning and diagnosing flight issues
 * @see read_barometer() for barometer sensor reading
 * @see Log_Write_Control_Tuning() for logged control data fields
 * @see AP_InertialSensor::write_notch_log_messages() for notch filter logging
 */
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        if (!should_log(MASK_LOG_FTN_FAST)) {
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
            AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
            gyro_fft.write_log_messages();
#endif
        }
    }
#endif
}

/**
 * @brief Get distance to active waypoint in meters
 * 
 * @details Queries the current flight mode for the distance to its active navigation
 *          target waypoint. This provides a mode-agnostic interface for telemetry and
 *          navigation monitoring.
 * 
 *          The meaning of "active waypoint" varies by mode:
 *          - Auto mode: Current mission waypoint
 *          - Guided mode: Current guided target position
 *          - RTL mode: Home position or rally point
 *          - Loiter mode: Loiter center point
 *          - Land mode: Target landing point
 *          - Other modes: May return 0 or undefined
 * 
 *          This function is primarily used by MAVLink telemetry to populate the
 *          NAV_CONTROLLER_OUTPUT message, which displays waypoint progress on ground
 *          control stations.
 * 
 * @param[out] distance Distance to active waypoint in meters
 * 
 * @return true Always returns true (distance is always available, may be 0)
 * 
 * @note Called by GCS_MAVLINK_Copter::send_nav_controller_output() for telemetry
 * @note Returns 0 for modes without an active navigation target
 * @see get_wp_bearing_deg() for direction to waypoint
 * @see get_wp_crosstrack_error_m() for path tracking error
 */
bool Copter::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance_m();
    return true;
}

/**
 * @brief Get bearing to active waypoint in degrees
 * 
 * @details Queries the current flight mode for the bearing (compass direction) to its
 *          active navigation target waypoint. Returns bearing in degrees clockwise from
 *          north (0-360°).
 * 
 *          The bearing represents the direct compass heading from vehicle's current
 *          position to the target waypoint. This is distinct from the desired heading,
 *          which may differ due to crosstrack correction or loiter circles.
 * 
 *          Used by ground control stations to display:
 *          - Navigation arrow showing waypoint direction
 *          - Heading error between vehicle heading and waypoint direction
 *          - Progress indicators for autonomous missions
 * 
 * @param[out] bearing Compass bearing to waypoint in degrees (0-360°, north = 0°)
 * 
 * @return true Always returns true (bearing is always available, may be 0)
 * 
 * @note Called by GCS_MAVLINK_Copter::send_nav_controller_output() for telemetry
 * @note Returns 0 for modes without an active navigation target
 * @note Bearing is in earth frame (NED), not relative to vehicle heading
 * @see get_wp_distance_m() for distance to waypoint
 * @see get_wp_crosstrack_error_m() for path tracking error
 */
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing_deg();
    return true;
}

/**
 * @brief Get crosstrack error for current navigation path in meters
 * 
 * @details Calculates the perpendicular distance between the vehicle's current position
 *          and the desired path between waypoints. Positive values indicate the vehicle
 *          is right of the desired path, negative values indicate left of path.
 * 
 *          Crosstrack error is meaningful for:
 *          - Auto mode: Distance from ideal straight line between consecutive waypoints
 *          - Guided mode: May indicate path deviation if path tracking active
 *          - Other navigation modes with defined paths
 * 
 *          Crosstrack error is used for:
 *          - Path following: Controller adjusts heading to reduce crosstrack error
 *          - Telemetry display: Shows navigation accuracy on ground station
 *          - Performance analysis: Post-flight evaluation of navigation precision
 * 
 *          Unit Conversion:
 *          Mode returns crosstrack error in centimeters (ArduPilot internal units).
 *          This function converts to meters (×0.01) for external interfaces like MAVLink.
 * 
 * @param[out] xtrack_error Crosstrack error in meters (positive = right of path)
 * 
 * @return true Always returns true (error value is always available, may be 0)
 * 
 * @note Called by GCS_MAVLINK_Copter::send_nav_controller_output() for telemetry
 * @note Returns 0 for modes without path following (Stabilize, AltHold, Loiter center)
 * @note Internal calculation in cm, converted to meters for external API
 * @see get_wp_distance_m() for distance to next waypoint
 * @see get_wp_bearing_deg() for direction to next waypoint
 */
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

/**
 * @brief Get target angular velocities in earth frame for external devices
 * 
 * @details Retrieves the vehicle's current attitude rate targets in earth-fixed reference
 *          frame (NED frame: North-East-Down). These represent the commanded angular
 *          velocities the vehicle is currently trying to achieve.
 * 
 *          Earth Frame Angular Rates:
 *          - X-axis (North): Pitch rate in rad/s (nose up positive)
 *          - Y-axis (East): Roll rate in rad/s (right wing down positive)
 *          - Z-axis (Down): Yaw rate in rad/s (clockwise looking down positive)
 * 
 *          The Z-axis (yaw rate) component is particularly important for camera gimbals
 *          that need to compensate for vehicle rotation to maintain stable pointing.
 *          Some advanced gimbals use all three axes for full rate feedforward compensation.
 * 
 *          Use Cases:
 *          - Camera gimbals: Feedforward rate compensation for smooth tracking
 *          - Payload stabilization: External stabilization systems
 *          - Telemetry: Ground station displays of vehicle dynamics
 *          - External control: Rate monitoring for companion computers
 * 
 *          Safety Behavior:
 *          Returns zero vector when landed or disarmed to indicate the vehicle is not
 *          actively controlling attitude. This prevents gimbals from responding to
 *          spurious rate commands during ground operations.
 * 
 *          Coordinate Frame:
 *          Earth frame (NED) is used rather than body frame because external devices
 *          typically need rates referenced to a fixed coordinate system. Body frame
 *          rates would be ambiguous due to changing vehicle orientation.
 * 
 * @param[out] rate_ef_targets Vector3f containing angular velocity targets:
 *                              .x = pitch rate in rad/s (earth frame)
 *                              .y = roll rate in rad/s (earth frame)  
 *                              .z = yaw rate in rad/s (earth frame)
 * 
 * @return true Always returns true (targets are always available, may be zero)
 * 
 * @note Returns zero vector if ap.land_complete is true (landed or disarmed)
 * @note Primarily used by camera mount/gimbal systems for rate feedforward
 * @note Values are in earth frame (NED), not body frame
 * @note Z-axis component (yaw rate) is most commonly used by gimbals
 * @see AC_AttitudeControl::get_rate_ef_targets() for rate target calculation
 * @see AP_Mount for gimbal integration that uses these rate targets
 */
bool Copter::get_rate_ef_targets(Vector3f& rate_ef_targets) const
{
    // always returns zero vector if landed or disarmed
    if (copter.ap.land_complete) {
        rate_ef_targets.zero();
    } else {
        rate_ef_targets = attitude_control->get_rate_ef_targets();
    }
    return true;
}

/**
 * @brief Constructor for the main Copter vehicle class
 * 
 * @details Initializes the Copter object with default values for flight control state,
 *          filters, and references to configuration parameters. This constructor is
 *          called once at program startup before setup() executes.
 * 
 *          Member Initialization:
 *          - flight_modes: Pointer to flight mode array (modes 1-6) from parameters
 *          - pos_variance_filt: Low-pass filter for EKF position variance (failsafe detection)
 *          - vel_variance_filt: Low-pass filter for EKF velocity variance (failsafe detection)
 *          - flightmode: Initial flight mode pointer set to Stabilize (safe default)
 *          - simple_cos_yaw/simple_sin_yaw: Cached trigonometric values for Simple mode
 *          - super_simple_cos_yaw/super_simple_sin_yaw: Cached values for Super Simple mode
 *          - land_accel_ef_filter: Low-pass filter for landing detection acceleration
 *          - rc_throttle_control_in_filter: Smoothing filter for throttle input
 *          - param_loader: Parameter table loader for AP_Param system
 * 
 *          Singleton Pattern:
 *          After construction, the global 'copter' instance becomes the singleton
 *          vehicle object. It inherits from AP_Vehicle and provides the Copter-specific
 *          implementation of all virtual methods.
 * 
 *          Initialization Sequence:
 *          1. Static construction (this constructor) - Initialize member variables
 *          2. HAL initialization - Hardware Abstraction Layer setup
 *          3. setup() - Full vehicle initialization (sensors, libraries, modes)
 *          4. loop() - Main execution loop begins
 * 
 * @note Constructor executes before any hardware is initialized or parameters loaded
 * @note All flight mode objects (mode_stabilize, mode_loiter, etc.) are constructed
 *       before this constructor runs
 * @see Copter::setup() for full vehicle initialization sequence
 * @see AP_Vehicle base class for inherited vehicle framework
 */
Copter::Copter(void)
    :
    flight_modes(&g.flight_mode1),
    pos_variance_filt(FS_EKF_FILT_DEFAULT),
    vel_variance_filt(FS_EKF_FILT_DEFAULT),
    flightmode(&mode_stabilize),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    param_loader(var_info)
{
}

/**
 * @brief Global Copter singleton instance
 * 
 * @details This is the single global instance of the Copter class that represents
 *          the entire vehicle. All ArduPilot code accesses vehicle state and methods
 *          through this singleton.
 * 
 *          Singleton Pattern Implementation:
 *          - 'copter' is the concrete Copter instance
 *          - 'vehicle' is an AP_Vehicle reference pointing to copter
 *          - Code can use either 'copter' (Copter-specific) or 'vehicle' (generic)
 * 
 *          The AP_HAL_MAIN_CALLBACKS macro registers the copter instance with the HAL,
 *          establishing it as the object whose setup() and loop() methods will be
 *          called by the HAL's main() function.
 * 
 * @note This singleton is constructed at program startup before main() executes
 * @see Copter::Copter() constructor for initialization
 * @see AP_Vehicle base class for vehicle framework interface
 */
Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
