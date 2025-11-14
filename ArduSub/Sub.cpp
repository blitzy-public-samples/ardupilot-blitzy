/**
 * @file Sub.cpp
 * @brief Main ArduSub vehicle implementation and scheduler
 * 
 * @details This file contains the core implementation of the ArduSub underwater vehicle class,
 *          including:
 *          - Sub singleton instantiation and initialization
 *          - Main scheduler task table defining all periodic operations
 *          - Fast loop and periodic loop functions for vehicle control
 *          - Sensor update and logging functions
 *          - Failsafe checking and health monitoring
 *          
 *          ArduSub is an underwater Remotely Operated Vehicle (ROV) autopilot system
 *          that provides depth hold, position hold, and autonomous navigation capabilities
 *          for submersible vehicles. The architecture is built around a high-frequency
 *          fast loop (typically 400Hz) for attitude control and lower frequency periodic
 *          tasks for sensor updates, logging, and GCS communication.
 * 
 * @note This file is safety-critical for underwater vehicle operations
 * @warning Modifications to scheduler timing or loop functions can affect vehicle stability
 * 
 * Source: ArduSub/Sub.cpp:1-538
 */

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

#include "Sub.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/**
 * @brief Constructor for main Sub vehicle class
 * 
 * @details Initializes the ArduSub underwater vehicle singleton and all subsystems.
 *          This constructor performs the following critical initialization sequence:
 * 
 *          **Flight Mode Initialization**:
 *          - Conditionally initializes flight_modes array if AP_SUB_RC_ENABLED
 *          - Otherwise sets default control_mode to MANUAL
 * 
 *          **Motor and Control System Initialization**:
 *          - Initializes motors library with MAIN_LOOP_RATE (typically 400Hz)
 *          - Sets default auto yaw mode to AUTO_YAW_LOOK_AT_NEXT_WP
 * 
 *          **Navigation System Initialization**:
 *          - Initializes inertial_nav with AHRS reference for position estimation
 *          - Creates ahrs_view with ROTATION_NONE (no frame rotation for underwater vehicles)
 * 
 *          **Control Loop Initialization**:
 *          - Initializes attitude_control with ahrs_view, aparm, and motors
 *          - Initializes pos_control (position controller) with dependencies
 *          - Initializes wp_nav (waypoint navigation) with full control chain
 *          - Initializes loiter_nav for station-keeping
 *          - Initializes circle_nav for circular navigation patterns
 * 
 *          **Parameter and Mode Management**:
 *          - Initializes param_loader with vehicle parameter info
 *          - Sets default flightmode pointer to mode_manual
 *          - Initializes auto_mode state to Auto_WP (waypoint following)
 *          - Initializes guided_mode state to Guided_WP
 * 
 *          **Singleton Pattern Enforcement**:
 *          - Checks that only one Sub instance exists system-wide
 *          - Panics (halts system) if duplicate instantiation attempted
 *          - This is critical for preventing multiple vehicle control instances
 * 
 *          **Failsafe Initialization**:
 *          - Sets failsafe.pilot_input = true (assume input present at boot)
 * 
 * @note All subsystems are initialized in the member initializer list for efficiency
 * @note Initialization order matches declaration order in Sub.h to avoid warnings
 * @warning This constructor must complete before scheduler starts running tasks
 * @warning Singleton enforcement prevents multiple vehicle instances - critical for safety
 * 
 * Source: ArduSub/Sub.cpp:27-54
 */
Sub::Sub()
    :

#if AP_SUB_RC_ENABLED
          flight_modes(&g.flight_mode1),  // Flight mode array for RC mode switching
#else
          control_mode(Mode::Number::MANUAL),  // Default to MANUAL mode when RC disabled
#endif
          motors(MAIN_LOOP_RATE),  // Motor library initialized with main loop rate (400Hz)
          auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP),  // Default auto yaw to face next waypoint
          inertial_nav(ahrs),  // Inertial navigation using AHRS for position estimation
          ahrs_view(ahrs, ROTATION_NONE),  // AHRS view with no rotation for Sub
          attitude_control(ahrs_view, aparm, motors),  // Attitude controller initialization
          pos_control(ahrs_view, motors, attitude_control),  // Position controller with dependencies
          wp_nav(ahrs_view, pos_control, attitude_control),  // Waypoint navigation controller
          loiter_nav(ahrs_view, pos_control, attitude_control),  // Loiter/station-keeping navigation
          circle_nav(ahrs_view, pos_control),  // Circle navigation for search patterns
          param_loader(var_info),  // Parameter loading system
          flightmode(&mode_manual),  // Default flight mode pointer
          auto_mode(Auto_WP),  // Auto mode state: waypoint following
          guided_mode(Guided_WP)  // Guided mode state: waypoint control
{
    // Initialize failsafe state - assume pilot input is present at boot
    failsafe.pilot_input = true;
    
    // Enforce singleton pattern - only one Sub instance allowed
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one Sub");
    }
    _singleton = this;
}

/**
 * @brief Scheduler task macro for Sub class methods
 * @details Convenience macro that wraps SCHED_TASK_CLASS specifically for Sub methods
 * @param func Sub class method name to schedule
 * @param rate_hz Execution rate in Hertz
 * @param max_time_micros Expected execution time in microseconds
 * @param priority Task priority (0-255, lower = higher priority)
 */
#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros, priority)

/**
 * @brief Fast task macro for highest priority Sub methods
 * @details Convenience macro for tasks that run at main loop rate (typically 400Hz)
 * @param func Sub class method name to run at fast rate
 * @warning Fast tasks must complete quickly to avoid affecting control loop timing
 */
#define FAST_TASK(func) FAST_TASK_CLASS(Sub, &sub, func)

/**
 * @brief ArduSub main scheduler task table
 * 
 * @details Defines all periodic tasks for the ArduSub vehicle, ordered by priority.
 *          This table is interleaved with the AP_Vehicle base class task table to
 *          determine final execution order.
 * 
 *          **Task Execution Framework**:
 *          - FAST_TASK entries run at MAIN_LOOP_RATE (typically 400Hz)
 *          - SCHED_TASK entries run at specified rate_hz
 *          - Tasks execute in priority order (lower number = higher priority)
 *          - Scheduler ensures tasks don't overrun their time budget
 * 
 *          **Critical Fast Loop Tasks** (MAIN_LOOP_RATE):
 *          1. INS update - Populate current gyro/accel data
 *          2. Rate controller - Execute angular rate control laws
 *          3. Motor output - Send PWM commands to motors
 *          4. AHRS/EKF update - State estimation and sensor fusion
 *          5. Inertial navigation - Position/velocity estimation
 *          6. EKF yaw reset check - Detect and handle yaw resets
 *          7. Flight mode update - Execute current mode control logic
 *          8. Home position update - Maintain home location from EKF
 *          9. Surface/bottom detection - Monitor depth boundaries
 * 
 *          **Periodic Tasks** (various rates):
 *          - 50Hz: Failsafe checks, RC input, actuator updates
 *          - 20Hz: Rangefinder updates
 *          - 10Hz: Battery/compass, altitude, auxiliary RC, turn counter
 *          - 3Hz: Leak detection, pressure/temperature checks, GCS failsafe
 *          - 1Hz: Pre-arm checks, logging, terrain updates
 *          - 400Hz: GCS receive/send, logger periodic tasks
 * 
 * @note All entries must be ordered by priority for correct scheduling
 * @note Task timing budgets (max_time_micros) are monitored for overruns
 * @warning Modifying task rates or priorities can affect vehicle stability
 * @warning Fast loop tasks are safety-critical and must execute within time budget
 * 
 * **SCHED_TASK Macro Arguments**:
 * - func: Static function or method name
 * - rate_hz: Execution rate in Hertz
 * - max_time_micros: Expected execution time in microseconds
 * - priority: 0-255 (lower = higher priority)
 * 
 * **SCHED_TASK_CLASS Macro Arguments**:
 * - Class: Class name of the method
 * - instance: Pointer to object instance
 * - method: Method name to call
 * - rate_hz: Execution rate in Hertz
 * - max_time_micros: Expected execution time in microseconds
 * - priority: 0-255 (lower = higher priority)
 * 
 * Source: ArduSub/Sub.cpp:84-166
 */

const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    // ========== FAST LOOP TASKS (MAIN_LOOP_RATE, typically 400Hz) ==========
    // These tasks are safety-critical and must complete within the loop period
    
    // Update INS immediately to get current gyro/accel data populated (highest priority)
    FAST_TASK_CLASS(AP_InertialSensor, &sub.ins, update),
    
    // Run low level rate controllers that only require IMU data
    // Executes PID control for angular rates (roll/pitch/yaw rate)
    FAST_TASK(run_rate_controller),
    
    // Send PWM outputs to motors/thrusters immediately after rate controller
    FAST_TASK(motors_output),
    
    // Run EKF state estimator - fuses sensor data for attitude/position (computationally expensive)
    FAST_TASK(read_AHRS),
    
    // Update inertial navigation (position and velocity estimation from accelerometers)
    FAST_TASK(read_inertia),
    
    // Check if EKF has reset target heading and update controllers accordingly
    FAST_TASK(check_ekf_yaw_reset),
    
    // Run the flight mode attitude/position controllers (mode-specific control logic)
    FAST_TASK(update_flight_mode),
    
    // Update home position from EKF if necessary (maintains home location)
    FAST_TASK(update_home_from_EKF),
    
    // Check if vehicle has reached surface or bottom depth limits
    FAST_TASK(update_surface_and_bottom_detector),
    
#if HAL_MOUNT_ENABLED
    // Camera gimbal mount fast update for smooth tracking
    FAST_TASK_CLASS(AP_Mount, &sub.camera_mount, update_fast),
#endif

    // ========== 50Hz TASKS ==========
    // Failsafe checks, RC input processing, actuator updates
    SCHED_TASK(fifty_hz_loop,         50,     75,   3),
    
#if AP_SUB_RC_ENABLED
    // Read RC radio input and process mode switches (50Hz for responsive input)
    SCHED_TASK(rc_loop,              50,    130,  3),
#endif
    
    // Update GPS position and status (50Hz)
    SCHED_TASK_CLASS(AP_GPS, &sub.gps, update, 50, 200,   6),
    
#if AP_OPTICALFLOW_ENABLED
    // Update optical flow sensor for velocity estimation (200Hz for high-rate sensor)
    SCHED_TASK_CLASS(AP_OpticalFlow,          &sub.optflow,             update,         200, 160,   9),
#endif
    
    // ========== 10-20Hz TASKS ==========
    // Sensor updates at moderate rates
    
    // Read battery voltage/current and compass (10Hz, compass read after battery for motor interference compensation)
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    
    // Update rangefinder/sonar for obstacle detection and terrain following (20Hz)
    SCHED_TASK(read_rangefinder,      20,    100,  15),
    
    // Update altitude from barometer and log control tuning data (10Hz)
    SCHED_TASK(update_altitude,       10,    100,  18),
    
#if AP_SUB_RC_ENABLED
    // Read auxiliary RC channel functions (10Hz)
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&sub.g2.rc_channels, read_aux_all, 10,  50,  18),
#endif
    
    // ========== 1-3Hz TASKS ==========
    // Slower periodic checks and updates
    
    // 3Hz loop: leak detection, failsafe checks (pressure, temperature, GCS, terrain)
    SCHED_TASK(three_hz_loop,          3,     75,  21),
    
    // Update turn counter for rotation tracking (10Hz)
    SCHED_TASK(update_turn_counter,   10,     50,  24),
    
    // 1Hz loop: pre-arm checks, notify updates, motor range updates, terrain logging
    SCHED_TASK(one_hz_loop,            1,    100,  33),
    
    // ========== HIGH-FREQUENCY COMMUNICATION ==========
    // GCS MAVLink communication at 400Hz for low latency
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180,  36),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550,  39),
    
    // ========== PERIPHERAL UPDATES ==========
    
#if HAL_MOUNT_ENABLED
    // Camera mount slower update for general positioning (50Hz)
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75,  45),
#endif

#if AP_CAMERA_ENABLED
    // Camera trigger and control updates (50Hz)
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update,              50,  75,  48),
#endif

    // ========== LOGGING TASKS ==========
#if HAL_LOGGING_ENABLED
    // Log attitude, PID, motor, RC data at 10Hz
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  51),
    
    // Log attitude and IMU at 25Hz (fast attitude logging)
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  54),
    
    // Log IMU at full loop rate if configured (high-rate logging)
    SCHED_TASK(loop_rate_logging, LOOP_RATE, 50,   55),
    
    // Logger periodic tasks (flush buffers, manage files) at 400Hz
    SCHED_TASK_CLASS(AP_Logger,           &sub.logger,       periodic_tasks,     400, 300,  57),
#endif
    
    // ========== SYSTEM MAINTENANCE ==========
    
    // INS periodic calibration and health checks (400Hz)
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50,  60),
    
#if HAL_LOGGING_ENABLED
    // Update scheduler performance logging (0.1Hz = every 10 seconds)
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75,  63),
#endif

#if AP_RPM_ENABLED
    // Update RPM sensor readings (10Hz)
    SCHED_TASK_CLASS(AP_RPM,              &sub.rpm_sensor,   update,              10, 200,  66),
#endif
    
    // Update terrain altitude database (10Hz)
    SCHED_TASK(terrain_update,        10,    100,  72),
    
#if AP_STATS_ENABLED
    // Update flight statistics (1Hz)
    SCHED_TASK(stats_update,           1,    200,  76),
#endif

    // ========== USER HOOK TASKS ==========
    // Optional user-defined custom code hooks at various rates
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75,  78),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75,  81),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75,  84),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75,  87),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75,  90),
#endif

};

/**
 * @brief Get scheduler task table for AP_Scheduler
 * 
 * @details Returns the Sub-specific scheduler task table to the AP_Scheduler framework.
 *          This allows the scheduler to execute vehicle-specific tasks along with
 *          common AP_Vehicle tasks in the correct priority order.
 * 
 * @param[out] tasks Pointer to task array set to Sub::scheduler_tasks
 * @param[out] task_count Number of tasks in the array
 * @param[out] log_bit Logging bitmask (MASK_LOG_PM for performance monitoring)
 * 
 * @note Called by AP_Scheduler during initialization to build complete task list
 * 
 * Source: ArduSub/Sub.cpp:168-175
 */
void Sub::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Sub::_failsafe_priorities[5];

/**
 * @brief Execute low-level angular rate controller
 * 
 * @details Runs the attitude rate controller (inner control loop) that converts desired
 *          angular rates to motor outputs. This is a fast loop task running at
 *          MAIN_LOOP_RATE (typically 400Hz).
 * 
 *          **Execution Sequence**:
 *          1. Get actual loop time from scheduler
 *          2. Update dt (delta time) in motors, attitude_control, and pos_control
 *          3. Run rate controller if not in MANUAL or MOTOR_DETECT mode
 * 
 *          The rate controller is bypassed in MANUAL mode (direct pilot control)
 *          and MOTOR_DETECT mode (motor testing without control loops).
 * 
 * @note Runs at MAIN_LOOP_RATE (typically 400Hz) - safety-critical timing
 * @warning Must complete quickly to avoid delaying other fast loop tasks
 * @warning Skipped in MANUAL and MOTOR_DETECT modes for direct control
 * 
 * Source: ArduSub/Sub.cpp:179-191
 */
void Sub::run_rate_controller()
{
    // Get actual loop time for accurate dt calculation
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    
    // Update delta time for all controllers (critical for PID accuracy)
    motors.set_dt_s(last_loop_time_s);
    attitude_control.set_dt_s(last_loop_time_s);
    pos_control.set_dt_s(last_loop_time_s);

    // Don't run rate controller in MANUAL or MOTOR_DETECT modes
    // MANUAL: pilot has direct control without stabilization
    // MOTOR_DETECT: testing individual motors without control loops
    if (control_mode != Mode::Number::MANUAL && control_mode != Mode::Number::MOTOR_DETECT) {
        // Run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }
}

/**
 * @brief 50Hz periodic task loop
 * 
 * @details Executes tasks that need to run at 50Hz for responsive failsafe monitoring
 *          and RC input processing. This rate provides good balance between CPU usage
 *          and responsiveness for safety checks.
 * 
 *          **Tasks Performed** (in order):
 *          1. Pilot input failsafe check - Detects loss of RC signal
 *          2. Crash detection check - Monitors for vehicle collision/impact
 *          3. EKF failsafe check - Monitors navigation solution health
 *          4. Sensor health check - Validates critical sensor functionality
 *          5. RC input reading (if RC disabled in build)
 *          6. Actuator updates - Update non-motor actuators (lights, grippers, etc.)
 * 
 * @note Runs at 50Hz as scheduled in scheduler_tasks table
 * @warning Failsafe checks are safety-critical and must execute reliably
 * 
 * Source: ArduSub/Sub.cpp:194-208
 */
void Sub::fifty_hz_loop()
{
    // Check for loss of pilot RC input (safety-critical)
    failsafe_pilot_input_check();

    // Check for vehicle crash/impact conditions
    failsafe_crash_check();

    // Check EKF navigation solution health
    failsafe_ekf_check();

    // Check critical sensor health (depth, compass, etc.)
    failsafe_sensors_check();
    
#if !AP_SUB_RC_ENABLED
    // Read RC input if not handled by rc_loop
    rc().read_input();
#endif
    
    // Update auxiliary actuators (lights, camera tilt, grippers, etc.)
    g2.actuators.update_actuators();
}

/**
 * @brief Update battery monitor and compass readings
 * 
 * @details Reads battery voltage/current and compass heading at 10Hz. Battery is
 *          read before compass because battery current data is used for motor
 *          interference compensation (compassmot).
 * 
 *          **Update Sequence**:
 *          1. Read battery voltage, current, and remaining capacity
 *          2. If compass available, set current throttle value for interference compensation
 *          3. Read compass and apply motor interference compensation
 * 
 *          The throttle value is passed to compass for real-time compensation of
 *          magnetic interference from motor/thruster currents.
 * 
 * @note Should be called at 10Hz as scheduled
 * @note Battery read first to provide current data for compass compensation
 * 
 * Source: ArduSub/Sub.cpp:212-222
 */
void Sub::update_batt_compass()
{
    // Read battery voltage, current, and capacity
    // Must be done before compass for motor interference compensation
    battery.read();

    if (AP::compass().available()) {
        // Update compass with current throttle value
        // Used for compassmot (motor interference compensation)
        compass.set_throttle(motors.get_throttle());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief 10Hz logging task
 * 
 * @details Logs vehicle data at 10Hz rate for standard resolution logging.
 *          Logging is conditional based on configured log bitmasks to conserve
 *          storage space and CPU time.
 * 
 *          **Data Logged** (when enabled):
 *          - Attitude (roll/pitch/yaw) at medium rate
 *          - Attitude targets and rate targets
 *          - PID controller states for roll/pitch/yaw/altitude
 *          - Motor outputs and battery status
 *          - RC inputs and servo outputs
 *          - Position controller navigation tuning
 *          - IMU vibration levels
 *          - Camera mount position
 * 
 * @note Attitude logging at 10Hz only if not logging at 25Hz (FAST) rate
 * @note Some logs (NTUN) only written when mode requires GPS or altitude
 * 
 * Source: ArduSub/Sub.cpp:227-261
 */
void Sub::ten_hz_logging_loop()
{
    // Log attitude data at medium rate if not already logging at fast rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();  // Roll/pitch/yaw angles
        attitude_control.Write_ANG();  // Angle targets
        attitude_control.Write_Rate(pos_control);  // Rate targets
        
        // Log PID controller information if enabled
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_U_pid().get_pid_info());
        }
    }
    
    // Log motor outputs and battery data
    if (should_log(MASK_LOG_MOTBATT)) {
        motors.Log_Write();
    }
    
    // Log RC receiver inputs
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    
    // Log servo/motor PWM outputs
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    
    // Log navigation tuning data for modes that use position control
    if (should_log(MASK_LOG_NTUN) && (sub.flightmode->requires_GPS() || sub.flightmode->requires_altitude())) {
        pos_control.write_log();
    }
    
    // Log IMU vibration levels
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    
#if HAL_MOUNT_ENABLED
    // Log camera mount position and state
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/**
 * @brief 25Hz logging task for fast attitude logging
 * 
 * @details Logs attitude and IMU data at 25Hz for higher resolution analysis
 *          of vehicle dynamics and control performance. This rate is useful
 *          for tuning and detailed flight analysis.
 * 
 *          **Data Logged** (when enabled):
 *          - Attitude (roll/pitch/yaw) at fast rate
 *          - Attitude and rate targets
 *          - PID controller states
 *          - IMU data at medium rate (if not logging at loop rate)
 * 
 * @note Only logs if MASK_LOG_ATTITUDE_FAST is enabled
 * @note IMU logged at 25Hz only if not logging at full loop rate
 * 
 * Source: ArduSub/Sub.cpp:265-283
 */
void Sub::twentyfive_hz_logging()
{
    // Log attitude data at fast rate
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();  // Current roll/pitch/yaw
        attitude_control.Write_ANG();  // Angle targets
        attitude_control.Write_Rate(pos_control);  // Rate targets
        
        // Log PID controller information for tuning analysis
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_U_pid().get_pid_info());
        }
    }

    // Log IMU data at medium rate if we're not already logging at loop rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}

/**
 * @brief Full loop rate IMU logging (typically 400Hz)
 * 
 * @details Logs IMU data at the full main loop rate for maximum resolution
 *          analysis of high-frequency vibrations and IMU performance. This
 *          generates large log files and should only be enabled when needed
 *          for detailed analysis or debugging.
 * 
 * @note Runs at LOOP_RATE (typically 400Hz) - generates large log files
 * @warning High-rate logging can fill SD cards quickly
 * 
 * Source: ArduSub/Sub.cpp:286-291
 */
void Sub::loop_rate_logging()
{
    // Log IMU data at full loop rate for high-resolution analysis
    if (should_log(MASK_LOG_IMU_FAST)) {
        AP::ins().Write_IMU();
    }
}
#endif  // HAL_LOGGING_ENABLED

/**
 * @brief 3Hz periodic task loop
 * 
 * @details Executes slower periodic tasks at 3.3Hz rate. This rate is suitable
 *          for environmental monitoring and non-critical failsafe checks that
 *          don't require fast response times.
 * 
 *          **Tasks Performed** (in order):
 *          1. Leak detector update - Monitor for water intrusion
 *          2. Leak failsafe check - Trigger failsafe if leak detected
 *          3. Internal pressure check - Monitor enclosure pressure
 *          4. Internal temperature check - Monitor enclosure temperature
 *          5. GCS communication failsafe - Detect loss of ground station link
 *          6. Terrain data failsafe - Check terrain database availability
 *          7. Servo/relay event updates - Execute timed servo/relay commands
 * 
 * @note Runs at 3.3Hz as scheduled in scheduler_tasks table
 * @note Environmental checks (leak, pressure, temperature) are Sub-specific
 * @warning GCS and terrain failsafes are safety-critical
 * 
 * Source: ArduSub/Sub.cpp:295-314
 */
void Sub::three_hz_loop()
{
    // Update leak detector sensor readings
    leak_detector.update();

    // Check for water leak and trigger failsafe if detected
    failsafe_leak_check();

    // Check internal enclosure pressure (over/under pressure conditions)
    failsafe_internal_pressure_check();

    // Check internal enclosure temperature (overheating protection)
    failsafe_internal_temperature_check();

    // Check if we've lost contact with the ground station
    failsafe_gcs_check();

    // Check if we've lost terrain altitude database
    failsafe_terrain_check();

#if AP_SERVORELAYEVENTS_ENABLED
    // Update timed servo and relay events
    ServoRelayEvents.update_events();
#endif
}

/**
 * @brief 1Hz periodic task loop
 * 
 * @details Executes slow periodic tasks at 1Hz rate. These tasks include pre-arm
 *          checks, notification updates, logging, and infrequent system updates
 *          that don't require high-rate execution.
 * 
 *          **Tasks Performed** (in order):
 *          1. Pre-arm safety checks and notification flag updates
 *          2. Log vehicle state if logging enabled
 *          3. Update motor throttle range when disarmed
 *          4. Enable auxiliary servo functions
 *          5. Log terrain data
 *          6. Update "likely flying" state for compass learning
 *          7. Update notch filter sample rates based on actual loop rate
 * 
 *          **Pre-arm Checks**: Validates vehicle is safe to arm including:
 *          - Sensor health (depth, compass, accelerometer, gyro)
 *          - Position estimation quality
 *          - Motor and servo functionality
 *          - Battery levels
 *          - RC calibration
 * 
 * @note Runs at 1Hz as scheduled in scheduler_tasks table
 * @note Pre-arm checks run continuously even when armed (for status reporting)
 * @note Throttle range updates only when disarmed to avoid mid-flight changes
 * @note Notch filter rates updated to track actual loop rate for best performance
 * 
 * Source: ArduSub/Sub.cpp:317-349
 */
void Sub::one_hz_loop()
{
    // Run pre-arm checks and update status flags for notification system
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();
    AP_Notify::flags.flying = motors.armed();

#if HAL_LOGGING_ENABLED
    // Log vehicle state flags for analysis
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }
#endif

    // Update motor throttle range calibration when disarmed
    // Don't update while armed to avoid unexpected motor behavior changes
    if (!motors.armed()) {
        motors.update_throttle_range();
    }

    // Update assigned servo functions and enable auxiliary servos
    // (lights, camera tilt, gripper, etc.)
    AP::srv().enable_aux_servos();

#if HAL_LOGGING_ENABLED
    // Log terrain altitude database status
    terrain_logging();
#endif

    // Set "likely flying" state when armed to allow compass learning
    // Compass learning uses this to know when to update calibration
    set_likely_flying(hal.util->get_soft_armed());

    // Update notch filter sample rates to match actual loop rate
    // Important for accurate harmonic notch filtering
    attitude_control.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
    pos_control.get_accel_U_pid().set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
}

/**
 * @brief Update AHRS and EKF state estimation
 * 
 * @details Runs the Attitude and Heading Reference System (AHRS) update which
 *          performs sensor fusion and state estimation. This is a fast loop task
 *          running at MAIN_LOOP_RATE (typically 400Hz).
 * 
 *          **Update Sequence**:
 *          1. AHRS update with true parameter (skip INS update - already done)
 *          2. AHRS view update for coordinate frame transformations
 * 
 *          The 'true' parameter tells AHRS to skip the INS (Inertial Navigation System)
 *          update because it has already been performed earlier in the fast loop by
 *          the INS.update() task.
 * 
 * @note Runs at MAIN_LOOP_RATE (typically 400Hz) - safety-critical
 * @note INS update already completed earlier in fast loop
 * @warning EKF state estimation must run at consistent high rate for accuracy
 * 
 * Source: ArduSub/Sub.cpp:351-358
 */
void Sub::read_AHRS()
{
    // Perform EKF state estimation and sensor fusion
    // Pass true to skip INS update (already done in fast_loop)
    ahrs.update(true);
    
    // Update AHRS view for coordinate frame transformations
    ahrs_view.update();
}

/**
 * @brief Update altitude/depth readings and log control tuning data
 * 
 * @details Reads barometer (depth sensor for underwater vehicles) at 10Hz and
 *          logs control tuning data for analysis. For ArduSub, the barometer
 *          measures depth rather than altitude above sea level.
 * 
 *          **Tasks Performed**:
 *          1. Read barometer/depth sensor
 *          2. Log control tuning data (if enabled)
 *          3. Log harmonic notch filter state (if enabled)
 *          4. Log gyro FFT data (if enabled)
 * 
 * @note Runs at 10Hz as scheduled
 * @note For Sub, barometer measures depth (pressure underwater)
 * @note Control tuning logs are essential for PID tuning and analysis
 * 
 * Source: ArduSub/Sub.cpp:361-377
 */
void Sub::update_altitude()
{
    // Read barometer to get depth measurement
    // For underwater vehicles, this provides depth below surface
    read_barometer();

#if HAL_LOGGING_ENABLED
    // Log control tuning data for PID analysis and tuning
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        // Log harmonic notch filter state for vibration analysis
        AP::ins().write_notch_log_messages();
#endif

#if HAL_GYROFFT_ENABLED
        // Log gyro FFT data for frequency domain vibration analysis
        gyro_fft.write_log_messages();
#endif
    }
#endif  // HAL_LOGGING_ENABLED
}

/**
 * @brief Check if depth sensor (barometer) is healthy for depth control
 * 
 * @details Validates that the depth sensor is present and functioning correctly
 *          before allowing depth-holding flight modes. This is critical for
 *          underwater vehicle safety as depth control requires accurate pressure
 *          measurements.
 * 
 * @return true if depth sensor is present and healthy
 * @return false if depth sensor missing or unhealthy
 * 
 * @note Sends warning message to ground station if check fails
 * @warning Depth control modes should not be allowed without healthy depth sensor
 * 
 * Source: ArduSub/Sub.cpp:379-389
 */
bool Sub::control_check_barometer()
{
    // Check if depth sensor is present
    if (!ap.depth_sensor_present) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    }
    
    // Check if depth sensor is healthy
    if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
    
    return true;
}

/**
 * @brief Get distance to current waypoint in meters
 * 
 * @details Helper function for MAVLink NAV_CONTROLLER_OUTPUT message.
 *          Returns the straight-line distance from the vehicle to the
 *          current waypoint target.
 * 
 * @param[out] distance Distance to waypoint in meters
 * @return true always (distance always available)
 * 
 * @note Used by GCS_MAVLINK_Sub::send_nav_controller_output()
 * @note Converts from centimeters to meters
 * 
 * Source: ArduSub/Sub.cpp:392-397
 */
bool Sub::get_wp_distance_m(float &distance) const
{
    // Get waypoint distance from wp_nav and convert cm to meters
    distance = sub.wp_nav.get_wp_distance_to_destination_cm() * 0.01;
    return true;
}

/**
 * @brief Get bearing to current waypoint in degrees
 * 
 * @details Helper function for MAVLink NAV_CONTROLLER_OUTPUT message.
 *          Returns the compass bearing from the vehicle to the current
 *          waypoint target.
 * 
 * @param[out] bearing Bearing to waypoint in degrees (0-360)
 * @return true always (bearing always available)
 * 
 * @note Used by GCS_MAVLINK_Sub::send_nav_controller_output()
 * @note Converts from centidegrees to degrees
 * 
 * Source: ArduSub/Sub.cpp:400-405
 */
bool Sub::get_wp_bearing_deg(float &bearing) const
{
    // Get waypoint bearing from wp_nav and convert centidegrees to degrees
    bearing = sub.wp_nav.get_wp_bearing_to_destination_cd() * 0.01;
    return true;
}

/**
 * @brief Get crosstrack error to waypoint path in meters
 * 
 * @details Helper function for MAVLink NAV_CONTROLLER_OUTPUT message.
 *          For ArduSub, crosstrack error is not calculated/reported as
 *          underwater vehicles typically don't follow strict paths due to
 *          currents and limited position holding capability.
 * 
 * @param[out] xtrack_error Crosstrack error in meters (always 0 for Sub)
 * @return true always
 * 
 * @note ArduSub does not calculate crosstrack error
 * @note Used by GCS_MAVLINK_Sub::send_nav_controller_output()
 * 
 * Source: ArduSub/Sub.cpp:408-413
 */
bool Sub::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // ArduSub does not report crosstrack error
    // Underwater navigation typically doesn't follow strict paths
    xtrack_error = 0;
    return true;
}

#if AP_STATS_ENABLED
/**
 * @brief Update flight statistics
 * 
 * @details Updates the AP_Stats module with current vehicle state. For ArduSub,
 *          the vehicle is considered "flying" (actually "operating") when motors
 *          are armed, regardless of actual movement.
 * 
 *          Statistics tracked include:
 *          - Total flight time (armed time)
 *          - Number of flights (arm/disarm cycles)
 *          - Fault counters
 * 
 * @note Runs at 1Hz as scheduled
 * @note For underwater vehicles, "flying" means "operating with motors armed"
 * 
 * Source: ArduSub/Sub.cpp:415-423
 */
void Sub::stats_update(void)
{
    // Set flying state based on armed status
    // For Sub, "flying" means motors armed and operating
    AP::stats()->set_flying(motors.armed());
}
#endif

/**
 * @brief Get altitude/depth relative to home position or EKF origin
 * 
 * @details Returns the vehicle's depth relative to the home position (surface dive point).
 *          For underwater vehicles, this represents depth below the point where the vehicle
 *          was armed or the manually set home location.
 * 
 *          **Calculation Method**:\n
 *          1. If EKF relative position available: Use EKF position D (down)\n
 *          2. Adjust for home altitude if home is set\n
 *          3. Otherwise: Fall back to barometer altitude\n
 *          4. Convert NED down coordinate to up/altitude convention
 * 
 * @return Altitude in meters (positive = above home, negative = below home/deeper)
 * @return 0 if depth sensor not present
 * 
 * @note Uses NED (North-East-Down) frame internally, converts to up for return
 * @note For Sub, positive values = shallower, negative = deeper
 * @note Falls back to barometer if EKF position unavailable
 * 
 * Source: ArduSub/Sub.cpp:425-447
 */
float Sub::get_alt_rel() const
{
    // Return 0 if no depth sensor available
    if (!ap.depth_sensor_present) {
        return 0;
    }

    // Get position D (down in NED frame) from EKF
    float posD;
    if (ahrs.get_relative_position_D_origin_float(posD)) {
        // If home position is set, adjust to be relative to home altitude
        if (ahrs.home_is_set()) {
            auto home = ahrs.get_home();
            // Convert home altitude from cm to m and subtract
            posD -= static_cast<float>(home.alt) * 0.01f;
        }
    } else {
        // Fall back to barometer altitude if EKF position not available
        // Barometer altitude is positive up, so negate
        posD = -AP::baro().get_altitude();
    }

    // Convert NED down to altitude up convention
    // Negative posD (deeper) becomes negative altitude (below home)
    return -posD;
}

/**
 * @brief Get altitude above mean sea level (MSL)
 * 
 * @details Returns the vehicle's absolute altitude above mean sea level. For underwater
 *          vehicles, this is typically a negative value (below sea level) when submerged.
 * 
 *          **Calculation Method**:\n
 *          1. Get EKF origin (reference point with known MSL altitude)\n
 *          2. Get position D (down) relative to EKF origin\n
 *          3. Add EKF origin altitude to get absolute MSL altitude\n
 *          4. Convert NED down to altitude up convention\n
 *          5. Fall back to barometer if EKF position unavailable
 * 
 * @return Altitude above mean sea level in meters
 * @return 0 if depth sensor not present or EKF origin not set
 * 
 * @note Requires valid EKF origin with known MSL altitude
 * @note For underwater operations, typically returns negative values (below sea level)
 * @note Falls back to barometer if EKF unavailable
 * 
 * Source: ArduSub/Sub.cpp:449-473
 */
float Sub::get_alt_msl() const
{
    // Return 0 if no depth sensor available
    if (!ap.depth_sensor_present) {
        return 0;
    }

    // Get EKF origin location (with known MSL altitude)
    Location origin;
    if (!ahrs.get_origin(origin)) {
        return 0;  // Can't calculate MSL altitude without origin
    }

    // Get position D (down in NED frame) relative to EKF origin
    float posD;
    if (!ahrs.get_relative_position_D_origin_float(posD)) {
        // Fall back to barometer altitude if EKF position not available
        posD = -AP::baro().get_altitude();
    }

    // Subtract EKF origin altitude to get position relative to MSL
    // origin.alt is in cm, convert to meters
    posD -= static_cast<float>(origin.alt) * 0.01f;

    // Convert NED down to altitude up convention
    // Negative posD (below MSL) becomes negative altitude
    return -posD;
}

/**
 * @brief Ensure EKF origin is set for position estimation
 * 
 * @details Verifies that the EKF has a valid origin point (reference location) for
 *          position estimation. The origin is needed for converting between relative
 *          positions and absolute GPS coordinates. For ArduSub, this is critical for
 *          GPS-enabled navigation modes.
 * 
 *          **Origin Setting Priority**:\n
 *          1. If origin already set: Return success\n
 *          2. If GPS present: Wait for GPS to set origin automatically\n
 *          3. If no GPS: Use backup origin parameters (BACKUP_ORIGIN_LAT/LON/ALT)\n
 * 
 *          **Backup Origin Validation**:\n
 *          - Checks that backup parameters are non-zero\n
 *          - Validates latitude/longitude are in valid ranges\n
 *          - Attempts to set as EKF origin\n
 *          - Logs and sends origin to ground station if successful\n
 * 
 * @return true if EKF origin is set (or successfully set)
 * @return false if origin not set and cannot be set
 * 
 * @note Required for GPS-based navigation modes
 * @note Backup origin useful for GPS-denied environments (pools, tanks)
 * @warning If GPS present but not ready, this returns false (waiting for GPS)
 * @warning Check EK3_SRC parameters if backup origin fails to set
 * 
 * Source: ArduSub/Sub.cpp:475-520
 */
bool Sub::ensure_ekf_origin()
{
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        // EKF origin is already set
        return true;
    }

    // If GPS is present, wait for it to set the origin automatically
    if (gps.num_sensors() > 0) {
        // GPS will set origin when it gets a fix
        // Return false to indicate origin not yet ready
        return false;
    }

    // No GPS available - try to use backup origin parameters
    // Convert backup origin parameters from degrees to 1e7 format (lat/lon)
    // and cm format (altitude)
    auto backup_origin = Location(static_cast<int32_t>(sub.g2.backup_origin_lat * 1e7),
                                  static_cast<int32_t>(sub.g2.backup_origin_lon * 1e7),
                                  static_cast<int32_t>(sub.g2.backup_origin_alt * 100),
                                  Location::AltFrame::ABSOLUTE);

    // Validate backup origin parameters are set
    if (backup_origin.lat == 0 || backup_origin.lng == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Backup location parameters are missing or zero");
        return false;
    }

    // Validate backup origin coordinates are in valid range
    if (!check_latlng(backup_origin.lat, backup_origin.lng)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Backup location parameters are not valid");
        return false;
    }

    // Attempt to set the backup origin as EKF origin
    if (!ahrs.set_origin(backup_origin)) {
        // Common issue: EK3_SRC1_POSXY set to GPS (3) but no GPS present
        gcs().send_text(MAV_SEVERITY_WARNING, "Failed to set origin, check EK3_SRC parameters");
        return false;
    }

    // Successfully set backup origin
    gcs().send_text(MAV_SEVERITY_INFO, "Using backup location");

#if HAL_LOGGING_ENABLED
    // Log the home and origin for analysis
    ahrs.Log_Write_Home_And_Origin();
#endif

    // Send EKF origin message to ground station
    gcs().send_message(MSG_ORIGIN);

    return true;
}

#if AP_SUB_RC_ENABLED
/**
 * @brief Process RC (radio control) input
 * 
 * @details Reads RC receiver input and processes mode switch changes.
 *          This function runs at a medium frequency (typically 10-25Hz)
 *          to read pilot stick inputs and flight mode switch position.
 * 
 *          **Operations Performed**:\n
 *          1. read_radio(): Read all RC channels and process pilot inputs\n
 *          2. read_mode_switch(): Check for mode changes from 3-position switch
 * 
 * @note Only compiled if AP_SUB_RC_ENABLED is defined
 * @note Runs at scheduler-defined rate (see scheduler_tasks table)
 * @note Mode switch typically mapped to channel 5 or 6
 * 
 * Source: ArduSub/Sub.cpp:522-529
 */
void Sub::rc_loop()
{
    // Read radio receiver and process all RC channel inputs
    read_radio();
    
    // Check 3-position flight mode switch for mode changes
    rc().read_mode_switch();
}
#endif

/**
 * @brief Static singleton pointer for Sub instance
 * @details Used to enforce single instance and provide global access
 */
Sub *Sub::_singleton = nullptr;

/**
 * @brief Global Sub vehicle instance
 * @details The main vehicle object instantiated at program startup
 */
Sub sub;

/**
 * @brief Reference to vehicle base class for AP_Vehicle interface
 * @details Provides polymorphic access to common vehicle functions
 */
AP_Vehicle& vehicle = sub;

/**
 * @brief HAL main entry point and callback setup
 * @details Registers the Sub object with the Hardware Abstraction Layer
 *          and establishes the main program entry point
 */
AP_HAL_MAIN_CALLBACKS(&sub);
