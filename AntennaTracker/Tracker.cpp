/*
   Lead developers: Matthew Ridley and Andrew Tridgell

   Please contribute your ideas! See https://ardupilot.org/dev for details

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
 * @file Tracker.cpp
 * @brief Main antenna tracker class implementation and scheduler task table
 * 
 * @details This file implements the core Tracker class functionality including:
 * - Scheduler task table defining all periodic tracker functions
 * - Periodic maintenance loops (1Hz housekeeping, 10Hz logging)
 * - Scheduler task registration interface
 * - Global tracker instance declaration
 * 
 * The scheduler_tasks[] array defines all periodic tasks that run on the
 * antenna tracker, including attitude estimation, RC input, tracking updates,
 * sensor reading, and telemetry communication. Tasks are prioritized and
 * scheduled by AP_Scheduler to meet timing constraints.
 * 
 * Source: AntennaTracker/Tracker.cpp
 */

#include "Tracker.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

#define SCHED_TASK(func, rate_hz, _max_time_micros, _prio) SCHED_TASK_CLASS(Tracker, &tracker, func, rate_hz, _max_time_micros, _prio)

/*
  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */

/**
 * @brief Scheduler task table defining all periodic antenna tracker functions
 * 
 * @details This table defines every periodic task that runs on the antenna tracker.
 * Each entry specifies:
 * - Function pointer or method to call
 * - Rate in Hz at which the function should be called
 * - Maximum expected execution time in microseconds
 * - Priority (0-255, lower number = higher priority)
 * 
 * Tasks are sorted by priority and interleaved with AP_Vehicle base class tasks.
 * The AP_Scheduler ensures timing constraints are met and monitors task execution
 * time to detect overruns. If a task consistently exceeds its allocated time,
 * warnings are logged.
 * 
 * Key task groups:
 * - Attitude estimation (AHRS) - 50Hz for accurate tracking
 * - RC input and tracking updates - 50Hz for responsive control
 * - Sensor updates (GPS, compass, baro, IMU) - 10-50Hz based on sensor requirements
 * - Communication (MAVLink GCS) - 50Hz for low-latency telemetry
 * - Logging - 10Hz for data recording
 * - Housekeeping - 1Hz for periodic maintenance
 * 
 * @note All entries must be ordered by priority for correct scheduler operation
 * @note Task execution times are monitored and logged if SCHED_DEBUG is enabled
 * 
 * Source: AntennaTracker/Tracker.cpp:68-85
 */
const AP_Scheduler::Task Tracker::scheduler_tasks[] = {
    // 50Hz: Update AHRS attitude estimation using latest sensor data
    SCHED_TASK(update_ahrs,            50,    1000,  5),
    
    // 50Hz: Read RC radio input for manual control
    SCHED_TASK(read_radio,             50,     200, 10),
    
    // 50Hz: Main tracking loop - calculates target position, bearing, and sends servo commands
    SCHED_TASK(update_tracking,        50,    1000, 15),
    
    // 10Hz: Update GPS driver and perform ground start logic if needed
    SCHED_TASK(update_GPS,             10,    4000, 20),
    
    // 10Hz: Read magnetometer for heading information
    SCHED_TASK(update_compass,         10,    1500, 25),
    
    // 10Hz: Read battery voltage and current sensors
    SCHED_TASK_CLASS(AP_BattMonitor,    &tracker.battery,   read,           10, 1500, 35),
    
    // 10Hz: Update barometer altitude reading
    SCHED_TASK_CLASS(AP_Baro,          &tracker.barometer,  update,         10, 1500, 40),
    
    // 50Hz: Process incoming MAVLink messages from ground control station
    SCHED_TASK_CLASS(GCS,              (GCS*)&tracker._gcs, update_receive, 50, 1700, 45),
    
    // 50Hz: Send outgoing MAVLink telemetry to ground control station
    SCHED_TASK_CLASS(GCS,              (GCS*)&tracker._gcs, update_send,    50, 3000, 50),
#if HAL_LOGGING_ENABLED
    // 10Hz: Write high-rate logs (IMU, attitude, RC input/output)
    SCHED_TASK(ten_hz_logging_loop,    10,    300, 60),
    
    // 50Hz: Logger periodic tasks (streaming to SD card, buffer management)
    SCHED_TASK_CLASS(AP_Logger,   &tracker.logger, periodic_tasks, 50,  300, 65),
#endif
    // 50Hz: IMU periodic tasks (filtering, calibration updates)
    SCHED_TASK_CLASS(AP_InertialSensor, &tracker.ins,       periodic,       50,   50, 70),
    
    // 1Hz: Periodic housekeeping (home setup, servo enable, PID updates, logging mask)
    SCHED_TASK(one_second_loop,         1,   3900, 80),
    
    // 1Hz: Update statistics (flight time, armed time)
    SCHED_TASK(stats_update,            1,    200, 90),
};

/**
 * @brief Returns scheduler task table to AP_Scheduler for task registration
 * 
 * @details This method is called by AP_Scheduler during initialization to register
 * all antenna tracker-specific periodic tasks. The scheduler will execute these tasks
 * according to their specified rates and priorities, interleaved with base AP_Vehicle tasks.
 * 
 * @param[out] tasks       Pointer to the task array (set to scheduler_tasks)
 * @param[out] task_count  Number of tasks in the array
 * @param[out] log_bit     Logging bit mask for this vehicle type (set to log all)
 * 
 * @note Task execution is monitored by the scheduler with timing constraints enforced
 * @note The log_bit value of (uint32_t)-1 enables all logging types for the tracker
 * 
 * Source: AntennaTracker/Tracker.cpp:107-113
 */
void Tracker::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = (uint32_t)-1;
}

/**
 * @brief 1Hz periodic housekeeping and maintenance tasks
 * 
 * @details This function performs periodic maintenance tasks that don't require
 * high-frequency execution. Called by the scheduler at 1Hz.
 * 
 * Tasks performed:
 * - Enable auxiliary servos for assigned functions
 * - Update armed/disarmed status LEDs via update_armed_disarmed()
 * - Set home location if not yet established (using current AHRS position)
 * - Update "likely flying" state based on armed status (enables compass learning)
 * - Update notification flags for LEDs/buzzers
 * - Update PID notch filter sample rates based on current loop rate
 * 
 * @note Called at 1Hz by scheduler (priority 80, max time 3900μs)
 * @note Home location is automatically set to current position if not yet initialized
 * @note "Likely flying" state must be set when armed to allow compass learning
 * 
 * Source: AntennaTracker/Tracker.cpp:126-141
 */
void Tracker::one_second_loop()
{
    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

    // updated armed/disarmed status LEDs
    update_armed_disarmed();

    if (!ahrs.home_is_set()) {
        // set home to current location
        Location temp_loc;
        if (ahrs.get_location(temp_loc)) {
            if (!set_home(temp_loc, false)) {
                // fail silently
            }
        }
    }

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    set_likely_flying(hal.util->get_soft_armed());

    AP_Notify::flags.flying = hal.util->get_soft_armed();

    g.pidYaw2Srv.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
}

#if HAL_LOGGING_ENABLED
/**
 * @brief 10Hz logging tasks for high-rate data recording
 * 
 * @details This function writes high-frequency log messages to the dataflash/SD card
 * if logging is enabled. Called by the scheduler at 10Hz.
 * 
 * Log messages written (if corresponding mask bit enabled):
 * - MASK_LOG_IMU: Inertial Measurement Unit data (accelerometer, gyroscope)
 * - MASK_LOG_ATTITUDE: Vehicle attitude (roll, pitch, yaw)
 * - MASK_LOG_RCIN: RC radio input channels
 * - MASK_LOG_RCOUT: Servo/motor output channels
 * 
 * @note Only compiled if HAL_LOGGING_ENABLED is defined
 * @note Called at 10Hz by scheduler (priority 60, max time 300μs)
 * @note Each log write is conditional on the corresponding logging mask bit
 * 
 * Source: AntennaTracker/Tracker.cpp:145-159
 */
void Tracker::ten_hz_logging_loop()
{
    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
    if (should_log(MASK_LOG_ATTITUDE)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
}
#endif

Mode *Tracker::mode_from_mode_num(const Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::STOP:
        ret = &mode_stop;
        break;
    case Mode::Number::SCAN:
        ret = &mode_scan;
        break;
    case Mode::Number::GUIDED:
        ret = &mode_guided;
        break;
    case Mode::Number::SERVOTEST:
        ret = &mode_servotest;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::INITIALISING:
        ret = &mode_initialising;
        break;
    }
    return ret;
}

/**
 * @brief Update flight statistics based on armed state
 * 
 * @details Updates the AP_Stats subsystem with the current armed state to track
 * flight time statistics. The tracker is considered "flying" when armed.
 * 
 * @note Called at 1Hz by scheduler (priority 90, max time 200μs)
 * 
 * Source: AntennaTracker/Tracker.cpp:281-284
 */
void Tracker::stats_update(void)
{
    AP::stats()->set_flying(hal.util->get_soft_armed());
}

/**
 * @brief Hardware Abstraction Layer singleton instance
 * 
 * @details Provides platform-independent access to hardware peripherals across
 * all supported boards (ChibiOS, Linux, ESP32, SITL).
 */
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/**
 * @brief Global Tracker singleton instance
 * 
 * @details Single global antenna tracker instance that is accessed throughout
 * the tracker codebase. This object is constructed at startup before main()
 * and contains all tracker state, configuration, and subsystem instances.
 * 
 * The Tracker class inherits from AP_Vehicle and implements antenna tracking
 * functionality including:
 * - Attitude estimation and control
 * - Target tracking calculations
 * - Servo control for pan/tilt
 * - Mode management (Manual, Auto, Scan, etc.)
 * - Telemetry and logging
 * 
 * @note This is the single tracker instance - there is only one per system
 * @note Constructor initializes all member variables and subsystem objects
 * @note Accessed globally as 'tracker' throughout AntennaTracker code
 * 
 * Source: AntennaTracker/Tracker.cpp:288-289
 */
Tracker tracker;

/**
 * @brief Reference to tracker as AP_Vehicle base class
 * 
 * @details Allows access to the tracker through the AP_Vehicle interface,
 * used by shared ArduPilot libraries that work with the base vehicle class.
 */
AP_Vehicle& vehicle = tracker;

AP_HAL_MAIN_CALLBACKS(&tracker);
