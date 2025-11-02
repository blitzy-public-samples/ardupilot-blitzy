/**
 * @file Plane.cpp
 * @brief ArduPlane main vehicle implementation file
 * 
 * @details This file contains the core implementation of the Plane class, which serves
 *          as the main vehicle controller for fixed-wing aircraft in ArduPilot. It includes:
 *          - Plane class constructor initializing all vehicle subsystems
 *          - Scheduler task table defining periodic execution of flight control tasks
 *          - Main loop methods for sensor updates, attitude control, and navigation
 *          - Flight stage management for takeoff, cruise, and landing phases
 *          - Integration with TECS (Total Energy Control System) for altitude/airspeed control
 *          - Quadplane VTOL support integration when HAL_QUADPLANE_ENABLED
 * 
 *          The scheduler architecture runs tasks at various frequencies:
 *          - FAST_TASK: Execute every loop (typically 400Hz for fixed-wing)
 *          - SCHED_TASK: Execute at specified Hz with priority-based scheduling
 * 
 * @note This file integrates with the AP_Scheduler to provide deterministic
 *       execution of flight control tasks with configurable priorities.
 * 
 * Lead developer: Andrew Tridgell
 * 
 * Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, 
 *             John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, 
 *             Paul Riseborough, Brandon Jones, Jon Challinger, Tom Pittenger
 * Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, 
 *             JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, 
 *             Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon
 * 
 * Please contribute your ideas! See https://ardupilot.org/dev for details
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Plane, &plane, func)


/**
 * @brief Scheduler task table defining all periodic tasks for fixed-wing flight control
 * 
 * @details This array defines the complete set of periodic tasks that make up the ArduPlane
 *          main control loop. Tasks are executed by the AP_Scheduler at specified rates and
 *          priorities. All entries must be ordered by priority for correct execution.
 * 
 *          Task Types:
 *          - FAST_TASK: Executed every main loop iteration (typically 400Hz), regardless of timing
 *          - SCHED_TASK: Executed at specified Hz with time budget management
 *          - SCHED_TASK_CLASS: Same as SCHED_TASK but calls a method on a class instance
 * 
 *          FAST_TASK arguments:
 *          - Function name: Method to call on every loop
 * 
 *          SCHED_TASK arguments:
 *          - Function name: Static function to call
 *          - Rate (Hz): Frequency at which the function should be called
 *          - Expected time (μs): Expected execution time for scheduling calculations
 *          - Priority (0-255): Lower number = higher priority
 * 
 *          SCHED_TASK_CLASS arguments:
 *          - Class name: Type of the object
 *          - Instance: Pointer to the object instance
 *          - Method name: Method to call on that instance
 *          - Rate (Hz): Frequency at which the method should be called
 *          - Expected time (μs): Expected execution time for scheduling calculations
 *          - Priority (0-255): Lower number = higher priority
 * 
 *          Critical Fast Tasks (executed every loop at ~400Hz):
 *          - ahrs_update: Update attitude and heading reference system
 *          - update_control_mode: Execute current flight mode control logic
 *          - stabilize: Calculate servo outputs for attitude stabilization
 *          - set_servos: Send calculated servo/motor commands to hardware
 * 
 *          Key Scheduled Tasks:
 *          - read_radio (50Hz): Process RC input from receiver
 *          - update_GPS_50Hz (50Hz): High-rate GPS position updates
 *          - navigate (10Hz): Calculate navigation commands to waypoints
 *          - update_alt (10Hz): Update altitude estimate and TECS controller
 *          - update_compass (10Hz): Read magnetometer and update heading
 *          - GCS update_receive/send (300Hz): Ground control station communication
 *          - Log_Write_FullRate (400Hz): High-rate flight data logging
 * 
 * @note FAST_TASK entries run on every loop even if the loop overruns its time budget.
 *       This ensures critical control functions always execute.
 * 
 * @warning Modifying task rates or priorities can significantly affect flight performance
 *          and stability. Tasks are carefully ordered to ensure proper control loop timing.
 * 
 * @see AP_Scheduler for task execution and time budget management
 * @see Plane::get_scheduler_tasks() for retrieving this table
 * 
 * Source: ArduPlane/Plane.cpp:62-150
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    FAST_TASK(ahrs_update),
    FAST_TASK(update_control_mode),
    FAST_TASK(stabilize),
    FAST_TASK(set_servos),
    SCHED_TASK(read_radio,             50,    100,   6),
    SCHED_TASK(check_short_failsafe,   50,    100,   9),
    SCHED_TASK(update_speed_height,    50,    200,  12),
    SCHED_TASK(update_throttle_hover, 100,     90,  24),
    SCHED_TASK_CLASS(RC_Channels,     (RC_Channels*)&plane.g2.rc_channels, read_mode_switch,           7,    100, 27),
    SCHED_TASK(update_GPS_50Hz,        50,    300,  30),
    SCHED_TASK(update_GPS_10Hz,        10,    400,  33),
    SCHED_TASK(navigate,               10,    150,  36),
    SCHED_TASK(update_compass,         10,    200,  39),
    SCHED_TASK(calc_airspeed_errors,   10,    100,  42),
    SCHED_TASK(update_alt,             10,    200,  45),
    SCHED_TASK(adjust_altitude_target, 10,    200,  48),
#if AP_ADVANCEDFAILSAFE_ENABLED
    SCHED_TASK(afs_fs_check,           10,    100,  51),
#endif
    SCHED_TASK(ekf_check,              10,     75,  54),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_receive,   300,  500,  57),
    SCHED_TASK_CLASS(GCS,            (GCS*)&plane._gcs,       update_send,      300,  750,  60),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents, &plane.ServoRelayEvents, update_events, 50, 150,  63),
#endif
    SCHED_TASK_CLASS(AP_BattMonitor, &plane.battery, read,   10, 300,  66),
#if AP_RANGEFINDER_ENABLED
    SCHED_TASK(read_rangefinder,       50,    100, 78),
#endif
#if AP_ICENGINE_ENABLED
    SCHED_TASK_CLASS(AP_ICEngine,      &plane.g2.ice_control, update,     10, 100,  81),
#endif
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(AP_OpticalFlow, &plane.optflow, update,    50,    50,  87),
#endif
    SCHED_TASK(one_second_loop,         1,    400,  90),
    SCHED_TASK(three_hz_loop,           3,     75,  93),
    SCHED_TASK(check_long_failsafe,     3,    400,  96),
#if AP_RPM_ENABLED
    SCHED_TASK_CLASS(AP_RPM,           &plane.rpm_sensor,     update,     10, 100,  99),
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    SCHED_TASK(airspeed_ratio_update,   1,    100,  102),
#endif // AP_AIRSPEED_AUTOCAL_ENABLE
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount, &plane.camera_mount, update, 50, 100, 105),
#endif // HAL_MOUNT_ENABLED
#if AP_CAMERA_ENABLED
    SCHED_TASK_CLASS(AP_Camera, &plane.camera, update,      50, 100, 108),
#endif // CAMERA == ENABLED
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler, &plane.scheduler, update_logging,         0.2,    100, 111),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(Log_Write_FullRate,        400,    300, 117),
    SCHED_TASK(update_logging10,        10,    300, 120),
    SCHED_TASK(update_logging25,        25,    300, 123),
#endif
#if HAL_SOARING_ENABLED
    SCHED_TASK(update_soaring,         50,    400, 126),
#endif
    SCHED_TASK(parachute_check,        10,    200, 129),
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK_CLASS(AP_Terrain, &plane.terrain, update, 10, 200, 132),
#endif // AP_TERRAIN_AVAILABLE
    SCHED_TASK(update_is_flying_5Hz,    5,    100, 135),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Logger,         &plane.logger, periodic_tasks, 50, 400, 138),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor, &plane.ins,    periodic,       50,  50, 141),
#if HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
    SCHED_TASK(avoidance_adsb_update,  10,    100, 144),
#endif  // HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED
    SCHED_TASK_CLASS(RC_Channels,       (RC_Channels*)&plane.g2.rc_channels, read_aux_all,           10,    200, 147),
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button, &plane.button, update, 5, 100, 150),
#endif
#if AP_LANDINGGEAR_ENABLED
    SCHED_TASK(landing_gear_update, 5, 50, 159),
#endif
#if AC_PRECLAND_ENABLED
    SCHED_TASK(precland_update, 400, 50, 160),
#endif
#if AP_QUICKTUNE_ENABLED
    SCHED_TASK(update_quicktune, 40, 100, 163),
#endif
};

/**
 * @brief Retrieve the scheduler task table for ArduPlane
 * 
 * @details Provides the AP_Scheduler with access to the vehicle-specific task table,
 *          allowing the scheduler to execute all periodic flight control tasks at their
 *          specified rates and priorities.
 * 
 * @param[out] tasks       Pointer to the beginning of the scheduler_tasks array
 * @param[out] task_count  Number of tasks in the scheduler_tasks array
 * @param[out] log_bit     Logging bitmask for performance monitoring (MASK_LOG_PM)
 * 
 * @note This method is called by AP_Vehicle during scheduler initialization
 * 
 * @see scheduler_tasks for the complete task table definition
 * @see AP_Scheduler::init() where this method is called
 */
void Plane::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

#if HAL_QUADPLANE_ENABLED
constexpr int8_t Plane::_failsafe_priorities[7];
#else
constexpr int8_t Plane::_failsafe_priorities[6];
#endif

/**
 * @brief Update Attitude and Heading Reference System (AHRS) - FAST_TASK executed every loop
 * 
 * @details This is one of the most critical functions in ArduPlane, called at main loop rate
 *          (typically 400Hz for fixed-wing). It performs the following operations:
 *          
 *          1. Updates soft_armed state for arming system
 *          2. Calls ahrs.update() to integrate IMU data and estimate attitude
 *          3. Logs IMU data if MASK_LOG_IMU is enabled
 *          4. Calculates scaled roll/pitch limits based on current attitude:
 *             - Roll limit scaled by cos(pitch) to maintain constant wing loading
 *             - Pitch limit scaled by cos(roll) for symmetric pitch authority
 *          5. Updates ground steering course error accumulator using earth-frame yaw rate
 *          6. Checks for EKF yaw resets (quadplane only)
 *          7. Updates inertial navigation for quadplane position control
 *          8. Logs video stabilization data if enabled
 * 
 *          The roll/pitch limit scaling is critical for maintaining control authority
 *          in high-bank or high-pitch attitudes. For example, at 60° bank angle,
 *          the effective roll limit is reduced by cos(60°) = 0.5.
 * 
 * @note This method runs at full loop rate (~400Hz) as a FAST_TASK, ensuring
 *       attitude estimation stays current for all subsequent control calculations
 * 
 * @warning This method must complete quickly (< 100μs typical) to avoid loop overruns
 * 
 * @see AP_AHRS::update() for attitude estimation details
 * @see Plane::stabilize() which uses the updated attitude for control calculations
 * 
 * Source: ArduPlane/Plane.cpp:168-220
 */
void Plane::ahrs_update()
{
    arming.update_soft_armed();

    ahrs.update();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
#endif

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = aparm.roll_limit*100;
    pitch_limit_min = aparm.pitch_limit_min;

    bool rotate_limits = true;
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        rotate_limits = false;
    }
#endif
    if (rotate_limits) {
        roll_limit_cd *= ahrs.cos_pitch();
        pitch_limit_min *= fabsf(ahrs.cos_roll());
    }

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

#if HAL_QUADPLANE_ENABLED
    // check if we have had a yaw reset from the EKF
    quadplane.check_yaw_reset();

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update();
    if (quadplane.available()) {  
        quadplane.pos_control->update_estimates();  
    }
#endif
#if AP_SCRIPTING_ENABLED && AP_FOLLOW_ENABLED
        g2.follow.update_estimates();
#endif

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
#endif
}

/**
 * @brief Update the 50Hz Total Energy Control System (TECS) speed and height controller
 * 
 * @details This method runs at 50Hz and updates the TECS controller which manages the
 *          coupled altitude and airspeed control for fixed-wing flight. TECS uses total
 *          energy principles to optimally allocate pitch and throttle control between
 *          maintaining altitude and maintaining airspeed.
 * 
 *          TECS is disabled (should_run_tecs = false) in the following conditions:
 *          - Flight mode doesn't use automatic throttle control
 *          - Quadplane should disable TECS (in VTOL modes or assisted flight)
 *          - Vehicle is in idle mode (balloon lift)
 *          - Vehicle is in glider pullup maneuver
 * 
 *          When TECS runs, it is called regardless of throttle suppression state
 *          because it needs to be running for proper takeoff detection.
 * 
 *          For quadplane, this method also updates throttle mixing when in VTOL mode
 *          or assisted flight to blend between fixed-wing and multicopter control.
 * 
 * @note Scheduled at 50Hz (every 20ms) via SCHED_TASK with priority 12
 * @note TECS continues running even when throttle is suppressed for takeoff detection
 * 
 * @see AP_TECS::update_50hz() for TECS controller implementation
 * @see Plane::update_alt() for the 10Hz TECS pitch/throttle calculation
 * @see control_mode->does_auto_throttle() for throttle automation check
 * 
 * Source: ArduPlane/Plane.cpp:225-257
 */
void Plane::update_speed_height(void)
{
    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif

    if (auto_state.idle_mode) {
        should_run_tecs = false;
    }

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (mode_auto.in_pullup()) {
        should_run_tecs = false;
    }
#endif

    if (should_run_tecs) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        TECS_controller.update_50hz();
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        quadplane.update_throttle_mix();
    }
#endif
}


/**
 * @brief Read and update compass (magnetometer) data
 * 
 * @details Reads the latest magnetometer measurements from all configured compass devices.
 *          This data feeds into the AHRS/EKF for heading estimation. The compass library
 *          handles multiple magnetometers, calibration offsets, motor interference
 *          compensation, and automatic compass selection.
 * 
 * @note Scheduled at 10Hz via SCHED_TASK with priority 39
 * @note Expected execution time: 200μs
 * 
 * @see AP_Compass::read() for magnetometer data acquisition
 * @see AP_AHRS for how compass data is fused with other sensors
 * 
 * Source: ArduPlane/Plane.cpp:263-266
 */
void Plane::update_compass(void)
{
    compass.read();
}

#if HAL_LOGGING_ENABLED
/**
 * @brief Perform 10Hz logging tasks
 * 
 * @details Handles medium-rate logging operations that run at 10Hz. This includes:
 *          - Attitude logging (if MASK_LOG_ATTITUDE_MED enabled and not logging faster)
 *          - Angle of Attack (AOA) and Sideslip Angle (SSA) logging
 *          - Camera mount logging (if HAL_MOUNT_ENABLED and MASK_LOG_CAMERA set)
 * 
 *          Logging rate hierarchy:
 *          - MASK_LOG_ATTITUDE_FULLRATE: 400Hz (highest priority, logged in fast_loop)
 *          - MASK_LOG_ATTITUDE_FAST: 25Hz (logged in update_logging25)
 *          - MASK_LOG_ATTITUDE_MED: 10Hz (logged here)
 *          
 *          If a faster rate is selected, this function skips attitude logging but still
 *          logs AOA/SSA data which is useful for aerodynamic analysis.
 * 
 * @note Scheduled at 10Hz via SCHED_TASK with priority 120
 * @note Expected execution time: 300μs
 * @note Only compiled if HAL_LOGGING_ENABLED
 * 
 * @see Log_Write_Attitude() for attitude data formatting
 * @see AP_AHRS::Write_AOA_SSA() for aerodynamic angle logging
 * @see AP_Mount::write_log() for gimbal logging
 * 
 * Source: ArduPlane/Plane.cpp:272-286
 */
void Plane::update_logging10(void)
{
    bool log_faster = (should_log(MASK_LOG_ATTITUDE_FULLRATE) || should_log(MASK_LOG_ATTITUDE_FAST));
    if (should_log(MASK_LOG_ATTITUDE_MED) && !log_faster) {
        Log_Write_Attitude();
        ahrs.Write_AOA_SSA();
    } else if (log_faster) {
        ahrs.Write_AOA_SSA();
    }
#if HAL_MOUNT_ENABLED
    if (should_log(MASK_LOG_CAMERA)) {
        camera_mount.write_log();
    }
#endif
}

/**
 * @brief Perform 25Hz logging tasks
 * 
 * @details Handles fast-rate logging operations that run at 25Hz. This includes:
 *          - Attitude logging (if MASK_LOG_ATTITUDE_FAST enabled and not logging at 400Hz)
 *          - Control tuning (CTUN) - TECS/throttle/pitch control data
 *          - Navigation tuning (NTUN) - L1 controller, waypoint navigation data
 *          - Guided mode status
 *          - RC input logging
 *          - IMU vibration logging
 *          - Harmonic notch filter logging (if not logging at full rate)
 *          - Gyro FFT logging (for dynamic notch filter tuning)
 * 
 *          Logging rate hierarchy for attitude:
 *          - MASK_LOG_ATTITUDE_FULLRATE: 400Hz (highest priority, logged elsewhere)
 *          - MASK_LOG_ATTITUDE_FAST: 25Hz (logged here)
 *          - MASK_LOG_ATTITUDE_MED: 10Hz (logged in update_logging10)
 * 
 *          The CTUN and NTUN logs are critical for tuning the aircraft's control loops
 *          and diagnosing control performance issues.
 * 
 * @note Scheduled at 25Hz via SCHED_TASK with priority 123
 * @note Expected execution time: 300μs
 * @note Only compiled if HAL_LOGGING_ENABLED
 * 
 * @see Log_Write_Attitude() for attitude data formatting
 * @see Log_Write_Control_Tuning() for TECS/control loop data
 * @see Log_Write_Nav_Tuning() for navigation controller data
 * @see Log_Write_Guided() for guided mode waypoint data
 * @see Log_Write_RC() for RC input logging
 * @see AP_InertialSensor::Write_Vibration() for vibration analysis
 * 
 * Source: ArduPlane/Plane.cpp:291-322
 */
void Plane::update_logging25(void)
{
    // MASK_LOG_ATTITUDE_FULLRATE logs at 400Hz, MASK_LOG_ATTITUDE_FAST at 25Hz, MASK_LOG_ATTIUDE_MED logs at 10Hz
    // highest rate selected wins
    bool log_faster = should_log(MASK_LOG_ATTITUDE_FULLRATE);
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !log_faster) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        if (!should_log(MASK_LOG_NOTCH_FULLRATE)) {
            AP::ins().write_notch_log_messages();
        }
#endif
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }

    if (should_log(MASK_LOG_NTUN)) {
        Log_Write_Nav_Tuning();
        Log_Write_Guided();
    }

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        AP::ins().Write_Vibration();
}
#endif  // HAL_LOGGING_ENABLED

/**
 * @brief Check Advanced Failsafe System (AFS) for failsafe conditions
 * 
 * @details Monitors the Advanced Failsafe System which provides enhanced failsafe
 *          capabilities for critical applications. AFS can trigger based on:
 *          - Loss of RC communication for extended period
 *          - Loss of GPS for extended period
 *          - Geofence breach with no recovery
 *          - Loss of ground control station communications
 * 
 *          When AFS triggers, it can execute more aggressive actions than standard
 *          failsafe, including:
 *          - Immediate termination (flight termination system)
 *          - Forced landing at current location
 *          - Return to configurable rally point
 * 
 *          The check is provided with the timestamp of the last valid RC message
 *          to help determine RC link health.
 * 
 * @note Scheduled at 10Hz via SCHED_TASK with priority 51
 * @note Expected execution time: 100μs
 * @note Only compiled if AP_ADVANCEDFAILSAFE_ENABLED
 * @note AFS is typically used in safety-critical or commercial applications
 * 
 * @warning AFS termination actions are irreversible and designed for safety
 * 
 * @see AP_AdvancedFailsafe::check() for AFS logic implementation
 * @see Plane::check_short_failsafe() for standard RC failsafe
 * @see Plane::check_long_failsafe() for GCS failsafe
 * 
 * Source: ArduPlane/Plane.cpp:329-333
 */
#if AP_ADVANCEDFAILSAFE_ENABLED
void Plane::afs_fs_check(void)
{
    afs.check(failsafe.AFS_last_valid_rc_ms);
}
#endif

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

/**
 * @brief Periodic tasks executed once per second
 * 
 * @details Performs various housekeeping and low-rate update tasks that don't need
 *          to run faster than 1Hz:
 * 
 *          1. Control Channel Update: Refreshes RC input channel assignments to allow
 *             runtime reconfiguration
 *          
 *          2. IO MCU Mixing Setup: Configures servo mixing on external IO controller
 *             (for boards with separate IO processors)
 *          
 *          3. ADSB Airspeed Limits: Updates ADSB system with current min/max airspeed
 *             parameters for collision avoidance calculations
 *          
 *          4. Default Airspeed Fusion: If enabled, provides AHRS with default airspeed
 *             estimate (average of min/max) with high variance for EKF fusion when
 *             airspeed sensor unavailable
 *          
 *          5. Auxiliary Servo Enable: Enables auxiliary servo outputs
 *          
 *          6. Notification Flags: Updates LED/buzzer notification system with:
 *             - Pre-arm check status
 *             - GPS check status
 *             - Armed state
 *          
 *          7. Terrain Logging: Logs terrain data if GPS logging enabled
 *          
 *          8. Home Position Update: Updates home position every 5 seconds when disarmed
 *             and GPS has good fix, resets landing altitude offset
 *          
 *          9. Timing Validation: Checks that scheduler loop period is correctly configured,
 *             logs internal error if G_Dt or loop rate don't match
 *          
 *          10. Notch Filter Update: Updates attitude controller notch filter sample rates
 *              based on measured loop rate for optimal harmonic rejection
 * 
 * @note Scheduled at 1Hz via SCHED_TASK with priority 90
 * @note Expected execution time: 400μs
 * @note Home position updates only when disarmed to prevent position jumps in flight
 * 
 * @see Plane::set_control_channels() for RC channel assignment
 * @see AP_Notify for notification LED/buzzer control
 * @see Plane::update_home() for home position setting
 * 
 * Source: ArduPlane/Plane.cpp:340-401
 */
void Plane::one_second_loop()
{
    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAL_WITH_IO_MCU
    iomcu.setup_mixing(g.override_channel.get(), g.mixing_gain, g2.manual_rc_mask);
#endif

#if HAL_ADSB_ENABLED
    adsb.set_stall_speed_cm(aparm.airspeed_min * 100); // convert m/s to cm/s
    adsb.set_max_speed(aparm.airspeed_max);
#endif

    if (flight_option_enabled(FlightOptions::ENABLE_DEFAULT_AIRSPEED)) {
        // use average of min and max airspeed as default airspeed fusion with high variance
        ahrs.writeDefaultAirSpeed((float)((aparm.airspeed_min + aparm.airspeed_max)/2),
                                  (float)((aparm.airspeed_max - aparm.airspeed_min)/2));
    }

    AP::srv().enable_aux_servos();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::Required::NO;

#if AP_TERRAIN_AVAILABLE && HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif

    // update home position if NOT armed and gps position has
    // changed. Update every 5s at most
    if (!arming.is_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            landing.alt_offset = 0;
    }

    // this ensures G_Dt is correct, catching startup issues with constructors
    // calling the scheduler methods
    if (!is_equal(1.0f/scheduler.get_loop_rate_hz(), scheduler.get_loop_period_s()) ||
        !is_equal(G_Dt, scheduler.get_loop_period_s())) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    const float loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.attitude_control->set_notch_sample_rate(loop_rate);
    }
#endif
    rollController.set_notch_sample_rate(loop_rate);
    pitchController.set_notch_sample_rate(loop_rate);
    yawController.set_notch_sample_rate(loop_rate);
}

/**
 * @brief Periodic tasks executed at 3Hz
 * 
 * @details Performs tasks that need to run 3 times per second. Currently used for:
 *          - Geofence checking: Monitors if vehicle is within configured fence boundaries
 *            and triggers appropriate failsafe actions if fence is breached
 * 
 * @note Scheduled at 3Hz via SCHED_TASK with priority 93
 * @note Expected execution time: 75μs
 * @note Fence checking only compiled in if AP_FENCE_ENABLED
 * 
 * @see Plane::fence_check() for fence violation detection and handling
 * @see AC_Fence for geofence implementation
 * 
 * Source: ArduPlane/Plane.cpp:403-408
 */
void Plane::three_hz_loop()
{
#if AP_FENCE_ENABLED
    fence_check();
#endif
}

#if AP_AIRSPEED_AUTOCAL_ENABLE
/**
 * @brief Automatically calibrate airspeed sensor ratio once per second
 * 
 * @details Performs in-flight automatic calibration of the airspeed sensor by comparing
 *          indicated airspeed with GPS ground speed and wind estimate. Over time, this
 *          adjusts the airspeed ratio to compensate for:
 *          - Pitot tube placement effects
 *          - Installation-specific pressure errors
 *          - Sensor drift over time
 * 
 *          Calibration is only performed when ALL conditions are met:
 *          - Soft armed (vehicle is armed)
 *          - fly_forward mode (not in VTOL or hover)
 *          - is_flying() (vehicle is airborne)
 *          - Airspeed sensor enabled
 *          - GPS 3D fix available
 *          - GPS ground speed >= 4 m/s (sufficient speed for accuracy)
 *          - Airspeed >= aparm.airspeed_min AND ground speed >= airspeed_min
 *            (prevents calibration at very low speeds where errors are large)
 *          - Roll angle <= roll_limit_cd (wings level or moderate bank)
 *          - Pitch angle within [pitch_limit_min, pitch_limit_max] (normal flight envelope)
 * 
 *          The calibration algorithm compares GPS velocity vector with indicated airspeed
 *          and wind estimate to determine the correct airspeed ratio. This is a gradual
 *          process that converges over multiple flights.
 * 
 * @note Scheduled at 1Hz via SCHED_TASK with priority 102
 * @note Expected execution time: 100μs
 * @note Only compiled if AP_AIRSPEED_AUTOCAL_ENABLE is defined
 * @note Calibration does not run in extreme attitudes to avoid corrupted measurements
 * 
 * @warning Incorrect calibration can lead to poor altitude/speed control. Automatic
 *          calibration should be monitored and verified against known good calibration.
 * 
 * @see AP_Airspeed::update_calibration() for calibration algorithm
 * @see AP_AHRS::get_fly_forward() for fly-forward state
 * 
 * Source: ArduPlane/Plane.cpp:414-442
 */
void Plane::airspeed_ratio_update(void)
{
    if (!hal.util->get_soft_armed() ||
        !ahrs.get_fly_forward() ||
        !is_flying() ||
        !airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.get_pitch_deg() > aparm.pitch_limit_max ||
        ahrs.get_pitch_deg() < pitch_limit_min) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
}
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

/**
 * @brief High-rate GPS update at 50Hz
 * 
 * @details Processes GPS receiver data at 50Hz and updates the vehicle's current location.
 *          This high rate update ensures we don't miss GPS messages and provides smooth
 *          position updates for navigation. The GPS library handles multiple receiver types
 *          (UBLOX, NMEA, SBF, etc.) and multiple GPS instances for redundancy.
 * 
 *          After updating GPS data, calls update_current_loc() to recalculate the vehicle's
 *          current position and relative altitude from home.
 * 
 * @note Scheduled at 50Hz via SCHED_TASK with priority 30
 * @note Expected execution time: 300μs
 * 
 * @see AP_GPS::update() for GPS message processing
 * @see Plane::update_current_loc() for position calculation
 * @see Plane::update_GPS_10Hz() for lower-rate GPS processing tasks
 * 
 * Source: ArduPlane/Plane.cpp:447-452
 */
void Plane::update_GPS_50Hz(void)
{
    gps.update();

    update_current_loc();
}

/**
 * @brief Lower-rate GPS processing at 10Hz
 * 
 * @details Performs GPS-dependent tasks that don't need to run at the full 50Hz GPS rate:
 *          
 *          1. Home Position Initialization: During startup, counts down good GPS fixes
 *             (ground_start_count) before setting home position. This ensures accurate
 *             initial altitude by waiting for GPS altitude to stabilize (typically 5 fixes).
 *          
 *          2. Home Position Setting: Once ground_start_count reaches 1 and we have valid
 *             position data, sets home position persistently and initializes waypoint tracking.
 *          
 *          3. Wind Estimation: Updates wind velocity estimate using GPS ground speed and
 *             airspeed measurements, which is used for navigation and TECS control.
 *          
 *          4. Ground Speed Undershoot Calculation: Calculates how much ground speed is
 *             below the target, accounting for wind effects.
 * 
 *          If GPS fix quality degrades below 3D, resets ground_start_count to require
 *          re-initialization when fix is regained.
 * 
 * @note Scheduled at 10Hz via SCHED_TASK with priority 33
 * @note Expected execution time: 400μs
 * @note Requires GPS_OK_FIX_3D or better for home position setting
 * 
 * @see Plane::update_GPS_50Hz() for high-rate GPS data acquisition
 * @see AP_AHRS::estimate_wind() for wind estimation algorithm
 * @see Plane::calc_gndspeed_undershoot() for ground speed tracking
 * 
 * Source: ArduPlane/Plane.cpp:457-491
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else if (!hal.util->was_watchdog_reset()) {
                if (!set_home_persistently(gps.location())) {
                    // silently ignore failure...
                }

                next_WP_loc = prev_WP_loc = home;

                ground_start_count = 0;
            }
        }

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/**
 * @brief Execute current flight mode control logic - FAST_TASK executed every loop
 * 
 * @details This critical method calls the update() function of the current flight mode,
 *          allowing each mode to execute its specific control logic. This is a FAST_TASK
 *          running at full loop rate (~400Hz) to ensure responsive control.
 * 
 *          Pre-update operations:
 *          - Clears hold_course_cd if not in AUTO or TAKEOFF mode (heading hold only
 *            needed during takeoff and landing)
 *          - Refreshes throttle limits to avoid using stale takeoff throttle constraints
 *          - Updates fly_forward state for AHRS based on vehicle configuration
 * 
 *          Post-update operations:
 *          - Checks takeoff direction for AUTOLAND mode if enabled
 * 
 *          Each flight mode's update() method typically:
 *          - Calculates desired attitude/altitude/speed targets
 *          - Manages mode-specific state machines
 *          - Handles mode-specific safety checks
 *          - Updates navigation waypoints
 * 
 * @note This method runs at full loop rate (~400Hz) as a FAST_TASK
 * @note The actual control calculations happen in each mode's update() implementation
 * 
 * @see Mode::update() for mode-specific control implementation
 * @see Plane::update_fly_forward() for fly-forward state management
 * @see Plane::stabilize() which runs after this to convert targets to servo outputs
 * 
 * Source: ArduPlane/Plane.cpp:496-514
 */
void Plane::update_control_mode(void)
{
    if ((control_mode != &mode_auto) && (control_mode != &mode_takeoff)) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }
    // refresh the throttle limits, to avoid using stale values
    // they will be updated once takeoff_calc_throttle is called
    takeoff_state.throttle_lim_max = 100.0f;
    takeoff_state.throttle_lim_min = -100.0f;

    update_fly_forward();

    control_mode->update();

#if MODE_AUTOLAND_ENABLED
    mode_autoland.check_takeoff_direction();
#endif
}


/**
 * @brief Update fly-forward state for AHRS and EKF
 * 
 * @details Sets whether the vehicle is flying forward as a fixed-wing aircraft, which
 *          allows the AHRS/EKF to make stronger assumptions about vehicle motion and
 *          produce better state estimates. The fly-forward flag affects:
 *          - Airspeed sensor fusion in the EKF
 *          - Wind estimation algorithms
 *          - GPS velocity weighting
 * 
 *          Fly-forward is set to TRUE when:
 *          - Operating as a pure fixed-wing aircraft in normal flight
 *          - Tailsitter in forward flight mode
 * 
 *          Fly-forward is set to FALSE when:
 *          - Quadplane in VTOL mode (multirotor control)
 *          - Quadplane in assisted flight (VTOL motors active)
 *          - In idle mode (balloon lift)
 *          - Landing with is_flying_forward() returning false
 * 
 * @note This affects EKF state estimation quality - incorrect settings can cause
 *       position or velocity estimate errors
 * 
 * @see AP_AHRS::set_fly_forward() for AHRS state update
 * @see AP_NavEKF3 for how fly-forward affects EKF sensor fusion
 * @see Plane::landing.is_flying_forward() for landing phase detection
 * 
 * Source: ArduPlane/Plane.cpp:517-553
 */
void Plane::update_fly_forward(void)
{
    // ensure we are fly-forward when we are flying as a pure fixed
    // wing aircraft. This helps the EKF produce better state
    // estimates as it can make stronger assumptions
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        if (quadplane.tailsitter.is_in_fw_flight()) {
            ahrs.set_fly_forward(true);
            return;
        }

        if (quadplane.in_vtol_mode()) {
            ahrs.set_fly_forward(false);
            return;
        }

        if (quadplane.in_assisted_flight()) {
            ahrs.set_fly_forward(false);
            return;
        }
    }
#endif

    if (auto_state.idle_mode) {
        // don't fuse airspeed when in balloon lift
        ahrs.set_fly_forward(false);
        return;
    }

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        ahrs.set_fly_forward(landing.is_flying_forward());
        return;
    }

    ahrs.set_fly_forward(true);
}

/**
 * @brief Set the current flight stage and handle stage transitions
 * 
 * @details Updates the flight_stage variable and performs necessary actions when
 *          transitioning between flight stages. Stage changes affect:
 *          - TECS controller parameters and behavior
 *          - Landing gear deployment/retraction
 *          - Logging and GCS notifications
 *          - Flight mode state management
 * 
 *          Stage transition actions:
 *          - Entering LAND stage: Deploys landing gear (if equipped)
 *          - Completing TAKEOFF stage (transitioning to NORMAL): Retracts landing gear
 *            if armed, safety off, and is_flying()
 *          - Entering ABORT_LANDING stage: Sends GCS notification with target climb altitude
 * 
 *          All stage changes trigger:
 *          - landing.handle_flight_stage_change() for landing-specific logic
 *          - Log_Write_Status() to record stage change in logs
 * 
 * @param[in] fs New flight stage to set (TAKEOFF, NORMAL, LAND, ABORT_LANDING, VTOL)
 * 
 * @note Stage changes only occur if fs != current flight_stage (no redundant updates)
 * @note ABORT_LANDING stage is "sticky" - once set, persists through landing sequence
 * 
 * @see AP_FixedWing::FlightStage enum for available stages
 * @see Plane::update_flight_stage() for automatic stage determination
 * @see AP_TECS for how stages affect control parameters
 * @see AP_Landing::handle_flight_stage_change() for landing logic
 * 
 * Source: ArduPlane/Plane.cpp:558-591
 */
void Plane::set_flight_stage(AP_FixedWing::FlightStage fs)
{
    if (fs == flight_stage) {
        return;
    }

    const bool is_landing = (fs == AP_FixedWing::FlightStage::LAND);

    landing.handle_flight_stage_change(is_landing);

#if AP_LANDINGGEAR_ENABLED
    if (is_landing) {
        plane.g2.landing_gear.deploy_for_landing();
    }

    const bool is_takeoff_complete = (flight_stage == AP_FixedWing::FlightStage::TAKEOFF &&
                                      fs == AP_FixedWing::FlightStage::NORMAL);
    if (is_takeoff_complete &&
        arming.is_armed_and_safety_off() &&
        is_flying()) {
            g2.landing_gear.retract_after_takeoff();
    }
#endif

    if (fs == AP_FixedWing::FlightStage::ABORT_LANDING) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm",
                        int(auto_state.takeoff_altitude_rel_cm/100));
    }

    flight_stage = fs;
#if HAL_LOGGING_ENABLED
    Log_Write_Status();
#endif
}

/**
 * @brief Update altitude estimate and TECS pitch/throttle controller at 10Hz
 * 
 * @details This method performs critical altitude-related updates:
 * 
 *          1. Barometer Update: Reads barometric pressure sensor for altitude estimation
 *          
 *          2. Sink Rate Calculation: Determines vertical velocity from:
 *             - AHRS velocity estimate (preferred, uses EKF fusion)
 *             - GPS vertical velocity (if AHRS unavailable)
 *             - Barometer climb rate (fallback if no GPS vertical velocity)
 *             Low-pass filtered to remove noise (0.8 old + 0.2 new)
 *          
 *          3. Flight Stage Update: Determines current flight phase (TAKEOFF, NORMAL, LAND, VTOL, ABORT_LANDING)
 *          
 *          4. TECS Pitch/Throttle Calculation: If conditions are met, calls TECS to calculate
 *             optimal pitch and throttle commands to maintain target altitude and airspeed:
 *             - Only runs when throttle is not suppressed
 *             - Disabled in idle mode, VTOL modes, or during scripting tricks
 *             - Uses current flight stage to adjust behavior (e.g., landing flare)
 *             - Applies RTL climb-out requirements if needed
 *             - Accounts for distance beyond landing waypoint during landing
 * 
 *          TECS (Total Energy Control System) manages the coupling between altitude and
 *          airspeed by treating them as forms of energy (potential and kinetic) and
 *          optimally allocating pitch and throttle to maintain both.
 * 
 * @note Scheduled at 10Hz via SCHED_TASK with priority 45
 * @note Expected execution time: 200μs
 * @note Sink rate is provided to parachute system for deployment logic
 * 
 * @warning Disabling TECS inappropriately can cause altitude/speed control failures
 * 
 * @see AP_Baro::update() for barometer data acquisition
 * @see AP_TECS::update_pitch_throttle() for energy management calculations
 * @see Plane::update_flight_stage() for flight phase determination
 * @see Plane::tecs_hgt_afe() for height above field elevation calculation
 * 
 * Source: ArduPlane/Plane.cpp:593-667
 */
void Plane::update_alt()
{
    barometer.update();

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if HAL_PARACHUTE_ENABLED
    parachute.set_sink_rate(auto_state.sink_rate);
#endif

    update_flight_stage();

#if AP_SCRIPTING_ENABLED
    if (nav_scripting_active()) {
        // don't call TECS while we are in a trick
        return;
    }
#endif

    bool should_run_tecs = control_mode->does_auto_throttle();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.should_disable_TECS()) {
        should_run_tecs = false;
    }
#endif

    if (auto_state.idle_mode) {
        should_run_tecs = false;
    }

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (mode_auto.in_pullup()) {
        should_run_tecs = false;
    }
#endif
    
    if (should_run_tecs && !throttle_suppressed) {

        float distance_beyond_land_wp = 0;
        if (flight_stage == AP_FixedWing::FlightStage::LAND &&
            current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = current_loc.get_distance(next_WP_loc);
        }

        tecs_target_alt_cm = relative_target_altitude_cm();

        if (control_mode == &mode_rtl && !rtl.done_climb && (g2.rtl_climb_min > 0 || (plane.flight_option_enabled(FlightOptions::CLIMB_BEFORE_TURN)))) {
            // ensure we do the initial climb in RTL. We add an extra
            // 10m in the demanded height to push TECS to climb
            // quickly
            tecs_target_alt_cm = MAX(tecs_target_alt_cm, prev_WP_loc.alt - home.alt) + (g2.rtl_climb_min+10)*100;
        }

        TECS_controller.update_pitch_throttle(tecs_target_alt_cm,
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor,
                                                 g.pitch_trim.get());
    }
}

/**
 * @brief Recalculate and update the current flight stage
 * 
 * @details Determines the current phase of flight which affects TECS behavior and control
 *          law selection. Flight stages include:
 * 
 *          - TAKEOFF: From takeoff initiation until takeoff_complete flag is set
 *                     (typically based on altitude gain). TECS uses special takeoff parameters.
 *          
 *          - NORMAL: Cruise flight. TECS operates in normal mode balancing altitude and airspeed.
 *          
 *          - LAND: During landing approach and flare. TECS transitions to landing parameters
 *                  and manages the landing flare when close to touchdown.
 *          
 *          - ABORT_LANDING: Go-around maneuver. TECS applies climb-out settings. This stage
 *                          is "sticky" - once entered, it persists while executing NAV_LAND.
 *          
 *          - VTOL: Quadplane VTOL operations. TECS is typically disabled, control passes
 *                  to quadplane position controllers.
 * 
 *          Flight stage changes trigger:
 *          - Landing gear deployment/retraction
 *          - TECS parameter switching
 *          - Status logging
 *          - Flight mode state changes
 * 
 *          Stage determination logic:
 *          1. Check for VTOL mode (if quadplane enabled)
 *          2. Check for TAKEOFF stage (auto_state.takeoff_complete == false)
 *          3. Check for LAND stage (mission command == NAV_LAND)
 *          4. Check for ABORT_LANDING (commanded go-around or throttle stick go-around)
 *          5. Default to NORMAL for other auto throttle modes
 * 
 * @note Called from update_alt() at 10Hz
 * @note Flight stage affects TECS gains, limits, and control behavior
 * 
 * @see Plane::set_flight_stage() for stage change handling
 * @see AP_TECS for how flight stage affects control parameters
 * @see AP_Landing for landing-specific flight stage logic
 * 
 * Source: ArduPlane/Plane.cpp:672-726
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (control_mode->does_auto_throttle() && !throttle_suppressed) {
        if (control_mode == &mode_auto) {
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
                return;
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if (landing.is_commanded_go_around() || flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else if (landing.get_abort_throttle_enable() && get_throttle_input() >= 90 &&
                           landing.request_go_around()) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Landing aborted via throttle");
                    set_flight_stage(AP_FixedWing::FlightStage::ABORT_LANDING);
                } else {
                    set_flight_stage(AP_FixedWing::FlightStage::LAND);
                }
                return;
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_FixedWing::FlightStage::VTOL);
                return;
            }
#endif
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        } else if ((control_mode != &mode_takeoff)
#if MODE_AUTOLAND_ENABLED
             && (control_mode != &mode_autoland)
#endif
             ) {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
            set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
        }
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() ||
        quadplane.in_assisted_flight()) {
        set_flight_stage(AP_FixedWing::FlightStage::VTOL);
        return;
    }
#endif
    set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
}




/**
 * @brief Automatically disarm after landing if configured delay has elapsed
 * 
 * @details Implements automatic disarming after a successful landing when land_DisarmDelay
 *          parameter is set to a non-zero value. This is a safety feature that prevents
 *          the vehicle from remaining armed on the ground indefinitely.
 * 
 *          Auto-disarm conditions (all must be true):
 *          - land_DisarmDelay > 0 (feature enabled)
 *          - !is_flying() (vehicle has landed)
 *          - arming_required() != NO (arming is required for this vehicle)
 *          - is_armed() (vehicle is currently armed)
 *          - Time since last_flying_ms >= land_DisarmDelay seconds
 * 
 *          When all conditions are met, initiates disarm with Method::AUTOLANDED and
 *          sends "Auto disarmed" message to ground control station.
 * 
 * @note Only called from AP_Landing when the landing library determines it's safe to disarm
 * @note This prevents accidental disarm during hard landings or bounces
 * @note If arming is not required (AP_Arming::Required::NO), auto-disarm is skipped
 * 
 * @see AP_Landing::update() where this method is called
 * @see AP_Arming::disarm() for disarm implementation
 * @see Plane::is_flying() for flight detection logic
 * 
 * Source: ArduPlane/Plane.cpp:736-749
 */
void Plane::disarm_if_autoland_complete()
{
    if (landing.get_disarm_delay() > 0 &&
        !is_flying() &&
        arming.arming_required() != AP_Arming::Required::NO &&
        arming.is_armed()) {
        /* we have auto disarm enabled. See if enough time has passed */
        if (millis() - auto_state.last_flying_ms >= landing.get_disarm_delay()*1000UL) {
            if (arming.disarm(AP_Arming::Method::AUTOLANDED)) {
                gcs().send_text(MAV_SEVERITY_INFO,"Auto disarmed");
            }
        }
    }
}

/**
 * @brief Trigger an aborted landing (go-around) maneuver
 * 
 * @details Initiates a go-around procedure when called during a landing approach or landing.
 *          This method implements several abort strategies in priority order:
 * 
 *          1. Quadplane VTOL Abort: If in quadplane VTOL auto mode, calls quadplane-specific
 *             abort logic which may transition to VTOL hover or climb
 * 
 *          2. User-Planned Abort Sequence: If a DO_LAND_START mission item exists with an
 *             abort sequence, jumps to that mission sequence for a pre-planned abort path
 * 
 *          3. Fixed-Wing Go-Around: If not using quadplane or fixed-wing spiral approach:
 *             - Initiates pitch-up and climb-out to safe altitude
 *             - Holds current heading during climb
 *             - After reaching abort altitude, one of these actions occurs (checked in order):
 *               a) Execute MAV_CMD_CONTINUE_AND_CHANGE_ALT if it's the next mission item
 *               b) Jump to DO_LAND_START if available for another approach
 *               c) Decrement mission index to repeat the landing approach
 * 
 *          Only triggers abort if:
 *          - Currently in AUTO mode
 *          - In LAND flight stage OR executing a land command (NAV_LAND, etc.)
 * 
 * @param[in] climb_to_alt_m Target altitude for climb-out in meters above home.
 *                           If zero, uses default takeoff_altitude_rel_cm.
 * 
 * @return true if abort was successfully initiated, false if conditions not met
 * 
 * @note This can be called from external sources (scripting, avoidance, pilot command)
 * @note Quadplane spiral approaches are not aborted with fixed-wing logic
 * @note Sets auto_state.next_wp_crosstrack = false to prevent crosstrack correction
 *       during the climb-out
 * 
 * @warning Aborting too late in the landing may not provide sufficient altitude for recovery
 * 
 * @see Plane::landing.request_go_around() for landing library abort initiation
 * @see Mission::jump_to_abort_landing_sequence() for mission abort sequences
 * @see QuadPlane::abort_landing() for VTOL-specific abort
 * 
 * Source: ArduPlane/Plane.cpp:751-800
 */
bool Plane::trigger_land_abort(const float climb_to_alt_m)
{
    if (plane.control_mode != &plane.mode_auto) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        return quadplane.abort_landing();
    }
#endif

    uint16_t mission_id = plane.mission.get_current_nav_cmd().id;
    bool is_in_landing = (plane.flight_stage == AP_FixedWing::FlightStage::LAND) ||
        plane.is_land_command(mission_id);
    if (is_in_landing) {
        // fly a user planned abort pattern if available
        if (plane.have_position && plane.mission.jump_to_abort_landing_sequence(plane.current_loc)) {
            return true;
        }

        // only fly a fixed wing abort if we aren't doing quadplane stuff, or potentially
        // shooting a quadplane approach
#if HAL_QUADPLANE_ENABLED
        const bool attempt_go_around =
            (!plane.quadplane.available()) ||
            ((!plane.quadplane.in_vtol_auto()) &&
                (!plane.quadplane.landing_with_fixed_wing_spiral_approach()));
#else
        const bool attempt_go_around = true;
#endif
        if (attempt_go_around) {
            // Initiate an aborted landing. This will trigger a pitch-up and
            // climb-out to a safe altitude holding heading then one of the
            // following actions will occur, check for in this order:
            // - If MAV_CMD_CONTINUE_AND_CHANGE_ALT is next command in mission,
            //      increment mission index to execute it
            // - else if DO_LAND_START is available, jump to it
            // - else decrement the mission index to repeat the landing approach

            if (!is_zero(climb_to_alt_m)) {
                plane.auto_state.takeoff_altitude_rel_cm = climb_to_alt_m * 100;
            }
            if (plane.landing.request_go_around()) {
                plane.auto_state.next_wp_crosstrack = false;
                return true;
            }
        }
    }
    return false;
}


/**
 * @brief Calculate height above field elevation for TECS controller
 * 
 * @details Provides TECS with the appropriate height reference depending on flight stage:
 * 
 *          During LAND stage:
 *          - Uses height_above_target() which is height above landing point
 *          - Applies rangefinder correction if available (subtracts rangefinder offset)
 *          - If external HAGL (Height Above Ground Level) is active, uses that
 *          - This allows TECS to detect proximity to ground and initiate landing flare
 * 
 *          During other stages (TAKEOFF, NORMAL, VTOL, ABORT_LANDING):
 *          - Uses relative_altitude which is altitude relative to home position
 *          - Standard reference for cruise flight altitude control
 * 
 *          The different height references are critical for proper TECS operation:
 *          - Landing requires height above landing point for flare timing
 *          - Normal flight uses height above home for waypoint altitude tracking
 * 
 *          Rangefinder correction accounts for variations in ground elevation during
 *          landing approach, improving flare timing accuracy.
 * 
 * @return Height above field elevation in meters
 * 
 * @note Called from update_alt() when calculating TECS pitch/throttle commands
 * @note Rangefinder data quality affects landing flare performance
 * 
 * @see AP_TECS::update_pitch_throttle() where this value is used
 * @see Plane::height_above_target() for landing height calculation
 * @see Plane::rangefinder_correction() for terrain-following offset
 * 
 * Source: ArduPlane/Plane.cpp:806-835
 */
float Plane::tecs_hgt_afe(void)
{
    /*
      pass the height above field elevation as the height above
      the ground when in landing, which means that TECS gets the
      rangefinder information and thus can know when the flare is
      coming.
    */
    float hgt_afe;

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {

        #if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
            // if external HAGL is active use that
            if (get_external_HAGL(hgt_afe)) {
                return hgt_afe;
            }
        #endif

        hgt_afe = height_above_target();
#if AP_RANGEFINDER_ENABLED
        hgt_afe -= rangefinder_correction();
#endif
    } else {
        // when in normal flight we pass the hgt_afe as relative
        // altitude to home
        hgt_afe = relative_altitude;
    }
    return hgt_afe;
}

/**
 * @brief Get distance to current waypoint in meters
 * 
 * @details Retrieves the distance from the vehicle to the currently active waypoint.
 *          Behavior varies by flight mode and vehicle type:
 * 
 *          - MANUAL mode: Returns false (no active waypoint)
 *          - Quadplane VTOL modes: Returns distance from quadplane waypoint navigator
 *            (in centimeters converted to meters), or 0 if not using waypoint navigation
 *          - Fixed-wing modes: Returns auto_state.wp_distance (calculated by L1 controller)
 * 
 *          This distance is used for:
 *          - GCS telemetry (NAV_CONTROLLER_OUTPUT message)
 *          - Mission progress monitoring
 *          - Waypoint arrival detection
 *          - Flight director guidance displays
 * 
 * @param[out] distance Distance to waypoint in meters. Set to 0 if no waypoint active.
 * 
 * @return true if distance is valid and vehicle has an active waypoint
 * @return false if in MANUAL mode or no waypoint navigation active
 * 
 * @note Distance is horizontal (2D) distance, not slant range
 * @note For quadplane, checks if wp_nav is being used (some VTOL modes don't use it)
 * 
 * @see GCS_MAVLINK_Plane::send_nav_controller_output() where this is called
 * @see AC_WPNav::get_wp_distance_to_destination_cm() for quadplane calculation
 * @see AP_L1_Control for fixed-wing waypoint distance calculation
 * 
 * Source: ArduPlane/Plane.cpp:838-852
 */
bool Plane::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        distance = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_distance_to_destination_cm() * 0.01 : 0;
        return true;
    }
#endif
    distance = auto_state.wp_distance;
    return true;
}

/**
 * @brief Get bearing to current waypoint in degrees
 * 
 * @details Retrieves the bearing from the vehicle to the currently active waypoint
 *          in degrees (0-360, with 0=North, 90=East, 180=South, 270=West).
 *          Behavior varies by flight mode and vehicle type:
 * 
 *          - MANUAL mode: Returns false (no active waypoint)
 *          - Quadplane VTOL modes: Returns bearing from quadplane waypoint navigator
 *            (converted from centidegrees to degrees), or 0 if not using waypoint navigation
 *          - Fixed-wing modes: Returns target_bearing from L1 navigation controller
 *            (converted from centidegrees to degrees)
 * 
 *          This bearing is used for:
 *          - GCS telemetry (NAV_CONTROLLER_OUTPUT message)
 *          - Flight director displays showing where to turn
 *          - Course deviation indicators
 *          - Navigation information on OSD
 * 
 * @param[out] bearing Bearing to waypoint in degrees (0-360). Set to 0 if no waypoint active.
 * 
 * @return true if bearing is valid and vehicle has an active waypoint
 * @return false if in MANUAL mode or no waypoint navigation active
 * 
 * @note Bearing is true bearing (relative to true north, not magnetic)
 * @note For fixed-wing, this is the desired track to the waypoint (L1 controller output)
 * 
 * @see GCS_MAVLINK_Plane::send_nav_controller_output() where this is called
 * @see AC_WPNav::get_wp_bearing_to_destination_cd() for quadplane calculation
 * @see AP_L1_Control::target_bearing_cd() for fixed-wing bearing calculation
 * 
 * Source: ArduPlane/Plane.cpp:854-868
 */
bool Plane::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        bearing = quadplane.using_wp_nav() ? quadplane.wp_nav->get_wp_bearing_to_destination_cd() : 0;
        return true;
    }
#endif
    bearing = nav_controller->target_bearing_cd() * 0.01;
    return true;
}

/**
 * @brief Get crosstrack error to flight path in meters
 * 
 * @details Retrieves the perpendicular distance from the vehicle to the desired flight path
 *          (the line between previous and current waypoints). Positive values indicate the
 *          vehicle is to the right of the desired track, negative values indicate left.
 * 
 *          Behavior varies by flight mode and vehicle type:
 *          - MANUAL mode: Returns false (no flight path)
 *          - Quadplane VTOL modes: Returns crosstrack error from quadplane waypoint navigator,
 *            or 0 if not using waypoint navigation
 *          - Fixed-wing modes: Returns crosstrack error from L1 navigation controller
 * 
 *          Crosstrack error is used for:
 *          - Course deviation indicator (CDI) displays on GCS
 *          - Navigation accuracy assessment
 *          - Determining if vehicle needs to turn back to track
 *          - Flight path following performance monitoring
 * 
 *          The L1 controller uses this error to calculate the required bank angle to
 *          return to the desired track while maintaining smooth turns.
 * 
 * @param[out] xtrack_error Crosstrack error in meters. Positive = right of track,
 *                          negative = left of track. Set to 0 if no flight path active.
 * 
 * @return true if crosstrack error is valid and vehicle is following a flight path
 * @return false if in MANUAL mode or no flight path navigation active
 * 
 * @note Crosstrack is perpendicular distance, not along-track distance
 * @note Large crosstrack errors may trigger the L1 controller to command tighter turns
 * 
 * @see GCS_MAVLINK_Plane::send_nav_controller_output() where this is called
 * @see AC_WPNav::crosstrack_error() for quadplane calculation
 * @see AP_L1_Control::crosstrack_error() for fixed-wing calculation
 * 
 * Source: ArduPlane/Plane.cpp:870-884
 */
bool Plane::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Plane::send_nav_controller_output()
    if (control_mode == &mode_manual) {
        return false;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode()) {
        xtrack_error = quadplane.using_wp_nav() ? quadplane.wp_nav->crosstrack_error() : 0;
        return true;
    }
#endif
    xtrack_error = nav_controller->crosstrack_error();
    return true;
}

#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
/**
 * @brief Set target waypoint location from external control or scripting
 * 
 * @details Allows external control systems (companion computers) or Lua scripts to
 *          command the vehicle to fly to a specific location. This method:
 * 
 *          1. Validates that vehicle is in GUIDED mode (required for external commands)
 *          2. Applies terrain altitude correction if needed via fix_terrain_WP()
 *          3. Converts relative altitudes to absolute altitude frame if necessary
 *          4. Commands the vehicle to fly to the specified location
 * 
 *          Security considerations:
 *          - Only accepts location updates in GUIDED mode (prevents accidental override
 *            of AUTO missions or other autonomous modes)
 *          - Terrain altitude handling ensures safe operation over varying terrain
 * 
 *          Typical use cases:
 *          - Companion computer sending navigation commands
 *          - Lua scripts implementing custom navigation logic
 *          - External path planning systems
 *          - Obstacle avoidance systems providing new waypoints
 * 
 * @param[in] target_loc Target location to fly to. Can be in any altitude frame
 *                       (ABSOLUTE, RELATIVE, TERRAIN). Terrain altitude will be
 *                       fixed if terrain following is enabled.
 * 
 * @return true if target location was accepted and set
 * @return false if not in GUIDED mode or location is invalid
 * 
 * @note Only compiled if AP_SCRIPTING_ENABLED or AP_EXTERNAL_CONTROL_ENABLED
 * @note Location must have valid lat/lon coordinates
 * @note Altitude frame conversion happens automatically
 * 
 * @warning External control should monitor vehicle progress and provide reasonable waypoints
 * 
 * @see Plane::set_guided_WP() for actual waypoint command implementation
 * @see Plane::fix_terrain_WP() for terrain altitude correction
 * @see Mode::Guided for GUIDED mode implementation
 * 
 * Source: ArduPlane/Plane.cpp:888-903
 */
bool Plane::set_target_location(const Location &target_loc)
{
    Location loc{target_loc};
    fix_terrain_WP(loc, __LINE__);

    if (plane.control_mode != &plane.mode_guided) {
        // only accept position updates when in GUIDED mode
        return false;
    }
    // add home alt if needed
    if (!loc.terrain_alt) {
        loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }
    plane.set_guided_WP(loc);
    return true;
}
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
/**
 * @brief Get current target waypoint location for scripting access
 * 
 * @details Retrieves the vehicle's current navigation target location (next_WP_loc)
 *          for use by Lua scripts. This allows scripts to:
 *          - Monitor where the vehicle is currently navigating to
 *          - Make decisions based on target location
 *          - Implement custom navigation logic that modifies the target
 *          - Coordinate with the autopilot's navigation system
 * 
 *          Target location is available in these modes:
 *          - RTL: Return home location
 *          - AVOID_ADSB: Avoidance waypoint
 *          - GUIDED: Commanded waypoint
 *          - AUTO: Current mission waypoint
 *          - LOITER: Loiter center point
 *          - TAKEOFF: Takeoff waypoint
 *          - QLOITER: Quadplane loiter center (if quadplane enabled)
 *          - QLAND: Quadplane landing point (if quadplane enabled)
 *          - QRTL: Quadplane RTL location (if quadplane enabled)
 * 
 *          For modes without a specific target location (MANUAL, STABILIZE, etc.),
 *          this method returns false.
 * 
 * @param[out] target_loc Populated with the current target waypoint location.
 *                        Includes latitude, longitude, altitude, and altitude frame.
 * 
 * @return true if vehicle has a valid target location in current mode
 * @return false if current mode does not have a navigation target
 * 
 * @note Only compiled if AP_SCRIPTING_ENABLED
 * @note The returned location is next_WP_loc (the waypoint being navigated toward)
 * @note For AUTO mode, this is the current mission waypoint
 * 
 * @see Plane::next_WP_loc for the target location variable
 * @see Plane::update_target_location() to modify the target from scripts
 * 
 * Source: ArduPlane/Plane.cpp:908-929
 */
bool Plane::get_target_location(Location& target_loc)
{
    switch (control_mode->mode_number()) {
    case Mode::Number::RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::AUTO:
    case Mode::Number::LOITER:
    case Mode::Number::TAKEOFF:
#if HAL_QUADPLANE_ENABLED
    case Mode::Number::QLOITER:
    case Mode::Number::QLAND:
    case Mode::Number::QRTL:
#endif
        target_loc = next_WP_loc;
        return true;
        break;
    default:
        break;
    }
    return false;
}

/**
 * @brief Update the current target waypoint location from scripting
 * 
 * @details Allows Lua scripts to modify the vehicle's current navigation target (next_WP_loc)
 *          in all automatic navigation modes. This enables scripts to:
 *          - Dynamically adjust waypoint locations during flight
 *          - Implement custom path planning algorithms
 *          - Respond to real-time sensor data or external inputs
 *          - Create dynamic navigation behaviors
 * 
 *          Race condition prevention:
 *          The method requires the caller to provide both the old and new locations.
 *          By verifying that old_loc matches the current next_WP_loc, we prevent race
 *          conditions where:
 *          - The mode changed between when the script read the location and updated it
 *          - Another script or the autopilot changed the waypoint
 *          - A mission command changed the target
 * 
 *          Process:
 *          1. Verify old_loc matches current next_WP_loc (location and altitude frame)
 *          2. Update next_WP_loc to new_loc
 *          3. Apply terrain altitude correction via fix_terrain_WP()
 *          4. For QLAND/QLOITER modes, update last_target_loc_set_ms timestamp
 * 
 *          Works in these modes:
 *          - All auto navigation modes (AUTO, GUIDED, RTL, LOITER, etc.)
 *          - Quadplane modes (QLOITER, QLAND, QRTL)
 * 
 * @param[in] old_loc Expected current target location. Must exactly match next_WP_loc
 *                    including altitude frame, or update will be rejected.
 * @param[in] new_loc New target location to navigate toward. Altitude frame must match
 *                    old_loc altitude frame.
 * 
 * @return true if waypoint was successfully updated
 * @return false if old_loc doesn't match current target or altitude frames differ
 * 
 * @note Only compiled if AP_SCRIPTING_ENABLED
 * @note Terrain altitude correction is automatically applied
 * @note Crosstrack correction is NOT reset - vehicle continues optimized path
 * 
 * @warning Script must read current location with get_target_location() immediately
 *          before calling this to minimize race condition window
 * 
 * @see Plane::get_target_location() to read current target
 * @see Plane::fix_terrain_WP() for terrain altitude handling
 * @see Location::same_loc_as() for location comparison logic
 * 
 * Source: ArduPlane/Plane.cpp:934-956
 */
bool Plane::update_target_location(const Location &old_loc, const Location &new_loc)
{
    /*
      by checking the caller has provided the correct old target
      location we prevent a race condition where the user changes mode
      or commands a different target in the controlling lua script
     */
    if (!old_loc.same_loc_as(next_WP_loc) ||
        old_loc.get_alt_frame() != new_loc.get_alt_frame()) {
        return false;
    }
    next_WP_loc = new_loc;

    fix_terrain_WP(next_WP_loc, __LINE__);

#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qland || control_mode == &mode_qloiter) {
        mode_qloiter.last_target_loc_set_ms = AP_HAL::millis();
    }
#endif

    return true;
}

/**
 * @brief Set velocity matching target for VTOL position control
 * 
 * @details Enables the quadplane position controller to match a moving target's velocity,
 *          useful for operations such as:
 *          - Following a moving vehicle (truck, boat, etc.)
 *          - Capturing a moving object
 *          - Formation flying with other aircraft
 *          - Landing on a moving platform
 * 
 *          When velocity matching is active, the quadplane position controller adds the
 *          target velocity to its commanded velocities, allowing the vehicle to maintain
 *          position relative to a moving target rather than a fixed GPS coordinate.
 * 
 *          The velocity match command is time-limited and will expire if not refreshed,
 *          preventing runaway behavior if the external system stops sending updates.
 * 
 *          Requirements:
 *          - Only works in quadplane VTOL modes or during VTOL landing sequences
 *          - Vehicle must have quadplane capability (HAL_QUADPLANE_ENABLED)
 * 
 * @param[in] velocity Target velocity vector in m/s, NED frame (North, East components only).
 *                     Typically represents the velocity of the target being tracked.
 *                     Example: {2.0, 1.0} means target moving 2 m/s north, 1 m/s east.
 * 
 * @return true if velocity matching was set (in VTOL mode or landing)
 * @return false if not in VTOL mode or quadplane not available
 * 
 * @note Only compiled if HAL_QUADPLANE_ENABLED and AP_SCRIPTING_ENABLED
 * @note Velocity match has a timeout - must be refreshed periodically
 * @note Z-axis (down) velocity is not used; altitude control remains independent
 * 
 * @see QuadPlane::poscontrol.velocity_match for storage
 * @see QuadPlane::poscontrol.last_velocity_match_ms for timeout tracking
 * @see AC_PosControl for position controller that uses this
 * 
 * Source: ArduPlane/Plane.cpp:959-969
 */
bool Plane::set_velocity_match(const Vector2f &velocity)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_mode() || quadplane.in_vtol_land_sequence()) {
        quadplane.poscontrol.velocity_match = velocity;
        quadplane.poscontrol.last_velocity_match_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

/**
 * @brief Override quadplane landing descent rate from scripting
 * 
 * @details Allows Lua scripts or external control to dynamically adjust the descent rate
 *          during VTOL landing, enabling:
 *          - Slower descents when detecting obstacles below
 *          - Faster descents in favorable conditions
 *          - Adaptive landing based on real-time sensor data
 *          - Precision landing with rangefinder feedback
 *          - Gentle touchdown on uneven terrain
 * 
 *          The override is time-limited and will expire if not refreshed, ensuring the
 *          vehicle reverts to normal landing parameters if the script stops updating.
 * 
 *          Requirements:
 *          - Must be in VTOL landing descent phase OR QLAND mode
 *          - Vehicle must have quadplane capability (HAL_QUADPLANE_ENABLED)
 * 
 *          Safety considerations:
 *          - Descent rate limits in position controller still apply
 *          - Vehicle will not exceed configured maximum descent rates
 *          - Override expires automatically to prevent stuck descent rates
 * 
 * @param[in] descent_rate Desired descent rate in m/s (positive = down).
 *                         Example: 0.5 means descend at 0.5 m/s (slow)
 *                                  2.0 means descend at 2.0 m/s (fast)
 *                         Negative values would command climb (unusual during landing).
 * 
 * @return true if descent rate override was accepted (in landing descent or QLAND)
 * @return false if not in applicable flight phase or quadplane not available
 * 
 * @note Only compiled if HAL_QUADPLANE_ENABLED and AP_SCRIPTING_ENABLED
 * @note Override has a timeout - must be refreshed periodically (typically at least 1Hz)
 * @note Normal landing descent rate is Q_LAND_SPEED parameter
 * 
 * @warning Very fast descent rates may result in hard landings
 * 
 * @see QuadPlane::poscontrol.override_descent_rate for storage
 * @see QuadPlane::poscontrol.last_override_descent_ms for timeout tracking
 * @see QuadPlane::in_vtol_land_descent() for landing phase detection
 * 
 * Source: ArduPlane/Plane.cpp:972-983
 */
bool Plane::set_land_descent_rate(float descent_rate)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_land_descent() ||
        control_mode == &mode_qland) {
        quadplane.poscontrol.override_descent_rate = descent_rate;
        quadplane.poscontrol.last_override_descent_ms = AP_HAL::millis();
        return true;
    }
#endif
    return false;
}

/**
 * @brief Set crosstrack starting point for navigation from scripting
 * 
 * @details Allows Lua scripts to control the crosstrack reference point (prev_WP_loc)
 *          when resuming missions or guided flight. This is useful for:
 *          - Resuming missions from a different location than where they were paused
 *          - Smoothly transitioning from scripted navigation back to waypoint navigation
 *          - Implementing custom crosstrack behavior
 *          - Preventing large crosstrack corrections when changing navigation context
 * 
 *          Crosstrack navigation uses a line from prev_WP_loc to next_WP_loc as the
 *          desired flight path. By setting prev_WP_loc appropriately, scripts can control
 *          how the L1 controller will navigate toward the next waypoint:
 *          - If prev_WP_loc is far from current position: vehicle may make large corrections
 *          - If prev_WP_loc is at/near current position: vehicle follows direct path
 *          - If prev_WP_loc is on the desired track: vehicle maintains smooth trajectory
 * 
 *          This method also enables crosstrack navigation (auto_state.crosstrack = true),
 *          ensuring the L1 controller will track the prev_WP→next_WP line rather than
 *          just heading directly to next_WP.
 * 
 *          Common use cases:
 *          - Script pauses mission, moves vehicle, then resumes: set prev_WP to current
 *            location to avoid large crosstrack correction
 *          - Script provides custom navigation, then returns to AUTO: set prev_WP to create
 *            smooth transition onto the mission path
 * 
 * @param[in] new_start_location Location to use as the crosstrack starting point (prev_WP_loc).
 *                               This becomes the "previous waypoint" for L1 navigation.
 *                               It's the script's responsibility to ensure this location
 *                               makes sense for the current navigation context.
 * 
 * @return true always (no validation performed)
 * 
 * @note Only compiled if AP_SCRIPTING_ENABLED
 * @note No validation is performed - script must ensure location is sensible
 * @note Enables crosstrack navigation automatically
 * @note Does not modify next_WP_loc (use update_target_location for that)
 * 
 * @warning Inappropriate prev_WP locations can cause unexpected navigation behavior
 * 
 * @see Plane::prev_WP_loc for previous waypoint storage
 * @see Plane::auto_state.crosstrack for crosstrack enable flag
 * @see AP_L1_Control for L1 controller that uses this for path following
 * 
 * Source: ArduPlane/Plane.cpp:988-992
 */
bool Plane::set_crosstrack_start(const Location &new_start_location)
{        
    prev_WP_loc = new_start_location;
    auto_state.crosstrack = true;
    return true;
}

#endif // AP_SCRIPTING_ENABLED

/**
 * @brief Check if vehicle is currently in landing phase
 * 
 * @details Returns true if the vehicle is actively landing, considering both fixed-wing
 *          and VTOL (quadplane) landing modes. This is used throughout the code for:
 *          - Disabling certain failsafe actions during landing
 *          - Adjusting control behavior for landing phase
 *          - Enabling landing-specific features (gear deployment, flare logic, etc.)
 *          - Logging and telemetry
 * 
 *          Landing detection logic:
 *          - For quadplane: Returns true if in VTOL landing descent
 *          - For fixed-wing: Delegates to control_mode->is_landing()
 *            which checks for LAND flight stage, QLAND mode, or landing sequences in AUTO
 * 
 *          Landing begins when:
 *          - AUTO mode executing NAV_LAND command (fixed-wing)
 *          - QLAND mode active (quadplane)
 *          - Quadplane in final descent phase of VTOL landing
 *          - AUTOLAND mode (if enabled)
 * 
 * @return true if vehicle is in landing phase (fixed-wing flare, VTOL descent, etc.)
 * @return false if in normal flight, takeoff, or other non-landing phases
 * 
 * @note For quadplane, VTOL descent is checked first (higher priority)
 * @note Landing detection affects behavior of failsafes and safety systems
 * @note May return true in abort-landing scenarios until abort completes
 * 
 * @see QuadPlane::in_vtol_land_descent() for VTOL landing detection
 * @see Mode::is_landing() for mode-specific landing detection
 * @see Plane::flight_stage for flight phase tracking
 * 
 * Source: ArduPlane/Plane.cpp:997-1005
 */
bool Plane::is_landing() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_land_descent()) {
        return true;
    }
#endif
    return control_mode->is_landing();
}

/**
 * @brief Check if vehicle is currently in takeoff phase
 * 
 * @details Returns true if the vehicle is actively taking off, considering both fixed-wing
 *          and VTOL (quadplane) takeoff modes. This is used throughout the code for:
 *          - Enabling takeoff-specific control laws (pitch limits, throttle management)
 *          - Disabling certain failsafe actions during takeoff
 *          - Adjusting navigation behavior during initial climb
 *          - Ground detection and automatic takeoff sequences
 * 
 *          Takeoff detection logic:
 *          - For quadplane: Returns true if in VTOL takeoff phase
 *          - For fixed-wing: Delegates to control_mode->is_taking_off()
 *            which checks for TAKEOFF flight stage or takeoff sequences in AUTO
 * 
 *          Takeoff begins when:
 *          - TAKEOFF mode is active (fixed-wing or VTOL)
 *          - AUTO mode executing NAV_TAKEOFF command
 *          - VTOL_TAKEOFF command in quadplane
 *          - Flight stage set to TAKEOFF (auto_state.takeoff_complete == false)
 * 
 *          Takeoff ends when:
 *          - Vehicle reaches target altitude (takeoff_altitude_rel_cm)
 *          - Takeoff heading achieved (for some takeoff types)
 *          - auto_state.takeoff_complete set to true
 * 
 * @return true if vehicle is in takeoff phase (ground roll, initial climb, etc.)
 * @return false if takeoff complete or not in takeoff mode
 * 
 * @note For quadplane, VTOL takeoff is checked first (higher priority)
 * @note Takeoff detection affects pitch/throttle limits and navigation behavior
 * @note Landing gear retraction typically happens at end of takeoff
 * 
 * @see QuadPlane::in_vtol_takeoff() for VTOL takeoff detection
 * @see Mode::is_taking_off() for mode-specific takeoff detection
 * @see Plane::auto_state.takeoff_complete for takeoff completion flag
 * @see Plane::flight_stage for flight phase tracking
 * 
 * Source: ArduPlane/Plane.cpp:1008-1016
 */
bool Plane::is_taking_off() const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_takeoff()) {
        return true;
    }
#endif
    return control_mode->is_taking_off();
}

#if HAL_QUADPLANE_ENABLED
/**
 * @brief Initiate quadplane VTOL takeoff from scripting
 * 
 * @details Allows Lua scripts to command a quadplane to perform a VTOL takeoff to a
 *          specified altitude. This is useful for:
 *          - Scripted mission sequences that need programmatic takeoff
 *          - Custom takeoff procedures based on sensor data
 *          - Automated testing and simulation
 *          - Integration with external flight management systems
 * 
 *          Requirements:
 *          - Vehicle must have quadplane capability (HAL_QUADPLANE_ENABLED)
 *          - Quadplane must be available (configured and initialized)
 * 
 *          The takeoff procedure:
 *          1. Arms motors (if not already armed, depending on configuration)
 *          2. Spools up motors to hover throttle
 *          3. Climbs vertically to specified altitude
 *          4. Hovers at target altitude until next command
 * 
 *          After reaching altitude, the vehicle can:
 *          - Transition to forward flight (if commanded)
 *          - Remain in hover (QHOVER/QLOITER)
 *          - Switch to another mode
 * 
 * @param[in] alt Target takeoff altitude in meters above home.
 *                Example: 20.0 means climb to 20 meters above home
 *                Must be positive value for upward takeoff.
 * 
 * @return true if takeoff was initiated successfully
 * @return false if quadplane not available or takeoff cannot be started
 * 
 * @note Only compiled if HAL_QUADPLANE_ENABLED and AP_SCRIPTING_ENABLED
 * @note Vehicle must be on ground and ready to takeoff
 * @note Arming checks still apply - vehicle must pass pre-arm checks
 * 
 * @see QuadPlane::do_user_takeoff() for takeoff implementation
 * @see QuadPlane::available() for quadplane availability check
 * 
 * Source: ArduPlane/Plane.cpp:1019-1021
 */
bool Plane::start_takeoff(const float alt) {
    return plane.quadplane.available() && quadplane.do_user_takeoff(alt);
}
#endif

/**
 * @brief Get roll and pitch angles for OSD display with appropriate corrections
 * 
 * @details Provides roll and pitch angles for On-Screen Display (OSD) that are
 *          corrected based on flight mode and configuration. This ensures the OSD
 *          displays intuitive attitude information that matches pilot expectations:
 * 
 *          VTOL mode (quadplane):
 *          - Returns quadplane's VTOL-specific AHRS view
 *          - Uses multicopter reference frame (level = hover attitude)
 *          - No pitch trim correction (VTOL should show actual attitude)
 * 
 *          Fixed-wing mode:
 *          - Returns standard AHRS roll and pitch
 *          - Applies pitch trim correction (unless disabled by flight option)
 *          - Corrects for PTCH_TRIM_DEG so OSD shows "0" when flying level
 * 
 *          The pitch trim correction accounts for the aircraft's natural trim angle
 *          in cruise. Without correction, the OSD would show a constant pitch offset
 *          even when the aircraft is flying straight and level.
 * 
 *          Flight option OSD_REMOVE_TRIM_PITCH allows disabling trim correction if
 *          pilots prefer to see absolute pitch angle instead of trim-corrected angle.
 * 
 * @param[out] roll Current roll angle in radians.
 *                  Range: [-π, π], positive = right wing down
 * @param[out] pitch Current pitch angle in radians, with trim correction applied in FW mode.
 *                   Range: [-π/2, π/2], positive = nose up
 *                   In fixed-wing: pitch = AHRS_pitch - PTCH_TRIM_DEG (unless option disabled)
 *                   In VTOL: pitch = quadplane AHRS view pitch (no correction)
 * 
 * @note Used by AP_OSD for artificial horizon and attitude display
 * @note Quadplane VTOL view takes priority when show_vtol_view() returns true
 * @note Pitch trim correction improves pilot situational awareness in fixed-wing flight
 * @note Angles are in radians for consistency with AHRS interface
 * 
 * @see FlightOptions::OSD_REMOVE_TRIM_PITCH for disabling pitch correction
 * @see QuadPlane::show_vtol_view() for VTOL display mode detection
 * @see g.pitch_trim for configured pitch trim angle
 * @see AP_OSD for OSD rendering system
 * 
 * Source: ArduPlane/Plane.cpp:1024-1039
 */
void Plane::get_osd_roll_pitch_rad(float &roll, float &pitch) const
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.show_vtol_view()) {
        pitch = quadplane.ahrs_view->pitch;
        roll = quadplane.ahrs_view->roll;
        return;
    }
#endif
    pitch = ahrs.get_pitch_rad();
    roll = ahrs.get_roll_rad();
    if (!(flight_option_enabled(FlightOptions::OSD_REMOVE_TRIM_PITCH))) {  // correct for PTCH_TRIM_DEG
        pitch -= g.pitch_trim * DEG_TO_RAD;
    }
}

/**
 * @brief Update current vehicle location from AHRS
 * 
 * @details Retrieves the latest position estimate from AHRS/EKF and updates:
 *          - current_loc: Full Location structure with lat/lon/alt
 *          - have_position: Flag indicating whether position estimate is available
 *          - relative_altitude: Altitude relative to home position in meters (NED frame)
 * 
 *          The position comes from EKF sensor fusion of GPS, barometer, airspeed,
 *          and inertial sensors. This is the vehicle's best estimate of current position.
 * 
 *          Relative altitude is calculated as the negative of the down component (D)
 *          of the position relative to home in NED (North-East-Down) coordinates.
 *          Negated because positive altitude is up, but NED down is positive down.
 * 
 * @note Called from update_GPS_50Hz(), executed at 50Hz
 * @note have_position will be false if AHRS doesn't have a valid position estimate
 *       (e.g., no GPS fix, EKF not initialized)
 * 
 * @see AP_AHRS::get_location() for position estimation
 * @see AP_AHRS::get_relative_position_D_home() for altitude calculation
 * 
 * Source: ArduPlane/Plane.cpp:1044-1051
 */
void Plane::update_current_loc(void)
{
    have_position = plane.ahrs.get_location(plane.current_loc);

    // re-calculate relative altitude
    ahrs.get_relative_position_D_home(plane.relative_altitude);
    relative_altitude *= -1.0f;
}

/**
 * @brief Check if a specific flight option is enabled
 * 
 * @details Tests whether a particular flight option flag is set in the g2.flight_options
 *          bitmask parameter. Flight options are used to enable optional or experimental
 *          features without adding individual parameters for each feature.
 * 
 *          Flight options allow users to customize vehicle behavior including:
 *          - OSD_REMOVE_TRIM_PITCH: Disable pitch trim correction on OSD
 *          - ENABLE_DEFAULT_AIRSPEED: Use default airspeed if sensor fails
 *          - CLIMB_BEFORE_TURN: Climb to RTL altitude before turning
 *          - CENTER_THROTTLE_TRIM: Use mid-stick as throttle trim
 *          - DISABLE_GROUND_PID_SUPPRESSION: Keep PID controllers active on ground
 *          - And many other optional behaviors
 * 
 *          This is a lightweight bit-test operation that checks if the specified
 *          option bit is set in the flight_options parameter.
 * 
 * @param[in] flight_option The FlightOptions enum value to check.
 *                          Each option corresponds to a bit in the bitmask.
 * 
 * @return true if the specified flight option is enabled (bit is set)
 * @return false if the flight option is disabled (bit is clear)
 * 
 * @note Used extensively throughout the codebase to conditionally enable features
 * @note FlightOptions is defined in Plane.h as an enum class
 * @note Multiple options can be enabled simultaneously (bitmask)
 * @note Options are set via the FLIGHT_OPTIONS parameter
 * 
 * @see Plane::g2.flight_options parameter for option bitmask
 * @see FlightOptions enum in Plane.h for available options
 * 
 * Source: ArduPlane/Plane.cpp:2199-2203
 */
bool Plane::flight_option_enabled(FlightOptions flight_option) const
{
    return g2.flight_options & flight_option;
}

#if AC_PRECLAND_ENABLED
/**
 * @brief Update precision landing system with current altitude information
 * 
 * @details Updates the precision landing (PrecLand) subsystem with current height
 *          above ground data. The PrecLand system uses this to calculate landing
 *          trajectories and align with visual landing targets (IR beacons, AprilTags).
 * 
 *          Height source priority:
 *          1. Rangefinder: If available and in range, provides accurate ground height
 *             - Height estimate in centimeters (converted from meters * 100)
 *             - Range validity flag indicates if reading is trustworthy
 *          2. Barometer/AHRS: If rangefinder unavailable, falls back to relative altitude
 *             - Less accurate near ground due to barometric lag
 *             - Passed as height=0 with in_range=false to indicate unavailable
 * 
 *          The PrecLand system uses height information for:
 *          - Calculating descent rate to target
 *          - Determining when to switch precision landing stages
 *          - Estimating time to ground contact
 *          - Compensating for sensor field-of-view changes with altitude
 * 
 * @note Only compiled if AC_PRECLAND_ENABLED
 * @note Called at 400Hz from scheduler task precland_update
 * @note Rangefinder height is preferred when available for accuracy
 * @note Height passed in centimeters (cm) to match AP_PrecLand interface
 * 
 * @see AP_PrecLand::update() for precision landing state machine
 * @see Plane::rangefinder_state.height_estimate for rangefinder height
 * @see Plane::rangefinder_state.in_range for rangefinder validity
 * 
 * Source: ArduPlane/Plane.cpp:2205-2214
 */
void Plane::precland_update(void)
{
    // alt will be unused if we pass false through as the second parameter:
#if AP_RANGEFINDER_ENABLED
    return g2.precland.update(rangefinder_state.height_estimate*100, rangefinder_state.in_range);
#else
    return g2.precland.update(0, false);
#endif
}
#endif

#if AP_QUICKTUNE_ENABLED
/**
 * @brief Update the quick-tune system for automated PID tuning
 * 
 * @details Updates the AP_Quicktune object which provides automated in-flight PID
 *          tuning capabilities. Quick-tune performs real-time system identification
 *          and optimization of control loop gains without requiring manual tuning.
 * 
 *          Quick-tune process:
 *          1. Pilot initiates tuning via RC switch or GCS command
 *          2. System applies test inputs (doublet or frequency sweep)
 *          3. Measures vehicle response (attitude rates, oscillations)
 *          4. Calculates optimal PID gains based on response
 *          5. Applies new gains and tests for stability
 *          6. Reverts to previous gains if vehicle becomes unstable
 * 
 *          Safety features:
 *          - Mode compatibility check: Reverts gains if pilot switches to
 *            non-quicktune-capable mode (e.g., MANUAL, STABILIZE)
 *          - Automatic reversion if tuning produces unstable behavior
 *          - Pilot can abort tuning at any time
 * 
 *          The supports_quicktune() callback allows quicktune to detect mode changes
 *          during tuning. If the pilot switches from a supported mode (FBWA, CRUISE,
 *          AUTOTUNE) to an unsupported mode, gains are reverted to prevent issues.
 * 
 *          Quicktune-capable modes:
 *          - FBWA, FBWB: Rate and attitude loop tuning
 *          - CRUISE: Rate and attitude tuning with airspeed control
 *          - AUTOTUNE: Dedicated tuning mode
 *          - AUTO, GUIDED: Tuning during autonomous flight
 * 
 * @note Only compiled if AP_QUICKTUNE_ENABLED
 * @note Called at 40Hz from scheduler task
 * @note Gains are automatically saved to EEPROM after successful tune
 * @note Vehicle must be in stable flight (not landing/takeoff) for tuning
 * 
 * @warning Experimental feature - monitor vehicle closely during first use
 * @warning Ensure good GPS and sufficient altitude before initiating tune
 * 
 * @see AP_Quicktune::update() for tuning state machine
 * @see Mode::supports_quicktune() for mode compatibility check
 * @see QUIK_ENABLE parameter to enable quicktune
 * 
 * Source: ArduPlane/Plane.cpp:2216-2226
 */
void Plane::update_quicktune(void)
{
    quicktune.update(control_mode->supports_quicktune());
}
#endif

/**
 * @brief Constructor for the main Plane vehicle class
 * 
 * @details Initializes the Plane object which serves as the central controller for
 *          fixed-wing flight. Most member initialization occurs through C++ member
 *          initializer lists in Plane.h, but some members require explicit initialization
 *          in the constructor body.
 * 
 *          Initialization performed here:
 *          - auto_state.takeoff_complete = true: Initializes takeoff state to complete.
 *            This prevents inadvertent takeoff mode behavior on startup. Will be set
 *            to false when a takeoff is actually commanded.
 * 
 *          Other subsystems are initialized through:
 *          - Member initializer lists in Plane.h (most objects)
 *          - Plane::init_ardupilot() during startup sequence
 *          - Individual subsystem init() methods called from setup()
 * 
 * @note This constructor is called once at startup before setup() runs
 * @note C++11 doesn't allow in-class initialization of bitfields, requiring
 *       initialization here rather than in the class definition
 * 
 * @see Plane.h for class definition and member declarations
 * @see Plane::init_ardupilot() for main initialization sequence
 * @see AP_Vehicle for base class initialization
 * 
 * Source: ArduPlane/Plane.cpp:1086-1090
 */
Plane::Plane(void)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

/**
 * @brief Global Plane vehicle instance
 * 
 * @details This is the single global instance of the Plane class that controls the vehicle.
 *          Created at program startup before main() runs. All ArduPlane code accesses
 *          vehicle state and methods through this global 'plane' object.
 * 
 *          The singleton pattern is used throughout ArduPilot for vehicle objects to:
 *          - Provide global access without requiring pointer passing
 *          - Ensure only one vehicle controller exists
 *          - Enable static callback functions to access vehicle state
 */
Plane plane;

/**
 * @brief Reference to vehicle instance via base class pointer
 * 
 * @details Provides access to the Plane instance through the AP_Vehicle base class
 *          interface. This allows common vehicle code to work with any vehicle type
 *          (Plane, Copter, Rover, Sub) through polymorphism.
 */
AP_Vehicle& vehicle = plane;

/**
 * @brief HAL main entry point registration
 * 
 * @details Registers the Plane object with the Hardware Abstraction Layer (HAL) to
 *          receive setup() and loop() callbacks. This macro:
 *          - Registers plane.setup() to be called once at startup
 *          - Registers plane.loop() to be called repeatedly at scheduler rate
 *          - Handles platform-specific main() function generation
 * 
 * @see AP_HAL for hardware abstraction layer details
 * @see Plane::setup() for initialization sequence
 * @see Plane::loop() for main control loop
 * 
 * Source: ArduPlane/Plane.cpp:1092-1095
 */
AP_HAL_MAIN_CALLBACKS(&plane);
