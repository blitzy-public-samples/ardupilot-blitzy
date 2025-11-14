/**
 * @file Blimp.cpp
 * @brief Main implementation file for Blimp lighter-than-air vehicle
 * 
 * @details This file contains the core implementation for the ArduPilot Blimp vehicle,
 *          including:
 *          - Global blimp instance definition (singleton pattern)
 *          - Scheduler task table (scheduler_tasks[]) defining all periodic loops
 *          - FAST_TASK entries running at main loop rate (typically 400Hz)
 *          - SCHED_TASK entries running at specific rates (100Hz, 50Hz, 10Hz, etc.)
 *          - Glue routines for periodic operations:
 *            * rc_loop: RC input processing at 100Hz
 *            * throttle_loop: Throttle and motor output at 50Hz
 *            * read_AHRS: Attitude/heading updates at fast loop rate
 *            * update_altitude: Altitude estimation at 10Hz
 *            * Logging hooks at various rates (full_rate, ten_hz, twentyfive_hz)
 *          - Coordinate frame transformation utilities (body frame <-> NED)
 *          - Main vehicle constructor and global instance initialization
 * 
 *          The Blimp vehicle is designed for lighter-than-air flight control with
 *          unique characteristics including buoyancy management, fin control for
 *          directional stability, and specialized flight modes for airship operations.
 * 
 * @note This file defines the global 'blimp' instance accessed throughout the codebase
 * @warning Timing constraints must be maintained for all scheduled tasks to ensure
 *          stable flight control
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
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

#include "Blimp.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros, priority) SCHED_TASK_CLASS(Blimp, &blimp, func, rate_hz, max_time_micros, priority)
#define FAST_TASK(func) FAST_TASK_CLASS(Blimp, &blimp, func)

/**
 * @brief Scheduler task table defining all periodic operations for the Blimp vehicle
 * 
 * @details This table defines the complete set of tasks that run periodically to control
 *          the Blimp lighter-than-air vehicle. Tasks are ordered by priority and interleaved
 *          with the base AP_Vehicle task table to determine execution order.
 * 
 *          Task Categories:
 *          
 *          FAST_TASK entries - Run at main loop rate (typically 400Hz):
 *          - AP_InertialSensor::update: Update IMU data immediately for current gyro readings
 *          - motors_output: Send motor commands to actuators with minimal latency
 *          - read_AHRS: Run EKF state estimator (computationally expensive)
 *          - read_inertia: Update inertial navigation state
 *          - check_ekf_reset: Detect and handle EKF resets of heading or position
 *          - update_flight_mode: Execute attitude controllers for current flight mode
 *          - update_home_from_EKF: Sync home position with EKF origin if necessary
 *          
 *          SCHED_TASK entries - Run at specified rates:
 *          - rc_loop (100Hz): Process RC input from transmitter/receiver
 *          - throttle_loop (50Hz): Update throttle and check auto-armed status
 *          - AP_GPS::update (50Hz): Process GPS data and update position
 *          - update_batt_compass (10Hz): Read battery voltage/current and compass
 *          - RC_Channels::read_aux_all (10Hz): Read auxiliary RC channels
 *          - update_altitude (10Hz): Read barometer and update altitude estimate
 *          - three_hz_loop (3Hz): GCS failsafe checks
 *          - AP_ServoRelayEvents::update_events (50Hz): Process servo/relay events
 *          - full_rate_logging (50Hz): High-rate attitude and PID logging
 *          - AP_Notify::update (50Hz): Update LED/buzzer notifications
 *          - one_hz_loop (1Hz): Low-rate tasks (auxiliary servos, state logging)
 *          - ekf_check (10Hz): Monitor EKF health and variance
 *          - check_vibration (10Hz): Monitor IMU vibration levels
 *          - gpsglitch_check (10Hz): Detect GPS position glitches
 *          - GCS::update_receive (400Hz): Receive MAVLink messages from ground station
 *          - GCS::update_send (400Hz): Send MAVLink telemetry to ground station
 *          - ten_hz_logging_loop (10Hz): Medium-rate logging (attitude, EKF, RC, IMU)
 *          - twentyfive_hz_logging (25Hz): 25Hz logging (EKF position, IMU data)
 *          - AP_Logger::periodic_tasks (400Hz): Logger housekeeping
 *          - AP_InertialSensor::periodic (400Hz): IMU periodic maintenance
 *          - AP_Scheduler::update_logging (0.1Hz): Scheduler performance logging
 * 
 *          Task Timing Requirements:
 *          - Fast tasks must complete within main loop period (~2.5ms at 400Hz)
 *          - Scheduled tasks have expected execution times in microseconds
 *          - Priority determines execution order (lower number = higher priority)
 *          - Tasks exceeding expected time generate scheduler overrun warnings
 * 
 * @note All entries must be ordered by priority (0-255, lower = higher priority)
 * @note This table is interleaved with AP_Vehicle::scheduler_tasks[] during execution
 * @note SCHED_TASK and FAST_TASK macros automatically fill in class/instance parameters
 * 
 * @warning Modifying task rates or adding expensive operations can cause timing violations
 * @warning Scheduler overruns can lead to degraded flight performance or instability
 * 
 * @see AP_Scheduler for task scheduling implementation
 * @see Blimp.h for FAST_TASK and SCHED_TASK macro definitions
 * 
 * SCHED_TASK macro arguments:
 * - name of static function to call
 * - rate (in Hertz) at which the function should be called
 * - expected time (in MicroSeconds) that the function should take to run
 * - priority (0 through 255, lower number meaning higher priority)
 *
 * SCHED_TASK_CLASS macro arguments:
 * - class name of method to be called
 * - instance on which to call the method
 * - method to call on that instance
 * - rate (in Hertz) at which the method should be called
 * - expected time (in MicroSeconds) that the method should take to run
 * - priority (0 through 255, lower number meaning higher priority)
 */
const AP_Scheduler::Task Blimp::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &blimp.ins, update),
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
    // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),

    SCHED_TASK(rc_loop,              100,    130,   3),
    SCHED_TASK(throttle_loop,         50,     75,   6),
    SCHED_TASK_CLASS(AP_GPS, &blimp.gps, update, 50, 200,   9),
    SCHED_TASK(update_batt_compass,   10,    120,  12),
    SCHED_TASK_CLASS(RC_Channels,          (RC_Channels*)&blimp.g2.rc_channels,      read_aux_all,    10,     50,  15),
    SCHED_TASK(update_altitude,       10,    100,  21),
    SCHED_TASK(three_hz_loop,          3,     75,  24),
#if AP_SERVORELAYEVENTS_ENABLED
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &blimp.ServoRelayEvents,      update_events, 50,     75,  27),
#endif
#if HAL_LOGGING_ENABLED
    SCHED_TASK(full_rate_logging,     50,    50,  33),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &blimp.notify,              update,          50,  90,  36),
    SCHED_TASK(one_hz_loop,            1,    100,  39),
    SCHED_TASK(ekf_check,             10,     75,  42),
    SCHED_TASK(check_vibration,       10,     50,  45),
    SCHED_TASK(gpsglitch_check,       10,     50,  48),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_receive, 400, 180,  51),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&blimp._gcs,          update_send,    400, 550,  54),
#if HAL_LOGGING_ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350,  57),
    SCHED_TASK(twentyfive_hz_logging, 25,    110,  60),
    SCHED_TASK_CLASS(AP_Logger,      &blimp.logger,           periodic_tasks, 400, 300,  63),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &blimp.ins,                 periodic,       400,  50,  66),
#if HAL_LOGGING_ENABLED
    SCHED_TASK_CLASS(AP_Scheduler,         &blimp.scheduler,           update_logging, 0.1,  75,  69),
#endif
};

/**
 * @brief Get the vehicle-specific scheduler tasks for the Blimp
 * 
 * @details This method provides the scheduler with access to the Blimp-specific
 *          task table. The AP_Scheduler combines this table with the base
 *          AP_Vehicle tasks and sorts them by priority to create the final
 *          execution schedule.
 * 
 * @param[out] tasks      Pointer to the Blimp scheduler_tasks array
 * @param[out] task_count Number of tasks in the scheduler_tasks array
 * @param[out] log_bit    Logging bit mask for performance monitoring (MASK_LOG_PM)
 * 
 * @note Called by AP_Scheduler during initialization
 * @note The returned task array must remain valid for the lifetime of the scheduler
 * 
 * @see scheduler_tasks[] for the complete task table definition
 */
void Blimp::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                uint8_t &task_count,
                                uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Blimp::_failsafe_priorities[4];

/**
 * @brief Process RC input from transmitter/receiver
 * 
 * @details This function reads pilot input from the radio receiver and processes
 *          flight mode switch changes. It is called at 100Hz to ensure responsive
 *          control and timely mode switching.
 *          
 *          Operations performed:
 *          - Read all RC channel values via read_radio()
 *          - Process 3-position (or more) flight mode switch
 *          - Update RC input data structures for use by flight mode controllers
 * 
 * @note Scheduled task - runs at 100Hz (every 10ms)
 * @note Expected execution time: 130 microseconds
 * @note Priority: 3 (relatively high priority for responsive pilot input)
 * 
 * @see read_radio() for RC channel reading implementation
 * @see RC_Channel::read_mode_switch() for mode switch processing
 */
void Blimp::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

/**
 * @brief Throttle and motor management loop
 * 
 * @details This function handles throttle-related updates and checks the auto-armed
 *          status of the vehicle. For the Blimp, this monitors whether the vehicle
 *          should be automatically armed based on throttle position and other conditions.
 *          
 *          The 50Hz rate provides adequate responsiveness for motor state management
 *          while not overloading the scheduler with high-frequency checks.
 * 
 * @note Scheduled task - runs at 50Hz (every 20ms)
 * @note Expected execution time: 75 microseconds
 * @note Priority: 6
 * 
 * @see update_auto_armed() for auto-arming logic
 */
void Blimp::throttle_loop()
{
    // check auto_armed status
    update_auto_armed();
}

/**
 * @brief Read battery and compass sensors
 * 
 * @details This function updates battery voltage/current readings and compass data.
 *          The battery is read first because its voltage may be used for motor
 *          interference compensation in the compass (compassmot calibration).
 *          
 *          The compass voltage is updated before reading to enable real-time
 *          compensation for electrical interference from motors and ESCs that
 *          varies with current draw.
 *          
 *          Running at 10Hz provides adequate update rate for these relatively
 *          slow-changing sensors while minimizing I2C bus traffic.
 * 
 * @note Scheduled task - runs at 10Hz (every 100ms)
 * @note Expected execution time: 120 microseconds
 * @note Priority: 12
 * @note Battery must be read before compass for motor interference compensation
 * 
 * @see AP_BattMonitor::read() for battery monitoring
 * @see Compass::read() for magnetometer reading
 * @see Compass::set_voltage() for motor interference compensation
 */
void Blimp::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

#if HAL_LOGGING_ENABLED
/**
 * @brief High-rate logging of attitude and PID controller data
 * 
 * @details This function logs critical flight control data at a high rate (50Hz)
 *          for detailed analysis and tuning. It conditionally logs:
 *          - Attitude data (roll, pitch, yaw) if MASK_LOG_ATTITUDE_FAST enabled
 *          - PID controller internal states if MASK_LOG_PID enabled
 *          
 *          High-rate logging is essential for:
 *          - PID tuning and optimization
 *          - Control loop analysis
 *          - Oscillation diagnosis
 *          - Performance debugging
 * 
 * @note Scheduled task - runs at 50Hz (every 20ms)
 * @note Expected execution time: 50 microseconds
 * @note Priority: 33
 * @note Only active when logging is enabled via parameters
 * 
 * @see Log_Write_Attitude() for attitude logging
 * @see Log_Write_PIDs() for PID state logging
 * @see should_log() for logging mask checking
 */
void Blimp::full_rate_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
    if (should_log(MASK_LOG_PID)) {
        Log_Write_PIDs();
    }
}

/**
 * @brief Medium-rate logging of system state and sensor data
 * 
 * @details This function logs various system parameters at 10Hz, providing a balance
 *          between data resolution and storage/bandwidth requirements. Logged data includes:
 *          
 *          - Attitude data (if medium-rate enabled and not already at high-rate)
 *          - EKF position and velocity estimates
 *          - RC input channels (RCIN) for pilot command analysis
 *          - RSSI (signal strength) if available
 *          - RC output channels (RCOUT) for actuator verification
 *          - IMU vibration levels for mechanical health monitoring
 *          
 *          This logging rate is appropriate for:
 *          - General flight analysis and troubleshooting
 *          - Long-duration flight recording
 *          - RC input/output correlation
 *          - Sensor health monitoring
 * 
 * @note Scheduled task - runs at 10Hz (every 100ms)
 * @note Expected execution time: 350 microseconds
 * @note Priority: 57
 * @note Logging is conditional based on LOG_BITMASK parameter settings
 * 
 * @see Log_Write_Attitude() for attitude logging
 * @see Log_Write_EKF_POS() for EKF state logging
 * @see AP_Logger::Write_RCIN() for RC input logging
 * @see AP_Logger::Write_RCOUT() for RC output logging
 * @see AP_InertialSensor::Write_Vibration() for vibration logging
 */
void Blimp::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
    // log EKF attitude data
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
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
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
}


/**
 * @brief High-medium rate logging for EKF and IMU data
 * 
 * @details This function provides 25Hz logging for data that requires higher resolution
 *          than the 10Hz loop but doesn't need the full 50Hz rate. This intermediate
 *          rate is optimal for:
 *          
 *          - EKF position/velocity state (when fast attitude logging enabled)
 *          - IMU raw sensor data (accelerometer and gyroscope readings)
 *          
 *          The 25Hz rate captures dynamics adequately for most analysis while
 *          reducing storage requirements compared to 50Hz logging.
 * 
 * @note Scheduled task - runs at 25Hz (every 40ms)
 * @note Expected execution time: 110 microseconds
 * @note Priority: 60
 * @note Conditional on LOG_BITMASK parameter settings
 * 
 * @see Log_Write_EKF_POS() for EKF position logging
 * @see AP_InertialSensor::Write_IMU() for IMU data logging
 */
void Blimp::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }
}
#endif  // HAL_LOGGING_ENABLED

/**
 * @brief Low-rate loop for ground station failsafe checks
 * 
 * @details This function runs at approximately 3Hz to perform less time-critical
 *          monitoring tasks. Currently implements:
 *          
 *          - Ground Control Station (GCS) communication health monitoring
 *          - Detection of lost telemetry link
 *          - Triggering of GCS failsafe actions if communication is lost
 *          
 *          The 3Hz rate is sufficient for failsafe detection while minimizing
 *          scheduler overhead, as GCS timeouts are typically on the order of seconds.
 * 
 * @note Scheduled task - runs at 3Hz (approximately every 333ms)
 * @note Expected execution time: 75 microseconds
 * @note Priority: 24
 * 
 * @see failsafe_gcs_check() for GCS failsafe logic
 */
void Blimp::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();
}

/**
 * @brief Very low-rate loop for infrequent housekeeping tasks
 * 
 * @details This function runs at 1Hz (once per second) to handle tasks that don't
 *          require frequent updates. Operations performed:
 *          
 *          - Log vehicle state flags (arming status, flight mode, etc.)
 *          - Enable and update auxiliary servo assignments
 *          - Update notification system flying status for LED/buzzer indicators
 *          - Update yaw PID notch filter sample rate based on actual loop performance
 *          
 *          The 1Hz rate minimizes CPU usage for these low-priority maintenance tasks
 *          while ensuring they execute regularly enough for proper system operation.
 * 
 * @note Scheduled task - runs at 1Hz (every 1000ms)
 * @note Expected execution time: 100 microseconds
 * @note Priority: 39
 * 
 * @see Log_Write_Data() for state logging
 * @see SRV_Channels::enable_aux_servos() for servo management
 * @see AP_Notify for LED/buzzer notification system
 */
void Blimp::one_hz_loop()
{
#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }
#endif

    // update assigned functions and enable auxiliary servos
    AP::srv().enable_aux_servos();

    AP_Notify::flags.flying = !ap.land_complete;

    blimp.pid_pos_yaw.set_notch_sample_rate(AP::scheduler().get_filtered_loop_rate_hz());
}

/**
 * @brief Update AHRS and retrieve vehicle state estimates
 * 
 * @details This critical fast-loop task runs the Extended Kalman Filter (EKF) to update
 *          the vehicle's attitude, position, and velocity estimates. It performs:
 *          
 *          1. AHRS update (EKF prediction and measurement updates) - computationally expensive
 *          2. Retrieve velocity estimate in NED (North-East-Down) frame from EKF
 *          3. Retrieve position estimate relative to EKF origin in NED frame
 *          4. Get yaw rate in earth frame
 *          5. Apply low-pass filtering to velocity components (X, Y, Z, Yaw rate)
 *          6. Log raw and filtered velocity/position data for analysis
 *          
 *          The INS (Inertial Navigation System) update is skipped in AHRS as it has
 *          already been performed earlier in the fast loop for minimum latency.
 *          
 *          Filtered velocities are used by position controllers to reduce noise and
 *          prevent control instability from high-frequency sensor variations.
 * 
 * @note FAST_TASK - runs at main loop rate (typically 400Hz)
 * @note This is one of the most computationally expensive tasks in the scheduler
 * @note Coordinate frame: NED (North-East-Down) for position and velocity
 * @note Units: position in meters, velocity in m/s, yaw rate in rad/s
 * 
 * @warning EKF computational load can cause timing issues if main loop rate is too high
 * @warning Filtered velocity state is critical for stable position control
 * 
 * @see AP_AHRS::update() for EKF state estimation
 * @see AP_AHRS::get_velocity_NED() for velocity retrieval
 * @see AP_AHRS::get_relative_position_NED_origin_float() for position retrieval
 */
void Blimp::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);

    IGNORE_RETURN(ahrs.get_velocity_NED(vel_ned));
    IGNORE_RETURN(ahrs.get_relative_position_NED_origin_float(pos_ned));

    vel_yaw = ahrs.get_yaw_rate_earth();
    Vector2f vel_xy_filtd = vel_xy_filter.apply({vel_ned.x, vel_ned.y});
    vel_ned_filtd = {vel_xy_filtd.x, vel_xy_filtd.y, vel_z_filter.apply(vel_ned.z)};
    vel_yaw_filtd = vel_yaw_filter.apply(vel_yaw);

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("VNF", "TimeUS,X,XF,Y,YF,Z,ZF,Yaw,YawF,PX,PY,PZ,PYaw", "Qffffffffffff",
                                AP_HAL::micros64(),
                                vel_ned.x,
                                vel_ned_filtd.x,
                                vel_ned.y,
                                vel_ned_filtd.y,
                                vel_ned.z,
                                vel_ned_filtd.z,
                                vel_yaw,
                                vel_yaw_filtd,
                                pos_ned.x,
                                pos_ned.y,
                                pos_ned.z,
                                blimp.ahrs.get_yaw_rad());
#endif
}

/**
 * @brief Read barometer and log control tuning data
 * 
 * @details This function updates altitude estimates from the barometric pressure sensor
 *          and logs control tuning information for analysis. Operations include:
 *          
 *          - Read barometer to update altitude estimate
 *          - Log harmonic notch filter data (if enabled) for vibration analysis
 *          - Log gyro FFT data (if enabled) for frequency domain analysis
 *          
 *          The 10Hz update rate is appropriate for barometric altitude which changes
 *          slowly compared to fast control loops. Control tuning logs help diagnose
 *          vibration issues and optimize notch filter performance.
 * 
 * @note Scheduled task - runs at 10Hz (every 100ms)
 * @note Expected execution time: 100 microseconds
 * @note Priority: 21
 * @note Logging conditional on MASK_LOG_CTUN parameter
 * 
 * @see read_barometer() for barometric pressure reading
 * @see AP_InertialSensor::write_notch_log_messages() for notch filter logging
 * @see AP_GyroFFT::write_log_messages() for FFT analysis logging
 */
void Blimp::update_altitude()
{
    // read in baro altitude
    read_barometer();

#if HAL_LOGGING_ENABLED
    if (should_log(MASK_LOG_CTUN)) {
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
        AP::ins().write_notch_log_messages();
#endif
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
#endif
}

/**
 * @brief Transform 2D vector from body frame to North-East frame
 * 
 * @details This function converts a 2D vector from body-fixed coordinates to
 *          North-East earth-fixed coordinates using the vehicle's current yaw angle.
 *          The transformation is performed in 2D (horizontal plane only) so that
 *          vertical (up) remains vertical in the world frame even when the blimp
 *          is not perfectly level.
 *          
 *          This is appropriate for lighter-than-air vehicles where pitch and roll
 *          attitudes are typically small and horizontal motion control can be
 *          decoupled from vertical control.
 *          
 *          Transformation matrix:
 *          [ NE_x ]   [ cos(yaw)  -sin(yaw) ] [ BF_x ]
 *          [ NE_y ] = [ sin(yaw)   cos(yaw) ] [ BF_y ]
 * 
 * @param[in,out] vec 2D vector to transform (modified in place)
 *                    Input: body frame coordinates (forward/right)
 *                    Output: North-East frame coordinates
 * 
 * @note Body frame: X = forward, Y = right
 * @note North-East frame: X = North, Y = East
 * @note Uses current vehicle yaw angle from AHRS
 * @note Units: preserves input units (typically m or m/s)
 * 
 * @see rotate_NE_to_BF() for inverse transformation
 * @see AP_AHRS::cos_yaw() and AP_AHRS::sin_yaw() for yaw angle
 */
void Blimp::rotate_BF_to_NE(Vector2f &vec)
{
    float ne_x = vec.x*ahrs.cos_yaw() - vec.y*ahrs.sin_yaw();
    float ne_y = vec.x*ahrs.sin_yaw() + vec.y*ahrs.cos_yaw();
    vec.x = ne_x;
    vec.y = ne_y;
}

/**
 * @brief Transform 2D vector from North-East frame to body frame
 * 
 * @details This function converts a 2D vector from North-East earth-fixed coordinates
 *          to body-fixed coordinates using the vehicle's current yaw angle. This is
 *          the inverse transformation of rotate_BF_to_NE().
 *          
 *          The 2D transformation maintains vertical as vertical in both frames,
 *          which is appropriate for lighter-than-air vehicles operating with small
 *          pitch and roll angles.
 *          
 *          Transformation matrix (inverse/transpose of BF_to_NE):
 *          [ BF_x ]   [  cos(yaw)  sin(yaw) ] [ NE_x ]
 *          [ BF_y ] = [ -sin(yaw)  cos(yaw) ] [ NE_y ]
 * 
 * @param[in,out] vec 2D vector to transform (modified in place)
 *                    Input: North-East frame coordinates
 *                    Output: body frame coordinates (forward/right)
 * 
 * @note North-East frame: X = North, Y = East
 * @note Body frame: X = forward, Y = right
 * @note Uses current vehicle yaw angle from AHRS
 * @note Units: preserves input units (typically m or m/s)
 * 
 * @see rotate_BF_to_NE() for forward transformation
 * @see AP_AHRS::cos_yaw() and AP_AHRS::sin_yaw() for yaw angle
 */
void Blimp::rotate_NE_to_BF(Vector2f &vec)
{
    float bf_x = vec.x*ahrs.cos_yaw() + vec.y*ahrs.sin_yaw();
    float bf_y = -vec.x*ahrs.sin_yaw() + vec.y*ahrs.cos_yaw();
    vec.x = bf_x;
    vec.y = bf_y;

}

/**
 * @brief Constructor for the main Blimp vehicle class
 * 
 * @details Initializes the Blimp vehicle object with default values and constructs
 *          member objects. The constructor performs initialization list setup for:
 *          
 *          - flight_modes: Flight mode configuration pointer
 *          - control_mode: Initial control mode (MANUAL)
 *          - rc_throttle_control_in_filter: Throttle input filter (1.0Hz cutoff)
 *          - inertial_nav: Inertial navigation system (requires AHRS reference)
 *          - param_loader: Parameter loading system (var_info table)
 *          - flightmode: Current flight mode pointer (initially mode_manual)
 *          
 *          Further initialization occurs in Blimp::setup() after hardware is initialized.
 * 
 * @note This constructor is called before main() during static initialization
 * @note Hardware interfaces (HAL) are not yet available during construction
 * @note Actual vehicle initialization occurs in setup() method
 * 
 * @see Blimp::setup() for full vehicle initialization sequence
 * @see Blimp.h for member variable declarations
 */
Blimp::Blimp(void)
    :
      flight_modes(&g.flight_mode1),
      control_mode(Mode::Number::MANUAL),
      rc_throttle_control_in_filter(1.0f),
      inertial_nav(ahrs),
      param_loader(var_info),
      flightmode(&mode_manual)
{
}

/**
 * @brief Global Blimp vehicle instance (singleton pattern)
 * 
 * @details This is the single global instance of the Blimp class that represents
 *          the vehicle throughout the codebase. The singleton pattern ensures there
 *          is exactly one vehicle object, which is referenced globally as 'blimp'.
 *          
 *          This instance is constructed during static initialization before main()
 *          is called, and persists for the lifetime of the program.
 * 
 * @note Accessed globally via the 'blimp' variable name
 * @note Constructed before main() during static initialization
 * @note Hardware initialization occurs later in setup() method
 * 
 * @see Blimp class definition in Blimp.h
 * @see Blimp::Blimp() constructor
 */
Blimp blimp;

/**
 * @brief Generic vehicle reference for polymorphic access
 * 
 * @details This reference provides access to the Blimp instance through the base
 *          AP_Vehicle interface, enabling vehicle-agnostic code to interact with
 *          the Blimp using common vehicle methods.
 * 
 * @note Used by libraries that operate on any vehicle type
 * @see AP_Vehicle base class for common vehicle interface
 */
AP_Vehicle& vehicle = blimp;

/**
 * @brief HAL main entry point registration
 * 
 * @details This macro registers the Blimp instance with the Hardware Abstraction Layer
 *          (HAL) as the main application callbacks object. The HAL will call methods
 *          on this object during the main program lifecycle:
 *          - setup() for initialization
 *          - loop() for main execution loop
 * 
 * @note This defines the program entry point after HAL initialization
 * @see AP_HAL_MAIN_CALLBACKS macro definition
 * @see Blimp::setup() and Blimp::loop() for main program flow
 */
AP_HAL_MAIN_CALLBACKS(&blimp);
