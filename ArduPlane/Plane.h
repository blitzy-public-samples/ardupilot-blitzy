/**
 * @file Plane.h
 * @brief Main header defining the Plane class - central coordinator for all ArduPlane fixed-wing and VTOL/QuadPlane operations
 * 
 * @details This file defines the Plane class which serves as the primary vehicle controller for ArduPlane.
 *          It coordinates all subsystems including flight control, navigation, sensors, actuators, communication,
 *          safety systems, and the QuadPlane hybrid VTOL functionality.
 * 
 * Lead developer: Andrew Tridgell & Tom Pittenger
 *
 * Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
 * Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon
 *
 * Please contribute your ideas! See http://dev.ardupilot.com for details
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
#pragma once

/**
 * @brief Required library includes for Plane vehicle functionality
 * 
 * @details Includes organized by category:
 * 
 * **Core Infrastructure:**
 * - AP_HAL: Hardware Abstraction Layer providing platform-independent interfaces
 * - AP_Common: Common utilities and definitions
 * - AP_Param: Parameter storage and management system
 * - StorageManager: Persistent storage abstraction
 * 
 * **Sensors:**
 * - AP_InertialSensor: IMU (gyro/accelerometer) interface and calibration
 * - AP_Airspeed: Airspeed sensor drivers and calibration
 * - AP_GPS: GPS receiver drivers and position estimation
 * - AP_Baro: Barometer drivers for altitude measurement
 * - AP_Compass: Magnetometer drivers for heading reference
 * - AP_RangeFinder: Distance sensor integration for terrain following and landing
 * - AP_OpticalFlow: Optical flow sensor for velocity estimation
 * - AP_RPM: Engine RPM monitoring
 * - AP_Beacon: Beacon-based positioning system
 * 
 * **Navigation and State Estimation:**
 * - AP_AHRS: Attitude and Heading Reference System (DCM and EKF backends)
 * - AP_NavEKF2/3: Extended Kalman Filter for sensor fusion and state estimation
 * - AP_L1_Control: L1 guidance controller for waypoint tracking
 * - AP_Navigation: Navigation helper functions
 * - AP_TECS: Total Energy Control System for altitude/airspeed management
 * - AP_InertialNav: Inertial navigation integration
 * 
 * **Control Systems:**
 * - APM_Control: Roll/pitch/yaw/steer controller implementations
 * - AP_AutoTune: Automatic PID tuning system
 * - AC_AttitudeControl: Multicopter attitude control (for QuadPlane)
 * - AC_PosControl: Multicopter position control (for QuadPlane)
 * 
 * **Communication:**
 * - GCS_MAVLink: MAVLink protocol implementation for ground station communication
 * - AP_Frsky_Telem: FrSky telemetry protocol
 * - AP_RCMapper: RC input channel mapping
 * - RC_Channel: RC receiver input processing
 * 
 * **Safety and Arming:**
 * - AP_Arming: Pre-arm and arming check system
 * - AP_AdvancedFailsafe: Outback Challenge advanced failsafe support
 * - AC_Fence: Geofencing system for flight boundary enforcement
 * 
 * **Mission Management:**
 * - AP_Mission: Mission command storage and execution
 * - AP_Rally: Rally point management for return-to-launch
 * - AP_Landing: Automated landing sequence control
 * 
 * **Peripherals and Accessories:**
 * - AP_Camera: Camera trigger and control
 * - AP_Mount: Gimbal/antenna mount control
 * - AP_OSD: On-screen display integration
 * - AP_Parachute: Parachute deployment system
 * - SRV_Channel: Servo output channel management
 * - AP_LandingGear: Landing gear retraction control
 * - AP_ICEngine: Internal combustion engine control
 * 
 * **Vehicle-Specific:**
 * - QuadPlane: Hybrid fixed-wing + multicopter VTOL functionality
 * - AP_Soaring: Thermal soaring and energy management
 * - AP_Follow: Vehicle following mode
 * 
 * **Logging and Data:**
 * - AP_Logger: Flight data logging to SD card
 * - AP_Terrain: Terrain data storage and lookup
 * - AP_BattMonitor: Battery voltage/current monitoring and failsafe
 * 
 * **Scheduling and Performance:**
 * - AP_Scheduler: Main loop task scheduler
 * - PerfInfo: Performance monitoring and loop timing
 * 
 * **Math and Filtering:**
 * - AP_Math: Vector/matrix/quaternion math library
 * - Filter: Digital filter implementations
 * 
 * **Advanced Features:**
 * - AP_ADSB: ADS-B aircraft tracking and avoidance
 * - AP_Avoidance: Obstacle avoidance integration
 * - AC_PrecLand: Precision landing with IR beacon
 * - AP_ExternalControl: External flight controller integration
 * - AP_Scripting: Lua scripting interface
 * - AP_Tuning: Real-time parameter tuning
 */
////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor/AP_InertialSensor.h> // Inertial Sensor Library
#include <AP_AccelCal/AP_AccelCal.h>                // interface and maths for accelerometer calibration
#include <AP_AHRS/AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <SRV_Channel/SRV_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder_config.h>     // Range finder library
#include <Filter/Filter.h>                     // Filter library
#include <AP_Camera/AP_Camera.h>          // Photo or video camera
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RPM/AP_RPM.h>
#include <AP_Beacon/AP_Beacon.h>

#include <AP_AdvancedFailsafe/AP_AdvancedFailsafe.h>
#include <APM_Control/APM_Control.h>
#include <APM_Control/AP_AutoTune.h>
#include <GCS_MAVLink/GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_Mount/AP_Mount.h>           // Camera/Antenna mount
#include <AP_Declination/AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>       // main loop scheduler
#include <AP_Scheduler/PerfInfo.h>                  // loop perf monitoring

#include <AP_Navigation/AP_Navigation.h>
#include <AP_L1_Control/AP_L1_Control.h>
#include <AP_RCMapper/AP_RCMapper.h>        // RC input mapping library

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_TECS/AP_TECS.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Mission/AP_Mission.h>     // Mission command library

#include <AP_Soaring/AP_Soaring.h>
#include <AP_BattMonitor/AP_BattMonitor.h> // Battery monitor library

#include <AP_Arming/AP_Arming.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <AP_OSD/AP_OSD.h>

#include <AP_Rally/AP_Rally.h>

#include <AP_OpticalFlow/AP_OpticalFlow.h>     // Optical Flow library
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Avoidance/AP_Avoidance_config.h>      // "ADSB" avoidance library
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Landing/AP_Landing.h>
#include <AP_LandingGear/AP_LandingGear.h>     // Landing Gear library
#include <AP_Follow/AP_Follow.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>
#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_ExternalControl_Plane.h"
#endif

#include <AC_PrecLand/AC_PrecLand_config.h>
#if AC_PRECLAND_ENABLED
 # include <AC_PrecLand/AC_PrecLand.h>
#endif

#include "GCS_MAVLink_Plane.h"
#include "GCS_Plane.h"
#include "quadplane.h"
#include <AP_Tuning/AP_Tuning_config.h>
#if AP_TUNING_ENABLED
#include "tuning.h"
#endif

// Configuration
#include "config.h"

#if AP_ADVANCEDFAILSAFE_ENABLED
#include "afs_plane.h"
#endif

// Local modules
#include "defines.h"
#include "mode.h"

#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/AP_Scripting.h>
#endif

#include "RC_Channel_Plane.h"     // RC Channel Library
#include "Parameters.h"
#if AP_ADSB_AVOIDANCE_ENABLED
#include "avoidance_adsb.h"
#endif  // AP_ADSB_AVOIDANCE_ENABLED
#include "AP_Arming_Plane.h"
#include "pullup.h"
#include "systemid.h"

/**
 * @class Plane
 * @brief Main vehicle class for ArduPlane fixed-wing and VTOL aircraft
 * 
 * @details The Plane class is the central coordinator inheriting from AP_Vehicle that manages all aspects
 *          of fixed-wing aircraft operation including conventional planes, flying wings, and hybrid VTOL QuadPlanes.
 * 
 * **Primary Responsibilities:**
 * 
 * **Flight Control:**
 * - Roll/pitch/yaw stabilization using PID controllers (APM_Control)
 * - TECS (Total Energy Control System) for coupled altitude/airspeed management
 * - L1 guidance controller for waypoint tracking and path following
 * - Attitude control integration with body frame to earth frame transformations
 * - Control surface mixing and servo output management
 * 
 * **Flight Mode System:**
 * - 26+ flight modes including:
 *   - Manual modes: MANUAL, ACRO, STABILIZE, TRAINING
 *   - Assisted modes: FBWA (Fly-By-Wire A), FBWB (Fly-By-Wire B), CRUISE, AUTOTUNE
 *   - Autonomous modes: AUTO, RTL (Return To Launch), LOITER, GUIDED, CIRCLE, TAKEOFF, THERMAL
 *   - QuadPlane Q-modes: QSTABILIZE, QHOVER, QLOITER, QLAND, QRTL, QACRO, QAUTOTUNE
 *   - Safety modes: AVOID_ADSB, AUTOLAND, LOITER_ALT_QLAND
 * - Mode transition logic with safety checks
 * - Mode-specific parameter sets and behaviors
 * 
 * **QuadPlane/VTOL Functionality (HAL_QUADPLANE_ENABLED):**
 * - Hybrid fixed-wing + multicopter operation
 * - Vertical takeoff and landing capabilities
 * - Transition between VTOL and fixed-wing flight
 * - Tiltrotor and tailsitter configurations
 * - VTOL-assisted fixed-wing flight for challenging conditions
 * - Separate QuadPlane subsystem with multicopter attitude/position controllers
 * 
 * **Navigation:**
 * - Waypoint following with configurable acceptance radius
 * - L1 controller for smooth path following
 * - Loiter (circle) with configurable radius and direction
 * - Return to launch with altitude management
 * - Guided mode for external control (GCS, companion computer)
 * - Mission execution from EEPROM with DO/COND commands
 * - Terrain following using rangefinder or terrain database
 * - Crosstrack error correction
 * 
 * **Safety Systems:**
 * - Comprehensive failsafe mechanisms:
 *   - RC signal loss (short and long failsafe)
 *   - GCS telemetry loss
 *   - Battery voltage/capacity failsafe
 *   - EKF navigation failure detection
 *   - Geofencing (breach actions)
 *   - ADS-B traffic avoidance
 * - Pre-arm and arming checks (AP_Arming_Plane)
 * - Crash detection and automatic disarm
 * - Stall prevention monitoring
 * - Parachute deployment support
 * - Advanced failsafe (AFS) for Outback Challenge compliance
 * 
 * **Sensor Integration:**
 * - IMU (Inertial Measurement Unit): Gyroscope and accelerometer fusion
 * - Airspeed sensor: Differential pressure with calibration and health monitoring
 * - GPS: Position, velocity, heading with multi-GPS blending
 * - Barometer: Altitude reference with temperature compensation
 * - Magnetometer: Compass heading with declination and calibration
 * - Rangefinder: Terrain following and precision landing
 * - Optical flow: Non-GPS velocity estimation
 * - Battery monitors: Voltage, current, remaining capacity
 * 
 * **Actuator Control:**
 * - Servo output mixing for control surfaces (ailerons, elevator, rudder)
 * - Motor/ESC control with throttle curves
 * - Flap deployment based on airspeed and mode
 * - Differential spoilers for roll control
 * - Airbrake deployment for descent control
 * - Reverse thrust support for landing
 * - Landing gear retraction/extension
 * - Vectored thrust (tiltrotor/tailsitter)
 * 
 * **Communication:**
 * - MAVLink protocol for ground station communication (GCS_MAVLink_Plane)
 * - Telemetry streaming with configurable rates
 * - Command handling (MAV_CMD_*)
 * - Mission upload/download protocol
 * - Parameter get/set protocol
 * - RC input processing with multiple protocols (SBUS, PPM, DSM, etc.)
 * - FrSky telemetry downlink
 * 
 * **Logging:**
 * - Comprehensive flight data recording via AP_Logger
 * - Log messages for attitude, navigation, control, sensors, performance
 * - Configurable log bitmask for selective logging
 * - SD card and flash storage backends
 * - Log replay capability for algorithm development
 * 
 * **Parameter System:**
 * - 500+ configurable parameters organized in groups (Parameters, ParametersG2)
 * - EEPROM persistence with wear leveling
 * - Runtime parameter modification via GCS
 * - Parameter defaults per vehicle type
 * - Parameter validation and constraints
 * 
 * **Scheduler Architecture:**
 * - Task-based scheduling with configurable rates (scheduler_tasks[])
 * - Priority-based task execution
 * - Performance monitoring and loop timing
 * - Typical tasks:
 *   - Fast loop (50-400 Hz): Attitude control, TECS update
 *   - Medium loop (10-50 Hz): Navigation, sensor reading
 *   - Slow loop (1-10 Hz): GCS communication, logging, monitoring
 * 
 * **Advanced Features:**
 * - Thermal soaring with vario and energy management
 * - Automatic takeoff with ground roll detection
 * - Automatic landing with flare and touchdown detection
 * - Formation flying and vehicle following
 * - Lua scripting for custom behaviors (AP_Scripting)
 * - External control integration for research/autonomy
 * - In-flight parameter tuning (AP_Tuning)
 * - System identification for model-based control
 * 
 * **Coordinate Systems:**
 * - Body frame: X-forward (nose), Y-right (right wing), Z-down
 * - NED (North-East-Down) earth frame for navigation
 * - Attitude representation: Quaternions (AHRS), Euler angles (control), DCM matrices
 * 
 * **Unit Conventions:**
 * - Angles: Centidegrees (roll_limit_cd) or radians (nav calculations)
 * - Distance: Centimeters (target_altitude_cm) or meters (waypoint calculations)
 * - Speed: Centimeters/second (target_airspeed_cm) or meters/second (TECS)
 * - Time: Milliseconds (timestamps) or seconds (control loops)
 * 
 * **Architecture Pattern:**
 * - Singleton pattern: Global 'plane' instance accessed throughout codebase
 * - Friend classes for tight integration with subsystems
 * - Mode polymorphism: Base Mode class with virtual methods for mode-specific behavior
 * - Event-driven for sensor updates and external events
 * - Periodic task scheduling for control loops
 * 
 * **Initialization Sequence:**
 * 1. Hardware initialization (HAL)
 * 2. init_ardupilot(): Parameter loading, sensor initialization, mode setup
 * 3. Scheduler task registration
 * 4. Pre-arm checks until ready to arm
 * 5. Arming sequence with safety checks
 * 6. Normal flight operation
 * 
 * **Main Loop Flow:**
 * 1. Scheduler executes tasks based on rates and priorities
 * 2. Fast loop: Read sensors → Update AHRS → Run attitude control → Output servos
 * 3. Medium loop: Update navigation → Process RC input → Check failsafes
 * 4. Slow loop: GCS communication → Logging → Parameter updates
 * 
 * @note This is a singleton class - access via the global 'plane' instance
 * @note Main entry point is Plane::init_ardupilot() called from HAL main()
 * @note Scheduler tasks defined in scheduler_tasks[] array with rates and priorities
 * 
 * @warning This is flight-critical code controlling aircraft capable of causing injury or property damage
 * @warning All modifications require extensive ground testing, SITL validation, and safe flight testing
 * @warning Incorrect control law changes can result in unstable flight or loss of control
 * @warning Safety systems (failsafe, arming checks) must never be weakened without thorough analysis
 * 
 * @see Mode Base class for flight mode implementations
 * @see QuadPlane For VTOL/hybrid functionality
 * @see AP_TECS For altitude/airspeed control algorithm
 * @see AP_L1_Control For waypoint navigation guidance
 */
class Plane : public AP_Vehicle {
public:
    /**
     * @brief Friend class declarations allowing subsystems direct access to Plane private members
     * 
     * @details This extensive friend list enables tight integration while maintaining encapsulation.
     *          Each friend class has legitimate need for direct access to Plane internals:
     * 
     * **Subsystem Friends:**
     * - GCS_MAVLINK_Plane, GCS_Plane: Ground station communication requiring access to all vehicle state
     * - Parameters, ParametersG2: Parameter system with direct access to member variables
     * - AP_Arming_Plane: Arming checks needing to examine safety-critical state
     * - QuadPlane, QAutoTune, VTOL_Assist, Tailsitter, Tiltrotor: VTOL subsystems requiring deep integration
     * - SLT_Transition, Tailsitter_Transition: VTOL transition state machines
     * - AP_Tuning_Plane: Real-time parameter tuning
     * - AP_AdvancedFailsafe_Plane: Advanced failsafe requiring full vehicle state access
     * - AP_Avoidance_Plane: Obstacle avoidance needing control authority
     * - RC_Channel_Plane, RC_Channels_Plane: RC input processing with mode interactions
     * - AP_ExternalControl_Plane: External flight control interface
     * - GliderPullup: Glider pullup maneuver requiring control override
     * - AP_SystemID: System identification requiring excitation and measurement
     * 
     * **Flight Mode Friends:**
     * All Mode-derived classes (Mode base class and specific mode implementations) have friend access
     * to implement mode-specific behaviors with direct access to navigation, control, and sensor state:
     * - Mode: Base class for all flight modes
     * - Manual modes: ModeManual, ModeStabilize, ModeTraining, ModeAcro
     * - FBW modes: ModeFBWA, ModeFBWB, ModeCruise
     * - Autonomous modes: ModeAuto, ModeRTL, ModeLoiter, ModeCircle, ModeTakeoff, ModeGuided
     * - QuadPlane modes: ModeQStabilize, ModeQHover, ModeQLoiter, ModeQLand, ModeQRTL, ModeQAcro, ModeQAutotune
     * - Advanced modes: ModeAutoTune, ModeAvoidADSB, ModeThermal, ModeLoiterAltQLand, ModeAutoLand
     * - Special modes: ModeInitializing
     */
    friend class GCS_MAVLINK_Plane;
    friend class Parameters;
    friend class ParametersG2;
    friend class AP_Arming_Plane;
    friend class QuadPlane;
    friend class QAutoTune;
    friend class AP_Tuning_Plane;
    friend class AP_AdvancedFailsafe_Plane;
    friend class AP_Avoidance_Plane;
    friend class GCS_Plane;
    friend class RC_Channel_Plane;
    friend class RC_Channels_Plane;
    friend class Tailsitter;
    friend class Tiltrotor;
    friend class SLT_Transition;
    friend class Tailsitter_Transition;
    friend class VTOL_Assist;

    friend class Mode;
    friend class ModeCircle;
    friend class ModeStabilize;
    friend class ModeTraining;
    friend class ModeAcro;
    friend class ModeFBWA;
    friend class ModeFBWB;
    friend class ModeCruise;
    friend class ModeAutoTune;
    friend class ModeAuto;
    friend class ModeRTL;
    friend class ModeLoiter;
    friend class ModeAvoidADSB;
    friend class ModeGuided;
    friend class ModeInitializing;
    friend class ModeManual;
    friend class ModeQStabilize;
    friend class ModeQHover;
    friend class ModeQLoiter;
    friend class ModeQLand;
    friend class ModeQRTL;
    friend class ModeQAcro;
    friend class ModeQAutotune;
    friend class ModeTakeoff;
    friend class ModeThermal;
    friend class ModeLoiterAltQLand;
#if MODE_AUTOLAND_ENABLED
    friend class ModeAutoLand;
#endif
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Plane;
#endif
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    friend class GliderPullup;
#endif
#if AP_PLANE_SYSTEMID_ENABLED
    friend class AP_SystemID;
#endif

    Plane(void);

private:

    /**
     * @brief Fixed-wing aircraft parameters shared across libraries
     * @details AP_FixedWing aparm contains parameters specific to fixed-wing flight that are passed
     *          to control libraries (TECS, L1, attitude controllers) including airframe limits,
     *          control surface parameters, and flight performance characteristics
     */
    AP_FixedWing aparm;

    /**
     * @brief Primary and secondary parameter groups stored in EEPROM
     * @details g contains the original parameter table, g2 contains newer parameters added to avoid
     *          EEPROM reorganization. Both are saved persistently and loaded on boot.
     */
    Parameters g;
    ParametersG2 g2;

    // mapping between input channels
    RCMapper rcmap;

    // primary input channels
    RC_Channel *channel_roll;
    RC_Channel *channel_pitch;
    RC_Channel *channel_throttle;
    RC_Channel *channel_rudder;
    RC_Channel *channel_flap;
    RC_Channel *channel_airbrake;

    // scaled roll limit based on pitch
    int32_t roll_limit_cd;
    float pitch_limit_min;

    // flight modes convenience array
    AP_Int8 *flight_modes = &g.flight_mode1;
    const uint8_t num_flight_modes = 6;

#if AP_RANGEFINDER_ENABLED
    AP_FixedWing::Rangefinder_State rangefinder_state;

    /*
      orientation of rangefinder to use for landing
     */
    Rotation rangefinder_orientation(void) const {
        return Rotation(g2.rangefinder_land_orient.get());
    }

#endif

#if AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED
    struct {
        // allow for external height above ground estimate
        float hagl;
        uint32_t last_update_ms;
        uint32_t timeout_ms;
    } external_hagl;
    bool get_external_HAGL(float &height_agl);
    void handle_external_hagl(const mavlink_command_int_t &packet);
#endif // AP_MAVLINK_MAV_CMD_SET_HAGL_ENABLED

    float get_landing_height(bool &using_rangefinder);


#if AP_RPM_ENABLED
    AP_RPM rpm_sensor;
#endif

    /**
     * @brief Total Energy Control System for coupled altitude/airspeed management
     * @details TECS manages the energy state of the aircraft by controlling throttle and pitch
     *          to achieve target altitude and airspeed while optimizing energy efficiency.
     *          Critical for smooth altitude changes, airspeed transitions, and coordinated climbs/descents.
     */
    AP_TECS TECS_controller{ahrs, aparm, landing, MASK_LOG_TECS};
    
    /**
     * @brief L1 guidance controller for waypoint tracking and path following
     * @details L1 controller provides smooth navigation guidance by computing lateral acceleration
     *          commands to follow straight lines, arcs, and loiter circles. Produces nav_roll_cd output.
     */
    AP_L1_Control L1_controller{ahrs, &TECS_controller};

    /**
     * @brief Attitude to servo controllers (roll, pitch, yaw, steering)
     * @details PID-based controllers that convert desired attitude (nav_roll_cd, nav_pitch_cd) to
     *          servo deflections. Include rate feedforward, integrator management, and airspeed scaling.
     *          Steer controller used for ground steering during takeoff/landing.
     */
    AP_RollController rollController{aparm};
    AP_PitchController pitchController{aparm};
    AP_YawController yawController{aparm};
    AP_SteerController steerController{};

    // Training mode
    bool training_manual_roll;  // user has manual roll control
    bool training_manual_pitch; // user has manual pitch control

    // should throttle be pass-thru in guided?
    bool guided_throttle_passthru;

    // are we doing calibration? This is used to allow heartbeat to
    // external failsafe boards during baro and airspeed calibration
    bool in_calibration;

    // are we currently in long failsafe but have postponed it in MODE TAKEOFF until min level alt is reached
    bool long_failsafe_pending;

    // GCS selection
    GCS_Plane _gcs; // avoid using this; use gcs()
    GCS_Plane &gcs() { return _gcs; }

    // selected navigation controller
    AP_Navigation *nav_controller = &L1_controller;

    // Camera
#if AP_CAMERA_ENABLED
    AP_Camera camera{MASK_LOG_CAMERA};
#endif

#if AP_OPTICALFLOW_ENABLED
    // Optical flow sensor
    AP_OpticalFlow optflow;
#endif

#if HAL_RALLY_ENABLED
    // Rally Points
    AP_Rally rally;
#endif

#if AC_PRECLAND_ENABLED
    void precland_update(void);
#endif

    // returns a Location for a rally point or home; if
    // HAL_RALLY_ENABLED is false, just home.
    Location calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt_amsl_cm) const;

#if OSD_ENABLED || OSD_PARAM_ENABLED
    AP_OSD osd;
#endif

    /**
     * @brief Flight mode object instances
     * @details Each mode is instantiated as a member variable. The control_mode pointer
     *          references the currently active mode. Mode objects implement enter(), update(),
     *          and other virtual methods to define mode-specific behavior.
     */
    ModeCircle mode_circle;
    ModeStabilize mode_stabilize;
    ModeTraining mode_training;
    ModeAcro mode_acro;
    ModeFBWA mode_fbwa;
    ModeFBWB mode_fbwb;
    ModeCruise mode_cruise;
    ModeAutoTune mode_autotune;
    ModeAuto mode_auto;
    ModeRTL mode_rtl;
    ModeLoiter mode_loiter;
#if HAL_ADSB_ENABLED
    ModeAvoidADSB mode_avoidADSB;
#endif
    ModeGuided mode_guided;
    ModeInitializing mode_initializing;
    ModeManual mode_manual;
#if HAL_QUADPLANE_ENABLED
    ModeQStabilize mode_qstabilize;
    ModeQHover mode_qhover;
    ModeQLoiter mode_qloiter;
    ModeQLand mode_qland;
    ModeQRTL mode_qrtl;
    ModeQAcro mode_qacro;
    ModeLoiterAltQLand mode_loiter_qland;
#if QAUTOTUNE_ENABLED
    ModeQAutotune mode_qautotune;
#endif  // QAUTOTUNE_ENABLED
#endif  // HAL_QUADPLANE_ENABLED
    ModeTakeoff mode_takeoff;
#if MODE_AUTOLAND_ENABLED
    ModeAutoLand mode_autoland;
#endif
#if HAL_SOARING_ENABLED
    ModeThermal mode_thermal;
#endif

#if AP_QUICKTUNE_ENABLED
    AP_Quicktune quicktune;
#endif
    
    /**
     * @brief Current and previous flight control mode pointers
     * @details control_mode points to the currently active Mode object (e.g., &mode_auto, &mode_fbwa).
     *          previous_mode stores the last active mode for restoration after failsafe recovery.
     *          Mode changes occur via set_mode() which calls exit() on old mode and enter() on new mode.
     */
    Mode *control_mode = &mode_initializing;
    Mode *previous_mode = &mode_initializing;

    // time of last mode change
    uint32_t last_mode_change_ms;

    // This is used to enable the inverted flight feature
    bool inverted_flight;

    // last time we ran roll/pitch stabilization
    uint32_t last_stabilize_ms;

    // Failsafe
    struct {
        // Used to track if the value on channel 3 (throttle) has fallen below the failsafe threshold
        // RC receiver should be set up to output a low throttle value when signal is lost
        bool rc_failsafe;

        // true if an adsb related failsafe has occurred
        bool adsb;

        // saved flight mode
        enum Mode::Number saved_mode_number;

        // A tracking variable for type of failsafe active
        // Used for failsafe based on loss of RC signal or GCS signal
        int16_t state;

        // number of low throttle values
        uint8_t throttle_counter;

        // A timer used to track how long we have been in a "short failsafe" condition due to loss of RC signal
        uint32_t short_timer_ms;

        uint32_t last_valid_rc_ms;

        //keeps track of the last valid rc as it relates to the AFS system
        //Does not count rc inputs as valid if the standard failsafe is on
        uint32_t AFS_last_valid_rc_ms;
    } failsafe;

#if HAL_QUADPLANE_ENABLED
    // Landing
    class VTOLApproach {
    public:
        enum class Stage {
            RTL,
            LOITER_TO_ALT,
            ENSURE_RADIUS,
            WAIT_FOR_BREAKOUT,
            APPROACH_LINE,
            VTOL_LANDING,
        };

        Stage approach_stage;
        float approach_direction_deg;
    } vtol_approach_s;
#endif

    bool any_failsafe_triggered() {
        return failsafe.state != FAILSAFE_NONE || battery.has_failsafed() || failsafe.adsb;
    }

    // A counter used to count down valid gps fixes to allow the gps estimate to settle
    // before recording our home position (and executing a ground start if we booted with an air start)
    uint8_t ground_start_count = 5;

    // true if we have a position estimate from AHRS
    bool have_position;

    // Airspeed
    // The calculated airspeed to use in FBW-B.  Also used in higher modes for insuring min ground speed is met.
    // Also used for flap deployment criteria.  Centimeters per second.
    int32_t target_airspeed_cm;
    int32_t new_airspeed_cm = -1;  //temp variable for AUTO and GUIDED mode speed changes

    // The difference between current and desired airspeed.  Used in the pitch controller.  Meters per second.
    float airspeed_error;

    // An amount that the airspeed should be increased in auto modes based on the user positioning the
    // throttle stick in the top half of the range.  Centimeters per second.
    int16_t airspeed_nudge_cm;

    // Similar to airspeed_nudge, but used when no airspeed sensor.
    // 0-(throttle_max - throttle_cruise) : throttle nudge in Auto mode using top 1/2 of throttle stick travel
    int16_t throttle_nudge;

    // Ground speed
    // The amount current ground speed is below min ground speed.  Centimeters per second
    int32_t groundspeed_undershoot;
    bool groundspeed_undershoot_is_valid;
    float last_groundspeed_undershoot_offset;

    // speed scaler for control surfaces, updated at 10Hz
    float surface_speed_scaler = 1.0;

    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT,
                           FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
                           _failsafe_priorities};

    struct {
        uint32_t last_tkoff_arm_time;
        uint32_t last_check_ms;
        uint32_t rudder_takeoff_warn_ms;
        uint32_t last_report_ms;
        bool launchTimerStarted;
        uint8_t accel_event_counter;
        uint32_t accel_event_ms;
        uint32_t start_time_ms;
        bool waiting_for_rudder_neutral;
        float throttle_lim_max;
        float throttle_lim_min;
        uint32_t throttle_max_timer_ms;
        uint32_t level_off_start_time_ms;
        // Good candidate for keeping the initial time for TKOFF_THR_MAX_T.
#if MODE_AUTOLAND_ENABLED
       struct {
            float heading; // deg
            bool initialized;
        } initial_direction;
#endif
    } takeoff_state;

    // ground steering controller state
    struct {
        // Direction held during phases of takeoff and landing centidegrees
        // A value of -1 indicates the course has not been set/is not in use
        // this is a 0..36000 value, or -1 for disabled
        int32_t hold_course_cd = -1;

        // locked_course and locked_course_cd are used in stabilize mode
        // when ground steering is active, and for steering in auto-takeoff
        bool locked_course;
        float locked_course_err;
        uint32_t last_steer_ms;
    } steer_state;

    // flight mode specific
    struct {
        // Altitude threshold to complete a takeoff command in autonomous
        // modes.  Centimeters above home
        int32_t takeoff_altitude_rel_cm;

        // Begin leveling out the enforced takeoff pitch angle min at this height to reduce/eliminate overshoot
        int32_t height_below_takeoff_to_level_off_cm;

        // the highest airspeed we have reached since entering AUTO. Used
        // to control ground takeoff
        float highest_airspeed;

        // turn angle for next leg of mission
        float next_turn_angle {90};

        // filtered sink rate for landing
        float sink_rate;

        // distance to next waypoint
        float wp_distance;

        // proportion to next waypoint
        float wp_proportion;

        // last time is_flying() returned true in milliseconds
        uint32_t last_flying_ms;

        // time stamp of when we start flying while in auto mode in milliseconds
        uint32_t started_flying_in_auto_ms;

        // barometric altitude at start of takeoff
        float baro_takeoff_alt;

        // initial pitch. Used to detect if nose is rising in a tail dragger
        int16_t initial_pitch_cd;

        // Minimum pitch to hold during takeoff command execution.  Hundredths of a degree
        int16_t takeoff_pitch_cd;

        // Flag for using gps ground course instead of INS yaw.  Set false when takeoff command in process.
        bool takeoff_complete;

        // are we headed to the land approach waypoint? Works for any nav type
        bool wp_is_land_approach;

        // should we fly inverted?
        bool inverted_flight;

        // should we enable cross-tracking for the next waypoint?
        bool next_wp_crosstrack;

        // should we use cross-tracking for this waypoint?
        bool crosstrack;

        // in FBWA taildragger takeoff mode
        bool fbwa_tdrag_takeoff_mode;

        // have we checked for an auto-land?
        bool checked_for_autoland;

        // Altitude threshold to complete a takeoff command in autonomous modes.  Centimeters
        // are we in idle mode? used for balloon launch to stop servo
        // movement until altitude is reached
        bool idle_mode;
        
        // are we in VTOL mode in AUTO?
        bool vtol_mode;

        // are we doing loiter mode as a VTOL?
        bool vtol_loiter;

        // how much correction have we added for terrain data
        float terrain_correction;

        // last home altitude for detecting changes
        int32_t last_home_alt_cm;

        // have we finished the takeoff ratation (when it applies)?
        bool rotation_complete;
    } auto_state;

#if AP_SCRIPTING_ENABLED
    // support for scripting nav commands, with verify
    struct {
        bool enabled;
        uint16_t id;
        float roll_rate_dps;
        float pitch_rate_dps;
        float yaw_rate_dps;
        float throttle_pct;
        uint32_t start_ms;
        uint32_t current_ms;
        float rudder_offset_pct;
        bool run_yaw_rate_controller;
    } nav_scripting;
#endif

    struct {
        // roll pitch yaw commanded from external controller in centidegrees
        Vector3l forced_rpy_cd;
        // last time we heard from the external controller
        Vector3l last_forced_rpy_ms;

        // throttle  commanded from external controller in percent
        float forced_throttle;
        uint32_t last_forced_throttle_ms;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        // airspeed adjustments
        float target_airspeed_cm = -1;  // don't default to zero here, as zero is a valid speed.
        float target_airspeed_accel;
        uint32_t target_airspeed_time_ms;

        // altitude adjustments
        Location target_location;
        float target_alt_rate;
        uint32_t target_alt_time_ms = 0;
        uint8_t target_mav_frame = -1;

        // heading track
        float target_heading = -4; // don't default to zero or -1 here, as both are valid headings in radians
        float target_heading_accel_limit;
        uint32_t target_heading_time_ms;
        guided_heading_type_t target_heading_type;
        bool target_heading_limit;
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } guided_state;

#if AP_LANDINGGEAR_ENABLED
    // landing gear state
    struct {
        AP_FixedWing::FlightStage last_flight_stage;
    } gear;
#endif

    struct {
        // on hard landings, only check once after directly a landing so you
        // don't trigger a crash when picking up the aircraft
        bool checkedHardLanding;

        // crash detection. True when we are crashed
        bool is_crashed;

        // impact detection flag. Expires after a few seconds via impact_timer_ms
        bool impact_detected;

        // debounce timer
        uint32_t debounce_timer_ms;

        // delay time for debounce to count to
        uint32_t debounce_time_total_ms;

        // length of time impact_detected has been true. Times out after a few seconds. Used to clip isFlyingProbability
        uint32_t impact_timer_ms;
    } crash_state;

    // this controls throttle suppression in auto modes
    bool throttle_suppressed;

#if AP_BATTERY_WATT_MAX_ENABLED
    // reduce throttle to eliminate battery over-current
    int8_t  throttle_watt_limit_max;
    int8_t  throttle_watt_limit_min; // for reverse thrust
    uint32_t throttle_watt_limit_timer_ms;
#endif

    AP_FixedWing::FlightStage flight_stage = AP_FixedWing::FlightStage::NORMAL;

    // probability of aircraft is currently in flight. range from 0 to
    // 1 where 1 is 100% sure we're in flight
    float isFlyingProbability;

    // previous value of is_flying()
    bool previous_is_flying;

    // time since started flying in any mode in milliseconds
    uint32_t started_flying_ms;

    // ground mode is true when disarmed and not flying
    bool ground_mode;

    // Navigation control variables
    // The instantaneous desired bank angle.  Hundredths of a degree
    int32_t nav_roll_cd;

    // The instantaneous desired pitch angle.  Hundredths of a degree
    int32_t nav_pitch_cd;

    // the aerodynamic load factor. This is calculated from the demanded
    // roll before the roll is clipped, using 1/cos(nav_roll)
    float aerodynamic_load_factor = 1.0f;

    // a smoothed airspeed estimate, used for limiting roll angle
    float smoothed_airspeed;

    // Mission library
    AP_Mission mission{
            FUNCTOR_BIND_MEMBER(&Plane::start_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::verify_command_callback, bool, const AP_Mission::Mission_Command &),
            FUNCTOR_BIND_MEMBER(&Plane::exit_mission_callback, void)};


#if HAL_PARACHUTE_ENABLED
    AP_Parachute parachute;
#endif

    // terrain handling
#if AP_TERRAIN_AVAILABLE
    AP_Terrain terrain;
#endif

    AP_Landing landing{mission,ahrs,&TECS_controller,nav_controller,aparm,
            FUNCTOR_BIND_MEMBER(&Plane::set_target_altitude_proportion, void, const Location&, float),
            FUNCTOR_BIND_MEMBER(&Plane::constrain_target_altitude_location, void, const Location&, const Location&),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::adjusted_relative_altitude_cm, int32_t),
            FUNCTOR_BIND_MEMBER(&Plane::disarm_if_autoland_complete, void),
            FUNCTOR_BIND_MEMBER(&Plane::update_flight_stage, void)};
#if HAL_ADSB_ENABLED
    AP_ADSB adsb;
#endif  // HAL_ADSB_ENABLED

#if AP_ADSB_AVOIDANCE_ENABLED
    // avoidance of adsb enabled vehicles (normally manned vehicles)
    AP_Avoidance_Plane avoidance_adsb{adsb};
#endif  // AP_ADSB_AVOIDANCE_ENABLED

    // Outback Challenge Failsafe Support
#if AP_ADVANCEDFAILSAFE_ENABLED
    AP_AdvancedFailsafe_Plane afs;
#endif

    /*
      meta data to support counting the number of circles in a loiter
    */
    struct {
        // previous target bearing, used to update sum_cd
        int32_t old_target_bearing_cd;

        // Total desired rotation in a loiter.  Used for Loiter Turns commands.
        int32_t total_cd;

        // total angle completed in the loiter so far
        int32_t sum_cd;

        // Direction for loiter. 1 for clockwise, -1 for counter-clockwise
        int8_t direction;

        // when loitering and an altitude is involved, this flag is true when it has been reached at least once
        bool reached_target_alt;

        // check for scenarios where updrafts can keep you from loitering down indefinitely.
        bool unable_to_acheive_target_alt;

        // start time of the loiter.  Milliseconds.
        uint32_t start_time_ms;

        // altitude at start of loiter loop lap. Used to detect delta alt of each lap.
        // only valid when sum_cd > 36000
        int32_t start_lap_alt_cm;
        int32_t next_sum_lap_cd;

        // The amount of time we should stay in a loiter for the Loiter Time command.  Milliseconds.
        uint32_t time_max_ms;

        // current value of loiter radius in metres used by the controller
        float radius;
    } loiter;

    // Conditional command
    // A value used in condition commands (eg delay, change alt, etc.)
    // For example in a change altitude command, it is the altitude to change to.
    int32_t condition_value;

    // A starting value used to check the status of a conditional command.
    // For example in a delay command the condition_start records that start time for the delay
    uint32_t condition_start;

    // 3D Location vectors
    // Location structure defined in AP_Common
    const Location &home = ahrs.get_home();

    // The location of the previous waypoint.  Used for track following and altitude ramp calculations
    Location prev_WP_loc {};

    // The plane's current location
    Location current_loc {};

    // The location of the current/active waypoint.  Used for altitude ramp, track following and loiter calculations.
    Location next_WP_loc {};

    // Altitude control
    struct {
        // target altitude above sea level in cm. Used for barometric
        // altitude navigation
        int32_t amsl_cm;

        // Altitude difference between previous and current waypoint in
        // centimeters. Used for altitude slope handling
        int32_t offset_cm;

#if AP_TERRAIN_AVAILABLE
        // are we trying to follow terrain?
        bool terrain_following;

        // are we waiting to load terrain data to init terrain following
        bool terrain_following_pending;

        // target altitude above terrain in cm, valid if terrain_following
        // is set
        int32_t terrain_alt_cm;

        // lookahead value for height error reporting
        float lookahead;
#endif

        // last input for FBWB/CRUISE height control
        float last_elevator_input;

        // last time we checked for pilot control of height
        uint32_t last_elev_check_us;
    } target_altitude {};

    float relative_altitude;

    struct {
        uint32_t last_trim_check;
        uint32_t last_trim_save;
    } auto_trim;

    struct {
        bool done_climb;
    } rtl;

    // last time home was updated while disarmed
    uint32_t last_home_update_ms;

    // Camera/Antenna mount tracking and stabilisation stuff
#if HAL_MOUNT_ENABLED
    AP_Mount camera_mount;
#endif

    // Arming/Disarming management class
    AP_Arming_Plane arming;

    AP_Param param_loader {var_info};

    // external control library
#if AP_EXTERNAL_CONTROL_ENABLED
    AP_ExternalControl_Plane external_control;
#endif

    /**
     * @brief Scheduler task table defining periodic functions and their execution rates
     * @details Array of AP_Scheduler::Task entries specifying:
     *          - Function pointer to task
     *          - Execution rate in Hz (e.g., 50Hz, 10Hz, 1Hz)
     *          - Maximum expected execution time in microseconds
     *          - Priority level for overload conditions
     *          Tasks include attitude control, navigation, sensor reading, GCS communication, logging
     */
    static const AP_Scheduler::Task scheduler_tasks[];
    
    /**
     * @brief Parameter metadata table for AP_Param system
     * @details Defines all parameters, their types, storage locations, defaults, and metadata
     *          Used by AP_Param for EEPROM persistence and GCS parameter protocol
     */
    static const AP_Param::Info var_info[];

#if HAL_QUADPLANE_ENABLED
    /**
     * @brief QuadPlane hybrid VTOL subsystem
     * @details Manages all multicopter/VTOL functionality including:
     *          - Vertical takeoff and landing
     *          - Hover and multicopter flight modes (QSTABILIZE, QHOVER, QLOITER, etc.)
     *          - Transition between fixed-wing and VTOL flight
     *          - Tiltrotor servo control and transition state machines
     *          - Tailsitter orientation management
     *          - VTOL-assisted fixed-wing flight
     *          - Separate multicopter attitude and position controllers
     * @note Only available when HAL_QUADPLANE_ENABLED is defined
     */
    QuadPlane quadplane{ahrs};
#endif

#if AP_TUNING_ENABLED
    // support for transmitter tuning
    AP_Tuning_Plane tuning;
#endif

    static const struct LogStructure log_structure[];

    // rudder mixing gain for differential thrust (0 - 1)
    float rudder_dt;

    // soaring mode-change timer
    uint32_t soaring_mode_timer_ms;

    // terrain disable for non AUTO modes, set with an RC Option switch
    bool non_auto_terrain_disable;
    bool terrain_disabled();
#if AP_TERRAIN_AVAILABLE
    bool terrain_enabled_in_current_mode() const;
    bool terrain_enabled_in_mode(Mode::Number num) const;
    enum class terrain_bitmask {
        ALL             = 1U << 0,
        FLY_BY_WIRE_B   = 1U << 1,
        CRUISE          = 1U << 2,
        AUTO            = 1U << 3,
        RTL             = 1U << 4,
        AVOID_ADSB      = 1U << 5,
        GUIDED          = 1U << 6,
        LOITER          = 1U << 7,
        CIRCLE          = 1U << 8,
        QRTL            = 1U << 9,
        QLAND           = 1U << 10,
        QLOITER         = 1U << 11,
        AUTOLAND        = 1U << 12,
    };
    struct TerrainLookupTable{
       Mode::Number mode_num;
       terrain_bitmask bitmask;
    };
    static const TerrainLookupTable Terrain_lookup[];
#endif

#if AP_QUICKTUNE_ENABLED
    void update_quicktune(void);
#endif

    // ========================================================================
    // Attitude and Altitude Control Methods (Attitude.cpp)
    // ========================================================================
    /**
     * @name Attitude and Altitude Control
     * @brief Methods for altitude management, attitude stabilization, and control loop execution
     * @{
     */
    
    void adjust_nav_pitch_throttle(void);  ///< Apply pitch/throttle adjustments from navigation
    void update_load_factor(void);  ///< Calculate aerodynamic load factor from roll angle
    void adjust_altitude_target();  ///< Adjust target altitude based on pilot input in FBWB/CRUISE
    void setup_alt_slope(void);  ///< Calculate altitude slope between waypoints
    int32_t get_RTL_altitude_cm() const;  ///< Get target altitude for RTL mode
    bool rangefinder_use(enum RangeFinderUse rangefinder_use) const;  ///< Check if rangefinder should be used
    float relative_ground_altitude(enum RangeFinderUse rangefinder_use);  ///< Get altitude above ground
    float relative_ground_altitude(enum RangeFinderUse rangefinder_use, bool use_terrain_if_available);  ///< Get altitude above ground or terrain
    void set_target_altitude_current(void);  ///< Set target altitude to current altitude
    void set_target_altitude_location(const Location &loc);  ///< Set target altitude from location
    int32_t relative_target_altitude_cm(void);  ///< Get target altitude relative to home
    void change_target_altitude(int32_t change_cm);  ///< Adjust target altitude by delta
    void set_target_altitude_proportion(const Location &loc, float proportion);  ///< Blend altitude between waypoints
#if AP_TERRAIN_AVAILABLE
    bool set_target_altitude_proportion_terrain(void);  ///< Set terrain following altitude proportion
#endif
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2);  ///< Constrain target between two altitudes
    int32_t calc_altitude_error_cm(void);  ///< Calculate difference between current and target altitude
    void check_fbwb_altitude(void);  ///< Check for pilot altitude adjustments in FBWB
    void reset_offset_altitude(void);  ///< Reset altitude offset to zero
    void set_offset_altitude_location(const Location &start_loc, const Location &destination_loc);  ///< Set altitude offset for slope
    bool above_location_current(const Location &loc);  ///< Check if above given location
    void setup_terrain_target_alt(Location &loc);  ///< Configure terrain following target altitude
    int32_t adjusted_altitude_cm(void);  ///< Get altitude adjusted for terrain/rangefinder
    int32_t adjusted_relative_altitude_cm(void);  ///< Get relative altitude adjusted for terrain/rangefinder
    float mission_alt_offset(void);  ///< Calculate mission altitude offset
    float height_above_target(void);  ///< Get height above target altitude
    float lookahead_adjustment(void);  ///< Calculate terrain lookahead adjustment
    void fix_terrain_WP(Location &loc, uint32_t linenum);  ///< Fix waypoint altitude for terrain data
#if AP_RANGEFINDER_ENABLED
    float rangefinder_correction(void);  ///< Get rangefinder altitude correction
    void rangefinder_height_update(void);  ///< Update rangefinder height measurement
    void rangefinder_terrain_correction(float &height);  ///< Apply rangefinder terrain correction
#endif
    void stabilize();  ///< Main stabilization function - calls roll/pitch/yaw stabilization
    void calc_throttle();  ///< Calculate throttle output using TECS
    void calc_nav_roll();  ///< Calculate desired roll angle from navigation
    void calc_nav_pitch();  ///< Calculate desired pitch angle from navigation
    float calc_speed_scaler(void);  ///< Calculate control surface scaling based on airspeed
    float get_speed_scaler(void) const { return surface_speed_scaler; }  ///< Get current speed scaler
    bool stick_mixing_enabled(void);  ///< Check if pilot stick mixing is enabled
    void stabilize_roll();  ///< Roll stabilization loop
    float stabilize_roll_get_roll_out();  ///< Get roll controller output
    void stabilize_pitch();  ///< Pitch stabilization loop
    float stabilize_pitch_get_pitch_out();  ///< Get pitch controller output
    void stabilize_stick_mixing_fbw();  ///< Mix pilot stick input in FBW modes
    void stabilize_yaw();  ///< Yaw stabilization loop
    int16_t calc_nav_yaw_coordinated();  ///< Calculate coordinated turn yaw
    int16_t calc_nav_yaw_course(void);  ///< Calculate yaw to track course
    int16_t calc_nav_yaw_ground(void);  ///< Calculate yaw based on ground track
    /** @} */ // End of Attitude and Altitude Control group

#if HAL_LOGGING_ENABLED

    // ========================================================================
    // Logging Methods (Log.cpp)
    // ========================================================================
    /**
     * @name Logging and Diagnostics
     * @brief Methods for binary logging of flight data
     * @{
     */

    // methods for AP_Vehicle:
    const AP_Int32 &get_log_bitmask() override { return g.log_bitmask; }  ///< Get logging bitmask parameter
    const struct LogStructure *get_log_structures() const override {
        return log_structure;  ///< Get log message structure definitions
    }
    uint8_t get_num_log_structures() const override;  ///< Get number of log structures

    // Log.cpp
    void Log_Write_FullRate(void);  ///< Log full rate attitude/control data
    void Log_Write_Attitude(void);  ///< Log attitude (roll/pitch/yaw)
    void Log_Write_Control_Tuning();  ///< Log control loop tuning data
    void Log_Write_OFG_Guided();  ///< Log offboard guided control data
    void Log_Write_Guided(void);  ///< Log guided mode data
    void Log_Write_Nav_Tuning();  ///< Log navigation tuning data
    void Log_Write_Status();  ///< Log vehicle status
    void Log_Write_RC(void);  ///< Log RC input channels
    void Log_Write_Vehicle_Startup_Messages();  ///< Log startup/initialization messages
    void Log_Write_AETR();  ///< Log aileron/elevator/throttle/rudder outputs
    /** @} */ // End of Logging group
#endif

    // ========================================================================
    // Parameter Management (Parameters.cpp)
    // ========================================================================
    /**
     * @name Parameter Management
     * @brief Methods for loading and managing vehicle parameters
     * @{
     */
    void load_parameters(void) override;  ///< Load parameters from EEPROM
    /** @} */ // End of Parameter Management group

    // ========================================================================
    // Mission Command Methods (commands_logic.cpp)
    // ========================================================================
    /**
     * @name Mission Command Logic
     * @brief Methods for executing and verifying mission commands
     * @details Implements AUTO mode mission command execution including takeoff,
     *          waypoint navigation, loiter patterns, landing, and conditional commands
     * @{
     */
    void set_next_WP(const Location &loc);  ///< Set next waypoint target
    void do_RTL(int32_t alt);  ///< Execute RTL (return to launch) command
    bool verify_takeoff();  ///< Verify takeoff command completion
    bool verify_loiter_unlim(const AP_Mission::Mission_Command &cmd);  ///< Verify unlimited loiter (always false - never completes)
    bool verify_loiter_time();  ///< Verify loiter time command completion
    bool verify_loiter_turns(const AP_Mission::Mission_Command &cmd);  ///< Verify loiter turns command completion
    bool verify_loiter_to_alt(const AP_Mission::Mission_Command &cmd);  ///< Verify loiter to altitude command completion
    bool verify_continue_and_change_alt();  ///< Verify continue and change altitude command
    bool verify_wait_delay();  ///< Verify delay command completion
    bool verify_within_distance();  ///< Verify within distance condition
    bool verify_altitude_wait(const AP_Mission::Mission_Command &cmd);  ///< Verify altitude wait condition
    void do_loiter_at_location();  ///< Start loiter at current location
    bool verify_loiter_heading(bool init);  ///< Verify loiter to heading completion
    void exit_mission_callback();  ///< Callback when exiting mission
    bool start_command(const AP_Mission::Mission_Command& cmd);  ///< Start executing mission command
    bool verify_command(const AP_Mission::Mission_Command& cmd);  ///< Verify mission command completion
    void do_takeoff(const AP_Mission::Mission_Command& cmd);  ///< Execute takeoff command
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);  ///< Execute waypoint navigation command
    void do_land(const AP_Mission::Mission_Command& cmd);  ///< Execute land command
#if HAL_QUADPLANE_ENABLED
    void do_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);  ///< Execute VTOL landing approach
#endif
    void loiter_set_direction_wp(const AP_Mission::Mission_Command& cmd);  ///< Set loiter direction from waypoint
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);  ///< Execute unlimited loiter command
    void do_loiter_turns(const AP_Mission::Mission_Command& cmd);  ///< Execute loiter turns command
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);  ///< Execute loiter time command
    void do_continue_and_change_alt(const AP_Mission::Mission_Command& cmd);  ///< Continue and change altitude
    void do_altitude_wait(const AP_Mission::Mission_Command& cmd);  ///< Wait for altitude condition
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);  ///< Loiter to altitude command
    void do_vtol_takeoff(const AP_Mission::Mission_Command& cmd);  ///< Execute VTOL takeoff command
    void do_vtol_land(const AP_Mission::Mission_Command& cmd);  ///< Execute VTOL land command
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);  ///< Verify waypoint navigation completion
#if HAL_QUADPLANE_ENABLED
    // vtol takeoff from AP_Vehicle for quadplane.
    bool start_takeoff(const float alt) override;  ///< Start VTOL takeoff to altitude
    bool verify_landing_vtol_approach(const AP_Mission::Mission_Command& cmd);  ///< Verify VTOL landing approach completion
#endif
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);  ///< Execute delay command
    void do_within_distance(const AP_Mission::Mission_Command& cmd);  ///< Set within distance condition
    bool do_change_speed(const AP_Mission::Mission_Command& cmd);  ///< Execute speed change command
    void do_set_home(const AP_Mission::Mission_Command& cmd);  ///< Execute set home command
    bool start_command_callback(const AP_Mission::Mission_Command &cmd);  ///< Mission library start command callback
    bool verify_command_callback(const AP_Mission::Mission_Command& cmd);  ///< Mission library verify command callback
    float get_wp_radius() const;  ///< Get waypoint acceptance radius

    bool is_land_command(uint16_t cmd) const;  ///< Check if command is a landing command

    bool do_change_speed(SPEED_TYPE speedtype, float speed_target_ms, float rhtottle_pct);  ///< Change speed (multiple types)
    /**
     * @brief Check if currently executing specific mission command
     * @param command Mission command ID to check
     * @return true if in specified AUTO mission command
     */
    bool in_auto_mission_id(uint16_t command) const;

#if AP_SCRIPTING_ENABLED
    // nav scripting support
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);  ///< Execute nav script time command
    bool verify_nav_script_time(const AP_Mission::Mission_Command& cmd);  ///< Verify nav script time completion
#endif
    /** @} */ // End of Mission Command Logic group

    // ========================================================================
    // Command and Home Management (commands.cpp)
    // ========================================================================
    /**
     * @name Command and Home Position Management
     * @brief Methods for guided mode commands and home position updates
     * @{
     */
    void set_guided_WP(const Location &loc);  ///< Set guided mode waypoint target

    // update home position. Return true if update done
    bool update_home();  ///< Update home position (called periodically)

    // update current_loc
    void update_current_loc(void);  ///< Update current vehicle location from AHRS

    // set home location and store it persistently:
    bool set_home_persistently(const Location &loc) WARN_IF_UNUSED;  ///< Set home and save to EEPROM
    bool set_home_to_current_location(bool lock) override WARN_IF_UNUSED;  ///< Set home to current position
    bool set_home(const Location& loc, bool lock) override WARN_IF_UNUSED;  ///< Set home to specified location
    /** @} */ // End of Command and Home Management group

    // ========================================================================
    // Control Mode Management (control_modes.cpp)
    // ========================================================================
    /**
     * @name Control Mode Management
     * @brief Methods for flight mode control and autotuning
     * @{
     */
    void autotune_start(void);  ///< Start autotune process
    void autotune_restore(void);  ///< Restore pre-autotune parameters
    void autotune_enable(bool enable);  ///< Enable/disable autotune
    bool fly_inverted(void);  ///< Check if flying inverted
    uint8_t get_mode() const override { return (uint8_t)control_mode->mode_number(); }  ///< Get current mode number
    Mode *mode_from_mode_num(const enum Mode::Number num);  ///< Get mode object from mode number
    bool current_mode_requires_mission() const override {
        return control_mode == &mode_auto;  ///< Check if current mode requires mission
    }

    bool autotuning;  ///< True when autotune is active
    /** @} */ // End of Control Mode Management group

    // ========================================================================
    // Failsafe Events (events.cpp)
    // ========================================================================
    /**
     * @name Failsafe Event Handlers
     * @brief Methods for handling failsafe triggers and recovery
     * @warning Safety-critical code - failsafe logic affects flight safety
     * @{
     */
    void failsafe_short_on_event(enum failsafe_state fstype, ModeReason reason);  ///< Trigger short failsafe (RC loss < threshold)
    void failsafe_long_on_event(enum failsafe_state fstype, ModeReason reason);  ///< Trigger long failsafe (RC loss > threshold)
    void failsafe_short_off_event(ModeReason reason);  ///< Clear short failsafe (RC recovered)
    void failsafe_long_off_event(ModeReason reason);  ///< Clear long failsafe (RC recovered)
    void handle_battery_failsafe(const char* type_str, const int8_t action);  ///< Handle battery failsafe trigger
    bool failsafe_in_landing_sequence() const;  ///< Check if in landing sequence (for failsafe decisions)
    /** @} */ // End of Failsafe Event Handlers group

#if AP_FENCE_ENABLED
    // ========================================================================
    // Geofence Management (fence.cpp)
    // ========================================================================
    /**
     * @name Geofence Management
     * @brief Methods for geofence breach detection and recovery
     * @{
     */
    void fence_check();  ///< Check for fence breaches
    void fence_checks_async() override;  ///< Async fence checks (from AP_Vehicle)
    bool fence_stickmixing() const;  ///< Check if stick mixing allowed during fence recovery
    bool in_fence_recovery() const;  ///< Check if in fence recovery mode
    uint8_t orig_breaches;  ///< Original fence breaches before recovery
    /** @} */ // End of Geofence Management group
#endif

    // ========================================================================
    // Main Loop and Scheduler Tasks (Plane.cpp)
    // ========================================================================
    /**
     * @name Main Loop and Periodic Tasks
     * @brief Core update methods called by scheduler at various rates
     * @details Methods organized by execution frequency:
     *          - 400Hz: AHRS update, control loops
     *          - 50Hz: GPS fast update, TECS
     *          - 10Hz: GPS position update, compass, sensors
     *          - 3Hz: Battery monitoring, arming checks
     *          - 1Hz: Logging, system status
     * @{
     */
    void disarm_if_autoland_complete();  ///< Disarm if autoland has completed
    bool trigger_land_abort(const float climb_to_alt_m);  ///< Abort landing and climb
    void get_osd_roll_pitch_rad(float &roll, float &pitch) const override;  ///< Get roll/pitch for OSD display
    float tecs_hgt_afe(void);  ///< Get TECS height above field elevation
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override;  ///< Get scheduler task table
    void ahrs_update();  ///< Update AHRS (400Hz)
    void update_speed_height(void);  ///< Update TECS speed/height controller (50Hz)
    void update_GPS_50Hz(void);  ///< Fast GPS update (50Hz)
    void update_GPS_10Hz(void);  ///< GPS position update (10Hz)
    void update_compass(void);  ///< Update compass (10Hz)
    void update_alt(void);  ///< Update altitude estimate (10Hz)
#if AP_ADVANCEDFAILSAFE_ENABLED
    void afs_fs_check(void);  ///< Advanced failsafe check
#endif
    void one_second_loop(void);  ///< 1Hz update tasks
    void three_hz_loop(void);  ///< 3Hz update tasks
#if AP_AIRSPEED_AUTOCAL_ENABLE
    void airspeed_ratio_update(void);  ///< Update airspeed sensor calibration ratio
#endif
    void update_logging10(void);  ///< 10Hz logging update
    void update_logging25(void);  ///< 25Hz logging update
    void update_control_mode(void);  ///< Update current control mode
    void update_fly_forward(void);  ///< Update fly-forward state
    void update_flight_stage();  ///< Update flight stage (takeoff/normal/land/abort)
    void set_flight_stage(AP_FixedWing::FlightStage fs);  ///< Set flight stage
    bool flight_option_enabled(FlightOptions flight_option) const;  ///< Check if flight option enabled
    /** @} */ // End of Main Loop and Periodic Tasks group

    // ========================================================================
    // Navigation Methods (navigation.cpp)
    // ========================================================================
    /**
     * @name Navigation and Waypoint Tracking
     * @brief Methods for waypoint navigation, loitering, and path following
     * @details Implements L1 controller updates, loiter management, and airspeed control
     * @{
     */
    void loiter_angle_reset(void);  ///< Reset loiter angle tracking
    void loiter_angle_update(void);  ///< Update loiter angle for circle counting
    void navigate();  ///< Main navigation update - calls L1 controller
    void check_home_alt_change(void);  ///< Check if home altitude has changed
    void calc_airspeed_errors();  ///< Calculate airspeed tracking errors
    float mode_auto_target_airspeed_cm();  ///< Get target airspeed for AUTO mode
    void calc_gndspeed_undershoot();  ///< Calculate ground speed undershoot
    void update_loiter(uint16_t radius);  ///< Update loiter circle navigation
    void update_loiter_update_nav(uint16_t radius);  ///< Update loiter and navigation
    void update_fbwb_speed_height(void);  ///< Update FBWB speed/height control
    void setup_turn_angle(void);  ///< Setup turn angle for next leg
    bool reached_loiter_target(void);  ///< Check if reached loiter target
    /** @} */ // End of Navigation group

    // ========================================================================
    // RC Radio Input/Output (radio.cpp)
    // ========================================================================
    /**
     * @name RC Radio Management
     * @brief Methods for RC input reading, channel mapping, and failsafe detection
     * @{
     */
    void set_control_channels(void) override;  ///< Map RC channels to control functions
    void init_rc_in();  ///< Initialize RC input channels
    void init_rc_out_main();  ///< Initialize main RC output channels
    void init_rc_out_aux();  ///< Initialize auxiliary RC output channels
    void read_radio();  ///< Read RC input values (called from main loop)
    int16_t rudder_input(void);  ///< Get rudder input from RC
    void control_failsafe();  ///< Check for RC failsafe condition
    void trim_radio();  ///< Auto-trim RC inputs
    bool rc_throttle_value_ok(void) const;  ///< Check if throttle value above failsafe threshold
    bool rc_failsafe_active(void) const;  ///< Check if RC failsafe is active
    /** @} */ // End of RC Radio Management group

#if AP_RANGEFINDER_ENABLED
    // ========================================================================
    // Sensor Reading (sensors.cpp)
    // ========================================================================
    /**
     * @name Sensor Reading
     * @brief Methods for reading sensor data
     * @{
     */
    void read_rangefinder(void);  ///< Read rangefinder distance sensors
    /** @} */ // End of Sensor Reading group
#endif

    // ========================================================================
    // System Initialization and Configuration (system.cpp)
    // ========================================================================
    /**
     * @name System Initialization
     * @brief System startup, mode changes, and core initialization
     * @{
     */
    __INITFUNC__ void init_ardupilot() override;  ///< Main initialization called at startup
    bool set_mode(Mode& new_mode, const ModeReason reason);  ///< Set flight mode by reference
    bool set_mode(const uint8_t mode, const ModeReason reason) override;  ///< Set mode by number
    bool set_mode_by_number(const Mode::Number new_mode_number, const ModeReason reason);  ///< Set mode by enum
    void check_long_failsafe();  ///< Check for long failsafe condition
    void check_short_failsafe();  ///< Check for short failsafe condition
    void startup_INS(void);  ///< Initialize inertial navigation system
    bool should_log(uint32_t mask);  ///< Check if logging enabled for mask
    int8_t throttle_percentage(void);  ///< Get current throttle as percentage
    void notify_mode(const Mode& mode);  ///< Notify GCS/LED of mode change
    bool gcs_mode_enabled(const Mode::Number mode_num) const;  ///< Check if mode enabled via GCS
    /** @} */ // End of System Initialization group

    // ========================================================================
    // Takeoff Management (takeoff.cpp)
    // ========================================================================
    /**
     * @name Takeoff Control
     * @brief Methods for managing takeoff sequence and ground operations
     * @{
     */
    bool auto_takeoff_check(void);  ///< Check if auto takeoff conditions met
    void takeoff_calc_roll(void);  ///< Calculate roll limits during takeoff
    void takeoff_calc_pitch(void);  ///< Calculate pitch angle during takeoff
    void takeoff_calc_throttle();  ///< Calculate throttle during takeoff
    int8_t takeoff_tail_hold(void);  ///< Tail hold logic for tail-dragger takeoff
    int16_t get_takeoff_pitch_min_cd(void);  ///< Get minimum pitch for takeoff (centidegrees)
    void landing_gear_update(void);  ///< Update landing gear state
    bool check_takeoff_timeout(void);  ///< Check if takeoff has timed out
    bool check_takeoff_timeout_level_off(void);  ///< Check if time to level off during takeoff
    /** @} */ // End of Takeoff Control group

    // ========================================================================
    // ADSB Avoidance (avoidance_adsb.cpp)
    // ========================================================================
    /**
     * @name ADSB Avoidance
     * @brief Methods for avoiding manned aircraft via ADSB
     * @{
     */
    void avoidance_adsb_update(void);  ///< Update ADSB avoidance logic
    /** @} */ // End of ADSB Avoidance group

    // ========================================================================
    // Servo and Actuator Control (servos.cpp)
    // ========================================================================
    /**
     * @name Servo and Actuator Control
     * @brief Methods for controlling servos, motors, flaps, and other actuators
     * @details Implements servo mixing, throttle management, and control surface coordination
     * @{
     */
    void set_servos();  ///< Calculate and set all servo outputs
    float apply_throttle_limits(float throttle_in);  ///< Apply throttle limits and constraints
    void set_throttle(void);  ///< Set throttle output
    void set_takeoff_expected(void);  ///< Notify ESCs that takeoff is expected
    void set_servos_flaps(void);  ///< Set flap servo positions
    void dspoiler_update(void);  ///< Update differential spoilers
    void airbrake_update(void);  ///< Update airbrake position
    void landing_neutral_control_surface_servos(void);  ///< Neutral servos during landing
    void servos_output(void);  ///< Output servo values to hardware
    void servos_auto_trim(void);  ///< Auto-trim servos in flight
    void servos_twin_engine_mix();  ///< Mix twin engine differential thrust
    void force_flare();  ///< Force landing flare
    void throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle);  ///< Limit throttle by power consumption
    void throttle_slew_limit();  ///< Apply throttle slew rate limiting
    bool suppress_throttle(void);  ///< Check if throttle should be suppressed
    void update_throttle_hover();  ///< Update hover throttle estimate
    void channel_function_mixer(SRV_Channel::Function func1_in, SRV_Channel::Function func2_in,
                                SRV_Channel::Function func1_out, SRV_Channel::Function func2_out) const;  ///< Mix two channel functions
    void flaperon_update();  ///< Update flaperon mixing
    void indicate_waiting_for_rud_neutral_to_takeoff(void);  ///< Indicate waiting for rudder neutral
    /** @} */ // End of Servo and Actuator Control group

    // ========================================================================
    // Flight State Detection (is_flying.cpp)
    // ========================================================================
    /**
     * @name Flight State Detection
     * @brief Methods for detecting if aircraft is flying and crash detection
     * @{
     */
    void update_is_flying_5Hz(void);  ///< Update is_flying state (5Hz)
    void crash_detection_update(void);  ///< Update crash detection logic
    bool in_preLaunch_flight_stage(void);  ///< Check if in pre-launch stage
    bool is_flying(void);  ///< Check if aircraft is currently flying
    /** @} */ // End of Flight State Detection group

    // ========================================================================
    // Parachute Management (parachute.cpp)
    // ========================================================================
    /**
     * @name Parachute Control
     * @brief Methods for parachute deployment
     * @{
     */
    void parachute_check();  ///< Check parachute deployment conditions
#if HAL_PARACHUTE_ENABLED
    void parachute_release();  ///< Release parachute
    bool parachute_manual_release();  ///< Manual parachute release via RC
#endif
    /** @} */ // End of Parachute Control group

    // ========================================================================
    // Soaring Support (soaring.cpp)
    // ========================================================================
#if HAL_SOARING_ENABLED
    /**
     * @name Soaring Mode
     * @brief Methods for autonomous soaring and thermal detection
     * @{
     */
    void update_soaring();  ///< Update soaring mode logic
    /** @} */ // End of Soaring Mode group
#endif

    // ========================================================================
    // RC Channel State (RC_Channel.cpp)
    // ========================================================================
    /**
     * @name RC Channel State Variables
     * @{
     */
    bool emergency_landing;  ///< Emergency landing flag from RC channel
    /** @} */ // End of RC Channel State group

    // ========================================================================
    // Waypoint Information Helpers (vehicle-specific)
    // ========================================================================
    /**
     * @name Waypoint Information
     * @brief Vehicle-specific waypoint information helpers for AP_Vehicle interface
     * @{
     */
    bool get_wp_distance_m(float &distance) const override;  ///< Get distance to next waypoint in meters
    bool get_wp_bearing_deg(float &bearing) const override;  ///< Get bearing to next waypoint in degrees
    bool get_wp_crosstrack_error_m(float &xtrack_error) const override;  ///< Get crosstrack error in meters
    /** @} */ // End of Waypoint Information group

    // ========================================================================
    // Reverse Thrust Support (reverse_thrust.cpp)
    // ========================================================================
    /**
     * @name Reverse Thrust
     * @brief Methods and state for reverse thrust (braking) control
     * @{
     */
    bool reversed_throttle;  ///< True if reverse thrust is currently active
    bool have_reverse_throttle_rc_option;  ///< True if reverse thrust RC option configured
    bool allow_reverse_thrust(void) const;  ///< Check if reverse thrust allowed in current mode
    bool have_reverse_thrust(void) const;  ///< Check if vehicle has reverse thrust capability
    float get_throttle_input(bool no_deadzone=false) const;  ///< Get throttle input (may be negative for reverse)
    float get_adjusted_throttle_input(bool no_deadzone=false) const;  ///< Get adjusted throttle input
    bool reverse_thrust_enabled(UseReverseThrust use_reverse_thrust_option) const;  ///< Check if reverse thrust enabled
    /** @} */ // End of Reverse Thrust group

#if AP_SCRIPTING_ENABLED
    // ========================================================================
    // Scripting Support (AP_Scripting integration)
    // ========================================================================
    /**
     * @name Lua Scripting Interface
     * @brief Methods for Lua script control and NAV_SCRIPT_TIME mission command
     * @{
     */
    bool nav_scripting_active(void);  ///< Check if nav scripting is active
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4) override;  ///< Get nav script command
    void nav_script_time_done(uint16_t id) override;  ///< Signal nav script command complete

    /**
     * @brief Command throttle percentage and roll, pitch, yaw target rates
     * @details For use with scripting controllers
     */
    void set_target_throttle_rate_rpy(float throttle_pct, float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps) override;
    void set_rudder_offset(float rudder_pct, bool run_yaw_rate_controller) override;  ///< Set rudder offset from script
    bool nav_scripting_enable(uint8_t mode) override;  ///< Enable/disable nav scripting
    /** @} */ // End of Lua Scripting Interface group
#endif
 
    // ========================================================================
    // Failsafe Action Definitions
    // ========================================================================
    /**
     * @enum Failsafe_Action
     * @brief Failsafe action types available to the plane
     * @details Defines possible responses to failsafe conditions
     */
    enum Failsafe_Action {
        Failsafe_Action_None      = 0,  ///< No action - continue current mode
        Failsafe_Action_RTL       = 1,  ///< Return to launch
        Failsafe_Action_Land      = 2,  ///< Land immediately
        Failsafe_Action_Terminate = 3,  ///< Terminate flight (cut motors)
#if HAL_QUADPLANE_ENABLED
        Failsafe_Action_QLand     = 4,  ///< QuadPlane vertical land
#endif
        Failsafe_Action_Parachute = 5,  ///< Deploy parachute
#if HAL_QUADPLANE_ENABLED
        Failsafe_Action_Loiter_alt_QLand = 6,  ///< Loiter to altitude then QLAND
#endif
        Failsafe_Action_AUTOLAND_OR_RTL = 7,  ///< Auto-land if in mission, otherwise RTL
    };

    /**
     * @brief Failsafe action priority list (highest priority first)
     * @details List of priorities for resolving multiple simultaneous failsafes
     * @note Sentinel value -1 must be last element
     */
    static constexpr int8_t _failsafe_priorities[] = {
                                                      Failsafe_Action_Terminate,
                                                      Failsafe_Action_Parachute,
#if HAL_QUADPLANE_ENABLED
                                                      Failsafe_Action_QLand,
#endif
                                                      Failsafe_Action_Land,
                                                      Failsafe_Action_RTL,
                                                      Failsafe_Action_None,
                                                      -1 // the priority list must end with a sentinel of -1
                                                     };
    static_assert(_failsafe_priorities[ARRAY_SIZE(_failsafe_priorities) - 1] == -1,
                  "_failsafe_priorities is missing the sentinel");

    // ========================================================================
    // EKF Failsafe Checks (ekf_check.cpp)
    // ========================================================================
    /**
     * @name EKF Failsafe
     * @brief EKF health monitoring and failsafe triggering
     * @details Monitors Extended Kalman Filter health, primarily for VTOL operation
     * @{
     */
    void ekf_check();  ///< Check EKF health status
    bool ekf_over_threshold();  ///< Check if EKF variance exceeds threshold
    void failsafe_ekf_event();  ///< Trigger EKF failsafe
    void failsafe_ekf_off_event(void);  ///< Clear EKF failsafe
    /** @} */ // End of EKF Failsafe group

    // ========================================================================
    // Configuration Enumerations
    // ========================================================================
    /**
     * @enum CrowMode
     * @brief Crow flap/spoiler configuration mode
     */
    enum class CrowMode {
        NORMAL,       ///< Normal flap operation
        PROGRESSIVE,  ///< Progressive crow
        CROW_DISABLED,  ///< Crow mode disabled
    };

    using ThrFailsafe = Parameters::ThrFailsafe;  ///< Alias for throttle failsafe enum

    CrowMode crow_mode = CrowMode::NORMAL;  ///< Current crow mode configuration

    /**
     * @enum FlareMode
     * @brief Landing flare configuration
     */
    enum class FlareMode {
        FLARE_DISABLED = 0,  ///< Flare disabled
        ENABLED_NO_PITCH_TARGET,  ///< Flare enabled without pitch target
        ENABLED_PITCH_TARGET  ///< Flare enabled with pitch target
    };
    
    /**
     * @enum AutoTuneAxis
     * @brief Axes available for auto-tuning (bitmask)
     */
    enum class AutoTuneAxis {
        ROLL  = 1U <<0,  ///< Roll axis tuning
        PITCH = 1U <<1,  ///< Pitch axis tuning
        YAW   = 1U <<2,  ///< Yaw axis tuning
    };

    FlareMode flare_mode;  ///< Current landing flare mode
    bool throttle_at_zero(void) const;  ///< Check if throttle is at zero

    // ========================================================================
    // RC Input Expo Handling
    // ========================================================================
    /**
     * @name RC Input Expo
     * @brief Apply exponential curves to RC inputs
     * @{
     */
    float roll_in_expo(bool use_dz) const;  ///< Apply expo to roll input
    float pitch_in_expo(bool use_dz) const;  ///< Apply expo to pitch input
    float rudder_in_expo(bool use_dz) const;  ///< Apply expo to rudder input
    /** @} */ // End of RC Input Expo group

    // ========================================================================
    // Mode State Tracking
    // ========================================================================
    ModeReason previous_mode_reason = ModeReason::UNKNOWN;  ///< Reason for entering previous mode

    // ========================================================================
    // TECS State Tracking
    // ========================================================================
    int32_t tecs_target_alt_cm;  ///< Last target altitude passed to TECS (centimeters)

public:
    // ========================================================================
    // Public Interface Methods
    // ========================================================================
    /**
     * @name Public Vehicle Interface
     * @brief Public methods accessible to external systems (scripting, external control)
     * @{
     */
    
    /**
     * @brief Check and handle all failsafe conditions
     * @details Main entry point for failsafe monitoring, called from main loop
     */
    void failsafe_check(void);
    
    /**
     * @brief Check if vehicle is currently landing
     * @return true if in landing phase
     */
    bool is_landing() const override;
    
    /**
     * @brief Check if vehicle is currently taking off
     * @return true if in takeoff phase
     */
    bool is_taking_off() const override;
    
#if AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED
    /**
     * @brief Set guided mode target location
     * @param target_loc Target location (lat/lon/alt)
     * @return true if target accepted
     * @note Available for scripting and external control
     */
    bool set_target_location(const Location& target_loc) override;
#endif //AP_SCRIPTING_ENABLED || AP_EXTERNAL_CONTROL_ENABLED

#if AP_SCRIPTING_ENABLED
    /**
     * @brief Get current guided mode target location
     * @param[out] target_loc Current target location
     * @return true if target available
     */
    bool get_target_location(Location& target_loc) override;
    
    /**
     * @brief Update guided mode target location (for scripting)
     * @param old_loc Expected current target location
     * @param new_loc New target location
     * @return true if update successful
     */
    bool update_target_location(const Location &old_loc, const Location &new_loc) override;
    
    /**
     * @brief Set velocity matching target for formation flying
     * @param velocity Target velocity vector (m/s, NE frame)
     * @return true if velocity target accepted
     */
    bool set_velocity_match(const Vector2f &velocity) override;

    /**
     * @brief Override landing descent rate from script
     * @param descent_rate Descent rate in m/s (negative to climb)
     * @return true if descent rate override accepted
     * @note Allows scripting to control landing sink rate
     */
    bool set_land_descent_rate(float descent_rate) override;

    /**
     * @brief Override mission/guided crosstrack start location
     * @param new_start_location New crosstrack start location
     * @return true if crosstrack start accepted
     * @warning Script must ensure provided location makes sense for current path
     */
    bool set_crosstrack_start(const Location &new_start_location) override;

#endif // AP_SCRIPTING_ENABLED

    /**
     * @brief Check if takeoff option flag is set
     * @param option Takeoff option to check
     * @return true if option is enabled in aparm.takeoff_options
     */
    bool tkoff_option_is_set(AP_FixedWing::TakeoffOption option) const {
        return (aparm.takeoff_options & int32_t(option)) != 0;
    }
    
    /** @} */ // End of Public Vehicle Interface group
   

};

extern Plane plane;

using AP_HAL::millis;
using AP_HAL::micros;
