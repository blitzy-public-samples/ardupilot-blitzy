/**
 * @file mode_guided_nogps.cpp
 * @brief Guided flight mode for GPS-denied environments
 * 
 * @details This flight mode implements guided control without GPS positioning,
 *          enabling autonomous flight in GPS-denied environments such as indoor
 *          spaces, under bridges, or in GPS-jammed scenarios. The mode relies on
 *          alternative position sources including:
 *          - Optical flow sensors for velocity estimation
 *          - Visual odometry systems for position tracking
 *          - External positioning systems (motion capture, UWB beacons)
 *          - Companion computer-provided position estimates via MAVLink
 * 
 *          This mode extends the standard Guided mode functionality but operates
 *          using non-GPS position sources integrated through the EKF. Velocity
 *          and attitude control algorithms remain the same as standard Guided mode,
 *          but position estimates come from alternative sensors rather than GPS.
 * 
 *          Typical use cases:
 *          - Indoor flight with optical flow and rangefinder
 *          - GPS-jammed outdoor environments with visual odometry
 *          - Research applications with external tracking systems
 *          - Warehouse or indoor inspection operations
 * 
 * @note This mode requires properly configured alternative position sources
 *       and EKF fusion of non-GPS positioning data. The EKF must be operating
 *       in a GPS-denied mode with valid alternative position estimates.
 * 
 * @warning Without valid position estimates from alternative sources, the vehicle
 *          will not maintain position hold and may drift. Always verify position
 *          source health before engaging this mode.
 * 
 * @see ModeGuided for the GPS-enabled guided mode implementation
 * @see AP_AHRS and AP_NavEKF3 for position source fusion
 * 
 * Source: ArduCopter/mode_guided_nogps.cpp
 */

#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 * 
 * This mode variant provides the same guided control interface as standard
 * Guided mode but operates without GPS. Position commands and velocity commands
 * are processed using alternative position sources fused in the EKF.
 */

/**
 * @brief Initialize the Guided NoGPS flight mode
 * 
 * @details Initializes the GPS-denied guided flight mode by setting up angle
 *          control as the default control method. This mode inherits most of
 *          its functionality from the standard ModeGuided class but is designed
 *          to operate without GPS position fixes.
 * 
 *          The initialization process:
 *          1. Invokes the parent ModeGuided angle control initialization
 *          2. Sets up attitude control targets for GPS-free operation
 *          3. Prepares to receive position/velocity commands from alternative sources
 * 
 *          Alternative position sources (optical flow, visual odometry, external
 *          positioning systems) must already be configured and providing valid
 *          data to the EKF before entering this mode. The mode itself does not
 *          configure or validate these sources - that responsibility lies with
 *          the EKF and pre-arm checks.
 * 
 * @param[in] ignore_checks If true, bypasses standard mode entry checks such as
 *                          position estimate validity. Should only be set true for
 *                          emergency mode changes or testing. Normal operation
 *                          should use false to ensure safe mode entry.
 * 
 * @return true if initialization successful (currently always returns true as
 *         angle control initialization cannot fail)
 * 
 * @note This mode starts in angle control, which maintains attitude but does not
 *       actively hold position. Position hold requires receiving position/velocity
 *       commands from a ground station or companion computer.
 * 
 * @note The mode does not verify that alternative position sources are available
 *       or healthy - this must be confirmed before mode entry through EKF checks
 * 
 * @warning Entering this mode without valid alternative position sources will
 *          result in attitude-only control with no position hold capability.
 *          The vehicle will drift with wind or momentum.
 * 
 * @see ModeGuided::angle_control_start() for the underlying initialization
 * @see Copter::mode_enter() for mode entry logic and checks
 */
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Start in angle control mode - maintains attitude setpoints but does not
    // actively control position. Position control is enabled when guided commands
    // are received from external sources (GCS or companion computer).
    // Alternative position sources (optical flow, visual odometry, or external
    // positioning) provide the position estimates needed for position control.
    ModeGuided::angle_control_start();
    return true;
}

/**
 * @brief Main control loop for Guided NoGPS mode
 * 
 * @details Executes the primary control loop for GPS-denied guided flight,
 *          running the angle controller to maintain commanded attitudes and
 *          process velocity/position commands based on alternative position sources.
 * 
 *          This function is called repeatedly at the main loop rate (typically
 *          400Hz) to maintain stable flight control. It delegates to the parent
 *          ModeGuided angle control implementation, which processes:
 *          - Attitude setpoints from guided commands
 *          - Velocity targets from companion computer or GCS
 *          - Position targets computed using non-GPS position estimates
 * 
 *          Position and velocity control in this mode relies on:
 *          - EKF position estimates from optical flow sensors
 *          - Visual odometry position estimates from camera systems
 *          - External positioning data from motion capture or UWB systems
 *          - Companion computer-provided position estimates via MAVLink
 * 
 *          The control hierarchy operates as:
 *          Position commands → Velocity commands → Attitude commands → Motor outputs
 * 
 *          Each level of the control hierarchy uses the same algorithms as GPS-based
 *          guided mode, but position/velocity estimates come from alternative sources
 *          rather than GPS. This ensures consistent control behavior regardless of
 *          positioning source.
 * 
 * @return void
 * 
 * @note Called at main loop rate (typically 400Hz for multicopters). Timing is
 *       critical for stable attitude control - do not add computationally expensive
 *       operations to this loop.
 * 
 * @note Position control accuracy depends entirely on the quality of alternative
 *       position sources. Optical flow requires good lighting and texture, visual
 *       odometry requires feature-rich environments, external positioning requires
 *       line-of-sight to beacons.
 * 
 * @warning If alternative position sources become invalid during flight (e.g.,
 *          optical flow loses tracking, visual odometry loses features), the EKF
 *          will report position estimate degradation and failsafes may trigger.
 *          Always monitor EKF status during GPS-denied operations.
 * 
 * @warning This mode provides no special handling for position source failures.
 *          Standard EKF failsafes and position estimate monitoring apply. Configure
 *          appropriate failsafe actions for GPS-denied operations.
 * 
 * @see ModeGuided::angle_control_run() for the underlying control implementation
 * @see AC_AttitudeControl for attitude control details
 * @see AC_PosControl for position/velocity control details
 * @see AP_NavEKF3 for position source fusion and health monitoring
 */
void ModeGuidedNoGPS::run()
{
    // Execute angle control loop using position/velocity estimates from alternative
    // sources (optical flow, visual odometry, external positioning, etc.).
    // The angle controller processes guided commands and converts them through the
    // position → velocity → attitude control hierarchy using non-GPS position data.
    ModeGuided::angle_control_run();
}

#endif
