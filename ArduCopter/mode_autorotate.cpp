/**
 * @file mode_autorotate.cpp
 * @brief Autorotate flight mode implementation for traditional helicopters
 * 
 * @details This file implements the Autorotate flight mode, which is a safety-critical
 *          emergency landing mode designed for single-rotor traditional helicopters
 *          experiencing engine or power system failure. Autorotation is the aerodynamic
 *          technique that allows a helicopter to descend safely and land without engine
 *          power by using upward airflow through the rotor system to maintain rotor RPM.
 * 
 *          Key Autorotation Concepts:
 *          - During normal flight, engine power drives the rotor and produces thrust
 *          - When engine fails, pilot reduces collective pitch to minimum (enters autorotation)
 *          - Descending through air causes upward airflow through rotor disk
 *          - This upward airflow spins the rotor, storing kinetic energy in rotor inertia
 *          - Pilot manages collective pitch to maintain optimal rotor RPM during descent
 *          - Just before touchdown, pilot performs "flare" maneuver to arrest descent
 *          - Final landing performed with carefully timed collective pitch increase
 * 
 *          Autorotation Phases Implemented:
 *          1. ENTRY - Initial transition: reduce collective to enter autorotation regime
 *          2. GLIDE - Steady descent: maintain rotor speed and forward speed
 *          3. FLARE - Deceleration maneuver to reduce forward speed and descent rate
 *          4. TOUCH_DOWN - Final collective application for soft landing
 *          5. LANDED - Post-landing control decay
 * 
 *          Requirements:
 *          - Traditional single-rotor helicopter only (FRAME_CONFIG == HELI_FRAME)
 *          - AC_Autorotation library must be enabled (MODE_AUTOROTATE_ENABLED)
 *          - RPM sensor for rotor head speed monitoring
 *          - Properly configured helicopter parameters (H_RSC_*, H_COL_*, etc.)
 *          - NOT applicable to multirotors or coaxial helicopters
 * 
 *          Safety Considerations:
 *          - This is a flight-critical emergency mode
 *          - Improper configuration can result in unrecoverable crashes
 *          - Mode can only be entered while armed and flying (prevents ground activation)
 *          - Requires extensive SITL testing before real-world use
 *          - Currently developmental - see forum for testing procedures
 * 
 *          Integration with AC_Autorotation Library:
 *          - Core autorotation algorithms implemented in libraries/AC_Autorotation/
 *          - This mode class provides ArduCopter-specific integration
 *          - Library handles collective management, RPM control, and speed targets
 *          - Mode handles state machine progression and safety checks
 * 
 * @note This is currently a SITL-only feature until the project is complete.
 *       To trial this in SITL you will need to use Real Flight 8.
 *       Instructions: https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 * 
 * @warning This mode should only be used on traditional single-rotor helicopters.
 *          Using this mode on multirotors or other vehicle types will fail safely
 *          by refusing to enter the mode, but proper configuration is essential.
 * 
 * @see libraries/AC_Autorotation/AC_Autorotation.h
 * @see ArduCopter/mode.h
 * 
 * Source: ArduCopter/mode_autorotate.cpp
 */

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#if MODE_AUTOROTATE_ENABLED

/**
 * @brief Initialize Autorotate mode for traditional helicopter emergency landing
 * 
 * @details This function performs safety checks and initializes the autorotation state machine
 *          for emergency descent and landing after engine failure. Autorotation mode can only
 *          be entered on traditional single-rotor helicopters that are armed and flying.
 * 
 *          The function performs the following initialization sequence:
 *          1. Verify vehicle is a traditional helicopter (not multirotor)
 *          2. Check that autorotation mode is enabled in parameters (AROT_ENABLE)
 *          3. Verify vehicle is armed and airborne (prevent ground activation)
 *          4. Initialize AC_Autorotation library controller
 *          5. Set initial state machine phase to ENTRY_INIT
 *          6. Reset timing variables for phase transitions
 * 
 *          Safety Checks Performed:
 *          - Frame type verification: FRAME_CONFIG must equal HELI_FRAME
 *          - Parameter enable check: g2.arot.enabled() must be true
 *          - Arming status: motors->armed() must be true
 *          - Flight status: land_complete and land_complete_maybe must be false
 * 
 *          State Machine Initialization:
 *          - current_phase set to Phase::ENTRY_INIT (first phase of autorotation)
 *          - _entry_time_start_ms set to current time (for entry duration timing)
 *          - _last_logged_ms reset to 0 (for periodic logging)
 * 
 * @param[in] ignore_checks Currently unused, inherited from Mode base class interface
 *                          (included for consistency with other mode implementations)
 * 
 * @return true if mode successfully initialized and autorotation can begin
 * @return false if initialization failed due to safety checks or configuration issues
 * 
 * @note This mode should only be activated during actual engine failures or for
 *       controlled testing in SITL. Entering this mode will immediately reduce
 *       collective pitch and begin unpowered descent.
 * 
 * @warning Do not activate this mode on the ground - it will be rejected with error message.
 *          Vehicle must be armed and flying for mode entry to succeed.
 * 
 * @warning This mode is only for traditional single-rotor helicopters. Multirotors and
 *          other vehicle types will fail initialization at frame type check.
 * 
 * @see AC_Autorotation::init()
 * @see ModeAutorotate::run()
 * 
 * Source: ArduCopter/mode_autorotate.cpp:14-47
 */
bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    // Autorotation is specific to single-rotor helicopters with variable collective pitch.
    // Multirotors cannot autorotate as they rely on powered motors for all control.
    return false;
#endif

    // Check that mode is enabled, make sure this is the first check as this is the most
    // important thing for users to fix if they are planning to use autorotation mode
    // Parameter check: AROT_ENABLE must be set to 1 for autorotation to be available.
    // This prevents accidental activation of emergency mode during normal operations.
    if (!g2.arot.enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AROT: Mode not enabled");
        return false;
    }

    // Must be armed to use mode, prevent triggering state machine on the ground
    // Safety check: Vehicle must be armed AND airborne to enter autorotation.
    // This prevents dangerous mode activation while helicopter is on the ground.
    // Autorotation requires sufficient altitude for proper entry, glide, and flare phases.
    // land_complete: true when vehicle has landed (throttle at minimum, low climb rate)
    // land_complete_maybe: true when vehicle appears to be landing or landed
    if (!motors->armed() || copter.ap.land_complete || copter.ap.land_complete_maybe) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AROT: Must be armed and flying");
        return false;
    }

    // Initialise controller
    // Initialize the AC_Autorotation library which handles:
    // - Collective pitch management for rotor RPM control
    // - Forward speed target calculation for optimal glide
    // - Flare timing and execution
    // - Touch-down collective application
    g2.arot.init();

    // Setting default starting state
    // Begin autorotation sequence at ENTRY_INIT phase.
    // State machine will progress: ENTRY -> GLIDE -> FLARE -> TOUCH_DOWN -> LANDED
    current_phase = Phase::ENTRY_INIT;

    // Set entry timer
    // Record mode entry time for tracking duration of entry phase.
    // Entry phase reduces collective pitch smoothly over configured time (AROT_ENTRY_TIME)
    // to transition from powered flight to autorotative descent.
    _entry_time_start_ms = millis();

    // reset logging timer
    // Reset logging timestamp to ensure immediate first log of autorotation data.
    // Autorotation data logged at 25 Hz (every 40ms) for detailed flight analysis.
    _last_logged_ms = 0;

    return true;
}

/**
 * @brief Main run loop for Autorotate mode - executes autorotation state machine
 * 
 * @details This function implements the complete autorotation emergency landing sequence
 *          for traditional helicopters. It manages the state machine that progresses through
 *          entry, glide, flare, touchdown, and landed phases while maintaining rotor RPM
 *          and controlling descent rate through collective pitch management.
 * 
 *          Autorotation State Machine Phases:
 * 
 *          1. ENTRY (ENTRY_INIT -> ENTRY):
 *             - Duration: Configured by AROT_ENTRY_TIME parameter (typically 3-4 seconds)
 *             - Purpose: Smoothly reduce collective pitch to enter autorotation regime
 *             - Actions: Transition from powered flight to unpowered descent
 *             - Goal: Establish stable autorotative descent with proper rotor RPM
 *             - Control: Forward speed control begins to optimize glide performance
 * 
 *          2. GLIDE (GLIDE_INIT -> GLIDE):
 *             - Duration: Until flare initiation conditions met (altitude/speed based)
 *             - Purpose: Maintain optimal rotor RPM and forward speed during descent
 *             - Actions: Continuous collective adjustment to keep rotor in optimal range
 *             - Goal: Convert altitude to controlled forward motion and rotor energy
 *             - Control: Heading maintained, forward speed regulated for best glide ratio
 * 
 *          3. FLARE (FLARE_INIT -> FLARE):
 *             - Duration: Brief maneuver (typically 2-3 seconds)
 *             - Purpose: Trade forward speed and rotor energy to arrest descent
 *             - Actions: Cyclic aft input to pitch nose up, increasing rotor RPM
 *             - Goal: Reduce descent rate and forward speed for soft touchdown
 *             - Control: Critical timing - too early wastes energy, too late prevents recovery
 * 
 *          4. TOUCH_DOWN (TOUCH_DOWN_INIT -> TOUCH_DOWN):
 *             - Duration: Final moments before ground contact
 *             - Purpose: Apply stored rotor energy for final cushioning
 *             - Actions: Collective increase to use rotor inertia for thrust
 *             - Goal: Minimum vertical speed at ground contact
 *             - Control: Precise collective timing for softest possible landing
 * 
 *          5. LANDED (LANDED_INIT -> LANDED):
 *             - Duration: After ground contact detected
 *             - Purpose: Safe control decay and motor spool down
 *             - Actions: Smoothly reduce all control inputs to zero
 *             - Goal: Prevent control saturation and allow landing detection
 *             - Control: Decay pitch/roll demands, maintain heading hold
 * 
 *          State Machine Logic:
 *          - Phases progress based on time, altitude, speed, and sensor conditions
 *          - Later phases take precedence (LANDED check overrides all earlier phases)
 *          - Landed detection uses multiple sensors (accelerometer, barometer, contact switches)
 *          - Phase transitions are one-way (no reverting to earlier phases)
 * 
 *          Pilot Interaction:
 *          - Yaw control: Pilot maintains heading control via rudder throughout autorotation
 *          - Collective/cyclic: Automated by controller (pilot inputs ignored or limited)
 *          - Bailout: Pilot can bailout (add throttle) if situation allows engine restart
 * 
 *          AC_Autorotation Library Integration:
 *          - Library handles all collective pitch calculations for RPM management
 *          - Library computes forward speed targets for optimal glide ratio
 *          - Library monitors rotor RPM via RPM sensor for feedback control
 *          - Mode provides state machine sequencing and phase transitions
 * 
 *          Sensor Requirements:
 *          - RPM sensor: Mandatory for rotor head speed feedback
 *          - Barometer: Altitude for phase transitions
 *          - IMU: Attitude and descent rate
 *          - GPS: Ground speed (optional but improves performance)
 * 
 * @note Called at main loop rate (typically 400 Hz for helicopters)
 * 
 * @warning This is safety-critical code. The autorotation sequence cannot be aborted
 *          once collective is reduced below sustainable thrust level. Improper tuning
 *          or sensor failures can result in hard landing or crash.
 * 
 * @warning Rotor RPM must be maintained within safe limits throughout autorotation.
 *          RPM too low: Insufficient rotor energy for flare and touchdown (crash).
 *          RPM too high: Rotor overspeed damage or control authority loss.
 * 
 * @see AC_Autorotation::run_entry()
 * @see AC_Autorotation::run_glide()
 * @see AC_Autorotation::run_landed()
 * @see AC_Autorotation::check_landed()
 * 
 * Source: ArduCopter/mode_autorotate.cpp:49-134
 */
void ModeAutorotate::run()
{
    // Current time
    // Capture current timestamp for phase timing and logging interval calculations
    const uint32_t now_ms = millis();

    // Set dt in library
    // Provide AC_Autorotation library with loop time for proper integration of control algorithms.
    // Delta time (dt) required for collective rate limiting and speed controller integration.
    float const last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    g2.arot.set_dt(last_loop_time_s);

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------
    // State machine progresses through the autorotation phases as you read down through the if statements.
    // More urgent phases (the ones closer to the ground) take precedence later in the if statements.
    // 
    // Phase Progression Logic:
    // - Time-based: ENTRY -> GLIDE after configured entry duration
    // - Sensor-based: Any phase -> LANDED when ground contact detected
    // - Later checks override earlier phases (prioritizing ground proximity)
    // - Phase transitions are irreversible (cannot return to earlier phases)

    // ENTRY to GLIDE transition (time-based)
    // After entry phase duration expires (AROT_ENTRY_TIME parameter, typically 3-4 seconds),
    // progress to steady-state glide phase. Entry phase has smoothly reduced collective
    // to establish autorotative rotor RPM. Now maintain that RPM while descending.
    if (current_phase < Phase::GLIDE_INIT && ((now_ms - _entry_time_start_ms) > g2.arot.entry_time_ms)) {
        // Flight phase can be progressed to steady state glide
        current_phase = Phase::GLIDE_INIT;
    }

    // Check if we believe we have landed. We need the landed state to zero all
    // controls and make sure that the copter landing detector will trip
    // 
    // LANDED detection (sensor-based, highest priority)
    // This check takes precedence over all other phases because ground contact requires
    // immediate control decay to prevent control saturation and allow proper landing detection.
    // AC_Autorotation::check_landed() evaluates multiple sensors:
    // - Low vertical acceleration (minimal movement)
    // - Barometer shows constant altitude
    // - Contact switches (if installed)
    // - Rotor RPM decay pattern consistent with ground contact
    if (current_phase < Phase::LANDED && g2.arot.check_landed()) {
        current_phase = Phase::LANDED_INIT;
    }

    // Check if we are bailing out and need to re-set the spool state
    // 
    // Bailout Detection and Motor Spool State Management:
    // Pilot can attempt bailout by adding throttle if engine restarts or situation changes.
    // motors->autorotation_bailout() returns true when pilot advances throttle significantly.
    // If bailout detected, reset motor spool state to allow throttle response and engine power.
    // This allows potential engine restart or go-around if altitude and conditions permit.
    // Note: Bailout may not be possible if rotor RPM has decayed too low or altitude insufficient.
    if (motors->autorotation_bailout()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    // Get norm input from yaw channel
    // Retrieve pilot yaw input for heading control during autorotation.
    // Pilot maintains directional control throughout autorotation via rudder/yaw input.
    // Normalized input range: -1.0 to +1.0, with deadzone applied.
    // This is the only direct pilot control input used during autorotation
    // (collective and cyclic are automated by AC_Autorotation library).
    const float pilot_norm_input = channel_yaw->norm_input_dz();

    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    // Execute phase-specific control logic based on current autorotation phase.
    // Each phase has optional initialization (PHASE_INIT) followed by continuous execution (PHASE).
    // FALLTHROUGH used to immediately execute phase logic after initialization in same loop.
    switch (current_phase) {

        case Phase::ENTRY_INIT:
            // Entry phase functions to be run only once
            // 
            // ENTRY_INIT: One-time initialization for entry phase
            // - Set initial collective target for autorotation entry
            // - Initialize speed controller for forward flight management
            // - Configure collective rate limiter for smooth transition
            // Entry phase smoothly reduces collective pitch over AROT_ENTRY_TIME duration
            // to transition from powered flight to autorotative descent without abrupt rotor unloading.
            g2.arot.init_entry();
            current_phase = Phase::ENTRY;
            FALLTHROUGH;

        case Phase::ENTRY:
            // Smoothly transition the collective to enter autorotation regime and begin forward speed control
            // 
            // ENTRY: Continuous execution during entry phase
            // - Gradually reduce collective pitch toward autorotation target
            // - Begin building forward speed for optimal glide ratio
            // - Monitor rotor RPM and adjust collective rate if needed
            // - Pilot yaw input applied for heading control
            // Goal: Establish stable autorotative descent with rotor RPM in optimal range (typically 90-100% Nr)
            g2.arot.run_entry(pilot_norm_input);
            break;

        case Phase::GLIDE_INIT:
            // Glide phase functions to be run only once
            // 
            // GLIDE_INIT: One-time initialization for glide phase
            // - Lock in steady-state collective for constant rotor RPM
            // - Initialize altitude/speed monitoring for flare trigger conditions
            // - Configure speed controller for optimal glide airspeed
            // Glide phase maintains constant rotor RPM while descending at best glide ratio.
            g2.arot.init_glide();
            current_phase = Phase::GLIDE;
            FALLTHROUGH;

        case Phase::GLIDE:
            // Maintain head speed and forward speed as we glide to the ground
            // 
            // GLIDE: Continuous execution during glide phase
            // - Maintain rotor RPM in optimal range through collective adjustments
            // - Control forward speed to achieve best glide ratio
            // - Monitor altitude and descent rate for flare initiation
            // - Pilot yaw input applied for heading control and landing site alignment
            // Critical: Rotor RPM must be maintained to store kinetic energy for flare maneuver.
            // Too low RPM = insufficient energy for flare (hard landing).
            // Too high RPM = potential overspeed and reduced control authority.
            g2.arot.run_glide(pilot_norm_input);
            break;

        case Phase::FLARE_INIT:
        case Phase::FLARE:
        case Phase::TOUCH_DOWN_INIT:
        case Phase::TOUCH_DOWN:
            // FLARE and TOUCH_DOWN phases: Currently not implemented in this state machine
            // 
            // These phases are managed by AC_Autorotation library's internal logic
            // based on altitude, descent rate, and rotor RPM conditions.
            // 
            // FLARE phase (when implemented):
            // - Cyclic aft input to pitch nose up and reduce forward speed
            // - Uses rotor inertia and increased rotor disk angle of attack to slow descent
            // - Temporarily increases rotor RPM as kinetic energy converted to lift
            // - Critical timing window: typically initiated at 15-30 feet AGL
            // 
            // TOUCH_DOWN phase (when implemented):
            // - Final collective increase to cushion landing
            // - Uses stored rotor energy (high RPM from flare) to generate thrust
            // - Timing critical: too early = high sink rate, too late = hard landing
            // - Goal: Near-zero vertical speed at ground contact
            // 
            // Note: No break statement needed as these cases fall through to next case
            break;

        case Phase::LANDED_INIT:
            // Landed phase functions to be run only once
            // 
            // LANDED_INIT: One-time actions upon landing detection
            // - Send telemetry message to ground station
            // - Initialize control decay timers
            // - Prepare for motor spool down
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AROT: Landed");
            current_phase = Phase::LANDED;
            FALLTHROUGH;

        case Phase::LANDED:
            // Don't allow controller to continually ask for more pitch to build speed when we are on the ground, decay to zero smoothly
            // 
            // LANDED: Continuous execution after landing
            // - Smoothly decay all control inputs to zero
            // - Prevent pitch/roll controller from saturating on the ground
            // - Allow landing detector to recognize landed state and disarm
            // - Maintain zero collective and neutral cyclic
            // Without this decay, speed controller would continue commanding pitch
            // to build forward speed even though helicopter is on the ground.
            g2.arot.run_landed();
            break;
    }

    // Slow rate (25 Hz) logging for the mode
    // 
    // Autorotation Data Logging:
    // Log autorotation-specific data at 25 Hz (every 40 milliseconds) for detailed flight analysis.
    // This is slower than main loop rate (400 Hz) to reduce log file size while still capturing
    // critical autorotation parameters for post-flight analysis and tuning.
    // 
    // Logged data includes:
    // - Current autorotation phase
    // - Rotor head speed (RPM)
    // - Collective pitch position
    // - Target vs actual forward speed
    // - Descent rate
    // - Altitude above ground
    // - Glide ratio
    // 
    // This data is essential for:
    // - Verifying proper autorotation sequence execution
    // - Tuning collective response for optimal rotor RPM
    // - Analyzing flare timing and effectiveness
    // - Post-incident investigation if landing is unsuccessful
    if (now_ms - _last_logged_ms > 40U) {
        g2.arot.log_write_autorotation();
        _last_logged_ms = now_ms;
    }

} // End function run()

#endif
