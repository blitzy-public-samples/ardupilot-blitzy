/**
 * @file RC_Channel_Plane.cpp
 * @brief Fixed-wing aircraft RC channel handling implementation
 * 
 * @details This file implements plane-specific RC channel handling including:
 * - RC failsafe detection and validation for fixed-wing aircraft
 * - Auxiliary switch function handling for plane-specific features
 * - Flight mode switching via RC channels
 * - Quadplane-specific RC options (Q_ASSIST, AIRMODE, weather vane)
 * - Fixed-wing specific features (crow flaps, landing flare, inverted flight)
 * - Soaring thermal detection RC control
 * - Emergency procedures (landing abort, parachute release)
 * 
 * The RC_Channel_Plane and RC_Channels_Plane classes extend the base RC_Channel
 * framework to provide fixed-wing specific behavior, handling differences between
 * traditional fixed-wing and quadplane (VTOL) configurations.
 * 
 * @note This implementation integrates with the Plane vehicle class singleton
 *       for state management and mode transitions.
 * 
 * @see RC_Channel_Plane.h for class definitions
 * @see RC_Channel/RC_Channel.h for base class implementation
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp
 */

#include "Plane.h"

#include "RC_Channel_Plane.h"
#include "qautotune.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Plane
#define RC_CHANNEL_SUBCLASS RC_Channel_Plane

#include <RC_Channel/RC_Channels_VarInfo.h>

/**
 * @brief Get the RC channel number configured for flight mode switching
 * 
 * @details Returns the channel number (1-16) configured via the FLTMODE_CH parameter
 *          for flight mode selection. This allows pilots to assign mode switching
 *          to any available RC channel (typically channel 5 or 8).
 * 
 * @return int8_t RC channel number (1-16), or -1 if not configured
 * 
 * @note This callback is not presently used on Plane - mode switching is handled
 *       directly by the mode switch handler. Retained for framework compatibility.
 * 
 * @see Plane::read_mode_switch() for actual mode switching implementation
 */
int8_t RC_Channels_Plane::flight_mode_channel_number() const
{
    return plane.g.flight_mode_channel.get();
}

/**
 * @brief Check if vehicle is currently in RC failsafe condition
 * 
 * @details Determines whether the aircraft has lost RC signal or is experiencing
 *          RC communication failures that trigger failsafe mode. This combines
 *          both active failsafe state and historical failsafe flags.
 * 
 *          RC failsafe is triggered when:
 *          - No valid RC pulses received within timeout period (typically 1 second)
 *          - Throttle failsafe action is active (FS_SHORT_ACTN or FS_LONG_ACTN)
 *          - RC signal quality drops below minimum threshold
 * 
 * @return true if in RC failsafe condition
 * @return false if RC signal is valid and reliable
 * 
 * @note This is checked before allowing pilot RC input to control the aircraft.
 *       During failsafe, autonomous failsafe actions take priority over RC input.
 * 
 * @see Plane::rc_failsafe_active() for real-time failsafe detection
 * @see has_valid_input() for complete input validation
 */
bool RC_Channels_Plane::in_rc_failsafe() const
{
    return (plane.rc_failsafe_active() || plane.failsafe.rc_failsafe);
}

/**
 * @brief Check if RC input is valid and safe to use for control
 * 
 * @details Validates that RC input is currently reliable and safe for direct
 *          pilot control of the aircraft. This performs comprehensive checks
 *          beyond simple signal presence, including failsafe state and throttle
 *          validation.
 * 
 *          Returns false if:
 *          - RC failsafe is active (signal loss or degradation)
 *          - Throttle failsafe counter is non-zero (throttle signal unreliable)
 * 
 * @return true if RC input is valid and safe for control
 * @return false if RC input should be ignored due to failsafe conditions
 * 
 * @note This is used by the mode logic to determine whether pilot stick inputs
 *       should be processed or ignored. During invalid input, the aircraft will
 *       maintain current autonomous behavior or execute failsafe actions.
 * 
 * @warning Safety-critical: Incorrectly reporting valid input during failsafe
 *          could allow unstable control with degraded RC signals.
 * 
 * @see in_rc_failsafe() for RC signal failsafe detection
 * @see Plane::failsafe for throttle counter and failsafe state
 */
bool RC_Channels_Plane::has_valid_input() const
{
    if (in_rc_failsafe()) {
        return false;
    }
    if (plane.failsafe.throttle_counter != 0) {
        return false;
    }
    return true;
}

/**
 * @brief Get the RC channel used for rudder stick arming
 * 
 * @details Returns the rudder channel pointer for stick-based arming/disarming.
 *          ArduPlane uses rudder stick position (typically full right for 2+ seconds)
 *          to arm the vehicle when ARMING_RUDDER parameter is enabled.
 * 
 * @return RC_Channel* Pointer to rudder channel object (typically channel 4)
 * 
 * @note Stick arming requires:
 *       - Rudder stick full right (>95%) for arming
 *       - Rudder stick full left (<5%) for disarming
 *       - Throttle at zero for safety
 *       - All pre-arm checks passed
 * 
 * @see AP_Arming_Plane for complete arming logic and safety checks
 * @see Plane::channel_rudder for the rudder channel object
 */
RC_Channel * RC_Channels_Plane::get_arming_channel(void) const
{
    return plane.channel_rudder;
}

/**
 * @brief Handle auxiliary switch for flight mode changes
 * 
 * @details Processes auxiliary RC switch positions to engage specific flight modes
 *          or return to mode switch flight mode. This allows pilots to temporarily
 *          override the main mode switch using auxiliary switches (e.g., RTL on
 *          switch position, AUTO on another switch).
 * 
 *          Behavior by switch position:
 *          - HIGH: Engage the specified mode (if mode change allowed)
 *          - MIDDLE/LOW: If currently in this mode, return to mode switch flight mode
 * 
 * @param[in] number Flight mode number to engage (Mode::Number enum)
 * @param[in] ch_flag Switch position (HIGH, MIDDLE, or LOW)
 * 
 * @note If mode change fails (e.g., mode not available, arming state prevents mode),
 *       the aircraft remains in current flight mode with no error indication.
 * 
 * @note This implements "momentary" mode switching - returning switch to LOW/MIDDLE
 *       returns to the main mode switch setting, allowing pilots to temporarily
 *       activate modes without changing the main mode switch.
 * 
 * @see Plane::set_mode_by_number() for mode transition logic
 * @see RC_Channels::reset_mode_switch() for mode switch reset behavior
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:41-57
 */
void RC_Channel_Plane::do_aux_function_change_mode(const Mode::Number number,
                                                   const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        plane.set_mode_by_number(number, ModeReason::AUX_FUNCTION);
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (plane.control_mode->mode_number() == number) {
            rc().reset_mode_switch();
        }
    }
}

#if HAL_QUADPLANE_ENABLED
/**
 * @brief Control quadplane Q_ASSIST state via auxiliary switch
 * 
 * @details Q_ASSIST provides automatic VTOL motor assistance during fixed-wing flight
 *          when the aircraft is approaching stall or unable to maintain altitude.
 *          This function allows pilots to override Q_ASSIST behavior using a 3-position
 *          RC switch for different flight scenarios.
 * 
 *          Switch positions:
 *          - HIGH: Force Q_ASSIST always enabled (VTOL motors always ready to assist)
 *          - MIDDLE: Normal Q_ASSIST operation (automatic engagement based on flight conditions)
 *          - LOW: Q_ASSIST disabled (fixed-wing only, no VTOL assistance)
 * 
 * @param[in] ch_flag Switch position (HIGH, MIDDLE, or LOW)
 * 
 * @note FORCE_ENABLED useful for:
 *       - High wind conditions requiring extra power
 *       - Learning to fly quadplane safely with backup always ready
 *       - Difficult terrain with limited landing options
 * 
 * @note ASSIST_DISABLED useful for:
 *       - Pure fixed-wing efficiency during cruise
 *       - Testing fixed-wing performance limits
 *       - Competition flying where VTOL assistance not allowed
 * 
 * @warning Disabling Q_ASSIST removes automatic stall protection. Only disable
 *          when confident in fixed-wing flight envelope and conditions.
 * 
 * @see VTOL_Assist::set_state() for Q_ASSIST state management
 * @see Q_ASSIST_SPEED, Q_ASSIST_ALT parameters for automatic trigger thresholds
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:60-78
 */
void RC_Channel_Plane::do_aux_function_q_assist_state(AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Force enabled");
            plane.quadplane.assist.set_state(VTOL_Assist::STATE::FORCE_ENABLED);
            break;

        case AuxSwitchPos::MIDDLE:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Enabled");
            plane.quadplane.assist.set_state(VTOL_Assist::STATE::ASSIST_ENABLED);
            break;

        case AuxSwitchPos::LOW:
            gcs().send_text(MAV_SEVERITY_INFO, "QAssist: Disabled");
            plane.quadplane.assist.set_state(VTOL_Assist::STATE::ASSIST_DISABLED);
            break;
    }
}
#endif  // HAL_QUADPLANE_ENABLED

/**
 * @brief Control crow flap mode via auxiliary switch
 * 
 * @details Crow flaps (also called butterfly or V-tail airbrakes) are a aerodynamic
 *          braking technique where both flaperons deflect upward simultaneously to
 *          increase drag and descent rate without increasing airspeed. This is
 *          commonly used in sailplanes and gliders for steep approaches.
 * 
 *          Switch positions:
 *          - HIGH: Crow flaps disabled (normal aileron operation)
 *          - MIDDLE: Progressive crow (gradual crow deployment proportional to input)
 *          - LOW: Normal crow flaps (full crow deployment when activated)
 * 
 * @param[in] ch_flag Switch position (HIGH, MIDDLE, or LOW)
 * 
 * @note Crow flap deployment is typically controlled by a separate input channel
 *       (configured via CROW_FLAP_IN parameter) when crow mode is enabled.
 * 
 * @note Progressive mode allows variable crow deployment for precise descent
 *       rate control, while normal mode provides full deployment for maximum
 *       drag and steepest descent.
 * 
 * @note Crow flaps affect both roll control (reduced aileron authority) and
 *       pitch trim (nose-down tendency). Pilots should anticipate these effects.
 * 
 * @see Plane::CrowMode enum for crow mode definitions
 * @see SRV_Channel for crow flap servo output configuration
 * @see CROW_FLAP_IN, CROW_FLAP_WEIGHT parameters
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:81-97
 */
void RC_Channel_Plane::do_aux_function_crow_mode(AuxSwitchPos ch_flag)
{
        switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.crow_mode = Plane::CrowMode::CROW_DISABLED;
            gcs().send_text(MAV_SEVERITY_INFO, "Crow Flaps Disabled");
            break;
        case AuxSwitchPos::MIDDLE:
            gcs().send_text(MAV_SEVERITY_INFO, "Progressive Crow Flaps"); 
            plane.crow_mode = Plane::CrowMode::PROGRESSIVE;   
            break;
        case AuxSwitchPos::LOW:
            plane.crow_mode = Plane::CrowMode::NORMAL;
            gcs().send_text(MAV_SEVERITY_INFO, "Normal Crow Flaps");
            break;
        }    
}

#if HAL_SOARING_ENABLED
/**
 * @brief Control autonomous soaring/thermalling mode via 3-position switch
 * 
 * @details Soaring mode enables the aircraft to autonomously detect and exploit
 *          thermals (rising air currents) to gain altitude without motor power.
 *          The soaring controller uses variometer data and flight patterns to
 *          identify thermal cores and circle within them for maximum lift.
 * 
 *          Switch positions:
 *          - HIGH: Auto mode change (controller autonomously switches between
 *                  LOITER for thermalling and CRUISE for inter-thermal gliding)
 *          - MIDDLE: Manual mode change (pilot manually switches modes, but
 *                    controller provides thermal detection and navigation guidance)
 *          - LOW: Soaring disabled (normal fixed-wing flight, no thermal detection)
 * 
 * @param[in] ch_flag Switch position (HIGH, MIDDLE, or LOW)
 * 
 * @note AUTO mode provides fully autonomous soaring - aircraft will automatically
 *       enter LOITER when thermal detected and CRUISE when thermal lost or topped out.
 * 
 * @note MANUAL mode provides thermal indicators to pilot but requires manual
 *       mode switching between LOITER (for circling) and CRUISE (for gliding).
 * 
 * @note Requires working variometer (vertical speed sensor) for thermal detection
 *       and appropriate SOAR_* parameters configured for local conditions.
 * 
 * @see SoaringController::ActiveStatus for soaring state definitions
 * @see SoaringController for thermal detection and navigation algorithms
 * @see SOAR_ENABLE, SOAR_ALT_MIN, SOAR_VSPEED parameters
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:100-117
 */
void RC_Channel_Plane::do_aux_function_soaring_3pos(AuxSwitchPos ch_flag)
{
    SoaringController::ActiveStatus desired_state = SoaringController::ActiveStatus::SOARING_DISABLED;

    switch (ch_flag) {
        case AuxSwitchPos::LOW:
            desired_state = SoaringController::ActiveStatus::SOARING_DISABLED;
            break;
        case AuxSwitchPos::MIDDLE:
            desired_state = SoaringController::ActiveStatus::MANUAL_MODE_CHANGE;
            break;
        case AuxSwitchPos::HIGH:
            desired_state = SoaringController::ActiveStatus::AUTO_MODE_CHANGE;
            break;
        }

    plane.g2.soaring_controller.set_pilot_desired_state(desired_state);
}
#endif

/**
 * @brief Control manual landing flare via auxiliary switch
 * 
 * @details Landing flare is the final phase of landing where the aircraft pitches
 *          up to arrest descent rate and reduce touchdown speed just before ground
 *          contact. This function allows manual flare initiation via RC switch for
 *          improved landing control in challenging conditions.
 * 
 *          Switch positions:
 *          - HIGH: Flare enabled with pitch target (commands specific nose-up attitude)
 *          - MIDDLE: Flare enabled without pitch target (reduces throttle, maintains level)
 *          - LOW: Flare disabled (normal flight control)
 * 
 * @param[in] ch_flag Switch position (HIGH, MIDDLE, or LOW)
 * 
 * @note ENABLED_PITCH_TARGET mode commands positive pitch angle (typically 5-15°)
 *       to actively arrest descent. This provides the strongest flare effect but
 *       requires sufficient airspeed to maintain control.
 * 
 * @note ENABLED_NO_PITCH_TARGET mode reduces throttle to idle and attempts to
 *       hold level flight, providing a gentler flare with less pitch change.
 *       Useful for aircraft with high wing loading or limited elevator authority.
 * 
 * @note Manual flare control is typically used when automatic flare timing is
 *       suboptimal (e.g., rough terrain, cross-wind landings, short field approaches).
 * 
 * @warning Initiating flare too high above ground can cause stall and hard landing.
 *          Typically activate flare at 1-3 meters altitude.
 * 
 * @see Plane::FlareMode enum for flare mode definitions
 * @see LAND_FLARE_ALT, LAND_FLARE_SEC parameters for automatic flare
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:120-133
 */
void RC_Channel_Plane::do_aux_function_flare(AuxSwitchPos ch_flag)
{
        switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.flare_mode = Plane::FlareMode::ENABLED_PITCH_TARGET;
            break;
        case AuxSwitchPos::MIDDLE:
            plane.flare_mode = Plane::FlareMode::ENABLED_NO_PITCH_TARGET;
            break;
        case AuxSwitchPos::LOW:
            plane.flare_mode = Plane::FlareMode::FLARE_DISABLED;
            break;
        }    
}


/**
 * @brief Initialize auxiliary RC function on system startup
 * 
 * @details Called during RC channel initialization to set initial state for each
 *          configured auxiliary function. Some functions require initialization to
 *          establish correct startup state (e.g., inverted flight OFF, crow flaps
 *          disabled), while others are stateless and require no initialization.
 * 
 *          This function categorizes auxiliary functions into three groups:
 *          1. No initialization needed (mode switches, momentary actions)
 *          2. Call run_aux_function() to initialize state (stateful features)
 *          3. Special initialization (e.g., reverse throttle range setup)
 * 
 * @param[in] ch_option Auxiliary function type (AUX_FUNC enum)
 * @param[in] ch_flag Current switch position at startup (HIGH/MIDDLE/LOW)
 * 
 * @note Functions requiring initialization (group 2) have current switch state
 *       applied via run_aux_function() to ensure system starts in correct state
 *       matching physical switch position.
 * 
 * @note REVERSE_THROTTLE has special initialization: sets throttle channel range
 *       and marks reverse throttle as available, but does NOT activate reverse
 *       thrust at startup for safety.
 * 
 * @note Most mode switches (AUTO, RTL, LOITER, etc.) do not initialize because
 *       startup mode is determined by initial mode switch, not auxiliary switches.
 * 
 * @see run_aux_function() for state application logic
 * @see RC_Channel::init_aux_function() for base class initialization
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:136-221
 */
void RC_Channel_Plane::init_aux_function(const RC_Channel::AUX_FUNC ch_option,
                                         const RC_Channel::AuxSwitchPos ch_flag)
{
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::TRAINING:
    case AUX_FUNC::FLAP:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::LOITER:
    case AUX_FUNC::MANUAL:
    case AUX_FUNC::RTL:
    case AUX_FUNC::TAKEOFF:
    case AUX_FUNC::FBWA:
    case AUX_FUNC::AIRBRAKE:
#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::QRTL:
    case AUX_FUNC::QSTABILIZE:
#endif
    case AUX_FUNC::FBWA_TAILDRAGGER:
    case AUX_FUNC::FWD_THR:
    case AUX_FUNC::LANDING_FLARE:
#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_RELEASE:
#endif
    case AUX_FUNC::MODE_SWITCH_RESET:
    case AUX_FUNC::CRUISE:
#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::ARMDISARM_AIRMODE:
#endif
    case AUX_FUNC::PLANE_AUTO_LANDING_ABORT:
    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
    case AUX_FUNC::EMERGENCY_LANDING_EN:
    case AUX_FUNC::FW_AUTOTUNE:
    case AUX_FUNC::VFWD_THR_OVERRIDE:
    case AUX_FUNC::PRECISION_LOITER:
#if QAUTOTUNE_ENABLED
    case AUX_FUNC::AUTOTUNE_TEST_GAINS:
#endif
#if AP_QUICKTUNE_ENABLED
    case AUX_FUNC::QUICKTUNE:
#endif
#if MODE_AUTOLAND_ENABLED
    case AUX_FUNC::AUTOLAND:
#endif
#if AP_PLANE_SYSTEMID_ENABLED
    case AUX_FUNC::SYSTEMID:
#endif
        break;
    case AUX_FUNC::SOARING:
#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::Q_ASSIST:
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::WEATHER_VANE_ENABLE:
#endif
#if AP_AIRSPEED_AUTOCAL_ENABLE
    case AUX_FUNC::ARSPD_CALIBRATE:
#endif
    case AUX_FUNC::TER_DISABLE:
    case AUX_FUNC::CROW_SELECT:
#if AP_ICENGINE_ENABLED
    case AUX_FUNC::ICE_START_STOP:
#endif
        run_aux_function(ch_option, ch_flag, AuxFuncTrigger::Source::INIT, ch_in);
        break;

    case AUX_FUNC::REVERSE_THROTTLE:
        plane.have_reverse_throttle_rc_option = true;
        // setup input throttle as a range. This is needed as init_aux_function is called
        // after set_control_channels()
        if (plane.channel_throttle) {
            plane.channel_throttle->set_range(100);
        }
        // note that we don't call do_aux_function() here as we don't
        // want to startup with reverse thrust
        break;

    default:
        // handle in parent class
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

/**
 * @brief Main handler for auxiliary RC switch functions
 * 
 * @details Processes auxiliary switch changes and executes corresponding fixed-wing
 *          specific actions. This is the primary dispatch function for all plane
 *          auxiliary functions, handling mode changes, feature toggles, and emergency
 *          procedures triggered by RC switches.
 * 
 *          Supported auxiliary functions include:
 *          - Flight mode selection (AUTO, RTL, LOITER, CIRCLE, MANUAL, etc.)
 *          - Fixed-wing features (inverted flight, crow flaps, landing flare)
 *          - Quadplane features (Q_ASSIST, AIRMODE, forward throttle override)
 *          - Soaring (thermal detection and autonomous thermalling)
 *          - Emergency procedures (landing abort, parachute, emergency landing)
 *          - Tuning and calibration (FW_AUTOTUNE, ARSPD_CALIBRATE, QUICKTUNE)
 *          - Input labels (FLAP, AIRBRAKE, FWD_THR - no action, just input identification)
 * 
 * @param[in] trigger Auxiliary function trigger containing:
 *                    - func: Function type (AUX_FUNC enum)
 *                    - pos: Switch position (HIGH/MIDDLE/LOW)
 *                    - source: Trigger source (RC, MAVLink, scripting)
 * 
 * @return true if function was handled successfully
 * @return false if function should be handled by base class
 * 
 * @note Input label functions (FLAP, AIRBRAKE, FWD_THR) intentionally do nothing
 *       - they exist solely to identify RC input channels in the ground station.
 * 
 * @note Many functions use 3-position switches (HIGH/MIDDLE/LOW) to provide
 *       multiple states or progressive control (e.g., Q_ASSIST, SOARING, AIRMODE).
 * 
 * @note Emergency functions (PARACHUTE_RELEASE, PLANE_AUTO_LANDING_ABORT) typically
 *       only respond to HIGH position for safety - prevents accidental activation.
 * 
 * @warning Safety-critical function: Incorrect auxiliary function processing could
 *          cause unexpected mode changes, disable safety features, or trigger
 *          emergency procedures. All state changes validated before execution.
 * 
 * @see init_aux_function() for startup initialization of auxiliary functions
 * @see RC_Channel::do_aux_function() for base class shared functions
 * @see AUX_FUNC enum for complete list of available functions
 * 
 * Source: ArduPlane/RC_Channel_Plane.cpp:224-504
 */
bool RC_Channel_Plane::do_aux_function(const AuxFuncTrigger &trigger)
{
    const AUX_FUNC &ch_option = trigger.func;
    const AuxSwitchPos &ch_flag = trigger.pos;

    switch(ch_option) {
    case AUX_FUNC::INVERTED:
        plane.inverted_flight = (ch_flag == AuxSwitchPos::HIGH);
        break;

    case AUX_FUNC::REVERSE_THROTTLE:
        plane.reversed_throttle = (ch_flag == AuxSwitchPos::HIGH);
        gcs().send_text(MAV_SEVERITY_INFO, "RevThrottle: %s", plane.reversed_throttle?"ENABLE":"DISABLE");
        break;

    case AUX_FUNC::AUTO:
        do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
        break;

    case AUX_FUNC::CIRCLE:
        do_aux_function_change_mode(Mode::Number::CIRCLE, ch_flag);
        break;

    case AUX_FUNC::ACRO:
        do_aux_function_change_mode(Mode::Number::ACRO, ch_flag);
        break;

    case AUX_FUNC::TRAINING:
        do_aux_function_change_mode(Mode::Number::TRAINING, ch_flag);
        break;
        
    case AUX_FUNC::LOITER:
        do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
        break;        

    case AUX_FUNC::GUIDED:
        do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
        break;

    case AUX_FUNC::MANUAL:
        do_aux_function_change_mode(Mode::Number::MANUAL, ch_flag);
        break;

    case AUX_FUNC::RTL:
        do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
        break;

    case AUX_FUNC::TAKEOFF:
        do_aux_function_change_mode(Mode::Number::TAKEOFF, ch_flag);
        break;

    case AUX_FUNC::FBWA:
        do_aux_function_change_mode(Mode::Number::FLY_BY_WIRE_A, ch_flag);
        break;

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::QRTL:
        do_aux_function_change_mode(Mode::Number::QRTL, ch_flag);
        break;

    case AUX_FUNC::QSTABILIZE:
        do_aux_function_change_mode(Mode::Number::QSTABILIZE, ch_flag);
        break;

    case AUX_FUNC::VFWD_THR_OVERRIDE: {
        const bool enable = (ch_flag == AuxSwitchPos::HIGH);
        if (enable != plane.quadplane.vfwd_enable_active) {
            plane.quadplane.vfwd_enable_active = enable;
            gcs().send_text(MAV_SEVERITY_INFO, "QFwdThr: %s", enable?"ON":"OFF");
        }
        break;
    }
#endif

#if HAL_SOARING_ENABLED
    case AUX_FUNC::SOARING:
        do_aux_function_soaring_3pos(ch_flag);
        break;
#endif

    case AUX_FUNC::FLAP:
    case AUX_FUNC::FBWA_TAILDRAGGER:
    case AUX_FUNC::AIRBRAKE:
        break; // input labels, nothing to do

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::Q_ASSIST:
        do_aux_function_q_assist_state(ch_flag);
        break;
#endif

    case AUX_FUNC::FWD_THR:
        break; // VTOL forward throttle input label, nothing to do

    case AUX_FUNC::TER_DISABLE:
            switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                plane.non_auto_terrain_disable = true;
                if (plane.control_mode->allows_terrain_disable()) {
                    plane.set_target_altitude_current();
                }
                break;
            case AuxSwitchPos::MIDDLE:
                break;
            case AuxSwitchPos::LOW:
                plane.non_auto_terrain_disable = false;
                if (plane.control_mode->allows_terrain_disable()) {
                    plane.set_target_altitude_current();
                }
                break;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "NON AUTO TERRN: %s", plane.non_auto_terrain_disable?"OFF":"ON");
        break;

    case AUX_FUNC::CROW_SELECT:
        do_aux_function_crow_mode(ch_flag);
        break;

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::AIRMODE:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.quadplane.air_mode = AirMode::ON;
            plane.quadplane.throttle_wait = false;
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            plane.quadplane.air_mode = AirMode::OFF;
            break;
        }
        break;
#endif

#if AP_AIRSPEED_AUTOCAL_ENABLE
    case AUX_FUNC::ARSPD_CALIBRATE:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.airspeed.set_calibration_enabled(true);
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            plane.airspeed.set_calibration_enabled(false);
            break;
        }
        break;
#endif

    case AUX_FUNC::LANDING_FLARE:
        do_aux_function_flare(ch_flag);
        break;

#if HAL_PARACHUTE_ENABLED
    case AUX_FUNC::PARACHUTE_RELEASE:
        if (ch_flag == AuxSwitchPos::HIGH) {
            plane.parachute_manual_release();
        }
        break;
#endif

    case AUX_FUNC::MODE_SWITCH_RESET:
        rc().reset_mode_switch();
        break;

    case AUX_FUNC::CRUISE:
        do_aux_function_change_mode(Mode::Number::CRUISE, ch_flag);
        break;

#if HAL_QUADPLANE_ENABLED
    case AUX_FUNC::ARMDISARM_AIRMODE:
        RC_Channel::do_aux_function_armdisarm(ch_flag);
        if (plane.arming.is_armed()) {
            plane.quadplane.air_mode = AirMode::ON;
            plane.quadplane.throttle_wait = false;
        }
        break;

    case AUX_FUNC::WEATHER_VANE_ENABLE: {
        if (plane.quadplane.weathervane != nullptr) {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    plane.quadplane.weathervane->allow_weathervaning(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    plane.quadplane.weathervane->allow_weathervaning(false);
                    break;
            }
        }
        break;
    }
#endif

    case AUX_FUNC::PLANE_AUTO_LANDING_ABORT:
        switch(ch_flag) {
        case AuxSwitchPos::HIGH:
            IGNORE_RETURN(plane.trigger_land_abort(0));
            break;
        case AuxSwitchPos::MIDDLE:
        case AuxSwitchPos::LOW:
            break;
        }
        break;

    case AUX_FUNC::TRIM_TO_CURRENT_SERVO_RC:
        if (ch_flag == AuxSwitchPos::HIGH) {
            plane.trim_radio();
        }
        break;

    case AUX_FUNC::EMERGENCY_LANDING_EN:
        switch (ch_flag) {
        case AuxSwitchPos::HIGH:
            plane.emergency_landing = true;
            break;
        case AuxSwitchPos::MIDDLE:
            break;
        case AuxSwitchPos::LOW:
            plane.emergency_landing = false;
            break;
        }
        break;

    case AUX_FUNC::FW_AUTOTUNE:
        if (ch_flag == AuxSwitchPos::HIGH && plane.control_mode->mode_allows_autotuning()) {
           plane.autotune_enable(true);
        } else if (ch_flag == AuxSwitchPos::HIGH) {
           GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Autotuning not allowed in this mode!");
        } else {
           plane.autotune_enable(false); 
        }
        break;

    case AUX_FUNC::PRECISION_LOITER:
        // handled by lua scripting, just ignore here
        break;

#if QAUTOTUNE_ENABLED
    case AUX_FUNC::AUTOTUNE_TEST_GAINS:
        plane.quadplane.qautotune.do_aux_function(ch_flag);
        break;
#endif

#if AP_QUICKTUNE_ENABLED
    case AUX_FUNC::QUICKTUNE:
        plane.quicktune.update_switch_pos(ch_flag);
        break;
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    case AUX_FUNC::SYSTEMID:
        if (ch_flag == AuxSwitchPos::HIGH) {
            plane.g2.systemid.start();
        } else if (ch_flag == AuxSwitchPos::LOW) {
            plane.g2.systemid.stop();
        }
        break;
#endif

#if AP_ICENGINE_ENABLED
    case AUX_FUNC::ICE_START_STOP:
        plane.g2.ice_control.do_aux_function(trigger);
        break;
#endif

#if MODE_AUTOLAND_ENABLED
    case AUX_FUNC::AUTOLAND:
        do_aux_function_change_mode(Mode::Number::AUTOLAND, ch_flag);
        break;

#endif

    default:
        return RC_Channel::do_aux_function(trigger);
    }

    return true;
}
