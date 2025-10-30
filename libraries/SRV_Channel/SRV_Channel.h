/*
  control of servo output ranges, trim and servo reversal. This can
  optionally be used to provide separation of input and output channel
  ranges so that RCn_MIN, RCn_MAX, RCn_TRIM and RCn_REV only apply to
  the input side of RC_Channel

  It works by running servo output calculations as normal, then
  re-mapping the output according to the servo MIN/MAX/TRIM/REV from
  this object

  Only 4 channels of ranges are defined as those match the input
  channels for R/C sticks
 */

/**
 * @file SRV_Channel.h
 * @brief Servo channel management and output function mapping
 * 
 * @details Provides servo/auxiliary output abstraction separating input RC channels
 *          from output PWM channels via configurable function mapping. Handles:
 *          - Output function assignment (motor, servo, camera, parachute, etc.)
 *          - PWM range configuration (min/max/trim/reverse per channel)
 *          - Scaled value conversion (angle/range → PWM microseconds)
 *          - Channel auto-assignment based on vehicle type
 *          - Protocol backends (Volz, SBUS, Robotis, BLHeli, FETtec OneWire)
 *          - Safety mechanisms (emergency stop, failsafe values, disabled channels)
 *          
 *          Architecture: SRV_Channel (per-channel configuration) + SRV_Channels (global manager)
 * 
 * @note This is the central servo output abstraction used by all vehicle types
 * @warning Misconfiguration can cause motor/servo reversal leading to crashes
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Bitmask.h>
#include <AP_Volz_Protocol/AP_Volz_Protocol.h>
#include <AP_RobotisServo/AP_RobotisServo.h>
#include <AP_SBusOut/AP_SBusOut.h>
#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_FETtecOneWire/AP_FETtecOneWire.h>

#include "SRV_Channel_config.h"

static_assert(NUM_SERVO_CHANNELS <= 32, "More than 32 servos not supported");

class SRV_Channels;

/*
  class SRV_Channel. The class SRV_Channels contains an array of
  SRV_Channel objects. This is done to fit within the AP_Param limit
  of 64 parameters per object.
*/

/**
 * @class SRV_Channel
 * @brief Individual servo/PWM output channel with function assignment and scaling
 * 
 * @details Represents one output channel with:
 *          - Assigned function (motor, aileron, camera trigger, etc.)
 *          - PWM limits (SERVOn_MIN, SERVOn_MAX, SERVOn_TRIM in microseconds)
 *          - Reversal flag (SERVOn_REVERSED)
 *          - Type configuration (angle vs range scaling)
 *          - Current output value
 *          
 *          Typical configuration via parameters:
 *          ```
 *          SERVO1_FUNCTION = 33  # Motor 1
 *          SERVO1_MIN = 1000     # Full reverse/stop (μs)
 *          SERVO1_MAX = 2000     # Full forward (μs)
 *          SERVO1_TRIM = 1500    # Neutral (μs)
 *          SERVO1_REVERSED = 0   # Normal direction
 *          ```
 *          
 *          Output workflow:
 *          1. Vehicle code calls SRV_Channels::set_output_scaled(function, value)
 *          2. SRV_Channels::calc_pwm() converts scaled values to PWM per channel
 *          3. calc_pwm() applies min/max/trim/reverse transformation
 *          4. output_ch() writes final PWM to hal.rcout hardware
 * 
 * @note Parameters stored via AP_Param system (RCn_ namespace for compatibility)
 * @note Thread-safe via semaphores in SRV_Channels manager
 * @warning Incorrect function assignment can mix up control surfaces causing crashes
 */
class SRV_Channel {
public:
    friend class SRV_Channels;

    // constructor
    SRV_Channel(void);

    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @enum Function
     * @brief Output function types for channel assignment
     * 
     * @details Defines ~190 possible output functions organized by category:
     *          
     *          **Motors** (k_motor1-k_motor32): Multicopter/VTOL motor outputs
     *          - Used with AP_Motors mixer for thrust allocation
     *          - Auto-assigned based on frame type (quad/hex/octo/etc.)
     *          - Support throttle curves, slew rate limiting, emergency stop
     *          
     *          **Control Surfaces** (k_aileron, k_elevator, k_rudder, k_flap, etc.):
     *          - Fixed-wing and VTOL control surfaces
     *          - Angle-based scaling around trim point
     *          - Support differential mixing (elevons, flaperons, v-tail)
     *          
     *          **Mounts/Gimbals** (k_mount_pan/tilt/roll, k_mount2_*):
     *          - Camera gimbal stabilization (2-axis and 3-axis)
     *          - Dual mount support for two independent gimbals
     *          - Integration with AP_Mount for MAVLink gimbal control
     *          
     *          **Actuators** (k_gripper, k_parachute_release, k_landing_gear, etc.):
     *          - Binary or range-based peripheral control
     *          - Typically used with mission commands (DO_GRIPPER, DO_PARACHUTE)
     *          
     *          **RC Pass-through** (k_rcin1-k_rcin16, k_rcin1_mapped-k_rcin16_mapped):
     *          - Direct RC input → output mapping
     *          - k_manual: Traditional manual control (deprecated)
     *          - k_rcin*: Pass-through for auxiliary switches/knobs
     *          - k_rcin*_mapped: Pass-through with min/max/trim applied
     *          
     *          **Scripting Outputs** (k_scripting1-k_scripting16):
     *          - Lua script-controlled outputs
     *          - Allow custom control logic in user scripts
     *          
     *          **LEDs** (k_LED_neopixel1-4, k_ProfiLED_1-3, k_ProfiLED_Clock):
     *          - Addressable LED control (WS2812, ProfiLED)
     *          - PWM-based protocols for LED strips
     *          
     *          **Special** (k_GPIO, k_alarm, k_min, k_max, k_trim):
     *          - k_GPIO=-1: Use pin as digital I/O (not PWM)
     *          - k_none=0: Disabled or generic PWM (DO_SET_SERVO target)
     *          - k_min/max/trim: Always output respective limit (testing)
     *          - k_alarm/alarm_inverted: Audible alert output
     * 
     * @note k_nr_aux_servo_functions is sentinel value (must be last enum)
     * @note New functions added before k_nr_aux_servo_functions to maintain ABI
     * @warning Changing enum values breaks parameter compatibility - never reorder
     */
    typedef enum
    {
        k_GPIO                  = -1,           ///< used as GPIO pin (input or output)
        k_none                  = 0,            ///< general use PWM output used by do-set-servo commands and lua scripts
        k_manual                = 1,            ///< manual, just pass-thru the RC in signal
        k_flap                  = 2,            ///< flap
        k_flap_auto             = 3,            ///< flap automated
        k_aileron               = 4,            ///< aileron
        k_unused1               = 5,            ///< unused function
        k_mount_pan             = 6,            ///< mount yaw (pan)
        k_mount_tilt            = 7,            ///< mount pitch (tilt)
        k_mount_roll            = 8,            ///< mount roll
        k_mount_open            = 9,            ///< mount open (deploy) / close (retract)
        k_cam_trigger           = 10,           ///< camera trigger
        k_egg_drop              = 11,           ///< egg drop, deprecated
        k_mount2_pan            = 12,           ///< mount2 yaw (pan)
        k_mount2_tilt           = 13,           ///< mount2 pitch (tilt)
        k_mount2_roll           = 14,           ///< mount2 roll
        k_mount2_open           = 15,           ///< mount2 open (deploy) / close (retract)
        k_dspoilerLeft1         = 16,           ///< differential spoiler 1 (left wing)
        k_dspoilerRight1        = 17,           ///< differential spoiler 1 (right wing)
        k_aileron_with_input    = 18,            ///< aileron, with rc input, deprecated
        k_elevator              = 19,            ///< elevator
        k_elevator_with_input   = 20,            ///< elevator, with rc input, deprecated
        k_rudder                = 21,            ///< secondary rudder channel
        k_sprayer_pump          = 22,            ///< crop sprayer pump channel
        k_sprayer_spinner       = 23,            ///< crop sprayer spinner channel
        k_flaperon_left         = 24,            ///< flaperon, left wing
        k_flaperon_right        = 25,            ///< flaperon, right wing
        k_steering              = 26,            ///< ground steering, used to separate from rudder
        k_parachute_release     = 27,            ///< parachute release
        k_gripper               = 28,            ///< gripper
        k_landing_gear_control  = 29,            ///< landing gear controller
        k_engine_run_enable     = 30,            ///< engine kill switch, used for gas airplanes and helicopters
        k_heli_rsc              = 31,            ///< helicopter RSC output
        k_heli_tail_rsc         = 32,            ///< helicopter tail RSC output
        k_motor1                = 33,            ///< these allow remapping of copter motors
        k_motor2                = 34,
        k_motor3                = 35,
        k_motor4                = 36,
        k_motor5                = 37,
        k_motor6                = 38,
        k_motor7                = 39,
        k_motor8                = 40,
        k_motor_tilt            = 41,            ///< tiltrotor motor tilt control
        k_generator_control     = 42,            ///< state control for generator
        k_tiltMotorRear         = 45,            ///<vectored thrust, rear tilt
        k_tiltMotorRearLeft     = 46,            ///<vectored thrust, rear left tilt
        k_tiltMotorRearRight    = 47,            ///<vectored thrust, rear right tilt
        k_rcin1                 = 51,            ///< these are for pass-thru from arbitrary rc inputs
        k_rcin2                 = 52,
        k_rcin3                 = 53,
        k_rcin4                 = 54,
        k_rcin5                 = 55,
        k_rcin6                 = 56,
        k_rcin7                 = 57,
        k_rcin8                 = 58,
        k_rcin9                 = 59,
        k_rcin10                = 60,
        k_rcin11                = 61,
        k_rcin12                = 62,
        k_rcin13                = 63,
        k_rcin14                = 64,
        k_rcin15                = 65,
        k_rcin16                = 66,
        k_ignition              = 67,
        k_choke                 = 68,           /// not used
        k_starter               = 69,
        k_throttle              = 70,
        k_tracker_yaw           = 71,            ///< antennatracker yaw
        k_tracker_pitch         = 72,            ///< antennatracker pitch
        k_throttleLeft          = 73,
        k_throttleRight         = 74,
        k_tiltMotorLeft         = 75,            ///< vectored thrust, left tilt
        k_tiltMotorRight        = 76,            ///< vectored thrust, right tilt
        k_elevon_left           = 77,
        k_elevon_right          = 78,
        k_vtail_left            = 79,
        k_vtail_right           = 80,
        k_boost_throttle        = 81,            ///< vertical booster throttle
        k_motor9                = 82,
        k_motor10               = 83,
        k_motor11               = 84,
        k_motor12               = 85,
        k_dspoilerLeft2         = 86,           ///< differential spoiler 2 (left wing)
        k_dspoilerRight2        = 87,           ///< differential spoiler 2 (right wing)
        k_winch                 = 88,
        k_mainsail_sheet        = 89,           ///< Main Sail control via sheet
        k_cam_iso               = 90,
        k_cam_aperture          = 91,
        k_cam_focus             = 92,
        k_cam_shutter_speed     = 93,
        k_scripting1            = 94,           ///< Scripting related outputs
        k_scripting2            = 95,
        k_scripting3            = 96,
        k_scripting4            = 97,
        k_scripting5            = 98,
        k_scripting6            = 99,
        k_scripting7            = 100,
        k_scripting8            = 101,
        k_scripting9            = 102,
        k_scripting10           = 103,
        k_scripting11           = 104,
        k_scripting12           = 105,
        k_scripting13           = 106,
        k_scripting14           = 107,
        k_scripting15           = 108,
        k_scripting16           = 109,
        k_airbrake              = 110,
        k_LED_neopixel1         = 120,
        k_LED_neopixel2         = 121,
        k_LED_neopixel3         = 122,
        k_LED_neopixel4         = 123,
        k_roll_out              = 124,
        k_pitch_out             = 125,
        k_thrust_out            = 126,
        k_yaw_out               = 127,
        k_wingsail_elevator     = 128,
        k_ProfiLED_1            = 129,
        k_ProfiLED_2            = 130,
        k_ProfiLED_3            = 131,
        k_ProfiLED_Clock        = 132,
        k_winch_clutch          = 133,
        k_min                   = 134,  // always outputs SERVOn_MIN
        k_trim                  = 135,  // always outputs SERVOn_TRIM
        k_max                   = 136,  // always outputs SERVOn_MAX
        k_mast_rotation         = 137,
        k_alarm                 = 138,
        k_alarm_inverted        = 139,
        k_rcin1_mapped          = 140,
        k_rcin2_mapped          = 141,
        k_rcin3_mapped          = 142,
        k_rcin4_mapped          = 143,
        k_rcin5_mapped          = 144,
        k_rcin6_mapped          = 145,
        k_rcin7_mapped          = 146,
        k_rcin8_mapped          = 147,
        k_rcin9_mapped          = 148,
        k_rcin10_mapped         = 149,
        k_rcin11_mapped         = 150,
        k_rcin12_mapped         = 151,
        k_rcin13_mapped         = 152,
        k_rcin14_mapped         = 153,
        k_rcin15_mapped         = 154,
        k_rcin16_mapped         = 155,
        k_lift_release          = 156,
        k_motor13               = 160,
        k_motor14               = 161,
        k_motor15               = 162,
        k_motor16               = 163,
        k_motor17               = 164,
        k_motor18               = 165,
        k_motor19               = 166,
        k_motor20               = 167,
        k_motor21               = 168,
        k_motor22               = 169,
        k_motor23               = 170,
        k_motor24               = 171,
        k_motor25               = 172,
        k_motor26               = 173,
        k_motor27               = 174,
        k_motor28               = 175,
        k_motor29               = 176,
        k_motor30               = 177,
        k_motor31               = 178,
        k_motor32               = 179,
        k_cam_zoom              = 180,
        k_lights1               = 181,
        k_lights2               = 182,
        k_video_switch          = 183,
        k_actuator1             = 184,   // Aux channels used for controlling user peripherals
        k_actuator2             = 185,
        k_actuator3             = 186,
        k_actuator4             = 187,
        k_actuator5             = 188,
        k_actuator6             = 189,
        k_nr_aux_servo_functions         ///< This must be the last enum value (only add new values _before_ this one)
    } Function;

    // check if a function is valid for indexing into functions
    static bool valid_function(Function fn) {
        return fn >= k_none && fn < k_nr_aux_servo_functions;
    }
    bool valid_function(void) const {
        return valid_function(function);
    }
    
    /**
     * @enum Limit
     * @brief PWM limit values for constrained output modes
     * 
     * @details Used by set_output_limit() and get_limit_pwm() to output fixed values:
     *          - TRIM: Output servo_trim (neutral position)
     *          - MIN: Output servo_min (minimum PWM, honors reverse)
     *          - MAX: Output servo_max (maximum PWM, honors reverse)
     *          - ZERO_PWM: Output 0μs (hardware disables output, motor off)
     *          
     *          Common uses:
     *          - Failsafe: Set motors to ZERO_PWM or MIN (disarm)
     *          - Centering: Set control surfaces to TRIM before arming
     *          - Testing: Verify full range by commanding MIN then MAX
     * 
     * @note MIN/MAX honor reversal (reversed channel swaps min/max output)
     * @note ZERO_PWM is absolute hardware off (used for ESC disarm)
     */
    enum class Limit {
        TRIM,
        MIN,
        MAX,
        ZERO_PWM
    };

    /**
     * @brief Set channel output directly in PWM microseconds
     * 
     * @param[in] pwm Pulse width in microseconds (typically 1000-2000)
     * @param[in] force If true, bypass disabled_mask and emergency_stop checks
     * 
     * @details Directly sets output PWM value, bypassing min/max/trim scaling.
     *          Used for:
     *          - DO_SET_SERVO mission commands (direct PWM control)
     *          - ESC calibration sequences
     *          - Servo output testing
     *          - Pass-through modes (k_manual, k_rcin*)
     *          
     *          Sets have_pwm_mask bit for this channel, indicating output is absolute PWM
     *          rather than scaled value requiring conversion via calc_pwm().
     * 
     * @note Marks channel as having direct PWM (cleared by set_output_scaled/norm/angle/range)
     * @note Output constrained to [min, max] unless force=true
     * @warning Bypassing force safety (force=true) can spin motors when disarmed - dangerous
     */
    void set_output_pwm(uint16_t pwm, bool force = false);

    // get the output value as a pwm value
    uint16_t get_output_pwm(void) const { return output_pwm; }

    /**
     * @brief Set output as normalized value from -1 to +1
     * 
     * @param[in] value Normalized output (-1.0 = min, 0.0 = trim, +1.0 = max)
     * 
     * @details Converts normalized value to PWM using configured min/max/trim:
     *          - value < 0: Interpolates between min and trim
     *          - value == 0: Outputs trim
     *          - value > 0: Interpolates between trim and max
     *          - Honors reverse flag (swaps min/max)
     *          
     *          Used for control surfaces requiring symmetric range around neutral.
     * 
     * @note Clears have_pwm_mask (output requires calc_pwm() conversion)
     * @note Units: Normalized (-1.0 to +1.0, unitless)
     */
    void set_output_norm(float value);

    /**
     * @brief Configure channel for angle-based output scaling
     * 
     * @param[in] angle Maximum deflection angle in centidegrees (e.g., 4500 = ±45°)
     * 
     * @details Configures channel to interpret scaled values as angles in centidegrees.
     *          When set, pwm_from_scaled_value() maps:
     *          - -angle → servo_min
     *          - 0 → servo_trim
     *          - +angle → servo_max
     *          
     *          Typical for control surfaces where commanded value is angle (elevator: -30° to +30°).
     * 
     * @note Units: centidegrees (100 centidegrees = 1 degree)
     * @note Sets type_angle=true flag for scaling mode selection
     * @note Typical values: 4500 (±45°), 3000 (±30°), 6000 (±60°)
     */
    void set_angle(int16_t angle);

    /**
     * @brief Configure channel for range-based output scaling
     * 
     * @param[in] high Maximum output value (low is always 0)
     * 
     * @details Configures channel to interpret scaled values as 0-to-high range.
     *          When set, pwm_from_scaled_value() maps:
     *          - 0 → servo_min
     *          - high → servo_max
     *          
     *          Typical for throttle/motor outputs where value is magnitude (0 = off, 1000 = full).
     * 
     * @note Units: Application-specific (typically 0-1000 for throttle, 0-100 for percentage)
     * @note Sets type_angle=false flag for scaling mode selection
     * @note Common values: 1000 (throttle), 100 (percentage), 10000 (fine control)
     */
    void set_range(uint16_t high);

    // return true if the channel is reversed
    bool get_reversed(void) const {
        return reversed != 0;
    }

    // set MIN/MAX parameters
    void set_output_min(uint16_t pwm) {
        servo_min.set(pwm);
    }
    void set_output_max(uint16_t pwm) {
        servo_max.set(pwm);
    }

    // get MIN/MAX/TRIM parameters
    uint16_t get_output_min(void) const {
        return servo_min;
    }
    uint16_t get_output_max(void) const {
        return servo_max;
    }
    uint16_t get_trim(void) const {
        return servo_trim;
    }

    /**
     * @brief Check if function is a motor output
     * 
     * @param[in] function Output function to test
     * 
     * @return true if function is k_motor1 through k_motor32
     * 
     * @note Used to identify motor outputs for emergency stop and failsafe
     * @note Motors stopped immediately when emergency_stop flag set
     */
    static bool is_motor(Function function);

    /**
     * @brief Check if function should stop during emergency stop
     * 
     * @param[in] function Output function to test
     * 
     * @return true if function is dangerous and should stop (motors, some actuators)
     * 
     * @details Emergency stop (e-stop) triggered by:
     *          - Crash detection
     *          - Manual emergency stop (RC switch or GCS command)
     *          - Critical failsafe conditions
     *          
     *          Affected outputs:
     *          - All motors (k_motor1-k_motor32)
     *          - Dangerous actuators requiring immediate stop
     *          
     *          Not affected:
     *          - Control surfaces (needed for glide/recovery)
     *          - Camera/gimbal (not dangerous)
     *          - Parachute (needed for emergency deployment)
     * 
     * @warning E-stop cuts power immediately - no graceful spin-down
     */
    static bool should_e_stop(Function function);

    // return true if function is for a control surface
    static bool is_control_surface(Function function);

    // return the function of a channel
    SRV_Channel::Function get_function(void) const {
        return (SRV_Channel::Function)function.get();
    }

    // return the motor number of a channel, or -1 if not a motor
    int8_t get_motor_num(void) const;

    // set and save function for reversed. Used in upgrade of parameters in plane
    void reversed_set_and_save_ifchanged(bool r) {
        reversed.set_and_save_ifchanged(r?1:0);
    }
    
    // return true if the SERVOn_FUNCTION has been configured in
    // either storage or a defaults file. This is used for upgrade of
    // parameters in plane
    bool function_configured(void) const {
        return function.configured();
    }

    // convert a scaled value (either range or angle depending on setup) to a pwm
    uint16_t pwm_from_scaled_value(float scaled_value) const;

    // specify that small rc input changes should be ignored during passthrough
    // used by DO_SET_SERVO commands
    void ignore_small_rcin_changes() { ign_small_rcin_changes = true; }

private:
    AP_Int16 servo_min;
    AP_Int16 servo_max;
    AP_Int16 servo_trim;
    // reversal, following convention that 1 means reversed, 0 means normal
    AP_Int8 reversed;
    AP_Enum16<Function> function;

    // a pending output value as PWM
    uint16_t output_pwm;

    // true for angle output type
    bool type_angle:1;

    // set_range() or set_angle() has been called
    bool type_setup:1;

    // the hal channel number
    uint8_t ch_num;

    // high point of angle or range output
    uint16_t high_out;

    // convert a 0..range_max to a pwm
    uint16_t pwm_from_range(float scaled_value) const;

    // convert a -angle_max..angle_max to a pwm
    uint16_t pwm_from_angle(float scaled_value) const;

    // convert a scaled output to a pwm value
    void calc_pwm(float output_scaled);

    // output value based on function
    void output_ch(void);

    // setup output type and range based on function
    void aux_servo_function_setup(void);

    // return PWM for a given limit value
    uint16_t get_limit_pwm(Limit limit) const;

    // get normalised output from -1 to 1, assuming 0 at mid point of servo_min/servo_max
    float get_output_norm(void);

    // a bitmask type wide enough for NUM_SERVO_CHANNELS
    typedef uint32_t servo_mask_t;

    // mask of channels where we have a output_pwm value. Cleared when a
    // scaled value is written. 
    static servo_mask_t have_pwm_mask;

    // previous radio_in during pass-thru
    int16_t previous_radio_in;

    // specify that small rcinput changes should be ignored during passthrough
    // used by DO_SET_SERVO commands
    bool ign_small_rcin_changes;

    // if true we should ignore all imputs on this channel
    bool override_active;

    void set_override(bool b) {override_active = b;};
};

/*
  class	SRV_Channels
*/

/**
 * @class SRV_Channels
 * @brief Global servo channel manager and function-to-channel mapper
 * 
 * @details Singleton manager coordinating all output channels:
 *          - Function-to-channel mapping (builds channel_mask for each function)
 *          - Multi-channel operations (set all motors, all flaps, etc.)
 *          - Protocol backend management (Volz, SBUS, BLHeli, DShot, etc.)
 *          - Slew rate limiting for smooth servo motion
 *          - Emergency stop coordination across all outputs
 *          - PWM batching (cork/push for atomic updates)
 *          
 *          Typical usage from vehicle code:
 *          ```cpp
 *          // Set motor outputs
 *          SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, 0.75);
 *          SRV_Channels::set_output_scaled(SRV_Channel::k_motor2, 0.75);
 *          
 *          // Convert scaled values to PWM and write to hardware
 *          SRV_Channels::calc_pwm();
 *          SRV_Channels::output_ch_all();
 *          
 *          // For atomic multi-channel updates:
 *          SRV_Channels::cork();
 *          SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, 1500);
 *          SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, 1500);
 *          SRV_Channels::push();  // Outputs all at once
 *          ```
 * 
 * @note Accessed via singleton AP::srv() accessor or SRV_Channels::get_singleton()
 * @note Thread-safe: Uses semaphores for override_counter access
 * @warning Never call output methods before init() - hardware not configured
 */
class SRV_Channels {
public:
    friend class SRV_Channel;

    // constructor
    SRV_Channels(void);

    static const struct AP_Param::GroupInfo var_info[];

    // set the default function for a channel
    static void set_default_function(uint8_t chan, SRV_Channel::Function function);

    /**
     * @brief Set output value for all channels with given function
     * 
     * @param[in] function Output function type (e.g., k_motor1)
     * @param[in] value PWM value in microseconds
     * 
     * @details Sets all channels assigned to the specified function to the given PWM value.
     *          If multiple channels have the same function, all are updated.
     *          Bypasses scaling - sets absolute PWM output.
     * 
     * @note Units: microseconds (typically 1000-2000)
     * @note Called at main loop rate for motor and servo outputs
     * @see set_output_scaled() for scaled value output
     */
    static void set_output_pwm(SRV_Channel::Function function, uint16_t value);

    /**
     * @brief Set output value for specific channel by channel number
     * 
     * @param[in] chan Physical output channel number (0-31)
     * @param[in] value PWM value in microseconds
     * 
     * @details Directly sets a physical channel's PWM output regardless of function assignment.
     *          Used for:
     *          - Testing specific hardware outputs
     *          - Calibration sequences
     *          - Direct channel control bypassing function mapping
     * 
     * @note Units: microseconds (typically 1000-2000)
     * @warning Bypasses function mapping - ensure channel number is correct
     * @warning Can control unintended outputs if channel number wrong
     */
    static void set_output_pwm_chan(uint8_t chan, uint16_t value);

    // get output value for a specific channel as a pwm value
    static bool get_output_pwm_chan(uint8_t chan, uint16_t &value);
    
    // set output value for a specific function channel as a pwm value for specified override time in ms
    static void set_output_pwm_chan_timeout(uint8_t chan, uint16_t value, uint16_t timeout_ms);

    /**
     * @brief Set output value for function as scaled value
     * 
     * @param[in] function Output function type (e.g., k_motor1, k_aileron)
     * @param[in] value Scaled output value (interpretation depends on channel configuration)
     * 
     * @details Sets scaled value for all channels with this function. Value interpretation:
     *          - Angle mode (control surfaces): value in centidegrees (e.g., -4500 to +4500)
     *          - Range mode (motors/throttle): value from 0 to high_out (e.g., 0-1000)
     *          
     *          Must be followed by calc_pwm() to convert scaled values to PWM microseconds.
     *          
     *          Typical workflow:
     *          1. set_output_scaled() for all functions
     *          2. calc_pwm() to convert all scaled values
     *          3. output_ch_all() to write PWM to hardware
     * 
     * @note Does NOT immediately output to hardware - requires calc_pwm() call
     * @note Units depend on channel type (centidegrees for angle, 0-range for range)
     * @see calc_pwm() to convert scaled values to PWM
     * @see set_angle() and set_range() for configuring scaling mode
     */
    static void set_output_scaled(SRV_Channel::Function function, float value);

    // get scaled output for the given function type.
    static float get_output_scaled(SRV_Channel::Function function);

    // get slew limited scaled output for the given function type
    static float get_slew_limited_output_scaled(SRV_Channel::Function function);

    // get pwm output for the first channel of the given function type.
    static bool get_output_pwm(SRV_Channel::Function function, uint16_t &value);

    // get normalised output (-1 to 1 with 0 at mid point of servo_min/servo_max)
    // Value is taken from pwm value.  Returns zero on error.
    static float get_output_norm(SRV_Channel::Function function);

    // set normalised output (-1 to 1 with 0 at mid point of servo_min/servo_max) for the given function
    static void set_output_norm(SRV_Channel::Function function, float value);

    /**
     * @brief Get bitmask of physical channels assigned to a function
     * 
     * @param[in] function Output function type (e.g., k_motor1, k_flap)
     * 
     * @return Bitmask with bits set for each channel having this function (bit 0 = channel 0)
     * 
     * @details Used to:
     *          - Identify which physical channels output a given function
     *          - Set hardware features (frequency, mode) for function's channels
     *          - Disable/enable groups of channels
     *          
     *          Example: If channels 0 and 2 assigned to k_aileron, returns 0x00000005 (bits 0,2 set)
     * 
     * @note Multiple channels can share same function (redundancy or mixing)
     * @note Returns 0 if function not assigned to any channel
     */
    static uint32_t get_output_channel_mask(SRV_Channel::Function function);

    /**
     * @brief Limit output slew rate for smooth servo motion
     * 
     * @param[in] function Output function type to limit
     * @param[in] slew_rate Maximum change rate in percent per second
     * @param[in] range Full scale range for percentage calculation
     * @param[in] dt Time delta since last update in seconds
     * 
     * @details Gradually transitions output to avoid:
     *          - Abrupt servo motion causing mechanical stress
     *          - Sudden motor power changes causing instability
     *          - High current draw from rapid ESC changes
     *          
     *          Algorithm: Limits scaled_output change to (slew_rate * range * dt / 100) per call
     *          
     *          Example: slew_rate=50, range=1000, dt=0.02 → max change = 10 per update
     * 
     * @note Applied before calc_pwm() conversion
     * @note Only affects scaled outputs (not direct PWM)
     * @note slew_rate=0 disables slew limiting
     * @note Units: slew_rate in %/s, range in native units, dt in seconds
     */
    static void set_slew_rate(SRV_Channel::Function function, float slew_rate, uint16_t range, float dt);

    // update channels last_scaled_output to match value
    static void set_slew_last_scaled_output(SRV_Channel::Function function, float value);

    /**
     * @brief Output PWM values to hardware for all channels
     * 
     * @details Writes pending PWM values to hal.rcout hardware for all configured channels.
     *          Respects:
     *          - disabled_mask (channels disabled by protocol backends like BLHeli)
     *          - emergency_stop flag (stops motors immediately)
     *          - invalid_mask (GPIO and other non-PWM functions)
     *          
     *          Called at main loop rate after calc_pwm() to send outputs to ESCs/servos.
     * 
     * @note Must call calc_pwm() first to convert scaled values to PWM
     * @note Typically called at 50-400Hz depending on vehicle type
     * @warning Outputs immediately - ensure calc_pwm() called with correct values
     * @see calc_pwm() to convert scaled values before output
     */
    static void output_ch_all(void);

    // setup output ESC scaling based on a channels MIN/MAX
    void set_esc_scaling_for(SRV_Channel::Function function);

    // return true when auto_trim enabled
    bool auto_trim_enabled(void) const { return auto_trim; }

    // adjust trim of a channel by a small increment
    void adjust_trim(SRV_Channel::Function function, float v);

    // set MIN/MAX parameters for a function
    static void set_output_min_max(SRV_Channel::Function function, uint16_t min_pwm, uint16_t max_pwm);

    // set MIN/MAX parameter defaults for a function
    static void set_output_min_max_defaults(SRV_Channel::Function function, uint16_t min_pwm, uint16_t max_pwm);

    // Save MIN/MAX/REVERSED parameters for a function
    static void save_output_min_max(SRV_Channel::Function function, uint16_t min_pwm, uint16_t max_pwm);

    // save trims
    void save_trim(void);

    // setup IO failsafe for all channels to trim
    static void setup_failsafe_trim_all_non_motors(void);

    // set output for all channels matching the given function type, allow radio_trim to center servo
    static void set_output_pwm_trimmed(SRV_Channel::Function function, int16_t value);

    // set and save the trim for a function channel to the output value
    static void set_trim_to_servo_out_for(SRV_Channel::Function function);

    // set the trim for a function channel to min of the channel honnoring reverse unless ignore_reversed is true
    static void set_trim_to_min_for(SRV_Channel::Function function, bool ignore_reversed = false);

    // set the trim for a function channel to given pwm
    static void set_trim_to_pwm_for(SRV_Channel::Function function, int16_t pwm);

    // set output to min value
    static void set_output_to_min(SRV_Channel::Function function);

    // set output to max value
    static void set_output_to_max(SRV_Channel::Function function);

    // set output to trim value
    static void set_output_to_trim(SRV_Channel::Function function);

    // copy radio_in to servo out
    static void copy_radio_in_out(SRV_Channel::Function function, bool do_input_output=false);

    // copy radio_in to servo_out by channel mask
    static void copy_radio_in_out_mask(uint32_t mask);

    // setup failsafe for an auxiliary channel function, by pwm
    static void set_failsafe_pwm(SRV_Channel::Function function, uint16_t pwm);

    /**
     * @brief Configure failsafe output limit for a function
     * 
     * @param[in] function Output function type
     * @param[in] limit Failsafe limit value (TRIM, MIN, MAX, or ZERO_PWM)
     * 
     * @details Sets the output value that will be used during RC failsafe conditions:
     *          - TRIM: Neutral position (typical for control surfaces)
     *          - MIN: Minimum PWM (typical for motors = off/disarmed)
     *          - MAX: Maximum PWM (rarely used)
     *          - ZERO_PWM: Hardware disable (motor off, no signal)
     *          
     *          Applied when:
     *          - RC signal lost
     *          - GCS link lost (in some modes)
     *          - Configured failsafe action triggered
     * 
     * @note Configured during vehicle init() based on function type
     * @note Motor functions typically use MIN or ZERO_PWM
     * @note Control surfaces typically use TRIM to maintain stability
     * @warning SAFETY-CRITICAL: Incorrect failsafe can cause crash
     */
    static void set_failsafe_limit(SRV_Channel::Function function, SRV_Channel::Limit limit);

    /**
     * @brief Immediately set output to a fixed limit value
     * 
     * @param[in] function Output function type
     * @param[in] limit Limit value to output (TRIM, MIN, MAX, or ZERO_PWM)
     * 
     * @details Forces output to configured limit immediately:
     *          - TRIM: Output servo_trim (neutral/center)
     *          - MIN: Output servo_min (minimum limit, honors reverse)
     *          - MAX: Output servo_max (maximum limit, honors reverse)
     *          - ZERO_PWM: Output 0μs (hardware disables output)
     *          
     *          Used for:
     *          - Pre-arm centering of control surfaces
     *          - Motor disarm sequences
     *          - Testing full range of motion
     *          - Manual servo positioning
     * 
     * @note Takes effect immediately on next output_ch_all() call
     * @note MIN/MAX honor reversed flag (swapped if reversed)
     * @see set_failsafe_limit() to configure failsafe behavior
     */
    static void set_output_limit(SRV_Channel::Function function, SRV_Channel::Limit limit);

    /**
     * @brief Check if a function is assigned to any output channel
     * 
     * @param[in] function Output function type to check
     * 
     * @return true if function assigned to at least one channel, false otherwise
     * 
     * @details Used by vehicle code to:
     *          - Detect available features (e.g., has gimbal if k_mount_pan assigned)
     *          - Skip processing for unassigned functions
     *          - Validate configuration before operations
     *          
     *          Checks internal function_mask bitmask built during init().
     * 
     * @note Returns false for k_none and invalid functions
     * @note Updated when parameters change via update_aux_servo_function()
     */
    static bool function_assigned(SRV_Channel::Function function);

    // set a servo_out value, and angle range, then calc_pwm
    static void move_servo(SRV_Channel::Function function,
                           int16_t value, int16_t angle_min, int16_t angle_max);

    // assign and enable auxiliary channels
    void enable_aux_servos(void);

    // enable channels by mask
    static void enable_by_mask(uint32_t mask);

    // return the current function for a channel
    static SRV_Channel::Function channel_function(uint8_t channel);

    /**
     * @brief Rebuild function-to-channel mapping after parameter changes
     * 
     * @details Scans all channels and rebuilds internal mapping structures:
     *          - function_mask: Bitmask of which functions are assigned
     *          - channel_mask per function: Which channels output each function
     *          - invalid_mask: Channels configured as GPIO or other non-PWM
     *          
     *          Called:
     *          - During init() to build initial mapping
     *          - When SERVOn_FUNCTION parameters change
     *          - After parameter reload from storage
     *          
     *          Enables function_assigned() and get_channel_for() lookups.
     * 
     * @note Automatically called when parameters change via AP_Param system
     * @note Manual call needed if programmatically modifying function assignments
     */
    static void update_aux_servo_function(void);

    /**
     * @brief Set default channel assignment for a function if not configured
     * 
     * @param[in] function Output function to assign
     * @param[in] channel Hardware channel number (0-31)
     * 
     * @return true if default was set, false if channel already configured
     * 
     * @details Sets SERVOn_FUNCTION parameter default value only if parameter
     *          is not already configured in storage or defaults file. Used for:
     *          - Vehicle-specific defaults (Copter assigns k_motor1-8 by frame type)
     *          - Board-specific defaults (hwdef files pre-assign common peripherals)
     *          - Feature-specific defaults (camera trigger on specific channel)
     *          
     *          Does NOT override user configuration.
     * 
     * @note Only affects unconfigured parameters (function_configured() == false)
     * @note Call before enable_aux_servos() to ensure defaults take effect
     */
    static bool set_aux_channel_default(SRV_Channel::Function function, uint8_t channel);

    /**
     * @brief Find hardware channel number for a function
     * 
     * @param[in]  function Output function type to find
     * @param[out] chan     Hardware channel number (0-31) if found
     * 
     * @return true if function assigned to at least one channel, false otherwise
     * 
     * @details Searches for first channel assigned to function and returns channel number.
     *          Used when:
     *          - Need channel number for hardware configuration (frequency, mode)
     *          - Interfacing with HAL layer (hal.rcout)
     *          - Channel-indexed operations
     *          
     *          If function assigned to multiple channels, returns lowest number.
     * 
     * @note Returns false if function not assigned to any channel
     * @note Use get_channel_for() to get SRV_Channel object instead of number
     */
    static bool find_channel(SRV_Channel::Function function, uint8_t &chan);

    /**
     * @brief Get first channel object assigned to a function
     * 
     * @param[in] function Output function type to find
     * 
     * @return Pointer to first SRV_Channel with this function, or nullptr if not assigned
     * 
     * @details Searches channel array for first match with given function.
     *          Used to:
     *          - Access channel-specific parameters (min/max/trim/reverse)
     *          - Modify individual channel configuration
     *          - Read current output state
     *          
     *          If multiple channels have same function, returns first match only.
     * 
     * @note Returns nullptr if function not assigned or invalid
     * @note Use get_output_channel_mask() to find all channels with function
     * @see find_channel() for channel number instead of object
     */
    static SRV_Channel *get_channel_for(SRV_Channel::Function function);

    /**
     * @brief Configure angle-based scaling for all channels with function
     * 
     * @param[in] function Output function type (typically control surface)
     * @param[in] angle Maximum deflection angle in centidegrees
     * 
     * @details Calls set_angle() on each channel assigned to function.
     *          Configures channels to interpret scaled values as angles:
     *          - -angle → servo_min PWM
     *          - 0 → servo_trim PWM
     *          - +angle → servo_max PWM
     *          
     *          Typical for control surfaces where commanded value is angle.
     * 
     * @note Units: centidegrees (4500 = ±45°, 3000 = ±30°)
     * @note Applied to all channels with this function
     * @see SRV_Channel::set_angle() for per-channel version
     */
    static void set_angle(SRV_Channel::Function function, uint16_t angle);

    /**
     * @brief Configure range-based scaling for all channels with function
     * 
     * @param[in] function Output function type (typically motor/throttle)
     * @param[in] range Maximum output value (low is always 0)
     * 
     * @details Calls set_range() on each channel assigned to function.
     *          Configures channels to interpret scaled values as 0-to-range:
     *          - 0 → servo_min PWM
     *          - range → servo_max PWM
     *          
     *          Typical for throttle/motor outputs where value is magnitude.
     * 
     * @note Units: Application-specific (1000 for throttle, 100 for percentage)
     * @note Applied to all channels with this function
     * @see SRV_Channel::set_range() for per-channel version
     */
    static void set_range(SRV_Channel::Function function, uint16_t range);

    /**
     * @brief Set PWM output frequency for all channels with given function
     * 
     * @param[in] function Output function type
     * @param[in] frequency Output update rate in Hz
     * 
     * @details Configures hardware output refresh rate for all channels assigned to function:
     *          - 50Hz: Standard analog servos
     *          - 400Hz: Digital servos, OneShot ESCs
     *          - 490Hz: High-speed ESCs
     *          - 1000Hz+: DShot protocols (actual rate depends on protocol)
     *          
     *          Applied to hal.rcout hardware timer configuration.
     *          
     *          Constraints:
     *          - Channels on same hardware timer must use same frequency
     *          - Frequency limited by hardware capabilities
     *          - Some protocols (DShot) ignore frequency setting
     * 
     * @note Call during init() or vehicle setup
     * @note Hardware may group channels - setting one affects others
     * @note Units: Hz (cycles per second)
     * @warning Excessive frequency (>500Hz) may not work with standard servos
     */
    static void set_rc_frequency(SRV_Channel::Function function, uint16_t frequency);

    // control pass-thru of channels
    void disable_passthrough(bool disable) {
        disabled_passthrough = disable;
    }

    // constrain to output min/max for function
    static void constrain_pwm(SRV_Channel::Function function);

    /**
     * @brief Convert all scaled output values to PWM microseconds
     * 
     * @details For each channel with scaled output (not direct PWM):
     *          1. Applies angle or range scaling based on type_angle flag
     *          2. Maps scaled value to [servo_min, servo_max] range via servo_trim
     *          3. Honors reversed flag (swaps min/max if reversed)
     *          4. Constrains to configured limits
     *          5. Stores result in output_pwm for output_ch_all()
     *          
     *          Skips channels with direct PWM set (have_pwm_mask bit set).
     *          
     *          Typical calling sequence:
     *          ```cpp
     *          SRV_Channels::set_output_scaled(k_motor1, 750);  // 0-1000 range
     *          SRV_Channels::set_output_scaled(k_aileron, -2000); // ±4500 centidegrees
     *          SRV_Channels::calc_pwm();  // Convert both to PWM
     *          SRV_Channels::output_ch_all();  // Write to hardware
     *          ```
     * 
     * @note Must be called after set_output_scaled() and before output_ch_all()
     * @note Typically called at main loop rate (50-400Hz)
     * @see set_output_scaled() to set scaled values
     * @see output_ch_all() to write PWM to hardware
     */
    static void calc_pwm(void);

    // return the ESC type for dshot commands
    static AP_HAL::RCOutput::DshotEscType get_dshot_esc_type() { return AP_HAL::RCOutput::DshotEscType(_singleton->dshot_esc_type.get()); }

    static uint8_t get_dshot_rate() { return _singleton->dshot_rate.get(); }

    static uint32_t get_rc_fs_mask() { return _singleton->rc_fs_mask.get(); }

    static SRV_Channel *srv_channel(uint8_t i) {
#if NUM_SERVO_CHANNELS > 0
        return i<NUM_SERVO_CHANNELS?&channels[i]:nullptr;
#else
        return nullptr;
#endif
    }

    // SERVO* parameters
    static void upgrade_parameters(void);

    // given a zero-based motor channel, return the k_motor function for that channel
    static SRV_Channel::Function get_motor_function(uint8_t channel) {
        if (channel < 8) {
            return SRV_Channel::Function(SRV_Channel::k_motor1+channel);
        }
        if (channel < 12) {
            return SRV_Channel::Function((SRV_Channel::k_motor9+(channel-8)));
        }
        return SRV_Channel::Function((SRV_Channel::k_motor13+(channel-12)));
    }

    /**
     * @brief Begin atomic multi-channel update batch
     * 
     * @details Buffers PWM outputs without writing to hardware until push() called.
     *          Used for:
     *          - Synchronizing multiple motor outputs (prevents timing glitches)
     *          - Atomic updates of related control surfaces
     *          - Reducing USB/serial latency in protocol backends
     *          
     *          Must be paired with push() to complete batch update.
     * 
     * @note Cork/push can be nested - inner cork/push ignored
     * @note Outputs buffered until matching push() call
     * @see push() to complete batch and write to hardware
     */
    void cork();

    /**
     * @brief Complete atomic multi-channel update batch
     * 
     * @details Writes all buffered PWM outputs to hardware atomically after cork().
     *          Ensures all channels update simultaneously to prevent:
     *          - Motor timing asymmetry causing instability
     *          - Control surface desynchronization
     *          - Protocol backend timing issues
     * 
     * @note Must be preceded by cork() call
     * @note Updates all channels in single hardware transaction where supported
     * @see cork() to begin batch update
     */
    void push();

    // disable PWM output to a set of channels given by a mask. This is used by the AP_BLHeli code
    static void set_disabled_channel_mask(uint32_t mask) { disabled_mask = mask; }
    static uint32_t get_disabled_channel_mask() { return disabled_mask; }

    /**
     * @brief Configure channels using digital protocols (DShot, etc.)
     * 
     * @param[in] dig_mask Bitmask of channels using digital output protocols
     * @param[in] rev_mask Bitmask of digital channels supporting bidirectional operation
     * 
     * @details Digital protocols use special timing/encoding vs PWM:
     *          - DShot: Digital ESC protocol with telemetry and commands
     *          - OneShot125/42: High-speed ESC protocols
     *          - ProfiLED: Addressable LED protocol
     *          
     *          dig_mask: Channels using any digital protocol (not analog PWM)
     *          rev_mask: Subset supporting thrust reversal (DShot-3D)
     *          
     *          Used by vehicle code to:
     *          - Enable digital-only features (ESC telemetry, commands)
     *          - Skip PWM-specific configuration
     *          - Configure bidirectional ESCs (reversible props)
     * 
     * @note Called during init() after protocol detection
     * @note rev_mask must be subset of dig_mask
     * @note Digital channels may ignore frequency and PWM limit settings
     * @see have_digital_outputs() to check if any digital outputs configured
     */
    static void set_digital_outputs(uint32_t dig_mask, uint32_t rev_mask);

    // return true if all of the outputs in mask are digital
    static bool have_digital_outputs(uint32_t mask) { return mask != 0 && (mask & digital_mask) == mask; }

    // return true if any of the outputs are digital
    static bool have_digital_outputs() { return digital_mask != 0; }

    /**
     * @brief Enable or disable emergency stop for all dangerous outputs
     * 
     * @param[in] state true to engage emergency stop, false to release
     * 
     * @details Emergency stop immediately stops all outputs identified by should_e_stop():
     *          - All motors (k_motor1-k_motor32)
     *          - Dangerous actuators
     *          
     *          Does NOT stop:
     *          - Control surfaces (needed for glide/recovery)
     *          - Parachute (needed for emergency deployment)
     *          - Camera/gimbal
     *          
     *          Triggered by:
     *          - Crash detection
     *          - Manual e-stop command (RC switch or GCS)
     *          - Critical failsafe conditions
     *          - Geofence breach with e-stop action
     * 
     * @note E-stop state persists until explicitly cleared
     * @note Outputs stopped immediately with no spin-down
     * @warning SAFETY-CRITICAL: Cuts motor power instantly - vehicle will fall/crash
     * @see should_e_stop() for list of affected outputs
     */
    static void set_emergency_stop(bool state);

    /**
     * @brief Check if emergency stop is currently engaged
     * 
     * @return true if emergency stop active, false otherwise
     * 
     * @note Used by vehicle code to determine if e-stop recovery needed
     */
    static bool get_emergency_stop() { return emergency_stop;}

    // singleton for Lua
    static SRV_Channels *get_singleton(void) {
        return _singleton;
    }

    static void zero_rc_outputs();

    /**
     * @brief Initialize servo channel system and configure hardware outputs
     * 
     * @param[in] motor_mask Bitmask of channels used for motors (for special handling)
     * @param[in] mode Output mode (PWM, OneShot, DShot, etc.)
     * 
     * @details Initializes the servo output system:
     *          1. Configures hal.rcout hardware backend
     *          2. Sets up protocol backends (Volz, SBUS, BLHeli, FETtec OneWire)
     *          3. Loads parameters for all channels
     *          4. Builds function-to-channel mapping
     *          5. Configures output frequencies and modes
     *          6. Applies GPIO mask for digital I/O pins
     *          
     *          Must be called during vehicle initialization before any output operations.
     * 
     * @note Call exactly once during startup
     * @note motor_mask used for special motor handling (e-stop, failsafe)
     * @note mode determines ESC protocol (PWM=standard, OneShot/DShot=high-speed)
     * @warning Output methods will fail/crash if init() not called first
     * @see set_rc_frequency() to configure per-function output rates
     */
    void init(uint32_t motor_mask = 0, AP_HAL::RCOutput::output_mode mode = AP_HAL::RCOutput::MODE_PWM_NONE);

    // return true if a channel is set to type GPIO
    static bool is_GPIO(uint8_t channel);

    // return true if a channel is set to type alarm
    static bool is_alarm(uint8_t channel) {
        return channel_function(channel) == SRV_Channel::k_alarm;
    }

    // return true if a channel is set to type alarm inverted
    static bool is_alarm_inverted(uint8_t channel) {
        return channel_function(channel) == SRV_Channel::k_alarm_inverted;
    }

    // return true if 32 channels are enabled
    static bool have_32_channels() {
#if NUM_SERVO_CHANNELS >= 17
        return _singleton->enable_32_channels.get() > 0;
#else
        return false;
#endif
    }

private:

    static bool disabled_passthrough;

    SRV_Channel::servo_mask_t trimmed_mask;

    static Bitmask<SRV_Channel::k_nr_aux_servo_functions> function_mask;
    static bool initialised;

    // this static arrangement is to avoid having static objects in AP_Param tables
    static SRV_Channel *channels;
    static SRV_Channels *_singleton;

#if AP_VOLZ_ENABLED
    // support for Volz protocol
    AP_Volz_Protocol volz;
#endif

#if AP_SBUSOUTPUT_ENABLED
    // support for SBUS protocol
    AP_SBusOut sbus;
#endif

#if AP_ROBOTISSERVO_ENABLED
    // support for Robotis servo protocol
    AP_RobotisServo robotis;
#endif

#if HAL_SUPPORT_RCOUT_SERIAL
    // support for BLHeli protocol
    AP_BLHeli blheli;
#endif

#if AP_FETTEC_ONEWIRE_ENABLED
    AP_FETtecOneWire fetteconwire;
#endif  // AP_FETTEC_ONEWIRE_ENABLED

    // mask of disabled channels
    static uint32_t disabled_mask;

    // mask of outputs which use a digital output protocol, not
    // PWM (eg. DShot)
    static uint32_t digital_mask;
    
    // mask of outputs which are digitally reversible (eg. DShot-3D)
    static uint32_t reversible_mask;

    // mask of channels with invalid funtions, eg GPIO
    static uint32_t invalid_mask;

    SRV_Channel obj_channels[NUM_SERVO_CHANNELS];

    // override loop counter
    static uint16_t override_counter[NUM_SERVO_CHANNELS];

    static struct srv_function {
        // mask of what channels this applies to
        SRV_Channel::servo_mask_t channel_mask;

        // scaled output for this function
        float output_scaled;
    } functions[SRV_Channel::k_nr_aux_servo_functions];

    AP_Int8 auto_trim;
    AP_Int16 default_rate;
    AP_Int8 dshot_rate;
    AP_Int8 dshot_esc_type;
    AP_Int32 gpio_mask;
    AP_Int32 rc_fs_mask;
#if NUM_SERVO_CHANNELS >= 17
    AP_Int8 enable_32_channels;
#endif

    // return true if passthrough is disabled
    static bool passthrough_disabled(void) {
        return disabled_passthrough;
    }

    static bool emergency_stop;

    // linked list for slew rate handling
    struct slew_list {
        slew_list(SRV_Channel::Function _func) : func(_func) {};
        const SRV_Channel::Function func;
        float last_scaled_output;
        float max_change;
        slew_list * next;
    };
    static slew_list *_slew;

    // semaphore for multi-thread use of override_counter array
    HAL_Semaphore override_counter_sem;
};

namespace AP {
    SRV_Channels &srv();
};
