/**
 * @file Fins.cpp
 * @brief Fin actuation system implementation for blimp lighter-than-air vehicle
 * 
 * @details This file implements the oscillating fin control system unique to blimp
 *          propulsion. Unlike traditional rotorcraft or fixed-wing control surfaces,
 *          blimp fins use sinusoidal oscillations for thrust generation and directional
 *          control. The system parameterizes fin behavior via AP_Param (frequency, turbo mode)
 *          and maps per-fin amplitude/offset factors to SRV_Channel servo outputs.
 *          
 *          Key features:
 *          - Sinusoidal fin actuation at configurable frequency (FREQ_HZ parameter)
 *          - Four-fin configuration (back, front, right, left) for 4-DOF control
 *          - Amplitude and offset mapping for right/front/down/yaw control axes
 *          - Turbo mode for frequency doubling under high-offset conditions
 *          - Integration with ArduPilot SRV_Channel servo output system
 *          
 *          Control flow: Control inputs → Amplitude/offset calculation → Sine wave
 *          generation → Servo output via SRV_Channels
 *          
 * Source: Blimp/Fins.cpp
 */

#include "Blimp.h"

#include <SRV_Channel/SRV_Channel.h>

// This is the scale used for RC inputs so that they can be scaled to the float point values used in the sine wave code.
#define FIN_SCALE_MAX 1000

/**
 * @brief Fins parameter group definition for AP_Param system
 * 
 * @details Defines the persistent configuration parameters for the fin actuation system.
 *          These parameters control the oscillation characteristics of the fins:
 *          
 *          - FREQ_HZ: Base oscillation frequency in Hertz (1-10 Hz, default 3 Hz)
 *            Controls how fast fins oscillate during normal operation. Higher frequencies
 *            generate more thrust but may increase power consumption and wear.
 *          
 *          - TURBO_MODE: Enables frequency doubling (0=disabled, 1=enabled, default 0)
 *            When enabled and offset exceeds 0.4 with amplitude below 0.6, fins oscillate
 *            at double frequency (2x FREQ_HZ) for enhanced directional control authority.
 *          
 *          This is the 2nd parameter group for the Fins class, registered with AP_Param
 *          for persistent storage and ground control station configuration access.
 */
const AP_Param::GroupInfo Fins::var_info[] = {

    // @Param: FREQ_HZ
    // @DisplayName: Fins frequency
    // @Description: This is the oscillation frequency of the fins
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("FREQ_HZ", 1, Fins, freq_hz, 3),

    // @Param: TURBO_MODE
    // @DisplayName: Enable turbo mode
    // @Description: Enables double speed on high offset.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TURBO_MODE", 2, Fins, turbo_mode, 0),

    AP_GROUPEND
};

/**
 * @brief Constructor for Fins object
 * 
 * @details Initializes the fin actuation system with the scheduler loop rate.
 *          Sets up default parameter values from var_info table via AP_Param.
 *          The loop rate determines the frequency at which output() is called
 *          and affects the temporal resolution of sine wave generation.
 * 
 * @param[in] loop_rate Scheduler loop rate in Hz (typically 50-400 Hz for blimp)
 *                      Used for timing calculations in sinusoidal output generation
 * 
 * @note Called during Blimp vehicle initialization before setup_fins()
 */
Fins::Fins(uint16_t loop_rate) :
    _loop_rate(loop_rate)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/**
 * @brief Initialize fin configuration and servo channel mapping
 * 
 * @details Configures the four-fin layout for blimp control by setting amplitude and
 *          offset factors for each control axis (right, front, down, yaw). Each fin
 *          contributes to vehicle motion based on its orientation and mixing factors.
 *          
 *          Fin configuration:
 *          - Fin 0 (Back):  Front amplitude=1.0, down amplitude/offset=0.5
 *          - Fin 1 (Front): Front amplitude=-1.0, down amplitude/offset=0.5
 *          - Fin 2 (Right): Right amplitude=-1.0, yaw amplitude/offset=0.5
 *          - Fin 3 (Left):  Right amplitude=1.0, yaw amplitude=0.5, yaw offset=-0.5
 *          
 *          The amplitude factors determine oscillation magnitude for thrust generation,
 *          while offset factors bias the fin position for directional control. Positive
 *          front moves vehicle forward, positive right moves vehicle right, positive
 *          down descends, positive yaw rotates clockwise (body frame).
 *          
 *          Also configures SRV_Channel angle ranges to FIN_SCALE_MAX (1000) for all
 *          four motor functions, establishing the PWM output scaling for servos.
 * 
 * @note Must be called after Fins construction and before output() calls
 * @note Fin mixing factors are hard-coded for standard blimp fin geometry
 * 
 * @see add_fin() for amplitude/offset factor assignment per fin
 * @see SRV_Channels::set_angle() for servo range configuration
 */
void Fins::setup_fins()
{
    //fin   #   r   f   d     y,    r   f     d     y               right, front, down, yaw for amplitude then for offset
    add_fin(0,  0,  1, 0.5,   0,    0,  0,  0.5,    0); //Back
    add_fin(1,  0, -1, 0.5,   0,    0,  0,  0.5,    0); //Front
    add_fin(2, -1,  0,   0, 0.5,    0,  0,    0,  0.5); //Right
    add_fin(3,  1,  0,   0, 0.5,    0,  0,    0, -0.5); //Left

    SRV_Channels::set_angle(SRV_Channel::k_motor1, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, FIN_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, FIN_SCALE_MAX);
}

/**
 * @brief Configure amplitude and offset mixing factors for a single fin
 * 
 * @details Assigns the control mixing factors that determine how each control axis
 *          (right, front, down, yaw) affects the specified fin's oscillation amplitude
 *          and position offset. These factors implement the control allocation matrix
 *          that maps 4-DOF control inputs to 4-fin outputs.
 *          
 *          Amplitude factors: Control oscillation magnitude for thrust generation
 *          - Positive values: Fin contributes thrust in positive axis direction
 *          - Negative values: Fin contributes thrust in negative axis direction
 *          - Zero: Fin does not contribute to that axis
 *          
 *          Offset factors: Control steady-state fin position bias
 *          - Positive values: Bias fin position in positive direction
 *          - Negative values: Bias fin position in negative direction
 *          - Used for directional control without changing oscillation amplitude
 *          
 *          The combination of amplitude and offset allows independent control of
 *          thrust magnitude and directional bias for each fin.
 * 
 * @param[in] fin_num        Fin index (0-3: back, front, right, left)
 * @param[in] right_amp_fac  Right axis amplitude factor (-1.0 to 1.0)
 * @param[in] front_amp_fac  Front axis amplitude factor (-1.0 to 1.0)
 * @param[in] down_amp_fac   Down axis amplitude factor (-1.0 to 1.0)
 * @param[in] yaw_amp_fac    Yaw axis amplitude factor (-1.0 to 1.0)
 * @param[in] right_off_fac  Right axis offset factor (-1.0 to 1.0)
 * @param[in] front_off_fac  Front axis offset factor (-1.0 to 1.0)
 * @param[in] down_off_fac   Down axis offset factor (-1.0 to 1.0)
 * @param[in] yaw_off_fac    Yaw axis offset factor (-1.0 to 1.0)
 * 
 * @note If fin_num is outside valid range (0 to NUM_FINS-1), function returns without
 *       modifying any factors (silent error handling for robustness)
 * @note Factors are unitless mixing gains applied to normalized control inputs (-1 to 1)
 * 
 * @see setup_fins() for standard fin configuration
 */
void Fins::add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float down_amp_fac, float yaw_amp_fac,
                   float right_off_fac, float front_off_fac, float down_off_fac, float yaw_off_fac)
{

    // ensure valid fin number is provided
    if (fin_num >= 0 && fin_num < NUM_FINS) {

        // set amplitude factors
        _right_amp_factor[fin_num] = right_amp_fac;
        _front_amp_factor[fin_num] = front_amp_fac;
        _down_amp_factor[fin_num] = down_amp_fac;
        _yaw_amp_factor[fin_num] = yaw_amp_fac;

        // set offset factors
        _right_off_factor[fin_num] = right_off_fac;
        _front_off_factor[fin_num] = front_off_fac;
        _down_off_factor[fin_num] = down_off_fac;
        _yaw_off_factor[fin_num] = yaw_off_fac;
    }
}

/**
 * @brief Generate sinusoidal fin outputs and update servo positions
 * 
 * @details Main output function called at scheduler loop rate to compute and apply
 *          sinusoidal fin positions. Implements the core blimp control algorithm:
 *          
 *          Algorithm flow:
 *          1. Check arming state - zero all outputs if disarmed for safety
 *          2. Log raw control inputs (right_out, front_out, down_out, yaw_out)
 *          3. Constrain control inputs to [-1, 1] range (logged before constraining
 *             to reveal tuning issues causing overshoots)
 *          4. Get current time for sine wave phase calculation
 *          5. For each fin (B,F,R,L = 0,1,2,3):
 *             a. Calculate amplitude by summing positive contributions from each axis
 *                (uses fmaxf for right/front, fabsf for down/yaw to ensure positive)
 *             b. Calculate offset by weighted sum of all control axes
 *             c. Average offset by number of contributing axes (prevent saturation)
 *             d. Limit amplitude if amplitude+offset exceeds 1.0 (prevent clipping)
 *             e. Apply turbo mode: double frequency if amplitude≤0.6 and |offset|≥0.4
 *             f. Generate sinusoidal position: pos = amp*cos(2π*freq_hz*freq*time) + offset
 *             g. Output scaled position to servo via SRV_Channels
 *          6. Log final amplitude and offset values
 *          
 *          Turbo mode rationale: When high offset (directional bias) is combined with
 *          low amplitude (reduced thrust), doubling frequency maintains thrust while
 *          preserving directional control authority.
 *          
 *          Frequency calculation: freq_hz parameter (default 3 Hz) multiplied by _freq
 *          multiplier (1 or 2 for turbo) determines oscillation rate. Phase advances
 *          continuously based on real-time clock (AP_HAL::micros).
 * 
 * @note Called at main loop rate (stored in _loop_rate, typically 50-400 Hz)
 * @note Disarming immediately stops all fin motion for safety
 * @note Logging occurs before and after amplitude/offset calculation for debugging
 * @note Uses cosine function (not sine) for phase, starting at maximum deflection
 * 
 * @warning Control inputs (right_out, front_out, down_out, yaw_out) must be set by
 *          calling code before output() - this function reads but does not compute them
 * 
 * @see Fins::output_min() for safe output during initialization
 * @see setup_fins() for fin mixing factor configuration
 */
void Fins::output()
{
    if (!_armed) {
        // set everything to zero so fins stop moving
        right_out = 0;
        front_out = 0;
        down_out  = 0;
        yaw_out   = 0;
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINI(right_out, front_out, down_out, yaw_out);
#endif

    //Constrain after logging so as to still show when sub-optimal tuning is causing massive overshoots.
    right_out = constrain_float(right_out, -1, 1);
    front_out = constrain_float(front_out, -1, 1);
    down_out = constrain_float(down_out, -1, 1);
    yaw_out = constrain_float(yaw_out, -1, 1);

    _time = AP_HAL::micros() * 1.0e-6;

    for (int8_t i=0; i<NUM_FINS; i++) {
        _amp[i] =  fmaxf(0,_right_amp_factor[i]*right_out) + fmaxf(0,_front_amp_factor[i]*front_out) +
                   fabsf(_down_amp_factor[i]*down_out) + fabsf(_yaw_amp_factor[i]*yaw_out);
        _off[i] = _right_off_factor[i]*right_out + _front_off_factor[i]*front_out +
                  _down_off_factor[i]*down_out + _yaw_off_factor[i]*yaw_out;
        _freq[i] = 1;

        _num_added = 0;
        if (fmaxf(0,_right_amp_factor[i]*right_out) > 0.0f) {
            _num_added++;
        }
        if (fmaxf(0,_front_amp_factor[i]*front_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_down_amp_factor[i]*down_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_yaw_amp_factor[i]*yaw_out) > 0.0f) {
            _num_added++;
        }

        if (_num_added > 0) {
            _off[i] = _off[i]/_num_added; //average the offsets
        }

        if ((_amp[i]+fabsf(_off[i])) > 1) {
            _amp[i] = 1 - fabsf(_off[i]);
        }

        if (turbo_mode) {
            //double speed fins if offset at max...
            if (_amp[i] <= 0.6 && fabsf(_off[i]) >= 0.4) {
                _freq[i] = 2;
            }
        }
        // finding and outputting current position for each servo from sine wave
        _pos[i]= _amp[i]*cosf(freq_hz * _freq[i] * _time * 2 * M_PI) + _off[i];
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _pos[i] * FIN_SCALE_MAX);
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINO(_amp, _off);
#endif
}

/**
 * @brief Output minimum (zero) fin positions for safe initialization or shutdown
 * 
 * @details Sets all control inputs to zero and calls output() to generate neutral
 *          fin positions. Used during vehicle initialization, pre-arming checks, or
 *          controlled shutdown sequences to ensure fins are in a safe, non-thrusting
 *          state. Unlike simply not calling output(), this explicitly sets servos to
 *          their neutral positions, which is important for servo initialization and
 *          preventing unexpected motion during power-up.
 *          
 *          When output() is called with zero control inputs, fins will output their
 *          pure offset values (typically 0.5 for back/front fins' down axis) without
 *          any oscillating amplitude component, resulting in stationary fins at their
 *          mechanical neutral positions.
 * 
 * @note Safe to call at any time, including when disarmed
 * @note Does not change arming state, only control inputs
 * 
 * @see output() for full output generation algorithm
 */
void Fins::output_min()
{
    right_out = 0;
    front_out = 0;
    down_out  = 0;
    yaw_out   = 0;
    Fins::output();
}
