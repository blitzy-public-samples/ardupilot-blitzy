#include "Tracker.h"

/**
 * @file mode_manual.cpp
 * @brief Manual control mode implementation for antenna tracker
 * 
 * @details Provides direct RC control of tracker servos without automatic tracking.
 *          In MANUAL mode, pilot RC inputs are passed directly to the tracker's
 *          yaw and pitch servos, allowing manual positioning of the antenna.
 *          No automatic target tracking or position calculations occur in this mode.
 */

/**
 * @brief Updates manual control mode by passing RC inputs directly to servo outputs
 * 
 * @details Called at 50Hz while in MANUAL mode. Directly copies pilot RC yaw/pitch 
 *          inputs to corresponding servo PWM outputs, constrained to configured servo 
 *          limits. No automatic target tracking occurs in this mode.
 *          
 *          The function performs these operations:
 *          1. Reads RC input from yaw channel (CH_YAW) and writes to tracker yaw servo
 *          2. Applies servo constraints (min/max limits) to yaw output
 *          3. Reads RC input from pitch channel (CH_PITCH) and writes to tracker pitch servo
 *          4. Applies servo constraints (min/max limits) to pitch output
 *          
 *          RC Channel Mapping:
 *          - CH_YAW: Yaw control input (typically right stick left/right)
 *          - CH_PITCH: Pitch control input (typically right stick up/down)
 *          
 *          Servo outputs are constrained by SRV_Channels::constrain_pwm() which 
 *          enforces the configured servo min/max PWM limits for safety.
 * 
 * @note Manual mode provides direct RC control without automatic tracking, 
 *       useful for initial setup, testing servo movement, and manual antenna positioning.
 * 
 * Source: AntennaTracker/mode_manual.cpp:11-19
 */
void ModeManual::update()
{
    // copy yaw and pitch input to output
    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, RC_Channels::rc_channel(CH_YAW)->get_radio_in());
    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_yaw);

    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, RC_Channels::rc_channel(CH_PITCH)->get_radio_in());
    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_pitch);
}
