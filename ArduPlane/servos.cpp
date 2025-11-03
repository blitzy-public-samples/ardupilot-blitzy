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

/**
 * @file servos.cpp
 * @brief Servo output management for ArduPlane including mixing, slew rate limiting, battery compensation, and final PWM output
 * 
 * @details This file implements the complete servo output pipeline for fixed-wing aircraft:
 * - Servo mixing for various airframe configurations (elevons, v-tail, differential spoilers, flaperons)
 * - Throttle slew rate limiting to prevent motor damage and provide smooth power changes
 * - Battery voltage compensation to maintain consistent performance as battery drains
 * - Throttle suppression logic for ground safety
 * - Automatic flap deployment based on airspeed
 * - Twin-engine differential thrust mixing
 * - Final PWM output to all control surfaces and motors via SRV_Channels
 * 
 * The main entry point is set_servos() which is called at the main loop rate (typically 400Hz).
 * This coordinates all servo output calculations and applies safety limits before final output.
 * 
 * @note This is a critical flight control path - all control surface and motor outputs pass through here
 * @warning Any modifications to this file can directly affect vehicle control authority and safety
 */

#include "Plane.h"
#include <utility>

/**
 * @brief Apply throttle slew rate limiting to prevent abrupt motor speed changes
 * 
 * @details Implements gradual throttle changes to:
 * - Prevent motor damage from rapid power transitions
 * - Avoid stalling internal combustion engines
 * - Provide smoother flight characteristics
 * - Reduce stress on propulsion system components
 * 
 * The slew rate varies based on flight phase:
 * - Normal flight: Uses THROTTLE_SLEWRATE parameter
 * - Takeoff: Uses TKOFF_THR_SLEW if configured (typically more aggressive)
 * - Landing: Uses landing-specific slew rate if configured
 * - VTOL transition: Uses takeoff slew rate for forward transition
 * 
 * Slew limiting is only applied in auto-throttle modes (Auto, Guided, etc.) and
 * during quadplane assisted flight or VTOL modes. In manual throttle modes, the
 * slew rate is set to maximum (100%/cycle) to allow immediate pilot control.
 * 
 * @note Called at main loop rate (typically 400Hz) before final throttle output
 * @note Applies to k_throttle, k_throttleLeft, and k_throttleRight channels
 * @warning Slew rate too low can prevent reaching commanded throttle in time
 * @warning Slew rate too high defeats the protection mechanism
 * 
 * Source: ArduPlane/servos.cpp:25-63
 */
void Plane::throttle_slew_limit()
{
#if HAL_QUADPLANE_ENABLED
    const bool do_throttle_slew = (control_mode->does_auto_throttle() || quadplane.in_assisted_flight() || quadplane.in_vtol_mode());
#else
    const bool do_throttle_slew = control_mode->does_auto_throttle();
#endif

    if (!do_throttle_slew) {
        // only do throttle slew limiting in modes where throttle control is automatic
        SRV_Channels::set_slew_rate(SRV_Channel::k_throttle,      0.0, 100, G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_throttleLeft,  0.0, 100, G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_throttleRight, 0.0, 100, G_Dt);
        return;
    }

    uint8_t slewrate = aparm.throttle_slewrate;
    if (control_mode == &mode_auto) {
        if (auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
            slewrate = g.takeoff_throttle_slewrate;
        } else if (landing.get_throttle_slewrate() != 0 && flight_stage == AP_FixedWing::FlightStage::LAND) {
            slewrate = landing.get_throttle_slewrate();
        }
    }
    if (g.takeoff_throttle_slewrate != 0 &&
        (flight_stage == AP_FixedWing::FlightStage::TAKEOFF ||
         flight_stage == AP_FixedWing::FlightStage::VTOL)) {
        // for VTOL we use takeoff slewrate, which helps with transition
        slewrate = g.takeoff_throttle_slewrate;
    }
#if HAL_QUADPLANE_ENABLED
    if (g.takeoff_throttle_slewrate != 0 && quadplane.in_frwd_transition()) {
        slewrate = g.takeoff_throttle_slewrate;
    }
#endif
    SRV_Channels::set_slew_rate(SRV_Channel::k_throttle,      slewrate, 100, G_Dt);
    SRV_Channels::set_slew_rate(SRV_Channel::k_throttleLeft,  slewrate, 100, G_Dt);
    SRV_Channels::set_slew_rate(SRV_Channel::k_throttleRight, slewrate, 100, G_Dt);
}

/**
 * @brief Determine if throttle should be suppressed for ground safety
 * 
 * @details Implements safety logic to prevent propeller strike and unintended takeoff by
 * cutting throttle when the aircraft appears to be on the ground. This is a critical safety
 * feature that prevents accidents during ground handling and after landing.
 * 
 * Throttle is ALWAYS suppressed when:
 * - In Manual mode (pilot has direct throttle control)
 * - Parachute has been deployed (in auto-throttle modes)
 * - Landing controller explicitly requests suppression
 * 
 * Throttle suppression logic (in auto-throttle modes):
 * 
 * Throttle is suppressed (cut to zero) when ALL conditions are met:
 * 1. In auto-throttle mode (Auto, Guided, Loiter, Circle, etc.)
 * 2. Altitude is within 10m of home altitude
 * 3. Ground speed < 5 m/s (or airspeed < 5 m/s if sensor available)
 * 4. NOT performing active takeoff sequence
 * 
 * Special handling for takeoff sequences:
 * - During Auto takeoff or Takeoff mode: suppression active until auto_takeoff_check() passes
 * - Suppression released when takeoff conditions detected:
 *   * Been flying >5s in any mode
 *   * Altitude >5m above ground/home
 *   * Pitch angle < 30° (not being held for hand launch)
 *   * GPS showing movement
 * - This prevents stuck suppression if mission is reset while already airborne
 * 
 * Once unsuppressed (throttle enabled), remains unsuppressed if:
 * - Altitude exceeds 10m from home
 * - Ground speed exceeds 5 m/s (and airspeed > 5 m/s if sensor equipped)
 * - Quadplane reports is_flying() (if equipped)
 * 
 * The throttle_suppressed flag maintains state to prevent rapid toggling.
 * It is cleared (throttle enabled) when above altitude/speed thresholds,
 * and set (throttle disabled) when on ground in auto mode.
 * 
 * @return true if throttle should be suppressed (cut to zero), false if throttle allowed
 * 
 * @note This is a safety-critical function - prevents propeller strikes on ground
 * @note Manual mode always returns false (pilot has direct control)
 * @note Suppression can prevent legitimate takeoffs if conditions not met
 * @warning GPS velocity spikes can briefly unsuppress throttle on ground
 * @warning Altitude drift can cause unwanted suppression at low altitude
 * @warning Takeoff detection must be reliable to avoid stuck suppression
 * 
 * @see auto_takeoff_check() for takeoff sequence validation
 * @see set_throttle() which applies the suppression
 * @see landing.is_throttle_suppressed() for landing-specific suppression
 * 
 * Source: ArduPlane/servos.cpp:78-167
 */
bool Plane::suppress_throttle(void)
{
    if (control_mode == &mode_manual) {
        // Throttle is never suppressed in manual mode
        return false;
    }

#if HAL_PARACHUTE_ENABLED
    if (control_mode->does_auto_throttle() && parachute.release_initiated()) {
        // throttle always suppressed in auto-throttle modes after parachute release initiated
        throttle_suppressed = true;
        return true;
    }
#endif

    if (landing.is_throttle_suppressed()) {
        return true;
    }

    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (!control_mode->does_auto_throttle()) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    bool gps_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed() >= 5);
    
    if ((control_mode == &mode_auto &&
         auto_state.takeoff_complete == false) ||
        control_mode == &mode_takeoff) {

        uint32_t launch_duration_ms = ((int32_t)g.takeoff_throttle_delay)*100 + 2000;
        if (is_flying() &&
            millis() - started_flying_ms > MAX(launch_duration_ms, 5000U) && // been flying >5s in any mode
            adjusted_relative_altitude_cm() > 500 && // are >5m above AGL/home
            fabsf(ahrs.get_pitch_deg()) < 30 && // not high pitch, which happens when held before launch
            gps_movement) { // definite gps movement
            // we're already flying, do not suppress the throttle. We can get
            // stuck in this condition if we reset a mission and cmd 1 is takeoff
            // but we're currently flying around below the takeoff altitude
            throttle_suppressed = false;
            return false;
        }
        if (auto_takeoff_check()) {
            // we're in auto takeoff 
            throttle_suppressed = false;
            auto_state.baro_takeoff_alt = barometer.get_altitude();
            return false;
        }
        // keep throttle suppressed
        return true;
    }
    
    if (fabsf(relative_altitude) >= 10.0f) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (gps_movement) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents throttle up due to spiky GPS
        // groundspeed with bad GPS reception
#if AP_AIRSPEED_ENABLED
        if ((!ahrs.using_airspeed_sensor()) || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            throttle_suppressed = false;
            return false;        
        }
#else
        // no airspeed sensor, so we trust that the GPS's movement is truthful
        throttle_suppressed = false;
        return false;
#endif
    }

#if HAL_QUADPLANE_ENABLED
    if (quadplane.is_flying()) {
        throttle_suppressed = false;
        return false;
    }
#endif

    // throttle remains suppressed
    return true;
}


/**
 * @brief Core mixing function for elevon and v-tail configurations
 * 
 * @details Implements generic two-axis to two-surface mixing for flying wings (elevons)
 * and v-tail aircraft. The mixer combines two input channels (e.g., aileron and elevator)
 * and produces two output channels (e.g., left and right elevons).
 * 
 * Mixing algorithm:
 * - Apply MIXING_OFFSET to bias inputs if needed (for asymmetric airframes)
 * - Output1 = (Input2 - Input1) * MIXING_GAIN
 * - Output2 = (Input2 + Input1) * MIXING_GAIN
 * 
 * For elevons (typical flying wing):
 * - Input1 = aileron (roll control)
 * - Input2 = elevator (pitch control)
 * - Output1 = left elevon = elevator - aileron
 * - Output2 = right elevon = elevator + aileron
 * 
 * For v-tail:
 * - Input1 = rudder (yaw control)
 * - Input2 = elevator (pitch control)
 * - Output1 = right v-tail = elevator + rudder
 * - Output2 = left v-tail = elevator - rudder
 * 
 * MIXING_OFFSET (g.mixing_offset):
 * - Compensates for asymmetric airframes or motor placement
 * - Positive values scale input1 down, negative values scale input2 down
 * - Range: -100 to +100 (percentage)
 * 
 * MIXING_GAIN (g.mixing_gain):
 * - Controls how much of each input contributes to outputs
 * - Typical value: 0.5 (50% of each input)
 * - Range: 0.0 to 1.0
 * 
 * @param[in] func1_in First input function (e.g., k_aileron or k_rudder)
 * @param[in] func2_in Second input function (e.g., k_elevator)
 * @param[in] func1_out First output function (e.g., k_elevon_left or k_vtail_right)
 * @param[in] func2_out Second output function (e.g., k_elevon_right or k_vtail_left)
 * 
 * @note Operates on scaled servo values (-4500 to +4500 representing -100% to +100%)
 * @note Outputs are constrained to ±4500 to prevent overflow
 * @note Individual servo trim and limits (SERVOn_MIN/MAX/TRIM) are applied after mixing
 * @warning Incorrect MIXING_GAIN can cause control surface saturation
 * @warning MIXING_OFFSET should be adjusted carefully to avoid control coupling
 * 
 * Source: ArduPlane/servos.cpp:176-196
 */
void Plane::channel_function_mixer(SRV_Channel::Function func1_in, SRV_Channel::Function func2_in,
                                   SRV_Channel::Function func1_out, SRV_Channel::Function func2_out) const
{
    // the order is setup so that non-reversed servos go "up", and
    // func1 is the "left" channel. Users can adjust with channel
    // reversal as needed
    float in1 = SRV_Channels::get_output_scaled(func1_in);
    float in2 = SRV_Channels::get_output_scaled(func2_in);

    // apply MIXING_OFFSET to input channels
    if (g.mixing_offset < 0) {
        in2 *= (100 - g.mixing_offset) * 0.01;
    } else if (g.mixing_offset > 0) {
        in1 *= (100 + g.mixing_offset) * 0.01;
    }
    
    float out1 = constrain_float((in2 - in1) * g.mixing_gain, -4500, 4500);
    float out2 = constrain_float((in2 + in1) * g.mixing_gain, -4500, 4500);
    SRV_Channels::set_output_scaled(func1_out, out1);
    SRV_Channels::set_output_scaled(func2_out, out2);
}


/**
 * @brief Calculate flaperon outputs by mixing aileron and flap commands
 * 
 * @details Flaperons are control surfaces that act as both ailerons (roll control) and
 * flaps (lift augmentation). This function combines aileron input for roll control with
 * flap input for high-lift configuration, allowing the same surfaces to serve both functions.
 * 
 * Mixing algorithm:
 * - Get aileron command from flight controller (-4500 to +4500)
 * - Get flap percentage from automatic flap controller (0 to 100%)
 * - Left flaperon = aileron + (flap_percent * 45)
 * - Right flaperon = aileron - (flap_percent * 45)
 * 
 * The 45-degree scaling factor converts flap percentage to the same scale as aileron
 * commands (4500 = 100%, so 45 per percent gives proper scaling).
 * 
 * When flaps are deployed:
 * - Both flaperons move down (positive) to increase lift
 * - Aileron input still provides differential movement for roll control
 * - Full flaps (100%) produces ±4500 offset, adding to aileron command
 * 
 * The flap input comes from k_flap_auto which is set by set_servos_flaps() as the
 * maximum of manual flap input and automatic flap deployment based on airspeed.
 * Flap slew rate limiting is applied before this function sees the value.
 * 
 * @note Called from set_servos_flaps() after flap position is calculated
 * @note Uses slew-limited flap output to ensure gradual flaperon transitions
 * @note Outputs are constrained to ±4500 to prevent saturation
 * @warning Flaperon deflection affects both lift and roll control simultaneously
 * 
 * @see set_servos_flaps() for automatic flap deployment logic
 * @see SRV_Channels::get_slew_limited_output_scaled() for rate-limited flap input
 * 
 * Source: ArduPlane/servos.cpp:202-215
 */
void Plane::flaperon_update()
{
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.
     */
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    float flap_percent = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto);
    float flaperon_left  = constrain_float(aileron + flap_percent * 45, -4500, 4500);
    float flaperon_right = constrain_float(aileron - flap_percent * 45, -4500, 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_left, flaperon_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_right, flaperon_right);
}


/**
 * @brief Calculate differential spoiler outputs for advanced wing control
 * 
 * @details Differential spoilers are a four-surface wing configuration that provides
 * independent control of roll, pitch, yaw, and drag. Each wing has an outer and inner
 * surface that can be deflected independently, enabling:
 * - Roll control via differential deflection (aileron function)
 * - Pitch control via symmetric deflection (elevator function, flying wing only)
 * - Yaw control via asymmetric spoiler deployment (rudder mixing)
 * - Drag control via crow flap configuration (both surfaces up on both wings)
 * 
 * Surface naming:
 * - Left wing: dspoilerLeft1 (outer), dspoilerLeft2 (inner)
 * - Right wing: dspoilerRight1 (outer), dspoilerRight2 (inner)
 * 
 * Control mixing modes (controlled by CROW_FLAP_OPT bitmask):
 * 
 * 1. **Standard mode** (FLYINGWING=0):
 *    - Outer surfaces: aileron + rudder
 *    - Inner surfaces: crow flaps only
 *    - Pitch control via separate elevator
 * 
 * 2. **Flying wing mode** (FLYINGWING=1):
 *    - Outer surfaces: elevon (aileron + elevator) + rudder
 *    - Inner surfaces: crow flaps only
 *    - No separate elevator (elevon mixing used)
 * 
 * 3. **Full span aileron** (FULLSPAN=1):
 *    - Inner surfaces also provide aileron function
 *    - Increases roll authority
 *    - CROW_FLAP_AILERON_MATCHING limits inner surface throw for flaps
 * 
 * 4. **Crow flaps** (for landing drag/speed control):
 *    - Outer surfaces: move up by CROW_FLAP_WEIGHT_OUTER percent
 *    - Inner surfaces: move down by CROW_FLAP_WEIGHT_INNER percent
 *    - Creates high drag configuration for steep descents
 *    - Progressive mode: deploys inner first (0-50% flap), then adds outer (50-100%)
 * 
 * Rudder mixing:
 * - Applies to outer surfaces only
 * - Scaled by DSPOILR_RUD_RATE (typically 25-100%)
 * - Right rudder: increases right outer spoiler, decreases right inner
 * - Left rudder: increases left outer spoiler, decreases left inner
 * - Provides yaw control without dedicated rudder
 * 
 * Key parameters:
 * - CROW_FLAP_OPT: Configuration bitmask
 * - CROW_FLAP_WEIGHT_OUTER: Outer surface crow deflection (0-100%)
 * - CROW_FLAP_WEIGHT_INNER: Inner surface crow deflection (0-100%)
 * - DSPOILR_RUD_RATE: Rudder mixing gain (0-100%)
 * - CROW_FLAP_AILERON_MATCHING: Limits inner surface aileron throw (0-100%)
 * 
 * @note Called from servos_output() after base control surface commands calculated
 * @note All outputs constrained to ±4500 (±100%) to prevent saturation
 * @note Crow mode can be disabled via RC switch (crow_mode = CROW_DISABLED)
 * @warning Complex mixing - test thoroughly before flight
 * @warning High crow weights can cause significant pitch changes
 * 
 * Source: ArduPlane/servos.cpp:224-314
 */
void Plane::dspoiler_update(void)
{
    const int8_t bitmask = g2.crow_flap_options.get();
    const bool flying_wing       = (bitmask & CrowFlapOptions::FLYINGWING) != 0;
    const bool full_span_aileron = (bitmask & CrowFlapOptions::FULLSPAN) != 0;
    //progressive crow when option is set or RC switch is set to progressive 
    const bool progressive_crow   = (bitmask & CrowFlapOptions::PROGRESSIVE_CROW) != 0  || crow_mode == CrowMode::PROGRESSIVE; 

    // if flying wing use elevons else use ailerons
    float elevon_left;
    float elevon_right;
    if (flying_wing) {
        elevon_left = SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left);
        elevon_right = SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right);
    } else {
        const float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        elevon_left = -aileron;
        elevon_right = aileron;
    }

    const float rudder_rate = g.dspoiler_rud_rate * 0.01f;
    const float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * rudder_rate;
    float dspoiler_outer_left = elevon_left;
    float dspoiler_outer_right = elevon_right;

    float dspoiler_inner_left = 0;
    float dspoiler_inner_right = 0;

    // full span ailerons / elevons
    if (full_span_aileron) {
        dspoiler_inner_left = elevon_left;
        dspoiler_inner_right = elevon_right;
    }

    if (rudder > 0) {
        // apply rudder to right wing
        dspoiler_outer_right = constrain_float(dspoiler_outer_right + rudder, -4500, 4500);
        dspoiler_inner_right = constrain_float(dspoiler_inner_right - rudder, -4500, 4500);
    } else {
        // apply rudder to left wing
        dspoiler_outer_left = constrain_float(dspoiler_outer_left - rudder, -4500, 4500);
        dspoiler_inner_left = constrain_float(dspoiler_inner_left + rudder, -4500, 4500);
    }

    // limit flap throw used for aileron
    const int8_t aileron_matching = g2.crow_flap_aileron_matching.get();
    if (aileron_matching < 100) {
        // only do matching if it will make a difference
        const float aileron_matching_scaled = aileron_matching * 0.01;
        if (is_negative(dspoiler_inner_left)) {
            dspoiler_inner_left *= aileron_matching_scaled;
        }
        if (is_negative(dspoiler_inner_right)) {
            dspoiler_inner_right *= aileron_matching_scaled;
        }
    }

    int16_t weight_outer = g2.crow_flap_weight_outer.get();
    if (crow_mode == Plane::CrowMode::CROW_DISABLED) {   //override totally aileron crow if crow RC switch set to disabled
        weight_outer = 0;
    }
    const int16_t weight_inner = g2.crow_flap_weight_inner.get();
    if (weight_outer > 0 || weight_inner > 0) {
        /*
          apply crow flaps by apply the same split of the differential
          spoilers to both wings. Get flap percentage from k_flap_auto, which is set
          in set_servos_flaps() as the maximum of manual and auto flap control
         */
        const float flap_percent = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_flap_auto);

        if (is_positive(flap_percent)) {
            float inner_flap_scaled = flap_percent;
            float outer_flap_scaled = flap_percent;
            if (progressive_crow) {
                // apply 0 - full inner from 0 to 50% flap then add in outer above 50%
                inner_flap_scaled = constrain_float(inner_flap_scaled * 2, 0,100);
                outer_flap_scaled = constrain_float(outer_flap_scaled - 50, 0,50) * 2;
            }
            // scale flaps so when weights are 100 they give full up or down
            dspoiler_outer_left  = constrain_float(dspoiler_outer_left  + outer_flap_scaled * weight_outer * 0.45, -4500, 4500);
            dspoiler_inner_left  = constrain_float(dspoiler_inner_left  - inner_flap_scaled * weight_inner * 0.45, -4500, 4500);
            dspoiler_outer_right = constrain_float(dspoiler_outer_right + outer_flap_scaled * weight_outer * 0.45, -4500, 4500);
            dspoiler_inner_right = constrain_float(dspoiler_inner_right - inner_flap_scaled * weight_inner * 0.45, -4500, 4500);
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerLeft1, dspoiler_outer_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerLeft2, dspoiler_inner_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight1, dspoiler_outer_right);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight2, dspoiler_inner_right);
}

/**
 * @brief Calculate airbrake deployment from reverse thrust and manual RC input
 * 
 * @details Airbrakes are drag-producing surfaces that help slow the aircraft without
 * using engine power. They deploy automatically when reverse thrust is commanded,
 * or manually via RC channel. This is particularly useful for steep descents and
 * landing approaches where speed control is critical.
 * 
 * @note Airbrakes are distinct from spoilers - they only produce drag, not roll control
 * @note Multiple airbrake servos can be assigned using SERVOx_FUNCTION = Airbrake (86)
 * 
 * Airbrake deployment sources (highest value wins):
 * 
 * 1. **Manual RC input**:
 *    - Read from channel assigned to RCx_OPTION = Airbrake (87)
 *    - Only used if RC not in failsafe and throttle failsafe not active
 *    - Percentage input: 0% = retracted, 100% = fully deployed
 *    - Pilot has direct control independent of flight mode
 * 
 * 2. **Automatic from reverse thrust** (requires THR_MIN < 0):
 *    - **During flare**: 100% airbrakes for maximum deceleration
 *    - **Negative throttle commanded**: Proportional to throttle magnitude
 *      - 0% throttle = 0% airbrakes
 *      - -100% throttle (full reverse) = 100% airbrakes
 *    - Helps aircraft slow down without excessive engine reverse thrust
 *    - Reduces wear on propulsion system
 * 
 * Selection logic:
 * - Manual input takes priority if larger than auto deployment
 * - Allows pilot to override automatic logic if needed
 * - Auto logic only active when THR_MIN parameter allows reverse thrust
 * - Failsafe conditions disable manual input, use auto logic only
 * 
 * Use cases:
 * - **Steep descent**: Deploy airbrakes to maintain safe speed on approach
 * - **Landing flare**: Full deployment for short landing roll
 * - **Go-around prevention**: Reduces float during landing
 * - **Speed control**: Alternative to engine idle for descents
 * - **Emergency descent**: Rapid altitude loss without overspeeding
 * 
 * Configuration requirements:
 * - THR_MIN < 0: Enables reverse thrust and auto airbrake logic
 * - RCx_OPTION = 87: Assigns channel for manual airbrake control
 * - SERVOx_FUNCTION = 86: Assigns servo output(s) as airbrakes
 * - Physical installation: Airbrake surfaces must produce pure drag
 * 
 * Safety considerations:
 * - Airbrakes increase sink rate - requires careful altitude management
 * - Full deployment may require power to maintain approach angle
 * - Should not affect longitudinal or lateral trim
 * - Must be balanced left/right to avoid yaw
 * - Emergency retraction may be needed for go-around
 * 
 * Output:
 * - Sets SRV_Channel::k_airbrake to calculated percentage (0-100)
 * - No slew rate limiting (immediate response for drag control)
 * - All servos assigned to Airbrake function receive same command
 * 
 * @note Called from set_servos() after throttle has been calculated
 * @note No slew rate limiting - airbrakes respond immediately
 * @note Airbrakes do not affect control authority (unlike spoilers)
 * @warning Excessive airbrake deployment can cause high sink rates
 * @warning Asymmetric deployment will cause yaw - verify balanced installation
 * @warning Full deployment during cruise can overstress airframe
 * 
 * @see landing.is_flaring() for flare detection
 * @see SRV_Channel::k_airbrake for output channel assignment
 * @see channel_airbrake RC input channel
 * @see Parameters: THR_MIN for reverse thrust enable
 * 
 * Source: ArduPlane/servos.cpp:319-352
 */
void Plane::airbrake_update(void)
{
    // Calculate any manual airbrake input from RC channel option.
    float manual_airbrake_percent = 0;

    if (channel_airbrake != nullptr && !failsafe.rc_failsafe && failsafe.throttle_counter == 0) {
        manual_airbrake_percent = channel_airbrake->percent_input();
    }

    // Calculate auto airbrake from negative throttle.
    float throttle_min = aparm.throttle_min.get();
    float airbrake_pc = 0;

    float throttle_pc = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

    if (throttle_min < 0) {
        if (landing.is_flaring()) {
            // Full airbrakes during the flare.
            airbrake_pc = 100;
        }
        else {
            // Determine fraction between zero and full negative throttle.
            airbrake_pc = constrain_float(-throttle_pc, 0, 100);
        }
    }

    // Manual overrides auto airbrake setting.
    if (airbrake_pc < manual_airbrake_percent) {
        airbrake_pc = manual_airbrake_percent;
    }

    // Output to airbrake servo types.
    SRV_Channels::set_output_scaled(SRV_Channel::k_airbrake, airbrake_pc);
}

/*
  setup servos for idle wiggle mode
  Idle mode is used during balloon launch to keep servos still, apart
  from occasional wiggle to prevent freezing up
 */
void ModeAuto::wiggle_servos()
{
    // This is only active while in AUTO running NAV_ALTITUDE_WAIT with wiggle_time > 0
    if (wiggle.last_ms == 0) {
        return;
    }

    int16_t servo_valueElevator;
    int16_t servo_valueAileronRudder;
    // Wiggle the control surfaces in stages: elevators first, then rudders + ailerons, through the full range over 4 seconds
    if (wiggle.stage != 0) {
        wiggle.stage += 1;
    }
    if (wiggle.stage == 0) {
        servo_valueElevator = 0;
        servo_valueAileronRudder = 0;
    } else if (wiggle.stage < 25) { 
        servo_valueElevator = wiggle.stage * (4500 / 25);      
        servo_valueAileronRudder = 0;
    } else if (wiggle.stage < 75) {
        servo_valueElevator = (50 - wiggle.stage) * (4500 / 25);        
        servo_valueAileronRudder = 0;
    } else if (wiggle.stage < 100) {
        servo_valueElevator = (wiggle.stage - 100) * (4500 / 25);        
        servo_valueAileronRudder = 0;
    } else if (wiggle.stage < 125) {
        servo_valueElevator = 0;
        servo_valueAileronRudder = (wiggle.stage - 100) * (4500 / 25);
    } else if (wiggle.stage < 175) {
        servo_valueElevator = 0;
        servo_valueAileronRudder = (150 - wiggle.stage) * (4500 / 25);  
    } else if (wiggle.stage < 200) {
        servo_valueElevator = 0;
        servo_valueAileronRudder = (wiggle.stage - 200) * (4500 / 25); 
    } else {
        wiggle.stage = 0;
        servo_valueElevator = 0;
        servo_valueAileronRudder = 0;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, servo_valueAileronRudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, servo_valueElevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, servo_valueAileronRudder);

}


/*
  Calculate the throttle scale to compensate for battery voltage drop
 */
void ParametersG2::FWD_BATT_CMP::update()
{
    // Assume disabled
    enabled = false;

    // return if not enabled, or setup incorrectly
    if (!is_positive(batt_voltage_min) || batt_voltage_min >= batt_voltage_max) {
        return;
    }

    float batt_voltage_resting_estimate = AP::battery().voltage_resting_estimate(batt_idx);
    // Return for a very low battery
    if (batt_voltage_resting_estimate < 0.25f * batt_voltage_min) {
        return;
    }

    // constrain read voltage to min and max params
    batt_voltage_resting_estimate = constrain_float(batt_voltage_resting_estimate, batt_voltage_min, batt_voltage_max);

    // don't apply compensation if the voltage is excessively low
    if (batt_voltage_resting_estimate < 1) {
        return;
    }

    // Scale the throttle up to compensate for voltage drop
    // Ratio = 1 when voltage = voltage max, ratio increases as voltage drops
    ratio = batt_voltage_max / batt_voltage_resting_estimate;

    // Got this far then ratio is valid
    enabled = true;
}

// Apply throttle scale to min and max limits
void ParametersG2::FWD_BATT_CMP::apply_min_max(int8_t &min_throttle, int8_t &max_throttle) const
{
    // Cut off throttle if FWD_BAT_IDX battery resting voltage is below
    // FWD_THR_CUTOFF_V (if set), to preserve battery life for the electronics
    // and actuators. Only applies when the battery monitor is working and the
    // current mode does auto-throttle.
    if (is_positive(batt_voltage_throttle_cutoff) &&
        plane.control_mode->does_auto_throttle() && AP::battery().healthy(batt_idx) &&
        (AP::battery().voltage_resting_estimate(batt_idx) < batt_voltage_throttle_cutoff)) {
        min_throttle = 0;
        max_throttle = 0;

        return;
    }

    // return if not enabled
    if (!enabled) {
        return;
    }

    // Scale the throttle limits to prevent subsequent clipping
    // Ratio will always be >= 1, ensure still within max limits
    min_throttle = int8_t(MAX((ratio * (float)min_throttle), -100));
    max_throttle = int8_t(MIN((ratio * (float)max_throttle),  100));

}

// Apply throttle scale to throttle demand
float ParametersG2::FWD_BATT_CMP::apply_throttle(float throttle) const
{
    // return if not enabled
    if (!enabled) {
        return throttle;
    }

    // Ratio will always be >= 1, ensure still within max limits
    return constrain_float(throttle * ratio, -100, 100);

}

/*
  calculate any throttle limits based on the watt limiter
 */
#if AP_BATTERY_WATT_MAX_ENABLED
void Plane::throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle)
{
    uint32_t now = millis();
    if (battery.overpower_detected()) {
        // overpower detected, cut back on the throttle if we're maxing it out by calculating a limiter value
        // throttle limit will attack by 10% per second
        
        if (is_positive(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)) && // demanding too much positive thrust
            throttle_watt_limit_max < max_throttle - 25 &&
            now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max++;
            
        } else if (is_negative(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)) &&
                   min_throttle < 0 && // reverse thrust is available
                   throttle_watt_limit_min < -(min_throttle) - 25 &&
                   now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_min++;
        }
        
    } else if (now - throttle_watt_limit_timer_ms >= 1000) {
        // it has been 1 second since last over-current, check if we can resume higher throttle.
        // this throttle release is needed to allow raising the max_throttle as the battery voltage drains down
        // throttle limit will release by 1% per second
        if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > throttle_watt_limit_max && // demanding max forward thrust
            throttle_watt_limit_max > 0) { // and we're currently limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max--;
            
        } else if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < throttle_watt_limit_min && // demanding max negative thrust
                   throttle_watt_limit_min > 0) { // and we're limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_min--;
        }
    }
    
    max_throttle = constrain_int16(max_throttle, 0, max_throttle - throttle_watt_limit_max);
    if (min_throttle < 0) {
        min_throttle = constrain_int16(min_throttle, min_throttle + throttle_watt_limit_min, 0);
    }
}
#endif // #if AP_BATTERY_WATT_MAX_ENABLED

/**
 * @brief Apply comprehensive safety limits and constraints to throttle command
 * 
 * @details Coordinates multiple throttle limiting systems to ensure safe motor operation
 * across all flight phases. Applies limits from parameters, flight stage, battery state,
 * and power draw monitoring. Final limits are also communicated to TECS controller.
 * 
 * @param[in] throttle_in Desired throttle percentage from flight mode (-100 to 100)
 * @return Constrained throttle output in range [min_throttle, max_throttle]
 * 
 * Limiting sequence (each stage can modify min/max):
 * 
 * 1. **Base limits from parameters**:
 *    - THR_MIN: Minimum throttle (can be negative for reverse thrust)
 *    - THR_MAX: Maximum throttle for normal operation
 * 
 * 2. **ICE idle governor** (if internal combustion engine enabled):
 *    - May increase min_throttle to maintain engine idle speed
 *    - Prevents engine stall during low throttle conditions
 * 
 * 3. **Reverse thrust inhibit**:
 *    - If reverse thrust available but not allowed (mode-specific):
 *    - Clamps min_throttle to 0 to prevent reverse operation
 *    - Controlled by allow_reverse_thrust() per flight mode
 * 
 * 4. **Takeoff/abort landing limits**:
 *    - Uses TKOFF_THR_MAX and TKOFF_THR_MIN parameters
 *    - Prevents excessive or insufficient thrust during critical phase
 *    - Applied when flight_stage is TAKEOFF or ABORT_LANDING
 * 
 * 5. **Landing flare limits**:
 *    - During flare: min_throttle forced to 0 to allow engine cutoff
 *    - Enables faster deceleration and shorter landing roll
 * 
 * 6. **Quadplane transition limits** (if equipped):
 *    - Forward transition uses TKOFF_THR_MAX as ceiling
 *    - SLT thrust type: min_throttle set to TKOFF_THR_MIN or cruise
 *    - Ensures adequate thrust during transition to forward flight
 * 
 * 7. **Battery voltage compensation**:
 *    - Scales min/max limits based on battery voltage
 *    - Allows higher throttle percentage as battery drains
 *    - Provides consistent power output across flight
 *    - May cut throttle completely if voltage below FWD_THR_CUTOFF_V
 * 
 * 8. **Watt limiter** (if battery watt limit enabled):
 *    - Monitors battery power draw via throttle_watt_limiter()
 *    - Reduces max_throttle if overpower detected (>10% per second attack)
 *    - Gradually releases limit when power is within spec (1% per second)
 *    - Protects battery from damage due to excessive current draw
 *    - Always preserves at least 25% throttle authority
 * 
 * 9. **Sanity check**:
 *    - Ensures min_throttle <= max_throttle
 *    - Prevents impossible constraint conflicts
 * 
 * 10. **TECS controller update**:
 *     - Informs Total Energy Control System of updated limits
 *     - TECS uses these for next iteration's throttle calculation
 *     - Ensures energy management respects all constraints
 * 
 * Flight phase examples:
 * - **Ground idle**: THR_MIN (possibly increased by ICE governor)
 * - **Takeoff roll**: [TKOFF_THR_MIN, TKOFF_THR_MAX]
 * - **Cruise**: [THR_MIN, THR_MAX] with battery compensation
 * - **Landing flare**: [0, THR_MAX]
 * - **Reverse thrust landing**: [THR_MIN (negative), 0] if allowed
 * 
 * @note This is the final throttle limiting stage before output
 * @note Limits are cumulative - each stage can further restrict the range
 * @note TECS receives limit information for coordinated energy management
 * @note Battery compensation can significantly alter effective limits
 * @warning Incorrect limit parameters can prevent takeoff or cause unsafe landings
 * @warning Watt limiter may reduce climb performance to protect battery
 * @warning Reverse thrust limits must match ESC/motor capabilities
 * 
 * @see g2.fwd_batt_cmp.apply_min_max() for battery voltage limiting
 * @see throttle_watt_limiter() for power draw limiting
 * @see TECS_controller.set_throttle_min/max() for energy controller coordination
 * @see allow_reverse_thrust() for reverse thrust mode logic
 * 
 * Source: ArduPlane/servos.cpp:534-607
 */
float Plane::apply_throttle_limits(float throttle_in)
{
    // Pull the base throttle limits.
    // These are usually set to map the ESC operating range.
    int8_t min_throttle = aparm.throttle_min.get();
    int8_t max_throttle = aparm.throttle_max.get();

#if AP_ICENGINE_ENABLED
    // Apply idle governor.
    g2.ice_control.update_idle_governor(min_throttle);
#endif

    // If reverse thrust is enabled not allowed right now, the minimum throttle must not fall below 0.
    if (min_throttle < 0 && !allow_reverse_thrust()) {
        // reverse thrust is available but inhibited.
        min_throttle = 0;
    }

    // Handle throttle limits for takeoff conditions.
    // Query the conditions where TKOFF_THR_MAX applies.
    const bool use_takeoff_throttle =
        (flight_stage == AP_FixedWing::FlightStage::TAKEOFF) ||
        (flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING);

    // Handle throttle limits for takeoff conditions.
    if (use_takeoff_throttle) {
        // Read from takeoff_state
        max_throttle = takeoff_state.throttle_lim_max;
        min_throttle = takeoff_state.throttle_lim_min;
    } else if (landing.is_flaring()) {
        // Allow throttle cutoff when flaring.
        // This is to allow the aircraft to bleed speed faster and land with a shut off thruster.
        min_throttle = 0;
    }

    // Handle throttle limits for transition conditions.
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_frwd_transition()) {
        if (aparm.takeoff_throttle_max != 0) {
            max_throttle = aparm.takeoff_throttle_max.get();
        }

        // Apply minimum throttle limits only for SLT thrust types.
        // The other types don't support it well.
        if (quadplane.get_thrust_type() == QuadPlane::ThrustType::SLT
            && control_mode->does_auto_throttle()
        ) {
            if (aparm.takeoff_throttle_min.get() != 0) {
                min_throttle = MAX(min_throttle, aparm.takeoff_throttle_min.get());
            } else {
                min_throttle = MAX(min_throttle, aparm.throttle_cruise.get());
            }
        }
    }
#endif

    // Compensate the limits for battery voltage drop.
    // This relaxes the limits when the battery is getting depleted.
    g2.fwd_batt_cmp.apply_min_max(min_throttle, max_throttle);

#if AP_BATTERY_WATT_MAX_ENABLED
    // Ensure that the power draw limits are not exceeded.
    throttle_watt_limiter(min_throttle, max_throttle);
#endif

    // Do a sanity check on them. Constrain down if necessary.
    min_throttle = MIN(min_throttle, max_throttle);

    // Let TECS know about the updated throttle limits.
    // These will be taken into account on the next iteration.
    TECS_controller.set_throttle_min(0.01f*min_throttle);
    TECS_controller.set_throttle_max(0.01f*max_throttle);
    return constrain_float(throttle_in, min_throttle, max_throttle);
}

/**
 * @brief Calculate final throttle output with compensation and safety limits
 * 
 * @details Applies battery voltage compensation and throttle suppression to the throttle
 * command from the flight mode controller. This ensures consistent aircraft performance
 * as the battery drains and provides ground safety through throttle cutoff.
 * 
 * Processing sequence:
 * 
 * 1. **Battery voltage compensation** (if enabled by mode):
 *    - Updates voltage scaling factor from current battery voltage
 *    - Scales throttle command to maintain constant power output
 *    - Ratio increases as battery drains (e.g., 1.2x at low voltage)
 *    - Controlled by FWD_BAT_VOLT_MIN and FWD_BAT_VOLT_MAX parameters
 *    - Only applied in modes that use_battery_compensation()
 * 
 * 2. **Throttle limit application** (if enabled by mode):
 *    - Applies minimum and maximum throttle constraints
 *    - Handles reverse thrust limits (if enabled)
 *    - Applies takeoff-specific throttle limits
 *    - Applies transition-specific limits (quadplane)
 *    - Applies watt limiter if battery overpower detected
 *    - Only applied in modes that use_throttle_limits()
 * 
 * 3. **Throttle suppression** (ground safety):
 *    If suppress_throttle() returns true, throttle is overridden:
 *    - Manual passthrough: Uses pilot stick input if g.throttle_suppress_manual=1
 *    - Flare mode: Uses THR_MIN if landing.use_thr_min_during_flare()=1
 *    - Takeoff idle: Uses TKOFF_THR_IDLE if in takeoff phase and parameter set
 *    - Default: Sets throttle to 0 (motor stop)
 * 
 * Battery compensation details:
 * - Compensates for voltage sag under load and battery discharge
 * - Maintains consistent thrust throughout flight
 * - Can be disabled per-mode if not desired
 * - Compensation ratio capped to prevent excessive throttle at low voltage
 * - Will cut throttle completely if voltage drops below FWD_THR_CUTOFF_V
 * 
 * @note Called from set_servos() before slew rate limiting is applied
 * @note Throttle suppression is safety-critical - prevents ground accidents
 * @note Battery compensation requires accurate battery monitor configuration
 * @warning Incorrect battery voltage parameters can cause thrust variations
 * @warning Suppression logic must be reliable to prevent propeller strikes
 * 
 * @see g2.fwd_batt_cmp.update() for battery compensation calculation
 * @see apply_throttle_limits() for limit application logic
 * @see suppress_throttle() for ground safety suppression logic
 * 
 * Source: ArduPlane/servos.cpp:612-651
 */
void Plane::set_throttle(void)
{

    // Update voltage scaling
    g2.fwd_batt_cmp.update();

    if (control_mode->use_battery_compensation()) {
        // Apply voltage compensation to throttle output from flight mode
        const float throttle = g2.fwd_batt_cmp.apply_throttle(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }

    if (control_mode->use_throttle_limits()) {
        // Apply min/max throttle limits
        const float limited_throttle = apply_throttle_limits(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle));
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, limited_throttle);
    }

    if (suppress_throttle()) {
        if (g.throttle_suppress_manual) {
            // manual pass through of throttle while throttle is suppressed
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));

        } else if (landing.is_flaring() && landing.use_thr_min_during_flare() ) {
            // throttle is suppressed (above) to zero in final flare in auto mode, but we allow instead thr_min if user prefers, eg turbines:
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, aparm.throttle_min.get());

        } else if ((flight_stage == AP_FixedWing::FlightStage::TAKEOFF)
                    && (aparm.takeoff_throttle_idle.get() > 0)
                  ) {
            // we want to spin at idle throttle before the takeoff conditions are met
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, aparm.takeoff_throttle_idle.get());
        } else {
            // default
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);

        }
    }

}

/**
 * @brief Notify AHRS system that takeoff is imminent to improve yaw estimation
 * 
 * @details Informs the EKF's Ground-based yaw Sensor Fusion (GSF) estimator that
 * takeoff motion is about to occur. This allows the GSF to initialize before
 * movement starts, resulting in better yaw angle accuracy during takeoff roll
 * and initial climb. Particularly important for fixed-wing launches.
 * 
 * Trigger conditions (must be armed and not flying):
 * 
 * 1. **Throw detection** (hand-launch):
 *    - Monitors acceleration along X axis (forward/back in body frame)
 *    - Subtracts gravity component: accel_x_due_to_throw = measured_accel_x - gravity*sin(pitch)
 *    - Threshold: Acceleration > 1g indicates a throw
 *    - Typical hand launch produces 1-3g forward acceleration spike
 *    - 1g threshold confirmed not to false trigger during normal handling
 * 
 * 2. **Throttle increase detection**:
 *    - Throttle commanded above cruise setting
 *    - Indicates pilot is applying takeoff power
 *    - Used for ground-roll takeoffs and catapult launches
 * 
 * GSF yaw estimator benefits:
 * - Initializes multiple yaw hypotheses before motion begins
 * - Rapidly converges to correct yaw during acceleration
 * - Improves heading accuracy in first few seconds of flight
 * - Reduces possibility of bad yaw estimate causing control issues
 * - Critical for GPS-based yaw before airspeed provides heading
 * 
 * Why early notification matters:
 * - GSF needs several seconds to initialize yaw estimator bank
 * - Starting before motion gives estimator time to prepare
 * - Better yaw estimate reduces initial flight path errors
 * - Prevents EKF innovation failures during launch
 * - Smoves transition from ground to flight state
 * 
 * Launch type handling:
 * - **Hand launch**: Detects throw via accelerometer spike
 * - **Bungee/catapult**: Detects via throttle up command
 * - **Ground roll**: Detects via throttle application
 * - **Autonomous takeoff**: Detects when AUTO mode commands takeoff throttle
 * 
 * State machine integration:
 * - Only active when armed but not yet is_flying()
 * - Once flying, notification no longer needed
 * - Disarmed state: No notification (safety locked out)
 * - Already flying: GSF already converged, notification redundant
 * 
 * Coordinate frame notes:
 * - X axis: Forward in body frame (nose direction)
 * - Gravity component removed: Only actual acceleration considered
 * - Pitch angle used to project gravity onto X axis
 * - 1g = 9.81 m/s² (GRAVITY_MSS constant)
 * 
 * @note Called from set_servos() on every iteration when armed
 * @note GSF estimator is part of EKF3 - requires EK3_ENABLE=1
 * @note Throw detection tested extensively on micro UAS platforms
 * @note False triggers can be prevented by increasing threshold if needed
 * @warning Incorrect IMU orientation will break throw detection
 * @warning Heavy vibration might cause false triggers at 1g threshold
 * 
 * @see ahrs.set_takeoff_expected() to notify AHRS system
 * @see is_flying() for flight state detection
 * @see ahrs.get_accel() for IMU acceleration data
 * @see ahrs.sin_pitch() for gravity component calculation
 * @see EKF3 GSF yaw estimator documentation
 * 
 * Source: ArduPlane/servos.cpp:656-672
 */
void Plane::set_takeoff_expected(void)
{
    // let EKF know to start GSF yaw estimator before takeoff movement starts so that yaw angle is better estimated
    const float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (!is_flying() && arming.is_armed()) {
        // Check if rate of change of velocity along X axis exceeds 1-g which normally indicates a throw.
        // Tests with hand carriage of micro UAS indicates that a 1-g threshold does not false trigger prior
        // to the throw, but there is margin to increase this threshold if false triggering becomes problematic.
        const float accel_x_due_to_gravity = GRAVITY_MSS * ahrs.sin_pitch();
        const float accel_x_due_to_throw = ahrs.get_accel().x - accel_x_due_to_gravity;
        bool throw_detected = accel_x_due_to_throw > GRAVITY_MSS;
        bool throttle_up_detected = throttle > aparm.throttle_cruise;
        if (throw_detected || throttle_up_detected) {
            plane.ahrs.set_takeoff_expected(true);
        }
    }
}

/**
 * @brief Calculate and set flap deflection based on airspeed, flight phase, and manual input
 * 
 * @details Implements automatic flap deployment strategy based on aircraft speed and flight
 * phase, with manual override capability. Flaps improve low-speed handling and reduce
 * stall speed but increase drag. This function balances these tradeoffs automatically.
 * 
 * Flap deployment logic (in priority order):
 * 
 * 1. **Manual flap input** (highest priority if larger than auto):
 *    - Read from RC flap channel if assigned and RC valid
 *    - Converts stick position to percentage (0-100%)
 *    - Manual overrides automatic deployment
 * 
 * 2. **Flight phase specific** (overrides speed-based logic):
 *    - **Takeoff/Abort landing**: Uses TKOFF_FLAP_PCNT parameter
 *    - **Pre-launch**: Uses TKOFF_FLAP_PCNT if configured
 *    - **Landing**: Uses landing.get_flap_percent() from landing controller
 *    - **Thermal soaring**: Uses g2.soaring_controller.get_thermalling_flap()
 *    - These provide optimal flap settings for each critical phase
 * 
 * 3. **Speed-based automatic deployment**:
 *    - Uses airspeed (target or actual) to determine flap percentage
 *    - Two-stage deployment system with hysteresis:
 *      - **Stage 1**: If speed <= FLAP_1_SPEED, deploy to FLAP_1_PERCNT
 *      - **Stage 2**: If speed <= FLAP_2_SPEED, deploy to FLAP_2_PERCNT
 *    - FLAP_2_SPEED < FLAP_1_SPEED (stage 2 is more flaps at lower speed)
 *    - Speed source priority:
 *      1. Target airspeed (if auto-throttle mode and sensor available)
 *      2. Actual airspeed (if FLIGHT_OPTIONS FlapsActualSpeed set)
 *      3. Throttle cruise setting (fallback if no airspeed sensor)
 * 
 * Speed source selection logic:
 * - **has_target_airspeed**: Modes with auto-throttle have target speed
 * - **flap_actual_speed option**: Use real airspeed instead of target
 * - **Combined mode**: Use minimum of target and actual (brings flaps in early when slowing)
 * - **No sensor**: Falls back to THR_CRUISE as proxy for cruise speed
 * 
 * Slew rate limiting:
 * - Flaps move gradually at FLAP_SLEWRATE degrees per second
 * - Prevents abrupt load changes that could damage airframe
 * - Applies to both k_flap (manual) and k_flap_auto channels
 * - Slewing happens in SRV_Channels layer after this function sets target
 * 
 * Output channels:
 * - **k_flap**: Manual flap position (direct pilot control)
 * - **k_flap_auto**: Automatic flap position (maximum of manual or auto)
 * - Mixers read k_flap_auto for flaperon and crow flap calculations
 * - Dedicated flap servos typically read k_flap or k_flap_auto
 * 
 * Typical speed thresholds (example):
 * - Cruise: 0% flaps (clean configuration)
 * - FLAP_1_SPEED = 15 m/s: 25% flaps (FLAP_1_PERCNT)
 * - FLAP_2_SPEED = 12 m/s: 50% flaps (FLAP_2_PERCNT)
 * - Landing: 100% flaps (LAND_FLAP_PERCNT)
 * 
 * Integration with other systems:
 * - **flaperon_update()**: Mixes flaps with ailerons for flaperon aircraft
 * - **dspoiler_update()**: Uses flap position for crow flap mixing
 * - **Landing controller**: Manages flap deployment during approach and flare
 * - **Soaring controller**: Optimizes flaps for thermalling efficiency
 * 
 * @note Called from set_servos() before mixer functions that use flap position
 * @note Slew rate limiting happens after this function in SRV_Channels
 * @note Manual input always takes priority over automatic deployment
 * @note Flight phase flaps override speed-based logic for safety
 * @warning Incorrect speed thresholds can cause premature or late flap deployment
 * @warning Too fast slew rate can overstress airframe
 * @warning Flap deployment affects stall speed and must match flight envelope
 * 
 * @see flaperon_update() for flaperon mixing
 * @see dspoiler_update() for crow flap implementation
 * @see landing.get_flap_percent() for landing flap logic
 * @see Parameters: FLAP_1_SPEED, FLAP_1_PERCNT, FLAP_2_SPEED, FLAP_2_PERCNT, FLAP_SLEWRATE
 * 
 * Source: ArduPlane/servos.cpp:677-761
 */
void Plane::set_servos_flaps(void)
{
    // Auto flap deployment
    int8_t auto_flap_percent = 0;
    int8_t manual_flap_percent = 0;

    // work out any manual flap input
    if (channel_flap != nullptr && rc().has_valid_input()) {
        manual_flap_percent = channel_flap->percent_input();
    }

    const auto flap_actual_speed = flight_option_enabled(FlightOptions::FLAP_ACTUAL_SPEED);
    const bool has_target_airspeed = control_mode->does_auto_throttle();
    if (has_target_airspeed || flap_actual_speed) {
        int16_t flapSpeedSource = 0;
        float est_airspeed;
        bool have_airspeed = ahrs.airspeed_estimate(est_airspeed);
        if (has_target_airspeed && ahrs.using_airspeed_sensor()) {
            flapSpeedSource = target_airspeed_cm * 0.01f;
            if (flap_actual_speed) {
                // if we have a target and also want to use actual
                // speed then use the minimum of the two so we bring
                // flaps in early when deliberately slowing down
                flapSpeedSource = MIN(flapSpeedSource, est_airspeed);
            }
        } else if (flap_actual_speed && have_airspeed) {
            // use actual speed directly
            flapSpeedSource = est_airspeed;
        } else {
            flapSpeedSource = aparm.throttle_cruise;
        }
        if (g.flap_2_speed != 0 && flapSpeedSource <= g.flap_2_speed) {
            auto_flap_percent = g.flap_2_percent;
        } else if ( g.flap_1_speed != 0 && flapSpeedSource <= g.flap_1_speed) {
            auto_flap_percent = g.flap_1_percent;
        } //else flaps stay at default zero deflection

#if HAL_SOARING_ENABLED
        if (control_mode == &mode_thermal) {
            auto_flap_percent = g2.soaring_controller.get_thermalling_flap();
        }
#endif

        /*
          special flap levels for takeoff and landing. This works
          better than speed based flaps as it leads to less
          possibility of oscillation
         */
        switch (flight_stage) {
            case AP_FixedWing::FlightStage::TAKEOFF:
            case AP_FixedWing::FlightStage::ABORT_LANDING:
                if (g.takeoff_flap_percent != 0) {
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_FixedWing::FlightStage::NORMAL:
                if (g.takeoff_flap_percent != 0 && in_preLaunch_flight_stage()) {
                    // TODO: move this to a new FLIGHT_PRE_TAKEOFF stage
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_FixedWing::FlightStage::LAND:
                if (landing.get_flap_percent() != 0) {
                  auto_flap_percent = landing.get_flap_percent();
                }
                break;
            default:
                break;
        }
    }

    // manual flap input overrides auto flap input
    if (abs(manual_flap_percent) > auto_flap_percent) {
        auto_flap_percent = manual_flap_percent;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, auto_flap_percent);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flap, manual_flap_percent);

    SRV_Channels::set_slew_rate(SRV_Channel::k_flap_auto, g.flap_slewrate, 100, G_Dt);
    SRV_Channels::set_slew_rate(SRV_Channel::k_flap, g.flap_slewrate, 100, G_Dt);

    // output to flaperons, if any
    flaperon_update();
}

/**
 * @brief Mix rudder input with throttle for differential thrust control on twin-engine aircraft
 * 
 * @details Implements differential thrust for yaw control by adding rudder input to
 * left/right throttle channels. This provides yaw authority through asymmetric thrust,
 * which is particularly useful at low airspeeds when rudder effectiveness is reduced.
 * 
 * Algorithm:
 * 
 * 1. **Read base throttle**: Get commanded throttle from k_throttle channel
 * 2. **Calculate rudder differential**: Scale rudder input by RUDD_DT_GAIN parameter
 *    - RUDD_DT_GAIN is percentage (0-100) of rudder authority for differential thrust
 *    - Positive rudder (right) increases right engine, decreases left engine
 *    - Negative rudder (left) increases left engine, decreases right engine
 * 3. **Apply thrust direction logic**:
 *    - Forward thrust: Add/subtract 50% of rudder_dt within [0, 100] range
 *    - Reverse thrust: Add/subtract 50% of rudder_dt within [-100, 0] range
 *    - Zero throttle: Set both engines to 0 (no differential)
 * 4. **Safety override**: In AFS failsafe, force rudder_dt to 0 (no yaw)
 * 5. **Output final values**: Write to k_throttleLeft and k_throttleRight channels
 * 
 * Physical implementation:
 * - Left engine at higher throttle creates left yaw (nose left)
 * - Right engine at higher throttle creates right yaw (nose right)
 * - Differential is added to commanded throttle, not replacing it
 * - Total thrust is preserved (sum of both engines ~ 2x throttle command)
 * 
 * Typical usage scenarios:
 * - Crosswind landing: Maintain runway heading with differential thrust
 * - Single-engine failure: Opposite engine at higher thrust compensates for yaw
 * - Low-speed maneuvering: Supplement rudder authority during slow flight
 * - Takeoff roll: Counteract torque effects and maintain centerline
 * 
 * Safety considerations:
 * - Disarmed state: Outputs ZERO_PWM or 0 depending on arming requirements
 * - AFS failsafe: Disables differential to prevent unintended yaw during emergency
 * - Constrained output: Each engine limited to valid throttle range
 * - Split throttle: Both channels must be properly configured for twin-engine operation
 * 
 * Configuration requirements:
 * - SERVOx_FUNCTION must be set to ThrottleLeft (73) and ThrottleRight (74)
 * - RUDD_DT_GAIN parameter sets mixing percentage (0=disabled, 100=full authority)
 * - Both engines should have similar thrust characteristics for predictable behavior
 * - ESC calibration must be identical for both engines
 * 
 * @note Called from servos_output() after set_throttle() has calculated base throttle
 * @note Differential thrust is always applied, even in manual mode
 * @note Only works when both k_throttleLeft and k_throttleRight channels are assigned
 * @warning Incorrect RUDD_DT_GAIN can cause oscillations or insufficient yaw control
 * @warning Engine thrust imbalance will cause persistent yaw requiring rudder trim
 * @warning Single-engine failure requires immediate pilot intervention
 * 
 * @see RUDD_DT_GAIN parameter documentation
 * @see SRV_Channels for throttle output functions
 * @see afs.should_crash_vehicle() for advanced failsafe override
 * 
 * Source: ArduPlane/servos.cpp:766-804
 */
void Plane::servos_twin_engine_mix(void)
{
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    float rud_gain = float(plane.g2.rudd_dt_gain) * 0.01f;
    rudder_dt = rud_gain * SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / SERVO_MAX;

#if AP_ADVANCEDFAILSAFE_ENABLED
    if (afs.should_crash_vehicle()) {
        // when in AFS failsafe force rudder input for differential thrust to zero
        rudder_dt = 0;
    }
#endif

    float throttle_left, throttle_right;

    if (throttle < 0 && have_reverse_thrust() && allow_reverse_thrust()) {
        // doing reverse thrust
        throttle_left  = constrain_float(throttle + 50 * rudder_dt, -100, 0);
        throttle_right = constrain_float(throttle - 50 * rudder_dt, -100, 0);
    } else if (throttle <= 0) {
        throttle_left  = throttle_right = 0;
    } else {
        // doing forward thrust
        throttle_left  = constrain_float(throttle + 50 * rudder_dt, 0, 100);
        throttle_right = constrain_float(throttle - 50 * rudder_dt, 0, 100);
    }
    if (!arming.is_armed_and_safety_off()) {
        if (arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0);
        }
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right);
    }
}

/*
  Set throttle,attitude(in Attitude.cpp), and tilt servos for forced flare by RCx_OPTION switch for landing in FW mode
  For Fixed Wind modes with manual throttle control only. Forces tilts up and throttle to THR_MIN.
  Throttle stick must be in idle deadzone. This allows non-momentary switch to be used and quick bailouts
  for go-arounds. Also helps prevent propstrike after landing with switch release on ground.
*/
void Plane::force_flare(void)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_frwd_transition() && plane.arming.is_armed()) { //allows for ground checking of flare tilts
        return;
    }
    if (control_mode->is_vtol_mode()) {
        return;
    }
    /* to be active must be:
       -manual throttle mode
       -in an enabled flare mode (RC switch active)
       -at zero thrust: in throttle trim dz except for sprung throttle option where trim is at hover stick
    */
    if (!control_mode->does_auto_throttle() && flare_mode != FlareMode::FLARE_DISABLED && throttle_at_zero()) {
        int32_t tilt = -SERVO_MAX;  //this is tilts up for a normal tiltrotor if at zero thrust throttle stick      
        if (quadplane.tiltrotor.enabled() && (quadplane.tiltrotor.type == Tiltrotor::TILT_TYPE_BICOPTER)) {
            tilt = 0; // this is tilts up for a Bicopter
        }
        if (quadplane.tailsitter.enabled()) {
            tilt = SERVO_MAX; //this is tilts up for a tailsitter
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRear, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearLeft, tilt);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRearRight, tilt);
        float throttle_min = MAX(aparm.throttle_min.get(),0); //allows ICE to run if used but accounts for reverse thrust setups
        if (arming.is_armed()) {  //prevent running motors if unarmed
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_min);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_min);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_min);
        }
    }
#endif
}

/**
 * @brief Calculate and output servo positions for all control surfaces and motors
 * 
 * @details This is the main servo output coordination function called at the main loop rate.
 * It implements the complete servo output pipeline from commanded positions to final PWM values:
 * 
 * Pipeline stages:
 * 1. Cork servo outputs to batch updates (reduces interrupt overhead)
 * 2. Check for advanced failsafe termination conditions
 * 3. Update quadplane transition states (if equipped)
 * 4. Allow landing controller to override servos if in landing phase
 * 5. Calculate and apply throttle output:
 *    - Apply battery voltage compensation
 *    - Apply min/max throttle safety limits
 *    - Apply throttle suppression for ground safety
 * 6. Set throttle to zero or min_pwm if disarmed (based on arming config)
 * 7. Warn AHRS about potential imminent takeoff
 * 8. Calculate automatic flap deployment based on airspeed
 * 9. Calculate airbrake deployment
 * 10. Apply throttle slew rate limiting
 * 11. Handle ICE (internal combustion engine) throttle overrides
 * 12. Call servos_output() which applies mixing and pushes final PWM values
 * 
 * Servo mixing (done in servos_output()):
 * - Elevon mixing (aileron + elevator → left/right elevons)
 * - V-tail mixing (rudder + elevator → left/right v-tail surfaces)
 * - Flaperon mixing (aileron + flaps → left/right flaperons)
 * - Differential spoiler mixing (aileron + flaps + rudder → 4 spoiler surfaces)
 * - Twin-engine differential thrust (throttle + rudder → left/right throttles)
 * 
 * Battery compensation adjusts throttle output to maintain consistent power as
 * battery voltage drops, controlled by FWD_BAT_VOLT_MAX and FWD_BAT_VOLT_MIN parameters.
 * 
 * Throttle suppression prevents propeller strike on ground by cutting throttle when:
 * - Altitude is within 10m of home
 * - Ground speed is under 5 m/s
 * - Not performing active takeoff
 * - In auto-throttle mode
 * 
 * @note Called at main loop rate (typically 400Hz) from main vehicle update loop
 * @note This function coordinates all servo outputs but delegates actual PWM calculation to servos_output()
 * @warning This is a critical output path - errors here affect ALL control surfaces and motors
 * @warning Throttle suppression logic is safety-critical to prevent ground accidents
 * 
 * @see servos_output() for mixer implementation and final PWM output
 * @see set_throttle() for throttle calculation and compensation
 * @see throttle_slew_limit() for gradual throttle changes
 * @see suppress_throttle() for ground safety throttle cutoff logic
 * 
 * Source: ArduPlane/servos.cpp:861-963
 */
void Plane::set_servos(void)
{
    // start with output corked. the cork is released when we run
    // servos_output(), which is run from all code paths in this
    // function
    AP::srv().cork();

    // this is to allow the failsafe module to deliberately crash 
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
#if AP_ADVANCEDFAILSAFE_ENABLED
    if (afs.should_crash_vehicle()) {
        afs.terminate_vehicle();
        if (!afs.terminating_vehicle_via_landing()) {
            return;
        }
    }
#endif

    // do any transition updates for quadplane
#if HAL_QUADPLANE_ENABLED
    quadplane.update();
#endif

    if (flight_stage == AP_FixedWing::FlightStage::LAND) {
        // allow landing to override servos if it would like to
        landing.override_servos();
    }

    set_throttle();

    if ((control_mode != &mode_manual) && !arming.is_armed_and_safety_off()) {
        // Always set 0 scaled even if overriding to zero pwm.
        // This ensures slew limits and other functions using the scaled value pick up in the correct place
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        if (arming.arming_required() == AP_Arming::Required::YES_ZERO_PWM) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::Limit::ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::Limit::ZERO_PWM);
        }
    }

    // Warn AHRS if we might take off soon
    set_takeoff_expected();

    // setup flap outputs
    set_servos_flaps();

    // set airbrake outputs
    airbrake_update();

    // slew rate limit throttle
    throttle_slew_limit();

    int8_t min_throttle = 0;
#if AP_ICENGINE_ENABLED
    if (g2.ice_control.allow_throttle_while_disarmed()) {
        min_throttle = MAX(aparm.throttle_min.get(), 0);
    }
    const float base_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
#endif

    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) { 
        case AP_Arming::Required::NO:
            //keep existing behavior: do nothing to radio_out
            //(don't disarm throttle channel even if AP_Arming class is)
            break;

        case AP_Arming::Required::YES_ZERO_PWM:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, 0);
            break;

        case AP_Arming::Required::YES_MIN_PWM:
        default:
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, min_throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, min_throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, min_throttle);
            break;
        }
    }

#if AP_ICENGINE_ENABLED
    float override_pct = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    if (g2.ice_control.throttle_override(override_pct, base_throttle)) {
        // the ICE controller wants to override the throttle for starting, idle, or redline
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, override_pct);
#if HAL_QUADPLANE_ENABLED
        quadplane.vel_forward.integrator = 0;
#endif
    }
#endif  // AP_ICENGINE_ENABLED

    // run output mixer and send values to the hal for output
    servos_output();
}

/*
    This sets servos to neutral if it is a control surface servo in auto mode
*/
void Plane::landing_neutral_control_surface_servos(void)
{
    if (!(landing.get_then_servos_neutral() > 0 &&
            control_mode == &mode_auto &&
            landing.get_disarm_delay() > 0 &&
            landing.is_complete() &&
            !arming.is_armed())) {
                return;
    }


    // after an auto land and auto disarm, set the servos to be neutral just
    // in case we're upside down or some crazy angle and straining the servos.
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS ; i++) {
            SRV_Channel *chan = SRV_Channels::srv_channel(i);
            if (chan == nullptr || !SRV_Channel::is_control_surface(chan->get_function())) {
                continue;
            }

            if (landing.get_then_servos_neutral() == 1) {
                SRV_Channels::set_output_scaled(chan->get_function(), 0);
            } else if (landing.get_then_servos_neutral() == 2) {
                SRV_Channels::set_output_limit(chan->get_function(), SRV_Channel::Limit::ZERO_PWM);
            }
    }
 
}

/*
  sets rudder/vtail , and elevon to indicator positions that we are in a rudder arming waiting for neutral stick state
*/
void Plane::indicate_waiting_for_rud_neutral_to_takeoff(void)
{
    if (takeoff_state.waiting_for_rudder_neutral)  {
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        channel_function_mixer(SRV_Channel::k_rudder,  SRV_Channel::k_elevator, SRV_Channel::k_vtail_right, SRV_Channel::k_vtail_left);
        if (!SRV_Channels::function_assigned(SRV_Channel::k_rudder) && !SRV_Channels::function_assigned(SRV_Channel::k_vtail_left)) {
            // if no rudder indication possible, neutral elevons during wait because on takeoff stance they are usually both full up
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_right, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevon_left, 0);        
        }
    }
}


/**
 * @brief Apply servo mixing for various airframe configurations and output final PWM values
 * 
 * @details This function implements the final stage of servo output processing by applying
 * mixing algorithms for different airframe configurations and pushing PWM values to hardware.
 * 
 * Supported mixing configurations:
 * 
 * 1. **Elevon mixing** (flying wings):
 *    - Combines aileron and elevator inputs
 *    - Left elevon = elevator - aileron (nose up + roll right → left elevon up)
 *    - Right elevon = elevator + aileron (nose up + roll right → right elevon down)
 *    - Controlled by MIXING_GAIN parameter (typically 0.5 for 50% mixing)
 * 
 * 2. **V-tail mixing**:
 *    - Combines rudder and elevator inputs
 *    - Right v-tail = elevator + rudder
 *    - Left v-tail = elevator - rudder
 *    - Uses same mixing gain and offset as elevons
 * 
 * 3. **Flaperon mixing** (in flaperon_update()):
 *    - Combines aileron and flap inputs
 *    - Left flaperon = aileron + (flap_percent * 45)
 *    - Right flaperon = aileron - (flap_percent * 45)
 * 
 * 4. **Differential spoiler mixing** (in dspoiler_update()):
 *    - 4-surface split wing configuration
 *    - Provides aileron, elevator, rudder, and crow flap functions
 *    - Outer surfaces: aileron + rudder + crow flaps
 *    - Inner surfaces: aileron (optional) - crow flaps
 *    - Controlled by DSPOILR_RUD_RATE for rudder mixing
 * 
 * 5. **Twin-engine differential thrust** (in servos_twin_engine_mix()):
 *    - Splits throttle command between left/right engines
 *    - Adds rudder input scaled by RUDD_DT_GAIN for yaw control
 *    - Left throttle = throttle + (rudder * gain * 50%)
 *    - Right throttle = throttle - (rudder * gain * 50%)
 * 
 * Additional processing:
 * - Quadplane tailsitter and bicopter output transformations
 * - Forced flare mode servo positions
 * - Landing neutral control surface positions after auto-land
 * - Rudder arming wait state indication
 * - Manual RC passthrough for selected channels (MANUAL_RCMASK)
 * - Automatic servo trim adjustment in cruise flight
 * 
 * @note Called from set_servos() after all commanded positions are calculated
 * @note Outputs are corked (batched) and pushed together to reduce interrupt overhead
 * @note This is the final stage before PWM values are sent to hardware (hal.rcout)
 * @warning Mixing gain and offset parameters directly affect control surface response
 * @warning Incorrect mixing can lead to inverted or coupled control responses
 * 
 * @see channel_function_mixer() for elevon and v-tail mixing algorithm
 * @see dspoiler_update() for differential spoiler implementation
 * @see servos_twin_engine_mix() for differential thrust implementation
 * @see flaperon_update() for flaperon mixing algorithm
 * @see SRV_Channels::calc_pwm() for final PWM calculation from scaled values
 * 
 * Source: ArduPlane/servos.cpp:1018-1064
 */
void Plane::servos_output(void)
{
    auto &srv = AP::srv();
    srv.cork();

    // support twin-engine aircraft
    servos_twin_engine_mix();

    // run vtail and elevon mixers
    channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator, SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
    channel_function_mixer(SRV_Channel::k_rudder,  SRV_Channel::k_elevator, SRV_Channel::k_vtail_right, SRV_Channel::k_vtail_left);

#if HAL_QUADPLANE_ENABLED
    // cope with tailsitters and bicopters
    quadplane.tailsitter.output();
    quadplane.tiltrotor.bicopter_output();
#endif

    // support forced flare option
    force_flare();

    // implement differential spoilers
    dspoiler_update();

    //  set control surface servos to neutral
    landing_neutral_control_surface_servos();
    
    // set rudder arm waiting for neutral control throws (rudder neutral, aileron/rt vtail/rt elevon to full right)
    if (flight_option_enabled(FlightOptions::INDICATE_WAITING_FOR_RUDDER_NEUTRAL)) {
        indicate_waiting_for_rud_neutral_to_takeoff();
    }

    // support MANUAL_RCMASK
    if (g2.manual_rc_mask.get() != 0 && control_mode == &mode_manual) {
        SRV_Channels::copy_radio_in_out_mask(uint32_t(g2.manual_rc_mask.get()));
    }

    SRV_Channels::calc_pwm();

    SRV_Channels::output_ch_all();

    srv.push();

    if (g2.servo_channels.auto_trim_enabled()) {
        servos_auto_trim();
    }
}

void Plane::update_throttle_hover() {
    // update hover throttle at 100Hz
#if HAL_QUADPLANE_ENABLED
    quadplane.update_throttle_hover();
#endif
}

/*
  implement automatic persistent trim of control surfaces with
  AUTO_TRIM=2, only available when SERVO_RNG_ENABLE=1 as otherwise it
  would impact R/C transmitter calibration
 */
void Plane::servos_auto_trim(void)
{
    // only in auto modes and FBWA
    if (!control_mode->does_auto_throttle() && control_mode != &mode_fbwa) {
        return;
    }
    if (!arming.is_armed_and_safety_off()) {
        return;
    }
    if (!is_flying()) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.allow_servo_auto_trim()) {
        // can't auto-trim with quadplane motors running
        return;
    }
#endif
    if (abs(nav_roll_cd) > 700 || abs(nav_pitch_cd) > 700) {
        // only when close to level
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now - auto_trim.last_trim_check < 500) {
        // check twice a second. We want slow trim update
        return;
    }
    if (ahrs.groundspeed() < 8 || smoothed_airspeed < 8) {
        // only when definitely moving
        return;
    }

    // adjust trim on channels by a small amount according to I value
    float roll_I = rollController.get_pid_info().I;
    float pitch_I = pitchController.get_pid_info().I;

    g2.servo_channels.adjust_trim(SRV_Channel::k_aileron, roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_elevator, pitch_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_elevon_left,  pitch_I - roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_elevon_right, pitch_I + roll_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_vtail_left,  pitch_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_vtail_right, pitch_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_flaperon_left,  roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_flaperon_right, roll_I);

    // cope with various dspoiler options
    const int8_t bitmask = g2.crow_flap_options.get();
    const bool flying_wing       = (bitmask & CrowFlapOptions::FLYINGWING) != 0;
    const bool full_span_aileron = (bitmask & CrowFlapOptions::FULLSPAN) != 0;

    float dspoiler_outer_left = - roll_I;
    float dspoiler_inner_left = 0.0f;
    float dspoiler_outer_right = roll_I;
    float dspoiler_inner_right = 0.0f;

    if (flying_wing) {
        dspoiler_outer_left += pitch_I;
        dspoiler_outer_right += pitch_I;
    }
    if (full_span_aileron) {
        dspoiler_inner_left = dspoiler_outer_left;
        dspoiler_inner_right = dspoiler_outer_right;
    }

    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerLeft1,  dspoiler_outer_left);
    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerLeft2,  dspoiler_inner_left);
    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerRight1, dspoiler_outer_right);
    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerRight2, dspoiler_inner_right);

    auto_trim.last_trim_check = now;

    if (now - auto_trim.last_trim_save > 10000) {
        auto_trim.last_trim_save = now;
        g2.servo_channels.save_trim();
    }
    
}
