/**
 * @file qautotune.h
 * @brief QuadPlane autotune implementation for ArduPlane
 * 
 * @details This file contains the QAutoTune class which provides automatic
 *          PID tuning functionality specifically for QuadPlane (hybrid VTOL)
 *          aircraft. The autotune process automatically determines optimal
 *          attitude control PID gains by performing controlled oscillations
 *          and measuring the vehicle response.
 * 
 *          The QAutoTune class extends AC_AutoTune_Multi to integrate with
 *          ArduPlane's QuadPlane flight control system, adapting the multicopter
 *          autotuning algorithm to work within the fixed-wing vehicle context.
 * 
 *          Autotune is typically available in SITL (Software In The Loop)
 *          simulation for safety during development and testing, though it
 *          can be enabled on other platforms via QAUTOTUNE_ENABLED.
 * 
 * @note This feature requires QAUTOTUNE_ENABLED to be defined, which is
 *       automatically enabled for SITL builds with quadplane support
 * 
 * @warning Autotune should only be performed in safe conditions with adequate
 *          altitude and away from obstacles. The vehicle will perform automated
 *          maneuvers that may be unpredictable if tuning parameters are poor.
 * 
 * @see AC_AutoTune_Multi for the base autotuning implementation
 * @see QuadPlane for the quadplane flight control system
 * 
 * Source: ArduPlane/qautotune.h
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include "quadplane.h"
#ifndef QAUTOTUNE_ENABLED
#define QAUTOTUNE_ENABLED (HAL_QUADPLANE_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if QAUTOTUNE_ENABLED

#include <AC_AutoTune/AC_AutoTune_Multi.h>

/**
 * @class QAutoTune
 * @brief QuadPlane-specific implementation of automatic PID tuning
 * 
 * @details QAutoTune extends the AC_AutoTune_Multi base class to provide
 *          automatic tuning of attitude controller PID gains for QuadPlane
 *          VTOL aircraft within the ArduPlane flight control system.
 * 
 *          The autotune state machine operates as follows:
 *          1. Initialization: Verify safe conditions and store initial PIDs
 *          2. Testing Phase: Apply test inputs to each axis sequentially
 *          3. Measurement: Record vehicle response to oscillations
 *          4. Calculation: Compute optimal PID gains from response data
 *          5. Completion: Apply tuned gains or restore original values
 * 
 *          The tuning process tests roll, pitch, and yaw axes independently,
 *          performing controlled oscillations at increasing frequencies to
 *          identify the vehicle's dynamic response characteristics. Pilot
 *          inputs are integrated to maintain position and altitude during
 *          the tuning sequence.
 * 
 *          This class adapts the autotune algorithm to work with ArduPlane's
 *          QuadPlane system by providing plane-specific implementations of
 *          pilot input handling and vertical velocity control.
 * 
 * @note Autotune requires the vehicle to be in QHOVER or QLOITER mode
 * @note Minimum safe altitude is required before autotune can begin
 * @note The process can be aborted by switching modes or disarming
 * 
 * @warning Vehicle will perform automated oscillations during tuning
 * @warning Requires adequate altitude (typically 10m+) and clear airspace
 * @warning Poor initial PIDs may result in unstable behavior - test in SITL first
 * @warning Wind conditions can affect tuning quality - calm conditions preferred
 * 
 * @see AC_AutoTune_Multi for base implementation and tuning algorithm
 * @see QuadPlane for the quadplane control system integration
 * @see AC_AttitudeControl for the attitude controller being tuned
 */
class QAutoTune : public AC_AutoTune_Multi
{
public:
    friend class QuadPlane;

    /**
     * @brief Initialize the autotune system and verify readiness
     * 
     * @details Performs initialization checks to verify the vehicle is ready
     *          for autotuning. Validates that:
     *          - Vehicle is armed and in a suitable flight mode
     *          - Altitude is sufficient for safe tuning maneuvers
     *          - Sensors are healthy and providing valid data
     *          - Initial PID values are within reasonable bounds
     * 
     *          If initialization succeeds, the autotune state machine begins
     *          and the current PID values are saved for potential restoration.
     * 
     * @return true if autotune initialization successful and tuning can begin
     * @return false if preconditions not met or initialization failed
     * 
     * @note Called automatically when autotune mode is entered
     * @note Failure typically results in mode change back to previous mode
     * 
     * @warning Ensure adequate altitude before initiating autotune
     */
    bool init() override;

protected:
    /**
     * @brief Get the pilot's desired vertical climb rate from RC input
     * 
     * @details Reads and processes the pilot's throttle stick input to determine
     *          the desired climb rate during autotune. This allows the pilot to
     *          maintain altitude control while the autotune system tests attitude
     *          responses on the roll, pitch, and yaw axes.
     * 
     *          The climb rate is derived from QuadPlane RC input processing,
     *          applying appropriate scaling, deadzone, and expo curves to convert
     *          raw RC input to a desired velocity in centimeters per second.
     * 
     * @return Desired climb rate in centimeters per second (positive = climb, negative = descend)
     * 
     * @note Allows pilot to maintain altitude during tuning sequence
     * @note Returns 0 if pilot input is within deadzone
     * @note Integrates with QuadPlane's normal climb rate control
     * 
     * @see QuadPlane::get_pilot_desired_climb_rate_cms()
     */
    float get_pilot_desired_climb_rate_cms(void) const override;

    /**
     * @brief Get pilot's desired roll, pitch, and yaw rate from RC inputs
     * 
     * @details Reads pilot stick inputs and converts them to desired body frame
     *          angular rates. During autotune, these inputs allow the pilot to
     *          maintain position and heading while the autotuning algorithm
     *          applies test signals to measure vehicle response.
     * 
     *          Pilot inputs are blended with autotune test signals to allow
     *          manual position correction during the tuning sequence. This
     *          prevents drift while maintaining the integrity of axis testing.
     * 
     * @param[out] des_roll_rad    Desired roll angle in radians (body frame)
     * @param[out] des_pitch_rad   Desired pitch angle in radians (body frame)  
     * @param[out] des_yaw_rate_rads Desired yaw rate in radians per second (body frame)
     * 
     * @note Pilot inputs are in body frame relative to current attitude
     * @note Expo curves and rate limits from QuadPlane RC configuration apply
     * @note Allows position hold while autotune tests individual axes
     * 
     * @see QuadPlane for RC input processing implementation
     * @see AC_AttitudeControl for attitude command execution
     */
    void get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads) override;

    /**
     * @brief Initialize vertical position and velocity limits for autotune
     * 
     * @details Sets up altitude constraints and vertical velocity limits to
     *          ensure safe operation during the autotune sequence. Configures:
     *          - Minimum altitude threshold for autotune operation
     *          - Maximum descent rate to prevent altitude loss
     *          - Maximum climb rate to prevent excessive altitude gain
     *          - Position controller limits for vertical axis
     * 
     *          These limits prevent the vehicle from descending too low or
     *          climbing too high during attitude testing maneuvers, maintaining
     *          safe altitude margins throughout the tuning process.
     * 
     * @note Called during autotune initialization
     * @note Limits are based on QuadPlane configuration parameters
     * @note Prevents altitude excursions during attitude oscillations
     * 
     * @warning Insufficient initial altitude will prevent autotune from starting
     * 
     * @see AC_PosControl for position controller limit implementation
     */
    void init_z_limits() override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Log current PID values and autotune state to dataflash
     * 
     * @details Records the current PID gains for roll, pitch, and yaw rate
     *          controllers along with autotune state information to the dataflash
     *          log. This logging enables:
     *          - Post-flight analysis of the tuning process
     *          - Verification of PID gain progression during tuning
     *          - Troubleshooting of autotune failures or poor results
     *          - Comparison of before/after tuning performance
     * 
     *          Log messages include P, I, and D gains for each axis being tuned,
     *          current autotune state, test axis, and measured response metrics.
     * 
     * @note Only compiled when HAL_LOGGING_ENABLED is defined
     * @note Called periodically during autotune execution
     * @note Log data can be analyzed with MAVExplorer or other log analysis tools
     * 
     * @see AP_Logger for dataflash logging system
     * @see AC_AutoTune_Multi::log_pids() for base implementation
     */
    void log_pids() override;
#endif
};

#endif // QAUTOTUNE_ENABLED
