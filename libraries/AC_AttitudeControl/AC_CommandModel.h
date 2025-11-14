#pragma once

/**
 * @file AC_CommandModel.h
 * @brief Pilot command shaping model for ArduPilot attitude control
 * 
 * @details Implements rate, expo, and time-constant based input shaping for pilot stick inputs.
 *          Converts raw pilot inputs into smooth rate commands with configurable response 
 *          characteristics. This class provides a parameterized model for shaping pilot
 *          control inputs to achieve desired vehicle response characteristics.
 *          
 *          The command model supports three key parameters:
 *          - RATE: Maximum output rate limiting
 *          - EXPO: Exponential response curve for non-linear stick feel
 *          - RATE_TC: Time constant for first-order filtering of rate commands
 *          
 *          Integration: Used by AC_AttitudeControl for converting pilot stick deflection
 *          into desired angular rates for roll, pitch, and yaw control.
 * 
 * @note All parameters are persisted via AP_Param system and configurable via ground station
 */

#include <AP_Param/AP_Param.h>

/**
 * @class AC_CommandModel
 * @brief Parameterized pilot command shaping model
 * 
 * @details Provides a three-parameter model for pilot input shaping that converts
 *          pilot stick deflection into smooth, controlled rate commands. The model
 *          applies exponential curves, rate limiting, and time-constant filtering
 *          to achieve desired vehicle response characteristics.
 *          
 *          Parameter Overview:
 *          - RATE: Maximum output rate (deg/s) - limits how fast vehicle can rotate
 *          - EXPO: Exponential curve factor [0, 1] - shapes stick response from linear
 *                  to exponential for finer control near center with faster response at edges
 *          - RATE_TC: Time constant (seconds) for first-order filtering - controls smoothness
 *                     of rate command changes (lower = crisper, higher = softer)
 *          
 *          Usage Pattern:
 *          This class is typically instantiated as part of attitude control input
 *          methods. Each control axis (roll, pitch, yaw) may have its own command
 *          model instance with independently tunable parameters.
 *          
 *          Parameter Persistence:
 *          All parameters are stored using AP_Param system, allowing runtime modification
 *          via ground control station and persistence across reboots.
 * 
 * @note The var_info[] array is defined in AC_CommandModel.cpp for AP_Param registration
 * @note EXPO = 0 produces linear stick response, EXPO = 1 produces fully exponential response
 * @note RATE_TC provides smoothing; lower values = sharper response, higher = softer response
 * 
 * @see AC_AttitudeControl for integration with attitude control loops
 */
class AC_CommandModel {
public:
    /**
     * @brief Construct a new command model with default parameter values
     * 
     * @param[in] initial_rate  Default maximum rate in deg/s (typically 90-360)
     * @param[in] initial_expo  Default exponential factor [0, 1] (typically 0.0-0.5)
     * @param[in] initial_tc    Default time constant in seconds (typically 0.05-0.5)
     * 
     * @note Default values are used until parameters are loaded from storage or set via GCS
     * @note AP_Param::setup_object_defaults() is called automatically to register parameters
     */
    AC_CommandModel(float initial_rate, float initial_expo, float initial_tc);

    /**
     * @brief Get the rate control input time constant
     * 
     * @return Time constant in seconds for first-order filtering of rate commands
     * 
     * @note Lower values (e.g., 0.05) produce crisp response, higher values (e.g., 0.5) produce soft response
     * @note Valid range: [0, 1] seconds
     */
    float get_rate_tc() const { return rate_tc; }
    
    /**
     * @brief Get the maximum commanded rate
     * 
     * @return Maximum rate in deg/s that will be commanded regardless of pilot input
     * 
     * @note This limits the maximum angular velocity the vehicle can achieve in rate-controlled modes
     * @note Typical values: 90-360 deg/s depending on vehicle type and agility requirements
     */
    float get_rate() const { return rate; }
    
    /**
     * @brief Get the exponential shaping factor
     * 
     * @return Exponential curve factor [0, 1] where 0 = linear response, 1 = fully exponential
     * 
     * @note Expo provides finer control near stick center while maintaining high rates at full deflection
     * @note Higher expo values make the vehicle less sensitive to small stick movements
     * @note Valid range: [-0.5, 1.0], typical values: 0.0-0.5
     */
    float get_expo() const { return expo; }

    /**
     * @brief Set the maximum commanded rate
     * 
     * @param[in] input Maximum rate in deg/s (valid range: 1-360)
     * 
     * @note This immediately updates the rate parameter stored in AP_Param
     * @note Changes take effect on next control loop iteration
     */
    void set_rate(float input) { rate.set(input); }

    /**
     * @brief AP_Param variable information table
     * 
     * @details Defines the parameter group structure for AP_Param system, including:
     *          - RATE: Maximum commanded rate (deg/s)
     *          - EXPO: Exponential shaping factor [-0.5, 1.0]
     *          - RATE_TC: Time constant for rate filtering (seconds)
     * 
     * @note Defined in AC_CommandModel.cpp with full parameter metadata
     * @note Used by AP_Param::setup_object_defaults() during construction
     * 
     * @see AC_CommandModel.cpp for complete parameter definitions with units and ranges
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:
    /**
     * @brief Rate control input time constant (seconds)
     * 
     * @details First-order filter time constant for smoothing rate command changes.
     *          Lower values produce crisper response, higher values produce softer response.
     *          Range: [0, 1] seconds
     *          Units: seconds (s)
     * 
     * @note Typical values: 0.05 (very crisp) to 0.5 (very soft)
     */
    AP_Float rate_tc;
    
    /**
     * @brief Maximum commanded rate (deg/s)
     * 
     * @details Maximum angular velocity that will be commanded regardless of pilot stick deflection.
     *          This parameter limits the maximum rate output of the command model.
     *          Range: [1, 360] deg/s
     *          Units: degrees per second (deg/s)
     * 
     * @note Higher values allow more aggressive vehicle rotation
     * @note Typical values: 90-180 deg/s for stable flight, 360+ deg/s for acrobatic flight
     */
    AP_Float rate;
    
    /**
     * @brief Exponential shaping factor
     * 
     * @details Controls the curvature of the stick-to-rate response curve.
     *          0 = linear response (output proportional to input)
     *          1 = fully exponential response (fine control near center, rapid response at edges)
     *          Range: [-0.5, 1.0]
     *          Units: dimensionless
     * 
     * @note Higher expo values reduce sensitivity to small stick movements near center
     * @note Allows precise control in hover while maintaining full rate authority at stick edges
     */
    AP_Float expo;

private:
    /**
     * @brief Default time constant value provided at construction
     * 
     * @details Stored as const to preserve initial configuration.
     *          Used by AP_Param as default if parameter not loaded from storage.
     */
    const float default_rate_tc;
    
    /**
     * @brief Default maximum rate value provided at construction
     * 
     * @details Stored as const to preserve initial configuration.
     *          Used by AP_Param as default if parameter not loaded from storage.
     */
    const float default_rate;
    
    /**
     * @brief Default expo value provided at construction
     * 
     * @details Stored as const to preserve initial configuration.
     *          Used by AP_Param as default if parameter not loaded from storage.
     */
    const float default_expo;
};

