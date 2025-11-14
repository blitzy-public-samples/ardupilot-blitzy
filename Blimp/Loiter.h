/**
 * @file Loiter.h
 * @brief Loiter controller class definition for blimp position and velocity control
 * 
 * @details This file defines the Loiter class which provides position and velocity
 *          control for airship station-keeping operations. The controller maintains
 *          blimp position and orientation in 3D space using PID-based control loops
 *          for each axis (x, y, z, yaw). The Vector4b helper class provides boolean
 *          flags to enable/disable control on individual axes.
 * 
 * @note Blimp control operates in NED (North-East-Down) coordinate frame
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

/**
 * @class Vector4b
 * @brief 4-dimensional boolean vector for axis enable/disable control
 * 
 * @details Helper class that stores boolean flags for x, y, z, and yaw axes.
 *          Used by the Loiter controller to selectively enable or disable
 *          control on individual axes. This allows partial control authority
 *          where some axes may be controlled while others remain free.
 *          
 *          The class provides logical AND (&&) and OR (||) operators for
 *          combining axis enable states.
 */
class Vector4b
{
public:
    bool    x;      ///< Enable flag for x-axis (North in NED frame)
    bool    y;      ///< Enable flag for y-axis (East in NED frame)
    bool    z;      ///< Enable flag for z-axis (Down in NED frame)
    bool    yaw;    ///< Enable flag for yaw rotation

    /**
     * @brief Default constructor initializing all axes to disabled (false)
     */
    constexpr Vector4b()
        : x(0)
        , y(0)
        , z(0)
        , yaw(0) {}

    /**
     * @brief Constructor with explicit axis enable states
     * 
     * @param[in] x0   Enable state for x-axis
     * @param[in] y0   Enable state for y-axis
     * @param[in] z0   Enable state for z-axis
     * @param[in] yaw0 Enable state for yaw axis
     */
    constexpr Vector4b(const bool x0, const bool y0, const bool z0, const bool yaw0)
        : x(x0)
        , y(y0)
        , z(z0)
        , yaw(yaw0) {}

    /**
     * @brief Logical AND operator for combining enable states
     * 
     * @details Performs element-wise AND operation. Resulting axis is enabled
     *          only if both input vectors have that axis enabled.
     * 
     * @param[in] v Second vector for AND operation
     * @return Vector4b with element-wise AND of enable states
     */
    Vector4b operator &&(const Vector4b &v)
    {
        Vector4b temp{x && v.x, y && v.y, z && v.z, yaw && v.yaw};
        return temp;
    }

    /**
     * @brief Logical OR operator for combining enable states
     * 
     * @details Performs element-wise OR operation. Resulting axis is enabled
     *          if either input vector has that axis enabled.
     * 
     * @param[in] v Second vector for OR operation
     * @return Vector4b with element-wise OR of enable states
     */
    Vector4b operator ||(const Vector4b &v)
    {
        Vector4b temp{x || v.x, y || v.y, z || v.z, yaw || v.yaw};
        return temp;
    }

};



/**
 * @class Loiter
 * @brief Position and velocity control for blimp station-keeping operations
 * 
 * @details The Loiter class provides PID-based position and velocity control
 *          for maintaining a blimp's position and orientation in 3D space.
 *          It supports two control modes:
 *          
 *          1. Position Control (run): Maintains a target position and yaw
 *          2. Velocity Control (run_vel): Tracks target velocities
 *          
 *          The controller operates in the NED (North-East-Down) coordinate frame
 *          and allows selective enabling/disabling of individual control axes
 *          through the axes_disabled parameter. This is particularly useful for
 *          airships where buoyancy affects vertical control differently than
 *          horizontal control.
 *          
 *          The scaler members allow adjustment of control authority on different
 *          axes to account for the asymmetric dynamics of lighter-than-air vehicles.
 * 
 * @note All position values are in meters, velocities in m/s, yaw in radians
 * @note Controller expects to be called at the main loop rate for proper PID operation
 * @warning Improper scaler values can lead to unstable blimp behavior
 */
class Loiter
{
public:
    friend class Blimp;  ///< Friend access for main vehicle class
    friend class Fins;   ///< Friend access for fin control class

    float scaler_xz;     ///< Control authority scaler for x and z axes (horizontal and vertical)
    float scaler_yyaw;   ///< Control authority scaler for y axis and yaw rotation

    /**
     * @brief Constructor for Loiter controller
     * 
     * @details Initializes the loiter controller with default scaler values of 1.0
     *          for all axes. The loop_rate parameter is provided for future use in
     *          time-dependent control calculations.
     * 
     * @param[in] loop_rate Main loop execution rate in Hz (typically 50-400 Hz)
     * 
     * @note Scaler values default to 1.0 (full control authority)
     */
    Loiter(uint16_t loop_rate)
    {
        scaler_xz = 1;
        scaler_yyaw = 1;
    };

    /**
     * @brief Run position control mode to maintain target position and yaw
     * 
     * @details Executes PID control loops to maintain the blimp at a target position
     *          and orientation in the NED coordinate frame. Calculates position errors,
     *          applies PID corrections, and outputs control commands to the fins.
     *          
     *          Individual axes can be disabled using the axes_disabled parameter,
     *          where true = disabled, false = enabled. This allows partial control
     *          such as maintaining horizontal position while allowing free vertical drift.
     *          
     *          The controller uses the scaler members to adjust control authority,
     *          which is important for blimps where aerodynamic effectiveness differs
     *          significantly between horizontal and vertical axes.
     * 
     * @param[in] target_pos Target position in NED frame (meters)
     * @param[in] target_yaw Target yaw angle (radians, 0 = North, positive = clockwise)
     * @param[in] axes_disabled Boolean flags indicating which axes to disable
     *                          (x = North, y = East, z = Down, yaw = rotation)
     * 
     * @note Must be called at main loop rate for proper PID integral and derivative terms
     * @note Position is in meters relative to home/origin in NED frame
     * @warning Disabling all axes will result in no control output
     */
    void run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled);
    
    /**
     * @brief Run velocity control mode to track target velocities
     * 
     * @details Executes velocity control to make the blimp track target velocities
     *          in the NED coordinate frame. This mode is useful for smooth transitions
     *          and trajectory following where velocity tracking is more important than
     *          precise position holding.
     *          
     *          Similar to position control, individual axes can be disabled through
     *          the axes_disabled parameter. The controller applies velocity feedback
     *          and outputs fin commands to achieve the desired velocities.
     *          
     *          Velocity control is often more responsive than position control for
     *          lighter-than-air vehicles which have significant inertia and environmental
     *          disturbances from wind.
     * 
     * @param[in] target_vel Target velocity vector in NED frame (m/s)
     * @param[in] target_vel_yaw Target yaw rate (rad/s, positive = clockwise)
     * @param[in] axes_disabled Boolean flags indicating which axes to disable
     *                          (x = North velocity, y = East velocity, z = Down velocity, yaw = yaw rate)
     * 
     * @note Must be called at main loop rate for proper control response
     * @note Velocities are in m/s in NED frame, yaw rate in rad/s
     * @warning Large velocity commands may exceed actuator authority or cause instability
     */
    void run_vel(Vector3f& target_vel, float& target_vel_yaw, Vector4b axes_disabled);
};
