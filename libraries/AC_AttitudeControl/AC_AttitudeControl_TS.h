#pragma once

/**
 * @file AC_AttitudeControl_TS.h
 * @brief Tailsitter quadplane attitude control specialization
 * 
 * @details Extends AC_AttitudeControl for tailsitter VTOL aircraft that transition 
 *          between multicopter and fixed-wing flight. Handles unique tailsitter 
 *          geometry where pitch near 90° requires special handling of roll/yaw 
 *          control mixing to maintain intuitive pilot inputs and control authority.
 *          
 *          Tailsitters operate with their propellers oriented vertically in hover
 *          and horizontally in forward flight, requiring attitude control adaptation
 *          as the vehicle transitions through vertical orientations where traditional
 *          Euler angle representations encounter singularities.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#include "AC_AttitudeControl_Multi.h"

/**
 * @class AC_AttitudeControl_TS
 * @brief Tailsitter attitude controller with roll/yaw mixing
 * 
 * @details Provides specialized input methods for tailsitter aircraft. When pitch 
 *          approaches ±90° (vertical orientation), roll and yaw controls are swapped 
 *          or mixed to maintain intuitive pilot inputs. This is critical because 
 *          tailsitters operate in both hover (vertical, nose-up) and forward flight 
 *          (horizontal) modes, and the control mapping must adapt to vehicle orientation.
 *          
 *          Key Features:
 *          - Roll/yaw mixing near vertical pitch angles to preserve control authority
 *          - Body-frame roll input (not Euler roll) to avoid singularities near pitch=90°
 *          - Optional pitch exclusion from relax_attitude_controllers() during transitions
 *          - Plane_controls parameter for fixed-wing-style control mapping
 *          
 *          Coordinate Frame Considerations:
 *          - Body-frame roll: Roll rotation in the vehicle's current frame of reference
 *          - Euler pitch: Pitch angle relative to earth frame
 *          - Near pitch=±90°, body-frame roll becomes earth-frame yaw (coordinate singularity)
 *          
 *          Typical Usage:
 *          - Hover mode: plane_controls=false, standard multicopter-style inputs
 *          - Transition: exclude_pitch=true to preserve pitch target
 *          - Forward flight: plane_controls=true for fixed-wing-style control
 * 
 * @note Used exclusively by tailsitter quadplanes (Plane with Q_FRAME_CLASS=10)
 * @note Roll/yaw mixing improves control authority near vertical pitch angles
 * @note Coordinate frame: body-frame roll (not euler roll) for intuitive control
 * 
 * @warning plane_controls parameter changes control mapping - must match vehicle orientation
 * @warning Roll angle is body-frame, not Euler frame (different near pitch=90°)
 * @warning Coordinate frame confusion can cause loss of control near vertical
 * 
 * @see AC_AttitudeControl Base attitude controller class
 * @see AC_AttitudeControl_Multi Multicopter attitude control (parent class)
 */
class AC_AttitudeControl_TS : public AC_AttitudeControl_Multi
{
public:
    /**
     * @brief Construct tailsitter attitude controller
     * 
     * @details Inherits constructor from AC_AttitudeControl_Multi parent class.
     *          No additional initialization required for tailsitter-specific functionality.
     * 
     * @see AC_AttitudeControl_Multi::AC_AttitudeControl_Multi
     */
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;

    /**
     * @brief Destructor
     * 
     * @details Empty virtual destructor to suppress compiler warning for class with virtual methods.
     */
    virtual ~AC_AttitudeControl_TS() {}

    /**
     * @brief Relax attitude controllers optionally excluding pitch
     * 
     * @details Ensures attitude controllers have zero errors to relax rate controller output.
     *          When exclude_pitch is true, only roll and yaw controllers are relaxed while
     *          pitch controller maintains its current target. This is critical during VTOL
     *          transitions when pitch angle must be maintained (e.g., during transition from
     *          hover to forward flight) while allowing roll and yaw to stabilize.
     *          
     *          Relaxing controllers zeros the error terms and smoothly reduces control output,
     *          preventing abrupt control changes when switching flight modes or after manual
     *          pilot intervention.
     * 
     * @param[in] exclude_pitch If true, pitch controller is not relaxed and preserves 
     *                          pitch target during transitions. If false, all attitude 
     *                          controllers (roll, pitch, yaw) are relaxed.
     * 
     * @note Used during VTOL transitions to maintain pitch angle control
     * @note exclude_pitch prevents unwanted pitch changes during mode transitions
     * @note Called when switching between flight modes or after pilot stick release
     * 
     * @warning Improper use during transitions can cause unexpected pitch changes
     * 
     * @see AC_AttitudeControl::relax_attitude_controllers Base class implementation
     */
    virtual void relax_attitude_controllers(bool exclude_pitch) override;


    /**
     * @brief Set tailsitter attitude with body-frame roll (centidegree interface)
     * 
     * @details Commands a body-frame roll angle (in centidegrees), an Euler pitch angle 
     *          (in centidegrees), and a yaw rate (in centidegrees/s). This is the centidegree
     *          wrapper for the radian implementation.
     *          
     *          See input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad() for full algorithmic details.
     * 
     * @param[in] plane_controls If true, enables fixed-wing-style control mapping with 
     *                           roll/yaw swap/mix near vertical pitch angles
     * @param[in] body_roll_cd   Body-frame roll angle in centidegrees (-9000 to 9000 typical)
     * @param[in] euler_pitch_cd Euler pitch angle in centidegrees (-9000 to 9000 typical)
     * @param[in] euler_yaw_rate_cds Yaw rate in centidegrees/s
     * 
     * @note Centidegree units: 100 centidegrees = 1 degree
     * @note Roll angle is body-frame, not Euler frame
     * 
     * @see input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad Full implementation details
     */
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds) override;
    
    /**
     * @brief Set tailsitter attitude with body-frame roll (radian interface)
     * 
     * @details Commands a body-frame roll angle (in radians), an Euler pitch angle 
     *          (in radians), and a yaw rate (in radians/s). This is the core implementation
     *          for tailsitter attitude control.
     *          
     *          Algorithm:
     *          - When plane_controls is true and pitch approaches ±90° (vertical orientation),
     *            the function swaps or mixes roll and yaw control effects to maintain intuitive
     *            pilot inputs and control authority
     *          - Body-frame roll (not Euler roll) is used to avoid gimbal lock singularities
     *            that occur when pitch = ±90°
     *          - Near vertical pitch, body-frame roll maps to earth-frame yaw due to vehicle
     *            orientation, and this method handles that transformation automatically
     *          
     *          Coordinate Frames:
     *          - body_roll_rad: Roll rotation in vehicle body frame (right-hand rule around 
     *            forward axis)
     *          - euler_pitch_rad: Pitch angle relative to earth frame (nose up = positive)
     *          - euler_yaw_rate_rads: Yaw rate in earth frame
     *          
     *          Usage Patterns:
     *          - Hover mode (vertical): plane_controls=false, standard multicopter behavior
     *          - Transition: plane_controls transitions from false to true as pitch decreases
     *          - Forward flight (horizontal): plane_controls=true, fixed-wing-style control
     * 
     * @param[in] plane_controls If true, enables fixed-wing-style control mapping with 
     *                           roll/yaw swap/mix near vertical pitch angles. Must match 
     *                           vehicle orientation to maintain correct control response.
     * @param[in] body_roll_rad  Body-frame roll angle in radians (-π to π typical range)
     * @param[in] euler_pitch_rad Euler pitch angle in radians (-π/2 to π/2 typical range)
     * @param[in] euler_yaw_rate_rads Yaw rate in radians/s
     * 
     * @note Used by tailsitter quadplanes (Plane with Q_FRAME_CLASS=10)
     * @note Roll/yaw mixing improves control authority near vertical pitch angles
     * @note Body-frame roll provides intuitive control regardless of pitch angle
     * @note Handles coordinate singularities near pitch=±90° automatically
     * 
     * @warning plane_controls parameter must match vehicle orientation state
     * @warning Roll angle is body-frame, NOT Euler frame (critical difference near pitch=90°)
     * @warning Coordinate frame confusion can cause loss of control near vertical orientation
     * @warning Rapid changes in plane_controls during transition can cause control glitches
     * 
     * @see AC_AttitudeControl::input_euler_angle_roll_pitch_yaw Base attitude control
     * @see Quaternion math for attitude representation avoiding singularities
     */
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float body_roll_rad, float euler_pitch_rad, float euler_yaw_rate_rads) override;
};
