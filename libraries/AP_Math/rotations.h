/*
 * rotations.h
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file rotations.h
 * @brief Predefined rotation enumeration for sensor orientation
 * 
 * @details This file defines standard 3D rotations used throughout ArduPilot
 *          for specifying sensor mounting orientations relative to the vehicle
 *          body frame. These rotations are used for IMU orientation, compass
 *          mounting, camera gimbal orientation, and other sensor alignments.
 *          
 *          The rotation system supports both predefined standard orientations
 *          and custom user-defined rotation matrices.
 */

#pragma once

/**
 * @enum Rotation
 * @brief Enumeration of standard 3D rotations for sensor mounting orientations
 * 
 * @details These rotations form a closed mathematical group: the composition
 *          of any two rotations in this set yields another rotation that is
 *          also in the set. This closure property is essential for maintaining
 *          consistency when chaining multiple coordinate transformations.
 *          
 *          Rotation Application:
 *          These rotations typically transform vectors from the sensor frame
 *          to the vehicle body frame. The sensor measures in its local frame,
 *          and this rotation aligns it with the vehicle's reference frame.
 *          
 *          Common Use Cases:
 *          - IMU orientation: Specify how the inertial measurement unit is
 *            mounted relative to the vehicle body frame
 *          - Compass mounting: Define magnetometer orientation for proper
 *            heading calculation
 *          - Camera gimbal orientation: Align camera coordinate system with
 *            vehicle frame for proper targeting and stabilization
 *          - External sensors: Any sensor with a fixed mounting orientation
 *            (rangefinders, optical flow sensors, etc.)
 * 
 * @note Rotation order: Rotations are applied as intrinsic rotations in the
 *       order roll first, then pitch, then yaw. This means each rotation is
 *       performed about the axes of the already-rotated coordinate system.
 * 
 * @note Angle convention: Positive rotation follows the right-hand rule.
 *       Point your right thumb along the positive axis direction; your fingers
 *       curl in the direction of positive rotation.
 * 
 * @warning EEPROM persistence: These rotation values are stored to EEPROM
 *          for configuration persistence across reboots. NEVER change the
 *          numbering of any existing entry! Always add new rotations at the
 *          end before ROTATION_MAX. Changing existing values will cause
 *          incorrect sensor orientation after firmware updates.
 * 
 * @warning MAVLink compatibility: This enumeration must match the
 *          MAV_SENSOR_ORIENTATION enumeration in the MAVLink protocol
 *          specification. Any new rotation added here must also be added
 *          to MAVLink messages to maintain ground station compatibility.
 * 
 * @note Rotation closure property: The mathematical closure of this rotation
 *       set is verified in the rotation test suite. Run tests/test_rotations.cpp
 *       if adding new rotations to verify the closure property is maintained.
 * 
 * @see Matrix3::from_rotation() for converting rotation enum to rotation matrix
 * @see Vector3::rotate() for applying these rotations to vectors
 */
enum Rotation : uint8_t {
    ROTATION_NONE                = 0,  ///< @brief Identity rotation (no rotation applied)
    ROTATION_YAW_45              = 1,  ///< @brief 45° yaw rotation (clockwise when looking down)
    ROTATION_YAW_90              = 2,  ///< @brief 90° yaw rotation (clockwise when looking down)
    ROTATION_YAW_135             = 3,  ///< @brief 135° yaw rotation (clockwise when looking down)
    ROTATION_YAW_180             = 4,  ///< @brief 180° yaw rotation (nose points backward)
    ROTATION_YAW_225             = 5,  ///< @brief 225° yaw rotation (clockwise when looking down)
    ROTATION_YAW_270             = 6,  ///< @brief 270° yaw rotation (clockwise when looking down)
    ROTATION_YAW_315             = 7,  ///< @brief 315° yaw rotation (clockwise when looking down)
    ROTATION_ROLL_180            = 8,  ///< @brief 180° roll rotation (upside down)
    ROTATION_ROLL_180_YAW_45     = 9,  ///< @brief Compound rotation: roll 180° then yaw 45°
    ROTATION_ROLL_180_YAW_90     = 10, ///< @brief Compound rotation: roll 180° then yaw 90°
    ROTATION_ROLL_180_YAW_135    = 11, ///< @brief Compound rotation: roll 180° then yaw 135°
    ROTATION_PITCH_180           = 12, ///< @brief 180° pitch rotation (nose points down)
    ROTATION_ROLL_180_YAW_225    = 13, ///< @brief Compound rotation: roll 180° then yaw 225°
    ROTATION_ROLL_180_YAW_270    = 14, ///< @brief Compound rotation: roll 180° then yaw 270°
    ROTATION_ROLL_180_YAW_315    = 15, ///< @brief Compound rotation: roll 180° then yaw 315°
    ROTATION_ROLL_90             = 16, ///< @brief 90° roll rotation (right side down)
    ROTATION_ROLL_90_YAW_45      = 17, ///< @brief Compound rotation: roll 90° then yaw 45°
    ROTATION_ROLL_90_YAW_90      = 18, ///< @brief Compound rotation: roll 90° then yaw 90°
    ROTATION_ROLL_90_YAW_135     = 19, ///< @brief Compound rotation: roll 90° then yaw 135°
    ROTATION_ROLL_270            = 20, ///< @brief 270° roll rotation (left side down)
    ROTATION_ROLL_270_YAW_45     = 21, ///< @brief Compound rotation: roll 270° then yaw 45°
    ROTATION_ROLL_270_YAW_90     = 22, ///< @brief Compound rotation: roll 270° then yaw 90°
    ROTATION_ROLL_270_YAW_135    = 23, ///< @brief Compound rotation: roll 270° then yaw 135°
    ROTATION_PITCH_90            = 24, ///< @brief 90° pitch rotation (nose points up)
    ROTATION_PITCH_270           = 25, ///< @brief 270° pitch rotation (nose points down)
    ROTATION_PITCH_180_YAW_90    = 26, ///< @brief Compound rotation: pitch 180° then yaw 90° (same as ROTATION_ROLL_180_YAW_270)
    ROTATION_PITCH_180_YAW_270   = 27, ///< @brief Compound rotation: pitch 180° then yaw 270° (same as ROTATION_ROLL_180_YAW_90)
    ROTATION_ROLL_90_PITCH_90    = 28, ///< @brief Compound rotation: roll 90° then pitch 90°
    ROTATION_ROLL_180_PITCH_90   = 29, ///< @brief Compound rotation: roll 180° then pitch 90°
    ROTATION_ROLL_270_PITCH_90   = 30, ///< @brief Compound rotation: roll 270° then pitch 90°
    ROTATION_ROLL_90_PITCH_180   = 31, ///< @brief Compound rotation: roll 90° then pitch 180°
    ROTATION_ROLL_270_PITCH_180  = 32, ///< @brief Compound rotation: roll 270° then pitch 180°
    ROTATION_ROLL_90_PITCH_270   = 33, ///< @brief Compound rotation: roll 90° then pitch 270°
    ROTATION_ROLL_180_PITCH_270  = 34, ///< @brief Compound rotation: roll 180° then pitch 270°
    ROTATION_ROLL_270_PITCH_270  = 35, ///< @brief Compound rotation: roll 270° then pitch 270°
    ROTATION_ROLL_90_PITCH_180_YAW_90 = 36, ///< @brief Compound rotation: roll 90°, pitch 180°, then yaw 90°
    ROTATION_ROLL_90_YAW_270     = 37, ///< @brief Compound rotation: roll 90° then yaw 270°
    ROTATION_ROLL_90_PITCH_68_YAW_293 = 38, ///< @brief Special compound rotation: roll 90°, pitch 68.8°, yaw 293.3° (for specific hardware)
    ROTATION_PITCH_315           = 39, ///< @brief 315° pitch rotation
    ROTATION_ROLL_90_PITCH_315   = 40, ///< @brief Compound rotation: roll 90° then pitch 315°
    ROTATION_PITCH_7             = 41, ///< @brief 7° pitch rotation (small angle correction)
    ROTATION_ROLL_45             = 42, ///< @brief 45° roll rotation
    ROTATION_ROLL_315            = 43, ///< @brief 315° roll rotation
    ///////////////////////////////////////////////////////////////////////
    // Do not add more rotations without checking that there is not a conflict
    // with the MAVLink spec. MAV_SENSOR_ORIENTATION is expected to match our
    // list of rotations here. If a new rotation is added it needs to be added
    // to the MAVLink messages as well.
    ///////////////////////////////////////////////////////////////////////
    ROTATION_MAX,                       ///< @brief Sentinel value marking end of predefined rotations (not a valid rotation)
    ROTATION_CUSTOM_OLD          = 100, ///< @brief Custom rotation matrix (legacy, used in ArduPilot 4.1 and older)
    ROTATION_CUSTOM_1            = 101, ///< @brief Custom rotation matrix slot 1 (user-defined rotation)
    ROTATION_CUSTOM_2            = 102, ///< @brief Custom rotation matrix slot 2 (user-defined rotation)
    ROTATION_CUSTOM_END,                ///< @brief Sentinel value marking end of custom rotation range
};


// definitions used by quaterion and vector3f
#define HALF_SQRT_2 0.70710678118654752440084436210485

/*
Here are the same values in a form suitable for a @Values attribute in
auto documentation:

@Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Yaw45Roll180,10:Yaw90Roll180,11:Yaw135Roll180,12:Pitch180,13:Yaw225Roll180,14:Yaw270Roll180,15:Yaw315Roll180,16:Roll90,17:Yaw45Roll90,18:Yaw90Roll90,19:Yaw135Roll90,20:Roll270,21:Yaw45Roll270,22:Yaw90Roll270,23:Yaw135Roll270,24:Pitch90,25:Pitch270,26:Yaw90Pitch180,27:Yaw270Pitch180,28:Pitch90Roll90,29:Pitch90Roll180,30:Pitch90Roll270,31:Pitch180Roll90,32:Pitch180Roll270,33:Pitch270Roll90,34:Pitch270Roll180,35:Pitch270Roll270,36:Yaw90Pitch180Roll90,37:Yaw270Roll90,38:Yaw293Pitch68Roll180,39:Pitch315,40:Pitch315Roll90,42:Roll45,43:Roll315,100:Custom 4.1 and older,101:Custom 1,102:Custom 2
 */
