/**
 * @file mode_manual.cpp
 * @brief Manual flight mode implementation for Blimp
 * 
 * @details This file implements manual flight mode for lighter-than-air blimp vehicles.
 *          Manual mode provides direct pilot control without any stabilization or assistance.
 *          The pilot's stick inputs are passed directly through to the fin actuators/motors
 *          without any attitude stabilization, position hold, or automated assistance.
 *          
 *          This mode is primarily used for:
 *          - Direct manual control by experienced pilots
 *          - Testing fin/motor response characteristics
 *          - Emergency control when automated systems fail
 *          - Training and familiarization with vehicle dynamics
 *          
 *          Coordinate System:
 *          - X axis (pilot.x): Forward/backward control → front_out (front fins)
 *          - Y axis (pilot.y): Left/right control → right_out (right fins)
 *          - Z axis (pilot.z): Up/down control → down_out (vertical fins)
 *          - Yaw axis (pilot_yaw): Rotation control → yaw_out (yaw fins)
 *          
 * @note This is the simplest flight mode with no automation. All control
 *       responsibility lies with the pilot. There is no altitude hold,
 *       heading hold, or stability augmentation.
 *       
 * @warning Due to the lighter-than-air dynamics of blimps (buoyancy effects,
 *          wind sensitivity), this mode requires careful pilot input to
 *          maintain controlled flight.
 */

#include "Blimp.h"

/**
 * @brief Execute manual flight mode control loop
 * 
 * @details This function implements the core manual mode control logic by:
 *          1. Reading pilot stick inputs via get_pilot_input()
 *          2. Directly mapping those inputs to motor/fin outputs without modification
 *          3. Sending commands to the motors/fins for immediate execution
 *          
 *          Control Mapping (direct passthrough):
 *          - pilot.x (pitch stick) → motors->front_out (front fin control)
 *          - pilot.y (roll stick) → motors->right_out (right fin control)
 *          - pilot.z (throttle stick) → motors->down_out (vertical fin/thrust control)
 *          - pilot_yaw (yaw stick) → motors->yaw_out (yaw fin control)
 *          
 *          No stabilization, limiting, or scaling is applied. The raw pilot
 *          input values are used directly as motor output commands. This provides
 *          maximum control authority but requires continuous pilot attention.
 *          
 *          Algorithm:
 *          Input → Direct Passthrough → Output (no processing)
 *          
 * @note This function is called at the main loop rate (typically 50-400Hz
 *       depending on scheduler configuration). The high execution frequency
 *       ensures responsive control for direct pilot input.
 *       
 * @note Unlike stabilized modes (e.g., Loiter, Auto), this mode does NOT:
 *       - Stabilize attitude or heading
 *       - Hold altitude or position
 *       - Limit control rates or angles
 *       - Apply any automated corrections
 *       - Compensate for wind or buoyancy changes
 *       
 * @warning The direct control nature means the pilot must continuously
 *          manage all aspects of flight including trim, buoyancy compensation,
 *          and environmental disturbances. Loss of pilot input or control
 *          signal will result in the vehicle maintaining last commanded
 *          fin positions with no automated recovery.
 *          
 * @see get_pilot_input() for pilot stick input processing
 * @see Blimp::motors for motor output interface
 * 
 * Source: Blimp/mode_manual.cpp:7-16
 */
void ModeManual::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);
    motors->right_out = pilot.y;
    motors->front_out = pilot.x;
    motors->yaw_out = pilot_yaw;
    motors->down_out = pilot.z;
}
