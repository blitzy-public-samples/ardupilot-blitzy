/**
 * @file APM_Control.h
 * @brief Convenience aggregator header for fixed-wing axis controllers
 * 
 * @details This header provides single-point inclusion of all fixed-wing vehicle
 *          control classes used in ArduPlane. It serves as a compile-time convenience
 *          aggregator that allows vehicle code to include all axis controllers with
 *          a single include directive rather than including each controller individually.
 * 
 *          This is a historical aggregator header maintained for backward compatibility
 *          in fixed-wing vehicle code. It contains no runtime logic, class definitions,
 *          or executable code - it exists purely to simplify include management.
 * 
 * **Included Controllers:**
 * - **AP_RollController**: Roll axis control for fixed-wing aircraft
 * - **AP_PitchController**: Pitch axis control for fixed-wing aircraft
 * - **AP_YawController**: Yaw/rudder control for fixed-wing aircraft
 * - **AP_SteerController**: Ground steering control for taxiing and rover modes
 * 
 * **Usage Pattern:**
 * @code{.cpp}
 * // In ArduPlane vehicle code or fixed-wing control modules:
 * #include <APM_Control/APM_Control.h>
 * 
 * // Now all four controller classes are available:
 * AP_RollController roll_controller;
 * AP_PitchController pitch_controller;
 * AP_YawController yaw_controller;
 * AP_SteerController steer_controller;
 * @endcode
 * 
 * **Alternative Approach:**
 * Individual controller headers can be included directly if only specific
 * controllers are needed:
 * @code{.cpp}
 * #include <APM_Control/AP_RollController.h>
 * #include <APM_Control/AP_PitchController.h>
 * @endcode
 * 
 * @note This header is specific to fixed-wing control. Multicopter attitude
 *       control uses the separate AC_AttitudeControl library.
 * 
 * @note This is a compile-time convenience only - no runtime overhead is
 *       introduced by using this aggregator header versus individual includes.
 * 
 * @see AP_RollController
 * @see AP_PitchController
 * @see AP_YawController
 * @see AP_SteerController
 * 
 * Source: libraries/APM_Control/APM_Control.h
 */

#include "AP_RollController.h"
#include "AP_PitchController.h"
#include "AP_YawController.h"
#include "AP_SteerController.h"
