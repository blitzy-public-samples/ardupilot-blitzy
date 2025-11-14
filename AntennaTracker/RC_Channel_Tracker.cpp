/**
 * @file RC_Channel_Tracker.cpp
 * @brief RC channel handling and mode switching for antenna tracker
 * 
 * @details This file implements tracker-specific RC input processing and channel
 *          interpretation for the AntennaTracker vehicle type. The antenna tracker
 *          uses a simplified RC input model compared to other vehicles, as it does
 *          not have flight modes in the traditional sense. RC inputs are primarily
 *          used for manual antenna positioning and parameter configuration.
 *          
 *          The RC channel implementation inherits from the common RC_Channel framework
 *          and customizes behavior specific to antenna tracking applications. Unlike
 *          aircraft or rovers, the tracker does not require a dedicated mode channel
 *          since tracking modes are typically switched via ground station commands
 *          rather than RC transmitter switches.
 *          
 *          Typical RC channel assignments for antenna tracker:
 *          - CH1: Yaw (pan) control for manual antenna positioning
 *          - CH2: Pitch (tilt) control for manual antenna positioning
 *          - CH3-CH4: Not typically used
 *          - CH5+: Auxiliary functions (not mode switching)
 * 
 * @note The var_info parameter table for RC channels is auto-generated through
 *       the RC_Channels_VarInfo.h macro system, providing standard RC parameter
 *       structure consistent with other ArduPilot vehicle types.
 * 
 * Source: AntennaTracker/RC_Channel_Tracker.cpp
 */

#include "Tracker.h"

#include "RC_Channel_Tracker.h"

/**
 * @brief Define tracker-specific RC channel subclass names
 * 
 * @details These macro definitions are required by the RC_Channels_VarInfo.h header
 *          to generate the appropriate parameter table (var_info) for the tracker
 *          vehicle type. The macro system allows the RC channel framework to create
 *          vehicle-specific parameter definitions while maintaining code reuse across
 *          all ArduPilot vehicle types (Copter, Plane, Rover, Sub, Tracker).
 *          
 *          RC_CHANNELS_SUBCLASS: Defines the multi-channel container class
 *          RC_CHANNEL_SUBCLASS: Defines the individual channel class
 *          
 *          When RC_Channels_VarInfo.h is included below, it expands these macros
 *          to create the var_info parameter table specific to RC_Channels_Tracker.
 * 
 * @note The var_info array contains AP_Param metadata for all RC channel parameters
 *       (RCx_MIN, RCx_MAX, RCx_TRIM, RCx_REVERSED, RCx_DZ, RCx_OPTION, etc.)
 *       that are exposed to ground control stations for configuration.
 */
#define RC_CHANNELS_SUBCLASS RC_Channels_Tracker
#define RC_CHANNEL_SUBCLASS RC_Channel_Tracker

/**
 * @brief Include standard RC channel parameter definitions
 * 
 * @details Including RC_Channels_VarInfo.h after defining RC_CHANNELS_SUBCLASS
 *          and RC_CHANNEL_SUBCLASS causes the preprocessor to generate the
 *          var_info parameter table for RC_Channels_Tracker. This table defines
 *          all configurable RC channel parameters and their metadata (descriptions,
 *          units, ranges, defaults) used by the parameter system (AP_Param).
 *          
 *          The generated var_info enables:
 *          - Parameter storage/retrieval from EEPROM
 *          - Parameter access via MAVLink parameter protocol
 *          - Ground station configuration of RC channel settings
 *          - Auto-generated parameter documentation
 * 
 * @see RC_Channel/RC_Channels_VarInfo.h for parameter table structure
 * @see libraries/AP_Param for parameter system documentation
 */
#include <RC_Channel/RC_Channels_VarInfo.h>

/**
 * @brief Returns the RC channel number used for flight mode switching
 * 
 * @details This method is part of the RC_Channels framework interface that allows
 *          vehicle-specific customization of which RC channel controls flight mode
 *          selection. For the antenna tracker, this functionality is not used because
 *          the tracker does not have traditional flight modes like aircraft or rovers.
 *          
 *          The antenna tracker operates primarily in these states:
 *          - MANUAL: Direct RC control of pan/tilt servos
 *          - SCAN: Automated scanning pattern
 *          - SERVO_TEST: Servo output testing
 *          - AUTO: Automatic tracking of target vehicle
 *          - INITIALISING: Startup state
 *          - STOP: Servos disabled
 *          
 *          However, these modes are typically commanded via MAVLink ground control
 *          station messages rather than RC channel switches. This simplifies the
 *          RC input requirements for antenna tracker deployments.
 *          
 *          The return value of 1 (rather than a valid channel number in range 0-15)
 *          effectively disables RC-based mode switching for the tracker vehicle type.
 * 
 * @return int8_t Always returns 1 to indicate no RC mode channel is configured
 *                (valid RC channels are numbered 0-15, so 1 is safely out of range
 *                when used as an array index in the calling code)
 * 
 * @note This callback is part of the RC_Channels base class virtual interface
 *       and must be implemented by all vehicle types, even if not actively used.
 * 
 * @note Unlike Copter (typically CH5), Plane (typically CH8), or Rover (typically CH5),
 *       the tracker does not monitor an RC channel for mode changes. Mode commands
 *       are expected to arrive via MAVLink SET_MODE or COMMAND_LONG messages.
 * 
 * @see RC_Channels::read_mode_switch() for how this channel number is used
 * @see Tracker::set_mode() for actual mode change implementation via MAVLink
 * @see Tracker::Mode for available tracker operating modes
 */
int8_t RC_Channels_Tracker::flight_mode_channel_number() const
{
    return 1; // tracker does not have a flight mode channel
}
