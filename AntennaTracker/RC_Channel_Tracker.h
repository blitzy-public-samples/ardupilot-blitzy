/**
 * @file RC_Channel_Tracker.h
 * @brief RC channel class definitions for antenna tracker
 * 
 * @details This file declares tracker-specific RC_Channel and RC_Channels classes
 *          that extend the base RC input handling for antenna tracker mode switching
 *          and channel interpretation. The tracker uses RC inputs for mode control
 *          and manual antenna positioning.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <RC_Channel/RC_Channel.h>

/**
 * @class RC_Channel_Tracker
 * @brief Single RC channel with tracker-specific functionality
 * 
 * @details Extends RC_Channel base class to provide antenna tracker-specific
 *          interpretation of RC channel inputs. This class handles individual
 *          channel processing for tracker mode channels and control inputs.
 *          Currently implements the standard RC_Channel interface without
 *          tracker-specific overrides, but provides a dedicated type for
 *          future tracker-specific channel behaviors.
 * 
 * @note This class is instantiated for each RC channel in the RC_Channels_Tracker array
 */
class RC_Channel_Tracker : public RC_Channel
{

public:

protected:

private:

};

/**
 * @class RC_Channels_Tracker
 * @brief RC channel array manager for antenna tracker
 * 
 * @details Manages all RC input channels for the antenna tracker, providing
 *          tracker-specific channel access and mode channel configuration.
 *          This class extends RC_Channels to handle the complete set of RC
 *          inputs, including mode switching and manual control channels.
 *          
 *          The tracker always trusts RC inputs (no RC failsafe logic),
 *          as losing RC simply means losing manual control while the
 *          tracker continues autonomous operation.
 * 
 * @note Accessed globally via the rc() singleton accessor function
 */
class RC_Channels_Tracker : public RC_Channels
{
public:

    /**
     * @brief Array of tracker-specific RC channel objects
     * 
     * @details Stores all NUM_RC_CHANNELS RC input channels as tracker-specific
     *          RC_Channel_Tracker objects. Each channel processes RC input
     *          according to tracker configuration and mode settings.
     * 
     * @note Accessed through the channel() method which provides bounds checking
     */
    RC_Channel_Tracker obj_channels[NUM_RC_CHANNELS];
    
    /**
     * @brief Get pointer to specific RC channel
     * 
     * @details Provides bounds-checked access to individual RC channels in the
     *          obj_channels array. Returns nullptr if channel number is invalid.
     * 
     * @param[in] chan Channel number (0-based index, 0 to NUM_RC_CHANNELS-1)
     * 
     * @return Pointer to RC_Channel_Tracker object for the requested channel,
     *         or nullptr if channel number is out of range
     * 
     * @note This overrides the base RC_Channels::channel() method to return
     *       tracker-specific RC_Channel_Tracker pointers
     */
    RC_Channel_Tracker *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

    /**
     * @brief Check if RC input is in failsafe condition
     * 
     * @details The antenna tracker always trusts RC inputs and never enters
     *          RC failsafe mode. Loss of RC signal simply means loss of manual
     *          control while the tracker continues autonomous tracking operation.
     *          This differs from other vehicles where RC failsafe triggers
     *          emergency procedures.
     * 
     * @return Always returns false - tracker never considers RC in failsafe
     * 
     * @note This behavior is intentional for antenna tracker operation where
     *       RC is only used for manual override, not safety-critical control
     */
    bool in_rc_failsafe() const override { return false; }

protected:

    /**
     * @brief Get RC channel number used for flight mode selection
     * 
     * @details Returns the channel number configured for mode switching on the
     *          antenna tracker. This method provides the RC channel that the
     *          user can use to switch between tracker modes (MANUAL, SCAN, AUTO).
     * 
     * @return Channel number (0-based) used for mode selection, or -1 if no
     *         mode channel is configured
     * 
     * @note These mode switching callbacks are not presently used on Tracker
     *       in the same way as other vehicles, but the method is provided
     *       for potential future mode switching functionality
     * 
     * @see Tracker::read_control_switch() for actual mode switching implementation
     */
    int8_t flight_mode_channel_number() const override;

};
