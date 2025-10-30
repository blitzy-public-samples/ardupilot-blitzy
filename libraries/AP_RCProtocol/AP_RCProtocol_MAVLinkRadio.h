/**
 * @file AP_RCProtocol_MAVLinkRadio.h
 * @brief MAVLink RC_CHANNELS_OVERRIDE message integration for remote control input
 * 
 * @details This protocol backend enables RC (Remote Control) input to be received via MAVLink
 *          RADIO_RC_CHANNELS messages instead of traditional hardware RC receivers. This allows
 *          ground control stations (GCS) or companion computers to provide virtual RC input
 *          for vehicle control, emergency override, or RC signal bridging.
 * 
 *          Primary use cases:
 *          - GCS emergency control when hardware RC signal is lost or unavailable
 *          - Companion computer automation providing RC commands programmatically
 *          - RC signal bridging where MAVLink relays RC from a remote transmitter
 *          - Testing and simulation environments requiring software-generated RC input
 * 
 *          Integration: Works with GCS_MAVLink message handling to receive RADIO_RC_CHANNELS
 *          messages and converts them into standard RC channel values processed by the
 *          AP_RCProtocol system alongside hardware RC inputs.
 * 
 * @warning SECURITY CRITICAL: RC override via MAVLink can directly control vehicle actuators.
 *          Ensure MAVLink communication is properly secured (encrypted links, authentication)
 *          if using this feature in operational environments. Unauthorized access to MAVLink
 *          could allow complete vehicle control takeover.
 * 
 * @note Message format: mavlink_radio_rc_channels_t contains array of channel values plus
 *       metadata (count, flags). Channel values are PWM microseconds (1000-2000μs range).
 * 
 * @note Failsafe behavior: If MAVLink RC messages stop arriving, the protocol backend will
 *       timeout after configured period, triggering RC failsafe as with hardware RC loss.
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

#include "AP_RCProtocol.h"

/**
 * @class AP_RCProtocol_MAVLinkRadio
 * @brief RC protocol backend for receiving RC channel input via MAVLink RADIO_RC_CHANNELS messages
 * 
 * @details This backend implements virtual RC input reception through MAVLink messaging rather
 *          than hardware radio receivers. It processes RADIO_RC_CHANNELS messages sent by
 *          ground control stations or companion computers, converting MAVLink channel data
 *          into the standard RC channel representation used throughout ArduPilot.
 * 
 *          The backend integrates with the AP_RCProtocol framework as one of several possible
 *          RC input sources (SBUS, PPM, DSM, CRSF, etc.), allowing MAVLink RC to be selected
 *          dynamically or used as failover when hardware RC is unavailable.
 * 
 *          Message reception is handled by GCS_MAVLink which routes RADIO_RC_CHANNELS messages
 *          to this backend via the update_radio_rc_channels() method. The backend updates
 *          internal RC channel state which is then consumed by the RC_Channels library for
 *          flight control, mode switching, and auxiliary functions.
 * 
 *          Update rate: Dependent on MAVLink message transmission rate from GCS/companion
 *          computer. Typical rates are 10-50Hz. Higher rates provide smoother control but
 *          increase telemetry bandwidth usage.
 * 
 *          Failsafe: If messages stop arriving, the backend will timeout and trigger RC
 *          failsafe procedures (RTL, Land, etc.) as configured in vehicle parameters.
 * 
 * @see AP_RCProtocol_Backend
 * @see GCS_MAVLink
 * @see RC_Channels
 */
class AP_RCProtocol_MAVLinkRadio : public AP_RCProtocol_Backend {
public:

    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    /**
     * @brief Process incoming MAVLink RADIO_RC_CHANNELS message and update RC channel values
     * 
     * @details This method is called by GCS_MAVLink when a RADIO_RC_CHANNELS message is received
     *          from a ground control station or companion computer. It extracts the channel values
     *          from the MAVLink packet, validates them, and updates the internal RC channel state
     *          that will be used for vehicle control.
     * 
     *          Processing steps:
     *          1. Validate packet pointer and channel count
     *          2. Extract channel values from mavlink_radio_rc_channels_t.channels[] array
     *          3. Convert MAVLink channel format to internal RC representation if needed
     *          4. Update last-received timestamp for failsafe monitoring
     *          5. Mark channels as valid/updated for RC_Channels library consumption
     * 
     *          The mavlink_radio_rc_channels_t packet structure contains:
     *          - channels[]: Array of up to 32 channel values (actual count in 'count' field)
     *          - count: Number of valid channels in the channels[] array
     *          - flags: Message flags (reserved for future use)
     *          - target_system: Destination system ID (should match vehicle system ID)
     *          - target_component: Destination component ID
     * 
     *          Channel value format: PWM microseconds in standard RC range (typically 1000-2000μs)
     *          - 1000μs: Minimum position (e.g., throttle off, stick full left/down)
     *          - 1500μs: Center/neutral position
     *          - 2000μs: Maximum position (e.g., full throttle, stick full right/up)
     * 
     *          Update rate: Called each time a RADIO_RC_CHANNELS message is received, typically
     *          10-50Hz depending on GCS/companion computer transmission rate. Higher rates provide
     *          smoother control response but consume more telemetry bandwidth.
     * 
     *          Failsafe timeout: If this method is not called for a configured timeout period
     *          (typically 1-2 seconds), the RC input will be marked invalid and RC failsafe
     *          procedures will trigger (RTL, Land, etc. as configured).
     * 
     * @param[in] packet Pointer to mavlink_radio_rc_channels_t message structure containing
     *                   channel values and metadata. Must not be nullptr. Channel values are
     *                   in PWM microseconds (1000-2000μs range). Valid channel count is
     *                   specified in packet->count field (maximum 32 channels).
     * 
     * @note This method is called from GCS_MAVLink context, typically at MAVLink message
     *       reception rate (not scheduler task rate). Ensure thread-safety if accessing
     *       shared RC state.
     * 
     * @note Channel mapping: MAVLink channel indices map directly to ArduPilot RC channel
     *       numbers (channels[0] = RC1, channels[1] = RC2, etc.). Standard channel assignments
     *       typically follow: RC1=Roll, RC2=Pitch, RC3=Throttle, RC4=Yaw, RC5+=Aux functions.
     * 
     * @warning Malformed or malicious MAVLink packets could provide out-of-range channel values
     *          or invalid channel counts. Implementation must validate all input data before
     *          updating RC state to prevent unsafe vehicle behavior.
     * 
     * @see GCS_MAVLink::handle_radio_rc_channels()
     * @see AP_RCProtocol_Backend::add_input()
     * @see RC_Channels
     */
    void update_radio_rc_channels(const mavlink_radio_rc_channels_t* packet) override;
};

#endif // AP_RCPROTOCOL_MAVLINK_RADIO_ENABLED

