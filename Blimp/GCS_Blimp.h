/**
 * @file GCS_Blimp.h
 * @brief Ground Control Station interface for Blimp vehicle
 * 
 * @details This file defines the GCS_Blimp class which provides the blimp-specific
 *          GCS wrapper implementation. It manages MAVLink communication channels
 *          for the blimp vehicle type and handles vehicle identification, sensor
 *          status reporting, and MAVLink backend creation.
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Blimp.h"

/**
 * @class GCS_Blimp
 * @brief Ground Control Station manager for Blimp vehicle
 * 
 * @details GCS_Blimp serves as the GCS factory and channel manager specifically
 *          for the blimp vehicle type. It inherits from GCS_Common and manages
 *          an array of MAVLink communication channels, providing blimp-specific
 *          implementations for vehicle identification (frame type, custom mode)
 *          and sensor status updates. The class acts as the primary interface
 *          between the blimp flight code and ground control stations via MAVLink.
 *          
 *          Key responsibilities:
 *          - Create and manage GCS_MAVLINK_Blimp channel instances
 *          - Report blimp-specific vehicle type and frame information
 *          - Update sensor status flags for telemetry
 *          - Control message sending timing relative to main loop
 */
class GCS_Blimp : public GCS
{
    friend class Blimp; // for access to _chan in parameter declarations

public:

    /**
     * @brief Macro-generated channel accessor methods
     * 
     * @details The following define expands to a pair of methods to retrieve a
     *          pointer to an object of the correct subclass for the link at
     *          offset ofs. These are of the form:
     *          - GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
     *          - const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
     *          
     *          This macro generates type-safe accessors for the MAVLink channel
     *          array, returning GCS_MAVLINK_Blimp pointers instead of base class.
     */
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Blimp);

    /**
     * @brief Update vehicle sensor status flags for MAVLink telemetry
     * 
     * @details Updates the sensor health and presence flags reported in MAVLink
     *          SYS_STATUS messages. This method is called periodically to reflect
     *          the current state of blimp sensors (IMU, compass, barometer, etc.)
     *          to ground control stations. Flags indicate sensor presence, health,
     *          and whether sensors are enabled/in-use.
     * 
     * @note Called from main telemetry update loop
     */
    void update_vehicle_sensor_status_flags(void) override;

    /**
     * @brief Get the vehicle's custom flight mode
     * 
     * @details Returns the current blimp flight mode as a uint32_t value for
     *          MAVLink HEARTBEAT messages. The custom mode allows ground stations
     *          to display blimp-specific mode names (e.g., Land, Manual, Velocity).
     * 
     * @return uint32_t Current blimp flight mode encoded as MAVLink custom_mode
     * 
     * @see Mode::Number enumeration in Blimp.h
     */
    uint32_t custom_mode() const override;
    
    /**
     * @brief Get the MAVLink frame type identifier
     * 
     * @details Returns MAV_TYPE_AIRSHIP to identify this vehicle as a blimp/airship
     *          in MAVLink HEARTBEAT messages. This allows ground stations to select
     *          appropriate UI elements and parameter sets for lighter-than-air vehicles.
     * 
     * @return MAV_TYPE MAV_TYPE_AIRSHIP constant
     * 
     * @note Value defined in MAVLink common.xml message definitions
     */
    MAV_TYPE frame_type() const override;

    /**
     * @brief Get human-readable frame type string
     * 
     * @details Returns a string describing the blimp frame configuration for
     *          display in ground station UI and logging. Typically returns "Blimp"
     *          or a more specific frame description if variants exist.
     * 
     * @return const char* Null-terminated frame description string
     * 
     * @note String must remain valid for lifetime of GCS object
     */
    const char* frame_string() const override;

    /**
     * @brief Check if vehicle initialization is complete
     * 
     * @details Returns true when the blimp has completed its initialization sequence
     *          and is ready for arming. Used to gate certain MAVLink commands and
     *          telemetry that should only be available after full vehicle startup.
     *          Initialization includes sensor calibration, parameter loading, and
     *          subsystem startup.
     * 
     * @return true Vehicle initialization complete, ready for operation
     * @return false Vehicle still initializing, not ready for full operation
     * 
     * @note Some MAVLink commands are rejected until this returns true
     */
    bool vehicle_initialised() const override;

protected:

    /**
     * @brief Get minimum scheduler loop time for MAVLink message sending
     * 
     * @details Returns the minimum amount of time (in microseconds) that must remain
     *          in the main scheduler loop before we are allowed to send any MAVLink
     *          messages. This ensures we prioritise the main flight control loop over
     *          communications, preventing telemetry from interfering with critical
     *          control tasks.
     *          
     *          For blimp, this is set to 250 microseconds to maintain stable control
     *          loop timing while still allowing responsive telemetry updates.
     * 
     * @return uint16_t Minimum remaining loop time in microseconds (250µs for blimp)
     * 
     * @note Lower values allow more frequent messages but risk control loop timing
     * @warning Reducing this value may impact flight stability if control loop is delayed
     */
    uint16_t min_loop_time_remaining_for_message_send_us() const override
    {
        return 250;
    }

    /**
     * @brief Factory method to create new MAVLink backend for a UART
     * 
     * @details Creates and returns a new GCS_MAVLINK_Blimp instance for the specified
     *          UART driver. This method is called by the GCS base class when initializing
     *          MAVLink channels. Uses NEW_NOTHROW to handle memory allocation failures
     *          gracefully (returns nullptr if allocation fails).
     * 
     * @param[in] uart UART driver instance for this MAVLink channel
     * 
     * @return GCS_MAVLINK_Blimp* Pointer to new MAVLink backend, or nullptr if allocation fails
     * 
     * @note Caller is responsible for checking nullptr return
     * @see GCS_MAVLINK_Blimp class for per-channel MAVLink implementation
     */
    GCS_MAVLINK_Blimp *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override
    {
        return NEW_NOTHROW GCS_MAVLINK_Blimp(uart);
    }

};
