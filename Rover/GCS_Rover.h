/**
 * @file GCS_Rover.h
 * @brief Rover-specific Ground Control Station interface and telemetry handling
 * 
 * @details This file defines the GCS_Rover class which extends the base GCS
 *          class to provide rover-specific MAVLink telemetry streaming,
 *          command handling, and ground control station communication for
 *          ground vehicle operations. It coordinates with GCS_MAVLink_Rover
 *          to implement the complete ground station communication protocol
 *          for ArduRover vehicles.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Rover.h"

/**
 * @class GCS_Rover
 * @brief Ground Control Station interface for ArduRover vehicles
 * 
 * @details GCS_Rover extends the base GCS class to provide rover-specific
 *          telemetry streaming, mode reporting, and status flag management
 *          for ground vehicle operations. This class serves as the top-level
 *          GCS coordinator for the Rover vehicle, managing multiple MAVLink
 *          communication channels and providing rover-specific implementations
 *          of vehicle state reporting.
 *          
 *          Key responsibilities:
 *          - Custom mode number translation for rover flight modes
 *          - Vehicle frame type identification (rover/boat/balancebot)
 *          - Initialization status reporting
 *          - Sensor status flag updates for rover-specific sensors
 *          - Simple/supersimple mode status (inherited but not used in rover)
 *          - Backend channel creation for rover-specific MAVLink handlers
 *          
 *          Thread Safety: Methods are called from the main vehicle thread
 *          and the GCS thread. Vehicle state access should be coordinated.
 * 
 * @note This class works in conjunction with GCS_MAVLink_Rover which handles
 *       individual MAVLink channel communication and message processing.
 * 
 * @see GCS_MAVLink_Rover
 * @see GCS
 * @see Rover
 */
class GCS_Rover : public GCS
{
    friend class Rover; // for access to _chan in parameter declarations

public:

    /**
     * @brief Channel access method definitions for rover-specific MAVLink backends
     * 
     * @details This macro expands to generate a pair of methods to retrieve a
     *          pointer to the correct GCS_MAVLink_Rover subclass for the MAVLink
     *          link at the specified offset. The expanded methods are:
     *          - GCS_MAVLINK_Rover *chan(const uint8_t ofs) override;
     *          - const GCS_MAVLINK_Rover *chan(const uint8_t ofs) override const;
     *          
     *          These provide type-safe access to rover-specific MAVLink channel
     *          handlers, allowing rover-specific message processing and telemetry.
     * 
     * @note Macro is defined in GCS_MAVLink/GCS.h
     * @see GCS_MAVLINK_Rover
     */
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Rover);

    /**
     * @brief Get the current custom mode number for the rover
     * 
     * @details Returns the current rover mode as a custom mode number suitable
     *          for MAVLink HEARTBEAT messages. This translates internal rover
     *          mode enums to the custom mode field expected by ground control
     *          stations for mode display and logging.
     * 
     * @return uint32_t Custom mode number representing current rover mode
     *                  (e.g., MANUAL=0, HOLD=4, AUTO=10, RTL=11, etc.)
     * 
     * @note Called at HEARTBEAT message rate (typically 1 Hz)
     * @see Mode::mode_number()
     */
    uint32_t custom_mode() const override;

    /**
     * @brief Get the MAVLink frame type for this rover vehicle
     * 
     * @details Returns the MAVLink vehicle frame type identifier that describes
     *          the physical configuration of this rover. Used in HEARTBEAT
     *          messages to inform ground stations of the vehicle type.
     *          
     *          Possible return values for rovers:
     *          - MAV_TYPE_GROUND_ROVER (10) - Standard wheeled/tracked rover
     *          - MAV_TYPE_SURFACE_BOAT (11) - Water surface vehicle
     *          - May return other types based on frame configuration
     * 
     * @return MAV_TYPE MAVLink frame type identifier for this vehicle
     * 
     * @note Called at HEARTBEAT message rate (typically 1 Hz)
     * @see MAV_TYPE enum in MAVLink common.xml
     */
    MAV_TYPE frame_type() const override;

    /**
     * @brief Check if the rover vehicle has completed initialization
     * 
     * @details Returns true if the rover has completed all initialization
     *          sequences and is ready for operation. This includes sensor
     *          initialization, calibration loading, parameter validation,
     *          and system startup checks. Used to gate certain operations
     *          and inform ground stations when the vehicle is ready.
     * 
     * @return true if vehicle initialization is complete
     * @return false if vehicle is still initializing or failed initialization
     * 
     * @note Reported in MAVLink SYS_STATUS messages
     * @see Rover::init_ardupilot()
     */
    bool vehicle_initialised() const override;

    /**
     * @brief Update rover-specific sensor status flags for telemetry
     * 
     * @details Updates the MAVLink sensor health and presence flags based on
     *          current rover sensor states. This includes wheel encoders,
     *          steering sensors, throttle sensors, and rover-specific
     *          navigation sensors. Called periodically to keep ground stations
     *          informed of sensor health status.
     *          
     *          Updates flags for:
     *          - Wheel encoder health and presence
     *          - Steering sensor functionality
     *          - Ground speed sensor availability
     *          - Rover-specific safety sensors
     * 
     * @note Called from GCS update loop at approximately 10 Hz
     * @warning Sensor flags affect ground station warnings and failsafe behavior
     * @see GCS::update_sensor_status_flags()
     */
    void update_vehicle_sensor_status_flags(void) override;

    /**
     * @brief Check if simple mode is currently active
     * 
     * @details Returns true if simple mode input processing is active.
     *          Simple mode is primarily a multicopter feature that adjusts
     *          control inputs relative to initial heading. For rovers,
     *          this typically returns false as the feature is not applicable
     *          to ground vehicle control.
     * 
     * @return true if simple mode is active (typically false for rovers)
     * @return false if simple mode is not active (normal for rovers)
     * 
     * @note This is a copter-specific feature; rover implementation exists
     *       for interface compatibility but typically returns false
     * @see Copter::simple_mode
     */
    bool simple_input_active() const override;

    /**
     * @brief Check if supersimple mode is currently active
     * 
     * @details Returns true if supersimple mode input processing is active.
     *          Supersimple mode is a multicopter feature that simplifies
     *          pilot input even further than simple mode. For rovers,
     *          this typically returns false as the feature is not applicable
     *          to ground vehicle control.
     * 
     * @return true if supersimple mode is active (typically false for rovers)
     * @return false if supersimple mode is not active (normal for rovers)
     * 
     * @note This is a copter-specific feature; rover implementation exists
     *       for interface compatibility but typically returns false
     * @see Copter::supersimple_mode
     */
    bool supersimple_input_active() const override;

protected:

    /**
     * @brief Create a new rover-specific MAVLink backend for a communication channel
     * 
     * @details Factory method that instantiates a new GCS_MAVLink_Rover object
     *          to handle MAVLink communication on the specified UART. This is
     *          called when initializing MAVLink channels to create the appropriate
     *          rover-specific message handlers for each telemetry link.
     *          
     *          Uses NEW_NOTHROW for safe allocation in memory-constrained
     *          embedded environment. Caller must check for nullptr on allocation
     *          failure.
     * 
     * @param[in] uart Reference to the UART driver for this MAVLink channel
     * 
     * @return GCS_MAVLINK_Rover* Pointer to newly created rover MAVLink backend,
     *                             or nullptr if allocation failed
     * 
     * @note Called during GCS initialization for each configured MAVLink port
     * @warning Returns nullptr on allocation failure - caller must handle gracefully
     * @see GCS_MAVLINK_Rover
     * @see AP_HAL::UARTDriver
     */
    GCS_MAVLINK_Rover *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Rover(uart);
    }

};
