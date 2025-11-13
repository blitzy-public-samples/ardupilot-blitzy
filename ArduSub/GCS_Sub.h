/**
 * @file GCS_Sub.h
 * @brief Ground Control Station interface for ArduSub underwater vehicles
 * 
 * @details This file defines the GCS_Sub class which manages Ground Control Station
 *          communication for ArduSub. It inherits from the base GCS class and
 *          provides Sub-specific implementations for MAVLink communication,
 *          vehicle status reporting, and message handling.
 *          
 *          The GCS_Sub class acts as a factory and manager for GCS_MAVLINK_Sub
 *          channel instances, with each channel representing a communication link
 *          to a ground control station (telemetry radio, USB, etc.).
 * 
 * @note This is part of the ArduSub vehicle-specific GCS implementation
 * @see GCS_MAVLINK_Sub for the per-channel MAVLink message handler implementation
 * @see GCS base class in GCS_MAVLink/GCS.h for the common GCS interface
 * 
 * Source: ArduSub/GCS_Sub.h:1-41
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Sub.h"

/**
 * @class GCS_Sub
 * @brief Sub-specific Ground Control Station manager
 * 
 * @details GCS_Sub manages all Ground Control Station communication channels for
 *          ArduSub underwater vehicles. It provides Sub-specific implementations
 *          of vehicle status reporting, custom flight mode encoding, and frame
 *          type identification for MAVLink communication.
 *          
 *          This class serves as a factory for creating GCS_MAVLINK_Sub channel
 *          instances (one per communication link) and coordinates message sending
 *          across all active channels while respecting scheduler timing constraints.
 *          
 *          Key Responsibilities:
 *          - Create and manage GCS_MAVLINK_Sub channel instances for each UART
 *          - Report Sub-specific vehicle sensor status flags to GCS
 *          - Encode Sub custom flight modes for MAVLink HEARTBEAT messages
 *          - Identify vehicle frame type (ROV) for GCS display
 *          - Enforce timing constraints for message sending to prioritize flight control
 *          
 *          Relationship with GCS_MAVLINK_Sub:
 *          GCS_Sub is the vehicle-level manager (singleton accessed via Sub::gcs())
 *          while GCS_MAVLINK_Sub handles per-channel message processing. GCS_Sub
 *          creates GCS_MAVLINK_Sub instances via new_gcs_mavlink_backend().
 * 
 * @note Accessed via Sub::gcs() singleton accessor
 * @warning Message sending timing is critical - min_loop_time_remaining_for_message_send_us()
 *          ensures MAVLink messages don't interfere with main control loop timing
 * 
 * @see GCS_MAVLINK_Sub for per-channel message handling
 * @see GCS base class for common GCS functionality
 * @see Sub main vehicle class
 */
class GCS_Sub : public GCS
{
    friend class Sub; // for access to _chan in parameter declarations

public:

    /**
     * @brief Channel accessor method definitions for Sub GCS instances
     * 
     * @details This macro expands to two methods that retrieve pointers to
     *          GCS_MAVLINK_Sub channel objects by channel offset:
     *          
     *          - GCS_MAVLINK_Sub *chan(const uint8_t ofs) override;
     *          - const GCS_MAVLINK_Sub *chan(const uint8_t ofs) override const;
     *          
     *          These methods provide type-safe access to Sub-specific MAVLink
     *          channel handlers, allowing vehicle code to access the correct
     *          GCS_MAVLINK_Sub subclass rather than the generic base class.
     * 
     * @param ofs Channel offset (0 to MAVLINK_COMM_NUM_BUFFERS-1)
     * @return Pointer to GCS_MAVLINK_Sub instance for the specified channel
     * 
     * @note Channels are indexed from 0, typically: 0=Serial1, 1=Serial2, etc.
     * @see GCS_MAVLINK_Sub for the Sub-specific channel implementation
     */
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Sub);

    /**
     * @brief Update vehicle-specific sensor status flags for MAVLink reporting
     * 
     * @details Updates the MAV_SYS_STATUS_SENSOR flags that are reported in
     *          MAVLink SYS_STATUS messages to ground control stations. This
     *          Sub-specific implementation sets status flags based on sensor
     *          health, calibration state, and availability.
     *          
     *          Sensor flags include health/presence of:
     *          - IMU (gyroscope/accelerometer)
     *          - Magnetometer (compass)
     *          - Barometer (depth sensor)
     *          - GPS (if used for surface positioning)
     *          - Optical flow (if available)
     *          - Vision positioning (if available)
     *          - Battery monitoring
     *          
     *          Called periodically by the GCS message sending scheduler to
     *          ensure ground stations have current sensor health information.
     * 
     * @note Overrides base GCS implementation with Sub-specific sensor priorities
     * @warning Accurate sensor status is critical for operator situation awareness
     * 
     * @see MAV_SYS_STATUS_SENSOR flags in MAVLink common.xml
     * @see GCS::update_vehicle_sensor_status_flags() base implementation
     */
    void update_vehicle_sensor_status_flags() override;

    /**
     * @brief Get current Sub custom flight mode for MAVLink HEARTBEAT
     * 
     * @details Returns the Sub-specific custom mode value that is sent in
     *          MAVLink HEARTBEAT messages to identify the current flight mode
     *          to ground control stations. The custom mode is vehicle-specific
     *          and maps to Sub::Mode enum values.
     *          
     *          Sub custom modes include: STABILIZE, ACRO, ALT_HOLD, AUTO,
     *          GUIDED, CIRCLE, SURFACE, POSHOLD, MANUAL, and MOTOR_DETECT.
     *          
     *          Ground stations use this value to display the current mode
     *          and enable/disable mode-specific UI elements.
     * 
     * @return uint32_t Custom mode value representing current Sub flight mode
     * 
     * @note Custom mode encoding is vehicle-specific (differs from Copter/Plane/Rover)
     * @see Sub::Mode enum for mode definitions
     * @see MAVLink HEARTBEAT message specification
     */
    uint32_t custom_mode() const override;

    /**
     * @brief Get MAVLink frame type for Sub vehicle
     * 
     * @details Returns the MAV_TYPE that identifies this vehicle as an
     *          underwater Remotely Operated Vehicle (ROV) to ground control
     *          stations. This is sent in MAVLink HEARTBEAT messages and
     *          allows ground stations to:
     *          - Display appropriate vehicle icon
     *          - Enable Sub-specific parameters and features
     *          - Apply underwater vehicle control paradigms
     *          - Filter/display relevant telemetry data
     * 
     * @return MAV_TYPE Frame type identifier (MAV_TYPE_SUBMARINE)
     * 
     * @note Always returns MAV_TYPE_SUBMARINE for ArduSub vehicles
     * @see MAV_TYPE enum in MAVLink common.xml
     * @see MAVLink HEARTBEAT message specification
     */
    MAV_TYPE frame_type() const override;

    /**
     * @brief Check if Sub vehicle initialization is complete
     * 
     * @details Returns whether the Sub vehicle has completed its initialization
     *          sequence and is ready for operation. This is used by the GCS
     *          layer to determine if it's safe to send certain commands or
     *          request certain data from the vehicle.
     *          
     *          Initialization includes:
     *          - Sensor calibration and health checks
     *          - Parameter loading from storage
     *          - Hardware initialization (motors, servos, etc.)
     *          - Subsystem startup (EKF, navigation, etc.)
     *          
     *          Ground stations may delay certain operations until this
     *          returns true to avoid sending commands during startup.
     * 
     * @return true if vehicle initialization is complete and vehicle is ready
     * @return false if vehicle is still initializing
     * 
     * @note Reported to GCS in status messages
     * @see Sub::init_ardupilot() for initialization sequence
     */
    bool vehicle_initialised() const override;

protected:

    /**
     * @brief Get minimum scheduler loop time that must remain before sending MAVLink messages
     * 
     * @details Returns the minimum amount of time (in microseconds) that must remain
     *          available in the main scheduler loop before the GCS layer is permitted
     *          to send MAVLink messages. This timing constraint ensures that MAVLink
     *          communication does not interfere with time-critical flight control tasks.
     *          
     *          The main scheduler runs at a fixed rate (typically 50Hz for Sub) and
     *          must complete all high-priority control tasks (attitude control, position
     *          control, sensor updates) before any lower-priority tasks like telemetry.
     *          
     *          If less than this threshold time remains in the current scheduler loop,
     *          MAVLink message sending is deferred to the next loop iteration to avoid
     *          overrunning the loop deadline.
     *          
     *          Sub uses 250 microseconds as the safety margin, which provides adequate
     *          protection for the control loop while still allowing reasonable telemetry
     *          throughput during normal operation.
     * 
     * @return uint16_t Minimum remaining time in microseconds (250µs for Sub)
     * 
     * @note Lower values increase telemetry rate but risk control loop timing violations
     * @warning Reducing this value below 250µs may cause scheduler overruns and affect
     *          vehicle stability, especially on lower-performance flight controllers
     * 
     * @see AP_Scheduler for main loop timing and task scheduling
     * @see GCS::send_message() for message sending that respects this constraint
     */
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;
    }

    /**
     * @brief Create a new GCS_MAVLINK_Sub backend instance for a communication channel
     * 
     * @details Factory method that instantiates a new GCS_MAVLINK_Sub object to handle
     *          MAVLink communication on a specific UART interface. This is called by
     *          the base GCS class during initialization for each enabled telemetry port.
     *          
     *          Each UART that is configured for MAVLink communication (typically serial
     *          ports configured with SERIALn_PROTOCOL = 1 or 2) gets its own dedicated
     *          GCS_MAVLINK_Sub instance to manage message encoding, decoding, and
     *          channel-specific state.
     *          
     *          Uses NEW_NOTHROW for heap allocation to handle out-of-memory conditions
     *          gracefully without exceptions (returns nullptr on allocation failure).
     * 
     * @param[in] uart Reference to HAL UART driver for this communication channel
     * 
     * @return GCS_MAVLINK_Sub* Pointer to newly created channel instance, or nullptr if allocation fails
     * 
     * @note Called once per enabled MAVLink serial port during GCS initialization
     * @warning Allocation failure (returns nullptr) will disable that telemetry channel
     * 
     * @see GCS_MAVLINK_Sub constructor
     * @see AP_HAL::UARTDriver for UART interface definition
     * @see NEW_NOTHROW for non-throwing heap allocation
     * @see GCS::init() for initialization sequence that calls this method
     */
    GCS_MAVLINK_Sub *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Sub(uart);
    }

};
