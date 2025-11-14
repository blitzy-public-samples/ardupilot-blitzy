/**
 * @file GCS_Copter.h
 * @brief Ground Control Station (GCS) interface for ArduCopter
 * 
 * @details This file defines the GCS_Copter class which provides the copter-specific
 *          implementation of the Ground Control Station interface. It manages MAVLink
 *          communication with ground control stations, telemetry streaming, and
 *          vehicle status reporting for multicopter vehicles.
 *          
 *          Architecture:
 *          - GCS_Copter extends the base GCS class with copter-specific behavior
 *          - Manages multiple MAVLink communication channels (radio, telemetry, USB, etc.)
 *          - Each channel is handled by a GCS_MAVLINK_Copter instance
 *          - Provides copter-specific MAVLink message handlers and status reporting
 *          - Coordinates telemetry streaming priorities with main flight control loop
 *          
 *          Relationship between classes:
 *          - GCS (base): Generic GCS interface for all vehicle types
 *          - GCS_Copter (this class): Copter-specific GCS manager/coordinator
 *          - GCS_MAVLINK_Copter: Per-channel MAVLink message handler implementation
 *          
 *          The GCS system operates with strict timing constraints to ensure flight
 *          control loop priority. Telemetry messages are only sent when sufficient
 *          time remains in the scheduler loop (see min_loop_time_remaining_for_message_send_us).
 * 
 * @note This is a safety-critical interface - MAVLink commands affect vehicle behavior
 * @warning Modifications to timing parameters can impact flight control stability
 * 
 * @see GCS base class in libraries/GCS_MAVLink/GCS.h
 * @see GCS_MAVLINK_Copter for per-channel MAVLink handling
 * @see Copter class for main vehicle interface
 */

#pragma once

#include <GCS_MAVLink/GCS.h>
#include "GCS_MAVLink_Copter.h"

/**
 * @class GCS_Copter
 * @brief Copter-specific Ground Control Station interface manager
 * 
 * @details GCS_Copter provides the top-level GCS interface for multicopter vehicles,
 *          extending the generic GCS base class with copter-specific functionality:
 *          
 *          Key Responsibilities:
 *          - Manage multiple MAVLink communication channels (typically 4-6 channels)
 *          - Report copter-specific vehicle status (flight mode, sensor health, etc.)
 *          - Convert copter flight modes to MAVLink custom_mode encoding
 *          - Provide copter-specific MAV_TYPE and frame type identification
 *          - Control telemetry streaming timing to prioritize flight control
 *          - Create and manage GCS_MAVLINK_Copter backend instances for each channel
 *          
 *          Communication Architecture:
 *          - Each physical link (radio, telemetry, USB, etc.) has its own MAVLink channel
 *          - GCS_Copter coordinates these channels and provides vehicle-level context
 *          - Individual channels are handled by GCS_MAVLINK_Copter instances
 *          - Base GCS class provides common MAVLink protocol infrastructure
 *          
 *          Timing Constraints:
 *          - Flight control loop runs at 400Hz (2.5ms period) on most platforms
 *          - GCS messages only sent when >250μs remains in loop (see min_loop_time_remaining)
 *          - This ensures telemetry never interferes with critical flight control
 *          
 *          MAVLink Protocol Support:
 *          - Supports MAVLink 1.0 and 2.0 protocols
 *          - Handles parameter protocol (read/write/list parameters)
 *          - Mission protocol (upload/download waypoint missions)
 *          - Command protocol (MAV_CMD_* commands for vehicle control)
 *          - Telemetry streaming (HEARTBEAT, ATTITUDE, GPS, etc.)
 *          - File transfer protocol (FTP for logs, terrain data)
 * 
 * @note Friend class Copter allows main vehicle to access internal channel array
 * @warning This is a safety-critical class - handles vehicle control commands
 */
class GCS_Copter : public GCS
{
    /**
     * @brief Friend declaration for Copter class to access internal channel array
     * 
     * @details The Copter class requires access to the internal _chan array for
     *          parameter declarations that are associated with specific MAVLink channels.
     *          This allows parameters like SERIALx_PROTOCOL to be properly configured.
     */
    friend class Copter; // for access to _chan in parameter declarations

public:

    /**
     * @brief Channel accessor method definitions macro
     * 
     * @details This macro (GCS_MAVLINK_CHAN_METHOD_DEFINITIONS) expands to generate
     *          two methods that retrieve pointers to the copter-specific MAVLink
     *          backend for a given channel offset. The generated methods are:
     *          
     *          GCS_MAVLINK_Copter *chan(const uint8_t ofs) override;
     *          const GCS_MAVLINK_Copter *chan(const uint8_t ofs) override const;
     *          
     *          These methods allow type-safe access to the copter-specific MAVLink
     *          channel handlers, enabling copter-specific message handling and
     *          telemetry streaming for each communication link.
     *          
     * @param ofs Channel offset (0-based index, typically 0-5 for available channels)
     * @return Pointer to GCS_MAVLINK_Copter instance for the specified channel
     * 
     * @note The macro is defined in the base GCS class header
     * @see GCS_MAVLINK_Copter for per-channel message handling implementation
     */
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Copter);

    /**
     * @brief Update vehicle sensor status flags for MAVLink SYS_STATUS reporting
     * 
     * @details This method updates the sensor health and availability flags that are
     *          reported in MAVLink SYS_STATUS messages. It queries copter-specific
     *          sensors and systems to determine their operational status:
     *          
     *          Monitored Sensors/Systems:
     *          - 3D gyroscope (MAV_SYS_STATUS_SENSOR_3D_GYRO)
     *          - 3D accelerometer (MAV_SYS_STATUS_SENSOR_3D_ACCEL)
     *          - 3D magnetometer (MAV_SYS_STATUS_SENSOR_3D_MAG)
     *          - Absolute pressure (barometer)
     *          - GPS position
     *          - Optical flow sensor
     *          - Vision position estimation
     *          - Laser rangefinder
     *          - Battery status
     *          - RC receiver
     *          - Motor outputs/ESCs
     *          
     *          Status Flags:
     *          - Present: Sensor is detected and configured
     *          - Enabled: Sensor is actively being used
     *          - Healthy: Sensor is providing valid data
     *          
     *          This information is critical for ground stations to display vehicle
     *          health and warn operators of sensor failures or degraded operation.
     *          
     * @note Called periodically by GCS update loop (typically at 1-10Hz)
     * @warning Accurate sensor status is critical for safe flight - pre-arm checks
     *          rely on these flags
     * 
     * @see MAVLink SYS_STATUS message definition
     * @see Copter::update_sensor_status_flags() for sensor health checks
     */
    void update_vehicle_sensor_status_flags(void) override;

    /**
     * @brief Get current copter flight mode as MAVLink custom_mode value
     * 
     * @details Converts the current ArduCopter flight mode to the MAVLink custom_mode
     *          encoding used in HEARTBEAT messages. The custom_mode is a vehicle-specific
     *          encoding of the flight mode that ground stations use to display the
     *          current mode to the operator.
     *          
     *          ArduCopter Flight Modes (examples):
     *          - STABILIZE (0): Manual rate control with self-leveling
     *          - ACRO (1): Manual rate control, no self-leveling
     *          - ALT_HOLD (2): Altitude hold with manual horizontal control
     *          - AUTO (3): Autonomous waypoint mission
     *          - GUIDED (4): External position/velocity control
     *          - LOITER (5): Position hold
     *          - RTL (6): Return to launch
     *          - LAND (9): Autonomous landing
     *          - And many more...
     *          
     *          The custom_mode value directly corresponds to the Mode::Number enum
     *          defined in mode.h.
     * 
     * @return Current flight mode as uint32_t MAVLink custom_mode encoding
     * 
     * @note Called for every HEARTBEAT message (typically 1Hz)
     * @see Mode::Number enum in mode.h for complete mode list
     * @see MAVLink HEARTBEAT message custom_mode field
     */
    uint32_t custom_mode() const override;

    /**
     * @brief Get the MAVLink frame type identifier for this copter
     * 
     * @details Returns the MAV_TYPE identifier that describes the airframe type
     *          for this multicopter vehicle. This is reported in MAVLink HEARTBEAT
     *          messages and allows ground stations to display appropriate icons,
     *          configure suitable parameters, and provide frame-specific features.
     *          
     *          Common Copter Frame Types:
     *          - MAV_TYPE_QUADROTOR: Standard quadcopter (X or + configuration)
     *          - MAV_TYPE_HEXAROTOR: Hexacopter (6 motors)
     *          - MAV_TYPE_OCTOROTOR: Octocopter (8 motors)
     *          - MAV_TYPE_TRICOPTER: Tricopter (3 motors with servo yaw)
     *          - MAV_TYPE_COAXIAL: Coaxial copter
     *          - MAV_TYPE_HELICOPTER: Traditional helicopter
     *          
     *          The frame type is determined from the configured motor layout
     *          (FRAME_CLASS parameter) and affects motor mixing, control allocation,
     *          and failsafe behavior.
     * 
     * @return MAV_TYPE enum value identifying the copter frame configuration
     * 
     * @note Called for every HEARTBEAT message (typically 1Hz)
     * @see MAVLink MAV_TYPE enum definition
     * @see AP_Motors::get_frame_mav_type() for frame type detection
     * @see FRAME_CLASS parameter in Copter parameters
     */
    MAV_TYPE frame_type() const override;

    /**
     * @brief Get human-readable frame type string for logging and display
     * 
     * @details Returns a text string describing the copter frame configuration,
     *          used for logging, console output, and some MAVLink STATUS_TEXT
     *          messages. This provides more detailed frame information than the
     *          MAV_TYPE enum value.
     *          
     *          Example Frame Strings:
     *          - "Quad"
     *          - "Hexa"
     *          - "Octa"
     *          - "OctaQuad"
     *          - "Tri"
     *          - "Y6"
     *          - "Heli"
     *          
     *          The string reflects both the motor count and geometric arrangement.
     * 
     * @return Pointer to constant string with frame type name
     * 
     * @note String lifetime is static - safe to store pointer
     * @see AP_Motors_Class::get_frame_string() for frame string generation
     */
    const char* frame_string() const override;

    /**
     * @brief Check if vehicle has completed initialization and is ready to arm
     * 
     * @details Returns true if the copter has completed all initialization sequences
     *          and is ready for arming and flight. This includes:
     *          
     *          Initialization Requirements:
     *          - EKF (Extended Kalman Filter) has initialized and converged
     *          - Compass calibration loaded (if compass enabled)
     *          - Barometer calibration complete
     *          - RC receiver detected and validated
     *          - Motor outputs initialized
     *          - Mode controller initialized
     *          - Home position set (for RTL failsafe)
     *          
     *          This status is used by:
     *          - Pre-arm checks (vehicle must be initialized to arm)
     *          - Ground stations (to enable/disable arm button)
     *          - MAVLink telemetry (reported in system status)
     *          
     *          Initialization typically completes within 5-15 seconds of boot,
     *          depending on GPS lock time and EKF convergence.
     * 
     * @return true if vehicle initialization is complete, false if still initializing
     * 
     * @note Ground stations should not allow arming until this returns true
     * @warning Attempting to arm before initialization completes will fail pre-arm checks
     * 
     * @see Copter::ap.initialised flag
     * @see Copter::one_second_loop() for initialization sequence
     */
    bool vehicle_initialised() const override;

    /**
     * @brief Check if SIMPLE mode input transformation is active
     * 
     * @details Returns true if SIMPLE mode is currently active and transforming pilot
     *          stick inputs from earth frame to body frame. SIMPLE mode makes the copter
     *          easier to fly by maintaining a consistent control reference direction:
     *          
     *          SIMPLE Mode Behavior:
     *          - Forward stick always moves copter away from pilot (based on initial heading)
     *          - Back stick always returns copter toward pilot
     *          - Left/right sticks move copter left/right relative to pilot
     *          - Yaw stick still rotates copter normally
     *          
     *          Technical Implementation:
     *          - Records copter heading at takeoff or mode entry as reference
     *          - Transforms pilot inputs from reference frame to current body frame
     *          - Applied in rate controller before attitude control
     *          
     *          SIMPLE mode can be enabled via:
     *          - Flight mode switch (SIMPLE flight mode)
     *          - SIMPLE mode bitmask parameter (enables SIMPLE in specific modes)
     *          - Auxiliary function switch
     *          
     *          This status is reported to ground stations for pilot awareness.
     * 
     * @return true if SIMPLE mode transformation is active, false otherwise
     * 
     * @note SIMPLE mode uses initial heading, SUPERSIMPLE uses home direction
     * @see Copter::simple_cos_yaw and Copter::simple_sin_yaw for transformation
     * @see SIMPLE_MODE parameter bitmask
     */
    bool simple_input_active() const override;

    /**
     * @brief Check if SUPERSIMPLE mode input transformation is active
     * 
     * @details Returns true if SUPERSIMPLE mode is currently active and transforming
     *          pilot stick inputs to always be relative to the home position direction.
     *          SUPERSIMPLE mode is an enhanced version of SIMPLE mode that continuously
     *          updates the reference direction:
     *          
     *          SUPERSIMPLE Mode Behavior:
     *          - Forward stick always moves copter away from home position
     *          - Back stick always moves copter toward home position
     *          - Left/right sticks move copter tangent to home radius
     *          - Reference direction continuously updates based on copter position
     *          
     *          Technical Implementation:
     *          - Calculates bearing from current position to home
     *          - Transforms pilot inputs from home-relative to body frame
     *          - Updates transformation every loop (400Hz typically)
     *          - More computationally expensive than SIMPLE mode
     *          
     *          SUPERSIMPLE mode advantages:
     *          - Intuitive control regardless of copter heading
     *          - Easier to fly back to launch point
     *          - Helpful for FPV flying or when copter orientation unclear
     *          
     *          SUPERSIMPLE mode limitations:
     *          - Requires valid GPS position and home position
     *          - Can be confusing when very close to home (direction ambiguous)
     *          - Not recommended for aerobatic flight
     *          
     *          This status is reported to ground stations for pilot awareness.
     * 
     * @return true if SUPERSIMPLE mode transformation is active, false otherwise
     * 
     * @note Requires GPS lock and valid home position to function
     * @warning SUPERSIMPLE behavior near home can be unpredictable
     * @see Copter::supersimple_cos_yaw and Copter::supersimple_sin_yaw
     * @see SUPERSIMPLE_MODE parameter bitmask
     */
    bool supersimple_input_active() const override;

protected:

    /**
     * @brief Get minimum scheduler loop time remaining before allowing message transmission
     * 
     * @details Returns the minimum amount of time (in microseconds) that must remain
     *          in the main scheduler loop before the GCS system is permitted to send
     *          MAVLink messages. This timing constraint is CRITICAL for maintaining
     *          flight control stability and safety.
     *          
     *          Flight Control Priority:
     *          - Main flight control loop runs at 400Hz (2500μs period) on most boards
     *          - Flight critical tasks must complete: rate controller, attitude controller,
     *            motor mixing, ESC output
     *          - GCS telemetry is important but not flight-critical
     *          - If loop overruns occur, flight control becomes unstable
     *          
     *          Timing Budget:
     *          - 250μs minimum remaining time ensures GCS messages don't cause overruns
     *          - Typical MAVLink message transmission: 50-200μs depending on message size
     *          - 250μs provides safety margin for message processing and transmission
     *          - If less than 250μs remains, message deferred to next loop iteration
     *          
     *          This value is smaller than some other vehicle types:
     *          - Copter: 250μs (most time-critical due to unstable dynamics)
     *          - Plane: 500-1000μs (more stable, longer loop time)
     *          - Rover: 1000μs (least time-critical)
     *          
     *          Impact of This Parameter:
     *          - Too large: Telemetry becomes slow/choppy, but flight is safe
     *          - Too small: Risk of loop overruns and flight instability
     *          - 250μs is tuned for reliable telemetry without compromising safety
     * 
     * @return Minimum loop time remaining in microseconds (250μs)
     * 
     * @note This is a safety-critical timing parameter
     * @warning Reducing this value can cause scheduler overruns and flight instability
     * @warning Do NOT modify without extensive testing in SITL and real flight
     * 
     * @see AP_Scheduler for loop timing and task scheduling
     * @see GCS::update() for message sending logic that checks this value
     */
    uint16_t min_loop_time_remaining_for_message_send_us() const override {
        return 250;
    }

    /**
     * @brief Factory method to create copter-specific MAVLink backend for a channel
     * 
     * @details Creates and returns a new GCS_MAVLINK_Copter instance to handle MAVLink
     *          communication on a specific UART or network interface. This method is
     *          called by the base GCS class during initialization to instantiate the
     *          vehicle-specific MAVLink handlers for each configured communication channel.
     *          
     *          Channel Types:
     *          - Serial/UART channels (telemetry radios, USB, companion computers)
     *          - Network channels (WiFi, Ethernet on supported boards)
     *          - Typically 4-6 channels available depending on board hardware
     *          
     *          Backend Lifecycle:
     *          1. GCS initialization calls this method for each configured channel
     *          2. new_gcs_mavlink_backend creates GCS_MAVLINK_Copter instance
     *          3. Backend initialized with UART driver for the physical interface
     *          4. Backend registers for periodic updates in GCS update loop
     *          5. Backend handles MAVLink messages for its channel throughout flight
     *          
     *          Memory Allocation:
     *          - Uses NEW_NOTHROW for safe allocation without exceptions
     *          - Allocation failure returns nullptr (checked by caller)
     *          - Allocated from heap during initialization (not flight-critical timing)
     *          
     *          Each GCS_MAVLINK_Copter backend handles:
     *          - Receiving and parsing MAVLink messages from ground station
     *          - Processing vehicle commands (MAV_CMD_*)
     *          - Streaming telemetry messages (ATTITUDE, GPS_RAW, etc.)
     *          - Managing message queues and priorities
     *          - Protocol version negotiation (MAVLink 1.0 vs 2.0)
     * 
     * @param[in] uart Reference to HAL UART driver for this communication channel
     * 
     * @return Pointer to newly created GCS_MAVLINK_Copter instance, or nullptr on failure
     * 
     * @note Called during GCS initialization, not in time-critical flight path
     * @see GCS_MAVLINK_Copter class for per-channel message handling
     * @see AP_HAL::UARTDriver for UART hardware abstraction
     * @see GCS::init() for channel initialization sequence
     */
    GCS_MAVLINK_Copter *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Copter(uart);
    }

};
