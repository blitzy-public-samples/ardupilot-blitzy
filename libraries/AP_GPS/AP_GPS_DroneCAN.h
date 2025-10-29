/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file AP_GPS_DroneCAN.h
 * @brief DroneCAN (UAVCAN) GPS driver implementation for ArduPilot
 * 
 * @details This file implements GPS receiver support over CAN bus using the DroneCAN
 *          (formerly UAVCAN) protocol. Supports multiple GNSS message types including:
 *          - uavcan.equipment.gnss.Fix2: Position, velocity, and accuracy data
 *          - uavcan.equipment.gnss.Auxiliary: DOP values and satellite information
 *          - ardupilot.gnss.Heading: Dual-antenna heading (moving baseline)
 *          - ardupilot.gnss.Status: GPS system health and error codes
 *          - ardupilot.gnss.MovingBaselineData: RTK correction data for base/rover
 * 
 *          Automatically discovers GPS nodes on the CAN bus and configures RTK
 *          base/rover roles via CAN parameter interface. Supports RTCM3 injection
 *          for RTK corrections and extraction for base station output.
 * 
 * @note Requires AP_GPS_DRONECAN_ENABLED and a functional DroneCAN interface
 * @note GPS type must be set to UAVCAN, UAVCAN_RTK_BASE, or UAVCAN_RTK_ROVER
 * 
 * Source: libraries/AP_GPS/AP_GPS_DroneCAN.h:1-166
 * Source: libraries/AP_GPS/AP_GPS_DroneCAN.cpp (implementation)
 */

//
//  DroneCAN GPS driver
//
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_DRONECAN_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include "GPS_Backend.h"
#include "RTCM3_Parser.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

/**
 * @class AP_GPS_DroneCAN
 * @brief GPS backend for DroneCAN (UAVCAN) GPS devices connected via CAN bus
 * 
 * @details Implements GPS receiver communication over CAN using the DroneCAN protocol.
 *          This backend supports multiple GNSS message types providing comprehensive
 *          positioning data:
 * 
 *          - **Fix2 Messages**: Primary position and velocity data in ECEF coordinates,
 *            automatically converted to WGS84 latitude/longitude and NED velocity frame
 *          - **Auxiliary Messages**: Dilution of Precision (HDOP/VDOP), satellite counts,
 *            and signal quality metrics
 *          - **Heading Messages**: Dual-antenna GPS heading for vehicles with multiple
 *            GPS receivers (moving baseline configuration)
 *          - **Status Messages**: System health, error codes, and operational status
 *          - **MovingBaseline Messages**: RTK correction data for base/rover operations
 * 
 *          **GPS Discovery and Configuration:**
 *          Automatically discovers GPS nodes on the CAN bus during initialization.
 *          Maintains a registry of detected modules to prevent multiple backends from
 *          claiming the same CAN node. Performs automatic configuration of RTK roles
 *          (base/rover) via DroneCAN parameter interface.
 * 
 *          **RTK Support:**
 *          - RTK Base Mode: Outputs RTCM3 correction data via get_RTCMV3()
 *          - RTK Rover Mode: Accepts RTCM3 corrections via inject_data()
 *          - Moving Baseline: Dual-GPS heading with automatic role configuration
 * 
 *          **Coordinate Systems:**
 *          - Input: ECEF (Earth-Centered Earth-Fixed) from GNSS Fix2 messages
 *          - Output: WGS84 lat/lon/alt (degrees/degrees/meters above ellipsoid)
 *          - Velocity: NED (North-East-Down) frame in m/s
 * 
 *          **Units:**
 *          - Position: degrees (latitude/longitude), meters (altitude)
 *          - Velocity: m/s in NED frame
 *          - Accuracy: meters (horizontal/vertical standard deviation)
 *          - Time: GPS time of week in milliseconds, system timestamps in microseconds
 * 
 * @note Requires AP_GPS_DRONECAN_ENABLED feature flag to be enabled
 * @note Thread-safe: Uses semaphores to protect interim_state from concurrent CAN callbacks
 * @warning GPS type parameter must be set to UAVCAN, UAVCAN_RTK_BASE, or UAVCAN_RTK_ROVER
 * @warning Configuration changes may require GPS module reboot via CAN parameter save
 * 
 * Source: libraries/AP_GPS/AP_GPS_DroneCAN.h:32-163
 * Source: libraries/AP_GPS/AP_GPS_DroneCAN.cpp (implementation details)
 */
class AP_GPS_DroneCAN : public AP_GPS_Backend {
public:
    /**
     * @brief Construct DroneCAN GPS backend for specific GPS instance and role
     * 
     * @param _gps Reference to AP_GPS frontend manager
     * @param _params Reference to GPS parameter storage for this instance
     * @param _state Reference to GPS_State structure for this instance
     * @param role GPS role: GPS_ROLE_NORMAL, GPS_ROLE_MB_BASE, or GPS_ROLE_MB_ROVER
     * 
     * @details Initializes the DroneCAN GPS backend with the specified role configuration.
     *          Allocates RTCM3 parser if operating in moving baseline mode (base or rover).
     *          Registers parameter callbacks for GPS node configuration. Sets up interim
     *          state storage for thread-safe data handling from CAN callbacks.
     * 
     * @note Does not perform GPS discovery - use probe() for automatic detection
     * @note RTCM3 parser only allocated when GPS_MOVING_BASELINE is enabled
     */
    AP_GPS_DroneCAN(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, AP_GPS::GPS_Role role);
    
    /**
     * @brief Destructor - releases resources and removes from detection registry
     * 
     * @details Frees RTCM3 parser if allocated, releases RTCM stream buffer,
     *          and removes this backend from the detected modules registry.
     */
    ~AP_GPS_DroneCAN();

    /**
     * @brief Read and process GPS data from DroneCAN messages
     * 
     * @return true if new data available and processed, false otherwise
     * 
     * @details Called periodically by GPS frontend at configured update rate.
     *          Performs GPS node configuration if not yet completed, then copies
     *          interim_state (populated by CAN callbacks) to main state under
     *          semaphore protection. Updates health status based on fix type and
     *          data freshness.
     * 
     * @note This method is called from the main thread, while CAN callbacks
     *       update interim_state from interrupt context
     */
    bool read() override;

    /**
     * @brief Check if GPS is providing valid position fixes
     * 
     * @return true if recent valid fixes received, false if stale or invalid
     * 
     * @details Health determined by:
     *          - Valid fix type (2D or 3D fix)
     *          - Recent message timestamps (not stale)
     *          - Successful message processing
     */
    bool is_healthy(void) const override;

    /**
     * @brief Check if GPS data is suitable for logging
     * 
     * @return true if fix quality sufficient for logging
     * 
     * @details Returns true if position data is accurate enough to be useful
     *          for flight logs and analysis. May be true even when not healthy
     *          enough for navigation.
     */
    bool logging_healthy(void) const override;

    /**
     * @brief Check if GPS node configuration completed
     * 
     * @return true if parameter configuration finished, false if still configuring
     * 
     * @details During initialization, the backend configures GPS node parameters
     *          via DroneCAN parameter protocol (setting GPS type, moving baseline
     *          CAN TX settings). Returns false until configuration sequence completes.
     *          Configuration may require GPS module reboot.
     * 
     * @note Frontend will not use GPS data until this returns true
     */
    bool is_configured(void) const override;

    /**
     * @brief Get GPS backend name string
     * 
     * @return Backend name including detected CAN node ID
     * 
     * @details Returns descriptive name for logging and GCS display.
     *          Format: "DroneCAN" or "DroneCAN-<node_id>" once node detected.
     */
    const char *name() const override { return _name; }

    /**
     * @brief Subscribe to DroneCAN GPS message types on specified interface
     * 
     * @param ap_dronecan DroneCAN interface to subscribe on
     * @return true if subscription successful for all message types
     * 
     * @details Registers message handlers for:
     *          - uavcan.equipment.gnss.Fix2 (position/velocity)
     *          - uavcan.equipment.gnss.Auxiliary (DOP/satellites)
     *          - ardupilot.gnss.Heading (dual-antenna heading)
     *          - ardupilot.gnss.Status (system health)
     *          - ardupilot.gnss.MovingBaselineData (RTK corrections, if enabled)
     *          - ardupilot.gnss.RelPosHeading (relative positioning, if enabled)
     * 
     * @note Called during DroneCAN interface initialization
     * @note Trampoline functions route callbacks to appropriate backend instance
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    
    /**
     * @brief Factory method to detect and create DroneCAN GPS backend
     * 
     * @param _gps AP_GPS frontend reference
     * @param _state GPS state storage reference
     * @return Pointer to new AP_GPS_DroneCAN instance or nullptr if not detected
     * 
     * @details Searches all CAN buses for GPS nodes by checking the detected
     *          modules registry. Creates backend instance for first available
     *          GPS node that matches the configured GPS type. Uses registry
     *          to ensure each GPS node is claimed by only one backend instance.
     * 
     * @note Called by AP_GPS frontend during GPS initialization
     * @note Returns nullptr if no unclaimed GPS nodes found or wrong GPS type
     */
    static AP_GPS_Backend* probe(AP_GPS &_gps, AP_GPS::GPS_State &_state);

    /**
     * @brief Static trampoline for uavcan.equipment.gnss.Fix2 message callbacks
     * 
     * @param ap_dronecan DroneCAN interface that received message
     * @param transfer CAN transfer metadata (timestamp, source node ID, transfer ID)
     * @param msg Received Fix2 message containing position, velocity, and accuracy
     * 
     * @details Routes Fix2 messages to the appropriate backend instance based on
     *          source node ID. Fix2 provides primary GNSS data including ECEF position,
     *          ECEF velocity, position covariance, and fix type (no fix/2D/3D).
     * 
     * @note Called from CAN interrupt context - must be fast and thread-safe
     */
    static void handle_fix2_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Fix2& msg);

    /**
     * @brief Static trampoline for uavcan.equipment.gnss.Auxiliary message callbacks
     * 
     * @param ap_dronecan DroneCAN interface that received message
     * @param transfer CAN transfer metadata
     * @param msg Received Auxiliary message containing DOP values and satellite counts
     * 
     * @details Routes Auxiliary messages providing HDOP, VDOP, and number of satellites
     *          used in solution. Supplements Fix2 data with signal quality metrics.
     */
    static void handle_aux_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Auxiliary& msg);
    
    /**
     * @brief Static trampoline for ardupilot.gnss.Heading message callbacks
     * 
     * @param ap_dronecan DroneCAN interface that received message
     * @param transfer CAN transfer metadata
     * @param msg Received Heading message from dual-antenna GPS
     * 
     * @details Routes heading messages from moving baseline GPS providing accurate
     *          heading/yaw information from dual antennas. Used when GPS type is
     *          configured for moving baseline operation.
     */
    static void handle_heading_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_Heading& msg);
    
    /**
     * @brief Static trampoline for ardupilot.gnss.Status message callbacks
     * 
     * @param ap_dronecan DroneCAN interface that received message
     * @param transfer CAN transfer metadata
     * @param msg Received Status message containing system health and error codes
     * 
     * @details Routes GPS status messages providing error codes, health flags, and
     *          operational status. Error codes can be retrieved via get_error_codes().
     */
    static void handle_status_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_Status& msg);
#if GPS_MOVING_BASELINE
    /**
     * @brief Static trampoline for ardupilot.gnss.MovingBaselineData message callbacks
     * 
     * @param ap_dronecan DroneCAN interface that received message
     * @param transfer CAN transfer metadata
     * @param msg Received MovingBaselineData containing RTCM3 correction fragments
     * 
     * @details Routes RTK correction data for moving baseline configurations.
     *          Base station sends RTCM3 data fragmented over multiple CAN messages.
     *          RTCM3 parser reassembles fragments for rover consumption or external output.
     * 
     * @note Only available when GPS_MOVING_BASELINE feature is enabled
     */
    static void handle_moving_baseline_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData& msg);
    
    /**
     * @brief Static trampoline for ardupilot.gnss.RelPosHeading message callbacks
     * 
     * @param ap_dronecan DroneCAN interface that received message
     * @param transfer CAN transfer metadata
     * @param msg Received RelPosHeading message with relative positioning data
     * 
     * @details Routes relative positioning messages providing baseline vectors and
     *          heading information in moving baseline RTK configurations.
     * 
     * @note Only available when GPS_MOVING_BASELINE feature is enabled
     */
    static void handle_relposheading_msg_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_gnss_RelPosHeading& msg);
#endif
    
    /**
     * @brief Validate configuration consistency across multiple DroneCAN GPS instances
     * 
     * @param failure_msg Buffer to store failure description if checks fail
     * @param failure_msg_len Size of failure message buffer in bytes
     * @return true if all checks pass, false with failure message if misconfigured
     * 
     * @details Performs pre-arm checks for moving baseline configurations:
     *          - Ensures exactly one base and one rover when moving baseline configured
     *          - Validates base and rover are using same CAN interface
     *          - Checks for conflicting GPS type settings across instances
     *          - Verifies required messages (RelPosHeading) are being received
     * 
     * @note Called during pre-arm check sequence
     * @warning Moving baseline requires specific GPS type configuration on both instances
     */
    static bool inter_instance_pre_arm_checks(char failure_msg[], uint16_t failure_msg_len);
    
    /**
     * @brief Inject RTCM3 correction data to GPS via CAN
     * 
     * @param data Pointer to RTCM3 correction data bytes
     * @param len Length of RTCM data in bytes
     * 
     * @details Buffers RTCM3 data for transmission to GPS node over CAN bus.
     *          Data is fragmented into multiple CAN messages if needed. Used for:
     *          - RTK corrections from base station to rover
     *          - NTRIP corrections from ground station to rover
     *          - External correction sources
     * 
     *          Injection rate limited to prevent CAN bus saturation.
     * 
     * @note Data buffered in _rtcm_stream and sent asynchronously by send_rtcm()
     * @note GPS node must support ardupilot.gnss.RTCMStream message type
     */
    void inject_data(const uint8_t *data, uint16_t len) override;

    /**
     * @brief Retrieve GPS error/status codes from ardupilot.gnss.Status message
     * 
     * @param error_codes Output parameter filled with GPS-specific error codes
     * @return true if status message received and error codes available, false otherwise
     * 
     * @details Error codes are GPS-specific bit flags indicating various error
     *          conditions (antenna issues, jamming, spoofing, etc.). Format depends
     *          on GPS receiver type. Only valid if Status messages have been received.
     */
    bool get_error_codes(uint32_t &error_codes) const override { error_codes = error_code; return seen_status; };

#if GPS_MOVING_BASELINE
    /**
     * @brief Retrieve RTCM3 correction data buffer for moving baseline RTK output
     * 
     * @param data Output pointer set to RTCM data buffer
     * @param len Output parameter filled with available RTCM data length in bytes
     * @return true if RTCM data available, false if buffer empty
     * 
     * @details Used by RTK base stations to extract RTCM3 correction data for
     *          transmission to rovers. Data parsed from ardupilot.gnss.MovingBaselineData
     *          messages and buffered by RTCM3 parser. Rover consumes this data via
     *          inject_data() or external telemetry link.
     * 
     * @note Only available in GPS_MOVING_BASELINE mode for base station role
     * @note Call clear_RTCMV3() after consuming data to free buffer space
     */
    bool get_RTCMV3(const uint8_t *&data, uint16_t &len) override;
    
    /**
     * @brief Clear consumed RTCM3 data from buffer
     * 
     * @details Releases buffered RTCM3 data after it has been read via get_RTCMV3().
     *          Must be called to prevent buffer overflow in base station mode.
     * 
     * @note Only available in GPS_MOVING_BASELINE mode
     */
    void clear_RTCMV3() override;
#endif

#if AP_DRONECAN_SEND_GPS
    /**
     * @brief Check if any GPS backend instance exists on specified DroneCAN interface
     * 
     * @param ap_dronecan DroneCAN interface to check
     * @return true if at least one GPS backend using this interface, false otherwise
     * 
     * @details Used to determine if GPS data forwarding is needed on this CAN interface.
     *          Checks the detected modules registry for any GPS backends bound to the
     *          specified DroneCAN interface.
     * 
     * @note Only available when AP_DRONECAN_SEND_GPS feature is enabled
     */
    static bool instance_exists(const AP_DroneCAN* ap_dronecan);
#endif

private:

    // ========== Configuration State Management ==========
    
    /**
     * @brief Flag indicating if GPS node parameter configuration is complete
     * 
     * @details Starts true, set to false if configuration needed. Configuration
     *          sequence sets GPS type and moving baseline parameters via CAN
     *          parameter protocol. Returns to true when STEP_FINISHED reached.
     */
    bool param_configured = true;
    
    /**
     * @enum config_step
     * @brief Configuration sequence steps for GPS node setup
     * 
     * @details Defines the state machine for configuring GPS parameters via DroneCAN:
     *          - STEP_SET_TYPE: Configure GPS_TYPE parameter (base/rover/normal)
     *          - STEP_SET_MB_CAN_TX: Configure moving baseline CAN transmit settings
     *          - STEP_SAVE_AND_REBOOT: Trigger parameter save and GPS module reboot
     *          - STEP_FINISHED: Configuration complete, normal operation
     */
    enum config_step {
        STEP_SET_TYPE = 0,          ///< Set GPS_TYPE parameter
        STEP_SET_MB_CAN_TX,         ///< Set moving baseline CAN TX parameter
        STEP_SAVE_AND_REBOOT,       ///< Save parameters and reboot GPS
        STEP_FINISHED               ///< Configuration complete
    };
    uint8_t cfg_step;               ///< Current configuration step
    bool requires_save_and_reboot;  ///< Flag indicating GPS needs parameter save/reboot

    /**
     * @brief Execute GPS node configuration sequence
     * 
     * @return true once configuration has finished, false while still configuring
     * 
     * @details Steps through configuration state machine to set GPS parameters via
     *          DroneCAN parameter protocol. Configures GPS type (base/rover/normal)
     *          and moving baseline CAN settings. May trigger GPS module reboot for
     *          parameters to take effect. Called from read() until returns true.
     * 
     * @note Configuration is asynchronous - may take several calls to complete
     */
    bool do_config(void);

    // ========== Message Handlers (Instance Methods) ==========
    
    /**
     * @brief Process uavcan.equipment.gnss.Fix2 message with position and velocity
     * 
     * @param msg Fix2 message containing ECEF position, velocity, covariance, and fix type
     * @param timestamp_usec System timestamp when message received (microseconds)
     * 
     * @details Converts ECEF coordinates to WGS84 lat/lon/alt, extracts velocity in
     *          NED frame, updates position covariance, and sets fix type. Updates
     *          interim_state under semaphore protection. Marks seen_fix2 flag for
     *          feature detection.
     * 
     * @note Called from CAN interrupt context via trampoline - must be thread-safe
     * @note Position: ECEF (m) → WGS84 (degrees/meters above ellipsoid)
     * @note Velocity: ECEF (m/s) → NED frame (m/s)
     */
    void handle_fix2_msg(const uavcan_equipment_gnss_Fix2& msg, uint64_t timestamp_usec);
    
    /**
     * @brief Process uavcan.equipment.gnss.Auxiliary message with DOP and satellite data
     * 
     * @param msg Auxiliary message containing HDOP, VDOP, and satellite count
     * 
     * @details Updates interim_state with horizontal and vertical dilution of precision
     *          values and number of satellites used in solution. Provides signal quality
     *          metrics supplementing Fix2 position data.
     * 
     * @note Called from CAN interrupt context - updates under semaphore protection
     */
    void handle_aux_msg(const uavcan_equipment_gnss_Auxiliary& msg);
    
    /**
     * @brief Process ardupilot.gnss.Heading message from dual-antenna GPS
     * 
     * @param msg Heading message containing accurate yaw from moving baseline GPS
     * 
     * @details Extracts heading and accuracy from dual-antenna GPS configuration.
     *          Updates interim_state yaw fields. Used for precise heading when GPS
     *          type configured for moving baseline with dual antennas.
     */
    void handle_heading_msg(const ardupilot_gnss_Heading& msg);
    
    /**
     * @brief Process ardupilot.gnss.Status message with system health and errors
     * 
     * @param msg Status message containing error codes and health flags
     * 
     * @details Stores GPS-specific error codes and status flags for retrieval via
     *          get_error_codes(). Marks seen_status flag. Error codes indicate antenna
     *          issues, jamming, spoofing, or other GPS-specific problems.
     */
    void handle_status_msg(const ardupilot_gnss_Status& msg);
    
    /**
     * @brief Convert and store velocity components in NED frame
     * 
     * @param vx Velocity in X direction (forward) in m/s
     * @param vy Velocity in Y direction (right) in m/s
     * @param vz Velocity in Z direction (down) in m/s
     * 
     * @details Helper function to convert velocity from body or ECEF frame to NED
     *          (North-East-Down) frame and store in interim_state. Called by Fix2
     *          message handler.
     * 
     * @note Velocity units: m/s in NED frame
     */
    void handle_velocity(const float vx, const float vy, const float vz);

#if GPS_MOVING_BASELINE
    /**
     * @brief Process ardupilot.gnss.MovingBaselineData with RTCM3 correction fragments
     * 
     * @param msg MovingBaselineData containing RTCM3 data fragment
     * @param node_id CAN node ID of sender (base station)
     * 
     * @details Receives fragmented RTCM3 correction data from base station via CAN.
     *          RTCM3 parser reassembles fragments into complete RTCM messages for:
     *          - Rover: Consumption via inject_data() for RTK positioning
     *          - Base: Output via get_RTCMV3() for transmission to external rovers
     * 
     * @note Only available when GPS_MOVING_BASELINE feature is enabled
     */
    void handle_moving_baseline_msg(const ardupilot_gnss_MovingBaselineData& msg, uint8_t node_id);
    
    /**
     * @brief Process ardupilot.gnss.RelPosHeading with relative positioning data
     * 
     * @param msg RelPosHeading message with baseline vector and heading
     * @param node_id CAN node ID of sender
     * 
     * @details Processes relative positioning information in moving baseline RTK
     *          configurations. Provides baseline vector between antennas and computed
     *          heading. Marks seen_relposheading flag for pre-arm checks.
     * 
     * @note Only available when GPS_MOVING_BASELINE feature is enabled
     */
    void handle_relposheading_msg(const ardupilot_gnss_RelPosHeading& msg, uint8_t node_id);
#endif

    // ========== Registry Management (Thread-Safe) ==========
    
    /**
     * @brief Acquire lock on detected modules registry
     * 
     * @return true if lock acquired
     * 
     * @details Thread-safe registry access for GPS node detection and assignment.
     *          Must call give_registry() after registry access complete.
     */
    static bool take_registry();
    
    /**
     * @brief Release lock on detected modules registry
     * 
     * @details Releases registry semaphore acquired by take_registry().
     */
    static void give_registry();
    
    /**
     * @brief Find DroneCAN GPS backend instance for specific CAN node
     * 
     * @param ap_dronecan DroneCAN interface to search
     * @param node_id CAN node ID to find
     * @return Pointer to backend instance if found, nullptr otherwise
     * 
     * @details Searches detected modules registry for backend instance handling
     *          the specified CAN node. Used by trampolines to route messages to
     *          correct backend instance.
     * 
     * @note Must be called with registry lock held (take_registry())
     */
    static AP_GPS_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    // ========== Data State Management ==========
    
    bool _new_data;  ///< Flag indicating new GPS data available since last read()
    
    /**
     * @brief Temporary GPS state storage updated by CAN callbacks
     * 
     * @details Interim storage for GPS data arriving via CAN interrupt callbacks.
     *          Protected by 'sem' semaphore. CAN handlers update this structure,
     *          then read() copies to main state under semaphore protection. Prevents
     *          race conditions between CAN interrupt context and main thread.
     */
    AP_GPS::GPS_State interim_state;

    /**
     * @brief Semaphore protecting interim_state from concurrent access
     * 
     * @details Ensures thread-safe access to interim_state between CAN callbacks
     *          (interrupt context) and read() method (main thread context). Must be
     *          held when reading or writing interim_state.
     */
    HAL_Semaphore sem;

    // ========== GPS Node Detection and Tracking ==========
    
    uint8_t _detected_module;    ///< CAN node ID (0-127) of detected GPS module
    bool seen_message;           ///< True if any message received from GPS node
    bool seen_fix2;              ///< True if Fix2 messages received (position/velocity)
    bool seen_aux;               ///< True if Auxiliary messages received (DOP/satellites)
    bool seen_status;            ///< True if Status messages received (health/errors)
    bool seen_relposheading;     ///< True if RelPosHeading messages received (moving baseline)
    bool seen_valid_height_ellipsoid;  ///< True if valid ellipsoidal height data received

    // ========== Health and Status ==========
    
    bool healthy;                ///< Overall GPS health status (valid fix, recent data)
    uint32_t status_flags;       ///< GPS status flags from ardupilot.gnss.Status message
    uint32_t error_code;         ///< GPS-specific error codes from ardupilot.gnss.Status
    char _name[16];              ///< Backend name string for logging/display

    // ========== Module Detection Registry (Static) ==========
    
    /**
     * @struct DetectedModules
     * @brief Registry entry for detected GPS modules on CAN bus
     * 
     * @details Maintains mapping between CAN nodes and GPS backend instances.
     *          Prevents multiple backends from claiming the same GPS node.
     *          Used for message routing and RTCM injection coordination.
     */
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;   ///< DroneCAN interface this GPS is on
        uint8_t node_id;            ///< CAN node ID (0-127) of GPS module
        uint8_t instance;           ///< GPS instance number (0 to GPS_MAX_RECEIVERS-1)
        uint32_t last_inject_ms;    ///< Timestamp of last RTCM injection (milliseconds)
        AP_GPS_DroneCAN* driver;    ///< Pointer to backend instance managing this GPS
    } _detected_modules[GPS_MAX_RECEIVERS];

    /**
     * @brief Semaphore protecting _detected_modules registry
     * 
     * @details Ensures thread-safe access to GPS module detection registry.
     *          Acquired via take_registry(), released via give_registry().
     */
    static HAL_Semaphore _sem_registry;

#if GPS_MOVING_BASELINE
    /**
     * @brief RTCM3 parser for extracting correction messages in moving baseline mode
     * 
     * @details Parses RTCM3 data fragments from ardupilot.gnss.MovingBaselineData
     *          messages, reassembling complete RTCM messages for:
     *          - Base mode: Output via get_RTCMV3() for rover consumption
     *          - Rover mode: Processing RTK corrections for precise positioning
     * 
     * @note Only allocated when GPS_MOVING_BASELINE feature enabled and role is base/rover
     */
    RTCM3_Parser *rtcm3_parser;
    
    uint32_t last_base_warning_ms;  ///< Timestamp of last base station warning message
#endif
    
    /**
     * @brief GPS role configuration set from GPS_TYPE parameter
     * 
     * @details Role determines GPS behavior:
     *          - GPS_ROLE_NORMAL: Standard GPS operation
     *          - GPS_ROLE_MB_BASE: Moving baseline RTK base station
     *          - GPS_ROLE_MB_ROVER: Moving baseline RTK rover
     */
    AP_GPS::GPS_Role role;

    // ========== Parameter Configuration Callbacks ==========
    
    /**
     * @brief Functor callback for integer parameter get/set responses
     * 
     * @details Handles responses to CAN parameter read/write operations for integer
     *          parameters (GPS_TYPE, MB_CAN_TX). Used during configuration sequence.
     */
    FUNCTOR_DECLARE(param_int_cb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    
    /**
     * @brief Functor callback for float parameter get/set responses
     * 
     * @details Handles responses to CAN parameter read/write operations for floating
     *          point parameters. Reserved for future parameter types.
     */
    FUNCTOR_DECLARE(param_float_cb, bool, AP_DroneCAN*, const uint8_t, const char*, float &);
    
    /**
     * @brief Functor callback for parameter save/reboot responses
     * 
     * @details Handles response to parameter save request. GPS module may reboot
     *          after saving parameters for changes to take effect.
     */
    FUNCTOR_DECLARE(param_save_cb, void, AP_DroneCAN*, const uint8_t, bool);

    /**
     * @brief Handle CAN parameter get/set response for integer parameters
     * 
     * @param ap_dronecan DroneCAN interface
     * @param node_id CAN node ID of GPS
     * @param name Parameter name string
     * @param value Parameter value (input for set, output for get)
     * @return true if parameter operation successful
     */
    bool handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, int32_t &value);
    
    /**
     * @brief Handle CAN parameter get/set response for float parameters
     * 
     * @param ap_dronecan DroneCAN interface
     * @param node_id CAN node ID of GPS
     * @param name Parameter name string
     * @param value Parameter value (input for set, output for get)
     * @return true if parameter operation successful
     */
    bool handle_param_get_set_response_float(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, float &value);
    
    /**
     * @brief Handle CAN parameter save response
     * 
     * @param ap_dronecan DroneCAN interface
     * @param node_id CAN node ID of GPS
     * @param success True if parameters saved successfully
     * 
     * @details Called when GPS responds to parameter save request. GPS may reboot
     *          after successful save for new parameters to take effect.
     */
    void handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success);
    
    /**
     * @brief Send buffered RTCM correction data to GPS via CAN
     * 
     * @details Transmits RTCM3 data from _rtcm_stream buffer to GPS node over CAN bus.
     *          Fragments large RTCM messages into multiple CAN frames if needed.
     *          Rate-limited to prevent CAN bus saturation. Called periodically from
     *          read() when injection data available.
     * 
     * @note Uses ardupilot.gnss.RTCMStream message type
     */
    void send_rtcm(void);

    // ========== RTCM Injection Stream ==========
    
    /**
     * @brief RTCM3 injection data buffer and state
     * 
     * @details Buffers RTCM correction data received via inject_data() for
     *          transmission to GPS node over CAN. Fragmented and sent by send_rtcm().
     *          Rate-limited via last_send_ms to prevent overwhelming CAN bus.
     */
    struct {
        uint32_t last_send_ms;  ///< Timestamp of last RTCM transmission (milliseconds)
        ByteBuffer *buf;        ///< Buffer holding RTCM data awaiting transmission
    } _rtcm_stream;

    /**
     * @brief Check if GPS type is a DroneCAN GPS variant
     * 
     * @param type GPS_Type value to check
     * @return true if type is UAVCAN, UAVCAN_RTK_BASE, or UAVCAN_RTK_ROVER
     * 
     * @details Helper function to determine if a GPS_Type parameter value indicates
     *          DroneCAN (UAVCAN) GPS operation. Returns true for all DroneCAN variants:
     *          - GPS_TYPE_UAVCAN: Standard DroneCAN GPS
     *          - GPS_TYPE_UAVCAN_RTK_BASE: DroneCAN RTK base station
     *          - GPS_TYPE_UAVCAN_RTK_ROVER: DroneCAN RTK rover
     * 
     * @note Used during probe() to match GPS instances with backend type
     */
    static bool is_dronecan_gps_type(AP_GPS::GPS_Type type) {
        return (
            type == AP_GPS::GPS_TYPE_UAVCAN ||
            type == AP_GPS::GPS_TYPE_UAVCAN_RTK_BASE ||
            type == AP_GPS::GPS_TYPE_UAVCAN_RTK_ROVER
       );
    }
};

#endif  // AP_GPS_DRONECAN_ENABLED
