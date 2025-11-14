/**
 * @file AP_BattMonitor_DroneCAN.h
 * @brief DroneCAN/UAVCAN battery monitoring backend
 * 
 * This file implements battery monitoring via CAN bus using the DroneCAN/UAVCAN protocol.
 * It subscribes to UAVCAN BatteryInfo and BatteryInfoAux messages to receive battery telemetry
 * from smart batteries and MPPT solar charge controllers over the CAN bus.
 * 
 * Supported Features:
 * - Voltage, current, and temperature monitoring
 * - State of charge (SoC) and state of health (SoH) reporting
 * - Individual cell voltages
 * - Remaining capacity and time remaining estimates
 * - Battery cycle count tracking
 * - MPPT solar charge controller integration with power control
 * - MAVLink fault status reporting
 * 
 * The backend receives updates at approximately 10Hz from CAN messages and provides
 * thread-safe access to battery state through semaphore protection.
 */

#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_DroneCAN/AP_DroneCAN.h>

/**
 * @brief Timeout for CAN message reception in microseconds (5 seconds)
 * @note Battery becomes unhealthy if no successful readings received within this period
 */
#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000

#ifndef AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
#define AP_BATTMONITOR_UAVCAN_MPPT_DEBUG 0
#endif

/**
 * @class AP_BattMonitor_DroneCAN
 * @brief DroneCAN/UAVCAN battery monitoring backend implementation
 * 
 * @details This backend subscribes to UAVCAN BatteryInfo and BatteryInfoAux messages
 *          over the DroneCAN bus to receive battery telemetry from smart batteries and
 *          MPPT solar charge controllers. It provides comprehensive battery monitoring
 *          including voltage, current, temperature, state of charge, cell voltages,
 *          and remaining capacity estimates.
 * 
 *          The backend maintains thread-safe access to battery state using semaphores
 *          and updates at approximately 10Hz based on CAN message reception rate.
 *          Messages older than 5 seconds cause the battery to be marked unhealthy.
 * 
 *          Special support is included for Packet Digital MPPT solar charge controllers,
 *          allowing remote power control via CAN RPC commands.
 * 
 * @note Thread-safe: All CAN callbacks use semaphore protection (_sem_battmon)
 * @warning State of charge can come from CAN messages or be calculated locally
 *          depending on message availability and configuration
 */
class AP_BattMonitor_DroneCAN : public AP_BattMonitor_Backend
{
public:
    /**
     * @enum BattMonitor_DroneCAN_Type
     * @brief Message type selection for DroneCAN battery monitoring
     * 
     * Currently only UAVCAN BatteryInfo messages are supported.
     * This enum allows for future expansion to support additional message types.
     */
    enum BattMonitor_DroneCAN_Type {
        UAVCAN_BATTERY_INFO = 0  ///< Use UAVCAN equipment.power.BatteryInfo messages
    };

    /**
     * @brief Constructor for DroneCAN battery monitor backend
     * 
     * @param[in] mon Reference to parent AP_BattMonitor object
     * @param[in] mon_state Reference to battery state structure to update
     * @param[in] type Message type to use (currently only UAVCAN_BATTERY_INFO supported)
     * @param[in] params Reference to battery monitor parameters
     * 
     * @details Initializes the backend and sets up DroneCAN message subscriptions.
     *          The backend will start receiving BatteryInfo and BatteryInfoAux messages
     *          from the specified CAN node once subscriptions are active.
     */
    AP_BattMonitor_DroneCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_DroneCAN_Type type, AP_BattMonitor_Params &params);

    /**
     * @brief Parameter table for DroneCAN battery monitor configuration
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Initialize the battery monitor
     * 
     * @details DroneCAN battery monitors require no initialization beyond construction,
     *          as CAN subscriptions are set up in the constructor. This method is
     *          provided to satisfy the backend interface.
     */
    void init() override {}

    /**
     * @brief Read the battery voltage and current from CAN messages
     * 
     * @details This method should be called at 10Hz to update the battery state.
     *          It retrieves the latest battery information received via CAN messages
     *          and updates the monitor state. The actual data arrives asynchronously
     *          via CAN callbacks at the message publication rate (typically 10Hz).
     * 
     * @note Battery is marked unhealthy if no messages received for 5 seconds
     *       (AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS)
     */
    void read() override;

    /**
     * @brief Get battery state of charge percentage
     * 
     * @param[out] percentage State of charge in percent (0-100)
     * @return true if SoC is available from CAN messages, false otherwise
     * 
     * @details Returns the state of charge received in BatteryInfo messages.
     *          If SoC is not available from CAN, the backend will calculate
     *          it locally by integrating current draw.
     */
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    /**
     * @brief Check if temperature measurement is available
     * @return true if battery reports temperature in CAN messages
     */
    bool has_temperature() const override { return _has_temperature; }

    /**
     * @brief Check if current measurement is available
     * @return true always - DroneCAN batteries always provide current
     */
    bool has_current() const override { return true; }

    /**
     * @brief Check if consumed energy measurement is available
     * @return true always - either from BatteryInfoAux message or by cumulative current integration
     * 
     * @details Consumed energy (mAh) is available either directly from the BatteryInfoAux
     *          message or by integrating current draw over time locally.
     */
    bool has_consumed_energy() const override { return true; }

    /**
     * @brief Check if time remaining estimate is available
     * @return true if battery provides time remaining in CAN messages
     */
    bool has_time_remaining() const override { return _has_time_remaining; }

    /**
     * @brief Check if individual cell voltages are available
     * @return true if battery reports individual cell voltages in CAN messages
     */
    bool has_cell_voltages() const override { return _has_cell_voltages; }

    /**
     * @brief Get battery cycle count
     * 
     * @param[out] cycles Number of charge/discharge cycles the battery has undergone
     * @return true if cycle count is available from BatteryInfoAux message
     * 
     * @details Cycle count is provided by smart batteries in the BatteryInfoAux message
     *          and indicates battery age and wear level.
     */
    bool get_cycle_count(uint16_t &cycles) const override;

    /**
     * @brief Get MAVLink battery fault bitmask
     * 
     * @return Bitmask of battery fault conditions (see MAV_BATTERY_FAULT enum)
     * 
     * @details Translates DroneCAN battery fault flags to MAVLink fault bitmask format
     *          for ground control station reporting. Includes faults like overvoltage,
     *          undervoltage, overcurrent, and overtemperature.
     */
    uint32_t get_mavlink_fault_bitmask() const override;

    /**
     * @brief Subscribe to DroneCAN battery messages
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface
     * @return true if subscriptions successful
     * 
     * @details Sets up subscriptions for BatteryInfo, BatteryInfoAux, and MPPT stream messages.
     */
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);
    
    /**
     * @brief Get backend instance for specific CAN node and battery ID
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface
     * @param[in] node_id CAN node ID of the battery device
     * @param[in] battery_id Battery identifier (for multi-battery devices)
     * @return Pointer to matching backend instance, or nullptr if not found
     */
    static AP_BattMonitor_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, uint8_t battery_id);
    
    /**
     * @brief Trampoline for BatteryInfo message callbacks
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface
     * @param[in] transfer CAN transfer information
     * @param[in] msg Received BatteryInfo message
     * 
     * @details Static callback that routes CAN messages to the appropriate backend instance.
     *          Called by DroneCAN driver when BatteryInfo messages are received.
     */
    static void handle_battery_info_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_power_BatteryInfo &msg);
    
    /**
     * @brief Trampoline for BatteryInfoAux message callbacks
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface
     * @param[in] transfer CAN transfer information
     * @param[in] msg Received BatteryInfoAux message
     * 
     * @details Static callback that routes CAN messages to the appropriate backend instance.
     *          BatteryInfoAux provides extended battery info like cycle count and nominal voltage.
     */
    static void handle_battery_info_aux_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_power_BatteryInfoAux &msg);
    
    /**
     * @brief Trampoline for MPPT stream message callbacks
     * 
     * @param[in] ap_dronecan Pointer to DroneCAN interface
     * @param[in] transfer CAN transfer information
     * @param[in] msg Received MPPT stream message
     * 
     * @details Static callback for MPPT solar charge controller telemetry messages.
     *          Routes messages to the appropriate MPPT battery backend instance.
     */
    static void handle_mppt_stream_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const mppt_Stream &msg);

    /**
     * @brief Control MPPT solar charge controller power state
     * 
     * @param[in] power_on true to power on MPPT, false to power off
     * 
     * @details Sends CAN RPC command to Packet Digital MPPT solar charge controllers
     *          to enable or disable power output. Used to control MPPT based on
     *          vehicle armed state.
     */
    void mppt_set_powered_state(bool power_on) override;

    /**
     * @brief Reset remaining battery capacity to specified percentage
     * 
     * @param[in] percentage New remaining capacity percentage (0-100)
     * @return true if reset successful
     * 
     * @details Allows manual reset of battery state of charge. Useful for
     *          synchronizing battery state after battery replacement or charging.
     */
    bool reset_remaining(float percentage) override;

private:
    /**
     * @brief Handle received BatteryInfo message
     * 
     * @param[in] msg Received BatteryInfo message from CAN bus
     * 
     * @details Processes BatteryInfo message containing voltage, current, temperature,
     *          and state of charge. Updates interim battery state with semaphore protection.
     */
    void handle_battery_info(const uavcan_equipment_power_BatteryInfo &msg);
    
    /**
     * @brief Handle received BatteryInfoAux message
     * 
     * @param[in] msg Received BatteryInfoAux message from CAN bus
     * 
     * @details Processes BatteryInfoAux message containing extended battery information
     *          including cycle count, nominal voltage, and remaining/full charge capacity.
     */
    void handle_battery_info_aux(const ardupilot_equipment_power_BatteryInfoAux &msg);
    
    /**
     * @brief Update interim battery state from CAN message data
     * 
     * @param[in] voltage Battery terminal voltage in volts
     * @param[in] current Battery current in amperes (positive = discharging)
     * @param[in] temperature_K Battery temperature in Kelvin
     * @param[in] soc State of charge percentage (0-100)
     * @param[in] soh_pct State of health percentage (0-100)
     * 
     * @details Updates the interim state structure with new measurements from CAN messages.
     *          The interim state is later copied to the main state in read() method.
     */
    void update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc, uint8_t soh_pct);

    /**
     * @brief Check if battery ID matches backend instance
     * 
     * @param[in] instance Battery monitor instance number
     * @param[in] battery_id Battery ID from CAN message
     * @return true if IDs match
     * 
     * @details Used to route CAN messages to the correct backend instance when
     *          multiple batteries are present on the CAN bus.
     */
    static bool match_battery_id(uint8_t instance, uint8_t battery_id);

    /**
     * @enum MPPT_FaultFlags
     * @brief Fault condition flags for MPPT solar charge controllers
     * 
     * Bitmask values indicating various fault conditions detected by MPPT devices.
     */
    enum class MPPT_FaultFlags : uint8_t {
        OVER_VOLTAGE        = (1U<<0),  ///< Input or output voltage exceeded limits
        UNDER_VOLTAGE       = (1U<<1),  ///< Input or output voltage below minimum
        OVER_CURRENT        = (1U<<2),  ///< Current exceeded rated limit
        OVER_TEMPERATURE    = (1U<<3),  ///< Temperature exceeded safe operating limit
    };
    
    /**
     * @brief Handle received MPPT stream message
     * 
     * @param[in] msg Received MPPT stream message from CAN bus
     * 
     * @details Processes telemetry from Packet Digital MPPT solar charge controllers,
     *          updating battery state and monitoring fault conditions.
     */
    void handle_mppt_stream(const mppt_Stream &msg);
    
    /**
     * @brief Check and update MPPT powered state based on vehicle state
     * 
     * @details Monitors vehicle armed state and sends power on/off commands to MPPT
     *          devices as needed. MPPT is typically powered on when vehicle is armed.
     */
    void mppt_check_powered_state();

#if AP_BATTMONITOR_UAVCAN_MPPT_DEBUG
    /**
     * @brief Report MPPT fault conditions to debug output
     * 
     * @param[in] instance Battery monitor instance number
     * @param[in] fault_flags Bitmask of MPPT_FaultFlags
     * 
     * @details Debug-only method that logs MPPT fault conditions for troubleshooting.
     */
    static void mppt_report_faults(const uint8_t instance, const uint8_t fault_flags);
    
    /**
     * @brief Get human-readable string for MPPT fault flag
     * 
     * @param[in] fault MPPT fault flag to convert
     * @return String description of fault condition
     * 
     * @details Debug-only method for converting fault flags to readable strings.
     */
    static const char* mppt_fault_string(const MPPT_FaultFlags fault);
#endif

    /**
     * @brief Determine if DroneCAN state of charge should be used
     * 
     * @return true if CAN-provided SoC should be used, false to calculate locally
     * 
     * @details Returns true if the battery provides valid state of charge via CAN messages.
     *          Returns false if state of charge should be calculated locally by integrating
     *          current draw (counting mAh consumed).
     */
    bool use_CAN_SoC() const;

    AP_BattMonitor::BattMonitor_State _interim_state;  ///< Interim battery state updated by CAN callbacks

    HAL_Semaphore _sem_battmon;  ///< Semaphore for thread-safe access between CAN callbacks and read() method

    AP_DroneCAN* _ap_dronecan;  ///< Pointer to DroneCAN interface for message subscriptions
    uint8_t _soc;  ///< State of charge in percent (0-100)
    uint8_t _node_id;  ///< CAN node ID of the battery device
    uint16_t _cycle_count;  ///< Number of charge/discharge cycles
    float _remaining_capacity_wh;  ///< Remaining battery capacity in watt-hours
    float _full_charge_capacity_wh;  ///< Full charge capacity in watt-hours
    bool _has_temperature;  ///< true if battery reports temperature
    bool _has_cell_voltages;  ///< true if battery reports individual cell voltages
    bool _has_time_remaining;  ///< true if battery reports estimated time remaining
    bool _has_battery_info_aux;  ///< true if BatteryInfoAux message received
    uint8_t _instance;  ///< Instance number of this battery monitor

    /**
     * @brief Current measurement scaling multiplier
     * @warning Adjust this calibration parameter if current readings are inaccurate
     * @details Scaling multiplier applied to current reports for calibration adjustment.
     *          Default is 1.0. Increase if current reads low, decrease if current reads high.
     */
    AP_Float _curr_mult;
    /**
     * @brief MPPT solar charge controller state variables
     * 
     * @details State tracking for Packet Digital MPPT solar charge controllers.
     *          Monitors power state, fault conditions, and manages remote power control.
     */
    struct {
        bool is_detected;               ///< true if this DroneCAN device is a Packet Digital MPPT
        bool powered_state;             ///< true if the MPPT is powered on, false if powered off
        bool vehicle_armed_last;        ///< Latest vehicle armed state, used to detect changes and control MPPT power
        uint8_t fault_flags;            ///< Bitmask of MPPT_FaultFlags indicating fault conditions
        uint32_t powered_state_remote_ms; ///< Timestamp in milliseconds when power state request was sent, zeroed on response (used for retry logic)
    } _mppt;

    /**
     * @brief Handle MPPT output enable RPC response
     * 
     * @param[in] transfer CAN transfer information
     * @param[in] response MPPT output enable response message
     * 
     * @details Processes response to power control commands sent to MPPT devices.
     */
    void handle_outputEnable_response(const CanardRxTransfer&, const mppt_OutputEnableResponse&);

    /**
     * @brief Callback object for MPPT output enable responses
     */
    Canard::ObjCallback<AP_BattMonitor_DroneCAN, mppt_OutputEnableResponse> mppt_outputenable_res_cb{this, &AP_BattMonitor_DroneCAN::handle_outputEnable_response};
    
    /**
     * @brief CAN client for MPPT output enable RPC calls
     */
    Canard::Client<mppt_OutputEnableResponse> *mppt_outputenable_client;
};
#endif
