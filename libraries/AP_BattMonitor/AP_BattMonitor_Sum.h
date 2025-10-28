/**
 * @file AP_BattMonitor_Sum.h
 * @brief Battery monitor backend for aggregating/summing multiple battery instances
 * 
 * This backend provides the ability to aggregate voltage, current, and capacity
 * measurements from multiple battery monitor instances into a single virtual battery.
 * This is useful for parallel battery configurations where the system needs to track
 * total capacity and current draw across multiple physical batteries.
 */

#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_SUM_ENABLED

#include "AP_BattMonitor.h"

/**
 * @class AP_BattMonitor_Sum
 * @brief Battery monitor backend that aggregates measurements from multiple battery instances
 * 
 * @details This backend aggregates voltage, current, and capacity from multiple battery
 *          monitor instances specified by the _sum_mask bitmask parameter. It is designed
 *          for multi-battery parallel configurations where:
 *          - Voltage is reported as the MINIMUM across all batteries (to prevent over-discharge)
 *          - Current is SUMMED across all batteries (total draw)
 *          - Consumed mAh is SUMMED across all batteries (total consumption)
 *          - Capacity is SUMMED across all batteries (total available)
 * 
 * @note Use case: Parallel battery configurations where total capacity is the sum of
 *       individual batteries, and the system should monitor the weakest battery's voltage
 *       to prevent damage.
 * 
 * @warning Voltage reporting uses the MINIMUM voltage across all batteries to avoid
 *          over-discharge of the weaker battery in a parallel configuration. This ensures
 *          the system responds to the most discharged battery.
 * 
 * @note Current and capacity are summed because in parallel configurations, the total
 *       current capability and energy storage is the sum of all batteries.
 */
class AP_BattMonitor_Sum : public AP_BattMonitor_Backend
{
public:

    /**
     * @brief Constructor for sum/aggregation battery monitor backend
     * 
     * @param[in] mon              Reference to the parent AP_BattMonitor object
     * @param[in] mon_state        Reference to the battery monitor state structure for this instance
     * @param[in] params           Reference to the battery monitor parameters for this instance
     * @param[in] instance         Battery monitor instance number for this backend
     */
    AP_BattMonitor_Sum(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params, uint8_t instance);

    /**
     * @brief Read and aggregate battery voltage and current from multiple battery instances
     * 
     * @details This method aggregates battery data from all instances enabled in _sum_mask at ~10Hz:
     *          - Voltage: Reports the MINIMUM voltage across all enabled battery instances to
     *            prevent over-discharge of the weakest battery in parallel configuration
     *          - Current: SUMS current draw across all enabled battery instances (total system draw)
     *          - Consumed mAh: SUMS consumed capacity across all enabled instances (total consumption)
     *          - Capacity: SUMS total capacity across all enabled instances (total available energy)
     *          - Temperature: Reports average temperature if available from source batteries
     * 
     *          Only battery instances with valid data (healthy status) are included in aggregation.
     */
    void read() override;

    /**
     * @brief Check if this battery monitor provides consumed energy information
     * 
     * @return true if consumed energy (mAh) is available from any source battery instance
     * @note Consumed energy is available whenever current measurement is available
     */
    bool has_consumed_energy() const override { return has_current(); }

    /**
     * @brief Check if this battery monitor provides current measurement
     * 
     * @return true if current measurement is available from any source battery instance
     * @note This flag is set during read() if any of the aggregated battery instances
     *       provide current measurement capability
     */
    bool has_current() const override { return _has_current; }

    /**
     * @brief Check if this battery monitor provides temperature measurement
     * 
     * @return true if temperature measurement is available from any source battery instance
     * @note This flag is set during read() if any of the aggregated battery instances
     *       provide temperature measurement capability
     */
    bool has_temperature() const override { return _has_temperature; }

    /**
     * @brief Initialize the sum battery monitor backend
     * 
     * @note No initialization required for sum backend as it aggregates from other
     *       battery monitor instances that handle their own initialization
     */
    void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int16  _sum_mask;          ///< Bitmask of battery instances to aggregate (bit 0 = instance 0, bit 1 = instance 1, etc.)
    uint8_t _instance;            ///< This backend's battery monitor instance number
    bool _has_current;            ///< True if any source battery instance provides current measurement capability
    bool _has_temperature;        ///< True if any source battery instance provides temperature measurement capability
};

#endif  // AP_BATTERY_SUM_ENABLED
