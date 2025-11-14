/**
 * @file AP_BattMonitor_Synthetic_Current.h
 * @brief Battery monitor backend that synthesizes current measurement from voltage and throttle
 * 
 * @details This backend provides current estimation when no physical current sensor is available.
 *          Current is derived from voltage drop and throttle percentage, allowing basic current
 *          monitoring and consumed energy tracking without dedicated hardware.
 */

#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

#if AP_BATTERY_SYNTHETIC_CURRENT_ENABLED
/**
 * @class AP_BattMonitor_Synthetic_Current
 * @brief Synthetic current estimation backend derived from AP_BattMonitor_Analog
 * 
 * @details This backend synthesizes current measurements when no physical current sensor is present.
 *          It derives from AP_BattMonitor_Analog to inherit voltage measurement capabilities,
 *          then estimates current based on voltage sag under load and throttle input percentage.
 *          
 *          The synthetic current approach allows systems without current sensors to still:
 *          - Estimate battery current consumption
 *          - Track approximate consumed energy (mAh)
 *          - Trigger current-based failsafes
 *          - Provide current telemetry to ground stations
 * 
 * @note The synthetic current estimation algorithm approximates current from the difference
 *       between maximum voltage and current voltage, combined with throttle percentage.
 *       Accuracy is lower than hardware current sensors but provides useful estimates
 *       for basic battery monitoring when hardware sensors are unavailable.
 */
class AP_BattMonitor_Synthetic_Current : public AP_BattMonitor_Analog
{
public:

    /**
     * @brief Constructor for synthetic current battery monitor backend
     * 
     * @param[in] mon           Reference to parent AP_BattMonitor instance
     * @param[in] mon_state     Reference to battery monitor state structure for this instance
     * @param[in] params        Reference to battery monitor parameters for this instance
     */
    AP_BattMonitor_Synthetic_Current(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /**
     * @brief Read battery voltage and synthesize current estimate from voltage drop and throttle
     * 
     * @details This method reads the actual battery voltage using the inherited analog voltage
     *          measurement, then synthesizes a current estimate using the algorithm:
     *          
     *          Current estimation approach:
     *          - Measures voltage sag (difference between maximum voltage and current voltage)
     *          - Correlates voltage drop with throttle percentage
     *          - Derives approximate current from voltage sag under load
     *          
     *          The synthetic current provides a reasonable approximation for battery monitoring
     *          when physical current sensors are not available. Accuracy depends on battery
     *          characteristics and load consistency.
     * 
     * @note Should be called at 10Hz for consistent current integration and consumed energy tracking
     * 
     * @warning Synthetic current estimates are less accurate than hardware current sensors and
     *          may not accurately reflect current during regenerative events or unusual load profiles
     */
    void read() override;

    /**
     * @brief Check if this battery monitor provides consumed energy information
     * 
     * @details Returns true because synthetic current estimation allows integration
     *          of current over time to calculate consumed energy (mAh), even though
     *          no physical current sensor is present.
     * 
     * @return true - consumed energy tracking is available through current synthesis
     */
    bool has_consumed_energy() const override { return true; }

    /**
     * @brief Check if this battery monitor provides current information
     * 
     * @details Returns true because current is synthesized from voltage and throttle,
     *          providing current measurements even without a physical current sensor.
     * 
     * @return true - current measurement is available through synthesis
     */
    bool has_current() const override { return true; }

    /**
     * @brief Initialize the synthetic current battery monitor backend
     * 
     * @details Empty implementation as initialization is handled by parent AP_BattMonitor_Analog class
     */
    void init(void) override {}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    AP_Float    _max_voltage;           ///< Maximum battery voltage in volts, used as reference in synthetic current calculation algorithm to determine voltage sag under load   
};
#endif
