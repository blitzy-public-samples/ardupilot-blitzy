#pragma once

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
 * @file AP_GPS_Blended.h
 * @brief Blended GPS backend that combines measurements from multiple GPS receivers
 * 
 * This backend implements a virtual GPS that blends data from 2 or more physical GPS receivers
 * using inverse-variance weighting based on accuracy metrics. The blending algorithm provides
 * improved accuracy and redundancy by computing a weighted average of position, velocity, and
 * other measurements from all available receivers.
 * 
 * Source: libraries/AP_GPS/AP_GPS_Blended.h
 * Source: libraries/AP_GPS/AP_GPS_Blended.cpp
 */

#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_BLENDED_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"

/**
 * @class AP_GPS_Blended
 * @brief Virtual GPS backend that blends measurements from multiple physical GPS receivers
 * 
 * @details This backend creates a synthetic GPS solution by combining data from 2 or more
 *          physical GPS receivers. The blending algorithm uses inverse-variance weighting
 *          based on accuracy metrics (HDOP, satellite count, velocity accuracy) to compute
 *          optimal blend weights for each receiver.
 * 
 *          The blended solution provides:
 *          - Improved accuracy through weighted averaging of multiple receivers
 *          - Redundancy and fault tolerance when one receiver degrades
 *          - Smooth transitions when receiver quality changes
 * 
 *          Blend weights are recalculated continuously and must sum to 1.0 across all
 *          GPS instances. Health monitoring tracks blending quality and flags conditions
 *          where receivers disagree significantly or blending calculations fail.
 * 
 *          Use case: Systems with multiple GPS receivers benefit from blending for improved
 *          position accuracy and reliability, especially in challenging RF environments.
 * 
 * @note This backend does not interface with physical hardware - it synthesizes data
 *       from other GPS_Backend instances.
 * 
 * Source: libraries/AP_GPS/AP_GPS_Blended.h:27-71
 */
class AP_GPS_Blended : public AP_GPS_Backend
{
public:

    AP_GPS_Blended(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, class AP_GPS::GPS_timing &_timing) :
        AP_GPS_Backend(_gps, _params, _state, nullptr),
        timing{_timing}
        { }

    /**
     * @brief Check if GPS blending is operating within healthy parameters
     * 
     * @details This method is used during pre-arm checks to verify that GPS blending
     *          is producing reliable results. The health counter increments when
     *          blending calculations fail or when receivers disagree significantly
     *          in their measurements.
     * 
     * @return true if blend health counter < 50 (healthy), false if unhealthy
     * 
     * @note Health counter range: 0 = perfectly healthy, 100 = very unhealthy
     * @note Threshold of 50 provides margin before declaring blending unhealthy
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:38-40
     */
    bool is_healthy() const override {
        return (_blend_health_counter < 50);
    }

    /**
     * @brief Read method override (no-op for blended GPS)
     * 
     * @details This backend does not read from hardware. It synthesizes GPS data
     *          from other GPS_Backend instances, so the read operation is a no-op.
     * 
     * @return Always returns true as blended GPS doesn't read from hardware
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:42
     */
    bool read() override { return true; }

    /**
     * @brief Get backend name for identification
     * 
     * @return "Blended" string identifier for this GPS backend type
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:44
     */
    const char *name() const override { return "Blended"; }

    /**
     * @brief Get blended receiver lag combining lags from all receivers
     * 
     * @details Computes a weighted average of receiver lags based on blend weights.
     *          This accounts for the time delay between GPS measurement and when
     *          the data is available to the flight controller.
     * 
     * @param[out] lag_sec Computed lag in seconds based on weighted average of receiver lags
     * 
     * @return true if lag successfully calculated, false otherwise
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:46
     * Source: libraries/AP_GPS/AP_GPS_Blended.cpp
     */
    bool get_lag(float &lag_sec) const override;
    
    /**
     * @brief Get weighted antenna offset position from all GPS receivers
     * 
     * @details Returns the blended antenna position offset computed as a weighted
     *          average of all GPS antenna positions based on current blend weights.
     *          This offset is used to correct vehicle position to account for
     *          GPS antenna location relative to vehicle center.
     * 
     * @return Vector3f antenna offset in meters (NED frame, relative to vehicle center)
     * 
     * @note Offset computed as weighted average based on blend weights
     * @note NED frame: North-East-Down coordinate system
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:47-49
     */
    const Vector3f &get_antenna_offset() const {
        return _blended_antenna_offset;
    }

    /**
     * @brief Calculate blend weights for each GPS receiver based on accuracy metrics
     * 
     * @details Weights are computed using inverse-variance weighting based on accuracy
     *          indicators from each GPS receiver (HDOP, satellite count, velocity accuracy).
     *          Receivers with better accuracy metrics receive higher weights in the blend.
     *          
     *          The algorithm ensures weights sum to 1.0 across all GPS instances and
     *          provides smooth transitions as receiver quality changes.
     * 
     * @return true if weights successfully calculated and sum to 1.0, false if calculation failed
     * 
     * @note This method must be called before calc_state() to update blend weights
     * @note Weights stored in _blend_weights array must sum to 1.0 for valid blending
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:53
     * Source: libraries/AP_GPS/AP_GPS_Blended.cpp
     */
    bool calc_weights(void);
    
    /**
     * @brief Calculate blended GPS state by combining states from all receivers
     * 
     * @details Computes weighted average of position, velocity, and other GPS measurements
     *          from all available receivers using the blend weights calculated by calc_weights().
     *          The resulting blended state is stored in this backend's GPS_State.
     *          
     *          This produces a synthetic GPS solution that optimally combines data from
     *          multiple receivers to improve accuracy and robustness.
     * 
     * @note Must be called after calc_weights() to use current blend weights
     * @note Updates internal GPS_State with blended position, velocity, accuracy metrics
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:55
     * Source: libraries/AP_GPS/AP_GPS_Blended.cpp
     */
    void calc_state(void);

    /**
     * @brief Reset blend health counter to indicate healthy blending
     * 
     * @details Sets the health counter to 0 (perfectly healthy). This is called
     *          when blending successfully produces a valid solution and receivers
     *          are in good agreement.
     * 
     * @note Counter increments when blend calculations fail or receivers disagree
     * @note Health range: 0 = perfectly healthy, 100 = very unhealthy
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:57-59
     */
    void zero_health_counter() {
        _blend_health_counter = 0;
    }

private:

    // GPS blending and switching
    
    /**
     * @brief Weighted average antenna position offset in meters (NED frame)
     * 
     * @details Computed from individual GPS antenna offsets weighted by blend weights.
     *          Used to correct vehicle position to account for GPS antenna location
     *          relative to vehicle center of mass.
     * 
     * Units: meters
     * Frame: NED (North-East-Down) relative to vehicle center
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:64
     */
    Vector3f _blended_antenna_offset;
    
    /**
     * @brief Combined receiver lag in seconds from weighted average
     * 
     * @details Accounts for time delay between GPS measurement epoch and when
     *          data is available to flight controller. Computed as weighted average
     *          of individual receiver lags.
     * 
     * Units: seconds
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:65
     */
    float _blended_lag_sec;
    
    /**
     * @brief Per-receiver blend weight array based on inverse variance
     * 
     * @details Weights computed from receiver accuracy metrics (HDOP, satellite count,
     *          velocity accuracy) using inverse-variance weighting. Receivers with
     *          better accuracy receive higher weights.
     *          
     *          CRITICAL: Weights must sum to 1.0 across all GPS instances for valid blending.
     * 
     * Range: 0.0 to 1.0 per receiver, sum must equal 1.0
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:66
     */
    float _blend_weights[GPS_MAX_RECEIVERS];
    
    /**
     * @brief Health metric indicating blending quality
     * 
     * @details Counter increments when blend calculations fail or receivers disagree
     *          significantly. Decrements or resets to 0 when blending is successful.
     *          Used in pre-arm checks to prevent arming with unhealthy GPS blending.
     * 
     * Range: 0 = perfectly healthy, 100 = very unhealthy
     * Threshold: < 50 considered healthy in is_healthy() check
     * 
     * Source: libraries/AP_GPS/AP_GPS_Blended.h:67
     */
    uint8_t _blend_health_counter;

    AP_GPS::GPS_timing &timing;
    bool _calc_weights(void);
};

#endif  // AP_GPS_BLENDED_ENABLED
