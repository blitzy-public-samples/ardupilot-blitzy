/**
 * @file LogStructure.h
 * @brief Binary log message structures for battery monitoring telemetry
 * 
 * @details Defines the data structures and format specifications for battery
 *          monitoring data logged to binary logs. Includes primary battery
 *          telemetry (BAT message) and individual cell voltages (BCL message).
 *          These structures are used by the AP_Logger subsystem to record
 *          battery state for post-flight analysis and real-time monitoring.
 */

#pragma once

#include <AP_Logger/LogStructure.h>

/**
 * @brief Battery monitor log message ID registration
 * 
 * @details Defines the set of log message IDs that the battery monitoring
 *          subsystem registers with AP_Logger. These IDs are used to identify
 *          battery-related messages in binary log files.
 * 
 * Registered message IDs:
 * - LOG_BAT_MSG: Primary battery telemetry (voltage, current, capacity)
 * - LOG_BCL_MSG: Individual cell voltages (cells 1-12)
 * 
 * @note BCL2 message (defined elsewhere) reports cells 13+ for batteries with >12 cells
 */
#define LOG_IDS_FROM_BATTMONITOR \
    LOG_BAT_MSG, \
    LOG_BCL_MSG

// @LoggerMessage: BAT
// @Description: Gathered battery data
// @Field: TimeUS: Time since system startup
// @Field: Inst: battery instance number
// @Field: Volt: measured voltage
// @Field: VoltR: estimated resting voltage
// @Field: Curr: measured current
// @Field: CurrTot: consumed Ah, current * time
// @Field: EnrgTot: consumed Wh, energy this battery has expended
// @Field: Temp: measured temperature
// @Field: Res: estimated battery resistance
// @Field: RemPct: remaining percentage
// @Field: H: health
// @Field: SH: state of health percentage.  0 if unknown

/**
 * @struct log_BAT
 * @brief Primary battery telemetry log message structure
 * 
 * @details Records comprehensive battery state including voltage, current,
 *          consumed capacity, temperature, and health metrics. Logged at
 *          regular intervals (typically 10Hz) for all active battery instances.
 *          This is the primary message for battery performance analysis and
 *          capacity estimation verification.
 * 
 * @note Supports up to 10 battery instances (0-9) via the instance field
 * @warning Temperature is stored as centidegrees Celsius (int16) for efficient
 *          packing, not as float like in the runtime state structure
 */
struct PACKED log_BAT {
    LOG_PACKET_HEADER;
    uint64_t time_us;              ///< Timestamp in microseconds since boot
    uint8_t  instance;             ///< Battery instance number (0-9)
    float    voltage;              ///< Measured battery voltage in volts
    float    voltage_resting;      ///< Sag-compensated voltage estimate in volts (no-load voltage)
    float    current_amps;         ///< Measured current in amperes (negative = discharging)
    float    current_total;        ///< Consumed capacity in ampere-hours (Ah)
    float    consumed_wh;          ///< Consumed energy in watt-hours (Wh)
    int16_t  temperature;          ///< Battery temperature in centidegrees Celsius (degrees C * 100)
    float    resistance;           ///< Estimated internal resistance in ohms
    uint8_t  rem_percent;          ///< Remaining capacity percentage (0-100)
    uint8_t  health;               ///< Health status boolean (0=unhealthy, 1=healthy)
    uint8_t  state_of_health_pct;  ///< Degradation metric percentage (0-100, 0=unknown)
};

// @LoggerMessage: BCL
// @Description: Battery cell voltage information
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: battery voltage
// @Field: V1: first cell voltage
// @Field: V2: second cell voltage
// @Field: V3: third cell voltage
// @Field: V4: fourth cell voltage
// @Field: V5: fifth cell voltage
// @Field: V6: sixth cell voltage
// @Field: V7: seventh cell voltage
// @Field: V8: eighth cell voltage
// @Field: V9: ninth cell voltage
// @Field: V10: tenth cell voltage
// @Field: V11: eleventh cell voltage
// @Field: V12: twelfth cell voltage

/**
 * @struct log_BCL
 * @brief Individual cell voltage telemetry log message
 * 
 * @details Records voltage for each individual cell in a battery pack,
 *          enabling detection of cell imbalances and identification of
 *          weak/failing cells. Cell voltages are stored as uint16 in
 *          millivolts for space efficiency.
 * 
 * @note This message reports cells 1-12. For batteries with >12 cells,
 *       cells 13+ are reported in the BCL2 message (defined elsewhere)
 * @note Unused cell positions are set to 0
 * 
 * @see log_BCL2 for cells 13+ on high-cell-count batteries
 */
struct PACKED log_BCL {
    LOG_PACKET_HEADER;
    uint64_t time_us;              ///< Timestamp in microseconds since boot
    uint8_t  instance;             ///< Battery instance number (0-9)
    float    voltage;              ///< Total pack voltage in volts
    uint16_t cell_voltages[12];    ///< Individual cell voltages in millivolts (mV), 0=unused cell
};

/**
 * @brief Log message format registration for battery monitoring
 * 
 * @details Defines the binary log format structures that AP_Logger uses to
 *          encode battery telemetry data. Each entry specifies message ID,
 *          size, name, field types, field names, units, and multipliers.
 * 
 * Format string field type codes:
 * - "Q" = uint64_t (8 bytes)
 * - "B" = uint8_t (1 byte)
 * - "H" = uint16_t (2 bytes)
 * - "f" = float (4 bytes)
 * - "c" = int16_t (2 bytes)
 * 
 * Unit string codes:
 * - "s" = seconds (time)
 * - "#" = dimensionless (counts, instance numbers)
 * - "v" = volts
 * - "A" = amperes
 * - "a" = ampere-hours (capacity)
 * - "X" = watt-hours (energy)
 * - "O" = ohms (resistance)
 * - "w" = degrees Celsius * 100 (centidegrees)
 * - "%" = percent (0-100)
 * - "-" = no unit / not applicable
 * 
 * Multiplier string codes:
 * - "F" = Time field (microseconds)
 * - "0" = No multiplier / raw value
 * - "C" = Centivolt multiplier (0.01)
 * - "?" = Unknown/varies
 * 
 * @note Temperature field in BAT message uses centidegrees (int16) for
 *       efficient packing, requiring conversion from float in code
 */
#define LOG_STRUCTURE_FROM_BATTMONITOR        \
    { LOG_BAT_MSG, sizeof(log_BAT), \
        "BAT", "QBfffffcfBBB", "TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H,SH", "s#vvAaXOw%-%", "F-000C0?0000" , true },  \
    { LOG_BCL_MSG, sizeof(log_BCL), \
        "BCL", "QBfHHHHHHHHHHHH", "TimeUS,Instance,Volt,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12", "s#vvvvvvvvvvvvv", "F-0CCCCCCCCCCCC" , true },
