/**
 * @file AP_Frsky_Backend.h
 * @brief Abstract backend base class for FrSky telemetry protocols (D and SPort variants)
 * 
 * This file defines the common interface and data structures for FrSky telemetry backends,
 * supporting both FrSky D protocol (legacy D-receivers at 9600 baud) and FrSky SPort/Passthrough
 * protocols (X-receivers at 57600 baud). The backend manages telemetry data conversion from
 * ArduPilot's internal representation to FrSky-specific formats.
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_Frsky_config.h"

#if AP_FRSKY_TELEM_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

/**
 * @class AP_Frsky_Backend
 * @brief Abstract base class defining common interface for FrSky D and SPort telemetry backends
 * 
 * @details This base class provides the framework for FrSky telemetry transmission, managing:
 * - Dedicated UART communication thread (1024 bytes stack, PRIORITY_RCIN priority, core 1)
 * - Conversion of ArduPilot state (GPS, attitude, battery, altitude) to FrSky telemetry formats
 * - Common data structures and helper methods for D and SPort protocol implementations
 * - Thread-safe sensor data access and formatting
 * 
 * The backend spawns a dedicated thread that continuously calls the send() method to transmit
 * telemetry data at approximately 200ms intervals. Subclasses must implement the send() pure
 * virtual method to provide protocol-specific transmission logic.
 * 
 * Supported protocols:
 * - FrSky D protocol: Legacy 9600 baud protocol for D-series receivers
 * - FrSky SPort: Smart Port 57600 baud protocol for X-series receivers
 * - FrSky SPort Passthrough: Enhanced SPort with OpenTX compatibility
 * 
 * @note Subclasses must implement the send() pure virtual method
 * @warning Telemetry runs on dedicated thread - sensor reads must be thread-safe
 * 
 * @see AP_Frsky_D for D protocol implementation
 * @see AP_Frsky_SPort for SPort implementation
 * @see AP_Frsky_SPort_Passthrough for Passthrough implementation
 */
class AP_Frsky_Backend
{
public:

    /**
     * @brief Initialize FrSky backend with UART driver
     * 
     * @param[in] port UART driver for FrSky telemetry communication
     * 
     * @note The UART is configured by init_serial_port() with protocol-specific baud rate
     */
    AP_Frsky_Backend(AP_HAL::UARTDriver *port) :
        _port(port) { }

    virtual ~AP_Frsky_Backend()  {}

    /**
     * @brief Initialize backend and start telemetry thread
     * 
     * @details Configures the UART port and spawns the dedicated telemetry thread.
     * The thread runs at PRIORITY_RCIN with 1024 bytes stack on core 1.
     * 
     * @return true if initialization successful and thread started, false otherwise
     * 
     * @note Must be called before telemetry transmission begins
     * @see init_serial_port()
     */
    virtual bool init();
    
    /**
     * @brief Transmit telemetry data (pure virtual - must be implemented by subclass)
     * 
     * @details This method is called repeatedly from the dedicated telemetry thread at
     * approximately 200ms intervals. Subclasses implement protocol-specific data formatting
     * and transmission logic.
     * 
     * @note Called from dedicated thread context, not main loop
     * @warning Must be thread-safe when accessing sensor data
     * 
     * @see loop() for calling context
     */
    virtual void send() = 0;

    /**
     * @brief SPort packet structure with dual views (8 bytes total)
     * 
     * @details Packed union allowing both byte-level and field-level access to SPort packets.
     * Used for efficient packet construction and transmission in SPort and Passthrough protocols.
     * 
     * Packet structure (8 bytes):
     * - Byte 0: Sensor ID (identifies logical sensor)
     * - Byte 1: Frame type (SPORT_DATA_FRAME, SPORT_UPLINK_FRAME, etc.)
     * - Bytes 2-3: Application/Data ID (16-bit identifier for data type)
     * - Bytes 4-7: Data payload (32-bit value, format depends on appid)
     * 
     * The union allows accessing the same memory as either structured fields or raw byte array,
     * facilitating both packet construction and UART transmission.
     */
    typedef union {
        struct PACKED {
            uint8_t sensor;     ///< @brief Sensor ID (SENSOR_ID_VARIO, SENSOR_ID_FAS, etc.)
            uint8_t frame;      ///< @brief Frame type (SPORT_DATA_FRAME=0x10 for data packets)
            uint16_t appid;     ///< @brief Application/data ID (identifies data type: ALT_ID, VARIO_ID, etc.)
            uint32_t data;      ///< @brief 32-bit data payload (format varies by appid)
        };
        uint8_t raw[8];         ///< @brief Byte-level view for UART transmission
    } sport_packet_t;

    /**
     * @brief Get initial baud rate for UART configuration
     * 
     * @details Returns the protocol-specific baud rate for UART initialization.
     * SPort protocol uses 57600 baud (default), while D protocol overrides to 9600 baud.
     * 
     * @return Baud rate in bits per second (57600 for SPort, 9600 for D protocol)
     * 
     * @note Virtual to allow D protocol override to 9600 baud
     * @see AP_Frsky_D::initial_baud() for D protocol override
     */
    virtual uint32_t initial_baud() const
    {
        return 57600;
    }

    /**
     * @brief External telemetry accessor for passthrough consumers
     * 
     * @details Retrieves telemetry packets for external consumers of SPort data
     * (e.g., external radio systems or logging). Base implementation returns false;
     * subclasses implementing passthrough functionality override this method.
     * 
     * @param[out] packet_array Array to fill with sport_packet_t structures
     * @param[in,out] packet_count Input: max packets requested, Output: packets written
     * @param[in] max_size Maximum number of packets that can fit in packet_array
     * 
     * @return true if data available and written to packet_array, false otherwise
     * 
     * @note Used by external consumers of SPort data for passthrough/bridging
     * @see AP_Frsky_SPort_Passthrough::get_telem_data() for implementation
     */
    virtual bool get_telem_data(sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size)
    {
        return false;
    }

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    /**
     * @brief Inject external telemetry data (bidirectional communication)
     * 
     * @details Allows external systems to inject telemetry data into the FrSky stream
     * for bidirectional communication. Base implementation returns false; subclasses
     * supporting bidirectional telemetry override this method.
     * 
     * @param[in] frame Frame type identifier
     * @param[in] appid Application/data ID
     * @param[in] data 32-bit data payload
     * 
     * @return true if data accepted and queued for transmission, false otherwise
     * 
     * @note Only available when HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL is enabled
     * @warning Must be thread-safe for external injection
     */
    virtual bool set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
    {
        return false;
    }
#endif

    /**
     * @brief Queue status text message for transmission
     * 
     * @details Queues a text status message (typically from GCS or system messages)
     * for transmission via FrSky telemetry. Base implementation is a no-op; subclasses
     * supporting text messages override this method.
     * 
     * @param[in] severity Message severity level (MAV_SEVERITY_EMERGENCY to MAV_SEVERITY_DEBUG)
     * @param[in] text Null-terminated message string
     * 
     * @note Text messages are typically sent via SPort Passthrough protocol
     * @see AP_Frsky_SPort_Passthrough::queue_text_message() for implementation
     */
    virtual void queue_text_message(MAV_SEVERITY severity, const char *text) { }

protected:

    AP_HAL::UARTDriver *_port;  ///< @brief UART driver for FrSky telemetry transmission

    /**
     * @brief Configure UART and spawn telemetry thread
     * 
     * @details Initializes the serial port with protocol-specific baud rate from initial_baud(),
     * disables flow control, and spawns the dedicated telemetry thread using
     * hal.scheduler->thread_create() with 1024 bytes stack, PRIORITY_RCIN, and core 1 affinity.
     * 
     * @return true if UART configured and thread spawned successfully, false otherwise
     * 
     * @note Called by init() during backend initialization
     * @see loop() for thread entry point
     */
    virtual bool init_serial_port();

    /**
     * @brief Convert relative altitude to FrSky navigation altitude format
     * 
     * @details Retrieves current relative altitude from AHRS and converts to FrSky format
     * with separate meters and centimeters fields. Updates _SPort_data.alt_nav_meters and
     * _SPort_data.alt_nav_cm.
     * 
     * @note Altitude is relative to home/arming position in meters
     */
    void calc_nav_alt(void);
    
    /**
     * @brief Convert GPS position to FrSky DDDMM.MMMM format
     * 
     * @details Converts decimal degree GPS coordinates (latitude/longitude) from AP_GPS
     * to FrSky format: DDDMM (degrees and minutes) + MMMM (fractional minutes 0-9999).
     * Updates _SPort_data latitude and longitude fields including hemisphere indicators.
     * 
     * @note GPS position format: DDD degrees, MM minutes, MMMM fractional minutes (0.0001 min resolution)
     */
    void calc_gps_position(void);
    
    /**
     * @brief Get RPM sensor value
     * 
     * @details Retrieves RPM value from AP_RPM library for specified sensor instance.
     * 
     * @param[in] instance RPM sensor instance (0-based index)
     * @param[out] value RPM value in revolutions per minute
     * 
     * @return true if sensor available and value retrieved, false if sensor not present
     * 
     * @note RPM sensors must be configured in AP_RPM library
     */
    bool calc_rpm(const uint8_t instance, int32_t &value) const;

    /**
     * @brief Get vertical speed from AHRS
     * 
     * @details Retrieves current vertical velocity from AP_AHRS inertial navigation.
     * Used for variometer telemetry in FrSky protocols.
     * 
     * @return Vertical speed in meters per second (positive = ascending, negative = descending)
     * 
     * @note Vertical speed is earth-frame Z-axis velocity (NED frame: positive down, negated for climb rate)
     */
    float get_vspeed_ms(void);

    /**
     * @brief Convert decimal GPS coordinate to FrSky DDDMM.MMMM format
     * 
     * @details Converts a GPS coordinate from decimal degrees to FrSky format where
     * degrees and minutes are separated. Used internally by calc_gps_position().
     * 
     * @param[in] dec GPS coordinate in decimal degrees (e.g., -122.4567 for longitude)
     * 
     * @return Formatted value suitable for FrSky transmission
     * 
     * @note Format: DDDMM.MMMM where DDD=degrees, MM=minutes, MMMM=fractional minutes
     */
    float format_gps(float dec);

    /**
     * @brief Cached telemetry data in FrSky format
     * 
     * @details Structure holding converted sensor data ready for FrSky transmission.
     * Data is populated by calc_* helper methods and consumed by protocol-specific send() implementations.
     */
    struct {
        int32_t vario_vspd;              ///< @brief Vertical speed in cm/s (positive = ascending)
        char lat_ns, lon_ew;             ///< @brief Hemisphere indicators: 'N'/'S' for latitude, 'E'/'W' for longitude
        uint16_t latdddmm;               ///< @brief Latitude degrees and minutes in DDDMM format
        uint16_t latmmmm;                ///< @brief Latitude fractional minutes (0-9999, representing 0.0000-0.9999 minutes)
        uint16_t londddmm;               ///< @brief Longitude degrees and minutes in DDDMM format
        uint16_t lonmmmm;                ///< @brief Longitude fractional minutes (0-9999, representing 0.0000-0.9999 minutes)
        uint16_t alt_gps_meters;         ///< @brief GPS altitude integer meters above MSL
        uint16_t alt_gps_cm;             ///< @brief GPS altitude fractional centimeters (0-99)
        uint16_t alt_nav_meters;         ///< @brief Navigation altitude integer meters (relative to home)
        uint16_t alt_nav_cm;             ///< @brief Navigation altitude fractional centimeters (0-99)
        int16_t speed_in_meter;          ///< @brief Ground speed integer meters per second
        uint16_t speed_in_centimeter;    ///< @brief Ground speed fractional centimeters per second (0-99)
        uint16_t yaw;                    ///< @brief Heading in centidegrees (0-35999, representing 0.00-359.99 degrees)
    } _SPort_data;

    /*
     * FrSky D Protocol Data IDs (for D-series receivers at 9600 baud)
     * 
     * D protocol uses 8-bit data IDs to identify sensor readings. Values are split into
     * BP (before point/integer part) and AP (after point/fractional part) for precision.
     */
    
    static const uint8_t DATA_ID_GPS_ALT_BP        = 0x01;  ///< @brief GPS altitude integer meters (BP = before point)
    static const uint8_t DATA_ID_TEMP1             = 0x02;  ///< @brief Temperature sensor 1 in degrees Celsius
    static const uint8_t DATA_ID_FUEL              = 0x04;  ///< @brief Fuel level in percent (0-100)
    static const uint8_t DATA_ID_TEMP2             = 0x05;  ///< @brief Temperature sensor 2 in degrees Celsius
    static const uint8_t DATA_ID_GPS_ALT_AP        = 0x09;  ///< @brief GPS altitude fractional meters (AP = after point)
    static const uint8_t DATA_ID_BARO_ALT_BP       = 0x10;  ///< @brief Barometric altitude integer meters (BP)
    static const uint8_t DATA_ID_GPS_SPEED_BP      = 0x11;  ///< @brief GPS speed integer knots (BP)
    static const uint8_t DATA_ID_GPS_LONG_BP       = 0x12;  ///< @brief GPS longitude integer degrees and minutes (BP)
    static const uint8_t DATA_ID_GPS_LAT_BP        = 0x13;  ///< @brief GPS latitude integer degrees and minutes (BP)
    static const uint8_t DATA_ID_GPS_COURS_BP      = 0x14;  ///< @brief GPS course/heading integer degrees (BP)
    static const uint8_t DATA_ID_GPS_SPEED_AP      = 0x19;  ///< @brief GPS speed fractional knots (AP)
    static const uint8_t DATA_ID_GPS_LONG_AP       = 0x1A;  ///< @brief GPS longitude fractional minutes (AP)
    static const uint8_t DATA_ID_GPS_LAT_AP        = 0x1B;  ///< @brief GPS latitude fractional minutes (AP)
    static const uint8_t DATA_ID_BARO_ALT_AP       = 0x21;  ///< @brief Barometric altitude fractional meters (AP)
    static const uint8_t DATA_ID_GPS_LONG_EW       = 0x22;  ///< @brief GPS longitude hemisphere: 0=East, 1=West
    static const uint8_t DATA_ID_GPS_LAT_NS        = 0x23;  ///< @brief GPS latitude hemisphere: 0=North, 1=South
    static const uint8_t DATA_ID_CURRENT           = 0x28;  ///< @brief Battery current in amperes
    static const uint8_t DATA_ID_VFAS              = 0x39;  ///< @brief Battery voltage in volts (VFAS = Voltage FrSky Ampere Sensor)

    static const uint8_t START_STOP_D              = 0x5E;  ///< @brief D protocol frame start/stop delimiter byte
    static const uint8_t BYTESTUFF_D               = 0x5D;  ///< @brief D protocol byte stuffing escape character

    /*
     * FrSky X/SPort Protocol Data IDs (for X-series receivers at 57600 baud)
     * 
     * SPort (Smart Port) protocol uses 16-bit data IDs to identify sensor readings.
     * These IDs are used in sport_packet_t.appid field.
     */
    
    static const uint16_t ALT_ID                    = 0x010F;  ///< @brief Barometric altitude in centimeters (int32_t, relative to home)
    static const uint16_t VARIO_ID                  = 0x011F;  ///< @brief Vertical speed in cm/s (int32_t, positive = ascending)
    static const uint16_t CURR_ID                   = 0x020F;  ///< @brief Battery current in 0.1A (uint32_t)
    static const uint16_t VFAS_ID                   = 0x021F;  ///< @brief Battery voltage in 0.01V (uint32_t)
    static const uint16_t TEMP1_ID                  = 0x040F;  ///< @brief Temperature 1 in degrees Celsius (int32_t)
    static const uint16_t TEMP2_ID                  = 0x041F;  ///< @brief Temperature 2 in degrees Celsius (int32_t)
    static const uint16_t RPM1_ID                   = 0x050E;  ///< @brief RPM sensor 1 in revolutions per minute (uint32_t)
    static const uint16_t RPM2_ID                   = 0x050F;  ///< @brief RPM sensor 2 in revolutions per minute (uint32_t)
    static const uint16_t FUEL_ID                   = 0x060F;  ///< @brief Fuel level in percent 0-100 (uint32_t)
    static const uint16_t GPS_LONG_LATI_FIRST_ID    = 0x0800;  ///< @brief GPS coordinates base ID (lat/lon encoded in data field)
    static const uint16_t GPS_ALT_ID                = 0x082F;  ///< @brief GPS altitude in centimeters (int32_t, MSL)
    static const uint16_t GPS_SPEED_ID              = 0x083F;  ///< @brief GPS ground speed in 0.01 knots (uint32_t)
    static const uint16_t GPS_COURS_ID              = 0x084F;  ///< @brief GPS course/heading in 0.01 degrees (uint32_t, 0-35999)
    static const uint16_t DIY_FIRST_ID              = 0x5000;  ///< @brief First ID for DIY/custom sensor data (0x5000-0xFFFF range)

    /*
     * SPort Protocol Framing Constants
     * 
     * Byte stuffing is used to prevent frame delimiter bytes appearing in data payload.
     */
    
    static const uint8_t FRAME_HEAD                = 0x7E;  ///< @brief Frame header/delimiter byte for SPort packets
    static const uint8_t FRAME_DLE                 = 0x7D;  ///< @brief Data Link Escape for byte stuffing (escapes 0x7E and 0x7D)
    static const uint8_t FRAME_XOR                 = 0x20;  ///< @brief XOR value applied after FRAME_DLE for byte stuffing

    static const uint8_t SPORT_DATA_FRAME          = 0x10;  ///< @brief SPort data frame type identifier (used in sport_packet_t.frame)

    /*
     * FrSky SPort Sensor IDs (logical sensor identifiers)
     * 
     * These IDs identify logical sensors in SPort protocol and are used in sport_packet_t.sensor field.
     * Multiple data types can be sent with the same sensor ID by using different appid values.
     */
    
    static const uint8_t SENSOR_ID_VARIO           = 0x00;  ///< @brief Variometer sensor (altitude, vertical speed)
    static const uint8_t SENSOR_ID_FAS             = 0x22;  ///< @brief FrSky Ampere Sensor (voltage, current, consumption)
    static const uint8_t SENSOR_ID_GPS             = 0x83;  ///< @brief GPS sensor (position, speed, altitude, course)
    static const uint8_t SENSOR_ID_RPM             = 0xE4;  ///< @brief RPM sensor (engine/motor RPM)
    static const uint8_t SENSOR_ID_SP2UR           = 0xC6;  ///< @brief SPort-to-UART adapter sensor (text messages, passthrough)

    /**
     * @brief FrSky telemetry configuration options bitmask
     * 
     * @details Configuration flags for FrSky telemetry behavior, typically set via
     * SERIALx_OPTIONS parameter for the FrSky telemetry port.
     */
    enum frsky_options_e : uint8_t {
        OPTION_AIRSPEED_AND_GROUNDSPEED = 1U<<0,  ///< @brief Enable transmission of both airspeed and groundspeed (default: groundspeed only)
    };

private:

    /**
     * @brief Main telemetry thread loop (never returns)
     * 
     * @details Dedicated thread entry point that continuously calls send() to transmit
     * telemetry data. Runs at PRIORITY_RCIN on core 1 with 1024 bytes stack.
     * The loop runs indefinitely, calling send() at approximately 200ms intervals.
     * 
     * @note This method never returns - it is the thread's main loop
     * @warning Thread safety: All sensor data access must be thread-safe
     * @see init_serial_port() for thread creation
     * @see send() for protocol-specific transmission implementation
     */
    void loop(void);

};

#endif  // AP_FRSKY_TELEM_ENABLED
