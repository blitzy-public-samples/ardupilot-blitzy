/**
 * @file AP_RangeFinder_Benewake_TF02.h
 * @brief Benewake TF02 serial LiDAR rangefinder backend (22-meter range)
 * 
 * @details This file implements support for the Benewake TF02 laser rangefinder,
 *          a compact time-of-flight (ToF) distance sensor communicating over UART.
 *          The TF02 uses an infrared laser for distance measurement with a
 *          22-meter maximum range, suitable for altitude hold, terrain following,
 *          and obstacle detection applications.
 *          
 *          The TF02 inherits the Benewake serial protocol from the base class
 *          AP_RangeFinder_Benewake, which handles frame parsing, CRC validation,
 *          and distance extraction. This backend specializes the base class by
 *          configuring TF02-specific parameters: 22m maximum range and signal
 *          byte presence in the protocol frames.
 *          
 *          Serial protocol operates at 115200 baud (8N1) with frame format:
 *          - Header: 0x59 0x59 (fixed frame start marker)
 *          - Distance: 2 bytes (low byte, high byte) in centimeters
 *          - Strength: 2 bytes (low byte, high byte) signal strength
 *          - Signal byte: 1 byte (TF02 includes this field)
 *          - Checksum: 1 byte (frame integrity validation)
 *          
 *          Protocol details and frame parsing are handled by the base class
 *          AP_RangeFinder_Benewake::get_reading() method.
 * 
 * @note TF02 specifications:
 *       - Measuring range: 0.3m to 22m (outdoor), 0.3m to 12m (indoor)
 *       - Accuracy: ±6cm @ <12m, ±1% @ >12m
 *       - Update rate: 100Hz maximum
 *       - Wavelength: 850nm infrared laser (Class 1 laser product, eye-safe)
 *       - Field of view: 2.3° beam divergence angle
 *       - Operating voltage: 5V DC
 *       - Current consumption: <800mA
 *       - Serial interface: UART at 115200 baud, 8N1, 3.3V/5V logic compatible
 * 
 * @note Benewake TF-series family comparison:
 *       - TF02: 22m range, has signal byte, 115200 baud (this class)
 *       - TF03: 180m range, has signal byte, 115200 baud
 *       - TFmini: 12m range, no signal byte, 115200 baud
 *       - TFminiPlus: 12m range with enhanced performance, configurable frame
 * 
 * @warning Serial port configuration:
 *          - Ensure serial port configured for 115200 baud in AP_SerialManager
 *          - Verify sensor powered with stable 5V supply (underpowering causes erratic readings)
 *          - Avoid mounting sensor where propeller wash or exhaust affects beam path
 *          - TF02 susceptible to interference from direct sunlight and reflective surfaces
 * 
 * @warning Performance considerations:
 *          - Maximum range reduces significantly in high ambient light conditions
 *          - Transparent or highly absorptive surfaces (water, dark objects) reduce max range
 *          - Sensor requires ~100ms warm-up time after power-on before accurate readings
 *          - Beam divergence means measurements are area-averaged, not point measurements
 * 
 * @see AP_RangeFinder_Benewake Base class implementing shared Benewake protocol
 * @see AP_RangeFinder_Backend_Serial Serial backend interface and lifecycle
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TF02_ENABLED

#include "AP_RangeFinder_Benewake.h"

/**
 * @class AP_RangeFinder_Benewake_TF02
 * @brief Benewake TF02 22-meter range serial LiDAR rangefinder backend
 * 
 * @details Backend driver for Benewake TF02 time-of-flight laser rangefinder.
 *          The TF02 provides distance measurements from 0.3m to 22m over a
 *          UART serial interface at 115200 baud.
 *          
 *          This class extends AP_RangeFinder_Benewake which handles the Benewake
 *          serial protocol frame parsing. The TF02-specific configuration includes:
 *          - Maximum detection range: 2200 cm (22 meters)
 *          - Signal byte present: yes (TF02 protocol includes signal strength byte)
 *          
 *          The sensor uses an 850nm infrared laser with 2.3° beam divergence,
 *          providing reliable distance measurements for terrain following, altitude
 *          hold, and obstacle detection in multicopter and fixed-wing applications.
 *          
 *          Integration with ArduPilot:
 *          - Set RNGFND1_TYPE = 19 (BenewakeTF02) in parameters
 *          - Configure serial port: SERIAL#_PROTOCOL = 9 (Rangefinder)
 *          - Serial port automatically set to 115200 baud by backend
 *          - Set RNGFND1_ORIENT to match mounting orientation (typically 25 = down)
 *          - Configure RNGFND1_MIN_CM (typically 30cm) and RNGFND1_MAX_CM (2200cm max)
 * 
 * @note Backend lifecycle:
 *       1. Factory creation via create() when Type::BenewakeTF02 selected
 *       2. Serial port initialization at 115200 baud (inherited from base)
 *       3. Continuous frame parsing in update() loop (base class get_reading())
 *       4. Distance published to state.distance_m when valid frames received
 *       5. Status transitions: NotConnected → NoData → Good based on frame reception
 * 
 * @warning Sensor limitations:
 *          - Minimum range 30cm (readings below this may be unreliable)
 *          - Maximum range degrades in sunlight or with low-reflectivity targets
 *          - Single-point measurement (not scanning LiDAR)
 *          - No built-in tilt compensation (use EKF terrain following for slopes)
 */
class AP_RangeFinder_Benewake_TF02 : public AP_RangeFinder_Benewake
{
public:

    /**
     * @brief Factory method to create Benewake TF02 rangefinder backend instance
     * 
     * @details Static factory method called by AP_RangeFinder::detect_instance() when
     *          rangefinder type parameter is set to Type::BenewakeTF02. Allocates a
     *          new TF02 backend on the heap using NEW_NOTHROW to handle potential
     *          memory allocation failures gracefully.
     *          
     *          After creation, the frontend calls init_serial() to bind the backend
     *          to a serial port configured in AP_SerialManager with SERIAL#_PROTOCOL = 9
     *          (Rangefinder). The backend automatically configures 115200 baud rate
     *          through the inherited initial_baudrate() method from base class.
     *          
     *          Memory allocation uses NEW_NOTHROW operator to prevent exceptions on
     *          embedded systems. If allocation fails (returns nullptr), the frontend
     *          logs an error and continues without this rangefinder instance.
     * 
     * @param[in] _state Reference to RangeFinder_State structure for publishing distance measurements
     *                   Contains: distance_m, signal_quality_pct, voltage_mv, status, range_valid_count, last_reading_ms
     * @param[in] _params Reference to AP_RangeFinder_Params for sensor configuration
     *                    Contains: type, orientation, min/max_distance, pos_offset, address, etc.
     * 
     * @return Pointer to newly allocated AP_RangeFinder_Benewake_TF02 backend instance,
     *         or nullptr if memory allocation fails
     * 
     * @note This method is called once during rangefinder initialization (AP_RangeFinder::init())
     * @note The returned pointer is stored in the frontend's drivers[] array and managed by frontend
     * @note Memory is freed when frontend destructor deletes backend pointers
     * 
     * @see AP_RangeFinder::detect_instance() Frontend detection and backend instantiation
     * @see AP_RangeFinder_Backend_Serial::init_serial() Serial port binding after creation
     */
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params) {
        return NEW_NOTHROW AP_RangeFinder_Benewake_TF02(_state, _params);
    }

protected:

    /**
     * @brief Return TF02 maximum detection range in centimeters
     * 
     * @details Overrides pure virtual method from AP_RangeFinder_Benewake base class
     *          to specify the TF02's maximum detection range of 22 meters (2200 cm).
     *          This value is used by the base class and frontend to:
     *          - Validate incoming distance measurements (readings above this are rejected)
     *          - Determine OutOfRangeHigh status transitions
     *          - Configure default max_distance parameter if not user-specified
     *          - Report sensor capabilities to ground control station
     *          
     *          The TF02 datasheet specifies 22m maximum range under optimal conditions:
     *          - Outdoor: 0.3m to 22m with high-reflectivity targets (>90% reflectance)
     *          - Indoor: 0.3m to 12m typical due to reduced ambient light interference
     *          - Degraded range: Direct sunlight, low-reflectivity (<10%), or transparent surfaces
     *          
     *          Actual achievable range depends on:
     *          - Target reflectivity (white surfaces: max range, black surfaces: ~50% range)
     *          - Ambient light conditions (sunlight significantly reduces max range)
     *          - Target size and shape (large flat surfaces better than small or angled)
     *          - Atmospheric conditions (fog, rain, dust degrade performance)
     * 
     * @return Maximum detection range in centimeters (2200 cm = 22 meters)
     * 
     * @note This is the sensor's physical maximum range, not the configured parameter
     * @note User can set RNGFND#_MAX_CM parameter lower than this for application-specific limits
     * @note Returned value is const - TF02 hardware specification, not runtime-configurable
     * 
     * @see AP_RangeFinder_Benewake::get_reading() Uses this value for range validation
     * @see AP_RangeFinder_Params::max_distance User-configurable maximum range limit
     */
    float model_dist_max_cm() const override { return 2200; }

    /**
     * @brief Indicates TF02 protocol includes signal strength byte in frames
     * 
     * @details Overrides virtual method from AP_RangeFinder_Benewake base class
     *          to indicate that TF02 serial protocol frames include a signal strength
     *          byte. This affects frame parsing in the base class get_reading() method.
     *          
     *          Benewake TF02 frame format with signal byte (9 bytes total):
     *          - Byte 0-1: Header (0x59 0x59)
     *          - Byte 2-3: Distance (low byte, high byte) in centimeters
     *          - Byte 4-5: Signal strength (low byte, high byte) - TF02 includes this
     *          - Byte 6: Signal quality byte (0-8 range) - TF02-specific field
     *          - Byte 7: Reserved/mode byte
     *          - Byte 8: Checksum (sum of bytes 0-7, lower 8 bits)
     *          
     *          The signal byte (byte 6) provides measurement quality indication:
     *          - 0-2: Weak signal, unreliable measurement (low target reflectivity)
     *          - 3-5: Moderate signal, acceptable measurement quality
     *          - 6-8: Strong signal, high confidence measurement
     *          
     *          In contrast, TFmini omits the signal quality byte, using 8-byte frames.
     *          The base class uses this flag to correctly parse frame structure and
     *          extract distance from the appropriate byte positions.
     * 
     * @return true indicating TF02 protocol includes signal byte in frame structure
     * 
     * @note Returning true causes base class to parse 9-byte frames instead of 8-byte
     * @note Signal byte value is not currently exposed to frontend (could be used for quality filtering)
     * @note Different from signal_quality_pct() which is not implemented for TF02 backend
     * 
     * @see AP_RangeFinder_Benewake::get_reading() Uses this flag for frame parsing
     * @see AP_RangeFinder_Benewake_TFMini::has_signal_byte() Returns false (8-byte frames)
     */
    bool has_signal_byte() const override { return true; }

private:

    /**
     * @brief Inherit constructor from AP_RangeFinder_Benewake base class
     * 
     * @details C++11 inheriting constructor syntax that imports the base class
     *          constructors, making them available for TF02 construction. This
     *          eliminates need to write explicit forwarding constructors.
     *          
     *          The base class AP_RangeFinder_Benewake constructor signature:
     *          AP_RangeFinder_Benewake(RangeFinder::RangeFinder_State &_state,
     *                                   AP_RangeFinder_Params &_params)
     *          
     *          This constructor:
     *          1. Passes state and params to AP_RangeFinder_Backend_Serial constructor
     *          2. No TF02-specific initialization needed (configuration via virtual methods)
     *          3. Serial port initialization handled by init_serial() call after construction
     *          4. Baud rate set to 115200 via inherited initial_baudrate() virtual method
     *          
     *          Constructor is private because construction only occurs through the
     *          static create() factory method, enforcing controlled instantiation and
     *          memory allocation patterns (NEW_NOTHROW for embedded safety).
     * 
     * @note Constructor invoked by NEW_NOTHROW in create() factory method
     * @note Private access prevents direct instantiation - must use create() factory
     * @note TF02 requires no additional member variables beyond inherited state
     * 
     * @see create() Factory method that invokes this constructor
     * @see AP_RangeFinder_Backend_Serial Base class constructor that initializes serial backend
     */
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;

};

#endif  // AP_RANGEFINDER_BENEWAKE_TF02_ENABLED
