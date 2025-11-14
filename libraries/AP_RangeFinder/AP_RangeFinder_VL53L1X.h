/**
 * @file AP_RangeFinder_VL53L1X.h
 * @brief Driver for ST VL53L1X I2C laser rangefinder
 * 
 * @details This file implements the backend driver for the ST VL53L1X Time-of-Flight
 *          (ToF) laser rangefinder sensor. The VL53L1X is the next-generation sensor
 *          after the VL53L0X, featuring improved performance with a maximum range of
 *          approximately 4 meters and better ambient light rejection.
 *          
 *          Key features:
 *          - I2C communication interface
 *          - Multiple distance modes (Short, Medium, Long)
 *          - Automatic calibration support
 *          - Configurable timing budget
 *          - Continuous measurement mode
 *          - 940nm VCSEL (Vertical Cavity Surface-Emitting Laser) emitter
 *          - Comprehensive error detection and reporting
 *          
 *          The driver manages sensor initialization, configuration, continuous ranging,
 *          and data acquisition through I2C communication. Register addresses are based
 *          on ST's VL53L1X API register map.
 * 
 * @note This driver requires the AP_HAL I2C device abstraction
 * @see AP_RangeFinder_Backend for base class interface
 * @see libraries/AP_RangeFinder/AP_RangeFinder_VL53L1X.cpp for implementation
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L1X_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/I2CDevice.h>

/**
 * @class AP_RangeFinder_VL53L1X
 * @brief Backend driver for ST VL53L1X Time-of-Flight laser rangefinder
 * 
 * @details Implements I2C communication and control logic for the VL53L1X sensor.
 *          This next-generation ToF sensor offers improved performance over the
 *          VL53L0X with 4-meter maximum range, better ambient light immunity,
 *          and more flexible configuration options.
 *          
 *          The driver handles:
 *          - Sensor detection and identification
 *          - Initialization and calibration
 *          - Distance mode configuration (Short/Medium/Long)
 *          - Timing budget management
 *          - Continuous measurement operation
 *          - Range data acquisition and filtering
 *          - Comprehensive error handling
 *          
 *          Communication: I2C at up to 400 kHz (Fast mode)
 *          Default I2C address: 0x29 (7-bit)
 *          
 * @note Inherits standard rangefinder backend interface
 * @warning Ensure proper I2C bus speed configuration for reliable operation
 */
class AP_RangeFinder_VL53L1X : public AP_RangeFinder_Backend
{

public:
    /**
     * @enum DistanceMode
     * @brief Operating distance modes for the VL53L1X sensor
     * 
     * @details Different ranging modes optimize the sensor for specific distance ranges
     *          and ambient light conditions. Each mode adjusts the VCSEL pulse timing
     *          and signal processing parameters.
     */
    enum class DistanceMode { 
        Short,    ///< Short distance mode: optimized for <1.3m, best ambient light immunity
        Medium,   ///< Medium distance mode: balanced performance up to ~3m
        Long,     ///< Long distance mode: maximum range up to ~4m, reduced ambient immunity
        Unknown   ///< Unknown/uninitialized mode
    };

    /**
     * @brief Static detection function to probe and initialize VL53L1X sensor
     * 
     * @details Attempts to detect and initialize a VL53L1X sensor on the I2C bus.
     *          This function is called by the rangefinder driver framework to discover
     *          and instantiate the sensor backend.
     *          
     *          Detection sequence:
     *          1. Attempts I2C communication with the device
     *          2. Reads and verifies sensor identification registers
     *          3. Performs sensor initialization if ID matches
     *          4. Configures the specified distance mode
     *          5. Returns initialized backend instance or nullptr on failure
     * 
     * @param[in,out] _state   Reference to rangefinder state structure
     * @param[in,out] _params  Reference to rangefinder parameters
     * @param[in]     _dev     I2C device pointer (ownership transferred)
     * @param[in]     mode     Desired distance mode (Short/Medium/Long)
     * 
     * @return Pointer to initialized AP_RangeFinder_VL53L1X instance on success, nullptr on failure
     * 
     * @note This is the primary entry point for sensor instantiation
     * @see check_id() for identification verification
     */
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev, DistanceMode mode);

    /**
     * @brief Update rangefinder state with latest measurement
     * 
     * @details Called periodically by the rangefinder framework to update sensor state.
     *          This method reads the latest range measurement from the sensor and updates
     *          the internal state structure with distance, status, and quality information.
     *          
     *          The update process includes:
     *          - Checking for new data availability
     *          - Reading range measurement
     *          - Applying calibration offsets
     *          - Filtering and validation
     *          - Updating state variables
     * 
     * @note Override of AP_RangeFinder_Backend::update()
     * @note This method is called at the scheduler rate (typically 20-50Hz)
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink enum value identifying this as a laser rangefinder.
     *          Used for telemetry reporting to ground control stations.
     * 
     * @return MAV_DISTANCE_SENSOR_LASER indicating laser-based ranging
     * 
     * @note Override of AP_RangeFinder_Backend::_get_mav_distance_sensor_type()
     */
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    /**
     * @enum DeviceError
     * @brief VL53L1X sensor error codes
     * 
     * @details Error status values returned by the VL53L1X in the RESULT__RANGE_STATUS
     *          register. These errors indicate various hardware, calibration, and
     *          measurement issues that can affect range accuracy.
     *          
     *          Error codes are defined by ST's VL53L1X API specification.
     *          
     * @note Error code 0 (NOUPDATE) indicates no new measurement, not an error condition
     * @note Errors 1-3 indicate hardware/calibration failures requiring sensor reset
     * @note Errors 4-8 indicate measurement quality issues
     * @note Error 9 indicates successful range measurement
     */
    enum DeviceError : uint8_t
    {
        NOUPDATE                    = 0,   ///< No range update available (not an error)
        VCSELCONTINUITYTESTFAILURE  = 1,   ///< VCSEL (laser) continuity test failed - hardware fault
        VCSELWATCHDOGTESTFAILURE    = 2,   ///< VCSEL watchdog test failed - hardware fault
        NOVHVVALUEFOUND             = 3,   ///< No valid VHV (VCSEL voltage) value found - calibration issue
        MSRCNOTARGET                = 4,   ///< MSRC (Minimum Signal Rate Check) indicates no target detected
        RANGEPHASECHECK             = 5,   ///< Range phase check failed - signal quality issue
        SIGMATHRESHOLDCHECK         = 6,   ///< Sigma (noise) threshold exceeded - low confidence measurement
        PHASECONSISTENCY            = 7,   ///< Phase consistency check failed - signal interference
        MINCLIP                     = 8,   ///< Minimum distance clipping - target too close
        RANGECOMPLETE               = 9,   ///< Range measurement complete and valid
        ALGOUNDERFLOW               = 10,  ///< Algorithm underflow - processing error
        ALGOOVERFLOW                = 11,  ///< Algorithm overflow - processing error
        RANGEIGNORETHRESHOLD        = 12,  ///< Range ignored due to threshold violation
        USERROICLIP                 = 13,  ///< User ROI (Region of Interest) clipping issue
        REFSPADCHARNOTENOUGHDPADS   = 14,  ///< Reference SPAD characterization: insufficient SPADs
        REFSPADCHARMORETHANTARGET   = 15,  ///< Reference SPAD characterization: exceeds target
        REFSPADCHARLESSTHANTARGET   = 16,  ///< Reference SPAD characterization: below target
        MULTCLIPFAIL                = 17,  ///< Multiple clip failure
        GPHSTREAMCOUNT0READY        = 18,  ///< GPH stream count ready indicator
        RANGECOMPLETE_NO_WRAP_CHECK = 19,  ///< Range complete without wrap check
        EVENTCONSISTENCY            = 20,  ///< Event consistency check failed
        MINSIGNALEVENTCHECK         = 21,  ///< Minimum signal event check failed
        RANGECOMPLETE_MERGED_PULSE  = 22,  ///< Range complete with merged pulse detection
    };

    /**
     * @enum regAddr
     * @brief VL53L1X register address map
     * 
     * @details Complete register address definitions for the VL53L1X Time-of-Flight sensor.
     *          Register addresses are 16-bit values accessed via I2C. The register map includes
     *          configuration, calibration, control, and result registers.
     *          
     *          Register groups:
     *          - 0x0000-0x0087: Configuration and control registers
     *          - 0x0088-0x00FF: Result and status registers
     *          - 0x0100-0x013F: Identification and SPAD configuration
     *          - 0x0400-0x046B: MCU utility and range calculation registers
     *          - 0x0470-0x04B5: Firmware patch registers
     *          - 0x0680-0x07A7: Ranging core hardware registers
     *          - 0x0780-0x0A41: NVM and SPAD enable registers
     *          - 0x0ED0-0x0FFC: Shadow and previous result registers
     *          
     *          Many registers have multi-byte variants (HI/LO for 16-bit, _0/_1/_2/_3 for 32-bit)
     *          to support both byte and word access patterns.
     * 
     * @note Register addresses and definitions based on ST VL53L1X API register map
     * @note Some register names include Pololu additions for 16-bit access convenience
     * @warning Improper register access can damage sensor or cause incorrect operation
     * @see ST VL53L1X datasheet and API documentation for detailed register descriptions
     */
    enum regAddr : uint16_t
    {
        // System configuration and control registers (0x0000-0x0087)
        SOFT_RESET                                                                 = 0x0000,  ///< Software reset control
        I2C_SLAVE__DEVICE_ADDRESS                                                  = 0x0001,
        ANA_CONFIG__VHV_REF_SEL_VDDPIX                                             = 0x0002,
        ANA_CONFIG__VHV_REF_SEL_VQUENCH                                            = 0x0003,
        ANA_CONFIG__REG_AVDD1V2_SEL                                                = 0x0004,
        ANA_CONFIG__FAST_OSC__TRIM                                                 = 0x0005,
        OSC_MEASURED__FAST_OSC__FREQUENCY                                          = 0x0006,
        OSC_MEASURED__FAST_OSC__FREQUENCY_HI                                       = 0x0006,
        OSC_MEASURED__FAST_OSC__FREQUENCY_LO                                       = 0x0007,
        VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                                      = 0x0008,
        VHV_CONFIG__COUNT_THRESH                                                   = 0x0009,
        VHV_CONFIG__OFFSET                                                         = 0x000A,
        VHV_CONFIG__INIT                                                           = 0x000B,
        GLOBAL_CONFIG__SPAD_ENABLES_REF_0                                          = 0x000D,
        GLOBAL_CONFIG__SPAD_ENABLES_REF_1                                          = 0x000E,
        GLOBAL_CONFIG__SPAD_ENABLES_REF_2                                          = 0x000F,
        GLOBAL_CONFIG__SPAD_ENABLES_REF_3                                          = 0x0010,
        GLOBAL_CONFIG__SPAD_ENABLES_REF_4                                          = 0x0011,
        GLOBAL_CONFIG__SPAD_ENABLES_REF_5                                          = 0x0012,
        GLOBAL_CONFIG__REF_EN_START_SELECT                                         = 0x0013,
        REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS                                      = 0x0014,
        REF_SPAD_MAN__REF_LOCATION                                                 = 0x0015,
        ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS                             = 0x0016,
        ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_HI                          = 0x0016,
        ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_LO                          = 0x0017,
        ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS                         = 0x0018,
        ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_HI                      = 0x0018,
        ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_LO                      = 0x0019,
        ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS                         = 0x001A,
        ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_HI                      = 0x001A,
        ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_LO                      = 0x001B,
        REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS                                      = 0x001C,
        REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_HI                                   = 0x001C,
        REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_LO                                   = 0x001D,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM                                         = 0x001E,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM_HI                                      = 0x001E,
        ALGO__PART_TO_PART_RANGE_OFFSET_MM_LO                                      = 0x001F,
        MM_CONFIG__INNER_OFFSET_MM                                                 = 0x0020,
        MM_CONFIG__INNER_OFFSET_MM_HI                                              = 0x0020,
        MM_CONFIG__INNER_OFFSET_MM_LO                                              = 0x0021,
        MM_CONFIG__OUTER_OFFSET_MM                                                 = 0x0022,
        MM_CONFIG__OUTER_OFFSET_MM_HI                                              = 0x0022,
        MM_CONFIG__OUTER_OFFSET_MM_LO                                              = 0x0023,
        DSS_CONFIG__TARGET_TOTAL_RATE_MCPS                                         = 0x0024,
        DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_HI                                      = 0x0024,
        DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_LO                                      = 0x0025,
        DEBUG__CTRL                                                                = 0x0026,
        TEST_MODE__CTRL                                                            = 0x0027,
        CLK_GATING__CTRL                                                           = 0x0028,
        NVM_BIST__CTRL                                                             = 0x0029,
        NVM_BIST__NUM_NVM_WORDS                                                    = 0x002A,
        NVM_BIST__START_ADDRESS                                                    = 0x002B,
        HOST_IF__STATUS                                                            = 0x002C,
        PAD_I2C_HV__CONFIG                                                         = 0x002D,
        PAD_I2C_HV__EXTSUP_CONFIG                                                  = 0x002E,
        GPIO_HV_PAD__CTRL                                                          = 0x002F,
        GPIO_HV_MUX__CTRL                                                          = 0x0030,
        GPIO__TIO_HV_STATUS                                                        = 0x0031,
        GPIO__FIO_HV_STATUS                                                        = 0x0032,
        ANA_CONFIG__SPAD_SEL_PSWIDTH                                               = 0x0033,
        ANA_CONFIG__VCSEL_PULSE_WIDTH_OFFSET                                       = 0x0034,
        ANA_CONFIG__FAST_OSC__CONFIG_CTRL                                          = 0x0035,
        SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS                                  = 0x0036,
        SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS                                = 0x0037,
        SIGMA_ESTIMATOR__SIGMA_REF_MM                                              = 0x0038,
        ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM                               = 0x0039,
        SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_0                                   = 0x003A,
        SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_1                                   = 0x003B,
        ALGO__RANGE_IGNORE_THRESHOLD_MCPS                                          = 0x003C,
        ALGO__RANGE_IGNORE_THRESHOLD_MCPS_HI                                       = 0x003C,
        ALGO__RANGE_IGNORE_THRESHOLD_MCPS_LO                                       = 0x003D,
        ALGO__RANGE_IGNORE_VALID_HEIGHT_MM                                         = 0x003E,
        ALGO__RANGE_MIN_CLIP                                                       = 0x003F,
        ALGO__CONSISTENCY_CHECK__TOLERANCE                                         = 0x0040,
        SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_2                                   = 0x0041,
        SD_CONFIG__RESET_STAGES_MSB                                                = 0x0042,
        SD_CONFIG__RESET_STAGES_LSB                                                = 0x0043,
        GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE                                      = 0x0044,
        GLOBAL_CONFIG__STREAM_DIVIDER                                              = 0x0045,
        SYSTEM__INTERRUPT_CONFIG_GPIO                                              = 0x0046,
        CAL_CONFIG__VCSEL_START                                                    = 0x0047,
        CAL_CONFIG__REPEAT_RATE                                                    = 0x0048,
        CAL_CONFIG__REPEAT_RATE_HI                                                 = 0x0048,
        CAL_CONFIG__REPEAT_RATE_LO                                                 = 0x0049,
        GLOBAL_CONFIG__VCSEL_WIDTH                                                 = 0x004A,
        PHASECAL_CONFIG__TIMEOUT_MACROP                                            = 0x004B,
        PHASECAL_CONFIG__TARGET                                                    = 0x004C,
        PHASECAL_CONFIG__OVERRIDE                                                  = 0x004D,
        DSS_CONFIG__ROI_MODE_CONTROL                                               = 0x004F,
        SYSTEM__THRESH_RATE_HIGH                                                   = 0x0050,
        SYSTEM__THRESH_RATE_HIGH_HI                                                = 0x0050,
        SYSTEM__THRESH_RATE_HIGH_LO                                                = 0x0051,
        SYSTEM__THRESH_RATE_LOW                                                    = 0x0052,
        SYSTEM__THRESH_RATE_LOW_HI                                                 = 0x0052,
        SYSTEM__THRESH_RATE_LOW_LO                                                 = 0x0053,
        DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT                                  = 0x0054,
        DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI                               = 0x0054,
        DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO                               = 0x0055,
        DSS_CONFIG__MANUAL_BLOCK_SELECT                                            = 0x0056,
        DSS_CONFIG__APERTURE_ATTENUATION                                           = 0x0057,
        DSS_CONFIG__MAX_SPADS_LIMIT                                                = 0x0058,
        DSS_CONFIG__MIN_SPADS_LIMIT                                                = 0x0059,
        MM_CONFIG__TIMEOUT_MACROP_A                                                = 0x005A, // added by Pololu for 16-bit accesses
        MM_CONFIG__TIMEOUT_MACROP_A_HI                                             = 0x005A,
        MM_CONFIG__TIMEOUT_MACROP_A_LO                                             = 0x005B,
        MM_CONFIG__TIMEOUT_MACROP_B                                                = 0x005C, // added by Pololu for 16-bit accesses
        MM_CONFIG__TIMEOUT_MACROP_B_HI                                             = 0x005C,
        MM_CONFIG__TIMEOUT_MACROP_B_LO                                             = 0x005D,
        RANGE_CONFIG__TIMEOUT_MACROP_A                                             = 0x005E, // added by Pololu for 16-bit accesses
        RANGE_CONFIG__TIMEOUT_MACROP_A_HI                                          = 0x005E,
        RANGE_CONFIG__TIMEOUT_MACROP_A_LO                                          = 0x005F,
        RANGE_CONFIG__VCSEL_PERIOD_A                                               = 0x0060,
        RANGE_CONFIG__TIMEOUT_MACROP_B                                             = 0x0061, // added by Pololu for 16-bit accesses
        RANGE_CONFIG__TIMEOUT_MACROP_B_HI                                          = 0x0061,
        RANGE_CONFIG__TIMEOUT_MACROP_B_LO                                          = 0x0062,
        RANGE_CONFIG__VCSEL_PERIOD_B                                               = 0x0063,
        RANGE_CONFIG__SIGMA_THRESH                                                 = 0x0064,
        RANGE_CONFIG__SIGMA_THRESH_HI                                              = 0x0064,
        RANGE_CONFIG__SIGMA_THRESH_LO                                              = 0x0065,
        RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                                = 0x0066,
        RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI                             = 0x0066,
        RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO                             = 0x0067,
        RANGE_CONFIG__VALID_PHASE_LOW                                              = 0x0068,
        RANGE_CONFIG__VALID_PHASE_HIGH                                             = 0x0069,
        SYSTEM__INTERMEASUREMENT_PERIOD                                            = 0x006C,
        SYSTEM__INTERMEASUREMENT_PERIOD_3                                          = 0x006C,
        SYSTEM__INTERMEASUREMENT_PERIOD_2                                          = 0x006D,
        SYSTEM__INTERMEASUREMENT_PERIOD_1                                          = 0x006E,
        SYSTEM__INTERMEASUREMENT_PERIOD_0                                          = 0x006F,
        SYSTEM__FRACTIONAL_ENABLE                                                  = 0x0070,
        SYSTEM__GROUPED_PARAMETER_HOLD_0                                           = 0x0071,
        SYSTEM__THRESH_HIGH                                                        = 0x0072,
        SYSTEM__THRESH_HIGH_HI                                                     = 0x0072,
        SYSTEM__THRESH_HIGH_LO                                                     = 0x0073,
        SYSTEM__THRESH_LOW                                                         = 0x0074,
        SYSTEM__THRESH_LOW_HI                                                      = 0x0074,
        SYSTEM__THRESH_LOW_LO                                                      = 0x0075,
        SYSTEM__ENABLE_XTALK_PER_QUADRANT                                          = 0x0076,
        SYSTEM__SEED_CONFIG                                                        = 0x0077,
        SD_CONFIG__WOI_SD0                                                         = 0x0078,
        SD_CONFIG__WOI_SD1                                                         = 0x0079,
        SD_CONFIG__INITIAL_PHASE_SD0                                               = 0x007A,
        SD_CONFIG__INITIAL_PHASE_SD1                                               = 0x007B,
        SYSTEM__GROUPED_PARAMETER_HOLD_1                                           = 0x007C,
        SD_CONFIG__FIRST_ORDER_SELECT                                              = 0x007D,
        SD_CONFIG__QUANTIFIER                                                      = 0x007E,
        ROI_CONFIG__USER_ROI_CENTRE_SPAD                                           = 0x007F,
        ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE                              = 0x0080,
        SYSTEM__SEQUENCE_CONFIG                                                    = 0x0081,
        SYSTEM__GROUPED_PARAMETER_HOLD                                             = 0x0082,
        POWER_MANAGEMENT__GO1_POWER_FORCE                                          = 0x0083,
        SYSTEM__STREAM_COUNT_CTRL                                                  = 0x0084,
        FIRMWARE__ENABLE                                                           = 0x0085,
        SYSTEM__INTERRUPT_CLEAR                                                    = 0x0086,
        SYSTEM__MODE_START                                                         = 0x0087,
        
        // Result and status registers (0x0088-0x00FF)
        RESULT__INTERRUPT_STATUS                                                   = 0x0088,
        RESULT__RANGE_STATUS                                                       = 0x0089,
        RESULT__REPORT_STATUS                                                      = 0x008A,
        RESULT__STREAM_COUNT                                                       = 0x008B,
        RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                                     = 0x008C,
        RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI                                  = 0x008C,
        RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO                                  = 0x008D,
        RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0                                    = 0x008E,
        RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI                                 = 0x008E,
        RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO                                 = 0x008F,
        RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                                        = 0x0090,
        RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI                                     = 0x0090,
        RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO                                     = 0x0091,
        RESULT__SIGMA_SD0                                                          = 0x0092,
        RESULT__SIGMA_SD0_HI                                                       = 0x0092,
        RESULT__SIGMA_SD0_LO                                                       = 0x0093,
        RESULT__PHASE_SD0                                                          = 0x0094,
        RESULT__PHASE_SD0_HI                                                       = 0x0094,
        RESULT__PHASE_SD0_LO                                                       = 0x0095,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                             = 0x0096,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI                          = 0x0096,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO                          = 0x0097,
        RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0                = 0x0098,
        RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI             = 0x0098,
        RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO             = 0x0099,
        RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0                                = 0x009A,
        RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                             = 0x009A,
        RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                             = 0x009B,
        RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0                                = 0x009C,
        RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                             = 0x009C,
        RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                             = 0x009D,
        RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0                                     = 0x009E,
        RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI                                  = 0x009E,
        RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO                                  = 0x009F,
        RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1                                     = 0x00A0,
        RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI                                  = 0x00A0,
        RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO                                  = 0x00A1,
        RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1                                    = 0x00A2,
        RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI                                 = 0x00A2,
        RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO                                 = 0x00A3,
        RESULT__AMBIENT_COUNT_RATE_MCPS_SD1                                        = 0x00A4,
        RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI                                     = 0x00A4,
        RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO                                     = 0x00A5,
        RESULT__SIGMA_SD1                                                          = 0x00A6,
        RESULT__SIGMA_SD1_HI                                                       = 0x00A6,
        RESULT__SIGMA_SD1_LO                                                       = 0x00A7,
        RESULT__PHASE_SD1                                                          = 0x00A8,
        RESULT__PHASE_SD1_HI                                                       = 0x00A8,
        RESULT__PHASE_SD1_LO                                                       = 0x00A9,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1                             = 0x00AA,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI                          = 0x00AA,
        RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO                          = 0x00AB,
        RESULT__SPARE_0_SD1                                                        = 0x00AC,
        RESULT__SPARE_0_SD1_HI                                                     = 0x00AC,
        RESULT__SPARE_0_SD1_LO                                                     = 0x00AD,
        RESULT__SPARE_1_SD1                                                        = 0x00AE,
        RESULT__SPARE_1_SD1_HI                                                     = 0x00AE,
        RESULT__SPARE_1_SD1_LO                                                     = 0x00AF,
        RESULT__SPARE_2_SD1                                                        = 0x00B0,
        RESULT__SPARE_2_SD1_HI                                                     = 0x00B0,
        RESULT__SPARE_2_SD1_LO                                                     = 0x00B1,
        RESULT__SPARE_3_SD1                                                        = 0x00B2,
        RESULT__THRESH_INFO                                                        = 0x00B3,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0                                     = 0x00B4,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3                                   = 0x00B4,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2                                   = 0x00B5,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1                                   = 0x00B6,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0                                   = 0x00B7,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD0                                      = 0x00B8,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3                                    = 0x00B8,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2                                    = 0x00B9,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1                                    = 0x00BA,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0                                    = 0x00BB,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0                                       = 0x00BC,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3                                     = 0x00BC,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2                                     = 0x00BD,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1                                     = 0x00BE,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0                                     = 0x00BF,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0                                     = 0x00C0,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3                                   = 0x00C0,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2                                   = 0x00C1,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1                                   = 0x00C2,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0                                   = 0x00C3,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1                                     = 0x00C4,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3                                   = 0x00C4,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2                                   = 0x00C5,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1                                   = 0x00C6,
        RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0                                   = 0x00C7,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD1                                      = 0x00C8,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3                                    = 0x00C8,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2                                    = 0x00C9,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1                                    = 0x00CA,
        RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0                                    = 0x00CB,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1                                       = 0x00CC,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3                                     = 0x00CC,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2                                     = 0x00CD,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1                                     = 0x00CE,
        RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0                                     = 0x00CF,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1                                     = 0x00D0,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3                                   = 0x00D0,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2                                   = 0x00D1,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1                                   = 0x00D2,
        RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0                                   = 0x00D3,
        RESULT_CORE__SPARE_0                                                       = 0x00D4,
        PHASECAL_RESULT__REFERENCE_PHASE                                           = 0x00D6,
        PHASECAL_RESULT__REFERENCE_PHASE_HI                                        = 0x00D6,
        PHASECAL_RESULT__REFERENCE_PHASE_LO                                        = 0x00D7,
        PHASECAL_RESULT__VCSEL_START                                               = 0x00D8,
        REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS                                 = 0x00D9,
        REF_SPAD_CHAR_RESULT__REF_LOCATION                                         = 0x00DA,
        VHV_RESULT__COLDBOOT_STATUS                                                = 0x00DB,
        VHV_RESULT__SEARCH_RESULT                                                  = 0x00DC,
        VHV_RESULT__LATEST_SETTING                                                 = 0x00DD,
        RESULT__OSC_CALIBRATE_VAL                                                  = 0x00DE,
        RESULT__OSC_CALIBRATE_VAL_HI                                               = 0x00DE,
        RESULT__OSC_CALIBRATE_VAL_LO                                               = 0x00DF,
        ANA_CONFIG__POWERDOWN_GO1                                                  = 0x00E0,
        ANA_CONFIG__REF_BG_CTRL                                                    = 0x00E1,
        ANA_CONFIG__REGDVDD1V2_CTRL                                                = 0x00E2,
        ANA_CONFIG__OSC_SLOW_CTRL                                                  = 0x00E3,
        TEST_MODE__STATUS                                                          = 0x00E4,
        FIRMWARE__SYSTEM_STATUS                                                    = 0x00E5,
        FIRMWARE__MODE_STATUS                                                      = 0x00E6,
        FIRMWARE__SECONDARY_MODE_STATUS                                            = 0x00E7,
        FIRMWARE__CAL_REPEAT_RATE_COUNTER                                          = 0x00E8,
        FIRMWARE__CAL_REPEAT_RATE_COUNTER_HI                                       = 0x00E8,
        FIRMWARE__CAL_REPEAT_RATE_COUNTER_LO                                       = 0x00E9,
        FIRMWARE__HISTOGRAM_BIN                                                    = 0x00EA,
        GPH__SYSTEM__THRESH_HIGH                                                   = 0x00EC,
        GPH__SYSTEM__THRESH_HIGH_HI                                                = 0x00EC,
        GPH__SYSTEM__THRESH_HIGH_LO                                                = 0x00ED,
        GPH__SYSTEM__THRESH_LOW                                                    = 0x00EE,
        GPH__SYSTEM__THRESH_LOW_HI                                                 = 0x00EE,
        GPH__SYSTEM__THRESH_LOW_LO                                                 = 0x00EF,
        GPH__SYSTEM__ENABLE_XTALK_PER_QUADRANT                                     = 0x00F0,
        GPH__SPARE_0                                                               = 0x00F1,
        GPH__SD_CONFIG__WOI_SD0                                                    = 0x00F2,
        GPH__SD_CONFIG__WOI_SD1                                                    = 0x00F3,
        GPH__SD_CONFIG__INITIAL_PHASE_SD0                                          = 0x00F4,
        GPH__SD_CONFIG__INITIAL_PHASE_SD1                                          = 0x00F5,
        GPH__SD_CONFIG__FIRST_ORDER_SELECT                                         = 0x00F6,
        GPH__SD_CONFIG__QUANTIFIER                                                 = 0x00F7,
        GPH__ROI_CONFIG__USER_ROI_CENTRE_SPAD                                      = 0x00F8,
        GPH__ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE                         = 0x00F9,
        GPH__SYSTEM__SEQUENCE_CONFIG                                               = 0x00FA,
        GPH__GPH_ID                                                                = 0x00FB,
        SYSTEM__INTERRUPT_SET                                                      = 0x00FC,
        INTERRUPT_MANAGER__ENABLES                                                 = 0x00FD,
        INTERRUPT_MANAGER__CLEAR                                                   = 0x00FE,
        INTERRUPT_MANAGER__STATUS                                                  = 0x00FF,
        
        // Identification and configuration registers (0x0100-0x013F)
        MCU_TO_HOST_BANK__WR_ACCESS_EN                                             = 0x0100,
        POWER_MANAGEMENT__GO1_RESET_STATUS                                         = 0x0101,
        PAD_STARTUP_MODE__VALUE_RO                                                 = 0x0102,
        PAD_STARTUP_MODE__VALUE_CTRL                                               = 0x0103,
        PLL_PERIOD_US                                                              = 0x0104,
        PLL_PERIOD_US_3                                                            = 0x0104,
        PLL_PERIOD_US_2                                                            = 0x0105,
        PLL_PERIOD_US_1                                                            = 0x0106,
        PLL_PERIOD_US_0                                                            = 0x0107,
        INTERRUPT_SCHEDULER__DATA_OUT                                              = 0x0108,
        INTERRUPT_SCHEDULER__DATA_OUT_3                                            = 0x0108,
        INTERRUPT_SCHEDULER__DATA_OUT_2                                            = 0x0109,
        INTERRUPT_SCHEDULER__DATA_OUT_1                                            = 0x010A,
        INTERRUPT_SCHEDULER__DATA_OUT_0                                            = 0x010B,
        NVM_BIST__COMPLETE                                                         = 0x010C,
        NVM_BIST__STATUS                                                           = 0x010D,
        IDENTIFICATION__MODEL_ID                                                   = 0x010F,
        IDENTIFICATION__MODULE_TYPE                                                = 0x0110,
        IDENTIFICATION__REVISION_ID                                                = 0x0111,
        IDENTIFICATION__MODULE_ID                                                  = 0x0112,
        IDENTIFICATION__MODULE_ID_HI                                               = 0x0112,
        IDENTIFICATION__MODULE_ID_LO                                               = 0x0113,
        ANA_CONFIG__FAST_OSC__TRIM_MAX                                             = 0x0114,
        ANA_CONFIG__FAST_OSC__FREQ_SET                                             = 0x0115,
        ANA_CONFIG__VCSEL_TRIM                                                     = 0x0116,
        ANA_CONFIG__VCSEL_SELION                                                   = 0x0117,
        ANA_CONFIG__VCSEL_SELION_MAX                                               = 0x0118,
        PROTECTED_LASER_SAFETY__LOCK_BIT                                           = 0x0119,
        LASER_SAFETY__KEY                                                          = 0x011A,
        LASER_SAFETY__KEY_RO                                                       = 0x011B,
        LASER_SAFETY__CLIP                                                         = 0x011C,
        LASER_SAFETY__MULT                                                         = 0x011D,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_0                                          = 0x011E,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_1                                          = 0x011F,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_2                                          = 0x0120,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_3                                          = 0x0121,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_4                                          = 0x0122,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_5                                          = 0x0123,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_6                                          = 0x0124,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_7                                          = 0x0125,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_8                                          = 0x0126,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_9                                          = 0x0127,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_10                                         = 0x0128,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_11                                         = 0x0129,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_12                                         = 0x012A,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_13                                         = 0x012B,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_14                                         = 0x012C,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_15                                         = 0x012D,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_16                                         = 0x012E,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_17                                         = 0x012F,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_18                                         = 0x0130,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_19                                         = 0x0131,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_20                                         = 0x0132,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_21                                         = 0x0133,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_22                                         = 0x0134,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_23                                         = 0x0135,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_24                                         = 0x0136,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_25                                         = 0x0137,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_26                                         = 0x0138,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_27                                         = 0x0139,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_28                                         = 0x013A,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_29                                         = 0x013B,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_30                                         = 0x013C,
        GLOBAL_CONFIG__SPAD_ENABLES_RTN_31                                         = 0x013D,
        ROI_CONFIG__MODE_ROI_CENTRE_SPAD                                           = 0x013E,
        ROI_CONFIG__MODE_ROI_XY_SIZE                                               = 0x013F,
        GO2_HOST_BANK_ACCESS__OVERRIDE                                             = 0x0300,
        
        // MCU utility and range calculation registers (0x0400-0x046F)
        MCU_UTIL_MULTIPLIER__MULTIPLICAND                                          = 0x0400,
        MCU_UTIL_MULTIPLIER__MULTIPLICAND_3                                        = 0x0400,
        MCU_UTIL_MULTIPLIER__MULTIPLICAND_2                                        = 0x0401,
        MCU_UTIL_MULTIPLIER__MULTIPLICAND_1                                        = 0x0402,
        MCU_UTIL_MULTIPLIER__MULTIPLICAND_0                                        = 0x0403,
        MCU_UTIL_MULTIPLIER__MULTIPLIER                                            = 0x0404,
        MCU_UTIL_MULTIPLIER__MULTIPLIER_3                                          = 0x0404,
        MCU_UTIL_MULTIPLIER__MULTIPLIER_2                                          = 0x0405,
        MCU_UTIL_MULTIPLIER__MULTIPLIER_1                                          = 0x0406,
        MCU_UTIL_MULTIPLIER__MULTIPLIER_0                                          = 0x0407,
        MCU_UTIL_MULTIPLIER__PRODUCT_HI                                            = 0x0408,
        MCU_UTIL_MULTIPLIER__PRODUCT_HI_3                                          = 0x0408,
        MCU_UTIL_MULTIPLIER__PRODUCT_HI_2                                          = 0x0409,
        MCU_UTIL_MULTIPLIER__PRODUCT_HI_1                                          = 0x040A,
        MCU_UTIL_MULTIPLIER__PRODUCT_HI_0                                          = 0x040B,
        MCU_UTIL_MULTIPLIER__PRODUCT_LO                                            = 0x040C,
        MCU_UTIL_MULTIPLIER__PRODUCT_LO_3                                          = 0x040C,
        MCU_UTIL_MULTIPLIER__PRODUCT_LO_2                                          = 0x040D,
        MCU_UTIL_MULTIPLIER__PRODUCT_LO_1                                          = 0x040E,
        MCU_UTIL_MULTIPLIER__PRODUCT_LO_0                                          = 0x040F,
        MCU_UTIL_MULTIPLIER__START                                                 = 0x0410,
        MCU_UTIL_MULTIPLIER__STATUS                                                = 0x0411,
        MCU_UTIL_DIVIDER__START                                                    = 0x0412,
        MCU_UTIL_DIVIDER__STATUS                                                   = 0x0413,
        MCU_UTIL_DIVIDER__DIVIDEND                                                 = 0x0414,
        MCU_UTIL_DIVIDER__DIVIDEND_3                                               = 0x0414,
        MCU_UTIL_DIVIDER__DIVIDEND_2                                               = 0x0415,
        MCU_UTIL_DIVIDER__DIVIDEND_1                                               = 0x0416,
        MCU_UTIL_DIVIDER__DIVIDEND_0                                               = 0x0417,
        MCU_UTIL_DIVIDER__DIVISOR                                                  = 0x0418,
        MCU_UTIL_DIVIDER__DIVISOR_3                                                = 0x0418,
        MCU_UTIL_DIVIDER__DIVISOR_2                                                = 0x0419,
        MCU_UTIL_DIVIDER__DIVISOR_1                                                = 0x041A,
        MCU_UTIL_DIVIDER__DIVISOR_0                                                = 0x041B,
        MCU_UTIL_DIVIDER__QUOTIENT                                                 = 0x041C,
        MCU_UTIL_DIVIDER__QUOTIENT_3                                               = 0x041C,
        MCU_UTIL_DIVIDER__QUOTIENT_2                                               = 0x041D,
        MCU_UTIL_DIVIDER__QUOTIENT_1                                               = 0x041E,
        MCU_UTIL_DIVIDER__QUOTIENT_0                                               = 0x041F,
        TIMER0__VALUE_IN                                                           = 0x0420,
        TIMER0__VALUE_IN_3                                                         = 0x0420,
        TIMER0__VALUE_IN_2                                                         = 0x0421,
        TIMER0__VALUE_IN_1                                                         = 0x0422,
        TIMER0__VALUE_IN_0                                                         = 0x0423,
        TIMER1__VALUE_IN                                                           = 0x0424,
        TIMER1__VALUE_IN_3                                                         = 0x0424,
        TIMER1__VALUE_IN_2                                                         = 0x0425,
        TIMER1__VALUE_IN_1                                                         = 0x0426,
        TIMER1__VALUE_IN_0                                                         = 0x0427,
        TIMER0__CTRL                                                               = 0x0428,
        TIMER1__CTRL                                                               = 0x0429,
        MCU_GENERAL_PURPOSE__GP_0                                                  = 0x042C,
        MCU_GENERAL_PURPOSE__GP_1                                                  = 0x042D,
        MCU_GENERAL_PURPOSE__GP_2                                                  = 0x042E,
        MCU_GENERAL_PURPOSE__GP_3                                                  = 0x042F,
        MCU_RANGE_CALC__CONFIG                                                     = 0x0430,
        MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE                                     = 0x0432,
        MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_HI                                  = 0x0432,
        MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_LO                                  = 0x0433,
        MCU_RANGE_CALC__SPARE_4                                                    = 0x0434,
        MCU_RANGE_CALC__SPARE_4_3                                                  = 0x0434,
        MCU_RANGE_CALC__SPARE_4_2                                                  = 0x0435,
        MCU_RANGE_CALC__SPARE_4_1                                                  = 0x0436,
        MCU_RANGE_CALC__SPARE_4_0                                                  = 0x0437,
        MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC                                  = 0x0438,
        MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_HI                               = 0x0438,
        MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_LO                               = 0x0439,
        MCU_RANGE_CALC__ALGO_VCSEL_PERIOD                                          = 0x043C,
        MCU_RANGE_CALC__SPARE_5                                                    = 0x043D,
        MCU_RANGE_CALC__ALGO_TOTAL_PERIODS                                         = 0x043E,
        MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_HI                                      = 0x043E,
        MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_LO                                      = 0x043F,
        MCU_RANGE_CALC__ALGO_ACCUM_PHASE                                           = 0x0440,
        MCU_RANGE_CALC__ALGO_ACCUM_PHASE_3                                         = 0x0440,
        MCU_RANGE_CALC__ALGO_ACCUM_PHASE_2                                         = 0x0441,
        MCU_RANGE_CALC__ALGO_ACCUM_PHASE_1                                         = 0x0442,
        MCU_RANGE_CALC__ALGO_ACCUM_PHASE_0                                         = 0x0443,
        MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS                                         = 0x0444,
        MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_3                                       = 0x0444,
        MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_2                                       = 0x0445,
        MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_1                                       = 0x0446,
        MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_0                                       = 0x0447,
        MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS                                        = 0x0448,
        MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_3                                      = 0x0448,
        MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_2                                      = 0x0449,
        MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_1                                      = 0x044A,
        MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_0                                      = 0x044B,
        MCU_RANGE_CALC__SPARE_6                                                    = 0x044C,
        MCU_RANGE_CALC__SPARE_6_HI                                                 = 0x044C,
        MCU_RANGE_CALC__SPARE_6_LO                                                 = 0x044D,
        MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD                                   = 0x044E,
        MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_HI                                = 0x044E,
        MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_LO                                = 0x044F,
        MCU_RANGE_CALC__NUM_SPADS                                                  = 0x0450,
        MCU_RANGE_CALC__NUM_SPADS_HI                                               = 0x0450,
        MCU_RANGE_CALC__NUM_SPADS_LO                                               = 0x0451,
        MCU_RANGE_CALC__PHASE_OUTPUT                                               = 0x0452,
        MCU_RANGE_CALC__PHASE_OUTPUT_HI                                            = 0x0452,
        MCU_RANGE_CALC__PHASE_OUTPUT_LO                                            = 0x0453,
        MCU_RANGE_CALC__RATE_PER_SPAD_MCPS                                         = 0x0454,
        MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_3                                       = 0x0454,
        MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_2                                       = 0x0455,
        MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_1                                       = 0x0456,
        MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_0                                       = 0x0457,
        MCU_RANGE_CALC__SPARE_7                                                    = 0x0458,
        MCU_RANGE_CALC__SPARE_8                                                    = 0x0459,
        MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS                                      = 0x045A,
        MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_HI                                   = 0x045A,
        MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_LO                                   = 0x045B,
        MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS                                       = 0x045C,
        MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_HI                                    = 0x045C,
        MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_LO                                    = 0x045D,
        MCU_RANGE_CALC__AMBIENT_RATE_MCPS                                          = 0x045E,
        MCU_RANGE_CALC__AMBIENT_RATE_MCPS_HI                                       = 0x045E,
        MCU_RANGE_CALC__AMBIENT_RATE_MCPS_LO                                       = 0x045F,
        MCU_RANGE_CALC__XTALK                                                      = 0x0460,
        MCU_RANGE_CALC__XTALK_HI                                                   = 0x0460,
        MCU_RANGE_CALC__XTALK_LO                                                   = 0x0461,
        MCU_RANGE_CALC__CALC_STATUS                                                = 0x0462,
        MCU_RANGE_CALC__DEBUG                                                      = 0x0463,
        MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS                           = 0x0464,
        MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_HI                        = 0x0464,
        MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_LO                        = 0x0465,
        MCU_RANGE_CALC__SPARE_0                                                    = 0x0468,
        MCU_RANGE_CALC__SPARE_1                                                    = 0x0469,
        MCU_RANGE_CALC__SPARE_2                                                    = 0x046A,
        MCU_RANGE_CALC__SPARE_3                                                    = 0x046B,
        
        // Firmware patch registers (0x0470-0x04B5)
        PATCH__CTRL                                                                = 0x0470,
        PATCH__JMP_ENABLES                                                         = 0x0472,
        PATCH__JMP_ENABLES_HI                                                      = 0x0472,
        PATCH__JMP_ENABLES_LO                                                      = 0x0473,
        PATCH__DATA_ENABLES                                                        = 0x0474,
        PATCH__DATA_ENABLES_HI                                                     = 0x0474,
        PATCH__DATA_ENABLES_LO                                                     = 0x0475,
        PATCH__OFFSET_0                                                            = 0x0476,
        PATCH__OFFSET_0_HI                                                         = 0x0476,
        PATCH__OFFSET_0_LO                                                         = 0x0477,
        PATCH__OFFSET_1                                                            = 0x0478,
        PATCH__OFFSET_1_HI                                                         = 0x0478,
        PATCH__OFFSET_1_LO                                                         = 0x0479,
        PATCH__OFFSET_2                                                            = 0x047A,
        PATCH__OFFSET_2_HI                                                         = 0x047A,
        PATCH__OFFSET_2_LO                                                         = 0x047B,
        PATCH__OFFSET_3                                                            = 0x047C,
        PATCH__OFFSET_3_HI                                                         = 0x047C,
        PATCH__OFFSET_3_LO                                                         = 0x047D,
        PATCH__OFFSET_4                                                            = 0x047E,
        PATCH__OFFSET_4_HI                                                         = 0x047E,
        PATCH__OFFSET_4_LO                                                         = 0x047F,
        PATCH__OFFSET_5                                                            = 0x0480,
        PATCH__OFFSET_5_HI                                                         = 0x0480,
        PATCH__OFFSET_5_LO                                                         = 0x0481,
        PATCH__OFFSET_6                                                            = 0x0482,
        PATCH__OFFSET_6_HI                                                         = 0x0482,
        PATCH__OFFSET_6_LO                                                         = 0x0483,
        PATCH__OFFSET_7                                                            = 0x0484,
        PATCH__OFFSET_7_HI                                                         = 0x0484,
        PATCH__OFFSET_7_LO                                                         = 0x0485,
        PATCH__OFFSET_8                                                            = 0x0486,
        PATCH__OFFSET_8_HI                                                         = 0x0486,
        PATCH__OFFSET_8_LO                                                         = 0x0487,
        PATCH__OFFSET_9                                                            = 0x0488,
        PATCH__OFFSET_9_HI                                                         = 0x0488,
        PATCH__OFFSET_9_LO                                                         = 0x0489,
        PATCH__OFFSET_10                                                           = 0x048A,
        PATCH__OFFSET_10_HI                                                        = 0x048A,
        PATCH__OFFSET_10_LO                                                        = 0x048B,
        PATCH__OFFSET_11                                                           = 0x048C,
        PATCH__OFFSET_11_HI                                                        = 0x048C,
        PATCH__OFFSET_11_LO                                                        = 0x048D,
        PATCH__OFFSET_12                                                           = 0x048E,
        PATCH__OFFSET_12_HI                                                        = 0x048E,
        PATCH__OFFSET_12_LO                                                        = 0x048F,
        PATCH__OFFSET_13                                                           = 0x0490,
        PATCH__OFFSET_13_HI                                                        = 0x0490,
        PATCH__OFFSET_13_LO                                                        = 0x0491,
        PATCH__OFFSET_14                                                           = 0x0492,
        PATCH__OFFSET_14_HI                                                        = 0x0492,
        PATCH__OFFSET_14_LO                                                        = 0x0493,
        PATCH__OFFSET_15                                                           = 0x0494,
        PATCH__OFFSET_15_HI                                                        = 0x0494,
        PATCH__OFFSET_15_LO                                                        = 0x0495,
        PATCH__ADDRESS_0                                                           = 0x0496,
        PATCH__ADDRESS_0_HI                                                        = 0x0496,
        PATCH__ADDRESS_0_LO                                                        = 0x0497,
        PATCH__ADDRESS_1                                                           = 0x0498,
        PATCH__ADDRESS_1_HI                                                        = 0x0498,
        PATCH__ADDRESS_1_LO                                                        = 0x0499,
        PATCH__ADDRESS_2                                                           = 0x049A,
        PATCH__ADDRESS_2_HI                                                        = 0x049A,
        PATCH__ADDRESS_2_LO                                                        = 0x049B,
        PATCH__ADDRESS_3                                                           = 0x049C,
        PATCH__ADDRESS_3_HI                                                        = 0x049C,
        PATCH__ADDRESS_3_LO                                                        = 0x049D,
        PATCH__ADDRESS_4                                                           = 0x049E,
        PATCH__ADDRESS_4_HI                                                        = 0x049E,
        PATCH__ADDRESS_4_LO                                                        = 0x049F,
        PATCH__ADDRESS_5                                                           = 0x04A0,
        PATCH__ADDRESS_5_HI                                                        = 0x04A0,
        PATCH__ADDRESS_5_LO                                                        = 0x04A1,
        PATCH__ADDRESS_6                                                           = 0x04A2,
        PATCH__ADDRESS_6_HI                                                        = 0x04A2,
        PATCH__ADDRESS_6_LO                                                        = 0x04A3,
        PATCH__ADDRESS_7                                                           = 0x04A4,
        PATCH__ADDRESS_7_HI                                                        = 0x04A4,
        PATCH__ADDRESS_7_LO                                                        = 0x04A5,
        PATCH__ADDRESS_8                                                           = 0x04A6,
        PATCH__ADDRESS_8_HI                                                        = 0x04A6,
        PATCH__ADDRESS_8_LO                                                        = 0x04A7,
        PATCH__ADDRESS_9                                                           = 0x04A8,
        PATCH__ADDRESS_9_HI                                                        = 0x04A8,
        PATCH__ADDRESS_9_LO                                                        = 0x04A9,
        PATCH__ADDRESS_10                                                          = 0x04AA,
        PATCH__ADDRESS_10_HI                                                       = 0x04AA,
        PATCH__ADDRESS_10_LO                                                       = 0x04AB,
        PATCH__ADDRESS_11                                                          = 0x04AC,
        PATCH__ADDRESS_11_HI                                                       = 0x04AC,
        PATCH__ADDRESS_11_LO                                                       = 0x04AD,
        PATCH__ADDRESS_12                                                          = 0x04AE,
        PATCH__ADDRESS_12_HI                                                       = 0x04AE,
        PATCH__ADDRESS_12_LO                                                       = 0x04AF,
        PATCH__ADDRESS_13                                                          = 0x04B0,
        PATCH__ADDRESS_13_HI                                                       = 0x04B0,
        PATCH__ADDRESS_13_LO                                                       = 0x04B1,
        PATCH__ADDRESS_14                                                          = 0x04B2,
        PATCH__ADDRESS_14_HI                                                       = 0x04B2,
        PATCH__ADDRESS_14_LO                                                       = 0x04B3,
        PATCH__ADDRESS_15                                                          = 0x04B4,
        PATCH__ADDRESS_15_HI                                                       = 0x04B4,
        PATCH__ADDRESS_15_LO                                                       = 0x04B5,
        
        // SPI, clock, and GPIO configuration registers (0x04C0-0x04FF)
        SPI_ASYNC_MUX__CTRL                                                        = 0x04C0,
        CLK__CONFIG                                                                = 0x04C4,
        GPIO_LV_MUX__CTRL                                                          = 0x04CC,
        GPIO_LV_PAD__CTRL                                                          = 0x04CD,
        PAD_I2C_LV__CONFIG                                                         = 0x04D0,
        PAD_STARTUP_MODE__VALUE_RO_GO1                                             = 0x04D4,
        HOST_IF__STATUS_GO1                                                        = 0x04D5,
        MCU_CLK_GATING__CTRL                                                       = 0x04D8,
        TEST__BIST_ROM_CTRL                                                        = 0x04E0,
        TEST__BIST_ROM_RESULT                                                      = 0x04E1,
        TEST__BIST_ROM_MCU_SIG                                                     = 0x04E2,
        TEST__BIST_ROM_MCU_SIG_HI                                                  = 0x04E2,
        TEST__BIST_ROM_MCU_SIG_LO                                                  = 0x04E3,
        TEST__BIST_RAM_CTRL                                                        = 0x04E4,
        TEST__BIST_RAM_RESULT                                                      = 0x04E5,
        TEST__TMC                                                                  = 0x04E8,
        TEST__PLL_BIST_MIN_THRESHOLD                                               = 0x04F0,
        TEST__PLL_BIST_MIN_THRESHOLD_HI                                            = 0x04F0,
        TEST__PLL_BIST_MIN_THRESHOLD_LO                                            = 0x04F1,
        TEST__PLL_BIST_MAX_THRESHOLD                                               = 0x04F2,
        TEST__PLL_BIST_MAX_THRESHOLD_HI                                            = 0x04F2,
        TEST__PLL_BIST_MAX_THRESHOLD_LO                                            = 0x04F3,
        TEST__PLL_BIST_COUNT_OUT                                                   = 0x04F4,
        TEST__PLL_BIST_COUNT_OUT_HI                                                = 0x04F4,
        TEST__PLL_BIST_COUNT_OUT_LO                                                = 0x04F5,
        TEST__PLL_BIST_GONOGO                                                      = 0x04F6,
        TEST__PLL_BIST_CTRL                                                        = 0x04F7,
        
        // Ranging core hardware control registers (0x0680-0x07D4)
        RANGING_CORE__DEVICE_ID                                                    = 0x0680,
        RANGING_CORE__REVISION_ID                                                  = 0x0681,
        RANGING_CORE__CLK_CTRL1                                                    = 0x0683,
        RANGING_CORE__CLK_CTRL2                                                    = 0x0684,
        RANGING_CORE__WOI_1                                                        = 0x0685,
        RANGING_CORE__WOI_REF_1                                                    = 0x0686,
        RANGING_CORE__START_RANGING                                                = 0x0687,
        RANGING_CORE__LOW_LIMIT_1                                                  = 0x0690,
        RANGING_CORE__HIGH_LIMIT_1                                                 = 0x0691,
        RANGING_CORE__LOW_LIMIT_REF_1                                              = 0x0692,
        RANGING_CORE__HIGH_LIMIT_REF_1                                             = 0x0693,
        RANGING_CORE__QUANTIFIER_1_MSB                                             = 0x0694,
        RANGING_CORE__QUANTIFIER_1_LSB                                             = 0x0695,
        RANGING_CORE__QUANTIFIER_REF_1_MSB                                         = 0x0696,
        RANGING_CORE__QUANTIFIER_REF_1_LSB                                         = 0x0697,
        RANGING_CORE__AMBIENT_OFFSET_1_MSB                                         = 0x0698,
        RANGING_CORE__AMBIENT_OFFSET_1_LSB                                         = 0x0699,
        RANGING_CORE__AMBIENT_OFFSET_REF_1_MSB                                     = 0x069A,
        RANGING_CORE__AMBIENT_OFFSET_REF_1_LSB                                     = 0x069B,
        RANGING_CORE__FILTER_STRENGTH_1                                            = 0x069C,
        RANGING_CORE__FILTER_STRENGTH_REF_1                                        = 0x069D,
        RANGING_CORE__SIGNAL_EVENT_LIMIT_1_MSB                                     = 0x069E,
        RANGING_CORE__SIGNAL_EVENT_LIMIT_1_LSB                                     = 0x069F,
        RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_MSB                                 = 0x06A0,
        RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_LSB                                 = 0x06A1,
        RANGING_CORE__TIMEOUT_OVERALL_PERIODS_MSB                                  = 0x06A4,
        RANGING_CORE__TIMEOUT_OVERALL_PERIODS_LSB                                  = 0x06A5,
        RANGING_CORE__INVERT_HW                                                    = 0x06A6,
        RANGING_CORE__FORCE_HW                                                     = 0x06A7,
        RANGING_CORE__STATIC_HW_VALUE                                              = 0x06A8,
        RANGING_CORE__FORCE_CONTINUOUS_AMBIENT                                     = 0x06A9,
        RANGING_CORE__TEST_PHASE_SELECT_TO_FILTER                                  = 0x06AA,
        RANGING_CORE__TEST_PHASE_SELECT_TO_TIMING_GEN                              = 0x06AB,
        RANGING_CORE__INITIAL_PHASE_VALUE_1                                        = 0x06AC,
        RANGING_CORE__INITIAL_PHASE_VALUE_REF_1                                    = 0x06AD,
        RANGING_CORE__FORCE_UP_IN                                                  = 0x06AE,
        RANGING_CORE__FORCE_DN_IN                                                  = 0x06AF,
        RANGING_CORE__STATIC_UP_VALUE_1                                            = 0x06B0,
        RANGING_CORE__STATIC_UP_VALUE_REF_1                                        = 0x06B1,
        RANGING_CORE__STATIC_DN_VALUE_1                                            = 0x06B2,
        RANGING_CORE__STATIC_DN_VALUE_REF_1                                        = 0x06B3,
        RANGING_CORE__MONITOR_UP_DN                                                = 0x06B4,
        RANGING_CORE__INVERT_UP_DN                                                 = 0x06B5,
        RANGING_CORE__CPUMP_1                                                      = 0x06B6,
        RANGING_CORE__CPUMP_2                                                      = 0x06B7,
        RANGING_CORE__CPUMP_3                                                      = 0x06B8,
        RANGING_CORE__OSC_1                                                        = 0x06B9,
        RANGING_CORE__PLL_1                                                        = 0x06BB,
        RANGING_CORE__PLL_2                                                        = 0x06BC,
        RANGING_CORE__REFERENCE_1                                                  = 0x06BD,
        RANGING_CORE__REFERENCE_3                                                  = 0x06BF,
        RANGING_CORE__REFERENCE_4                                                  = 0x06C0,
        RANGING_CORE__REFERENCE_5                                                  = 0x06C1,
        RANGING_CORE__REGAVDD1V2                                                   = 0x06C3,
        RANGING_CORE__CALIB_1                                                      = 0x06C4,
        RANGING_CORE__CALIB_2                                                      = 0x06C5,
        RANGING_CORE__CALIB_3                                                      = 0x06C6,
        RANGING_CORE__TST_MUX_SEL1                                                 = 0x06C9,
        RANGING_CORE__TST_MUX_SEL2                                                 = 0x06CA,
        RANGING_CORE__TST_MUX                                                      = 0x06CB,
        RANGING_CORE__GPIO_OUT_TESTMUX                                             = 0x06CC,
        RANGING_CORE__CUSTOM_FE                                                    = 0x06CD,
        RANGING_CORE__CUSTOM_FE_2                                                  = 0x06CE,
        RANGING_CORE__SPAD_READOUT                                                 = 0x06CF,
        RANGING_CORE__SPAD_READOUT_1                                               = 0x06D0,
        RANGING_CORE__SPAD_READOUT_2                                               = 0x06D1,
        RANGING_CORE__SPAD_PS                                                      = 0x06D2,
        RANGING_CORE__LASER_SAFETY_2                                               = 0x06D4,
        
        // NVM (Non-Volatile Memory) control registers (0x0780-0x07A7)
        RANGING_CORE__NVM_CTRL__MODE                                               = 0x0780,
        RANGING_CORE__NVM_CTRL__PDN                                                = 0x0781,
        RANGING_CORE__NVM_CTRL__PROGN                                              = 0x0782,
        RANGING_CORE__NVM_CTRL__READN                                              = 0x0783,
        RANGING_CORE__NVM_CTRL__PULSE_WIDTH_MSB                                    = 0x0784,
        RANGING_CORE__NVM_CTRL__PULSE_WIDTH_LSB                                    = 0x0785,
        RANGING_CORE__NVM_CTRL__HV_RISE_MSB                                        = 0x0786,
        RANGING_CORE__NVM_CTRL__HV_RISE_LSB                                        = 0x0787,
        RANGING_CORE__NVM_CTRL__HV_FALL_MSB                                        = 0x0788,
        RANGING_CORE__NVM_CTRL__HV_FALL_LSB                                        = 0x0789,
        RANGING_CORE__NVM_CTRL__TST                                                = 0x078A,
        RANGING_CORE__NVM_CTRL__TESTREAD                                           = 0x078B,
        RANGING_CORE__NVM_CTRL__DATAIN_MMM                                         = 0x078C,
        RANGING_CORE__NVM_CTRL__DATAIN_LMM                                         = 0x078D,
        RANGING_CORE__NVM_CTRL__DATAIN_LLM                                         = 0x078E,
        RANGING_CORE__NVM_CTRL__DATAIN_LLL                                         = 0x078F,
        RANGING_CORE__NVM_CTRL__DATAOUT_MMM                                        = 0x0790,
        RANGING_CORE__NVM_CTRL__DATAOUT_LMM                                        = 0x0791,
        RANGING_CORE__NVM_CTRL__DATAOUT_LLM                                        = 0x0792,
        RANGING_CORE__NVM_CTRL__DATAOUT_LLL                                        = 0x0793,
        RANGING_CORE__NVM_CTRL__ADDR                                               = 0x0794,
        RANGING_CORE__NVM_CTRL__DATAOUT_ECC                                        = 0x0795,
        
        // Return SPAD enable registers (0x0796-0x0A32)
        RANGING_CORE__RET_SPAD_EN_0                                                = 0x0796,
        RANGING_CORE__RET_SPAD_EN_1                                                = 0x0797,
        RANGING_CORE__RET_SPAD_EN_2                                                = 0x0798,
        RANGING_CORE__RET_SPAD_EN_3                                                = 0x0799,
        RANGING_CORE__RET_SPAD_EN_4                                                = 0x079A,
        RANGING_CORE__RET_SPAD_EN_5                                                = 0x079B,
        RANGING_CORE__RET_SPAD_EN_6                                                = 0x079C,
        RANGING_CORE__RET_SPAD_EN_7                                                = 0x079D,
        RANGING_CORE__RET_SPAD_EN_8                                                = 0x079E,
        RANGING_CORE__RET_SPAD_EN_9                                                = 0x079F,
        RANGING_CORE__RET_SPAD_EN_10                                               = 0x07A0,
        RANGING_CORE__RET_SPAD_EN_11                                               = 0x07A1,
        RANGING_CORE__RET_SPAD_EN_12                                               = 0x07A2,
        RANGING_CORE__RET_SPAD_EN_13                                               = 0x07A3,
        RANGING_CORE__RET_SPAD_EN_14                                               = 0x07A4,
        RANGING_CORE__RET_SPAD_EN_15                                               = 0x07A5,
        RANGING_CORE__RET_SPAD_EN_16                                               = 0x07A6,
        RANGING_CORE__RET_SPAD_EN_17                                               = 0x07A7,
        RANGING_CORE__SPAD_SHIFT_EN                                                = 0x07BA,
        RANGING_CORE__SPAD_DISABLE_CTRL                                            = 0x07BB,
        RANGING_CORE__SPAD_EN_SHIFT_OUT_DEBUG                                      = 0x07BC,
        RANGING_CORE__SPI_MODE                                                     = 0x07BD,
        RANGING_CORE__GPIO_DIR                                                     = 0x07BE,
        RANGING_CORE__VCSEL_PERIOD                                                 = 0x0880,
        RANGING_CORE__VCSEL_START                                                  = 0x0881,
        RANGING_CORE__VCSEL_STOP                                                   = 0x0882,
        RANGING_CORE__VCSEL_1                                                      = 0x0885,
        RANGING_CORE__VCSEL_STATUS                                                 = 0x088D,
        RANGING_CORE__STATUS                                                       = 0x0980,
        RANGING_CORE__LASER_CONTINUITY_STATE                                       = 0x0981,
        RANGING_CORE__RANGE_1_MMM                                                  = 0x0982,
        RANGING_CORE__RANGE_1_LMM                                                  = 0x0983,
        RANGING_CORE__RANGE_1_LLM                                                  = 0x0984,
        RANGING_CORE__RANGE_1_LLL                                                  = 0x0985,
        RANGING_CORE__RANGE_REF_1_MMM                                              = 0x0986,
        RANGING_CORE__RANGE_REF_1_LMM                                              = 0x0987,
        RANGING_CORE__RANGE_REF_1_LLM                                              = 0x0988,
        RANGING_CORE__RANGE_REF_1_LLL                                              = 0x0989,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_MMM                                  = 0x098A,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LMM                                  = 0x098B,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLM                                  = 0x098C,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLL                                  = 0x098D,
        RANGING_CORE__RANGING_TOTAL_EVENTS_1_MMM                                   = 0x098E,
        RANGING_CORE__RANGING_TOTAL_EVENTS_1_LMM                                   = 0x098F,
        RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLM                                   = 0x0990,
        RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLL                                   = 0x0991,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_MMM                                    = 0x0992,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LMM                                    = 0x0993,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLM                                    = 0x0994,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLL                                    = 0x0995,
        RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_MM                                   = 0x0996,
        RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LM                                   = 0x0997,
        RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LL                                   = 0x0998,
        RANGING_CORE__AMBIENT_MISMATCH_MM                                          = 0x0999,
        RANGING_CORE__AMBIENT_MISMATCH_LM                                          = 0x099A,
        RANGING_CORE__AMBIENT_MISMATCH_LL                                          = 0x099B,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_MMM                              = 0x099C,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LMM                              = 0x099D,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLM                              = 0x099E,
        RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLL                              = 0x099F,
        RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_MMM                               = 0x09A0,
        RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LMM                               = 0x09A1,
        RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLM                               = 0x09A2,
        RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLL                               = 0x09A3,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_MMM                                = 0x09A4,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LMM                                = 0x09A5,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLM                                = 0x09A6,
        RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLL                                = 0x09A7,
        RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_MM                               = 0x09A8,
        RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LM                               = 0x09A9,
        RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LL                               = 0x09AA,
        RANGING_CORE__AMBIENT_MISMATCH_REF_MM                                      = 0x09AB,
        RANGING_CORE__AMBIENT_MISMATCH_REF_LM                                      = 0x09AC,
        RANGING_CORE__AMBIENT_MISMATCH_REF_LL                                      = 0x09AD,
        RANGING_CORE__GPIO_CONFIG__A0                                              = 0x0A00,
        RANGING_CORE__RESET_CONTROL__A0                                            = 0x0A01,
        RANGING_CORE__INTR_MANAGER__A0                                             = 0x0A02,
        RANGING_CORE__POWER_FSM_TIME_OSC__A0                                       = 0x0A06,
        RANGING_CORE__VCSEL_ATEST__A0                                              = 0x0A07,
        RANGING_CORE__VCSEL_PERIOD_CLIPPED__A0                                     = 0x0A08,
        RANGING_CORE__VCSEL_STOP_CLIPPED__A0                                       = 0x0A09,
        RANGING_CORE__CALIB_2__A0                                                  = 0x0A0A,
        RANGING_CORE__STOP_CONDITION__A0                                           = 0x0A0B,
        RANGING_CORE__STATUS_RESET__A0                                             = 0x0A0C,
        RANGING_CORE__READOUT_CFG__A0                                              = 0x0A0D,
        RANGING_CORE__WINDOW_SETTING__A0                                           = 0x0A0E,
        RANGING_CORE__VCSEL_DELAY__A0                                              = 0x0A1A,
        RANGING_CORE__REFERENCE_2__A0                                              = 0x0A1B,
        RANGING_CORE__REGAVDD1V2__A0                                               = 0x0A1D,
        RANGING_CORE__TST_MUX__A0                                                  = 0x0A1F,
        RANGING_CORE__CUSTOM_FE_2__A0                                              = 0x0A20,
        RANGING_CORE__SPAD_READOUT__A0                                             = 0x0A21,
        RANGING_CORE__CPUMP_1__A0                                                  = 0x0A22,
        RANGING_CORE__SPARE_REGISTER__A0                                           = 0x0A23,
        RANGING_CORE__VCSEL_CONT_STAGE5_BYPASS__A0                                 = 0x0A24,
        RANGING_CORE__RET_SPAD_EN_18                                               = 0x0A25,
        RANGING_CORE__RET_SPAD_EN_19                                               = 0x0A26,
        RANGING_CORE__RET_SPAD_EN_20                                               = 0x0A27,
        RANGING_CORE__RET_SPAD_EN_21                                               = 0x0A28,
        RANGING_CORE__RET_SPAD_EN_22                                               = 0x0A29,
        RANGING_CORE__RET_SPAD_EN_23                                               = 0x0A2A,
        RANGING_CORE__RET_SPAD_EN_24                                               = 0x0A2B,
        RANGING_CORE__RET_SPAD_EN_25                                               = 0x0A2C,
        RANGING_CORE__RET_SPAD_EN_26                                               = 0x0A2D,
        RANGING_CORE__RET_SPAD_EN_27                                               = 0x0A2E,
        RANGING_CORE__RET_SPAD_EN_28                                               = 0x0A2F,
        RANGING_CORE__RET_SPAD_EN_29                                               = 0x0A30,
        RANGING_CORE__RET_SPAD_EN_30                                               = 0x0A31,
        RANGING_CORE__RET_SPAD_EN_31                                               = 0x0A32,
        
        // Advanced ranging core configuration registers (0x0A33-0x0A41)
        RANGING_CORE__REF_SPAD_EN_0__EWOK                                          = 0x0A33,
        RANGING_CORE__REF_SPAD_EN_1__EWOK                                          = 0x0A34,
        RANGING_CORE__REF_SPAD_EN_2__EWOK                                          = 0x0A35,
        RANGING_CORE__REF_SPAD_EN_3__EWOK                                          = 0x0A36,
        RANGING_CORE__REF_SPAD_EN_4__EWOK                                          = 0x0A37,
        RANGING_CORE__REF_SPAD_EN_5__EWOK                                          = 0x0A38,
        RANGING_CORE__REF_EN_START_SELECT                                          = 0x0A39,
        RANGING_CORE__REGDVDD1V2_ATEST__EWOK                                       = 0x0A41,
        
        // Reset control and base address registers (0x0B00-0x0E00)
        SOFT_RESET_GO1                                                             = 0x0B00,
        PRIVATE__PATCH_BASE_ADDR_RSLV                                              = 0x0E00,
        
        // Previous shadow result registers - previous measurement data (0x0ED0-0x0F1C)
        PREV_SHADOW_RESULT__INTERRUPT_STATUS                                       = 0x0ED0,
        PREV_SHADOW_RESULT__RANGE_STATUS                                           = 0x0ED1,
        PREV_SHADOW_RESULT__REPORT_STATUS                                          = 0x0ED2,
        PREV_SHADOW_RESULT__STREAM_COUNT                                           = 0x0ED3,
        PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                         = 0x0ED4,
        PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI                      = 0x0ED4,
        PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO                      = 0x0ED5,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0                        = 0x0ED6,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI                     = 0x0ED6,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO                     = 0x0ED7,
        PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                            = 0x0ED8,
        PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI                         = 0x0ED8,
        PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO                         = 0x0ED9,
        PREV_SHADOW_RESULT__SIGMA_SD0                                              = 0x0EDA,
        PREV_SHADOW_RESULT__SIGMA_SD0_HI                                           = 0x0EDA,
        PREV_SHADOW_RESULT__SIGMA_SD0_LO                                           = 0x0EDB,
        PREV_SHADOW_RESULT__PHASE_SD0                                              = 0x0EDC,
        PREV_SHADOW_RESULT__PHASE_SD0_HI                                           = 0x0EDC,
        PREV_SHADOW_RESULT__PHASE_SD0_LO                                           = 0x0EDD,
        PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                 = 0x0EDE,
        PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI              = 0x0EDE,
        PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO              = 0x0EDF,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0    = 0x0EE0,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI = 0x0EE0,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO = 0x0EE1,
        PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0                    = 0x0EE2,
        PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                 = 0x0EE2,
        PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                 = 0x0EE3,
        PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0                    = 0x0EE4,
        PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                 = 0x0EE4,
        PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                 = 0x0EE5,
        PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0                         = 0x0EE6,
        PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI                      = 0x0EE6,
        PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO                      = 0x0EE7,
        PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1                         = 0x0EE8,
        PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI                      = 0x0EE8,
        PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO                      = 0x0EE9,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1                        = 0x0EEA,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI                     = 0x0EEA,
        PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO                     = 0x0EEB,
        PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1                            = 0x0EEC,
        PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI                         = 0x0EEC,
        PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO                         = 0x0EED,
        PREV_SHADOW_RESULT__SIGMA_SD1                                              = 0x0EEE,
        PREV_SHADOW_RESULT__SIGMA_SD1_HI                                           = 0x0EEE,
        PREV_SHADOW_RESULT__SIGMA_SD1_LO                                           = 0x0EEF,
        PREV_SHADOW_RESULT__PHASE_SD1                                              = 0x0EF0,
        PREV_SHADOW_RESULT__PHASE_SD1_HI                                           = 0x0EF0,
        PREV_SHADOW_RESULT__PHASE_SD1_LO                                           = 0x0EF1,
        PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1                 = 0x0EF2,
        PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI              = 0x0EF2,
        PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO              = 0x0EF3,
        PREV_SHADOW_RESULT__SPARE_0_SD1                                            = 0x0EF4,
        PREV_SHADOW_RESULT__SPARE_0_SD1_HI                                         = 0x0EF4,
        PREV_SHADOW_RESULT__SPARE_0_SD1_LO                                         = 0x0EF5,
        PREV_SHADOW_RESULT__SPARE_1_SD1                                            = 0x0EF6,
        PREV_SHADOW_RESULT__SPARE_1_SD1_HI                                         = 0x0EF6,
        PREV_SHADOW_RESULT__SPARE_1_SD1_LO                                         = 0x0EF7,
        PREV_SHADOW_RESULT__SPARE_2_SD1                                            = 0x0EF8,
        PREV_SHADOW_RESULT__SPARE_2_SD1_HI                                         = 0x0EF8,
        PREV_SHADOW_RESULT__SPARE_2_SD1_LO                                         = 0x0EF9,
        PREV_SHADOW_RESULT__SPARE_3_SD1                                            = 0x0EFA,
        PREV_SHADOW_RESULT__SPARE_3_SD1_HI                                         = 0x0EFA,
        PREV_SHADOW_RESULT__SPARE_3_SD1_LO                                         = 0x0EFB,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0                         = 0x0EFC,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3                       = 0x0EFC,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2                       = 0x0EFD,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1                       = 0x0EFE,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0                       = 0x0EFF,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0                          = 0x0F00,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3                        = 0x0F00,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2                        = 0x0F01,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1                        = 0x0F02,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0                        = 0x0F03,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0                           = 0x0F04,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3                         = 0x0F04,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2                         = 0x0F05,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1                         = 0x0F06,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0                         = 0x0F07,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0                         = 0x0F08,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3                       = 0x0F08,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2                       = 0x0F09,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1                       = 0x0F0A,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0                       = 0x0F0B,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1                         = 0x0F0C,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3                       = 0x0F0C,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2                       = 0x0F0D,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1                       = 0x0F0E,
        PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0                       = 0x0F0F,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1                          = 0x0F10,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3                        = 0x0F10,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2                        = 0x0F11,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1                        = 0x0F12,
        PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0                        = 0x0F13,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1                           = 0x0F14,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3                         = 0x0F14,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2                         = 0x0F15,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1                         = 0x0F16,
        PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0                         = 0x0F17,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1                         = 0x0F18,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3                       = 0x0F18,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2                       = 0x0F19,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1                       = 0x0F1A,
        PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0                       = 0x0F1B,
        PREV_SHADOW_RESULT_CORE__SPARE_0                                           = 0x0F1C,
        
        // Debug and GPH (Grouped Parameter Hold) registers (0x0F20-0x0F47)
        RESULT__DEBUG_STATUS                                                       = 0x0F20,
        RESULT__DEBUG_STAGE                                                        = 0x0F21,
        GPH__SYSTEM__THRESH_RATE_HIGH                                              = 0x0F24,
        GPH__SYSTEM__THRESH_RATE_HIGH_HI                                           = 0x0F24,
        GPH__SYSTEM__THRESH_RATE_HIGH_LO                                           = 0x0F25,
        GPH__SYSTEM__THRESH_RATE_LOW                                               = 0x0F26,
        GPH__SYSTEM__THRESH_RATE_LOW_HI                                            = 0x0F26,
        GPH__SYSTEM__THRESH_RATE_LOW_LO                                            = 0x0F27,
        GPH__SYSTEM__INTERRUPT_CONFIG_GPIO                                         = 0x0F28,
        GPH__DSS_CONFIG__ROI_MODE_CONTROL                                          = 0x0F2F,
        GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT                             = 0x0F30,
        GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI                          = 0x0F30,
        GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO                          = 0x0F31,
        GPH__DSS_CONFIG__MANUAL_BLOCK_SELECT                                       = 0x0F32,
        GPH__DSS_CONFIG__MAX_SPADS_LIMIT                                           = 0x0F33,
        GPH__DSS_CONFIG__MIN_SPADS_LIMIT                                           = 0x0F34,
        GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI                                        = 0x0F36,
        GPH__MM_CONFIG__TIMEOUT_MACROP_A_LO                                        = 0x0F37,
        GPH__MM_CONFIG__TIMEOUT_MACROP_B_HI                                        = 0x0F38,
        GPH__MM_CONFIG__TIMEOUT_MACROP_B_LO                                        = 0x0F39,
        GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_HI                                     = 0x0F3A,
        GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_LO                                     = 0x0F3B,
        GPH__RANGE_CONFIG__VCSEL_PERIOD_A                                          = 0x0F3C,
        GPH__RANGE_CONFIG__VCSEL_PERIOD_B                                          = 0x0F3D,
        GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_HI                                     = 0x0F3E,
        GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_LO                                     = 0x0F3F,
        GPH__RANGE_CONFIG__SIGMA_THRESH                                            = 0x0F40,
        GPH__RANGE_CONFIG__SIGMA_THRESH_HI                                         = 0x0F40,
        GPH__RANGE_CONFIG__SIGMA_THRESH_LO                                         = 0x0F41,
        GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                           = 0x0F42,
        GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI                        = 0x0F42,
        GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO                        = 0x0F43,
        GPH__RANGE_CONFIG__VALID_PHASE_LOW                                         = 0x0F44,
        GPH__RANGE_CONFIG__VALID_PHASE_HIGH                                        = 0x0F45,
        FIRMWARE__INTERNAL_STREAM_COUNT_DIV                                        = 0x0F46,
        FIRMWARE__INTERNAL_STREAM_COUNTER_VAL                                      = 0x0F47,
        
        // DSS (Dynamic SPAD Selection) calculation registers (0x0F54-0x0F83)
        DSS_CALC__ROI_CTRL                                                         = 0x0F54,
        DSS_CALC__SPARE_1                                                          = 0x0F55,
        DSS_CALC__SPARE_2                                                          = 0x0F56,
        DSS_CALC__SPARE_3                                                          = 0x0F57,
        DSS_CALC__SPARE_4                                                          = 0x0F58,
        DSS_CALC__SPARE_5                                                          = 0x0F59,
        DSS_CALC__SPARE_6                                                          = 0x0F5A,
        DSS_CALC__SPARE_7                                                          = 0x0F5B,
        DSS_CALC__USER_ROI_SPAD_EN_0                                               = 0x0F5C,
        DSS_CALC__USER_ROI_SPAD_EN_1                                               = 0x0F5D,
        DSS_CALC__USER_ROI_SPAD_EN_2                                               = 0x0F5E,
        DSS_CALC__USER_ROI_SPAD_EN_3                                               = 0x0F5F,
        DSS_CALC__USER_ROI_SPAD_EN_4                                               = 0x0F60,
        DSS_CALC__USER_ROI_SPAD_EN_5                                               = 0x0F61,
        DSS_CALC__USER_ROI_SPAD_EN_6                                               = 0x0F62,
        DSS_CALC__USER_ROI_SPAD_EN_7                                               = 0x0F63,
        DSS_CALC__USER_ROI_SPAD_EN_8                                               = 0x0F64,
        DSS_CALC__USER_ROI_SPAD_EN_9                                               = 0x0F65,
        DSS_CALC__USER_ROI_SPAD_EN_10                                              = 0x0F66,
        DSS_CALC__USER_ROI_SPAD_EN_11                                              = 0x0F67,
        DSS_CALC__USER_ROI_SPAD_EN_12                                              = 0x0F68,
        DSS_CALC__USER_ROI_SPAD_EN_13                                              = 0x0F69,
        DSS_CALC__USER_ROI_SPAD_EN_14                                              = 0x0F6A,
        DSS_CALC__USER_ROI_SPAD_EN_15                                              = 0x0F6B,
        DSS_CALC__USER_ROI_SPAD_EN_16                                              = 0x0F6C,
        DSS_CALC__USER_ROI_SPAD_EN_17                                              = 0x0F6D,
        DSS_CALC__USER_ROI_SPAD_EN_18                                              = 0x0F6E,
        DSS_CALC__USER_ROI_SPAD_EN_19                                              = 0x0F6F,
        DSS_CALC__USER_ROI_SPAD_EN_20                                              = 0x0F70,
        DSS_CALC__USER_ROI_SPAD_EN_21                                              = 0x0F71,
        DSS_CALC__USER_ROI_SPAD_EN_22                                              = 0x0F72,
        DSS_CALC__USER_ROI_SPAD_EN_23                                              = 0x0F73,
        DSS_CALC__USER_ROI_SPAD_EN_24                                              = 0x0F74,
        DSS_CALC__USER_ROI_SPAD_EN_25                                              = 0x0F75,
        DSS_CALC__USER_ROI_SPAD_EN_26                                              = 0x0F76,
        DSS_CALC__USER_ROI_SPAD_EN_27                                              = 0x0F77,
        DSS_CALC__USER_ROI_SPAD_EN_28                                              = 0x0F78,
        DSS_CALC__USER_ROI_SPAD_EN_29                                              = 0x0F79,
        DSS_CALC__USER_ROI_SPAD_EN_30                                              = 0x0F7A,
        DSS_CALC__USER_ROI_SPAD_EN_31                                              = 0x0F7B,
        DSS_CALC__USER_ROI_0                                                       = 0x0F7C,
        DSS_CALC__USER_ROI_1                                                       = 0x0F7D,
        DSS_CALC__MODE_ROI_0                                                       = 0x0F7E,
        DSS_CALC__MODE_ROI_1                                                       = 0x0F7F,
        
        // Sigma estimator, VHV, and phase calibration result registers (0x0F80-0x0F8F)
        SIGMA_ESTIMATOR_CALC__SPARE_0                                              = 0x0F80,
        VHV_RESULT__PEAK_SIGNAL_RATE_MCPS                                          = 0x0F82,
        VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_HI                                       = 0x0F82,
        VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_LO                                       = 0x0F83,
        VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF                                        = 0x0F84,
        VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_3                                      = 0x0F84,
        VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_2                                      = 0x0F85,
        VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_1                                      = 0x0F86,
        VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_0                                      = 0x0F87,
        PHASECAL_RESULT__PHASE_OUTPUT_REF                                          = 0x0F88,
        PHASECAL_RESULT__PHASE_OUTPUT_REF_HI                                       = 0x0F88,
        PHASECAL_RESULT__PHASE_OUTPUT_REF_LO                                       = 0x0F89,
        
        // DSS result registers (0x0F8A-0x0F91)
        DSS_RESULT__TOTAL_RATE_PER_SPAD                                            = 0x0F8A,
        DSS_RESULT__TOTAL_RATE_PER_SPAD_HI                                         = 0x0F8A,
        DSS_RESULT__TOTAL_RATE_PER_SPAD_LO                                         = 0x0F8B,
        DSS_RESULT__ENABLED_BLOCKS                                                 = 0x0F8C,
        DSS_RESULT__NUM_REQUESTED_SPADS                                            = 0x0F8E,
        DSS_RESULT__NUM_REQUESTED_SPADS_HI                                         = 0x0F8E,
        DSS_RESULT__NUM_REQUESTED_SPADS_LO                                         = 0x0F8F,
        
        // MM (Minimum/Maximum) result registers (0x0F92-0x0F97)
        MM_RESULT__INNER_INTERSECTION_RATE                                         = 0x0F92,
        MM_RESULT__INNER_INTERSECTION_RATE_HI                                      = 0x0F92,
        MM_RESULT__INNER_INTERSECTION_RATE_LO                                      = 0x0F93,
        MM_RESULT__OUTER_COMPLEMENT_RATE                                           = 0x0F94,
        MM_RESULT__OUTER_COMPLEMENT_RATE_HI                                        = 0x0F94,
        MM_RESULT__OUTER_COMPLEMENT_RATE_LO                                        = 0x0F95,
        MM_RESULT__TOTAL_OFFSET                                                    = 0x0F96,
        MM_RESULT__TOTAL_OFFSET_HI                                                 = 0x0F96,
        MM_RESULT__TOTAL_OFFSET_LO                                                 = 0x0F97,
        
        // Crosstalk (XTALK) calculation and result registers (0x0F98-0x0FA7)
        XTALK_CALC__XTALK_FOR_ENABLED_SPADS                                        = 0x0F98,
        XTALK_CALC__XTALK_FOR_ENABLED_SPADS_3                                      = 0x0F98,
        XTALK_CALC__XTALK_FOR_ENABLED_SPADS_2                                      = 0x0F99,
        XTALK_CALC__XTALK_FOR_ENABLED_SPADS_1                                      = 0x0F9A,
        XTALK_CALC__XTALK_FOR_ENABLED_SPADS_0                                      = 0x0F9B,
        XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS                                      = 0x0F9C,
        XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_3                                    = 0x0F9C,
        XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_2                                    = 0x0F9D,
        XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_1                                    = 0x0F9E,
        XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_0                                    = 0x0F9F,
        XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS                                  = 0x0FA0,
        XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_3                                = 0x0FA0,
        XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_2                                = 0x0FA1,
        XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_1                                = 0x0FA2,
        XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_0                                = 0x0FA3,
        XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS                                  = 0x0FA4,
        XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_3                                = 0x0FA4,
        XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_2                                = 0x0FA5,
        XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_1                                = 0x0FA6,
        XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_0                                = 0x0FA7,
        
        // Range result and accumulation registers (0x0FA8-0x0FAD)
        RANGE_RESULT__ACCUM_PHASE                                                  = 0x0FA8,
        RANGE_RESULT__ACCUM_PHASE_3                                                = 0x0FA8,
        RANGE_RESULT__ACCUM_PHASE_2                                                = 0x0FA9,
        RANGE_RESULT__ACCUM_PHASE_1                                                = 0x0FAA,
        RANGE_RESULT__ACCUM_PHASE_0                                                = 0x0FAB,
        RANGE_RESULT__OFFSET_CORRECTED_RANGE                                       = 0x0FAC,
        RANGE_RESULT__OFFSET_CORRECTED_RANGE_HI                                    = 0x0FAC,
        RANGE_RESULT__OFFSET_CORRECTED_RANGE_LO                                    = 0x0FAD,
        
        // Shadow phase calibration result and shadow result registers (0x0FAE-0x0FDB)
        // Shadow registers hold previous measurement data while new measurement is acquired
        SHADOW_PHASECAL_RESULT__VCSEL_START                                        = 0x0FAE,
        SHADOW_RESULT__INTERRUPT_STATUS                                            = 0x0FB0,
        SHADOW_RESULT__RANGE_STATUS                                                = 0x0FB1,
        SHADOW_RESULT__REPORT_STATUS                                               = 0x0FB2,
        SHADOW_RESULT__STREAM_COUNT                                                = 0x0FB3,
        SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                              = 0x0FB4,
        SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI                           = 0x0FB4,
        SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO                           = 0x0FB5,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0                             = 0x0FB6,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI                          = 0x0FB6,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO                          = 0x0FB7,
        SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                                 = 0x0FB8,
        SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI                              = 0x0FB8,
        SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO                              = 0x0FB9,
        SHADOW_RESULT__SIGMA_SD0                                                   = 0x0FBA,
        SHADOW_RESULT__SIGMA_SD0_HI                                                = 0x0FBA,
        SHADOW_RESULT__SIGMA_SD0_LO                                                = 0x0FBB,
        SHADOW_RESULT__PHASE_SD0                                                   = 0x0FBC,
        SHADOW_RESULT__PHASE_SD0_HI                                                = 0x0FBC,
        SHADOW_RESULT__PHASE_SD0_LO                                                = 0x0FBD,
        SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                      = 0x0FBE,
        SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI                   = 0x0FBE,
        SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO                   = 0x0FBF,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0         = 0x0FC0,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI      = 0x0FC0,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO      = 0x0FC1,
        SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0                         = 0x0FC2,
        SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                      = 0x0FC2,
        SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                      = 0x0FC3,
        SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0                         = 0x0FC4,
        SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                      = 0x0FC4,
        SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                      = 0x0FC5,
        SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0                              = 0x0FC6,
        SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI                           = 0x0FC6,
        SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO                           = 0x0FC7,
        SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1                              = 0x0FC8,
        SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI                           = 0x0FC8,
        SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO                           = 0x0FC9,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1                             = 0x0FCA,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI                          = 0x0FCA,
        SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO                          = 0x0FCB,
        SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1                                 = 0x0FCC,
        SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI                              = 0x0FCC,
        SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO                              = 0x0FCD,
        SHADOW_RESULT__SIGMA_SD1                                                   = 0x0FCE,
        SHADOW_RESULT__SIGMA_SD1_HI                                                = 0x0FCE,
        SHADOW_RESULT__SIGMA_SD1_LO                                                = 0x0FCF,
        SHADOW_RESULT__PHASE_SD1                                                   = 0x0FD0,
        SHADOW_RESULT__PHASE_SD1_HI                                                = 0x0FD0,
        SHADOW_RESULT__PHASE_SD1_LO                                                = 0x0FD1,
        SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1                      = 0x0FD2,
        SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI                   = 0x0FD2,
        SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO                   = 0x0FD3,
        SHADOW_RESULT__SPARE_0_SD1                                                 = 0x0FD4,
        SHADOW_RESULT__SPARE_0_SD1_HI                                              = 0x0FD4,
        SHADOW_RESULT__SPARE_0_SD1_LO                                              = 0x0FD5,
        SHADOW_RESULT__SPARE_1_SD1                                                 = 0x0FD6,
        SHADOW_RESULT__SPARE_1_SD1_HI                                              = 0x0FD6,
        SHADOW_RESULT__SPARE_1_SD1_LO                                              = 0x0FD7,
        SHADOW_RESULT__SPARE_2_SD1                                                 = 0x0FD8,
        SHADOW_RESULT__SPARE_2_SD1_HI                                              = 0x0FD8,
        SHADOW_RESULT__SPARE_2_SD1_LO                                              = 0x0FD9,
        SHADOW_RESULT__SPARE_3_SD1                                                 = 0x0FDA,
        SHADOW_RESULT__THRESH_INFO                                                 = 0x0FDB,
        
        // Shadow result core registers - detailed event counters (0x0FDC-0x0FFC)
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0                              = 0x0FDC,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3                            = 0x0FDC,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2                            = 0x0FDD,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1                            = 0x0FDE,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0                            = 0x0FDF,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0                               = 0x0FE0,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3                             = 0x0FE0,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2                             = 0x0FE1,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1                             = 0x0FE2,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0                             = 0x0FE3,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0                                = 0x0FE4,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3                              = 0x0FE4,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2                              = 0x0FE5,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1                              = 0x0FE6,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0                              = 0x0FE7,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0                              = 0x0FE8,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3                            = 0x0FE8,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2                            = 0x0FE9,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1                            = 0x0FEA,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0                            = 0x0FEB,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1                              = 0x0FEC,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3                            = 0x0FEC,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2                            = 0x0FED,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1                            = 0x0FEE,
        SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0                            = 0x0FEF,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1                               = 0x0FF0,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3                             = 0x0FF0,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2                             = 0x0FF1,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1                             = 0x0FF2,
        SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0                             = 0x0FF3,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1                                = 0x0FF4,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3                              = 0x0FF4,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2                              = 0x0FF5,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1                              = 0x0FF6,
        SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0                              = 0x0FF7,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1                              = 0x0FF8,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3                            = 0x0FF8,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2                            = 0x0FF9,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1                            = 0x0FFA,
        SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0                            = 0x0FFB,
        SHADOW_RESULT_CORE__SPARE_0                                                = 0x0FFC,
        
        // Shadow phase calibration result - final registers (0x0FFE-0x0FFF)
        SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_HI                                 = 0x0FFE,
        SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_LO                                 = 0x0FFF,
    };

    /**
     * @brief Constructor for VL53L1X rangefinder backend
     * 
     * @param[in,out] _state Reference to rangefinder state structure shared with frontend
     * @param[in]     _params Reference to rangefinder parameters (address, orientation, etc.)
     * @param[in]     dev I2C device handle for communication with the sensor
     * 
     * @note Constructor transfers ownership of the I2C device to this backend instance
     */
    AP_RangeFinder_VL53L1X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    /**
     * @brief Initialize the VL53L1X sensor with specified distance mode
     * 
     * @details Performs complete sensor initialization sequence:
     *          - Verifies sensor ID
     *          - Soft resets the sensor
     *          - Loads default configuration
     *          - Sets specified distance mode (Short/Medium/Long)
     *          - Configures timing budget for measurements
     *          - Starts continuous ranging operation
     *          - Initializes calibration variables
     * 
     * @param[in] mode Distance mode to configure (Short, Medium, or Long)
     * 
     * @return true if initialization successful, false on any error
     * 
     * @note Marked with __INITFUNC__ to place in special init memory section on some platforms
     * @warning Must be called before any ranging operations can be performed
     */
    __INITFUNC__ bool init(DistanceMode mode);
    
    /**
     * @brief Periodic timer callback for reading sensor data
     * 
     * @details Called by scheduler at regular intervals to:
     *          - Check if new measurement data is available
     *          - Read distance measurement from sensor
     *          - Perform running average for calibration
     *          - Update rangefinder state with new reading
     *          - Handle measurement errors and sensor status
     * 
     * @note Called from I2C bus thread context, must be non-blocking
     * @see AP_HAL::Device::register_periodic_callback()
     */
    void timer();

    /**
     * @brief Verify sensor identity by reading device ID registers
     * 
     * @details Reads the IDENTIFICATION__MODEL_ID and IDENTIFICATION__MODULE_TYPE
     *          registers to confirm this is a genuine VL53L1X sensor.
     *          
     *          Expected values:
     *          - Model ID: 0xEA
     *          - Module Type: 0xCC
     * 
     * @return true if sensor ID matches expected VL53L1X values, false otherwise
     * 
     * @note Should be called early in initialization to validate hardware
     */
    bool check_id(void);

    /**
     * @brief Retrieve a single distance measurement from the sensor
     * 
     * @details Reads the ranging result registers and extracts the measured distance.
     *          The measurement is read from RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0
     *          registers, which provide crosstalk-compensated distance values.
     *          
     *          Also checks RESULT__RANGE_STATUS for measurement quality and
     *          potential error conditions.
     * 
     * @param[out] reading_cm Distance measurement in centimeters
     * 
     * @return true if valid reading obtained, false on I2C error or invalid measurement
     * 
     * @note Measurement range: 0-400 cm typical (depending on distance mode and target)
     * @warning Returns false if sensor reports error conditions (no target, signal fail, etc.)
     */
    bool get_reading(uint16_t &reading_cm);
    
    /**
     * @brief I2C device handle for sensor communication
     * 
     * @details Owned pointer to I2C device interface used for all register reads
     *          and writes. Automatically manages device lifecycle and I2C bus access.
     *          
     *          Communication parameters:
     *          - Default address: 0x29 (7-bit)
     *          - Speed: Up to 400 kHz (Fast mode I2C)
     *          - 16-bit register addressing
     */
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    /**
     * @brief Timing guard value for measurement timing budget calculations
     * 
     * @details Used in calculating measurement timing budgets for LOWPOWER_AUTONOMOUS mode.
     *          
     *          Calculation breakdown (in microseconds):
     *          - VHV loop time: LOWPOWER_AUTO_VHV_LOOP_DURATION_US + 
     *                          (LOWPOWERAUTO_VHV_LOOP_BOUND * LOWPOWER_AUTO_VHV_LOOP_DURATION_US)
     *                        = 245 + (3 * 245) = 980 μs
     *          
     *          - TimingGuard: LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
     *                        LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + 
     *                        VHV loop time
     *                      = 1448 + 2100 + 980 = 4528 μs
     *          
     *          This overhead must be subtracted from the user-specified timing budget
     *          to determine the time available for actual ranging measurements.
     * 
     * @note Units: microseconds
     * @note Based on ST VL53L1X API timing constants
     */
    static const uint32_t TimingGuard = 4528;

    /**
     * @brief Target signal rate for Dynamic SPAD Selection (DSS)
     * 
     * @details Default value written to DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register.
     *          DSS automatically adjusts the number of active SPADs (Single Photon
     *          Avalanche Diodes) to maintain this target signal rate under varying
     *          lighting conditions and target distances.
     *          
     *          Value 0x0A00 in 9.7 fixed-point format represents:
     *          0x0A00 >> 7 = 20 Mcps (mega counts per second)
     * 
     * @note Units: Mcps in 9.7 fixed-point format (value << 7)
     * @note Higher values improve long-range performance but may increase noise
     */
    static const uint16_t TargetRate = 0x0A00;

    /**
     * @brief Fast oscillator frequency measurement from sensor
     * 
     * @details Read from OSC_MEASURED__FAST_OSC__FREQUENCY register during initialization.
     *          Used to calculate timing parameters and macro periods for ranging operations.
     *          
     *          Typical value: ~0x01C0 (448 in decimal), representing the oscillator
     *          frequency in a device-specific format used for timing calculations.
     * 
     * @note Read once during init() and cached for subsequent timing calculations
     */
    uint16_t fast_osc_frequency;
    
    /**
     * @brief Oscillator calibration value from sensor
     * 
     * @details Read from RESULT__OSC_CALIBRATE_VAL register during initialization.
     *          Factory-calibrated value specific to each sensor used in macro period
     *          and timing budget calculations.
     *          
     *          This value compensates for manufacturing variations in the sensor's
     *          internal oscillator.
     * 
     * @note Read once during init() and cached for subsequent timing calculations
     */
    uint16_t osc_calibrate_val;
    
    /**
     * @brief Accumulator for distance measurements during calibration
     * 
     * @details Running sum of distance measurements in millimeters used to calculate
     *          average during the calibration period. Reset to 0 at start of calibration.
     *          
     *          Used in conjunction with `counter` to compute calibration offset.
     * 
     * @note Units: millimeters (accumulated sum)
     * @see counter, calibrated
     */
    uint32_t sum_mm;
    
    /**
     * @brief Count of measurements taken during calibration period
     * 
     * @details Incremented for each valid measurement during calibration.
     *          Once counter reaches threshold, calibration is complete and average
     *          offset is computed as sum_mm / counter.
     * 
     * @note Reset to 0 at start of calibration
     * @see sum_mm, calibrated
     */
    uint32_t counter;
    
    /**
     * @brief Calibration completion flag
     * 
     * @details Set to true once initial calibration period is complete.
     *          
     *          - false: Still collecting calibration samples
     *          - true: Calibration complete, applying offset to measurements
     *          
     *          Calibration typically requires collecting measurements for a short
     *          period to establish sensor-specific offset correction.
     * 
     * @note Initialized to false in init()
     */
    bool calibrated;

    /**
     * @brief Read 8-bit value from sensor register
     * 
     * @details Performs I2C read transaction with 16-bit register address.
     *          VL53L1X uses 16-bit register addressing requiring MSB-first addressing.
     * 
     * @param[in]  reg   16-bit register address to read from
     * @param[out] value Reference to store the read 8-bit value
     * 
     * @return true if read successful, false on I2C communication error
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Blocking I2C operation - called from timer thread
     */
    bool read_register(uint16_t reg, uint8_t &value) WARN_IF_UNUSED;
    
    /**
     * @brief Read 16-bit value from sensor register
     * 
     * @details Reads two consecutive 8-bit registers and combines them into 16-bit value.
     *          Data is read in big-endian format (MSB first, then LSB).
     * 
     * @param[in]  reg   16-bit register address to read from (MSB register)
     * @param[out] value Reference to store the read 16-bit value
     * 
     * @return true if read successful, false on I2C communication error
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Automatically reads reg and reg+1 and combines them
     */
    bool read_register16(uint16_t reg, uint16_t &value) WARN_IF_UNUSED;
    
    /**
     * @brief Write 8-bit value to sensor register
     * 
     * @details Performs I2C write transaction with 16-bit register address.
     * 
     * @param[in] reg   16-bit register address to write to
     * @param[in] value 8-bit value to write
     * 
     * @return true if write successful, false on I2C communication error
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Blocking I2C operation - called from timer thread
     */
    bool write_register(uint16_t reg, uint8_t value) WARN_IF_UNUSED;
    
    /**
     * @brief Write 16-bit value to sensor register
     * 
     * @details Writes 16-bit value as two consecutive 8-bit register writes.
     *          Data is written in big-endian format (MSB first, then LSB).
     * 
     * @param[in] reg   16-bit register address to write to (MSB register)
     * @param[in] value 16-bit value to write
     * 
     * @return true if write successful, false on I2C communication error
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Automatically writes to reg and reg+1
     */
    bool write_register16(uint16_t reg, uint16_t value) WARN_IF_UNUSED;
    
    /**
     * @brief Write 32-bit value to sensor register
     * 
     * @details Writes 32-bit value as four consecutive 8-bit register writes.
     *          Data is written in big-endian format (MSB first through LSB last).
     * 
     * @param[in] reg   16-bit register address to write to (MSB register)
     * @param[in] value 32-bit value to write
     * 
     * @return true if write successful, false on I2C communication error
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Automatically writes to reg, reg+1, reg+2, reg+3
     */
    bool write_register32(uint16_t reg, uint32_t value) WARN_IF_UNUSED;
    /**
     * @brief Check if new measurement data is ready
     * 
     * @details Polls RESULT__INTERRUPT_STATUS register to check if a new ranging
     *          measurement has completed and data is available to read.
     *          
     *          In continuous ranging mode, this indicates the sensor has completed
     *          a ranging cycle and result registers contain fresh data.
     * 
     * @return true if new data is ready to read, false if no new data available
     * 
     * @note Non-blocking poll operation
     * @note Does not clear the interrupt status - reading results will clear it
     */
    bool dataReady(void);
    
    /**
     * @brief Perform software reset of the sensor
     * 
     * @details Issues soft reset command via SOFT_RESET register, then waits for
     *          sensor to complete reset and become ready for configuration.
     *          
     *          Reset sequence:
     *          1. Write 0x00 to SOFT_RESET register (initiate reset)
     *          2. Write 0x01 to SOFT_RESET register (release reset)
     *          3. Wait for FIRMWARE__SYSTEM_STATUS to indicate ready
     *          
     *          After reset, all sensor registers return to default values and
     *          sensor must be reconfigured before use.
     * 
     * @return true if reset successful, false if reset or ready check failed
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Blocking operation with timeout waiting for sensor ready status
     * @note Called during init() to ensure known sensor state
     */
    bool reset(void) WARN_IF_UNUSED;
    
    /**
     * @brief Configure ranging distance mode
     * 
     * @details Sets the distance mode which affects maximum range, timing budget,
     *          and measurement accuracy characteristics.
     *          
     *          Distance modes:
     *          - Short: Up to 1.3m, better ambient immunity, faster measurements
     *          - Medium: Up to 3m, balanced performance
     *          - Long: Up to 4m, maximum range but longer timing budget required
     *          
     *          Each mode configures different VCSEL pulse periods and timing parameters
     *          to optimize for the target range. Mode selection affects:
     *          - RANGE_CONFIG__VCSEL_PERIOD_A/B registers
     *          - Timeout and timing budget calculations
     *          - Sigma (noise) threshold values
     * 
     * @param[in] distance_mode The desired distance mode (Short, Medium, or Long)
     * 
     * @return true if mode set successfully, false on register write failure
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Should be called during init() before starting continuous ranging
     * @note Changing mode during operation may cause measurement glitches
     */
    bool setDistanceMode(DistanceMode distance_mode) WARN_IF_UNUSED;
    /**
     * @brief Set measurement timing budget
     * 
     * @details Configures the maximum time allowed for a single ranging measurement.
     *          Longer timing budgets allow more signal averaging and improve accuracy
     *          and maximum range, but reduce update rate.
     *          
     *          The timing budget is allocated across multiple ranging phases:
     *          - Phase A (initial range): MM_CONFIG__TIMEOUT_MACROP_A
     *          - Phase B (final range): RANGE_CONFIG__TIMEOUT_MACROP_A/B
     *          - Overhead: TimingGuard (VHV calibration, ranging overhead)
     *          
     *          Valid range: Depends on distance mode
     *          - Minimum: ~20ms (required overhead + minimal ranging time)
     *          - Maximum: Limited by uint32_t timeout register values
     *          
     *          Timing budget affects measurement quality vs speed tradeoff.
     * 
     * @param[in] budget_us Desired timing budget in microseconds
     * 
     * @return true if timing budget set successfully, false on invalid budget or register write failure
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Must be called after setDistanceMode() as budget calculations depend on mode
     * @note Actual budget may be slightly adjusted to match hardware constraints
     * @see TimingGuard, getMeasurementTimingBudget()
     */
    bool setMeasurementTimingBudget(uint32_t budget_us) WARN_IF_UNUSED;
    
    /**
     * @brief Get current measurement timing budget
     * 
     * @details Reads current timing budget configuration from sensor registers and
     *          calculates total timing budget including all ranging phases and overhead.
     *          
     *          Reads timeout values from:
     *          - MM_CONFIG__TIMEOUT_MACROP_A/B (pre-range phase)
     *          - RANGE_CONFIG__TIMEOUT_MACROP_A/B (final range phase)
     *          
     *          Then adds TimingGuard overhead to compute total budget.
     * 
     * @param[out] budget Reference to store calculated timing budget in microseconds
     * 
     * @return true if budget read successfully, false on register read failure
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Useful for verifying timing budget after setMeasurementTimingBudget()
     * @see setMeasurementTimingBudget(), TimingGuard
     */
    bool getMeasurementTimingBudget(uint32_t &budget) WARN_IF_UNUSED;
    
    /**
     * @brief Start continuous ranging mode with specified intermeasurement period
     * 
     * @details Configures sensor for continuous autonomous ranging with specified
     *          time between measurements. Sensor will continuously perform ranging
     *          cycles at the specified interval.
     *          
     *          Configuration steps:
     *          1. Set intermeasurement period (SYSTEM__INTERMEASUREMENT_PERIOD)
     *          2. Configure ranging mode and sequences
     *          3. Start autonomous ranging mode (SYSTEM__MODE_START)
     *          
     *          The intermeasurement period must be >= measurement timing budget.
     *          If period < budget, measurements will run back-to-back with no gap.
     *          If period > budget, sensor idles between measurements to save power.
     * 
     * @param[in] period_ms Time between measurement starts in milliseconds
     * 
     * @return true if continuous mode started successfully, false on configuration failure
     * 
     * @note WARN_IF_UNUSED: Compiler will warn if return value is not checked
     * @note Must be called after setDistanceMode() and setMeasurementTimingBudget()
     * @note Sensor will continue ranging until reset or power cycled
     * @note New data available at approximately period_ms intervals (check with dataReady())
     */
    bool startContinuous(uint32_t period_ms) WARN_IF_UNUSED;
    /**
     * @brief Decode timeout value from register format to macro clocks
     * 
     * @details VL53L1X timeout registers use a compressed format to represent large
     *          timeout values in 16 bits. This function decodes the register value
     *          into actual macro clock counts.
     *          
     *          Register format: [15:0] = timeout_encoded
     *          - Bits [7:0]: LSB (mantissa-like value)
     *          - Bits [15:8]: MSB (exponent-like value)
     *          
     *          Decoding formula per ST API:
     *          timeout_mclks = (LSB + 1) * 2^MSB
     *          
     *          This allows representing timeouts from ~1 to ~16,777,216 macro clocks.
     * 
     * @param[in] reg_val 16-bit encoded timeout value from register
     * 
     * @return Decoded timeout value in macro clock cycles
     * 
     * @note Macro clocks are timing units specific to the sensor's ranging core
     * @see encodeTimeout(), timeoutMclksToMicroseconds()
     */
    uint32_t decodeTimeout(uint16_t reg_val);
    
    /**
     * @brief Encode timeout value from macro clocks to register format
     * 
     * @details Converts timeout in macro clock cycles to the compressed 16-bit
     *          register format used by VL53L1X timeout registers.
     *          
     *          Encoding process finds MSB (exponent) and LSB (mantissa) such that:
     *          timeout_mclks ≈ (LSB + 1) * 2^MSB
     *          
     *          The algorithm finds the smallest MSB where:
     *          timeout_mclks / 2^MSB fits in LSB field (≤ 255)
     *          
     *          Some precision may be lost for values that don't exactly match the
     *          compressed format.
     * 
     * @param[in] timeout_mclks Timeout value in macro clock cycles
     * 
     * @return 16-bit encoded timeout value suitable for writing to timeout registers
     * 
     * @note Inverse operation of decodeTimeout()
     * @see decodeTimeout(), timeoutMicrosecondsToMclks()
     */
    uint16_t encodeTimeout(uint32_t timeout_mclks);
    
    /**
     * @brief Convert timeout from macro clocks to microseconds
     * 
     * @details Converts sensor-specific macro clock timing units to real-world
     *          microseconds based on the current macro period (which depends on
     *          VCSEL pulse period and oscillator frequency).
     *          
     *          Conversion formula:
     *          timeout_us = timeout_mclks * macro_period_us
     *          
     *          The macro period represents the duration of one macro clock cycle
     *          in microseconds, calculated from the sensor's oscillator frequency
     *          and VCSEL configuration.
     * 
     * @param[in] timeout_mclks Timeout value in macro clock cycles
     * @param[in] macro_period_us Duration of one macro clock cycle in microseconds
     * 
     * @return Timeout value in microseconds
     * 
     * @note macro_period_us should be calculated using calcMacroPeriod()
     * @see timeoutMicrosecondsToMclks(), calcMacroPeriod()
     */
    uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
    
    /**
     * @brief Convert timeout from microseconds to macro clocks
     * 
     * @details Converts real-world microseconds to sensor-specific macro clock
     *          timing units for writing to timeout registers.
     *          
     *          Conversion formula:
     *          timeout_mclks = timeout_us / macro_period_us
     *          
     *          This is the inverse operation of timeoutMclksToMicroseconds().
     * 
     * @param[in] timeout_us Timeout value in microseconds
     * @param[in] macro_period_us Duration of one macro clock cycle in microseconds
     * 
     * @return Timeout value in macro clock cycles
     * 
     * @note Result may be rounded if timeout_us is not an exact multiple of macro_period_us
     * @note macro_period_us should be calculated using calcMacroPeriod()
     * @see timeoutMclksToMicroseconds(), calcMacroPeriod()
     */
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
    
    /**
     * @brief Calculate macro period from VCSEL pulse period
     * 
     * @details Computes the duration of one macro clock cycle in microseconds based
     *          on the VCSEL (Vertical Cavity Surface Emitting Laser) pulse period
     *          and sensor oscillator calibration values.
     *          
     *          The macro period determines the timing resolution for ranging operations
     *          and is used to convert between macro clock cycles and real-world time.
     *          
     *          Calculation per ST API:
     *          macro_period_us = (2304 * vcsel_period * osc_calibrate_val) / fast_osc_frequency
     *          
     *          Where:
     *          - 2304 is a hardware-specific constant
     *          - vcsel_period is typically 10, 12, 14, or 18 (from VCSEL period registers)
     *          - osc_calibrate_val and fast_osc_frequency are read during init()
     * 
     * @param[in] vcsel_period VCSEL pulse period value from sensor configuration
     * 
     * @return Macro period duration in microseconds
     * 
     * @note Uses cached osc_calibrate_val and fast_osc_frequency values
     * @note VCSEL period affects ranging performance (longer = better signal, longer timing)
     * @see timeoutMclksToMicroseconds(), timeoutMicrosecondsToMclks()
     */
    uint32_t calcMacroPeriod(uint8_t vcsel_period) const;
    
    /**
     * @brief Setup manual calibration mode
     * 
     * @details Configures sensor for manual calibration to establish offset correction.
     *          Manual calibration allows the driver to collect measurements at a known
     *          distance and compute an offset to correct for sensor-specific biases.
     *          
     *          This function prepares the sensor hardware for calibration by:
     *          - Disabling automatic calibration features
     *          - Configuring measurement parameters for calibration
     *          - Initializing calibration state variables (sum_mm, counter, calibrated)
     *          
     *          After calling this function, the driver collects multiple measurements
     *          in timer() callback, accumulates them, and computes average offset.
     * 
     * @return true if calibration setup successful, false on configuration error
     * 
     * @note Called during init() or when recalibration is needed
     * @note Sensor should be aimed at a known target distance during calibration
     * @note Calibration improves absolute accuracy but is not required for operation
     * @see sum_mm, counter, calibrated
     */
    bool setupManualCalibration(void);
};

#endif  // AP_RANGEFINDER_VL53L1X_ENABLED
