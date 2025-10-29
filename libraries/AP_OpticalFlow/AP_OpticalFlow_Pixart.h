/**
 * @file AP_OpticalFlow_Pixart.h
 * @brief PixArt PMW3900/3901 SPI optical flow sensor backend
 * 
 * @details Hardware: PixArt PAW3900/3901 gaming sensor chips adapted for drone optical flow
 *          Protocol: SPI register-based interface with SROM firmware upload
 *          Features: High-speed motion detection (up to 250 frames/s theoretical, typically ~100Hz)
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AP_OpticalFlow_config.h"

#if AP_OPTICALFLOW_PIXART_ENABLED

#include "AP_OpticalFlow_Backend.h"

#include <AP_HAL/utility/OwnPtr.h>

/**
 * @class AP_OpticalFlow_Pixart
 * @brief PixArt PMW3900/3901 SPI optical flow sensor backend
 * 
 * @details Detects PMW3900 or PMW3901 via SPI product ID register, uploads SROM firmware blob
 *          (4KB from AP_OpticalFlow_Pixart_SROM.h), configures sensor with model-specific init
 *          sequences, performs periodic SPI motion_burst reads to capture delta_x/delta_y/quality,
 *          accumulates motion between frontend updates, converts pixels to rad/s using
 *          flow_pixel_scaling (1.26e-3 rad/pixel).
 * 
 *          Algorithm:
 *          1. Detect model via product ID register (distinguishes 3900 vs 3901)
 *          2. Upload 4096-byte SROM firmware blob via SPI burst writes
 *          3. Apply model-specific initialization register sequences
 *          4. Register periodic timer callback (~100Hz) for motion_burst reads
 *          5. Accumulate delta_x/delta_y between frontend update() calls
 *          6. Convert accumulated pixels to rad/s, apply gyro compensation
 * 
 * @note Requires SROM firmware upload at initialization - see AP_OpticalFlow_Pixart_SROM.h
 * @warning SPI must operate at correct speed (typically 2-4 MHz) for reliable communication
 * @warning Surface texture requirements: high contrast features, proper distance (0.08-3m typical)
 * @warning Lighting requirements: avoid direct sunlight or IR sources
 */
class AP_OpticalFlow_Pixart : public OpticalFlow_backend
{
public:
    /**
     * @brief Initialize PixArt optical flow sensor backend
     * 
     * @details Constructor stores SPI device name and frontend reference. Actual sensor
     *          initialization (model detection, SROM upload, configuration) is performed
     *          in setup_sensor() called from detect().
     * 
     * @param[in] devname SPI device name (e.g. "pixartflow" from HAL board definition)
     * @param[in] _frontend AP_OpticalFlow manager reference for update callbacks
     */
    AP_OpticalFlow_Pixart(const char *devname, AP_OpticalFlow &_frontend);

    /**
     * @brief Initialize the sensor (no operation - detection does setup)
     * 
     * @details Empty implementation because all initialization is performed during detect()
     *          via setup_sensor(). This is marked override to satisfy OpticalFlow_backend
     *          interface requirements.
     * 
     * @note Detection and SROM upload occur in setup_sensor() before object construction
     */
    void init() override {}

    /**
     * @brief Process accumulated motion integrals and update frontend
     * 
     * @details Called periodically by AP_OpticalFlow frontend to retrieve accumulated flow data.
     *          Computes time delta from integral timestamps, converts accumulated pixel counts
     *          to flow rates (rad/s) using flow_pixel_scaling (1.26e-3 rad/pixel), applies
     *          gyro compensation if available, and calls _update_frontend() with results.
     * 
     *          Algorithm:
     *          1. Calculate dt from integral.last_frame_us and integral.sum_us
     *          2. Convert integral.sum (pixels) to flow rates: rate = sum * flow_pixel_scaling / dt
     *          3. Apply gyro compensation from integral.gyro
     *          4. Call _update_frontend() with flow_x, flow_y, quality
     *          5. Reset integral accumulators
     * 
     * @return via _update_frontend() callback to AP_OpticalFlow manager
     * 
     * @note Update rate depends on frontend scheduler (typically 10-20Hz)
     * @note Timer callback runs at ~100Hz, accumulating motion between update() calls
     */
    void update(void) override;

    /**
     * @brief Static factory method to detect and initialize PixArt sensor
     * 
     * @details Probes SPI bus for PixArt PMW3900/3901 sensor, uploads SROM firmware,
     *          and performs model-specific initialization. Returns configured instance
     *          or nullptr if detection fails.
     * 
     *          Detection sequence:
     *          1. Attempt to open SPI device with specified devname
     *          2. Call setup_sensor() which:
     *             - Reads product ID register to detect 3900 vs 3901
     *             - Uploads 4096-byte SROM firmware blob
     *             - Applies model-specific register configuration
     *             - Validates SROM ID (0xED)
     *          3. Register periodic timer callback for motion_burst reads
     *          4. Return configured instance or nullptr on failure
     * 
     * @param[in] devname SPI device name (e.g. "pixartflow" from board hwdef)
     * @param[in] _frontend AP_OpticalFlow manager reference
     * 
     * @return Configured AP_OpticalFlow_Pixart instance on success, nullptr on failure
     * 
     * @note Called during AP_OpticalFlow sensor discovery at boot
     * @note SROM upload takes significant time (~100ms) during initialization
     */
    static AP_OpticalFlow_Pixart *detect(const char *devname, AP_OpticalFlow &_frontend);

private:
    /// SPI device handle for communication with PixArt sensor
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    /**
     * @enum model
     * @brief PixArt sensor model identifier
     * 
     * @details Determines which initialization sequence to use. Detected via product ID
     *          register during setup_sensor(). PMW3900 and PMW3901 require different
     *          register configurations for optimal performance.
     */
    enum {
        PIXART_3900=0,  ///< PMW3900 sensor model
        PIXART_3901=1   ///< PMW3901 sensor model
    } model;
    
    /**
     * @struct RegData
     * @brief Register address/value pair for initialization sequences
     * 
     * @details Used in init_data_3900, init_data_3901_1, init_data_3901_2 arrays
     *          to configure sensor registers during setup. Each array contains
     *          model-specific register settings applied via load_configuration().
     */
    struct RegData {
        uint8_t reg;    ///< Register address (0x00-0xFF)
        uint8_t value;  ///< Register value to write (0x00-0xFF)
    };

    /**
     * @struct MotionBurst
     * @brief 10-byte SPI motion burst read structure
     * 
     * @details Packed structure matching PixArt motion burst register layout.
     *          Read via single SPI transaction to minimize latency and ensure
     *          coherent motion data capture. All fields are sensor-native format.
     * 
     * @note PACKED attribute ensures no padding between fields for SPI transfer
     * @warning Field order must match sensor burst register layout exactly
     */
    struct PACKED MotionBurst {
        uint8_t motion;          ///< Motion detected flag (bit 7) and motion overflow bits
        uint8_t observation;     ///< Observation flags and feature detection
        int16_t delta_x;         ///< Signed 16-bit pixel motion X (pixels since last burst)
        int16_t delta_y;         ///< Signed 16-bit pixel motion Y (pixels since last burst)
        uint8_t squal;           ///< Surface quality 0-255 (higher = better feature tracking)
        uint8_t rawdata_sum;     ///< Raw sensor data sum for diagnostics
        uint8_t max_raw;         ///< Maximum raw pixel value
        uint8_t min_raw;         ///< Minimum raw pixel value
        uint8_t shutter_upper;   ///< Shutter time upper byte (exposure control)
        uint8_t shutter_lower;   ///< Shutter time lower byte (exposure control)
    } burst;

    /**
     * @struct integral
     * @brief Motion accumulation structure for inter-update integration
     * 
     * @details Accumulates delta_x/delta_y from high-rate timer() callbacks (~100Hz)
     *          between lower-rate frontend update() calls (~10-20Hz). Provides smooth
     *          flow data by integrating multiple sensor readings per update cycle.
     * 
     * @note Timer callback accumulates motion at sensor rate, update() converts to rates
     */
    struct {
        Vector2l sum;            ///< Accumulated pixel deltas (x,y) since last update() [pixels]
        uint32_t last_frame_us;  ///< Timestamp of first motion burst in current integration [microseconds]
        uint32_t sum_us;         ///< Timestamp of last motion burst in current integration [microseconds]
        Vector2f gyro;           ///< Gyro sample for velocity compensation [rad/s]
    } integral;
    
    static const uint8_t srom_data[];       ///< 4096-byte SROM firmware blob (defined in AP_OpticalFlow_Pixart_SROM.h)
    static const uint8_t srom_id;           ///< Expected SROM ID 0xED for validation after upload
    static const RegData init_data_3900[];  ///< PMW3900 initialization register sequence
    static const RegData init_data_3901_1[];///< PMW3901 initialization sequence part 1
    static const RegData init_data_3901_2[];///< PMW3901 initialization sequence part 2
    const float flow_pixel_scaling = 1.26e-3; ///< Conversion factor 1.26e-3 rad/pixel (sensor geometry)

    /**
     * @brief Detect model, upload SROM firmware, and configure sensor
     * 
     * @details Performs complete sensor initialization sequence:
     *          1. Read product ID register to detect PMW3900 vs PMW3901
     *          2. Call srom_download() to upload 4096-byte firmware blob
     *          3. Validate SROM ID register reads 0xED
     *          4. Apply model-specific init_data register sequences
     *          5. Configure sensor for continuous operation
     * 
     * @return true if sensor detected and configured successfully, false on failure
     * 
     * @note Called during detect() static factory method
     * @warning SROM upload is required - sensor will not function without firmware
     * @warning Takes ~100ms to complete due to SROM upload time
     */
    bool setup_sensor(void);
    
    /**
     * @brief Write single byte to sensor register via SPI
     * 
     * @param[in] reg Register address (0x00-0xFF)
     * @param[in] value Byte value to write (0x00-0xFF)
     * 
     * @note Includes required SPI timing delays per datasheet
     */
    void reg_write(uint8_t reg, uint8_t value);
    
    /**
     * @brief Read single byte from sensor register via SPI
     * 
     * @param[in] reg Register address (0x00-0xFF)
     * @return uint8_t Register value read (0x00-0xFF)
     * 
     * @note Includes required SPI timing delays per datasheet
     */
    uint8_t reg_read(uint8_t reg);
    
    /**
     * @brief Read 16-bit signed value from sensor register pair via SPI
     * 
     * @param[in] reg Base register address (reads reg and reg+1)
     * @return int16_t Signed 16-bit value (typically delta_x or delta_y)
     * 
     * @note Little-endian byte order: LSB at reg, MSB at reg+1
     */
    int16_t reg_read16s(uint8_t reg);
    
    /**
     * @brief Read 16-bit unsigned value from sensor register pair via SPI
     * 
     * @param[in] reg Base register address (reads reg and reg+1)
     * @return uint16_t Unsigned 16-bit value
     * 
     * @note Little-endian byte order: LSB at reg, MSB at reg+1
     */
    uint16_t reg_read16u(uint8_t reg);

    /**
     * @brief Upload 4096-byte SROM firmware blob to sensor via SPI
     * 
     * @details Performs firmware upload sequence per PixArt datasheet:
     *          1. Enable SROM download mode via register write
     *          2. Transfer srom_data[] array (4096 bytes) via SPI burst writes
     *          3. Disable SROM download mode
     *          4. Validate SROM ID register reads expected value (0xED)
     * 
     * @note Required at every sensor power-up - SROM is volatile memory
     * @warning Sensor will not provide valid motion data until SROM is loaded
     * @warning Upload takes ~100ms - blocks during initialization
     */
    void srom_download(void);
    
    /**
     * @brief Apply register initialization array to sensor
     * 
     * @details Writes sequence of register/value pairs to configure sensor.
     *          Used with init_data_3900, init_data_3901_1, init_data_3901_2 arrays.
     * 
     * @param[in] init_data Pointer to RegData array of register/value pairs
     * @param[in] n Number of entries in init_data array
     * 
     * @note Includes required delays between register writes per datasheet
     */
    void load_configuration(const RegData *init_data, uint16_t n);

    /**
     * @brief Periodic SPI callback for motion burst reads
     * 
     * @details Registered as periodic timer callback (~100Hz) by detect().
     *          Calls motion_burst() to read sensor, accumulates delta_x/delta_y
     *          into integral.sum for later conversion to rates by update().
     * 
     * @note High-rate timer callback minimizes motion data latency
     * @note Runs at sensor update rate (typically ~100Hz)
     */
    void timer(void);
    
    /**
     * @brief Read 10-byte MotionBurst structure via SPI and update integrals
     * 
     * @details Performs single SPI burst read transaction to capture coherent motion data.
     *          Extracts motion/delta_x/delta_y/squal from burst structure, updates
     *          integral.sum with delta_x/delta_y, stores gyro sample for compensation.
     * 
     *          Algorithm:
     *          1. Initiate motion burst read (0x16 register)
     *          2. Read 10-byte MotionBurst structure via SPI
     *          3. Check motion bit to verify data validity
     *          4. Accumulate delta_x/delta_y into integral.sum
     *          5. Update integral timestamps for dt calculation
     *          6. Store gyro sample for update() compensation
     * 
     * @note Called by timer() at ~100Hz rate
     * @note Burst read minimizes SPI transaction overhead vs individual register reads
     */
    void motion_burst(void);

    uint32_t last_burst_us;   ///< Timestamp of last motion_burst read [microseconds]
    uint32_t last_update_ms;  ///< Timestamp of last frontend update() call [milliseconds]
};

#endif  // AP_OPTICALFLOW_PIXART_ENABLED
