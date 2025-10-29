/**
 * @file AP_RangeFinder_BBB_PRU.h
 * @brief BeagleBone Black PRU (Programmable Real-time Unit) ultrasonic rangefinder driver
 * 
 * @details This driver provides distance sensing support using the BeagleBone Black's
 *          PRU subsystem for hardware-accelerated ultrasonic sensor interfacing.
 *          The PRU handles real-time signal generation and echo timing independently
 *          of the main ARM processor, providing deterministic timing for accurate
 *          distance measurements.
 * 
 * @note This backend is Linux HAL-specific and requires BeagleBone Black hardware
 *       with PRU kernel support enabled (typically via remoteproc or UIO drivers).
 * 
 * @warning Platform restrictions: Only functional on BeagleBone Black with AM335x
 *          PRU-ICSS (Programmable Real-time Unit and Industrial Communication Subsystem).
 *          Will not function on other ARM boards or x86 platforms.
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BBB_PRU_ENABLED

#include "AP_RangeFinder_Backend.h"

/**
 * @name PRU Memory-Mapped I/O Base Addresses
 * @brief Physical memory addresses for PRU0 subsystem access on AM335x
 * @{
 */

/** @brief PRU0 Control Register Base Address
 *  @details Base address for PRU0 control and status registers including:
 *           - Control register (enable/disable PRU)
 *           - Status register (execution state)
 *           - Wake-up enable register
 *  Memory-mapped access to these registers controls PRU execution
 */
#define PRU0_CTRL_BASE 0x4a322000

/** @brief PRU0 Instruction RAM Base Address
 *  @details Starting address of PRU0's 8KB instruction RAM where PRU firmware code resides.
 *           The PRU firmware handles ultrasonic pulse generation and echo timing.
 */
#define PRU0_IRAM_BASE 0x4a334000

/** @brief PRU0 Instruction RAM Size in bytes (8KB) */
#define PRU0_IRAM_SIZE 0x2000

/** @brief PRU0 Data RAM Base Address
 *  @details Starting address of PRU0's 8KB data RAM used for:
 *           - Shared memory communication between ARM and PRU
 *           - Storage of distance measurements and status
 *           - PRU firmware variables and buffers
 */
#define PRU0_DRAM_BASE 0x4a300000

/** @brief PRU0 Data RAM Size in bytes (8KB) */
#define PRU0_DRAM_SIZE 0x2000

/** @} */ // End of PRU Memory-Mapped I/O group

/**
 * @struct range
 * @brief Shared memory structure for PRU rangefinder data
 * 
 * @details This structure is mapped to PRU0 data RAM and used for bidirectional
 *          communication between the ARM processor and PRU. The PRU firmware writes
 *          distance measurements and status information, which the ARM processor
 *          reads via memory-mapped I/O.
 * 
 * @note Structure layout must match the corresponding structure in PRU firmware
 *       to ensure correct data interpretation. Any changes require updating both
 *       ARM-side and PRU-side code.
 * 
 * @note Units for distance field are determined by PRU firmware configuration,
 *       typically raw timer counts or pre-converted to centimeters/millimeters.
 */
struct range {
        /** @brief Distance measurement from ultrasonic sensor
         *  @details Units determined by PRU firmware configuration. Common configurations:
         *           - Raw timer ticks (requires ARM-side conversion)
         *           - Centimeters (pre-converted by PRU)
         *           - Millimeters (pre-converted by PRU)
         *  Value represents round-trip time or calculated distance based on firmware.
         */
        uint32_t distance;

        /** @brief Status flags from PRU measurement
         *  @details Status indicator for measurement validity:
         *           - 0: Invalid/no echo received
         *           - 1: Valid measurement
         *           - Other values: Firmware-specific error codes
         *  Check this field before using distance value to avoid stale or invalid data.
         */
	uint32_t status;
};

/**
 * @class AP_RangeFinder_BBB_PRU
 * @brief BeagleBone Black PRU-based rangefinder backend implementation
 * 
 * @details This backend leverages the BeagleBone Black's PRU (Programmable Real-time Unit)
 *          subsystem for hardware-accelerated ultrasonic distance sensing. Unlike I2C or
 *          UART-based rangefinder backends, this implementation uses memory-mapped I/O
 *          to communicate with PRU firmware that handles real-time ultrasonic pulse
 *          generation and echo timing.
 * 
 *          Architecture:
 *          - PRU firmware generates ultrasonic pulses and measures echo timing
 *          - Distance data written to PRU data RAM by firmware
 *          - ARM processor reads measurements via memory-mapped access
 *          - No I2C, SPI, or UART protocols involved
 * 
 *          Hardware Requirements:
 *          - BeagleBone Black (AM335x processor with PRU-ICSS)
 *          - PRU kernel support (remoteproc or UIO driver)
 *          - Ultrasonic sensor connected to PRU GPIO pins
 *          - PRU firmware loaded and running
 * 
 * @note This backend is Linux HAL-specific and only functional on BeagleBone Black.
 *       Detection will fail on other platforms.
 * 
 * @warning Platform-specific: Requires BeagleBone Black hardware with PRU support
 *          enabled in kernel. Will not work on other ARM boards, x86, or without
 *          PRU kernel drivers.
 * 
 * @see AP_RangeFinder_Backend for base class interface
 */
class AP_RangeFinder_BBB_PRU : public AP_RangeFinder_Backend
{
public:
    /*
        Constructor:
        The constructor also initialises the rangefinder. Note that this
        constructor is not called until detect() returns true, so we
        already know that we should setup the rangefinder
    */
    using AP_RangeFinder_Backend::AP_RangeFinder_Backend;

    /**
     * @brief Static detection function for PRU rangefinder hardware availability
     * 
     * @details Probes for BeagleBone Black PRU hardware support by attempting to access
     *          PRU memory-mapped registers. This method verifies:
     *          - Running on Linux HAL (BeagleBone platform)
     *          - PRU memory regions accessible via /dev/mem or similar
     *          - PRU kernel drivers loaded (remoteproc or UIO)
     *          - PRU0 subsystem available and responsive
     * 
     *          Called during rangefinder initialization to determine if this backend
     *          should be instantiated. Returns false on non-Linux platforms or when
     *          PRU hardware is unavailable.
     * 
     * @return true if PRU hardware detected and accessible, false otherwise
     * 
     * @note This is a static method called before object construction. Only if this
     *       returns true will the AP_RangeFinder_BBB_PRU object be created.
     * 
     * @warning Platform-specific: Only returns true on BeagleBone Black with PRU
     *          kernel support. Always returns false on other platforms (ChibiOS,
     *          generic Linux boards, SITL, etc.).
     */
    static bool detect();

    /**
     * @brief Update rangefinder state by reading distance from PRU memory
     * 
     * @details Reads the latest distance measurement from PRU data RAM via memory-mapped
     *          I/O. The PRU firmware continuously updates the shared memory structure
     *          with new measurements, and this method retrieves the most recent data.
     * 
     *          Update process:
     *          1. Memory-map PRU0 data RAM to access range struct
     *          2. Read distance and status fields from shared memory
     *          3. Validate measurement using status field
     *          4. Update backend state with distance (if valid) or error status
     *          5. Apply any scaling/conversion from PRU firmware units to standard units
     * 
     *          Called periodically by the rangefinder scheduler (typically 50-100Hz)
     *          to maintain current distance readings.
     * 
     * @note This method performs memory-mapped I/O reads, which are fast but require
     *       proper memory barriers on ARM to ensure data coherency between PRU and CPU.
     * 
     * @note Distance units from PRU firmware must be converted to centimeters as expected
     *       by the RangeFinder library base class.
     * 
     * @warning Direct memory access without proper synchronization could read partially
     *          updated values. Implementation must ensure atomic reads or use PRU firmware
     *          synchronization mechanisms.
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink sensor type enumeration for telemetry reporting
     *          to ground control stations. Identifies this sensor as ultrasonic for
     *          proper interpretation by GCS software.
     * 
     * @return MAV_DISTANCE_SENSOR_ULTRASOUND indicating ultrasonic sensor type
     * 
     * @note This identification is used in DISTANCE_SENSOR MAVLink messages sent
     *       to ground stations, allowing GCS to display appropriate sensor type
     *       information and apply sensor-specific handling.
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

};

#endif  // AP_RANGEFINDER_BBB_PRU_ENABLED
