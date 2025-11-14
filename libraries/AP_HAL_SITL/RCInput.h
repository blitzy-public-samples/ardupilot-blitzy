/**
 * @file RCInput.h
 * @brief RC (Radio Control) input simulation for Software In The Loop (SITL) testing
 * 
 * @details This file provides the SITL implementation of the HAL RC input interface,
 *          allowing simulation of various RC protocols and input sources for testing
 *          autopilot behavior without physical RC hardware.
 *          
 *          SITL RC input supports multiple input sources:
 *          - Simulated RC protocols (PPM, SBUS, DSM, SPEKTRUM, CRSF, FPORT, etc.)
 *          - Joystick input mapped to RC channels via MAVProxy or QGroundControl
 *          - MAVLink RC_CHANNELS_OVERRIDE messages from ground station
 *          - Command-line RC override via sim_vehicle.py parameters
 *          - Automated test framework RC commands (autotest)
 *          
 *          The SITL RC input can simulate protocol-specific behaviors including:
 *          - Different frame formats and timing characteristics
 *          - Protocol-specific channel counts and resolution
 *          - Signal loss and failsafe conditions (by stopping RC input)
 *          - Protocol decoding edge cases for testing RC_Channel library
 *          
 *          Typical RC channel mapping (matching standard vehicle firmware):
 *          - Channel 1: Roll (aileron)
 *          - Channel 2: Pitch (elevator)
 *          - Channel 3: Throttle
 *          - Channel 4: Yaw (rudder)
 *          - Channel 5-16: Auxiliary functions (flight modes, camera control, etc.)
 * 
 * @note When AP_RCPROTOCOL_ENABLED is defined, RC input is forwarded through
 *       the AP::RC() singleton to properly test protocol decoding and processing
 *       as it occurs on real hardware.
 * 
 * @warning RC input must be actively provided by the simulation environment,
 *          external control tool (MAVProxy, QGroundControl), or test framework.
 *          If no RC input is provided, the autopilot will detect RC failsafe
 *          conditions just as it would with physical hardware.
 * 
 * @see AP_HAL::RCInput for the abstract HAL interface definition
 * @see AP_RCProtocol for RC protocol decoding implementation
 * @see libraries/RC_Channel for RC channel management and mapping
 * @see Tools/autotest for automated RC input testing examples
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SITL_RC_INPUT_CHANNELS 16

#include "AP_HAL_SITL.h"

/**
 * @class HALSITL::RCInput
 * @brief SITL implementation of RC input interface for autopilot simulation
 * 
 * @details Provides simulated RC (radio control) receiver functionality for SITL,
 *          enabling comprehensive testing of RC input handling, protocol decoding,
 *          failsafe behavior, and pilot input processing without physical hardware.
 *          
 *          This implementation interfaces with the SITL simulation framework to
 *          receive RC input values from various sources including joysticks,
 *          MAVLink override commands, and automated test scripts.
 *          
 *          Integration with simulation:
 *          - Receives RC channel values from SITL simulation state
 *          - Can simulate different RC protocols for protocol testing
 *          - Supports dynamic channel count changes
 *          - Provides realistic timing and update behavior
 *          - Can simulate signal loss for failsafe testing
 *          
 *          When AP_RCPROTOCOL_ENABLED is defined, this class forwards RC data
 *          through the AP::RC() protocol decoder singleton, allowing testing of
 *          the complete RC processing pipeline including protocol-specific parsing,
 *          timing validation, and error detection as occurs on real hardware.
 *          
 *          Usage in SITL:
 *          1. Launch sim_vehicle.py with desired RC input source
 *          2. Connect joystick via MAVProxy: `module load joystick`
 *          3. Or use MAVLink RC override from ground station
 *          4. Or inject RC values via autotest framework
 *          
 *          Failsafe simulation:
 *          To test RC failsafe behavior, stop providing RC input updates.
 *          The autopilot will detect loss of RC input and trigger failsafe
 *          actions according to configuration, identical to physical hardware.
 * 
 * @note This class is instantiated as a singleton by the SITL HAL and accessed
 *       via the hal.rcin interface pointer.
 * 
 * @warning RC input values should be in standard PWM microsecond units (typically
 *          1000-2000μs) matching physical RC receivers. Invalid values may cause
 *          unexpected autopilot behavior or trigger safety checks.
 * 
 * @see AP_HAL::RCInput for parent interface contract
 * @see SITL::SIM for simulation framework integration
 */
class HALSITL::RCInput : public AP_HAL::RCInput {
public:
    /**
     * @brief Construct SITL RC input handler
     * 
     * @details Initializes the RC input interface for SITL simulation.
     *          Actual RC input initialization occurs in init() method.
     */
    explicit RCInput() {}
    
    /**
     * @brief Initialize SITL RC input subsystem
     * 
     * @details Performs one-time initialization of the RC input interface,
     *          setting up connections to the SITL simulation framework for
     *          receiving RC channel data.
     *          
     *          When AP_RCPROTOCOL_ENABLED is defined, this also initializes
     *          the RC protocol decoder (AP::RC()) to enable testing of
     *          protocol-specific parsing and timing behavior.
     *          
     *          Called once during HAL initialization before vehicle setup.
     * 
     * @note This method is called automatically by the SITL HAL during
     *       board initialization and should not be called directly.
     */
    void init() override;
    
    /**
     * @brief Check if new RC input data is available
     * 
     * @details Queries whether RC input values have been updated since the
     *          last time they were read. Used by the vehicle code to determine
     *          if pilot input has changed and requires processing.
     *          
     *          In SITL, this returns true when:
     *          - New RC channel data received from simulation
     *          - MAVLink RC_CHANNELS_OVERRIDE message processed
     *          - Joystick input updated via MAVProxy
     *          - Test framework injected new RC values
     *          
     *          Returns false when no updates available, which may indicate
     *          RC input source stopped providing data (simulating signal loss).
     * 
     * @return true if new RC input data is available for reading
     * @return false if no updates since last read (potential RC failsafe)
     * 
     * @note This method is typically called at main loop rate (50-400Hz
     *       depending on vehicle) to poll for pilot input changes.
     * 
     * @see read() to retrieve updated channel values
     */
    bool new_input() override;
    
    /**
     * @brief Get number of RC channels currently available
     * 
     * @details Returns the count of RC channels being provided by the
     *          current input source. This may vary depending on:
     *          - Simulated RC protocol (PPM=8, SBUS=16, DSM=12, etc.)
     *          - Number of channels in MAVLink RC_CHANNELS_OVERRIDE
     *          - Joystick configuration (number of axes/buttons mapped)
     *          - Test framework configuration
     *          
     *          The vehicle uses this to determine which channels are valid
     *          for reading. Channels beyond this count will return 0.
     *          
     *          Maximum supported: SITL_RC_INPUT_CHANNELS (16)
     * 
     * @return uint8_t Number of valid RC channels (0-16)
     * 
     * @note Channel count may change dynamically if RC input source changes
     *       during simulation (e.g., switching from 8-channel PPM simulation
     *       to 16-channel SBUS simulation).
     * 
     * @see SITL_RC_INPUT_CHANNELS for maximum channel count
     * @see read() to retrieve individual channel values
     */
    uint8_t num_channels() override;
    
    /**
     * @brief Read value of a single RC channel
     * 
     * @details Retrieves the current PWM value (in microseconds) for the
     *          specified RC channel. Values typically range from 1000-2000μs
     *          with 1500μs as neutral/center, matching physical RC receivers.
     *          
     *          Channel mapping follows standard conventions:
     *          - Channel 0: Roll (aileron) - 1000μs=left, 1500μs=center, 2000μs=right
     *          - Channel 1: Pitch (elevator) - 1000μs=forward, 1500μs=center, 2000μs=back
     *          - Channel 2: Throttle - 1000μs=low, 2000μs=high (no neutral point)
     *          - Channel 3: Yaw (rudder) - 1000μs=left, 1500μs=center, 2000μs=right
     *          - Channel 4+: Auxiliary functions (mode switches, camera control, etc.)
     *          
     *          When AP_RCPROTOCOL_ENABLED is defined, this method retrieves
     *          values from AP::RC() protocol decoder, testing the complete
     *          RC processing pipeline.
     * 
     * @param[in] ch Channel number to read (0-based index, 0-15)
     * 
     * @return uint16_t Channel value in microseconds (typically 1000-2000)
     * @return 0 if channel index is invalid or channel not available
     * 
     * @note Channel values should be validated against configured min/max
     *       limits and deadzone settings by RC_Channel library before use.
     * 
     * @warning Reading beyond num_channels() returns 0, which may be
     *          misinterpreted as valid low throttle on channel 2.
     * 
     * @see num_channels() to check valid channel count
     * @see read(uint16_t*, uint8_t) for bulk channel reading
     * @see libraries/RC_Channel for channel processing and mapping
     */
    uint16_t read(uint8_t ch) override;
    
    /**
     * @brief Read multiple RC channels into provided buffer
     * 
     * @details Efficiently retrieves multiple RC channel values in a single
     *          call, copying PWM values (in microseconds) into the provided
     *          array. Useful for bulk processing of RC inputs.
     *          
     *          Copies up to the minimum of:
     *          - Requested length (len parameter)
     *          - Number of available channels (num_channels())
     *          - Buffer capacity (caller responsibility)
     *          
     *          Channels beyond available count are not written to buffer.
     *          
     *          When AP_RCPROTOCOL_ENABLED is defined, this retrieves values
     *          from the AP::RC() protocol decoder.
     * 
     * @param[out] periods Array to receive RC channel values (PWM microseconds)
     * @param[in]  len     Maximum number of channels to read (array size)
     * 
     * @return uint8_t Actual number of channels copied to array
     * @return 0 if no RC channels available or invalid parameters
     * 
     * @note Caller must ensure periods array has capacity for at least len elements.
     *       Reading beyond array bounds causes undefined behavior.
     * 
     * @note This method is more efficient than multiple read(uint8_t) calls
     *       when processing many channels.
     * 
     * @warning Buffer must be large enough for requested length. No bounds
     *          checking is performed. Oversized len may cause memory corruption.
     * 
     * @see read(uint8_t) for single channel reading
     * @see num_channels() to determine available channel count before calling
     */
    uint8_t read(uint16_t* periods, uint8_t len) override;
};

#endif

