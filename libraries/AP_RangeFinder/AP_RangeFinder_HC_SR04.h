/**
 * @file AP_RangeFinder_HC_SR04.h
 * @brief HC-SR04 ultrasonic rangefinder backend driver
 * 
 * This file implements support for the HC-SR04 low-cost ultrasonic distance sensor,
 * a popular hobby-grade rangefinder that uses PWM pulse width measurement for
 * distance determination.
 * 
 * The HC-SR04 operates by:
 * 1. Sending a 10μs trigger pulse on a GPIO pin
 * 2. Receiving an echo pulse whose width is proportional to distance
 * 3. Converting pulse width to distance using the speed of sound
 * 
 * Distance calculation: distance_m = (pulse_width_us * speed_of_sound) / 2
 * The division by 2 accounts for the round-trip travel time of the ultrasonic pulse.
 * 
 * Hardware Interface:
 * - Trigger Pin: GPIO output for initiating measurement
 * - Echo Pin: PWM input for measuring return pulse width
 * 
 * Typical Range: 2cm to 400cm (depending on target reflectivity)
 * Update Rate: Approximately 50Hz maximum
 * 
 * @note This is a low-cost hobby sensor with limited accuracy and reliability
 *       compared to professional-grade rangefinders. Best suited for non-critical
 *       obstacle detection and altitude hold in controlled environments.
 * 
 * @warning Not suitable for safety-critical applications like precision landing
 *          or collision avoidance due to:
 *          - Sensitivity to acoustic noise
 *          - Narrow beam width (~15 degrees)
 *          - Limited range and accuracy
 *          - Susceptibility to glitches from surface angles
 * 
 * @see AP_RangeFinder_Backend for base class interface
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_HC_SR04_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include "AP_RangeFinder_Params.h"

/**
 * @class AP_RangeFinder_HC_SR04
 * @brief Backend driver for HC-SR04 PWM ultrasonic rangefinder
 * 
 * @details This backend implements distance measurement using the HC-SR04 ultrasonic
 *          sensor, which measures distance by timing the width of a PWM echo pulse.
 *          
 *          Measurement Cycle:
 *          1. Driver asserts trigger pin HIGH for 10μs
 *          2. Sensor emits 8-cycle ultrasonic burst at 40kHz
 *          3. Sensor sets echo pin HIGH when pulse transmitted
 *          4. Sensor sets echo pin LOW when echo received
 *          5. Driver measures echo pulse width using PWMSource
 *          6. Distance = (pulse_width_us * 343m/s) / 2 / 1000000
 *          
 *          Glitch Filtering:
 *          The driver implements glitch detection to filter spurious readings
 *          caused by acoustic reflections, narrow beam angles, or electrical noise.
 *          Multiple consecutive outliers are required before updating the reported
 *          distance to prevent sudden jumps in readings.
 *          
 *          Hardware Requirements:
 *          - One GPIO pin for trigger output
 *          - One timer-capable pin for PWM echo input
 *          - 5V power supply (some variants work with 3.3V)
 *          
 *          Typical Applications:
 *          - Non-critical altitude hold over flat terrain
 *          - Obstacle detection in calm environments
 *          - Educational and hobby applications
 *          - Indoor navigation with good acoustic conditions
 *          
 * @note The HC-SR04 is highly sensitive to environmental conditions including
 *       temperature (affects speed of sound), humidity, and ambient acoustic noise.
 *       
 * @warning Do not use as the primary rangefinder for:
 *          - Precision landing operations
 *          - Terrain following over complex surfaces
 *          - Flight over water (poor acoustic reflection)
 *          - High-speed flight (limited update rate)
 */
class AP_RangeFinder_HC_SR04 : public AP_RangeFinder_Backend
{
public:
    /**
     * @brief Construct a new HC-SR04 rangefinder backend instance
     * 
     * @details Initializes the HC-SR04 driver with the provided state and parameter
     *          objects. The constructor sets up initial values but does not configure
     *          hardware pins - that occurs during the detect() and subsequent pin
     *          initialization phase.
     * 
     * @param[in,out] _state   Reference to RangeFinder state structure for storing
     *                          distance measurements and status
     * @param[in]     _params  Reference to RangeFinder parameters including pin
     *                          assignments and configuration
     * 
     * @note Constructor does not perform hardware initialization. Pin configuration
     *       and PWM source setup occur in check_pins() called from update().
     * 
     * @see detect() for hardware detection logic
     * @see check_pins() for pin initialization
     */
    AP_RangeFinder_HC_SR04(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    /**
     * @brief Detect if HC-SR04 hardware is properly configured
     * 
     * @details Static detection function that validates the HC-SR04 configuration
     *          by checking if the required pins are properly specified in parameters.
     *          
     *          Detection Criteria:
     *          - Echo pin must be configured (via RNGFND_PIN parameter)
     *          - Pin must be valid and available for PWM input
     *          
     *          This function does not communicate with the sensor hardware; it only
     *          validates that the configuration is sufficient to attempt driver
     *          initialization.
     * 
     * @param[in] _params  Reference to RangeFinder parameters to validate
     * 
     * @return true if pins are properly configured and HC-SR04 can be initialized
     * @return false if configuration is invalid or pins unavailable
     * 
     * @note Called by the rangefinder backend factory during sensor initialization
     * @note Does not verify that an actual HC-SR04 sensor is connected
     * 
     * @see check_pins() for runtime pin validation
     */
    static bool detect(AP_RangeFinder_Params &_params);

    /**
     * @brief Update rangefinder state with latest distance measurement
     * 
     * @details Main update function called periodically by the scheduler. Performs
     *          the following operations:
     *          
     *          1. Validates pin configuration (first call only)
     *          2. Sends trigger pulse if sufficient time has elapsed
     *          3. Reads PWM pulse width from echo pin via PWMSource
     *          4. Converts pulse width to distance in meters
     *          5. Applies glitch filtering to reject spurious readings
     *          6. Updates rangefinder state with filtered distance
     *          
     *          PWM to Distance Conversion:
     *          pulse_width_us = PWMSource measurement (microseconds)
     *          distance_m = (pulse_width_us * 0.000343) / 2
     *          where 0.000343 = speed of sound in m/μs at sea level, 20°C
     *          
     *          Update Rate Control:
     *          Trigger pulses are rate-limited to prevent echo interference.
     *          Typical trigger interval is 20ms (50Hz maximum update rate).
     *          
     *          Glitch Filtering:
     *          Sudden changes in distance are counted as potential glitches.
     *          Multiple consecutive similar readings are required before
     *          updating the reported distance to filter acoustic reflections
     *          and electrical noise.
     * 
     * @note Called at scheduler rate (typically 20Hz for rangefinders)
     * @note Distance calculation assumes standard atmospheric conditions
     *       (temperature ~20°C, sea level pressure)
     * 
     * @warning Speed of sound varies with temperature (~0.6 m/s per °C).
     *          Extreme temperatures will introduce systematic distance errors.
     * 
     * @see check_pins() for pin initialization on first call
     * @see check_trigger_pin() for trigger pulse generation
     * @see check_echo_pin() for PWM measurement reading
     */
    void update(void) override;

protected:

    /**
     * @brief Get MAVLink distance sensor type identifier
     * 
     * @details Returns the MAVLink enum identifying this as an ultrasonic distance
     *          sensor for telemetry reporting to ground control stations.
     *          
     *          This information is used in MAVLink DISTANCE_SENSOR messages to
     *          inform the GCS about the type of rangefinder hardware in use.
     * 
     * @return MAV_DISTANCE_SENSOR_ULTRASOUND indicating ultrasonic sensor type
     * 
     * @note Overrides AP_RangeFinder_Backend virtual method
     * @see MAVLink DISTANCE_SENSOR message definition
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:

    /**
     * @brief Initialize and validate GPIO pins for HC-SR04 operation
     * 
     * @details Performs one-time initialization of trigger and echo pins on first
     *          call to update(). Validates that:
     *          - Echo pin is properly configured for PWM input
     *          - Trigger pin (if specified) is available for GPIO output
     *          
     *          For echo pin: Initializes PWMSource to measure pulse width
     *          For trigger pin: Configures GPIO as output and sets initial LOW state
     *          
     *          Called once during the first update() cycle. If initialization fails,
     *          subsequent calls will retry until successful or sensor is disabled.
     * 
     * @return true if both pins successfully initialized
     * @return false if pin configuration failed (sensor will not function)
     * 
     * @note Called automatically by update() on first execution
     * @see check_echo_pin() for echo pin specific initialization
     * @see check_trigger_pin() for trigger pin management
     */
    bool check_pins();

    /**
     * @brief Initialize and validate echo pin for PWM pulse width measurement
     * 
     * @details Configures the echo pin for PWM input capture using AP_HAL::PWMSource.
     *          The echo pin receives a pulse from the HC-SR04 whose width is
     *          proportional to the measured distance.
     *          
     *          Pin Requirements:
     *          - Must support timer input capture or equivalent PWM measurement
     *          - Specified via RNGFND_PIN parameter
     *          
     *          PWMSource Configuration:
     *          - Measures positive pulse width (HIGH duration)
     *          - Captures pulse width in microseconds
     *          - Filters invalid/noise pulses automatically
     * 
     * @return true if echo pin successfully initialized for PWM measurement
     * @return false if pin is invalid or PWMSource setup failed
     * 
     * @note Called by check_pins() during initialization
     * @see AP_HAL::PWMSource for pulse width measurement implementation
     */
    bool check_echo_pin();

    /**
     * @brief Generate trigger pulse to initiate HC-SR04 measurement cycle
     * 
     * @details Sends a 10μs HIGH pulse on the trigger pin to command the HC-SR04
     *          to begin a new distance measurement. The sensor responds by:
     *          1. Transmitting an 8-cycle 40kHz ultrasonic burst
     *          2. Raising the echo pin HIGH
     *          3. Lowering the echo pin when echo returns (or timeout occurs)
     *          
     *          Trigger Pulse Timing:
     *          - Pulse width: 10 microseconds minimum (HC-SR04 specification)
     *          - Trigger rate: Limited to prevent echo interference (~50Hz max)
     *          
     *          Rate Limiting:
     *          Triggers are throttled using last_ping_ms to ensure sufficient time
     *          for the previous measurement to complete and echoes to dissipate.
     *          Typical minimum interval is 20ms between triggers.
     * 
     * @note Only generates trigger pulse if sufficient time has elapsed since last ping
     * @note If trigger pin is not configured, sensor must be triggered externally
     * 
     * @warning Triggering too rapidly can cause echo interference and false readings
     * 
     * @see last_ping_ms for trigger rate control
     */
    void check_trigger_pin();

    /**
     * @brief GPIO pin number for ultrasonic trigger output
     * 
     * @details Pin used to send the 10μs trigger pulse that initiates each
     *          HC-SR04 measurement cycle. Set to -1 if no trigger pin is configured
     *          (external triggering required).
     *          
     *          Configured via RNGFND_TRIG_PIN parameter (if supported by HAL).
     *          
     * @note Set to -1 if trigger pin not used (sensor triggered externally)
     * @note Must be a valid GPIO pin capable of digital output
     */
    int8_t trigger_pin;

    /**
     * @brief Last valid distance measurement in meters
     * 
     * @details Stores the most recently accepted distance reading after glitch
     *          filtering. Used for comparison with new measurements to detect
     *          sudden jumps that may indicate spurious readings.
     *          
     *          Glitch Detection Algorithm:
     *          If new reading differs significantly from last_distance_m,
     *          increment glitch_count. Only update last_distance_m when
     *          multiple consecutive similar readings confirm a real change.
     *          
     *          This filtering prevents sensor glitches from:
     *          - Acoustic reflections off narrow or angled surfaces
     *          - Electrical noise on echo pin
     *          - Cross-talk from nearby ultrasonic sensors
     *          
     * @note Updated only after glitch filtering confirms measurement validity
     * @see glitch_count for glitch detection logic
     */
    float last_distance_m;

    /**
     * @brief Consecutive glitch counter for filtering spurious measurements
     * 
     * @details Counts the number of consecutive measurements that differ
     *          significantly from the last accepted distance. Used to implement
     *          hysteresis in distance updates to reject transient glitches while
     *          still responding to genuine distance changes.
     *          
     *          Glitch Filtering Logic:
     *          - If (new_reading ~= last_distance_m): glitch_count = 0
     *          - If (new_reading differs): glitch_count++
     *          - If (glitch_count > threshold): Accept new reading as valid
     *          
     *          This provides a balance between:
     *          - Rejecting single spurious readings (noise immunity)
     *          - Responding to real distance changes (tracking ability)
     *          
     *          Typical threshold: 2-3 consecutive outliers required before update
     * 
     * @note Reset to zero when new reading matches expected distance
     * @see last_distance_m for reference distance comparison
     */
    uint8_t glitch_count;

    /**
     * @brief PWM pulse width measurement source for echo pin
     * 
     * @details HAL PWMSource object that measures the width of the echo pulse
     *          received from the HC-SR04 sensor. The pulse width is directly
     *          proportional to the time-of-flight of the ultrasonic pulse.
     *          
     *          Measurement Process:
     *          1. PWMSource captures rising edge of echo pulse (start time)
     *          2. PWMSource captures falling edge of echo pulse (end time)
     *          3. Pulse width = (end_time - start_time) in microseconds
     *          4. Distance = (pulse_width * speed_of_sound) / 2
     *          
     *          PWMSource Features:
     *          - Hardware timer-based measurement for accuracy
     *          - Automatic filtering of incomplete/invalid pulses
     *          - Thread-safe access to pulse width data
     *          
     *          Expected Pulse Width Range:
     *          - Minimum: ~116 μs (2cm minimum range)
     *          - Maximum: ~23,200 μs (400cm maximum range)
     *          - Out-of-range: No echo or invalid reading
     * 
     * @note Initialized by check_echo_pin() during pin setup
     * @note Pulse width measured in microseconds
     * 
     * @see AP_HAL::PWMSource for implementation details
     * @see check_echo_pin() for initialization
     */
    AP_HAL::PWMSource pwm_source;

    /**
     * @brief Timestamp of last trigger pulse in milliseconds
     * 
     * @details Records system time (in milliseconds) when the most recent trigger
     *          pulse was sent. Used to implement rate limiting of trigger pulses
     *          to prevent echo interference and allow time for previous measurements
     *          to complete.
     *          
     *          Rate Limiting Logic:
     *          Only send new trigger pulse if:
     *          (current_time_ms - last_ping_ms) >= minimum_interval
     *          
     *          Typical minimum interval: 20ms (50Hz maximum ping rate)
     *          
     *          This prevents:
     *          - Echo interference from overlapping measurements
     *          - Electrical noise from rapid pin toggling
     *          - CPU overhead from excessive update rates
     *          
     * @note Time in milliseconds from AP_HAL::millis()
     * @note Updated each time check_trigger_pin() sends a trigger pulse
     * 
     * @see check_trigger_pin() for trigger pulse generation and rate control
     */
    uint32_t last_ping_ms;
};

#endif  // AP_RANGEFINDER_HC_SR04_ENABLED
