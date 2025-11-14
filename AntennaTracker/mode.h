/**
 * @file mode.h
 * @brief Flight mode class hierarchy for antenna tracker
 * 
 * @details Defines the Mode base class and all concrete mode implementations
 *          for the AntennaTracker vehicle. Modes control how the antenna tracker
 *          points the antenna servos - from manual RC control to automatic vehicle
 *          tracking with intelligent fallback scanning.
 *          
 *          Available modes: Manual, Stop, Scan, ServoTest, Guided, Auto, Initialising
 *          
 * @note All modes must inherit from Mode base class and implement the pure virtual
 *       interface methods
 * 
 * Source: AntennaTracker/mode.h:1-111
 */

#pragma once

#include <stdint.h>
#include <AP_Math/AP_Math.h>

/**
 * @class Mode
 * @brief Abstract base class for all antenna tracker modes
 * 
 * @details Defines the interface that all concrete tracker modes must implement.
 *          Each mode controls how the tracker servos are commanded based on:
 *          - RC pilot input (Manual mode)
 *          - Automatic vehicle tracking (Auto mode)
 *          - External MAVLink commands (Guided mode)
 *          - Scanning patterns (Scan mode)
 *          - Fixed position (Stop mode)
 *          - Direct servo testing (ServoTest mode)
 *          
 *          The Mode base class provides protected helper methods for coordinate
 *          transformations and angle calculations used by multiple mode implementations.
 *          
 *          Mode lifecycle: Instantiated at startup, update() called at 50Hz when active.
 * 
 * @note This is a pure virtual class and cannot be instantiated directly
 * @see ModeAuto, ModeGuided, ModeManual, ModeScan, ModeStop, ModeServoTest, ModeInitialising
 */
class Mode {
public:
    /**
     * @enum Mode::Number
     * @brief Mode identifier constants for antenna tracker
     * 
     * @details Unique numeric identifiers for each mode, used in MAVLink messages,
     *          parameter storage, and mode switching logic. Mode numbers must remain
     *          stable for ground station compatibility.
     */
    enum class Number {
        MANUAL=0,        ///< Direct RC control of servos - pilot commands pan/tilt via RC inputs
        STOP=1,          ///< Hold current position - maintains servo positions, no tracking movement
        SCAN=2,          ///< Sweep search pattern - scans through yaw/pitch ranges to locate vehicle
        SERVOTEST=3,     ///< Direct PWM commands from GCS - allows specific PWM values for testing/calibration
        GUIDED=4,        ///< External attitude control via MAVLink SET_ATTITUDE_TARGET - for companion computer control
        AUTO=10,         ///< Automatic vehicle tracking with scan fallback - primary operational mode
        INITIALISING=16  ///< Startup mode during tracker initialization - no servo movement until ready
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    Mode() {}

    // do not allow copying
    CLASS_NO_COPY(Mode);

    /**
     * @brief Returns the unique numeric identifier for this mode
     * 
     * @return Mode::Number The mode identifier constant
     * 
     * @note Used for MAVLink reporting and mode switching logic
     */
    virtual Mode::Number number() const = 0;
    
    /**
     * @brief Returns the human-readable name of this mode
     * 
     * @return const char* Mode name string (e.g., "Auto", "Manual", "Scan")
     * 
     * @note Used for logging and ground station display
     */
    virtual const char* name() const = 0;

    /**
     * @brief Indicates whether servos must be armed for this mode to operate
     * 
     * @return true if servos must be armed (modes that move servos)
     * @return false if mode can operate without armed servos (Stop, Initialising)
     * 
     * @note Affects arming check logic and safety interlocks
     */
    virtual bool requires_armed_servos() const = 0;

    /**
     * @brief Main mode update function called at 50Hz
     * 
     * @details Implements the mode-specific behavior by calculating desired servo
     *          positions and commanding servo outputs. Each mode implements its own
     *          tracking/control algorithm in this method.
     * 
     * @note Called by main scheduler loop at 50Hz when this mode is active
     * @note Concrete implementations must not block - execution must complete quickly
     */
    virtual void update() = 0;

protected:
    /**
     * @brief Implements scanning search pattern algorithm
     * 
     * @details Sweeps antenna through configured yaw and pitch ranges to search for
     *          tracked vehicle. Used by SCAN mode and as fallback in AUTO mode when
     *          vehicle position is lost or invalid.
     *          
     *          Scanning pattern: sweeps yaw left-to-right at current pitch, then
     *          increments pitch and reverses yaw direction.
     * 
     * @note Called at 50Hz during SCAN mode or AUTO fallback
     * @see ModeScan, ModeAuto
     */
    void update_scan();
    
    /**
     * @brief Implements automatic vehicle tracking algorithm
     * 
     * @details Calculates antenna pointing angles to track target vehicle based on
     *          vehicle GPS position, tracker position, and vehicle altitude. Falls
     *          back to scanning if vehicle position becomes invalid.
     *          
     *          Uses earth-frame to body-frame coordinate transformations to handle
     *          tracker orientation and azimuth limits.
     * 
     * @note Called at 50Hz during AUTO mode
     * @note Primary operational mode for antenna tracking
     * @see ModeAuto
     */
    void update_auto();

    /**
     * @brief Determines yaw direction reversal for azimuth limit handling
     * 
     * @details Calculates whether yaw angle needs to be reversed to handle tracker
     *          mechanical azimuth limits (typically 0-360° or -180° to +180°).
     *          Required when tracker cannot rotate full 360° continuously.
     * 
     * @return true if yaw direction should be reversed
     * @return false if normal yaw direction
     * 
     * @note Used for trackers with limited azimuth range
     */
    bool get_ef_yaw_direction();

    /**
     * @brief Calculates tracking angle errors for servo control
     * 
     * @details Computes difference between current antenna pointing angles and
     *          desired target angles, accounting for direction reversal. Result
     *          used for servo position commands and rate limiting.
     * 
     * @param[in] pitch Target pitch angle in degrees
     * @param[in] yaw Target yaw angle in degrees
     * @param[in] direction_reversed True if yaw direction is reversed for limits
     * 
     * @note Angle errors stored internally for servo command generation
     * @note Called during tracking calculations in AUTO and GUIDED modes
     */
    void calc_angle_error(float pitch, float yaw, bool direction_reversed);
    
    /**
     * @brief Converts earth frame angles to body frame angles
     * 
     * @details Transforms pointing angles from earth-fixed frame (NED - North-East-Down)
     *          to tracker body frame, accounting for tracker orientation. Required
     *          because vehicle position is in earth frame but servos operate in body frame.
     * 
     * @param[in]  pitch Earth frame pitch angle in degrees
     * @param[in]  yaw Earth frame yaw angle in degrees
     * @param[out] bf_pitch Body frame pitch angle in degrees
     * @param[out] bf_yaw Body frame yaw angle in degrees
     * 
     * @note Coordinate frames: Earth frame is NED (North-East-Down), body frame is tracker-relative
     */
    void convert_ef_to_bf(float pitch, float yaw, float& bf_pitch, float& bf_yaw);
    
    /**
     * @brief Converts body frame angles to earth frame angles
     * 
     * @details Transforms pointing angles from tracker body frame to earth-fixed frame
     *          (NED - North-East-Down). Inverse of convert_ef_to_bf(). Used when
     *          reporting antenna orientation in earth-relative coordinates.
     * 
     * @param[in]  pitch Body frame pitch angle in degrees
     * @param[in]  yaw Body frame yaw angle in degrees
     * @param[out] ef_pitch Earth frame pitch angle in degrees
     * @param[out] ef_yaw Earth frame yaw angle in degrees
     * 
     * @return true if conversion successful
     * @return false if conversion failed (invalid angles or undefined orientation)
     * 
     * @note Coordinate frames: Body frame is tracker-relative, earth frame is NED
     */
    bool convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw);
};

/**
 * @class ModeAuto
 * @brief Automatic vehicle tracking with intelligent fallback to scanning
 * 
 * @details Primary operational mode for antenna tracking. Automatically points
 *          antenna at tracked vehicle using GPS position data. When vehicle
 *          position becomes invalid or lost, automatically falls back to scanning
 *          pattern to reacquire the vehicle.
 *          
 *          Tracking algorithm:
 *          1. Receive vehicle position via MAVLink GPS messages
 *          2. Calculate required antenna pointing angles (pitch/yaw)
 *          3. Transform to body frame and command servos
 *          4. If position lost > timeout: switch to scanning pattern
 *          5. When vehicle reacquired: resume automatic tracking
 * 
 * @note This is the recommended mode for operational antenna tracking
 * @note Requires armed servos and valid vehicle position data
 * @note Falls back to scan pattern automatically when vehicle position lost
 * 
 * @see update_auto(), update_scan()
 */
class ModeAuto : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::AUTO; }
    const char* name() const override { return "Auto"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

/**
 * @class ModeGuided
 * @brief External attitude control via MAVLink SET_ATTITUDE_TARGET messages
 * 
 * @details Allows external control of antenna pointing by companion computer or
 *          ground station via MAVLink. Attitude targets are specified as quaternions
 *          and converted to servo commands. Supports both absolute attitude targets
 *          and yaw rate commands.
 *          
 *          Control flow:
 *          1. External system sends MAVLink SET_ATTITUDE_TARGET message
 *          2. Message handler calls set_angle() with target quaternion
 *          3. update() converts quaternion to pitch/yaw angles
 *          4. Angles transformed to body frame and sent to servos
 * 
 * @note Requires armed servos
 * @note Used for companion computer control and advanced automation
 * @note Target attitude must be updated regularly (typically 10Hz minimum)
 * 
 * @see set_angle()
 */
class ModeGuided : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::GUIDED; }
    const char* name() const override { return "Guided"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;

    /**
     * @brief Set target attitude for antenna pointing
     * 
     * @details Called by MAVLink message handler when SET_ATTITUDE_TARGET received.
     *          Stores target quaternion attitude and optional yaw rate for conversion
     *          to servo commands during update() execution.
     * 
     * @param[in] target_att Target attitude quaternion (earth frame reference)
     * @param[in] use_yaw_rate True to use yaw_rate_rads instead of attitude yaw
     * @param[in] yaw_rate_rads Yaw rate command in radians/second (if use_yaw_rate true)
     * 
     * @note Must be called regularly (10Hz+) to maintain control
     * @note Quaternion represents desired antenna orientation in earth frame
     */
    void set_angle(const Quaternion &target_att, bool use_yaw_rate, float yaw_rate_rads) {
        _target_att = target_att;
        _use_yaw_rate = use_yaw_rate;
        _yaw_rate_rads = yaw_rate_rads;
    }

private:
    Quaternion _target_att;     ///< Target attitude quaternion from MAVLink command
    bool _use_yaw_rate;         ///< True to use yaw rate instead of absolute yaw angle
    float _yaw_rate_rads;       ///< Commanded yaw rate in radians/second
};

/**
 * @class ModeInitialising
 * @brief Startup mode active during tracker initialization
 * 
 * @details Temporary mode used during tracker boot sequence. Prevents servo
 *          movement while hardware initialization, sensor calibration, and system
 *          checks are in progress. Automatically transitions to configured initial
 *          mode (typically AUTO or MANUAL) once initialization completes.
 *          
 *          Initialization sequence:
 *          1. Tracker boots in INITIALISING mode
 *          2. Hardware/sensor initialization runs
 *          3. Pre-arm checks execute
 *          4. Mode switches to configured initial mode
 * 
 * @note Does not require armed servos - no movement commanded
 * @note Automatically exits to initial mode after successful initialization
 * @note update() is empty - no servo commands while initializing
 */
class ModeInitialising : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::INITIALISING; }
    const char* name() const override { return "Initialising"; }
    bool requires_armed_servos() const override { return false; }
    void update() override {};
};

/**
 * @class ModeManual
 * @brief Direct RC control of antenna servos
 * 
 * @details Provides direct manual control of antenna pointing via RC transmitter.
 *          RC input channels are mapped directly to servo PWM outputs, allowing
 *          pilot to manually aim the antenna. Useful for initial setup, testing,
 *          calibration, and manual operation when automatic tracking unavailable.
 *          
 *          Control mapping (typical):
 *          - RC yaw channel → Yaw servo (azimuth/pan)
 *          - RC pitch channel → Pitch servo (elevation/tilt)
 *          
 *          RC inputs are scaled, limited, and converted to servo commands.
 * 
 * @note Requires armed servos for safety
 * @note Useful for setup, testing, and manual operation
 * @note RC failsafe applies - servo positions held on RC signal loss
 */
class ModeManual : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::MANUAL; }
    const char* name() const override { return "Manual"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

/**
 * @class ModeScan
 * @brief Scanning search pattern mode for vehicle acquisition
 * 
 * @details Sweeps antenna through configured yaw and pitch ranges using systematic
 *          search pattern to locate and acquire tracked vehicle. Used to find vehicle
 *          when position unknown or when AUTO mode loses tracking. Scanning continues
 *          until vehicle acquired or mode manually changed.
 *          
 *          Scan pattern algorithm:
 *          1. Start at minimum yaw and pitch angles
 *          2. Sweep yaw from min to max at current pitch (or max to min)
 *          3. Increment pitch by configured step
 *          4. Reverse yaw direction and sweep again
 *          5. Repeat until max pitch reached, then restart from min pitch
 *          
 *          Scan ranges configured via SCAN_YAW_MIN, SCAN_YAW_MAX, SCAN_PITCH_MIN,
 *          SCAN_PITCH_MAX parameters.
 * 
 * @note Requires armed servos
 * @note Used for initial vehicle acquisition and reacquisition after loss
 * @note Also used as automatic fallback in AUTO mode
 * 
 * @see update_scan()
 */
class ModeScan : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::SCAN; }
    const char* name() const override { return "Scan"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

/**
 * @class ModeServoTest
 * @brief Direct servo PWM test mode for calibration and diagnostics
 * 
 * @details Allows ground control station to command specific PWM values directly
 *          to servos for testing, calibration, and range verification. Bypasses
 *          normal angle calculations and control loops - PWM values sent directly
 *          to servo outputs.
 *          
 *          Typical use cases:
 *          - Servo mechanical range calibration
 *          - Servo direction verification
 *          - Servo response testing
 *          - Hardware troubleshooting
 *          - PWM output verification
 *          
 *          GCS sends MAVLink DO_SET_SERVO commands with servo number and PWM value.
 * 
 * @warning Only for ground testing and calibration
 * @warning Do not use during flight or normal operations
 * @note Requires armed servos
 * @note PWM values typically range 1000-2000 microseconds
 * 
 * @see set_servo()
 */
class ModeServoTest : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::SERVOTEST; }
    const char* name() const override { return "ServoTest"; }
    bool requires_armed_servos() const override { return true; }
    void update() override {};

    /**
     * @brief Set specific PWM value for a servo channel
     * 
     * @details Called by MAVLink DO_SET_SERVO command handler. Directly commands
     *          specified PWM value to servo output, bypassing normal control loops.
     *          Used for calibration and testing only.
     * 
     * @param[in] servo_num Servo output number (typically 1=yaw, 2=pitch)
     * @param[in] pwm PWM pulse width in microseconds (typically 1000-2000)
     * 
     * @return true if servo command accepted and sent
     * @return false if servo_num invalid or command rejected
     * 
     * @warning Only use for ground testing - direct PWM commands bypass safety limits
     * @note PWM values outside configured servo min/max will be limited
     */
    bool set_servo(uint8_t servo_num, uint16_t pwm);
};

/**
 * @class ModeStop
 * @brief Hold current position mode - no tracking movement
 * 
 * @details Maintains antenna at current servo positions with no tracking or movement.
 *          Servos hold their last commanded positions. Used when tracking should be
 *          paused while keeping tracker powered and ready.
 *          
 *          Use cases:
 *          - Pause tracking temporarily
 *          - Hold position during setup or configuration
 *          - Maintain current aim point without updates
 *          - Safe mode when tracking not needed
 * 
 * @note Does not require armed servos - position holding is passive
 * @note Servos maintain last commanded position (no active control)
 * @note update() is empty - no new servo commands generated
 * @note Different from Manual mode - no RC input processed in Stop
 */
class ModeStop : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::STOP; }
    const char* name() const override { return "Stop"; }
    bool requires_armed_servos() const override { return false; }
    void update() override {};
};
