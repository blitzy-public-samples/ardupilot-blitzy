/**
 * @file AP_MotorsHeli_Dual.h
 * @brief Motor control class for dual main rotor helicopters (tandem, transverse, intermeshing)
 * 
 * @details This file implements motor control and mixing logic for helicopters with two main rotors.
 *          Supports three dual rotor configurations:
 *          - Tandem: Two rotors mounted longitudinally (front and rear), e.g., CH-47 Chinook
 *          - Transverse: Two rotors mounted laterally (side by side)
 *          - Intermeshing: Two rotors with overlapping blade paths, counter-rotating to avoid collision
 *          
 *          Each rotor has its own swashplate (with up to 3 servos each) and rotor speed controller (RSC).
 *          Control is achieved through differential collective pitch and cyclic inputs to both rotors.
 *          
 *          Key control mechanisms:
 *          - Pitch control: Differential collective between front/rear rotors (tandem mode)
 *          - Roll control: Differential cyclic between left/right rotors (transverse mode)
 *          - Yaw control: Differential rotor torque via speed or collective pitch differences
 *          - Collective: Synchronized collective pitch on both rotors for vertical thrust
 * 
 * @author Fredrik Hedberg
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/AP_Motors/AP_MotorsHeli_Dual.h
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"
#include "AP_MotorsHeli_Swash.h"

/**
 * @defgroup DualRotorModes Dual Rotor Configuration Modes
 * @{
 */
/** @brief Tandem mode - rotors mounted longitudinally (front and aft), used for pitch control via differential collective */
#define AP_MOTORS_HELI_DUAL_MODE_TANDEM                0
/** @brief Transverse mode - rotors mounted laterally (side by side), used for roll control via differential cyclic */
#define AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE            1
/** @brief Intermeshing mode - rotors side by side with overlapping blade paths, requires precise phase synchronization */
#define AP_MOTORS_HELI_DUAL_MODE_INTERMESHING          2
/** @} */

/**
 * @brief Default differential-collective-pitch scaler
 * @details Controls the strength of pitch response from differential collective in tandem mode.
 *          Lower values provide gentler pitch response, higher values provide more aggressive pitch control.
 */
#define AP_MOTORS_HELI_DUAL_DCP_SCALER             0.25f

/**
 * @brief Maximum number of swashplate servos for dual rotor configuration
 * @details Each rotor typically has 3 swashplate servos, so 6 total for both rotors
 */
#define AP_MOTORS_HELI_DUAL_NUM_SWASHPLATE_SERVOS    6

/**
 * @brief Default minimum collective PWM for the rear/second swashplate
 * @details PWM value in microseconds representing minimum collective pitch
 */
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MIN 1250

/**
 * @brief Default maximum collective PWM for the rear/second swashplate
 * @details PWM value in microseconds representing maximum collective pitch
 */
#define AP_MOTORS_HELI_DUAL_COLLECTIVE2_MAX 1750

/**
 * @class AP_MotorsHeli_Dual
 * @brief Motor control and mixing for dual main rotor helicopters
 * 
 * @details This class implements motor control for helicopters with two main rotors, supporting
 *          tandem (CH-47 style), transverse, and intermeshing configurations. Each rotor has its
 *          own swashplate with up to 3 servos and an independent rotor speed controller (RSC).
 * 
 * **Architecture**:
 * - Two swashplates (_swashplate1, _swashplate2) for independent rotor control
 * - Two RSC controllers for managing rotor speeds
 * - Differential control mixing distributes pitch, roll, yaw commands across both rotors
 * - Mode-specific mixing functions (mix_tandem, mix_transverse, mix_intermeshing)
 * 
 * **Control Mechanisms by Mode**:
 * 
 * *Tandem Mode* (rotors front and rear):
 * - Pitch: Differential collective between front and rear rotors
 * - Roll: Synchronized lateral cyclic on both rotors
 * - Yaw: Differential rotor torque via speed or collective differences
 * - Collective: Synchronized vertical thrust on both rotors
 * 
 * *Transverse Mode* (rotors side by side):
 * - Pitch: Synchronized longitudinal cyclic on both rotors
 * - Roll: Differential cyclic between left and right rotors
 * - Yaw: Differential rotor torque via speed or collective differences
 * - Collective: Synchronized vertical thrust on both rotors
 * 
 * *Intermeshing Mode* (overlapping counter-rotating rotors):
 * - Requires precise rotor phase synchronization to prevent blade strikes
 * - Uses differential collective and cyclic similar to transverse
 * - Critical phase angle parameter (H_PHASE_ANGLE) prevents collisions
 * 
 * **Key Parameters**:
 * - H_DUAL_MODE: Selects rotor configuration (0=Tandem, 1=Transverse, 2=Intermeshing)
 * - H_DCP_SCALER: Differential collective-to-pitch scaling (tandem mode)
 * - H_YAW_SCALER: Yaw response scaling via differential torque
 * - H_COL_MIN/MAX: Collective pitch limits for primary rotor
 * - H_COL2_MIN/MAX: Collective pitch limits for secondary rotor
 * - H_DCP_TRIM: Differential collective trim for hover balance
 * - H_DCP_YAW: Yaw feed-forward when differential collective applied
 * - H_YAW_REV_EXPO: Yaw reverser smoothing (intermeshing mode)
 * 
 * **Swashplate Configuration**:
 * - Swashplate 1: Typically front rotor (tandem) or left rotor (transverse)
 * - Swashplate 2: Typically rear rotor (tandem) or right rotor (transverse)
 * - Each swashplate supports 3-axis control (collective, longitudinal cyclic, lateral cyclic)
 * 
 * @note Rotor phasing in intermeshing mode is critical - incorrect settings can cause blade strikes
 * @note Both swashplates must be properly configured and tested before first flight
 * @note Differential limits must be set to prevent mechanical binding or excessive servo loads
 * 
 * @warning Incorrect rotor phasing in intermeshing configuration can result in catastrophic rotor collision
 * @warning Excessive differential collective can cause one rotor to stall or over-speed
 * @warning Both RSC controllers must be properly configured before attempting flight
 * @warning Servo mechanical limits must be validated - software limits may not match hardware range
 * 
 * @see AP_MotorsHeli Base class for common helicopter motor functions
 * @see AP_MotorsHeli_Swash Swashplate mixing and servo control
 * @see AP_MotorsHeli_RSC Rotor speed controller
 */
class AP_MotorsHeli_Dual : public AP_MotorsHeli {
public:
    /**
     * @brief Constructor for dual rotor helicopter motor controller
     * 
     * @details Initializes the dual rotor helicopter motor control system with two independent
     *          swashplates and RSC controllers. Sets up default parameters for differential
     *          collective pitch, yaw scaling, and rotor configuration mode.
     * 
     * @param[in] speed_hz Servo update rate in Hz (default: AP_MOTORS_HELI_SPEED_DEFAULT, typically 125Hz)
     *                     Higher rates provide smoother servo movement but increase processor load
     * 
     * @note This constructor also initializes AP_Param defaults for all dual-heli parameters
     * @note Both swashplates are initialized with their default servo channel assignments
     */
    AP_MotorsHeli_Dual(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };


    /**
     * @brief Set servo update rate for both swashplates and RSC controllers
     * 
     * @details Configures the PWM output rate for all servos controlling both swashplates
     *          and both rotor speed controllers. Higher rates provide smoother servo motion
     *          but increase CPU load. Rate must be supported by servo hardware.
     * 
     * @param[in] speed_hz Desired servo update rate in Hz
     *                     Typical values: 50Hz (analog servos), 125Hz (digital servos), 
     *                     200-333Hz (high-speed digital servos)
     * 
     * @note This affects all 6 swashplate servos (3 per rotor) plus RSC outputs
     * @note Verify servos support the selected rate - incompatible servos may jitter or fail
     * @warning Rates above 333Hz may cause excessive servo heating or reduced lifespan
     */
    void set_update_rate( uint16_t speed_hz ) override;

    /**
     * @brief Send mixed control outputs to both swashplates and RSC controllers
     * 
     * @details This is the main output function called by the flight controller at the configured
     *          loop rate (typically 400Hz). It takes the mixed servo commands and writes them to
     *          the hardware servo outputs for both swashplates and both RSC controllers.
     *          
     *          Output sequence:
     *          1. Write swashplate 1 servo positions (CH_1, CH_2, CH_3, CH_7)
     *          2. Write swashplate 2 servo positions (CH_4, CH_5, CH_6, CH_8)
     *          3. Update RSC throttle outputs for both rotors
     * 
     * @note Called at main loop rate after move_actuators() completes mixing calculations
     * @note Servo outputs are bounded by configured min/max limits before transmission
     * @warning Ensure servo endpoints are properly configured to prevent mechanical binding
     */
    void output_to_motors() override;

    /**
     * @brief Recalculate mixing and scaling factors for dual rotor control
     * 
     * @details Computes all scaling factors used in motor mixing based on current parameter
     *          settings. Called when parameters change or system initializes. Calculates:
     *          - Collective-to-throttle scaling for both rotors
     *          - Differential collective pitch scaling (H_DCP_SCALER)
     *          - Yaw response scaling (H_YAW_SCALER)
     *          - Swashplate geometry factors for both rotors
     *          - Rotor speed controller scaling
     * 
     * @note Called during initialization and whenever relevant parameters are modified
     * @note Not called during flight unless parameters change via ground station
     * @see calculate_armed_scalars() for scalars that can update during flight
     */
    void calculate_scalars() override;

    /**
     * @brief Recalculate scalars that are allowed to change during armed flight
     * 
     * @details Computes mixing scalars that can safely change while the helicopter is armed
     *          and rotors are spinning. This allows dynamic adjustment of certain control
     *          characteristics without disarming. Recalculates:
     *          - Dynamic collective scaling based on rotor speed
     *          - Yaw feed-forward compensation (H_DCP_YAW)
     *          - Differential collective trim (H_DCP_TRIM)
     *          
     *          Called at regular intervals during armed operation to allow parameter updates.
     * 
     * @note Called periodically during armed state to allow safe in-flight parameter updates
     * @note More restricted than calculate_scalars() - only updates safe-to-change parameters
     * @warning Changes to critical geometry or mixing parameters still require disarm/rearm cycle
     */
    void calculate_armed_scalars() override;

    /**
     * @brief Move all servos through their full range for mechanical validation
     * 
     * @details Executes a pre-flight servo test sequence that cycles all swashplate servos
     *          through their full mechanical range to verify proper operation, centering, and
     *          freedom of movement. Test sequence includes:
     *          - Collective sweep from min to max
     *          - Cyclic pitch sweep (longitudinal)
     *          - Cyclic roll sweep (lateral)
     *          - Combined cyclic circular motion
     *          - Both swashplates tested simultaneously
     * 
     * @note Rotors should NOT be spinning during servo test
     * @note Pilot should verify no mechanical binding or interference occurs
     * @warning Ensure rotor head is free to move - remove rotor blades if necessary for initial setup
     * @warning Do not run servo test with rotor blades installed unless rotors are secured
     */
    void servo_test() override;

    /**
     * @brief Perform pre-arm safety checks specific to dual rotor configuration
     * 
     * @details Validates that all dual rotor systems are properly configured and safe for flight.
     *          Checks performed:
     *          - Both swashplates configured with valid servo assignments
     *          - Collective ranges set properly for both rotors (COL_MIN < COL_MAX)
     *          - Rotor mode is valid (tandem, transverse, or intermeshing)
     *          - Both RSC controllers configured
     *          - Swashplate geometry parameters within acceptable ranges
     *          - Differential collective scaler not zero
     *          - For intermeshing mode: rotor phase angle configured
     * 
     * @param[in]  buflen Length of the provided buffer for error messages
     * @param[out] buffer Character buffer to store failure reason if check fails
     * 
     * @return true if all checks pass and helicopter is safe to arm, false otherwise
     * 
     * @note Called automatically during arming sequence before motors start
     * @note Failure messages are displayed to pilot via ground station
     * @warning Vehicle will not arm if any check fails - all issues must be resolved
     */
    bool arming_checks(size_t buflen, char *buffer) const override;

#if HAL_LOGGING_ENABLED
    /**
     * @brief Write dual rotor helicopter motor data to onboard log
     * 
     * @details Logs diagnostic data for both rotors to binary dataflash log at 10Hz rate.
     *          Logged data includes:
     *          - Desired collective positions for both rotors
     *          - Desired cyclic positions (roll, pitch) for both rotors
     *          - Rotor speed commands for both RSCs
     *          - Differential collective pitch value
     *          - Yaw input and differential torque command
     *          - Servo positions for all 6 swashplate servos
     *          
     *          Log data is essential for post-flight analysis and tuning.
     * 
     * @note Called at 10Hz regardless of main loop rate to limit log data volume
     * @note Log message type: HELI or DROT (check LogStructure.h for exact format)
     * @see AP_Logger For log message format definitions
     */
    void Log_Write(void) override;
#endif

    /**
     * @brief Parameter table for dual rotor helicopter configuration
     * 
     * @details Defines all configurable parameters specific to dual rotor helicopters.
     *          Parameters include rotor mode, differential collective scaling, yaw scaling,
     *          collective limits for secondary rotor, trim values, and phase angles.
     *          
     *          Key parameters registered:
     *          - H_DUAL_MODE: Rotor configuration mode
     *          - H_DCP_SCALER: Differential collective-to-pitch scaling
     *          - H_YAW_SCALER: Yaw response scaling
     *          - H_COL2_MIN/MAX: Secondary rotor collective limits
     *          - H_DCP_TRIM: Differential collective trim
     *          - H_DCP_YAW: Yaw feed-forward compensation
     *          - H_YAW_REV_EXPO: Yaw reverser smoothing (intermeshing)
     * 
     * @note Parameters are stored in EEPROM and persist across power cycles
     * @note Ground control stations read this table to display available parameters
     */
    static const struct AP_Param::GroupInfo var_info[];

protected:

    /**
     * @brief Initialize servo and RSC output channels for dual rotor configuration
     * 
     * @details Sets up hardware output channels for both swashplates and RSC controllers.
     *          Configures PWM outputs, sets default servo ranges, and initializes both
     *          swashplate objects with their respective channel assignments. Called once
     *          during system initialization.
     * 
     * @note Must be called before attempting to move servos or start rotors
     * @note Initializes 6 swashplate servo channels plus 2 RSC throttle channels
     */
    void init_outputs () override;

    /**
     * @brief Send rotor control commands to both RSC controllers
     * 
     * @details Updates both rotor speed controllers with the current control state and
     *          desired rotor speeds. Handles state transitions for both rotors:
     *          - IDLE: Rotors stopped or spinning down
     *          - RAMP_UP: Gradual rotor spin-up
     *          - ACTIVE: Normal flight operations
     *          - RAMP_DOWN: Controlled rotor shutdown
     * 
     * @param[in] state Current rotor control state for both RSCs
     * 
     * @note Both RSCs are updated simultaneously to maintain synchronized rotor speeds
     * @note State transitions are rate-limited to prevent sudden rotor speed changes
     */
    void update_motor_control(AP_MotorsHeli_RSC::RotorControlState state) override;

    /**
     * @brief Calculate individual swashplate servo position for one axis
     * 
     * @details Computes the servo position for a single servo on a specified swashplate
     *          based on the swashplate's configured geometry and the desired control inputs.
     *          This function handles the geometric mixing for one swashplate axis.
     * 
     * @param[in] swash_num   Swashplate number (1 or 2)
     * @param[in] swash_axis  Axis being calculated (0=collective, 1=longitudinal cyclic, 2=lateral cyclic)
     * @param[in] pitch_input Desired longitudinal cyclic input (-1.0 to 1.0)
     * @param[in] roll_input  Desired lateral cyclic input (-1.0 to 1.0)
     * @param[in] yaw_input   Yaw input for differential torque (not typically used in swashplate calculation)
     * @param[in] coll_input  Desired collective input (0.0 to 1.0)
     * 
     * @return Calculated servo position for the specified swashplate and axis (-1.0 to 1.0)
     * 
     * @note This is a low-level mixing function called by the mode-specific mix functions
     * @see AP_MotorsHeli_Swash for swashplate geometry configuration
     */
    float get_swashplate(int8_t swash_num, int8_t swash_axis, float pitch_input, float roll_input, float yaw_input, float coll_input);

    /**
     * @brief Main mixing function to distribute control inputs to both swashplates
     * 
     * @details Core mixing logic that takes attitude and collective commands from the flight
     *          controller and distributes them to both swashplates using differential control.
     *          Routing depends on configured dual rotor mode (tandem, transverse, intermeshing).
     *          
     *          This function:
     *          1. Applies differential collective for pitch/roll (mode-dependent)
     *          2. Calculates yaw via differential rotor torque
     *          3. Applies trim and scaling factors
     *          4. Calls appropriate mode-specific mixing function
     *          5. Bounds outputs to configured limits
     * 
     * @param[in] roll_out  Desired roll output from attitude controller (-1.0 to 1.0)
     * @param[in] pitch_out Desired pitch output from attitude controller (-1.0 to 1.0)
     * @param[in] coll_in   Desired collective/throttle input (0.0 to 1.0)
     * @param[in] yaw_out   Desired yaw output from attitude controller (-1.0 to 1.0)
     * 
     * @note Called at main loop rate (typically 400Hz) after attitude controller updates
     * @note Output is stored in _servo_out[] array for transmission to hardware
     * @see mix_tandem(), mix_transverse(), mix_intermeshing() for mode-specific implementations
     */
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)  override;

    /**
     * @brief Return frame type identifier string for logging and display
     * @return Constant string "HELI_DUAL" identifying this as a dual rotor helicopter
     */
    const char* _get_frame_string() const override { return "HELI_DUAL"; }

    /**
     * @name Swashplate Control Objects
     * @{
     */
    /** @brief Primary swashplate (front/left rotor) with servos on CH_1, CH_2, CH_3 and collective on CH_7 */
    AP_MotorsHeli_Swash _swashplate1 { CH_1, CH_2, CH_3, CH_7, 1U };
    
    /** @brief Secondary swashplate (rear/right rotor) with servos on CH_4, CH_5, CH_6 and collective on CH_8 */
    AP_MotorsHeli_Swash _swashplate2 { CH_4, CH_5, CH_6, CH_8, 2U };
    /** @} */

    /**
     * @name Servo Test Variables
     * @brief Variables used during pre-flight servo range testing
     * @{
     */
    /** @brief Current cyclic oscillation angle in radians during servo test (0 to 2π) */
    float _oscillate_angle = 0.0f;
    
    /** @brief Elapsed time tracker for servo test cycle timing in seconds */
    float _servo_test_cycle_time = 0.0f;
    
    /** @brief Collective position override during servo test (0.0 to 1.0) */
    float _collective_test = 0.0f;
    
    /** @brief Roll cyclic override during servo test (-1.0 to 1.0) */
    float _roll_test = 0.0f;
    
    /** @brief Pitch cyclic override during servo test (-1.0 to 1.0) */
    float _pitch_test = 0.0f;
    /** @} */
    
    /**
     * @brief Mixed servo output values ready for transmission to hardware
     * @details Array holding final calculated servo positions after mixing, scaling, and limiting.
     *          Indices 0-5 correspond to the 6 swashplate servos, additional indices for RSC outputs.
     *          Values are typically in range suitable for PWM output (e.g., -1.0 to 1.0 or PWM microseconds).
     */
    float _servo_out[8];

    /**
     * @name Configuration Parameters
     * @brief User-configurable parameters for dual rotor helicopter tuning
     * @{
     */
    
    /**
     * @brief Minimum collective PWM position for secondary swashplate (H_COL2_MIN)
     * @details Lowest servo position in microseconds for the rear/right rotor's collective pitch.
     *          Must be less than _collective2_max. Typical range: 1000-1500 μs.
     * @note Set to match mechanical limits - too low may cause servo binding
     */
    AP_Int16        _collective2_min;
    
    /**
     * @brief Maximum collective PWM position for secondary swashplate (H_COL2_MAX)
     * @details Highest servo position in microseconds for the rear/right rotor's collective pitch.
     *          Must be greater than _collective2_min. Typical range: 1500-2000 μs.
     * @note Set to match mechanical limits - too high may cause servo binding
     */
    AP_Int16        _collective2_max;
    
    /**
     * @brief Dual rotor configuration mode (H_DUAL_MODE)
     * @details Selects the rotor arrangement:
     *          - 0 = Tandem (front/rear rotors for pitch control)
     *          - 1 = Transverse (side-by-side rotors for roll control)
     *          - 2 = Intermeshing (overlapping counter-rotating rotors)
     * @warning Must be set correctly for vehicle configuration - incorrect mode causes loss of control
     */
    AP_Int8         _dual_mode;
    
    /**
     * @brief Differential collective pitch scaler (H_DCP_SCALER)
     * @details Scaling factor for pitch control via differential collective in tandem mode.
     *          Higher values = more aggressive pitch response. Typical range: 0.1 to 0.5.
     *          In tandem mode: pitch_diff = H_DCP_SCALER × pitch_command
     * @note Start with default (0.25) and adjust for desired pitch response
     */
    AP_Float        _dcp_scaler;
    
    /**
     * @brief Differential collective yaw feed-forward (H_DCP_YAW)
     * @details Automatic yaw compensation when differential collective is applied.
     *          Compensates for yaw disturbance caused by differential rotor torque.
     *          Range: 0.0 (no compensation) to 1.0 (full compensation).
     * @note Tune to minimize yaw deviation when applying pitch/roll commands
     */
    AP_Float        _dcp_yaw_effect;
    
    /**
     * @brief Yaw response scaler (H_YAW_SCALER)
     * @details Scaling factor for yaw control via differential rotor torque.
     *          Controls yaw authority by varying rotor speed or collective pitch differences.
     *          Typical range: 0.5 to 2.0.
     * @note Higher values increase yaw responsiveness but may cause oscillation
     */
    AP_Float        _yaw_scaler;
    
    /**
     * @brief Differential collective pitch trim (H_DCP_TRIM)
     * @details Trim adjustment for differential collective to balance hover.
     *          Used to correct for CG offset, asymmetric rotor efficiency, or rigging differences.
     *          Range: -1.0 to 1.0 (positive = more collective on rear/right rotor).
     * @note Adjust to achieve level hover with neutral pitch stick
     */
    AP_Float        _dcp_trim;
    
    /**
     * @brief Yaw reverser smoothing exponent for intermeshing mode (H_YAW_REV_EXPO)
     * @details Exponential smoothing factor for yaw reversals in intermeshing rotor configuration.
     *          Prevents abrupt yaw direction changes that could cause rotor phase issues.
     *          Range: 0.0 (linear) to 1.0 (maximum smoothing).
     * @note Only used in intermeshing mode (H_DUAL_MODE = 2)
     */
    AP_Float        _yaw_rev_expo;
    /** @} */

    /**
     * @brief Calculated zero-thrust collective percentage for secondary rotor
     * @details Internal variable storing the collective position where the secondary rotor
     *          produces zero net thrust. Used for proper scaling of collective inputs.
     *          Calculated from _collective2_min and _collective2_max during init.
     */
    float _collective2_zero_thrst_pct;

private:

    /**
     * @brief Mix and output swashplates for tandem rotor configuration
     * 
     * @details Implements tandem (longitudinal) rotor mixing where rotors are mounted front and rear.
     *          Control distribution:
     *          - Pitch: Differential collective between front and rear rotors (scaled by H_DCP_SCALER)
     *          - Roll: Synchronized lateral cyclic on both swashplates
     *          - Yaw: Differential rotor speed/torque (scaled by H_YAW_SCALER)
     *          - Collective: Equal collective pitch on both rotors for vertical thrust
     *          
     *          Front rotor collective = collective_input - (pitch_input × H_DCP_SCALER)
     *          Rear rotor collective = collective_input + (pitch_input × H_DCP_SCALER)
     * 
     * @param[in] pitch_input       Desired pitch output (-1.0 to 1.0, forward positive)
     * @param[in] roll_input        Desired roll output (-1.0 to 1.0, right positive)
     * @param[in] yaw_input         Desired yaw output (-1.0 to 1.0, right positive)
     * @param[in] collective1_input Collective for front rotor (0.0 to 1.0)
     * @param[in] collective2_input Collective for rear rotor (0.0 to 1.0)
     * 
     * @note Called by move_actuators() when H_DUAL_MODE = 0 (tandem)
     * @note Applies H_DCP_TRIM to balance differential collective
     * @see mix_transverse(), mix_intermeshing() for other rotor configurations
     */
    void mix_tandem(float pitch_input, float roll_input, float yaw_input, float collective1_input, float collective2_input);

    /**
     * @brief Mix and output swashplates for transverse rotor configuration
     * 
     * @details Implements transverse (lateral) rotor mixing where rotors are mounted side by side.
     *          Control distribution:
     *          - Pitch: Synchronized longitudinal cyclic on both swashplates
     *          - Roll: Differential cyclic between left and right rotors
     *          - Yaw: Differential rotor speed/torque (scaled by H_YAW_SCALER)
     *          - Collective: Equal collective pitch on both rotors for vertical thrust
     *          
     *          Left rotor cyclic = roll_input × differential_factor
     *          Right rotor cyclic = -roll_input × differential_factor
     * 
     * @param[in] pitch_input       Desired pitch output (-1.0 to 1.0, forward positive)
     * @param[in] roll_input        Desired roll output (-1.0 to 1.0, right positive)
     * @param[in] yaw_input         Desired yaw output (-1.0 to 1.0, right positive)
     * @param[in] collective1_input Collective for left rotor (0.0 to 1.0)
     * @param[in] collective2_input Collective for right rotor (0.0 to 1.0)
     * 
     * @note Called by move_actuators() when H_DUAL_MODE = 1 (transverse)
     * @see mix_tandem(), mix_intermeshing() for other rotor configurations
     */
    void mix_transverse(float pitch_input, float roll_input, float yaw_input, float collective1_input, float collective2_input);

    /**
     * @brief Mix and output swashplates for intermeshing rotor configuration
     * 
     * @details Implements intermeshing rotor mixing where counter-rotating rotors have overlapping
     *          blade paths. Similar to transverse but with critical rotor phase synchronization
     *          to prevent blade strikes. Uses H_YAW_REV_EXPO for smooth yaw reversals.
     *          
     *          Control distribution:
     *          - Pitch: Synchronized longitudinal cyclic
     *          - Roll: Differential cyclic between rotors
     *          - Yaw: Differential torque with exponential smoothing
     *          - Collective: Synchronized with phase compensation
     *          
     *          Rotor phase angle (H_PHASE_ANGLE) offsets rotor positions to maintain safe clearance.
     * 
     * @param[in] pitch_input       Desired pitch output (-1.0 to 1.0, forward positive)
     * @param[in] roll_input        Desired roll output (-1.0 to 1.0, right positive)
     * @param[in] yaw_input         Desired yaw output (-1.0 to 1.0, right positive)
     * @param[in] collective1_input Collective for first rotor (0.0 to 1.0)
     * @param[in] collective2_input Collective for second rotor (0.0 to 1.0)
     * 
     * @note Called by move_actuators() when H_DUAL_MODE = 2 (intermeshing)
     * @warning Rotor phase angle MUST be configured correctly to prevent blade collision
     * @warning Yaw reversals are smoothed by H_YAW_REV_EXPO to prevent phase issues
     * @see mix_tandem(), mix_transverse() for other rotor configurations
     */
    void mix_intermeshing(float pitch_input, float roll_input, float yaw_input, float collective1_input, float collective2_input);

};
