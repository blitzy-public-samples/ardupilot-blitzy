/**
 * @file AP_Compass_ExternalAHRS.h
 * @brief External AHRS magnetometer interface driver
 * 
 * @details This driver integrates magnetometer (compass) data from external
 *          Attitude and Heading Reference Systems (AHRS) into ArduPilot's
 *          compass subsystem. External AHRS units typically provide integrated
 *          IMU, magnetometer, and GNSS data with onboard sensor fusion.
 * 
 *          Supported external AHRS systems include:
 *          - VectorNav VN-100, VN-200, VN-300 series
 *          - LORD MicroStrain 3DM-GX5, 3DM-GQ7 series
 *          - Other systems implementing AP_ExternalAHRS interface
 * 
 *          The driver receives magnetic field measurements from the external
 *          AHRS unit and integrates them with ArduPilot's compass calibration
 *          and selection logic, allowing the external magnetometer to be used
 *          alongside or instead of onboard compass sensors.
 * 
 * @note Magnetic field data is received in milligauss units and transformed
 *       to body frame coordinates before integration into the compass system.
 * 
 * Source: libraries/AP_Compass/AP_Compass_ExternalAHRS.h
 */

#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_EXTERNALAHRS_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

/**
 * @class AP_Compass_ExternalAHRS
 * @brief Compass backend for External AHRS magnetometer integration
 * 
 * @details This backend class integrates magnetometer data from external AHRS
 *          units into ArduPilot's compass subsystem. External AHRS systems
 *          typically provide pre-calibrated, temperature-compensated magnetic
 *          field measurements as part of their integrated sensor suite.
 * 
 *          Integration Architecture:
 *          - Receives mag_data_message_t from AP_ExternalAHRS subsystem
 *          - Transforms magnetic field vector to ArduPilot body frame
 *          - Registers compass instance with standard ArduPilot compass system
 *          - Participates in compass health monitoring and selection
 * 
 *          Coordinate Frame Handling:
 *          - External AHRS provides magnetic field in device-specific frame
 *          - Driver transforms to ArduPilot body frame (NED convention)
 *          - Magnetic field components: (X: forward, Y: right, Z: down)
 * 
 *          Unit Conventions:
 *          - Magnetic field strength: milligauss (mG)
 *          - Typical Earth field: ~500 mG total magnitude
 *          - Range: ±1300 mG horizontal, ±6500 mG vertical components
 * 
 *          Instance Management:
 *          - Supports multiple external AHRS sources simultaneously
 *          - Each external AHRS magnetometer registered as separate instance
 *          - Instance numbering managed by compass backend registration
 * 
 *          Applications:
 *          - Integration of VectorNav VN-100/200/300 magnetometers
 *          - LORD MicroStrain 3DM-GX5/GQ7 compass integration
 *          - Custom external AHRS magnetometer sources
 *          - Redundant compass configurations with external validation
 * 
 * @note This driver does NOT perform compass calibration - it relies on
 *       calibration performed by the external AHRS unit or ArduPilot's
 *       standard compass calibration if raw data is provided.
 * 
 * @warning Ensure external AHRS coordinate frame configuration matches
 *          ArduPilot body frame orientation. Misaligned frames will cause
 *          heading errors and potential navigation failures.
 */
class AP_Compass_ExternalAHRS : public AP_Compass_Backend
{
public:
    /**
     * @brief Construct External AHRS compass backend instance
     * 
     * @details Initializes compass backend for receiving magnetometer data
     *          from an external AHRS unit. Registers the compass instance
     *          with ArduPilot's compass subsystem and sets up message handling.
     * 
     * @param[in] instance External AHRS instance number (0-based)
     *                     Used to identify which external AHRS source provides
     *                     this magnetometer data when multiple units present
     * 
     * @note Constructor is called during AP_ExternalAHRS initialization when
     *       magnetometer support is detected and enabled in external unit
     */
    AP_Compass_ExternalAHRS(uint8_t instance);

    /**
     * @brief Read and update compass data from external AHRS
     * 
     * @details This method is called periodically by the compass driver
     *          scheduler to update compass measurements. For external AHRS
     *          backends, the actual data reception occurs asynchronously via
     *          handle_external() callback when messages arrive from the
     *          external AHRS unit.
     * 
     *          This implementation typically has minimal work since data
     *          updates are driven by external AHRS message reception rather
     *          than polling. Any data transformation or validation is
     *          performed in handle_external().
     * 
     * @note Called at compass backend update rate (typically 50-100 Hz)
     *       but actual data update rate determined by external AHRS
     */
    void read(void) override;

private:
    /**
     * @brief Handle incoming magnetic field data from external AHRS
     * 
     * @details This callback method receives magnetometer measurements from
     *          the external AHRS subsystem and integrates them into ArduPilot's
     *          compass system. Called asynchronously when mag_data_message_t
     *          messages arrive from the external AHRS unit.
     * 
     *          Processing Steps:
     *          1. Extract magnetic field vector from message (X, Y, Z components)
     *          2. Validate field magnitude is within reasonable range
     *          3. Transform from external AHRS frame to body frame if needed
     *          4. Update compass instance with new field measurement
     *          5. Update compass health status based on data validity
     * 
     *          Message Format (mag_data_message_t):
     *          - field: Vector3f containing magnetic field components
     *          - Units: milligauss (mG) for each axis
     *          - Frame: External AHRS device frame (may require transformation)
     *          - Timestamp: Measurement time from external AHRS
     * 
     *          Coordinate Transformations:
     *          - External AHRS may use different body frame convention
     *          - Driver ensures data presented in ArduPilot body frame (NED)
     *          - Rotation applied based on AHRS_ORIENTATION parameter if set
     * 
     *          Data Validation:
     *          - Checks for reasonable magnetic field magnitude (200-800 mG typical)
     *          - Rejects measurements outside physical limits (±6500 mG per axis)
     *          - Monitors update rate to detect communication failures
     * 
     * @param[in] pkt Magnetic field data message from external AHRS containing:
     *                - field: 3D magnetic field vector in milligauss
     *                - timestamp: Measurement time from external unit
     * 
     * @note This method is called from AP_ExternalAHRS context, not the
     *       main compass driver thread. Ensure thread-safe access to shared data.
     * 
     * @warning Incorrect coordinate frame transformations will cause heading
     *          errors. Verify external AHRS orientation matches vehicle frame.
     */
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt) override;
    
    /**
     * @brief External AHRS instance number for this compass backend
     * 
     * @details Identifies which external AHRS unit provides magnetometer data
     *          for this compass instance. Used when multiple external AHRS
     *          units are present to associate compass measurements with their
     *          source device.
     * 
     *          Range: 0 to (max external AHRS instances - 1)
     */
    uint8_t instance;
};

#endif  // AP_COMPASS_EXTERNALAHRS_ENABLED
