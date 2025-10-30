/**
 * @file GCS_Dummy.h
 * @brief Lightweight GCS backend for examples and testing tools
 * 
 * @details This file provides minimal GCS_MAVLINK implementations used by
 *          example sketches and development utilities. GCS_MAVLINK_Dummy
 *          stubs out message sending and vehicle state reporting, allowing
 *          tools to use GCS infrastructure without actual telemetry transport
 *          or flight control integration.
 * 
 * @note This is NOT for production use in flight code - only for examples,
 *       testing, and development tools that need GCS API access without
 *       real vehicle communication.
 * 
 * @warning This implementation has no actual telemetry or vehicle control.
 *          All message sending is stubbed out, and vehicle state methods
 *          return dummy values.
 */

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include "GCS.h"
#include <AP_Common/AP_FWVersion.h>

#define THISFIRMWARE "GCSDummy V3.1.4-dev"

#define FW_MAJOR 3
#define FW_MINOR 1
#define FW_PATCH 4
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

/**
 * @class GCS_MAVLINK_Dummy
 * @brief Minimal GCS_MAVLINK implementation for examples and test tools
 * 
 * @details Provides stubbed-out message sending and vehicle state reporting
 *          for non-flight testing scenarios. Used by example sketches and
 *          utilities that need GCS infrastructure without actual telemetry
 *          or vehicle control.
 *          
 *          This backend pretends to successfully send all messages but
 *          actually discards them. Vehicle state queries return fixed
 *          dummy values. No navigation, PID tuning, or mode information
 *          is provided.
 * 
 * @note Used by example programs in libraries/*/examples/ and development
 *       tools that instantiate GCS for API compatibility but don't need
 *       real communication.
 * 
 * @warning NOT for production flight code. Has no telemetry transport and
 *          no vehicle state integration.
 */
class GCS_MAVLINK_Dummy : public GCS_MAVLINK
{
public:

    using GCS_MAVLINK::GCS_MAVLINK;

private:

    /**
     * @brief Attempt to send a MAVLink message (stub implementation)
     * 
     * @param[in] id Message identifier from ap_message enum
     * 
     * @return Always returns true (messages are not actually sent)
     * 
     * @note This stub allows code to call message sending without errors,
     *       but no actual transmission occurs. Used by examples that need
     *       GCS API but not real telemetry.
     */
    bool try_send_message(enum ap_message id) override { return true; }

protected:

    /**
     * @brief Get vehicle base mode flags (dummy value)
     * 
     * @return MAV_MODE_FLAG_CUSTOM_MODE_ENABLED flag as dummy base mode
     * 
     * @note Returns fixed dummy value. Does not reflect actual vehicle state.
     */
    uint8_t base_mode() const override { return (uint8_t)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    
    /**
     * @brief Get vehicle system status (dummy value)
     * 
     * @return MAV_STATE_CALIBRATING as dummy status
     * 
     * @note Returns fixed dummy value. Does not reflect actual vehicle state.
     */
    MAV_STATE vehicle_system_status() const override { return MAV_STATE_CALIBRATING; }

    /**
     * @brief Send navigation controller output (stub implementation)
     * 
     * @note Empty implementation. No navigation output is generated or sent.
     */
    void send_nav_controller_output() const override {};
    
    /**
     * @brief Send PID tuning information (stub implementation)
     * 
     * @note Empty implementation. No PID tuning data is generated or sent.
     */
    void send_pid_tuning() override {};
    
    /**
     * @brief Report available flight modes (stub implementation)
     * 
     * @param[in] index Mode index to query
     * 
     * @return Always returns 0 (no available modes reported)
     * 
     * @note Stub implementation. Does not report any available flight modes.
     */
    uint8_t send_available_mode(uint8_t index) const override { return 0; }
};

/**
 * @class GCS_Dummy
 * @brief GCS singleton implementation for examples and test tools
 * 
 * @details Provides minimal GCS infrastructure without real telemetry transport.
 *          Creates GCS_MAVLINK_Dummy backend instances for each communication
 *          channel. Used by example programs and test utilities that need GCS
 *          API access but not actual vehicle communication or control.
 *          
 *          This singleton manages dummy GCS backends and provides stubbed
 *          vehicle identification (frame type and custom mode). Text messages
 *          can be sent but are handled by stub implementation.
 * 
 * @note Used by example sketches in libraries/*/examples/ and development
 *       tools. NOT for production flight code.
 * 
 * @warning No actual telemetry, vehicle state, or flight control integration.
 *          All functionality is stubbed for API compatibility only.
 */

extern const AP_HAL::HAL& hal;

class GCS_Dummy : public GCS
{
public:

    using GCS::GCS;

protected:

    /**
     * @brief Factory method to create new GCS_MAVLINK_Dummy backend instances
     * 
     * @param[in] uart UART driver for this backend (not actually used for communication)
     * 
     * @return Pointer to newly allocated GCS_MAVLINK_Dummy instance, or nullptr on allocation failure
     * 
     * @note Called by GCS infrastructure when initializing communication channels.
     *       Allocates dummy backend that stubs out all message handling.
     */
    GCS_MAVLINK_Dummy *new_gcs_mavlink_backend(AP_HAL::UARTDriver &uart) override {
        return NEW_NOTHROW GCS_MAVLINK_Dummy(uart);
    }

private:
    // the following define expands to a pair of methods to retrieve a
    // pointer to an object of the correct subclass for the link at
    // offset ofs.  These are of the form:
    // GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override;
    // const GCS_MAVLINK_XXXX *chan(const uint8_t ofs) override const;
    GCS_MAVLINK_CHAN_METHOD_DEFINITIONS(GCS_MAVLINK_Dummy);

    /**
     * @brief Send formatted text message to ground control station
     * 
     * @param[in] severity Message severity level (MAV_SEVERITY enum)
     * @param[in] fmt Printf-style format string
     * @param[in] arg_list Variable argument list for format string
     * @param[in] dest_bitmask Bitmask of destination channels
     * 
     * @note Implementation in GCS_Dummy.cpp. May output to console or discard
     *       depending on example/tool requirements.
     */
    void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t dest_bitmask) override;

    /**
     * @brief Get vehicle frame type (dummy value)
     * 
     * @return MAV_TYPE_FIXED_WING as dummy frame type
     * 
     * @note Returns fixed dummy value. Does not reflect actual vehicle configuration.
     */
    MAV_TYPE frame_type() const override { return MAV_TYPE_FIXED_WING; }
    
    /**
     * @brief Get vehicle custom mode number (dummy value)
     * 
     * @return Magic number 3 as dummy custom mode
     * 
     * @note Returns arbitrary fixed value. Does not reflect actual vehicle mode.
     */
    uint32_t custom_mode() const override { return 3; } // magic number
};

#endif  // HAL_GCS_ENABLED
