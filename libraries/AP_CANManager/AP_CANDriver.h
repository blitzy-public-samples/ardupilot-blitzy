/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

/**
 * @file AP_CANDriver.h
 * @brief Abstract interface for CAN protocol drivers
 * 
 * @details Defines pure virtual contract that all CAN protocol drivers must implement. 
 *          AP_CANManager uses this interface to initialize drivers, link them to physical 
 *          CAN interfaces, and optionally support 11-bit auxiliary drivers. Implementations 
 *          include DroneCAN, PiccoloCAN, KDECAN, and various sensor-specific protocols. 
 *          Header-only design minimizes compilation dependencies.
 * 
 * @note Subclasses must implement init() and add_interface() as pure virtuals
 * @see AP_CANManager, CANSensor, AP_DroneCAN
 */

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

/**
 * @brief Forward declarations to avoid circular includes
 * @note AP_CANManager and CANSensor defined in separate headers
 */
class AP_CANManager;
class CANSensor;

/**
 * @class AP_CANDriver
 * @brief Abstract base class defining protocol driver interface contract
 * 
 * @details Pure virtual interface for all CAN protocol drivers in ArduPilot.
 *          AP_CANManager instantiates and manages concrete implementations.
 *          
 *          Driver lifecycle: construct → init() → add_interface() → operational
 *          
 *          Optional support for 11-bit auxiliary drivers via add_11bit_driver()
 *          allows a primary driver to host secondary sensor protocols sharing
 *          the same CAN interface but using different addressing/protocol.
 *          
 *          Optional frame transmission support for auxiliary drivers via 
 *          write_aux_frame() enables primary driver to handle physical 
 *          transmission for auxiliary sensor drivers that lack direct 
 *          interface access.
 *          
 *          Friend class AP_CANManager grants access to private/protected 
 *          registration mechanisms. Header-only design reduces build time cost.
 * 
 * @note All subclasses must implement init() and add_interface() pure virtuals
 * @warning Driver instance must remain valid for lifetime of AP_CANManager
 * @see AP_CANManager::register_driver(), CANSensor
 */
class AP_CANDriver
{
public:

    /**
     * @brief Grant AP_CANManager access to driver internals for registration
     * @note Enables manager to coordinate driver initialization without exposing public methods
     */
    friend class AP_CANManager;

    /**
     * @brief Initialize protocol driver with assigned index and filter configuration
     * 
     * @param[in] driver_index Driver slot index (0 to HAL_MAX_CAN_PROTOCOL_DRIVERS-1) assigned by manager
     * @param[in] enable_filters true to enable hardware CAN ID filtering for performance, false for promiscuous mode
     * 
     * @details Called by AP_CANManager during system initialization. Driver should allocate 
     *          resources, configure filtering if enabled, prepare for add_interface() calls. 
     *          Return void - initialization failures should be reported via AP::can().log_text()
     * 
     * @note Called exactly once per driver during vehicle startup
     * @warning Must complete before CAN communication begins - no blocking operations
     * @see AP_CANManager::init()
     */
    virtual void init(uint8_t driver_index, bool enable_filters) = 0;

    /**
     * @brief Link a physical CAN interface to this protocol driver
     * 
     * @param[in] can_iface Pointer to HAL CAN interface (remains valid for driver lifetime)
     * @return true if interface successfully added, false if driver cannot accept more interfaces
     * 
     * @details AP_CANManager calls this for each CAN_Pn_DRIVER parameter mapping to this 
     *          driver's index. Driver may be linked to multiple physical interfaces if 
     *          configured. Driver should store pointer and begin monitoring for incoming 
     *          frames. Implementation typically spawns receive thread or registers callback.
     * 
     * @note May be called multiple times to add multiple interfaces to same driver
     * @warning Interface pointer must remain valid - do not store stack/temporary addresses
     * @see AP_HAL::CANIface
     */
    virtual bool add_interface(AP_HAL::CANIface* can_iface) = 0;

    /**
     * @brief Register 11-bit auxiliary sensor driver (optional override)
     * 
     * @param[in] sensor Pointer to CANSensor subclass implementing 11-bit protocol
     * @return true if auxiliary driver successfully registered, false if not supported
     * 
     * @details Default implementation returns false. Override in drivers supporting 
     *          CAN_Dn_PROTOCOL2 auxiliary protocol parameter (e.g., DroneCAN supports 
     *          11-bit rangefinders alongside 29-bit DroneCAN). Auxiliary driver shares 
     *          same CAN interface but uses different addressing/protocol.
     * 
     * @note Optional feature - most drivers do not override this method
     * @see CANSensor, AP_CANManager::register_11bit_driver()
     */
    virtual bool add_11bit_driver(CANSensor *sensor) { return false; }

    /**
     * @brief Transmit frame on behalf of 11-bit auxiliary driver (optional override)
     * 
     * @param[in,out] out_frame CAN frame to transmit (may be modified by HAL)
     * @param[in] timeout_us Transmit timeout in microseconds
     * @return true if frame transmitted successfully, false if timeout/error or not supported
     * 
     * @details Default implementation returns false. Override in drivers supporting auxiliary 
     *          11-bit protocol transmission. Primary driver handles physical transmission for 
     *          auxiliary sensor drivers that lack direct interface access.
     * 
     * @note Only required if add_11bit_driver() is implemented
     * @warning Must implement timeout to prevent indefinite blocking
     */
    virtual bool write_aux_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us) { return false; }
};

/**
 * @example Typical AP_CANDriver implementation pattern
 * 
 * @code
 * class MyProtocolDriver : public AP_CANDriver {
 * public:
 *     void init(uint8_t driver_index, bool enable_filters) override {
 *         _driver_index = driver_index;
 *         // Initialize resources, configure filters
 *     }
 *     
 *     bool add_interface(AP_HAL::CANIface* can_iface) override {
 *         _can_iface = can_iface;
 *         // Start receive thread or register callback
 *         return true;
 *     }
 *     
 * private:
 *     uint8_t _driver_index;
 *     AP_HAL::CANIface* _can_iface;
 * };
 * @endcode
 */
