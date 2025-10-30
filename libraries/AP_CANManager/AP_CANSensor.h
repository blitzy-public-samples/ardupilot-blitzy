/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  CANSensor class, for easy creation of CAN sensors using custom CAN protocols
 */

/**
 * @file AP_CANSensor.h
 * @brief Base class for CAN sensor drivers using custom protocols
 * 
 * @details Provides threaded receive loop, frame transmission, and interface registration
 *          for protocol drivers. Two classes are provided:
 *          - CANSensor: Single backend supporting one sensor instance
 *          - MultiCAN: Multi-backend supporting multiple sensor instances with callback forwarding
 * 
 * @author ArduPilot Development Team
 */
 
#pragma once

#include "AP_CAN.h"
#include "AP_CANDriver.h"
#ifndef HAL_BUILD_AP_PERIPH
#include "AP_CANManager.h"
#endif

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

/**
 * @class CANSensor
 * @brief Base class for CAN protocol drivers implementing custom sensor protocols
 * 
 * @details CANSensor provides a framework for implementing CAN-based sensor drivers
 *          using custom protocols. Key features:
 *          - Inherits from AP_CANDriver to implement the protocol driver contract
 *          - Spawns a dedicated thread running loop() at specified stack size
 *          - Provides write_frame() for transmitting CAN frames with timeout/select
 *          - Subclasses implement pure virtual handle_frame() to parse received CAN frames
 *          - Automatically registers with AP_CANManager during init()
 *          - Thread-safe frame transmission via semaphore
 *          - Supports both main firmware and AP_Periph builds with conditional compilation
 * 
 * @note Subclass must implement handle_frame() to parse protocol-specific frames
 * @warning Constructor stack_size parameter must be sufficient for subclass needs (default 2048 bytes)
 * 
 * @see AP_CANDriver
 * @see AP_CANManager
 * @see AP_Proximity_Benewake_CAN
 * @see AP_RangeFinder_Benewake_CAN
 */
class CANSensor : public AP_CANDriver {
public:
    /**
     * @brief Construct CAN sensor driver with specified thread stack size
     * 
     * @param[in] driver_name Human-readable name for thread/logging (e.g., 'RangeFinder_Benewake')
     * @param[in] stack_size Thread stack size in bytes (default 2048), must accommodate subclass processing
     * 
     * @note Does not start thread - init() must be called first
     */
    CANSensor(const char *driver_name, uint16_t stack_size=2048);

    /* Do not allow copies */
    CLASS_NO_COPY(CANSensor);

    /**
     * @brief Initialize driver, register with manager, start receive thread
     * 
     * @param[in] driver_index Driver slot index assigned by AP_CANManager
     * @param[in] enable_filters Enable hardware CAN filtering for this driver
     * 
     * @details Registers driver via register_driver(), stores driver_index,
     *          spawns loop() thread with configured stack_size
     * 
     * @note Called by AP_CANManager during system initialization
     */
    void init(uint8_t driver_index, bool enable_filters) override;
    
    /**
     * @brief Link physical CAN interface to this driver
     * 
     * @param[in] can_iface Pointer to HAL CAN interface (must remain valid)
     * @return true if interface successfully added
     * 
     * @details Stores interface pointer for write_frame() transmission, enables receive loop
     * 
     * @warning Can be called multiple times to add multiple interfaces
     */
    bool add_interface(AP_HAL::CANIface* can_iface) override;

    /**
     * @brief Check if driver has been successfully initialized and registered
     * @return true if init() completed successfully and driver is operational
     * @note Useful for subclass conditional logic during startup
     */
    bool initialized() const { return _initialized; }

    /**
     * @brief Pure virtual handler for incoming CAN frames
     * 
     * @param[in,out] frame Received CAN frame with ID, DLC, data[] array
     * 
     * @details Called by receive loop for each frame matching driver's filter.
     *          Subclass parses protocol-specific data, updates internal state,
     *          publishes to frontend
     * 
     * @note Runs in dedicated thread context, must be thread-safe if accessing shared data
     * @warning Long processing delays can cause CAN RX buffer overflow - keep handler fast
     */
    virtual void handle_frame(AP_HAL::CANFrame &frame) = 0;

    /**
     * @brief Transmit CAN frame with timeout
     * 
     * @param[in,out] out_frame CAN frame to transmit (may be modified by HAL)
     * @param[in] timeout_us Transmit timeout in microseconds
     * @return true if frame transmitted successfully, false on timeout or error
     * 
     * @details Uses select() with timeout to avoid blocking indefinitely if bus saturated
     * 
     * @note Thread-safe via internal semaphore
     * @warning High-frequency transmission can saturate CAN bus (max ~8000 frames/sec @ 1Mbps)
     */
    bool write_frame(AP_HAL::CANFrame &out_frame, const uint32_t timeout_us);

#ifdef HAL_BUILD_AP_PERIPH
    /**
     * @brief Configure CAN interface for AP_Periph peripheral firmware
     * 
     * @param[in] i Interface index (0-based)
     * @param[in] protocol Protocol type for this interface
     * @param[in] iface HAL CAN interface pointer
     * 
     * @note AP_Periph uses simplified configuration without full AP_CANManager
     */
    static void set_periph(const uint8_t i, const AP_CAN::Protocol protocol, AP_HAL::CANIface* iface) {
        if (i < ARRAY_SIZE(_periph)) {
            _periph[i].protocol = protocol;
            _periph[i].iface = iface;
        }
    }

    /**
     * @brief Retrieve protocol type for driver index in AP_Periph build
     * 
     * @param[in] i Driver index
     * @return AP_CAN::Protocol enum, Protocol::None if invalid
     */
    static AP_CAN::Protocol get_driver_type(const uint8_t i)
    {
        if (i < ARRAY_SIZE(_periph)) {
            return _periph[i].protocol;
        }
        return AP_CAN::Protocol::None;
    }
#else
    /**
     * @brief Retrieve protocol type via AP_CANManager singleton
     * 
     * @param[in] i Driver index
     * @return AP_CAN::Protocol enum from manager cache
     */
    static AP_CAN::Protocol get_driver_type(const uint8_t i) { return AP::can().get_driver_type(i); }
#endif

protected:
    /**
     * @brief Register this driver with AP_CANManager (main firmware) or store in _periph array (AP_Periph)
     * 
     * @param[in] dtype Protocol type from AP_CAN::Protocol enum
     * 
     * @details Called internally by init(), handles build-specific registration
     */
    void register_driver(AP_CAN::Protocol dtype);

private:
    /**
     * @brief Thread main loop: receive frames, call handle_frame(), yield
     * 
     * @details Blocking receive from _can_iface, dispatches to handle_frame(),
     *          runs until thread terminated
     * 
     * @note Private method running in dedicated thread context
     * @warning Must not be called directly - spawned by init()
     */
    void loop();

    const char *const _driver_name;
    const uint16_t _stack_size;
    bool _initialized;
    uint8_t _driver_index;

    // this is true when we are setup as an auxillary driver using CAN_Dn_PROTOCOL2
    bool is_aux_11bit_driver;

    AP_CANDriver *_can_driver;
    HAL_BinarySemaphore sem_handle;
    AP_HAL::CANIface* _can_iface;

#ifdef HAL_BUILD_AP_PERIPH
    /**
     * @brief AP_Periph-specific driver registration implementation
     * 
     * @param[in] dtype Protocol type
     * 
     * @note Stores driver info in static _periph array instead of AP_CANManager
     */
    void register_driver_periph(const AP_CAN::Protocol dtype);
    
    /**
     * @brief Per-interface driver configuration for AP_Periph builds
     * 
     * @details Stores HAL interface pointer and protocol type without full manager overhead
     */
    struct CANSensor_Periph {
        AP_HAL::CANIface* iface;
        AP_CAN::Protocol protocol;
    } static _periph[HAL_NUM_CAN_IFACES];
#endif
};

/**
 * @class MultiCAN
 * @brief CAN sensor driver supporting multiple backend consumers via callback forwarding
 * 
 * @details MultiCAN extends CANSensor to enable frame multicasting to multiple backends:
 *          - Single CAN protocol driver forwards frames to multiple registered backends
 *          - Use case: Multiple rangefinders or proximity sensors on same CAN bus
 *          - Backends register ForwardCanFrame callbacks via linked list
 *          - handle_frame() distributes each received frame to all registered callbacks
 *          - Thread-safe callback list via HAL_Semaphore
 * 
 * @note Useful when multiple sensor instances share same CAN protocol
 * @warning All callbacks must execute quickly to avoid blocking receive thread
 * 
 * @see CANSensor
 * @see AP_Proximity_Benewake_CAN
 * @see AP_RangeFinder_Benewake_CAN
 */
class MultiCAN : public CANSensor {
public:
    /**
     * @brief Callback function type for forwarding CAN frames to backends
     * 
     * @param[in,out] frame CAN frame to process
     * @return true if backend successfully handled frame
     */
    FUNCTOR_TYPEDEF(ForwardCanFrame, bool, AP_HAL::CANFrame &);

    /**
     * @brief Construct multi-backend CAN driver
     * 
     * @param[in] cf Callback function to register for this instance
     * @param[in] can_type Protocol type from AP_CAN::Protocol enum
     * @param[in] driver_name Human-readable name for driver
     * 
     * @details Registers callback in static linked list, enabling frame distribution to multiple backends
     */
    MultiCAN(ForwardCanFrame cf, AP_CAN::Protocol can_type, const char *driver_name);

    /**
     * @brief Receive frame and distribute to all registered callbacks
     * 
     * @param[in,out] frame Received CAN frame
     * 
     * @details Iterates through MultiCANLinkedList, calls each registered ForwardCanFrame callback
     * 
     * @note Callbacks execute sequentially in receive thread context
     */
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    /**
     * @class MultiCANLinkedList
     * @brief Thread-safe linked list managing MultiCAN frame forwarding callbacks
     * 
     * @details Static persistence across MultiCAN instances, semaphore-protected list operations
     */
    class MultiCANLinkedList {
    public:
        struct CANSensor_Multi {
            ForwardCanFrame _callback;
            CANSensor_Multi* next = nullptr;
        };

        /**
         * @brief Add callback to forwarding list
         * 
         * @param[in] callback ForwardCanFrame functor to register
         * 
         * @details Appends to linked list, protected by semaphore
         */
        void register_callback(ForwardCanFrame callback);

        /**
         * @brief Distribute frame to all registered callbacks in list
         * 
         * @param[in,out] frame Frame to forward
         * 
         * @details Iterates list under semaphore, calls each callback sequentially
         */
        void handle_frame(AP_HAL::CANFrame &frame);
        HAL_Semaphore sem;

    private:
        CANSensor_Multi* head = nullptr;
    };

    /**
     * @brief Static linked list instance for callback persistence
     * 
     * @note Shared across all MultiCAN instances to enable frame multicasting
     */
    static MultiCANLinkedList* callbacks;
};

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS

