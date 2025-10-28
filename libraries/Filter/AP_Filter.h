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
 */

/**
 * @file AP_Filter.h
 * @brief ArduPilot integration layer for configurable filter management with parameter persistence
 * 
 * @details This file provides the AP_Param-managed filter framework for ArduPilot,
 *          enabling runtime-configurable filters with EEPROM persistence. The system
 *          supports multiple filter types (currently FILTER_NONE and FILTER_NOTCH)
 *          with parameters that persist across reboots.
 *          
 *          Key components:
 *          - AP_Filter: Virtual base class for filter implementations
 *          - AP_Filter_params: Parameter container for filter configuration
 *          - AP_NotchFilter_params: Notch filter implementation with frequency, quality, and attenuation parameters
 *          - AP_Filters: Singleton manager for multiple filter instances
 *          
 *          The filter manager (AP_Filters) handles dynamic backend allocation only when
 *          the vehicle is unarmed, preventing heap allocation during flight operations.
 *          
 * @warning This entire file is gated by AP_FILTER_ENABLED from AP_Filter_config.h
 * @warning Filter backends are only allocated when vehicle is unarmed (hal.util->get_soft_armed() == false)
 * 
 * @see NotchFilter.h for the underlying NotchFilterFloat implementation
 * @see AP_Param.h for parameter persistence system
 * 
 * Source: libraries/Filter/AP_Filter.h:1-93
 */
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_Filter_config.h"
#include "NotchFilter.h"

#if AP_FILTER_ENABLED

/**
 * @class AP_Filter
 * @brief Base class for AP_Param-managed filters with runtime configuration
 * 
 * @details This abstract base class provides the interface for filter types that can be
 *          managed through ArduPilot's parameter system (AP_Param). It enables filters
 *          to be configured at runtime with settings persisted to EEPROM.
 *          
 *          Purpose:
 *          - Provides virtual interface for filter setup operations
 *          - Integrates with AP_Param system for parameter persistence
 *          - Supports multiple filter types through FilterType enumeration
 *          - Currently supports FILTER_NONE (disabled) and FILTER_NOTCH (notch filter)
 *          
 *          Architecture:
 *          - Subclasses implement specific filter types (e.g., AP_NotchFilter_params)
 *          - Each filter type implements the setup methods for its specific configuration
 *          - Filter type stored in _type member for runtime type identification
 *          
 *          Integration:
 *          - Managed by AP_Filters singleton which handles up to AP_FILTER_NUM_FILTERS instances
 *          - Parameters loaded from EEPROM via AP_Param::load_object_from_eeprom()
 *          - Backend allocation occurs only when vehicle is unarmed for safety
 * 
 * @note Subclasses should override setup methods for their specific filter type
 * @see AP_NotchFilter_params for notch filter implementation example
 * @see AP_Filters for the singleton manager that controls filter instances
 */
class AP_Filter {
public:
    /**
     * @enum FilterType
     * @brief Enumeration of supported filter types
     * 
     * @details Defines the available filter types that can be configured through
     *          the parameter system. Each type corresponds to a different filter
     *          implementation.
     */
    enum class FilterType : uint8_t {
        FILTER_NONE            = 0,  ///< No filtering applied (passthrough)
        FILTER_NOTCH           = 1,  ///< Notch filter (removes specific frequency)
    };   

    /**
     * @brief Constructor for filter base class
     * 
     * @param[in] type Filter type from FilterType enumeration
     * 
     * @note Sets the filter type but does not initialize the filter backend
     * @see init() for filter initialization
     */
    AP_Filter(FilterType type);

    /**
     * @brief Initialize the filter instance
     * 
     * @details Performs filter initialization. Specific initialization behavior
     *          depends on the filter type and is implemented by subclasses.
     * 
     * @note Called by AP_Filters::update() after backend allocation
     */
    void init();

    /**
     * @brief Configure a notch filter from parameter settings
     * 
     * @details Virtual method to set up a NotchFilterFloat instance with parameters
     *          from the AP_Param system. Base class returns false; notch filter
     *          implementations override to configure actual filter parameters.
     * 
     * @param[out] filter NotchFilterFloat instance to configure
     * @param[in] sample_rate Sample rate of the data to be filtered in Hz
     * 
     * @return true if filter was successfully configured, false otherwise
     * 
     * @note Base implementation returns false (no-op)
     * @note Subclasses like AP_NotchFilter_params override this to implement actual configuration
     * 
     * @see AP_NotchFilter_params::setup_notch_filter() for implementation example
     */
    virtual bool setup_notch_filter(NotchFilterFloat& filter, float sample_rate) { return false; }

    FilterType _type;  ///< Filter type identifier from FilterType enumeration
};

/**
 * @struct AP_Filter_params
 * @brief Parameter container for basic filter configuration
 * 
 * @details This structure holds the AP_Param-managed parameters for filter configuration.
 *          It contains the filter type selection parameter that determines which filter
 *          backend will be instantiated.
 *          
 *          Parameter Integration:
 *          - Integrates with AP_Param::GroupInfo through var_info[] table
 *          - Filter type parameter (_type) persisted to EEPROM
 *          - Used by AP_Filters manager to determine backend allocation
 *          
 *          Usage Pattern:
 *          - AP_Filters creates an array of AP_Filter_params[AP_FILTER_NUM_FILTERS]
 *          - Parameters loaded via AP_Param::load_object_from_eeprom()
 *          - Type selection triggers appropriate backend allocation
 * 
 * @note This is a base parameter structure; specific filter types have their own parameter structs
 * @see AP_NotchFilter_params for notch filter parameter implementation
 * @see AP_Param::GroupInfo for parameter metadata system
 */
struct AP_Filter_params {
public:
    /**
     * @brief Constructor for filter parameter container
     * 
     * @details Initializes parameter structure with default values
     */
    AP_Filter_params();

    /**
     * @brief Parameter metadata table for AP_Param system
     * 
     * @details Contains GroupInfo entries defining parameter names, types,
     *          offsets, and metadata for ground station display and EEPROM storage.
     */
    static const struct AP_Param::GroupInfo var_info[];

    AP_Enum<AP_Filter::FilterType> _type;  ///< Filter type selection parameter (FILTER_NONE, FILTER_NOTCH, etc.)
};

/**
 * @struct AP_NotchFilter_params
 * @brief Notch filter implementation with AP_Param-managed configuration parameters
 * 
 * @details This structure extends AP_Filter to provide a complete notch filter implementation
 *          with runtime-configurable parameters persisted through the AP_Param system.
 *          
 *          Notch Filter Characteristics:
 *          - Removes energy at a specific center frequency
 *          - Configurable bandwidth via quality factor (Q)
 *          - Adjustable attenuation depth
 *          
 *          Parameters:
 *          - _center_freq_hz: Target frequency to attenuate in Hz
 *          - _quality: Quality factor (Q) - higher values = narrower notch
 *          - _attenuation_dB: Attenuation depth in decibels (negative values)
 *          
 *          Implementation:
 *          - Implements setup_notch_filter() to configure NotchFilterFloat instances
 *          - Reads parameters from EEPROM on initialization
 *          - Applies configuration to filter instances on setup
 *          
 *          Usage Pattern:
 *          1. Parameters configured via ground station or parameter files
 *          2. AP_Filters manager calls setup_notch_filter() with target filter instance
 *          3. Method configures NotchFilterFloat with stored parameter values
 *          4. Filter ready for use with apply() method
 * 
 * @note Inherits from AP_Filter base class, not AP_Filter_params
 * @note Backend allocated only when vehicle is unarmed
 * @see NotchFilterFloat for the underlying filter implementation
 * @see AP_Filter::setup_notch_filter() for base virtual method
 */
struct AP_NotchFilter_params : public AP_Filter {
public:
    /**
     * @brief Constructor for notch filter parameters
     * 
     * @details Initializes notch filter parameter structure with default values.
     *          Sets filter type to FILTER_NOTCH and initializes parameter defaults.
     */
    AP_NotchFilter_params();

    /**
     * @brief Configure a NotchFilterFloat instance from stored parameters
     * 
     * @details Reads the center frequency, quality factor, and attenuation parameters
     *          and applies them to configure the provided NotchFilterFloat instance.
     *          
     *          Configuration Process:
     *          - Validates parameter values (frequency > 0, quality > 0)
     *          - Calculates filter coefficients based on sample rate
     *          - Applies configuration to filter instance
     * 
     * @param[out] filter NotchFilterFloat instance to configure
     * @param[in] sample_rate Sample rate of data to be filtered in Hz
     * 
     * @return true if filter was successfully configured with valid parameters, false otherwise
     * 
     * @note Returns false if parameters are invalid (e.g., zero frequency or negative quality)
     * @note Filter must be reinitialized if sample rate changes
     * 
     * @warning Changing parameters during flight may cause transients in filtered signals
     * 
     * @see NotchFilterFloat::init() for underlying filter initialization
     */
    bool setup_notch_filter(NotchFilterFloat& filter, float sample_rate) override;

    /**
     * @brief Parameter metadata table for AP_Param system
     * 
     * @details Contains GroupInfo entries for notch filter parameters including
     *          center frequency, quality factor, and attenuation, with metadata
     *          for ground station display and EEPROM storage.
     */
    static const struct AP_Param::GroupInfo var_info[];

    AP_Float _center_freq_hz;     ///< Center frequency to notch out in Hz (e.g., 60.0 for 60Hz noise)
    AP_Float _quality;            ///< Quality factor Q (dimensionless) - higher = narrower notch (typical: 5-20)
    AP_Float _attenuation_dB;     ///< Attenuation at center frequency in dB (negative value, e.g., -40.0)
};

/**
 * @class AP_Filters
 * @brief Singleton manager for multiple configurable filter instances
 * 
 * @details This singleton class manages an array of AP_FILTER_NUM_FILTERS filter instances,
 *          providing centralized configuration, parameter loading, and runtime management.
 *          
 *          Key Responsibilities:
 *          - Manages up to AP_FILTER_NUM_FILTERS filter instances (typically 4 or 8)
 *          - Handles dynamic backend allocation based on filter type parameters
 *          - Loads filter configurations from EEPROM via AP_Param system
 *          - Provides 1Hz update loop to process configuration changes
 *          - Enforces safe allocation policy (only when vehicle is unarmed)
 *          
 *          Runtime Allocation Strategy:
 *          - Filter backends allocated dynamically based on parameter _type
 *          - Allocation only occurs when hal.util->get_soft_armed() returns false
 *          - Prevents heap allocation during flight for safety
 *          - Once allocated, backends persist until parameter change
 *          
 *          Parameter Integration:
 *          - Each filter has parameters in params[AP_FILTER_NUM_FILTERS] array
 *          - Parameters loaded via AP_Param::load_object_from_eeprom()
 *          - Backend-specific parameters registered via backend_var_info[] pointers
 *          - Ground stations can configure via parameter protocol
 *          
 *          Configuration Process:
 *          1. Parameters loaded from EEPROM at boot
 *          2. update() called at 1Hz checks for configuration changes
 *          3. When unarmed, allocates appropriate backend based on _type
 *          4. Backend init() and setup methods called
 *          5. Filter ready for use via get_filter()
 *          
 *          Singleton Access:
 *          - Use AP::filters() accessor from AP namespace
 *          - get_singleton() returns pointer to singleton instance
 *          
 * @note Number of filters defined by AP_FILTER_NUM_FILTERS from AP_Filter_config.h
 * @note Backends only allocated when vehicle is unarmed (safety constraint)
 * 
 * @warning Singleton creation enforces single instance with panic on violation (SITL)
 * @warning Do not allocate backends during armed state to avoid heap allocation in flight
 * 
 * @see AP::filters() for preferred singleton accessor
 * @see AP_Filter for base filter interface
 * @see AP_Filter_config.h for AP_FILTER_NUM_FILTERS definition
 */
class AP_Filters {
public:
    /**
     * @brief Constructor for filter manager singleton
     * 
     * @details Initializes the filter array and parameter structures.
     *          Sets up singleton pointer and prepares for parameter loading.
     * 
     * @note Only one instance should be created (enforced by singleton pattern)
     */
    AP_Filters();

    CLASS_NO_COPY(AP_Filters);

    /**
     * @brief Get pointer to the singleton instance
     * 
     * @return Pointer to the AP_Filters singleton instance, or nullptr if not created
     * 
     * @note Prefer using AP::filters() accessor instead of calling this directly
     * @see AP::filters() for the preferred access method
     */
    static AP_Filters *get_singleton(void) { return singleton; }

    /**
     * @brief Initialize the filter manager
     * 
     * @details Performs initial setup of the filter manager including
     *          parameter loading from EEPROM and initial backend allocation
     *          if vehicle is unarmed.
     * 
     * @note Called during system initialization
     * @note Backend allocation deferred if vehicle is armed
     */
    void init();

    /**
     * @brief Process configuration changes and manage filter backends
     * 
     * @details Called at 1Hz to monitor parameter changes and allocate/deallocate
     *          filter backends as needed. Backend allocation only occurs when
     *          the vehicle is unarmed (hal.util->get_soft_armed() == false).
     *          
     *          Update Process:
     *          1. Check if vehicle is armed (skip allocation if armed)
     *          2. Check each filter's _type parameter
     *          3. Allocate appropriate backend if type changed
     *          4. Call backend init() and setup methods
     *          5. Free backends if type changed to FILTER_NONE
     * 
     * @note Called at 1Hz (once per second) to process configuration changes
     * @note Backend allocation only when vehicle is unarmed for safety
     * 
     * @warning Do not call at high rate as it performs heap allocation checks
     */
    void update();

    /**
     * @brief Parameter metadata table for AP_Param system
     * 
     * @details Contains GroupInfo entries for the filter manager and array of
     *          filter parameter structures, enabling EEPROM persistence and
     *          ground station configuration.
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Array of pointers to backend-specific parameter metadata
     * 
     * @details Static array of GroupInfo pointers for backend parameter registration.
     *          Each filter backend type registers its parameter structure here to
     *          enable parameter persistence for dynamically allocated backends.
     * 
     * @note Size is AP_FILTER_NUM_FILTERS to match filter array size
     */
    static const struct AP_Param::GroupInfo *backend_var_info[AP_FILTER_NUM_FILTERS];

    /**
     * @brief Retrieve a filter instance by index
     * 
     * @param[in] filt_num Filter index from 0 to AP_FILTER_NUM_FILTERS-1
     * 
     * @return Pointer to AP_Filter instance, or nullptr if invalid index or not allocated
     * 
     * @note Returns nullptr if filt_num >= AP_FILTER_NUM_FILTERS
     * @note Returns nullptr if backend not yet allocated (e.g., vehicle armed or type is FILTER_NONE)
     * 
     * @warning Always check for nullptr before using returned pointer
     */
    AP_Filter* get_filter(uint8_t filt_num);

private:
    AP_Filter* filters[AP_FILTER_NUM_FILTERS];         ///< Array of filter backend pointers (nullptr if not allocated)
    AP_Filter_params params[AP_FILTER_NUM_FILTERS];    ///< Array of filter parameter structures

    static AP_Filters *singleton;  ///< Singleton instance pointer
};

/**
 * @namespace AP
 * @brief ArduPilot namespace for singleton accessors
 * 
 * @details The AP namespace provides convenient accessor functions for ArduPilot
 *          singleton instances, including the filter manager.
 */
namespace AP {
    /**
     * @brief Access the filter manager singleton
     * 
     * @details Returns a reference to the AP_Filters singleton instance for managing
     *          configurable filter instances. This is the preferred method for accessing
     *          the filter manager throughout ArduPilot code.
     *          
     *          Usage Example:
     *          @code
     *          AP_Filter* filt = AP::filters().get_filter(0);
     *          if (filt != nullptr) {
     *              // Use filter
     *          }
     *          @endcode
     * 
     * @return Reference to AP_Filters singleton instance
     * 
     * @note This is the preferred accessor method over AP_Filters::get_singleton()
     * @see AP_Filters for filter manager documentation
     */
    AP_Filters &filters();
};

#endif // AP_FILTER_ENABLED

