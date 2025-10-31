/**
 * @file    AP_Rally.h
 * @brief   Rally point management system for safe return-to-launch operations
 * 
 * @details The AP_Rally library provides a comprehensive rally point management
 *          system that enables vehicles to identify and navigate to pre-defined
 *          safe landing locations during Return-To-Launch (RTL) operations.
 *          
 *          Rally points are alternative landing sites stored in non-volatile memory
 *          that can be used instead of the home location for emergency returns.
 *          This is particularly useful for:
 *          - Vehicles operating in areas where the launch location is unsuitable for landing
 *          - Multi-site operations where the closest safe landing point may vary
 *          - Preventing accidental RTL to distant airfields when rally points from previous flights remain
 *          
 *          Architecture Overview:
 *          - Storage: Rally points are persisted in EEPROM via StorageManager
 *          - Retrieval: Index-based access to individual rally points with validation
 *          - Selection: Intelligent nearest-point algorithms with distance limiting
 *          - RTL Integration: Automatic best rally point selection during emergency returns
 *          
 *          The library implements singleton pattern for global access and includes
 *          safety mechanisms such as distance limiting (RALLY_LIMIT_KM) to prevent
 *          unintended use of rally points from different operating locations.
 * 
 * @note Initial implementation: Michael Day, September 2013
 * @note Moved to AP_Rally lib: Andrew Chapman, April 2014
 * 
 * @see Location
 * @see AP_Param
 * @see StorageManager
 */
#pragma once

#include "AP_Rally_config.h"

#if HAL_RALLY_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

/**
 * @struct RallyLocation
 * @brief  Storage structure for a single rally point with location and behavior flags
 * 
 * @details This structure defines the complete specification of a rally point including
 *          geographic coordinates, altitude information, landing parameters, and behavioral
 *          flags. Rally points are stored in non-volatile memory using this packed format
 *          to minimize storage requirements.
 *          
 *          Coordinate System:
 *          - Latitude and longitude use scaled integer representation (degrees * 1e7) for
 *            precision without floating-point storage overhead
 *          - Altitude can be specified in different frames (ABOVE_HOME or ABSOLUTE) via flags
 *          
 *          Landing Behavior:
 *          - Supports configurable landing direction for runway alignment or wind consideration
 *          - Optional automatic landing sequence after reaching rally point
 *          - Break altitude for transitioning from loiter to landing approach
 *          
 * @note The PACKED attribute ensures consistent memory layout across platforms for reliable
 *       storage and retrieval. Total structure size is 15 bytes (verified by ASSERT_STORAGE_SIZE).
 * @note Structure size is critical for storage capacity calculations in get_rally_max()
 * 
 * @see Location::AltFrame
 * @see AP_Rally::rally_location_to_location()
 */
struct PACKED RallyLocation {
    int32_t lat;        ///< Latitude in degrees * 1e7 (WGS84 datum)
    int32_t lng;        ///< Longitude in degrees * 1e7 (WGS84 datum)
    int16_t alt;        ///< Transit and loiter altitude in meters (frame specified by alt_frame flags)
    int16_t break_alt;  ///< Break altitude in meters - transition from loiter to landing at this altitude
    uint16_t land_dir;  ///< Preferred landing direction in centidegrees (0-35999, where 0 = North)
    
    /**
     * @brief Behavioral flags and altitude frame specification
     * 
     * @details This union allows access to the flags both as a single byte (for storage/transfer)
     *          and as individual bitfields (for logical operations). The bitfield provides:
     *          - Wind consideration for landing site selection
     *          - Automatic landing sequence triggering
     *          - Altitude frame interpretation (ABOVE_HOME vs ABSOLUTE vs other Location::AltFrame values)
     */
    union {
        uint8_t flags;  ///< Packed byte representation of all flags
        struct {
            uint8_t favorable_winds : 1; ///< Bit 0: Seek favorable winds when choosing landing point
            uint8_t do_auto_land    : 1; ///< Bit 1: Execute automatic landing sequence after reaching rally point
            uint8_t alt_frame_valid : 1; ///< Bit 2: If true, use alt_frame field; if false, default to Location::AltFrame::ABOVE_HOME
            uint8_t alt_frame       : 2; ///< Bits 3-4: Altitude frame from Location::AltFrame enum (ABSOLUTE, ABOVE_HOME, ABOVE_ORIGIN, ABOVE_TERRAIN)
            uint8_t unused          : 3; ///< Bits 5-7: Reserved for future use
        };
    };
};

/**
 * @class AP_Rally
 * @brief Rally point management system for safe return-to-launch operations
 * 
 * @details AP_Rally provides comprehensive management of rally points - pre-defined safe
 *          landing locations that serve as alternatives to the home position during
 *          Return-To-Launch (RTL) operations. The class handles persistent storage,
 *          retrieval, validation, and intelligent selection of rally points.
 *          
 *          Core Responsibilities:
 *          - **Storage Management**: Persists rally points to EEPROM via StorageManager
 *          - **Point Retrieval**: Index-based access with bounds checking and validation
 *          - **Selection Algorithms**: Intelligent nearest-point selection with distance limits
 *          - **RTL Integration**: Calculates optimal return destination considering rally points and home
 *          - **Change Tracking**: Monitors modifications for ground station synchronization
 *          
 *          Rally Point Selection Logic:
 *          The calc_best_rally_or_home_location() method implements sophisticated logic to
 *          choose between rally points and home location:
 *          1. Find nearest rally point to current position
 *          2. Check if nearest rally point is within RALLY_LIMIT_KM
 *          3. Compare distances to rally point vs home location
 *          4. Select closest valid destination considering RALLY_INCL_HOME setting
 *          
 *          Distance Limiting (RALLY_LIMIT_KM):
 *          This critical safety parameter prevents unintended use of rally points from
 *          different operating locations. If the nearest rally point exceeds the limit
 *          and home is closer, RTL will use home instead of the distant rally point.
 *          
 *          Storage Architecture:
 *          Rally points are stored in the StorageManager::StorageRally area. The maximum
 *          number of points is determined by available storage divided by sizeof(RallyLocation).
 *          
 *          Thread Safety:
 *          This class uses singleton pattern (accessed via AP::rally()). Access methods
 *          are const-correct but do not provide internal locking. Modifications should
 *          occur only from the main thread or via GCS MAVLink handlers.
 * 
 * @note Access via singleton: AP::rally() or AP_Rally::get_singleton()
 * @note Rally points are 1-indexed in some GCS protocols but 0-indexed in this implementation
 * 
 * @warning Modifying rally points during flight can affect safe return behavior
 * @warning RALLY_LIMIT_KM must be set appropriately for operating environment to prevent
 *          accidental RTL to distant airfields
 * 
 * @see RallyLocation
 * @see Location
 * @see StorageManager
 */
class AP_Rally {
public:
    AP_Rally();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Rally);

    /**
     * @brief Retrieve a rally point by index
     * 
     * @details Retrieves a rally point from persistent storage at the specified index.
     *          Performs bounds checking to ensure the index is valid (less than total count).
     *          If successful, populates the provided RallyLocation structure with data
     *          read from EEPROM via StorageManager.
     * 
     * @param[in]  i    Zero-based index of rally point to retrieve (0 to get_rally_total()-1)
     * @param[out] ret  RallyLocation structure to populate with rally point data
     * 
     * @return true if rally point successfully retrieved, false if index out of bounds
     * 
     * @note Rally points are stored contiguously in memory, so valid indices are 0 through N-1
     * @note This method does not validate the rally point coordinates, only the index
     * 
     * @see set_rally_point_with_index()
     * @see get_rally_total()
     */
    bool get_rally_point_with_index(uint8_t i, RallyLocation &ret) const;
    
    /**
     * @brief Store a rally point at the specified index
     * 
     * @details Writes a rally point to persistent EEPROM storage at the specified index.
     *          Performs bounds checking and validation before writing. Updates the
     *          last_change_time_ms to notify ground stations of modifications.
     * 
     * @param[in] i         Zero-based index where rally point should be stored
     * @param[in] rallyLoc  Rally point data to write to storage
     * 
     * @return true if rally point successfully stored, false if index out of bounds or invalid
     * 
     * @warning This method writes to EEPROM, which has limited write cycles. Avoid
     *          frequent modifications during flight operations.
     * @warning Rally point modifications affect RTL destination selection and safe return behavior
     * @warning Index must be less than get_rally_max() to prevent storage overflow
     * 
     * @note Updates _last_change_time_ms for ground station synchronization
     * @note Virtual is_valid() method is called for vehicle-specific validation
     * 
     * @see get_rally_point_with_index()
     * @see last_change_time_ms()
     */
    bool set_rally_point_with_index(uint8_t i, const RallyLocation &rallyLoc);
    
    /**
     * @brief Get the current number of rally points stored
     * 
     * @details Returns the count of rally points currently loaded in the system.
     *          This count is maintained in the RALLY_TOTAL parameter and persisted
     *          across reboots.
     * 
     * @return Number of rally points (0-255)
     * 
     * @note Return value is always less than or equal to get_rally_max()
     * @note Count is stored in _rally_point_total_count parameter
     * 
     * @see get_rally_max()
     * @see truncate()
     */
    uint8_t get_rally_total() const {
        return (uint8_t)_rally_point_total_count;
    }
    
    /**
     * @brief Get the maximum number of rally points that can be stored
     * 
     * @details Calculates the maximum rally point capacity based on available storage
     *          space in the StorageManager::StorageRally area. The capacity is determined
     *          by dividing total storage size by sizeof(RallyLocation) which is 15 bytes.
     * 
     * @return Maximum number of rally points that can be stored (0-255, capped at 255)
     * 
     * @note Actual capacity depends on board configuration and storage allocation
     * @note Return value is capped at 255 even if more storage is available (uint8_t index limit)
     * @note Typical capacity ranges from 10-100 rally points depending on hardware
     * 
     * @see get_rally_total()
     * @see RallyLocation (storage structure size)
     */
    uint8_t get_rally_max(void) const {
        const uint16_t ret = _storage.size() / uint16_t(sizeof(RallyLocation));
        if (ret > 255) {
            return 255;
        }
        return (uint8_t)ret;
    }
    
    /**
     * @brief Reduce the number of rally points to the specified count
     * 
     * @details Truncates the rally point list to the specified number by updating
     *          the RALLY_TOTAL parameter. Does not erase data from storage, but
     *          points beyond the new count are no longer accessible. Updates
     *          last_change_time_ms to notify ground stations.
     * 
     * @param[in] num  New rally point count (must be <= current count)
     * 
     * @note This is a non-destructive operation - data remains in storage and can be
     *       recovered by increasing RALLY_TOTAL again
     * @note Updates _last_change_time_ms for ground station synchronization
     * 
     * @see get_rally_total()
     * @see append()
     */
    void truncate(uint8_t num);
    
    /**
     * @brief Append a new rally point to the end of the list
     * 
     * @details Adds a rally point to the end of the current rally point list if space
     *          is available. Performs validation and checks that the list is not already
     *          at maximum capacity before adding.
     * 
     * @param[in] loc  Rally point location and parameters to append
     * 
     * @return true if rally point successfully appended, false if list is full or invalid
     * 
     * @warning Writes to EEPROM with limited write cycle lifetime
     * @warning Adding rally points during flight affects RTL destination selection
     * 
     * @note Increments _rally_point_total_count on success
     * @note Updates _last_change_time_ms for ground station synchronization
     * 
     * @see set_rally_point_with_index()
     * @see get_rally_max()
     */
    bool append(const RallyLocation &loc) WARN_IF_UNUSED;

    /**
     * @brief Get the rally point distance limit in kilometers
     * 
     * @details Returns the RALLY_LIMIT_KM parameter value, which specifies the maximum
     *          distance to a rally point before it is considered too far. This safety
     *          mechanism prevents unintended use of rally points from different operating
     *          locations (e.g., leftover rally points from a different airfield).
     *          
     *          If the nearest rally point is farther than this limit and home is closer,
     *          RTL will navigate to home instead of the rally point. Setting to 0 disables
     *          distance limiting and always uses the nearest rally point.
     * 
     * @return Rally point distance limit in kilometers (0 = unlimited)
     * 
     * @warning Setting this value too high may cause RTL to distant rally points from
     *          previous flights at different locations
     * @warning Setting to 0 removes safety check for distant rally points
     * 
     * @note Default values vary by vehicle type (Copter: 0.3km, Plane: 5.0km, Rover: 0.5km)
     * 
     * @see calc_best_rally_or_home_location()
     * @see find_nearest_rally_point()
     */
    float get_rally_limit_km() const { return _rally_limit_km; }

    /**
     * @brief Convert RallyLocation storage format to standard Location
     * 
     * @details Converts the compact RallyLocation storage structure to the standard
     *          ArduPilot Location format used throughout the flight code. Handles
     *          coordinate scaling (degrees * 1e7 to degrees), altitude frame conversion,
     *          and flag interpretation.
     *          
     *          Altitude Frame Handling:
     *          - If alt_frame_valid flag is set, uses the alt_frame field value
     *          - Otherwise defaults to Location::AltFrame::ABOVE_HOME
     *          - Supports ABSOLUTE, ABOVE_HOME, ABOVE_ORIGIN, and ABOVE_TERRAIN frames
     * 
     * @param[in] ret  RallyLocation structure in storage format
     * 
     * @return Location structure with properly scaled coordinates and altitude frame
     * 
     * @note This is a const method that does not modify the rally point
     * @note Altitude frame affects how navigation interprets the altitude value
     * 
     * @see RallyLocation
     * @see Location::AltFrame
     * @see calc_best_rally_or_home_location()
     */
    Location rally_location_to_location(const RallyLocation &ret) const;

    /**
     * @brief Calculate the best return destination (rally point or home)
     * 
     * @details Implements the core rally point selection algorithm for RTL operations.
     *          Determines whether to return to a rally point or home location based on:
     *          - Current vehicle position
     *          - Distance to nearest rally point
     *          - Distance to home location
     *          - RALLY_LIMIT_KM distance threshold
     *          - RALLY_INCL_HOME parameter setting
     *          
     *          Selection Algorithm:
     *          1. Find nearest rally point using find_nearest_rally_point()
     *          2. Check if nearest rally is within RALLY_LIMIT_KM
     *          3. If RALLY_INCL_HOME enabled, compare distances to rally vs home
     *          4. Select destination with shorter distance (if rally within limit)
     *          5. Set altitude to rtl_home_alt_amsl_cm parameter
     * 
     * @param[in] current_loc         Current vehicle position (lat, lng, alt)
     * @param[in] rtl_home_alt_amsl_cm RTL altitude in centimeters AMSL (Above Mean Sea Level)
     * 
     * @return Location of best return destination with RTL altitude set
     * 
     * @warning This method directly affects safe return behavior during emergencies
     * @warning Incorrect RALLY_LIMIT_KM can cause RTL to unsuitable locations
     * 
     * @note Returns home location if no valid rally points exist or all exceed distance limit
     * @note Altitude in returned Location is set to rtl_home_alt_amsl_cm regardless of source
     * 
     * @see find_nearest_rally_point()
     * @see get_rally_limit_km()
     */
    Location calc_best_rally_or_home_location(const Location &current_loc, float rtl_home_alt_amsl_cm) const;
    
    /**
     * @brief Find the nearest rally point to the specified location
     * 
     * @details Searches all rally points to find the one with minimum distance from
     *          the specified location. Performs distance calculations in 2D (horizontal
     *          plane) using the Location::get_distance() method. Does not apply
     *          RALLY_LIMIT_KM filtering - that is done by calc_best_rally_or_home_location().
     * 
     * @param[in]  myloc  Reference location to measure distances from
     * @param[out] ret    RallyLocation structure populated with nearest rally point data
     * 
     * @return true if a rally point was found, false if no rally points exist
     * 
     * @note Distance calculation uses horizontal 2D distance (ignores altitude difference)
     * @note Does not validate rally point coordinates or apply distance limits
     * @note Returns false immediately if get_rally_total() is 0
     * 
     * @see calc_best_rally_or_home_location()
     * @see get_rally_point_with_index()
     */
    bool find_nearest_rally_point(const Location &myloc, RallyLocation &ret) const;

    /**
     * @brief Get timestamp of last rally point modification
     * 
     * @details Returns the system time in milliseconds when rally points were last
     *          modified (added, changed, or deleted). Used by ground control stations
     *          to detect when they need to re-download the rally point list for
     *          synchronization.
     * 
     * @return Timestamp in milliseconds since system boot (AP_HAL::millis())
     * 
     * @note Initialized to 0xFFFFFFFF on startup to indicate "unknown" state
     * @note Updated by set_rally_point_with_index(), append(), and truncate()
     * @note Ground stations can compare this with their last sync time to detect changes
     * 
     * @see set_rally_point_with_index()
     * @see append()
     * @see truncate()
     */
    uint32_t last_change_time_ms(void) const { return _last_change_time_ms; }

    /**
     * @brief Parameter group definition for AP_Param system
     * 
     * @details Defines the rally point parameters that are stored persistently and
     *          accessible via ground control stations:
     *          
     *          - RALLY_TOTAL (index 0): Number of rally points currently loaded
     *          - RALLY_LIMIT_KM (index 1): Maximum distance to rally point in kilometers
     *          - RALLY_INCL_HOME (index 2): Whether to include home as a rally point option
     *          
     *          These parameters are defined with @Param tags in AP_Rally.cpp for automatic
     *          documentation generation and ground station integration.
     * 
     * @note Parameter indices must remain stable across firmware versions for compatibility
     * @note Modifications to this structure require parameter table versioning consideration
     * 
     * @see AP_Param::GroupInfo
     * @see AP_Rally.cpp (lines 29-52 for @Param definitions)
     */
    static const struct AP_Param::GroupInfo var_info[];

    /**
     * @brief Get singleton instance of AP_Rally
     * 
     * @details Returns pointer to the singleton AP_Rally instance. The singleton pattern
     *          ensures only one rally point manager exists per vehicle, providing consistent
     *          rally point state across all flight code.
     * 
     * @return Pointer to AP_Rally singleton instance, or nullptr if not initialized
     * 
     * @note Prefer using AP::rally() namespace accessor for cleaner syntax
     * @note Singleton is initialized during vehicle startup
     * 
     * @see AP::rally()
     */
    static AP_Rally *get_singleton() { return _singleton; }


private:
    static AP_Rally *_singleton;

    virtual bool is_valid(const Location &rally_point) const { return true; }

    static StorageAccess _storage;

    // parameters
    AP_Int8  _rally_point_total_count;
    AP_Float _rally_limit_km;
    AP_Int8  _rally_incl_home;

    uint32_t _last_change_time_ms = 0xFFFFFFFF;
};

/**
 * @namespace AP
 * @brief ArduPilot singleton accessor namespace
 */
namespace AP {
    /**
     * @brief Global accessor for AP_Rally singleton instance
     * 
     * @details Provides convenient access to the rally point manager from anywhere
     *          in the flight code. This is the preferred method for accessing rally
     *          point functionality rather than using AP_Rally::get_singleton().
     *          
     *          Example usage:
     *          @code
     *          Location best_destination = AP::rally()->calc_best_rally_or_home_location(current_loc, rtl_alt);
     *          uint8_t num_rally_points = AP::rally()->get_rally_total();
     *          @endcode
     * 
     * @return Pointer to AP_Rally singleton instance
     * 
     * @note Returns nullptr if rally point system is disabled (HAL_RALLY_ENABLED=0)
     * @note Implemented in AP_Rally.cpp
     * 
     * @see AP_Rally::get_singleton()
     */
    AP_Rally *rally();
};

#endif  // HAL_RALLY_ENABLED
