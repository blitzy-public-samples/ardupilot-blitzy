/**
 * @file AC_PolyFence_loader.h
 * @brief Polygon and circle fence storage, indexing, and breach detection system
 * 
 * @details This file implements the polygon and circular fence management system for ArduPilot,
 *          providing persistent storage, efficient indexing, and geometric breach detection
 *          for inclusion and exclusion zones.
 * 
 *          Key features:
 *          - Persistent storage of polygon and circle fences to EEPROM or SD card
 *          - Efficient indexing system for fast fence lookup and validation
 *          - Coordinate transformation from lat/lon to NE offset from EKF origin
 *          - Geometric algorithms for point-in-polygon and circle distance calculations
 *          - Thread-safe access via HAL_Semaphore
 *          - Support for MAVLink fence upload/download protocol
 * 
 *          Storage Format:
 *          Fences are stored in a sequential format with type markers and vertex counts.
 *          Each fence item includes its type (polygon/circle, inclusion/exclusion), followed
 *          by the fence data (lat/lon points for polygons, center+radius for circles).
 * 
 *          Coordinate Frames:
 *          - Storage: Latitude/longitude in 1e-7 degrees (Vector2l)
 *          - Runtime: NE offsets in centimeters from EKF origin (Vector2f)
 *          - Circle radius: Stored and returned in meters (float)
 * 
 * @note This is a safety-critical component - fence breaches trigger failsafe actions
 * @warning Fence configuration must ensure vehicle has valid operational areas
 * 
 * @author ArduPilot Development Team
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 */

#pragma once

#include "AC_Fence_config.h"
#include <AP_Math/AP_Math.h>

// CIRCLE_INCLUSION_INT stores the radius an a 32-bit integer in
// metres.  This was a bug, and CIRCLE_INCLUSION was created to store
// as a 32-bit float instead.  We save as _INT in the case that the
// radius looks like an integer as a backwards-compatibility measure.
// For 4.2 we might consider only loading _INT and always saving as
// float, and in 4.3 considering _INT invalid

// CODE_REMOVAL
// ArduPilot 4.7 no longer stores circle radiuses that look like
//   integers as integer item types, so any time a fence is saved the
//   use of the deprecated types is fixed.
// ArduPilot 4.8 warns if it loads an integer item, warns user to re-upload the fence
// ArduPilot 4.9 warns if it loads an integer item, warns user to re-upload the fence
// ArduPilot 4.10 removes support for them

/**
 * @enum AC_PolyFenceType
 * @brief Fence type identifiers used for storage and indexing of polygon and circular fences
 * 
 * @details These type markers are stored in persistent storage to identify different
 *          fence types. Each fence in storage begins with a type marker followed by
 *          the fence data (vertex count and points for polygons, or center and radius
 *          for circles).
 * 
 *          Inclusion zones: Vehicle must remain INSIDE these boundaries
 *          Exclusion zones: Vehicle must remain OUTSIDE these boundaries
 * 
 * @note Type values are chosen to be out-of-band for legacy fence storage formats
 * @warning Changing these values will break compatibility with stored fences
 */
enum class AC_PolyFenceType : uint8_t {
    END_OF_STORAGE        = 99,  ///< Marker indicating end of fence storage data
    POLYGON_INCLUSION     = 98,  ///< Polygonal inclusion zone - vehicle must stay inside
    POLYGON_EXCLUSION     = 97,  ///< Polygonal exclusion zone - vehicle must stay outside
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    CIRCLE_EXCLUSION_INT  = 96,  ///< @deprecated Circular exclusion with integer radius (meters) - legacy format
#endif  // AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    RETURN_POINT          = 95,  ///< Designated return point for fence breach recovery (lat/lon)
#if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    CIRCLE_INCLUSION_INT  = 94,  ///< @deprecated Circular inclusion with integer radius (meters) - legacy format
#endif // #if AC_POLYFENCE_CIRCLE_INT_SUPPORT_ENABLED
    CIRCLE_EXCLUSION      = 93,  ///< Circular exclusion zone with float radius (meters) - vehicle must stay outside
    CIRCLE_INCLUSION      = 92,  ///< Circular inclusion zone with float radius (meters) - vehicle must stay inside
};

/**
 * @class AC_PolyFenceItem
 * @brief Data structure for passing fence information to and from the polyfence loader
 * 
 * @details AC_PolyFenceItem provides a generic container for fence data exchange between
 *          the fence loader and external systems (e.g., MAVLink protocol handlers).
 *          The structure uses a discriminated union pattern where the 'type' field
 *          determines which other fields are valid.
 * 
 *          Field usage by fence type:
 *          - POLYGON_INCLUSION/EXCLUSION: type, loc (first vertex), vertex_count
 *          - CIRCLE_INCLUSION/EXCLUSION: type, loc (center), radius
 *          - RETURN_POINT: type, loc
 *          - END_OF_STORAGE: type only
 * 
 *          Coordinate format:
 *          - loc: Latitude/longitude in 1e-7 degrees (Vector2l: lat, lon)
 *          - radius: Meters (float)
 * 
 * @note Currently uses all fields for all types - future optimization could use union
 * @todo Make this a union or use subclasses to reduce memory footprint
 */
class AC_PolyFenceItem {
public:
    AC_PolyFenceType type;    ///< Fence type identifier
    Vector2l loc;             ///< Location in lat/lon (1e-7 degrees) - meaning depends on type
    uint8_t vertex_count;     ///< Number of vertices (polygons only)
    float radius;             ///< Radius in meters (circles only)
};

#if AP_FENCE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

/**
 * @class AC_PolyFence_loader
 * @brief Manages persistent storage, indexing, and breach detection for polygon and circular fences
 * 
 * @details AC_PolyFence_loader provides comprehensive fence management functionality including:
 * 
 *          Storage Management:
 *          - Persistent storage to EEPROM or SD card (configured via StorageManager)
 *          - Sequential storage format with type markers and fence data
 *          - Format validation and corruption detection
 *          - Atomic fence updates via write_fence()
 * 
 *          Indexing System:
 *          - Efficient in-memory index of fence storage for fast lookups
 *          - Lazy indexing - created on first access, cached for performance
 *          - Index invalidation when storage modified
 * 
 *          Coordinate Transformation:
 *          - Fences stored in lat/lon (Vector2l in 1e-7 degrees)
 *          - Runtime conversion to NE offsets from EKF origin (Vector2f in centimeters)
 *          - Automatic coordinate update when EKF origin changes
 *          - Thread-safe coordinate access via _loaded_fence_sem
 * 
 *          Breach Detection:
 *          - Point-in-polygon detection using ray casting algorithm
 *          - Signed distance calculations for proximity warnings
 *          - Circle intersection and containment checks
 *          - Combined check of all inclusion and exclusion zones
 * 
 *          Fence Lifecycle:
 *          1. init() - Initialize storage subsystem
 *          2. load_from_storage() - Read fences from persistent storage
 *          3. Transform lat/lon to NE offsets from EKF origin
 *          4. update() - Called at 10Hz to maintain coordinate validity
 *          5. breached() - Check vehicle position against all fences
 * 
 *          Thread Safety:
 *          - _loaded_fence_sem protects access to transformed fence coordinates
 *          - Lock semaphore before accessing fence arrays
 *          - MAVLink handlers use semaphore for atomic fence updates
 * 
 *          MAVLink Integration:
 *          - Supports MISSION_ITEM_INT protocol for fence upload/download
 *          - Backward compatibility with legacy FENCE_POINT protocol (conditional)
 * 
 * @note This is a safety-critical component - fence breaches trigger failsafe actions
 * @warning All fence coordinates depend on valid EKF origin - coordinates invalid if origin changes
 * @warning Fence configuration must ensure vehicle has at least one valid operational area
 * @warning Storage corruption can render all fences invalid - format() required for recovery
 * 
 * Source: libraries/AC_Fence/AC_PolyFence_loader.h:1-459
 */
class AC_PolyFence_loader
{

public:

    /**
     * @brief Construct AC_PolyFence_loader with parameter references
     * 
     * @param[in] total     Reference to FENCE_TOTAL parameter (for FENCE_POINT protocol compatibility)
     * @param[in] options   Reference to FENCE_OPTIONS parameter (fence behavior configuration)
     */
    AC_PolyFence_loader(AP_Int8 &total, const AP_Int16 &options) :
        _total(total),
        _options(options) {}

    /* Do not allow copies */
    CLASS_NO_COPY(AC_PolyFence_loader);

    /**
     * @brief Initialize fence storage subsystem
     * 
     * @details Initializes the persistent storage interface and prepares the fence loader
     *          for operation. Must be called once during vehicle startup before any fence
     *          operations.
     * 
     * @note Call this during vehicle initialization, typically in setup()
     */
    void init();

    // methods primarily for MissionItemProtocol_Fence to use:
    
    /**
     * @brief Return the total number of fence items stored in persistent storage
     * 
     * @return Total count of fence items (polygon vertices, circle centers, markers)
     * 
     * @note Used by MAVLink MISSION_COUNT protocol to report fence size to ground station
     */
    uint16_t num_stored_items() const { return _eeprom_item_count; }
    
    /**
     * @brief Retrieve a specific fence item from storage by sequence number
     * 
     * @param[in]  seq  Sequence number (0-based index) of item to retrieve
     * @param[out] item AC_PolyFenceItem structure to receive the fence data
     * 
     * @return true if item retrieved successfully, false if seq is out of range or storage error
     * 
     * @note Used by MAVLink MISSION_REQUEST protocol to send fences to ground station
     */
    bool get_item(const uint16_t seq, AC_PolyFenceItem &item) WARN_IF_UNUSED;

    ///
    /// exclusion polygons
    ///
    
    /**
     * @brief Get the number of exclusion polygon fences currently loaded
     * 
     * @return Count of polygon exclusion zones (vehicle must stay outside these)
     * 
     * @note Returns 0 if fences not loaded or no exclusion polygons defined
     */
    uint8_t get_exclusion_polygon_count() const {
        return _num_loaded_exclusion_boundaries;
    }

    /**
     * @brief Retrieve exclusion polygon vertex array by index
     * 
     * @param[in]  index      Zero-based index of exclusion polygon to retrieve
     * @param[out] num_points Filled with number of vertices in the polygon
     * 
     * @return Pointer to array of Vector2f vertices, or nullptr if index invalid
     * 
     * @details Returns polygon vertices as NE offsets in centimeters from the EKF origin.
     *          The polygon is closed (first and last points connect automatically).
     *          Coordinates are in NE frame (North-East-Down convention):
     *          - x component: North offset in cm
     *          - y component: East offset in cm
     * 
     * @warning Returned pointer is only valid until next load_from_storage() or unload()
     * @warning Coordinates become invalid if EKF origin changes - check via update()
     * @note Thread-safe access requires locking _loaded_fence_sem
     */
    Vector2f* get_exclusion_polygon(uint16_t index, uint16_t &num_points) const;

    /**
     * @brief Get timestamp of last successful fence load from storage
     * 
     * @return System time in milliseconds (from millis()) when fences were loaded, or 0 if not loaded
     * 
     * @note Use to detect when fence configuration has been updated
     */
    uint32_t get_exclusion_polygon_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// inclusion polygons
    ///
    
    /**
     * @brief Get the number of inclusion polygon fences currently loaded
     * 
     * @return Count of polygon inclusion zones (vehicle must stay inside these)
     * 
     * @note Returns 0 if fences not loaded or no inclusion polygons defined
     * @warning If inclusion polygon exists, vehicle MUST remain inside at least one inclusion zone
     */
    uint8_t get_inclusion_polygon_count() const {
        return _num_loaded_inclusion_boundaries;
    }

    /**
     * @brief Retrieve inclusion polygon vertex array by index
     * 
     * @param[in]  index      Zero-based index of inclusion polygon to retrieve
     * @param[out] num_points Filled with number of vertices in the polygon
     * 
     * @return Pointer to array of Vector2f vertices, or nullptr if index invalid
     * 
     * @details Returns polygon vertices as NE offsets in centimeters from the EKF origin.
     *          The polygon is closed (first and last points connect automatically).
     *          Coordinates are in NE frame (North-East-Down convention):
     *          - x component: North offset in cm
     *          - y component: East offset in cm
     * 
     * @warning Returned pointer is only valid until next load_from_storage() or unload()
     * @warning Coordinates become invalid if EKF origin changes - check via update()
     * @warning Vehicle must remain inside at least one inclusion polygon when defined
     * @note Thread-safe access requires locking _loaded_fence_sem
     */
    Vector2f* get_inclusion_polygon(uint16_t index, uint16_t &num_points) const;

    /**
     * @brief Get timestamp of last successful fence load from storage
     * 
     * @return System time in milliseconds (from millis()) when fences were loaded, or 0 if not loaded
     * 
     * @note Use to detect when fence configuration has been updated
     */
    uint32_t get_inclusion_polygon_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// exclusion circles
    ///
    
    /**
     * @brief Get the number of exclusion circle fences currently loaded
     * 
     * @return Count of circular exclusion zones (vehicle must stay outside these)
     * 
     * @note Returns 0 if fences not loaded or no exclusion circles defined
     */
    uint8_t get_exclusion_circle_count() const {
        return _num_loaded_circle_exclusion_boundaries;
    }

    /**
     * @brief Retrieve exclusion circle parameters by index
     * 
     * @param[in]  index         Zero-based index of exclusion circle to retrieve
     * @param[out] center_pos_cm Center position as NE offset in cm from EKF origin
     * @param[out] radius        Radius in meters
     * 
     * @return true if circle retrieved successfully, false if index out of range
     * 
     * @details Returns circle center as NE offset in centimeters from EKF origin.
     *          Coordinates are in NE frame (North-East-Down convention):
     *          - x component: North offset in cm
     *          - y component: East offset in cm
     *          Radius is in meters (not centimeters).
     * 
     * @warning Center coordinates become invalid if EKF origin changes - check via update()
     * @note Thread-safe access requires locking _loaded_fence_sem
     */
    bool get_exclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const;

    /**
     * @brief Get timestamp of last successful fence load from storage
     * 
     * @return System time in milliseconds (from millis()) when fences were loaded, or 0 if not loaded
     * 
     * @note Use to detect when fence configuration has been updated
     */
    uint32_t get_exclusion_circle_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// inclusion circles
    ///
    
    /**
     * @brief Get the number of inclusion circle fences currently loaded
     * 
     * @return Count of circular inclusion zones (vehicle must stay inside these)
     * 
     * @note Returns 0 if fences not loaded or no inclusion circles defined
     * @warning If inclusion circle exists, vehicle MUST remain inside at least one inclusion zone
     */
    uint8_t get_inclusion_circle_count() const {
        return _num_loaded_circle_inclusion_boundaries;
    }

    /**
     * @brief Retrieve inclusion circle parameters by index
     * 
     * @param[in]  index         Zero-based index of inclusion circle to retrieve
     * @param[out] center_pos_cm Center position as NE offset in cm from EKF origin
     * @param[out] radius        Radius in meters
     * 
     * @return true if circle retrieved successfully, false if index out of range
     * 
     * @details Returns circle center as NE offset in centimeters from EKF origin.
     *          Coordinates are in NE frame (North-East-Down convention):
     *          - x component: North offset in cm
     *          - y component: East offset in cm
     *          Radius is in meters (not centimeters).
     * 
     * @warning Center coordinates become invalid if EKF origin changes - check via update()
     * @warning Vehicle must remain inside at least one inclusion circle when defined
     * @note Thread-safe access requires locking _loaded_fence_sem
     */
    bool get_inclusion_circle(uint8_t index, Vector2f &center_pos_cm, float &radius) const;

    /**
     * @brief Check if margin distance is valid for all inclusion circles
     * 
     * @param[in] margin Margin distance in meters to check against circle radii
     * 
     * @return true if margin is less than all inclusion circle radii, false otherwise
     * 
     * @details Used to validate that proximity warnings (FENCE_MARGIN parameter) are
     *          not larger than the inclusion circle radius, which would trigger constant
     *          warnings even at circle center.
     * 
     * @note Returns true if no inclusion circles defined
     */
    bool check_inclusion_circle_margin(float margin) const;

    ///
    /// mavlink
    ///
    
    /**
     * @brief Handle MAVLink fence protocol messages from ground control station
     * 
     * @param[in] link Reference to GCS_MAVLINK instance for sending responses
     * @param[in] msg  MAVLink message to process
     * 
     * @details Processes fence upload/download protocol messages:
     *          - MISSION_COUNT: Begin fence upload sequence
     *          - MISSION_ITEM_INT: Receive fence items
     *          - MISSION_REQUEST: Send fence items to GCS
     *          - MISSION_CLEAR_ALL: Clear all stored fences
     *          - FENCE_FETCH_POINT: Legacy protocol support (conditional)
     *          - FENCE_POINT: Legacy protocol support (conditional)
     * 
     * @note Called by GCS_MAVLINK message routing when fence-related messages arrive
     */
    void handle_msg(class GCS_MAVLINK &link, const mavlink_message_t& msg);

    /**
     * @brief Check if vehicle's current position breaches any fence
     * 
     * @return true if vehicle is outside inclusion zones or inside exclusion zones, false otherwise
     * 
     * @details Checks vehicle position against all loaded fences:
     *          - Must be inside ALL inclusion polygons (if any exist)
     *          - Must be outside ALL exclusion polygons
     *          - Must be inside ALL inclusion circles (if any exist)
     *          - Must be outside ALL exclusion circles
     * 
     *          Breach conditions:
     *          - Outside any inclusion zone → breached
     *          - Inside any exclusion zone → breached
     *          - If only exclusions exist, vehicle is valid anywhere outside exclusions
     * 
     * @warning This is a safety-critical function - breach triggers failsafe actions
     * @note Uses current vehicle position from AHRS
     * @note Returns false (not breached) if fences not loaded or EKF origin invalid
     */
    bool breached() const WARN_IF_UNUSED;
    
    /**
     * @brief Check if specified location breaches any fence and calculate distance
     * 
     * @param[in]  loc                    Location to check (lat/lon/alt)
     * @param[out] distance_outside_fence Signed distance to nearest fence boundary in meters
     *                                    (positive = outside fence, negative = inside valid area)
     * 
     * @return true if location is outside inclusion zones or inside exclusion zones, false otherwise
     * 
     * @details Calculates breach status and proximity to fence boundaries using:
     *          - Point-in-polygon algorithm: Ray casting method for polygon containment
     *          - Circle distance: Euclidean distance from circle center minus radius
     *          - Signed distance: Negative when safely inside fence, positive when breached
     * 
     *          The returned distance is the minimum distance to the nearest fence boundary:
     *          - For inclusion zones: distance from inside valid area to fence edge
     *          - For exclusion zones: distance from outside exclusion zone to its edge
     *          - Positive values indicate breach (outside valid area)
     *          - Negative values indicate safe (inside valid area with margin)
     * 
     * @warning This is a safety-critical function - used for proximity warnings
     * @note Returns false (not breached) if fences not loaded or EKF origin invalid
     * @note Distance calculation is approximate for complex polygon shapes
     */
    bool breached(const Location& loc, float& distance_outside_fence) const WARN_IF_UNUSED;
    
    /**
     * @brief Check if specified location breaches any fence (without distance calculation)
     * 
     * @param[in] loc Location to check (lat/lon/alt)
     * 
     * @return true if location is outside inclusion zones or inside exclusion zones, false otherwise
     * 
     * @details Convenience wrapper for breached(loc, distance) that discards distance value.
     *          See full breached() documentation for breach detection algorithm details.
     * 
     * @warning This is a safety-critical function - breach triggers failsafe actions
     */
    bool breached(const Location& loc) const WARN_IF_UNUSED
    {
        float distance_outside_fence;
        return breached(loc, distance_outside_fence);
    }

    /**
     * @brief Check if any inclusion boundary fences are available
     * 
     * @return true if at least one inclusion polygon fence is loaded, false otherwise
     * 
     * @note Does not include inclusion circles - only polygon boundaries
     */
    bool inclusion_boundary_available() const WARN_IF_UNUSED {
        return _num_loaded_inclusion_boundaries != 0;
    }

    /**
     * @brief Check if fences have been successfully loaded from storage
     * 
     * @return true if fences loaded and available for use, false otherwise
     * 
     * @note Returns false if load never attempted, load failed, or fences were unloaded
     * @note Use this to verify fence system is operational before relying on breach detection
     */
    bool loaded() const WARN_IF_UNUSED {
        return _load_time_ms != 0;
    };

    /**
     * @brief Get maximum number of fence items that can be stored
     * 
     * @return Maximum item count based on available storage space
     * 
     * @details Calculates maximum storage capacity based on StorageManager allocation.
     *          Each item requires different storage:
     *          - Polygon vertex: 8 bytes (lat/lon as Vector2l)
     *          - Circle: 12 bytes (center lat/lon + radius float)
     *          - Type markers: 1 byte each
     * 
     * @note Used by MAVLink protocol to advertise upload capacity to ground station
     */
    uint16_t max_items() const;

    /**
     * @brief Validate and write fence items to persistent storage
     * 
     * @param[in] new_items Pointer to array of fence items to store
     * @param[in] count     Number of items in array
     * 
     * @return true if fences validated and written successfully, false if validation failed or write error
     * 
     * @details Performs atomic fence update:
     *          1. Validates fence configuration (valid types, counts, no gaps)
     *          2. Checks storage space availability
     *          3. Writes all fences to storage in sequential format
     *          4. Marks end-of-storage
     *          5. Invalidates index and loaded fences (forces reload)
     * 
     *          Validation checks:
     *          - All fence types valid
     *          - Polygon vertex counts reasonable (3-127 vertices)
     *          - Circle radii positive
     *          - Total size fits in storage
     *          - At least one valid operational area exists
     * 
     * @warning Old fence configuration is lost if write succeeds - no undo capability
     * @warning Fence breach detection unavailable until load_from_storage() completes
     * @warning Must have at least one inclusion zone OR have no inclusion zones at all
     * @note Thread-safe via internal semaphore locking
     * @note Automatically called by MAVLink fence upload protocol
     */
    bool write_fence(const AC_PolyFenceItem *new_items, uint16_t count)  WARN_IF_UNUSED;

    /*
     * Loaded Fence functionality
     *
     * methods and members to do with fences stored in memory.  The
     * locations are translated into offset-from-origin-in-metres
     */

    /**
     * @brief Load fences from persistent storage and transform to NE coordinates
     * 
     * @return true if fences loaded and transformed successfully, false if error
     * 
     * @details Performs complete fence loading sequence:
     *          1. Check storage format is valid (formatted flag present)
     *          2. Build index of fences in storage if not already indexed
     *          3. Allocate memory for transformed fence coordinates
     *          4. Get current EKF origin for coordinate transformation
     *          5. Read each fence from storage (lat/lon format)
     *          6. Transform lat/lon to NE offsets in cm from EKF origin
     *          7. Store transformed coordinates in _loaded_offsets_from_origin
     *          8. Mark fences as loaded with timestamp
     * 
     *          Coordinate transformation:
     *          - Input: Latitude/longitude in 1e-7 degrees (Vector2l)
     *          - Output: NE offsets in centimeters from EKF origin (Vector2f)
     *          - Uses EKF origin for coordinate reference frame
     * 
     *          Memory allocation:
     *          - _loaded_offsets_from_origin: All polygon vertices + circle centers
     *          - _loaded_inclusion_boundary: Array of inclusion polygon descriptors
     *          - _loaded_exclusion_boundary: Array of exclusion polygon descriptors
     *          - _loaded_circle_inclusion_boundary: Array of inclusion circles
     *          - _loaded_circle_exclusion_boundary: Array of exclusion circles
     * 
     * @warning Requires valid EKF origin - returns false if origin not initialized
     * @warning Allocates heap memory - may fail on memory-constrained systems
     * @warning Existing loaded fences are freed - any pointers to them become invalid
     * @note Called automatically by update() when fences not loaded
     * @note Thread-safe via _loaded_fence_sem
     */
    bool load_from_storage() WARN_IF_UNUSED;

    /**
     * @brief Get semaphore for thread-safe access to loaded fence data
     * 
     * @return Reference to HAL_Semaphore protecting fence coordinate arrays
     * 
     * @details Lock this semaphore before accessing:
     *          - _loaded_offsets_from_origin
     *          - _loaded_inclusion_boundary
     *          - _loaded_exclusion_boundary
     *          - _loaded_circle_inclusion_boundary
     *          - _loaded_circle_exclusion_boundary
     * 
     * @note Must be locked by external code accessing fence arrays
     * @note breached() methods lock internally - external locking not required
     */
    HAL_Semaphore &get_loaded_fence_semaphore(void) {
        return _loaded_fence_sem;
    }

    /**
     * @brief Periodic update to maintain fence validity
     * 
     * @details Called at 10Hz (typically from main scheduler) to:
     *          - Detect EKF origin changes
     *          - Trigger fence reload when origin changes
     *          - Ensure coordinate transformations remain valid
     *          - Attempt initial load if not yet loaded
     * 
     *          When EKF origin changes, all fence coordinates become invalid
     *          and must be retransformed from lat/lon to new NE offsets.
     * 
     * @note Must be called regularly for fence system to remain operational
     * @warning Fence breach detection invalid during reload period (brief)
     */
    void update();

    /**
     * @brief Retrieve the designated fence return point
     * 
     * @param[out] ret Return point in lat/lon (1e-7 degrees)
     * 
     * @return true if return point defined and retrieved, false if no return point stored
     * 
     * @details The return point is a designated safe location where the vehicle should
     *          navigate when a fence breach occurs. Stored in persistent storage as
     *          absolute lat/lon coordinates.
     * 
     *          Return point is optional - fence configuration can exist without one.
     *          If no return point defined, vehicle typically returns to home or rally point.
     * 
     * @note This reads from storage directly - does not require fences to be loaded
     * @note Return point type is AC_PolyFenceType::RETURN_POINT in storage
     */
    bool get_return_point(Vector2l &ret) WARN_IF_UNUSED;

    /**
     * @brief Get total count of all fence types
     * 
     * @return Sum of all polygon and circle fences (inclusion + exclusion)
     * 
     * @note Does not include return point in count
     */
    uint16_t total_fence_count() const {
        return (get_exclusion_polygon_count() +
                get_inclusion_polygon_count() +
                get_exclusion_circle_count() +
                get_inclusion_circle_count());
    }


#if AP_SDCARD_STORAGE_ENABLED
    /**
     * @brief Check if SD card fence storage initialization failed
     * 
     * @return true if SD card storage was configured but initialization failed
     * 
     * @details When FENCE_OPTIONS bit for SD card storage is set but SD card
     *          is unavailable or failed to initialize, this returns true.
     *          Used to alert user that fence storage is not functional.
     * 
     * @note Only available when AP_SDCARD_STORAGE_ENABLED is defined
     */
    bool failed_sdcard_storage(void) const {
        return _failed_sdcard_storage;
    }
#endif

private:
    // multi-thread access support
    HAL_Semaphore _loaded_fence_sem;  ///< Protects access to transformed fence coordinate arrays

    /*
     * Fence storage Index related functions
     */
    
    /**
     * @class FenceIndex
     * @brief Index entry describing a fence in persistent storage
     * 
     * @details The fence index provides fast lookup of fence locations in storage
     *          without scanning through all stored data. Index is built by scanning
     *          storage once and caching fence locations, types, and item counts.
     * 
     *          Index structure:
     *          - type: Fence type (polygon/circle, inclusion/exclusion)
     *          - count: Number of items in this fence (vertices for polygon, 1 for circle)
     *          - storage_offset: Byte offset in storage where this fence begins
     * 
     * @note Index is rebuilt when storage is modified or on first access after boot
     */
    class FenceIndex {
    public:
        AC_PolyFenceType type;     ///< Type of fence stored at this index entry
        uint16_t count;            ///< Number of items (vertices or circles) in this fence
        uint16_t storage_offset;   ///< Byte offset in storage where fence data begins
    };
    
    /**
     * @brief Count number of fences of specified type in the index
     * 
     * @param[in] type Fence type to count
     * 
     * @return Number of fences of specified type
     * 
     * @note Requires index to be built via check_indexed()
     */
    uint16_t index_fence_count(const AC_PolyFenceType type);

    /**
     * @brief Invalidate and free the fence index
     * 
     * @details Frees index memory and marks index as invalid, forcing rebuild
     *          on next access. Called when storage is modified or formatted.
     */
    void void_index() {
        delete[] _index;
        _index = nullptr;
        _index_attempted = false;
        _indexed = false;
    }

    /**
     * @brief Ensure fence index is built and valid
     * 
     * @return true if index exists or was successfully created, false on error
     * 
     * @details Lazy index creation - builds index on first call, returns cached
     *          index on subsequent calls. Index creation involves scanning entire
     *          storage space to catalog fence locations.
     */
    bool check_indexed() WARN_IF_UNUSED;

    /**
     * @brief Find first fence of specified type in index
     * 
     * @param[in] type Fence type to search for
     * 
     * @return Pointer to FenceIndex entry, or nullptr if not found
     */
    FenceIndex *find_first_fence(const AC_PolyFenceType type) const;

    /**
     * @brief Find index entry and loaded array offset for sequence number
     * 
     * @param[in]  seq   Item sequence number (0-based)
     * @param[out] entry Pointer to FenceIndex entry containing this item
     * @param[out] i     Offset in _loaded_offsets_from_origin where item is stored
     * 
     * @return true if sequence number found, false if out of range
     * 
     * @details Maps sequence number (linear item index) to fence and offset within
     *          the loaded coordinate array. Used for MAVLink item access.
     */
    bool find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const WARN_IF_UNUSED;
    
    /**
     * @brief Find storage offset for item by sequence number
     * 
     * @param[in]  seq                 Item sequence number (0-based)
     * @param[out] offset              Byte offset in storage where item is located
     * @param[out] type                Fence type of the fence containing this item
     * @param[out] vertex_count_offset Offset to vertex count field in storage
     * 
     * @return true if sequence number found, false if out of range
     * 
     * @details Uses index to efficiently map sequence number to storage location
     *          without scanning entire storage. Used for item retrieval.
     */
    bool find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const WARN_IF_UNUSED;

    /**
     * @brief Calculate total number of polygon vertices plus return point
     * 
     * @return Total count of polygon vertices across all polygons plus return point (if present)
     * 
     * @details Used to determine memory allocation size for _loaded_offsets_from_origin.
     *          Does not include circle centers (counted separately).
     */
    uint16_t sum_of_polygon_point_counts_and_returnpoint();

    /*
     * storage-related methods - dealing with fence_storage
     */

    /**
     * @brief Magic number indicating fence storage is formatted
     * 
     * @details Value 235 chosen to be out-of-band for legacy fence storage formats.
     *          Written as first byte in storage when format() is called.
     *          Checked by formatted() to verify storage initialization.
     * 
     * Storage format structure:
     * Byte 0: magic number (235)
     * Byte 1+: Sequence of fences, each with:
     *   - Type byte (AC_PolyFenceType)
     *   - Fence data (varies by type):
     *     * Polygon: vertex_count (uint8_t) + vertex_count * Vector2l (lat/lon)
     *     * Circle: center Vector2l (lat/lon) + radius (float)
     *     * Return point: Vector2l (lat/lon)
     * Last: END_OF_STORAGE type marker (99)
     * 
     * @warning Do not change this value - breaks compatibility with stored fences
     */
    static const uint8_t new_fence_storage_magic = 235;

    /**
     * @brief Validate fence item array for consistency and safety
     * 
     * @param[in] new_items Array of fence items to validate
     * @param[in] count     Number of items in array
     * 
     * @return true if fence configuration is valid and safe, false otherwise
     * 
     * @details Validation checks:
     *          - All fence types are valid enum values
     *          - Polygon vertex counts are reasonable (3-127 vertices)
     *          - Circle radii are positive
     *          - Total storage size fits in allocated space
     *          - END_OF_STORAGE marker present at end
     *          - At least one valid operational area exists:
     *            * Either inclusion zones defined (vehicle must stay inside)
     *            * Or only exclusion zones (vehicle valid anywhere outside exclusions)
     *          - No conflicting fence configurations
     * 
     * @warning This is a safety check - invalid fences rejected to prevent lockout
     */
    bool validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const WARN_IF_UNUSED;

    /**
     * @brief Byte offset of END_OF_STORAGE marker in storage
     * 
     * @details Cached offset to end-of-storage marker for efficient storage extension.
     *          Updated when storage is modified. Used by write operations to determine
     *          where to append new fences or where to truncate.
     */
    uint16_t _eos_offset;

    /**
     * @brief Check if storage space is formatted for use
     * 
     * @return true if storage contains magic number indicating it's formatted
     * 
     * @details Checks first byte of storage for new_fence_storage_magic value.
     *          Unformatted storage or legacy fence storage will not match.
     */
    bool formatted() const WARN_IF_UNUSED;
    
    /**
     * @brief Format storage space for fence storage
     * 
     * @return true if format successful, false on write error
     * 
     * @details Initializes storage by:
     *          1. Writing magic number at offset 0
     *          2. Writing END_OF_STORAGE marker at offset 1
     *          3. Clearing _eos_offset to 1
     *          4. Invalidating index and loaded fences
     * 
     * @warning Erases all existing fence data - cannot be undone
     */
    bool format() WARN_IF_UNUSED;


    /*
     * Loaded Fence functionality
     *
     * methods and members to do with fences stored in memory.  The
     * locations are translated into offset-from-origin-in-metres
     */

    /**
     * @brief Free memory allocated for loaded fence coordinates
     * 
     * @details Deallocates all loaded fence arrays:
     *          - _loaded_offsets_from_origin (NE coordinates)
     *          - _loaded_points_lla (lat/lon coordinates)
     *          - _loaded_inclusion_boundary (polygon descriptors)
     *          - _loaded_exclusion_boundary (polygon descriptors)
     *          - _loaded_circle_inclusion_boundary (circle data)
     *          - _loaded_circle_exclusion_boundary (circle data)
     * 
     * Resets all counts to zero and clears _load_time_ms.
     * Called when EKF origin changes or storage is modified.
     */
    void unload();

    /**
     * @brief Pointer to return point in NE coordinate array
     * 
     * @details Points into _loaded_offsets_from_origin array where return point
     *          NE offset is stored. nullptr if no return point defined.
     * 
     * Units: Centimeters from EKF origin in NE frame
     */
    Vector2f *_loaded_return_point;

    /**
     * @brief Pointer to return point in lat/lon array
     * 
     * @details Points into _loaded_points_lla array where return point
     *          lat/lon is stored. nullptr if no return point defined.
     * 
     * Units: 1e-7 degrees (Vector2l format)
     */
    Vector2l *_loaded_return_point_lla;

    /**
     * @class InclusionBoundary
     * @brief Descriptor for a loaded inclusion polygon fence
     * 
     * @details Provides access to polygon vertices in both coordinate formats:
     *          - points: NE offsets in cm from EKF origin (for breach detection)
     *          - points_lla: Original lat/lon in 1e-7 degrees (for display/storage)
     *          - count: Number of vertices in polygon
     * 
     * Pointers reference into larger arrays (_loaded_offsets_from_origin and
     * _loaded_points_lla) where all fence data is stored contiguously.
     */
    class InclusionBoundary {
    public:
        Vector2f *points;     ///< Pointer into _loaded_offsets_from_origin - NE offsets in cm
        Vector2l *points_lla; ///< Pointer into _loaded_points_lla - lat/lon in 1e-7 degrees
        uint8_t count;        ///< Number of vertices in this polygon
    };
    InclusionBoundary *_loaded_inclusion_boundary;  ///< Array of inclusion polygon descriptors

    uint8_t _num_loaded_inclusion_boundaries;       ///< Count of loaded inclusion polygons

    /**
     * @class ExclusionBoundary
     * @brief Descriptor for a loaded exclusion polygon fence
     * 
     * @details Provides access to polygon vertices in both coordinate formats:
     *          - points: NE offsets in cm from EKF origin (for breach detection)
     *          - points_lla: Original lat/lon in 1e-7 degrees (for display/storage)
     *          - count: Number of vertices in polygon
     * 
     * Pointers reference into larger arrays (_loaded_offsets_from_origin and
     * _loaded_points_lla) where all fence data is stored contiguously.
     */
    class ExclusionBoundary {
    public:
        Vector2f *points;     ///< Pointer into _loaded_offsets_from_origin - NE offsets in cm
        Vector2l *points_lla; ///< Pointer into _loaded_points_lla - lat/lon in 1e-7 degrees
        uint8_t count;        ///< Number of vertices in this polygon
    };
    ExclusionBoundary *_loaded_exclusion_boundary;  ///< Array of exclusion polygon descriptors

    uint8_t _num_loaded_exclusion_boundaries;       ///< Count of loaded exclusion polygons

    /**
     * @brief Array of all polygon vertices and return point in NE coordinates
     * 
     * @details Contiguous array holding all polygon vertices and return point transformed
     *          to NE offsets from EKF origin. Individual polygon and circle boundary
     *          structures point into this array.
     * 
     * Coordinate format:
     * - x component: North offset in centimeters from EKF origin
     * - y component: East offset in centimeters from EKF origin
     * - Frame: North-East-Down (NED) convention
     * 
     * @warning Coordinates become invalid if EKF origin changes
     * @note Protected by _loaded_fence_sem for thread-safe access
     */
    Vector2f *_loaded_offsets_from_origin;
    
    /**
     * @brief Array of all polygon vertices and return point in lat/lon
     * 
     * @details Parallel array to _loaded_offsets_from_origin containing original
     *          lat/lon coordinates. Used for display and coordinate validation.
     * 
     * Coordinate format: Vector2l with lat/lon in 1e-7 degrees
     */
    Vector2l *_loaded_points_lla;
    
    /**
     * @brief EKF origin used for coordinate transformation
     * 
     * @details Stores the EKF origin location at the time fences were loaded.
     *          Used to detect origin changes that require fence reload.
     *          All _loaded_offsets_from_origin coordinates are relative to this origin.
     */
    Location loaded_origin;

    /**
     * @class ExclusionCircle
     * @brief Descriptor for a loaded exclusion circle fence
     * 
     * @details Contains circle parameters in multiple coordinate formats:
     *          - pos_cm: Center as NE offset in cm from EKF origin (for breach detection)
     *          - point: Center as lat/lon in 1e-7 degrees (for display/storage)
     *          - radius: Circle radius in meters
     * 
     * Breach detection: Vehicle is breached if distance from center < radius
     */
    class ExclusionCircle {
    public:
        Vector2f pos_cm; ///< Circle center NE offset in cm from EKF origin
        Vector2l point;  ///< Circle center lat/lon in 1e-7 degrees
        float radius;    ///< Circle radius in meters
    };
    ExclusionCircle *_loaded_circle_exclusion_boundary;  ///< Array of loaded exclusion circles
    
    uint8_t _num_loaded_circle_exclusion_boundaries;     ///< Count of loaded exclusion circles

    /**
     * @class InclusionCircle
     * @brief Descriptor for a loaded inclusion circle fence
     * 
     * @details Contains circle parameters in multiple coordinate formats:
     *          - pos_cm: Center as NE offset in cm from EKF origin (for breach detection)
     *          - point: Center as lat/lon in 1e-7 degrees (for display/storage)
     *          - radius: Circle radius in meters
     * 
     * Breach detection: Vehicle is breached if distance from center > radius
     * @warning If inclusion circles exist, vehicle must be inside at least one
     */
    class InclusionCircle {
    public:
        Vector2f pos_cm;    ///< Circle center NE offset in cm from EKF origin
        Vector2l point;     ///< Circle center lat/lon in 1e-7 degrees
        float radius;       ///< Circle radius in meters
    };
    InclusionCircle *_loaded_circle_inclusion_boundary;  ///< Array of loaded inclusion circles

    uint8_t _num_loaded_circle_inclusion_boundaries;     ///< Count of loaded inclusion circles

    /**
     * @brief Flag indicating fence load has been attempted
     * 
     * @details Set to true after first load_from_storage() call regardless of success.
     *          Used to prevent repeated load attempts if storage is invalid or empty.
     *          Reset to false by unload() or when storage is modified.
     */
    bool _load_attempted;

    /**
     * @brief Timestamp of last successful fence load
     * 
     * @details System time from millis() when fences were successfully loaded and
     *          transformed. Zero if fences have never been loaded or were unloaded.
     *          Used by update_ms() methods to report fence configuration age.
     * 
     * @note Non-zero value indicates fences are currently loaded and valid
     */
    uint32_t _load_time_ms;

    /**
     * @brief Transform lat/lon point to NE offset from origin
     * 
     * @param[in]  origin  EKF origin location (reference point)
     * @param[in]  point   Point to transform (lat/lon in 1e-7 degrees)
     * @param[out] pos_cm  Transformed point as NE offset in cm from origin
     * 
     * @return true if transformation successful, false if origin invalid
     * 
     * @details Performs coordinate transformation:
     *          1. Calculate bearing and distance from origin to point
     *          2. Convert to North/East offset in meters
     *          3. Scale to centimeters
     *          4. Store in pos_cm as Vector2f(north_cm, east_cm)
     * 
     *          Coordinate frame: NE offsets in NED (North-East-Down) convention
     *          - pos_cm.x = North offset in centimeters (positive = north of origin)
     *          - pos_cm.y = East offset in centimeters (positive = east of origin)
     * 
     * @note This transformation is approximate for large distances (>1km) due to Earth curvature
     * @warning Requires valid origin with initialized lat/lon
     */
    bool scale_latlon_from_origin(const Location &origin,
                                  const Vector2l &point,
                                  Vector2f &pos_cm) const WARN_IF_UNUSED;
   
    /**
     * @brief Read polygon vertices from storage and transform to NE coordinates
     * 
     * @param[in]     origin              EKF origin for coordinate transformation
     * @param[in,out] read_offset         Byte offset in storage (updated as data is read)
     * @param[in]     vertex_count        Number of vertices to read
     * @param[in,out] next_storage_point  Pointer into NE coordinate array (updated after write)
     * @param[in,out] next_storage_point_lla Pointer into lat/lon array (updated after write)
     * 
     * @return true if all vertices read and transformed successfully, false on error
     * 
     * @details Reads vertex_count lat/lon points from storage sequentially:
     *          1. Read Vector2l (lat/lon in 1e-7 degrees) from storage
     *          2. Store in next_storage_point_lla array
     *          3. Transform to NE offset using scale_latlon_from_origin()
     *          4. Store in next_storage_point array
     *          5. Advance both pointers and read_offset
     * 
     * @note Pointers are advanced by vertex_count after call completes
     * @note read_offset is advanced by (vertex_count * sizeof(Vector2l)) bytes
     */
    bool read_polygon_from_storage(const Location &origin,
                                   uint16_t &read_offset,
                                   const uint8_t vertex_count,
                                   Vector2f *&next_storage_point,
                                   Vector2l *&next_storage_point_lla) WARN_IF_UNUSED;

#if AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT
    /*
     * FENCE_POINT protocol compatibility
     */
    
    /**
     * @brief Handle legacy FENCE_FETCH_POINT MAVLink message
     * 
     * @param[in] link Reference to GCS_MAVLINK for sending response
     * @param[in] msg  FENCE_FETCH_POINT message requesting fence point
     * 
     * @details Legacy protocol handler for old fence protocol. Retrieves fence point
     *          by index and sends FENCE_POINT response. Only supports single inclusion
     *          polygon and return point.
     * 
     * @note Only available when AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT enabled
     * @deprecated Use MISSION_ITEM_INT protocol instead
     */
    void handle_msg_fetch_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    
    /**
     * @brief Handle legacy FENCE_POINT MAVLink message
     * 
     * @param[in] link Reference to GCS_MAVLINK for sending acknowledgment
     * @param[in] msg  FENCE_POINT message containing fence point data
     * 
     * @details Legacy protocol handler for old fence upload protocol. Receives fence
     *          points one at a time and stores to create inclusion polygon. Updates
     *          FENCE_TOTAL parameter to track point count.
     * 
     * @note Only available when AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT enabled
     * @deprecated Use MISSION_ITEM_INT protocol instead
     */
    void handle_msg_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    
    /**
     * @brief Check if stored fences are compatible with legacy FENCE_POINT protocol
     * 
     * @return true if fence configuration can be represented in FENCE_POINT format
     * 
     * @details FENCE_POINT protocol limitations:
     *          - Supports only ONE inclusion polygon
     *          - Supports only ONE return point
     *          - No exclusion zones
     *          - No circular fences
     * 
     * Returns true if storage contains compatible configuration, false otherwise.
     * 
     * @note Only available when AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT enabled
     */
    bool contains_compatible_fence() const WARN_IF_UNUSED;

    /**
     * @brief Get or create inclusion fence for FENCE_POINT protocol
     * 
     * @return Pointer to FenceIndex for inclusion polygon, or nullptr on error
     * 
     * @details Finds existing inclusion polygon in storage, or creates new empty one
     *          if none exists. May format storage if uninitialized. Used by legacy
     *          FENCE_POINT upload to ensure target fence exists.
     * 
     * @note Only available when AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT enabled
     * @warning May format storage, erasing existing incompatible fences
     */
    FenceIndex *get_or_create_include_fence();
    
    /**
     * @brief Get or create return point for FENCE_POINT protocol
     * 
     * @return Pointer to FenceIndex for return point, or nullptr on error
     * 
     * @details Finds existing return point in storage, or creates new one if none
     *          exists. May format storage if uninitialized. Used by legacy FENCE_POINT
     *          upload to store designated return location.
     * 
     * @note Only available when AC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT enabled
     * @warning May format storage, erasing existing incompatible fences
     */
    FenceIndex *get_or_create_return_point();
#endif

    /**
     * @brief Write fence type byte to storage
     * 
     * @param[in,out] offset Storage byte offset (updated after write)
     * @param[in]     type   Fence type to write
     * 
     * @return true if write successful, false on storage error
     * 
     * @details Low-level primitive to write AC_PolyFenceType as uint8_t to storage.
     *          Advances offset by sizeof(uint8_t) on success.
     * 
     * @note Used during fence upload to write type markers
     */
    bool write_type_to_storage(uint16_t &offset, AC_PolyFenceType type) WARN_IF_UNUSED;
    
    /**
     * @brief Write lat/lon point to storage
     * 
     * @param[in,out] offset Storage byte offset (updated after write)
     * @param[in]     latlon Point to write (lat/lon in 1e-7 degrees)
     * 
     * @return true if write successful, false on storage error
     * 
     * @details Low-level primitive to write Vector2l (latitude/longitude) to storage.
     *          Storage format: int32_t lat, int32_t lon (8 bytes total).
     *          Advances offset by sizeof(Vector2l) on success.
     * 
     * @note Used during fence upload to write fence vertices
     */
    bool write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon) WARN_IF_UNUSED;
    
    /**
     * @brief Read lat/lon point from storage
     * 
     * @param[in,out] read_offset Storage byte offset (updated after read)
     * @param[out]    latlon      Retrieved point (lat/lon in 1e-7 degrees)
     * 
     * @return true if read successful, false on storage error
     * 
     * @details Low-level primitive to read Vector2l (latitude/longitude) from storage.
     *          Storage format: int32_t lat, int32_t lon (8 bytes total).
     *          Advances read_offset by sizeof(Vector2l) on success.
     * 
     * @note Used during fence loading to read fence vertices
     */
    bool read_latlon_from_storage(uint16_t &read_offset, Vector2l &latlon) const WARN_IF_UNUSED;

    /**
     * @brief Write end-of-storage marker to storage
     * 
     * @param[in,out] offset Storage byte offset (updated after write)
     * 
     * @return true if write successful, false on storage error
     * 
     * @details Writes END_OF_STORAGE type marker to indicate no more fences follow.
     *          Updates _eos_offset member to track end-of-storage location for future
     *          fence additions.
     * 
     * @note Must be called after writing all fences to terminate storage properly
     */
    bool write_eos_to_storage(uint16_t &offset);

    /**
     * @brief Reference to FENCE_TOTAL parameter
     * 
     * @details Used solely for compatibility with legacy FENCE_POINT protocol.
     *          Tracks number of points in inclusion polygon for old GCS compatibility.
     * 
     * @note Modern fence protocol uses MISSION_ITEM_INT and does not require this
     */
    AP_Int8 &_total;
    
    /**
     * @brief Reference to FENCE_OPTIONS parameter
     * 
     * @details Fence configuration options bitmask. Used to check fence behavior settings.
     */
    const AP_Int16 &_options;
    
    /**
     * @brief Previous value of FENCE_TOTAL parameter
     * 
     * @details Cached to detect changes for fence reload triggers.
     */
    uint8_t _old_total;


    /**
     * @brief Scan fence storage and call callback for each fence found
     * 
     * @param[in] scan_fn Callback function invoked for each fence
     * 
     * @return true if scan completed successfully, false if storage corrupt
     * 
     * @details Traverses fence storage sequentially from beginning to END_OF_STORAGE
     *          marker, calling scan_fn(type, offset) for each fence encountered.
     *          
     *          Storage layout per fence:
     *          - Type byte (AC_PolyFenceType)
     *          - Fence-specific data (vertices, radius, etc.)
     *          - Next fence immediately follows
     * 
     *          Validates storage format and detects corruption. Returns false if:
     *          - Storage not formatted (no magic number)
     *          - Invalid type encountered
     *          - Offset exceeds storage bounds
     *          - No END_OF_STORAGE marker found
     * 
     * @note This is the foundation for indexing and counting operations
     * @warning Caller must handle corrupt storage gracefully (may require format)
     */
    FUNCTOR_TYPEDEF(scan_fn_t, void, const AC_PolyFenceType, uint16_t);
    bool scan_eeprom(scan_fn_t scan_fn) WARN_IF_UNUSED;
    
    /**
     * @brief Callback to count fences during storage scan
     * 
     * @param[in] type        Fence type encountered
     * @param[in] read_offset Storage offset where fence starts
     * 
     * @details Designed to be passed to scan_eeprom(). Increments fence and item
     *          counters as storage is traversed. Results stored in:
     *          - _eeprom_fence_count: Total number of fences
     *          - _eeprom_item_count: Total number of fence items (vertices, circles, etc.)
     * 
     * @note Must be called before index_eeprom() to allocate correct index size
     */
    void scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset);
    
    /**
     * @brief Total number of fences in storage
     * 
     * @details Populated by scan_eeprom_count_fences(). Includes all fence types:
     *          polygons (inclusion/exclusion), circles (inclusion/exclusion), and
     *          return point.
     */
    uint16_t _eeprom_fence_count;
    
    /**
     * @brief Total number of fence items in storage
     * 
     * @details Populated by scan_eeprom_count_fences(). Counts individual items:
     *          - Each polygon vertex is one item
     *          - Each circle is one item
     *          - Return point is one item
     * 
     * Used for MAVLink protocol count reporting and storage validation.
     */
    uint16_t _eeprom_item_count;

    /**
     * @brief Callback to build fence index during storage scan
     * 
     * @param[in] type        Fence type encountered
     * @param[in] read_offset Storage offset where fence starts
     * 
     * @details Designed to be passed to scan_eeprom() after _index has been allocated.
     *          Populates _index array with FenceIndex entries containing:
     *          - Fence type
     *          - Item count (vertices for polygon, 1 for circle)
     *          - Storage offset for direct access
     * 
     * @note Requires scan_eeprom_count_fences() first to determine allocation size
     * @note _index must be pre-allocated to size _eeprom_fence_count
     */
    void scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset);
    
    /**
     * @brief Array of FenceIndex entries describing fences in storage
     * 
     * @details Dynamically allocated array with _num_fences entries. Each entry
     *          contains type, count, and storage offset for one fence. Enables fast
     *          random access to fences without scanning entire storage.
     * 
     * Nullptr if indexing not yet performed or failed.
     */
    FenceIndex *_index;
    
    /**
     * @brief True if storage indexing succeeded
     * 
     * @details Set true after successful index_eeprom() call. If false, index is
     *          invalid and storage access requires full scan.
     */
    bool _indexed;
    
    /**
     * @brief True if indexing has been attempted
     * 
     * @details Prevents repeated indexing attempts if first attempt failed. Reset
     *          by void_index() when storage is modified.
     */
    bool _index_attempted;
    
    /**
     * @brief Number of fences in _index array
     * 
     * @details Should equal _eeprom_fence_count after successful indexing. Used for
     *          bounds checking when accessing _index.
     */
    uint16_t _num_fences;

    /**
     * @brief Refresh count of fences in permanent storage
     * 
     * @return true if count successful, false if storage corrupt or unformatted
     * 
     * @details Scans storage using scan_eeprom() with scan_eeprom_count_fences callback
     *          to update _eeprom_fence_count and _eeprom_item_count. Called during
     *          initialization and after fence modifications.
     * 
     * @note Prerequisite for index_eeprom()
     */
    bool count_eeprom_fences() WARN_IF_UNUSED;
    
    /**
     * @brief Build or rebuild fence index
     * 
     * @return true if indexing successful, false on allocation failure or corrupt storage
     * 
     * @details Allocates _index array and populates it by scanning storage:
     *          1. Calls count_eeprom_fences() to determine fence count
     *          2. Allocates _index = new FenceIndex[_eeprom_fence_count]
     *          3. Scans storage with scan_eeprom_index_fences to populate index
     *          4. Sets _indexed = true on success
     * 
     * Invalidates and recreates existing index. Sets _index_attempted flag to prevent
     * repeated failures.
     * 
     * @note Required before using find_first_fence() or find_storage_offset_for_seq()
     */
    bool index_eeprom() WARN_IF_UNUSED;

    /**
     * @brief Calculate storage space required for fence items
     * 
     * @param[in] new_items Fence items to measure
     * @param[in] count     Number of items in array
     * 
     * @return Number of bytes required in storage
     * 
     * @details Calculates total storage requirement including:
     *          - Type bytes for each fence
     *          - Lat/lon coordinates for polygon vertices
     *          - Radius values for circles
     *          - Vertex count bytes
     *          - END_OF_STORAGE marker
     * 
     * Used to validate sufficient storage space before write_fence().
     */
    uint16_t fence_storage_space_required(const AC_PolyFenceItem *new_items, uint16_t count);

#if AP_SDCARD_STORAGE_ENABLED
    /**
     * @brief True if SD card storage initialization failed
     * 
     * @details Set during init() if SD card storage was expected but could not be
     *          initialized. Used to report configuration errors to user.
     * 
     * @note Only available when AP_SDCARD_STORAGE_ENABLED is defined
     */
    bool _failed_sdcard_storage;
#endif
};

#endif // AP_FENCE_ENABLED
