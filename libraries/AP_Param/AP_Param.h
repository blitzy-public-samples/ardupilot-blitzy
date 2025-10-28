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

/**
 * @file AP_Param.h
 * @brief ArduPilot parameter system for persistent configuration storage and management
 * 
 * @details The AP_Param subsystem provides:
 * - Persistent storage of configuration parameters across reboots (EEPROM/Flash/FRAM)
 * - Runtime parameter registration and lookup by name or index
 * - MAVLink parameter protocol implementation (PARAM_REQUEST_LIST, PARAM_SET, etc.)
 * - Default value management with optional external defaults files
 * - Parameter conversion/migration between firmware versions
 * - Nested parameter groups for structured organization
 * - Metadata system (@Param tags) for auto-documentation and ground station integration
 * 
 * Architecture:
 * - Storage backend: StorageManager provides abstraction over EEPROM/Flash/FRAM
 * - Registration: Vehicle code declares parameters via var_info[] arrays with GroupInfo entries
 * - Lookup: Parameters found by name, index, or memory pointer
 * - Persistence: Asynchronous save queue with I/O thread processing
 * - Conversion: Legacy parameter migration preserves user settings across firmware updates
 * 
 * Storage format:
 * - EEPROM header (magic bytes + revision) at offset 0
 * - Parameter headers (9-bit key + 18-bit group_element + 5-bit type) followed by value payload
 * - Sentinel record (0x1FF key) marks end of used storage
 * - Compact binary format minimizes storage usage
 * 
 * Parameter naming:
 * - Top-level: Direct names (e.g., "SYSID_THISMAV")
 * - Grouped: Dot notation (e.g., "COMPASS_OFS_X", "INS_GYRO_FILTER")
 * - Nested groups: Multiple levels (e.g., "COMPASS_OFS.x", "COMPASS_OFS.y")
 * - Maximum name length: AP_MAX_NAME_SIZE (16 characters)
 * 
 * Thread safety:
 * - Save operations use queue + I/O thread to avoid blocking flight control
 * - Semaphore protects scalar count cache during concurrent enumeration
 * - GCS parameter operations synchronized via MAVLink message handlers
 * 
 * @note Core dependency for all ArduPilot configuration - removal breaks system
 * @note Parameter changes may require reboot to take effect (vehicle-dependent)
 * @warning Corrupting parameter storage causes loss of all configuration
 * @warning Exceeding storage capacity (sentinal_offset) prevents parameter saves
 * 
 * @see StorageManager for storage backend abstraction
 * @see GCS_MAVLink for parameter protocol implementation
 * @see AP_BoardConfig for runtime parameter permissions
 */
#pragma once

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <cmath>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <StorageManager/StorageManager.h>
#include <AP_Scripting/AP_Scripting_config.h>

#include "AP_Param_config.h"

#include "float.h"

/**
 * @def AP_MAX_NAME_SIZE
 * @brief Maximum length of parameter name including null terminator
 * 
 * @details Parameter names limited to 15 characters + null terminator (16 bytes total).
 *          Names longer than 15 characters truncated with warning during registration.
 *          Includes dots for nested groups (e.g., "COMPASS_OFS_X" = 14 chars).
 * 
 * @note Increasing this value breaks storage format compatibility
 * @warning MAVLink parameter protocol also has 16-byte name limit
 */
#define AP_MAX_NAME_SIZE 16

/**
 * @def AP_PARAM_KEY_DUMP
 * @brief Enable debug code for dumping parameter keys at boot
 * 
 * @details When set to 1, logs all parameter keys and group_elements during
 *          parameter table registration. Used for debugging storage corruption.
 *          Defaults to 0 (disabled) to save flash space.
 * 
 * @note Enable temporarily for debugging, not for production builds
 */
#ifndef AP_PARAM_KEY_DUMP
#define AP_PARAM_KEY_DUMP 0
#endif

/**
 * @def AP_PARAM_DEFAULTS_ENABLED
 * @brief Enable external defaults file and embedded defaults support
 * 
 * @details Controls whether parameter system supports loading defaults from
 *          filesystem or embedded firmware regions. Defaults to HAL_GCS_ENABLED
 *          (GCS communication includes defaults support).
 * 
 * @note Set to 0 on minimal builds to save flash space
 * @see load_defaults_file(), reload_defaults_file()
 */
#if defined(HAL_GCS_ENABLED)
    #define AP_PARAM_DEFAULTS_ENABLED HAL_GCS_ENABLED
#else
    #define AP_PARAM_DEFAULTS_ENABLED 1
#endif

/**
 * @def AP_PARAM_MAX_EMBEDDED_PARAM
 * @brief Maximum size in bytes of embedded parameter defaults blob
 * 
 * @details When FORCE_APJ_DEFAULT_PARAMETERS enabled, reserves space in firmware
 *          for parameter defaults that can be modified by apj_tool.py without
 *          recompiling. Size varies by target flash capacity:
 *          - Small targets (≤1MB): 1024 bytes
 *          - Large targets (>1MB): 8192 bytes
 *          - FORCE_APJ_DEFAULT_PARAMETERS disabled: 0 bytes
 * 
 * @note Embedded defaults increase firmware size
 * @note Modifiable post-build via apj_tool.py --set-defaults
 * @see FORCE_APJ_DEFAULT_PARAMETERS in AP_Param_config.h
 */
#ifndef AP_PARAM_MAX_EMBEDDED_PARAM
  #if FORCE_APJ_DEFAULT_PARAMETERS
    #if HAL_PROGRAM_SIZE_LIMIT_KB <= 1024
      #define AP_PARAM_MAX_EMBEDDED_PARAM 1024
    #else
      #define AP_PARAM_MAX_EMBEDDED_PARAM 8192
    #endif
  #else
    #define AP_PARAM_MAX_EMBEDDED_PARAM 0
  #endif
#endif

/**
 * @def AP_PARAM_DYNAMIC_ENABLED
 * @brief Enable runtime creation of parameter tables (scripting support)
 * 
 * @details When enabled, allows dynamic parameter table registration via
 *          add_table() and add_group() APIs. Required for Lua scripting
 *          parameter support. Defaults to AP_SCRIPTING_ENABLED.
 * 
 * @note Increases flash and RAM usage slightly
 * @see add_table(), add_group()
 */
#ifndef AP_PARAM_DYNAMIC_ENABLED
#define AP_PARAM_DYNAMIC_ENABLED AP_SCRIPTING_ENABLED
#endif

/**
 * @def AP_PARAM_MAX_DYNAMIC
 * @brief Maximum number of dynamically created parameter tables
 * 
 * @details Limits runtime parameter table allocations from scripting.
 *          Each script can register parameter table consuming one slot.
 *          Defaults to 10 tables maximum.
 * 
 * @note Each dynamic table consumes RAM for registration data
 */
#ifndef AP_PARAM_MAX_DYNAMIC
#define AP_PARAM_MAX_DYNAMIC 10
#endif

/**
 * @def AP_PARAM_DYNAMIC_KEY_BASE
 * @brief Starting key value for dynamically allocated parameter tables
 * 
 * @details Dynamic tables assigned keys starting from 300 to avoid conflicts
 *          with static vehicle parameter keys (typically 0-255).
 *          Keys 300-309 reserved for dynamic tables (up to AP_PARAM_MAX_DYNAMIC).
 * 
 * @note Must not conflict with vehicle k_param_* enum values
 * @warning Changing this breaks storage compatibility for scripting parameters
 */
#define AP_PARAM_DYNAMIC_KEY_BASE 300

/**
 * @name Parameter Behavior Flags
 * @brief Flags controlling parameter registration, visibility, and storage behavior
 * @{
 */

/**
 * @def AP_PARAM_FLAG_NESTED_OFFSET
 * @brief Nested group uses offset calculation (not inheritance)
 * 
 * @details Indicates parameter group is nested via object composition rather than
 *          class inheritance. Memory address calculated using offset from parent.
 *          Set automatically by AP_SUBGROUPINFO macro.
 * 
 * @note Internal flag - set by macros, not directly by user code
 * @see AP_SUBGROUPINFO, AP_SUBGROUPEXTENSION
 */
#define AP_PARAM_FLAG_NESTED_OFFSET (1<<0)

/**
 * @def AP_PARAM_FLAG_POINTER
 * @brief Parameter object accessed via pointer dereference
 * 
 * @details Indicates member is pointer to object rather than direct member.
 *          Parameter system dereferences pointer before accessing parameters.
 *          Used for dynamically allocated objects.
 *          Set automatically by AP_SUBGROUPPTR and AP_GOBJECTPTR macros.
 * 
 * @note Pointer must be valid (non-null) when parameters accessed
 * @warning Null pointer causes fault during parameter access
 * @see AP_SUBGROUPPTR, AP_GOBJECTPTR
 */
#define AP_PARAM_FLAG_POINTER       (1<<1)

/**
 * @def AP_PARAM_FLAG_ENABLE
 * @brief Enable parameter controlling subtree visibility
 * 
 * @details Marks parameter as enable control for parameter subtree.
 *          When enable parameter is zero, all parameters in same group
 *          are hidden from GCS parameter list.
 *          Used for optional features (e.g., COMPASS_ENABLE hides compass parameters).
 * 
 * Example:
 * ```cpp
 * AP_GROUPINFO_FLAGS("ENABLE", 0, Compass, _enable, 1, AP_PARAM_FLAG_ENABLE),
 * // When COMPASS_ENABLE = 0, all other COMPASS_* parameters hidden
 * ```
 * 
 * @note Hiding parameters from GCS does not prevent firmware access
 * @note Hidden parameters still consume storage space
 */
#define AP_PARAM_FLAG_ENABLE        (1<<2)

/**
 * @def AP_PARAM_FLAG_NO_SHIFT
 * @brief Disable automatic index shift for nested groups
 * 
 * @details Normally nested groups shift index 0 to 63 to avoid parent conflicts.
 *          This flag disables shift when guaranteed no conflict exists.
 * 
 * @note Advanced flag - rarely used, requires careful analysis
 * @warning Incorrect use causes parameter index collisions
 */
#define AP_PARAM_FLAG_NO_SHIFT      (1<<3)

/**
 * @def AP_PARAM_FLAG_INFO_POINTER
 * @brief var_info table accessed via pointer indirection
 * 
 * @details Indicates GroupInfo.group_info_ptr is pointer to var_info table pointer
 *          rather than direct pointer to table. Enables dynamic table construction.
 *          Set automatically by AP_SUBGROUPVARPTR and AP_GOBJECTVARPTR macros.
 * 
 * @note Used for scripting and dynamically loaded parameter tables
 * @see AP_SUBGROUPVARPTR, AP_GOBJECTVARPTR
 */
#define AP_PARAM_FLAG_INFO_POINTER  (1<<4)

/**
 * @def AP_PARAM_FLAG_INTERNAL_USE_ONLY
 * @brief Parameter visible to GCS but firmware-controlled only
 * 
 * @details Parameter appears in GCS parameter list but cannot be modified via MAVLink.
 *          Firmware updates value based on internal state.
 *          Used for status parameters and computed values.
 * 
 * Example:
 * ```cpp
 * AP_GROUPINFO_FLAGS("STATUS", 5, MyClass, _status, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),
 * // GCS can read STATUS but MAVLink PARAM_SET rejected
 * ```
 * 
 * @note Different from AP_PARAM_FLAG_HIDDEN - parameter is visible
 * @note Prevents accidental user modification of computed values
 */
#define AP_PARAM_FLAG_INTERNAL_USE_ONLY (1<<5)

/**
 * @def AP_PARAM_FLAG_HIDDEN
 * @brief Hide parameter from GCS parameter download
 * 
 * @details Parameter excluded from MAVLink PARAM_REQUEST_LIST response.
 *          Used for debug parameters, deprecated parameters, or internal-only settings.
 *          Firmware can still access and save parameter normally.
 * 
 * @note Hidden parameters still consume storage and are in firmware
 * @note Can be accessed by name if GCS knows parameter exists
 */
#define AP_PARAM_FLAG_HIDDEN (1<<6)

/**
 * @def AP_PARAM_FLAG_DEFAULT_POINTER
 * @brief Default value computed at runtime from object member
 * 
 * @details Default value read from another member variable instead of compile-time
 *          constant. GroupInfo.def_value_offset holds offset to default value member.
 *          Set automatically by AP_GROUPINFO_FLAGS_DEFAULT_POINTER macro.
 * 
 * @note Enables board-specific or runtime-computed defaults
 * @see AP_GROUPINFO_FLAGS_DEFAULT_POINTER
 */
#define AP_PARAM_FLAG_DEFAULT_POINTER (1<<7)

/** @} */ // end of Parameter Behavior Flags group

// keep all flags before the FRAME tags

/**
 * @name Vehicle and Frame Type Flags
 * @brief Flags for hiding parameters based on vehicle/frame type
 * @{
 */

/**
 * @def AP_PARAM_FRAME_TYPE_SHIFT
 * @brief Bit position for frame type flags in GroupInfo.flags field
 * 
 * @details Frame type flags occupy bits 8+ of flags field, allowing combination
 *          with lower 8 bits used for behavior flags (POINTER, ENABLE, etc.).
 *          Frame flags shifted left by this amount before OR-ing into flags.
 * 
 * @note Set automatically by AP_GROUPINFO_FRAME and AP_GROUPINFO_FLAGS_FRAME macros
 * @see AP_GROUPINFO_FRAME, AP_GROUPINFO_FLAGS_FRAME
 */
#define AP_PARAM_FRAME_TYPE_SHIFT   8

/**
 * @def AP_PARAM_FRAME_COPTER
 * @brief Parameter visible only on multicopter vehicles
 * 
 * @details Parameter shown when vehicle reports frame type includes copter.
 *          Use AP_GROUPINFO_FRAME macro with this flag to create copter-only parameters.
 * 
 * Example:
 * ```cpp
 * AP_GROUPINFO_FRAME("HOVER_THR", 5, Copter, _hover_thr, 0.5f, AP_PARAM_FRAME_COPTER)
 * // Parameter visible only on copter, hidden on plane/rover/sub
 * ```
 * 
 * @note Frame type set at boot via AP_Param::set_frame_type_flags()
 * @see AP_Param::set_frame_type_flags()
 */
#define AP_PARAM_FRAME_COPTER       (1<<0)

/**
 * @def AP_PARAM_FRAME_ROVER
 * @brief Parameter visible only on ground vehicle (rover) vehicles
 * 
 * @details Parameter shown when vehicle reports frame type includes rover.
 * 
 * @see AP_PARAM_FRAME_COPTER for usage example
 */
#define AP_PARAM_FRAME_ROVER        (1<<1)

/**
 * @def AP_PARAM_FRAME_PLANE
 * @brief Parameter visible only on fixed-wing (plane) vehicles
 * 
 * @details Parameter shown when vehicle reports frame type includes plane.
 * 
 * @see AP_PARAM_FRAME_COPTER for usage example
 */
#define AP_PARAM_FRAME_PLANE        (1<<2)

/**
 * @def AP_PARAM_FRAME_SUB
 * @brief Parameter visible only on underwater (sub) vehicles
 * 
 * @details Parameter shown when vehicle reports frame type includes submarine.
 * 
 * @see AP_PARAM_FRAME_COPTER for usage example
 */
#define AP_PARAM_FRAME_SUB          (1<<3)

/**
 * @def AP_PARAM_FRAME_TRICOPTER
 * @brief Parameter visible only on tricopter frame types
 * 
 * @details Parameter shown when vehicle reports frame type includes tricopter.
 *          More specific than AP_PARAM_FRAME_COPTER for tri-specific parameters.
 * 
 * @see AP_PARAM_FRAME_COPTER for usage example
 */
#define AP_PARAM_FRAME_TRICOPTER    (1<<4)

/**
 * @def AP_PARAM_FRAME_HELI
 * @brief Parameter visible only on helicopter frame types
 * 
 * @details Parameter shown when vehicle reports frame type includes helicopter.
 *          Used for traditional helicopter-specific parameters (swash plates, tail rotor, etc.).
 * 
 * @see AP_PARAM_FRAME_COPTER for usage example
 */
#define AP_PARAM_FRAME_HELI         (1<<5)

/**
 * @def AP_PARAM_FRAME_BLIMP
 * @brief Parameter visible only on blimp (airship) vehicles
 * 
 * @details Parameter shown when vehicle reports frame type includes blimp.
 *          Used for lighter-than-air vehicle specific parameters.
 * 
 * @see AP_PARAM_FRAME_COPTER for usage example
 */
#define AP_PARAM_FRAME_BLIMP        (1<<6)

/** @} */ // end of Vehicle and Frame Type Flags group

// a variant of offsetof() to work around C++ restrictions.
// this can only be used when the offset of a variable in a object
// is constant and known at compile time
#define AP_VAROFFSET(type, element) (((ptrdiff_t)(&((const type *)1)->element))-1)

/**
 * @def AP_CLASSTYPE(clazz, element)
 * @brief Extract parameter type from class member at compile time
 * 
 * @param clazz Containing class name
 * @param element Member variable name (must be AP_ParamT-derived type)
 * 
 * @return uint8_t Parameter type enum value (AP_PARAM_INT8, AP_PARAM_FLOAT, etc.)
 * 
 * @details Uses pointer arithmetic trick to access vtype member without object instance.
 *          Evaluates at compile time - no runtime overhead.
 * 
 * @note Internal macro - use AP_GROUPINFO family instead
 */
#define AP_CLASSTYPE(clazz, element) ((uint8_t)(((const clazz *) 1)->element.vtype))

/**
 * @def AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags)
 * @brief Define scalar parameter in var_info array with custom flags
 * 
 * @param name Parameter name string (without class prefix, max 16 chars)
 * @param idx Parameter index 0-63 within this group (must be unique and stable)
 * @param clazz Containing class name
 * @param element Member variable name in class (AP_Int8/16/32/Float type)
 * @param def Default value (float literal)
 * @param flags Behavior flags (AP_PARAM_FLAG_* constants, frame type masks)
 * 
 * @details Creates GroupInfo entry with specified flags for:
 *          - AP_PARAM_FLAG_ENABLE: Enable/disable parameter subtrees
 *          - AP_PARAM_FLAG_HIDDEN: Hide from GCS parameter list
 *          - AP_PARAM_FLAG_INTERNAL_USE_ONLY: GCS visible but firmware-controlled
 *          - Frame type masks (bits 8+): Vehicle-specific visibility
 * 
 * Example:
 * ```cpp
 * AP_GROUPINFO_FLAGS("PARAM", 0, MyClass, _param, 1.0f, AP_PARAM_FLAG_ENABLE)
 * ```
 * 
 * @note idx must remain stable across firmware versions - changing breaks storage
 * @warning Reusing idx for different parameter loses user configuration
 * @see AP_GROUPINFO for basic usage without flags
 */
#define AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags) { name, AP_VAROFFSET(clazz, element), {def_value : def}, flags, idx, AP_CLASSTYPE(clazz, element)}

/**
 * @def AP_GROUPINFO_FRAME(name, idx, clazz, element, def, frame_flags)
 * @brief Define vehicle-specific parameter visible only on specified frame types
 * 
 * @param name Parameter name string
 * @param idx Parameter index 0-63 within group
 * @param clazz Containing class name
 * @param element Member variable name
 * @param def Default value
 * @param frame_flags Frame type bitmask (AP_PARAM_FRAME_COPTER, _PLANE, _ROVER, etc.)
 * 
 * @details Hides parameter from GCS unless vehicle type matches frame_flags.
 *          Useful for frame-specific tuning parameters.
 * 
 * Example:
 * ```cpp
 * AP_GROUPINFO_FRAME("HOVER_THR", 5, Copter, _hover_thr, 0.5f, AP_PARAM_FRAME_COPTER)
 * ```
 * 
 * @note Parameter storage persists across vehicle types (not deleted on type change)
 */
#define AP_GROUPINFO_FRAME(name, idx, clazz, element, def, frame_flags) AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, (frame_flags)<<AP_PARAM_FRAME_TYPE_SHIFT )

/**
 * @def AP_GROUPINFO_FLAGS_FRAME(name, idx, clazz, element, def, flags, frame_flags)
 * @brief Define parameter with both custom flags and frame type restrictions
 * 
 * @param name Parameter name string
 * @param idx Parameter index 0-63 within group
 * @param clazz Containing class name
 * @param element Member variable name
 * @param def Default value
 * @param flags Behavior flags (AP_PARAM_FLAG_* constants)
 * @param frame_flags Frame type bitmask
 * 
 * @details Combines AP_GROUPINFO_FLAGS and AP_GROUPINFO_FRAME functionality.
 * 
 * Example:
 * ```cpp
 * AP_GROUPINFO_FLAGS_FRAME("ENABLE", 0, Copter, _enable, 1, 
 *                          AP_PARAM_FLAG_ENABLE, AP_PARAM_FRAME_COPTER)
 * ```
 */
#define AP_GROUPINFO_FLAGS_FRAME(name, idx, clazz, element, def, flags, frame_flags) AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, flags|((frame_flags)<<AP_PARAM_FRAME_TYPE_SHIFT) )

/**
 * @def AP_GROUPINFO_FLAGS_DEFAULT_POINTER(name, idx, clazz, element, def)
 * @brief Define parameter with default value computed from another member offset
 * 
 * @param name Parameter name string
 * @param idx Parameter index 0-63 within group
 * @param clazz Containing class name
 * @param element Member variable name
 * @param def Member variable containing default value (not a literal)
 * 
 * @details Default value loaded from def member at runtime instead of compile-time
 *          constant. Useful when defaults computed dynamically (e.g., from hardware ID).
 * 
 * @note def must be member of same class, not a literal value
 * @note Sets AP_PARAM_FLAG_DEFAULT_POINTER automatically
 */
#define AP_GROUPINFO_FLAGS_DEFAULT_POINTER(name, idx, clazz, element, def) {  name, AP_VAROFFSET(clazz, element), {def_value_offset : AP_VAROFFSET(clazz, element) - AP_VAROFFSET(clazz, def)}, AP_PARAM_FLAG_DEFAULT_POINTER, idx, AP_CLASSTYPE(clazz, element) }

/**
 * @def AP_GROUPINFO(name, idx, clazz, element, def)
 * @brief Define scalar parameter in var_info array (most common usage)
 * 
 * @param name Parameter name string (without class prefix, max 16 chars)
 * @param idx Parameter index 0-63 within this group (must be unique and stable)
 * @param clazz Containing class name
 * @param element Member variable name in class (AP_Int8/16/32/Float type)
 * @param def Default value (float literal)
 * 
 * @details Standard parameter declaration for scalar values without special flags.
 *          Creates GroupInfo entry with zero flags.
 * 
 * Example:
 * ```cpp
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     AP_GROUPINFO("SPEED", 0, MyClass, _speed, 1.5f),
 *     AP_GROUPINFO("ACCEL", 1, MyClass, _accel, 2.0f),
 *     AP_GROUPEND
 * };
 * ```
 * 
 * @note idx must be unique within group and remain stable across firmware versions
 * @note Changing idx breaks parameter storage compatibility
 * @warning Reusing idx for different parameter loses user configuration
 */
#define AP_GROUPINFO(name, idx, clazz, element, def) AP_GROUPINFO_FLAGS(name, idx, clazz, element, def, 0)

/**
 * @def AP_NESTEDGROUPINFO(clazz, idx)
 * @brief Inherit parameters from parent class into derived class var_info
 * 
 * @param clazz Parent class name (must have static var_info[] array)
 * @param idx Parameter index within this group (typically 0 for inheritance)
 * 
 * @details Incorporates parent class parameters into derived class without name prefix.
 *          Parameters appear at same level as derived class parameters.
 *          Used for class inheritance where derived class extends parent parameters.
 * 
 * Example:
 * ```cpp
 * const AP_Param::GroupInfo DerivedClass::var_info[] = {
 *     AP_NESTEDGROUPINFO(BaseClass, 0),  // Inherit base parameters
 *     AP_GROUPINFO("DERIVED_PARAM", 1, DerivedClass, _param, 1.0f),
 *     AP_GROUPEND
 * };
 * ```
 * 
 * @note Empty name string means no prefix added to inherited parameters
 * @note Commonly uses idx=0 but any valid index works
 */
#define AP_NESTEDGROUPINFO(clazz, idx) { "", 0, { group_info : clazz::var_info }, 0, idx, AP_PARAM_GROUP }

/**
 * @def AP_SUBGROUPINFO(element, name, idx, thisclazz, elclazz)
 * @brief Define nested parameter group in var_info array
 * 
 * @param element Member variable name (object instance of elclazz)
 * @param name Parameter name prefix for nested group
 * @param idx Parameter index within this group (0-63)
 * @param thisclazz Containing class name
 * @param elclazz Nested object class name (must have static var_info[])
 * 
 * @details Creates nested parameter namespace with dot notation.
 *          Parameters from nested object appear as PREFIX_NESTED_PARAM.
 * 
 * Example:
 * ```cpp
 * class MyClass {
 *     CompassOffsets _offsets;
 *     static const AP_Param::GroupInfo var_info[];
 * };
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     AP_SUBGROUPINFO(_offsets, "OFS", 2, MyClass, CompassOffsets),
 *     // Creates parameters: OFS_X, OFS_Y, OFS_Z
 *     AP_GROUPEND
 * };
 * ```
 * 
 * @note name becomes prefix - choose short names to stay within 16 char limit
 * @note Sets AP_PARAM_FLAG_NESTED_OFFSET automatically
 * @warning Changing nesting structure breaks parameter storage compatibility
 */
#define AP_SUBGROUPINFO(element, name, idx, thisclazz, elclazz) { name, AP_VAROFFSET(thisclazz, element), { group_info : elclazz::var_info }, AP_PARAM_FLAG_NESTED_OFFSET, idx, AP_PARAM_GROUP }

/**
 * @def AP_SUBGROUPEXTENSION(name, idx, clazz, vinfo)
 * @brief Declare additional parameter table for same class (extends parameter space)
 * 
 * @param name Parameter name prefix for extension table
 * @param idx Parameter index for extension (typically sequential after main table)
 * @param clazz Class name containing extension table
 * @param vinfo Extension var_info array name (e.g., var_info2, var_info_extension)
 * 
 * @details Extends parameter space beyond single var_info[] array limit.
 *          Used when class has more than 64 parameters (6-bit index limit).
 * 
 * Example:
 * ```cpp
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     // ... 63 parameters (idx 0-62) ...
 *     AP_SUBGROUPEXTENSION("EXT", 63, MyClass, var_info2),
 *     AP_GROUPEND
 * };
 * const AP_Param::GroupInfo MyClass::var_info2[] = {
 *     // Additional parameters with names like EXT_PARAM1, EXT_PARAM2
 *     AP_GROUPINFO("PARAM1", 0, MyClass, _param64, 1.0f),
 *     AP_GROUPEND
 * };
 * ```
 * 
 * @note Zero offset indicates extension is same object, different table
 * @note Rarely needed - most classes fit within 64 parameter limit
 */
#define AP_SUBGROUPEXTENSION(name, idx, clazz, vinfo) { name, 0, { group_info : clazz::vinfo }, AP_PARAM_FLAG_NESTED_OFFSET, idx, AP_PARAM_GROUP }

/**
 * @def AP_SUBGROUPPTR(element, name, idx, thisclazz, elclazz)
 * @brief Define nested parameter group where member is a pointer (dynamically allocated)
 * 
 * @param element Pointer member variable name (pointer to elclazz)
 * @param name Parameter name prefix for nested group
 * @param idx Parameter index within this group
 * @param thisclazz Containing class name
 * @param elclazz Pointed-to class name (must have static var_info[])
 * 
 * @details Like AP_SUBGROUPINFO but follows pointer to find nested parameters.
 *          Used when nested object allocated dynamically or conditionally.
 * 
 * Example:
 * ```cpp
 * class MyClass {
 *     SubSystem *_subsystem;  // Pointer, not direct member
 *     static const AP_Param::GroupInfo var_info[];
 * };
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     AP_SUBGROUPPTR(_subsystem, "SUB", 3, MyClass, SubSystem),
 *     AP_GROUPEND
 * };
 * ```
 * 
 * @note Pointer must be valid (non-null) when parameters accessed
 * @note Sets AP_PARAM_FLAG_POINTER automatically
 * @warning If pointer null at parameter access, causes fault
 */
#define AP_SUBGROUPPTR(element, name, idx, thisclazz, elclazz) { name, AP_VAROFFSET(thisclazz, element), { group_info : elclazz::var_info }, AP_PARAM_FLAG_POINTER, idx, AP_PARAM_GROUP }

/**
 * @def AP_SUBGROUPVARPTR(element, name, idx, thisclazz, var_info)
 * @brief Define nested group with pointer member AND dynamic var_info table
 * 
 * @param element Pointer member variable name
 * @param name Parameter name prefix
 * @param idx Parameter index within group
 * @param thisclazz Containing class name
 * @param var_info Variable holding pointer to var_info array (not direct reference)
 * 
 * @details Double indirection: follows pointer to object, then pointer to var_info.
 *          Used for dynamically constructed parameter tables (scripting, plugins).
 * 
 * Example:
 * ```cpp
 * const AP_Param::GroupInfo *dynamic_var_info = get_dynamic_table();
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     AP_SUBGROUPVARPTR(_plugin, "PLUG", 5, MyClass, dynamic_var_info),
 *     AP_GROUPEND
 * };
 * ```
 * 
 * @note Sets both AP_PARAM_FLAG_POINTER and AP_PARAM_FLAG_INFO_POINTER
 * @note Rarely used - primarily for AP_Scripting dynamic tables
 * @warning Both pointers must be valid when parameters accessed
 */
#define AP_SUBGROUPVARPTR(element, name, idx, thisclazz, var_info) { name, AP_VAROFFSET(thisclazz, element), { group_info_ptr : &var_info }, AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER, idx, AP_PARAM_GROUP }

/**
 * @def AP_GROUPEND
 * @brief Sentinel marking end of var_info[] array
 * 
 * @details Terminates var_info[] array with recognizable sentinel entry.
 *          Index 0xFF and type AP_PARAM_NONE signal end of table.
 *          Required at end of every var_info[] array.
 * 
 * Example:
 * ```cpp
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     AP_GROUPINFO("PARAM1", 0, MyClass, _param1, 1.0f),
 *     AP_GROUPINFO("PARAM2", 1, MyClass, _param2, 2.0f),
 *     AP_GROUPEND  // Required terminator
 * };
 * ```
 * 
 * @note Always required - missing AP_GROUPEND causes memory corruption
 * @warning Forgetting AP_GROUPEND leads to reading invalid memory
 */
#define AP_GROUPEND     { "", 0,       { group_info : nullptr }, 0, 0xFF, AP_PARAM_NONE }

/**
 * @def GSCALAR(v, name, def)
 * @brief Define top-level scalar parameter from Parameters::g struct
 * 
 * @param v Member variable name in Parameters::g
 * @param name Parameter name string
 * @param def Default value
 * 
 * @details Used in vehicle Parameters.cpp to declare top-level scalar parameters
 *          stored in the global Parameters::g structure.
 * 
 * Example:
 * ```cpp
 * const AP_Param::Info var_info[] = {
 *     GSCALAR(sysid_this_mav, "SYSID_THISMAV", 1),
 *     GSCALAR(sysid_my_gcs, "SYSID_MYGCS", 255),
 *     // ...
 * };
 * ```
 * 
 * @note Vehicle-specific macro - requires Parameters class structure
 * @note Key auto-generated from variable name (k_param_sysid_this_mav)
 */
#define GSCALAR(v, name, def)                { name, &AP_PARAM_VEHICLE_NAME.g.v,                   {def_value : def},                   0,                                                  Parameters::k_param_ ## v,          AP_PARAM_VEHICLE_NAME.g.v.vtype }

/**
 * @def GARRAY(v, index, name, def)
 * @brief Define top-level array element parameter from Parameters::g struct
 * 
 * @param v Array member variable name in Parameters::g
 * @param index Array element index
 * @param name Parameter name string
 * @param def Default value
 * 
 * @details Used for array parameters where each element has unique name and key.
 * 
 * Example:
 * ```cpp
 * GARRAY(rc_channel, 0, "RC1_OPTION", 0),
 * GARRAY(rc_channel, 1, "RC2_OPTION", 0),
 * ```
 * 
 * @note Key combines variable name and index (k_param_rc_channel0)
 * @note Each array element consumes separate top-level key
 */
#define GARRAY(v, index, name, def)          { name, &AP_PARAM_VEHICLE_NAME.g.v[index],            {def_value : def},                   0,                                                  Parameters::k_param_ ## v ## index, AP_PARAM_VEHICLE_NAME.g.v[index].vtype }

/**
 * @def ASCALAR(v, name, def)
 * @brief Define top-level scalar parameter from Parameters::aparm struct
 * 
 * @param v Member variable name in Parameters::aparm
 * @param name Parameter name string
 * @param def Default value
 * 
 * @details Used for common AP_Vehicle parameters stored in aparm structure.
 *          Similar to GSCALAR but references different parent struct.
 * 
 * @note aparm typically holds AP_Vehicle common parameters
 */
#define ASCALAR(v, name, def)                { name, (const void *)&AP_PARAM_VEHICLE_NAME.aparm.v, {def_value : def},                   0,                                                  Parameters::k_param_ ## v,          AP_PARAM_VEHICLE_NAME.aparm.v.vtype }

/**
 * @def GGROUP(v, name, class)
 * @brief Define top-level parameter group from Parameters::g struct
 * 
 * @param v Member variable name in Parameters::g
 * @param name Parameter name prefix
 * @param class Class name containing var_info[] array
 * 
 * @details Declares parameter group at top level of parameter tree.
 *          All class parameters appear with given name prefix.
 * 
 * Example:
 * ```cpp
 * GGROUP(compass, "COMPASS_", Compass),
 * // Creates parameters: COMPASS_OFS_X, COMPASS_OFS_Y, etc.
 * ```
 */
#define GGROUP(v, name, class)               { name, &AP_PARAM_VEHICLE_NAME.g.v,                   {group_info : class::var_info},      0,                                                  Parameters::k_param_ ## v,          AP_PARAM_GROUP }

/**
 * @def GOBJECT(v, name, class)
 * @brief Define top-level parameter group from vehicle class member
 * 
 * @param v Member variable name in vehicle class (not Parameters::g)
 * @param name Parameter name prefix
 * @param class Class name containing var_info[] array
 * 
 * @details Used for vehicle class members that are objects with parameters.
 *          Object is direct member (not pointer).
 * 
 * Example:
 * ```cpp
 * GOBJECT(ins, "INS_", AP_InertialSensor),
 * GOBJECT(ahrs, "AHRS_", AP_AHRS),
 * ```
 */
#define GOBJECT(v, name, class)              { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info : class::var_info},      0,                                                  Parameters::k_param_ ## v,          AP_PARAM_GROUP }

/**
 * @def GOBJECTPTR(v, name, class)
 * @brief Define top-level parameter group where vehicle member is pointer
 * 
 * @param v Pointer member variable name in vehicle class
 * @param name Parameter name prefix
 * @param class Class name containing var_info[] array
 * 
 * @details Like GOBJECT but member is pointer to object (dynamically allocated).
 * 
 * Example:
 * ```cpp
 * GOBJECTPTR(camera, "CAM_", AP_Camera),
 * ```
 * 
 * @note Pointer must be valid when parameters accessed
 */
#define GOBJECTPTR(v, name, class)           { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info : class::var_info},      AP_PARAM_FLAG_POINTER,                              Parameters::k_param_ ## v,          AP_PARAM_GROUP }

/**
 * @def GOBJECTVARPTR(v, name, var_info_ptr)
 * @brief Define top-level group with dynamic var_info table pointer
 * 
 * @param v Pointer member variable name
 * @param name Parameter name prefix
 * @param var_info_ptr Variable holding pointer to var_info array
 * 
 * @details Double indirection for dynamically constructed parameter tables.
 * 
 * @note Used for scripting and dynamically loaded modules
 */
#define GOBJECTVARPTR(v, name, var_info_ptr) { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info_ptr : var_info_ptr},     AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER, Parameters::k_param_ ## v,          AP_PARAM_GROUP }

/**
 * @def GOBJECTN(v, pname, name, class)
 * @brief Define top-level group with explicit parameter key name
 * 
 * @param v Member variable name in vehicle class
 * @param pname Parameter key enum name (without k_param_ prefix)
 * @param name Parameter name prefix string
 * @param class Class name containing var_info[] array
 * 
 * @details Like GOBJECT but allows decoupling enum name from variable name.
 *          Used when variable name differs from desired parameter key.
 * 
 * Example:
 * ```cpp
 * GOBJECTN(g2.scripting, scripting, "SCR_", AP_Scripting),
 * // Uses k_param_scripting key with g2.scripting variable
 * ```
 */
#define GOBJECTN(v, pname, name, class)      { name, (const void *)&AP_PARAM_VEHICLE_NAME.v,       {group_info : class::var_info},      0,                                                  Parameters::k_param_ ## pname,      AP_PARAM_GROUP }

/**
 * @def PARAM_VEHICLE_INFO
 * @brief Include common AP_Vehicle parameters in vehicle parameter table
 * 
 * @details First entry in vehicle var_info[] table incorporating base
 *          vehicle class parameters. Uses k_param_vehicle key.
 * 
 * Example:
 * ```cpp
 * const AP_Param::Info Copter::var_info[] = {
 *     PARAM_VEHICLE_INFO,  // AP_Vehicle parameters first
 *     GSCALAR(sysid_this_mav, "SYSID_THISMAV", 1),
 *     // ... vehicle-specific parameters ...
 * };
 * ```
 * 
 * @note Typically first entry in every vehicle parameter table
 */
#define PARAM_VEHICLE_INFO                   { "",   (const void *)&AP_PARAM_VEHICLE_NAME,         {group_info : AP_Vehicle::var_info}, 0,                                                  Parameters::k_param_vehicle,        AP_PARAM_GROUP }

/**
 * @def AP_VAREND
 * @brief Sentinel marking end of top-level var_info[] array
 * 
 * @details Terminates vehicle top-level parameter table.
 *          Required at end of every top-level var_info[] array.
 * 
 * Example:
 * ```cpp
 * const AP_Param::Info Copter::var_info[] = {
 *     PARAM_VEHICLE_INFO,
 *     GSCALAR(sysid_this_mav, "SYSID_THISMAV", 1),
 *     AP_VAREND  // Required terminator
 * };
 * ```
 * 
 * @warning Missing AP_VAREND causes memory corruption during parameter enumeration
 */
#define AP_VAREND                            { "",   nullptr,                                      {group_info : nullptr },             0,                                                  0,                                  AP_PARAM_NONE }

/**
 * @def AP_GROUP_ELEM_IDX(subgrp_idx, grp_idx)
 * @brief Combine nested group indices into single group_element value
 * 
 * @param subgrp_idx Sub-group index (0-63, lower 6 bits)
 * @param grp_idx Parent group index (0-63, upper 6 bits)
 * 
 * @return uint16_t Combined group_element encoding (12 bits total)
 * 
 * @details Encodes two-level nesting into 12-bit value for storage header.
 *          Supports up to 3 nesting levels in full 18-bit group_element.
 * 
 * @note Internal macro - parameter system uses automatically
 * @note Formula: (grp_idx << 6) | subgrp_idx
 */
#define AP_GROUP_ELEM_IDX(subgrp_idx, grp_idx) (grp_idx << 6 | subgrp_idx)

/**
 * @enum ap_var_type
 * @brief Parameter type enumeration for storage and type safety
 * 
 * @details Identifies parameter storage type in var_info tables and storage headers.
 *          Each type has specific storage size and serialization format.
 *          Type encoded in 5-bit field of storage header.
 */
enum ap_var_type {
    /**
     * @brief No parameter type (used for sentinels and uninitialized entries)
     */
    AP_PARAM_NONE    = 0,
    
    /**
     * @brief 8-bit signed integer parameter (-128 to 127)
     * @note Storage size: 1 byte
     * @see AP_Int8
     */
    AP_PARAM_INT8,
    
    /**
     * @brief 16-bit signed integer parameter (-32768 to 32767)
     * @note Storage size: 2 bytes
     * @see AP_Int16
     */
    AP_PARAM_INT16,
    
    /**
     * @brief 32-bit signed integer parameter (-2^31 to 2^31-1)
     * @note Storage size: 4 bytes
     * @see AP_Int32
     */
    AP_PARAM_INT32,
    
    /**
     * @brief 32-bit IEEE-754 floating-point parameter
     * @note Storage size: 4 bytes
     * @note Precision: ~7 decimal digits
     * @see AP_Float
     */
    AP_PARAM_FLOAT,
    
    /**
     * @brief 3D vector of floats (x, y, z components)
     * @note Storage size: 12 bytes (3 × float)
     * @note Can be accessed as vector or individual scalar elements (_X, _Y, _Z)
     * @see AP_Vector3f
     */
    AP_PARAM_VECTOR3F,
    
    /**
     * @brief Parameter group container (not a scalar value)
     * @note No direct storage - contains nested parameters
     * @note Used for nested parameter groups and objects
     */
    AP_PARAM_GROUP
};


/**
 * @class AP_Param
 * @brief Core parameter management system for ArduPilot persistent configuration
 * 
 * @details AP_Param provides the foundation for ArduPilot's parameter system,
 *          managing thousands of configuration parameters across vehicle code and libraries.
 * 
 * Key responsibilities:
 * - **Registration**: Vehicle code registers parameters via var_info[] tables
 * - **Persistence**: Saves parameters to non-volatile storage (EEPROM/Flash/FRAM)
 * - **Lookup**: Find parameters by name, index, or memory pointer
 * - **Enumeration**: Iterate all registered parameters for GCS downloads
 * - **Migration**: Convert parameters from old firmware versions during upgrades
 * - **Protocol**: Implement MAVLink parameter protocol (PARAM_REQUEST_LIST, PARAM_SET, etc.)
 * - **Defaults**: Load defaults from embedded firmware or external files
 * 
 * Storage architecture:
 * - Binary format in EEPROM/Flash with header + parameter headers + payloads
 * - 9-bit keys (0-511) identify top-level parameter groups
 * - 18-bit group_element field encodes nested group paths (3 levels × 6 bits)
 * - Asynchronous saves via queue + I/O thread prevent blocking flight control
 * 
 * Typical usage:
 * ```cpp
 * // 1. Declare parameters in class
 * class MyClass {
 *     AP_Int16 _param1;
 *     AP_Float _param2;
 *     static const AP_Param::GroupInfo var_info[];
 * };
 * 
 * // 2. Define var_info[] registration table
 * const AP_Param::GroupInfo MyClass::var_info[] = {
 *     AP_GROUPINFO("PARAM1", 0, MyClass, _param1, 10),
 *     AP_GROUPINFO("PARAM2", 1, MyClass, _param2, 1.5f),
 *     AP_GROUPEND
 * };
 * 
 * // 3. Register at vehicle level
 * const AP_Param::Info Parameters::var_info[] = {
 *     GOBJECT(my_object, "MY_", MyClass),
 *     AP_VAREND
 * };
 * 
 * // 4. Access parameters
 * int16_t value = my_object._param1;           // Read
 * my_object._param2.set_and_save(2.0f);        // Write and persist
 * ```
 * 
 * Thread safety:
 * - Public methods are thread-safe for concurrent access
 * - Save operations queued and processed on I/O thread
 * - Semaphore protects cached parameter count
 * 
 * @note Core system dependency - all ArduPilot configuration relies on AP_Param
 * @note Parameter keys and indices must remain stable across firmware versions
 * @warning Corrupting parameter storage causes loss of all vehicle configuration
 * 
 * @see AP_ParamT for typed parameter wrappers (AP_Int8, AP_Int16, AP_Int32, AP_Float)
 * @see AP_ParamV for vector parameters (AP_Vector3f)
 * @see StorageManager for storage backend abstraction
 * @see GCS_MAVLink for parameter protocol implementation
 */
class AP_Param
{
public:
    /**
     * @struct GroupInfo
     * @brief Metadata descriptor for a parameter within a parameter group
     * 
     * @details Defines one entry in a var_info[] array describing a parameter's:
     * - Name: Human-readable identifier (max AP_MAX_NAME_SIZE chars)
     * - Location: Memory offset within containing object
     * - Default: Initial value if not stored or factory reset
     * - Type: Scalar (int/float) or nested group
     * - Index: Position identifier within group (0-63 per nesting level)
     * - Flags: Behavior modifiers (pointer, nested, enable, frame-specific, etc.)
     * 
     * Union default value variants:
     * - def_value: Direct float default for scalars
     * - def_value_offset: Offset-based default via FLAG_DEFAULT_POINTER
     * - group_info: Pointer to nested group var_info array
     * - group_info_ptr: Pointer to pointer (for FLAG_INFO_POINTER)
     * 
     * Flags control behavior:
     * - FLAG_NESTED_OFFSET: Nested group without inheritance
     * - FLAG_POINTER: Object allocated dynamically (follow pointer)
     * - FLAG_ENABLE: Enable/disable entire parameter subtree
     * - FLAG_INFO_POINTER: var_info is pointer (dynamic tables)
     * - FLAG_INTERNAL_USE_ONLY: GCS visible but firmware-controlled
     * - FLAG_HIDDEN: Hide from GCS parameter list
     * - FLAG_DEFAULT_POINTER: Default is offset from containing object
     * - Frame flags (bits 8+): Vehicle/frame-specific visibility (COPTER, PLANE, ROVER, etc.)
     * 
     * Typical usage in var_info[] array:
     * ```cpp
     * const AP_Param::GroupInfo MyClass::var_info[] = {
     *     AP_GROUPINFO("PARAM1", 0, MyClass, _param1, 1.0f),
     *     AP_GROUPINFO_FLAGS("PARAM2", 1, MyClass, _param2, 2.0f, AP_PARAM_FLAG_ENABLE),
     *     AP_SUBGROUPINFO(_subgroup, "SUB", 2, MyClass, SubClass),
     *     AP_GROUPEND
     * };
     * ```
     * 
     * @note idx must be unique within group and remain stable across firmware versions
     * @note Changing idx breaks parameter storage compatibility
     * @warning Reusing idx for different parameter loses user configuration
     */
    struct GroupInfo {
        const char *name;
        ptrdiff_t offset; // offset within the object
        union {
            const struct GroupInfo *group_info;
            const struct GroupInfo **group_info_ptr; // when AP_PARAM_FLAG_INFO_POINTER is set in flags
            const float def_value;
            ptrdiff_t def_value_offset; // Default value offset from param object, when AP_PARAM_FLAG_DEFAULT_POINTER is set in flags
        };
        uint16_t flags;
        uint8_t idx;  // identifier within the group
        uint8_t type; // AP_PARAM_*
    };
    /**
     * @struct Info
     * @brief Top-level parameter table entry mapping names to memory locations
     * 
     * @details Defines one entry in the _var_info[] table passed to AP_Param constructor.
     *          Similar to GroupInfo but for top-level parameter groups with assigned keys.
     *          
     *          Key assignment:
     *          - Each top-level entry has unique 9-bit key (0-511)
     *          - Keys defined in vehicle Parameters.h as k_param_* enum
     *          - Key stability critical for storage compatibility across versions
     * 
     * @note Top-level table defined in vehicle-specific Parameters.cpp
     * @note First entry typically uses PARAM_VEHICLE_INFO macro for AP_Vehicle parameters
     */
    struct Info {
        const char *name;
        const void *ptr;    // pointer to the variable in memory
        union {
            const struct GroupInfo *group_info;
            const struct GroupInfo **group_info_ptr; // when AP_PARAM_FLAG_INFO_POINTER is set in flags
            const float def_value;
            ptrdiff_t def_value_offset; // Default value offset from param object, when AP_PARAM_FLAG_DEFAULT_POINTER is set in flags
        };
        uint16_t flags;
        uint16_t key; // k_param_*
        uint8_t type; // AP_PARAM_*
    };
    /**
     * @struct ConversionInfo  
     * @brief Descriptor for parameter migration from old firmware to new
     * 
     * @details Specifies how to locate and convert a parameter from previous
     *          firmware version. Used in convert_old_parameters() to preserve
     *          user configuration across refactoring.
     *          
     *          Migration scenarios:
     *          - Renaming: old_key + old_group_element → new_name
     *          - Moving: Parameter relocated to different group or top-level
     *          - Type change: int8 → int16, int16 → float with scaling
     *          - Class reorganization: G2 object → AP_Vehicle members
     * 
     * @note Conversion tables must remain indefinitely for user upgrade paths
     * @warning Removing conversion breaks users upgrading from old firmware
     */
    struct ConversionInfo {
        uint16_t old_key; // k_param_*
        uint32_t old_group_element; // index in old object
        enum ap_var_type type; // AP_PARAM_*
        const char *new_name;
    };

    // param default table element
    struct defaults_table_struct {
        const char *name;   // parameter name
        float value;        // parameter value
    };

    /**
     * @brief Initialize parameter system and verify/format storage
     * 
     * @details Performs one-time initialization at boot:
     * - Validates EEPROM header magic bytes and revision
     * - Reformats storage if header invalid or version mismatch
     * - Scans storage to find sentinel (end of parameters)
     * - Validates var_info table consistency (no duplicate keys)
     * - Sets up save queue and I/O thread integration
     * 
     * @return bool true if initialization successful, false on critical error
     * 
     * @note Called once from vehicle setup() before parameter loads
     * @note Reformat erases all stored parameters - users lose configuration
     * @warning Failure prevents parameter system from functioning
     */
    static bool setup();

    // constructor with var_info
    AP_Param(const struct Info *info)
    {
        _var_info = info;
        uint16_t i;
        for (i=0; info[i].type != AP_PARAM_NONE; i++) ;
        _num_vars = i;
#if AP_PARAM_DYNAMIC_ENABLED
        _num_vars_base = _num_vars;
#endif
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_Param must be singleton");
        }
        _singleton = this;
    }

    // empty constructor
    AP_Param() {}

    /**
     * @struct ParamToken
     * @brief Opaque iterator state for first()/next() parameter enumeration
     * 
     * @details Bitfield structure encoding current enumeration position:
     * - key (9 bits): Top-level var_info index
     * - group_element (18 bits): Nested group path (3 levels × 6 bits)
     * - idx (4 bits): Array element index for vector types
     * - last_disabled (1 bit): Skip disabled subtrees flag
     * 
     * @note Treat as opaque - internal structure may change
     * @note Zero-initialized token starts enumeration via first()
     */
    typedef struct {
        uint32_t key : 9;
        uint32_t idx : 4; // offset into array types
        uint32_t group_element : 18;
        uint32_t last_disabled : 1;
    } ParamToken;


    // nesting structure for recursive call states
    struct GroupNesting {
        static const uint8_t numlevels = 2;
        uint8_t level;
        const struct GroupInfo *group_ret[numlevels];
    };

    // return true if AP_Param has been initialised via setup()
    static bool initialised(void);

    // the 'group_id' of a element of a group is the 18 bit identifier
    // used to distinguish between this element of the group and other
    // elements of the same group. It is calculated using a bit shift per
    // level of nesting, so the first level of nesting gets 6 bits the 2nd
    // level gets the next 6 bits, and the 3rd level gets the last 6
    // bits. This limits groups to having at most 64 elements.
    static uint32_t group_id(const struct GroupInfo *grpinfo, uint32_t base, uint8_t i, uint8_t shift);

    /**
     * @brief Copy parameter name with group prefix to buffer
     * 
     * @param[in] info Top-level parameter info structure
     * @param[in] ginfo Group info structure for nested parameters
     * @param[in] group_nesting Nesting path through parameter groups
     * @param[in] idx Array element index for vector parameters
     * @param[out] buffer Destination buffer for parameter name
     * @param[in] bufferSize Total size of destination buffer
     * @param[in] force_scalar If true, append _X/_Y/_Z suffix for vector elements
     * 
     * @details Constructs full parameter name by traversing group nesting hierarchy.
     *          For nested parameters, joins group names with underscores.
     *          
     *          Example: For COMPASS_OFS_X in nested groups:
     *          "COMPASS" + "_" + "OFS" + "_X" = "COMPASS_OFS_X"
     * 
     * @note If variable has no name, buffer contains empty string
     * @note If combined name exceeds bufferSize, result is truncated
     * @note Buffer is always null-terminated
     */
    void copy_name_info(const struct AP_Param::Info *info,
                        const struct GroupInfo *ginfo,
                        const struct GroupNesting &group_nesting,
                        uint8_t idx, char *buffer, size_t bufferSize, bool force_scalar=false) const;

    /**
     * @brief Copy parameter name to buffer using token
     * 
     * @param[in] token ParamToken from first()/next() enumeration
     * @param[out] buffer Destination buffer for parameter name
     * @param[in] bufferSize Total size of destination buffer
     * @param[in] force_scalar If true, append _X/_Y/_Z suffix for vector elements
     * 
     * @details Uses token to look up parameter Info and constructs full name.
     *          Convenience wrapper around copy_name_info() for enumeration use case.
     * 
     * @note Typical use during parameter list download (MAVLink PARAM_REQUEST_LIST)
     * @note Buffer is always null-terminated
     * @note If name exceeds bufferSize, result is truncated
     */
    void copy_name_token(const ParamToken &token, char *buffer, size_t bufferSize, bool force_scalar=false) const;

    /**
     * @brief Locate parameter by name for reading or modification
     * 
     * @param[in] name Full parameter name with dots for nested groups (e.g., "COMPASS_OFS_X")
     * @param[out] ptype Optional pointer to receive parameter type (AP_PARAM_INT8, etc.)
     * @param[out] flags Optional pointer to receive parameter flags
     * 
     * @return AP_Param* Pointer to parameter in memory, or nullptr if not found
     * 
     * @note Names are case-sensitive
     * @note Vector parameters (AP_Vector3f) can be accessed as vector or individual scalars (_X/_Y/_Z suffix)
     * @note Use set_value() or cast to appropriate AP_ParamT<> type to modify value
     * 
     * @warning Returned pointer invalid if parameter storage reallocated (rare)
     */
    static AP_Param * find(const char *name, enum ap_var_type *ptype, uint16_t *flags = nullptr);

    /**
     * @brief Set default value for parameter by name
     * 
     * @param[in] name Full parameter name (e.g., "COMPASS_OFS_X")
     * @param[in] value Default value to assign
     * 
     * @return bool true if parameter found and default set, false if name not found
     * 
     * @details Sets default value in defaults_list for parameter. This default
     *          is used during factory reset or if parameter not found in storage.
     *          Does not modify current RAM value or persistent storage.
     * 
     * @note Typically used during initialization to set programmatic defaults
     * @note Does not trigger save or notify - only affects future loads
     */
    static bool set_default_by_name(const char *name, float value);

    /**
     * @brief Set multiple parameter defaults from table
     * 
     * @param[in] table Pointer to array of defaults_table_struct structures
     * @param[in] count Number of elements in table array
     * 
     * @details Bulk default value assignment from table. Each table entry specifies
     *          parameter name and default value. Efficient alternative to multiple
     *          set_default_by_name() calls.
     * 
     * @note Table typically defined in vehicle-specific defaults.cpp
     * @note Called during boot after parameter registration
     */
    static void set_defaults_from_table(const struct defaults_table_struct *table, uint8_t count);

    /**
     * @brief Set parameter value by name (runtime parameter modification)
     * 
     * @param[in] name Full parameter name (e.g., "COMPASS_OFS_X")
     * @param[in] value New value to assign
     * 
     * @return bool true if parameter found and set, false if name not found
     * 
     * @details Updates RAM only (volatile, lost on reboot)
     *          
     *          Typical uses:
     *          - GCS parameter set commands (MAVLink PARAM_SET)
     *          - Lua scripting parameter modification
     *          - Auto-tuning writing optimized PIDs
     *          - Mission commands changing parameters
     * 
     * @note Type conversion automatic: float value converted to int8/int16/int32 as needed
     * @warning Setting flight-critical parameters mid-flight can cause crashes
     */
    static bool set_by_name(const char *name, float value);

    /**
     * @brief Get parameter value by name (runtime parameter read)
     * 
     * @param[in] name Full parameter name (e.g., "COMPASS_OFS_X")
     * @param[out] value Reference to receive parameter value
     * 
     * @return bool true if parameter found and value retrieved, false if name not found
     * 
     * @details Reads current parameter value from RAM (not storage).
     *          Automatic type conversion to float from underlying storage type.
     * 
     * @note Primarily used by scripting interface for parameter access
     * @note Integer parameters converted to float without scaling
     * @note Vector parameters require _X/_Y/_Z suffix for individual elements
     */
    static bool get(const char *name, float &value);

    /**
     * @brief Set parameter value by name and persist to storage
     * 
     * @param[in] name Full parameter name (e.g., "COMPASS_OFS_X")
     * @param[in] value New value to assign
     * 
     * @return bool true if parameter found and set, false if name not found
     * 
     * @details Updates RAM + queues persistent save
     *          
     *          Typical uses:
     *          - GCS parameter set commands (MAVLink PARAM_SET)
     *          - Lua scripting parameter modification
     *          - Auto-tuning writing optimized PIDs
     *          - Mission commands changing parameters
     * 
     * @note Type conversion automatic: float value converted to int8/int16/int32 as needed
     * @note Respects read-only flags and permission checks
     * @warning Setting flight-critical parameters mid-flight can cause crashes
     */
    static bool set_and_save_by_name(const char *name, float value);
    static bool set_and_save_by_name_ifchanged(const char *name, float value);

    /**
     * @brief Find parameter by enumeration index
     * 
     * @param[in] idx Enumeration index (0 to count_parameters()-1)
     * @param[out] ptype Optional pointer to receive parameter type
     * @param[out] token Optional pointer to receive ParamToken for this parameter
     * 
     * @return AP_Param* Pointer to parameter, or nullptr if index invalid
     * 
     * @details Locates parameter by its position in enumeration order.
     *          Index corresponds to position in first()/next() traversal.
     *          
     *          Used by MAVLink PARAM_REQUEST_READ by index.
     * 
     * @note Index order is deterministic but not alphabetical
     * @note Expensive operation - prefer find() by name when possible
     */
    static AP_Param * find_by_index(uint16_t idx, enum ap_var_type *ptype, ParamToken *token);

    /**
     * @brief Find parameter by name and return token
     * 
     * @param[in] name Full parameter name
     * @param[out] ptype Optional pointer to receive parameter type
     * @param[out] token Optional pointer to receive ParamToken
     * 
     * @return AP_Param* Pointer to parameter, or nullptr if not found
     * 
     * @details Equivalent to find() but also returns token for enumeration context.
     *          Token can be used with copy_name_token() and other token-based methods.
     */
    static AP_Param* find_by_name(const char* name, enum ap_var_type *ptype, ParamToken *token);

    /**
     * @brief Find storage key for parameter by memory pointer (internal)
     * 
     * @param[in] ptr Pointer to parameter in memory
     * @param[in] vindex Top-level var_info index
     * @param[in] group_info Group metadata if nested
     * @param[in] offset Memory offset adjustment for groups
     * @param[out] key Storage key (9-bit top-level key)
     * 
     * @return bool true if key found, false otherwise
     * 
     * @note Internal helper for find_key_by_pointer()
     * @note Recursively searches nested groups
     */
    static bool find_key_by_pointer_group(const void *ptr, uint16_t vindex, const struct GroupInfo *group_info,
                                          ptrdiff_t offset, uint16_t &key);
    
    /**
     * @brief Find storage key for parameter by memory pointer
     * 
     * @param[in] ptr Pointer to parameter variable in memory
     * @param[out] key Storage key for parameter
     * 
     * @return bool true if parameter found and key retrieved, false otherwise
     * 
     * @details Reverse lookup: given parameter address, find storage key.
     *          Used internally for save operations.
     * 
     * @note Searches entire var_info hierarchy
     */
    static bool find_key_by_pointer(const void *ptr, uint16_t &key);

    /**
     * @brief Find top-level storage key by pointer
     * 
     * @param[in] ptr Pointer to top-level parameter or group object
     * @param[out] key Top-level storage key (9-bit)
     * 
     * @return bool true if top-level key found, false otherwise
     * 
     * @details Finds key for top-level var_info entry only, not nested parameters.
     *          Used for group-level operations.
     */
    static bool find_top_level_key_by_pointer(const void *ptr, uint16_t &key);


    /**
     * @brief Find top-level parameter object by name
     * 
     * @param[in] name Full parameter name
     * 
     * @return AP_Param* Pointer to parameter object, or nullptr if not found
     * 
     * @details Finds parameter in top-level var_info table only.
     *          Does not search nested groups - name must match top-level entry.
     * 
     * @note Variable must have name to be found
     * @note Faster than find() for top-level parameters
     * @note Returns nullptr for nested parameters
     */
    static AP_Param * find_object(const char *name);

    /**
     * @brief Notify all connected GCS of current parameter value
     * 
     * @details Sends PARAM_VALUE MAVLink message to all connected ground stations
     *          with current parameter value. Used after runtime parameter changes
     *          to keep GCS synchronized.
     * 
     * @note Non-blocking - queues message for transmission
     * @note Called automatically by set_and_notify() and set_and_save()
     */
    void notify() const;

    /**
     * @brief Save parameter to storage synchronously (blocking)
     * 
     * @param[in] force_save If true, save even if value equals default
     * @param[in] send_to_gcs If true, notify GCS after save
     * 
     * @details Immediately writes parameter to persistent storage, blocking
     *          until I/O complete. Used in critical paths where save must
     *          complete before proceeding (e.g., before reboot).
     * 
     * @warning Blocks calling thread during flash write (10-50ms)
     * @warning Do not call from flight-critical code - use save() instead
     * @note Prefer save() for non-blocking queued save
     */
    void save_sync(bool force_save, bool send_to_gcs);

    /**
     * @brief Flush all pending parameter saves to storage
     * 
     * @details Blocks until save queue is empty and all pending saves complete.
     *          Ensures all parameter changes are persisted to storage.
     * 
     * @note Called before reboot to ensure configuration not lost
     * @note Blocking operation - may take 100-500ms if queue deep
     * @warning Must be called before power loss to preserve unsaved changes
     */
    static void flush(void);

    /**
     * @brief Queue parameter for asynchronous save to persistent storage
     * 
     * @param[in] force_save If true, save even if value equals default
     * 
     * @details Adds parameter to save queue processed by I/O thread's save_io_handler().
     *          Deferred saving prevents blocking flight-critical code during flash writes.
     *          Multiple rapid changes coalesced into single write for efficiency.
     *          
     *          Save timing:
     *          - Queued immediately (microseconds)
     *          - Physical write within 50-200ms depending on queue depth
     *          - flush() forces immediate completion (use before reboot)
     * 
     * @note Non-blocking - returns immediately without waiting for storage write
     * @note Changes visible to other code immediately (RAM updated)
     * @warning If storage full, save silently fails - check get_eeprom_full()
     */
    void save(bool force_save=false);

    /**
     * @brief Load parameter value from persistent storage into memory
     * 
     * @return bool true if parameter found and loaded from storage, false if not stored
     * 
     * @details Searches storage for parameter header matching this parameter's
     *          key and group_element. If found, reads value payload into memory.
     *          If not found, retains current memory value (typically default).
     * 
     * @note Rarely called directly - load_all() loads all parameters at boot
     * @note Missing from storage is not an error - indicates default value in use
     */
    bool load(void);

    /**
     * @brief Load all registered parameters from persistent storage at boot
     * 
     * @return bool true if all parameters loaded successfully, false if any errors
     * 
     * @details Boot sequence parameter loading:
     * 1. Scan entire storage region for parameter headers
     * 2. Match each header to registered var_info entry by key
     * 3. Load value payload into parameter memory location
     * 4. Apply any defaults from embedded or external defaults files
     * 5. Process parameter overrides from defaults files
     * 
     * @note Called once during vehicle initialization after var_info registration
     * @note Parameters not found in storage retain compiled-in defaults
     * @note Logs warning for parameters found in storage but no longer registered
     */
    static bool load_all();

    // return true if eeprom is full, used for arming check
    static bool get_eeprom_full(void) {
        return eeprom_full;
    }

    // returns storage space used:
    static uint16_t storage_used() { return sentinal_offset; }

    // returns storage space :
    static uint16_t storage_size() { return _storage.size(); }

    /**
     * @brief Reload defaults file after dynamic parameter allocation
     * 
     * @param[in] last_pass If true, this is final defaults load pass
     * 
     * @details Reloads external defaults file (typically from filesystem) after
     *          pointer-based parameters have been allocated. Required because
     *          pointer parameters don't exist until runtime allocation completes.
     *          
     *          Two-pass loading:
     *          - First pass: Load defaults for statically allocated parameters
     *          - Second pass: Load defaults for dynamically allocated parameters
     * 
     * @note Called from vehicle initialization after pointer parameter setup
     * @note Only effective if AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED
     */
    static void reload_defaults_file(bool last_pass);

    static void load_object_from_eeprom(const void *object_pointer, const struct GroupInfo *group_info);

    // set a AP_Param variable to a specified value
    static void         set_value(enum ap_var_type type, void *ptr, float def_value);

    /*
      set a parameter to a float
    */
    void set_float(float value, enum ap_var_type var_type);

    // load default values for scalars in a group
    static void         setup_object_defaults(const void *object_pointer, const struct GroupInfo *group_info);

    // set a value directly in an object.
    // return true if the name was found and set, else false.
    // This should only be used by example code, not by mainline vehicle code
    static bool set_object_value(const void *object_pointer,
                                 const struct GroupInfo *group_info,
                                 const char *name, float value);

    // load default values for all scalars in the main sketch. This
    // does not recurse into the sub-objects    
    static void         setup_sketch_defaults(void);

    // find an old parameter and return it.
    static bool find_old_parameter(const struct ConversionInfo *info, AP_Param *value);

    /**
     * @brief Migrate parameters from previous firmware version during upgrade
     * 
     * @param[in] conversion_table Array of ConversionInfo descriptors
     * @param[in] table_size Number of entries in conversion_table
     * @param[in] flags Optional flags (CONVERT_FLAG_REVERSE, CONVERT_FLAG_FORCE)
     * 
     * @details Searches storage for parameters using old keys/group_elements,
     *          renames/remaps them to new locations, and optionally scales values.
     *          Preserves user configuration across firmware refactoring.
     *          
     *          Called during boot after load_all() to migrate legacy parameters.
     *          Migration is one-time per firmware upgrade - parameters resaved
     *          with new keys after conversion.
     * 
     * @note Conversion tables accumulate over time - never remove old conversions
     * @note CONVERT_FLAG_REVERSE handles _REV → _REVERSED suffix changes  
     * @note CONVERT_FLAG_FORCE overwrites parameters even if already configured
     * @warning Missing conversion causes users to lose configured parameters
     */
    static void         convert_old_parameters(const struct ConversionInfo *conversion_table, uint8_t table_size, uint8_t flags=0);
    // convert old vehicle parameters to new object parameters with scaling - assumes we use the same scaling factor for all values in the table
    static void         convert_old_parameters_scaled(const ConversionInfo *conversion_table, uint8_t table_size, float scaler, uint8_t flags);

    // convert an object which was stored in a vehicle's G2 into a new
    // object in AP_Vehicle.cpp:
    struct G2ObjectConversion {
        void *object_pointer;
        const struct AP_Param::GroupInfo *var_info;
        uint16_t old_index;  // Old parameter index in g2
    };
    static void         convert_g2_objects(const void *g2, const G2ObjectConversion g2_conversions[], uint8_t num_conversions);

    // convert an object which was stored in a vehicle's top-level
    // Parameters object into a new object in AP_Vehicle.cpp:
    struct TopLevelObjectConversion {
        void *object_pointer;
        const struct AP_Param::GroupInfo *var_info;
        uint16_t old_index;  // Old parameter index in g
    };
    static void         convert_toplevel_objects(const TopLevelObjectConversion g2_conversions[], uint8_t num_conversions);

    /*
      convert width of a parameter, allowing update to wider scalar
      values without changing the parameter indexes. This will return
      true if the parameter was converted from an old parameter value
    */
    bool convert_parameter_width(ap_var_type old_ptype, float scale_factor=1.0) {
        return _convert_parameter_width(old_ptype, scale_factor, false);
    }
    bool convert_centi_parameter(ap_var_type old_ptype) {
        return convert_parameter_width(old_ptype, 0.01f);
    }
    // Converting bitmasks should be done bitwise rather than numerically
    bool convert_bitmask_parameter_width(ap_var_type old_ptype) {
        return _convert_parameter_width(old_ptype, 1.0, true);
    }

    /**
     * @enum Conversion flags for convert_old_parameter()
     * @brief Control parameter conversion behavior during firmware upgrades
     */
    enum {
        CONVERT_FLAG_REVERSE=1, ///< Handle _REV suffix to _REVERSED suffix renaming
        CONVERT_FLAG_FORCE=2    ///< Store new value even if parameter already configured in storage
    };
    
    /**
     * @brief Convert single parameter with optional scaling
     * 
     * @param[in] info Conversion info descriptor (old key, new name, type)
     * @param[in] scaler Multiplicative scaling factor (e.g., 100.0 for centidegrees to degrees)
     * @param[in] flags Conversion behavior flags (CONVERT_FLAG_REVERSE, CONVERT_FLAG_FORCE)
     * 
     * @details Locates parameter using old storage key, applies scaling, and stores
     *          with new key. Preserves user configuration across firmware refactoring.
     * 
     * @note Called during boot by convert_old_parameters()
     */
    static void         convert_old_parameter(const struct ConversionInfo *info, float scaler, uint8_t flags=0);

    // move all parameters from a class to a new location
    // is_top_level: Is true if the class had its own top level key, param_key. It is false if the class was a subgroup
    static void         convert_class(uint16_t param_key, void *object_pointer,
                                        const struct AP_Param::GroupInfo *group_info,
                                        uint16_t old_index, bool is_top_level);

    /*
      fetch a parameter value based on the index within a group. This
      is used to find the old value of a parameter that has been
      removed from an object.
    */
    static bool get_param_by_index(void *obj_ptr, uint8_t idx, ap_var_type old_ptype, void *pvalue);
    
    /**
     * @brief Erase all parameters from persistent storage (factory reset)
     * 
     * @details Wipes entire parameter storage region and reinitializes with
     *          sentinel marker. All user configuration lost - parameters revert
     *          to compiled-in or defaults file values on next boot.
     * 
     * @warning DESTRUCTIVE - all user parameter configuration permanently erased
     * @warning Requires reboot to reload default values
     * @note Typically used for factory reset or recovering from corruption
     */
    static void         erase_all(void);

    /**
     * @brief Begin enumeration of all registered parameters
     * 
     * @param[in,out] token Opaque iterator state (zero-initialized before first call)
     * @param[out] ptype Parameter type (AP_PARAM_INT8/INT16/INT32/FLOAT/VECTOR3F/GROUP)
     * @param[out] default_val Optional default value for parameter
     * 
     * @return AP_Param* Pointer to first parameter, or nullptr if none exist
     * 
     * @note Token must be zero-initialized before first call
     * @note Parameters enumerated in storage key order (not alphabetical)
     * @see next() to continue enumeration
     */
    static AP_Param *      first(ParamToken *token, enum ap_var_type *ptype, float *default_val = nullptr);

    /**
     * @brief Continue enumeration of all registered parameters
     * 
     * @param[in,out] token Opaque iterator state from previous first() or next() call
     * @param[out] ptype Parameter type (AP_PARAM_INT8/INT16/INT32/FLOAT/VECTOR3F/GROUP)
     * @param[out] default_val Optional default value for parameter
     * 
     * @return AP_Param* Pointer to next parameter, or nullptr when enumeration complete
     * 
     * @details Enumerates parameters in storage key order (not alphabetical).
     *          Typical usage for parameter list download (MAVLink PARAM_REQUEST_LIST):
     *          ```cpp
     *          AP_Param::ParamToken token;
     *          AP_Param *param;
     *          enum ap_var_type ptype;
     *          while ((param = AP_Param::next(&token, &ptype)) != nullptr) {
     *              char name[AP_MAX_NAME_SIZE];
     *              param->copy_name_token(token, name, sizeof(name));
     *              send_parameter(name, param->cast_to_float(ptype));
     *          }
     *          ```
     * 
     * @note Groups (AP_PARAM_GROUP) enumerate as containers, then recurse into members
     * @note Vector3f types enumerate as single vector, then optionally as 3 scalars
     * @note Hidden and disabled parameters skipped based on flags
     */
    static AP_Param *      next(ParamToken *token, enum ap_var_type *ptype) { return  next(token, ptype, false); }
    static AP_Param *      next(ParamToken *token, enum ap_var_type *ptype, bool skip_disabled, float *default_val = nullptr);

    /**
     * @brief Get next scalar parameter, skipping groups during enumeration
     * 
     * @param[in,out] token Opaque iterator state from previous call
     * @param[out] ptype Parameter type (only scalar types: INT8/INT16/INT32/FLOAT)
     * @param[out] default_val Optional default value for parameter
     * 
     * @return AP_Param* Pointer to next scalar parameter, or nullptr when complete
     * 
     * @details Similar to next() but automatically skips AP_PARAM_GROUP entries,
     *          recursing into groups to find scalar leaf parameters. Useful for
     *          operations that only process scalar values (e.g., saving all parameters).
     * 
     * @note Will not return AP_PARAM_GROUP or AP_PARAM_VECTOR3F, only scalar types
     * @note Recursively explores nested groups to find scalars
     */
    static AP_Param *       next_scalar(ParamToken *token, enum ap_var_type *ptype, float *default_val = nullptr);

    /**
     * @brief Get storage size in bytes for a parameter type
     * 
     * @param[in] type Parameter type from ap_var_type enum
     * 
     * @return uint8_t Size in bytes (1, 2, 4, or 12)
     * 
     * @details Type sizes:
     *          - AP_PARAM_INT8: 1 byte
     *          - AP_PARAM_INT16: 2 bytes
     *          - AP_PARAM_INT32: 4 bytes
     *          - AP_PARAM_FLOAT: 4 bytes
     *          - AP_PARAM_VECTOR3F: 12 bytes (3 floats)
     *          - AP_PARAM_GROUP: 0 bytes (no payload, container only)
     * 
     * @note Used to calculate storage offsets and payload sizes
     */
    static uint8_t				type_size(enum ap_var_type type);

    /**
     * @brief Convert parameter value to float for transmission or display
     * 
     * @param[in] type Parameter type from ap_var_type enum
     * 
     * @return float Value converted to float representation
     * 
     * @details Type conversions:
     *          - INT8/INT16/INT32: Cast to float
     *          - FLOAT: Direct return
     *          - VECTOR3F: Returns magnitude (length) of vector
     *          - GROUP: Returns 0.0 (groups have no scalar value)
     * 
     * @note Used for MAVLink PARAM_VALUE messages (float only protocol)
     * @note Vector3f elements (_X, _Y, _Z) should be cast individually
     */
    float                   cast_to_float(enum ap_var_type type) const;

    /**
     * @brief Validate parameter table consistency at startup
     * 
     * @details Performs sanity checks on var_info[] tables:
     *          - Checks for duplicate keys in top-level table
     *          - Validates parameter name lengths (< AP_MAX_NAME_SIZE)
     *          - Checks for duplicate indices within groups
     *          - Validates nesting depth (max 3 levels)
     *          - Verifies no key exceeds 9-bit limit (0-511)
     * 
     * @note Called from setup() during initialization
     * @note Failures logged to console, may trigger AP_BoardConfig::config_error()
     * @warning Inconsistent tables cause unpredictable parameter behavior
     */
    static void             check_var_info(void);

    /**
     * @brief Check if parameter has been explicitly set (not default value)
     * 
     * @return bool true if parameter stored in EEPROM, false if using default
     * 
     * @details A parameter is "configured" if it has been saved to persistent
     *          storage, indicating user has explicitly set it. Parameters using
     *          compiled-in or defaults-file values are not configured.
     * 
     * @note Useful for detecting first-time setup or factory reset state
     */
    bool configured(void) const;

    /**
     * @brief Check if parameter is marked read-only
     * 
     * @return bool true if parameter cannot be modified via GCS/MAVLink
     * 
     * @details Read-only parameters:
     *          - Set via defaults file with @READONLY directive
     *          - Can be modified by firmware but not by ground station
     *          - Useful for locking safety-critical or factory-set parameters
     *          - Enforced in set_and_save() and MAVLink PARAM_SET handlers
     * 
     * @note Read-only status cleared on parameter erase/factory reset
     * @warning Overriding read-only via code bypasses safety restrictions
     */
    bool is_read_only(void) const;

    /**
     * @brief Get persistent storage key for a parameter token key
     * 
     * @param[in] key Token key from var_info index
     * 
     * @return uint16_t Persistent key used in storage headers
     * 
     * @note Maps dynamic or temporary keys to stable storage keys
     */
    static uint16_t get_persistent_key(uint16_t key) { return var_info(key).key; }

    /**
     * @brief Check if parameter can be modified via MAVLink/GCS
     * 
     * @param[in] flags Permission flags from AP_BoardConfig
     * 
     * @return bool true if parameter settable via MAVLink, false if restricted
     * 
     * @details Permission checks:
     *          - Read-only parameters: Always blocked
     *          - AP_PARAM_FLAG_INTERNAL_USE_ONLY: Blocked unless override flag set
     *          - AP_BoardConfig permissions: Board-specific restrictions
     *          - Safety checks: May block certain parameters in flight
     * 
     * @note Called by PARAM_SET handlers before applying parameter changes
     * @warning Bypassing this check can allow unsafe parameter modifications
     */
    bool allow_set_via_mavlink(uint16_t flags) const;

    /**
     * @brief Count total parameters visible to GCS (for PARAM_REQUEST_LIST response)
     * 
     * @return uint16_t Total parameter count including nested groups and vector elements
     * 
     * @details Cached count computed on first call and invalidated when:
     * - Parameter tables modified (scripting adds dynamic tables)
     * - Frame type flags changed (unhides vehicle-specific parameters)
     * - Enable parameters toggled (shows/hides subtrees)
     * 
     * @note Expensive operation (full tree traversal) - result cached after first call
     * @note Count includes individual vector elements (e.g., COMPASS_OFS_X/Y/Z count as 3)
     */
    static uint16_t count_parameters(void);

    /**
     * @brief Invalidate cached parameter count
     * 
     * @details Forces count_parameters() to recompute on next call.
     *          Called when parameter visibility changes due to:
     *          - Dynamic parameter tables added (scripting)
     *          - Frame type flags changed
     *          - Enable parameters toggled
     * 
     * @note Automatically called by set_frame_type_flags()
     */
    static void invalidate_count(void);

    /**
     * @brief Control visibility of disabled parameter groups
     * 
     * @param[in] value true to hide disabled groups, false to show all
     * 
     * @details When enabled, parameters with AP_PARAM_FLAG_ENABLE that are
     *          set to 0 will hide their entire subtree from enumeration.
     *          Useful for reducing parameter list clutter in GCS.
     * 
     * @note Affects count_parameters() and first()/next() enumeration
     */
    static void set_hide_disabled_groups(bool value) { _hide_disabled_groups = value; }

    /**
     * @brief Enable vehicle/frame-specific parameters
     * 
     * @param[in] flags_to_set Frame type flags (AP_PARAM_FRAME_* bitmask)
     * 
     * @details Unhides parameters marked with frame-specific flags matching
     *          flags_to_set. Called during vehicle initialization to expose
     *          only parameters relevant to current vehicle type.
     *          
     *          Frame flags:
     *          - AP_PARAM_FRAME_COPTER: Multicopter parameters
     *          - AP_PARAM_FRAME_PLANE: Fixed-wing parameters
     *          - AP_PARAM_FRAME_ROVER: Ground vehicle parameters
     *          - AP_PARAM_FRAME_SUB: Underwater vehicle parameters
     * 
     * @note Flags are OR'd together (cumulative, not replaced)
     * @note Automatically invalidates parameter count cache
     */
    static void set_frame_type_flags(uint16_t flags_to_set) {
        invalidate_count();
        _frame_type_flags |= flags_to_set;
    }

    /**
     * @brief Check if parameter's frame flags match current vehicle type
     * 
     * @param[in] flags Parameter's frame type flags from GroupInfo
     * 
     * @return bool true if parameter visible for current frame type
     * 
     * @note Called during enumeration to filter frame-specific parameters
     */
    static bool check_frame_type(uint16_t flags);

#if AP_PARAM_KEY_DUMP
    /**
     * @brief Print all parameters to serial port (debugging utility)
     * 
     * @param[in] port Output stream for parameter listing
     * @param[in] showKeyValues true to show internal keys, false for names only
     * 
     * @details Enumerates all parameters and prints name=value pairs.
     *          When showKeyValues=true, also displays storage keys for debugging.
     *          
     *          Output format:
     *          - Standard: PARAM_NAME=value
     *          - With keys: PARAM_NAME=value [key=123, group_element=456]
     * 
     * @note Only available when AP_PARAM_KEY_DUMP enabled (debug builds)
     * @note Can be slow on large parameter sets - avoid in flight-critical code
     */
    static void         show_all(AP_HAL::BetterStream *port, bool showKeyValues=false);

    /**
     * @brief Print single parameter value to serial port
     * 
     * @param[in] param Pointer to parameter to display
     * @param[in] name Parameter name string
     * @param[in] ptype Parameter type (for correct formatting)
     * @param[in] port Output stream
     * 
     * @note Only available when AP_PARAM_KEY_DUMP enabled (debug builds)
     */
    static void         show(const AP_Param *param, 
                             const char *name,
                             enum ap_var_type ptype, 
                             AP_HAL::BetterStream *port);

    /**
     * @brief Print single parameter value using token for name lookup
     * 
     * @param[in] param Pointer to parameter to display
     * @param[in] token ParamToken for name resolution
     * @param[in] ptype Parameter type (for correct formatting)
     * @param[in] port Output stream
     * 
     * @note Only available when AP_PARAM_KEY_DUMP enabled (debug builds)
     */
    static void         show(const AP_Param *param, 
                             const ParamToken &token,
                             enum ap_var_type ptype, 
                             AP_HAL::BetterStream *port);
#endif // AP_PARAM_KEY_DUMP

    /**
     * @brief Get AP_Param singleton instance
     * 
     * @return AP_Param* Pointer to the singleton instance
     * 
     * @note Used internally and by unit tests
     */
    static AP_Param *get_singleton() { return _singleton; }

#if AP_PARAM_DYNAMIC_ENABLED
    /**
     * @brief Add dynamic parameter table at runtime (scripting support)
     * 
     * @param[in] key Top-level parameter key (must be unallocated)
     * @param[in] prefix Parameter name prefix for this table
     * @param[in] num_params Number of parameters to allocate in table
     * 
     * @return bool true if table added successfully, false on error
     * 
     * @details Allows Lua scripts to create custom parameter tables dynamically.
     *          Allocates space for num_params float parameters under prefix.
     *          Example: add_table(100, "SCR", 10) creates SCR_PARAM0..SCR_PARAM9.
     * 
     * @note Only available when AP_PARAM_DYNAMIC_ENABLED (Lua scripting enabled)
     * @note Limited to AP_PARAM_MAX_DYNAMIC tables
     * @warning Keys must not conflict with static parameter tables
     */
    static bool add_table(uint8_t key, const char *prefix, uint8_t num_params);
    
    /**
     * @brief Add single parameter to dynamic table
     * 
     * @param[in] key Table key from add_table()
     * @param[in] param_num Parameter index within table (0..num_params-1)
     * @param[in] pname Parameter name suffix
     * @param[in] default_value Default float value
     * 
     * @return bool true if parameter added successfully, false on error
     * 
     * @details Sets name and default for dynamic parameter created by add_table().
     *          Full parameter name will be PREFIX_pname.
     * 
     * @note Only available when AP_PARAM_DYNAMIC_ENABLED
     */
    static bool add_param(uint8_t key, uint8_t param_num, const char *pname, float default_value);
    
    /**
     * @brief Load int32 parameter value by key (scripting helper)
     * 
     * @param[in] key Parameter key
     * @param[in] group_element Group element path
     * @param[out] value Loaded parameter value
     * 
     * @return bool true if parameter found and loaded, false otherwise
     * 
     * @note Only available when AP_PARAM_DYNAMIC_ENABLED
     */
    static bool load_int32(uint16_t key, uint32_t group_element, int32_t &value);
#endif

    /**
     * @brief Load parameter defaults from external text file
     * 
     * @param[in] filename Path to defaults file (e.g., "@ROMFS/defaults.parm")
     * @param[in] last_pass true for final pass (apply overrides), false for defaults only
     * 
     * @return bool true if file loaded successfully, false on error
     * 
     * @details Parses text defaults file containing param=value pairs.
     *          File format supports:
     *          - Comments: Lines starting with #
     *          - Parameters: PARAM_NAME,value
     *          - Metadata: @READONLY, @FORCED, @ENABLE, @DEFAULT
     *          - Conditionals: @include, @define
     *          
     *          Two-pass loading:
     *          - First pass (last_pass=false): Sets default values
     *          - Second pass (last_pass=true): Applies forced overrides
     * 
     * @note Requires AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED
     * @note Supports @ROMFS paths and filesystem paths
     * @see AP_Filesystem for path resolution
     */
    static bool load_defaults_file(const char *filename, bool last_pass);

protected:

    /**
     * @brief Store default value in linked list for parameter
     * 
     * @param[in] ap Pointer to parameter object
     * @param[in] v Default value to store
     * 
     * @details Adds parameter default value to internal linked list used for:
     *          - Reporting defaults to GCS via first()/next() enumeration
     *          - Determining if parameter differs from default (for save optimization)
     *          - Defaults file processing and override resolution
     * 
     * @note Called internally during defaults file loading and parameter initialization
     * @note Multiple defaults for same parameter: last one wins
     */
    static void add_default(AP_Param *ap, float v);

private:
    static AP_Param *_singleton;

    /**
     * @struct EEPROM_header
     * @brief Storage header at offset 0 identifying AP_Param formatted storage
     * 
     * @details Placed at beginning of parameter storage (EEPROM/Flash/FRAM) to:
     *          - Identify storage as AP_Param formatted (via magic bytes)
     *          - Track storage format revision for migration compatibility
     *          - Validate storage integrity on boot
     * 
     * Header fields:
     * - magic[2]: Magic bytes [0xAE, 0x47] identifying AP_Param storage
     * - revision: Storage format version (currently 6)
     * - spare: Reserved byte for future use (currently 0)
     * 
     * Validation at boot:
     * - If magic bytes incorrect, storage reformatted (all parameters lost)
     * - If revision mismatches, storage reformatted or migrated
     * - Valid header: parameter loading proceeds
     * 
     * @note Storage reformat erases all user configuration
     * @note Header size fixed at 4 bytes for storage layout stability
     * @warning Changing header size breaks storage format compatibility
     */
    struct EEPROM_header {
        uint8_t magic[2];   ///< Magic bytes [0xAE, 0x47] identifying AP_Param storage
        uint8_t revision;   ///< Storage format revision (currently 6)
        uint8_t spare;      ///< Reserved for future use
    };
    static_assert(sizeof(struct EEPROM_header) == 4, "Bad EEPROM_header size!");

    static uint16_t sentinal_offset;

    /**
     * @struct Param_header
     * @brief Binary storage header preceding each parameter value in EEPROM/Flash
     * 
     * @details 32-bit packed bitfield structure encoding parameter identity and type.
     *          Follows EEPROM_header in storage, repeated for each stored parameter.
     *          Each Param_header is immediately followed by parameter value payload.
     * 
     * Bitfield layout (32 bits total):
     * - key_low (8 bits): Lower 8 bits of 9-bit parameter key (bits 0-7)
     * - type (5 bits): Parameter type from ap_var_type enum (AP_PARAM_INT8, etc.)
     * - key_high (1 bit): Upper bit of 9-bit parameter key (bit 8)
     * - group_element (18 bits): Nested group path encoding (3 levels × 6 bits)
     * 
     * Key encoding (9 bits total, 0-511):
     * - Identifies top-level parameter group from var_info[] table
     * - Split across key_low and key_high for binary compatibility
     * - Full key value: (key_high << 8) | key_low
     * - Sentinel key 0x1FF marks end of stored parameters
     * 
     * Type field (5 bits, 0-31):
     * - Determines value payload size following header
     * - INT8/INT16/INT32/FLOAT: 1/2/4/4 bytes
     * - VECTOR3F: 12 bytes (3 floats)
     * - GROUP: No payload, contains nested parameters
     * 
     * Group_element encoding (18 bits):
     * - Encodes path through up to 3 levels of nested groups
     * - Each level: 6 bits (0-63 index within parent group)
     * - Level 1: bits 0-5, Level 2: bits 6-11, Level 3: bits 12-17
     * - Example: 0x040201 = Level1[1], Level2[2], Level3[4]
     * - Value 0 indicates top-level parameter (no nesting)
     * 
     * Storage format example:
     * ```
     * Offset  Content
     * ------  -------
     * 0x0000  EEPROM_header (4 bytes)
     * 0x0004  Param_header (key=5, type=FLOAT, group_element=0)
     * 0x0008  Float value (4 bytes)
     * 0x000C  Param_header (key=10, type=INT16, group_element=0x41)
     * 0x0010  Int16 value (2 bytes)
     * 0x0012  Param_header (key=0x1FF, sentinel)
     * ```
     * 
     * @note Binary layout must remain stable for storage compatibility
     * @note Sentinel header (key=0x1FF) marks end of parameters, no payload follows
     * @warning Modifying bitfield layout breaks storage format across firmware versions
     * 
     * @see ap_var_type for type field values
     * @see EEPROM_header for storage initialization
     */
    struct Param_header {
        // to get 9 bits for key we needed to split it into two parts to keep binary compatibility
        uint32_t key_low : 8;          ///< Lower 8 bits of 9-bit parameter key (0-255)
        uint32_t type : 5;             ///< Parameter type from ap_var_type enum (0-31)
        uint32_t key_high : 1;         ///< Upper bit of 9-bit parameter key (bit 8)
        uint32_t group_element : 18;   ///< Nested group path (3 levels × 6 bits, 0-262143)
    };
    static_assert(sizeof(struct Param_header) == 4, "Bad Param_header size!");

    // number of bits in each level of nesting of groups
    static const uint8_t        _group_level_shift = 6;
    static const uint8_t        _group_bits  = 18;

    static const uint16_t       _sentinal_key   = 0x1FF;
    static const uint8_t        _sentinal_type  = 0x1F;
    static const uint8_t        _sentinal_group = 0xFF;

    static uint16_t             _frame_type_flags;

    /*
      this is true if when scanning a defaults file we find all of the parameters
     */
    static bool done_all_default_params;

    /*
      structure for built-in defaults file that can be modified using apj_tool.py
     */
#if AP_PARAM_MAX_EMBEDDED_PARAM > 0
    struct PACKED param_defaults_struct {
        char magic_str[8];
        uint8_t param_magic[8];
        uint16_t max_length;
        volatile uint16_t length;
        volatile char data[AP_PARAM_MAX_EMBEDDED_PARAM];
    };
    static const param_defaults_struct param_defaults_data;
#endif


    static void                 check_group_info(const struct GroupInfo *group_info, uint16_t *total_size, 
                                                 uint8_t max_bits, uint8_t prefix_length);
    static bool                 duplicate_key(uint16_t vindex, uint16_t key);

    static bool adjust_group_offset(uint16_t vindex, const struct GroupInfo &group_info, ptrdiff_t &new_offset);
    static bool get_base(const struct Info &info, ptrdiff_t &base);

    /**
     * @brief Get group_info pointer based on flags (internal helper)
     * 
     * @param[in] ginfo GroupInfo structure
     * 
     * @return const struct GroupInfo* Pointer to group_info, following indirection if FLAG_INFO_POINTER set
     * 
     * @note Handles FLAG_INFO_POINTER indirection for dynamic parameter tables
     */
    static const struct GroupInfo *get_group_info(const struct GroupInfo &ginfo);

    /**
     * @brief Get group_info pointer based on flags (internal helper)
     * 
     * @param[in] ginfo Info structure (top-level)
     * 
     * @return const struct GroupInfo* Pointer to group_info, following indirection if FLAG_INFO_POINTER set
     * 
     * @note Overload for top-level Info structures
     */
    static const struct GroupInfo *get_group_info(const struct Info &ginfo);

    const struct Info *         find_var_info_group(
                                    const struct GroupInfo *    group_info,
                                    uint16_t                    vindex,
                                    uint32_t                    group_base,
                                    uint8_t                     group_shift,
                                    ptrdiff_t                   group_offset,
                                    uint32_t *                  group_element,
                                    const struct GroupInfo *   &group_ret,
                                    struct GroupNesting        &group_nesting,
                                    uint8_t *                   idx) const;
    const struct Info *         find_var_info(
                                    uint32_t *                group_element,
                                    const struct GroupInfo *  &group_ret,
                                    struct GroupNesting       &group_nesting,
                                    uint8_t *                 idx) const;
    const struct Info *			find_var_info_token(const ParamToken &token,
                                                    uint32_t *                 group_element,
                                                    const struct GroupInfo *  &group_ret,
                                                    struct GroupNesting       &group_nesting,
                                                    uint8_t *                  idx) const;
    static const struct Info *  find_by_header_group(
                                    struct Param_header phdr, void **ptr,
                                    uint16_t vindex,
                                    const struct GroupInfo *group_info,
                                    uint32_t group_base,
                                    uint8_t group_shift,
                                    ptrdiff_t group_offset);
    static const struct Info *  find_by_header(
                                    struct Param_header phdr,
                                    void **ptr);
    void                        add_vector3f_suffix(
                                    char *buffer,
                                    size_t buffer_size,
                                    uint8_t idx) const;
    static AP_Param *           find_group(
                                    const char *name,
                                    uint16_t vindex,
                                    ptrdiff_t group_offset,
                                    const struct GroupInfo *group_info,
                                    enum ap_var_type *ptype);
    static void                 write_sentinal(uint16_t ofs);
    static uint16_t             get_key(const Param_header &phdr);
    static void                 set_key(Param_header &phdr, uint16_t key);
    static bool                 is_sentinal(const Param_header &phrd);
    static bool                 scan(
                                    const struct Param_header *phdr,
                                    uint16_t *pofs);
    static void                 eeprom_write_check(
                                    const void *ptr,
                                    uint16_t ofs,
                                    uint8_t size);
    static AP_Param *           next_group(
                                    const uint16_t vindex,
                                    const struct GroupInfo *group_info,
                                    bool *found_current,
                                    const uint32_t group_base,
                                    const uint8_t group_shift,
                                    const ptrdiff_t group_offset,
                                    ParamToken *token,
                                    enum ap_var_type *ptype,
                                    bool skip_disabled,
                                    float *default_val);

    // find a default value given a pointer to a default value in flash
    static float get_default_value(const AP_Param *object_ptr, const struct GroupInfo &info);
    static float get_default_value(const AP_Param *object_ptr, const struct Info &info);

    static bool parse_param_line(char *line, char **vname, float &value, bool &read_only);

    /*
      load a parameter defaults file. This happens as part of load_all()
     */
    static bool count_defaults_in_file(const char *filename, uint16_t &num_defaults);
    static bool count_param_defaults(const volatile char *ptr, int32_t length, uint16_t &count);
    static bool read_param_defaults_file(const char *filename, bool last_pass, uint16_t &idx);

    // load a defaults.parm using AP_FileSystem:
    static void load_defaults_file_from_filesystem(const char *filename, bool lastpass);
    // load an @ROMFS defaults.parm using ROMFS API:
    static void load_defaults_file_from_romfs(const char *filename, bool lastpass);

    // load defaults from supplied string:
    static void load_param_defaults(const volatile char *ptr, int32_t length, bool last_pass);

    /*
      load defaults from embedded parameters
     */
    static bool count_embedded_param_defaults(uint16_t &count);
    static void load_embedded_param_defaults(bool last_pass);

    // return true if the parameter is configured in the defaults file
    bool configured_in_defaults_file(bool &read_only) const;

    // return true if the parameter is configured in EEPROM/FRAM
    bool configured_in_storage(void) const;

    /*
      convert width of a parameter, allowing update to wider scalar
      values without changing the parameter indexes. This will return
      true if the parameter was converted from an old parameter value
    */
    bool _convert_parameter_width(ap_var_type old_ptype, float scale_factor, bool bitmask);

    // send a parameter to all GCS instances
    void send_parameter(const char *name, enum ap_var_type param_header_type, uint8_t idx) const;

    static StorageAccess        _storage;
    static StorageAccess        _storage_bak;
    static uint16_t             _num_vars;
    static uint16_t             _parameter_count;
    static uint16_t             _count_marker;
    static uint16_t             _count_marker_done;
    static HAL_Semaphore        _count_sem;
    static const struct Info *  _var_info;

#if AP_PARAM_DYNAMIC_ENABLED
    // allow for a dynamically allocated var table
    static uint16_t             _num_vars_base;
    static struct Info *        _var_info_dynamic;
    static const struct AP_Param::Info &var_info(uint16_t i) {
        return i<_num_vars_base? _var_info[i] : _var_info_dynamic[i-_num_vars_base];
    }
    static uint8_t _dynamic_table_sizes[AP_PARAM_MAX_DYNAMIC];
#else
    // simple static var table in flash
    static const struct Info &var_info(uint16_t i) {
        return _var_info[i];
    }
#endif

    /*
      list of overridden values from load_defaults_file()
    */
    struct param_override {
        const AP_Param *object_ptr;
        float value;
        bool read_only; // param is marked @READONLY
    };
    static struct param_override *param_overrides;
    static uint16_t num_param_overrides;
    static uint16_t param_overrides_len;
    static uint16_t num_read_only;

    // values filled into the EEPROM header
    static const uint8_t        k_EEPROM_magic0      = 0x50;
    static const uint8_t        k_EEPROM_magic1      = 0x41; ///< "AP"
    static const uint8_t        k_EEPROM_revision    = 6; ///< current format revision

    static bool _hide_disabled_groups;

    // support for background saving of parameters. We pack it to reduce memory for the
    // queue
    struct PACKED param_save {
        AP_Param *param;
        bool force_save;
    };
    static ObjectBuffer_TS<struct param_save> save_queue;
    static bool registered_save_handler;

    // background function for saving parameters
    void save_io_handler(void);

    // Store default values from add_default() calls in linked list
    struct defaults_list {
        AP_Param *ap;
        float val;
        defaults_list *next;
    };
    static defaults_list *default_list;
    static void check_default(AP_Param *ap, float *default_value);

    static bool eeprom_full;
};

namespace AP {
    AP_Param *param();
};

/**
 * @class AP_ParamTBase
 * @brief Base class for typed parameter wrappers providing type-safe access
 * 
 * @tparam T Parameter storage type (int8_t, int16_t, int32_t, float)
 * @tparam PT AP_PARAM_* type enum value
 * 
 * @details Provides foundational typed parameter operations:
 *          - Type-safe get/set methods
 *          - Default value management
 *          - Combined set-and-save operations
 *          - GCS notification integration
 *          - Enable parameter support
 *          
 *          Not used directly - see AP_ParamT and specializations below.
 * 
 * @note Lightweight wrapper - size exactly matches storage type T
 */
template<typename T, ap_var_type PT>
class AP_ParamTBase : public AP_Param
{
public:
    static const ap_var_type        vtype = PT;

    /**
     * @brief Get current parameter value
     * 
     * @return const T& Reference to parameter value
     * 
     * @note Returns reference for efficiency - value not copied
     */
    const T &get(void) const {
        return _value;
    }

    /**
     * @brief Set parameter value (RAM only, not persistent)
     * 
     * @param[in] v New value to assign
     * 
     * @note Changes RAM only - lost on reboot unless saved
     * @note Does not notify GCS of change
     * @note Does not trigger parameter change callbacks
     */
    void set(const T &v) {
        _value = v;
    }

    /**
     * @brief Set ENABLE parameter and invalidate count if disabling subtree
     * 
     * @param[in] v New value (typically 0=disabled, 1=enabled)
     * 
     * @details For parameters with AP_PARAM_FLAG_ENABLE, sets value and
     *          invalidates parameter count cache if disabling (v==0) to
     *          hide associated parameter subtree from enumeration.
     * 
     * @note Only use for parameters declared with AP_PARAM_FLAG_ENABLE
     */
    void set_enable(const T &v);
    
    /**
     * @brief Set parameter to default if unconfigured
     * 
     * @param[in] v Default value to apply
     * 
     * @details Sets value only if parameter not configured (not in storage,
     *          not in defaults file). Used during initialization to establish
     *          compiled-in defaults without overriding user configuration.
     * 
     * @note No effect if parameter already configured or stored
     */
    void set_default(const T &v);

    /**
     * @brief Set parameter value and register as default
     * 
     * @param[in] v Value to set as both current and default
     * 
     * @details Sets current value and registers it as the default value
     *          for comparison and reset purposes. Used when programmatically
     *          establishing runtime-computed defaults.
     * 
     * @note Affects configured() and comparison with defaults
     */
    void set_and_default(const T &v);

    /**
     * @brief Set parameter value and notify GCS of change
     * 
     * @param[in] v New value to assign
     * 
     * @details Sets value in RAM and sends PARAM_VALUE message to all
     *          connected ground control stations. Does not save to storage.
     * 
     * @note Use for runtime changes that should be visible to GCS immediately
     * @note Does not persist - lost on reboot unless also saved
     */
    void set_and_notify(const T &v);

    /**
     * @brief Set parameter value and queue persistent save
     * 
     * @param[in] v New value to assign and save
     * 
     * @details Sets value in RAM and queues asynchronous save to storage.
     *          Save processed by I/O thread within 50-200ms.
     * 
     * @note Non-blocking - returns immediately
     * @note Use when parameter change should persist across reboots
     * @note Automatically notifies GCS of change
     */
    void set_and_save(const T &v);

    /**
     * @brief Set parameter and save only if value changed
     * 
     * @param[in] v New value to potentially save
     * 
     * @details Optimized version of set_and_save() that skips storage write
     *          if new value equals current RAM value. Reduces flash wear and
     *          I/O thread load for repeated sets with same value.
     * 
     * @warning Only use when value hasn't been set() separately first,
     *          otherwise EEPROM won't update correctly
     * @note Compares against RAM value, not stored value
     */
    void set_and_save_ifchanged(const T &v);

    /**
     * @brief Convert parameter value to float for generic processing
     * 
     * @return float Parameter value as floating-point
     * 
     * @details Enables generic parameter handling in MAVLink protocol and
     *          parameter enumeration. Integer types promoted to float.
     * 
     * @note Required by AP_Param base class interface
     */
    float cast_to_float(void) const;

protected:
    T _value;
};

/**
 * @class AP_ParamT
 * @brief Typed parameter wrapper for compile-time type safety
 * 
 * @tparam T Parameter storage type (int8_t, int16_t, int32_t, float)
 * @tparam PT AP_PARAM_* type enum value
 * 
 * @details Provides type-safe parameter access with implicit conversion operators.
 *          Typical usage in vehicle class:
 *          ```cpp
 *          class MyClass {
 *              AP_Int16 _param1;  // typedef of AP_ParamT<int16_t, AP_PARAM_INT16>
 *              AP_Float _param2;  // typedef of AP_ParamT<float, AP_PARAM_FLOAT>
 *              static const AP_Param::GroupInfo var_info[];
 *          };
 *          ```
 *          
 *          Type specializations:
 *          - AP_Int8, AP_Int16, AP_Int32: Integer parameters
 *          - AP_Float: Floating-point parameters
 *          - AP_Vector3f: 3D vector parameters (via AP_ParamV specialization)
 * 
 * @note Lightweight wrapper - size exactly matches storage type T
 * @note Implicitly convertible to T for reading, assign T for writing
 */
template<typename T, ap_var_type PT>
class AP_ParamT : public AP_ParamTBase<T, PT> // for int and smaller types
{
public:
    /**
     * @brief Implicit conversion operator to enable parameter usage as scalar value
     * 
     * @return const T& Reference to parameter value
     * 
     * @details Allows parameter to be used directly in expressions:
     *          ```cpp
     *          AP_Int16 speed_param;
     *          int16_t speed = speed_param;  // Implicit conversion
     *          if (speed_param > 100) { }    // Direct comparison
     *          ```
     * 
     * @note Returns reference for compatibility with code expecting references
     * @warning Implicit conversion can cause truncation in ternary expressions
     *          with mixed types. Use .get() explicitly if type precision critical.
     */
    operator const T &() const {
        return this->_value;
    }
};

/**
 * @class AP_ParamT<float, AP_PARAM_FLOAT>
 * @brief Specialized float parameter with templated conversion operator
 * 
 * @details Float specialization provides templated conversion operator to prevent
 *          unintended implicit conversions that could cause truncation. The template
 *          parameter forces exact match requirement per C++ [over.ics.user] clause 3.
 */
template<>
class AP_ParamT<float, AP_PARAM_FLOAT> : public AP_ParamTBase<float, AP_PARAM_FLOAT>
{
public:
    /**
     * @brief Templated conversion operator to float reference
     * 
     * @tparam X Template parameter to enforce exact type match (prevents implicit conversions)
     * 
     * @return const float& Reference to parameter value
     * 
     * @details Returns reference (not value) for compatibility with code expecting references.
     *          Templated to forbid further implicit conversions per [over.ics.user] clause 3.
     *          This prevents silent truncation in expressions like:
     *          `float v = true ? float_param : 0;`
     *          
     *          Trade-off: Also prevents implicit conversion to double. Use explicit cast
     *          or .get() method when double precision needed:
     *          - Explicit cast: `double d = (double)float_param;`
     *          - get() method: `double d = float_param.get();`
     * 
     * @note Template parameter X defaults to true and is not used - exists only for type safety
     * @warning Prevents implicit conversion to int - could cause compilation errors in generic code
     */
    template<bool X = true>
    operator const float &() const {
        return this->_value;
    }

    /**
     * @brief Explicit conversion to int
     * 
     * @return int Parameter value truncated to integer
     * 
     * @note Explicit cast required: `int x = (int)float_param;`
     * @warning Truncates fractional part without rounding
     */
    explicit operator int () const {
        return (int)this->_value;
    }

    /**
     * @brief Explicit conversion to double
     * 
     * @return double Parameter value converted to double precision
     * 
     * @note Explicit cast required: `double x = (double)float_param;`
     */
    explicit operator double () const {
        return (double)this->_value;
    }

#if defined(__clang__)
    // inexplicably, clang will not use the built-in operator implementations
    // for floats on two AP_ParamT<float>s, so provide them for it.

    float operator -() const { return -this->_value; } // unary minus

#define PARAM_SELF_OPER(R, OP) \
    R operator OP (const AP_ParamT<float, AP_PARAM_FLOAT>& other) const { return this->_value OP other._value; }

    PARAM_SELF_OPER(float, +);
    PARAM_SELF_OPER(float, -);
    PARAM_SELF_OPER(float, *);
    PARAM_SELF_OPER(float, /);
    PARAM_SELF_OPER(bool, >);
    PARAM_SELF_OPER(bool, <);
    PARAM_SELF_OPER(bool, <=);
    PARAM_SELF_OPER(bool, >=);
    // != and == are unsafe on floats

#undef PARAM_SELF_OPER

#endif
};

/**
 * @class AP_ParamV
 * @brief Templated parameter class for vector/composite types
 * 
 * @tparam T Parameter storage type (typically Vector3f)
 * @tparam PT AP_PARAM_* type enum value
 * 
 * @details Provides parameter wrapper for non-scalar types (vectors, structures).
 *          Unlike AP_ParamT for scalars, AP_ParamV handles composite types that:
 *          - Store multiple values as single parameter unit
 *          - Can be accessed both as whole object and individual elements
 *          - Require special storage and enumeration handling
 * 
 * Primary usage: AP_Vector3f (Vector3<float>) for 3D vectors like compass offsets.
 * 
 * Example usage:
 * ```cpp
 * class Compass {
 *     AP_Vector3f _offset;  // typedef of AP_ParamV<Vector3f, AP_PARAM_VECTOR3F>
 *     static const AP_Param::GroupInfo var_info[];
 * };
 * 
 * // Access as vector
 * Vector3f offset = compass._offset;
 * 
 * // Individual elements exposed as _X, _Y, _Z parameters to GCS
 * ```
 * 
 * @note Vector parameters store 12 bytes (3 floats) in single storage entry
 * @note Enumeration exposes both vector and individual scalar elements
 */
template<typename T, ap_var_type PT>
class AP_ParamV : public AP_Param
{
public:

    static const ap_var_type        vtype = PT;

    /**
     * @brief Get current parameter value
     * 
     * @return const T& Reference to parameter value
     * 
     * @note Returns reference - value not copied for efficiency
     */
    const T &get(void) const {
        return _value;
    }

    /**
     * @brief Set parameter value (RAM only, not persistent)
     * 
     * @param[in] v New value to assign
     * 
     * @note Changes RAM only - lost on reboot unless saved
     * @note Does not notify GCS of change
     */
    void set(const T &v) {
        _value = v;
    }

    /**
     * @brief Set parameter value and notify GCS of change
     * 
     * @param[in] v New value to assign
     * 
     * @details Sets value in RAM and sends PARAM_VALUE message to all
     *          connected ground control stations. Does not save to storage.
     * 
     * @note Use for runtime changes that should be visible to GCS immediately
     * @note Does not persist - lost on reboot unless also saved
     */
    void set_and_notify(const T &v);

    /**
     * @brief Set parameter value and queue persistent save
     * 
     * @param[in] v New value to assign and save
     * 
     * @details Sets value in RAM and queues asynchronous save to storage.
     *          Save processed by I/O thread within 50-200ms.
     * 
     * @note Non-blocking - returns immediately
     * @note Use when parameter change should persist across reboots
     * @note Automatically notifies GCS of change
     */
    void set_and_save(const T &v);

    /**
     * @brief Set parameter and save only if value changed
     * 
     * @param[in] v New value to potentially save
     * 
     * @details Optimized version of set_and_save() that skips storage write
     *          if new value equals current RAM value. Reduces flash wear and
     *          I/O thread load for repeated sets with same value.
     * 
     * @warning Only use when value hasn't been set() separately first,
     *          otherwise EEPROM won't update correctly
     * @note Compares against RAM value, not stored value
     */
    void set_and_save_ifchanged(const T &v);


    /**
     * @brief Implicit conversion to parameter value type
     * 
     * @return const T& Reference to parameter value
     * 
     * @details Allows AP_ParamV objects to be used directly where T is expected:
     *          ```cpp
     *          AP_Vector3f offset_param;
     *          Vector3f offset = offset_param;  // Implicit conversion
     *          float x = offset_param.x;        // Direct member access
     *          ```
     * 
     * @note As T is user-defined type, no numeric conversion issues
     * @note Returns reference for efficiency and compatibility
     */
    operator const T &() const {
        return _value;
    }

protected:
    T        _value;
};


/**
 * @def AP_PARAMDEF(_t, _suffix, _pt)
 * @brief Convenience macro for defining scalar parameter type instances
 * 
 * @param _t Base storage type (int8_t, int16_t, int32_t, float)
 * @param _suffix Type name suffix for AP_* typename
 * @param _pt Corresponding ap_var_type enum value
 * 
 * @details Creates typedef for AP_ParamT template instantiation.
 *          Enables concise parameter type declarations in vehicle code.
 * 
 * Example expansion: AP_PARAMDEF(float, Float, AP_PARAM_FLOAT)
 * expands to: typedef AP_ParamT<float, AP_PARAM_FLOAT> AP_Float;
 */
#define AP_PARAMDEF(_t, _suffix, _pt)   typedef AP_ParamT<_t, _pt> AP_ ## _suffix;
AP_PARAMDEF(float, Float, AP_PARAM_FLOAT);    // defines AP_Float, requires specialization!
AP_PARAMDEF(int8_t, Int8, AP_PARAM_INT8);     // defines AP_Int8
AP_PARAMDEF(int16_t, Int16, AP_PARAM_INT16);  // defines AP_Int16
AP_PARAMDEF(int32_t, Int32, AP_PARAM_INT32);  // defines AP_Int32

/**
 * @def AP_PARAMDEFV(_t, _suffix, _pt)
 * @brief Convenience macro for defining vector/composite parameter type instances
 * 
 * @param _t Base composite type (typically Vector3f)
 * @param _suffix Type name suffix for AP_* typename
 * @param _pt Corresponding ap_var_type enum value
 * 
 * @details Creates typedef for AP_ParamV template instantiation.
 *          Used in AP_Math.h to define AP_Vector3f.
 * 
 * Example expansion: AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F)
 * expands to: typedef AP_ParamV<Vector3f, AP_PARAM_VECTOR3F> AP_Vector3f;
 */
#define AP_PARAMDEFV(_t, _suffix, _pt)   typedef AP_ParamV<_t, _pt> AP_ ## _suffix;

// Compile-time safety check: prevent implicit AP_Float to int conversion
// (see AP_ParamT<float> specialization for explicit operator int() = delete)
static_assert(not std::is_convertible<AP_Float, int>::value, "illegal conversion possible");

/**
 * @class AP_Enum
 * @brief Template class for 8-bit enumeration parameters
 * 
 * @tparam eclass Enum class type for type-safe enumeration values
 * 
 * @details Provides type-safe enum parameter wrapper based on AP_Int8.
 *          Enables compile-time enum type checking while storing as int8_t.
 * 
 * Typical usage:
 * ```cpp
 * enum class MyEnum : uint8_t {
 *     OPTION_A = 0,
 *     OPTION_B = 1,
 *     OPTION_C = 2
 * };
 * 
 * class MyClass {
 *     AP_Enum<MyEnum> _mode;
 *     static const AP_Param::GroupInfo var_info[];
 * };
 * 
 * // Type-safe access
 * if (my_object._mode == MyEnum::OPTION_A) { ... }
 * my_object._mode.set(MyEnum::OPTION_B);
 * ```
 * 
 * @note Inherits all AP_Int8 functionality (load, save, notify, etc.)
 * @note Enum values must fit in int8_t range (-128 to 127 or 0 to 255)
 */
template<typename eclass>
class AP_Enum : public AP_Int8
{
public:
    /**
     * @brief Implicit conversion to enum type
     * 
     * @return eclass Enum value
     * 
     * @note Enables direct use in comparisons and assignments
     */
    operator const eclass () const {
        return (eclass)_value;
    }
    
    /**
     * @brief Set parameter to enum value (RAM only)
     * 
     * @param[in] v Enum value to assign
     * 
     * @note Does not persist - lost on reboot unless saved
     */
    void set(eclass v) {
        AP_Int8::set(int8_t(v));
    }
    
    /**
     * @brief Set parameter to enum value and queue persistent save
     * 
     * @param[in] v Enum value to assign and save
     * 
     * @note Non-blocking - save queued to I/O thread
     */
    void set_and_save(eclass v) {
        AP_Int8::set_and_save(int8_t(v));
    }
};

/**
 * @class AP_Enum16
 * @brief Template class for 16-bit enumeration parameters
 * 
 * @tparam eclass Enum class type for type-safe enumeration values
 * 
 * @details Provides type-safe enum parameter wrapper based on AP_Int16.
 *          Use when enum values exceed int8_t range (> 127 or < -128).
 * 
 * Typical usage:
 * ```cpp
 * enum class LargeEnum : uint16_t {
 *     OPTION_FIRST = 0,
 *     OPTION_MIDDLE = 1000,
 *     OPTION_LAST = 5000
 * };
 * 
 * class MyClass {
 *     AP_Enum16<LargeEnum> _large_mode;
 *     static const AP_Param::GroupInfo var_info[];
 * };
 * ```
 * 
 * @note Inherits all AP_Int16 functionality
 * @note Enum values must fit in int16_t range (-32768 to 32767 or 0 to 65535)
 * @note Uses 2 bytes storage vs 1 byte for AP_Enum
 */
template<typename eclass>
class AP_Enum16 : public AP_Int16
{
public:
    /**
     * @brief Implicit conversion to enum type
     * 
     * @return eclass Enum value
     * 
     * @note Enables direct use in comparisons and assignments
     */
    operator const eclass () const {
        return (eclass)_value;
    }
    
    /**
     * @brief Set parameter to enum value (RAM only)
     * 
     * @param[in] v Enum value to assign
     * 
     * @note Does not persist - lost on reboot unless saved
     */
    void set(eclass v) {
        AP_Int16::set(int16_t(v));
    }
    
    /**
     * @brief Set parameter to enum value and queue persistent save
     * 
     * @param[in] v Enum value to assign and save
     * 
     * @note Non-blocking - save queued to I/O thread
     */
    void set_and_save(eclass v) {
        AP_Int16::set_and_save(int16_t(v));
    }
};
