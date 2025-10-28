#pragma once

/**
 * @file Filter.h
 * @brief Umbrella header for the ArduPilot Filter library - single include for all filter types
 * 
 * @details This header provides convenient access to the complete ArduPilot digital filter library
 *          by including all commonly-used filter implementations. Include this file to access:
 *          - FilterClass.h - Filter<T> base interface for all filter types
 *          - AverageFilter.h - Moving average filters with circular buffer
 *          - DerivativeFilter.h - Timestamp-aware derivative filters for rate estimation
 *          - FilterWithBuffer.h - Base class providing circular buffer storage
 *          - LowPassFilter.h - First-order low-pass filters (constant and variable dt)
 *          - ModeFilter.h - Median/mode filters (rank-order statistics)
 *          - Butter.h - Butterworth filter with pre-computed coefficients
 * 
 *          The filter version (AP_FILTER_VERSION) is logged in the VER message for online
 *          analysis tool compatibility. Analysis tools need to know which filter formulas
 *          and transfer functions to display for a given version of the code.
 * 
 * @note Additional filter types NOT included by this header (must be explicitly included):
 *       - LowPassFilter2p.h - Second-order (two-pole) low-pass filters
 *       - NotchFilter.h - Notch filters for rejecting specific frequencies
 *       - HarmonicNotchFilter.h - Harmonic notch filter banks for motor noise rejection
 *       - SlewLimiter.h - Slew rate limiting filters
 *       - AP_Filter.h - ArduPilot parameter integration and singleton manager
 * 
 * @usage
 * @code
 * #include <Filter/Filter.h>
 * 
 * // Create a low-pass filter with 20 Hz cutoff
 * LowPassFilterFloat lpf(20.0f);
 * 
 * // Apply filter to sensor reading
 * float filtered_value = lpf.apply(raw_sensor_value, dt);
 * @endcode
 * 
 * @copyright Copyright (c) 2010-2025 ArduPilot.org
 * 
 * Source: libraries/Filter/Filter.h:1-20
 */

/**
 * @mainpage ArduPilot Filter Library
 * 
 * @section filter_overview Overview
 * 
 * The ArduPilot Filter library provides a comprehensive set of digital signal processing (DSP)
 * filters for sensor noise reduction, rate limiting, harmonic rejection, and signal conditioning
 * throughout the autopilot system. All filters are designed for real-time embedded operation
 * with minimal CPU overhead and memory footprint.
 * 
 * @section filter_components Components
 * 
 * The library includes the following filter types:
 * 
 * **Base Classes:**
 * - Filter<T> - Abstract base interface defining apply() method for all filters
 * - FilterWithBuffer<T,N> - Base class providing circular buffer storage for windowed filters
 * 
 * **Low-Pass Filters:**
 * - LowPassFilter<T> - First-order (one-pole) low-pass filter with constant or variable dt
 * - LowPassFilter2p<T> - Second-order (two-pole) low-pass filter (not included by Filter.h)
 * - DigitalBiquadFilter<T> - Biquad IIR filter implementation (various filter types)
 * - Butter<T,N> - Butterworth filter with compile-time order and pre-computed coefficients
 * 
 * **Notch Filters:**
 * - NotchFilter<T> - Single notch filter for rejecting specific frequencies (not included by Filter.h)
 * - HarmonicNotchFilter<T> - Bank of notch filters for harmonic rejection (not included by Filter.h)
 * 
 * **Derivative Filters:**
 * - DerivativeFilter<T,N> - Timestamp-aware derivative estimation with optional low-pass filtering
 * 
 * **Statistical Filters:**
 * - ModeFilter<T,N> - Mode filter using rank-order statistics (median/mode)
 * - AverageFilter<T,N> - Simple moving average filter with circular buffer
 * 
 * **Rate Limiting:**
 * - SlewLimiter<T> - Slew rate limiter for smooth transitions (not included by Filter.h)
 * 
 * @section filter_integration Integration with ArduPilot
 * 
 * Filters integrate with the ArduPilot parameter system through:
 * - AP_Param - Parameter storage and persistence to EEPROM
 * - AP_Filter - Singleton manager for filter instances with parameter binding
 * - VER log message - Logs AP_FILTER_VERSION for analysis tool compatibility
 * 
 * @section filter_applications Applications
 * 
 * **Sensor Noise Reduction:**
 * - IMU accelerometer/gyroscope filtering (notch filters for motor noise)
 * - Barometer filtering for altitude estimation (low-pass)
 * - GPS velocity filtering (low-pass, derivative)
 * 
 * **Control System:**
 * - PID derivative term filtering (low-pass to reduce noise amplification)
 * - Setpoint smoothing (slew limiter)
 * - Complementary filtering in attitude estimation
 * 
 * **Harmonic Rejection:**
 * - Motor-induced vibration rejection using harmonic notch filter banks
 * - Dynamic notch filter frequency tracking based on motor RPM
 * 
 * @section filter_design Design Philosophy
 * 
 * The Filter library follows these design principles:
 * 
 * **Header-Only Templates:**
 * Most filters are implemented as header-only templates for efficiency. The compiler can
 * inline and optimize filter operations when types are known at compile time. Explicit
 * template instantiations in .cpp files are provided for common types (float, int16_t, etc.)
 * 
 * **Compile-Time vs Runtime Configuration:**
 * - Compile-time: Butter.h uses template parameters for filter order (zero runtime overhead)
 * - Runtime: LowPassFilter allows dynamic cutoff frequency changes (flexibility for tuning)
 * 
 * **Minimal Dependencies:**
 * Filters depend only on AP_Math for mathematical operations and AP_Param where parameter
 * integration is needed. This keeps filters lightweight and reusable.
 * 
 * **Explicit dt Handling:**
 * Most filters require explicit time delta (dt) to be passed to apply() method rather than
 * assuming a constant update rate. This improves accuracy when scheduler timing varies.
 * 
 * @section filter_version Version Tracking
 * 
 * AP_FILTER_VERSION is incremented when significant filtering algorithm changes occur.
 * This version is logged in the VER message to ensure online analysis tools (e.g., log
 * viewers) can display the correct filter transfer functions and frequency responses
 * corresponding to the code version that generated the log.
 * 
 * @note Current filter version: AP_FILTER_VERSION = 2
 * 
 * @see FilterClass.h for the base Filter<T> interface
 * @see LowPassFilter.h for first-order low-pass filter implementation
 * @see NotchFilter.h for harmonic rejection filters
 * @see AP_Filter.h for parameter integration and filter management
 */

// Include core filter headers
#include "FilterClass.h"       // Filter<T> base interface
#include "AverageFilter.h"     // Moving average filters with circular buffer
#include "DerivativeFilter.h"  // Timestamp-aware derivative estimation filters
#include "FilterWithBuffer.h"  // Circular buffer base class for windowed filters
#include "LowPassFilter.h"     // First-order low-pass filters (constant and variable dt)
#include "ModeFilter.h"        // Median/mode filters using rank-order statistics
#include "Butter.h"            // Butterworth filter with pre-computed coefficients

/**
 * @def AP_FILTER_VERSION
 * @brief Filter algorithm version number for log analysis tool compatibility
 * 
 * @details This version number is logged in the VER (version) message to assist online
 *          analysis tools in determining which filter algorithms and transfer functions
 *          were used to generate a particular log file.
 * 
 *          When significant changes are made to filtering algorithms (e.g., notch filter
 *          implementation changes, low-pass filter formula updates, harmonic notch
 *          modifications), this version should be incremented to ensure log analysis
 *          tools display the correct filter frequency responses and transfer functions.
 * 
 *          Version History:
 *          - Version 1: Initial filter implementations
 *          - Version 2: Current version (harmonic notch filter updates, improved biquad implementation)
 * 
 * @note Log analysis tools (e.g., UAV LogViewer, MAVExplorer) use this version to:
 *       - Display correct filter transfer functions in frequency domain plots
 *       - Calculate phase delay introduced by filters
 *       - Show accurate magnitude response in Bode plots
 *       - Ensure filter configuration parameters are interpreted correctly
 * 
 * @warning Do NOT increment this version for documentation-only changes or non-functional
 *          code updates. Only increment when actual filter mathematics or behavior changes.
 */
#define AP_FILTER_VERSION 2

