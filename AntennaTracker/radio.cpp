#include "Tracker.h"

/**
 * @file radio.cpp
 * @brief RC radio input handling for antenna tracker
 * 
 * @details Reads and processes RC receiver input channels for manual control.
 *          This module provides the interface between the RC receiver hardware
 *          and the tracker control system, enabling manual operation and
 *          optional mode switching via RC transmitter.
 */

// Functions to read the RC radio input

/**
 * @brief Reads RC radio input from receiver
 * 
 * @details Called by scheduler at configured rate (typically 50Hz).
 *          Delegates to RC_Channels::read_input() which performs the
 *          following operations:
 *          - Reads PWM values from RC receiver hardware via HAL
 *          - Updates RC channel objects with current input values
 *          - Detects RC failsafe conditions (no signal, out of range)
 *          - Makes RC inputs available for mode implementations
 *          
 *          The RC input values are used by the tracker for manual control
 *          of pan/tilt servos and optional mode switching.
 * 
 * @note RC input required for MANUAL mode and optional for mode switching
 * @note RC failsafe handling performed by RC_Channels subsystem
 * 
 * @see RC_Channels::read_input()
 * 
 * Source: AntennaTracker/radio.cpp:5-8
 */
void Tracker::read_radio()
{
    rc().read_input();
}
