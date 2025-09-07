/**
 * @file switch.h
 * @brief This file contains the declaration for the switch reading function.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The function in this file is responsible for reading the state of the
 * configuration switches and returning a code that represents the current
 * switch settings.
 */
#ifndef SWITCH_H
#define SWITCH_H

#include "globals.h"
#include <cstdint>

/**
 * @brief Reads the switch code from the configuration switches.
 *
 * @return A code representing the current switch settings.
 */
uint8_t readSwitchCode();

#endif // SWITCH_H