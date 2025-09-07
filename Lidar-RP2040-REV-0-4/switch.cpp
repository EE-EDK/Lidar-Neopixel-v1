/**
 * @file switch.cpp
 * @brief This file contains the implementation for the switch reading function.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The function in this file is responsible for reading the state of the
 * configuration switches and returning a code that represents the current
 * switch settings.
 */

#include "switch.h"
#include "globals.h"

/**
 * @brief Reads the switch code from the configuration switches.
 *
 * @details This function reads the state of the three configuration switches (S1, S2,
 * and S4) and combines them into a 3-bit code. The switches are active
 * low, so the digitalRead values are inverted.
 *
 * @return A code representing the current switch settings (0-7).
 */
uint8_t readSwitchCode() {
  return (!digitalRead(S4_PIN) << 2) | (!digitalRead(S2_PIN) << 1) | !digitalRead(S1_PIN);
}