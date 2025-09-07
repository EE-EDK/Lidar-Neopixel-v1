/**
 * @file status.h
 * @brief This file contains the declarations for the status handling functions.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for handling status indicators,
 * such as the status LED and debug output. They provide a way to monitor the
 * system's state and performance.
 */
#ifndef STATUS_H
#define STATUS_H

#include "globals.h"

/**
 * @brief Handles the debug output.
 *
 * @details This function is responsible for printing debug information to the serial port.
 * It is typically called periodically when debug output is enabled.
 */
void handleDebugOutput();

/**
 * @brief Handles the status LED.
 *
 * @details This function controls the status LED to provide visual feedback on the
 * system's state. The LED can be used to indicate different modes of operation,
 * such as initialization, running, and error states.
 */
void handleStatusLED();

/**
 * @brief Reports the status of Core 0.
 *
 * @details This function is responsible for reporting the status of Core 0. This may
 * include information about the LiDAR sensor, data acquisition, and any
 * errors that have occurred.
 */
void reportCore0Status();

/**
 * @brief Reports the status of Core 1.
 *
 * @details This function is responsible for reporting the status of Core 1. This may
 * include information about data processing, GUI communication, and system
 * performance.
 */
void reportCore1Status();

#endif // STATUS_H