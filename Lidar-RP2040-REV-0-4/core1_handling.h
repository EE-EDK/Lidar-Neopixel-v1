/**
 * @file core1_handling.h
 * @brief This file contains the declarations for functions that handle Core 1 operations.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for managing the main loop of Core 1,
 * which includes processing the Core 1 state machine and processing incoming
 * LiDAR frames from Core 0.
 */
#ifndef CORE1_HANDLING_H
#define CORE1_HANDLING_H

#include "globals.h"

/**
 * @brief Main handler for Core 1 loop.
 *
 * @details This function is called repeatedly in the main loop of Core 1. It is responsible for
 * orchestrating the tasks performed by Core 1.
 */
void loop1_handler();
/**
 * @brief Processes the state machine for Core 1.
 *
 * @details This function manages the different states of Core 1. It handles tasks such as
 * system initialization, processing data, and managing system logic.
 */
void processCore1StateMachine();
/**
 * @brief Processes incoming LiDAR frames from Core 0.
 *
 * @details This function retrieves LiDAR frames from the shared buffer, processes them,
 * and performs calculations such as velocity estimation.
 */
void processIncomingFrames();

#endif // CORE1_HANDLING_H