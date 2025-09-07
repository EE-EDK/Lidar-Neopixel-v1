/**
 * @file core0_handling.h
 * @brief This file contains the declarations for functions that handle Core 0 operations.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for managing the main loop of Core 0,
 * which includes processing the Core 0 state machine, handling LiDAR serial data,
 * and performing health checks and recovery operations for the LiDAR sensor.
 */
#ifndef CORE0_HANDLING_H
#define CORE0_HANDLING_H

#include "globals.h"

/**
 * @brief Main handler for Core 0 loop.
 *
 * @details This function is called repeatedly in the main loop of Core 0. It is responsible for
 * orchestrating the tasks performed by Core 0, such as processing the state machine.
 */
void loop0_handler();
/**
 * @brief Processes the state machine for Core 0.
 *
 * @details This function manages the different states of Core 0, such as initialization,
 * data acquisition, and error handling. It transitions between states based on
 * system events and sensor data.
 */
void processCore0StateMachine();
/**
 * @brief Processes incoming serial data from the LiDAR sensor.
 *
 * @details This function reads and parses data from the LiDAR sensor's serial port. It extracts
 * distance and quality information from the data stream and stores it for processing.
 */
void processLidarSerial();
/**
 * @brief Attempts to recover the LiDAR sensor from an error state.
 *
 * @param recovery_level The level of recovery to attempt.
 * @return True if recovery was successful, false otherwise.
 */
bool attemptRecovery(uint8_t recovery_level);
/**
 * @brief Checks the health of the LiDAR sensor.
 *
 * @details This function performs a health check on the LiDAR sensor to ensure it is functioning
 * correctly. It may involve sending a command to the sensor and checking the response.
 *
 * @return True if the sensor is healthy, false otherwise.
 */
bool checkLidarSensorHealth(); 

#endif // CORE0_HANDLING_H