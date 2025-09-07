/**
 * @file init.h
 * @brief This file contains the declarations for the initialization functions.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for initializing the system, including
 * setting up the hardware, initializing mutexes, and preparing the software
 * for operation.
 */
#ifndef INIT_H
#define INIT_H

#include "globals.h"

/**
 * @brief Main setup function, called from Core 0's setup().
 *
 * @details This function performs the initial setup for the system, including initializing
 * the serial port for debugging and initializing the mutexes for inter-core
 * communication.
 */
void main_setup();

/**
 * @brief Setup handler for Core 1.
 *
 * @details This function is called from Core 1's setup1() function. It is responsible for
 * initializing the functionalities that will be handled by Core 1.
 */
void setup1_handler();

/**
 * @brief Initializes the GPIO pins for Core 1.
 *
 * @details This function sets up the direction and initial state of the GPIO pins that are
 * controlled by Core 1.
 */
void initializePinsCore1();

#endif // INIT_H