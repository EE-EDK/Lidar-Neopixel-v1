/**
 * @file Lidar-RP2040-REV-0-4.ino
 * @brief Main entry point for the Lidar-RP2040-REV-0-3 firmware.
 * @author Ethan Kunz
 * @version 1.0
 * @date 2025-09-06
 *
 * @details This file serves as the main entry point for the firmware running on the RP2040 microcontroller.
 * It leverages the dual-core capabilities of the RP2040.
 * - **Core 0**: Handles LiDAR data acquisition. Its execution starts with `setup()` and continues in `loop()`.
 * - **Core 1**: Manages data processing, system logic, and GUI communication. Its execution starts with `setup1()` and continues in `loop1()`.
 *
 * The actual logic for each core is abstracted into handler functions, which are defined in other files,
 * promoting modularity and readability.
 */

#include "globals.h"
#include "init.h"
#include "core0_handling.h"
#include "core1_handling.h"

/**
 * @brief Setup function for Core 0.
 *
 * @details This function is the entry point for Core 0. It calls `main_setup()` to perform
 * essential initializations, such as setting up serial communication for debugging,
 * initializing mutexes for safe data sharing between cores, and preparing the Core 0 state machine.
 */
void setup() {
  main_setup();
}

/**
 * @brief Main loop for Core 0.
 *
 * @details This function is executed repeatedly on Core 0 after `setup()` has completed. It calls
 * `loop0_handler()`, which contains the main logic for Core 0, primarily focused on
 * acquiring data from the LiDAR sensor.
 */
void loop() {
  loop0_handler();
}

/**
 * @brief Setup function for Core 1.
 *
 * @details This function is the entry point for Core 1. It calls `setup1_handler()` to initialize
 * hardware pins, set up storage for configuration and data logging, and prepare the
 * Core 1 state machine for operation.
 */
void setup1() {
  setup1_handler();
}

/**
 * @brief Main loop for Core 1.
 *
 * @details This function is executed repeatedly on Core 1 after `setup1()` has completed.
 * It calls `loop1_handler()`, which is responsible for processing the data acquired by Core 0,
 * implementing the device's main logic, and handling communication with the graphical user interface (GUI).
 */
void loop1() {
  loop1_handler();
}