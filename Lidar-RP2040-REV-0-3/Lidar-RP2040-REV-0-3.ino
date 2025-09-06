/**
 * @file Lidar-RP2040-REV-0-1.ino
 * @brief Main entry point for the RP2040 LiDAR Controller.
 * @version 6.3 (Modular Refactor)
 * @date August 27, 2025
 * @author Arduino-Pico Port with Native Dual-Core Support
 *
 * This file initializes the system and launches the main loops for both cores.
 * The core logic is handled by functions in the associated .cpp modules.
 */

#include "globals.h"
#include "init.h"
#include "core0_handling.h"
#include "core1_handling.h"

/**
 * @brief Core 0 setup. Initializes serial, mutexes, and the Core 0 state machine.
 */
void setup() {
  main_setup();
}

/**
 * @brief Core 0 main loop. Handles LiDAR data acquisition.
 */
void loop() {
  loop0_handler();
}

/**
 * @brief Core 1 setup. Initializes pins, storage, and the Core 1 state machine.
 */
void setup1() {
  setup1_handler();
}

/**
 * @brief Core 1 main loop. Handles data processing, logic, and GUI communication.
 */
void loop1() {
  loop1_handler();
}