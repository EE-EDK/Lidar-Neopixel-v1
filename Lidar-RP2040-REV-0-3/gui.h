/**
 * @file gui.h
 * @brief This file contains the declaration for the GUI command processing function.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The function in this file is responsible for processing commands received from the
 * graphical user interface (GUI).
 */
#ifndef GUI_H
#define GUI_H

#include "globals.h"

/**
 * @brief Processes commands received from the GUI.
 *
 * @details This function reads and parses commands from the serial port that are sent by the
 * GUI. It handles various commands for configuring the device, retrieving data,
 * and controlling its operation.
 */
void processGuiCommands();

#endif // GUI_H