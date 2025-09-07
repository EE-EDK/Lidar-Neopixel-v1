/**
 * @file storage.h
 * @brief This file contains the declarations for the storage handling functions.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for managing the device's
 * configuration, including loading, saving, and validating the configuration
 * data stored in non-volatile memory.
 */
#ifndef STORAGE_H
#define STORAGE_H

#include "globals.h"

/**
 * @brief Loads the default configuration.
 *
 * @details This function loads a set of default values into the current configuration.
 * This is typically used when no valid configuration is found in storage.
 */
void loadDefaultConfig();

/**
 * @brief Validates the given configuration.
 *
 * @param config The configuration to validate.
 * @return True if the configuration is valid, false otherwise.
 */
bool validateConfiguration(const LidarConfiguration& config);

/**
 * @brief Calculates the checksum for the given configuration.
 *
 * @param config The configuration to calculate the checksum for.
 * @return The calculated checksum.
 */
uint16_t calculateChecksum(const LidarConfiguration& config);

/**
 * @brief Loads the configuration from storage.
 *
 * @details This function reads the configuration data from non-volatile storage and
 * loads it into the current configuration. If no valid configuration is found,
 * it loads the default configuration.
 */
void loadConfiguration();

/**
 * @brief Saves the current configuration to storage.
 *
 * @return True if the configuration was saved successfully, false otherwise.
 */
bool saveConfiguration();

/**
 * @brief Performs a factory reset.
 *
 * @details This function resets the configuration to its default values and saves it
 * to storage.
 */
void factoryReset();

#endif // STORAGE_H