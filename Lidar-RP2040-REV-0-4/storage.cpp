/**
 * @file storage.cpp
 * @brief This file contains the implementation for the storage handling functions.
 * @author The Lidar-RP2040-REV-0-4 Team
 * @version 1.0
 * @date 2025-09-07
 *
 * @details This file provides the implementation for managing the device's
 * configuration, including loading, saving, and validating the configuration
 * data stored in non-volatile memory. Now includes support for global
 * parameter configuration.
 */

#include "storage.h"
#include "globals_config.h"  // NEW: Include globals configuration

/**
 * @brief Loads the default configuration.
 *
 * @details This function loads a set of default values into the current configuration.
 * This is typically used when no valid configuration is found in storage.
 */
void loadDefaultConfig() {
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Loading factory default configuration...");
  uint16_t default_distances[8] = { 50, 100, 200, 300, 400, 500, 600, 700 };
  int16_t default_vel_min[8] = { -2200, -2200, -2200, -2200, -2200, -2200, -2200, -2200 };
  int16_t default_vel_max[8] = { -250, -250, -250, -250, -250, -250, -250, -250 };
  uint8_t default_rules[8][4] = {
    { 0, 0, 0, 1 }, { 0, 0, 1, 1 }, { 0, 1, 0, 1 }, { 0, 1, 1, 1 }, 
    { 1, 0, 0, 1 }, { 1, 0, 1, 0 }, { 1, 1, 0, 0 }, { 1, 1, 1, 0 }
  };
  memcpy(currentConfig.distance_thresholds, default_distances, sizeof(default_distances));
  memcpy(currentConfig.velocity_min_thresholds, default_vel_min, sizeof(default_vel_min));
  memcpy(currentConfig.velocity_max_thresholds, default_vel_max, sizeof(default_vel_max));
  memcpy(currentConfig.trigger_rules, default_rules, sizeof(default_rules));
  currentConfig.use_velocity_trigger = true;
  currentConfig.enable_debug = false;
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Factory defaults loaded");
}

/**
 * @brief Validates the given configuration.
 *
 * @param config The configuration to validate.
 * @return True if the configuration is valid, false otherwise.
 */
bool validateConfiguration(const LidarConfiguration& config) {
  for (int i = 0; i < 8; i++) {
    if (config.distance_thresholds[i] < MIN_DISTANCE_CM || 
        config.distance_thresholds[i] > MAX_DISTANCE_CM) {
      safeSerialPrintfln("Config validation failed: distance[%d] = %d out of range", i, config.distance_thresholds[i]);
      safeSetErrorFlag(ERROR_FLAG_CONFIG_ERROR, true);
      return false;
    }
    if (config.velocity_min_thresholds[i] > config.velocity_max_thresholds[i]) {
      safeSerialPrintfln("Config validation failed: velocity range[%d] min > max", i);
      safeSetErrorFlag(ERROR_FLAG_CONFIG_ERROR, true);
      return false;
    }
  }
  safeSetErrorFlag(ERROR_FLAG_CONFIG_ERROR, false);
  return true;
}

/**
 * @brief Calculates the checksum for the given configuration.
 *
 * @param config The configuration to calculate the checksum for.
 * @return The calculated checksum.
 */
uint16_t calculateChecksum(const LidarConfiguration& config) {
  uint16_t sum = 0;
  const uint8_t* p = (const uint8_t*)&config;
  for (size_t i = 0; i < sizeof(config) - sizeof(config.checksum); ++i) {
    sum += p[i];
  }
  return sum;
}

/**
 * @brief Loads the configuration from storage.
 *
 * @details This function reads the configuration data from non-volatile storage and
 * loads it into the current configuration. If no valid configuration is found,
 * it loads the default configuration.
 */
void loadConfiguration() {
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Attempting to load configuration from LittleFS...");
  if (!LittleFS.begin()) {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: LittleFS mount failed, formatting...");
    LittleFS.format();
    if (!LittleFS.begin()) {
      if (isDebugEnabled()) safeSerialPrintln("Core 1: LittleFS initialization failed, using defaults");
      loadDefaultConfig();
      return;
    }
  }

  if (LittleFS.exists(CONFIG_FILE_PATH)) {
    File configFile = LittleFS.open(CONFIG_FILE_PATH, "r");
    if (configFile) {
      size_t bytesRead = configFile.readBytes((char*)&currentConfig, sizeof(LidarConfiguration));
      configFile.close();
      if (isDebugEnabled()) safeSerialPrintfln("Core 1: Read %zu bytes from config file", bytesRead);

      if (bytesRead == sizeof(LidarConfiguration)) {
        uint16_t calculated_checksum = calculateChecksum(currentConfig);
        if (isDebugEnabled()) safeSerialPrintfln("Core 1: Checksum - stored: %d, calculated: %d", currentConfig.checksum, calculated_checksum);
        
        if (calculated_checksum == currentConfig.checksum && validateConfiguration(currentConfig)) {
          if (isDebugEnabled()) safeSerialPrintln("Core 1: Valid configuration loaded from LittleFS");
          mutex_enter_blocking(&comm_mutex);
          core_comm.enable_debug = currentConfig.enable_debug;
          mutex_exit(&comm_mutex);
          return;
        } else if (isDebugEnabled()) safeSerialPrintln("Core 1: Configuration validation failed - using defaults");
      } else if (isDebugEnabled()) safeSerialPrintln("Core 1: Invalid file size - using defaults");
    } else if (isDebugEnabled()) safeSerialPrintln("Core 1: Could not open config file - using defaults");
  } else if (isDebugEnabled()) safeSerialPrintln("Core 1: Config file not found - using defaults");

  loadDefaultConfig();
  mutex_enter_blocking(&comm_mutex);
  core_comm.enable_debug = currentConfig.enable_debug;
  mutex_exit(&comm_mutex);
  
  if (isDebugEnabled()) {
    safeSerialPrintfln("Core 1: Config summary - Mode: %s, Debug: %s",
      currentConfig.use_velocity_trigger ? "Distance+Velocity" : "Distance Only",
      currentConfig.enable_debug ? "ON" : "OFF");
  }
}

/**
 * @brief Saves the current configuration to storage.
 *
 * @return True if the configuration was saved successfully, false otherwise.
 */
bool saveConfiguration() {
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Saving configuration to LittleFS...");
  if (!validateConfiguration(currentConfig)) {
    safeSerialPrintln("Core 1: ERROR - Cannot save invalid configuration");
    return false;
  }
  currentConfig.checksum = calculateChecksum(currentConfig);

  File configFile = LittleFS.open(CONFIG_FILE_PATH, "w");
  if (!configFile) {
    safeSerialPrintln("Core 1: ERROR - Failed to open config file for writing");
    return false;
  }
  size_t bytesWritten = configFile.write((uint8_t*)&currentConfig, sizeof(LidarConfiguration));
  configFile.close();

  if (isDebugEnabled()) safeSerialPrintfln("Core 1: Wrote %zu bytes to config file", bytesWritten);
  
  bool config_success = false;
  if (bytesWritten == sizeof(LidarConfiguration)) {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Configuration successfully saved to LittleFS");
    config_success = true;
  } else {
    safeSerialPrintln("Core 1: ERROR - Incomplete write to config file");
  }
  
  // NEW: Also save global configuration
  if (!saveGlobalConfiguration()) {
    safeSerialPrintln("Core 1: WARNING - Failed to save global configuration");
    // Don't fail the entire operation, just warn
  }
  
  return config_success;
}

/**
 * @brief Performs a factory reset.
 *
 * @details This function resets the configuration to its default values and saves it
 * to storage. It then reboots the device.
 */
void factoryReset() {
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Performing factory reset...");
  if (LittleFS.remove(CONFIG_FILE_PATH)) {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Config file removed from LittleFS");
  } else if (isDebugEnabled()) safeSerialPrintln("Core 1: Config file removal failed (may not exist)");

  // NEW: Also reset globals
  factoryResetGlobals();

  loadDefaultConfig();
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Factory reset complete, rebooting in 100ms...");
  delay(100);
  rp2040.restart();
}