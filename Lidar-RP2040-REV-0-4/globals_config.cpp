/**
 * @file globals_config.cpp
 * @brief Implementation of runtime configurable global parameters
 * @author The Lidar-RP2040-REV-0-4 Team
 * @version 1.0
 * @date 2025-09-07
 */

#include "globals_config.h"

// Global instance
GlobalConfiguration runtimeGlobals;

/**
 * @brief Load default global configuration values (safe parameters only)
 */
void loadDefaultGlobals() {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Loading default global configuration...");
    
    // System settings
    runtimeGlobals.config_mode_timeout_ms = CONFIG_MODE_TIMEOUT_MS;
    runtimeGlobals.min_strength_threshold = MIN_STRENGTH_THRESHOLD;
    
    // Recovery & error handling
    runtimeGlobals.max_recovery_attempts = MAX_RECOVERY_ATTEMPTS;
    runtimeGlobals.recovery_attempt_delay_ms = RECOVERY_ATTEMPT_DELAY_MS;
    
    // Timing & performance
    runtimeGlobals.startup_delay_ms = STARTUP_DELAY_MS;
    runtimeGlobals.lidar_init_step_delay_ms = LIDAR_INIT_STEP_DELAY_MS;
    runtimeGlobals.lidar_final_delay_ms = LIDAR_FINAL_DELAY_MS;
    runtimeGlobals.command_response_delay_ms = COMMAND_RESPONSE_DELAY_MS;
    
    // Debug & monitoring
    runtimeGlobals.debug_output_interval_ms = DEBUG_OUTPUT_INTERVAL_MS;
    runtimeGlobals.status_check_interval_ms = STATUS_CHECK_INTERVAL_MS;
    runtimeGlobals.performance_report_interval_ms = PERFORMANCE_REPORT_INTERVAL_MS;
    runtimeGlobals.critical_error_report_interval_ms = CRITICAL_ERROR_REPORT_INTERVAL_MS;
    
    // Signal processing
    runtimeGlobals.distance_deadband_threshold_cm = DISTANCE_DEADBAND_THRESHOLD_CM;
    runtimeGlobals.velocity_deadband_threshold_cm_s = VELOCITY_DEADBAND_THRESHOLD_CM_S;
    
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Default globals loaded");
}

/**
 * @brief Validate global configuration values (safe parameters only)
 */
bool validateGlobalConfiguration(const GlobalConfiguration& config) {
    // Validate ranges for safe runtime parameters
    if (config.config_mode_timeout_ms < 1000 || config.config_mode_timeout_ms > 60000) {
        safeSerialPrintln("Global validation failed: config_mode_timeout_ms out of range");
        return false;
    }
    
    if (config.min_strength_threshold < 50 || config.min_strength_threshold > 1000) {
        safeSerialPrintln("Global validation failed: min_strength_threshold out of range");
        return false;
    }
    
    if (config.velocity_deadband_threshold_cm_s < 0.1f || config.velocity_deadband_threshold_cm_s > 5.0f) {
        safeSerialPrintln("Global validation failed: velocity_deadband_threshold_cm_s out of range");
        return false;
    }
    
    if (config.distance_deadband_threshold_cm < 1 || config.distance_deadband_threshold_cm > 10) {
        safeSerialPrintln("Global validation failed: distance_deadband_threshold_cm out of range");
        return false;
    }
    
    if (config.max_recovery_attempts < 1 || config.max_recovery_attempts > 10) {
        safeSerialPrintln("Global validation failed: max_recovery_attempts out of range");
        return false;
    }
    
    // Validate timing parameters
    if (config.startup_delay_ms < 100 || config.startup_delay_ms > 5000) {
        safeSerialPrintln("Global validation failed: startup_delay_ms out of range");
        return false;
    }
    
    if (config.debug_output_interval_ms < 50 || config.debug_output_interval_ms > 5000) {
        safeSerialPrintln("Global validation failed: debug_output_interval_ms out of range");
        return false;
    }
    
    if (config.recovery_attempt_delay_ms < 1000 || config.recovery_attempt_delay_ms > 30000) {
        safeSerialPrintln("Global validation failed: recovery_attempt_delay_ms out of range");
        return false;
    }
    
    if (config.lidar_init_step_delay_ms < 100 || config.lidar_init_step_delay_ms > 2000) {
        safeSerialPrintln("Global validation failed: lidar_init_step_delay_ms out of range");
        return false;
    }
    
    if (config.lidar_final_delay_ms < 50 || config.lidar_final_delay_ms > 1000) {
        safeSerialPrintln("Global validation failed: lidar_final_delay_ms out of range");
        return false;
    }
    
    if (config.command_response_delay_ms < 10 || config.command_response_delay_ms > 500) {
        safeSerialPrintln("Global validation failed: command_response_delay_ms out of range");
        return false;
    }
    
    if (config.status_check_interval_ms < 1000 || config.status_check_interval_ms > 30000) {
        safeSerialPrintln("Global validation failed: status_check_interval_ms out of range");
        return false;
    }
    
    if (config.performance_report_interval_ms < 5000 || config.performance_report_interval_ms > 60000) {
        safeSerialPrintln("Global validation failed: performance_report_interval_ms out of range");
        return false;
    }
    
    if (config.critical_error_report_interval_ms < 500 || config.critical_error_report_interval_ms > 10000) {
        safeSerialPrintln("Global validation failed: critical_error_report_interval_ms out of range");
        return false;
    }
    
    return true;
}

/**
 * @brief Calculate checksum for global configuration
 */
uint16_t calculateGlobalsChecksum(const GlobalConfiguration& config) {
    uint16_t sum = 0;
    const uint8_t* p = (const uint8_t*)&config;
    for (size_t i = 0; i < sizeof(config) - sizeof(config.checksum); ++i) {
        sum += p[i];
    }
    return sum;
}

/**
 * @brief Load global configuration from storage
 */
void loadGlobalConfiguration() {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Loading global configuration from LittleFS...");
    
    const char* GLOBALS_FILE_PATH = "/lidar_globals.dat";
    
    if (LittleFS.exists(GLOBALS_FILE_PATH)) {
        File globalsFile = LittleFS.open(GLOBALS_FILE_PATH, "r");
        if (globalsFile) {
            size_t bytesRead = globalsFile.readBytes((char*)&runtimeGlobals, sizeof(GlobalConfiguration));
            globalsFile.close();
            
            if (bytesRead == sizeof(GlobalConfiguration)) {
                uint16_t calculated_checksum = calculateGlobalsChecksum(runtimeGlobals);
                
                if (calculated_checksum == runtimeGlobals.checksum && validateGlobalConfiguration(runtimeGlobals)) {
                    if (isDebugEnabled()) safeSerialPrintln("Core 1: Valid global configuration loaded from LittleFS");
                    return;
                } else {
                    if (isDebugEnabled()) safeSerialPrintln("Core 1: Global configuration validation failed - using defaults");
                }
            } else {
                if (isDebugEnabled()) safeSerialPrintln("Core 1: Invalid globals file size - using defaults");
            }
        } else {
            if (isDebugEnabled()) safeSerialPrintln("Core 1: Could not open globals file - using defaults");
        }
    } else {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Globals file not found - using defaults");
    }
    
    loadDefaultGlobals();
}

/**
 * @brief Save global configuration to storage
 */
bool saveGlobalConfiguration() {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Saving global configuration to LittleFS...");
    
    if (!validateGlobalConfiguration(runtimeGlobals)) {
        safeSerialPrintln("Core 1: ERROR - Cannot save invalid global configuration");
        return false;
    }
    
    runtimeGlobals.checksum = calculateGlobalsChecksum(runtimeGlobals);
    
    const char* GLOBALS_FILE_PATH = "/lidar_globals.dat";
    
    File globalsFile = LittleFS.open(GLOBALS_FILE_PATH, "w");
    if (!globalsFile) {
        safeSerialPrintln("Core 1: ERROR - Failed to open globals file for writing");
        return false;
    }
    
    size_t bytesWritten = globalsFile.write((uint8_t*)&runtimeGlobals, sizeof(GlobalConfiguration));
    globalsFile.close();
    
    if (bytesWritten == sizeof(GlobalConfiguration)) {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Global configuration successfully saved to LittleFS");
        return true;
    } else {
        safeSerialPrintln("Core 1: ERROR - Incomplete write to globals file");
        return false;
    }
}

/**
 * @brief Factory reset global configuration
 */
void factoryResetGlobals() {
    if (isDebugEnabled()) safeSerialPrintln("Core 1: Factory resetting global configuration...");
    
    const char* GLOBALS_FILE_PATH = "/lidar_globals.dat";
    
    if (LittleFS.remove(GLOBALS_FILE_PATH)) {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Globals file removed from LittleFS");
    } else {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Globals file removal failed (may not exist)");
    }
    
    loadDefaultGlobals();
}