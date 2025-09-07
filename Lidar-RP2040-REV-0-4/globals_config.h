/**
 * @file globals_config.h
 * @brief Runtime configurable global parameters
 * @author The Lidar-RP2040-REV-0-4 Team
 * @version 1.0
 * @date 2025-09-07
 */

#ifndef GLOBALS_CONFIG_H
#define GLOBALS_CONFIG_H

#include "globals.h"

/**
 * @brief Structure to hold runtime-configurable global parameters (safe parameters only)
 */
struct GlobalConfiguration {
  // System settings
  uint32_t config_mode_timeout_ms;  ///< Time to wait for GUI connection
  uint32_t min_strength_threshold;  ///< Minimum LiDAR signal strength

  // Recovery & error handling
  uint32_t max_recovery_attempts;      ///< Maximum recovery attempts
  uint32_t recovery_attempt_delay_ms;  ///< Delay between recovery attempts

  // Timing & performance
  uint32_t startup_delay_ms;           ///< Initial system startup delay
  uint32_t lidar_init_step_delay_ms;   ///< Delay between LiDAR config commands
  uint32_t lidar_final_delay_ms;       ///< Final delay before data collection
  uint32_t command_response_delay_ms;  ///< Command acknowledgment delay

  // Debug & monitoring
  uint32_t debug_output_interval_ms;           ///< Debug output frequency
  uint32_t status_check_interval_ms;           ///< Status report frequency
  uint32_t performance_report_interval_ms;     ///< Performance statistics frequency
  uint32_t critical_error_report_interval_ms;  ///< Critical error message rate limiting

  // Signal processing
  uint32_t distance_deadband_threshold_cm;  ///< Distance noise filtering threshold
  float velocity_deadband_threshold_cm_s;   ///< Velocity noise filtering threshold

  uint16_t checksum;  ///< Checksum for validation
};

/**
 * @brief Global instance of runtime configuration
 */
extern GlobalConfiguration runtimeGlobals;

/**
 * @brief Function declarations for globals management
 */
void loadDefaultGlobals();
bool validateGlobalConfiguration(const GlobalConfiguration& config);
uint16_t calculateGlobalsChecksum(const GlobalConfiguration& config);
void loadGlobalConfiguration();
bool saveGlobalConfiguration();
void factoryResetGlobals();

// Accessor macros for runtime globals (safe parameters only)
#define RUNTIME_CONFIG_MODE_TIMEOUT_MS (runtimeGlobals.config_mode_timeout_ms)
#define RUNTIME_MIN_STRENGTH_THRESHOLD (runtimeGlobals.min_strength_threshold)
#define RUNTIME_MAX_RECOVERY_ATTEMPTS (runtimeGlobals.max_recovery_attempts)
#define RUNTIME_RECOVERY_ATTEMPT_DELAY_MS (runtimeGlobals.recovery_attempt_delay_ms)
#define RUNTIME_STARTUP_DELAY_MS (runtimeGlobals.startup_delay_ms)
#define RUNTIME_LIDAR_INIT_STEP_DELAY_MS (runtimeGlobals.lidar_init_step_delay_ms)
#define RUNTIME_LIDAR_FINAL_DELAY_MS (runtimeGlobals.lidar_final_delay_ms)
#define RUNTIME_COMMAND_RESPONSE_DELAY_MS (runtimeGlobals.command_response_delay_ms)
#define RUNTIME_DEBUG_OUTPUT_INTERVAL_MS (runtimeGlobals.debug_output_interval_ms)
#define RUNTIME_STATUS_CHECK_INTERVAL_MS (runtimeGlobals.status_check_interval_ms)
#define RUNTIME_PERFORMANCE_REPORT_INTERVAL_MS (runtimeGlobals.performance_report_interval_ms)
#define RUNTIME_CRITICAL_ERROR_REPORT_INTERVAL_MS (runtimeGlobals.critical_error_report_interval_ms)
#define RUNTIME_DISTANCE_DEADBAND_THRESHOLD_CM (runtimeGlobals.distance_deadband_threshold_cm)
#define RUNTIME_VELOCITY_DEADBAND_THRESHOLD_CM_S (runtimeGlobals.velocity_deadband_threshold_cm_s)

#endif  // GLOBALS_CONFIG_H