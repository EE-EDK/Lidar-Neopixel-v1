/**
 * @file status.cpp
 * @brief This file contains the implementation of the status handling functions.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for handling status indicators,
 * such as the status LED and debug output.
 */

#include "status.h"
#include "globals.h"
#include "globals_config.h"

/**
 * @brief Handles the debug output.
 *
 * @details This function prints debug information to the serial port periodically
 * when debug output is enabled. The information includes velocity, signal
 * strength, distance, error flags, and trigger status.
 */
void handleDebugOutput() {
    if (safeMillisElapsed(timing_info.last_debug_output, millis()) >= RUNTIME_DEBUG_OUTPUT_INTERVAL_MS) {
    if (current_state == STATE_RUNNING && isDebugEnabled()) {
      uint32_t frames_received_local, error_flags_local;
      uint8_t switch_code_local, buffer_count_local;
      bool trigger_output_local;
      float velocity_local;
      uint16_t distance_local, strength_local;

      mutex_enter_blocking(&comm_mutex);
      frames_received_local = core_comm.frames_received;
      error_flags_local = core_comm.error_flags;
      switch_code_local = core_comm.switch_code;
      trigger_output_local = core_comm.trigger_output;
      velocity_local = core_comm.velocity;
      distance_local = core_comm.distance;
      strength_local = core_comm.strength;
      mutex_exit(&comm_mutex);

      buffer_count_local = getBufferUtilization();
      safeSerialPrintfln("DEBUG: Velocity=%6.1fcm/s   Strength=%5d   Dist=%4dcm   Errors=0x%02x   Trigger=%s",
        velocity_local, strength_local, distance_local, error_flags_local,
        trigger_output_local ? "ACTIVE" : "INACTIVE");
    }
    timing_info.last_debug_output = millis();
  }
}

/**
 * @brief Handles the status LED.
 *
 * @details This function controls the status LED to provide visual feedback on the
 * system's state. The LED blinks at different rates to indicate different
 * conditions, such as configuration mode, buffer warnings, and communication
 * timeouts.
 */
void handleStatusLED() {
  static uint32_t last_blink = 0;
  static bool led_state = false;
  uint32_t now = millis();
  uint32_t blink_rate = 1000;
  uint32_t error_flags_local;

  mutex_enter_blocking(&comm_mutex);
  error_flags_local = core_comm.error_flags;
  mutex_exit(&comm_mutex);

  if (current_state == STATE_CONFIG) blink_rate = 100;
  else if (error_flags_local & ERROR_FLAG_BUFFER_CRITICAL) blink_rate = 10;
  else if (error_flags_local & ERROR_FLAG_BUFFER_WARNING) blink_rate = 200;
  else if (error_flags_local & ERROR_FLAG_COMM_TIMEOUT) blink_rate = 300;

  if (safeMillisElapsed(last_blink, now) > blink_rate) {
    led_state = !led_state;
    digitalWrite(STATUS_LED_PIN, led_state);
    last_blink = now;
  }
}

/**
 * @brief Reports the status of Core 0.
 *
 * @details This function is a placeholder for reporting the status of Core 0.
 * The implementation has been optimized out in the current version.
 */
void reportCore0Status() {
  static uint32_t last_status_report = 0;
  if (safeMillisElapsed(last_status_report, millis()) >= RUNTIME_STATUS_CHECK_INTERVAL_MS) {
    if (core0_state == CORE0_READY && isDebugEnabled()) {
      // Status reporting preserved but optimized
    }
    last_status_report = millis();
  }
}

/**
 * @brief Reports the status of Core 1.
 *
 * @details This function is a placeholder for reporting the status of Core 1.
 * The implementation has been optimized out in the current version.
 */
void reportCore1Status() {
  static uint32_t last_status_report = 0;
  if (safeMillisElapsed(last_status_report, millis()) >= RUNTIME_STATUS_CHECK_INTERVAL_MS) {
    if ((current_state == STATE_RUNNING || current_state == STATE_CONFIG) && isDebugEnabled()) {
      // Status reporting preserved but optimized
    }
    last_status_report = millis();
  }
}