/**
 * @file globals.cpp
 * @brief This file contains the definitions for global variables and helper functions.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details This file provides the memory allocation for all global variables declared
 * in `globals.h`. It also contains the implementations for various thread-safe
 * helper functions that are used throughout the firmware.
 */

#include "globals.h"

// ===== GLOBAL VARIABLE DEFINITIONS =====
LidarFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint8_t buffer_head = 0;
volatile uint8_t buffer_tail = 0;
volatile uint8_t buffer_count = 0;

CoreComm core_comm = { 
  false,  // lidar_initialized
  false,  // core1_ready
  false,  // enable_debug
  false,  // trigger_output
  false,  // config_mode_active
  0,      // switch_code
  0,      // error_flags
  0,      // frames_received
  0,      // frames_processed
  0,      // dropped_frames
  0,      // last_frame_time
  0,      // performance_counter
  0.0f,   // velocity
  0,      // distance
  0,      // strength
  0       // recovery_attempts
};

TimingInfo timing_info = { 0 };
PerformanceMetrics perf_metrics = { 0 };
mutex_t buffer_mutex;
mutex_t comm_mutex;
mutex_t serial_mutex;
mutex_t perf_mutex;
Core0InitState core0_state = CORE0_STARTUP;
uint32_t core0_state_timer = 0;
Core1InitState core1_state = CORE1_STARTUP;
uint32_t core1_state_timer = 0;
SystemState current_state = STATE_INIT;
LidarConfiguration currentConfig;
static char debug_buffer[DEBUG_BUFFER_SIZE];

/**
 * @brief Thread-safe helper functions.
 * @{
 */

/**
 * @brief Calculates the elapsed time in microseconds, handling timer wraparound.
 * @param start The start time in microseconds.
 * @param current The current time in microseconds.
 * @return The elapsed time in microseconds.
 */
uint32_t safeMicrosElapsed(uint32_t start, uint32_t current) {
  return (current >= start) ? (current - start) : (UINT32_MAX - start + current + 1);
}

/**
 * @brief Calculates the elapsed time in milliseconds, handling timer wraparound.
 * @param start The start time in milliseconds.
 * @param current The current time in milliseconds.
 * @return The elapsed time in milliseconds.
 */
uint32_t safeMillisElapsed(uint32_t start, uint32_t current) {
  return (current >= start) ? (current - start) : (UINT32_MAX - start + current + 1);
}

/**
 * @brief Sets or clears an error flag in a thread-safe manner.
 * @param flag The error flag to set or clear.
 * @param set True to set the flag, false to clear it.
 */
void safeSetErrorFlag(uint32_t flag, bool set) {
  mutex_enter_blocking(&comm_mutex);
  bool was_set = (core_comm.error_flags & flag) != 0;
  if (set) {
    core_comm.error_flags |= flag;
    if (!was_set) {
      mutex_enter_blocking(&perf_mutex);
      if (flag == ERROR_FLAG_FRAME_CORRUPTION) perf_metrics.frame_corruption_count++;
      if (flag == ERROR_FLAG_VELOCITY_CALC_ERROR) perf_metrics.velocity_calc_errors++;
      mutex_exit(&perf_mutex);
    }
  } else {
    core_comm.error_flags &= ~flag;
  }
  mutex_exit(&comm_mutex);
}

/**
 * @brief Prints a formatted string to the serial port in a thread-safe manner.
 * @param format The format string.
 * @param ... The arguments for the format string.
 */
void safeSerialPrintf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  mutex_enter_blocking(&serial_mutex);
  vsnprintf(debug_buffer, DEBUG_BUFFER_SIZE, format, args);
  debug_buffer[DEBUG_BUFFER_SIZE - 1] = '\0';
  Serial.print(debug_buffer);
  mutex_exit(&serial_mutex);
  va_end(args);
}

/**
 * @brief Prints a formatted string followed by a newline to the serial port in a thread-safe manner.
 * @param format The format string.
 * @param ... The arguments for the format string.
 */
void safeSerialPrintfln(const char* format, ...) {
  va_list args;
  va_start(args, format);
  mutex_enter_blocking(&serial_mutex);
  vsnprintf(debug_buffer, DEBUG_BUFFER_SIZE, format, args);
  debug_buffer[DEBUG_BUFFER_SIZE - 1] = '\0';
  Serial.println(debug_buffer);
  mutex_exit(&serial_mutex);
  va_end(args);
}

/**
 * @brief Prints a string followed by a newline to the serial port in a thread-safe manner.
 * @param msg The string to print.
 */
void safeSerialPrintln(const String& msg) {
  mutex_enter_blocking(&serial_mutex);
  Serial.println(msg);
  mutex_exit(&serial_mutex);
}

/**
 * @brief Prints a string to the serial port in a thread-safe manner.
 * @param msg The string to print.
 */
void safeSerialPrint(const String& msg) {
  mutex_enter_blocking(&serial_mutex);
  Serial.print(msg);
  mutex_exit(&serial_mutex);
}

/**
 * @brief Sets the Core 1 ready flag in a thread-safe manner.
 * @param value The value to set the flag to.
 */
void safeSetCore1Ready(bool value) {
  mutex_enter_blocking(&comm_mutex);
  core_comm.core1_ready = value;
  mutex_exit(&comm_mutex);
}

/**
 * @brief Gets the Core 1 ready flag in a thread-safe manner.
 * @return The value of the Core 1 ready flag.
 */
bool safeGetCore1Ready() {
  mutex_enter_blocking(&comm_mutex);
  bool val = core_comm.core1_ready;
  mutex_exit(&comm_mutex);
  return val;
}

/**
 * @brief Sets the LiDAR initialized flag in a thread-safe manner.
 * @param value The value to set the flag to.
 */
void safeSetLidarInitialized(bool value) {
  mutex_enter_blocking(&comm_mutex);
  core_comm.lidar_initialized = value;
  mutex_exit(&comm_mutex);
}

/**
 * @brief Increments the frames received counter in a thread-safe manner.
 */
void safeIncrementFramesReceived() {
  mutex_enter_blocking(&comm_mutex);
  core_comm.frames_received++;
  core_comm.last_frame_time = millis();
  mutex_exit(&comm_mutex);
}

/**
 * @brief Increments the frames processed counter in a thread-safe manner.
 */
void safeIncrementFramesProcessed() {
  mutex_enter_blocking(&comm_mutex);
  core_comm.frames_processed++;
  mutex_exit(&comm_mutex);
}

/**
 * @brief Increments the dropped frames counter in a thread-safe manner.
 */
void safeIncrementDroppedFrames() {
  mutex_enter_blocking(&comm_mutex);
  core_comm.dropped_frames++;
  mutex_exit(&comm_mutex);
  safeSetErrorFlag(ERROR_FLAG_BUFFER_OVERFLOW, true);
}

/**
 * @brief Checks if debug output is enabled in a thread-safe manner.
 * @return True if debug output is enabled, false otherwise.
 */
bool isDebugEnabled() {
  mutex_enter_blocking(&comm_mutex);
  bool debug_enabled = core_comm.enable_debug;
  mutex_exit(&comm_mutex);
  return debug_enabled;
}

/**
 * @brief Updates the adaptive timeout based on the observed frame rate.
 * @param observed_frame_rate The observed frame rate.
 */
void updateAdaptiveTimeout(uint32_t observed_frame_rate) {
  if (observed_frame_rate > 0) {
    timing_info.adaptive_timeout_us = (3000000 / observed_frame_rate);
    if (timing_info.adaptive_timeout_us < 1000) timing_info.adaptive_timeout_us = 1000;
    if (timing_info.adaptive_timeout_us > 10000) timing_info.adaptive_timeout_us = 10000;
  } else {
    timing_info.adaptive_timeout_us = FRAME_TIMEOUT_US;
  }
}
/** @} */

/**
 * @brief Enhanced buffer management functions.
 * @{
 */

/**
 * @brief Pushes a LiDAR frame to the circular buffer in a thread-safe manner.
 * @param frame The LiDAR frame to push.
 * @return True if the frame was pushed successfully, false if the buffer was full.
 */
bool atomicBufferPush(const LidarFrame& frame) {
  bool success = false;
  bool should_set_warning = false;
  bool should_set_critical = false;
  bool should_clear_warnings = false;
  uint8_t current_count = 0;

  mutex_enter_blocking(&buffer_mutex);
  if (buffer_count < FRAME_BUFFER_SIZE) {
    frame_buffer[buffer_head] = frame;
    buffer_head = (buffer_head + 1) % FRAME_BUFFER_SIZE;
    buffer_count++;
    success = true;
    current_count = buffer_count;
    if (buffer_count >= BUFFER_WARNING_THRESHOLD) {
      should_set_warning = true;
      if (buffer_count >= BUFFER_CRITICAL_THRESHOLD) {
        should_set_critical = true;
      }
    } else {
      should_clear_warnings = true;
    }
  }
  mutex_exit(&buffer_mutex);

  if (success) {
    mutex_enter_blocking(&perf_mutex);
    if (current_count > perf_metrics.max_buffer_utilization) {
      perf_metrics.max_buffer_utilization = current_count;
    }
    mutex_exit(&perf_mutex);
    safeIncrementFramesReceived();
    if (should_clear_warnings) {
      safeSetErrorFlag(ERROR_FLAG_BUFFER_WARNING, false);
      safeSetErrorFlag(ERROR_FLAG_BUFFER_CRITICAL, false);
    } else if (should_set_warning) {
      safeSetErrorFlag(ERROR_FLAG_BUFFER_WARNING, true);
      if (should_set_critical) {
        safeSetErrorFlag(ERROR_FLAG_BUFFER_CRITICAL, true);
      }
    }
  } else {
    safeIncrementDroppedFrames();
  }
  return success;
}

/**
 * @brief Pops a LiDAR frame from the circular buffer in a thread-safe manner.
 * @param frame A reference to a LidarFrame object to store the popped frame.
 * @return True if a frame was popped successfully, false if the buffer was empty.
 */
bool atomicBufferPop(LidarFrame& frame) {
  bool success = false;
  mutex_enter_blocking(&buffer_mutex);
  if (buffer_count > 0) {
    frame = frame_buffer[buffer_tail];
    buffer_tail = (buffer_tail + 1) % FRAME_BUFFER_SIZE;
    buffer_count--;
    success = true;
    if (buffer_count < BUFFER_WARNING_THRESHOLD) {
      safeSetErrorFlag(ERROR_FLAG_BUFFER_WARNING, false);
      safeSetErrorFlag(ERROR_FLAG_BUFFER_CRITICAL, false);
    }
  }
  mutex_exit(&buffer_mutex);
  if (success) {
    safeIncrementFramesProcessed();
  }
  return success;
}

/**
 * @brief Gets the number of frames currently in the buffer in a thread-safe manner.
 * @return The number of frames in the buffer.
 */
uint8_t getBufferUtilization() {
  mutex_enter_blocking(&buffer_mutex);
  uint8_t count = buffer_count;
  mutex_exit(&buffer_mutex);
  return count;
}
/** @} */