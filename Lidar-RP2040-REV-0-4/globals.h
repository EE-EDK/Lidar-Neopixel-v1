/**
 * @file globals.h
 * @brief Global declarations for the Lidar-RP2040-REV-0-3 project.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2024-07-16
 *
 * @details This file contains all the global constants, data structures, enumerations,
 * external variable declarations, and function prototypes that are shared across
 * the entire firmware. Centralizing these declarations makes them easier to
 * manage and helps to avoid circular dependencies.
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <LittleFS.h>
#include <pico/multicore.h>
#include <string.h>
#include <stdio.h>
#include <cstdarg>

// ===== SHARED CONSTANTS =====
// Operation Mode Configuration
/** @brief Set to false for 800Hz operation (affects all timing, buffers, and performance parameters) */
#define USE_1000HZ_MODE true

#if USE_1000HZ_MODE
/** @brief LiDAR sample rate - higher = more data but more CPU load */
#define TARGET_FREQUENCY_HZ 1000
/** @brief Circular buffer capacity - larger = more buffering but more RAM usage */
#define FRAME_BUFFER_SIZE 32
/** @brief Buffer fill level to trigger warnings - lower = earlier warnings */
#define BUFFER_WARNING_THRESHOLD 24
/** @brief Buffer fill level for critical state - lower = more conservative */
#define BUFFER_CRITICAL_THRESHOLD 28
/** @brief Max time to wait for frame completion - shorter = faster timeout recovery */
#define FRAME_TIMEOUT_US 3000
#else
/** @brief LiDAR sample rate - lower = less CPU load but reduced temporal resolution */
#define TARGET_FREQUENCY_HZ 800
/** @brief Smaller buffer for lower data rates - reduces RAM usage */
#define FRAME_BUFFER_SIZE 24
/** @brief Proportionally adjusted warning threshold for smaller buffer */
#define BUFFER_WARNING_THRESHOLD 18
/** @brief Proportionally adjusted critical threshold for smaller buffer */
#define BUFFER_CRITICAL_THRESHOLD 21
/** @brief Shorter timeout for faster frame rate recovery */
#define FRAME_TIMEOUT_US 2000
#endif

/** @brief UART speed for LiDAR communication - must match sensor setting or communication fails */
#define LIDAR_BAUD_RATE 460800
/** @brief USB serial speed for debugging - higher = faster output but may cause data loss */
#define DEBUG_BAUD_RATE 115200
/** @brief Time to wait for config commands on startup - shorter = faster boot, longer = more time to connect GUI */
#define CONFIG_MODE_TIMEOUT_MS 15000
/** @brief Minimum signal quality - higher = more reliable but may reject valid weak signals */
#define MIN_STRENGTH_THRESHOLD 200
/** @brief Minimum valid distance measurement - prevents false readings from sensor dead zone */
#define MIN_DISTANCE_CM 7
/** @brief Maximum valid distance measurement - prevents false readings from sensor maximum range */
#define MAX_DISTANCE_CM 1200
/** @brief First frame synchronization byte - must match LiDAR protocol specification */
#define FRAME_SYNC_BYTE1 0x59
/** @brief Second frame synchronization byte - must match LiDAR protocol specification */
#define FRAME_SYNC_BYTE2 0x59
/** @brief LittleFS storage path - change to use different filename for config storage */
#define CONFIG_FILE_PATH "/lidar_config.dat"

/** @brief Limit recovery attempts before giving up */
#define MAX_RECOVERY_ATTEMPTS 3
/** @brief Longer delay between recovery attempts */
#define RECOVERY_ATTEMPT_DELAY_MS 5000

// Non-blocking timing constants
/** @brief Initial system startup delay - shorter = faster boot, longer = more stable initialization */
#define STARTUP_DELAY_MS 1000
/** @brief Delay between LiDAR configuration commands - shorter = faster init, longer = more reliable */
#define LIDAR_INIT_STEP_DELAY_MS 500
/** @brief Final delay before data collection starts - allows sensor to stabilize */
#define LIDAR_FINAL_DELAY_MS 100
/** @brief Delay for command acknowledgment - sensor-dependent timing requirement */
#define COMMAND_RESPONSE_DELAY_MS 50
/** @brief Frequency of debug output - shorter = more frequent updates but more serial traffic */
#define DEBUG_OUTPUT_INTERVAL_MS 150
/** @brief Frequency of status reports - shorter = more monitoring but more CPU overhead */
#define STATUS_CHECK_INTERVAL_MS 5000
/** @brief Frequency of performance statistics - longer intervals provide better averaging */
#define PERFORMANCE_REPORT_INTERVAL_MS 10000
/** @brief Rate limiting for critical error messages - prevents serial spam during failures */
#define CRITICAL_ERROR_REPORT_INTERVAL_MS 2000

/**
 * @brief Enhanced error flags (bitmask values - expanded for better granularity)
 * @{
 */
/** @brief Communication timeout error bit - used in bitmask operations */
#define ERROR_FLAG_COMM_TIMEOUT 0x01
/** @brief Buffer overflow error bit - indicates data loss condition */
#define ERROR_FLAG_BUFFER_OVERFLOW 0x02
/** @brief LiDAR initialization failure bit - hardware or communication problem */
#define ERROR_FLAG_LIDAR_INIT_FAILED 0x04
/** @brief Buffer warning level bit - early indication of processing lag */
#define ERROR_FLAG_BUFFER_WARNING 0x08
/** @brief Buffer critical level bit - imminent data loss warning */
#define ERROR_FLAG_BUFFER_CRITICAL 0x10
/** @brief Frame data corruption detected */
#define ERROR_FLAG_FRAME_CORRUPTION 0x20
/** @brief Velocity calculation error */
#define ERROR_FLAG_VELOCITY_CALC_ERROR 0x40
/** @brief Configuration validation error */
#define ERROR_FLAG_CONFIG_ERROR 0x80
/** @} */

/**
 * @brief Recovery levels for graduated error handling
 * @{
 */
/** @brief No recovery action needed. */
#define RECOVERY_LEVEL_NONE 0
/** @brief Flush the serial buffer to clear stale data. */
#define RECOVERY_LEVEL_BUFFER_FLUSH 1
/** @brief Perform a soft reset of the serial port. */
#define RECOVERY_LEVEL_SOFT_RESET 2
/** @brief Trigger a full re-initialization of the LiDAR sensor. */
#define RECOVERY_LEVEL_FULL_REINIT 3
/** @} */

/**
 * @brief Pin definitions for the RP2040 microcontroller.
 * @{
 */
#define S1_PIN 10
#define S2_PIN 11
#define S4_PIN 12
#define ROTARY_CONN_PIN 13
#define EXT_TRIG_PIN 14
#define EXT_TRIG_EN_PIN 15
#define STATUS_LED_PIN 25
#define TRIG_PULSE_LOW_PIN 16
#define NEOPIXEL_PIN 18
/** @} */

// Debug buffer size for optimized output
#define DEBUG_BUFFER_SIZE 256

// Velocity calculation configuration
#define VELOCITY_DEADBAND_THRESHOLD_CM_S 1.0f
#define DISTANCE_DEADBAND_THRESHOLD_CM 1
#define MPH_TO_CMS 44.704f  // Conversion factor from MPH to cm/s

/**
 * @brief Defines the states for the Core 0 initialization state machine.
 */
enum Core0InitState {
  CORE0_STARTUP,                ///< Initial startup state for Core 0.
  CORE0_SERIAL_INIT_LOW,        ///< Initialize serial at a low baud rate for configuration.
  CORE0_SET_BAUD_RATE,          ///< Send command to set the LiDAR to a higher baud rate.
  CORE0_SAVE_SETTINGS,          ///< Send command to save the new settings to the LiDAR.
  CORE0_BAUD_RATE_WAIT,         ///< Wait for the LiDAR to apply the new baud rate.
  CORE0_SERIAL_INIT_HIGH,       ///< Re-initialize serial at the high baud rate.
  CORE0_LIDAR_STOP,             ///< Send stop command to the LiDAR.
  CORE0_LIDAR_RATE,             ///< Set the LiDAR's measurement frequency.
  CORE0_LIDAR_ENABLE,           ///< Enable the LiDAR's data streaming.
  CORE0_LIDAR_CLEANUP,           ///< Clean up serial buffer before starting data collection.
  CORE0_READY                   ///< Core 0 is ready for data collection.
};

/**
 * @brief Defines the states for the Core 1 initialization state machine.
 */
enum Core1InitState {
  CORE1_STARTUP,                ///< Initial startup state for Core 1.
  CORE1_PINS_INIT,              ///< Initialize GPIO pins for Core 1.
  CORE1_CONFIG_LOAD,            ///< Load configuration from storage.
  CORE1_CONFIG_MODE_CHECK,      ///< Check if the system should enter configuration mode.
  CORE1_READY                   ///< Core 1 is ready.
};

/**
 * @brief Defines the overall system operating states.
 */
enum SystemState {
  STATE_INIT,                   ///< The system is initializing.
  STATE_RUNNING,                ///< The system is in normal operation mode.
  STATE_CONFIG                  ///< The system is in configuration mode.
};

/**
 * @brief Holds the configuration settings for the LiDAR sensor.
 */
struct LidarConfiguration {
  uint16_t distance_thresholds[8];      ///< Distance thresholds for each switch position.
  int16_t velocity_min_thresholds[8];   ///< Minimum velocity thresholds for each switch position.
  int16_t velocity_max_thresholds[8];   ///< Maximum velocity thresholds for each switch position.
  uint8_t trigger_rules[8][4];          ///< Trigger rules for each switch position.
  bool use_velocity_trigger;            ///< Flag to enable or disable velocity-based triggering.
  bool enable_debug;                    ///< Flag to enable or disable debug output.
  uint16_t checksum;                    ///< Checksum to verify the integrity of the configuration.
};

/**
 * @brief Represents a single frame of data from the LiDAR sensor.
 */
struct LidarFrame {
  uint16_t distance;                    ///< The distance measurement in centimeters.
  uint16_t strength;                    ///< The strength of the LiDAR signal.
  uint16_t temperature;                 ///< The internal temperature of the LiDAR sensor.
  uint32_t timestamp;                   ///< The timestamp of when the frame was received.
  bool valid;                           ///< Flag indicating if the frame is valid.
};

/**
 * @brief Structure for communication and data sharing between cores.
 */
struct CoreComm {
  volatile bool lidar_initialized;      ///< Flag indicating if the LiDAR is initialized.
  volatile bool core1_ready;            ///< Flag indicating if Core 1 is ready.
  volatile bool enable_debug;           ///< Flag to enable or disable debug output.
  volatile bool trigger_output;         ///< The current state of the trigger output.
  volatile bool config_mode_active;     ///< Flag indicating if the system is in configuration mode.
  volatile uint8_t switch_code;         ///< The current switch code.
  volatile uint32_t error_flags;        ///< A bitmask of error flags.
  volatile uint32_t frames_received;    ///< The number of frames received from the LiDAR.
  volatile uint32_t frames_processed;   ///< The number of frames processed by Core 1.
  volatile uint32_t dropped_frames;     ///< The number of dropped frames.
  volatile uint32_t last_frame_time;    ///< The timestamp of the last received frame.
  volatile uint32_t performance_counter;///< A counter for performance metrics.
  volatile float velocity;              ///< The calculated velocity.
  volatile uint16_t distance;           ///< The last measured distance.
  volatile uint16_t strength;           ///< The last measured signal strength.
  volatile uint32_t recovery_attempts;  ///< The number of recovery attempts.
};

/**
 * @brief Structure to hold timing information for performance analysis.
 */
struct TimingInfo {
  uint32_t core0_init_start;            ///< The start time of Core 0 initialization.
  uint32_t core0_init_complete;         ///< The completion time of Core 0 initialization.
  uint32_t core1_init_start;            ///< The start time of Core 1 initialization.
  uint32_t core1_init_complete;         ///< The completion time of Core 1 initialization.
  uint32_t lidar_init_start;            ///< The start time of LiDAR initialization.
  uint32_t lidar_init_complete;         ///< The completion time of LiDAR initialization.
  uint32_t last_debug_output;           ///< The timestamp of the last debug output.
  uint32_t last_status_check;           ///< The timestamp of the last status check.
  uint32_t last_performance_report;   ///< The timestamp of the last performance report.
  uint32_t frames_per_second;           ///< The number of frames processed per second.
  uint32_t avg_processing_time_us;    ///< The average processing time per frame in microseconds.
  uint32_t adaptive_timeout_us;         ///< The adaptive timeout for frame reception in microseconds.
};

/**
 * @brief Structure to hold performance metrics.
 */
struct PerformanceMetrics {
  uint32_t max_buffer_utilization;      ///< The maximum buffer utilization.
  uint32_t velocity_calc_errors;        ///< The number of velocity calculation errors.
  uint32_t frame_corruption_count;      ///< The number of frame corruption errors.
  uint32_t recovery_attempt_count;      ///< The number of recovery attempts.
};

/**
 * @brief Extern declarations for global variables.
 * @{
 */
extern LidarFrame frame_buffer[FRAME_BUFFER_SIZE];     ///< Shared buffer for LiDAR frames.
extern volatile uint8_t buffer_head;                   ///< Head index of the circular buffer.
extern volatile uint8_t buffer_tail;                   ///< Tail index of the circular buffer.
extern volatile uint8_t buffer_count;                  ///< Number of frames in the buffer.
extern CoreComm core_comm;                             ///< Shared data between cores.
extern TimingInfo timing_info;                         ///< Timing information for performance monitoring.
extern PerformanceMetrics perf_metrics;               ///< Performance metrics.
extern mutex_t buffer_mutex;                          ///< Mutex for protecting the frame buffer.
extern mutex_t comm_mutex;                            ///< Mutex for protecting the core communication structure.
extern mutex_t serial_mutex;                          ///< Mutex for protecting the serial port.
extern mutex_t perf_mutex;                            ///< Mutex for protecting the performance metrics.
extern Core0InitState core0_state;                     ///< The current state of the Core 0 state machine.
extern uint32_t core0_state_timer;                     ///< Timer for the Core 0 state machine.
extern Core1InitState core1_state;                     ///< The current state of the Core 1 state machine.
extern uint32_t core1_state_timer;                     ///< Timer for the Core 1 state machine.
extern SystemState current_state;                      ///< The overall system state.
extern LidarConfiguration currentConfig;               ///< The current LiDAR configuration.
/** @} */

/**
 * @brief Helper functions for thread-safe operations and other utilities.
 * @{
 */
uint32_t safeMicrosElapsed(uint32_t start, uint32_t current);
uint32_t safeMillisElapsed(uint32_t start, uint32_t current);
void safeSetErrorFlag(uint32_t flag, bool set);
void safeSerialPrintf(const char* format, ...);
void safeSerialPrintfln(const char* format, ...);
void safeSerialPrintln(const String& msg);
void safeSerialPrint(const String& msg);
void safeSetCore1Ready(bool value);
bool safeGetCore1Ready();
void safeSetLidarInitialized(bool value);
void safeIncrementFramesReceived();
void safeIncrementFramesProcessed();
void safeIncrementDroppedFrames();
bool isDebugEnabled();
void updateAdaptiveTimeout(uint32_t observed_frame_rate);
bool atomicBufferPush(const LidarFrame& frame);
bool atomicBufferPop(LidarFrame& frame);
uint8_t getBufferUtilization();
/** @} */

#endif // GLOBALS_H