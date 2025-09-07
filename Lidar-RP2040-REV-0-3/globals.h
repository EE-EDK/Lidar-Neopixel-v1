/**
 * @file globals.h
 * @brief Global constants, structures, and function prototypes
 * @version 6.3.2
 * @date September 06, 2025 
 * @revision Rev 2 - Added config_mode_active flag for health monitoring control
 * @changes:
 *   - Added config_mode_active to CoreComm struct
 *   - Enables CONFIG mode isolation from health monitoring
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
#define USE_1000HZ_MODE true  ///< Set to false for 800Hz operation (affects all timing, buffers, and performance parameters)

#if USE_1000HZ_MODE
#define TARGET_FREQUENCY_HZ 1000        ///< LiDAR sample rate - higher = more data but more CPU load
#define FRAME_BUFFER_SIZE 32            ///< Circular buffer capacity - larger = more buffering but more RAM usage
#define BUFFER_WARNING_THRESHOLD 24     ///< Buffer fill level to trigger warnings - lower = earlier warnings
#define BUFFER_CRITICAL_THRESHOLD 28    ///< Buffer fill level for critical state - lower = more conservative
#define FRAME_TIMEOUT_US 3000           ///< Max time to wait for frame completion - shorter = faster timeout recovery
#else
#define TARGET_FREQUENCY_HZ 800         ///< LiDAR sample rate - lower = less CPU load but reduced temporal resolution
#define FRAME_BUFFER_SIZE 24            ///< Smaller buffer for lower data rates - reduces RAM usage
#define BUFFER_WARNING_THRESHOLD 18     ///< Proportionally adjusted warning threshold for smaller buffer
#define BUFFER_CRITICAL_THRESHOLD 21    ///< Proportionally adjusted critical threshold for smaller buffer
#define FRAME_TIMEOUT_US 2000           ///< Shorter timeout for faster frame rate recovery
#endif

#define LIDAR_BAUD_RATE 460800          ///< UART speed for LiDAR communication - must match sensor setting or communication fails
#define DEBUG_BAUD_RATE 115200          ///< USB serial speed for debugging - higher = faster output but may cause data loss
#define CONFIG_MODE_TIMEOUT_MS 15000    ///< Time to wait for config commands on startup - shorter = faster boot, longer = more time to connect GUI
#define MIN_STRENGTH_THRESHOLD 200      ///< Minimum signal quality - higher = more reliable but may reject valid weak signals
#define MIN_DISTANCE_CM 7               ///< Minimum valid distance measurement - prevents false readings from sensor dead zone
#define MAX_DISTANCE_CM 1200            ///< Maximum valid distance measurement - prevents false readings from sensor maximum range
#define FRAME_SYNC_BYTE1 0x59           ///< First frame synchronization byte - must match LiDAR protocol specification
#define FRAME_SYNC_BYTE2 0x59           ///< Second frame synchronization byte - must match LiDAR protocol specification
#define CONFIG_FILE_PATH "/lidar_config.dat" ///< LittleFS storage path - change to use different filename for config storage

#define MAX_RECOVERY_ATTEMPTS 3        // Limit recovery attempts before giving up
#define RECOVERY_ATTEMPT_DELAY_MS 5000  // Longer delay between recovery attempts

// Non-blocking timing constants
#define STARTUP_DELAY_MS 1000           ///< Initial system startup delay - shorter = faster boot, longer = more stable initialization
#define LIDAR_INIT_STEP_DELAY_MS 500    ///< Delay between LiDAR configuration commands - shorter = faster init, longer = more reliable
#define LIDAR_FINAL_DELAY_MS 100        ///< Final delay before data collection starts - allows sensor to stabilize
#define COMMAND_RESPONSE_DELAY_MS 50    ///< Delay for command acknowledgment - sensor-dependent timing requirement
#define DEBUG_OUTPUT_INTERVAL_MS 150    ///< Frequency of debug output - shorter = more frequent updates but more serial traffic
#define STATUS_CHECK_INTERVAL_MS 5000   ///< Frequency of status reports - shorter = more monitoring but more CPU overhead
#define PERFORMANCE_REPORT_INTERVAL_MS 10000  ///< Frequency of performance statistics - longer intervals provide better averaging
#define CRITICAL_ERROR_REPORT_INTERVAL_MS 2000 ///< Rate limiting for critical error messages - prevents serial spam during failures

// Enhanced error flags (bitmask values - expanded for better granularity)
#define ERROR_FLAG_COMM_TIMEOUT 0x01    ///< Communication timeout error bit - used in bitmask operations
#define ERROR_FLAG_BUFFER_OVERFLOW 0x02 ///< Buffer overflow error bit - indicates data loss condition
#define ERROR_FLAG_LIDAR_INIT_FAILED 0x04 ///< LiDAR initialization failure bit - hardware or communication problem
#define ERROR_FLAG_BUFFER_WARNING 0x08  ///< Buffer warning level bit - early indication of processing lag
#define ERROR_FLAG_BUFFER_CRITICAL 0x10 ///< Buffer critical level bit - imminent data loss warning
#define ERROR_FLAG_FRAME_CORRUPTION 0x20 ///< Frame data corruption detected
#define ERROR_FLAG_VELOCITY_CALC_ERROR 0x40 ///< Velocity calculation error
#define ERROR_FLAG_CONFIG_ERROR 0x80     ///< Configuration validation error

// Recovery levels for graduated error handling
#define RECOVERY_LEVEL_NONE 0
#define RECOVERY_LEVEL_BUFFER_FLUSH 1
#define RECOVERY_LEVEL_SOFT_RESET 2
#define RECOVERY_LEVEL_FULL_REINIT 3

// Pin definitions
#define S1_PIN 10
#define S2_PIN 11
#define S4_PIN 12
#define ROTARY_CONN_PIN 13
#define EXT_TRIG_PIN 14
#define EXT_TRIG_EN_PIN 15
#define STATUS_LED_PIN 25
#define TRIG_PULSE_LOW_PIN 16
#define NEOPIXEL_PIN 18

// Debug buffer size for optimized output
#define DEBUG_BUFFER_SIZE 256

// Velocity calculation configuration
#define VELOCITY_DEADBAND_THRESHOLD_CM_S 1.0f
#define DISTANCE_DEADBAND_THRESHOLD_CM 1
#define MPH_TO_CMS 44.704f  // Conversion factor from MPH to cm/s

// ===== INITIALIZATION STATE MACHINES =====
enum Core0InitState {
  CORE0_STARTUP, CORE0_SERIAL_INIT_LOW, CORE0_SET_BAUD_RATE, CORE0_SAVE_SETTINGS,
  CORE0_BAUD_RATE_WAIT, CORE0_SERIAL_INIT_HIGH, CORE0_LIDAR_STOP, CORE0_LIDAR_RATE,
  CORE0_LIDAR_ENABLE, CORE0_LIDAR_CLEANUP, CORE0_READY
};

enum Core1InitState {
  CORE1_STARTUP, CORE1_PINS_INIT, CORE1_CONFIG_LOAD, CORE1_CONFIG_MODE_CHECK, CORE1_READY
};

enum SystemState { STATE_INIT, STATE_RUNNING, STATE_CONFIG };

// ===== CONFIGURATION & COMMUNICATION STRUCTURES =====
struct LidarConfiguration {
  uint16_t distance_thresholds[8];
  int16_t velocity_min_thresholds[8];
  int16_t velocity_max_thresholds[8];
  uint8_t trigger_rules[8][4];
  bool use_velocity_trigger;
  bool enable_debug;
  uint16_t checksum;
};

struct LidarFrame {
  uint16_t distance;
  uint16_t strength;
  uint16_t temperature;
  uint32_t timestamp;
  bool valid;
};

struct CoreComm {
  volatile bool lidar_initialized;
  volatile bool core1_ready;
  volatile bool enable_debug;
  volatile bool trigger_output;
  volatile bool config_mode_active;  // REV 2: Added for health monitoring control
  volatile uint8_t switch_code;
  volatile uint32_t error_flags;
  volatile uint32_t frames_received;
  volatile uint32_t frames_processed;
  volatile uint32_t dropped_frames;
  volatile uint32_t last_frame_time;
  volatile uint32_t performance_counter;
  volatile float velocity;
  volatile uint16_t distance;
  volatile uint16_t strength;
  volatile uint32_t recovery_attempts;
};

struct TimingInfo {
  uint32_t core0_init_start;
  uint32_t core0_init_complete;
  uint32_t core1_init_start;
  uint32_t core1_init_complete;
  uint32_t lidar_init_start;
  uint32_t lidar_init_complete;
  uint32_t last_debug_output;
  uint32_t last_status_check;
  uint32_t last_performance_report;
  uint32_t frames_per_second;
  uint32_t avg_processing_time_us;
  uint32_t adaptive_timeout_us;
};

struct PerformanceMetrics {
  uint32_t max_buffer_utilization;
  uint32_t velocity_calc_errors;
  uint32_t frame_corruption_count;
  uint32_t recovery_attempt_count;
};

// ===== EXTERN GLOBAL VARIABLES =====
extern LidarFrame frame_buffer[FRAME_BUFFER_SIZE];
extern volatile uint8_t buffer_head;
extern volatile uint8_t buffer_tail;
extern volatile uint8_t buffer_count;
extern CoreComm core_comm;
extern TimingInfo timing_info;
extern PerformanceMetrics perf_metrics;
extern mutex_t buffer_mutex;
extern mutex_t comm_mutex;
extern mutex_t serial_mutex;
extern mutex_t perf_mutex;
extern Core0InitState core0_state;
extern uint32_t core0_state_timer;
extern Core1InitState core1_state;
extern uint32_t core1_state_timer;
extern SystemState current_state;
extern LidarConfiguration currentConfig;

// ===== HELPER FUNCTION PROTOTYPES =====
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

#endif // GLOBALS_H