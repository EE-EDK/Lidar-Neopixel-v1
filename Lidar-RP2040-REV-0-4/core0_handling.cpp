/**
 * @file core0_handling.cpp
 * @brief This file contains the implementation for functions that handle Core 0 operations.
 * @author The Lidar-RP2040-REV-0-4 Team
 * @version 1.0
 * @date 2025-09-07
 *
 * @details The functions in this file are responsible for managing the main loop of Core 0,
 * which includes processing the Core 0 state machine, handling LiDAR serial data,
 * and performing health checks and recovery operations for the LiDAR sensor.
 * Updated to support runtime global configuration parameters.
 */

#include "core0_handling.h"
#include "globals.h"
#include "globals_config.h"  // NEW: Include runtime globals support
#include "status.h"

/**
 * @brief Main handler for the Core 0 loop.
 *
 * @details This function is called repeatedly in the main loop of Core 0. It is responsible for
 * orchestrating the tasks performed by Core 0. It processes the Core 0 state machine,
 * and if Core 1 is ready and Core 0 is in the ready state, it processes incoming
 * LiDAR serial data. It also periodically reports the status of Core 0.
 */
void loop0_handler() {
  processCore0StateMachine();
  if (safeGetCore1Ready() && core0_state == CORE0_READY) {
    processLidarSerial();
  }

  static uint32_t last_status_report = 0;
  if (safeMillisElapsed(last_status_report, millis()) >= RUNTIME_STATUS_CHECK_INTERVAL_MS) {
    reportCore0Status();
    last_status_report = millis();
  }

  yield();
}

/**
 * @brief Processes the state machine for Core 0.
 *
 * @details This function manages the different states of Core 0. It handles the lifecycle
 * of the LiDAR sensor, from startup and initialization to ready state and health
 * monitoring. The state machine transitions through various stages to configure
 * the LiDAR sensor, including setting the baud rate and enabling the data stream.
 * Once the sensor is operational, it monitors for communication timeouts and
 * triggers recovery mechanisms if necessary.
 */
void processCore0StateMachine() {
    uint32_t current_time = millis();
    switch (core0_state) {
    case CORE0_STARTUP:
      if (safeMillisElapsed(core0_state_timer, current_time) >= RUNTIME_STARTUP_DELAY_MS) {
        if (isDebugEnabled()) safeSerialPrintln("Core 0: Startup delay complete. Initializing serial at 115200 baud to configure sensor...");
        core0_state = CORE0_SERIAL_INIT_LOW;
        core0_state_timer = current_time;
      }
      break;
    case CORE0_SERIAL_INIT_LOW:
      {
        Serial1.begin(115200);
        if (isDebugEnabled()) safeSerialPrintln("Core 0: Serial1 at 115200. Sending baud rate change command...");
        core0_state = CORE0_SET_BAUD_RATE;
        core0_state_timer = current_time;
        break;
      }

    case CORE0_SET_BAUD_RATE:
      {
        uint8_t setBaudCmd[] = { 0x5A, 0x08, 0x06, 0x00, 0x08, 0x07, 0x00, 0x77 };
        Serial1.write(setBaudCmd, sizeof(setBaudCmd));
        if (isDebugEnabled()) safeSerialPrintln("Core 0: Baud rate command sent. Sending save settings command...");
        core0_state = CORE0_SAVE_SETTINGS;
        core0_state_timer = current_time;
        break;
      }

    case CORE0_SAVE_SETTINGS:
      {
        if (safeMillisElapsed(core0_state_timer, current_time) >= 100) { 
          uint8_t saveCmd[] = { 0x5A, 0x04, 0x11, 0x6F };
          Serial1.write(saveCmd, sizeof(saveCmd));
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Save settings command sent. Waiting for sensor to apply...");
          core0_state = CORE0_BAUD_RATE_WAIT;
          core0_state_timer = current_time;
        }
        break;
      }

    case CORE0_BAUD_RATE_WAIT:
      {
        if (safeMillisElapsed(core0_state_timer, current_time) >= 1000) { 
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Wait complete. Re-initializing serial at 460800 baud...");
          core0_state = CORE0_SERIAL_INIT_HIGH;
          core0_state_timer = current_time;
        }
        break;
      }

    case CORE0_SERIAL_INIT_HIGH:
      {
        Serial1.begin(LIDAR_BAUD_RATE);
        timing_info.lidar_init_start = current_time;
        if (isDebugEnabled()) {
          safeSerialPrintfln("Core 0: Serial1 re-initialized at %d baud", LIDAR_BAUD_RATE);
        }
        
        if (isDebugEnabled()) safeSerialPrintln("Core 0: Sending LiDAR stop command...");
        uint8_t stopCmd[] = { 0x5A, 0x05, 0x07, 0x00, 0x66 };
        Serial1.write(stopCmd, sizeof(stopCmd));

        core0_state = CORE0_LIDAR_STOP;
        core0_state_timer = current_time;
        break;
      }

    case CORE0_LIDAR_STOP:
      {
        if (safeMillisElapsed(core0_state_timer, current_time) >= RUNTIME_LIDAR_INIT_STEP_DELAY_MS) {
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Stop command delay complete, setting frequency...");
          #if USE_1000HZ_MODE
          uint8_t rateCmd[] = { 0x5A, 0x06, 0x03, 0xE8, 0x03, 0x4E };
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Setting 1000Hz mode");
          #else
          uint8_t rateCmd[] = { 0x5A, 0x06, 0x03, 0x20, 0x03, 0x86 };
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Setting 800Hz mode");
          #endif

          Serial1.write(rateCmd, sizeof(rateCmd));
          core0_state = CORE0_LIDAR_RATE;
          core0_state_timer = current_time;
        }
        break;
      }

    case CORE0_LIDAR_RATE:
      {
        if (safeMillisElapsed(core0_state_timer, current_time) >= RUNTIME_LIDAR_INIT_STEP_DELAY_MS) {
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Frequency command delay complete, enabling LiDAR...");
          uint8_t enableCmd[] = { 0x5A, 0x05, 0x07, 0x01, 0x67 };
          Serial1.write(enableCmd, sizeof(enableCmd));

          core0_state = CORE0_LIDAR_ENABLE;
          core0_state_timer = current_time;
        }
        break;
      }

    case CORE0_LIDAR_ENABLE:
      {
        if (safeMillisElapsed(core0_state_timer, current_time) >= RUNTIME_LIDAR_FINAL_DELAY_MS) {
          if (isDebugEnabled()) safeSerialPrintln("Core 0: Enable command delay complete, clearing buffers...");
          while (Serial1.available()) Serial1.read();

          core0_state = CORE0_LIDAR_CLEANUP;
          core0_state_timer = current_time;
        }
        break;
      }

    case CORE0_LIDAR_CLEANUP:
      {
        timing_info.lidar_init_complete = current_time;
        timing_info.core0_init_complete = current_time;

        if (isDebugEnabled()) {
          safeSerialPrintfln("Core 0: LiDAR initialization complete in %lu ms", 
            timing_info.lidar_init_complete - timing_info.lidar_init_start);
          safeSerialPrintfln("Core 0: Total Core 0 initialization time: %lu ms",
            timing_info.core0_init_complete - timing_info.core0_init_start);
        }
        
        mutex_enter_blocking(&comm_mutex);
        core_comm.last_frame_time = current_time; // Set initial time here
        core_comm.recovery_attempts = 0;
        mutex_exit(&comm_mutex);

        safeSetLidarInitialized(true);
        core0_state = CORE0_READY;
        if (isDebugEnabled()) safeSerialPrintln("Core 0: Ready for data collection");
        break;
      }

    case CORE0_READY:
      {
        // System ready - handle health monitoring
        static bool system_fully_ready = false; 
        if (safeGetCore1Ready()) {
          
          // This block runs only once, when Core 1 signals it's ready.
          if (!system_fully_ready) {
            system_fully_ready = true;
            // Reset the grace period timer now that the whole system is operational.
            mutex_enter_blocking(&comm_mutex);
            core_comm.last_frame_time = millis(); 
            mutex_exit(&comm_mutex);
            if (isDebugEnabled()) {
                safeSerialPrintln("Core 0: System fully operational. Starting communication health monitor.");
            }
          }

          // REV 2: Skip health monitoring if config mode is active
          bool config_active = false;
          mutex_enter_blocking(&comm_mutex);
          config_active = core_comm.config_mode_active;
          mutex_exit(&comm_mutex);

          if (!config_active) {
            static uint32_t last_health_check = 0;
            if (safeMillisElapsed(last_health_check, current_time) > 5000) {
              uint32_t last_frame_ms;
              uint32_t recovery_attempts;
              
              mutex_enter_blocking(&comm_mutex);
              last_frame_ms = core_comm.last_frame_time;
              recovery_attempts = core_comm.recovery_attempts;
              mutex_exit(&comm_mutex);

              uint32_t comm_timeout = safeMillisElapsed(last_frame_ms, current_time);
              if (comm_timeout > 2000) {
                if (recovery_attempts == 0) {
                  safeSerialPrintfln("Core 0: Communication timeout %lu ms, attempting buffer flush", comm_timeout);
                  attemptRecovery(RECOVERY_LEVEL_BUFFER_FLUSH);
                  safeSetErrorFlag(ERROR_FLAG_COMM_TIMEOUT, true);
                } else if (recovery_attempts == 1) {
                  safeSerialPrintfln("Core 0: Communication timeout %lu ms, attempting soft reset", comm_timeout);
                  attemptRecovery(RECOVERY_LEVEL_SOFT_RESET);
                  safeSetErrorFlag(ERROR_FLAG_COMM_TIMEOUT, true);
                } else if (recovery_attempts >= 2) {
                  safeSerialPrintfln("Core 0: CRITICAL - Communication lost for %lu ms, full reinitialization", comm_timeout);
                  if (attemptRecovery(RECOVERY_LEVEL_FULL_REINIT)) {
                    core0_state = CORE0_STARTUP;
                    core0_state_timer = current_time;
                    safeSetErrorFlag(ERROR_FLAG_COMM_TIMEOUT, true);
                    system_fully_ready = false; // Reset for re-initialization
                  }
                }
              }
              last_health_check = current_time;
            }
          } else {
            // Config mode active - skip health monitoring
            if (isDebugEnabled()) {
              static uint32_t last_config_notice = 0;
              if (safeMillisElapsed(last_config_notice, current_time) > 30000) { // Every 30 seconds
                safeSerialPrintln("Core 0: Config mode active - health monitoring suspended");
                last_config_notice = current_time;
              }
            }
          }
        }
        break;
      }
    }
}

/**
 * @brief Processes incoming serial data from the LiDAR sensor.
 *
 * @details This function is responsible for reading and parsing the data stream from the LiDAR
 * sensor. It synchronizes with the data frames by looking for a specific byte
 * pattern. Once a frame is received, it validates the checksum and then extracts
 * the distance, strength, and temperature information. Valid frames are pushed
 * into a shared buffer for Core 1 to process. The function also includes
 * performance monitoring and error handling, such as detecting frame corruption
 * and timeouts.
 */
void processLidarSerial() {
  static uint8_t sync_state = 0, frame_data[9], frame_index = 0;
  static uint32_t frame_start_time = 0, last_frame_debug = 0;
  static uint32_t valid_frames = 0, invalid_frames = 0;
  static uint32_t last_raw_data_debug = 0;
  static uint32_t last_health_check = 0;
  static uint32_t consecutive_sync_failures = 0;

  uint32_t current_time = millis();

  // Periodic health check if no frames are being processed
  if (safeMillisElapsed(last_health_check, current_time) > 10000) { // Every 10 seconds
    if (valid_frames == 0 && invalid_frames == 0) {
      if (isDebugEnabled()) {
        safeSerialPrintln("Core 0: No frames processed recently, performing health check");
      }
      checkLidarSensorHealth();
    }
    last_health_check = current_time;
  }

  // Enhanced raw data monitoring (without resetting counters)
  if (isDebugEnabled() && safeMillisElapsed(last_raw_data_debug, current_time) > 10000) {
    //safeSerialPrintfln("Core 0: Serial status - Available: %d, Sync state: %d, Valid frames: %lu, Invalid: %lu", 
     // Serial1.available(), sync_state, valid_frames, invalid_frames);
    last_raw_data_debug = current_time;
    // Note: Do NOT reset frame counters here - let the performance section handle it
  }

  // Look for frame sync
  if (sync_state == 0 && Serial1.available() >= 2) {
    uint8_t first_byte = Serial1.read();
    if (first_byte == FRAME_SYNC_BYTE1 && Serial1.peek() == FRAME_SYNC_BYTE2) {
      frame_data[0] = FRAME_SYNC_BYTE1;
      frame_data[1] = Serial1.read();
      frame_index = 2;
      frame_start_time = micros();
      sync_state = 1;
      consecutive_sync_failures = 0; // Reset failure counter
      
      if (isDebugEnabled() && safeMillisElapsed(last_frame_debug, current_time) > 5000) {
        //safeSerialPrintln("Core 0: Frame sync found, reading frame data");
        last_frame_debug = current_time;
      }
    } else {
      consecutive_sync_failures++;
      
      // Log sync issues periodically
      if (isDebugEnabled() && consecutive_sync_failures % 100 == 0) {
        safeSerialPrintfln("Core 0: Sync failure #%lu - Expected: 0x%02X, Got: 0x%02X", 
          consecutive_sync_failures, FRAME_SYNC_BYTE1, first_byte);
      }
      
      // If too many sync failures, perform health check
      if (consecutive_sync_failures > 1000) {
        safeSerialPrintln("Core 0: Too many sync failures, performing emergency health check");
        checkLidarSensorHealth();
        consecutive_sync_failures = 0;
      }
    }
  }

  // Read frame data
  while (Serial1.available() && sync_state == 1 && frame_index < 9) {
    frame_data[frame_index++] = Serial1.read();
    
    if (frame_index >= 9) {
      sync_state = 0;
      
      // Calculate and verify checksum
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) {
        checksum += frame_data[i];
      }
      
      if (checksum == frame_data[8]) {
        // Checksum valid - clear error flags
        safeSetErrorFlag(ERROR_FLAG_FRAME_CORRUPTION, false);
        static uint32_t consecutive_good_frames = 0;
        consecutive_good_frames++;
        
        // Clear communication timeout after several good frames
        safeSetErrorFlag(ERROR_FLAG_COMM_TIMEOUT, false);
        if (consecutive_good_frames >= 5) {
          mutex_enter_blocking(&comm_mutex);
          core_comm.recovery_attempts = 0;
          mutex_exit(&comm_mutex);
          consecutive_good_frames = 0;
        }
        
        // Parse frame data
        LidarFrame new_frame;
        new_frame.distance = frame_data[2] | (frame_data[3] << 8);
        new_frame.strength = frame_data[4] | (frame_data[5] << 8);
        new_frame.temperature = frame_data[6] | (frame_data[7] << 8);

        // Enhanced frame validation and debugging
        if (isDebugEnabled() && safeMillisElapsed(last_frame_debug, current_time) > 5000) {
          //safeSerialPrintfln("Core 0: Raw frame data - Dist: %d cm, Strength: %d, Temp: %d", 
           // new_frame.distance, new_frame.strength, new_frame.temperature);
          
          // Show raw bytes for debugging
          //safeSerialPrint("Core 0: Raw bytes: ");
          for (int i = 0; i < 9; i++) {
            //safeSerialPrintf("0x%02X ", frame_data[i]);
          }
          //safeSerialPrintln("");
          last_frame_debug = current_time;
        }

        // Validate frame data ranges - USE RUNTIME GLOBAL for strength threshold
        if (new_frame.distance >= MIN_DISTANCE_CM && new_frame.distance <= MAX_DISTANCE_CM &&
            new_frame.strength >= RUNTIME_MIN_STRENGTH_THRESHOLD) {
          
          new_frame.timestamp = micros();
          new_frame.valid = true;
          valid_frames++;

          // Try to add frame to buffer
          if (!atomicBufferPush(new_frame)) {
            // REV 2: Suppress buffer overflow messages during config mode
            bool config_active = false;
            mutex_enter_blocking(&comm_mutex);
            config_active = core_comm.config_mode_active;
            mutex_exit(&comm_mutex);

            if (!config_active) {
              static uint32_t last_overflow_report = 0;
              if (safeMillisElapsed(last_overflow_report, current_time) > RUNTIME_CRITICAL_ERROR_REPORT_INTERVAL_MS) {
                safeSerialPrintfln("Core 0: CRITICAL - Buffer overflow! Dropping frames (util: %d/%d)", 
                  getBufferUtilization(), FRAME_BUFFER_SIZE);
                last_overflow_report = current_time;
              }
            }
          } else {
            // Update communication timestamp on successful frame processing
            mutex_enter_blocking(&comm_mutex);
            core_comm.last_frame_time = current_time;
            mutex_exit(&comm_mutex);
          }

        } else {
          // Frame data out of valid range
          invalid_frames++;
          safeSetErrorFlag(ERROR_FLAG_FRAME_CORRUPTION, true);
          
          if (isDebugEnabled()) {
            safeSerialPrintfln("Core 0: Frame validation failed - Dist: %d (range: %d-%d), Strength: %d (min: %d)", 
              new_frame.distance, MIN_DISTANCE_CM, MAX_DISTANCE_CM, 
              new_frame.strength, RUNTIME_MIN_STRENGTH_THRESHOLD);
          }
          
          // Reset consecutive good frames counter
          static uint32_t* consecutive_good_frames_ptr = nullptr;
          if (consecutive_good_frames_ptr == nullptr) {
            static uint32_t consecutive_good_frames_static = 0;
            consecutive_good_frames_ptr = &consecutive_good_frames_static;
          }
          *consecutive_good_frames_ptr = 0;
        }
      } else {
        // Checksum failed
        invalid_frames++;
        safeSetErrorFlag(ERROR_FLAG_FRAME_CORRUPTION, true);
        
        if (isDebugEnabled()) {
          safeSerialPrintfln("Core 0: Checksum mismatch - Calculated: 0x%02X, Received: 0x%02X", 
            checksum, frame_data[8]);
          
          // Show the problematic frame data
          safeSerialPrint("Core 0: Bad frame data: ");
          for (int i = 0; i < 9; i++) {
            safeSerialPrintf("0x%02X ", frame_data[i]);
          }
          safeSerialPrintln("");
        }
        
        // Reset consecutive good frames counter
        static uint32_t* consecutive_good_frames_ptr = nullptr;
        if (consecutive_good_frames_ptr == nullptr) {
          static uint32_t consecutive_good_frames_static = 0;
          consecutive_good_frames_ptr = &consecutive_good_frames_static;
        }
        *consecutive_good_frames_ptr = 0;
      }
    }
  }

  // Handle frame timeout
  if (sync_state > 0 && safeMicrosElapsed(frame_start_time, micros()) > timing_info.adaptive_timeout_us) {
    sync_state = 0;
    static uint32_t last_timeout_report = 0;
    
    if (isDebugEnabled() && safeMillisElapsed(last_timeout_report, current_time) > 5000) {
      safeSerialPrintfln("Core 0: Frame timeout after %lu microseconds (partial frame, index: %d)", 
        timing_info.adaptive_timeout_us, frame_index);
      last_timeout_report = current_time;
    }
  }

  // Update performance metrics (FIXED VERSION)
  static uint32_t last_perf_update = 0;
  if (safeMillisElapsed(last_perf_update, current_time) > 1000) {
    uint32_t total_frames = valid_frames + invalid_frames;
    
    if (total_frames > 0) {
      timing_info.frames_per_second = total_frames;
      updateAdaptiveTimeout(timing_info.frames_per_second);
      
      if (isDebugEnabled()) {
        //safeSerialPrintfln("Core 0: Performance - %lu fps (%lu valid, %lu invalid), Timeout: %lu us", 
         // total_frames, valid_frames, invalid_frames, timing_info.adaptive_timeout_us);
      }
      
      // Reset frame counters AFTER checking and reporting
      valid_frames = invalid_frames = 0;
    } else {
      // No frames processed - potential problem
      if (isDebugEnabled()) {
        safeSerialPrintln("Core 0: WARNING - No frames processed in last second");
      }
    }
    
    last_perf_update = current_time;
  }
}

/**
 * @brief Attempts to recover the LiDAR sensor from an error state.
 *
 * @details This function is called when a communication timeout or other critical error is
 * detected. It implements a multi-level recovery strategy. Depending on the
 * `recovery_level` parameter, it can perform a buffer flush, a soft reset of
 * the serial port, or trigger a full re-initialization of the LiDAR sensor.
 * The function is designed to be non-blocking and will only attempt recovery
 * periodically to avoid flooding the system with recovery attempts.
 *
 * @param recovery_level The level of recovery to attempt. This can be
 *                       `RECOVERY_LEVEL_BUFFER_FLUSH`, `RECOVERY_LEVEL_SOFT_RESET`,
 *                       or `RECOVERY_LEVEL_FULL_REINIT`.
 * @return True if a recovery action was taken or triggered, false otherwise.
 */
bool attemptRecovery(uint8_t recovery_level) {
  static uint32_t last_recovery_attempt = 0;
  uint32_t current_time = millis();
  uint32_t current_attempts = 0;

  if (safeMillisElapsed(last_recovery_attempt, current_time) < RUNTIME_RECOVERY_ATTEMPT_DELAY_MS) {
    return false;
  }

  mutex_enter_blocking(&comm_mutex);
  core_comm.recovery_attempts++;
  current_attempts = core_comm.recovery_attempts;
  mutex_exit(&comm_mutex);
  
  switch (recovery_level) {
    case RECOVERY_LEVEL_BUFFER_FLUSH:
      mutex_enter_blocking(&buffer_mutex);
      buffer_head = buffer_tail = buffer_count = 0;
      mutex_exit(&buffer_mutex);
      while (Serial1.available()) Serial1.read();
      
      // Add sensor health check
      if (!checkLidarSensorHealth()) {
        safeSerialPrintln("Core 0: WARNING - LiDAR sensor not responding to health check");
      }
      
      safeSerialPrintfln("Core 0: Recovery Level 1 - Buffer flush completed (attempt %lu)", current_attempts);
      break;
      
    case RECOVERY_LEVEL_SOFT_RESET:
      Serial1.end();
      delay(500); // Longer delay
      Serial1.begin(LIDAR_BAUD_RATE);
      delay(200);
      
      if (!checkLidarSensorHealth()) {
        safeSerialPrintln("Core 0: ERROR - LiDAR sensor still not responding after soft reset");
      }
      
      safeSerialPrintfln("Core 0: Recovery Level 2 - Soft reset completed (attempt %lu)", current_attempts);
      break;
      
    case RECOVERY_LEVEL_FULL_REINIT:
      safeSerialPrintfln("Core 0: Recovery Level 3 - Full reinitialization triggered (attempt %lu)", current_attempts);
      
      // Reset recovery counter to prevent infinite reinit loops
      mutex_enter_blocking(&comm_mutex);
      if (core_comm.recovery_attempts > RUNTIME_MAX_RECOVERY_ATTEMPTS) {
        safeSerialPrintln("Core 0: CRITICAL - Too many recovery attempts, system may be unstable");
        core_comm.recovery_attempts = 0; // Reset to prevent continuous reinit
        mutex_exit(&comm_mutex);
        return false; // Don't trigger reinit
      }
      mutex_exit(&comm_mutex);
      
      return true;
      
    default:
      return false;
  }

  last_recovery_attempt = current_time;
  
  mutex_enter_blocking(&perf_mutex);
  perf_metrics.recovery_attempt_count++;
  mutex_exit(&perf_mutex);
  
  return true;
}

/**
 * @brief Checks the health of the LiDAR sensor.
 *
 * @details This function performs a basic health check of the LiDAR sensor by checking if
 * there is any data available in the serial buffer. In a healthy state, the
 * sensor should be continuously streaming data. If the buffer is empty, it
 * may indicate a problem with the sensor or the connection. This function
 * provides a non-intrusive way to monitor the sensor's status without sending
 * any commands that might interrupt the data flow.
 *
 * @return True if the sensor is considered healthy (i.e., data is available),
 *         false otherwise.
 */
bool checkLidarSensorHealth() {
  if (isDebugEnabled()) {
    safeSerialPrintln("Core 0: Checking sensor health by monitoring data stream...");
  }
  
  // Instead of sending commands, just check if we're receiving data
  int available = Serial1.available();
  
  if (isDebugEnabled()) {
    safeSerialPrintfln("Core 0: LiDAR health check - %d bytes in buffer", available);
  }
  
  // Sensor is healthy if there's data in the buffer or we've received frames recently
  bool sensor_healthy = available > 0;
  
  if (isDebugEnabled()) {
    safeSerialPrintfln("Core 0: LiDAR health check result: %s", 
      sensor_healthy ? "HEALTHY (streaming data)" : "NO DATA");
  }
  
  return sensor_healthy;
}