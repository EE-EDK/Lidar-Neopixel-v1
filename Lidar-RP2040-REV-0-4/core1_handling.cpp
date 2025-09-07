/**
 * @file core1_handling.cpp
 * @brief This file contains the implementation for functions that handle Core 1 operations.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The functions in this file are responsible for managing the main loop of Core 1.
 * This includes processing the Core 1 state machine, handling GUI commands,
 * processing incoming LiDAR frames from Core 0, and managing the overall
 * system state.
 */

#include "core1_handling.h"
#include "globals.h"
#include "init.h"
#include "storage.h"
#include "gui.h"
#include "status.h"
#include "switch.h"
#include "trigger.h"
#include "calculations.h"
#include "neopixel_integration.h"

/**
 * @brief Main handler for the Core 1 loop.
 *
 * @details This function is the main entry point for Core 1's continuous operations. It
 * orchestrates the various tasks handled by Core 1, including processing the
 * state machine, managing status LEDs, and handling the main application logic
 * for both configuration and running states. It also periodically reports the
 * status of Core 1.
 */
void loop1_handler() {
  processCore1StateMachine();
  handleStatusLED();

  if (core1_state != (Core1InitState)999) {  // Not in terminal state
    updateNeoPixelStatus(NEO_INITIALIZING);
  }

  // Continuous error monitoring for red flash (only when system is ready)
  if (core1_state == (Core1InitState)999) {  // In terminal state (ready)
    static uint32_t last_error_check = 0;
    if (safeMillisElapsed(last_error_check, millis()) >= 50) {  // Check every 50ms
      uint32_t error_flags = 0;
      mutex_enter_blocking(&comm_mutex);
      error_flags = core_comm.error_flags;
      mutex_exit(&comm_mutex);

      if (error_flags != 0) {
        updateNeoPixelStatus(NEO_ERROR);  // Force error display
      }
      last_error_check = millis();
    }
  }

  if (current_state == STATE_CONFIG) {
    // CONFIG MODE: Only GUI commands and buffer drain
    processGuiCommands();
    updateNeoPixelStatus(NEO_CONFIG);

    // REV 2: Simple buffer drain to prevent overflow - no processing
    LidarFrame discard_frame;
    while (atomicBufferPop(discard_frame)) {
      // Just discard frames to prevent buffer overflow
      // No velocity calculation, no trigger logic, no data processing
    }

  } else if (current_state == STATE_RUNNING) {
    // RUNNING MODE: Full processing
    static uint32_t last_switch_read = 0;
    if (safeMillisElapsed(last_switch_read, millis()) >= 10) {
      mutex_enter_blocking(&comm_mutex);
      core_comm.switch_code = readSwitchCode();
      mutex_exit(&comm_mutex);
      last_switch_read = millis();
    }

    processIncomingFrames();

    if (isDebugEnabled()) {
      handleDebugOutput();
    }
  }

  static uint32_t last_status_report = 0;
  if (safeMillisElapsed(last_status_report, millis()) >= STATUS_CHECK_INTERVAL_MS) {
    reportCore1Status();
    last_status_report = millis();
  }

  yield();
}

/**
 * @brief Processes the state machine for Core 1.
 *
 * @details This function manages the initialization sequence and state transitions for Core 1.
 * It ensures that pins are initialized, configuration is loaded, and the system
 * correctly enters either configuration mode or normal running mode. The state
 * machine progresses through a series of states to bring Core 1 to a ready
 * state, after which it signals its readiness to Core 0.
 */
void processCore1StateMachine() {
  uint32_t current_time = millis();
  switch (core1_state) {
    case CORE1_STARTUP:
      if (safeMillisElapsed(core1_state_timer, current_time) >= 500) {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Startup delay complete, initializing pins...");
        core1_state = CORE1_PINS_INIT;
        core1_state_timer = current_time;
      }
      break;

    case CORE1_PINS_INIT:
      initializePinsCore1();  // This includes NeoPixel initialization with blue breathing
      if (isDebugEnabled()) safeSerialPrintln("Core 1: Pin initialization complete, loading configuration...");
      core1_state = CORE1_CONFIG_LOAD;
      core1_state_timer = current_time;
      break;

    case CORE1_CONFIG_LOAD:
      loadConfiguration();
      if (isDebugEnabled()) safeSerialPrintln("Core 1: Configuration loaded, checking for config mode...");
      core1_state = CORE1_CONFIG_MODE_CHECK;
      core1_state_timer = current_time;
      break;

    case CORE1_CONFIG_MODE_CHECK:
      if (Serial.available() > 0) {
        current_state = STATE_CONFIG;

        // REV 2: Set config mode flag to disable health monitoring
        mutex_enter_blocking(&comm_mutex);
        core_comm.config_mode_active = true;
        mutex_exit(&comm_mutex);

        updateNeoPixelStatus(NEO_CONFIG);
        if (isDebugEnabled()) {
          safeSerialPrintln("Core 1: Configuration mode triggered by serial input");
          safeSerialPrintln("Core 1: WARNING - Config mode active. Reset required to exit.");
        }
        core1_state = CORE1_READY;
        break;
      }
      if (safeMillisElapsed(core1_state_timer, current_time) >= CONFIG_MODE_TIMEOUT_MS) {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Config mode timeout - entering normal operation");
        current_state = STATE_RUNNING;

        // REV 2: Ensure config mode flag is clear for normal operation
        mutex_enter_blocking(&comm_mutex);
        core_comm.config_mode_active = false;
        mutex_exit(&comm_mutex);

        core1_state = CORE1_READY;
      }
      break;

    case CORE1_READY:
      // Set appropriate NeoPixel display based on system state
      if (current_state == STATE_RUNNING) {
        // Clear initialization display, ready for distance-based colors
        updateNeoPixelStatus(NEO_DISTANCE, 1000, 0, 255);  // Default distant reading
        if (isDebugEnabled()) {
          safeSerialPrintln("====================================");
          safeSerialPrintln("ENTERING NORMAL OPERATION MODE");
          safeSerialPrintln("LiDAR processing active");
          safeSerialPrintln("NeoPixel: Distance display active");
          safeSerialPrintln("LED will blink slowly (1000ms)");
          safeSerialPrintln("====================================");
        }
      } else if (current_state == STATE_CONFIG) {
        if (isDebugEnabled()) {
          safeSerialPrintln("====================================");
          safeSerialPrintln("ENTERING CONFIGURATION MODE");
          safeSerialPrintln("Ready for GUI commands on Serial");
          safeSerialPrintln("NeoPixel: Purple flashing");
          safeSerialPrintln("LED will blink rapidly (100ms)");
          safeSerialPrintln("RESET REQUIRED TO EXIT CONFIG MODE");
          safeSerialPrintln("====================================");
        }
      }

      timing_info.core1_init_complete = current_time;
      if (isDebugEnabled()) {
        safeSerialPrintfln("Core 1: Total initialization time: %lu ms",
                           timing_info.core1_init_complete - timing_info.core1_init_start);
      }
      safeSetCore1Ready(true);
      if (isDebugEnabled()) safeSerialPrintln("Core 1: Initialization complete - READY");
      core1_state = (Core1InitState)999;  // Terminal state
      break;
  }
}

/**
 * @brief Processes incoming LiDAR frames from Core 0.
 *
 * @details This function is the core of the data processing pipeline on Core 1. It pops
 * LiDAR frames from the shared atomic buffer, where they are placed by Core 0.
 * For each frame, it calculates the velocity, checks against the configured
 * trigger conditions (distance and velocity), and manages the trigger output.
 * It also updates the NeoPixel status based on the current distance and
 * velocity, and handles debug output if enabled.
 */
void processIncomingFrames() {
  static uint32_t frames_processed_count = 0;
  static AdaptiveVelocityCalculator velocity_calc;
  static bool last_trigger_state = false;
  LidarFrame frame;

  // REV 2: Only process frames in RUNNING mode for performance
  // Process multiple frames per call to prevent buffer buildup
  int frames_this_cycle = 0;
  const int MAX_FRAMES_PER_CYCLE = 5;  // Prevent excessive processing time

  while (atomicBufferPop(frame) && frames_this_cycle < MAX_FRAMES_PER_CYCLE) {
    frames_processed_count++;
    frames_this_cycle++;

    velocity_calc.addFrame(frame);
    float calculated_velocity = velocity_calc.calculateVelocity();

    uint8_t switch_code;
    mutex_enter_blocking(&comm_mutex);
    switch_code = core_comm.switch_code;
    mutex_exit(&comm_mutex);

    bool distance_ok = (frame.distance <= currentConfig.distance_thresholds[switch_code]);
    bool velocity_ok = !currentConfig.use_velocity_trigger || (calculated_velocity >= currentConfig.velocity_min_thresholds[switch_code] && calculated_velocity <= currentConfig.velocity_max_thresholds[switch_code]);

    bool raw_trigger = distance_ok && velocity_ok;
    bool debounced_trigger = trigger_debouncer.update(raw_trigger);
    bool final_trigger = trigger_latch.update(debounced_trigger);

    digitalWrite(TRIG_PULSE_LOW_PIN, final_trigger ? LOW : HIGH);

    // REV 2: Trigger flash on rising edge (trigger activation)
    if (final_trigger && !last_trigger_state) {
      triggerNeoPixelFlash();  // Start flash sequence tied to trigger latch
      if (isDebugEnabled()) {
        safeSerialPrintfln("Core 1: TRIGGER! Distance=%dcm, Velocity=%.1fcm/s, Switch=%d",
                           frame.distance, calculated_velocity, switch_code);
      }
    }
    last_trigger_state = final_trigger;

    mutex_enter_blocking(&comm_mutex);
    core_comm.trigger_output = final_trigger;
    core_comm.velocity = calculated_velocity;
    core_comm.distance = frame.distance;
    core_comm.strength = frame.strength;
    mutex_exit(&comm_mutex);

    // REV 2: Update NeoPixel with current data (only in normal operation)
    // Trigger flash takes priority and will override this temporarily
    if (current_state == STATE_RUNNING) {
      // Convert LiDAR strength (0-4096) to brightness (0-255)
      uint8_t brightness = (frame.strength > 4096) ? 255 : (frame.strength * 255) / 4096;
      updateNeoPixelStatus(NEO_DISTANCE, frame.distance, calculated_velocity, brightness);
    }
  }

  static uint32_t last_processing_report = 0;
  if (isDebugEnabled() && safeMillisElapsed(last_processing_report, millis()) >= PERFORMANCE_REPORT_INTERVAL_MS) {
    if (frames_processed_count > 0) {
      safeSerialPrintfln("Core 1: Processed %lu frames in last %d ms",
                         frames_processed_count, PERFORMANCE_REPORT_INTERVAL_MS);
    }
    frames_processed_count = 0;
    last_processing_report = millis();
  }
}