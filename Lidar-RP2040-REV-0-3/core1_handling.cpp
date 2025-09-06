#include "core1_handling.h"
#include "globals.h"
#include "init.h"
#include "storage.h"
#include "gui.h"
#include "status.h"
#include "switch.h"
#include "trigger.h"
#include "calculations.h"
#include "neopixel_integration.h"  // ADDED: Include NeoPixel system

void loop1_handler() {
  processCore1StateMachine();
  handleStatusLED();

  if (current_state == STATE_CONFIG) {
    processGuiCommands();
    // ADDED: Keep config mode display active
    updateNeoPixelStatus(NEO_CONFIG);
  } else if (current_state == STATE_RUNNING) {
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
      initializePinsCore1();  // This now includes NeoPixel initialization with blue breathing
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
        // ADDED: Switch to config mode display (purple flashing)
        updateNeoPixelStatus(NEO_CONFIG);
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Configuration mode triggered by serial input");
        core1_state = CORE1_READY;
        break;
      }
      if (safeMillisElapsed(core1_state_timer, current_time) >= CONFIG_MODE_TIMEOUT_MS) {
        if (isDebugEnabled()) safeSerialPrintln("Core 1: Config mode timeout - entering normal operation");
        current_state = STATE_RUNNING;
        core1_state = CORE1_READY;
      }
      break;
      
    case CORE1_READY:
      // ADDED: Set appropriate NeoPixel display based on system state
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
          safeSerialPrintln("====================================");
        }
      }
      // END ADDED SECTION
      
      timing_info.core1_init_complete = current_time;
      if (isDebugEnabled()) {
        safeSerialPrintfln("Core 1: Total initialization time: %lu ms",
          timing_info.core1_init_complete - timing_info.core1_init_start);
      }
      safeSetCore1Ready(true);
      if (isDebugEnabled()) safeSerialPrintln("Core 1: Initialization complete - READY");
      core1_state = (Core1InitState)999; // Terminal state
      break;
  }
}

void processIncomingFrames() {
  static uint32_t frames_processed_count = 0;
  static AdaptiveVelocityCalculator velocity_calc;
  static bool last_trigger_state = false;  // ADDED: Track trigger state changes for flash
  LidarFrame frame;

  while (atomicBufferPop(frame)) {
    frames_processed_count++;
    velocity_calc.addFrame(frame);
    float calculated_velocity = velocity_calc.calculateVelocity();
    
    uint8_t switch_code;
    mutex_enter_blocking(&comm_mutex);
    switch_code = core_comm.switch_code;
    mutex_exit(&comm_mutex);

    bool distance_ok = (frame.distance <= currentConfig.distance_thresholds[switch_code]);
    bool velocity_ok = !currentConfig.use_velocity_trigger || 
                       (calculated_velocity >= currentConfig.velocity_min_thresholds[switch_code] && 
                        calculated_velocity <= currentConfig.velocity_max_thresholds[switch_code]);
    
    bool raw_trigger = distance_ok && velocity_ok;
    bool debounced_trigger = trigger_debouncer.update(raw_trigger);
    bool final_trigger = trigger_latch.update(debounced_trigger);

    digitalWrite(TRIG_PULSE_LOW_PIN, final_trigger ? LOW : HIGH);
    
    // ADDED: Trigger flash on rising edge (trigger activation)
    if (final_trigger && !last_trigger_state) {
      triggerNeoPixelFlash();  // Start 3-flash white sequence
      if (isDebugEnabled()) {
        safeSerialPrintfln("Core 1: TRIGGER! Distance=%dcm, Velocity=%.1fcm/s, Switch=%d", 
                          frame.distance, calculated_velocity, switch_code);
      }
    }
    last_trigger_state = final_trigger;
    // END ADDED SECTION
    
    mutex_enter_blocking(&comm_mutex);
    core_comm.trigger_output = final_trigger;
    core_comm.velocity = calculated_velocity;
    core_comm.distance = frame.distance;
    core_comm.strength = frame.strength;
    mutex_exit(&comm_mutex);
    
    // ADDED: Update NeoPixel with current data (only in normal operation)
    // Note: Trigger flash takes priority and will override this temporarily
    if (current_state == STATE_RUNNING) {
      // Convert LiDAR strength (0-4096) to brightness (0-255)
      uint8_t brightness = (frame.strength > 4096) ? 255 : (frame.strength * 255) / 4096;
      updateNeoPixelStatus(NEO_DISTANCE, frame.distance, calculated_velocity, brightness);
    }
    // END ADDED SECTION
  }

  static uint32_t last_processing_report = 0;
  if (isDebugEnabled() && safeMillisElapsed(last_processing_report, millis()) >= PERFORMANCE_REPORT_INTERVAL_MS) {
    frames_processed_count = 0;
    last_processing_report = millis();
  }
}