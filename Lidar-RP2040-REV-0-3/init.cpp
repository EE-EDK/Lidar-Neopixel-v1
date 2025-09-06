#include "init.h"
#include "globals.h"
#include "status.h"
#include "neopixel_integration.h"  


void initializePinsCore1() {
  if (isDebugEnabled()) safeSerialPrintln("Core 1: Configuring GPIO pins...");
  pinMode(S1_PIN, INPUT_PULLUP);
  pinMode(S2_PIN, INPUT_PULLUP);
  pinMode(S4_PIN, INPUT_PULLUP);
  pinMode(ROTARY_CONN_PIN, INPUT_PULLUP);
  pinMode(EXT_TRIG_PIN, INPUT);
  pinMode(EXT_TRIG_EN_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(TRIG_PULSE_LOW_PIN, OUTPUT);

  digitalWrite(TRIG_PULSE_LOW_PIN, HIGH);
  digitalWrite(STATUS_LED_PIN, LOW);

// Initialize NeoPixel system
  if (initNeoPixel(NEOPIXEL_PIN)) {
    if (isDebugEnabled()) {
      safeSerialPrintfln("Core 1: NeoPixel initialized successfully on pin %d", NEOPIXEL_PIN);
    }
    // Start initialization display (blue breathing)
    updateNeoPixelStatus(NEO_INITIALIZING);
  } else {
    if (isDebugEnabled()) {
      safeSerialPrintfln("Core 1: WARNING - NeoPixel initialization failed on pin %d", NEOPIXEL_PIN);
    }
  }

  if (isDebugEnabled()) {
    safeSerialPrintln("Core 1: GPIO and NeoPixel configuration complete");
    safeSerialPrintfln("Core 1: Pin assignments - S1:%d, S2:%d, S4:%d, LED:%d, TRIG:%d, NEOPIXEL:%d",
      S1_PIN, S2_PIN, S4_PIN, STATUS_LED_PIN, TRIG_PULSE_LOW_PIN, NEOPIXEL_PIN);
  }
}

void main_setup() {
  timing_info.core0_init_start = millis();
  mutex_init(&buffer_mutex);
  mutex_init(&comm_mutex);
  mutex_init(&serial_mutex);
  mutex_init(&perf_mutex);

  Serial.begin(DEBUG_BAUD_RATE);
  uint32_t serial_start = millis();
  while (!Serial && (millis() - serial_start) < 3000) {
    delay(100);
  }

  if (isDebugEnabled()) {
    safeSerialPrintln("=====================================");
    safeSerialPrintln("RP2040 LiDAR Controller v6.3 Starting");
    safeSerialPrintln("Arduino-Pico Native Dual-Core Version");
    safeSerialPrintln("Enhanced Reliability & Performance");
    safeSerialPrintln("=====================================");
    safeSerialPrintfln("Core 0: Initializing at %lu ms", millis());
  }

  timing_info.adaptive_timeout_us = FRAME_TIMEOUT_US;
  core0_state_timer = millis();
  core0_state = CORE0_STARTUP;
  if (isDebugEnabled()) {
    safeSerialPrintln("Core 0: Mutex initialization complete");
    safeSerialPrintln("Core 0: Starting serial initialization sequence...");
  }
}

void setup1_handler() {
  timing_info.core1_init_start = millis();
  if (isDebugEnabled()) {
    safeSerialPrintfln("Core 1: Initializing at %lu ms", timing_info.core1_init_start);
  }
  core1_state_timer = millis();
  core1_state = CORE1_STARTUP;
}