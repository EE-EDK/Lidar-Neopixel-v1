#include "trigger.h"

TriggerDebouncer trigger_debouncer;
TriggerLatch trigger_latch;

bool TriggerLatch::update(bool trigger_event) {
    uint32_t now = millis();
    switch (state) {
      case IDLE:
        if (trigger_event) {
          state = LATCHED;
          latch_start_time = now;
        }
        break;
      case LATCHED:
        if (safeMillisElapsed(latch_start_time, now) >= latch_duration_ms) {
          state = IDLE;
        }
        break;
    }
    return (state == LATCHED);
}

bool TriggerDebouncer::update(bool raw_state) {
    uint32_t now = millis();
    if (raw_state != last_raw_state) {
      last_change_time = now;
      last_raw_state = raw_state;
    }
    
    if (raw_state && !current_state && safeMillisElapsed(last_change_time, now) >= debounce_on_ms) {
      current_state = true;
      last_state_time = now;
    } else if (!raw_state && current_state && 
               safeMillisElapsed(last_change_time, now) >= debounce_off_ms && 
               safeMillisElapsed(last_state_time, now) >= min_pulse_ms) {
      current_state = false;
      last_state_time = now;
    }
    
    return current_state;
}