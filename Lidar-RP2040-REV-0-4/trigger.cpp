/**
 * @file trigger.cpp
 * @brief This file contains the implementation for the trigger handling classes.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The classes in this file are responsible for managing the trigger logic,
 * including debouncing the raw trigger signal and latching the trigger
 * for a specific duration.
 */

#include "trigger.h"

TriggerDebouncer trigger_debouncer;
TriggerLatch trigger_latch;

/**
 * @brief Updates the trigger latch with a new trigger event.
 *
 * @param trigger_event The new trigger event.
 * @return The current state of the latch.
 */
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

/**
 * @brief Updates the debouncer with a new raw trigger state.
 *
 * @param raw_state The new raw trigger state.
 * @return The debounced trigger state.
 */
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