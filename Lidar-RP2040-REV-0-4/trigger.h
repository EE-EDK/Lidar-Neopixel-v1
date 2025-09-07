/**
 * @file trigger.h
 * @brief This file contains the declarations for the trigger handling classes.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The classes in this file are responsible for managing the trigger logic,
 * including debouncing the raw trigger signal and latching the trigger
 * for a specific duration.
 */
#ifndef TRIGGER_H
#define TRIGGER_H

#include "globals.h"

/**
 * @class TriggerLatch
 * @brief Latches the trigger signal for a specific duration.
 *
 * @details This class takes a trigger event and latches it in the 'on' state for a
 * configurable duration. This is used to ensure that the trigger signal
 * remains active for a minimum amount of time.
 */
class TriggerLatch {
private:
  enum State { IDLE, LATCHED };
  State state = IDLE;
  uint32_t latch_start_time = 0;
  const uint32_t latch_duration_ms = 3000;
public:
  /**
   * @brief Updates the trigger latch with a new trigger event.
   *
   * @param trigger_event The new trigger event.
   * @return The current state of the latch.
   */
  bool update(bool trigger_event);
};

/**
 * @class TriggerDebouncer
 * @brief Debounces the raw trigger signal.
 *
 * @details This class takes a raw trigger signal and debounces it to remove any
 * spurious transitions. It uses separate on and off debounce delays to
 * provide more control over the debouncing behavior.
 */
class TriggerDebouncer {
private:
  uint32_t last_change_time = 0;
  uint32_t last_state_time = 0;
  bool current_state = false;
  bool last_raw_state = false;
  const uint32_t debounce_on_ms = 30;
  const uint32_t debounce_off_ms = 50;
  const uint32_t min_pulse_ms = 20;
public:
  /**
   * @brief Updates the debouncer with a new raw trigger state.
   *
   * @param raw_state The new raw trigger state.
   * @return The debounced trigger state.
   */
  bool update(bool raw_state);
};

/** @brief The global trigger debouncer instance. */
extern TriggerDebouncer trigger_debouncer;
/** @brief The global trigger latch instance. */
extern TriggerLatch trigger_latch;

#endif // TRIGGER_H