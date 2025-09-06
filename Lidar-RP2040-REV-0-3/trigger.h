#ifndef TRIGGER_H
#define TRIGGER_H

#include "globals.h"

class TriggerLatch {
private:
  enum State { IDLE, LATCHED };
  State state = IDLE;
  uint32_t latch_start_time = 0;
  const uint32_t latch_duration_ms = 3000;
public:
  bool update(bool trigger_event);
};

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
  bool update(bool raw_state);
};

extern TriggerDebouncer trigger_debouncer;
extern TriggerLatch trigger_latch;

#endif // TRIGGER_H