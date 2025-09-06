#include "switch.h"
#include "globals.h"

uint8_t readSwitchCode() {
  return (!digitalRead(S4_PIN) << 2) | (!digitalRead(S2_PIN) << 1) | !digitalRead(S1_PIN);
}