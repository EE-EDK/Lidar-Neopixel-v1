#ifndef CORE0_HANDLING_H
#define CORE0_HANDLING_H

#include "globals.h"

void loop0_handler();
void processCore0StateMachine();
void processLidarSerial();
bool attemptRecovery(uint8_t recovery_level);
bool checkLidarSensorHealth(); 

#endif // CORE0_HANDLING_H