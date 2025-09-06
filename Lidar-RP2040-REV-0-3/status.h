#ifndef STATUS_H
#define STATUS_H

#include "globals.h"

void handleDebugOutput();
void handleStatusLED();
void reportCore0Status();
void reportCore1Status();

#endif // STATUS_H