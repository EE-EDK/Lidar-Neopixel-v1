#ifndef STORAGE_H
#define STORAGE_H

#include "globals.h"

void loadDefaultConfig();
bool validateConfiguration(const LidarConfiguration& config);
uint16_t calculateChecksum(const LidarConfiguration& config);
void loadConfiguration();
bool saveConfiguration();
void factoryReset();

#endif // STORAGE_H