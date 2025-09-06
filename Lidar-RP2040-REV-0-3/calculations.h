#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include "globals.h"

class AdaptiveVelocityCalculator {
private:
  static constexpr int MAX_HISTORY = 15;
  LidarFrame history[MAX_HISTORY];
  uint8_t count = 0;
  float last_velocity = 0.0f;
  uint32_t error_count = 0;
  uint32_t last_movement_time = 0;
public:
  float calculateVelocity();
  void addFrame(const LidarFrame& frame);
};

#endif // CALCULATIONS_H