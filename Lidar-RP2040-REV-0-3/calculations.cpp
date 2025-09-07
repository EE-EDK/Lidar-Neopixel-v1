/**
 * @file calculations.cpp
 * @brief This file contains the implementation of the AdaptiveVelocityCalculator class.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The AdaptiveVelocityCalculator class is designed to calculate the velocity of an object
 * based on a series of LiDAR frames. This implementation file provides the logic for
 * calculating velocity and managing the history of LiDAR frames.
 */

#include "calculations.h"

/**
 * @brief Calculates the velocity based on the stored LiDAR frames.
 *
 * @details This function calculates the velocity of an object by analyzing the distance and time
 * differences between consecutive LiDAR frames stored in the history. It uses a median
 * filter to reduce noise and provides a more stable velocity reading. It also includes
 * logic to handle deadbands and error conditions.
 *
 * @return The calculated velocity in cm/s.
 */
float AdaptiveVelocityCalculator::calculateVelocity() {
    if (count < 5) return 0.0f;
    float velocities[5];
    int valid_velocities = 0;
    int small_movement_count = 0;
    
    for (int i = 2; i < count && valid_velocities < 5; i += 2) {
      uint32_t time_diff = safeMicrosElapsed(history[i].timestamp, history[0].timestamp);
      if (time_diff > 1000 && time_diff < 50000) {
        int32_t dist_diff = (int32_t)history[0].distance - (int32_t)history[i].distance;
        if (abs(dist_diff) <= DISTANCE_DEADBAND_THRESHOLD_CM) {
          small_movement_count++;
        }
        float velocity = ((float)dist_diff * 1000000.0f) / (float)time_diff;
        velocities[valid_velocities++] = velocity;
      }
    }
  
    if (valid_velocities == 0) {
      error_count++;
      safeSetErrorFlag(ERROR_FLAG_VELOCITY_CALC_ERROR, error_count > 10);
      return last_velocity;
    }
    
if (small_movement_count >= (valid_velocities / 2 + 1)) {
    // Reset velocity to 0 immediately if we detect mostly small movements
    last_velocity = 0.0f;
    last_movement_time = millis();
    error_count = 0;
    safeSetErrorFlag(ERROR_FLAG_VELOCITY_CALC_ERROR, false);
    return last_velocity;
}
    
    float median_velocity;
    if (valid_velocities == 1) {
      median_velocity = velocities[0];
    } else {
      for (int i = 0; i < valid_velocities - 1; i++) {
        for (int j = i + 1; j < valid_velocities; j++) {
          if (velocities[i] > velocities[j]) {
            float temp = velocities[i];
            velocities[i] = velocities[j];
            velocities[j] = temp;
          }
        }
      }
      median_velocity = velocities[valid_velocities / 2];
    }
    
    if (abs(median_velocity) <= VELOCITY_DEADBAND_THRESHOLD_CM_S) {
      median_velocity = 0.0f;
    }
    
    last_velocity = median_velocity;
    last_movement_time = millis();
    error_count = 0;
    safeSetErrorFlag(ERROR_FLAG_VELOCITY_CALC_ERROR, false);
    return last_velocity;
}

/**
 * @brief Adds a new LiDAR frame to the history.
 *
 * @details This function adds a new LidarFrame to the beginning of the history array, shifting
 * the older frames back. The history is used by the calculateVelocity() function to
 * determine the object's velocity.
 *
 * @param frame The LidarFrame to add to the history.
 */
void AdaptiveVelocityCalculator::addFrame(const LidarFrame& frame) {
    for (int i = MAX_HISTORY - 1; i > 0; i--) {
      history[i] = history[i - 1];
    }
    history[0] = frame;
    if (count < MAX_HISTORY) count++;
}