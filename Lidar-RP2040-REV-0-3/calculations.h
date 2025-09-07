/**
 * @file calculations.h
 * @brief This file contains the declaration of the AdaptiveVelocityCalculator class.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details The AdaptiveVelocityCalculator class is designed to calculate the velocity of an object
 * based on a series of LiDAR frames. It uses an adaptive algorithm to provide more
 * accurate velocity readings, especially in noisy environments.
 */

#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include "globals.h"

/**
 * @class AdaptiveVelocityCalculator
 * @brief Calculates velocity adaptively from LiDAR frames.
 *
 * @details This class maintains a history of LiDAR frames and uses them to calculate the velocity
 * of a detected object. It includes mechanisms to handle errors and adapt to varying
 * conditions to provide a stable velocity reading.
 */
class AdaptiveVelocityCalculator {
private:
  /**
   * @brief The maximum number of LiDAR frames to store in the history.
   */
  static constexpr int MAX_HISTORY = 15;

  /**
   * @brief An array to store the history of LiDAR frames.
   */
  LidarFrame history[MAX_HISTORY];

  /**
   * @brief The number of frames currently stored in the history.
   */
  uint8_t count = 0;

  /**
   * @brief The last calculated velocity.
   */
  float last_velocity = 0.0f;

  /**
   * @brief A counter for the number of consecutive errors encountered.
   */
  uint32_t error_count = 0;

  /**
   * @brief The timestamp of the last detected movement.
   */
  uint32_t last_movement_time = 0;

public:
  /**
   * @brief Calculates the velocity based on the stored LiDAR frames.
   *
   * @return The calculated velocity in meters per second.
   */
  float calculateVelocity();

  /**
   * @brief Adds a new LiDAR frame to the history.
   *
   * @param frame The LidarFrame to add.
   */
  void addFrame(const LidarFrame& frame);
};

#endif // CALCULATIONS_H