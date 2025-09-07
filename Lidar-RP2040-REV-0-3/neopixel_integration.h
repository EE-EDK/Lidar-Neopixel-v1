/**
 * @file neopixel_integration.h
 * @brief This file contains the declarations for the NeoPixel integration.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details This file provides an interface for controlling a NeoPixel LED to provide
 * visual feedback on the system's status. It includes functions for
 * initializing the NeoPixel, setting its color, and displaying different
 * patterns to indicate various states such as initialization, configuration,
 * error, and normal operation.
 */

#ifndef NEOPIXEL_INTEGRATION_H
#define NEOPIXEL_INTEGRATION_H

#include "globals.h"
#include <Adafruit_NeoPixel.h>

/**
 * @brief Defines the different modes for the NeoPixel display.
 */
enum NeoPixelMode {
    NEO_OFF,                ///< NeoPixel is off.
    NEO_DISTANCE,           ///< Normal distance display with a red-yellow-blue gradient.
    NEO_INITIALIZING,       ///< Blue breathing pattern during startup.
    NEO_CONFIG,             ///< Purple flashing pattern in configuration mode.
    NEO_ERROR,              ///< Red flashing pattern for error states.
    NEO_TRIGGER_FLASH       ///< White trigger flash synchronized with the trigger output.
};

/** @brief The duration in milliseconds for the NeoPixel flash 'on' state. */
const uint32_t NEOPIXEL_FLASH_ON_MS = 100;
/** @brief The duration in milliseconds for the NeoPixel flash 'off' state. */
const uint32_t NEOPIXEL_FLASH_OFF_MS = 100;
/** @brief The smoothing factor for the distance and strength values. */
const float NEOPIXEL_SMOOTHING_ALPHA = 0.3f;

/**
 * @class NeoPixelController
 * @brief Manages the NeoPixel LED.
 *
 * @details This class provides a low-level interface for controlling the NeoPixel LED.
 * It encapsulates the Adafruit_NeoPixel library and provides methods for
 * initializing the LED, setting its color, and clearing it. It also includes
 * state for managing trigger flashes and smoothing distance and strength values.
 */
class NeoPixelController {
private:
    Adafruit_NeoPixel* strip; ///< Pointer to the Adafruit_NeoPixel instance.
    bool initialized;         ///< Flag indicating if the NeoPixel is initialized.
    
    bool trigger_flash_requested; ///< Flag indicating if a trigger flash has been requested.
    
public:
    float smoothed_distance;     ///< The smoothed distance value.
    float smoothed_strength;     ///< The smoothed strength value.
    bool smoothing_initialized;  ///< Flag indicating if the smoothing has been initialized.
    
    /**
     * @brief Construct a new NeoPixelController object.
     */
    NeoPixelController();

    /**
     * @brief Destroy the NeoPixelController object.
     */
    ~NeoPixelController();
    
    /**
     * @brief Initializes the NeoPixel.
     * @param pin The pin the NeoPixel is connected to.
     * @param num_pixels The number of pixels in the strip.
     * @return True if initialization was successful, false otherwise.
     */
    bool init(uint8_t pin, uint8_t num_pixels = 1);

    /**
     * @brief Sets the color of the NeoPixel.
     * @param r The red component of the color.
     * @param g The green component of the color.
     * @param b The blue component of the color.
     */
    void setColor(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Clears the NeoPixel.
     */
    void clear();

    /**
     * @brief Checks if the NeoPixel is ready.
     * @return True if the NeoPixel is ready, false otherwise.
     */
    bool isReady() const { return initialized && strip != nullptr; }
    
    /**
     * @brief Requests a trigger flash.
     */
    void requestTriggerFlash() { trigger_flash_requested = true; }

    /**
     * @brief Checks if a trigger flash has been requested.
     * @return True if a trigger flash has been requested, false otherwise.
     */
    bool isTriggerFlashRequested() const { return trigger_flash_requested; }

    /**
     * @brief Clears the trigger flash request.
     */
    void clearTriggerFlashRequest() { trigger_flash_requested = false; }
};

/**
 * @brief The global instance of the NeoPixelController.
 */
extern NeoPixelController neopixel;

/**
 * @brief High-level interface functions for controlling the NeoPixel.
 * @{
 */

/**
 * @brief Initializes the NeoPixel system.
 * @param pin The pin the NeoPixel is connected to.
 * @return True if initialization was successful, false otherwise.
 */
bool initNeoPixel(uint8_t pin);

/**
 * @brief Updates the NeoPixel status based on the current mode.
 * @param mode The NeoPixel mode to set.
 * @param distance The current distance measurement.
 * @param velocity The current velocity measurement.
 * @param strength The current signal strength.
 */
void updateNeoPixelStatus(NeoPixelMode mode, uint16_t distance = 0, float velocity = 0, uint8_t strength = 255);

/**
 * @brief Triggers a flash of the NeoPixel.
 */
void triggerNeoPixelFlash();
void triggerGuiSuccessGlow();

/** @} */

/**
 * @brief Utility functions for calculating NeoPixel colors.
 * @{
 */

/**
 * @brief Calculates the color for the distance display.
 * @param distance_cm The distance in centimeters.
 * @param velocity_cm_s The velocity in centimeters per second.
 * @param signal_strength The signal strength.
 * @return The calculated color in 32-bit format.
 */
uint32_t calculateDistanceColor(uint16_t distance_cm, float velocity_cm_s, uint8_t signal_strength);

/**
 * @brief Gets the color for a given status mode.
 * @param mode The status mode.
 * @param time_ms The current time in milliseconds.
 * @return The calculated color in 32-bit format.
 */
uint32_t getStatusColor(NeoPixelMode mode, uint32_t time_ms);

/**
 * @brief Gets the color for the trigger flash.
 * @param time_ms The current time in milliseconds.
 * @param trigger_active True if the trigger is active, false otherwise.
 * @return The calculated color in 32-bit format.
 */
uint32_t getTriggerFlashColor(uint32_t time_ms, bool trigger_active);

/** @} */

#endif // NEOPIXEL_INTEGRATION_H