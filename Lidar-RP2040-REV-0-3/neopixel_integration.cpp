/**
 * @file neopixel_integration.cpp
 * @brief This file contains the implementation for the NeoPixel integration.
 * @author The Lidar-RP2040-REV-0-3 Team
 * @version 1.0
 * @date 2025-09-06
 *
 * @details This file provides the implementation for controlling a NeoPixel LED to
 * provide visual feedback on the system's status.
 */

#include "neopixel_integration.h"
#include "trigger.h"

// Global instance
NeoPixelController neopixel;

/**
 * @brief Construct a new NeoPixelController::NeoPixelController object
 */
NeoPixelController::NeoPixelController()
  : strip(nullptr), initialized(false), trigger_flash_requested(false),
    smoothed_distance(0), smoothed_strength(0), smoothing_initialized(false) {
}

/**
 * @brief Destroy the NeoPixelController::NeoPixelController object
 */
NeoPixelController::~NeoPixelController() {
  if (strip) {
    delete strip;
    strip = nullptr;
  }
}

/**
 * @brief Initializes the NeoPixel.
 * @param pin The pin the NeoPixel is connected to.
 * @param num_pixels The number of pixels in the strip.
 * @return True if initialization was successful, false otherwise.
 */
bool NeoPixelController::init(uint8_t pin, uint8_t num_pixels) {
  if (strip) {
    delete strip;
  }

  // Create Adafruit NeoPixel object
  strip = new Adafruit_NeoPixel(num_pixels, pin, NEO_GRB + NEO_KHZ800);

  if (!strip) {
    return false;
  }

  // Initialize the library
  strip->begin();
  strip->show();              // Initialize all pixels to 'off'
  strip->setBrightness(200);  // Set to ~78% brightness

  initialized = true;
  clear();

  return true;
}

/**
 * @brief Sets the color of the NeoPixel.
 * @param r The red component of the color.
 * @param g The green component of the color.
 * @param b The blue component of the color.
 */
void NeoPixelController::setColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!isReady()) return;

  strip->setPixelColor(0, strip->Color(r, g, b));
  strip->show();
}

/**
 * @brief Clears the NeoPixel.
 */
void NeoPixelController::clear() {
  if (!isReady()) return;

  strip->clear();
  strip->show();
}

/**
 * @brief Initializes the NeoPixel system.
 * @param pin The pin the NeoPixel is connected to.
 * @return True if initialization was successful, false otherwise.
 */
bool initNeoPixel(uint8_t pin) {
  return neopixel.init(pin, 1);  // Single pixel
}

/**
 * @brief Triggers a flash of the NeoPixel.
 */
void triggerNeoPixelFlash() {
  neopixel.requestTriggerFlash();
}

/**
 * @brief Updates the NeoPixel status based on the current mode.
 * @param mode The NeoPixel mode to set.
 * @param distance The current distance measurement.
 * @param velocity The current velocity measurement.
 * @param strength The current signal strength.
 */
void updateNeoPixelStatus(NeoPixelMode mode, uint16_t distance, float velocity, uint8_t strength) {
  if (!neopixel.isReady()) return;

  static uint32_t last_update = 0;
  static uint32_t error_flash_start = 0;
  static bool error_flash_active = false;
  uint32_t now = millis();

  // Rate limit updates to 50Hz for performance
  if (now - last_update < 20) return;
  last_update = now;

  // Check for error conditions (second highest priority)
  uint32_t error_flags = 0;
  mutex_enter_blocking(&comm_mutex);
  error_flags = core_comm.error_flags;
  mutex_exit(&comm_mutex);

  if (error_flags != 0 && !error_flash_active) {
    error_flash_start = now;
    error_flash_active = true;
  } else if (error_flags == 0) {
    error_flash_active = false;
  }

  // Handle trigger flash with absolute priority
  bool trigger_currently_active = (digitalRead(TRIG_PULSE_LOW_PIN) == LOW);

  if (trigger_currently_active && neopixel.isTriggerFlashRequested()) {
    uint32_t color = getTriggerFlashColor(now, true);
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    neopixel.setColor(r, g, b);
    return;
  } else if (!trigger_currently_active) {
    neopixel.clearTriggerFlashRequest();
  }

  // Handle error flash (second priority)
  if (error_flash_active && (now - error_flash_start) < 3000) {
    uint32_t color = getStatusColor(NEO_ERROR, now);
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    neopixel.setColor(r, g, b);
    return;
  } else if (error_flash_active && (now - error_flash_start) >= 3000) {
    error_flash_active = false;
  }

  // Handle normal modes
  uint32_t color;
  switch (mode) {
    case NEO_OFF:
      neopixel.clear();
      return;

    case NEO_DISTANCE:
      color = calculateDistanceColor(distance, velocity, strength);
      break;

    case NEO_TRIGGER_FLASH:
      return;

    case NEO_INITIALIZING:
    case NEO_CONFIG:
    case NEO_ERROR:
      color = getStatusColor(mode, now);
      break;

    default:
      neopixel.clear();
      return;
  }

  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  neopixel.setColor(r, g, b);
}

/**
 * @brief Calculates the color for the distance display.
 * @param distance_cm The distance in centimeters.
 * @param velocity_cm_s The velocity in centimeters per second.
 * @param signal_strength The signal strength.
 * @return The calculated color in 32-bit format.
 */
uint32_t calculateDistanceColor(uint16_t distance_cm, float velocity_cm_s, uint8_t signal_strength) {
  // Apply smoothing to reduce noise and flickering
  if (!neopixel.smoothing_initialized) {
    neopixel.smoothed_distance = distance_cm;
    neopixel.smoothed_strength = signal_strength;
    neopixel.smoothing_initialized = true;
  } else {
    neopixel.smoothed_distance = NEOPIXEL_SMOOTHING_ALPHA * distance_cm + (1.0f - NEOPIXEL_SMOOTHING_ALPHA) * neopixel.smoothed_distance;
    neopixel.smoothed_strength = NEOPIXEL_SMOOTHING_ALPHA * signal_strength + (1.0f - NEOPIXEL_SMOOTHING_ALPHA) * neopixel.smoothed_strength;
  }

  // Use smoothed values
  float distance = neopixel.smoothed_distance;
  float strength = neopixel.smoothed_strength;

  // Clamp distance to valid range
  if (distance < MIN_DISTANCE_CM) distance = MIN_DISTANCE_CM;
  if (distance > MAX_DISTANCE_CM) distance = MAX_DISTANCE_CM;

  // REV 2: Calculate position in range (0.0 = close/hot, 1.0 = far/cool)
  float position = (distance - MIN_DISTANCE_CM) / (MAX_DISTANCE_CM - MIN_DISTANCE_CM);

  // REV 2: Distance-based heat map colors (red → yellow → blue)
  uint8_t r, g, b;

  if (position <= 0.5f) {
    // First half: Red (0.0) to Yellow (0.5)
    float local_pos = position * 2.0f;  // 0.0 to 1.0
    r = 255;
    g = (uint8_t)(255 * local_pos);  // Increase green to make yellow
    b = 0;
  } else {
    // Second half: Yellow (0.5) to Blue (1.0)
    float local_pos = (position - 0.5f) * 2.0f;  // 0.0 to 1.0
    r = (uint8_t)(255 * (1.0f - local_pos));     // Decrease red
    g = (uint8_t)(255 * (1.0f - local_pos));     // Decrease green
    b = (uint8_t)(255 * local_pos);              // Increase blue
  }

  // REV 2: Apply velocity-based saturation modulation
  float saturation_factor = 1.0f;  // Default saturation
  float abs_velocity = abs(velocity_cm_s);

  if (abs_velocity > 5.0f) {  // Only apply if there's significant movement
    if (velocity_cm_s < -5.0f) {
      // Approaching - make more vivid (increase saturation)
      saturation_factor = 1.2f;  // +20% saturation
    } else if (velocity_cm_s > 5.0f) {
      // Receding - make more muted (decrease saturation)
      saturation_factor = 0.7f;  // -30% saturation
    }
  }

  // Apply saturation factor (limit to prevent overflow)
  r = (uint8_t)(constrain(r * saturation_factor, 0, 255));
  g = (uint8_t)(constrain(g * saturation_factor, 0, 255));
  b = (uint8_t)(constrain(b * saturation_factor, 0, 255));

  // Apply brightness based on signal strength (30-100%)
  float brightness = 0.3f + 0.7f * (strength / 255.0f);
  r = (uint8_t)(r * brightness);
  g = (uint8_t)(g * brightness);
  b = (uint8_t)(b * brightness);

  // Return as 32-bit color (0x00RRGGBB format)
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
 * @brief Gets the color for the trigger flash.
 * @param time_ms The current time in milliseconds.
 * @param trigger_active True if the trigger is active, false otherwise.
 * @return The calculated color in 32-bit format.
 */
uint32_t getTriggerFlashColor(uint32_t time_ms, bool trigger_active) {
  if (!trigger_active) {
    return 0x00000000;  // Off when trigger not active
  }

  // REV 2: 5Hz flash (200ms cycle = 100ms on + 100ms off)
  const uint32_t FLASH_CYCLE_MS = NEOPIXEL_FLASH_ON_MS + NEOPIXEL_FLASH_OFF_MS;
  uint32_t cycle_position = time_ms % FLASH_CYCLE_MS;

  // Flash is on for first 100ms of each 200ms cycle
  if (cycle_position < NEOPIXEL_FLASH_ON_MS) {
    return 0x00FFFFFF;  // Full brightness white (0x00RRGGBB)
  } else {
    return 0x00000000;  // Off
  }
}

/**
 * @brief Gets the color for a given status mode.
 * @param mode The status mode.
 * @param time_ms The current time in milliseconds.
 * @return The calculated color in 32-bit format.
 */
uint32_t getStatusColor(NeoPixelMode mode, uint32_t time_ms) {
  switch (mode) {
    case NEO_INITIALIZING:
    {
        // Slow, deep breathing blue animation (3 second cycle)
        uint32_t cycle_time = time_ms % 3000;  // 0 to 2999 (slow cycle)
        float phase = cycle_time / 3000.0f;    // 0.0 to 1.0
        
        // Smooth breathing curve using parabolic function
        float breath_curve;
        if (phase < 0.5f) {
            // Inhale: smooth acceleration to peak
            float t = phase * 2.0f;  // 0.0 to 1.0
            breath_curve = t * t;    // Parabolic curve (smooth start)
        } else {
            // Exhale: smooth deceleration from peak
            float t = (1.0f - phase) * 2.0f;  // 1.0 to 0.0
            breath_curve = t * t;             // Parabolic curve (smooth end)
        }
        
        // Very dramatic brightness range for deep breathing effect
        float brightness = 0.02f + 0.98f * breath_curve;  // 0.02 to 1.0 (very dramatic)
        
        uint8_t blue = (uint8_t)(255 * brightness);
        return (uint32_t)blue;  // 0x000000BB
    }

    case NEO_CONFIG:
      {
        // Purple flashing (1Hz = 1000ms cycle)
        bool on = (time_ms % 1000) < 500;
        return on ? 0x00800080 : 0x00000000;  // Purple or off
      }

    case NEO_ERROR:
      {
        // Red fast flashing (4Hz = 250ms cycle)
        bool on = (time_ms % 250) < 125;
        return on ? 0x00FF0000 : 0x00000000;  // Red or off
      }

    default:
      return 0x00000000;  // Off
  }
}