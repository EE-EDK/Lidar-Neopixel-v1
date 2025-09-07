/**
 * @file neopixel_integration.h
 * @brief NeoPixel LED control system for LiDAR status and distance visualization
 * @version 6.3.2
 * @date September 06, 2025 
 * @revision Rev 2 - 5Hz trigger flash synchronized with trigger latch
 * @changes:
 *   - Updated trigger flash timing to 5Hz (100ms on/off)
 *   - Trigger flash tied to TriggerLatch state for synchronization
 *   - Distance-based red→yellow→blue color gradient
 *   - Velocity-based saturation modulation
 */

#ifndef NEOPIXEL_INTEGRATION_H
#define NEOPIXEL_INTEGRATION_H

#include "globals.h"
#include <Adafruit_NeoPixel.h>

// NeoPixel system states
enum NeoPixelMode {
    NEO_OFF,
    NEO_DISTANCE,      // Normal distance display with red→yellow→blue gradient
    NEO_INITIALIZING,  // Blue breathing during startup
    NEO_CONFIG,        // Purple flashing in configuration mode 
    NEO_ERROR,         // Red flashing for error states
    NEO_TRIGGER_FLASH  // White trigger flash synchronized with trigger output
};

// REV 2: Updated trigger flash constants for 5Hz operation
const uint32_t NEOPIXEL_FLASH_ON_MS = 100;   // 100ms on (5Hz = 200ms period)
const uint32_t NEOPIXEL_FLASH_OFF_MS = 100;  // 100ms off (5Hz = 200ms period)
const float NEOPIXEL_SMOOTHING_ALPHA = 0.3f; // Smoothing factor for distance/strength

class NeoPixelController {
private:
    Adafruit_NeoPixel* strip;
    bool initialized;
    
    // REV 2: Simplified trigger flash - tied to external trigger latch state
    bool trigger_flash_requested;
    
public:
    // Smoothing state for distance display
    float smoothed_distance;
    float smoothed_strength;
    bool smoothing_initialized;
    
    NeoPixelController();
    ~NeoPixelController();
    
    bool init(uint8_t pin, uint8_t num_pixels = 1);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void clear();
    bool isReady() const { return initialized && strip != nullptr; }
    
    // REV 2: Simplified trigger flash interface
    void requestTriggerFlash() { trigger_flash_requested = true; }
    bool isTriggerFlashRequested() const { return trigger_flash_requested; }
    void clearTriggerFlashRequest() { trigger_flash_requested = false; }
};

// Global instance
extern NeoPixelController neopixel;

// High-level interface functions
bool initNeoPixel(uint8_t pin);
void updateNeoPixelStatus(NeoPixelMode mode, uint16_t distance = 0, float velocity = 0, uint8_t strength = 255);
void triggerNeoPixelFlash();

// Utility functions
uint32_t calculateDistanceColor(uint16_t distance_cm, float velocity_cm_s, uint8_t signal_strength);
uint32_t getStatusColor(NeoPixelMode mode, uint32_t time_ms);
uint32_t getTriggerFlashColor(uint32_t time_ms, bool trigger_active);

#endif // NEOPIXEL_INTEGRATION_H