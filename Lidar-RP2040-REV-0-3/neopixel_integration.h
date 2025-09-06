// neopixel_integration.h - FIXED: Compilation errors resolved
#ifndef NEOPIXEL_INTEGRATION_H
#define NEOPIXEL_INTEGRATION_H

#include "globals.h"
#include <Adafruit_NeoPixel.h>

// NeoPixel system states
enum NeoPixelMode {
    NEO_OFF,
    NEO_DISTANCE,      // Normal distance display
    NEO_INITIALIZING,  // Blue breathing
    NEO_CONFIG,        // Purple flashing  
    NEO_ERROR,         // Red flashing
    NEO_TRIGGER_FLASH  // White trigger flash (3 times)
};

// FIXED: Move constants outside class to avoid initialization issues
const uint8_t NEOPIXEL_TOTAL_FLASHES = 3;
const uint32_t NEOPIXEL_FLASH_ON_MS = 100;
const uint32_t NEOPIXEL_FLASH_OFF_MS = 100;
const float NEOPIXEL_SMOOTHING_ALPHA = 0.3f;

class NeoPixelController {
private:
    Adafruit_NeoPixel* strip;
    bool initialized;
    
    // Trigger flash state
    bool trigger_flash_active;
    uint32_t trigger_flash_start;
    uint8_t flash_count;
    
public:
    // FIXED: Make smoothing state public to allow external access
    float smoothed_distance;
    float smoothed_strength;
    bool smoothing_initialized;
    
    NeoPixelController();
    ~NeoPixelController();
    
    bool init(uint8_t pin, uint8_t num_pixels = 1);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void clear();
    bool isReady() const { return initialized && strip != nullptr; }
    
    // Trigger flash functions
    void startTriggerFlash();
    bool isTriggerFlashing() const { return trigger_flash_active; }
    
    // FIXED: Add public accessors for trigger flash state
    uint32_t getTriggerFlashStart() const { return trigger_flash_start; }
    void setTriggerFlashInactive() { trigger_flash_active = false; }
    uint8_t getFlashCount() const { return flash_count; }
    void setFlashCount(uint8_t count) { flash_count = count; }
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
uint32_t getTriggerFlashColor(uint32_t elapsed_ms, uint8_t* flash_count_out);

#endif // NEOPIXEL_INTEGRATION_H