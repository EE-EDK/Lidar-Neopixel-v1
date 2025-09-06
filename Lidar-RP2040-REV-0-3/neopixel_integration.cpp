// neopixel_integration.cpp - Implementation using Adafruit NeoPixel library
#include "neopixel_integration.h"

// Global instance
NeoPixelController neopixel;

NeoPixelController::NeoPixelController() 
    : strip(nullptr), initialized(false), trigger_flash_active(false), 
      trigger_flash_start(0), flash_count(0), smoothed_distance(0), 
      smoothed_strength(0), smoothing_initialized(false) {
}

NeoPixelController::~NeoPixelController() {
    if (strip) {
        delete strip;
        strip = nullptr;
    }
}

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
    strip->show();  // Initialize all pixels to 'off'
    strip->setBrightness(200);  // Set to ~78% brightness
    
    initialized = true;
    clear();
    
    return true;
}

void NeoPixelController::setColor(uint8_t r, uint8_t g, uint8_t b) {
    if (!isReady()) return;
    
    strip->setPixelColor(0, strip->Color(r, g, b));
    strip->show();
}

void NeoPixelController::clear() {
    if (!isReady()) return;
    
    strip->clear();
    strip->show();
}

void NeoPixelController::startTriggerFlash() {
    if (!isReady()) return;
    
    trigger_flash_active = true;
    trigger_flash_start = millis();
    flash_count = 0;
}

// ===== HIGH-LEVEL INTERFACE FUNCTIONS =====

bool initNeoPixel(uint8_t pin) {
    return neopixel.init(pin, 1);  // Single pixel
}

void triggerNeoPixelFlash() {
    neopixel.startTriggerFlash();
}

void updateNeoPixelStatus(NeoPixelMode mode, uint16_t distance, float velocity, uint8_t strength) {
    if (!neopixel.isReady()) return;
    
    static uint32_t last_update = 0;
    uint32_t now = millis();
    
    // Rate limit updates to 50Hz (except during trigger flash)
    if (!neopixel.isTriggerFlashing() && now - last_update < 20) return;
    last_update = now;
    
    // Handle trigger flash (highest priority)
    if (neopixel.isTriggerFlashing()) {
        uint32_t elapsed = now - neopixel.getTriggerFlashStart();
        uint8_t flash_count;
        uint32_t color = getTriggerFlashColor(elapsed, &flash_count);
        
        // Update internal flash count
        neopixel.setFlashCount(flash_count);
        
        // Check if flash sequence is complete
        if (flash_count >= NEOPIXEL_TOTAL_FLASHES && 
            elapsed > (NEOPIXEL_TOTAL_FLASHES * 
                      (NEOPIXEL_FLASH_ON_MS + NEOPIXEL_FLASH_OFF_MS))) {
            neopixel.setTriggerFlashInactive();
            // Fall through to normal mode processing
        } else {
            // Extract RGB and set color
            uint8_t r = (color >> 16) & 0xFF;
            uint8_t g = (color >> 8) & 0xFF;
            uint8_t b = color & 0xFF;
            neopixel.setColor(r, g, b);
            return;
        }
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
            // This mode is handled above
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
    
    // Extract RGB and set color
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;
    neopixel.setColor(r, g, b);
}

// ===== UTILITY FUNCTIONS =====

uint32_t calculateDistanceColor(uint16_t distance_cm, float velocity_cm_s, uint8_t signal_strength) {
    // Apply smoothing to reduce noise and flickering
    if (!neopixel.smoothing_initialized) {
        neopixel.smoothed_distance = distance_cm;
        neopixel.smoothed_strength = signal_strength;
        neopixel.smoothing_initialized = true;
    } else {
        neopixel.smoothed_distance = NEOPIXEL_SMOOTHING_ALPHA * distance_cm + 
                                    (1.0f - NEOPIXEL_SMOOTHING_ALPHA) * neopixel.smoothed_distance;
        neopixel.smoothed_strength = NEOPIXEL_SMOOTHING_ALPHA * signal_strength + 
                                    (1.0f - NEOPIXEL_SMOOTHING_ALPHA) * neopixel.smoothed_strength;
    }
    
    // Use smoothed values
    float distance = neopixel.smoothed_distance;
    float strength = neopixel.smoothed_strength;
    
    // Clamp distance to valid range
    if (distance < MIN_DISTANCE_CM) distance = MIN_DISTANCE_CM;
    if (distance > MAX_DISTANCE_CM) distance = MAX_DISTANCE_CM;
    
    // Calculate position in range (0.0 = close, 1.0 = far)
    float position = (distance - MIN_DISTANCE_CM) / (MAX_DISTANCE_CM - MIN_DISTANCE_CM);
    
    // Determine direction
    bool approaching = velocity_cm_s < -5.0f;  // Moving toward sensor
    
    uint8_t r, g, b;
    
    if (approaching) {
        // Approaching: Red (close) to Yellow (far)
        r = 255;
        g = (uint8_t)(255 * position);  // More yellow when far
        b = 0;
    } else {
        // Receding/stationary: Green (close) to Blue (far)
        r = 0;
        g = (uint8_t)(255 * (1.0f - position));  // More green when close
        b = (uint8_t)(255 * position);           // More blue when far
    }
    
    // Apply brightness based on signal strength (30-100%)
    float brightness = 0.3f + 0.7f * (strength / 255.0f);
    r = (uint8_t)(r * brightness);
    g = (uint8_t)(g * brightness);
    b = (uint8_t)(b * brightness);
    
    // Return as 32-bit color (0x00RRGGBB format)
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

uint32_t getTriggerFlashColor(uint32_t elapsed_ms, uint8_t* flash_count_out) {
    const uint32_t FLASH_CYCLE_MS = NEOPIXEL_FLASH_ON_MS + NEOPIXEL_FLASH_OFF_MS;
    
    uint8_t current_flash = elapsed_ms / FLASH_CYCLE_MS;
    uint32_t cycle_position = elapsed_ms % FLASH_CYCLE_MS;
    
    if (flash_count_out) {
        *flash_count_out = current_flash;
    }
    
    // If we're still within the 3 flash limit and in the "on" portion of the cycle
    if (current_flash < NEOPIXEL_TOTAL_FLASHES && cycle_position < NEOPIXEL_FLASH_ON_MS) {
        return 0x00FFFFFF;  // Full brightness white (0x00RRGGBB)
    } else {
        return 0x00000000;  // Off
    }
}

uint32_t getStatusColor(NeoPixelMode mode, uint32_t time_ms) {
    switch (mode) {
        case NEO_INITIALIZING:
            {
                // Breathing blue (2 second cycle)
                float phase = (time_ms % 2000) / 2000.0f;
                float brightness = 0.5f + 0.5f * sin(phase * 2 * PI);
                uint8_t blue = (uint8_t)(200 * brightness);
                return (uint32_t)blue;  // 0x000000BB
            }
            
        case NEO_CONFIG:
            {
                // Purple flashing (1Hz)
                bool on = (time_ms % 1000) < 500;
                return on ? 0x00800080 : 0x00000000;  // Purple or off
            }
            
        case NEO_ERROR:
            {
                // Red fast flashing (4Hz)
                bool on = (time_ms % 250) < 125;
                return on ? 0x00FF0000 : 0x00000000;  // Red or off
            }
            
        default:
            return 0x00000000;  // Off
    }
}