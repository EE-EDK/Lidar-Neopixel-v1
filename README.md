LiDAR RP2040 Controller Field Manual
This manual provides a comprehensive overview of the LiDAR RP2040 Controller, a dual-core system designed for precise distance and velocity measurements. The controller uses a high-speed LiDAR sensor and continuously monitors for configurable trigger conditions, activating an output on TRIG_PULSE_LOW_PIN (GPIO16) for a duration of 3 seconds. The firmware, version v6.3 Dual-Core Architecture, features enhanced performance and robust functionality.
Key Features
Dual-Core Operation: Core 0 manages high-speed LiDAR communication (up to 1000Hz), while Core 1 processes data and system logic.
GUI Configuration: Real-time configuration via a serial interface.
NeoPixel Status Display: Visual feedback for distance (heat map), velocity (saturation), trigger events (white flash), and system status.
Enhanced Performance: Features adaptive velocity calculation with noise filtering and supports 800Hz/1000Hz operation modes.
LittleFS Storage: Stores non-volatile configuration settings with checksum validation.
Thread-Safe Operation: Utilizes mutex-protected inter-core communication and atomic buffer operations.
2. GUI Configuration System
The controller can be configured via a serial interface. To enter configuration mode, you must power on the device and send any character to the serial terminal within a 15-second window. The NeoPixel will change from a blue breathing pattern to a purple flashing pattern to indicate that the device is in configuration mode.
Configuration Commands
The GUI uses a packet-based protocol with a format of 0x7E [CMD] [LEN] [PAYLOAD...] [CHECKSUM].


Command
Function
Payload
'S'
Get system status
None
'D'/'d'
Get/Set distance thresholds
Position (0-7), Value (cm)
'V'/'v'
Get/Set velocity thresholds
Type ('m'/'x'), Position, Value
'M'/'m'
Get/Set trigger mode
1=Distance only, 2=Distance+Velocity
'G'/'g'
Get/Set debug output
0=Disabled, 1=Enabled
'W'
Save configuration
None
'R'
System reset
None
'F'
Factory reset
None

3. Dual-Core Operation Flow
Core 0: LiDAR Communication & Data Acquisition
This core is responsible for high-speed sensor communication and data buffering.
Initializes the LiDAR sensor at 460800 baud for high-speed operation.
Sets the frequency to 800Hz or 1000Hz based on the configuration.
Validates incoming data frames before inserting them into a circular buffer.
Monitors for communication timeouts and performs graduated recovery.
Core 1: Data Processing & System Logic
This core handles all data processing and user interaction.
Pulls data frames from the atomic buffer for processing.
Calculates velocity using median filtering across a 15-frame history.
Evaluates trigger conditions (distance and velocity) with debouncing and a 3-second latch.
Manages the NeoPixel display, GUI commands, and configuration storage.
Monitors physical configuration switches.
5. Configuration
The device's behavior is configured via physical switches and the GUI. The trigger output (TRIG_PULSE_LOW_PIN) is an active-low signal that activates when an object meets the configured distance and, if enabled, velocity thresholds.
Trigger Logic Summary
Distance Check: The object's distance is less than or equal to the configured threshold.
Velocity Check: (If enabled) The object's velocity is within the set min/max range.
Debouncing: A 30ms on-delay and 50ms off-delay prevent false triggers from noise.
Latching: Once triggered, the output remains active for a 3-second duration.
6. Hardware & Wiring Diagram
Proper wiring is crucial for operation. The following table summarizes the key GPIO pin assignments.
GPIO Pin
Function
Direction
Description
GP0
UART TX
Output
Serial transmit to LiDAR sensor
GP1
UART RX
Input
Serial receive from LiDAR sensor
GP10
S1 Switch
Input
Configuration switch bit 0
GP11
S2 Switch
Input
Configuration switch bit 1
GP12
S4 Switch
Input
Configuration switch bit 2
GP13
Rotary Connection
Input
Switch board connection detect
GP14
External Trigger
Input
External trigger input signal
GP15
External Enable
Input
External enable/disable signal
GP16
Trigger Output
Output
Main trigger output (active low)
GP18
NeoPixel Data
Output
WS2812B data signal
GP25
Status LED
Output
Onboard status indicator

10. Troubleshooting with NeoPixel & LED Indicators
The system provides visual feedback to help with troubleshooting.
NeoPixel Status Indicators
NeoPixel Pattern
Meaning & What to Check
Heat Map Colors
Normal Operation. Red (close) → Yellow (medium) → Blue (far). Saturation indicates velocity.
White 5Hz Flash
Trigger Active. Synchronized with the 3-second trigger latch.
Blue Breathing
System Initialization. Occurs during startup and LiDAR configuration.
Purple Flashing
Configuration Mode. Ready for serial commands. A green glow indicates a successful command.
Red 4Hz Flash
Error State. Communication timeout or sensor malfunction. Check LiDAR connections.
Off
Power/Hardware Issue. Check the power supply or GPIO18 connection.

Traditional Status LED (GPIO25)
LED Pattern
Meaning & System State
1000ms Blink
Normal Operation. Indicates a healthy system heartbeat.
100ms Blink
Configuration Mode. The GUI interface is active.
200ms Blink
Buffer Warning. Frame processing lag detected.
300ms Blink
Communication Timeout. LiDAR sensor is not responding.
10ms Blink
Critical Buffer State. Imminent frame loss.

12. RP2040 Programming Guide
To program the controller, you'll use the Arduino IDE with the RP2040 board package.
Step 1: Arduino IDE Setup
Install the latest Arduino IDE.
Add the RP2040 board manager URL in File > Preferences: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json.
Install the "Raspberry Pi Pico/RP2040" board package from Tools > Board > Boards Manager.
Install the Adafruit NeoPixel and LittleFS libraries using the Library Manager.
Step 2: Board Configuration
Configure the following settings in the Arduino IDE:
Board: Generic RP2040
CPU Speed: 133 MHz
USB Stack: Adafruit TinyUSB
Flash Size: 2MB (Sketch: 1MB, FS: 1MB)
Step 3: Programming Methods
USB Programming (Boot Mode): Hold the BOOTSEL button while plugging in the device, and it will appear as a mass storage device. Select the correct serial port and upload the sketch.
Picoprobe/Debug Probe: Connect the SWD pins and use Sketch > Upload Using Programmer after selecting Picoprobe (CMSIS-DAP) from Tools > Programmer.
13. Appendix: Configuration Parameters
While most parameters are now GUI-configurable, some key settings can be adjusted at compile-time.
Parameter Name
Default
Description & Impact
USE_1000HZ_MODE
true
Selects 1000Hz or 800Hz operation mode.
FRAME_BUFFER_SIZE
32/24
Inter-core circular buffer capacity.
MIN_STRENGTH_THRESHOLD
200
Minimum LiDAR signal quality required.
CONFIG_MODE_TIMEOUT_MS
15000
The 15-second window for entering GUI configuration mode.
VELOCITY_DEADBAND_THRESHOLD
1.0 cm/s
Minimum velocity change to register movement.
TRIGGER_LATCH_DURATION_MS
3000
How long the trigger output remains active.

14. Version History
This firmware version is a major redesign and migration from the previous ATtiny1634 architecture.
Version 6.3 (Current): Introduced the dual-core RP2040 architecture, real-time GUI configuration, LittleFS storage, and enhanced performance features.
Version 14.0 (Legacy): The final release for the ATtiny1634 platform, which featured a single-core architecture, EEPROM storage, and a maximum frame rate of 50Hz. The RP2040 version offers a 16-20x improvement in frame rate and 264x more RAM compared to the legacy system.
