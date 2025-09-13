LiDAR RP2040 Controller Field Manual

This manual offers an in-depth introduction to the LiDAR RP2040 Controller, a dual-core system engineered for accurate distance and speed measurements. The controller employs a high-speed LiDAR sensor that consistently tracks configurable trigger conditions, activating an output on TRIG_PULSE_LOW_PIN (GPIO16) for three seconds. The firmware, identified as version v6.3 Dual-Core Architecture, brings improved performance and robust capabilities.

Key Features

- **Dual-Core Functionality**: Core 0 oversees high-speed LiDAR communication (up to 1000Hz), while Core 1 is responsible for data processing and system logic.
- **GUI Configuration**: Real-time setup is facilitated through a serial interface.
- **NeoPixel Status Display**: Provides visual indicators for distance (represented as a heat map), speed (shown through saturation), trigger events (indicated by a white flash), and overall system status.
- **Improved Performance**: Incorporates adaptive velocity calculation with noise filtering and supports operational modes of 800Hz/1000Hz.
- **LittleFS Storage**: Maintains non-volatile configuration settings with checksum verification.
- **Thread-Safe Operation**: Employs mutex-protected inter-core communication and atomic buffer operations.

2. GUI Configuration System

The controller can be set up through a serial interface. To access configuration mode, power on the device and send any character to the serial terminal within a 15-second time frame. The NeoPixel will shift from a blue breathing pattern to a purple flashing pattern, indicating that the device is in configuration mode.

Configuration Commands

The GUI utilizes a packet-based protocol structured as 0x7E [CMD] [LEN] [PAYLOAD...] [CHECKSUM].

- 'S': Retrieve system status (no payload).
- 'D'/'d': Get/Set distance thresholds (Position (0-7), Value (cm)).
- 'V'/'v': Get/Set velocity thresholds (Type ('m'/'x'), Position, Value).
- 'M'/'m': Get/Set trigger mode (1=Distance only, 2=Distance+Velocity).
- 'G'/'g': Get/Set debug output (0=Disabled, 1=Enabled).
- 'W': Save configuration (no payload).
- 'R': System reset (no payload).
- 'F': Factory reset (no payload).

3. Dual-Core Operation Flow

**Core 0: LiDAR Communication & Data Collection**

This core manages high-speed sensor communication and data buffering. It initializes the LiDAR sensor at a baud rate of 460800 for rapid operation, establishes the frequency at 800Hz or 1000Hz according to configuration, validates incoming data frames prior to placing them into a circular buffer, and monitors for communication timeouts while executing graduated recovery.

**Core 1: Data Processing & System Logic**

This core is tasked with all data processing and user interaction. It retrieves data frames from the atomic buffer for analysis, calculates velocity utilizing median filtering over a 15-frame history, assesses trigger conditions (distance and velocity) with debouncing and a 3-second latch, and oversees the NeoPixel display, GUI commands, and configuration storage while also monitoring physical configuration switches.

5. Configuration

The device's operation can be adjusted through physical switches and the GUI. The trigger output (TRIG_PULSE_LOW_PIN) is an active-low signal that engages when an object satisfies the configured distance and, if applicable, velocity thresholds.

**Trigger Logic Summary**

- **Distance Check**: The object's distance is less than or equal to the set threshold.
- **Velocity Check**: (If enabled) The object's speed is within the specified minimum/maximum range.
- **Debouncing**: A 30ms activation delay and 50ms deactivation delay help avoid false triggers due to noise.
- **Latching**: Once triggered, the output remains active for three seconds.

6. Hardware & Wiring Diagram

Proper wiring is essential for functionality. The table below outlines the key GPIO pin assignments:

| GPIO Pin | Function               | Direction | Description                               |
|----------|-----------------------|-----------|-------------------------------------------|
| GP0      | UART TX               | Output    | Serial transmit to LiDAR sensor           |
| GP1      | UART RX               | Input     | Serial receive from LiDAR sensor          |
| GP10     | S1 Switch             | Input     | Configuration switch bit 0                |
| GP11     | S2 Switch             | Input     | Configuration switch bit 1                |
| GP12     | S4 Switch             | Input     | Configuration switch bit 2                |
| GP13     | Rotary Connection      | Input     | Switch board connection detection          |
| GP14     | External Trigger       | Input     | External trigger input signal              |
| GP15     | External Enable        | Input     | External enable/disable signal            |
| GP16     | Trigger Output         | Output    | Main trigger output (active low)          |
| GP18     | NeoPixel Data          | Output    | WS2812B data signal                       |
| GP25     | Status LED            | Output    | Onboard status indicator                   |

10. Troubleshooting with NeoPixel & LED Indicators

The system offers visual feedback to assist with troubleshooting.

**NeoPixel Status Indicators**

- **Heat Map Colors**: Normal operation. Red (close) → Yellow (medium) → Blue (far). Saturation reflects velocity.
- **White 5Hz Flash**: Trigger active, synchronized with the 3-second trigger latch.
- **Blue Breathing**: System initialization. Occurs during startup and LiDAR configuration.
- **Purple Flashing**: Configuration mode. Ready for serial commands. A green glow signifies a successful command.
- **Red 4Hz Flash**: Error state. Communication timeout or sensor malfunction. Check LiDAR connections.
- **Off**: Power/hardware issue. Verify power supply or GPIO18 connection.

**Traditional Status LED (GPIO25)**

- **1000ms Blink**: Normal operation. Indicates a healthy system heartbeat.
- **100ms Blink**: Configuration mode. The GUI interface is active.
- **200ms Blink**: Buffer warning. Frame processing lag detected.
- **300ms Blink**: Communication timeout. LiDAR sensor is unresponsive.
- **10ms Blink**: Critical buffer state. Imminent frame loss.

12. RP2040 Programming Guide

To program the controller, utilize the Arduino IDE with the RP2040 board package.

**Step 1: Arduino IDE Setup**

- Install the latest version of the Arduino IDE.
- Add the RP2040 board manager URL in File > Preferences: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json.
- Install the "Raspberry Pi Pico/RP2040" board package from Tools > Board > Boards Manager.
- Use the Library Manager to install the Adafruit NeoPixel and LittleFS libraries.

**Step 2: Board Configuration**

Set the following parameters in the Arduino IDE:

| Setting                | Value              | Description                              |
|-----------------------|--------------------|------------------------------------------|
| Board                 | Generic RP2040     |                                          |
| CPU Speed             | 133 MHz            |                                          |
| USB Stack             | Adafruit TinyUSB    |                                          |
| Flash Size            | 2MB (Sketch: 1MB, FS: 1MB) |                                 |

**Step 3: Programming Methods**

- **USB Programming (Boot Mode)**: Press the BOOTSEL button while connecting the device, and it will show as a mass storage device. Select the appropriate serial port and upload the sketch.
- **Picoprobe/Debug Probe**: Connect to the SWD pins and select Sketch > Upload Using Programmer after choosing Picoprobe (CMSIS-DAP) from Tools > Programmer.

13. Appendix: Configuration Parameters

While most parameters can now be configured via the GUI, several critical settings can be adjusted at compile time.

| Parameter Name              | Default      | Description & Impact                        |
|-----------------------------|--------------|---------------------------------------------|
| USE_1000HZ_MODE             | true         | Chooses between 1000Hz or 800Hz operation.|
| FRAME_BUFFER_SIZE            | 32/24        | Capacity of inter-core circular buffer.    |
| MIN_STRENGTH_THRESHOLD       | 200          | Minimum LiDAR signal quality required.     |
| CONFIG_MODE_TIMEOUT_MS       | 15000        | The 15-second window for entering GUI configuration mode. |
| VELOCITY_DEADBAND_THRESHOLD   | 1.0 cm/s     | Minimum velocity change to register movement. |
| TRIGGER_LATCH_DURATION_MS    | 3000         | Duration for which the trigger output remains active. |

14. Version History

This firmware version marks a significant redesign and transition from the earlier ATtiny1634 architecture.

- **Version 6.3 (Current)**: Introduced the dual-core RP2040 architecture, real-time GUI configuration, LittleFS storage, and improved performance features.
- **Version 14.0 (Legacy)**: The final release for the ATtiny1634 platform, which featured a single-core architecture, EEPROM storage, and a maximum frame rate of 50Hz. The RP2040 version delivers a 16-20x enhancement in frame rate and 264x more RAM compared to its predecessor.
