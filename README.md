# dual-watch-winder-esp32

ESP32-based dual watch winder for a 3D-printed enclosure. Controls two 28BYJ-48 stepper motors through ULN2003 drivers and exposes a web interface over WiFi for configuration, status, and manual control. Features include configurable TPD (Turns Per Day), turbo mode for quick spin/testing, and a 3-position hardware switch for preset profiles.

## Key features
- Dual stepper motor control (two independent drums)
- Web interface hosted on the ESP32 for configuration and control
- WiFi connectivity (Station mode with optional AP fallback)
- Configurable TPD and rotation direction per drum
- Turbo mode for quick manual spin/testing
- 3-position hardware switch for preset profiles
- PlatformIO project structure
- Uses ArduinoJson (for configuration) and AccelStepper (for precise motion control)
- Dark/light theme web UI with responsive design

## Hardware requirements
- 1x ESP32 development board (e.g., ESP32-DevKitC / WROOM-32)
- 2x 28BYJ-48 5V stepper motors
- 2x ULN2003 stepper driver boards
- 5V power supply capable of at least 1.5–2A (for two motors + ESP32 headroom)
- 1x 3-position DPDT toggle switch (for presets)
- Connection wires (Dupont), shared ground between ESP32 and motor drivers
- 3D printed parts for the mechanical assembly

## Wiring and pin configuration
GPIO assignments as defined in `include/config.h`:

**Motor 1 (Left):**
- ULN2003 IN1 → ESP32 GPIO 13
- ULN2003 IN2 → ESP32 GPIO 12  
- ULN2003 IN3 → ESP32 GPIO 14
- ULN2003 IN4 → ESP32 GPIO 27

**Motor 2 (Right):**
- ULN2003 IN1 → ESP32 GPIO 26
- ULN2003 IN2 → ESP32 GPIO 25
- ULN2003 IN3 → ESP32 GPIO 33
- ULN2003 IN4 → ESP32 GPIO 32

**3-position DPDT switch:**
- Pin A → ESP32 GPIO 16 (with INPUT_PULLUP)
- Pin B → ESP32 GPIO 17 (with INPUT_PULLUP)
- Common → GND

**Status LED (optional):**
- LED → ESP32 GPIO 4 (set to -1 in config.h to disable)

**Power notes:**
- Motors/ULN2003 boards powered from 5V
- ESP32 logic is 3.3V; ensure a common ground with motor driver boards
- 28BYJ-48 uses 4096 steps per revolution in half-step mode

## Setup and installation
1) Install VS Code and the PlatformIO extension.
2) Clone this repository or open the folder in VS Code and let PlatformIO index dependencies.
3) Configure WiFi credentials in `include/config.h`:
   ```cpp
   static const char* WIFI_SSID = "YOUR_WIFI_NETWORK";
   static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
   ```
4) Build and flash:
   ```bash
   pio run
   pio run -t upload
   ```
5) Open the serial monitor to obtain the device IP:
   ```bash
   pio device monitor -b 115200
   ```
6) Browse to the ESP32's IP address to access the web interface.

## Configuration and usage
- **Web interface:** Navigate to the ESP32's IP address shown in serial output
- **WiFi setup:** If STA connection fails, device creates "Winder-Setup" access point
- **TPD configuration:** Set turns per day (0-1200) for each motor independently
- **Direction control:** Choose CW, CCW, or Alternating for each motor
- **Turbo mode:** Quick 5 or 10-minute continuous rotation for testing
- **3-position switch presets:**
  - Position 0: 500 TPD, alternating direction (both motors)  
  - Position 1: Manual control via web interface
  - Position 2: 800 TPD, Motor 1 CW, Motor 2 CCW

## Software dependencies
Managed by PlatformIO (see platformio.ini for versions):
- **ArduinoJson** (^7.0) - JSON configuration and API responses
- **AccelStepper** (^1.64) - Precise stepper motor control
- **ESP32 Arduino core** - Core framework

## Project structure
- `platformio.ini` — PlatformIO environments and library dependencies
- `src/main.cpp` — Main firmware source with embedded web interface
- `include/config.h` — Hardware configuration and WiFi credentials  
- `lib/` — Optional local libraries
- `test/` — Optional unit tests

## Web interface features
- Real-time status monitoring
- Individual motor control (TPD, direction)
- Turbo mode controls
- WiFi network scanning and setup
- Dark/light theme toggle
- Responsive design for mobile devices
- mDNS support (access via http://winder.local)

## Notes
- Always share ground between ESP32 and ULN2003 boards
- Use a robust 5V supply; current spikes occur during motor starts/acceleration
- If you change GPIOs, update both wiring and firmware constants in `include/config.h`
- The web interface is embedded in the firmware (no separate filesystem upload needed)
- Settings are automatically saved to ESP32 non-volatile storage

## License
MIT License. See LICENSE for details.

## Acknowledgments
- ArduinoJson by Benoit Blanchon
- AccelStepper by Mike McCauley and contributors
- ESP32 Arduino core maintainers and contributors