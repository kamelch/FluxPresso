# 🚀 Gaggia Enhanced Espresso Controller

> **Transform your Gaggia espresso machine into a smart, PID-controlled café-quality brewing system**

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--S3-orange.svg)](https://platformio.org/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

This project upgrades classic Gaggia espresso machines (Classic, Classic Pro, etc.) with modern electronics, precise temperature control, and advanced features like shot profiling, steam management, and WiFi connectivity.

![Gaggia Controller](https://via.placeholder.com/800x400/1a1a1a/ffffff?text=Add+Your+Setup+Photo+Here)

---

## ⚠️ **IMPORTANT SAFETY WARNING**

**This project involves HIGH VOLTAGE electrical work (110V-240V AC) and pressurized boilers.**

- You must have experience with electrical wiring and safety
- Improper installation can cause **electric shock, burns, fire, or equipment damage**
- Check your local electrical codes and regulations
- **USE AT YOUR OWN RISK** - no warranty is provided
- Consider having your installation inspected by a qualified electrician

**If you're not comfortable working with mains voltage, DO NOT attempt this modification.**

---

## ✨ Features

### 🌡️ Temperature Control
- **Dual PID Controllers**: Separate tuning for brew (93°C) and steam (145°C) modes
- **±0.1°C Precision**: Rock-solid temperature stability for perfect extractions
- **Predictive Modeling**: Anticipates temperature changes for faster recovery
- **Adaptive Preheat**: Learns optimal warmup times for your machine

### ☕ Brew Features
- **Shot Profiling**: Pre-infusion, bloom, and brew phases with customizable timings
- **Pressure Profiling**: Pump ramping for smooth flow transitions
- **Multiple Profiles**: Save and switch between different brewing recipes
- **Shot History**: Track 20 previous shots with statistics and temperature stability scores
- **Auto-Tare Scale**: Integrated HX711 load cell for weight-based dosing

### 💨 DreamSteam Technology
Advanced steam mode with intelligent boiler refilling:
- **Conservative Mode**: Gentle refills for stable temperature
- **Aggressive Mode**: Faster refills for power users
- **Adaptive Mode**: Learns your steaming patterns
- Temperature-based load detection prevents boiler depletion

### 📊 Smart Features
- **WiFi Connectivity**: Remote monitoring and control
- **Shelly Smart Plug Integration**: Auto power-on scheduling
- **Real-Time Charts**: Live temperature and pressure visualization
- **Diagnostic Mode**: Detailed PID tuning data for optimization
- **Shot Persistence**: History saved to flash memory

### 🛡️ Safety Systems
- Hardware watchdog timer (500ms timeout)
- Over-temperature protection (165°C limit)
- Thermocouple fault detection
- Zero-cross detection monitoring
- Manual pump timeout (60s max)
- Sensor failure detection
- Automatic fault recovery

---

## 🧰 Hardware Requirements

### Display Module
- **Guition JC3248W535EN** - 3.5" QSPI touchscreen (480×320)
  - ESP32-S3 microcontroller built-in
  - 16MB flash memory
  - PSRAM support
  - [Purchase Link](#) *(Add your preferred supplier)*

### Sensors & Components
| Component | Model/Type | Purpose | Approx. Cost |
|-----------|------------|---------|--------------|
| Thermocouple | K-Type with MAX31855 | Boiler temperature | $10-15 |
| Load Cell | 5kg HX711 | Weight measurement | $8-12 |
| SSR | 25A-40A Solid State Relay | Heater control | $10-20 |
| Triac Board | Zero-cross AC dimmer | Pump control | $15-25 |
| Solenoid Valve | 3-way solenoid (optional) | Flow control | $20-30 |
| Power Supply | 5V 2A | Controller power | $8-12 |
| Misc | Wires, connectors, enclosure | - | $20-30 |

**Total Hardware Cost: ~$90-150** (excluding display)

### Recommended Tools
- Soldering iron and solder
- Multimeter (essential for testing)
- Crimping tool for connectors
- Heat shrink tubing
- Wire strippers
- Basic hand tools

---

## 📋 Pin Configuration

The firmware is pre-configured for standard Gaggia wiring:

```cpp
// Gaggia Peripheral Connections
#define PIN_AC_PWM      43    // Triac Gate (Pump Control)
#define PIN_AC_ZC       44    // Zero Cross Detection
#define PIN_SOLENOID    5     // Solenoid Valve
#define PIN_MAX_CS      6     // Thermocouple Chip Select
#define PIN_MAX_DO      7     // Thermocouple Data Out
#define PIN_MAX_CLK     15    // Thermocouple Clock
#define PIN_SSR         16    // Solid State Relay (Heater)
#define PIN_HX_DT       17    // Load Cell Data
#define PIN_HX_SCK      18    // Load Cell Clock
```

*See [project_config.h](src/project_config.h) for full pin mappings*

---

## 🚀 Getting Started

### 1. Software Setup

#### Prerequisites
- [PlatformIO IDE](https://platformio.org/platformio-ide) (or PlatformIO CLI)
- [Visual Studio Code](https://code.visualstudio.com/) (recommended)
- Git

#### Clone the Repository
```bash
git clone https://github.com/kamelch/Gaggia_Project-.git
cd Gaggia_Project-
```

#### Install Dependencies
PlatformIO will automatically download all required libraries:
- LVGL v8.3.11 (UI framework)
- Adafruit MAX31855 (thermocouple)
- HX711 (load cell)
- ArduinoJson
- GFX Library for Arduino
- TAMC_GT911 (touchscreen)

### 2. Configuration

#### ⚙️ Essential Settings

Edit `src/project_config.h`:

```cpp
// 🌍 SET YOUR GRID FREQUENCY (CRITICAL!)
#define GRID_FREQ_HZ    50    // Europe: 50Hz, US/Canada: 60Hz

// 🌡️ Safety Limits
#define SAFETY_MAX_TEMP_C       165.0  // Adjust if needed

// 📡 WiFi Credentials
#define WIFI_SSID       "YourNetworkName"
#define WIFI_PASSWORD   "YourPassword"

// 🔌 Shelly Smart Plug (optional)
#define SHELLY_ENABLED  true
#define SHELLY_IP       "192.168.1.100"
```

#### 🎛️ PID Tuning (Optional - Defaults are good)
```cpp
// Brew Mode PID
#define BREW_PID_KP     25.0f
#define BREW_PID_KI     0.8f
#define BREW_PID_KD     8.0f

// Steam Mode PID
#define STEAM_PID_KP    35.0f
#define STEAM_PID_KI    1.2f
#define STEAM_PID_KD    12.0f
```

### 3. Build & Flash

#### Using PlatformIO IDE (VS Code)
1. Open project folder in VS Code
2. Click PlatformIO icon (alien head) in sidebar
3. Select `esp32-s3-devkitc-1` environment
4. Click "Build" (✓) or "Upload" (→)

#### Using Command Line
```bash
# Build firmware
pio run -e esp32-s3-devkitc-1

# Upload to device
pio run -e esp32-s3-devkitc-1 -t upload

# Monitor serial output
pio device monitor -b 115200
```

### 4. Testing

#### Pre-Flight Checks
1. ✅ Verify all sensor connections
2. ✅ Test zero-cross detection (watch serial monitor)
3. ✅ Calibrate thermocouple (boiling water = 100°C)
4. ✅ Calibrate scale with known weights
5. ✅ Test emergency shutdown (set max temp to 40°C temporarily)

#### First Power-On
```bash
# Watch the serial monitor for diagnostics
pio device monitor -b 115200
```

Expected output:
```
========================================
Enhanced Espresso Controller v2.0
with PID & DreamSteam
========================================

Display initialized: 480x320
Thermocouple OK: 23.4°C
Scale OK: 0.0g
Zero-cross detected: OK
WiFi connected: 192.168.1.x
System ready!
```

---

## 🎯 How to Use

### Basic Operation

#### Startup Sequence
1. Turn on machine (or use Shelly auto-power)
2. Wait for preheat (~5-10 minutes)
3. Green "READY" indicator appears
4. Pull your shot!

#### Brewing Coffee
1. **Load portafilter** → Scale auto-tares
2. **Tap brew button** → Starts shot profile:
   - Pre-infusion (9 bars, 6 seconds)
   - Bloom pause (1 second)
   - Full brew (9 bars, 25 seconds)
3. **Auto-stop** when target weight reached
4. Drain phase cleans 3-way valve

#### Steaming Milk
1. **Switch to steam mode** (tap steam icon)
2. Wait for temperature ramp (145°C)
3. **Open steam wand** → DreamSteam automatically refills boiler
4. **Close wand** → Returns to brew temperature

### Advanced Features

#### Custom Shot Profiles
```
Main Menu → Profiles → Create New
- Set pre-infusion time (0-15s)
- Set bloom pause (0-5s)
- Set brew time (15-60s)
- Adjust temperatures (85-95°C)
- Save profile
```

#### Shot History Analysis
```
Main Menu → History
- View last 20 shots
- Temperature stability scores
- Extraction times and yields
- Compare profile performance
```

#### Manual Mode
```
Main Menu → Manual
- Direct pump power control (0-100%)
- Temperature override
- ⚠️ Safety timeout after 60 seconds
```

---

## 📐 Wiring Diagram

### Simplified Connection Overview

```
┌─────────────────────────────────────┐
│   GUITION JC3248W535EN Display      │
│         (ESP32-S3)                  │
└──────┬──────────────────────────────┘
       │
       ├─[P1]─► Triac Board ──────► Pump (AC)
       │        (Zero-cross input)
       │
       ├─[P2]─► SSR Relay ────────► Heating Element (AC)
       │     └─► Solenoid Valve
       │     └─► MAX31855 ──────► K-Type Thermocouple
       │
       └─[P3]─► HX711 ────────────► Load Cell (Scale)
```

### Detailed Wiring Guide
See [WIRING.md](docs/WIRING.md) for:
- Step-by-step connection diagrams
- Safety grounding requirements
- AC voltage handling best practices
- Troubleshooting common issues

**🎨 HTML Wiring Diagram:** Open `Diagram_index.html` in your browser for an interactive visual guide.

---

## 🔧 Calibration

### Thermocouple Calibration
```cpp
// In your code, adjust MAX31855 offset if needed
// Boiling water should read 100°C ±2°C at sea level
```

### Scale Calibration
1. Place known weight (e.g., 100g) on scale
2. Note raw reading in serial monitor
3. Calculate calibration factor:
   ```
   factor = raw_reading / actual_weight
   ```
4. Update in code or UI settings

### PID Tuning (Advanced Users)
Use Ziegler-Nichols or auto-tune method:
```
Enable diagnostics:
#define ENABLE_PID_DIAGNOSTICS true

Monitor PID output in serial console
Adjust Kp, Ki, Kd for your machine's thermal mass
```

---

## 📊 Performance Specifications

| Metric | Specification |
|--------|---------------|
| Temperature Accuracy | ±0.1°C |
| Temperature Stability | ±0.5°C during extraction |
| PID Update Rate | 20 Hz (50ms) |
| UI Refresh Rate | 20 Hz |
| Shot Timer Resolution | 1ms |
| Scale Resolution | 0.1g |
| WiFi Update Rate | 5 Hz (200ms) |
| Max Shot Duration | 60 seconds (safety limit) |

---

## 🐛 Troubleshooting

### Common Issues

#### ❌ Thermocouple reads 900°C or NaN
- **Cause:** Disconnected or faulty sensor
- **Fix:** Check wiring, ensure good connection to boiler

#### ❌ Zero-cross detection fails
- **Cause:** Incorrect AC phase tap or damaged optocoupler
- **Fix:** Verify AC connections, test with oscilloscope

#### ❌ Temperature oscillates wildly
- **Cause:** PID tuning incorrect for your machine
- **Fix:** Reduce Kp gain, increase derivative filter

#### ❌ WiFi won't connect
- **Cause:** Wrong credentials or 5GHz network
- **Fix:** Use 2.4GHz network, check SSID/password

#### ❌ Screen doesn't respond
- **Cause:** Touch calibration or frozen firmware
- **Fix:** Hard reset, check LVGL configuration

### Debug Mode
Enable verbose logging:
```cpp
#define CORE_DEBUG_LEVEL 3
```

View logs:
```bash
pio device monitor -b 115200 --filter esp32_exception_decoder
```

---

## 🏗️ Project Structure

```
Gaggia_Project-/
├── src/
│   ├── main.cpp              # Entry point
│   ├── project_config.h      # All configuration
│   ├── hal.cpp/h             # Hardware abstraction
│   ├── common_types.h        # Shared data structures
│   │
│   ├── controllers/          # Control algorithms
│   │   ├── pid_controller.h
│   │   └── dreamsteam_controller.h
│   │
│   ├── logic/                # Core logic
│   │   ├── espresso_sm.h     # State machine
│   │   └── safety.h          # Safety checks
│   │
│   ├── tasks/                # FreeRTOS tasks
│   │   ├── core_task.cpp     # Hardware control loop
│   │   ├── ui_task.cpp       # Display updates
│   │   └── network_task.cpp  # WiFi/Shelly
│   │
│   ├── ui/                   # User interface
│   │   └── decent_ui.cpp     # LVGL UI code
│   │
│   └── utils/                # Helper functions
│       ├── sensor_filters.h
│       ├── pump_ramp.h
│       ├── temp_predictor.h
│       └── shot_history.h
│
├── test/                     # Unit tests
│   ├── test_pid.cpp
│   ├── test_safety.cpp
│   └── test_state_machine.cpp
│
├── platformio.ini            # Build configuration
├── partitions_16MB.csv       # Flash memory layout
├── Diagram_index.html        # Wiring diagram
└── README.md                 # This file
```

---

## 🧪 Testing

Run unit tests (host machine only):
```bash
pio test -e native
```

Tests cover:
- ✅ PID controller stability
- ✅ Safety limit enforcement
- ✅ State machine transitions
- ✅ Sensor filtering algorithms

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Guidelines
- Follow existing code style
- Add unit tests for new features
- Update documentation
- Test thoroughly on hardware

---

## 📜 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

### Third-Party Licenses
- LVGL: MIT License
- MAX31855 Library: BSD License
- HX711 Library: MIT License

---

## 🙏 Acknowledgments

- **Gaggia Community** - For documenting machine internals
- **Home-Barista Forum** - PID tuning insights
- **r/espresso** - Testing feedback and encouragement
- **LVGL Team** - Excellent embedded GUI library
- **PlatformIO** - Seamless ESP32 development

---

## 📚 Resources

### Documentation
- [Full Installation Guide](docs/INSTALLATION.md) *(create this)*
- [PID Tuning Guide](docs/PID_TUNING.md) *(create this)*
- [API Reference](docs/API.md) *(create this)*

### Community
- [Discussion Forum](https://github.com/kamelch/Gaggia_Project-/discussions)
- [Report Issues](https://github.com/kamelch/Gaggia_Project-/issues)
- [Project Wiki](https://github.com/kamelch/Gaggia_Project-/wiki)

### Related Projects
- [Gaggiuino](https://github.com/Zer0-bit/gaggiuino) - Alternative controller
- [Decent Espresso](https://decentespresso.com/) - Commercial inspiration
- [PID Controllers Guide](https://en.wikipedia.org/wiki/PID_controller)

---

## 📸 Gallery

*Add photos of your build here!*

| Build Stage | Description |
|-------------|-------------|
| ![Install](https://via.placeholder.com/300x200) | Display mounted on machine |
| ![Wiring](https://via.placeholder.com/300x200) | Clean wiring inside machine |
| ![UI](https://via.placeholder.com/300x200) | Beautiful touchscreen UI |
| ![Shot](https://via.placeholder.com/300x200) | Perfect espresso extraction |

---

## 🎯 Roadmap

- [x] Basic PID temperature control
- [x] Shot profiling with pre-infusion
- [x] DreamSteam technology
- [x] WiFi connectivity
- [x] Shot history tracking
- [ ] Bluetooth app (iOS/Android)
- [ ] Cloud data logging
- [ ] Machine learning shot optimization
- [ ] Pressure sensor integration
- [ ] Flow meter support
- [ ] Multi-language UI

---

## 💡 FAQ

**Q: Will this work on my Gaggia [model]?**  
A: Tested on Classic and Classic Pro. Should work on any single-boiler Gaggia with similar internals.

**Q: Do I need the scale?**  
A: No, it's optional. The controller works fine without weight-based dosing.

**Q: Can I use a different display?**  
A: Theoretically yes, but you'd need to port the LVGL UI and pin mappings.

**Q: How long does installation take?**  
A: Expect 4-8 hours for first-time installation with testing.

**Q: What's the warranty on my machine after modding?**  
A: This modification will **void your manufacturer warranty**. Proceed accordingly.

**Q: Can I revert to stock?**  
A: Yes, all modifications are reversible. Keep original parts!

---

## 📞 Support

- 🐛 Found a bug? [Open an issue](https://github.com/kamelch/Gaggia_Project-/issues)
- 💡 Have an idea? [Start a discussion](https://github.com/kamelch/Gaggia_Project-/discussions)
- 📧 Email: *your-email@example.com* *(add your contact)*

---

## ⭐ Star History

If you find this project useful, please consider giving it a star! ⭐

[![Star History Chart](https://api.star-history.com/svg?repos=kamelch/Gaggia_Project-&type=Date)](https://star-history.com/#kamelch/Gaggia_Project-&Date)

---

<div align="center">

**Made with ☕ by coffee nerds, for coffee nerds**

[⬆ Back to Top](#-gaggia-enhanced-espresso-controller)

</div>
