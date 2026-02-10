# Smart Gaggia: ESP32-S3 PID & Profiling Controller

A complete hardware and firmware overhaul for Gaggia Classic espresso machines. This project replaces the stock thermostat logic with an industrial-grade **ESP32-S3** controller, featuring **Dual-PID thermal management**, **pressure profiling**, and a reactive **touchscreen UI**.

> **⚠️ HIGH VOLTAGE WARNING**
> This project involves modifying mains electricity (110V/240V). Improper installation can result in electrocution, fire, or machine damage. **Proceed at your own risk.**

---

## ☕ Key Capabilities

### 1. Precision Thermal Engine
* **Dual-Loop PID:** Dedicated control loops for Brew Mode (93°C target) and Steam Mode (145°C target).
* **Zero-Cross Switching:** SSR control synchronized with AC zero-crossing to minimize electrical noise and stress on components.
* **Predictive Heating:** Algorithms anticipate thermal drops during flushing to recover temperature faster than stock hardware.

### 2. Pressure & Flow Profiling
* **Automated Shot Stages:** The `EspressoSM` state machine orchestrates a professional extraction cycle:
    1.  **Pre-infusion:** Low-pressure soaking (e.g., 6s) to saturate the puck.
    2.  **Bloom:** Pump pause (e.g., 1s) to allow pressure equalization.
    3.  **Extraction:** Ramp to full 9-bar pressure for the main pour.
* **Scalable Metrics:** Supports integration with an **HX711 Load Cell** for gravimetric (weight-based) shot stopping.

### 3. "DreamSteam" Technology
* **Smart Refill:** Unlike standard PID mods, this system manages the pump during steaming to maintain boiler water levels, providing sustained steam pressure without "gassing out" extracted from `dreamsteam_controller.h`.

### 4. Safety First Architecture
* **Hardware Watchdogs:** System halts all outputs if the MCU freezes (500ms timeout).
* **Thermal Runaway Protection:** Hard cutoff at **165°C** to prevent boiler overheating.
* **Sanity Checks:** Logic to detect disconnected sensors (NaN readings) or zero-cross failures.

---

## 🛠️ Hardware Stack

| Component | Specification | Role |
| :--- | :--- | :--- |
| **MCU / Display** | **Guition JC3248W535EN** | ESP32-S3 + 3.5" Touchscreen (Running LVGL 8.3) |
| **Thermocouple** | K-Type + MAX31855 | High-speed temp reading (±0.25°C precision) |
| **Relay** | 40A SSR | Controls the boiler heating element |
| **Dimmer** | RobotDyn AC Dimmer | Chops AC wave for pump pressure control |
| **Scale** | HX711 + Load Cell | (Optional) Measures shot output in real-time |

---

## 🔌 Pin Configuration (ESP32-S3)

*Defined in `src/project_config.h`*

* **Heater (SSR):** GPIO 16
* **Pump (Dimmer):** GPIO 43 (PWM) / GPIO 44 (Zero-Cross)
* **Solenoid Valve:** GPIO 5
* **Thermocouple (SPI):** CS: 6, DO: 7, CLK: 15

---

## 🚀 Getting Started

### 1. Prerequisites
* **PlatformIO:** Required for building the firmware.
* **Libraries:** LVGL, Adafruit MAX31855, HX711 (auto-installed via `platformio.ini`).

### 2. Installation
1.  **Flash the Firmware:**
    ```bash
    pio run -e esp32-s3-devkitc-1 -t upload
    ```
2.  **Wiring:** Follow the diagram in `Diagram_index.html` carefully. Ensure high-voltage lines are isolated.
3.  **Calibration:**
    * Set `SCALE_CALIBRATION` in `app_config.h` using a known weight.
    * Verify `GRID_FREQ_HZ` (50Hz vs 60Hz) matches your region for accurate pump dimming.

### 3. Usage
* **Idle:** Screen displays current boiler temp. Tap to enter Menu.
* **Brew:** Press the "Coffee" icon. The machine runs the configured Profile (Pre-infuse -> Bloom -> Brew).
* **Steam:** Toggle Steam Mode. Wait for temp to reach 145°C. The system will pulse the pump gently to keep steam wet and powerful.

---

## 🔧 Developer Notes

* **State Machine:** The extracted logic in `src/logic/espresso_sm.h` is decoupled from hardware, allowing extraction profiles to be unit-tested on a PC (`pio test -e native`).
* **UI Customization:** Built with **LVGL** (Light and Versatile Graphics Library). UI assets are located in `src/ui/`.

---

## ⚠️ Disclaimer

This firmware is experimental. The authors are not responsible for damage to your machine or personal injury. **Always unplug your machine before working on internals.**
