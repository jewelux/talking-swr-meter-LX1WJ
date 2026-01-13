<p align="right">
  <img src="docs/Logo.png" alt="Talking SWR Meter Logo" width="200">
</p>


# Accessible Talking SWR Meter
### Speech-guided SWR and RF power meter for blind and visually impaired amateur radio operators

This project implements a fully “talking” SWR and RF power meter based on an **ESP32-S3** and a **directional coupler**.  
It is designed to be used without any visual display, making it especially suitable for blind and visually impaired amateur radio operators.

The device announces RF power and SWR via:
- **Speech output** (I2S audio amplifier, voice clips stored in flash – no SD card required)
- **Morse code** (buzzer)
- **Audio tuning mode** (buzzer pitch corresponds to SWR)
- **Built-in web interface** (Wi-Fi AP + mDNS hostname)

---

## Firmware / Release
This repository contains the **V51** firmware release.  
Inside the sketch, the firmware version string is shown as **FW_VERSION = 5.4.1**.

Key improvements in this release:
- mDNS hostname **swrmeter.local**
- Persistent user settings (speaker volume, buzzer volume, Morse speed)
- Failsafe settings versioning (auto-reset on schema mismatch)
- Web UI shows current settings + “Reset Settings”
- Audible confirmation after settings auto-save (short double-beep)

---

## Quick Start (first power-up)

### 1) Flash the firmware
Open the `.ino` in Arduino IDE and upload to your ESP32-S3.

Recommended board/core:
- ESP32 Arduino core (Espressif)
- Select your ESP32-S3 board variant

### 2) Connect to the web interface
You have two options:

**A) mDNS (recommended):**
- Open: `http://swrmeter.local`

**B) Access-Point fallback:**
- Connect your phone/PC to the ESP32 AP SSID shown as: `SWR-Meter-S3-FW <version>`
- Then open: `http://192.168.4.1`

---

## Controls (T1..T6)
The device uses **6 pushbuttons** connected as a **resistor ladder** to a single ADC input.

Typical functions (short/long press may differ depending on your build):
- Speak power / SWR
- Morse output
- Enter/exit tuning mode
- Adjust volumes
- Adjust Morse speed

> Tip: The device supports “Cancel / Stop output” behavior to keep interaction responsive.

---

## Web Interface Pages

- `/` : Status page (latest power & SWR) and **Current Settings**
- `/config` : Configuration page
- `/cal` : Calibration page
- `/factoryReset` : Factory reset (configuration + calibration)
- `/resetSettings` : **Reset user settings only** (speaker/buzzer/morse)

Data endpoints (used internally):
- `/data`
- `/saveConfig`
- `/saveCal`
- `/cal/export`
- `/cal/import`

---

## Persistent Settings (important)
The following **user settings are stored in NVS (flash)**:
- Speaker volume (5 steps)
- Buzzer volume (5 steps)
- Morse speed (BPM)

Behavior:
- When you change a setting, it is marked “dirty”.
- After ~1–2 seconds without further changes, settings are automatically saved.
- A **short double-beep** confirms the save.

Failsafe:
- Settings are stored together with a **settings version number**.
- If the firmware expects a different version, the device automatically loads defaults and re-saves them.

---

## Calibration (directional coupler)
A directional coupler is required. It must provide two DC voltages proportional to RF power:
- **FWD** (forward power)
- **REV** (reflected power)

Calibration is done via the web UI:
- Enter calibration points (Power ↔ Voltage)
- Export/Import calibration as JSON for backup
- Calibration is stored in NVS

---

## Troubleshooting

### `swrmeter.local` does not work
- Use the AP fallback: `http://192.168.4.1`
- Some systems require mDNS/Bonjour support
- VPN / guest Wi-Fi isolation can break mDNS

### Settings do not persist
- Verify you hear the **double-beep** after changing settings (auto-save confirmation)
- Use `/resetSettings` if needed, then set your preferred values again

---

## Licenses
Licenses remain unchanged:
- `LICENSE` applies to firmware/source code
- `LICENSE-docs` applies to documentation

---

## Repository Structure (recommended)
- `src/` contains firmware releases:
  - `ESP32S3_SWRMeter_V51.ino`
  - `voice_data.h`
- Root contains docs:
  - `README.md`
  - `bom.txt`
  - `talking-swr-meter-principle.txt`
or persons. RF systems can generate heat, high
voltages, and strong electromagnetic fields. Always follow your local
regulations and operate responsibly.
