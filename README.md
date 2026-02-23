<p align="right">
  <img src="docs/Logo.png" alt="Talking SWR Meter Logo" width="200">
</p>

# Talking SWR Meter LX1WJ

Accessible talking SWR + RF power meter for blind & visually impaired amateur radio operators.

## What it is
A standalone SWR/power meter based on an ESP32-S3 and an RF directional coupler (FWD/REV DC outputs).  
All key information is provided via **speech** and **Morse**, so no display is required.

## 🤝 Project Philosophy & Status
**This project is not for commercial sale.**
Its purpose is to **empower the amateur radio community** to **build and donate** accessible controllers for blind or visually impaired operators under the **open-source model**:

## Key features (overview)
- Speech output via I2S audio amplifier (e.g. MAX98357A) + speaker
- Morse output via buzzer
- Tuning mode: continuous SWR with audio pitch feedback (tracks minima)
- 6-button user interface using a resistor ladder on one ADC pin
- Built-in web UI (ESP32 Wi-Fi AP) for status, settings, and calibration
- mDNS hostname: `swrmeter.local` (if supported by client)
- Settings + calibration stored persistently (NVS)
- Calibration export/import as JSON

## ⚠️ Disclaimer / Safety Notice
This project is provided **for experimental and educational use only**, **AS IS**, without any warranty.

- The device is **not a certified measuring instrument**.
- Measurements may be inaccurate and can lead to **equipment damage** or **unsafe RF operation** if relied upon.
- You are responsible for correct wiring, safe RF practices, and compliance with local regulations.
- Always verify results with trusted instruments (e.g. wattmeter/dummy load) before relying on them.

**Use at your own risk.**

## Hardware required (minimum)
- ESP32-S3 development board
- RF directional coupler / SWR bridge (50 Ω) with DC outputs: FWD and REV (keep within 0…3.3 V at ADC)
- I2S audio amplifier + speaker
- Piezo buzzer (PWM)
- 6 pushbuttons + resistor ladder (single ADC input)
- 12 V supply (and regulators as needed)

## Quick start
1. Flash firmware to the ESP32-S3 (Arduino IDE).
2. Power on: the device starts a Wi-Fi access point named `SWR-Meter-S3-FW <version>`.
3. Connect with phone/PC and open: `http://192.168.4.1/`
4. Use the buttons to announce SWR/power (speech or Morse).
5. Calibrate your coupler in the web UI (required for meaningful watt readings).

## Documentation
- **Detailed operating guide:** [USAGE.md](USAGE.md)
- (planned) Hardware/KiCad, wiring, enclosure, BOM: `docs/` and `hardware/`

## License
Firmware source code: GPL-3.0  
Documentation: CC BY-SA 4.0  
See `LICENSE` and `LICENSE-docs`.



