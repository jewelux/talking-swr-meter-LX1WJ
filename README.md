<p align="right">
  <img src="docs/Logo.png" alt="Talking SWR Meter Logo" width="200">
</p>


# Accessible Talking SWR Meter
### A speech-guided SWR and RF power meter for blind and visually impaired amateur radio operators

This project implements a fully “talking” SWR and RF power meter based on an ESP32-S3 and a directional coupler.  
It is designed to be used **without any visual display**, making it especially suitable for **blind and visually impaired radio amateurs**.

The device announces RF power and SWR via:

- **speech output** (I2S audio module, no SD card required)
- **Morse code** (buzzer)
- **audio-based tuning mode** (pitch corresponds to SWR)
- **a built-in web interface** (ESP32 access point)

Note: This project is actively evolving. The design, features, and documentation will be refined over time as development progresses. The goal is to create a reliable, accessible tool for amateur radio operators, with a focus on usability and inclusion.

The goal of this project is to promote **inclusion in amateur radio** by demonstrating that common measurement tools can be made fully accessible.

---

# ✨ Features

## 🔊 Speech output
- Uses an I2S audio amplifier (e.g., MAX98357)
- 16-bit PCM samples stored in PROGMEM  
- Announces:
  - RF output power in watts  
  - SWR with one decimal place  
- No SD card required

## 🔔 Morse code buzzer
- Encodes:
  - RF power (f.ex. 0–99 watts)
  - SWR × 10 (e.g., 1.3 → “13”)  
- Adjustable speed  
- Confirmation beeps for settings  
- Useful in noisy environments

## 🎧 Tuning mode
- Continuous SWR measurement every ~100 ms  
- Buzzer pitch varies with SWR  
  - higher pitch → worse SWR  
  - lower pitch → better SWR  
- Short beep when a new SWR minimum is detected  
- Excellent for antenna tuning using only hearing

## 🌐 Built-in Web Interface (ESP32 Access Point)
The ESP32-S3 runs as a Wi-Fi access point.  
Open a browser and navigate to:
http://192.168.4.1


Provided pages:

- **Status page:** shows latest measured power & SWR  
- **Configuration:** timing, volumes, morse speed, thresholds …  
- **Calibration:** enter up to 10 calibration points for the coupler  
- **Factory reset:** restore defaults  

All data (configuration & calibration) is stored in **NVS (flash)**.

## 🔘 Six-button resistor ladder (single ADC input)
- 6 pushbuttons connected via a resistor ladder to **one ADC pin**  
- Short- and long-press detection in software  
- Saves GPIO pins  
- Compact and efficient design

---

# 🧱 Hardware Overview

## Required components
- ESP32-S3 development board  
- Directional coupler (50 Ω system impedance)  
- Two DC outputs from the coupler:
  - **FWD** (forward power)
  - **REV** (reflected power)  
- MAX98357 or similar I2S audio amplifier  
- Small loudspeaker  
- Piezo buzzer (LEDC/PWM driven)  
- 6 pushbuttons + resistor ladder  
- 5 V / 3.3 V power supply  

## High-level block diagram

```
      +-----------------------+
      |       ESP32-S3        |
      |                       |
 ADC1 | <--- FWD -------------+--- Directional Coupler
 ADC2 | <--- REV -------------+   (Forward / Reflected Power)
      |
      | GPIO9 --> Button Ladder --> T1..T6 Buttons
      |
      | I2S_BCLK/LRCLK/DOUT --> MAX98357 --> Speaker
      |
      | LEDC PWM --> Buzzer
      |
      +-----------------------+
```


# 📡 Directional Coupler Requirements

A directional coupler with 50 Ω system impedance is required.
It must provide two DC voltages proportional to RF power:

- **FWD** — forward power voltage (0–3.3 V recommended)
- **REV** — reflected power voltage (0–3.3 V recommended)

These voltages are read by two ADC inputs on the ESP32-S3.

### Voltage level considerations

Many directional couplers or SWR bridges produce output voltages that do **not** fall directly into the safe ADC input range of the ESP32-S3 (0–3.3 V).  
Depending on the coupler design, the voltage may be:

- too **high** (may exceed 3.3 V at higher RF power)
- too **low** (millivolt-level signals at low RF power)
- non-linear or noisy

To ensure correct ADC operation and protect the ESP32-S3, an intermediate conditioning stage may be required:

#### If the coupler output voltage is **too high**:
Use a **resistor divider** (voltage divider) to reduce the signal into the safe range.  
Ensure the divider output never exceeds 3.3 V even at the maximum RF power you plan to measure.

#### If the coupler output voltage is **too low**:
Use a **simple op-amp amplifier** or buffer stage to increase the voltage.  

#### If your coupler already outputs 0–3.3 V:
You may connect it **directly** to the ESP32-S3 ADC pins, assuming:

- a common ground exists between the RF section and ESP32
- the output impedance is below ~10 kΩ (for stable ADC readings)
- no RF leakage reaches the microcontroller

### Calibration

Regardless of signal conditioning, the firmware relies on **10 calibration points** (Power ↔ Voltage).  
After hardware wiring, you must perform calibration via the embedded web interface to achieve accurate measurements over the full RF range.






### Calibration mapping

The web interface allows you to enter **up to 10 calibration points**, each consisting of:

- measured RF power (in watts)
- corresponding FWD voltage (in volts)

The ESP32 interpolates between these points to convert voltage → power.

### Accuracy considerations

Measurement accuracy depends mainly on:

- the directivity and quality of the directional coupler  
- stability and linearity of its DC output voltages  
- correct 50 Ω RF wiring and shielding  
- well-distributed calibration points across the used power range  

---

# 🧠 Design Decisions

## 1. Non-blocking architecture
Earlier versions suffered from blocked Wi-Fi access when audio or long delays occurred.  
The current design:

- avoids long blocking delays  
- regularly services the web server  
- keeps the AP stable even during speech output and tuning  

## 2. I2S audio instead of analog output
Using an I2S amplifier (MAX98357):

- avoids analog filtering  
- gives clean 16-bit audio output  
- requires only BCLK, LRCLK, and DOUT  
- allows digital volume control  

## 3. Buzzer as independent audio channel
Reasons for including a buzzer:

- Morse is extremely efficient  
- ideal in noisy environments  
- tuning via pitch is instantaneous  
- provides confirmation tones  

## 4. Button ladder
Why a resistor ladder?

- saves GPIO pins  
- allows 6 buttons on one ADC input  
- simple hardware  
- extremely reliable after calibration  

---

# 📦 Software Overview

- Arduino IDE sketch  
- Uses:
  - WiFi (Access Point mode)
  - WebServer  
  - Preferences (NVS)
  - I2S driver (`driver/i2s.h`)  
- Version **V4.5**:
  - button ladder on GPIO 9  
  - speech data in PROGMEM  
  - tri-lingual serial debug output (EN / DE / FR)  
  - non-blocking architecture  
  - directional coupler calibration through the web interface  

---

# 🎛 Button Functions (T1..T6)

| Button | Short Press                                 | Long Press                        |
|--------|----------------------------------------------|-----------------------------------|
| **T1** | Measurement in Morse                         | Tuning mode ON/OFF                |
| **T2** | Measurement with speech output               | Voice self-test (all clips)       |
| **T3** | Buzzer volume ↓                               | Buzzer volume ↑                    |
| **T4** | Speech volume ↓                               | Speech volume ↑                    |
| **T5** | Morse speed slower                           | Reset to default speed            |
| **T6** | Morse speed faster                           | Reset to default speed            |

---

# 🧑‍🦯 Operating Instructions (for blind users)

### Power-up
- Turn on the device  
- You will hear **“OK” in Morse**  
- Device is ready  

### Basic measurement
1. Connect transmitter → coupler → antenna  
2. Key the transmitter  
3. Press:  
   - **T1** for Morse measurement  
   - **T2** for spoken measurement  

The device announces:
- RF output power  
- SWR (1 decimal place)

### Antenna tuning
1. Press **T1 long** → tuning mode  
2. Maintain a continuous carrier, low power
3. Adjust antenna tuner  
4. Listen for:
   - lower pitch → better SWR  
   - short beep → new minimum found  
5. Press **T1 long** to exit tuning mode  

---

# 🌐 Using the Web Interface

1. Connect to the Wi-Fi network:  
   **SWR-Meter-S3-V4.5**
2. Open your browser:  
   `http://192.168.4.1`
3. Use:
   - **Status page**  
   - **Configuration page**  
   - **Calibration page**  
   - **Factory reset**  

---

# 🎚 Calibration

1. Use the web interface
2. Connect Esp32-S3 to Arduino-Ide via usb
3. Read output on serial monitor
4. Apply a known RF power to the coupler  
5. Read the corresponding FWD voltage in serial monitor
6. Open the calibration page  
7. Enter pairs of:
   - Power (watts)  
   - Voltage (volts)  
8. Fill all 10 points across your typical power range  
9. Save  

The device stores the data in NVS and uses it for future measurements.

---

# 🧩 Possible Extensions

- additional languages  
- MQTT logging  
- Bluetooth support  
- external display for sighted users  
- vibration feedback  

---

# ❤️ Inclusion in Amateur Radio

This project demonstrates that RF measurement tools **can be made accessible**.  
Blind and visually impaired operators should be able to tune antennas and measure SWR **independently**.

Pull requests and community contributions are highly welcome.


## Disclaimer / Haftungsausschluss

This project is provided "as is", without any warranty of any kind.
Building, modifying, or using the hardware and software is entirely at
your own risk. The authors assume no liability for any damage to radio
equipment, property, or persons. RF systems can generate heat, high
voltages, and strong electromagnetic fields. Always follow your local
regulations and operate responsibly.
