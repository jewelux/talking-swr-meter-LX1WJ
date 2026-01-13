<p align="right">
  <img src="docs/Logo.png" alt="Talking SWR Meter Logo" width="200">
</p>

# Talking SWR Meter (ESP32)

This project implements a talking SWR and RF power meter based on an ESP32.
It is designed to be operated without a display and provides all relevant
information via audio output.

The device measures forward and reflected power using a directional coupler
and calculates RF power and SWR. Interaction is performed using buttons,
acoustic feedback, and a web interface.

---

## Project Evolution

This project is intentionally designed as an evolving system.

The Talking SWR Meter did not start as a fixed or finished product, but as a
practical solution to real operating needs. Features and refinements have been
added step by step, driven by hands-on use, operator feedback, and practical
experience in real operating environments.

Firmware, hardware concepts, and the user interface are subject to continuous
improvement. Evolution is considered more important than freezing the design
at an early stage, allowing the project to adapt to new requirements and
different user needs over time.

The project should therefore be understood as an open and growing design
rather than a static device.

---

## Features

- Measurement of forward power, reflected power, and SWR
- Spoken output of power and SWR via loudspeaker (voice samples stored in flash)
- Morse code output via buzzer
- Audio tuning mode with pitch related to SWR
- Six-button user interface using a resistor ladder on a single ADC input
- Integrated web interface for configuration and calibration
- WiFi access point and mDNS support (`swrmeter.local`)
- Calibration stored in non-volatile memory (NVS), exportable as JSON
- Persistent user settings:
  - speaker volume
  - buzzer volume
  - morse speed

---

## Hardware

- ESP32-S3 development board
- Directional coupler with DC outputs for forward and reflected power
- I2S audio amplifier (e.g. MAX98357A) and loudspeaker
- Buzzer for morse code and tuning feedback
- Six push buttons connected as a resistor ladder
- Optional enclosure and RF connectors (SO-239 or N-type)

---

## Operation

The device is operated using six buttons (T1–T6). Depending on short or long
presses, the buttons trigger spoken output, morse output, tuning mode, or
adjust user settings.

All actions provide immediate acoustic feedback. Ongoing speech output can
be cancelled to keep operation responsive.

---

## Web Interface

The ESP32 hosts a built-in web server accessible via:

- `http://swrmeter.local` (mDNS)
- or the assigned IP address

The web interface allows:
- viewing current power and SWR values
- editing configuration parameters
- performing calibration
- exporting and importing calibration data
- resetting user settings or performing a factory reset

---

## Persistent Settings

User settings are stored in non-volatile memory (NVS), including speaker
volume, buzzer volume, and morse speed.

Settings are saved automatically after a short delay when changed.
A short audible confirmation indicates successful storage.

A settings version number is used internally to ensure compatibility between
firmware updates. If a mismatch is detected, default values are restored
automatically.

---

## Calibration

Calibration is performed using a known load and defined transmitter power
levels. Calibration data maps ADC readings to RF power in watts.

Calibration data can be exported and imported as a JSON file via the web
interface for backup and reuse.

---

## Inclusion and Accessibility

This project was created with a strong focus on inclusion.

Many amateur radio instruments rely on visual displays and are therefore
difficult or impossible to use for blind or visually impaired operators.
The Talking SWR Meter addresses this limitation by providing all relevant
information through audio output.

By combining spoken announcements, morse code, and acoustic tuning feedback,
the device enables blind radio amateurs to independently measure and tune
their antennas without assistance.

The project is intended as a contribution toward more inclusive amateur
radio equipment and to encourage further development of accessible tools
within the amateur radio community.
