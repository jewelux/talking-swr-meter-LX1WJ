# Talking SWR Meter – Operating Guide (Non-Visual Use)

Applies to version V5.4.1 (FW_VERSION 5.4.1) and later.

This document describes how to operate the Talking SWR Meter **without any visual
feedback**. All functions can be accessed using buttons and acoustic output
(speech, morse code, and tones).

The guide is written primarily for blind and visually impaired amateur radio
operators.

---

## 1. Basic Operating Concept

The Talking SWR Meter is controlled using **six push buttons**, labeled T1 to T6.
The buttons are connected as a resistor ladder and are detected by the firmware
as individual keys.

Each button supports:
- **short press**
- **long press**

The device always provides **acoustic feedback** for user actions.

Important rule:
> **Any button press immediately cancels ongoing speech or morse output.**

This makes the device responsive and prevents long waiting times.

---

## 2. Acoustic Feedback Types

The device uses several types of audio output:

### 2.1 Speech Output (Loudspeaker)
Spoken announcements are used for:
- RF power in watts
- SWR value
- status confirmations

Speech is played through the loudspeaker connected to the I2S audio amplifier.

### 2.2 Morse Output (Buzzer)
Morse code is output via the buzzer and is used for:
- alternative numeric output
- confirmation signals
- tuning support

Morse speed is adjustable.

### 2.3 Beeps and Tones
Beeps are used for:
- button feedback
- volume level indication
- confirmation of stored settings

Example:
- **Number of beeps = selected level** (1 to 5)

A **short double-beep** confirms that user settings have been saved.

---

## 3. Powering On the Device

When the device is powered on:

1. The system initializes.
2. After a short delay, the device is ready for operation.
3. No visual confirmation is required; the device can be used immediately.

User settings (volume, morse speed) are restored automatically from memory.

---

## 4. Speaking RF Power and SWR

### 4.1 Speaking RF Power 
- Press the assigned button for power output (typically **T2, short press**).
- The device speaks the measured forward power in watts.

Example:
> “Power: twenty watts”

If no RF power is present, the device announces zero.

### 4.2 Speaking SWR
- Press the assigned button for SWR output (typically **T2, short press**).
- The device speaks the SWR value with one decimal digit.

Example:
> “SWR: one point five”

If forward power is too low, a safe default is announced.

---

## 5. Tuning Mode (Audio SWR Feedback)

Tuning mode provides continuous acoustic feedback to help adjust an antenna.

### 5.1 Entering Tuning Mode
- Press and hold the assigned tuning button (typically **T2, long press**).

### 5.2 Interpreting the Sound
- The buzzer emits a continuous tone.
- **Lower pitch indicates better SWR.**
- **Higher pitch indicates worse SWR.**

While tuning:
- Adjust the antenna until the pitch reaches its minimum.

### 5.3 Leaving Tuning Mode
- Press the tuning button again (short or long press).
- Or press any other button.

---

## 6. Morse Output Mode

Morse output can be used as an alternative to speech, especially in noisy
environments.

### 6.1 Activating Morse Output
- Press the assigned button (typically **T3**).

Numbers and values are output as morse code via the buzzer.

### 6.2 Adjusting Morse Speed
- Use the assigned buttons (typically **T5 / T6**).
- Each adjustment is confirmed acoustically.

Morse speed is stored persistently.

---

## 7. Adjusting Volumes

### 7.1 Speaker Volume (Speech)
- Use the assigned volume button (typically **T4**).
- Each press cycles through 5 volume levels.
- The number of beeps indicates the selected level.

### 7.2 Buzzer Volume (Morse and Beeps)
- Use the assigned buzzer volume control (typically **long press on T4**).
- Volume levels are indicated by beep count.

### 7.3 Saving Volumes
- After changing a volume, wait briefly.
- A **short double-beep** confirms that the setting has been stored.

Volumes are restored automatically after power-up.

---

## 8. Canceling Output

At any time:
- **Press any button** to immediately stop ongoing speech or morse output.

This applies to:
- spoken announcements
- morse sequences
- tuning mode

This rule is central to fast and stress-free operation.

---

## 9. Persistent User Settings

The following settings are stored in non-volatile memory:
- speaker volume
- buzzer volume
- morse speed

Settings are saved automatically after a short idle time.
A double-beep confirms successful storage.

The device uses an internal settings version number to ensure compatibility
between firmware updates. If an incompatible version is detected, default
settings are restored automatically.

---

## 10. Web Interface (Optional)

Although the device can be fully operated without sight, a web interface is
available for configuration and calibration.

Access methods:
- `http://swrmeter.local`
- or the device IP address

The web interface allows:
- calibration
- configuration
- resetting user settings
- factory reset

The web interface is optional and not required for normal operation.

---

## 11. Practical Operating Tips

- Always cancel long speech output if you already know the value.
- Use tuning mode for antenna adjustment instead of repeated spoken SWR.
- Trust the double-beep: it means your settings are safely stored.
- Perform calibration with a reliable dummy load and known transmitter power.

---

## 12. Summary

The Talking SWR Meter is designed for **independent, non-visual operation**.
All essential functions are accessible through buttons and acoustic feedback.

No display is required at any stage of normal operation.

End of document.
