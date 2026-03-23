# USAGE - Talking SWR Meter (ESP32-S3, Firmware V5.5)

## Important safety note
This device is intended for **experimental and assistive use only**.
It is **not a certified measuring instrument**.

- Verify all wiring with power off.
- Keep FWD/REV voltages within the ESP32 ADC range (0...3.3 V).
- Always confirm measurements with a trusted external wattmeter.

---

## 1. Power-up and access
After power-up the device starts:

- Wi-Fi access point: `SWR-Meter-S3-FW 5.5`
- Web server: `http://192.168.4.1/`
- mDNS hostname: `http://swrmeter.local/` (if supported)

The web UI is required for configuration and calibration.

---

## 2. Button overview
Six pushbuttons (T1...T6) are read via a resistor ladder on one ADC input.  
Buttons support **short** and **long** presses.

---

## 3. Button functions

### T1 - Morse measurement / tuning
- **Short press:**  
  Single measurement with Morse output (forward power and SWR x10).
- **Long press:**  
  Toggle tuning mode.
  - Continuous SWR measurement
  - Audio pitch follows SWR
  - New SWR minima are indicated acoustically
- Any button press stops tuning immediately.

### T2 - Speech measurement / audio test
- **Short press:**  
  Single measurement with speech output (power and SWR).
- **Long press:**  
  Voice self-test (plays all stored voice samples).

### T3 - Buzzer volume
- **Short press:** Decrease buzzer volume (stepwise).
- **Long press:** Increase buzzer volume (stepwise).

### T4 - Speech volume
- **Short press:** Decrease speech volume (stepwise).
- **Long press:** Increase speech volume (stepwise).

### T5 - Morse speed (slower)
- **Short press:** Decrease Morse speed.
- **Long press:** Restore default Morse speed.

### T6 - Morse speed (faster)
- **Short press:** Increase Morse speed.
- **Long press:** Restore default Morse speed.

Volume and speed settings are stored persistently.

---

## 4. Measurement behavior
- Forward (FWD) and reflected (REV) voltages are sampled via ADC.
- Firmware computes:
  - Forward power (W)
  - Reflected power (W)
  - Standing Wave Ratio (SWR)
- Without calibration, watt readings are **not meaningful**.

---

## 5. Web interface

### Main pages
- `/` - Status and live readings
- `/config` - Configuration
- `/cal` - Calibration

### Data and maintenance
- `/data` - Live data (JSON)
- `/cal/capture` - Capture Ufwd/Uref for one calibration row
- `/cal/export` - Export calibration as JSON
- `/cal/export.csv` - Export calibration as CSV
- `/cal/import` - Import calibration from JSON or CSV
- `/saveConfig` - Store configuration
- `/saveCal` - Store calibration
- `/resetSettings` - Reset user settings
- `/factoryReset` - Reset all settings and calibration

---

## 6. Calibration procedure (mandatory)
Calibration maps ADC voltages to real RF power.

V5.5 uses an extended calibration table with **66 rows**.
The default table is prepared as a measurement plan with preset power and SWR targets.

Two practical calibration methods are supported:

1. **Normal measurement**
   - Use a good reference SWR/power meter.
   - Enter the real forward power into `Pfwd [W]`.
   - Enter the measured SWR into `SWR read`.
   - Use **Measure** on the corresponding row to capture `Ufwd [V]` and `Uref [V]`.

2. **Dummy load + voltage measurement**
   - ......

Additional notes:

- **Clear** clears `Pfwd`, `Ufwd` and `Uref` in one row.
- **Clear all Pfwd/Ufwd/Uref** clears those three fields in all rows after confirmation.
- `SWR read` stays untouched by the clear functions.
- Save calibration after editing.
- Export JSON or CSV as backup.

Repeat calibration if the coupler or analog front-end changes.

---

## 7. Reset options
- **Reset settings:**  
  Restores default user settings, keeps calibration.
- **Factory reset:**  
  Restores all defaults and deletes calibration.

Both options are available in the web UI.

---

## 8. Troubleshooting
- **No speech output:**  
  Check I2S wiring and speaker, run T2 long (self-test).
- **Wrong power/SWR values:**  
  Calibration missing, calibration points implausible, or FWD/REV lines swapped.
- **Measure button fills wrong voltages:**  
  Check ADC wiring, common ground, and make sure the coupler outputs stay within 0...3.3 V.
- **Unstable readings:**  
  Improve RF grounding, shorten analog leads, add filtering.
- **Web UI unreachable:**  
  Reconnect to AP and use `http://192.168.4.1/`.
