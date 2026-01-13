#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <time.h>
#include <WebServer.h>
#include <Preferences.h>

extern "C" {
#include "driver/i2s.h"
}

// PROGMEM voice bank
#include "voice_data.h"

// --------------------------------------------------------
// Firmware / settings versioning
// --------------------------------------------------------
static const char* FW_VERSION = "5.4.1";
static const uint16_t USER_SETTINGS_VERSION = 1;  // bump when changing stored keys/meaning

// Forward declaration to keep Arduino's auto-prototypes happy
enum VoiceClip : uint8_t;


// ========================================================
// ESP32-S3 TALKING SWR METER V5.4.1
//
// - RF measurement (forward/reflected) with calibration table
// - 6-button ladder on one ADC input (T1..T6)
// - Speech output from PROGMEM (zero..nine, point, power, watts, swr)
// - Buzzer for Morse, tuning tone and startup "OK" in Morse
// - WiFi AP + main page + configuration + calibration page
//
// Button functions:
//   T1 short : Single measurement + Morse (power & SWR*10)
//   T1 long  : Tuning mode ON/OFF (SWR → buzzer pitch)
//   T2 short : Single measurement + speech from PROGMEM
//   T2 long  : Speech self test (all clips in sequence)
//   T3 short : Buzzer volume one step down
//   T3 long  : Buzzer volume one step up
//   T4 short : Speech volume one step down
//   T4 long  : Speech volume one step up
//   T5 short : Morse speed slower (-STEP_MORSE_BPM), speed in Morse
//   T5 long  : Restore default Morse speed
//   T6 short : Morse speed faster (+STEP_MORSE_BPM)
//   T6 long  : Same as T5 long (restore default)
// ========================================================

// --------------------------------------------------------
// Pin assignment
// --------------------------------------------------------

// RF measurement
const int PIN_FWD = 1;  // forward power (ADC)
const int PIN_REV = 2;  // reflected power (ADC)

// Button ladder
const int PIN_PTT = 9;  // ADC input for ladder T1..T6

// I2S / MAX98357
const int I2S_BCLK_PIN = 4;
const int I2S_LRCLK_PIN = 5;
const int I2S_DOUT_PIN = 6;
const int AMP_SD_PIN = 7;  // SD/EN of MAX98357

// Buzzer (Morse, tuning, short beeps) via LEDC
const int PIN_BUZZER = 8;
const int BUZZER_PWM_BITS = 10;  // 10-bit -> 0..1023
const int BUZZER_PWM_MAX = (1 << BUZZER_PWM_BITS) - 1;
const int BUZZER_TONE_FREQ = 800;  // base beep / Morse freq

// I2S / Audio
const int I2S_SAMPLE_RATE = 16000;  // Hz, must match voice samples

// ADC parameters
const float VREF = 3.30f;
const int ADC_MAX = 4095;
const int NUM_SAMPLES_HF = 64;

// RF fallback calibration (used when no valid table exists in NVS)
// NOTE: must be writable because we load/store these values from/to NVS.
float A_FWD = 0.316f;
float B_FWD = -0.175f;
float A_REV = 0.316f;
float B_REV = -0.175f;

// Optional: legacy "3V2" reference used by some older calibration logic/pages.
// Keep it writable for NVS compatibility even if the rest of the sketch uses VREF.
float VREF_3V2 = 3.20f;

// Minimum power for meaningful SWR
float P_MIN_ACTIVE = 1.0f;  // W -> below 1W no meaningful SWR

// Global last measurement values (for speech + web)
float g_powerW = 0.0f;
float g_swr = 1.0f;

// --------------------------------------------------------
// Button ladder configuration
// --------------------------------------------------------

// Mid voltages (measured with stable 3.3 V rail)
float BTN_MID_NO = 3.300f;
float BTN_MID_T1 = 0.265f;
float BTN_MID_T2 = 0.540f;
float BTN_MID_T3 = 0.973f;
float BTN_MID_T4 = 1.560f;
float BTN_MID_T5 = 2.184f;
float BTN_MID_T6 = 2.658f;

// Detection ranges (± Volts)
float BTN_RANGE_NO = 0.257f;
float BTN_RANGE_T1 = 0.110f;
float BTN_RANGE_T2 = 0.110f;
float BTN_RANGE_T3 = 0.173f;
float BTN_RANGE_T4 = 0.235f;
float BTN_RANGE_T5 = 0.189f;
float BTN_RANGE_T6 = 0.189f;

// Debounce and short/long timing
unsigned long BUTTON_LONG_MS = 700;  // >= 700 ms = long press
unsigned long DEBOUNCE_MS = 40;

// Button state variables
uint8_t rawButton = 0;  // 0 = none, 1..6 = T1..T6
uint8_t currentButton = 0;
uint8_t lastButton = 0;
uint8_t pressedButton = 0;
unsigned long lastDebounceTime = 0;
unsigned long pressStartTime = 0;

// --------------------------------------------------------
// Morse parameters
// --------------------------------------------------------

int DEFAULT_MORSE_BPM = 15;
int MIN_MORSE_BPM = 5;
int MAX_MORSE_BPM = 40;
int STEP_MORSE_BPM = 5;
int MORSE_TONE_FREQ = 800;  // Hz for Morse

int morseBpm = DEFAULT_MORSE_BPM;
int morseUnitMs = 1200 / 15;  // base dot length in ms

// --------------------------------------------------------
// Tuning parameters
// --------------------------------------------------------

unsigned long TUNING_SAMPLE_MS = 100;  // new sample every 100 ms

float TUNING_SWR_MIN = 1.0f;
float TUNING_SWR_MAX = 5.0f;
int TUNING_FREQ_MIN = 100;
int TUNING_FREQ_MAX = 3000;
float TUNING_MIN_DELTA = 0.02f;

// --------------------------------------------------------
// Speech parameters
// --------------------------------------------------------

// Pause between "power ... watts" and "swr ..." in ms
unsigned long SPEECH_PAUSE_MS = 200;

// Speech volume levels (5 steps via index)
const float SPEECH_LEVELS[5] = { 0.10f, 0.25f, 0.50f, 0.75f, 1.00f };
static const int DEFAULT_SPEECH_LEVEL_INDEX = 2;  // 0..4
int speechLevelIndex = DEFAULT_SPEECH_LEVEL_INDEX;     // 0..4 -> default = 0.5x
float SPEECH_VOLUME = 0.50f;  // updated from index

// Output cancel (any button press stops ongoing speech/Morse/beeps)
volatile bool cancelOutput = false;

// Persisted user settings (deferred save to reduce flash wear)
bool settingsDirty = false;
unsigned long lastSettingsChangeMs = 0;

void markSettingsDirty() {
  settingsDirty = true;
  lastSettingsChangeMs = millis();
}

// Forward decl (used by resetToDefaultUserSettings)
void updateMorseTiming();




// --------------------------------------------------------
// Buzzer volume (5 steps)
// --------------------------------------------------------

const int BUZZER_LEVELS[5] = { 2, 10, 30, 100, 700 };
static const int DEFAULT_BUZZER_LEVEL_INDEX = 3;  // 0..4
int buzzerLevelIndex = DEFAULT_BUZZER_LEVEL_INDEX;            // 0..4
int BUZZER_DUTY = BUZZER_LEVELS[3];  // default

void resetToDefaultUserSettings() {
  speechLevelIndex = DEFAULT_SPEECH_LEVEL_INDEX;
  SPEECH_VOLUME = SPEECH_LEVELS[speechLevelIndex];

  buzzerLevelIndex = DEFAULT_BUZZER_LEVEL_INDEX;
  BUZZER_DUTY = BUZZER_LEVELS[buzzerLevelIndex];

  morseBpm = DEFAULT_MORSE_BPM;
  updateMorseTiming();
}


// --------------------------------------------------------
// Tuning state
// --------------------------------------------------------

bool tuningModeActive = false;
float tuningBestSWR = 999.0f;
unsigned long lastTuningSample = 0;
int tuningLastFreq = 0;
bool tuningHasTone = false;

// --------------------------------------------------------
// WiFi / Webserver
// --------------------------------------------------------

String AP_SSID = String("SWR-Meter-S3-FW ") + FW_VERSION;
const char *HOSTNAME = "swrmeter";  // for mDNS / hostname

WebServer server(80);



// Calibration JSON import buffer (used by /cal/import multipart upload)
String g_calImportBuf;
// --------------------------------------------------------
// Calibration data (FWD/REV) - extended (non-symmetric coupler)
// --------------------------------------------------------

Preferences prefs;      // calibration
Preferences prefsUser;  // user settings (volume, morse, etc.)

// We store up to 30 measurement rows entered on the AP calibration page.
// From these rows we build two independent lookup tables:
//   Ufwd -> Pfwd   and   Uref -> Pref (derived from Pfwd + SWR read)
// using piecewise-linear interpolation.
const int CAL_ROWS = 26;

// One row = one real measurement point (can be partially empty)
struct CalRow {
  float Pfwd_W;    // output / true forward power (W) from reference wattmeter
  float Ufwd_V;    // ESP ADC input voltage for forward channel (V)
  float Uref_V;    // ESP ADC input voltage for reflected channel (V)
  float Swr_read;  // SWR from reference instrument (used to derive reflected power)
};

CalRow calRows[CAL_ROWS];

// --- Default calibration row presets (helper values only) ---
// These values are meant as *targets* while measuring. You overwrite them with the real Pfwd reading.
// Only Pfwd_W is prefilled; Ufwd/Uref/SWR remain 0 until you enter real measured values.
static const int CAL_PRESET_COUNT = 26;
static const float defaultPfwdPreset[CAL_PRESET_COUNT] = {
  10, 20, 30, 40, 50, 60, 70, 80, 90, 100,  // SWR ~ 1.0 (dummy load), cover full forward range
  10, 20, 30,                               // SWR ~ 2.0
  10, 20, 30,                               // SWR ~ 2.5
  10, 20, 30,                               // SWR ~ 3.0
  10, 20, 30,                               // SWR ~ 4.0
  10, 20, 30,                               // SWR ~ 5.0
  50                                        // optional validation point (SWR ~ 3..4)
};

static bool isCalTableEmpty() {
  for (int i = 0; i < CAL_ROWS; i++) {
    if (calRows[i].Pfwd_W != 0.0f || calRows[i].Ufwd_V != 0.0f || calRows[i].Uref_V != 0.0f || calRows[i].Swr_read != 0.0f) {
      return false;
    }
  }
  return true;
}

static void applyDefaultPfwdPresetsIfEmpty() {
  if (!isCalTableEmpty()) return;
  for (int i = 0; i < CAL_ROWS; i++) {
    calRows[i].Pfwd_W = (i < CAL_PRESET_COUNT) ? defaultPfwdPreset[i] : 0.0f;
    calRows[i].Ufwd_V = 0.0f;
    calRows[i].Uref_V = 0.0f;
    calRows[i].Swr_read = 0.0f;
  }
}


// Derived interpolation tables (sorted by voltage)
float calFwdU[CAL_ROWS];
float calFwdP[CAL_ROWS];
int calFwdN = 0;

float calRevU[CAL_ROWS];
float calRevP[CAL_ROWS];
int calRevN = 0;

// Validity flags per direction
bool calibrationFwdValid = false;
bool calibrationRevValid = false;

// Info only (shown on page)
float P_MAX_3V2 = 100.0f;

// --------------------------------------------------------
// Helpers
// --------------------------------------------------------

inline float absDiff(float a, float b) {
  return (a > b) ? (a - b) : (b - a);
}

void updateMorseTiming();

// Average voltage for RF measurement (FWD/REV)
float readAveragedVoltage(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < NUM_SAMPLES_HF; i++) {
    int raw = analogRead(pin);
    if (raw < 0) raw = 0;
    if (raw > ADC_MAX) raw = ADC_MAX;
    sum += raw;
  }
  float avgRaw = (float)sum / (float)NUM_SAMPLES_HF;
  return (avgRaw / ADC_MAX) * VREF;
}

// Read ladder voltage for buttons (smaller sample count)
float readPTTVoltage() {
  const int samples = 16;
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    int raw = analogRead(PIN_PTT);
    if (raw < 0) raw = 0;
    if (raw > ADC_MAX) raw = ADC_MAX;
    sum += raw;
  }
  float avgRaw = (float)sum / (float)samples;
  return (avgRaw / ADC_MAX) * VREF;
}

// Voltage -> raw button (0 = none, 1..6 = T1..T6)
uint8_t readPTTButtonRaw() {
  float v = readPTTVoltage();

  // no button
  if (absDiff(v, BTN_MID_NO) <= BTN_RANGE_NO) return 0;

  if (absDiff(v, BTN_MID_T1) <= BTN_RANGE_T1) return 1;
  if (absDiff(v, BTN_MID_T2) <= BTN_RANGE_T2) return 2;
  if (absDiff(v, BTN_MID_T3) <= BTN_RANGE_T3) return 3;
  if (absDiff(v, BTN_MID_T4) <= BTN_RANGE_T4) return 4;
  if (absDiff(v, BTN_MID_T5) <= BTN_RANGE_T5) return 5;
  if (absDiff(v, BTN_MID_T6) <= BTN_RANGE_T6) return 6;

  return 0;
}

// --------------------------------------------------------
// RF measurement and calibration
// --------------------------------------------------------

// Fallback: quadratic approximation
float voltageToPowerFallback(float U, float a, float b) {
  float num = U - b;
  if (num <= 0.0f || a <= 0.0f) return 0.0f;
  float p = (num / a) * (num / a);
  return (p < 0.0f) ? 0.0f : p;
}

// Main function: voltage (V) -> power (W) using a direction-specific table
float voltageToPowerTable(float U, const float *tabU, const float *tabP, int tabN, bool tabValid,
                          float aFallback, float bFallback) {
  if (!tabValid || tabN < 3) {
    return voltageToPowerFallback(U, aFallback, bFallback);
  }

  if (U <= 0.0f) return 0.0f;

  // Find segment in ascending U
  int i = 0;
  while (i < tabN && tabU[i] < U) {
    i++;
  }

  if (i == 0) {
    // below first calibration point -> scale linearly downwards
    if (tabU[0] <= 0.0f || tabP[0] <= 0.0f) return 0.0f;
    float scale = U / tabU[0];
    return tabP[0] * scale;
  }

  if (i >= tabN) {
    // above last calibration point -> linear extrapolation of last two
    int j1 = tabN - 2;
    int j2 = tabN - 1;
    float U1 = tabU[j1];
    float U2 = tabU[j2];
    float P1 = tabP[j1];
    float P2 = tabP[j2];
    if (U2 <= U1) return P2;
    float t = (U - U1) / (U2 - U1);
    return P1 + t * (P2 - P1);
  }

  // interpolate between tabU[i-1] and tabU[i]
  float U1 = tabU[i - 1];
  float U2 = tabU[i];
  float P1 = tabP[i - 1];
  float P2 = tabP[i];

  if (U2 <= U1) return P1;

  float t = (U - U1) / (U2 - U1);
  return P1 + t * (P2 - P1);
}

// Build derived interpolation tables from the 30-row measurement table.
// Call after loading or saving calibration.

/*
  === Calibration Concept (Two-Phase Model) ===

  Phase 1: Calibration Data Acquisition
  -------------------------------------
  You enter measured reference values into the calibration table:
    - Pfwd_W   : forward power in Watts from a reference wattmeter
    - Ufwd_V   : forward ADC voltage (ESP input voltage)
    - Uref_V   : reflected ADC voltage (ESP input voltage)
    - SWR_read : SWR from a reference instrument

  During this phase, any internally calculated SWR (SWR_calc) is NOT used
  for calibration and may be inaccurate. Only the measured reference values
  are stored (NVS / export JSON).

  Phase 2: Validation & Operation
  -------------------------------
  From the stored measurement table we derive two independent interpolation
  curves:
    Ufwd_V -> Pfwd_W   (forward curve)
    Uref_V -> Pref_W   (reverse curve; Pref is derived from Pfwd + SWR_read)

  After saving/importing, the device calculates SWR from live Ufwd/Uref using
  these curves. SWR_calc is then used ONLY as a validation tool by comparing
  it to SWR_read at test points.

  This strict separation makes the calibration reproducible and stable.
*/
void rebuildDerivedCalibrationTables() {
  calFwdN = 0;
  calRevN = 0;

  // Collect forward points
  for (int i = 0; i < CAL_ROWS; i++) {
    if (calRows[i].Pfwd_W > 0.0f && calRows[i].Ufwd_V > 0.0f) {
      calFwdP[calFwdN] = calRows[i].Pfwd_W;
      calFwdU[calFwdN] = calRows[i].Ufwd_V;
      calFwdN++;
    }
    if (calRows[i].Pfwd_W > 0.0f && calRows[i].Uref_V > 0.0f && calRows[i].Swr_read >= 1.0f) {
      // Derive reflected power from forward power and reference SWR:
      // gamma = (SWR-1)/(SWR+1), Pref = Pfwd * gamma^2
      float g = (calRows[i].Swr_read - 1.0f) / (calRows[i].Swr_read + 1.0f);
      if (g < 0.0f) g = 0.0f;
      float Pref = calRows[i].Pfwd_W * g * g;

      if (Pref > 0.0f) {
        calRevP[calRevN] = Pref;
        calRevU[calRevN] = calRows[i].Uref_V;
        calRevN++;
      }
    }
  }

  // Sort by voltage (simple bubble sort; N <= 30)
  auto sortPairsByU = [](float *u, float *p, int n) {
    for (int a = 0; a < n - 1; a++) {
      for (int b = 0; b < n - 1 - a; b++) {
        if (u[b] > u[b + 1]) {
          float tu = u[b];
          u[b] = u[b + 1];
          u[b + 1] = tu;
          float tp = p[b];
          p[b] = p[b + 1];
          p[b + 1] = tp;
        }
      }
    }
  };

  sortPairsByU(calFwdU, calFwdP, calFwdN);
  sortPairsByU(calRevU, calRevP, calRevN);

  // Basic validity: at least 3 points each direction
  calibrationFwdValid = (calFwdN >= 3);
  calibrationRevValid = (calRevN >= 3);

  Serial.print("Derived cal points: FWD=");
  Serial.print(calFwdN);
  Serial.print(" REV=");
  Serial.println(calRevN);
}

float calcSWR(float Pfwd, float Prev) {
  if (Pfwd < 0.01f) return 1.0f;
  if (Prev <= 0.0f) return 1.0f;

  float gamma = sqrtf(Prev / Pfwd);
  if (gamma >= 1.0f) return 20.0f;

  float swr = (1.0f + gamma) / (1.0f - gamma);
  if (swr < 1.0f) swr = 1.0f;
  if (swr > 20.0f) swr = 20.0f;
  return swr;
}

// Helper for the calibration web page: interpolate (Voltage -> Power) from derived calibration tables.
// Uses linear interpolation between the nearest points and clamps outside the known range.
// (The main measurement path uses voltageToPowerTable(); here we only need a lightweight
// lookup for "SWR calc" rendering on the HTML page.)
float interpolatePowerFromVoltage(float U, const float *uTab, const float *pTab, int n) {
  if (!uTab || !pTab || n < 2) return 0.0f;
  if (U <= uTab[0]) return pTab[0];
  if (U >= uTab[n - 1]) return pTab[n - 1];

  // Find segment
  for (int i = 0; i < n - 1; i++) {
    float u0 = uTab[i];
    float u1 = uTab[i + 1];
    if (U >= u0 && U <= u1) {
      float p0 = pTab[i];
      float p1 = pTab[i + 1];
      float du = (u1 - u0);
      if (fabsf(du) < 1e-6f) return p0;
      float t = (U - u0) / du;
      return p0 + t * (p1 - p0);
    }
  }
  // Shouldn't happen with sorted tables, but be safe.
  return pTab[n - 1];
}

// Perform measurement and update globals
void measurePowerAndSWR(float &P_fwd, float &P_rev, float &swr) {
  float U_fwd = readAveragedVoltage(PIN_FWD);
  float U_rev = readAveragedVoltage(PIN_REV);

  // Independent calibration per direction (non-symmetric coupler)
  P_fwd = voltageToPowerTable(U_fwd, calFwdU, calFwdP, calFwdN, calibrationFwdValid, A_FWD, B_FWD);
  P_rev = voltageToPowerTable(U_rev, calRevU, calRevP, calRevN, calibrationRevValid, A_REV, B_REV);

  if (P_fwd < P_MIN_ACTIVE) {
    // below threshold: no meaningful SWR -> set to 1.0
    swr = 1.0f;
    P_rev = 0.0f;
    g_powerW = 0.0f;
    g_swr = 1.0f;
  } else {
    swr = calcSWR(P_fwd, P_rev);
    g_powerW = P_fwd;
    g_swr = swr;
  }

  Serial.println("---- Measurement ----");
  Serial.print("U_fwd[V]: ");
  Serial.print(U_fwd, 3);
  Serial.print("  U_rev[V]: ");
  Serial.print(U_rev, 3);
  Serial.print("  P_fwd[W]: ");
  Serial.print(P_fwd, 2);
  Serial.print("  P_rev[W]: ");
  Serial.print(P_rev, 2);
  Serial.print("  SWR: ");
  Serial.println(swr, 2);
}



void requestCancelOutput() {
  cancelOutput = true;
  // stop buzzer immediately
  ledcWrite(PIN_BUZZER, 0);
  ledcWriteTone(PIN_BUZZER, 0);
  // stop audio amp (if on)
  audioAmpOff();
}

void clearCancelOutput() {
  cancelOutput = false;
}

void beepCount(uint8_t count) {
  // count = 1..5, encoded as short beeps, abortable
  for (uint8_t i = 0; i < count; i++) {
    if (readPTTButtonRaw() != 0) cancelOutput = true;
    if (cancelOutput) return;
    ledcWriteTone(PIN_BUZZER, MORSE_TONE_FREQ);
    ledcWrite(PIN_BUZZER, BUZZER_DUTY);
    unsigned long t0 = millis();
    while (millis() - t0 < 60) {  // ON
      if (readPTTButtonRaw() != 0) cancelOutput = true;
      if (cancelOutput) {
        ledcWrite(PIN_BUZZER, 0);
        return;
      }
      server.handleClient();
      yield();
    }
    ledcWrite(PIN_BUZZER, 0);
    t0 = millis();
    while (millis() - t0 < 60) {  // OFF
      if (readPTTButtonRaw() != 0) cancelOutput = true;
      if (cancelOutput) return;
      server.handleClient();
      yield();
    }
  }
}

// --------------------------------------------------------
// Buzzer / Morse
// --------------------------------------------------------

void updateMorseTiming() {
  if (morseBpm < MIN_MORSE_BPM) morseBpm = MIN_MORSE_BPM;
  if (morseBpm > MAX_MORSE_BPM) morseBpm = MAX_MORSE_BPM;
  if (morseBpm <= 0) morseBpm = 5;
  morseUnitMs = 1200 / morseBpm;
}

void morseTone(bool on, int units) {
  if (on) {
    ledcWriteTone(PIN_BUZZER, MORSE_TONE_FREQ);
    ledcWrite(PIN_BUZZER, BUZZER_DUTY);
  } else {
    ledcWrite(PIN_BUZZER, 0);
  }
  unsigned long t0 = millis();
  unsigned long dur = (unsigned long)morseUnitMs * (unsigned long)units;
  while (millis() - t0 < dur) {
    if (readPTTButtonRaw() != 0) cancelOutput = true;
    if (cancelOutput) {
      ledcWrite(PIN_BUZZER, 0);
      return;
    }
    server.handleClient();
    yield();
    delay(5);
  }
}

const char *digitToMorse(char c) {
  switch (c) {
    case '0': return "-----";
    case '1': return ".----";
    case '2': return "..---";
    case '3': return "...--";
    case '4': return "....-";
    case '5': return ".....";
    case '6': return "-....";
    case '7': return "--...";
    case '8': return "---..";
    case '9': return "----.";
    default: return "";
  }
}

void playMorseDigit(char c) {
  const char *pattern = digitToMorse(c);
  if (*pattern == '\0') return;

  for (const char *p = pattern; *p != '\0'; ++p) {
    if (*p == '.') {
      morseTone(true, 1);
    } else if (*p == '-') {
      morseTone(true, 3);
    }
    morseTone(false, 1);
  }
  morseTone(false, 2);
}

void playTwoDigits(int value) {
  if (value < 0) value = 0;
  if (value > 99) value = 99;

  int tens = value / 10;
  int ones = value % 10;

  char c_tens = '0' + tens;
  char c_ones = '0' + ones;

  playMorseDigit(c_tens);
  playMorseDigit(c_ones);
}

void playMeasurementMorse(int powerInt, int swrTimes10Int) {
  // first power, then SWR*10, with gaps
  playTwoDigits(powerInt);
  morseTone(false, 4);
  playTwoDigits(swrTimes10Int);
  morseTone(false, 7);
}

void announceMorseSpeed() {
  int val = morseBpm;
  if (val < 0) val = 0;
  if (val > 99) val = 99;

  Serial.print("New Morse speed: ");
  Serial.print(morseBpm);
  Serial.println(" WPM");

  playTwoDigits(val);
  morseTone(false, 7);
}

// Startup "OK" in Morse at 20 WPM
void playStartupOK() {
  int oldBpm = morseBpm;
  morseBpm = 20;
  updateMorseTiming();

  // O = --- (3 dashes)
  morseTone(true, 3);
  morseTone(false, 1);
  morseTone(true, 3);
  morseTone(false, 1);
  morseTone(true, 3);
  morseTone(false, 3);  // gap between letters

  // K = -.- (dash dot dash)
  morseTone(true, 3);
  morseTone(false, 1);
  morseTone(true, 1);
  morseTone(false, 1);
  morseTone(true, 3);
  morseTone(false, 3);  // end gap

  morseBpm = oldBpm;
  updateMorseTiming();
}

// --------------------------------------------------------
// I2S / Audio
// --------------------------------------------------------

void audioAmpOn() {
  digitalWrite(AMP_SD_PIN, HIGH);
}

void audioAmpOff() {
  digitalWrite(AMP_SD_PIN, LOW);
}

void initI2S() {
  i2s_config_t i2s_config;
  memset(&i2s_config, 0, sizeof(i2s_config));

  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.sample_rate = I2S_SAMPLE_RATE;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
  i2s_config.intr_alloc_flags = 0;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 256;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = true;

  i2s_pin_config_t pin_config;
  memset(&pin_config, 0, sizeof(pin_config));
  pin_config.bck_io_num = I2S_BCLK_PIN;
  pin_config.ws_io_num = I2S_LRCLK_PIN;
  pin_config.data_out_num = I2S_DOUT_PIN;
  pin_config.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t err;

  err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("i2s_driver_install error: %d\n", err);
  } else {
    Serial.println("I2S driver installed.");
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("i2s_set_pin error: %d\n", err);
  } else {
    Serial.println("I2S pins configured.");
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
}

// --------------------------------------------------------
// Streaming PROGMEM clips with volume control
// --------------------------------------------------------

bool playClipProgmemGain(const uint8_t *data, size_t length, float clipGain) {
  const size_t CHUNK = 512;
  static uint8_t buffer[CHUNK];

  const size_t totalSamples = length / 2;
  const size_t rampSamples = (totalSamples > 320) ? 160 : (totalSamples / 2);  // ~10ms fade at 16 kHz

  size_t offsetBytes = 0;
  while (offsetBytes < length) {
    // allow cancel even while we're "blocking" inside playback
    if (readPTTButtonRaw() != 0) cancelOutput = true;
    if (cancelOutput) return false;

    size_t n = length - offsetBytes;
    if (n > CHUNK) n = CHUNK;

    memcpy(buffer, data + offsetBytes, n);

    // apply volume (16-bit PCM, little endian) with per-clip gain + short fade-in/out
    int16_t *samples = (int16_t *)buffer;
    size_t sampleCount = n / 2;
    size_t sampleBase = offsetBytes / 2;

    const float g = SPEECH_VOLUME * clipGain;

    for (size_t i = 0; i < sampleCount; i++) {
      size_t sIdx = sampleBase + i;

      float fade = 1.0f;
      if (rampSamples > 0) {
        if (sIdx < rampSamples) {
          fade = (float)sIdx / (float)rampSamples;
        } else if (sIdx > (totalSamples - rampSamples)) {
          size_t t = totalSamples - sIdx;
          if (t > rampSamples) t = rampSamples;
          fade = (float)t / (float)rampSamples;
        }
      }

      int32_t v = samples[i];
      v = (int32_t)(v * g * fade);
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }

    size_t bytesWritten = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, buffer, n, &bytesWritten, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("i2s_write error: %d\n", err);
      return false;
    }

    // keep web responsive
    server.handleClient();
    yield();

    offsetBytes += n;
  }

  return true;
}

bool playClipProgmem(const uint8_t *data, size_t length) {
  return playClipProgmemGain(data, length, 1.0f);
}


// --------------------------------------------------------
// Voice clips (PROGMEM) with per-clip loudness equalization
// --------------------------------------------------------

enum VoiceClip : uint8_t {
  CLIP_ZERO = 0,
  CLIP_ONE,
  CLIP_TWO,
  CLIP_THREE,
  CLIP_FOUR,
  CLIP_FIVE,
  CLIP_SIX,
  CLIP_SEVEN,
  CLIP_EIGHT,
  CLIP_NINE,
  CLIP_POINT,
  CLIP_POWER,
  CLIP_WATTS,
  CLIP_SWR,
  CLIP_COUNT
};

const float CLIP_GAIN[CLIP_COUNT] = { 0.973f, 0.832f, 1.212f, 0.943f, 0.704f, 0.937f, 1.663f, 1.096f, 1.410f, 0.919f, 1.034f, 1.028f, 0.904f, 1.126f };

bool playClip(VoiceClip clip) {
  switch (clip) {
    case CLIP_ZERO: return playClipProgmemGain(voice_zero, voice_zero_len, CLIP_GAIN[clip]);
    case CLIP_ONE: return playClipProgmemGain(voice_one, voice_one_len, CLIP_GAIN[clip]);
    case CLIP_TWO: return playClipProgmemGain(voice_two, voice_two_len, CLIP_GAIN[clip]);
    case CLIP_THREE: return playClipProgmemGain(voice_three, voice_three_len, CLIP_GAIN[clip]);
    case CLIP_FOUR: return playClipProgmemGain(voice_four, voice_four_len, CLIP_GAIN[clip]);
    case CLIP_FIVE: return playClipProgmemGain(voice_five, voice_five_len, CLIP_GAIN[clip]);
    case CLIP_SIX: return playClipProgmemGain(voice_six, voice_six_len, CLIP_GAIN[clip]);
    case CLIP_SEVEN: return playClipProgmemGain(voice_seven, voice_seven_len, CLIP_GAIN[clip]);
    case CLIP_EIGHT: return playClipProgmemGain(voice_eight, voice_eight_len, CLIP_GAIN[clip]);
    case CLIP_NINE: return playClipProgmemGain(voice_nine, voice_nine_len, CLIP_GAIN[clip]);
    case CLIP_POINT: return playClipProgmemGain(voice_point, voice_point_len, CLIP_GAIN[clip]);
    case CLIP_POWER: return playClipProgmemGain(voice_power, voice_power_len, CLIP_GAIN[clip]);
    case CLIP_WATTS: return playClipProgmemGain(voice_watts, voice_watts_len, CLIP_GAIN[clip]);
    case CLIP_SWR: return playClipProgmemGain(voice_swr, voice_swr_len, CLIP_GAIN[clip]);
    default: return false;
  }
}

// --------------------------------------------------------
inline bool playVoiceClip(VoiceClip clip) { return playClip(clip); }
inline bool playVoiceClipProgmem(const uint8_t* data, size_t length) { return playClipProgmem(data, length); }

// Numbers -> voice building blocks
// --------------------------------------------------------

void playDigit(int d) {
  switch (d) {
    case 0: playVoiceClip(CLIP_ZERO); break;
    case 1: playVoiceClip(CLIP_ONE); break;
    case 2: playVoiceClip(CLIP_TWO); break;
    case 3: playVoiceClip(CLIP_THREE); break;
    case 4: playVoiceClip(CLIP_FOUR); break;
    case 5: playVoiceClip(CLIP_FIVE); break;
    case 6: playVoiceClip(CLIP_SIX); break;
    case 7: playVoiceClip(CLIP_SEVEN); break;
    case 8: playVoiceClip(CLIP_EIGHT); break;
    case 9: playVoiceClip(CLIP_NINE); break;
  }
}

void sayNumberInt0to99(int n) {
  if (n < 0) n = 0;
  if (n > 99) n = 99;

  int tens = n / 10;
  int ones = n % 10;

  if (tens == 0) {
    playDigit(ones);
  } else {
    playDigit(tens);
    playDigit(ones);
  }
}

void speakWatts(float P_fwd) {
  // "power" + number + "watts"
  playVoiceClip(CLIP_POWER);

  if (P_fwd < P_MIN_ACTIVE) {
    playDigit(0);
  } else {
    int P_round = (int)(P_fwd + 0.5f);
    if (P_round < 0) P_round = 0;
    if (P_round > 99) P_round = 99;
    sayNumberInt0to99(P_round);
  }

  playVoiceClip(CLIP_WATTS);
}

void speakSWR(float swr) {
  if (swr < 1.0f) swr = 1.0f;
  if (swr > 9.9f) swr = 9.9f;

  playVoiceClipProgmem(voice_swr, voice_swr_len);

  int swr10 = (int)(swr * 10.0f + 0.5f);
  int integer = swr10 / 10;
  int fraction = swr10 % 10;

  playDigit(integer);
  playVoiceClip(CLIP_POINT);
  playDigit(fraction);
}

// Full speech measurement
void speakMeasurement() {
  float P_fwd, P_rev, swr;
  measurePowerAndSWR(P_fwd, P_rev, swr);

  audioAmpOn();
  delay(5);

  speakWatts(P_fwd);

  if (SPEECH_PAUSE_MS > 0) {
    delay(SPEECH_PAUSE_MS);
  }

  speakSWR(swr);

  int16_t silence[80];
  memset(silence, 0, sizeof(silence));
  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, silence, sizeof(silence), &bytesWritten, portMAX_DELAY);

  audioAmpOff();
}

// T2 long: voice self test (play all clips)
void playAllVoiceFilesTest() {
  Serial.println("Starting voice self-test (all clips from PROGMEM)...");

  audioAmpOn();
  delay(5);

  playVoiceClip(CLIP_ZERO);
  playVoiceClip(CLIP_ONE);
  playVoiceClip(CLIP_TWO);
  playVoiceClip(CLIP_THREE);
  playVoiceClip(CLIP_FOUR);
  playVoiceClip(CLIP_FIVE);
  playVoiceClip(CLIP_SIX);
  playVoiceClip(CLIP_SEVEN);
  playVoiceClip(CLIP_EIGHT);
  playVoiceClip(CLIP_NINE);

  playVoiceClip(CLIP_POINT);
  playVoiceClip(CLIP_POWER);
  playVoiceClip(CLIP_WATTS);
  playVoiceClip(CLIP_SWR);

  int16_t silence[80];
  memset(silence, 0, sizeof(silence));
  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, silence, sizeof(silence), &bytesWritten, portMAX_DELAY);

  audioAmpOff();

  Serial.println("Voice self-test finished.");
}

// --------------------------------------------------------
// Tuning mode
// --------------------------------------------------------

int swrToFrequency(float swr) {
  if (swr < TUNING_SWR_MIN) swr = TUNING_SWR_MIN;
  if (swr > TUNING_SWR_MAX) swr = TUNING_SWR_MAX;

  float ratio = (swr - TUNING_SWR_MIN) / (TUNING_SWR_MAX - TUNING_SWR_MIN);
  float f = TUNING_FREQ_MIN + ratio * (TUNING_FREQ_MAX - TUNING_FREQ_MIN);
  return (int)(f + 0.5f);
}

void startTuningMode() {
  tuningModeActive = true;
  tuningBestSWR = 999.0f;
  lastTuningSample = 0;
  tuningLastFreq = 0;
  tuningHasTone = false;

  Serial.println("Tuning mode ON");

  // reference tone for SWR = 1
  int fRef = swrToFrequency(1.0f);
  ledcWriteTone(PIN_BUZZER, fRef);
  ledcWrite(PIN_BUZZER, BUZZER_DUTY);
  delay(1000);
  ledcWrite(PIN_BUZZER, 0);
  delay(500);
}

void stopTuningMode() {
  tuningModeActive = false;
  Serial.println("Tuning mode OFF");

  ledcWrite(PIN_BUZZER, 0);
  tuningHasTone = false;
  tuningLastFreq = 0;
}

void handleTuningMode() {
  unsigned long now = millis();
  if (now - lastTuningSample < TUNING_SAMPLE_MS) {
    return;
  }
  lastTuningSample = now;

  float U_fwd = readAveragedVoltage(PIN_FWD);
  float U_rev = readAveragedVoltage(PIN_REV);
  float P_fwd = voltageToPowerTable(U_fwd, calFwdU, calFwdP, calFwdN, calibrationFwdValid, A_FWD, B_FWD);
  float P_rev = voltageToPowerTable(U_rev, calRevU, calRevP, calRevN, calibrationRevValid, A_REV, B_REV);
  float swr = (P_fwd < P_MIN_ACTIVE) ? 1.0f : calcSWR(P_fwd, P_rev);

  if (P_fwd < P_MIN_ACTIVE) {
    g_powerW = 0.0f;
    g_swr = 1.0f;
    ledcWrite(PIN_BUZZER, 0);
    tuningHasTone = false;
    return;
  } else {
    g_powerW = P_fwd;
    g_swr = swr;
  }

  int f = swrToFrequency(swr);

  bool newMin = (swr < tuningBestSWR - TUNING_MIN_DELTA);
  if (newMin) {
    tuningBestSWR = swr;

    Serial.print("New tuning SWR minimum: ");
    Serial.println(swr, 2);

    // short beep to indicate new best SWR
    ledcWriteTone(PIN_BUZZER, MORSE_TONE_FREQ);
    ledcWrite(PIN_BUZZER, BUZZER_DUTY);
    delay(morseUnitMs);
    ledcWrite(PIN_BUZZER, 0);
  }

  if (!tuningHasTone || abs(f - tuningLastFreq) > 3) {
    ledcWriteTone(PIN_BUZZER, f);
    ledcWrite(PIN_BUZZER, BUZZER_DUTY);
    tuningLastFreq = f;
    tuningHasTone = true;
  }
}

// --------------------------------------------------------
// Factory defaults
// --------------------------------------------------------

void resetConfigToDefaults() {
  // Measurement
  P_MIN_ACTIVE = 1.0f;

  // Morse
  DEFAULT_MORSE_BPM = 15;
  MIN_MORSE_BPM = 5;
  MAX_MORSE_BPM = 40;
  STEP_MORSE_BPM = 5;
  MORSE_TONE_FREQ = 800;

  // Button timings
  BUTTON_LONG_MS = 700;
  DEBOUNCE_MS = 40;

  // Ladder mid voltages
  BTN_MID_NO = 3.300f;
  BTN_MID_T1 = 0.265f;
  BTN_MID_T2 = 0.540f;
  BTN_MID_T3 = 0.973f;
  BTN_MID_T4 = 1.560f;
  BTN_MID_T5 = 2.184f;
  BTN_MID_T6 = 2.658f;

  // Ladder ranges
  BTN_RANGE_NO = 0.257f;
  BTN_RANGE_T1 = 0.110f;
  BTN_RANGE_T2 = 0.110f;
  BTN_RANGE_T3 = 0.173f;
  BTN_RANGE_T4 = 0.235f;
  BTN_RANGE_T5 = 0.189f;
  BTN_RANGE_T6 = 0.189f;

  // Tuning
  TUNING_SAMPLE_MS = 100;
  TUNING_SWR_MIN = 1.0f;
  TUNING_SWR_MAX = 5.0f;
  TUNING_FREQ_MIN = 100;
  TUNING_FREQ_MAX = 3000;
  TUNING_MIN_DELTA = 0.02f;

  // Speech
  SPEECH_PAUSE_MS = 200;
  speechLevelIndex = 2;
  SPEECH_VOLUME = SPEECH_LEVELS[speechLevelIndex];

  // Buzzer
  buzzerLevelIndex = 3;
  BUZZER_DUTY = BUZZER_LEVELS[buzzerLevelIndex];

  // Calibration defaults
  P_MAX_3V2 = 100.0f;
  calibrationFwdValid = false;
  calibrationRevValid = false;

  // Clear table
  for (int i = 0; i < CAL_ROWS; i++) {
    calRows[i].Pfwd_W = 0.0f;
    calRows[i].Ufwd_V = 0.0f;
    // derived: reflected power is calculated from Pfwd_W + Swr_read
    // (see rebuildDerivedCalibrationTables)

    calRows[i].Uref_V = 0.0f;
    calRows[i].Swr_read = 0.0f;
  }

  rebuildDerivedCalibrationTables();
  morseBpm = DEFAULT_MORSE_BPM;
  updateMorseTiming();
}

// --------------------------------------------------------
// Calibration save/load
// --------------------------------------------------------

void saveCalibrationToNVS() {
  prefs.begin("swrCal", false);

  // Store a version so we can evolve the layout later
  prefs.putUInt("ver", 3);

  prefs.putFloat("Pmax", P_MAX_3V2);

  // Store the whole table as a binary blob (more compact than 150 separate keys)
  prefs.putBytes("rows", (const void *)calRows, sizeof(calRows));

  // Store validity flags (derived tables are rebuilt on boot anyway)
  prefs.putBool("fwdOk", calibrationFwdValid);
  prefs.putBool("revOk", calibrationRevValid);

  prefs.end();
  Serial.println("Calibration saved to NVS.");
}

void loadCalibrationFromNVS() {
  prefs.begin("swrCal", true);

  uint32_t ver = prefs.getUInt("ver", 0);
  if (ver == 2) {
    // Migrate older layout (ver 2): rows contained Prev_W and Urev_V.
    struct CalRowV2 {
      float Pfwd_W;
      float Ufwd_V;
      float Prev_W;
      float Urev_V;
      float Swr_read;
    };
    CalRowV2 oldRows[CAL_ROWS];
    size_t need = sizeof(oldRows);
    if (prefs.isKey("rows") && prefs.getBytesLength("rows") == need) {
      prefs.getBytes("rows", (void *)oldRows, need);
      for (int i = 0; i < CAL_ROWS; i++) {
        calRows[i].Pfwd_W = oldRows[i].Pfwd_W;
        calRows[i].Ufwd_V = oldRows[i].Ufwd_V;
        calRows[i].Uref_V = oldRows[i].Urev_V;
        calRows[i].Swr_read = oldRows[i].Swr_read;
      }
    } else {
      memset(calRows, 0, sizeof(calRows));
      applyDefaultPfwdPresetsIfEmpty();
    }
  } else if (ver == 3) {
    // Current layout
    size_t need = sizeof(calRows);
    if (prefs.isKey("rows") && prefs.getBytesLength("rows") == need) {
      prefs.getBytes("rows", (void *)calRows, need);
    } else {
      memset(calRows, 0, sizeof(calRows));
      applyDefaultPfwdPresetsIfEmpty();
    }
  } else {
    // Older or unknown layout -> ignore and use defaults/fallback
    prefs.end();
    Serial.println("No compatible calibration in NVS (using fallback / empty table).");
    calibrationFwdValid = false;
    calibrationRevValid = false;
    // If table is empty, prefill Pfwd targets to make measuring faster.
    applyDefaultPfwdPresetsIfEmpty();
    // calRows stays whatever is in RAM; derived tables rebuilt anyway
    rebuildDerivedCalibrationTables();
    return;
  }


  // Load fallback parameters (still used outside calibrated ranges / if table invalid)
  A_FWD = prefs.getFloat("Afwd", A_FWD);
  B_FWD = prefs.getFloat("Bfwd", B_FWD);
  A_REV = prefs.getFloat("Arev", A_REV);
  B_REV = prefs.getFloat("Brev", B_REV);
  VREF_3V2 = prefs.getFloat("Vref", VREF_3V2);
  P_MAX_3V2 = prefs.getFloat("Pmax", P_MAX_3V2);

  calibrationFwdValid = prefs.getBool("fwdOk", false);
  calibrationRevValid = prefs.getBool("revOk", false);

  prefs.end();

  rebuildDerivedCalibrationTables();
  Serial.println("Calibration loaded from NVS.");
}


  // --------------------------------------------------------
  // User settings (persist across reboot)
  // --------------------------------------------------------

  void loadUserSettingsFromNVS() {
    if (!prefsUser.begin("swrSet", true)) {
      Serial.println("WARN: prefsUser begin (RO) failed");
      return;
    }

    uint16_t ver = prefsUser.getUShort("ver", 0);
    if (ver != USER_SETTINGS_VERSION) {
      prefsUser.end();
      Serial.printf("User settings version mismatch (have %u, want %u). Resetting to defaults.\n", ver, USER_SETTINGS_VERSION);
      resetToDefaultUserSettings();
      // Save defaults back to NVS
      saveUserSettingsToNVS();
      return;
    }

    speechLevelIndex = prefsUser.getInt("speechIdx", speechLevelIndex);
    buzzerLevelIndex = prefsUser.getInt("buzzIdx", buzzerLevelIndex);
    morseBpm = prefsUser.getInt("morseBpm", morseBpm);

    prefsUser.end();

    // apply derived values
    if (speechLevelIndex < 0) speechLevelIndex = 0;
    if (speechLevelIndex > 4) speechLevelIndex = 4;
    SPEECH_VOLUME = SPEECH_LEVELS[speechLevelIndex];

    if (buzzerLevelIndex < 0) buzzerLevelIndex = 0;
    if (buzzerLevelIndex > 4) buzzerLevelIndex = 4;
    BUZZER_DUTY = BUZZER_LEVELS[buzzerLevelIndex];

    updateMorseTiming();

    Serial.println("User settings loaded from NVS.");
  }

  void saveUserSettingsToNVS() {
    if (!prefsUser.begin("swrSet", false)) {
      Serial.println("WARN: prefsUser begin (RW) failed");
      return;
    }
    prefsUser.putUShort("ver", USER_SETTINGS_VERSION);
    prefsUser.putInt("speechIdx", speechLevelIndex);
    prefsUser.putInt("buzzIdx", buzzerLevelIndex);
    prefsUser.putInt("morseBpm", morseBpm);
    prefsUser.end();

    settingsDirty = false;
    Serial.println("User settings saved to NVS.");
    // acoustic feedback: double-beep (abortable)
    beepCount(2);
  }

String buildHtmlPage() {
  String page =
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>SWR Meter S3</title>"
    "<style>"
    "body{font-family:sans-serif;padding:1.5rem;background:#111;color:#eee;}"
    "h1{font-size:1.6rem;margin-bottom:0.5rem;}"
    ".box{border:1px solid #555;border-radius:0.5rem;padding:1rem;margin-top:1rem;}"
    ".label{color:#aaa;font-size:0.9rem;}"
    ".value{font-size:1.8rem;margin:0.3rem 0;}"
    ".btn{display:inline-block;padding:0.3rem 0.7rem;margin-top:0.7rem;"
    "border:1px solid #555;border-radius:0.4rem;background:#fff;color:#000;text-decoration:none;margin-right:0.5rem;}"
    "</style>"
    "</head><body>";
  page += "<h1>SWR Meter (ESP32-S3 FW ";
  page += String(FW_VERSION);
  page += ", PROGMEM voice)</h1>";
  page += "<p>Last measured values:</p>"
    "<div class='box'>"
    "<div class='label'>Output power</div>"
    "<div class='value' id='powerVal'>";
  page += String(g_powerW, 1);
  page += " W</div><div class='label'>SWR</div><div class='value' id='swrVal'>";
  page += String(g_swr, 2);
  page +=
    "</div></div>"
    "<div class='box'>"
    "<div class='label'>Current settings</div>"
    "<div class='label'>Speaker volume</div><div class='value' id='spkVol'>-</div>"
    "<div class='label'>Buzzer volume</div><div class='value' id='buzVol'>-</div>"
    "<div class='label'>Morse speed (BPM)</div><div class='value' id='morseBpm'>-</div>"
    "<form method='POST' action='/resetSettings' onsubmit='return confirm(\"Reset user settings to defaults?\");'>"
    "<button class='btn' type='submit'>Reset Settings</button>"
    "</form>"
    "</div>"
    "<p><a class='btn' href='/config'>Configuration</a>"
    "<a class='btn' href='/cal'>Calibration</a></p>"
    "<p>Note: values are updated on T1/T2 measurements and in tuning mode.</p>"
    "<script>"
    "async function poll(){"
    "try{"
    "const r=await fetch('/data');"
    "if(!r.ok)return;"
    "const d=await r.json();"
    "document.getElementById('powerVal').textContent=d.power.toFixed(1)+' W';"
    "document.getElementById('swrVal').textContent=d.swr.toFixed(2);if(d.speechIdx!==undefined){document.getElementById('spkVol').textContent=(d.speechIdx+1)+' / 5';}if(d.buzzIdx!==undefined){document.getElementById('buzVol').textContent=(d.buzzIdx+1)+' / 5';}if(d.morseBpm!==undefined){document.getElementById('morseBpm').textContent=d.morseBpm;}"
    "}catch(e){}"
    "setTimeout(poll,1000);"
    "}"
    "window.addEventListener('load',poll);"
    "</script>"
    "</body></html>";
  return page;
}

String buildConfigPage(const String &msg) {
  String page =
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>SWR Meter Configuration</title>"
    "<style>"
    "body{font-family:sans-serif;padding:1.5rem;background:#111;color:#eee;}"
    "h1{font-size:1.6rem;}"
    "h2{font-size:1.2rem;margin:1.2rem 0 0.4rem 0;}"
    "fieldset{border:1px solid #555;margin-bottom:1rem;padding:0.7rem;border-radius:0.5rem;}"
    "legend{padding:0 0.3rem;}"
    "label{display:block;margin-top:0.3rem;font-size:0.9rem;}"
    "input{width:100%;max-width:10rem;padding:0.2rem;margin-top:0.1rem;background:#222;color:#eee;border:1px solid #555;border-radius:0.3rem;}"
    "button{font-size:1.0rem;padding:0.4rem 0.8rem;margin-top:0.8rem;}"
    ".btn{display:inline-block;padding:0.3rem 0.7rem;border:1px solid #555;border-radius:0.4rem;background:#fff;color:#000;text-decoration:none;margin-right:0.5rem;}"
    ".msg{margin-bottom:0.7rem;color:#7f7;}"
    "</style>"
    "</head><body>"
    "<h1>SWR Meter Configuration</h1>"
    "<p><a class='btn' href='/'>Back to status</a>"
    "<a class='btn' href='/cal'>Calibration</a></p>";

  if (msg.length() > 0) {
    page += "<div class='msg'>" + msg + "</div>";
  }

  page += "<form method='POST' action='/saveConfig'>";

  // Measurement
  page += "<h2 id=\'measurement\'>Measurement</h2>";
  page += "<fieldset><legend>Measurement</legend>";
  page += "<label>P_MIN_ACTIVE [W]<br><input name='P_MIN_ACTIVE' value='" + String(P_MIN_ACTIVE, 2) + "'></label>";
  page += "</fieldset>";

  // Speech
  page += "<h2 id=\'speech\'>Speech output</h2>";
  page += "<fieldset><legend>Speech output</legend>";
  page += "<label>SPEECH_PAUSE_MS [ms]<br><input name='SPEECH_PAUSE_MS' value='" + String(SPEECH_PAUSE_MS) + "'></label>";
  page += "<label>Current SPEECH_VOLUME factor (info, changed by T4/T3 in 5 steps)<br>"
          "<input disabled value='"
          + String(SPEECH_VOLUME, 2) + "'></label>";
  page += "</fieldset>";

  // Buzzer
  page += "<h2 id=\'buzzer\'>Buzzer</h2>";
  page += "<fieldset><legend>Buzzer</legend>";
  page += "<label>Current BUZZER_DUTY (info, changed by T3 in 5 steps)<br>"
          "<input disabled value='"
          + String(BUZZER_DUTY) + "'></label>";
  page += "</fieldset>";

  // Morse
  page += "<h2 id=\'morse\'>Morse</h2>";
  page += "<fieldset><legend>Morse</legend>";
  page += "<label>DEFAULT_MORSE_BPM<br><input name='DEFAULT_MORSE_BPM' value='" + String(DEFAULT_MORSE_BPM) + "'></label>";
  page += "<label>MIN_MORSE_BPM<br><input name='MIN_MORSE_BPM' value='" + String(MIN_MORSE_BPM) + "'></label>";
  page += "<label>MAX_MORSE_BPM<br><input name='MAX_MORSE_BPM' value='" + String(MAX_MORSE_BPM) + "'></label>";
  page += "<label>STEP_MORSE_BPM<br><input name='STEP_MORSE_BPM' value='" + String(STEP_MORSE_BPM) + "'></label>";
  page += "<label>MORSE_TONE_FREQ [Hz]<br><input name='MORSE_TONE_FREQ' value='" + String(MORSE_TONE_FREQ) + "'></label>";
  page += "</fieldset>";

  // Button timings
  page += "<h2 id=\'buttons\'>Button timings</h2>";
  page += "<fieldset><legend>Button timings</legend>";
  page += "<label>BUTTON_LONG_MS [ms]<br><input name='BUTTON_LONG_MS' value='" + String(BUTTON_LONG_MS) + "'></label>";
  page += "<label>DEBOUNCE_MS [ms]<br><input name='DEBOUNCE_MS' value='" + String(DEBOUNCE_MS) + "'></label>";
  page += "</fieldset>";

  // Button ladder mid voltages
  page += "<h2 id=\'ladder-mid\'>Button ladder mid voltages [V]</h2>";
  page += "<fieldset><legend>Button ladder mid voltages [V]</legend>";
  page += "<label>BTN_MID_NO<br><input name='BTN_MID_NO' value='" + String(BTN_MID_NO, 3) + "'></label>";
  page += "<label>BTN_MID_T1<br><input name='BTN_MID_T1' value='" + String(BTN_MID_T1, 3) + "'></label>";
  page += "<label>BTN_MID_T2<br><input name='BTN_MID_T2' value='" + String(BTN_MID_T2, 3) + "'></label>";
  page += "<label>BTN_MID_T3<br><input name='BTN_MID_T3' value='" + String(BTN_MID_T3, 3) + "'></label>";
  page += "<label>BTN_MID_T4<br><input name='BTN_MID_T4' value='" + String(BTN_MID_T4, 3) + "'></label>";
  page += "<label>BTN_MID_T5<br><input name='BTN_MID_T5' value='" + String(BTN_MID_T5, 3) + "'></label>";
  page += "<label>BTN_MID_T6<br><input name='BTN_MID_T6' value='" + String(BTN_MID_T6, 3) + "'></label>";
  page += "</fieldset>";

  // Button ladder ranges
  page += "<h2 id=\'ladder-range\'>Button ladder ranges ± [V]</h2>";
  page += "<fieldset><legend>Button ladder ranges ± [V]</legend>";
  page += "<label>BTN_RANGE_NO<br><input name='BTN_RANGE_NO' value='" + String(BTN_RANGE_NO, 3) + "'></label>";
  page += "<label>BTN_RANGE_T1<br><input name='BTN_RANGE_T1' value='" + String(BTN_RANGE_T1, 3) + "'></label>";
  page += "<label>BTN_RANGE_T2<br><input name='BTN_RANGE_T2' value='" + String(BTN_RANGE_T2, 3) + "'></label>";
  page += "<label>BTN_RANGE_T3<br><input name='BTN_RANGE_T3' value='" + String(BTN_RANGE_T3, 3) + "'></label>";
  page += "<label>BTN_RANGE_T4<br><input name='BTN_RANGE_T4' value='" + String(BTN_RANGE_T4, 3) + "'></label>";
  page += "<label>BTN_RANGE_T5<br><input name='BTN_RANGE_T5' value='" + String(BTN_RANGE_T5, 3) + "'></label>";
  page += "<label>BTN_RANGE_T6<br><input name='BTN_RANGE_T6' value='" + String(BTN_RANGE_T6, 3) + "'></label>";
  page += "</fieldset>";

  // Tuning
  page += "<h2 id=\'tuning\'>Tuning mode</h2>";
  page += "<fieldset><legend>Tuning mode</legend>";
  page += "<label>TUNING_SAMPLE_MS [ms]<br><input name='TUNING_SAMPLE_MS' value='" + String(TUNING_SAMPLE_MS) + "'></label>";
  page += "<label>TUNING_SWR_MIN<br><input name='TUNING_SWR_MIN' value='" + String(TUNING_SWR_MIN, 2) + "'></label>";
  page += "<label>TUNING_SWR_MAX<br><input name='TUNING_SWR_MAX' value='" + String(TUNING_SWR_MAX, 2) + "'></label>";
  page += "<label>TUNING_FREQ_MIN [Hz]<br><input name='TUNING_FREQ_MIN' value='" + String(TUNING_FREQ_MIN) + "'></label>";
  page += "<label>TUNING_FREQ_MAX [Hz]<br><input name='TUNING_FREQ_MAX' value='" + String(TUNING_FREQ_MAX) + "'></label>";
  page += "<label>TUNING_MIN_DELTA<br><input name='TUNING_MIN_DELTA' value='" + String(TUNING_MIN_DELTA, 3) + "'></label>";
  page += "</fieldset>";

  page += "<button type='submit'>Save configuration</button></form>";

  page +=
    "<form method='POST' action='/factoryReset' "
    "onsubmit=\"return confirm('Reset all settings (including calibration) to factory defaults?');\">"
    "<button type='submit'>Restore factory defaults</button>"
    "</form>";

  page += "</body></html>";
  return page;
}

String buildCalPage(const String &msg) {
  String page =
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>SWR Meter Calibration</title>"
    "<style>"
    "body{font-family:sans-serif;padding:1.5rem;background:#111;color:#eee;}"
    "h1{font-size:1.6rem;}"
    "h2{font-size:1.2rem;margin:1.2rem 0 0.4rem 0;}"
    "table{border-collapse:collapse;width:100%;max-width:60rem;}"
    "th,td{border:1px solid #555;padding:0.25rem;font-size:0.85rem;text-align:center;}"
    "input{width:100%;box-sizing:border-box;padding:0.2rem;background:#222;color:#eee;border:1px solid #555;border-radius:0.3rem;font-size:0.85rem;}"
    "input[readonly]{opacity:0.8;}"
    "fieldset{border:1px solid #555;margin-bottom:1rem;padding:0.7rem;border-radius:0.5rem;}"
    "legend{padding:0 0.3rem;}"
    ".btn{display:inline-block;padding:0.3rem 0.7rem;border:1px solid #555;border-radius:0.4rem;background:#fff;color:#000;text-decoration:none;margin-right:0.5rem;}"
    "button{font-size:1.0rem;padding:0.4rem 0.8rem;margin-top:0.8rem;}"
    ".msg{margin-bottom:0.7rem;color:#7f7;}"
    ".hint{color:#aaa;font-size:0.9rem;}"
    "</style>"
    "</head><body>"
    "<h1>SWR Meter Calibration (30 rows)</h1>"
    "<p><a class='btn' href='/'>Back to status</a>"
    "<a class='btn' href='/config'>Configuration</a></p>";

  if (msg.length() > 0) {
    page += "<div class='msg'>" + msg + "</div>";
  }

  page += "<p class='hint'>Goal: enter REAL measured values. The firmware will build two independent calibration curves "
          "(forward and reverse) using linear interpolation. This handles a non-symmetric directional coupler.</p>";

  page +=
    "<p class='hint'>Columns:</p>"
    "<ul class='hint'>"
    "<li><b>Pfwd</b>: output / true forward power in W from a reference wattmeter</li>"
    "<li><b>Ufwd</b> and <b>Uref</b>: ESP ADC voltages at the forward and reflected inputs</li>"
    "<li><b>SWR read</b>: reference SWR value (optional, for comparison)</li>"
    "<li><b>SWR calc</b>: calculated from Ufwd/Uref using the currently stored calibration curves</li>"
    "</ul>";

  page += "<form method='POST' action='/saveCal'>";

  page += "<h2 id=\'general\'>General</h2>";
  page += "<fieldset><legend>General</legend>";
  page += "<label>P_MAX_3V2 (info only: max power around 3.2V)<br>"
          "<input name='P_MAX_3V2' value='"
          + String(P_MAX_3V2, 1) + "'></label>";
  page += "<div class='hint'>Current derived points: FWD=" + String(calFwdN) + " ("
          + (calibrationFwdValid ? "active" : "fallback") + "), REV=" + String(calRevN) + " ("
          + (calibrationRevValid ? "active" : "fallback") + ")</div>";
  page += "</fieldset>";

  page += "<h2 id=\'table\'>Measurement table</h2>";
  page += "<fieldset><legend>Measurement table</legend>";
  page += "<table>";
  page += "<tr>"
          "<th>#</th>"
          "<th>Pfwd [W]</th><th>Ufwd [V]</th>"
          "<th>Uref [V]</th>"
          "<th>SWR read</th><th>SWR calc</th>"
          "</tr>";

  for (int i = 0; i < CAL_ROWS; i++) {
    float swr_calc = 1.0f;
    // Calculate using currently stored calibration curves (voltage -> power)
    float Pf = (calRows[i].Ufwd_V > 0.0f) ? interpolatePowerFromVoltage(calRows[i].Ufwd_V, calFwdU, calFwdP, calFwdN) : 0.0f;
    float Pr = (calRows[i].Uref_V > 0.0f) ? interpolatePowerFromVoltage(calRows[i].Uref_V, calRevU, calRevP, calRevN) : 0.0f;
    swr_calc = calcSWR(Pf, Pr);

    page += "<tr><td>" + String(i + 1) + "</td>";

    page += "<td><input name='Pf" + String(i) + "' value='" + String(calRows[i].Pfwd_W, 2) + "'></td>";
    page += "<td><input name='Uf" + String(i) + "' value='" + String(calRows[i].Ufwd_V, 3) + "'></td>";
    page += "<td><input name='Ur" + String(i) + "' value='" + String(calRows[i].Uref_V, 3) + "'></td>";
    page += "<td><input name='Sr" + String(i) + "' value='" + String(calRows[i].Swr_read, 2) + "'></td>";

    // Calculated SWR is read-only (updated by JS too)
    page += "<td><input readonly id='Sc" + String(i) + "' value='" + String(swr_calc, 2) + "'></td>";

    page += "</tr>";
  }

  page += "</table>";
  page += "</fieldset>";

  page += "<button type='submit'>Save calibration</button>";
  page += "</form>";
  page += "<div style='height:0.9rem'></div>";

  // Backup / restore calibration (JSON)
  page += "<h2 id=\'backup\'>Backup / Restore</h2>";
  page += "<fieldset><legend>Backup / Restore</legend>";
  page += "<a class='btn' id='exportCal' href='/cal/export'>Export calibration (JSON)</a>";
  page += "<div class='hint' style='margin-top:0.4rem'>Export creates a JSON file you can archive on your PC. Import replaces the current calibration and rebuilds the derived curves.</div>";
  page += "<div style='height:0.6rem'></div>";
  page += "<form method='POST' action='/cal/import' enctype='multipart/form-data'>";
  page += "<input type='file' name='calfile' accept='.json,application/json' required> ";
  page += "<button type='submit'>Import calibration</button>";
  page += "</form>";
  page += "</fieldset>";


  // JS: live SWR calc (approximation based on Uref/Ufwd ratio).
  // Exact SWR calc on page load uses the stored calibration curves on the ESP32.
  page += "<script>"
          "function toNum(x){const v=parseFloat(x);return isNaN(v)?0:v;}"
          "function clipSWR(s){if(s<1)s=1; if(s>20)s=20; return s;}"
          "function calcSWRfromU(Uf,Ur){"
          "if(Uf<=0||Ur<=0) return 1.0;"
          "let g=Ur/Uf;"
          "if(g>=0.999) return 20.0;"
          "let s=(1+g)/(1-g);"
          "return clipSWR(s);"
          "}"
          "function hookRow(i){"
          "const uf=document.querySelector(`[name='Uf${i}']`);"
          "const ur=document.querySelector(`[name='Ur${i}']`);"
          "const out=document.getElementById(`Sc${i}`);"
          "function upd(){out.value=calcSWRfromU(toNum(uf.value),toNum(ur.value)).toFixed(2);}"
          "uf.addEventListener('input',upd);"
          "ur.addEventListener('input',upd);"
          "}";
  page += "for(let i=0;i<" + String(CAL_ROWS) + ";i++)hookRow(i);";
  page += "const ex=document.getElementById('exportCal');"
          "if(ex){"
          "const d=new Date();"
          "const pad=(n)=>String(n).padStart(2,'0');"
          "const ts=`${d.getFullYear()}${pad(d.getMonth()+1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}`;"
          "ex.href=`/cal/export?ts=${ts}`;"
          "}";
  page += "</script>";

  page += "</body></html>";
  return page;
}

// --------------------------------------------------------
// Webserver handlers
// --------------------------------------------------------

void handleRoot() {
  server.send(200, "text/html", buildHtmlPage());
}

void handleData() {
  String json = "{ \"power\": ";
  json += String(g_powerW, 3);
  json += ", \"swr\": ";
  json += String(g_swr, 3);

  // current settings
  json += ", \"speechIdx\": ";
  json += String(speechLevelIndex);
  json += ", \"buzzIdx\": ";
  json += String(buzzerLevelIndex);
  json += ", \"morseBpm\": ";
  json += String(morseBpm);

  json += " }";
  server.send(200, "application/json", json);
}


void handleConfig() {
  server.send(200, "text/html", buildConfigPage(""));
}

void handleCal() {
  server.send(200, "text/html", buildCalPage(""));
}

void handleSaveConfig() {
  auto getFloat = [](const String &name, float &var) {
    if (server.hasArg(name)) {
      var = server.arg(name).toFloat();
    }
  };
  auto getULong = [](const String &name, unsigned long &var) {
    if (server.hasArg(name)) {
      var = (unsigned long)server.arg(name).toInt();
    }
  };
  auto getInt = [](const String &name, int &var) {
    if (server.hasArg(name)) {
      var = server.arg(name).toInt();
    }
  };

  // Measurement
  getFloat("P_MIN_ACTIVE", P_MIN_ACTIVE);

  // Speech
  getULong("SPEECH_PAUSE_MS", SPEECH_PAUSE_MS);

  // Morse
  getInt("DEFAULT_MORSE_BPM", DEFAULT_MORSE_BPM);
  getInt("MIN_MORSE_BPM", MIN_MORSE_BPM);
  getInt("MAX_MORSE_BPM", MAX_MORSE_BPM);
  getInt("STEP_MORSE_BPM", STEP_MORSE_BPM);
  getInt("MORSE_TONE_FREQ", MORSE_TONE_FREQ);

  // Button timings
  getULong("BUTTON_LONG_MS", BUTTON_LONG_MS);
  getULong("DEBOUNCE_MS", DEBOUNCE_MS);

  // Ladder mid voltages
  getFloat("BTN_MID_NO", BTN_MID_NO);
  getFloat("BTN_MID_T1", BTN_MID_T1);
  getFloat("BTN_MID_T2", BTN_MID_T2);
  getFloat("BTN_MID_T3", BTN_MID_T3);
  getFloat("BTN_MID_T4", BTN_MID_T4);
  getFloat("BTN_MID_T5", BTN_MID_T5);
  getFloat("BTN_MID_T6", BTN_MID_T6);

  // Ladder ranges
  getFloat("BTN_RANGE_NO", BTN_RANGE_NO);
  getFloat("BTN_RANGE_T1", BTN_RANGE_T1);
  getFloat("BTN_RANGE_T2", BTN_RANGE_T2);
  getFloat("BTN_RANGE_T3", BTN_RANGE_T3);
  getFloat("BTN_RANGE_T4", BTN_RANGE_T4);
  getFloat("BTN_RANGE_T5", BTN_RANGE_T5);
  getFloat("BTN_RANGE_T6", BTN_RANGE_T6);

  // Tuning
  getULong("TUNING_SAMPLE_MS", TUNING_SAMPLE_MS);
  getFloat("TUNING_SWR_MIN", TUNING_SWR_MIN);
  getFloat("TUNING_SWR_MAX", TUNING_SWR_MAX);
  getInt("TUNING_FREQ_MIN", TUNING_FREQ_MIN);
  getInt("TUNING_FREQ_MAX", TUNING_FREQ_MAX);
  getFloat("TUNING_MIN_DELTA", TUNING_MIN_DELTA);

  // Consistency checks
  if (TUNING_SWR_MIN < 1.0f) TUNING_SWR_MIN = 1.0f;
  if (TUNING_SWR_MAX <= TUNING_SWR_MIN) TUNING_SWR_MAX = TUNING_SWR_MIN + 0.5f;

  if (MIN_MORSE_BPM < 1) MIN_MORSE_BPM = 1;
  if (MAX_MORSE_BPM <= MIN_MORSE_BPM) MAX_MORSE_BPM = MIN_MORSE_BPM + 1;

  morseBpm = DEFAULT_MORSE_BPM;
  updateMorseTiming();

  server.send(200, "text/html", buildConfigPage("Configuration saved."));
}

void handleSaveCal() {
  auto getFloat = [](const String &name, float &outVar) {
    if (server.hasArg(name)) {
      outVar = server.arg(name).toFloat();
    }
  };

  getFloat("P_MAX_3V2", P_MAX_3V2);

  for (int i = 0; i < CAL_ROWS; i++) {
    String nPf = "Pf" + String(i);
    String nUf = "Uf" + String(i);
    String nUr = "Ur" + String(i);
    String nSr = "Sr" + String(i);

    getFloat(nPf, calRows[i].Pfwd_W);
    getFloat(nUf, calRows[i].Ufwd_V);
    getFloat(nUr, calRows[i].Uref_V);
    getFloat(nSr, calRows[i].Swr_read);

    // sanitize negatives
    if (calRows[i].Pfwd_W < 0.0f) calRows[i].Pfwd_W = 0.0f;
    if (calRows[i].Ufwd_V < 0.0f) calRows[i].Ufwd_V = 0.0f;
    if (calRows[i].Uref_V < 0.0f) calRows[i].Uref_V = 0.0f;
    if (calRows[i].Swr_read < 0.0f) calRows[i].Swr_read = 0.0f;
  }

  // Rebuild derived tables and decide validity
  rebuildDerivedCalibrationTables();

  // Store validity flags (for info); tables are derived again on boot
  saveCalibrationToNVS();

  String msg;
  if (calibrationFwdValid && calibrationRevValid) {
    msg = "Calibration saved. Forward + reverse curves active.";
  } else if (calibrationFwdValid && !calibrationRevValid) {
    msg = "Calibration saved. Forward curve active, reverse uses fallback (REV needs >= 3 valid points).";
  } else if (!calibrationFwdValid && calibrationRevValid) {
    msg = "Calibration saved. Reverse curve active, forward uses fallback (FWD needs >= 3 valid points).";
  } else {
    msg = "Calibration saved, but not enough valid points (need >= 3 per direction) – fallback active.";
  }

  server.send(200, "text/html", buildCalPage(msg));
}

// --- Calibration Export / Import (Option A: JSON via Web UI) ---

// Helper: extract a float value for a given key from a JSON object string.
static float jsonGetFloat(const String &obj, const char *key, float defVal) {
  String k = String("\"") + key + "\"";
  int p = obj.indexOf(k);
  if (p < 0) return defVal;
  p = obj.indexOf(':', p);
  if (p < 0) return defVal;
  p++;
  while (p < (int)obj.length() && (obj[p] == ' ' || obj[p] == '\n' || obj[p] == '\r' || obj[p] == '\t')) p++;

  int e = p;
  while (e < (int)obj.length()) {
    char c = obj[e];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
      e++;
    } else {
      break;
    }
  }
  if (e <= p) return defVal;
  return obj.substring(p, e).toFloat();
}

static bool parseCalibrationJson(const String &json, String &err) {
  // Reset table
  memset(calRows, 0, sizeof(calRows));

  int pPoints = json.indexOf("\"points\"");
  if (pPoints < 0) {
    err = "JSON missing 'points' array.";
    return false;
  }

  int pArr = json.indexOf('[', pPoints);
  if (pArr < 0) {
    err = "JSON 'points' is not an array.";
    return false;
  }

  int i = pArr + 1;
  int row = 0;

  while (i < (int)json.length() && row < CAL_ROWS) {
    int arrEnd = json.indexOf(']', i);
    int o = json.indexOf('{', i);
    if (o < 0) break;
    if (arrEnd >= 0 && o > arrEnd) break;

    int brace = 0;
    int j = o;
    for (; j < (int)json.length(); j++) {
      if (json[j] == '{') brace++;
      else if (json[j] == '}') {
        brace--;
        if (brace == 0) {
          j++;
          break;
        }
      }
    }
    if (brace != 0) {
      err = "JSON object braces not balanced.";
      return false;
    }

    String obj = json.substring(o, j);

    calRows[row].Pfwd_W = jsonGetFloat(obj, "Pfwd_W", 0.0f);
    calRows[row].Ufwd_V = jsonGetFloat(obj, "Ufwd_V", 0.0f);
    calRows[row].Uref_V = jsonGetFloat(obj, "Uref_V", 0.0f);
    calRows[row].Swr_read = jsonGetFloat(obj, "SWR_read", 0.0f);

    row++;
    i = j;
  }

  // If import was empty, keep presets to make measuring easier
  applyDefaultPfwdPresetsIfEmpty();

  rebuildDerivedCalibrationTables();
  saveCalibrationToNVS();
  return true;
}

void handleCalExport() {
  // Build a compact JSON (only non-empty rows)
  String out;
  out.reserve(4096);

  out += "{\n";
  out += "  \"version\": 3,\n";
  out += "  \"cal_rows\": " + String(CAL_ROWS) + ",\n";
  out += "  \"points\": [\n";

  bool first = true;
  for (int i = 0; i < CAL_ROWS; i++) {
    const CalRow &r = calRows[i];
    bool nonEmpty = (r.Pfwd_W != 0.0f) || (r.Ufwd_V != 0.0f) || (r.Uref_V != 0.0f) || (r.Swr_read != 0.0f);
    if (!nonEmpty) continue;

    if (!first) out += ",\n";
    first = false;

    out += "    {";
    out += "\"Pfwd_W\":" + String(r.Pfwd_W, 3) + ",";
    out += "\"Ufwd_V\":" + String(r.Ufwd_V, 4) + ",";
    out += "\"Uref_V\":" + String(r.Uref_V, 4) + ",";
    out += "\"SWR_read\":" + String(r.Swr_read, 3);
    out += "}";
  }

  out += "\n  ]\n";
  out += "}\n";

  String ts = server.hasArg("ts") ? server.arg("ts") : String("unknown");
  ts.replace("\\", "");
  ts.replace("\"", "");
  ts.replace("..", "");
  ts.replace("/", "");
  ts.replace(" ", "");
  String fname = "SWR_Calibration_" + ts + ".json";
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
  server.send(200, "application/json", out);
}

void handleCalImportUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_calImportBuf = "";
    g_calImportBuf.reserve(8192);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // Append incoming chunk (text JSON)
    g_calImportBuf.concat((const char *)upload.buf, upload.currentSize);
  }
}

void handleCalImport() {
  String json;

  // Prefer multipart upload buffer; fall back to raw body (application/json)
  if (g_calImportBuf.length() > 0) {
    json = g_calImportBuf;
    g_calImportBuf = "";
  } else if (server.hasArg("plain")) {
    json = server.arg("plain");
  }

  if (json.length() < 10) {
    server.send(400, "text/html", buildCalPage("Import failed: empty file/body."));
    return;
  }

  String err;
  if (!parseCalibrationJson(json, err)) {
    server.send(400, "text/html", buildCalPage("Import failed: " + err));
    return;
  }

  server.send(200, "text/html", buildCalPage("Calibration imported and saved to NVS."));
}


void handleFactoryReset() {
  resetConfigToDefaults();
  calibrationFwdValid = false;
  calibrationRevValid = false;
  saveCalibrationToNVS();  // clears table too
  server.send(200, "text/html", buildConfigPage("Factory defaults loaded (including calibration)."));
}


void handleResetSettings() {
  // Reset ONLY user settings (volume/morse) to defaults and persist them.
  resetToDefaultUserSettings();

  if (prefsUser.begin("swrSet", false)) {
    prefsUser.clear();
    prefsUser.end();
  }

  // Save defaults back to NVS (also writes version)
  saveUserSettingsToNVS();

  server.send(200, "text/plain", "OK: settings reset. Rebooting...");
  delay(200);
  ESP.restart();
}

void handleNotFound() {
  server.send(404, "text/plain", "404 - Not found");
}

// --------------------------------------------------------
// Button event handler
// --------------------------------------------------------

void handleButtonEvent(uint8_t button, bool isShort) {
  // Any button press cancels ongoing speech/morse/beeps immediately
  requestCancelOutput();
  // allow cancellation to propagate into any running loops
  delay(1);
  clearCancelOutput();

  Serial.print("Button event: T");
  Serial.print(button);
  Serial.println(isShort ? " short" : " long");

  // T1: Morse / Tuning
  if (button == 1) {
    // if tuning mode is active, T1 (short or long) turns tuning OFF
    if (tuningModeActive) {
      stopTuningMode();
      return;
    }

    if (isShort) {
      // T1 short: measurement + Morse output
      float P_fwd, P_rev, swr;
      measurePowerAndSWR(P_fwd, P_rev, swr);

      int powerInt;
      int swrTimes10Int;

      if (P_fwd < P_MIN_ACTIVE) {
        powerInt = 0;
        swrTimes10Int = 0;
      } else {
        float P_round = P_fwd + 0.5f;
        if (P_round < 0.0f) P_round = 0.0f;
        if (P_round > 99.0f) P_round = 99.0f;
        powerInt = (int)P_round;

        float swrTimes10 = swr * 10.0f + 0.5f;
        if (swrTimes10 < 0.0f) swrTimes10 = 0.0f;
        if (swrTimes10 > 99.0f) swrTimes10 = 99.0f;
        swrTimes10Int = (int)swrTimes10;
      }

      Serial.print("-> Morse powerInt: ");
      Serial.print(powerInt);
      Serial.print("  SWR*10 int: ");
      Serial.println(swrTimes10Int);

      playMeasurementMorse(powerInt, swrTimes10Int);
    } else {
      // T1 long: enable tuning mode
      startTuningMode();
    }
    return;
  }

  // In tuning mode: ignore other buttons
  if (tuningModeActive) {
    Serial.println("Tuning mode: other buttons ignored.");
    return;
  }

  // T2: speech
  if (button == 2) {
    if (isShort) {
      // T2 short: measurement + speech output
      speakMeasurement();
    } else {
      // T2 long: speech self-test
      playAllVoiceFilesTest();
    }
    return;
  }

  // T3: buzzer volume (5 steps, wrap-around)
  if (button == 3) {
    if (isShort) {
      // one step quieter
      if (buzzerLevelIndex == 0) buzzerLevelIndex = 4;
      else buzzerLevelIndex--;
    } else {
      // one step louder
      if (buzzerLevelIndex == 4) buzzerLevelIndex = 0;
      else buzzerLevelIndex++;
    }
    BUZZER_DUTY = BUZZER_LEVELS[buzzerLevelIndex];

    Serial.print("New buzzer level: index=");
    Serial.print(buzzerLevelIndex);
    Serial.print("  duty=");
    Serial.println(BUZZER_DUTY);

    markSettingsDirty();

    // short confirmation beep
    ledcWriteTone(PIN_BUZZER, MORSE_TONE_FREQ);
    ledcWrite(PIN_BUZZER, BUZZER_DUTY);
    delay(150);
    ledcWrite(PIN_BUZZER, 0);
    delay(50);
    return;
  }

  // T4: speech volume (5 steps, wrap-around)
  if (button == 4) {
    if (isShort) {
      if (speechLevelIndex == 0) speechLevelIndex = 4;
      else speechLevelIndex--;
    } else {
      if (speechLevelIndex == 4) speechLevelIndex = 0;
      else speechLevelIndex++;
    }
    SPEECH_VOLUME = SPEECH_LEVELS[speechLevelIndex];

    Serial.print("New speech level: index=");
    Serial.print(speechLevelIndex);
    Serial.print("  factor=");
    Serial.println(SPEECH_VOLUME, 2);

    // beep-code: number of beeps = speech level (1..5)
    markSettingsDirty();
    beepCount((uint8_t)(speechLevelIndex + 1));
    return;
  }

  // T5/T6: Morse speed
  if (button == 5) {
    if (isShort) {
      morseBpm -= STEP_MORSE_BPM;
      updateMorseTiming();
    } else {
      morseBpm = DEFAULT_MORSE_BPM;
      updateMorseTiming();
      markSettingsDirty();
    }
    announceMorseSpeed();
    return;
  }

  if (button == 6) {
    if (isShort) {
      morseBpm += STEP_MORSE_BPM;
      updateMorseTiming();
    } else {
      morseBpm = DEFAULT_MORSE_BPM;
      updateMorseTiming();
      markSettingsDirty();
    }
    announceMorseSpeed();
    return;
  }

  Serial.println("This button has no special function in this sketch.");
}

// --------------------------------------------------------
// setup() and loop()
// --------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== ESP32-S3 SWR Meter V5.4.0: UX+A11Y+mDNS+Prefs+AudioEq (PROGMEM voice) ===");

  // Load defaults
  resetConfigToDefaults();

  // Try load calibration from NVS (overwrites default calibration if valid)
  loadCalibrationFromNVS();

  // Load persisted user settings (volume, morse speed, etc.)
  loadUserSettingsFromNVS();

  // ADC setup
  analogReadResolution(12);
  pinMode(PIN_FWD, INPUT);
  pinMode(PIN_REV, INPUT);
  pinMode(PIN_PTT, INPUT);

  // Buzzer via LEDC
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  bool buzzerOk = ledcAttach(PIN_BUZZER, 2000, BUZZER_PWM_BITS);  // 2 kHz, 10-bit
  Serial.print("LEDC buzzer attach: ");
  Serial.println(buzzerOk ? "OK" : "ERROR");

  // AMP
  pinMode(AMP_SD_PIN, OUTPUT);
  audioAmpOff();

  // I2S
  initI2S();

  Serial.println("Using speech data from PROGMEM (voice_data.h).");
  Serial.println("No SD card required.");

  // WiFi AP
  Serial.println("Starting WiFi access point...");
  WiFi.mode(WIFI_AP);
  WiFi.setHostname(HOSTNAME);
  bool apOk = WiFi.softAP(AP_SSID.c_str());
  if (!apOk) {
    Serial.println("ERROR: Access point could not be started!");
  } else {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP started, SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP IP: ");
    Serial.println(ip);
  }

  // mDNS (best-effort; depends on client/network support)
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("mDNS responder started: http://");
    Serial.print(HOSTNAME);
    Serial.println(".local/");
  } else {
    Serial.println("WARN: mDNS responder could not be started.");
  }

  // Webserver routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/config", handleConfig);
  server.on("/saveConfig", HTTP_POST, handleSaveConfig);
  server.on("/factoryReset", HTTP_POST, handleFactoryReset);
  server.on("/resetSettings", HTTP_POST, handleResetSettings);
  server.on("/cal", handleCal);
  server.on("/cal/export", HTTP_GET, handleCalExport);
  server.on("/cal/import", HTTP_POST, handleCalImport, handleCalImportUpload);
  server.on("/saveCal", HTTP_POST, handleSaveCal);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("Webserver running on port 80.");

  Serial.println("Setup complete.");
  Serial.println("T1 short: Morse measurement, T1 long: tuning mode ON/OFF.");
  Serial.println("T2 short: speech measurement, T2 long: voice self-test.");
  Serial.println("T3: buzzer volume (5 steps), T4: speech volume (5 steps).");
  Serial.println("T5/T6: Morse speed control.");

  // Startup “OK” in Morse at 20 WPM
  playStartupOK();
}

void loop() {
  // HTTP handling
  server.handleClient();

  // Button debounce and edge detection
  uint8_t newRaw = readPTTButtonRaw();

  if (newRaw != rawButton) {
    rawButton = newRaw;
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) >= DEBOUNCE_MS) {
    currentButton = rawButton;
  }

  if (currentButton != 0 && lastButton == 0) {
    // button pressed
    pressedButton = currentButton;
    pressStartTime = millis();

    float vNow = readPTTVoltage();
    Serial.print("Button pressed: T");
    Serial.print(pressedButton);
    Serial.print("  (V = ");
    Serial.print(vNow, 3);
    Serial.println(")");
  } else if (currentButton == 0 && lastButton != 0) {
    // button released
    unsigned long pressDuration = millis() - pressStartTime;
    bool isShort = (pressDuration < BUTTON_LONG_MS);

    if (pressedButton != 0) {
      handleButtonEvent(pressedButton, isShort);
    }
    pressedButton = 0;
  }

  lastButton = currentButton;

  // Tuning mode handling
  if (tuningModeActive) {
    handleTuningMode();
  }


  // Persist user settings (deferred save to reduce flash wear)
  if (settingsDirty && (millis() - lastSettingsChangeMs) > 1500) {
    saveUserSettingsToNVS();
  }

  delay(10);
}