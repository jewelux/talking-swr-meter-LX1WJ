/*
 * ESP32-S3 TALKING SWR METER
 * Version 4.5
 * 
 * Copyright (C) 2025 Jean Weber LX1WJ
 * Licensed under the GNU GPLv3.
 */


#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

extern "C" {
  #include "driver/i2s.h"
}

// PROGMEM voice bank
#include "voice_data.h"

// ========================================================
// ESP32-S3 TALKING SWR METER V4.4
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
const int I2S_BCLK_PIN  = 4;
const int I2S_LRCLK_PIN = 5;
const int I2S_DOUT_PIN  = 6;
const int AMP_SD_PIN    = 7;  // SD/EN of MAX98357

// Buzzer (Morse, tuning, short beeps) via LEDC
const int PIN_BUZZER       = 8;
const int BUZZER_PWM_BITS  = 10;                 // 10-bit -> 0..1023
const int BUZZER_PWM_MAX   = (1 << BUZZER_PWM_BITS) - 1;
const int BUZZER_TONE_FREQ = 800;                // base beep / Morse freq

// I2S / Audio
const int I2S_SAMPLE_RATE = 16000;    // Hz, must match voice samples

// ADC parameters
const float VREF    = 3.30f;
const int   ADC_MAX = 4095;
const int   NUM_SAMPLES_HF = 64;

// RF fallback calibration (used when no valid table exists in NVS)
const float A_FWD = 0.316f;
const float B_FWD = -0.175f;
const float A_REV = 0.316f;
const float B_REV = -0.175f;

// Minimum power for meaningful SWR
float P_MIN_ACTIVE = 1.0f;  // W -> below 1W no meaningful SWR

// Global last measurement values (for speech + web)
float g_powerW = 0.0f;
float g_swr    = 1.0f;

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
unsigned long BUTTON_LONG_MS = 700;   // >= 700 ms = long press
unsigned long DEBOUNCE_MS    = 40;

// Button state variables
uint8_t       rawButton        = 0;  // 0 = none, 1..6 = T1..T6
uint8_t       currentButton    = 0;
uint8_t       lastButton       = 0;
uint8_t       pressedButton    = 0;
unsigned long lastDebounceTime = 0;
unsigned long pressStartTime   = 0;

// --------------------------------------------------------
// Morse parameters
// --------------------------------------------------------

int   DEFAULT_MORSE_BPM = 15;
int   MIN_MORSE_BPM     = 5;
int   MAX_MORSE_BPM     = 40;
int   STEP_MORSE_BPM    = 5;
int   MORSE_TONE_FREQ   = 800;   // Hz for Morse

int   morseBpm    = DEFAULT_MORSE_BPM;
int   morseUnitMs = 1200 / 15;   // base dot length in ms

// --------------------------------------------------------
// Tuning parameters
// --------------------------------------------------------

unsigned long TUNING_SAMPLE_MS = 100;  // new sample every 100 ms

float TUNING_SWR_MIN   = 1.0f;
float TUNING_SWR_MAX   = 5.0f;
int   TUNING_FREQ_MIN  = 100;
int   TUNING_FREQ_MAX  = 3000;
float TUNING_MIN_DELTA = 0.02f;

// --------------------------------------------------------
// Speech parameters
// --------------------------------------------------------

// Pause between "power ... watts" and "swr ..." in ms
unsigned long SPEECH_PAUSE_MS = 200;

// Speech volume levels (5 steps via index)
const float SPEECH_LEVELS[5] = {0.10f, 0.25f, 0.50f, 0.75f, 1.00f};
int   speechLevelIndex       = 2;      // 0..4 -> default = 0.5x
float SPEECH_VOLUME          = 0.50f;  // updated from index

// --------------------------------------------------------
// Buzzer volume (5 steps)
// --------------------------------------------------------

const int BUZZER_LEVELS[5] = {2, 10, 30, 100, 700};
int       buzzerLevelIndex = 3;                 // 0..4
int       BUZZER_DUTY      = BUZZER_LEVELS[3];  // default

// --------------------------------------------------------
// Tuning state
// --------------------------------------------------------

bool   tuningModeActive        = false;
float  tuningBestSWR           = 999.0f;
unsigned long lastTuningSample = 0;
int    tuningLastFreq          = 0;
bool   tuningHasTone           = false;

// --------------------------------------------------------
// WiFi / Webserver
// --------------------------------------------------------

const char* AP_SSID = "SWR-Meter-S3-V4.4";
WebServer   server(80);

// --------------------------------------------------------
// Calibration data (FWD/REV)
// --------------------------------------------------------

Preferences prefs;
const int CAL_POINTS = 10;

float P_MAX_3V2 = 100.0f;   // info only: max power at about 3.2 V
float calP[CAL_POINTS];     // calibration power values (ascending)
float calU[CAL_POINTS];     // corresponding voltages (FWD)
bool  calibrationValid = false;

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

// Main function: voltage (V) -> power (W) using calibration points
float voltageToPowerCalibrated(float U) {
  if (!calibrationValid) {
    return voltageToPowerFallback(U, A_FWD, B_FWD);
  }

  if (U <= 0.0f) return 0.0f;

  // Check if table is all zero -> fallback
  bool allZero = true;
  for (int i = 0; i < CAL_POINTS; i++) {
    if (calP[i] > 0.0f && calU[i] > 0.0f) {
      allZero = false;
      break;
    }
  }
  if (allZero) {
    return voltageToPowerFallback(U, A_FWD, B_FWD);
  }

  // Find segment in ascending U
  int i = 0;
  while (i < CAL_POINTS && calU[i] < U) {
    i++;
  }

  if (i == 0) {
    // below first calibration point -> scale linearly downwards
    if (calU[0] <= 0.0f || calP[0] <= 0.0f) return 0.0f;
    float scale = U / calU[0];
    return calP[0] * scale;
  }

  if (i >= CAL_POINTS) {
    // above last calibration point -> linear extrapolation of last two
    int j1 = CAL_POINTS - 2;
    int j2 = CAL_POINTS - 1;
    float U1 = calU[j1];
    float U2 = calU[j2];
    float P1 = calP[j1];
    float P2 = calP[j2];
    if (U2 <= U1) return P2;
    float t = (U - U1) / (U2 - U1);
    return P1 + t * (P2 - P1);
  }

  // interpolate between calU[i-1] and calU[i]
  float U1 = calU[i - 1];
  float U2 = calU[i];
  float P1 = calP[i - 1];
  float P2 = calP[i];

  if (U2 <= U1) return P1;

  float t = (U - U1) / (U2 - U1);
  return P1 + t * (P2 - P1);
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

// Perform measurement and update globals
void measurePowerAndSWR(float &P_fwd, float &P_rev, float &swr) {
  float U_fwd = readAveragedVoltage(PIN_FWD);
  float U_rev = readAveragedVoltage(PIN_REV);

  // Use same calibration curve for FWD and REV (directional coupler characteristic)
  P_fwd = voltageToPowerCalibrated(U_fwd);
  P_rev = voltageToPowerCalibrated(U_rev);

  if (P_fwd < P_MIN_ACTIVE) {
    // below threshold: no meaningful SWR -> set to 1.0
    swr     = 1.0f;
    P_rev   = 0.0f;
    g_powerW = 0.0f;
    g_swr    = 1.0f;
  } else {
    swr     = calcSWR(P_fwd, P_rev);
    g_powerW = P_fwd;
    g_swr    = swr;
  }

  Serial.println("---- Measurement ----");
  Serial.print("U_fwd[V]: "); Serial.print(U_fwd, 3);
  Serial.print("  U_rev[V]: "); Serial.print(U_rev, 3);
  Serial.print("  P_fwd[W]: "); Serial.print(P_fwd, 2);
  Serial.print("  P_rev[W]: "); Serial.print(P_rev, 2);
  Serial.print("  SWR: ");     Serial.println(swr, 2);
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
  delay(morseUnitMs * units);
}

const char* digitToMorse(char c) {
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
    default:  return "";
  }
}

void playMorseDigit(char c) {
  const char* pattern = digitToMorse(c);
  if (*pattern == '\0') return;

  for (const char* p = pattern; *p != '\0'; ++p) {
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
  if (value < 0)  value = 0;
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
  if (val < 0)  val = 0;
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
  i2s_config.dma_buf_len   = 256;
  i2s_config.use_apll      = false;
  i2s_config.tx_desc_auto_clear = true;

  i2s_pin_config_t pin_config;
  memset(&pin_config, 0, sizeof(pin_config));
  pin_config.bck_io_num   = I2S_BCLK_PIN;
  pin_config.ws_io_num    = I2S_LRCLK_PIN;
  pin_config.data_out_num = I2S_DOUT_PIN;
  pin_config.data_in_num  = I2S_PIN_NO_CHANGE;

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

bool playClipProgmem(const uint8_t* data, size_t length) {
  const size_t CHUNK = 512;
  static uint8_t buffer[CHUNK];

  size_t offset = 0;
  while (offset < length) {
    size_t n = length - offset;
    if (n > CHUNK) n = CHUNK;

    memcpy(buffer, data + offset, n);

    // apply volume (16-bit PCM, little endian)
    int16_t* samples = (int16_t*)buffer;
    size_t sampleCount = n / 2;
    for (size_t i = 0; i < sampleCount; i++) {
      int32_t v = samples[i];
      v = (int32_t)(v * SPEECH_VOLUME);
      if (v >  32767) v =  32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }

    size_t bytesWritten = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, buffer, n, &bytesWritten, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.printf("i2s_write error: %d\n", err);
      return false;
    }

    offset += n;
  }

  return true;
}

// --------------------------------------------------------
// Numbers -> voice building blocks
// --------------------------------------------------------

void playDigit(int d) {
  switch (d) {
    case 0: playClipProgmem(voice_zero,  voice_zero_len);  break;
    case 1: playClipProgmem(voice_one,   voice_one_len);   break;
    case 2: playClipProgmem(voice_two,   voice_two_len);   break;
    case 3: playClipProgmem(voice_three, voice_three_len); break;
    case 4: playClipProgmem(voice_four,  voice_four_len);  break;
    case 5: playClipProgmem(voice_five,  voice_five_len);  break;
    case 6: playClipProgmem(voice_six,   voice_six_len);   break;
    case 7: playClipProgmem(voice_seven, voice_seven_len); break;
    case 8: playClipProgmem(voice_eight, voice_eight_len); break;
    case 9: playClipProgmem(voice_nine,  voice_nine_len);  break;
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
  playClipProgmem(voice_power, voice_power_len);

  if (P_fwd < P_MIN_ACTIVE) {
    playDigit(0);
  } else {
    int P_round = (int)(P_fwd + 0.5f);
    if (P_round < 0)   P_round = 0;
    if (P_round > 99)  P_round = 99;
    sayNumberInt0to99(P_round);
  }

  playClipProgmem(voice_watts, voice_watts_len);
}

void speakSWR(float swr) {
  if (swr < 1.0f) swr = 1.0f;
  if (swr > 9.9f) swr = 9.9f;

  playClipProgmem(voice_swr, voice_swr_len);

  int swr10 = (int)(swr * 10.0f + 0.5f);
  int integer  = swr10 / 10;
  int fraction = swr10 % 10;

  playDigit(integer);
  playClipProgmem(voice_point, voice_point_len);
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

  playClipProgmem(voice_zero,  voice_zero_len);
  playClipProgmem(voice_one,   voice_one_len);
  playClipProgmem(voice_two,   voice_two_len);
  playClipProgmem(voice_three, voice_three_len);
  playClipProgmem(voice_four,  voice_four_len);
  playClipProgmem(voice_five,  voice_five_len);
  playClipProgmem(voice_six,   voice_six_len);
  playClipProgmem(voice_seven, voice_seven_len);
  playClipProgmem(voice_eight, voice_eight_len);
  playClipProgmem(voice_nine,  voice_nine_len);

  playClipProgmem(voice_point, voice_point_len);
  playClipProgmem(voice_power, voice_power_len);
  playClipProgmem(voice_watts, voice_watts_len);
  playClipProgmem(voice_swr,   voice_swr_len);

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
  tuningBestSWR    = 999.0f;
  lastTuningSample = 0;
  tuningLastFreq   = 0;
  tuningHasTone    = false;

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
  tuningHasTone  = false;
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
  float P_fwd = voltageToPowerCalibrated(U_fwd);
  float P_rev = voltageToPowerCalibrated(U_rev);
  float swr   = (P_fwd < P_MIN_ACTIVE) ? 1.0f : calcSWR(P_fwd, P_rev);

  if (P_fwd < P_MIN_ACTIVE) {
    g_powerW = 0.0f;
    g_swr    = 1.0f;
    ledcWrite(PIN_BUZZER, 0);
    tuningHasTone = false;
    return;
  } else {
    g_powerW = P_fwd;
    g_swr    = swr;
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
    tuningHasTone  = true;
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
  MIN_MORSE_BPM     = 5;
  MAX_MORSE_BPM     = 40;
  STEP_MORSE_BPM    = 5;
  MORSE_TONE_FREQ   = 800;

  // Button timings
  BUTTON_LONG_MS = 700;
  DEBOUNCE_MS    = 40;

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
  TUNING_SWR_MIN   = 1.0f;
  TUNING_SWR_MAX   = 5.0f;
  TUNING_FREQ_MIN  = 100;
  TUNING_FREQ_MAX  = 3000;
  TUNING_MIN_DELTA = 0.02f;

  // Speech
  SPEECH_PAUSE_MS  = 200;
  speechLevelIndex = 2;
  SPEECH_VOLUME    = SPEECH_LEVELS[speechLevelIndex];

  // Buzzer
  buzzerLevelIndex = 3;
  BUZZER_DUTY      = BUZZER_LEVELS[buzzerLevelIndex];

  // Calibration defaults (dummy until real data is stored)
  P_MAX_3V2       = 100.0f;
  calibrationValid = false;
  for (int i = 0; i < CAL_POINTS; i++) {
    calP[i] = (i + 1) * 10.0f;        // 10W, 20W, ..., 100W
    float U = (float)(i + 1) * (3.2f / (float)CAL_POINTS);
    calU[i] = U;
  }

  morseBpm = DEFAULT_MORSE_BPM;
  updateMorseTiming();
}

// --------------------------------------------------------
// Calibration save/load
// --------------------------------------------------------

void saveCalibrationToNVS() {
  prefs.begin("swrCal", false);
  prefs.putBool("valid", calibrationValid);
  prefs.putFloat("Pmax", P_MAX_3V2);
  for (int i = 0; i < CAL_POINTS; i++) {
    String keyP = "P" + String(i);
    String keyU = "U" + String(i);
    prefs.putFloat(keyP.c_str(), calP[i]);
    prefs.putFloat(keyU.c_str(), calU[i]);
  }
  prefs.end();
  Serial.println("Calibration saved to NVS.");
}

void loadCalibrationFromNVS() {
  prefs.begin("swrCal", true);
  bool valid = prefs.getBool("valid", false);
  if (!valid) {
    prefs.end();
    Serial.println("No valid calibration in NVS (using fallback).");
    calibrationValid = false;
    return;
  }

  P_MAX_3V2 = prefs.getFloat("Pmax", P_MAX_3V2);
  for (int i = 0; i < CAL_POINTS; i++) {
    String keyP = "P" + String(i);
    String keyU = "U" + String(i);
    calP[i] = prefs.getFloat(keyP.c_str(), calP[i]);
    calU[i] = prefs.getFloat(keyU.c_str(), calU[i]);
  }
  prefs.end();

  calibrationValid = true;
  Serial.println("Calibration loaded from NVS.");
}

// --------------------------------------------------------
// Webserver pages (HTML)
// --------------------------------------------------------

String buildHtmlPage() {
  String page =
    "<!DOCTYPE html><html lang='en'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>SWR Meter S3 V4.4</title>"
    "<style>"
    "body{font-family:sans-serif;padding:1.5rem;background:#111;color:#eee;}"
    "h1{font-size:1.6rem;margin-bottom:0.5rem;}"
    ".box{border:1px solid #555;border-radius:0.5rem;padding:1rem;margin-top:1rem;}"
    ".label{color:#aaa;font-size:0.9rem;}"
    ".value{font-size:1.8rem;margin:0.3rem 0;}"
    "a.btn{display:inline-block;padding:0.3rem 0.7rem;margin-top:0.7rem;"
    "border:1px solid #555;border-radius:0.4rem;color:#eee;text-decoration:none;margin-right:0.5rem;}"
    "</style>"
    "</head><body>"
    "<h1>SWR Meter (ESP32-S3 V4.4, PROGMEM voice)</h1>"
    "<p>Last measured values:</p>"
    "<div class='box'>"
    "<div class='label'>Output power</div>"
    "<div class='value' id='powerVal'>";
  page += String(g_powerW, 1);
  page += " W</div><div class='label'>SWR</div><div class='value' id='swrVal'>";
  page += String(g_swr, 2);
  page +=
    "</div></div>"
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
        "document.getElementById('swrVal').textContent=d.swr.toFixed(2);"
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
    "fieldset{border:1px solid #555;margin-bottom:1rem;padding:0.7rem;border-radius:0.5rem;}"
    "legend{padding:0 0.3rem;}"
    "label{display:block;margin-top:0.3rem;font-size:0.9rem;}"
    "input{width:100%;max-width:10rem;padding:0.2rem;margin-top:0.1rem;background:#222;color:#eee;border:1px solid #555;border-radius:0.3rem;}"
    "button{font-size:1.0rem;padding:0.4rem 0.8rem;margin-top:0.8rem;}"
    "a.btn{display:inline-block;padding:0.3rem 0.7rem;border:1px solid #555;border-radius:0.4rem;color:#eee;text-decoration:none;margin-right:0.5rem;}"
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
  page += "<fieldset><legend>Measurement</legend>";
  page += "<label>P_MIN_ACTIVE [W]<br><input name='P_MIN_ACTIVE' value='" + String(P_MIN_ACTIVE, 2) + "'></label>";
  page += "</fieldset>";

  // Speech
  page += "<fieldset><legend>Speech output</legend>";
  page += "<label>SPEECH_PAUSE_MS [ms]<br><input name='SPEECH_PAUSE_MS' value='" + String(SPEECH_PAUSE_MS) + "'></label>";
  page += "<label>Current SPEECH_VOLUME factor (info, changed by T4/T3 in 5 steps)<br>"
          "<input disabled value='" + String(SPEECH_VOLUME, 2) + "'></label>";
  page += "</fieldset>";

  // Buzzer
  page += "<fieldset><legend>Buzzer</legend>";
  page += "<label>Current BUZZER_DUTY (info, changed by T3 in 5 steps)<br>"
          "<input disabled value='" + String(BUZZER_DUTY) + "'></label>";
  page += "</fieldset>";

  // Morse
  page += "<fieldset><legend>Morse</legend>";
  page += "<label>DEFAULT_MORSE_BPM<br><input name='DEFAULT_MORSE_BPM' value='" + String(DEFAULT_MORSE_BPM) + "'></label>";
  page += "<label>MIN_MORSE_BPM<br><input name='MIN_MORSE_BPM' value='" + String(MIN_MORSE_BPM) + "'></label>";
  page += "<label>MAX_MORSE_BPM<br><input name='MAX_MORSE_BPM' value='" + String(MAX_MORSE_BPM) + "'></label>";
  page += "<label>STEP_MORSE_BPM<br><input name='STEP_MORSE_BPM' value='" + String(STEP_MORSE_BPM) + "'></label>";
  page += "<label>MORSE_TONE_FREQ [Hz]<br><input name='MORSE_TONE_FREQ' value='" + String(MORSE_TONE_FREQ) + "'></label>";
  page += "</fieldset>";

  // Button timings
  page += "<fieldset><legend>Button timings</legend>";
  page += "<label>BUTTON_LONG_MS [ms]<br><input name='BUTTON_LONG_MS' value='" + String(BUTTON_LONG_MS) + "'></label>";
  page += "<label>DEBOUNCE_MS [ms]<br><input name='DEBOUNCE_MS' value='" + String(DEBOUNCE_MS) + "'></label>";
  page += "</fieldset>";

  // Button ladder mid voltages
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
    "table{border-collapse:collapse;width:100%;max-width:20rem;}"
    "th,td{border:1px solid #555;padding:0.3rem;font-size:0.9rem;text-align:center;}"
    "input{width:100%;box-sizing:border-box;padding:0.2rem;background:#222;color:#eee;border:1px solid #555;border-radius:0.3rem;font-size:0.85rem;}"
    "fieldset{border:1px solid #555;margin-bottom:1rem;padding:0.7rem;border-radius:0.5rem;}"
    "legend{padding:0 0.3rem;}"
    "a.btn{display:inline-block;padding:0.3rem 0.7rem;border:1px solid #555;border-radius:0.4rem;color:#eee;text-decoration:none;margin-right:0.5rem;}"
    "button{font-size:1.0rem;padding:0.4rem 0.8rem;margin-top:0.8rem;}"
    ".msg{margin-bottom:0.7rem;color:#7f7;}"
    "</style>"
    "</head><body>"
    "<h1>SWR Meter Calibration</h1>"
    "<p><a class='btn' href='/'>Back to status</a>"
    "<a class='btn' href='/config'>Configuration</a></p>";

  if (msg.length() > 0) {
    page += "<div class='msg'>" + msg + "</div>";
  }

  page +=
    "<p>Procedure:</p>"
    "<ol>"
    "<li>Connect the directional coupler and apply a known RF power.</li>"
    "<li>Measure the true power P with an external wattmeter.</li>"
    "<li>Read the ADC/voltage at the FWD input.</li>"
    "<li>Fill 10 points across the usable power range.</li>"
    "<li>Click save. The calibration is then used in normal operation.</li>"
    "</ol>";

  page += "<form method='POST' action='/saveCal'>";

  page += "<fieldset><legend>General</legend>";
  page += "<label>P_MAX_3V2 (max power at ~3.2V, info only)<br>"
          "<input name='P_MAX_3V2' value='" + String(P_MAX_3V2, 1) + "'></label>";
  page += "</fieldset>";

  page += "<fieldset><legend>Calibration points</legend>";
  page += "<table><tr><th>#</th><th>P [W]</th><th>U_fwd [V]</th></tr>";

  for (int i = 0; i < CAL_POINTS; i++) {
    page += "<tr><td>";
    page += String(i + 1);
    page += "</td><td><input name='P";
    page += String(i);
    page += "' value='";
    page += String(calP[i], 2);
    page += "'></td><td><input name='U";
    page += String(i);
    page += "' value='";
    page += String(calU[i], 3);
    page += "'></td></tr>";
  }

  page += "</table>";
  page += "</fieldset>";

  page += "<button type='submit'>Save calibration</button>";
  page += "</form>";

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
      var = (unsigned long) server.arg(name).toInt();
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
  getInt("MIN_MORSE_BPM",     MIN_MORSE_BPM);
  getInt("MAX_MORSE_BPM",     MAX_MORSE_BPM);
  getInt("STEP_MORSE_BPM",    STEP_MORSE_BPM);
  getInt("MORSE_TONE_FREQ",   MORSE_TONE_FREQ);

  // Button timings
  getULong("BUTTON_LONG_MS",  BUTTON_LONG_MS);
  getULong("DEBOUNCE_MS",     DEBOUNCE_MS);

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
  getFloat("TUNING_SWR_MIN",   TUNING_SWR_MIN);
  getFloat("TUNING_SWR_MAX",   TUNING_SWR_MAX);
  getInt("TUNING_FREQ_MIN",    TUNING_FREQ_MIN);
  getInt("TUNING_FREQ_MAX",    TUNING_FREQ_MAX);
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

  for (int i = 0; i < CAL_POINTS; i++) {
    String nameP = "P" + String(i);
    String nameU = "U" + String(i);
    getFloat(nameP, calP[i]);
    getFloat(nameU, calU[i]);
  }

  // simple plausibility check
  calibrationValid = true;
  int nonZero = 0;
  for (int i = 0; i < CAL_POINTS; i++) {
    if (calP[i] > 0.0f && calU[i] > 0.0f) nonZero++;
  }
  if (nonZero < 3) {
    calibrationValid = false;
  }

  saveCalibrationToNVS();

  String msg;
  if (calibrationValid) {
    msg = "Calibration saved and activated.";
  } else {
    msg = "Less than 3 valid points entered – calibration disabled (fallback active).";
  }

  server.send(200, "text/html", buildCalPage(msg));
}

void handleFactoryReset() {
  resetConfigToDefaults();
  calibrationValid = false;
  saveCalibrationToNVS();   // marks calibration as invalid too
  server.send(200, "text/html", buildConfigPage("Factory defaults loaded (including calibration)."));
}

void handleNotFound() {
  server.send(404, "text/plain", "404 - Not found");
}

// --------------------------------------------------------
// Button event handler
// --------------------------------------------------------

void handleButtonEvent(uint8_t button, bool isShort) {
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
        powerInt      = 0;
        swrTimes10Int = 0;
      } else {
        float P_round = P_fwd + 0.5f;
        if (P_round < 0.0f)  P_round = 0.0f;
        if (P_round > 99.0f) P_round = 99.0f;
        powerInt = (int)P_round;

        float swrTimes10 = swr * 10.0f + 0.5f;
        if (swrTimes10 < 0.0f)  swrTimes10 = 0.0f;
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

    // short confirmation beep (buzzer)
    ledcWriteTone(PIN_BUZZER, MORSE_TONE_FREQ);
    ledcWrite(PIN_BUZZER, BUZZER_DUTY);
    delay(80);
    ledcWrite(PIN_BUZZER, 0);
    delay(40);
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
  Serial.println("=== ESP32-S3 SWR Meter V4.4: PROGMEM voice, calibration, no SD card ===");

  // Load defaults
  resetConfigToDefaults();

  // Try load calibration from NVS (overwrites default calibration if valid)
  loadCalibrationFromNVS();

  // ADC setup
  analogReadResolution(12);
  pinMode(PIN_FWD, INPUT);
  pinMode(PIN_REV, INPUT);
  pinMode(PIN_PTT, INPUT);

  // Buzzer via LEDC
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  bool buzzerOk = ledcAttach(PIN_BUZZER, 2000, BUZZER_PWM_BITS); // 2 kHz, 10-bit
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
  bool apOk = WiFi.softAP(AP_SSID);
  if (!apOk) {
    Serial.println("ERROR: Access point could not be started!");
  } else {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP started, SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP IP: ");
    Serial.println(ip);
  }

  // Webserver routes
  server.on("/",             handleRoot);
  server.on("/data",         handleData);
  server.on("/config",       handleConfig);
  server.on("/saveConfig",   HTTP_POST, handleSaveConfig);
  server.on("/factoryReset", HTTP_POST, handleFactoryReset);
  server.on("/cal",          handleCal);
  server.on("/saveCal",      HTTP_POST, handleSaveCal);
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
    pressedButton  = currentButton;
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

  delay(10);
}


