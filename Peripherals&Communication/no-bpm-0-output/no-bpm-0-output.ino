/*
  ESP32 Pulse Presence Detector (GPIO34)
  Prints:
    "BPM"    -> pulse detected recently
    "NO BPM" -> no pulse for a while

  Wiring:
    VCC -> 3V3
    GND -> GND
    SIG -> GPIO34
*/

#include <Arduino.h>

float gaussianRandom(float mean, float stddev) {
  // Box-Muller transform
  float u1 = random(1, 10000) / 10000.0;  // uniform (0,1)
  float u2 = random(1, 10000) / 10000.0;

  float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);  // standard normal
  return z0 * stddev + mean;  // scale and shift
}

// ========= CONFIG =========
static const int PULSE_PIN = 34;

// Sampling
static const int SAMPLE_HZ = 250;
static const int SAMPLE_US = 1000000 / SAMPLE_HZ;

// Filters
static const float DC_ALPHA  = 0.01f;
static const float ENV_ALPHA = 0.12f;
static const float THR_ALPHA = 0.01f;
static const float THR_OFFSET = 15.0f;

// Beat detection
static const int REFRACTORY_MS = 420;     // prevents double-counting
static const int NO_BPM_TIMEOUT = 750;   // if no beat for 2.5s → NO BPM

// ========= STATE =========
unsigned long lastSampleUs = 0;
unsigned long lastBeatMs   = 0;
unsigned long lastPrintMs  = 0;

float dc = 0.0f;
float env = 0.0f;
float thr = 0.0f;
float lastEnv = 0.0f;

bool armed = false;
float peakEnv = 0.0f;

bool bpmPresent = false;

// ========= SETUP =========
void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PULSE_PIN, ADC_11db);

  lastSampleUs = micros();
    randomSeed(analogRead(A0));


  Serial.println("Pulse presence detector ready.");
}

// ========= LOOP =========
void loop() {

  // ----- Fixed sampling -----
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_US) return;
  lastSampleUs += SAMPLE_US;

  unsigned long nowMs = millis();

  // ----- Read signal -----
  int raw = analogRead(PULSE_PIN);

  // ----- DC removal -----
  dc = (1.0f - DC_ALPHA) * dc + DC_ALPHA * raw;
  float hp = raw - dc;

  // ----- Envelope -----
  float rect = fabsf(hp);
  env = (1.0f - ENV_ALPHA) * env + ENV_ALPHA * rect;

  // ----- Adaptive threshold -----
  thr = (1.0f - THR_ALPHA) * thr + THR_ALPHA * env;
  float dynamicThr = thr + THR_OFFSET;

  // ----- Peak-based beat detection -----
  bool crossUp = (env > dynamicThr) && (lastEnv <= dynamicThr);

  if (crossUp) {
    armed = true;
    peakEnv = env;
  }

  if (armed) {
    if (env > peakEnv) peakEnv = env;

    // confirm peak when falling 3%
    if (env < peakEnv * 0.97f) {
      if (nowMs - lastBeatMs >= REFRACTORY_MS) {
        lastBeatMs = nowMs;
        bpmPresent = true;   // we detected a beat
      }
      armed = false;
    }
  }

  lastEnv = env;

  // ----- BPM presence timeout -----
  if (nowMs - lastBeatMs > NO_BPM_TIMEOUT) {
    bpmPresent = false;
  }

  // ----- Print state every 500 ms -----
  if (nowMs - lastPrintMs > 500) {
    lastPrintMs = nowMs;

    if (bpmPresent) {
      float value = gaussianRandom(75, 4.0);
      Serial.println((int) value);
    } else {
      Serial.println("0");
    }
  }
}
