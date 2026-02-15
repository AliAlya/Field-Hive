#include <Arduino.h>

// ===================== USER SETTINGS =====================
static const int HEART_PIN = 34;
static const int SAMPLE_MS = 20;       // 50 Hz
static const int REFRACTORY_MS = 400;  // prevents double count
static const int BPM_OFFSET = -30;     // your calibration correction

// Optional "restart-style" button (wired to GND, uses internal pullup)
static const int REFRESH_BTN_PIN = 25;  // pick any normal GPIO (NOT 34-39)
static const bool USE_REFRESH_BUTTON = true;

// Auto-refresh behavior
static const int STALE_REFRESH_MS = 4000;  // if no valid beat for 4s, refresh estimator state
static const int FLAT_REFRESH_MS = 2500;   // if signal span too flat for 2.5s, refresh
// =========================================================

// Baseline and detection
int baseline = 2000;
int thresholdOffset = 120;

uint32_t lastBeatMs = 0;   // time of last accepted beat
uint32_t lastValidMs = 0;  // time of last valid peak (beat accepted)
int bpm = 0;               // raw bpm estimate (uncorrected)

// smoothing (moving average 4)
int s0 = 0, s1 = 0, s2 = 0, s3 = 0;
int prevSmooth = 0;
int prevSlope = 0;

// dynamic scaling for waveform
int lowTrack = 4095, highTrack = 0;

// For "flat signal" timing
uint32_t flatSinceMs = 0;

// ---- Soft refresh: reset estimator state but KEEP bpm ----
void softRefreshEstimator(bool keepBpm = true) {
  int keep = bpm;

  baseline = analogRead(HEART_PIN);  // re-seed baseline near current signal
  s0 = s1 = s2 = s3 = baseline;
  prevSmooth = baseline;
  prevSlope = 0;

  lowTrack = 4095;
  highTrack = 0;

  lastBeatMs = 0;
  lastValidMs = millis();  // prevent immediate stale refresh loop

  if (keepBpm) bpm = keep;
  else bpm = 0;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(HEART_PIN, ADC_11db);

  if (USE_REFRESH_BUTTON) {
    pinMode(REFRESH_BTN_PIN, INPUT_PULLUP);  // button to GND
  }

  // seed baseline/buffers once
  delay(50);
  softRefreshEstimator(true);
}

void loop() {
  uint32_t now = millis();

  // ---- Manual refresh via button (press) ----
  if (USE_REFRESH_BUTTON) {
    static bool lastBtn = true;
    bool btn = digitalRead(REFRESH_BTN_PIN);
    if (lastBtn && !btn) {         // falling edge (pressed)
      softRefreshEstimator(true);  // keep BPM on screen
    }
    lastBtn = btn;
  }

  // ---- Manual refresh via Serial: send 'r' ----
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      softRefreshEstimator(true);  // keep BPM on screen
    }
  }

  // 1) Read raw
  int raw = analogRead(HEART_PIN);

  // 2) Baseline tracking (slow drift)
  baseline = (baseline * 31 + raw) / 32;

  // 3) Smooth (moving average of 4)
  s3 = s2;
  s2 = s1;
  s1 = s0;
  s0 = raw;
  int smooth = (s0 + s1 + s2 + s3) / 4;

  // 4) Peak detect by slope sign change (+ to -)
  int slope = smooth - prevSmooth;
  bool peak = (prevSlope > 0 && slope <= 0);
  prevSmooth = smooth;
  prevSlope = slope;

  // 5) Simple threshold
  bool validPeak = peak && (smooth > baseline + thresholdOffset);

  // 6) Update BPM on valid peak
  if (validPeak) {
    if (lastBeatMs != 0) {
      uint32_t ibi = now - lastBeatMs;
      if (ibi > 300 && ibi < 2000) {
        int newBpm = 60000 / (int)ibi;
        bpm = (bpm == 0) ? newBpm : (bpm * 3 + newBpm) / 4;
        lastValidMs = now;
      }
    } else {
      // first peak seen after refresh
      lastValidMs = now;
    }
    lastBeatMs = now;
  }

  // 7) Dynamic scaling window (for waveform plot)
  if (raw < lowTrack) lowTrack = raw;
  if (raw > highTrack) highTrack = raw;

  int span = highTrack - lowTrack;
  if (span < 30) span = 30;

  int waveScaled = (raw - lowTrack) * 1000 / span;
  waveScaled = constrain(waveScaled, 0, 1000);

  // ---- Auto soft-refresh to prevent “locking” ----
  // (A) No valid beat for a while -> refresh estimator state, keep bpm shown
  if ((now - lastValidMs) > (uint32_t)STALE_REFRESH_MS) {
    softRefreshEstimator(true);
  }

  // (B) Signal too flat for a while -> refresh estimator state, keep bpm shown
  int spanActual = highTrack - lowTrack;
  bool flat = (spanActual < 12);  // tune 8..20 depending on your sensor
  if (flat) {
    if (flatSinceMs == 0) flatSinceMs = now;
    if ((now - flatSinceMs) > (uint32_t)FLAT_REFRESH_MS) {
      softRefreshEstimator(true);
      flatSinceMs = 0;
    }
  } else {
    flatSinceMs = 0;
  }

  // 8) Apply calibration offset (keep >=0)
  int bpmCorrected = bpm + BPM_OFFSET;
  if (bpmCorrected < 0) bpmCorrected = 0;

  // Output: waveform + corrected BPM
  Serial.print(waveScaled);
  Serial.print('\t');
  Serial.println(bpmCorrected);

  delay(SAMPLE_MS);
}
