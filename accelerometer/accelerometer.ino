#include <Wire.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <U8g2lib.h>

// ===================== OLED (U8g2) =====================
// SSD1306 128x64 over I2C (hardware I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ===================== MPU6050 CONFIG =====================
static const uint8_t MPU_ADDR = 0x68;
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

// Movement threshold using delta (same guidance as earlier)
// You can tune this later.
static const float MOVE_THRESHOLD = 0.05f;

// ===================== ESP-NOW ENCRYPTION KEYS =====================
static const uint8_t PMK_KEY[16] = {
  0x1A,0x2B,0x3C,0x4D,0x5E,0x6F,0x70,0x81,
  0x92,0xA3,0xB4,0xC5,0xD6,0xE7,0xF8,0x09
};

static const uint8_t LMK_KEY[16] = {
  0x55,0x44,0x33,0x22,0x11,0x00,0xAA,0xBB,
  0xCC,0xDD,0xEE,0xFF,0x10,0x20,0x30,0x40
};

// ===================== ESP-NOW PACKET =====================
typedef struct {
  char message[80];   // enough for 6 ints + colons
} DataPacket;

DataPacket txData;
DataPacket rxData;

// Put the OTHER ESP32's MAC here (receiver)
uint8_t peerMAC[] = {0xB0, 0xCB, 0xD8, 0xE6, 0x1A, 0x50}; // Arduino B0:CB:D8:E6:1A:50

// ===================== BPM CODE (UNCHANGED LOGIC) =====================
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

// We store latest BPM int here (integration only)
volatile int latestBpmInt = 0;

// ESP32 sometimes doesn't define A0 depending on board variant.
// This keeps YOUR randomSeed(analogRead(A0)) line intact.
#ifndef A0
#define A0 36
#endif

// Call this often (non-blocking); logic inside is unchanged
void updateBpmLogic() {
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
  // (We do NOT print; we store exactly what you would've printed.)
  if (nowMs - lastPrintMs > 500) {
    lastPrintMs = nowMs;

    if (bpmPresent) {
      float value = gaussianRandom(75, 4.0);
      latestBpmInt = (int)value;
    } else {
      latestBpmInt = 0;
    }
  }
}

// ===================== MPU6050 HELPERS =====================
void mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

bool mpuReadBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)MPU_ADDR, (int)len, (int)true);
  if (got != len) return false;

  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool readAccelG(float &ax, float &ay, float &az) {
  uint8_t d[6];
  if (!mpuReadBytes(0x3B, d, 6)) return false;

  int16_t rawAx = (int16_t)((d[0] << 8) | d[1]);
  int16_t rawAy = (int16_t)((d[2] << 8) | d[3]);
  int16_t rawAz = (int16_t)((d[4] << 8) | d[5]);

  // ±2g => 16384 LSB/g
  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;
  return true;
}

// ===================== ESP-NOW CALLBACKS =====================
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // keep minimal; OLED is the main UI
}

void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  int copyLen = min((int)sizeof(rxData), len);
  memcpy(&rxData, incomingData, copyLen);

  // optional debug
  Serial.print("RX: ");
  Serial.println(rxData.message);
}

bool addPeer(const uint8_t *mac) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = true;                 // enable encryption
  memcpy(peerInfo.lmk, LMK_KEY, 16);       // shared per-peer key

  if (esp_now_is_peer_exist(mac)) return true;
  return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

// ===================== OLED DRAW =====================
void drawOLED(int bpm_i, bool moving,
              int ax_i, int ay_i, int az_i,
              int mag_i, int del_i) {

  char line1[24];
  char line2[24];
  char line3[24];
  char line4[24];

  snprintf(line1, sizeof(line1), "BPM:%d  %s", bpm_i, moving ? "MOVE" : "STILL");
  snprintf(line2, sizeof(line2), "d:%d  mag:%d", del_i, mag_i);
  snprintf(line3, sizeof(line3), "ax:%d ay:%d", ax_i, ay_i);
  snprintf(line4, sizeof(line4), "az:%d", az_i);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.drawStr(0, 12, line1);
  u8g2.drawStr(0, 28, line2);
  u8g2.drawStr(0, 44, line3);
  u8g2.drawStr(0, 60, line4);

  u8g2.sendBuffer();
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  // --- Shared I2C bus (MPU + OLED) ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // OLED address: you said 0x78, U8g2 wants 8-bit address => 0x3C<<1
  u8g2.setI2CAddress(0x3C << 1);
  u8g2.begin();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 14, "Init...");
  u8g2.sendBuffer();

  // --- MPU init ---
  mpuWriteByte(0x6B, 0x00); // wake
  mpuWriteByte(0x1C, 0x00); // ±2g

  // --- BPM ADC setup (same as your code) ---
  analogReadResolution(12);
  analogSetPinAttenuation(PULSE_PIN, ADC_11db);
  lastSampleUs = micros();
  randomSeed(analogRead(A0));

  // --- ESP-NOW ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }
  esp_now_set_pmk(PMK_KEY);
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  if (!addPeer(peerMAC)) {
    Serial.println("Peer add failed");
    while (true) {}
  }

  Serial.println("Streaming ax:ay:az:mag:delta:bpm");
  u8g2.clearBuffer();
  u8g2.drawStr(0, 14, "Streaming...");
  u8g2.sendBuffer();
}

// ===================== LOOP =====================
void loop() {
  // Keep BPM sampler running at 250Hz (non-blocking)
  updateBpmLogic();

  // Send accel packet at ~20 Hz
  static unsigned long lastTxMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastTxMs < 50) return;
  lastTxMs = nowMs;

  float ax, ay, az;
  if (!readAccelG(ax, ay, az)) return;

  float mag   = sqrtf(ax*ax + ay*ay + az*az);
  float delta = fabsf(mag - 1.0f);

  bool moving = (delta > MOVE_THRESHOLD);

  // scale ×1000 for accel/mag/delta
  int ax_i  = (int)(ax * 1000);
  int ay_i  = (int)(ay * 1000);
  int az_i  = (int)(az * 1000);
  int mag_i = (int)(mag * 1000);
  int del_i = (int)(delta * 1000);

  int bpm_i = latestBpmInt; // NOT scaled

  // ax:ay:az:mag:delta:bpm  (your requested TX format)
  snprintf(txData.message, sizeof(txData.message),
           "%d:%d:%d:%d:%d:%d",
           ax_i, ay_i, az_i, mag_i, del_i, bpm_i);

  esp_now_send(peerMAC, (uint8_t*)&txData, sizeof(txData));

  // Optional debug
  // Serial.println(txData.message);

  // ===================== ONLY CHANGE: OLED UPDATE RATE + SMOOTHING =====================
  // Update OLED slower and smoother by taking a weighted average over incoming samples.
  // This does NOT change TX data, BPM logic, MPU logic, or anything else.
    // ===================== ONLY CHANGE: OLED UPDATE RATE + STABLE MOVEMENT =====================

  static unsigned long lastOledValsMs = 0;
  static unsigned long lastOledMoveMs = 0;
  static unsigned long moveStateChangeMs = 0;

  static const unsigned long OLED_VALS_PERIOD_MS = 250; // numbers refresh (unchanged)
  static const unsigned long OLED_MOVE_PERIOD_MS = 180; // slower movement refresh
  static const unsigned long MOVE_STABLE_MS      = 220; // must stay stable before flipping label

  static const float ALPHA = 0.15f; // smoothing weight
  static float sax = 0, say = 0, saz = 0, smag = 0, sdel = 0, sbpm = 0;

  // ---------- Smooth numeric display ----------
  sax  = (1.0f - ALPHA) * sax  + ALPHA * ax_i;
  say  = (1.0f - ALPHA) * say  + ALPHA * ay_i;
  saz  = (1.0f - ALPHA) * saz  + ALPHA * az_i;
  smag = (1.0f - ALPHA) * smag + ALPHA * mag_i;
  sdel = (1.0f - ALPHA) * sdel + ALPHA * del_i;
  sbpm = (1.0f - ALPHA) * sbpm + ALPHA * bpm_i;

  // ---------- Stable movement detection ----------
  static bool stableMoving = false;      // what OLED shows
  static bool lastRawMoving = false;     // previous raw state

  static const unsigned long MOVE_ON_STABLE_MS  = 10;   // how long raw must be MOVE before OLED shows MOVE
  static const unsigned long MOVE_OFF_STABLE_MS = 220;  // how long raw must be STILL before OLED shows STILL

  if (moving != lastRawMoving) {
    moveStateChangeMs = nowMs;   // start timer for this transition
    lastRawMoving = moving;
  }

  unsigned long required = moving ? MOVE_ON_STABLE_MS : MOVE_OFF_STABLE_MS;

  if ((nowMs - moveStateChangeMs) >= required) {
    stableMoving = moving;
  }
  // ---------- Refresh conditions ----------
  bool doValsUpdate = (nowMs - lastOledValsMs >= OLED_VALS_PERIOD_MS);
  bool doMoveUpdate = (nowMs - lastOledMoveMs >= OLED_MOVE_PERIOD_MS);

  if (doValsUpdate || doMoveUpdate) {
    if (doValsUpdate) lastOledValsMs = nowMs;
    if (doMoveUpdate) lastOledMoveMs = nowMs;

    drawOLED((int)sbpm, stableMoving,
             (int)sax, (int)say, (int)saz,
             (int)smag, (int)sdel);
  }

}
