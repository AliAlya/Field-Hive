#include <Wire.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

static const uint8_t MPU_ADDR = 0x68;
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

typedef struct {
  char message[64];   // bigger buffer for numbers
} DataPacket;

DataPacket txData;
DataPacket rxData;

// ===== PUT RECEIVER MAC HERE =====
uint8_t peerMAC[] = {0xB0, 0xCB, 0xD8, 0xE6, 0x1A, 0x50};

// ---------- MPU FUNCTIONS ----------
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

  ax = rawAx / 16384.0f;
  ay = rawAy / 16384.0f;
  az = rawAz / 16384.0f;
  return true;
}

// ---------- ESP-NOW CALLBACKS ----------
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "TX OK" : "TX FAIL");
}

void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  int copyLen = min((int)sizeof(rxData), len);
  memcpy(&rxData, incomingData, copyLen);

  Serial.print("RX: ");
  Serial.println(rxData.message);
}

bool addPeer(const uint8_t *mac) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(mac)) return true;
  return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(300);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  mpuWriteByte(0x6B, 0x00); // wake MPU
  mpuWriteByte(0x1C, 0x00); // ±2g

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);

  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) {}
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  if (!addPeer(peerMAC)) {
    Serial.println("Peer add failed");
    while (true) {}
  }

  Serial.println("Streaming MPU data...");
}

// ---------- LOOP ----------
void loop() {
  float ax, ay, az;

  if (!readAccelG(ax, ay, az)) {
    Serial.println("MPU read fail");
    delay(100);
    return;
  }

  float mag   = sqrtf(ax*ax + ay*ay + az*az);
  float delta = fabsf(mag - 1.0f);

  // scale ×1000 → integers
  int ax_i = (int)(ax * 1000);
  int ay_i = (int)(ay * 1000);
  int az_i = (int)(az * 1000);
  int mag_i = (int)(mag * 1000);
  int del_i = (int)(delta * 1000);

  // build colon-separated payload
  snprintf(txData.message, sizeof(txData.message),
           "%d:%d:%d:%d:%d",
           ax_i, ay_i, az_i, mag_i, del_i);

  // send
  esp_now_send(peerMAC, (uint8_t*)&txData, sizeof(txData));

  // also print locally for debugging
  Serial.println(txData.message);

  delay(50);  // ~20 Hz stream
}
