#include <Arduino_RouterBridge.h>
#include <Arduino_Modulino.h>

static float f_acc_x, f_acc_y, f_acc_z, f_mag, f_delta;
static int acc_x, acc_y, acc_z, mag, delta, heart_rate;

static char buffer[128];

static unsigned long previousMillis = 0;
static const unsigned long intervalMs = 16; // ~62.5 Hz

void setup() {
  // USB -> Arduino Serial Monitor
  Serial.begin(115200);
  delay(200);
  Serial.println("UNO Q booting...");

  // UART on pins 0/1 -> ESP32
  Serial1.begin(115200);
  Serial1.setTimeout(5);
  Serial.println("Serial1 (pins 0/1) ready");
}

void loop() {
  // If this errors, remove it.
  Bridge.update();

  while (Serial1.available()) {
    int len = Serial1.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';
    if (len > 0 && buffer[len - 1] == '\r') buffer[len - 1] = '\0';

    // Debug: print every received line to USB serial monitor
    Serial.print("RAW: ");
    Serial.println(buffer);

    if (strncmp(buffer, "DATA: ", 6) != 0) continue;

    unsigned long now = millis();
    if (now - previousMillis < intervalMs) continue;
    previousMillis = now;

    char *msg = buffer + 6;

    int parsed = sscanf(msg, "%d:%d:%d:%d:%d:%d",
                        &acc_x, &acc_y, &acc_z, &mag, &delta, &heart_rate);
    if (parsed != 6) {
      Serial.println("Parse failed");
      continue;
    }

    f_acc_x = acc_x / 1000.0f;
    f_acc_y = acc_y / 1000.0f;
    f_acc_z = acc_z / 1000.0f;
    f_mag   = mag   / 1000.0f;
    f_delta = delta / 1000.0f;

    Bridge.notify("record_sensor_movement",
                  f_acc_x, f_acc_y, f_acc_z,
                  f_mag, f_delta,
                  heart_rate);
  }

  delay(1);
}