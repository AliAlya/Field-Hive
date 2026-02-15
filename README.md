# Field-Hive

Field-Hive is a prototype health and rescue-support monitoring system that combines:
- ESP32 sensor nodes
- Arduino-compatible microcontroller integration
- Camera-based vitals inference API stream
- Wireless telemetry (ESP-NOW + UDP)

The project tracks heart activity and motion, then shares data for remote monitoring during emergency or rescue scenarios.

## What It Monitors
- Heart activity (pulse/BPM estimate)
- Motion/activity (MPU6050 accelerometer magnitude/delta)
- Camera-derived vitals (heart rate and breathing rate)

## Repository Structure
- `MakeUofT.ino`: standalone ESP32 pulse/BPM processing sketch with auto-refresh logic and serial output.
- `accelerometer/accelerometer.ino`: ESP32 combined sketch for:
  - MPU6050 activity sampling
  - BPM presence logic
  - ESP-NOW encrypted telemetry
  - OLED status display
- `Peripherals&Communication/ESP_Comm/ESP_Comm.ino`: ESP32 telemetry bridge that sends `ax:ay:az:mag:delta:bpm` via ESP-NOW.
- `Peripherals&Communication/sending_info_arduino_Q/sending_info_arduino_Q.ino`: accelerometer-only ESP-NOW sender example.
- `Peripherals&Communication/no-bpm-0-output/no-bpm-0-output.ino`: pulse presence detector that prints BPM or `0`.
- `hello_vitals.cpp`: camera API client (SmartSpectra/Presage stack) that reads RTSP input and sends vitals over UDP.

## High-Level Data Flow
1. ESP32 samples pulse and activity sensors.
2. ESP32 packages telemetry and transmits via ESP-NOW.
3. Camera pipeline (`hello_vitals.cpp`) computes HR/RR from video.
4. Camera vitals are sent as UDP JSON packets to a receiver (for dashboard/alerts).
5. Combined streams can be used for responder awareness and rescue decision support.

## Hardware Used
- ESP32 dev board(s)
- Arduino Uno Q (as used in your setup)
- Pulse sensor (analog output to ESP32 ADC pin)
- MPU6050 accelerometer/IMU (I2C)
- SSD1306 128x64 OLED (I2C, optional display)
- Camera source providing RTSP stream
- Networked receiver (Raspberry Pi / laptop) for UDP packets

## Default Pin/Bus Configuration (from code)
- Pulse input: `GPIO34`
- I2C SDA: `GPIO21`
- I2C SCL: `GPIO22`
- OLED address in code: `0x3C` (set with `0x3C << 1` in U8g2)

## Dependencies

### Arduino / ESP32
- ESP32 board support package (Arduino IDE)
- Libraries used in sketches:
  - `Wire`
  - `WiFi`
  - `esp_now`
  - `U8g2lib` (for OLED sketch)

### C++ Camera Vitals App (`hello_vitals.cpp`)
Requires the Presage SmartSpectra/Physiology SDK stack shown in includes:
- `smartspectra/...`
- `physiology/...`
- `glog`
- standard Linux UDP headers (`sys/socket.h`, `arpa/inet.h`, `unistd.h`)

## Configuration Checklist
Before running, update these values in code:
- ESP-NOW peer MAC addresses (`peerMAC[]`)
- ESP-NOW keys (`PMK_KEY`, `LMK_KEY`) when encryption is enabled
- UDP target IP/port in `hello_vitals.cpp` (currently set to `172.17.214.32:5005`)
- RTSP source URL in `hello_vitals.cpp`
- `SMARTSPECTRA_API_KEY` environment variable (or pass as CLI argument)

## Quick Start

### 1) Flash ESP32 sketch(es)
- Open the desired `.ino` in Arduino IDE.
- Select the correct ESP32 board and serial port.
- Install missing libraries.
- Upload and open Serial Monitor (`115200`).

### 2) Run camera vitals sender
Build `hello_vitals.cpp` in your SDK environment, then run:

```bash
export SMARTSPECTRA_API_KEY="<your_api_key>"
./hello_vitals
```

or:

```bash
./hello_vitals <your_api_key>
```

The app will reconnect automatically if the stream drops.

## Packet Formats
- ESP-NOW sensor packet:
  - `ax:ay:az:mag:delta:bpm`
- UDP camera vitals packet:
  - `{ "hr": <pulse>, "br": <breathing> }`
- UDP boot packet:
  - `{ "boot": 1 }`

## Intended Use
This repository is a prototype for field monitoring and rescue support. It is not a certified medical device and should not be used as a sole source for clinical decisions.
