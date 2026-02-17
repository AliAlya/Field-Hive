import json
from datetime import datetime
from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.vibration_anomaly_detection import VibrationAnomalyDetection

logger = Logger("vibration-detector")

vibration_detection = VibrationAnomalyDetection(anomaly_detection_threshold=1.0)

ui = WebUI()

def on_override_th(value: float):
    logger.info(f"Setting new anomaly threshold: {value}")
    vibration_detection.anomaly_detection_threshold = float(value)

ui.on_message("override_th", lambda sid, threshold: on_override_th(threshold))

def get_status(anomaly_detected: bool):
    return {
        "anomaly": anomaly_detected,
        "status_text": "Anomaly detected!" if anomaly_detected else "No anomaly"
    }

# --- minimal "trouble" detection config ---
def classify_trouble(move_ms2: float, hr: int) -> str | None:
    if hr < 0:
        return None

    if move_ms2 >= 0.09 and hr <= 50:
        return "problem with 02 HR"
    if move_ms2 <= 0.09 and hr >= 80:
        return "support 02"
    return None

def on_detected_anomaly(anomaly_score: float, classification: dict):
    logger.warning(f"ANOMALY score={anomaly_score:.3f} class={classification}")

    anomaly_payload = {
        "score": float(anomaly_score),
        "classification": classification,
        "timestamp": datetime.now().isoformat()
    }
    ui.send_message("anomaly_detected", json.dumps(anomaly_payload))
    ui.send_message("status_update", get_status(True))

vibration_detection.on_anomaly(on_detected_anomaly)

def record_sensor_movement(x: float, y: float, z: float, mag: float, delta: float, heart_rate: int):
    x_ms2 = x * 9.81
    y_ms2 = y * 9.81
    z_ms2 = z * 9.81

    ui.send_message("sample", {
        "x": x_ms2, "y": y_ms2, "z": z_ms2,
        "mag": float(mag), "delta": float(delta), "hr": int(heart_rate),
        "ts": datetime.now().isoformat()
    })

    vibration_detection.accumulate_samples((x_ms2, y_ms2, z_ms2))

    move_ms2 = float(delta)
    label = classify_trouble(move_ms2, int(heart_rate))
    if label is not None:
        ui.send_message("trouble_detected", json.dumps({
            "label": label,
            "hr": int(heart_rate),
            "move": float(move_ms2),
            "timestamp": datetime.now().isoformat()
        }))

Bridge.provide("record_sensor_movement", record_sensor_movement)
App.run()