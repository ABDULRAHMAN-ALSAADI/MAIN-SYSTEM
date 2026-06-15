#!/usr/bin/env python3
"""
Conveyor CV Bridge v5 — SINGLE CAMERA PIPELINE

Correct architecture:
  ESP32-CAM must have only ONE stream client.

Bad old architecture:
  Browser dashboard opened http://ESP32CAM_IP:81/stream
  CV bridge also opened http://ESP32CAM_IP:81/stream
  ESP32-CAM closed one connection:
    ConnectionResetError 10054

Fixed architecture:
  CV bridge is the only client connected to ESP32-CAM.
  CV bridge republishes:
    raw stream       -> http://127.0.0.1:5060/raw.mjpg
    processed stream -> http://127.0.0.1:5060/stream.mjpg

Dashboard shows both streams from the bridge, not directly from the ESP32-CAM.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Generator

import cv2
import numpy as np
import requests
import paho.mqtt.client as mqtt
from flask import Flask, Response, jsonify


CONFIG_FILE = Path("conveyor_bridge_config.json")

DEFAULT_CONFIG = {
    "conveyor_camera_ip": "192.168.137.152",
    "camera_stream_port": 81,
    "servo_port": 82,

    "mqtt_broker_ip": "127.0.0.1",
    "mqtt_port": 1883,

    "min_area": 500,
    "send_delay_s": 0.50,

    "publish_stable_frames": 8,
    "publish_cooldown_s": 4.0,

    "enable_servo_command": True,
    "enable_mqtt_publish": True,

    "shape_name": "SQUARE",

    "color_to_recipe": {
        "RED": "RED_SQUARE_TO_SHELF_A",
        "GREEN": "GREEN_SQUARE_TO_SHELF_C",
        "BLUE": "BLUE_SQUARE_TO_SHELF_B"
    }
}


def load_config() -> dict[str, Any]:
    if not CONFIG_FILE.exists():
        CONFIG_FILE.write_text(json.dumps(DEFAULT_CONFIG, indent=2), encoding="utf-8")
        return dict(DEFAULT_CONFIG)

    try:
        user_cfg = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
    except Exception:
        user_cfg = {}

    cfg = dict(DEFAULT_CONFIG)
    for key, value in user_cfg.items():
        if isinstance(value, dict) and isinstance(cfg.get(key), dict):
            merged = dict(cfg[key])
            merged.update(value)
            cfg[key] = merged
        else:
            cfg[key] = value

    CONFIG_FILE.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    return cfg


CFG = load_config()

latest_lock = threading.RLock()
latest_raw_jpeg: bytes | None = None
latest_processed_jpeg: bytes | None = None

latest_status: dict[str, Any] = {
    "ok": False,
    "state": "BOOTING",
    "camera": "",
    "mqtt": "DISCONNECTED",
    "detectedColor": "NO COLOR",
    "detectedCode": "",
    "area": 0,
    "shape": "",
    "packageClass": "",
    "lastPublish": "",
    "lastServo": "",
    "frames": 0,
    "fps": 0.0,
    "reader": "single_client_requests_mjpeg",
    "rawStream": "http://127.0.0.1:5060/raw.mjpg",
    "processedStream": "http://127.0.0.1:5060/stream.mjpg",
}

running = True
mqtt_client: mqtt.Client | None = None
mqtt_connected = False


def clean_ip(ip: str) -> str:
    ip = str(ip or "").strip()
    ip = ip.replace("http://", "").replace("https://", "").strip("/")
    return ip


def camera_ip() -> str:
    return clean_ip(str(CFG["conveyor_camera_ip"]))


def camera_stream_url() -> str:
    return f"http://{camera_ip()}:{int(CFG['camera_stream_port'])}/stream"


def servo_url(code: str) -> str:
    return f"http://{camera_ip()}:{int(CFG['servo_port'])}/servo?color={code}"


def make_mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id="conveyor_cv_bridge_single_pipeline")
    except Exception:
        return mqtt.Client(client_id="conveyor_cv_bridge_single_pipeline")


def mqtt_on_connect(client, userdata, flags, rc):
    global mqtt_connected
    mqtt_connected = (rc == 0)
    with latest_lock:
        latest_status["mqtt"] = "CONNECTED" if mqtt_connected else f"FAILED rc={rc}"
    if mqtt_connected:
        publish_status("ONLINE")
        print("[MQTT] connected")


def mqtt_on_disconnect(client, userdata, rc):
    global mqtt_connected
    mqtt_connected = False
    with latest_lock:
        latest_status["mqtt"] = f"DISCONNECTED rc={rc}"
    print("[MQTT] disconnected", rc)


def connect_mqtt():
    global mqtt_client

    if not bool(CFG.get("enable_mqtt_publish", True)):
        with latest_lock:
            latest_status["mqtt"] = "DISABLED"
        print("[MQTT] disabled")
        return

    client = make_mqtt_client()
    client.on_connect = mqtt_on_connect
    client.on_disconnect = mqtt_on_disconnect

    try:
        client.reconnect_delay_set(1, 5)
    except Exception:
        pass

    broker = str(CFG["mqtt_broker_ip"])
    port = int(CFG["mqtt_port"])

    try:
        mqtt_client = client
        client.connect(broker, port, keepalive=30)
        client.loop_start()
        print(f"[MQTT] connecting to {broker}:{port}")
    except Exception as exc:
        with latest_lock:
            latest_status["mqtt"] = f"CONNECT ERROR: {exc}"
        print("[MQTT] connect error:", exc)


def publish(topic: str, payload: dict[str, Any], retain: bool = False):
    if not mqtt_client or not mqtt_connected:
        return False
    try:
        mqtt_client.publish(topic, json.dumps(payload), retain=retain)
        return True
    except Exception:
        return False


def publish_status(state: str):
    publish("cell/conveyor/status", {
        "robot": "conveyor",
        "state": state,
        "cameraIp": camera_ip(),
        "cameraStream": camera_stream_url(),
        "rawBridgeStream": "http://127.0.0.1:5060/raw.mjpg",
        "processedBridgeStream": "http://127.0.0.1:5060/stream.mjpg",
        "source": "conveyor_cv_bridge_single_pipeline",
    }, retain=True)


def classify_shape(_contour) -> str:
    return str(CFG.get("shape_name", "SQUARE")).upper()


def detect_color(frame):
    frame = cv2.resize(frame, (640, 480))
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Same thresholds as the original friend code.
    lower_red1 = np.array([0, 100, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 70])
    upper_red2 = np.array([180, 255, 255])

    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    lower_green = np.array([30, 60, 50])
    upper_green = np.array([100, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    lower_blue = np.array([95, 100, 70])
    upper_blue = np.array([135, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)

    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

    masks = [
        ("R", red_mask, (0, 0, 255), "RED"),
        ("G", green_mask, (0, 255, 0), "GREEN"),
        ("B", blue_mask, (255, 0, 0), "BLUE"),
    ]

    min_area = int(CFG.get("min_area", 500))
    best = {
        "code": "",
        "name": "NO COLOR",
        "area": 0,
        "box": None,
        "color": (0, 255, 255),
        "contour": None,
    }

    for code, mask, color_bgr, name in masks:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        c = max(contours, key=cv2.contourArea)
        area = int(cv2.contourArea(c))

        if area > min_area and area > best["area"]:
            x, y, w, h = cv2.boundingRect(c)
            best = {
                "code": code,
                "name": name,
                "area": area,
                "box": (x, y, w, h),
                "color": color_bgr,
                "contour": c,
            }

    return frame, best


def draw_overlay(frame, result, fps: float):
    color_name = result["name"]
    area = result["area"]
    box = result["box"]
    color = result["color"]

    if box is not None:
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, f"{color_name} area={area}", (x, max(25, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    cv2.putText(frame, f"Detected: {color_name}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, f"MQTT: {latest_status.get('mqtt', '-')}", (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(frame, "Single camera pipeline: ESP32 -> CV bridge -> dashboard", (20, 430),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(frame, f"FPS {fps:.1f}", (20, 458),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return frame


def make_package(color_name: str, shape_name: str):
    recipe_map = CFG.get("color_to_recipe", {})
    package_class = str(recipe_map.get(color_name, f"{color_name}_{shape_name}")).upper()

    return {
        "jobId": "JOB_" + uuid.uuid4().hex[:8].upper(),
        "packageClass": package_class,
        "color": color_name,
        "shape": shape_name,
        "destination": package_class.split("_TO_")[-1] if "_TO_" in package_class else "",
        "event": "PACKAGE_READY",
        "source": "conveyor_cv_bridge_single_pipeline",
        "cameraIp": camera_ip(),
        "timeMs": int(time.time() * 1000),
    }


def blank_frame(text: str):
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(frame, text[:44], (22, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
    ok, jpg = cv2.imencode(".jpg", frame)
    return jpg.tobytes() if ok else b""


def encode_jpeg(frame, quality: int = 82) -> bytes | None:
    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return jpg.tobytes() if ok else None


def mjpeg_frames(url: str) -> Generator[tuple[bytes, np.ndarray], None, None]:
    headers = {
        "User-Agent": "RobotCellCVBridge/1.0",
        "Accept": "multipart/x-mixed-replace,image/jpeg,*/*",
        "Connection": "close",
    }

    with requests.get(url, stream=True, timeout=(4, 20), headers=headers) as r:
        r.raise_for_status()
        data = b""

        for chunk in r.iter_content(chunk_size=8192):
            if not running:
                return
            if not chunk:
                continue

            data += chunk

            while True:
                start = data.find(b"\xff\xd8")
                end = data.find(b"\xff\xd9")

                if start == -1 or end == -1 or end <= start:
                    break

                jpg = data[start:end + 2]
                data = data[end + 2:]

                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    yield jpg, frame

            if len(data) > 2_000_000:
                data = data[-200_000:]


def vision_loop():
    global latest_raw_jpeg, latest_processed_jpeg

    last_sent_code = ""
    last_send_time = 0.0

    stable_code = ""
    stable_count = 0
    last_published_code = ""
    last_publish_time = 0.0

    frame_count = 0
    fps_t0 = time.time()
    fps = 0.0
    last_console_detection = ""

    while running:
        url = camera_stream_url()

        with latest_lock:
            latest_status["state"] = "CONNECTING_CAMERA"
            latest_status["camera"] = url
            latest_raw_jpeg = blank_frame("Waiting for ESP32-CAM...")
            latest_processed_jpeg = blank_frame("Connecting CV bridge to ESP32-CAM...")

        print("[CAMERA] single pipeline connecting:", url)

        try:
            publish_status("CONNECTING_CAMERA")

            for raw_jpg, frame in mjpeg_frames(url):
                now = time.time()

                # Save raw frame immediately for the dashboard.
                with latest_lock:
                    latest_raw_jpeg = raw_jpg

                frame_count += 1
                if now - fps_t0 >= 1.0:
                    fps = frame_count / (now - fps_t0)
                    frame_count = 0
                    fps_t0 = now

                frame, result = detect_color(frame)
                code = result["code"]
                color_name = result["name"]
                area = result["area"]
                shape_name = ""
                package_class = ""

                if code:
                    shape_name = classify_shape(result["contour"])
                    package_class = str(CFG.get("color_to_recipe", {}).get(
                        color_name, f"{color_name}_{shape_name}"
                    )).upper()

                    if color_name != last_console_detection:
                        print(f"[DETECT] {color_name} area={area} class={package_class}")
                        last_console_detection = color_name

                    if bool(CFG.get("enable_servo_command", True)):
                        delay = float(CFG.get("send_delay_s", 0.50))
                        if code != last_sent_code or (now - last_send_time) > delay:
                            try:
                                requests.get(servo_url(code), timeout=0.25)
                                last_sent_code = code
                                last_send_time = now
                                with latest_lock:
                                    latest_status["lastServo"] = f"Sent {code}"
                            except Exception:
                                with latest_lock:
                                    latest_status["lastServo"] = "Servo command failed"

                    if code == stable_code:
                        stable_count += 1
                    else:
                        stable_code = code
                        stable_count = 1

                    stable_needed = int(CFG.get("publish_stable_frames", 8))
                    cooldown = float(CFG.get("publish_cooldown_s", 4.0))

                    if stable_count >= stable_needed and (
                        code != last_published_code or (now - last_publish_time) > cooldown
                    ):
                        payload = make_package(color_name, shape_name)
                        if bool(CFG.get("enable_mqtt_publish", True)):
                            publish("cell/conveyor/package", payload, retain=False)
                        last_published_code = code
                        last_publish_time = now
                        with latest_lock:
                            latest_status["lastPublish"] = json.dumps(payload)
                        print("[MQTT PACKAGE]", payload)

                else:
                    stable_code = ""
                    stable_count = 0
                    last_console_detection = ""

                processed = draw_overlay(frame, result, fps)
                processed_jpg = encode_jpeg(processed)

                if processed_jpg:
                    with latest_lock:
                        latest_processed_jpeg = processed_jpg
                        latest_status["ok"] = True
                        latest_status["state"] = "RUNNING"
                        latest_status["detectedColor"] = color_name
                        latest_status["detectedCode"] = code
                        latest_status["area"] = area
                        latest_status["shape"] = shape_name
                        latest_status["packageClass"] = package_class
                        latest_status["frames"] += 1
                        latest_status["fps"] = round(fps, 1)

                publish_status("RUNNING")

        except Exception as exc:
            print("[CAMERA ERROR]", repr(exc))
            with latest_lock:
                latest_status["ok"] = False
                latest_status["state"] = "CAMERA_ERROR"
                latest_status["camera"] = url
                latest_raw_jpeg = blank_frame("ESP32-CAM connection closed")
                latest_processed_jpeg = blank_frame("Camera error. Stop dashboard direct stream.")
            time.sleep(1.8)


app = Flask(__name__)


@app.after_request
def add_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Cache-Control"] = "no-store"
    return response


@app.get("/")
def index():
    return """
    <html>
    <head><title>Conveyor CV Bridge</title></head>
    <body style="background:#000;color:#fff;font-family:Arial">
      <h2>Conveyor CV Bridge v5 — Single Camera Pipeline</h2>
      <p>ESP32-CAM has one client only: this bridge.</p>
      <p><a style="color:#93c5fd" href="/raw.mjpg">Raw bridge stream</a></p>
      <p><a style="color:#93c5fd" href="/stream.mjpg">Processed OpenCV stream</a></p>
      <p><a style="color:#93c5fd" href="/status">Status JSON</a></p>
      <h3>Processed</h3>
      <img src="/stream.mjpg" style="max-width:900px;width:100%;border:1px solid #333">
      <h3>Raw</h3>
      <img src="/raw.mjpg" style="max-width:900px;width:100%;border:1px solid #333">
    </body>
    </html>
    """


@app.get("/status")
def status():
    with latest_lock:
        return jsonify(dict(latest_status))


def stream_frames(kind: str):
    def gen():
        while True:
            with latest_lock:
                if kind == "raw":
                    frame = latest_raw_jpeg or blank_frame("Waiting for raw frame...")
                else:
                    frame = latest_processed_jpeg or blank_frame("Waiting for processed frame...")
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.045)

    return gen


@app.get("/raw.mjpg")
def raw_mjpg():
    return Response(stream_frames("raw")(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/stream.mjpg")
def processed_mjpg():
    return Response(stream_frames("processed")(), mimetype="multipart/x-mixed-replace; boundary=frame")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-ip", default=None)
    parser.add_argument("--mqtt-broker", default=None)
    parser.add_argument("--mqtt-port", type=int, default=None)
    parser.add_argument("--http-port", type=int, default=5060)
    return parser.parse_args()


def main():
    global CFG

    args = parse_args()

    if args.camera_ip:
        CFG["conveyor_camera_ip"] = clean_ip(args.camera_ip)
    if args.mqtt_broker:
        CFG["mqtt_broker_ip"] = args.mqtt_broker
    if args.mqtt_port:
        CFG["mqtt_port"] = int(args.mqtt_port)

    CONFIG_FILE.write_text(json.dumps(CFG, indent=2), encoding="utf-8")

    print("Conveyor CV Bridge v5 — SINGLE CAMERA PIPELINE")
    print("ESP32-CAM source:", camera_stream_url())
    print("Raw bridge stream: http://127.0.0.1:%d/raw.mjpg" % args.http_port)
    print("Processed stream: http://127.0.0.1:%d/stream.mjpg" % args.http_port)
    print("Status: http://127.0.0.1:%d/status" % args.http_port)
    print("MQTT broker:", CFG["mqtt_broker_ip"], CFG["mqtt_port"])
    print("Do not open ESP32-CAM direct stream in the dashboard/browser while bridge is running.")

    connect_mqtt()
    threading.Thread(target=vision_loop, daemon=True).start()
    app.run(host="0.0.0.0", port=args.http_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
