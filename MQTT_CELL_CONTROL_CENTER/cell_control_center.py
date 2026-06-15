
from __future__ import annotations

import json
import re
import socket
import subprocess
import threading
from urllib.parse import quote
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
import webbrowser
from collections import deque
from pathlib import Path
from typing import Any

import requests
import cv2
import numpy as np
import paho.mqtt.client as mqtt
from flask import Flask, jsonify, request, Response


PORT = 5050
CONFIG_FILE = Path("cell_control_center_config.json")
ROBOT_BROKER_IP = "192.168.137.1"

TOPIC = {
    "conveyor_status": "cell/conveyor/status",
    "conveyor_package": "cell/conveyor/package",
    "conveyor_cube": "cell/conveyor/cube",
    "conveyor_box_filled": "cell/conveyor/box_filled",
    "conveyor_cmd": "cell/conveyor/cmd",
    "amr_status": "cell/amr/status",
    "amr_event": "cell/amr/event",
    "amr_cmd": "cell/amr/cmd",
    "arm_status": "cell/arm/status",
    "arm_event": "cell/arm/event",
    "coordinator_status": "cell/coordinator/status",
    "coordinator_event": "cell/coordinator/event",
}


def ms() -> int:
    return int(time.time() * 1000)


def tcp_port_open(host: str, port: int, timeout: float = 0.6) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except Exception:
        return False


def mosquitto_path() -> str | None:
    candidates = [
        r"C:\Program Files\mosquitto\mosquitto.exe",
        r"C:\Program Files (x86)\mosquitto\mosquitto.exe",
        "mosquitto",
    ]
    for item in candidates:
        try:
            if item.endswith(".exe") and Path(item).exists():
                return item
            if item == "mosquitto":
                p = subprocess.run([item, "-h"], capture_output=True, text=True, timeout=2)
                if p.returncode in (0, 1):
                    return item
        except Exception:
            continue
    return None


def default_workflow_rules() -> list[dict[str, Any]]:
    return [
        {"color":"RED","box":"BoxRed","recipe":"RED_BOX","flowName":"RED_FLOW","delayBeforeArmMs":0,"filledEvent":"BoxRed_filled","armLoadedEvent":"BoxRed_loaded_on_amr","deliveredEvent":"BoxRed_finished_manual_delivery","pickup":"CONVEYOR_RED_BOX_SLOT","enabled":True},
        {"color":"GREEN","box":"BoxGreen","recipe":"GREEN_BOX_A1","flowName":"GREEN_FLOW","delayBeforeArmMs":0,"filledEvent":"BoxGreen_filled","armLoadedEvent":"BoxGreen_loaded_on_amr","deliveredEvent":"BoxGreen_finished_manual_delivery","pickup":"CONVEYOR_GREEN_BOX_SLOT","enabled":True},
        {"color":"BLUE","box":"BoxBlue","recipe":"BLUE_BOX","flowName":"BLUE_FLOW","delayBeforeArmMs":0,"filledEvent":"BoxBlue_filled","armLoadedEvent":"BoxBlue_loaded_on_amr","deliveredEvent":"BoxBlue_finished_manual_delivery","pickup":"CONVEYOR_BLUE_BOX_SLOT","enabled":True},
    ]


ARM_RECIPE_BY_COLOR = {
    "RED": "RED_BOX",
    "GREEN": "GREEN_BOX_A1",
    "BLUE": "BLUE_BOX",
}

DEFAULT_SORTER_SERVO_ANGLES = {
    "R": 30,
    "G": 85,
    "B": 150,
    "CENTER": 90,
}

SORTER_SERVO_TARGET_CODES = {
    "R": "R",
    "RED": "R",
    "G": "G",
    "GREEN": "G",
    "B": "B",
    "BLUE": "B",
    "CENTER": "CENTER",
}

LEGACY_ARM_RECIPE_NAMES = {
    "RED": {"RED_SQUARE_TO_SHELF_A", "RED_BOX_TO_SHELF_A", "REDBOX_TO_SHELF_A"},
    "GREEN": {"GREEN_SQUARE_TO_SHELF_C", "GREEN_BOX_TO_SHELF_C", "GREENBOX_TO_SHELF_C"},
    "BLUE": {"BLUE_SQUARE_TO_SHELF_B", "BLUE_BOX_TO_SHELF_B", "BLUEBOX_TO_SHELF_B"},
}


def migrate_manual_arm_workflow_rules(rules: Any) -> tuple[list[dict[str, Any]], bool]:
    if not isinstance(rules, list):
        return default_workflow_rules(), True
    migrated = []
    changed = False
    for raw in rules:
        if not isinstance(raw, dict):
            changed = True
            continue
        rule = dict(raw)
        color = str(rule.get("color", "")).strip().upper()
        expected_recipe = ARM_RECIPE_BY_COLOR.get(color)
        current_recipe = str(rule.get("recipe", "")).strip()
        if expected_recipe and (not current_recipe or current_recipe in LEGACY_ARM_RECIPE_NAMES.get(color, set())):
            rule["recipe"] = expected_recipe
            changed = changed or current_recipe != expected_recipe
        expected_done = f"{rule.get('box') or f'Box{color.title()}'}_finished_manual_delivery"
        current_done = str(rule.get("deliveredEvent", "")).strip()
        if expected_recipe and (not current_done or current_done.endswith("_delivered_to_target")):
            rule["deliveredEvent"] = expected_done
            changed = changed or current_done != expected_done
        migrated.append(rule)
    return migrated, changed


def safe_json_load(path: Path) -> dict[str, Any]:
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return {}


class AppState:
    def __init__(self):
        saved = safe_json_load(CONFIG_FILE)
        migrated_rules, workflow_migrated = migrate_manual_arm_workflow_rules(saved.get("workflow_rules", default_workflow_rules()))
        self.lock = threading.RLock()

        saved_broker_ip = str(saved.get("broker_ip", "127.0.0.1")).strip()
        # Dashboard connects locally. ESP32 devices connect to the same broker through 192.168.137.1.
        if saved_broker_ip in ("", "192.168.137.1", "localhost"):
            saved_broker_ip = "127.0.0.1"

        self.config = {
            "broker_ip": saved_broker_ip,
            "broker_port": int(saved.get("broker_port", 1883)),
            "arm_ip": saved.get("arm_ip", ""),
            "amr_ip": saved.get("amr_ip", ""),
            "conveyor_ip": saved.get("conveyor_ip", ""),
            "auto_coordinate": bool(saved.get("auto_coordinate", True)),
            "box_capacity": int(saved.get("box_capacity", 4)),
            "simulate_amr": bool(saved.get("simulate_amr", True)),
            "amr_step_delay_s": float(saved.get("amr_step_delay_s", 2.0)),
            "workflow_rules": migrated_rules,
            "cv_color_profiles": saved.get("cv_color_profiles", {}),
            "sorter_servo_angles": {
                key: max(0, min(180, int(saved.get("sorter_servo_angles", {}).get(key, angle))))
                for key, angle in DEFAULT_SORTER_SERVO_ANGLES.items()
            },
        }

        self.client: mqtt.Client | None = None
        self.mqtt_connected = False
        self.broker_proc: subprocess.Popen | None = None
        self.broker_running = False

        self.system = "BOOTING"
        self.next_action = "Starting control center."
        self.mission_state = "BOOTING"
        self.command_seq = 0
        self.pending_acks = {}
        self.ack_history = deque(maxlen=30)
        self.done_jobs = deque(maxlen=30)
        self.last_command = None
        self.last_box_filled_key = ""
        self.operation_generation = 0
        self.discovery = {
            "running": False,
            "lastScanMs": 0,
            "lastScanText": "Not scanned yet.",
            "found": {},
            "subnet": "192.168.137",
            "ports": [80],
        }

        self.nodes = {
            "coordinator": {"state": "BOOTING", "seen": 0, "last": "", "http": "LOCAL", "ip": "127.0.0.1", "httpSeen": 0, "httpInfo": ""},
            "mqtt": {"state": "OFFLINE", "seen": 0, "last": "", "http": "N/A", "ip": "", "httpSeen": 0, "httpInfo": ""},
            "conveyor": {"state": "WAITING", "seen": 0, "last": "", "http": "UNKNOWN", "ip": self.config["conveyor_ip"], "httpSeen": 0, "httpInfo": ""},
            "amr": {"state": "WAITING", "seen": 0, "last": "", "http": "UNKNOWN", "ip": self.config["amr_ip"], "httpSeen": 0, "httpInfo": ""},
            "arm": {"state": "WAITING", "seen": 0, "last": "", "http": "UNKNOWN", "ip": self.config["arm_ip"], "httpSeen": 0, "httpInfo": ""},
        }

        self.recipes: list[dict[str, Any]] = []
        self.recipe_sig = ""
        self.selected_recipe_name = ""
        self.selected_recipe_index = -1

        self.job_number = 1
        self.current_job = "JOB_001"
        self.current_package = ""
        self.last_arm_event_key = ""
        self.last_amr_event_key = ""

        self.box_counts = {"RED": 0, "GREEN": 0, "BLUE": 0}
        self.total_cube_counts = {"RED": 0, "GREEN": 0, "BLUE": 0}
        self.box_states = {"RED": "FILLING", "GREEN": "FILLING", "BLUE": "FILLING"}
        self.active_box_color = ""
        self.active_box_name = ""
        self.active_box_recipe = ""
        self.active_box_job = ""
        self.box_queue = deque()

        self.timeline = deque(maxlen=100)
        self.logs = deque(maxlen=160)
        if workflow_migrated:
            self.logs.appendleft({"t": time.strftime("%H:%M:%S"), "msg": "Migrated legacy arm recipe mappings to RED_BOX / GREEN_BOX_A1 / BLUE_BOX."})
            self.save_config()

    def save_config(self):
        with self.lock:
            CONFIG_FILE.write_text(json.dumps(self.config, indent=2), encoding="utf-8")

    def log(self, msg: str):
        with self.lock:
            self.logs.appendleft({"t": time.strftime("%H:%M:%S"), "msg": str(msg)})

    def event(self, source: str, text: str, data: Any = None):
        with self.lock:
            self.timeline.appendleft({
                "t": time.strftime("%H:%M:%S"),
                "source": source,
                "text": text,
                "data": data,
            })

    def set_mission(self, state: str, next_action: str | None = None, system: str | None = None):
        with self.lock:
            self.mission_state = str(state)
            if system is not None:
                self.system = str(system)
            else:
                self.system = str(state)
            if next_action is not None:
                self.next_action = str(next_action)

    def add_pending_ack(self, target: str, command: str, expected: str | list[str], timeout_s: float, payload: dict[str, Any], topic: str):
        with self.lock:
            self.command_seq += 1
            command_id = f"CMD_{self.command_seq:04d}"
            expected_list = expected if isinstance(expected, list) else [expected]
            item = {
                "commandId": command_id,
                "target": target,
                "command": command,
                "expected": expected_list,
                "sentMs": ms(),
                "timeoutMs": int(float(timeout_s) * 1000),
                "status": "WAITING",
                "topic": topic,
                "payload": dict(payload),
            }
            self.pending_acks[command_id] = item
            self.last_command = dict(item)
            return command_id

    def resolve_ack(self, event_name: str, data: Any = None):
        event_name = str(event_name or "")
        if not event_name:
            return False
        data = data if isinstance(data, dict) else {}
        incoming_command_id = str(data.get("commandId", "")).strip()
        incoming_job_id = str(data.get("jobId", "")).strip()
        resolved = []
        with self.lock:
            for command_id, ack in list(self.pending_acks.items()):
                if event_name not in ack.get("expected", []):
                    continue
                if incoming_command_id and incoming_command_id != command_id:
                    continue
                ack_job_id = str(ack.get("payload", {}).get("jobId", "")).strip()
                if incoming_job_id and ack_job_id and incoming_job_id != ack_job_id:
                    continue
                ack["status"] = "ACKED"
                ack["ackMs"] = ms()
                ack["ackEvent"] = event_name
                ack["ackData"] = data
                self.ack_history.appendleft(dict(ack))
                resolved.append(command_id)
                del self.pending_acks[command_id]
        return bool(resolved)

    def mark_done_job(self, payload: dict[str, Any]):
        with self.lock:
            item = dict(payload)
            item["t"] = time.strftime("%H:%M:%S")
            self.done_jobs.appendleft(item)

    def active_faults(self) -> list[dict[str, Any]]:
        faults = []
        now = ms()
        with self.lock:
            nodes = {k: dict(v) for k, v in self.nodes.items()}
            config = dict(self.config)
            pending = {k: dict(v) for k, v in self.pending_acks.items()}
            mqtt_connected = self.mqtt_connected

        if not mqtt_connected:
            faults.append({"code": "MQTT_OFFLINE", "level": "FAULT", "message": "MQTT broker connection is offline."})

        for name in ("arm", "conveyor"):
            ip = str(config.get(f"{name}_ip", "")).strip()
            if not ip:
                continue
            node = nodes.get(name, {})
            seen_age = now - int(node.get("seen") or 0)
            http_age = now - int(node.get("httpSeen") or 0)
            http_ok = node.get("http") == "ONLINE"
            if seen_age > 15000 and not (http_ok and http_age < 16000):
                faults.append({"code": f"{name.upper()}_OFFLINE", "level": "FAULT", "message": f"{name.upper()} has no fresh heartbeat/status."})

        if not bool(config.get("simulate_amr", True)):
            ip = str(config.get("amr_ip", "")).strip()
            node = nodes.get("amr", {})
            seen_age = now - int(node.get("seen") or 0)
            http_age = now - int(node.get("httpSeen") or 0)
            http_ok = node.get("http") == "ONLINE"
            if ip and seen_age > 15000 and not (http_ok and http_age < 16000):
                faults.append({"code": "AMR_OFFLINE", "level": "FAULT", "message": "AMR has no fresh heartbeat/status."})

        for ack in pending.values():
            age = now - int(ack.get("sentMs") or 0)
            timeout = int(ack.get("timeoutMs") or 0)
            if timeout > 0 and age > timeout:
                ack["status"] = "TIMEOUT"
                faults.append({
                    "code": "ACK_TIMEOUT",
                    "level": "FAULT",
                    "message": f"{ack.get('target','?').upper()} did not acknowledge {ack.get('command','?')}.",
                    "commandId": ack.get("commandId", ""),
                })

        try:
            with CV_LOCK:
                cv_state = CV.get("state", "")
                cv_err = CV.get("lastError", "")
            if cv_state in ("CAMERA_ERROR", "ERROR", "FRAME_READ_FAILED"):
                faults.append({"code": "CAMERA_FAULT", "level": "FAULT", "message": f"OpenCV camera state: {cv_state}. {cv_err}"})
        except Exception:
            pass

        return faults

    def seen(self, node: str, state: str | None = None, data: Any = None):
        with self.lock:
            if node not in self.nodes:
                return
            self.nodes[node]["seen"] = ms()
            if state is not None:
                self.nodes[node]["state"] = state
            if data is not None:
                self.nodes[node]["last"] = data if isinstance(data, str) else json.dumps(data, ensure_ascii=False)

    def http_seen(self, node: str, ok: bool, info: str, ip: str):
        with self.lock:
            if node not in self.nodes:
                return
            self.nodes[node]["ip"] = ip
            self.nodes[node]["httpSeen"] = ms()
            self.nodes[node]["http"] = "ONLINE" if ok else "OFFLINE"
            self.nodes[node]["httpInfo"] = info
            if ok and self.nodes[node]["state"] in ("WAITING", "WAITING_IP", "HTTP_OFFLINE", "UNKNOWN"):
                self.nodes[node]["state"] = "HTTP_ONLINE"
            elif not ok and self.nodes[node]["state"] in ("WAITING", "HTTP_ONLINE", "UNKNOWN"):
                self.nodes[node]["state"] = "HTTP_OFFLINE"

    def snap(self) -> dict[str, Any]:
        with self.lock:
            now = ms()
            nodes = {}

            for name, d in self.nodes.items():
                age = now - int(d["seen"] or 0)
                http_age = now - int(d["httpSeen"] or 0)
                http_ok = d.get("http") in ("ONLINE", "LOCAL", "N/A")

                if name == "mqtt":
                    led = "online" if self.mqtt_connected else "offline"
                elif name == "coordinator":
                    led = "online"
                elif age < 6000 or (http_ok and http_age < 7000):
                    led = "online"
                elif age < 15000 or (http_ok and http_age < 16000):
                    led = "warning"
                else:
                    led = "offline"

                nodes[name] = dict(d)
                nodes[name]["ageMs"] = age
                nodes[name]["httpAgeMs"] = http_age
                nodes[name]["led"] = led

            return {
                "config": dict(self.config),
                "brokerRunning": self.broker_running,
                "mqttConnected": self.mqtt_connected,
                "system": self.system,
                "nextAction": self.next_action,
                "nodes": nodes,
                "recipes": list(self.recipes),
                "selectedRecipeName": self.selected_recipe_name,
                "selectedRecipeIndex": self.selected_recipe_index,
                "currentJob": self.current_job,
                "currentPackage": self.current_package,
                "missionState": self.mission_state,
                "faults": self.active_faults(),
                "pendingAcks": list(self.pending_acks.values()),
                "ackHistory": list(self.ack_history),
                "doneJobs": list(self.done_jobs),
                "lastCommand": dict(self.last_command) if self.last_command else None,
                "discovery": dict(self.discovery),
                "boxCounts": dict(self.box_counts),
                "totalCubeCounts": dict(self.total_cube_counts),
                "boxStates": dict(self.box_states),
                "boxCapacity": int(self.config.get("box_capacity", 4)),
                "simulateAmr": bool(self.config.get("simulate_amr", True)),
                "activeBoxColor": self.active_box_color,
                "activeBoxName": self.active_box_name,
                "activeBoxRecipe": self.active_box_recipe,
                "activeBoxJob": self.active_box_job,
                "queuedBoxes": list(self.box_queue),
                "workflowRules": list(self.config.get("workflow_rules", default_workflow_rules())),
                "timeline": list(self.timeline),
                "logs": list(self.logs),
            }


S = AppState()
app = Flask(__name__)
ARM_HTTP_LOCK = threading.RLock()


def mqtt_client_factory():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id="robot_cell_control_center")
    except Exception:
        return mqtt.Client(client_id="robot_cell_control_center")


def publish(topic: str, payload: dict[str, Any], retain: bool = False) -> bool:
    with S.lock:
        client = S.client
        connected = S.mqtt_connected

    if not client or not connected:
        S.log(f"MQTT offline. Cannot publish {topic}.")
        return False

    payload = dict(payload)
    payload.setdefault("schema", "robot_cell.v1")
    payload.setdefault("timeMs", ms())
    payload.setdefault("source", "coordinator")
    text = json.dumps(payload)
    client.publish(topic, text, retain=retain)
    S.log(f"OUT {topic}: {text}")
    return True


def start_broker() -> bool:
    with S.lock:
        broker_port = int(S.config.get("broker_port", 1883))
        proc = S.broker_proc
        if proc and proc.poll() is None:
            S.broker_running = True
            return True

    # Restore old reliable behavior: dashboard uses local MQTT.
    if tcp_port_open("127.0.0.1", broker_port, timeout=0.4):
        with S.lock:
            S.broker_running = True
        if tcp_port_open(ROBOT_BROKER_IP, broker_port, timeout=0.6):
            S.log(f"MQTT broker reachable on localhost and robot network {ROBOT_BROKER_IP}:{broker_port}.")
        else:
            S.log(
                f"MQTT broker is LOCALHOST-ONLY. Dashboard can connect, but robots cannot reach "
                f"{ROBOT_BROKER_IP}:{broker_port}. Run SETUP_MQTT_BROKER_ADMIN.ps1 as Administrator."
            )
        return True

    exe = mosquitto_path()
    if not exe:
        S.log("Mosquitto not found. Install Mosquitto or start broker manually on port 1883.")
        return False

    # One broker, two access addresses:
    #   Dashboard -> 127.0.0.1:1883
    #   ESP32     -> 192.168.137.1:1883
    cfg = Path.cwd() / "mosquitto_robot_cell.conf"
    cfg.write_text(
        f"listener {broker_port} 0.0.0.0\\nallow_anonymous true\\n",
        encoding="utf-8",
    )

    try:
        proc = subprocess.Popen(
            [exe, "-v", "-c", str(cfg)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        with S.lock:
            S.broker_proc = proc
            S.broker_running = True

        threading.Thread(target=broker_reader, daemon=True).start()
        time.sleep(0.8)

        if tcp_port_open("127.0.0.1", broker_port, timeout=1.0):
            S.log(f"Mosquitto broker started. Dashboard uses 127.0.0.1:{broker_port}; ESP32 uses 192.168.137.1:{broker_port}.")
            if not tcp_port_open(ROBOT_BROKER_IP, broker_port, timeout=0.5):
                S.log(f"Warning: broker local is OK, but {ROBOT_BROKER_IP}:{broker_port} is not reachable. Run SETUP_MQTT_BROKER_ADMIN.ps1 as Administrator.")
            return True

        S.log("Mosquitto process started but local broker port is not reachable yet.")
        return False

    except Exception as exc:
        S.log(f"Broker start failed: {exc}")
        return False


def broker_reader():
    with S.lock:
        proc = S.broker_proc
    if not proc or not proc.stdout:
        return

    for line in proc.stdout:
        txt = line.strip()
        low = txt.lower()
        if "running" in low or "error" in low or "opening" in low or "new connection" in low or "address already" in low:
            S.log("BROKER: " + txt)


def mqtt_connect() -> bool:
    with S.lock:
        broker = S.config["broker_ip"]
        port = int(S.config["broker_port"])
        existing = S.client
        already_connected = S.mqtt_connected

    if existing and already_connected:
        return True

    if existing and not already_connected:
        try:
            existing.loop_stop()
            existing.disconnect()
        except Exception:
            pass

    try:
        client = mqtt_client_factory()
        client.on_connect = on_mqtt_connect
        client.on_disconnect = on_mqtt_disconnect
        client.on_message = on_mqtt_message
        try:
            client.reconnect_delay_set(min_delay=1, max_delay=5)
        except Exception:
            pass

        with S.lock:
            S.client = client

        client.connect(broker, port, keepalive=45)
        client.loop_start()

        S.log(f"Connecting MQTT to {broker}:{port}.")
        return True
    except Exception as exc:
        with S.lock:
            S.mqtt_connected = False
            S.nodes["mqtt"]["state"] = "OFFLINE"
        S.log(f"MQTT connect failed: {exc}")
        return False


def mqtt_disconnect():
    with S.lock:
        client = S.client
        S.client = None
        S.mqtt_connected = False
        S.nodes["mqtt"]["state"] = "OFFLINE"

    try:
        if client:
            client.loop_stop()
            client.disconnect()
    except Exception:
        pass


def on_mqtt_connect(client, userdata, flags, rc):
    if rc == 0:
        with S.lock:
            S.mqtt_connected = True
            if S.mission_state == "BOOTING":
                S.system = "ONLINE"
                S.mission_state = "IDLE"
                S.next_action = "Monitoring all subsystems."
            elif S.mission_state == "FAULT" and not S.active_box_job:
                S.system = "ONLINE"
                S.mission_state = "IDLE"
                S.next_action = "MQTT reconnected. Monitoring all subsystems."
            elif S.mission_state == "FAULT":
                S.next_action = "MQTT reconnected. Review the active mission before retrying or resetting."
        S.seen("mqtt", "CONNECTED", {"broker": S.config["broker_ip"]})
        client.subscribe("cell/#")
        publish(TOPIC["coordinator_status"], {"robot": "coordinator", "state": "ONLINE"}, retain=True)
        S.event("COORDINATOR", "MQTT connected")
        S.log("MQTT connected. Subscribed to cell/#.")
    else:
        S.log(f"MQTT rejected connection rc={rc}")


def on_mqtt_disconnect(client, userdata, rc):
    with S.lock:
        S.mqtt_connected = False
        if S.mission_state != "STOPPED":
            S.system = "MQTT OFFLINE"
            S.mission_state = "FAULT"
            S.next_action = "Broker connection lost."
    S.seen("mqtt", "DISCONNECTED", {"rc": rc})
    S.log(f"MQTT disconnected rc={rc}")


def on_mqtt_message(client, userdata, msg):
    topic = msg.topic
    text = msg.payload.decode(errors="replace")

    try:
        data = json.loads(text)
    except Exception:
        data = {"raw": text}

    if topic != TOPIC["arm_status"]:
        S.log(f"IN {topic}: {text}")

    if topic == TOPIC["conveyor_status"]:
        S.seen("conveyor", data.get("state", "ONLINE"), data)
        S.event("CONVEYOR", "MQTT status update", data)

    elif topic == TOPIC["conveyor_package"]:
        package = data.get("packageClass", "")
        S.seen("conveyor", "PACKAGE_READY", data)
        S.event("CONVEYOR", f"package ready: {package}", data)
        handle_conveyor_package(data)

    elif topic == TOPIC["conveyor_cube"]:
        color = data.get("color", "")
        S.seen("conveyor", f"{color}_CUBE_COUNTED", data)
        S.event("CONVEYOR", f"{color} cube counted", data)
        if data.get("source") != "internal_opencv_dashboard":
            record_conveyor_cube(color)

    elif topic == TOPIC["conveyor_box_filled"]:
        box = data.get("box", "")
        S.seen("conveyor", f"{box}_FILLED", data)
        S.event("CONVEYOR", f"{box} filled", data)
        handle_conveyor_box_filled(data)

    elif topic == TOPIC["amr_status"]:
        S.seen("amr", data.get("state", "ONLINE"), data)
        S.event("AMR", "MQTT status update", data)

    elif topic == TOPIC["amr_event"]:
        event = data.get("event", "EVENT")
        S.seen("amr", event, data)
        S.event("AMR", event, data)
        handle_amr_event(data)

    elif topic == TOPIC["arm_status"]:
        S.seen("arm", data.get("state", "ONLINE"), data)

    elif topic == TOPIC["arm_event"]:
        event = data.get("event", "EVENT")
        S.seen("arm", event, data)
        S.event("ARM", event, data)


def clean_ip(ip: str) -> str:
    ip = str(ip or "").strip()
    ip = ip.replace("http://", "").replace("https://", "")
    ip = ip.strip("/")
    return ip


def test_http_node(node: str, ip: str | None = None, timeout: float = 1.2) -> dict[str, Any]:
    node = node.lower().strip()
    if ip is None:
        with S.lock:
            if node == "arm":
                ip = S.config["arm_ip"]
            elif node == "amr":
                ip = S.config["amr_ip"]
            elif node == "conveyor":
                ip = S.config["conveyor_ip"]
            else:
                ip = ""
    ip = clean_ip(ip)

    if not ip:
        S.http_seen(node, False, "No IP set", "")
        return {"ok": False, "node": node, "ip": "", "message": "No IP set"}

    paths = {
        "arm": ["/job/status", "/net", "/status", "/"],
        "amr": ["/status", "/job/status", "/net", "/"],
        "conveyor": ["/status", "/net", "/"],
    }.get(node, ["/status", "/net", "/"])

    base = "http://" + ip
    last_error = ""

    for path in paths:
        url = base + path
        try:
            r = requests.get(url, timeout=timeout)
            if 200 <= r.status_code < 400:
                info = f"HTTP {r.status_code} {path}"
                status_payload = {}
                try:
                    status_payload = r.json()
                except Exception:
                    status_payload = {"raw": r.text[:350]}
                S.http_seen(node, True, info, ip)
                # Important: HTTP 200 means connected. Robot state like NOT_HOMED is not a connection failure.
                if isinstance(status_payload, dict):
                    robot_state = str(status_payload.get("state") or status_payload.get("amrState") or status_payload.get("status") or "HTTP_ONLINE")
                    S.seen(node, robot_state, status_payload)
                else:
                    S.seen(node, "HTTP_ONLINE", {"url": url})
                return {"ok": True, "node": node, "ip": ip, "url": url, "message": info, "status": r.status_code, "data": status_payload}
            last_error = f"HTTP {r.status_code} {path}"
        except Exception as exc:
            last_error = str(exc)

    S.http_seen(node, False, last_error or "No response", ip)
    return {"ok": False, "node": node, "ip": ip, "message": last_error or "No response"}


def arm_base_url() -> str:
    with S.lock:
        return "http://" + clean_ip(S.config["arm_ip"])


def arm_get(path: str, timeout: float = 2.5):
    try:
        with ARM_HTTP_LOCK:
            r = requests.get(arm_base_url() + path, timeout=timeout)
        try:
            return r.status_code, r.json()
        except Exception:
            return r.status_code, {"ok": False, "raw": r.text[:500]}
    except Exception as exc:
        return 0, {"ok": False, "err": str(exc)}


def recipe_signature(recipes: list[dict[str, Any]]) -> str:
    parts = []
    for r in recipes:
        steps = r.get("steps", [])
        parts.append(f"{r.get('index')}|{r.get('name')}|{len(steps)}")
        for st in steps:
            parts.append(
                f"{st.get('index')}:{st.get('name')}:{st.get('command')}:{st.get('a1')}:{st.get('a2')}:{st.get('z')}:{st.get('gripper')}:{st.get('waitMs')}"
            )
    return "||".join(parts)


def refresh_arm_recipes() -> bool:
    code, data = arm_get("/config", timeout=3.5)
    if code != 200 or not isinstance(data.get("packageRecipes"), list):
        return False

    recipes = data["packageRecipes"]
    sig = recipe_signature(recipes)

    with S.lock:
        S.seen("arm")
        if sig == S.recipe_sig:
            return True

        old = S.selected_recipe_name
        S.recipes = recipes
        S.recipe_sig = sig

        if not recipes:
            S.selected_recipe_name = ""
            S.selected_recipe_index = -1
            S.system = "NO RECIPES"
            S.next_action = "Create/save recipes in the arm webpage."
            S.event("ARM", "no recipes stored")
            return True

        names = [str(r.get("name", "")) for r in recipes]
        selected = names.index(old) if old in names else 0
        recipe = recipes[selected]

        S.selected_recipe_name = str(recipe.get("name", ""))
        S.selected_recipe_index = int(recipe.get("index", selected))
        S.event("ARM", "recipes synced", {"recipes": names})

    return True


def poll_arm_status() -> bool:
    code, data = arm_get("/job/status", timeout=2.5)
    if code != 200:
        return False

    S.http_seen("arm", True, "HTTP 200 /job/status", S.config["arm_ip"])

    state = data.get("state", "UNKNOWN")
    S.seen("arm", state, data)

    publish(TOPIC["arm_status"], {
        "robot": "arm",
        "state": state,
        "jobId": data.get("jobId", ""),
        "packageClass": data.get("packageClass", ""),
        "recipe": data.get("recipe", ""),
        "result": data.get("result", ""),
        "message": data.get("message", ""),
    }, retain=True)

    key = f"{state}:{data.get('jobId','')}"
    with S.lock:
        if S.mission_state == "STOPPED":
            S.last_arm_event_key = key
            return True
        if key == S.last_arm_event_key:
            return True
        active_job = S.active_box_job

    status_job = str(data.get("jobId", "")).strip()
    if state == "DONE" and active_job and status_job and status_job != active_job:
        with S.lock:
            S.last_arm_event_key = key
        S.log(f"Ignored arm DONE for stale job {status_job}; active job is {active_job}.")
        return True
    if state == "DONE" and not active_job:
        with S.lock:
            S.last_arm_event_key = key
        S.log(f"Ignored arm DONE for {status_job or 'unknown job'} because no box mission is active.")
        return True
    if state == "DONE":
        with S.lock:
            mission_state = S.mission_state
        if mission_state not in ("ARM_LOADING_AMR", "ARM_RUNNING"):
            with S.lock:
                S.last_arm_event_key = key
            S.log(f"Ignored arm DONE for {status_job or active_job}; mission state is {mission_state}.")
            return True

    if state == "DONE":
        with S.lock:
            S.last_arm_event_key = key
            S.system = "BOX LOADED ON AMR"
            S.mission_state = "BOX_LOADED_ON_AMR"
            S.next_action = "Arm finished loading. Waiting for manual AMR delivery."
        handle_arm_loaded_on_amr(data)

    elif state in ("FAULT", "STOPPED"):
        with S.lock:
            S.last_arm_event_key = key
            S.system = state
            S.mission_state = "FAULT"
            S.next_action = "Arm fault/stopped. Human reset required."
        S.event("ARM", state, data)
        publish(TOPIC["arm_event"], {
            "event": state,
            "jobId": data.get("jobId", S.current_job),
            "message": data.get("message", ""),
        })

    return True


def select_recipe_by_name(name: str) -> bool:
    target = str(name).strip()
    with S.lock:
        for i, recipe in enumerate(S.recipes):
            if str(recipe.get("name", "")).strip() == target:
                S.selected_recipe_name = str(recipe.get("name", ""))
                S.selected_recipe_index = int(recipe.get("index", i))
                return True
    return False


def arm_prepare_for_new_job() -> dict[str, Any]:
    code, data = arm_get("/job/status", timeout=5)
    state = str(data.get("state", "")).strip().upper()
    result = {"stage": "prepare", "ok": False, "code": code, "state": state, "data": data}
    S.log(f"ARM prepare /job/status -> {code}: {data}")

    if code != 200:
        return result
    if state == "READY":
        result["ok"] = True
        return result
    if state != "DONE":
        return result

    clear_code, clear_data = arm_get("/job/clear", timeout=5)
    S.log(f"ARM prepare /job/clear -> {clear_code}: {clear_data}")
    result.update({
        "clearAttempted": True,
        "clearCode": clear_code,
        "clearData": clear_data,
        "ok": clear_code == 200 and bool(clear_data.get("ok")),
    })
    if result["ok"]:
        result["state"] = "READY"
        S.seen("arm", "READY", clear_data)
    return result


def arm_load_job() -> dict[str, Any]:
    with S.lock:
        idx = int(S.selected_recipe_index)
        recipe_name = S.selected_recipe_name
        job = S.current_job

    if idx < 0:
        S.log("No selected arm recipe.")
        return {"stage": "load", "ok": False, "code": 0, "data": {"err": "No selected arm recipe."}}

    path = f"/job/load?recipe={idx}&jobId={quote(job)}&packageClass={quote(recipe_name)}"
    code, data = arm_get(path, timeout=5)
    S.log(f"ARM /job/load -> {code}: {data}")
    result = {"stage": "load", "ok": code == 200 and bool(data.get("ok")), "code": code, "data": data}

    if result["ok"]:
        S.seen("arm", "JOB_LOADED", data)
    return result


def arm_wait_amr() -> dict[str, Any]:
    code, data = arm_get("/job/wait-amr", timeout=5)
    S.log(f"ARM /job/wait-amr -> {code}: {data}")
    result = {"stage": "wait-amr", "ok": code == 200 and bool(data.get("ok")), "code": code, "data": data}

    if result["ok"]:
        S.seen("arm", "WAITING_FOR_AMR", data)
    return result


def arm_amr_arrived() -> dict[str, Any]:
    code, data = arm_get("/job/amr-arrived", timeout=5)
    S.log(f"ARM /job/amr-arrived -> {code}: {data}")
    result = {"stage": "start", "ok": code == 200 and bool(data.get("ok")), "code": code, "data": data}

    if result["ok"]:
        S.seen("arm", "RUNNING", data)
        with S.lock:
            S.system = "ARM RUNNING"
            S.mission_state = "ARM_LOADING_AMR"
            S.next_action = "Arm is moving the filled box from the conveyor onto the AMR."
    return result



# ============================================================
# Box filling + arm loading + AMR target delivery workflow
# ============================================================

BOX_META = {
    "RED": {
        "box": "BoxRed",
        "boxFilledEvent": "BoxRed_filled",
        "armLoadedEvent": "BoxRed_loaded_on_amr",
        "deliveredEvent": "BoxRed_delivered_to_target",
        "target": "TARGET_RED",
        "pickup": "CONVEYOR_RED_BOX_SLOT",
        "recipes": ["RED_BOX"],
    },
    "GREEN": {
        "box": "BoxGreen",
        "boxFilledEvent": "BoxGreen_filled",
        "armLoadedEvent": "BoxGreen_loaded_on_amr",
        "deliveredEvent": "BoxGreen_delivered_to_target",
        "target": "TARGET_GREEN",
        "pickup": "CONVEYOR_GREEN_BOX_SLOT",
        "recipes": ["GREEN_BOX_A1"],
    },
    "BLUE": {
        "box": "BoxBlue",
        "boxFilledEvent": "BoxBlue_filled",
        "armLoadedEvent": "BoxBlue_loaded_on_amr",
        "deliveredEvent": "BoxBlue_delivered_to_target",
        "target": "TARGET_BLUE",
        "pickup": "CONVEYOR_BLUE_BOX_SLOT",
        "recipes": ["BLUE_BOX"],
    },
}



def workflow_rules() -> list[dict[str, Any]]:
    with S.lock:
        rules = S.config.get("workflow_rules", default_workflow_rules())
    if not isinstance(rules, list):
        rules = default_workflow_rules()
    out=[]
    for r in rules:
        if not isinstance(r, dict): continue
        color = normalize_color(r.get("color", ""))
        if not color: continue
        box = str(r.get("box") or f"Box{color.title()}").strip()
        recipe = str(r.get("recipe") or "").strip()
        if not recipe: recipe = ARM_RECIPE_BY_COLOR.get(color, f"{color}_BOX")
        out.append({
            "flowName": str(r.get("flowName") or r.get("name") or f"{color}_SYSTEM_FLOW").strip(),
            "color": color,
            "box": box,
            "recipe": recipe,
            "amrRecipeIndex": int(r.get("amrRecipeIndex", 0 if color == "RED" else 1 if color == "GREEN" else 2)),
            "amrRecipeName": str(r.get("amrRecipeName") or "").strip(),
            "target": str(r.get("target") or f"TARGET_{color}").strip(),
            "delayBeforeAmrMs": max(0, int(float(r.get("delayBeforeAmrMs", 500)))),
            "delayBeforeArmMs": max(0, int(float(r.get("delayBeforeArmMs", 0)))),
            "filledEvent": str(r.get("filledEvent") or f"{box}_filled").strip(),
            "armLoadedEvent": str(r.get("armLoadedEvent") or f"{box}_loaded_on_amr").strip(),
            "deliveredEvent": str(r.get("deliveredEvent") or f"{box}_delivered_to_target").strip(),
            # Legacy fields remain readable so old saved files and integrations still load.
            "pickedEvent": str(r.get("pickedEvent") or f"Done_Picking_{box}").strip(),
            "arrivalEvent": str(r.get("arrivalEvent") or "AMR_arrived_to_arm_station").strip(),
            "dockingEvent": str(r.get("dockingEvent") or "Done_Docking").strip(),
            "readyEvent": str(r.get("readyEvent") or f"{box}_ready").strip(),
            "shelfEvent": str(r.get("shelfEvent") or f"{box}_in_shelf").strip(),
            "pickup": str(r.get("pickup") or f"CONVEYOR_{color}_BOX_SLOT").strip(),
            "enabled": bool(r.get("enabled", True)),
        })
    return out


def save_workflow_rules(rules: list[dict[str, Any]]):
    rules, _ = migrate_manual_arm_workflow_rules(rules)
    cleaned=[]
    for r in rules:
        if not isinstance(r, dict): continue
        color=normalize_color(r.get("color", ""))
        if not color: continue
        box=str(r.get("box") or f"Box{color.title()}").strip()
        cleaned.append({
            "flowName": str(r.get("flowName") or r.get("name") or f"{color}_SYSTEM_FLOW").strip(),
            "color": color,
            "box": box,
            "recipe": str(r.get("recipe") or ARM_RECIPE_BY_COLOR.get(color, f"{color}_BOX")).strip(),
            "amrRecipeIndex": int(float(r.get("amrRecipeIndex", 0 if color == "RED" else 1 if color == "GREEN" else 2))),
            "amrRecipeName": str(r.get("amrRecipeName") or "").strip(),
            "target": str(r.get("target") or f"TARGET_{color}").strip(),
            "delayBeforeAmrMs": max(0, int(float(r.get("delayBeforeAmrMs", 500)))),
            "delayBeforeArmMs": max(0, int(float(r.get("delayBeforeArmMs", 0)))),
            "filledEvent": str(r.get("filledEvent") or f"{box}_filled").strip(),
            "armLoadedEvent": str(r.get("armLoadedEvent") or f"{box}_loaded_on_amr").strip(),
            "deliveredEvent": str(r.get("deliveredEvent") or f"{box}_delivered_to_target").strip(),
            "pickedEvent": str(r.get("pickedEvent") or f"Done_Picking_{box}").strip(),
            "arrivalEvent": str(r.get("arrivalEvent") or "AMR_arrived_to_arm_station").strip(),
            "dockingEvent": str(r.get("dockingEvent") or "Done_Docking").strip(),
            "readyEvent": str(r.get("readyEvent") or f"{box}_ready").strip(),
            "shelfEvent": str(r.get("shelfEvent") or f"{box}_in_shelf").strip(),
            "pickup": str(r.get("pickup") or f"CONVEYOR_{color}_BOX_SLOT").strip(),
            "enabled": bool(r.get("enabled", True)),
        })
    with S.lock:
        S.config["workflow_rules"] = cleaned
    S.save_config()


def configured_meta_for_color(color: str) -> dict[str, Any] | None:
    color=normalize_color(color)
    for r in workflow_rules():
        if r["enabled"] and r["color"] == color:
            return {
                "flowName": r.get("flowName", f"{r['color']}_SYSTEM_FLOW"),
                "box": r["box"],
                "boxFilledEvent": r["filledEvent"],
                "amrRecipeIndex": int(r.get("amrRecipeIndex", 0)),
                "amrRecipeName": r.get("amrRecipeName", ""),
                "target": r.get("target", f"TARGET_{r['color']}"),
                "delayBeforeAmrMs": int(r.get("delayBeforeAmrMs", 500)),
                "delayBeforeArmMs": int(r.get("delayBeforeArmMs", 0)),
                "armLoadedEvent": r["armLoadedEvent"],
                "deliveredEvent": r["deliveredEvent"],
                "pickedEvent": r["pickedEvent"],
                "arrivalEvent": r["arrivalEvent"],
                "dockingEvent": r["dockingEvent"],
                "readyEvent": r["readyEvent"],
                "shelfEvent": r["shelfEvent"],
                "pickup": r["pickup"],
                "recipes": [r["recipe"]],
            }
    return None


def meta_for_ready_event(event: str) -> tuple[str, dict[str, Any]] | tuple[str, None]:
    event=str(event or "").strip()
    for r in workflow_rules():
        if r["enabled"] and r["readyEvent"] == event:
            return r["color"], configured_meta_for_color(r["color"])
    return "", None


def meta_for_delivered_event(event: str) -> tuple[str, dict[str, Any]] | tuple[str, None]:
    event = str(event or "").strip()
    for r in workflow_rules():
        if r["enabled"] and r["deliveredEvent"] == event:
            return r["color"], configured_meta_for_color(r["color"])
    return "", None


def workflow_colors() -> list[str]:
    return [r["color"] for r in workflow_rules() if r.get("enabled", True)]


def ensure_box_keys(color: str):
    color=normalize_color(color)
    with S.lock:
        if color not in S.box_counts: S.box_counts[color]=0
        if color not in S.total_cube_counts: S.total_cube_counts[color]=0
        if color not in S.box_states: S.box_states[color]="FILLING"

def normalize_color(color: str) -> str:
    return str(color or "").strip().upper()


def meta_for_color(color: str) -> dict[str, Any] | None:
    return configured_meta_for_color(color) or BOX_META.get(normalize_color(color))


def recipe_for_box_color(color: str) -> str:
    meta = meta_for_color(color)
    if not meta:
        return ""
    candidates = [str(x) for x in meta["recipes"]]
    with S.lock:
        stored_names = [str(r.get("name", "")) for r in S.recipes]
    for candidate in candidates:
        if candidate in stored_names:
            return candidate
    for candidate in candidates:
        for stored in stored_names:
            if stored.upper() == candidate.upper():
                return stored
    return candidates[-1]


def select_recipe_for_color(color: str) -> bool:
    recipe_name = recipe_for_box_color(color)
    if not recipe_name:
        return False
    return select_recipe_by_name(recipe_name)


def box_status_payload(color: str) -> dict[str, Any]:
    color = normalize_color(color)
    meta = meta_for_color(color) or {}
    with S.lock:
        capacity = int(S.config.get("box_capacity", 4))
        return {
            "color": color,
            "box": meta.get("box", f"Box{color.title()}"),
            "count": int(S.box_counts.get(color, 0)),
            "totalCount": int(S.total_cube_counts.get(color, 0)),
            "capacity": capacity,
            "state": S.box_states.get(color, "UNKNOWN"),
            "recipe": recipe_for_box_color(color),
        }


def reset_box_color(color: str):
    color = normalize_color(color)
    meta0 = meta_for_color(color)
    if not meta0:
        return
    ensure_box_keys(color)
    with S.lock:
        S.box_counts[color] = 0
        S.box_states[color] = "FILLING"
        S.box_queue = deque(item for item in S.box_queue if normalize_color(item.get("color", "")) != color)
        if S.active_box_color == color:
            S.operation_generation += 1
            S.active_box_color = ""
            S.active_box_name = ""
            S.active_box_recipe = ""
            S.active_box_job = ""
    S.event("CONVEYOR", f"{meta0['box']} reset", box_status_payload(color))
    publish(TOPIC["conveyor_status"], {
        "robot": "conveyor",
        "state": "BOX_RESET",
        **box_status_payload(color),
    }, retain=False)


def reset_all_boxes():
    for c in workflow_colors() or ("RED", "GREEN", "BLUE"):
        reset_box_color(c)


def record_conveyor_cube(color_name: str):
    color = normalize_color(color_name)
    meta = meta_for_color(color)
    if not meta:
        return
    ensure_box_keys(color)

    with S.lock:
        state = S.box_states.get(color, "FILLING")
        capacity = int(S.config.get("box_capacity", 4))
        job = S.current_job

        if state != "FILLING":
            payload = box_status_payload(color)
            S.event("CONVEYOR", f"{meta['box']} ignored cube because state={state}", payload)
            return

        S.box_counts[color] = int(S.box_counts.get(color, 0)) + 1
        S.total_cube_counts[color] = int(S.total_cube_counts.get(color, 0)) + 1
        count = int(S.box_counts[color])
        total = int(S.total_cube_counts[color])
        filled = count >= capacity

        if filled:
            S.box_states[color] = "FILLED_WAITING_ARM"
            S.job_number += 1
            job = f"JOB_{S.job_number:03d}"
            if not S.active_box_job:
                S.current_job = job
                S.current_package = meta["boxFilledEvent"]

    cube_payload = {
        "event": "CUBE_COUNTED",
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "count": count,
        "totalCount": total,
        "capacity": capacity,
        "state": S.box_states.get(color, "FILLING"),
        "source": "internal_opencv_dashboard",
        "timeMs": int(time.time() * 1000),
    }
    publish(TOPIC["conveyor_cube"], cube_payload, retain=False)
    publish(TOPIC["conveyor_status"], {
        "robot": "conveyor",
        "state": "COUNTING",
        "counts": dict(S.box_counts),
        "boxStates": dict(S.box_states),
        "lastCube": cube_payload,
    }, retain=True)
    S.seen("conveyor", f"{color}_COUNT_{count}", cube_payload)
    S.event("CONVEYOR", f"{color} cube counted {count}/{capacity}", cube_payload)
    with S.lock:
        if not filled:
            S.mission_state = "FILLING_BOX"
            S.system = "FILLING_BOX"
            S.next_action = f"{meta['box']} filling: {count}/{capacity}."

    if filled:
        recipe_name = recipe_for_box_color(color)
        filled_payload = {
            "event": meta["boxFilledEvent"],
            "jobId": job,
            "color": color,
            "box": meta["box"],
            "count": count,
            "capacity": capacity,
            "recipe": recipe_name,
            "pickup": meta["pickup"],
            "source": "internal_opencv_dashboard",
            "timeMs": int(time.time() * 1000),
        }
        publish(TOPIC["conveyor_box_filled"], filled_payload, retain=False)
        handle_conveyor_box_filled(filled_payload)


def schedule_delayed(delay_ms: int, fn, *args):
    try:
        delay_ms = max(0, int(delay_ms))
    except Exception:
        delay_ms = 0
    if delay_ms <= 0:
        fn(*args)
    else:
        threading.Timer(delay_ms / 1000.0, fn, args=args).start()


def start_next_queued_box() -> bool:
    with S.lock:
        if S.mission_state == "STOPPED" or S.active_box_job or not S.box_queue:
            return False
        data = dict(S.box_queue.popleft())
        S.last_box_filled_key = ""
    S.event("COORDINATOR", f"Starting next queued box: {data.get('box', data.get('color', 'box'))}", data)
    handle_conveyor_box_filled(data)
    return True


def handle_conveyor_box_filled(data: dict[str, Any]):
    color = normalize_color(data.get("color", ""))
    meta = meta_for_color(color)
    if not meta:
        return

    job = data.get("jobId", S.current_job)
    recipe_name = recipe_for_box_color(color)
    event_key = f"{data.get('event', meta['boxFilledEvent'])}:{job}:{color}:{meta['box']}"

    with S.lock:
        if S.mission_state == "STOPPED":
            S.log(f"Ignored box-filled event while STOPPED: {event_key}. Reset the mission to resume.")
            return
        if event_key == S.last_box_filled_key:
            S.log(f"Ignored duplicate box-filled event {event_key}.")
            return
        S.last_box_filled_key = event_key
        if S.active_box_job:
            if str(job) == str(S.active_box_job):
                S.log(f"Ignored duplicate box-filled event for active job {job}.")
                return
            already_queued = any(str(item.get("jobId", "")) == str(job) for item in S.box_queue)
            if not already_queued:
                queued = dict(data)
                queued.update({"event": data.get("event", meta["boxFilledEvent"]), "jobId": job, "color": color, "box": meta["box"], "recipe": recipe_name})
                S.box_queue.append(queued)
                S.box_states[color] = "QUEUED_WAITING_ARM"
                S.next_action = f"{S.active_box_name} is active. {meta['box']} is queued next for arm recipe {recipe_name}."
            queue_position = next((i + 1 for i, item in enumerate(S.box_queue) if str(item.get("jobId", "")) == str(job)), len(S.box_queue))
            active_job = S.active_box_job
            active_box = S.active_box_name
            if already_queued:
                S.log(f"Ignored duplicate queued box-filled event {event_key}.")
            else:
                S.event("COORDINATOR", f"{meta['box']} queued behind {active_box}", {"jobId": job, "activeJobId": active_job, "queuePosition": queue_position, "recipe": recipe_name})
            return
        S.operation_generation += 1
        generation = S.operation_generation
        S.current_job = job
        S.current_package = meta["boxFilledEvent"]
        S.active_box_color = color
        S.active_box_name = meta["box"]
        S.active_box_recipe = recipe_name
        S.active_box_job = job
        S.system = meta["boxFilledEvent"]
        S.mission_state = "BOX_FILLED"
        S.box_states[color] = "FILLED_WAITING_ARM"
        delay_arm = int(meta.get("delayBeforeArmMs", 0))
        S.next_action = f"{meta['box']} is full. Arm recipe {recipe_name} will load it onto the AMR after {delay_arm} ms."
        auto = bool(S.config.get("auto_coordinate", True))

    if not auto:
        with S.lock:
            S.system = "MONITOR_ONLY"
            S.mission_state = "BOX_FILLED_MONITOR_ONLY"
            S.next_action = f"{meta['box']} is full. Auto coordinator is OFF; the arm was not started."
        S.event("COORDINATOR", "Monitor-only mode blocked arm load", {"jobId": job, "box": meta["box"]})
        return

    if not select_recipe_by_name(recipe_name):
        S.event("COORDINATOR", f"no stored arm recipe for {meta['box']} ({recipe_name})")
        with S.lock:
            S.system = "NO MATCHING ARM RECIPE"
            S.mission_state = "ARM_RECIPE_MISSING"
            S.box_states[color] = "ARM_RECIPE_MISSING"
            S.next_action = f"Arm recipe {recipe_name} is not loaded. Refresh recipes, then press Retry Active Arm Job."
        return

    publish(TOPIC["coordinator_event"], {
        "event": "BOX_FILLED_ACCEPTED",
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "recipe": recipe_name,
        "pickup": meta["pickup"],
        }, retain=False)

    delay_arm = int(meta.get("delayBeforeArmMs", 0))
    S.event("COORDINATOR", "Arm load command scheduled", {"flow": meta.get("flowName"), "delayBeforeArmMs": delay_arm, "armRecipe": recipe_name})
    schedule_delayed(delay_arm, execute_arm_recipe_start, color, recipe_name, job, generation)


def publish_amr_delivery_command(color: str, data: dict[str, Any] | None = None, generation: int | None = None):
    color = normalize_color(color)
    meta = meta_for_color(color)
    if not meta:
        return
    job = data.get("jobId", S.current_job) if data else S.current_job
    with S.lock:
        if generation is not None:
            if generation != S.operation_generation or job != S.active_box_job:
                S.log(f"Cancelled stale AMR delivery command for {job}.")
                return
        elif S.active_box_job and job != S.active_box_job:
            S.log(f"Cancelled AMR delivery command for inactive job {job}.")
            return
    payload = {
        "cmd": "DELIVER_BOX_TO_TARGET",
        "flowName": meta.get("flowName", ""),
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "pickup": "ARM_STATION",
        "recipeIndex": int(meta.get("amrRecipeIndex", 0 if color == "RED" else 1 if color == "GREEN" else 2)),
        "recipeName": meta.get("amrRecipeName", ""),
        "destination": meta.get("target", f"TARGET_{color}"),
        "message": meta["armLoadedEvent"],
        "robot": "coordinator",
        "state": "COMMAND_SENT",
    }
    command_id = S.add_pending_ack("amr", "DELIVER_BOX_TO_TARGET", ["AMR_DELIVERY_COMMAND_ACCEPTED", "AMR_COMMAND_ACCEPTED"], 8.0, payload, TOPIC["amr_cmd"])
    payload["commandId"] = command_id
    S.seen("amr", "COMMAND_SENT", payload)
    S.event("COORDINATOR", f"AMR command: deliver {meta['box']} to {payload['destination']}", payload)
    with S.lock:
        S.mission_state = "WAITING_AMR_ACK"
        S.system = "WAITING_AMR_ACK"
        S.box_states[color] = "LOADED_WAITING_AMR"
        S.next_action = f"Waiting for AMR to accept delivery of {meta['box']} to {payload['destination']}."
    if not publish(TOPIC["amr_cmd"], payload, retain=False):
        with S.lock:
            S.pending_acks.pop(command_id, None)
            S.system = "AMR COMMAND FAILED"
            S.mission_state = "FAULT"
            S.next_action = "AMR command could not be published because MQTT is offline."


def publish_amr_pick_command(color: str, data: dict[str, Any] | None = None, generation: int | None = None):
    """Compatibility wrapper for integrations still calling the old function name."""
    publish_amr_delivery_command(color, data, generation)


def emit_amr_event(payload: dict[str, Any]):
    publish(TOPIC["amr_event"], payload, retain=False)
    handle_amr_event(payload)


def simulate_amr_delivery(color: str, job: str | None = None, generation: int | None = None):
    color = normalize_color(color)
    meta = meta_for_color(color)
    if not meta:
        return

    ensure_box_keys(color)
    with S.lock:
        if S.mission_state == "STOPPED":
            S.log("Ignored AMR simulation request while STOPPED. Reset the mission to resume.")
            return
        delay = float(S.config.get("amr_step_delay_s", 2.0))
        if generation is None:
            generation = S.operation_generation
        if not job:
            S.job_number += 1
            job = f"JOB_{S.job_number:03d}"
            S.current_job = job
        S.active_box_color = color
        S.active_box_name = meta["box"]
        S.active_box_recipe = recipe_for_box_color(color)
        S.active_box_job = job
        S.current_package = meta["armLoadedEvent"]
        S.box_states[color] = "AMR_DELIVERING"
        S.system = "AMR DELIVERY SIM RUNNING"
        S.next_action = f"Simulating delivery of {meta['box']} to {meta.get('target', f'TARGET_{color}')}."

    def still_active() -> bool:
        with S.lock:
            return generation == S.operation_generation and job == S.active_box_job

    S.event("AMR SIM", f"started simulated target delivery for {meta['box']}")
    if not still_active():
        return
    emit_amr_event({
        "event": "AMR_DELIVERY_COMMAND_ACCEPTED",
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "destination": meta.get("target", f"TARGET_{color}"),
        "source": "amr_sim",
    })
    time.sleep(delay)
    if not still_active():
        return
    emit_amr_event({
        "event": "ARRIVED_AT_TARGET",
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "destination": meta.get("target", f"TARGET_{color}"),
        "source": "amr_sim",
    })
    time.sleep(delay)
    if not still_active():
        return
    emit_amr_event({
        "event": meta["deliveredEvent"],
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "destination": meta.get("target", f"TARGET_{color}"),
        "source": "amr_sim",
    })


def simulate_amr_flow(color: str, job: str | None = None, generation: int | None = None):
    """Compatibility alias: the AMR now simulates only final-target delivery."""
    simulate_amr_delivery(color, job, generation)


def execute_arm_recipe_start(color: str, recipe_name: str, job: str, generation: int):
    meta = meta_for_color(color)
    if not meta:
        return
    with S.lock:
        if generation != S.operation_generation or job != S.active_box_job:
            S.log(f"Cancelled stale arm start for {job}.")
            return
    if not select_recipe_by_name(recipe_name):
        with S.lock:
            S.system = "NO MATCHING ARM RECIPE"
            S.mission_state = "FAULT"
            S.next_action = f"Arm recipe {recipe_name} not found."
        S.event("COORDINATOR", f"cannot start arm: missing recipe {recipe_name}")
        return

    prepare = arm_prepare_for_new_job()
    load = arm_load_job() if prepare["ok"] else {"stage": "load", "ok": False, "skipped": True}
    wait = arm_wait_amr() if load["ok"] else {"stage": "wait-amr", "ok": False, "skipped": True}
    start = arm_amr_arrived() if wait["ok"] else {"stage": "start", "ok": False, "skipped": True}
    result = {"recipe": recipe_name, "jobId": job, "prepare": prepare, "load": load, "waitAmr": wait, "start": start}
    S.event("ARM", "recipe trigger result", result)
    with S.lock:
        if start["ok"]:
            S.box_states[color] = "ARM_LOADING_AMR"
            S.mission_state = "ARM_LOADING_AMR"
            S.next_action = f"Arm is loading {meta['box']} onto the AMR."
        else:
            failed = next(step for step in (prepare, load, wait, start) if not step["ok"] and not step.get("skipped"))
            detail = failed.get("clearData") if failed.get("clearAttempted") else failed.get("data")
            detail = detail or {}
            reason = str(detail.get("err") or detail.get("msg") or "").strip()
            if not reason:
                reason = f"HTTP {failed.get('code', 0)} from arm controller."
            stage = str(failed.get("stage", "start")).upper()
            S.system = f"ARM {stage} FAILED"
            S.mission_state = "FAULT"
            S.box_states[color] = "ARM_START_FAILED"
            S.next_action = f"Arm {failed.get('stage', 'start')} failed: {reason} Press Retry Active Arm Job after correcting it."


def retry_active_arm_job() -> dict[str, Any]:
    with S.lock:
        color = S.active_box_color
        job = S.active_box_job
        stopped = S.mission_state == "STOPPED"
    if stopped:
        return {"ok": False, "err": "Mission is STOPPED. Reset the mission before retrying."}
    if not color or not job:
        return {"ok": False, "err": "No active filled box job to retry."}

    recipe_name = recipe_for_box_color(color)
    if not recipe_name or not select_recipe_by_name(recipe_name):
        with S.lock:
            S.system = "NO MATCHING ARM RECIPE"
            S.mission_state = "ARM_RECIPE_MISSING"
            S.next_action = f"Arm recipe {recipe_name or '-'} is not loaded."
        return {"ok": False, "err": f"Arm recipe {recipe_name or '-'} is not loaded.", "recipe": recipe_name}

    meta = meta_for_color(color) or {}
    with S.lock:
        S.operation_generation += 1
        generation = S.operation_generation
        S.active_box_recipe = recipe_name
        S.current_job = job
        S.current_package = meta.get("boxFilledEvent", "")
        S.box_states[color] = "FILLED_WAITING_ARM"
        S.system = "ARM RETRY SCHEDULED"
        S.mission_state = "BOX_FILLED"
        delay_arm = int(meta.get("delayBeforeArmMs", 0))
        S.next_action = f"Retrying arm recipe {recipe_name} after {delay_arm} ms."
    S.event("COORDINATOR", "Retrying active arm job", {"jobId": job, "color": color, "recipe": recipe_name})
    schedule_delayed(delay_arm, execute_arm_recipe_start, color, recipe_name, job, generation)
    return {"ok": True, "jobId": job, "color": color, "recipe": recipe_name, "delayBeforeArmMs": delay_arm}


def handle_box_ready_at_arm(data: dict[str, Any]):
    color = normalize_color(data.get("color", ""))
    meta = meta_for_color(color)
    if not meta:
        color2, meta2 = meta_for_ready_event(data.get("event", ""))
        color, meta = color2, meta2
    if not meta:
        S.event("AMR", f"ready ignored: no rule for {data.get('event','')}/{color}")
        return

    job = data.get("jobId", S.current_job)
    recipe_name = data.get("recipe") or recipe_for_box_color(color)

    with S.lock:
        if S.mission_state == "STOPPED":
            S.log(f"Ignored box-ready event for {job} while STOPPED. Reset the mission to resume.")
            return
        if S.active_box_job and job and job != S.active_box_job:
            S.log(f"Ignored box-ready event for stale job {job}; active job is {S.active_box_job}.")
            return
        S.current_job = job
        S.current_package = meta["readyEvent"]
        S.active_box_color = color
        S.active_box_name = meta["box"]
        S.active_box_recipe = recipe_name
        S.active_box_job = job
        S.system = meta["readyEvent"]
        S.mission_state = "BOX_READY_FOR_ARM"
        S.next_action = f"{meta['box']} is docked and ready. Starting arm recipe {recipe_name}."
        auto = bool(S.config.get("auto_coordinate", True))
        generation = S.operation_generation

    publish(TOPIC["coordinator_event"], {
        "event": meta["readyEvent"],
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "recipe": recipe_name,
    }, retain=False)

    if not auto:
        with S.lock:
            S.system = "MONITOR_ONLY"
            S.mission_state = "BOX_READY_MONITOR_ONLY"
            S.next_action = f"{meta['box']} is ready. Auto coordinator is OFF; the arm was not started."
        S.event("COORDINATOR", "Monitor-only mode blocked arm start", {"jobId": job, "recipe": recipe_name})
        return

    delay_arm = int(meta.get("delayBeforeArmMs", 0))
    with S.lock:
        S.next_action = f"{meta['box']} ready at arm station. Starting arm recipe {recipe_name} after {delay_arm} ms."
    S.event("COORDINATOR", "Arm command scheduled", {"flow": meta.get("flowName"), "delayBeforeArmMs": delay_arm, "armRecipe": recipe_name})
    schedule_delayed(delay_arm, execute_arm_recipe_start, color, recipe_name, job, generation)


def handle_arm_loaded_on_amr(data: dict[str, Any] | None = None):
    with S.lock:
        color = S.active_box_color
        box = S.active_box_name
        job = S.active_box_job or S.current_job
        recipe = S.active_box_recipe
        stopped = S.mission_state == "STOPPED"

    if stopped:
        return

    meta = meta_for_color(color)
    if not meta:
        return

    payload = {
        "event": meta["armLoadedEvent"],
        "jobId": job,
        "color": color,
        "box": box,
        "recipe": recipe,
        "destination": meta.get("target", f"TARGET_{color}"),
        "source": "arm",
        "timeMs": int(time.time() * 1000),
    }
    publish(TOPIC["arm_event"], payload, retain=False)
    publish(TOPIC["coordinator_event"], payload, retain=False)
    S.event("ARM", meta["armLoadedEvent"], payload)
    with S.lock:
        S.current_package = meta["armLoadedEvent"]
        S.box_states[color] = "ON_AMR_WAITING_OPERATOR"
        S.system = meta["armLoadedEvent"]
        S.mission_state = "WAITING_OPERATOR_FINISH"
        S.next_action = f"{box} is on the AMR. Manually deliver it and return the AMR, then press Finished Job in the AMR tab."
    S.event("COORDINATOR", "Waiting for operator to finish manual AMR delivery", {"jobId": job, "box": box, "queuedBoxes": len(S.box_queue)})


def start_simulated_amr_delivery(color: str, job: str, generation: int):
    threading.Thread(target=simulate_amr_delivery, args=(color, job, generation), daemon=True).start()


def complete_active_delivery_to_target(data: dict[str, Any] | None = None):
    with S.lock:
        color = S.active_box_color or normalize_color((data or {}).get("color", ""))
        box = S.active_box_name or (data or {}).get("box", "")
        job = S.active_box_job or (data or {}).get("jobId", S.current_job)
        recipe = S.active_box_recipe

    meta = meta_for_color(color)
    if not meta or not job:
        return

    payload = {
        "event": meta["deliveredEvent"],
        "jobId": job,
        "color": color,
        "box": box or meta["box"],
        "recipe": recipe,
        "destination": (data or {}).get("destination") or "MANUAL_AMR_DELIVERY",
        "source": (data or {}).get("source", "operator_finished_job"),
        "timeMs": int(time.time() * 1000),
    }
    publish(TOPIC["coordinator_event"], payload, retain=False)
    publish(TOPIC["conveyor_cmd"], {"cmd": "JOB_DONE", "jobId": job, "color": color}, retain=False)
    S.event("AMR", meta["deliveredEvent"], payload)
    S.mark_done_job(payload)
    reset_box_color(color)
    with S.lock:
        S.system = meta["deliveredEvent"]
        S.mission_state = "JOB_DONE"
        S.next_action = f"{payload['box']} manual delivery finished. Checking the filled-box queue."
    start_next_queued_box()


def complete_active_box_to_shelf():
    """Compatibility wrapper for older callers; completion now means target delivery."""
    complete_active_delivery_to_target()

def handle_conveyor_package(data: dict[str, Any]):
    package = data.get("packageClass", "")
    job = data.get("jobId", "")
    rule = next((r for r in workflow_rules() if r.get("enabled") and r.get("recipe") == package), None)
    if not rule:
        S.event("COORDINATOR", f"legacy package ignored: no workflow rule uses arm recipe {package}")
        with S.lock:
            S.system = "NO MATCHING WORKFLOW"
            S.next_action = "Add a workflow rule whose arm recipe matches packageClass."
        return

    meta = configured_meta_for_color(rule["color"])
    handle_conveyor_box_filled({
        "event": meta["boxFilledEvent"],
        "jobId": job or S.current_job,
        "color": rule["color"],
        "box": meta["box"],
        "recipe": package,
        "source": "legacy_conveyor_package",
    })


def publish_amr_command():
    with S.lock:
        color = S.active_box_color
        job = S.active_box_job or S.current_job
        generation = S.operation_generation
    publish_amr_delivery_command(color, {"jobId": job}, generation)


def handle_amr_event(data: dict[str, Any]):
    event = str(data.get("event", ""))
    color = normalize_color(data.get("color", ""))
    box = data.get("box", "")
    job = data.get("jobId", S.current_job)
    key = f"{event}:{job}:{color}:{box}"
    with S.lock:
        if S.mission_state == "STOPPED":
            S.last_amr_event_key = key
            S.log(f"Ignored AMR event {event or 'EVENT'} while STOPPED.")
            return
        if key == S.last_amr_event_key:
            return
        S.last_amr_event_key = key
        active_job = S.active_box_job

    if active_job and job and job != active_job:
        S.log(f"Ignored AMR event {event or 'EVENT'} for stale job {job}; active job is {active_job}.")
        return

    S.resolve_ack(event, data)

    S.seen("amr", event or "EVENT", data)

    if event in ("AMR_DELIVERY_COMMAND_ACCEPTED", "AMR_COMMAND_ACCEPTED"):
        with S.lock:
            S.system = "AMR_COMMAND_ACCEPTED"
            S.mission_state = "AMR_DELIVERING"
            if color:
                S.box_states[color] = "AMR_DELIVERING"
            S.next_action = f"AMR accepted delivery command for {box or color}."
        S.event("AMR", event, data)
        return

    if event in ("DELIVERY_STARTED", "LEFT_ARM_STATION", "EN_ROUTE_TO_TARGET"):
        with S.lock:
            S.system = event
            S.mission_state = "AMR_DELIVERING"
            if color:
                S.box_states[color] = "AMR_DELIVERING"
            S.next_action = f"AMR is delivering {box or color} to the target."
        S.event("AMR", event, data)
        return

    if event in ("ARRIVED_AT_TARGET", "AMR_ARRIVED_AT_TARGET", "TARGET_REACHED"):
        with S.lock:
            S.system = "AMR AT TARGET"
            S.mission_state = "AMR_AT_TARGET"
            if color:
                S.box_states[color] = "AMR_AT_TARGET"
            S.next_action = "AMR arrived at the target. Waiting for delivery confirmation."
        S.event("AMR", event, data)
        return

    color2, meta2 = meta_for_delivered_event(event)
    if meta2 or event in ("DELIVERED_TO_TARGET", "BOX_DELIVERED", "DELIVERY_COMPLETE"):
        if meta2 and not data.get("color"):
            data["color"] = color2
            color = color2
        S.event("AMR", f"{event} received; waiting for operator Finished Job confirmation", data)
        return

    if event in ("AMR_PICK_COMMAND_ACCEPTED", "PICKED_BOX", "ARRIVED_AT_ARM_STATION", "AMR_arrived_to_arm_station", "Done_Docking", "Docking_Done", "FINISH_DOCKING") or event.endswith("_ready"):
        S.event("AMR", f"ignored legacy AMR-first event: {event}", data)
        return

    S.event("AMR", event or "event", data)



def worker_coordinator_status():
    while True:
        try:
            with S.lock:
                payload = {
                    "robot": "coordinator",
                    "state": S.mission_state,
                    "system": S.system,
                    "jobId": S.current_job,
                    "package": S.current_package,
                    "nextAction": S.next_action,
                    "faultCount": len(S.active_faults()),
                }
            publish(TOPIC["coordinator_status"], payload, retain=True)
        except Exception as exc:
            S.log(f"Coordinator status worker error: {exc}")
        time.sleep(1.0)


def worker_mqtt_watchdog():
    while True:
        try:
            with S.lock:
                connected = S.mqtt_connected
            if not connected:
                start_broker()
                time.sleep(0.3)
                mqtt_connect()
        except Exception as exc:
            S.log(f"MQTT watchdog error: {exc}")
        time.sleep(5.0)


def worker_boot():
    time.sleep(0.4)
    start_broker()
    time.sleep(0.4)
    mqtt_connect()


def worker_recipes():
    while True:
        try:
            refresh_arm_recipes()
        except Exception as exc:
            S.log(f"Recipe worker error: {exc}")
        time.sleep(10.0)


def worker_arm():
    while True:
        try:
            poll_arm_status()
        except Exception as exc:
            S.log(f"Arm worker error: {exc}")
        time.sleep(1.5)


def worker_http_ping():
    while True:
        try:
            with S.lock:
                targets = [
                    ("amr", S.config["amr_ip"]),
                    ("conveyor", S.config["conveyor_ip"]),
                ]
            for node, ip in targets:
                if clean_ip(ip):
                    test_http_node(node, ip, timeout=1.0)
        except Exception as exc:
            S.log(f"HTTP ping worker error: {exc}")
        time.sleep(4.0)



# ============================================================
# Integrated Conveyor OpenCV Pipeline
# ============================================================

CV_LOCK = threading.RLock()
CV = {
    "enabled": True,
    "state": "WAITING_IP",
    "cameraUrl": "",
    "reader": "opencv_videocapture",
    "fps": 0.0,
    "frames": 0,
    "detectedColor": "NO COLOR",
    "detectedCode": "",
    "area": 0,
    "shape": "",
    "packageClass": "",
    "usingLearnedProfile": False,
    "lastServo": "",
    "lastServoCode": "",
    "lastServoAngle": None,
    "lastServoResponse": {},
    "sorterState": "IDLE",
    "lastPublish": "",
    "lastError": "",
    "lastIp": "",
    "rawJpeg": None,
    "processedJpeg": None,
    "latestFrame": None,
}

CV_RESTART = threading.Event()


def cv_blank_frame(text: str) -> bytes:
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(frame, str(text)[:42], (25, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
    ok, jpg = cv2.imencode(".jpg", frame)
    return jpg.tobytes() if ok else b""


def cv_set_blank(text: str):
    jpg = cv_blank_frame(text)
    with CV_LOCK:
        CV["rawJpeg"] = jpg
        CV["processedJpeg"] = jpg


def cv_camera_ip() -> str:
    with S.lock:
        return clean_ip(S.config.get("conveyor_ip", ""))


def cv_stream_url() -> str:
    ip = cv_camera_ip()
    return f"http://{ip}:81/stream" if ip else ""


def cv_servo_angle_url(angle: int) -> str:
    ip = cv_camera_ip()
    return f"http://{ip}:82/servo-test?angle={angle}" if ip else ""


def normalize_sorter_servo_angle(value: Any) -> int:
    try:
        angle = int(round(float(value)))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("angle must be a number from 0 to 180") from exc
    return max(0, min(180, angle))


def sorter_servo_angles() -> dict[str, int]:
    with S.lock:
        saved = dict(S.config.get("sorter_servo_angles", {}))
    angles = {}
    for code, default in DEFAULT_SORTER_SERVO_ANGLES.items():
        try:
            angles[code] = normalize_sorter_servo_angle(saved.get(code, default))
        except ValueError:
            angles[code] = default
    return angles


def cv_encode(frame, quality: int = 92):
    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return jpg.tobytes() if ok else None


CV_MIN_AREA_PIXELS = 500
CV_MIN_AREA_RATIO = 0.003
CV_MIN_FILL_RATIO = 0.40
CV_MIN_COLOR_COVERAGE = 0.70
CV_BORDER_MARGIN = 4
CV_MIN_MEAN_SATURATION = 120
CV_MIN_MEAN_VALUE = 70
CV_MIN_CHANNEL_DIFFERENCE = 20
CV_MIN_CHANNEL_RATIO = 1.12
CV_CONFIRM_FRAMES = 2
CV_CANDIDATE_MISS_FRAMES = 2
CV_REARM_FRAMES = 6
CV_CALIBRATION_ROI = (220, 140, 420, 340)
CV_CALIBRATION_MIN_SATURATION = 50
CV_CALIBRATION_MIN_VALUE = 45
CV_CALIBRATION_MIN_PIXELS = 1200
CV_CALIBRATION_HUE_BANDS = {
    "RED": ((0, 18), (160, 179)),
    "GREEN": ((25, 100),),
    "BLUE": ((85, 145),),
}

CV_COLOR_RULES = (
    {
        "code": "R",
        "name": "RED",
        "drawColor": (0, 0, 255),
        "channel": 2,
        "ranges": (
            ((0, 100, 70), (10, 255, 255)),
            ((170, 100, 70), (179, 255, 255)),
        ),
        "minMeanSaturation": 100,
        "minMeanValue": 70,
    },
    {
        "code": "G",
        "name": "GREEN",
        "drawColor": (0, 255, 0),
        "channel": 1,
        "ranges": (((40, 110, 45), (85, 255, 255)),),
        "minMeanSaturation": 110,
        "minMeanValue": 50,
        "minChannelDifference": 18,
        "minChannelRatio": 1.25,
    },
    {
        "code": "B",
        "name": "BLUE",
        "drawColor": (255, 0, 0),
        "channel": 0,
        "ranges": (((100, 140, 70), (130, 255, 255)),),
    },
)

CV_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
CV_DRAW_COLOR_BY_CODE = {
    "R": (0, 0, 255),
    "G": (0, 255, 0),
    "B": (255, 0, 0),
}


def cv_new_sorter_cycle_state() -> dict[str, Any]:
    return {
        "candidateCode": "",
        "candidateName": "",
        "candidateCount": 0,
        "candidateMisses": 0,
        "positionCode": "",
        "positionName": "",
        "activeCode": "",
        "activeName": "",
        "objectLocked": False,
        "clearFrames": 0,
    }


def cv_sorter_cycle_step(
    state: dict[str, Any],
    raw_code: str,
    raw_name: str,
) -> dict[str, Any]:
    raw_code = str(raw_code or "").strip().upper()
    raw_name = str(raw_name or "NO COLOR").strip().upper()
    action = {
        "sortCode": "",
        "sortName": "",
        "displayCode": "",
        "displayName": "NO COLOR",
        "statusName": "NO COLOR",
        "sorterState": "IDLE",
        "rearmed": False,
    }

    position_code = str(state.get("positionCode", ""))
    position_name = str(state.get("positionName", ""))
    if state.get("objectLocked"):
        active_code = str(state.get("activeCode", position_code))
        active_name = str(state.get("activeName", position_name or "NO COLOR"))
        if raw_code:
            state["clearFrames"] = 0
        else:
            state["clearFrames"] = int(state.get("clearFrames", 0)) + 1

        action.update({
            "displayCode": active_code,
            "displayName": active_name,
            "statusName": f"SORTING {active_name} - WAITING FOR CUBE TO CLEAR",
            "sorterState": f"WAITING AT {position_name}",
        })

        if int(state.get("clearFrames", 0)) >= CV_REARM_FRAMES:
            state["objectLocked"] = False
            state["activeCode"] = ""
            state["activeName"] = ""
            state["clearFrames"] = 0
            state["candidateCode"] = ""
            state["candidateName"] = ""
            state["candidateCount"] = 0
            state["candidateMisses"] = 0
            action.update({
                "displayCode": "",
                "displayName": "NO COLOR",
                "statusName": f"NO COLOR - READY, SORTER AT {position_name}",
                "sorterState": f"READY AT {position_name}",
                "rearmed": True,
            })
        return action

    if raw_code:
        state["candidateMisses"] = 0
        if raw_code == state.get("candidateCode"):
            state["candidateCount"] = int(state.get("candidateCount", 0)) + 1
        else:
            state["candidateCode"] = raw_code
            state["candidateName"] = raw_name
            state["candidateCount"] = 1

        count = int(state["candidateCount"])
        if count >= CV_CONFIRM_FRAMES:
            state["positionCode"] = raw_code
            state["positionName"] = raw_name
            state["activeCode"] = raw_code
            state["activeName"] = raw_name
            state["objectLocked"] = True
            state["clearFrames"] = 0
            action.update({
                "sortCode": raw_code,
                "sortName": raw_name,
                "displayCode": raw_code,
                "displayName": raw_name,
                "statusName": f"SORTING {raw_name} - STAYING AT {raw_name}",
                "sorterState": f"WAITING AT {raw_name}",
            })
        else:
            action["statusName"] = f"VERIFYING {raw_name} {count}/{CV_CONFIRM_FRAMES}"
            action["sorterState"] = "VERIFYING"
        return action

    if state.get("candidateCode"):
        state["candidateMisses"] = int(state.get("candidateMisses", 0)) + 1
        if int(state["candidateMisses"]) <= CV_CANDIDATE_MISS_FRAMES:
            action["statusName"] = (
                f"VERIFYING {state.get('candidateName', 'COLOR')} "
                f"{state.get('candidateCount', 0)}/{CV_CONFIRM_FRAMES}"
            )
            action["sorterState"] = "VERIFYING"
            return action

    state["candidateCode"] = ""
    state["candidateName"] = ""
    state["candidateCount"] = 0
    state["candidateMisses"] = 0
    if position_name:
        action["statusName"] = f"NO COLOR - READY, SORTER AT {position_name}"
        action["sorterState"] = f"READY AT {position_name}"
    return action


def cv_saved_color_profiles() -> dict[str, Any]:
    with S.lock:
        raw = S.config.get("cv_color_profiles", {})
        return dict(raw) if isinstance(raw, dict) else {}


def cv_detection_rules():
    rules = [dict(rule) for rule in CV_COLOR_RULES]
    base_by_name = {rule["name"]: rule for rule in CV_COLOR_RULES}

    for color, profile in cv_saved_color_profiles().items():
        color = str(color).upper()
        base = base_by_name.get(color)
        ranges = profile.get("ranges", []) if isinstance(profile, dict) else []
        if not base or not isinstance(ranges, list):
            continue
        try:
            learned_ranges = tuple(
                (tuple(int(v) for v in lower), tuple(int(v) for v in upper))
                for lower, upper in ranges
            )
        except Exception:
            continue
        if not learned_ranges:
            continue

        learned = dict(base)
        learned.update({
            "ranges": learned_ranges,
            "learned": True,
            "minMeanSaturation": max(
                CV_CALIBRATION_MIN_SATURATION,
                int(profile.get("minMeanSaturation", CV_CALIBRATION_MIN_SATURATION)),
            ),
            "minMeanValue": max(
                CV_CALIBRATION_MIN_VALUE,
                int(profile.get("minMeanValue", CV_CALIBRATION_MIN_VALUE)),
            ),
            "skipChannelDominance": True,
        })
        rules.append(learned)

    return rules


def cv_make_color_mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        range_mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.bitwise_or(mask, range_mask)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, CV_MORPH_KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CV_MORPH_KERNEL)


def cv_target_channel_is_dominant(
    mean_bgr,
    target_channel: int,
    min_difference: float = CV_MIN_CHANNEL_DIFFERENCE,
    min_ratio: float = CV_MIN_CHANNEL_RATIO,
) -> bool:
    target = mean_bgr[target_channel]
    strongest_other = max(
        mean_bgr[index] for index in range(3) if index != target_channel
    )
    return (
        target - strongest_other >= min_difference
        and target >= strongest_other * min_ratio
    )


def cv_detect_color(frame):
    # Strict color classifier shared with the Yasin vision implementation.
    frame = cv2.resize(frame, (640, 480))
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    min_area = max(
        CV_MIN_AREA_PIXELS,
        frame.shape[0] * frame.shape[1] * CV_MIN_AREA_RATIO,
    )

    best = {
        "code": "",
        "name": "NO COLOR",
        "area": 0,
        "box": None,
        "drawColor": (0, 255, 255),
        "coverage": 0.0,
    }

    for rule in cv_detection_rules():
        mask = cv_make_color_mask(hsv, rule["ranges"])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if (
                x <= CV_BORDER_MARGIN
                or y <= CV_BORDER_MARGIN
                or x + w >= frame.shape[1] - CV_BORDER_MARGIN
                or y + h >= frame.shape[0] - CV_BORDER_MARGIN
            ):
                continue
            if area / float(w * h) < CV_MIN_FILL_RATIO:
                continue

            contour_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
            contour_pixels = max(1, cv2.countNonZero(contour_mask))
            color_coverage = cv2.countNonZero(
                cv2.bitwise_and(mask, contour_mask)
            ) / float(contour_pixels)
            if color_coverage < CV_MIN_COLOR_COVERAGE:
                continue

            mean_hsv = cv2.mean(hsv, mask=contour_mask)
            if (
                mean_hsv[1] < rule.get("minMeanSaturation", CV_MIN_MEAN_SATURATION)
                or mean_hsv[2] < rule.get("minMeanValue", CV_MIN_MEAN_VALUE)
            ):
                continue

            mean_bgr = cv2.mean(blurred, mask=contour_mask)[:3]
            if not rule.get("skipChannelDominance") and not cv_target_channel_is_dominant(
                mean_bgr,
                rule["channel"],
                rule.get("minChannelDifference", CV_MIN_CHANNEL_DIFFERENCE),
                rule.get("minChannelRatio", CV_MIN_CHANNEL_RATIO),
            ):
                continue

            if area > best["area"]:
                best = {
                    "code": rule["code"],
                    "name": rule["name"],
                    "area": int(area),
                    "box": (x, y, w, h),
                    "drawColor": rule["drawColor"],
                    "coverage": color_coverage,
                    "learnedProfile": bool(rule.get("learned")),
                }

    return frame, best


def cv_recipe_for_color(color_name: str) -> str:
    mapping = {
        "RED": "RED_SQUARE_TO_SHELF_A",
        "GREEN": "GREEN_SQUARE_TO_SHELF_C",
        "BLUE": "BLUE_SQUARE_TO_SHELF_B",
    }
    return mapping.get(str(color_name).upper(), "")


def cv_make_package(color_name: str):
    recipe = cv_recipe_for_color(color_name)
    return {
        "jobId": f"JOB_{int(time.time() * 1000) % 100000000:08d}",
        "packageClass": recipe,
        "color": color_name,
        "shape": "SQUARE",
        "destination": recipe.split("_TO_")[-1] if "_TO_" in recipe else "",
        "event": "PACKAGE_READY",
        "source": "internal_opencv_dashboard",
        "cameraIp": cv_camera_ip(),
        "timeMs": int(time.time() * 1000),
    }


def cv_draw(frame, result, fps: float):
    color_name = result.get("statusName", result["name"])
    area = result["area"]
    box = result["box"]
    draw_color = result["drawColor"]

    if box is not None:
        x, y, w, h = box
        cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
        cv2.putText(frame, f"{color_name} area={area}", (x, max(25, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2)

    cv2.putText(frame, f"Detected: {color_name}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    confirmed_name = result["name"]
    cv2.putText(frame, f"Package: {cv_recipe_for_color(confirmed_name) if result['code'] else '-'}", (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(frame, f"FPS {fps:.1f} | Internal OpenCV pipeline", (20, 455),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    return frame


def cv_process_frame(frame, fps: float, result=None):
    raw_frame = cv2.resize(
        frame,
        (640, 480),
        interpolation=cv2.INTER_AREA if frame.shape[1] > 640 else cv2.INTER_CUBIC,
    )
    raw = cv_encode(raw_frame)
    if result is None:
        processed_frame, result = cv_detect_color(frame)
    else:
        processed_frame = raw_frame.copy()
    processed_frame = cv_draw(processed_frame, result, fps)
    processed = cv_encode(processed_frame)

    confirmed_name = result["name"]
    status_name = result.get("statusName", confirmed_name)
    recipe = cv_recipe_for_color(confirmed_name) if result["code"] else ""

    with CV_LOCK:
        CV["latestFrame"] = raw_frame.copy()
        if raw:
            CV["rawJpeg"] = raw
        if processed:
            CV["processedJpeg"] = processed
        CV["state"] = "RUNNING"
        CV["detectedColor"] = status_name
        CV["detectedCode"] = result["code"]
        CV["area"] = result["area"]
        CV["shape"] = "SQUARE" if result["code"] else ""
        CV["packageClass"] = recipe
        CV["usingLearnedProfile"] = bool(result.get("learnedProfile"))
        CV["fps"] = round(fps, 1)
        CV["frames"] = int(CV["frames"]) + 1
        CV["lastError"] = ""

    return result


def cv_hue_in_expected_band(color: str, hue: int) -> bool:
    return any(lower <= hue <= upper for lower, upper in CV_CALIBRATION_HUE_BANDS[color])


def cv_hue_ranges(center: int, radius: int, lower_s: int, lower_v: int):
    low = center - radius
    high = center + radius
    if low < 0:
        return [
            [[0, lower_s, lower_v], [high, 255, 255]],
            [[180 + low, lower_s, lower_v], [179, 255, 255]],
        ]
    if high > 179:
        return [
            [[low, lower_s, lower_v], [179, 255, 255]],
            [[0, lower_s, lower_v], [high - 180, 255, 255]],
        ]
    return [[[low, lower_s, lower_v], [high, 255, 255]]]


def cv_create_color_profile(frame, color: str) -> dict[str, Any]:
    color = normalize_color(color)
    if color not in ("RED", "GREEN", "BLUE"):
        raise ValueError("Color must be RED, GREEN, or BLUE.")

    frame = cv2.resize(frame, (640, 480))
    x1, y1, x2, y2 = CV_CALIBRATION_ROI
    roi = cv2.GaussianBlur(frame[y1:y2, x1:x2], (5, 5), 0)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    pixels = hsv.reshape(-1, 3)
    valid = pixels[
        (pixels[:, 1] >= CV_CALIBRATION_MIN_SATURATION)
        & (pixels[:, 2] >= CV_CALIBRATION_MIN_VALUE)
    ]
    if len(valid) < CV_CALIBRATION_MIN_PIXELS:
        raise ValueError("Calibration target is too dark or not colorful enough. Move the cube into the center box and improve lighting.")

    hue_hist = np.bincount(valid[:, 0], weights=valid[:, 1], minlength=180)
    hue_center = int(np.argmax(hue_hist))
    if not cv_hue_in_expected_band(color, hue_center):
        raise ValueError(f"The camera sample does not look like {color}. Check lighting and place only the {color} cube in the center box.")

    hue_distance = np.minimum(
        np.abs(valid[:, 0].astype(np.int16) - hue_center),
        180 - np.abs(valid[:, 0].astype(np.int16) - hue_center),
    )
    cluster = valid[hue_distance <= 14]
    if len(cluster) < CV_CALIBRATION_MIN_PIXELS:
        raise ValueError("Could not isolate one stable cube color in the center box.")

    radius = int(np.clip(np.percentile(hue_distance[hue_distance <= 14], 95) + 4, 7, 18))
    lower_s = int(np.clip(np.percentile(cluster[:, 1], 10) - 20, CV_CALIBRATION_MIN_SATURATION, 230))
    lower_v = int(np.clip(np.percentile(cluster[:, 2], 10) - 12, CV_CALIBRATION_MIN_VALUE, 230))
    profile = {
        "color": color,
        "ranges": cv_hue_ranges(hue_center, radius, lower_s, lower_v),
        "hsvCenter": [
            hue_center,
            int(np.median(cluster[:, 1])),
            int(np.median(cluster[:, 2])),
        ],
        "minMeanSaturation": lower_s,
        "minMeanValue": lower_v,
        "samplePixels": int(len(cluster)),
        "learnedAt": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    return profile


def cv_calibrate_color(color: str) -> dict[str, Any]:
    with CV_LOCK:
        frame = CV.get("latestFrame")
        frame = frame.copy() if frame is not None else None
    if frame is None:
        raise ValueError("No camera frame is available yet. Wait for the processed camera feed.")

    profile = cv_create_color_profile(frame, color)
    with S.lock:
        profiles = dict(S.config.get("cv_color_profiles", {}))
        profiles[profile["color"]] = profile
        S.config["cv_color_profiles"] = profiles
    S.save_config()
    S.log(f"Learned camera color profile for {profile['color']}: {profile['hsvCenter']}")
    return profile


def cv_move_servo_angle(angle: Any, code: str = "MANUAL") -> dict[str, Any]:
    code = str(code or "MANUAL").strip().upper()
    try:
        requested_angle = normalize_sorter_servo_angle(angle)
    except ValueError as exc:
        result = {"ok": False, "code": code, "err": str(exc)}
        with CV_LOCK:
            CV["lastServo"] = result["err"]
            CV["lastServoCode"] = code
            CV["lastServoResponse"] = result
        return result

    url = cv_servo_angle_url(requested_angle)
    if not url:
        result = {
            "ok": False,
            "code": code,
            "requestedAngle": requested_angle,
            "err": "No conveyor IP configured.",
        }
        with CV_LOCK:
            CV["lastServo"] = result["err"]
            CV["lastServoCode"] = code
            CV["lastServoResponse"] = result
        return result
    try:
        response = requests.get(url, timeout=1.0)
        try:
            data = response.json()
        except Exception:
            data = {"raw": response.text[:300]}
        ok = bool(response.ok and isinstance(data, dict) and data.get("ok") is True)
        angle = data.get("servoAngle") if isinstance(data, dict) else None
        result = {
            "ok": ok,
            "code": code,
            "requestedAngle": requested_angle,
            "status": response.status_code,
            "url": url,
            "data": data,
        }
        with CV_LOCK:
            CV["lastServo"] = f"{code} -> {angle} deg" if ok and angle is not None else f"{code} HTTP {response.status_code}"
            CV["lastServoCode"] = code
            CV["lastServoAngle"] = angle
            CV["lastServoResponse"] = result
        if not ok:
            S.log(f"Conveyor servo command failed: {result}")
        return result
    except Exception as exc:
        result = {
            "ok": False,
            "code": code,
            "requestedAngle": requested_angle,
            "url": url,
            "err": str(exc),
        }
        with CV_LOCK:
            CV["lastServo"] = f"Servo failed: {exc}"
            CV["lastServoCode"] = code
            CV["lastServoResponse"] = result
        S.log(f"Conveyor servo command failed: {result}")
        return result


def cv_send_servo(code: str, allow_center: bool = False) -> dict[str, Any]:
    code = SORTER_SERVO_TARGET_CODES.get(str(code or "").strip().upper(), "")
    if not code:
        return {"ok": False, "code": "", "err": "Servo target must be RED, GREEN, BLUE, or CENTER."}
    if code == "CENTER" and not allow_center:
        result = {
            "ok": False,
            "code": code,
            "err": "Automatic CENTER movement is disabled. CENTER is manual calibration only.",
        }
        with CV_LOCK:
            CV["lastServo"] = "Blocked automatic CENTER"
            CV["lastServoCode"] = code
            CV["lastServoResponse"] = result
        S.log("Blocked automatic sorter CENTER command.")
        return result
    return cv_move_servo_angle(sorter_servo_angles()[code], code)


def cv_apply_camera_quality(ip: str):
    """Best-effort ESP32-CAM tuning. Unsupported camera firmware safely ignores it."""
    settings = (
        ("framesize", 8),  # VGA 640x480
        ("quality", 10),   # Lower ESP32 JPEG quality value means a clearer image
    )
    applied = []
    for name, value in settings:
        try:
            response = requests.get(
                f"http://{ip}/control?var={name}&val={value}",
                timeout=0.8,
            )
            if response.ok:
                applied.append(name)
        except Exception:
            continue
    if applied:
        S.log(f"ESP32-CAM quality settings applied: {', '.join(applied)}.")


def cv_publish_package(color_name: str):
    # Legacy name kept. In v12 this counts one mini cube instead of publishing a package directly.
    record_conveyor_cube(color_name)



def cv_mjpeg_frame_reader(url: str):
    """Read ESP32-CAM MJPEG stream with requests and decode frames manually.
    This avoids Windows OpenCV/FFMPEG stream issues and keeps the camera inside the main dashboard.
    """
    headers = {
        "User-Agent": "RobotCellInternalOpenCV/1.0",
        "Accept": "multipart/x-mixed-replace,image/jpeg,*/*",
        "Connection": "close",
    }
    with requests.get(url, stream=True, timeout=(4, 20), headers=headers) as r:
        r.raise_for_status()
        data = b""
        for chunk in r.iter_content(chunk_size=8192):
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

def cv_worker_opencv():
    S.log("Internal OpenCV worker started: requests MJPEG reader.")
    last_ip = ""
    last_tuned_ip = ""
    sorter_cycle = cv_new_sorter_cycle_state()
    frame_count = 0
    fps_t0 = time.time()
    fps = 0.0

    cv_set_blank("Set Conveyor IP and wait for camera")

    while True:
        try:
            if CV_RESTART.is_set():
                CV_RESTART.clear()
                last_ip = ""
                last_tuned_ip = ""
                sorter_cycle = cv_new_sorter_cycle_state()
                cv_set_blank("Restarting OpenCV pipeline...")

            ip = cv_camera_ip()
            if not ip:
                with CV_LOCK:
                    CV["state"] = "WAITING_IP"
                    CV["reader"] = "requests_mjpeg"
                    CV["lastError"] = "No conveyor IP"
                cv_set_blank("No conveyor IP set")
                time.sleep(1.0)
                continue

            url = cv_stream_url()
            if ip != last_ip:
                S.log(f"Internal OpenCV connecting to {url}")
                last_ip = ip
            if ip != last_tuned_ip:
                cv_apply_camera_quality(ip)
                last_tuned_ip = ip

            with CV_LOCK:
                CV["state"] = "CONNECTING"
                CV["reader"] = "requests_mjpeg"
                CV["cameraUrl"] = url
                CV["lastIp"] = ip
                CV["lastError"] = ""
            cv_set_blank("Connecting to ESP32-CAM stream...")

            for raw_jpg, frame in cv_mjpeg_frame_reader(url):
                if CV_RESTART.is_set():
                    break

                now = time.time()
                frame_count += 1
                if now - fps_t0 >= 1.0:
                    fps = frame_count / (now - fps_t0)
                    frame_count = 0
                    fps_t0 = now

                # Always publish raw frame immediately.
                with CV_LOCK:
                    CV["state"] = "RUNNING"
                    CV["lastError"] = ""
                    CV["rawJpeg"] = raw_jpg

                _, detected = cv_detect_color(frame)
                raw_code = detected["code"]
                raw_name = detected["name"]
                result = dict(detected)

                cycle = cv_sorter_cycle_step(
                    sorter_cycle,
                    raw_code,
                    raw_name,
                )
                display_code = cycle["displayCode"]
                if display_code:
                    if raw_code != display_code:
                        result["box"] = None
                        result["area"] = 0
                        result["coverage"] = 0.0
                    result["code"] = display_code
                    result["name"] = cycle["displayName"]
                    result["drawColor"] = CV_DRAW_COLOR_BY_CODE[display_code]
                else:
                    result["code"] = ""
                    result["name"] = "NO COLOR"
                result["statusName"] = cycle["statusName"]

                result = cv_process_frame(frame, fps, result)
                with CV_LOCK:
                    CV["sorterState"] = cycle["sorterState"]

                if cycle["sortCode"]:
                    cv_send_servo(cycle["sortCode"])
                    record_conveyor_cube(cycle["sortName"])

        except Exception as exc:
            with CV_LOCK:
                CV["state"] = "CAMERA_ERROR"
                CV["reader"] = "requests_mjpeg"
                CV["lastError"] = str(exc)
            cv_set_blank("Camera error. Check Conveyor IP.")
            S.log(f"Internal OpenCV camera error: {exc}")
            time.sleep(1.2)


def cv_status_snapshot():
    with CV_LOCK:
        data = {k: v for k, v in CV.items() if k not in ("rawJpeg", "processedJpeg", "latestFrame")}
    data["calibrationRoi"] = list(CV_CALIBRATION_ROI)
    data["colorProfiles"] = cv_saved_color_profiles()
    return data



@app.get("/")
def index():
    html = Path(__file__).with_name("index.html").read_text(encoding="utf-8")
    return Response(html, mimetype="text/html")



# ============================================================
# HTTP Auto Discovery
# ============================================================
def classify_device_by_http(ip: str) -> tuple[str | None, dict[str, Any]]:
    """
    Finds known subsystem devices by probing their HTTP endpoints.
    This lets the system work even if Windows hotspot gives random DHCP IPs.

    Arm v10.22:
      /job/status
      /status
      /net
      /recipe/library

    AMR:
      /
      /status

    Conveyor/ESP32-CAM:
      /status
      /capture
      :81/stream usually not checked here because scanner uses port 80.
    """
    info: dict[str, Any] = {"ip": ip, "score": 0, "checks": []}

    # Arm checks
    try:
        r = requests.get(f"http://{ip}/job/status", timeout=0.35)
        txt = r.text[:500]
        if r.status_code == 200 and ("arm" in txt.lower() or "job" in txt.lower() or "recipe" in txt.lower() or "state" in txt.lower()):
            info["checks"].append("/job/status")
            info["jobStatus"] = txt
            return "arm", info
    except Exception:
        pass

    try:
        r = requests.get(f"http://{ip}/recipe/library", timeout=0.35)
        txt = r.text[:500]
        if r.status_code == 200 and ("recipe" in txt.lower() or "library" in txt.lower() or "count" in txt.lower()):
            info["checks"].append("/recipe/library")
            info["recipeLibrary"] = txt
            return "arm", info
    except Exception:
        pass

    try:
        r = requests.get(f"http://{ip}/status", timeout=0.35)
        txt = r.text[:700]
        low = txt.lower()
        info["checks"].append("/status")
        info["status"] = txt

        # AMR status usually has x/y/th/dist/obs fields.
        if any(k in low for k in ['"x"', '"y"', '"th"', '"dist"', '"obs"', "amr"]):
            return "amr", info

        # ESP32-CAM / conveyor often exposes simple status or camera-specific data.
        if any(k in low for k in ["camera", "stream", "conveyor", "servo", "esp32-cam", "cam"]):
            return "conveyor", info
    except Exception:
        pass

    # Root page classification
    try:
        r = requests.get(f"http://{ip}/", timeout=0.35)
        txt = r.text[:1000]
        low = txt.lower()
        info["checks"].append("/")
        info["root"] = txt

        if "scara" in low or "arm" in low or "recipe" in low or "homing" in low:
            return "arm", info
        if "amr" in low or "navigator" in low or "go to goal" in low:
            return "amr", info
        if "camera" in low or "esp32-cam" in low or "stream" in low or "conveyor" in low:
            return "conveyor", info
    except Exception:
        pass

    # Camera capture check last
    try:
        r = requests.get(f"http://{ip}/capture", timeout=0.35)
        ct = str(r.headers.get("content-type", "")).lower()
        if r.status_code == 200 and "image" in ct:
            info["checks"].append("/capture")
            return "conveyor", info
    except Exception:
        pass

    return None, info


def scan_subnet_for_devices(subnet: str = "192.168.137") -> dict[str, Any]:
    found: dict[str, Any] = {}
    start_ms = ms()
    ips = [f"{subnet}.{i}" for i in range(2, 255)]

    with ThreadPoolExecutor(max_workers=48) as ex:
        futures = {ex.submit(classify_device_by_http, ip): ip for ip in ips}
        for fut in as_completed(futures):
            ip = futures[fut]
            try:
                node, info = fut.result()
            except Exception as exc:
                node, info = None, {"ip": ip, "error": str(exc)}
            if node and node not in found:
                found[node] = info

    elapsed = ms() - start_ms
    return {"found": found, "elapsedMs": elapsed, "subnet": subnet}


def apply_discovery_results(result: dict[str, Any]) -> None:
    found = dict(result.get("found", {}))
    changed = []

    with S.lock:
        S.discovery["lastScanMs"] = ms()
        S.discovery["running"] = False
        S.discovery["found"] = found
        S.discovery["lastScanText"] = f"Found {len(found)} device(s) in {result.get('elapsedMs', 0)} ms."

        for node, info in found.items():
            ip = clean_ip(info.get("ip", ""))
            if not ip:
                continue
            key = f"{node}_ip"
            old = S.config.get(key, "")
            S.config[key] = ip
            S.nodes[node]["ip"] = ip
            if old != ip:
                changed.append((node, old, ip))

    if changed:
        S.save_config()
        for node, old, ip in changed:
            S.event("DISCOVERY", f"{node.upper()} IP updated: {old or '-'} -> {ip}")
            S.log(f"Auto discovery updated {node} IP to {ip}")


def discovery_worker(subnet: str):
    try:
        result = scan_subnet_for_devices(subnet)
        apply_discovery_results(result)
    except Exception as exc:
        with S.lock:
            S.discovery["running"] = False
            S.discovery["lastScanMs"] = ms()
            S.discovery["lastScanText"] = f"Discovery failed: {exc}"
        S.log(f"Discovery failed: {exc}")




@app.get("/api/broker/diagnose")
def api_broker_diagnose():
    with S.lock:
        broker_port = int(S.config.get("broker_port", 1883))
        connected = bool(S.mqtt_connected)
        running = bool(S.broker_running)
    local_ok = tcp_port_open("127.0.0.1", broker_port, timeout=0.5)
    esp_ok = tcp_port_open(ROBOT_BROKER_IP, broker_port, timeout=0.7)
    return jsonify({
        "ok": local_ok and connected,
        "dashboardBrokerIp": "127.0.0.1",
        "laptopHotspotBrokerIp": ROBOT_BROKER_IP,
        "port": broker_port,
        "dashboardMqttConnected": connected,
        "brokerRunning": running,
        "local127Reachable": local_ok,
        "robotNetworkBrokerReachable": esp_ok,
        "message": (
            "Dashboard MQTT is OK. robot-network broker access is also OK."
            if (local_ok and esp_ok) else
            "Dashboard local MQTT is OK, but robots cannot reach the broker. Run SETUP_MQTT_BROKER_ADMIN.ps1 as Administrator."
            if local_ok else
            "Local broker is not reachable. Press Start Broker."
        )
    })



@app.get("/api/arm/status")
def api_arm_status_real():
    code, data = arm_get("/job/status", timeout=4)
    if code == 0:
        code, data = arm_get("/status", timeout=3)
    return jsonify({"ok": code == 200, "code": code, "data": data})


@app.get("/api/arm/config")
def api_arm_config_real():
    code, data = arm_get("/config", timeout=5)
    return jsonify({"ok": code == 200, "code": code, "data": data})


@app.post("/api/arm/load-recipe")
def api_arm_load_recipe():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(data.get("index", S.selected_recipe_index if S.selected_recipe_index >= 0 else 0))
    package = str(data.get("packageClass", data.get("recipe", ""))).strip()
    if not package:
        with S.lock:
            if 0 <= idx < len(S.recipes):
                package = str(S.recipes[idx].get("name", f"RECIPE_{idx}"))
            else:
                package = f"RECIPE_{idx}"
    job = str(data.get("jobId", S.current_job or "JOB_MANUAL"))
    path = f"/job/load?recipe={idx}&jobId={quote(job)}&packageClass={quote(package)}"
    code, resp = arm_get(path, timeout=6)
    S.event("ARM", f"HTTP load arm recipe {idx}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp, "recipeIndex": idx, "packageClass": package})


@app.post("/api/arm/start-loaded")
def api_arm_start_loaded():
    code1, resp1 = arm_get("/job/wait-amr", timeout=5)
    wait_ok = code1 == 200 and bool(resp1.get("ok", True))
    code2, resp2 = arm_get("/job/amr-arrived", timeout=6) if wait_ok else (0, {"ok": False, "skip": True})
    S.event("ARM", "HTTP start loaded arm recipe", {"waitAmr": resp1, "amrArrived": resp2})
    return jsonify({"ok": code2 == 200 and bool(resp2.get("ok", True)), "waitCode": code1, "startCode": code2, "wait": resp1, "start": resp2})


@app.post("/api/arm/run-recipe-test")
def api_arm_run_recipe_test():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(data.get("index", 0))
    package = str(data.get("packageClass", f"TEST_RECIPE_{idx}"))
    job = str(data.get("jobId", f"JOB_ARM_TEST_{ms()}"))
    load_path = f"/job/load?recipe={idx}&jobId={quote(job)}&packageClass={quote(package)}"
    code0, resp0 = arm_get(load_path, timeout=6)
    load_ok = code0 == 200 and bool(resp0.get("ok", True))
    code1, resp1 = arm_get("/job/wait-amr", timeout=5) if load_ok else (0, {"ok": False, "skip": True})
    wait_ok = code1 == 200 and bool(resp1.get("ok", True))
    code2, resp2 = arm_get("/job/amr-arrived", timeout=6) if wait_ok else (0, {"ok": False, "skip": True})
    S.event("ARM", f"HTTP run arm recipe test {idx}", {"load": resp0, "wait": resp1, "start": resp2})
    return jsonify({"ok": code2 == 200 and bool(resp2.get("ok", True)), "load": resp0, "wait": resp1, "start": resp2})


def _http_probe(ip: str, paths: list[str], timeout: float = 1.8) -> dict[str, Any]:
    ip = clean_ip(ip)
    if not ip:
        return {"ok": False, "ip": "", "message": "No IP configured", "tested": []}
    tested = []
    for path in paths:
        url = f"http://{ip}{path}"
        try:
            r = requests.get(url, timeout=timeout)
            item = {"url": url, "status": r.status_code}
            try:
                item["json"] = r.json()
            except Exception:
                item["text"] = r.text[:350]
            tested.append(item)
            if 200 <= r.status_code < 300:
                return {"ok": True, "ip": ip, "url": url, "message": f"OK {path}", "tested": tested}
        except Exception as exc:
            tested.append({"url": url, "error": str(exc)})
    return {"ok": False, "ip": ip, "message": "All HTTP probes failed", "tested": tested}


@app.post("/api/check/mqtt")
def api_check_mqtt():
    with S.lock:
        port = int(S.config.get("broker_port", 1883))
        connected = bool(S.mqtt_connected)
        running = bool(S.broker_running)
    local_ok = tcp_port_open("127.0.0.1", port, timeout=0.5)
    laptop_ok = tcp_port_open(ROBOT_BROKER_IP, port, timeout=0.7)
    return jsonify({"ok": bool(local_ok and connected and laptop_ok), "name": "MQTT broker check", "whatThisTests": "Dashboard connection and robot-network access to the same MQTT broker.", "dashboardUses": f"127.0.0.1:{port}", "laptopBrokerAddress": f"{ROBOT_BROKER_IP}:{port}", "brokerRunning": running, "dashboardMqttConnected": connected, "localReachable": local_ok, "robotNetworkReachable": laptop_ok, "fix": "" if laptop_ok else "Run SETUP_MQTT_BROKER_ADMIN.ps1 as Administrator, then retry."})


@app.post("/api/check/arm")
def api_check_arm():
    with S.lock:
        ip = S.config.get("arm_ip", "")
    probes = _http_probe(ip, ["/job/status", "/config", "/", "/status"], timeout=2.5)
    status_code, status_data = arm_get("/job/status", timeout=4)
    code, data = arm_get("/config", timeout=4)
    recipes = data.get("packageRecipes", []) if isinstance(data, dict) else []
    status_ok = status_code == 200 and isinstance(status_data, dict)
    recipe_ok = code == 200 and isinstance(recipes, list)
    return jsonify({"ok": bool(status_ok and recipe_ok), "name": "SCARA arm check", "whatThisTests": "Required arm /job/status and /config recipe endpoints.", "ip": clean_ip(ip), "http": probes, "statusEndpointOk": status_ok, "statusPreview": status_data, "recipeEndpointOk": recipe_ok, "recipeCount": len(recipes) if isinstance(recipes, list) else 0, "configPreview": data})


@app.post("/api/check/amr")
def api_check_amr():
    with S.lock:
        ip = S.config.get("amr_ip", "")
        simulate = bool(S.config.get("simulate_amr", True))
    if simulate:
        return jsonify({"ok": True, "name": "AMR check", "mode": "SIMULATED", "whatThisTests": "AMR simulation mode is enabled. No real AMR HTTP request or MQTT command was sent.", "ip": clean_ip(ip), "mqttCommandPublished": False})
    status_probe = _http_probe(ip, ["/status", "/identity", "/"], timeout=2.0)
    routes_probe = _http_probe(ip, ["/amr/route-recipes", "/amr/recipes"], timeout=3.5)
    mqtt_payload = {"cmd": "STATUS", "commandId": f"CHECK_AMR_STATUS_{ms()}"}
    mqtt_ok = publish(TOPIC["amr_cmd"], mqtt_payload, retain=False)
    return jsonify({"ok": bool(status_probe.get("ok")), "name": "AMR check", "whatThisTests": "AMR HTTP status, AMR route recipe endpoint, and MQTT STATUS command publish.", "ip": clean_ip(ip), "httpStatus": status_probe, "routeRecipes": routes_probe, "mqttCommandPublished": mqtt_ok, "mqttPayload": mqtt_payload})


@app.post("/api/check/conveyor")
def api_check_conveyor():
    with S.lock:
        ip = S.config.get("conveyor_ip", "")
    probe = _http_probe(ip, ["/status", "/", "/capture"], timeout=2.0)
    cv = cv_status_snapshot()
    cv_ok = cv.get("state") == "RUNNING" and int(cv.get("frames", 0) or 0) > 0 and not cv.get("lastError")
    return jsonify({"ok": bool(cv_ok), "name": "Conveyor + vision check", "whatThisTests": "Integrated OpenCV camera pipeline; port-80 device HTTP is reported separately.", "ip": clean_ip(ip), "deviceHttpOk": bool(probe.get("ok")), "http": probe, "opencvOk": bool(cv_ok), "opencv": cv, "streams": {"processed": "/cv/processed.mjpg", "raw": "/cv/raw.mjpg"}})


@app.get("/api/work-order")
def api_work_order():
    snap = S.snap()
    color = snap.get("activeBoxColor", "") or ""
    box = snap.get("activeBoxName", "") or ""
    recipe = snap.get("activeBoxRecipe", "") or ""
    state = snap.get("missionState", "")
    if not box or state in ("IDLE", "BOOTING"):
        title = "No active mission"
        description = "Waiting for a box to reach capacity."
    else:
        title = f"{box} mission"
        description = f"Move {box}: conveyor -> arm loads AMR -> manual delivery -> Finished Job. Arm recipe: {recipe or '-'}"
    return jsonify({"jobId": snap.get("currentJob", ""), "title": title, "description": description, "missionState": state, "color": color, "box": box, "armRecipe": recipe, "nextAction": snap.get("nextAction", "")})



@app.post("/api/settings/clear-ips")
def api_settings_clear_ips():
    with S.lock:
        S.config["arm_ip"] = ""
        S.config["amr_ip"] = ""
        S.config["conveyor_ip"] = ""
        S.nodes["arm"]["ip"] = ""
        S.nodes["amr"]["ip"] = ""
        S.nodes["conveyor"]["ip"] = ""
        for node in ("arm", "amr", "conveyor"):
            S.nodes[node]["state"] = "WAITING_IP"
            S.nodes[node]["http"] = "UNKNOWN"
            S.nodes[node]["httpInfo"] = ""
    S.save_config()
    S.log("Cleared Arm/AMR/Conveyor IP settings.")
    return jsonify({"ok": True, "message": "Robot IPs cleared. Enter the real IPs and press Save."})


@app.post("/api/settings/set-robot-ips")
def api_settings_set_robot_ips():
    data = request.get_json(force=True, silent=True) or {}
    arm_ip = clean_ip(data.get("arm_ip", ""))
    amr_ip = clean_ip(data.get("amr_ip", ""))
    conveyor_ip = clean_ip(data.get("conveyor_ip", ""))

    with S.lock:
        S.config["arm_ip"] = arm_ip
        S.config["amr_ip"] = amr_ip
        S.config["conveyor_ip"] = conveyor_ip
        S.nodes["arm"]["ip"] = arm_ip
        S.nodes["amr"]["ip"] = amr_ip
        S.nodes["conveyor"]["ip"] = conveyor_ip
    S.save_config()
    S.log(f"Robot IPs saved. arm={arm_ip or '-'} amr={amr_ip or '-'} conveyor={conveyor_ip or '-'}")
    return jsonify({"ok": True, "arm_ip": arm_ip, "amr_ip": amr_ip, "conveyor_ip": conveyor_ip})


@app.post("/api/settings/save-and-test-node")
def api_settings_save_and_test_node():
    data = request.get_json(force=True, silent=True) or {}
    node = str(data.get("node", "")).lower().strip()
    ip = clean_ip(data.get("ip", ""))

    if node not in ("arm", "amr", "conveyor"):
        return jsonify({"ok": False, "err": "node must be arm, amr, or conveyor"}), 400

    key = f"{node}_ip" if node != "conveyor" else "conveyor_ip"

    with S.lock:
        S.config[key] = ip
        S.nodes[node]["ip"] = ip
    S.save_config()

    result = test_http_node(node, ip=ip, timeout=3.0)
    S.log(f"Saved and tested {node} IP {ip}: {result}")
    return jsonify({"ok": bool(result.get("ok")), "node": node, "savedIp": ip, "test": result})


@app.post("/api/settings/clear-ips-force")
def api_settings_clear_ips_force():
    with S.lock:
        S.config["arm_ip"] = ""
        S.config["amr_ip"] = ""
        S.config["conveyor_ip"] = ""
        S.nodes["arm"]["ip"] = ""
        S.nodes["amr"]["ip"] = ""
        S.nodes["conveyor"]["ip"] = ""
        for node in ("arm", "amr", "conveyor"):
            S.nodes[node]["state"] = "WAITING_IP"
            S.nodes[node]["http"] = "UNKNOWN"
            S.nodes[node]["httpInfo"] = ""
    S.save_config()
    return jsonify({"ok": True, "message": "All robot IPs cleared from config and live nodes."})


@app.post("/api/arm/save-ip-and-verify")
def api_arm_save_ip_and_verify():
    data = request.get_json(force=True, silent=True) or {}
    ip = clean_ip(data.get("ip", ""))
    with S.lock:
        S.config["arm_ip"] = ip
        S.nodes["arm"]["ip"] = ip
    S.save_config()

    result = test_http_node("arm", ip=ip, timeout=3.0)
    connected = bool(result.get("ok"))
    arm_state = ""
    if connected and isinstance(result.get("data"), dict):
        arm_state = str(result["data"].get("state", "HTTP_ONLINE"))

    message = "Arm web is reachable."
    if connected and arm_state == "NOT_HOMED":
        message = "Arm is CONNECTED. State is NOT_HOMED, so press Startup Confirm then Home. This is not an IP problem."
    elif not connected:
        message = "Could not reach the arm web server on HTTP port 80."

    return jsonify({"ok": connected, "connected": connected, "ip": ip, "armState": arm_state, "message": message, "test": result})

@app.get("/api/state")
def api_state():
    data = S.snap()
    data["cv"] = cv_status_snapshot()
    return jsonify(data)


@app.post("/api/config")
def api_config():
    data = request.get_json(force=True, silent=True) or {}
    with S.lock:
        if "broker_ip" in data:
            S.config["broker_ip"] = clean_ip(data["broker_ip"]) if str(data["broker_ip"]).strip().startswith("http") else str(data["broker_ip"]).strip()
        if "broker_port" in data:
            S.config["broker_port"] = int(data["broker_port"])
        if "arm_ip" in data:
            S.config["arm_ip"] = clean_ip(data["arm_ip"])
            S.nodes["arm"]["ip"] = S.config["arm_ip"]
        if "amr_ip" in data:
            S.config["amr_ip"] = clean_ip(data["amr_ip"])
            S.nodes["amr"]["ip"] = S.config["amr_ip"]
        if "conveyor_ip" in data:
            S.config["conveyor_ip"] = clean_ip(data["conveyor_ip"])
            S.nodes["conveyor"]["ip"] = S.config["conveyor_ip"]
        if "auto_coordinate" in data:
            S.config["auto_coordinate"] = bool(data["auto_coordinate"])
        if "box_capacity" in data:
            S.config["box_capacity"] = max(1, int(data["box_capacity"]))
        if "simulate_amr" in data:
            S.config["simulate_amr"] = bool(data["simulate_amr"])
        if "amr_step_delay_s" in data:
            S.config["amr_step_delay_s"] = max(0.2, float(data["amr_step_delay_s"]))
        if "sorter_servo_angles" in data and isinstance(data["sorter_servo_angles"], dict):
            angles = sorter_servo_angles()
            for raw_target, raw_angle in data["sorter_servo_angles"].items():
                target = SORTER_SERVO_TARGET_CODES.get(str(raw_target).strip().upper())
                if target:
                    angles[target] = normalize_sorter_servo_angle(raw_angle)
            S.config["sorter_servo_angles"] = angles
        if "workflow_rules" in data and isinstance(data["workflow_rules"], list):
            S.config["workflow_rules"], _ = migrate_manual_arm_workflow_rules(data["workflow_rules"])
    S.save_config()
    S.log("Configuration saved.")
    return jsonify({"ok": True, "config": dict(S.config)})


@app.post("/api/start-broker")
def api_start_broker():
    return jsonify({"ok": start_broker()})


@app.post("/api/connect-mqtt")
def api_connect_mqtt():
    return jsonify({"ok": mqtt_connect()})


@app.post("/api/refresh-recipes")
def api_refresh_recipes():
    return jsonify({"ok": refresh_arm_recipes()})


@app.post("/api/test-node")
def api_test_node():
    data = request.get_json(force=True, silent=True) or {}
    node = str(data.get("node", "")).lower().strip()
    ip = data.get("ip", None)
    result = test_http_node(node, ip, timeout=1.6)
    S.log(f"HTTP test {node}: {result}")
    return jsonify(result)



@app.post("/api/arm/startup-confirm")
def api_arm_startup_confirm():
    code, data = arm_get("/startup/confirm", timeout=4)
    S.log(f"ARM /startup/confirm -> {code}: {data}")
    return jsonify({"ok": bool(code == 200 and data.get("ok", False)), "code": code, "data": data})


@app.post("/api/arm/home")
def api_arm_home():
    code, data = arm_get("/home", timeout=6)
    S.log(f"ARM /home -> {code}: {data}")
    return jsonify({"ok": bool(code in (200, 202) and data.get("ok", False)), "code": code, "data": data})


@app.post("/api/arm/off")
def api_arm_off():
    code, data = arm_get("/off", timeout=4)
    S.log(f"ARM /off -> {code}: {data}")
    return jsonify({"ok": bool(code == 200 and data.get("ok", False)), "code": code, "data": data})

@app.post("/api/abort-arm")
def api_abort_arm():
    code, data = arm_get("/job/abort", timeout=5)
    S.log(f"ARM /job/abort -> {code}: {data}")
    return jsonify({"ok": code == 200 and bool(data.get("ok")), "data": data})


@app.post("/api/arm/retry-active-box")
def api_arm_retry_active_box():
    if not refresh_arm_recipes():
        return jsonify({
            "ok": False,
            "err": f"Arm controller at {S.config.get('arm_ip') or '-'} did not answer /config. Restore the arm connection before retrying.",
        }), 503
    result = retry_active_arm_job()
    return jsonify(result), (200 if result.get("ok") else 409)




@app.get("/api/workflow/rules")
def api_workflow_rules():
    return jsonify({"ok": True, "rules": workflow_rules(), "armRecipes": [str(r.get("name", "")) for r in S.recipes]})


@app.post("/api/workflow/rules/save")
def api_workflow_rules_save():
    data = request.get_json(force=True, silent=True) or {}
    rules = data.get("rules", [])
    if not isinstance(rules, list):
        return jsonify({"ok": False, "err": "rules must be a list"}), 400
    save_workflow_rules(rules)
    for r in workflow_rules():
        ensure_box_keys(r["color"])
    S.log("Workflow rules saved.")
    return jsonify({"ok": True, "rules": workflow_rules()})


@app.post("/api/workflow/rules/add-default")
def api_workflow_rules_add_default():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", "YELLOW"))
    rules = workflow_rules()
    if any(r["color"] == color for r in rules):
        return jsonify({"ok": False, "err": "color already exists"}), 409
    box = f"Box{color.title()}"
    rules.append({"flowName": f"{color}_SYSTEM_FLOW", "color": color, "box": box, "recipe": f"{color}_BOX", "delayBeforeArmMs": 0, "filledEvent": f"{box}_filled", "armLoadedEvent": f"{box}_loaded_on_amr", "deliveredEvent": f"{box}_finished_manual_delivery", "pickup": f"CONVEYOR_{color}_BOX_SLOT", "enabled": True})
    save_workflow_rules(rules)
    ensure_box_keys(color)
    return jsonify({"ok": True, "rules": workflow_rules()})


@app.post("/api/boxes/reset")
def api_boxes_reset():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", ""))
    if meta_for_color(color):
        reset_box_color(color)
    else:
        reset_all_boxes()
    return jsonify({"ok": True, "boxCounts": dict(S.box_counts), "boxStates": dict(S.box_states)})


@app.post("/api/boxes/force-filled")
def api_boxes_force_filled():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", "RED"))
    meta = meta_for_color(color)
    if not meta:
        return jsonify({"ok": False, "err": "bad color"}), 400
    with S.lock:
        S.box_counts[color] = int(S.config.get("box_capacity", 4))
        S.box_states[color] = "FILLED_WAITING_ARM"
        S.job_number += 1
        job = f"JOB_{S.job_number:03d}"
        if not S.active_box_job:
            S.current_job = job
    payload = {
        "event": meta["boxFilledEvent"],
        "jobId": job,
        "color": color,
        "box": meta["box"],
        "count": S.box_counts[color],
        "capacity": int(S.config.get("box_capacity", 4)),
        "recipe": recipe_for_box_color(color),
        "pickup": meta["pickup"],
        "source": "manual_force",
    }
    publish(TOPIC["conveyor_box_filled"], payload, retain=False)
    handle_conveyor_box_filled(payload)
    return jsonify({"ok": True, "payload": payload})


@app.post("/api/amr/sim-flow")
def api_amr_sim_flow():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", S.active_box_color or (workflow_colors()[0] if workflow_colors() else "RED")))
    meta = meta_for_color(color)
    if not meta:
        return jsonify({"ok": False, "err": "bad color"}), 400
    with S.lock:
        job = S.active_box_job or S.current_job
    threading.Thread(target=simulate_amr_delivery, args=(color, job), daemon=True).start()
    return jsonify({"ok": True, "color": color, "jobId": job})


@app.post("/api/amr/sim-event")
def api_amr_sim_event():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", S.active_box_color or (workflow_colors()[0] if workflow_colors() else "RED")))
    meta = meta_for_color(color)
    if not meta:
        return jsonify({"ok": False, "err": "bad color"}), 400
    event = str(data.get("event", ""))
    if not event:
        event = meta["deliveredEvent"]
    payload = {
        "event": event,
        "jobId": S.active_box_job or S.current_job,
        "color": color,
        "box": meta["box"],
        "recipe": recipe_for_box_color(color),
        "source": "manual_amr_sim",
    }
    emit_amr_event(payload)
    return jsonify({"ok": True, "payload": payload})



@app.post("/api/discovery/scan")
def api_discovery_scan():
    data = request.get_json(force=True, silent=True) or {}
    subnet = str(data.get("subnet") or S.discovery.get("subnet", "192.168.137")).strip()
    octets = subnet.split(".")
    if not re.match(r"^\d{1,3}\.\d{1,3}\.\d{1,3}$", subnet) or any(int(x) > 255 for x in octets):
        return jsonify({"ok": False, "err": "subnet must look like 192.168.137"}), 400

    with S.lock:
        if S.discovery.get("running"):
            return jsonify({"ok": False, "err": "scan already running", "discovery": dict(S.discovery)}), 409
        S.discovery["running"] = True
        S.discovery["subnet"] = subnet
        S.discovery["lastScanText"] = f"Scanning {subnet}.2-254..."

    threading.Thread(target=discovery_worker, args=(subnet,), daemon=True).start()
    S.event("DISCOVERY", f"Started HTTP device scan on {subnet}.0/24")
    return jsonify({"ok": True, "message": "scan started"})


@app.post("/api/discovery/apply")
def api_discovery_apply():
    with S.lock:
        result = {"found": dict(S.discovery.get("found", {})), "elapsedMs": 0, "subnet": S.discovery.get("subnet", "192.168.137")}
    apply_discovery_results(result)
    return jsonify({"ok": True, "discovery": S.discovery})


@app.post("/api/discovery/set-static-defaults")
def api_discovery_static_defaults():
    with S.lock:
        S.config["broker_ip"] = "127.0.0.1"
        S.config["arm_ip"] = "192.168.137.190"
        S.config["conveyor_ip"] = "192.168.137.191"
        S.config["amr_ip"] = "192.168.137.192"
        S.nodes["arm"]["ip"] = S.config["arm_ip"]
        S.nodes["conveyor"]["ip"] = S.config["conveyor_ip"]
        S.nodes["amr"]["ip"] = S.config["amr_ip"]
    S.save_config()
    S.event("DISCOVERY", "Static default IP plan applied.")
    return jsonify({"ok": True, "config": S.config})



def amr_get(path: str, timeout: float = 4.0):
    ip = clean_ip(S.config.get("amr_ip", ""))
    if not ip:
        return None, {"ok": False, "err": "AMR IP missing"}
    try:
        r = requests.get(f"http://{ip}{path}", timeout=timeout)
        try:
            data = r.json()
        except Exception:
            data = {"raw": r.text[:1000]}
        return r.status_code, data
    except Exception as exc:
        return None, {"ok": False, "err": str(exc)}


@app.get("/api/amr/recipes")
def api_amr_recipes():
    code, data = amr_get("/amr/recipes", timeout=5)
    return jsonify({"ok": code == 200, "code": code, "data": data})


@app.post("/api/amr/run-recipe")
def api_amr_run_recipe():
    data = request.get_json(force=True, silent=True) or {}
    name = str(data.get("name", "RED_BOX_TO_TARGET")).strip()
    code, resp = amr_get("/amr/run-recipe?name=" + quote(name), timeout=5)
    S.event("AMR", f"HTTP run recipe {name}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp})


@app.post("/api/amr/real-flow")
def api_amr_real_flow():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", "RED")) or "RED"
    code, resp = amr_get("/amr/test-flow?color=" + quote(color), timeout=5)
    S.event("AMR", f"HTTP real flow {color}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp})


@app.post("/api/amr/stop-real")
def api_amr_stop_real():
    code, resp = amr_get("/amr/stop", timeout=5)
    S.event("AMR", "HTTP stop", {"code": code, "response": resp})
    return jsonify({"ok": code == 200, "code": code, "data": resp})


@app.post("/api/amr/reset-odom")
def api_amr_reset_odom():
    data = request.get_json(force=True, silent=True) or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    th = float(data.get("th", 0))
    code, resp = amr_get(f"/amr/reset-odom?x={x}&y={y}&th={th}", timeout=5)
    S.event("AMR", "HTTP reset odom", {"code": code, "response": resp})
    return jsonify({"ok": code == 200, "code": code, "data": resp})



@app.get("/api/amr/real-status")
def api_amr_real_status():
    code, data = amr_get("/status", timeout=4)
    return jsonify({"ok": code == 200, "code": code, "data": data})


@app.get("/api/amr/route-recipes")
def api_amr_route_recipes():
    code, data = amr_get("/amr/route-recipes", timeout=6)
    return jsonify({"ok": code == 200, "code": code, "data": data})


@app.post("/api/amr/http-go")
def api_amr_http_go():
    data = request.get_json(force=True, silent=True) or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    th = float(data.get("th", 0))
    code, resp = amr_get(f"/nav?x={x}&y={y}&th={th}", timeout=5)
    S.event("AMR", f"HTTP go to X{x} Y{y} TH{th}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200, "code": code, "data": resp})


@app.post("/api/amr/http-drive")
def api_amr_http_drive():
    data = request.get_json(force=True, silent=True) or {}
    left = int(float(data.get("left", 0)))
    right = int(float(data.get("right", 0)))
    code, resp = amr_get(f"/drive?left={left}&right={right}", timeout=4)
    S.event("AMR", f"HTTP drive L{left} R{right}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200, "code": code, "data": resp})


@app.post("/api/amr/http-stop")
def api_amr_http_stop():
    code, resp = amr_get("/amr/recipe/stop", timeout=4)
    if code != 200:
        code, resp = amr_get("/amr/stop", timeout=4)
    amr_get("/drive?left=0&right=0", timeout=3)
    S.event("AMR", "HTTP stop AMR", {"code": code, "response": resp})
    return jsonify({"ok": code == 200, "code": code, "data": resp})


@app.post("/api/amr/http-reset-odom")
def api_amr_http_reset_odom():
    data = request.get_json(force=True, silent=True) or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    th = float(data.get("th", 0))
    code, resp = amr_get(f"/amr/reset-odom?x={x}&y={y}&th={th}", timeout=5)
    S.event("AMR", f"HTTP reset odom X{x} Y{y} TH{th}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200, "code": code, "data": resp})


@app.post("/api/amr/http-save-current")
def api_amr_http_save_current():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(float(data.get("idx", 0)))
    wait = int(float(data.get("wait", 300)))
    code, resp = amr_get(f"/amr/recipe/add-current?idx={idx}&wait={wait}", timeout=5)
    S.event("AMR", f"HTTP save current pose to recipe {idx}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp})


@app.post("/api/amr/http-run-route")
def api_amr_http_run_route():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(float(data.get("idx", 0)))
    code, resp = amr_get(f"/amr/recipe/run?idx={idx}", timeout=5)
    S.event("AMR", f"HTTP run route recipe {idx}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp})


@app.post("/api/amr/http-delete-last")
def api_amr_http_delete_last():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(float(data.get("idx", 0)))
    code, resp = amr_get(f"/amr/recipe/delete-last?idx={idx}", timeout=5)
    S.event("AMR", f"HTTP delete last step recipe {idx}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp})


@app.post("/api/amr/http-clear-route")
def api_amr_http_clear_route():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(float(data.get("idx", 0)))
    code, resp = amr_get(f"/amr/recipe/clear?idx={idx}", timeout=5)
    S.event("AMR", f"HTTP clear route recipe {idx}", {"code": code, "response": resp})
    return jsonify({"ok": code == 200 and bool(resp.get("ok", True)), "code": code, "data": resp})


@app.post("/api/amr/mqtt-status-test")
def api_amr_mqtt_status_test():
    payload = {"cmd": "STATUS", "commandId": f"TEST_STATUS_{ms()}"}
    ok = publish(TOPIC["amr_cmd"], payload, retain=False)
    return jsonify({"ok": ok, "topic": TOPIC["amr_cmd"], "payload": payload})


@app.post("/api/amr/mqtt-run-recipe-test")
def api_amr_mqtt_run_recipe_test():
    data = request.get_json(force=True, silent=True) or {}
    idx = int(float(data.get("idx", 0)))
    payload = {
        "cmd": "RUN_RECIPE",
        "recipeIndex": idx,
        "jobId": f"JOB_TEST_RECIPE_{idx}",
        "commandId": f"TEST_RUN_RECIPE_{ms()}",
    }
    ok = publish(TOPIC["amr_cmd"], payload, retain=False)
    return jsonify({"ok": ok, "topic": TOPIC["amr_cmd"], "payload": payload})


@app.post("/api/amr/mqtt-pick-color-test")
def api_amr_mqtt_pick_color_test():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", "RED")) or "RED"
    meta = meta_for_color(color) or {}
    payload = {
        "cmd": "DELIVER_BOX_TO_TARGET",
        "jobId": f"JOB_TEST_{color}",
        "commandId": f"TEST_DELIVER_{color}_{ms()}",
        "color": color,
        "box": meta.get("box", f"Box{color.title()}"),
        "pickup": "ARM_STATION",
        "destination": meta.get("target", f"TARGET_{color}"),
        "recipeIndex": int(meta.get("amrRecipeIndex", 0 if color == "RED" else 1 if color == "GREEN" else 2)),
        "recipeName": meta.get("amrRecipeName", ""),
    }
    ok = publish(TOPIC["amr_cmd"], payload, retain=False)
    return jsonify({"ok": ok, "topic": TOPIC["amr_cmd"], "payload": payload})


@app.post("/api/amr/finished-job")
def api_amr_finished_job():
    with S.lock:
        state = S.mission_state
        job = S.active_box_job
        color = S.active_box_color
        box = S.active_box_name
    if not job:
        return jsonify({"ok": False, "err": "No active box job is waiting for manual AMR delivery."}), 409
    if state != "WAITING_OPERATOR_FINISH":
        return jsonify({"ok": False, "err": f"Active job is not ready to finish. Current state: {state}"}), 409

    complete_active_delivery_to_target({
        "jobId": job,
        "color": color,
        "box": box,
        "destination": "MANUAL_AMR_DELIVERY",
        "source": "operator_finished_job",
    })
    return jsonify({"ok": True, "finishedJobId": job, "finishedBox": box, "state": S.snap()})


@app.post("/api/job/reset")
def api_job_reset():
    with S.lock:
        S.operation_generation += 1
        S.last_box_filled_key = ""
        S.command_seq += 1
        S.current_job = f"JOB_{S.command_seq:03d}"
        S.current_package = ""
        S.active_box_color = ""
        S.active_box_name = ""
        S.active_box_recipe = ""
        S.active_box_job = ""
        S.box_queue.clear()
        S.pending_acks.clear()
        S.system = "IDLE"
        S.mission_state = "IDLE"
        S.next_action = "Current job cleared. System ready."
    S.event("COORDINATOR", "current job reset")
    return jsonify({"ok": True})


@app.post("/api/job/retry-last")
def api_job_retry_last():
    with S.lock:
        last = dict(S.last_command) if S.last_command else None
    if not last:
        return jsonify({"ok": False, "err": "No last command"}), 404

    topic = last.get("topic", "")
    payload = dict(last.get("payload", {}))
    payload["retryOf"] = payload.get("commandId", last.get("commandId", ""))
    payload["retry"] = True

    if topic == TOPIC["amr_cmd"]:
        expected = ["AMR_DELIVERY_COMMAND_ACCEPTED", "AMR_COMMAND_ACCEPTED"] if payload.get("cmd") == "DELIVER_BOX_TO_TARGET" else "AMR_COMMAND_ACCEPTED"
        command_id = S.add_pending_ack("amr", payload.get("cmd", "COMMAND"), expected, 8.0, payload, topic)
        payload["commandId"] = command_id

    ok = publish(topic, payload, retain=False)
    with S.lock:
        S.system = "RETRY_SENT" if ok else "RETRY_FAILED"
        S.mission_state = "WAITING_ACK" if ok else "FAULT"
        S.next_action = f"Retried last command on {topic}."
    S.event("COORDINATOR", "retried last command", payload)
    return jsonify({"ok": ok, "payload": payload})


@app.post("/api/faults/clear")
def api_faults_clear():
    with S.lock:
        # Clear only coordinator-side pending ACK timers. Real robot faults still reappear if the robot stays offline/faulted.
        S.pending_acks.clear()
        if S.mission_state == "FAULT":
            S.mission_state = "IDLE"
            S.system = "IDLE"
            S.next_action = "Coordinator faults cleared. Check robots before starting."
    S.event("COORDINATOR", "faults cleared")
    return jsonify({"ok": True})


@app.post("/api/stop-all")
def api_stop_all():
    amr_ok = publish(TOPIC["amr_cmd"], {"cmd": "STOP", "event": "STOP_ALL", "robot": "coordinator"}, retain=False)
    conveyor_ok = publish(TOPIC["conveyor_cmd"], {"cmd": "STOP", "event": "STOP_ALL", "robot": "coordinator"}, retain=False)
    code, data = arm_get("/job/abort", timeout=4)
    arm_ok = code in (200, 202) and bool(data.get("ok", True))
    with S.lock:
        S.operation_generation += 1
        S.pending_acks.clear()
        S.system = "STOP_ALL"
        S.mission_state = "STOPPED"
        S.next_action = "Stop requested for conveyor, AMR, and arm. Verify each subsystem is stopped."
    S.event("COORDINATOR", "STOP_ALL", {"amrPublished": amr_ok, "conveyorPublished": conveyor_ok, "armCode": code, "arm": data})
    return jsonify({"ok": bool(amr_ok and conveyor_ok and arm_ok), "amrCommandPublished": amr_ok, "conveyorCommandPublished": conveyor_ok, "armOk": arm_ok, "armCode": code, "arm": data})


@app.post("/api/boxes/adjust")
def api_boxes_adjust():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", ""))
    delta = int(data.get("delta", 0))
    if not color or delta == 0:
        return jsonify({"ok": False, "err": "color and nonzero delta required"}), 400
    ensure_box_keys(color)
    if delta > 0:
        for _ in range(delta):
            record_conveyor_cube(color)
    else:
        with S.lock:
            S.box_counts[color] = max(0, int(S.box_counts.get(color, 0)) + delta)
            if S.box_counts[color] < int(S.config.get("box_capacity", 4)):
                S.box_states[color] = "FILLING"
        S.event("CONVEYOR", f"manual count adjustment {color} {delta}", box_status_payload(color))
    return jsonify({"ok": True, "status": box_status_payload(color)})


@app.post("/api/sim/conveyor")
def api_sim_conveyor():
    data = request.get_json(force=True, silent=True) or {}

    if "color" in data:
        color = normalize_color(data.get("color", ""))
        if not meta_for_color(color):
            return jsonify({"ok": False, "err": "bad color"}), 400
        record_conveyor_cube(color)
        return jsonify({
            "ok": True,
            "mode": "CUBE_DETECTION",
            "status": box_status_payload(color),
        })

    package = data.get("packageClass") or S.selected_recipe_name

    with S.lock:
        S.job_number += 1
        S.current_job = f"JOB_{S.job_number:03d}"
        job = S.current_job

    payload = {
        "jobId": job,
        "packageClass": package,
        "event": "PACKAGE_READY",
    }
    publish(TOPIC["conveyor_package"], payload)
    return jsonify({"ok": True, "payload": payload})


@app.post("/api/sim/amr-arrived")
def api_sim_amr_arrived():
    with S.lock:
        payload = {
            "event": "ARRIVED_AT_TARGET",
            "jobId": S.current_job,
            "packageClass": S.selected_recipe_name,
        }
    publish(TOPIC["amr_event"], payload)
    return jsonify({"ok": True, "payload": payload})


@app.post("/api/sim/full")
def api_sim_full():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", S.active_box_color or "RED")) or "RED"
    meta = meta_for_color(color)
    if not meta:
        return jsonify({"ok": False, "err": "bad color"}), 400
    with S.lock:
        if S.active_box_job:
            return jsonify({"ok": False, "err": "A box job is already active. Finish or reset it before full simulation."}), 409
        S.job_number += 1
        S.current_job = f"JOB_{S.job_number:03d}"
        job = S.current_job
        S.operation_generation += 1
        generation = S.operation_generation
        S.active_box_color = color
        S.active_box_name = meta["box"]
        S.active_box_recipe = recipe_for_box_color(color)
        S.active_box_job = job
        S.box_states[color] = "FILLED_WAITING_ARM"
        S.system = meta["boxFilledEvent"]
        S.mission_state = "BOX_FILLED"
        S.next_action = f"Full simulation: {meta['box']} is ready for the arm."

    def later():
        time.sleep(1.2)
        with S.lock:
            if generation != S.operation_generation or job != S.active_box_job:
                return
            S.box_states[color] = "ARM_LOADING_AMR"
            S.system = "ARM RUNNING"
            S.mission_state = "ARM_LOADING_AMR"
            S.next_action = f"Full simulation: arm is loading {meta['box']} onto the AMR."
        S.event("ARM SIM", "arm loading AMR", {"jobId": job, "color": color, "box": meta["box"]})
        time.sleep(1.2)
        S.event("ARM SIM", meta["armLoadedEvent"], {"jobId": job, "color": color, "box": meta["box"]})
        handle_arm_loaded_on_amr({"state": "DONE", "jobId": job, "source": "full_simulation"})

    threading.Thread(target=later, daemon=True).start()
    return jsonify({"ok": True, "jobId": job, "color": color, "mode": "FULL_SIMULATION"})



@app.get("/cv/status")
def api_cv_status():
    return jsonify(cv_status_snapshot())


@app.post("/api/conveyor/servo-test")
def api_conveyor_servo_test():
    data = request.get_json(force=True, silent=True) or {}
    code = str(data.get("code", "")).strip().upper()
    if code not in ("R", "G", "B", "CENTER"):
        return jsonify({"ok": False, "err": "code must be R, G, B, or CENTER"}), 400
    result = cv_send_servo(code, allow_center=(code == "CENTER"))
    return jsonify(result), (200 if result.get("ok") else 503)


@app.get("/api/conveyor/servo-calibration")
def api_conveyor_servo_calibration():
    with CV_LOCK:
        last = dict(CV.get("lastServoResponse", {}))
    return jsonify({"ok": True, "angles": sorter_servo_angles(), "last": last})


@app.post("/api/conveyor/servo-angle")
def api_conveyor_servo_angle():
    data = request.get_json(force=True, silent=True) or {}
    result = cv_move_servo_angle(data.get("angle"), "MANUAL")
    status = 400 if "angle must be" in str(result.get("err", "")) else (200 if result.get("ok") else 503)
    return jsonify(result), status


@app.post("/api/conveyor/servo-save")
def api_conveyor_servo_save():
    data = request.get_json(force=True, silent=True) or {}
    target = SORTER_SERVO_TARGET_CODES.get(str(data.get("target", "")).strip().upper())
    if not target:
        return jsonify({"ok": False, "err": "target must be RED, GREEN, BLUE, or CENTER"}), 400
    try:
        angle = normalize_sorter_servo_angle(data.get("angle"))
    except ValueError as exc:
        return jsonify({"ok": False, "err": str(exc)}), 400
    with S.lock:
        angles = sorter_servo_angles()
        angles[target] = angle
        S.config["sorter_servo_angles"] = angles
    S.save_config()
    S.log(f"Saved sorter servo {target} position: {angle} degrees")
    return jsonify({"ok": True, "target": target, "angle": angle, "angles": angles})


@app.get("/cv/debug")
def api_cv_debug():
    data = cv_status_snapshot()
    html = "<html><body style='background:#000;color:#fff;font-family:Arial'>"
    html += "<h2>Internal OpenCV Debug</h2>"
    html += "<pre>" + json.dumps(data, indent=2) + "</pre>"
    html += "<h3>Raw</h3><img src='/cv/raw.mjpg' style='max-width:720px;width:100%;border:1px solid #333'>"
    html += "<h3>Processed</h3><img src='/cv/processed.mjpg' style='max-width:720px;width:100%;border:1px solid #333'>"
    html += "</body></html>"
    return Response(html, mimetype="text/html")


@app.post("/api/cv/restart")
def api_cv_restart():
    CV_RESTART.set()
    S.log("Internal OpenCV pipeline restart requested.")
    return jsonify({"ok": True})


@app.post("/api/cv/calibrate")
def api_cv_calibrate():
    data = request.get_json(force=True, silent=True) or {}
    try:
        profile = cv_calibrate_color(data.get("color", ""))
        return jsonify({"ok": True, "profile": profile, "profiles": cv_saved_color_profiles()})
    except ValueError as exc:
        return jsonify({"ok": False, "err": str(exc)}), 400


@app.post("/api/cv/calibration/reset")
def api_cv_calibration_reset():
    data = request.get_json(force=True, silent=True) or {}
    color = normalize_color(data.get("color", ""))
    if color not in ("RED", "GREEN", "BLUE"):
        return jsonify({"ok": False, "err": "Color must be RED, GREEN, or BLUE."}), 400
    with S.lock:
        profiles = dict(S.config.get("cv_color_profiles", {}))
        profiles.pop(color, None)
        S.config["cv_color_profiles"] = profiles
    S.save_config()
    S.log(f"Reset learned camera color profile for {color}.")
    return jsonify({"ok": True, "profiles": cv_saved_color_profiles()})


@app.get("/cv/raw.mjpg")
def cv_raw_mjpg():
    def gen():
        while True:
            with CV_LOCK:
                frame = CV.get("rawJpeg") or cv_blank_frame("Waiting for raw frame")
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.045)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.get("/cv/processed.mjpg")
def cv_processed_mjpg():
    def gen():
        while True:
            with CV_LOCK:
                frame = CV.get("processedJpeg") or cv_blank_frame("Waiting for processed frame")
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.045)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")



def start_workers():
    threading.Thread(target=worker_boot, daemon=True).start()
    threading.Thread(target=worker_recipes, daemon=True).start()
    threading.Thread(target=worker_arm, daemon=True).start()
    threading.Thread(target=worker_http_ping, daemon=True).start()
    threading.Thread(target=worker_mqtt_watchdog, daemon=True).start()
    threading.Thread(target=worker_coordinator_status, daemon=True).start()
    threading.Thread(target=cv_worker_opencv, daemon=True).start()



def auto_start_services():
    try:
        ok = start_broker()
        if ok:
            time.sleep(0.8)
            mqtt_connect()
            S.log("Auto-started broker and MQTT connection.")
        else:
            S.log("Auto-start broker failed. Press Start Broker manually.")
    except Exception as exc:
        S.log(f"Auto-start services failed: {exc}")


if __name__ == "__main__":
    S.log("Starting Robot Cell Control Center v33 arm HTTP online fix.")
    threading.Thread(target=auto_start_services, daemon=True).start()
    start_workers()
    try:
        threading.Timer(1.0, lambda: webbrowser.open(f"http://127.0.0.1:{PORT}")).start()
    except Exception:
        pass
    app.run(host="0.0.0.0", port=PORT, debug=False, threaded=True)
