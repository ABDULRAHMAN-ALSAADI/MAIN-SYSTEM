# Robot Cell Control Center - Integration Guide

## 1. Purpose

This folder contains the laptop-side supervisor for a robot cell:

```text
ESP32-CAM / conveyor -> box counting -> SCARA arm loads AMR -> AMR delivers to target
```

The control center provides:

- A Flask dashboard and API on `http://127.0.0.1:5050`.
- An integrated OpenCV pipeline for the ESP32-CAM.
- MQTT coordination through a local Mosquitto broker.
- HTTP monitoring and manual controls for the AMR and SCARA arm.
- Workflow rules that map each filled color box to its arm loading recipe.
- A FIFO filled-box queue and an operator `Finished Job` control for manual AMR delivery.
- AMR simulation for testing when the real AMR is unavailable.

It does not contain the firmware for the conveyor, ESP32-CAM, AMR, or arm.

## 2. Network Plan

The intended Windows hotspot network is:

| Component | Address / Port | Purpose |
|---|---|---|
| Dashboard | `127.0.0.1:5050` | Browser UI and coordinator API |
| MQTT from laptop | `127.0.0.1:1883` | Coordinator connection to Mosquitto |
| MQTT from robots | `192.168.137.1:1883` | Robots connect to the laptop broker |
| Conveyor camera | `http://CONVEYOR_IP:81/stream` | MJPEG camera stream |
| Conveyor servo | `http://CONVEYOR_IP:82/servo?color=R` | Color/servo command |
| AMR HTTP | `http://AMR_IP/...` | Status, movement, and route recipes |
| Arm HTTP | `http://ARM_IP/...` | Status, homing, recipes, and jobs |

Windows Firewall must allow inbound TCP port `1883` for robot MQTT access.
Mosquitto 2 defaults to localhost-only mode when no listener is configured.
`SETUP_MQTT_BROKER_ADMIN.ps1` configures the listener and a firewall rule
limited to `192.168.137.0/24`.

## 3. Startup

1. Install Mosquitto for Windows.
2. Run `SETUP_MQTT_BROKER_ADMIN.ps1` once from PowerShell as Administrator.
3. Run `RUN_CONTROL_CENTER.bat`.
4. Open `http://127.0.0.1:5050`.
5. Enter the arm, AMR, and conveyor IP addresses in Settings.
6. Confirm the AMR is at the arm loading position and will be controlled manually.
7. Run Preflight.
8. Confirm the arm is connected, startup-confirmed, homed, and ready.

Do not run `conveyor_cv_bridge.py` at the same time as the main control center.
Both programs would open the ESP32-CAM stream, and the ESP32-CAM may disconnect
one of them.

## 4. Main Workflow

1. The integrated OpenCV worker opens `http://CONVEYOR_IP:81/stream`.
2. It detects RED, GREEN, or BLUE using HSV masks.
3. A detection must be stable for 8 frames and have contour area above 500.
4. One cube is counted. The same object is not counted again until it leaves.
5. When a color box reaches `box_capacity`, a box-filled event is created.
6. The coordinator reads the enabled workflow rule for that color.
7. If another job is active, the filled box is added to the FIFO queue.
8. The coordinator starts the matching arm recipe:
   `GREEN_BOX_A1`, `RED_BOX`, or `BLUE_BOX`.
9. When arm `/job/status` reports `DONE`, the coordinator waits while the
   operator manually delivers the box and returns the AMR.
10. The operator presses `Finished Job` in the AMR tab. The coordinator resets
    that box and immediately starts the next queued filled box, if one exists.

`Auto coordinator = OFF` is monitor-only mode. Box and AMR events are observed,
but automatic AMR and arm commands are blocked.

The automatic production workflow does not send AMR movement commands. AMR
movement is controlled manually by the operator.

## 5. MQTT Broker

The coordinator attempts to start Mosquitto with:

```text
listener 1883 0.0.0.0
allow_anonymous true
```

The coordinator subscribes to:

```text
cell/#
```

All JSON messages published by the coordinator include:

```json
{
  "schema": "robot_cell.v1",
  "timeMs": 1710000000000,
  "source": "coordinator"
}
```

## 6. Conveyor / Vision Contract

### What the conveyor must provide

Required for integrated vision:

| Method | Endpoint | Expected behavior |
|---|---|---|
| GET | `http://CONVEYOR_IP:81/stream` | Continuous MJPEG stream |
| GET | `http://CONVEYOR_IP:82/servo?color=R` | Handle `R`, `G`, or `B` |

Recommended for health checks and discovery:

| Method | Endpoint |
|---|---|
| GET | `/status` |
| GET | `/net` |
| GET | `/` |
| GET | `/capture` |

### What the conveyor may publish

`cell/conveyor/status`

```json
{
  "robot": "conveyor",
  "state": "ONLINE"
}
```

`cell/conveyor/cube`

```json
{
  "event": "CUBE_COUNTED",
  "color": "RED",
  "box": "BoxRed",
  "jobId": "JOB_002"
}
```

External cube events are added to the coordinator's box counter. External
devices must not use source `internal_opencv_dashboard`, because that source is
reserved for the coordinator's own loop-prevention messages.

`cell/conveyor/box_filled`

```json
{
  "event": "BoxRed_filled",
  "jobId": "JOB_002",
  "color": "RED",
  "box": "BoxRed",
  "count": 4,
  "capacity": 4,
  "recipe": "RED_BOX",
  "pickup": "CONVEYOR_RED_BOX_SLOT"
}
```

`cell/conveyor/package` is a legacy direct-package path. Its `packageClass`
must exactly match an arm recipe name.

### What the conveyor receives

`cell/conveyor/cmd`

```json
{"cmd": "JOB_DONE", "jobId": "JOB_002"}
```

```json
{"cmd": "STOP", "event": "STOP_ALL"}
```

The conveyor firmware is responsible for deciding how to stop motors and how
to react to `JOB_DONE`.

## 7. AMR Contract

The AMR is manually controlled. The automatic box workflow does not require an
AMR MQTT command or AMR event. After delivery and return, the operator presses
`Finished Job` in the AMR tab.

### What the AMR must provide over HTTP

Monitoring:

| Method | Endpoint | Expected response |
|---|---|---|
| GET | `/status` | JSON status |
| GET | `/identity` or `/` | Optional discovery identity |
| GET | `/amr/route-recipes` | Route recipe list |
| GET | `/amr/recipes` | Optional alternative recipe list |

Manual control and route teaching:

| Method | Endpoint |
|---|---|
| GET | `/nav?x=X&y=Y&th=TH` |
| GET | `/drive?left=L&right=R` |
| GET | `/amr/stop` |
| GET | `/amr/reset-odom?x=X&y=Y&th=TH` |
| GET | `/amr/recipe/add-current?idx=N&wait=MS` |
| GET | `/amr/recipe/run?idx=N` |
| GET | `/amr/recipe/delete-last?idx=N` |
| GET | `/amr/recipe/clear?idx=N` |

### What the AMR receives over MQTT

Topic: `cell/amr/cmd`

Optional manual/test commands:

```text
STATUS
RUN_RECIPE
STOP
```

### What the AMR must publish

Status topic:

```text
cell/amr/status
```

Event topic:

```text
cell/amr/event
```

AMR events are monitored for information, but they do not complete the active
box job. Only the operator `Finished Job` action completes it.

## 8. SCARA Arm Contract

### What the arm must provide

| Method | Endpoint | Expected behavior |
|---|---|---|
| GET | `/job/status` | Current state and job information |
| GET | `/config` | JSON containing `packageRecipes` |
| GET | `/startup/confirm` | Confirm startup |
| GET | `/home` | Home the arm |
| GET | `/off` | Turn motors off |
| GET | `/job/load?recipe=N&jobId=ID&packageClass=NAME` | Load recipe |
| GET | `/job/wait-amr` | Put loaded job into AMR-wait state |
| GET | `/job/amr-arrived` | Start the loaded job |
| GET | `/job/abort` | Abort current job |

Example `/job/status`:

```json
{
  "ok": true,
  "state": "READY",
  "jobId": "JOB_002",
  "packageClass": "RED_BOX",
  "result": ""
}
```

Recognized arm states include:

```text
NOT_HOMED
READY
WAITING_FOR_AMR
RUNNING
DONE
FAULT
STOPPED
```

`NOT_HOMED` means the arm HTTP service is connected. It is not a network
failure.

Example `/config`:

```json
{
  "packageRecipes": [
    {
      "index": 0,
      "name": "RED_BOX",
      "steps": []
    }
  ]
}
```

The workflow rule's arm recipe name must match a stored arm recipe name.
That recipe must move the filled conveyor box onto the AMR. The coordinator
interprets arm state `DONE` as "box loaded on AMR", not as final job completion.

### What the coordinator publishes for the arm

The coordinator polls the arm over HTTP and republishes status to:

```text
cell/arm/status
cell/arm/event
```

The arm does not need MQTT for the current automatic path.

## 9. Coordinator Outputs

`cell/coordinator/status` is published every second and retained:

```json
{
  "robot": "coordinator",
  "state": "ARM_LOADING_AMR",
  "system": "ARM RUNNING",
  "jobId": "JOB_002",
  "package": "BoxRed_filled",
  "nextAction": "Arm is loading BoxRed onto the AMR.",
  "faultCount": 0
}
```

`cell/coordinator/event` publishes workflow transitions such as:

```text
BOX_FILLED_ACCEPTED
BoxRed_loaded_on_amr
BoxRed_finished_manual_delivery
```

## 10. Default Workflow Rules

| Color | Box | Arm recipe | Filled event | Arm loaded event | Finished Job event |
|---|---|---|---|---|---|
| RED | BoxRed | `RED_BOX` | `BoxRed_filled` | `BoxRed_loaded_on_amr` | `BoxRed_finished_manual_delivery` |
| GREEN | BoxGreen | `GREEN_BOX_A1` | `BoxGreen_filled` | `BoxGreen_loaded_on_amr` | `BoxGreen_finished_manual_delivery` |
| BLUE | BoxBlue | `BLUE_BOX` | `BoxBlue_filled` | `BoxBlue_loaded_on_amr` | `BoxBlue_finished_manual_delivery` |

The UI sometimes labels BLUE as BLACK/BLUE, but the current OpenCV pipeline
detects blue pixels only. It does not contain a black-object HSV detector.

## 11. Stop and Reset Behavior

- `Stop All` publishes STOP to the AMR and conveyor and calls arm `/job/abort`.
- STOP is latched in the coordinator.
- New box-filled and AMR delivery events are ignored while STOPPED.
- Press `Reset Mission` before resuming automatic operation.
- Resetting a mission or active box cancels delayed arm/AMR commands and
  simulated AMR delivery steps.

The dashboard stop is network-dependent and is not a safety-rated emergency
stop. Physical machinery must have a hardware emergency-stop circuit.

## 12. Preflight Checklist

Before automatic operation:

1. Hardware emergency stop is tested.
2. Mosquitto is running.
3. `127.0.0.1:1883` is reachable from the laptop.
4. `192.168.137.1:1883` is reachable from robot devices.
5. Conveyor camera stream works.
6. Conveyor servo endpoint accepts `R`, `G`, and `B`.
7. AMR publishes status and accepts a STATUS command.
8. AMR is at the arm loading position before an arm recipe starts.
9. Arm `/job/status` is reachable.
10. Arm is startup-confirmed, homed, and READY.
11. Arm contains recipes named exactly `GREEN_BOX_A1`, `RED_BOX`, and `BLUE_BOX`.
12. The operator presses `Finished Job` only after delivery and AMR return.

Conveyor preflight requires the integrated OpenCV pipeline to be running and
receiving frames. Arm preflight requires both `/job/status` and `/config`.

## 13. Automated Verification

Run:

```powershell
python -m unittest -v test_control_center.py
python -m py_compile cell_control_center.py conveyor_cv_bridge.py
```

The tests verify coordinator logic only. They do not prove that the physical
conveyor, AMR, arm, camera, network, or emergency-stop hardware behaves
correctly.
