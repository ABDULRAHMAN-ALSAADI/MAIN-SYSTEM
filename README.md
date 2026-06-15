# Robot Cell Factory Supervisor

Central coordination and web-control software for an integrated robotic production cell containing:

* SCARA robotic arm
* Conveyor belt
* Computer-vision sorting system
* Autonomous mobile robot
* MQTT communication layer
* Web-based operator dashboard

The coordinator manages communication, task order, subsystem status, workflow events, and operator commands. Real-time motor control and local safety operations remain inside each subsystem controller.

---

## System Purpose

The production cell performs the following sequence:

1. The conveyor moves an object into the camera area.
2. The vision system detects and classifies the object.
3. The servo sorting mechanism directs the object to the correct box.
4. The coordinator monitors the box counters.
5. When a box becomes full, a workflow event is created.
6. The coordinator selects the SCARA recipe assigned to that box.
7. The SCARA arm loads the box onto the AMR.
8. The AMR transports the box to the required station.
9. The coordinator records task completion and starts the next queued job.

The workflow can be operated automatically or with manual confirmation depending on the selected system mode.

---

## Main Features

* Central web dashboard
* MQTT-based event communication
* HTTP communication with robotic subsystems
* Dynamic subsystem IP configuration
* Network device scanning
* Saved connection profiles
* SCARA recipe synchronization and execution
* Conveyor and vision monitoring
* Box counter and filled-box queue management
* Servo sorter calibration
* Camera color-profile configuration
* AMR status and manual-control tools
* Workflow-rule configuration
* Emergency stop and abort commands
* System logs and device-health monitoring

---

## System Architecture

```text
Web Browser
     │
     ▼
Python Coordinator
     │
     ├── MQTT Broker
     │      ├── Conveyor and Vision System
     │      ├── SCARA Robot Arm
     │      └── Autonomous Mobile Robot
     │
     └── HTTP Communication
            ├── Conveyor Controller
            ├── SCARA Controller
            └── AMR Controller
```

The coordinator is the upper-level controller. It sends task commands and receives system states but does not generate real-time motor pulses.

Each robotic subsystem remains responsible for:

* Motor control
* Sensor reading
* Local safety limits
* Calibration
* Homing
* Emergency behavior
* Low-level motion execution

---

## Communication Methods

### MQTT

MQTT is used for event-based communication, including:

* Box-filled events
* Robot-ready states
* Task-start events
* Task-completed events
* Error messages
* Heartbeat messages
* Workflow progress
* Queue updates

### HTTP

HTTP is used for direct commands and status requests, including:

* Checking whether a device is online
* Reading subsystem status
* Starting a SCARA recipe
* Homing the SCARA arm
* Sending an AMR target
* Stopping a subsystem
* Calibrating the sorter servo
* Refreshing recipe or route lists

---

## Dynamic Network Configuration

Subsystem IP addresses are not fixed.

The assigned addresses may change when:

* A different router is used
* A mobile hotspot is restarted
* A computer hotspot is recreated
* DHCP assigns a new address
* A subsystem reconnects to the network
* The system is moved to another location

For this reason, the coordinator provides a **Settings** page where the current addresses can be entered and saved.

Configurable network values include:

* MQTT broker address
* MQTT broker port
* SCARA arm address
* AMR address
* Conveyor and vision address
* Network scan range
* HTTP ports when required

The coordinator also provides network scanning and connection-checking tools to help locate devices on the current network.

Do not place permanent subsystem IP addresses directly inside the Python source code.

---

## Important MQTT Address Rule

When Mosquitto is running on the same computer as the coordinator, the coordinator may connect using:

```text
127.0.0.1
```

However, ESP32 devices and other computers cannot use `127.0.0.1` to reach that broker.

For an ESP32 device, `127.0.0.1` refers to the ESP32 itself.

External devices must use the coordinator computer’s current Wi-Fi, Ethernet, or hotspot address. This address must be checked whenever the network changes.

On Windows, the current address can be found using:

```bash
ipconfig
```

Use the IPv4 address belonging to the active Wi-Fi, Ethernet, or mobile-hotspot adapter.

---

## Repository Structure

```text
.
├── cell_control_center.py
├── cell_control_center_config.json
├── test_control_center.py
├── requirements.txt
├── README.md
└── docs/
    └── images/
```

### `cell_control_center.py`

Main coordinator application.

It contains:

* Flask web server
* Dashboard routes
* REST API endpoints
* MQTT client
* HTTP subsystem communication
* Workflow manager
* Filled-box queue
* SCARA recipe control
* AMR control
* Conveyor and vision control
* System-state monitoring
* Logging and error handling

### `cell_control_center_config.json`

Stores editable system settings such as:

* Current subsystem addresses
* MQTT settings
* Workflow rules
* Box capacity
* Recipe mappings
* AMR operation mode
* Saved connection plan
* User-selected coordinator options

This file should be edited through the web interface whenever possible.

### `test_control_center.py`

Contains tests for:

* Coordinator routes
* Configuration loading
* Workflow behavior
* HTTP communication
* MQTT communication
* Queue handling
* Subsystem status checks

---

## Requirements

* Python 3.10 or newer
* Mosquitto MQTT broker
* All subsystems connected to the same network
* A modern web browser

Install the required Python packages:

```bash
pip install -r requirements.txt
```

Main dependencies include:

```text
Flask
requests
paho-mqtt
opencv-python
numpy
```

---

## Running the System

### 1. Connect the Devices

Connect the following devices to the same local network:

* Coordinator computer
* SCARA ESP32
* Conveyor controller
* ESP32-CAM
* AMR controller

### 2. Start the MQTT Broker

Start Mosquitto on the coordinator computer or another reachable computer.

```bash
mosquitto -v
```

### 3. Start the Coordinator

```bash
python cell_control_center.py
```

### 4. Open the Dashboard

On the coordinator computer, open:

```text
http://127.0.0.1:5050
```

To open the dashboard from another device, use the coordinator computer’s current network address followed by the configured web port.

### 5. Configure the Current Connections

Open the **Settings** page and enter the addresses currently assigned to:

* MQTT broker
* SCARA arm
* AMR
* Conveyor and vision system

Save the connection plan after verifying the addresses.

### 6. Check the Subsystems

Use the dashboard connection tools to confirm that:

* MQTT is connected
* The SCARA controller responds
* The AMR controller responds
* The conveyor controller responds
* The camera stream is available
* Required SCARA recipes are loaded

---

## Web Interface

### Main

Displays:

* Overall cell state
* Active job
* Connected services
* Filled-box queue
* Box counters
* Robot states
* Recent system activity
* Errors and warnings

### Workflow

Displays the current task sequence and workflow progress.

### Conveyor + Vision

Provides:

* Camera-stream monitoring
* Color counters
* Manual counter control
* Servo sorter calibration
* Camera color teaching
* Conveyor connection tests
* OpenCV restart controls

### Arm

Provides:

* SCARA connection status
* Homing
* Motor disable
* Abort
* Recipe synchronization
* Recipe loading
* Recipe execution
* Current arm state

### AMR

Provides:

* AMR connection status
* Manual driving tools
* Target-position commands
* Odometry reset
* Route synchronization
* Delivery completion controls
* MQTT and HTTP tests

### Settings

Provides:

* Dynamic network configuration
* MQTT configuration
* Network scanning
* Connection-plan saving
* Box-capacity configuration
* Workflow-rule configuration
* SCARA recipe mapping
* AMR operation mode

### Logs

Displays:

* Received MQTT messages
* HTTP requests
* Workflow events
* State changes
* Connection failures
* Errors and warnings

---

## Workflow Rules

Workflow rules connect conveyor events to SCARA and AMR operations.

A rule may contain:

* Object color
* Box identifier
* Filled-box event
* SCARA recipe name
* Delay before recipe execution
* Arm-loaded event
* Delivery-completed event
* Enabled or disabled state

Recipe names must exactly match the names stored in the SCARA controller.

The coordinator does not assume fixed recipe names. They are selected from the recipes currently reported by the SCARA subsystem.

---

## Screenshots

Store interface screenshots inside:

```text
docs/images/
```

Recommended filenames:

```text
main-dashboard.png
workflow-page.png
conveyor-vision-page.png
arm-page.png
amr-page.png
settings-page.png
```

Example:

```markdown
![Main Dashboard](docs/images/main-dashboard.png)
```

---

## Troubleshooting

### A Subsystem Address Changed

1. Open the coordinator Settings page.
2. Run the network scan.
3. Identify the current device address.
4. Update the subsystem address.
5. Save the connection plan.
6. Run the connection check again.

### MQTT Works Locally but ESP32 Devices Cannot Connect

Check that the ESP32 devices are using the coordinator computer’s current network address, not `127.0.0.1`.

Also check:

* MQTT broker port
* Windows Firewall
* Mosquitto listener configuration
* Wi-Fi or hotspot connection
* Whether all devices are on the same subnet

### SCARA Recipes Are Missing

Check:

* SCARA address
* SCARA HTTP server
* Wi-Fi connection
* Recipe endpoint
* Stored recipes

Then refresh the recipe list from the Arm page.

### Camera Stream Is Missing

Check:

* ESP32-CAM connection
* Current camera address
* Camera initialization
* MJPEG endpoint
* OpenCV process
* Network quality

### Wrong Workflow Recipe Starts

Check the workflow rule and confirm that its recipe name exactly matches a recipe currently stored in the SCARA controller.

---

## Safety

This software controls physical robotic equipment.

Before operation:

* Clear the working area.
* Home the SCARA arm.
* Verify the selected recipe.
* Confirm that the AMR path is clear.
* Calibrate the sorter servo.
* Check all limit switches.
* Keep emergency-stop controls accessible.
* Test each subsystem separately before integrated operation.

The coordinator must not be used as the only safety layer. Local electrical and mechanical safety protections must remain active in every subsystem.

---

## Project

This software was developed for the graduation project:

**Autonomous Robotic Classification, Transportation and Material-Handling System**

The project integrates computer vision, conveyor automation, robotic manipulation, autonomous mobile transportation, embedded control, wireless communication, and web-based supervision.

---

## Authors

* Abdulrahman Saadallah
* Muhammed Havva
* Osama Mohammed

Department of Mechatronics Engineering
Karabük University
2026
