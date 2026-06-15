MQTT CELL CONTROL CENTER v33 — ARM HTTP ONLINE FIX
=================================================

Fixes the confusion:
  - Arm web reachable at http://ARM_IP/job/status means the arm is CONNECTED.
  - State NOT_HOMED is normal before homing. It should not be treated as disconnected.
  - Port 1883 is MQTT broker on the laptop.
  - ESP32 arm web/API uses HTTP port 80.

New:
  - Verify Arm IP button.
  - /api/arm/save-ip-and-verify endpoint.
  - HTTP test now parses /job/status and marks arm state as NOT_HOMED / READY / etc immediately.

Documentation:
  - SYSTEM_INTEGRATION_GUIDE.md
  - BUG_AUDIT_REPORT.md

First-time MQTT setup:
  - Run SETUP_MQTT_BROKER_ADMIN.ps1 from PowerShell as Administrator.
  - This makes Mosquitto reachable from ESP32 devices at 192.168.137.1:1883.

If arm returns:
  {"ok":true,"state":"NOT_HOMED",...}
then the dashboard should show the arm as reachable, and the next action is Startup Confirm + Home.
