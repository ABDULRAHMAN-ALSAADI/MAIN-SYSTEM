CONVEYOR CV BRIDGE v3 — COLOR DETECTION + WEB STREAM + MQTT
==============================================================

This bridge replaces the old camera_test.py window.

It does the same color detection logic:
  RED HSV mask
  GREEN HSV mask
  BLUE HSV mask
  morphology open/close
  biggest contour
  area threshold
  servo command to ESP32-CAM

But now it also:
  shows processed camera inside the Robot Cell Control Center
  publishes conveyor package messages to MQTT
  reports status at /status

Run dashboard:
  python -m pip install -r requirements.txt
  python cell_control_center.py

Open dashboard:
  http://127.0.0.1:5050

Run CV bridge:
  RUN_CONVEYOR_CV_BRIDGE.bat

Or manually:
  python conveyor_cv_bridge.py

Processed stream:
  http://127.0.0.1:5060/stream.mjpg

Status:
  http://127.0.0.1:5060/status

Edit this file before running if the ESP32-CAM IP changes:
  conveyor_bridge_config.json

Important settings:
  conveyor_camera_ip = your ESP32-CAM IP
  mqtt_broker_ip = 127.0.0.1 when running bridge on the laptop
  color_to_recipe = maps detected color to arm recipe name

Example:
  RED -> RED_SQUARE_TO_SHELF_A

The arm recipe name must match packageClass exactly.
