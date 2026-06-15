CV BRIDGE v4 FIX
=================

The previous bridge used cv2.VideoCapture for the ESP32-CAM stream.
On some Windows/OpenCV builds, browser streaming works but VideoCapture does not
process frames reliably.

v4 fixes this by:
  reading the ESP32-CAM MJPEG stream with requests
  extracting JPEG frames manually
  decoding frames with cv2.imdecode
  running the same RED/GREEN/BLUE HSV detection
  serving processed frames at /stream.mjpg

Run:
  RUN_CONVEYOR_CV_BRIDGE.bat

Then open:
  http://127.0.0.1:5060

Status:
  http://127.0.0.1:5060/status

Processed stream:
  http://127.0.0.1:5060/stream.mjpg

If processed stream shows "Camera error":
  edit conveyor_bridge_config.json and confirm conveyor_camera_ip.
