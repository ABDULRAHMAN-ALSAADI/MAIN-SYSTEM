WHY v9 FIXES THE CAMERA BRIDGE
================================

The log showed:
  ConnectionResetError(10054, 'An existing connection was forcibly closed by the remote host')

That means the ESP32-CAM closed the stream connection.

The reason is the old design opened the ESP32-CAM stream twice:
  1. Dashboard raw camera <img> opened http://ESP32CAM_IP:81/stream
  2. OpenCV bridge opened http://ESP32CAM_IP:81/stream

ESP32-CAM is weak. It often cannot keep multiple MJPEG clients alive.

v9 uses a single camera pipeline:
  ESP32-CAM -> CV bridge

Then the CV bridge provides two local streams:
  /raw.mjpg
  /stream.mjpg

The dashboard no longer embeds the ESP32 direct stream.
