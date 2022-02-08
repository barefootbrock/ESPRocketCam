# ESPRocketCam
A camera and sensor logger for my water rockets using the ESP32-CAM.

The Arduino program records video and data from a MPU6050 and BMP280 to a micro SD card. Video is recorded at around 5-15Hz and sensors are read at about 20Hz.

The video is recorded to a .jpgs file using a protocol I made up. It is very simple: the size of the frame (4 bytes), a jpg image (with that size), another 4 bytes, another frame, etc. This makes the code simpler and does not have a problem if power is removed at any time. "read jpgs.py" will automatically convert it to an avi video.

Flashing of the bright white light means video is being written to the SD card. If the ESP has any problems starting the recording, it will blink an error code (listed in the Arduino code) and boot in debug mode next time. Debug mode can also be accessed by restarting the ESP before it finishes booting (before the red light turns off).

A breif flash of the red light indicates debug mode. In debug mode, the ESP creates an access point and web server. This allows new code to be uploaded with OTA (/update) and has a serial monitor (/webserial).
