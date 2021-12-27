# ESP_Thermal_Camera_WebServer

Welcome to see the micro project based on ESP32. <br>
Thermal Camera based on: <br>
1) ESP32 (Wrover)
2) MLX 90640 Thermal Camera 32x24 pixels
3) 0.95' OLED (SD1331)
4) ESPAsyncWebServer
5) SPIFFS - file system 
<br>

## Requirements
- https://github.com/me-no-dev/arduino-esp32fs-plugin
- https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino ( available in ardduino )
- https://github.com/me-no-dev/AsyncTCP
- https://github.com/me-no-dev/ESPAsyncWebServer


## Changes
- no autosave to spiffs to reduce wear
- client side parsing
- output json
- use iron flir pallette on webinterface.

## Features

Important features - TO DO LIST:
- [x] Grabbing thermal image from MLX90640
- [x] Test image on OLED
- [x] Show thermal image on OLED
- [x] Build WebServer with ESPAsyncWebServer
- [x] Automatic update of variables (e.g. MaxTemp) on Website
- [x] Save thermal image as picture (BMP) in SPIFFS
- [x] Show Thermal Image (BMP) on Website
- [x] Automatic update of BMP (suitable SetInterval in script)
- [x] Case - 3D Model and print it on FDM 3D Printer
- [ ] Tweak updating Thermal Image (now SetInterval set to 1 sec - buggy image if set <1sec or there is more Clients)
- [ ] Maybe Stream thermal image to Website (because why not) - faster updating
- [ ] Add Switch on Website to On/Off OLED display
- [ ] settings and info page
- [ ] use flir iron pallete on display

## testing

To test the html part without esp32.
( test html application/json )

The fake json data is in data/cgi-fake .

```bash
  cd ESP_Thermal_Camera_WebServer/data/html
 python -m http.server 8000
```
 use browser at http://127.0.0.1:8000

## Notes
 Note: 10/10/2019<br>
I have MLX90640 files that are 6 months old. The latest files from Melexis contain a bug in the MLX90640_API.cpp file.
