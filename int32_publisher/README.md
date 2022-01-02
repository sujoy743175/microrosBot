# Ultrasonic Sensor to Detected Height Changes using ESP32

This example uses the example from ESP32's WiFi examples. I do not own any rights to their content rather I modified it to work with my demo. This is where you can find the original files: https://github.com/espressif/esp-idf/tree/v4.0.1/examples/wifi

## How to use example

### Configure the project

```
idf.py menuconfig
```

* Set serial port under Serial Flasher Options.

* Set WiFi SSID and WiFi Password and Maximum retry under Example Configuration Options.

* Set the MQTT Broker URL. Always starts with `mqtt://`

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.


### READ MORE and WATCH A STEP BY STEP PROCEDURE on IoT Simplified
TODO: Add link to IoT Simplified post

