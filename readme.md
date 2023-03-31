<h3 align="center">ESP32 WiFi BLE Co-exist Project Template with idf</h1>

<p align="center">
  <a href="#">
    <img src="https://img.shields.io/badge/ESP32-4MB-yellowgreen" alt="Example Badge">
  </a>
</p>


This project template is designed to provide a starting point for building projects that require both WiFi and BLE functionality on the ESP32 microcontroller. The template is written in C language and uses the ESP-IDF (Espressif IoT Development Framework).

This template is comprised of BLE SSP Server role along with Wi-Fi in STA or AP mode based on the configuration and credentials if present.

#### Folder Structure
```
├── clientApp.code-workspace
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── inc
│   │   ├── ble_activity.h
│   │   ├── main.h
│   │   └── wifi_activity.h
│   ├── Kconfig.projbuild
│   ├── main.c                  # Main App Entry Point.
│   └── src                     # No files inside this folder for now.
├── partition_table
│   ├── partitionTable.bin
│   └── partitionTable.csv
├── readme.md
├── sdkconfig
├── sdkconfig.defaults          # default config defines here which should reflect on menuconfig.
├── sdkconfig.defaults.esp32c2
├── sdkconfig.defaults.esp32c3
├── sdkconfig.defaults.esp32h2
├── sdkconfig.defaults.esp32s3
├── sdkconfig.old
├── spiffs
│   ├── dashboard.html
│   ├── index.html
│   ├── siimple-icons.css
│   └── siimple.min.css
└── ssp_server_example_readme.md    # Default readme of ssp ble esp32 example project.
```


#### Getting Started

To use this template, you will need an ESP32 SoC, ESP-IDF installed, and a toolchain for building the code. Follow these steps to get started:

1. Clone this repository to your local machine:

```
~/>git clone https://github.com/pixma/esp32_wifi_ble_coexist_template.git
```
2. Open this project or folder using VSCode for quickly build, flash and monitor your ESP32 SoC.

#### Features
This template includes the following features:
- Co-existence between WiFi and BLE
- Basic WiFi functionality for connecting to a network and sending/receiving data.
- Basic BLE functionality for advertising and getting connected to a peripheral device.
- Basic MQTT Client to demonstrate the ease of Interacting with ESP32 SoC when BLE & Wi-Fi Co-Exist.

#### Usage
You can use this template as a starting point for your own ESP32 projects that require both WiFi and BLE functionality. Modify the code to fit your specific requirements, and use the ESP-IDF documentation as a reference for additional functions and libraries.

#### sdconfig.defaults
```
CONFIG_BT_ENABLED=y                             #Enable BLE
CONFIG_ESP_WIFI_SSID="<Your SSID Name>"
CONFIG_ESP_WIFI_PASSWORD="Your PassKey"
CONFIG_ESP_MAXIMUM_RETRY=5
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y

# Custom MQTT related defines: Should reflect in menuconfigme
CONFIG_BROKER_URL="mqtt://mqtt.eclipseprojects.io"
CONFIG_BROKER_TOPIC_TO_PUBLISH="/topic/dual-stack/node/uplink"
CONFIG_BROKER_TOPIC_TO_SUBSCRIBE="/topic/dual-stack/node/downlink"

# partition related defines
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="./partition_table/partitionTable.csv"
CONFIG_PARTITION_TABLE_FILENAME="./partition_table/partitionTable.csv"
CONFIG_PARTITION_TABLE_OFFSET=0x8000

CONFIG_ESP_SOFTAP_WIFI_SSID="BWM-NODE"
CONFIG_ESP_SOFTAP_WIFI_PASSWORD="passWord99*"
CONFIG_ESP_SOFTAP_MAX_STA_CONN=4
CONFIG_ESPTOOLPY_FLASHFREQ="80m"
CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="4MB"
CONFIG_ESPTOOLPY_BEFORE_RESET=y
CONFIG_ESPTOOLPY_BEFORE="default_reset"
CONFIG_ESPTOOLPY_AFTER_RESET=y
CONFIG_ESPTOOLPY_AFTER="hard_reset"
CONFIG_ESPTOOLPY_MONITOR_BAUD=115200
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240
CONFIG_HTTPD_MAX_REQ_HDR_LEN=1024
CONFIG_HTTPD_MAX_URI_LEN=512
CONFIG_HTTPD_ERR_RESP_NO_DELAY=y
CONFIG_HTTPD_PURGE_BUF_LEN=32
```
**Note:** After running menuconfig from terminal or from VSCode, Do Check the"Software Controls WiFi/Bluetooth Coexistence".

#### Template Details

- app_main() [Start]
- checkSpiffSystem() [Checks SPIFF File System Partition]
- [Process Block] (Steps to initiate NVS Flash + BLE Init & register relevant handlers)
- Decision to Make using function call "whichMode()"
This decision is made by checking 
    - If Wifi Credentials found then go to STA_MODE.
    - Else, go to AP Mode where it will start a http Web Server with REST APIs.

- When connected to Provided or saved Router AP Creds, MQTT Client initiates and tries to connect to broker server.
- This template has some incomplete code under WebServer REST API Dashboard URL Page.
- This template demonstrates:
  - How to Enable Wi-Fi and BLE to Co-Exist on single available Antenna for ESP32 SoC.
  - How to pack mini website comprised of HTML and CSS with inFile JS Script and to be placed in Flash with the help of SPIFF for ESP32.
  - Enables MQTT Client over WiFi/Internet In order to create a scenario of Heavy traffic to Check if any visible Stuttering or Packet Loss which BLE & WiFi enabled.
