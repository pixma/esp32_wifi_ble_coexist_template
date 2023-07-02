<h3 align="center">ESP32 WiFi BLE Co-exist Project Template with idf</h1>

<p align="center">
  <a href="#">
    <img src="https://img.shields.io/badge/ESP32S3-8MB-blue" alt="Example Badge">
  </a>
</p>


This project template is designed to provide a starting point for building projects that require both WiFi and BLE functionality on the ESP32 microcontroller. The template is written in C language and uses the ESP-IDF (Espressif IoT Development Framework).

This template is comprised of BLE SSP Server role along with Wi-Fi in STA or AP mode based on the configuration and credentials if present.

**Table of Content**

- [Folder Structure](#folder-structure)
- [Getting Started](#getting-started)
- [Features](#features)
- [Application](#application)
- [sdconfig.defaults](#sdconfigdefaults)
- [Template Details](#template-details)
  - [Partiton Map](#partiton-map)


---

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

---


#### Getting Started

To use this template, you will need an ESP32 SoC, ESP-IDF installed, and a toolchain for building the code. Follow these steps to get started:

1. Clone this repository to your local machine:

```
~/>git clone https://github.com/pixma/esp32_wifi_ble_coexist_template.git
```
2. On VSCode, Configure your Target Device to ESP32S3 or related module by ```Ctrl + Shift + P -> ESP-IDF: Device Configuration.```. By doing this your local project's sdkconfig will get updated for your desired-selected SoC.
3. Do check and edit your other config via ```menuconfig```.
4. Build, flash and monitor your ESP32 SoC/Module.

#### Features
This template includes the following features:
- Co-existence between WiFi and BLE
- Basic WiFi functionality for connecting to a network and sending/receiving data.
- Basic BLE functionality for advertising and getting connected to a peripheral device.
- Basic MQTT Client to demonstrate the ease of Interacting with ESP32 SoC when BLE & Wi-Fi Co-Exist.

#### Application
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
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y    # Flash Related Config as per your Module
CONFIG_ESPTOOLPY_FLASHSIZE="8MB"    # Flash Related Config as per your Module
CONFIG_ESPTOOLPY_BEFORE_RESET=y
CONFIG_ESPTOOLPY_BEFORE="default_reset"
CONFIG_ESPTOOLPY_AFTER_RESET=y
CONFIG_ESPTOOLPY_AFTER="hard_reset"
CONFIG_ESPTOOLPY_MONITOR_BAUD=115200
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y # CPU Clock related config as per your module and SoC
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240   # CPU Clock related config as per your module 
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
- This template has *some incomplete code under WebServer REST API Dashboard URL Page*.
- This template demonstrates:
  - How to Enable Wi-Fi and BLE to Co-Exist on single available Antenna for ESP32 SoC.
  - How to pack mini website comprised of HTML and CSS with inFile JS Script and to be placed in Flash with the help of SPIFF for ESP32.
  - Enables MQTT Client over WiFi/Internet In order to create a scenario of Heavy traffic to Check if any visible Stuttering or Packet Loss which BLE & WiFi enabled.

---

##### Partiton Map

ESP32 SoC Partiton Map for 8MB Flash:
```
# ESP-IDF Partition Table (S3 - 8MB)
# ---------------------------------
# nvs for settings :        @x9000      for 24kB    (0x6000)
# phy_init :                @xF000      for 4kB     (0x1000)
# factory app code area:    @x10000     for 5MB     (0x500000) : 5120 kB
# storage data for spiffs:  @x510000    for 2560kB  (0x280000) : 2.5MB
# Unused - ~ 448kb. [Not used for now and not allocated].
#
# OTA : Not mentioned or arranged - 
# If required you may have to add "otadata" ~ 8kB and "ota_0" ~ > 1MB.
# ---------------------------------
# Name, Type, SubType, Offset, Size, Flags
nvs,data,nvs,0x9000,0x6000,,
phy_init,data,phy,0xF000,0x1000,,
factory,app,factory,0x10000,0x500000,,
storage,data,spiffs,0x510000,0x280000,,

```

| Partition Name | Type | SubType | Offset | Size | kB / MB |
| --- | --- | --- | --- | --- | --- |
| nvs | data | nvs | 0x9000 | 0x6000 | 24kB |
| phy_init | data | phy | 0xF000 | 0x1000 | 4kB |
| factory | app | factory | 0x10000 | 0x500000 | 5120kB |
| storage | data | spiffs | 0x510000 | 0x280000 | 2560kB |

No OTA partition is mentioned or arranged. If required you may have to add "otadata" ~ 8kB and "ota_0" ~ > 1MB.

- Link to an excel sheet for calculating partition : [ESP32 Partition Generator: https://docs.google.com/spreadsheets/d/1M6_qx00ZR82jyWa2slCMcvLtnXR9Q9dnis5mR5YChW8/edit?pli=1#gid=0]([https://](https://docs.google.com/spreadsheets/d/1M6_qx00ZR82jyWa2slCMcvLtnXR9Q9dnis5mR5YChW8/edit?pli=1#gid=0))


- For more details on performance, size etc check here [Performance Size: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/performance/size.html]([https://](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/performance/size.html))


**Last lines at the build** completion
```
esptool.py v4.5.1
Creating esp32s3 image...
Merged 2 ELF sections
Successfully created esp32s3 image.
Generated <your_project_dir>/WorkLabs/ESP32_BLE_WiFi_ClientApp/clientApp/build/bleWiFiClientApp.bin
[8/8] cd <your_project_dir>/WorkLabs/ESP32_B.../clientApp/build/bleWiFiClientApp.bin
bleWiFiClientApp.bin binary size 0x16ea20 bytes. Smallest app partition is 0x500000 bytes. 0x3915e0 bytes (71%) free.
```


**Upon Build** : Size related output:
```
Total sizes:
Used static IRAM:   88686 bytes ( 273554 remain, 24.5% used)
      .text size:   87659 bytes
   .vectors size:    1027 bytes
Used stat D/IRAM:   48020 bytes ( 297836 remain, 13.9% used)
      .data size:   18300 bytes
      .bss  size:   29720 bytes
Used Flash size : 1396251 bytes
      .text     : 1061503 bytes
      .rodata   :  334492 bytes
Total image size: 1503237 bytes (.bin may be padded larger)
```

---
END.