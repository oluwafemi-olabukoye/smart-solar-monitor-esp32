# ESP32 Smart Solar Power Monitoring and Control System

An ESP32-based smart solar monitoring and control firmware built with ESP-IDF 5.2.  
The system monitors solar, grid adapter, battery voltage, ambient temperature, daylight condition, and AC load energy consumption using PZEM-004T. It also supports LCD display, relay-based grid charger control, buzzer alerts, and future IoT data upload.

## Features

- Battery voltage monitoring
- Solar voltage monitoring
- Grid adapter detection
- LDR-based daylight detection
- DHT22 temperature and humidity monitoring
- 20x4 I2C LCD display
- Relay control for grid charger connection
- Buzzer alert system
- PZEM-004T AC energy monitoring
- Modular ESP-IDF firmware structure
- Prepared for Wi-Fi/IoT data upload

## System Architecture

```text
Solar Panel → Solar Charge Controller → Battery

Grid AC → AC-DC Adapter/Charger → Relay → Battery

Battery → Inverter → PZEM-004T → AC Load
