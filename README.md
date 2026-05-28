# Smart Solar Monitor ESP32

An ESP32-based Smart Solar Power Monitoring and Control System built with ESP-IDF.  
The project monitors solar charging, grid charging availability, battery voltage, environmental temperature, daylight condition, and AC load energy consumption using a PZEM-004T energy meter.

The system is designed for small DC/AC solar installations where a battery is charged primarily from solar, with grid charging available as backup.

---

## Project Overview

This project uses an ESP32 microcontroller to monitor and control a smart solar power system. It captures real-time system parameters, displays them on a 20x4 I2C LCD, and prepares the system for future IoT data upload.

The project focuses on:

- Solar charging monitoring
- Grid adapter/charger detection
- Battery voltage monitoring
- LDR-based daylight detection
- Ambient temperature and humidity monitoring
- AC load energy monitoring using PZEM-004T
- Relay-based grid charger control
- LCD-based real-time system display
- Buzzer alerts for abnormal conditions
- IoT-ready architecture

---

## System Architecture

```text
Solar Panel
   ↓
Solar Charge Controller
   ↓
Battery
   ↓
Inverter
   ↓
PZEM-004T
   ↓
AC Load


Grid AC
   ↓
AC-DC Adapter/Charger
   ↓
Relay Module
   ↓
Battery
