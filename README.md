# 🍼 STM32 Baby Room Climate Guardian 👶  
**_In Progress_**  
*Bare-metal STM32F401RE temperature & humidity monitoring system*

---

## 🧸 Project Vision — Baby Room Climate Guardian

### 🩵 Problem
Newborns are highly sensitive to temperature and humidity.  
Most baby monitors only track sound or video — not the environmental comfort that affects sleep quality, breathing, and skin health.  
Parents often don’t know when the room becomes too hot, cold, or dry — especially at night.

### 🎯 Goal
Build a smart baby-room monitor that continuously measures and reports environmental conditions, notifies parents of anomalies, and logs data for trend analysis — reliable, low-cost, and safe.

---

## 🧠 Overview
> **Note:** This project is a work in progress and not yet finished.

This project is a **bare-metal firmware** running on an **STM32F401RE Nucleo** board.  
It interfaces with a **BME280 sensor** (via I²C) for temperature and humidity monitoring and an **ESP8266 Wi-Fi module** (via UART) to send readings to a remote dashboard — forming the basis for a connected baby-room monitor.

This project showcases low-level embedded development skills, including direct register manipulation and peripheral driver design without HAL abstraction.  
It reflects the ability to analyze microcontroller datasheets and reference manuals, translating hardware specifications into working C code.  
The implementation emphasizes a deep understanding of:
- Clock trees  
- GPIO modes  
- Bus interfaces  
- Serial communication protocols

---

## ⚙️ Features
- Custom **register-level drivers** (no HAL)
  - GPIO  
  - RCC (Clock Control)  
  - USART (Serial Communication)  
  - I²C (for BME280)
- Modular structure for easy extension

---

## 🧰 Toolchain
- **Compiler:** `arm-none-eabi-gcc`  
- **CMake:** 3.22+  
- **Debugger/Flasher:** ST-Link  
- **Editor:** Visual Studio Code with STM32 extensions  

---
