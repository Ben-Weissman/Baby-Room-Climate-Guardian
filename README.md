# STM32 Baby Room Climate Guardian 👶 - ** In Progress* 
*Bare-metal STM32F401RE temperature & humidity monitoring system*

---

## 🧸 Project Vision: Baby Room Climate Guardian
Problem<br>

<sub>Newborns are highly sensitive to temperature and humidity.<br>
Most baby monitors only track sound or video, not the environmental comfort that affects sleep quality, breathing, and skin health.<br>
Parents often don’t know when the room becomes too hot, cold, or dry — especially at night.<br></sub>

Goal<br>

<sub>Build a smart baby-room monitor that continuously measures and reports environmental conditions, notifies parents of anomalies, and logs data for trend analysis — reliable, low-cost, and safe.</sub>

## 🧠 Overview
Please note, this is a work in progress and not yet finished.<br>
<sub>This project is a **bare-metal firmware** running on an STM32F401RE Nucleo board.<br>
It interfaces with a **BME280 sensor** (via I²C) for temperature and humidity monitoring and an **ESP8266 Wi-Fi module** (via UART) to send readings to a remote dashboard — forming the basis for a connected baby-room monitor.<br>
This project showcases low-level embedded development skills, including direct register manipulation and peripheral driver design without HAL abstraction.<br>
It reflects the ability to analyze microcontroller datasheets and reference manuals, translating hardware specifications into working C code.<br>
The implementation emphasizes a deep understanding of clock trees, GPIO modes, bus interfaces, and serial communication protocols.<br></sub>
---

## ⚙️ Features
- Custom **register-level drivers** (no HAL)
  - GPIO
  - RCC (clock control)
  - USART (serial communication)
  - I²C (for BME280)
- Modular structure for easy extension.
---


---

## 🧰 Toolchain
- **Compiler:** `arm-none-eabi-gcc`
- **CMake:** 3.22+
- **Debugger/Flasher:** ST-Link
- **Editor:** Visual Studio Code with ST extensions


