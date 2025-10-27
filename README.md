# STM32 Baby Room Climate Guardian 👶 - ** In Progress* 
*Bare-metal STM32F401RE temperature & humidity monitoring system*

---

## 🧠 Overview
Please note, this is a work in progress and not yet finished.
This project is a **bare-metal firmware** running on an STM32F401RE Nucleo board.  
It interfaces with a **BME280 sensor** (via I²C) for temperature and humidity monitoring and an **ESP8266 Wi-Fi module** (via UART) to send readings to a remote dashboard — forming the basis for a connected baby-room monitor.
This project showcases low-level embedded development skills, including direct register manipulation and peripheral driver design without HAL abstraction.
It reflects the ability to analyze microcontroller datasheets and reference manuals, translating hardware specifications into working C code.
The implementation emphasizes a deep understanding of clock trees, GPIO modes, bus interfaces, and serial communication protocols.
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


