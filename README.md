# 🍼 STM32 Baby Room Climate Guardian 👶

**_In Progress_**  
_Bare-metal STM32F401RE temperature & humidity monitoring system_

---

## 🧸 Project Vision — From a Real Need

### Overview

As a new parent, I wanted a way to **know my baby’s room is always comfortable** — not too hot, not too dry, not too cold.  
Most monitors only capture sound or video, but **environmental comfort** has just as much impact on a baby’s sleep and health.  
This project started from that need: a small, reliable system that measures and reports room conditions in real time.

---

## 🎯 Goal

### Objective

Design a **bare-metal embedded system** that:

- Continuously measures temperature and humidity
- Sends data wirelessly to a simple dashboard
- Alerts parents when conditions go out of range

All while keeping the firmware **lightweight**, **modular**, and **built entirely from scratch** — no vendor libraries, no HAL.

---

## 🧠 Technical Overview

### Architecture

The firmware runs directly on the **STM32F401RE Nucleo** board and communicates with:

- A **BME280** sensor over **I²C** for temperature & humidity readings
- An **ESP8266** module over **UART** for Wi-Fi connectivity

### Core Focus

This project demonstrates:

- Writing **register-level peripheral drivers** (GPIO, RCC, USART, I²C)
- Working directly with **microcontroller datasheets and reference manuals**
- Understanding **clock configuration**, **bus timing**, and **low-level I/O control**

The focus isn’t just on driver design, but on using them to **solve a practical, real-world problem**.

---

## ⚙️ Features

### Current Capabilities

- Fully custom, HAL-free peripheral drivers:
  - GPIO
  - RCC (Clock Control)
  - USART (Serial Communication)
  - I²C (for BME280)
- Modular and extensible structure
- Real-time environmental monitoring logic in progress

---

## 🧰 Toolchain

### Development Environment

- **Compiler:** `arm-none-eabi-gcc`
- **Build System:** CMake 3.22+
- **Debugger:** ST-Link
- **Editor:** Visual Studio Code

---

## 📚 What I Learned / Skills Demonstrated

### Key Takeaways

- Low-level **embedded C programming** without HAL
- Building custom drivers using **direct register access**
- Deep understanding of **STM32 architecture and clock systems**
- **I²C and UART** protocol implementation and debugging
- Working with **CMake**, **GCC toolchain**, and **VS Code** for embedded workflows
- Turning a **personal problem** into a **technical solution** — bridging software engineering and real-world impact

---

## 🧾 Status

### Project Progress

This is an ongoing project — next milestones include:

- Completing the BME280 driver
- Integrating Wi-Fi data upload via ESP8266
- Building a small monitoring dashboard

---

### Author

Created and developed by **Ben Weissman**  
Computer Science student and embedded systems enthusiast.
