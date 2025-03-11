Bathroom Emergency Detection System

A real-time **emergency alert system** designed to detect incidents in private restrooms and notify staff discreetly.

---

## Project Overview
This system consists of multiple components working together to monitor and respond to emergencies in restrooms:
- **🚪 Door Sensor:** Detects door state changes (open/closed).
- **📟 Pager System:** Notifies staff via a **wireless alert mechanism**.
- **⚠️ Alert System:** Handles alert escalation and emergency signals.
- **🧠 Main MCU:** Coordinates communication between components.

---
Note: to use PLATFORMIO (as intended here), descend into your subsystem directory, it will not work at the top level!!

HAL – Provides high-level APIs that simplify peripheral control but remove low-level hardware access flexibility. (Switch to this method if RF modules aren't working before SB)

Bare-Metal – Directly interacts with hardware registers, offering full control without abstraction layers. (please please work)
