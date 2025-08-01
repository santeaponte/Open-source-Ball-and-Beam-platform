# Open-source-Ball-and-Beam-platform


This repository contains all the resources needed to build and control a low-cost, open-source **Ball and Beam** experimental system for control education and research.

## 🧠 About the Project

The **Ball and Beam** system is a classic benchmark in control engineering due to its unstable and nonlinear dynamics. It allows the implementation of various control strategies, from basic PID to state feedback and modern model-based control.

Unlike most online examples that use commercial servo motors (which encapsulate the control logic internally), this project uses a standard **DC motor** driven by a **linear power stage**, offering complete control over the actuation and feedback loop.

The main goal is to provide a replicable and documented platform that enables hands-on learning in automatic control.

---

## 📦 Repository Contents

- `/firmware/` – Code for the ESP32 microcontroller (Arduino-compatible)
- `/electronics/` – Schematics and PCB layouts for:
  - Power stage
  - Signal conditioning circuits
  - Sensor interfaces
- `/cad/` – 3D models and assembly drawings (designed in SolidWorks)
- `/docs/` – System overview, wiring diagrams, and build instructions
- `/control_models/` – Example MATLAB/Python scripts for modeling, simulation, and controller design

---

## ⚙️ Hardware Overview

| Component            | Description                                 |
|---------------------|---------------------------------------------|
| Actuator             | DC motor (with linear amplifier)           |
| Microcontroller      | ESP32 (dual-core, Wi-Fi + Bluetooth)       |
| Beam angle sensor    | Potentiometer (analog input)               |
| Ball position sensor | HC-SR04 Ultrasonic sensor                  |
| Power supply         | ±12V or custom linear supply               |
| Structure            | Custom 3D-printed / laser-cut mechanical parts |

---

## 🚀 Getting Started

### 1. Clone the repository
```bash
git clone https://github.com/santeaponte/Open-source-Ball-and-Beam-platform
