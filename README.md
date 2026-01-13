# 2025-2026 Blackwing Industries Programming Intern Repository

This repository contains the software for a Raspberry Pi–based Unmanned Ground Vehicle (UGV) and a digital weighing scale developed for the Blackwing Industries Engineering Design & Development Company. All EDD Prog intern responsibilities reside here.

## Repository Structure
  - UGV/        # UGV control software
  - Nest/      # Weighing scale software
  - Shared/     # Shared utilities and helpers


(files to be implemented)

UGV/
src/
  - main.cpp # UGV entry point
  - motors/ # Motor control implementations
  - sensors/ # UGV sensors (IMU, ultrasonic, etc.)
  - control/ # Navigation and control logic
  - include/ # UGV header files

Nest/
src/
  - main.cpp # Scale entry point
  - sensors/ # Load cell / HX711 interface
  - calibration/ # Calibration routines
  - include/ # Scale header files

Shared/
src/
  - include/ # Shared headers and utilities

---

## Language & Tools

- **Language:** C++
- **Compiler:** `g++` (MinGW / MSYS2 on Windows, native `g++` on Raspberry Pi)
- **Editor:** Visual Studio Code
- **Version Control:** Git + GitHub

---

### Building and Running

## UGV
From the repository root:



```bash
g++ ugv/src/main.cpp -o ugv_main
./ugv_main

## Nest

From the repository root:

g++ scale/src/main.cpp -o scale_main
./scale_main

```

On the Raspberry Pi, the same commands are used after cloning the repository.

### Hardware Overview

##UGV

  - Raspberry Pi (primary controller)

  - DC motors with motor driver (TBD)

  - Sensors (IMU, distance sensors, etc.)

## Nest

  - Load cell

  - HX711 ADC module

  - Raspberry Pi

---

### Development Workflow

1. Code is written and tested locally using Visual Studio Code.

2. Changes are committed and pushed to GitHub.

3. The repository is pulled onto the Raspberry Pi.

4. Code is compiled and executed on the Raspberry Pi for hardware testing.

---

###Project Checklist

 - Repository and build environment setup

 - Project structure defined

 - UGV motor driver implementation

 - UGV sensor integration

 - Scale sensor interface (HX711)

 - Calibration and testing

 - Hardware integration and validation

---

##Contributors

  Shawn Hong
  Rokus Kam
  Blackwing Industries Programming Dept.