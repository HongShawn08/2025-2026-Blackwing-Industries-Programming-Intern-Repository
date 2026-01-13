# 2025-2026 Blackwing Industries Programming Intern Repository
    This repository contains the software for a Raspberry Pi–based Unmanned Ground Vehicle (UGV) and a digital weighing scale developed for the Blackwing Industries Engineering Design & Development Company. All EDD Prog intern responsibilities reside here.

## Repository Structure
UGV/        # UGV control software
Nest/      # Weighing scale software
shared/     # Shared utilities and helpers


(files to be implemented)

ugv/
  main.py        # UGV entry point
  motors/        # Motor control interfaces
  sensors/       # UGV sensors (IMU, ultrasonic, etc.)
  control/       # Navigation & control logic

scale/
  main.py        # Scale entry point
  sensors/       # Load cell / HX711 interface
  calibration/   # Calibration routines

shared/
  utils/         # Shared helpers