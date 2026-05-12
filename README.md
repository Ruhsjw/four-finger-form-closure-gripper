# Four-Finger Robotic Gripper Motor Control Firmware

Embedded motor-control firmware for a four-finger robotic gripper, built around DJI M2006 motors and C610 motor drivers.  
The project focuses on angle-position control, PID regulation, CAN-based motor communication, and real-time debugging for robotic grasping applications.

## Project Overview

This repository contains the embedded control code used to drive and tune the actuators of a four-finger robotic gripper.  
The firmware was developed using STM32CubeIDE and targets an STM32F427-based microcontroller platform.

The main control objective is to achieve stable motor angle positioning for robotic finger motion.  
The system uses PID-based feedback control and supports real-time monitoring of motor states, target angles, and control variables through STM32CubeMonitor.

This project was part of a robotic grasping system designed for form-closure grasp experiments.

## Key Features

- Angle-position control for DJI M2006 motors with C610 motor drivers
- PID-based closed-loop motor control
- CAN communication interface for motor command and feedback
- STM32CubeIDE-based firmware configuration and development
- Real-time monitoring of motor angle, speed, and PID variables using STM32CubeMonitor
- Adjustable target angle and PID parameters for grasping behavior tuning

## Hardware Platform

- Microcontroller: STM32F427-series MCU
- Motor: DJI M2006
- Motor Driver: DJI C610 ESC
- Communication: CAN bus
- Debugging / Monitoring: STM32CubeMonitor
- Development Environment: STM32CubeIDE

## Software Architecture

The project is organized as an STM32 embedded firmware project.

Core components include:

```text
.
├── Core/                  # Main STM32 application source files
├── Drivers/               # STM32 HAL drivers and device support files
├── casia-gripper-driver.ioc # STM32CubeMX configuration file
├── STM32F427IIHX_FLASH.ld # Flash linker script
├── STM32F427IIHX_RAM.ld   # RAM linker script
├── .cproject              # STM32CubeIDE project configuration
├── .project               # Eclipse / STM32CubeIDE project metadata
└── README.md

