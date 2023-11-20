# MAXON EPOS4 Interface

This repository contains project files build for STM32F4 MCU that interfaces with EPOS4 controllers, via CANOpen communication.

Project build includes:

  - Basic CANOpen Protocol stack for basic communication with MAXON controller
  - FreeRTOS usage for task management and work scheduling over real time control applications
  - USB Interface for bidirectional Tx/Rx communication
  - Main CAN functionalities that handle CANOpen physical layer, based on STM32 HAL libraries 
  - ADC readings and PID implementation with Linear Sensor feedback for improved motor control
