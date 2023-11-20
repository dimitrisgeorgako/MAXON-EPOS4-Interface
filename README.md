# MAXON EPOS4 Interface with STM32 family MCUs

This repository contains project files build for STM32F4 MCU, but can be integrated into multiple ARM processor project files. 
It provides the basic interface with EPOS4 MAXON BLDC motor controllers for real time applications, via CANOpen communication, as well as 
appropriate error handling.

Project build includes custom libraries and src files, containing:

  - Basic CANOpen Protocol stack for basic communication with MAXON controller
  - FreeRTOS usage for task management and work scheduling over real time control applications
  - USB Interface for bidirectional Tx/Rx communication as host device
  - Main CAN functionalities that handle CANOpen physical layer, based on STM32 HAL libraries 
  - ADC readings and PID implementation with Linear Sensor feedback for improved motor control

Main CANOpen libraries can be found inside files: ./Core/Src/CAN_Open_Functions.c and ./Core/Inc/CAN_Open_Functions.h...
USB stack Rx messages are handled via Interrupt mode, based on: ./USB_DEVICE/App/usb_cdc_if.c: static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)

CANOpen network information can be viewed on: https://www.csselectronics.com/pages/canopen-tutorial-simple-intro
