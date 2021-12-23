# BioLogger
## If you use the Arduino code for your project please cite this paper:

## Description

we describe in details an Arduino-based stand-alone logger featuring a 32-bit microcontroller, an IMU, a Bluetooth link and a SD card. Angular speeds and 3D orientations measurements (roll, pitch and yaw angle) of the hydrofoil are done at a sampling frequency of 100Hz.
Thr Biologger was originally designed for measuring inertial movement of an hydrofoil fixed to a kiteboard (see paper here: ).

## Ressources

Arduino is the main code for the Arduino Nina33 BLE including an IMU and a Bluetooth link:
https://store-usa.arduino.cc/products/arduino-nano-33-ble

The Arduino code for the Nina33 board is provided: nano33_SD_IMU_BLE_4_2_logger1

The Adafruit adalogger board was connected though a custom-made connection board to the Nina board:
https://www.adafruit.com/product/2922

The CAD file of the connection board is provided.

The attitude estimation is implemented on the basis of the Seb Madgwick's algorithm: 
https://ieeexplore.ieee.org/abstract/document/5975346

The calibration was done by using this method:
https://github.com/FemmeVerbeek/Arduino_LSM9DS1

## How to communicate with the logger

Once connected to the logger :
- To trig the logger : send an "A" (hex 41) either through Bluetooth or UART link
- To stop the logger : send a "Z" (hex 5A) either through Bluetooth or UART link
- To read the Euler angles use as an example the Python sript for Linux : bleak_loggerOK 2.py. Please install the bleak BLe client for python:
https://github.com/hbldh/bleak

The Euler angles are send via BLE link automatically ervery time a read event is generated. So it is only necessary to read the logger to receive the Euler angle values.

## If you think to use the Biologger arduino code for your project please cite this paper:
