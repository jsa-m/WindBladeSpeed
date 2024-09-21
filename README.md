# WindBladeSpeed

WindBladeSpeed is a project designed to measure the speed of windmill blades using a microcontroller and a 9-axis sensor (accelerometer, magnetometer, and gyroscope). This setup captures real-time rotational data and extracts the wind blade speed from an equation obtained with a regression analysis then sends it over Bluetooth Low Energy (BLE) to a connected device. 

This aplicattion could be usefull for example for maintenance prediction, abnormal speeds could indicate potential mechanical issues or wear and tear, allowing for predictive maintenance and reducing downtime

This repository contains three key source codes:

## WINDBLADESPEED_MOUNTED
This section is responsible for reading and processing data from the ESP32, 9-axis sensor (accelerometer, magnetometer, gyroscope), and a 
pressure sensor to estimate environmental parameters like wind speed and sends it to BLE client.

## WINDBLADESPEED_LOGGER
This application connects to a BLE server (another ESP32) to retrieve sensor data using BLE notifications. It reads data and sends it
over a serial interface. Used for calibration.

## WINDBLADESPEED_APP
Final application responsible to transmit the wind blade speed extracted from two calibrated regression equations over over Bluetooth Low Energy (BLE) to a connected device.
