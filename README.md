# Arduino Satellite Antenna
Arduino code for RV and Marine Satellite Dish Control

## Overview
The goal of this project is to develop a simple, but sophisticated, code base for controlling pointing of RV and Marine satellite antennas.
These antennas can be used for a variety of purposes, including for example Amateur Radio 10 GHz and satellite tracking.
For now the immediate goal is to document features of the antenna and develop an arduino based control scheme.
Future versions may include Raspberry Pis, SDRs, and GNU Radio.
Control is via USB at the moment, and this repo may contain companion Python code for talking to the arduino.
I also intend to document the specific Antennas I choose to modify, including voltages, wiring interfaces, etc.

### Controller Hardware, Software, & Tutorials
I tend to favor Adafruit Feather Wings:
- Microcontroller: Adafruit Feather M0 Adalogger
  - Hardware: [Adafruit Feather M0 Adalogger](https://www.adafruit.com/product/2796)
  - Tutorial: [Adafruit Tutorial](https://learn.adafruit.com/adafruit-feather-m0-adalogger/)
- Motor Controller: DC Motor + Stepper FeatherWing Add-on For All Feather Boards
  - Hardware: [DC Motor + Stepper FeatherWing Add-on For All Feather Boards](https://www.adafruit.com/product/2927)
  - Software: [Adafruit_Motor_Shield_V2_Library](https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library)
  - Tutorial: [Adafruit Stepper + DC Motor FeatherWing](https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/overview)
- Sensor (not a feather): 
  - Hardware: [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 - STEMMA QT / Qwiic](https://www.adafruit.com/product/4646)
  - Software: [Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055)
  - Tutorial: [Adafruit BNO055 Absolute Orientation Sensor](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
- Sensor Interface:
  - For connecting the BNO055 to the Featherwing stack
  - [SparkFun Qwiic / Stemma QT FeatherWing (Shield for Thing Plus)](https://www.adafruit.com/product/4515)

## Winegard G2 Carryout
I am starting with this RV antenna for modification.
This antenna includes the following features:
- Solid metal 30cm parabolic reflector antenna with subreflector
- rear mounted LNB interface, with options for different feed mounting (with a little metal fabrication)
- Solid aluminum base.
  - Can be modified for full travel to 90 degrees elevation by cutting away some of the metal.
  - Minimum elevation is about 20 degrees.
  - over 360 degrees of azimuth travel
- Stepper motors with timing belt interfaces to the axes of rotation.
- Solid mechanical stops
- Cable pass through with two RF paths and DC power.  I have modified this by removing one RF path and installing USB instead.
