# Brushless-7-Click-Arduino
This repo is mainly about the library to integrate the brushess 7 click with arduino.

![Circuit Connection Diagram](Circuit diagram.png])  
*Example connection diagram for TC78B009FTG motor driver*

Arduino library for controlling brushless motors via I2C and PWM, designed for motor drivers like the Toshiba TC78B009FTG. Provides comprehensive control over motor parameters and protection features.

## Features
- 📡 I2C configuration (100kHz default)
- 🎛️ PWM speed control (up to 23.4kHz)
- ⚙️ Multiple control modes (Duty/RPM/Stop)
- 🔄 Configurable soft-start and speed slope
- ⚠️ Over-current/thermal protection
- 📊 Current monitoring & error reporting

## Hardware Requirements
- TB67B000AHG or compatible 3-phase brushless driver
- Arduino Uno/Mega/Nano
- Brushless motor (12V-42V suggested)
- Power supply (match motor specs)
- Basic connection components:
  - 10kΩ resistor (SBY pin pull-up)
  - 0.1μF capacitor (VM pin decoupling)

## Installation
1. Download [Brushless7.zip](https://github.com/yourusername/Brushless7/archive/refs/heads/main.zip)
2. Arduino IDE: `Sketch > Include Library > Add .ZIP Library`
3. Install dependencies:
   - [Arduino Wire library](https://www.arduino.cc/en/Reference/Wire) (built-in)
   - [PWM Library](https://github.com/yourusername/PWM) (included in examples)

## Basic Usage
```cpp
#include <Brushless7.h>

// Pin definitions
#define SBY_PIN 4
#define DIR_PIN 3
#define SPD_PIN 9
#define CMO_PIN A0
#define I2C_ADDR 0x29

Brushless7 motor(SBY_PIN, I2C_ADDR, SPD_PIN, DIR_PIN, CMO_PIN);

void setup() {
  Serial.begin(115200);
  motor.begin();
  
  // Essential configurations
  motor.configureDefault();
  motor.configureBrake();
  motor.configureOCP();
  
  motor.controlModeSet(CTRL_DUTY);
  motor.setStartDuty(10.0f);
  motor.setMaxDuty(100.0f);
}

void loop() {
  motor.setSpeedPWM(200); // 0-255 PWM value
  motor.printRotationSpeedHz();
  delay(1000);
}
