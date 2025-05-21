# Brushless-7-Click-Arduino

Arduino library for controlling 3-phase brushless DC motors using the Toshiba **TC78B009FTG** driver via I²C and PWM. Supports speed control, soft-start, protection settings, and real-time diagnostics.

<p align="center">
  <img src="Circuit diagram.png" alt="Circuit Diagram" width="700">
</p>

*Example wiring for Brushless 7 Click and Arduino Uno R4 Minima*

---

## 📦 Features

- 📡 **I²C communication** (configurable registers)
- 🎛️ **PWM-based speed control** (default 23.4 kHz on pin 9)
- ⚙️ **Control modes**: duty-based, rpm, stop
- 🔄 **Startup configuration**: soft-start ramp, brake setup, closed-loop support
- ⚠️ **Protection features**: overcurrent detection, alert monitoring
- 📊 **Diagnostics**: current monitor and FG feedback for RPM measurement

---

## 🛠️ Hardware Requirements

- ✔️ **Brushless 7 Click board** (TC78B009FTG)
- ✔️ Arduino Uno R4 Minima (5 V logic recommended)
- ✔️ 3-phase brushless DC motor (12 V)
- ✔️ External 11–27 V VM power supply
- ⚠️ Ensure correct switch settings on the Click board:
  - **SW2**: Upper position (analog voltage/pwm duty), Lower position (I²C)
  - **SW3/SW4**: I²C address (default: `0x29`)

---

## 📂 Installation

1. Download this repository as `.zip`  
   [Download ZIP](https://github.com/yourusername/Brushless7/archive/refs/heads/main.zip)
2. Go to `Documents > Arduino/libraries ` to create a folder (brushless7)
3. Put the Brushless7.cpp and Brushless7.h files in this folder
4. Go to `Documents > Arduino/libraries/examples ` to create a folder (brushless7_example)
5. Put the brushless7_example.ino file in this folder
6. Open brushless7_example.ino file on Arduino IDE and upload it to the Arduino.

---

## 🔌 Pinout

| TC78B009FTG Pin | Arduino Pin | Description         |
|-----------------|--------------|---------------------|
| SDA/SCL         | SDA/SCL      | I²C communication   |
| SBY             | 4            | Standby control     |
| DIR             | 3            | Rotation direction  |
| SPD             | 9            | PWM speed input     |
| CMO             | A0           | Current monitor     |
| 3.3V            | 3.3V         | 3.3V voltage supply |
| 5V              | 5V           | 5V voltage supply   |
| INT (ALR/FG)    | optional     | Fault or speed flag |
| GND             | GND          | Ground              |

---

## 🚀 Basic Usage Example
To control the motor speed, adjust the `max_rpm` value in setSpeedSlope function. For our case, 50 and 200 are the minimum and maximum values we can set respectively.
```cpp
//setSpeedSlope(uint16_t max_rpm, uint16_t start_rpm, float start_duty, float max_duty)
motor.setSpeedSlope(200, 50, 10.0f, 100.0f);







