#include <Brushless7.h>
#include "pwm.h"

// pin assignments
#define INT_PIN   2
#define DIR_PIN   3
#define SBY_PIN   4
#define SPD_PIN   9
#define CMO_PIN   A0
#define I2C_ADDR 0x29
// #define NVM_CMD_REG 0x86    // NVM command register
// #define NVM_STA_REG 0x87    // NVM status register

Brushless7 motor(SBY_PIN, I2C_ADDR, SPD_PIN, DIR_PIN, CMO_PIN, INT_PIN); // Initialize motor object
PwmOut objPWMD9(SPD_PIN); // Set pin 9 object

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("[Example] Starting Brushless7...");
  
  motor.begin(); 
  //set OUTPUT FREQ at 23.4 khz & 0% duty (default is 490 Hz for pin 9)
  objPWMD9.begin(23400.0f, 0.0f); 

  Wire.begin();
  Wire.setClock(100000);

  // --- Blocking Loop: Wait Until Device is Found ---
  bool deviceFound = false;
  while (!deviceFound) {
    Wire.beginTransmission(I2C_ADDR);
    delay(100);
    if (Wire.endTransmission() == 0) {
      deviceFound = true;
      Serial.println("I2C found at 0x29!");
    } else {
      Serial.println("Device not found. Retrying...");
      delay(500);  // Wait before retrying
    }
  }

  // Serial.println("\nReading back values:");
  // for (uint8_t reg = 1; reg <= 24; reg++) {
  //   uint8_t out;
  //   bool ok = motor.readRegister(reg, out);
  //   Serial.print("  Reg ");
  //   if (reg < 10) Serial.print(' ');
  //   Serial.print(reg);
  //   Serial.print(" → ");
  //   if (ok) {
  //     Serial.print("0b");
  //     //if (out < 16) Serial.print('0');
  //     Serial.print(out, BIN);
  //   } else {
  //     Serial.print("READ ERR");
  //   }
  //   Serial.println();
  //   //delay(500);
  // }

  motor.configureDefault();
  motor.configureBrake(); 
  motor.configureStartup();
  motor.configureOCP();
  motor.configureSoftStart();
  motor.configureMaxSpeed();
  motor.configureSpeedControl();
  motor.configureCommutation();
  motor.setDirection(0);

  motor.controlModeSet(CTRL_DUTY);

  motor.setMaxDuty(100.0f);
  motor.setStartDuty(10.0f);  
  motor.setStopDuty(5.0f);
  motor.setStartRPM(50);
  motor.setSpeedSlope(200, 50, 10.0f, 100.0f);
  // motor.setMaxOpen(true);
  // motor.setMaxOff(false);
}

void loop() {
  static float D_perc = 99.0f;
  objPWMD9.pulse_perc(D_perc);
  //motor.printRotationSpeedHz();
  //motor.printErrors();
  delay(1000);
}
