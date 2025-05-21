#include "Brushless7.h"

Brushless7::Brushless7(uint8_t sby_pin, uint8_t i2c_addr, uint8_t spd_pin, uint8_t dir_pin, uint8_t cmo_pin, uint8_t int_pin)
  : _sby_pin(sby_pin), _i2c_addr(i2c_addr), _spd_pin(spd_pin), _dir_pin(dir_pin), _cmo_pin(cmo_pin), _int_pin(int_pin)
{ 
}

void Brushless7::begin() {
  pinMode(_sby_pin, OUTPUT);
  digitalWrite(_sby_pin, HIGH); // For normal operation (8.1.1); if LOW: standby mode; addr 14[7]
  // Speed PWM
  pinMode(_spd_pin, OUTPUT);

  // Direction
  pinMode(_dir_pin, OUTPUT);
  digitalWrite(_dir_pin, HIGH);

  // Current monitor (analog input)
  pinMode(_cmo_pin, INPUT);
}

//  Default Configuration
bool Brushless7::configureDefault() {
  Serial.println("[Brushless7] Applying default configuration...");

  struct RegInit {
    uint8_t reg;
    uint8_t val;
  };
  const RegInit defaults[] = { // Reg 2 to 24, add more if needed
    { REG_STOP_DUTY, 0b10000000 }, //nOSTOP [7]=1
    { REG_START_DUTY, 0b00000000 },
    { REG_CHANGE_DUTY, 0b00000000 },
    { REG_MAX_DUTY, 0b00000000 },
    { REG_STARTRPM_LOW, 0b00000000 },
    { REG_STARTRPM_HIGH, 0b00000000 },
    { REG_SPEEDSLOPE_LOW, 0b00000000 },
    { REG_SPEEDSLOPE_HIGH, 0b00000000 },
    { REG_SPEEDSLOP2, 0b00000000 },
    { REG_SPEEDSLOP2_VCPMASK_OPENLOOP, 0b00000010 }, //11 [0]:OL=1 [1]:disable  CPVSD (git: )
    { REG_KIX_KI, 0b00000000 }, //12   (git: 01111111)
    { REG_KPX_KP, 0b00000000 }, //13   (git: 01111111)
    { REG_STBYMODE_DIR_POLEPAIR_MAXSPEED_FGON, 0b01111101 }, //14: 0 0 111 11 1 (git: 0 0 110 11 1)
    { REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK, 0b00000011 }, //15:   (git: 000 1 0 0 01)
    { REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS, 0b00011110 }, //16:  (git: 00010000)
    { REG_SSADDSEL_SSUPSEL_SSDUTYCHGLIMIT_DUTYUPTIME, 0b11000011 }, //17:
    { REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH, 0b00001111 }, //18:   (git: 00000001) [3]=1 disable isd
    { REG_BRAKE_CONFIG, 0b00000011 }, //19: 0b00000 0 1 0 [0]:disable
    { REG_TRE_PRETIP_TIP, 0b10100010 }, //20  (git: 0b10100010)
    { REG_LA_FMAX_FST, 0b00001100 }, //21
    { REG_SLOP_LAP_FPWM_DEADTIME, 0b10000010 }, //22 
    { REG_ISDLVL_OCPLVL_SOURCE_SINK, 0b01011111 }, //23 (git: 00111111)
    { REG_COMP_HYS, 0b01000000 }, //24
    //{ REG_SPD_LOW, 0b01010100}, //27 spd low 
    //{ REG_SPD_HIGH, 0b10000000} //28 spd high 
    { REG_SPD_LOW, 0b00000000}, //27 spd low 
    { REG_SPD_HIGH, 0b00000000} //28 spd high  
  };

  bool success = true;

  for (int i = 0; i < sizeof(defaults) / sizeof(defaults[0]); i++) {
    uint8_t reg = defaults[i].reg;
    uint8_t val = defaults[i].val;

    if (!writeRegister(reg, val)) {
      Serial.print("[Brushless7] Failed to write reg 0x");
      Serial.println(reg, HEX);
      success = false;
    } else {
      Serial.print("[Brushless7] Wrote 0b");
      for (int b = 7; b >= 0; b--) Serial.print((val >> b) & 1);
      Serial.print(" -> reg 0x");
      Serial.println(reg, HEX);
    }
  }

  return success;
}

//START DUTY
bool Brushless7::setStartDuty(float duty_percent) {
  Serial.print("[Brushless7] Setting start duty: ");
  Serial.print(duty_percent);
  Serial.println("%");

  if (duty_percent < START_DUTY_MIN || duty_percent > START_DUTY_MAX) {
    Serial.println("[Brushless7] ERROR: Duty out of range!");
    return false;
  }
  
  uint16_t raw_value = (uint16_t)(duty_percent * START_DUTY_SCALE);
  Serial.print("[Brushless7] Raw I2C value: 0x");
  Serial.println(raw_value, HEX);

  bool success = writeRegister(REG_START_DUTY, (uint8_t)(raw_value & 0xFF));
  Serial.println(success ? "[Brushless7] I2C write SUCCESS":"[Brushless7] I2C write FAILED");
  return success;

}

//STOP DUTY
bool Brushless7::setStopDuty(float duty_percent) {
  Serial.print("[Brushless7] Setting stop duty: ");
  Serial.print(duty_percent);
  Serial.println("%");

  if (duty_percent < STOP_DUTY_MIN || duty_percent > STOP_DUTY_MAX) {
    Serial.println("[Brushless7] ERROR: Duty out of range!");
    return false;
  }
  
   // Convert percentage to 8-bit value (0-255), then mask to 7 bits
  uint8_t raw_value = (uint8_t)(duty_percent * STOP_DUTY_SCALE) & 0x7F;

  Serial.print("[Brushless7] Raw I2C value: 0x");
  Serial.println(raw_value, HEX);

  // Read current register, CLEAR bit 7 (NOSTOP=0), set new duty
  uint8_t existing_reg;
  if (!readRegister(REG_STOP_DUTY, existing_reg)) {
    Serial.println("[Brushless7] ERROR: Failed to read STOP_DUTY reg");
    return false;
  }
  
  uint8_t new_value = (existing_reg & 0x80) | raw_value; // Keep bit 7 if needed
  // OR force NOSTOP=0:
  // uint8_t new_value = raw_value; // Directly write (assumes bit 7=0)

  bool success = writeRegister(REG_STOP_DUTY, (uint8_t)(raw_value & 0xFF));
  Serial.println(success ? "[Brushless7] I2C write SUCCESS":"[Brushless7] I2C write FAILED");
  return success;
}

//CHANGE DUTY
bool Brushless7::setChangeDuty(float duty_percent) {
  // Serial.print("[Brushless7] Setting change duty: ");
  // Serial.print(duty_percent);
  // Serial.println("%");

  if (duty_percent < CHANGE_DUTY_MIN || duty_percent > CHANGE_DUTY_MAX) {
    Serial.println("[Brushless7] ERROR: Duty out of range!");
    return false;
  }
  
  uint16_t raw_value = (uint16_t)(duty_percent * CHANGE_DUTY_SCALE);
  Serial.print("[Brushless7] Raw I2C value: 0x");
  Serial.println(raw_value, HEX);

  bool success = writeRegister(REG_CHANGE_DUTY, (uint8_t)(raw_value & 0xFF));
  Serial.println(success ? "[Brushless7] I2C write SUCCESS":"[Brushless7] I2C write FAILED");
  return success;
}

//MAX DUTY
bool Brushless7::setMaxDuty(float duty_percent) {
  // Serial.print("[Brushless7] Setting max duty: ");
  // Serial.print(duty_percent);
  // Serial.println("%");

  if (duty_percent < MAX_DUTY_MIN || duty_percent > MAX_DUTY_MAX) {
    Serial.println("[Brushless7] ERROR: Duty out of range!");
    return false;
  }
  
  uint16_t raw_value = (uint16_t)(duty_percent * 5.12f - 257.0f + 0.5f); //0.5f for round up
  Serial.print("[Brushless7] Raw I2C value: 0x");
  Serial.println(raw_value, HEX);

  bool success = writeRegister(REG_MAX_DUTY, (uint8_t)(raw_value & 0xFF));
  Serial.println(success ? "[Brushless7] I2C write SUCCESS":"[Brushless7] I2C write FAILED");
  return success;
}

// START RPM
bool Brushless7::setStartRPM(uint16_t rpm) {
  // rpm must range between 0 - 4095 rpm
  if (rpm < STARTRPM_MIN || rpm > STARTRPM_MAX) {
    Serial.println("[Brushless7] ERROR: STARTRPM out of range!");
    return false;
  }

  // Split into lower 8 bits and upper 4 bits
  uint8_t rpm_low = rpm & 0xFF;         // Bits [7:0]
  uint8_t rpm_high = (rpm >> 8) & 0x0F; // Bits [11:8], masked to 4 bits
  Serial.print("[Brushless7] RPM: "); Serial.print(rpm);
  Serial.print(", Low: 0x"); Serial.print(rpm_low, HEX);
  Serial.print(", High: 0x"); Serial.println(rpm_high, HEX);

  if (!writeRegister(REG_STARTRPM_LOW, rpm_low)) {
    Serial.println("[Brushless7] Failed to write STARTRPM_LOW");
    return false;
  }

 // Write high nibble into bits [7:4] of REG_STARTRPM_HIGH (lower nibble must be 0)
  return writeRegister(REG_STARTRPM_HIGH, rpm_high << 4);
}

// SpeedSlope, maxopen, maxoff
bool Brushless7::setSpeedSlope(uint16_t max_rpm, uint16_t start_rpm, float start_duty, float max_duty) {

  uint16_t STARTDUTY = (uint16_t)(start_duty * START_DUTY_SCALE);

  uint16_t MAXDUTY = (uint16_t)(max_duty * 5.12f - 257.0f + 0.5f);
  //Serial.print((float)( (float)(max_rpm - start_rpm) / (float)(MAXDUTY - STARTDUTY + 257)));
  // TO BE CHANGED: Amplitude for closed loop = 64; open loop = 1024
  uint16_t speedslope = (uint16_t)(( (float)( (float)(max_rpm - start_rpm) / (float)(MAXDUTY - STARTDUTY + 257)) * 64.0f) + 0.5f);

  // Debug: Print computed values
  // Serial.print("[Brushless7] SPEEDSLOPE (14-bit raw): ");
  // Serial.print(speedslope);
  // Serial.print(" (0b");
  // for (int i = 13; i >= 0; i--) Serial.print((speedslope >> i) & 1);
  // Serial.println(")");

  if (speedslope > SPEEDSLOPE_MAX) {
    Serial.println("[Brushless7] ERROR: SPEEDSLOPE out of range!");
    return false;
  }
  // --- Register 0x08 (Low Byte: Bits [7:0]) ---
  uint8_t speed_low = speedslope & 0xFF; // Extract lower 8 bits (0b0101100111 → 0x67)
  Serial.print("[Brushless7] Reg 8 (0x08) = 0b");
  Serial.println(speed_low, BIN); // Expected: 0b01100111
  writeRegister(REG_SPEEDSLOPE_LOW, speed_low);

  // --- Register 0x09 (High Byte: Bits [13:8] → [7:2]) ---
  // 3. Read-modify-write high byte (register 0x09)
  Wire.beginTransmission(_i2c_addr);
  Wire.write(REG_SPEEDSLOPE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)_i2c_addr,(uint8_t) 1);
  uint8_t reg9 = Wire.read();
  Serial.print("[Brushless7] Reg 9 (before update) = 0b");
  Serial.print(reg9, BIN);
  Serial.println(" (0x" + String(reg9, HEX) + ")");
  uint8_t speed_high = (speedslope >> 8) & 0x3F;

  // 2. Merge with MAXOPEN/MAXOFF (reg9[1:0])
  reg9 = (reg9 & 0x03) | (speed_high << 2); // Align to [7:2] (still needs shift for positioning)
  // Debug: Print register 9 binary
  // Serial.print("[Brushless7] Reg 9 (0x09) = 0b");
  // Serial.println(reg9, BIN); 

  writeRegister(REG_SPEEDSLOPE_HIGH, reg9);
  return true;
}

// Reg 9 MAXOPEN  (ENABLE = true or false)
bool Brushless7::setMaxOpen(bool enable) {
  // Read current value of register 9 (0x09)
  Wire.beginTransmission(_i2c_addr);
  Wire.write(REG_SPEEDSLOPE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  uint8_t reg9 = Wire.read();

  // Debug: Print current state of register 9 (binary)
  // Serial.print("[Brushless7] Reg 9 (before MAXOPEN update): 0b");
  // Serial.println(reg9, BIN);

  // Update MAXOPEN bit (bit 1)
  if (enable) reg9 |= BIT_MAXOPEN;
  else        reg9 &= ~BIT_MAXOPEN;

  // Debug: Print new state
  // Serial.print("[Brushless7] Reg 9 (after MAXOPEN update):  0b");
  // Serial.println(reg9, BIN);
  // writeRegister(REG_SPEEDSLOPE_HIGH, reg9); // write back

  Serial.println("[Brushless7] MAXOPEN updated successfully!");
  return true;
}

// Reg 9 MAXOFF  (ENABLE = true or false)
bool Brushless7::setMaxOff(bool enable) {
  // Read current value of register 9 (0x09)
  Wire.beginTransmission(_i2c_addr);
  Wire.write(REG_SPEEDSLOPE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  uint8_t reg9 = Wire.read();

  // Debug: Print current state of register 9 (binary)
  // Serial.print("[Brushless7] Reg 9 (before MAXOFF update):  0b");
  // Serial.println(reg9, BIN);

  // Update MAXOFF bit (bit 0)
  if (enable) reg9 |= BIT_MAXOFF;
  else        reg9 &= ~BIT_MAXOFF;

  // Debug: Print new state
  // Serial.print("[Brushless7] Reg 9 (after MAXOFF update):   0b");
  // Serial.println(reg9, BIN);
  writeRegister(REG_SPEEDSLOPE_HIGH, reg9); // write back

  Serial.println("[Brushless7] MAXOFF updated successfully!");
  return true;
}


// 19[3:7] Brake configuration
bool Brushless7::configureBrake() {
    // 1) read current
    uint8_t v;
    if (! readRegister(REG_BRAKE_CONFIG, v)) return false;
    Serial.print("Brake before: 0b"); Serial.println(v, BIN);

    // 2) clear only those fields, then OR in new values
    v &= ~(BRAKE_WAIT_TIME | BRAKE_WAIT_MODE_HIZ | BRAKE_WAIT_CON_TO_IDLE);

    // 3) OR in your hard-coded config:
    v |= BRAKE_WAIT_TIME;
    v |= BRAKE_WAIT_MODE_HIZ;
    v |= BRAKE_WAIT_CON_TO_IDLE;

    // Serial.print("[Brushless7] Brake after : 0b");
    // Serial.println(v, BIN);
    // Serial.print("Brake after:  0b"); Serial.println(v, BIN);

    return writeRegister(REG_BRAKE_CONFIG, v);
}

bool Brushless7::configureStartup() {
  // 1. DC Excitation Configuration (0x14: REG_TRE_PRETIP_TIP)
  uint8_t dc_val;
  if (!readRegister(REG_TRE_PRETIP_TIP, dc_val)) {
    Serial.println("[Brushless7] ERROR: Failed to read 1st DC excitation reg");
    return false;
  }
  Serial.print("[Brushless7] REG_TRE_PRETIP_TIP before: 0b");
  Serial.println(dc_val, BIN);

  dc_val &= 0b11100000;  // Clear bits 4:0
  dc_val |= (DC_PRETIP | DC_TIP);
  writeRegister(REG_TRE_PRETIP_TIP, dc_val);
  Serial.print("[Brushless7] REG_TRE_PRETIP_TIP after : 0b");
  Serial.println(dc_val, BIN);

  // 2. Forced Commutation Frequency (0x15: REG_LA_FMAX_FST)
  uint8_t fc_val;
  if (!readRegister(REG_LA_FMAX_FST, fc_val)) {
    Serial.println("[Brushless7] ERROR: Failed to read Forced Commutation Freq reg");
    return false;
  }
  Serial.print("[Brushless7] REG_LA_FMAX_FST before: 0b");
  Serial.println(fc_val, BIN);

  fc_val &= 0b11111100;  // Clear bits 1:0
  fc_val |= FST;

  writeRegister(REG_LA_FMAX_FST, fc_val);
  Serial.print("[Brushless7] REG_LA_FMAX_FST after : 0b");
  Serial.println(fc_val, BIN);

  // 3. Comparator Hysteresis (0x18: REG_COMP_HYS)
  uint8_t hys_val;
  if (!readRegister(REG_COMP_HYS, hys_val)) {
    Serial.println("[Brushless7] ERROR: Failed to read Hysterisis reg");
    return false;
  }
  Serial.print("[Brushless7] REG_COMP_HYS before: 0b");
  Serial.println(hys_val, BIN);

  hys_val &= 0b00111111;  // Clear bits 7:6
  hys_val |= COMP_HYS;

  writeRegister(REG_COMP_HYS, hys_val);
  Serial.print("[Brushless7] REG_COMP_HYS after : 0b");
  Serial.println(hys_val, BIN);

  Serial.println("[Brushless7] Startup configuration complete.");
  return true;
}

bool Brushless7::configureOCP() {
  // 1. REG 16 0x10: STARTCURRENT + OCPDIS (REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS)
  uint8_t val_10;
  if (!readRegister(REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS, val_10)) {
    Serial.println("[Brushless7] ERROR: Failed to read OCP reg16");
    return false;
  }
  // Serial.print("[Brushless7] REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS before: 0b");
  // Serial.println(val_10, BIN);

  val_10 &= 0b11110000;  // clear bits 3:1 and bit 0
  val_10 |= (STARTCURRENT | OCPDIS);
  writeRegister(REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS, val_10);
  // Serial.print("[Brushless7] REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS after : 0b");
  // Serial.println(val_10, BIN);

  // 2. REG 15 0x0F: OCPMASK digital filter (REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK)
  uint8_t val_0F;
  if (!readRegister(REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK, val_0F)) {
    Serial.println("[Brushless7] ERROR: Failed to read OCP reg15");
    return false;
  }
  // Serial.print("[Brushless7] REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK before: 0b");
  // Serial.println(val_0F, BIN);

  val_0F &= 0b11111100;  // clear bits 1:0
  val_0F |= OCPMASK;

  writeRegister(REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK, val_0F);
  // Serial.print("[Brushless7] REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK after : 0b");
  // Serial.println(val_0F, BIN);

  // 3. REG 18 0x12: RS_SEL analog filter (REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH)
  uint8_t val_12;
  if (!readRegister(REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH, val_12)) {
    Serial.println("[Brushless7] ERROR: Failed to read OCP reg18");
    return false;
  }  
  // Serial.print("[Brushless7] REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH before: 0b");
  // Serial.println(val_12, BIN);

  val_12 &= 0b11111001;  // clear bits 2:1
  val_12 |= RS_SEL;
  writeRegister(REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH, val_12);
  // Serial.print("[Brushless7] REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH after : 0b");
  // Serial.println(val_12, BIN);

  // 4. REG 23 0x17: OCP_LVL threshold (REG_ISDLVL_OCPLVL_SOURCE_SINK)
  uint8_t val_17;
  if (!readRegister(REG_ISDLVL_OCPLVL_SOURCE_SINK, val_17)) {
    Serial.println("[Brushless7] ERROR: Failed to read OCP reg23");
    return false;
  } 
  // Serial.print("[Brushless7] REG_ISDLVL_OCPLVL_SOURCE_SINK before: 0b");
  // Serial.println(val_17, BIN);

  val_17 &= 0b10111111;  // clear bit 6
  val_17 |= OCP_LVL;

  writeRegister(REG_ISDLVL_OCPLVL_SOURCE_SINK, val_17);
  // Serial.print("[Brushless7] REG_ISDLVL_OCPLVL_SOURCE_SINK after : 0b");
  // Serial.println(val_17, BIN);

  Serial.println("[Brushless7] OCP configuration complete.");
  return true;
}

bool Brushless7::configureSoftStart() {
  uint8_t val;
  if (!readRegister(REG_SSADDSEL_SSUPSEL_SSDUTYCHGLIMIT_DUTYUPTIME, val)) {
    Serial.println("[Brushless7] ERROR: Failed to read SOFTstart reg");
    return false;
  } 
  // Serial.print("[Brushless7] Reg 17 before (soft start): 0b");
  // Serial.println(val, BIN);
  // Clear only bits [7:1] (keep DUTY_UP_TIME [0] untouched)
  val &= 0b00000001;  // preserve bit 0
  val |= (SS_DUTYCHGLIMIT | SS_UP_SEL | SS_ADD_SEL);

  writeRegister(REG_SSADDSEL_SSUPSEL_SSDUTYCHGLIMIT_DUTYUPTIME, val);
  // Serial.print("[Brushless7] Reg 17 after (soft start):  0b");
  // Serial.println(val, BIN);

  return true;
}

bool Brushless7::configureMaxSpeed() {
  uint8_t val;
  if (!readRegister(REG_STBYMODE_DIR_POLEPAIR_MAXSPEED_FGON, val)) {
    Serial.println("[Brushless7] ERROR: Failed to read MAXSPEED reg");
    return false;
  } 
  // Serial.print("[Brushless7] Reg 14 before (MAXSPEED): 0b");
  // Serial.println(val, BIN);

  val &= 0b11111001;  // clear bits [2:1]
  val |= MAXSPEED;    // insert new MAXSPEED bits

  writeRegister(REG_STBYMODE_DIR_POLEPAIR_MAXSPEED_FGON, val);
  // Serial.print("[Brushless7] Reg 14 after (MAXSPEED):  0b");
  // Serial.println(val, BIN);
  return true;
}

bool Brushless7::configureSpeedControl() {
  // 1. Update Reg 17[0]: DUTY_UP_TIME
  uint8_t reg17;
  if (!readRegister(REG_SSADDSEL_SSUPSEL_SSDUTYCHGLIMIT_DUTYUPTIME, reg17)) {
    Serial.println("[Brushless7] ERROR: Failed to read SPEEDCONTROL reg");
    return false;
  } 
  // Serial.print("[Brushless7] Reg 17 before (DUTY_UP_TIME): 0b");
  // Serial.println(reg17, BIN);

  reg17 &= 0b11111110;      // Clear bit 0
  reg17 |= DUTY_UP_TIME;    // Set bit 0 to configured value

  writeRegister(REG_SSADDSEL_SSUPSEL_SSDUTYCHGLIMIT_DUTYUPTIME, reg17);

  // Serial.print("[Brushless7] Reg 17 after (DUTY_UP_TIME):  0b");
  // Serial.println(reg17, BIN);

  // 2. Update Reg 16[6:4]: DUTYCHGLIMIT
  uint8_t reg16;
  if (!readRegister(REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS, reg17)) {
    Serial.println("[Brushless7] ERROR: Failed to read DUTYCHGLIMIT reg");
    return false;
  } 
  // Serial.print("[Brushless7] Reg 16 before (DUTYCHGLIMIT): 0b");
  // Serial.println(reg16, BIN);

  reg16 &= 0b10001111;      // Clear bits 6:4
  reg16 |= DUTYCHGLIMIT;

  writeRegister(REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS, reg16);
  // Serial.print("[Brushless7] Reg 16 after (DUTYCHGLIMIT):  0b");
  // Serial.println(reg16, BIN);

  Serial.println("[Brushless7] Speed control configuration complete.");
  return true;
}

bool Brushless7::configureCommutation() {
  // --- Configure SLOP and LAP in REG 0x16 ---
  uint8_t reg16;
  if (!readRegister(REG_SLOP_LAP_FPWM_DEADTIME, reg16)) {
    Serial.println("[Brushless7] ERROR: Failed to read sLOP & LAP reg");
    return false;
  } 
  // Serial.print("[Brushless7] Reg 16 before (SLOP/LAP): 0b");
  // Serial.println(reg16, BIN);

  reg16 &= 0b00011111;       // Clear bits 7:5 (SLOP and LAP)
  reg16 |= (SLOP | LAP);     // Apply new SLOP and LAP

  writeRegister(REG_SLOP_LAP_FPWM_DEADTIME, reg16);

  // Serial.print("[Brushless7] Reg 16 after (SLOP/LAP):  0b");
  // Serial.println(reg16, BIN);

  // --- Configure LA in REG 0x15 (preserve FST bits 1:0) ---
  uint8_t reg15;
  if (!readRegister(REG_LA_FMAX_FST, reg15)) {
    Serial.println("[Brushless7] ERROR: Failed to read LA reg");
    return false;
  } 
  // Serial.print("[Brushless7] Reg 15 before (LA): 0b");
  // Serial.println(reg15, BIN);

  reg15 &= 0b00001111;       // Clear bits 7:4 (LA)
  reg15 |= LA;               // Apply new lead angle setting

  writeRegister(REG_LA_FMAX_FST, reg15);

  // Serial.print("[Brushless7] Reg 15 after (LA):  0b");
  // Serial.println(reg15, BIN);

  Serial.println(F("[Brushless7] Commutation configuration complete."));
  return true;
}

void Brushless7::setSpeedPWM(uint8_t duty) {
  // assumes begin() did pinMode(_spd_pin, OUTPUT);
  analogWrite(_spd_pin, duty);
}

void Brushless7::setDirection(bool forward) {
  // assumes begin() did pinMode(_dir_pin, OUTPUT);
  digitalWrite(_dir_pin, forward ? HIGH : LOW);
}

bool Brushless7::controlModeSet(uint8_t mode) {
    uint8_t d2, d9, d11;

    // read current values
    if (!readRegister(REG_STOP_DUTY,  d2))  return false;
    if (!readRegister(REG_SPEEDSLOPE_HIGH,  d9))  return false;
    if (!readRegister(REG_SPEEDSLOP2_VCPMASK_OPENLOOP, d11)) return false;

    switch (mode) {
        case CTRL_RPM:
            d2  |= 0x80;       // REG_2 bit7 = 1 
            d9  &= 0xFC;       // REG_9 bits1:0 = 00
            d11 &= 0xFE;       // REG_11 bit0 = 0
            break;

        case CTRL_DUTY:
            d2  |= 0x80;       // REG_2 bit7 = 1
            d9  = (d9 & 0xFC) | 0x01;  // REG_9 bits1:0 = 01
            d11 |= 0x01;       // REG_11 bit0 = 1
            break;

        case CTRL_STOP:
            d2  &= 0x7F;       // REG_2 bit7 = 0
            d9  &= 0xFC;       // REG_9 bits1:0 = 00
            // leave d11 unchanged
            break;

        default:
            return false;
    }

    // write them back
    if (!writeRegister(REG_STOP_DUTY,  d2))  return false;
    if (!writeRegister(REG_SPEEDSLOPE_HIGH,  d9))  return false;
    if (mode != CTRL_STOP) { // REG_11 only changed for RPM/DUTY
        if (!writeRegister(REG_SPEEDSLOP2_VCPMASK_OPENLOOP, d11)) return false;
    }
    return true;
}

bool Brushless7::printRotationSpeedHz() {
    uint8_t hi, lo;
    // Read MSB
    if (! readRegister(REG_HZ_CNT_HIGH, hi)) {
        Serial.println("[Brushless7] ERR reading FG MSB");
        return false;
    }
    // Read LSB
    if (! readRegister(REG_HZ_CNT_LOW, lo)) {
        Serial.println("[Brushless7] ERR reading FG LSB");
        return false;
    }
    Serial.print("hi");Serial.println(hi, BIN);
    Serial.print("low");Serial.println(lo, BIN);
    uint16_t hz_cnt = ((uint16_t)hi << 8) | lo;
    if (hz_cnt == 0xFFFF) {
        Serial.println("[Brushless7] FG count invalid (0xFFFF)");
        return false;
    }
    if (hz_cnt == 0) {
        Serial.println("[Brushless7] Motor stopped or no FG pulses");
        return true;
    }

    // Table 8.33: rotation frequency [Hz] = 250000 / hz_cnt
    float rpm = (250000.0f / (float)hz_cnt) * 60.0f;

    Serial.print("[Brushless7] Rotation speed = ");
    Serial.print(rpm, 2);
    Serial.println(" RPM");

    return true;
}

// REG 0 error print
void Brushless7::printErrors() {
    uint8_t st;
    // register 0 holds all the error flags in bits 0–5:
    // bit0 = ST_FAIL, bit1 = UD_SPD, bit2 = OV_SPD,
    // bit3 = ISD,    bit4 = TSD,    bit5 = CPVSD
    if (! readRegister(0, st)) {
        Serial.println("[Brushless7] ERROR: cannot read status reg");
        return;
    }

    bool any = false;
    if (st & (1<<0)) { Serial.println("⚠️ Startup failure");       any = true; }
    if (st & (1<<1)) { Serial.println("⚠️ Under-speed error");      any = true; }
    if (st & (1<<2)) { Serial.println("⚠️ Over-speed error");       any = true; }
    if (st & (1<<3)) { Serial.println("⚠️ Output over-current");    any = true; }
    if (st & (1<<4)) { Serial.println("⚠️ Thermal shutdown");       any = true; }
    if (st & (1<<5)) { Serial.println("⚠️ Charge-pump low voltage");any = true; }

    if (!any) {
        Serial.println("✅ No errors detected");
    }
}

// UPDATED read register to prevent reading error with "out"
bool Brushless7::readRegister(uint8_t reg_addr, uint8_t &out) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg_addr);
  if (Wire.endTransmission(false) != 0)   // repeated-start
    return false;

  if (Wire.requestFrom(_i2c_addr, (uint8_t)1, (uint8_t)true) < 1)
    return false;

  out = Wire.read();
  return true;
}

bool Brushless7::writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Debug: Prints register value in binary and hex
void Brushless7::printRegisterBinary(uint8_t reg_addr) {
  uint8_t value;
  // 1) Read with error checking
  if (!readRegister(reg_addr, value)) {
    Serial.print("[Brushless7] ERROR: Failed to read reg 0x");
    Serial.println(reg_addr, HEX);
    return;
  }
  Serial.print("Reg 0x");
  Serial.print(reg_addr, HEX);
  Serial.print(": BIN ");
  for (int i = 7; i >= 0; i--) {
    Serial.print((value >> i) & 1);
    if (i == 7) Serial.print("(NOSTOP) "); // Label bit 7
    else if (i == 0) Serial.print("(LSB)");
  }
  Serial.print(", HEX 0x");
  Serial.println(value, HEX);
}


