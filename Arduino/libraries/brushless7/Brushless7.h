#ifndef BRUSHLESS7_H
#define BRUSHLESS7_H

#include <Arduino.h>
#include <Wire.h>

class Brushless7 {
  private:
    uint8_t _sby_pin,  _i2c_addr, _spd_pin, _dir_pin, _cmo_pin, _int_pin;

    // Register addresses
    #define REG_STOP_DUTY  0x02
    #define REG_START_DUTY  0x03
    #define REG_CHANGE_DUTY  0x04
    #define REG_MAX_DUTY  0x05
    #define REG_STARTRPM_LOW  0x06  // Lower 8 bits of RPM
    #define REG_STARTRPM_HIGH  0x07 // Upper 4 bits of RPM (bits [7:4])
    #define REG_SPEEDSLOPE_LOW  0x08 // Lower 8 bits of speedslope1  
    #define REG_SPEEDSLOPE_HIGH  0x09 // speedslope1 [2:7], maxopen [1], maxoff [0]
    #define REG_SPEEDSLOP2  0x0A // Reg 10 ([7:0])
    #define REG_SPEEDSLOP2_VCPMASK_OPENLOOP  0x0B // Reg 11 ([7:2], 1, 0)
    #define REG_KIX_KI  0x0C // Reg 12 (7, [6:0])
    #define REG_KPX_KP  0x0D // Reg 13 (7, [6:0])
    #define REG_STBYMODE_DIR_POLEPAIR_MAXSPEED_FGON  0x0E // Reg 14 (7, 6, [5:3], [2:1], 0)
    #define REG_FGSEL_TSPSEL_SPDINV_LATCH_OCPMASK  0x0F // Register 15 ([7:5], 4, 3, 2, [1:0])
    #define REG_LOCKDIS_DUTYCHG_STARTCURR_OCPDIS  0X10 // Register 16 (7, [6:4], [3:1], 0)
    #define REG_SSADDSEL_SSUPSEL_SSDUTYCHGLIMIT_DUTYUPTIME  0x11 // Reg 17 ([7:6], [5:4], [3:1], 0)
    #define REG_RPMLIMIT_BRKINV_ISDMASK_RSSEL_ANTITHROUGH  0x12 // Register 18 ([7:5], 4, 3, [2:1], 0)
    #define REG_BRAKE_CONFIG  0x13  //Register 19 ([7:3]brake, [2] LOCK_BRK, [1]alertinv, [0] tsd_mask)
    #define REG_TRE_PRETIP_TIP 0x14 // Register 20 ([2:0] DC 2nd, [4:3]DC 1st, [7:5] TRE)
    #define REG_LA_FMAX_FST  0x15 // Register 21 ([7:4], [3:2], [1:0] forced commu freq)
    #define REG_SLOP_LAP_FPWM_DEADTIME  0x16 // Register 22 (7, [6:5], [4:2], [1:0])
    #define REG_ISDLVL_OCPLVL_SOURCE_SINK  0x17 //Register 23 (7, 6, [5:3], [2:0])
    #define REG_COMP_HYS  0x18 // Register 24 ([7:6] hysterisis, [5:0] -)
    #define REG_SLAVE_ADRS  0x19 // Reg 25 ([7:1])
    #define REG_SPD_LOW  0x1B // Reg 27 ([7:0])
    #define REG_SPD_HIGH  0x1C // Reg 28 ([7:6])
    #define REG_HZ_CNT_LOW  0x1D // Reg 29 ([7:0])
    #define REG_HZ_CNT_HIGH  0x1E // Reg 30 ([7:0])


    //START DUTY static constants
    #define START_DUTY_MIN  0.0f
    #define START_DUTY_MAX  49.8f
    #define START_DUTY_SCALE  5.12f  // 512/100
    //STOP DUTY static constants
    #define STOP_DUTY_MIN  0.0f
    #define STOP_DUTY_MAX  49.6f
    #define STOP_DUTY_SCALE  2.56f  // 256/100
    //CHANGE DUTY static constants
    #define CHANGE_DUTY_MIN  0.4f
    #define CHANGE_DUTY_MAX  99.6f
    #define CHANGE_DUTY_SCALE  2.56f
    // MAX DUTY static constants
    #define MAX_DUTY_MIN  50.2f
    #define MAX_DUTY_MAX  100.0f
    //START RPM static constants 
    #define STARTRPM_MIN  0
    #define STARTRPM_MAX  4096
    //SPEEDSLOPE, MAXOPEN, MAXOFF
    #define BIT_MAXOPEN  0x02  // Bit 1
    #define BIT_MAXOFF   0x01  // Bit 0 
    #define SPEEDSLOPE_MAX   16383 

    // Brake configuration bits 19[3:7] Section 8.1.2
    #define BRAKE_WAIT_TIME  0b00000000
    #define BRAKE_WAIT_MODE_HIZ  0b00000000  // WAIT_MODE = 0
    #define BRAKE_WAIT_MODE_SHORT  0b00010000  // WAIT_MODE = 1
    #define BRAKE_WAIT_CON_TO_IDLE  0b00000000  // WAIT_CON = 0
    #define BRAKE_WAIT_CON_DIRECT_START  0b00001000  // WAIT_CON = 1

    // DC excitation bits (Startup, Section 8.1.4)
    #define DC_PRETIP    0b00011000  // 20[4:3]
    #define DC_TIP     0b00000111  // 20[2:0]

    // Forced commutation (Startup, Section 8.1.4)
    #define FST  0b00000000 // 21[1:0]  Go higher if want faster

    // Comparator hysteresis (Startup, Section 8.1.4)
    #define COMP_HYS  0b10000000  // 24[7:6]

    // Output current protection setup (Section 8.1.5)
    #define STARTCURRENT  0b00001110  // 16[3:1] = 111
    #define OCPDIS  0b00000000  // 16[0] = 0
    #define OCPMASK  0b00000001  // 15[1:0] = 01
    #define RS_SEL  0b00000000  // 18[2:1] = 00
    #define OCP_LVL  0b01000000  // 23[6] = 1

    // Soft start (Section 8.1.6)
    #define SS_DUTYCHGLIMIT 0b00000010  // 17[3:1] = 001
    #define SS_UP_SEL       0b00000000  // 17[5:4]
    #define SS_ADD_SEL      0b11000000  // 17[7:6] Startup I limit + 50% current
    #define MAXSPEED        0b00000010  // 14[2:1]

    // Speed control (Section 8.1.7)
    #define DUTY_UP_TIME     0b00000000  // Reg 17[0]
    #define DUTYCHGLIMIT     0b00010000  // Reg 16[6:4] = 001

    // Current monitor output (Section 8.1.8) set by default config 23[6] = 0

    // Commutation method & Lead Angle Control (Section 8.1.9)  ; FST was set in 8.1.4
    #define SLOP 0b10000000          // 22[7] = 1 (hard switching)
    #define LAP  0b00000000          // 22[6:5] = 00 (120deg commutation)
    #define LA   0b01010000          // 21[7:4] = 0000 (0deg)

    // Rotation direction (Section 8.1.10) set by default config 14[6] = 0 
    // Brake function (Section 8.1.11) set by default config 18[4] = 0
    // PWM frequency (Section 8.1.12) set by default config 22[4:2] = 000
    // External FET Gate Drive Output (Section 8.1.13) set by default config 23[5:0] = 111111

    // Control‐mode constants (matching BRUSHLESS7_CTRL_TYPE_x from the Click API)
  #define CTRL_RPM   0  // I²C RPM control
  #define CTRL_DUTY  1  // PWM duty control
  #define CTRL_STOP  2  // STOP mode (FETs off)

  
  public:
    Brushless7(uint8_t sby_pin, uint8_t i2c_addr, uint8_t spd_pin, uint8_t dir_pin, uint8_t cmo_pin, uint8_t int_pin);
    void begin(); // Initialize the pins
    bool configureDefault(); // Set up default configurations
    bool configureBrake(); // Brake config 19[3:7]
    bool configureStartup(); // Section 8.1.4 Startup sequence
    bool configureOCP(); // Section 8.1.5 Configures output current protection (OCP)
    bool configureSoftStart(); // Section 8.1.6 soft start
    bool configureMaxSpeed(); // Section 8.1.6 soft start
    bool configureSpeedControl(); // Section 8.1.7 Speed Control
    bool configureCommutation(); // Section 8.1.9 Commutation method & Lead Angle Control

    // Closed loop control (Section 8.2.1, Table 8.39)
    bool setStartDuty(float duty_percent);
    bool setStopDuty(float duty_percent);
    bool setChangeDuty(float duty_percent);
    bool setMaxDuty(float duty_percent);
    bool setStartRPM(uint16_t rpm);
    bool setSpeedSlope(uint16_t max_rpm, uint16_t start_rpm, float start_duty, float max_duty);
    bool setMaxOpen(bool enable); // Check Table 8.42
    bool setMaxOff(bool enable); // Check Table 8.42

    void setSpeedPWM(uint8_t duty); // Set duty cycle (SPD pin) to control the RPM 
    void setDirection(bool forward); // Set the direction of rotation

    bool controlModeSet(uint8_t mode); // Set up control mode: duty/rpm/stop
    bool printRotationSpeedHz(); // Print out the RPM, but not accurate (hardware issue)

    void printErrors(); // Print error detection
    bool readRegister(uint8_t reg_addr, uint8_t &out); // Read register value
    bool writeRegister(uint8_t reg, uint8_t val); // Write register value
    void printRegisterBinary(uint8_t reg_addr); // Debug: prints register in binary/hex
};

#endif
