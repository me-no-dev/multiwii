/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
April  2011     V1.7
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

/*******************************/
/****CONFIGURABLE PARAMETERS****/
/*******************************/

/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1190

/* The type of multicopter */
//#define GIMBAL
//#define BI
#define TRI
//#define QUADP
//#define QUADX
//#define Y4
//#define Y6
//#define HEX6
//#define HEX6X
//#define FLYING_WING //experimental

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

#define PROMINI  //Arduino type
//#define MEGA

//enable internal I2C pull ups
#define INTERNAL_I2C_PULLUPS

//****** advanced users settings   *************

/* Failsave settings - added by MIS
   Failsafe check pulse on THROTTLE channel. If the pulse is OFF (on only THROTTLE or on all channels) the failsafe procedure is initiated.
   After FAILSAVE_DELAY time of pulse absence, the level mode is on (if ACC or nunchuk is avaliable), PITCH, ROLL and YAW is centered
   and THROTTLE is set to FAILSAVE_THR0TTLE value. You must set this value to descending about 1m/s or so for best results. 
   This value is depended from your configuration, AUW and some other params. 
   Next, afrer FAILSAVE_OFF_DELAY the copter is disarmed, and motors is stopped.
   If RC pulse coming back before reached FAILSAVE_OFF_DELAY time, after the small quard time the RC control is returned to normal.
   If you use serial sum PPM, the sum converter must completly turn off the PPM SUM pusles for this FailSafe functionality.*/
#define FAILSAFE                                  // Alex: comment this line if you want to deactivate the failsafe function
#define FAILSAVE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAVE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAVE_THR0TTLE  (MINTHROTTLE + 200)    // Throttle level used for landing - may be relative to MINTHROTTLE - as in this case


/* The following lines apply only for a pitch/roll tilt stabilization system
   It is not compatible with Y6 or HEX6 or HEX6X
   Uncomment the first line to activate it */
//#define SERVO_TILT
#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10

/* I2C gyroscope */
//#define ITG3200
//#define L3G4200D

/* I2C accelerometer */
//#define ADXL345
//#define BMA020
//#define BMA180
//#define NUNCHACK  // if you want to use the nunckuk as a standalone I2C ACC without WMP

/* I2C barometer */
//#define BMP085

/* I2C magnetometer */
//#define HMC5843
//#define HMC5883

/* ADC accelerometer */ // for 5DOF from sparkfun, uses analog PIN A1/A2/A3
//#define ADCACC

/* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
   IF YOUR RECEIVER IS NOT CONCERNED, DON'T UNCOMMENT ANYTHING. Note this is mandatory for a Y6 setup
   Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL //For Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL //For some Hitec/Sanwa/Others

/* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
   if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
   it is relevent only for a conf with NK */
#define INTERLEAVING_DELAY 3000

/* for V BAT monitoring
   after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
   with R1=33k and R2=51k
   vbat = [0;1023]*16/VBATSCALE */
#define VBAT              // comment this line to suppress the vbat code
#define VBATSCALE     131 // change this value if readed Battery voltage is different than real voltage
#define VBATLEVEL1_3S 107 // 10,7V
#define VBATLEVEL2_3S 103 // 10,3V
#define VBATLEVEL3_3S 99  // 9.9V

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
   it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000

/* this is the value for the ESCs when thay are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000

/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 1850

/* This is the speed of the serial interface. 115200 kbit/s is the best option for a USB connection.*/
#define SERIAL_COM_SPEED 115200

/* In order to save space, it's possibile to desactivate the LCD configuration functions
   comment this line only if you don't plan to used a LCD */
#define LCD_CONF

/* to use Cat's whisker TEXTSTAR LCD, uncomment following line.
   Pleae note this display needs a full 4 wire connection to (+5V, Gnd, RXD, TXD )
   Configure display as follows: 115K baud, and TTL levels for RXD and TXD, terminal mode
   NO rx / tx line reconfiguration, use natural pins */
//#define LCD_TEXTSTAR

/* motors will not spin when the throttle command is in low position
   this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/* experimental
   camera trigger function : activated via AUX1 UP, servo output=A2 */
//#define CAMTRIG
#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500

/* to sum the power consumption from battery it is possible to sum up the signals sent to each ESC separately. */
/* Under the asumption that a) the function from ESC signal to consumed power  quite follows a universal characteristic function */
/* and b) the time base i.e. main loop time is quite stable and known */
/* it will be possible to derive a value that corresponds to total consumed power */
//#define POWERMETER
// the sum of all powermeters ranges from [0:60000 e4] theoretically.
// the alarm level from eeprom is out of [0:255], so we multipy alarm level with PLEVELSCALE and with 1e4 before comparing
// the default value should be fine; for a 1000mAh battery on my TRI it gives an step size of ~10mAh to define alarm level
// how to find your plevelscale for your particular copter and particular battery range in 3 easy steps
// only neccesssary if you get overrun or feel adventurous:
  // 1 fly copter, at end of flight use LCD and read the sum value (example 8040 for 1000mAh battery)
  // 2 optional: charge battery and memorize mAh (example: 930mAh)
  // 3 divide sum value by 255, add 30% margin (example: 8040/255 = 31; add 30% margin => 40)
// defaults should be fine - only touch if you know what you are doing
#define PLEVELSCALE 100 
#define PLEVELDIV 10000

/* to monitor system values (battery level, loop time etc. with LCD enable this */
/* note: for now you must send single characters 'A', 'B', 'C', 'D' to request 4 different screens */
/* New: the info screen on the LCD does get updated automatically - for stop press same button again */
/* easy to use with Textstar LCD - the 4 buttons are preconfigured to send 'A', 'B', 'C', 'D' */
//#define LCD_TELEMETRY
/* on page B it gives a bar graph which shows how much voltage battery has left. Range from 0 to 12 Volt is not very informative */
/* so we try do define a meaningful part. For a 3S battery we define full=12,6V and calculate how much it is above first warning level */
/* Example: 12.6V - VBATLEVEL1_3S  (for me = 126 - 102 = 24) */
#define VBATREF 24 

/* to log values like max loop time and others to come, we need extra variables */
/* if you do not want the additional computing time or are short on memory, then comment the following */
#define LOG_VALUES

//****** end of advanced users settings *************

/**************************************/
/****END OF CONFIGURABLE PARAMETERS****/
/**************************************/

#include <EEPROM.h>

#define LEDPIN 13     // will be changed for MEGA in a future version
#define POWERPIN 12   // will be changed for MEGA in a future version
#define V_BATPIN 3    // Analog PIN 3
#define STABLEPIN     // will be defined for MEGA in a future version

#include "/Volumes/Vault/Users/js/Documents/RC-Heli/TriWiiCopter/meineCombi-7-v1-7/TRI60.h"

#if defined(PROMINI)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_SWITCH              PINB |= 1<<5;     //switch LEDPIN state (digital PIN 13)
  #define LEDPIN_OFF                 PORTB &= ~(1<<5);
  #define LEDPIN_ON                  PORTB |= (1<<5);
  #define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
  #define BUZZERPIN_ON               PORTB |= 1;
  #define BUZZERPIN_OFF              PORTB &= ~1;
  #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
  #define POWERPIN_ON                PORTB |= 1<<4;
  #define POWERPIN_OFF               PORTB &= ~(1<<4); //switch OFF WMP, digital PIN 12
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTD &= ~1;
  #define LCDPIN_ON                  PORTD |= 1;
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT); //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
  #define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(A0,OUTPUT);
  #define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<0;
  #define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<0);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(A1,OUTPUT);
  #define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<1;
  #define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<1);
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(11,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTB |= 1<<3;
  #define DIGITAL_BI_LEFT_LOW        PORTB &= ~(1<<3);
  #define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING); //PIN 0
  #define MOTOR_ORDER                9,10,11,3,6,5  //for a quad+: rear,right,left,front
  #define DIGITAL_CAM_PINMODE        pinMode(A2,OUTPUT);
  #define DIGITAL_CAM_HIGH           PORTC |= 1<<2;
  #define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);
  //RX PIN assignment inside the port //for PORTD
  #define THROTTLEPIN                2
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    7   //unused just for compatibility with MEGA
  #define CAM1PIN                    7   //unused just for compatibility with MEGA
  #define CAM2PIN                    7   //unused just for compatibility with MEGA
  #define ISR_UART                   ISR(USART_UDRE_vect)
#endif
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_SWITCH              PINB |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7);
  #define BUZZERPIN_PINMODE          pinMode (31, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<6;
  #define BUZZERPIN_OFF              PORTC &= ~1<<6;
  #define POWERPIN_PINMODE           pinMode (12, OUTPUT);
  #define POWERPIN_ON                PORTB |= 1<<6;
  #define POWERPIN_OFF               PORTB &= ~(1<<6);
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1;      //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;       //switch OFF digital PIN 0
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT); //PIN 2 //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
  #define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(A0,OUTPUT); // not the final choice
  #define DIGITAL_TILT_PITCH_HIGH    PORTF |= 1<<0;
  #define DIGITAL_TILT_PITCH_LOW     PORTF &= ~(1<<0);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(A1,OUTPUT); // not the final choice
  #define DIGITAL_TILT_ROLL_HIGH     PORTF |= 1<<1;
  #define DIGITAL_TILT_ROLL_LOW      PORTF &= ~(1<<1);
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(6,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTE |= 1<<6;
  #define DIGITAL_BI_LEFT_LOW        PORTE &= ~(1<<6);
  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19
  #define MOTOR_ORDER                3,5,6,2,7,8   //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
  #define DIGITAL_CAM_PINMODE        pinMode(A2,OUTPUT); // not the final choice
  #define DIGITAL_CAM_HIGH           PORTF |= 1<<2;
  #define DIGITAL_CAM_LOW            PORTF &= ~(1<<2);
  //RX PIN assignment inside the port //for PORTK
  #define THROTTLEPIN                0  //PIN 62 =  PIN A8
  #define ROLLPIN                    1  //PIN 63 =  PIN A9
  #define PITCHPIN                   2  //PIN 64 =  PIN A10
  #define YAWPIN                     3  //PIN 65 =  PIN A11
  #define AUX1PIN                    4  //PIN 66 =  PIN A12
  #define AUX2PIN                    5  //PIN 67 =  PIN A13
  #define CAM1PIN                    6  //PIN 68 =  PIN A14
  #define CAM2PIN                    7  //PIN 69 =  PIN A15
  #define ISR_UART                   ISR(USART0_UDRE_vect)
#endif

#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
  #define SERVO
#endif

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK)
  #define I2C_ACC
#endif

#if defined(GIMBAL) || defined(FLYING_WING)
  #define NUMBER_MOTOR 0
#elif defined(BI)
  #define NUMBER_MOTOR 2
#elif defined(TRI)
  #define NUMBER_MOTOR 3
#elif defined(QUADP) || defined(QUADX) || defined(Y4)
  #define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
  #define NUMBER_MOTOR 6
#endif

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define CAMPITCH   6
#define CAMROLL    7

static uint32_t previousTime;
static uint32_t neutralizeTime;
static uint32_t rcTime;
static uint32_t currentTime;
static uint16_t cycleTime;          // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
#ifdef LOG_VALUES
static uint16_t cycleTimeMax = 0;          // highest ever cycle timen 
#endif
static uint16_t meanTime = 2000;    // this is the average time of the loop: around 2ms for a WMP config and 6ms for a NK+WMP config
static uint16_t calibratingA;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint8_t armed = 0;
static int16_t acc_1G = 200;       //this is the 1G measured acceleration (nunchuk)
static int16_t acc_25deg = 85;     //this is the the ACC value measured on x or y axis for a 25deg inclination (nunchuk) = acc_1G * sin(25)
static uint8_t nunchukPresent = 0;
static uint8_t accPresent = 0;     //I2C or ADC acc present
static uint8_t gyroPresent = 0;    //I2C or ADC gyro present (other than WMP)
static uint8_t magPresent = 0;     //I2C or ADC compass present
static uint8_t baroPresent = 0;    //I2C or ADC baro present
static uint8_t accMode = 0;        //if level mode is a activated
static uint8_t magMode = 0;        //if compass heading hold is a activated
static uint8_t baroMode = 0;       //if altitude hold is activated
static int16_t accADC[3];
static int16_t gyroADC[3];
static int16_t magADC[3];
static int16_t heading,magHold;
static int16_t altitudeSmooth = 0;
static uint8_t calibratedACC = 0;
static uint8_t vbat;               //battery voltage in 0.1V steps
#if defined(POWERMETER)
#ifndef VBAT
	#error "to use powermeter, you must also define and configure VBAT"
#endif
static uint32_t pMeter[7]; //we use [0:5] for six motors,[6] for sum
static uint8_t pMeterV; // dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm; // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
#define PARAMMAX 25
#define PARAMMOTORSTART  16
#define PARAMMOTOREND    24
#define PARAMMOTOROFFSET 17
#else
#define PARAMMAX 16
#endif
// *********************
// I2C general functions
// *********************

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REAPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

void i2c_write(uint8_t data ) {	
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}

void waitTransmissionI2C() {
  uint8_t count = 255;
  while (count-->0 && !(TWCR & (1<<TWINT)) );
  if (count<2) { //we are in a blocking state => we don't insist
    TWCR = 0;  //and we force a reset on TWINT register
    neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay after the hard reset
  }
}

void checkStatusI2C() {
  if ( TW_STATUS  == 0xF8) { //TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
    // WMP does not respond anymore => we do a hard reset. I did not find another way to solve it. It takes only 13ms to reset and init to WMP or WMP+NK
    TWCR = 0;
    POWERPIN_OFF //switch OFF WMP
    delay(1);  
    POWERPIN_ON  //switch ON WMP
    delay(10);
    #if defined(ITG3200) || defined(L3G4200D)
    #else
      i2c_WMP_init(0);
    #endif
    neutralizeTime = micros(); //we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay after the hard reset
  }
}


// **************************
// I2C Barometer BOSCH BMP085
// **************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)

// sensor registers from the BOSCH BMP085 datasheet
#if defined(BMP085)
static int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;
static uint16_t  ac4,ac5,ac6;

static uint16_t ut; //uncompensated T
static uint32_t up; //uncompensated P
int32_t temperature = 0;
int32_t pressure = 0;
int16_t altitude = 0;
static int16_t altitudeZero;
static int16_t altitudeHold;

void  i2c_Baro_init() {
  delay(10);
  ac1 = i2c_BMP085_readIntRegister(0xAA);
  ac2 = i2c_BMP085_readIntRegister(0xAC);
  ac3 = i2c_BMP085_readIntRegister(0xAE);
  ac4 = i2c_BMP085_readIntRegister(0xB0);
  ac5 = i2c_BMP085_readIntRegister(0xB2);
  ac6 = i2c_BMP085_readIntRegister(0xB4);
  b1  = i2c_BMP085_readIntRegister(0xB6);
  b2  = i2c_BMP085_readIntRegister(0xB8);
  mb  = i2c_BMP085_readIntRegister(0xBA);
  mc  = i2c_BMP085_readIntRegister(0xBC);
  md  = i2c_BMP085_readIntRegister(0xBE);
  
  baroPresent = 1;
  i2c_BMP085_calibrate();
}

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(unsigned char r) {
  uint8_t msb, lsb;
  
  i2c_rep_start(0xEE + 0);
  i2c_write(r);
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((int16_t)msb<<8) | ((int16_t)lsb));
}

// read uncompensated temperature value: send command first
void i2c_BMP085_readUT_Command() {
  i2c_rep_start(0xEE + 0);
  i2c_write(0xf4);
  i2c_write(0x2e);
  i2c_rep_start(0xEE + 0);
  i2c_write(0xF6);
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
uint16_t i2c_BMP085_readUT_Result() {
  uint8_t msb, lsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((uint16_t)msb<<8) | ((uint16_t)lsb));
}

// read uncompensated pressure value: send command first
void i2c_BMP085_readUP_Command () {
  i2c_rep_start(0xEE + 0);
  i2c_write(0xf4);
  i2c_write(0xf4); //control register value for oversampling setting 3
  i2c_rep_start(0xEE + 0); //I2C write direction => 0
  i2c_write(0xf6);
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
uint32_t i2c_BMP085_readUP_Result () {
  uint8_t msb, lsb, xlsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb = i2c_readAck();
  lsb = i2c_readAck();
  xlsb = i2c_readNak();
  return (((uint32_t)msb<<16) | ((uint32_t)lsb<<8) | ((uint32_t)xlsb)) >>5;
}

// deduction of true temperature and pressure from sensor, code is described in the BMP085 specs
void i2c_BMP085_CompensatedSensor() {
  int32_t x1, x2, x3, b3, b5, b6, p;
  uint32_t b4, b7;
  uint16_t a,b;

  //calculate true temperature
  x1 = (int32_t)(ut - ac6) * ac5 >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  temperature = (b5 + 8) >> 4;
  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = ( ( ((int32_t) ac1 * 4 + x3)<<3 ) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = (up - b3) * (50000 >> 3);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

//in a whole cycle: we read temperature one time and pressure 5 times
void i2c_Baro_update() {
  static uint32_t t;
  static uint8_t state1 =0,state2 = 0;

  if ( (micros()-t )  < 30000 ) return; //each read is spaced by 30ms
  t = micros();
  
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (state1 == 0) { 
    if (state2 == 0) {
      i2c_BMP085_readUT_Command();
      state2=1;
    } else {
      ut = i2c_BMP085_readUT_Result();
      state2=0;
      state1=5;
    }
  } else {
    if (state2 == 0) {
      i2c_BMP085_readUP_Command();
      state2=1;
    } else {
      up = i2c_BMP085_readUP_Result();
      state2=0;
      state1--;
      i2c_BMP085_CompensatedSensor();
      altitude = (1.0 - pow(float(pressure)/101325.0, 0.190295)) * 443300 - altitudeZero; // altitude in decimeter from starting point
      if ( abs(altitude-altitudeSmooth) < 100 ) //avoid altitude spike
        altitudeSmooth = (altitudeSmooth*7+altitude+4)/8;
    }
  }
}

void i2c_BMP085_calibrate() {
  altitudeZero = 0;
  for (uint8_t i=0;i<7;i++) {
    i2c_Baro_update();
    delay(35);
  }
  altitudeZero = altitude;
}
#endif


// **************************
// I2C Accelerometer ADXL345 
// **************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
#if defined(ADXL345)
static uint8_t rawADC_ADXL345[6];

void i2c_ACC_init () {
  delay(10);
  i2c_rep_start(0x3A+0);      // I2C write direction
  i2c_write(0x2D);            // register 2D Power CTRL
  i2c_write(1<<3);            // Set measure bit 3 on
  i2c_rep_start(0x3A+0);      // I2C write direction 
  i2c_write(0x31);            // DATA_FORMAT register
  i2c_write(0x0B);            // Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_rep_start(0x3A+0);      // I2C write direction 
  i2c_write(0x2C);            // BW_RATE
  i2c_write(8+2+1);           // 200Hz sampling (see table 5 of the spec)

  acc_1G = 250;
  acc_25deg = 106; // = acc_1G * sin(25 deg)
  accPresent = 1;
}

void i2c_ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  i2c_rep_start(0x3A);     // I2C write direction
  i2c_write(0x32);         // Start multiple read at reg 0x32 ADX
  i2c_rep_start(0x3A +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_ADXL345[i]=i2c_readAck();}
  rawADC_ADXL345[5]= i2c_readNak();

  accADC[ROLL]  = - ((rawADC_ADXL345[3]<<8) | rawADC_ADXL345[2]);
  accADC[PITCH] =   ((rawADC_ADXL345[1]<<8) | rawADC_ADXL345[0]);
  accADC[YAW]   = - ((rawADC_ADXL345[5]<<8) | rawADC_ADXL345[4]);
}
#endif


// **************************
// contribution from opie11 (rc-grooups)
// I2C Accelerometer BMA180
// **************************
// I2C adress: 0x80 (8bit)    0x40 (7bit)
#if defined(BMA180)
static uint8_t rawADC_BMA180[6];

void i2c_ACC_init () {
  delay(10);
  i2c_rep_start(0x80+0);      // I2C write direction 
  i2c_write(0x0D);            // ctrl_reg0
  i2c_write(1<<4);            // Set bit 4 to 1 to enable writing
  i2c_rep_start(0x80+0);       
  i2c_write(0x35);            // 
  i2c_write(3<<1);            // range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
  i2c_rep_start(0x80+0);
  i2c_write(0x20);            // bw_tcs reg: bits 4-7 to set bw
  i2c_write(0<<4);            // bw to 10Hz (low pass filter)

  acc_1G = 273;
  acc_25deg = 113; // = acc_1G * sin(25 deg)
  accPresent = 1;
}

void i2c_ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_rep_start(0x80);     // I2C write direction
  i2c_write(0x02);         // Start multiple read at reg 0x02 acc_x_lsb
  i2c_rep_start(0x80 +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_BMA180[i]=i2c_readAck();}
  rawADC_BMA180[5]= i2c_readNak();

  accADC[ROLL]  = - (((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0]))>>2)/10; // opie settings: + ; FFIMU: -
  accADC[PITCH] = - (((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2]))>>2)/10;
  accADC[YAW]   = - (((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4]))>>2)/10;
}
#endif

// **************
// contribution from Point65 and mgros (rc-grooups)
// BMA020 I2C
// **************
// I2C adress: 0x70 (8bit)
#if defined(BMA020)
static uint8_t rawADC_BMA020[6];

void i2c_ACC_init(){
  byte control;
  
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x15);         // 
  i2c_write(0x80);         // Write B10000000 at 0x15 init BMA020

  i2c_rep_start(0x70);     // 
  i2c_write(0x14);         //  
  i2c_write(0x71);         // 
  i2c_rep_start(0x71);     //
  control = i2c_readNak();
 
  control = control >> 5;  //ensure the value of three fist bits of reg 0x14 see BMA020 documentation page 9
  control = control << 2;
  control = control | 0x00; //Range 2G 00
  control = control << 3;
  control = control | 0x00; //Bandwidth 25 Hz 000
 
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x14);         // Start multiple read at reg 0x32 ADX
  i2c_write(control);

  acc_1G = 240;
  acc_25deg = 101; // = acc_1G * sin(25 deg)
  accPresent = 1;
}

void i2c_ACC_getADC(){
  TWBR = ((16000000L / 400000L) - 16) / 2;
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x02);         // Start multiple read at reg 0x32 ADX
  i2c_write(0x71);  
  i2c_rep_start(0x71);  //I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_BMA020[i]=i2c_readAck();}
  rawADC_BMA020[5]= i2c_readNak();

  accADC[ROLL]  =  (((rawADC_BMA020[1])<<8) | ((rawADC_BMA020[0]>>1)<<1))/64;
  accADC[PITCH] =  (((rawADC_BMA020[3])<<8) | ((rawADC_BMA020[2]>>1)<<1))/64;
  accADC[YAW]   = -(((rawADC_BMA020[5])<<8) | ((rawADC_BMA020[4]>>1)<<1))/64;
}
#endif


// **************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// **************************
#if defined(L3G4200D)
static uint8_t rawADC_L3G4200D[6];

void i2c_Gyro_init() {
  delay(100);
  i2c_rep_start(0XD2+0);      // CTRL_REG1
  i2c_write(0x20);            // 400Hz ODR, 20hz filter, run!
  i2c_write(0x8F); 
  i2c_rep_start(0XD2+0);      // CTRL_REG5
  i2c_write(0x24);            // low pass filter enable
  i2c_write(0x02);
  
  gyroPresent = 1;  
}

void i2c_Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0XD2);     // I2C write direction
  i2c_write(0x80 | 0x28);  // Start multiple read
  i2c_rep_start(0XD2 +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_L3G4200D[i]=i2c_readAck();}
  rawADC_L3G4200D[5]= i2c_readNak();

  gyroADC[ROLL]  =  ((rawADC_L3G4200D[1]<<8) | rawADC_L3G4200D[0])/20 ;
  gyroADC[PITCH] =  ((rawADC_L3G4200D[3]<<8) | rawADC_L3G4200D[2])/20 ;
  gyroADC[YAW]   =  -((rawADC_L3G4200D[5]<<8) | rawADC_L3G4200D[4])/20 ;
}
#endif



// **************************
// I2C Gyroscope ITG3200 
// **************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)  // for sparkfun breakout board default jumper
// I2C adress: 0xD0 (8bit)   0x68 (7bit)  // for FreeFlight IMU board default jumper
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND) <- this is the case for the code here
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
#if defined(ITG3200)
static uint8_t rawADC_ITG3200[6];

void i2c_Gyro_init() {
  delay(100);
  i2c_rep_start(0XD0+0);      // I2C write direction 
  i2c_write(0x3E);            // Power Management register
  i2c_write(0x80);            //   reset device
  i2c_write(0x16);            // register DLPF_CFG - low pass filter configuration & sample rate
  i2c_write(0x1D);            //   10Hz Low Pass Filter Bandwidth - Internal Sample Rate 1kHz
  i2c_write(0x3E);            // Power Management register
  i2c_write(0x01);            //   PLL with X Gyro reference
  delay(100);
  gyroPresent = 1;  
}

void i2c_Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0XD0);     // I2C write direction
  i2c_write(0X1D);         // Start multiple read
  i2c_rep_start(0XD0 +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_ITG3200[i]=i2c_readAck();}
  rawADC_ITG3200[5]= i2c_readNak();

  gyroADC[ROLL]  = + ( ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3])/5/4 );
  gyroADC[PITCH] = - ( ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1])/5/4 );
  gyroADC[YAW]   = - ( ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5])/5/4 );
}
#endif

// **************************
// I2C Compass HMC5843 
// **************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)

#if defined(HMC5843)
static uint8_t rawADC_HMC5843[6];

void i2c_Mag_init() { 
  delay(100);
  i2c_rep_start(0X3C+0);      // I2C write direction 
  i2c_write(0x02);            // Write to Mode register
  i2c_write(0x00);            //   Continuous-Conversion Mode
  magPresent = 1;  
}

void i2c_Mag_getADC() {
  static uint32_t t;
  if ( (micros()-t )  < 100000 ) return; //each read is spaced by 100ms
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0X3C);     // I2C write direction
  i2c_write(0X03);         // Start multiple read
  i2c_rep_start(0X3C +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_HMC5843[i]=i2c_readAck();}
  rawADC_HMC5843[5]= i2c_readNak();

  magADC[ROLL]  =   ((rawADC_HMC5843[0]<<8) | rawADC_HMC5843[1]);
  magADC[PITCH] =   ((rawADC_HMC5843[2]<<8) | rawADC_HMC5843[3]);
  magADC[YAW]   = - ((rawADC_HMC5843[4]<<8) | rawADC_HMC5843[5]);
}
#endif

// **************************
// I2C Compass HMC5883 
// **************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)

#if defined(HMC5883)
static uint8_t rawADC_HMC5883[6];

void i2c_Mag_init() { 
  delay(100);
  i2c_rep_start(0X3C+0);      // I2C write direction 
  i2c_write(0x02);            // Write to Mode register
  i2c_write(0x00);            //   Continuous-Conversion Mode
  magPresent = 1;  
}

void i2c_Mag_getADC () {
  static uint32_t t;
  if ( (micros()-t )  < 100000 ) return; //each read is spaced by 100ms
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0X3C);     // I2C write direction
  i2c_write(0X03);         // Start multiple read
  i2c_rep_start(0X3C +1);  // I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC_HMC5883[i]=i2c_readAck();}
  rawADC_HMC5883[5]= i2c_readNak();

  magADC[ROLL]  =   ((rawADC_HMC5883[4]<<8) | rawADC_HMC5883[5]); // note: manual calibration: +300
  magADC[PITCH] =  -((rawADC_HMC5883[0]<<8) | rawADC_HMC5883[1]); // +200
  magADC[YAW]   =  -((rawADC_HMC5883[2]<<8) | rawADC_HMC5883[3]); // +50
}
#endif

// **************************
// standalone I2C NUNCHUK
// **************************
#if defined(NUNCHACK)
static uint8_t rawADC_NUN[6];

void i2c_ACC_init() {
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0xFB); 
  i2c_write(0x00); 
  delay(250);
  accPresent = 1;
}

void i2c_ACC_getADC() {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  i2c_rep_start(0xA4 + 1);//I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++)
    rawADC_NUN[i]=i2c_readAck();
  rawADC_NUN[5]= i2c_readNak();

  accADC[ROLL]  =   ( (rawADC_NUN[3]<<2)        + ((rawADC_NUN[5]>>4)&0x2) );
  accADC[PITCH] = - ( (rawADC_NUN[2]<<2)        + ((rawADC_NUN[5]>>3)&0x2) );
  accADC[YAW]   = - ( ((rawADC_NUN[4]&0xFE)<<2) + ((rawADC_NUN[5]>>5)&0x6) );
}
#endif


// **************************
// I2C Wii Motion Plus 
// **************************
// I2C adress 1: 0xA6 (8bit)    0x53 (7bit)
// I2C adress 2: 0xA4 (8bit)    0x52 (7bit)

static uint8_t rawADC_WMP[6];

void i2c_WMP_init(uint8_t d) {
  delay(d);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  delay(d);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xFE); 
  i2c_write(0x05); 
  delay(d);
  if (d>0) {
    uint8_t numberAccRead = 0;
    for(uint8_t i=0;i<100;i++) {
      delay(3);
      if (rawIMU(0) == 0) numberAccRead++; // we detect here is nunchuk extension is available
    }
    if (numberAccRead>25)
      nunchukPresent = 1;
    delay(10);
  }
}

void i2c_WMP_getRawADC() {
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  i2c_rep_start(0xA4 + 1);//I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++)
    rawADC_WMP[i]=i2c_readAck();
  rawADC_WMP[5]= i2c_readNak();
}

// **************************
// ADC ACC
// **************************
#if defined(ADCACC)
void adc_ACC_init(){
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
}

void adc_ACC_getRawADC() {
  accADC[ROLL]  =  -analogRead(A1);
  accADC[PITCH] =  -analogRead(A2);
  accADC[YAW]   =  -analogRead(A3);

  acc_1G = 75;
  acc_25deg = 32; // = acc_1G * sin(25 deg)
  accPresent = 1;  
}
#endif

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3]  = {0,0,0};
static int16_t angle[2];      //absolute angle inclination in multiple of 0.1 degree
static int16_t accSmooth[3];  //projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer


uint8_t rawIMU(uint8_t withACC) { //if the WMP or NK are oriented differently, it can be changed here
  if (withACC) {
    #if defined(I2C_ACC)
      i2c_ACC_getADC();
    #endif
    #if defined(ADCACC)
      adc_ACC_getRawADC();
    #endif  
  }
  #if defined(ITG3200) || defined(L3G4200D)
    i2c_Gyro_getADC();
    return 1;
  #else
    i2c_WMP_getRawADC();
    if ( (rawADC_WMP[5]&0x02) == 0x02 && (rawADC_WMP[5]&0x01) == 0 ) {// motion plus data
      gyroADC[ROLL]   = - ( ((rawADC_WMP[5]>>2)<<8) + rawADC_WMP[2] );
      gyroADC[PITCH]  = - ( ((rawADC_WMP[4]>>2)<<8) + rawADC_WMP[1] );
      gyroADC[YAW]    = - ( ((rawADC_WMP[3]>>2)<<8) + rawADC_WMP[0] );
      return 1;
    } else if ( (rawADC_WMP[5]&0x02) == 0 && (rawADC_WMP[5]&0x01) == 0) { //nunchuk data
      #if defined(I2C_ACC) || defined(ADCACC)
        return 2;
      #else
        accADC[ROLL]  =   ( (rawADC_WMP[3]<<2)        + ((rawADC_WMP[5]>>4)&0x2) );
        accADC[PITCH] = - ( (rawADC_WMP[2]<<2)        + ((rawADC_WMP[5]>>3)&0x2) );
        accADC[YAW]   = - ( ((rawADC_WMP[4]&0xFE)<<2) + ((rawADC_WMP[5]>>5)&0x6) );
        return 0;
      #endif
    } else
      return 2;
  #endif
}

uint8_t updateIMU(uint8_t withACC) {
  static int32_t g[3];
  static int32_t a[3];
  uint8_t axis;
  static int16_t previousGyroADC[3] = {0,0,0};
  uint8_t r;
  r=rawIMU(withACC);
  
  if (currentTime < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {gyroADC[axis]=0;accADC[axis]=0;}
    accADC[YAW] = acc_1G;
  } else {
    if (r == 1) { //gyro
      if (calibratingG>0) {
        for (axis = 0; axis < 3; axis++) {
          if (calibratingG>1) {
            if (calibratingG == 400) g[axis]=0;
            g[axis] +=gyroADC[axis];
            gyroADC[axis]=0;
          } else {
            gyroZero[axis]=(g[axis]+200)/399;
            blinkLED(10,15,1+3*nunchukPresent);
          }
        }
        calibratingG--;
      }
      #if defined(ITG3200) || defined(L3G4200D)
        gyroADC[ROLL]  = gyroADC[ROLL]  - gyroZero[ROLL];
        gyroADC[PITCH] = gyroADC[PITCH] - gyroZero[PITCH];
        gyroADC[YAW]   = gyroADC[YAW]   - gyroZero[YAW];
      #else
        gyroADC[ROLL]  = gyroADC[ROLL]  - gyroZero[ROLL];
        gyroADC[PITCH] = gyroADC[PITCH] - gyroZero[PITCH];
        gyroADC[YAW]   = gyroADC[YAW]   - gyroZero[YAW];
        gyroADC[ROLL]  = (rawADC_WMP[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL] ;   //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
        gyroADC[PITCH] = (rawADC_WMP[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH] ;  //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
        gyroADC[YAW]   = (rawADC_WMP[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW] ;
      #endif
      //anti gyro glitch, limit the variation between two consecutive readings
      for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-100,previousGyroADC[axis]+100);
        previousGyroADC[axis] = gyroADC[axis];
      }
    }
    if (r == 0 || ( (accPresent == 1) && (withACC == 1) ) ) { //nunchuk or i2c ACC
      if (calibratingA>0) {
        if (calibratingA>1) {
          for (uint8_t axis = 0; axis < 3; axis++) {
            if (calibratingA == 400) a[axis]=0;
            a[axis] +=accADC[axis];
            accADC[axis]=0;
          }
        } else {
          accZero[ROLL]  = (a[ROLL]+200)/399;
          accZero[PITCH] = (a[PITCH]+200)/399;
          accZero[YAW]   = (a[YAW]+200)/399+acc_1G; // for nunchuk 200=1G
          writeParams(); // write accZero in EEPROM
        }
        calibratingA--;
      } else {
        accADC[ROLL]  =    accADC[ROLL]  - accZero[ROLL] ;
        accADC[PITCH] =    accADC[PITCH] - accZero[PITCH];
        accADC[YAW]   = - (accADC[YAW]   - accZero[YAW]) ;
      }
    }
  }  
  return r;
}

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCp[3] = {0,0,0};
  int16_t gyroADCinter[3];
  static int16_t lastAccADC[3] = {0,0,0};
  static int16_t similarNumberAccData[3];
  static int16_t gyroDeviation[3];
  static uint32_t timeInterleave;
  static int16_t gyroYawSmooth = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  if (nunchukPresent) {
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    updateIMU(0);
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    while(updateIMU(0) != 1) ; // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis]+16)/4/8; // /4 is to average 4 values ; /8 is to reduce the sensibility of gyro
      gyroADCprevious[axis] = gyroADC[axis];
    }
  } else {
    #if defined(I2C_ACC) || defined(ADCACC)
      getEstimatedAttitude();
      updateIMU(1); //with I2C or ADC ACC
    #else
      updateIMU(0); //without ACC
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    updateIMU(0); //without ACC
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis]+12)/3/8; // /3 is to average 3 values ; /8 is to reduce the sensibility of gyro
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      #if not defined (I2C_ACC) && not defined (ADCACC)
        accADC[axis]=0;
      #endif
    }
  }
  #if defined(TRI)
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW]+1)/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
}


// ************************************
// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// for angles under 25deg, we use an approximation to speed up the angle calculation
// magnetometer addition for small angles
// ************************************
void getEstimatedAttitude(){
  uint8_t axis;  
  float R, RGyro[3];                 //R obtained from last estimated value and gyro movement;
  static float REst[3] = {0,0,1} ;   // init acc in stable mode
  static float A[2];                 //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
  float wGyro = 300;               // gyro weight/smooting factor
  float invW = 1.0/(1 + 300);
  float gyroFactor;
  static uint8_t small_angle=1;
  static uint16_t tPrevious;
  uint16_t tCurrent,deltaTime;
  float a[2], mag[2], cos_[2];

  tCurrent = micros();
  deltaTime = tCurrent-tPrevious;
  tPrevious = tCurrent;

  #if defined(ITG3200) || defined(L3G4200D)
    gyroFactor = deltaTime/300e6; //empirical
  #else
    gyroFactor = deltaTime/200e6; //empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  #endif
  
  for (axis=0;axis<2;axis++) a[axis] = gyroADC[axis]  * gyroFactor;
  for (axis=0;axis<3;axis++) accSmooth[axis] =(accSmooth[axis]*7+accADC[axis]+4)/8;
  
  if(accSmooth[YAW] > 0 ){ //we want to be sure we are not flying inverted  
    // a very nice trigonometric approximation: under 25deg, the error of this approximation is less than 1 deg:
    //   sin(x) =~= x =~= arcsin(x)
    //   angle_axis = arcsin(ACC_axis/ACC_1G) =~= ACC_axis/ACC_1G
    // the angle calculation is much more faster in this case
    if (accSmooth[ROLL]<acc_25deg && accSmooth[ROLL]>-acc_25deg && accSmooth[PITCH]<acc_25deg && accSmooth[PITCH]>-acc_25deg) {
      for (axis=0;axis<2;axis++) {
        A[axis] +=a[axis];
        A[axis] = ((float)accSmooth[axis]/acc_1G + A[axis]*wGyro)*invW; // =~= sin axis
        #if defined(HMC5843) || defined(HMC5883)
          cos_[axis] = 1-A[axis]*A[axis]/2; // cos(x) =~= 1-x^2/2
        #endif
      } 
      small_angle=1;
    } else {
      //magnitude vector size
      R = sqrt(square(accSmooth[ROLL]) + square(accSmooth[PITCH]) + square(accSmooth[YAW]));
      for (axis=0;axis<2;axis++) {
        if ( acc_1G*3/5 < R && R < acc_1G*7/5 && small_angle == 0 ) //if accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation
          A[axis] = atan2(REst[axis],REst[YAW]);
        A[axis] +=a[axis];
        cos_[axis] = cos(A[axis]);
        RGyro[axis]  = sin(A[axis])  / sqrt( 1.0 + square(cos_[axis])  * square(tan(A[1-axis]))); //reverse calculation of RwGyro from Awz angles
      }
      RGyro[YAW] = sqrt(abs(1.0 - square(RGyro[ROLL]) - square(RGyro[PITCH])));
      for (axis=0;axis<3;axis++)
        REst[axis] = (accADC[axis]/R + wGyro* RGyro[axis])  * invW; //combine Accelerometer and gyro readings
      small_angle=0;
    }
    #if defined(HMC5843) || defined(HMC5883)
      mag[PITCH] = -magADC[PITCH]*cos_[PITCH]+magADC[ROLL]*A[ROLL]*A[PITCH]+magADC[YAW]*cos_[ROLL]*A[PITCH];
      mag[ROLL] = magADC[ROLL]*cos_[ROLL]-magADC[YAW]*A[ROLL];
      heading = -degrees(atan2(mag[PITCH],mag[ROLL]));
    #endif
  }
  for (axis=0;axis<2;axis++) angle[axis] = A[axis]*572.9577951; //angle in multiple of 0.1 degree
}


// *************************
// motor and servo functions
// *************************
uint8_t PWM_PIN[6] = {MOTOR_ORDER};
  
static int16_t axisPID[3];
static int16_t motor[6];
static int16_t servo[4] = {1500,1500,1500,1500};
volatile uint8_t atomicServo[4] = {250,250,250,250};

//for HEX Y6 and HEX6/HEX6X flat and for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;

void writeMotors() { // [1000;2000] => [125;250]
  for(uint8_t i=0;i<min(NUMBER_MOTOR,4);i++)
    analogWrite(PWM_PIN[i], motor[i]>>3);
  #if (NUMBER_MOTOR == 6) && defined(MEGA)
    analogWrite(PWM_PIN[4], motor[4]>>3);
    analogWrite(PWM_PIN[5], motor[5]>>3);
  #endif
  #if (NUMBER_MOTOR == 6) && defined(PROMINI)
    atomicPWM_PIN5_highState = motor[5]/8;
    atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
    atomicPWM_PIN6_highState = motor[4]/8;
    atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
  #endif
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++)
    motor[i]=mc;
  writeMotors();
}

void initializeMotors() {
  for(uint8_t i=0;i<NUMBER_MOTOR;i++)
    pinMode(PWM_PIN[i],OUTPUT);
  writeAllMotors(1000);
  delay(300);
}

#if defined(SERVO)
void initializeServo() {
  #if defined(TRI)
    DIGITAL_SERVO_TRI_PINMODE
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
    DIGITAL_TILT_ROLL_PINMODE
    DIGITAL_TILT_PITCH_PINMODE
  #endif
  #if defined(CAMTRIG)
    DIGITAL_CAM_PINMODE
  #endif
  #if defined(BI)
    DIGITAL_SERVO_TRI_PINMODE
    DIGITAL_BI_LEFT_PINMODE
  #endif
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}

// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
// algorithm strategy:
// pulse high servo 0 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 0
// pulse high servo 1 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 1
// pulse high servo 2 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 2
// pulse high servo 3 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 3
// do nothing for 14 x 1000 us
ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
    #if defined(TRI) || defined (BI)
      DIGITAL_SERVO_TRI_HIGH
    #endif
    OCR0A+= 250; // 1000 us
    state++ ;
  } else if (state == 1) {
    OCR0A+= atomicServo[0]; // 1000 + [0-1020] us
    state++;
  } else if (state == 2) {
    #if defined(TRI) || defined (BI)
      DIGITAL_SERVO_TRI_LOW
    #endif
    #if defined(BI)
      DIGITAL_BI_LEFT_HIGH
    #endif
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
      DIGITAL_TILT_PITCH_HIGH
    #endif
    OCR0A+= 250; // 1000 us
    state++;
  } else if (state == 3) {
    OCR0A+= atomicServo[1]; // 1000 + [0-1020] us
    state++;
  } else if (state == 4) {
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
      DIGITAL_TILT_PITCH_LOW
      DIGITAL_TILT_ROLL_HIGH
    #endif
    #if defined(BI)
      DIGITAL_BI_LEFT_LOW
    #endif
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 5) {
    OCR0A+= atomicServo[2]; // 1000 + [0-1020] us
    state++;
  } else if (state == 6) {
    #if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
      DIGITAL_TILT_ROLL_LOW
    #endif
    #if defined(CAMTRIG)
      DIGITAL_CAM_HIGH
    #endif
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 7) {
    OCR0A+= atomicServo[3]; // 1000 + [0-1020] us
    state++;
  } else if (state == 8) {
    #if defined(CAMTRIG)
      DIGITAL_CAM_LOW
    #endif
    count = 10; // 12 x 1000 us
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 9) {
    if (count > 0) count--;
    else state = 0;
    OCR0A+= 250;
  }
}
#endif

#if (NUMBER_MOTOR == 6) && defined(PROMINI)
void initializeSoftPWM() {
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<5; //digital PIN 5 high
    OCR0A+= atomicPWM_PIN5_highState; //250 x 4 microsecons = 1ms
    state = 1;
  } else if (state == 1) {
    OCR0A+= atomicPWM_PIN5_highState;
    state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<5); //digital PIN 5 low
    OCR0A+= atomicPWM_PIN5_lowState;
    state = 0;
  }
}

ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<6;OCR0B+= atomicPWM_PIN6_highState;state = 1;
  } else if (state == 1) {
    OCR0B+= atomicPWM_PIN6_highState;state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<6);OCR0B+= atomicPWM_PIN6_lowState;state = 0;
  }
}
#endif

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

volatile int16_t failsafeCnt = 0;

static uint8_t pinRcChannel[8] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,CAM1PIN,CAM2PIN};
volatile uint16_t rcPinValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]
static int16_t rcData[8] ; // interval [1000;2000]
static int16_t rcCommand[4] ; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t rcHysteresis[8] ;
static int16_t rcData4Values[8][4];

static uint8_t rcRate8;
static uint8_t rcExpo8;
static float rcFactor1; 
static float rcFactor2;

// ***PPM SUM SIGNAL***
#ifdef SERIAL_SUM_PPM
static uint8_t rcChannel[8] = {SERIAL_SUM_PPM};
#endif
volatile uint16_t rcValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

// Configure each rc pin for PCINT
void configureReceiver() {
  #ifndef SERIAL_SUM_PPM
    for (uint8_t chan = 0; chan < 8; chan++)
      for (uint8_t a = 0; a < 4; a++)
        rcData4Values[chan][a] = 1500; //we initiate the default value of each channel. If there is no RC receiver connected, we will see those values
    #if defined(PROMINI)
      // PCINT activated only for specific pin inside [D0-D7]  , [D2 D4 D5 D6 D7] for this multicopter
      PORTD   = (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTD (no high impedence PINs)
      PCMSK2 |= (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); 
      PCICR   = 1<<2; // PCINT activated only for the port dealing with [D0-D7] PINs
    #endif
    #if defined(MEGA)
      // PCINT activated only for specific pin inside [A8-A15]
      DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
      PORTK   = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
      PCMSK2 |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
      PCICR   = 1<<2; // PCINT activated only for PORTK dealing with [A8-A15] PINs
    #endif
  #else
    PPM_PIN_INTERRUPT
  #endif
}

#ifndef SERIAL_SUM_PPM
ISR(PCINT2_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;

  #if defined(PROMINI)
    pin = PIND;             // PIND indicates the state of each PIN for the arduino port dealing with [D0-D7] digital pins (8 bits variable)
  #endif
  #if defined(MEGA)
    pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
  #endif
  mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

  cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
  
  // mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
  // chan = pin sequence of the port. chan begins at D2 and ends at D7
  if (mask & 1<<2)           //indicates the bit 2 of the arduino port [D0-D7], that is to say digital pin 2, if 1 => this pin has just changed
    if (!(pin & 1<<2)) {     //indicates if the bit 2 of the arduino port [D0-D7] is not at a high state (so that we match here only descending PPM pulse)
      dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcPinValue[2] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
    } else edgeTime[2] = cTime;    // if the bit 2 of the arduino port [D0-D7] is at a high state (ascending PPM pulse), we memorize the time
  if (mask & 1<<4)   //same principle for other channels   // avoiding a for() is more than twice faster, and it's important to minimize execution time in ISR
    if (!(pin & 1<<4)) {
      dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcPinValue[4] = dTime;
    } else edgeTime[4] = cTime;
  if (mask & 1<<5)
    if (!(pin & 1<<5)) {
      dTime = cTime-edgeTime[5]; if (900<dTime && dTime<2200) rcPinValue[5] = dTime;
    } else edgeTime[5] = cTime;
  if (mask & 1<<6)
    if (!(pin & 1<<6)) {
      dTime = cTime-edgeTime[6]; if (900<dTime && dTime<2200) rcPinValue[6] = dTime;
    } else edgeTime[6] = cTime;
  if (mask & 1<<7)
    if (!(pin & 1<<7)) {
      dTime = cTime-edgeTime[7]; if (900<dTime && dTime<2200) rcPinValue[7] = dTime;
    } else edgeTime[7] = cTime;
  #if defined(MEGA)
    if (mask & 1<<0)    
      if (!(pin & 1<<0)) {
        dTime = cTime-edgeTime[0]; if (900<dTime && dTime<2200) rcPinValue[0] = dTime; 
      } else edgeTime[0] = cTime; 
    if (mask & 1<<1)      
      if (!(pin & 1<<1)) {
        dTime = cTime-edgeTime[1]; if (900<dTime && dTime<2200) rcPinValue[1] = dTime; 
      } else edgeTime[1] = cTime;
    if (mask & 1<<3)
      if (!(pin & 1<<3)) {
        dTime = cTime-edgeTime[3]; if (900<dTime && dTime<2200) rcPinValue[3] = dTime;
      } else edgeTime[3] = cTime;
  #endif
  #if defined(FAILSAFE)
    if (mask & 1<<THROTTLEPIN) {    // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
      if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0; }
  #endif
}

#else 
void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = 0;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>3000) chan = 0;
  else {
    if(900<diff && diff<2200 && chan<8 ) rcValue[chan] = diff;
    chan++;
    #if defined(FAILSAFE)
      if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;   // clear FailSafe counter - added by MIS
    #endif
  }
}
#endif

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
  #ifndef SERIAL_SUM_PPM
    data = rcPinValue[pinRcChannel[chan]]; // Let's copy the data Atomically
  #else
    data = rcValue[rcChannel[chan]]; 
  #endif
  SREG = oldSREG;
  sei();// Let's enable the interrupts
  return data; // We return the value correctly copied when the IRQ's where disabled
}
  
void computeRC() {
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;

  rc4ValuesIndex++;
  for (chan = 0; chan < 8; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcData[chan] = 0;
    for (a = 0; a < 4; a++)
      rcData[chan] += rcData4Values[chan][a];
    rcData[chan]= (rcData[chan]+2)/4;
    if ( rcData[chan] < rcHysteresis[chan] -3)  rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +3)  rcHysteresis[chan] = rcData[chan]-2;
  }
}


// ****************
// EEPROM functions
// ****************
static uint8_t P8[3], I8[3], D8[3]; //8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3]; 
static uint8_t PLEVEL8,ILEVEL8;
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t checkNewConf = 134;
static uint8_t activateAcc8,activateBaro8,activateMag8;
static uint8_t activateCamStab8,activateCamTrig8;
#if defined(POWERMETER)
static uint8_t powerTrigger1=0; // trigger for alarm based on power consumption
#endif

void readEEPROM() {
  uint8_t i,p=1;
  for(i=0;i<3;i++) {P8[i] = EEPROM.read(p++);I8[i] = EEPROM.read(p++);D8[i] = EEPROM.read(p++);}
  PLEVEL8 = EEPROM.read(p++);ILEVEL8 = EEPROM.read(p++);
  rcRate8 = EEPROM.read(p++);rcExpo8 = EEPROM.read(p++);
  rollPitchRate = EEPROM.read(p++);
  yawRate = EEPROM.read(p++);
  dynThrPID = EEPROM.read(p++);
  activateAcc8 = EEPROM.read(p++);activateBaro8 = EEPROM.read(p++);activateMag8 = EEPROM.read(p++);
  activateCamStab8 = EEPROM.read(p++);activateCamTrig8 = EEPROM.read(p++);
  for(i=0;i<3;i++) accZero[i] = (EEPROM.read(p++)&0xff) + (EEPROM.read(p++)<<8);
  #if defined(POWERMETER)
  powerTrigger1 = EEPROM.read(p++);
  pAlarm = (uint32_t) powerTrigger1 * (uint32_t) PLEVELSCALE * (uint32_t) PLEVELDIV; // need to cast before multiplying
  #endif
  //note on the following lines: we do this calcul here because it's a static and redundant result and we don't want to load the critical loop whith it
  rcFactor1 = rcRate8/50.0*rcExpo8/100.0/250000.0;
  rcFactor2 = (100-rcExpo8)*rcRate8/5000.0;
}

void writeParams() {
  uint8_t i,p=1;
  EEPROM.write(0, checkNewConf);
  for(i=0;i<3;i++) {EEPROM.write(p++,P8[i]);  EEPROM.write(p++,I8[i]);  EEPROM.write(p++,D8[i]);}
  EEPROM.write(p++,PLEVEL8);EEPROM.write(p++,ILEVEL8);
  EEPROM.write(p++,rcRate8);EEPROM.write(p++,rcExpo8);
  EEPROM.write(p++,rollPitchRate);
  EEPROM.write(p++,yawRate);
  EEPROM.write(p++,dynThrPID);
  EEPROM.write(p++,activateAcc8);EEPROM.write(p++,activateBaro8);EEPROM.write(p++,activateMag8);
  EEPROM.write(p++,activateCamStab8);EEPROM.write(p++,activateCamTrig8);
  for(i=0;i<3;i++) {EEPROM.write(p++,accZero[i]);EEPROM.write(p++,accZero[i]>>8&0xff);}
  #if defined(POWERMETER)
  EEPROM.write(p++,powerTrigger1);
  #endif
  readEEPROM();
  blinkLED(15,20,1);
}

void checkFirstTime() {
  if ( EEPROM.read(0) != checkNewConf ) {
    P8[ROLL] = 40; I8[ROLL] = 30; D8[ROLL] = 15;
    P8[PITCH] = 40; I8[PITCH] = 30; D8[PITCH] = 15;
    P8[YAW]  = 80; I8[YAW]  = 0;  D8[YAW]  = 0;
    PLEVEL8 = 140; ILEVEL8 = 45;
    rcRate8 = 45;
    rcExpo8 = 65;
    rollPitchRate = 0;
    yawRate = 0;
    dynThrPID = 0;
    activateAcc8 = 0;activateBaro8 = 0;activateMag8 = 0;
    activateCamStab8 = 0;activateCamTrig8 = 0;
    powerTrigger1 = 0;
    writeParams();
  }
}

// *****************************
// LCD & display & monitoring
// *****************************

// 1000000 / 9600  = 104 microseconds at 9600 baud.
// we set it below to take some margin with the running interrupts
#define BITDELAY 102
void LCDprint(uint8_t i) {
  #if defined(LCD_TEXTSTAR)
    Serial.print( i , BYTE);
  #else
    LCDPIN_OFF
    delayMicroseconds(BITDELAY);
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      if (i & mask) LCDPIN_ON else LCDPIN_OFF // choose bit
      delayMicroseconds(BITDELAY);
    }
    LCDPIN_ON //switch ON digital PIN 0
    delayMicroseconds(BITDELAY);
  #endif
}

void LCDprintChar(const char *s) {
  while (*s) LCDprint(*s++);
}

void initLCD() {
  blinkLED(20,30,1);
  #if defined(LCD_TEXTSTAR)
    // Cat's Whisker Technologies 'TextStar' Module CW-LCD-02
    // http://cats-whisker.com/resources/documents/cw-lcd-02_datasheet.pdf
    // Modified by Luca Brizzi aka gtrick90 @ RCG
    LCDprint(0xFE);LCDprint(0x43);LCDprint(0x02); //cursor blink mode
    LCDprint(0x0c); //clear screen
    LCDprintChar("MultiWii Config");
    LCDprint(0x0d); // carriage return
    LCDprintChar("for all params");
    delay(2500);
    LCDprint(0x0c); //clear screen
  #else
    Serial.end();
    //init LCD
    PINMODE_LCD //TX PIN for LCD = Arduino RX PIN (more convenient to connect a servo plug on arduino pro mini)
  #endif
}



void configurationLoop() {
  uint8_t chan,i;
  uint8_t param,paramActive;
  uint8_t val,valActive;
  static char line1[17],line2[17];
  uint8_t LCD=1;
  uint8_t refreshLCD = 1;

  typedef struct {
    char*    paramText;
    uint8_t* var;
    uint8_t  decimal;
    uint8_t  increment;
  } paramStruct;
  
  static paramStruct p[] = {
  {"PITCH&ROLL P", &P8[ROLL],1,1},
  {"ROLL   P", &P8[ROLL],1,1},     {"ROLL   I", &I8[ROLL],3,5},  {"ROLL   D", &D8[ROLL],0,1},
  {"PITCH  P", &P8[PITCH],1,1},    {"PITCH  I", &I8[PITCH],3,5}, {"PITCH  D", &D8[PITCH],0,1},
  {"YAW    P", &P8[YAW],1,1},      {"YAW    I", &I8[YAW],3,5},   {"YAW    D", &D8[YAW],0,1},
  {"LEVEL  P", &PLEVEL8,1,1},      {"LEVEL  I", &ILEVEL8,3,5},
  {"RC RATE", &rcRate8,2,2},       {"RC EXPO", &rcExpo8,2,2},
  {"PITCH&ROLL RATE", &rollPitchRate,2,2}, {"YAW RATE", &yawRate,2,2},
  {"THROTTLE PID", &dynThrPID,2,2},
#if defined(POWERMETER)
  {"pMeter Motor 0", &pMeterV,16,0}, //17
  {"pMeter Motor 1", &pMeterV,16,0}, //18
  {"pMeter Motor 2", &pMeterV,16,0}, //19
  {"pMeter Motor 3", &pMeterV,16,0}, //20  
  {"pMeter Motor 4", &pMeterV,16,0}, //21
  {"pMeter Motor 5", &pMeterV,16,0}, //22
  {"pMeter Sum", &pMeterV,16,0}, //23
  {"pAlarm /100", &powerTrigger1,0,1}, //24
  {"Battery Volt", &vbat,1,0}, //25
#endif
  };

  initLCD();
  param = 0;
  while (LCD == 1) {
    if (refreshLCD == 1) {
      strcpy(line2,"                ");
      strcpy(line1,"                ");
      i=0; char* point = p[param].paramText; while (*point) line1[i++] = *point++;
      uint16_t unit = *p[param].var;
      #if defined(POWERMETER)
      if (param > PARAMMOTORSTART && param < PARAMMOTOREND)
        // pmeter values need special treatment, too many digits to fit standard 8 bit scheme
        unit = pMeter[param-PARAMMOTOROFFSET] / PLEVELDIV; // [0:1000] * 1000/3 samples per second(loop time) * 60 seconds *5 minutes -> [0:10000 e4] per motor
                                            // (that is full throttle for 5 minutes sampling with high sampling rate for wmp only)
                                            // times 6 for a maximum of 6 motors equals [0:60000 e4] for the sum
                                            // we are only interested in the big picture, so drop 4 lower digits base 10
      #endif
      if (param == 12) {unit *=2;} // RC RATE can go up to 500
      char c1 = '0'+unit/100; char c2 = '0'+unit/10-(unit/100)*10; char c3 = '0'+unit-(unit/10)*10;
      if (p[param].decimal == 0) {line2[6] = c1;  line2[7] = c2;   line2[8] = c3;}
      if (p[param].decimal == 1) {line2[5] = c1;  line2[6] = c2;   line2[7] = '.'; line2[8] = c3;}
      if (p[param].decimal == 2) {line2[5] = c1;  line2[6] = '.';  line2[7] = c2;  line2[8] = c3;}
      if (p[param].decimal == 3) {line2[4] = '0'; line2[5] = '.';  line2[6] = c1;  line2[7] = c2; line2[8] = c3;}
      #if defined(POWERMETER) // so far, only used for POWERMETER functionality, but otherwise it is general purpose
      if (p[param].decimal == 16) { // not 16 digits but a 16 bit unsigned value; so need 5 digits base 10 to represent [0:65535]
        line2[4] = '0' + unit / 10000;
        line2[5] = '0' + unit / 1000 - (unit/10000) * 10;
        line2[6] = '0' + unit / 100  - (unit/1000)  * 10;
        line2[7] = '0' + unit / 10   - (unit/100)   * 10;
        line2[8] = '0' + unit        - (unit/10)    * 10;
      }
      #endif
      #if defined(LCD_TEXTSTAR)
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
        LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
      #else
        LCDprint(0xFE);LCDprint(128);LCDprintChar(line1);
        LCDprint(0xFE);LCDprint(192);LCDprintChar(line2);
      #endif
      refreshLCD=0;
    }
    for (chan = ROLL; chan < 4; chan++) rcData[chan] = readRawRC(chan);
    //switch config param with pitch
    if (rcData[PITCH] < MINCHECK && paramActive == 0 && param<= PARAMMAX) {
      paramActive = 1;refreshLCD=1;blinkLED(10,20,1);
      param++;
      if (param>PARAMMAX) param=0;
    }
    if (rcData[PITCH] > MAXCHECK && paramActive == 0 && param>=0) {
      paramActive = 1;refreshLCD=1;blinkLED(10,20,1);
      if (param==0) param=PARAMMAX; else param--;
    }
    if (rcData[PITCH] < MAXCHECK && rcData[PITCH] > MINCHECK)  paramActive = 0;
    //+ or - param with low and high roll
    if (rcData[ROLL] < MINCHECK && valActive == 0 && *p[param].var>p[param].increment-1) {
      valActive = 1;refreshLCD=1;blinkLED(10,20,1);
      *p[param].var -= p[param].increment;  //set val -
      if (param == 0) *p[4].var = *p[0].var; //PITCH P
    }
    if (rcData[ROLL] > MAXCHECK && valActive == 0) {
      valActive = 1;refreshLCD=1;blinkLED(10,20,1);
      *p[param].var += p[param].increment;       //set val +
      if (param == 0) *p[4].var = *p[0].var; //PITCH P
    }
    if (rcData[ROLL] < MAXCHECK && rcData[ROLL]  > MINCHECK) valActive = 0;
    if (rcData[YAW]  < MINCHECK && rcData[PITCH] > MAXCHECK) LCD = 0; // save and exit
    if (rcData[YAW]  > MAXCHECK && rcData[PITCH] > MAXCHECK) LCD = 2; // exit without save: eeprom has only 100.000 write cycles
  }
  #if defined(LCD_TEXTSTAR)
    blinkLED(20,30,1);
    LCDprint(0x0c); //clear screen
    if ( LCD == 0) LCDprintChar("Saving Settings.."); else LCDprintChar("skipping Save.");
  #endif
  if ( LCD == 0) writeParams();
  #if defined(LCD_TEXTSTAR)
    LCDprintChar("..done! Exit.");
  #else
    Serial.begin(SERIAL_COM_SPEED);
  #endif
}


void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_SWITCH //switch LEDPIN state
      BUZZERPIN_ON
      delay(wait);
      BUZZERPIN_OFF
    }
    delay(60);
  }
}

/* need this global to have access in serial monitor */
static uint32_t vbatRaw = 0;       //used for smoothing voltage reading
  
void annexCode() { //this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t serialTime;
  static uint32_t buzzerTime;
  static uint32_t calibrateTime;
  static uint32_t calibratedAccTime;
  static uint8_t  buzzerState = 0;
  static uint8_t buzzerFreq;         //delay between buzzer ring
  uint8_t axis;
  uint8_t prop1,prop2;

  for(axis=0;axis<2;axis++) {
    //PITCH & ROLL dynamic PID adjustemnt, depending on stick deviation
    prop1 = 100-min(abs(rcData[axis]-1500)/5,100)*rollPitchRate/100;
    //PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE]<1500)                               prop2 = 100;
    else if (rcData[THROTTLE]>1499 && rcData[THROTTLE]<2000) prop2 = 100 - (rcData[THROTTLE]-1500) * dynThrPID/500;
    else                                                     prop2 = 100 - dynThrPID;
    dynP8[axis] = P8[axis]*prop1/100*prop2/100;
    dynD8[axis] = D8[axis]*prop1/100*prop2/100;
  }
  
  //YAW dynamic PID adjustemnt
  prop1 = 100-min(abs(rcData[YAW]-1500)/5,100)*yawRate/100;
  dynP8[YAW] = P8[YAW]*prop1/100;
  dynD8[YAW] = D8[YAW]*prop1/100;

  #if defined(VBAT)
    vbatRaw = (vbatRaw*15 + analogRead(V_BATPIN)*16)>>4; // smoothing of vbat readings  
    vbat = vbatRaw / VBATSCALE;                          // result is Vbatt in 0.1V steps
     
    if ( (vbat>VBATLEVEL1_3S) 
    #if defined(POWERMETER)
                         && ( (pMeter[6] < pAlarm) || (pAlarm == 0) )
    #endif
                                                                        )
    {                                          //VBAT ok AND powermeter ok, buzzer off
      buzzerFreq = 0; buzzerState = 0; BUZZERPIN_OFF;
    }
    #if defined(POWERMETER)
    else if (pMeter[6] > pAlarm)                              // sound alarm for powermeter
      buzzerFreq = 6;
    #endif
    else if (vbat>VBATLEVEL2_3S)
      buzzerFreq = 1;
    else if (vbat>VBATLEVEL3_3S)
      buzzerFreq = 2;
    else
      buzzerFreq = 4;
    if (buzzerFreq) {
      if (buzzerState && (currentTime > buzzerTime + 250000) ) {
        buzzerState = 0;BUZZERPIN_OFF;buzzerTime = currentTime;
      } else if ( !buzzerState && (currentTime > (buzzerTime + (2000000>>buzzerFreq))) ) {
         buzzerState = 1;BUZZERPIN_ON;buzzerTime = currentTime;
      }
    }
  #endif

  if ( (currentTime > calibrateTime + 100000)  && ( (calibratingA>0 && (nunchukPresent == 1 || accPresent == 1)) || (calibratingG>0) ) ) {  // Calibration phasis
    LEDPIN_SWITCH
    calibrateTime = currentTime;
  } else if ( (calibratingA==0) || (calibratingG==0 && !(nunchukPresent == 1 || accPresent == 1)) ) {
    if (armed) LEDPIN_ON
    else if (calibratedACC == 1) LEDPIN_OFF
  }
  
  if ( currentTime > calibratedAccTime + 500000 ) {
    if ( (nunchukPresent == 1 || accPresent == 1) && (abs(accADC[ROLL])>50 || abs(accADC[PITCH])>50 || abs(accADC[YAW])>400) ) {
      calibratedACC = 0; //the multi uses ACC and is not calibrated or is too much inclinated
      LEDPIN_SWITCH
      calibratedAccTime = currentTime;
    } else
      calibratedACC = 1;
  }
  if (currentTime > serialTime + 20000) { // 50Hz
    serialCom();
    serialTime = currentTime;
  }
  for(axis=0;axis<2;axis++)
    rcCommand[axis]   = (rcHysteresis[axis]-MIDRC) * (rcFactor2 + rcFactor1*square((rcHysteresis[axis]-MIDRC)));
  rcCommand[THROTTLE] = (MAXTHROTTLE-MINTHROTTLE)/(2000.0-MINCHECK) * (rcHysteresis[THROTTLE]-MINCHECK) + MINTHROTTLE;
  rcCommand[YAW]      = rcHysteresis[YAW]-MIDRC;
}


void setup() {
  Serial.begin(SERIAL_COM_SPEED);
  LEDPIN_PINMODE
  POWERPIN_PINMODE
  BUZZERPIN_PINMODE
  POWERPIN_OFF
  initializeMotors();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  delay(200);
  POWERPIN_ON
  delay(100);
  i2c_init();
  delay(100);
  
  #if defined(ITG3200) || defined(L3G4200D)
    i2c_Gyro_init();
  #else
    i2c_WMP_init(250);
  #endif
  
  #if defined(BMP085)
    i2c_Baro_init();
  #endif
  
  #if defined(I2C_ACC) 
    i2c_ACC_init();
  #endif 
  
  #if defined(ADCACC)
    adc_ACC_init();
  #endif

  #if defined(HMC5843) || defined (HMC5883)
    i2c_Mag_init();
  #endif

  #if defined(SERVO)
    initializeServo();
  #elif (NUMBER_MOTOR == 6) && defined(PROMINI)
    initializeSoftPWM();
  #endif
  previousTime = micros();
  #if defined(GIMBAL) || defined(FLYING_WING)
   calibratingA = 400;
  #else
    calibratingA = 0;
  #endif
  calibratingG = 400;
  #if defined(POWERMETER)
  for(uint8_t i=0;i<7;i++) // 6 is the maximum number of possible motors, array is larger by one to append the sum
    pMeter[i]=0; // initialize all counters
  #endif
}

// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error;
  int32_t errorAngle;
  int16_t delta;
  int16_t PTerm,ITerm,DTerm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3];
  static int16_t delta2[3];
  int16_t maxMotor;
  static int32_t errorGyroI[3] = {0,0,0};
  static int32_t errorAngleI[2] = {0,0};
  static int16_t altitudeHold = 0;
  static uint8_t altitudeLock;
  static uint8_t camCycle = 0;
  static uint8_t camState = 0;
  static uint32_t camTime,magTime;
  static uint8_t rcOptions;
  #if defined(POWERMETER)
  uint32_t amp;
  /* real square function */
  /* const uint32_t amperes[16] = {3, 15, 34, 62, 97, 140, 191, 250, 315, 389, 472, 562, 659, 765, 879, 1000 };*/
  /* experimentall curve (above square curve for values > 50%) */
  /* const uint32_t amperes[16] =   {3, 16, 34, 53, 92, 145, 224, 309, 382, 461, 566, 671, 789, 882, 941, 1000 }; */
  /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 1000 */
  const uint32_t amperes[16] =   {31, 246, 831, 1969, 3845, 6645, 10551, 15750, 22425, 30762, 40944, 53156, 67583, 84410, 103821, 126000 };
  #endif

  if (currentTime > (rcTime + 20000) ) { // 50Hz
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && armed==1) {          // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++) {                                        // after specified guard time after RC signal is lost (in 0.1sec)
          rcHysteresis[i] = MIDRC;
          rcData[i] = MIDRC;
        }
        rcHysteresis[THROTTLE] = FAILSAVE_THR0TTLE;
        rcData[THROTTLE] = FAILSAVE_THR0TTLE;
        if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) {  // Turn OFF motors after specified Time (in 0.1sec)
          armed = 0;
          writeAllMotors(MINCOMMAND);
        }
      }
      failsafeCnt++;
    #endif
    // end of failsave routine - next change is made with RcOptions setting
    if (rcData[THROTTLE] < MINCHECK) {
      errorGyroI[ROLL] = 0;
      errorGyroI[PITCH] = 0;
      errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0;
      errorAngleI[PITCH] = 0;
      rcDelayCommand++;
      if ( (rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK)  && armed == 1) {
        if (rcDelayCommand == 20) { // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
          armed = 0;
          writeAllMotors(MINCOMMAND);
        }
      } else if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) calibratingG=400;
      } else if ( (rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
        if (rcDelayCommand == 20) {
          armed = 1;
          writeAllMotors(MINTHROTTLE);
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          atomicServo[0] = 125;  //we center the yaw gyro in conf mode
          #if defined(LCD_CONF)
            configurationLoop(); //beginning LCD configuration
          #endif
          previousTime = micros();
        }
      } else {
        rcDelayCommand = 0;
      }
    } else if (rcData[THROTTLE] > MAXCHECK && armed == 0) {
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {
        if (rcDelayCommand == 20) calibratingA=400;
        rcDelayCommand++;
      } else if (rcData[PITCH] > MAXCHECK) {
         accZero[PITCH]++;writeParams();
      } else if (rcData[PITCH] < MINCHECK) {
         accZero[PITCH]--;writeParams();
      } else if (rcData[ROLL] > MAXCHECK) {
         accZero[ROLL]++;writeParams();
      } else if (rcData[ROLL] < MINCHECK) {
         accZero[ROLL]--;writeParams();
      } else {
        rcDelayCommand = 0;
      }
    }
    #ifdef LOG_VALUES
      else // update max value here, so do not get cycle time of the motor arming (which is way higher than normal)
        if ( (cycleTime > cycleTimeMax) && armed ) cycleTimeMax = cycleTime; // remember highscore
    #endif
    rcOptions = (rcData[AUX1]<1300) + (1300<rcData[AUX1] && rcData[AUX1]<1700)*2 + (rcData[AUX1]>1700)*4
               +(rcData[AUX2]<1300)*8 + (1300<rcData[AUX2] && rcData[AUX2]<1700)*16 + (rcData[AUX2]>1700)*32;
    //note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
    if (((rcOptions & activateAcc8) || (failsafeCnt > 5*FAILSAVE_DELAY) ) && (nunchukPresent == 1 || accPresent == 1)) accMode = 1; else accMode = 0;  // modified by MIS for failsave support
    #if defined(BMP085)
    if (rcOptions & activateBaro8 && baroPresent == 1) {
      if (baroMode == 0) {
        baroMode = 1;
        altitudeHold = altitudeSmooth;
      }
    } else baroMode = 0;
    #endif
    #if defined(HMC5843) || defined(HMC5883)
    if (rcOptions & activateMag8 && magPresent == 1) {
      if (magMode == 0) {
        magMode = 1;
        magHold = heading;
      }
    } else magMode = 0;
    #endif
    rcTime = currentTime; 
  }

  #if defined(HMC5843) || defined(HMC5883)
    i2c_Mag_getADC();
  #endif
  #if defined(BMP085)
    i2c_Baro_update();
  #endif
    
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  #if defined(BMP085)
    if (baroMode) rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE]-3*(altitudeSmooth-altitudeHold),max(rcCommand[THROTTLE]-200,MINTHROTTLE),min(rcCommand[THROTTLE]+200,MAXTHROTTLE));
  #endif
  #if defined(HMC5843) || defined(HMC5883)
    if (-50 < rcCommand[YAW] && rcCommand[YAW] <50 && magMode == 1) {
      int16_t dif = heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      rcCommand[YAW] -= dif;
      magTime = micros();
    } else {
      if (micros() - magTime > 1000 ) { //we add a small timing (1s) to reselect the new compass hold angle
        magHold = heading;
      }
    }
  #endif

  #ifdef BI
    static int16_t gyroPitchSmooth = 0;
    gyroData[PITCH] = ((int32_t)gyroPitchSmooth*10+gyroData[PITCH]+5)/11;
    gyroPitchSmooth = gyroData[PITCH];
  #endif

  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if (accMode == 1 && axis<2 ) { //LEVEL MODE
      errorAngle = rcCommand[axis]/2 - angle[axis]/2;
      PTerm      = (errorAngle)*PLEVEL8/50 - gyroData[axis]*dynP8[axis]/10;
      
      errorAngleI[axis] +=  errorAngle;
      errorAngleI[axis]  = constrain(errorAngleI[axis],-5000,+5000); //WindUp
      ITerm              = errorAngleI[axis] *ILEVEL8/2000;
    } else { //ACRO MODE or YAW axis
      error = rcCommand[axis]*10/P8[axis] - gyroData[axis];
      PTerm = rcCommand[axis]-gyroData[axis]*dynP8[axis]/10;
      
      errorGyroI[axis] += error;
      errorGyroI[axis]  = constrain(errorGyroI[axis],-2000,+2000); //WindUp
      if (abs(gyroData[axis])>80) errorGyroI[axis] = 0;
      ITerm = errorGyroI[axis]*I8[axis]/1000;
    }
    delta          = gyroData[axis] - lastGyro[axis];
    DTerm          = (delta1[axis]+delta2[axis]+delta+1)*dynD8[axis]/3;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
    lastGyro[axis] = gyroData[axis];

    axisPID[axis] =  PTerm + ITerm - DTerm;
  }


  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  #ifdef BI
    motor[0] = rcCommand[THROTTLE] + axisPID[ROLL];                                            //LEFT
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL];                                            //RIGHT
    servo[0]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[1]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = rcCommand[THROTTLE] + axisPID[PITCH]*4/6 ;                 //REAR
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]/6 ; //RIGHT
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]/6 ; //LEFT
    servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #ifdef QUADP
    motor[0] = rcCommand[THROTTLE] + axisPID[PITCH] - YAW_DIRECTION * axisPID[YAW]; //REAR
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL]  + YAW_DIRECTION * axisPID[YAW]; //RIGHT
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL]  + YAW_DIRECTION * axisPID[YAW]; //LEFT
    motor[3] = rcCommand[THROTTLE] - axisPID[PITCH] - YAW_DIRECTION * axisPID[YAW]; //FRONT
  #endif
  #ifdef QUADX
    motor[0] = rcCommand[THROTTLE] - axisPID[ROLL] + axisPID[PITCH] - YAW_DIRECTION * axisPID[YAW]; //REAR_R
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH] + YAW_DIRECTION * axisPID[YAW]; //FRONT_R
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL] + axisPID[PITCH] + YAW_DIRECTION * axisPID[YAW]; //REAR_L
    motor[3] = rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH] - YAW_DIRECTION * axisPID[YAW]; //FRONT_L
  #endif
  #ifdef Y4
    motor[0] = rcCommand[THROTTLE]                  + axisPID[PITCH] - YAW_DIRECTION * axisPID[YAW]; //REAR_1
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL]  - axisPID[PITCH];                                //FRONT_R
    motor[2] = rcCommand[THROTTLE]                  + axisPID[PITCH] + YAW_DIRECTION * axisPID[YAW]; //REAR_2
    motor[3] = rcCommand[THROTTLE] + axisPID[ROLL]  - axisPID[PITCH];                                //FRONT_L
  #endif
  #ifdef Y6
    motor[0] = rcCommand[THROTTLE]                 + axisPID[PITCH]*4/3 + YAW_DIRECTION * axisPID[YAW]; //REAR
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]*2/3 - YAW_DIRECTION * axisPID[YAW]; //RIGHT
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]*2/3 - YAW_DIRECTION * axisPID[YAW]; //LEFT
    motor[3] = rcCommand[THROTTLE]                 + axisPID[PITCH]*4/3 - YAW_DIRECTION * axisPID[YAW]; //UNDER_REAR
    motor[4] = rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]*2/3 + YAW_DIRECTION * axisPID[YAW]; //UNDER_RIGHT
    motor[5] = rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]*2/3 + YAW_DIRECTION * axisPID[YAW]; //UNDER_LEFT
  #endif
  #ifdef HEX6
    motor[0] = rcCommand[THROTTLE] - axisPID[ROLL]/2 + axisPID[PITCH]/2 + YAW_DIRECTION * axisPID[YAW]; //REAR_R
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL]/2 - axisPID[PITCH]/2 - YAW_DIRECTION * axisPID[YAW]; //FRONT_R
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL]/2 + axisPID[PITCH]/2 + YAW_DIRECTION * axisPID[YAW]; //REAR_L
    motor[3] = rcCommand[THROTTLE] + axisPID[ROLL]/2 - axisPID[PITCH]/2 - YAW_DIRECTION * axisPID[YAW]; //FRONT_L
    motor[4] = rcCommand[THROTTLE]                   - axisPID[PITCH]   + YAW_DIRECTION * axisPID[YAW]; //FRONT
    motor[5] = rcCommand[THROTTLE]                   + axisPID[PITCH]   - YAW_DIRECTION * axisPID[YAW]; //REAR
  #endif
  #ifdef HEX6X
    motor[0] = rcCommand[THROTTLE] - axisPID[ROLL]/2 + axisPID[PITCH]/2 + YAW_DIRECTION * axisPID[YAW]; //REAR_R
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL]/2 - axisPID[PITCH]/2 + YAW_DIRECTION * axisPID[YAW]; //FRONT_R
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL]/2 + axisPID[PITCH]/2 - YAW_DIRECTION * axisPID[YAW]; //REAR_L
    motor[3] = rcCommand[THROTTLE] + axisPID[ROLL]/2 - axisPID[PITCH]/2 - YAW_DIRECTION * axisPID[YAW]; //FRONT_L
    motor[4] = rcCommand[THROTTLE] - axisPID[ROLL]                      - YAW_DIRECTION * axisPID[YAW]; //RIGHT
    motor[5] = rcCommand[THROTTLE] + axisPID[ROLL]                      + YAW_DIRECTION * axisPID[YAW]; //LEFT
  #endif
  #ifdef SERVO_TILT
    if (rcOptions & activateCamStab8 ) {
      servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 , TILT_PITCH_MIN, TILT_PITCH_MAX);
      servo[2] = constrain(TILT_ROLL_MIDDLE  + TILT_ROLL_PROP  * angle[ROLL]  /16 , TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
      servo[1] = TILT_PITCH_MIDDLE;
      servo[2] = TILT_ROLL_MIDDLE;
    }
  #endif
  #ifdef GIMBAL
    servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP   * angle[ROLL]  /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #ifdef FLYING_WING
    servo[1]  = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000); //LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
    servo[2]  = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000); //RIGHT
  #endif

  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (armed == 0)
      motor[i] = MINCOMMAND;
  }

  #if defined(CAMTRIG)
    if (camCycle==1) {
      if (camState == 0) {
        servo[3] = CAM_SERVO_HIGH;
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
       if ( (millis() - camTime) > CAM_TIME_HIGH ) {
         servo[3] = CAM_SERVO_LOW;
         camState = 2;
         camTime = millis();
       }
      } else { //camState ==2
       if ( (millis() - camTime) > CAM_TIME_LOW ) {
         camState = 0;
         camCycle = 0;
       }
      }
    } 
    if (rcOptions & activateCamTrig8) camCycle=1;
  #endif

  #if defined(SERVO)
    atomicServo[0] = (servo[0]-1000)/4;
    atomicServo[1] = (servo[1]-1000)/4;
    atomicServo[2] = (servo[2]-1000)/4;
    atomicServo[3] = (servo[3]-1000)/4;
  #endif
  
  writeMotors();
  
  #if defined(POWERMETER)
  if (vbat) { // by all means - must avoid division by zero 
    for (uint8_t i =0;i<NUMBER_MOTOR;i++) {

      amp = amperes[(motor[i] - 1000)>>6] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 16 ranges; lookup amp
      pMeter[i]+= amp; // sum up over time the mapped ESC input 
                               // this is poor man's integral
      pMeter[6]+= amp; // total sum over all motors
    }
  }
  #endif
}

static uint8_t point;
static uint8_t s[128];
void serialize16(int16_t a) {s[point++]  = a; s[point++]  = a>>8&0xff;}
void serialize8(uint8_t a)  {s[point++]  = a;}

// ***********************************
// Interrupt driven UART transmitter for MIS_OSD
// ***********************************
static uint8_t tx_ptr;

ISR_UART {
  UDR0 = s[tx_ptr++];         /* Transmit next byte */
  if ( tx_ptr == point )        /* Check if all data is transmitted */
    UCSR0B &= ~(1<<UDRIE0);     /* Disable transmitter UDRE interrupt */
}

void UartSendData() {      // start of the data block transmission
  tx_ptr = 0;
  UCSR0A |= (1<<UDRE0);        /* Clear UDRE interrupt flag */
  UCSR0B |= (1<<UDRIE0);       /* Enable transmitter UDRE interrupt */
  UDR0 = s[tx_ptr++];          /* Start transmission */
}

void serialCom() {
  int16_t a;
  uint8_t i;
  uint16_t intPowerMeterSum, intPowerTrigger1;    
#ifdef LCD_TELEMETRY
  char line1[17],line2[17];
  static uint8_t telemetry = 0;

  switch (telemetry) { // output telemetry data, if one of four modes is set
  case 'A': // button A on Textstar LCD -> cycle time
    strcpy(line1,"Cycle    _____us"); //uin16_t cycleTime
    /*            0123456789.12345*/
    strcpy(line2,"CycleMax _____us"); //uin16_t cycleTimeMax
    line1[9] = '0' + cycleTime / 10000;
    line1[10] = '0' + cycleTime / 1000 - (cycleTime/10000) * 10;
    line1[11] = '0' + cycleTime / 100  - (cycleTime/1000)  * 10;
    line1[12] = '0' + cycleTime / 10   - (cycleTime/100)   * 10;
    line1[13] = '0' + cycleTime        - (cycleTime/10)    * 10;
  #ifdef LOG_VALUES
    line2[9] = '0' + cycleTimeMax / 10000;
    line2[10] = '0' + cycleTimeMax / 1000 - (cycleTimeMax/10000) * 10;
    line2[11] = '0' + cycleTimeMax / 100  - (cycleTimeMax/1000)  * 10;
    line2[12] = '0' + cycleTimeMax / 10   - (cycleTimeMax/100)   * 10;
    line2[13] = '0' + cycleTimeMax        - (cycleTimeMax/10)    * 10;
    LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
  #endif
    LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
    break;
  case 'B': // button B on Textstar LCD -> Voltage, PowerSum and power alarm trigger value
    strcpy(line1,"__._V "); //uint8_t vbat, uint32_t vbatRaw
    /*            0123456789.12345*/
    strcpy(line2,"Psum _____   ___"); // intPowerMeterSum, intPowerTrigger1
  #ifdef VBAT
    line1[0] = '0'+vbat/100; line1[1] = '0'+vbat/10-(vbat/100)*10; line1[3] = '0'+vbat-(vbat/10)*10;
  #endif
  #ifdef POWERMETER
    intPowerMeterSum = (pMeter[6]/PLEVELDIV);
    line2[5] = '0' + intPowerMeterSum / 10000;
    line2[6] = '0' + intPowerMeterSum / 1000 - (intPowerMeterSum/10000) * 10;
    line2[7] = '0' + intPowerMeterSum / 100  - (intPowerMeterSum/1000)  * 10;
    line2[8] = '0' + intPowerMeterSum / 10   - (intPowerMeterSum/100)   * 10;
    line2[9] = '0' + intPowerMeterSum        - (intPowerMeterSum/10)    * 10;
    line2[13] = '0'+powerTrigger1/100; line2[14] = '0'+powerTrigger1/10-(powerTrigger1/100)*10; line2[15] = '0'+powerTrigger1-(powerTrigger1/10)*10;
  #endif
    LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
    LCDprint(0xFE);LCDprint('b');LCDprint(10);LCDprint(((vbat-VBATLEVEL1_3S)*100)/VBATREF); // bar graph
    LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
    break;
  case 'C': // button C on Textstar LCD -> angles 
    uint16_t unit;
    strcpy(line1,"AngX  ___._ deg ");
    /*            0123456789.12345*/
    strcpy(line2,"AngY  ___._ deg ");
    if (angle[0] < 0 ) {
      unit = -angle[0];
      line1[5] = '-';
    } else 
      unit = angle[0];
    //line1[5] = '0' + unit / 10000;
    line1[6] = '0' + unit / 1000; //- (unit/10000) * 10;
    line1[7] = '0' + unit / 100  - (unit/1000)  * 10;
    line1[8] = '0' + unit / 10   - (unit/100)   * 10;
    line1[10] = '0' + unit       - (unit/10)    * 10;
    if (angle[1] < 0 ) {
      unit = -angle[1];
      line2[5] = '-';
    } else 
      unit = angle[1];
    //line2[5] = '0' + unit / 10000;
    line2[6] = '0' + unit / 1000; //- (unit/10000) * 10;
    line2[7] = '0' + unit / 100  - (unit/1000)  * 10;
    line2[8] = '0' + unit / 10   - (unit/100)   * 10;
    line2[10] = '0' + unit       - (unit/10)    * 10;
    LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
    LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
    break;      
  case 'D': // button C on Textstar LCD 
    strcpy(line1,"Button D        ");
    /*            0123456789.12345*/
    strcpy(line2,"         pressed");
    LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
    LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
    break; 
  }
#endif
  if (Serial.available()) {
    switch (Serial.read()) {
    #ifdef LCD_TELEMETRY
      case 'A': // button A press
      if (telemetry=='A') telemetry = 0; else { telemetry = 'A'; LCDprint(12); /* clear screen */ }
      break;    
      case 'B': // button B press
      if (telemetry=='B') telemetry = 0; else { telemetry = 'B'; LCDprint(12); /* clear screen */ }
      break;    
      case 'C': // button C press
      if (telemetry=='C') telemetry = 0; else { telemetry = 'C'; LCDprint(12); /* clear screen */ }
      break;    
      case 'D': // button D press
      if (telemetry=='D') telemetry = 0; else { telemetry = 'D'; LCDprint(12); /* clear screen */ }
      break;
      case 'a': // button A release
      case 'b': // button B release
      case 'c': // button C release
      case 'd': // button D release
      break;      
    #endif
      case 'M': // Multiwii @ arduino to GUI all data
      point=0;
      serialize8('M');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]); //12
      serialize16(altitudeSmooth);
      serialize16(heading); // compass
      for(i=0;i<4;i++) serialize16(servo[i]); //24
      for(i=0;i<6;i++) serialize16(motor[i]); //36
      for(i=0;i<8;i++) serialize16(rcHysteresis[i]); //52
      serialize8(nunchukPresent|accPresent<<1|baroPresent<<2|magPresent<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(angle[i]/10); //60
    #if defined(TRI)
      serialize8(1);
    #elif defined(QUADP)
      serialize8(2);
    #elif defined(QUADX)
      serialize8(3);
    #elif defined(BI)
      serialize8(4);
    #elif defined(GIMBAL)
      serialize8(5);
    #elif defined(Y6)
      serialize8(6);
    #elif defined(HEX6)
      serialize8(7);
    #elif defined(FLYING_WING)
      serialize8(8);
    #elif defined(Y4)
      serialize8(9);
    #elif defined(HEX6X)
      serialize8(10);
    #endif
      for(i=0;i<3;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}//70
      serialize8(PLEVEL8);serialize8(ILEVEL8);
      serialize8(rcRate8); serialize8(rcExpo8);
      serialize8(rollPitchRate); serialize8(yawRate);
      serialize8(dynThrPID);
      serialize8(activateAcc8);serialize8(activateBaro8);serialize8(activateMag8);//80
      serialize8(activateCamStab8);serialize8(activateCamTrig8);//82

      #if defined(POWERMETER)
      intPowerMeterSum = (pMeter[6]/PLEVELDIV);
      intPowerTrigger1 = powerTrigger1 * PLEVELSCALE;
      #else
      intPowerMeterSum = 0;  
      intPowerTrigger1 = 0;   
      #endif    
      serialize16(intPowerMeterSum);  
      serialize16(intPowerTrigger1);
      serialize8(vbat);
      serialize8('M');
      Serial.write(s,point);
      break;
    case 'O':  // arduino to OSD data - contribution from MIS
      point=0;
      serialize8('O');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      serialize16(altitudeSmooth);
      serialize16(heading); // compass - 16 bytes
      for(i=0;i<2;i++) serialize16(angle[i]); //20
      for(i=0;i<6;i++) serialize16(motor[i]); //32
      for(i=0;i<6;i++) {serialize16(rcHysteresis[i]);} //44
      serialize8(nunchukPresent|accPresent<<1|baroPresent<<2|magPresent<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize8(vbat);     // Vbatt 47
      serialize8(17);  // MultiWii Firmware version
      serialize8('O'); //49
      UartSendData();
      break;
    case 'E': //GUI write params to eeprom @ arduino
      while (Serial.available()<23) {}
      for(i=0;i<3;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();}
      PLEVEL8 = Serial.read(); ILEVEL8 = Serial.read();
      rcRate8 = Serial.read(); rcExpo8 = Serial.read();
      rollPitchRate = Serial.read(); yawRate = Serial.read();
      dynThrPID = Serial.read();
      activateAcc8 = Serial.read();activateBaro8 = Serial.read();activateMag8 = Serial.read();
      activateCamStab8 = Serial.read();activateCamTrig8 = Serial.read();
      powerTrigger1 = (Serial.read() + 256* Serial.read() ) / PLEVELSCALE; // we rely on writeParams() to compute corresponding pAlarm value
      writeParams();
      break;
    case 'S': //GUI to arduino Sensor calibration reques
      calibratingA=400;
      break;
    }
  }
}
