/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
April  2011     V1.dev
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include <EEPROM.h>

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
  #define STABLEPIN_PINMODE          ;
  #define STABLEPIN_ON               ;
  #define STABLEPIN_OFF              ;
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
  #define V_BATPIN                   3    // Analog PIN 3
#endif
#if defined(MEGA)
  #define LEDPIN_PINMODE             pinMode (13, OUTPUT);
  #define LEDPIN_SWITCH              PINB |= (1<<7);
  #define LEDPIN_ON                  PORTB |= (1<<7);
  #define LEDPIN_OFF                 PORTB &= ~(1<<7);
  #define BUZZERPIN_PINMODE          pinMode (31, OUTPUT);
  #define BUZZERPIN_ON               PORTC |= 1<<6;
  #define BUZZERPIN_OFF              PORTC &= ~1<<6;
  #define POWERPIN_PINMODE           pinMode (37, OUTPUT);
  #define POWERPIN_ON                PORTC |= 1<<0;
  #define POWERPIN_OFF               PORTC &= ~(1<<0);
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
  #define PINMODE_LCD                pinMode(0, OUTPUT);
  #define LCDPIN_OFF                 PORTE &= ~1;      //switch OFF digital PIN 0
  #define LCDPIN_ON                  PORTE |= 1;       //switch OFF digital PIN 0
  #define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
  #define STABLEPIN_ON               PORTC |= 1<<6;
  #define STABLEPIN_OFF              PORTC &= ~1<<6;
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT); //PIN 2 //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
  #define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);
  #define DIGITAL_TILT_PITCH_PINMODE pinMode(33,OUTPUT); // 33
  #define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<4;
  #define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<4);
  #define DIGITAL_TILT_ROLL_PINMODE  pinMode(34,OUTPUT);pinMode(44,OUTPUT); // 34 + 44
  #define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<3;PORTL |= 1<<5;
  #define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<3);PORTL |= 1<<5;
  #define DIGITAL_BI_LEFT_PINMODE    pinMode(6,OUTPUT); 
  #define DIGITAL_BI_LEFT_HIGH       PORTE |= 1<<6;
  #define DIGITAL_BI_LEFT_LOW        PORTE &= ~(1<<6);
  #define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);  //PIN 19
  #define MOTOR_ORDER                3,5,6,2,7,8,9,10   //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
  #define DIGITAL_CAM_PINMODE        pinMode(35,OUTPUT);pinMode(45,OUTPUT); // 35 + 45
  #define DIGITAL_CAM_HIGH           PORTC |= 1<<2;PORTL |= 1<<4;
  #define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);PORTL |= 1<<4;
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
  #define V_BATPIN                   3    // Analog PIN 3
#endif


#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(ADCACC)
  #define ACC
#endif

#if defined(HMC5883) || defined(HMC5843)
  #define MAG
#endif

#if defined(ITG3200) || defined(L3G4200D)
  #define GYRO
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

static uint32_t currentTime;
static uint16_t previousTime;
static uint32_t neutralizeTime;
static uint16_t cycleTime;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint8_t armed = 0;
static int16_t acc_1G = 200;       //this is the 1G measured acceleration (nunchuk)
static uint8_t nunchukPresent = 0;
static uint8_t accPresent = 0;     //I2C or ADC acc or Nunchuk present
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
static uint8_t okToArm = 0;
static uint8_t rcOptions;
  
// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

volatile int16_t failsafeCnt = 0;

static int16_t rcData[8];    // interval [1000;2000]
static int16_t rcCommand[4]; // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 

static uint8_t rcRate8;
static uint8_t rcExpo8;
static float rcFactor1; 
static float rcFactor2;


// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3]  = {0,0,0};
static int16_t magZero[3]  = {0,0,0};
static int16_t angle[2]    = {0,0};  //absolute angle inclination in multiple of 0.1 degree

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[4] = {1500,1500,1500,1500};

// ****************
// EEPROM functions
// ****************
static uint8_t P8[3], I8[3], D8[3]; //8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3]; 
static uint8_t PLEVEL8,ILEVEL8;
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t activateAcc8,activateBaro8,activateMag8;
static uint8_t activateCamStab8,activateCamTrig8,activateArm8;

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

void annexCode() { //this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t serialTime;
  static uint32_t buzzerTime;
  static uint32_t calibratedAccTime;
  static uint8_t  buzzerState = 0;
  static uint32_t vbatRaw = 0;       //used for smoothing voltage reading
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
    vbat = vbatRaw / VBATSCALE;                  // result is Vbatt in 0.1V steps
     
    if (vbat>VBATLEVEL1_3S) {
      buzzerFreq = 0; buzzerState = 0; BUZZERPIN_OFF;
    } else if (vbat>VBATLEVEL2_3S)
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

  if ( ( (calibratingA>0 && (accPresent == 1)) || (calibratingG>0) ) ) {  // Calibration phasis
    LEDPIN_SWITCH
  } else {
    if (calibratedACC == 1) LEDPIN_OFF
    if (armed) LEDPIN_ON
  }

  if ( currentTime > calibratedAccTime + 500000 ) {
    if (abs(angle[ROLL])>150 || abs(angle[PITCH])>150) { //more than 15 deg detection
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
    rcCommand[axis]   = (rcData[axis]-MIDRC) * (rcFactor2 + rcFactor1*square((rcData[axis]-MIDRC)));
  rcCommand[THROTTLE] = (MAXTHROTTLE-MINTHROTTLE)/(2000.0-MINCHECK) * (rcData[THROTTLE]-MINCHECK) + MINTHROTTLE;
  rcCommand[YAW]      = rcData[YAW]-MIDRC;
}


void setup() {
  Serial.begin(SERIAL_COM_SPEED);
  LEDPIN_PINMODE
  POWERPIN_PINMODE
  BUZZERPIN_PINMODE
  STABLEPIN_PINMODE
  POWERPIN_OFF
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  initSensors();
 
  previousTime = micros();
  #if defined(GIMBAL) || defined(FLYING_WING)
   calibratingA = 400;
  #endif
  calibratingG = 400;
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
  static int32_t errorGyroI[3] = {0,0,0};
  static int32_t errorAngleI[2] = {0,0};
  static int16_t altitudeHold = 0;
  static uint8_t altitudeLock;
  static uint8_t camCycle = 0;
  static uint8_t camState = 0;
  static uint16_t camTime;
  static uint32_t magTime;
  static uint32_t rcTime;

  if (currentTime > (rcTime + 20000) ) { // 50Hz
    rcTime = currentTime; 
    computeRC();
    // Failsafe routine - added by MIS
    #if defined(FAILSAFE)
      if ( failsafeCnt > (5*FAILSAVE_DELAY) && armed==1) {          // Stabilize, and set Throttle to specified level
        for(i=0; i<3; i++)                                          // after specified guard time after RC signal is lost (in 0.1sec)
          rcData[i] = MIDRC;
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
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      rcDelayCommand++;
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) calibratingG=400;
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          servo[0] = 1500; //we center the yaw gyro in conf mode
          writeServos();
          #if defined(LCD_CONF)
            configurationLoop(); //beginning LCD configuration
          #endif
          previousTime = micros();
        }
      } else if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) calibratingG=400;
      } else if (activateArm8 > 0) {
        if ((rcOptions & activateArm8) && okToArm) armed = 1;
        else if (armed) armed = 0;
        rcDelayCommand = 0;
      } else if ( (rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK)  && armed == 1) {
        if (rcDelayCommand == 20) armed = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
        if (rcDelayCommand == 20) armed = 1;
      } else
        rcDelayCommand = 0;
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
    rcOptions = (rcData[AUX1]<1300) + (1300<rcData[AUX1] && rcData[AUX1]<1700)*2 + (rcData[AUX1]>1700)*4
               +(rcData[AUX2]<1300)*8 + (1300<rcData[AUX2] && rcData[AUX2]<1700)*16 + (rcData[AUX2]>1700)*32;
    //note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
    if (((rcOptions & activateAcc8) || (failsafeCnt > 5*FAILSAVE_DELAY) ) && (accPresent == 1)) accMode = 1; else accMode = 0;  // modified by MIS for failsave support
    if ((rcOptions & activateArm8) == 0) okToArm = 1;
    if (accMode == 1) STABLEPIN_ON else STABLEPIN_OFF;

    #if defined(BMP085)
    if (rcOptions & activateBaro8 && baroPresent == 1) {
      if (baroMode == 0) {
        baroMode = 1;
        altitudeHold = altitudeSmooth;
      }
    } else baroMode = 0;
    #endif
    #if defined(MAG)
    if (rcOptions & activateMag8 && magPresent == 1) {
      if (magMode == 0) {
        magMode = 1;
        magHold = heading;
      }
    } else magMode = 0;
    #endif
  }

  #if defined(MAG)
    Mag_getADC();
    if (-50 < rcCommand[YAW] && rcCommand[YAW] <50 && magMode == 1) {
      int16_t dif = heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      rcCommand[YAW] -= dif;
      magTime = millis();
    } else {
      if ( millis() - magTime > 1000 ) { //we add a small timing (1s) to reselect the new compass hold angle
        magHold = heading;
      }
    }
  #endif
  #if defined(BMP085)
    Baro_update();
    if (baroMode) rcCommand[THROTTLE] = constrain(rcCommand[THROTTLE]-3*(altitudeSmooth-altitudeHold),max(rcCommand[THROTTLE]-200,MINTHROTTLE),min(rcCommand[THROTTLE]+200,MAXTHROTTLE));
  #endif
    
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;


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
  
  mixTable();
  writeServos();
  writeMotors();
}


