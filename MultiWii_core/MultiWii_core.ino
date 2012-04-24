/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
March  2012     V2.0
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/


#include "core_config.h"
#include <avr/pgmspace.h>
#define  VERSION  200


/*********** RC alias *****************/

#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

#define PIDALT     3
#define PIDVEL     4
#define PIDGPS     5
#define PIDLEVEL   6
#define PIDMAG     7

#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10

#define CHECKBOXITEMS 11
#define PIDITEMS 8

static uint8_t  armed = 0;
static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint8_t  calibratedACC = 0;
static int16_t  acc_25deg;
static uint16_t acc_1G;             // this is the 1G measured acceleration
static uint8_t  nunchuk = 0;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static uint8_t  okToArm = 0;
static uint8_t  rcOptions[CHECKBOXITEMS];
static int16_t  debug1,debug2,debug3,debug4;
static int16_t  accTrim[2] = {0, 0};
static uint8_t  accMode = 0;        // if level mode is a activated
static uint8_t  headTX,tailTX;
static uint8_t  bufTX[256];      // 256 is choosen to avoid modulo operations on 8 bits pointers

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

volatile uint16_t rcValue[18] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
volatile int16_t failsafeCnt = 0;
static int16_t failsafeEvents = 0;
static int16_t rcData[8];          // interval [1000;2000]
static int16_t rcCommand[4];       // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static uint8_t rcRate8;
static uint8_t rcExpo8;
static int16_t lookupRX[7];       // lookup table for expo & RC rate
volatile uint8_t rcFrameComplete; // for serial rc receiver Spektrum
static uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages

// **************
// gyro+acc IMU
// **************
uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3]  = {0,0,0};
static int16_t magZero[3]  = {0,0,0};
static int16_t angle[2]    = {0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static int8_t  smallAngle25 = 1;

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[8] = {1500,1500,1500,1500,1500,1500,1500,1500};

// **********************
// EEPROM 
// **********************
static uint8_t P8[8], I8[8], D8[8]; // 8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint16_t activate[CHECKBOXITEMS];

//log
static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      BUZZERPIN_ON;
      delay(wait);
      BUZZERPIN_OFF;
    }
    delay(60);
  }
}


void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  static uint8_t  buzzerFreq;         // delay between buzzer ring
  uint8_t axis,prop1,prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<1500) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)dynThrPID*(rcData[THROTTLE]-1500)/500;
    } else {
      prop2 = 100 - dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    uint16_t tmp = min(abs(rcData[axis]-MIDRC),500);
    if(axis!=2) { //ROLL & PITCH
      uint16_t tmp2 = tmp/100;
      rcCommand[axis] = lookupRX[tmp2] + (tmp-tmp2*100) * (lookupRX[tmp2+1]-lookupRX[tmp2]) / 100;
      prop1 = 100-(uint16_t)rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  rcCommand[THROTTLE] = MINTHROTTLE + (int32_t)(MAXTHROTTLE-MINTHROTTLE)* (rcData[THROTTLE]-MINCHECK)/(2000-MINCHECK);
  
  if ( (calibratingA>0 && (ACC || nunchuk) ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (calibratedACC == 1) {LEDPIN_OFF;}
    if (armed) {LEDPIN_ON;}
  }

  if ( currentTime > calibratedAccTime ) {
    if (smallAngle25 == 0) {
      calibratedACC = 0; // the multi uses ACC and is not calibrated or is too much inclinated
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 500000;
    } else
      calibratedACC = 1;
  }

  serialCom();
}

void setup() {
  SerialOpen(0,SERIAL_COM_SPEED);
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  initOutput();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  initSensors();
  previousTime = micros();
  calibratingG = 400;
}


// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error,errorAngle;
  int16_t delta,deltaSum;
  int16_t PTerm,ITerm,DTerm;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3],delta2[3];
  static int16_t errorGyroI[3] = {0,0,0};
  static int16_t errorAngleI[2] = {0,0};
  static uint32_t rcTime  = 0;
  static int16_t initialThrottleHold;
  
  #if defined(SPEKTRUM)
    if (rcFrameComplete) computeRC();
  #endif
  #if defined(OPENLRSv2MULTI) 
    Read_OpenLRS_RC();
  #endif 
  
  if (currentTime > rcTime ) { // 50Hz
    rcTime = currentTime + 20000;
    computeRC();
    if (rcData[THROTTLE] < MINCHECK) {
      errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
      rcDelayCommand++;
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          calibratingG=400;
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {       
          previousTime = micros();
        }
      }
      else if (activate[BOXARM] > 0) {
        if ( rcOptions[BOXARM] && okToArm ) {
	  armed = 1;
        } else if (armed) armed = 0;
        rcDelayCommand = 0;
      } else if ( (rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK)  && armed == 1) {
        if (rcDelayCommand == 20) armed = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
      } else if ( (rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
        if (rcDelayCommand == 20) {
	  armed = 1;
        }
      } else
        rcDelayCommand = 0;
    } else if (rcData[THROTTLE] > MAXCHECK && armed == 0) {
      if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {        // throttle=max, yaw=left, pitch=min
        if (rcDelayCommand == 20) calibratingA=400;
        rcDelayCommand++;
      } else if (rcData[PITCH] > MAXCHECK) {
         accTrim[PITCH]+=2;writeParams(1);
      } else if (rcData[PITCH] < MINCHECK) {
         accTrim[PITCH]-=2;writeParams(1);
      } else if (rcData[ROLL] > MAXCHECK) {
         accTrim[ROLL]+=2;writeParams(1);
      } else if (rcData[ROLL] < MINCHECK) {
         accTrim[ROLL]-=2;writeParams(1);
      } else {
        rcDelayCommand = 0;
      }
    }

    for(i=0;i<CHECKBOXITEMS;i++) {   
      rcOptions[i] = (
       ((rcData[AUX1]<1300)    | (1300<rcData[AUX1] && rcData[AUX1]<1700)<<1 | (rcData[AUX1]>1700)<<2
       |(rcData[AUX2]<1300)<<3 | (1300<rcData[AUX2] && rcData[AUX2]<1700)<<4 | (rcData[AUX2]>1700)<<5) & activate[i])>0;
    }

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
    if (rcOptions[BOXACC] && ACC) { 
      // bumpless transfer to Level mode
      if (!accMode) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        accMode = 1;
      }  
    } else accMode = 0;

    if (rcOptions[BOXARM] == 0) okToArm = 1;
  }
 
  computeIMU();
  // Measure loop rate just afer reading the sensors
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  //**** PITCH & ROLL & YAW PID ****    
  for(axis=0;axis<3;axis++) {
    if (accMode == 1 && axis<2 ) { //LEVEL MODE
      // 50 degrees max inclination
      errorAngle = constrain(2*rcCommand[axis],-500,+500) - angle[axis] + accTrim[axis]; //16 bits is ok here
      #ifdef LEVEL_PDF
        PTerm      = -(int32_t)angle[axis]*P8[PIDLEVEL]/100 ;
      #else  
        PTerm      = (int32_t)errorAngle*P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
      #endif
      PTerm = constrain(PTerm,-D8[PIDLEVEL]*5,+D8[PIDLEVEL]*5);

      errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
      ITerm              = ((int32_t)errorAngleI[axis]*I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
    } else { //ACRO MODE or YAW axis
      if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                               else error = (int32_t)rcCommand[axis]*10*8/P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
      error -= gyroData[axis];

      PTerm = rcCommand[axis];
      
      errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
      if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
      ITerm = (errorGyroI[axis]/125*I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
    }
    if (abs(gyroData[axis])<160) PTerm -=          gyroData[axis]*dynP8[axis]/10/8; // 16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
                            else PTerm -= (int32_t)gyroData[axis]*dynP8[axis]/10/8; // 32 bits is needed for calculation   

    delta          = gyroData[axis] - lastGyro[axis];                               // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro[axis] = gyroData[axis];
    deltaSum       = delta1[axis]+delta2[axis]+delta;
    delta2[axis]   = delta1[axis];
    delta1[axis]   = delta;
 
    if (abs(deltaSum)<640) DTerm = (deltaSum*dynD8[axis])>>5;                       // 16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
                      else DTerm = ((int32_t)deltaSum*dynD8[axis])>>5;              // 32 bits is needed for calculation
                      
    axisPID[axis] =  PTerm + ITerm - DTerm;
  }

  mixTable();
  writeServos();
  writeMotors();
}
