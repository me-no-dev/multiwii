/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
April  2011     V1.8
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include <EEPROM.h>


#if defined(PROMINI)
  #define LEDPIN_PINMODE             ;pinMode (13, OUTPUT);
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
  #define V_BATPIN                   3    // Analog PIN 3
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
  #define V_BATPIN                   3    // Analog PIN 3
#endif

#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
  #define SERVO
#endif

#if defined(ADXL345) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK)
  #define I2C_ACC
#endif

#if defined(HMC5883) || defined(HMC5843)
  #define I2C_MAG
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
static uint16_t meanTime = 2000;    // this is the average time of the loop: around 2ms for a WMP config and 6ms for a NK+WMP config
static uint16_t calibratingA;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingG;
static uint8_t armed = 0;
static int16_t acc_1G = 200;       //this is the 1G measured acceleration (nunchuk)
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


// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3]  = {0,0,0};
static int16_t angle[2];      //absolute angle inclination in multiple of 0.1 degree
static int16_t accSmooth[3];  //projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer




// *************************
// motor and servo functions
// *************************
uint8_t PWM_PIN[6] = {MOTOR_ORDER};
  
static int16_t axisPID[3];
static int16_t motor[6];
static int16_t servo[4] = {1500,1500,1500,1500};
volatile uint8_t atomicServo[4] = {250,250,250,250};



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
  static uint32_t calibrateTime;
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

  #if defined(I2C_MAG)
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
    #if defined(I2C_MAG)
    if (rcOptions & activateMag8 && magPresent == 1) {
      if (magMode == 0) {
        magMode = 1;
        magHold = heading;
      }
    } else magMode = 0;
    #endif
    rcTime = currentTime; 
  }

  #if defined(I2C_MAG)
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
  #if defined(I2C_MAG)
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
    motor[0] = rcCommand[THROTTLE] + axisPID[PITCH]*4/3 ;                 //REAR
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]*2/3 ; //RIGHT
    motor[2] = rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]*2/3 ; //LEFT
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
    motor[0] = rcCommand[THROTTLE]                  + axisPID[PITCH] - YAW_DIRECTION * axisPID[YAW]; //REAR_1 CW
    motor[1] = rcCommand[THROTTLE] - axisPID[ROLL]  - axisPID[PITCH];                                //FRONT_R CCW
    motor[2] = rcCommand[THROTTLE]                  + axisPID[PITCH] + YAW_DIRECTION * axisPID[YAW]; //REAR_2 CCW
    motor[3] = rcCommand[THROTTLE] + axisPID[ROLL]  - axisPID[PITCH];                                //FRONT_L CW
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
}


