/*
MultiWiiCopter by Alexandre Dubus
 www.multiwii.com
 March  2013     V2.2
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
 
 May 2013 V2.2.9
 Major rewrite by Prof Greg Egan.
 Reduction to a "Lite" version to support Atmel 32u4 processors using MPU6050 
 sensors with 4 motors, no servos and faster PWM for brushed DC motors.
 Control replaced with that from UAVX. etc. etc.
 */

#include <avr/io.h>

#include "config.h"
#include "def.h"


#include <avr/pgmspace.h>
#define  VERSION  229 

/*********** RC alias *****************/
enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

const char pidnames[] PROGMEM =
"ROLL;"
"PITCH;"
"YAW;"
"ALT;"
"Pos;"
"PosR;"
"NavR;"
"LEVEL;"
"MAG;"
"VEL;"
;

enum box {
  BOXARM,
  BOXANGLE,
  BOXHORIZON,
  CHECKBOXITEMS
};

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
"ARM;"
"ANGLE;"
"HORIZON;"
;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
  1, //"ANGLE;"
  2, //"HORIZON;"
};

static uint32_t currentTimeuS = 0;
static uint32_t rcTimeuS  = 0;
bool rcFrameOK = false;
bool rcNewValues = false;
static uint16_t previousCycleuS = 0;
static uint32_t nextCycleuS = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
static uint16_t calibratingG;
static uint16_t acc_1G;            // this is the 1G measured acceleration
static uint16_t acc_25deg;
static int16_t  gyroADC[3], accADC[3], magADC[3];
static int16_t  accSmooth[3] = {0, 0, 512};
static int16_t  heading, magHold, headFreeModeHold; // [-180;+180]
static uint8_t  vbat; // battery voltage in 0.1V steps
static uint8_t  vbatMin = 99;  // lowest battery voltage in 0.1V steps
static uint8_t  rcOptions[CHECKBOXITEMS];
static int16_t  maxRollPitchStick = 0;
static int32_t  BaroAlt, EstAlt, AltHold; // in cm
static int16_t  BaroPID = 0;
static int16_t  errorAltitudeI = 0;
static int16_t  vario = 0;              // variometer in cm/s

static bool inFailsafe = false;
static int16_t  rcGlitches = 0;
static int16_t  debug[4];
static int16_t  sonarAlt; //to think about the unit

struct flags_struct {
uint8_t OK_TO_ARM :
  1 ;
uint8_t ARMED :
  1 ;
uint8_t I2C_INIT_DONE :
  1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
uint8_t ACC_CALIBRATED :
  1 ;
uint8_t NUNCHUKDATA :
  1 ;
uint8_t ANGLE_MODE :
  1 ;
uint8_t HORIZON_MODE :
  1 ;
uint8_t MAG_MODE :
  1 ;
uint8_t BARO_MODE :
  1 ;
uint8_t GPS_HOME_MODE :
  1 ;
uint8_t GPS_HOLD_MODE :
  1 ;
uint8_t HEADFREE_MODE :
  1 ;
uint8_t PASSTHRU_MODE :
  1 ;
uint8_t GPS_FIX :
  1 ;
uint8_t GPS_FIX_HOME :
  1 ;
uint8_t SMALL_ANGLES_25 :
  1 ;
uint8_t CALIBRATE_MAG :
  1 ;
uint8_t VARIO_MODE :
  1;
} 
f;

static int16_t  i2c_errors_count = 0;
static int16_t  annex650_overrun_count = 0;

// **********************
// power meter
// **********************

static uint16_t intPowerMeterSum, intPowerTrigger1;

// ******************
// rc functions
// ******************
#define MINCHECK 1200 // 1100
#define MAXCHECK 1800 // 1900

#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

static int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

static int16_t rcData[RC_CHANS];    // interval [1000;2000]
static int16_t rcCommand[RC_CHANS];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
static int16_t lookupPitchRollRC[6];// lookup table for expo & RC rate PITCH+ROLL
static int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE
static uint16_t rssi;               // range: [0;1023]

#if defined(SPEKTRUM)
volatile uint8_t  spekFrameFlags;
volatile uint32_t spekTimeLast;
#endif

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = {
  0,0,0};
static int16_t gyroZero[3] = {
  0,0,0};
static int16_t angle[2]    = {
  0,0};  // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// **********************
// PID common variables
// **********************
int16_t RateIntE[3] = {
  0,0,0};
int16_t AngleIntE[2] = {
  0,0};

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[NUMBER_MOTOR];

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[3], dynD8[3];

static struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  bool accCalibrated;
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE ! 
} 
global_conf;


static struct {
  uint8_t checkNewConf;
  uint8_t P8[PIDITEMS], I8[PIDITEMS], D8[PIDITEMS];
  uint8_t rcRate8;
  uint8_t rcExpo8;
  uint8_t rollPitchRate;
  uint8_t yawRate;
  uint8_t dynThrPID;
  uint8_t thrMid8;
  uint8_t thrExpo8;
  int16_t accTrim[2];
  uint16_t activate[CHECKBOXITEMS];
  uint8_t powerTrigger1;
  int16_t failsafe_throttle;
  uint16_t cycletimeuS;
  int16_t minthrottle;
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE ! 
} 
conf;


// **********************
// GPS common variables
// **********************
static int32_t  GPS_coord[2];
static int32_t  GPS_home[2];
static int32_t  GPS_hold[2];
static uint8_t  GPS_numSat;
static uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
static int16_t  GPS_directionToHome;                         // direction to home - unit: degree
static uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
static uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
static uint8_t  GPS_update = false;                              // a binary toogle to distinct a GPS position update
static int16_t  GPS_angle[2] = { 
  0, 0};                      // the angles that must be applied for GPS correction
static uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
static uint8_t  GPS_Enable  = 0;

#define LAT  0
#define LON  1
// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
static int16_t  nav[2];
static int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial GPS only variables
//navigation mode
#define NAV_MODE_NONE          0
#define NAV_MODE_POSHOLD       1
#define NAV_MODE_WP            2
static uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) { // DO NOT CALL IN FLIGHT - USES DELAYS
  uint8_t i,r;
  
  for (r = 0; r < repeat; r++) {
    for(i = 0; i<num; i++) {
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
    }
    delay(60); 
  }
} // blinkLED

void annexCode() { // this code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds

  if ( (calibratingA>0 ) || (calibratingG>0) ) // Calibration phase
    LEDPIN_TOGGLE;
  else
    if (f.ACC_CALIBRATED)
      LEDPIN_OFF;
    else
      if (f.ARMED)
        LEDPIN_ON;
    
  doRates();
  serialCom();

} // annexCode


void go_arm(void) {
  
  f.ARMED = (calibratingG == 0) && f.ACC_CALIBRATED;

  if(!f.ARMED) 
    blinkLED(2, 255, 1);
} // go_arm

void go_disarm(void) {
  f.ARMED = false; 
} // go_disarm


void doStickArming(uint8_t rcSticks, bool stickArm) {
  
  if ( conf.activate[BOXARM] == 0 ) {
    if (stickArm) {
#if defined(ALLOW_ARM_DISARM_VIA_TX_YAW)
        if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm(); 
#endif
#if defined(ALLOW_ARM_DISARM_VIA_TX_ROLL)
        if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm(); 
#endif        
    } else {
#if defined(ALLOW_ARM_DISARM_VIA_TX_ROLL)
        if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();
#endif
#if defined(ALLOW_ARM_DISARM_VIA_TX_ROLL)
        if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm(); 
#endif
    }
  } 
} // doStickArming
  

void doStickProgramming(void) {
  static uint8_t rcDelayCommand = 0; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks; // this hold sticks position for command combos
  bool updateParams;
  uint8_t stTmp = 0;
  uint16_t auxState = 0;
  uint8_t i;

  for(i = 0; i < 4; i++) {
    stTmp >>= 2;
    if(rcData[i] > MINCHECK) 
      stTmp |= 0x80; // check for MIN
    if(rcData[i] < MAXCHECK) 
      stTmp |= 0x40; // check for MAX
  }

  if(stTmp == rcSticks) {
    if(rcDelayCommand < 250) 
      rcDelayCommand++;
  }
  else 
    rcDelayCommand = 0;
  rcSticks = stTmp;

  // perform actions    
  if (rcData[THROTTLE] <= MINCHECK) { 
    RateIntE[ROLL] = RateIntE[PITCH] = RateIntE[YAW] = AngleIntE[ROLL] = AngleIntE[PITCH] = 0;
    if (conf.activate[BOXARM] > 0) // Arming via ARM BOX
      if ( rcOptions[BOXARM] && f.OK_TO_ARM ) 
        go_arm(); 
      else 
        if (f.ARMED) 
        go_disarm();
  }

  if(rcDelayCommand == 20) {
    if( f.ARMED) 
      doStickArming(rcSticks, false);
    else {

      doStickArming(rcSticks, true);

      if (!f.ARMED) {

        switch (rcSticks) {
        case THR_LO + YAW_LO + PIT_LO + ROL_CE: calibratingG = 512; break;
        case THR_HI + YAW_LO + PIT_LO + ROL_CE: calibratingA = 512; break;
        case THR_LO + YAW_HI + PIT_HI + ROL_CE: break; // Enter LCD config
        case THR_HI + YAW_HI + PIT_LO + ROL_CE: f.CALIBRATE_MAG = true; break; 
        default: 
          break;
        } // switch

        updateParams = true;
        switch (rcSticks) { 
        case THR_HI + YAW_CE + PIT_HI + ROL_CE: conf.accTrim[PITCH] ++; break;
        case THR_HI + YAW_CE + PIT_LO + ROL_CE: conf.accTrim[PITCH] --; break;
        case THR_HI + YAW_CE + PIT_CE + ROL_HI: conf.accTrim[ROLL] ++; break;
        case THR_HI + YAW_CE + PIT_CE + ROL_LO: conf.accTrim[ROLL] --; break;     
        default: updateParams = false; break;
        } // switch

        if ( updateParams ) {
          writeParams(1);
          rcDelayCommand = 0; // allow autorepetition
        }
      }
    }
  }

  for(i = 0; i<4; i++)
    auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);

  for(i = 0; i < CHECKBOXITEMS; i++)
    rcOptions[i] = (auxState & conf.activate[i])>0;

  // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false

  if ( rcOptions[BOXANGLE] || inFailsafe ) { // bumpless transfer to Level mode
    if (!f.ANGLE_MODE) {
      AngleIntE[ROLL] = AngleIntE[PITCH] = 0;
      f.ANGLE_MODE = true;
    }  
  } 
  else // failsafe support
  f.ANGLE_MODE = false;

  if ( rcOptions[BOXHORIZON] ) {
    f.ANGLE_MODE = false;
    if (!f.HORIZON_MODE) {
      AngleIntE[ROLL] = AngleIntE[PITCH] = 0;
      f.HORIZON_MODE = true;
    }
  } 
  else 
    f.HORIZON_MODE = false;

  if (rcOptions[BOXARM] == 0) 
    f.OK_TO_ARM = true; 

} // doStickProgramming


void setup() {  

  SerialOpen(0,SERIAL0_COM_SPEED);
  SerialOpen(1,SERIAL1_COM_SPEED);

  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  global_conf.currentSet=0;
  readEEPROM();
  readGlobalSet();
  readEEPROM();                                    // load current setting data
  blinkLED(2,40,global_conf.currentSet+1);          
  configureReceiver();
  initSensors();
  previousCycleuS = micros();
  calibratingG = 512;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles

  ADCSRA |= _BV(ADPS2) ; 
  ADCSRA &= ~_BV(ADPS1); 
  ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11

  f.ARMED = false;
  f.SMALL_ANGLES_25 = false; 

} // setup


void loop (void) {

  cycleTime = micros() - previousCycleuS;

  while(micros() < nextCycleuS) {
  }; // wait

  currentTimeuS = previousCycleuS = micros();
  nextCycleuS = previousCycleuS + conf.cycletimeuS;

  getRatesAndAccelerations();
  getEstimatedAttitude();  
  computeControl();
  mixTable();
  writeMotors();

  if (rxReady()){ 
    computeRC(); 
    doStickProgramming();
  }
  
  annexCode();

} // loop

















