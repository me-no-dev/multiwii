/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
July  2012     V2.1
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "def.h"

uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;
int16_t  annex650_overrun_count = 0;
int16_t  headFreeModeHold;

uint8_t dynP8[3], dynD8[3];

int16_t  debug[4];

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      BUZZERPIN_ON;
      delay(wait);
      BUZZERPIN_OFF;
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delay(60);
  }
}

void annexCode() { // this code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp,tmp2;
  #if defined(BUZZER)
    static uint8_t  buzzerFreq;         // delay between buzzer ring
  #endif
  uint8_t axis,prop1,prop2;

  #define BREAKPOINT 1500
  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  if   (rcData[THROTTLE]<BREAKPOINT) {
    prop2 = 100;
  } else {
    if (rcData[THROTTLE]<2000) {
      prop2 = 100 - (uint16_t)conf.dynThrPID*(rcData[THROTTLE]-BREAKPOINT)/(2000-BREAKPOINT);
    } else {
      prop2 = 100 - conf.dynThrPID;
    }
  }

  for(axis=0;axis<3;axis++) {
    tmp = min(abs(rcData[axis]-MIDRC),500);
    #if defined(DEADBAND)
      if (tmp>DEADBAND) { tmp -= DEADBAND; }
      else { tmp=0; }
    #endif
    if(axis!=2) { //ROLL & PITCH
      tmp2 = tmp/100;
      rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp-tmp2*100) * (lookupPitchRollRC[tmp2+1]-lookupPitchRollRC[tmp2]) / 100;
      prop1 = 100-(uint16_t)conf.rollPitchRate*tmp/500;
      prop1 = (uint16_t)prop1*prop2/100;
    } else {      // YAW
      rcCommand[axis] = tmp;
      prop1 = 100-(uint16_t)conf.yawRate*tmp/500;
    }
    dynP8[axis] = (uint16_t)conf.P8[axis]*prop1/100;
    dynD8[axis] = (uint16_t)conf.D8[axis]*prop1/100;
    if (rcData[axis]<MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE],MINCHECK,2000);
  tmp = (uint32_t)(tmp-MINCHECK)*1000/(2000-MINCHECK); // [MINCHECK;2000] -> [0;1000]
  tmp2 = tmp/100;
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp-tmp2*100) * (lookupThrottleRC[tmp2+1]-lookupThrottleRC[tmp2]) / 100; // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH]*cosDiff + rcCommand[ROLL]*sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL]*cosDiff - rcCommand[PITCH]*sinDiff; 
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  #if defined(POWERMETER_HARD)
    uint16_t pMeterRaw;               // used for current reading
    static uint16_t psensorTimer = 0;
    if (! (++psensorTimer % PSENSORFREQ)) {
      pMeterRaw =  analogRead(PSENSORPIN);
      powerValue = ( PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL); // do not use abs(), it would induce implicit cast to uint and overrun
      if ( powerValue < 333) {  // only accept reasonable values. 333 is empirical
      #ifdef LCD_TELEMETRY
        if (powerValue > powerMax) powerMax = powerValue;
      #endif
      } else {
        powerValue = 333;
      }        
      pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
    }
  #endif

  #if defined(VBAT)
    static uint8_t vbatTimer = 0;
    uint16_t vbatRaw = 0;

    if (! (++vbatTimer % VBATFREQ)) {
    	vbatRaw = analogRead(V_BATPIN);
    	vbat = (int16_t)(vbatRaw * VBATSCALE);                  // result is Vbatt in 0.1V steps
    }
    if ( ( (vbat>VBATLEVEL1_3S) 
    #if defined(POWERMETER)
                         && ( (pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0) )
    #endif
                       )  || (NO_VBAT>vbat)                              ) // ToLuSe
    {                                          // VBAT ok AND powermeter ok, buzzer off
      buzzerFreq = 0;
    #if defined(POWERMETER)
    } else if (pMeter[PMOTOR_SUM] > pAlarm) {                             // sound alarm for powermeter
      buzzerFreq = 4;
    #endif
    } else if (vbat>VBATLEVEL2_3S) buzzerFreq = 1;
    else if (vbat>VBATLEVEL3_3S)   buzzerFreq = 2;
    else                           buzzerFreq = 4;
  #endif
  #if defined(BUZZER)
    buzzer(buzzerFreq); // external buzzer routine that handles buzzer events globally now
  #endif
  
  if ( (calibratingA>0 && ACC ) || (calibratingG>0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {LEDPIN_OFF;}
    if (f.ARMED) {LEDPIN_ON;}
  }

  #if defined(LED_RING)
    static uint32_t LEDTime;
    if ( currentTime > LEDTime ) {
      LEDTime = currentTime + 50000;
      i2CLedRingState();
    }
  #endif

  #if defined(LED_FLASHER)
    auto_switch_led_flasher();
  #endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 500000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

  #if defined(GPS_PROMINI)
    if(GPS_Enable == 0) {serialCom();}
  #else
    serialCom();
  #endif

  #if defined(POWERMETER)
    intPowerMeterSum = (pMeter[PMOTOR_SUM]/PLEVELDIV);
    intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE; 
  #endif

  #ifdef LCD_TELEMETRY_AUTO
    static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
    static uint8_t telemetryAutoIndex = 0;
    static uint16_t telemetryAutoTimer = 0;
    if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ){
      telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
      LCDclear(); // make sure to clear away remnants
    }
  #endif  
  #ifdef LCD_TELEMETRY
    static uint16_t telemetryTimer = 0;
    if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
      #if (LCD_TELEMETRY_DEBUG+0 > 0)
        telemetry = LCD_TELEMETRY_DEBUG;
      #endif
      if (telemetry) lcd_telemetry();
    }
  #endif
  
  #if GPS & defined(GPS_LED_INDICATOR)
    static uint32_t GPSLEDTime;
    if ( currentTime > GPSLEDTime && (GPS_numSat >= 5)) {
      GPSLEDTime = currentTime + 150000;
      LEDPIN_TOGGLE;
    }
  #endif

  #if defined(LOG_VALUES) && (LOG_VALUES == 2)
    if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
    if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
  #endif
  #ifdef LCD_TELEMETRY
    if (f.ARMED) armedTime += (uint32_t)cycleTime;
    #if BARO
      if (!f.ARMED) {
        BAROaltStart = BaroAlt;
        BAROaltMax = BaroAlt;
      } else {
        if (BaroAlt > BAROaltMax) BAROaltMax = BaroAlt;
      }
    #endif
  #endif
}

int main() {

	  uint16_t failsafeCnt = 0;//zzz dummy dec
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
	  #ifdef LCD_TELEMETRY_STEP
	    static char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
	    static uint8_t telemetryStepIndex = 0;
	  #endif

	InitClocks();

	Delay1mS(1000); // let things settle!

	InitHarness();
	InitPortCombination();
	Delay1mS(100);

  #if !defined(GPS_PROMINI)
    SerialOpen(0,SERIAL_COM_BAUD);
  #endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readEEPROM();
 //zzz configureReceiver();
  initSensors();
  #if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)
    GPS_set_pids();
  #endif
  previousTime = micros();
  #if defined(GIMBAL)
   calibratingA = 400;
  #endif
  calibratingG = 400;
  #if defined(POWERMETER)
    for(uint8_t i=0;i<=PMOTOR_SUM;i++)
      pMeter[i]=0;
  #endif
  #if defined(ARMEDTIMEWARNING)
    ArmedTimeWarningMicroSeconds = (ARMEDTIMEWARNING *1000000);
  #endif
  /************************************/
  #if defined(GPS_SERIAL)
    SerialOpen(GPS_SERIAL,GPS_BAUD);  
    delay(400);  
    for(uint8_t i=0;i<=5;i++){
      GPS_NewData(); 
      LEDPIN_ON
      delay(20);
      LEDPIN_OFF
      delay(80);
    }
    if(!GPS_Present){
      SerialEnd(GPS_SERIAL);
      SerialOpen(0,SERIAL_COM_BAUD);
    }      
    #if !defined(GPS_PROMINI)
      GPS_Present = 1;
    #endif
    GPS_Enable = GPS_Present;    
  #endif
  /************************************/
 
  #if defined(I2C_GPS) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
   GPS_Enable = 1;
  #endif
  
  #if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64)
    initLCD();
  #endif
  #ifdef LCD_TELEMETRY_DEBUG
    telemetry_auto = 1;
  #endif
  #ifdef LCD_CONF_DEBUG
    configurationLoop();
  #endif
  #ifdef LANDING_LIGHTS_DDR
    init_landing_lights();
  #endif

  #if defined(LED_FLASHER)
    init_led_flasher();
    led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
  #endif
  f.SMALL_ANGLES_25=1; // important for gyro only conf

  while (1) {

  #if defined(SPEKTRUM)
    if (rcFrameComplete) computeRC();
  #endif

  #define RC_FREQ 50

    if (currentTime > rcTime ) { // 50Hz

    	//SendMinPacket(0);

      rcTime = currentTime + 20000;
      computeRC();
      // Failsafe routine - added by MIS
      #if defined(FAILSAFE)
        if ( failsafeCnt > (5*FAILSAVE_DELAY) && f.ARMED) {                  // Stabilize, and set Throttle to specified level
          for(i=0; i<3; i++) rcData[i] = MIDRC;                               // after specified guard time after RC signal is lost (in 0.1sec)
          rcData[THROTTLE] = FAILSAVE_THROTTLE;
          if (failsafeCnt > 5*(FAILSAVE_DELAY+FAILSAVE_OFF_DELAY)) {          // Turn OFF motors after specified Time (in 0.1sec)
            f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
            f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
          }
          failsafeEvents++;
        }
        if ( failsafeCnt > (5*FAILSAVE_DELAY) && !f.ARMED) {  //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
            f.ARMED = 0;   // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
            f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
        }
        failsafeCnt++;
      #endif
      // end of failsave routine - next change is made with RcOptions setting
      if (rcData[THROTTLE] < MINCHECK) {
        errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        rcDelayCommand++;
        if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && !f.ARMED) {
          if (rcDelayCommand == 20) {
            calibratingG=400;
            #if GPS
              GPS_reset_home_position();
            #endif
          }
        } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
          if (rcDelayCommand == 20) {
            #ifdef TRI
              servo[5] = 1500; // we center the yaw servo in conf mode
              writeServos();
            #endif
            #ifdef FLYING_WING
              servo[0]  = conf.wing_left_mid;
              servo[1]  = conf.wing_right_mid;
              writeServos();
            #endif
            #ifdef AIRPLANE
              for(i = 4; i<7 ;i++) servo[i] = 1500;
              writeServos();
            #endif
            #if defined(LCD_CONF)
              configurationLoop(); // beginning LCD configuration
            #endif
            previousTime = micros();
          }
        }
        #if defined(INFLIGHT_ACC_CALIBRATION)
          else if (!f.ARMED && rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK && rcData[ROLL] > MAXCHECK){
            if (rcDelayCommand == 20){
              if (AccInflightCalibrationMeasurementDone){                // trigger saving into eeprom after landing
                AccInflightCalibrationMeasurementDone = 0;
                AccInflightCalibrationSavetoEEProm = 1;
              }else{
                AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
                #if defined(BUZZER)
                if (AccInflightCalibrationArmed){
                  toggleBeep = 2;
                } else {
                  toggleBeep = 3;
                }
                #endif
              }
            }
         }
       #endif
        else if (conf.activate[BOXARM] > 0) {
          if ( rcOptions[BOXARM] && f.OK_TO_ARM
          #if defined(FAILSAFE)
            && failsafeCnt <= 1
          #endif
          ) {
  	  f.ARMED = 1;
  	  headFreeModeHold = heading;
          } else if (f.ARMED) f.ARMED = 0;
          rcDelayCommand = 0;
        #ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        } else if ( (rcData[YAW] < MINCHECK )  && f.ARMED) {
          if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
        } else if ( (rcData[YAW] > MAXCHECK ) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
          if (rcDelayCommand == 20) {
  	  f.ARMED = 1;
  	  headFreeModeHold = heading;
          }
        #endif
        #ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        } else if ( (rcData[ROLL] < MINCHECK)  && f.ARMED) {
          if (rcDelayCommand == 20) f.ARMED = 0; // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
        } else if ( (rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && !f.ARMED && calibratingG == 0 && f.ACC_CALIBRATED) {
          if (rcDelayCommand == 20) {
            f.ARMED = 1;
            headFreeModeHold = heading;
          }
        #endif
        #ifdef LCD_TELEMETRY_AUTO
        } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
          if (rcDelayCommand == 20) {
             if (telemetry_auto) {
                telemetry_auto = 0;
                telemetry = 0;
             } else
                telemetry_auto = 1;
          }
        #endif
        #ifdef LCD_TELEMETRY_STEP
        } else if (rcData[ROLL] > MAXCHECK && rcData[PITCH] > MAXCHECK && !f.ARMED) {
          if (rcDelayCommand == 20) {
            telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
            LCDclear(); // make sure to clear away remnants
          }
        #endif
        } else
          rcDelayCommand = 0;
      } else if (rcData[THROTTLE] > MAXCHECK && !f.ARMED) {
        if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {        // throttle=max, yaw=left, pitch=min
          if (rcDelayCommand == 20) calibratingA=400;
          rcDelayCommand++;
        } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) { // throttle=max, yaw=right, pitch=min
          if (rcDelayCommand == 20) f.CALIBRATE_MAG = 1; // MAG calibration request
          rcDelayCommand++;
        } else if (rcData[PITCH] > MAXCHECK) {
           conf.angleTrim[PITCH]+=2;writeParams(1);
           #if defined(LED_RING)
             blinkLedRing();
           #endif
        } else if (rcData[PITCH] < MINCHECK) {
           conf.angleTrim[PITCH]-=2;writeParams(1);
           #if defined(LED_RING)
             blinkLedRing();
           #endif
        } else if (rcData[ROLL] > MAXCHECK) {
           conf.angleTrim[ROLL]+=2;writeParams(1);
           #if defined(LED_RING)
             blinkLedRing();
           #endif
        } else if (rcData[ROLL] < MINCHECK) {
           conf.angleTrim[ROLL]-=2;writeParams(1);
           #if defined(LED_RING)
             blinkLedRing();
           #endif
        } else {
          rcDelayCommand = 0;
        }
      }
      #if defined(LED_FLASHER)
        led_flasher_autoselect_sequence();
      #endif

      #if defined(INFLIGHT_ACC_CALIBRATION)
        if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ){ // Copter is airborne and you are turning it off via boxarm : start measurement
          InflightcalibratingA = 50;
          AccInflightCalibrationArmed = 0;
        }
        if (rcOptions[BOXPASSTHRU]) {      // Use the Passthru Option to activate : Passthru = TRUE Meausrement started, Land and passtrhu = 0 measurement stored
          if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone){
            InflightcalibratingA = 50;
          }
        }else if(AccInflightCalibrationMeasurementDone && !f.ARMED){
          AccInflightCalibrationMeasurementDone = 0;
          AccInflightCalibrationSavetoEEProm = 1;
        }
      #endif

      uint16_t auxState = 0;
      for(i=0;i<4;i++)
        auxState |= (rcData[AUX1+i]<1300)<<(3*i) | (1300<rcData[AUX1+i] && rcData[AUX1+i]<1700)<<(3*i+1) | (rcData[AUX1+i]>1700)<<(3*i+2);
      for(i=0;i<CHECKBOXITEMS;i++)
        rcOptions[i] = (auxState & conf.activate[i])>0;

      // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
      if (( rcOptions[BOXACC] || (failsafeCnt > 5*FAILSAVE_DELAY) ) && ACC ) {
        // bumpless transfer to Level mode
        if (!f.ACC_MODE) {
          errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
          f.ACC_MODE = 1;
        }
      } else {
        // failsafe support
        f.ACC_MODE = 0;
      }

      if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
      if (f.ACC_MODE) {STABLEPIN_ON;} else {STABLEPIN_OFF;}

      #if BARO
        if (rcOptions[BOXBARO]) {
          if (!f.BARO_MODE) {
            f.BARO_MODE = 1;
            AltHold = EstAlt;
            initialThrottleHold = rcCommand[THROTTLE];
            errorAltitudeI = 0;
            BaroPID=0;
          }
        } else {
          f.BARO_MODE = 0;
        }
      #endif
      #if MAG
        if (rcOptions[BOXMAG]) {
          if (!f.MAG_MODE) {
            f.MAG_MODE = 1;
            magHold = heading;
          }
        } else {
          f.MAG_MODE = 0;
        }
        if (rcOptions[BOXHEADFREE]) {
          if (!f.HEADFREE_MODE) {
            f.HEADFREE_MODE = 1;
          }
        } else {
          f.HEADFREE_MODE = 0;
        }
        if (rcOptions[BOXHEADADJ]) {
          headFreeModeHold = heading; // acquire new heading
        }
      #endif

      #if GPS
        #if defined(I2C_GPS)
        static uint8_t GPSNavReset = 1;
        if (f.GPS_FIX && GPS_numSat >= 5 ) {
          if (!rcOptions[BOXGPSHOME] && !rcOptions[BOXGPSHOLD] )
            {    //Both boxes are unselected
              if (GPSNavReset == 0 ) {
                 GPSNavReset = 1;
                 GPS_I2C_command(I2C_GPS_COMMAND_STOP_NAV,0);
              }
            }
          if (rcOptions[BOXGPSHOME]) {
           if (!f.GPS_HOME_MODE)  {
              f.GPS_HOME_MODE = 1;
              GPSNavReset = 0;
              GPS_I2C_command(I2C_GPS_COMMAND_START_NAV,0);        //waypoint zero
            }
          } else {
            f.GPS_HOME_MODE = 0;
          }
          if (rcOptions[BOXGPSHOLD]) {
            if (!f.GPS_HOLD_MODE & !f.GPS_HOME_MODE) {
              f.GPS_HOLD_MODE = 1;
              GPSNavReset = 0;
              GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD,0);
            }
          } else {
            f.GPS_HOLD_MODE = 0;
          }
        }
        #endif
        #if defined(GPS_SERIAL) || defined(TINY_GPS) || defined(GPS_FROM_OSD)
        if (f.GPS_FIX && GPS_numSat >= 5 ) {
          if (rcOptions[BOXGPSHOME]) {
            if (!f.GPS_HOME_MODE)  {
              f.GPS_HOME_MODE = 1;
              GPS_set_next_wp(&GPS_home[LAT],&GPS_home[LON]);
              nav_mode    = NAV_MODE_WP;
            }
          } else {
            f.GPS_HOME_MODE = 0;
          }
          if (rcOptions[BOXGPSHOLD]) {
            if (!f.GPS_HOLD_MODE) {
              f.GPS_HOLD_MODE = 1;
              GPS_hold[LAT] = GPS_coord[LAT];
              GPS_hold[LON] = GPS_coord[LON];
              GPS_set_next_wp(&GPS_hold[LAT],&GPS_hold[LON]);
              nav_mode = NAV_MODE_POSHOLD;
            }
          } else {
            f.GPS_HOLD_MODE = 0;
          }
        }
        #endif
      #endif

      if (rcOptions[BOXPASSTHRU]) {f.PASSTHRU_MODE = 1;}
      else {f.PASSTHRU_MODE = 0;}

      #ifdef FIXEDWING
        f.HEADFREE_MODE = 0;
      #endif
    } else { // not in rc loop
      static uint8_t taskOrder=0; // never call all functions in the same loop, to avoid high delay spikes
      switch (taskOrder++ % 5) {
        case 0:
          #if MAG
            Mag_getADC();
          #endif
          break;
        case 1:
          #if BARO
            Baro_update();
          #endif
          break;
        case 2:
          #if BARO
            getEstimatedAltitude();
          #endif
          break;
        case 3:
          #if GPS
            if(GPS_Enable) GPS_NewData();
          #endif
          break;
        case 4:
          #if SONAR
            Sonar_update();debug[2] = sonarAlt;
          #endif
          #ifdef LANDING_LIGHTS_DDR
            auto_switch_landing_lights();
          #endif
          break;
      }
    }
   
    computeIMU();
    // Measure loop rate just afer reading the sensors
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

    #if MAG
      if (abs(rcCommand[YAW]) <70 && f.MAG_MODE) {
        int16_t dif = heading - magHold;
        if (dif <= - 180) dif += 360;
        if (dif >= + 180) dif -= 360;
        if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif*conf.P8[PIDMAG]/30;  // 18 deg
      } else magHold = heading;
    #endif

    #if BARO
      if (f.BARO_MODE) {
        if (abs(rcCommand[THROTTLE]-initialThrottleHold)>ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
          f.BARO_MODE = 0; // so that a new althold reference is defined
        }
        rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
      }
    #endif
    #if GPS
      if ( (!f.GPS_HOME_MODE && !f.GPS_HOLD_MODE) || !f.GPS_FIX_HOME ) {
        GPS_reset_nav(); // If GPS is not activated. Reset nav loops and all nav related parameters
      } else {
        float sin_yaw_y = sin(heading*0.0174532925f);
        float cos_yaw_x = cos(heading*0.0174532925f);

     #if defined(NAV_SLEW_RATE)
        nav_rated[LON] += constrain(wrap_18000(nav[LON]-nav_rated[LON]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
        nav_rated[LAT] += constrain(wrap_18000(nav[LAT]-nav_rated[LAT]),-NAV_SLEW_RATE,NAV_SLEW_RATE);
        GPS_angle[ROLL]   = (nav_rated[LON]*cos_yaw_x - nav_rated[LAT]*sin_yaw_y) /10;
        GPS_angle[PITCH]  = (nav_rated[LON]*sin_yaw_y + nav_rated[LAT]*cos_yaw_x) /10;
     #else
        GPS_angle[ROLL]   = (nav[LON]*cos_yaw_x - nav[LAT]*sin_yaw_y) /10;
        GPS_angle[PITCH]  = (nav[LON]*sin_yaw_y + nav[LAT]*cos_yaw_x) /10;
     #endif

      }
    #endif


    //**** PITCH & ROLL & YAW PID ****
    for(axis=0;axis<3;axis++) {
      if (f.ACC_MODE && axis<2 ) { //LEVEL MODE
        // 50 degrees max inclination
        errorAngle = constrain(2*rcCommand[axis] + GPS_angle[axis],-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
        #ifdef LEVEL_PDF
          PTerm      = -(int32_t)angle[axis]*conf.P8[PIDLEVEL]/100 ;
        #else
          PTerm      = (int32_t)errorAngle*conf.P8[PIDLEVEL]/100 ;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
        #endif
        PTerm = constrain(PTerm,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);

        errorAngleI[axis]  = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
        ITerm              = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
      } else { //ACRO MODE or YAW axis
        if (abs(rcCommand[axis])<350) error =          rcCommand[axis]*10*8/conf.P8[axis] ; // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
                                 else error = (int32_t)rcCommand[axis]*10*8/conf.P8[axis] ; // 32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
        error -= gyroData[axis];

        PTerm = rcCommand[axis];

        errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);          // WindUp   16 bits is ok here
        if (abs(gyroData[axis])>640) errorGyroI[axis] = 0;
        ITerm = (errorGyroI[axis]/125*conf.I8[axis])>>6;                                   // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
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
  return 0;
} // main
