#ifndef MULTIWII_H_
#define MULTIWII_H_

#define  VERSION  221

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

#define MINCHECK 1100
#define MAXCHECK 1900

extern const char pidnames[];
extern const char boxnames[];
extern const uint8_t boxids[];

extern uint32_t currentTime;
extern uint16_t previousTime;
extern uint16_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t  magHold,headFreeModeHold;
extern uint8_t  vbatMin;
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern int32_t  BaroAlt,AltHold;
extern int16_t  BaroPID;
extern int16_t  errorAltitudeI;

extern int16_t  i2c_errors_count;
extern uint8_t alarmArray[16];
extern global_conf_t global_conf;

extern imu_t imu;
extern analog_t analog;
extern alt_t alt;
extern att_t att;

extern int16_t debug[4];

extern conf_t conf;

extern int16_t  annex650_overrun_count;
extern flags_struct_t f;
extern uint16_t intPowerTrigger1;

extern int16_t gyroZero[3];
extern int16_t angle[2];


#if BARO
  extern int32_t baroPressure;
  extern int32_t baroTemperature;
  extern int32_t baroPressureSum;
#endif

extern int16_t axisPID[3];
extern int16_t motor[8];
extern int16_t servo[8];

extern int16_t failsafeEvents;
extern volatile int16_t failsafeCnt;

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern int16_t rcCommand[4];
extern uint8_t rcSerialCount;
extern int16_t lookupPitchRollRC[5];
extern int16_t lookupThrottleRC[11];

#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  #define PMOTOR_SUM 8                     // index into pMeter[] for sum
  extern uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  extern uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  extern uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  extern uint16_t powerValue;              // last known current
#endif

#if defined(LCD_TELEMETRY)
  extern uint8_t telemetry;
  extern uint8_t telemetry_auto;
#endif
#ifdef LCD_TELEMETRY_STEP
  extern char telemetryStepSequence[];
  extern uint8_t telemetryStepIndex;
#endif

#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  extern uint16_t cycleTimeMax;       // highest ever cycle timen
  extern uint16_t cycleTimeMin;       // lowest ever cycle timen
  extern int32_t  BAROaltMax;         // maximum value
  extern uint8_t  GPS_speedMax;       // maximum speed from gps
  extern uint16_t powerValueMaxMAH;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  extern uint32_t armedTime;
#endif

void annexCode();

#endif /* MULTIWII_H_ */
