
#include "def.h"

  const char * boxnames PROGMEM = // names for dynamic generation of config GUI
    "ACC;"
    "BARO;"
    "MAG;"
    "CAMSTAB;"
    "CAMTRIG;"
    "ARM;"
    "GPS HOME;"
    "GPS HOLD;"
    "PASSTHRU;"
    "HEADFREE;"
    "BEEPER;"
    "LEDMAX;"
    "LLIGHTS;"
    "HEADADJ;"
  ;

  const char * pidnames PROGMEM =
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

uint8_t SerialAvailable(uint8_t port);//zzz
uint8_t SerialRead(uint8_t port);
//void serialize32(uint32_t a);
//void serialize16(int16_t a);
//void serialize8(uint8_t a);

static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE];

void UartSendData(void){//buffer flush?

}

void SerialOpen(uint8_t port, uint32_t baud){}

uint8_t read8()  {
	return inBuf[indRX++]&0xff;
 // return RxChar(0);
}

void serialize8(uint8_t a) {
 checksum ^=a;
  TxChar(TelemetrySerial,a);
}

uint8_t SerialAvailable(uint8_t port) {
	return serialAvailable(port);
}

uint8_t SerialRead(uint8_t port){

	return RxChar(port);
}


// Multiwii Serial Protocol 0 
#define MSP_VERSION				 0

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}

void headSerialResponse(uint8_t err, uint8_t s) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum);UartSendData();
}

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************


void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serializeNames(const char * pch) {
	while (*pch != (char) 0)
		serialize8(*pch++);
}

void serialCom() {
  uint8_t c;  
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;
  

  while (SerialAvailable(TelemetrySerial)) {

    c = SerialRead(TelemetrySerial);

    if (c_state == IDLE) {
      c_state = (c=='$') ? HEADER_START : IDLE;
      if (c_state == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
    } else if (c_state == HEADER_START) {
      c_state = (c=='M') ? HEADER_M : IDLE;
    } else if (c_state == HEADER_M) {
      c_state = (c=='<') ? HEADER_ARROW : IDLE;
    } else if (c_state == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        c_state = IDLE;
        continue;
      }
      dataSize = c;
      offset = 0;
      checksum = 0;
      indRX = 0;
      checksum ^= c;
      c_state = HEADER_SIZE;  // the command is to follow
    } else if (c_state == HEADER_SIZE) {
      cmdMSP = c;
      checksum ^= c;
      c_state = HEADER_CMD;
    } else if (c_state == HEADER_CMD && offset < dataSize) {
      checksum ^= c;
      inBuf[offset++] = c;
    } else if (c_state == HEADER_CMD && offset >= dataSize) {
      if (checksum == c) {  // compare calculated and transferred checksum
        evaluateCommand();  // we got a valid packet, evaluate it
     }
      c_state = IDLE;
    }
  }
}

void evaluateCommand() {
	uint8_t i;

  switch(cmdMSP) {
   case MSP_SET_RAW_RC:
     for(i=0;i<8;i++) {
       rcData[i] = read16();
     }
     headSerialReply(0);
     break;
#if GPS
   case MSP_SET_RAW_GPS:
     f.GPS_FIX = read8();
     GPS_numSat = read8();
     GPS_coord[LAT] = read32();
     GPS_coord[LON] = read32();
     GPS_altitude = read16();
     GPS_speed = read16();
     GPS_update |= 2;              // New data signalisation to GPS functions
     headSerialReply(0);
     break;
#endif
   case MSP_SET_PID:
     for(i=0;i<PIDITEMS;i++) {
       conf.P8[i]=read8();
       conf.I8[i]=read8();
       conf.D8[i]=read8();
     }
     headSerialReply(0);
     break;
   case MSP_SET_BOX:
     for(i=0;i<CHECKBOXITEMS;i++) {
       conf.activate[i]=read16();
     }
     headSerialReply(0);
     break;
   case MSP_SET_RC_TUNING:
     conf.rcRate8 = read8();
     conf.rcExpo8 = read8();
     conf.rollPitchRate = read8();
     conf.yawRate = read8();
     conf.dynThrPID = read8();
     conf.thrMid8 = read8();
     conf.thrExpo8 = read8();
     headSerialReply(0);
     break;
   case MSP_SET_MISC:
     #if defined(POWERMETER)
       conf.powerTrigger1 = read16() / PLEVELSCALE;
     #endif
     headSerialReply(0);
     break;
   case MSP_IDENT:
     headSerialReply(7);
     serialize8(VERSION);   // multiwii version
     serialize8(MULTITYPE); // type of multicopter
     serialize8(MSP_VERSION);         // MultiWii Serial Protocol Version
     serialize32(0);        // "capability"
     break;
   case MSP_STATUS:
     headSerialReply(10);
     serialize16(cycleTime);
     serialize16(i2c_errors_count);
     serialize16(ACC|BARO<<1|MAG<<2|GPS<<3|SONAR<<4);
     serialize32(f.ACC_MODE<<BOXACC|f.BARO_MODE<<BOXBARO|f.MAG_MODE<<BOXMAG|f.ARMED<<BOXARM|
                 rcOptions[BOXCAMSTAB]<<BOXCAMSTAB | rcOptions[BOXCAMTRIG]<<BOXCAMTRIG |
                 f.GPS_HOME_MODE<<BOXGPSHOME|f.GPS_HOLD_MODE<<BOXGPSHOLD|f.HEADFREE_MODE<<BOXHEADFREE|
                 f.PASSTHRU_MODE<<BOXPASSTHRU|rcOptions[BOXBEEPERON]<<BOXBEEPERON|rcOptions[BOXLEDMAX]<<BOXLEDMAX|rcOptions[BOXLLIGHTS]<<BOXLLIGHTS|rcOptions[BOXHEADADJ]<<BOXHEADADJ);
     break;
   case MSP_RAW_IMU:
     headSerialReply(18);
     for(i=0;i<3;i++) serialize16(accSmooth[i]);
     for(i=0;i<3;i++) serialize16(gyroData[i]);
     for(i=0;i<3;i++) serialize16(magADC[i]);
     break;
   case MSP_SERVO:
     headSerialReply(16);
     for(i=0;i<8;i++)
       #if defined(SERVO)
       serialize16(servo[i]);
       #else
       serialize16(0);
       #endif
     break;
   case MSP_MOTOR:
     headSerialReply(16);
     for(i=0;i<8;i++) {
       serialize16( (i < NUMBER_MOTOR) ? motor[i] : 0 );
     }
     break;
   case MSP_RC:
     headSerialReply(16);
     for(i=0;i<8;i++) serialize16(rcData[i]);
     break;
#if GPS
   case MSP_RAW_GPS:
     headSerialReply(14);
     serialize8(f.GPS_FIX);
     serialize8(GPS_numSat);
     serialize32(GPS_coord[LAT]);
     serialize32(GPS_coord[LON]);
     serialize16(GPS_altitude);
     serialize16(GPS_speed);
     break;
   case MSP_COMP_GPS:
     headSerialReply(5);
     serialize16(GPS_distanceToHome);
     serialize16(GPS_directionToHome);
     serialize8(GPS_update & 1);
     break;
#endif
   case MSP_ATTITUDE:
     headSerialReply(8);
     for(i=0;i<2;i++) serialize16(angle[i]);
     serialize16(heading);
     serialize16(headFreeModeHold);
     break;
   case MSP_ALTITUDE:
     headSerialReply(4);
     serialize32(EstAlt);
     break;
   case MSP_BAT:
     headSerialReply(3);
     serialize8(vbat);
     serialize16(intPowerMeterSum);
     break;
   case MSP_RC_TUNING:
     headSerialReply(7);
     serialize8(conf.rcRate8);
     serialize8(conf.rcExpo8);
     serialize8(conf.rollPitchRate);
     serialize8(conf.yawRate);
     serialize8(conf.dynThrPID);
     serialize8(conf.thrMid8);
     serialize8(conf.thrExpo8);
     break;
   case MSP_PID:
     headSerialReply(3*PIDITEMS);
     for(i=0;i<PIDITEMS;i++) {
       serialize8(conf.P8[i]);
       serialize8(conf.I8[i]);
       serialize8(conf.D8[i]);
     }
     break;
   case MSP_BOX:
     headSerialReply(2*CHECKBOXITEMS);
     for(i=0;i<CHECKBOXITEMS;i++) {
       serialize16(conf.activate[i]);
     }
     break;
   case MSP_BOXNAMES:
	   headSerialReply(sizeof(boxnames)-1);
	   serializeNames(boxnames);
     break;
   case MSP_PIDNAMES:
	   headSerialReply(sizeof(pidnames)-1);
   	   serializeNames(pidnames);
     break;
   case MSP_MISC:
     headSerialReply(2);
     serialize16(intPowerTrigger1);
     break;
   case MSP_MOTOR_PINS:
     headSerialReply(8);
     for(i=0;i<8;i++) {
    	 serialize8(motor[i]>>4);
     }
     break;

#if defined(USE_MSP_WP)    
   case MSP_WP:
     {
      uint8_t wp_no = read8();    //get the wp number  
      headSerialReply(12);
      if (wp_no == 0) {
        serialize8(0);                   //wp0
        serialize32(GPS_home[LAT]);
        serialize32(GPS_home[LON]);
        serialize16(0);                  //altitude will come here 
        serialize8(0);                   //nav flag will come here
      } else if (wp_no == 16)
      {
        serialize8(16);                  //wp16
        serialize32(GPS_hold[LAT]);
        serialize32(GPS_hold[LON]);
        serialize16(0);                  //altitude will come here 
        serialize8(0);                   //nav flag will come here
      } 
     }
     break;  
#endif	 
	 
   case MSP_RESET_CONF:
     conf.checkNewConf++;
     checkFirstTime();
     headSerialReply(0);
     break;
   case MSP_ACC_CALIBRATION:
     calibratingA=400;
     headSerialReply(0);
     break;
   case MSP_MAG_CALIBRATION:
     f.CALIBRATE_MAG = 1;
     headSerialReply(0);
     break;
   case MSP_EEPROM_WRITE:
     writeParams(0);
     headSerialReply(0);
     break;
   case MSP_DEBUG:
     headSerialReply(8);
     for(i=0;i<4;i++) {
       serialize16(debug[i]); // 4 variables are here for general monitoring purpose
     }
     break;
   default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
     headSerialError(0);
     break;
  }
  tailSerialReply();
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  switch (sr) {
  // Note: we may receive weird characters here which could trigger unwanted features during flight.
  //       this could lead to a crash easily.
  //       Please use if (!f.ARMED) where neccessary
    #ifdef LCD_CONF
    case 's':
    case 'S':
      if (!f.ARMED) configurationLoop();
      break;
    #endif
    #ifdef LCD_TELEMETRY
    case 'A': // button A press
      toggle_telemetry(1);
      break;
    case 'B': // button B press
      toggle_telemetry(2);
      break;
    case 'C': // button C press
      toggle_telemetry(3);
      break;
    case 'D': // button D press
      toggle_telemetry(4);
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    #if defined(LOG_VALUES) && defined(DEBUG)
    case 'R':
    #endif
    #ifdef DEBUG
    case 'F':
    #endif
      toggle_telemetry(sr);
      break;
    case 'a': // button A release
    case 'b': // button B release
    case 'c': // button C release
    case 'd': // button D release
      break;
    #endif // LCD_TELEMETRY
  }
}

// *******************************************************
// For Teensy 2.0, these function emulate the API used for ProMicro
// it cant have the same name as in the arduino API because it wont compile for the promini (eaven if it will be not compiled)
// *******************************************************
#if defined(TEENSY20)
  unsigned char T_USB_Available(){
    int n = Serial.available();
    if (n > 255) n = 255;
    return n;
  }
#endif





