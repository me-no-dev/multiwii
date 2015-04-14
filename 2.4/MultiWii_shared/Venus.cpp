#include "Arduino.h"
#include "Venus.h"
#include "def.h"
#include "types.h"
#include "stdint.h"

#include "Serial.h"

#ifdef VENUS

#define VENUS_SERIAL_OPEN(x) SerialOpen(GPS_SERIAL, x)
#define VENUS_SERIAL_WRITE(x) SerialWrite(GPS_SERIAL, x)
#define VENUS_SERIAL_AVAILABLE() SerialAvailable(GPS_SERIAL)
#define VENUS_SERIAL_READ() SerialRead(GPS_SERIAL)


venus_message venus_ctx;

#ifdef VENUS_DEBUG
void VenusDebugWriteMsg(const char* desc, bool crlf=true)
{
  Serial.print("$GPREM,");
  Serial.print(desc);
  Serial.print(",L");
  Serial.print(venus_ctx.length);
  Serial.print(',');
  Serial.print((short)venus_ctx.id);
  for(char i=0; i<venus_ctx.length; i++) {
      Serial.print(',');
      Serial.print((short)venus_ctx.body[i]);
  }
  if(crlf)
    Serial.println();  
}
#else
#define VenusDebugWriteMsg(desc) {}
#endif

void VenusWriteImmediate()
{
  int pls=0;
  byte cs=venus_ctx.id;
  while(pls<venus_ctx.length)
    cs = cs ^ venus_ctx.body[pls++];
  pls++;
  VENUS_SERIAL_WRITE(0xA0);
  VENUS_SERIAL_WRITE(0xA1);
  VENUS_SERIAL_WRITE((pls>>8)&0xff);
  VENUS_SERIAL_WRITE(pls&0xff);
  VENUS_SERIAL_WRITE(venus_ctx.id);
  for(pls=0; pls<venus_ctx.length; pls++)
	  VENUS_SERIAL_WRITE(venus_ctx.body[pls]);
  //Serial.write(body, length);
  VENUS_SERIAL_WRITE(cs);
  VENUS_SERIAL_WRITE(0x0D);
  VENUS_SERIAL_WRITE(0x0A);
  VenusDebugWriteMsg("OUT");
}

#if 1
void VenusWriteNull()
{
  venus_ctx.id=VENUS_NACK;
  venus_ctx.length=1;
  venus_ctx.body[0]=0;
  VenusWriteImmediate(); 
}
#endif

//inline void VenusWriteImmediate(byte msgid, byte a) { venus_ctx.length=1; venus_ctx.id=msgid; venus_ctx.body[0]=a; VenusWriteImmediate(); }
//inline void VenusWriteImmediate(byte msgid, byte a, byte b) { venus_ctx.length=1; venus_ctx.id=msgid; venus_ctx.body[0]=a; venus_ctx.body[1]=b; VenusWriteImmediate(); }



#if 0
#define SWAP16(x) ((x&0xff)<<8 | (x>>8))
#define SWAP32(x) ((x&0xff)<<24 | ((x&0xff00)<<8) | ((x&0xff0000)>>8) | ((x&0xff000000)>>24))
#else
uint16_t SWAP16(uint16_t x) { return ((x&0xff)<<8 | (x>>8)); }
uint32_t SWAP32(uint32_t x) { return ((x&0xff)<<24 | ((x&0xff00)<<8) | ((x&0xff0000)>>8) | ((x&0xff000000)>>24)); }
#endif

void VenusFixLocationEndianess()
{
  venus_ctx.location.gps_week=SWAP16(venus_ctx.location.gps_week);
  venus_ctx.location.gps_tow = SWAP32(venus_ctx.location.gps_tow);
  venus_ctx.location.latitude = SWAP32(venus_ctx.location.latitude);
  venus_ctx.location.longitude = SWAP32(venus_ctx.location.longitude);
  venus_ctx.location.ellipsoid_alt = SWAP32(venus_ctx.location.ellipsoid_alt);
  venus_ctx.location.sealevel_alt = SWAP32(venus_ctx.location.sealevel_alt);
  venus_ctx.location.gdop = SWAP16(venus_ctx.location.gdop);
  venus_ctx.location.pdop = SWAP16(venus_ctx.location.pdop);
  venus_ctx.location.hdop = SWAP16(venus_ctx.location.hdop);
  venus_ctx.location.vdop = SWAP16(venus_ctx.location.vdop);
  venus_ctx.location.tdop = SWAP16(venus_ctx.location.tdop);
  venus_ctx.location.ecef.x = SWAP32(venus_ctx.location.ecef.x);
  venus_ctx.location.ecef.y = SWAP32(venus_ctx.location.ecef.y);
  venus_ctx.location.ecef.z = SWAP32(venus_ctx.location.ecef.z);
  venus_ctx.location.vel.x = SWAP32(venus_ctx.location.vel.x);
  venus_ctx.location.vel.y = SWAP32(venus_ctx.location.vel.y);
  venus_ctx.location.vel.z = SWAP32(venus_ctx.location.vel.z);
}

short VenusProcessInput(int c)
{
  static byte state=0;
  static char n=0;
  static int cr=0;
  
  switch(state) {
    case 0: if(c==0xA0) state++; break;
    case 1: if(c==0xA1) state++; else state=0; break;
    case 2: venus_ctx.length=c<<8; state++; break;  // read payload length (2 bytes)
    case 3: venus_ctx.length|=c; state++; break;
    case 4: 
      venus_ctx.id=cr=c; n=0; 
      if(--venus_ctx.length>0) state++; else state=6; // if no payload then skip next state
      break;
    case 5: // read bytes of the payload
      if(n<VENUS_MAX_PAYLOAD)
        venus_ctx.body[n]=c;
      n++;
      cr ^= c;  // adjust checksum
      if(n>=venus_ctx.length) state++;
      break;
    case 6: 
      if(c==cr) 
        state++; 
      else {
        state=0; 
        return VENUS_BADCR; 
      } break; // check checksum, abort if not-equal
    case 7: if(c==0x0d) state++; break;
    case 8: 
      state=0;
      if(venus_ctx.id==VENUS_GPS_LOCATION)
        VenusFixLocationEndianess();
      return (c==0x0A) ? ((n<=VENUS_MAX_PAYLOAD)?VENUS_OK:VENUS_CLIPPED): VENUS_INCOMPLETE; 
    default: 
      state=0; 
      return VENUS_INCOMPLETE;
  }
  return VENUS_MORE;
}

// reads the next binary message
short VenusRead(int timeout)
{
  short result;
  // read with timeout or infinite
  unsigned long long stopat = millis() + timeout;
  while(timeout==0 || millis()<stopat) {
    if (VENUS_SERIAL_AVAILABLE() && ((result=VenusProcessInput(VENUS_SERIAL_READ()))==VENUS_OK || result==VENUS_CLIPPED)) {
        // a complete message was received
#ifdef VENUS_DEBUG
        VenusDebugWriteMsg("IN",false);
        switch(result) {
          case VENUS_OK:          Serial.println(",OK"); break;
          case VENUS_TIMEOUT:     Serial.println(",TO"); break;
          case VENUS_MORE:        Serial.println(",MORE"); break;
          case VENUS_NACKED:      Serial.println(",NACKED"); break;
          case VENUS_ACK_NOREPLY: Serial.println(",NOREPLY"); break;
          case VENUS_BADCR:       Serial.println(",BADCR"); break;
          case VENUS_INCOMPLETE:  Serial.println(",INC"); break;
          case VENUS_CLIPPED:     Serial.println(",CLIP"); break;
          default:
            Serial.print(",R");
            Serial.println(result);
        }
#endif
        return result;
    }
  }
#ifdef VENUS_DEBUG
  Serial.println("$GPREM,IN,TIMEOUT");
#endif
  return VENUS_TIMEOUT;
}

#if 0
short VenusAsyncRead()
{
  short result;
  if (VENUS_SERIAL_AVAILABLE() && (result=VenusProcessInput(VENUS_SERIAL_READ()))==VENUS_OK || result==VENUS_CLIPPED)
        // a complete message was received
        return result;
  return VENUS_MORE;
}
#endif

short VenusWrite(int timeout)
{
  byte msgid = venus_ctx.id;
  unsigned long long stopat = millis() + timeout;
  VenusWriteImmediate();      // write msg
  while(millis() < stopat) {    // now wait for ack
    VenusRead(stopat - millis());
    if(venus_ctx.id==VENUS_ACK && venus_ctx.body[0]==msgid)
      return VENUS_OK;
    else if(venus_ctx.id==VENUS_NACK && venus_ctx.body[0]==msgid)
      return VENUS_NACKED;
    //else
    //  VenusDispatchMessage();  // dispatch this message through the normal channels
  }
  return VENUS_TIMEOUT;
}

/* conflicts with VenusWrite(timeout)
bool VenusWrite(byte queryid)
{
    venus_ctx.id = queryid;
    venus_ctx.length = 0;
    return VenusWrite(1000);
}*/

// return the response msgid for a given query msgid
byte VenusWhatIsResponseMsgIdOf(byte msgid)
{
  if(msgid>0x60 && msgid<0x6f)
    return msgid;
  switch(msgid) {
         case VENUS_QUERY_SW_VERSION: return VENUS_REPORT_SW_VERSION; 
    case VENUS_QUERY_GPS_UPDATE_RATE: return VENUS_REPORT_GPS_UPDATE_RATE; 
            case VENUS_GET_EPHEMERIS: return VENUS_REPORT_EPHEMERIS; 
              case VENUS_QUERY_DATUM: return VENUS_GPS_DATUM; 
               case VENUS_QUERY_WAAS: return VENUS_GPS_WAAS_STATUS; 
        case VENUS_QUERY_GPS_PINNING: return VENUS_GPS_PINNING_STATUS; 
           case VENUS_QUERY_NAV_MODE: return VENUS_GPS_NAV_MODE; 
          case VENUS_QUERY_1PPS_MODE: return VENUS_GPS_1PPS_MODE; 
         case VENUS_QUERY_POWER_MODE: return VENUS_GPS_POWER_MODE;
          default: return 0;
  }
}

byte Venus8WhatIsResponseMsgIdOf(byte msgid)
{
  switch(msgid) {
             case VENUS8_QUERY_NAV_MODE: return VENUS8_REPORT_NAV_MODE;
   case VENUS8_QUERY_CONSTELLATION_TYPE: return VENUS8_REPORT_CONSTELLATION_TYPE;
                 case VENUS8_QUERY_SAEE: return VENUS8_REPORT_SAEE;
    default: return 0;
  }
}

// send a query to the GPS module and expect/waitfor a reply
short VenusQuery(int timeout)
{
  int orig_timeout = timeout;
  byte msgid=venus_ctx.id, reply_msgid=VenusWhatIsResponseMsgIdOf(venus_ctx.id);
  bool isExtMsg = msgid>0x60 && msgid<0x6f;
  
  // venus8 extension has two id fields
  byte msgid8 =0, reply_msgid8 =0;
  if(isExtMsg) {
   msgid8 = venus_ctx.body[0];
   reply_msgid8 = Venus8WhatIsResponseMsgIdOf(msgid8);
  }
  
  unsigned long long stopat = millis() + timeout;
  bool acked=false;
  short result;
  VenusWriteImmediate();      // write msg
  while(millis() < stopat) {    // now wait for ack
    result=VenusRead(stopat - millis());
    if(result == VENUS_TIMEOUT)
      goto timedout;
    else if(venus_ctx.id==VENUS_ACK && venus_ctx.body[0]==msgid) {
      if(!isExtMsg || venus_ctx.body[1] == msgid8) {
        acked=true;  // got the ack, now the response
        timeout = orig_timeout;
        //Serial.println("$GPSREM,ACK");
      }
    } else if(venus_ctx.id==VENUS_NACK && venus_ctx.body[0]==msgid) {
      if(!isExtMsg || venus_ctx.body[1] == msgid8) {
        //Serial.println("$GPSREM,NACK");
        return VENUS_NACKED;
      }
    } else if(venus_ctx.id==reply_msgid) {
      if(!isExtMsg || venus_ctx.body[0] == reply_msgid8) {
        //Serial.println("$GPSREM,SUCCESS");
        return result;
      }
    } //else
      ///VenusDispatchMessage();  // dispatch this message through the normal channels
  }
timedout:
  return acked ? VENUS_ACK_NOREPLY : VENUS_TIMEOUT;
}

byte VenusGetOption(byte msgid)
{
  venus_ctx.length = 0;
  venus_ctx.id = msgid;
  return (VenusQuery(VENUS_DEFAULT_TIMEOUT)==VENUS_OK)
    ? venus_ctx.body[0]
    : 0;
}

short VenusSetOption(byte option, byte value, bool flash)
{
    venus_ctx.id = option;
    venus_ctx.length = 2;
    venus_ctx.body[0] = value;
    venus_ctx.body[1] = flash?VENUS_FLASH:VENUS_SRAM;
    short result=VenusWrite(VENUS_DEFAULT_TIMEOUT);
    if(result!=VENUS_TIMEOUT && flash)
      delay(VENUS_DELAY_AFTER_FLASH);  // slight delay for writing to flash
    return result;
}

byte Venus8GetExtendedOption(byte ext_number, byte msgid)
{
  venus_ctx.length = 1;
  venus_ctx.id = ext_number;
  venus_ctx.body[0] = msgid;
#ifndef VENUS_DEBUG
  return (VenusQuery(VENUS_DEFAULT_TIMEOUT)==VENUS_OK)
    ? venus_ctx.body[1]
    : 0;
#else
  short result=VenusQuery(VENUS_DEFAULT_TIMEOUT);
  Serial.print("$GPREM,GET,M");
  Serial.print((short)msgid);
  Serial.print(",R");
  Serial.print(result);
  Serial.print(',');
  Serial.print(venus_ctx.body[0]);
  Serial.print(',');
  Serial.println(venus_ctx.body[1]);
  return (result==VENUS_OK)
    ? venus_ctx.body[1]
    : 0;
 #endif
}

short Venus8SetExtendedOption(byte ext_number, byte option, byte value, bool flash)
{
    venus_ctx.id = ext_number;
    venus_ctx.length = 3;
    venus_ctx.body[0] = option;
    venus_ctx.body[1] = value;
    venus_ctx.body[2] = flash?VENUS_FLASH:VENUS_SRAM;
    short result=VenusWrite(VENUS_DEFAULT_TIMEOUT);
    if(result!=VENUS_TIMEOUT && flash)
      delay(VENUS_DELAY_AFTER_FLASH);  // slight delay for writing to flash
    return result;
}
  
uint32_t VenusGetBaudRate(byte n)
{
  switch(n) {
    case -1: return GPS_BAUD;  // the default speed
    case 0: return 4800;
    case 1: return 9600;
//    case 2: return 19200;  // dont bother using these
//    case 3: return 28800;
    case 4: return 57600;
    case 5: return 115200;
    default: return 0;
  }
}

byte VenusGetBaudRateOrdinal(uint32_t baud)
{
  switch(baud) {
    case 4800: return 0;
    case 9600: return 1;
    case 19200: return 2;
    case 28800: return 3;
    case 57600: return 4;
    default: return 5;
  }
}

char VenusSetBaudRate(uint32_t baudrate, bool flash)
{
  char result;
  venus_ctx.id = VENUS_CONFIG_SERIAL_PORT;
  venus_ctx.length = 3;
  venus_ctx.body[0] = 0;  // Venus device's COM1
  venus_ctx.body[1] = VenusGetBaudRateOrdinal(baudrate);
  venus_ctx.body[2] = flash?VENUS_FLASH:VENUS_SRAM;
  if((result=VenusWrite(VENUS_DEFAULT_TIMEOUT)) == VENUS_OK) {
    VENUS_SERIAL_OPEN(baudrate);
    
    // wait until we can communicate
    short attempts = 5;
    short wait = 100;
    venus_ctx.length = 1;
    venus_ctx.id = VENUS_QUERY_SW_VERSION;
    venus_ctx.body[0] = 1;  // system code
    while(attempts-- >0 && VenusQuery(VENUS_DEFAULT_TIMEOUT)!=VENUS_OK) {
      delay(wait);
      wait <<=1;  // multiply by 2 so as to progressively delay longer each fail
    }
    
    return (attempts>0) ? VENUS_OK : VENUS_ACK_NOREPLY;
  } else {
    return result;
  }
}

#ifdef VenusPowerPin
void VenusPowerCycle()
{
  // enable the 3V3B regulator thus enabling the GPS
  pinMode(VenusPowerPin, OUTPUT); 
  digitalWrite(VenusPowerPin, 0); 
  delay(50); 
  digitalWrite(VenusPowerPin, 1);  
}
#else
#define VenusPowerCycle() {}  // no GPS_PWR_EN connected
#endif

void short_beep()
{
  digitalWrite(32, 1); 
  delay(75); 
  digitalWrite(32, 0);  
  delay(300); 
}

void long_beep()
{
  digitalWrite(32, 1); 
  delay(200); 
  digitalWrite(32, 0);  
  delay(300); 
}

void VenusError(short code)
{
  if(code==0) 
    return;
#ifdef VENUS_DEBUG
  Serial.print("$GPERR,");
  Serial.println(code);
#endif

  long_beep();
  long_beep();
  long_beep();
  while(code>=1)
  {
    code >>=1; // divide by 2
    short_beep();
  }
}

bool VenusScan(unsigned long desiredBaud, byte desiredMessageFormat)
{
  // detect the baud rate
  char result;
  uint32_t baud;
  int attempts=2;
  char i = VenusGetBaudRateOrdinal(desiredBaud);
  
  while(attempts >0) {
    short_beep();
    
    // try to communicate at this baud rate
    baud = VenusGetBaudRate(i);
    if(baud>0) {
#ifdef VENUS_DEBUG
      Serial.print("$BAUD,");
      Serial.println(baud);
#endif
      
      VENUS_SERIAL_OPEN(baud);
      //VenusWriteNull();
      
      venus_ctx.id=VENUS_QUERY_GPS_UPDATE_RATE;
      venus_ctx.length=0;
      if((result=VenusQuery(VENUS_DEFAULT_TIMEOUT)) == VENUS_OK) {
        VenusSetOutput(VENUS_OUTPUT_NONE, false);
  
        if(baud != desiredBaud) {
          if(VenusSetBaudRate(desiredBaud, true)!=VENUS_OK)
            VenusError(STARTUP_BAUDRATE);
        }
  
        if(VenusSetOutput(desiredMessageFormat, false)!=VENUS_OK)
          VenusError(STARTUP_MSGMODE);
  
        return true;  // successful connect
      }
    }
    
    // try next baud level
    if(--i <0) {
        i = VenusGetBaudRateOrdinal(VENUS_MAX_BAUDRATE);
        attempts--;
    }

    VenusPowerCycle();
    delay(4000);  // wait for GPS to recover
  }
  VenusError(STARTUP_NORESPONSE);
  return false;
}

bool GPSModuleInit()
{
  VenusPowerCycle();
  return VenusScan(GPS_BAUD, VENUS_OUTPUT_MODE);
}

bool GPSConfigureDefaults()
{
  // query options state
  short result;
  unsigned char values[5];
  values[0] = VenusQueryNavMode();
  values[1] = VenusQueryPositionPinning();
  values[2] = VenusQuerySAEE();
  values[3] = VenusQueryPowerMode();
  values[4] = VenusQueryUpdateRate();
  
#if VENUS_DEBUG
  Serial.print("$GPOPT");  // not standard NMEA but it works and spits out in GPS Viewer
  for(char i=0; i<sizeof(values); i++) {
      Serial.print(',');
      Serial.print((short)values[i]);
  }
  Serial.println();
#endif

#if 1
  // configure NAV MODE
  if(values[0]!=VENUS8_NAVMODE_AIRBORNE && (result=VenusSetNavMode(VENUS8_NAVMODE_AIRBORNE, VENUS_FLASH))!=VENUS_OK)
    VenusError(STARTUP_NAVMODE);
  
  // disable POSITION PINNING
  if(values[1]!=VENUS_POSPINNING_DISABLE && (result=VenusSetPositionPinning(VENUS_POSPINNING_DISABLE, VENUS_FLASH))!=VENUS_OK)
    VenusError(STARTUP_POSPINNING);

  // disable SAEE
  if(values[2]!=VENUS8_SAEE_DISABLE && (result=VenusSetSAEE(VENUS8_SAEE_DISABLE, VENUS_FLASH))!=VENUS_OK)
    VenusError(STARTUP_NOSAEE);

  // NORMAL power mode
  if(values[3]!=VENUS_POWERMODE_NORMAL && (result=VenusSetPowerMode(VENUS_POWERMODE_NORMAL, VENUS_FLASH))!=VENUS_OK)
    VenusError(STARTUP_POWERMODE);

  // enable the update rate
  if(values[4]!=VENUS_UPDATE_RATE && VenusSetUpdateRate(VENUS_UPDATE_RATE, true)!=VENUS_OK)
    VenusError(STARTUP_UPDATERATE);
#endif
  
#ifdef VENUS_DEBUG
  Serial.println("$GPREM,SUCCESS");
#endif
}

#endif  // ifdef VENUS

