#ifndef __VENUS6_H
#define __VENUS6_H

#include "config.h"

#if defined(VENUS)
#include <stdint.h>

typedef unsigned char byte;

//#define VenusSerial Serial2

// enable for Venus8 chipsets, otherwise defaults to Venus6
#define VENUS8

// specify the GPS 3v regulators EN pin that is connected to the Atmel (or undefine if not connected)
#define VenusPowerPin  36

// the default update rate
#ifndef VENUS_UPDATE_RATE
#define VENUS_UPDATE_RATE  20
#endif

// enable to get a lot of output in NMEA format (uses GPREM for remark)
//#define VENUS_DEBUG

// the amount of time to wait for a response from the VenusGPS chipset. This is only
// relevant on startup during configuration and no sync waits are performed during normal
// gps loop.
#define VENUS_DEFAULT_TIMEOUT    3000
#define VENUS_FAST_TIMEOUT    500			// when checking baud rate and certain actions expected to return fast

// the maximum number of bytes that can be received in one message (size of the receive buffer)
#define VENUS_MAX_PAYLOAD        60

// maximum and minimum baudrate
#define VENUS_MAX_BAUDRATE       115200 // warning: 115200 has 2% error on atmel, ideal is 57600 @ 0.8%
#define VENUS_MIN_BAUDRATE       9600

// after writing a setting to flash we must wait a small delay (in millis)
// before the GPS to recovers.
#define VENUS_DELAY_AFTER_FLASH  50


/* Venus6 message codes */
        // Input System Messages
#define VENUS_SYS_RESTART                 0x01
#define VENUS_QUERY_SW_VERSION            0x02
#define VENUS_QUERY_SW_CRC                0x03
#define VENUS_SET_FACTORY_DEFAULTS        0x04
#define VENUS_CONFIG_SERIAL_PORT          0x05
#define VENUS_CONFIG_NMEA                 0x08
#define VENUS_CONFIG_OUTPUT_MSG_FORMAT    0x09
#define VENUS_CONFIG_POWER_MODE           0x0C
#define VENUS_CONFIG_GPS_UPDATE_RATE      0x0E
#define VENUS_QUERY_GPS_UPDATE_RATE       0x10
#define VENUS_QUERY_POWER_MODE            0x15
        // Input GPS Messages
#define VENUS_CONFIG_DATUM                0x29
#define VENUS_QUERY_DATUM                 0x2D
#define VENUS_GET_EPHEMERIS               0x30
#define VENUS_SET_EPHEMERIS               0x31
#define VENUS_CONFIG_WAAS                 0x37
#define VENUS_QUERY_WAAS                  0x38
#define VENUS_CONFIG_GPS_PINNING          0x39
#define VENUS_QUERY_GPS_PINNING           0x3A
#define VENUS_CONFIG_GPS_PINNING_PARAMS   0x3B
#define VENUS_CONFIG_NAV_MODE             0x3C
#define VENUS_QUERY_NAV_MODE              0x3D
#define VENUS_CONFIG_1PPS_MODE            0x3E
#define VENUS_QUERY_1PPS_MODE             0x3F
        // Output System Messages
#define VENUS_REPORT_SW_VERSION           0x80
#define VENUS_REPORT_SW_CRC               0x81
#define VENUS_REPORT_GPS_UPDATE_RATE      0x86
#define VENUS_REPORT_EPHEMERIS            0xB1
        // Output GPS Messages
#define VENUS_GPS_LOCATION                0xA8
#define VENUS_GPS_DATUM                   0xAE
#define VENUS_GPS_WAAS_STATUS             0xB3
#define VENUS_GPS_PINNING_STATUS          0xB4
#define VENUS_GPS_NAV_MODE                0xB5
#define VENUS_GPS_1PPS_MODE               0xB6
#define VENUS_GPS_POWER_MODE              0xB9

        // Request Acknowledge
#define VENUS_ACK                         0x83
#define VENUS_NACK                        0x84

#ifdef VENUS8
  // command messages
#define VENUS8_EXT2                       0x62
#define VENUS8_EXT3                       0x63
#define VENUS8_EXT4                       0x64
#define VENUS8_CONFIG_NAV_MODE            0x17  // w/EXT4
#define VENUS8_QUERY_NAV_MODE             0x18  // w/EXT4
#define VENUS8_CONFIG_CONSTELLATION_TYPE  0x19  // w/EXT4
#define VENUS8_QUERY_CONSTELLATION_TYPE   0x1a  // w/EXT4
#define VENUS8_CONFIG_SAEE                0x01  // w/EXT3
#define VENUS8_QUERY_SAEE                 0x02  // w/EXT3
  // Response codes
#define VENUS8_REPORT_NAV_MODE            0x8b  // w/EXT4
#define VENUS8_REPORT_CONSTELLATION_TYPE  0x8c  // w/EXT4
#define VENUS8_REPORT_SAEE                0x80  // w/EXT3
#endif


// Read/write errors (will be char)
#define VENUS_OK           0  // read successful
#define VENUS_TIMEOUT     -1  // no data received within timeout period
#define VENUS_MORE        -2  // more data remaining
#define VENUS_NACKED      -3  // message reply was a nack
#define VENUS_ACK_NOREPLY -4  // message was incomplete
#define VENUS_BADCR       -5  // checksum failure
#define VENUS_INCOMPLETE  -6  // message was incomplete
#define VENUS_CLIPPED     -7  // message buffer too small, response payload was clipped

// Startup error codes
#define STARTUP_NORESPONSE         1
#define STARTUP_BAUDRATE           2
#define STARTUP_MSGMODE            4
#define STARTUP_UPDATERATE         8
#define STARTUP_QUERYOPTIONS      16
#define STARTUP_NAVMODE           32
#define STARTUP_POSPINNING        64
#define STARTUP_POWERMODE        128
#define STARTUP_NOSAEE           256

// NAVIGATION MODES
#define VENUS_NAVMODE_AUTO                0    // defaults to car
#define VENUS_NAVMODE_PEDESTRIAN          1
#ifdef VENUS8
#define VENUS8_NAVMODE_CAR                 2
#define VENUS8_NAVMODE_MARINE              3
#define VENUS8_NAVMODE_BALLON              4    // is this balloon?
#define VENUS8_NAVMODE_AIRBORNE            5
#define VENUS8_NAVMODE_SURVEY_AND_MAPPING  6
#endif

// POSITION PINNING
#ifdef VENUS8
#define VENUS_POSPINNING_AUTO     0
#define VENUS_POSPINNING_ENABLE   1
#define VENUS_POSPINNING_DISABLE  2
#else
#define VENUS_POSPINNING_AUTO     0
#define VENUS_POSPINNING_ENABLE   1
#define VENUS_POSPINNING_DISABLE  0
#endif

// POWER MODES
#define VENUS_POWERMODE_NORMAL    0
#define VENUS_POWERMODE_POWERSAVE 1

// SAEE MODES
#define VENUS8_SAEE_DEFAULT       0
#define VENUS8_SAEE_ENABLE        1
#define VENUS8_SAEE_DISABLE       2

// typical SRAM or FLASH attribute
#define VENUS_SRAM            0
#define VENUS_FLASH           1

// MESSAGE OUTPUT MODES
#define VENUS_OUTPUT_NONE        0
#define VENUS_OUTPUT_NMEA        1
#define VENUS_OUTPUT_BINARY      2

#if !defined(VENUS_OUTPUT_MODE)
#if defined(NMEA)
#define VENUS_OUTPUT_MODE VENUS_OUTPUT_NMEA
#else
#define VENUS_OUTPUT_MODE VENUS_OUTPUT_BINARY
#endif
#endif

#define VENUS_INT16(h,l)   ((h<<8)|l)

typedef struct {
    int32_t x;
  int32_t y;
  int32_t z;
} xyz32_t;

typedef struct {
  uint8_t  fixmode;
  uint8_t  sv_count;  // satellites
  uint16_t gps_week;
  uint32_t gps_tow;
  int32_t latitude;
  int32_t longitude;
  uint32_t ellipsoid_alt;
  uint32_t sealevel_alt;
  uint16_t gdop, pdop, hdop, vdop, tdop;
  xyz32_t ecef;
  xyz32_t vel;
} venus_location;

typedef struct _venus_message {
  unsigned char length;       // payload length;
  byte id;    // message id
  union {
    byte body[VENUS_MAX_PAYLOAD];
    venus_location location;
  };
} venus_message;

extern venus_message venus_ctx;

// initializes the GPS module and sets up defaults for the best airborne performance
// this also scans for the GPS baud rate. You really only need to call this function.
bool GPSModuleInit();

// seeks out the GPS chipset by scanning baudrates, will also setup the
// desired update rate and message format. (called by GPSModuleInit)
bool VenusScan(unsigned long desiredBaud=57600, byte desiredMessageFormat=VENUS_OUTPUT_BINARY);

// process a byte of input, the return value indicates if venus_message struct holds
// a valid message.
short VenusProcessInput(int c);

// configure the default GPS settings (as per config defines)
// already done as part of the GPSModuleInit()
bool GPSConfigureDefaults();

// get the value of a single-byte VENUS_QUERY_xxxx option
byte VenusGetOption(byte msgid);

// set the single-byte value of a VENUS_CONFIG_xxxx option
short VenusSetOption(byte option, byte value, bool flash);

#ifdef VENUS8
// get the value of a single-byte VENUS8_QUERY_xxxxx option (extended option)
byte Venus8GetExtendedOption(byte ext_number, byte msgid);

// set the single-byte value of a VENUS8_CONFIG_xxxx option (extended option)
// these extended messages are under accessed under 1 of 3 extensions (62,63,64) though they may later add (60-6f)
short Venus8SetExtendedOption(byte ext_number, byte option, byte value, bool flash);
#endif

/* Convenience functions 
 *
 * These functions are inline and simply call the Venus(Get/Set)Option or Venus8(Get/Set)ExtendedOption.
 *
 */
 
// Set how often the location should be updated/sent from the GPS receiver
inline byte VenusQueryUpdateRate() { return VenusGetOption(VENUS_QUERY_GPS_UPDATE_RATE); }
inline char VenusSetUpdateRate(byte updaterate, bool flash) { return VenusSetOption(VENUS_CONFIG_GPS_UPDATE_RATE, updaterate, flash); }

// Set the output of GPS location message to Binary or NMEA or None
// (this does not affect other communication, you can still send/receive binary config/query messages)
inline char VenusSetOutput(byte outputmode, bool flash) { return VenusSetOption(VENUS_CONFIG_OUTPUT_MSG_FORMAT, outputmode, flash); }

// GPS Wide Area Augmentation System
inline byte VenusQueryWAAS() { return VenusGetOption(VENUS_QUERY_WAAS); }
inline char VenusSetWAAS(char enable, bool flash) { return VenusSetOption(VENUS_CONFIG_WAAS, enable, flash); }

// Set power mode to POWER SAVE or NORMAL
inline byte VenusQueryPowerMode() { return VenusGetOption(VENUS_QUERY_POWER_MODE); }
inline char VenusSetPowerMode(char enable, bool flash) { return VenusSetOption(VENUS_CONFIG_POWER_MODE, enable, flash); }

// position pinning snaps GPS location to prevent it from constantly moving
// (not a good thing for our airborne copters!)
inline byte VenusQueryPositionPinning() { return VenusGetOption(VENUS_QUERY_GPS_PINNING); }
inline char VenusSetPositionPinning(char enable, bool flash) { return VenusSetOption(VENUS_CONFIG_GPS_PINNING, enable, flash); }

// Self-Aided Ephemeris Estimation (SAEE) is a GNSS feature to aide in the power-on gps lock
// This feature cannot be enabled while the update rate is more than once a second so we disable it
inline byte VenusQuerySAEE() { return Venus8GetExtendedOption(VENUS8_EXT3, VENUS8_QUERY_SAEE); }
inline char VenusSetSAEE(uint32_t value, bool flash) { return Venus8SetExtendedOption(VENUS8_EXT3, VENUS8_CONFIG_SAEE, value, flash); }

// Nav Mode to Auto, Car, Pedestrian, *Airborne*
#ifdef VENUS8
inline byte VenusQueryNavMode() { return Venus8GetExtendedOption(VENUS8_EXT4, VENUS8_QUERY_NAV_MODE); }
inline char VenusSetNavMode(char navmode, bool flash) { return Venus8SetExtendedOption(VENUS8_EXT4, VENUS8_CONFIG_NAV_MODE, navmode, flash); }
#else
inline byte VenusQueryNavMode() { return VenusGetOption(VENUS_QUERY_NAV_MODE); }
inline char VenusSetNavMode(char navmode, bool flash) { return VenusSetOption(VENUS_CONFIG_NAV_MODE, navmode ? 1 : 0 , flash); }
#endif


#endif
#endif

