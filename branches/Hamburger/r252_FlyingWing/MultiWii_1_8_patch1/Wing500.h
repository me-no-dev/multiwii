/* Flying Wing - mini wing 500 -  configs */

/* 
hier nur eintragen, was nicht default ist.
im original sketch nach "include config.h" ein 
	include auf dieses file einfuegen 
*/

#undef TRI
#undef QUADX
#undef SERIAL_SUM_PPM
#undef GPS
#undef GPS_SERIAL
#undef FFIMUv2
#undef QUADRINO
#undef MEGA



#define MINTHROTTLE 1140 // for plush 12A
#define MAXTHROTTLE 2000


#define FLYING_WING
#define MOTOR_STOP


//#define DEADBAND 10


#define PROMINI


#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones


//#define LEVEL_PDF
#define ITG3200_LPF_42HZ


#define LCD_CONF
#define LCD_TEXTSTAR
#define LCD_TELEMETRY 109011
//#define LCD_TELEMETRY_AUTO 2070123

// bytes sparen
#define FAILSAFE
//#undef VBAT


#define LOG_VALUES


/* Flying Wing: you can change change servo orientation and servo min/max values here */
/* valid for all flight modes, even passThrough mode */
/* need to setup servo directions here; no need to swap servos amongst channels at rx */ 
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // left servo center pos. - use this for trim
#define WING_RIGHT_MID 1500 // right servo center pos. - use this for trim
#define WING_LEFT_MIN  1020 // limit servo travel range
#define WING_LEFT_MAX  2000 // limit servo travel range
#define WING_RIGHT_MIN 1020 // limit servo travel range
#define WING_RIGHT_MAX 2000 // limit servo travel range

