/* TRI60 3S configs */

/* 
hier nur eintragen, was nicht default ist.
im original sketch nach "end of advanced users settings" ein 
	include auf dieses file einfuegen 
*/


#define MINTHROTTLE 1120 // for Super Simple ESCs 10A

#define TRI

#define YAW_DIRECTION -1



#define FAILSAVE_DELAY     20 


#define FREEIMUv01

/* fax8 freeimu 0.1 */
#undef INTERNAL_I2C_PULLUPS
//#define ITG3200 // gyro
//#define ADXL345 // acc
//#define HMC5843 // magneto
//#define ADXL345_ADDRESS 0xA6 // taken from fabio's library and his post http://wbb.multiwii.com/viewtopic.php?f=8&t=262


#define VBATSCALE     78 //79 //110 // mein spezieller Spannungsteiler 3S
#define VBATLEVEL1_3S 102 // 10.2V unter Last
#define VBATLEVEL2_3S 100 // 10.0V unter Last
#define VBATLEVEL3_3S 98  // 9.8V unter Last


#define LCD_CONF

#define LCD_TEXTSTAR



//#define MOTOR_STOP


#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000


#define POWERMETER hard
#define PLEVELDIVSOFT 10000
#define PLEVELDIV 447214L //424745L // to convert the sum into mAh divide by this value
#define PSENSORNULL 496

#define LCD_TELEMETRY
#define VBATREF 24 // 12.6V - VBATLEVEL1_3S  (for me = 126 - 102 = 24)
//#define LCD_TELEMETRY_AUTO 2000000

#define LOG_VALUES

#ifdef INTERNAL_I2C_PULLUPS
	#error "for freeimu from fax8, you MUST undefine internal i2c pullups. Else you destroy board!"
#endif
