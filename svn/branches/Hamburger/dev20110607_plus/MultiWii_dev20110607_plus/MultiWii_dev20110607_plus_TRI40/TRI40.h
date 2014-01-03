/* TRI60 3S configs */

/* 
hier nur eintragen, was nicht default ist.
im original sketch nach "end of advanced users settings" ein 
	include auf dieses file einfuegen 
*/


#define MINTHROTTLE 1140 // for Super Simple ESCs 10A

#define TRI

#define YAW_DIRECTION -1


#define I2C_SPEED 400000L 

#define FAILSAVE_DELAY     20 



/* wmp + bma020 */
#undef INTERNAL_I2C_PULLUPS
#define BMA020 // acc


#define VBATSCALE     130 // mein Spannungsteiler fuer 3S : 33k + 51k
#define VBATLEVEL1_3S 102 // 10.2V unter Last
#define VBATLEVEL2_3S 100 // 10.0V unter Last
#define VBATLEVEL3_3S 98  // 9.8V unter Last


#define LCD_CONF

#define LCD_TEXTSTAR


//#define MOTOR_STOP


#define TRI_YAW_CONSTRAINT_MIN 1120
#define TRI_YAW_CONSTRAINT_MAX 1900


#define POWERMETER
//#define PLEVELDIV 3730 //37550 // setzen auf 10.000 / 900mAh * 3200 counter

#define LCD_TELEMETRY
#define VBATREF 24 // 12.6V - VBATLEVEL1_3S  (for me = 126 - 102 = 24)
#define LCD_TELEMETRY_AUTO 2500000

