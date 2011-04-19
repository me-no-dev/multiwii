/*******************************/
/****CONFIGURABLE PARAMETERS****/
/*******************************/

/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1190

/* The type of multicopter */
//#define GIMBAL
//#define BI
#define TRI
//#define QUADP
//#define QUADX
//#define Y4
//#define Y6
//#define HEX6
//#define HEX6X
//#define FLYING_WING //experimental

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

//#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

#define PROMINI  //Arduino type
//#define MEGA

//enable internal I2C pull ups
//#define INTERNAL_I2C_PULLUPS

//****** advanced users settings   *************

/* Failsave settings - added by MIS
   Failsafe check pulse on THROTTLE channel. If the pulse is OFF (on only THROTTLE or on all channels) the failsafe procedure is initiated.
   After FAILSAVE_DELAY time of pulse absence, the level mode is on (if ACC or nunchuk is avaliable), PITCH, ROLL and YAW is centered
   and THROTTLE is set to FAILSAVE_THR0TTLE value. You must set this value to descending about 1m/s or so for best results. 
   This value is depended from your configuration, AUW and some other params. 
   Next, afrer FAILSAVE_OFF_DELAY the copter is disarmed, and motors is stopped.
   If RC pulse coming back before reached FAILSAVE_OFF_DELAY time, after the small quard time the RC control is returned to normal.
   If you use serial sum PPM, the sum converter must completly turn off the PPM SUM pusles for this FailSafe functionality.*/
#define FAILSAFE                                  // Alex: comment this line if you want to deactivate the failsafe function
#define FAILSAVE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAVE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAVE_THR0TTLE  (MINTHROTTLE + 200)    // Throttle level used for landing - may be relative to MINTHROTTLE - as in this case


/* The following lines apply only for a pitch/roll tilt stabilization system
   It is not compatible with Y6 or HEX6 or HEX6X
   Uncomment the first line to activate it */
//#define SERVO_TILT
#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10

/* I2C gyroscope */
#define ITG3200
//#define L3G4200D

/* I2C accelerometer */
//#define ADXL345
//#define BMA020
#define BMA180
//#define NUNCHACK  // if you want to use the nunckuk as a standalone I2C ACC without WMP

/* I2C barometer */
#define BMP085

/* I2C magnetometer */
//#define HMC5843
#define HMC5883

/* ADC accelerometer */ // for 5DOF from sparkfun, uses analog PIN A1/A2/A3
//#define ADCACC

/* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
   IF YOUR RECEIVER IS NOT CONCERNED, DON'T UNCOMMENT ANYTHING. Note this is mandatory for a Y6 setup
   Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL //For Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL //For some Hitec/Sanwa/Others

/* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
   if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
   it is relevent only for a conf with NK */
#define INTERLEAVING_DELAY 3000

/* for V BAT monitoring
   after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
   with R1=33k and R2=51k
   vbat = [0;1023]*16/VBATSCALE */
#define VBAT              // comment this line to suppress the vbat code
#define VBATSCALE     131 // change this value if readed Battery voltage is different than real voltage
#define VBATLEVEL1_3S 107 // 10,7V
#define VBATLEVEL2_3S 103 // 10,3V
#define VBATLEVEL3_3S 99  // 9.9V

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
   it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000

/* this is the value for the ESCs when thay are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000

/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 1850

/* This is the speed of the serial interface. 115200 kbit/s is the best option for a USB connection.*/
#define SERIAL_COM_SPEED 115200

/* In order to save space, it's possibile to desactivate the LCD configuration functions
   comment this line only if you don't plan to used a LCD */
#define LCD_CONF

/* to use Cat's whisker TEXTSTAR LCD, uncomment following line.
   Pleae note this display needs a full 4 wire connection to (+5V, Gnd, RXD, TXD )
   Configure display as follows: 115K baud, and TTL levels for RXD and TXD, terminal mode
   NO rx / tx line reconfiguration, use natural pins */
//#define LCD_TEXTSTAR

/* motors will not spin when the throttle command is in low position
   this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/* experimental
   camera trigger function : activated via AUX1 UP, servo output=A2 */
//#define CAMTRIG
#define CAM_SERVO_HIGH 2000  // the position of HIGH state servo
#define CAM_SERVO_LOW 1020   // the position of LOW state servo
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms
#define CAM_TIME_LOW 1000    // the duration of LOW state servo expressed in ms

/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500

//****** end of advanced users settings *************

/**************************************/
/****END OF CONFIGURABLE PARAMETERS****/
/**************************************/
