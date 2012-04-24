
/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1220
#define MINTHROTTLE 1150 

/* The type of multicopter */
//#define GIMBAL
//#define BI
//#define TRI
//#define QUADP
#define QUADX
//#define Y4
//#define Y6
//#define HEX6
//#define HEX6X
//#define OCTOX8
//#define OCTOFLATP
//#define OCTOFLATX
//#define FLYING_WING
//#define VTAIL4

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

//enable internal I2C pull ups
#define INTERNAL_I2C_PULLUPS


/********************************************************************/
/****               Sensors                                     *****/
/********************************************************************/
/*
modules are activated by uncommenting their header files 
porc's modules are still autodetected in core_def.h
WMP is active as long as no other gyro or IMU with gyro is defined
*/

//=== Single Gyros ===//
// ITG3200 & ITG3205
//#include "ITG320X_config.h"


//=== Single ACC's ===//
// BMA020
//#include "BMA020_config.h"
// BMA180
//#include "BMA180_config.h"


//=== IMU's ===//
// MPU6050
//#include "MPU6050_config.h"



/********************************************************************/
/****               RX's                                        *****/
/********************************************************************/
// standard RX is active as long as no special rx is activated

/* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2
   IF YOUR RECEIVER IS NOT CONCERNED, DON'T UNCOMMENT ANYTHING. Note this is mandatory for a Y6 setup on a promini
   Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4 //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For Robe/Hitec/Futaba
//#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4 //For some Hitec/Sanwa/Others

// spectrum satellite RX
//#include "Spekt_Sat_config.h"


/********************************************************************/
/****               advanced users settings                     *****/
/********************************************************************/



/* interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config
   if the ACC calibration time is very long (20 or 30s), try to increase this delay up to 4000
   it is relevent only for a conf with NK */
#define INTERLEAVING_DELAY 3000

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
   it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000


/********************************************************************/
/****           motor, servo and other presets                   ****/
/********************************************************************/

/* The following lines apply only for a pitch/roll tilt stabilization system
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

/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000

/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 1850

/* This is the speed of the serial interface. 115200 kbit/s is the best option for a USB connection.*/
#define SERIAL_COM_SPEED 115200

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500









/********************************************************************/
/****           system related                                   ****/
/********************************************************************/

#include "core_def.h"
