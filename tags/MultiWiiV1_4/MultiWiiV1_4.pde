/*
MultiWiiCopter by Alexandre Dubus
radio-commande.com
November 2010     V1.4
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/


/*******************************/
/****CONFIGURABLE PARAMETERS****/
/*******************************/

/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller) */
/* This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
#define MINTHROTTLE 1120 // for Super Simple ESCs 10A

/* The type of multicopter */
//#define GIMBAL
//#define BI //experimental
#define TRI
//#define QUADP
//#define QUADX
//#define Y6
//#define HEX6 // <- in a next release

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction (TRICOPTER only)
//#define YAW_DIRECTION -1

#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

/* This is the speed of the serial interface. 115200 kbit/s is the best option for a USB connection.*/
#define SERIAL_COM_SPEED 115200

/* The following lines apply only for specific receiver with only one PPM sum signal, on digital PIN 2 */
/* If your receiver is concerned, uncomment on of these line. Note this is mandatory for a Y6 setup */
/* Select the right line depending on your radio brand. Feel free to modify the order in your PPM order is different */
//#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1 //For Graupner/Spektrum
//#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1 //For Robe/Hitec/Futaba

/* The following lines apply only for a pitch/roll tilt stabilization system */
/* It is compatible only with a QUAD+ or QUAX setup or a pure GIMBLE setup*/
/* Uncomment the first line to activate it */
//#define SERVO_TILT
#define TILT_PITCH_MIN    1020    //servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000    //servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500    //servo neutral value
#define TILT_PITCH_PROP   10      //servo proportional (tied to angle) ; can be negative to invert movement
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500
#define TILT_ROLL_PROP    10

/* In order to save space, it's possibile to desactivate the LCD configuration functions */
/* comment this line only if you don't plan to used a LCD */
#define LCD_CONF

/**************************************/
/****END OF CONFIGURABLE PARAMETERS****/
/**************************************/


#define INTERLEAVING_DELAY 3000 // interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config

//for V BAT monitoring
//after the resistor divisor we should get [0V;5V]->[0;[1023] on analog V_BATPIN
//the code is implemented but not yet adjusted to empirical V levels
#define VBATLEVEL1_3S 1000
#define VBATLEVEL2_3S 800
#define VBATLEVEL3_3S 700


#include <EEPROM.h>

//RX PIN assignment
#define THROTTLEPIN 2
#define ROLLPIN 4
#define PITCHPIN 5
#define YAWPIN 6
#define AUX1PIN 7


/*********** motor PIN alias ************/
#define PWM_PIN1 9
#define PWM_PIN2 10
#define PWM_PIN3 11
#define PWM_PIN4 3
#define PWM_PIN5 6
#define PWM_PIN6 5

//PIN for TRI and QUAD+ and Y6(TOP motor)
#define REAR_PIN        PWM_PIN1
#define RIGHT_PIN       PWM_PIN2
#define LEFT_PIN        PWM_PIN3

//PIN for QUAD+
#define FRONT_PIN       PWM_PIN4

//PIN for QUADX 
#define REAR_RIGHT_PIN  PWM_PIN1
#define FRONT_RIGHT_PIN PWM_PIN2
#define REAR_LEFT_PIN   PWM_PIN3
#define FRONT_LEFT_PIN  PWM_PIN4

//PIN for Y6 
#define UNDER_REAR_PIN  PWM_PIN4  
#define UNDER_RIGHT_PIN PWM_PIN5
#define UNDER_LEFT_PIN  PWM_PIN6


#define LEDPIN 13
#define POWERPIN 12
#define BUZZERPIN 8              
#define V_BATPIN 3        //Analog PIN 3

#define SERVO_TILT_PITCH_PIN A0
#define SERVO_TILT_ROLL_PIN  A1

/*********** motor alias *****************/
//for TRI and QUAD+ and Y6(TOP motor)  
#define REAR 0
#define RIGHT 1
#define LEFT 2

// alias for QUAD+      
#define FRONT 3

// alias for QUADX
#define REAR_R 0
#define FRONT_R 1
#define REAR_L 2
#define FRONT_L 3

// alias for Y6
#define UNDER_REAR 3
#define UNDER_RIGHT 4
#define UNDER_LEFT 5


/*********** RC alias *****************/
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define AUX1 4


static uint32_t previousTime;
static uint32_t neutralizeTime;     //when there is an error on I2C bus, we neutralize the values during a short time
static uint32_t rcTime;
static uint32_t currentTime;
static uint8_t nunchukPresent = 0;
static uint8_t levelModeParam = 0;  //if level mode is a activated on the radio : channel 5 value superior to 1700
static uint16_t cycleTime;          // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t meanTime = 2000;    // this is the average time of the loop: around 2ms for a WMP config and 6ms for a NK+WMP config
// to be more precise, the calibration is now done is the main loop. Calibrating decreases at each cycle up to 0, then we enter in a normal mode.
// we separate the calibration of ACC and gyro, because number of measure is not always equal.
static uint16_t calibratingA;
static uint16_t calibratingG;
static uint8_t armed = 0;

// **************
// Wii Motion Plus I2C
// **************
static uint8_t rawADC[6];

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void) {
  PORTC |= 1<<4; // activate internal pull-ups PIN A4 for twi
  PORTC |= 1<<5; // activate internal pull-ups PIN A5 for twi
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REAPEAT START condition
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
  TWDR = address; // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wail until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

void i2c_write(uint8_t data ) {	
  TWDR = data; // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C(); // wait until transmission completed
  checkStatusI2C(); // check value of TWI Status Register
}

uint8_t i2c_readAck() {
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  waitTransmissionI2C();
  return TWDR;
}

uint8_t i2c_readNak(void) {
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
  return TWDR;
}

void waitTransmissionI2C() {
  uint8_t count = 255;
  while (count-->0 && !(TWCR & (1<<TWINT)) );
  if (count<2) { //we are in a blocking state => we don't insist
    TWCR = 0;  //and we force a reset on TWINT register
    neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay after the hard reset
  }
}

void checkStatusI2C() {
  if ( (TW_STATUS & 0xF8) == 0xF8) { //TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
    // WMP does not respond anymore => we do a hard reset. I did not find another way to solve it. It takes only 13ms to reset and init to WMP or WMP+NK
    TWCR = 0;
    digitalWrite(POWERPIN,0);
    delay(1);  
    digitalWrite(POWERPIN,1);
    delay(10);  
    i2c_rep_start(0xA6);
    i2c_write(0xF0);
    i2c_write(0x55);
    i2c_rep_start(0xA6);
    i2c_write(0xFE);
    i2c_write(0x05);
    neutralizeTime = micros(); //we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay (20ms) after the hard reset
  }
}

void initI2C(void) {
  i2c_init();
  delay(250);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  delay(250);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xFE); 
  i2c_write(0x05); 
  delay(250);
}

void getI2C() {
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  i2c_rep_start(0xA4 + 1);//I2C read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC[i]=i2c_readAck();}
  rawADC[5]= i2c_readNak();
}

// **************
// gyro+nunchuk IMU
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3]  = {0,0,0};
static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t angle[2]; //absolute angle inclination in Deg

uint8_t rawIMU () { //if the WMP or NK are oriented differently, it can be changed here
  getI2C();
  if ( rawADC[5]&0x02 ) {// motion plus data
    gyroADC[PITCH]  = - ( ((rawADC[4]>>2)<<8) + rawADC[1] );
    gyroADC[ROLL]   = - ( ((rawADC[5]>>2)<<8) + rawADC[2] );
    gyroADC[YAW]    = - ( ((rawADC[3]>>2)<<8) + rawADC[0] );
    return 1;
  } else { //nunchuk data
    accADC[PITCH] = - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) );
    accADC[ROLL]  =   ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) );
    accADC[YAW]   = - ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) );
    return 0;
  }
}

void initIMU(void) {
  uint8_t numberAccRead = 0;
  initI2C();  
  for(uint8_t i=0;i<100;i++) {
    delay(3);
    if (rawIMU() == 0) numberAccRead++; // we detect here is nunchuk extension is available
  }
  if (numberAccRead>30)
    nunchukPresent = 1;

  delay(10);
}

uint8_t updateIMU() {
  static int32_t g[3];
  static int32_t a[3];
  uint8_t axis;
  
  if (rawIMU()) { //gyro
    if (calibratingG>0) {
      for (axis = 0; axis < 3; axis++) {
        if (calibratingG>1) {
          if (calibratingG == 400) g[axis]=0;
          g[axis] +=gyroADC[axis];
          gyroADC[axis]=0;
        } else {
          gyroZero[axis]=(g[axis]+200)/399;
          blinkLED(10,15,1);
        }
      }
      calibratingG--;
    } else {
      gyroADC[PITCH] = gyroADC[PITCH] - gyroZero[PITCH];
      gyroADC[ROLL]  = gyroADC[ROLL]  - gyroZero[ROLL];
      gyroADC[YAW]   = gyroADC[YAW]   - gyroZero[YAW]; 
    }
    gyroADC[PITCH] = (rawADC[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH] ;  //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
    gyroADC[ROLL]  = (rawADC[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL] ;   //the ratio 1/5 is not exactly the IDG600 or IFG650 specification 
    gyroADC[YAW]   = (rawADC[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW] ;
    return 1;
  } else { //nunchuk
    if (calibratingA>0 && nunchukPresent) {
      if (calibratingA>1) {
        for (axis = 0; axis < 3; axis++) {
          if (calibratingA == 400) a[axis]=0;
          a[axis] +=accADC[axis];
          accADC[axis]=0;
        }
      } else {
        accZero[ROLL]=(a[ROLL]+200)/399;
        accZero[PITCH]=(a[PITCH]+200)/399;
        accZero[YAW]=(a[YAW]+200)/399+200; // for nunchuk 200=1G
        writeParams(); // write accZero in EEPROM
        blinkLED(10,15,1);
      }
      calibratingA--;
    } else {
      accADC[PITCH] =    accADC[PITCH] - accZero[PITCH];
      accADC[ROLL]  =    accADC[ROLL]  - accZero[ROLL] ;
      accADC[YAW]   = - (accADC[YAW]   - accZero[YAW]) ;

    }
    return 0;
  }
}

// ************************************
// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// with this algorithm, we can get absolute angles for a stable mode integration
// ************************************
void getEstimatedInclination(){
  int8_t i;  
  float R;
  static float RxEst = 0;      // init acc in stable mode
  static float RyEst = 0;
  static float RzEst = 1;
  static float Axz,Ayz;        //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
  static int16_t accSmooth[3]; //projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer       
  float RxGyro,RyGyro,RzGyro;  //R obtained from last estimated value and gyro movement
  uint8_t wGyro = 150;         // gyro weight/smooting factor
  static float invW = 1.0/(1 + 150);
  float gyroFactor = 2*INTERLEAVING_DELAY/166e6; //empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  float a,b;

  for (i=0;i<3;i++)
     accSmooth[i] =(accSmooth[i]*7+accADC[i]+4)/8;

  //normalize vector (convert to a vector with same direction and with length 1)
  R = sqrt(square(accSmooth[ROLL]) + square(accSmooth[PITCH]) + square(accSmooth[YAW]));

  if(accADC[YAW] > 20 && R != 0.0 ){ //we want to be sure we are not flying inverted
    //get angles between projection of R on ZX/ZY plane and Z axis, based on last REst
    //Convert ADC value for to physical units
    //For gyro it will return  deg/ms (rate of rotation)
  
    a = gyroADC[ROLL]  * gyroFactor;
    b = gyroADC[PITCH] * gyroFactor; 
    if (R > 280.0 || R < 120) { //if accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation
      Axz +=  a;     //and we use only gyro integration
      Ayz +=  b;
    } else if (RzEst != 0.0) {
      Axz = atan2(RxEst,RzEst) + a;  // convert ADC value for to physical units
      Ayz = atan2(RyEst,RzEst) + b;  // and get updated angle according to gyro movement
    }
    
    //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
    RxGyro = sin(Axz) / sqrt( 1.0 + square(cos(Axz)) * square(tan(Ayz)) );
    RyGyro = sin(Ayz) / sqrt( 1.0 + square(cos(Ayz)) * square(tan(Axz)) );        
    RzGyro = sqrt(1 - square(RxGyro) - square(RyGyro));
  
    //combine Accelerometer and gyro readings
    RxEst = (accSmooth[ROLL]/R + wGyro* RxGyro)  * invW;
    RyEst = (accSmooth[PITCH]/R + wGyro* RyGyro) * invW;
    RzEst = (accSmooth[YAW]/R + wGyro* RzGyro)   * invW;
  }
  angle[ROLL]  =  degrees(Axz);
  angle[PITCH] =  degrees(Ayz);
}

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCp[3] = {0,0,0};
  int16_t gyroADCinter[3];
  static int16_t lastAccADC[3] = {0,0,0};
  static int16_t similarNumberAccData[3];
  static int16_t gyroDeviation[3];
  static uint32_t timeInterleave;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  if (nunchukPresent) {
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //it replaces delayMicroseconds(3000); //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    updateIMU();
    if (calibratingA == 0 && currentTime > (neutralizeTime + 20000)) getEstimatedInclination(); //getEstimatedInclination computation must last less than 3ms
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //it replaces delayMicroseconds(3000); //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    while(updateIMU() != 1) ; // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis]+16)/4/8; // /4 is to average 4 values ; /8 is to reduce the sensibility of gyro
      gyroADCprevious[axis] = gyroADC[axis];
    }

/*
    for (axis = 0; axis < 2; axis++) {
      //automatic gyro calibration
      //adaptation of FabQuad proposition http://aeroquad.com/entry.php?4-Calibrate-gyros-...-during-flight
      if (abs(lastAccADC[axis]-accADC[axis]) < 4) {
        similarNumberAccData[axis]++;
        if (similarNumberAccData[axis] >= 4) {
          gyroDeviation[axis] += gyroData[axis];
          similarNumberAccData[axis]=0;
          if (gyroDeviation[axis] >50) {
            gyroZero[axis]+=5;
            gyroDeviation[axis]=0;
          } else if (gyroDeviation[axis] <-50) {
            gyroZero[axis]-=5;
            gyroDeviation[axis]=0;
          }
        }
      } else {
        similarNumberAccData[axis]=0;
        lastAccADC[axis] = accADC[axis];
      }
    }
*/
  } else {
    updateIMU();
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    while((micros()-timeInterleave)<650) ; //empirical it replaces delayMicroseconds(650); //interleaving delay between 2 consecutive reads
    updateIMU();
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis]+12)/3/8; // /3 is to average 3 values ; /8 is to reduce the sensibility of gyro
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      accADC[axis]=0;
    }
  }
  if (currentTime < (neutralizeTime + 20000)) //we neutralize data for 20ms in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) gyroData[axis]=0;
}

// *************************
// motor and servo functions
// *************************
#define MINCOMMAND 1000
#define MAXCOMMAND 1850

static int16_t axisPID[3];
static int16_t motor[6];
static int16_t servoYaw = 1500;
volatile int8_t atomicServoYaw = 125;

#ifdef Y6
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;
#endif

void writeMotors() { // [1000;2000] => [125;250]
  #if defined(BI)
    analogWrite(REAR_PIN, motor[REAR]>>3);
    analogWrite(FRONT_PIN, motor[FRONT]>>3);
  #elif defined(TRI)
    analogWrite(REAR_PIN, motor[REAR]>>3);
    analogWrite(RIGHT_PIN, motor[RIGHT]>>3);
    analogWrite(LEFT_PIN, motor[LEFT]>>3);
  #elif defined(QUADP)
    analogWrite(REAR_PIN, motor[REAR]>>3);
    analogWrite(RIGHT_PIN, motor[RIGHT]>>3);
    analogWrite(LEFT_PIN, motor[LEFT]>>3);
    analogWrite(FRONT_PIN, motor[FRONT]>>3);
  #elif defined(QUADX)
    analogWrite(REAR_RIGHT_PIN, motor[REAR_R]>>3);
    analogWrite(FRONT_RIGHT_PIN, motor[FRONT_R]>>3);
    analogWrite(REAR_LEFT_PIN, motor[REAR_L]>>3);
    analogWrite(FRONT_LEFT_PIN, motor[FRONT_L]>>3);
  #elif defined(Y6) 
    analogWrite(REAR_PIN, motor[REAR]>>3);
    analogWrite(RIGHT_PIN, motor[RIGHT]>>3);
    analogWrite(LEFT_PIN, motor[LEFT]>>3);
    analogWrite(UNDER_REAR_PIN, motor[UNDER_REAR]>>3);
    atomicPWM_PIN5_highState = (motor[UNDER_LEFT]-1000)/4+4;
    atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
    atomicPWM_PIN6_highState = (motor[UNDER_RIGHT]-1000)/4+4;
    atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
  #endif
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<6;i++)
    motor[i]=mc;
  writeMotors();
}

void initializeMotors() {  
  #if defined(BI)
    pinMode(FRONT_PIN, OUTPUT);
    pinMode(REAR_PIN, OUTPUT);
  #elif defined(TRI)
    pinMode(LEFT_PIN, OUTPUT);pinMode(RIGHT_PIN, OUTPUT);
    pinMode(REAR_PIN, OUTPUT);
  #elif defined(QUADP)
    pinMode(FRONT_PIN, OUTPUT);
    pinMode(LEFT_PIN, OUTPUT);pinMode(RIGHT_PIN, OUTPUT);
    pinMode(REAR_PIN, OUTPUT);
  #elif defined(QUADX)
    pinMode(REAR_LEFT_PIN, OUTPUT); pinMode(REAR_RIGHT_PIN, OUTPUT);
    pinMode(FRONT_LEFT_PIN, OUTPUT);pinMode(FRONT_RIGHT_PIN, OUTPUT);
  #elif defined(Y6) 
    pinMode(LEFT_PIN, OUTPUT);      pinMode(RIGHT_PIN, OUTPUT);
    pinMode(UNDER_LEFT_PIN, OUTPUT);pinMode(UNDER_RIGHT_PIN, OUTPUT);
    pinMode(REAR_PIN, OUTPUT);
    pinMode(UNDER_REAR_PIN, OUTPUT);
  #endif
  writeAllMotors(2000); //max motor command is maintained for 100ms at the initialization
  delay(100); //it's a way to calibrate the ESCs
  writeAllMotors(1000);
  delay(300);
}

#ifdef TRI
void initializeServo() {
  pinMode(3,OUTPUT);
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}

// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
// algorithm strategy:
// pulse high
// do nothing for 1000 us
// do nothing for [0 to 1000] us
// pulse down
// do nothing for 18 counter cycles
ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  static uint8_t count;

  if (state == 0) {
    //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
    PORTD |= 1<<3; // this is 25 time faster than DigitalWrite ! coded for Digital Pin 3 only
    OCR0A+= 250; // 1000 us
    state++ ;
  } else if (state == 1) {
    OCR0A+= atomicServoYaw; // 1000 + [0-1020] us
    state++;
  } else if (state == 2) {
    PORTD &= ~(1<<3);
    count = 16; // 18 x 1020 us
    state++;
    OCR0A+= 255; // 1020 us
  } else if (state == 3) {
    if (count > 0) count--;
    else state = 0;
    OCR0A+= 255;
  }
}
#endif

#if defined(BI)
static int16_t servoFront = 1500; //for BICOPTER
volatile int8_t atomicServoFront = 125; //for BICOPTER

void initializeServo() {
  pinMode(RIGHT_PIN,OUTPUT); //thses 2 PIN are used to control 2 servos
  pinMode(LEFT_PIN,OUTPUT);
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t count,state = 0;
  if (state == 0) {
    PORTB |= 1<<3; //digital PIN 11
    OCR0A+= 250;state++ ;
  } else if (state == 1) {
    OCR0A+= atomicServoYaw; state++;
  } else if (state == 2) {
    PORTB &= ~(1<<3);count=16;state++;OCR0A+= 255;
  } else if (state == 3) {
    if (count > 0) count--; else state = 0;
    OCR0A+= 255;
  }
}

ISR(TIMER0_COMPB_vect) {
  static uint8_t count,state = 0;
  if (state == 0) {
    PORTB |= 1<<2; //digital PIN 10
    OCR0B+= 250;state++ ;
  } else if (state == 1) {
    OCR0B+= atomicServoFront;state++;
  } else if (state == 2) {
    PORTB &= ~(1<<2);count=16;state++;OCR0B+= 255;
  } else if (state == 3) {
    if (count > 0) count--; else state = 0;
    OCR0B+= 255;
  }
}
#endif

#ifdef Y6
void initializeSoftPWM() {
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<5; //digital PIN 5 high
    OCR0A+= 250; //250 x 4 microsecons = 1ms
    state = 1;
  } else if (state == 1) {
    OCR0A+= atomicPWM_PIN5_highState;
    state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<5); //digital PIN 5 low
    OCR0A+= atomicPWM_PIN5_lowState;
    state = 0;
  }
}

ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<6;OCR0B+= 250;state = 1;
  } else if (state == 1) {
    OCR0B+= atomicPWM_PIN6_highState;state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<6);OCR0B+= atomicPWM_PIN6_lowState;state = 0;
  }
}
#endif

#if defined(SERVO_TILT) || defined(GIMBAL)
static int16_t servoPitch = 1500;
static int16_t servoRoll = 1500;
volatile int8_t atomicServoPitch = 125;
volatile int8_t atomicServoRoll = 125;

void initializeServo() {
  pinMode(SERVO_TILT_PITCH_PIN,OUTPUT); //thses 2 PIN are used to control 2 servos
  pinMode(SERVO_TILT_ROLL_PIN, OUTPUT);
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t count,state = 0;
  if (state == 0) {
    digitalWrite(SERVO_TILT_PITCH_PIN,1);
    OCR0A+= 250;state++ ;
  } else if (state == 1) {
    OCR0A+= atomicServoPitch;
    state++;
  } else if (state == 2) {
    digitalWrite(SERVO_TILT_PITCH_PIN,0);
    count=16;state++;OCR0A+= 255;
  } else if (state == 3) {
    if (count > 0) count--; else state = 0;
    OCR0A+= 255;
  }
}

ISR(TIMER0_COMPB_vect) {
  static uint8_t count,state = 0;
  if (state == 0) {
    digitalWrite(SERVO_TILT_ROLL_PIN,1);
    OCR0B+= 250;state++ ;
  } else if (state == 1) {
    OCR0B+= atomicServoRoll;
    state++;
  } else if (state == 2) {
    digitalWrite(SERVO_TILT_ROLL_PIN,0);
    count=16;state++;OCR0B+= 255;
  } else if (state == 3) {
    if (count > 0) count--; else state = 0;
    OCR0B+= 255;
  }
}
#endif

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

static uint8_t pinRcChannel[5] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN};
volatile uint16_t rcPinValue[8] = {0,0,1000,0,1500,1500,1500,1000};; // interval [1000;2000]
static int16_t rcData[5] ; // interval [1000;2000]
static int16_t rcCommand[4] = {0,0,0,0}; // interval [-500;+500]
static int16_t rcHysteresis[5] ;
static int16_t rcData4Values[5][4];

static uint8_t rcRate8;
static uint8_t rcExpo8;
static float rcFactor1; 
static float rcFactor2;

// ***PPM SUM SIGNAL***
#ifdef SERIAL_SUM_PPM
static uint8_t rcChannel[5] = {SERIAL_SUM_PPM};
#endif
volatile uint16_t rcValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

// Configure each rc pin for PCINT
void configureReceiver() {
  #ifndef SERIAL_SUM_PPM
    uint8_t chan,a;
    for (chan=0; chan < 6; chan++) {
      pinMode(pinRcChannel[chan], INPUT);
      for (a = 0; a < 4; a++)
        rcData4Values[chan][a] = 1500; //we initiate the default value of each channel. If there is no RC receiver connected, we will see those values
    }
    // PCINT activated only for specific pin inside [D0-D7]  , [D2 D4 D5 D6 D7] for this multicopter
    PCMSK2 |= (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); 
    PCICR   = 1<<2; // PCINT activated only for [D0-D7] port
  #else
    attachInterrupt(0, rxInt, RISING);
  #endif
}

#ifndef SERIAL_SUM_PPM
ISR(PCINT2_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
  uint8_t mask;
  uint8_t pind;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[5];
  static uint8_t PCintLast;

  pind = PIND;              // PIND indicates the state of each PIN for the arduino port dealing with [D0-D7] digital pins (8 bits variable)
  mask = pind ^ PCintLast;  // doing a ^ between the current interruption and the last one indicates wich pin changed
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  PCintLast = pind;         // we memorize the current state of all PINs [D0-D7]
  cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
  // mask is pins [D0-D7] that have changed
  // chan = pin sequence of the port. chan begins at D2 and ends at D7
  if (mask & 1<<2) {          //indicates the bit 2 of the arduino port [D0-D7], that is to say digital pin 2, if 1 => this pin has just changed
    if (!(pind & 1<<2)) {     //indicates if the bit 2 of the arduino port [D0-D7] is not at a high state (so that we match here only descending PPM pulse)
      dTime = cTime-edgeTime[0]; if (900<dTime && dTime<2200) rcPinValue[2] = dTime; // just a verification: the value must be in the range [1000;2000] + some margin
    } else edgeTime[0] = cTime;    // if the bit 2 of the arduino port [D0-D7] is at a high state (ascending PPM pulse), we memorize the time
  } 
  if (mask & 1<<4) { //same principle for other channels   // avoiding a for() is more than twice faster, and it's important to minimize execution time in ISR
    if (!(pind & 1<<4)) {
      dTime = cTime-edgeTime[1]; if (900<dTime && dTime<2200) rcPinValue[4] = dTime;
    } else edgeTime[1] = cTime;
  }
  if (mask & 1<<5) {
    if (!(pind & 1<<5)) {
      dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcPinValue[5] = dTime;
    } else edgeTime[2] = cTime;
  }
  if (mask & 1<<6) {
    if (!(pind & 1<<6)) {
      dTime = cTime-edgeTime[3]; if (900<dTime && dTime<2200) rcPinValue[6] = dTime;
    } else edgeTime[3] = cTime;
  }
  if (mask & 1<<7) {
    if (!(pind & 1<<7)) {
      dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcPinValue[7] = dTime;
    } else edgeTime[4] = cTime;
  }
}

#else 
void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t currentChannel = 0;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>5000) currentChannel = 0;
  else {
    if(900<diff && diff<2200 && currentChannel<8 ) rcValue[currentChannel] = diff;
    currentChannel++;
  }
}
#endif


uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
  #ifndef SERIAL_SUM_PPM
    data = rcPinValue[pinRcChannel[chan]]; // Let's copy the data Atomically
  #else
    data = rcValue[rcChannel[chan]]; 
  #endif
  SREG = oldSREG; // Let's enable the interrupts
  return data; // We return the value correctly copied when the IRQ's where disabled
}
  
void computeRC() {
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;

  rc4ValuesIndex++;
  for (chan = 0; chan < 5; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcData[chan] = 0;
    for (a = 0; a < 4; a++)
      rcData[chan] += rcData4Values[chan][a];
    rcData[chan]= (rcData[chan]+2)/4;
    if ( rcData[chan] < rcHysteresis[chan] -4)  rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +4)  rcHysteresis[chan] = rcData[chan]-2;
  }
}


// ****************
// EEPROM functions
// ****************
static uint8_t  P8[3], I8[3], D8[3]; //8 bits is much faster and the code is much shorter
static uint8_t  dynP8[3], dynI8[3], dynD8[3]; 
static uint8_t accStrength8;
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID1,dynThrPID2,dynThrPID3;

#define MAGICALNUMBER_ADR 0
#define PROLL_ADR 4
#define IROLL_ADR 8
#define DROLL_ADR 12
#define PPITCH_ADR 16
#define IPITCH_ADR 20
#define DPITCH_ADR 24
#define PYAW_ADR 28
#define IYAW_ADR 32
#define DYAW_ADR 36
#define RCRATE_ADR 40
#define RCEXPO_ADR 44
#define ACCSTRENGTH_ADR 48
#define ROLLPITCHRATE_ADR 52
#define YAWRATE_ADR 56
#define DYNTHR1_ADR 64
#define DYNTHR2_ADR 68
#define DYNTHR3_ADR 72
#define XACCMSB_ADR 80
#define XACCLSB_ADR 81
#define YACCMSB_ADR 82
#define YACCLSB_ADR 83
#define ZACCMSB_ADR 84
#define ZACCLSB_ADR 85

void readEEPROM() {
  P8[ROLL]  = EEPROM.read(PROLL_ADR);  I8[ROLL]  = EEPROM.read(IROLL_ADR);  D8[ROLL]  = EEPROM.read(DROLL_ADR);
  P8[PITCH] = EEPROM.read(PPITCH_ADR); I8[PITCH] = EEPROM.read(IPITCH_ADR); D8[PITCH] = EEPROM.read(DPITCH_ADR);
  P8[YAW]   = EEPROM.read(PYAW_ADR);   I8[YAW]   = EEPROM.read(IYAW_ADR);   D8[YAW]   = EEPROM.read(DYAW_ADR);
  rcRate8   = EEPROM.read(RCRATE_ADR);
  rcExpo8   = EEPROM.read(RCEXPO_ADR);
  accStrength8 = EEPROM.read(ACCSTRENGTH_ADR);
  rollPitchRate = EEPROM.read(ROLLPITCHRATE_ADR);
  yawRate = EEPROM.read(YAWRATE_ADR);
  dynThrPID1=  EEPROM.read(DYNTHR1_ADR);  dynThrPID2= EEPROM.read(DYNTHR2_ADR);  dynThrPID3= EEPROM.read(DYNTHR3_ADR);
  
  accZero[0] = (EEPROM.read(XACCMSB_ADR)<<8) + (EEPROM.read(XACCLSB_ADR)&0xff);
  accZero[1] = (EEPROM.read(YACCMSB_ADR)<<8) + (EEPROM.read(YACCLSB_ADR)&0xff);
  accZero[2] = (EEPROM.read(ZACCMSB_ADR)<<8) + (EEPROM.read(ZACCLSB_ADR)&0xff);

  //note on the following lines: we do this calcul here because it's a static and redundant result and we don't want to load the critical loop whith it
  rcFactor1 = rcRate8/100.0*rcExpo8/100.0/250000.0;
  rcFactor2 = (100-rcExpo8)*rcRate8/10000.0;
}

void writeParams() {
  EEPROM.write(MAGICALNUMBER_ADR, 125);
  EEPROM.write(PROLL_ADR,P8[ROLL]);   EEPROM.write(IROLL_ADR,I8[ROLL]);   EEPROM.write(DROLL_ADR,D8[ROLL]);
  EEPROM.write(PPITCH_ADR,P8[PITCH]); EEPROM.write(IPITCH_ADR,I8[PITCH]); EEPROM.write(DPITCH_ADR,D8[PITCH]);
  EEPROM.write(PYAW_ADR,P8[YAW]);     EEPROM.write(IYAW_ADR,I8[YAW]);     EEPROM.write(DYAW_ADR,D8[YAW]);
  EEPROM.write(RCRATE_ADR,rcRate8);
  EEPROM.write(RCEXPO_ADR,rcExpo8);
  EEPROM.write(ACCSTRENGTH_ADR,accStrength8);
  EEPROM.write(ROLLPITCHRATE_ADR,rollPitchRate);
  EEPROM.write(YAWRATE_ADR,yawRate);
  EEPROM.write(DYNTHR1_ADR,dynThrPID1);EEPROM.write(DYNTHR2_ADR,dynThrPID2);EEPROM.write(DYNTHR3_ADR,dynThrPID3);
  
  EEPROM.write(XACCLSB_ADR,accZero[0]);EEPROM.write(XACCMSB_ADR,accZero[0]>>8&0xff);
  EEPROM.write(YACCLSB_ADR,accZero[1]);EEPROM.write(YACCMSB_ADR,accZero[1]>>8&0xff);
  EEPROM.write(ZACCLSB_ADR,accZero[2]);EEPROM.write(ZACCMSB_ADR,accZero[2]>>8&0xff);

  readEEPROM();
}

void checkFirstTime() {
  if ( EEPROM.read(MAGICALNUMBER_ADR) != 125 ) {
    P8[ROLL] = 40; I8[ROLL] = 30; D8[ROLL] = 13;
    P8[PITCH] = 40; I8[PITCH] = 30; D8[PITCH] = 13;
    P8[YAW]  = 80; I8[YAW]  = 0;  D8[YAW]  = 0;
    rcRate8 = 90;
    rcExpo8 = 65;
    accStrength8 = 100;
    rollPitchRate = 0;
    yawRate = 0;
    dynThrPID1 = 100; dynThrPID2 = 100; dynThrPID3 = 100;
    writeParams();
  }
}


// *****************************
// LCD & display & monitoring
// *****************************

// 1000000 / 9600  = 104 microseconds at 9600 baud.
// the minimum supported value is 96 for sparkfun serial LCD
// we set it at the minimum to take some margin with the running interrupts
#define BITDELAY 96
void LCDprint(uint8_t i) {
  digitalWrite(0, LOW);
  delayMicroseconds(BITDELAY);
  for (uint8_t mask = 0x01; mask; mask <<= 1) {
    if (i & mask) // choose bit
      digitalWrite(0,HIGH); // send 1
    else
      digitalWrite(0,LOW); // send 1
    delayMicroseconds(BITDELAY);
  }
    digitalWrite(0, HIGH);
  delayMicroseconds(BITDELAY);
}

void LCDprintChar(const char *s) {
  while (*s) LCDprint(*s++);
}

void initLCD() {
  Serial.end();
  blinkLED(20,30,1);
  //init LCD
  pinMode(0, OUTPUT); //TX PIN for LCD = Arduino RX PIN (more convenient to connect a servo plug on arduino pro mini)
  LCDprint(0xFE);LCDprint(0x0D); //cursor blink mode
}

void annexCode() { //this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  uint16_t vbat10bits,a,b;
  static uint32_t serialTime;
  static uint32_t buzzerTime;
  static uint32_t calibrateTime;
  static uint8_t  buzzerState = 0;
  uint8_t axis;
  uint8_t prop1,prop2;

  for(axis=0;axis<2;axis++) {
    //PITCH & ROLL dynamic PID adjustemnt, depending on stick deviation
    prop1 = 100-min(abs(rcData[axis]-1500)/5,100)*rollPitchRate/100;
    //PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE]<1400)                               prop2 = 100;
    else if (rcData[THROTTLE]>1399 && rcData[THROTTLE]<1600) prop2 = 100 -  (rcData[THROTTLE]-1400) * (100 - dynThrPID1)/200;
    else if (rcData[THROTTLE]>1599 && rcData[THROTTLE]<1800) prop2 = dynThrPID1 -  (rcData[THROTTLE]-1600)*(dynThrPID1 - dynThrPID2)/200;
    else if (rcData[THROTTLE]>1799 && rcData[THROTTLE]<2000) prop2 = dynThrPID2 -  (rcData[THROTTLE]-1800)*(dynThrPID2 - dynThrPID3)/200;
    else                                                     prop2 = dynThrPID3;
    dynP8[axis] = P8[axis]*prop1/100*prop2/100;
    dynD8[axis] = D8[axis]*prop1/100*prop2/100;
  }
  
  //YAW dynamic PID adjustemnt
  prop1 = 100-min(abs(rcData[YAW]-1500)/5,100)*yawRate/100;
  dynP8[YAW] =    P8[YAW]*prop1/100;
  dynD8[YAW] =    D8[YAW]*prop1/100;

  vbat10bits = analogRead(V_BATPIN);

  if (vbat10bits>VBATLEVEL1_3S) {                                          //VBAT ok, buzzer off
    digitalWrite(BUZZERPIN,0);
  } else if ((vbat10bits>VBATLEVEL2_3S) && (vbat10bits<VBATLEVEL1_3S)) {   //First level 0.25s beep spacing 1s
    if (buzzerState && (currentTime > buzzerTime + 250000) ) {
      buzzerState = 0;digitalWrite(BUZZERPIN,0);buzzerTime = currentTime;
    } else if ( !buzzerState && (currentTime > buzzerTime + 1000000) ) {
      buzzerState = 1;digitalWrite(BUZZERPIN,1);buzzerTime = currentTime;
    }
  } else if ((vbat10bits>VBATLEVEL3_3S) && (vbat10bits<VBATLEVEL2_3S)) {   //First level 0.25s beep spacing 0.5s
    if (buzzerState && (currentTime > buzzerTime + 250000) ) {
      buzzerState = 0;digitalWrite(BUZZERPIN,0);buzzerTime = currentTime;
    } else if ( !buzzerState && (currentTime > buzzerTime + 500000) ) {
      buzzerState = 1;digitalWrite(BUZZERPIN,1);buzzerTime = currentTime;
    }
  } else {                                                                 //Last level 0.25s beep spacing 0.25s
    if (buzzerState && (currentTime > buzzerTime + 250000) ) {
      buzzerState = 0;digitalWrite(BUZZERPIN,0);buzzerTime = currentTime;
    } else if (!buzzerState  && (currentTime > buzzerTime + 250000) ) {
      buzzerState = 1;digitalWrite(BUZZERPIN,1);buzzerTime = currentTime;
    }
  }

  if ( (currentTime > calibrateTime + 100000)  && ( (calibratingA>0 && nunchukPresent) || (calibratingG>0) ) ) {         // Calibration phasis
    PINB |= 1<<5; //switch LEDPIN state (digital PIN 13)
    calibrateTime = currentTime;
  } else if ( (calibratingA==0) || (calibratingG==0 && !nunchukPresent) ) {
    if (armed) digitalWrite(LEDPIN,1);
    else digitalWrite(LEDPIN,0);
  }

  if (currentTime > serialTime + 20000) { // 50Hz
    serialCom();
    serialTime = currentTime;
  }

  rcCommand[ROLL]     =  (rcHysteresis[ROLL]-1500) * (rcFactor2 + rcFactor1*square((rcHysteresis[ROLL]-1500)));
  rcCommand[PITCH]    = (rcHysteresis[PITCH]-1500) * (rcFactor2 + rcFactor1*square((rcHysteresis[PITCH]-1500)));
  rcCommand[THROTTLE] = (MAXCOMMAND-MINTHROTTLE)/(2000.0-1100.0) * ((rcHysteresis[THROTTLE]-1500)+400) + MINTHROTTLE-1500;
  rcCommand[YAW]      =  rcHysteresis[YAW]-1500;

}

void configurationLoop() {
  uint8_t chan;
  uint8_t param;
  uint8_t paramActive;
  uint8_t val;
  uint8_t valActive;
  static char line1[17];
  static char line2[17];
  uint8_t LCD=1;
  uint8_t posLCD = 130;
  uint8_t refreshLCD = 1;

  initLCD();
  param = 1;
  while (LCD == 1) {
    if (refreshLCD == 1) {
      if (param < 4) {
        if (param == 1) {strcpy(line1," *P*    I     D "); posLCD = 130;}  //no more sprintf(), better for code size
        if (param == 2) {strcpy(line1,"  P    *I*    D "); posLCD = 136;}
        if (param == 3) {strcpy(line1,"  P     I    *D*"); posLCD = 142;}
        line2[0] = '0'+P8[ROLL]/100;line2[1] = '0'+P8[ROLL]/10-(P8[ROLL]/100)*10;line2[2] = '.';line2[3] = '0'+P8[ROLL]-(P8[ROLL]/10)*10;
        line2[4] = ' ';line2[5] = ' ';
        line2[6] = '0';line2[7] = '.';line2[8] = '0'+I8[ROLL]/100;line2[9] = '0'+I8[ROLL]/10-(I8[ROLL]/100)*10;line2[10] = '0'+I8[ROLL]-(I8[ROLL]/10)*10;
        line2[11] = ' ';line2[12] = ' ';
        line2[13] = '-';line2[14] = '0'+D8[PITCH]/10;line2[15] = '0'+D8[PITCH]-(D8[PITCH]/10)*10;  
      }
      if (param == 4) {
        line2[0] = '0'+P8[YAW]/100;line2[1] = '0'+P8[YAW]/10-(P8[YAW]/100)*10;line2[2] = '.';line2[3] = '0'+P8[YAW]-(P8[YAW]/10)*10;
        strcpy(line2+4,"            ");
        strcpy(line1,"*P YAW*        "); posLCD = 130;
      }
      LCDprint(0xFE);LCDprint(128);LCDprintChar(line1); //refresh line 1 of LCD
      LCDprint(0xFE);LCDprint(192);LCDprintChar(line2); //refresh line 2 of LCD
      LCDprint(0xFE);LCDprint(posLCD); //cursor position
      refreshLCD=0;
    }
    for (chan = ROLL; chan < 4; chan++)
      rcData[chan] = readRawRC(chan);

    //switch config param with pitch
    if (rcData[PITCH] < MINCHECK && paramActive == 0) {
      paramActive = 1;
      refreshLCD=1;
      param++; if (param>4) param=4;
      if (param < 5) {blinkLED(10,20,param);}
    }
    if (rcData[PITCH] > MAXCHECK && paramActive == 0) {
      paramActive = 1;
      refreshLCD=1;
      param--; if (param<1) param=1;
      if (param < 5) {blinkLED(10,20,param);}
    }
    if (rcData[PITCH] < MAXCHECK && rcData[PITCH] > MINCHECK)  paramActive = 0;

    //+ or - param with low and high roll
    if (rcData[ROLL] < MINCHECK && valActive == 0) {
      valActive = 1;
      refreshLCD=1;
      //set val -
      if (param == 1 && (P8[ROLL]>0)) {P8[ROLL] -= 1 ;P8[PITCH]=P8[ROLL];}
      if (param == 2 && (I8[ROLL]>4)) {I8[ROLL] -= 5 ;I8[PITCH]=I8[ROLL];}
      if (param == 3) {D8[ROLL] += 1; D8[PITCH]=D8[ROLL];}
      if (param == 4 && (P8[YAW]>9)) {P8[YAW] -= 10;}
      blinkLED(10,30,1);
    }
    if (rcData[ROLL] > MAXCHECK && valActive == 0) {
      valActive = 1;
      refreshLCD=1;
      //set val +
      if (param == 1) {P8[ROLL] += 1;P8[PITCH]=P8[ROLL];}
      if (param == 2) {I8[ROLL] += 5;I8[PITCH]=I8[ROLL];}
      if (param == 3 && (D8[ROLL]>0)) {D8[ROLL] -= 1;D8[PITCH]=D8[ROLL];}
      if (param == 4) {P8[YAW] += 10;}
      blinkLED(10,30,1);
    }
    if (rcData[ROLL] < MAXCHECK && rcData[ROLL] > MINCHECK) valActive = 0;
    
    if (rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK)
      LCD = 0;
  }
  writeParams();
  blinkLED(20,30,1);
  Serial.begin(SERIAL_COM_SPEED);
}

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      PINB |= 1<<5; //switch LEDPIN state (digital PIN 13)
      digitalWrite(BUZZERPIN,1);
      delay(wait);
      digitalWrite(BUZZERPIN,0);
    }
    delay(60);
  }
  digitalWrite(BUZZERPIN,0);
}
  
void setup() {
  Serial.begin(SERIAL_COM_SPEED);
  pinMode (LEDPIN, OUTPUT);
  pinMode (POWERPIN, OUTPUT);
  pinMode (BUZZERPIN, OUTPUT);
  digitalWrite(POWERPIN,LOW); //switch OFF WMP
  #ifndef GIMBAL
  initializeMotors();
  #endif
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  delay(200);
  digitalWrite(POWERPIN,HIGH); //switch ON WMP
  delay(100);
  initIMU();
  #if defined(TRI) || defined(BI) || defined(SERVO_TILT) || defined(GIMBAL)
    initializeServo();
  #endif
  #if defined(Y6)
    initializeSoftPWM();
  #endif
  previousTime = micros();
  #if defined(GIMBAL)
   calibratingA = 400;
  #else
    calibratingA = 0;
  #endif
  calibratingG = 400;
}


// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  uint8_t axis,i;
  int16_t error;
  float timeFactor;
  int16_t delta;
  static int16_t lastGyro[3] = {0,0,0};
  static int16_t delta1[3];
  static int16_t delta2[3];
  uint16_t maxMotor;
  static int32_t errorI[3] = {0,0,0};
  static int16_t gyroYawSmooth = 0;

  if (currentTime > (rcTime + 20000) ) { // 50Hz
    computeRC();
    if (rcData[THROTTLE] < MINCHECK) {
      errorI[ROLL] = 0;
      errorI[PITCH] = 0;
      errorI[YAW] = 0;
      rcDelayCommand++;
      if (rcData[YAW] < MINCHECK && armed == 1) {
        if (rcDelayCommand == 20) { // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
          armed = 0;
          writeAllMotors(MINCOMMAND);
        }
      } else if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          calibratingA=400;
          calibratingG=400;
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0) {
        if (rcDelayCommand == 20) {
          armed = 1;
          writeAllMotors(MINTHROTTLE);
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          atomicServoYaw = 125; //we center the yaw gyro in conf mode
          #if defined(LCD_CONF)
            configurationLoop(); //beginning LCD configuration
          #endif
          previousTime = micros();
        }
      } else {
        rcDelayCommand = 0;
      }
    }
    if (rcData[AUX1] > 1700 && nunchukPresent == 1) {
      levelModeParam = 1;
    } else {
      levelModeParam = 0;
    }
    rcTime = currentTime; 
  }

  computeIMU();
  
  // Measure loop rate just afer reading the sensors
  // meanTime and cycleTime are used to scale the PID term I and D
  // variation are caused by irregular calls to RC loop functions and interruptions
  currentTime = micros();

  if (currentTime > (neutralizeTime + 20000)) { // we calculate cycle time only is data are valid
    cycleTime = currentTime - previousTime;
    meanTime = ((uint32_t)meanTime*39 + cycleTime+20)/40;
  }
  timeFactor = (float)cycleTime/(float)meanTime;
  previousTime = currentTime;

  #ifdef BI
    static int16_t gyroRollSmooth = 0;
    gyroData[ROLL] = ((int32_t)gyroRollSmooth*10+gyroData[ROLL]+5)/11;
    gyroRollSmooth = gyroData[ROLL];
  #endif

  gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW]+1)/3;
  gyroYawSmooth = gyroData[YAW];

  //**** PITCH & ROLL & YAW PID ****
  for(axis=0;axis<3;axis++) {  
    error = rcCommand[axis]*10/P8[axis] - gyroData[axis];

    if (abs(gyroData[axis]) < 80 ) {
      errorI[axis] += error*timeFactor;
      errorI[axis] = constrain(errorI[axis],-2000,+2000); //WindUp
    } else
      errorI[axis] = 0;

    delta = (gyroData[axis] - lastGyro[axis])/timeFactor;

    axisPID[axis] =  rcCommand[axis] - gyroData[axis]*dynP8[axis]/10 -  (delta1[axis] + delta2[axis] + delta +1)* dynD8[axis] /3;
    axisPID[axis] += errorI[axis]*I8[axis]/1000;
 
    delta2[axis] = delta1[axis];
    delta1[axis] = delta;
    lastGyro[axis] = gyroData[axis];

    if (levelModeParam == 1 && axis < 2) {
      axisPID[axis]-=angle[axis]*accStrength8/25;
    }
  }

  if (armed ) {
    #ifdef BI
      motor[FRONT] = 1500 + rcCommand[THROTTLE] - axisPID[PITCH] ;
      motor[REAR]  = 1500 + rcCommand[THROTTLE] + axisPID[PITCH] ;
    #endif
    #ifdef TRI
      motor[REAR]  = 1500 + rcCommand[THROTTLE] + axisPID[PITCH]*4/3 ;
      motor[RIGHT] = 1500 + rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]*2/3 ;
      motor[LEFT]  = 1500 + rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]*2/3 ;
    #endif
    #ifdef QUADP
      motor[FRONT] = 1500 + rcCommand[THROTTLE] - axisPID[PITCH] - axisPID[YAW];
      motor[RIGHT] = 1500 + rcCommand[THROTTLE] - axisPID[ROLL]  + axisPID[YAW];
      motor[LEFT]  = 1500 + rcCommand[THROTTLE] + axisPID[ROLL]  + axisPID[YAW];
      motor[REAR]  = 1500 + rcCommand[THROTTLE] + axisPID[PITCH] - axisPID[YAW];
    #endif
    #ifdef QUADX
      motor[FRONT_L] = 1500 + rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW];
      motor[FRONT_R] = 1500 + rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW];
      motor[REAR_L]  = 1500 + rcCommand[THROTTLE] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW];
      motor[REAR_R]  = 1500 + rcCommand[THROTTLE] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW];
    #endif
    #ifdef Y6
      motor[REAR]  = 1500 + rcCommand[THROTTLE] + axisPID[PITCH]*4/3                       + axisPID[YAW];
      motor[RIGHT] = 1500 + rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]*2/3       - axisPID[YAW];
      motor[LEFT]  = 1500 + rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]*2/3       - axisPID[YAW];
      motor[UNDER_REAR]  = 1500 + rcCommand[THROTTLE] + axisPID[PITCH]*4/3                 - axisPID[YAW];
      motor[UNDER_RIGHT] = 1500 + rcCommand[THROTTLE] - axisPID[ROLL] - axisPID[PITCH]*2/3 + axisPID[YAW];
      motor[UNDER_LEFT]  = 1500 + rcCommand[THROTTLE] + axisPID[ROLL] - axisPID[PITCH]*2/3 + axisPID[YAW];
    #endif

  }
  #if defined(BI)
    servoFront     = 1500 + YAW_DIRECTION * axisPID[YAW] + axisPID[ROLL];
    servoYaw       = 1500 + YAW_DIRECTION * axisPID[YAW] - axisPID[ROLL];
    maxMotor = max(motor[REAR],motor[FRONT]);
  #elif defined(TRI)
    servoYaw       = 1500 + YAW_DIRECTION * axisPID[YAW];
    maxMotor = max(motor[0],max(motor[1],motor[2]));
  #elif defined(QUADP) || defined(QUADX)
    maxMotor = max(motor[0],max(motor[1],max(motor[2],motor[3])));
  #elif defined(Y6)
    maxMotor = max(motor[0],max(motor[1],max(motor[2],max(motor[3],max(motor[4],motor[5])))));
  #endif
  #ifndef GIMBAL
  if (maxMotor > MAXCOMMAND) { // this is a way to still have good gyro corrections if at least one motor reaches its max.
    motor[REAR]  -= maxMotor - MAXCOMMAND;
    motor[RIGHT] -= maxMotor - MAXCOMMAND;
    motor[LEFT]  -= maxMotor - MAXCOMMAND;
    #if defined(BI) || defined(QUADP) || defined(QUADX)
      motor[FRONT]  -= maxMotor - MAXCOMMAND;
    #elif defined(Y6)
      motor[UNDER_REAR]  -= maxMotor - MAXCOMMAND;
      motor[UNDER_RIGHT] -= maxMotor - MAXCOMMAND;
      motor[UNDER_LEFT]  -= maxMotor - MAXCOMMAND;
    #endif
  }
  #endif

  #if defined(BI)
    servoYaw   = constrain(servoYaw, 1020, 2000);
    servoFront = constrain(servoFront, 1020, 2000);
    atomicServoYaw = (servoYaw-1000)/4;
    atomicServoFront = (servoFront-1000)/4;
    for (i = 0; i < 2; i++) {
  #elif defined(TRI)
    servoYaw = constrain(servoYaw, 1020, 2000);
    atomicServoYaw = (servoYaw-1000)/4;
    for (i = 0; i < 3; i++) {
  #elif defined(QUADP) || defined(QUADX)
    for (i = 0; i < 4; i++) {
  #elif defined(Y6)
    for (i = 0; i < 6; i++) {
  #endif
  #ifndef GIMBAL
      motor[i] = constrain(motor[i], MINTHROTTLE, MAXCOMMAND);
      if ((rcData[THROTTLE]) < MINCHECK)
        motor[i] = MINTHROTTLE;
      if (armed == 0)
        motor[i] = MINCOMMAND;
    }
  #endif
    
  #if defined(SERVO_TILT)
    servoPitch   = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] *6 /10 , TILT_PITCH_MIN, TILT_PITCH_MAX);
    servoRoll = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] *6 /10  , TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #if defined(GIMBAL)
    servoPitch   = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] *6 /10 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servoRoll = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] *6 /10  + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)
    atomicServoPitch = (servoPitch-1000)/4;
    atomicServoRoll = (servoRoll-1000)/4;
  #endif
  
  #ifndef GIMBAL
    writeMotors();
  #endif
}

void serialCom() {
  int16_t a;
  uint8_t s[63];
  if (Serial.available()) {
    switch (Serial.read()) {
    case 'A': //arduino to GUI all data
      s[0] = 'A';
      a=accADC[ROLL];          s[2]  = a>>8&0xff; s[1]  = a;
      a=accADC[PITCH];         s[4]  = a>>8&0xff; s[3]  = a;     
      a=accADC[YAW];           s[6]  = a>>8&0xff; s[5]  = a;
      a=gyroData[ROLL];        s[8]  = a>>8&0xff; s[7]  = a;
      a=gyroData[PITCH];       s[10] = a>>8&0xff; s[9]  = a;
      a=gyroData[YAW];         s[12] = a>>8&0xff; s[11] = a;
    #if defined(BI)
      a=servoFront;            s[14] = a>>8&0xff; s[13] = a;
      a=servoYaw;              s[16] = a>>8&0xff; s[15] = a;
    #elif defined(GIMBAL)
      a=servoPitch;            s[14] = a>>8&0xff; s[13] = a;
      a=servoRoll;             s[16] = a>>8&0xff; s[15] = a;
    #else
      a=motor[RIGHT];          s[14] = a>>8&0xff; s[13] = a;
      a=motor[LEFT];           s[16] = a>>8&0xff; s[15] = a;
    #endif
      a=motor[REAR];           s[18] = a>>8&0xff; s[17] = a;
    #ifdef TRI
      a=servoYaw;              s[20] = a>>8&0xff; s[19] = a;
    #else
      a=motor[FRONT];          s[20] = a>>8&0xff; s[19] = a;
    #endif
      a=motor[UNDER_REAR];     s[22] = a>>8&0xff; s[21] = a;
      a=motor[UNDER_RIGHT];    s[24] = a>>8&0xff; s[23] = a;
      a=motor[UNDER_LEFT];     s[26] = a>>8&0xff; s[25] = a;

      a=rcHysteresis[THROTTLE];s[28] = a>>8&0xff; s[27] = a; //rc stick
      a=rcHysteresis[ROLL];    s[30] = a>>8&0xff; s[29] = a;
      a=rcHysteresis[PITCH];   s[32] = a>>8&0xff; s[31] = a;
      a=rcHysteresis[YAW];     s[34] = a>>8&0xff; s[33] = a;
      a=rcHysteresis[AUX1];    s[36] = a>>8&0xff; s[35] = a;
                               s[38] = nunchukPresent;
                               s[37] = levelModeParam;
      a=cycleTime;             s[40] = a>>8&0xff; s[39] = a;
      a=angle[ROLL];           s[42] = a>>8&0xff; s[41] = a;
      a=angle[PITCH];          s[44] = a>>8&0xff; s[43] = a;
    #if defined(TRI)
                               s[45] = 1; //multi type
    #elif defined(QUADP)
                               s[45] = 2;
    #elif defined(QUADX)
                               s[45] = 3;
    #elif defined(BI)
                               s[45] = 4;
    #elif defined(GIMBAL)
                               s[45] = 5;
    #elif defined(Y6)
                               s[45] = 6;
    #endif
      s[46] = P8[ROLL];  s[47] = I8[ROLL];  s[48] = D8[ROLL];
      s[49] = P8[PITCH]; s[50] = I8[PITCH]; s[51] = D8[PITCH];
      s[52] = P8[YAW];   s[53] = I8[YAW];   s[54] = D8[YAW];
      s[55] = rcRate8;
      s[56] = rcExpo8;
      s[57] = accStrength8;
      s[58] = rollPitchRate;
      s[59] = yawRate;
      s[60] = dynThrPID1;
      s[61] = dynThrPID2;
      s[62] = dynThrPID3;
      Serial.write(s,63);
      break;      
    case 'C': //GUI to arduino param
      while (Serial.available()<18) {}
      P8[ROLL]  = Serial.read();  I8[ROLL]  = Serial.read();  D8[ROLL]  = Serial.read();
      P8[PITCH] = Serial.read();  I8[PITCH] = Serial.read();  D8[PITCH] = Serial.read();
      P8[YAW]   = Serial.read();  I8[YAW]   = Serial.read();  D8[YAW]   = Serial.read();
      rcRate8   = Serial.read();
      rcExpo8   = Serial.read();
      accStrength8 = Serial.read();
      rollPitchRate = Serial.read();yawRate = Serial.read();Serial.read();
      dynThrPID1 = Serial.read();dynThrPID2 = Serial.read();dynThrPID3 = Serial.read();
      writeParams();
      break;
    }
  }
}

