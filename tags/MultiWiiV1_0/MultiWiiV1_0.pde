/*
MultiWiiCopter by Alexandre Dubus
radio-commande.com
September 2010     V1.0
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A

//The type of multicopter
#define TRI
//#define QUADP
//#define QUADX

#define YAW_DIRECTION 1 // if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

//#define I2C_SPEED 100000L //100kHz normal mode
#define I2C_SPEED 400000L   //400kHz fast mode

#define SERIAL_COM_SPEED 115200

#define INTERLEAVING_DELAY 3000 // interleaving delay in micro seconds between 2 readings WMP/NK in a WMP+NK config

//#define SERIAL_SUM_PPM  // for specific receiver with only one PPM sum signal, on digital PIN 2
#define PPM_ORDER         PITCH,YAW,THROTTLE,ROLL,AUX1 //For Graupner/Spektrum
//#define PPM_ORDER       PITCH,ROLL,THROTTLE,YAW,AUX1 //For Robe/Futaba (to confirm)








#include <EEPROM.h>

//PIN assignment
#define THROTTLEPIN 2
#define ROLLPIN 4
#define PITCHPIN 5
#define YAWPIN 6
#define AUX1PIN 7

//PIN for TRICOPTER and QUAD+   //QUADX equivalency 
#define REARMOTORPIN 9          //REAR_RIGHT_MOTORPIN
#define RIGHTMOTORPIN 10        //FRONT_RIGHT_MOTORPIN
#define LEFTMOTORPIN 11         //REAR_LEFT_MOTORPIN
#define FRONTMOTORPIN 3         //FRONT_LEFT_MOTORPIN

#define LEDPIN 13
#define POWERPIN 12

// alias for TRI and QUAD+      //QUADX equivalency
#define REAR 0                  //REAR_R
#define RIGHT 1                 //FRONT_R
#define LEFT 2                  //REAR_L
#define FRONT 3                 //FRONT_L

// alias QUADX equivalency
#define REAR_R REAR
#define FRONT_R RIGHT
#define REAR_L LEFT
#define FRONT_L FRONT


// alias for RC
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define AUX1 4


static uint32_t previousTime;
static uint32_t neutralizeTime; //when there is an error on I2C bus, we neutralize the values during a short time
static uint32_t currentTime;
static uint8_t nunchukPresent = 0;
static uint8_t levelModeParam = 0;  //if level mode is a activated on the radio : channel 5 value superior to 1700
static uint8_t levelModeActive = 0; //if PITCH/ROLL stick level are centered enough: level mode is activated in the stabilization loop
static uint32_t cycleTime; // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint32_t meanTime = 2000; // this is the average time of the loop: around 2ms for a WMP config and 6ms for a NK+WMP config
// to be more precise, the calibration is now done is the main loop. Calibrating decreases at each cycle up to 0, then we enter in a normal mode.
// we separate the calibration of ACC and gyro, because number of measure is not always equal.
static uint16_t calibratingA;
static uint16_t calibratingG;
static uint32_t delayLED = 200000; //5Hz if no nunchuk is present

// **************
// Wii Motion Plus I2C & gyro+nunchuk functions
// **************
static int16_t gyroData[3] = {0,0,0};
static int16_t accData[3] = {0,0,0};

static int16_t gyroZero[3] = {0,0,0};
static int16_t accZero[3] = {0,0,0};

static int16_t gyroADC[3];
static int16_t accADC[3];
static uint8_t rawADC[6];

static int16_t angle[2]; //absolute angle inclination in Deg

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
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
    neutralizeTime = micros(); //we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay after the hard reset
  }
}

void initI2C(void) {
  i2c_init();
  delay(250);
  i2c_rep_start(0xA6 + 0);//write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  delay(250);
  i2c_rep_start(0xA6 + 0);//write direction => 0
  i2c_write(0xFE); 
  i2c_write(0x05); 
  delay(250);
}

void getI2C() {
  i2c_rep_start(0xA4 + 0);//write direction => 0
  i2c_write(0x00);
  i2c_rep_start(0xA4 + 1);//read direction => 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC[i]=i2c_readAck();}
  rawADC[5]= i2c_readNak();
}

uint8_t rawIMU () {
  getI2C();
  if ( rawADC[5]&0x02 ) {// motion plus data
    gyroADC[PITCH]  = - (((rawADC[4]>>2)<<8) + rawADC[1]);
    gyroADC[ROLL]   = - (((rawADC[5]>>2)<<8) + rawADC[2]);
    gyroADC[YAW]    = - (((rawADC[3]>>2)<<8) + rawADC[0])>>3;
    return 1;
  } else { //nunchuk data
    accADC[PITCH] = - (rawADC[2]<<2) - ((rawADC[5]>>3)&0x2);
    accADC[ROLL]  =  (rawADC[3]<<2) + ((rawADC[5]>>4)&0x2);
    accADC[YAW]   = -((rawADC[4]&0xFE)<<2) - ((rawADC[5]>>5)&0x6);
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
  if (numberAccRead>30) {
    nunchukPresent = 1;
    delayLED = 50000; //20Hz is nunchuk is present
  }
  delay(10);
}

void updateIMU() {
  static int32_t g[3];
  static int32_t a[3];
  uint8_t axis;
  
  if (rawIMU()) { //gyro
    if (calibratingG>0) {
      for (axis = 0; axis < 3; axis++) {
        if (calibratingG>1) {
          if (calibratingG == 400)
            g[axis]=0;
          g[axis] +=gyroADC[axis];
          gyroADC[axis]=0;
        } else
          gyroZero[axis]=g[axis]/399;
      }
      calibratingG--;
    } else {
      gyroADC[PITCH] = gyroADC[PITCH] - gyroZero[PITCH];
      gyroADC[ROLL]  = gyroADC[ROLL]  - gyroZero[ROLL];
      gyroADC[YAW]   = gyroADC[YAW]   - gyroZero[YAW]; 
    }
    gyroADC[PITCH] = (rawADC[4]&0x02)>>1  ? gyroADC[PITCH]/10 : gyroADC[PITCH]/2 ;
    gyroADC[ROLL]  = (rawADC[3]&0x01)     ? gyroADC[ROLL]/10 : gyroADC[ROLL]/2 ;
    gyroADC[YAW]   = (rawADC[3]&0x02)>>1  ? gyroADC[YAW] : gyroADC[YAW]*5 ;
  } else { //nunchuk
    if (calibratingA>0) {
      for (axis = 0; axis < 3; axis++) {
        if (calibratingA>1) {
          if (calibratingA == 400)
            a[axis]=0;
          a[axis] +=accADC[axis];
          accADC[axis]=0;
        } else {
          accZero[axis]=a[axis]/399;
          accZero[YAW]=a[YAW]/399+200; // for nunchuk 200=1G
        }
      }
      calibratingA--;
    } else {
      accADC[PITCH] = accADC[PITCH] - accZero[PITCH];
      accADC[ROLL]  = accADC[ROLL]  - accZero[ROLL];
      accADC[YAW]   = - accADC[YAW] + accZero[YAW];
    }
  }
}

// ************************************
// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// with this algorithm, we can get absolute angles for a stable mode integration
// ************************************
void getEstimatedInclination(){
  int8_t signRzGyro;  
  float R;
  static float RxEst = 0; // init acc in stable mode
  static float RyEst = 0;
  static float RzEst = 1;
  float Axz,Ayz;           //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
  float RxAcc,RyAcc,RzAcc;         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer       
  float RxGyro,RyGyro,RzGyro;        //R obtained from last estimated value and gyro movement
  float wGyro = 50.0f; // gyro weight/smooting factor

  float atanx,atany;
  float gyroFactor;
  
  //get accelerometer readings in g, gives us RAcc vector
  RxAcc = accADC[ROLL];
  RyAcc = accADC[PITCH];
  RzAcc = accADC[YAW];

  //normalize vector (convert to a vector with same direction and with length 1)
  R = sqrt(square(RxAcc) + square(RyAcc) + square(RzAcc));
  RxAcc /= R;
  RyAcc /= R;  
  RzAcc /= R;  

  gyroFactor = meanTime/83e6; //empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  
  //evaluate R Gyro vector
  if(abs(RzEst) < 0.1f){
    //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
    //in this case skip the gyro data and just use previous estimate
    RxGyro = RxEst;
    RyGyro = RyEst;
    RzGyro = RzEst;
  }else{
    //get angles between projection of R on ZX/ZY plane and Z axis, based on last REst
    //Convert ADC value for to physical units
    //For gyro it will return  deg/ms (rate of rotation)
    atanx = atan2(RxEst,RzEst);
    atany = atan2(RyEst,RzEst);
  
    Axz = atanx + gyroADC[ROLL]  * gyroFactor;  // convert ADC value for to physical units
    Ayz = atany + gyroADC[PITCH] * gyroFactor; // and get updated angle according to gyro movement
  
    //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
    signRzGyro = ( cos(Axz) >=0 ) ? 1 : -1;

    //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
    RxGyro = sin(Axz) / sqrt( 1 + square(cos(Axz)) * square(tan(Ayz)) );
    RyGyro = sin(Ayz) / sqrt( 1 + square(cos(Ayz)) * square(tan(Axz)) );        
    RzGyro = signRzGyro * sqrt(1 - square(RxGyro) - square(RyGyro));
  }
  //combine Accelerometer and gyro readings
  RxEst = (RxAcc + wGyro* RxGyro) / (1.0 + wGyro);
  RyEst = (RyAcc + wGyro* RyGyro) / (1.0 + wGyro);
  RzEst = (RzAcc + wGyro* RzGyro) / (1.0 + wGyro);

  angle[ROLL]  =  180/PI * Axz;
  angle[PITCH] =  180/PI * Ayz;
}

void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  static int16_t gyroADCp[3] = {0,0,0};
  static int16_t gyroADCinter[3];
  static int16_t lastAccADC[3] = {0,0,0};
  static int16_t similarNumberAccData[3];
  static int16_t gyroDeviation[3];
  static float   accDataFloat[3] = {0.0,0.0,0.0};;
  static uint32_t timeInterleaveI2C,b;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. Yhis delay should be something between 2 and 3ms. It works with 2.5ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  if (nunchukPresent) {
    while((micros()-timeInterleaveI2C)<INTERLEAVING_DELAY) ; //it replaces delayMicroseconds(3000); //interleaving delay between 2 consecutive reads
    timeInterleaveI2C=micros();
    updateIMU();
    if (calibratingA == 0) getEstimatedInclination(); //getEstimatedInclination computation must last less than 3ms
    while((micros()-timeInterleaveI2C)<INTERLEAVING_DELAY) ; //it replaces delayMicroseconds(3000); //interleaving delay between 2 consecutive reads
    timeInterleaveI2C=micros();
    updateIMU();

    for (axis = 0; axis < 3; axis++) {
      gyroData[axis] = gyroADC[axis]/4;
      accDataFloat[axis] = (accDataFloat[axis]*30.0+accADC[axis])/31.0; //it is a megasmoothing of acc values
      accData[axis] = accDataFloat[axis];
    }
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
  } else {
    updateIMU();
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    delayMicroseconds(650); //empirical
    updateIMU();
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  (gyroADC[axis]+gyroADCp[axis])/8; // /4 /2
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]*2+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis];
    }
  }
  if (currentTime < (neutralizeTime + 20000)) { //we neutralize data for 20ms in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {
     gyroData[axis]=0;
     accData[axis]=0;
    }
  }
}

// *************************
// motor and servo functions
// *************************

#define MINCOMMAND 1000
#define MAXCOMMAND 1850

static int16_t axisPID[3];
static int16_t motor[4];
static int16_t servoYaw = 1500;
volatile int8_t atomicServoYaw = 125;

void writeMotors() {
  analogWrite(REARMOTORPIN, motor[REAR]/8);  // [1000;2000] => [125;250]
  analogWrite(RIGHTMOTORPIN, motor[RIGHT]/8);
  analogWrite(LEFTMOTORPIN, motor[LEFT]/8);
  #ifndef TRI
  analogWrite(FRONTMOTORPIN, motor[FRONT]/8);
  #endif
}

void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  motor[REAR] = mc;
  motor[RIGHT] = mc;
  motor[LEFT] = mc;
  #ifndef TRI
  motor[FRONT] = mc;
  #endif
  writeMotors();
}


void initializeMotors() {
  pinMode(REARMOTORPIN,OUTPUT);
  pinMode(RIGHTMOTORPIN,OUTPUT);
  pinMode(LEFTMOTORPIN,OUTPUT);
  #ifndef TRI
  pinMode(FRONTMOTORPIN,OUTPUT);
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

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

static uint8_t pinRcChannel[5] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN};
volatile uint16_t rcPinValue[8] = {0,0,1000,0,1500,1500,1500,1000};; // interval [1000;2000]
static uint16_t rcData[5] ; // interval [1000;2000]
static int16_t rcCommand[5] = {0,0,0,0,0}; // interval [-500;+500]
static int16_t rcHysteresis[5] ;
static int16_t rcData4Values[5][4];

static float rcRate;
static float rcExpo;

// ***PPM SUM SIGNAL***
static uint8_t rcChannel[5] = {PPM_ORDER};
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
    PCMSK2 |= 1<<2 | 1<<4 | 1<<5 | 1<<6 | 1<<7; 
    PCICR   = 1<<2; // PCINT activated only for [D0-D7] port
  #else
    attachInterrupt(0, rxInt, RISING);
  #endif
}

#ifndef SERIAL_SUM_PPM
ISR(PCINT2_vect,ISR_NOBLOCK) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
  uint8_t mask;
  uint8_t pind;
  uint16_t cTime;
  static uint16_t edgeTime[5];
  static uint8_t PCintLast;

  cTime = micros();         //micros() return a uint32_t, but it is not usefull to keep the whole bits
  pind = PIND;              // PIND indicates the state of each PIN for the arduino port dealing with [D0-D7] digital pins (8 bits variable)
  mask = pind ^ PCintLast;  // doing a ^ between the current interruption and the last one indicates wich pin changed
  PCintLast = pind;         //we memorize the current state of all PINs [D0-D7]
  // mask is pins [D0-D7] that have changed
  // chan = pin sequence of the port. chan begins at D2 and ends at D7
  if (mask & 1<<2) {          //indicates the bit 2 of the arduino port [D0-D7], that is to say digital pin 2, if 1 => this pin has just changed
    if (!(pind & 1<<2)) {     //indicates if the bit 2 of the arduino port [D0-D7] is not at a high state (so that we match here only descending PPM pulse)
      if ((cTime - edgeTime[0])<2500) if ((cTime - edgeTime[0])>900) rcPinValue[2] = cTime - edgeTime[0]; // just a verification: the value must be in the range [1000;2000] + some margin
    } else edgeTime[0] = cTime;    // if the bit 2 of the arduino port [D0-D7] is at a high state (ascending PPM pulse), we memorize the time
  } 
  if (mask & 1<<4) { //same principle for other channels   // avoiding a for() is more than twice faster, and it's important to minimize execution time in ISR
    if (!(pind & 1<<4)) {
      if ((cTime - edgeTime[1])<2200) if ((cTime - edgeTime[1])>900) rcPinValue[4] = cTime - edgeTime[1];
    } else edgeTime[1] = cTime;
  }
  if (mask & 1<<5) {
    if (!(pind & 1<<5)) {
      if ((cTime - edgeTime[2])<2200) if ((cTime - edgeTime[2])>900) rcPinValue[5] = cTime - edgeTime[2];
    } else edgeTime[2] = cTime;
  }
  if (mask & 1<<6) {
    if (!(pind & 1<<6)) {
      if ((cTime - edgeTime[3])<2200) if ((cTime - edgeTime[3])>900) rcPinValue[6] = cTime - edgeTime[3];
    } else edgeTime[3] = cTime;
  }
  if (mask & 1<<7) {
    if (!(pind & 1<<7)) {
      if ((cTime - edgeTime[4])<2200) if ((cTime - edgeTime[4])>900) rcPinValue[7] = cTime - edgeTime[4];
    } else edgeTime[4] = cTime;
  }
}

#else 
void rxInt() {
  uint32_t now;
  uint16_t diff;
  static uint32_t last = 0;
  static uint8_t currentChannel = 0;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>5000) currentChannel = 0;
  else {
    if(diff<=2200 && diff>=900) rcValue[currentChannel] = diff;
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
    rcData[chan] /= 4;
    if ( rcData[chan] < rcHysteresis[chan] -4)
      rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +4)
      rcHysteresis[chan] = rcData[chan]-2;
    rcCommand[chan] = rcHysteresis[chan]-1500;  
  }
  rcCommand[ROLL] = rcRate * rcCommand[ROLL] * (1-rcExpo + rcExpo*rcCommand[ROLL]*rcCommand[ROLL]*1/250000.0);
  rcCommand[PITCH] = rcRate * rcCommand[PITCH] * (1-rcExpo + rcExpo*rcCommand[PITCH]*rcCommand[PITCH]*1/250000.0);
  
  rcCommand[THROTTLE] = (MAXCOMMAND-MINTHROTTLE)/(2000.0-1100.0) * (rcCommand[THROTTLE]+400) + MINTHROTTLE-1500;
}

// *************
// PID functions
// *************
static float P[3], I[3], D[3];
static float accStrength;

// ****************
// EEPROM functions
// ****************
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

static float MAGICALNUMBER;

void readEEPROM() {
  MAGICALNUMBER = readFloat(MAGICALNUMBER_ADR);
  P[ROLL] = readFloat(PROLL_ADR);
  I[ROLL] = readFloat(IROLL_ADR);
  D[ROLL] = readFloat(DROLL_ADR);
  P[PITCH] = readFloat(PPITCH_ADR);
  I[PITCH] = readFloat(IPITCH_ADR);
  D[PITCH] = readFloat(DPITCH_ADR);
  P[YAW]  = readFloat(PYAW_ADR);
  I[YAW]  = readFloat(IYAW_ADR);
  D[YAW]  = readFloat(DYAW_ADR);
  rcRate  = readFloat(RCRATE_ADR);
  rcExpo  = readFloat(RCEXPO_ADR);
  accStrength = readFloat(ACCSTRENGTH_ADR);
}

void writeParams() {
  writeFloat(MAGICALNUMBER, MAGICALNUMBER_ADR);
  writeFloat(P[ROLL], PROLL_ADR);
  writeFloat(I[ROLL], IROLL_ADR);
  writeFloat(D[ROLL], DROLL_ADR);
  writeFloat(P[PITCH], PPITCH_ADR);
  writeFloat(I[PITCH], IPITCH_ADR);
  writeFloat(D[PITCH], DPITCH_ADR);
  writeFloat(P[YAW],  PYAW_ADR);
  writeFloat(I[YAW],  IYAW_ADR);
  writeFloat(D[YAW],  DYAW_ADR);
  writeFloat(rcRate,  RCRATE_ADR);
  writeFloat(rcExpo,  RCEXPO_ADR);
  writeFloat(accStrength,  ACCSTRENGTH_ADR);
}

void checkFirstTime() {
  if ( MAGICALNUMBER != 713705 ) {
    MAGICALNUMBER = 713705;
    P[ROLL] = 4.0; I[ROLL] = 0.03; D[ROLL] = -13;
    P[PITCH] = 4.0; I[PITCH] = 0.03; D[PITCH] = -13;
    P[YAW]  = 8.0; I[YAW]  = 0.0;  D[YAW]  = 0.0;
    rcRate = 0.9;
    rcExpo = 0.65;
    accStrength = 1.0;
    writeParams();
  }
}

float readFloat(uint8_t add) {
  static float val;
  for(uint8_t i=0;i<4;i++)
    ((uint8_t*)&val)[i]=EEPROM.read(add+i);
  return val;
}

void writeFloat(float val, uint8_t add) {
  for(uint8_t i=0;i<4;i++)
    EEPROM.write(add+i,((uint8_t*)&val)[i]);
}


// *****************
// LCD configuration
// *****************

// 1000000 / 9600  = 104 microseconds at 9600 baud.
// the minimum supported value is 96 for sparkfun serial LCD
// we set it at the minimum to take some margin with the running interrupts
void LCDprint(uint8_t i) {
  uint16_t bitDelay = 96;

  digitalWrite(0, LOW);
  delayMicroseconds(bitDelay);
  for (uint8_t mask = 0x01; mask; mask <<= 1) {
    if (i & mask) // choose bit
      digitalWrite(0,HIGH); // send 1
    else
      digitalWrite(0,LOW); // send 1
    delayMicroseconds(bitDelay);
  }
  digitalWrite(0, HIGH);
  delayMicroseconds(bitDelay);
}

void LCDprintChar(const char *s) {
    while (*s) LCDprint(*s++);
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

  Serial.end();
  blinkLED(20,30,1);
  //init LCD
  pinMode(0, OUTPUT); //TX PIN for LCD = Arduino RX PIN (more convenient to connect a servo plug on arduino pro mini)
  LCDprint(0xFE);LCDprint(0x0D); //cursor blink mode

  param = 1;
  while (LCD == 1) {
    if (refreshLCD == 1) {
      if (param < 4) {
        if (param == 1) {sprintf(line1," *P*    I     D "); posLCD = 130;}
        if (param == 2) {sprintf(line1,"  P    *I*    D "); posLCD = 136;}
        if (param == 3) {sprintf(line1,"  P     I    *D*"); posLCD = 142;}
        sprintf(line2,"%2d.%1d  0.%03d  -%2d",(uint8_t)P[ROLL],(uint8_t)((P[ROLL]-(uint8_t)P[ROLL])*10),(uint16_t)(I[ROLL]*1000.0+1),(uint8_t)(-D[ROLL]));
      }
      if (param == 4) {
        sprintf(line2,"%2d.%1d             ",(uint8_t)P[YAW],(uint8_t)((P[YAW]-(uint8_t)P[YAW])*10));
        sprintf(line1,"*P YAW*        "); posLCD = 130;
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
      if (param < 4) {blinkLED(10,20,param);}
    }
    if (rcData[PITCH] > MAXCHECK && paramActive == 0) {
      paramActive = 1;
      refreshLCD=1;
      param--; if (param<1) param=1;
      if (param < 4) {blinkLED(10,20,param);}
    }
    if (rcData[PITCH] < MAXCHECK && rcData[PITCH] > MINCHECK)  paramActive = 0;

    //+ or - param with low and high roll
    if (rcData[ROLL] < MINCHECK && valActive == 0) {
      valActive = 1;
      refreshLCD=1;
      //set val -
      if (param == 1) {P[ROLL] -= 0.1; if (P[ROLL]<0) P[ROLL]=0.0;writeFloat(P[ROLL],PROLL_ADR);P[PITCH]=P[ROLL];writeFloat(P[PITCH],PPITCH_ADR);}
      if (param == 2) {I[ROLL] -= 0.005; if (I[ROLL]<0) I[ROLL]=0.0;writeFloat(I[ROLL],IROLL_ADR);I[PITCH]=I[ROLL];writeFloat(I[PITCH],IPITCH_ADR);}
      if (param == 3) {D[ROLL] -= 1; writeFloat(D[ROLL], DROLL_ADR);D[PITCH]=D[ROLL];writeFloat(D[PITCH], DPITCH_ADR);}
      if (param == 4) {P[YAW] -= 1; if (P[YAW]<0) P[YAW]=0.0;writeFloat(P[YAW],PYAW_ADR);}
      blinkLED(10,30,1);
    }
    if (rcData[ROLL] > MAXCHECK && valActive == 0) {
      valActive = 1;
      refreshLCD=1;
      //set val +
      if (param == 1) {P[ROLL] += 0.1;writeFloat(P[ROLL],PROLL_ADR);P[PITCH]=P[ROLL];writeFloat(P[PITCH],PPITCH_ADR);}
      if (param == 2) {I[ROLL] += 0.005;writeFloat(I[ROLL],IROLL_ADR);I[PITCH]=I[ROLL];writeFloat(I[PITCH],IPITCH_ADR);}
      if (param == 3) {D[ROLL] += 1; if (D[ROLL]>0) D[ROLL]=0;writeFloat(D[ROLL],DROLL_ADR);D[PITCH]=D[ROLL];writeFloat(D[PITCH], DPITCH_ADR);}
      if (param == 4) {P[YAW] += 1;writeFloat(P[YAW],PYAW_ADR);}
      blinkLED(10,30,1);
    }
    if (rcData[ROLL] < MAXCHECK && rcData[ROLL] > MINCHECK) valActive = 0;
    
    if (rcData[YAW] < MINCHECK && rcData[PITCH] > MAXCHECK)
      LCD = 0;
  }
  blinkLED(20,30,1);
  Serial.begin(SERIAL_COM_SPEED);
}

void blinkLED(uint8_t num, uint8_t wait,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      digitalWrite(LEDPIN, HIGH); delay(wait);
      digitalWrite(LEDPIN, LOW); delay(wait);
    }
    delay(60);
  }
}
  
void setup() {
  Serial.begin(SERIAL_COM_SPEED);
  pinMode (LEDPIN, OUTPUT);
  pinMode (POWERPIN, OUTPUT);
  digitalWrite(POWERPIN,LOW); //switch OFF WMP power
  initializeMotors();
  readEEPROM();
  checkFirstTime();
  configureReceiver();
  delay(500);
  digitalWrite(POWERPIN,HIGH); //switch ON WMP power
  delay(100);
  initIMU();
  digitalWrite(LEDPIN, HIGH);
  #ifdef TRI
    initializeServo();
  #endif
  previousTime = micros();
  calibratingA = 400;
  calibratingG = 400;
}


// ******** Main Loop *********
void loop () {
  uint8_t chan,i,axis;
  uint32_t serialTime;
  static uint32_t rcTime;
  static uint8_t armed = 0;
  float error;
  float dTerm;
  float timeFactor;
  float windUp = 1000.0;
  float tmp;
  static float errorI[3];
  static float lastGyro[3];
  static float delta1[3];
  static float delta2[3];
  uint16_t maxMotor;
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or cut the motors
  static uint32_t LEDTime;
  static uint8_t statusLED = 1;

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
          blinkLED(10,100,1);
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0) {
        if (rcDelayCommand == 20) {
          armed = 1;
          writeAllMotors(MINTHROTTLE);
        }
      } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
        if (rcDelayCommand == 20) {
          configurationLoop(); //beginning LCD configuration
          meanTime = 2000;
          previousTime = micros();
        }
      } else {
        rcDelayCommand = 0;
      }
    }
    if (rcData[AUX1] > 1700 && nunchukPresent == 1) {
      levelModeParam = 1;
      if ( rcData[PITCH]>1350 &&  rcData[PITCH]<1650 && rcData[ROLL]>1350 &&  rcData[ROLL]<1650 ) levelModeActive = 1;
      else levelModeActive = 0;
    } else {
      levelModeParam = 0;
      levelModeActive = 0;
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
    meanTime = (39*meanTime + cycleTime)/40;
  }
  timeFactor = (float)cycleTime/(float)meanTime;
  previousTime = currentTime;

  //**** PITCH & ROLL & YAW PID ****
  for(axis=0;axis<3;axis++) {
    error = rcCommand[axis]/P[axis] - gyroData[axis];
  
    if (gyroData[axis] < 80 && gyroData[axis] > -80) {
      errorI[axis] += error*timeFactor;
      if (errorI[axis] < -windUp) errorI[axis] = -windUp;
      else if (errorI[axis] > windUp) errorI[axis] = windUp;
    } else
      errorI[axis] = 0.0;

    tmp = (gyroData[axis] - lastGyro[axis])/timeFactor;
    dTerm = D[axis] * (delta1[axis] + delta2[axis] + tmp)/3.0;
    delta2[axis] = delta1[axis];
    delta1[axis] = tmp;
    lastGyro[axis] = gyroData[axis];

    axisPID[axis] = P[axis] * error + I[axis] * errorI[axis] + dTerm;
    
    if (levelModeActive == 1 && axis < 2) {
      axisPID[axis]-=accStrength*angle[axis];
    }
  }

  if (armed ) {
    
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

  }
  #ifdef TRI
    servoYaw     = 1500 + YAW_DIRECTION * axisPID[YAW];
    maxMotor = max(motor[REAR],max(motor[RIGHT],motor[LEFT]));
  #else
    maxMotor = max(motor[REAR],max(motor[RIGHT],max(motor[LEFT],motor[FRONT])));
  #endif
  if (maxMotor > MAXCOMMAND) { // this is a way to still have good gyro corrections if at least one motor reaches its max.
    motor[REAR]  -= maxMotor - MAXCOMMAND;
    motor[RIGHT] -= maxMotor - MAXCOMMAND;
    motor[LEFT]  -= maxMotor - MAXCOMMAND;
    #ifndef TRI
      motor[FRONT]  -= maxMotor - MAXCOMMAND;
    #endif
  }
  #ifdef TRI
    servoYaw = constrain(servoYaw, 1020, 2000);
    atomicServoYaw = (servoYaw-1000)/4;
    for (i = REAR; i < 3; i++) {
  #else
    for (i = REAR; i < 4; i++) {
  #endif
      motor[i] = constrain(motor[i], MINTHROTTLE, MAXCOMMAND);
      if ((rcCommand[THROTTLE]+1500) < MINCHECK)
        motor[i] = MINTHROTTLE;
      if (armed == 0)
        motor[i] = MINCOMMAND;
    }

  writeMotors();
  
  if (currentTime > (serialTime + 20000)) { // 50Hz
    serialCom();
    serialTime = currentTime;
  }
  
  if (currentTime > (LEDTime + delayLED) && (calibratingA == 0 || calibratingA == 400) ) { // 5Hz or 20Hz
    statusLED = 1- statusLED;
    digitalWrite(LEDPIN,statusLED);
    LEDTime = currentTime;
  }
}

void serialCom() {
  if (Serial.available()) {
    switch (Serial.read()) {
    case 'A': //all data
      int a;
      uint8_t s[40];
      s[0] = 'A';
      a=accData[ROLL];         s[2]  = a>>8&0xff; s[1]  = a;
      a=accData[PITCH];        s[4]  = a>>8&0xff; s[3]  = a;     
      a=accData[YAW];          s[6]  = a>>8&0xff; s[5]  = a;
      a=gyroData[ROLL];        s[8]  = a>>8&0xff; s[7]  = a;
      a=gyroData[PITCH];       s[10] = a>>8&0xff; s[9]  = a;
      a=gyroData[YAW];         s[12] = a>>8&0xff; s[11] = a;
      a=motor[RIGHT];          s[14] = a>>8&0xff; s[13] = a; //motor & servo
      a=motor[LEFT];           s[16] = a>>8&0xff; s[15] = a;
      a=motor[REAR];           s[18] = a>>8&0xff; s[17] = a;
      
      #ifdef TRI
      a=servoYaw;              s[20] = a>>8&0xff; s[19] = a;
      #else
      a=motor[FRONT];          s[20] = a>>8&0xff; s[19] = a;
      #endif
      
      a=rcHysteresis[THROTTLE];s[22] = a>>8&0xff; s[21] = a; //rc stick
      a=rcHysteresis[ROLL];    s[24] = a>>8&0xff; s[23] = a;
      a=rcHysteresis[PITCH];   s[26] = a>>8&0xff; s[25] = a;
      a=rcHysteresis[YAW];     s[28] = a>>8&0xff; s[27] = a;
      a=rcHysteresis[AUX1];    s[30] = a>>8&0xff; s[29] = a;
                               s[32] = nunchukPresent;
                               s[31] = levelModeParam;
      a=cycleTime;             s[34] = a>>8&0xff; s[33] = a;
      a=angle[ROLL];           s[36] = a>>8&0xff; s[35] = a;
      a=angle[PITCH];          s[38] = a>>8&0xff; s[37] = a;
      #ifdef TRI
                               s[39] = 1; //multi type
      #endif
      #ifdef QUADP
                               s[39] = 2;
      #endif
      #ifdef QUADX
                               s[39] = 3;      
      #endif
      Serial.write(s,40);
      break;
    case 'B': //arduino to GUI param    
      Serial.write('B');
      for(uint8_t i=0;i<3;i++) {
        Serial.print(P[i]);Serial.write(',');
        Serial.print(I[i]*10);Serial.write(',');
        Serial.print(D[i]);Serial.write(',');
      }
      Serial.print(rcRate*10.0);Serial.write(',');
      Serial.print(rcExpo*10.0);Serial.write(',');
      Serial.print(accStrength);Serial.write(',');
      Serial.write(';');
      break;
    case 'C': //GUI to arduino param
      P[ROLL] = readSerialFloat(); I[ROLL] = readSerialFloat(); D[ROLL] = readSerialFloat();
      P[PITCH] = readSerialFloat(); I[PITCH] = readSerialFloat(); D[PITCH] = readSerialFloat();
      P[YAW] = readSerialFloat();  I[YAW] = readSerialFloat();  D[YAW] = readSerialFloat();
      rcRate = readSerialFloat();  rcExpo = readSerialFloat(); accStrength = readSerialFloat();
      writeParams();
      break;
    }
  }
}

float readSerialFloat() {
  uint8_t index = 0;
  char data[32] = "";
  do {
    while (Serial.available() == 0) {}
    data[index++] = Serial.read();
  }  while ((data[index-1] != ';') && (index < 32));
  return atof(data);
}
