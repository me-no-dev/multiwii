// ***************************
// board orientation and setup
// ***************************

//default board orientation (match FFIMU default orientation)
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}

//default I2C address (match FFIMU choice)
#define ADXL345_ADDRESS 0x3A
#define BMA180_ADDRESS 0x80
//#define BMA180_ADDRESS 0x82
#define ITG3200_ADDRESS 0XD0
//#define ITG3200_ADDRESS 0XD2 // (sparkun BB board)

uint8_t rawADC[6];

// *********************
// I2C general functions
// *********************

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;        // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;  // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWSTO); // send REPEAT START condition
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
  if ( TW_STATUS  == 0xF8) { //TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
    // WMP does not respond anymore => we do a hard reset. I did not find another way to solve it. It takes only 13ms to reset and init to WMP or WMP+NK
    TWCR = 0;
    POWERPIN_OFF //switch OFF WMP
    delay(1);  
    POWERPIN_ON  //switch ON WMP
    delay(10);
    #if defined(GYRO)
    #else
      WMP_init(0);
    #endif
    neutralizeTime = micros(); //we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay after the hard reset
  }
}

void getSixRawADC(uint8_t add) {
  i2c_rep_start(add );  // I2C read direction => add includes + 1
  for(uint8_t i = 0; i < 5; i++) {
    rawADC[i]=i2c_readAck();}
  rawADC[5]= i2c_readNak();
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;
  
  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG>1) {
        if (calibratingG == 400) g[axis]=0;
        g[axis] +=gyroADC[axis];
        gyroADC[axis]=0;
        gyroZero[axis]=0;
      } else {
        gyroZero[axis]=(g[axis]+200)/399;
        blinkLED(10,15,1+3*nunchukPresent);
      }
    }
    calibratingG--;
  }
  //anti gyro glitch, limit the variation between two consecutive readings
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-100,previousGyroADC[axis]+100);
    previousGyroADC[axis] = gyroADC[axis];
  }
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    if (calibratingA>1) {
      for (uint8_t axis = 0; axis < 3; axis++) {
        if (calibratingA == 400) a[axis]=0;
        a[axis] +=accADC[axis];
        accADC[axis]=0;
        accZero[axis]=0;
      }
    } else {
      accZero[ROLL]  = (a[ROLL]+200)/399;
      accZero[PITCH] = (a[PITCH]+200)/399;
      accZero[YAW]   = (a[YAW]+200)/399-acc_1G; // for nunchuk 200=1G
      writeParams(); // write accZero in EEPROM
    }
    calibratingA--;
  }
  accADC[ROLL]  -=  accZero[ROLL] ;
  accADC[PITCH] -=  accZero[PITCH];
  accADC[YAW]   -=  accZero[YAW] ;
}


// **************************
// I2C Barometer BOSCH BMP085
// **************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)

// sensor registers from the BOSCH BMP085 datasheet
#if defined(BMP085)
static int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;
static uint16_t  ac4,ac5,ac6;

static uint16_t ut; //uncompensated T
static uint32_t up; //uncompensated P
int32_t temperature = 0;
int32_t pressure = 0;
int16_t altitude = 0;
static int16_t altitudeZero;
static int16_t altitudeHold;

void  Baro_init() {
  delay(10);
  ac1 = i2c_BMP085_readIntRegister(0xAA);
  ac2 = i2c_BMP085_readIntRegister(0xAC);
  ac3 = i2c_BMP085_readIntRegister(0xAE);
  ac4 = i2c_BMP085_readIntRegister(0xB0);
  ac5 = i2c_BMP085_readIntRegister(0xB2);
  ac6 = i2c_BMP085_readIntRegister(0xB4);
  b1  = i2c_BMP085_readIntRegister(0xB6);
  b2  = i2c_BMP085_readIntRegister(0xB8);
  mb  = i2c_BMP085_readIntRegister(0xBA);
  mc  = i2c_BMP085_readIntRegister(0xBC);
  md  = i2c_BMP085_readIntRegister(0xBE);
  
  baroPresent = 1;

  altitudeZero = 0;
  for (uint8_t i=0;i<7;i++) {
    Baro_update();
    delay(35);
  }
  altitudeZero = altitude;
}

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(unsigned char r) {
  uint8_t msb, lsb;
  
  i2c_rep_start(0xEE + 0);
  i2c_write(r);
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((int16_t)msb<<8) | ((int16_t)lsb));
}

// read uncompensated temperature value: send command first
void i2c_BMP085_readUT_Command() {
  i2c_rep_start(0xEE + 0);
  i2c_write(0xf4);
  i2c_write(0x2e);
  i2c_rep_start(0xEE + 0);
  i2c_write(0xF6);
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
uint16_t i2c_BMP085_readUT_Result() {
  uint8_t msb, lsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb=i2c_readAck();
  lsb=i2c_readNak();
  return (((uint16_t)msb<<8) | ((uint16_t)lsb));
}

// read uncompensated pressure value: send command first
void i2c_BMP085_readUP_Command () {
  i2c_rep_start(0xEE + 0);
  i2c_write(0xf4);
  i2c_write(0xf4); //control register value for oversampling setting 3
  i2c_rep_start(0xEE + 0); //I2C write direction => 0
  i2c_write(0xf6);
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
uint32_t i2c_BMP085_readUP_Result () {
  uint8_t msb, lsb, xlsb;
  i2c_rep_start(0xEE + 1);//I2C read direction => 1
  msb = i2c_readAck();
  lsb = i2c_readAck();
  xlsb = i2c_readNak();
  return (((uint32_t)msb<<16) | ((uint32_t)lsb<<8) | ((uint32_t)xlsb)) >>5;
}

// deduction of true temperature and pressure from sensor, code is described in the BMP085 specs
void i2c_BMP085_CompensatedSensor() {
  int32_t x1, x2, x3, b3, b5, b6, p;
  uint32_t b4, b7;
  uint16_t a,b;

  //calculate true temperature
  x1 = (int32_t)(ut - ac6) * ac5 >> 15;
  x2 = ((int32_t) mc << 11) / (x1 + md);
  b5 = x1 + x2;
  temperature = (b5 + 8) >> 4;
  //calculate true pressure
  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = ( ( ((int32_t) ac1 * 4 + x3)<<3 ) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = (up - b3) * (50000 >> 3);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure = p + ((x1 + x2 + 3791) >> 4);
}

//in a whole cycle: we read temperature one time and pressure 5 times
void Baro_update() {
  static uint32_t t;
  static uint8_t state1 =0,state2 = 0;

  if ( (micros()-t )  < 40000 ) return; //each read is spaced by 50ms, the spec advises at least 25.5ms
  t = micros();
  
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (state1 == 0) { 
    if (state2 == 0) {
      i2c_BMP085_readUT_Command();
      state2=1;
    } else {
      ut = i2c_BMP085_readUT_Result();
      state2=0;
      state1=1; //5
    }
  } else {
    if (state2 == 0) {
      i2c_BMP085_readUP_Command();
      state2=1;
    } else {
      up = i2c_BMP085_readUP_Result();
      state2=0;
      state1--;
      i2c_BMP085_CompensatedSensor();
      altitude = (1.0 - pow(float(pressure)/101325.0, 0.190295)) * 443300 - altitudeZero; // altitude in decimeter from starting point
      if ( abs(altitude-altitudeSmooth) < 100 ) //avoid altitude spike
        altitudeSmooth = (altitudeSmooth*3+altitude+2)/4;
    }
  }
}
#endif


// **************************
// I2C Accelerometer ADXL345 
// **************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
#if defined(ADXL345)
void ACC_init () {
  delay(10);
  i2c_rep_start(ADXL345_ADDRESS+0);   // I2C write direction
  i2c_write(0x2D);                    // register 2D Power CTRL
  i2c_write(1<<3);                    // Set measure bit 3 on
  i2c_rep_start(ADXL345_ADDRESS+0);   // I2C write direction 
  i2c_write(0x31);                    // DATA_FORMAT register
  i2c_write(0x0B);                    // Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_rep_start(ADXL345_ADDRESS+0);   // I2C write direction 
  i2c_write(0x2C);                    // BW_RATE
  i2c_write(8+2+1);                   // 200Hz sampling (see table 5 of the spec)

  acc_1G = 250;
  accPresent = 1;
}

void ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  i2c_rep_start(ADXL345_ADDRESS);     // I2C write direction
  i2c_write(0x32);         // Start multiple read at reg 0x32 ADX
  getSixRawADC(ADXL345_ADDRESS+1);

  ACC_ORIENTATION( - ((rawADC[3]<<8) | rawADC[2]) ,
                     ((rawADC[1]<<8) | rawADC[0]) ,
                     ((rawADC[5]<<8) | rawADC[4]) );
  ACC_Common();
}
#endif


// **************************
// contribution from opie11 (rc-grooups)
// I2C Accelerometer BMA180
// **************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
#if defined(BMA180)
void ACC_init () {
  delay(10);
  i2c_rep_start(BMA180_ADDRESS+0);   // I2C write direction 
  i2c_write(0x0D);                   // ctrl_reg0
  i2c_write(1<<4);                   // Set bit 4 to 1 to enable writing
  i2c_rep_start(BMA180_ADDRESS+0);       
  i2c_write(0x35);          
  i2c_write(3<<1);                   // range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
  i2c_rep_start(BMA180_ADDRESS+0);
  i2c_write(0x20);                   // bw_tcs reg: bits 4-7 to set bw
  i2c_write(0<<4);                   // bw to 10Hz (low pass filter)

  acc_1G = 273;
  accPresent = 1;
}

void ACC_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2;  // Optional line.  Sensor is good for it in the spec.
  i2c_rep_start(BMA180_ADDRESS);     // I2C write direction
  i2c_write(0x02);         // Start multiple read at reg 0x02 acc_x_lsb
  getSixRawADC(BMA180_ADDRESS+1);

  ACC_ORIENTATION(  - (((rawADC[1]<<8) | (rawADC[0]))>>2)/10 ,
                    - (((rawADC[3]<<8) | (rawADC[2]))>>2)/10 ,
                      (((rawADC[5]<<8) | (rawADC[4]))>>2)/10 );
  ACC_Common();
}
#endif

// **************
// contribution from Point65 and mgros (rc-grooups)
// BMA020 I2C
// **************
// I2C adress: 0x70 (8bit)
#if defined(BMA020)
void ACC_init(){
  byte control;
  
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x15);         // 
  i2c_write(0x80);         // Write B10000000 at 0x15 init BMA020

  i2c_rep_start(0x70);     // 
  i2c_write(0x14);         //  
  i2c_write(0x71);         // 
  i2c_rep_start(0x71);     //
  control = i2c_readNak();
 
  control = control >> 5;  //ensure the value of three fist bits of reg 0x14 see BMA020 documentation page 9
  control = control << 2;
  control = control | 0x00; //Range 2G 00
  control = control << 3;
  control = control | 0x00; //Bandwidth 25 Hz 000
 
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x14);         // Start multiple read at reg 0x32 ADX
  i2c_write(control);

  acc_1G = 240;
  accPresent = 1;
}

void ACC_getADC(){
  TWBR = ((16000000L / 400000L) - 16) / 2;
  i2c_rep_start(0x70);     // I2C write direction
  i2c_write(0x02);         // Start multiple read at reg 0x32 ADX
  i2c_write(0x71);  
  getSixRawADC(0x71);

  ACC_ORIENTATION(  (((rawADC[1])<<8) | ((rawADC[0]>>1)<<1))/64 ,
                    (((rawADC[3])<<8) | ((rawADC[2]>>1)<<1))/64 ,
                    (((rawADC[5])<<8) | ((rawADC[4]>>1)<<1))/64 );
  ACC_Common();
}
#endif

// **************************
// standalone I2C Nunchuk
// **************************
#if defined(NUNCHACK)
void ACC_init() {
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0xFB); 
  i2c_write(0x00); 
  delay(250);
  accPresent = 1;
}

void ACC_getADC() {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  getSixRawADC(0xA4 + 1);

  ACC_ORIENTATION(  ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) ) ,
                  - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) ) ,
                    ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) ));
  ACC_Common();
}
#endif

// **************************
// ADC ACC
// **************************
#if defined(ADCACC)
void ACC_init(){
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  
  acc_1G = 75;
  accPresent = 1;  
}

void ACC_getADC() {
  ACC_ORIENTATION( -analogRead(A1) ,
                   -analogRead(A2) ,
                    analogRead(A3) );
  ACC_Common();
}
#endif

// **************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// **************************
#if defined(L3G4200D)
void Gyro_init() {
  delay(100);
  i2c_rep_start(0XD2+0);      // CTRL_REG1
  i2c_write(0x20);            // 400Hz ODR, 20hz filter, run!
  i2c_write(0x8F); 
  i2c_rep_start(0XD2+0);      // CTRL_REG5
  i2c_write(0x24);            // low pass filter enable
  i2c_write(0x02);
  
  gyroPresent = 1;  
}

void Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0XD2);     // I2C write direction
  i2c_write(0x80 | 0x28);  // Start multiple read
  getSixRawADC(0XD2 +1);

  GYRO_ORIENTATION(  ((rawADC[1]<<8) | rawADC[0])/20  ,
                     ((rawADC[3]<<8) | rawADC[2])/20  ,
                    -((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}
#endif

// **************************
// I2C Gyroscope ITG3200 
// **************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
#if defined(ITG3200)
void Gyro_init() {
  delay(100);
  i2c_rep_start(ITG3200_ADDRESS+0);  // I2C write direction 
  i2c_write(0x3E);                   // Power Management register
  i2c_write(0x80);                   //   reset device
  delay(5);
  i2c_rep_start(ITG3200_ADDRESS+0);  // I2C write direction 
  i2c_write(0x15);                   // register Sample Rate Divider
  i2c_write(0x7);                    //   7: 8000Hz/(7+1) = 1000Hz . more than twice the need
  delay(5);
  i2c_rep_start(ITG3200_ADDRESS+0);  // I2C write direction 
  i2c_write(0x16);                   // register DLPF_CFG - low pass filter configuration & sample rate
  i2c_write(0x18);                   //   256Hz Low Pass Filter Bandwidth - Internal Sample Rate 8kHz
  delay(5);
  i2c_rep_start(ITG3200_ADDRESS+0);  // I2C write direction 
  i2c_write(0x3E);                   // Power Management register
  i2c_write(0x03);                   //   PLL with Z Gyro reference
  delay(100);
  gyroPresent = 1;  
}

void Gyro_getADC () {
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(ITG3200_ADDRESS);     // I2C write direction
  i2c_write(0X1D);         // Start multiple read
  getSixRawADC(ITG3200_ADDRESS +1);
  
  GYRO_ORIENTATION(  + ( ((rawADC[2]<<8) | rawADC[3])/3 ) ,
                     - ( ((rawADC[0]<<8) | rawADC[1])/3 ) ,
                     - ( ((rawADC[4]<<8) | rawADC[5])/3 ) );
  GYRO_Common();
}
#endif


// **************************
// I2C Compass HMC5843 & HMC5883
// **************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)

static uint8_t  calibratingM = 0;

#if defined(MAG)
void Mag_init() { 
  delay(100);
  i2c_rep_start(0X3C+0);      // I2C write direction 
  i2c_write(0x02);            // Write to Mode register
  i2c_write(0x00);            //   Continuous-Conversion Mode
  magPresent = 1;  
}

void Mag_getADC() {
  static uint32_t t,tCal = 0;
  uint8_t axis;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];

  if ( (micros()-t )  < 100000 ) return; //each read is spaced by 100ms
  t = micros();
  TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_rep_start(0X3C);     // I2C write direction
  i2c_write(0X03);         // Start multiple read
  getSixRawADC(0X3C +1);

  #if defined(HMC5843)
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[2]<<8) | rawADC[3]) ,
                    -((rawADC[4]<<8) | rawADC[5]) );
  #endif
  #if defined (HMC5883)
    MAG_ORIENTATION( ((rawADC[4]<<8) | rawADC[5]) ,
                    -((rawADC[0]<<8) | rawADC[1]) ,
                    -((rawADC[2]<<8) | rawADC[3]) );
  #endif

  if (calibratingM == 1) {
    tCal = t;
    for(axis=0;axis<3;axis++) {magZero[axis] = 0;magZeroTempMin[axis] = 0; magZeroTempMax[axis] = 0;}
    calibratingM = 0;
  }
  magADC[ROLL]  -= magZero[ROLL];
  magADC[PITCH] -= magZero[PITCH];
  magADC[YAW]   -= magZero[YAW];

  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_SWITCH
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
      writeParams();
    }
  }
}
#endif


// **************************
// I2C Wii Motion Plus + optional Nunchuk
// **************************
// I2C adress 1: 0xA6 (8bit)    0x53 (7bit)
// I2C adress 2: 0xA4 (8bit)    0x52 (7bit)

void WMP_init(uint8_t d) {
  delay(d);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xF0); 
  i2c_write(0x55); 
  delay(d);
  i2c_rep_start(0xA6 + 0);//I2C write direction => 0
  i2c_write(0xFE); 
  i2c_write(0x05); 
  delay(d);
  if (d>0) {
    uint8_t numberAccRead = 0;
    for(uint8_t i=0;i<100;i++) {
      delay(3);
      if (WMP_getRawADC() == 0) numberAccRead++; // we detect here is nunchuk extension is available
    }
    if (numberAccRead>25) {
      nunchukPresent = 1;
      accPresent = 1;
    }
    delay(10);
  }
}

uint8_t WMP_getRawADC() {
  uint8_t axis;

  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  i2c_rep_start(0xA4 + 0);//I2C write direction => 0
  i2c_write(0x00);
  getSixRawADC(0xA4 + 1);

  if (currentTime < (neutralizeTime + NEUTRALIZE_DELAY)) {//we neutralize data in case of blocking+hard reset state
    for (axis = 0; axis < 3; axis++) {gyroADC[axis]=0;accADC[axis]=0;}
    accADC[YAW] = acc_1G;
    return 1;
  } 

  if ( (rawADC[5]&0x02) == 0x02 && (rawADC[5]&0x01) == 0 ) {// motion plus data
    gyroADC[ROLL]   = - ( ((rawADC[5]>>2)<<8) + rawADC[2] );
    gyroADC[PITCH]  = - ( ((rawADC[4]>>2)<<8) + rawADC[1] );
    gyroADC[YAW]    = - ( ((rawADC[3]>>2)<<8) + rawADC[0] );
    GYRO_Common();
    gyroADC[ROLL]  = (rawADC[3]&0x01)     ? gyroADC[ROLL]/5  : gyroADC[ROLL] ;   //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
    gyroADC[PITCH] = (rawADC[4]&0x02)>>1  ? gyroADC[PITCH]/5 : gyroADC[PITCH] ;  //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
    gyroADC[YAW]   = (rawADC[3]&0x02)>>1  ? gyroADC[YAW]/5   : gyroADC[YAW] ;    // this step must be done after zero compensation
    
    return 1;
  } else if ( (rawADC[5]&0x02) == 0 && (rawADC[5]&0x01) == 0) { //nunchuk data
    accADC[ROLL]  =   ( (rawADC[3]<<2)        + ((rawADC[5]>>4)&0x2) );
    accADC[PITCH] = - ( (rawADC[2]<<2)        + ((rawADC[5]>>3)&0x2) );
    accADC[YAW]   =   ( ((rawADC[4]&0xFE)<<2) + ((rawADC[5]>>5)&0x6) );
    
    ACC_Common();
    return 0;
  } else
    return 2;
}

void initSensors() {
  delay(200);
  POWERPIN_ON
  delay(100);
  i2c_init();
  delay(100);
  #if defined(GYRO)
    Gyro_init();
  #else
    WMP_init(250);
  #endif
  #if defined(BMP085)
    Baro_init();
  #endif
  #if defined(ACC) 
    ACC_init();
  #endif
  #if defined(MAG)
    Mag_init();
  #endif
}
